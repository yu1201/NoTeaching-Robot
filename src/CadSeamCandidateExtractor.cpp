#include "CadSeamCandidateExtractor.h"

#include "OpenCascadeOperationGuard.h"

#include <Bnd_Box.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepAlgoAPI_Section.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepGProp.hxx>
#include <BRepLProp_SLProps.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <BRep_Tool.hxx>
#include <GCPnts_UniformAbscissa.hxx>
#include <GProp_GProps.hxx>
#include <Geom2d_Curve.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <Precision.hxx>
#include <STEPControl_Reader.hxx>
#include <STEPCAFControl_Reader.hxx>
#include <ShapeAnalysis_FreeBounds.hxx>
#include <Standard_Failure.hxx>
#include <Standard_Version.hxx>
#include <TopAbs_Orientation.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_HSequenceOfShape.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopTools_ListIteratorOfListOfShape.hxx>
#include <TopTools_ListOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Wire.hxx>
#include <TDataStd_Name.hxx>
#include <TDF_Label.hxx>
#include <TDocStd_Document.hxx>
#include <XCAFApp_Application.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <XCAFPrs_DocumentExplorer.hxx>
#include <XCAFPrs_DocumentNode.hxx>
#include <gp_Dir.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Vec.hxx>

#if OCC_VERSION_MAJOR != 7 || OCC_VERSION_MINOR != 9 || OCC_VERSION_MAINTENANCE != 3
#error "CadSeamCandidateExtractor requires Open CASCADE Technology 7.9.3."
#endif

#include <QCryptographicHash>
#include <QDataStream>
#include <QFileInfo>
#include <QMutexLocker>
#include <QSet>

#include <algorithm>
#include <cmath>
#include <exception>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <utility>
#include <vector>

namespace
{
constexpr qint64 kHardMaximumFileBytes = 256LL * 1024LL * 1024LL;
constexpr int kHardMaximumRoots = 1024;
constexpr int kHardMaximumFaces = 100000;
constexpr int kHardMaximumSharedEdges = 1000000;
constexpr int kHardMaximumRootPairs = 65536;
constexpr int kHardMaximumCandidates = 20000;
constexpr int kHardMaximumPointsPerCandidate = 20000;
constexpr double kPi = 3.14159265358979323846;

using SourceKind = CadSeamCandidateExtractor::SourceKind;
using Candidate = CadSeamCandidateExtractor::Candidate;
using Options = CadSeamCandidateExtractor::Options;
using Statistics = CadSeamCandidateExtractor::Statistics;

struct EdgeEvidence
{
    TopoDS_Edge edge;
    double dihedralDegrees = 0.0;
    int firstFaceIndex = -1;
    int secondFaceIndex = -1;
};

struct AssemblyComponent
{
    TopoDS_Shape shape;
    QString instanceName;
    QString productName;
    int componentIndex = -1;
};

struct DocumentGuard
{
    Handle(TDocStd_Document) document;
    ~DocumentGuard()
    {
        if (!document.IsNull()) XCAFApp_Application::GetApplication()->Close(document);
    }
};

bool FinitePoint(const Eigen::Vector3d& point)
{
    return point.allFinite();
}

QString ReadFailureText(IFSelect_ReturnStatus status)
{
    switch (status)
    {
    case IFSelect_RetVoid:
        return QStringLiteral("STEP 文件中没有可读取的数据。");
    case IFSelect_RetError:
        return QStringLiteral("STEP 文件格式或输入参数错误。");
    case IFSelect_RetFail:
        return QStringLiteral("Open CASCADE 解析 STEP 文件失败。");
    case IFSelect_RetStop:
        return QStringLiteral("STEP 文件解析被底层读取器中止。");
    case IFSelect_RetDone:
        break;
    }
    return QStringLiteral("STEP 文件返回了未知读取状态。");
}

QString ExceptionDetail(const char* message)
{
    if (message == nullptr || *message == '\0') return QString();
    QString detail = QString::fromLocal8Bit(message).simplified();
    if (detail.size() > 400) detail = detail.left(400) + QStringLiteral("...");
    return detail;
}

QString LabelName(const TDF_Label& label)
{
    if (label.IsNull()) return QString();
    Handle(TDataStd_Name) attribute;
    if (!label.FindAttribute(TDataStd_Name::GetID(), attribute) || attribute.IsNull())
        return QString();
#ifdef _WIN32
    return QString::fromWCharArray(attribute->Get().ToWideString()).trimmed();
#else
    const TCollection_ExtendedString& value = attribute->Get();
    QByteArray utf8(value.LengthOfCString() + 1, '\0');
    Standard_PCharacter destination = utf8.data();
    value.ToUTF8CString(destination);
    return QString::fromUtf8(utf8.constData()).trimmed();
#endif
}

bool IsCorrugatedPanelName(const QString& name)
{
    return name.contains(QStringLiteral("波纹板"))
        || name.contains(QStringLiteral("corrugated"), Qt::CaseInsensitive);
}

bool IsSeamSimulationName(const QString& name)
{
    return name.contains(QStringLiteral("焊缝模拟"))
        || name.contains(QStringLiteral("weld seam"), Qt::CaseInsensitive)
        || name.contains(QStringLiteral("weld simulation"), Qt::CaseInsensitive);
}

QVector<AssemblyComponent> ReadAssemblyComponents(
    const QString& stepFilePath,
    QString& error)
{
#ifdef _WIN32
    const std::filesystem::path nativePath(stepFilePath.toStdWString());
#else
    const std::filesystem::path nativePath(stepFilePath.toUtf8().constData());
#endif
    std::ifstream input(nativePath, std::ios::in | std::ios::binary);
    if (!input.is_open() || input.fail())
    {
        error = QStringLiteral("无法为装配焊缝语义重新打开 STEP 文件。");
        return {};
    }
    STEPCAFControl_Reader reader;
    reader.SetNameMode(Standard_True);
    reader.SetColorMode(Standard_False);
    reader.SetLayerMode(Standard_False);
    reader.SetPropsMode(Standard_False);
    if (reader.ReadStream("assembly.step", input) != IFSelect_RetDone)
    {
        error = QStringLiteral("无法读取 STEP 装配产品名称。");
        return {};
    }
    input.close();
    reader.ChangeReader().SetSystemLengthUnit(1.0);
    DocumentGuard guard;
    XCAFApp_Application::GetApplication()->NewDocument(
        TCollection_ExtendedString("MDTV-XCAF"), guard.document);
    if (guard.document.IsNull() || !reader.Transfer(guard.document))
    {
        error = QStringLiteral("STEP 装配产品结构转换失败。");
        return {};
    }
    const Handle(XCAFDoc_ShapeTool) shapeTool =
        XCAFDoc_DocumentTool::ShapeTool(guard.document->Main());
    if (shapeTool.IsNull())
    {
        error = QStringLiteral("STEP 没有可用的 XCAF 装配结构。");
        return {};
    }
    QVector<AssemblyComponent> components;
    XCAFPrs_DocumentExplorer explorer(
        guard.document, XCAFPrs_DocumentExplorerFlags_NoStyle);
    for (; explorer.More(); explorer.Next())
    {
        if (explorer.CurrentDepth() != 1) continue;
        const XCAFPrs_DocumentNode& node = explorer.Current();
        const TopoDS_Shape shape = XCAFPrs_DocumentExplorer::FindShapeFromPathId(
            guard.document, node.Id);
        if (shape.IsNull()) continue;
        AssemblyComponent component;
        component.shape = shape;
        component.instanceName = LabelName(node.Label);
        component.productName = LabelName(
            node.RefLabel.IsNull() ? node.Label : node.RefLabel);
        component.componentIndex = components.size() + 1;
        components.push_back(std::move(component));
    }
    return components;
}

bool ValidateOptions(const Options& options, QString& error)
{
    if (options.maximumFileBytes <= 0 || options.maximumFileBytes > kHardMaximumFileBytes
        || options.maximumRoots <= 0 || options.maximumRoots > kHardMaximumRoots
        || options.maximumFaces <= 0 || options.maximumFaces > kHardMaximumFaces
        || options.maximumSharedEdges <= 0
        || options.maximumSharedEdges > kHardMaximumSharedEdges
        || options.maximumRootPairs < 0 || options.maximumRootPairs > kHardMaximumRootPairs
        || options.maximumCandidates <= 0 || options.maximumCandidates > kHardMaximumCandidates
        || options.maximumPointsPerCandidate < 2
        || options.maximumPointsPerCandidate > kHardMaximumPointsPerCandidate)
    {
        error = QStringLiteral("STEP 焊缝候选提取的数量或文件限制无效。");
        return false;
    }
    if (!std::isfinite(options.minimumLengthMm) || options.minimumLengthMm <= 0.0
        || !std::isfinite(options.samplingStepMm) || options.samplingStepMm <= 0.0
        || !std::isfinite(options.minimumDihedralDegrees)
        || !std::isfinite(options.maximumDihedralDegrees)
        || options.minimumDihedralDegrees < 0.0
        || options.maximumDihedralDegrees > 90.0
        || options.minimumDihedralDegrees > options.maximumDihedralDegrees
        || !std::isfinite(options.linearToleranceMm)
        || options.linearToleranceMm <= 0.0 || options.linearToleranceMm > 10.0)
    {
        error = QStringLiteral("STEP 焊缝候选提取的长度、采样、二面角或容差参数无效。");
        return false;
    }
    return true;
}

Eigen::Vector3d ToEigen(const gp_Pnt& point)
{
    return Eigen::Vector3d(point.X(), point.Y(), point.Z());
}

bool FaceNormalOnEdge(
    const TopoDS_Edge& edge,
    const TopoDS_Face& face,
    double edgeParameter,
    double tolerance,
    Eigen::Vector3d& normal)
{
    Standard_Real first = 0.0;
    Standard_Real last = 0.0;
    const Handle(Geom2d_Curve) curve2d = BRep_Tool::CurveOnSurface(
        edge, face, first, last);
    if (curve2d.IsNull() || !std::isfinite(first) || !std::isfinite(last)) return false;

    BRepAdaptor_Curve curve3d(edge);
    const double curveFirst = curve3d.FirstParameter();
    const double curveLast = curve3d.LastParameter();
    if (!std::isfinite(curveFirst) || !std::isfinite(curveLast)
        || curveLast <= curveFirst)
    {
        return false;
    }
    const double ratio = std::clamp(
        (edgeParameter - curveFirst) / (curveLast - curveFirst), 0.0, 1.0);
    const double curve2dParameter = first + ratio * (last - first);
    const gp_Pnt2d uv = curve2d->Value(curve2dParameter);
    BRepAdaptor_Surface surface(face, Standard_False);
    BRepLProp_SLProps properties(surface, uv.X(), uv.Y(), 1, tolerance);
    if (!properties.IsNormalDefined()) return false;
    const gp_Dir direction = properties.Normal();
    normal = Eigen::Vector3d(direction.X(), direction.Y(), direction.Z());
    if (face.Orientation() == TopAbs_REVERSED) normal = -normal;
    if (!FinitePoint(normal) || normal.norm() <= 1.0e-12) return false;
    normal.normalize();
    return true;
}

bool EdgeDihedralDegrees(
    const TopoDS_Edge& edge,
    const TopoDS_Face& firstFace,
    const TopoDS_Face& secondFace,
    double tolerance,
    double& degrees)
{
    BRepAdaptor_Curve curve(edge);
    const double first = curve.FirstParameter();
    const double last = curve.LastParameter();
    if (!std::isfinite(first) || !std::isfinite(last) || last <= first) return false;
    const double middle = first + 0.5 * (last - first);
    Eigen::Vector3d firstNormal;
    Eigen::Vector3d secondNormal;
    if (!FaceNormalOnEdge(edge, firstFace, middle, tolerance, firstNormal)
        || !FaceNormalOnEdge(edge, secondFace, middle, tolerance, secondNormal))
    {
        return false;
    }
    const double cosine = std::clamp(std::abs(firstNormal.dot(secondNormal)), 0.0, 1.0);
    degrees = std::acos(cosine) * 180.0 / kPi;
    return std::isfinite(degrees);
}

double EdgeLength(const TopoDS_Edge& edge)
{
    GProp_GProps properties;
    BRepGProp::LinearProperties(edge, properties);
    const double length = std::abs(properties.Mass());
    return std::isfinite(length) ? length : 0.0;
}

bool AppendEdgeSamples(
    const TopoDS_Edge& edge,
    double edgeLength,
    const Options& options,
    QVector<Eigen::Vector3d>& points)
{
    BRepAdaptor_Curve curve(edge);
    const double first = curve.FirstParameter();
    const double last = curve.LastParameter();
    if (!std::isfinite(first) || !std::isfinite(last) || last <= first) return false;

    const int requestedPoints = std::clamp(
        static_cast<int>(std::ceil(edgeLength / options.samplingStepMm)) + 1,
        2,
        options.maximumPointsPerCandidate);
    GCPnts_UniformAbscissa uniform(
        curve, requestedPoints, first, last, options.linearToleranceMm);
    const bool reversed = edge.Orientation() == TopAbs_REVERSED;
    const int count = uniform.IsDone() ? uniform.NbPoints() : requestedPoints;
    if (count < 2) return false;

    for (int pointIndex = 0; pointIndex < count; ++pointIndex)
    {
        const int orderedIndex = reversed ? count - pointIndex : pointIndex + 1;
        const double parameter = uniform.IsDone()
            ? uniform.Parameter(orderedIndex)
            : first + (last - first) * static_cast<double>(orderedIndex - 1)
                / static_cast<double>(count - 1);
        const Eigen::Vector3d point = ToEigen(curve.Value(parameter));
        if (!FinitePoint(point)) return false;
        if (!points.isEmpty()
            && (points.back() - point).norm() <= options.linearToleranceMm)
        {
            continue;
        }
        if (points.size() >= options.maximumPointsPerCandidate) return false;
        points.push_back(point);
    }
    return points.size() >= 2;
}

QByteArray CandidateDigest(const Candidate& candidate)
{
    QByteArray canonical;
    QDataStream stream(&canonical, QIODevice::WriteOnly);
    stream.setByteOrder(QDataStream::LittleEndian);
    stream << static_cast<qint64>(std::llround(candidate.lengthMm * 1000.0));
    stream << static_cast<qint32>(candidate.pointsModelMm.size());

    QVector<Eigen::Vector3d> ordered = candidate.pointsModelMm;
    const auto pointKey = [](const Eigen::Vector3d& point)
    {
        return std::array<qint64, 3>{
            std::llround(point.x() * 1000.0),
            std::llround(point.y() * 1000.0),
            std::llround(point.z() * 1000.0)
        };
    };
    if (ordered.size() >= 2 && pointKey(ordered.back()) < pointKey(ordered.front()))
        std::reverse(ordered.begin(), ordered.end());
    for (const Eigen::Vector3d& point : ordered)
    {
        const auto key = pointKey(point);
        stream << key[0] << key[1] << key[2];
    }
    return QCryptographicHash::hash(canonical, QCryptographicHash::Sha256).toHex();
}

QByteArray EdgeGeometryDigest(const TopoDS_Edge& edge, double tolerance)
{
    BRepAdaptor_Curve curve(edge);
    const double first = curve.FirstParameter();
    const double last = curve.LastParameter();
    if (!std::isfinite(first) || !std::isfinite(last) || last <= first)
        return QByteArray();
    const double quantum = std::max(tolerance, 1.0e-6);
    const auto pointKey = [quantum](const gp_Pnt& point)
    {
        return std::array<qint64, 3>{
            std::llround(point.X() / quantum),
            std::llround(point.Y() / quantum),
            std::llround(point.Z() / quantum)
        };
    };
    auto firstKey = pointKey(curve.Value(first));
    auto lastKey = pointKey(curve.Value(last));
    if (lastKey < firstKey) std::swap(firstKey, lastKey);
    const auto middleKey = pointKey(curve.Value(first + 0.5 * (last - first)));
    QByteArray canonical;
    QDataStream stream(&canonical, QIODevice::WriteOnly);
    stream.setByteOrder(QDataStream::LittleEndian);
    for (qint64 value : firstKey) stream << value;
    for (qint64 value : middleKey) stream << value;
    for (qint64 value : lastKey) stream << value;
    stream << static_cast<qint64>(std::llround(EdgeLength(edge) / quantum));
    return QCryptographicHash::hash(canonical, QCryptographicHash::Sha256).toHex();
}

bool AppendCandidateFromShape(
    const TopoDS_Shape& chainShape,
    SourceKind sourceKind,
    const Options& options,
    const TopTools_IndexedMapOfShape* evidenceEdges,
    const std::vector<EdgeEvidence>* evidence,
    int firstRootIndex,
    int secondRootIndex,
    QVector<Candidate>& result,
    QSet<QByteArray>& digests,
    Statistics& statistics)
{
    Candidate candidate;
    candidate.sourceKind = sourceKind;
    candidate.firstRootIndex = firstRootIndex;
    candidate.secondRootIndex = secondRootIndex;
    double weightedDihedral = 0.0;
    double weightedLength = 0.0;

    for (BRepTools_WireExplorer explorer(TopoDS::Wire(chainShape)); explorer.More(); explorer.Next())
    {
        const TopoDS_Edge edge = explorer.Current();
        const double length = EdgeLength(edge);
        if (length <= options.linearToleranceMm) continue;
        if (!AppendEdgeSamples(edge, length, options, candidate.pointsModelMm)) return false;
        candidate.lengthMm += length;

        if (evidenceEdges != nullptr && evidence != nullptr)
        {
            const int edgeIndex = evidenceEdges->FindIndex(edge);
            if (edgeIndex > 0 && edgeIndex <= static_cast<int>(evidence->size()))
            {
                const EdgeEvidence& item = evidence->at(static_cast<size_t>(edgeIndex - 1));
                if (candidate.firstFaceIndex < 0)
                {
                    candidate.firstFaceIndex = item.firstFaceIndex;
                    candidate.secondFaceIndex = item.secondFaceIndex;
                }
                weightedDihedral += item.dihedralDegrees * length;
                weightedLength += length;
            }
        }
    }
    if (candidate.lengthMm < options.minimumLengthMm
        || candidate.pointsModelMm.size() < 2)
    {
        ++statistics.rejectedShortCount;
        return true;
    }
    candidate.dihedralDegrees = weightedLength > 0.0
        ? weightedDihedral / weightedLength : 0.0;
    const QByteArray digest = CandidateDigest(candidate);
    if (digests.contains(digest))
    {
        ++statistics.duplicateCount;
        return true;
    }
    if (result.size() >= options.maximumCandidates) return false;
    digests.insert(digest);
    candidate.candidateId = QStringLiteral("CS-%1").arg(
        QString::fromLatin1(digest.left(16)).toUpper());
    result.push_back(candidate);
    return true;
}

bool AppendConnectedEdges(
    const TopTools_HSequenceOfShape& inputEdges,
    SourceKind sourceKind,
    const Options& options,
    const TopTools_IndexedMapOfShape* evidenceEdges,
    const std::vector<EdgeEvidence>* evidence,
    int firstRootIndex,
    int secondRootIndex,
    QVector<Candidate>& result,
    QSet<QByteArray>& digests,
    Statistics& statistics)
{
    if (inputEdges.Length() <= 0) return true;
    Handle(TopTools_HSequenceOfShape) edges = new TopTools_HSequenceOfShape(inputEdges);
    Handle(TopTools_HSequenceOfShape) wires;
    ShapeAnalysis_FreeBounds::ConnectEdgesToWires(
        edges, options.linearToleranceMm, Standard_False, wires);
    if (wires.IsNull()) return true;
    for (Standard_Integer wireIndex = 1; wireIndex <= wires->Length(); ++wireIndex)
    {
        const TopoDS_Shape& shape = wires->Value(wireIndex);
        if (shape.IsNull() || shape.ShapeType() != TopAbs_WIRE) continue;
        if (!AppendCandidateFromShape(
                shape, sourceKind, options, evidenceEdges, evidence,
                firstRootIndex, secondRootIndex, result, digests, statistics))
        {
            return false;
        }
    }
    return true;
}

bool ShapeBounds(
    const TopoDS_Shape& shape,
    Eigen::Vector3d& minimum,
    Eigen::Vector3d& maximum)
{
    Bnd_Box box;
    BRepBndLib::Add(shape, box, Standard_False);
    if (box.IsVoid() || box.IsOpen()) return false;
    Standard_Real xMinimum = 0.0, yMinimum = 0.0, zMinimum = 0.0;
    Standard_Real xMaximum = 0.0, yMaximum = 0.0, zMaximum = 0.0;
    box.Get(xMinimum, yMinimum, zMinimum, xMaximum, yMaximum, zMaximum);
    minimum = Eigen::Vector3d(xMinimum, yMinimum, zMinimum);
    maximum = Eigen::Vector3d(xMaximum, yMaximum, zMaximum);
    return minimum.allFinite() && maximum.allFinite()
        && (maximum.array() >= minimum.array()).all();
}

bool AppendSemanticCandidate(
    SourceKind sourceKind,
    QVector<Eigen::Vector3d> points,
    int firstComponentIndex,
    int secondComponentIndex,
    const Options& options,
    QVector<Candidate>& result,
    QSet<QByteArray>& digests,
    Statistics& statistics)
{
    Candidate candidate;
    candidate.sourceKind = sourceKind;
    candidate.firstRootIndex = firstComponentIndex;
    candidate.secondRootIndex = secondComponentIndex;
    candidate.pointsModelMm = std::move(points);
    for (int index = 1; index < candidate.pointsModelMm.size(); ++index)
        candidate.lengthMm += (candidate.pointsModelMm.at(index)
            - candidate.pointsModelMm.at(index - 1)).norm();
    if (candidate.lengthMm < options.minimumLengthMm
        || candidate.pointsModelMm.size() < 2)
    {
        ++statistics.rejectedShortCount;
        return true;
    }
    const QByteArray digest = CandidateDigest(candidate);
    if (digests.contains(digest))
    {
        ++statistics.duplicateCount;
        return true;
    }
    if (result.size() >= options.maximumCandidates) return false;
    digests.insert(digest);
    candidate.candidateId = QStringLiteral("CS-%1").arg(
        QString::fromLatin1(digest.left(16)).toUpper());
    result.push_back(std::move(candidate));
    return true;
}

bool AppendCorrugatedButtJoints(
    const QVector<AssemblyComponent>& panels,
    const Options& options,
    QVector<Candidate>& result,
    QSet<QByteArray>& digests,
    Statistics& statistics)
{
    struct ComponentBounds
    {
        const AssemblyComponent* component = nullptr;
        Eigen::Vector3d minimum = Eigen::Vector3d::Zero();
        Eigen::Vector3d maximum = Eigen::Vector3d::Zero();
    };
    QVector<ComponentBounds> bounded;
    for (const AssemblyComponent& panel : panels)
    {
        ComponentBounds item;
        item.component = &panel;
        if (ShapeBounds(panel.shape, item.minimum, item.maximum)) bounded.push_back(item);
    }
    const double contactTolerance = std::max(options.linearToleranceMm * 4.0, 0.2);
    for (int firstIndex = 0; firstIndex < bounded.size(); ++firstIndex)
    {
        for (int secondIndex = firstIndex + 1; secondIndex < bounded.size(); ++secondIndex)
        {
            const ComponentBounds& first = bounded.at(firstIndex);
            const ComponentBounds& second = bounded.at(secondIndex);
            ++statistics.rootPairCount;
            int contactAxis = -1;
            double contactCoordinate = 0.0;
            double bestGap = std::numeric_limits<double>::infinity();
            for (int axis = 0; axis < 3; ++axis)
            {
                const double forwardGap = std::abs(first.maximum[axis] - second.minimum[axis]);
                const double reverseGap = std::abs(second.maximum[axis] - first.minimum[axis]);
                const double gap = std::min(forwardGap, reverseGap);
                if (gap <= contactTolerance && gap < bestGap)
                {
                    bestGap = gap;
                    contactAxis = axis;
                    contactCoordinate = forwardGap <= reverseGap
                        ? 0.5 * (first.maximum[axis] + second.minimum[axis])
                        : 0.5 * (second.maximum[axis] + first.minimum[axis]);
                }
            }
            if (contactAxis < 0) continue;
            std::array<int, 2> otherAxes{};
            int otherCount = 0;
            for (int axis = 0; axis < 3; ++axis)
                if (axis != contactAxis) otherAxes[static_cast<size_t>(otherCount++)] = axis;
            std::array<double, 2> overlapMinimum{};
            std::array<double, 2> overlapMaximum{};
            std::array<double, 2> overlapSpan{};
            bool validOverlap = true;
            for (int index = 0; index < 2; ++index)
            {
                const int axis = otherAxes[static_cast<size_t>(index)];
                overlapMinimum[static_cast<size_t>(index)] =
                    std::max(first.minimum[axis], second.minimum[axis]);
                overlapMaximum[static_cast<size_t>(index)] =
                    std::min(first.maximum[axis], second.maximum[axis]);
                overlapSpan[static_cast<size_t>(index)] =
                    overlapMaximum[static_cast<size_t>(index)]
                    - overlapMinimum[static_cast<size_t>(index)];
                validOverlap = validOverlap
                    && overlapSpan[static_cast<size_t>(index)] > contactTolerance;
            }
            if (!validOverlap) continue;
            const int seamOtherIndex = overlapSpan[0] >= overlapSpan[1] ? 0 : 1;
            const int thicknessOtherIndex = 1 - seamOtherIndex;
            const int seamAxis = otherAxes[static_cast<size_t>(seamOtherIndex)];
            const int thicknessAxis = otherAxes[static_cast<size_t>(thicknessOtherIndex)];
            const double seamMinimum = overlapMinimum[static_cast<size_t>(seamOtherIndex)];
            const double seamMaximum = overlapMaximum[static_cast<size_t>(seamOtherIndex)];
            if (seamMaximum - seamMinimum < options.minimumLengthMm) continue;
            Eigen::Vector3d start = Eigen::Vector3d::Zero();
            Eigen::Vector3d end = Eigen::Vector3d::Zero();
            start[contactAxis] = end[contactAxis] = contactCoordinate;
            start[thicknessAxis] = end[thicknessAxis] = 0.5 * (
                overlapMinimum[static_cast<size_t>(thicknessOtherIndex)]
                + overlapMaximum[static_cast<size_t>(thicknessOtherIndex)]);
            start[seamAxis] = seamMinimum;
            end[seamAxis] = seamMaximum;
            const int pointCount = std::clamp(
                static_cast<int>(std::ceil((seamMaximum - seamMinimum)
                    / options.samplingStepMm)) + 1,
                2, options.maximumPointsPerCandidate);
            QVector<Eigen::Vector3d> points;
            points.reserve(pointCount);
            for (int pointIndex = 0; pointIndex < pointCount; ++pointIndex)
            {
                const double ratio = static_cast<double>(pointIndex)
                    / static_cast<double>(pointCount - 1);
                points.push_back(start + ratio * (end - start));
            }
            if (!AppendSemanticCandidate(
                    SourceKind::CorrugatedButtJoint, std::move(points),
                    first.component->componentIndex, second.component->componentIndex,
                    options, result, digests, statistics))
            {
                return false;
            }
            ++statistics.corrugatedButtJointCount;
            ++statistics.intersectedRootPairCount;
        }
    }
    return true;
}

double PolylineLength(const QVector<Eigen::Vector3d>& points)
{
    double length = 0.0;
    for (int index = 1; index < points.size(); ++index)
        length += (points.at(index) - points.at(index - 1)).norm();
    return length;
}

QVector<Eigen::Vector3d> ResamplePolyline(
    const QVector<Eigen::Vector3d>& points,
    int count)
{
    QVector<Eigen::Vector3d> result;
    if (points.size() < 2 || count < 2) return result;
    QVector<double> cumulative(points.size(), 0.0);
    for (int index = 1; index < points.size(); ++index)
        cumulative[index] = cumulative[index - 1]
            + (points.at(index) - points.at(index - 1)).norm();
    const double total = cumulative.back();
    if (!std::isfinite(total) || total <= 1.0e-9) return result;
    result.reserve(count);
    int segment = 1;
    for (int index = 0; index < count; ++index)
    {
        const double target = total * static_cast<double>(index)
            / static_cast<double>(count - 1);
        while (segment < cumulative.size() - 1 && cumulative.at(segment) < target)
            ++segment;
        const double startLength = cumulative.at(segment - 1);
        const double endLength = cumulative.at(segment);
        const double ratio = endLength > startLength
            ? (target - startLength) / (endLength - startLength) : 0.0;
        result.push_back(points.at(segment - 1)
            + ratio * (points.at(segment) - points.at(segment - 1)));
    }
    return result;
}

QVector<Eigen::Vector3d> CyclicArc(
    const QVector<Eigen::Vector3d>& loop,
    int start,
    int end,
    int direction)
{
    QVector<Eigen::Vector3d> result;
    if (loop.isEmpty() || start < 0 || end < 0) return result;
    int index = start;
    result.push_back(loop.at(index));
    while (index != end && result.size() <= loop.size() + 1)
    {
        index = (index + direction + loop.size()) % loop.size();
        result.push_back(loop.at(index));
    }
    return result;
}

bool CorrugatedEndCenterline(
    const TopoDS_Shape& panel,
    const Options& options,
    QVector<Eigen::Vector3d>& centerline)
{
    centerline.clear();
    Eigen::Vector3d minimum;
    Eigen::Vector3d maximum;
    if (!ShapeBounds(panel, minimum, maximum)) return false;
    const Eigen::Vector3d span = maximum - minimum;
    Eigen::Index seamAxisIndex = 0;
    span.maxCoeff(&seamAxisIndex);
    const int endAxis = static_cast<int>(seamAxisIndex);
    if (endAxis != 1)
    {
        // 当前波纹板产品约定：最长挤出方向是模型 Y；未知轴向不猜测。
        return false;
    }
    const double inset = std::min(
        std::max(options.linearToleranceMm * 4.0, 0.2), span.y() * 0.01);
    const double sectionY = minimum.y() + inset;
    const TopoDS_Face sectionFace = BRepBuilderAPI_MakeFace(
        gp_Pln(gp_Pnt(0.0, sectionY, 0.0), gp_Dir(0.0, 1.0, 0.0))).Face();
    BRepAlgoAPI_Section section(panel, sectionFace, Standard_False);
    section.SetNonDestructive(Standard_True);
    section.SetFuzzyValue(options.linearToleranceMm);
    section.Approximation(Standard_True);
    section.Build();
    if (!section.IsDone() || section.HasErrors()) return false;
    Handle(TopTools_HSequenceOfShape) edges = new TopTools_HSequenceOfShape();
    QSet<QByteArray> edgeDigests;
    for (TopExp_Explorer explorer(section.Shape(), TopAbs_EDGE);
         explorer.More(); explorer.Next())
    {
        const TopoDS_Edge edge = TopoDS::Edge(explorer.Current());
        if (EdgeLength(edge) <= options.linearToleranceMm) continue;
        const QByteArray digest = EdgeGeometryDigest(edge, options.linearToleranceMm);
        if (!digest.isEmpty() && edgeDigests.contains(digest)) continue;
        if (!digest.isEmpty()) edgeDigests.insert(digest);
        edges->Append(edge);
    }
    if (edges->IsEmpty()) return false;
    Handle(TopTools_HSequenceOfShape) wires;
    ShapeAnalysis_FreeBounds::ConnectEdgesToWires(
        edges, options.linearToleranceMm, Standard_False, wires);
    if (wires.IsNull()) return false;
    QVector<Eigen::Vector3d> loop;
    double longest = 0.0;
    Options samplingOptions = options;
    samplingOptions.maximumPointsPerCandidate = kHardMaximumPointsPerCandidate;
    for (Standard_Integer wireIndex = 1; wireIndex <= wires->Length(); ++wireIndex)
    {
        const TopoDS_Shape& wireShape = wires->Value(wireIndex);
        if (wireShape.ShapeType() != TopAbs_WIRE) continue;
        QVector<Eigen::Vector3d> sampled;
        double wireLength = 0.0;
        bool valid = true;
        for (BRepTools_WireExplorer explorer(TopoDS::Wire(wireShape));
             explorer.More(); explorer.Next())
        {
            const TopoDS_Edge edge = explorer.Current();
            const double length = EdgeLength(edge);
            if (!AppendEdgeSamples(edge, length, samplingOptions, sampled))
            {
                valid = false;
                break;
            }
            wireLength += length;
        }
        if (valid && sampled.size() >= 4 && wireLength > longest)
        {
            longest = wireLength;
            loop = std::move(sampled);
        }
    }
    if (loop.size() < 4) return false;
    if ((loop.front() - loop.back()).norm() <= options.linearToleranceMm * 4.0)
        loop.removeLast();
    if (loop.size() < 4) return false;
    Eigen::Vector3d loopMinimum = loop.front();
    Eigen::Vector3d loopMaximum = loop.front();
    for (const Eigen::Vector3d& point : loop)
    {
        loopMinimum = loopMinimum.cwiseMin(point);
        loopMaximum = loopMaximum.cwiseMax(point);
    }
    const Eigen::Vector3d loopSpan = loopMaximum - loopMinimum;
    const int primaryAxis = loopSpan.x() >= loopSpan.z() ? 0 : 2;
    int minimumIndex = 0;
    int maximumIndex = 0;
    for (int index = 1; index < loop.size(); ++index)
    {
        if (loop.at(index)[primaryAxis] < loop.at(minimumIndex)[primaryAxis])
            minimumIndex = index;
        if (loop.at(index)[primaryAxis] > loop.at(maximumIndex)[primaryAxis])
            maximumIndex = index;
    }
    QVector<Eigen::Vector3d> firstArc = CyclicArc(loop, minimumIndex, maximumIndex, 1);
    QVector<Eigen::Vector3d> secondArc = CyclicArc(loop, minimumIndex, maximumIndex, -1);
    const double firstLength = PolylineLength(firstArc);
    const double secondLength = PolylineLength(secondArc);
    if (firstLength <= options.linearToleranceMm
        || secondLength <= options.linearToleranceMm)
    {
        return false;
    }
    const int pointCount = std::clamp(
        static_cast<int>(std::ceil(0.5 * (firstLength + secondLength)
            / options.samplingStepMm)) + 1,
        2, options.maximumPointsPerCandidate);
    firstArc = ResamplePolyline(firstArc, pointCount);
    secondArc = ResamplePolyline(secondArc, pointCount);
    if (firstArc.size() != pointCount || secondArc.size() != pointCount) return false;
    centerline.reserve(pointCount);
    for (int index = 0; index < pointCount; ++index)
    {
        Eigen::Vector3d point = 0.5 * (firstArc.at(index) + secondArc.at(index));
        point.y() = minimum.y();
        centerline.push_back(point);
    }
    if (centerline.back()[primaryAxis] < centerline.front()[primaryAxis])
        std::reverse(centerline.begin(), centerline.end());
    return PolylineLength(centerline) >= options.minimumLengthMm;
}

bool AppendCorrugatedBaseJoints(
    const QVector<AssemblyComponent>& panels,
    const Options& options,
    QVector<Candidate>& result,
    QSet<QByteArray>& digests,
    Statistics& statistics)
{
    struct PanelLine
    {
        int componentIndex = -1;
        QVector<Eigen::Vector3d> points;
    };
    QVector<PanelLine> lines;
    for (const AssemblyComponent& panel : panels)
    {
        PanelLine line;
        line.componentIndex = panel.componentIndex;
        if (CorrugatedEndCenterline(panel.shape, options, line.points))
            lines.push_back(std::move(line));
    }
    if (lines.isEmpty()) return true;
    const auto centreZ = [](const PanelLine& line)
    {
        double minimum = line.points.front().z();
        double maximum = minimum;
        for (const Eigen::Vector3d& point : line.points)
        {
            minimum = std::min(minimum, point.z());
            maximum = std::max(maximum, point.z());
        }
        return 0.5 * (minimum + maximum);
    };
    std::sort(lines.begin(), lines.end(), [&centreZ](const PanelLine& left, const PanelLine& right)
    {
        return centreZ(left) < centreZ(right);
    });
    double largestGap = 0.0;
    int splitIndex = -1;
    for (int index = 1; index < lines.size(); ++index)
    {
        const double gap = centreZ(lines.at(index)) - centreZ(lines.at(index - 1));
        if (gap > largestGap)
        {
            largestGap = gap;
            splitIndex = index;
        }
    }
    QVector<QVector<PanelLine>> groups;
    if (splitIndex > 0 && largestGap > 100.0)
    {
        groups.push_back(lines.mid(0, splitIndex));
        groups.push_back(lines.mid(splitIndex));
    }
    else
    {
        groups.push_back(lines);
    }

    const auto xRange = [](const QVector<Eigen::Vector3d>& points)
    {
        double minimum = points.front().x();
        double maximum = minimum;
        for (const Eigen::Vector3d& point : points)
        {
            minimum = std::min(minimum, point.x());
            maximum = std::max(maximum, point.x());
        }
        return std::pair<double, double>(minimum, maximum);
    };
    const auto closestPair = [](const QVector<Eigen::Vector3d>& first,
                                const QVector<Eigen::Vector3d>& second)
    {
        std::pair<int, int> best(0, 0);
        double bestDistance = std::numeric_limits<double>::infinity();
        int secondCursor = 0;
        for (int firstIndex = 0; firstIndex < first.size(); ++firstIndex)
        {
            while (secondCursor + 1 < second.size()
                && second.at(secondCursor + 1).x() < first.at(firstIndex).x())
            {
                ++secondCursor;
            }
            for (int offset = -2; offset <= 2; ++offset)
            {
                const int secondIndex = std::clamp(
                    secondCursor + offset, 0, static_cast<int>(second.size()) - 1);
                const double distance =
                    (first.at(firstIndex) - second.at(secondIndex)).norm();
                if (distance < bestDistance)
                {
                    bestDistance = distance;
                    best = { firstIndex, secondIndex };
                }
            }
        }
        return best;
    };

    for (QVector<PanelLine>& group : groups)
    {
        if (group.isEmpty()) continue;
        std::sort(group.begin(), group.end(), [&xRange](const PanelLine& left, const PanelLine& right)
        {
            return xRange(left.points).first < xRange(right.points).first;
        });
        QVector<Eigen::Vector3d> joined = group.front().points;
        for (int lineIndex = 1; lineIndex < group.size(); ++lineIndex)
        {
            QVector<Eigen::Vector3d> extension = group.at(lineIndex).points;
            if (extension.front().x() > extension.back().x())
                std::reverse(extension.begin(), extension.end());
            if (joined.front().x() > joined.back().x())
                std::reverse(joined.begin(), joined.end());
            const auto joinedRange = xRange(joined);
            const auto extensionRange = xRange(extension);
            if (extensionRange.second <= joinedRange.second + options.linearToleranceMm)
                continue;
            const std::pair<int, int> splice = closestPair(joined, extension);
            joined = joined.mid(0, splice.first + 1)
                + extension.mid(splice.second + 1);
        }
        if (joined.size() < 2) continue;
        if (!AppendSemanticCandidate(
                SourceKind::CorrugatedBaseJoint, std::move(joined),
                group.front().componentIndex, -1,
                options, result, digests, statistics))
        {
            return false;
        }
        ++statistics.corrugatedBaseJointCount;
    }
    return true;
}

TopTools_IndexedMapOfShape SolidOrShellComponents(const TopoDS_Shape& shape)
{
    TopTools_IndexedMapOfShape components;
    TopExp::MapShapes(shape, TopAbs_SOLID, components);
    if (components.IsEmpty()) TopExp::MapShapes(shape, TopAbs_SHELL, components);
    return components;
}
}

bool CadSeamCandidateExtractor::ExtractFromStepFile(
    const QString& stepFilePath,
    QVector<Candidate>& candidates,
    QString& error,
    Statistics* statistics,
    const Options* tighterLimits)
{
    QMutexLocker<QMutex> operationLock(&occtsync::OperationMutex());
    candidates.clear();
    error.clear();
    if (statistics != nullptr) *statistics = Statistics();
    Statistics collected;

    const auto fail = [&candidates, &error](const QString& message)
    {
        candidates.clear();
        error = message;
        return false;
    };

    try
    {
        const Options options = tighterLimits != nullptr ? *tighterLimits : Options();
        if (!ValidateOptions(options, error)) return false;
        if (stepFilePath.isEmpty() || stepFilePath.contains(QChar::Null))
            return fail(QStringLiteral("STEP 文件路径为空或包含非法字符。"));
        const QFileInfo originalInfo(stepFilePath);
        if (!originalInfo.exists() || !originalInfo.isFile() || originalInfo.isSymLink())
            return fail(QStringLiteral("STEP 路径不是可读取的普通文件：%1").arg(stepFilePath));
        const QString suffix = originalInfo.suffix().toLower();
        if (suffix != QStringLiteral("step") && suffix != QStringLiteral("stp"))
            return fail(QStringLiteral("文件扩展名不是 .step 或 .stp。"));
        const qint64 originalSize = originalInfo.size();
        const qint64 originalModifiedMs = originalInfo.lastModified().toMSecsSinceEpoch();
        if (originalSize <= 0 || originalSize > options.maximumFileBytes)
            return fail(QStringLiteral("STEP 文件为空或超过候选提取大小上限。"));

#ifdef _WIN32
        const std::filesystem::path nativePath(stepFilePath.toStdWString());
#else
        const std::filesystem::path nativePath(stepFilePath.toUtf8().constData());
#endif
        std::ifstream input(nativePath, std::ios::in | std::ios::binary);
        if (!input.is_open() || input.fail())
            return fail(QStringLiteral("无法打开 STEP 文件：%1").arg(stepFilePath));
        STEPControl_Reader reader;
        const IFSelect_ReturnStatus readStatus = reader.ReadStream("input.step", input);
        input.close();
        if (readStatus != IFSelect_RetDone) return fail(ReadFailureText(readStatus));
        reader.SetSystemLengthUnit(1.0);
        const Standard_Integer rootCount = reader.NbRootsForTransfer();
        if (rootCount <= 0 || rootCount > options.maximumRoots)
            return fail(QStringLiteral("STEP 产品根节点为空或超过上限：%1").arg(rootCount));
        const Standard_Integer transferred = reader.TransferRoots();
        if (transferred != rootCount)
            return fail(QStringLiteral("STEP 根节点仅成功转换 %1/%2 个，拒绝不完整模型。")
                .arg(transferred).arg(rootCount));
        const TopoDS_Shape shape = reader.OneShape();
        if (shape.IsNull()) return fail(QStringLiteral("STEP 未生成可用 B-Rep。"));

        TopTools_IndexedMapOfShape faces;
        TopExp::MapShapes(shape, TopAbs_FACE, faces);
        collected.faceCount = faces.Extent();
        if (collected.faceCount <= 0 || collected.faceCount > options.maximumFaces)
            return fail(QStringLiteral("STEP B-Rep 面数量为空或超过上限：%1")
                .arg(collected.faceCount));

        QSet<QByteArray> digests;
        bool semanticCandidatesExtracted = false;
        if (options.preferAssemblySeamSemantics)
        {
            QString assemblyError;
            const QVector<AssemblyComponent> components = ReadAssemblyComponents(
                stepFilePath, assemblyError);
            QVector<AssemblyComponent> corrugatedPanels;
            QVector<AssemblyComponent> seamSimulationParts;
            for (const AssemblyComponent& component : components)
            {
                const QString combinedName = component.productName
                    + QLatin1Char(' ') + component.instanceName;
                if (IsCorrugatedPanelName(combinedName))
                    corrugatedPanels.push_back(component);
                else if (IsSeamSimulationName(combinedName))
                    seamSimulationParts.push_back(component);
            }
            collected.corrugatedComponentCount = corrugatedPanels.size();
            if (!corrugatedPanels.isEmpty() && !seamSimulationParts.isEmpty())
            {
                collected.assemblySemanticsUsed = true;
                collected.rootCount = components.size();
                if (!AppendCorrugatedButtJoints(
                        corrugatedPanels, options,
                        candidates, digests, collected)
                    || !AppendCorrugatedBaseJoints(
                        [&corrugatedPanels, &seamSimulationParts]()
                        {
                            QVector<AssemblyComponent> allPanels = corrugatedPanels;
                            allPanels += seamSimulationParts;
                            return allPanels;
                        }(), options,
                        candidates, digests, collected))
                {
                    return fail(QStringLiteral(
                        "STEP 装配语义焊缝候选数量或采样点数超过上限。"));
                }
                semanticCandidatesExtracted = !candidates.isEmpty();
            }
            else if (corrugatedPanels.size() >= 2)
            {
                // 没有专用端件命名时只以相邻波纹板接触面作为保守语义候选；
                // 底部焊缝仍由通用实体截交回退，不凭包围盒猜框架归属。
                collected.assemblySemanticsUsed = true;
                collected.rootCount = components.size();
                if (!AppendCorrugatedButtJoints(
                        corrugatedPanels, options,
                        candidates, digests, collected))
                {
                    return fail(QStringLiteral(
                        "STEP 波纹板对接候选数量或采样点数超过上限。"));
                }
                semanticCandidatesExtracted = !candidates.isEmpty();
            }
        }
        if (!semanticCandidatesExtracted && options.includeSharedEdges)
        {
            TopTools_IndexedDataMapOfShapeListOfShape edgeFaces;
            TopExp::MapShapesAndUniqueAncestors(
                shape, TopAbs_EDGE, TopAbs_FACE, edgeFaces, Standard_False);
            if (edgeFaces.Extent() > options.maximumSharedEdges)
                return fail(QStringLiteral("STEP 拓扑边数量超过候选提取上限：%1")
                    .arg(edgeFaces.Extent()));

            std::map<std::pair<int, int>, Handle(TopTools_HSequenceOfShape)> groupedEdges;
            TopTools_IndexedMapOfShape evidenceEdges;
            std::vector<EdgeEvidence> evidence;
            for (Standard_Integer edgeIndex = 1; edgeIndex <= edgeFaces.Extent(); ++edgeIndex)
            {
                const TopTools_ListOfShape& ancestors = edgeFaces.FindFromIndex(edgeIndex);
                if (ancestors.Extent() != 2) continue;
                TopTools_ListIteratorOfListOfShape iterator(ancestors);
                const TopoDS_Face firstFace = TopoDS::Face(iterator.Value());
                iterator.Next();
                const TopoDS_Face secondFace = TopoDS::Face(iterator.Value());
                const TopoDS_Edge edge = TopoDS::Edge(edgeFaces.FindKey(edgeIndex));
                double angle = 0.0;
                if (!EdgeDihedralDegrees(
                        edge, firstFace, secondFace, options.linearToleranceMm, angle))
                {
                    continue;
                }
                if (angle + 1.0e-9 < options.minimumDihedralDegrees
                    || angle - 1.0e-9 > options.maximumDihedralDegrees)
                {
                    ++collected.rejectedTangentCount;
                    continue;
                }
                const double length = EdgeLength(edge);
                if (length < options.minimumLengthMm)
                {
                    ++collected.rejectedShortCount;
                    continue;
                }
                int firstFaceIndex = faces.FindIndex(firstFace);
                int secondFaceIndex = faces.FindIndex(secondFace);
                if (firstFaceIndex > secondFaceIndex) std::swap(firstFaceIndex, secondFaceIndex);
                const std::pair<int, int> facePair(firstFaceIndex, secondFaceIndex);
                Handle(TopTools_HSequenceOfShape)& sequence = groupedEdges[facePair];
                if (sequence.IsNull()) sequence = new TopTools_HSequenceOfShape();
                sequence->Append(edge);
                evidenceEdges.Add(edge);
                evidence.push_back({ edge, angle, firstFaceIndex, secondFaceIndex });
                ++collected.sharedEdgeCount;
            }
            for (const auto& group : groupedEdges)
            {
                if (!AppendConnectedEdges(
                        *group.second, SourceKind::SharedEdge, options,
                        &evidenceEdges, &evidence, -1, -1,
                        candidates, digests, collected))
                {
                    return fail(QStringLiteral("STEP 共享边候选数量或采样点数超过上限。"));
                }
            }
        }

        if (!semanticCandidatesExtracted && options.includeRootIntersections)
        {
            const TopTools_IndexedMapOfShape components = SolidOrShellComponents(shape);
            collected.rootCount = components.Extent();
            if (collected.rootCount > options.maximumRoots)
                return fail(QStringLiteral("STEP 实体/壳体数量超过候选提取上限：%1")
                    .arg(collected.rootCount));
            const qint64 possiblePairs = static_cast<qint64>(collected.rootCount)
                * static_cast<qint64>(collected.rootCount - 1) / 2;
            if (possiblePairs > options.maximumRootPairs)
                return fail(QStringLiteral("STEP 实体两两截交组合超过上限：%1")
                    .arg(possiblePairs));

            QVector<Bnd_Box> bounds;
            bounds.reserve(collected.rootCount);
            for (int componentIndex = 1; componentIndex <= collected.rootCount; ++componentIndex)
            {
                Bnd_Box box;
                BRepBndLib::Add(components(componentIndex), box, Standard_False);
                box.Enlarge(options.linearToleranceMm);
                bounds.push_back(box);
            }
            for (int firstIndex = 1; firstIndex <= collected.rootCount; ++firstIndex)
            {
                for (int secondIndex = firstIndex + 1;
                     secondIndex <= collected.rootCount; ++secondIndex)
                {
                    ++collected.rootPairCount;
                    if (bounds.at(firstIndex - 1).IsOut(bounds.at(secondIndex - 1))) continue;
                    TopTools_ListOfShape arguments;
                    TopTools_ListOfShape tools;
                    arguments.Append(components(firstIndex));
                    tools.Append(components(secondIndex));
                    BRepAlgoAPI_Section section;
                    section.SetArguments(arguments);
                    section.SetTools(tools);
                    section.SetNonDestructive(Standard_True);
                    section.SetFuzzyValue(options.linearToleranceMm);
                    section.Approximation(Standard_True);
                    section.Build();
                    if (!section.IsDone() || section.HasErrors())
                        return fail(QStringLiteral("STEP 第 %1/%2 个实体截交计算失败。")
                            .arg(firstIndex).arg(secondIndex));

                    Handle(TopTools_HSequenceOfShape) sectionEdges =
                        new TopTools_HSequenceOfShape();
                    QSet<QByteArray> sectionEdgeDigests;
                    for (TopExp_Explorer edgeExplorer(
                             section.Shape(), TopAbs_EDGE);
                         edgeExplorer.More(); edgeExplorer.Next())
                    {
                        const TopoDS_Edge edge = TopoDS::Edge(edgeExplorer.Current());
                        // 一条真实波纹焊缝通常由许多短 B-Rep 边首尾相接组成。
                        // 必须先保留有效小段并拼线，再由 AppendCandidateFromShape
                        // 按整条候选总长度应用 minimumLengthMm；逐边过滤会把整条
                        // 波纹焊缝误删，只留下板件折弯形成的长直棱。
                        if (EdgeLength(edge) > options.linearToleranceMm)
                        {
                            const QByteArray edgeDigest = EdgeGeometryDigest(
                                edge, options.linearToleranceMm);
                            if (!edgeDigest.isEmpty()
                                && sectionEdgeDigests.contains(edgeDigest))
                            {
                                ++collected.duplicateCount;
                                continue;
                            }
                            if (!edgeDigest.isEmpty()) sectionEdgeDigests.insert(edgeDigest);
                            sectionEdges->Append(edge);
                        }
                        else
                            ++collected.rejectedShortCount;
                    }
                    if (sectionEdges->IsEmpty()) continue;
                    ++collected.intersectedRootPairCount;
                    if (!AppendConnectedEdges(
                            *sectionEdges, SourceKind::ShapeIntersection, options,
                            nullptr, nullptr, firstIndex, secondIndex,
                            candidates, digests, collected))
                    {
                        return fail(QStringLiteral("STEP 截交候选数量或采样点数超过上限。"));
                    }
                }
            }
        }

        const QFileInfo finalInfo(stepFilePath);
        if (!finalInfo.exists() || !finalInfo.isFile() || finalInfo.isSymLink()
            || finalInfo.size() != originalSize
            || finalInfo.lastModified().toMSecsSinceEpoch() != originalModifiedMs)
        {
            return fail(QStringLiteral("STEP 文件在候选提取期间发生变化，结果已丢弃。"));
        }
        std::sort(candidates.begin(), candidates.end(), [](const Candidate& left, const Candidate& right)
        {
            if (std::abs(left.lengthMm - right.lengthMm) > 1.0e-9)
                return left.lengthMm > right.lengthMm;
            return left.candidateId < right.candidateId;
        });
        if (statistics != nullptr) *statistics = collected;
        return true;
    }
    catch (const Standard_Failure& failure)
    {
        const QString detail = ExceptionDetail(failure.GetMessageString());
        return fail(detail.isEmpty()
            ? QStringLiteral("Open CASCADE 在焊缝候选提取时发生异常。")
            : QStringLiteral("Open CASCADE 在焊缝候选提取时发生异常：%1").arg(detail));
    }
    catch (const std::bad_alloc&)
    {
        return fail(QStringLiteral("STEP 焊缝候选提取内存不足。"));
    }
    catch (const std::exception& exception)
    {
        const QString detail = ExceptionDetail(exception.what());
        return fail(detail.isEmpty()
            ? QStringLiteral("STEP 焊缝候选提取发生异常。")
            : QStringLiteral("STEP 焊缝候选提取发生异常：%1").arg(detail));
    }
    catch (...)
    {
        return fail(QStringLiteral("STEP 焊缝候选提取发生未知异常。"));
    }
}
