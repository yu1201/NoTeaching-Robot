#include "StepModelImporter.h"

#include "OpenCascadeOperationGuard.h"

#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <Poly_Triangle.hxx>
#include <Poly_Triangulation.hxx>
#include <Standard_Failure.hxx>
#include <Standard_Version.hxx>
#include <STEPControl_Reader.hxx>
#include <TColStd_SequenceOfAsciiString.hxx>
#include <TopAbs_Orientation.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>

#if OCC_VERSION_MAJOR != 7 || OCC_VERSION_MINOR != 9 || OCC_VERSION_MAINTENANCE != 3
#error "StepModelImporter requires Open CASCADE Technology 7.9.3."
#endif

#include <QDateTime>
#include <QFileInfo>
#include <QMutex>
#include <QMutexLocker>

#include <algorithm>
#include <cmath>
#include <exception>
#include <filesystem>
#include <fstream>
#include <limits>
#include <new>
#include <utility>

namespace
{
constexpr qint64 kHardMaximumFileBytes = 256LL * 1024LL * 1024LL;
constexpr qsizetype kHardMaximumVertices = 2'000'000;
constexpr qsizetype kHardMaximumTriangles = 4'000'000;
constexpr double kMinimumExplicitDeflectionMm = 1.0e-6;
constexpr double kMinimumAngularDeflectionRadians = 1.0e-6;
constexpr double kPi = 3.14159265358979323846;

bool IsFinitePoint(const Eigen::Vector3d& point)
{
    return point.allFinite();
}

bool FitsFloat(const Eigen::Vector3d& point)
{
    const double maximum = static_cast<double>((std::numeric_limits<float>::max)());
    return point.cwiseAbs().maxCoeff() <= maximum;
}

bool HasSafeFloatRoundTrip(const Eigen::Vector3d& point, double maximumErrorMm)
{
    if (!FitsFloat(point) || !std::isfinite(maximumErrorMm) || maximumErrorMm <= 0.0)
        return false;
    const Eigen::Vector3d stored = point.cast<float>().cast<double>();
    return (stored - point).cwiseAbs().maxCoeff() <= maximumErrorMm;
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
    return QStringLiteral("STEP 文件返回了未知的读取状态。");
}

QString ExceptionDetail(const char* message)
{
    if (message == nullptr || *message == '\0') return QString();
    QString text = QString::fromLocal8Bit(message).simplified();
    constexpr qsizetype kMaximumDetailCharacters = 400;
    if (text.size() > kMaximumDetailCharacters)
        text = text.left(kMaximumDetailCharacters) + QStringLiteral("...");
    return text;
}

QStringList ReadLengthUnitNames(STEPControl_Reader& reader)
{
    TColStd_SequenceOfAsciiString lengthNames;
    TColStd_SequenceOfAsciiString angleNames;
    TColStd_SequenceOfAsciiString solidAngleNames;
    reader.FileUnits(lengthNames, angleNames, solidAngleNames);

    QStringList result;
    for (Standard_Integer index = 1; index <= lengthNames.Length(); ++index)
    {
        const QString name = QString::fromLatin1(lengthNames.Value(index).ToCString()).trimmed();
        if (!name.isEmpty() && !result.contains(name, Qt::CaseInsensitive)) result.append(name);
    }
    return result;
}

bool ValidateOptions(const StepModelImporter::Options& options, QString& error)
{
    if (options.maximumFileBytes <= 0 || options.maximumFileBytes > kHardMaximumFileBytes)
    {
        error = QStringLiteral("STEP 文件大小限制无效：允许范围为 1 字节到 256 MiB。");
        return false;
    }
    if (options.maximumVertices < 3 || options.maximumVertices > kHardMaximumVertices)
    {
        error = QStringLiteral("STEP 顶点数限制无效：允许范围为 3 到 2000000。");
        return false;
    }
    if (options.maximumTriangles < 1 || options.maximumTriangles > kHardMaximumTriangles)
    {
        error = QStringLiteral("STEP 三角形数限制无效：允许范围为 1 到 4000000。");
        return false;
    }
    if (!std::isfinite(options.linearDeflectionMm) || options.linearDeflectionMm < 0.0
        || (options.linearDeflectionMm > 0.0
            && options.linearDeflectionMm < kMinimumExplicitDeflectionMm))
    {
        error = QStringLiteral("STEP 线性离散误差无效：应为 0（自适应）或不小于 0.000001 mm。");
        return false;
    }
    if (!std::isfinite(options.angularDeflectionRadians)
        || options.angularDeflectionRadians < kMinimumAngularDeflectionRadians
        || options.angularDeflectionRadians > kPi)
    {
        error = QStringLiteral("STEP 角度离散误差无效：应位于 (0, π] 弧度范围内。");
        return false;
    }
    return true;
}
}

bool StepModelImporter::ImportFile(
    const QString& stepFilePath,
    WorkpieceMeshBuilder::Mesh& mesh,
    QString& error,
    Statistics* statistics,
    const Options* tighterLimits)
{
    QMutexLocker<QMutex> importLock(&occtsync::OperationMutex());

    mesh = WorkpieceMeshBuilder::Mesh();
    error.clear();
    if (statistics != nullptr) *statistics = Statistics();

    const auto fail = [&mesh, &error](const QString& message)
    {
        mesh = WorkpieceMeshBuilder::Mesh();
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
        const QString extension = originalInfo.suffix().toLower();
        if (extension != QStringLiteral("step") && extension != QStringLiteral("stp"))
            return fail(QStringLiteral("文件扩展名不是 .step 或 .stp：%1").arg(stepFilePath));
        const qint64 originalFileSize = originalInfo.size();
        const qint64 originalModifiedMs = originalInfo.lastModified().toMSecsSinceEpoch();
        if (originalFileSize <= 0 || originalFileSize > options.maximumFileBytes)
        {
            return fail(QStringLiteral("STEP 文件为空或超过大小限制（当前上限 %1 MiB）。")
                            .arg(options.maximumFileBytes / (1024LL * 1024LL)));
        }

#ifdef _WIN32
        const std::filesystem::path nativePath(stepFilePath.toStdWString());
#else
        const QByteArray encodedPath = stepFilePath.toUtf8();
        const std::filesystem::path nativePath(encodedPath.constData());
#endif
        std::ifstream input(nativePath, std::ios::in | std::ios::binary);
        if (!input.is_open() || input.fail())
            return fail(QStringLiteral("无法打开 STEP 文件：%1").arg(stepFilePath));

        STEPControl_Reader reader;
        const IFSelect_ReturnStatus readStatus = reader.ReadStream("input.step", input);
        input.close();
        if (readStatus != IFSelect_RetDone) return fail(ReadFailureText(readStatus));

        Statistics importedStatistics;
        importedStatistics.occtVersion = QString::fromLatin1(OCC_VERSION_COMPLETE);
        importedStatistics.sourceLengthUnits = ReadLengthUnitNames(reader);
        if (importedStatistics.sourceLengthUnits.isEmpty())
        {
            return fail(QStringLiteral(
                "STEP 文件没有可验证的长度单位，已拒绝导入以避免把未知单位误当成毫米。"));
        }

        // 只设置本 reader 的目标系统单位：OCCT 的 1.0 表示 1 mm。不要修改进程级全局单位。
        reader.SetSystemLengthUnit(1.0);
        const Standard_Integer rootCount = reader.NbRootsForTransfer();
        if (rootCount <= 0) return fail(QStringLiteral("STEP 文件中没有可转换的产品根节点。"));
        const Standard_Integer transferredRootCount = reader.TransferRoots();
        if (transferredRootCount != rootCount)
        {
            return fail(QStringLiteral("STEP 根节点仅成功转换 %1/%2 个，已拒绝不完整模型。")
                            .arg(transferredRootCount)
                            .arg(rootCount));
        }

        TopoDS_Shape shape = reader.OneShape();
        if (shape.IsNull()) return fail(QStringLiteral("STEP 转换完成但没有生成可用几何体。"));

        // 先移除文件中可能携带的旧离散网格，确保本次 deflection 参数真正生效。
        BRepTools::Clean(shape);

        Bnd_Box shapeBounds;
        BRepBndLib::Add(shape, shapeBounds, Standard_False);
        if (shapeBounds.IsVoid() || shapeBounds.IsOpen())
            return fail(QStringLiteral("STEP 模型包围盒为空或无界，无法进行安全三角化。"));

        Standard_Real xMinimum = 0.0;
        Standard_Real yMinimum = 0.0;
        Standard_Real zMinimum = 0.0;
        Standard_Real xMaximum = 0.0;
        Standard_Real yMaximum = 0.0;
        Standard_Real zMaximum = 0.0;
        shapeBounds.Get(xMinimum, yMinimum, zMinimum, xMaximum, yMaximum, zMaximum);
        const Eigen::Vector3d exactMinimum(xMinimum, yMinimum, zMinimum);
        const Eigen::Vector3d exactMaximum(xMaximum, yMaximum, zMaximum);
        if (!IsFinitePoint(exactMinimum) || !IsFinitePoint(exactMaximum)
            || !FitsFloat(exactMinimum) || !FitsFloat(exactMaximum))
        {
            return fail(QStringLiteral("STEP 模型包围盒含非有限值或超出本工程浮点坐标范围。"));
        }
        const double diagonalMm = (exactMaximum - exactMinimum).norm();
        if (!std::isfinite(diagonalMm) || diagonalMm <= 0.0)
            return fail(QStringLiteral("STEP 模型包围盒尺寸无效。"));

        importedStatistics.linearDeflectionMm = options.linearDeflectionMm > 0.0
            ? options.linearDeflectionMm
            : std::clamp(diagonalMm / 5000.0, 0.02, 0.20);
        const double maximumFloatRoundTripErrorMm = std::clamp(
            importedStatistics.linearDeflectionMm * 0.10, 0.001, 0.010);

        BRepMesh_IncrementalMesh triangulator(
            shape,
            importedStatistics.linearDeflectionMm,
            Standard_False,
            options.angularDeflectionRadians,
            Standard_True);
        if (!triangulator.IsDone())
            return fail(QStringLiteral("Open CASCADE 未能完成 STEP 模型三角化。"));

        // 第一遍只计数并验证每一个面都确实有离散结果，避免不可信计数驱动无界分配。
        qsizetype candidateVertexCount = 0;
        qsizetype candidateTriangleCount = 0;
        for (TopExp_Explorer explorer(shape, TopAbs_FACE); explorer.More(); explorer.Next())
        {
            ++importedStatistics.sourceFaceCount;
            const TopoDS_Face face = TopoDS::Face(explorer.Current());
            TopLoc_Location location;
            const Handle(Poly_Triangulation) faceMesh = BRep_Tool::Triangulation(face, location);
            if (faceMesh.IsNull() || faceMesh->NbNodes() < 3 || faceMesh->NbTriangles() < 1)
            {
                return fail(QStringLiteral("STEP 第 %1 个曲面未生成完整三角网格，已拒绝部分模型。")
                                .arg(importedStatistics.sourceFaceCount));
            }

            const qsizetype faceVertexCount = static_cast<qsizetype>(faceMesh->NbNodes());
            const qsizetype faceTriangleCount = static_cast<qsizetype>(faceMesh->NbTriangles());
            if (candidateVertexCount > options.maximumVertices - faceVertexCount)
                return fail(QStringLiteral("STEP 三角化后顶点数超过限制（当前上限 %1）。")
                                .arg(options.maximumVertices));
            if (candidateTriangleCount > options.maximumTriangles - faceTriangleCount)
                return fail(QStringLiteral("STEP 三角化后三角形数超过限制（当前上限 %1）。")
                                .arg(options.maximumTriangles));
            candidateVertexCount += faceVertexCount;
            candidateTriangleCount += faceTriangleCount;
        }
        if (importedStatistics.sourceFaceCount == 0)
            return fail(QStringLiteral("STEP 模型不包含可三角化的曲面。"));

        WorkpieceMeshBuilder::Mesh importedMesh;
        importedMesh.vertices.reserve(candidateVertexCount);
        importedMesh.normals.reserve(candidateVertexCount);
        importedMesh.indices.reserve(candidateTriangleCount * 3);

        Eigen::Vector3d meshMinimum = Eigen::Vector3d::Constant(
            (std::numeric_limits<double>::max)());
        Eigen::Vector3d meshMaximum = Eigen::Vector3d::Constant(
            (std::numeric_limits<double>::lowest)());

        // 每个 CAD 面复制一份节点，使法线只在该面内部平均，不跨实体硬边错误平滑。
        for (TopExp_Explorer explorer(shape, TopAbs_FACE); explorer.More(); explorer.Next())
        {
            const TopoDS_Face face = TopoDS::Face(explorer.Current());
            TopLoc_Location location;
            const Handle(Poly_Triangulation) faceMesh = BRep_Tool::Triangulation(face, location);
            if (faceMesh.IsNull())
                return fail(QStringLiteral("STEP 曲面网格在转换过程中意外丢失。"));

            const qsizetype firstVertex = importedMesh.vertices.size();
            const gp_Trsf transformation = location.Transformation();
            for (Standard_Integer nodeIndex = 1; nodeIndex <= faceMesh->NbNodes(); ++nodeIndex)
            {
                gp_Pnt point = faceMesh->Node(nodeIndex);
                point.Transform(transformation);
                const Eigen::Vector3d converted(point.X(), point.Y(), point.Z());
                if (!IsFinitePoint(converted) || !HasSafeFloatRoundTrip(
                        converted, maximumFloatRoundTripErrorMm))
                {
                    return fail(QStringLiteral(
                        "STEP顶点转为本工程float网格时精度损失超过 %1 mm；"
                        "模型原点可能离工件过远，请在CAD中建立靠近工件的模型原点后重试。")
                            .arg(maximumFloatRoundTripErrorMm, 0, 'f', 3));
                }

                const Eigen::Vector3f storedPoint = converted.cast<float>();
                const Eigen::Vector3d storedPointDouble = storedPoint.cast<double>();
                if (!storedPoint.allFinite())
                    return fail(QStringLiteral("STEP 三角网格含非有限值或超出浮点范围的顶点。"));

                importedMesh.vertices.append(storedPoint);
                importedMesh.normals.append(Eigen::Vector3f::Zero());
                // 统计必须描述最终落盘的 float 网格，而不是转换前的 double 坐标。
                meshMinimum = meshMinimum.cwiseMin(storedPointDouble);
                meshMaximum = meshMaximum.cwiseMax(storedPointDouble);
            }

            qsizetype validFaceTriangles = 0;
            const bool reverseWinding = (face.Orientation() == TopAbs_REVERSED)
                != static_cast<bool>(transformation.IsNegative());
            for (Standard_Integer triangleIndex = 1;
                 triangleIndex <= faceMesh->NbTriangles();
                 ++triangleIndex)
            {
                Standard_Integer localIndex0 = 0;
                Standard_Integer localIndex1 = 0;
                Standard_Integer localIndex2 = 0;
                faceMesh->Triangle(triangleIndex).Get(localIndex0, localIndex1, localIndex2);
                if (reverseWinding) std::swap(localIndex1, localIndex2);

                const Standard_Integer nodeCount = faceMesh->NbNodes();
                if (localIndex0 < 1 || localIndex0 > nodeCount
                    || localIndex1 < 1 || localIndex1 > nodeCount
                    || localIndex2 < 1 || localIndex2 > nodeCount)
                {
                    return fail(QStringLiteral("STEP 三角网格包含越界的顶点索引。"));
                }

                const qsizetype index0 = firstVertex + static_cast<qsizetype>(localIndex0 - 1);
                const qsizetype index1 = firstVertex + static_cast<qsizetype>(localIndex1 - 1);
                const qsizetype index2 = firstVertex + static_cast<qsizetype>(localIndex2 - 1);
                if (index0 == index1 || index1 == index2 || index0 == index2)
                {
                    ++importedStatistics.skippedDegenerateTriangles;
                    continue;
                }

                const Eigen::Vector3d point0 = importedMesh.vertices.at(index0).cast<double>();
                const Eigen::Vector3d point1 = importedMesh.vertices.at(index1).cast<double>();
                const Eigen::Vector3d point2 = importedMesh.vertices.at(index2).cast<double>();
                const Eigen::Vector3d edge01 = point1 - point0;
                const Eigen::Vector3d edge02 = point2 - point0;
                const Eigen::Vector3d cross = edge01.cross(edge02);
                const double maximumEdgeSquared = (std::max)(
                    edge01.squaredNorm(),
                    (std::max)(edge02.squaredNorm(), (point2 - point1).squaredNorm()));
                const double minimumCrossSquared = (std::max)(
                    1.0e-24,
                    maximumEdgeSquared * maximumEdgeSquared * 1.0e-24);
                const double crossSquared = cross.squaredNorm();
                if (!std::isfinite(crossSquared) || crossSquared <= minimumCrossSquared)
                {
                    ++importedStatistics.skippedDegenerateTriangles;
                    continue;
                }

                if (importedMesh.indices.size() / 3 >= options.maximumTriangles)
                    return fail(QStringLiteral("STEP 有效三角形数超过限制。"));
                importedMesh.indices.append(static_cast<quint32>(index0));
                importedMesh.indices.append(static_cast<quint32>(index1));
                importedMesh.indices.append(static_cast<quint32>(index2));

                const Eigen::Vector3f unitNormal = (cross / std::sqrt(crossSquared)).cast<float>();
                importedMesh.normals[index0] += unitNormal;
                importedMesh.normals[index1] += unitNormal;
                importedMesh.normals[index2] += unitNormal;
                ++validFaceTriangles;
            }
            if (validFaceTriangles == 0)
                return fail(QStringLiteral("STEP 中存在仅含退化三角形的曲面，已拒绝不完整模型。"));
            ++importedStatistics.meshedFaceCount;
        }

        for (Eigen::Vector3f& normal : importedMesh.normals)
        {
            const float length = normal.norm();
            if (std::isfinite(length) && length > 1.0e-9f)
                normal /= length;
            else
                normal = Eigen::Vector3f::UnitZ();
        }

        if (!importedMesh.IsValid() || importedMesh.normals.size() != importedMesh.vertices.size()
            || importedMesh.indices.size() % 3 != 0)
        {
            return fail(QStringLiteral("STEP 转换得到的网格结构不完整。"));
        }
        for (const Eigen::Vector3f& vertex : importedMesh.vertices)
        {
            if (!vertex.allFinite()) return fail(QStringLiteral("STEP 网格含非有限顶点。"));
        }
        for (const Eigen::Vector3f& normal : importedMesh.normals)
        {
            if (!normal.allFinite() || normal.squaredNorm() <= 0.0f)
                return fail(QStringLiteral("STEP 网格含无效法线。"));
        }
        for (quint32 index : importedMesh.indices)
        {
            if (static_cast<qsizetype>(index) >= importedMesh.vertices.size())
                return fail(QStringLiteral("STEP 网格含越界索引。"));
        }

        QFileInfo finalInfo(stepFilePath);
        finalInfo.refresh();
        if (!finalInfo.exists() || !finalInfo.isFile() || finalInfo.isSymLink()
            || finalInfo.size() != originalFileSize
            || finalInfo.lastModified().toMSecsSinceEpoch() != originalModifiedMs)
        {
            return fail(QStringLiteral("STEP 文件在导入过程中发生变化，请重新导入。"));
        }

        importedStatistics.vertexCount = importedMesh.vertices.size();
        importedStatistics.triangleCount = importedMesh.indices.size() / 3;
        importedStatistics.boundsMinMm = meshMinimum;
        importedStatistics.boundsMaxMm = meshMaximum;

        mesh = std::move(importedMesh);
        if (statistics != nullptr) *statistics = std::move(importedStatistics);
        return true;
    }
    catch (const Standard_Failure& exception)
    {
        const QString detail = ExceptionDetail(exception.GetMessageString());
        return fail(detail.isEmpty()
                ? QStringLiteral("Open CASCADE 在导入 STEP 时发生异常。")
                : QStringLiteral("Open CASCADE 在导入 STEP 时发生异常：%1").arg(detail));
    }
    catch (const std::bad_alloc&)
    {
        return fail(QStringLiteral("导入 STEP 时内存不足，模型可能过于复杂。"));
    }
    catch (const std::exception& exception)
    {
        const QString detail = ExceptionDetail(exception.what());
        return fail(detail.isEmpty()
                ? QStringLiteral("导入 STEP 时发生标准库异常。")
                : QStringLiteral("导入 STEP 时发生异常：%1").arg(detail));
    }
    catch (...)
    {
        return fail(QStringLiteral("导入 STEP 时发生未知异常。"));
    }
}
