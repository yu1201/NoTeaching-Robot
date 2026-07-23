#include "CadModel3DView.h"

#include "OpenCascadeOperationGuard.h"
#include "RobotCadAssemblyLoader.h"
#include "RobotCollisionEnvelopeStore.h"

#ifdef _MSC_VER
#pragma warning(push)
// OCCT 7.9.3 的 V3d_Light.hxx 内部仍引用其本版标记为 deprecated 的兼容
// handle；这是第三方头自身的过渡告警，不应被本工程的 /sdl 门禁提升为错误。
#pragma warning(disable: 4996)
#endif

#include <AIS_DisplayMode.hxx>
#include <AIS_InteractiveContext.hxx>
#include <AIS_Point.hxx>
#include <AIS_Shape.hxx>
#include <Aspect_DisplayConnection.hxx>
#include <Aspect_TypeOfLine.hxx>
#include <Aspect_TypeOfMarker.hxx>
#include <Aspect_TypeOfTriedronPosition.hxx>
#include <Bnd_Box.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepGProp.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRep_Tool.hxx>
#include <BRep_Builder.hxx>
#include <GProp_GProps.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <Geom_CartesianPoint.hxx>
#include <Graphic3d_MaterialAspect.hxx>
#include <Graphic3d_NameOfMaterial.hxx>
#include <Graphic3d_TypeOfShadingModel.hxx>
#include <Graphic3d_ZLayerId.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <OpenGl_GraphicDriver.hxx>
#include <Prs3d_Drawer.hxx>
#include <Prs3d_LineAspect.hxx>
#include <Prs3d_PointAspect.hxx>
#include <Quantity_Color.hxx>
#include <STEPControl_Reader.hxx>
#include <Standard_Failure.hxx>
#include <Standard_Version.hxx>
#include <TColStd_SequenceOfAsciiString.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopLoc_Location.hxx>
#include <V3d_TypeOfOrientation.hxx>
#include <V3d_TypeOfVisualization.hxx>
#include <V3d_View.hxx>
#include <V3d_Viewer.hxx>
#include <WNT_Window.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Quaternion.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

#ifdef _MSC_VER
#pragma warning(pop)
#endif

#if OCC_VERSION_MAJOR != 7 || OCC_VERSION_MINOR != 9 || OCC_VERSION_MAINTENANCE != 3
#error "CadModel3DView requires Open CASCADE Technology 7.9.3."
#endif

#include <QApplication>
#include <QByteArray>
#include <QCryptographicHash>
#include <QDir>
#include <QEvent>
#include <QFile>
#include <QFileInfo>
#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QMouseEvent>
#include <QMutexLocker>
#include <QPaintEngine>
#include <QRegularExpression>
#include <QRect>
#include <QResizeEvent>
#include <QShowEvent>
#include <QThread>
#include <QTimer>
#include <QToolButton>
#include <QWheelEvent>

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <filesystem>
#include <fstream>
#include <limits>
#include <utility>
#include <vector>

namespace cadview
{
namespace
{
constexpr qint64 kMaximumCadFileBytes = 256LL * 1024LL * 1024LL;

Quantity_Color ToOcctColor(const QColor& color)
{
    const auto channel = [](float value) -> Standard_Real
    {
        return std::clamp(static_cast<Standard_Real>(value), 0.0, 1.0);
    };
    return Quantity_Color(
        channel(color.redF()),
        channel(color.greenF()),
        channel(color.blueF()),
        Quantity_TOC_RGB);
}

gp_Pnt ToPoint(const Vec3d& point)
{
    return gp_Pnt(point.x, point.y, point.z);
}

bool IsFinite(const Vec3d& point)
{
    return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

bool ValidateRobotCollisionEnvelopeForScene(
    const RobotCollisionEnvelopeStore::EnvelopeSet& envelope,
    QString& error)
{
    if (!RobotCollisionEnvelopeStore::ValidateForUse(envelope, error))
        return false;
    constexpr double kTolerance = 1.0e-9;
    if ((envelope.sourceUp - Eigen::Vector3d::UnitY()).norm() > kTolerance)
    {
        error = QStringLiteral(
            "当前 SA10 场景只接受源装配 +Y 朝上的机器人碰撞简模。");
        return false;
    }
    if (envelope.overallExpandedBoundsMm.minimumMm.y()
        < envelope.baseMinimumYmm - kTolerance)
    {
        error = QStringLiteral("机器人碰撞简模有几何体低于 J0 安装地面。");
        return false;
    }
    return true;
}

QString NormalizedExistingPath(const QString& path)
{
    const QFileInfo info(path);
    const QString canonical = info.canonicalFilePath();
    return QDir::cleanPath(canonical.isEmpty() ? info.absoluteFilePath() : canonical);
}

QString StableFileSha256(
    const QString& path,
    const QFileInfo& expected,
    QString& error)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly) || file.size() != expected.size())
    {
        error = QStringLiteral("无法稳定读取原始 STEP 以校验 SHA-256：%1")
            .arg(file.errorString());
        return QString();
    }
    QCryptographicHash hash(QCryptographicHash::Sha256);
    QByteArray block(1024 * 1024, Qt::Uninitialized);
    while (!file.atEnd())
    {
        const qint64 count = file.read(block.data(), block.size());
        if (count < 0)
        {
            error = QStringLiteral("读取原始 STEP 以校验 SHA-256 失败：%1")
                .arg(file.errorString());
            return QString();
        }
        if (count > 0)
        {
            hash.addData(QByteArrayView(block.constData(), static_cast<qsizetype>(count)));
        }
    }
    file.close();
    QFileInfo after(path);
    after.refresh();
    if (!after.exists() || !after.isFile() || after.isSymLink()
        || after.size() != expected.size()
        || after.lastModified().toMSecsSinceEpoch()
            != expected.lastModified().toMSecsSinceEpoch())
    {
        error = QStringLiteral("原始 STEP 在 SHA-256 校验过程中发生变化。");
        return QString();
    }
    return QString::fromLatin1(hash.result().toHex()).toLower();
}

bool SamePath(const QString& left, const QString& right)
{
#ifdef _WIN32
    return NormalizedExistingPath(left).compare(
        NormalizedExistingPath(right), Qt::CaseInsensitive) == 0;
#else
    return NormalizedExistingPath(left) == NormalizedExistingPath(right);
#endif
}

QString ReadStatusText(IFSelect_ReturnStatus status)
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
    return QStringLiteral("STEP 文件返回未知读取状态。");
}

TopoDS_Compound SegmentShape(const GuideSegment& segment)
{
    TopoDS_Compound compound;
    BRep_Builder builder;
    builder.MakeCompound(compound);
    const gp_Pnt start = ToPoint(segment.start);
    const gp_Pnt end = ToPoint(segment.end);
    builder.Add(compound, BRepBuilderAPI_MakeEdge(start, end).Edge());

    if (!segment.arrowHead) return compound;
    gp_Vec direction(start, end);
    const double length = direction.Magnitude();
    if (length <= 1.0e-6) return compound;
    direction.Normalize();
    gp_Vec reference(0.0, 0.0, 1.0);
    if (std::abs(direction.Dot(reference)) > 0.9) reference = gp_Vec(0.0, 1.0, 0.0);
    gp_Vec side = direction.Crossed(reference);
    if (side.Magnitude() <= 1.0e-9) return compound;
    side.Normalize();
    const double headLength = std::clamp(length * 0.25, 6.0, 30.0);
    const gp_Pnt back = end.Translated(direction.Multiplied(-headLength));
    const gp_Pnt left = back.Translated(side.Multiplied(headLength * 0.45));
    const gp_Pnt right = back.Translated(side.Multiplied(-headLength * 0.45));
    builder.Add(compound, BRepBuilderAPI_MakeEdge(end, left).Edge());
    builder.Add(compound, BRepBuilderAPI_MakeEdge(end, right).Edge());
    return compound;
}

Vec3d Add(const Vec3d& left, const Vec3d& right)
{
    return { left.x + right.x, left.y + right.y, left.z + right.z };
}

Vec3d Subtract(const Vec3d& left, const Vec3d& right)
{
    return { left.x - right.x, left.y - right.y, left.z - right.z };
}

Vec3d Multiply(const Vec3d& value, double scale)
{
    return { value.x * scale, value.y * scale, value.z * scale };
}

double Dot(const Vec3d& left, const Vec3d& right)
{
    return left.x * right.x + left.y * right.y + left.z * right.z;
}

double Length(const Vec3d& value)
{
    return std::sqrt(Dot(value, value));
}

bool Normalize(const Vec3d& input, Vec3d& output)
{
    const double length = Length(input);
    if (!std::isfinite(length) || length <= 1.0e-9) return false;
    output = Multiply(input, 1.0 / length);
    return IsFinite(output);
}

Vec3d Cross(const Vec3d& left, const Vec3d& right)
{
    return {
        left.y * right.z - left.z * right.y,
        left.z * right.x - left.x * right.z,
        left.x * right.y - left.y * right.x
    };
}

bool NormalizeFixtureAxes(
    const VSlotFixture& fixture,
    Vec3d& axisX,
    Vec3d& axisY,
    Vec3d& axisZ)
{
    if (!Normalize(fixture.axisX, axisX)
        || !Normalize(fixture.axisY, axisY)
        || !Normalize(fixture.axisZ, axisZ))
    {
        return false;
    }
    if (std::abs(Dot(axisX, axisY)) > 1.0e-5
        || std::abs(Dot(axisX, axisZ)) > 1.0e-5
        || std::abs(Dot(axisY, axisZ)) > 1.0e-5)
    {
        return false;
    }
    Vec3d handed;
    if (!Normalize(Cross(axisX, axisY), handed) || Dot(handed, axisZ) < 0.99999)
        return false;
    return true;
}

gp_Pnt FixturePoint(
    const Vec3d& anchor,
    const Vec3d& axisX,
    const Vec3d& axisY,
    double x,
    double y)
{
    return ToPoint(Add(anchor, Add(Multiply(axisX, x), Multiply(axisY, y))));
}

TopoDS_Shape BuildVSlotShape(const VSlotFixture& fixture)
{
    Vec3d axisX;
    Vec3d axisY;
    Vec3d axisZ;
    if (!fixture.visible || !IsFinite(fixture.anchor)
        || !NormalizeFixtureAxes(fixture, axisX, axisY, axisZ)
        || !std::isfinite(fixture.longLengthMm)
        || !std::isfinite(fixture.shortLengthMm)
        || !std::isfinite(fixture.wallThicknessMm)
        || !std::isfinite(fixture.wallHeightMm)
        || fixture.longLengthMm <= 1.0e-6
        || fixture.shortLengthMm <= 1.0e-6
        || fixture.wallThicknessMm <= 1.0e-6
        || fixture.wallHeightMm <= 1.0e-6)
    {
        return TopoDS_Shape();
    }

    const double thickness = std::min(
        fixture.wallThicknessMm,
        std::min(fixture.longLengthMm, fixture.shortLengthMm) * 0.25);
    BRepBuilderAPI_MakePolygon polygon;
    polygon.Add(FixturePoint(fixture.anchor, axisX, axisY, -thickness, -thickness));
    polygon.Add(FixturePoint(fixture.anchor, axisX, axisY, fixture.longLengthMm, -thickness));
    polygon.Add(FixturePoint(fixture.anchor, axisX, axisY, fixture.longLengthMm, 0.0));
    polygon.Add(FixturePoint(fixture.anchor, axisX, axisY, 0.0, 0.0));
    polygon.Add(FixturePoint(fixture.anchor, axisX, axisY, 0.0, fixture.shortLengthMm));
    polygon.Add(FixturePoint(fixture.anchor, axisX, axisY, -thickness, fixture.shortLengthMm));
    polygon.Close();
    if (!polygon.IsDone()) return TopoDS_Shape();

    BRepBuilderAPI_MakeFace face(polygon.Wire());
    if (!face.IsDone()) return TopoDS_Shape();
    BRepPrimAPI_MakePrism prism(
        face.Face(),
        gp_Vec(axisZ.x, axisZ.y, axisZ.z).Multiplied(fixture.wallHeightMm));
    if (!prism.IsDone()) return TopoDS_Shape();
    return prism.Shape();
}

bool NormalizeGroundAxes(
    const GroundSurface& ground,
    Vec3d& axisX,
    Vec3d& axisY,
    Vec3d& axisZ)
{
    VSlotFixture frame;
    frame.axisX = ground.axisX;
    frame.axisY = ground.axisY;
    frame.axisZ = ground.axisZ;
    return NormalizeFixtureAxes(frame, axisX, axisY, axisZ);
}

Vec3d ProjectToPlane(
    const Vec3d& point,
    const Vec3d& planePoint,
    const Vec3d& unitNormal)
{
    return Subtract(point, Multiply(
        unitNormal, Dot(Subtract(point, planePoint), unitNormal)));
}

TopoDS_Shape BuildGroundSlab(
    const GroundSurface& ground,
    const Vec3d& center,
    double widthMm,
    double depthMm)
{
    Vec3d axisX;
    Vec3d axisY;
    Vec3d axisZ;
    if (!ground.visible || !IsFinite(center)
        || !NormalizeGroundAxes(ground, axisX, axisY, axisZ)
        || !std::isfinite(widthMm) || !std::isfinite(depthMm)
        || !std::isfinite(ground.thicknessMm)
        || widthMm <= 1.0e-6 || depthMm <= 1.0e-6
        || ground.thicknessMm <= 1.0e-6)
    {
        return TopoDS_Shape();
    }

    const double halfWidth = widthMm * 0.5;
    const double halfDepth = depthMm * 0.5;
    BRepBuilderAPI_MakePolygon polygon;
    polygon.Add(FixturePoint(center, axisX, axisY, -halfWidth, -halfDepth));
    polygon.Add(FixturePoint(center, axisX, axisY, halfWidth, -halfDepth));
    polygon.Add(FixturePoint(center, axisX, axisY, halfWidth, halfDepth));
    polygon.Add(FixturePoint(center, axisX, axisY, -halfWidth, halfDepth));
    polygon.Close();
    if (!polygon.IsDone()) return TopoDS_Shape();
    BRepBuilderAPI_MakeFace face(polygon.Wire());
    if (!face.IsDone()) return TopoDS_Shape();
    BRepPrimAPI_MakePrism prism(
        face.Face(),
        gp_Vec(axisZ.x, axisZ.y, axisZ.z).Multiplied(-ground.thicknessMm));
    if (!prism.IsDone()) return TopoDS_Shape();
    return prism.Shape();
}

TopoDS_Compound BuildGroundGrid(
    const GroundSurface& ground,
    const Vec3d& center,
    double widthMm,
    double depthMm,
    double spacingMm)
{
    TopoDS_Compound compound;
    BRep_Builder builder;
    builder.MakeCompound(compound);
    Vec3d axisX;
    Vec3d axisY;
    Vec3d axisZ;
    if (!ground.visible || !IsFinite(center)
        || !NormalizeGroundAxes(ground, axisX, axisY, axisZ)
        || !std::isfinite(widthMm) || !std::isfinite(depthMm)
        || !std::isfinite(spacingMm)
        || widthMm <= 1.0e-6 || depthMm <= 1.0e-6 || spacingMm <= 1.0e-6)
    {
        return compound;
    }

    const Vec3d liftedCenter = Add(center, Multiply(axisZ, 0.35));
    const double halfWidth = widthMm * 0.5;
    const double halfDepth = depthMm * 0.5;
    const int xLines = std::min(80, static_cast<int>(std::floor(widthMm / spacingMm)));
    const int yLines = std::min(80, static_cast<int>(std::floor(depthMm / spacingMm)));
    for (int index = -xLines / 2; index <= xLines / 2; ++index)
    {
        const double x = std::clamp(index * spacingMm, -halfWidth, halfWidth);
        builder.Add(compound, BRepBuilderAPI_MakeEdge(
            FixturePoint(liftedCenter, axisX, axisY, x, -halfDepth),
            FixturePoint(liftedCenter, axisX, axisY, x, halfDepth)).Edge());
    }
    for (int index = -yLines / 2; index <= yLines / 2; ++index)
    {
        const double y = std::clamp(index * spacingMm, -halfDepth, halfDepth);
        builder.Add(compound, BRepBuilderAPI_MakeEdge(
            FixturePoint(liftedCenter, axisX, axisY, -halfWidth, y),
            FixturePoint(liftedCenter, axisX, axisY, halfWidth, y)).Edge());
    }
    return compound;
}

TopoDS_Shape BuildScanRegionShape(const ScanRegion& region)
{
    Vec3d normal;
    if (!IsFinite(region.center) || !Normalize(region.outwardNormal, normal)
        || !std::isfinite(region.radiusMm) || !std::isfinite(region.thicknessMm)
        || region.radiusMm <= 1.0e-6 || region.thicknessMm <= 1.0e-6)
    {
        return TopoDS_Shape();
    }
    // 轻微抬离 CAD 表面，避免与原始 B-Rep 面发生 Z-fighting；偏移远小于扫描 ROI。
    const Vec3d origin = Add(region.center, Multiply(normal, region.thicknessMm * 0.15));
    BRepPrimAPI_MakeCylinder cylinder(
        gp_Ax2(ToPoint(origin), gp_Dir(normal.x, normal.y, normal.z)),
        region.radiusMm,
        region.thicknessMm);
    cylinder.Build();
    if (!cylinder.IsDone()) return TopoDS_Shape();
    return cylinder.Shape();
}

struct GroundSnapSegment
{
    Vec3d startModel;
    Vec3d endModel;
};

struct GroundFaceRecord
{
    GroundFaceCandidate summary;
    TopoDS_Compound faces;
    QVector<Vec3d> snapHull;
    QVector<GroundSnapSegment> snapSegments;
};

struct GroundSnapPoint2d
{
    double u = 0.0;
    double v = 0.0;
};

QVector<Vec3d> BuildGroundSnapHull(
    const QVector<Vec3d>& sourcePoints,
    const Vec3d& planePoint,
    const Vec3d& planeNormal)
{
    Vec3d normal;
    if (!IsFinite(planePoint) || !Normalize(planeNormal, normal)) return {};

    // 选择与法向最不平行的全局轴，避免接近轴向时叉乘数值退化。
    const double absX = std::abs(normal.x);
    const double absY = std::abs(normal.y);
    const double absZ = std::abs(normal.z);
    const Vec3d reference = absX <= absY && absX <= absZ
        ? Vec3d{ 1.0, 0.0, 0.0 }
        : (absY <= absZ ? Vec3d{ 0.0, 1.0, 0.0 }
                        : Vec3d{ 0.0, 0.0, 1.0 });
    Vec3d axisU;
    if (!Normalize(Cross(reference, normal), axisU)) return {};
    Vec3d axisV;
    if (!Normalize(Cross(normal, axisU), axisV)) return {};

    std::vector<GroundSnapPoint2d> projected;
    projected.reserve(static_cast<size_t>(sourcePoints.size()));
    double minU = (std::numeric_limits<double>::max)();
    double minV = (std::numeric_limits<double>::max)();
    double maxU = (std::numeric_limits<double>::lowest)();
    double maxV = (std::numeric_limits<double>::lowest)();
    for (const Vec3d& point : sourcePoints)
    {
        if (!IsFinite(point)) continue;
        const Vec3d delta = Subtract(point, planePoint);
        const GroundSnapPoint2d candidate{ Dot(delta, axisU), Dot(delta, axisV) };
        if (!std::isfinite(candidate.u) || !std::isfinite(candidate.v)) continue;
        projected.push_back(candidate);
        minU = std::min(minU, candidate.u);
        minV = std::min(minV, candidate.v);
        maxU = std::max(maxU, candidate.u);
        maxV = std::max(maxV, candidate.v);
    }
    if (projected.size() < 3) return {};

    const double span = std::hypot(maxU - minU, maxV - minV);
    // 模型通常以毫米计。容差随面尺寸缓慢缩放，同时限制上下界，既消除
    // STEP 重复顶点的浮点抖动，也不会吞掉真实的小倒角外角。
    const double duplicateTolerance = std::clamp(span * 1.0e-9, 1.0e-6, 1.0e-3);
    const double turnTolerance = std::max(
        1.0e-12, duplicateTolerance * std::max(span, 1.0));
    std::sort(projected.begin(), projected.end(), [](const auto& left, const auto& right)
        {
            if (left.u != right.u) return left.u < right.u;
            return left.v < right.v;
        });

    std::vector<GroundSnapPoint2d> unique;
    unique.reserve(projected.size());
    for (const GroundSnapPoint2d& point : projected)
    {
        bool duplicate = false;
        for (auto previous = unique.rbegin(); previous != unique.rend(); ++previous)
        {
            if (point.u - previous->u > duplicateTolerance) break;
            if (std::hypot(point.u - previous->u, point.v - previous->v)
                <= duplicateTolerance)
            {
                duplicate = true;
                break;
            }
        }
        if (!duplicate) unique.push_back(point);
    }
    if (unique.size() < 3) return {};

    const auto turn = [](const GroundSnapPoint2d& a,
        const GroundSnapPoint2d& b, const GroundSnapPoint2d& c)
        {
            return (b.u - a.u) * (c.v - a.v)
                - (b.v - a.v) * (c.u - a.u);
        };
    std::vector<GroundSnapPoint2d> hull;
    hull.reserve(unique.size() * 2);
    for (const GroundSnapPoint2d& point : unique)
    {
        while (hull.size() >= 2
            && turn(hull[hull.size() - 2], hull.back(), point) <= turnTolerance)
        {
            hull.pop_back();
        }
        hull.push_back(point);
    }
    const size_t lowerSize = hull.size();
    for (auto iterator = unique.rbegin() + 1; iterator != unique.rend(); ++iterator)
    {
        while (hull.size() > lowerSize
            && turn(hull[hull.size() - 2], hull.back(), *iterator) <= turnTolerance)
        {
            hull.pop_back();
        }
        hull.push_back(*iterator);
    }
    if (!hull.empty()) hull.pop_back();
    if (hull.size() < 3) return {};

    QVector<Vec3d> result;
    result.reserve(static_cast<qsizetype>(hull.size()));
    for (const GroundSnapPoint2d& point : hull)
    {
        // 始终重建到所选支撑平面，避免输入 STEP 顶点的微小离面误差传给 V 槽。
        result.push_back(Add(planePoint,
            Add(Multiply(axisU, point.u), Multiply(axisV, point.v))));
    }
    return result;
}

QVector<Vec3d> ExtractGroundSnapPoints(const GroundFaceRecord& record)
{
    if (!record.snapHull.isEmpty()) return record.snapHull;
    QVector<Vec3d> sourcePoints;
    for (TopExp_Explorer explorer(record.faces, TopAbs_VERTEX);
         explorer.More(); explorer.Next())
    {
        const gp_Pnt point = BRep_Tool::Pnt(TopoDS::Vertex(explorer.Current()));
        const Vec3d candidate = { point.X(), point.Y(), point.Z() };
        if (IsFinite(candidate)) sourcePoints.push_back(candidate);
    }
    // 只取 TopoDS_VERTEX 会漏掉圆弧、椭圆和样条边曲线内部的真实
    // +X/+Y 支撑极值。在候选分析阶段对非直线边按弧长采样，然后
    // 缓存凸包；鼠标拖动过程不会重复进行 B-Rep 曲线遍历。
    for (TopExp_Explorer explorer(record.faces, TopAbs_EDGE);
         explorer.More(); explorer.Next())
    {
        const TopoDS_Edge edge = TopoDS::Edge(explorer.Current());
        try
        {
            BRepAdaptor_Curve curve(edge);
            const double first = curve.FirstParameter();
            const double last = curve.LastParameter();
            if (!std::isfinite(first) || !std::isfinite(last) || last <= first)
                continue;
            int segments = 1;
            if (curve.GetType() != GeomAbs_Line)
            {
                GProp_GProps properties;
                BRepGProp::LinearProperties(edge, properties);
                const double length = std::abs(properties.Mass());
                if (!std::isfinite(length) || length <= 1.0e-6) continue;
                segments = std::clamp(
                    static_cast<int>(std::ceil(length / 5.0)), 16, 512);
            }
            for (int index = 0; index <= segments; ++index)
            {
                const double ratio = static_cast<double>(index) / segments;
                const gp_Pnt point = curve.Value(first + (last - first) * ratio);
                const Vec3d candidate = { point.X(), point.Y(), point.Z() };
                if (IsFinite(candidate)) sourcePoints.push_back(candidate);
            }
        }
        catch (...)
        {
            // 单条损坏边不得让整个 STEP 候选分析失败；其端点仍已在上方收集。
        }
    }
    return BuildGroundSnapHull(
        sourcePoints, record.summary.centerModel, record.summary.groundUpModel);
}

QVector<GroundSnapSegment> ExtractGroundSnapSegments(const GroundFaceRecord& record)
{
    if (!record.snapSegments.isEmpty()) return record.snapSegments;
    QVector<GroundSnapSegment> result;
    for (TopExp_Explorer explorer(record.faces, TopAbs_EDGE);
         explorer.More(); explorer.Next())
    {
        const TopoDS_Edge edge = TopoDS::Edge(explorer.Current());
        try
        {
            BRepAdaptor_Curve curve(edge);
            const double first = curve.FirstParameter();
            const double last = curve.LastParameter();
            if (!std::isfinite(first) || !std::isfinite(last) || last <= first)
                continue;
            int segmentCount = 1;
            if (curve.GetType() != GeomAbs_Line)
            {
                GProp_GProps properties;
                BRepGProp::LinearProperties(edge, properties);
                const double length = std::abs(properties.Mass());
                if (!std::isfinite(length) || length <= 1.0e-6) continue;
                segmentCount = std::clamp(
                    static_cast<int>(std::ceil(length / 5.0)), 16, 512);
            }
            Vec3d previous;
            bool hasPrevious = false;
            for (int index = 0; index <= segmentCount; ++index)
            {
                const double ratio = static_cast<double>(index) / segmentCount;
                const gp_Pnt point = curve.Value(first + (last - first) * ratio);
                const Vec3d current = { point.X(), point.Y(), point.Z() };
                if (!IsFinite(current))
                {
                    hasPrevious = false;
                    continue;
                }
                if (hasPrevious && Length(Subtract(current, previous)) > 1.0e-6)
                    result.push_back({ previous, current });
                previous = current;
                hasPrevious = true;
            }
        }
        catch (...)
        {
            // 损坏的单条边只失去其连续接触证明，不能回退为凸包虚拟边。
        }
    }
    return result;
}

std::vector<GroundFaceRecord> AnalyzeGroundFaceCandidates(const TopoDS_Shape& shape)
{
    Bnd_Box globalBounds;
    globalBounds.SetGap(0.0);
    BRepBndLib::AddOptimal(
        shape, globalBounds, Standard_False, Standard_False);
    if (globalBounds.IsVoid() || globalBounds.IsOpen()) return {};
    Standard_Real xMin = 0.0;
    Standard_Real yMin = 0.0;
    Standard_Real zMin = 0.0;
    Standard_Real xMax = 0.0;
    Standard_Real yMax = 0.0;
    Standard_Real zMax = 0.0;
    globalBounds.Get(xMin, yMin, zMin, xMax, yMax, zMax);
    const double dx = xMax - xMin;
    const double dy = yMax - yMin;
    const double dz = zMax - zMin;
    const double diagonal = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (!std::isfinite(diagonal) || diagonal <= 1.0e-6) return {};
    const double supportTolerance = std::max(0.05, diagonal * 1.0e-5);
    const double mergePlaneTolerance = std::max(0.10, diagonal * 2.0e-5);
    constexpr double kNormalCosine = 0.9999984769132877; // 0.1 degree

    const auto canonicalNormal = [](Vec3d normal)
    {
        constexpr double epsilon = 1.0e-12;
        if (normal.z < -epsilon
            || (std::abs(normal.z) <= epsilon && normal.y < -epsilon)
            || (std::abs(normal.z) <= epsilon && std::abs(normal.y) <= epsilon
                && normal.x < 0.0))
        {
            normal = Multiply(normal, -1.0);
        }
        return normal;
    };

    std::vector<GroundFaceRecord> records;
    BRep_Builder compoundBuilder;
    for (TopExp_Explorer explorer(shape, TopAbs_FACE); explorer.More(); explorer.Next())
    {
        const TopoDS_Face face = TopoDS::Face(explorer.Current());
        BRepAdaptor_Surface surface(face, Standard_True);
        if (surface.GetType() != GeomAbs_Plane) continue;

        GProp_GProps properties;
        BRepGProp::SurfaceProperties(face, properties, Standard_False, Standard_False);
        const double area = std::abs(properties.Mass());
        const gp_Pnt center = properties.CentreOfMass();
        if (!std::isfinite(area) || area <= 1.0e-6
            || !std::isfinite(center.X()) || !std::isfinite(center.Y())
            || !std::isfinite(center.Z()))
        {
            continue;
        }

        const gp_Dir planeDirection = surface.Plane().Axis().Direction();
        Vec3d normal = canonicalNormal(
            { planeDirection.X(), planeDirection.Y(), planeDirection.Z() });
        const Vec3d centerPoint = { center.X(), center.Y(), center.Z() };

        bool merged = false;
        for (GroundFaceRecord& record : records)
        {
            if (Dot(record.summary.groundUpModel, normal) < kNormalCosine)
                continue;
            const double recordOffset = Dot(
                record.summary.groundUpModel, record.summary.centerModel);
            const double candidateOffset = Dot(record.summary.groundUpModel, centerPoint);
            if (std::abs(recordOffset - candidateOffset) > mergePlaneTolerance)
                continue;
            const double combinedArea = record.summary.areaMm2 + area;
            record.summary.centerModel = Multiply(Add(
                Multiply(record.summary.centerModel, record.summary.areaMm2),
                Multiply(centerPoint, area)), 1.0 / combinedArea);
            Vec3d combinedNormal;
            if (Normalize(Add(
                    Multiply(record.summary.groundUpModel, record.summary.areaMm2),
                    Multiply(normal, area)), combinedNormal))
            {
                record.summary.groundUpModel = combinedNormal;
            }
            record.summary.areaMm2 = combinedArea;
            record.summary.largestFaceAreaMm2 = std::max(
                record.summary.largestFaceAreaMm2, area);
            compoundBuilder.Add(record.faces, face);
            merged = true;
            break;
        }
        if (!merged)
        {
            GroundFaceRecord record;
            record.summary.centerModel = centerPoint;
            record.summary.groundUpModel = normal;
            record.summary.areaMm2 = area;
            record.summary.largestFaceAreaMm2 = area;
            record.summary.supportToleranceMm = supportTolerance;
            compoundBuilder.MakeCompound(record.faces);
            compoundBuilder.Add(record.faces, face);
            records.push_back(record);
        }
    }

    // 先按单个连续 B-Rep 面的绝对面积筛掉小脚，避免对大量小面逐一做
    // 精确全形状投影。支撑极值检查后再按相对面积做第二次门禁。
    const double preliminaryMinimumArea = std::max(
        100.0, diagonal * diagonal * 1.0e-3);
    records.erase(std::remove_if(records.begin(), records.end(),
        [preliminaryMinimumArea](const GroundFaceRecord& record)
        {
            return record.summary.largestFaceAreaMm2 < preliminaryMinimumArea;
        }), records.end());

    std::vector<GroundFaceRecord> supportingRecords;
    supportingRecords.reserve(records.size());
    for (GroundFaceRecord& record : records)
    {
        const Vec3d canonical = record.summary.groundUpModel;
        const gp_Quaternion quaternion(
            gp_Vec(canonical.x, canonical.y, canonical.z),
            gp_Vec(0.0, 0.0, 1.0));
        gp_Trsf rotation;
        rotation.SetRotation(quaternion);
        const TopoDS_Shape rotatedShape = shape.Moved(
            TopLoc_Location(rotation), Standard_True);
        Bnd_Box projectedBounds;
        projectedBounds.SetGap(0.0);
        BRepBndLib::AddOptimal(
            rotatedShape, projectedBounds, Standard_False, Standard_False);
        if (projectedBounds.IsVoid() || projectedBounds.IsOpen()) continue;

        Standard_Real projectedXMin = 0.0;
        Standard_Real projectedYMin = 0.0;
        Standard_Real projectedZMin = 0.0;
        Standard_Real projectedXMax = 0.0;
        Standard_Real projectedYMax = 0.0;
        Standard_Real projectedZMax = 0.0;
        projectedBounds.Get(
            projectedXMin, projectedYMin, projectedZMin,
            projectedXMax, projectedYMax, projectedZMax);
        gp_Pnt rotatedCenter = ToPoint(record.summary.centerModel);
        rotatedCenter.Transform(rotation);
        const double planeZ = rotatedCenter.Z();
        if (std::abs(planeZ - projectedZMin) <= supportTolerance)
        {
            record.summary.groundUpModel = canonical;
        }
        else if (std::abs(planeZ - projectedZMax) <= supportTolerance)
        {
            record.summary.groundUpModel = Multiply(canonical, -1.0);
        }
        else
        {
            // 即使面积很大，只要不是整个 B-Rep 的外包络支撑极值就不能落地。
            continue;
        }
        supportingRecords.push_back(std::move(record));
    }
    records = std::move(supportingRecords);

    std::sort(records.begin(), records.end(), [](const GroundFaceRecord& left,
        const GroundFaceRecord& right)
        {
            if (left.summary.largestFaceAreaMm2 != right.summary.largestFaceAreaMm2)
                return left.summary.largestFaceAreaMm2 > right.summary.largestFaceAreaMm2;
            return left.summary.areaMm2 > right.summary.areaMm2;
        });
    if (records.empty()) return records;
    const double minimumArea = std::max({
        100.0,
        records.front().summary.largestFaceAreaMm2 * 0.05,
        diagonal * diagonal * 1.0e-3
    });
    records.erase(std::remove_if(records.begin(), records.end(),
        [minimumArea](const GroundFaceRecord& record)
        {
            return record.summary.largestFaceAreaMm2 < minimumArea;
        }), records.end());
    if (records.size() > 12) records.resize(12);
    for (GroundFaceRecord& record : records)
    {
        record.snapHull = ExtractGroundSnapPoints(record);
        record.snapSegments = ExtractGroundSnapSegments(record);
    }
    return records;
}

struct OverlayLabel
{
    Vec3d position;
    QString text;
    QColor color;
    bool emphasized = false;
};

}

class CadModel3DView::Impl
{
public:
    enum class DragMode
    {
        None,
        Rotate,
        Pan,
        VSlot
    };

    enum class RobotPresentationKind
    {
        None,
        DetailedBRep,
        CollisionEnvelope
    };

    struct RobotPlacementMetadata
    {
        bool valid = false;
        Eigen::Vector3d sourceUp = Eigen::Vector3d::UnitY();
        double sourceFloorYmm = 0.0;
        Eigen::Vector3d sourceBaseCenterMm = Eigen::Vector3d::Zero();
        RobotCollisionEnvelopeStore::AxisAlignedBounds sourceBoundsMm;
    };

    explicit Impl(CadModel3DView* owner)
        : q(owner)
    {
        statusLabel = new QLabel(q);
        statusLabel->setAttribute(Qt::WA_TransparentForMouseEvents, true);
        statusLabel->setWordWrap(true);
        statusLabel->setStyleSheet(QStringLiteral(
            "QLabel { color:#263238; background:rgba(255,255,255,238); "
            "border:1px solid #90a4ae; border-radius:4px; padding:6px 9px; }"));

        rotationPanel = new QFrame(q);
        rotationPanel->setObjectName(QStringLiteral("cadRotationPanel"));
        rotationPanel->setAttribute(Qt::WA_NoMousePropagation, true);
        rotationPanel->setStyleSheet(QStringLiteral(
            "QFrame#cadRotationPanel { background:rgba(255,255,255,242); "
            "border:1px solid #78909c; border-radius:5px; } "
            "QLabel { color:#263238; border:0; padding:0 3px; } "
            "QToolButton { color:#173746; background:#eef7fa; border:1px solid #78909c; "
            "border-radius:4px; padding:5px 8px; font-weight:600; } "
            "QToolButton:hover { background:#d8f0f5; } "
            "QToolButton:pressed { background:#b9e3eb; } "
            "QToolButton:disabled { color:#9eaaae; background:#edf0f1; }"));
        QHBoxLayout* rotationLayout = new QHBoxLayout(rotationPanel);
        rotationLayout->setContentsMargins(6, 5, 6, 5);
        rotationLayout->setSpacing(5);
        rotationStepLabel = new QLabel(rotationPanel);
        rotationLayout->addWidget(rotationStepLabel);
        const auto makeRotationButton = [this, rotationLayout](
            const QString& objectName, const QString& text)
            {
                QToolButton* button = new QToolButton(rotationPanel);
                button->setObjectName(objectName);
                button->setText(text);
                // 每次旋转会重新建立工件/工装场景并核对门禁，禁止按住自动
                // 连发，避免大型 STEP 的GUI事件积压和角度过冲。
                button->setAutoRepeat(false);
                rotationLayout->addWidget(button);
                return button;
            };
        workpieceRotateMinusButton = makeRotationButton(
            QStringLiteral("workpieceRotateMinusButton"), QStringLiteral("工件 ↻"));
        workpieceRotatePlusButton = makeRotationButton(
            QStringLiteral("workpieceRotatePlusButton"), QStringLiteral("工件 ↺"));
        vSlotRotateMinusButton = makeRotationButton(
            QStringLiteral("vSlotRotateMinusButton"), QStringLiteral("V槽 ↻"));
        vSlotRotatePlusButton = makeRotationButton(
            QStringLiteral("vSlotRotatePlusButton"), QStringLiteral("V槽 ↺"));
        QObject::connect(workpieceRotateMinusButton, &QToolButton::clicked, q,
            [this]()
            {
                if (workpieceRotationRequestedCallback)
                    workpieceRotationRequestedCallback(-rotationStepDegrees);
            });
        QObject::connect(workpieceRotatePlusButton, &QToolButton::clicked, q,
            [this]()
            {
                if (workpieceRotationRequestedCallback)
                    workpieceRotationRequestedCallback(rotationStepDegrees);
            });
        QObject::connect(vSlotRotateMinusButton, &QToolButton::clicked, q,
            [this]()
            {
                if (vSlotRotationRequestedCallback)
                    vSlotRotationRequestedCallback(-rotationStepDegrees);
            });
        QObject::connect(vSlotRotatePlusButton, &QToolButton::clicked, q,
            [this]()
            {
                if (vSlotRotationRequestedCallback)
                    vSlotRotationRequestedCallback(rotationStepDegrees);
            });
        rotationPanel->hide();
        RefreshRotationControls();
        SetStatus(QStringLiteral("尚未载入原始 STEP CAD 模型。"));
    }

    ~Impl()
    {
        ClearLabelWidgets();
        if (!context.IsNull()) context->RemoveAll(Standard_False);
        annotations.clear();
        vSlotObject.Nullify();
        theoreticalRobotObjects.clear();
        robotCollisionEnvelopeObjects.clear();
        modelObject.Nullify();
        if (!view.IsNull()) view->Remove();
        view.Nullify();
        nativeWindow.Nullify();
        context.Nullify();
        viewer.Nullify();
        graphicDriver.Nullify();
        displayConnection.Nullify();
    }

    void SetStatus(const QString& text)
    {
        statusText = text;
        statusLabel->setText(text);
        statusLabel->show();
        statusLabel->raise();
        if (rotationPanel != nullptr && rotationPanel->isVisible()) rotationPanel->raise();
    }

    void RefreshRotationControls()
    {
        if (rotationPanel == nullptr) return;
        const bool workpieceReady = displayedGroundSurface.visible
            && displayedGroundSurface.workpieceRotatable
            && static_cast<bool>(workpieceRotationRequestedCallback);
        const bool vSlotReady = VSlotDragEnabled()
            && static_cast<bool>(vSlotRotationRequestedCallback);
        workpieceRotateMinusButton->setEnabled(workpieceReady);
        workpieceRotatePlusButton->setEnabled(workpieceReady);
        vSlotRotateMinusButton->setEnabled(vSlotReady);
        vSlotRotatePlusButton->setEnabled(vSlotReady);
        const bool wholeStep = std::abs(
            rotationStepDegrees - std::round(rotationStepDegrees)) <= 1.0e-9;
        rotationStepLabel->setText(QStringLiteral("视图内旋转 %1°")
            .arg(rotationStepDegrees, 0, 'f', wholeStep ? 0 : 1));
        rotationPanel->setVisible(workpieceReady || vSlotReady);
        if (rotationPanel->isVisible())
        {
            rotationPanel->adjustSize();
            const int x = std::max(12, q->width() - rotationPanel->width() - 12);
            rotationPanel->move(x, 66);
            rotationPanel->raise();
        }
    }

    void ResizeStatus()
    {
        const int available = std::max(220, q->width() - 24);
        statusLabel->setGeometry(12, 12, available, 48);
        RefreshRotationControls();
        UpdateLabelPositions();
        statusLabel->raise();
    }

    void ClearLabelWidgets()
    {
        for (QLabel* widget : labelWidgets) delete widget;
        labelWidgets.clear();
        overlayLabels.clear();
    }

    void SetOverlayLabels(const QVector<OverlayLabel>& labels)
    {
        ClearLabelWidgets();
        overlayLabels = labels;
        labelWidgets.reserve(labels.size());
        for (const OverlayLabel& spec : labels)
        {
            QLabel* widget = new QLabel(spec.text, q);
            widget->setAttribute(Qt::WA_TransparentForMouseEvents, true);
            QFont font = widget->font();
            font.setBold(true);
            font.setPixelSize(spec.emphasized ? 17 : 14);
            widget->setFont(font);
            widget->setStyleSheet(QStringLiteral(
                "QLabel { color:%1; background:rgba(255,255,255,238); "
                "border:1px solid %2; border-radius:4px; padding:3px 6px; }")
                .arg(spec.color.name(), spec.color.darker(145).name()));
            widget->adjustSize();
            labelWidgets.push_back(widget);
        }
        UpdateLabelPositions();
    }

    void UpdateLabelPositions()
    {
        QVector<QRect> occupied;
        if (statusLabel != nullptr && statusLabel->isVisible())
            occupied.push_back(statusLabel->geometry().adjusted(-3, -3, 3, 3));
        if (rotationPanel != nullptr && rotationPanel->isVisible())
            occupied.push_back(rotationPanel->geometry().adjusted(-3, -3, 3, 3));
        const qsizetype count = std::min(overlayLabels.size(), labelWidgets.size());
        for (qsizetype index = 0; index < count; ++index)
        {
            QLabel* widget = labelWidgets.at(index);
            QPointF projected;
            if (!ProjectToOverlay(overlayLabels.at(index).position, projected))
            {
                widget->hide();
                continue;
            }
            widget->adjustSize();
            int x = qRound(projected.x() + 9.0);
            int y = qRound(projected.y() - widget->height() - 7.0);
            x = std::clamp(x, 4, std::max(4, q->width() - widget->width() - 4));
            y = std::clamp(y, 64, std::max(64, q->height() - widget->height() - 4));
            QRect candidate(x, y, widget->width(), widget->height());
            for (int attempt = 0; attempt < 12; ++attempt)
            {
                const bool intersects = std::any_of(
                    occupied.cbegin(), occupied.cend(),
                    [&candidate](const QRect& other)
                    {
                        return candidate.adjusted(-3, -3, 3, 3).intersects(other);
                    });
                if (!intersects) break;
                const int nextY = candidate.bottom() + 6;
                if (nextY + candidate.height() <= q->height() - 4)
                {
                    candidate.moveTop(nextY);
                }
                else
                {
                    candidate.translate(-std::max(18, candidate.width() / 3),
                        -candidate.height() - 7);
                    candidate.moveLeft(std::clamp(candidate.left(), 4,
                        std::max(4, q->width() - candidate.width() - 4)));
                    candidate.moveTop(std::clamp(candidate.top(), 64,
                        std::max(64, q->height() - candidate.height() - 4)));
                }
            }
            widget->move(candidate.topLeft());
            widget->show();
            widget->raise();
            occupied.push_back(candidate.adjusted(-3, -3, 3, 3));
        }
        statusLabel->raise();
        if (rotationPanel != nullptr && rotationPanel->isVisible()) rotationPanel->raise();
    }

    bool EnsureViewer(QString* error = nullptr)
    {
        if (initialized) return true;
        try
        {
            // Windows 原生窗口不需要 X11/Wayland 显示连接；空连接也是 OCCT
            // 官方 Qt/WNT 示例使用的构造方式。
            displayConnection.Nullify();
            graphicDriver = new OpenGl_GraphicDriver(displayConnection);
            viewer = new V3d_Viewer(graphicDriver);
            viewer->SetDefaultLights();
            viewer->SetLightOn();
            viewer->SetDefaultBackgroundColor(Quantity_Color(1.0, 1.0, 1.0, Quantity_TOC_RGB));
            context = new AIS_InteractiveContext(viewer);
            context->SetAutoActivateSelection(Standard_False);
            context->SetDisplayMode(AIS_Shaded, Standard_False);
            view = viewer->CreateView();
            boundWinId = q->winId();
            nativeWindow = new WNT_Window(
                reinterpret_cast<Aspect_Handle>(boundWinId));
            view->SetWindow(nativeWindow);
            if (!nativeWindow->IsMapped()) nativeWindow->Map();
            view->SetBackgroundColor(Quantity_Color(1.0, 1.0, 1.0, Quantity_TOC_RGB));
            view->SetProj(V3d_TypeOfOrientation_Zup_AxoRight);
            view->SetShadingModel(Graphic3d_TypeOfShadingModel_Phong);
            view->MustBeResized();
            initialized = true;
            DisplayModel(false);
            RebuildAnnotations();
            if (pendingFit) FitAll();
            // showEvent 发生时布局/高 DPI 原生 HWND 的最终尺寸仍可能尚未提交；
            // 下一轮事件再按真实 client rect 更新一次，并重新适配模型。
            ScheduleNativeResize(!shape.IsNull());
            return true;
        }
        catch (const Standard_Failure& failure)
        {
            const char* message = failure.GetMessageString();
            lastError = QStringLiteral("初始化 OCCT CAD 视图失败：%1")
                .arg(message == nullptr ? QStringLiteral("未知错误")
                                        : QString::fromLocal8Bit(message).simplified());
        }
        catch (const std::exception& exception)
        {
            lastError = QStringLiteral("初始化 CAD 视图异常：%1")
                .arg(QString::fromLocal8Bit(exception.what()).simplified());
        }
        catch (...)
        {
            lastError = QStringLiteral("初始化 CAD 视图发生未知异常。");
        }
        SetStatus(lastError);
        if (error != nullptr) *error = lastError;
        return false;
    }

    void RemoveAnnotations()
    {
        if (!context.IsNull())
        {
            if (!vSlotObject.IsNull())
            {
                context->Deactivate(vSlotObject);
                context->Remove(vSlotObject, Standard_False);
            }
            for (const Handle(AIS_InteractiveObject)& object : annotations)
                if (!object.IsNull()) context->Remove(object, Standard_False);
        }
        vSlotObject.Nullify();
        groundObject.Nullify();
        groundGridObject.Nullify();
        annotations.clear();
        displayedScanRegions = 0;
        vSlotLabelStart = -1;
        dragMode = DragMode::None;
        vSlotMovedDuringDrag = false;
        vSlotSnappedToWorkpiece = false;
        q->unsetCursor();
    }

    void AddAnnotation(
        const Handle(AIS_InteractiveObject)& object,
        int displayMode = 0,
        bool topmost = true)
    {
        if (object.IsNull() || context.IsNull()) return;
        context->Display(object, displayMode, -1, Standard_False);
        if (topmost) context->SetZLayer(object, Graphic3d_ZLayerId_Topmost);
        annotations.push_back(object);
    }

    Vec3d ModelDirectionToDisplay(const Vec3d& value) const
    {
        return {
            Dot(value, displayFrameAxisX),
            Dot(value, displayFrameAxisY),
            Dot(value, displayFrameAxisZ)
        };
    }

    Vec3d ModelToDisplay(const Vec3d& value) const
    {
        return Add(ModelDirectionToDisplay(value), displayFrameTranslation);
    }

    Vec3d DisplayDirectionToModel(const Vec3d& value) const
    {
        return Add(
            Multiply(displayFrameAxisX, value.x),
            Add(Multiply(displayFrameAxisY, value.y),
                Multiply(displayFrameAxisZ, value.z)));
    }

    Vec3d DisplayToModel(const Vec3d& value) const
    {
        return DisplayDirectionToModel(Subtract(value, displayFrameTranslation));
    }

    bool UpdateDisplayFrame(
        const GroundSurface& ground,
        const VSlotFixture& fixture)
    {
        Vec3d axisX = { 1.0, 0.0, 0.0 };
        Vec3d axisY = { 0.0, 1.0, 0.0 };
        Vec3d axisZ = { 0.0, 0.0, 1.0 };
        Vec3d normalizedX;
        Vec3d normalizedY;
        Vec3d normalizedZ;
        // 工件显示框架与 V 槽自身 yaw 必须分离。GroundSurface 的轴即使在
        // visible=false 时仍由上层提供；旧调用方未提供时再退回 V 槽轴。
        if (NormalizeGroundAxes(ground, normalizedX, normalizedY, normalizedZ))
        {
            axisX = normalizedX;
            axisY = normalizedY;
            axisZ = normalizedZ;
        }
        else if (fixture.visible
            && NormalizeFixtureAxes(fixture, normalizedX, normalizedY, normalizedZ))
        {
            axisX = normalizedX;
            axisY = normalizedY;
            axisZ = normalizedZ;
        }
        const bool changed = Length(Subtract(displayFrameAxisX, axisX)) > 1.0e-8
            || Length(Subtract(displayFrameAxisY, axisY)) > 1.0e-8
            || Length(Subtract(displayFrameAxisZ, axisZ)) > 1.0e-8;
        Vec3d pivotModel = {};
        if (fixture.visible && IsFinite(fixture.anchor)) pivotModel = fixture.anchor;
        else if (ground.visible && IsFinite(ground.planePoint)) pivotModel = ground.planePoint;
        const Vec3d fixedDisplayPivot = ModelToDisplay(pivotModel);
        displayFrameAxisX = axisX;
        displayFrameAxisY = axisY;
        displayFrameAxisZ = axisZ;
        if (changed)
        {
            displayFrameTranslation = Subtract(
                fixedDisplayPivot, ModelDirectionToDisplay(pivotModel));
        }
        return changed;
    }

    TopLoc_Location ModelDisplayLocation() const
    {
        gp_Trsf transform;
        transform.SetValues(
            displayFrameAxisX.x, displayFrameAxisX.y, displayFrameAxisX.z,
                displayFrameTranslation.x,
            displayFrameAxisY.x, displayFrameAxisY.y, displayFrameAxisY.z,
                displayFrameTranslation.y,
            displayFrameAxisZ.x, displayFrameAxisZ.y, displayFrameAxisZ.z,
                displayFrameTranslation.z);
        return TopLoc_Location(transform);
    }

    void ApplyModelDisplayFrame()
    {
        if (context.IsNull() || modelObject.IsNull()) return;
        context->SetLocation(modelObject, ModelDisplayLocation());
    }

    bool ShapeBoundsAtLocation(
        const TopoDS_Shape& source,
        const TopLoc_Location& location,
        Vec3d& minimum,
        Vec3d& maximum) const
    {
        minimum = {};
        maximum = {};
        if (source.IsNull()) return false;
        try
        {
            const TopoDS_Shape located = source.Moved(location, Standard_True);
            Bnd_Box bounds;
            bounds.SetGap(0.0);
            BRepBndLib::AddOptimal(located, bounds, Standard_False, Standard_False);
            if (bounds.IsVoid() || bounds.IsOpen()) return false;
            Standard_Real xMin = 0.0;
            Standard_Real yMin = 0.0;
            Standard_Real zMin = 0.0;
            Standard_Real xMax = 0.0;
            Standard_Real yMax = 0.0;
            Standard_Real zMax = 0.0;
            bounds.Get(xMin, yMin, zMin, xMax, yMax, zMax);
            minimum = { xMin, yMin, zMin };
            maximum = { xMax, yMax, zMax };
            return IsFinite(minimum) && IsFinite(maximum)
                && minimum.x <= maximum.x
                && minimum.y <= maximum.y
                && minimum.z <= maximum.z;
        }
        catch (...)
        {
            return false;
        }
    }

    bool HasActiveRobotPresentation() const
    {
        if (robotPresentationKind == RobotPresentationKind::CollisionEnvelope)
            return robotCollisionEnvelopeObjects.size() == 7;
        return robotPresentationKind == RobotPresentationKind::DetailedBRep
            && !theoreticalRobotShape.IsNull();
    }

    bool ResolveTheoreticalRobotLocation(TopLoc_Location& location)
    {
        robotBaseDisplay = {};
        robotLabelPosition = {};
        if (!theoreticalRobotVisibleRequested
            || !HasActiveRobotPresentation()
            || shape.IsNull()
            || !displayedGroundSurface.visible
            || !robotPlacementMetadata.valid)
        {
            return false;
        }

        Vec3d groundX;
        Vec3d groundY;
        Vec3d groundZ;
        if (!NormalizeGroundAxes(displayedGroundSurface, groundX, groundY, groundZ)
            || Dot(groundZ, { 0.0, 0.0, 1.0 }) < 0.99999)
        {
            return false;
        }

        Vec3d workpieceMinimum;
        Vec3d workpieceMaximum;
        if (!ShapeBoundsAtLocation(
                shape, ModelDisplayLocation(), workpieceMinimum, workpieceMaximum))
        {
            return false;
        }

        const RobotCollisionEnvelopeStore::AxisAlignedBounds& assemblyBounds =
            robotPlacementMetadata.sourceBoundsMm;
        const double sourceBaseX = robotPlacementMetadata.sourceBaseCenterMm.x();
        const double sourceBaseZ = robotPlacementMetadata.sourceBaseCenterMm.z();
        // 机器人应以 J0 安装面落地，而不是把任意关节的最低点误当成基座。
        const double sourceFloorY = robotPlacementMetadata.sourceFloorYmm;

        // 厂商 SA10 总装以 +Y 为竖直方向。先映射为应用 +Z，再绕地面 Z
        // 旋转 -90°，使默认姿态的手臂朝向位于机器人 +X 侧的工件：
        //   (x,y,z) -> (-z+bz, -x+bx, y-floorY)
        // 读取器已经在后台给出有限总装包围盒。此固定正交映射可以直接
        // 变换其区间，避免每次 overlay 刷新都在 GUI 线程重新遍历 2.6 万面。
        const Vec3d normalizedMinimum = {
            -assemblyBounds.maximumMm.z() + sourceBaseZ,
            -assemblyBounds.maximumMm.x() + sourceBaseX,
             assemblyBounds.minimumMm.y() - sourceFloorY
        };
        const Vec3d normalizedMaximum = {
            -assemblyBounds.minimumMm.z() + sourceBaseZ,
            -assemblyBounds.minimumMm.x() + sourceBaseX,
             assemblyBounds.maximumMm.y() - sourceFloorY
        };
        if (!IsFinite(normalizedMinimum) || !IsFinite(normalizedMaximum)) return false;

        constexpr double kWorkpieceClearanceMm = 600.0;
        const double translateX = workpieceMinimum.x
            - kWorkpieceClearanceMm - normalizedMaximum.x;
        const double translateY = 0.5 * (workpieceMinimum.y + workpieceMaximum.y)
            - 0.5 * (normalizedMinimum.y + normalizedMaximum.y);
        const double translateZ = displayedGroundSurface.planePoint.z;
        if (!std::isfinite(translateX) || !std::isfinite(translateY)
            || !std::isfinite(translateZ))
        {
            return false;
        }

        gp_Trsf sceneTransform;
        sceneTransform.SetValues(
             0.0, 0.0, -1.0, sourceBaseZ + translateX,
            -1.0, 0.0,  0.0, sourceBaseX + translateY,
             0.0, 1.0,  0.0, -sourceFloorY + translateZ);
        location = TopLoc_Location(sceneTransform);
        robotBaseDisplay = { translateX, translateY, translateZ };
        robotLabelPosition = {
            translateX + 0.5 * (normalizedMinimum.x + normalizedMaximum.x),
            translateY + 0.5 * (normalizedMinimum.y + normalizedMaximum.y),
            translateZ + normalizedMaximum.z
        };
        return IsFinite(robotBaseDisplay) && IsFinite(robotLabelPosition);
    }

    QString TheoreticalRobotLabelText() const
    {
        if (robotPresentationKind == RobotPresentationKind::CollisionEnvelope)
            return QStringLiteral("机器人碰撞简模（静态 / 未标定）");
        const qsizetype total = static_cast<qsizetype>(
            theoreticalRobotLoadResult.displayBlocks.size());
        if (theoreticalRobotDisplayComplete)
            return QStringLiteral("理论机器人（静态位置 / 未标定）");
        return QStringLiteral("理论机器人加载中 %1/%2（静态 / 未标定）")
            .arg(theoreticalRobotNextDisplayBlock)
            .arg(total);
    }

    void UpdateTheoreticalRobotProgressLabel()
    {
        if (theoreticalRobotLabelIndex < 0
            || theoreticalRobotLabelIndex >= overlayLabels.size()
            || theoreticalRobotLabelIndex >= labelWidgets.size())
        {
            return;
        }
        const QString text = TheoreticalRobotLabelText();
        overlayLabels[theoreticalRobotLabelIndex].text = text;
        QLabel* label = labelWidgets[theoreticalRobotLabelIndex];
        label->setText(text);
        label->adjustSize();
        UpdateLabelPositions();
    }

    void CancelTheoreticalRobotPresentation()
    {
        ++theoreticalRobotDisplayGeneration;
        theoreticalRobotBlockScheduled = false;
        if (!context.IsNull())
        {
            for (const Handle(AIS_Shape)& object : theoreticalRobotObjects)
            {
                if (!object.IsNull()) context->Remove(object, Standard_False);
            }
        }
        theoreticalRobotObjects.clear();
        theoreticalRobotNextDisplayBlock = 0;
        theoreticalRobotDisplayComplete = false;
        theoreticalRobotPresentationInitialized = false;
        theoreticalRobotDisplayed = false;
        theoreticalRobotPlacementResolved = false;
        theoreticalRobotLabelIndex = -1;
        theoreticalRobotFitFirstBlock = false;
        theoreticalRobotFitCompletion = false;
    }

    void ScheduleNextTheoreticalRobotBlock()
    {
        if (theoreticalRobotBlockScheduled || theoreticalRobotDisplayComplete
            || theoreticalRobotNextDisplayBlock
                >= theoreticalRobotLoadResult.displayBlocks.size())
        {
            return;
        }
        theoreticalRobotBlockScheduled = true;
        const quint64 generation = theoreticalRobotDisplayGeneration;
        QTimer::singleShot(0, q, [this, generation]()
        {
            if (generation != theoreticalRobotDisplayGeneration) return;
            theoreticalRobotBlockScheduled = false;
            BuildNextTheoreticalRobotBlock(generation);
        });
    }

    void BuildNextTheoreticalRobotBlock(quint64 generation)
    {
        if (generation != theoreticalRobotDisplayGeneration
            || robotPresentationKind != RobotPresentationKind::DetailedBRep
            || !initialized || context.IsNull())
        {
            return;
        }
        TopLoc_Location location;
        if (!ResolveTheoreticalRobotLocation(location)) return;
        theoreticalRobotLocation = location;
        theoreticalRobotPlacementResolved = true;
        if (theoreticalRobotNextDisplayBlock
            >= theoreticalRobotLoadResult.displayBlocks.size())
        {
            theoreticalRobotDisplayComplete = true;
            UpdateTheoreticalRobotProgressLabel();
            return;
        }

        const std::shared_ptr<TopoDS_Shape>& block =
            theoreticalRobotLoadResult.displayBlocks[theoreticalRobotNextDisplayBlock];
        if (!block || block->IsNull())
        {
            ++theoreticalRobotDisplayGeneration;
            theoreticalRobotBlockScheduled = false;
            lastError = QStringLiteral("理论机器人显示块为空，已停止渐进显示。");
            SetStatus(lastError);
            return;
        }

        try
        {
            if (!theoreticalRobotPresentationInitialized)
            {
                theoreticalRobotPresentationInitialized = true;
                ++theoreticalRobotPresentationBuildCount;
            }
            Handle(AIS_Shape) object = new AIS_Shape(*block);
            object->SetColor(Quantity_Color(0.24, 0.48, 0.70, Quantity_TOC_RGB));
            object->Attributes()->SetFaceBoundaryDraw(Standard_False);
            object->Attributes()->SetDeviationCoefficient(0.0015);
            object->Attributes()->SetDeviationAngle(0.28);
            context->SetMaterial(
                object,
                Graphic3d_MaterialAspect(Graphic3d_NOM_SATIN),
                Standard_False);
            context->SetLocation(object, theoreticalRobotLocation);
            context->Display(object, AIS_Shaded, -1, Standard_False);
            context->Deactivate(object);
            theoreticalRobotObjects.push_back(object);
            ++theoreticalRobotNextDisplayBlock;
            theoreticalRobotDisplayed = true;
            theoreticalRobotDisplayComplete = theoreticalRobotNextDisplayBlock
                == theoreticalRobotLoadResult.displayBlocks.size();
            UpdateTheoreticalRobotProgressLabel();
            context->UpdateCurrentViewer();
            if (theoreticalRobotFitFirstBlock)
            {
                theoreticalRobotFitFirstBlock = false;
                FitAll();
            }
            else if (theoreticalRobotDisplayComplete && theoreticalRobotFitCompletion)
            {
                theoreticalRobotFitCompletion = false;
                FitAll();
            }
            else if (!view.IsNull())
            {
                view->Redraw();
                UpdateLabelPositions();
            }
        }
        catch (const Standard_Failure& failure)
        {
            ++theoreticalRobotDisplayGeneration;
            theoreticalRobotBlockScheduled = false;
            lastError = QStringLiteral("理论机器人渐进显示失败：%1")
                .arg(failure.GetMessageString() == nullptr
                    ? QStringLiteral("未知 OCCT 异常")
                    : QString::fromLocal8Bit(failure.GetMessageString()).simplified());
            SetStatus(lastError);
            return;
        }
        catch (...)
        {
            ++theoreticalRobotDisplayGeneration;
            theoreticalRobotBlockScheduled = false;
            lastError = QStringLiteral("理论机器人渐进显示发生未知异常。");
            SetStatus(lastError);
            return;
        }
        if (!theoreticalRobotDisplayComplete)
            ScheduleNextTheoreticalRobotBlock();
    }

    void DisplayTheoreticalRobot()
    {
        theoreticalRobotDisplayed = false;
        theoreticalRobotPlacementResolved = false;
        if (!initialized || context.IsNull()) return;

        TopLoc_Location location;
        if (!ResolveTheoreticalRobotLocation(location))
        {
            for (const Handle(AIS_Shape)& object : theoreticalRobotObjects)
            {
                if (!object.IsNull() && context->IsDisplayed(object))
                    context->Erase(object, Standard_False);
            }
            for (const Handle(AIS_Shape)& object : robotCollisionEnvelopeObjects)
            {
                if (!object.IsNull() && context->IsDisplayed(object))
                    context->Erase(object, Standard_False);
            }
            return;
        }
        theoreticalRobotLocation = location;
        theoreticalRobotPlacementResolved = true;
        if (robotPresentationKind == RobotPresentationKind::CollisionEnvelope)
        {
            for (const Handle(AIS_Shape)& object : theoreticalRobotObjects)
            {
                if (!object.IsNull() && context->IsDisplayed(object))
                    context->Erase(object, Standard_False);
            }
            theoreticalRobotDisplayed = true;
            for (const Handle(AIS_Shape)& object : robotCollisionEnvelopeObjects)
            {
                if (object.IsNull())
                {
                    theoreticalRobotDisplayed = false;
                    continue;
                }
                context->SetLocation(object, theoreticalRobotLocation);
                if (!context->IsDisplayed(object))
                    context->Display(object, AIS_Shaded, -1, Standard_False);
                context->Deactivate(object);
            }
            theoreticalRobotDisplayComplete = theoreticalRobotDisplayed;
            theoreticalRobotNextDisplayBlock = theoreticalRobotDisplayed ? 7 : 0;
            return;
        }
        for (const Handle(AIS_Shape)& object : robotCollisionEnvelopeObjects)
        {
            if (!object.IsNull() && context->IsDisplayed(object))
                context->Erase(object, Standard_False);
        }
        for (const Handle(AIS_Shape)& object : theoreticalRobotObjects)
        {
            if (object.IsNull()) continue;
            context->SetLocation(object, theoreticalRobotLocation);
            if (!context->IsDisplayed(object))
                context->Display(object, AIS_Shaded, -1, Standard_False);
            context->Deactivate(object);
            theoreticalRobotDisplayed = true;
        }
        ScheduleNextTheoreticalRobotBlock();
    }

    VSlotFixture DisplayFixture() const
    {
        VSlotFixture display = vSlotFixture;
        display.anchor = ModelToDisplay(vSlotFixture.anchor);
        display.axisX = ModelDirectionToDisplay(vSlotFixture.axisX);
        display.axisY = ModelDirectionToDisplay(vSlotFixture.axisY);
        display.axisZ = ModelDirectionToDisplay(vSlotFixture.axisZ);
        return display;
    }

    GroundSurface DisplayGroundSurface() const
    {
        GroundSurface display = groundSurface;
        display.planePoint = ModelToDisplay(groundSurface.planePoint);
        display.axisX = ModelDirectionToDisplay(groundSurface.axisX);
        display.axisY = ModelDirectionToDisplay(groundSurface.axisY);
        display.axisZ = ModelDirectionToDisplay(groundSurface.axisZ);
        return display;
    }

    bool TheoreticalRobotSceneBounds(Vec3d& minimum, Vec3d& maximum) const
    {
        minimum = {};
        maximum = {};
        if (!robotPlacementMetadata.valid) return false;
        const RobotCollisionEnvelopeStore::AxisAlignedBounds& source =
            robotPlacementMetadata.sourceBoundsMm;
        minimum = {
            (std::numeric_limits<double>::max)(),
            (std::numeric_limits<double>::max)(),
            (std::numeric_limits<double>::max)()
        };
        maximum = {
            (std::numeric_limits<double>::lowest)(),
            (std::numeric_limits<double>::lowest)(),
            (std::numeric_limits<double>::lowest)()
        };
        try
        {
            for (const double x : { source.minimumMm.x(), source.maximumMm.x() })
            {
                for (const double y : { source.minimumMm.y(), source.maximumMm.y() })
                {
                    for (const double z : { source.minimumMm.z(), source.maximumMm.z() })
                    {
                        const gp_Pnt point = gp_Pnt(x, y, z).Transformed(
                            theoreticalRobotLocation.Transformation());
                        minimum.x = std::min(minimum.x, point.X());
                        minimum.y = std::min(minimum.y, point.Y());
                        minimum.z = std::min(minimum.z, point.Z());
                        maximum.x = std::max(maximum.x, point.X());
                        maximum.y = std::max(maximum.y, point.Y());
                        maximum.z = std::max(maximum.z, point.Z());
                    }
                }
            }
        }
        catch (...)
        {
            return false;
        }
        return IsFinite(minimum) && IsFinite(maximum);
    }

    bool ResolveGroundLayout(
        Vec3d& center,
        double& widthMm,
        double& depthMm,
        double& gridSpacingMm) const
    {
        Vec3d axisX;
        Vec3d axisY;
        Vec3d axisZ;
        if (!displayedGroundSurface.visible
            || !NormalizeGroundAxes(displayedGroundSurface, axisX, axisY, axisZ))
        {
            return false;
        }

        double minimumU = (std::numeric_limits<double>::max)();
        double maximumU = (std::numeric_limits<double>::lowest)();
        double minimumV = (std::numeric_limits<double>::max)();
        double maximumV = (std::numeric_limits<double>::lowest)();
        const auto addPoint = [&](const Vec3d& point)
        {
            if (!IsFinite(point)) return;
            const double u = Dot(point, axisX);
            const double v = Dot(point, axisY);
            minimumU = std::min(minimumU, u);
            maximumU = std::max(maximumU, u);
            minimumV = std::min(minimumV, v);
            maximumV = std::max(maximumV, v);
        };

        if (!shape.IsNull())
        {
            try
            {
                const TopoDS_Shape displayedShape = shape.Moved(
                    ModelDisplayLocation(), Standard_True);
                Bnd_Box bounds;
                bounds.SetGap(0.0);
                BRepBndLib::AddOptimal(
                    displayedShape, bounds, Standard_False, Standard_False);
                if (!bounds.IsVoid() && !bounds.IsOpen())
                {
                    Standard_Real xMin = 0.0;
                    Standard_Real yMin = 0.0;
                    Standard_Real zMin = 0.0;
                    Standard_Real xMax = 0.0;
                    Standard_Real yMax = 0.0;
                    Standard_Real zMax = 0.0;
                    bounds.Get(xMin, yMin, zMin, xMax, yMax, zMax);
                    for (const double x : { static_cast<double>(xMin), static_cast<double>(xMax) })
                    {
                        for (const double y : { static_cast<double>(yMin), static_cast<double>(yMax) })
                        {
                            for (const double z : { static_cast<double>(zMin), static_cast<double>(zMax) })
                                addPoint({ x, y, z });
                        }
                    }
                }
            }
            catch (...)
            {
                // 地面只是外显辅助；包围盒失败时仍可用 V 槽尺寸生成有限地板。
            }
        }

        if (theoreticalRobotPlacementResolved && HasActiveRobotPresentation())
        {
            Vec3d robotMinimum;
            Vec3d robotMaximum;
            if (TheoreticalRobotSceneBounds(robotMinimum, robotMaximum))
            {
                for (const double x : { robotMinimum.x, robotMaximum.x })
                {
                    for (const double y : { robotMinimum.y, robotMaximum.y })
                    {
                        for (const double z : { robotMinimum.z, robotMaximum.z })
                            addPoint({ x, y, z });
                    }
                }
            }
        }

        addPoint(displayedVSlotFixture.anchor);
        addPoint(Add(displayedVSlotFixture.anchor,
            Multiply(displayedVSlotFixture.axisX, displayedVSlotFixture.longLengthMm)));
        addPoint(Add(displayedVSlotFixture.anchor,
            Multiply(displayedVSlotFixture.axisY, displayedVSlotFixture.shortLengthMm)));
        if (!(minimumU <= maximumU) || !(minimumV <= maximumV)) return false;

        const double rawWidth = std::max(1.0, maximumU - minimumU);
        const double rawDepth = std::max(1.0, maximumV - minimumV);
        const double referenceSpan = std::max(rawWidth, rawDepth);
        const double margin = std::clamp(referenceSpan * 0.10, 180.0, 900.0);
        widthMm = std::max(1200.0, rawWidth + margin * 2.0);
        depthMm = std::max(1200.0, rawDepth + margin * 2.0);
        const double middleU = (minimumU + maximumU) * 0.5;
        const double middleV = (minimumV + maximumV) * 0.5;
        center = Add(displayedGroundSurface.planePoint,
            Add(Multiply(axisX, middleU - Dot(displayedGroundSurface.planePoint, axisX)),
                Multiply(axisY, middleV - Dot(displayedGroundSurface.planePoint, axisY))));

        gridSpacingMm = 100.0;
        while (std::max(widthMm, depthMm) / gridSpacingMm > 32.0
            && gridSpacingMm < 5000.0)
        {
            gridSpacingMm *= gridSpacingMm < 500.0 ? 2.0 : 2.5;
        }
        return IsFinite(center) && std::isfinite(widthMm)
            && std::isfinite(depthMm) && std::isfinite(gridSpacingMm);
    }

    bool VSlotDragEnabled() const
    {
        return displayedVSlotFixture.visible
            && displayedVSlotFixture.draggable
            && displayedGroundSurface.visible
            && displayedGroundSurface.supportCandidateIndex >= 0
            && displayedGroundSurface.supportCandidateIndex
                < static_cast<int>(groundFaceRecords.size());
    }

    bool EvaluateWorkpieceSnapCorner(
        const Vec3d& proposedDisplayAnchor,
        const Vec3d& proposedAxisX,
        const Vec3d& proposedAxisY,
        const Vec3d& proposedAxisZ,
        Vec3d& closestDisplayPoint,
        double& closestDistance) const
    {
        closestDisplayPoint = {};
        closestDistance = (std::numeric_limits<double>::max)();
        if (!VSlotDragEnabled() || !IsFinite(proposedDisplayAnchor)) return false;
        const int candidateIndex = displayedGroundSurface.supportCandidateIndex;
        const QVector<Vec3d> modelPoints = ExtractGroundSnapPoints(
            groundFaceRecords[static_cast<size_t>(candidateIndex)]);
        if (modelPoints.size() < 3) return false;
        Vec3d normal;
        if (!Normalize(displayedGroundSurface.axisZ, normal)) return false;
        Vec3d fixtureAxisX;
        Vec3d fixtureAxisY;
        Vec3d fixtureAxisZ;
        VSlotFixture proposedFixture = displayedVSlotFixture;
        proposedFixture.axisX = proposedAxisX;
        proposedFixture.axisY = proposedAxisY;
        proposedFixture.axisZ = proposedAxisZ;
        if (!NormalizeFixtureAxes(
                proposedFixture, fixtureAxisX, fixtureAxisY, fixtureAxisZ)
            || Dot(fixtureAxisZ, normal) < 0.99999) return false;
        const Vec3d projectedAnchor = ProjectToPlane(
            proposedDisplayAnchor, displayedGroundSurface.planePoint, normal);
        struct FixturePoint2d
        {
            double u = 0.0;
            double v = 0.0;
        };
        QVector<FixturePoint2d> fixturePoints;
        fixturePoints.reserve(modelPoints.size());
        double minimumU = (std::numeric_limits<double>::max)();
        double minimumV = (std::numeric_limits<double>::max)();
        for (const Vec3d& modelPoint : modelPoints)
        {
            const Vec3d displayPoint = ProjectToPlane(
                ModelToDisplay(modelPoint), displayedGroundSurface.planePoint, normal);
            const FixturePoint2d point{
                Dot(displayPoint, fixtureAxisX), Dot(displayPoint, fixtureAxisY)
            };
            if (!std::isfinite(point.u) || !std::isfinite(point.v)) continue;
            fixturePoints.push_back(point);
            minimumU = std::min(minimumU, point.u);
            minimumV = std::min(minimumV, point.v);
        }
        if (fixturePoints.size() < 3
            || !std::isfinite(minimumU) || !std::isfinite(minimumV)) return false;

        // V 槽内角的开口固定指向 +X/+Y。用支撑面外轮廓在这两个
        // 方向的最小支撑值构造“虚拟外角”：这样所选落地面外轮廓都位于
        // 槽体 +X/+Y 开口内，不会因选错象限而穿过底面轮廓。对倒角工件，虚拟外角
        // 位于两条支撑边延长线交点，比把 anchor 强行放到倒角顶点更符合实物。
        const double supportTolerance = groundFaceRecords[static_cast<size_t>(
            candidateIndex)].summary.supportToleranceMm;
        const double contactTolerance = std::clamp(
            std::max(0.5, supportTolerance * 2.0), 0.5, 2.0);
        const double minimumIndependentContact = std::max(
            contactTolerance * 2.0, displayedVSlotFixture.wallThicknessMm);
        bool contactsLongXWall = false;
        bool contactsShortYWall = false;
        for (const FixturePoint2d& point : fixturePoints)
        {
            const double localX = point.u - minimumU;
            const double localY = point.v - minimumV;
            if (std::abs(point.v - minimumV) <= contactTolerance
                && localX >= minimumIndependentContact
                && localX <= displayedVSlotFixture.longLengthMm + contactTolerance)
            {
                contactsLongXWall = true;
            }
            if (std::abs(point.u - minimumU) <= contactTolerance
                && localY >= minimumIndependentContact
                && localY <= displayedVSlotFixture.shortLengthMm + contactTolerance)
            {
                contactsShortYWall = true;
            }
        }
        // 凸包只用于支撑极值，绝不能把相邻凸包点当成真实材料边：凹形或
        // 多个不相连共面脚会产生跨空隙的虚拟桥。有限墙区间重叠只能由
        // 原始 B-Rep edge（曲线为同一edge内的连续采样段）证明。
        const QVector<GroundSnapSegment> realSegments = ExtractGroundSnapSegments(
            groundFaceRecords[static_cast<size_t>(candidateIndex)]);
        for (const GroundSnapSegment& segment : realSegments)
        {
            const Vec3d startDisplay = ProjectToPlane(
                ModelToDisplay(segment.startModel),
                displayedGroundSurface.planePoint,
                normal);
            const Vec3d endDisplay = ProjectToPlane(
                ModelToDisplay(segment.endModel),
                displayedGroundSurface.planePoint,
                normal);
            const FixturePoint2d start{
                Dot(startDisplay, fixtureAxisX), Dot(startDisplay, fixtureAxisY)
            };
            const FixturePoint2d end{
                Dot(endDisplay, fixtureAxisX), Dot(endDisplay, fixtureAxisY)
            };
            if (std::abs(start.v - minimumV) <= contactTolerance
                && std::abs(end.v - minimumV) <= contactTolerance)
            {
                const double overlapStart = std::max(
                    minimumIndependentContact,
                    std::min(start.u, end.u) - minimumU);
                const double overlapEnd = std::min(
                    displayedVSlotFixture.longLengthMm,
                    std::max(start.u, end.u) - minimumU);
                if (overlapEnd - overlapStart > contactTolerance)
                    contactsLongXWall = true;
            }
            if (std::abs(start.u - minimumU) <= contactTolerance
                && std::abs(end.u - minimumU) <= contactTolerance)
            {
                const double overlapStart = std::max(
                    minimumIndependentContact,
                    std::min(start.v, end.v) - minimumV);
                const double overlapEnd = std::min(
                    displayedVSlotFixture.shortLengthMm,
                    std::max(start.v, end.v) - minimumV);
                if (overlapEnd - overlapStart > contactTolerance)
                    contactsShortYWall = true;
            }
        }
        if (!contactsLongXWall || !contactsShortYWall) return false;

        const double planeU = Dot(displayedGroundSurface.planePoint, fixtureAxisX);
        const double planeV = Dot(displayedGroundSurface.planePoint, fixtureAxisY);
        closestDisplayPoint = Add(displayedGroundSurface.planePoint,
            Add(Multiply(fixtureAxisX, minimumU - planeU),
                Multiply(fixtureAxisY, minimumV - planeV)));
        closestDistance = Length(Subtract(closestDisplayPoint, projectedAnchor));
        return IsFinite(closestDisplayPoint) && std::isfinite(closestDistance);
    }

    bool ClosestWorkpieceSnapCorner(
        const Vec3d& proposedDisplayAnchor,
        Vec3d& closestDisplayPoint,
        double& closestDistance) const
    {
        return EvaluateWorkpieceSnapCorner(
            proposedDisplayAnchor,
            displayedVSlotFixture.axisX,
            displayedVSlotFixture.axisY,
            displayedVSlotFixture.axisZ,
            closestDisplayPoint,
            closestDistance);
    }

    struct VSlotSnapPose
    {
        Vec3d anchor;
        Vec3d axisX;
        Vec3d axisY;
        Vec3d axisZ;
        double distanceMm = (std::numeric_limits<double>::max)();
        double angleDegrees = (std::numeric_limits<double>::max)();
        bool valid = false;
    };

    bool ClosestWorkpieceSnapPose(
        const Vec3d& proposedDisplayAnchor,
        VSlotSnapPose& best) const
    {
        best = {};
        if (!VSlotDragEnabled() || !IsFinite(proposedDisplayAnchor)) return false;
        Vec3d currentX;
        Vec3d currentY;
        Vec3d normal;
        if (!NormalizeFixtureAxes(
                displayedVSlotFixture, currentX, currentY, normal)) return false;

        const double snapDistance = std::clamp(
            displayedGroundSurface.snapDistanceMm, 5.0, 250.0);
        constexpr double kOrientationCaptureDegrees = 12.0;
        const auto consider = [&](const Vec3d& candidateX, const Vec3d& candidateY,
            double angleDegrees)
            {
                Vec3d anchor;
                double distance = 0.0;
                if (!EvaluateWorkpieceSnapCorner(
                        proposedDisplayAnchor,
                        candidateX,
                        candidateY,
                        normal,
                        anchor,
                        distance)
                    || distance > snapDistance
                    || !std::isfinite(angleDegrees))
                {
                    return;
                }
                // 位置优先，距离基本相同时保持最接近当前朝向的候选，防止
                // 对称工件在相邻90度姿态间跳动。
                const bool better = !best.valid
                    || distance < best.distanceMm - 0.5
                    || (std::abs(distance - best.distanceMm) <= 0.5
                        && angleDegrees < best.angleDegrees);
                if (!better) return;
                best.anchor = anchor;
                best.axisX = candidateX;
                best.axisY = candidateY;
                best.axisZ = normal;
                best.distanceMm = distance;
                best.angleDegrees = angleDegrees;
                best.valid = true;
            };

        // 已经精确对齐的姿态优先保留。
        consider(currentX, currentY, 0.0);

        const int candidateIndex = displayedGroundSurface.supportCandidateIndex;
        if (candidateIndex < 0
            || candidateIndex >= static_cast<int>(groundFaceRecords.size()))
        {
            return best.valid;
        }
        const QVector<Vec3d> modelPoints = ExtractGroundSnapPoints(
            groundFaceRecords[static_cast<size_t>(candidateIndex)]);
        if (modelPoints.size() < 3) return best.valid;
        for (qsizetype index = 0; index < modelPoints.size(); ++index)
        {
            const Vec3d start = ProjectToPlane(
                ModelToDisplay(modelPoints.at(index)),
                displayedGroundSurface.planePoint,
                normal);
            const Vec3d end = ProjectToPlane(
                ModelToDisplay(modelPoints.at((index + 1) % modelPoints.size())),
                displayedGroundSurface.planePoint,
                normal);
            Vec3d tangent;
            if (!Normalize(Subtract(end, start), tangent)) continue;
            for (const double sign : { 1.0, -1.0 })
            {
                const Vec3d candidateX = Multiply(tangent, sign);
                Vec3d candidateY;
                if (!Normalize(Cross(normal, candidateX), candidateY)) continue;
                const double cosine = std::clamp(Dot(currentX, candidateX), -1.0, 1.0);
                const double angleDegrees = std::acos(cosine) * 180.0 / std::acos(-1.0);
                if (angleDegrees <= kOrientationCaptureDegrees)
                    consider(candidateX, candidateY, angleDegrees);
            }
        }
        return best.valid;
    }

    bool AnchorMatchesWorkpieceSnapCorner(const Vec3d& displayAnchor) const
    {
        Vec3d closestPoint;
        double closestDistance = 0.0;
        if (!ClosestWorkpieceSnapCorner(
                displayAnchor, closestPoint, closestDistance)) return false;
        const int candidateIndex = displayedGroundSurface.supportCandidateIndex;
        const double supportTolerance = groundFaceRecords[static_cast<size_t>(
            candidateIndex)].summary.supportToleranceMm;
        // 这里是“已接触”复核公差，不是鼠标磁吸的捕获距离。
        const double contactTolerance = std::clamp(
            std::max(0.5, supportTolerance * 2.0), 0.5, 2.0);
        Vec3d normal;
        if (!Normalize(displayedGroundSurface.axisZ, normal)) return false;
        const double planeDistance = std::abs(Dot(
            Subtract(displayAnchor, displayedGroundSurface.planePoint), normal));
        return planeDistance <= contactTolerance
            && closestDistance <= contactTolerance;
    }

    ScanRegion DisplayScanRegion(const ScanRegion& source) const
    {
        ScanRegion display = source;
        display.center = ModelToDisplay(source.center);
        display.outwardNormal = ModelDirectionToDisplay(source.outwardNormal);
        return display;
    }

    GuideSegment DisplayGuide(const GuideSegment& source) const
    {
        GuideSegment display = source;
        display.start = ModelToDisplay(source.start);
        display.end = ModelToDisplay(source.end);
        return display;
    }

    StationMarker DisplayMarker(const StationMarker& source) const
    {
        StationMarker display = source;
        display.position = ModelToDisplay(source.position);
        return display;
    }

    void RebuildAnnotations()
    {
        if (!initialized || context.IsNull()) return;
        RemoveAnnotations();
        theoreticalRobotLabelIndex = -1;
        QVector<OverlayLabel> labels;
        displayedVSlotFixture = DisplayFixture();
        displayedGroundSurface = DisplayGroundSurface();
        vSlotSnappedToWorkpiece = AnchorMatchesWorkpieceSnapCorner(
            displayedVSlotFixture.anchor);
        DisplayTheoreticalRobot();
        if (theoreticalRobotPlacementResolved)
        {
            theoreticalRobotLabelIndex = labels.size();
            labels.push_back({
                robotLabelPosition,
                TheoreticalRobotLabelText(),
                robotPresentationKind == RobotPresentationKind::CollisionEnvelope
                    ? QColor(211, 67, 35) : QColor(38, 93, 145),
                false });
        }

        Vec3d groundCenter;
        double groundWidth = 0.0;
        double groundDepth = 0.0;
        double groundGridSpacing = 0.0;
        if (ResolveGroundLayout(
                groundCenter, groundWidth, groundDepth, groundGridSpacing))
        {
            const TopoDS_Shape slabShape = BuildGroundSlab(
                displayedGroundSurface, groundCenter, groundWidth, groundDepth);
            if (!slabShape.IsNull())
            {
                groundObject = new AIS_Shape(slabShape);
                groundObject->SetColor(ToOcctColor(displayedGroundSurface.color));
                groundObject->SetTransparency(0.16);
                groundObject->Attributes()->SetFaceBoundaryDraw(Standard_True);
                groundObject->Attributes()->SetFaceBoundaryAspect(new Prs3d_LineAspect(
                    Quantity_Color(0.48, 0.52, 0.55, Quantity_TOC_RGB),
                    Aspect_TOL_SOLID,
                    1.2));
                context->SetMaterial(
                    groundObject,
                    Graphic3d_MaterialAspect(Graphic3d_NOM_PLASTER),
                    Standard_False);
                AddAnnotation(groundObject, AIS_Shaded, false);
            }
            const TopoDS_Compound gridShape = BuildGroundGrid(
                displayedGroundSurface,
                groundCenter,
                groundWidth,
                groundDepth,
                groundGridSpacing);
            if (!gridShape.IsNull())
            {
                groundGridObject = new AIS_Shape(gridShape);
                groundGridObject->Attributes()->SetWireAspect(new Prs3d_LineAspect(
                    Quantity_Color(0.58, 0.61, 0.63, Quantity_TOC_RGB),
                    Aspect_TOL_SOLID,
                    1.0));
                groundGridObject->Attributes()->SetLineAspect(new Prs3d_LineAspect(
                    Quantity_Color(0.58, 0.61, 0.63, Quantity_TOC_RGB),
                    Aspect_TOL_SOLID,
                    1.0));
                AddAnnotation(groundGridObject, AIS_WireFrame, false);
            }
        }

        const TopoDS_Shape fixtureShape = BuildVSlotShape(displayedVSlotFixture);
        if (!fixtureShape.IsNull())
        {
            vSlotObject = new AIS_Shape(fixtureShape);
            vSlotObject->SetColor(vSlotSnappedToWorkpiece
                ? Quantity_Color(0.16, 0.68, 0.34, Quantity_TOC_RGB)
                : ToOcctColor(displayedVSlotFixture.color));
            vSlotObject->SetTransparency(0.04);
            vSlotObject->Attributes()->SetFaceBoundaryDraw(Standard_True);
            vSlotObject->Attributes()->SetFaceBoundaryAspect(new Prs3d_LineAspect(
                Quantity_Color(0.36, 0.20, 0.02, Quantity_TOC_RGB),
                Aspect_TOL_SOLID,
                1.6));
            context->SetMaterial(
                vSlotObject,
                Graphic3d_MaterialAspect(Graphic3d_NOM_SATIN),
                Standard_False);
            context->Display(vSlotObject, AIS_Shaded, 0, Standard_False);
            // 继承模型深度但在主模型之后绘制，确保与 CAD 相交时仍能辨认实体。
            context->SetZLayer(vSlotObject, Graphic3d_ZLayerId_Top);
            if (VSlotDragEnabled())
                context->Activate(vSlotObject, 0, Standard_True);
            fixtureShapeAnchor = displayedVSlotFixture.anchor;

            Vec3d axisX;
            Vec3d axisY;
            Vec3d axisZ;
            if (NormalizeFixtureAxes(displayedVSlotFixture, axisX, axisY, axisZ))
            {
                fixtureShapeAxisX = axisX;
                fixtureShapeAxisY = axisY;
                fixtureShapeAxisZ = axisZ;
                vSlotLabelStart = labels.size();
                labels.push_back({
                    Add(displayedVSlotFixture.anchor,
                        Add(Multiply(axisX, displayedVSlotFixture.longLengthMm),
                            Multiply(axisZ, displayedVSlotFixture.wallHeightMm))),
                    QStringLiteral("V槽 +X 长边"), QColor(244, 67, 54), false });
                labels.push_back({
                    Add(displayedVSlotFixture.anchor,
                        Add(Multiply(axisY, displayedVSlotFixture.shortLengthMm),
                            Multiply(axisZ, displayedVSlotFixture.wallHeightMm))),
                    QStringLiteral("V槽 +Y 短边"), QColor(67, 160, 71), false });
            }
        }

        if (selectedGroundFaceCandidate >= 0
            && selectedGroundFaceCandidate < static_cast<int>(groundFaceRecords.size()))
        {
            Handle(AIS_Shape) contactFace = new AIS_Shape(
                groundFaceRecords[static_cast<size_t>(selectedGroundFaceCandidate)].faces);
            contactFace->SetColor(Quantity_Color(0.10, 0.62, 0.92, Quantity_TOC_RGB));
            contactFace->SetTransparency(0.38);
            contactFace->Attributes()->SetFaceBoundaryDraw(Standard_True);
            contactFace->Attributes()->SetFaceBoundaryAspect(new Prs3d_LineAspect(
                Quantity_Color(0.02, 0.28, 0.52, Quantity_TOC_RGB),
                Aspect_TOL_SOLID,
                2.4));
            AddAnnotation(contactFace, AIS_Shaded, false);
            context->SetLocation(contactFace, ModelDisplayLocation());
            context->SetZLayer(contactFace, Graphic3d_ZLayerId_Top);
        }

        for (const ScanRegion& region : scanRegions)
        {
            const ScanRegion displayRegion = DisplayScanRegion(region);
            const TopoDS_Shape regionShape = BuildScanRegionShape(displayRegion);
            if (regionShape.IsNull()) continue;
            Handle(AIS_Shape) patch = new AIS_Shape(regionShape);
            patch->SetColor(ToOcctColor(displayRegion.color));
            patch->SetTransparency(displayRegion.selected ? 0.04 : 0.32);
            patch->Attributes()->SetFaceBoundaryDraw(Standard_True);
            patch->Attributes()->SetFaceBoundaryAspect(new Prs3d_LineAspect(
                ToOcctColor(displayRegion.color.darker(145)),
                Aspect_TOL_SOLID,
                displayRegion.selected ? 2.8 : 1.4));
            context->SetMaterial(
                patch,
                Graphic3d_MaterialAspect(Graphic3d_NOM_PLASTIC),
                Standard_False);
            AddAnnotation(patch, AIS_Shaded, false);
            context->SetZLayer(patch, Graphic3d_ZLayerId_Top);
            ++displayedScanRegions;
        }

        for (const GuideSegment& guide : guides)
        {
            const GuideSegment displayGuide = DisplayGuide(guide);
            if (!IsFinite(displayGuide.start) || !IsFinite(displayGuide.end)) continue;
            const gp_Pnt start = ToPoint(displayGuide.start);
            const gp_Pnt end = ToPoint(displayGuide.end);
            if (start.Distance(end) <= 1.0e-6) continue;
            Handle(AIS_Shape) line = new AIS_Shape(SegmentShape(displayGuide));
            line->Attributes()->SetWireAspect(new Prs3d_LineAspect(
                ToOcctColor(displayGuide.color), Aspect_TOL_SOLID,
                std::max(1.0, displayGuide.width)));
            line->Attributes()->SetLineAspect(new Prs3d_LineAspect(
                ToOcctColor(displayGuide.color), Aspect_TOL_SOLID,
                std::max(1.0, displayGuide.width)));
            AddAnnotation(line, AIS_WireFrame, true);
            if (!displayGuide.label.isEmpty())
            {
                labels.push_back({ displayGuide.end, displayGuide.label,
                    displayGuide.color, false });
            }
        }
        for (const StationMarker& marker : markers)
        {
            const StationMarker displayMarker = DisplayMarker(marker);
            if (!IsFinite(displayMarker.position)) continue;
            const gp_Pnt position = ToPoint(displayMarker.position);
            Handle(AIS_Point) point = new AIS_Point(new Geom_CartesianPoint(position));
            point->Attributes()->SetPointAspect(new Prs3d_PointAspect(
                Aspect_TOM_O_PLUS,
                ToOcctColor(displayMarker.color),
                std::max(3.0, displayMarker.size)));
            AddAnnotation(point, 0, true);
            if (!displayMarker.label.isEmpty())
            {
                labels.push_back({ displayMarker.position, displayMarker.label,
                    displayMarker.color, displayMarker.size >= 9.0 });
            }
        }
        SetOverlayLabels(labels);
        RefreshRotationControls();
        context->UpdateCurrentViewer();
        if (!view.IsNull()) view->Redraw();
        UpdateLabelPositions();
    }

    void DisplayModel(bool fit)
    {
        if (!initialized || context.IsNull())
        {
            pendingFit = pendingFit || fit;
            return;
        }
        if (!modelObject.IsNull())
        {
            context->Remove(modelObject, Standard_False);
            modelObject.Nullify();
        }
        if (!shape.IsNull())
        {
            modelObject = new AIS_Shape(shape);
            modelObject->SetColor(Quantity_Color(0.72, 0.78, 0.84, Quantity_TOC_RGB));
            modelObject->Attributes()->SetFaceBoundaryDraw(Standard_True);
            modelObject->Attributes()->SetFaceBoundaryAspect(new Prs3d_LineAspect(
                Quantity_Color(0.12, 0.22, 0.28, Quantity_TOC_RGB),
                Aspect_TOL_SOLID,
                1.0));
            modelObject->Attributes()->SetDeviationCoefficient(0.001);
            modelObject->Attributes()->SetDeviationAngle(0.25);
            context->SetMaterial(
                modelObject,
                Graphic3d_MaterialAspect(Graphic3d_NOM_SATIN),
                Standard_False);
            context->Display(modelObject, AIS_Shaded, -1, Standard_False);
            ApplyModelDisplayFrame();
        }
        context->UpdateCurrentViewer();
        if (fit) FitAll();
        else if (!view.IsNull()) view->Redraw();
        UpdateLabelPositions();
    }

    void FitAll()
    {
        pendingFit = false;
        if (view.IsNull() || shape.IsNull()) return;
        view->FitAll(0.02, Standard_False);
        view->ZFitAll();
        view->Redraw();
        UpdateLabelPositions();
    }

    QPoint OcctPixel(const QPointF& logicalPosition) const
    {
        const qreal dpr = q->devicePixelRatioF();
        return QPoint(
            qRound(logicalPosition.x() * dpr),
            qRound(logicalPosition.y() * dpr));
    }

    bool ProjectToOverlay(const Vec3d& point, QPointF& projected) const
    {
        if (view.IsNull() || !IsFinite(point)) return false;
        try
        {
            Standard_Integer pixelX = 0;
            Standard_Integer pixelY = 0;
            view->Convert(point.x, point.y, point.z, pixelX, pixelY);
            const qreal dpr = q->devicePixelRatioF();
            projected = QPointF(pixelX / dpr, pixelY / dpr);
            return std::isfinite(projected.x()) && std::isfinite(projected.y())
                && projected.x() >= -100.0 && projected.x() <= q->width() + 100.0
                && projected.y() >= -100.0 && projected.y() <= q->height() + 100.0;
        }
        catch (...)
        {
            return false;
        }
    }

    void UpdateVSlotLabelPositions()
    {
        if (vSlotLabelStart < 0
            || vSlotLabelStart + 1 >= overlayLabels.size())
        {
            return;
        }
        Vec3d axisX;
        Vec3d axisY;
        Vec3d axisZ;
        if (!NormalizeFixtureAxes(displayedVSlotFixture, axisX, axisY, axisZ)) return;
        overlayLabels[vSlotLabelStart].position = Add(
            displayedVSlotFixture.anchor,
            Add(Multiply(axisX, displayedVSlotFixture.longLengthMm),
                Multiply(axisZ, displayedVSlotFixture.wallHeightMm)));
        overlayLabels[vSlotLabelStart + 1].position = Add(
            displayedVSlotFixture.anchor,
            Add(Multiply(axisY, displayedVSlotFixture.shortLengthMm),
                Multiply(axisZ, displayedVSlotFixture.wallHeightMm)));
        UpdateLabelPositions();
    }

    bool HitVSlot(const QPoint& pixel, bool redrawHighlight)
    {
        if (!VSlotDragEnabled()
            || context.IsNull() || view.IsNull() || vSlotObject.IsNull()) return false;
        context->MoveTo(pixel.x(), pixel.y(), view, redrawHighlight ? Standard_True : Standard_False);
        return context->HasDetected()
            && context->DetectedInteractive() == vSlotObject;
    }

    bool VSlotPlanePoint(const QPoint& pixel, Vec3d& result) const
    {
        if (!VSlotDragEnabled() || view.IsNull() || vSlotObject.IsNull()) return false;
        Vec3d normal;
        if (!Normalize(displayedGroundSurface.axisZ, normal)) return false;
        Standard_Real x = 0.0;
        Standard_Real y = 0.0;
        Standard_Real z = 0.0;
        Standard_Real vx = 0.0;
        Standard_Real vy = 0.0;
        Standard_Real vz = 0.0;
        try
        {
            view->ConvertWithProj(
                pixel.x(), pixel.y(), x, y, z, vx, vy, vz);
        }
        catch (...)
        {
            return false;
        }
        const Vec3d rayOrigin = { x, y, z };
        const Vec3d rayDirection = { vx, vy, vz };
        const double denominator = Dot(rayDirection, normal);
        if (!IsFinite(rayOrigin) || !IsFinite(rayDirection)
            || !std::isfinite(denominator) || std::abs(denominator) <= 1.0e-8)
        {
            return false;
        }
        const double distance = Dot(Subtract(
            displayedGroundSurface.planePoint, rayOrigin), normal)
            / denominator;
        if (!std::isfinite(distance) || std::abs(distance) > 1.0e9) return false;
        result = Add(rayOrigin, Multiply(rayDirection, distance));
        return IsFinite(result);
    }

    void SetVSlotDragVisual(bool active)
    {
        if (context.IsNull() || vSlotObject.IsNull()) return;
        context->SetColor(
            vSlotObject,
            vSlotSnappedToWorkpiece
                ? Quantity_Color(0.16, 0.68, 0.34, Quantity_TOC_RGB)
                : active
                ? Quantity_Color(1.0, 0.50, 0.02, Quantity_TOC_RGB)
                : ToOcctColor(displayedVSlotFixture.color),
            Standard_False);
        context->SetTransparency(
            vSlotObject, active ? 0.0 : 0.04, Standard_False);
        context->Redisplay(vSlotObject, Standard_False);
    }

    bool SetDraggedVSlotPose(
        const Vec3d& anchor,
        const Vec3d& requestedAxisX,
        const Vec3d& requestedAxisY,
        const Vec3d& requestedAxisZ,
        bool snappedToWorkpiece = false)
    {
        if (!VSlotDragEnabled() || !IsFinite(anchor)
            || vSlotObject.IsNull() || context.IsNull()) return false;
        Vec3d normal;
        if (!Normalize(displayedGroundSurface.axisZ, normal)) return false;
        VSlotFixture requestedFixture = displayedVSlotFixture;
        requestedFixture.axisX = requestedAxisX;
        requestedFixture.axisY = requestedAxisY;
        requestedFixture.axisZ = requestedAxisZ;
        Vec3d nextAxisX;
        Vec3d nextAxisY;
        Vec3d nextAxisZ;
        if (!NormalizeFixtureAxes(
                requestedFixture, nextAxisX, nextAxisY, nextAxisZ)
            || Dot(nextAxisZ, normal) < 0.99999)
        {
            return false;
        }
        const Vec3d projectedAnchor = ProjectToPlane(
            anchor, displayedGroundSurface.planePoint, normal);

        // 原始实体以 fixtureShape* 为局部基准。一次刚体变换同时更新 anchor
        // 与朝向，避免磁吸方向改变时重建 AIS 对象并中断正在进行的拖动。
        const auto component = [](const Vec3d& value, int index)
            {
                return index == 0 ? value.x : (index == 1 ? value.y : value.z);
            };
        const auto matrixValue = [&](int row, int column)
            {
                return component(nextAxisX, row) * component(fixtureShapeAxisX, column)
                    + component(nextAxisY, row) * component(fixtureShapeAxisY, column)
                    + component(nextAxisZ, row) * component(fixtureShapeAxisZ, column);
            };
        const double r11 = matrixValue(0, 0);
        const double r12 = matrixValue(0, 1);
        const double r13 = matrixValue(0, 2);
        const double r21 = matrixValue(1, 0);
        const double r22 = matrixValue(1, 1);
        const double r23 = matrixValue(1, 2);
        const double r31 = matrixValue(2, 0);
        const double r32 = matrixValue(2, 1);
        const double r33 = matrixValue(2, 2);
        const Vec3d transformedBaseAnchor = {
            r11 * fixtureShapeAnchor.x + r12 * fixtureShapeAnchor.y + r13 * fixtureShapeAnchor.z,
            r21 * fixtureShapeAnchor.x + r22 * fixtureShapeAnchor.y + r23 * fixtureShapeAnchor.z,
            r31 * fixtureShapeAnchor.x + r32 * fixtureShapeAnchor.y + r33 * fixtureShapeAnchor.z
        };
        const Vec3d translation = Subtract(projectedAnchor, transformedBaseAnchor);
        if (!IsFinite(translation) || Length(translation) > 1.0e6) return false;
        gp_Trsf pose;
        pose.SetValues(
            r11, r12, r13, translation.x,
            r21, r22, r23, translation.y,
            r31, r32, r33, translation.z);
        context->SetLocation(vSlotObject, TopLoc_Location(pose));
        displayedVSlotFixture.anchor = projectedAnchor;
        displayedVSlotFixture.axisX = nextAxisX;
        displayedVSlotFixture.axisY = nextAxisY;
        displayedVSlotFixture.axisZ = nextAxisZ;
        vSlotFixture.anchor = DisplayToModel(projectedAnchor);
        vSlotFixture.axisX = DisplayDirectionToModel(nextAxisX);
        vSlotFixture.axisY = DisplayDirectionToModel(nextAxisY);
        vSlotFixture.axisZ = DisplayDirectionToModel(nextAxisZ);
        vSlotSnappedToWorkpiece = snappedToWorkpiece;
        UpdateVSlotLabelPositions();
        context->UpdateCurrentViewer();
        if (!view.IsNull()) view->Redraw();
        return true;
    }

    bool SetDraggedVSlotAnchor(const Vec3d& anchor, bool snappedToWorkpiece = false)
    {
        return SetDraggedVSlotPose(
            anchor,
            displayedVSlotFixture.axisX,
            displayedVSlotFixture.axisY,
            displayedVSlotFixture.axisZ,
            snappedToWorkpiece);
    }

    bool SnapVSlotToGroundFeature()
    {
        if (!VSlotDragEnabled()) return false;
        VSlotSnapPose closest;
        if (!ClosestWorkpieceSnapPose(displayedVSlotFixture.anchor, closest))
        {
            vSlotSnappedToWorkpiece = false;
            return false;
        }
        const bool sameAnchor = Length(Subtract(
            closest.anchor, displayedVSlotFixture.anchor)) <= 1.0e-6;
        const bool sameDirection = Dot(
            closest.axisX, displayedVSlotFixture.axisX) >= 1.0 - 1.0e-10;
        if (sameAnchor && sameDirection)
        {
            vSlotSnappedToWorkpiece = true;
            return false;
        }
        return SetDraggedVSlotPose(
            closest.anchor,
            closest.axisX,
            closest.axisY,
            closest.axisZ,
            true);
    }

    void ResizeNativeView(bool fit)
    {
        if (!initialized || view.IsNull()) return;
        try
        {
            const WId currentWinId = q->winId();
            if (nativeWindow.IsNull() || boundWinId != currentWinId)
            {
                nativeWindow = new WNT_Window(
                    reinterpret_cast<Aspect_Handle>(currentWinId));
                view->SetWindow(nativeWindow);
                boundWinId = currentWinId;
            }
            nativeWindow->DoResize();
            view->MustBeResized();
            if (fit && !shape.IsNull()) FitAll();
            else view->Redraw();
            UpdateLabelPositions();
        }
        catch (const Standard_Failure& failure)
        {
            const char* message = failure.GetMessageString();
            SetStatus(QStringLiteral("更新 CAD 原生视口尺寸失败：%1")
                .arg(message == nullptr ? QStringLiteral("未知 OCCT 异常")
                                        : QString::fromLocal8Bit(message).simplified()));
        }
    }

    void ScheduleNativeResize(bool fit)
    {
        resizeNeedsFit = resizeNeedsFit || fit;
        if (resizeScheduled) return;
        resizeScheduled = true;
        QTimer::singleShot(0, q, [this]()
        {
            resizeScheduled = false;
            const bool shouldFit = resizeNeedsFit;
            resizeNeedsFit = false;
            ResizeNativeView(shouldFit);
        });
    }

    CadModel3DView* q = nullptr;
    QLabel* statusLabel = nullptr;
    QFrame* rotationPanel = nullptr;
    QLabel* rotationStepLabel = nullptr;
    QToolButton* workpieceRotateMinusButton = nullptr;
    QToolButton* workpieceRotatePlusButton = nullptr;
    QToolButton* vSlotRotateMinusButton = nullptr;
    QToolButton* vSlotRotatePlusButton = nullptr;
    QVector<OverlayLabel> overlayLabels;
    QVector<QLabel*> labelWidgets;
    QString statusText;
    QString loadedPath;
    QString loadedSourceSha256;
    QString lastError;
    qint64 loadedSourceSize = -1;
    qint64 loadedSourceModifiedMs = -1;
    TopoDS_Shape shape;
    int faceCount = 0;
    QString theoreticalRobotLoadedPath;
    QString theoreticalRobotSourceSha256;
    qint64 theoreticalRobotSourceSize = -1;
    qint64 theoreticalRobotSourceModifiedMs = -1;
    TopoDS_Shape theoreticalRobotShape;
    RobotCadAssemblyLoader::Result theoreticalRobotLoadResult;
    RobotCollisionEnvelopeStore::EnvelopeSet robotCollisionEnvelope;
    RobotPlacementMetadata robotPlacementMetadata;
    RobotPresentationKind robotPresentationKind = RobotPresentationKind::None;
    TopLoc_Location theoreticalRobotLocation;
    Vec3d robotBaseDisplay;
    Vec3d robotLabelPosition;
    VSlotFixture vSlotFixture;
    VSlotFixture displayedVSlotFixture;
    GroundSurface groundSurface;
    GroundSurface displayedGroundSurface;
    Vec3d displayFrameAxisX = { 1.0, 0.0, 0.0 };
    Vec3d displayFrameAxisY = { 0.0, 1.0, 0.0 };
    Vec3d displayFrameAxisZ = { 0.0, 0.0, 1.0 };
    Vec3d displayFrameTranslation;
    QVector<ScanRegion> scanRegions;
    std::vector<GroundFaceRecord> groundFaceRecords;
    QVector<GuideSegment> guides;
    QVector<StationMarker> markers;
    Handle(Aspect_DisplayConnection) displayConnection;
    Handle(OpenGl_GraphicDriver) graphicDriver;
    Handle(V3d_Viewer) viewer;
    Handle(AIS_InteractiveContext) context;
    Handle(V3d_View) view;
    Handle(Aspect_Window) nativeWindow;
    Handle(AIS_Shape) modelObject;
    std::vector<Handle(AIS_Shape)> theoreticalRobotObjects;
    std::vector<Handle(AIS_Shape)> robotCollisionEnvelopeObjects;
    Handle(AIS_Shape) groundObject;
    Handle(AIS_Shape) groundGridObject;
    Handle(AIS_Shape) vSlotObject;
    std::vector<Handle(AIS_InteractiveObject)> annotations;
    std::function<void(const Vec3d&)> vSlotMovedCallback;
    std::function<void(const VSlotFixture&, bool)> vSlotPoseChangedCallback;
    std::function<void(bool)> vSlotSnapStateCallback;
    std::function<void(double)> workpieceRotationRequestedCallback;
    std::function<void(double)> vSlotRotationRequestedCallback;
    double rotationStepDegrees = 5.0;
    QPoint lastMousePos;
    Vec3d fixtureShapeAnchor;
    Vec3d fixtureShapeAxisX = { 1.0, 0.0, 0.0 };
    Vec3d fixtureShapeAxisY = { 0.0, 1.0, 0.0 };
    Vec3d fixtureShapeAxisZ = { 0.0, 0.0, 1.0 };
    Vec3d dragStartAnchor;
    Vec3d dragStartPlanePoint;
    DragMode dragMode = DragMode::None;
    int vSlotLabelStart = -1;
    int displayedScanRegions = 0;
    int selectedGroundFaceCandidate = -1;
    WId boundWinId = 0;
    bool initialized = false;
    bool pendingFit = false;
    bool resizeScheduled = false;
    bool resizeNeedsFit = false;
    bool vSlotMovedDuringDrag = false;
    bool vSlotSnappedToWorkpiece = false;
    bool theoreticalRobotVisibleRequested = true;
    bool theoreticalRobotDisplayed = false;
    bool theoreticalRobotPlacementResolved = false;
    bool theoreticalRobotDisplayComplete = false;
    bool theoreticalRobotPresentationInitialized = false;
    bool theoreticalRobotBlockScheduled = false;
    bool theoreticalRobotFitFirstBlock = false;
    bool theoreticalRobotFitCompletion = false;
    size_t theoreticalRobotNextDisplayBlock = 0;
    quint64 theoreticalRobotDisplayGeneration = 0;
    int theoreticalRobotLabelIndex = -1;
    int theoreticalRobotPresentationBuildCount = 0;
};

CadModel3DView::CadModel3DView(QWidget* parent)
    : QWidget(parent)
{
    setAttribute(Qt::WA_NativeWindow, true);
    setAttribute(Qt::WA_PaintOnScreen, true);
    setAttribute(Qt::WA_NoSystemBackground, true);
    setAutoFillBackground(false);
    setMouseTracking(true);
    setFocusPolicy(Qt::StrongFocus);
    setMinimumSize(720, 520);
    m_impl = std::make_unique<Impl>(this);
    m_impl->ResizeStatus();
}

CadModel3DView::~CadModel3DView() = default;

bool CadModel3DView::SetStepFile(
    const QString& sourceStepPath,
    QString& error,
    bool preserveView)
{
    return SetStepFile(sourceStepPath, QString(), error, preserveView);
}

bool CadModel3DView::SetStepFile(
    const QString& sourceStepPath,
    const QString& expectedSourceSha256,
    QString& error,
    bool preserveView)
{
    Q_ASSERT(QThread::currentThread() == qApp->thread());
    error.clear();
    m_impl->lastError.clear();
    const QString expectedHash = expectedSourceSha256.trimmed().toLower();
    static const QRegularExpression sha256Pattern(QStringLiteral("^[0-9a-f]{64}$"));
    const QFileInfo before(sourceStepPath);
    const bool readableFile = before.exists() && before.isFile() && !before.isSymLink();
    if (preserveView
        && !m_impl->shape.IsNull()
        && SamePath(m_impl->loadedPath, sourceStepPath)
        && readableFile
        && before.size() == m_impl->loadedSourceSize
        && before.lastModified().toMSecsSinceEpoch() == m_impl->loadedSourceModifiedMs
        && (expectedHash.isEmpty() || expectedHash == m_impl->loadedSourceSha256))
    {
        // size/mtime 不能代替内容身份。即使外部程序保留了时间戳，也必须复算源
        // SHA-256 后才能复用内存中的 B-Rep。
        QString quickHashError;
        const QString currentHash = StableFileSha256(
            sourceStepPath, before, quickHashError);
        if (!currentHash.isEmpty()
            && currentHash == m_impl->loadedSourceSha256
            && (expectedHash.isEmpty() || currentHash == expectedHash))
        {
            return true;
        }
    }

    // 非保留刷新必须重读原始 STEP；文件快照或身份发生变化时也必须先清空旧
    // 显示，失败后绝不能继续展示上一个 B-Rep 造成误认。
    ClearModel(QStringLiteral("正在读取原始 STEP CAD 模型…"));
    if (!readableFile)
    {
        error = QStringLiteral("原始 STEP 不是可读取的普通文件：%1").arg(sourceStepPath);
        m_impl->lastError = error;
        m_impl->SetStatus(error);
        return false;
    }
    if (!expectedHash.isEmpty() && !sha256Pattern.match(expectedHash).hasMatch())
    {
        error = QStringLiteral("追溯元数据中的原始 STEP SHA-256 无效。");
        m_impl->lastError = error;
        m_impl->SetStatus(error);
        return false;
    }
    if (before.size() <= 0 || before.size() > kMaximumCadFileBytes)
    {
        error = QStringLiteral("原始 STEP 为空或超过 256 MiB 显示上限。");
        m_impl->lastError = error;
        m_impl->SetStatus(error);
        return false;
    }
    const QString extension = before.suffix().toLower();
    if (extension != QStringLiteral("step") && extension != QStringLiteral("stp"))
    {
        error = QStringLiteral("CAD 外显仅接受原始 .step/.stp 文件。");
        m_impl->lastError = error;
        m_impl->SetStatus(error);
        return false;
    }

    QString hashError;
    const QString sourceHash = StableFileSha256(sourceStepPath, before, hashError);
    if (sourceHash.isEmpty() || (!expectedHash.isEmpty() && sourceHash != expectedHash))
    {
        error = sourceHash.isEmpty()
            ? hashError
            : QStringLiteral("原始 STEP SHA-256 与模型追溯元数据不一致，禁止外显。");
        m_impl->lastError = error;
        m_impl->SetStatus(error);
        return false;
    }

    TopoDS_Shape loadedShape;
    int loadedFaceCount = 0;
    std::vector<GroundFaceRecord> loadedGroundFaces;
    try
    {
        QMutexLocker<QMutex> lock(&occtsync::OperationMutex());
#ifdef _WIN32
        const std::filesystem::path nativePath(sourceStepPath.toStdWString());
#else
        const QByteArray encodedPath = sourceStepPath.toUtf8();
        const std::filesystem::path nativePath(encodedPath.constData());
#endif
        std::ifstream input(nativePath, std::ios::in | std::ios::binary);
        if (!input.is_open() || input.fail())
        {
            error = QStringLiteral("无法打开原始 STEP：%1").arg(sourceStepPath);
        }
        else
        {
            STEPControl_Reader reader;
            const IFSelect_ReturnStatus status = reader.ReadStream("source.step", input);
            input.close();
            if (status != IFSelect_RetDone)
            {
                error = ReadStatusText(status);
            }
            else
            {
                TColStd_SequenceOfAsciiString lengthUnits;
                TColStd_SequenceOfAsciiString angleUnits;
                TColStd_SequenceOfAsciiString solidAngleUnits;
                reader.FileUnits(lengthUnits, angleUnits, solidAngleUnits);
                if (lengthUnits.IsEmpty())
                {
                    error = QStringLiteral("原始 STEP 没有可验证的长度单位，拒绝显示以避免标注错位。");
                }
                else
                {
                    reader.SetSystemLengthUnit(1.0);
                    const Standard_Integer roots = reader.NbRootsForTransfer();
                    if (roots <= 0 || reader.TransferRoots() != roots)
                    {
                        error = QStringLiteral("原始 STEP 根节点未能完整转换，拒绝显示残缺 CAD。");
                    }
                    else
                    {
                        loadedShape = reader.OneShape();
                        if (loadedShape.IsNull())
                        {
                            error = QStringLiteral("原始 STEP 未生成可显示的 B-Rep 形状。");
                        }
                        else
                        {
                            for (TopExp_Explorer faces(loadedShape, TopAbs_FACE); faces.More(); faces.Next())
                                ++loadedFaceCount;
                            if (loadedFaceCount <= 0)
                            {
                                loadedShape.Nullify();
                                error = QStringLiteral("原始 STEP 不包含可显示的曲面。");
                            }
                            else
                            {
                                loadedGroundFaces = AnalyzeGroundFaceCandidates(loadedShape);
                            }
                        }
                    }
                }
            }
        }
    }
    catch (const Standard_Failure& failure)
    {
        const char* message = failure.GetMessageString();
        error = QStringLiteral("读取原始 STEP 失败：%1")
            .arg(message == nullptr ? QStringLiteral("未知 OCCT 异常")
                                    : QString::fromLocal8Bit(message).simplified());
    }
    catch (const std::exception& exception)
    {
        error = QStringLiteral("读取原始 STEP 异常：%1")
            .arg(QString::fromLocal8Bit(exception.what()).simplified());
    }
    catch (...)
    {
        error = QStringLiteral("读取原始 STEP 发生未知异常。");
    }

    QFileInfo after(sourceStepPath);
    after.refresh();
    if (error.isEmpty()
        && (!after.exists() || !after.isFile() || after.isSymLink()
            || after.size() != before.size()
            || after.lastModified().toMSecsSinceEpoch()
                != before.lastModified().toMSecsSinceEpoch()))
    {
        error = QStringLiteral("原始 STEP 在读取过程中发生变化，已拒绝显示。");
        loadedShape.Nullify();
    }
    if (error.isEmpty())
    {
        QString postReadHashError;
        const QString postReadHash = StableFileSha256(
            sourceStepPath, after, postReadHashError);
        if (postReadHash.isEmpty() || postReadHash != sourceHash)
        {
            error = postReadHash.isEmpty()
                ? postReadHashError
                : QStringLiteral("原始 STEP 在解析前后内容身份发生变化，已拒绝显示。");
            loadedShape.Nullify();
        }
    }
    if (!error.isEmpty() || loadedShape.IsNull())
    {
        if (error.isEmpty()) error = QStringLiteral("原始 STEP CAD 加载失败。");
        m_impl->lastError = error;
        m_impl->SetStatus(error);
        update();
        return false;
    }

    if (!preserveView)
    {
        // 新模型先回到无平移的文件坐标框架；随后 SetSceneOverlays 会围绕
        // 该模板的工装 anchor 建立新的落地显示枢轴。
        m_impl->displayFrameAxisX = { 1.0, 0.0, 0.0 };
        m_impl->displayFrameAxisY = { 0.0, 1.0, 0.0 };
        m_impl->displayFrameAxisZ = { 0.0, 0.0, 1.0 };
        m_impl->displayFrameTranslation = {};
    }
    m_impl->shape = loadedShape;
    m_impl->faceCount = loadedFaceCount;
    m_impl->groundFaceRecords = std::move(loadedGroundFaces);
    m_impl->selectedGroundFaceCandidate = -1;
    m_impl->loadedPath = NormalizedExistingPath(sourceStepPath);
    m_impl->loadedSourceSha256 = sourceHash;
    m_impl->loadedSourceSize = before.size();
    m_impl->loadedSourceModifiedMs = before.lastModified().toMSecsSinceEpoch();
    m_impl->SetStatus(QStringLiteral(
        "原始 STEP / B-Rep：%1　曲面 %2　可落地大面 %3　实体+边线显示　"
        "顶部悬浮按钮旋转工件或V槽 / 左键拖动V槽并自动对齐吸附 / "
        "左键空白旋转视角 / 中键或右键平移 / 滚轮缩放 / 双击复位")
        .arg(QFileInfo(sourceStepPath).fileName())
        .arg(loadedFaceCount)
        .arg(m_impl->groundFaceRecords.size()));
    m_impl->DisplayModel(!preserveView);
    update();
    return true;
}

bool CadModel3DView::AdoptTheoreticalRobotAssembly(
    const QString& sourceStepPath,
    const QString& expectedSourceSha256,
    RobotCadAssemblyLoader::Result&& loaded,
    QString& error,
    bool preserveView)
{
    Q_ASSERT(QThread::currentThread() == qApp->thread());
    error.clear();
    const QString expectedHash = expectedSourceSha256.trimmed().toLower();
    static const QRegularExpression sha256Pattern(QStringLiteral("^[0-9a-f]{64}$"));
    const QFileInfo sourceInfo(sourceStepPath);
    if (!sourceInfo.exists() || !sourceInfo.isFile() || sourceInfo.isSymLink()
        || sourceInfo.size() <= 0 || sourceInfo.size() > kMaximumCadFileBytes)
    {
        error = QStringLiteral("后台解析完成后，理论机器人 STEP 已缺失或越界。");
        return false;
    }
    if (!sha256Pattern.match(expectedHash).hasMatch())
    {
        error = QStringLiteral("后台理论机器人资产 SHA-256 无效。");
        return false;
    }
    QString hashError;
    const QString currentHash = StableFileSha256(sourceStepPath, sourceInfo, hashError);
    if (currentHash.isEmpty() || currentHash != expectedHash)
    {
        error = currentHash.isEmpty()
            ? hashError
            : QStringLiteral("理论机器人资产在后台解析期间发生变化。");
        return false;
    }
    if (!loaded.assemblyShape || loaded.assemblyShape->IsNull()
        || loaded.statistics.jointComponentCount != 7
        || !loaded.base.valid
        || !loaded.base.sourceUp.allFinite()
        || (loaded.base.sourceUp - Eigen::Vector3d::UnitY()).norm() > 1.0e-9
        || !loaded.statistics.assemblyBoundsMm.valid
        || loaded.displayBlocks.empty()
        || loaded.statistics.displayBlockCount
            != static_cast<qsizetype>(loaded.displayBlocks.size())
        || std::any_of(
            loaded.displayBlocks.cbegin(), loaded.displayBlocks.cend(),
            [](const std::shared_ptr<TopoDS_Shape>& block)
            {
                return !block || block->IsNull();
            }))
    {
        error = QStringLiteral("后台解析结果不是完整、有限的 J0-J6 理论机器人。");
        return false;
    }
    if (loaded.statistics.sourceSha256 != currentHash)
    {
        error = QStringLiteral("后台 J0-J6 解析结果与当前受控 STEP 不是同一内容。");
        return false;
    }
    if (!loaded.statistics.displayTriangulationPrepared)
    {
        error = QStringLiteral("理论机器人尚未在后台生成显示缓存，拒绝在界面线程补算。");
        return false;
    }
    if (loaded.statistics.assemblyBoundsMm.minimumMm.y()
        < loaded.base.minimumYmm - 2.0)
    {
        error = QStringLiteral(
            "机器人 J0 安装面上方约定无效：其它关节低于基座超过 2 mm，拒绝自动贴地。");
        return false;
    }

    m_impl->CancelTheoreticalRobotPresentation();
    if (!m_impl->context.IsNull())
    {
        for (const Handle(AIS_Shape)& object : m_impl->robotCollisionEnvelopeObjects)
        {
            if (!object.IsNull()) m_impl->context->Remove(object, Standard_False);
        }
    }
    m_impl->robotCollisionEnvelopeObjects.clear();
    m_impl->robotCollisionEnvelope = RobotCollisionEnvelopeStore::EnvelopeSet();
    m_impl->robotPresentationKind = Impl::RobotPresentationKind::DetailedBRep;
    m_impl->robotPlacementMetadata.valid = true;
    m_impl->robotPlacementMetadata.sourceUp = loaded.base.sourceUp;
    m_impl->robotPlacementMetadata.sourceFloorYmm = loaded.base.minimumYmm;
    m_impl->robotPlacementMetadata.sourceBaseCenterMm =
        loaded.base.conservativeBaseCenterMm;
    m_impl->robotPlacementMetadata.sourceBoundsMm.minimumMm =
        loaded.statistics.assemblyBoundsMm.minimumMm;
    m_impl->robotPlacementMetadata.sourceBoundsMm.maximumMm =
        loaded.statistics.assemblyBoundsMm.maximumMm;
    m_impl->theoreticalRobotShape = *loaded.assemblyShape;
    m_impl->theoreticalRobotLoadResult = std::move(loaded);
    m_impl->theoreticalRobotLoadedPath = NormalizedExistingPath(sourceStepPath);
    m_impl->theoreticalRobotSourceSha256 = currentHash;
    m_impl->theoreticalRobotSourceSize = sourceInfo.size();
    m_impl->theoreticalRobotSourceModifiedMs =
        sourceInfo.lastModified().toMSecsSinceEpoch();
    m_impl->theoreticalRobotFitFirstBlock = !preserveView;
    m_impl->theoreticalRobotFitCompletion = !preserveView;
    m_impl->RebuildAnnotations();
    update();
    return true;
}

bool CadModel3DView::SetRobotCollisionEnvelope(
    const RobotCollisionEnvelopeStore::EnvelopeSet& envelope,
    QString& error,
    bool preserveView)
{
    Q_ASSERT(QThread::currentThread() == qApp->thread());
    if (!ValidateRobotCollisionEnvelopeForScene(envelope, error)) return false;

    std::array<const RobotCollisionEnvelopeStore::JointEnvelope*, 7> joints{};
    for (const RobotCollisionEnvelopeStore::JointEnvelope& joint : envelope.joints)
        joints[static_cast<size_t>(joint.jointIndex)] = &joint;

    std::vector<Handle(AIS_Shape)> builtObjects;
    builtObjects.reserve(7);
    try
    {
        for (const RobotCollisionEnvelopeStore::JointEnvelope* joint : joints)
        {
            if (joint == nullptr)
            {
                error = QStringLiteral("机器人碰撞简模缺少关节边界。");
                return false;
            }
            const Eigen::Vector3d dimensions =
                joint->collisionBoundsMm.maximumMm
                - joint->collisionBoundsMm.minimumMm;
            const TopoDS_Shape box = BRepPrimAPI_MakeBox(
                gp_Pnt(
                    joint->collisionBoundsMm.minimumMm.x(),
                    joint->collisionBoundsMm.minimumMm.y(),
                    joint->collisionBoundsMm.minimumMm.z()),
                dimensions.x(), dimensions.y(), dimensions.z()).Shape();
            if (box.IsNull())
            {
                error = QStringLiteral("J%1 的 OCCT 碰撞 box 构造失败。")
                    .arg(joint->jointIndex);
                return false;
            }
            Handle(AIS_Shape) object = new AIS_Shape(box);
            object->SetColor(Quantity_Color(0.92, 0.25, 0.10, Quantity_TOC_RGB));
            object->SetTransparency(0.48);
            object->Attributes()->SetFaceBoundaryDraw(Standard_True);
            object->Attributes()->SetFaceBoundaryAspect(new Prs3d_LineAspect(
                Quantity_Color(0.55, 0.10, 0.04, Quantity_TOC_RGB),
                Aspect_TOL_SOLID,
                1.6));
            object->Attributes()->SetDeviationCoefficient(0.01);
            if (!m_impl->context.IsNull())
            {
                m_impl->context->SetMaterial(
                    object,
                    Graphic3d_MaterialAspect(Graphic3d_NOM_SATIN),
                    Standard_False);
            }
            builtObjects.push_back(object);
        }
    }
    catch (const Standard_Failure& failure)
    {
        error = QStringLiteral("构造机器人碰撞简模失败：%1")
            .arg(failure.GetMessageString() == nullptr
                ? QStringLiteral("未知 OCCT 异常")
                : QString::fromLocal8Bit(failure.GetMessageString()).simplified());
        return false;
    }
    catch (...)
    {
        error = QStringLiteral("构造机器人碰撞简模发生未知异常。");
        return false;
    }

    m_impl->CancelTheoreticalRobotPresentation();
    if (!m_impl->context.IsNull())
    {
        for (const Handle(AIS_Shape)& object : m_impl->robotCollisionEnvelopeObjects)
        {
            if (!object.IsNull()) m_impl->context->Remove(object, Standard_False);
        }
    }
    m_impl->robotCollisionEnvelopeObjects = std::move(builtObjects);
    m_impl->robotCollisionEnvelope = envelope;
    m_impl->robotPresentationKind = Impl::RobotPresentationKind::CollisionEnvelope;
    m_impl->robotPlacementMetadata.valid = true;
    m_impl->robotPlacementMetadata.sourceUp = envelope.sourceUp;
    m_impl->robotPlacementMetadata.sourceFloorYmm = envelope.baseMinimumYmm;
    m_impl->robotPlacementMetadata.sourceBaseCenterMm = envelope.baseCenterMm;
    m_impl->robotPlacementMetadata.sourceBoundsMm =
        envelope.overallExpandedBoundsMm;
    // 碰撞简模已接管显示，及时释放详细总装及其渐进显示块。
    m_impl->theoreticalRobotShape.Nullify();
    m_impl->theoreticalRobotLoadResult = RobotCadAssemblyLoader::Result();
    m_impl->theoreticalRobotLoadedPath.clear();
    m_impl->theoreticalRobotSourceSha256 = envelope.sourceStepSha256;
    m_impl->theoreticalRobotSourceSize = -1;
    m_impl->theoreticalRobotSourceModifiedMs = -1;
    m_impl->theoreticalRobotDisplayComplete = true;
    m_impl->theoreticalRobotNextDisplayBlock = 7;
    m_impl->theoreticalRobotPresentationInitialized = true;
    ++m_impl->theoreticalRobotPresentationBuildCount;
    m_impl->theoreticalRobotFitFirstBlock = false;
    m_impl->theoreticalRobotFitCompletion = false;
    m_impl->RebuildAnnotations();
    if (!preserveView) m_impl->FitAll();
    update();
    return true;
}

bool CadModel3DView::IsRobotCollisionEnvelope() const
{
    return m_impl->robotPresentationKind
        == Impl::RobotPresentationKind::CollisionEnvelope;
}

void CadModel3DView::ClearTheoreticalRobot()
{
    Q_ASSERT(QThread::currentThread() == qApp->thread());
    m_impl->CancelTheoreticalRobotPresentation();
    if (!m_impl->context.IsNull())
    {
        for (const Handle(AIS_Shape)& object : m_impl->robotCollisionEnvelopeObjects)
        {
            if (!object.IsNull()) m_impl->context->Remove(object, Standard_False);
        }
    }
    m_impl->robotCollisionEnvelopeObjects.clear();
    m_impl->robotCollisionEnvelope = RobotCollisionEnvelopeStore::EnvelopeSet();
    m_impl->robotPlacementMetadata = Impl::RobotPlacementMetadata();
    m_impl->robotPresentationKind = Impl::RobotPresentationKind::None;
    m_impl->theoreticalRobotShape.Nullify();
    m_impl->theoreticalRobotLoadResult = RobotCadAssemblyLoader::Result();
    m_impl->theoreticalRobotLoadedPath.clear();
    m_impl->theoreticalRobotSourceSha256.clear();
    m_impl->theoreticalRobotSourceSize = -1;
    m_impl->theoreticalRobotSourceModifiedMs = -1;
    m_impl->robotBaseDisplay = {};
    m_impl->robotLabelPosition = {};
    m_impl->RebuildAnnotations();
    update();
}

void CadModel3DView::SetTheoreticalRobotVisible(bool visible)
{
    Q_ASSERT(QThread::currentThread() == qApp->thread());
    if (m_impl->theoreticalRobotVisibleRequested == visible) return;
    m_impl->theoreticalRobotVisibleRequested = visible;
    m_impl->RebuildAnnotations();
    if (visible && m_impl->theoreticalRobotDisplayed) m_impl->FitAll();
    update();
}

void CadModel3DView::ClearModel(const QString& message)
{
    Q_ASSERT(QThread::currentThread() == qApp->thread());
    m_impl->shape.Nullify();
    m_impl->faceCount = 0;
    m_impl->groundFaceRecords.clear();
    m_impl->selectedGroundFaceCandidate = -1;
    m_impl->loadedPath.clear();
    m_impl->loadedSourceSha256.clear();
    m_impl->loadedSourceSize = -1;
    m_impl->loadedSourceModifiedMs = -1;
    m_impl->lastError.clear();
    m_impl->vSlotFixture = VSlotFixture();
    m_impl->displayedVSlotFixture = VSlotFixture();
    m_impl->groundSurface = GroundSurface();
    m_impl->displayedGroundSurface = GroundSurface();
    m_impl->displayFrameTranslation = {};
    m_impl->UpdateDisplayFrame(GroundSurface(), VSlotFixture());
    m_impl->scanRegions.clear();
    m_impl->guides.clear();
    m_impl->markers.clear();
    m_impl->RebuildAnnotations();
    m_impl->DisplayModel(false);
    m_impl->SetStatus(message.isEmpty()
        ? QStringLiteral("尚未载入原始 STEP CAD 模型。") : message);
    update();
}

void CadModel3DView::SetAnnotations(
    const QVector<GuideSegment>& guides,
    const QVector<StationMarker>& markers)
{
    SetSceneOverlays(VSlotFixture(), {}, guides, markers);
}

void CadModel3DView::SetSceneOverlays(
    const VSlotFixture& fixture,
    const QVector<ScanRegion>& scanRegions,
    const QVector<GuideSegment>& guides,
    const QVector<StationMarker>& markers)
{
    GroundSurface workpieceFrame;
    workpieceFrame.axisX = fixture.axisX;
    workpieceFrame.axisY = fixture.axisY;
    workpieceFrame.axisZ = fixture.axisZ;
    SetSceneOverlays(workpieceFrame, fixture, scanRegions, guides, markers);
}

void CadModel3DView::SetSceneOverlays(
    const GroundSurface& ground,
    const VSlotFixture& fixture,
    const QVector<ScanRegion>& scanRegions,
    const QVector<GuideSegment>& guides,
    const QVector<StationMarker>& markers)
{
    Q_ASSERT(QThread::currentThread() == qApp->thread());
    const bool robotWasPlaced = m_impl->theoreticalRobotPlacementResolved;
    const bool displayFrameChanged = m_impl->UpdateDisplayFrame(ground, fixture);
    m_impl->groundSurface = ground;
    m_impl->vSlotFixture = fixture;
    m_impl->scanRegions = scanRegions;
    m_impl->guides = guides;
    m_impl->markers = markers;
    m_impl->ApplyModelDisplayFrame();
    m_impl->RebuildAnnotations();
    const bool robotBecamePlaced = !robotWasPlaced
        && m_impl->theoreticalRobotPlacementResolved;
    // 模型/工装旋转是编辑操作，不能每次都 FitAll 重置用户观察视角。
    // 只有机器人第一次加入场景时才扩展一次视野。
    if (robotBecamePlaced && !m_impl->shape.IsNull())
        m_impl->FitAll();
    else if (displayFrameChanged && !m_impl->view.IsNull())
        m_impl->view->Redraw();
    update();
}

void CadModel3DView::ClearSceneOverlays()
{
    SetSceneOverlays(VSlotFixture(), {}, {}, {});
}

void CadModel3DView::SetVSlotMovedCallback(std::function<void(const Vec3d&)> callback)
{
    m_impl->vSlotMovedCallback = std::move(callback);
}

void CadModel3DView::SetVSlotPoseChangedCallback(
    std::function<void(const VSlotFixture&, bool)> callback)
{
    m_impl->vSlotPoseChangedCallback = std::move(callback);
}

void CadModel3DView::SetVSlotSnapStateCallback(std::function<void(bool)> callback)
{
    m_impl->vSlotSnapStateCallback = std::move(callback);
}

void CadModel3DView::SetWorkpieceRotationRequestedCallback(
    std::function<void(double)> callback)
{
    m_impl->workpieceRotationRequestedCallback = std::move(callback);
    m_impl->RefreshRotationControls();
}

void CadModel3DView::SetVSlotRotationRequestedCallback(
    std::function<void(double)> callback)
{
    m_impl->vSlotRotationRequestedCallback = std::move(callback);
    m_impl->RefreshRotationControls();
}

void CadModel3DView::SetRotationStepDegrees(double degrees)
{
    if (!std::isfinite(degrees)) return;
    m_impl->rotationStepDegrees = std::clamp(std::abs(degrees), 0.1, 180.0);
    m_impl->RefreshRotationControls();
}

void CadModel3DView::SetGroundFaceCandidateHighlight(int candidateIndex)
{
    if (candidateIndex < 0
        || candidateIndex >= static_cast<int>(m_impl->groundFaceRecords.size()))
    {
        candidateIndex = -1;
    }
    if (m_impl->selectedGroundFaceCandidate == candidateIndex) return;
    m_impl->selectedGroundFaceCandidate = candidateIndex;
    m_impl->RebuildAnnotations();
    update();
}

void CadModel3DView::FitAll()
{
    if (!m_impl->initialized)
    {
        m_impl->pendingFit = true;
        update();
        return;
    }
    m_impl->FitAll();
}

QString CadModel3DView::LoadedSourcePath() const { return m_impl->loadedPath; }
QString CadModel3DView::LastError() const { return m_impl->lastError; }
bool CadModel3DView::HasCadShape() const { return !m_impl->shape.IsNull(); }
int CadModel3DView::LoadedFaceCount() const { return m_impl->faceCount; }
int CadModel3DView::AnnotationLabelCount() const
{
    return m_impl->overlayLabels.size();
}
bool CadModel3DView::HasDisplayedVSlot() const
{
    return !m_impl->vSlotObject.IsNull()
        && !m_impl->context.IsNull()
        && m_impl->context->IsDisplayed(m_impl->vSlotObject);
}
bool CadModel3DView::HasDisplayedGround() const
{
    return !m_impl->groundObject.IsNull()
        && !m_impl->context.IsNull()
        && m_impl->context->IsDisplayed(m_impl->groundObject);
}
bool CadModel3DView::HasTheoreticalRobotShape() const
{
    return m_impl->HasActiveRobotPresentation();
}
bool CadModel3DView::HasDisplayedTheoreticalRobot() const
{
    if (m_impl->robotPresentationKind
        == Impl::RobotPresentationKind::CollisionEnvelope)
    {
        if (!m_impl->theoreticalRobotPlacementResolved
            || m_impl->robotCollisionEnvelopeObjects.size() != 7
            || m_impl->context.IsNull())
        {
            return false;
        }
        return std::all_of(
            m_impl->robotCollisionEnvelopeObjects.cbegin(),
            m_impl->robotCollisionEnvelopeObjects.cend(),
            [this](const Handle(AIS_Shape)& object)
            {
                return !object.IsNull() && m_impl->context->IsDisplayed(object);
            });
    }
    if (!m_impl->theoreticalRobotDisplayComplete
        || m_impl->theoreticalRobotObjects.empty()
        || m_impl->context.IsNull())
    {
        return false;
    }
    return std::all_of(
        m_impl->theoreticalRobotObjects.cbegin(),
        m_impl->theoreticalRobotObjects.cend(),
        [this](const Handle(AIS_Shape)& object)
        {
            return !object.IsNull() && m_impl->context->IsDisplayed(object);
        });
}
QString CadModel3DView::LoadedTheoreticalRobotPath() const
{
    return m_impl->theoreticalRobotLoadedPath;
}
int CadModel3DView::TheoreticalRobotJointComponentCount() const
{
    if (IsRobotCollisionEnvelope())
        return m_impl->robotCollisionEnvelope.joints.size();
    return static_cast<int>(
        m_impl->theoreticalRobotLoadResult.statistics.jointComponentCount);
}
int CadModel3DView::TheoreticalRobotPresentationBuildCount() const
{
    return m_impl->theoreticalRobotPresentationBuildCount;
}
int CadModel3DView::TheoreticalRobotDisplayBlockCount() const
{
    if (IsRobotCollisionEnvelope())
        return static_cast<int>(m_impl->robotCollisionEnvelopeObjects.size());
    return static_cast<int>(m_impl->theoreticalRobotLoadResult.displayBlocks.size());
}
int CadModel3DView::TheoreticalRobotDisplayedBlockCount() const
{
    if (IsRobotCollisionEnvelope())
        return HasDisplayedTheoreticalRobot()
            ? static_cast<int>(m_impl->robotCollisionEnvelopeObjects.size()) : 0;
    return static_cast<int>(m_impl->theoreticalRobotNextDisplayBlock);
}
bool CadModel3DView::IsTheoreticalRobotDisplayComplete() const
{
    if (IsRobotCollisionEnvelope()) return HasDisplayedTheoreticalRobot();
    return m_impl->theoreticalRobotDisplayComplete;
}
bool CadModel3DView::IsTheoreticalRobotPresentationInProgress() const
{
    if (IsRobotCollisionEnvelope()) return false;
    return m_impl->theoreticalRobotVisibleRequested
        && m_impl->theoreticalRobotPlacementResolved
        && !m_impl->theoreticalRobotDisplayComplete
        && m_impl->theoreticalRobotNextDisplayBlock
            < m_impl->theoreticalRobotLoadResult.displayBlocks.size();
}
Vec3d CadModel3DView::TheoreticalRobotBaseDisplayPosition() const
{
    return m_impl->robotBaseDisplay;
}
int CadModel3DView::DisplayedScanRegionCount() const
{
    return m_impl->displayedScanRegions;
}
Vec3d CadModel3DView::VSlotAnchor() const
{
    return m_impl->vSlotFixture.anchor;
}
bool CadModel3DView::IsVSlotSnappedToWorkpiece() const
{
    return m_impl->vSlotSnappedToWorkpiece;
}
QVector<GroundFaceCandidate> CadModel3DView::GroundFaceCandidates() const
{
    QVector<GroundFaceCandidate> candidates;
    candidates.reserve(static_cast<qsizetype>(m_impl->groundFaceRecords.size()));
    for (const GroundFaceRecord& record : m_impl->groundFaceRecords)
        candidates.push_back(record.summary);
    return candidates;
}
QVector<Vec3d> CadModel3DView::GroundFaceSnapPoints(int candidateIndex) const
{
    if (candidateIndex < 0
        || candidateIndex >= static_cast<int>(m_impl->groundFaceRecords.size()))
    {
        return {};
    }
    return ExtractGroundSnapPoints(
        m_impl->groundFaceRecords[static_cast<size_t>(candidateIndex)]);
}
#ifdef CAD_MODEL_3D_VIEW_TESTING
QVector<Vec3d> CadModel3DView::BuildGroundSnapHullForTesting(
    const QVector<Vec3d>& points,
    const Vec3d& planePoint,
    const Vec3d& planeNormal)
{
    return BuildGroundSnapHull(points, planePoint, planeNormal);
}
#endif
bool CadModel3DView::ProjectModelPoint(const Vec3d& point, QPointF& widgetPoint) const
{
    return m_impl->ProjectToOverlay(m_impl->ModelToDisplay(point), widgetPoint);
}
QSize CadModel3DView::NativeViewportSizePixels() const
{
    if (m_impl->nativeWindow.IsNull()) return QSize();
    Standard_Integer width = 0;
    Standard_Integer height = 0;
    m_impl->nativeWindow->Size(width, height);
    return QSize(width, height);
}

bool CadModel3DView::HasDisplayedShape() const
{
    return !m_impl->modelObject.IsNull()
        && !m_impl->context.IsNull()
        && m_impl->context->IsDisplayed(m_impl->modelObject);
}

QPaintEngine* CadModel3DView::paintEngine() const { return nullptr; }

bool CadModel3DView::event(QEvent* event)
{
    const bool handled = QWidget::event(event);
    if (m_impl
        && (event->type() == QEvent::WinIdChange
            || event->type() == QEvent::DevicePixelRatioChange))
    {
        m_impl->ScheduleNativeResize(false);
    }
    if (m_impl && event->type() == QEvent::Leave
        && m_impl->dragMode == Impl::DragMode::None)
    {
        if (!m_impl->context.IsNull()) m_impl->context->ClearDetected(Standard_True);
        unsetCursor();
    }
    return handled;
}

void CadModel3DView::showEvent(QShowEvent* event)
{
    QWidget::showEvent(event);
    QString ignored;
    m_impl->EnsureViewer(&ignored);
    m_impl->ResizeStatus();
    m_impl->ScheduleNativeResize(!m_impl->shape.IsNull());
}

void CadModel3DView::paintEvent(QPaintEvent*)
{
    QString ignored;
    if (m_impl->EnsureViewer(&ignored) && !m_impl->view.IsNull()) m_impl->view->Redraw();
    m_impl->UpdateLabelPositions();
}

void CadModel3DView::resizeEvent(QResizeEvent* event)
{
    QWidget::resizeEvent(event);
    m_impl->ScheduleNativeResize(false);
    m_impl->ResizeStatus();
}

void CadModel3DView::mousePressEvent(QMouseEvent* event)
{
    if (m_impl->view.IsNull()) return;
    if (m_impl->dragMode != Impl::DragMode::None)
    {
        // 左键拖动 V 槽时忽略额外中/右键按下，避免 dragMode 被改成
        // Pan 后丢失 V 槽位置与吸附状态回调。
        event->accept();
        return;
    }
    m_impl->lastMousePos = m_impl->OcctPixel(event->position());
    if (event->button() == Qt::LeftButton)
    {
        if (m_impl->HitVSlot(m_impl->lastMousePos, true))
        {
            Vec3d planePoint;
            if (m_impl->VSlotPlanePoint(m_impl->lastMousePos, planePoint))
            {
                m_impl->dragMode = Impl::DragMode::VSlot;
                m_impl->dragStartAnchor = m_impl->displayedVSlotFixture.anchor;
                m_impl->dragStartPlanePoint = planePoint;
                m_impl->vSlotMovedDuringDrag = false;
                m_impl->SetVSlotDragVisual(true);
                setCursor(Qt::ClosedHandCursor);
                event->accept();
                return;
            }
        }
        m_impl->dragMode = Impl::DragMode::Rotate;
        m_impl->view->StartRotation(m_impl->lastMousePos.x(), m_impl->lastMousePos.y());
        setCursor(Qt::ClosedHandCursor);
    }
    else if (event->button() == Qt::MiddleButton || event->button() == Qt::RightButton)
    {
        m_impl->dragMode = Impl::DragMode::Pan;
        setCursor(Qt::SizeAllCursor);
    }
    event->accept();
}

void CadModel3DView::mouseMoveEvent(QMouseEvent* event)
{
    if (m_impl->view.IsNull()) return;
    const QPoint current = m_impl->OcctPixel(event->position());
    if (m_impl->dragMode == Impl::DragMode::None)
    {
        const bool overVSlot = m_impl->HitVSlot(current, true);
        if (overVSlot) setCursor(Qt::OpenHandCursor);
        else unsetCursor();
        m_impl->UpdateLabelPositions();
        if (overVSlot) event->accept();
        else QWidget::mouseMoveEvent(event);
        return;
    }
    if (m_impl->dragMode == Impl::DragMode::VSlot)
    {
        Vec3d planePoint;
        if (m_impl->VSlotPlanePoint(current, planePoint))
        {
            Vec3d normal;
            if (Normalize(m_impl->displayedGroundSurface.axisZ, normal))
            {
                Vec3d displacement = Subtract(planePoint, m_impl->dragStartPlanePoint);
                displacement = Subtract(displacement, Multiply(normal, Dot(displacement, normal)));
                const Vec3d anchor = Add(m_impl->dragStartAnchor, displacement);
                if (Length(Subtract(anchor, m_impl->displayedVSlotFixture.anchor)) > 1.0e-5
                    && m_impl->SetDraggedVSlotAnchor(anchor))
                {
                    m_impl->vSlotMovedDuringDrag = true;
                    // 拖动过程就执行磁吸，而不是只在松手后突然跳变。
                    m_impl->SnapVSlotToGroundFeature();
                    m_impl->SetVSlotDragVisual(true);
                }
            }
        }
        m_impl->lastMousePos = current;
        event->accept();
        return;
    }
    if (m_impl->dragMode == Impl::DragMode::Rotate)
    {
        m_impl->view->Rotation(current.x(), current.y());
    }
    else
    {
        const QPoint delta = current - m_impl->lastMousePos;
        m_impl->view->Pan(delta.x(), -delta.y());
    }
    m_impl->lastMousePos = current;
    m_impl->view->Redraw();
    m_impl->UpdateLabelPositions();
    event->accept();
}

void CadModel3DView::mouseReleaseEvent(QMouseEvent* event)
{
    const bool wasVSlotDrag = m_impl->dragMode == Impl::DragMode::VSlot;
    if (wasVSlotDrag && event->button() != Qt::LeftButton)
    {
        // V 槽拖动由左键开始，只能由左键释放提交。组合鼠标操作中的
        // 右键释放不得导致位置与吸附状态回调不同步。
        event->accept();
        return;
    }
    bool snappedVSlot = false;
    if (wasVSlotDrag)
    {
        snappedVSlot = m_impl->SnapVSlotToGroundFeature();
        m_impl->SetVSlotDragVisual(false);
        if (!m_impl->context.IsNull()) m_impl->context->ClearDetected(Standard_False);
        if (!m_impl->view.IsNull()) m_impl->view->Redraw();
    }
    const bool committedVSlot = wasVSlotDrag
        && (m_impl->vSlotMovedDuringDrag || snappedVSlot);
    const Vec3d committedAnchor = m_impl->vSlotFixture.anchor;
    const VSlotFixture committedFixture = m_impl->vSlotFixture;
    const bool committedSnapState = m_impl->vSlotSnappedToWorkpiece;
    m_impl->dragMode = Impl::DragMode::None;
    m_impl->vSlotMovedDuringDrag = false;
    unsetCursor();
    event->accept();
    if (wasVSlotDrag && event->button() == Qt::LeftButton
        && m_impl->vSlotSnapStateCallback)
        m_impl->vSlotSnapStateCallback(m_impl->vSlotSnappedToWorkpiece);
    if (committedVSlot && m_impl->vSlotPoseChangedCallback)
        m_impl->vSlotPoseChangedCallback(committedFixture, committedSnapState);
    else if (committedVSlot && m_impl->vSlotMovedCallback)
        m_impl->vSlotMovedCallback(committedAnchor);
}

void CadModel3DView::mouseDoubleClickEvent(QMouseEvent* event)
{
    if (event->button() == Qt::LeftButton) FitAll();
    event->accept();
}

void CadModel3DView::wheelEvent(QWheelEvent* event)
{
    if (m_impl->view.IsNull()) return;
    const int delta = event->angleDelta().y();
    if (delta == 0) return;
    const double factor = std::pow(1.0015, static_cast<double>(delta));
    const double nextScale = std::clamp(
        m_impl->view->Scale() * factor,
        1.0e-9,
        1.0e12);
    m_impl->view->SetScale(nextScale);
    m_impl->view->Redraw();
    m_impl->UpdateLabelPositions();
    event->accept();
}
}
