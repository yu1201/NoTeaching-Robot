#include "RobotModelOriginalPreview.h"

#include "RobotCadAssemblyLoader.h"
#include "RobotModelRemoteCatalog.h"
#include "TheoreticalRobotModelStore.h"

#include <BRep_Tool.hxx>
#include <Poly_Triangle.hxx>
#include <Poly_Triangulation.hxx>
#include <Standard_Failure.hxx>
#include <TopAbs_Orientation.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <QColor>
#include <QFileInfo>
#include <QFont>
#include <QFontDatabase>
#include <QLinearGradient>
#include <QPainter>
#include <QPointF>
#include <QRectF>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <new>
#include <vector>

namespace
{
constexpr qint64 kMaximumPreviewTriangles = 8LL * 1000LL * 1000LL;

struct ProjectedVertex
{
    double x = 0.0;
    double y = 0.0;
    double depth = 0.0;
};

bool IsSafeDisplayName(const QString& value)
{
    if (value.isEmpty() || value.size() > 256) return false;
    for (const QChar ch : value)
    {
        if (ch.unicode() < 0x20 || ch.unicode() == 0x7f) return false;
    }
    return true;
}

QFont PreviewFont(const QFont& base)
{
    QFont font(base);
    const QStringList families = QFontDatabase::families();
    const QStringList candidates = {
        QStringLiteral("Microsoft YaHei UI"),
        QStringLiteral("Microsoft YaHei"),
        QStringLiteral("微软雅黑"),
        QStringLiteral("SimHei"),
        QStringLiteral("黑体"),
        QStringLiteral("Noto Sans CJK SC"),
        QStringLiteral("Arial")
    };
    for (const QString& candidate : candidates)
    {
        if (!families.contains(candidate, Qt::CaseInsensitive)) continue;
        font.setFamily(candidate);
        break;
    }
    return font;
}

QPointF ProjectPoint(const Eigen::Vector3d& point)
{
    // 源机器人模型约定 +Y 向上。该等轴投影让 +Y 保持屏幕向上，同时能看到
    // X/Z 两个水平方向，缩略图不会退化成单一侧面的轮廓。
    return QPointF(
        0.866025403784 * (point.x() - point.z()),
        -point.y() + 0.5 * (point.x() + point.z()));
}

double ProjectDepth(const Eigen::Vector3d& point)
{
    return point.x() + point.y() + point.z();
}

double Edge(
    const ProjectedVertex& first,
    const ProjectedVertex& second,
    double x,
    double y)
{
    return (second.x - first.x) * (y - first.y)
        - (second.y - first.y) * (x - first.x);
}

bool RenderLoadedAssembly(
    const RobotCadAssemblyLoader::Result& assembly,
    const QString& displayName,
    QImage& image,
    QString& error)
{
    image = QImage();
    if (!assembly.assemblyShape
        || assembly.assemblyShape->IsNull()
        || !assembly.statistics.detailedPresentationBuilt
        || !assembly.statistics.displayTriangulationPrepared
        || !assembly.statistics.assemblyBoundsMm.valid)
    {
        error = QStringLiteral("原始 STEP 总装没有可供缩略图使用的完整 B-Rep 显示数据。");
        return false;
    }

    const Eigen::Vector3d minimum =
        assembly.statistics.assemblyBoundsMm.minimumMm;
    const Eigen::Vector3d maximum =
        assembly.statistics.assemblyBoundsMm.maximumMm;
    if (!minimum.allFinite() || !maximum.allFinite()
        || (maximum - minimum).minCoeff() < 0.0)
    {
        error = QStringLiteral("原始 STEP 总装包围盒无效。");
        return false;
    }

    QRectF projectedBounds;
    for (int x = 0; x < 2; ++x)
        for (int y = 0; y < 2; ++y)
            for (int z = 0; z < 2; ++z)
            {
                const QPointF projected = ProjectPoint(Eigen::Vector3d(
                    x ? maximum.x() : minimum.x(),
                    y ? maximum.y() : minimum.y(),
                    z ? maximum.z() : minimum.z()));
                projectedBounds |= QRectF(projected, QSizeF(0.01, 0.01));
            }
    if (!projectedBounds.isValid()
        || projectedBounds.width() <= 1.0e-9
        || projectedBounds.height() <= 1.0e-9)
    {
        error = QStringLiteral("原始 STEP 总装投影范围退化。");
        return false;
    }

    QImage rendered(
        RobotModelRemoteCatalog::PreviewWidth,
        RobotModelRemoteCatalog::PreviewHeight,
        QImage::Format_ARGB32);
    QPainter backgroundPainter(&rendered);
    QLinearGradient background(
        0, 0, 0, RobotModelRemoteCatalog::PreviewHeight);
    background.setColorAt(0.0, QColor(14, 55, 72));
    background.setColorAt(1.0, QColor(4, 15, 23));
    backgroundPainter.fillRect(rendered.rect(), background);
    backgroundPainter.end();

    const QRectF target(
        30.0,
        42.0,
        RobotModelRemoteCatalog::PreviewWidth - 60.0,
        RobotModelRemoteCatalog::PreviewHeight - 78.0);
    const double scale = std::min(
        target.width() / projectedBounds.width(),
        target.height() / projectedBounds.height());
    const QPointF sourceCenter = projectedBounds.center();
    const QPointF targetCenter = target.center();
    auto projectToScreen = [&](const Eigen::Vector3d& point)
    {
        const QPointF projected = ProjectPoint(point);
        const QPointF screen =
            targetCenter + (projected - sourceCenter) * scale;
        return ProjectedVertex{
            screen.x(), screen.y(), ProjectDepth(point)
        };
    };

    const int width = rendered.width();
    const int height = rendered.height();
    std::vector<double> depthBuffer(
        static_cast<size_t>(width * height),
        -std::numeric_limits<double>::infinity());
    const Eigen::Vector3d lightDirection =
        Eigen::Vector3d(-0.35, 0.78, 0.52).normalized();
    qint64 triangleCount = 0;
    qint64 paintedPixelCount = 0;

    for (TopExp_Explorer explorer(
             *assembly.assemblyShape, TopAbs_FACE);
         explorer.More();
         explorer.Next())
    {
        const TopoDS_Face face = TopoDS::Face(explorer.Current());
        TopLoc_Location location;
        const Handle(Poly_Triangulation) triangulation =
            BRep_Tool::Triangulation(face, location);
        if (triangulation.IsNull()) continue;
        const gp_Trsf transform = location.Transformation();

        for (Standard_Integer triangleIndex = 1;
             triangleIndex <= triangulation->NbTriangles();
             ++triangleIndex)
        {
            if (++triangleCount > kMaximumPreviewTriangles)
            {
                error = QStringLiteral(
                    "原始 STEP 缩略图三角形超过安全上限（%1）。")
                    .arg(kMaximumPreviewTriangles);
                return false;
            }

            Standard_Integer indices[3] = { 0, 0, 0 };
            triangulation->Triangle(triangleIndex).Get(
                indices[0], indices[1], indices[2]);
            if (face.Orientation() == TopAbs_REVERSED)
                std::swap(indices[1], indices[2]);

            std::array<Eigen::Vector3d, 3> world{};
            std::array<ProjectedVertex, 3> screen{};
            bool valid = true;
            for (int vertexIndex = 0; vertexIndex < 3; ++vertexIndex)
            {
                gp_Pnt point = triangulation->Node(indices[vertexIndex]);
                point.Transform(transform);
                world[static_cast<size_t>(vertexIndex)] =
                    Eigen::Vector3d(point.X(), point.Y(), point.Z());
                if (!world[static_cast<size_t>(vertexIndex)].allFinite())
                {
                    valid = false;
                    break;
                }
                screen[static_cast<size_t>(vertexIndex)] =
                    projectToScreen(
                        world[static_cast<size_t>(vertexIndex)]);
            }
            if (!valid) continue;

            const double signedArea = Edge(
                screen[0], screen[1], screen[2].x, screen[2].y);
            if (!std::isfinite(signedArea)
                || std::abs(signedArea) <= 1.0e-9)
            {
                continue;
            }

            const Eigen::Vector3d normal =
                (world[1] - world[0]).cross(world[2] - world[0]);
            const double normalLength = normal.norm();
            if (!std::isfinite(normalLength) || normalLength <= 1.0e-12)
                continue;
            const double diffuse = std::abs(
                normal.dot(lightDirection) / normalLength);
            const double brightness = std::clamp(
                0.32 + diffuse * 0.68, 0.0, 1.0);
            const int red = static_cast<int>(
                std::clamp(42.0 + 58.0 * brightness, 0.0, 255.0));
            const int green = static_cast<int>(
                std::clamp(106.0 + 112.0 * brightness, 0.0, 255.0));
            const int blue = static_cast<int>(
                std::clamp(142.0 + 105.0 * brightness, 0.0, 255.0));
            const QRgb color = qRgb(red, green, blue);

            const int minX = std::max(
                static_cast<int>(target.left()),
                static_cast<int>(std::floor(std::min({
                    screen[0].x, screen[1].x, screen[2].x }))));
            const int maxX = std::min(
                static_cast<int>(target.right()),
                static_cast<int>(std::ceil(std::max({
                    screen[0].x, screen[1].x, screen[2].x }))));
            const int minY = std::max(
                static_cast<int>(target.top()),
                static_cast<int>(std::floor(std::min({
                    screen[0].y, screen[1].y, screen[2].y }))));
            const int maxY = std::min(
                static_cast<int>(target.bottom()),
                static_cast<int>(std::ceil(std::max({
                    screen[0].y, screen[1].y, screen[2].y }))));
            if (minX > maxX || minY > maxY) continue;

            for (int y = minY; y <= maxY; ++y)
            {
                QRgb* row = reinterpret_cast<QRgb*>(
                    rendered.scanLine(y));
                for (int x = minX; x <= maxX; ++x)
                {
                    const double sampleX = x + 0.5;
                    const double sampleY = y + 0.5;
                    const double weight0 =
                        Edge(screen[1], screen[2], sampleX, sampleY)
                        / signedArea;
                    const double weight1 =
                        Edge(screen[2], screen[0], sampleX, sampleY)
                        / signedArea;
                    const double weight2 = 1.0 - weight0 - weight1;
                    constexpr double tolerance = -1.0e-7;
                    if (weight0 < tolerance
                        || weight1 < tolerance
                        || weight2 < tolerance)
                    {
                        continue;
                    }
                    const double depth =
                        weight0 * screen[0].depth
                        + weight1 * screen[1].depth
                        + weight2 * screen[2].depth;
                    const size_t offset =
                        static_cast<size_t>(y * width + x);
                    if (depth <= depthBuffer[offset]) continue;
                    depthBuffer[offset] = depth;
                    row[x] = color;
                    ++paintedPixelCount;
                }
            }
        }
    }

    if (triangleCount <= 0 || paintedPixelCount <= 0)
    {
        error = QStringLiteral(
            "原始 STEP 总装没有生成可见的缩略图曲面。");
        return false;
    }

    // 根据深度突变和模型外轮廓加一层细暗边，保留 CAD 零件层次，同时不画
    // 碰撞盒线框。这里只处理已经通过 z-buffer 的最终可见像素。
    const double depthThreshold =
        std::max(1.0, (maximum - minimum).norm() * 0.004);
    QImage outlined = rendered;
    for (int y = 1; y < height - 1; ++y)
    {
        QRgb* row = reinterpret_cast<QRgb*>(outlined.scanLine(y));
        for (int x = 1; x < width - 1; ++x)
        {
            const size_t offset = static_cast<size_t>(y * width + x);
            const double depth = depthBuffer[offset];
            if (!std::isfinite(depth)) continue;
            const std::array<size_t, 4> neighbours = {
                offset - 1,
                offset + 1,
                offset - static_cast<size_t>(width),
                offset + static_cast<size_t>(width)
            };
            bool edgePixel = false;
            for (const size_t neighbour : neighbours)
            {
                if (!std::isfinite(depthBuffer[neighbour])
                    || std::abs(depth - depthBuffer[neighbour])
                        > depthThreshold)
                {
                    edgePixel = true;
                    break;
                }
            }
            if (!edgePixel) continue;
            const QColor existing(row[x]);
            row[x] = qRgb(
                existing.red() * 55 / 100,
                existing.green() * 55 / 100,
                existing.blue() * 55 / 100);
        }
    }

    QPainter painter(&outlined);
    painter.setRenderHint(QPainter::TextAntialiasing, true);
    painter.setFont(PreviewFont(painter.font()));
    painter.setPen(QColor(226, 242, 248));
    QFont titleFont = painter.font();
    titleFont.setBold(true);
    titleFont.setPointSize(12);
    painter.setFont(titleFont);
    painter.drawText(
        QRectF(
            18.0, 10.0,
            RobotModelRemoteCatalog::PreviewWidth - 36.0, 28.0),
        Qt::AlignLeft | Qt::AlignVCenter,
        displayName);
    QFont noteFont = painter.font();
    noteFont.setBold(false);
    noteFont.setPointSize(8);
    painter.setFont(noteFont);
    painter.setPen(QColor(144, 202, 224));
    painter.drawText(
        QRectF(
            18.0,
            RobotModelRemoteCatalog::PreviewHeight - 30.0,
            RobotModelRemoteCatalog::PreviewWidth - 36.0,
            20.0),
        Qt::AlignLeft | Qt::AlignVCenter,
        QStringLiteral("原始 STEP 总装预览 · %1 个曲面")
            .arg(assembly.statistics.faceCount));
    painter.end();

    image = outlined;
    return true;
}
}

bool RobotModelOriginalPreview::RenderStepFile(
    const QString& stepFilePath,
    const QString& displayName,
    QImage& image,
    QString& error)
{
    image = QImage();
    error.clear();
    const QFileInfo source(stepFilePath);
    if (!source.exists() || !source.isFile() || source.isSymLink()
        || !IsSafeDisplayName(displayName))
    {
        error = QStringLiteral(
            "原始 STEP 缩略图输入文件或显示名称无效。");
        return false;
    }

    try
    {
        RobotCadAssemblyLoader::Options options;
        options.includePipeline = false;
        options.buildDetailedPresentation = true;
        options.prepareDisplayTriangulation = true;
        options.maximumFileBytes =
            TheoreticalRobotModelStore::MaximumAssetBytes;
        RobotCadAssemblyLoader::Result assembly;
        if (!RobotCadAssemblyLoader::LoadFile(
                source.absoluteFilePath(),
                assembly,
                error,
                &options))
        {
            return false;
        }
        return RenderLoadedAssembly(assembly, displayName, image, error);
    }
    catch (const Standard_Failure& exception)
    {
        error = QStringLiteral("生成原始 STEP 缩略图时 OCCT 异常：%1")
            .arg(QString::fromLocal8Bit(
                exception.GetMessageString()).simplified());
    }
    catch (const std::bad_alloc&)
    {
        error = QStringLiteral("生成原始 STEP 缩略图时内存不足。");
    }
    catch (const std::exception& exception)
    {
        error = QStringLiteral("生成原始 STEP 缩略图时异常：%1")
            .arg(QString::fromLocal8Bit(exception.what()).simplified());
    }
    catch (...)
    {
        error = QStringLiteral("生成原始 STEP 缩略图时发生未知异常。");
    }
    image = QImage();
    return false;
}
