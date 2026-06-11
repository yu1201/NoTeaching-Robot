#include "WorkpieceMeshBuilder.h"

#include "RobotDataHelper.h"

#include <opencv2/imgproc.hpp>

#include <QDataStream>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QTextStream>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace
{
constexpr auto WORKPIECE_MESH_FILE_NAME = "PreciseLaserPoint_WorkpieceMesh.ply";
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

bool IsFiniteVec(const Eigen::Vector3f& v)
{
    return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
}

double MedianOf(std::vector<double>& values)
{
    if (values.empty())
    {
        return 0.0;
    }
    const size_t mid = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + mid, values.end());
    return values[mid];
}

// 帧（扫描线）切分：帧内沿横向轴单调推进，新帧从横向起点重新开始。
// 以整体包围盒的短轴为横向轴；横向标量相对帧内已推进方向回退超过阈值即认为换帧。
QVector<QPair<int, int>> SplitScanFrames(
    const QVector<RobotCalculation::IndexedPoint3D>& points,
    int* transverseAxis)
{
    QVector<QPair<int, int>> frames;
    if (points.size() < 4)
    {
        return frames;
    }

    Eigen::Vector3d minBound = points.first().point;
    Eigen::Vector3d maxBound = points.first().point;
    for (const auto& p : points)
    {
        minBound = minBound.cwiseMin(p.point);
        maxBound = maxBound.cwiseMax(p.point);
    }
    const Eigen::Vector3d span = maxBound - minBound;
    // 横向轴 = XY 中跨度较小的一维（焊缝沿长轴推进，扫描线沿短轴展开）。
    const int axis = span.x() <= span.y() ? 0 : 1;
    if (transverseAxis != nullptr)
    {
        *transverseAxis = axis;
    }

    // 帧内相邻点横向步距中位数，用于回退阈值。
    std::vector<double> steps;
    steps.reserve(1024);
    for (int i = 1; i < std::min<int>(points.size(), 5000); ++i)
    {
        const double d = std::abs(points[i].point[axis] - points[i - 1].point[axis]);
        if (d > 1e-9)
        {
            steps.push_back(d);
        }
    }
    const double stepMedian = std::max(1e-3, MedianOf(steps));
    const double backJump = stepMedian * 20.0;  // 回退超过 20 倍点距 = 换帧

    int frameStart = 0;
    double direction = 0.0;  // 帧内推进方向（符号），首两点确定
    for (int i = 1; i < points.size(); ++i)
    {
        const double delta = points[i].point[axis] - points[i - 1].point[axis];
        if (direction == 0.0 && std::abs(delta) > 1e-9)
        {
            direction = delta > 0.0 ? 1.0 : -1.0;
        }
        // 逆向大幅回退 → 新帧
        if (direction != 0.0 && delta * direction < -backJump)
        {
            if (i - frameStart >= 3)
            {
                frames.push_back({ frameStart, i });
            }
            frameStart = i;
            direction = 0.0;
        }
    }
    if (points.size() - frameStart >= 3)
    {
        frames.push_back({ frameStart, points.size() });
    }
    return frames;
}
}

QString WorkpieceMeshBuilder::MeshCacheFileName()
{
    return QString::fromLatin1(WORKPIECE_MESH_FILE_NAME);
}

QString WorkpieceMeshBuilder::MeshCachePath(const QString& laserDir)
{
    return QDir(laserDir).filePath(MeshCacheFileName());
}

bool WorkpieceMeshBuilder::BuildFromScanlineCloud(
    const QVector<RobotCalculation::IndexedPoint3D>& cloudPoints,
    Mesh& mesh,
    QString& error,
    int targetMaxTriangles)
{
    mesh = Mesh();
    if (cloudPoints.size() < 16)
    {
        error = QString("点云点数过少（%1），无法网格化。").arg(cloudPoints.size());
        return false;
    }

    int axis = 0;
    const QVector<QPair<int, int>> frames = SplitScanFrames(cloudPoints, &axis);
    if (frames.size() < 3)
    {
        error = QString("扫描线分帧失败（仅 %1 帧），点云可能无序。").arg(frames.size());
        return false;
    }

    // 横向范围与目标列数：列数取各帧点数中位（保细节），按目标三角规模行列联合抽稀。
    double tMin = std::numeric_limits<double>::max();
    double tMax = std::numeric_limits<double>::lowest();
    std::vector<double> frameSizes;
    frameSizes.reserve(frames.size());
    for (const auto& f : frames)
    {
        frameSizes.push_back(double(f.second - f.first));
        for (int i = f.first; i < f.second; ++i)
        {
            const double t = cloudPoints[i].point[axis];
            tMin = std::min(tMin, t);
            tMax = std::max(tMax, t);
        }
    }
    if (tMax - tMin < 1e-6)
    {
        error = "横向跨度为零，无法网格化。";
        return false;
    }
    int columns = static_cast<int>(std::clamp(MedianOf(frameSizes), 16.0, 4096.0));
    int rows = frames.size();
    if (targetMaxTriangles > 0)
    {
        // 三角数 ≈ 2*(rows-1)*(columns-1)，超目标时行列等比抽稀。
        const double tri = 2.0 * (rows - 1) * (columns - 1);
        if (tri > targetMaxTriangles)
        {
            const double scale = std::sqrt(targetMaxTriangles / tri);
            rows = std::max(3, static_cast<int>(rows * scale));
            columns = std::max(16, static_cast<int>(columns * scale));
        }
    }

    // 帧抽稀映射 + 每帧按横向参数重采样到固定列（线性插值；缺口超过阈值的列填 NaN）。
    std::vector<double> gapSamples;
    gapSamples.reserve(4096);
    {
        const auto& f0 = frames[frames.size() / 2];
        for (int i = f0.first + 1; i < f0.second; ++i)
        {
            gapSamples.push_back(std::abs(cloudPoints[i].point[axis] - cloudPoints[i - 1].point[axis]));
        }
    }
    const double columnWidth = (tMax - tMin) / std::max(1, columns - 1);
    const double gapMedian = std::max(MedianOf(gapSamples), 1e-3);
    const double bridgeLimit = std::max(gapMedian, columnWidth) * 4.0;  // 缺口超过 4 倍点距不桥接

    QVector<Eigen::Vector3f> grid(static_cast<qsizetype>(rows) * columns,
        Eigen::Vector3f(kNaN, kNaN, kNaN));
    for (int r = 0; r < rows; ++r)
    {
        const auto& frame = frames[static_cast<int>(
            double(r) * (frames.size() - 1) / std::max(1, rows - 1))];
        // 帧内点按横向参数排序后插值（蛇形扫描方向交替也被归一）。
        std::vector<std::pair<double, Eigen::Vector3d>> line;
        line.reserve(frame.second - frame.first);
        for (int i = frame.first; i < frame.second; ++i)
        {
            line.push_back({ cloudPoints[i].point[axis], cloudPoints[i].point });
        }
        std::sort(line.begin(), line.end(),
            [](const auto& a, const auto& b) { return a.first < b.first; });

        size_t cursor = 0;
        for (int c = 0; c < columns; ++c)
        {
            const double t = tMin + columnWidth * c;
            while (cursor + 1 < line.size() && line[cursor + 1].first < t)
            {
                ++cursor;
            }
            if (cursor + 1 >= line.size())
            {
                break;
            }
            const auto& a = line[cursor];
            const auto& b = line[cursor + 1];
            if (t < a.first - bridgeLimit || t > b.first + bridgeLimit
                || (b.first - a.first) > bridgeLimit)
            {
                continue;  // 该列落在数据缺口里，保持 NaN
            }
            const double w = (b.first - a.first) > 1e-12
                ? std::clamp((t - a.first) / (b.first - a.first), 0.0, 1.0)
                : 0.0;
            const Eigen::Vector3d p = a.second * (1.0 - w) + b.second * w;
            grid[static_cast<qsizetype>(r) * columns + c] = p.cast<float>();
        }
    }

    // 2×2 单元三角化：NaN 跳过 + 最大边长阈值（防止把遮挡缺口/帧间断裂糊死）。
    const float maxEdge = static_cast<float>(
        std::max(columnWidth, (tMax - tMin) / columns) * 8.0
        + 8.0);
    const float maxEdgeSq = maxEdge * maxEdge;
    auto vertexAt = [&grid, columns](int r, int c) -> const Eigen::Vector3f&
        {
            return grid[static_cast<qsizetype>(r) * columns + c];
        };
    auto edgeOk = [maxEdgeSq](const Eigen::Vector3f& a, const Eigen::Vector3f& b)
        {
            return (a - b).squaredNorm() <= maxEdgeSq;
        };

    // 顶点压缩：仅保留被三角形引用的网格点。
    QVector<qint32> vertexIndex(grid.size(), -1);
    mesh.vertices.reserve(grid.size() / 2);
    mesh.indices.reserve(static_cast<qsizetype>(rows) * columns);
    auto ensureVertex = [&](int r, int c) -> qint32
        {
            const qsizetype flat = static_cast<qsizetype>(r) * columns + c;
            if (vertexIndex[flat] < 0)
            {
                vertexIndex[flat] = mesh.vertices.size();
                mesh.vertices.push_back(grid[flat]);
            }
            return vertexIndex[flat];
        };
    auto addTriangle = [&](int r0, int c0, int r1, int c1, int r2, int c2)
        {
            const Eigen::Vector3f& a = vertexAt(r0, c0);
            const Eigen::Vector3f& b = vertexAt(r1, c1);
            const Eigen::Vector3f& c = vertexAt(r2, c2);
            if (!IsFiniteVec(a) || !IsFiniteVec(b) || !IsFiniteVec(c))
            {
                return;
            }
            if (!edgeOk(a, b) || !edgeOk(b, c) || !edgeOk(c, a))
            {
                return;
            }
            mesh.indices.push_back(static_cast<quint32>(ensureVertex(r0, c0)));
            mesh.indices.push_back(static_cast<quint32>(ensureVertex(r1, c1)));
            mesh.indices.push_back(static_cast<quint32>(ensureVertex(r2, c2)));
        };
    for (int r = 0; r + 1 < rows; ++r)
    {
        for (int c = 0; c + 1 < columns; ++c)
        {
            // 固定对角线切法（数据近规则，自适应增益有限）：
            // (r,c)-(r+1,c)-(r,c+1) 与 (r,c+1)-(r+1,c)-(r+1,c+1)
            addTriangle(r, c, r + 1, c, r, c + 1);
            addTriangle(r, c + 1, r + 1, c, r + 1, c + 1);
        }
    }

    if (!mesh.IsValid())
    {
        error = "三角化结果为空（点云可能严重缺失或分帧异常）。";
        return false;
    }

    // 顶点法线 = 邻接面法线累加归一；统一让平均法线朝 +Z（朝上看工件）。
    mesh.normals = QVector<Eigen::Vector3f>(mesh.vertices.size(), Eigen::Vector3f::Zero());
    double sumZ = 0.0;
    for (qsizetype i = 0; i + 2 < mesh.indices.size(); i += 3)
    {
        const Eigen::Vector3f& a = mesh.vertices[mesh.indices[i]];
        const Eigen::Vector3f& b = mesh.vertices[mesh.indices[i + 1]];
        const Eigen::Vector3f& c = mesh.vertices[mesh.indices[i + 2]];
        const Eigen::Vector3f n = (b - a).cross(c - a);
        mesh.normals[mesh.indices[i]] += n;
        mesh.normals[mesh.indices[i + 1]] += n;
        mesh.normals[mesh.indices[i + 2]] += n;
        sumZ += n.z();
    }
    if (sumZ < 0.0)
    {
        // 平均朝下：统一翻转绕序与法线。
        for (qsizetype i = 0; i + 2 < mesh.indices.size(); i += 3)
        {
            std::swap(mesh.indices[i + 1], mesh.indices[i + 2]);
        }
        for (auto& n : mesh.normals)
        {
            n = -n;
        }
    }
    for (auto& n : mesh.normals)
    {
        const float len = n.norm();
        n = len > 1e-9f ? Eigen::Vector3f(n / len) : Eigen::Vector3f(0.f, 0.f, 1.f);
    }
    return true;
}

bool WorkpieceMeshBuilder::SaveMeshPly(const QString& filePath, const Mesh& mesh, QString& error)
{
    if (!mesh.IsValid())
    {
        error = "网格为空，未写出模型文件。";
        return false;
    }
    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate))
    {
        error = QString("写入模型文件失败：%1").arg(filePath);
        return false;
    }
    const qsizetype faceCount = mesh.indices.size() / 3;
    const QByteArray header = QStringLiteral(
        "ply\n"
        "format binary_little_endian 1.0\n"
        "comment NoTeaching-Robot workpiece mesh cache\n"
        "element vertex %1\n"
        "property float x\nproperty float y\nproperty float z\n"
        "property float nx\nproperty float ny\nproperty float nz\n"
        "element face %2\n"
        "property list uchar uint vertex_indices\n"
        "end_header\n")
        .arg(mesh.vertices.size())
        .arg(faceCount)
        .toLatin1();
    file.write(header);

    QByteArray vertexBlock;
    vertexBlock.resize(mesh.vertices.size() * 6 * sizeof(float));
    float* vp = reinterpret_cast<float*>(vertexBlock.data());
    for (qsizetype i = 0; i < mesh.vertices.size(); ++i)
    {
        const Eigen::Vector3f& v = mesh.vertices[i];
        const Eigen::Vector3f& n = mesh.normals[i];
        *vp++ = v.x(); *vp++ = v.y(); *vp++ = v.z();
        *vp++ = n.x(); *vp++ = n.y(); *vp++ = n.z();
    }
    file.write(vertexBlock);

    QByteArray faceBlock;
    faceBlock.resize(faceCount * (1 + 3 * sizeof(quint32)));
    char* fp = faceBlock.data();
    for (qsizetype i = 0; i + 2 < mesh.indices.size(); i += 3)
    {
        *fp++ = char(3);
        quint32 idx[3] = { mesh.indices[i], mesh.indices[i + 1], mesh.indices[i + 2] };
        std::memcpy(fp, idx, sizeof(idx));
        fp += sizeof(idx);
    }
    file.write(faceBlock);
    file.close();
    return true;
}

bool WorkpieceMeshBuilder::LoadMeshPly(const QString& filePath, Mesh& mesh, QString& error)
{
    mesh = Mesh();
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly))
    {
        error = QString("打开模型文件失败：%1").arg(filePath);
        return false;
    }
    // 仅解析自家写出的 header 形态（binary_little_endian + xyz/nxnynz + uchar+uint face）。
    qsizetype vertexCount = 0;
    qsizetype faceCount = 0;
    bool headerEnd = false;
    while (!file.atEnd())
    {
        const QByteArray line = file.readLine().trimmed();
        if (line.startsWith("element vertex"))
        {
            vertexCount = line.mid(15).trimmed().toLongLong();
        }
        else if (line.startsWith("element face"))
        {
            faceCount = line.mid(13).trimmed().toLongLong();
        }
        else if (line == "end_header")
        {
            headerEnd = true;
            break;
        }
        else if (line.startsWith("format") && !line.contains("binary_little_endian"))
        {
            error = "模型文件格式不是二进制 PLY。";
            return false;
        }
    }
    if (!headerEnd || vertexCount < 3 || faceCount < 1)
    {
        error = "模型文件头不完整。";
        return false;
    }

    QByteArray vertexBlock = file.read(vertexCount * 6 * sizeof(float));
    if (vertexBlock.size() != vertexCount * 6 * static_cast<qsizetype>(sizeof(float)))
    {
        error = "模型文件顶点数据不完整。";
        return false;
    }
    mesh.vertices.resize(vertexCount);
    mesh.normals.resize(vertexCount);
    const float* vp = reinterpret_cast<const float*>(vertexBlock.constData());
    for (qsizetype i = 0; i < vertexCount; ++i)
    {
        mesh.vertices[i] = Eigen::Vector3f(vp[0], vp[1], vp[2]);
        mesh.normals[i] = Eigen::Vector3f(vp[3], vp[4], vp[5]);
        vp += 6;
    }

    QByteArray faceBlock = file.read(faceCount * (1 + 3 * static_cast<qsizetype>(sizeof(quint32))));
    if (faceBlock.size() != faceCount * (1 + 3 * static_cast<qsizetype>(sizeof(quint32))))
    {
        error = "模型文件面数据不完整。";
        return false;
    }
    mesh.indices.resize(faceCount * 3);
    const char* fp = faceBlock.constData();
    for (qsizetype i = 0; i < faceCount; ++i)
    {
        if (*fp++ != char(3))
        {
            error = "模型文件存在非三角面。";
            return false;
        }
        quint32 idx[3];
        std::memcpy(idx, fp, sizeof(idx));
        fp += sizeof(idx);
        mesh.indices[i * 3] = idx[0];
        mesh.indices[i * 3 + 1] = idx[1];
        mesh.indices[i * 3 + 2] = idx[2];
    }
    return mesh.IsValid();
}

bool WorkpieceMeshBuilder::EnsureMeshCache(
    const QString& laserDir,
    const QString& cloudFilePath,
    QString& error)
{
    const QString cachePath = MeshCachePath(laserDir);
    if (QFileInfo::exists(cachePath))
    {
        return true;
    }
    if (!QFileInfo::exists(cloudFilePath))
    {
        error = QString("缺少完整点云文件：%1").arg(cloudFilePath);
        return false;
    }
    QVector<RobotCalculation::IndexedPoint3D> cloudPoints;
    if (!RobotDataHelper::LoadIndexedPoint3DFile(cloudFilePath, cloudPoints, &error))
    {
        return false;
    }
    Mesh mesh;
    if (!BuildFromScanlineCloud(cloudPoints, mesh, error))
    {
        return false;
    }
    return SaveMeshPly(cachePath, mesh, error);
}

QImage WorkpieceMeshBuilder::RenderHeightMap(const Mesh& mesh, int maxImageWidth)
{
    if (mesh.vertices.isEmpty())
    {
        return QImage();
    }
    Eigen::Vector3f minBound = mesh.vertices.first();
    Eigen::Vector3f maxBound = mesh.vertices.first();
    for (const auto& v : mesh.vertices)
    {
        minBound = minBound.cwiseMin(v);
        maxBound = maxBound.cwiseMax(v);
    }
    const float spanX = std::max(1e-3f, maxBound.x() - minBound.x());
    const float spanY = std::max(1e-3f, maxBound.y() - minBound.y());

    int width = maxImageWidth;
    int height = static_cast<int>(width * (spanY / spanX));
    if (height > maxImageWidth * 4)
    {
        height = maxImageWidth * 4;
        width = std::max(64, static_cast<int>(height * (spanX / spanY)));
    }
    height = std::clamp(height, 64, 8192);

    cv::Mat heightField(height, width, CV_32FC1, cv::Scalar(std::numeric_limits<float>::quiet_NaN()));
    for (const auto& v : mesh.vertices)
    {
        const int px = std::clamp(static_cast<int>((v.x() - minBound.x()) / spanX * (width - 1)), 0, width - 1);
        const int py = std::clamp(static_cast<int>((v.y() - minBound.y()) / spanY * (height - 1)), 0, height - 1);
        float& cell = heightField.at<float>(height - 1 - py, px);  // Y 向上
        if (std::isnan(cell) || v.z() > cell)
        {
            cell = v.z();  // 同格取最高（俯视可见面）
        }
    }

    // 小孔填补（近邻扩散一次）后做梯度光照。
    cv::Mat mask = heightField != heightField;  // NaN mask
    cv::Mat filled = heightField.clone();
    cv::patchNaNs(filled, minBound.z());
    cv::Mat blurred;
    cv::blur(filled, blurred, cv::Size(3, 3));
    filled.setTo(0, mask);
    blurred.copyTo(filled, mask);

    cv::Mat gradX;
    cv::Mat gradY;
    cv::Sobel(filled, gradX, CV_32F, 1, 0, 3);
    cv::Sobel(filled, gradY, CV_32F, 0, 1, 3);

    // Lambert 简化光照（光源左上方）+ 高度伪彩。
    cv::Mat shade(height, width, CV_32FC1);
    const float lx = -0.5f;
    const float ly = -0.5f;
    const float lz = 1.0f;
    const float lightLen = std::sqrt(lx * lx + ly * ly + lz * lz);
    for (int y = 0; y < height; ++y)
    {
        const float* gx = gradX.ptr<float>(y);
        const float* gy = gradY.ptr<float>(y);
        float* s = shade.ptr<float>(y);
        for (int x = 0; x < width; ++x)
        {
            const float nx = -gx[x];
            const float ny = -gy[x];
            const float nz = 1.0f;
            const float len = std::sqrt(nx * nx + ny * ny + nz * nz) * lightLen;
            s[x] = std::clamp((nx * lx + ny * ly + nz * lz) / std::max(len, 1e-6f), 0.0f, 1.0f);
        }
    }

    cv::Mat heightNorm;
    cv::normalize(filled, heightNorm, 0, 255, cv::NORM_MINMAX, CV_8UC1, ~mask);
    cv::Mat colored;
    cv::applyColorMap(heightNorm, colored, cv::COLORMAP_TURBO);
    for (int y = 0; y < height; ++y)
    {
        cv::Vec3b* row = colored.ptr<cv::Vec3b>(y);
        const float* s = shade.ptr<float>(y);
        const uchar* m = mask.ptr<uchar>(y);
        for (int x = 0; x < width; ++x)
        {
            if (m[x])
            {
                row[x] = cv::Vec3b(24, 18, 14);  // 无数据区深色
                continue;
            }
            const float k = 0.35f + 0.65f * s[x];
            row[x] = cv::Vec3b(
                static_cast<uchar>(row[x][0] * k),
                static_cast<uchar>(row[x][1] * k),
                static_cast<uchar>(row[x][2] * k));
        }
    }

    cv::Mat rgb;
    cv::cvtColor(colored, rgb, cv::COLOR_BGR2RGB);
    return QImage(rgb.data, rgb.cols, rgb.rows, static_cast<int>(rgb.step), QImage::Format_RGB888).copy();
}
