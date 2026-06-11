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
#include <cstdlib>
#include <cstring>
#include <limits>
#include <vector>

namespace
{
constexpr auto WORKPIECE_MESH_FILE_NAME = "PreciseLaserPoint_WorkpieceMesh.ply";
// 网格化算法版本：写入 PLY 注释头；EnsureMeshCache 据此让旧算法生成的缓存自动失效重建。
constexpr auto WORKPIECE_MESH_CACHE_TAG = "NoTeaching-Robot workpiece mesh cache v3";
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

// 帧（扫描线）切分：帧内相邻点距很小（亚毫米级），帧间跳变是从一条线尾部跳到
// 下一条线头部的大距离。用 3D 相邻点距突变切帧——与帧内坐标折返（波纹起伏、
// 竖立板）完全无关，对任意工件摆放姿态都成立。
QVector<QPair<int, int>> SplitScanFrames(
    const QVector<RobotCalculation::IndexedPoint3D>& points)
{
    QVector<QPair<int, int>> frames;
    if (points.size() < 4)
    {
        return frames;
    }

    // 相邻点距中位数（帧内典型点距）。
    std::vector<double> steps;
    steps.reserve(8192);
    const int sampleLimit = std::min<int>(points.size(), 20000);
    for (int i = 1; i < sampleLimit; ++i)
    {
        const double d = (points[i].point - points[i - 1].point).norm();
        if (d > 1e-9)
        {
            steps.push_back(d);
        }
    }
    const double stepMedian = std::max(1e-3, MedianOf(steps));
    const double frameJump = stepMedian * 15.0;  // 相邻点距超过 15 倍中位点距 = 换帧

    int frameStart = 0;
    for (int i = 1; i < points.size(); ++i)
    {
        if ((points[i].point - points[i - 1].point).norm() > frameJump)
        {
            if (i - frameStart >= 3)
            {
                frames.push_back({ frameStart, i });
            }
            frameStart = i;
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

bool WorkpieceMeshBuilder::IsMeshCacheValid(const QString& filePath)
{
    QFile cacheFile(filePath);
    if (!cacheFile.open(QIODevice::ReadOnly))
    {
        return false;
    }
    return cacheFile.read(256).contains(WORKPIECE_MESH_CACHE_TAG);
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

    const QVector<QPair<int, int>> frames = SplitScanFrames(cloudPoints);
    if (frames.size() < 3)
    {
        error = QString("扫描线分帧失败（仅 %1 帧），点云可能无序。").arg(frames.size());
        return false;
    }

    // 列数取各帧点数中位（保细节），按目标三角规模行列联合抽稀。
    std::vector<double> frameSizes;
    frameSizes.reserve(frames.size());
    for (const auto& f : frames)
    {
        frameSizes.push_back(double(f.second - f.first));
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

    // 帧内典型点距，用于缺口判定。
    std::vector<double> gapSamples;
    gapSamples.reserve(4096);
    {
        const auto& f0 = frames[frames.size() / 2];
        for (int i = f0.first + 1; i < f0.second; ++i)
        {
            gapSamples.push_back((cloudPoints[i].point - cloudPoints[i - 1].point).norm());
        }
    }
    const double gapMedian = std::max(MedianOf(gapSamples), 1e-3);

    // 列对齐必须按"空间位置"而不是线内序号/弧长百分比——每帧激光线的起点和长度随
    // 波纹/视场边缘漂移，按百分比对齐会让相邻行同列错位（表面呈斜向拉丝）。
    // 取所有帧弦向（帧首→帧尾）的统一平均作为横向轴，全帧点在该轴上的投影做全局栅格：
    // 相邻行同列 = 同一物理横向位置；蛇形扫描方向差异也被弦向符号统一吸收。
    Eigen::Vector3d chordDir = Eigen::Vector3d::Zero();
    for (const auto& frame : frames)
    {
        Eigen::Vector3d chord = cloudPoints[frame.second - 1].point - cloudPoints[frame.first].point;
        const double len = chord.norm();
        if (len < 1e-6)
        {
            continue;
        }
        chord /= len;
        if (chord.dot(chordDir) < 0.0)
        {
            chord = -chord;
        }
        chordDir += chord;
    }
    if (chordDir.norm() < 1e-9)
    {
        error = "扫描线弦方向退化，无法确定横向轴。";
        return false;
    }
    chordDir.normalize();

    double tMin = std::numeric_limits<double>::max();
    double tMax = std::numeric_limits<double>::lowest();
    for (const auto& frame : frames)
    {
        for (int i = frame.first; i < frame.second; ++i)
        {
            const double t = cloudPoints[i].point.dot(chordDir);
            tMin = std::min(tMin, t);
            tMax = std::max(tMax, t);
        }
    }
    if (tMax - tMin < 1e-6)
    {
        error = "横向跨度为零，无法网格化。";
        return false;
    }
    const double columnWidth = (tMax - tMin) / std::max(1, columns - 1);
    const double bridgeLimit = std::max(gapMedian, columnWidth) * 4.0;  // 缺口超 4 倍点距不桥接

    QVector<Eigen::Vector3f> grid(static_cast<qsizetype>(rows) * columns,
        Eigen::Vector3f(kNaN, kNaN, kNaN));
    std::vector<std::pair<double, Eigen::Vector3d>> line;
    for (int r = 0; r < rows; ++r)
    {
        const auto& frame = frames[static_cast<int>(
            double(r) * (frames.size() - 1) / std::max(1, rows - 1))];
        line.clear();
        line.reserve(frame.second - frame.first);
        for (int i = frame.first; i < frame.second; ++i)
        {
            line.push_back({ cloudPoints[i].point.dot(chordDir), cloudPoints[i].point });
        }
        // 弦向投影对波纹截面近似单调，排序兜底局部噪声折返。
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
            // 列落在帧覆盖范围外或落在帧内 3D 缺口里 → 保持 NaN（不桥接遮挡/缺失）。
            if (t < a.first - bridgeLimit || t > b.first + bridgeLimit
                || (b.second - a.second).norm() > bridgeLimit)
            {
                continue;
            }
            const double segment = b.first - a.first;
            const double w = segment > 1e-12
                ? std::clamp((t - a.first) / segment, 0.0, 1.0)
                : 0.0;
            const Eigen::Vector3d p = a.second * (1.0 - w) + b.second * w;
            grid[static_cast<qsizetype>(r) * columns + c] = p.cast<float>();
        }
    }

    // 2×2 单元三角化：NaN 跳过 + 最大边长阈值（防止把遮挡缺口/帧间断裂糊死）。
    // 阈值兼顾帧间行距（行抽稀后变大）与列宽。
    double frameAdvance = gapMedian;
    if (frames.size() >= 2)
    {
        const auto& fa = frames[frames.size() / 2 - 1];
        const auto& fb = frames[frames.size() / 2];
        frameAdvance = (cloudPoints[fb.first].point - cloudPoints[fa.first].point).norm()
            * std::max(1.0, double(frames.size()) / rows);
    }
    const float maxEdge = static_cast<float>(
        std::max(columnWidth, frameAdvance) * 8.0 + 8.0);
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

    // 顶点法线 = 邻接面法线累加归一。绕序由网格参数化天然一致；朝向不强行统一
    //（工件摆放姿态任意），显示端用双面光照，CloudCompare 默认也双面渲染。
    mesh.normals = QVector<Eigen::Vector3f>(mesh.vertices.size(), Eigen::Vector3f::Zero());
    for (qsizetype i = 0; i + 2 < mesh.indices.size(); i += 3)
    {
        const Eigen::Vector3f& a = mesh.vertices[mesh.indices[i]];
        const Eigen::Vector3f& b = mesh.vertices[mesh.indices[i + 1]];
        const Eigen::Vector3f& c = mesh.vertices[mesh.indices[i + 2]];
        const Eigen::Vector3f n = (b - a).cross(c - a);
        mesh.normals[mesh.indices[i]] += n;
        mesh.normals[mesh.indices[i + 1]] += n;
        mesh.normals[mesh.indices[i + 2]] += n;
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
        "comment %3\n"
        "element vertex %1\n"
        "property float x\nproperty float y\nproperty float z\n"
        "property float nx\nproperty float ny\nproperty float nz\n"
        "element face %2\n"
        "property list uchar uint vertex_indices\n"
        "end_header\n")
        .arg(mesh.vertices.size())
        .arg(faceCount)
        .arg(QString::fromLatin1(WORKPIECE_MESH_CACHE_TAG))
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

bool WorkpieceMeshBuilder::SaveMeshStl(const QString& filePath, const Mesh& mesh, QString& error)
{
    if (!mesh.IsValid())
    {
        error = "网格为空，未导出 STL。";
        return false;
    }
    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate))
    {
        error = QString("写入 STL 文件失败：%1").arg(filePath);
        return false;
    }

    // 二进制 STL：80 字节头 + uint32 三角数 + 每三角形 [面法线3f + 顶点9f + 2字节属性]。
    char header[80] = { 0 };
    std::snprintf(header, sizeof(header), "NoTeaching-Robot workpiece mesh");
    file.write(header, sizeof(header));
    const quint32 triangleCount = static_cast<quint32>(mesh.indices.size() / 3);
    file.write(reinterpret_cast<const char*>(&triangleCount), sizeof(triangleCount));

    QByteArray block;
    block.resize(qsizetype(triangleCount) * 50);
    char* out = block.data();
    for (qsizetype i = 0; i + 2 < mesh.indices.size(); i += 3)
    {
        const Eigen::Vector3f& a = mesh.vertices[mesh.indices[i]];
        const Eigen::Vector3f& b = mesh.vertices[mesh.indices[i + 1]];
        const Eigen::Vector3f& c = mesh.vertices[mesh.indices[i + 2]];
        Eigen::Vector3f n = (b - a).cross(c - a);
        const float len = n.norm();
        n = len > 1e-12f ? Eigen::Vector3f(n / len) : Eigen::Vector3f(0.f, 0.f, 1.f);
        const float values[12] = {
            n.x(), n.y(), n.z(),
            a.x(), a.y(), a.z(),
            b.x(), b.y(), b.z(),
            c.x(), c.y(), c.z() };
        std::memcpy(out, values, sizeof(values));
        out += sizeof(values);
        out[0] = 0;
        out[1] = 0;
        out += 2;
    }
    file.write(block);
    file.close();
    return true;
}

bool WorkpieceMeshBuilder::LoadCloudPointsWithProgress(
    const QString& cloudFilePath,
    QVector<RobotCalculation::IndexedPoint3D>& points,
    QString& error,
    const ProgressCallback& progress)
{
    points.clear();
    QFile file(cloudFilePath);
    if (!file.open(QIODevice::ReadOnly))
    {
        error = QString("打开点云文件失败：%1").arg(cloudFilePath);
        return false;
    }
    const qint64 totalBytes = std::max<qint64>(1, file.size());
    points.reserve(static_cast<qsizetype>(std::min<qint64>(totalBytes / 40, 8000000)));

    qint64 lineCounter = 0;
    while (!file.atEnd())
    {
        const QByteArray line = file.readLine();
        // 每 ~1.6 万行报一次进度并检查取消（约每 0.7MB 一次，开销可忽略）。
        if ((++lineCounter & 0x3FFF) == 0 && progress)
        {
            const int percent = static_cast<int>(file.pos() * 100 / totalBytes);
            if (!progress(percent, QStringLiteral("读取完整点云")))
            {
                error = QStringLiteral("已取消");
                return false;
            }
        }
        // 手写解析 "index x y z"（空格分隔）：表头/非数字行 strtoll 解析失败自动跳过。
        const char* cursor = line.constData();
        char* next = nullptr;
        const long long index = std::strtoll(cursor, &next, 10);
        if (next == cursor)
        {
            continue;
        }
        cursor = next;
        const double x = std::strtod(cursor, &next);
        if (next == cursor)
        {
            continue;
        }
        cursor = next;
        const double y = std::strtod(cursor, &next);
        if (next == cursor)
        {
            continue;
        }
        cursor = next;
        const double z = std::strtod(cursor, &next);
        if (next == cursor)
        {
            continue;
        }
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
        {
            continue;
        }
        RobotCalculation::IndexedPoint3D point;
        point.index = static_cast<int>(index);
        point.point = Eigen::Vector3d(x, y, z);
        points.push_back(point);
    }
    if (points.size() < 16)
    {
        error = QString("点云文件有效点过少（%1）：%2").arg(points.size()).arg(cloudFilePath);
        return false;
    }
    return true;
}

bool WorkpieceMeshBuilder::EnsureMeshCache(
    const QString& laserDir,
    const QString& cloudFilePath,
    QString& error,
    const ProgressCallback& progress)
{
    const QString cachePath = MeshCachePath(laserDir);
    if (IsMeshCacheValid(cachePath))
    {
        return true;
    }
    if (!QFileInfo::exists(cloudFilePath))
    {
        error = QString("缺少完整点云文件：%1").arg(cloudFilePath);
        return false;
    }

    // 0~80%：读取点云（最耗时的阶段，按字节进度换算）。
    const ProgressCallback readProgress = progress
        ? [&progress](int percent, const QString& stage)
            {
                return progress(percent * 80 / 100, stage);
            }
        : ProgressCallback();
    QVector<RobotCalculation::IndexedPoint3D> cloudPoints;
    if (!LoadCloudPointsWithProgress(cloudFilePath, cloudPoints, error, readProgress))
    {
        return false;
    }

    if (progress && !progress(82, QStringLiteral("扫描线网格化")))
    {
        error = QStringLiteral("已取消");
        return false;
    }
    Mesh mesh;
    if (!BuildFromScanlineCloud(cloudPoints, mesh, error))
    {
        return false;
    }

    if (progress && !progress(95, QStringLiteral("写出模型缓存")))
    {
        error = QStringLiteral("已取消");
        return false;
    }
    if (!SaveMeshPly(cachePath, mesh, error))
    {
        return false;
    }
    if (progress)
    {
        progress(100, QStringLiteral("完成"));
    }
    return true;
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
    // 工件摆放姿态任意：高度轴 = 包围盒跨度最小的轴（表面偏移方向），
    // 展开面 = 另两轴（竖立波纹板即 板面两轴 展开、水平轴当"高度"）。
    const Eigen::Vector3f span3 = maxBound - minBound;
    int heightAxis = 0;
    if (span3.y() <= span3.x() && span3.y() <= span3.z())
    {
        heightAxis = 1;
    }
    else if (span3.z() <= span3.x() && span3.z() <= span3.y())
    {
        heightAxis = 2;
    }
    const int axisU = heightAxis == 0 ? 1 : 0;
    const int axisV = heightAxis == 2 ? 1 : 2;
    const float spanU = std::max(1e-3f, span3[axisU]);
    const float spanV = std::max(1e-3f, span3[axisV]);

    int width = maxImageWidth;
    int height = static_cast<int>(width * (spanV / spanU));
    if (height > maxImageWidth * 4)
    {
        height = maxImageWidth * 4;
        width = std::max(64, static_cast<int>(height * (spanU / spanV)));
    }
    height = std::clamp(height, 64, 8192);

    cv::Mat heightField(height, width, CV_32FC1, cv::Scalar(std::numeric_limits<float>::quiet_NaN()));
    for (const auto& v : mesh.vertices)
    {
        const int px = std::clamp(static_cast<int>((v[axisU] - minBound[axisU]) / spanU * (width - 1)), 0, width - 1);
        const int py = std::clamp(static_cast<int>((v[axisV] - minBound[axisV]) / spanV * (height - 1)), 0, height - 1);
        float& cell = heightField.at<float>(height - 1 - py, px);
        if (std::isnan(cell) || v[heightAxis] > cell)
        {
            cell = v[heightAxis];  // 同格取偏移最大（观察方向可见面）
        }
    }

    // 小孔填补（近邻扩散一次）后做梯度光照。
    cv::Mat mask = heightField != heightField;  // NaN mask
    cv::Mat filled = heightField.clone();
    cv::patchNaNs(filled, minBound[heightAxis]);
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
