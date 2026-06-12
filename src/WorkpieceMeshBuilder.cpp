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
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <vector>

namespace
{
constexpr auto WORKPIECE_MESH_FILE_NAME = "PreciseLaserPoint_WorkpieceMesh.ply";
// 网格化算法版本：写入 PLY 注释头；EnsureMeshCache 据此让旧算法生成的缓存自动失效重建。
constexpr auto WORKPIECE_MESH_CACHE_TAG = "NoTeaching-Robot workpiece mesh cache v7";
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

bool IsFiniteVec(const Eigen::Vector3f& v)
{
    return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
}

bool IsFiniteVec(const Eigen::Vector3d& v)
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

}

QString WorkpieceMeshBuilder::MeshCacheFileName()
{
    return QString::fromLatin1(WORKPIECE_MESH_FILE_NAME);
}

QString WorkpieceMeshBuilder::MeshCachePath(const QString& laserDir)
{
    return QDir(laserDir).filePath(MeshCacheFileName());
}

QString WorkpieceMeshBuilder::EditedCloudFileName()
{
    return QStringLiteral("PreciseLaserPoint_WorkpieceCloud_Edited.txt");
}

QString WorkpieceMeshBuilder::EditedCloudPath(const QString& laserDir)
{
    return QDir(laserDir).filePath(EditedCloudFileName());
}

QString WorkpieceMeshBuilder::ResolveCloudSourcePath(const QString& laserDir)
{
    const QString edited = EditedCloudPath(laserDir);
    if (QFileInfo::exists(edited))
    {
        return edited;
    }
    return QDir(laserDir).filePath(QStringLiteral("PreciseLaserPoint_WorkpieceCloud.txt"));
}

bool WorkpieceMeshBuilder::SaveCloudPointsTxt(
    const QString& filePath,
    const QVector<RobotCalculation::IndexedPoint3D>& points,
    QString& error)
{
    const QString tempPath = filePath + QStringLiteral(".tmp");
    QFile file(tempPath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate))
    {
        error = QString("写入点云文件失败：%1").arg(tempPath);
        return false;
    }
    // snprintf 批量拼缓冲直写（340 万行级，QTextStream 逐行格式化慢一个量级）。
    QByteArray buffer;
    constexpr int kFlushSize = 1 << 22;
    buffer.reserve(kFlushSize + 128);
    char line[128];
    for (const auto& p : points)
    {
        const int len = std::snprintf(line, sizeof(line), "%d %.6f %.6f %.6f\n",
            p.index, p.point.x(), p.point.y(), p.point.z());
        buffer.append(line, len);
        if (buffer.size() >= kFlushSize)
        {
            file.write(buffer);
            buffer.clear();
        }
    }
    file.write(buffer);
    if (file.error() != QFileDevice::NoError)
    {
        file.close();
        QFile::remove(tempPath);
        error = QString("写入点云文件失败：%1").arg(tempPath);
        return false;
    }
    file.close();
    QFile::remove(filePath);
    if (!QFile::rename(tempPath, filePath))
    {
        QFile::remove(tempPath);
        error = QString("替换点云文件失败：%1").arg(filePath);
        return false;
    }
    return true;
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

    // 单帧滤波会把每帧激光线保留成 2~3 条主直线段（多段折线），任何"每帧一条线"的
    // 条带化假设都不成立。改用与扫描结构完全解耦的方案：PCA 求板面主平面，全部点
    // 投影到主平面规则栅格（格内中位数聚合，抗离群噪声簇），再做规则网格三角化——
    // 与帧结构、段数、点序、摆放姿态全部无关，连续性由点云密度天然保证。

    // 1) 抽样 PCA：质心 + 主平面两轴（u,v）+ 法向（n，最小特征值方向）。
    const int pcaStride = std::max(1, int(cloudPoints.size() / 200000));
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    qint64 pcaCount = 0;
    // 离线重建路径读回的 _WorkpieceCloud.txt 可能含 "nan"/"inf" 行（上游变换异常时
    // 程序原样落盘，QString::toDouble 解析成功），非有限点进投格 int 转换与 std::sort
    // 是未定义行为——全部坐标循环一律跳过非有限点。
    for (int i = 0; i < cloudPoints.size(); i += pcaStride)
    {
        if (!IsFiniteVec(cloudPoints[i].point))
        {
            continue;
        }
        centroid += cloudPoints[i].point;
        ++pcaCount;
    }
    if (pcaCount == 0)
    {
        error = "点云不含有效（有限值）坐标，无法网格化。";
        return false;
    }
    centroid /= double(pcaCount);
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (int i = 0; i < cloudPoints.size(); i += pcaStride)
    {
        if (!IsFiniteVec(cloudPoints[i].point))
        {
            continue;
        }
        const Eigen::Vector3d d = cloudPoints[i].point - centroid;
        covariance += d * d.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigenSolver(covariance);
    if (eigenSolver.info() != Eigen::Success)
    {
        error = "点云主平面求解失败。";
        return false;
    }
    // 特征值升序：列0=法向（最小展布），列1/列2=主平面两轴。
    const Eigen::Vector3d normalAxis = eigenSolver.eigenvectors().col(0);
    const Eigen::Vector3d axisU = eigenSolver.eigenvectors().col(2);
    const Eigen::Vector3d axisV = eigenSolver.eigenvectors().col(1);

    // 2) 全点投影到 (u,v,h)，求范围与典型点距。
    double uMin = std::numeric_limits<double>::max();
    double uMax = std::numeric_limits<double>::lowest();
    double vMin = uMin;
    double vMax = uMax;
    for (const auto& p : cloudPoints)
    {
        if (!IsFiniteVec(p.point))
        {
            continue;
        }
        const Eigen::Vector3d d = p.point - centroid;
        const double u = d.dot(axisU);
        const double v = d.dot(axisV);
        uMin = std::min(uMin, u);
        uMax = std::max(uMax, u);
        vMin = std::min(vMin, v);
        vMax = std::max(vMax, v);
    }
    const double spanU = uMax - uMin;
    const double spanV = vMax - vMin;
    if (spanU < 1e-3 || spanV < 1e-3)
    {
        error = "点云主平面跨度退化，无法网格化。";
        return false;
    }
    std::vector<double> gapSamples;
    gapSamples.reserve(8192);
    for (int i = 1; i < std::min<int>(cloudPoints.size(), 20000); ++i)
    {
        const double d = (cloudPoints[i].point - cloudPoints[i - 1].point).norm();
        if (d > 1e-9)
        {
            gapSamples.push_back(d);
        }
    }
    const double gapMedian = std::max(MedianOf(gapSamples), 1e-3);

    // 3) 栅格尺寸：按目标三角规模定格宽，且不小于典型点距（避免大量空格穿孔）。
    const double cellTarget = std::max(1.0, double(std::max(targetMaxTriangles, 20000)) / 2.0);
    double cellSize = std::sqrt(spanU * spanV / cellTarget);
    cellSize = std::max(cellSize, gapMedian * 1.2);
    const int columns = std::clamp(int(spanU / cellSize) + 1, 16, 8192);
    const int rows = std::clamp(int(spanV / cellSize) + 1, 16, 8192);
    const double stepU = spanU / std::max(1, columns - 1);
    const double stepV = spanV / std::max(1, rows - 1);

    // 4) 格内高度聚合：最大簇中位数（抗离群噪声簇与跨面混样）。先收集每格样本（短数组）。
    QVector<float> heightField(static_cast<qsizetype>(rows) * columns, kNaN);
    {
        std::vector<std::vector<float>> cellSamples(static_cast<size_t>(rows) * columns);
        for (const auto& p : cloudPoints)
        {
            if (!IsFiniteVec(p.point))
            {
                continue;
            }
            const Eigen::Vector3d d = p.point - centroid;
            const int c = std::clamp(int((d.dot(axisU) - uMin) / stepU + 0.5), 0, columns - 1);
            const int r = std::clamp(int((d.dot(axisV) - vMin) / stepV + 0.5), 0, rows - 1);
            cellSamples[static_cast<size_t>(r) * columns + c].push_back(float(d.dot(normalAxis)));
        }
        for (size_t i = 0; i < cellSamples.size(); ++i)
        {
            auto& samples = cellSamples[i];
            if (samples.empty())
            {
                continue;
            }
            // 按高度排序后以间隙切簇，只取点数最多的簇的中位数：跨立面过渡格
            // （平台点+立面点混在一格）与悬空噪声簇被分到小簇排除，避免聚合值
            // 在两个面之间摆动拉出竖直细丝；纯立面格内高度连续无大间隙，整簇
            // 保留为墙面。
            std::sort(samples.begin(), samples.end());
            constexpr float kClusterGap = 3.0f;
            size_t bestBegin = 0;
            size_t bestCount = 0;
            size_t runBegin = 0;
            for (size_t j = 1; j <= samples.size(); ++j)
            {
                if (j == samples.size() || samples[j] - samples[j - 1] > kClusterGap)
                {
                    const size_t runCount = j - runBegin;
                    if (runCount > bestCount)
                    {
                        bestBegin = runBegin;
                        bestCount = runCount;
                    }
                    runBegin = j;
                }
            }
            heightField[static_cast<qsizetype>(i)] = samples[bestBegin + bestCount / 2];
        }
    }

    // 4.5) 后处理：a) 小孔填补——扫描线间隔大于格宽时部分格子采不到点形成虚线状孔洞，
    // 对"有 ≥3 个有效邻居"的空格取邻居中值填补（最多两轮，大面积无数据区不会被糊死）；
    // b) 3×3 中值滤波——去掉立面窄条等处格间跳动形成的毛刺。
    auto cellAt = [&heightField, columns, rows](int r, int c) -> float
        {
            if (r < 0 || r >= rows || c < 0 || c >= columns)
            {
                return kNaN;
            }
            return heightField[static_cast<qsizetype>(r) * columns + c];
        };
    for (int pass = 0; pass < 2; ++pass)
    {
        QVector<float> filled = heightField;
        int filledCount = 0;
        for (int r = 0; r < rows; ++r)
        {
            for (int c = 0; c < columns; ++c)
            {
                if (!std::isnan(cellAt(r, c)))
                {
                    continue;
                }
                // 仅填"被数据包夹"的内部孔格：横向或纵向两侧 2 格内均有有效数据。
                // 扫描间隔形成的细孔两侧必有数据；自由立边外侧的空格只有单侧有
                // 数据，不满足包夹——避免填补向边界外扩、在立边顶长出悬空裙边。
                auto hasValidToward = [&cellAt, r, c](int dr, int dc)
                    {
                        return !std::isnan(cellAt(r + dr, c + dc))
                            || !std::isnan(cellAt(r + 2 * dr, c + 2 * dc));
                    };
                const bool spanned = (hasValidToward(0, -1) && hasValidToward(0, 1))
                    || (hasValidToward(-1, 0) && hasValidToward(1, 0));
                if (!spanned)
                {
                    continue;
                }
                float neighbors[8];
                int neighborCount = 0;
                for (int dr = -1; dr <= 1; ++dr)
                {
                    for (int dc = -1; dc <= 1; ++dc)
                    {
                        if (dr == 0 && dc == 0)
                        {
                            continue;
                        }
                        const float h = cellAt(r + dr, c + dc);
                        if (!std::isnan(h))
                        {
                            neighbors[neighborCount++] = h;
                        }
                    }
                }
                if (neighborCount >= 3)
                {
                    std::nth_element(neighbors, neighbors + neighborCount / 2, neighbors + neighborCount);
                    filled[static_cast<qsizetype>(r) * columns + c] = neighbors[neighborCount / 2];
                    ++filledCount;
                }
            }
        }
        heightField = filled;
        if (filledCount == 0)
        {
            break;
        }
    }
    for (int smoothPass = 0; smoothPass < 2; ++smoothPass)
    {
        QVector<float> smoothed = heightField;
        for (int r = 0; r < rows; ++r)
        {
            for (int c = 0; c < columns; ++c)
            {
                if (std::isnan(cellAt(r, c)))
                {
                    continue;
                }
                float window[9];
                int windowCount = 0;
                for (int dr = -1; dr <= 1; ++dr)
                {
                    for (int dc = -1; dc <= 1; ++dc)
                    {
                        const float h = cellAt(r + dr, c + dc);
                        if (!std::isnan(h))
                        {
                            window[windowCount++] = h;
                        }
                    }
                }
                std::nth_element(window, window + windowCount / 2, window + windowCount);
                smoothed[static_cast<qsizetype>(r) * columns + c] = window[windowCount / 2];
            }
        }
        heightField = smoothed;
    }

    // 5) 网格顶点 = 质心 + u·axisU + v·axisV + h·normalAxis。
    QVector<Eigen::Vector3f> grid(static_cast<qsizetype>(rows) * columns,
        Eigen::Vector3f(kNaN, kNaN, kNaN));
    for (int r = 0; r < rows; ++r)
    {
        for (int c = 0; c < columns; ++c)
        {
            const float h = heightField[static_cast<qsizetype>(r) * columns + c];
            if (std::isnan(h))
            {
                continue;
            }
            const Eigen::Vector3d p = centroid
                + axisU * (uMin + stepU * c)
                + axisV * (vMin + stepV * r)
                + normalAxis * double(h);
            grid[static_cast<qsizetype>(r) * columns + c] = p.cast<float>();
        }
    }

    // 2×2 单元三角化：NaN 跳过 + 最大边长阈值（防止把遮挡缺口糊死；
    // 立面/折弯处相邻格高度差大但属真实表面，阈值给足跨立面的余量）。
    const float maxEdge = static_cast<float>(std::max(stepU, stepV) * 6.0 + 25.0);
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
    // 先写临时文件再原子替换：直接 Truncate 覆写时若进程中途终止，截断文件的
    // header（含缓存标记）已落盘会被 IsMeshCacheValid 误判有效，加载报数据不完整。
    const QString tempPath = filePath + QStringLiteral(".tmp");
    QFile file(tempPath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate))
    {
        error = QString("写入模型文件失败：%1").arg(tempPath);
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
    if (file.error() != QFileDevice::NoError)
    {
        file.close();
        QFile::remove(tempPath);
        error = QString("写入模型文件失败：%1").arg(tempPath);
        return false;
    }
    file.close();
    QFile::remove(filePath);
    if (!QFile::rename(tempPath, filePath))
    {
        QFile::remove(tempPath);
        error = QString("替换模型文件失败：%1").arg(filePath);
        return false;
    }
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
