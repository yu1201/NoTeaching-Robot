#include "WorkpieceMeshBuilder.h"

#include "RobotDataHelper.h"

#include <opencv2/imgproc.hpp>

#include <QDataStream>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QSaveFile>
#include <QTextStream>

#include <algorithm>
#include <cerrno>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <new>
#include <vector>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#endif

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
    // SaveMeshPly 是模型库的持久化信任边界。Mesh::IsValid() 只用于轻量判断，
    // 这里必须完整验证，否则法线缺失会在下面按顶点写出时越界，坏索引也会被永久落盘。
    if (mesh.normals.size() != mesh.vertices.size())
    {
        error = QStringLiteral("网格法线数量与顶点数量不一致，拒绝写出模型文件。");
        return false;
    }
    if (mesh.indices.size() % 3 != 0)
    {
        error = QStringLiteral("网格三角索引数量不是3的整数倍，拒绝写出模型文件。");
        return false;
    }
    constexpr qsizetype kMaximumPersistedVertices = 2'000'000;
    constexpr qsizetype kMaximumPersistedTriangles = 4'000'000;
    if (mesh.vertices.size() > kMaximumPersistedVertices
        || mesh.indices.size() / 3 > kMaximumPersistedTriangles)
    {
        error = QStringLiteral("网格超过PLY持久化上限（200万顶点/400万三角形）。");
        return false;
    }
    for (qsizetype i = 0; i < mesh.vertices.size(); ++i)
    {
        if (!mesh.vertices.at(i).allFinite() || !mesh.normals.at(i).allFinite())
        {
            error = QStringLiteral("网格第%1个顶点或法线包含非有限值，拒绝写出模型文件。")
                .arg(i);
            return false;
        }
    }
    for (qsizetype i = 0; i < mesh.indices.size(); ++i)
    {
        if (mesh.indices.at(i) >= static_cast<quint32>(mesh.vertices.size()))
        {
            error = QStringLiteral("网格第%1个索引越界，拒绝写出模型文件。").arg(i);
            return false;
        }
    }
    // QSaveFile 在同目录写临时文件并原子提交；禁用 direct-write fallback，确保覆盖失败或
    // 进程中断时旧模型保持不变，不会先删除旧 PLY 再留下空缺。
    QSaveFile file(filePath);
    file.setDirectWriteFallback(false);
    if (!file.open(QIODevice::WriteOnly))
    {
        error = QStringLiteral("写入模型文件失败：%1").arg(file.errorString());
        return false;
    }
    const auto writeBlock = [&file](const QByteArray& block)
    {
        return file.write(block) == block.size();
    };
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
    if (!writeBlock(header))
    {
        file.cancelWriting();
        error = QStringLiteral("写入模型文件头失败：%1").arg(file.errorString());
        return false;
    }

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
    if (!writeBlock(vertexBlock))
    {
        file.cancelWriting();
        error = QStringLiteral("写入模型顶点失败：%1").arg(file.errorString());
        return false;
    }

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
    if (!writeBlock(faceBlock))
    {
        file.cancelWriting();
        error = QStringLiteral("写入模型三角形失败：%1").arg(file.errorString());
        return false;
    }
    if (!file.commit())
    {
        error = QStringLiteral("原子提交模型文件失败，旧模型保持不变：%1")
            .arg(file.errorString());
        return false;
    }
    return true;
}

bool WorkpieceMeshBuilder::LoadMeshPly(
    const QString& filePath,
    Mesh& mesh,
    QString& error,
    const CancelCallback& cancelRequested)
{
    // 这里是一个信任边界：基准模型可以由用户导入，不能让 PLY 头驱动
    // 无界分配。上限覆盖本工程 150 万三角形的正常缓存，同时将单模型
    // 常驻数据压在可控范围内。
    constexpr qint64 kMaximumFileBytes = 256LL * 1024LL * 1024LL;
    constexpr qint64 kMaximumHeaderBytes = 64LL * 1024LL;
    constexpr qsizetype kMaximumHeaderLineBytes = 1024;
    constexpr int kMaximumHeaderLines = 128;
    constexpr qint64 kMaximumVertices = 2'000'000;
    constexpr qint64 kMaximumFaces = 4'000'000;
    constexpr qint64 kVertexRecordBytes = 6LL * static_cast<qint64>(sizeof(float));
    constexpr qint64 kFaceRecordBytes = 1LL + 3LL * static_cast<qint64>(sizeof(quint32));
    constexpr qint64 kRecordsPerChunk = 4096;

    mesh = Mesh();
    error.clear();
    const auto cancelled = [&cancelRequested]()
    {
        return cancelRequested && cancelRequested();
    };
    const auto fail = [&error](const QString& message)
    {
        error = message;
        return false;
    };

    try
    {
        if (cancelled()) return fail(QStringLiteral("加载模型已取消。"));

        const QFileInfo info(filePath);
        if (!info.exists() || !info.isFile() || info.isSymLink())
            return fail(QStringLiteral("模型路径不是可读的普通文件：%1").arg(filePath));
        if (info.size() <= 0 || info.size() > kMaximumFileBytes)
            return fail(QStringLiteral("模型文件大小超出限制（最大 256MiB）。"));

        QFile file(filePath);
        if (!file.open(QIODevice::ReadOnly))
            return fail(QStringLiteral("打开模型文件失败：%1").arg(filePath));
        const qint64 openedFileSize = file.size();
        if (openedFileSize != info.size() || openedFileSize <= 0 || openedFileSize > kMaximumFileBytes)
            return fail(QStringLiteral("模型文件在加载前已变化或超出大小限制。"));

        qint64 vertexCount = -1;
        qint64 faceCount = -1;
        bool sawPly = false;
        bool sawFormat = false;
        bool sawVertexElement = false;
        bool sawFaceElement = false;
        bool headerEnd = false;
        int vertexPropertyIndex = 0;
        int facePropertyIndex = 0;
        enum class Element { None, Vertex, Face } currentElement = Element::None;
        static const QByteArray kVertexProperties[] = {
            "property float x", "property float y", "property float z",
            "property float nx", "property float ny", "property float nz"
        };

        for (int lineNumber = 0; lineNumber < kMaximumHeaderLines && !file.atEnd(); ++lineNumber)
        {
            if (cancelled()) return fail(QStringLiteral("加载模型已取消。"));
            const QByteArray rawLine = file.readLine(kMaximumHeaderLineBytes + 1);
            if (rawLine.isEmpty() && file.error() != QFileDevice::NoError)
                return fail(QStringLiteral("读取模型文件头失败。"));
            if (rawLine.size() > kMaximumHeaderLineBytes
                || (!rawLine.endsWith('\n') && !file.atEnd()))
                return fail(QStringLiteral("模型文件头存在超长行。"));
            if (file.pos() > kMaximumHeaderBytes)
                return fail(QStringLiteral("模型文件头超过 64KiB 限制。"));

            const QByteArray line = rawLine.trimmed();
            if (lineNumber == 0)
            {
                if (line != "ply") return fail(QStringLiteral("模型文件缺少 PLY 签名。"));
                sawPly = true;
                continue;
            }
            if (line.startsWith("comment ") || line.startsWith("obj_info ")) continue;
            if (line == "format binary_little_endian 1.0")
            {
                if (sawFormat || sawVertexElement)
                    return fail(QStringLiteral("模型文件 format 声明位置或数量非法。"));
                sawFormat = true;
                continue;
            }
            if (line.startsWith("element "))
            {
                const QList<QByteArray> fields = line.simplified().split(' ');
                if (fields.size() != 3 || !sawFormat)
                    return fail(QStringLiteral("模型文件 element 声明非法。"));
                bool countOk = false;
                const qint64 count = fields.at(2).toLongLong(&countOk, 10);
                if (!countOk)
                    return fail(QStringLiteral("模型文件 element 数量非法。"));
                if (fields.at(1) == "vertex" && !sawVertexElement && !sawFaceElement)
                {
                    if (count < 3 || count > kMaximumVertices)
                        return fail(QStringLiteral("模型顶点数超出限制。"));
                    vertexCount = count;
                    sawVertexElement = true;
                    currentElement = Element::Vertex;
                }
                else if (fields.at(1) == "face" && sawVertexElement && !sawFaceElement
                         && vertexPropertyIndex == 6)
                {
                    if (count < 1 || count > kMaximumFaces)
                        return fail(QStringLiteral("模型面数超出限制。"));
                    faceCount = count;
                    sawFaceElement = true;
                    currentElement = Element::Face;
                }
                else
                {
                    return fail(QStringLiteral("模型文件含未支持或重复的 element。"));
                }
                continue;
            }
            if (line.startsWith("property "))
            {
                if (currentElement == Element::Vertex)
                {
                    if (vertexPropertyIndex >= 6 || line != kVertexProperties[vertexPropertyIndex++])
                        return fail(QStringLiteral("模型顶点属性 schema 不匹配。"));
                }
                else if (currentElement == Element::Face)
                {
                    if (facePropertyIndex != 0
                        || line != "property list uchar uint vertex_indices")
                        return fail(QStringLiteral("模型面属性 schema 不匹配。"));
                    ++facePropertyIndex;
                }
                else
                {
                    return fail(QStringLiteral("模型属性缺少所属 element。"));
                }
                continue;
            }
            if (line == "end_header")
            {
                headerEnd = true;
                break;
            }
            return fail(QStringLiteral("模型文件头含未支持的声明。"));
        }

        if (!sawPly || !sawFormat || !headerEnd || !sawVertexElement || !sawFaceElement
            || vertexPropertyIndex != 6 || facePropertyIndex != 1)
            return fail(QStringLiteral("模型文件头不完整或 schema 不匹配。"));

        const qint64 headerBytes = file.pos();
        const qint64 vertexBytes = vertexCount * kVertexRecordBytes;
        const qint64 faceBytes = faceCount * kFaceRecordBytes;
        if (headerBytes <= 0 || headerBytes > kMaximumHeaderBytes
            || headerBytes + vertexBytes + faceBytes != openedFileSize)
            return fail(QStringLiteral("模型载荷长度与文件头不匹配。"));

        if (cancelled()) return fail(QStringLiteral("加载模型已取消。"));
        Mesh loaded;
        loaded.vertices.resize(static_cast<qsizetype>(vertexCount));
        loaded.normals.resize(static_cast<qsizetype>(vertexCount));
        loaded.indices.resize(static_cast<qsizetype>(faceCount * 3));

        for (qint64 base = 0; base < vertexCount; base += kRecordsPerChunk)
        {
            if (cancelled()) return fail(QStringLiteral("加载模型已取消。"));
            const qint64 recordCount = std::min(kRecordsPerChunk, vertexCount - base);
            const qint64 bytes = recordCount * kVertexRecordBytes;
            const QByteArray block = file.read(bytes);
            if (block.size() != bytes)
                return fail(QStringLiteral("模型文件顶点数据不完整。"));
            for (qint64 offset = 0; offset < recordCount; ++offset)
            {
                float values[6];
                std::memcpy(values, block.constData() + offset * kVertexRecordBytes, sizeof(values));
                const Eigen::Vector3f vertex(values[0], values[1], values[2]);
                const Eigen::Vector3f normal(values[3], values[4], values[5]);
                if (!IsFiniteVec(vertex) || !IsFiniteVec(normal))
                    return fail(QStringLiteral("模型文件含 NaN/Inf 顶点或法线。"));
                const qsizetype index = static_cast<qsizetype>(base + offset);
                loaded.vertices[index] = vertex;
                loaded.normals[index] = normal;
            }
        }

        for (qint64 base = 0; base < faceCount; base += kRecordsPerChunk)
        {
            if (cancelled()) return fail(QStringLiteral("加载模型已取消。"));
            const qint64 recordCount = std::min(kRecordsPerChunk, faceCount - base);
            const qint64 bytes = recordCount * kFaceRecordBytes;
            const QByteArray block = file.read(bytes);
            if (block.size() != bytes)
                return fail(QStringLiteral("模型文件面数据不完整。"));
            for (qint64 offset = 0; offset < recordCount; ++offset)
            {
                const char* record = block.constData() + offset * kFaceRecordBytes;
                if (static_cast<unsigned char>(record[0]) != 3U)
                    return fail(QStringLiteral("模型文件存在非三角面。"));
                quint32 indices[3];
                std::memcpy(indices, record + 1, sizeof(indices));
                if (indices[0] >= static_cast<quint32>(vertexCount)
                    || indices[1] >= static_cast<quint32>(vertexCount)
                    || indices[2] >= static_cast<quint32>(vertexCount))
                    return fail(QStringLiteral("模型面引用了越界顶点。"));
                const qsizetype index = static_cast<qsizetype>((base + offset) * 3);
                loaded.indices[index] = indices[0];
                loaded.indices[index + 1] = indices[1];
                loaded.indices[index + 2] = indices[2];
            }
        }

        if (cancelled()) return fail(QStringLiteral("加载模型已取消。"));
        if (file.error() != QFileDevice::NoError || file.pos() != openedFileSize || !file.atEnd())
            return fail(QStringLiteral("模型文件末尾或读取状态异常。"));
        if (!loaded.IsValid()) return fail(QStringLiteral("模型网格无效。"));
        mesh = std::move(loaded);
        return true;
    }
    catch (const std::bad_alloc&)
    {
        mesh = Mesh();
        return fail(QStringLiteral("加载模型所需内存超出可用范围。"));
    }
    catch (const std::exception& ex)
    {
        mesh = Mesh();
        return fail(QStringLiteral("加载模型发生异常：%1")
            .arg(QString::fromLocal8Bit(ex.what()).left(256)));
    }
    catch (...)
    {
        mesh = Mesh();
        return fail(QStringLiteral("加载模型发生未知异常。"));
    }
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
    const ProgressCallback& progress,
    const CloudLoadLimits* tighterLimits)
{
    constexpr qint64 kMaximumFileBytes = 768LL * 1024LL * 1024LL;
    constexpr qint64 kMaximumPhysicalLines = 10'000'000;
    constexpr qsizetype kMaximumLineBytes = 64 * 1024;
    constexpr qsizetype kMaximumValidPoints = 8'000'000;
    constexpr qint64 kMaximumPollIntervalLines = 64 * 1024;
    constexpr qint64 kProgressPollBytes = 1LL * 1024LL * 1024LL;

    error.clear();
    const auto fail = [&error](const QString& message)
    {
        error = message;
        return false;
    };

    try
    {
        const CloudLoadLimits defaults;
        const CloudLoadLimits& limits = tighterLimits != nullptr ? *tighterLimits : defaults;
        if (limits.maximumFileBytes <= 0 || limits.maximumFileBytes > kMaximumFileBytes
            || limits.maximumPhysicalLines <= 0
            || limits.maximumPhysicalLines > kMaximumPhysicalLines
            || limits.maximumLineBytes <= 0 || limits.maximumLineBytes > kMaximumLineBytes
            || limits.maximumValidPoints < 16 || limits.maximumValidPoints > kMaximumValidPoints
            || limits.progressPollIntervalLines <= 0
            || limits.progressPollIntervalLines > kMaximumPollIntervalLines)
            return fail(QStringLiteral("点云加载限制参数非法。"));

        if (progress && !progress(0, QStringLiteral("检查点云文件")))
            return fail(QStringLiteral("已取消"));

        const QFileInfo info(cloudFilePath);
        if (!info.exists() || !info.isFile() || info.isSymLink())
            return fail(QStringLiteral("点云路径不是可读的普通文件：%1").arg(cloudFilePath));
#ifdef _WIN32
        const QString nativePath = QDir::toNativeSeparators(info.absoluteFilePath());
        const DWORD attributes = ::GetFileAttributesW(
            reinterpret_cast<LPCWSTR>(nativePath.utf16()));
        if (attributes == INVALID_FILE_ATTRIBUTES
            || (attributes & FILE_ATTRIBUTE_DIRECTORY) != 0
            || (attributes & FILE_ATTRIBUTE_REPARSE_POINT) != 0
            || (attributes & FILE_ATTRIBUTE_DEVICE) != 0)
            return fail(QStringLiteral("点云路径不得是 Windows reparse/device 链接。"));
#endif
        if (info.canonicalFilePath().isEmpty())
            return fail(QStringLiteral("无法确认点云文件的规范路径。"));
        if (info.size() <= 0 || info.size() > limits.maximumFileBytes)
            return fail(QStringLiteral("点云文件为空或超过大小限制（最大 %1MiB）。")
                .arg(limits.maximumFileBytes / (1024 * 1024)));

        QFile file(cloudFilePath);
        if (!file.open(QIODevice::ReadOnly))
            return fail(QStringLiteral("打开点云文件失败：%1").arg(cloudFilePath));
        const qint64 openedFileSize = file.size();
        if (openedFileSize != info.size() || openedFileSize <= 0
            || openedFileSize > limits.maximumFileBytes)
            return fail(QStringLiteral("点云文件在加载前已变化或超出大小限制。"));

        QVector<RobotCalculation::IndexedPoint3D> loaded;
        loaded.reserve(static_cast<qsizetype>(std::min<qint64>(
            openedFileSize / 40, std::min<qint64>(limits.maximumValidPoints, 1'000'000))));

        qint64 lineCounter = 0;
        qint64 lastProgressBytes = 0;
        while (file.pos() < openedFileSize)
        {
            const QByteArray line = file.readLine(limits.maximumLineBytes + 1);
            if (line.isEmpty() && file.error() != QFileDevice::NoError)
                return fail(QStringLiteral("读取点云文件失败。"));
            if (line.size() > limits.maximumLineBytes
                || (!line.endsWith('\n') && file.pos() < openedFileSize))
                return fail(QStringLiteral("点云第 %1 行超过 64KiB 限制。")
                    .arg(lineCounter + 1));
            if (file.pos() > openedFileSize)
                return fail(QStringLiteral("点云文件在读取期间增长。"));
            ++lineCounter;
            if (lineCounter > limits.maximumPhysicalLines)
                return fail(QStringLiteral("点云物理行数超过限制。"));

            const qint64 currentBytes = file.pos();
            if (progress
                && (lineCounter % limits.progressPollIntervalLines == 0
                    || currentBytes - lastProgressBytes >= kProgressPollBytes))
            {
                lastProgressBytes = currentBytes;
                const int percent = static_cast<int>(currentBytes * 100 / openedFileSize);
                if (!progress(percent, QStringLiteral("读取完整点云")))
                    return fail(QStringLiteral("已取消"));
            }

            // 手写解析 "index x y z"（空格分隔）。纯文字表头允许跳过；
            // 一旦首列是数字，整行必须严格完整，禁止静默吞掉损坏数据。
            const char* cursor = line.constData();
            const char* const end = line.constData() + line.size();
            char* next = nullptr;
            errno = 0;
            const long long index = std::strtoll(cursor, &next, 10);
            if (next == cursor)
            {
                const QByteArray normalized = line.simplified();
                static const QByteArray kSimpleHeader("index x y z");
                static const QByteArray kLegacyHeader(
                    "index frame_index line_point_index x y z camera_x camera_y camera_z "
                    "robot_x robot_y robot_z robot_rx robot_ry robot_rz");
                // 只允许空行、明确 # 注释和两种已知 schema 表头。其他非数字
                // 行是损坏数据，禁止在后续已有 16 个有效点时被静默吞掉。
                if (normalized.isEmpty() || normalized.startsWith('#')
                    || normalized == kSimpleHeader || normalized == kLegacyHeader)
                    continue;
                return fail(QStringLiteral("点云第 %1 行是未知非数字内容，不是允许的表头/注释。")
                    .arg(lineCounter));
            }
            if (errno == ERANGE
                || index < static_cast<long long>(std::numeric_limits<int>::min())
                || index > static_cast<long long>(std::numeric_limits<int>::max()))
                return fail(QStringLiteral("点云第 %1 行 index 超出 int 范围。").arg(lineCounter));

            cursor = next;
            // 同时支持当前 "index x y z" 与历史完整扫描行
            // "index frame line x y z camera...robot..."。仅接受 4/15 列两种完整
            // schema，避免旧文件被错把 frame/line 当成 XYZ，也不静默接受尾字段。
            double values[14] = {};
            int valueCount = 0;
            while (true)
            {
                while (cursor < end && std::isspace(static_cast<unsigned char>(*cursor))) ++cursor;
                if (cursor == end) break;
                if (valueCount >= 14)
                    return fail(QStringLiteral("点云第 %1 行列数超出支持的 schema。").arg(lineCounter));
                const double value = std::strtod(cursor, &next);
                if (next == cursor)
                    return fail(QStringLiteral("点云第 %1 行含非数字字段。").arg(lineCounter));
                if (!std::isfinite(value))
                    return fail(QStringLiteral("点云第 %1 行含 NaN/Inf 数值。").arg(lineCounter));
                values[valueCount++] = value;
                cursor = next;
            }
            if (valueCount != 3 && valueCount != 14)
                return fail(QStringLiteral("点云第 %1 行不是 4 列或 15 列完整 schema。")
                    .arg(lineCounter));
            const int xyzOffset = valueCount == 3 ? 0 : 2;
            const double x = values[xyzOffset];
            const double y = values[xyzOffset + 1];
            const double z = values[xyzOffset + 2];
            if (loaded.size() >= limits.maximumValidPoints)
                return fail(QStringLiteral("点云有效点数超过限制。"));

            RobotCalculation::IndexedPoint3D point;
            point.index = static_cast<int>(index);
            point.point = Eigen::Vector3d(x, y, z);
            loaded.push_back(point);
        }

        if (file.error() != QFileDevice::NoError
            || file.pos() != openedFileSize || file.size() != openedFileSize)
            return fail(QStringLiteral("点云文件在读取期间变化或读取失败。"));
        if (loaded.size() < 16)
            return fail(QStringLiteral("点云文件有效点过少（%1）：%2")
                .arg(loaded.size()).arg(cloudFilePath));
        if (progress && !progress(100, QStringLiteral("完成")))
            return fail(QStringLiteral("已取消"));

        points = std::move(loaded);  // 只有所有边界/取消检查全过才原子发布
        return true;
    }
    catch (const std::bad_alloc&)
    {
        return fail(QStringLiteral("点云加载所需内存超出可用范围。"));
    }
    catch (const std::exception& ex)
    {
        return fail(QStringLiteral("点云加载发生异常：%1")
            .arg(QString::fromLocal8Bit(ex.what()).left(256)));
    }
    catch (...)
    {
        return fail(QStringLiteral("点云加载发生未知异常。"));
    }
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
