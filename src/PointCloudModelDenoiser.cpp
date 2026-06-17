#include "PointCloudModelDenoiser.h"

#include <QElapsedTimer>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace
{
using Vec3 = Eigen::Vector3d;

// 均匀网格空间哈希：三角形按其 AABB 覆盖的格子登记，查询点按所在格邻域取候选三角形。
struct TriangleGrid
{
    Vec3 minBound = Vec3::Zero();
    double cell = 1.0;
    int nx = 1, ny = 1, nz = 1;
    std::vector<std::vector<int>> buckets;  // 大小 nx*ny*nz，存三角形索引

    static int clampi(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

    inline long long cellIndex(int cx, int cy, int cz) const
    {
        return (static_cast<long long>(cz) * ny + cy) * nx + cx;
    }

    void cellOf(const Vec3& p, int& cx, int& cy, int& cz) const
    {
        cx = clampi(static_cast<int>(std::floor((p.x() - minBound.x()) / cell)), 0, nx - 1);
        cy = clampi(static_cast<int>(std::floor((p.y() - minBound.y()) / cell)), 0, ny - 1);
        cz = clampi(static_cast<int>(std::floor((p.z() - minBound.z()) / cell)), 0, nz - 1);
    }
};
}  // namespace

double PointCloudModelDenoiser::PointTriangleDistanceSq(
    const Eigen::Vector3d& p, const Eigen::Vector3d& a,
    const Eigen::Vector3d& b, const Eigen::Vector3d& c)
{
    // Christer Ericson, Real-Time Collision Detection, ClosestPtPointTriangle（Voronoi 区域分支）。
    const Vec3 ab = b - a;
    const Vec3 ac = c - a;
    const Vec3 ap = p - a;
    const double d1 = ab.dot(ap);
    const double d2 = ac.dot(ap);
    if (d1 <= 0.0 && d2 <= 0.0) return ap.squaredNorm();  // 顶点 a

    const Vec3 bp = p - b;
    const double d3 = ab.dot(bp);
    const double d4 = ac.dot(bp);
    if (d3 >= 0.0 && d4 <= d3) return bp.squaredNorm();  // 顶点 b

    const double vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0)
    {
        const double v = d1 / (d1 - d3);
        return (ap - v * ab).squaredNorm();  // 边 ab
    }

    const Vec3 cp = p - c;
    const double d5 = ab.dot(cp);
    const double d6 = ac.dot(cp);
    if (d6 >= 0.0 && d5 <= d6) return cp.squaredNorm();  // 顶点 c

    const double vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0)
    {
        const double w = d2 / (d2 - d6);
        return (ap - w * ac).squaredNorm();  // 边 ac
    }

    const double va = d3 * d6 - d5 * d4;
    if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0)
    {
        const double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return (p - (b + w * (c - b))).squaredNorm();  // 边 bc
    }

    const double denom = 1.0 / (va + vb + vc);
    const double v = vb * denom;
    const double w = vc * denom;
    return (p - (a + ab * v + ac * w)).squaredNorm();  // 三角形内部投影
}

QVector<RobotCalculation::IndexedPoint3D> PointCloudModelDenoiser::DenoiseByModelDistance(
    const QVector<RobotCalculation::IndexedPoint3D>& cloud,
    const WorkpieceMeshBuilder::Mesh& mesh,
    const Options& options, Stats* stats, const LogCallback& log)
{
    Stats local;
    local.input = static_cast<int>(cloud.size());

    auto finish = [&](const QVector<RobotCalculation::IndexedPoint3D>& out)
        -> QVector<RobotCalculation::IndexedPoint3D>
    {
        local.kept = static_cast<int>(out.size());
        if (stats) *stats = local;
        return out;
    };

    if (cloud.isEmpty()) return finish(cloud);
    if (!mesh.IsValid() || mesh.indices.size() < 3)
    {
        if (log) log(QStringLiteral("点云模型去噪：模型无效，跳过（原样返回 %1 点）").arg(cloud.size()));
        return finish(cloud);  // 模型无效则全保留，绝不误删真实点
    }

    const int triCount = mesh.indices.size() / 3;
    const int vtxCount = mesh.vertices.size();

    QElapsedTimer timer;
    timer.start();

    // 顶点一次性转 double。
    std::vector<Vec3> V(static_cast<size_t>(vtxCount));
    for (int i = 0; i < vtxCount; ++i) V[i] = mesh.vertices[i].cast<double>();

    // mesh 包围盒 + 平均边长（用于自动定格边长）。
    Vec3 lo = V[0], hi = V[0];
    for (const Vec3& v : V) { lo = lo.cwiseMin(v); hi = hi.cwiseMax(v); }
    double edgeSum = 0.0;
    long long edgeN = 0;
    for (int t = 0; t < triCount; ++t)
    {
        const quint32 i0 = mesh.indices[t * 3], i1 = mesh.indices[t * 3 + 1], i2 = mesh.indices[t * 3 + 2];
        if (i0 >= static_cast<quint32>(vtxCount) || i1 >= static_cast<quint32>(vtxCount) || i2 >= static_cast<quint32>(vtxCount))
            continue;  // 越界索引跳过
        edgeSum += (V[i1] - V[i0]).norm() + (V[i2] - V[i1]).norm() + (V[i0] - V[i2]).norm();
        edgeN += 3;
    }
    const double avgEdge = edgeN > 0 ? edgeSum / static_cast<double>(edgeN) : options.distanceThresholdMm;

    const double threshold = options.distanceThresholdMm;
    const double thresholdSq = threshold * threshold;
    double cell = options.cellSizeMm > 0.0 ? options.cellSizeMm : std::max(2.0 * threshold, avgEdge);
    if (cell <= 1e-6) cell = std::max(1.0, threshold);

    TriangleGrid grid;
    grid.minBound = lo;
    grid.cell = cell;
    const Vec3 span = hi - lo;
    auto recomputeDims = [&]()
    {
        grid.nx = std::max(1, static_cast<int>(std::ceil(span.x() / grid.cell)) + 1);
        grid.ny = std::max(1, static_cast<int>(std::ceil(span.y() / grid.cell)) + 1);
        grid.nz = std::max(1, static_cast<int>(std::ceil(span.z() / grid.cell)) + 1);
    };
    recomputeDims();
    // 兜底：格子总数过大时增大格边长，避免内存爆掉（工件尺度下通常不触发）。
    const long long kCellCap = 8LL * 1024 * 1024;
    while (static_cast<long long>(grid.nx) * grid.ny * grid.nz > kCellCap && grid.cell < span.maxCoeff())
    {
        grid.cell *= 1.5;
        recomputeDims();
    }

    grid.buckets.assign(static_cast<size_t>(static_cast<long long>(grid.nx) * grid.ny * grid.nz), {});
    for (int t = 0; t < triCount; ++t)
    {
        const quint32 i0 = mesh.indices[t * 3], i1 = mesh.indices[t * 3 + 1], i2 = mesh.indices[t * 3 + 2];
        if (i0 >= static_cast<quint32>(vtxCount) || i1 >= static_cast<quint32>(vtxCount) || i2 >= static_cast<quint32>(vtxCount))
            continue;
        const Vec3& A = V[i0];
        const Vec3& B = V[i1];
        const Vec3& C = V[i2];
        const Vec3 tlo = A.cwiseMin(B).cwiseMin(C);
        const Vec3 thi = A.cwiseMax(B).cwiseMax(C);
        int cx0, cy0, cz0, cx1, cy1, cz1;
        grid.cellOf(tlo, cx0, cy0, cz0);
        grid.cellOf(thi, cx1, cy1, cz1);
        for (int z = cz0; z <= cz1; ++z)
            for (int y = cy0; y <= cy1; ++y)
                for (int x = cx0; x <= cx1; ++x)
                    grid.buckets[static_cast<size_t>(grid.cellIndex(x, y, z))].push_back(t);
    }
    local.buildMs = timer.elapsed();

    // 搜索环数：覆盖阈值球（再多一圈保险），但不超过 maxSearchRings。
    const int rings = std::max(1, std::min(options.maxSearchRings,
        static_cast<int>(std::ceil(threshold / grid.cell)) + 1));

    timer.restart();
    QVector<RobotCalculation::IndexedPoint3D> kept;
    kept.reserve(cloud.size());
    for (const RobotCalculation::IndexedPoint3D& ip : cloud)
    {
        const Vec3& p = ip.point;
        int cx, cy, cz;
        grid.cellOf(p, cx, cy, cz);

        double minSq = std::numeric_limits<double>::max();
        bool found = false;
        const int zlo = std::max(0, cz - rings), zhi = std::min(grid.nz - 1, cz + rings);
        const int ylo = std::max(0, cy - rings), yhi = std::min(grid.ny - 1, cy + rings);
        const int xlo = std::max(0, cx - rings), xhi = std::min(grid.nx - 1, cx + rings);
        for (int z = zlo; z <= zhi && minSq > 1e-12; ++z)
            for (int y = ylo; y <= yhi && minSq > 1e-12; ++y)
                for (int x = xlo; x <= xhi && minSq > 1e-12; ++x)
                {
                    const std::vector<int>& bucket = grid.buckets[static_cast<size_t>(grid.cellIndex(x, y, z))];
                    for (int t : bucket)
                    {
                        found = true;
                        const double dsq = PointTriangleDistanceSq(
                            p, V[mesh.indices[t * 3]], V[mesh.indices[t * 3 + 1]], V[mesh.indices[t * 3 + 2]]);
                        if (dsq < minSq) minSq = dsq;
                        if (minSq <= 1e-12) break;
                    }
                }

        if (!found)
        {
            // 邻域内无任何模型三角形可参照。
            if (options.keepPointsWithoutSurface) kept.push_back(ip);
            else ++local.removedNoSurface;
            continue;
        }
        if (minSq <= thresholdSq)
        {
            kept.push_back(ip);
            const double d = std::sqrt(minSq);
            if (d > local.maxKeptDistMm) local.maxKeptDistMm = d;
        }
        else
        {
            ++local.removedFar;  // 离任何模型面都太远 → 噪声
        }
    }
    local.queryMs = timer.elapsed();

    if (log)
    {
        const QString noSurf = options.keepPointsWithoutSurface
            ? QStringLiteral("无面参照保留")
            : QStringLiteral("无面参照剔除 %1").arg(local.removedNoSurface);
        log(QStringLiteral("点云模型去噪：%1→%2 点（远离剔除 %3，%4，阈值 %5mm，格边 %6mm，建表 %7ms 查询 %8ms）")
                .arg(local.input)
                .arg(kept.size())
                .arg(local.removedFar)
                .arg(noSurf)
                .arg(threshold, 0, 'f', 2)
                .arg(grid.cell, 0, 'f', 2)
                .arg(local.buildMs)
                .arg(local.queryMs));
    }
    return finish(kept);
}
