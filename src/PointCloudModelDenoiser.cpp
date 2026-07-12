#include "PointCloudModelDenoiser.h"

#include <QElapsedTimer>

#include <algorithm>
#include <cmath>
#include <limits>
#include <new>
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
        // 先用浮点范围夹紧再转 int：点云可能远在模型包围盒外，
        // 直接把巨大 double 转 int 会产生未定义/实现定义结果。
        const auto coordinate = [this](double value, double minimum, int count)
        {
            const double scaled = (value - minimum) / cell;
            if (!std::isfinite(scaled) || scaled <= 0.0) return 0;
            if (scaled >= static_cast<double>(count - 1)) return count - 1;
            return clampi(static_cast<int>(std::floor(scaled)), 0, count - 1);
        };
        cx = coordinate(p.x(), minBound.x(), nx);
        cy = coordinate(p.y(), minBound.y(), ny);
        cz = coordinate(p.z(), minBound.z(), nz);
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
    local.input = cloud.size() > std::numeric_limits<int>::max()
        ? std::numeric_limits<int>::max()
        : static_cast<int>(cloud.size());
    const auto isCancelled = [&options]()
    {
        return options.cancelRequested && options.cancelRequested();
    };

    auto finish = [&](const QVector<RobotCalculation::IndexedPoint3D>& out)
        -> QVector<RobotCalculation::IndexedPoint3D>
    {
        local.kept = static_cast<int>(out.size());
        if (stats) *stats = local;
        return out;
    };
    const auto cancelResult = [&]() -> QVector<RobotCalculation::IndexedPoint3D>
    {
        local.cancelled = true;
        return finish({});
    };
    const auto resourceFailure = [&](const QString& message)
        -> QVector<RobotCalculation::IndexedPoint3D>
    {
        local.resourceLimitExceeded = true;
        if (log) log(QStringLiteral("点云模型去噪已安全终止：%1").arg(message));
        return finish({});
    };

    // 空 bucket 本身也占内存；引用总数、单三角形 AABB 覆盖和单 bucket
    // 均独立限幅，避免瘦长/巨大三角形把一个索引扩张成数 GB。
    constexpr long long kMaximumGridCells = 1LL * 1024LL * 1024LL;
    constexpr long long kMaximumCellsPerTriangle = 64LL * 1024LL;
    constexpr long long kMaximumTriangleReferences = 16LL * 1024LL * 1024LL;
    constexpr size_t kMaximumTrianglesPerBucket = 256U * 1024U;
    constexpr int kMaximumVertices = 2'000'000;
    constexpr int kMaximumTriangles = 4'000'000;

    try
    {
        if (isCancelled()) return cancelResult();
        if (cloud.isEmpty()) return finish(cloud);
        if (!mesh.IsValid() || mesh.indices.size() < 3)
        {
            if (log) log(QStringLiteral("点云模型去噪：模型无效，跳过（原样返回 %1 点）").arg(cloud.size()));
            return finish(cloud);  // 模型无效则全保留，绝不误删真实点
        }
        if (mesh.indices.size() % 3 != 0
            || mesh.vertices.size() > kMaximumVertices
            || mesh.indices.size() / 3 > kMaximumTriangles)
            return resourceFailure(QStringLiteral("网格顶点/三角形数超出限制"));
        if (!std::isfinite(options.distanceThresholdMm)
            || options.distanceThresholdMm <= 0.0
            || options.distanceThresholdMm > 1'000'000.0
            || !std::isfinite(options.cellSizeMm)
            || options.cellSizeMm < 0.0
            || options.maxSearchRings < 1
            || options.maxSearchRings > 64)
            return resourceFailure(QStringLiteral("去噪阈值或空间索引参数非法"));

        const int triCount = static_cast<int>(mesh.indices.size() / 3);
        const int vtxCount = static_cast<int>(mesh.vertices.size());

        QElapsedTimer timer;
        timer.start();

        // 顶点一次性转 double；非有限值不允许进入 floor/int 格子换算。
        std::vector<Vec3> V(static_cast<size_t>(vtxCount));
        for (int i = 0; i < vtxCount; ++i)
        {
            if ((i & 0x3ff) == 0 && isCancelled()) return cancelResult();
            if (!mesh.vertices[i].allFinite())
                return resourceFailure(QStringLiteral("网格含 NaN/Inf 顶点"));
            V[static_cast<size_t>(i)] = mesh.vertices[i].cast<double>();
        }

        // mesh 包围盒 + 平均边长（用于自动定格边长）。
        Vec3 lo = V[0], hi = V[0];
        for (int i = 0; i < vtxCount; ++i)
        {
            if ((i & 0x3ff) == 0 && isCancelled()) return cancelResult();
            lo = lo.cwiseMin(V[static_cast<size_t>(i)]);
            hi = hi.cwiseMax(V[static_cast<size_t>(i)]);
        }
        double edgeSum = 0.0;
        long long edgeN = 0;
        for (int t = 0; t < triCount; ++t)
        {
            if ((t & 0x3ff) == 0 && isCancelled()) return cancelResult();
            const quint32 i0 = mesh.indices[t * 3];
            const quint32 i1 = mesh.indices[t * 3 + 1];
            const quint32 i2 = mesh.indices[t * 3 + 2];
            if (i0 >= static_cast<quint32>(vtxCount)
                || i1 >= static_cast<quint32>(vtxCount)
                || i2 >= static_cast<quint32>(vtxCount))
                return resourceFailure(QStringLiteral("网格含越界顶点索引"));
            edgeSum += (V[i1] - V[i0]).norm()
                + (V[i2] - V[i1]).norm()
                + (V[i0] - V[i2]).norm();
            edgeN += 3;
        }

        const double threshold = options.distanceThresholdMm;
        const double thresholdSq = threshold * threshold;
        const double avgEdge = edgeN > 0 && std::isfinite(edgeSum)
            ? edgeSum / static_cast<double>(edgeN)
            : threshold;
        double cell = options.cellSizeMm > 0.0
            ? options.cellSizeMm
            : std::max(2.0 * threshold, avgEdge);
        if (!std::isfinite(cell) || cell <= 1e-6) cell = std::max(1.0, threshold);

        TriangleGrid grid;
        grid.minBound = lo;
        const Vec3 span = hi - lo;
        if (!span.allFinite() || span.minCoeff() < 0.0)
            return resourceFailure(QStringLiteral("网格包围盒非法"));
        const double maxSpan = span.maxCoeff();
        // 在转 int 前先限制单轴格数，避免极端坐标触发浮点到整数溢出。
        if (maxSpan > 0.0) cell = std::max(cell, maxSpan / 1024.0);
        grid.cell = cell;
        const auto recomputeDims = [&]() -> bool
        {
            const double dx = std::ceil(span.x() / grid.cell) + 1.0;
            const double dy = std::ceil(span.y() / grid.cell) + 1.0;
            const double dz = std::ceil(span.z() / grid.cell) + 1.0;
            if (!std::isfinite(dx) || !std::isfinite(dy) || !std::isfinite(dz)
                || dx < 1.0 || dy < 1.0 || dz < 1.0
                || dx > 4096.0 || dy > 4096.0 || dz > 4096.0)
                return false;
            grid.nx = static_cast<int>(dx);
            grid.ny = static_cast<int>(dy);
            grid.nz = static_cast<int>(dz);
            return true;
        };
        if (!recomputeDims())
            return resourceFailure(QStringLiteral("空间索引维度超出限制"));
        long long gridCellCount = static_cast<long long>(grid.nx) * grid.ny * grid.nz;
        while (gridCellCount > kMaximumGridCells && grid.cell < std::max(maxSpan, 1.0))
        {
            if (isCancelled()) return cancelResult();
            grid.cell *= 1.5;
            if (!recomputeDims())
                return resourceFailure(QStringLiteral("空间索引维度超出限制"));
            gridCellCount = static_cast<long long>(grid.nx) * grid.ny * grid.nz;
        }
        if (gridCellCount <= 0 || gridCellCount > kMaximumGridCells)
            return resourceFailure(QStringLiteral("空间索引格子数超出限制"));

        grid.buckets.assign(static_cast<size_t>(gridCellCount), {});
        long long totalReferences = 0;
        for (int t = 0; t < triCount; ++t)
        {
            if ((t & 0x3ff) == 0 && isCancelled()) return cancelResult();
            const quint32 i0 = mesh.indices[t * 3];
            const quint32 i1 = mesh.indices[t * 3 + 1];
            const quint32 i2 = mesh.indices[t * 3 + 2];
            const Vec3& A = V[i0];
            const Vec3& B = V[i1];
            const Vec3& C = V[i2];
            const Vec3 tlo = A.cwiseMin(B).cwiseMin(C);
            const Vec3 thi = A.cwiseMax(B).cwiseMax(C);
            int cx0, cy0, cz0, cx1, cy1, cz1;
            grid.cellOf(tlo, cx0, cy0, cz0);
            grid.cellOf(thi, cx1, cy1, cz1);
            const long long coverX = static_cast<long long>(cx1) - cx0 + 1;
            const long long coverY = static_cast<long long>(cy1) - cy0 + 1;
            const long long coverZ = static_cast<long long>(cz1) - cz0 + 1;
            if (coverX <= 0 || coverY <= 0 || coverZ <= 0
                || coverX > kMaximumCellsPerTriangle / coverY
                || coverX * coverY > kMaximumCellsPerTriangle / coverZ)
                return resourceFailure(QStringLiteral("单三角形 AABB 覆盖格子超出限制"));
            const long long coverage = coverX * coverY * coverZ;
            if (totalReferences > kMaximumTriangleReferences - coverage)
                return resourceFailure(QStringLiteral("三角形空间引用总数超出限制"));

            for (int z = cz0; z <= cz1; ++z)
            {
                for (int y = cy0; y <= cy1; ++y)
                {
                    for (int x = cx0; x <= cx1; ++x)
                    {
                        if ((totalReferences & 0xff) == 0 && isCancelled())
                            return cancelResult();
                        std::vector<int>& bucket = grid.buckets[
                            static_cast<size_t>(grid.cellIndex(x, y, z))];
                        if (bucket.size() >= kMaximumTrianglesPerBucket)
                            return resourceFailure(QStringLiteral("单空间格候选三角形数超出限制"));
                        bucket.push_back(t);
                        ++totalReferences;
                    }
                }
            }
        }
        local.buildMs = timer.elapsed();

        // 搜索环数：覆盖阈值球（再多一圈保险），但不超过 maxSearchRings。
        const int rings = std::max(1, std::min(options.maxSearchRings,
            static_cast<int>(std::ceil(threshold / grid.cell)) + 1));

        timer.restart();
        QVector<RobotCalculation::IndexedPoint3D> kept;
        kept.reserve(cloud.size());
        long long visitedCells = 0;
        long long visitedCandidates = 0;
        for (qsizetype pointIndex = 0; pointIndex < cloud.size(); ++pointIndex)
        {
            if ((pointIndex & 0xff) == 0 && isCancelled()) return cancelResult();
            const RobotCalculation::IndexedPoint3D& ip = cloud.at(pointIndex);
            const Vec3& p = ip.point;
            if (!p.allFinite())
                return resourceFailure(QStringLiteral("待处理点云含 NaN/Inf 坐标"));
            int cx, cy, cz;
            grid.cellOf(p, cx, cy, cz);

            double minSq = std::numeric_limits<double>::max();
            bool found = false;
            const int zlo = std::max(0, cz - rings), zhi = std::min(grid.nz - 1, cz + rings);
            const int ylo = std::max(0, cy - rings), yhi = std::min(grid.ny - 1, cy + rings);
            const int xlo = std::max(0, cx - rings), xhi = std::min(grid.nx - 1, cx + rings);
            for (int z = zlo; z <= zhi && minSq > 1e-12; ++z)
            {
                for (int y = ylo; y <= yhi && minSq > 1e-12; ++y)
                {
                    for (int x = xlo; x <= xhi && minSq > 1e-12; ++x)
                    {
                        if ((visitedCells++ & 0x3f) == 0 && isCancelled())
                            return cancelResult();
                        const std::vector<int>& bucket = grid.buckets[
                            static_cast<size_t>(grid.cellIndex(x, y, z))];
                        for (int t : bucket)
                        {
                            if ((visitedCandidates++ & 0xff) == 0 && isCancelled())
                                return cancelResult();
                            found = true;
                            const double dsq = PointTriangleDistanceSq(
                                p, V[mesh.indices[t * 3]],
                                V[mesh.indices[t * 3 + 1]],
                                V[mesh.indices[t * 3 + 2]]);
                            if (dsq < minSq) minSq = dsq;
                            if (minSq <= 1e-12) break;
                        }
                    }
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
    catch (const std::bad_alloc&)
    {
        return resourceFailure(QStringLiteral("空间索引所需内存超出可用范围"));
    }
    catch (const std::exception& ex)
    {
        return resourceFailure(QStringLiteral("空间索引异常：%1")
            .arg(QString::fromLocal8Bit(ex.what()).left(256)));
    }
    catch (...)
    {
        return resourceFailure(QStringLiteral("空间索引未知异常"));
    }
}
