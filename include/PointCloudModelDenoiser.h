#pragma once

#include "RobotCalculation.h"
#include "WorkpieceMeshBuilder.h"

#include <Eigen/Dense>

#include <QString>
#include <QVector>

#include <functional>
#include <vector>

// 用参考三角网格模型对有噪点云做「点到模型表面距离」去噪。
//
// 设计意图（先测后焊 B 档去噪端）：把一个干净的参考表面（既可是同款工件的逆向
// 重建网格，也可是经 BCPD 等非刚性配准「形变贴合到新点云」后的网格）当作真值表面，
// 点云中偏离该表面超过阈值的点判为噪声（贴面毛刺 / 悬空散点 / 跨面混样）剔除；贴合
// 表面的点——哪怕它体现了新工件真实的尺寸/波形差异——一律保留。
//
// 安全不变量：本模块只「删点」，绝不移动/替换/吸附任何点到模型上，因此不会篡改
// 新工件的真实几何（焊缝位置由保留下来的真实点决定，不会被模型带偏）。
//
// 实现：纯 Eigen + 均匀网格空间哈希加速点到三角形最近距离查询，零第三方点云库依赖
// （不引入 CGAL/PCL），适配本工程对安装包体积的约束。
class PointCloudModelDenoiser
{
public:
    struct Options
    {
        // 点到模型表面距离 > 此阈值判为噪声并剔除（mm）。
        double distanceThresholdMm = 3.0;

        // 空间哈希格边长（mm）。0 = 自动：取 max(2×阈值, 网格三角形平均边长)。
        double cellSizeMm = 0.0;

        // 落在模型覆盖范围外、邻域内找不到任何候选三角形的点（"无表面可参照"）：
        // true 保留（保守，避免误删模型没覆盖到的真实区域）；false 当噪声剔除。
        bool keepPointsWithoutSurface = true;

        // 邻域搜索的格子环数上限（每环向外扩一圈格子直到命中三角形或达上限）。
        int maxSearchRings = 3;
    };

    struct Stats
    {
        int input = 0;        // 输入点数
        int kept = 0;         // 保留点数
        int removedFar = 0;   // 因超距离阈值剔除
        int removedNoSurface = 0;  // 因无表面参照剔除（keepPointsWithoutSurface=false 时）
        double maxKeptDistMm = 0.0;  // 保留点里到表面的最大距离（用于核对阈值是否合理）
        qint64 buildMs = 0;   // 空间哈希构建耗时
        qint64 queryMs = 0;   // 全部点查询耗时
    };

    using LogCallback = std::function<void(const QString&)>;

    // 用模型对点云去噪，返回保留下来的点（保持原 index 与坐标，不做任何位移）。
    // mesh 无效（顶点/三角形不足）时原样返回输入点云并在 stats 标记。
    static QVector<RobotCalculation::IndexedPoint3D> DenoiseByModelDistance(
        const QVector<RobotCalculation::IndexedPoint3D>& cloud,
        const WorkpieceMeshBuilder::Mesh& mesh,
        const Options& options = Options(),
        Stats* stats = nullptr,
        const LogCallback& log = LogCallback());

    // 点到单个三角形的最小距离平方（Ericson, ClosestPtPointTriangle）。公开供测试复用。
    static double PointTriangleDistanceSq(
        const Eigen::Vector3d& p,
        const Eigen::Vector3d& a,
        const Eigen::Vector3d& b,
        const Eigen::Vector3d& c);
};
