#include "ReverseMeshSeamProjector.h"

#include <cmath>

bool ReverseMeshSeamProjector::Project(
    const WorkpieceMeshBuilder::Mesh& mesh,
    const QVector<Eigen::Vector3d>& seedPathModelMm,
    Result& result,
    QString& error,
    const Options* tighterOptions)
{
    result = Result();
    error.clear();
    const Options options = tighterOptions != nullptr ? *tighterOptions : Options();
    if (mesh.vertices.size() < 3)
    {
        error = QStringLiteral("逆向网格有效顶点不足，无法执行种子投影。");
        return false;
    }
    if (seedPathModelMm.size() < 2)
    {
        error = QStringLiteral("至少需要2个模型坐标种子点；无种子时禁止盲目提取焊缝。");
        return false;
    }
    if (!std::isfinite(options.sampleStepMm) || options.sampleStepMm <= 0.0
        || !std::isfinite(options.stationWindowMm) || options.stationWindowMm <= 0.0
        || !std::isfinite(options.transverseWindowMm) || options.transverseWindowMm <= 0.0
        || !std::isfinite(options.zBandBelowMm) || options.zBandBelowMm <= 0.0
        || !std::isfinite(options.zBandAboveMm) || options.zBandAboveMm <= 0.0
        || options.maximumOutputPoints < 2 || options.maximumOutputPoints > 4096)
    {
        error = QStringLiteral("逆向网格种子投影参数无效或超过安全上限。");
        return false;
    }
    if (seedPathModelMm.size() > options.maximumOutputPoints)
    {
        error = QStringLiteral("种子点数 %1 超过模板单焊缝上限 %2。")
            .arg(seedPathModelMm.size()).arg(options.maximumOutputPoints);
        return false;
    }

    QVector<RobotCalculation::IndexedPoint3D> cloud;
    cloud.reserve(mesh.vertices.size());
    for (int index = 0; index < mesh.vertices.size(); ++index)
    {
        const Eigen::Vector3f& vertex = mesh.vertices.at(index);
        const Eigen::Vector3d point = vertex.cast<double>();
        if (!point.allFinite()) continue;
        RobotCalculation::IndexedPoint3D indexed;
        indexed.index = index + 1;
        indexed.point = point;
        cloud.push_back(indexed);
    }
    QVector<RobotCalculation::IndexedPoint3D> seed;
    seed.reserve(seedPathModelMm.size());
    for (int index = 0; index < seedPathModelMm.size(); ++index)
    {
        if (!seedPathModelMm.at(index).allFinite())
        {
            error = QStringLiteral("种子轨迹含非有限模型坐标。");
            return false;
        }
        RobotCalculation::IndexedPoint3D indexed;
        indexed.index = index + 1;
        indexed.point = seedPathModelMm.at(index);
        seed.push_back(indexed);
    }

    RobotCalculation::LowerWeldFilterParams params;
    params.geometryStrategy = RobotCalculation::LowerWeldGeometryStrategy::WorkpieceProjection;
    params.sampleAxis = options.sampleAxis;
    params.sampleStep = options.sampleStepMm;
    params.projectionStationWindowMm = options.stationWindowMm;
    params.projectionTransverseWindowMm = options.transverseWindowMm;
    params.projectionZBandBelowMm = options.zBandBelowMm;
    params.projectionZBandAboveMm = options.zBandAboveMm;
    const RobotCalculation::LowerWeldFilterResult projected =
        RobotCalculation::ProjectWorkpieceCloudToLowerWeldPath(cloud, seed, params);
    if (!projected.ok)
    {
        error = projected.error.isEmpty()
            ? QStringLiteral("逆向网格种子投影失败。") : projected.error;
        return false;
    }
    if (projected.points.size() < 2
        || projected.points.size() > options.maximumOutputPoints)
    {
        error = QStringLiteral("逆向网格投影输出点数无效或超过安全上限。");
        return false;
    }
    result.pathModelMm.reserve(projected.points.size());
    for (const RobotCalculation::LowerWeldFilterPoint& point : projected.points)
    {
        if (!point.point.allFinite())
        {
            result = Result();
            error = QStringLiteral("逆向网格投影输出含非有限坐标。");
            return false;
        }
        result.pathModelMm.push_back(point.point);
    }
    result.meshVertexCount = cloud.size();
    result.seedPointCount = seed.size();
    result.measuredPointCount = projected.measuredCount;
    result.fallbackPointCount = projected.extendedCount;
    return true;
}
