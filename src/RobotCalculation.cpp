#include "RobotCalculation.h"
#include "RobotPoseTransform.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include <QPair>

namespace
{
double SampleAxisValue(const Eigen::Vector3d& point, RobotCalculation::SampleAxis axis)
{
    return axis == RobotCalculation::SampleAxis::AxisX ? point.x() : point.y();
}

Eigen::Vector3d SetSampleAxisValue(const Eigen::Vector3d& point, RobotCalculation::SampleAxis axis, double axisValue)
{
    Eigen::Vector3d adjusted = point;
    if (axis == RobotCalculation::SampleAxis::AxisX)
    {
        adjusted.x() = axisValue;
    }
    else
    {
        adjusted.y() = axisValue;
    }
    return adjusted;
}

void AppendExpandedGroovePoint(
    QVector<RobotCalculation::LowerWeldClassifiedPoint>& points,
    int& nextIndex,
    const Eigen::Vector3d& point,
    RobotCalculation::LowerWeldPointType type,
    const QString& source)
{
    RobotCalculation::LowerWeldClassifiedPoint classifiedPoint;
    classifiedPoint.index = nextIndex++;
    classifiedPoint.point = point;
    classifiedPoint.type = type;
    classifiedPoint.source = source;
    points.push_back(classifiedPoint);
}

struct GeometryProjectedPoint
{
    int inputIndex = 0;
    Eigen::Vector3d point = Eigen::Vector3d::Zero();
    double s = 0.0;
    double h = 0.0;
    double n = 0.0;
    double smoothH = 0.0;
    double smoothN = 0.0;
};

Eigen::Vector3d UnitAxis(int dimension)
{
    Eigen::Vector3d axis = Eigen::Vector3d::Zero();
    axis(std::max(0, std::min(2, dimension))) = 1.0;
    return axis;
}

int DominantDimension(const Eigen::Vector3d& axis)
{
    int dimension = 0;
    double value = std::abs(axis.x());
    if (std::abs(axis.y()) > value)
    {
        dimension = 1;
        value = std::abs(axis.y());
    }
    if (std::abs(axis.z()) > value)
    {
        dimension = 2;
    }
    return dimension;
}

QPair<Eigen::Vector3d, Eigen::Vector3d> BuildGeometryAxes(
    const QVector<RobotCalculation::IndexedPoint3D>& validPoints,
    const Eigen::Vector3d& center)
{
    Eigen::Vector3d mainAxis = Eigen::Vector3d::UnitY();
    Eigen::Vector3d sideAxis = Eigen::Vector3d::UnitX();

    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (const RobotCalculation::IndexedPoint3D& sample : validPoints)
    {
        const Eigen::Vector3d delta = sample.point - center;
        covariance += delta * delta.transpose();
    }
    covariance /= static_cast<double>(std::max(1, static_cast<int>(validPoints.size())));

    bool hasPcaAxes = false;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    if (solver.info() == Eigen::Success)
    {
        mainAxis = solver.eigenvectors().col(2);
        sideAxis = solver.eigenvectors().col(1);
        hasPcaAxes = mainAxis.squaredNorm() > std::numeric_limits<double>::epsilon()
            && sideAxis.squaredNorm() > std::numeric_limits<double>::epsilon();
    }

    if (!hasPcaAxes)
    {
        Eigen::Vector3d minPoint = validPoints.first().point;
        Eigen::Vector3d maxPoint = validPoints.first().point;
        for (const RobotCalculation::IndexedPoint3D& sample : validPoints)
        {
            minPoint = minPoint.cwiseMin(sample.point);
            maxPoint = maxPoint.cwiseMax(sample.point);
        }

        const Eigen::Vector3d ranges = maxPoint - minPoint;
        int mainDimension = 0;
        if (ranges.y() > ranges.x())
        {
            mainDimension = 1;
        }
        if (ranges.z() > ranges(mainDimension))
        {
            mainDimension = 2;
        }

        int sideDimension = mainDimension == 0 ? 1 : 0;
        for (int dimension = 0; dimension < 3; ++dimension)
        {
            if (dimension != mainDimension && ranges(dimension) > ranges(sideDimension))
            {
                sideDimension = dimension;
            }
        }

        mainAxis = UnitAxis(mainDimension);
        sideAxis = UnitAxis(sideDimension);
    }

    if (mainAxis.squaredNorm() <= std::numeric_limits<double>::epsilon())
    {
        mainAxis = Eigen::Vector3d::UnitY();
    }
    mainAxis.normalize();

    const Eigen::Vector3d scanDelta = validPoints.last().point - validPoints.first().point;
    if (mainAxis.dot(scanDelta) < 0.0)
    {
        mainAxis = -mainAxis;
    }

    sideAxis -= mainAxis * sideAxis.dot(mainAxis);
    if (sideAxis.squaredNorm() <= std::numeric_limits<double>::epsilon())
    {
        sideAxis = UnitAxis((DominantDimension(mainAxis) + 1) % 3);
        sideAxis -= mainAxis * sideAxis.dot(mainAxis);
    }
    if (sideAxis.squaredNorm() <= std::numeric_limits<double>::epsilon())
    {
        sideAxis = Eigen::Vector3d::UnitX();
    }
    sideAxis.normalize();

    const int dominantSideDimension = DominantDimension(sideAxis);
    if (sideAxis(dominantSideDimension) < 0.0)
    {
        sideAxis = -sideAxis;
    }

    return qMakePair(mainAxis, sideAxis);
}

void SmoothGeometryProjectedPoints(QVector<GeometryProjectedPoint>* projected, int smoothRadius)
{
    if (projected == nullptr || projected->isEmpty())
    {
        return;
    }

    const int radius = std::max(1, std::min(8, smoothRadius));
    const QVector<GeometryProjectedPoint> source = *projected;
    const int projectedCount = static_cast<int>(source.size());
    for (int index = 0; index < projectedCount; ++index)
    {
        const int begin = std::max(0, index - radius);
        const int end = std::min(projectedCount - 1, index + radius);
        double sideSum = 0.0;
        double normalSum = 0.0;
        int count = 0;
        for (int sampleIndex = begin; sampleIndex <= end; ++sampleIndex)
        {
            sideSum += source[sampleIndex].h;
            normalSum += source[sampleIndex].n;
            ++count;
        }
        (*projected)[index].smoothH = count > 0 ? sideSum / static_cast<double>(count) : source[index].h;
        (*projected)[index].smoothN = count > 0 ? normalSum / static_cast<double>(count) : source[index].n;
    }
}

QVector<GeometryProjectedPoint> ProjectGeometryPoints(
    const QVector<RobotCalculation::IndexedPoint3D>& validPoints,
    const Eigen::Vector3d& center,
    const Eigen::Vector3d& mainAxis,
    const Eigen::Vector3d& sideAxis,
    const Eigen::Vector3d& normalAxis,
    int smoothRadius)
{
    QVector<GeometryProjectedPoint> projected;
    projected.reserve(validPoints.size());
    for (const RobotCalculation::IndexedPoint3D& sample : validPoints)
    {
        const Eigen::Vector3d delta = sample.point - center;
        GeometryProjectedPoint point;
        point.inputIndex = sample.index;
        point.point = sample.point;
        point.s = delta.dot(mainAxis);
        point.h = delta.dot(sideAxis);
        point.n = delta.dot(normalAxis);
        point.smoothH = point.h;
        point.smoothN = point.n;
        projected.push_back(point);
    }

    SmoothGeometryProjectedPoints(&projected, smoothRadius);
    return projected;
}

double GeometryDistanceToSegment2D(
    const QVector<GeometryProjectedPoint>& points,
    int first,
    int last,
    int index)
{
    const double x1 = points[first].s;
    const double y1 = points[first].smoothH;
    const double x2 = points[last].s;
    const double y2 = points[last].smoothH;
    const double x0 = points[index].s;
    const double y0 = points[index].smoothH;

    const double dx = x2 - x1;
    const double dy = y2 - y1;
    const double denominator = dx * dx + dy * dy;
    if (denominator <= std::numeric_limits<double>::epsilon())
    {
        const double px = x0 - x1;
        const double py = y0 - y1;
        return std::sqrt(px * px + py * py);
    }

    const double ratio = ((x0 - x1) * dx + (y0 - y1) * dy) / denominator;
    const double projectedX = x1 + ratio * dx;
    const double projectedY = y1 + ratio * dy;
    const double px = x0 - projectedX;
    const double py = y0 - projectedY;
    return std::sqrt(px * px + py * py);
}

void DouglasPeuckerGeometry(
    const QVector<GeometryProjectedPoint>& points,
    int first,
    int last,
    double tolerance,
    QVector<char>& keep)
{
    if (last <= first + 1)
    {
        return;
    }

    double maxDistance = -1.0;
    int maxIndex = -1;
    for (int index = first + 1; index < last; ++index)
    {
        const double distance = GeometryDistanceToSegment2D(points, first, last, index);
        if (distance > maxDistance)
        {
            maxDistance = distance;
            maxIndex = index;
        }
    }

    if (maxIndex >= 0 && maxDistance > tolerance)
    {
        keep[maxIndex] = 1;
        DouglasPeuckerGeometry(points, first, maxIndex, tolerance, keep);
        DouglasPeuckerGeometry(points, maxIndex, last, tolerance, keep);
    }
}

QVector<int> BuildGeometryKeyIndexes(
    const QVector<GeometryProjectedPoint>& projected,
    const RobotCalculation::LowerWeldFilterParams& params)
{
    QVector<int> keyIndexes;
    if (projected.isEmpty())
    {
        return keyIndexes;
    }
    if (projected.size() == 1)
    {
        keyIndexes.push_back(0);
        return keyIndexes;
    }

    const double tolerance = std::max(0.5, std::min(8.0, params.piecewiseFitTolerance > 0.0
        ? params.piecewiseFitTolerance
        : 2.0));
    QVector<char> keep(projected.size(), 0);
    keep[0] = 1;
    keep[projected.size() - 1] = 1;
    DouglasPeuckerGeometry(projected, 0, projected.size() - 1, tolerance, keep);

    for (int index = 0; index < keep.size(); ++index)
    {
        if (keep[index])
        {
            keyIndexes.push_back(index);
        }
    }

    const double minSegmentLength = std::max(4.0, params.sampleStep * 2.0);
    bool removed = true;
    while (removed && keyIndexes.size() > 2)
    {
        removed = false;
        for (int index = 1; index + 1 < keyIndexes.size(); ++index)
        {
            const Eigen::Vector3d& previous = projected[keyIndexes[index - 1]].point;
            const Eigen::Vector3d& current = projected[keyIndexes[index]].point;
            const Eigen::Vector3d& next = projected[keyIndexes[index + 1]].point;
            if ((current - previous).norm() < minSegmentLength || (next - current).norm() < minSegmentLength)
            {
                keyIndexes.removeAt(index);
                removed = true;
                break;
            }
        }
    }

    if (keyIndexes.size() < 2)
    {
        keyIndexes.clear();
        keyIndexes.push_back(0);
        keyIndexes.push_back(projected.size() - 1);
    }

    return keyIndexes;
}

double GeometrySmoothHRange(const QVector<GeometryProjectedPoint>& projected)
{
    if (projected.isEmpty())
    {
        return 0.0;
    }

    double minValue = projected.first().smoothH;
    double maxValue = projected.first().smoothH;
    for (const GeometryProjectedPoint& point : projected)
    {
        minValue = std::min(minValue, point.smoothH);
        maxValue = std::max(maxValue, point.smoothH);
    }
    return maxValue - minValue;
}

double GeometrySmoothHMedian(const QVector<GeometryProjectedPoint>& projected)
{
    QVector<double> values;
    values.reserve(projected.size());
    for (const GeometryProjectedPoint& point : projected)
    {
        values.push_back(point.smoothH);
    }
    if (values.isEmpty())
    {
        return 0.0;
    }

    std::sort(values.begin(), values.end());
    const int middle = values.size() / 2;
    if (values.size() % 2 == 1)
    {
        return values[middle];
    }
    return (values[middle - 1] + values[middle]) * 0.5;
}

QVector<int> PruneRedundantSameSideGeometryKeys(
    const QVector<GeometryProjectedPoint>& projected,
    QVector<int> keyIndexes)
{
    if (keyIndexes.size() <= 4)
    {
        return keyIndexes;
    }

    const double medianH = GeometrySmoothHMedian(projected);
    const double sideDeadband = std::max(0.5, GeometrySmoothHRange(projected) * 0.05);
    auto sideSign = [&](int keyIndex) -> int
    {
        const double delta = projected[keyIndex].smoothH - medianH;
        if (delta > sideDeadband)
        {
            return 1;
        }
        if (delta < -sideDeadband)
        {
            return -1;
        }
        return 0;
    };

    bool removed = true;
    while (removed && keyIndexes.size() > 4)
    {
        removed = false;
        for (int index = 1; index + 1 < keyIndexes.size() - 1; ++index)
        {
            const int previousSign = sideSign(keyIndexes[index - 1]);
            const int currentSign = sideSign(keyIndexes[index]);
            const int nextSign = sideSign(keyIndexes[index + 1]);
            if (currentSign == 0 || previousSign != currentSign || nextSign != currentSign)
            {
                continue;
            }

            // A corrugated seam needs both ends of a high/low platform. A third
            // key on the same side is usually a small kink inside that platform,
            // and keeping it creates crossed 2 mm expansion segments.
            keyIndexes.removeAt(index);
            removed = true;
            break;
        }
    }

    return keyIndexes;
}

struct GeometryFittedLine2D
{
    Eigen::Vector2d point = Eigen::Vector2d::Zero();
    Eigen::Vector2d direction = Eigen::Vector2d::UnitX();
    bool valid = false;
};

struct GeometryFittedScalarLine
{
    double centerS = 0.0;
    double centerValue = 0.0;
    double slope = 0.0;
    bool valid = false;

    double valueAt(double station) const
    {
        return centerValue + slope * (station - centerS);
    }
};

double Cross2D(const Eigen::Vector2d& first, const Eigen::Vector2d& second)
{
    return first.x() * second.y() - first.y() * second.x();
}

double GeometryMedianScalar(QVector<double> values);

Eigen::Vector2d GeometryProjection2D(const GeometryProjectedPoint& point)
{
    return Eigen::Vector2d(point.s, point.h);
}

Eigen::Vector2d GeometrySmoothedProjection2D(const GeometryProjectedPoint& point)
{
    return Eigen::Vector2d(point.s, point.smoothH);
}

double GeometryPointToSegmentDistance2D(
    const Eigen::Vector2d& point,
    const Eigen::Vector2d& first,
    const Eigen::Vector2d& second)
{
    const Eigen::Vector2d segment = second - first;
    const double denominator = segment.squaredNorm();
    if (denominator <= std::numeric_limits<double>::epsilon())
    {
        return (point - first).norm();
    }

    const double ratio = std::max(0.0, std::min(1.0, (point - first).dot(segment) / denominator));
    return (point - (first + segment * ratio)).norm();
}

double NearestGeometryProjectionDistance(
    const QVector<GeometryProjectedPoint>& projected,
    const Eigen::Vector2d& projection,
    int firstIndex,
    int lastIndex)
{
    if (projected.isEmpty())
    {
        return std::numeric_limits<double>::infinity();
    }

    const int begin = std::max(0, std::min(firstIndex, lastIndex));
    const int end = std::min(static_cast<int>(projected.size()) - 1, std::max(firstIndex, lastIndex));
    double nearestDistance = std::numeric_limits<double>::infinity();
    for (int index = begin; index <= end; ++index)
    {
        nearestDistance = std::min(
            nearestDistance,
            (GeometrySmoothedProjection2D(projected[index]) - projection).norm());
    }
    return nearestDistance;
}

bool GeometryLocalMainBand(
    const QVector<GeometryProjectedPoint>& projected,
    double station,
    double stationRadius,
    double baseThreshold,
    double* medianH,
    double* threshold,
    int* supportCount)
{
    QVector<double> values;
    values.reserve(32);
    for (const GeometryProjectedPoint& point : projected)
    {
        if (std::abs(point.s - station) <= stationRadius)
        {
            values.push_back(point.smoothH);
        }
    }
    if (values.size() < 8)
    {
        return false;
    }

    const double medianValue = GeometryMedianScalar(values);
    QVector<double> deviations;
    deviations.reserve(values.size());
    for (double value : values)
    {
        deviations.push_back(std::abs(value - medianValue));
    }

    const double madValue = GeometryMedianScalar(deviations);
    const double thresholdValue = std::max(baseThreshold, std::min(8.0, madValue * 4.0 + 0.8));
    int support = 0;
    for (double value : values)
    {
        if (std::abs(value - medianValue) <= thresholdValue)
        {
            ++support;
        }
    }

    if (support * 2 < values.size())
    {
        return false;
    }

    if (medianH != nullptr)
    {
        *medianH = medianValue;
    }
    if (threshold != nullptr)
    {
        *threshold = thresholdValue;
    }
    if (supportCount != nullptr)
    {
        *supportCount = support;
    }
    return true;
}

Eigen::Vector3d GeometryPointFromProjection(
    const Eigen::Vector3d& center,
    const Eigen::Vector3d& mainAxis,
    const Eigen::Vector3d& sideAxis,
    const Eigen::Vector3d& normalAxis,
    const Eigen::Vector2d& projection,
    double normalValue)
{
    return center
        + mainAxis * projection.x()
        + sideAxis * projection.y()
        + normalAxis * normalValue;
}

GeometryFittedLine2D FitGeometrySegmentLine(
    const QVector<GeometryProjectedPoint>& projected,
    int firstIndex,
    int lastIndex)
{
    GeometryFittedLine2D line;
    if (projected.isEmpty())
    {
        return line;
    }

    const int begin = std::max(0, std::min(firstIndex, lastIndex));
    const int end = std::min(static_cast<int>(projected.size()) - 1, std::max(firstIndex, lastIndex));
    const int pointCount = end - begin + 1;
    if (pointCount <= 1)
    {
        return line;
    }

    // Trim a small transition area near corners, but keep enough samples for
    // short segments. The fitted line represents the local straight edge, not
    // the rounded/noisy turning area.
    const int trim = pointCount >= 12 ? std::min(pointCount / 6, 8) : 0;
    const int fitBegin = begin + trim;
    const int fitEnd = end - trim;
    const int fitCount = fitEnd - fitBegin + 1;
    if (fitCount < 2)
    {
        const Eigen::Vector2d p0 = GeometrySmoothedProjection2D(projected[begin]);
        const Eigen::Vector2d p1 = GeometrySmoothedProjection2D(projected[end]);
        const Eigen::Vector2d direction = p1 - p0;
        if (direction.squaredNorm() <= std::numeric_limits<double>::epsilon())
        {
            return line;
        }
        line.point = (p0 + p1) * 0.5;
        line.direction = direction.normalized();
        line.valid = true;
        return line;
    }

    Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
    for (int index = fitBegin; index <= fitEnd; ++index)
    {
        centroid += GeometrySmoothedProjection2D(projected[index]);
    }
    centroid /= static_cast<double>(fitCount);

    Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
    for (int index = fitBegin; index <= fitEnd; ++index)
    {
        const Eigen::Vector2d delta = GeometrySmoothedProjection2D(projected[index]) - centroid;
        covariance += delta * delta.transpose();
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(covariance);
    if (solver.info() == Eigen::Success)
    {
        line.direction = solver.eigenvectors().col(1);
    }
    else
    {
        line.direction = GeometrySmoothedProjection2D(projected[end]) - GeometrySmoothedProjection2D(projected[begin]);
    }

    if (line.direction.squaredNorm() <= std::numeric_limits<double>::epsilon())
    {
        return line;
    }
    line.direction.normalize();

    const Eigen::Vector2d segmentDirection = GeometrySmoothedProjection2D(projected[end]) - GeometrySmoothedProjection2D(projected[begin]);
    if (line.direction.dot(segmentDirection) < 0.0)
    {
        line.direction = -line.direction;
    }

    line.point = centroid;
    line.valid = true;
    return line;
}

double GeometryMedianScalar(QVector<double> values)
{
    if (values.isEmpty())
    {
        return 0.0;
    }
    std::sort(values.begin(), values.end());
    const int middle = values.size() / 2;
    if ((values.size() % 2) == 1)
    {
        return values[middle];
    }
    return (values[middle - 1] + values[middle]) * 0.5;
}

double GeometryMedianSmoothHInRange(
    const QVector<GeometryProjectedPoint>& projected,
    int begin,
    int end)
{
    QVector<double> values;
    const int clampedBegin = std::max(0, begin);
    const int clampedEnd = std::min(static_cast<int>(projected.size()) - 1, end);
    if (clampedEnd < clampedBegin)
    {
        return 0.0;
    }

    values.reserve(clampedEnd - clampedBegin + 1);
    for (int index = clampedBegin; index <= clampedEnd; ++index)
    {
        values.push_back(projected[index].smoothH);
    }
    return GeometryMedianScalar(values);
}

double GeometryMadSmoothHInRange(
    const QVector<GeometryProjectedPoint>& projected,
    int begin,
    int end,
    double medianValue)
{
    QVector<double> deviations;
    const int clampedBegin = std::max(0, begin);
    const int clampedEnd = std::min(static_cast<int>(projected.size()) - 1, end);
    if (clampedEnd < clampedBegin)
    {
        return 0.0;
    }

    deviations.reserve(clampedEnd - clampedBegin + 1);
    for (int index = clampedBegin; index <= clampedEnd; ++index)
    {
        deviations.push_back(std::abs(projected[index].smoothH - medianValue));
    }
    return GeometryMedianScalar(deviations);
}

double GeometryMedianSmoothNInRange(
    const QVector<GeometryProjectedPoint>& projected,
    int begin,
    int end)
{
    QVector<double> values;
    const int clampedBegin = std::max(0, begin);
    const int clampedEnd = std::min(static_cast<int>(projected.size()) - 1, end);
    if (clampedEnd < clampedBegin)
    {
        return 0.0;
    }

    values.reserve(clampedEnd - clampedBegin + 1);
    for (int index = clampedBegin; index <= clampedEnd; ++index)
    {
        values.push_back(projected[index].smoothN);
    }
    return GeometryMedianScalar(values);
}

double GeometryMadSmoothNInRange(
    const QVector<GeometryProjectedPoint>& projected,
    int begin,
    int end,
    double medianValue)
{
    QVector<double> deviations;
    const int clampedBegin = std::max(0, begin);
    const int clampedEnd = std::min(static_cast<int>(projected.size()) - 1, end);
    if (clampedEnd < clampedBegin)
    {
        return 0.0;
    }

    deviations.reserve(clampedEnd - clampedBegin + 1);
    for (int index = clampedBegin; index <= clampedEnd; ++index)
    {
        deviations.push_back(std::abs(projected[index].smoothN - medianValue));
    }
    return GeometryMedianScalar(deviations);
}

double GeometryLocalNormalValue(
    const QVector<GeometryProjectedPoint>& projected,
    int fallbackIndex,
    double station,
    double stationRadius,
    double baseThreshold)
{
    if (projected.isEmpty())
    {
        return 0.0;
    }

    const int clampedFallback = std::max(0, std::min(fallbackIndex, static_cast<int>(projected.size()) - 1));
    double mainBandH = 0.0;
    double mainBandThreshold = 0.0;
    const bool hasMainBand = GeometryLocalMainBand(
        projected,
        station,
        stationRadius,
        baseThreshold,
        &mainBandH,
        &mainBandThreshold,
        nullptr);

    QVector<double> values;
    values.reserve(32);
    for (const GeometryProjectedPoint& point : projected)
    {
        if (std::abs(point.s - station) > stationRadius)
        {
            continue;
        }
        if (hasMainBand && std::abs(point.smoothH - mainBandH) > mainBandThreshold * 1.35)
        {
            continue;
        }
        values.push_back(point.smoothN);
    }

    if (values.size() >= 4)
    {
        return GeometryMedianScalar(values);
    }

    values.clear();
    const int indexRadius = 8;
    const int begin = std::max(0, clampedFallback - indexRadius);
    const int end = std::min(static_cast<int>(projected.size()) - 1, clampedFallback + indexRadius);
    values.reserve(end - begin + 1);
    for (int index = begin; index <= end; ++index)
    {
        values.push_back(projected[index].smoothN);
    }

    return values.isEmpty() ? projected[clampedFallback].smoothN : GeometryMedianScalar(values);
}

QVector<int> PruneGeometrySpikeDrivenKeys(
    const QVector<GeometryProjectedPoint>& projected,
    QVector<int> keyIndexes,
    const RobotCalculation::LowerWeldFilterParams& params)
{
    if (projected.size() < 24 || keyIndexes.size() <= 3)
    {
        return keyIndexes;
    }

    const int projectedCount = static_cast<int>(projected.size());
    const int smoothRadius = std::max(1, params.smoothRadius);
    const int baselineRadius = std::max(18, std::min(90, smoothRadius * 8 + 26));
    const int guardCount = std::max(4, std::min(14, baselineRadius / 5));
    const double baseThreshold = std::max(1.8, std::min(6.0, params.piecewiseFitTolerance * 1.4 + 0.8));
    const double maxSpikeSpan = std::max(10.0, std::min(48.0, params.sampleStep * 20.0));

    bool removed = true;
    while (removed && keyIndexes.size() > 3)
    {
        removed = false;
        for (int keyPosition = 1; keyPosition + 1 < keyIndexes.size(); ++keyPosition)
        {
            const int keyIndex = keyIndexes[keyPosition];
            if (keyIndex <= guardCount || keyIndex + guardCount >= projectedCount)
            {
                continue;
            }

            const int previousKey = keyIndexes[keyPosition - 1];
            const int nextKey = keyIndexes[keyPosition + 1];
            const double previousSpan = std::abs(projected[keyIndex].s - projected[previousKey].s);
            const double nextSpan = std::abs(projected[nextKey].s - projected[keyIndex].s);
            if (previousSpan < params.sampleStep * 2.0 || nextSpan < params.sampleStep * 2.0)
            {
                continue;
            }

            const int leftBegin = std::max(previousKey, keyIndex - baselineRadius);
            const int leftEnd = keyIndex - guardCount;
            const int rightBegin = keyIndex + guardCount;
            const int rightEnd = std::min(nextKey, keyIndex + baselineRadius);
            if (leftEnd - leftBegin + 1 < 5 || rightEnd - rightBegin + 1 < 5)
            {
                continue;
            }

            const double leftMedian = GeometryMedianSmoothHInRange(projected, leftBegin, leftEnd);
            const double rightMedian = GeometryMedianSmoothHInRange(projected, rightBegin, rightEnd);
            const double baseline = (leftMedian + rightMedian) * 0.5;
            const double localMedian = GeometryMedianSmoothHInRange(projected, leftBegin, rightEnd);
            const double localMad = GeometryMadSmoothHInRange(projected, leftBegin, rightEnd, localMedian);
            const double threshold = std::max(baseThreshold, std::min(8.0, localMad * 3.5 + 0.8));
            const double sideGap = std::abs(leftMedian - rightMedian);
            const double prominence = std::abs(projected[keyIndex].smoothH - baseline);

            // Real corrugation corners change the baseline on both sides. A short
            // lifted branch/noise island usually has similar side baselines and a
            // single high/low key in the middle; do not let that key anchor the
            // expanded weld line.
            if (sideGap > threshold * 1.35 || prominence < threshold * 1.35)
            {
                continue;
            }

            const int sign = projected[keyIndex].smoothH >= baseline ? 1 : -1;
            int runBegin = keyIndex;
            int runEnd = keyIndex;
            while (runBegin - 1 > previousKey
                && sign * (projected[runBegin - 1].smoothH - baseline) > threshold * 0.75)
            {
                --runBegin;
            }
            while (runEnd + 1 < nextKey
                && sign * (projected[runEnd + 1].smoothH - baseline) > threshold * 0.75)
            {
                ++runEnd;
            }

            const double runSpan = std::abs(projected[runEnd].s - projected[runBegin].s);
            const int runCount = runEnd - runBegin + 1;
            if (runSpan <= maxSpikeSpan || runCount <= 24)
            {
                keyIndexes.removeAt(keyPosition);
                removed = true;
                break;
            }
        }
    }

    return keyIndexes;
}

QVector<GeometryProjectedPoint> RemoveGeometryProjectedBranchOutliers(
    const QVector<GeometryProjectedPoint>& projected,
    const RobotCalculation::LowerWeldFilterParams& params,
    int* rejectedCount)
{
    if (rejectedCount != nullptr)
    {
        *rejectedCount = 0;
    }
    if (projected.size() < 32)
    {
        return projected;
    }

    QVector<GeometryProjectedPoint> current = projected;
    int totalRejected = 0;
    for (int pass = 0; pass < 2; ++pass)
    {
        const int projectedCount = static_cast<int>(current.size());
        const int smoothRadius = std::max(1, params.smoothRadius);
        const int baselineRadius = std::max(24, std::min(120, smoothRadius * 10 + 34));
        const int guardCount = std::max(5, std::min(18, baselineRadius / 5));
        const double baseThreshold = std::max(1.6, std::min(5.5, params.piecewiseFitTolerance * 1.25 + 0.7));
        const double maxBranchSpan = std::max(28.0, std::min(110.0, params.sampleStep * 45.0));
        const double sameStationRadius = std::max(6.0, std::min(36.0, params.sampleStep * 18.0));
        const int minSameStationNeighborCount = 10;

        QVector<char> candidate(projectedCount, 0);
        for (int index = guardCount; index + guardCount < projectedCount; ++index)
        {
            const int leftBegin = std::max(0, index - baselineRadius);
            const int leftEnd = index - guardCount;
            const int rightBegin = index + guardCount;
            const int rightEnd = std::min(projectedCount - 1, index + baselineRadius);
            if (leftEnd - leftBegin + 1 < 6 || rightEnd - rightBegin + 1 < 6)
            {
                continue;
            }

            const double leftMedian = GeometryMedianSmoothHInRange(current, leftBegin, leftEnd);
            const double rightMedian = GeometryMedianSmoothHInRange(current, rightBegin, rightEnd);
            const double baseline = (leftMedian + rightMedian) * 0.5;
            const double localMedian = GeometryMedianSmoothHInRange(current, leftBegin, rightEnd);
            const double localMad = GeometryMadSmoothHInRange(current, leftBegin, rightEnd, localMedian);
            const double threshold = std::max(baseThreshold, std::min(7.5, localMad * 3.0 + 0.7));
            const double sideGap = std::abs(leftMedian - rightMedian);
            const double prominence = std::abs(current[index].smoothH - baseline);
            const double leftNormalMedian = GeometryMedianSmoothNInRange(current, leftBegin, leftEnd);
            const double rightNormalMedian = GeometryMedianSmoothNInRange(current, rightBegin, rightEnd);
            const double normalBaseline = (leftNormalMedian + rightNormalMedian) * 0.5;
            const double localNormalMedian = GeometryMedianSmoothNInRange(current, leftBegin, rightEnd);
            const double localNormalMad = GeometryMadSmoothNInRange(current, leftBegin, rightEnd, localNormalMedian);
            const double normalThreshold = std::max(baseThreshold, std::min(7.5, localNormalMad * 3.0 + 0.7));
            const double normalSideGap = std::abs(leftNormalMedian - rightNormalMedian);
            const double normalProminence = std::abs(current[index].smoothN - normalBaseline);

            // If both sides stay on the same baseline but the middle suddenly
            // leaves it, this is usually a laser branch/noise island rather
            // than a real corrugated-plate corner.
            bool isBranchCandidate = false;
            if (sideGap <= threshold * 1.25 && prominence >= threshold * 1.25)
            {
                isBranchCandidate = true;
            }
            if (normalSideGap <= normalThreshold * 1.25 && normalProminence >= normalThreshold * 1.25)
            {
                isBranchCandidate = true;
            }

            // A branch can be contiguous in acquisition order, so left/right
            // medians alone may treat it as a valid local segment.  Compare
            // points at the same scan station instead: if most nearby samples
            // form a main band and this point sits well outside it, remove it
            // before segment fitting/intersection.
            int sameBegin = index;
            while (sameBegin > 0 && current[index].s - current[sameBegin - 1].s <= sameStationRadius)
            {
                --sameBegin;
            }
            int sameEnd = index;
            while (sameEnd + 1 < projectedCount && current[sameEnd + 1].s - current[index].s <= sameStationRadius)
            {
                ++sameEnd;
            }

            const int sameCount = sameEnd - sameBegin + 1;
            if (sameCount >= minSameStationNeighborCount)
            {
                QVector<double> sameStationValues;
                QVector<double> sameStationNormalValues;
                sameStationValues.reserve(sameCount);
                sameStationNormalValues.reserve(sameCount);
                for (int sameIndex = sameBegin; sameIndex <= sameEnd; ++sameIndex)
                {
                    if (sameIndex != index)
                    {
                        sameStationValues.push_back(current[sameIndex].smoothH);
                        sameStationNormalValues.push_back(current[sameIndex].smoothN);
                    }
                }

                const double sameMedian = GeometryMedianScalar(sameStationValues);
                QVector<double> sameDeviations;
                sameDeviations.reserve(sameStationValues.size());
                for (double value : sameStationValues)
                {
                    sameDeviations.push_back(std::abs(value - sameMedian));
                }

                const double sameMad = GeometryMedianScalar(sameDeviations);
                const double sameThreshold = std::max(baseThreshold, std::min(8.0, sameMad * 4.0 + 0.8));
                int mainBandSupport = 0;
                for (double value : sameStationValues)
                {
                    if (std::abs(value - sameMedian) <= sameThreshold)
                    {
                        ++mainBandSupport;
                    }
                }

                const double pointDelta = std::abs(current[index].smoothH - sameMedian);
                if (pointDelta >= sameThreshold * 1.35
                    && pointDelta >= baseThreshold * 1.6
                    && mainBandSupport * 2 >= sameStationValues.size())
                {
                    isBranchCandidate = true;
                }

                const double sameNormalMedian = GeometryMedianScalar(sameStationNormalValues);
                QVector<double> sameNormalDeviations;
                sameNormalDeviations.reserve(sameStationNormalValues.size());
                for (double value : sameStationNormalValues)
                {
                    sameNormalDeviations.push_back(std::abs(value - sameNormalMedian));
                }

                const double sameNormalMad = GeometryMedianScalar(sameNormalDeviations);
                const double sameNormalThreshold = std::max(baseThreshold, std::min(8.0, sameNormalMad * 4.0 + 0.8));
                int normalBandSupport = 0;
                for (double value : sameStationNormalValues)
                {
                    if (std::abs(value - sameNormalMedian) <= sameNormalThreshold)
                    {
                        ++normalBandSupport;
                    }
                }

                const double normalPointDelta = std::abs(current[index].smoothN - sameNormalMedian);
                if (normalPointDelta >= sameNormalThreshold * 1.35
                    && normalPointDelta >= baseThreshold * 1.6
                    && normalBandSupport * 2 >= sameStationNormalValues.size())
                {
                    isBranchCandidate = true;
                }
            }

            if (isBranchCandidate)
            {
                candidate[index] = 1;
            }
        }

        QVector<char> keep(projectedCount, 1);
        int passRejected = 0;
        for (int index = 0; index < projectedCount;)
        {
            if (!candidate[index])
            {
                ++index;
                continue;
            }

            const int runBegin = index;
            while (index < projectedCount && candidate[index])
            {
                ++index;
            }
            const int runEnd = index - 1;
            const double runSpan = std::abs(current[runEnd].s - current[runBegin].s);
            const int runCount = runEnd - runBegin + 1;
            if (runSpan <= maxBranchSpan || runCount <= 72)
            {
                for (int removeIndex = runBegin; removeIndex <= runEnd; ++removeIndex)
                {
                    keep[removeIndex] = 0;
                    ++passRejected;
                }
            }
        }

        if (passRejected <= 0 || projectedCount - passRejected < 16)
        {
            break;
        }

        QVector<GeometryProjectedPoint> filtered;
        filtered.reserve(projectedCount - passRejected);
        for (int index = 0; index < projectedCount; ++index)
        {
            if (keep[index])
            {
                filtered.push_back(current[index]);
            }
        }
        current = filtered;
        SmoothGeometryProjectedPoints(&current, params.smoothRadius);
        totalRejected += passRejected;
    }

    if (rejectedCount != nullptr)
    {
        *rejectedCount = totalRejected;
    }
    return current;
}

double GeometryClampUnit(double value)
{
    return std::max(-1.0, std::min(1.0, value));
}

Eigen::Vector2d OrientGeometryDirection(Eigen::Vector2d direction, const Eigen::Vector2d& reference)
{
    if (direction.norm() < 1e-9)
    {
        return direction;
    }
    direction.normalize();
    if (reference.norm() > 1e-9 && direction.dot(reference) < 0.0)
    {
        direction = -direction;
    }
    return direction;
}

double GeometryLineDirectionAngle(const Eigen::Vector2d& first, const Eigen::Vector2d& second)
{
    if (first.norm() < 1e-9 || second.norm() < 1e-9)
    {
        return std::numeric_limits<double>::infinity();
    }
    const Eigen::Vector2d firstNormalized = first.normalized();
    const Eigen::Vector2d secondNormalized = second.normalized();
    return std::acos(GeometryClampUnit(std::abs(firstNormalized.dot(secondNormalized))));
}

double GeometryPointLineDistance2D(const Eigen::Vector2d& point, const GeometryFittedLine2D& line)
{
    if (!line.valid || line.direction.norm() < 1e-9)
    {
        return std::numeric_limits<double>::infinity();
    }
    return std::abs(Cross2D(point - line.point, line.direction.normalized()));
}

GeometryFittedLine2D FitGeometryRobustSegmentLine(
    const QVector<GeometryProjectedPoint>& projected,
    int firstIndex,
    int lastIndex)
{
    GeometryFittedLine2D fallbackLine = FitGeometrySegmentLine(projected, firstIndex, lastIndex);
    if (!fallbackLine.valid || projected.isEmpty())
    {
        return fallbackLine;
    }

    const int begin = std::max(0, std::min(firstIndex, lastIndex));
    const int end = std::min(static_cast<int>(projected.size()) - 1, std::max(firstIndex, lastIndex));
    const int pointCount = end - begin + 1;
    if (pointCount < 14)
    {
        return fallbackLine;
    }

    const int edgeTrim = std::min(12, std::max(2, pointCount / 10));
    const int fitBegin = begin + edgeTrim;
    const int fitEnd = end - edgeTrim;
    const int fitCount = fitEnd - fitBegin + 1;
    if (fitCount < 8)
    {
        return fallbackLine;
    }

    Eigen::Vector2d segmentDirection =
        GeometrySmoothedProjection2D(projected[end]) - GeometrySmoothedProjection2D(projected[begin]);
    if (segmentDirection.norm() < 1e-6)
    {
        segmentDirection = fallbackLine.direction;
    }
    segmentDirection = OrientGeometryDirection(segmentDirection, fallbackLine.direction);
    if (segmentDirection.norm() < 1e-9)
    {
        return fallbackLine;
    }

    struct DirectionSample
    {
        Eigen::Vector2d direction = Eigen::Vector2d::Zero();
        double weight = 1.0;
        int begin = 0;
        int end = 0;
    };

    QVector<DirectionSample> samples;
    const int windowPoints = std::max(6, std::min(28, std::max(6, fitCount / 4)));
    const int stepPoints = std::max(3, windowPoints / 2);
    for (int windowBegin = fitBegin; windowBegin <= fitEnd - 4; windowBegin += stepPoints)
    {
        const int windowEnd = std::min(fitEnd, windowBegin + windowPoints - 1);
        if (windowEnd - windowBegin + 1 < 5)
        {
            continue;
        }

        const GeometryFittedLine2D windowLine = FitGeometrySegmentLine(projected, windowBegin, windowEnd);
        if (!windowLine.valid)
        {
            continue;
        }

        DirectionSample sample;
        sample.direction = OrientGeometryDirection(windowLine.direction, segmentDirection);
        sample.weight = std::max(1.0, std::abs(projected[windowEnd].s - projected[windowBegin].s));
        sample.begin = windowBegin;
        sample.end = windowEnd;
        samples.push_back(sample);
    }

    if (samples.size() < 2)
    {
        return fallbackLine;
    }

    auto averageDirection = [&](const QVector<DirectionSample>& directionSamples) -> Eigen::Vector2d {
        Eigen::Vector2d sum = Eigen::Vector2d::Zero();
        for (const DirectionSample& sample : directionSamples)
        {
            sum += OrientGeometryDirection(sample.direction, segmentDirection) * sample.weight;
        }
        if (sum.norm() < 1e-9)
        {
            return fallbackLine.direction;
        }
        return OrientGeometryDirection(sum, segmentDirection);
    };

    const double kPi = 3.14159265358979323846;
    Eigen::Vector2d robustDirection = averageDirection(samples);
    QVector<DirectionSample> keptSamples;
    const double angleLimit = 15.0 * kPi / 180.0;
    for (const DirectionSample& sample : samples)
    {
        if (GeometryLineDirectionAngle(sample.direction, robustDirection) <= angleLimit)
        {
            keptSamples.push_back(sample);
        }
    }
    if (keptSamples.size() >= 2)
    {
        robustDirection = averageDirection(keptSamples);
    }

    QVector<char> acceptedPoint(fitCount, 1);
    if (keptSamples.size() >= 2 && keptSamples.size() < samples.size())
    {
        acceptedPoint.fill(0);
        for (const DirectionSample& sample : keptSamples)
        {
            const int acceptedBegin = std::max(fitBegin, sample.begin);
            const int acceptedEnd = std::min(fitEnd, sample.end);
            for (int index = acceptedBegin; index <= acceptedEnd; ++index)
            {
                acceptedPoint[index - fitBegin] = 1;
            }
        }

        int acceptedCount = 0;
        for (char accepted : acceptedPoint)
        {
            if (accepted)
            {
                ++acceptedCount;
            }
        }
        if (acceptedCount < std::max(4, fitCount / 3))
        {
            acceptedPoint.fill(1);
        }
    }

    QVector<double> normalValues;
    normalValues.reserve(fitCount);
    for (int index = fitBegin; index <= fitEnd; ++index)
    {
        normalValues.push_back(projected[index].smoothN);
    }
    const double normalMedian = GeometryMedianScalar(normalValues);
    QVector<double> normalDeviations;
    normalDeviations.reserve(normalValues.size());
    for (double value : normalValues)
    {
        normalDeviations.push_back(std::abs(value - normalMedian));
    }
    const double normalMad = GeometryMedianScalar(normalDeviations);
    const double normalLimit = std::max(1.2, std::min(8.0, normalMad * 4.0 + 0.8));
    int normalBandSupport = 0;
    for (double value : normalValues)
    {
        if (std::abs(value - normalMedian) <= normalLimit)
        {
            ++normalBandSupport;
        }
    }
    if (normalBandSupport >= std::max(5, fitCount * 2 / 3))
    {
        int acceptedCount = 0;
        for (int index = fitBegin; index <= fitEnd; ++index)
        {
            if (std::abs(projected[index].smoothN - normalMedian) > normalLimit)
            {
                acceptedPoint[index - fitBegin] = 0;
            }
            if (acceptedPoint[index - fitBegin])
            {
                ++acceptedCount;
            }
        }
        if (acceptedCount < std::max(4, fitCount / 3))
        {
            acceptedPoint.fill(1);
        }
    }

    auto isAcceptedPoint = [&](int index) -> bool {
        if (index < fitBegin || index > fitEnd)
        {
            return false;
        }
        return acceptedPoint[index - fitBegin] != 0;
    };

    GeometryFittedLine2D robustLine;
    robustLine.valid = true;
    robustLine.direction = robustDirection;

    const double dirX = robustDirection.x();
    const double dirY = robustDirection.y();
    double normalX = -dirY;
    double normalY = dirX;
    const double normalLength = std::sqrt(normalX * normalX + normalY * normalY);
    if (normalLength > 1e-9)
    {
        normalX /= normalLength;
        normalY /= normalLength;
    }
    else
    {
        normalX = 0.0;
        normalY = 1.0;
    }

    QVector<double> axisValues;
    QVector<double> normalOffsets;
    axisValues.reserve(fitCount);
    normalOffsets.reserve(fitCount);
    for (int index = fitBegin; index <= fitEnd; ++index)
    {
        if (!isAcceptedPoint(index))
        {
            continue;
        }
        const double x = projected[index].s;
        const double y = projected[index].smoothH;
        axisValues.push_back(x * dirX + y * dirY);
        normalOffsets.push_back(x * normalX + y * normalY);
    }
    if (axisValues.isEmpty() || normalOffsets.isEmpty())
    {
        return fallbackLine;
    }
    const double medianAxisValue = GeometryMedianScalar(axisValues);
    const double medianNormalOffset = GeometryMedianScalar(normalOffsets);
    robustLine.point.x() = dirX * medianAxisValue + normalX * medianNormalOffset;
    robustLine.point.y() = dirY * medianAxisValue + normalY * medianNormalOffset;

    auto scalarDistanceToRobustLine = [&](const GeometryProjectedPoint& projectedPoint) -> double {
        const double deltaX = projectedPoint.s - robustLine.point.x();
        const double deltaY = projectedPoint.smoothH - robustLine.point.y();
        return std::abs(deltaX * dirY - deltaY * dirX);
    };

    QVector<double> distances;
    distances.reserve(fitCount);
    for (int index = fitBegin; index <= fitEnd; ++index)
    {
        if (!isAcceptedPoint(index))
        {
            continue;
        }
        distances.push_back(scalarDistanceToRobustLine(projected[index]));
    }
    const double medianDistance = GeometryMedianScalar(distances);
    const double distanceLimit = std::max(1.0, std::min(8.0, medianDistance * 3.0 + 0.5));

    QVector<double> inlierAxisValues;
    QVector<double> inlierNormalOffsets;
    inlierAxisValues.reserve(fitCount);
    inlierNormalOffsets.reserve(fitCount);
    for (int index = fitBegin; index <= fitEnd; ++index)
    {
        if (!isAcceptedPoint(index))
        {
            continue;
        }
        if (scalarDistanceToRobustLine(projected[index]) <= distanceLimit)
        {
            const double x = projected[index].s;
            const double y = projected[index].smoothH;
            inlierAxisValues.push_back(x * dirX + y * dirY);
            inlierNormalOffsets.push_back(x * normalX + y * normalY);
        }
    }
    if (inlierAxisValues.size() >= std::max(4, fitCount / 3))
    {
        const double inlierMedianAxisValue = GeometryMedianScalar(inlierAxisValues);
        const double inlierMedianNormalOffset = GeometryMedianScalar(inlierNormalOffsets);
        robustLine.point.x() = dirX * inlierMedianAxisValue + normalX * inlierMedianNormalOffset;
        robustLine.point.y() = dirY * inlierMedianAxisValue + normalY * inlierMedianNormalOffset;
    }

    return robustLine;
}

QVector<GeometryFittedLine2D> BuildRobustGeometrySegmentLines(
    const QVector<GeometryProjectedPoint>& projected,
    const QVector<int>& keyIndexes)
{
    QVector<GeometryFittedLine2D> segmentLines;
    if (keyIndexes.size() < 2)
    {
        return segmentLines;
    }

    segmentLines.reserve(keyIndexes.size() - 1);
    for (int index = 0; index + 1 < keyIndexes.size(); ++index)
    {
        segmentLines.push_back(FitGeometryRobustSegmentLine(projected, keyIndexes[index], keyIndexes[index + 1]));
    }
    return segmentLines;
}

GeometryFittedLine2D FitGeometryLocalSegmentLineWithSpan(
    const QVector<GeometryProjectedPoint>& projected,
    int firstIndex,
    int lastIndex,
    bool useLastSide,
    double localSpan)
{
    if (projected.isEmpty())
    {
        return GeometryFittedLine2D();
    }

    int begin = std::max(0, std::min(firstIndex, lastIndex));
    int end = std::min(static_cast<int>(projected.size()) - 1, std::max(firstIndex, lastIndex));
    if (end <= begin + 2)
    {
        return FitGeometrySegmentLine(projected, begin, end);
    }

    const double totalSpan = std::abs(projected[end].s - projected[begin].s);
    const double clampedLocalSpan = std::max(8.0, std::min(80.0, localSpan));
    if (totalSpan > clampedLocalSpan)
    {
        if (useLastSide)
        {
            while (begin + 4 < end && std::abs(projected[end].s - projected[begin].s) > clampedLocalSpan)
            {
                ++begin;
            }
        }
        else
        {
            while (begin + 4 < end && std::abs(projected[end].s - projected[begin].s) > clampedLocalSpan)
            {
                --end;
            }
        }
    }

    return FitGeometryRobustSegmentLine(projected, begin, end);
}

GeometryFittedScalarLine FitGeometrySegmentNormalLine(
    const QVector<GeometryProjectedPoint>& projected,
    int firstIndex,
    int lastIndex)
{
    GeometryFittedScalarLine line;
    if (projected.isEmpty())
    {
        return line;
    }

    const int begin = std::max(0, std::min(firstIndex, lastIndex));
    const int end = std::min(static_cast<int>(projected.size()) - 1, std::max(firstIndex, lastIndex));
    const int pointCount = end - begin + 1;
    if (pointCount < 2)
    {
        return line;
    }

    const int trim = pointCount >= 12 ? std::min(pointCount / 6, 8) : 0;
    const int fitBegin = begin + trim;
    const int fitEnd = end - trim;
    const int fitCount = fitEnd - fitBegin + 1;
    if (fitCount < 2)
    {
        return line;
    }

    QVector<double> normalValues;
    normalValues.reserve(fitCount);
    for (int index = fitBegin; index <= fitEnd; ++index)
    {
        normalValues.push_back(projected[index].smoothN);
    }

    const double normalMedian = GeometryMedianScalar(normalValues);
    QVector<double> deviations;
    deviations.reserve(normalValues.size());
    for (double value : normalValues)
    {
        deviations.push_back(std::abs(value - normalMedian));
    }
    const double normalMad = GeometryMedianScalar(deviations);
    const double normalLimit = std::max(1.2, std::min(8.0, normalMad * 4.0 + 0.8));

    QVector<int> acceptedIndexes;
    acceptedIndexes.reserve(fitCount);
    int normalBandSupport = 0;
    for (int index = fitBegin; index <= fitEnd; ++index)
    {
        if (std::abs(projected[index].smoothN - normalMedian) <= normalLimit)
        {
            ++normalBandSupport;
        }
    }

    const bool useNormalBand = normalBandSupport >= std::max(4, fitCount * 2 / 3);
    for (int index = fitBegin; index <= fitEnd; ++index)
    {
        if (!useNormalBand || std::abs(projected[index].smoothN - normalMedian) <= normalLimit)
        {
            acceptedIndexes.push_back(index);
        }
    }

    if (acceptedIndexes.size() < 2)
    {
        line.centerS = projected[(fitBegin + fitEnd) / 2].s;
        line.centerValue = normalMedian;
        line.slope = 0.0;
        line.valid = true;
        return line;
    }

    double meanS = 0.0;
    double meanN = 0.0;
    for (int index : acceptedIndexes)
    {
        meanS += projected[index].s;
        meanN += projected[index].smoothN;
    }
    meanS /= static_cast<double>(acceptedIndexes.size());
    meanN /= static_cast<double>(acceptedIndexes.size());

    double numerator = 0.0;
    double denominator = 0.0;
    for (int index : acceptedIndexes)
    {
        const double deltaS = projected[index].s - meanS;
        numerator += deltaS * (projected[index].smoothN - meanN);
        denominator += deltaS * deltaS;
    }

    line.centerS = meanS;
    line.centerValue = meanN;
    line.slope = denominator > 1e-9 ? numerator / denominator : 0.0;
    line.valid = true;
    return line;
}

GeometryFittedScalarLine FitGeometryLocalSegmentNormalLineWithSpan(
    const QVector<GeometryProjectedPoint>& projected,
    int firstIndex,
    int lastIndex,
    bool useLastSide,
    double localSpan)
{
    if (projected.isEmpty())
    {
        return GeometryFittedScalarLine();
    }

    int begin = std::max(0, std::min(firstIndex, lastIndex));
    int end = std::min(static_cast<int>(projected.size()) - 1, std::max(firstIndex, lastIndex));
    if (end <= begin + 2)
    {
        return FitGeometrySegmentNormalLine(projected, begin, end);
    }

    const double totalSpan = std::abs(projected[end].s - projected[begin].s);
    const double clampedLocalSpan = std::max(8.0, std::min(80.0, localSpan));
    if (totalSpan > clampedLocalSpan)
    {
        if (useLastSide)
        {
            while (begin + 4 < end && std::abs(projected[end].s - projected[begin].s) > clampedLocalSpan)
            {
                ++begin;
            }
        }
        else
        {
            while (begin + 4 < end && std::abs(projected[end].s - projected[begin].s) > clampedLocalSpan)
            {
                --end;
            }
        }
    }

    return FitGeometrySegmentNormalLine(projected, begin, end);
}

GeometryFittedLine2D FitGeometryLocalSegmentLine(
    const QVector<GeometryProjectedPoint>& projected,
    int firstIndex,
    int lastIndex,
    bool useLastSide)
{
    if (projected.isEmpty())
    {
        return GeometryFittedLine2D();
    }

    const int begin = std::max(0, std::min(firstIndex, lastIndex));
    const int end = std::min(static_cast<int>(projected.size()) - 1, std::max(firstIndex, lastIndex));
    const double totalSpan = begin < end ? std::abs(projected[end].s - projected[begin].s) : 0.0;
    return FitGeometryLocalSegmentLineWithSpan(
        projected,
        firstIndex,
        lastIndex,
        useLastSide,
        std::max(20.0, std::min(80.0, totalSpan * 0.6)));
}

bool IntersectGeometryLines(
    const GeometryFittedLine2D& first,
    const GeometryFittedLine2D& second,
    Eigen::Vector2d* intersection)
{
    if (intersection == nullptr || !first.valid || !second.valid)
    {
        return false;
    }

    const double denominator = Cross2D(first.direction, second.direction);
    if (std::abs(denominator) < 1e-6)
    {
        return false;
    }

    const Eigen::Vector2d delta = second.point - first.point;
    const double ratio = Cross2D(delta, second.direction) / denominator;
    *intersection = first.point + first.direction * ratio;
    return std::isfinite(intersection->x()) && std::isfinite(intersection->y());
}

bool IsGeometryCornerProjectionUsable(
    const QVector<GeometryProjectedPoint>& projected,
    const QVector<int>& keyIndexes,
    int keyPosition,
    const Eigen::Vector2d& candidate,
    const Eigen::Vector2d& reference)
{
    if (keyPosition <= 0 || keyPosition + 1 >= keyIndexes.size())
    {
        return true;
    }
    if (!std::isfinite(candidate.x()) || !std::isfinite(candidate.y()))
    {
        return false;
    }

    const double previousS = projected[keyIndexes[keyPosition - 1]].s;
    const double nextS = projected[keyIndexes[keyPosition + 1]].s;
    const double minS = std::min(previousS, nextS);
    const double maxS = std::max(previousS, nextS);
    const double span = std::max(1.0, maxS - minS);
    const double margin = std::max(5.0, std::min(14.0, span * 0.12));
    if (candidate.x() < minS - margin || candidate.x() > maxS + margin)
    {
        return false;
    }

    const double maxCornerShift = std::max(5.0, std::min(18.0, span * 0.14));
    if ((candidate - reference).norm() > maxCornerShift)
    {
        return false;
    }

    const int begin = keyIndexes[keyPosition - 1];
    const int end = keyIndexes[keyPosition + 1];
    const double maxCloudDistance = std::max(2.5, std::min(6.0, span * 0.05));
    if (NearestGeometryProjectionDistance(projected, candidate, begin, end) > maxCloudDistance)
    {
        return false;
    }

    double mainBandH = 0.0;
    double mainBandThreshold = 0.0;
    const double stationRadius = std::max(6.0, std::min(36.0, span * 0.08));
    const double baseThreshold = std::max(1.6, std::min(5.5, span * 0.015 + 0.8));
    if (GeometryLocalMainBand(
        projected,
        candidate.x(),
        stationRadius,
        baseThreshold,
        &mainBandH,
        &mainBandThreshold,
        nullptr))
    {
        const double mainBandDelta = std::abs(candidate.y() - mainBandH);
        if (mainBandDelta >= mainBandThreshold * 1.35
            && mainBandDelta >= baseThreshold * 1.6)
        {
            return false;
        }
    }

    return true;
}

QVector<Eigen::Vector3d> BuildFittedGeometryKeyPoints(
    const QVector<GeometryProjectedPoint>& projected,
    const QVector<int>& keyIndexes,
    const Eigen::Vector3d& center,
    const Eigen::Vector3d& mainAxis,
    const Eigen::Vector3d& sideAxis,
    const Eigen::Vector3d& normalAxis)
{
    QVector<Eigen::Vector3d> keyPoints;
    keyPoints.reserve(keyIndexes.size());
    if (keyIndexes.isEmpty())
    {
        return keyPoints;
    }

    const QVector<GeometryFittedLine2D> segmentLines = BuildRobustGeometrySegmentLines(projected, keyIndexes);

    for (int index = 0; index < keyIndexes.size(); ++index)
    {
        const int keyIndex = keyIndexes[index];
        Eigen::Vector2d projection = (index == 0 || index + 1 == keyIndexes.size())
            ? GeometryProjection2D(projected[keyIndex])
            : GeometrySmoothedProjection2D(projected[keyIndex]);
        double normalValue = (index == 0 || index + 1 == keyIndexes.size())
            ? projected[keyIndex].n
            : projected[keyIndex].smoothN;
        if (index > 0 && index + 1 < keyIndexes.size())
        {
            const double previousS = projected[keyIndexes[index - 1]].s;
            const double nextS = projected[keyIndexes[index + 1]].s;
            const double span = std::max(1.0, std::abs(nextS - previousS));
            bool hasBestIntersection = false;
            double bestScore = std::numeric_limits<double>::infinity();
            Eigen::Vector2d bestIntersection = projection;
            auto considerIntersection = [&](const Eigen::Vector2d& candidate, double shiftWeight)
            {
                if (!IsGeometryCornerProjectionUsable(projected, keyIndexes, index, candidate, projection))
                {
                    return;
                }

                const double cloudDistance = NearestGeometryProjectionDistance(
                    projected,
                    candidate,
                    keyIndexes[index - 1],
                    keyIndexes[index + 1]);
                const double score = cloudDistance + (candidate - projection).norm() * shiftWeight;
                if (score < bestScore)
                {
                    bestScore = score;
                    bestIntersection = candidate;
                    hasBestIntersection = true;
                }
            };

            if (index - 1 < segmentLines.size() && index < segmentLines.size())
            {
                Eigen::Vector2d intersection = projection;
                if (IntersectGeometryLines(segmentLines[index - 1], segmentLines[index], &intersection))
                {
                    considerIntersection(intersection, 0.22);
                }
            }

            const double candidateSpans[] = {
                std::max(10.0, span * 0.18),
                std::max(14.0, span * 0.28),
                std::max(20.0, span * 0.40),
                std::max(28.0, span * 0.55),
                std::max(36.0, std::min(80.0, span * 0.70))
            };

            for (double localSpan : candidateSpans)
            {
                const GeometryFittedLine2D leftLine = FitGeometryLocalSegmentLineWithSpan(
                    projected,
                    keyIndexes[index - 1],
                    keyIndex,
                    true,
                    localSpan);
                const GeometryFittedLine2D rightLine = FitGeometryLocalSegmentLineWithSpan(
                    projected,
                    keyIndex,
                    keyIndexes[index + 1],
                    false,
                    localSpan);
                Eigen::Vector2d intersection = projection;
                if (IntersectGeometryLines(leftLine, rightLine, &intersection))
                {
                    considerIntersection(intersection, 0.15);
                }
            }

            if (hasBestIntersection)
            {
                projection = bestIntersection;
            }

            const double stationRadius = std::max(6.0, std::min(36.0, span * 0.08));
            const double baseThreshold = std::max(1.6, std::min(5.5, span * 0.015 + 0.8));
            const double localNormalValue = GeometryLocalNormalValue(
                projected,
                keyIndex,
                projection.x(),
                stationRadius,
                baseThreshold);
            QVector<double> normalCandidates;
            normalCandidates.reserve(12);
            for (double localSpan : candidateSpans)
            {
                const GeometryFittedScalarLine leftNormalLine = FitGeometryLocalSegmentNormalLineWithSpan(
                    projected,
                    keyIndexes[index - 1],
                    keyIndex,
                    true,
                    localSpan);
                const GeometryFittedScalarLine rightNormalLine = FitGeometryLocalSegmentNormalLineWithSpan(
                    projected,
                    keyIndex,
                    keyIndexes[index + 1],
                    false,
                    localSpan);
                if (leftNormalLine.valid)
                {
                    const double value = leftNormalLine.valueAt(projection.x());
                    if (std::isfinite(value))
                    {
                        normalCandidates.push_back(value);
                    }
                }
                if (rightNormalLine.valid)
                {
                    const double value = rightNormalLine.valueAt(projection.x());
                    if (std::isfinite(value))
                    {
                        normalCandidates.push_back(value);
                    }
                }
            }

            if (normalCandidates.size() >= 2)
            {
                const double segmentNormalMedian = GeometryMedianScalar(normalCandidates);
                const double localDelta = std::abs(localNormalValue - segmentNormalMedian);
                if (localDelta <= std::max(1.8, baseThreshold * 1.5))
                {
                    normalCandidates.push_back(localNormalValue);
                }
                normalValue = GeometryMedianScalar(normalCandidates);
            }
            else
            {
                normalValue = localNormalValue;
            }
        }

        keyPoints.push_back(GeometryPointFromProjection(
            center,
            mainAxis,
            sideAxis,
            normalAxis,
            projection,
            normalValue));
    }

    return keyPoints;
}

QVector<int> RefineGeometryKeysBySegmentDeviation(
    const QVector<GeometryProjectedPoint>& projected,
    QVector<int> keyIndexes,
    const Eigen::Vector3d& center,
    const Eigen::Vector3d& mainAxis,
    const Eigen::Vector3d& sideAxis,
    const RobotCalculation::LowerWeldFilterParams& params)
{
    if (projected.size() < 4 || keyIndexes.size() < 2)
    {
        return keyIndexes;
    }

    const double tolerance = params.piecewiseFitTolerance > 0.0 ? params.piecewiseFitTolerance : 2.0;
    const double maxSegmentDeviation = std::max(2.5, std::min(6.0, tolerance * 2.0));
    const int projectedCount = static_cast<int>(projected.size());
    const int maxKeyCount = std::min(96, std::max(16, projectedCount / 6));

    for (int iteration = 0; iteration < 64 && keyIndexes.size() < maxKeyCount; ++iteration)
    {
        bool inserted = false;
        for (int segmentIndex = 0; segmentIndex + 1 < keyIndexes.size(); ++segmentIndex)
        {
            const int begin = std::min(keyIndexes[segmentIndex], keyIndexes[segmentIndex + 1]);
            const int end = std::max(keyIndexes[segmentIndex], keyIndexes[segmentIndex + 1]);
            if (end <= begin + 4)
            {
                continue;
            }

            // Segment refinement must be driven by the measured smoothed path,
            // not by already-intersected key points.  Otherwise one bad local
            // intersection can pull the next refinement round away from the
            // actual point cloud and create the visible crossed/raised lines.
            const Eigen::Vector2d firstProjection = GeometrySmoothedProjection2D(projected[begin]);
            const Eigen::Vector2d secondProjection = GeometrySmoothedProjection2D(projected[end]);
            const int edgeGap = std::max(2, std::min(8, (end - begin) / 10));
            const double normalMedian = GeometryMedianSmoothNInRange(projected, begin, end);
            const double normalMad = GeometryMadSmoothNInRange(projected, begin, end, normalMedian);
            const double normalLimit = std::max(1.2, std::min(8.0, normalMad * 4.0 + 0.8));
            int normalSupport = 0;
            for (int sampleIndex = begin; sampleIndex <= end; ++sampleIndex)
            {
                if (std::abs(projected[sampleIndex].smoothN - normalMedian) <= normalLimit)
                {
                    ++normalSupport;
                }
            }
            const bool useNormalGate = normalSupport >= std::max(6, (end - begin + 1) * 2 / 3);

            double maxDistance = 0.0;
            int maxDistanceIndex = -1;
            for (int sampleIndex = begin + edgeGap; sampleIndex <= end - edgeGap; ++sampleIndex)
            {
                if (useNormalGate
                    && std::abs(projected[sampleIndex].smoothN - normalMedian) > normalLimit)
                {
                    continue;
                }

                const double distance = GeometryPointToSegmentDistance2D(
                    GeometrySmoothedProjection2D(projected[sampleIndex]),
                    firstProjection,
                    secondProjection);
                if (distance > maxDistance)
                {
                    maxDistance = distance;
                    maxDistanceIndex = sampleIndex;
                }
            }

            if (maxDistanceIndex > begin
                && maxDistanceIndex < end
                && maxDistance > maxSegmentDeviation)
            {
                keyIndexes.insert(segmentIndex + 1, maxDistanceIndex);
                inserted = true;
                break;
            }
        }

        if (!inserted)
        {
            break;
        }
    }

    return keyIndexes;
}

QVector<int> PruneNonMonotonicFittedGeometryKeys(
    const QVector<GeometryProjectedPoint>& projected,
    QVector<int> keyIndexes,
    const Eigen::Vector3d& center,
    const Eigen::Vector3d& mainAxis,
    const Eigen::Vector3d& sideAxis,
    const Eigen::Vector3d& normalAxis)
{
    if (keyIndexes.size() <= 3)
    {
        return keyIndexes;
    }

    bool removed = true;
    while (removed && keyIndexes.size() > 3)
    {
        removed = false;
        const QVector<Eigen::Vector3d> fittedKeyPoints = BuildFittedGeometryKeyPoints(
            projected,
            keyIndexes,
            center,
            mainAxis,
            sideAxis,
            normalAxis);
        if (fittedKeyPoints.size() != keyIndexes.size())
        {
            return keyIndexes;
        }

        double previousS = (fittedKeyPoints.first() - center).dot(mainAxis);
        for (int index = 1; index < fittedKeyPoints.size(); ++index)
        {
            const double currentS = (fittedKeyPoints[index] - center).dot(mainAxis);
            if (currentS > previousS + 1e-6)
            {
                previousS = currentS;
                continue;
            }

            int removeIndex = index;
            if (removeIndex >= keyIndexes.size() - 1)
            {
                removeIndex = keyIndexes.size() - 2;
            }
            if (removeIndex <= 0)
            {
                removeIndex = 1;
            }

            keyIndexes.removeAt(removeIndex);
            removed = true;
            break;
        }
    }

    return keyIndexes;
}

RobotCalculation::LowerWeldPointType GeometryCornerType(
    const QVector<GeometryProjectedPoint>& projected,
    const QVector<int>& keyIndexes,
    int keyPosition)
{
    using PointType = RobotCalculation::LowerWeldPointType;
    if (keyPosition <= 0)
    {
        return PointType::Start;
    }
    if (keyPosition >= keyIndexes.size() - 1)
    {
        return PointType::End;
    }

    const double previous = projected[keyIndexes[keyPosition - 1]].smoothH;
    const double current = projected[keyIndexes[keyPosition]].smoothH;
    const double next = projected[keyIndexes[keyPosition + 1]].smoothH;
    const double prominence = current - (previous + next) * 0.5;
    return prominence >= 0.0 ? PointType::InnerCorner : PointType::OuterCorner;
}

double GeometryCornerProminence(
    const QVector<GeometryProjectedPoint>& projected,
    const QVector<int>& keyIndexes,
    int keyPosition)
{
    if (keyPosition <= 0 || keyPosition >= keyIndexes.size() - 1)
    {
        return std::numeric_limits<double>::infinity();
    }

    const double previous = projected[keyIndexes[keyPosition - 1]].smoothH;
    const double current = projected[keyIndexes[keyPosition]].smoothH;
    const double next = projected[keyIndexes[keyPosition + 1]].smoothH;
    return current - (previous + next) * 0.5;
}

QVector<int> MergeAdjacentGeometryCorners(
    const QVector<GeometryProjectedPoint>& projected,
    QVector<int> keyIndexes)
{
    if (keyIndexes.size() <= 3)
    {
        return keyIndexes;
    }

    bool removed = true;
    while (removed && keyIndexes.size() > 3)
    {
        removed = false;
        for (int index = 1; index + 1 < keyIndexes.size() - 1; ++index)
        {
            const RobotCalculation::LowerWeldPointType currentType =
                GeometryCornerType(projected, keyIndexes, index);
            const RobotCalculation::LowerWeldPointType nextType =
                GeometryCornerType(projected, keyIndexes, index + 1);
            if (currentType != nextType)
            {
                continue;
            }

            const double currentProminence = std::abs(GeometryCornerProminence(projected, keyIndexes, index));
            const double nextProminence = std::abs(GeometryCornerProminence(projected, keyIndexes, index + 1));
            keyIndexes.removeAt(currentProminence < nextProminence ? index : index + 1);
            removed = true;
            break;
        }
    }

    return keyIndexes;
}

void CountLowerWeldClassifiedPoints(RobotCalculation::LowerWeldClassificationResult& result)
{
    result.startCount = 0;
    result.endCount = 0;
    result.innerCornerCount = 0;
    result.outerCornerCount = 0;
    result.normalCount = 0;
    result.noiseCount = 0;
    for (const RobotCalculation::LowerWeldClassifiedPoint& point : result.points)
    {
        switch (point.type)
        {
        case RobotCalculation::LowerWeldPointType::Start:
            ++result.startCount;
            break;
        case RobotCalculation::LowerWeldPointType::End:
            ++result.endCount;
            break;
        case RobotCalculation::LowerWeldPointType::InnerCorner:
            ++result.innerCornerCount;
            break;
        case RobotCalculation::LowerWeldPointType::OuterCorner:
            ++result.outerCornerCount;
            break;
        case RobotCalculation::LowerWeldPointType::Noise:
            ++result.noiseCount;
            break;
        case RobotCalculation::LowerWeldPointType::Normal:
        default:
            ++result.normalCount;
            break;
        }
    }
}

double MedianValue(QVector<double> values)
{
    if (values.isEmpty())
    {
        return 0.0;
    }

    std::sort(values.begin(), values.end());
    const int middle = values.size() / 2;
    if (values.size() % 2 == 1)
    {
        return values[middle];
    }
    return (values[middle - 1] + values[middle]) * 0.5;
}

double ValueRange(const QVector<double>& values)
{
    if (values.isEmpty())
    {
        return 0.0;
    }

    const auto minmax = std::minmax_element(values.begin(), values.end());
    return *minmax.second - *minmax.first;
}

bool IsFinitePoint(const Eigen::Vector3d& point)
{
    return std::isfinite(point.x()) && std::isfinite(point.y()) && std::isfinite(point.z());
}

Eigen::Vector3d MedianGeometryPoint(
    const QVector<RobotCalculation::IndexedPoint3D>& points,
    int begin,
    int end,
    int skipIndex)
{
    QVector<double> xs;
    QVector<double> ys;
    QVector<double> zs;
    xs.reserve(std::max(0, end - begin));
    ys.reserve(std::max(0, end - begin));
    zs.reserve(std::max(0, end - begin));
    for (int index = begin; index <= end; ++index)
    {
        if (index == skipIndex)
        {
            continue;
        }

        xs.push_back(points[index].point.x());
        ys.push_back(points[index].point.y());
        zs.push_back(points[index].point.z());
    }

    return Eigen::Vector3d(
        MedianValue(xs),
        MedianValue(ys),
        MedianValue(zs));
}

QVector<RobotCalculation::IndexedPoint3D> RemoveGeometryLocalOutliers(
    const QVector<RobotCalculation::IndexedPoint3D>& points,
    const RobotCalculation::LowerWeldFilterParams& params,
    int* rejectedCount)
{
    if (rejectedCount != nullptr)
    {
        *rejectedCount = 0;
    }
    if (points.size() < 9)
    {
        return points;
    }

    const int radius = std::max(4, std::min(10, params.smoothRadius + 3));
    const double configuredThreshold = params.zContinuityThreshold > 0.0
        ? params.zContinuityThreshold * 1.25
        : 2.5;
    const double baseThreshold = std::max(2.0, std::min(6.0, configuredThreshold));

    QVector<RobotCalculation::IndexedPoint3D> current = points;
    int totalRejected = 0;
    for (int pass = 0; pass < 2; ++pass)
    {
        QVector<char> keep(current.size(), 1);
        int passRejected = 0;
        for (int index = 0; index < current.size(); ++index)
        {
            const int begin = std::max(0, index - radius);
            const int end = std::min(static_cast<int>(current.size()) - 1, index + radius);
            if (end - begin < 4)
            {
                continue;
            }

            const Eigen::Vector3d localMedian = MedianGeometryPoint(current, begin, end, index);
            QVector<double> localDistances;
            localDistances.reserve(end - begin);
            for (int sampleIndex = begin; sampleIndex <= end; ++sampleIndex)
            {
                if (sampleIndex == index)
                {
                    continue;
                }

                localDistances.push_back((current[sampleIndex].point - localMedian).norm());
            }

            const double localMedianDistance = MedianValue(localDistances);
            const double dynamicThreshold = std::max(
                baseThreshold,
                std::min(8.0, localMedianDistance * 6.0 + 0.5));
            const double currentDistance = (current[index].point - localMedian).norm();
            if (currentDistance > dynamicThreshold)
            {
                keep[index] = 0;
                ++passRejected;
            }
        }

        if (passRejected <= 0)
        {
            break;
        }

        QVector<RobotCalculation::IndexedPoint3D> filtered;
        filtered.reserve(current.size() - passRejected);
        for (int index = 0; index < current.size(); ++index)
        {
            if (keep[index])
            {
                filtered.push_back(current[index]);
            }
        }

        current = filtered;
        totalRejected += passRejected;
        if (current.size() < 9)
        {
            break;
        }
    }

    if (rejectedCount != nullptr)
    {
        *rejectedCount = totalRejected;
    }
    return current;
}

QPair<double, double> LinearFit1D(const QVector<double>& xs, const QVector<double>& ys)
{
    if (xs.isEmpty() || ys.isEmpty() || xs.size() != ys.size())
    {
        return qMakePair(0.0, 0.0);
    }

    const int count = xs.size();
    double sumX = 0.0;
    double sumY = 0.0;
    double sumXX = 0.0;
    double sumXY = 0.0;
    for (int index = 0; index < count; ++index)
    {
        sumX += xs[index];
        sumY += ys[index];
        sumXX += xs[index] * xs[index];
        sumXY += xs[index] * ys[index];
    }

    const double denominator = static_cast<double>(count) * sumXX - sumX * sumX;
    if (std::abs(denominator) <= std::numeric_limits<double>::epsilon())
    {
        return qMakePair(0.0, sumY / static_cast<double>(count));
    }

    const double slope = (static_cast<double>(count) * sumXY - sumX * sumY) / denominator;
    const double intercept = (sumY - slope * sumX) / static_cast<double>(count);
    return qMakePair(slope, intercept);
}

struct RawFilterPoint
{
    double sampleAxisValue = 0.0;
    Eigen::Vector3d point = Eigen::Vector3d::Zero();
    bool valid = false;
    QString source;
};

struct LinearFitPrefixSums
{
    QVector<double> sumX;
    QVector<double> sumY;
    QVector<double> sumXX;
    QVector<double> sumXY;
    QVector<double> sumYY;
};

struct LinearFitSegment
{
    double slope = 0.0;
    double intercept = 0.0;
    double error = std::numeric_limits<double>::infinity();
    bool valid = false;
};

struct ThreeSegmentLineFit
{
    int firstEnd = -1;
    int secondEnd = -1;
    LinearFitSegment firstPrimary;
    LinearFitSegment secondPrimary;
    LinearFitSegment thirdPrimary;
    LinearFitSegment firstSecondary;
    LinearFitSegment secondSecondary;
    LinearFitSegment thirdSecondary;
    double error = std::numeric_limits<double>::infinity();
    bool valid = false;
};

struct PiecewiseLineModel
{
    int begin = 0;
    int end = 0;
    LinearFitSegment primary;
    LinearFitSegment secondary;
};

int TrapezoidSlopeClass(double slope, double horizontalThreshold)
{
    if (std::abs(slope) <= horizontalThreshold)
    {
        return 0;
    }
    return slope > 0.0 ? 1 : -1;
}

Eigen::Vector3d EvaluateLineModelPoint(
    RobotCalculation::SampleAxis axis,
    double axisValue,
    const LinearFitSegment& primary,
    const LinearFitSegment& secondary)
{
    Eigen::Vector3d point = Eigen::Vector3d::Zero();
    if (axis == RobotCalculation::SampleAxis::AxisY)
    {
        point.x() = primary.slope * axisValue + primary.intercept;
        point.y() = axisValue;
        point.z() = secondary.slope * axisValue + secondary.intercept;
    }
    else
    {
        point.x() = axisValue;
        point.y() = primary.slope * axisValue + primary.intercept;
        point.z() = secondary.slope * axisValue + secondary.intercept;
    }
    return point;
}

LinearFitPrefixSums BuildLinearFitPrefixSums(const QVector<double>& xs, const QVector<double>& ys)
{
    LinearFitPrefixSums sums;
    const int count = xs.size();
    sums.sumX.resize(count + 1);
    sums.sumY.resize(count + 1);
    sums.sumXX.resize(count + 1);
    sums.sumXY.resize(count + 1);
    sums.sumYY.resize(count + 1);

    sums.sumX[0] = 0.0;
    sums.sumY[0] = 0.0;
    sums.sumXX[0] = 0.0;
    sums.sumXY[0] = 0.0;
    sums.sumYY[0] = 0.0;

    for (int index = 0; index < count; ++index)
    {
        sums.sumX[index + 1] = sums.sumX[index] + xs[index];
        sums.sumY[index + 1] = sums.sumY[index] + ys[index];
        sums.sumXX[index + 1] = sums.sumXX[index] + xs[index] * xs[index];
        sums.sumXY[index + 1] = sums.sumXY[index] + xs[index] * ys[index];
        sums.sumYY[index + 1] = sums.sumYY[index] + ys[index] * ys[index];
    }

    return sums;
}

LinearFitSegment FitLinearRange(const LinearFitPrefixSums& sums, int begin, int end)
{
    LinearFitSegment result;
    if (begin < 0 || end < begin || end + 1 >= sums.sumX.size())
    {
        return result;
    }

    const int count = end - begin + 1;
    if (count <= 0)
    {
        return result;
    }

    const double sumX = sums.sumX[end + 1] - sums.sumX[begin];
    const double sumY = sums.sumY[end + 1] - sums.sumY[begin];
    const double sumXX = sums.sumXX[end + 1] - sums.sumXX[begin];
    const double sumXY = sums.sumXY[end + 1] - sums.sumXY[begin];
    const double sumYY = sums.sumYY[end + 1] - sums.sumYY[begin];

    const double countValue = static_cast<double>(count);
    const double denominator = countValue * sumXX - sumX * sumX;
    if (std::abs(denominator) <= std::numeric_limits<double>::epsilon())
    {
        result.slope = 0.0;
        result.intercept = sumY / countValue;
    }
    else
    {
        result.slope = (countValue * sumXY - sumX * sumY) / denominator;
        result.intercept = (sumY - result.slope * sumX) / countValue;
    }

    result.error = sumYY
        + result.slope * result.slope * sumXX
        + countValue * result.intercept * result.intercept
        + 2.0 * result.slope * result.intercept * sumX
        - 2.0 * result.slope * sumXY
        - 2.0 * result.intercept * sumY;
    if (result.error < 0.0 && std::abs(result.error) < 1e-9)
    {
        result.error = 0.0;
    }
    result.valid = true;
    return result;
}

ThreeSegmentLineFit FitThreeLineSegments(
    const QVector<double>& xs,
    const QVector<double>& primaryTargets,
    const QVector<double>& secondaryTargets)
{
    ThreeSegmentLineFit bestFit;
    if (xs.size() != primaryTargets.size() || xs.size() != secondaryTargets.size())
    {
        return bestFit;
    }

    const int pointCount = xs.size();
    constexpr int kMinSegmentPointCount = 3;
    if (pointCount < kMinSegmentPointCount * 3)
    {
        return bestFit;
    }

    const LinearFitPrefixSums primarySums = BuildLinearFitPrefixSums(xs, primaryTargets);
    const LinearFitPrefixSums secondarySums = BuildLinearFitPrefixSums(xs, secondaryTargets);

    for (int firstEnd = kMinSegmentPointCount - 1;
         firstEnd <= pointCount - kMinSegmentPointCount * 2 - 1;
         ++firstEnd)
    {
        for (int secondEnd = firstEnd + kMinSegmentPointCount;
             secondEnd <= pointCount - kMinSegmentPointCount - 1;
             ++secondEnd)
        {
            const LinearFitSegment firstPrimary = FitLinearRange(primarySums, 0, firstEnd);
            const LinearFitSegment secondPrimary = FitLinearRange(primarySums, firstEnd + 1, secondEnd);
            const LinearFitSegment thirdPrimary = FitLinearRange(primarySums, secondEnd + 1, pointCount - 1);
            const LinearFitSegment firstSecondary = FitLinearRange(secondarySums, 0, firstEnd);
            const LinearFitSegment secondSecondary = FitLinearRange(secondarySums, firstEnd + 1, secondEnd);
            const LinearFitSegment thirdSecondary = FitLinearRange(secondarySums, secondEnd + 1, pointCount - 1);

            if (!firstPrimary.valid || !secondPrimary.valid || !thirdPrimary.valid ||
                !firstSecondary.valid || !secondSecondary.valid || !thirdSecondary.valid)
            {
                continue;
            }

            const double totalError =
                firstPrimary.error + secondPrimary.error + thirdPrimary.error +
                firstSecondary.error + secondSecondary.error + thirdSecondary.error;
            if (totalError < bestFit.error)
            {
                bestFit.firstEnd = firstEnd;
                bestFit.secondEnd = secondEnd;
                bestFit.firstPrimary = firstPrimary;
                bestFit.secondPrimary = secondPrimary;
                bestFit.thirdPrimary = thirdPrimary;
                bestFit.firstSecondary = firstSecondary;
                bestFit.secondSecondary = secondSecondary;
                bestFit.thirdSecondary = thirdSecondary;
                bestFit.error = totalError;
                bestFit.valid = true;
            }
        }
    }

    return bestFit;
}

double PointToLineSegmentDistance(const Eigen::Vector3d& point,
    const Eigen::Vector3d& segmentBegin,
    const Eigen::Vector3d& segmentEnd)
{
    const Eigen::Vector3d segment = segmentEnd - segmentBegin;
    const double segmentNormSquared = segment.squaredNorm();
    if (segmentNormSquared <= std::numeric_limits<double>::epsilon())
    {
        return (point - segmentBegin).norm();
    }

    const double projectionRatio =
        std::clamp((point - segmentBegin).dot(segment) / segmentNormSquared, 0.0, 1.0);
    const Eigen::Vector3d projectionPoint = segmentBegin + projectionRatio * segment;
    return (point - projectionPoint).norm();
}

void CollectPiecewiseBreakpoints(const QVector<Eigen::Vector3d>& points,
    int begin,
    int end,
    double tolerance,
    QVector<int>& breakpoints)
{
    if (end - begin < 2)
    {
        return;
    }

    double maxDistance = -1.0;
    int splitIndex = -1;
    for (int index = begin + 1; index < end; ++index)
    {
        const double distance =
            PointToLineSegmentDistance(points[index], points[begin], points[end]);
        if (distance > maxDistance)
        {
            maxDistance = distance;
            splitIndex = index;
        }
    }

    if (splitIndex > begin && splitIndex < end && maxDistance > tolerance)
    {
        CollectPiecewiseBreakpoints(points, begin, splitIndex, tolerance, breakpoints);
        breakpoints.push_back(splitIndex);
        CollectPiecewiseBreakpoints(points, splitIndex, end, tolerance, breakpoints);
    }
}

QVector<int> BuildPiecewiseBreakpoints(const QVector<Eigen::Vector3d>& points,
    double tolerance,
    int minSegmentPoints)
{
    QVector<int> breakpoints;
    if (points.size() < 2)
    {
        return breakpoints;
    }

    breakpoints.push_back(0);
    CollectPiecewiseBreakpoints(points, 0, points.size() - 1, tolerance, breakpoints);
    breakpoints.push_back(points.size() - 1);

    std::sort(breakpoints.begin(), breakpoints.end());
    breakpoints.erase(std::unique(breakpoints.begin(), breakpoints.end()), breakpoints.end());

    const int minimumPoints = std::max(2, minSegmentPoints);
    bool changed = true;
    while (changed && breakpoints.size() > 2)
    {
        changed = false;
        for (int index = 1; index < breakpoints.size() - 1; ++index)
        {
            const int leftCount = breakpoints[index] - breakpoints[index - 1] + 1;
            const int rightCount = breakpoints[index + 1] - breakpoints[index] + 1;
            if (leftCount < minimumPoints || rightCount < minimumPoints)
            {
                breakpoints.removeAt(index);
                changed = true;
                break;
            }
        }
    }

    return breakpoints;
}
}

T_ROBOT_COORS RobotCalculation::InterpolateRobotPose(const std::vector<TimestampedRobotPose>& robotSamples, qint64 targetTimestampUs)
{
    if (robotSamples.empty())
    {
        return T_ROBOT_COORS();
    }

    if (targetTimestampUs <= robotSamples.front().timestampUs)
    {
        return robotSamples.front().pose;
    }
    if (targetTimestampUs >= robotSamples.back().timestampUs)
    {
        return robotSamples.back().pose;
    }

    const auto upper = std::lower_bound(robotSamples.begin(), robotSamples.end(), targetTimestampUs,
        [](const TimestampedRobotPose& sample, qint64 timestamp)
        {
            return sample.timestampUs < timestamp;
        });

    if (upper == robotSamples.begin())
    {
        return upper->pose;
    }

    const auto lower = upper - 1;
    const qint64 dt = upper->timestampUs - lower->timestampUs;
    const double ratio = dt > 0 ? static_cast<double>(targetTimestampUs - lower->timestampUs) / static_cast<double>(dt) : 0.0;

    auto lerp = [ratio](double a, double b)
        {
            return a + (b - a) * ratio;
        };

    return T_ROBOT_COORS(
        lerp(lower->pose.dX, upper->pose.dX),
        lerp(lower->pose.dY, upper->pose.dY),
        lerp(lower->pose.dZ, upper->pose.dZ),
        lerp(lower->pose.dRX, upper->pose.dRX),
        lerp(lower->pose.dRY, upper->pose.dRY),
        lerp(lower->pose.dRZ, upper->pose.dRZ),
        lerp(lower->pose.dBX, upper->pose.dBX),
        lerp(lower->pose.dBY, upper->pose.dBY),
        lerp(lower->pose.dBZ, upper->pose.dBZ));
}

Eigen::Vector3d RobotCalculation::CalcLaserPointInRobot(const T_ROBOT_COORS& robotPose,
    const Eigen::Vector3d& cameraPoint,
    const HandEyeMatrixConfig& calibration)
{
    const Eigen::Matrix3d robotRotation =
        RobotPoseTransform::RotationFromPose(robotPose, calibration.robotType);
    return robotRotation * (calibration.rotation * cameraPoint + calibration.translation)
        + Eigen::Vector3d(robotPose.dX, robotPose.dY, robotPose.dZ);
}

RobotCalculation::LowerWeldFilterResult RobotCalculation::FilterLowerWeldPath(
    const QVector<IndexedPoint3D>& inputPoints,
    const LowerWeldFilterParams& params)
{
    LowerWeldFilterResult result;
    result.inputPointCount = inputPoints.size();

    if (inputPoints.isEmpty())
    {
        result.error = "输入点为空，无法滤波。";
        return result;
    }
    if (params.sampleStep <= 0.0)
    {
        result.error = "采样步长必须大于 0。";
        return result;
    }
    if (params.searchWindow < 0.0)
    {
        result.error = "搜索窗口不能小于 0。";
        return result;
    }
    if (params.minPointCount <= 0)
    {
        result.error = "最小点数必须大于 0。";
        return result;
    }

    QVector<IndexedPoint3D> lowerPoints;
    lowerPoints.reserve(inputPoints.size());
    for (const IndexedPoint3D& sample : inputPoints)
    {
        if (!IsFinitePoint(sample.point))
        {
            continue;
        }
        if (sample.point.z() < params.zThreshold)
        {
            lowerPoints.push_back(sample);
        }
    }

    result.lowerPointCount = lowerPoints.size();
    if (lowerPoints.size() < params.minPointCount)
    {
        result.error = QString("满足 Z 阈值的下层点太少，仅 %1 个。").arg(lowerPoints.size());
        return result;
    }

    std::sort(lowerPoints.begin(), lowerPoints.end(), [&params](const IndexedPoint3D& left, const IndexedPoint3D& right)
        {
            return SampleAxisValue(left.point, params.sampleAxis) < SampleAxisValue(right.point, params.sampleAxis);
        });

    // 焊道理论上接近平面，先按主轴邻域剔除局部 Z 突变点，避免尖刺噪点进入后续重采样。
    if (params.zJumpThreshold > 0.0)
    {
        QVector<IndexedPoint3D> zJumpFilteredPoints;
        zJumpFilteredPoints.reserve(lowerPoints.size());
        for (int index = 0; index < lowerPoints.size(); ++index)
        {
            const double axisCenter = SampleAxisValue(lowerPoints[index].point, params.sampleAxis);
            QVector<double> neighborZs;
            for (int sampleIndex = 0; sampleIndex < lowerPoints.size(); ++sampleIndex)
            {
                if (sampleIndex == index)
                {
                    continue;
                }

                const double axisDelta = std::abs(
                    SampleAxisValue(lowerPoints[sampleIndex].point, params.sampleAxis) - axisCenter);
                if (axisDelta > params.searchWindow)
                {
                    continue;
                }

                neighborZs.push_back(lowerPoints[sampleIndex].point.z());
            }

            if (neighborZs.size() >= 2)
            {
                const double localMedianZ = MedianValue(neighborZs);
                if (std::abs(lowerPoints[index].point.z() - localMedianZ) > params.zJumpThreshold)
                {
                    ++result.zJumpRejectedCount;
                    continue;
                }
            }

            zJumpFilteredPoints.push_back(lowerPoints[index]);
        }

        lowerPoints = zJumpFilteredPoints;
    }

    result.lowerPointCount = lowerPoints.size();
    if (lowerPoints.size() < params.minPointCount)
    {
        result.error = QString("剔除 Z 突变点后可用下层点太少，仅 %1 个。").arg(lowerPoints.size());
        return result;
    }

    const double minAxis = SampleAxisValue(lowerPoints.front().point, params.sampleAxis);
    const double maxAxis = SampleAxisValue(lowerPoints.back().point, params.sampleAxis);
    const double startAxis = std::round(minAxis / params.sampleStep) * params.sampleStep;
    const double endAxis = std::round(maxAxis / params.sampleStep) * params.sampleStep;

    QVector<RawFilterPoint> rawPoints;
    for (double sampleAxis = startAxis; sampleAxis <= endAxis + 1e-9; sampleAxis += params.sampleStep)
    {
        QVector<double> xs;
        QVector<double> ys;
        QVector<double> zs;
        for (const IndexedPoint3D& sourcePoint : lowerPoints)
        {
            if (std::abs(SampleAxisValue(sourcePoint.point, params.sampleAxis) - sampleAxis) > params.searchWindow)
            {
                continue;
            }
            xs.push_back(sourcePoint.point.x());
            ys.push_back(sourcePoint.point.y());
            zs.push_back(sourcePoint.point.z());
        }

        RawFilterPoint rawPoint;
        rawPoint.sampleAxisValue = sampleAxis;
        if (xs.size() >= params.minPointCount)
        {
            rawPoint.point = Eigen::Vector3d(MedianValue(xs), MedianValue(ys), MedianValue(zs));
            rawPoint.point = SetSampleAxisValue(rawPoint.point, params.sampleAxis, sampleAxis);
            rawPoint.valid = true;
            rawPoint.source = "measured";
        }
        rawPoints.push_back(rawPoint);
    }

    // 再按采样后的局部连续性剔除离散抖动点：若当前点明显偏离前后邻点构成的局部直线，则认为不连续。
    if (params.zContinuityThreshold > 0.0)
    {
        QVector<bool> rejectFlags(rawPoints.size(), false);
        for (int index = 0; index < rawPoints.size(); ++index)
        {
            if (!rawPoints[index].valid || rawPoints[index].source != "measured")
            {
                continue;
            }

            int previous = index - 1;
            while (previous >= 0 &&
                (!rawPoints[previous].valid || rawPoints[previous].source != "measured"))
            {
                --previous;
            }

            int next = index + 1;
            while (next < rawPoints.size() &&
                (!rawPoints[next].valid || rawPoints[next].source != "measured"))
            {
                ++next;
            }

            if (previous < 0 || next >= rawPoints.size())
            {
                continue;
            }

            const double previousAxis = rawPoints[previous].sampleAxisValue;
            const double currentAxis = rawPoints[index].sampleAxisValue;
            const double nextAxis = rawPoints[next].sampleAxisValue;
            const double axisSpan = nextAxis - previousAxis;
            if (std::abs(axisSpan) <= std::numeric_limits<double>::epsilon())
            {
                continue;
            }

            const double ratio = (currentAxis - previousAxis) / axisSpan;
            const double expectedZ =
                rawPoints[previous].point.z() +
                (rawPoints[next].point.z() - rawPoints[previous].point.z()) * ratio;
            if (std::abs(rawPoints[index].point.z() - expectedZ) > params.zContinuityThreshold)
            {
                rejectFlags[index] = true;
            }
        }

        for (int index = 0; index < rawPoints.size(); ++index)
        {
            if (!rejectFlags[index])
            {
                continue;
            }

            rawPoints[index].valid = false;
            rawPoints[index].source.clear();
            ++result.zContinuityRejectedCount;
        }
    }

    int firstValidIndex = -1;
    int lastValidIndex = -1;
    for (int index = 0; index < rawPoints.size(); ++index)
    {
        if (!rawPoints[index].valid)
        {
            continue;
        }
        if (firstValidIndex < 0)
        {
            firstValidIndex = index;
        }
        lastValidIndex = index;
    }

    if (firstValidIndex < 0)
    {
        result.error = "未找到可用的下层焊道点。";
        return result;
    }

    for (int index = 0; index < rawPoints.size(); ++index)
    {
        if (rawPoints[index].valid)
        {
            continue;
        }

        int previous = index - 1;
        while (previous >= 0 && !rawPoints[previous].valid)
        {
            --previous;
        }

        int next = index + 1;
        while (next < rawPoints.size() && !rawPoints[next].valid)
        {
            ++next;
        }

        if (previous >= 0 && next < rawPoints.size())
        {
            const double previousAxis = rawPoints[previous].sampleAxisValue;
            const double nextAxis = rawPoints[next].sampleAxisValue;
            const double ratio = std::abs(nextAxis - previousAxis) <= std::numeric_limits<double>::epsilon()
                ? 0.0
                : (rawPoints[index].sampleAxisValue - previousAxis) / (nextAxis - previousAxis);
            rawPoints[index].point = rawPoints[previous].point + (rawPoints[next].point - rawPoints[previous].point) * ratio;
            rawPoints[index].point = SetSampleAxisValue(rawPoints[index].point, params.sampleAxis, rawPoints[index].sampleAxisValue);
            rawPoints[index].valid = true;
            rawPoints[index].source = "interpolated";
            continue;
        }

        if (previous >= 0)
        {
            rawPoints[index].point = SetSampleAxisValue(rawPoints[previous].point, params.sampleAxis, rawPoints[index].sampleAxisValue);
            rawPoints[index].valid = true;
            rawPoints[index].source = "extended";
            continue;
        }

        if (next < rawPoints.size())
        {
            rawPoints[index].point = SetSampleAxisValue(rawPoints[next].point, params.sampleAxis, rawPoints[index].sampleAxisValue);
            rawPoints[index].valid = true;
            rawPoints[index].source = "extended";
        }
    }

    const int smoothRadius = std::max(0, params.smoothRadius);
    result.points.reserve(rawPoints.size());
    for (int index = 0; index < rawPoints.size(); ++index)
    {
        if (!rawPoints[index].valid)
        {
            continue;
        }

        const int begin = std::max(0, index - smoothRadius);
        const int end = std::min(index + smoothRadius, static_cast<int>(rawPoints.size()) - 1);
        Eigen::Vector3d smoothedPoint = Eigen::Vector3d::Zero();
        int validCount = 0;
        for (int sampleIndex = begin; sampleIndex <= end; ++sampleIndex)
        {
            if (!rawPoints[sampleIndex].valid)
            {
                continue;
            }
            smoothedPoint += rawPoints[sampleIndex].point;
            ++validCount;
        }

        if (validCount <= 0)
        {
            continue;
        }

        smoothedPoint /= static_cast<double>(validCount);
        smoothedPoint = SetSampleAxisValue(smoothedPoint, params.sampleAxis, rawPoints[index].sampleAxisValue);

        LowerWeldFilterPoint outputPoint;
        outputPoint.index = result.points.size() + 1;
        outputPoint.point = smoothedPoint;
        outputPoint.source = rawPoints[index].source;
        result.points.push_back(outputPoint);

        if (outputPoint.source == "measured")
        {
            ++result.measuredCount;
        }
        else if (outputPoint.source == "interpolated")
        {
            ++result.interpolatedCount;
        }
        else if (outputPoint.source == "extended")
        {
            ++result.extendedCount;
        }
    }

    if (result.points.isEmpty())
    {
        result.error = "滤波后没有生成有效焊道。";
        return result;
    }

    if (params.keepLongestSegmentOnly && result.points.size() >= 2)
    {
        struct SegmentRange
        {
            int begin = 0;
            int end = 0;
        };

        QVector<SegmentRange> segments;
        SegmentRange currentSegment;
        currentSegment.begin = 0;
        currentSegment.end = 0;
        for (int index = 1; index < result.points.size(); ++index)
        {
            const double distance =
                (result.points[index].point - result.points[index - 1].point).norm();
            if (params.segmentBreakDistance > 0.0 && distance > params.segmentBreakDistance)
            {
                segments.push_back(currentSegment);
                currentSegment.begin = index;
                currentSegment.end = index;
                continue;
            }

            currentSegment.end = index;
        }
        segments.push_back(currentSegment);

        int bestSegmentIndex = 0;
        int bestSegmentLength = 0;
        for (int index = 0; index < segments.size(); ++index)
        {
            const int length = segments[index].end - segments[index].begin + 1;
            if (length > bestSegmentLength)
            {
                bestSegmentLength = length;
                bestSegmentIndex = index;
            }
        }

        if (!segments.isEmpty())
        {
            const SegmentRange bestSegment = segments[bestSegmentIndex];
            QVector<LowerWeldFilterPoint> longestSegmentPoints;
            longestSegmentPoints.reserve(bestSegmentLength);
            for (int index = bestSegment.begin; index <= bestSegment.end; ++index)
            {
                LowerWeldFilterPoint point = result.points[index];
                point.index = longestSegmentPoints.size() + 1;
                longestSegmentPoints.push_back(point);
            }

            result.segmentRejectedCount = result.points.size() - longestSegmentPoints.size();
            result.points = longestSegmentPoints;
        }
    }

    if (result.points.isEmpty())
    {
        result.error = "连续段筛选后没有保留有效焊道。";
        return result;
    }

    if (params.fitMode == LowerWeldFitMode::LineFit)
    {
        if (result.points.size() < 2)
        {
            result.error = "直线拟合至少需要 2 个点。";
            return result;
        }

        const int trimCount = std::max(0, params.lineFitTrimCount);
        const int fitBegin = trimCount;
        const int fitEnd = result.points.size() - trimCount;
        if (fitBegin >= fitEnd || (fitEnd - fitBegin) < 2)
        {
            result.error = QString("直线拟合裁首尾点数过大，剩余点数不足。当前输出点=%1，裁剪=%2。")
                .arg(result.points.size())
                .arg(trimCount);
            return result;
        }

        QVector<double> axisValues;
        QVector<double> firstTargets;
        QVector<double> secondTargets;
        axisValues.reserve(fitEnd - fitBegin);
        firstTargets.reserve(fitEnd - fitBegin);
        secondTargets.reserve(fitEnd - fitBegin);

        for (int pointIndex = fitBegin; pointIndex < fitEnd; ++pointIndex)
        {
            const LowerWeldFilterPoint& point = result.points[pointIndex];
            const double axisValue = SampleAxisValue(point.point, params.sampleAxis);
            axisValues.push_back(axisValue);
            if (params.sampleAxis == SampleAxis::AxisY)
            {
                firstTargets.push_back(point.point.x());
                secondTargets.push_back(point.point.z());
            }
            else
            {
                firstTargets.push_back(point.point.y());
                secondTargets.push_back(point.point.z());
            }
        }

        const QPair<double, double> firstLine = LinearFit1D(axisValues, firstTargets);
        const QPair<double, double> secondLine = LinearFit1D(axisValues, secondTargets);

        for (int index = 0; index < result.points.size(); ++index)
        {
            const double axisValue =
                SampleAxisValue(result.points[index].point, params.sampleAxis);
            Eigen::Vector3d linePoint = result.points[index].point;
            if (params.sampleAxis == SampleAxis::AxisY)
            {
                linePoint.x() = firstLine.first * axisValue + firstLine.second;
                linePoint.y() = axisValue;
                linePoint.z() = secondLine.first * axisValue + secondLine.second;
            }
            else
            {
                linePoint.x() = axisValue;
                linePoint.y() = firstLine.first * axisValue + firstLine.second;
                linePoint.z() = secondLine.first * axisValue + secondLine.second;
            }

            result.points[index].point = linePoint;
            result.points[index].source = "linefit";
        }
        result.fitSegmentCount = 1;
    }
    else if (params.fitMode == LowerWeldFitMode::TrapezoidFit)
    {
        const int trimCount = std::max(0, params.lineFitTrimCount);
        const int fitBegin = trimCount;
        const int fitEnd = result.points.size() - trimCount;
        const int minSegmentPointCount = std::max(3, params.piecewiseMinSegmentPoints);
        if (fitBegin >= fitEnd || (fitEnd - fitBegin) < minSegmentPointCount * 2)
        {
            result.error = QString("梯形分段拟合剩余点数不足。当前输出点=%1，裁剪=%2，至少需要 %3 个点。")
                .arg(result.points.size())
                .arg(trimCount)
                .arg(minSegmentPointCount * 2);
            return result;
        }

        QVector<Eigen::Vector3d> fitPoints;
        QVector<double> axisValues;
        QVector<double> firstTargets;
        QVector<double> secondTargets;
        fitPoints.reserve(fitEnd - fitBegin);
        axisValues.reserve(fitEnd - fitBegin);
        firstTargets.reserve(fitEnd - fitBegin);
        secondTargets.reserve(fitEnd - fitBegin);

        for (int pointIndex = fitBegin; pointIndex < fitEnd; ++pointIndex)
        {
            const LowerWeldFilterPoint& point = result.points[pointIndex];
            fitPoints.push_back(point.point);
            const double axisValue = SampleAxisValue(point.point, params.sampleAxis);
            axisValues.push_back(axisValue);
            if (params.sampleAxis == SampleAxis::AxisY)
            {
                firstTargets.push_back(point.point.x());
                secondTargets.push_back(point.point.z());
            }
            else
            {
                firstTargets.push_back(point.point.y());
                secondTargets.push_back(point.point.z());
            }
        }

        const QVector<int> breakpoints = BuildPiecewiseBreakpoints(
            fitPoints,
            std::max(0.1, params.piecewiseFitTolerance),
            minSegmentPointCount);
        if (breakpoints.size() < 2)
        {
            result.error = "梯形模板拟合失败，无法生成有效分段。";
            return result;
        }

        const LinearFitPrefixSums primarySums = BuildLinearFitPrefixSums(axisValues, firstTargets);
        const LinearFitPrefixSums secondarySums = BuildLinearFitPrefixSums(axisValues, secondTargets);

        QVector<PiecewiseLineModel> rawSegments;
        rawSegments.reserve(breakpoints.size() - 1);
        for (int segmentIndex = 0; segmentIndex < breakpoints.size() - 1; ++segmentIndex)
        {
            PiecewiseLineModel model;
            model.begin = breakpoints[segmentIndex];
            model.end = breakpoints[segmentIndex + 1];
            model.primary = FitLinearRange(primarySums, model.begin, model.end);
            model.secondary = FitLinearRange(secondarySums, model.begin, model.end);
            if (!model.primary.valid || !model.secondary.valid)
            {
                result.error = "梯形模板拟合失败，存在无法拟合的分段。";
                return result;
            }
            rawSegments.push_back(model);
        }

        const bool primaryIsTemplate = ValueRange(firstTargets) >= ValueRange(secondTargets);
        QVector<double> nonHorizontalSlopes;
        nonHorizontalSlopes.reserve(rawSegments.size());
        for (const PiecewiseLineModel& model : rawSegments)
        {
            const double templateSlope = primaryIsTemplate ? model.primary.slope : model.secondary.slope;
            if (std::abs(templateSlope) > 1e-6)
            {
                nonHorizontalSlopes.push_back(std::abs(templateSlope));
            }
        }

        double dominantSlope = nonHorizontalSlopes.isEmpty() ? 0.0 : MedianValue(nonHorizontalSlopes);
        const double horizontalThreshold = dominantSlope > 1e-6
            ? std::max(0.02, dominantSlope * 0.35)
            : 0.05;

        struct TrapezoidTemplateSegment
        {
            int begin = 0;
            int end = 0;
            LinearFitSegment primary;
            LinearFitSegment secondary;
            int slopeClass = 0;
        };

        QVector<TrapezoidTemplateSegment> templateSegments;
        templateSegments.reserve(rawSegments.size());
        for (const PiecewiseLineModel& model : rawSegments)
        {
            const double templateSlope = primaryIsTemplate ? model.primary.slope : model.secondary.slope;
            const int slopeClass = TrapezoidSlopeClass(templateSlope, horizontalThreshold);
            if (!templateSegments.isEmpty() && templateSegments.back().slopeClass == slopeClass)
            {
                templateSegments.back().end = model.end;
                continue;
            }

            TrapezoidTemplateSegment segment;
            segment.begin = model.begin;
            segment.end = model.end;
            segment.slopeClass = slopeClass;
            templateSegments.push_back(segment);
        }

        for (TrapezoidTemplateSegment& segment : templateSegments)
        {
            segment.primary = FitLinearRange(primarySums, segment.begin, segment.end);
            segment.secondary = FitLinearRange(secondarySums, segment.begin, segment.end);
            if (!segment.primary.valid || !segment.secondary.valid)
            {
                result.error = "梯形模板拟合失败，模板分段重拟合失败。";
                return result;
            }
        }

        nonHorizontalSlopes.clear();
        for (const TrapezoidTemplateSegment& segment : templateSegments)
        {
            const double templateSlope = primaryIsTemplate ? segment.primary.slope : segment.secondary.slope;
            if (std::abs(templateSlope) > horizontalThreshold)
            {
                nonHorizontalSlopes.push_back(std::abs(templateSlope));
            }
        }
        if (!nonHorizontalSlopes.isEmpty())
        {
            dominantSlope = MedianValue(nonHorizontalSlopes);
        }

        QVector<double> segmentStartAxes;
        QVector<double> segmentEndAxes;
        QVector<Eigen::Vector3d> segmentStartPoints;
        QVector<Eigen::Vector3d> segmentEndPoints;
        segmentStartAxes.reserve(templateSegments.size());
        segmentEndAxes.reserve(templateSegments.size());
        segmentStartPoints.reserve(templateSegments.size());
        segmentEndPoints.reserve(templateSegments.size());

        for (TrapezoidTemplateSegment& segment : templateSegments)
        {
            LinearFitSegment snappedPrimary = segment.primary;
            LinearFitSegment snappedSecondary = segment.secondary;
            LinearFitSegment& templateModel = primaryIsTemplate ? snappedPrimary : snappedSecondary;
            const double axisMid = (axisValues[segment.begin] + axisValues[segment.end]) * 0.5;
            const double fittedMid = templateModel.slope * axisMid + templateModel.intercept;
            if (segment.slopeClass == 0)
            {
                templateModel.slope = 0.0;
            }
            else
            {
                templateModel.slope = static_cast<double>(segment.slopeClass) * dominantSlope;
            }
            templateModel.intercept = fittedMid - templateModel.slope * axisMid;

            segment.primary = snappedPrimary;
            segment.secondary = snappedSecondary;

            const double startAxis = axisValues[segment.begin];
            const double endAxis = axisValues[segment.end];
            segmentStartAxes.push_back(startAxis);
            segmentEndAxes.push_back(endAxis);
            segmentStartPoints.push_back(EvaluateLineModelPoint(
                params.sampleAxis, startAxis, segment.primary, segment.secondary));
            segmentEndPoints.push_back(EvaluateLineModelPoint(
                params.sampleAxis, endAxis, segment.primary, segment.secondary));
        }

        for (int segmentIndex = 0; segmentIndex + 1 < templateSegments.size(); ++segmentIndex)
        {
            const double jointAxis = axisValues[templateSegments[segmentIndex].end];
            const Eigen::Vector3d currentJoint = EvaluateLineModelPoint(
                params.sampleAxis,
                jointAxis,
                templateSegments[segmentIndex].primary,
                templateSegments[segmentIndex].secondary);
            const Eigen::Vector3d nextJoint = EvaluateLineModelPoint(
                params.sampleAxis,
                jointAxis,
                templateSegments[segmentIndex + 1].primary,
                templateSegments[segmentIndex + 1].secondary);
            const Eigen::Vector3d mergedJoint = (currentJoint + nextJoint) * 0.5;
            segmentEndPoints[segmentIndex] = mergedJoint;
            segmentStartPoints[segmentIndex + 1] = mergedJoint;
        }

        for (int index = 0; index < result.points.size(); ++index)
        {
            const int fitPointCount = static_cast<int>(fitPoints.size());
            int localIndex = std::clamp(index - fitBegin, 0, fitPointCount - 1);
            int matchedSegment = 0;
            while (matchedSegment + 1 < templateSegments.size() &&
                localIndex > templateSegments[matchedSegment].end)
            {
                ++matchedSegment;
            }

            const double axisValue =
                SampleAxisValue(result.points[index].point, params.sampleAxis);
            const double segmentStartAxis = segmentStartAxes[matchedSegment];
            const double segmentEndAxis = segmentEndAxes[matchedSegment];
            const Eigen::Vector3d& segmentStartPoint = segmentStartPoints[matchedSegment];
            const Eigen::Vector3d& segmentEndPoint = segmentEndPoints[matchedSegment];
            double interpolationRatio = 0.0;
            if (std::abs(segmentEndAxis - segmentStartAxis) > std::numeric_limits<double>::epsilon())
            {
                interpolationRatio =
                    (axisValue - segmentStartAxis) / (segmentEndAxis - segmentStartAxis);
            }
            interpolationRatio = std::clamp(interpolationRatio, 0.0, 1.0);

            Eigen::Vector3d segmentPoint =
                segmentStartPoint + (segmentEndPoint - segmentStartPoint) * interpolationRatio;
            segmentPoint = SetSampleAxisValue(segmentPoint, params.sampleAxis, axisValue);
            result.points[index].point = segmentPoint;
            result.points[index].source = QString("trapfit%1").arg(matchedSegment + 1);
        }
        result.fitSegmentCount = templateSegments.size();
    }
    else if (params.fitMode == LowerWeldFitMode::PiecewiseLineFit)
    {
        const int trimCount = std::max(0, params.lineFitTrimCount);
        const int fitBegin = trimCount;
        const int fitEnd = result.points.size() - trimCount;
        if (fitBegin >= fitEnd || (fitEnd - fitBegin) < 2)
        {
            result.error = QString("多段分段拟合剩余点数不足。当前输出点=%1，裁剪=%2。")
                .arg(result.points.size())
                .arg(trimCount);
            return result;
        }

        QVector<Eigen::Vector3d> fitPoints;
        QVector<double> axisValues;
        QVector<double> firstTargets;
        QVector<double> secondTargets;
        fitPoints.reserve(fitEnd - fitBegin);
        axisValues.reserve(fitEnd - fitBegin);
        firstTargets.reserve(fitEnd - fitBegin);
        secondTargets.reserve(fitEnd - fitBegin);

        for (int pointIndex = fitBegin; pointIndex < fitEnd; ++pointIndex)
        {
            const LowerWeldFilterPoint& point = result.points[pointIndex];
            fitPoints.push_back(point.point);
            const double axisValue = SampleAxisValue(point.point, params.sampleAxis);
            axisValues.push_back(axisValue);
            if (params.sampleAxis == SampleAxis::AxisY)
            {
                firstTargets.push_back(point.point.x());
                secondTargets.push_back(point.point.z());
            }
            else
            {
                firstTargets.push_back(point.point.y());
                secondTargets.push_back(point.point.z());
            }
        }

        const QVector<int> breakpoints = BuildPiecewiseBreakpoints(
            fitPoints,
            std::max(0.1, params.piecewiseFitTolerance),
            params.piecewiseMinSegmentPoints);
        if (breakpoints.size() < 2)
        {
            result.error = "多段分段拟合失败，无法生成有效分段。";
            return result;
        }

        const LinearFitPrefixSums primarySums = BuildLinearFitPrefixSums(axisValues, firstTargets);
        const LinearFitPrefixSums secondarySums = BuildLinearFitPrefixSums(axisValues, secondTargets);
        QVector<PiecewiseLineModel> segmentModels;
        segmentModels.reserve(breakpoints.size() - 1);
        for (int segmentIndex = 0; segmentIndex < breakpoints.size() - 1; ++segmentIndex)
        {
            PiecewiseLineModel model;
            model.begin = breakpoints[segmentIndex];
            model.end = breakpoints[segmentIndex + 1];
            model.primary = FitLinearRange(primarySums, model.begin, model.end);
            model.secondary = FitLinearRange(secondarySums, model.begin, model.end);
            if (!model.primary.valid || !model.secondary.valid)
            {
                result.error = "多段分段拟合失败，存在无法拟合的分段。";
                return result;
            }
            segmentModels.push_back(model);
        }

        QVector<double> segmentStartAxes;
        QVector<double> segmentEndAxes;
        QVector<Eigen::Vector3d> segmentStartPoints;
        QVector<Eigen::Vector3d> segmentEndPoints;
        segmentStartAxes.reserve(segmentModels.size());
        segmentEndAxes.reserve(segmentModels.size());
        segmentStartPoints.reserve(segmentModels.size());
        segmentEndPoints.reserve(segmentModels.size());

        for (int segmentIndex = 0; segmentIndex < segmentModels.size(); ++segmentIndex)
        {
            const PiecewiseLineModel& model = segmentModels[segmentIndex];
            const double startAxis = axisValues[model.begin];
            const double endAxis = axisValues[model.end];
            segmentStartAxes.push_back(startAxis);
            segmentEndAxes.push_back(endAxis);
            segmentStartPoints.push_back(EvaluateLineModelPoint(
                params.sampleAxis, startAxis, model.primary, model.secondary));
            segmentEndPoints.push_back(EvaluateLineModelPoint(
                params.sampleAxis, endAxis, model.primary, model.secondary));
        }

        for (int segmentIndex = 0; segmentIndex + 1 < segmentModels.size(); ++segmentIndex)
        {
            const double jointAxis = axisValues[segmentModels[segmentIndex].end];
            const Eigen::Vector3d currentJoint = EvaluateLineModelPoint(
                params.sampleAxis,
                jointAxis,
                segmentModels[segmentIndex].primary,
                segmentModels[segmentIndex].secondary);
            const Eigen::Vector3d nextJoint = EvaluateLineModelPoint(
                params.sampleAxis,
                jointAxis,
                segmentModels[segmentIndex + 1].primary,
                segmentModels[segmentIndex + 1].secondary);
            const Eigen::Vector3d mergedJoint = (currentJoint + nextJoint) * 0.5;
            segmentEndPoints[segmentIndex] = mergedJoint;
            segmentStartPoints[segmentIndex + 1] = mergedJoint;
        }

        for (int index = 0; index < result.points.size(); ++index)
        {
            const int fitPointCount = static_cast<int>(fitPoints.size());
            int localIndex = std::clamp(index - fitBegin, 0, fitPointCount - 1);
            int matchedSegment = 0;
            while (matchedSegment + 1 < segmentModels.size() &&
                localIndex > segmentModels[matchedSegment].end)
            {
                ++matchedSegment;
            }

            const PiecewiseLineModel& model = segmentModels[matchedSegment];
            const double axisValue =
                SampleAxisValue(result.points[index].point, params.sampleAxis);
            const double segmentStartAxis = segmentStartAxes[matchedSegment];
            const double segmentEndAxis = segmentEndAxes[matchedSegment];
            const Eigen::Vector3d& segmentStartPoint = segmentStartPoints[matchedSegment];
            const Eigen::Vector3d& segmentEndPoint = segmentEndPoints[matchedSegment];
            double interpolationRatio = 0.0;
            if (std::abs(segmentEndAxis - segmentStartAxis) > std::numeric_limits<double>::epsilon())
            {
                interpolationRatio =
                    (axisValue - segmentStartAxis) / (segmentEndAxis - segmentStartAxis);
            }
            interpolationRatio = std::clamp(interpolationRatio, 0.0, 1.0);

            Eigen::Vector3d segmentPoint =
                segmentStartPoint + (segmentEndPoint - segmentStartPoint) * interpolationRatio;
            segmentPoint = SetSampleAxisValue(segmentPoint, params.sampleAxis, axisValue);

            result.points[index].point = segmentPoint;
            result.points[index].source = QString("piecefit%1").arg(matchedSegment + 1);
        }

        result.fitSegmentCount = segmentModels.size();
    }

    result.ok = true;
    return result;
}

QString RobotCalculation::RobotPoseCsv(qint64 timestampUs, const T_ROBOT_COORS& pose)
{
    return QString("%1,%2,%3,%4,%5,%6,%7,%8,%9,%10")
        .arg(QString::number(timestampUs))
        .arg(pose.dX, 0, 'f', 6)
        .arg(pose.dY, 0, 'f', 6)
        .arg(pose.dZ, 0, 'f', 6)
        .arg(pose.dRX, 0, 'f', 6)
        .arg(pose.dRY, 0, 'f', 6)
        .arg(pose.dRZ, 0, 'f', 6)
        .arg(pose.dBX, 0, 'f', 6)
        .arg(pose.dBY, 0, 'f', 6)
        .arg(pose.dBZ, 0, 'f', 6);
}

RobotCalculation::LowerWeldClassificationResult RobotCalculation::ClassifyLowerWeldPoints(
    const LowerWeldFilterResult& filterResult,
    SampleAxis sampleAxis)
{
    LowerWeldClassificationResult result;
    if (!filterResult.ok)
    {
        result.error = filterResult.error.isEmpty()
            ? "滤波结果无效，无法分类。"
            : filterResult.error;
        return result;
    }
    if (filterResult.points.isEmpty())
    {
        result.error = "滤波结果为空，无法分类。";
        return result;
    }

    result.points.reserve(filterResult.points.size());
    QVector<double> primaryValues;
    primaryValues.reserve(filterResult.points.size());
    const int pointCount = static_cast<int>(filterResult.points.size());

    for (const LowerWeldFilterPoint& point : filterResult.points)
    {
        LowerWeldClassifiedPoint classifiedPoint;
        classifiedPoint.index = point.index;
        classifiedPoint.point = point.point;
        classifiedPoint.type = LowerWeldPointType::Normal;
        classifiedPoint.source = point.source;
        result.points.push_back(classifiedPoint);

        if (sampleAxis == SampleAxis::AxisY)
        {
            primaryValues.push_back(point.point.x());
        }
        else
        {
            primaryValues.push_back(point.point.y());
        }
    }

    auto percentileValue = [](QVector<double> values, double percentile) -> double
    {
        if (values.isEmpty())
        {
            return 0.0;
        }
        std::sort(values.begin(), values.end());
        const double clampedPercentile = std::clamp(percentile, 0.0, 1.0);
        const int index = static_cast<int>(std::round(clampedPercentile * (values.size() - 1)));
        return values[index];
    };

    QVector<double> slopes;
    slopes.reserve(result.points.size());
    for (int index = 0; index < pointCount; ++index)
    {
        const int previousIndex = std::max(0, index - 1);
        const int nextIndex = std::min(pointCount - 1, index + 1);
        const double previousAxis = (sampleAxis == SampleAxis::AxisY)
            ? result.points[previousIndex].point.y()
            : result.points[previousIndex].point.x();
        const double nextAxis = (sampleAxis == SampleAxis::AxisY)
            ? result.points[nextIndex].point.y()
            : result.points[nextIndex].point.x();
        const double deltaAxis = nextAxis - previousAxis;
        if (std::abs(deltaAxis) < 1e-6)
        {
            slopes.push_back(0.0);
            continue;
        }
        const double deltaPrimary = primaryValues[nextIndex] - primaryValues[previousIndex];
        slopes.push_back(deltaPrimary / deltaAxis);
    }

    const double lowRiseThreshold = percentileValue(primaryValues, 0.30);
    const double lowFallThreshold = percentileValue(primaryValues, 0.32);
    const double highThreshold = percentileValue(primaryValues, 0.64);
    const double startFlatSlopeThreshold = 0.25;
    const double risingSlopeThreshold = 0.40;
    const double fallingSlopeThreshold = -0.45;

    enum class LowerWeldPhase
    {
        SeekingStart,
        LowPlatform,
        RisingEdge,
        HighPlatform,
        FallingEdge
    };

    LowerWeldPhase phase = LowerWeldPhase::SeekingStart;
    bool startAssigned = false;

    if (!result.points.isEmpty())
    {
        for (int index = 0; index < result.points.size(); ++index)
        {
            const double primary = primaryValues[index];
            const double slope = slopes[index];

            if (!startAssigned)
            {
                if (primary <= lowFallThreshold && std::abs(slope) <= startFlatSlopeThreshold)
                {
                    result.points[index].type = LowerWeldPointType::Start;
                    result.startCount = 1;
                    startAssigned = true;
                    phase = LowerWeldPhase::LowPlatform;
                }
                continue;
            }

            switch (phase)
            {
            case LowerWeldPhase::LowPlatform:
                if (slope >= risingSlopeThreshold && primary >= lowRiseThreshold)
                {
                    result.points[index].type = LowerWeldPointType::InnerCorner;
                    ++result.innerCornerCount;
                    phase = LowerWeldPhase::RisingEdge;
                }
                break;
            case LowerWeldPhase::RisingEdge:
                if (primary >= highThreshold)
                {
                    result.points[index].type = LowerWeldPointType::OuterCorner;
                    ++result.outerCornerCount;
                    phase = LowerWeldPhase::HighPlatform;
                }
                break;
            case LowerWeldPhase::HighPlatform:
                if (slope <= fallingSlopeThreshold && primary >= highThreshold)
                {
                    result.points[index].type = LowerWeldPointType::OuterCorner;
                    ++result.outerCornerCount;
                    phase = LowerWeldPhase::FallingEdge;
                }
                break;
            case LowerWeldPhase::FallingEdge:
                if (primary <= lowFallThreshold)
                {
                    result.points[index].type = LowerWeldPointType::InnerCorner;
                    ++result.innerCornerCount;
                    phase = LowerWeldPhase::LowPlatform;
                }
                break;
            case LowerWeldPhase::SeekingStart:
            default:
                break;
            }
        }
    }

    if (!startAssigned)
    {
        result.points.front().type = LowerWeldPointType::Start;
        result.startCount = 1;
    }

    if (result.points.size() >= 2)
    {
        result.points.back().type = LowerWeldPointType::End;
        result.endCount = 1;
    }

    for (const LowerWeldClassifiedPoint& point : result.points)
    {
        switch (point.type)
        {
        case LowerWeldPointType::Start:
            break;
        case LowerWeldPointType::End:
            break;
        case LowerWeldPointType::InnerCorner:
            break;
        case LowerWeldPointType::OuterCorner:
            break;
        case LowerWeldPointType::Noise:
            ++result.noiseCount;
            break;
        case LowerWeldPointType::Normal:
        default:
            ++result.normalCount;
            break;
        }
    }

    result.ok = true;
    return result;
}

RobotCalculation::MeasureThenWeldAnalysisResult RobotCalculation::AnalyzeMeasureThenWeldLowerWeldPath(
    const QVector<IndexedPoint3D>& inputPoints,
    const LowerWeldFilterParams& params)
{
    MeasureThenWeldAnalysisResult result;
    result.filterResult = FilterLowerWeldPath(inputPoints, params);
    if (!result.filterResult.ok)
    {
        result.error = result.filterResult.error;
        return result;
    }

    result.classificationResult = ClassifyLowerWeldPoints(result.filterResult, params.sampleAxis);
    if (!result.classificationResult.ok)
    {
        result.error = result.classificationResult.error;
        return result;
    }

    result.ok = true;
    return result;
}

RobotCalculation::MeasureThenWeldAnalysisResult RobotCalculation::AnalyzeMeasureThenWeldLowerWeldPathGeometry(
    const QVector<IndexedPoint3D>& inputPoints,
    const LowerWeldFilterParams& params)
{
    MeasureThenWeldAnalysisResult result;
    result.filterResult.inputPointCount = inputPoints.size();

    if (inputPoints.isEmpty())
    {
        result.error = "输入点为空，无法分析。";
        return result;
    }
    if (params.sampleStep <= 0.0)
    {
        result.error = "采样步长必须大于 0。";
        return result;
    }

    QVector<IndexedPoint3D> validPoints;
    validPoints.reserve(inputPoints.size());
    for (const IndexedPoint3D& sample : inputPoints)
    {
        if (IsFinitePoint(sample.point))
        {
            validPoints.push_back(sample);
        }
    }

    int denoiseRejectedCount = 0;
    const QVector<IndexedPoint3D> denoisedPoints =
        RemoveGeometryLocalOutliers(validPoints, params, &denoiseRejectedCount);

    if (denoisedPoints.size() < std::max(2, params.minPointCount))
    {
        result.error = QString("有效点太少，仅 %1 个，无法进行几何特征提取。")
            .arg(denoisedPoints.size());
        return result;
    }

    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    for (const IndexedPoint3D& sample : denoisedPoints)
    {
        center += sample.point;
    }
    center /= static_cast<double>(denoisedPoints.size());

    const QPair<Eigen::Vector3d, Eigen::Vector3d> axes = BuildGeometryAxes(denoisedPoints, center);
    Eigen::Vector3d normalAxis = axes.first.cross(axes.second);
    if (normalAxis.squaredNorm() <= std::numeric_limits<double>::epsilon())
    {
        normalAxis = UnitAxis((DominantDimension(axes.first) + 2) % 3);
        normalAxis -= axes.first * normalAxis.dot(axes.first);
        normalAxis -= axes.second * normalAxis.dot(axes.second);
    }
    if (normalAxis.squaredNorm() <= std::numeric_limits<double>::epsilon())
    {
        normalAxis = Eigen::Vector3d::UnitZ();
    }
    normalAxis.normalize();

    QVector<GeometryProjectedPoint> projected = ProjectGeometryPoints(
        denoisedPoints,
        center,
        axes.first,
        axes.second,
        normalAxis,
        params.smoothRadius);
    int projectedBranchRejectedCount = 0;
    projected = RemoveGeometryProjectedBranchOutliers(projected, params, &projectedBranchRejectedCount);
    denoiseRejectedCount += projectedBranchRejectedCount;

    result.filterResult.lowerPointCount = projected.size();
    result.filterResult.zContinuityRejectedCount = denoiseRejectedCount;
    if (projected.size() < std::max(2, params.minPointCount))
    {
        result.error = QString("有效点太少，仅 %1 个，无法进行几何特征提取。")
            .arg(projected.size());
        return result;
    }

    QVector<int> keyIndexes = PruneRedundantSameSideGeometryKeys(
        projected,
        BuildGeometryKeyIndexes(projected, params));
    keyIndexes = PruneGeometrySpikeDrivenKeys(projected, keyIndexes, params);
    keyIndexes = PruneNonMonotonicFittedGeometryKeys(
        projected,
        keyIndexes,
        center,
        axes.first,
        axes.second,
        normalAxis);
    keyIndexes = RefineGeometryKeysBySegmentDeviation(
        projected,
        keyIndexes,
        center,
        axes.first,
        axes.second,
        params);
    keyIndexes = PruneGeometrySpikeDrivenKeys(projected, keyIndexes, params);
    keyIndexes = PruneNonMonotonicFittedGeometryKeys(
        projected,
        keyIndexes,
        center,
        axes.first,
        axes.second,
        normalAxis);
    if (keyIndexes.size() < 2)
    {
        result.error = "几何特征提取失败，未找到可用的起点/终点。";
        return result;
    }
    const QVector<Eigen::Vector3d> fittedKeyPoints = BuildFittedGeometryKeyPoints(
        projected,
        keyIndexes,
        center,
        axes.first,
        axes.second,
        normalAxis);
    if (fittedKeyPoints.size() != keyIndexes.size())
    {
        result.error = "几何特征拟合失败，无法生成拟合交点。";
        return result;
    }

    result.filterResult.points.reserve(projected.size());
    for (const GeometryProjectedPoint& point : projected)
    {
        LowerWeldFilterPoint outputPoint;
        outputPoint.index = point.inputIndex;
        outputPoint.point = point.point;
        outputPoint.source = denoiseRejectedCount > 0 ? "geometry_denoised" : "geometry_original";
        result.filterResult.points.push_back(outputPoint);
    }

    const int keyPointCount = static_cast<int>(keyIndexes.size());
    result.filterResult.measuredCount = keyPointCount;
    result.filterResult.interpolatedCount = 0;
    result.filterResult.extendedCount = 0;
    result.filterResult.fitSegmentCount = std::max(1, keyPointCount - 1);
    result.filterResult.ok = true;

    const double expandStepMm = params.sampleStep > 0.0 ? params.sampleStep : 2.0;
    result.keyPoints.clear();
    result.keyPoints.reserve(keyIndexes.size());
    result.classificationResult.points.clear();
    result.classificationResult.points.reserve(keyIndexes.size() * 8);
    result.classificationResult.ok = true;
    result.classificationResult.error.clear();

    int nextIndex = 1;
    int interpolatedCount = 0;
    for (int index = 0; index < keyIndexes.size(); ++index)
    {
        const Eigen::Vector3d currentPoint = fittedKeyPoints[index];
        const LowerWeldPointType pointType = GeometryCornerType(projected, keyIndexes, index);
        const QString source = pointType == LowerWeldPointType::Start
            ? "geometry_start"
            : (pointType == LowerWeldPointType::End
                ? "geometry_end"
                : (pointType == LowerWeldPointType::InnerCorner
                    ? "geometry_inner"
                    : "geometry_outer"));

        LowerWeldClassifiedPoint keyPoint;
        keyPoint.index = projected[keyIndexes[index]].inputIndex;
        keyPoint.point = currentPoint;
        keyPoint.type = pointType;
        keyPoint.source = source;
        result.keyPoints.push_back(keyPoint);

        AppendExpandedGroovePoint(
            result.classificationResult.points,
            nextIndex,
            currentPoint,
            pointType,
            source);

        if (index + 1 >= keyIndexes.size())
        {
            continue;
        }

        const Eigen::Vector3d nextPoint = fittedKeyPoints[index + 1];
        const Eigen::Vector3d delta = nextPoint - currentPoint;
        const double segmentLength = delta.norm();
        if (segmentLength <= std::numeric_limits<double>::epsilon())
        {
            continue;
        }

        for (double distance = expandStepMm; distance < segmentLength - 1e-9; distance += expandStepMm)
        {
            const double ratio = distance / segmentLength;
            AppendExpandedGroovePoint(
                result.classificationResult.points,
                nextIndex,
                currentPoint + delta * ratio,
                LowerWeldPointType::Normal,
                "geometry_2mm");
            ++interpolatedCount;
        }
    }

    result.filterResult.interpolatedCount = interpolatedCount;
    result.filterResult.extendedCount = result.classificationResult.points.size();
    CountLowerWeldClassifiedPoints(result.classificationResult);

    result.ok = true;
    return result;
}

RobotCalculation::MeasureThenWeldAnalysisResult RobotCalculation::AnalyzeMeasureThenWeldLowerWeldPathDirect(
    const QVector<IndexedPoint3D>& inputPoints,
    const LowerWeldFilterParams& params)
{
    return AnalyzeMeasureThenWeldLowerWeldPathGeometry(inputPoints, params);
}

int RobotCalculation::LowerWeldPointTypeCode(LowerWeldPointType type)
{
    return static_cast<int>(type);
}

QString RobotCalculation::LowerWeldPointTypeName(LowerWeldPointType type)
{
    switch (type)
    {
    case LowerWeldPointType::Start:
        return "start";
    case LowerWeldPointType::End:
        return "end";
    case LowerWeldPointType::InnerCorner:
        return "inner_corner";
    case LowerWeldPointType::OuterCorner:
        return "outer_corner";
    case LowerWeldPointType::Noise:
        return "noise";
    case LowerWeldPointType::Normal:
    default:
        return "normal";
    }
}

QString RobotCalculation::Vector3Csv(qint64 timestampUs, const Eigen::Vector3d& point, const QString& extra)
{
    QString line = QString("%1,%2,%3,%4")
        .arg(QString::number(timestampUs))
        .arg(point.x(), 0, 'f', 6)
        .arg(point.y(), 0, 'f', 6)
        .arg(point.z(), 0, 'f', 6);
    if (!extra.isEmpty())
    {
        line += "," + extra;
    }
    return line;
}

QString RobotCalculation::RobotPoseIndexedCsv(int index, const T_ROBOT_COORS& pose)
{
    return QString("%1,%2,%3,%4,%5,%6,%7,%8,%9,%10")
        .arg(index)
        .arg(pose.dX, 0, 'f', 6)
        .arg(pose.dY, 0, 'f', 6)
        .arg(pose.dZ, 0, 'f', 6)
        .arg(pose.dRX, 0, 'f', 6)
        .arg(pose.dRY, 0, 'f', 6)
        .arg(pose.dRZ, 0, 'f', 6)
        .arg(pose.dBX, 0, 'f', 6)
        .arg(pose.dBY, 0, 'f', 6)
        .arg(pose.dBZ, 0, 'f', 6);
}

QString RobotCalculation::Vector3IndexedCsv(int index, const Eigen::Vector3d& point, const QString& extra)
{
    QString line = QString("%1,%2,%3,%4")
        .arg(index)
        .arg(point.x(), 0, 'f', 6)
        .arg(point.y(), 0, 'f', 6)
        .arg(point.z(), 0, 'f', 6);
    if (!extra.isEmpty())
    {
        line += "," + extra;
    }
    return line;
}

QString RobotCalculation::Vector3IndexedSpaceText(int index, const Eigen::Vector3d& point, const QString& extra)
{
    QString line = QString("%1 %2 %3 %4")
        .arg(index)
        .arg(point.x(), 0, 'f', 6)
        .arg(point.y(), 0, 'f', 6)
        .arg(point.z(), 0, 'f', 6);
    if (!extra.isEmpty())
    {
        line += " " + extra;
    }
    return line;
}
