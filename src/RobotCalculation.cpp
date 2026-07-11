#include "RobotCalculation.h"
#include "RobotPoseTransform.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include <QDir>
#include <QFile>
#include <QPair>
#include <QString>
#include <QStringList>
#include <QTextStream>

namespace
{
void CountLowerWeldClassifiedPoints(RobotCalculation::LowerWeldClassificationResult& result);

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
    const QString& source,
    const QString& segmentKindAfter = QString())
{
    RobotCalculation::LowerWeldClassifiedPoint classifiedPoint;
    classifiedPoint.index = nextIndex++;
    classifiedPoint.point = point;
    classifiedPoint.type = type;
    classifiedPoint.source = source;
    classifiedPoint.segmentKindAfter = segmentKindAfter;
    points.push_back(classifiedPoint);
}

bool HasCornerCompensation(const RobotCalculation::LowerWeldFilterParams& params)
{
    if (!params.enableCornerCompensation)
    {
        return false;
    }

    const auto hasValue = [](const RobotCalculation::LowerWeldFilterParams::CornerCompensation& comp)
        {
            return std::abs(comp.innerToOuterMm) > 1e-9
                || std::abs(comp.innerToInnerMm) > 1e-9
                || std::abs(comp.outerToOuterMm) > 1e-9
                || std::abs(comp.outerToInnerMm) > 1e-9;
        };
    return hasValue(params.risingCornerCompensation)
        || hasValue(params.fallingCornerCompensation);
}

double ProfileAxisValue(const Eigen::Vector3d& point, RobotCalculation::SampleAxis axis)
{
    return axis == RobotCalculation::SampleAxis::AxisY ? point.x() : point.y();
}

QString SegmentKindFromKeyGeometry(
    const Eigen::Vector3d& begin,
    const Eigen::Vector3d& end,
    RobotCalculation::SampleAxis axis,
    double lowHighMidpoint,
    double profileRange)
{
    const double dPath = SampleAxisValue(end, axis) - SampleAxisValue(begin, axis);
    const double dProfile = ProfileAxisValue(end, axis) - ProfileAxisValue(begin, axis);
    const double slopeThresholdMm = std::max(1.0, profileRange * 0.12);
    const double slopeRatioThreshold = 0.25;
    const bool isSlope =
        std::abs(dProfile) >= slopeThresholdMm
        && std::abs(dProfile) >= std::abs(dPath) * slopeRatioThreshold;
    if (isSlope)
    {
        return dProfile >= 0.0 ? "rising_edge" : "falling_edge";
    }

    const double middleProfile = (ProfileAxisValue(begin, axis) + ProfileAxisValue(end, axis)) * 0.5;
    return middleProfile <= lowHighMidpoint ? "low_platform" : "high_platform";
}

QVector<QString> AssignSegmentKindsFromKeyPoints(
    const QVector<RobotCalculation::LowerWeldClassifiedPoint>& keyPoints,
    RobotCalculation::SampleAxis axis)
{
    QVector<QString> segmentKinds;
    if (keyPoints.size() < 2)
    {
        return segmentKinds;
    }

    segmentKinds.reserve(keyPoints.size() - 1);
    bool hasAllStoredKinds = true;
    for (int index = 0; index + 1 < keyPoints.size(); ++index)
    {
        const QString kind = keyPoints[index].segmentKindAfter.trimmed();
        if (kind.isEmpty())
        {
            hasAllStoredKinds = false;
            break;
        }
        segmentKinds.push_back(kind);
    }
    if (hasAllStoredKinds)
    {
        return segmentKinds;
    }

    double minProfile = std::numeric_limits<double>::max();
    double maxProfile = std::numeric_limits<double>::lowest();
    for (const RobotCalculation::LowerWeldClassifiedPoint& point : keyPoints)
    {
        const double profile = ProfileAxisValue(point.point, axis);
        minProfile = std::min(minProfile, profile);
        maxProfile = std::max(maxProfile, profile);
    }

    segmentKinds.clear();
    if (!std::isfinite(minProfile) || !std::isfinite(maxProfile))
    {
        segmentKinds.fill("low_platform", keyPoints.size() - 1);
        return segmentKinds;
    }

    const double profileRange = std::max(0.0, maxProfile - minProfile);
    const double lowHighMidpoint = (minProfile + maxProfile) * 0.5;
    for (int index = 0; index + 1 < keyPoints.size(); ++index)
    {
        segmentKinds.push_back(SegmentKindFromKeyGeometry(
            keyPoints[index].point,
            keyPoints[index + 1].point,
            axis,
            lowHighMidpoint,
            profileRange));
    }
    return segmentKinds;
}

bool IsRisingSegmentKind(const QString& kind)
{
    return kind.compare("rising_edge", Qt::CaseInsensitive) == 0;
}

bool IsFallingSegmentKind(const QString& kind)
{
    return kind.compare("falling_edge", Qt::CaseInsensitive) == 0;
}

const RobotCalculation::LowerWeldFilterParams::CornerCompensation* CornerCompensationForSlopeKind(
    const QString& slopeKind,
    const RobotCalculation::LowerWeldFilterParams& params)
{
    if (IsRisingSegmentKind(slopeKind))
    {
        return &params.risingCornerCompensation;
    }
    if (IsFallingSegmentKind(slopeKind))
    {
        return &params.fallingCornerCompensation;
    }
    return nullptr;
}

double CornerCompensationMm(
    RobotCalculation::LowerWeldPointType pointType,
    RobotCalculation::LowerWeldPointType otherPointType,
    const QString& slopeKind,
    const RobotCalculation::LowerWeldFilterParams& params)
{
    const RobotCalculation::LowerWeldFilterParams::CornerCompensation* comp =
        CornerCompensationForSlopeKind(slopeKind, params);
    if (comp == nullptr)
    {
        return 0.0;
    }

    if (pointType == RobotCalculation::LowerWeldPointType::InnerCorner)
    {
        if (otherPointType == RobotCalculation::LowerWeldPointType::OuterCorner)
        {
            return comp->innerToOuterMm;
        }
        if (otherPointType == RobotCalculation::LowerWeldPointType::InnerCorner)
        {
            return comp->innerToInnerMm;
        }
    }
    else if (pointType == RobotCalculation::LowerWeldPointType::OuterCorner)
    {
        if (otherPointType == RobotCalculation::LowerWeldPointType::OuterCorner)
        {
            return comp->outerToOuterMm;
        }
        if (otherPointType == RobotCalculation::LowerWeldPointType::InnerCorner)
        {
            return comp->outerToInnerMm;
        }
    }
    return 0.0;
}

RobotCalculation::LowerWeldClassificationResult BuildExpandedClassificationFromKeyPoints(
    const QVector<RobotCalculation::LowerWeldClassifiedPoint>& keyPoints,
    double sampleStep,
    const QString& normalSource)
{
    RobotCalculation::LowerWeldClassificationResult result;
    if (keyPoints.size() < 2)
    {
        result.error = "拐点数量不足，无法生成 2mm 焊道点。";
        return result;
    }
    if (sampleStep <= 0.0)
    {
        result.error = "采样步长必须大于 0。";
        return result;
    }

    result.points.reserve(keyPoints.size() * 8);
    int nextIndex = 1;
    for (int index = 0; index < keyPoints.size(); ++index)
    {
        const RobotCalculation::LowerWeldClassifiedPoint& current = keyPoints[index];
        const QString segmentKindAfter = index + 1 < keyPoints.size()
            ? current.segmentKindAfter
            : QString();
        AppendExpandedGroovePoint(
            result.points,
            nextIndex,
            current.point,
            current.type,
            current.source,
            segmentKindAfter);

        if (index + 1 >= keyPoints.size())
        {
            continue;
        }

        const Eigen::Vector3d delta = keyPoints[index + 1].point - current.point;
        const double segmentLength = delta.norm();
        if (segmentLength <= std::numeric_limits<double>::epsilon())
        {
            continue;
        }

        for (double distance = sampleStep; distance < segmentLength - 1e-9; distance += sampleStep)
        {
            const double ratio = distance / segmentLength;
            AppendExpandedGroovePoint(
                result.points,
                nextIndex,
                current.point + delta * ratio,
                RobotCalculation::LowerWeldPointType::Normal,
                normalSource,
                segmentKindAfter);
        }
    }

    result.ok = true;
    CountLowerWeldClassifiedPoints(result);
    return result;
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

    if (smoothRadius <= 0)
    {
        // 输入已去噪(smoothRadius<=0)：不做均值平滑（平滑会削圆尖角、把拐点向缓侧搬移半个窗口），
        // smoothH/smoothN 直接取原值。
        for (GeometryProjectedPoint& point : *projected)
        {
            point.smoothH = point.h;
            point.smoothN = point.n;
        }
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

double GeometryPercentileScalar(QVector<double> values, double percentile)
{
    if (values.isEmpty())
    {
        return 0.0;
    }
    std::sort(values.begin(), values.end());
    const double clampedPercentile = std::max(0.0, std::min(1.0, percentile));
    const int index = static_cast<int>(std::round(clampedPercentile * (values.size() - 1)));
    return values[std::max(0, std::min(static_cast<int>(values.size()) - 1, index))];
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

double WorkpieceProjectionStationValue(
    const Eigen::Vector3d& point,
    RobotCalculation::SampleAxis axis)
{
    return axis == RobotCalculation::SampleAxis::AxisX ? point.x() : point.y();
}

double WorkpieceProjectionTransverseValue(
    const Eigen::Vector3d& point,
    RobotCalculation::SampleAxis axis)
{
    return axis == RobotCalculation::SampleAxis::AxisX ? point.y() : point.x();
}




QVector<Eigen::Vector3d> DownsampleWorkpieceProjectionCandidates(
    const QVector<Eigen::Vector3d>& candidates,
    int maxCount)
{
    if (maxCount <= 0 || candidates.size() <= maxCount)
    {
        return candidates;
    }

    QVector<Eigen::Vector3d> sampled;
    sampled.reserve(maxCount);
    const int stride = std::max(1, static_cast<int>(std::ceil(
        static_cast<double>(candidates.size()) / static_cast<double>(maxCount))));
    for (int index = 0; index < candidates.size() && sampled.size() < maxCount; index += stride)
    {
        sampled.push_back(candidates[index]);
    }
    return sampled;
}

QVector<Eigen::Vector3d> SelectMiddleWorkpieceProjectionLayerCandidates(
    const QVector<Eigen::Vector3d>& candidates,
    int maxCount,
    double layerLowPercent,
    double layerHighPercent)
{
    if (candidates.size() < 16)
    {
        return DownsampleWorkpieceProjectionCandidates(candidates, maxCount);
    }

    QVector<double> zValues;
    zValues.reserve(candidates.size());
    for (const Eigen::Vector3d& point : candidates)
    {
        zValues.push_back(point.z());
    }
    std::sort(zValues.begin(), zValues.end());

    const double lowRatio = std::clamp(layerLowPercent, 0.0, 100.0) / 100.0;
    const double highRatio = std::clamp(std::max(layerHighPercent, layerLowPercent), 0.0, 100.0) / 100.0;
    const int lastIndex = zValues.size() - 1;
    const int lowIndex = candidates.size() >= 100
        ? std::clamp(static_cast<int>(std::floor(lastIndex * lowRatio)), 0, lastIndex)
        : 0;
    const int highIndex = std::clamp(static_cast<int>(std::floor(lastIndex * highRatio)), lowIndex, lastIndex);
    const double lowerZ = zValues[lowIndex];
    const double upperZ = zValues[highIndex];

    QVector<Eigen::Vector3d> selected;
    selected.reserve(std::min(candidates.size(), maxCount > 0 ? maxCount : candidates.size()));
    for (const Eigen::Vector3d& point : candidates)
    {
        if (point.z() >= lowerZ - 0.2 && point.z() <= upperZ + 0.2)
        {
            selected.push_back(point);
        }
    }

    if (selected.size() < 8)
    {
        // 选出过少时上下各放宽 5 个百分点重选。
        const int fallbackLowIndex = std::clamp(
            static_cast<int>(std::floor(lastIndex * std::max(0.0, lowRatio - 0.05))),
            0,
            lastIndex);
        const int fallbackHighIndex = std::clamp(
            static_cast<int>(std::floor(lastIndex * std::min(1.0, highRatio + 0.05))),
            fallbackLowIndex,
            lastIndex);
        selected.clear();
        const double fallbackLowerZ = zValues[fallbackLowIndex];
        const double fallbackUpperZ = zValues[fallbackHighIndex];
        for (const Eigen::Vector3d& point : candidates)
        {
            if (point.z() >= fallbackLowerZ - 0.2 && point.z() <= fallbackUpperZ + 0.2)
            {
                selected.push_back(point);
            }
        }
    }

    return DownsampleWorkpieceProjectionCandidates(selected, maxCount);
}

struct WorkpieceProjectionSeedCandidates
{
    RobotCalculation::IndexedPoint3D seed;
    QVector<Eigen::Vector3d> candidates;
};

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

struct GeometryMedianProfilePoint
{
    double s = 0.0;
    double smoothH = 0.0;
    double smoothN = 0.0;
    int projectedIndex = 0;
};

Eigen::Vector2d GeometryProfileProjection2D(const GeometryMedianProfilePoint& point)
{
    return Eigen::Vector2d(point.s, point.smoothH);
}

QVector<GeometryMedianProfilePoint> BuildGeometryMedianFitProfile(
    const QVector<GeometryProjectedPoint>& projected,
    const RobotCalculation::LowerWeldFilterParams& params)
{
    QVector<GeometryMedianProfilePoint> profile;
    if (projected.isEmpty())
    {
        return profile;
    }

    QVector<int> sortedIndexes;
    sortedIndexes.reserve(projected.size());
    for (int index = 0; index < projected.size(); ++index)
    {
        sortedIndexes.push_back(index);
    }
    std::sort(sortedIndexes.begin(), sortedIndexes.end(),
        [&projected](int left, int right)
        {
            return projected[left].s < projected[right].s;
        });

    const double binWidth = std::max(0.5, std::min(1.0, params.sampleStep * 0.5));
    for (int cursor = 0; cursor < sortedIndexes.size();)
    {
        const int begin = cursor;
        const double binStart = projected[sortedIndexes[cursor]].s;
        while (cursor < sortedIndexes.size()
            && projected[sortedIndexes[cursor]].s <= binStart + binWidth)
        {
            ++cursor;
        }

        QVector<double> stationValues;
        QVector<double> sideValues;
        QVector<double> normalValues;
        stationValues.reserve(cursor - begin);
        sideValues.reserve(cursor - begin);
        normalValues.reserve(cursor - begin);
        for (int item = begin; item < cursor; ++item)
        {
            const GeometryProjectedPoint& point = projected[sortedIndexes[item]];
            stationValues.push_back(point.s);
            sideValues.push_back(point.smoothH);
            normalValues.push_back(point.smoothN);
        }

        GeometryMedianProfilePoint profilePoint;
        profilePoint.s = GeometryMedianScalar(stationValues);
        profilePoint.smoothH = GeometryMedianScalar(sideValues);
        profilePoint.smoothN = GeometryMedianScalar(normalValues);

        int bestIndex = sortedIndexes[begin];
        double bestScore = std::numeric_limits<double>::infinity();
        for (int item = begin; item < cursor; ++item)
        {
            const int projectedIndex = sortedIndexes[item];
            const double score =
                std::abs(projected[projectedIndex].s - profilePoint.s)
                + std::abs(projected[projectedIndex].smoothH - profilePoint.smoothH);
            if (score < bestScore)
            {
                bestScore = score;
                bestIndex = projectedIndex;
            }
        }
        profilePoint.projectedIndex = bestIndex;
        profile.push_back(profilePoint);
    }

    return profile;
}

GeometryFittedLine2D FitGeometryProfileLine(
    const QVector<GeometryMedianProfilePoint>& profile,
    int firstIndex,
    int lastIndex)
{
    GeometryFittedLine2D line;
    if (profile.isEmpty())
    {
        return line;
    }

    const int begin = std::max(0, std::min(firstIndex, lastIndex));
    const int end = std::min(static_cast<int>(profile.size()) - 1, std::max(firstIndex, lastIndex));
    const int pointCount = end - begin + 1;
    if (pointCount < 2)
    {
        return line;
    }

    Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
    for (int index = begin; index <= end; ++index)
    {
        centroid += GeometryProfileProjection2D(profile[index]);
    }
    centroid /= static_cast<double>(pointCount);

    Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
    for (int index = begin; index <= end; ++index)
    {
        const Eigen::Vector2d delta = GeometryProfileProjection2D(profile[index]) - centroid;
        covariance += delta * delta.transpose();
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(covariance);
    if (solver.info() == Eigen::Success)
    {
        line.direction = solver.eigenvectors().col(1);
    }
    else
    {
        line.direction = GeometryProfileProjection2D(profile[end]) - GeometryProfileProjection2D(profile[begin]);
    }

    if (line.direction.squaredNorm() <= std::numeric_limits<double>::epsilon())
    {
        return line;
    }
    line.direction.normalize();

    const Eigen::Vector2d segmentDirection =
        GeometryProfileProjection2D(profile[end]) - GeometryProfileProjection2D(profile[begin]);
    if (line.direction.dot(segmentDirection) < 0.0)
    {
        line.direction = -line.direction;
    }

    line.point = centroid;
    line.valid = true;
    return line;
}

void CollectRobustSegmentedGeometryProfileKeys(
    const QVector<GeometryMedianProfilePoint>& profile,
    int first,
    int last,
    const RobotCalculation::LowerWeldFilterParams& params,
    QVector<char>& keep)
{
    const int pointCount = last - first + 1;
    const int minSegmentPointCount = std::max(8, params.piecewiseMinSegmentPoints);
    const double minSegmentLengthMm = std::max(18.0, params.sampleStep * 9.0);
    if (pointCount < minSegmentPointCount * 2)
    {
        return;
    }
    if (std::abs(profile[last].s - profile[first].s) < minSegmentLengthMm * 2.0)
    {
        return;
    }

    const GeometryFittedLine2D line = FitGeometryProfileLine(profile, first, last);
    if (!line.valid)
    {
        return;
    }

    const int edgeGap = std::max(1, std::min(4, pointCount / 10));
    QVector<double> residuals;
    QVector<int> residualIndexes;
    residuals.reserve(pointCount);
    residualIndexes.reserve(pointCount);
    double maxResidual = 0.0;
    int maxResidualIndex = -1;
    for (int index = first + edgeGap; index <= last - edgeGap; ++index)
    {
        const double residual = GeometryPointLineDistance2D(GeometryProfileProjection2D(profile[index]), line);
        if (!std::isfinite(residual))
        {
            continue;
        }
        residuals.push_back(residual);
        residualIndexes.push_back(index);
        if (residual > maxResidual)
        {
            maxResidual = residual;
            maxResidualIndex = index;
        }
    }

    if (residuals.isEmpty() || maxResidualIndex <= first || maxResidualIndex >= last)
    {
        return;
    }

    const double robustDeviation = GeometryPercentileScalar(residuals, 0.85);
    const double tolerance = std::max(1.5, params.piecewiseFitTolerance * 0.5);
    if (robustDeviation <= tolerance || maxResidual <= tolerance * 1.2)
    {
        return;
    }

    if (maxResidualIndex - first + 1 < minSegmentPointCount
        || last - maxResidualIndex + 1 < minSegmentPointCount
        || std::abs(profile[maxResidualIndex].s - profile[first].s) < minSegmentLengthMm
        || std::abs(profile[last].s - profile[maxResidualIndex].s) < minSegmentLengthMm)
    {
        return;
    }

    keep[maxResidualIndex] = 1;
    CollectRobustSegmentedGeometryProfileKeys(profile, first, maxResidualIndex, params, keep);
    CollectRobustSegmentedGeometryProfileKeys(profile, maxResidualIndex, last, params, keep);
}

QVector<int> BuildRobustSegmentedGeometryKeyIndexes(
    const QVector<GeometryProjectedPoint>& projected,
    const RobotCalculation::LowerWeldFilterParams& params)
{
    QVector<int> keyIndexes;
    if (projected.size() < std::max(16, params.piecewiseMinSegmentPoints * 2))
    {
        return BuildGeometryKeyIndexes(projected, params);
    }

    const QVector<GeometryMedianProfilePoint> profile = BuildGeometryMedianFitProfile(projected, params);
    if (profile.size() < std::max(8, params.piecewiseMinSegmentPoints * 2))
    {
        return BuildGeometryKeyIndexes(projected, params);
    }

    QVector<char> keep(profile.size(), 0);
    keep[0] = 1;
    keep[profile.size() - 1] = 1;
    CollectRobustSegmentedGeometryProfileKeys(profile, 0, profile.size() - 1, params, keep);

    keyIndexes.reserve(profile.size());
    keyIndexes.push_back(0);
    for (int profileIndex = 0; profileIndex < profile.size(); ++profileIndex)
    {
        if (keep[profileIndex])
        {
            keyIndexes.push_back(profile[profileIndex].projectedIndex);
        }
    }
    keyIndexes.push_back(projected.size() - 1);
    std::sort(keyIndexes.begin(), keyIndexes.end());
    keyIndexes.erase(std::unique(keyIndexes.begin(), keyIndexes.end()), keyIndexes.end());

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
        return BuildGeometryKeyIndexes(projected, params);
    }
    return keyIndexes;
}

QVector<GeometryProjectedPoint> RemoveGeometrySlopeWaveOutliers(
    const QVector<GeometryProjectedPoint>& projected,
    const RobotCalculation::LowerWeldFilterParams& params,
    int* rejectedCount)
{
    if (rejectedCount != nullptr)
    {
        *rejectedCount = 0;
    }
    if (projected.size() < 24)
    {
        return projected;
    }

    const QVector<int> coarseKeys = BuildGeometryKeyIndexes(projected, params);
    if (coarseKeys.size() < 2)
    {
        return projected;
    }

    QVector<char> keep(projected.size(), 1);
    const double slopeMinAbsSlope = 0.35;
    const double cornerGuardMm = std::max(6.0, params.sampleStep * 3.0);
    const double minSlopeSegmentLengthMm = std::max(18.0, params.sampleStep * 9.0);
    const double residualBaseMm = 0.8;
    const double residualMadK = 3.0;
    const int minCorePointCount = std::max(8, params.piecewiseMinSegmentPoints);

    for (int segment = 0; segment + 1 < coarseKeys.size(); ++segment)
    {
        const int left = coarseKeys[segment];
        const int right = coarseKeys[segment + 1];
        if (right <= left)
        {
            continue;
        }

        const double ds = projected[right].s - projected[left].s;
        if (std::abs(ds) < minSlopeSegmentLengthMm)
        {
            continue;
        }

        const double roughSlope = (projected[right].smoothH - projected[left].smoothH) / ds;
        if (std::abs(roughSlope) < slopeMinAbsSlope)
        {
            continue;
        }

        const double minS = std::min(projected[left].s, projected[right].s) + cornerGuardMm;
        const double maxS = std::max(projected[left].s, projected[right].s) - cornerGuardMm;
        if (maxS <= minS)
        {
            continue;
        }

        int coreBegin = -1;
        int coreEnd = -1;
        QVector<int> coreIndexes;
        coreIndexes.reserve(right - left + 1);
        for (int index = left; index <= right; ++index)
        {
            if (projected[index].s < minS || projected[index].s > maxS)
            {
                continue;
            }
            if (coreBegin < 0)
            {
                coreBegin = index;
            }
            coreEnd = index;
            coreIndexes.push_back(index);
        }

        if (coreIndexes.size() < minCorePointCount || coreBegin < 0 || coreEnd <= coreBegin)
        {
            continue;
        }

        const GeometryFittedLine2D line = FitGeometryRobustSegmentLine(projected, coreBegin, coreEnd);
        if (!line.valid)
        {
            continue;
        }

        QVector<double> residuals;
        residuals.reserve(coreIndexes.size());
        for (int index : coreIndexes)
        {
            residuals.push_back(GeometryPointLineDistance2D(GeometrySmoothedProjection2D(projected[index]), line));
        }
        if (residuals.size() < minCorePointCount)
        {
            continue;
        }

        const double medianResidual = GeometryMedianScalar(residuals);
        QVector<double> deviations;
        deviations.reserve(residuals.size());
        for (double residual : residuals)
        {
            deviations.push_back(std::abs(residual - medianResidual));
        }
        const double sigma = 1.4826 * GeometryMedianScalar(deviations);
        const double threshold = std::max(residualBaseMm, residualMadK * sigma);

        for (int index : coreIndexes)
        {
            const double residual =
                GeometryPointLineDistance2D(GeometrySmoothedProjection2D(projected[index]), line);
            if (residual > medianResidual + threshold)
            {
                keep[index] = 0;
            }
        }
    }

    int totalRejected = 0;
    for (char item : keep)
    {
        if (!item)
        {
            ++totalRejected;
        }
    }
    if (totalRejected <= 0 || projected.size() - totalRejected < 16)
    {
        return projected;
    }

    QVector<GeometryProjectedPoint> filtered;
    filtered.reserve(projected.size() - totalRejected);
    for (int index = 0; index < projected.size(); ++index)
    {
        if (keep[index])
        {
            filtered.push_back(projected[index]);
        }
    }
    SmoothGeometryProjectedPoints(&filtered, params.smoothRadius);

    if (rejectedCount != nullptr)
    {
        *rejectedCount = totalRejected;
    }
    return filtered;
}

GeometryFittedLine2D FitGeometryIndexedLine(
    const QVector<GeometryProjectedPoint>& projected,
    const QVector<int>& indexes,
    const Eigen::Vector2d& directionHint)
{
    GeometryFittedLine2D line;
    if (projected.isEmpty() || indexes.size() < 2)
    {
        return line;
    }

    auto fitIndexes = [&](const QVector<int>& fitIndexes) -> GeometryFittedLine2D
    {
        GeometryFittedLine2D fitted;
        if (fitIndexes.size() < 2)
        {
            return fitted;
        }

        Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
        for (int index : fitIndexes)
        {
            centroid += GeometrySmoothedProjection2D(projected[index]);
        }
        centroid /= static_cast<double>(fitIndexes.size());

        Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
        for (int index : fitIndexes)
        {
            const Eigen::Vector2d delta = GeometrySmoothedProjection2D(projected[index]) - centroid;
            covariance += delta * delta.transpose();
        }

        Eigen::Vector2d direction = directionHint;
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(covariance);
        if (solver.info() == Eigen::Success)
        {
            direction = solver.eigenvectors().col(1);
        }
        if (direction.squaredNorm() <= std::numeric_limits<double>::epsilon())
        {
            direction = GeometrySmoothedProjection2D(projected[fitIndexes.last()])
                - GeometrySmoothedProjection2D(projected[fitIndexes.first()]);
        }
        if (direction.squaredNorm() <= std::numeric_limits<double>::epsilon())
        {
            return fitted;
        }
        if (directionHint.squaredNorm() > std::numeric_limits<double>::epsilon())
        {
            direction = OrientGeometryDirection(direction, directionHint);
        }
        else
        {
            direction.normalize();
        }

        fitted.point = centroid;
        fitted.direction = direction.normalized();
        fitted.valid = true;
        return fitted;
    };

    QVector<int> validIndexes;
    validIndexes.reserve(indexes.size());
    for (int index : indexes)
    {
        if (index >= 0 && index < projected.size())
        {
            validIndexes.push_back(index);
        }
    }
    if (validIndexes.size() < 2)
    {
        return line;
    }

    line = fitIndexes(validIndexes);
    if (!line.valid || validIndexes.size() < 8)
    {
        return line;
    }

    QVector<double> residuals;
    residuals.reserve(validIndexes.size());
    for (int index : validIndexes)
    {
        residuals.push_back(GeometryPointLineDistance2D(GeometrySmoothedProjection2D(projected[index]), line));
    }
    const double medianResidual = GeometryMedianScalar(residuals);
    QVector<double> deviations;
    deviations.reserve(residuals.size());
    for (double residual : residuals)
    {
        deviations.push_back(std::abs(residual - medianResidual));
    }
    const double madResidual = GeometryMedianScalar(deviations);
    const double residualLimit = medianResidual + std::max(0.8, madResidual * 3.5);

    QVector<int> acceptedIndexes;
    acceptedIndexes.reserve(validIndexes.size());
    for (int index : validIndexes)
    {
        const double residual =
            GeometryPointLineDistance2D(GeometrySmoothedProjection2D(projected[index]), line);
        if (residual <= residualLimit)
        {
            acceptedIndexes.push_back(index);
        }
    }
    if (acceptedIndexes.size() >= std::max(4, static_cast<int>(validIndexes.size()) / 2))
    {
        const GeometryFittedLine2D refinedLine = fitIndexes(acceptedIndexes);
        if (refinedLine.valid)
        {
            line = refinedLine;
        }
    }

    return line;
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

GeometryFittedLine2D FitGeometrySlopeConsistentSegmentCoreLine(
    const QVector<GeometryProjectedPoint>& projected,
    int firstIndex,
    int lastIndex)
{
    if (projected.isEmpty())
    {
        return GeometryFittedLine2D();
    }

    const int begin = std::max(0, std::min(firstIndex, lastIndex));
    const int end = std::min(static_cast<int>(projected.size()) - 1, std::max(firstIndex, lastIndex));
    const GeometryFittedLine2D fallbackLine = FitGeometryRobustSegmentLine(projected, begin, end);
    const int pointCount = end - begin + 1;
    if (pointCount < 8)
    {
        return fallbackLine;
    }

    Eigen::Vector2d segmentDirection =
        GeometrySmoothedProjection2D(projected[end]) - GeometrySmoothedProjection2D(projected[begin]);
    if (segmentDirection.squaredNorm() <= std::numeric_limits<double>::epsilon())
    {
        segmentDirection = fallbackLine.direction;
    }
    if (segmentDirection.squaredNorm() <= std::numeric_limits<double>::epsilon())
    {
        return fallbackLine;
    }
    segmentDirection.normalize();

    struct SlopeWindow
    {
        Eigen::Vector2d direction = Eigen::Vector2d::Zero();
        double weight = 1.0;
        int begin = 0;
        int end = 0;
    };

    QVector<SlopeWindow> windows;
    const int windowPoints = std::max(5, std::min(24, std::max(5, pointCount / 5)));
    const int stepPoints = std::max(2, windowPoints / 2);
    for (int windowBegin = begin; windowBegin <= end - 3; windowBegin += stepPoints)
    {
        const int windowEnd = std::min(end, windowBegin + windowPoints - 1);
        if (windowEnd - windowBegin + 1 < 4)
        {
            continue;
        }

        const GeometryFittedLine2D windowLine = FitGeometrySegmentLine(projected, windowBegin, windowEnd);
        if (!windowLine.valid)
        {
            continue;
        }

        SlopeWindow window;
        window.direction = OrientGeometryDirection(windowLine.direction, segmentDirection);
        window.weight = std::max(1.0, std::abs(projected[windowEnd].s - projected[windowBegin].s));
        window.begin = windowBegin;
        window.end = windowEnd;
        windows.push_back(window);
    }
    if (windows.size() < 2)
    {
        return fallbackLine;
    }

    const double kPi = 3.14159265358979323846;
    const double angleLimit = 14.0 * kPi / 180.0;
    int bestWindowIndex = -1;
    double bestWeight = -1.0;
    for (int candidateIndex = 0; candidateIndex < windows.size(); ++candidateIndex)
    {
        double clusterWeight = 0.0;
        for (const SlopeWindow& window : windows)
        {
            if (GeometryLineDirectionAngle(window.direction, windows[candidateIndex].direction) <= angleLimit)
            {
                clusterWeight += window.weight;
            }
        }
        if (clusterWeight > bestWeight)
        {
            bestWeight = clusterWeight;
            bestWindowIndex = candidateIndex;
        }
    }
    if (bestWindowIndex < 0)
    {
        return fallbackLine;
    }

    const double totalSpan = std::abs(projected[end].s - projected[begin].s);
    // 加大排圆弧力度：上限放宽到覆盖大圆弧拐角；启用门槛从 4× 降到 2×，凹槽底部短平台等短段也排两端
    // 圆弧，不再整段把拐角圆弧吃进直线方向。若剔后核心点/跨度不足，下方 minAccepted 检查会回退兜底。
    const double cornerGuard = std::max(4.0, std::min(24.0, totalSpan * 0.18));
    const bool enableCornerGuard = totalSpan >= cornerGuard * 2.0;
    QVector<char> accepted(pointCount, 0);
    for (const SlopeWindow& window : windows)
    {
        if (GeometryLineDirectionAngle(window.direction, windows[bestWindowIndex].direction) > angleLimit)
        {
            continue;
        }
        for (int index = window.begin; index <= window.end; ++index)
        {
            if (enableCornerGuard
                && (std::abs(projected[index].s - projected[begin].s) < cornerGuard
                    || std::abs(projected[index].s - projected[end].s) < cornerGuard))
            {
                continue;
            }
            accepted[index - begin] = 1;
        }
    }

    QVector<int> acceptedIndexes;
    acceptedIndexes.reserve(pointCount);
    for (int index = begin; index <= end; ++index)
    {
        if (accepted[index - begin])
        {
            acceptedIndexes.push_back(index);
        }
    }

    const int minAcceptedCount = std::max(5, std::min(16, pointCount / 3));
    const double minAcceptedSpan = std::max(8.0, std::min(28.0, totalSpan * 0.30));
    const double acceptedSpan = acceptedIndexes.size() >= 2
        ? std::abs(projected[acceptedIndexes.last()].s - projected[acceptedIndexes.first()].s)
        : 0.0;
    if (acceptedIndexes.size() < minAcceptedCount || acceptedSpan < minAcceptedSpan)
    {
        return fallbackLine;
    }

    const GeometryFittedLine2D coreLine = FitGeometryIndexedLine(projected, acceptedIndexes, segmentDirection);
    return coreLine.valid ? coreLine : fallbackLine;
}

QVector<GeometryFittedLine2D> BuildGeometrySegmentLines(
    const QVector<GeometryProjectedPoint>& projected,
    const QVector<int>& keyIndexes,
    bool useSlopeConsistentCore)
{
    if (!useSlopeConsistentCore)
    {
        return BuildRobustGeometrySegmentLines(projected, keyIndexes);
    }

    QVector<GeometryFittedLine2D> segmentLines;
    if (keyIndexes.size() < 2)
    {
        return segmentLines;
    }

    segmentLines.reserve(keyIndexes.size() - 1);
    for (int index = 0; index + 1 < keyIndexes.size(); ++index)
    {
        segmentLines.push_back(FitGeometrySlopeConsistentSegmentCoreLine(
            projected,
            keyIndexes[index],
            keyIndexes[index + 1]));
    }
    return segmentLines;
}

GeometryFittedLine2D FitGeometryLocalSegmentLineWithSpan(
    const QVector<GeometryProjectedPoint>& projected,
    int firstIndex,
    int lastIndex,
    bool useLastSide,
    double localSpan,
    bool useSlopeConsistentCore)
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

    const GeometryFittedLine2D fallbackLine = FitGeometryRobustSegmentLine(projected, begin, end);
    if (!useSlopeConsistentCore)
    {
        return fallbackLine;
    }

    const int pointCount = end - begin + 1;
    if (pointCount < 8)
    {
        return fallbackLine;
    }

    Eigen::Vector2d segmentDirection =
        GeometrySmoothedProjection2D(projected[end]) - GeometrySmoothedProjection2D(projected[begin]);
    if (segmentDirection.squaredNorm() <= std::numeric_limits<double>::epsilon())
    {
        segmentDirection = fallbackLine.direction;
    }
    if (segmentDirection.squaredNorm() <= std::numeric_limits<double>::epsilon())
    {
        return fallbackLine;
    }
    segmentDirection.normalize();

    struct SlopeWindow
    {
        Eigen::Vector2d direction = Eigen::Vector2d::Zero();
        double weight = 1.0;
        int begin = 0;
        int end = 0;
    };

    QVector<SlopeWindow> windows;
    const int windowPoints = std::max(5, std::min(18, std::max(5, pointCount / 4)));
    const int stepPoints = std::max(2, windowPoints / 2);
    for (int windowBegin = begin; windowBegin <= end - 3; windowBegin += stepPoints)
    {
        const int windowEnd = std::min(end, windowBegin + windowPoints - 1);
        if (windowEnd - windowBegin + 1 < 4)
        {
            continue;
        }

        const GeometryFittedLine2D windowLine = FitGeometrySegmentLine(projected, windowBegin, windowEnd);
        if (!windowLine.valid)
        {
            continue;
        }

        SlopeWindow window;
        window.direction = OrientGeometryDirection(windowLine.direction, segmentDirection);
        window.weight = std::max(1.0, std::abs(projected[windowEnd].s - projected[windowBegin].s));
        window.begin = windowBegin;
        window.end = windowEnd;
        windows.push_back(window);
    }
    if (windows.size() < 2)
    {
        return fallbackLine;
    }

    const double kPi = 3.14159265358979323846;
    const double angleLimit = 14.0 * kPi / 180.0;
    int bestWindowIndex = -1;
    double bestWeight = -1.0;
    for (int candidateIndex = 0; candidateIndex < windows.size(); ++candidateIndex)
    {
        double clusterWeight = 0.0;
        for (const SlopeWindow& window : windows)
        {
            if (GeometryLineDirectionAngle(window.direction, windows[candidateIndex].direction) <= angleLimit)
            {
                clusterWeight += window.weight;
            }
        }
        if (clusterWeight > bestWeight)
        {
            bestWeight = clusterWeight;
            bestWindowIndex = candidateIndex;
        }
    }
    if (bestWindowIndex < 0)
    {
        return fallbackLine;
    }

    auto buildAcceptedIndexes = [&](bool useCornerGuard) -> QVector<int>
    {
        QVector<char> accepted(pointCount, 0);
        const double cornerS = useLastSide ? projected[end].s : projected[begin].s;
        const double cornerGuard = std::max(3.0, std::min(10.0, totalSpan * 0.12));
        const bool enableCornerGuard = useCornerGuard && totalSpan >= cornerGuard * 3.0;
        for (const SlopeWindow& window : windows)
        {
            if (GeometryLineDirectionAngle(window.direction, windows[bestWindowIndex].direction) > angleLimit)
            {
                continue;
            }
            for (int index = window.begin; index <= window.end; ++index)
            {
                if (enableCornerGuard && std::abs(projected[index].s - cornerS) < cornerGuard)
                {
                    continue;
                }
                accepted[index - begin] = 1;
            }
        }

        QVector<int> indexes;
        indexes.reserve(pointCount);
        for (int index = begin; index <= end; ++index)
        {
            if (accepted[index - begin])
            {
                indexes.push_back(index);
            }
        }
        return indexes;
    };

    QVector<int> acceptedIndexes = buildAcceptedIndexes(true);
    const int minAcceptedCount = std::max(5, std::min(12, pointCount / 3));
    const double minAcceptedSpan = std::max(6.0, std::min(18.0, totalSpan * 0.25));
    auto acceptedSpan = [&](const QVector<int>& indexes) -> double
    {
        if (indexes.size() < 2)
        {
            return 0.0;
        }
        return std::abs(projected[indexes.last()].s - projected[indexes.first()].s);
    };
    if (acceptedIndexes.size() < minAcceptedCount || acceptedSpan(acceptedIndexes) < minAcceptedSpan)
    {
        acceptedIndexes = buildAcceptedIndexes(false);
    }
    if (acceptedIndexes.size() < minAcceptedCount || acceptedSpan(acceptedIndexes) < minAcceptedSpan)
    {
        return fallbackLine;
    }

    const GeometryFittedLine2D slopeConsistentLine =
        FitGeometryIndexedLine(projected, acceptedIndexes, segmentDirection);
    return slopeConsistentLine.valid ? slopeConsistentLine : fallbackLine;
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
    const Eigen::Vector2d& reference,
    bool cornerApexMode = false)
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

    // 顶点模式(排圆弧开关开、求真实折弯顶点)放宽允许偏移：两直线延长交点离平滑种子本就更远。
    const double maxCornerShift = cornerApexMode
        ? std::max(12.0, std::min(50.0, span * 0.30))
        : std::max(5.0, std::min(18.0, span * 0.14));
    if ((candidate - reference).norm() > maxCornerShift)
    {
        return false;
    }

    // “离最近点云太远”与“偏离主带高度”这两道闸只在非顶点模式生效：凹槽/坡口的真实折弯顶点
    // 本就离圆弧点云最远、且高度偏离平台主带，顶点模式下用它们会把正确交点误杀、再退回落在圆弧上的种子。
    // 顶点模式改由 s 范围(margin) + 放宽后的 maxCornerShift 兜底，防止交点离谱跑飞。
    if (!cornerApexMode)
    {
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
    }

    return true;
}

// 直线在指定主轴 s 处的侧向高度 h（GeometryFittedLine2D 为点+方向式；近垂直线退化取点 y）。
double GeometryLineHeightAtStation(const GeometryFittedLine2D& line, double station)
{
    if (std::abs(line.direction.x()) < 1e-9) return line.point.y();
    const double t = (station - line.point.x()) / line.direction.x();
    return line.point.y() + t * line.direction.y();
}

// 检测焊缝中心线上的板材搭接 X 错位台阶（双侧平台最小二乘判据，多判据对抗验证后选定，零误检）：
// 候选过渡中心 s0=(s[i]+s[i+1])/2，两侧各留 gap 后取长 W 的窗口对 h=k*s+b 做最小二乘。
// 判错位需同时满足：两侧斜率 |kL|,|kR|<kmax（近零斜率平台——这是与拐角斜边 |k|≈0.35 的核心区别，
// 裕度高一个量级），两侧残差 rms<rmax（平整，排波纹/噪声），中心处两线高度差 |hR-hL|>jump（台阶高）。
// 沿主轴 NMS 把过渡区多候选与过冲毛刺塌缩为单点。一律用原始 h（不用 smoothH：均值平滑会把 ~2mm
// 窄跨度台阶展宽削平，削弱可检幅度导致漏检）。返回每个错位「过渡右侧第一点」的 projected 下标(升序)。
QVector<int> DetectLapMisalignmentBoundaries(
    const QVector<GeometryProjectedPoint>& projected,
    const RobotCalculation::LowerWeldFilterParams& params)
{
    QVector<int> result;
    const int n = static_cast<int>(projected.size());
    if (n < 12) return result;

    const double jump = std::max(0.3, params.lapStepHeightThresholdMm);   // 台阶高门(中心两线高度差)
    const double W = std::max(2.0, params.lapStepStationWindowMm);         // 单侧拟合窗口长度(主轴 mm)
    const double rmax = std::max(0.02, params.lapStepSideFlatnessMm);      // 平台残差 rms 门(排波纹/噪声)
    const double kmax = std::max(0.02, params.lapStepPlatformSlopeMax);    // 平台斜率门(排拐角斜边)
    const double gap = 1.5;            // 过渡保护带：拟合窗避开过渡区，不吃进台阶上升段
    const double nmsSep = 4.0;         // 沿主轴 NMS 抑制半径
    const int minSidePts = 4;          // 每侧最少有效点，不足则跳过该候选

    // 对索引列表 idxs 上的 h=k*s+b 做最小二乘，输出斜率 k、截距 b、残差 rms。
    auto fitLine = [&](const QVector<int>& idxs, double& k, double& b, double& rms) -> bool {
        const int m = idxs.size();
        if (m < 2) return false;
        double Ss = 0.0, Sh = 0.0, Sss = 0.0, Ssh = 0.0;
        for (int j : idxs) { const double s = projected[j].s, h = projected[j].h; Ss += s; Sh += h; Sss += s*s; Ssh += s*h; }
        const double den = m * Sss - Ss * Ss;
        if (std::abs(den) < 1e-9) { k = 0.0; b = Sh / m; }   // 近垂直退化：按常数处理
        else { k = (m * Ssh - Ss * Sh) / den; b = (Sh - k * Ss) / m; }
        double e2 = 0.0;
        for (int j : idxs) { const double d = projected[j].h - (k * projected[j].s + b); e2 += d*d; }
        rms = std::sqrt(e2 / m);
        return true;
    };

    struct Cand { int idx; double s0; double step; };
    QVector<Cand> cands;
    for (int i = 0; i + 1 < n; ++i)
    {
        const double s0 = 0.5 * (projected[i].s + projected[i + 1].s);   // 候选过渡中心(相邻两点之间)
        QVector<int> L, R;                                               // 按 |s-s0| 选两侧窗口点(不依赖方向/等距)
        for (int j = 0; j < n; ++j)
        {
            const double d = projected[j].s - s0;
            if (d <= -gap && d >= -(gap + W)) L.push_back(j);
            else if (d >= gap && d <= (gap + W)) R.push_back(j);
        }
        if (L.size() < minSidePts || R.size() < minSidePts) continue;
        double kL, bL, rL, kR, bR, rR;
        if (!fitLine(L, kL, bL, rL) || !fitLine(R, kR, bR, rR)) continue;
        if (std::abs(kL) > kmax || std::abs(kR) > kmax) continue;        // 两侧必须近零斜率平台(排拐角斜边)
        if (rL > rmax || rR > rmax) continue;                            // 两侧必须平整(排波纹/噪声)
        const double step = std::abs((kR * s0 + bR) - (kL * s0 + bL));   // 中心处两拟合线高度差=台阶高
        if (step < jump) continue;                                       // 台阶高门
        cands.push_back({ i + 1, s0, step });                            // 报过渡右侧第一点
    }
    if (cands.isEmpty()) return result;

    // 沿主轴 NMS：|Δs0|<nmsSep 视为同簇，保留 step 最大者(吸收过渡多候选 + 过冲毛刺，单台阶只出 1 个中心)。
    std::sort(cands.begin(), cands.end(), [](const Cand& a, const Cand& b) { return a.step > b.step; });
    QVector<Cand> kept;
    for (const Cand& c : cands)
    {
        bool suppressed = false;
        for (const Cand& k : kept)
            if (std::abs(k.s0 - c.s0) < nmsSep) { suppressed = true; break; }
        if (!suppressed) kept.push_back(c);
    }
    for (const Cand& c : kept) result.push_back(c.idx);
    std::sort(result.begin(), result.end());
    return result;
}

QVector<Eigen::Vector3d> BuildFittedGeometryKeyPoints(
    const QVector<GeometryProjectedPoint>& projected,
    const QVector<int>& keyIndexes,
    const Eigen::Vector3d& center,
    const Eigen::Vector3d& mainAxis,
    const Eigen::Vector3d& sideAxis,
    const Eigen::Vector3d& normalAxis,
    bool useSlopeConsistentCornerFit,
    const QVector<char>& isLapStepKey)
{
    QVector<Eigen::Vector3d> keyPoints;
    keyPoints.reserve(keyIndexes.size());
    if (keyIndexes.isEmpty())
    {
        return keyPoints;
    }

    const QVector<GeometryFittedLine2D> segmentLines =
        BuildGeometrySegmentLines(projected, keyIndexes, useSlopeConsistentCornerFit);

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
            // 搭接错位台阶端点：两侧线平行错开、求交无意义，直接取本侧段直线在本点 s 的值，
            // 形成干净 X 台阶。台阶对的左点(右邻也是台阶点)用左段线(板1)、右点用右段线(板2)。
            if (index < isLapStepKey.size() && isLapStepKey[index] != 0)
            {
                const double stepS = GeometrySmoothedProjection2D(projected[keyIndex]).x();
                const bool isLeftOfPair = (index + 1 < isLapStepKey.size() && isLapStepKey[index + 1] != 0);
                const int lineIdx = isLeftOfPair ? (index - 1) : index;
                const Eigen::Vector2d sideProj = (lineIdx >= 0 && lineIdx < segmentLines.size())
                    ? Eigen::Vector2d(stepS, GeometryLineHeightAtStation(segmentLines[lineIdx], stepS))
                    : GeometrySmoothedProjection2D(projected[keyIndex]);
                keyPoints.push_back(GeometryPointFromProjection(
                    center, mainAxis, sideAxis, normalAxis, sideProj, projected[keyIndex].smoothN));
                continue;
            }
            const double previousS = projected[keyIndexes[index - 1]].s;
            const double nextS = projected[keyIndexes[index + 1]].s;
            const double span = std::max(1.0, std::abs(nextS - previousS));
            bool hasBestIntersection = false;
            double bestScore = std::numeric_limits<double>::infinity();
            Eigen::Vector2d bestIntersection = projection;
            auto considerIntersection = [&](const Eigen::Vector2d& candidate, double shiftWeight)
            {
                if (!IsGeometryCornerProjectionUsable(projected, keyIndexes, index, candidate, projection, useSlopeConsistentCornerFit))
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

            if (!useSlopeConsistentCornerFit)
            {
                for (double localSpan : candidateSpans)
                {
                    const GeometryFittedLine2D leftLine = FitGeometryLocalSegmentLineWithSpan(
                        projected,
                        keyIndexes[index - 1],
                        keyIndex,
                        true,
                        localSpan,
                        false);
                    const GeometryFittedLine2D rightLine = FitGeometryLocalSegmentLineWithSpan(
                        projected,
                        keyIndex,
                        keyIndexes[index + 1],
                        false,
                        localSpan,
                        false);
                    Eigen::Vector2d intersection = projection;
                    if (IntersectGeometryLines(leftLine, rightLine, &intersection))
                    {
                        considerIntersection(intersection, 0.15);
                    }
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
            QVector<double> segmentDistances;
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
                segmentDistances.push_back(distance);
                if (distance > maxDistance)
                {
                    maxDistance = distance;
                    maxDistanceIndex = sampleIndex;
                }
            }

            const double triggerDistance =
                params.geometryStrategy == RobotCalculation::LowerWeldGeometryStrategy::LegacyGeometry
                    ? maxDistance
                    : GeometryPercentileScalar(segmentDistances, 0.85);
            if (maxDistanceIndex > begin
                && maxDistanceIndex < end
                && triggerDistance > maxSegmentDeviation)
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
    const Eigen::Vector3d& normalAxis,
    bool useSlopeConsistentCornerFit)
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
            normalAxis,
            useSlopeConsistentCornerFit,
            QVector<char>());
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

// ===== 方位角拐点检测（输入已去噪时替代 DP，由 inputAlreadyDenoised 分支调用）=====
// 在 (s, smoothH) 投影面上，对每点用前后各 K 点的局部最小二乘拟合方向求带符号转角；转角超阈的
// 点经"同区域保留转角最大"NMS 作为拐角。相比 DP 取"离弦最远点"（缓变/台阶段拐点偏移、平滑后漏检
// 真折角、调容差又两头不靠），方位角峰值天然落在真折角顶点，按转折角度判定能区分真折角与波纹起伏。
QVector<int> BuildAzimuthCornerKeyIndexes(
    const QVector<GeometryProjectedPoint>& projected,
    const RobotCalculation::LowerWeldFilterParams& params)
{
    QVector<int> keys;
    const int n = static_cast<int>(projected.size());
    if (n < 2)
    {
        if (n == 1) keys.push_back(0);
        return keys;
    }

    const int k = std::max(2, params.azimuthHeadingWindow);
    constexpr double kPi = 3.14159265358979323846;
    const double turnThreshold = std::max(1.0, params.azimuthTurnThresholdDeg) * kPi / 180.0;
    const double nmsSpan = std::max(1.0, params.azimuthNmsSpanMm);
    const int guard = k;

    QVector<double> signedTurn(n, 0.0);
    for (int i = guard; i < n - guard; ++i)
    {
        // 局部最小二乘拟合左右两段方向（比两点差分更抗波纹板侧向小起伏），方向对齐到焊缝前进向
        //（拟合方向符号任意，用端点差分参考向量定号）。
        Eigen::Vector2d leftDir = FitGeometrySegmentLine(projected, i - k, i).direction;
        Eigen::Vector2d rightDir = FitGeometrySegmentLine(projected, i, i + k).direction;
        const Eigen::Vector2d leftRef =
            GeometrySmoothedProjection2D(projected[i]) - GeometrySmoothedProjection2D(projected[i - k]);
        const Eigen::Vector2d rightRef =
            GeometrySmoothedProjection2D(projected[i + k]) - GeometrySmoothedProjection2D(projected[i]);
        if (leftDir.squaredNorm() <= 1e-12 || rightDir.squaredNorm() <= 1e-12) continue;
        if (leftDir.dot(leftRef) < 0.0) leftDir = -leftDir;
        if (rightDir.dot(rightRef) < 0.0) rightDir = -rightDir;
        leftDir.normalize();
        rightDir.normalize();
        signedTurn[i] = std::atan2(Cross2D(leftDir, rightDir), leftDir.dot(rightDir));
    }

    QVector<int> candidates;
    for (int i = guard; i < n - guard; ++i)
    {
        if (std::abs(signedTurn[i]) >= turnThreshold) candidates.push_back(i);
    }
    // 非极大值抑制：按 |转角| 降序贪心选取，抑制主轴弧长距离 < nmsSpan 的其它候选。
    std::sort(candidates.begin(), candidates.end(),
        [&](int a, int b) { return std::abs(signedTurn[a]) > std::abs(signedTurn[b]); });
    QVector<int> corners;
    for (int idx : candidates)
    {
        bool suppressed = false;
        for (int kept : corners)
        {
            if (std::abs(projected[idx].s - projected[kept].s) < nmsSpan) { suppressed = true; break; }
        }
        if (!suppressed) corners.push_back(idx);
    }
    std::sort(corners.begin(), corners.end());

    const double minSegMm = std::max(params.sampleStep * 2.0, 2.0);
    keys.push_back(0);
    for (int idx : corners)
    {
        if (idx <= 0 || idx >= n - 1) continue;
        if (std::abs(projected[idx].s - projected[keys.last()].s) < minSegMm) continue;
        keys.push_back(idx);
    }
    if (keys.last() != n - 1) keys.push_back(n - 1);
    return keys;
}

// 直线化兜底：某相邻关键点段内实际点偏离两端弦超阈 → 该段藏着漏检拐点，插入离弦最远点，避免
// 下游线性插值把折线连成直线(抄近路)。残差阈值须大于波纹起伏幅度，真直段不触发。
QVector<int> SplitNonStraightAzimuthSegments(
    const QVector<GeometryProjectedPoint>& projected,
    const QVector<int>& keyIndexes,
    const RobotCalculation::LowerWeldFilterParams& params)
{
    QVector<int> keys = keyIndexes;
    if (keys.size() < 2) return keys;
    const double residualThreshold = std::max(0.5, params.azimuthStraightenResidualMm);
    for (int iter = 0; iter < 8; ++iter)
    {
        QVector<int> next;
        next.push_back(keys.first());
        bool inserted = false;
        for (int kp = 0; kp + 1 < keys.size(); ++kp)
        {
            const int a = keys[kp];
            const int b = keys[kp + 1];
            double maxDist = -1.0;
            int maxIdx = -1;
            for (int i = a + 1; i < b; ++i)
            {
                const double d = GeometryDistanceToSegment2D(projected, a, b, i);
                if (d > maxDist) { maxDist = d; maxIdx = i; }
            }
            if (maxIdx > a && maxIdx < b && maxDist > residualThreshold)
            {
                next.push_back(maxIdx);
                inserted = true;
            }
            next.push_back(b);
        }
        keys = next;
        if (!inserted) break;
    }
    return keys;
}

// 起终点先验自适应细化(两条几何路径通用：方位角粗拟合后 / DP 拐点+prune 后均可调用)：对仍偏直的段
// 做按需细化。仅当段内点「一致弓向弦的同一侧」（单侧占比 >= kOneSidedMin，借此区分真弓弯与左右乱跳的
// 随机噪声）且离弦峰值超过位置相关地板时，才补该段离弦最远点为拐点。地板按主轴弧长位置加权：首尾各
// kEndFrac 端区用低地板(azimuthRefineFloorMm)，中段用高地板(×kMidMultiple)——体现「漏检拐点多出现在
// 引入/收尾段」的现场先验，避免中段把周期起伏过补成拐点。azimuthRefineFloorMm<=0 时关闭(保持原结果)。
QVector<int> RefineCornersByCoherentBow(
    const QVector<GeometryProjectedPoint>& projected,
    const QVector<int>& keyIndexes,
    const RobotCalculation::LowerWeldFilterParams& params)
{
    QVector<int> keys = keyIndexes;
    const double floorMm = params.azimuthRefineFloorMm;
    if (!params.cornerRefineEnable || floorMm <= 0.0 || keys.size() < 2 || projected.size() < 3)
    {
        return keys;  // 开关关 / 地板<=0 / 点太少 → 不细化，保持原拐点
    }
    // 门限参数化(界面可调)：单侧弓出占比门、中段地板倍数、端区弧长占比；越界由配置层钳制兜底。
    const double kOneSidedMin = std::min(1.0, std::max(0.5, params.cornerRefineOneSidedFrac));
    const double kMidMultiple = std::max(1.0, params.cornerRefineMidMultiple);
    const double kEndFrac = std::min(0.5, std::max(0.0, params.cornerRefineEndFrac));
    const double minSegMm = std::max(params.sampleStep * 2.0, 2.0);
    const double s0 = projected.first().s;
    const double sN = projected.last().s;
    const double span = std::abs(sN - s0);

    for (int iter = 0; iter < 8; ++iter)
    {
        QVector<int> next;
        next.push_back(keys.first());
        bool inserted = false;
        for (int kp = 0; kp + 1 < keys.size(); ++kp)
        {
            const int a = keys[kp];
            const int b = keys[kp + 1];
            const Eigen::Vector2d pa = GeometrySmoothedProjection2D(projected[a]);
            const Eigen::Vector2d pb = GeometrySmoothedProjection2D(projected[b]);
            const Eigen::Vector2d chord = pb - pa;
            const double chordLen = chord.norm();
            double maxDist = -1.0;
            int maxIdx = -1;
            int positiveSide = 0;
            int totalSide = 0;
            for (int i = a + 1; i < b; ++i)
            {
                const double dist = GeometryDistanceToSegment2D(projected, a, b, i);
                if (dist > maxDist)
                {
                    maxDist = dist;
                    maxIdx = i;
                }
                if (chordLen > 1e-9)
                {
                    // 点在弦的哪一侧：叉积符号；一致弓弯各点同号，随机噪声正负相抵
                    const double sideSign = Cross2D(chord, GeometrySmoothedProjection2D(projected[i]) - pa);
                    if (sideSign >= 0.0)
                    {
                        ++positiveSide;
                    }
                    ++totalSide;
                }
            }
            if (totalSide > 0 && maxIdx > a && maxIdx < b)
            {
                const double oneSided =
                    std::max(positiveSide, totalSide - positiveSide) / static_cast<double>(totalSide);
                const double frac = span > 1e-9 ? std::abs(projected[maxIdx].s - s0) / span : 0.5;
                const bool endZone = frac < kEndFrac || frac > 1.0 - kEndFrac;
                const double floorHere = endZone ? floorMm : floorMm * kMidMultiple;
                const bool farEnough =
                    std::abs(projected[maxIdx].s - projected[a].s) >= minSegMm
                    && std::abs(projected[b].s - projected[maxIdx].s) >= minSegMm;
                if (maxDist > floorHere && oneSided >= kOneSidedMin && farEnough)
                {
                    next.push_back(maxIdx);
                    inserted = true;
                }
            }
            next.push_back(b);
        }
        keys = next;
        if (!inserted)
        {
            break;
        }
    }
    return keys;
}

// 在稠密路径的一段区域内做「坡-平台-坡」三段直线拟合，返回两个折点(=平台两端角)的 projected 索引。
// 自变量 s(主轴弧长)、因变量 smoothH(侧向)。位置由几何重算而非从拐点集挑选，故对源头多检/漏检免疫。
QPair<int, int> FitPlatformTwoCorners(
    const QVector<GeometryProjectedPoint>& projected,
    double regLoS,
    double regHiS,
    int minSegPoints)
{
    const double lo = std::min(regLoS, regHiS);
    const double hi = std::max(regLoS, regHiS);
    QVector<int> reg;
    for (int i = 0; i < projected.size(); ++i)
    {
        if (projected[i].s >= lo && projected[i].s <= hi) reg.push_back(i);
    }
    std::sort(reg.begin(), reg.end(), [&](int a, int b) { return projected[a].s < projected[b].s; });
    const int m = reg.size();
    const int mp = std::max(3, minSegPoints);
    if (m < 3 * mp) return qMakePair(-1, -1);

    std::vector<double> Sx(m + 1, 0.0), Sy(m + 1, 0.0), Sxx(m + 1, 0.0), Sxy(m + 1, 0.0), Syy(m + 1, 0.0);
    for (int k = 0; k < m; ++k)
    {
        const double x = projected[reg[k]].s;
        const double y = projected[reg[k]].smoothH;
        Sx[k + 1] = Sx[k] + x;       Sy[k + 1] = Sy[k] + y;
        Sxx[k + 1] = Sxx[k] + x * x; Sxy[k + 1] = Sxy[k] + x * y; Syy[k + 1] = Syy[k] + y * y;
    }
    auto res = [&](int i, int j) -> double {  // 点 [i,j] 的最小二乘直线拟合残差平方和(O(1))
        const int c = j - i + 1;
        if (c < 2) return 0.0;
        const double sx = Sx[j + 1] - Sx[i], sy = Sy[j + 1] - Sy[i];
        const double sxx = Sxx[j + 1] - Sxx[i], sxy = Sxy[j + 1] - Sxy[i], syy = Syy[j + 1] - Syy[i];
        const double sxxc = sxx - sx * sx / c, sxyc = sxy - sx * sy / c, syyc = syy - sy * sy / c;
        if (sxxc <= 1e-12) return syyc;
        return std::max(0.0, syyc - sxyc * sxyc / sxxc);
    };
    double best = std::numeric_limits<double>::max();
    int bb1 = -1, bb2 = -1;
    for (int b1 = mp; b1 < m - 2 * mp; ++b1)
    {
        const double r1 = res(0, b1);
        for (int b2 = b1 + mp; b2 < m - mp; ++b2)
        {
            const double tot = r1 + res(b1, b2) + res(b2, m - 1);
            if (tot < best) { best = tot; bb1 = b1; bb2 = b2; }
        }
    }
    if (bb1 < 0 || bb2 < 0) return qMakePair(-1, -1);
    return qMakePair(reg[bb1], reg[bb2]);
}

// 拐点 inner/outer 结构约束(平台重算)：波纹角成对出现(谷=II、峰=OO，搭接台阶点豁免)。按类型游程把拐点
// 归成平台(源头多检只把游程拉长、不改平台数与交替，故对多检稳)，每个平台在 [上一平台末角, 下一平台首角]
// 区域用 FitPlatformTwoCorners 从稠密路径重算恰好 2 个边界角——多检(5→2/3→2)删对、缺点(1→2)补齐，一套
// 覆盖。位置全由几何重算，绕开被过检污染的 prominence/间距。lap 角原样保留并按 s 归并。<=0/关 则不动。
QVector<int> RefitCornersByPlatformPattern(
    const QVector<GeometryProjectedPoint>& projected,
    const QVector<int>& keyIndexes,
    const QVector<char>& isLapStepKey,
    const RobotCalculation::LowerWeldFilterParams& params)
{
    const int m = keyIndexes.size();
    if (!params.cornerPatternRefitEnable || m < 4 || projected.size() < 24)
    {
        return keyIndexes;
    }
    const double startS = projected[keyIndexes.first()].s;
    const double endS = projected[keyIndexes.last()].s;

    // 自然拐点(非端点、非 lap)的类型(+1 inner / -1 outer)与 s
    struct NatCorner { int keyPos; int type; double s; };
    QVector<NatCorner> nat;
    for (int k = 1; k < m - 1; ++k)
    {
        if (k < isLapStepKey.size() && isLapStepKey[k]) continue;  // 搭接台阶点豁免
        const double prom = projected[keyIndexes[k]].smoothH
            - (projected[keyIndexes[k - 1]].smoothH + projected[keyIndexes[k + 1]].smoothH) * 0.5;
        nat.push_back({ k, prom >= 0.0 ? 1 : -1, projected[keyIndexes[k]].s });
    }
    if (nat.size() < 2)
    {
        return keyIndexes;  // 自然拐点太少，不足以谈成对规律
    }

    // 按类型游程分平台
    QVector<QPair<int, int>> runs;  // [i,j] into nat
    for (int i = 0; i < nat.size(); )
    {
        int j = i;
        while (j + 1 < nat.size() && nat[j + 1].type == nat[i].type) ++j;
        runs.push_back(qMakePair(i, j));
        i = j + 1;
    }

    QVector<int> refit;  // 重算后的自然拐点 projected 索引
    for (int r = 0; r < runs.size(); ++r)
    {
        const int i = runs[r].first, j = runs[r].second;
        const double regLoS = (r > 0) ? nat[runs[r - 1].second].s : startS;
        const double regHiS = (r + 1 < runs.size()) ? nat[runs[r + 1].first].s : endS;
        const QPair<int, int> two = FitPlatformTwoCorners(
            projected, regLoS, regHiS, std::max(3, params.cornerPlatformMinSegPoints));
        if (two.first >= 0 && two.second >= 0 && two.first != two.second)
        {
            refit.push_back(two.first);
            refit.push_back(two.second);
        }
        else
        {
            for (int t = i; t <= j; ++t) refit.push_back(keyIndexes[nat[t].keyPos]);  // 拟合失败回退保留原角
        }
    }

    // 重建：起点 + 重算自然角 + lap 角(原样) + 终点，按 s 升序去重
    QVector<int> result;
    result.push_back(keyIndexes.first());
    result += refit;
    for (int k = 1; k < m - 1; ++k)
    {
        if (k < isLapStepKey.size() && isLapStepKey[k]) result.push_back(keyIndexes[k]);
    }
    result.push_back(keyIndexes.last());
    const bool ascending = endS >= startS;
    std::sort(result.begin(), result.end(), [&](int a, int b) {
        return ascending ? projected[a].s < projected[b].s : projected[a].s > projected[b].s;
    });
    result.erase(std::unique(result.begin(), result.end()), result.end());
    return result.size() >= 2 ? result : keyIndexes;
}

// 端区某点的"航向弯折角"(度)：前后各 win 点在 (s,smoothH) 平面各拟合一条直线，取两方向夹角(锐角)。
// 直段≈0°，真拐角≈典型角。用于在"周期反推"的预测窗口内确认是否真有拐点。
double GeometryBendAngleDegAt(
    const QVector<GeometryProjectedPoint>& projected,
    int idx,
    int win)
{
    const int n = projected.size();
    if (idx <= 0 || idx >= n - 1)
    {
        return 0.0;
    }
    const int a = std::max(0, idx - win);
    const int b = std::min(n - 1, idx + win);
    const GeometryFittedLine2D before = FitGeometrySegmentLine(projected, a, idx);
    const GeometryFittedLine2D after = FitGeometrySegmentLine(projected, idx, b);
    if (!before.valid || !after.valid)
    {
        return 0.0;
    }
    double dot = std::abs(before.direction.dot(after.direction));
    dot = std::min(1.0, std::max(0.0, dot));
    constexpr double kPiRad = 3.14159265358979323846;
    return std::acos(dot) * 180.0 / kPiRad;
}

// 端区周期一致性补拐点(波纹板专用)：波纹拐点近似周期。
//   长度判据：中段可靠拐点的"中位段长 L"=周期；端区(起点段/终点段)长度 ≥ ratio*L → 里面漏了拐点。
//   角度判据：典型拐角弯折角 A(中段中位)；按周期反推漏点位置后，在 ±窗口内找弯折角峰，
//             峰值 ≥ max(minBendDeg, 0.5*A) 才认(直段≈0°不会误补)。长度定位+角度确认双判据。
//   搭接：相邻两个 isLapStepKey 拐点合成一段(两侧相加)再算周期，否则搭接短段污染统计。
// 只"插入"非搭接拐点、不动现有拐点；返回新的 keyIndexes(保持原 s 升/降序去重)。insertedCount 回传新增数。
QVector<int> RecoverEndRegionCornersByPeriod(
    const QVector<GeometryProjectedPoint>& projected,
    const QVector<int>& keyIndexes,
    const QVector<char>& isLapStepKey,
    double ratioThreshold,
    double minBendDeg,
    int* insertedCount)
{
    if (insertedCount != nullptr)
    {
        *insertedCount = 0;
    }
    const int m = keyIndexes.size();
    if (m < 5 || projected.size() < 16)
    {
        return keyIndexes;  // 拐点/点数太少，不足以估周期
    }

    // 1) 搭接对合成：相邻两个 lap step 拐点合成一个"代表点"(中点弧长)。
    std::vector<double> repS;
    repS.reserve(m);
    for (int k = 0; k < m; )
    {
        if (k + 1 < m && k + 1 < isLapStepKey.size() && isLapStepKey[k] && isLapStepKey[k + 1])
        {
            repS.push_back(0.5 * (projected[keyIndexes[k]].s + projected[keyIndexes[k + 1]].s));
            k += 2;
        }
        else
        {
            repS.push_back(projected[keyIndexes[k]].s);
            ++k;
        }
    }
    const int mm = static_cast<int>(repS.size());
    if (mm < 4)
    {
        return keyIndexes;  // 需 起点 + >=2 中段 + 终点
    }

    // 2) 中段段长(排除首段 rep0->1、末段 rep[mm-2]->[mm-1]) 中位数 = 周期 L。
    std::vector<double> interiorSeg;
    for (int k = 1; k <= mm - 3; ++k)
    {
        interiorSeg.push_back(std::abs(repS[k + 1] - repS[k]));
    }
    if (interiorSeg.empty())
    {
        return keyIndexes;
    }
    std::sort(interiorSeg.begin(), interiorSeg.end());
    const double L = interiorSeg[interiorSeg.size() / 2];
    if (L <= 2.0)
    {
        return keyIndexes;
    }

    // 3) 典型拐角弯折角 A = 中段(非搭接)拐点弯折角中位数 → 确认阈值 acceptBend。
    const int bendWin = 10;
    std::vector<double> interiorBend;
    for (int k = 1; k < m - 1; ++k)
    {
        if (k < isLapStepKey.size() && isLapStepKey[k])
        {
            continue;
        }
        const double bend = GeometryBendAngleDegAt(projected, keyIndexes[k], bendWin);
        if (bend > 0.0)
        {
            interiorBend.push_back(bend);
        }
    }
    double typicalBend = minBendDeg;
    if (!interiorBend.empty())
    {
        std::sort(interiorBend.begin(), interiorBend.end());
        typicalBend = interiorBend[interiorBend.size() / 2];
    }
    const double acceptBend = std::max(minBendDeg, 0.5 * typicalBend);

    // 在弧长窗口内找弯折角最大且 ≥acceptBend 的 projected 索引。
    auto findCornerInWindow = [&](double sCenter, double halfWin) -> int
    {
        int bestIdx = -1;
        double bestBend = acceptBend;
        for (int i = bendWin; i + bendWin < projected.size(); ++i)
        {
            if (std::abs(projected[i].s - sCenter) > halfWin)
            {
                continue;
            }
            const double bend = GeometryBendAngleDegAt(projected, i, bendWin);
            if (bend > bestBend)
            {
                bestBend = bend;
                bestIdx = i;
            }
        }
        return bestIdx;
    };

    QVector<int> result = keyIndexes;
    int inserted = 0;

    // 对一个端段[sNear(端点侧)..sFar(内侧拐点)]：长度≥ratio*L 则按周期从内侧拐点往端点反推补点。
    auto tryRecoverEndSegment = [&](double sNear, double sFar)
    {
        const double segLen = std::abs(sFar - sNear);
        if (segLen < ratioThreshold * L)
        {
            return;  // 段长不足(部分周期收尾)，正常不补
        }
        const double dir = (sFar >= sNear) ? 1.0 : -1.0;
        const double sLo = std::min(sNear, sFar) + 0.15 * L;
        const double sHi = std::max(sNear, sFar) - 0.15 * L;
        const int maxMissing = static_cast<int>(segLen / L) + 1;
        for (int j = 1; j <= maxMissing; ++j)
        {
            const double sPred = sFar - dir * j * L;
            if (sPred < sLo || sPred > sHi)
            {
                continue;
            }
            const int idx = findCornerInWindow(sPred, 0.35 * L);
            if (idx < 0 || result.contains(idx))
            {
                continue;
            }
            bool tooClose = false;
            for (int existing : result)
            {
                if (std::abs(projected[existing].s - projected[idx].s) < 0.15 * L)
                {
                    tooClose = true;
                    break;
                }
            }
            if (!tooClose)
            {
                result.push_back(idx);
                ++inserted;
            }
        }
    };

    tryRecoverEndSegment(repS[0], repS[1]);              // 起点段
    tryRecoverEndSegment(repS[mm - 1], repS[mm - 2]);    // 终点段

    if (inserted > 0)
    {
        const bool ascending = projected[keyIndexes.last()].s >= projected[keyIndexes.first()].s;
        std::sort(result.begin(), result.end(), [&](int a, int b) {
            return ascending ? projected[a].s < projected[b].s : projected[a].s > projected[b].s;
        });
        result.erase(std::unique(result.begin(), result.end()), result.end());
    }
    if (insertedCount != nullptr)
    {
        *insertedCount = inserted;
    }
    return result;
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

double GeometryKeyDistance(
    const QVector<GeometryProjectedPoint>& projected,
    const QVector<int>& keyIndexes,
    int firstPosition,
    int secondPosition)
{
    if (firstPosition < 0
        || secondPosition < 0
        || firstPosition >= keyIndexes.size()
        || secondPosition >= keyIndexes.size())
    {
        return std::numeric_limits<double>::infinity();
    }

    return (projected[keyIndexes[secondPosition]].point - projected[keyIndexes[firstPosition]].point).norm();
}

// 周期/角度删错拐点：相邻【同类】拐点弧长间距 ≪ 周期L(< distFrac*L)说明其中一个找错——波纹同类拐角(II/OO 各
// 界定一个平台)间距≈一个周期，挨太近多由平台重算把一个平台的两个边界角凑到一起所致。合并成一个：删弯折角离
// 典型角更远的那个。搭接台阶对(相邻两个 isLapStepKey)豁免(本就该挨着)。端点不动。只删不插、保持原序。
QVector<int> MergeTooCloseSameTypeCorners(
    const QVector<GeometryProjectedPoint>& projected,
    const QVector<int>& keyIndexes,
    const QVector<char>& isLapStepKey,
    double distFrac,
    int* removedCount)
{
    if (removedCount != nullptr) { *removedCount = 0; }
    const int m0 = keyIndexes.size();
    if (m0 < 5) { return keyIndexes; }

    // 周期 L = 中段段长中位(搭接合成、排除首末段)——与端区补漏 Pass 同口径。
    std::vector<double> repS;
    for (int k = 0; k < m0; )
    {
        if (k + 1 < m0 && k + 1 < isLapStepKey.size() && isLapStepKey[k] && isLapStepKey[k + 1])
        {
            repS.push_back(0.5 * (projected[keyIndexes[k]].s + projected[keyIndexes[k + 1]].s));
            k += 2;
        }
        else
        {
            repS.push_back(projected[keyIndexes[k]].s);
            ++k;
        }
    }
    const int mm = static_cast<int>(repS.size());
    if (mm < 4) { return keyIndexes; }
    std::vector<double> seg;
    for (int k = 1; k <= mm - 3; ++k) { seg.push_back(std::abs(repS[k + 1] - repS[k])); }
    if (seg.empty()) { return keyIndexes; }
    std::sort(seg.begin(), seg.end());
    const double L = seg[seg.size() / 2];
    if (L <= 2.0) { return keyIndexes; }
    const double minGap = std::max(0.05, distFrac) * L;

    const int bendWin = 10;
    std::vector<double> ib;
    for (int k = 1; k < m0 - 1; ++k)
    {
        if (k < isLapStepKey.size() && isLapStepKey[k]) { continue; }
        const double b = GeometryBendAngleDegAt(projected, keyIndexes[k], bendWin);
        if (b > 0.0) { ib.push_back(b); }
    }
    double typical = 8.0;
    if (!ib.empty()) { std::sort(ib.begin(), ib.end()); typical = ib[ib.size() / 2]; }

    QVector<int> result = keyIndexes;
    QVector<char> lapKey = isLapStepKey;
    // 段 [posA,posB] 的侧向斜率(|ΔsmoothH/Δs|)：平台≈平(小)、上/下坡≈陡(大)。用来分辨"真平台"与"坡上假双拐点"。
    const double flatSlopeThresh = 0.15;  // <此≈平台，≥此≈坡(波纹平台斜率≈0.03~0.1，坡≈0.3~0.4)
    auto segSlope = [&](int posA, int posB) -> double
    {
        const double ds = projected[result[posB]].s - projected[result[posA]].s;
        if (std::abs(ds) < 1e-6) { return 0.0; }
        return std::abs((projected[result[posB]].smoothH - projected[result[posA]].smoothH) / ds);
    };
    int removed = 0;
    bool changed = true;
    while (changed && result.size() > 4)
    {
        changed = false;
        for (int k = 1; k + 1 <= result.size() - 2; ++k)  // 仅中段相邻对(k,k+1 均为内部拐点)
        {
            // 搭接对豁免
            if ((k < lapKey.size() && lapKey[k]) || (k + 1 < lapKey.size() && lapKey[k + 1])) { continue; }
            // 必须同类
            if (GeometryCornerType(projected, result, k) != GeometryCornerType(projected, result, k + 1)) { continue; }
            // 间距(主轴弧长)≥ minGap 则正常，不删
            const double gap = std::abs(projected[result[k]].s - projected[result[k + 1]].s);
            if (gap >= minGap) { continue; }
            // 【平台保护】两同类拐点【之间】若是平的(flat)→这是一段真平台(哪怕很短)，两端都是真边界，绝不删。
            // 只有当它们之间是【坡】(斜率陡)时，才说明一条坡上多切了一个拐点(假双拐点)，才删。
            if (segSlope(k, k + 1) < flatSlopeThresh) { continue; }
            // 删"坡中"那个(两侧都陡)，保留贴着平台的边界角(另一侧是平台)。两者都/都不像坡中→按弯折角离典型远近。
            const bool kMidEdge = (k - 1 >= 0) && segSlope(k - 1, k) >= flatSlopeThresh;
            const bool k1MidEdge = (k + 2 < result.size()) && segSlope(k + 1, k + 2) >= flatSlopeThresh;
            int removeAt;
            if (kMidEdge && !k1MidEdge) { removeAt = k; }
            else if (k1MidEdge && !kMidEdge) { removeAt = k + 1; }
            else
            {
                const double bk = GeometryBendAngleDegAt(projected, result[k], bendWin);
                const double bk1 = GeometryBendAngleDegAt(projected, result[k + 1], bendWin);
                removeAt = (std::abs(bk - typical) >= std::abs(bk1 - typical)) ? k : (k + 1);
            }
            result.removeAt(removeAt);
            lapKey.removeAt(removeAt);
            ++removed;
            changed = true;
            break;  // 索引已变，重扫
        }
    }
    if (removedCount != nullptr) { *removedCount = removed; }
    return result;
}

// 点 idx 处侧向局部斜率 |dh/ds|（窗口 ±win 点最小二乘拟合）。平台≈0(平)，上/下坡≈陡。
double LocalLateralSlopeAt(const QVector<GeometryProjectedPoint>& projected, int idx, int win)
{
    const int n = projected.size();
    const int a = std::max(0, idx - win);
    const int b = std::min(n - 1, idx + win);
    const int cnt = b - a + 1;
    if (cnt < 3) { return 0.0; }
    double sx = 0.0, sy = 0.0, sxx = 0.0, sxy = 0.0;
    for (int k = a; k <= b; ++k)
    {
        const double x = projected[k].s;
        const double y = projected[k].smoothH;
        sx += x; sy += y; sxx += x * x; sxy += x * y;
    }
    const double denom = static_cast<double>(cnt) * sxx - sx * sx;
    if (std::abs(denom) < 1e-9) { return 0.0; }
    return std::abs((static_cast<double>(cnt) * sxy - sx * sy) / denom);
}

// 按平台边界重定拐点：波纹的拐点全部位于"平台↔坡"交界。本 Pass 沿焊道检测"平的段(平台)"，
// 对每个平台保证两端各有一个边界拐点、删掉卡在平台内部(放错位)的拐点。判定针对原拐点集逐平台进行：
// 平台两端都已有角且内部无角 → 该平台正确，原样不动(幂等)；否则补缺失边界角、删内部误检角。
// 治"拐点被放到平台正中间→平台塌成长坡消失""端区漏平台边界角"。搭接拐点与起终点不参与，保持不变。
QVector<int> SnapCornersToPlatforms(
    const QVector<GeometryProjectedPoint>& projected,
    const QVector<int>& keyIndexes,
    const QVector<char>& isLapStepKey,
    double flatSlopeThresh,
    double minPlatformFrac,
    int* changedCount)
{
    if (changedCount != nullptr) { *changedCount = 0; }
    const int m0 = keyIndexes.size();
    const int n = projected.size();
    if (m0 < 4 || n < 32) { return keyIndexes; }

    // 周期 L = 中段段长中位(搭接对合成到中点)——与其它端区 Pass 同口径
    std::vector<double> repS;
    for (int k = 0; k < m0; )
    {
        if (k + 1 < m0 && k + 1 < isLapStepKey.size() && isLapStepKey[k] && isLapStepKey[k + 1])
        { repS.push_back(0.5 * (projected[keyIndexes[k]].s + projected[keyIndexes[k + 1]].s)); k += 2; }
        else { repS.push_back(projected[keyIndexes[k]].s); ++k; }
    }
    const int mm = static_cast<int>(repS.size());
    if (mm < 4) { return keyIndexes; }
    std::vector<double> seg;
    for (int k = 1; k <= mm - 3; ++k) { seg.push_back(std::abs(repS[k + 1] - repS[k])); }
    if (seg.empty()) { return keyIndexes; }
    std::sort(seg.begin(), seg.end());
    const double L = seg[seg.size() / 2];
    if (L <= 2.0) { return keyIndexes; }

    // 斜率窗 ≈ 7mm（按主轴点间距换算）
    const double span = std::abs(projected[n - 1].s - projected[0].s);
    const double spacing = span > 1e-6 ? span / (n - 1) : 1.0;
    const int win = std::min(40, std::max(6, static_cast<int>(std::lround(7.0 / std::max(0.05, spacing)))));

    // 检测平台运行(flat run)：连续的平点；长度≥minPlatLen 才算真平台
    std::vector<char> flat(n, 0);
    for (int i = 0; i < n; ++i)
    {
        flat[i] = LocalLateralSlopeAt(projected, i, win) < flatSlopeThresh ? 1 : 0;
    }
    const double minPlatLen = std::max(10.0, minPlatformFrac * L);
    std::vector<std::pair<int, int>> plats;
    for (int i = 0; i < n; )
    {
        if (!flat[i]) { ++i; continue; }
        int j = i;
        while (j + 1 < n && flat[j + 1]) { ++j; }
        if (std::abs(projected[j].s - projected[i].s) >= minPlatLen) { plats.emplace_back(i, j); }
        i = j + 1;
    }
    if (plats.empty()) { return keyIndexes; }

    // 逐平台判定（针对原 keyIndexes，统一收集删/插后一次性应用，避免中途索引漂移）
    const double marc = std::max(10.0, 0.18 * L);  // 把"已在平台边界附近"的现有角视作边界角的容差
    std::vector<char> removeFlag(m0, 0);
    std::vector<int> insertIdx;
    for (const auto& pab : plats)
    {
        const double sa = projected[pab.first].s;
        const double sb = projected[pab.second].s;
        const double lo = std::min(sa, sb), hi = std::max(sa, sb);
        bool hasA = false, hasB = false;
        std::vector<int> interior;
        // 边界判定(hasA/hasB)：起点/终点/搭接角同样是合法平台边界，全部计入——否则会误判端区/搭接两侧平台缺边界而插入重复角。
        for (int k = 0; k < m0; ++k)
        {
            const double sc = projected[keyIndexes[k]].s;
            if (std::abs(sc - sa) <= marc) { hasA = true; }
            if (std::abs(sc - sb) <= marc) { hasB = true; }
        }
        // 内部放错角(可删)：仅限常规中段角(非起终点、非搭接)。
        for (int k = 1; k < m0 - 1; ++k)
        {
            if (k < isLapStepKey.size() && isLapStepKey[k]) { continue; }
            const double sc = projected[keyIndexes[k]].s;
            if (sc > lo + marc && sc < hi - marc) { interior.push_back(k); }
        }
        if (interior.empty() && hasA && hasB) { continue; }  // 该平台已正确，不动
        for (int k : interior) { removeFlag[k] = 1; }         // 删平台内部放错的角
        if (!hasA) { insertIdx.push_back(pab.first); }        // 补缺失的平台边界角
        if (!hasB) { insertIdx.push_back(pab.second); }
    }

    int changed = static_cast<int>(insertIdx.size());
    for (int k = 0; k < m0; ++k) { if (removeFlag[k]) { ++changed; } }
    if (changed == 0) { return keyIndexes; }

    QVector<int> result;
    for (int k = 0; k < m0; ++k) { if (!removeFlag[k]) { result.push_back(keyIndexes[k]); } }
    for (int idx : insertIdx) { if (!result.contains(idx)) { result.push_back(idx); } }
    const bool ascending = projected[keyIndexes.last()].s >= projected[keyIndexes.first()].s;
    std::sort(result.begin(), result.end(), [&](int a, int b) {
        return ascending ? projected[a].s < projected[b].s : projected[a].s > projected[b].s;
    });
    result.erase(std::unique(result.begin(), result.end()), result.end());
    if (changedCount != nullptr) { *changedCount = changed; }
    return result;
}

QVector<int> PruneShortSameTypeGeometryRuns(
    const QVector<GeometryProjectedPoint>& projected,
    QVector<int> keyIndexes,
    const RobotCalculation::LowerWeldFilterParams& params)
{
    if (projected.isEmpty() || keyIndexes.size() <= 4)
    {
        return keyIndexes;
    }

    const double shortSameTypeSpan = std::max(
        24.0,
        std::max(
            params.sampleStep > 0.0 ? params.sampleStep * 12.0 : 24.0,
            params.segmentBreakDistance > 0.0 ? params.segmentBreakDistance * 1.5 : 18.0));

    bool removed = true;
    while (removed && keyIndexes.size() > 4)
    {
        removed = false;
        for (int runBegin = 1; runBegin + 2 < keyIndexes.size() - 1 && !removed;)
        {
            const RobotCalculation::LowerWeldPointType runType =
                GeometryCornerType(projected, keyIndexes, runBegin);
            if (runType == RobotCalculation::LowerWeldPointType::Start
                || runType == RobotCalculation::LowerWeldPointType::End)
            {
                ++runBegin;
                continue;
            }

            int runEnd = runBegin;
            while (runEnd + 1 < keyIndexes.size() - 1
                && GeometryCornerType(projected, keyIndexes, runEnd + 1) == runType)
            {
                ++runEnd;
            }

            if (runEnd - runBegin + 1 < 3)
            {
                runBegin = runEnd + 1;
                continue;
            }

            for (int position = runBegin; position < runEnd; ++position)
            {
                const double span = GeometryKeyDistance(projected, keyIndexes, position, position + 1);
                if (span >= shortSameTypeSpan)
                {
                    continue;
                }

                int removePosition = -1;
                if (position == runBegin)
                {
                    const double nextSpan = GeometryKeyDistance(projected, keyIndexes, position + 1, position + 2);
                    if (nextSpan >= shortSameTypeSpan)
                    {
                        removePosition = position;
                    }
                }
                else if (position + 1 == runEnd)
                {
                    const double previousSpan = GeometryKeyDistance(projected, keyIndexes, position - 1, position);
                    if (previousSpan >= shortSameTypeSpan)
                    {
                        removePosition = position + 1;
                    }
                }
                else
                {
                    const double previousSpan = GeometryKeyDistance(projected, keyIndexes, position - 1, position);
                    const double nextSpan = GeometryKeyDistance(projected, keyIndexes, position + 1, position + 2);
                    removePosition = previousSpan >= nextSpan ? position + 1 : position;
                }

                if (removePosition > 0 && removePosition < keyIndexes.size() - 1)
                {
                    keyIndexes.removeAt(removePosition);
                    removed = true;
                    break;
                }
            }

            if (!removed)
            {
                runBegin = runEnd + 1;
            }
        }
    }

    return keyIndexes;
}

double GeometryPolylineLength(const QVector<Eigen::Vector3d>& points)
{
    double length = 0.0;
    for (int index = 1; index < points.size(); ++index)
    {
        length += (points[index] - points[index - 1]).norm();
    }
    return length;
}

double GeometryProjectedSpan(const QVector<GeometryProjectedPoint>& projected, double* minStation = nullptr, double* maxStation = nullptr)
{
    if (projected.isEmpty())
    {
        if (minStation != nullptr)
        {
            *minStation = 0.0;
        }
        if (maxStation != nullptr)
        {
            *maxStation = 0.0;
        }
        return 0.0;
    }

    double minValue = projected.first().s;
    double maxValue = projected.first().s;
    for (const GeometryProjectedPoint& point : projected)
    {
        minValue = std::min(minValue, point.s);
        maxValue = std::max(maxValue, point.s);
    }
    if (minStation != nullptr)
    {
        *minStation = minValue;
    }
    if (maxStation != nullptr)
    {
        *maxStation = maxValue;
    }
    return maxValue - minValue;
}

QString PercentText(double ratio)
{
    return QString("%1%").arg(ratio * 100.0, 0, 'f', 1);
}

QVector<Eigen::Vector2d> GeometryKeyProjections(
    const QVector<Eigen::Vector3d>& keyPoints,
    const Eigen::Vector3d& center,
    const Eigen::Vector3d& mainAxis,
    const Eigen::Vector3d& sideAxis)
{
    QVector<Eigen::Vector2d> projections;
    projections.reserve(keyPoints.size());
    for (const Eigen::Vector3d& point : keyPoints)
    {
        const Eigen::Vector3d delta = point - center;
        projections.push_back(Eigen::Vector2d(delta.dot(mainAxis), delta.dot(sideAxis)));
    }
    return projections;
}

bool ValidateGeometryCoverage(
    int finitePointCount,
    double projectedSpan,
    const RobotCalculation::LowerWeldFilterParams& params,
    QString* error)
{
    if (!params.validationCoverageEnabled)
    {
        return true;
    }

    if (finitePointCount < params.validationMinFinitePointCount)
    {
        if (error != nullptr)
        {
            *error = QString("点云有效性检测失败：有效点数过少，当前 %1 个，要求至少 %2 个。")
                .arg(finitePointCount)
                .arg(params.validationMinFinitePointCount);
        }
        return false;
    }

    if (projectedSpan < params.validationMinProjectedSpanMm)
    {
        if (error != nullptr)
        {
            *error = QString("点云有效性检测失败：扫描主轴覆盖不足，当前 %1 mm，要求至少 %2 mm。")
                .arg(projectedSpan, 0, 'f', 3)
                .arg(params.validationMinProjectedSpanMm, 0, 'f', 3);
        }
        return false;
    }

    return true;
}

bool ValidateGeometryContinuity(
    const QVector<GeometryProjectedPoint>& projected,
    double minStation,
    double maxStation,
    const RobotCalculation::LowerWeldFilterParams& params,
    QString* error)
{
    if (!params.validationContinuityEnabled || projected.isEmpty())
    {
        return true;
    }

    const double span = maxStation - minStation;
    if (span <= std::numeric_limits<double>::epsilon())
    {
        return true;
    }

    const double binWidth = std::max(0.5, params.sampleStep > 0.0 ? params.sampleStep : 2.0);
    const int binCount = std::max(1, static_cast<int>(std::floor(span / binWidth)) + 1);
    QVector<char> occupied(binCount, 0);
    for (const GeometryProjectedPoint& point : projected)
    {
        const int binIndex = std::max(0, std::min(binCount - 1, static_cast<int>(std::floor((point.s - minStation) / binWidth))));
        occupied[binIndex] = 1;
    }

    int occupiedCount = 0;
    int longestRun = 0;
    int currentRun = 0;
    for (char value : occupied)
    {
        if (value)
        {
            ++occupiedCount;
            ++currentRun;
            longestRun = std::max(longestRun, currentRun);
        }
        else
        {
            currentRun = 0;
        }
    }

    const double coverageRatio = static_cast<double>(occupiedCount) / static_cast<double>(binCount);
    if (coverageRatio < params.validationMinStationCoverageRatio)
    {
        if (error != nullptr)
        {
            *error = QString("点云有效性检测失败：主轴采样覆盖率过低，当前 %1，要求至少 %2。")
                .arg(PercentText(coverageRatio))
                .arg(PercentText(params.validationMinStationCoverageRatio));
        }
        return false;
    }

    const double longestRatio = static_cast<double>(longestRun) / static_cast<double>(binCount);
    if (longestRatio < params.validationMinLongestContinuousRatio)
    {
        if (error != nullptr)
        {
            *error = QString("点云有效性检测失败：最长连续采样段过短，当前 %1，要求至少 %2。")
                .arg(PercentText(longestRatio))
                .arg(PercentText(params.validationMinLongestContinuousRatio));
        }
        return false;
    }

    return true;
}

bool ValidateGeometryRejectedRatio(
    int finitePointCount,
    int rejectedCount,
    const RobotCalculation::LowerWeldFilterParams& params,
    QString* error)
{
    if (!params.validationDenoiseRatioEnabled || finitePointCount <= 0)
    {
        return true;
    }

    const double rejectedRatio = static_cast<double>(std::max(0, rejectedCount)) / static_cast<double>(finitePointCount);
    if (rejectedRatio > params.validationMaxRejectedRatio)
    {
        if (error != nullptr)
        {
            *error = QString("点云有效性检测失败：滤波剔除比例过高，当前 %1，允许最大 %2。")
                .arg(PercentText(rejectedRatio))
                .arg(PercentText(params.validationMaxRejectedRatio));
        }
        return false;
    }

    return true;
}

bool ValidateGeometryKeyPoints(
    const QVector<RobotCalculation::LowerWeldClassifiedPoint>& keyPoints,
    const QVector<Eigen::Vector2d>& keyProjections,
    const RobotCalculation::LowerWeldFilterParams& params,
    QStringList* warnings,
    QString* error)
{
    if (!params.validationKeyPointEnabled)
    {
        return true;
    }

    if (keyPoints.size() < params.validationMinKeyPointCount)
    {
        if (error != nullptr)
        {
            *error = QString("点云有效性检测失败：起终点/拐点数量不足，当前 %1 个，要求至少 %2 个。")
                .arg(keyPoints.size())
                .arg(params.validationMinKeyPointCount);
        }
        return false;
    }

    int cornerCount = 0;
    int startCount = 0;
    int endCount = 0;
    for (const RobotCalculation::LowerWeldClassifiedPoint& point : keyPoints)
    {
        if (point.type == RobotCalculation::LowerWeldPointType::Start)
        {
            ++startCount;
        }
        else if (point.type == RobotCalculation::LowerWeldPointType::End)
        {
            ++endCount;
        }
        else if (point.type == RobotCalculation::LowerWeldPointType::InnerCorner
            || point.type == RobotCalculation::LowerWeldPointType::OuterCorner)
        {
            ++cornerCount;
        }
    }
    if (startCount != 1 || endCount != 1)
    {
        if (error != nullptr)
        {
            *error = QString("点云有效性检测失败：起点/终点必须各恰好 1 个，当前起点 %1 个、终点 %2 个。")
                .arg(startCount)
                .arg(endCount);
        }
        return false;
    }
    if (cornerCount < params.validationMinCornerCount)
    {
        if (error != nullptr)
        {
            *error = QString("点云有效性检测失败：拐点数量不足，当前 %1 个，要求至少 %2 个。")
                .arg(cornerCount)
                .arg(params.validationMinCornerCount);
        }
        return false;
    }

    for (int index = 1; index < keyPoints.size(); ++index)
    {
        const double segmentLength = (keyPoints[index].point - keyPoints[index - 1].point).norm();
        const bool isLapStepPair = keyPoints[index - 1].isLapStepBoundary
            && keyPoints[index].isLapStepBoundary;
        const bool isEndpointAdjacent =
            keyPoints[index - 1].type == RobotCalculation::LowerWeldPointType::Start
            || keyPoints[index].type == RobotCalculation::LowerWeldPointType::End;
        // SDK 直出不提供搭接语义；首尾裁剪却可能合法地产生 2.5mm 端段。
        // 因此仅“成对搭接”或“紧邻唯一端点”的段使用 0.25mm 重合硬门限，内部非搭接仍保持 3mm。
        const double hardMinimumMm = (isLapStepPair || isEndpointAdjacent) ? 0.25 : 3.0;
        if (!std::isfinite(segmentLength) || segmentLength < hardMinimumMm)
        {
            if (error != nullptr)
            {
                *error = QString("点云有效性检测失败：第 %1 段%2关键点距离无效或过短，当前 %3 mm，硬门限 %4 mm。")
                    .arg(index)
                    .arg(isLapStepPair ? "搭接台阶" : (isEndpointAdjacent ? "端点相邻" : "非搭接"))
                    .arg(segmentLength, 0, 'f', 3)
                    .arg(hardMinimumMm, 0, 'f', 3);
            }
            return false;
        }
        if (!isLapStepPair
            && params.validationMinSegmentLengthMm > hardMinimumMm
            && segmentLength < params.validationMinSegmentLengthMm
            && warnings != nullptr)
        {
            warnings->push_back(QString("第 %1 段非搭接关键点距离 %2 mm，低于审计建议值 %3 mm（硬门限仍为 %4 mm）。")
                .arg(index)
                .arg(segmentLength, 0, 'f', 3)
                .arg(params.validationMinSegmentLengthMm, 0, 'f', 3)
                .arg(hardMinimumMm, 0, 'f', 3));
        }
    }

    for (int index = 1; index < keyProjections.size(); ++index)
    {
        if (keyProjections[index].x() <= keyProjections[index - 1].x() + 1e-6)
        {
            if (error != nullptr)
            {
                *error = QString("点云有效性检测失败：拟合关键点主轴顺序不单调，第 %1 个关键点未前进。")
                    .arg(index + 1);
            }
            return false;
        }
    }

    return true;
}

bool ValidateGeometryResidual(
    const QVector<GeometryProjectedPoint>& projected,
    const QVector<Eigen::Vector2d>& keyProjections,
    const RobotCalculation::LowerWeldFilterParams& params,
    QString* error)
{
    if (!params.validationResidualEnabled || projected.isEmpty() || keyProjections.size() < 2)
    {
        return true;
    }

    QVector<double> residuals;
    residuals.reserve(projected.size());
    int inlierCount = 0;
    for (const GeometryProjectedPoint& point : projected)
    {
        double minDistance = std::numeric_limits<double>::infinity();
        const Eigen::Vector2d projection = GeometrySmoothedProjection2D(point);
        for (int segment = 0; segment + 1 < keyProjections.size(); ++segment)
        {
            minDistance = std::min(
                minDistance,
                GeometryPointToSegmentDistance2D(
                    projection,
                    keyProjections[segment],
                    keyProjections[segment + 1]));
        }
        if (!std::isfinite(minDistance))
        {
            continue;
        }
        residuals.push_back(minDistance);
        if (params.validationResidualInlierThresholdMm <= 0.0
            || minDistance <= params.validationResidualInlierThresholdMm)
        {
            ++inlierCount;
        }
    }

    if (residuals.isEmpty())
    {
        if (error != nullptr)
        {
            *error = "点云有效性检测失败：无法计算拟合残差。";
        }
        return false;
    }

    const double medianResidual = GeometryMedianScalar(residuals);
    if (params.validationMaxMedianResidualMm > 0.0
        && medianResidual > params.validationMaxMedianResidualMm)
    {
        if (error != nullptr)
        {
            *error = QString("点云有效性检测失败：拟合中位残差过大，当前 %1 mm，允许最大 %2 mm。")
                .arg(medianResidual, 0, 'f', 3)
                .arg(params.validationMaxMedianResidualMm, 0, 'f', 3);
        }
        return false;
    }

    const double p95Residual = GeometryPercentileScalar(residuals, 0.95);
    if (params.validationMaxP95ResidualMm > 0.0
        && p95Residual > params.validationMaxP95ResidualMm)
    {
        if (error != nullptr)
        {
            *error = QString("点云有效性检测失败：拟合 P95 残差过大，当前 %1 mm，允许最大 %2 mm。")
                .arg(p95Residual, 0, 'f', 3)
                .arg(params.validationMaxP95ResidualMm, 0, 'f', 3);
        }
        return false;
    }

    const double inlierRatio = static_cast<double>(inlierCount) / static_cast<double>(residuals.size());
    if (inlierRatio < params.validationMinResidualInlierRatio)
    {
        if (error != nullptr)
        {
            *error = QString("点云有效性检测失败：拟合内点比例过低，当前 %1，要求至少 %2，内点阈值 %3 mm。")
                .arg(PercentText(inlierRatio))
                .arg(PercentText(params.validationMinResidualInlierRatio))
                .arg(params.validationResidualInlierThresholdMm, 0, 'f', 3);
        }
        return false;
    }

    return true;
}

bool ValidateGeometryOutput(
    const RobotCalculation::LowerWeldClassificationResult& classification,
    const QVector<Eigen::Vector3d>& fittedKeyPoints,
    double projectedSpan,
    const RobotCalculation::LowerWeldFilterParams& params,
    QString* error)
{
    if (!params.validationOutputEnabled)
    {
        return true;
    }

    double maxOutputStepMm = 0.0;
    for (int index = 0; index < classification.points.size(); ++index)
    {
        const Eigen::Vector3d& point = classification.points[index].point;
        if (!std::isfinite(point.x()) || !std::isfinite(point.y()) || !std::isfinite(point.z()))
        {
            if (error != nullptr)
            {
                *error = QString("点云有效性检测失败：输出焊道第 %1 点包含 NaN/Inf。")
                    .arg(index + 1);
            }
            return false;
        }
        if (index > 0)
        {
            maxOutputStepMm = std::max(
                maxOutputStepMm,
                (point - classification.points[index - 1].point).norm());
        }
    }
    constexpr double kHardMaxOutputStepMm = 50.0;
    if (maxOutputStepMm > kHardMaxOutputStepMm)
    {
        if (error != nullptr)
        {
            *error = QString("点云有效性检测失败：输出焊道相邻点最大跳距 %1 mm，超过结构硬门限 %2 mm。")
                .arg(maxOutputStepMm, 0, 'f', 3)
                .arg(kHardMaxOutputStepMm, 0, 'f', 1);
        }
        return false;
    }

    if (classification.points.size() < params.validationMinOutputPointCount)
    {
        if (error != nullptr)
        {
            *error = QString("点云有效性检测失败：输出焊道点过少，当前 %1 个，要求至少 %2 个。")
                .arg(classification.points.size())
                .arg(params.validationMinOutputPointCount);
        }
        return false;
    }

    const double outputLength = GeometryPolylineLength(fittedKeyPoints);
    if (projectedSpan > std::numeric_limits<double>::epsilon()
        && params.validationMinOutputLengthRatio > 0.0)
    {
        const double lengthRatio = outputLength / projectedSpan;
        if (lengthRatio < params.validationMinOutputLengthRatio)
        {
            if (error != nullptr)
            {
                *error = QString("点云有效性检测失败：输出焊道长度与输入覆盖不匹配，当前比例 %1，要求至少 %2。")
                    .arg(PercentText(lengthRatio))
                    .arg(PercentText(params.validationMinOutputLengthRatio));
            }
            return false;
        }
    }

    return true;
}

bool ValidateGeometryAnalysisResult(
    int finitePointCount,
    int rejectedCount,
    const QVector<GeometryProjectedPoint>& projected,
    const QVector<Eigen::Vector3d>& fittedKeyPoints,
    const Eigen::Vector3d& center,
    const Eigen::Vector3d& mainAxis,
    const Eigen::Vector3d& sideAxis,
    const RobotCalculation::MeasureThenWeldAnalysisResult& result,
    const RobotCalculation::LowerWeldFilterParams& params,
    RobotCalculation::MeasureThenWeldAnalysisResult::PointCloudQualityReport* qualityReport,
    QString* error)
{
    double minStation = 0.0;
    double maxStation = 0.0;
    const double projectedSpan = GeometryProjectedSpan(projected, &minStation, &maxStation);
    const QVector<Eigen::Vector2d> keyProjections =
        GeometryKeyProjections(fittedKeyPoints, center, mainAxis, sideAxis);

    RobotCalculation::MeasureThenWeldAnalysisResult::PointCloudQualityReport report;
    report.evaluated = true;
    report.auditOnly = params.validationAuditOnly;
    report.inputPointCount = finitePointCount + std::max(0, rejectedCount);
    report.finitePointCount = finitePointCount;
    report.rejectedPointCount = std::max(0, rejectedCount);
    report.keyPointCount = result.keyPoints.size();
    report.outputPointCount = result.classificationResult.points.size();
    report.projectedSpanMm = projectedSpan;
    report.rejectedRatio = finitePointCount > 0
        ? static_cast<double>(std::max(0, rejectedCount)) / static_cast<double>(finitePointCount)
        : 0.0;

    for (const RobotCalculation::LowerWeldClassifiedPoint& point : result.keyPoints)
    {
        if (point.type == RobotCalculation::LowerWeldPointType::InnerCorner
            || point.type == RobotCalculation::LowerWeldPointType::OuterCorner)
        {
            ++report.cornerCount;
        }
    }

    if (!projected.isEmpty() && projectedSpan > std::numeric_limits<double>::epsilon())
    {
        const double binWidth = std::max(0.5, params.sampleStep > 0.0 ? params.sampleStep : 2.0);
        const int binCount = std::max(1, static_cast<int>(std::floor(projectedSpan / binWidth)) + 1);
        QVector<char> occupied(binCount, 0);
        for (const GeometryProjectedPoint& point : projected)
        {
            const int binIndex = std::max(0, std::min(
                binCount - 1,
                static_cast<int>(std::floor((point.s - minStation) / binWidth))));
            occupied[binIndex] = 1;
        }
        int occupiedCount = 0;
        int longestRun = 0;
        int currentRun = 0;
        for (char value : occupied)
        {
            if (value)
            {
                ++occupiedCount;
                longestRun = std::max(longestRun, ++currentRun);
            }
            else
            {
                currentRun = 0;
            }
        }
        report.stationCoverageRatio = static_cast<double>(occupiedCount) / static_cast<double>(binCount);
        report.longestContinuousRatio = static_cast<double>(longestRun) / static_cast<double>(binCount);
    }

    QVector<double> residuals;
    residuals.reserve(projected.size());
    int residualInlierCount = 0;
    if (keyProjections.size() >= 2)
    {
        for (const GeometryProjectedPoint& point : projected)
        {
            double minDistance = std::numeric_limits<double>::infinity();
            const Eigen::Vector2d projection = GeometrySmoothedProjection2D(point);
            for (int segment = 0; segment + 1 < keyProjections.size(); ++segment)
            {
                minDistance = std::min(
                    minDistance,
                    GeometryPointToSegmentDistance2D(
                        projection,
                        keyProjections[segment],
                        keyProjections[segment + 1]));
            }
            if (std::isfinite(minDistance))
            {
                residuals.push_back(minDistance);
                if (params.validationResidualInlierThresholdMm <= 0.0
                    || minDistance <= params.validationResidualInlierThresholdMm)
                {
                    ++residualInlierCount;
                }
            }
        }
    }
    if (!residuals.isEmpty())
    {
        report.medianResidualMm = GeometryMedianScalar(residuals);
        report.p95ResidualMm = GeometryPercentileScalar(residuals, 0.95);
        report.residualInlierRatio =
            static_cast<double>(residualInlierCount) / static_cast<double>(residuals.size());
    }

    report.minNonLapSegmentLengthMm = std::numeric_limits<double>::infinity();
    report.minLapStepSegmentLengthMm = std::numeric_limits<double>::infinity();
    for (int index = 1; index < result.keyPoints.size(); ++index)
    {
        const double length = (result.keyPoints[index].point - result.keyPoints[index - 1].point).norm();
        const bool lapPair = result.keyPoints[index - 1].isLapStepBoundary
            && result.keyPoints[index].isLapStepBoundary;
        double& minimum = lapPair
            ? report.minLapStepSegmentLengthMm
            : report.minNonLapSegmentLengthMm;
        minimum = std::min(minimum, length);
    }
    if (!std::isfinite(report.minNonLapSegmentLengthMm))
    {
        report.minNonLapSegmentLengthMm = 0.0;
    }
    if (!std::isfinite(report.minLapStepSegmentLengthMm))
    {
        report.minLapStepSegmentLengthMm = 0.0;
    }

    report.outputLengthMm = GeometryPolylineLength(fittedKeyPoints);
    report.outputLengthRatio = projectedSpan > std::numeric_limits<double>::epsilon()
        ? report.outputLengthMm / projectedSpan
        : 0.0;
    for (int index = 1; index < result.classificationResult.points.size(); ++index)
    {
        report.maxOutputStepMm = std::max(
            report.maxOutputStepMm,
            (result.classificationResult.points[index].point
                - result.classificationResult.points[index - 1].point).norm());
    }

    auto applyCheck = [&report](bool passed, const QString& checkError)
    {
        if (!passed)
        {
            report.failures.push_back(checkError.isEmpty()
                ? QStringLiteral("点云质量检查失败（未提供具体原因）。")
                : checkError);
        }
    };
    QString checkError;
    bool checkPassed = ValidateGeometryCoverage(finitePointCount, projectedSpan, params, &checkError);
    applyCheck(checkPassed, checkError);
    checkError.clear();
    checkPassed = ValidateGeometryContinuity(projected, minStation, maxStation, params, &checkError);
    applyCheck(checkPassed, checkError);
    checkError.clear();
    checkPassed = ValidateGeometryRejectedRatio(finitePointCount, rejectedCount, params, &checkError);
    applyCheck(checkPassed, checkError);
    checkError.clear();
    checkPassed = ValidateGeometryKeyPoints(result.keyPoints, keyProjections, params, &report.warnings, &checkError);
    applyCheck(checkPassed, checkError);
    checkError.clear();
    checkPassed = ValidateGeometryResidual(projected, keyProjections, params, &checkError);
    applyCheck(checkPassed, checkError);
    checkError.clear();
    checkPassed = ValidateGeometryOutput(result.classificationResult, fittedKeyPoints, projectedSpan, params, &checkError);
    applyCheck(checkPassed, checkError);

    report.passed = report.failures.isEmpty();
    if (qualityReport != nullptr)
    {
        *qualityReport = report;
    }
    if (error != nullptr)
    {
        *error = report.failures.join(QStringLiteral("；"));
    }
    return report.passed;
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


struct RawFilterPoint
{
    double sampleAxisValue = 0.0;
    Eigen::Vector3d point = Eigen::Vector3d::Zero();
    bool valid = false;
    QString source;
};












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

    // 姿态角(rx/ry/rz)按最近等价角插值：先把 b 相对 a 的差规范化到 (-180,180]，避免在 ±180 环绕处
    // 朴素线性插值插出错误中间角（如 179 与 -179 物理相邻却线性插成 0）。不跨界时结果与 lerp 完全一致。
    // 此前缺此处理：起/终点机器人姿态在 rx 抖动跨 ±180 的帧会把点云整体变换偏，在完整点云端部形成翘起散点。
    auto lerpAngleDeg = [ratio](double a, double b)
        {
            double diff = b - a;
            while (diff > 180.0) diff -= 360.0;
            while (diff < -180.0) diff += 360.0;
            return a + diff * ratio;
        };

    return T_ROBOT_COORS(
        lerp(lower->pose.dX, upper->pose.dX),
        lerp(lower->pose.dY, upper->pose.dY),
        lerp(lower->pose.dZ, upper->pose.dZ),
        lerpAngleDeg(lower->pose.dRX, upper->pose.dRX),
        lerpAngleDeg(lower->pose.dRY, upper->pose.dRY),
        lerpAngleDeg(lower->pose.dRZ, upper->pose.dRZ),
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


RobotCalculation::LowerWeldFilterResult RobotCalculation::ProjectWorkpieceCloudToLowerWeldPath(
    const QVector<IndexedPoint3D>& workpieceCloudInput,
    const QVector<IndexedPoint3D>& seedPathInput,
    const LowerWeldFilterParams& params)
{
    LowerWeldFilterResult result;
    result.inputPointCount = workpieceCloudInput.size();
    if (workpieceCloudInput.size() < 3)
    {
        result.error = QString("完整点云点数过少，仅 %1 个，无法拟合底板。")
            .arg(workpieceCloudInput.size());
        return result;
    }
    if (seedPathInput.size() < 2)
    {
        result.error = QString("种子焊道点数过少，仅 %1 个，无法保留立板波浪轮廓。")
            .arg(seedPathInput.size());
        return result;
    }

    QVector<int> sortedIndexes;
    sortedIndexes.reserve(workpieceCloudInput.size());
    for (int index = 0; index < workpieceCloudInput.size(); ++index)
    {
        if (IsFinitePoint(workpieceCloudInput[index].point))
        {
            sortedIndexes.push_back(index);
        }
    }
    std::sort(sortedIndexes.begin(), sortedIndexes.end(), [&](int left, int right)
    {
        return WorkpieceProjectionStationValue(workpieceCloudInput[left].point, params.sampleAxis)
            < WorkpieceProjectionStationValue(workpieceCloudInput[right].point, params.sampleAxis);
    });

    if (sortedIndexes.size() < 3)
    {
        result.error = "完整点云没有足够的有效点，无法拟合底板。";
        return result;
    }

    // 各窗口优先用显式投影参数；0 表示按滤波参数自动派生（原硬编码行为）。
    const double sampleStep = params.sampleStep > 0.0 ? params.sampleStep : 2.0;
    const double stationWindow = params.projectionStationWindowMm > 0.0
        ? params.projectionStationWindowMm
        : std::max(2.5, sampleStep * 1.5);
    const double transverseWindow = params.projectionTransverseWindowMm > 0.0
        ? params.projectionTransverseWindowMm
        : std::max(10.0, params.searchWindow > 0.0 ? params.searchWindow * 1.5 : 12.0);
    const double zBandBelow = params.projectionZBandBelowMm > 0.0
        ? params.projectionZBandBelowMm
        : std::max(14.0, params.zContinuityThreshold > 0.0 ? params.zContinuityThreshold * 4.0 : 14.0);
    const double zBandAbove = params.projectionZBandAboveMm > 0.0
        ? params.projectionZBandAboveMm
        : std::max(12.0, params.zJumpThreshold > 0.0 ? params.zJumpThreshold * 2.5 : 12.0);
    const int kMaxCandidatePerSeed = params.projectionMaxCandidatePerSeed > 0
        ? params.projectionMaxCandidatePerSeed
        : 160;

    auto lowerStationBound = [&](double value)
    {
        return std::lower_bound(sortedIndexes.begin(), sortedIndexes.end(), value, [&](int pointIndex, double station)
        {
            return WorkpieceProjectionStationValue(workpieceCloudInput[pointIndex].point, params.sampleAxis) < station;
        });
    };

    QVector<WorkpieceProjectionSeedCandidates> seedCandidateSets;
    seedCandidateSets.reserve(seedPathInput.size());
    QVector<Eigen::Vector3d> bottomCandidates;
    bottomCandidates.reserve(std::min(static_cast<int>(seedPathInput.size()) * kMaxCandidatePerSeed, 240000));
    int visitedLocalPointCount = 0;
    for (const IndexedPoint3D& seed : seedPathInput)
    {
        if (!IsFinitePoint(seed.point))
        {
            continue;
        }

        const double seedStation = WorkpieceProjectionStationValue(seed.point, params.sampleAxis);
        const double seedTransverse = WorkpieceProjectionTransverseValue(seed.point, params.sampleAxis);
        const auto begin = lowerStationBound(seedStation - stationWindow);
        const auto end = lowerStationBound(seedStation + stationWindow);

        QVector<Eigen::Vector3d> localCandidates;
        localCandidates.reserve(kMaxCandidatePerSeed * 2);
        for (auto it = begin; it != end; ++it)
        {
            const Eigen::Vector3d& point = workpieceCloudInput[*it].point;
            if (std::abs(WorkpieceProjectionTransverseValue(point, params.sampleAxis) - seedTransverse) > transverseWindow)
            {
                continue;
            }
            if (point.z() < seed.point.z() - zBandBelow || point.z() > seed.point.z() + zBandAbove)
            {
                continue;
            }
            localCandidates.push_back(point);
            ++visitedLocalPointCount;
        }

        WorkpieceProjectionSeedCandidates seedCandidates;
        seedCandidates.seed = seed;
        seedCandidates.candidates = SelectMiddleWorkpieceProjectionLayerCandidates(
            localCandidates,
            kMaxCandidatePerSeed,
            params.projectionLayerLowPercent,
            params.projectionLayerHighPercent);
        if (seedCandidates.candidates.isEmpty() && !localCandidates.isEmpty())
        {
            seedCandidates.candidates =
                DownsampleWorkpieceProjectionCandidates(localCandidates, kMaxCandidatePerSeed);
        }
        bottomCandidates += seedCandidates.candidates;
        seedCandidateSets.push_back(seedCandidates);
    }

    if (bottomCandidates.size() < 12)
    {
        result.error = QString("完整点云中未找到足够的底板候选点：候选=%1，访问局部点=%2。")
            .arg(bottomCandidates.size())
            .arg(visitedLocalPointCount);
        return result;
    }

    QVector<double> projectedZValues;
    QVector<bool> projectedZValid;
    projectedZValues.reserve(seedCandidateSets.size());
    projectedZValid.reserve(seedCandidateSets.size());
    int profileValueCount = 0;
    int fallbackValueCount = 0;
    for (const WorkpieceProjectionSeedCandidates& seedCandidates : seedCandidateSets)
    {
        QVector<double> localZValues;
        localZValues.reserve(seedCandidates.candidates.size());
        for (const Eigen::Vector3d& point : seedCandidates.candidates)
        {
            localZValues.push_back(point.z());
        }

        if (!localZValues.isEmpty())
        {
            projectedZValues.push_back(GeometryMedianScalar(localZValues));
            projectedZValid.push_back(true);
            ++profileValueCount;
        }
        else
        {
            projectedZValues.push_back(seedCandidates.seed.point.z());
            projectedZValid.push_back(false);
            ++fallbackValueCount;
        }
    }

    const int smoothRadius = params.projectionSmoothRadius > 0
        ? params.projectionSmoothRadius
        : std::clamp(params.smoothRadius, 1, 4);
    QVector<double> smoothedProjectedZ = projectedZValues;
    for (int index = 0; index < projectedZValues.size(); ++index)
    {
        QVector<double> values;
        const int beginIndex = std::max(0, index - smoothRadius);
        const int endIndex = std::min(static_cast<int>(projectedZValues.size()) - 1, index + smoothRadius);
        values.reserve(endIndex - beginIndex + 1);
        for (int nearbyIndex = beginIndex; nearbyIndex <= endIndex; ++nearbyIndex)
        {
            if (projectedZValid[nearbyIndex])
            {
                values.push_back(projectedZValues[nearbyIndex]);
            }
        }
        if (!values.isEmpty())
        {
            smoothedProjectedZ[index] = GeometryMedianScalar(values);
        }
    }

    result.points.reserve(seedCandidateSets.size());
    for (int seedIndex = 0; seedIndex < seedCandidateSets.size(); ++seedIndex)
    {
        const IndexedPoint3D& seed = seedCandidateSets[seedIndex].seed;
        if (!IsFinitePoint(seed.point))
        {
            continue;
        }

        LowerWeldFilterPoint point;
        point.index = seed.index;
        point.point = seed.point;
        point.point.z() = smoothedProjectedZ[seedIndex];
        point.source = projectedZValid[seedIndex]
            ? "workpiece_middle_bottom_profile_projected"
            : "workpiece_middle_bottom_seed_fallback";
        result.points.push_back(point);
    }

    if (result.points.size() < std::max(2, params.minPointCount))
    {
        result.error = QString("底板投影后有效焊道点过少，仅 %1 个。").arg(result.points.size());
        return result;
    }

    result.lowerPointCount = bottomCandidates.size();
    result.zContinuityRejectedCount = visitedLocalPointCount - bottomCandidates.size();
    result.segmentRejectedCount = visitedLocalPointCount;
    result.fitSegmentCount = profileValueCount;
    result.measuredCount = result.points.size();
    result.interpolatedCount = profileValueCount;
    result.extendedCount = fallbackValueCount;
    result.ok = true;
    return result;
}


RobotCalculation::LowerWeldClassificationResult RobotCalculation::BuildCornerCompensatedLowerWeldClassification(
    const QVector<LowerWeldClassifiedPoint>& keyPoints,
    const LowerWeldFilterParams& params,
    QVector<LowerWeldClassifiedPoint>* compensatedKeyPoints)
{
    if (compensatedKeyPoints != nullptr)
    {
        compensatedKeyPoints->clear();
    }

    LowerWeldClassificationResult emptyResult;
    if (!HasCornerCompensation(params))
    {
        return emptyResult;
    }
    if (keyPoints.size() < 2)
    {
        emptyResult.error = "拐点数量不足，无法进行拐点补偿。";
        return emptyResult;
    }

    QVector<LowerWeldClassifiedPoint> adjustedKeyPoints = keyPoints;
    QVector<QString> segmentKinds = AssignSegmentKindsFromKeyPoints(keyPoints, params.sampleAxis);
    if (segmentKinds.size() != keyPoints.size() - 1)
    {
        emptyResult.error = "拐点段属性不足，无法进行拐点补偿。";
        return emptyResult;
    }

    for (int index = 0; index < adjustedKeyPoints.size(); ++index)
    {
        if (index < segmentKinds.size())
        {
            adjustedKeyPoints[index].segmentKindAfter = segmentKinds[index];
        }

        const LowerWeldPointType pointType = adjustedKeyPoints[index].type;
        if (pointType != LowerWeldPointType::InnerCorner
            && pointType != LowerWeldPointType::OuterCorner)
        {
            continue;
        }

        QString nearbySlopeSegmentKind;
        if (index < segmentKinds.size()
            && (IsRisingSegmentKind(segmentKinds[index]) || IsFallingSegmentKind(segmentKinds[index])))
        {
            nearbySlopeSegmentKind = segmentKinds[index];
        }
        else if (index > 0
            && (IsRisingSegmentKind(segmentKinds[index - 1]) || IsFallingSegmentKind(segmentKinds[index - 1])))
        {
            nearbySlopeSegmentKind = segmentKinds[index - 1];
        }

        if (nearbySlopeSegmentKind.isEmpty())
        {
            continue;
        }

        Eigen::Vector3d offset = Eigen::Vector3d::Zero();
        const auto appendAdjacentCompensation = [&](int otherIndex, int segmentIndex)
            {
                if (otherIndex < 0 || otherIndex >= keyPoints.size()
                    || segmentIndex < 0 || segmentIndex >= segmentKinds.size())
                {
                    return;
                }

                const QString effectiveSlopeKind =
                    IsRisingSegmentKind(segmentKinds[segmentIndex]) || IsFallingSegmentKind(segmentKinds[segmentIndex])
                        ? segmentKinds[segmentIndex]
                        : nearbySlopeSegmentKind;
                const double compensationMm = CornerCompensationMm(
                    pointType,
                    keyPoints[otherIndex].type,
                    effectiveSlopeKind,
                    params);
                if (std::abs(compensationMm) <= 1e-9)
                {
                    return;
                }

                const Eigen::Vector3d direction = keyPoints[otherIndex].point - keyPoints[index].point;
                const double length = direction.norm();
                if (length <= std::numeric_limits<double>::epsilon())
                {
                    return;
                }

                offset += direction.normalized() * compensationMm;
            };

        appendAdjacentCompensation(index - 1, index - 1);
        appendAdjacentCompensation(index + 1, index);
        adjustedKeyPoints[index].point += offset;
    }

    if (compensatedKeyPoints != nullptr)
    {
        *compensatedKeyPoints = adjustedKeyPoints;
    }

    return BuildExpandedClassificationFromKeyPoints(
        adjustedKeyPoints,
        params.sampleStep > 0.0 ? params.sampleStep : 2.0,
        "geometry_2mm_corner_comp");
}

namespace
{
// 调试导出：把每段直线拟合“用到的点集”和“拟合出的直线”写成 CloudCompare 友好的 ASCII 点云，
// 用来直观核对分段几何拟合是否正确。生成到 <outputDir>/FitDebug/：
//   fit_all_points.txt   所有段的输入点，按段号上色(R G B)，附 dist_to_fit(到本段拟合直线的垂距)、smoothN 两个标量；
//   fit_all_lines.txt    每段拟合直线沿 s 密集采样并还原回 3D 的点，与点集同段同色；
//   fit_keypoints.txt    起点/终点/拐点（红色，附类型码）；
//   segments/seg_XX_*    每段单独的输入点集与拟合直线（两种组织方式都给）；
//   fit_axes.txt         本次拟合使用的局部坐标系(质心 center + 三轴 main/side/normal)。
// 列以空格分隔，表头用 // 开头(CloudCompare 导入时会自动当注释跳过)。outputDir 为空时不导出。
void ExportGeometryFitDebugClouds(
    const QVector<GeometryProjectedPoint>& projected,
    const QVector<int>& keyIndexes,
    const QVector<Eigen::Vector3d>& fittedKeyPoints,
    const Eigen::Vector3d& center,
    const Eigen::Vector3d& mainAxis,
    const Eigen::Vector3d& sideAxis,
    const Eigen::Vector3d& normalAxis,
    bool useSlopeConsistentCornerFit,
    const QString& outputDir)
{
    if (outputDir.isEmpty() || keyIndexes.size() < 2 || projected.isEmpty())
    {
        return;
    }

    const QString debugRoot = QDir(outputDir).filePath(QStringLiteral("FitDebug"));
    const QString segmentDir = QDir(debugRoot).filePath(QStringLiteral("segments"));
    QDir().mkpath(debugRoot);
    QDir().mkpath(segmentDir);

    const auto writeCloud = [](const QString& path, const QStringList& lines)
    {
        QFile file(path);
        if (!file.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate))
        {
            return;
        }
        QTextStream stream(&file);
        for (const QString& line : lines)
        {
            stream << line << '\n';
        }
    };

    const auto formatPoint = [](const Eigen::Vector3d& p, int r, int g, int b, const QString& extra)
    {
        QString row = QString("%1 %2 %3 %4 %5 %6")
            .arg(p.x(), 0, 'f', 6)
            .arg(p.y(), 0, 'f', 6)
            .arg(p.z(), 0, 'f', 6)
            .arg(r)
            .arg(g)
            .arg(b);
        if (!extra.isEmpty())
        {
            row += QLatin1Char(' ');
            row += extra;
        }
        return row;
    };

    static const int kPalette[12][3] = {
        {230, 25, 75}, {60, 180, 75}, {255, 200, 20}, {0, 130, 200},
        {245, 130, 48}, {145, 30, 180}, {70, 200, 240}, {240, 50, 230},
        {170, 220, 40}, {250, 150, 190}, {0, 160, 160}, {200, 170, 255}
    };

    const QVector<GeometryFittedLine2D> segmentLines =
        BuildGeometrySegmentLines(projected, keyIndexes, useSlopeConsistentCornerFit);

    QStringList allPointLines;
    allPointLines << QStringLiteral("// X Y Z R G B segment dist_to_fit smoothN");
    QStringList allLineLines;
    allLineLines << QStringLiteral("// X Y Z R G B segment");

    const int segmentCount = keyIndexes.size() - 1;
    for (int seg = 0; seg < segmentCount; ++seg)
    {
        const int begin = std::min(keyIndexes[seg], keyIndexes[seg + 1]);
        const int end = std::max(keyIndexes[seg], keyIndexes[seg + 1]);
        if (begin < 0 || end >= projected.size() || end < begin)
        {
            continue;
        }
        const int r = kPalette[seg % 12][0];
        const int g = kPalette[seg % 12][1];
        const int b = kPalette[seg % 12][2];

        const GeometryFittedLine2D line =
            seg < segmentLines.size() ? segmentLines[seg] : GeometryFittedLine2D();
        const GeometryFittedScalarLine normalLine =
            FitGeometrySegmentNormalLine(projected, begin, end);
        const double fallbackNormal = projected[(begin + end) / 2].smoothN;

        QStringList segPointLines;
        segPointLines << QStringLiteral("// X Y Z R G B dist_to_fit smoothN");
        QStringList segLineLines;
        segLineLines << QStringLiteral("// X Y Z R G B");

        // 1) 该段拟合“用到的点集”——附每点到本段拟合直线的垂距 dist_to_fit，
        //    在 CloudCompare 里按这个标量上色即可看出哪些点贴合、哪些被当作离群点甩开。
        for (int i = begin; i <= end; ++i)
        {
            double dist = 0.0;
            if (line.valid)
            {
                const Eigen::Vector2d v =
                    Eigen::Vector2d(projected[i].s, projected[i].smoothH) - line.point;
                dist = std::abs(Cross2D(v, line.direction));
            }
            const QString extraAll = QString("%1 %2 %3")
                .arg(seg)
                .arg(dist, 0, 'f', 6)
                .arg(projected[i].smoothN, 0, 'f', 6);
            const QString extraSeg = QString("%1 %2")
                .arg(dist, 0, 'f', 6)
                .arg(projected[i].smoothN, 0, 'f', 6);
            allPointLines << formatPoint(projected[i].point, r, g, b, extraAll);
            segPointLines << formatPoint(projected[i].point, r, g, b, extraSeg);
        }

        // 2) 该段拟合出的直线：在 (s, smoothH) 平面沿 s 密集采样，再用 normalLine 估高度 n，
        //    最后 GeometryPointFromProjection 还原回 3D，便于和点集叠加比对。
        if (line.valid)
        {
            const double sLo = std::min(projected[begin].s, projected[end].s);
            const double sHi = std::max(projected[begin].s, projected[end].s);
            const double step = 0.5;
            const bool nonVertical = std::abs(line.direction.x()) > 1e-6;
            for (double s = sLo; s <= sHi + 1e-9; s += step)
            {
                Eigen::Vector2d sh;
                if (nonVertical)
                {
                    const double t = (s - line.point.x()) / line.direction.x();
                    sh = line.point + t * line.direction;
                }
                else
                {
                    sh = Eigen::Vector2d(s, line.point.y());
                }
                const double nv = normalLine.valid ? normalLine.valueAt(sh.x()) : fallbackNormal;
                const Eigen::Vector3d p3 =
                    GeometryPointFromProjection(center, mainAxis, sideAxis, normalAxis, sh, nv);
                allLineLines << formatPoint(p3, r, g, b, QString::number(seg));
                segLineLines << formatPoint(p3, r, g, b, QString());
            }
        }

        const QString tag = QString("%1").arg(seg, 2, 10, QLatin1Char('0'));
        writeCloud(QDir(segmentDir).filePath(QStringLiteral("seg_%1_points.txt").arg(tag)), segPointLines);
        writeCloud(QDir(segmentDir).filePath(QStringLiteral("seg_%1_line.txt").arg(tag)), segLineLines);
    }

    // 3) 关键点（起点/终点/拐点）
    QStringList keyPointLines;
    keyPointLines << QStringLiteral("// X Y Z R G B key_type(1=start 2=end 3=inner_corner 4=outer_corner)");
    for (int k = 0; k < fittedKeyPoints.size() && k < keyIndexes.size(); ++k)
    {
        const RobotCalculation::LowerWeldPointType type =
            GeometryCornerType(projected, keyIndexes, k);
        keyPointLines << formatPoint(fittedKeyPoints[k], 255, 0, 0, QString::number(static_cast<int>(type)));
    }

    // 4) 本次拟合使用的局部坐标系，便于理解/复现 (s,h,n) 投影
    QStringList axisLines;
    axisLines << QStringLiteral("// role X Y Z");
    const auto axisRow = [](const QString& role, const Eigen::Vector3d& v)
    {
        return QString("%1 %2 %3 %4")
            .arg(role)
            .arg(v.x(), 0, 'f', 6)
            .arg(v.y(), 0, 'f', 6)
            .arg(v.z(), 0, 'f', 6);
    };
    axisLines << axisRow(QStringLiteral("center"), center);
    axisLines << axisRow(QStringLiteral("mainAxis_s"), mainAxis);
    axisLines << axisRow(QStringLiteral("sideAxis_h"), sideAxis);
    axisLines << axisRow(QStringLiteral("normalAxis_n"), normalAxis);

    writeCloud(QDir(debugRoot).filePath(QStringLiteral("fit_all_points.txt")), allPointLines);
    writeCloud(QDir(debugRoot).filePath(QStringLiteral("fit_all_lines.txt")), allLineLines);
    writeCloud(QDir(debugRoot).filePath(QStringLiteral("fit_keypoints.txt")), keyPointLines);
    writeCloud(QDir(debugRoot).filePath(QStringLiteral("fit_axes.txt")), axisLines);
}
}  // namespace

namespace
{
// 基础焊道首尾段截断：沿点列数组序按累积弧长(mm)截掉开头 headMm、结尾 tailMm 的点（拟合提取关键点之前调用）。
// 与焊接方向无关(开头=数组首点侧/结尾=末点侧)，用于剔除扫描进/出端坏点。截后不足 2 点 或 head+tail≥总弧长
// 时放弃截断(原样返回)，避免把焊道截没。keptFrom/keptTo 回传保留区间(供日志)。
QVector<RobotCalculation::IndexedPoint3D> TrimLowerWeldPathByArcLength(
    const QVector<RobotCalculation::IndexedPoint3D>& pts,
    double headMm, double tailMm, int* keptFrom, int* keptTo)
{
    const int n = pts.size();
    if (keptFrom != nullptr) { *keptFrom = 0; }
    if (keptTo != nullptr) { *keptTo = n - 1; }
    const double headCut = std::max(0.0, headMm);
    const double tailCut = std::max(0.0, tailMm);
    if (n < 3 || (headCut <= 0.0 && tailCut <= 0.0))
    {
        return pts;
    }
    std::vector<double> arc(n, 0.0);
    for (int i = 1; i < n; ++i)
    {
        arc[i] = arc[i - 1] + (pts[i].point - pts[i - 1].point).norm();
    }
    const double total = arc[n - 1];
    if (headCut + tailCut >= total)
    {
        return pts;
    }
    int from = 0;
    while (from < n - 1 && arc[from] < headCut) { ++from; }
    int to = n - 1;
    while (to > 0 && (total - arc[to]) < tailCut) { --to; }
    if (to - from + 1 < 2)
    {
        return pts;
    }
    if (keptFrom != nullptr) { *keptFrom = from; }
    if (keptTo != nullptr) { *keptTo = to; }
    QVector<RobotCalculation::IndexedPoint3D> out;
    out.reserve(to - from + 1);
    for (int i = from; i <= to; ++i)
    {
        out.push_back(pts[i]);
    }
    return out;
}
}  // namespace

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

    // 基础焊道首尾段截断（拟合提取关键点之前、去噪之前；所有几何拟合方案②③④通用，与焊接方向解耦）。
    if (params.enableEdgeTruncate && (params.truncateHeadMm > 0.0 || params.truncateTailMm > 0.0))
    {
        validPoints = TrimLowerWeldPathByArcLength(
            validPoints, params.truncateHeadMm, params.truncateTailMm, nullptr, nullptr);
    }

    // 输入已去噪(SDK is_remove_noise 开)时跳过程序自身的 MAD 去噪——重复处理会削圆尖角、移位拐点。
    const bool skipDenoise = params.inputAlreadyDenoised;

    int denoiseRejectedCount = 0;
    const QVector<IndexedPoint3D> denoisedPoints = skipDenoise
        ? validPoints
        : RemoveGeometryLocalOutliers(validPoints, params, &denoiseRejectedCount);

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
        skipDenoise ? 0 : params.smoothRadius);  // 已去噪输入不平滑(平滑会削圆尖角、移位拐点)
    int projectedBranchRejectedCount = 0;
    if (!skipDenoise)  // 已去噪输入无分支岛，跳过分支离群剔除以免误伤拐角附近点
    {
        projected = RemoveGeometryProjectedBranchOutliers(projected, params, &projectedBranchRejectedCount);
        denoiseRejectedCount += projectedBranchRejectedCount;
    }
    int slopeWaveRejectedCount = 0;
    if (params.geometryStrategy == LowerWeldGeometryStrategy::SlopeWaveFiltered)
    {
        projected = RemoveGeometrySlopeWaveOutliers(projected, params, &slopeWaveRejectedCount);
        denoiseRejectedCount += slopeWaveRejectedCount;
    }

    result.filterResult.lowerPointCount = projected.size();
    result.filterResult.zContinuityRejectedCount = denoiseRejectedCount;
    if (projected.size() < std::max(2, params.minPointCount))
    {
        result.error = QString("有效点太少，仅 %1 个，无法进行几何特征提取。")
            .arg(projected.size());
        return result;
    }

    QVector<int> keyIndexes;
    if (skipDenoise)
    {
        // 输入已去噪：方位角拐点检测 + 直线化兜底（替代 DP，避免缓变/台阶拐点标偏、漏检真折角）。
        // 顶点位置仍由 BuildFittedGeometryKeyPoints 局部求交精修，内外角仍由 GeometryCornerType 判定。
        keyIndexes = BuildAzimuthCornerKeyIndexes(projected, params);
        keyIndexes = SplitNonStraightAzimuthSegments(projected, keyIndexes, params);
    }
    else
    {
        const QVector<int> initialKeyIndexes =
            params.geometryStrategy == LowerWeldGeometryStrategy::RobustSegmentedKeys
                ? BuildRobustSegmentedGeometryKeyIndexes(projected, params)
                : BuildGeometryKeyIndexes(projected, params);
        keyIndexes = PruneRedundantSameSideGeometryKeys(projected, initialKeyIndexes);
        keyIndexes = PruneGeometrySpikeDrivenKeys(projected, keyIndexes, params);
        keyIndexes = PruneShortSameTypeGeometryRuns(projected, keyIndexes, params);
        keyIndexes = PruneNonMonotonicFittedGeometryKeys(
            projected,
            keyIndexes,
            center,
            axes.first,
            axes.second,
            normalAxis,
            params.useSlopeConsistentCornerFit);
        keyIndexes = RefineGeometryKeysBySegmentDeviation(
            projected,
            keyIndexes,
            center,
            axes.first,
            axes.second,
            params);
        keyIndexes = PruneGeometrySpikeDrivenKeys(projected, keyIndexes, params);
        keyIndexes = PruneShortSameTypeGeometryRuns(projected, keyIndexes, params);
        keyIndexes = PruneNonMonotonicFittedGeometryKeys(
            projected,
            keyIndexes,
            center,
            axes.first,
            axes.second,
            normalAxis,
            params.useSlopeConsistentCornerFit);
    }
    // 起终点先验自适应细化(两条路径统一)：方位角分支(SdkBaseWeldFit)与 DP 分支(CloudFit/特征点)
    // 都在此对仍偏直的段补回被直线化抹平的"相干弓"拐点——尤其端区引入/收尾段的缓弓(单侧弓向门挡随机
    // 噪声；端区低地板/中段高地板)。修复 CloudFit 模式末端漏拐点(现场 RobotC/20260625_015)。
    keyIndexes = RefineCornersByCoherentBow(projected, keyIndexes, params);
    // 板材搭接错位检测：错位点作硬段边界注入 keyIndexes(在所有 prune/refine 之后注入，避免被清洗)，
    // 使搭接两侧点云各自独立拟合；isLapStepKey 标记台阶端点，供 BuildFittedGeometryKeyPoints 取本侧线端点。
    QVector<char> isLapStepKey(keyIndexes.size(), 0);
    if (params.enableLapMisalignmentSplit)
    {
        const QVector<int> lapCenters = DetectLapMisalignmentBoundaries(projected, params);
        QVector<int> stepValues;
        QVector<int> merged = keyIndexes;
        for (int ci : lapCenters)
        {
            if (ci - 1 > 0 && ci < static_cast<int>(projected.size()) - 1)
            {
                merged.push_back(ci - 1);  // 台阶左点(板1侧端)
                merged.push_back(ci);      // 台阶右点(板2侧端)
                stepValues.push_back(ci - 1);
                stepValues.push_back(ci);
            }
        }
        if (!stepValues.isEmpty())
        {
            std::sort(merged.begin(), merged.end());
            merged.erase(std::unique(merged.begin(), merged.end()), merged.end());
            keyIndexes = merged;
            isLapStepKey = QVector<char>(keyIndexes.size(), 0);
            for (int k = 0; k < keyIndexes.size(); ++k)
                if (stepValues.contains(keyIndexes[k])) isLapStepKey[k] = 1;
        }
    }
    // 拐点 inner/outer 结构约束：按平台从稠密路径重算每平台 2 个边界角(修源头多检/漏检)。会改变
    // keyIndexes 的个数/顺序，故先记下 lap 角的 projected 索引，重算后据此重建 isLapStepKey 与之对齐。
    {
        QVector<int> lapValues;
        for (int k = 0; k < keyIndexes.size() && k < isLapStepKey.size(); ++k)
            if (isLapStepKey[k]) lapValues.push_back(keyIndexes[k]);
        keyIndexes = RefitCornersByPlatformPattern(projected, keyIndexes, isLapStepKey, params);
        isLapStepKey = QVector<char>(keyIndexes.size(), 0);
        for (int k = 0; k < keyIndexes.size(); ++k)
            if (lapValues.contains(keyIndexes[k])) isLapStepKey[k] = 1;
        // 端区周期一致性补拐点：用波纹周期(中段中位段长)+典型拐角弯折角，反推并补回端区(起/终段)被检测
        // 盲区漏掉的拐点(尤其起点引入段)。只插入非搭接拐点；插入后据 lapValues 重建 isLapStepKey 对齐。
        if (params.enableEndPeriodCornerRecover)
        {
            int recoveredCount = 0;
            keyIndexes = RecoverEndRegionCornersByPeriod(
                projected, keyIndexes, isLapStepKey,
                params.endPeriodRatioThreshold, params.endPeriodMinBendDeg, &recoveredCount);
            if (recoveredCount > 0)
            {
                isLapStepKey = QVector<char>(keyIndexes.size(), 0);
                for (int k = 0; k < keyIndexes.size(); ++k)
                    if (lapValues.contains(keyIndexes[k])) isLapStepKey[k] = 1;
            }
            // 周期/角度删错拐点：平台重算等可能把一个平台的两个同类边界角凑到 ≪ 周期(如4mm)，
            // SDK+拟合路径又跳过了 Prune 去重，这里据"相邻同类拐点间距≪周期=找错"合并清理。搭接对豁免。
            int mergedCount = 0;
            keyIndexes = MergeTooCloseSameTypeCorners(
                projected, keyIndexes, isLapStepKey, params.endPeriodMergeFrac, &mergedCount);
            if (mergedCount > 0)
            {
                isLapStepKey = QVector<char>(keyIndexes.size(), 0);
                for (int k = 0; k < keyIndexes.size(); ++k)
                    if (lapValues.contains(keyIndexes[k])) isLapStepKey[k] = 1;
            }
        }
        // 按平台边界重定拐点：波纹拐点全在"平台↔坡"交界。检测平的段(平台)，确保每个平台两端各有边界角、
        // 删掉卡在平台内部(放错位)的角。治"拐点放到平台中间→平台塌成长坡消失"、端区漏平台边界角。
        // 对已正确的平台幂等不动；只纠正错的。搭接/起终点不参与。插入/删除后据 lapValues 重建 isLapStepKey。
        if (params.enablePlatformCornerSnap)
        {
            int snappedCount = 0;
            keyIndexes = SnapCornersToPlatforms(
                projected, keyIndexes, isLapStepKey,
                params.platformSnapFlatSlope, params.platformSnapMinFrac, &snappedCount);
            if (snappedCount > 0)
            {
                isLapStepKey = QVector<char>(keyIndexes.size(), 0);
                for (int k = 0; k < keyIndexes.size(); ++k)
                    if (lapValues.contains(keyIndexes[k])) isLapStepKey[k] = 1;
            }
        }
    }
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
        normalAxis,
        params.useSlopeConsistentCornerFit,
        isLapStepKey);
    if (fittedKeyPoints.size() != keyIndexes.size())
    {
        result.error = "几何特征拟合失败，无法生成拟合交点。";
        return result;
    }

    if (params.exportFitDebugCloud)
    {
        ExportGeometryFitDebugClouds(
            projected,
            keyIndexes,
            fittedKeyPoints,
            center,
            axes.first,
            axes.second,
            normalAxis,
            params.useSlopeConsistentCornerFit,
            params.fitDebugDir);
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

    for (int index = 0; index < keyIndexes.size(); ++index)
    {
        const Eigen::Vector3d currentPoint = fittedKeyPoints[index];
        const bool isStepKey = (index < isLapStepKey.size() && isLapStepKey[index] != 0);
        LowerWeldPointType pointType;
        if (isStepKey)
        {
            // 搭接台阶端点：按本点 smoothH 相对配对台阶点的高低定内/外角(高=Inner 低=Outer，与 prominence 一致)。
            const bool isLeftOfPair = (index + 1 < isLapStepKey.size() && isLapStepKey[index + 1] != 0);
            const int pairIdx = isLeftOfPair ? index + 1 : index - 1;
            const double myH = projected[keyIndexes[index]].smoothH;
            const double pairH = (pairIdx >= 0 && pairIdx < keyIndexes.size())
                ? projected[keyIndexes[pairIdx]].smoothH : myH;
            pointType = (myH >= pairH) ? LowerWeldPointType::InnerCorner : LowerWeldPointType::OuterCorner;
        }
        else
        {
            pointType = GeometryCornerType(projected, keyIndexes, index);
        }
        const QString source = isStepKey
            ? "geometry_lap_step"
            : (pointType == LowerWeldPointType::Start
                ? "geometry_start"
                : (pointType == LowerWeldPointType::End
                    ? "geometry_end"
                    : (pointType == LowerWeldPointType::InnerCorner
                        ? "geometry_inner"
                        : "geometry_outer")));

        LowerWeldClassifiedPoint keyPoint;
        keyPoint.index = projected[keyIndexes[index]].inputIndex;
        keyPoint.point = currentPoint;
        keyPoint.type = pointType;
        keyPoint.source = source;
        keyPoint.isLapStepBoundary = isStepKey;
        result.keyPoints.push_back(keyPoint);
    }

    const QVector<QString> segmentKinds = AssignSegmentKindsFromKeyPoints(result.keyPoints, params.sampleAxis);
    for (int index = 0; index < result.keyPoints.size() && index < segmentKinds.size(); ++index)
    {
        result.keyPoints[index].segmentKindAfter = segmentKinds[index];
    }

    result.classificationResult = BuildExpandedClassificationFromKeyPoints(
        result.keyPoints,
        expandStepMm,
        "geometry_2mm");
    if (!result.classificationResult.ok)
    {
        result.error = result.classificationResult.error;
        return result;
    }

    QString validationError;
    if (!ValidateGeometryAnalysisResult(
            validPoints.size(),
            denoiseRejectedCount,
            projected,
            fittedKeyPoints,
            center,
            axes.first,
            axes.second,
            result,
            params,
            &result.qualityReport,
            &validationError))
    {
        result.qualityReport.inputPointCount = inputPoints.size();
        if (!params.validationAuditOnly)
        {
            result.error = validationError;
            return result;
        }
    }
    result.qualityReport.inputPointCount = inputPoints.size();

    result.cornerCompensatedClassificationResult =
        BuildCornerCompensatedLowerWeldClassification(
            result.keyPoints,
            params,
            &result.cornerCompensatedKeyPoints);

    result.filterResult.interpolatedCount = result.classificationResult.normalCount;
    result.filterResult.extendedCount = result.classificationResult.points.size();

    result.ok = true;
    return result;
}

RobotCalculation::MeasureThenWeldAnalysisResult RobotCalculation::AnalyzeMeasureThenWeldLowerWeldPathDirect(
    const QVector<IndexedPoint3D>& inputPoints,
    const LowerWeldFilterParams& params)
{
    return AnalyzeMeasureThenWeldLowerWeldPathGeometry(inputPoints, params);
}

RobotCalculation::MeasureThenWeldAnalysisResult::PointCloudQualityReport
RobotCalculation::EvaluateMeasureThenWeldOutputQuality(
    int inputPointCount,
    int finiteInputPointCount,
    int rejectedPointCount,
    const QVector<LowerWeldClassifiedPoint>& keyPoints,
    const LowerWeldClassificationResult& classification,
    const LowerWeldFilterParams& params)
{
    MeasureThenWeldAnalysisResult::PointCloudQualityReport report;
    report.evaluated = true;
    report.auditOnly = params.validationAuditOnly;
    report.inputPointCount = std::max(0, inputPointCount);

    QVector<IndexedPoint3D> finitePoints;
    finitePoints.reserve(classification.points.size());
    for (const LowerWeldClassifiedPoint& point : classification.points)
    {
        if (std::isfinite(point.point.x())
            && std::isfinite(point.point.y())
            && std::isfinite(point.point.z()))
        {
            IndexedPoint3D sample;
            sample.index = point.index;
            sample.point = point.point;
            finitePoints.push_back(sample);
        }
    }
    if (finitePoints.size() < 2)
    {
        report.finitePointCount = finitePoints.size();
        report.outputPointCount = classification.points.size();
        report.failures.push_back(QString("点云有效性检测失败：SDK直出焊道有限点不足，当前 %1 个。")
            .arg(finitePoints.size()));
        return report;
    }

    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    for (const IndexedPoint3D& point : finitePoints)
    {
        center += point.point;
    }
    center /= static_cast<double>(finitePoints.size());
    const QPair<Eigen::Vector3d, Eigen::Vector3d> axes = BuildGeometryAxes(finitePoints, center);
    Eigen::Vector3d normalAxis = axes.first.cross(axes.second);
    if (normalAxis.squaredNorm() <= std::numeric_limits<double>::epsilon())
    {
        normalAxis = Eigen::Vector3d::UnitZ();
    }
    normalAxis.normalize();

    QVector<GeometryProjectedPoint> projected;
    projected.reserve(finitePoints.size());
    for (const IndexedPoint3D& point : finitePoints)
    {
        const Eigen::Vector3d delta = point.point - center;
        GeometryProjectedPoint output;
        output.inputIndex = point.index;
        output.point = point.point;
        output.s = delta.dot(axes.first);
        output.h = delta.dot(axes.second);
        output.n = delta.dot(normalAxis);
        output.smoothH = output.h;
        output.smoothN = output.n;
        projected.push_back(output);
    }

    QVector<Eigen::Vector3d> fittedKeyPoints;
    fittedKeyPoints.reserve(keyPoints.size());
    for (const LowerWeldClassifiedPoint& point : keyPoints)
    {
        fittedKeyPoints.push_back(point.point);
    }

    MeasureThenWeldAnalysisResult analysis;
    analysis.keyPoints = keyPoints;
    analysis.classificationResult = classification;
    QString validationError;
    // SDK 直出模式的采集点数/无效比例来自 SDK 实际收到的完整点云；连续性、残差和输出密度
    // 则评估本次 SDK 返回并将被执行的轨迹。两者不得再用 max(output,input) 或固定 rejected=0 伪造。
    const int coverageFinitePointCount = std::max(0, finiteInputPointCount);
    ValidateGeometryAnalysisResult(
        coverageFinitePointCount,
        std::max(0, rejectedPointCount),
        projected,
        fittedKeyPoints,
        center,
        axes.first,
        axes.second,
        analysis,
        params,
        &report,
        &validationError);
    report.inputPointCount = std::max(0, inputPointCount);
    return report;
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
