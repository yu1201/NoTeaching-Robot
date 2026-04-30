#include "RobotCalculation.h"
#include "SkFunction.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
constexpr double kMeasurePi = 3.14159265358979323846;

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

RobotCalculation::LowerWeldPointType GroovePointTypeForIndex(int index, int totalCount)
{
    using PointType = RobotCalculation::LowerWeldPointType;
    if (totalCount <= 0)
    {
        return PointType::Normal;
    }
    if (index <= 0)
    {
        return PointType::Start;
    }
    if (index >= totalCount - 1)
    {
        return PointType::End;
    }
    return (index % 2 == 1) ? PointType::InnerCorner : PointType::OuterCorner;
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

std::vector<SkPoint3D> ToSkPoint3DList(const QVector<RobotCalculation::IndexedPoint3D>& inputPoints)
{
    std::vector<SkPoint3D> points;
    points.reserve(static_cast<size_t>(inputPoints.size()));
    for (const RobotCalculation::IndexedPoint3D& inputPoint : inputPoints)
    {
        points.emplace_back(
            inputPoint.point.x(),
            inputPoint.point.y(),
            inputPoint.point.z(),
            inputPoint.index);
    }
    return points;
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

Eigen::Matrix3d RobotCalculation::RotX(double w)
{
    const double c = std::cos(w);
    const double s = std::sin(w);
    return (Eigen::Matrix3d() << 1.0, 0.0, 0.0,
                                  0.0, c, -s,
                                  0.0, s, c).finished();
}

Eigen::Matrix3d RobotCalculation::RotY(double p)
{
    const double c = std::cos(p);
    const double s = std::sin(p);
    return (Eigen::Matrix3d() << c, 0.0, s,
                                  0.0, 1.0, 0.0,
                                  -s, 0.0, c).finished();
}

Eigen::Matrix3d RobotCalculation::RotZ(double r)
{
    const double c = std::cos(r);
    const double s = std::sin(r);
    return (Eigen::Matrix3d() << c, -s, 0.0,
                                  s, c, 0.0,
                                  0.0, 0.0, 1.0).finished();
}

Eigen::Matrix3d RobotCalculation::FanucRot(double w, double p, double r)
{
    return RotZ(r) * RotY(p) * RotX(w);
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
    const double w = robotPose.dRX * kMeasurePi / 180.0;
    const double p = robotPose.dRY * kMeasurePi / 180.0;
    const double r = robotPose.dRZ * kMeasurePi / 180.0;
    const Eigen::Matrix3d robotRotation = FanucRot(w, p, r);
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

RobotCalculation::MeasureThenWeldAnalysisResult RobotCalculation::AnalyzeMeasureThenWeldLowerWeldPathDirect(
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
        if (!IsFinitePoint(sample.point))
        {
            continue;
        }
        validPoints.push_back(sample);
    }

    result.filterResult.lowerPointCount = validPoints.size();
    if (validPoints.size() < std::max(2, params.minPointCount))
    {
        result.error = QString("有效点太少，仅 %1 个，无法进行特征提取。")
            .arg(validPoints.size());
        return result;
    }

    const std::vector<SkPoint3D> grooveInputPoints = ToSkPoint3DList(validPoints);
    std::vector<SkPoint3D> turningPoints;
    if (getGrooveTurningPoints(grooveInputPoints, turningPoints) != TRUE || turningPoints.size() < 2)
    {
        result.error = "特征点提取失败，未找到可用的起点/终点/拐点。";
        return result;
    }

    result.filterResult.points.reserve(validPoints.size());
    for (const IndexedPoint3D& point : validPoints)
    {
        LowerWeldFilterPoint outputPoint;
        outputPoint.index = result.filterResult.points.size() + 1;
        outputPoint.point = point.point;
        outputPoint.source = "original";
        result.filterResult.points.push_back(outputPoint);
    }

    result.filterResult.measuredCount = static_cast<int>(turningPoints.size());
    result.filterResult.interpolatedCount = 0;
    result.filterResult.extendedCount = 0;
    result.filterResult.fitSegmentCount = 1;
    result.filterResult.ok = true;

    const double expandStepMm = params.sampleStep > 0.0 ? params.sampleStep : 2.0;
    const int turningPointCount = static_cast<int>(turningPoints.size());

    result.classificationResult.points.clear();
    result.classificationResult.points.reserve(
        static_cast<int>(turningPoints.size()) + static_cast<int>(turningPoints.size()) * 8);
    result.classificationResult.ok = true;
    result.classificationResult.error.clear();

    int nextIndex = 1;
    int interpolatedCount = 0;
    for (int index = 0; index < turningPointCount; ++index)
    {
        const SkPoint3D& turningPoint = turningPoints[static_cast<size_t>(index)];
        const Eigen::Vector3d currentPoint(turningPoint.x, turningPoint.y, turningPoint.z);
        const LowerWeldPointType pointType = GroovePointTypeForIndex(index, turningPointCount);
        AppendExpandedGroovePoint(
            result.classificationResult.points,
            nextIndex,
            currentPoint,
            pointType,
            "turning");

        if (index + 1 >= turningPointCount)
        {
            continue;
        }

        const SkPoint3D& nextTurningPoint = turningPoints[static_cast<size_t>(index + 1)];
        const Eigen::Vector3d nextPoint(nextTurningPoint.x, nextTurningPoint.y, nextTurningPoint.z);
        const Eigen::Vector3d delta = nextPoint - currentPoint;
        const double segmentLength = delta.norm();
        if (segmentLength <= std::numeric_limits<double>::epsilon())
        {
            continue;
        }

        if (expandStepMm > std::numeric_limits<double>::epsilon())
        {
            for (double distance = expandStepMm; distance < segmentLength - 1e-9; distance += expandStepMm)
            {
                const double ratio = distance / segmentLength;
                const Eigen::Vector3d expandedPoint = currentPoint + delta * ratio;
                AppendExpandedGroovePoint(
                    result.classificationResult.points,
                    nextIndex,
                    expandedPoint,
                    LowerWeldPointType::Normal,
                    "expanded_2mm");
                ++interpolatedCount;
            }
        }
    }

    result.filterResult.measuredCount = turningPointCount;
    result.filterResult.interpolatedCount = interpolatedCount;
    result.filterResult.extendedCount = result.classificationResult.points.size();
    result.classificationResult.startCount = 0;
    result.classificationResult.endCount = 0;
    result.classificationResult.innerCornerCount = 0;
    result.classificationResult.outerCornerCount = 0;
    result.classificationResult.normalCount = 0;
    result.classificationResult.noiseCount = 0;
    for (const LowerWeldClassifiedPoint& point : result.classificationResult.points)
    {
        switch (point.type)
        {
        case LowerWeldPointType::Start:
            ++result.classificationResult.startCount;
            break;
        case LowerWeldPointType::End:
            ++result.classificationResult.endCount;
            break;
        case LowerWeldPointType::InnerCorner:
            ++result.classificationResult.innerCornerCount;
            break;
        case LowerWeldPointType::OuterCorner:
            ++result.classificationResult.outerCornerCount;
            break;
        case LowerWeldPointType::Noise:
            ++result.classificationResult.noiseCount;
            break;
        case LowerWeldPointType::Normal:
        default:
            ++result.classificationResult.normalCount;
            break;
        }
    }

    result.ok = true;
    return result;
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
