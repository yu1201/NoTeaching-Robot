#include "MeasureThenWeldFilterFit.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <numeric>
#include <set>
#include <sstream>
#include <utility>
#include <vector>

// 波纹板拐点管线在 (s, smoothH) 平面上用 Eigen 做 2D 线性代数（直线 PCA / 方向夹角）。
// Eigen 头文件已随附在本模块的 third_party/eigen/ 下（header-only，整目录拷走即可编，无需另装/联网）；
// CMake 默认用该随附副本，也可用 -DEIGEN3_INCLUDE_DIR 或 find_package(Eigen3) 改用系统 Eigen。
#include <Eigen/Dense>

namespace mtw_filter_fit
{
namespace
{
constexpr double kEpsilon = 1e-9;

Point3D operator+(const Point3D& left, const Point3D& right)
{
    return { left.x + right.x, left.y + right.y, left.z + right.z };
}

Point3D operator-(const Point3D& left, const Point3D& right)
{
    return { left.x - right.x, left.y - right.y, left.z - right.z };
}

Point3D operator*(const Point3D& point, double scale)
{
    return { point.x * scale, point.y * scale, point.z * scale };
}

Point3D operator/(const Point3D& point, double scale)
{
    return scale == 0.0 ? Point3D() : Point3D{ point.x / scale, point.y / scale, point.z / scale };
}

double Dot(const Point3D& left, const Point3D& right)
{
    return left.x * right.x + left.y * right.y + left.z * right.z;
}

Point3D Cross(const Point3D& left, const Point3D& right)
{
    return {
        left.y * right.z - left.z * right.y,
        left.z * right.x - left.x * right.z,
        left.x * right.y - left.y * right.x
    };
}

double NormSquared(const Point3D& point)
{
    return Dot(point, point);
}

double Norm(const Point3D& point)
{
    return std::sqrt(NormSquared(point));
}

Point3D Normalize(Point3D point, const Point3D& fallback)
{
    const double norm = Norm(point);
    if (norm <= kEpsilon || !std::isfinite(norm))
    {
        return fallback;
    }
    return point / norm;
}

bool IsFinite(const Point3D& point)
{
    return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

double AxisValue(const Point3D& point, SampleAxis axis)
{
    return axis == SampleAxis::AxisX ? point.x : point.y;
}

double PrimaryValue(const Point3D& point, SampleAxis axis)
{
    return axis == SampleAxis::AxisY ? point.x : point.y;
}

double WorkpieceStationValue(const Point3D& point, SampleAxis axis)
{
    return axis == SampleAxis::AxisY ? point.y : point.x;
}

double WorkpieceTransverseValue(const Point3D& point, SampleAxis axis)
{
    return axis == SampleAxis::AxisY ? point.x : point.y;
}

Point3D SetAxisValue(Point3D point, SampleAxis axis, double value)
{
    if (axis == SampleAxis::AxisX)
    {
        point.x = value;
    }
    else
    {
        point.y = value;
    }
    return point;
}

std::string Number(double value)
{
    std::ostringstream out;
    out << std::fixed << std::setprecision(6) << value;
    return out.str();
}

std::string PointLine(int index, const Point3D& point, const std::string& extra)
{
    std::ostringstream out;
    out << index << ' '
        << std::fixed << std::setprecision(6)
        << point.x << ' ' << point.y << ' ' << point.z;
    if (!extra.empty())
    {
        out << ' ' << extra;
    }
    return out.str();
}

double Median(std::vector<double> values)
{
    if (values.empty())
    {
        return 0.0;
    }
    std::sort(values.begin(), values.end());
    const std::size_t middle = values.size() / 2;
    if (values.size() % 2 == 1)
    {
        return values[middle];
    }
    return (values[middle - 1] + values[middle]) * 0.5;
}

double Percentile(std::vector<double> values, double percentile)
{
    if (values.empty())
    {
        return 0.0;
    }
    std::sort(values.begin(), values.end());
    const double p = std::clamp(percentile, 0.0, 1.0);
    const std::size_t index = static_cast<std::size_t>(std::round(p * static_cast<double>(values.size() - 1)));
    return values[index];
}

double ValueRange(const std::vector<double>& values)
{
    if (values.empty())
    {
        return 0.0;
    }
    const auto minmax = std::minmax_element(values.begin(), values.end());
    return *minmax.second - *minmax.first;
}

std::vector<IndexedPoint3D> FinitePoints(const std::vector<IndexedPoint3D>& input)
{
    std::vector<IndexedPoint3D> output;
    output.reserve(input.size());
    for (const IndexedPoint3D& point : input)
    {
        if (IsFinite(point.point))
        {
            output.push_back(point);
        }
    }
    return output;
}

Point3D MedianPoint(const std::vector<IndexedPoint3D>& points, int begin, int end, int skipIndex)
{
    std::vector<double> xs;
    std::vector<double> ys;
    std::vector<double> zs;
    xs.reserve(static_cast<std::size_t>(std::max(0, end - begin)));
    ys.reserve(xs.capacity());
    zs.reserve(xs.capacity());
    for (int index = begin; index <= end; ++index)
    {
        if (index == skipIndex)
        {
            continue;
        }
        xs.push_back(points[static_cast<std::size_t>(index)].point.x);
        ys.push_back(points[static_cast<std::size_t>(index)].point.y);
        zs.push_back(points[static_cast<std::size_t>(index)].point.z);
    }
    return { Median(xs), Median(ys), Median(zs) };
}

std::vector<IndexedPoint3D> RemoveLocalOutliers(
    const std::vector<IndexedPoint3D>& points,
    const FilterFitParams& params,
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

    std::vector<IndexedPoint3D> current = points;
    int totalRejected = 0;
    for (int pass = 0; pass < 2 && current.size() >= 9; ++pass)
    {
        std::vector<char> keep(current.size(), 1);
        int passRejected = 0;
        for (int index = 0; index < static_cast<int>(current.size()); ++index)
        {
            const int begin = std::max(0, index - radius);
            const int end = std::min(static_cast<int>(current.size()) - 1, index + radius);
            if (end - begin < 4)
            {
                continue;
            }

            const Point3D localMedian = MedianPoint(current, begin, end, index);
            std::vector<double> localDistances;
            localDistances.reserve(static_cast<std::size_t>(end - begin));
            for (int sample = begin; sample <= end; ++sample)
            {
                if (sample != index)
                {
                    localDistances.push_back(Norm(current[static_cast<std::size_t>(sample)].point - localMedian));
                }
            }

            const double localMedianDistance = Median(localDistances);
            const double dynamicThreshold = std::max(baseThreshold, std::min(8.0, localMedianDistance * 6.0 + 0.5));
            if (Norm(current[static_cast<std::size_t>(index)].point - localMedian) > dynamicThreshold)
            {
                keep[static_cast<std::size_t>(index)] = 0;
                ++passRejected;
            }
        }

        if (passRejected <= 0)
        {
            break;
        }

        std::vector<IndexedPoint3D> filtered;
        filtered.reserve(current.size() - static_cast<std::size_t>(passRejected));
        for (std::size_t index = 0; index < current.size(); ++index)
        {
            if (keep[index])
            {
                filtered.push_back(current[index]);
            }
        }
        current = std::move(filtered);
        totalRejected += passRejected;
    }

    if (rejectedCount != nullptr)
    {
        *rejectedCount = totalRejected;
    }
    return current;
}

struct RawFilterPoint
{
    double axis = 0.0;
    Point3D point;
    bool valid = false;
    std::string source;
};

struct LinearFitSegment
{
    double slope = 0.0;
    double intercept = 0.0;
    double error = std::numeric_limits<double>::infinity();
    bool valid = false;
};

struct PrefixSums
{
    std::vector<double> sumX;
    std::vector<double> sumY;
    std::vector<double> sumXX;
    std::vector<double> sumXY;
    std::vector<double> sumYY;
};

std::pair<double, double> LinearFit1D(const std::vector<double>& xs, const std::vector<double>& ys)
{
    if (xs.empty() || xs.size() != ys.size())
    {
        return { 0.0, 0.0 };
    }
    const double count = static_cast<double>(xs.size());
    double sumX = 0.0;
    double sumY = 0.0;
    double sumXX = 0.0;
    double sumXY = 0.0;
    for (std::size_t index = 0; index < xs.size(); ++index)
    {
        sumX += xs[index];
        sumY += ys[index];
        sumXX += xs[index] * xs[index];
        sumXY += xs[index] * ys[index];
    }
    const double denominator = count * sumXX - sumX * sumX;
    if (std::abs(denominator) <= kEpsilon)
    {
        return { 0.0, sumY / count };
    }
    const double slope = (count * sumXY - sumX * sumY) / denominator;
    const double intercept = (sumY - slope * sumX) / count;
    return { slope, intercept };
}

PrefixSums BuildPrefixSums(const std::vector<double>& xs, const std::vector<double>& ys)
{
    PrefixSums sums;
    const std::size_t count = xs.size();
    sums.sumX.assign(count + 1, 0.0);
    sums.sumY.assign(count + 1, 0.0);
    sums.sumXX.assign(count + 1, 0.0);
    sums.sumXY.assign(count + 1, 0.0);
    sums.sumYY.assign(count + 1, 0.0);
    for (std::size_t index = 0; index < count; ++index)
    {
        sums.sumX[index + 1] = sums.sumX[index] + xs[index];
        sums.sumY[index + 1] = sums.sumY[index] + ys[index];
        sums.sumXX[index + 1] = sums.sumXX[index] + xs[index] * xs[index];
        sums.sumXY[index + 1] = sums.sumXY[index] + xs[index] * ys[index];
        sums.sumYY[index + 1] = sums.sumYY[index] + ys[index] * ys[index];
    }
    return sums;
}

LinearFitSegment FitRange(const PrefixSums& sums, int begin, int end)
{
    LinearFitSegment result;
    if (begin < 0 || end < begin || end + 1 >= static_cast<int>(sums.sumX.size()))
    {
        return result;
    }

    const double count = static_cast<double>(end - begin + 1);
    const double sumX = sums.sumX[static_cast<std::size_t>(end + 1)] - sums.sumX[static_cast<std::size_t>(begin)];
    const double sumY = sums.sumY[static_cast<std::size_t>(end + 1)] - sums.sumY[static_cast<std::size_t>(begin)];
    const double sumXX = sums.sumXX[static_cast<std::size_t>(end + 1)] - sums.sumXX[static_cast<std::size_t>(begin)];
    const double sumXY = sums.sumXY[static_cast<std::size_t>(end + 1)] - sums.sumXY[static_cast<std::size_t>(begin)];
    const double sumYY = sums.sumYY[static_cast<std::size_t>(end + 1)] - sums.sumYY[static_cast<std::size_t>(begin)];
    const double denominator = count * sumXX - sumX * sumX;
    if (std::abs(denominator) <= kEpsilon)
    {
        result.slope = 0.0;
        result.intercept = sumY / count;
    }
    else
    {
        result.slope = (count * sumXY - sumX * sumY) / denominator;
        result.intercept = (sumY - result.slope * sumX) / count;
    }
    result.error = sumYY
        + result.slope * result.slope * sumXX
        + count * result.intercept * result.intercept
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

Point3D EvaluateLineModelPoint(
    SampleAxis axis,
    double axisValue,
    const LinearFitSegment& primary,
    const LinearFitSegment& secondary)
{
    if (axis == SampleAxis::AxisY)
    {
        return { primary.slope * axisValue + primary.intercept, axisValue, secondary.slope * axisValue + secondary.intercept };
    }
    return { axisValue, primary.slope * axisValue + primary.intercept, secondary.slope * axisValue + secondary.intercept };
}

double PointSegmentDistance(const Point3D& point, const Point3D& begin, const Point3D& end)
{
    const Point3D segment = end - begin;
    const double lengthSquared = NormSquared(segment);
    if (lengthSquared <= kEpsilon)
    {
        return Norm(point - begin);
    }
    const double ratio = std::clamp(Dot(point - begin, segment) / lengthSquared, 0.0, 1.0);
    return Norm(point - (begin + segment * ratio));
}

void CollectPiecewiseBreakpoints(
    const std::vector<Point3D>& points,
    int begin,
    int end,
    double tolerance,
    std::vector<int>& breakpoints)
{
    if (end - begin < 2)
    {
        return;
    }

    double maxDistance = -1.0;
    int splitIndex = -1;
    for (int index = begin + 1; index < end; ++index)
    {
        const double distance = PointSegmentDistance(
            points[static_cast<std::size_t>(index)],
            points[static_cast<std::size_t>(begin)],
            points[static_cast<std::size_t>(end)]);
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

std::vector<int> BuildPiecewiseBreakpoints(
    const std::vector<Point3D>& points,
    double tolerance,
    int minSegmentPoints)
{
    std::vector<int> breakpoints;
    if (points.size() < 2)
    {
        return breakpoints;
    }
    breakpoints.push_back(0);
    CollectPiecewiseBreakpoints(points, 0, static_cast<int>(points.size()) - 1, tolerance, breakpoints);
    breakpoints.push_back(static_cast<int>(points.size()) - 1);
    std::sort(breakpoints.begin(), breakpoints.end());
    breakpoints.erase(std::unique(breakpoints.begin(), breakpoints.end()), breakpoints.end());

    const int minimumPoints = std::max(2, minSegmentPoints);
    bool changed = true;
    while (changed && breakpoints.size() > 2)
    {
        changed = false;
        for (int index = 1; index + 1 < static_cast<int>(breakpoints.size()); ++index)
        {
            const int leftCount = breakpoints[static_cast<std::size_t>(index)] - breakpoints[static_cast<std::size_t>(index - 1)] + 1;
            const int rightCount = breakpoints[static_cast<std::size_t>(index + 1)] - breakpoints[static_cast<std::size_t>(index)] + 1;
            if (leftCount < minimumPoints || rightCount < minimumPoints)
            {
                breakpoints.erase(breakpoints.begin() + index);
                changed = true;
                break;
            }
        }
    }
    return breakpoints;
}

bool ApplyPiecewiseFit(FilterResult& result, const FilterFitParams& params, bool trapezoidTemplate)
{
    const int trimCount = std::max(0, params.lineFitTrimCount);
    const int fitBegin = trimCount;
    const int fitEnd = static_cast<int>(result.points.size()) - trimCount;
    if (fitBegin >= fitEnd || fitEnd - fitBegin < 2)
    {
        result.error = "piecewise fit has too few points after trimming.";
        return false;
    }

    std::vector<Point3D> fitPoints;
    std::vector<double> axes;
    std::vector<double> primaryTargets;
    std::vector<double> secondaryTargets;
    fitPoints.reserve(static_cast<std::size_t>(fitEnd - fitBegin));
    for (int index = fitBegin; index < fitEnd; ++index)
    {
        const Point3D point = result.points[static_cast<std::size_t>(index)].point;
        const double axisValue = AxisValue(point, params.sampleAxis);
        fitPoints.push_back(point);
        axes.push_back(axisValue);
        primaryTargets.push_back(params.sampleAxis == SampleAxis::AxisY ? point.x : point.y);
        secondaryTargets.push_back(point.z);
    }

    const std::vector<int> breakpoints = BuildPiecewiseBreakpoints(
        fitPoints,
        std::max(0.1, params.piecewiseFitTolerance),
        params.piecewiseMinSegmentPoints);
    if (breakpoints.size() < 2)
    {
        result.error = "piecewise fit could not build valid breakpoints.";
        return false;
    }

    const PrefixSums primarySums = BuildPrefixSums(axes, primaryTargets);
    const PrefixSums secondarySums = BuildPrefixSums(axes, secondaryTargets);

    struct Segment
    {
        int begin = 0;
        int end = 0;
        LinearFitSegment primary;
        LinearFitSegment secondary;
        int slopeClass = 0;
    };

    std::vector<Segment> segments;
    segments.reserve(breakpoints.size() - 1);
    for (std::size_t index = 0; index + 1 < breakpoints.size(); ++index)
    {
        Segment segment;
        segment.begin = breakpoints[index];
        segment.end = breakpoints[index + 1];
        segment.primary = FitRange(primarySums, segment.begin, segment.end);
        segment.secondary = FitRange(secondarySums, segment.begin, segment.end);
        if (!segment.primary.valid || !segment.secondary.valid)
        {
            result.error = "piecewise fit contains an invalid segment.";
            return false;
        }
        segments.push_back(segment);
    }

    if (trapezoidTemplate && !segments.empty())
    {
        const bool primaryIsTemplate = ValueRange(primaryTargets) >= ValueRange(secondaryTargets);
        std::vector<double> nonHorizontalSlopes;
        for (const Segment& segment : segments)
        {
            const double slope = primaryIsTemplate ? segment.primary.slope : segment.secondary.slope;
            if (std::abs(slope) > 1e-6)
            {
                nonHorizontalSlopes.push_back(std::abs(slope));
            }
        }
        double dominantSlope = nonHorizontalSlopes.empty() ? 0.0 : Median(nonHorizontalSlopes);
        const double horizontalThreshold = dominantSlope > 1e-6 ? std::max(0.02, dominantSlope * 0.35) : 0.05;
        for (Segment& segment : segments)
        {
            LinearFitSegment& model = primaryIsTemplate ? segment.primary : segment.secondary;
            segment.slopeClass = std::abs(model.slope) <= horizontalThreshold ? 0 : (model.slope > 0.0 ? 1 : -1);
        }
        nonHorizontalSlopes.clear();
        for (const Segment& segment : segments)
        {
            const LinearFitSegment& model = primaryIsTemplate ? segment.primary : segment.secondary;
            if (std::abs(model.slope) > horizontalThreshold)
            {
                nonHorizontalSlopes.push_back(std::abs(model.slope));
            }
        }
        if (!nonHorizontalSlopes.empty())
        {
            dominantSlope = Median(nonHorizontalSlopes);
        }
        for (Segment& segment : segments)
        {
            LinearFitSegment& model = primaryIsTemplate ? segment.primary : segment.secondary;
            const double axisMid = (axes[static_cast<std::size_t>(segment.begin)] + axes[static_cast<std::size_t>(segment.end)]) * 0.5;
            const double fitMid = model.slope * axisMid + model.intercept;
            model.slope = static_cast<double>(segment.slopeClass) * dominantSlope;
            model.intercept = fitMid - model.slope * axisMid;
        }
    }

    std::vector<double> segmentStartAxes;
    std::vector<double> segmentEndAxes;
    std::vector<Point3D> segmentStartPoints;
    std::vector<Point3D> segmentEndPoints;
    for (const Segment& segment : segments)
    {
        const double startAxis = axes[static_cast<std::size_t>(segment.begin)];
        const double endAxis = axes[static_cast<std::size_t>(segment.end)];
        segmentStartAxes.push_back(startAxis);
        segmentEndAxes.push_back(endAxis);
        segmentStartPoints.push_back(EvaluateLineModelPoint(params.sampleAxis, startAxis, segment.primary, segment.secondary));
        segmentEndPoints.push_back(EvaluateLineModelPoint(params.sampleAxis, endAxis, segment.primary, segment.secondary));
    }

    // Join adjacent fitted segments at a shared midpoint so the exported weld path is continuous.
    for (std::size_t segment = 0; segment + 1 < segments.size(); ++segment)
    {
        const double jointAxis = axes[static_cast<std::size_t>(segments[segment].end)];
        const Point3D current = EvaluateLineModelPoint(params.sampleAxis, jointAxis, segments[segment].primary, segments[segment].secondary);
        const Point3D next = EvaluateLineModelPoint(params.sampleAxis, jointAxis, segments[segment + 1].primary, segments[segment + 1].secondary);
        const Point3D merged = (current + next) * 0.5;
        segmentEndPoints[segment] = merged;
        segmentStartPoints[segment + 1] = merged;
    }

    for (int index = 0; index < static_cast<int>(result.points.size()); ++index)
    {
        const int fitPointCount = static_cast<int>(fitPoints.size());
        int localIndex = std::clamp(index - fitBegin, 0, fitPointCount - 1);
        std::size_t matchedSegment = 0;
        while (matchedSegment + 1 < segments.size() && localIndex > segments[matchedSegment].end)
        {
            ++matchedSegment;
        }
        const double axisValue = AxisValue(result.points[static_cast<std::size_t>(index)].point, params.sampleAxis);
        double ratio = 0.0;
        if (std::abs(segmentEndAxes[matchedSegment] - segmentStartAxes[matchedSegment]) > kEpsilon)
        {
            ratio = (axisValue - segmentStartAxes[matchedSegment])
                / (segmentEndAxes[matchedSegment] - segmentStartAxes[matchedSegment]);
        }
        ratio = std::clamp(ratio, 0.0, 1.0);
        Point3D fitted = segmentStartPoints[matchedSegment]
            + (segmentEndPoints[matchedSegment] - segmentStartPoints[matchedSegment]) * ratio;
        fitted = SetAxisValue(fitted, params.sampleAxis, axisValue);
        result.points[static_cast<std::size_t>(index)].point = fitted;
        result.points[static_cast<std::size_t>(index)].source =
            (trapezoidTemplate ? "trapfit" : "piecefit") + std::to_string(matchedSegment + 1);
    }
    result.fitSegmentCount = static_cast<int>(segments.size());
    return true;
}

void CountClassification(ClassificationResult& result)
{
    result.startCount = 0;
    result.endCount = 0;
    result.innerCornerCount = 0;
    result.outerCornerCount = 0;
    result.normalCount = 0;
    result.noiseCount = 0;
    for (const ClassifiedPoint& point : result.points)
    {
        switch (point.type)
        {
        case WeldPointType::Start:
            ++result.startCount;
            break;
        case WeldPointType::End:
            ++result.endCount;
            break;
        case WeldPointType::InnerCorner:
            ++result.innerCornerCount;
            break;
        case WeldPointType::OuterCorner:
            ++result.outerCornerCount;
            break;
        case WeldPointType::Noise:
            ++result.noiseCount;
            break;
        case WeldPointType::Normal:
        default:
            ++result.normalCount;
            break;
        }
    }
}

ClassificationResult ClassifyLowerWeldPoints(const FilterResult& filterResult, SampleAxis sampleAxis)
{
    ClassificationResult result;
    if (!filterResult.ok)
    {
        result.error = filterResult.error.empty() ? "filter result is invalid." : filterResult.error;
        return result;
    }
    if (filterResult.points.empty())
    {
        result.error = "filter result is empty.";
        return result;
    }

    std::vector<double> primaryValues;
    primaryValues.reserve(filterResult.points.size());
    result.points.reserve(filterResult.points.size());
    for (const FilterPoint& point : filterResult.points)
    {
        result.points.push_back({ point.index, point.point, WeldPointType::Normal, point.source, {} });
        primaryValues.push_back(PrimaryValue(point.point, sampleAxis));
    }

    const double lowRiseThreshold = Percentile(primaryValues, 0.30);
    const double lowFallThreshold = Percentile(primaryValues, 0.32);
    const double highThreshold = Percentile(primaryValues, 0.64);
    constexpr double startFlatSlopeThreshold = 0.25;
    constexpr double risingSlopeThreshold = 0.40;
    constexpr double fallingSlopeThreshold = -0.45;

    std::vector<double> slopes;
    slopes.reserve(result.points.size());
    for (int index = 0; index < static_cast<int>(result.points.size()); ++index)
    {
        const int previous = std::max(0, index - 1);
        const int next = std::min(static_cast<int>(result.points.size()) - 1, index + 1);
        const double previousAxis = AxisValue(result.points[static_cast<std::size_t>(previous)].point, sampleAxis);
        const double nextAxis = AxisValue(result.points[static_cast<std::size_t>(next)].point, sampleAxis);
        const double deltaAxis = nextAxis - previousAxis;
        slopes.push_back(std::abs(deltaAxis) < 1e-6
            ? 0.0
            : (primaryValues[static_cast<std::size_t>(next)] - primaryValues[static_cast<std::size_t>(previous)]) / deltaAxis);
    }

    enum class Phase
    {
        SeekingStart,
        LowPlatform,
        RisingEdge,
        HighPlatform,
        FallingEdge
    };

    Phase phase = Phase::SeekingStart;
    bool startAssigned = false;
    for (int index = 0; index < static_cast<int>(result.points.size()); ++index)
    {
        const double primary = primaryValues[static_cast<std::size_t>(index)];
        const double slope = slopes[static_cast<std::size_t>(index)];
        if (!startAssigned)
        {
            if (primary <= lowFallThreshold && std::abs(slope) <= startFlatSlopeThreshold)
            {
                result.points[static_cast<std::size_t>(index)].type = WeldPointType::Start;
                startAssigned = true;
                phase = Phase::LowPlatform;
            }
            continue;
        }

        switch (phase)
        {
        case Phase::LowPlatform:
            if (slope >= risingSlopeThreshold && primary >= lowRiseThreshold)
            {
                result.points[static_cast<std::size_t>(index)].type = WeldPointType::InnerCorner;
                phase = Phase::RisingEdge;
            }
            break;
        case Phase::RisingEdge:
            if (primary >= highThreshold)
            {
                result.points[static_cast<std::size_t>(index)].type = WeldPointType::OuterCorner;
                phase = Phase::HighPlatform;
            }
            break;
        case Phase::HighPlatform:
            if (slope <= fallingSlopeThreshold && primary >= highThreshold)
            {
                result.points[static_cast<std::size_t>(index)].type = WeldPointType::OuterCorner;
                phase = Phase::FallingEdge;
            }
            break;
        case Phase::FallingEdge:
            if (primary <= lowFallThreshold)
            {
                result.points[static_cast<std::size_t>(index)].type = WeldPointType::InnerCorner;
                phase = Phase::LowPlatform;
            }
            break;
        case Phase::SeekingStart:
        default:
            break;
        }
    }

    if (!startAssigned)
    {
        result.points.front().type = WeldPointType::Start;
    }
    if (result.points.size() >= 2)
    {
        result.points.back().type = WeldPointType::End;
    }
    CountClassification(result);
    result.ok = true;
    return result;
}

struct ProjectedPoint
{
    int inputIndex = 0;
    Point3D point;
    double s = 0.0;
    double h = 0.0;
    double n = 0.0;
    double smoothH = 0.0;
    double smoothN = 0.0;
};

struct Axes
{
    Point3D center;
    Point3D main;
    Point3D side;
    Point3D normal;
};

Point3D UnitForAxis(SampleAxis axis)
{
    return axis == SampleAxis::AxisX ? Point3D{ 1.0, 0.0, 0.0 } : Point3D{ 0.0, 1.0, 0.0 };
}

Axes BuildAxes(const std::vector<IndexedPoint3D>& validPoints, SampleAxis axis)
{
    Axes axes;
    for (const IndexedPoint3D& point : validPoints)
    {
        axes.center = axes.center + point.point;
    }
    axes.center = axes.center / static_cast<double>(std::max<std::size_t>(1, validPoints.size()));

    Point3D scanDelta = validPoints.empty() ? Point3D() : validPoints.back().point - validPoints.front().point;
    axes.main = Normalize(scanDelta, UnitForAxis(axis));
    if (Dot(axes.main, UnitForAxis(axis)) < 0.0)
    {
        axes.main = axes.main * -1.0;
    }

    Point3D preferredSide = axis == SampleAxis::AxisY ? Point3D{ 1.0, 0.0, 0.0 } : Point3D{ 0.0, 1.0, 0.0 };
    preferredSide = preferredSide - axes.main * Dot(preferredSide, axes.main);
    axes.side = Normalize(preferredSide, axis == SampleAxis::AxisY ? Point3D{ 1.0, 0.0, 0.0 } : Point3D{ 0.0, 1.0, 0.0 });
    axes.normal = Normalize(Cross(axes.main, axes.side), Point3D{ 0.0, 0.0, 1.0 });
    axes.side = Normalize(Cross(axes.normal, axes.main), axes.side);
    return axes;
}

std::vector<ProjectedPoint> ProjectGeometryPoints(
    const std::vector<IndexedPoint3D>& validPoints,
    const Axes& axes,
    int smoothRadius)
{
    std::vector<ProjectedPoint> projected;
    projected.reserve(validPoints.size());
    for (const IndexedPoint3D& sample : validPoints)
    {
        const Point3D delta = sample.point - axes.center;
        ProjectedPoint point;
        point.inputIndex = sample.index;
        point.point = sample.point;
        point.s = Dot(delta, axes.main);
        point.h = Dot(delta, axes.side);
        point.n = Dot(delta, axes.normal);
        point.smoothH = point.h;
        point.smoothN = point.n;
        projected.push_back(point);
    }

    std::sort(projected.begin(), projected.end(),
        [](const ProjectedPoint& left, const ProjectedPoint& right)
        {
            return left.s < right.s;
        });

    const int radius = std::max(1, std::min(8, smoothRadius));
    const std::vector<ProjectedPoint> source = projected;
    for (int index = 0; index < static_cast<int>(projected.size()); ++index)
    {
        const int begin = std::max(0, index - radius);
        const int end = std::min(static_cast<int>(projected.size()) - 1, index + radius);
        double sideSum = 0.0;
        double normalSum = 0.0;
        int count = 0;
        for (int sample = begin; sample <= end; ++sample)
        {
            sideSum += source[static_cast<std::size_t>(sample)].h;
            normalSum += source[static_cast<std::size_t>(sample)].n;
            ++count;
        }
        projected[static_cast<std::size_t>(index)].smoothH = sideSum / static_cast<double>(count);
        projected[static_cast<std::size_t>(index)].smoothN = normalSum / static_cast<double>(count);
    }
    return projected;
}

std::vector<ProjectedPoint> RemoveProjectedBranchOutliers(
    const std::vector<ProjectedPoint>& projected,
    const FilterFitParams& params,
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

    const int radius = std::max(12, std::min(48, params.smoothRadius * 8 + 16));
    const int guard = std::max(3, radius / 5);
    const double baseThreshold = std::max(1.6, std::min(6.0, params.piecewiseFitTolerance * 1.25 + 0.7));
    std::vector<char> keep(projected.size(), 1);
    int rejected = 0;

    for (int index = guard; index + guard < static_cast<int>(projected.size()); ++index)
    {
        const int leftBegin = std::max(0, index - radius);
        const int leftEnd = index - guard;
        const int rightBegin = index + guard;
        const int rightEnd = std::min(static_cast<int>(projected.size()) - 1, index + radius);
        if (leftEnd <= leftBegin || rightEnd <= rightBegin)
        {
            continue;
        }

        std::vector<double> baselineValues;
        baselineValues.reserve(static_cast<std::size_t>((leftEnd - leftBegin + 1) + (rightEnd - rightBegin + 1)));
        for (int sample = leftBegin; sample <= leftEnd; ++sample)
        {
            baselineValues.push_back(projected[static_cast<std::size_t>(sample)].smoothH);
        }
        for (int sample = rightBegin; sample <= rightEnd; ++sample)
        {
            baselineValues.push_back(projected[static_cast<std::size_t>(sample)].smoothH);
        }
        const double baseline = Median(baselineValues);
        std::vector<double> deviations;
        deviations.reserve(baselineValues.size());
        for (double value : baselineValues)
        {
            deviations.push_back(std::abs(value - baseline));
        }
        const double threshold = std::max(baseThreshold, std::min(8.0, Median(deviations) * 4.0 + 0.8));
        if (std::abs(projected[static_cast<std::size_t>(index)].smoothH - baseline) > threshold * 1.8)
        {
            keep[static_cast<std::size_t>(index)] = 0;
            ++rejected;
        }
    }

    if (rejected <= 0 || projected.size() - static_cast<std::size_t>(rejected) < 16)
    {
        return projected;
    }
    std::vector<ProjectedPoint> filtered;
    filtered.reserve(projected.size() - static_cast<std::size_t>(rejected));
    for (std::size_t index = 0; index < projected.size(); ++index)
    {
        if (keep[index])
        {
            filtered.push_back(projected[index]);
        }
    }
    if (rejectedCount != nullptr)
    {
        *rejectedCount = rejected;
    }
    return filtered;
}

struct Line2D
{
    double sx = 0.0;
    double hy = 0.0;
    double dirS = 1.0;
    double dirH = 0.0;
    bool valid = false;
};

struct ScalarLine
{
    double centerS = 0.0;
    double centerValue = 0.0;
    double slope = 0.0;
    bool valid = false;

    double ValueAt(double station) const
    {
        return centerValue + slope * (station - centerS);
    }
};

double Cross2D(double ax, double ay, double bx, double by)
{
    return ax * by - ay * bx;
}

double Dot2D(double ax, double ay, double bx, double by)
{
    return ax * bx + ay * by;
}

double NormalizeLineDirection(Line2D& line)
{
    const double length = std::hypot(line.dirS, line.dirH);
    if (length <= kEpsilon || !std::isfinite(length))
    {
        line.valid = false;
        return 0.0;
    }
    line.dirS /= length;
    line.dirH /= length;
    return length;
}

Line2D FitLine2D(const std::vector<ProjectedPoint>& points, int begin, int end)
{
    Line2D line;
    begin = std::max(0, std::min(begin, end));
    end = std::min(static_cast<int>(points.size()) - 1, std::max(begin, end));
    if (end - begin + 1 < 2)
    {
        return line;
    }

    std::vector<double> ss;
    std::vector<double> hs;
    ss.reserve(static_cast<std::size_t>(end - begin + 1));
    hs.reserve(ss.capacity());
    for (int index = begin; index <= end; ++index)
    {
        ss.push_back(points[static_cast<std::size_t>(index)].s);
        hs.push_back(points[static_cast<std::size_t>(index)].smoothH);
    }
    const std::pair<double, double> fit = LinearFit1D(ss, hs);
    const double centerS = (points[static_cast<std::size_t>(begin)].s + points[static_cast<std::size_t>(end)].s) * 0.5;
    line.sx = centerS;
    line.hy = fit.first * centerS + fit.second;
    line.dirS = 1.0;
    line.dirH = fit.first;
    if (NormalizeLineDirection(line) <= kEpsilon)
    {
        return Line2D();
    }
    line.valid = true;
    return line;
}

Line2D OrientLineDirection(Line2D line, const Line2D& directionHint)
{
    if (line.valid && directionHint.valid
        && Dot2D(line.dirS, line.dirH, directionHint.dirS, directionHint.dirH) < 0.0)
    {
        line.dirS = -line.dirS;
        line.dirH = -line.dirH;
    }
    return line;
}

double LineDirectionAngle(const Line2D& first, const Line2D& second)
{
    if (!first.valid || !second.valid)
    {
        return 0.0;
    }
    const double dot = std::clamp(std::abs(Dot2D(first.dirS, first.dirH, second.dirS, second.dirH)), 0.0, 1.0);
    return std::acos(dot);
}

double PointLineDistance2D(double s, double h, const Line2D& line)
{
    if (!line.valid)
    {
        return 0.0;
    }
    const double dx = s - line.sx;
    const double dy = h - line.hy;
    return std::abs(dx * line.dirH - dy * line.dirS);
}

double DistanceToLine2D(const ProjectedPoint& point, const Line2D& line)
{
    return PointLineDistance2D(point.s, point.smoothH, line);
}

Line2D FitIndexedLine2D(
    const std::vector<ProjectedPoint>& points,
    const std::vector<int>& indexes,
    const Line2D& directionHint)
{
    Line2D line;
    std::vector<double> ss;
    std::vector<double> hs;
    ss.reserve(indexes.size());
    hs.reserve(indexes.size());
    for (int index : indexes)
    {
        if (index >= 0 && index < static_cast<int>(points.size()))
        {
            ss.push_back(points[static_cast<std::size_t>(index)].s);
            hs.push_back(points[static_cast<std::size_t>(index)].smoothH);
        }
    }
    if (ss.size() < 2)
    {
        return line;
    }

    const std::pair<double, double> fit = LinearFit1D(ss, hs);
    const double centerS = Median(ss);
    line.sx = centerS;
    line.hy = fit.first * centerS + fit.second;
    line.dirS = 1.0;
    line.dirH = fit.first;
    if (NormalizeLineDirection(line) <= kEpsilon)
    {
        return Line2D();
    }
    line.valid = true;
    return OrientLineDirection(line, directionHint);
}

Line2D FitRobustLine2D(const std::vector<ProjectedPoint>& points, int begin, int end)
{
    begin = std::max(0, std::min(begin, end));
    end = std::min(static_cast<int>(points.size()) - 1, std::max(begin, end));
    Line2D line = FitLine2D(points, begin, end);
    if (!line.valid || end - begin + 1 < 8)
    {
        return line;
    }

    std::vector<double> residuals;
    residuals.reserve(static_cast<std::size_t>(end - begin + 1));
    for (int index = begin; index <= end; ++index)
    {
        residuals.push_back(DistanceToLine2D(points[static_cast<std::size_t>(index)], line));
    }
    const double medianResidual = Median(residuals);
    std::vector<double> deviations;
    deviations.reserve(residuals.size());
    for (double residual : residuals)
    {
        deviations.push_back(std::abs(residual - medianResidual));
    }
    const double residualLimit = medianResidual + std::max(0.8, Median(deviations) * 3.5);

    std::vector<int> accepted;
    accepted.reserve(residuals.size());
    for (int index = begin; index <= end; ++index)
    {
        if (DistanceToLine2D(points[static_cast<std::size_t>(index)], line) <= residualLimit)
        {
            accepted.push_back(index);
        }
    }
    if (accepted.size() >= std::max<std::size_t>(4, static_cast<std::size_t>(end - begin + 1) / 2))
    {
        const Line2D refinedLine = FitIndexedLine2D(points, accepted, line);
        if (refinedLine.valid)
        {
            line = refinedLine;
        }
    }
    return line;
}

Line2D FitSlopeConsistentSegmentCoreLine(
    const std::vector<ProjectedPoint>& projected,
    int firstIndex,
    int lastIndex)
{
    if (projected.empty())
    {
        return Line2D();
    }

    const int begin = std::max(0, std::min(firstIndex, lastIndex));
    const int end = std::min(static_cast<int>(projected.size()) - 1, std::max(firstIndex, lastIndex));
    const Line2D fallbackLine = FitRobustLine2D(projected, begin, end);
    const int pointCount = end - begin + 1;
    if (pointCount < 8)
    {
        return fallbackLine;
    }

    Line2D segmentHint;
    segmentHint.sx = projected[static_cast<std::size_t>(begin)].s;
    segmentHint.hy = projected[static_cast<std::size_t>(begin)].smoothH;
    segmentHint.dirS = projected[static_cast<std::size_t>(end)].s - projected[static_cast<std::size_t>(begin)].s;
    segmentHint.dirH = projected[static_cast<std::size_t>(end)].smoothH - projected[static_cast<std::size_t>(begin)].smoothH;
    if (NormalizeLineDirection(segmentHint) <= kEpsilon)
    {
        segmentHint = fallbackLine;
    }
    else
    {
        segmentHint.valid = true;
    }
    if (!segmentHint.valid)
    {
        return fallbackLine;
    }

    struct SlopeWindow
    {
        Line2D line;
        double weight = 1.0;
        int begin = 0;
        int end = 0;
    };

    std::vector<SlopeWindow> windows;
    const int windowPoints = std::max(5, std::min(24, std::max(5, pointCount / 5)));
    const int stepPoints = std::max(2, windowPoints / 2);
    for (int windowBegin = begin; windowBegin <= end - 3; windowBegin += stepPoints)
    {
        const int windowEnd = std::min(end, windowBegin + windowPoints - 1);
        if (windowEnd - windowBegin + 1 < 4)
        {
            continue;
        }

        Line2D windowLine = FitLine2D(projected, windowBegin, windowEnd);
        if (!windowLine.valid)
        {
            continue;
        }

        SlopeWindow window;
        window.line = OrientLineDirection(windowLine, segmentHint);
        window.weight = std::max(1.0,
            std::abs(projected[static_cast<std::size_t>(windowEnd)].s - projected[static_cast<std::size_t>(windowBegin)].s));
        window.begin = windowBegin;
        window.end = windowEnd;
        windows.push_back(window);
    }
    if (windows.size() < 2)
    {
        return fallbackLine;
    }

    constexpr double kPi = 3.14159265358979323846;
    const double angleLimit = 14.0 * kPi / 180.0;
    int bestWindowIndex = -1;
    double bestWeight = -1.0;
    for (int candidateIndex = 0; candidateIndex < static_cast<int>(windows.size()); ++candidateIndex)
    {
        double clusterWeight = 0.0;
        for (const SlopeWindow& window : windows)
        {
            if (LineDirectionAngle(window.line, windows[static_cast<std::size_t>(candidateIndex)].line) <= angleLimit)
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

    const double totalSpan = std::abs(projected[static_cast<std::size_t>(end)].s - projected[static_cast<std::size_t>(begin)].s);
    const double cornerGuard = std::max(3.0, std::min(12.0, totalSpan * 0.10));
    const bool enableCornerGuard = totalSpan >= cornerGuard * 4.0;
    std::vector<char> accepted(static_cast<std::size_t>(pointCount), 0);
    const Line2D& dominantLine = windows[static_cast<std::size_t>(bestWindowIndex)].line;
    for (const SlopeWindow& window : windows)
    {
        if (LineDirectionAngle(window.line, dominantLine) > angleLimit)
        {
            continue;
        }
        for (int index = window.begin; index <= window.end; ++index)
        {
            if (enableCornerGuard
                && (std::abs(projected[static_cast<std::size_t>(index)].s - projected[static_cast<std::size_t>(begin)].s) < cornerGuard
                    || std::abs(projected[static_cast<std::size_t>(index)].s - projected[static_cast<std::size_t>(end)].s) < cornerGuard))
            {
                continue;
            }
            accepted[static_cast<std::size_t>(index - begin)] = 1;
        }
    }

    std::vector<int> acceptedIndexes;
    acceptedIndexes.reserve(static_cast<std::size_t>(pointCount));
    for (int index = begin; index <= end; ++index)
    {
        if (accepted[static_cast<std::size_t>(index - begin)])
        {
            acceptedIndexes.push_back(index);
        }
    }

    const int minAcceptedCount = std::max(5, std::min(16, pointCount / 3));
    const double minAcceptedSpan = std::max(8.0, std::min(28.0, totalSpan * 0.30));
    const double acceptedSpan = acceptedIndexes.size() >= 2
        ? std::abs(projected[static_cast<std::size_t>(acceptedIndexes.back())].s
            - projected[static_cast<std::size_t>(acceptedIndexes.front())].s)
        : 0.0;
    if (static_cast<int>(acceptedIndexes.size()) < minAcceptedCount || acceptedSpan < minAcceptedSpan)
    {
        return fallbackLine;
    }

    const Line2D coreLine = FitIndexedLine2D(projected, acceptedIndexes, segmentHint);
    return coreLine.valid ? coreLine : fallbackLine;
}

Line2D FitLocalSegmentLineWithSpan(
    const std::vector<ProjectedPoint>& projected,
    int firstIndex,
    int lastIndex,
    bool useLastSide,
    double localSpan,
    bool useSlopeConsistentCore)
{
    if (projected.empty())
    {
        return Line2D();
    }

    int begin = std::max(0, std::min(firstIndex, lastIndex));
    int end = std::min(static_cast<int>(projected.size()) - 1, std::max(firstIndex, lastIndex));
    if (end <= begin + 2)
    {
        return FitLine2D(projected, begin, end);
    }

    const double totalSpan = std::abs(projected[static_cast<std::size_t>(end)].s - projected[static_cast<std::size_t>(begin)].s);
    const double clampedLocalSpan = std::max(8.0, std::min(80.0, localSpan));
    if (totalSpan > clampedLocalSpan)
    {
        if (useLastSide)
        {
            while (begin + 4 < end
                && std::abs(projected[static_cast<std::size_t>(end)].s - projected[static_cast<std::size_t>(begin)].s)
                    > clampedLocalSpan)
            {
                ++begin;
            }
        }
        else
        {
            while (begin + 4 < end
                && std::abs(projected[static_cast<std::size_t>(end)].s - projected[static_cast<std::size_t>(begin)].s)
                    > clampedLocalSpan)
            {
                --end;
            }
        }
    }

    const Line2D fallbackLine = FitRobustLine2D(projected, begin, end);
    if (!useSlopeConsistentCore)
    {
        return fallbackLine;
    }
    return FitSlopeConsistentSegmentCoreLine(projected, begin, end);
}

std::vector<Line2D> BuildGeometrySegmentLines(
    const std::vector<ProjectedPoint>& projected,
    const std::vector<int>& keyIndexes,
    bool useSlopeConsistentCore)
{
    std::vector<Line2D> segmentLines;
    if (keyIndexes.size() < 2)
    {
        return segmentLines;
    }
    segmentLines.reserve(keyIndexes.size() - 1);
    for (int index = 0; index + 1 < static_cast<int>(keyIndexes.size()); ++index)
    {
        const int begin = keyIndexes[static_cast<std::size_t>(index)];
        const int end = keyIndexes[static_cast<std::size_t>(index + 1)];
        segmentLines.push_back(useSlopeConsistentCore
            ? FitSlopeConsistentSegmentCoreLine(projected, begin, end)
            : FitRobustLine2D(projected, begin, end));
    }
    return segmentLines;
}

std::vector<ProjectedPoint> RemoveSlopeWaveOutliers(
    const std::vector<ProjectedPoint>& projected,
    const FilterFitParams& params,
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

    const int window = std::max(8, params.piecewiseMinSegmentPoints);
    const double residualBase = 0.8;
    std::vector<char> keep(projected.size(), 1);
    int rejected = 0;

    for (int index = window; index + window < static_cast<int>(projected.size()); ++index)
    {
        const Line2D line = FitLine2D(projected, index - window, index + window);
        std::vector<double> residuals;
        residuals.reserve(static_cast<std::size_t>(window * 2 + 1));
        for (int sample = index - window; sample <= index + window; ++sample)
        {
            residuals.push_back(DistanceToLine2D(projected[static_cast<std::size_t>(sample)], line));
        }
        const double medianResidual = Median(residuals);
        std::vector<double> deviations;
        deviations.reserve(residuals.size());
        for (double residual : residuals)
        {
            deviations.push_back(std::abs(residual - medianResidual));
        }
        const double threshold = std::max(residualBase, Median(deviations) * 4.0 + 0.5);
        if (DistanceToLine2D(projected[static_cast<std::size_t>(index)], line) > medianResidual + threshold)
        {
            keep[static_cast<std::size_t>(index)] = 0;
            ++rejected;
        }
    }

    if (rejected <= 0 || projected.size() - static_cast<std::size_t>(rejected) < 16)
    {
        return projected;
    }
    std::vector<ProjectedPoint> filtered;
    filtered.reserve(projected.size() - static_cast<std::size_t>(rejected));
    for (std::size_t index = 0; index < projected.size(); ++index)
    {
        if (keep[index])
        {
            filtered.push_back(projected[index]);
        }
    }
    if (rejectedCount != nullptr)
    {
        *rejectedCount = rejected;
    }
    return filtered;
}

double GeometryDistanceToSegment2D(
    const std::vector<ProjectedPoint>& points,
    int first,
    int last,
    int index)
{
    const double ax = points[static_cast<std::size_t>(first)].s;
    const double ay = points[static_cast<std::size_t>(first)].smoothH;
    const double bx = points[static_cast<std::size_t>(last)].s;
    const double by = points[static_cast<std::size_t>(last)].smoothH;
    const double px = points[static_cast<std::size_t>(index)].s;
    const double py = points[static_cast<std::size_t>(index)].smoothH;
    const double dx = bx - ax;
    const double dy = by - ay;
    const double lengthSquared = dx * dx + dy * dy;
    if (lengthSquared <= kEpsilon)
    {
        return std::hypot(px - ax, py - ay);
    }
    const double t = std::clamp(((px - ax) * dx + (py - ay) * dy) / lengthSquared, 0.0, 1.0);
    return std::hypot(px - (ax + dx * t), py - (ay + dy * t));
}

void DouglasPeucker(
    const std::vector<ProjectedPoint>& points,
    int first,
    int last,
    double tolerance,
    std::vector<char>& keep)
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
        keep[static_cast<std::size_t>(maxIndex)] = 1;
        DouglasPeucker(points, first, maxIndex, tolerance, keep);
        DouglasPeucker(points, maxIndex, last, tolerance, keep);
    }
}

std::vector<int> BuildGeometryKeyIndexes(const std::vector<ProjectedPoint>& projected, const FilterFitParams& params)
{
    std::vector<int> keyIndexes;
    if (projected.empty())
    {
        return keyIndexes;
    }
    if (projected.size() == 1)
    {
        return { 0 };
    }

    double tolerance = std::max(0.5, std::min(8.0, params.piecewiseFitTolerance > 0.0 ? params.piecewiseFitTolerance : 2.0));
    if (params.geometryStrategy == GeometryStrategy::RobustSegmentedKeys)
    {
        tolerance = std::max(0.45, tolerance * 0.65);
    }
    std::vector<char> keep(projected.size(), 0);
    keep.front() = 1;
    keep.back() = 1;
    DouglasPeucker(projected, 0, static_cast<int>(projected.size()) - 1, tolerance, keep);
    for (int index = 0; index < static_cast<int>(keep.size()); ++index)
    {
        if (keep[static_cast<std::size_t>(index)])
        {
            keyIndexes.push_back(index);
        }
    }

    const double minSegmentLength = std::max(4.0, params.sampleStep * 2.0);
    bool removed = true;
    while (removed && keyIndexes.size() > 2)
    {
        removed = false;
        for (std::size_t index = 1; index + 1 < keyIndexes.size(); ++index)
        {
            const Point3D previous = projected[static_cast<std::size_t>(keyIndexes[index - 1])].point;
            const Point3D current = projected[static_cast<std::size_t>(keyIndexes[index])].point;
            const Point3D next = projected[static_cast<std::size_t>(keyIndexes[index + 1])].point;
            if (Norm(current - previous) < minSegmentLength || Norm(next - current) < minSegmentLength)
            {
                keyIndexes.erase(keyIndexes.begin() + static_cast<std::ptrdiff_t>(index));
                removed = true;
                break;
            }
        }
    }
    if (keyIndexes.size() < 2)
    {
        keyIndexes = { 0, static_cast<int>(projected.size()) - 1 };
    }
    return keyIndexes;
}

WeldPointType GeometryCornerType(const std::vector<ProjectedPoint>& projected, const std::vector<int>& keyIndexes, int keyPosition)
{
    if (keyPosition <= 0)
    {
        return WeldPointType::Start;
    }
    if (keyPosition >= static_cast<int>(keyIndexes.size()) - 1)
    {
        return WeldPointType::End;
    }
    const double previous = projected[static_cast<std::size_t>(keyIndexes[static_cast<std::size_t>(keyPosition - 1)])].smoothH;
    const double current = projected[static_cast<std::size_t>(keyIndexes[static_cast<std::size_t>(keyPosition)])].smoothH;
    const double next = projected[static_cast<std::size_t>(keyIndexes[static_cast<std::size_t>(keyPosition + 1)])].smoothH;
    return current - (previous + next) * 0.5 >= 0.0
        ? WeldPointType::InnerCorner
        : WeldPointType::OuterCorner;
}

std::vector<int> PruneShortSameTypeRuns(
    const std::vector<ProjectedPoint>& projected,
    std::vector<int> keyIndexes,
    const FilterFitParams& params)
{
    if (projected.empty() || keyIndexes.size() <= 4)
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
        for (int position = 1; position + 2 < static_cast<int>(keyIndexes.size()) - 1 && !removed;)
        {
            const WeldPointType runType = GeometryCornerType(projected, keyIndexes, position);
            int runEnd = position;
            while (runEnd + 1 < static_cast<int>(keyIndexes.size()) - 1
                && GeometryCornerType(projected, keyIndexes, runEnd + 1) == runType)
            {
                ++runEnd;
            }
            if (runEnd - position + 1 < 3)
            {
                position = runEnd + 1;
                continue;
            }

            for (int candidate = position; candidate < runEnd; ++candidate)
            {
                const double span = Norm(
                    projected[static_cast<std::size_t>(keyIndexes[static_cast<std::size_t>(candidate + 1)])].point
                    - projected[static_cast<std::size_t>(keyIndexes[static_cast<std::size_t>(candidate)])].point);
                if (span < shortSameTypeSpan)
                {
                    keyIndexes.erase(keyIndexes.begin() + candidate);
                    removed = true;
                    break;
                }
            }
            if (!removed)
            {
                position = runEnd + 1;
            }
        }
    }
    return keyIndexes;
}

bool IntersectLines(const Line2D& first, const Line2D& second, double* outS, double* outH)
{
    if (!first.valid || !second.valid || outS == nullptr || outH == nullptr)
    {
        return false;
    }
    const double denominator = first.dirS * second.dirH - first.dirH * second.dirS;
    if (std::abs(denominator) < 1e-6)
    {
        return false;
    }
    const double dx = second.sx - first.sx;
    const double dy = second.hy - first.hy;
    const double t = (dx * second.dirH - dy * second.dirS) / denominator;
    *outS = first.sx + first.dirS * t;
    *outH = first.hy + first.dirH * t;
    return std::isfinite(*outS) && std::isfinite(*outH);
}

double ProjectionDistance(double firstS, double firstH, double secondS, double secondH)
{
    return std::hypot(firstS - secondS, firstH - secondH);
}

double NearestProjectionDistance(
    const std::vector<ProjectedPoint>& projected,
    double s,
    double h,
    int begin,
    int end)
{
    if (projected.empty())
    {
        return std::numeric_limits<double>::infinity();
    }
    begin = std::max(0, std::min(begin, end));
    end = std::min(static_cast<int>(projected.size()) - 1, std::max(begin, end));
    double best = std::numeric_limits<double>::infinity();
    for (int index = begin; index <= end; ++index)
    {
        const ProjectedPoint& point = projected[static_cast<std::size_t>(index)];
        best = std::min(best, ProjectionDistance(point.s, point.smoothH, s, h));
    }
    return best;
}

bool LocalMainBand(
    const std::vector<ProjectedPoint>& projected,
    double station,
    double stationRadius,
    double baseThreshold,
    double* outMedianH,
    double* outThreshold)
{
    std::vector<double> values;
    for (const ProjectedPoint& point : projected)
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

    const double medianH = Median(values);
    std::vector<double> deviations;
    deviations.reserve(values.size());
    for (double value : values)
    {
        deviations.push_back(std::abs(value - medianH));
    }
    const double threshold = std::max(baseThreshold, std::min(8.0, Median(deviations) * 4.0 + 0.8));
    int support = 0;
    for (double value : values)
    {
        if (std::abs(value - medianH) <= threshold)
        {
            ++support;
        }
    }
    if (support * 2 < static_cast<int>(values.size()))
    {
        return false;
    }

    if (outMedianH != nullptr)
    {
        *outMedianH = medianH;
    }
    if (outThreshold != nullptr)
    {
        *outThreshold = threshold;
    }
    return true;
}

bool IsCornerProjectionUsable(
    const std::vector<ProjectedPoint>& projected,
    const std::vector<int>& keyIndexes,
    int keyPosition,
    double candidateS,
    double candidateH,
    double referenceS,
    double referenceH)
{
    if (keyPosition <= 0 || keyPosition + 1 >= static_cast<int>(keyIndexes.size()))
    {
        return true;
    }
    if (!std::isfinite(candidateS) || !std::isfinite(candidateH))
    {
        return false;
    }

    const double previousS = projected[static_cast<std::size_t>(keyIndexes[static_cast<std::size_t>(keyPosition - 1)])].s;
    const double nextS = projected[static_cast<std::size_t>(keyIndexes[static_cast<std::size_t>(keyPosition + 1)])].s;
    const double minS = std::min(previousS, nextS);
    const double maxS = std::max(previousS, nextS);
    const double span = std::max(1.0, maxS - minS);
    const double margin = std::max(5.0, std::min(14.0, span * 0.12));
    if (candidateS < minS - margin || candidateS > maxS + margin)
    {
        return false;
    }

    const double maxCornerShift = std::max(5.0, std::min(18.0, span * 0.14));
    if (ProjectionDistance(candidateS, candidateH, referenceS, referenceH) > maxCornerShift)
    {
        return false;
    }

    const int begin = keyIndexes[static_cast<std::size_t>(keyPosition - 1)];
    const int end = keyIndexes[static_cast<std::size_t>(keyPosition + 1)];
    const double maxCloudDistance = std::max(2.5, std::min(6.0, span * 0.05));
    if (NearestProjectionDistance(projected, candidateS, candidateH, begin, end) > maxCloudDistance)
    {
        return false;
    }

    double mainBandH = 0.0;
    double mainBandThreshold = 0.0;
    const double stationRadius = std::max(6.0, std::min(36.0, span * 0.08));
    const double baseThreshold = std::max(1.6, std::min(5.5, span * 0.015 + 0.8));
    if (LocalMainBand(projected, candidateS, stationRadius, baseThreshold, &mainBandH, &mainBandThreshold))
    {
        const double mainBandDelta = std::abs(candidateH - mainBandH);
        if (mainBandDelta >= mainBandThreshold * 1.35
            && mainBandDelta >= baseThreshold * 1.6)
        {
            return false;
        }
    }

    return true;
}

double LocalNormalValue(
    const std::vector<ProjectedPoint>& projected,
    int fallbackIndex,
    double station,
    double stationRadius,
    double baseThreshold)
{
    if (projected.empty())
    {
        return 0.0;
    }

    const int clampedFallback = std::max(0, std::min(fallbackIndex, static_cast<int>(projected.size()) - 1));
    double mainBandH = 0.0;
    double mainBandThreshold = 0.0;
    const bool hasMainBand =
        LocalMainBand(projected, station, stationRadius, baseThreshold, &mainBandH, &mainBandThreshold);

    std::vector<double> values;
    values.reserve(32);
    for (const ProjectedPoint& point : projected)
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
        return Median(values);
    }

    const int indexRadius = 8;
    const int begin = std::max(0, clampedFallback - indexRadius);
    const int end = std::min(static_cast<int>(projected.size()) - 1, clampedFallback + indexRadius);
    values.clear();
    values.reserve(static_cast<std::size_t>(end - begin + 1));
    for (int index = begin; index <= end; ++index)
    {
        values.push_back(projected[static_cast<std::size_t>(index)].smoothN);
    }
    return values.empty() ? projected[static_cast<std::size_t>(clampedFallback)].smoothN : Median(values);
}

ScalarLine FitSegmentNormalLine(const std::vector<ProjectedPoint>& projected, int firstIndex, int lastIndex)
{
    ScalarLine line;
    if (projected.empty())
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

    std::vector<double> normalValues;
    normalValues.reserve(static_cast<std::size_t>(fitCount));
    for (int index = fitBegin; index <= fitEnd; ++index)
    {
        normalValues.push_back(projected[static_cast<std::size_t>(index)].smoothN);
    }

    const double normalMedian = Median(normalValues);
    std::vector<double> deviations;
    deviations.reserve(normalValues.size());
    for (double value : normalValues)
    {
        deviations.push_back(std::abs(value - normalMedian));
    }
    const double normalLimit = std::max(1.2, std::min(8.0, Median(deviations) * 4.0 + 0.8));

    int normalBandSupport = 0;
    for (int index = fitBegin; index <= fitEnd; ++index)
    {
        if (std::abs(projected[static_cast<std::size_t>(index)].smoothN - normalMedian) <= normalLimit)
        {
            ++normalBandSupport;
        }
    }
    const bool useNormalBand = normalBandSupport >= std::max(4, fitCount * 2 / 3);

    std::vector<int> acceptedIndexes;
    acceptedIndexes.reserve(static_cast<std::size_t>(fitCount));
    for (int index = fitBegin; index <= fitEnd; ++index)
    {
        if (!useNormalBand
            || std::abs(projected[static_cast<std::size_t>(index)].smoothN - normalMedian) <= normalLimit)
        {
            acceptedIndexes.push_back(index);
        }
    }

    if (acceptedIndexes.size() < 2)
    {
        line.centerS = projected[static_cast<std::size_t>((fitBegin + fitEnd) / 2)].s;
        line.centerValue = normalMedian;
        line.valid = true;
        return line;
    }

    double meanS = 0.0;
    double meanN = 0.0;
    for (int index : acceptedIndexes)
    {
        meanS += projected[static_cast<std::size_t>(index)].s;
        meanN += projected[static_cast<std::size_t>(index)].smoothN;
    }
    meanS /= static_cast<double>(acceptedIndexes.size());
    meanN /= static_cast<double>(acceptedIndexes.size());

    double numerator = 0.0;
    double denominator = 0.0;
    for (int index : acceptedIndexes)
    {
        const double deltaS = projected[static_cast<std::size_t>(index)].s - meanS;
        numerator += deltaS * (projected[static_cast<std::size_t>(index)].smoothN - meanN);
        denominator += deltaS * deltaS;
    }

    line.centerS = meanS;
    line.centerValue = meanN;
    line.slope = denominator > 1e-9 ? numerator / denominator : 0.0;
    line.valid = true;
    return line;
}

ScalarLine FitLocalSegmentNormalLineWithSpan(
    const std::vector<ProjectedPoint>& projected,
    int firstIndex,
    int lastIndex,
    bool useLastSide,
    double localSpan)
{
    if (projected.empty())
    {
        return ScalarLine();
    }

    int begin = std::max(0, std::min(firstIndex, lastIndex));
    int end = std::min(static_cast<int>(projected.size()) - 1, std::max(firstIndex, lastIndex));
    if (end <= begin + 2)
    {
        return FitSegmentNormalLine(projected, begin, end);
    }

    const double totalSpan = std::abs(projected[static_cast<std::size_t>(end)].s - projected[static_cast<std::size_t>(begin)].s);
    const double clampedLocalSpan = std::max(8.0, std::min(80.0, localSpan));
    if (totalSpan > clampedLocalSpan)
    {
        if (useLastSide)
        {
            while (begin + 4 < end
                && std::abs(projected[static_cast<std::size_t>(end)].s - projected[static_cast<std::size_t>(begin)].s)
                    > clampedLocalSpan)
            {
                ++begin;
            }
        }
        else
        {
            while (begin + 4 < end
                && std::abs(projected[static_cast<std::size_t>(end)].s - projected[static_cast<std::size_t>(begin)].s)
                    > clampedLocalSpan)
            {
                --end;
            }
        }
    }

    return FitSegmentNormalLine(projected, begin, end);
}

Point3D PointFromProjection(const Axes& axes, double s, double h, double n)
{
    return axes.center + axes.main * s + axes.side * h + axes.normal * n;
}

std::vector<Point3D> BuildFittedKeyPoints(
    const std::vector<ProjectedPoint>& projected,
    const std::vector<int>& keyIndexes,
    const Axes& axes,
    bool useSlopeConsistentCornerFit,
    const std::vector<char>& isLapStepKey = std::vector<char>())
{
    std::vector<Point3D> keyPoints;
    keyPoints.reserve(keyIndexes.size());
    const std::vector<Line2D> segmentLines =
        BuildGeometrySegmentLines(projected, keyIndexes, useSlopeConsistentCornerFit);
    for (int position = 0; position < static_cast<int>(keyIndexes.size()); ++position)
    {
        const int keyIndex = keyIndexes[static_cast<std::size_t>(position)];
        double s = projected[static_cast<std::size_t>(keyIndex)].s;
        double h = projected[static_cast<std::size_t>(keyIndex)].smoothH;
        double n = projected[static_cast<std::size_t>(keyIndex)].smoothN;
        // 搭接错位台阶端点：两侧线平行错开、求交无意义，直接取本点原始投影(保留干净 X 台阶)，跳过求交细化。
        const bool isLapStep = position < static_cast<int>(isLapStepKey.size()) && isLapStepKey[static_cast<std::size_t>(position)] != 0;
        if (!isLapStep && position > 0 && position + 1 < static_cast<int>(keyIndexes.size()))
        {
            const double previousS = projected[static_cast<std::size_t>(keyIndexes[static_cast<std::size_t>(position - 1)])].s;
            const double nextS = projected[static_cast<std::size_t>(keyIndexes[static_cast<std::size_t>(position + 1)])].s;
            const double span = std::max(1.0, std::abs(nextS - previousS));
            bool hasBestIntersection = false;
            double bestScore = std::numeric_limits<double>::infinity();
            double bestS = s;
            double bestH = h;
            auto considerIntersection = [&](double candidateS, double candidateH, double shiftWeight)
            {
                if (!IsCornerProjectionUsable(projected, keyIndexes, position, candidateS, candidateH, s, h))
                {
                    return;
                }

                const double cloudDistance = NearestProjectionDistance(
                    projected,
                    candidateS,
                    candidateH,
                    keyIndexes[static_cast<std::size_t>(position - 1)],
                    keyIndexes[static_cast<std::size_t>(position + 1)]);
                const double score = cloudDistance + ProjectionDistance(candidateS, candidateH, s, h) * shiftWeight;
                if (score < bestScore)
                {
                    bestScore = score;
                    bestS = candidateS;
                    bestH = candidateH;
                    hasBestIntersection = true;
                }
            };

            if (position - 1 < static_cast<int>(segmentLines.size()) && position < static_cast<int>(segmentLines.size()))
            {
                double intersectionS = s;
                double intersectionH = h;
                if (IntersectLines(
                    segmentLines[static_cast<std::size_t>(position - 1)],
                    segmentLines[static_cast<std::size_t>(position)],
                    &intersectionS,
                    &intersectionH))
                {
                    considerIntersection(intersectionS, intersectionH, 0.22);
                }
            }

            const std::array<double, 5> candidateSpans = {
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
                    const Line2D leftLine = FitLocalSegmentLineWithSpan(
                        projected,
                        keyIndexes[static_cast<std::size_t>(position - 1)],
                        keyIndex,
                        true,
                        localSpan,
                        false);
                    const Line2D rightLine = FitLocalSegmentLineWithSpan(
                        projected,
                        keyIndex,
                        keyIndexes[static_cast<std::size_t>(position + 1)],
                        false,
                        localSpan,
                        false);
                    double intersectionS = s;
                    double intersectionH = h;
                    if (IntersectLines(leftLine, rightLine, &intersectionS, &intersectionH))
                    {
                        considerIntersection(intersectionS, intersectionH, 0.15);
                    }
                }
            }

            if (hasBestIntersection)
            {
                s = bestS;
                h = bestH;
            }

            const double stationRadius = std::max(6.0, std::min(36.0, span * 0.08));
            const double baseThreshold = std::max(1.6, std::min(5.5, span * 0.015 + 0.8));
            const double localNormalValue = LocalNormalValue(projected, keyIndex, s, stationRadius, baseThreshold);
            std::vector<double> normalCandidates;
            normalCandidates.reserve(12);
            for (double localSpan : candidateSpans)
            {
                const ScalarLine leftNormalLine = FitLocalSegmentNormalLineWithSpan(
                    projected,
                    keyIndexes[static_cast<std::size_t>(position - 1)],
                    keyIndex,
                    true,
                    localSpan);
                const ScalarLine rightNormalLine = FitLocalSegmentNormalLineWithSpan(
                    projected,
                    keyIndex,
                    keyIndexes[static_cast<std::size_t>(position + 1)],
                    false,
                    localSpan);
                if (leftNormalLine.valid)
                {
                    const double value = leftNormalLine.ValueAt(s);
                    if (std::isfinite(value))
                    {
                        normalCandidates.push_back(value);
                    }
                }
                if (rightNormalLine.valid)
                {
                    const double value = rightNormalLine.ValueAt(s);
                    if (std::isfinite(value))
                    {
                        normalCandidates.push_back(value);
                    }
                }
            }

            if (normalCandidates.size() >= 2)
            {
                const double segmentNormalMedian = Median(normalCandidates);
                const double localDelta = std::abs(localNormalValue - segmentNormalMedian);
                if (localDelta <= std::max(1.8, baseThreshold * 1.5))
                {
                    normalCandidates.push_back(localNormalValue);
                }
                n = Median(normalCandidates);
            }
            else
            {
                n = localNormalValue;
            }
        }
        keyPoints.push_back(PointFromProjection(axes, s, h, n));
    }
    return keyPoints;
}

void AppendClassifiedPoint(
    std::vector<ClassifiedPoint>& points,
    int& nextIndex,
    const Point3D& point,
    WeldPointType type,
    const std::string& source)
{
    points.push_back({ nextIndex++, point, type, source, {} });
}

int FindKeyOrdinal(const std::vector<int>& keyIndexes, int pointIndex)
{
    const auto it = std::find(keyIndexes.begin(), keyIndexes.end(), pointIndex);
    return it == keyIndexes.end() ? -1 : static_cast<int>(std::distance(keyIndexes.begin(), it));
}

std::vector<int> BuildTrackKeyIndexes(const std::vector<ExternalTrackPoint>& points)
{
    std::vector<int> indexes;
    for (int index = 0; index < static_cast<int>(points.size()); ++index)
    {
        if (points[static_cast<std::size_t>(index)].type != TrackPointType::Normal)
        {
            indexes.push_back(index);
        }
    }
    if (indexes.empty() || indexes.front() != 0)
    {
        indexes.insert(indexes.begin(), 0);
    }
    if (indexes.back() != static_cast<int>(points.size()) - 1)
    {
        indexes.push_back(static_cast<int>(points.size()) - 1);
    }
    return indexes;
}

std::string SegmentKindFromGeometry(
    const Point3D& begin,
    const Point3D& end,
    SampleAxis axis,
    double lowHighMidpoint,
    double profileRange)
{
    const double dPath = AxisValue(end, axis) - AxisValue(begin, axis);
    const double dProfile = PrimaryValue(end, axis) - PrimaryValue(begin, axis);
    const double slopeThresholdMm = std::max(1.0, profileRange * 0.12);
    const double slopeRatioThreshold = 0.25;
    const bool isSlope = std::abs(dProfile) >= slopeThresholdMm
        && std::abs(dProfile) >= std::abs(dPath) * slopeRatioThreshold;
    if (isSlope)
    {
        return dProfile >= 0.0 ? "rising_edge" : "falling_edge";
    }
    const double middleProfile = (PrimaryValue(begin, axis) + PrimaryValue(end, axis)) * 0.5;
    return middleProfile <= lowHighMidpoint ? "low_platform" : "high_platform";
}

std::vector<std::string> AssignSegmentKinds(
    const std::vector<ExternalTrackPoint>& points,
    const std::vector<int>& keyIndexes,
    SampleAxis axis)
{
    std::vector<std::string> segmentKinds;
    if (keyIndexes.size() < 2)
    {
        return segmentKinds;
    }

    std::vector<double> profileValues;
    profileValues.reserve(points.size());
    for (const ExternalTrackPoint& point : points)
    {
        profileValues.push_back(PrimaryValue(point.point, axis));
    }
    const auto minmax = std::minmax_element(profileValues.begin(), profileValues.end());
    const double minProfile = *minmax.first;
    const double maxProfile = *minmax.second;
    const double range = std::max(0.0, maxProfile - minProfile);
    const double midpoint = (minProfile + maxProfile) * 0.5;
    for (std::size_t index = 0; index + 1 < keyIndexes.size(); ++index)
    {
        segmentKinds.push_back(SegmentKindFromGeometry(
            points[static_cast<std::size_t>(keyIndexes[index])].point,
            points[static_cast<std::size_t>(keyIndexes[index + 1])].point,
            axis,
            midpoint,
            range));
    }
    return segmentKinds;
}

WeldPointType CornerTypeFromAdjacentSegments(const std::vector<std::string>& segmentKinds, int keyOrdinal)
{
    const std::string previous = keyOrdinal > 0 && keyOrdinal - 1 < static_cast<int>(segmentKinds.size())
        ? segmentKinds[static_cast<std::size_t>(keyOrdinal - 1)]
        : std::string();
    const std::string next = keyOrdinal < static_cast<int>(segmentKinds.size())
        ? segmentKinds[static_cast<std::size_t>(keyOrdinal)]
        : std::string();
    if (previous == "rising_edge"
        || previous == "high_platform"
        || next == "high_platform"
        || next == "falling_edge")
    {
        return WeldPointType::OuterCorner;
    }
    return WeldPointType::InnerCorner;
}

std::string SourceName(TrackPointType type)
{
    switch (type)
    {
    case TrackPointType::Start:
        return "external_start";
    case TrackPointType::End:
        return "external_end";
    case TrackPointType::Corner:
        return "external_corner";
    case TrackPointType::Normal:
    default:
        return "external_2mm";
    }
}
}

FilterFitParams MeasureThenWeldDefaultParams(SampleAxis sampleAxis, GeometryStrategy geometryStrategy)
{
    FilterFitParams params;
    params.sampleAxis = sampleAxis;
    params.geometryStrategy = geometryStrategy;
    params.fitMode = FitMode::PreservePath;
    params.zThreshold = -230.0;
    params.zJumpThreshold = 3.0;
    params.zContinuityThreshold = 2.0;
    params.segmentBreakDistance = 6.0;
    params.keepLongestSegmentOnly = true;
    params.sampleStep = 2.0;
    params.searchWindow = 8.0;
    params.lineFitTrimCount = 0;
    params.piecewiseFitTolerance = 4.0;
    params.piecewiseMinSegmentPoints = 10;
    params.minPointCount = 4;
    params.smoothRadius = 3;
    return params;
}

FilterResult FilterLowerWeldPath(const std::vector<IndexedPoint3D>& inputPoints, const FilterFitParams& params)
{
    FilterResult result;
    result.inputPointCount = static_cast<int>(inputPoints.size());
    if (inputPoints.empty())
    {
        result.error = "input is empty.";
        return result;
    }
    if (params.sampleStep <= 0.0)
    {
        result.error = "sampleStep must be greater than zero.";
        return result;
    }
    if (params.minPointCount <= 0)
    {
        result.error = "minPointCount must be greater than zero.";
        return result;
    }

    std::vector<IndexedPoint3D> lowerPoints;
    lowerPoints.reserve(inputPoints.size());
    for (const IndexedPoint3D& sample : inputPoints)
    {
        if (IsFinite(sample.point) && sample.point.z < params.zThreshold)
        {
            lowerPoints.push_back(sample);
        }
    }
    result.lowerPointCount = static_cast<int>(lowerPoints.size());
    if (static_cast<int>(lowerPoints.size()) < params.minPointCount)
    {
        result.error = "too few points pass zThreshold.";
        return result;
    }

    std::sort(lowerPoints.begin(), lowerPoints.end(),
        [&params](const IndexedPoint3D& left, const IndexedPoint3D& right)
        {
            return AxisValue(left.point, params.sampleAxis) < AxisValue(right.point, params.sampleAxis);
        });

    if (params.zJumpThreshold > 0.0)
    {
        std::vector<IndexedPoint3D> filtered;
        filtered.reserve(lowerPoints.size());
        for (std::size_t index = 0; index < lowerPoints.size(); ++index)
        {
            const double center = AxisValue(lowerPoints[index].point, params.sampleAxis);
            std::vector<double> neighborZs;
            for (std::size_t sample = 0; sample < lowerPoints.size(); ++sample)
            {
                if (sample == index)
                {
                    continue;
                }
                if (std::abs(AxisValue(lowerPoints[sample].point, params.sampleAxis) - center) <= params.searchWindow)
                {
                    neighborZs.push_back(lowerPoints[sample].point.z);
                }
            }
            if (neighborZs.size() >= 2
                && std::abs(lowerPoints[index].point.z - Median(neighborZs)) > params.zJumpThreshold)
            {
                ++result.zJumpRejectedCount;
                continue;
            }
            filtered.push_back(lowerPoints[index]);
        }
        lowerPoints = std::move(filtered);
    }

    if (static_cast<int>(lowerPoints.size()) < params.minPointCount)
    {
        result.error = "too few points remain after z-jump filtering.";
        return result;
    }

    const double minAxis = AxisValue(lowerPoints.front().point, params.sampleAxis);
    const double maxAxis = AxisValue(lowerPoints.back().point, params.sampleAxis);
    const double startAxis = std::round(minAxis / params.sampleStep) * params.sampleStep;
    const double endAxis = std::round(maxAxis / params.sampleStep) * params.sampleStep;

    std::vector<RawFilterPoint> rawPoints;
    for (double sampleAxis = startAxis; sampleAxis <= endAxis + 1e-9; sampleAxis += params.sampleStep)
    {
        std::vector<double> xs;
        std::vector<double> ys;
        std::vector<double> zs;
        for (const IndexedPoint3D& source : lowerPoints)
        {
            if (std::abs(AxisValue(source.point, params.sampleAxis) - sampleAxis) > params.searchWindow)
            {
                continue;
            }
            xs.push_back(source.point.x);
            ys.push_back(source.point.y);
            zs.push_back(source.point.z);
        }
        RawFilterPoint raw;
        raw.axis = sampleAxis;
        if (static_cast<int>(xs.size()) >= params.minPointCount)
        {
            raw.point = SetAxisValue({ Median(xs), Median(ys), Median(zs) }, params.sampleAxis, sampleAxis);
            raw.valid = true;
            raw.source = "measured";
        }
        rawPoints.push_back(raw);
    }

    if (params.zContinuityThreshold > 0.0)
    {
        std::vector<char> reject(rawPoints.size(), 0);
        for (int index = 0; index < static_cast<int>(rawPoints.size()); ++index)
        {
            if (!rawPoints[static_cast<std::size_t>(index)].valid
                || rawPoints[static_cast<std::size_t>(index)].source != "measured")
            {
                continue;
            }
            int previous = index - 1;
            while (previous >= 0 && (!rawPoints[static_cast<std::size_t>(previous)].valid
                || rawPoints[static_cast<std::size_t>(previous)].source != "measured"))
            {
                --previous;
            }
            int next = index + 1;
            while (next < static_cast<int>(rawPoints.size()) && (!rawPoints[static_cast<std::size_t>(next)].valid
                || rawPoints[static_cast<std::size_t>(next)].source != "measured"))
            {
                ++next;
            }
            if (previous < 0 || next >= static_cast<int>(rawPoints.size()))
            {
                continue;
            }
            const double span = rawPoints[static_cast<std::size_t>(next)].axis - rawPoints[static_cast<std::size_t>(previous)].axis;
            if (std::abs(span) <= kEpsilon)
            {
                continue;
            }
            const double ratio = (rawPoints[static_cast<std::size_t>(index)].axis - rawPoints[static_cast<std::size_t>(previous)].axis) / span;
            const double expectedZ = rawPoints[static_cast<std::size_t>(previous)].point.z
                + (rawPoints[static_cast<std::size_t>(next)].point.z - rawPoints[static_cast<std::size_t>(previous)].point.z) * ratio;
            if (std::abs(rawPoints[static_cast<std::size_t>(index)].point.z - expectedZ) > params.zContinuityThreshold)
            {
                reject[static_cast<std::size_t>(index)] = 1;
            }
        }
        for (std::size_t index = 0; index < rawPoints.size(); ++index)
        {
            if (reject[index])
            {
                rawPoints[index].valid = false;
                rawPoints[index].source.clear();
                ++result.zContinuityRejectedCount;
            }
        }
    }

    bool hasValid = false;
    for (const RawFilterPoint& raw : rawPoints)
    {
        hasValid = hasValid || raw.valid;
    }
    if (!hasValid)
    {
        result.error = "no valid path point remains.";
        return result;
    }

    for (int index = 0; index < static_cast<int>(rawPoints.size()); ++index)
    {
        if (rawPoints[static_cast<std::size_t>(index)].valid)
        {
            continue;
        }
        int previous = index - 1;
        while (previous >= 0 && !rawPoints[static_cast<std::size_t>(previous)].valid)
        {
            --previous;
        }
        int next = index + 1;
        while (next < static_cast<int>(rawPoints.size()) && !rawPoints[static_cast<std::size_t>(next)].valid)
        {
            ++next;
        }
        if (previous >= 0 && next < static_cast<int>(rawPoints.size()))
        {
            const double span = rawPoints[static_cast<std::size_t>(next)].axis - rawPoints[static_cast<std::size_t>(previous)].axis;
            const double ratio = std::abs(span) <= kEpsilon ? 0.0 : (rawPoints[static_cast<std::size_t>(index)].axis - rawPoints[static_cast<std::size_t>(previous)].axis) / span;
            rawPoints[static_cast<std::size_t>(index)].point = rawPoints[static_cast<std::size_t>(previous)].point
                + (rawPoints[static_cast<std::size_t>(next)].point - rawPoints[static_cast<std::size_t>(previous)].point) * ratio;
            rawPoints[static_cast<std::size_t>(index)].point = SetAxisValue(rawPoints[static_cast<std::size_t>(index)].point, params.sampleAxis, rawPoints[static_cast<std::size_t>(index)].axis);
            rawPoints[static_cast<std::size_t>(index)].valid = true;
            rawPoints[static_cast<std::size_t>(index)].source = "interpolated";
        }
        else if (previous >= 0)
        {
            rawPoints[static_cast<std::size_t>(index)].point = SetAxisValue(rawPoints[static_cast<std::size_t>(previous)].point, params.sampleAxis, rawPoints[static_cast<std::size_t>(index)].axis);
            rawPoints[static_cast<std::size_t>(index)].valid = true;
            rawPoints[static_cast<std::size_t>(index)].source = "extended";
        }
        else if (next < static_cast<int>(rawPoints.size()))
        {
            rawPoints[static_cast<std::size_t>(index)].point = SetAxisValue(rawPoints[static_cast<std::size_t>(next)].point, params.sampleAxis, rawPoints[static_cast<std::size_t>(index)].axis);
            rawPoints[static_cast<std::size_t>(index)].valid = true;
            rawPoints[static_cast<std::size_t>(index)].source = "extended";
        }
    }

    const int smoothRadius = std::max(0, params.smoothRadius);
    for (int index = 0; index < static_cast<int>(rawPoints.size()); ++index)
    {
        if (!rawPoints[static_cast<std::size_t>(index)].valid)
        {
            continue;
        }
        const int begin = std::max(0, index - smoothRadius);
        const int end = std::min(static_cast<int>(rawPoints.size()) - 1, index + smoothRadius);
        Point3D smoothed;
        int count = 0;
        for (int sample = begin; sample <= end; ++sample)
        {
            if (rawPoints[static_cast<std::size_t>(sample)].valid)
            {
                smoothed = smoothed + rawPoints[static_cast<std::size_t>(sample)].point;
                ++count;
            }
        }
        if (count <= 0)
        {
            continue;
        }
        smoothed = SetAxisValue(smoothed / static_cast<double>(count), params.sampleAxis, rawPoints[static_cast<std::size_t>(index)].axis);
        result.points.push_back({ static_cast<int>(result.points.size()) + 1, smoothed, rawPoints[static_cast<std::size_t>(index)].source });
        if (rawPoints[static_cast<std::size_t>(index)].source == "measured")
        {
            ++result.measuredCount;
        }
        else if (rawPoints[static_cast<std::size_t>(index)].source == "interpolated")
        {
            ++result.interpolatedCount;
        }
        else if (rawPoints[static_cast<std::size_t>(index)].source == "extended")
        {
            ++result.extendedCount;
        }
    }

    if (result.points.empty())
    {
        result.error = "filter produced no output point.";
        return result;
    }

    if (params.keepLongestSegmentOnly && result.points.size() >= 2)
    {
        struct Range { int begin = 0; int end = 0; };
        std::vector<Range> ranges;
        Range current;
        for (int index = 1; index < static_cast<int>(result.points.size()); ++index)
        {
            if (params.segmentBreakDistance > 0.0
                && Norm(result.points[static_cast<std::size_t>(index)].point - result.points[static_cast<std::size_t>(index - 1)].point) > params.segmentBreakDistance)
            {
                ranges.push_back(current);
                current = { index, index };
            }
            else
            {
                current.end = index;
            }
        }
        ranges.push_back(current);
        const Range best = *std::max_element(ranges.begin(), ranges.end(),
            [](const Range& left, const Range& right)
            {
                return left.end - left.begin < right.end - right.begin;
            });
        std::vector<FilterPoint> longest;
        for (int index = best.begin; index <= best.end; ++index)
        {
            FilterPoint point = result.points[static_cast<std::size_t>(index)];
            point.index = static_cast<int>(longest.size()) + 1;
            longest.push_back(point);
        }
        result.segmentRejectedCount = static_cast<int>(result.points.size() - longest.size());
        result.points = std::move(longest);
    }

    if (params.fitMode == FitMode::LineFit)
    {
        const int trimCount = std::max(0, params.lineFitTrimCount);
        const int fitBegin = trimCount;
        const int fitEnd = static_cast<int>(result.points.size()) - trimCount;
        if (fitBegin >= fitEnd || fitEnd - fitBegin < 2)
        {
            result.error = "line fit has too few points after trimming.";
            return result;
        }
        std::vector<double> axes;
        std::vector<double> primaryTargets;
        std::vector<double> secondaryTargets;
        for (int index = fitBegin; index < fitEnd; ++index)
        {
            const Point3D point = result.points[static_cast<std::size_t>(index)].point;
            const double axisValue = AxisValue(point, params.sampleAxis);
            axes.push_back(axisValue);
            primaryTargets.push_back(params.sampleAxis == SampleAxis::AxisY ? point.x : point.y);
            secondaryTargets.push_back(point.z);
        }
        const std::pair<double, double> primaryLine = LinearFit1D(axes, primaryTargets);
        const std::pair<double, double> secondaryLine = LinearFit1D(axes, secondaryTargets);
        for (FilterPoint& point : result.points)
        {
            const double axisValue = AxisValue(point.point, params.sampleAxis);
            if (params.sampleAxis == SampleAxis::AxisY)
            {
                point.point = { primaryLine.first * axisValue + primaryLine.second, axisValue, secondaryLine.first * axisValue + secondaryLine.second };
            }
            else
            {
                point.point = { axisValue, primaryLine.first * axisValue + primaryLine.second, secondaryLine.first * axisValue + secondaryLine.second };
            }
            point.source = "linefit";
        }
        result.fitSegmentCount = 1;
    }
    else if (params.fitMode == FitMode::TrapezoidFit)
    {
        if (!ApplyPiecewiseFit(result, params, true))
        {
            return result;
        }
    }
    else if (params.fitMode == FitMode::PiecewiseLineFit)
    {
        if (!ApplyPiecewiseFit(result, params, false))
        {
            return result;
        }
    }

    result.ok = true;
    return result;
}

// 调试导出：把每段直线拟合“用到的点集”和“拟合出的直线”写成 CloudCompare 友好的 ASCII 点云，
// 与主工程 RobotCalculation::ExportGeometryFitDebugClouds 等价（只是用标准库实现，不依赖 Qt/Eigen）。
// 生成到 outputDir/：fit_all_points.txt(点集，附 dist_to_fit 标量)、fit_all_lines.txt(拟合直线采样)、
// fit_keypoints.txt(起点/终点/拐点)、fit_axes.txt(局部坐标系)、segments/seg_XX_*(每段单独文件)。
void ExportFitDebugClouds(
    const std::vector<ProjectedPoint>& projected,
    const std::vector<int>& keyIndexes,
    const std::vector<Point3D>& fittedKeyPoints,
    const Axes& axes,
    bool useSlopeConsistentCornerFit,
    const std::string& outputDir)
{
    if (outputDir.empty() || keyIndexes.size() < 2 || projected.empty())
    {
        return;
    }

    namespace fs = std::filesystem;
    const fs::path root(outputDir);
    const fs::path segDir = root / "segments";
    std::error_code ec;
    fs::create_directories(segDir, ec);

    const auto fmt = [](double v)
    {
        std::ostringstream os;
        os << std::fixed << std::setprecision(6) << v;
        return os.str();
    };
    const auto pointRow = [&fmt](const Point3D& p, int r, int g, int b, const std::string& extra)
    {
        std::string row = fmt(p.x) + " " + fmt(p.y) + " " + fmt(p.z) + " "
            + std::to_string(r) + " " + std::to_string(g) + " " + std::to_string(b);
        if (!extra.empty())
        {
            row += " ";
            row += extra;
        }
        return row;
    };
    const auto writeCloud = [](const fs::path& path, const std::vector<std::string>& lines)
    {
        std::ofstream out(path);
        if (!out)
        {
            return;
        }
        for (const std::string& line : lines)
        {
            out << line << '\n';
        }
    };

    static const int palette[12][3] = {
        {230, 25, 75}, {60, 180, 75}, {255, 200, 20}, {0, 130, 200},
        {245, 130, 48}, {145, 30, 180}, {70, 200, 240}, {240, 50, 230},
        {170, 220, 40}, {250, 150, 190}, {0, 160, 160}, {200, 170, 255}
    };

    const std::vector<Line2D> segmentLines =
        BuildGeometrySegmentLines(projected, keyIndexes, useSlopeConsistentCornerFit);

    std::vector<std::string> allPointLines{ "// X Y Z R G B segment dist_to_fit smoothN" };
    std::vector<std::string> allLineLines{ "// X Y Z R G B segment" };

    const int segmentCount = static_cast<int>(keyIndexes.size()) - 1;
    for (int seg = 0; seg < segmentCount; ++seg)
    {
        const int begin = std::min(keyIndexes[static_cast<std::size_t>(seg)], keyIndexes[static_cast<std::size_t>(seg + 1)]);
        const int end = std::max(keyIndexes[static_cast<std::size_t>(seg)], keyIndexes[static_cast<std::size_t>(seg + 1)]);
        if (begin < 0 || end >= static_cast<int>(projected.size()) || end < begin)
        {
            continue;
        }
        const int r = palette[seg % 12][0];
        const int g = palette[seg % 12][1];
        const int b = palette[seg % 12][2];

        const Line2D line =
            seg < static_cast<int>(segmentLines.size()) ? segmentLines[static_cast<std::size_t>(seg)] : Line2D();
        const ScalarLine normalLine = FitSegmentNormalLine(projected, begin, end);
        const double fallbackNormal = projected[static_cast<std::size_t>((begin + end) / 2)].smoothN;
        const double dirLen = std::hypot(line.dirS, line.dirH);

        std::vector<std::string> segPointLines{ "// X Y Z R G B dist_to_fit smoothN" };
        std::vector<std::string> segLineLines{ "// X Y Z R G B" };

        // 1) 该段拟合用到的点集，附每点到本段拟合直线的垂距 dist_to_fit。
        for (int i = begin; i <= end; ++i)
        {
            const ProjectedPoint& pp = projected[static_cast<std::size_t>(i)];
            double dist = 0.0;
            if (line.valid && dirLen > kEpsilon)
            {
                const double vs = pp.s - line.sx;
                const double vh = pp.smoothH - line.hy;
                dist = std::abs(Cross2D(vs, vh, line.dirS, line.dirH)) / dirLen;
            }
            allPointLines.push_back(pointRow(pp.point, r, g, b, std::to_string(seg) + " " + fmt(dist) + " " + fmt(pp.smoothN)));
            segPointLines.push_back(pointRow(pp.point, r, g, b, fmt(dist) + " " + fmt(pp.smoothN)));
        }

        // 2) 该段拟合出的直线，沿 s 密集采样后还原回 3D。
        if (line.valid && dirLen > kEpsilon)
        {
            const double sLo = std::min(projected[static_cast<std::size_t>(begin)].s, projected[static_cast<std::size_t>(end)].s);
            const double sHi = std::max(projected[static_cast<std::size_t>(begin)].s, projected[static_cast<std::size_t>(end)].s);
            const double step = 0.5;
            const bool nonVertical = std::abs(line.dirS) > 1e-6;
            for (double s = sLo; s <= sHi + 1e-9; s += step)
            {
                const double h = nonVertical ? (line.hy + (s - line.sx) / line.dirS * line.dirH) : line.hy;
                const double nv = normalLine.valid ? normalLine.ValueAt(s) : fallbackNormal;
                const Point3D p3 = PointFromProjection(axes, s, h, nv);
                allLineLines.push_back(pointRow(p3, r, g, b, std::to_string(seg)));
                segLineLines.push_back(pointRow(p3, r, g, b, std::string()));
            }
        }

        std::ostringstream tag;
        tag << std::setw(2) << std::setfill('0') << seg;
        writeCloud(segDir / ("seg_" + tag.str() + "_points.txt"), segPointLines);
        writeCloud(segDir / ("seg_" + tag.str() + "_line.txt"), segLineLines);
    }

    // 3) 关键点（起点/终点/拐点）
    std::vector<std::string> keyPointLines{ "// X Y Z R G B key_type(1=start 2=end 3=inner_corner 4=outer_corner)" };
    for (std::size_t k = 0; k < fittedKeyPoints.size() && k < keyIndexes.size(); ++k)
    {
        const WeldPointType type = GeometryCornerType(projected, keyIndexes, static_cast<int>(k));
        keyPointLines.push_back(pointRow(fittedKeyPoints[k], 255, 0, 0, std::to_string(WeldPointTypeCode(type))));
    }

    // 4) 本次拟合使用的局部坐标系
    std::vector<std::string> axisLines{ "// role X Y Z" };
    const auto axisRow = [&fmt](const std::string& role, const Point3D& v)
    {
        return role + " " + fmt(v.x) + " " + fmt(v.y) + " " + fmt(v.z);
    };
    axisLines.push_back(axisRow("center", axes.center));
    axisLines.push_back(axisRow("mainAxis_s", axes.main));
    axisLines.push_back(axisRow("sideAxis_h", axes.side));
    axisLines.push_back(axisRow("normalAxis_n", axes.normal));

    writeCloud(root / "fit_all_points.txt", allPointLines);
    writeCloud(root / "fit_all_lines.txt", allLineLines);
    writeCloud(root / "fit_keypoints.txt", keyPointLines);
    writeCloud(root / "fit_axes.txt", axisLines);
}

// ============================================================================
// 波纹板「先测后焊」当前在用的几何拐点管线（与主程序 RobotCalculation 逐函数对齐）。
// 全部工作在 (s, smoothH) 二维平面，2D 线性代数用 Eigen（随附在 third_party/eigen/）；输入/输出均为 keyIndexes
// (指向 projected 的下标向量)，便于在 orchestrator 里串成级联。详见 CHANGELOG / README。
// ============================================================================
namespace
{
template <typename T>
bool VecContains(const std::vector<T>& v, const T& value)
{
    return std::find(v.begin(), v.end(), value) != v.end();
}

// 2D 拟合直线：质心 point + 单位方向 direction（都在 (s, smoothH) 平面内）。valid=false 表示点太少没拟出来。
struct GeometryFittedLine2D
{
    Eigen::Vector2d point = Eigen::Vector2d::Zero();
    Eigen::Vector2d direction = Eigen::Vector2d::UnitX();
    bool valid = false;
};

// 2D 叉积的 z 分量（first × second）：>0 左转、<0 右转，用于求「带符号」转角时配合 atan2。
inline double Cross2D(const Eigen::Vector2d& first, const Eigen::Vector2d& second)
{
    return first.x() * second.y() - first.y() * second.x();
}

// 取一点在拟合用的 (s, smoothH) 二维平面坐标：s=主轴投影(沿弧长向)、smoothH=平滑后的侧向高度。
inline Eigen::Vector2d GeometrySmoothedProjection2D(const ProjectedPoint& point)
{
    return Eigen::Vector2d(point.s, point.smoothH);
}

// 把 [firstIndex, lastIndex] 这一段点在 (s, smoothH) 平面拟合成一条直线（PCA / 主成分方向）。
// 要点：拟合前在两端各裁掉一小段「过渡带」(trim)，因为靠近拐角的点是圆角/噪声转折区，会把直线方向带偏；
// 留下中间的「直边核心」拟合，得到的方向更代表这条边的真实走向。方向最后对齐到「段端点差分」的指向。
// 对应主程序 RobotCalculation::FitGeometrySegmentLine。
GeometryFittedLine2D FitGeometrySegmentLine(
    const std::vector<ProjectedPoint>& projected, int firstIndex, int lastIndex)
{
    GeometryFittedLine2D line;
    if (projected.empty())
    {
        return line;
    }
    // 规整区间端点（允许传入顺序颠倒），并夹进数组范围
    const int begin = std::max(0, std::min(firstIndex, lastIndex));
    const int end = std::min(static_cast<int>(projected.size()) - 1, std::max(firstIndex, lastIndex));
    const int pointCount = end - begin + 1;
    if (pointCount <= 1)
    {
        return line;  // 不足两点，拟不出直线
    }
    // 两端各裁掉 trim 个点的过渡带（段够长才裁，最多裁 8 个），只用中间「直边核心」拟合
    const int trim = pointCount >= 12 ? std::min(pointCount / 6, 8) : 0;
    const int fitBegin = begin + trim;
    const int fitEnd = end - trim;
    const int fitCount = fitEnd - fitBegin + 1;
    if (fitCount < 2)
    {
        // 核心点不足：退化为「直接连两端点」的方向
        const Eigen::Vector2d p0 = GeometrySmoothedProjection2D(projected[begin]);
        const Eigen::Vector2d p1 = GeometrySmoothedProjection2D(projected[end]);
        const Eigen::Vector2d direction = p1 - p0;
        if (direction.squaredNorm() <= std::numeric_limits<double>::epsilon())
        {
            return line;  // 两端重合，无方向
        }
        line.point = (p0 + p1) * 0.5;
        line.direction = direction.normalized();
        line.valid = true;
        return line;
    }
    // 1) 求核心点质心
    Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
    for (int index = fitBegin; index <= fitEnd; ++index)
    {
        centroid += GeometrySmoothedProjection2D(projected[index]);
    }
    centroid /= static_cast<double>(fitCount);
    // 2) 求 2x2 协方差矩阵
    Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
    for (int index = fitBegin; index <= fitEnd; ++index)
    {
        const Eigen::Vector2d delta = GeometrySmoothedProjection2D(projected[index]) - centroid;
        covariance += delta * delta.transpose();
    }
    // 3) 协方差最大特征值对应的特征向量 = 点云主方向 = 直线方向（col(1) 是最大特征值，Eigen 升序排列）
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
    // 4) 特征向量方向有正负二义性 → 翻到与「段首→段尾」一致，保证下游算转角时朝向稳定
    const Eigen::Vector2d segmentDirection =
        GeometrySmoothedProjection2D(projected[end]) - GeometrySmoothedProjection2D(projected[begin]);
    if (line.direction.dot(segmentDirection) < 0.0)
    {
        line.direction = -line.direction;
    }
    line.point = centroid;
    line.valid = true;
    return line;
}

// 点 idx 处的「弯折角」(度)：把 idx 左侧 ±win 段与右侧 ±win 段各拟合成一条直线，取两方向的夹角。
// 直段≈0°，真折角处显著>0。用于端区周期补漏/删错把「位置候选」再用角度二次确认（直段不会被误判成拐点）。
// 对应主程序 RobotCalculation::GeometryBendAngleDegAt。
double GeometryBendAngleDegAt(const std::vector<ProjectedPoint>& projected, int idx, int win)
{
    const int n = static_cast<int>(projected.size());
    if (idx <= 0 || idx >= n - 1)
    {
        return 0.0;  // 端点没有左右两段可比，弯折角无定义
    }
    const int a = std::max(0, idx - win);      // 左段起点（夹在数组范围内）
    const int b = std::min(n - 1, idx + win);  // 右段终点
    const GeometryFittedLine2D before = FitGeometrySegmentLine(projected, a, idx);  // 左段直线方向
    const GeometryFittedLine2D after = FitGeometrySegmentLine(projected, idx, b);   // 右段直线方向
    if (!before.valid || !after.valid)
    {
        return 0.0;  // 点太少、拟合不出方向
    }
    // 用 |方向点积| 求夹角：取绝对值是因为只关心「转了多大角」，不区分左拐/右拐、不区分方向朝向。
    double dot = std::abs(before.direction.dot(after.direction));
    dot = std::min(1.0, std::max(0.0, dot));  // 夹到 [0,1]，防浮点误差让 acos 出 NaN
    constexpr double kPiRad = 3.14159265358979323846;
    return std::acos(dot) * 180.0 / kPiRad;  // 弧度→度
}

// 点 idx 处的「侧向局部斜率」|dh/ds|：在 ±win 邻域对 (s, smoothH) 做最小二乘直线，取斜率绝对值。
// 几何含义：平台段 smoothH 基本不变 → 斜率≈0；上坡/下坡段 smoothH 随 s 明显变化 → 斜率较大。
// 「按平台边界重定拐点」就用它来区分「平的段(平台)」与「斜的段(坡)」。对应主程序 RobotCalculation::LocalLateralSlopeAt。
double LocalLateralSlopeAt(const std::vector<ProjectedPoint>& projected, int idx, int win)
{
    const int n = static_cast<int>(projected.size());
    const int a = std::max(0, idx - win);
    const int b = std::min(n - 1, idx + win);
    const int cnt = b - a + 1;
    if (cnt < 3)
    {
        return 0.0;  // 邻域点太少，斜率不可靠
    }
    // 最小二乘所需的累加量：Σs、Σh、Σs²、Σ(s·h)
    double sx = 0.0, sy = 0.0, sxx = 0.0, sxy = 0.0;
    for (int k = a; k <= b; ++k)
    {
        const double x = projected[k].s;        // 自变量 = 主轴 s
        const double y = projected[k].smoothH;  // 因变量 = 侧向高度 h
        sx += x; sy += y; sxx += x * x; sxy += x * y;
    }
    const double denom = static_cast<double>(cnt) * sxx - sx * sx;  // 最小二乘分母 = n·Σs² - (Σs)²
    if (std::abs(denom) < 1e-9)
    {
        return 0.0;  // s 几乎全相同（竖直），斜率退化
    }
    return std::abs((static_cast<double>(cnt) * sxy - sx * sy) / denom);  // |斜率| = |n·Σsh - Σs·Σh| / denom
}

// 板材搭接「X 错位台阶」检测：两块波纹板拼接处，下层焊道在侧向 h 上有一个小台阶(错边)。
// 思路——对每个相邻点中点 s0：往左、右各空开一段死区 gap，再各取宽 W 的窗口；两窗口分别对侧向 h 拟合直线，
// 同时满足三个条件才认定这里是一个错位台阶：
//   ① 两侧都近似平台（拟合斜率 |k| ≤ kmax）；② 两侧都拟合得很好（残差 RMS ≤ rmax，排掉波纹/噪声段）；
//   ③ 两条线在 s0 处的高度差 step ≥ jump（台阶足够高）。命中则记下「过渡右侧第一点」(i+1) 作为台阶位置。
// 最后按台阶高度降序做 NMS（同一台阶 nmsSep 范围内只保留最高的那个候选）。
// 这些台阶点会被注入关键点并标成搭接角，后续各步都豁免它（不被平滑/合并/重定）。
// 对应主程序 RobotCalculation::DetectLapMisalignmentBoundaries，阈值取自 params.lapStep*。
std::vector<int> DetectLapMisalignmentBoundaries(
    const std::vector<ProjectedPoint>& projected, const FilterFitParams& params)
{
    std::vector<int> result;
    const int n = static_cast<int>(projected.size());
    if (n < 12) return result;

    const double jump = std::max(0.3, params.lapStepHeightThresholdMm);   // 台阶高度阈值：两平台高度差≥此才算台阶
    const double W = std::max(2.0, params.lapStepStationWindowMm);         // 左右各取的拟合窗口宽（主轴 mm）
    const double rmax = std::max(0.02, params.lapStepSideFlatnessMm);      // 两侧拟合残差上限（排掉非平台段）
    const double kmax = std::max(0.02, params.lapStepPlatformSlopeMax);    // 两侧斜率上限（平台应近水平）
    const double gap = 1.5;        // 中点两侧各空开的死区，避开台阶本身的过渡点污染拟合
    const double nmsSep = 4.0;     // NMS 主轴间隔：此距离内只留台阶最高的一个候选
    const int minSidePts = 4;      // 每侧至少点数，否则拟合不可靠

    // 一侧点集做 h = k·s + b 的最小二乘，并回报拟合残差 RMS（rms 越小越「平整」）
    auto fitLine = [&](const std::vector<int>& idxs, double& k, double& b, double& rms) -> bool {
        const int m = static_cast<int>(idxs.size());
        if (m < 2) return false;
        double Ss = 0.0, Sh = 0.0, Sss = 0.0, Ssh = 0.0;
        for (int j : idxs) { const double s = projected[j].s, h = projected[j].h; Ss += s; Sh += h; Sss += s * s; Ssh += s * h; }
        const double den = m * Sss - Ss * Ss;
        if (std::abs(den) < 1e-9) { k = 0.0; b = Sh / m; }  // s 退化 → 取水平线
        else { k = (m * Ssh - Ss * Sh) / den; b = (Sh - k * Ss) / m; }
        double e2 = 0.0;
        for (int j : idxs) { const double d = projected[j].h - (k * projected[j].s + b); e2 += d * d; }
        rms = std::sqrt(e2 / m);
        return true;
    };

    struct Cand { int idx; double s0; double step; };  // 候选台阶：注入点、台阶主轴位置、台阶高
    std::vector<Cand> cands;
    for (int i = 0; i + 1 < n; ++i)  // 遍历每个相邻点中点作为候选台阶位置 s0
    {
        const double s0 = 0.5 * (projected[i].s + projected[i + 1].s);
        std::vector<int> L, R;
        for (int j = 0; j < n; ++j)  // 收集 s0 左/右 [gap, gap+W] 窗口内的点
        {
            const double d = projected[j].s - s0;
            if (d <= -gap && d >= -(gap + W)) L.push_back(j);
            else if (d >= gap && d <= (gap + W)) R.push_back(j);
        }
        if (static_cast<int>(L.size()) < minSidePts || static_cast<int>(R.size()) < minSidePts) continue;
        double kL, bL, rL, kR, bR, rR;
        if (!fitLine(L, kL, bL, rL) || !fitLine(R, kR, bR, rR)) continue;
        if (std::abs(kL) > kmax || std::abs(kR) > kmax) continue;  // 条件①：两侧都得是平台
        if (rL > rmax || rR > rmax) continue;                      // 条件②：两侧都拟合得很平整
        const double step = std::abs((kR * s0 + bR) - (kL * s0 + bL));  // 两平台在 s0 处的高度差 = 台阶高
        if (step < jump) continue;                                 // 条件③：台阶够高
        cands.push_back({ i + 1, s0, step });  // 命中：注入「过渡右侧第一点」
    }
    if (cands.empty()) return result;

    // 按台阶高降序，NMS：同一台阶附近(<nmsSep)只保留最高的那个候选，避免一处台阶报多次
    std::sort(cands.begin(), cands.end(), [](const Cand& a, const Cand& b) { return a.step > b.step; });
    std::vector<Cand> kept;
    for (const Cand& c : cands)
    {
        bool suppressed = false;
        for (const Cand& k : kept)
            if (std::abs(k.s0 - c.s0) < nmsSep) { suppressed = true; break; }
        if (!suppressed) kept.push_back(c);
    }
    for (const Cand& c : kept) result.push_back(c.idx);
    std::sort(result.begin(), result.end());  // 返回前按主轴顺序排好
    return result;
}

// 方位角拐点检测——SdkBaseWeldFit（inputAlreadyDenoised=true）路径的拐点主检测器。
// 原理：在每个内部点 i 处，分别对左侧 [i-k, i]、右侧 [i, i+k] 做直线拟合得到两个方向，量出两方向的
// 「带符号转角」(atan2)。直段两方向几乎一致 → 转角≈0；真折角处转角显著。再用阈值筛 + 按 |转角| 降序的
// NMS（同一处折角只留转得最厉害的那个点）得到拐点；最后拼上起点/终点、并丢掉挨太近(<minSegMm)的点。
// 比旧的 Douglas-Peucker「离弦最远点」更能区分「真折角」与「波纹平台上的轻微起伏」，故去噪输入用它。
// 对应主程序 RobotCalculation::BuildAzimuthCornerKeyIndexes。参数取自 params.azimuth*。
std::vector<int> BuildAzimuthCornerKeyIndexes(
    const std::vector<ProjectedPoint>& projected, const FilterFitParams& params)
{
    std::vector<int> keys;
    const int n = static_cast<int>(projected.size());
    if (n < 2)
    {
        if (n == 1) keys.push_back(0);
        return keys;
    }

    const int k = std::max(2, params.azimuthHeadingWindow);  // 左右各拟合方向用的窗口点数
    constexpr double kPi = 3.14159265358979323846;
    const double turnThreshold = std::max(1.0, params.azimuthTurnThresholdDeg) * kPi / 180.0;  // 判拐点的转角阈值(弧度)
    const double nmsSpan = std::max(1.0, params.azimuthNmsSpanMm);  // NMS 主轴间隔(同一折角抑制范围)
    const int guard = k;  // 头尾各留 k 点，保证左右窗口都取得到

    // 1) 逐点求带符号转角：左右两段方向各自先翻到与「段端点差分」一致(消除 PCA 方向二义性)，再算夹角
    std::vector<double> signedTurn(n, 0.0);
    for (int i = guard; i < n - guard; ++i)
    {
        Eigen::Vector2d leftDir = FitGeometrySegmentLine(projected, i - k, i).direction;
        Eigen::Vector2d rightDir = FitGeometrySegmentLine(projected, i, i + k).direction;
        const Eigen::Vector2d leftRef =
            GeometrySmoothedProjection2D(projected[i]) - GeometrySmoothedProjection2D(projected[i - k]);
        const Eigen::Vector2d rightRef =
            GeometrySmoothedProjection2D(projected[i + k]) - GeometrySmoothedProjection2D(projected[i]);
        if (leftDir.squaredNorm() <= 1e-12 || rightDir.squaredNorm() <= 1e-12) continue;
        if (leftDir.dot(leftRef) < 0.0) leftDir = -leftDir;     // 翻向，保证沿前进方向
        if (rightDir.dot(rightRef) < 0.0) rightDir = -rightDir;
        leftDir.normalize();
        rightDir.normalize();
        // atan2(叉积, 点积) = 左方向转到右方向的带符号角；这里只用其绝对值大小判拐点
        signedTurn[i] = std::atan2(Cross2D(leftDir, rightDir), leftDir.dot(rightDir));
    }

    // 2) 转角超阈的点入候选，再按 |转角| 从大到小做 NMS：主轴近邻(<nmsSpan)内只保留转得最厉害的一个
    std::vector<int> candidates;
    for (int i = guard; i < n - guard; ++i)
    {
        if (std::abs(signedTurn[i]) >= turnThreshold) candidates.push_back(i);
    }
    std::sort(candidates.begin(), candidates.end(),
        [&](int a, int b) { return std::abs(signedTurn[a]) > std::abs(signedTurn[b]); });
    std::vector<int> corners;
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

    // 3) 组装关键点：起点 + 各拐点(丢掉端点、丢掉与上一关键点主轴距离<minSegMm 的) + 终点
    const double minSegMm = std::max(params.sampleStep * 2.0, 2.0);
    keys.push_back(0);
    for (int idx : corners)
    {
        if (idx <= 0 || idx >= n - 1) continue;
        if (std::abs(projected[idx].s - projected[keys.back()].s) < minSegMm) continue;
        keys.push_back(idx);
    }
    if (keys.back() != n - 1) keys.push_back(n - 1);
    return keys;
}

// 直线化兜底：方位角检测后，若某相邻关键点段内实际点偏离「两端连线(弦)」超过阈值，说明这段里还藏着漏检的
// 拐点 → 把该段离弦最远的点补进来。反复迭代(最多 8 轮)直到没有新插入。对应 RobotCalculation::SplitNonStraightAzimuthSegments。
std::vector<int> SplitNonStraightAzimuthSegments(
    const std::vector<ProjectedPoint>& projected, const std::vector<int>& keyIndexes, const FilterFitParams& params)
{
    std::vector<int> keys = keyIndexes;
    if (keys.size() < 2) return keys;
    const double residualThreshold = std::max(0.5, params.azimuthStraightenResidualMm);  // 离弦阈值
    for (int iter = 0; iter < 8; ++iter)
    {
        std::vector<int> next;
        next.push_back(keys.front());
        bool inserted = false;
        for (int kp = 0; kp + 1 < static_cast<int>(keys.size()); ++kp)
        {
            const int a = keys[kp];
            const int b = keys[kp + 1];
            // 找本段 (a,b) 内离弦最远的点
            double maxDist = -1.0;
            int maxIdx = -1;
            for (int i = a + 1; i < b; ++i)
            {
                const double d = GeometryDistanceToSegment2D(projected, a, b, i);
                if (d > maxDist) { maxDist = d; maxIdx = i; }
            }
            if (maxIdx > a && maxIdx < b && maxDist > residualThreshold)
            {
                next.push_back(maxIdx);  // 离弦超阈 → 该段不直，补这个拐点
                inserted = true;
            }
            next.push_back(b);
        }
        keys = next;
        if (!inserted) break;  // 本轮没补新点 → 收敛
    }
    return keys;
}

// 「相干弓」自适应补拐点：对每段(相邻关键点之间)找离弦最远点，但只有同时满足两个条件才补，避免误补：
//   ① 一致弓向：段内的点大多数偏在弦的【同一侧】(占比 ≥ kOneSidedMin)。随机噪声会两侧乱偏、占比≈50%，
//      只有真折角才让整段相干地拱向一边——以此挡掉噪声。
//   ② 峰值够高：离弦峰值 > 与【位置相关】的地板。端区(起/终各 kEndFrac 比例内)用低地板 floorMm，更敏感
//      (端区是检测盲区、易漏)；中段用更高的地板 floorMm×kMidMultiple，更保守(中段一般检得准)。
// 再加 farEnough：补出的点距两端都 ≥ minSegMm，避免产生过短段。反复迭代(最多 8 轮)直到无新增。
// 开关关或 azimuthRefineFloorMm≤0 时整步不动。对应 RobotCalculation::RefineCornersByCoherentBow，参数取自 cornerRefine*。
std::vector<int> RefineCornersByCoherentBow(
    const std::vector<ProjectedPoint>& projected, const std::vector<int>& keyIndexes, const FilterFitParams& params)
{
    std::vector<int> keys = keyIndexes;
    const double floorMm = params.azimuthRefineFloorMm;  // 端区离弦地板(基准)
    if (!params.cornerRefineEnable || floorMm <= 0.0 || keys.size() < 2 || projected.size() < 3)
    {
        return keys;  // 开关关 / 无地板 / 数据太少 → 不动
    }
    const double kOneSidedMin = std::min(1.0, std::max(0.5, params.cornerRefineOneSidedFrac));  // 一致弓向占比门
    const double kMidMultiple = std::max(1.0, params.cornerRefineMidMultiple);  // 中段地板 = 端区地板 × 此
    const double kEndFrac = std::min(0.5, std::max(0.0, params.cornerRefineEndFrac));  // 端区占整长比例
    const double minSegMm = std::max(params.sampleStep * 2.0, 2.0);  // 最短段(避免补出过短段)
    const double s0 = projected.front().s;
    const double sN = projected.back().s;
    const double span = std::abs(sN - s0);  // 全长(用于判断某点落在端区还是中段)

    for (int iter = 0; iter < 8; ++iter)
    {
        std::vector<int> next;
        next.push_back(keys.front());
        bool inserted = false;
        for (int kp = 0; kp + 1 < static_cast<int>(keys.size()); ++kp)
        {
            const int a = keys[kp];
            const int b = keys[kp + 1];
            const Eigen::Vector2d pa = GeometrySmoothedProjection2D(projected[a]);
            const Eigen::Vector2d pb = GeometrySmoothedProjection2D(projected[b]);
            const Eigen::Vector2d chord = pb - pa;  // 段两端连成的弦
            const double chordLen = chord.norm();
            // 扫描本段：记离弦最远点 + 统计有多少点偏在弦的「正侧」(叉积符号)
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
                // 一致弓向占比 = max(正侧, 负侧) / 总数；越接近 1 越「相干」(都拱一边)
                const double oneSided =
                    std::max(positiveSide, totalSide - positiveSide) / static_cast<double>(totalSide);
                // 该点在全长中的相对位置 → 落端区还是中段 → 选对应地板
                const double frac = span > 1e-9 ? std::abs(projected[maxIdx].s - s0) / span : 0.5;
                const bool endZone = frac < kEndFrac || frac > 1.0 - kEndFrac;
                const double floorHere = endZone ? floorMm : floorMm * kMidMultiple;
                const bool farEnough =
                    std::abs(projected[maxIdx].s - projected[a].s) >= minSegMm
                    && std::abs(projected[b].s - projected[maxIdx].s) >= minSegMm;
                // 三条件齐备(峰够高 + 弓向相干 + 不太近)才补这个拐点
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
            break;  // 收敛
        }
    }
    return keys;
}

// 在稠密路径的一段区域 [regLoS, regHiS] 内做「坡-平台-坡」三段直线拟合，返回两个折点(平台两端角)的 projected 索引。
// 做法：用前缀和让任意子段的「直线拟合残差」O(1) 可取，再暴力枚举两个断点 b1<b2，使三段残差之和最小。
// b1、b2 即「坡↔平台」「平台↔坡」两处交界 = 这个平台的两个边界角。点数不足 3×每段最少点时放弃(返回 -1,-1)。
std::pair<int, int> FitPlatformTwoCorners(
    const std::vector<ProjectedPoint>& projected, double regLoS, double regHiS, int minSegPoints)
{
    // 1) 收集区域内的点并按主轴 s 排好
    const double lo = std::min(regLoS, regHiS);
    const double hi = std::max(regLoS, regHiS);
    std::vector<int> reg;
    for (int i = 0; i < static_cast<int>(projected.size()); ++i)
    {
        if (projected[i].s >= lo && projected[i].s <= hi) reg.push_back(i);
    }
    std::sort(reg.begin(), reg.end(), [&](int a, int b) { return projected[a].s < projected[b].s; });
    const int m = static_cast<int>(reg.size());
    const int mp = std::max(3, minSegPoints);   // 每段最少点数
    if (m < 3 * mp) return std::make_pair(-1, -1);  // 三段都凑不够 → 放弃

    // 2) (s, smoothH) 各阶前缀和：使任意子段 [i,j] 的直线拟合残差能 O(1) 算出
    std::vector<double> Sx(m + 1, 0.0), Sy(m + 1, 0.0), Sxx(m + 1, 0.0), Sxy(m + 1, 0.0), Syy(m + 1, 0.0);
    for (int k = 0; k < m; ++k)
    {
        const double x = projected[reg[k]].s;
        const double y = projected[reg[k]].smoothH;
        Sx[k + 1] = Sx[k] + x;       Sy[k + 1] = Sy[k] + y;
        Sxx[k + 1] = Sxx[k] + x * x; Sxy[k + 1] = Sxy[k] + x * y; Syy[k + 1] = Syy[k] + y * y;
    }
    // 子段 [i,j] 拟合成直线后的残差平方和：= Syy_c - Sxy_c² / Sxx_c（中心化协方差；Sxx_c≈0 即竖直，退化为 Syy_c）
    auto res = [&](int i, int j) -> double {
        const int c = j - i + 1;
        if (c < 2) return 0.0;
        const double sx = Sx[j + 1] - Sx[i], sy = Sy[j + 1] - Sy[i];
        const double sxx = Sxx[j + 1] - Sxx[i], sxy = Sxy[j + 1] - Sxy[i], syy = Syy[j + 1] - Syy[i];
        const double sxxc = sxx - sx * sx / c, sxyc = sxy - sx * sy / c, syyc = syy - sy * sy / c;
        if (sxxc <= 1e-12) return syyc;
        return std::max(0.0, syyc - sxyc * sxyc / sxxc);
    };
    // 3) 暴力枚举两个断点 b1<b2(各段都 ≥mp 点)，取三段残差之和最小的一组 = 最佳「坡|平台|坡」切分
    double best = std::numeric_limits<double>::max();
    int bb1 = -1, bb2 = -1;
    for (int b1 = mp; b1 < m - 2 * mp; ++b1)
    {
        const double r1 = res(0, b1);              // 第一段(坡)残差，固定 b1 时不变
        for (int b2 = b1 + mp; b2 < m - mp; ++b2)
        {
            const double tot = r1 + res(b1, b2) + res(b2, m - 1);  // 坡 + 平台 + 坡
            if (tot < best) { best = tot; bb1 = b1; bb2 = b2; }
        }
    }
    if (bb1 < 0 || bb2 < 0) return std::make_pair(-1, -1);
    return std::make_pair(reg[bb1], reg[bb2]);  // 两断点的 projected 索引 = 平台两端角
}

// 「平台重算」——按波纹的 inner/outer 结构约束，把每个平台的拐点重算成恰好 2 个边界角。
// 背景：波纹拐点呈 …II OO II OO…（高平台两端是两个同类内角 II，低平台两端是两个同类外角 OO）。
// 步骤：① 给每个(非搭接)内部拐点按 smoothH 凸起量定 inner(+1)/outer(-1)；② 把同类相邻的拐点并成「游程」，
//   一个游程 = 一个平台；③ 对每个平台，取「上一游程末角 ~ 下一游程首角」这段稠密路径(含平台+两侧坡)，用
//   FitPlatformTwoCorners 重算出恰好 2 个边界角，替换原游程里的角(多检 3/5→2、漏检 1→2 一并修正)；拟合失败
//   则保留原角兜底。④ 起点/终点/搭接角原样保留，最后按 s 归并去重。
// 开关关、点太少或拐点不足 2 时整步不动。对应 RobotCalculation::RefitCornersByPlatformPattern。
std::vector<int> RefitCornersByPlatformPattern(
    const std::vector<ProjectedPoint>& projected, const std::vector<int>& keyIndexes,
    const std::vector<char>& isLapStepKey, const FilterFitParams& params)
{
    const int m = static_cast<int>(keyIndexes.size());
    if (!params.cornerPatternRefitEnable || m < 4 || projected.size() < 24)
    {
        return keyIndexes;
    }
    const double startS = projected[keyIndexes.front()].s;
    const double endS = projected[keyIndexes.back()].s;

    // ① 自然拐点 + 类型：prominence = 本点 smoothH 减去左右邻点均值；≥0 凸起=inner，<0 凹陷=outer。搭接角跳过。
    struct NatCorner { int keyPos; int type; double s; };
    std::vector<NatCorner> nat;
    for (int k = 1; k < m - 1; ++k)
    {
        if (k < static_cast<int>(isLapStepKey.size()) && isLapStepKey[k]) continue;
        const double prom = projected[keyIndexes[k]].smoothH
            - (projected[keyIndexes[k - 1]].smoothH + projected[keyIndexes[k + 1]].smoothH) * 0.5;
        nat.push_back({ k, prom >= 0.0 ? 1 : -1, projected[keyIndexes[k]].s });
    }
    if (nat.size() < 2)
    {
        return keyIndexes;
    }

    // ② 同类相邻拐点并成游程 [i,j]，每个游程对应一个平台
    std::vector<std::pair<int, int>> runs;
    for (int i = 0; i < static_cast<int>(nat.size()); )
    {
        int j = i;
        while (j + 1 < static_cast<int>(nat.size()) && nat[j + 1].type == nat[i].type) ++j;
        runs.push_back(std::make_pair(i, j));
        i = j + 1;
    }

    // ③ 逐平台重算 2 个边界角；区域取「上一游程末角 ~ 下一游程首角」(含本平台 + 两侧坡)，端部用起/终点兜底
    std::vector<int> refit;
    for (int r = 0; r < static_cast<int>(runs.size()); ++r)
    {
        const int i = runs[r].first, j = runs[r].second;
        const double regLoS = (r > 0) ? nat[runs[r - 1].second].s : startS;
        const double regHiS = (r + 1 < static_cast<int>(runs.size())) ? nat[runs[r + 1].first].s : endS;
        const std::pair<int, int> two = FitPlatformTwoCorners(
            projected, regLoS, regHiS, std::max(3, params.cornerPlatformMinSegPoints));
        if (two.first >= 0 && two.second >= 0 && two.first != two.second)
        {
            refit.push_back(two.first);
            refit.push_back(two.second);
        }
        else
        {
            for (int t = i; t <= j; ++t) refit.push_back(keyIndexes[nat[t].keyPos]);  // 拟合失败 → 保留原角
        }
    }

    // ④ 起点 + 重算角 + 搭接角 + 终点 → 按 s 排序去重
    std::vector<int> result;
    result.push_back(keyIndexes.front());
    result.insert(result.end(), refit.begin(), refit.end());
    for (int k = 1; k < m - 1; ++k)
    {
        if (k < static_cast<int>(isLapStepKey.size()) && isLapStepKey[k]) result.push_back(keyIndexes[k]);
    }
    result.push_back(keyIndexes.back());
    const bool ascending = endS >= startS;
    std::sort(result.begin(), result.end(), [&](int a, int b) {
        return ascending ? projected[a].s < projected[b].s : projected[a].s > projected[b].s;
    });
    result.erase(std::unique(result.begin(), result.end()), result.end());
    return result.size() >= 2 ? result : keyIndexes;  // 退化兜底
}

// 「端区周期补漏」——专治起点段/终点段(检测盲区)漏掉的拐点。
// 核心观察：波纹拐点近似【等周期】。先从中段可靠拐点算出「中位段长 L = 一个周期」；若某个端段(第一段或
// 最后一段)长度 ≥ ratioThreshold·L，说明这段里塞了不止一个周期、藏着漏检的拐点。于是从端段【内侧】那个
// 拐点出发，按周期 L 往端点方向逐个反推「漏点应在的 s 位置」，再在该位置 ±窗口内找【弯折角】峰值来【确认】
// (角度 ≥ max(minBendDeg, 0.5×典型拐角角)；直段角≈0 不会误补)。位置反推 + 角度确认，双判据。
// 只【插入】非搭接的新拐点、不动现有拐点；搭接台阶两角先合成一个代表点再算周期。对应 RobotCalculation::RecoverEndRegionCornersByPeriod。
std::vector<int> RecoverEndRegionCornersByPeriod(
    const std::vector<ProjectedPoint>& projected, const std::vector<int>& keyIndexes,
    const std::vector<char>& isLapStepKey, double ratioThreshold, double minBendDeg, int* insertedCount)
{
    if (insertedCount != nullptr)
    {
        *insertedCount = 0;
    }
    const int m = static_cast<int>(keyIndexes.size());
    if (m < 5 || projected.size() < 16)
    {
        return keyIndexes;
    }

    // 1) 代表点序列 repS：把关键点压成各自的主轴 s；搭接台阶的两个角合成一个中点(不让台阶撑大周期估计)
    std::vector<double> repS;
    repS.reserve(m);
    for (int k = 0; k < m; )
    {
        if (k + 1 < m && k + 1 < static_cast<int>(isLapStepKey.size()) && isLapStepKey[k] && isLapStepKey[k + 1])
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
        return keyIndexes;
    }

    // 2) 周期 L = 【中段】相邻代表点段长的中位数（去掉首尾段，端段可能本就是漏点的长段、不能参与估周期）
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

    // 3) 典型拐角角 = 中段各拐点弯折角的中位数；确认阈值 acceptBend = max(minBendDeg, 0.5×典型角)
    const int bendWin = 10;
    std::vector<double> interiorBend;
    for (int k = 1; k < m - 1; ++k)
    {
        if (k < static_cast<int>(isLapStepKey.size()) && isLapStepKey[k])
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

    // 在 sCenter ±halfWin 内找弯折角最大且 ≥acceptBend 的点；找不到返回 -1（角度确认）
    auto findCornerInWindow = [&](double sCenter, double halfWin) -> int
    {
        int bestIdx = -1;
        double bestBend = acceptBend;
        for (int i = bendWin; i + bendWin < static_cast<int>(projected.size()); ++i)
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

    std::vector<int> result = keyIndexes;
    int inserted = 0;

    // 处理一个端段：sNear=端点侧代表点、sFar=内侧代表点。仅当段长 ≥ ratioThreshold·L 才认为漏了拐点；
    // 然后从内侧 sFar 按周期 L 向端点逐个反推预测位置 sPred，在 ±0.35L 窗口内用弯折角确认、且不与已有点重叠/太近时补入。
    auto tryRecoverEndSegment = [&](double sNear, double sFar)
    {
        const double segLen = std::abs(sFar - sNear);
        if (segLen < ratioThreshold * L)
        {
            return;  // 端段不够长 → 没漏点
        }
        const double dir = (sFar >= sNear) ? 1.0 : -1.0;        // 从 sFar 往端点的方向
        const double sLo = std::min(sNear, sFar) + 0.15 * L;    // 预测位置允许范围(两端各留 0.15L 余量)
        const double sHi = std::max(sNear, sFar) - 0.15 * L;
        const int maxMissing = static_cast<int>(segLen / L) + 1; // 这段最多可能漏几个
        for (int j = 1; j <= maxMissing; ++j)
        {
            const double sPred = sFar - dir * j * L;  // 第 j 个漏点的预测主轴位置
            if (sPred < sLo || sPred > sHi)
            {
                continue;
            }
            const int idx = findCornerInWindow(sPred, 0.35 * L);  // 角度确认
            if (idx < 0 || VecContains(result, idx))
            {
                continue;  // 该处没有真折角，或已存在
            }
            bool tooClose = false;
            for (int existing : result)
            {
                if (std::abs(projected[existing].s - projected[idx].s) < 0.15 * L)
                {
                    tooClose = true;  // 离已有拐点太近，不补(避免双拐点)
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

    tryRecoverEndSegment(repS[0], repS[1]);            // 起点段：端点=repS[0]、内侧=repS[1]
    tryRecoverEndSegment(repS[mm - 1], repS[mm - 2]);  // 终点段：端点=repS[末]、内侧=repS[末-1]

    if (inserted > 0)
    {
        const bool ascending = projected[keyIndexes.back()].s >= projected[keyIndexes.front()].s;
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

// 周期/角度删错拐点：相邻【同类】拐点弧长间距 ≪ 周期L 说明一个找错。但两拐点【之间】若是平的(真平台)
// 绝不删；只有之间是【坡】(假双拐点)才删，且删两侧都陡的"坡中"那个、保留贴平台的边界角。搭接对豁免。
std::vector<int> MergeTooCloseSameTypeCorners(
    const std::vector<ProjectedPoint>& projected, const std::vector<int>& keyIndexes,
    const std::vector<char>& isLapStepKey, double distFrac, int* removedCount)
{
    if (removedCount != nullptr) { *removedCount = 0; }
    const int m0 = static_cast<int>(keyIndexes.size());
    if (m0 < 5) { return keyIndexes; }

    std::vector<double> repS;
    for (int k = 0; k < m0; )
    {
        if (k + 1 < m0 && k + 1 < static_cast<int>(isLapStepKey.size()) && isLapStepKey[k] && isLapStepKey[k + 1])
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
        if (k < static_cast<int>(isLapStepKey.size()) && isLapStepKey[k]) { continue; }
        const double b = GeometryBendAngleDegAt(projected, keyIndexes[k], bendWin);
        if (b > 0.0) { ib.push_back(b); }
    }
    double typical = 8.0;
    if (!ib.empty()) { std::sort(ib.begin(), ib.end()); typical = ib[ib.size() / 2]; }

    std::vector<int> result = keyIndexes;
    std::vector<char> lapKey = isLapStepKey;
    const double flatSlopeThresh = 0.15;  // 段间侧向斜率 < 此判「平」(平台)，≥此判「斜」(坡)
    // 两关键点之间这一段的侧向斜率 |Δh/Δs|
    auto segSlope = [&](int posA, int posB) -> double
    {
        const double ds = projected[result[posB]].s - projected[result[posA]].s;
        if (std::abs(ds) < 1e-6) { return 0.0; }
        return std::abs((projected[result[posB]].smoothH - projected[result[posA]].smoothH) / ds);
    };
    int removed = 0;
    bool changed = true;
    while (changed && static_cast<int>(result.size()) > 4)  // 每删一个就重扫，直到没得删
    {
        changed = false;
        for (int k = 1; k + 1 <= static_cast<int>(result.size()) - 2; ++k)  // 看相邻两个中段拐点 (k, k+1)
        {
            // 搭接对豁免、必须同类、间距必须 < minGap(≈周期的 distFrac)，否则不是「找错的双拐点」
            if ((k < static_cast<int>(lapKey.size()) && lapKey[k]) || (k + 1 < static_cast<int>(lapKey.size()) && lapKey[k + 1])) { continue; }
            if (GeometryCornerType(projected, result, k) != GeometryCornerType(projected, result, k + 1)) { continue; }
            const double gap = std::abs(projected[result[k]].s - projected[result[k + 1]].s);
            if (gap >= minGap) { continue; }
            // 【平台保护】两拐点【之间】若是平的(真平台，哪怕很短)→绝不删；只有之间是【坡】(假双拐点)才删
            if (segSlope(k, k + 1) < flatSlopeThresh) { continue; }
            // 删「坡中」那个：哪个拐点的另一侧也是坡(两侧都陡=卡在坡中间=假角)就删哪个，保留贴着平台的边界角
            const bool kMidEdge = (k - 1 >= 0) && segSlope(k - 1, k) >= flatSlopeThresh;
            const bool k1MidEdge = (k + 2 < static_cast<int>(result.size())) && segSlope(k + 1, k + 2) >= flatSlopeThresh;
            int removeAt;
            if (kMidEdge && !k1MidEdge) { removeAt = k; }
            else if (k1MidEdge && !kMidEdge) { removeAt = k + 1; }
            else
            {
                // 两者都像/都不像坡中 → 退而求其次：删弯折角离「典型角」更远的那个(更可疑)
                const double bk = GeometryBendAngleDegAt(projected, result[k], bendWin);
                const double bk1 = GeometryBendAngleDegAt(projected, result[k + 1], bendWin);
                removeAt = (std::abs(bk - typical) >= std::abs(bk1 - typical)) ? k : (k + 1);
            }
            result.erase(result.begin() + removeAt);
            lapKey.erase(lapKey.begin() + removeAt);
            ++removed;
            changed = true;
            break;  // 索引已变，跳出重扫
        }
    }
    if (removedCount != nullptr) { *removedCount = removed; }
    return result;
}

// 「按平台边界重定拐点」——最根本的结构性纠正，治【拐点被放到平台正中间→平台被拉直成长坡而消失】。
// 关键事实：波纹的拐点【全部位于「平台↔坡」交界】。本步沿焊道把「平的段(平台)」检测出来，然后对每个平台：
//   保证它两端各有一个边界拐点（缺则补在平台端点）、并删掉卡在平台【内部】(放错位)的拐点。
// 对【已经正确】的平台幂等不动(两端本就有角、内部本就无角)，只纠正错的——所以能统一治「缺/重/放错位」三种情况。
// 搭接角、起点、终点都算合法边界(参与 hasA/hasB 判定)但不会被删。对应 RobotCalculation::SnapCornersToPlatforms。
std::vector<int> SnapCornersToPlatforms(
    const std::vector<ProjectedPoint>& projected, const std::vector<int>& keyIndexes,
    const std::vector<char>& isLapStepKey, double flatSlopeThresh, double minPlatformFrac, int* changedCount)
{
    if (changedCount != nullptr) { *changedCount = 0; }
    const int m0 = static_cast<int>(keyIndexes.size());
    const int n = static_cast<int>(projected.size());
    if (m0 < 4 || n < 32) { return keyIndexes; }

    // 周期 L = 中段代表点段长中位数（搭接对合成中点），与端区周期补漏同口径
    std::vector<double> repS;
    for (int k = 0; k < m0; )
    {
        if (k + 1 < m0 && k + 1 < static_cast<int>(isLapStepKey.size()) && isLapStepKey[k] && isLapStepKey[k + 1])
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

    // 斜率检测窗 win ≈ 7mm（按点间距换算）；逐点判平/斜
    const double span = std::abs(projected[n - 1].s - projected[0].s);
    const double spacing = span > 1e-6 ? span / (n - 1) : 1.0;
    const int win = std::min(40, std::max(6, static_cast<int>(std::lround(7.0 / std::max(0.05, spacing)))));

    std::vector<char> flat(n, 0);
    for (int i = 0; i < n; ++i)
    {
        flat[i] = LocalLateralSlopeAt(projected, i, win) < flatSlopeThresh ? 1 : 0;  // 局部斜率小=平台点
    }
    // 把连续的平台点并成「平台段 [i,j]」，只保留长度 ≥ minPlatLen 的(短的当噪声忽略)
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

    // 逐平台判定（对原 keyIndexes 一次性收集删/插，避免边改边漂移）。marc=容差：现有角在平台端 ±marc 内即算「该端已有边界角」
    const double marc = std::max(10.0, 0.18 * L);
    std::vector<char> removeFlag(m0, 0);
    std::vector<int> insertIdx;
    for (const auto& pab : plats)
    {
        const double sa = projected[pab.first].s;   // 平台左端 s
        const double sb = projected[pab.second].s;  // 平台右端 s
        const double lo = std::min(sa, sb), hi = std::max(sa, sb);
        bool hasA = false, hasB = false;
        std::vector<int> interior;
        // hasA/hasB：两端是否【已有】角(含起终点/搭接角，都是合法边界——不计入会误判缺边界而补出重复角)
        for (int k = 0; k < m0; ++k)
        {
            const double sc = projected[keyIndexes[k]].s;
            if (std::abs(sc - sa) <= marc) { hasA = true; }
            if (std::abs(sc - sb) <= marc) { hasB = true; }
        }
        // interior：卡在平台【深内部】(离两端都 >marc)的常规角(非起终点/搭接) = 放错位，待删
        for (int k = 1; k < m0 - 1; ++k)
        {
            if (k < static_cast<int>(isLapStepKey.size()) && isLapStepKey[k]) { continue; }
            const double sc = projected[keyIndexes[k]].s;
            if (sc > lo + marc && sc < hi - marc) { interior.push_back(k); }
        }
        if (interior.empty() && hasA && hasB) { continue; }  // 该平台已正确：两端有角、内部无角 → 不动(幂等)
        for (int k : interior) { removeFlag[k] = 1; }        // 删内部放错的角
        if (!hasA) { insertIdx.push_back(pab.first); }       // 缺左边界角 → 补在平台左端
        if (!hasB) { insertIdx.push_back(pab.second); }      // 缺右边界角 → 补在平台右端
    }

    int changed = static_cast<int>(insertIdx.size());
    for (int k = 0; k < m0; ++k) { if (removeFlag[k]) { ++changed; } }
    if (changed == 0) { return keyIndexes; }  // 所有平台都正确 → 原样返回(幂等)

    // 应用：留下未删的 + 补入新边界角(去重) → 按 s 排序去重
    std::vector<int> result;
    for (int k = 0; k < m0; ++k) { if (!removeFlag[k]) { result.push_back(keyIndexes[k]); } }
    for (int idx : insertIdx) { if (!VecContains(result, idx)) { result.push_back(idx); } }
    const bool ascending = projected[keyIndexes.back()].s >= projected[keyIndexes.front()].s;
    std::sort(result.begin(), result.end(), [&](int a, int b) {
        return ascending ? projected[a].s < projected[b].s : projected[a].s > projected[b].s;
    });
    result.erase(std::unique(result.begin(), result.end()), result.end());
    if (changedCount != nullptr) { *changedCount = changed; }
    return result;
}

// 基础焊道首尾段截断：沿累积弧长，从【开头】(首点侧)截掉 headMm、从【结尾】(末点侧)截掉 tailMm 的点。
// 注意是按【点列数组顺序】算开头/结尾，与实际焊接方向无关——用于剔除扫描进/出端的坏点。
// 在拟合提取关键点【之前】执行。对②③④各拟合方案通用。对应 RobotCalculation::TrimLowerWeldPathByArcLength。
std::vector<IndexedPoint3D> TrimLowerWeldPathByArcLength(
    const std::vector<IndexedPoint3D>& pts, double headMm, double tailMm)
{
    const int n = static_cast<int>(pts.size());
    const double headCut = std::max(0.0, headMm);
    const double tailCut = std::max(0.0, tailMm);
    if (n < 3 || (headCut <= 0.0 && tailCut <= 0.0))
    {
        return pts;  // 点太少或没要求截断 → 原样返回
    }
    // 逐点累积弧长 arc[i]
    std::vector<double> arc(n, 0.0);
    for (int i = 1; i < n; ++i)
    {
        arc[i] = arc[i - 1] + Norm(pts[i].point - pts[i - 1].point);
    }
    const double total = arc[n - 1];
    if (headCut + tailCut >= total)
    {
        return pts;  // 要截的比总长还长 → 放弃，避免截空
    }
    int from = 0;
    while (from < n - 1 && arc[from] < headCut) { ++from; }        // 开头：跳过弧长 < headCut 的点
    int to = n - 1;
    while (to > 0 && (total - arc[to]) < tailCut) { --to; }        // 结尾：跳过「距末点弧长 < tailCut」的点
    if (to - from + 1 < 2)
    {
        return pts;  // 截完不足两点 → 放弃
    }
    std::vector<IndexedPoint3D> out;
    out.reserve(to - from + 1);
    for (int i = from; i <= to; ++i)
    {
        out.push_back(pts[i]);
    }
    return out;
}
}  // namespace (波纹板拐点管线)

int BilateralPresmoothSdkBaseWeld(std::vector<IndexedPoint3D>& points, double windowMm, double edgeMm)
{
    const int n = static_cast<int>(points.size());
    if (n < 5 || windowMm <= 1e-6 || edgeMm <= 1e-6)
    {
        return 0;
    }

    std::vector<double> arc(n, 0.0);
    for (int i = 1; i < n; ++i)
    {
        arc[i] = arc[i - 1] + Norm(points[i].point - points[i - 1].point);
    }

    const int tanWin = 2;  // 局部切线用 ±tanWin 个点中心差分估，抗单点抖动
    auto localTangent = [&](int i) -> Point3D
    {
        const int a = std::max(0, i - tanWin);
        const int b = std::min(n - 1, i + tanWin);
        const Point3D t = points[b].point - points[a].point;
        const double nrm = Norm(t);
        return nrm > 1e-9 ? Point3D{ t.x / nrm, t.y / nrm, t.z / nrm } : Point3D{ 0.0, 1.0, 0.0 };
    };

    const double sigmaS2x2 = 2.0 * windowMm * windowMm;
    const double sigmaR2x2 = 2.0 * edgeMm * edgeMm;
    const double halfSpan = 3.0 * windowMm;  // 高斯 3σ 截断

    std::vector<Point3D> out(static_cast<std::size_t>(n));
    int moved = 0;
    for (int i = 0; i < n; ++i)
    {
        // 首末点视为起终点，不动（与主程序跳过 Start/End 类型一致——SdkBase 有序点首=起、末=终）。
        if (i == 0 || i == n - 1)
        {
            out[i] = points[i].point;
            continue;
        }
        const Point3D ti = localTangent(i);
        Point3D acc = points[i].point;  // 中心点 ds=0,perp=0 → 权重 1
        double wsum = 1.0;
        auto accumulate = [&](int j)
        {
            const Point3D d = points[j].point - points[i].point;
            const Point3D dperp = d - ti * Dot(d, ti);
            const double perp = Norm(dperp);
            const double ds = std::abs(arc[j] - arc[i]);
            const double w = std::exp(-ds * ds / sigmaS2x2) * std::exp(-perp * perp / sigmaR2x2);
            acc = acc + points[j].point * w;
            wsum += w;
        };
        for (int j = i - 1; j >= 0; --j)
        {
            if (arc[i] - arc[j] > halfSpan) break;
            accumulate(j);
        }
        for (int j = i + 1; j < n; ++j)
        {
            if (arc[j] - arc[i] > halfSpan) break;
            accumulate(j);
        }
        out[i] = (wsum > 1e-12) ? Point3D{ acc.x / wsum, acc.y / wsum, acc.z / wsum } : points[i].point;
        if (Norm(out[i] - points[i].point) > 1e-6)
        {
            ++moved;
        }
    }
    for (int i = 0; i < n; ++i)
    {
        points[i].point = out[i];
    }
    return moved;
}

// 先测后焊几何主入口：输入有序的基础焊道点(SdkBase 或激光特征点)，输出 PreservePath 轨迹、分类点和关键点(拐点)。
// 与主程序 RobotCalculation::AnalyzeMeasureThenWeldLowerWeldPathGeometry 逐步对齐，整条管线顺序如下：
//   0. 有限性过滤 → 首尾段截断(可选) → MAD/分支/坡面波动去噪(若输入未去噪)
//   1. 建主轴坐标系 BuildAxes → 投影成 (s,h,smoothH) ProjectGeometryPoints
//   2. 拐点检测：已去噪输入(SdkBaseWeldFit)走【方位角法 BuildAzimuthCornerKeyIndexes + 直线化兜底】；
//      否则走【Douglas-Peucker BuildGeometryKeyIndexes + 短同类游程清理】
//   3. 相干弓自适应补点 RefineCornersByCoherentBow(默认关，已被平台重算取代)
//   4. 搭接错位检测 DetectLapMisalignmentBoundaries：把台阶点注入关键点并用 isLapStepKey 标记(后续各步豁免)
//   5. 结构纠正级联（每步改变关键点后都按 lap 角 s 重建 isLapStepKey 对齐）：
//        平台重算 RefitCornersByPlatformPattern → 端区周期补漏 RecoverEndRegionCornersByPeriod →
//        删错 MergeTooCloseSameTypeCorners → 按平台边界重定 SnapCornersToPlatforms
//   6. 关键点求交定位 BuildFittedKeyPoints → 分类 start/end/inner/outer + 2mm 加密普通点。
// 注：SdkBase 双边预平滑 BilateralPresmoothSdkBaseWeld 是【本函数之外】的可选预处理，由调用方在传入前先调。
AnalysisResult AnalyzeMeasureThenWeldPath(const std::vector<IndexedPoint3D>& inputPoints, const FilterFitParams& params)
{
    AnalysisResult result;
    result.filterResult.inputPointCount = static_cast<int>(inputPoints.size());
    if (inputPoints.empty())
    {
        result.error = "input is empty.";
        return result;
    }
    if (params.sampleStep <= 0.0)
    {
        result.error = "sampleStep must be greater than zero.";
        return result;
    }

    std::vector<IndexedPoint3D> validPoints = FinitePoints(inputPoints);

    // 基础焊道首尾段截断（拟合提取关键点之前、去噪之前；②③④拟合方案通用，与焊接方向解耦）。
    if (params.enableEdgeTruncate && (params.truncateHeadMm > 0.0 || params.truncateTailMm > 0.0))
    {
        validPoints = TrimLowerWeldPathByArcLength(validPoints, params.truncateHeadMm, params.truncateTailMm);
    }

    // 输入已去噪(SDK is_remove_noise 开)时跳过自身 MAD 去噪/分支去噪与投影平滑——重复处理会削圆尖角、移位拐点。
    const bool skipDenoise = params.inputAlreadyDenoised;
    int denoiseRejected = 0;
    if (!skipDenoise)
    {
        validPoints = RemoveLocalOutliers(validPoints, params, &denoiseRejected);
    }
    if (static_cast<int>(validPoints.size()) < std::max(2, params.minPointCount))
    {
        result.error = "too few valid points for geometry feature fitting.";
        return result;
    }

    const Axes axes = BuildAxes(validPoints, params.sampleAxis);
    std::vector<ProjectedPoint> projected =
        ProjectGeometryPoints(validPoints, axes, skipDenoise ? 0 : params.smoothRadius);
    if (!skipDenoise)
    {
        int branchRejected = 0;
        projected = RemoveProjectedBranchOutliers(projected, params, &branchRejected);
        denoiseRejected += branchRejected;
    }
    if (params.geometryStrategy == GeometryStrategy::SlopeWaveFiltered)
    {
        int slopeRejected = 0;
        projected = RemoveSlopeWaveOutliers(projected, params, &slopeRejected);
        denoiseRejected += slopeRejected;
    }
    if (static_cast<int>(projected.size()) < std::max(2, params.minPointCount))
    {
        result.error = "too few points remain after geometry denoise.";
        return result;
    }

    // 关键点检测：已去噪输入(SdkBaseWeldFit)用【方位角法】+直线化兜底；否则用 Douglas-Peucker + 短同类游程清理。
    std::vector<int> keyIndexes;
    if (skipDenoise)
    {
        keyIndexes = BuildAzimuthCornerKeyIndexes(projected, params);
        keyIndexes = SplitNonStraightAzimuthSegments(projected, keyIndexes, params);
    }
    else
    {
        keyIndexes = BuildGeometryKeyIndexes(projected, params);
        keyIndexes = PruneShortSameTypeRuns(projected, keyIndexes, params);
    }

    // 起终点先验自适应细化(相干弓，两条路径通用；默认关，已被平台重算取代)。
    keyIndexes = RefineCornersByCoherentBow(projected, keyIndexes, params);

    // 板材搭接错位检测：错位点作硬段边界注入 keyIndexes(在 prune/refine 之后注入避免被清洗)；isLapStepKey 标记台阶端点。
    std::vector<char> isLapStepKey(keyIndexes.size(), 0);
    if (params.enableLapMisalignmentSplit)
    {
        const std::vector<int> lapCenters = DetectLapMisalignmentBoundaries(projected, params);
        std::vector<int> stepValues;
        std::vector<int> merged = keyIndexes;
        for (int ci : lapCenters)
        {
            if (ci - 1 > 0 && ci < static_cast<int>(projected.size()) - 1)
            {
                merged.push_back(ci - 1);
                merged.push_back(ci);
                stepValues.push_back(ci - 1);
                stepValues.push_back(ci);
            }
        }
        if (!stepValues.empty())
        {
            std::sort(merged.begin(), merged.end());
            merged.erase(std::unique(merged.begin(), merged.end()), merged.end());
            keyIndexes = merged;
            isLapStepKey.assign(keyIndexes.size(), 0);
            for (int k = 0; k < static_cast<int>(keyIndexes.size()); ++k)
                if (VecContains(stepValues, keyIndexes[k])) isLapStepKey[k] = 1;
        }
    }

    // 平台重算(II/OO 结构) → 端区周期补漏/删错 → 按平台边界重定。每步改变 keyIndexes 个数/顺序后，
    // 据 lap 角的 projected 索引重建 isLapStepKey 对齐(与主程序 AnalyzeMeasureThenWeldLowerWeldPathGeometry 一致)。
    {
        std::vector<int> lapValues;
        for (int k = 0; k < static_cast<int>(keyIndexes.size()) && k < static_cast<int>(isLapStepKey.size()); ++k)
            if (isLapStepKey[k]) lapValues.push_back(keyIndexes[k]);
        auto rebuildLap = [&]() {
            isLapStepKey.assign(keyIndexes.size(), 0);
            for (int k = 0; k < static_cast<int>(keyIndexes.size()); ++k)
                if (VecContains(lapValues, keyIndexes[k])) isLapStepKey[k] = 1;
        };
        keyIndexes = RefitCornersByPlatformPattern(projected, keyIndexes, isLapStepKey, params);
        rebuildLap();
        if (params.enableEndPeriodCornerRecover)
        {
            int recovered = 0;
            keyIndexes = RecoverEndRegionCornersByPeriod(
                projected, keyIndexes, isLapStepKey,
                params.endPeriodRatioThreshold, params.endPeriodMinBendDeg, &recovered);
            if (recovered > 0) rebuildLap();
            int mergedCount = 0;
            keyIndexes = MergeTooCloseSameTypeCorners(
                projected, keyIndexes, isLapStepKey, params.endPeriodMergeFrac, &mergedCount);
            if (mergedCount > 0) rebuildLap();
        }
        if (params.enablePlatformCornerSnap)
        {
            int snapped = 0;
            keyIndexes = SnapCornersToPlatforms(
                projected, keyIndexes, isLapStepKey,
                params.platformSnapFlatSlope, params.platformSnapMinFrac, &snapped);
            if (snapped > 0) rebuildLap();
        }
    }

    if (keyIndexes.size() < 2)
    {
        result.error = "could not generate start/end key points.";
        return result;
    }

    const std::vector<Point3D> fittedKeyPoints =
        BuildFittedKeyPoints(projected, keyIndexes, axes, params.useSlopeConsistentCornerFit, isLapStepKey);
    if (fittedKeyPoints.size() != keyIndexes.size())
    {
        result.error = "could not fit corner key points.";
        return result;
    }

    if (params.exportFitDebugCloud && !params.fitDebugDir.empty())
    {
        ExportFitDebugClouds(
            projected, keyIndexes, fittedKeyPoints, axes,
            params.useSlopeConsistentCornerFit, params.fitDebugDir);
    }

    result.filterResult.points.reserve(projected.size());
    for (const ProjectedPoint& point : projected)
    {
        result.filterResult.points.push_back({
            point.inputIndex,
            point.point,
            denoiseRejected > 0 ? "geometry_denoised" : "geometry_original"
        });
    }
    result.filterResult.ok = true;
    result.filterResult.lowerPointCount = static_cast<int>(projected.size());
    result.filterResult.zContinuityRejectedCount = denoiseRejected;
    result.filterResult.measuredCount = static_cast<int>(keyIndexes.size());
    result.filterResult.fitSegmentCount = std::max(1, static_cast<int>(keyIndexes.size()) - 1);

    result.classificationResult.ok = true;
    int nextIndex = 1;
    int interpolated = 0;
    for (int ordinal = 0; ordinal < static_cast<int>(keyIndexes.size()); ++ordinal)
    {
        const Point3D current = fittedKeyPoints[static_cast<std::size_t>(ordinal)];
        const WeldPointType type = GeometryCornerType(projected, keyIndexes, ordinal);
        const std::string source =
            type == WeldPointType::Start ? "geometry_start"
            : (type == WeldPointType::End ? "geometry_end"
            : (type == WeldPointType::InnerCorner ? "geometry_inner" : "geometry_outer"));

        ClassifiedPoint keyPoint;
        keyPoint.index = projected[static_cast<std::size_t>(keyIndexes[static_cast<std::size_t>(ordinal)])].inputIndex;
        keyPoint.point = current;
        keyPoint.type = type;
        keyPoint.source = source;
        result.keyPoints.push_back(keyPoint);

        AppendClassifiedPoint(result.classificationResult.points, nextIndex, current, type, source);
        if (ordinal + 1 >= static_cast<int>(keyIndexes.size()))
        {
            continue;
        }

        const Point3D next = fittedKeyPoints[static_cast<std::size_t>(ordinal + 1)];
        const Point3D delta = next - current;
        const double length = Norm(delta);
        if (length <= kEpsilon)
        {
            continue;
        }
        // This is the "拐点生成" expansion used by the workflow:
        // fitted start/corner/end key points are kept, then normal weld points are interpolated every sampleStep mm.
        for (double distance = params.sampleStep; distance < length - 1e-9; distance += params.sampleStep)
        {
            AppendClassifiedPoint(
                result.classificationResult.points,
                nextIndex,
                current + delta * (distance / length),
                WeldPointType::Normal,
                "geometry_2mm");
            ++interpolated;
        }
    }
    result.filterResult.interpolatedCount = interpolated;
    result.filterResult.extendedCount = static_cast<int>(result.classificationResult.points.size());
    CountClassification(result.classificationResult);
    result.ok = true;
    return result;
}

FilterResult ProjectWorkpieceCloudToLowerWeldPath(
    const std::vector<IndexedPoint3D>& workpieceCloudInput,
    const std::vector<IndexedPoint3D>& seedPathInput,
    const FilterFitParams& params)
{
    FilterResult result;
    result.inputPointCount = static_cast<int>(workpieceCloudInput.size());
    if (workpieceCloudInput.size() < 3)
    {
        result.error = "workpiece cloud has too few points.";
        return result;
    }
    if (seedPathInput.size() < 2)
    {
        result.error = "seed path has too few points.";
        return result;
    }

    std::vector<int> sortedIndexes;
    sortedIndexes.reserve(workpieceCloudInput.size());
    for (int index = 0; index < static_cast<int>(workpieceCloudInput.size()); ++index)
    {
        if (IsFinite(workpieceCloudInput[static_cast<std::size_t>(index)].point))
        {
            sortedIndexes.push_back(index);
        }
    }
    std::sort(sortedIndexes.begin(), sortedIndexes.end(),
        [&](int left, int right)
        {
            return WorkpieceStationValue(workpieceCloudInput[static_cast<std::size_t>(left)].point, params.sampleAxis)
                < WorkpieceStationValue(workpieceCloudInput[static_cast<std::size_t>(right)].point, params.sampleAxis);
        });
    if (sortedIndexes.size() < 3)
    {
        result.error = "workpiece cloud has too few finite points.";
        return result;
    }

    const double sampleStep = params.sampleStep > 0.0 ? params.sampleStep : 2.0;
    const double stationWindow = std::max(2.5, sampleStep * 1.5);
    const double transverseWindow = std::max(10.0, params.searchWindow > 0.0 ? params.searchWindow * 1.5 : 12.0);
    const double zBandBelow = std::max(14.0, params.zContinuityThreshold > 0.0 ? params.zContinuityThreshold * 4.0 : 14.0);
    const double zBandAbove = std::max(12.0, params.zJumpThreshold > 0.0 ? params.zJumpThreshold * 2.5 : 12.0);

    auto lowerStationBound = [&](double value)
    {
        return std::lower_bound(sortedIndexes.begin(), sortedIndexes.end(), value,
            [&](int pointIndex, double station)
            {
                return WorkpieceStationValue(workpieceCloudInput[static_cast<std::size_t>(pointIndex)].point, params.sampleAxis) < station;
            });
    };

    std::vector<double> projectedZValues;
    std::vector<char> projectedZValid;
    projectedZValues.reserve(seedPathInput.size());
    projectedZValid.reserve(seedPathInput.size());
    int candidateCount = 0;
    int visitedLocalCount = 0;
    for (const IndexedPoint3D& seed : seedPathInput)
    {
        if (!IsFinite(seed.point))
        {
            continue;
        }
        const double seedStation = WorkpieceStationValue(seed.point, params.sampleAxis);
        const double seedTransverse = WorkpieceTransverseValue(seed.point, params.sampleAxis);
        const auto begin = lowerStationBound(seedStation - stationWindow);
        const auto end = lowerStationBound(seedStation + stationWindow);
        std::vector<double> localZ;
        for (auto it = begin; it != end; ++it)
        {
            const Point3D& point = workpieceCloudInput[static_cast<std::size_t>(*it)].point;
            if (std::abs(WorkpieceTransverseValue(point, params.sampleAxis) - seedTransverse) > transverseWindow)
            {
                continue;
            }
            if (point.z < seed.point.z - zBandBelow || point.z > seed.point.z + zBandAbove)
            {
                continue;
            }
            localZ.push_back(point.z);
            ++visitedLocalCount;
        }
        if (!localZ.empty())
        {
            // Match the workflow intent: keep the middle/bottom profile rather than the highest reflected layer.
            std::sort(localZ.begin(), localZ.end());
            const std::size_t beginIndex = localZ.size() / 5;
            const std::size_t endIndex = std::max(beginIndex + 1, localZ.size() * 3 / 5);
            std::vector<double> middleLayer(localZ.begin() + static_cast<std::ptrdiff_t>(beginIndex),
                localZ.begin() + static_cast<std::ptrdiff_t>(std::min(endIndex, localZ.size())));
            projectedZValues.push_back(Median(middleLayer));
            projectedZValid.push_back(1);
            candidateCount += static_cast<int>(localZ.size());
        }
        else
        {
            projectedZValues.push_back(seed.point.z);
            projectedZValid.push_back(0);
        }
    }

    if (projectedZValues.size() < 2 || candidateCount < 12)
    {
        result.error = "not enough bottom-profile candidates in workpiece cloud.";
        return result;
    }

    std::vector<double> smoothed = projectedZValues;
    const int smoothRadius = std::clamp(params.smoothRadius, 1, 4);
    for (int index = 0; index < static_cast<int>(projectedZValues.size()); ++index)
    {
        const int begin = std::max(0, index - smoothRadius);
        const int end = std::min(static_cast<int>(projectedZValues.size()) - 1, index + smoothRadius);
        std::vector<double> values;
        for (int sample = begin; sample <= end; ++sample)
        {
            if (projectedZValid[static_cast<std::size_t>(sample)])
            {
                values.push_back(projectedZValues[static_cast<std::size_t>(sample)]);
            }
        }
        if (!values.empty())
        {
            smoothed[static_cast<std::size_t>(index)] = Median(values);
        }
    }

    int seedOrdinal = 0;
    int fallbackCount = 0;
    for (const IndexedPoint3D& seed : seedPathInput)
    {
        if (!IsFinite(seed.point))
        {
            continue;
        }
        Point3D projected = seed.point;
        projected.z = smoothed[static_cast<std::size_t>(seedOrdinal)];
        if (!projectedZValid[static_cast<std::size_t>(seedOrdinal)])
        {
            ++fallbackCount;
        }
        result.points.push_back({
            seed.index,
            projected,
            projectedZValid[static_cast<std::size_t>(seedOrdinal)]
                ? "workpiece_middle_bottom_profile_projected"
                : "workpiece_middle_bottom_seed_fallback"
        });
        ++seedOrdinal;
    }

    if (static_cast<int>(result.points.size()) < std::max(2, params.minPointCount))
    {
        result.error = "too few projected weld path points.";
        return result;
    }

    result.lowerPointCount = candidateCount;
    result.zContinuityRejectedCount = visitedLocalCount - candidateCount;
    result.segmentRejectedCount = visitedLocalCount;
    result.fitSegmentCount = static_cast<int>(result.points.size()) - fallbackCount;
    result.measuredCount = static_cast<int>(result.points.size());
    result.interpolatedCount = result.fitSegmentCount;
    result.extendedCount = fallbackCount;
    result.ok = true;
    return result;
}

std::vector<ExternalTrackPoint> ResampleTrackPoints(const std::vector<ExternalTrackPoint>& rawTrackPoints, double stepMm)
{
    std::vector<ExternalTrackPoint> output;
    if (rawTrackPoints.empty())
    {
        return output;
    }

    const double safeStep = stepMm > 0.0 ? stepMm : 2.0;
    output.reserve(rawTrackPoints.size() * 2);
    int nextIndex = 1;
    auto append = [&](const Point3D& point, TrackPointType type)
    {
        output.push_back({ nextIndex++, point, type });
    };
    append(rawTrackPoints.front().point, rawTrackPoints.front().type);
    for (std::size_t index = 0; index + 1 < rawTrackPoints.size(); ++index)
    {
        const Point3D begin = rawTrackPoints[index].point;
        const Point3D end = rawTrackPoints[index + 1].point;
        const Point3D delta = end - begin;
        const double length = Norm(delta);
        if (length > safeStep)
        {
            for (double distance = safeStep; distance < length - 1e-9; distance += safeStep)
            {
                append(begin + delta * (distance / length), TrackPointType::Normal);
            }
        }
        append(end, rawTrackPoints[index + 1].type);
    }
    if (!output.empty())
    {
        output.front().type = TrackPointType::Start;
        output.back().type = TrackPointType::End;
    }
    return output;
}

AnalysisResult BuildAnalysisFromExternalTrack(
    const std::vector<ExternalTrackPoint>& trackPoints,
    int originalInputPointCount,
    const FilterFitParams& params)
{
    AnalysisResult result;
    result.filterResult.inputPointCount = originalInputPointCount;
    if (trackPoints.size() < 2)
    {
        result.error = "external track has too few points.";
        return result;
    }

    const std::vector<int> keyIndexes = BuildTrackKeyIndexes(trackPoints);
    const std::vector<std::string> segmentKinds = AssignSegmentKinds(trackPoints, keyIndexes, params.sampleAxis);
    for (int index = 0; index < static_cast<int>(trackPoints.size()); ++index)
    {
        const ExternalTrackPoint& trackPoint = trackPoints[static_cast<std::size_t>(index)];
        const bool isFirst = index == 0;
        const bool isLast = index == static_cast<int>(trackPoints.size()) - 1;
        WeldPointType type = WeldPointType::Normal;
        if (isFirst || trackPoint.type == TrackPointType::Start)
        {
            type = WeldPointType::Start;
        }
        else if (isLast || trackPoint.type == TrackPointType::End)
        {
            type = WeldPointType::End;
        }
        else if (trackPoint.type == TrackPointType::Corner)
        {
            const int keyOrdinal = FindKeyOrdinal(keyIndexes, index);
            type = keyOrdinal >= 0
                ? CornerTypeFromAdjacentSegments(segmentKinds, keyOrdinal)
                : WeldPointType::InnerCorner;
        }

        const std::string source = SourceName(trackPoint.type);
        result.filterResult.points.push_back({ trackPoint.index, trackPoint.point, source });
        ClassifiedPoint classified;
        classified.index = trackPoint.index;
        classified.point = trackPoint.point;
        classified.type = type;
        classified.source = source;
        const int keyOrdinal = FindKeyOrdinal(keyIndexes, index);
        if (keyOrdinal >= 0 && keyOrdinal < static_cast<int>(segmentKinds.size()))
        {
            classified.segmentKindAfter = segmentKinds[static_cast<std::size_t>(keyOrdinal)];
        }
        result.classificationResult.points.push_back(classified);
        if (type != WeldPointType::Normal && type != WeldPointType::Noise)
        {
            result.keyPoints.push_back(classified);
        }
    }

    const int outputCount = static_cast<int>(trackPoints.size());
    const int keyCount = static_cast<int>(result.keyPoints.size());
    result.filterResult.ok = true;
    result.filterResult.lowerPointCount = outputCount;
    result.filterResult.fitSegmentCount = std::max(1, keyCount - 1);
    result.filterResult.measuredCount = keyCount;
    result.filterResult.interpolatedCount = std::max(0, outputCount - keyCount);
    result.filterResult.extendedCount = result.filterResult.interpolatedCount;
    result.classificationResult.ok = true;
    CountClassification(result.classificationResult);

    if (result.classificationResult.startCount <= 0 || result.classificationResult.endCount <= 0)
    {
        result.error = "external track lacks start or end point.";
        return result;
    }
    result.ok = true;
    return result;
}

int WeldPointTypeCode(WeldPointType type)
{
    return static_cast<int>(type);
}

const char* WeldPointTypeName(WeldPointType type)
{
    switch (type)
    {
    case WeldPointType::Start:
        return "start";
    case WeldPointType::End:
        return "end";
    case WeldPointType::InnerCorner:
        return "inner_corner";
    case WeldPointType::OuterCorner:
        return "outer_corner";
    case WeldPointType::Noise:
        return "noise";
    case WeldPointType::Normal:
    default:
        return "normal";
    }
}

int TrackPointTypeCode(TrackPointType type)
{
    switch (type)
    {
    case TrackPointType::Start:
        return 1;
    case TrackPointType::End:
        return 2;
    case TrackPointType::Corner:
        return 3;
    case TrackPointType::Normal:
    default:
        return 5;
    }
}

const char* TrackPointTypeName(TrackPointType type)
{
    switch (type)
    {
    case TrackPointType::Start:
        return "start";
    case TrackPointType::End:
        return "end";
    case TrackPointType::Corner:
        return "corner";
    case TrackPointType::Normal:
    default:
        return "normal";
    }
}

std::vector<std::string> BuildFilterOutputLines(const FilterResult& result)
{
    std::vector<std::string> lines;
    lines.reserve(result.points.size() + 1);
    lines.push_back("index x y z source");
    for (const FilterPoint& point : result.points)
    {
        lines.push_back(PointLine(point.index, point.point, point.source));
    }
    return lines;
}

std::vector<std::string> BuildClassifiedOutputLines(const ClassificationResult& result)
{
    std::vector<std::string> lines;
    lines.reserve(result.points.size() + 2);
    lines.push_back("# index x y z type_code type_name source");
    lines.push_back("# 1=start 2=end 3=inner_corner 4=outer_corner 5=normal 6=noise");
    for (const ClassifiedPoint& point : result.points)
    {
        std::ostringstream out;
        out << PointLine(point.index, point.point, "")
            << ' ' << WeldPointTypeCode(point.type)
            << ' ' << WeldPointTypeName(point.type)
            << ' ' << (point.source.empty() ? "-" : point.source);
        lines.push_back(out.str());
    }
    return lines;
}

std::vector<std::string> BuildKeyPointOutputLines(const std::vector<ClassifiedPoint>& keyPoints)
{
    std::vector<std::string> lines;
    lines.reserve(keyPoints.size() + 2);
    lines.push_back("# source_index x y z type_code type_name source");
    lines.push_back("# 1=start 2=end 3=inner_corner 4=outer_corner");
    for (const ClassifiedPoint& point : keyPoints)
    {
        if (point.type == WeldPointType::Normal || point.type == WeldPointType::Noise)
        {
            continue;
        }
        std::ostringstream out;
        out << PointLine(point.index, point.point, "")
            << ' ' << WeldPointTypeCode(point.type)
            << ' ' << WeldPointTypeName(point.type)
            << ' ' << (point.source.empty() ? "-" : point.source);
        lines.push_back(out.str());
    }
    return lines;
}

std::vector<std::string> BuildNoiseOutputLines(
    const std::vector<IndexedPoint3D>& inputPoints,
    const FilterResult& fitResult)
{
    std::set<int> validIndexes;
    for (const FilterPoint& point : fitResult.points)
    {
        validIndexes.insert(point.index);
    }

    std::vector<std::string> lines;
    lines.reserve(inputPoints.size() + 2);
    lines.push_back("# index x y z type_code type_name source");
    lines.push_back("# 1=start 2=end 3=inner_corner 4=outer_corner 5=normal 6=noise");
    for (const IndexedPoint3D& point : inputPoints)
    {
        if (validIndexes.find(point.index) != validIndexes.end())
        {
            continue;
        }
        std::ostringstream out;
        out << PointLine(point.index, point.point, "") << " 6 noise raw";
        lines.push_back(out.str());
    }
    return lines;
}

std::vector<std::string> BuildExternalTrackOutputLines(
    const std::vector<ExternalTrackPoint>& points,
    const std::string& source)
{
    std::vector<std::string> lines;
    lines.reserve(points.size() + 2);
    lines.push_back("# index x y z sdk_type_code sdk_type_name source");
    lines.push_back("# 1=start 2=end 3=corner 5=normal");
    for (const ExternalTrackPoint& point : points)
    {
        std::ostringstream out;
        out << PointLine(point.index, point.point, "")
            << ' ' << TrackPointTypeCode(point.type)
            << ' ' << TrackPointTypeName(point.type)
            << ' ' << source;
        lines.push_back(out.str());
    }
    return lines;
}
}
