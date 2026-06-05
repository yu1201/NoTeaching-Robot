#include "MeasureThenWeldFilterFit.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <numeric>
#include <set>
#include <sstream>

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
    bool useSlopeConsistentCornerFit)
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
        if (position > 0 && position + 1 < static_cast<int>(keyIndexes.size()))
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
    int denoiseRejected = 0;
    validPoints = RemoveLocalOutliers(validPoints, params, &denoiseRejected);
    if (static_cast<int>(validPoints.size()) < std::max(2, params.minPointCount))
    {
        result.error = "too few valid points for geometry feature fitting.";
        return result;
    }

    const Axes axes = BuildAxes(validPoints, params.sampleAxis);
    std::vector<ProjectedPoint> projected = ProjectGeometryPoints(validPoints, axes, params.smoothRadius);
    int branchRejected = 0;
    projected = RemoveProjectedBranchOutliers(projected, params, &branchRejected);
    denoiseRejected += branchRejected;
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

    std::vector<int> keyIndexes = BuildGeometryKeyIndexes(projected, params);
    keyIndexes = PruneShortSameTypeRuns(projected, keyIndexes, params);
    if (keyIndexes.size() < 2)
    {
        result.error = "could not generate start/end key points.";
        return result;
    }

    const std::vector<Point3D> fittedKeyPoints =
        BuildFittedKeyPoints(projected, keyIndexes, axes, params.useSlopeConsistentCornerFit);
    if (fittedKeyPoints.size() != keyIndexes.size())
    {
        result.error = "could not fit corner key points.";
        return result;
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
