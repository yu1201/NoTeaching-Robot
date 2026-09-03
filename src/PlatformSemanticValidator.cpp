#include "PlatformSemanticValidator.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

namespace PlatformSemanticValidator
{
namespace
{
constexpr double kMinimumBandSeparation = 3.0;
constexpr double kEpsilon = 1e-9;

struct BandSample
{
    double path = 0.0;
    double value = 0.0;
    double label = 0.0;
};

struct BandFit
{
    bool valid = false;
    bool sufficientEvidence = false;
    int positiveCount = 0;
    int negativeCount = 0;
    double intercept = 0.0;
    double drift = 0.0;
    double separation = 0.0;
    double residualRms = 0.0;
    double maxResidual = 0.0;
};

bool IsFinite(double value)
{
    return std::isfinite(value);
}

double Median(std::vector<double> values)
{
    if (values.empty())
    {
        return 0.0;
    }
    const std::size_t middle = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + middle, values.end());
    double result = values[middle];
    if ((values.size() % 2) == 0)
    {
        const auto lower = std::max_element(values.begin(), values.begin() + middle);
        result = (*lower + result) * 0.5;
    }
    return result;
}

bool SolveThreeByThree(
    std::array<std::array<double, 3>, 3> matrix,
    std::array<double, 3> rhs,
    std::array<double, 3>& solution)
{
    for (int pivot = 0; pivot < 3; ++pivot)
    {
        int best = pivot;
        for (int row = pivot + 1; row < 3; ++row)
        {
            if (std::abs(matrix[row][pivot]) > std::abs(matrix[best][pivot]))
            {
                best = row;
            }
        }
        if (std::abs(matrix[best][pivot]) <= 1e-10)
        {
            return false;
        }
        if (best != pivot)
        {
            std::swap(matrix[best], matrix[pivot]);
            std::swap(rhs[best], rhs[pivot]);
        }
        const double divisor = matrix[pivot][pivot];
        for (int column = pivot; column < 3; ++column)
        {
            matrix[pivot][column] /= divisor;
        }
        rhs[pivot] /= divisor;
        for (int row = 0; row < 3; ++row)
        {
            if (row == pivot)
            {
                continue;
            }
            const double factor = matrix[row][pivot];
            for (int column = pivot; column < 3; ++column)
            {
                matrix[row][column] -= factor * matrix[pivot][column];
            }
            rhs[row] -= factor * rhs[pivot];
        }
    }
    solution = rhs;
    return std::all_of(solution.begin(), solution.end(), IsFinite);
}

bool WeightedBandFit(
    const std::vector<BandSample>& samples,
    const std::vector<double>& weights,
    double centeredPath,
    std::array<double, 3>& coefficients)
{
    std::array<std::array<double, 3>, 3> normal{};
    std::array<double, 3> rhs{};
    for (std::size_t index = 0; index < samples.size(); ++index)
    {
        const double weight = weights[index];
        const std::array<double, 3> feature{
            1.0,
            samples[index].path - centeredPath,
            samples[index].label
        };
        for (int row = 0; row < 3; ++row)
        {
            rhs[row] += weight * feature[row] * samples[index].value;
            for (int column = 0; column < 3; ++column)
            {
                normal[row][column] += weight * feature[row] * feature[column];
            }
        }
    }
    return SolveThreeByThree(normal, rhs, coefficients);
}

BandFit FitTwoBands(const std::vector<BandSample>& samples)
{
    BandFit fit;
    if (samples.empty())
    {
        return fit;
    }
    for (const BandSample& sample : samples)
    {
        if (!IsFinite(sample.path) || !IsFinite(sample.value)
            || std::abs(sample.label) < 0.25)
        {
            return fit;
        }
        sample.label > 0.0 ? ++fit.positiveCount : ++fit.negativeCount;
    }
    if (fit.positiveCount == 0 || fit.negativeCount == 0)
    {
        return fit;
    }

    fit.valid = true;
    double positiveSum = 0.0;
    double negativeSum = 0.0;
    for (const BandSample& sample : samples)
    {
        (sample.label > 0.0 ? positiveSum : negativeSum) += sample.value;
    }
    fit.separation = positiveSum / fit.positiveCount
        - negativeSum / fit.negativeCount;

    fit.sufficientEvidence = samples.size() >= 4
        && fit.positiveCount >= 2
        && fit.negativeCount >= 2;
    if (!fit.sufficientEvidence)
    {
        return fit;
    }

    double centeredPath = 0.0;
    for (const BandSample& sample : samples)
    {
        centeredPath += sample.path;
    }
    centeredPath /= static_cast<double>(samples.size());

    std::vector<double> weights(samples.size(), 1.0);
    std::array<double, 3> coefficients{};
    for (int iteration = 0; iteration < 3; ++iteration)
    {
        if (!WeightedBandFit(samples, weights, centeredPath, coefficients))
        {
            fit.sufficientEvidence = false;
            return fit;
        }
        std::vector<double> absoluteResiduals;
        absoluteResiduals.reserve(samples.size());
        for (const BandSample& sample : samples)
        {
            const double prediction = coefficients[0]
                + coefficients[1] * (sample.path - centeredPath)
                + coefficients[2] * sample.label;
            absoluteResiduals.push_back(std::abs(sample.value - prediction));
        }
        const double robustScale = std::max(0.2, 1.4826 * Median(absoluteResiduals));
        const double huberLimit = 1.5 * robustScale;
        for (std::size_t index = 0; index < weights.size(); ++index)
        {
            weights[index] = absoluteResiduals[index] <= huberLimit
                ? 1.0
                : huberLimit / absoluteResiduals[index];
        }
    }

    fit.intercept = coefficients[0] - coefficients[1] * centeredPath;
    fit.drift = coefficients[1];
    fit.separation = coefficients[2];
    double squaredResidualSum = 0.0;
    for (const BandSample& sample : samples)
    {
        const double prediction = fit.intercept
            + fit.drift * sample.path
            + fit.separation * sample.label;
        const double residual = sample.value - prediction;
        squaredResidualSum += residual * residual;
        fit.maxResidual = std::max(fit.maxResidual, std::abs(residual));
    }
    fit.residualRms = std::sqrt(
        squaredResidualSum / static_cast<double>(samples.size()));
    return fit;
}

std::string FormatDouble(double value, int precision = 3)
{
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(precision) << value;
    return stream.str();
}

std::string CornerName(CornerType type)
{
    switch (type)
    {
    case CornerType::Inner:
        return "inner";
    case CornerType::Outer:
        return "outer";
    default:
        return "other";
    }
}

bool IsCorner(CornerType type)
{
    return type == CornerType::Inner || type == CornerType::Outer;
}

double CornerLabel(CornerType type)
{
    return type == CornerType::Inner ? 0.5 : -0.5;
}

bool IsPlatformKind(const std::string& kind)
{
    return kind == "low_platform" || kind == "high_platform";
}

bool IsSlopeKind(const std::string& kind)
{
    return kind == "rising_edge" || kind == "falling_edge";
}

bool IsAllowedTransition(const std::string& begin, const std::string& end)
{
    return (begin == "high_platform" && end == "falling_edge")
        || (begin == "falling_edge" && end == "low_platform")
        || (begin == "low_platform" && end == "rising_edge")
        || (begin == "rising_edge" && end == "high_platform");
}
}

CandidateCheck EvaluateCandidate(
    const std::vector<CandidateKeyPoint>& keyPoints,
    double flatSlopeThreshold)
{
    CandidateCheck check;
    const double threshold = std::max(0.01, flatSlopeThreshold);
    std::vector<BandSample> platformSamples;
    std::vector<const CandidateKeyPoint*> cleanCorners;
    for (const CandidateKeyPoint& point : keyPoints)
    {
        if (IsCorner(point.type) && !point.lapBoundary)
        {
            cleanCorners.push_back(&point);
        }
    }

    bool previousShapeKnown = false;
    bool previousWasSlope = false;
    for (std::size_t index = 0; index + 1 < keyPoints.size(); ++index)
    {
        const CandidateKeyPoint& begin = keyPoints[index];
        const CandidateKeyPoint& end = keyPoints[index + 1];
        if (!IsCorner(begin.type) || !IsCorner(end.type)
            || begin.lapBoundary || end.lapBoundary)
        {
            previousShapeKnown = false;
            continue;
        }
        ++check.checkedSegmentCount;
        const double pathDelta = end.path - begin.path;
        const double profileDelta = end.profile - begin.profile;
        if (!IsFinite(pathDelta) || !IsFinite(profileDelta))
        {
            check.failures.push_back("non-finite corner segment");
            check.diagnostics.push_back(
                "raw_index=" + std::to_string(begin.rawIndex) + "->"
                + std::to_string(end.rawIndex)
                + ", begin_xyz=(" + FormatDouble(begin.x) + ","
                + FormatDouble(begin.y) + "," + FormatDouble(begin.z) + ")"
                + ", end_xyz=(" + FormatDouble(end.x) + ","
                + FormatDouble(end.y) + "," + FormatDouble(end.z) + ")"
                + ", non-finite corner segment");
            continue;
        }
        const double slope = std::abs(pathDelta) > kEpsilon
            ? std::abs(profileDelta) / std::abs(pathDelta)
            : std::numeric_limits<double>::infinity();
        const bool topologyIsSlope = begin.type != end.type;
        const bool geometryIsSlope = std::abs(pathDelta) <= kEpsilon
            ? std::abs(profileDelta) > 1e-6
            : slope >= threshold;
        if (topologyIsSlope)
        {
            ++check.slopeSegmentCount;
        }
        else
        {
            ++check.platformSegmentCount;
            platformSamples.push_back({
                (begin.path + end.path) * 0.5,
                (begin.profile + end.profile) * 0.5,
                CornerLabel(begin.type)
            });
        }
        if (topologyIsSlope != geometryIsSlope)
        {
            check.failures.push_back(
                "raw_index=" + std::to_string(begin.rawIndex) + "->"
                + std::to_string(end.rawIndex) + ", topology="
                + (topologyIsSlope ? "slope" : "platform")
                + ", geometric_slope=" + FormatDouble(slope, 4));
            check.diagnostics.push_back(
                "raw_index=" + std::to_string(begin.rawIndex) + "->"
                + std::to_string(end.rawIndex)
                + ", begin_xyz=(" + FormatDouble(begin.x) + ","
                + FormatDouble(begin.y) + "," + FormatDouble(begin.z) + ")"
                + ", end_xyz=(" + FormatDouble(end.x) + ","
                + FormatDouble(end.y) + "," + FormatDouble(end.z) + ")"
                + ", path=" + FormatDouble(begin.path) + "->"
                + FormatDouble(end.path)
                + ", profile=" + FormatDouble(begin.profile) + "->"
                + FormatDouble(end.profile)
                + ", topology="
                + (topologyIsSlope ? "slope" : "platform")
                + ", geometric_slope=" + FormatDouble(slope, 4));
        }
        if (previousShapeKnown && previousWasSlope == topologyIsSlope)
        {
            check.failures.push_back(
                "raw_index=" + std::to_string(begin.rawIndex)
                + ", adjacent segment shapes do not alternate");
            check.diagnostics.push_back(
                "raw_index=" + std::to_string(begin.rawIndex)
                + ", xyz=(" + FormatDouble(begin.x) + ","
                + FormatDouble(begin.y) + "," + FormatDouble(begin.z) + ")"
                + ", path=" + FormatDouble(begin.path)
                + ", profile=" + FormatDouble(begin.profile)
                + ", adjacent segment shapes do not alternate");
        }
        previousShapeKnown = true;
        previousWasSlope = topologyIsSlope;
    }

    const BandFit fit = FitTwoBands(platformSamples);
    check.sufficientEvidence = fit.sufficientEvidence;
    check.bandSeparation = std::abs(fit.separation);
    check.residualRms = fit.residualRms;
    check.maxResidual = fit.maxResidual;
    check.bandDirection = fit.separation > kEpsilon ? 1
        : (fit.separation < -kEpsilon ? -1 : 0);
    if (fit.sufficientEvidence)
    {
        if (check.bandSeparation < kMinimumBandSeparation)
        {
            check.failures.push_back(
                "two platform bands are not separated, gap="
                + FormatDouble(check.bandSeparation) + " mm");
        }
        if (check.residualRms > std::max(0.75, 0.18 * check.bandSeparation)
            || check.maxResidual > std::max(1.5, 0.35 * check.bandSeparation))
        {
            check.failures.push_back(
                "platform band residual is excessive, rms="
                + FormatDouble(check.residualRms) + " mm, max="
                + FormatDouble(check.maxResidual) + " mm, gap="
                + FormatDouble(check.bandSeparation) + " mm");
        }
        double directionResidualSum = 0.0;
        int directionCount = 0;
        for (std::size_t index = 0; index + 1 < keyPoints.size(); ++index)
        {
            const CandidateKeyPoint& begin = keyPoints[index];
            const CandidateKeyPoint& end = keyPoints[index + 1];
            if (!IsCorner(begin.type) || !IsCorner(end.type)
                || begin.type == end.type || begin.lapBoundary || end.lapBoundary)
            {
                continue;
            }
            const double actualDelta = end.profile - begin.profile;
            const double expectedDelta = fit.drift * (end.path - begin.path)
                + fit.separation * (CornerLabel(end.type) - CornerLabel(begin.type));
            if (actualDelta * expectedDelta <= 0.0
                || std::abs(actualDelta) < std::max(1.0, 0.25 * check.bandSeparation))
            {
                check.failures.push_back(
                    "raw_index=" + std::to_string(begin.rawIndex) + "->"
                    + std::to_string(end.rawIndex)
                    + ", slope direction conflicts with platform bands");
                check.diagnostics.push_back(
                    "raw_index=" + std::to_string(begin.rawIndex) + "->"
                    + std::to_string(end.rawIndex)
                    + ", begin_xyz=(" + FormatDouble(begin.x) + ","
                    + FormatDouble(begin.y) + "," + FormatDouble(begin.z) + ")"
                    + ", end_xyz=(" + FormatDouble(end.x) + ","
                    + FormatDouble(end.y) + "," + FormatDouble(end.z) + ")"
                    + ", path=" + FormatDouble(begin.path) + "->"
                    + FormatDouble(end.path)
                    + ", profile=" + FormatDouble(begin.profile) + "->"
                    + FormatDouble(end.profile)
                    + ", slope direction conflicts with platform bands");
            }
            directionResidualSum += std::abs(actualDelta - expectedDelta)
                / std::max(kMinimumBandSeparation, check.bandSeparation);
            ++directionCount;
        }
        const double directionResidual = directionCount > 0
            ? directionResidualSum / directionCount
            : 0.0;
        check.score = check.residualRms
                / std::max(kMinimumBandSeparation, check.bandSeparation)
            + 0.25 * check.maxResidual
                / std::max(kMinimumBandSeparation, check.bandSeparation)
            + 0.25 * directionResidual;
    }
    else
    {
        check.score = std::numeric_limits<double>::infinity();
    }

    std::ostringstream fingerprint;
    fingerprint << "corners=";
    for (const CandidateKeyPoint* point : cleanCorners)
    {
        fingerprint << (point->type == CornerType::Inner ? 'I' : 'O');
    }
    fingerprint << ";band=" << check.bandDirection
                << ";p=" << check.platformSegmentCount
                << ";s=" << check.slopeSegmentCount;
    check.fingerprint = fingerprint.str();
    check.valid = check.checkedSegmentCount > 0 && check.failures.empty();
    return check;
}

CandidateSelection SelectCandidate(
    const CandidateCheck& refitOn,
    const CandidateCheck& refitOff)
{
    if (!refitOn.valid && !refitOff.valid)
    {
        return CandidateSelection::Neither;
    }
    if (refitOn.valid && !refitOff.valid)
    {
        return CandidateSelection::RefitOn;
    }
    if (!refitOn.valid && refitOff.valid)
    {
        return CandidateSelection::RefitOff;
    }

    if (refitOn.sufficientEvidence != refitOff.sufficientEvidence)
    {
        return refitOn.sufficientEvidence
            ? CandidateSelection::RefitOn
            : CandidateSelection::RefitOff;
    }
    if (!refitOn.sufficientEvidence)
    {
        return refitOn.fingerprint == refitOff.fingerprint
            ? CandidateSelection::RefitOn
            : CandidateSelection::Ambiguous;
    }

    const auto clearlyBetter = [](double best, double other)
    {
        return best + 0.02 <= other * 0.70;
    };
    if (clearlyBetter(refitOff.score, refitOn.score))
    {
        return CandidateSelection::RefitOff;
    }
    if (clearlyBetter(refitOn.score, refitOff.score))
    {
        return CandidateSelection::RefitOn;
    }
    return refitOn.fingerprint == refitOff.fingerprint
        ? CandidateSelection::RefitOn
        : CandidateSelection::Ambiguous;
}

AssignedCheck EvaluateAssignedSegments(
    const std::vector<AssignedSegment>& segments,
    double flatSlopeThreshold)
{
    AssignedCheck check;
    const double threshold = std::max(0.01, flatSlopeThreshold);
    std::vector<BandSample> platformSamples;
    std::vector<std::string> interiorKinds;
    std::string previousInteriorKind;
    bool previousCanConnect = false;
    for (const AssignedSegment& segment : segments)
    {
        if (segment.lapStep || segment.endpointSegment)
        {
            previousCanConnect = false;
            continue;
        }
        ++check.checkedSegmentCount;
        const bool platform = IsPlatformKind(segment.kind);
        const bool slope = IsSlopeKind(segment.kind);
        if (!platform && !slope)
        {
            check.failures.push_back(
                "raw_index=" + std::to_string(segment.beginRawIndex) + "->"
                + std::to_string(segment.endRawIndex)
                + ", unknown segment kind=" + segment.kind);
            continue;
        }
        interiorKinds.push_back(segment.kind);
        if (previousCanConnect
            && !IsAllowedTransition(previousInteriorKind, segment.kind))
        {
            check.failures.push_back(
                "invalid segment transition=" + previousInteriorKind
                + "->" + segment.kind);
        }
        previousInteriorKind = segment.kind;
        previousCanConnect = true;
        const double depthDelta = segment.depthEnd - segment.depthBegin;
        const double ratio = segment.lateralSpan > kEpsilon
            ? std::abs(depthDelta) / segment.lateralSpan
            : (std::abs(depthDelta) > kEpsilon
                ? std::numeric_limits<double>::infinity() : 0.0);
        if (platform)
        {
            ++check.platformSegmentCount;
            if (ratio >= threshold)
            {
                check.failures.push_back(
                    "raw_index=" + std::to_string(segment.beginRawIndex) + "->"
                    + std::to_string(segment.endRawIndex)
                    + ", platform kind has slope=" + FormatDouble(ratio, 4));
            }
            platformSamples.push_back({
                (segment.pathBegin + segment.pathEnd) * 0.5,
                (segment.depthBegin + segment.depthEnd) * 0.5,
                segment.kind == "low_platform" ? 0.5 : -0.5
            });
        }
        else
        {
            ++check.slopeSegmentCount;
            if (ratio < threshold)
            {
                check.failures.push_back(
                    "raw_index=" + std::to_string(segment.beginRawIndex) + "->"
                    + std::to_string(segment.endRawIndex)
                    + ", slope kind has slope=" + FormatDouble(ratio, 4));
            }
            if ((segment.kind == "rising_edge" && depthDelta >= -kEpsilon)
                || (segment.kind == "falling_edge" && depthDelta <= kEpsilon))
            {
                check.failures.push_back(
                    "raw_index=" + std::to_string(segment.beginRawIndex) + "->"
                    + std::to_string(segment.endRawIndex)
                    + ", slope direction conflicts with kind=" + segment.kind);
            }
        }
    }

    const BandFit fit = FitTwoBands(platformSamples);
    check.sufficientEvidence = fit.sufficientEvidence;
    check.lowHighSeparation = fit.separation;
    check.residualRms = fit.residualRms;
    check.maxResidual = fit.maxResidual;
    if (fit.sufficientEvidence)
    {
        if (fit.separation < kMinimumBandSeparation)
        {
            check.failures.push_back(
                "low platform is not farther along measurement gun depth, low-high="
                + FormatDouble(fit.separation) + " mm");
        }
        if (fit.residualRms > std::max(0.75, 0.18 * std::abs(fit.separation))
            || fit.maxResidual > std::max(1.5, 0.35 * std::abs(fit.separation)))
        {
            check.failures.push_back(
                "assigned platform band residual is excessive, rms="
                + FormatDouble(fit.residualRms) + " mm, max="
                + FormatDouble(fit.maxResidual) + " mm, low-high="
                + FormatDouble(fit.separation) + " mm");
        }
    }

    std::ostringstream fingerprint;
    for (std::size_t index = 0; index < interiorKinds.size(); ++index)
    {
        if (index > 0)
        {
            fingerprint << '>';
        }
        fingerprint << interiorKinds[index];
    }
    check.fingerprint = fingerprint.str();
    check.valid = check.checkedSegmentCount > 0 && check.failures.empty();
    return check;
}
}
