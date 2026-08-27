#include "PlatformSemanticValidator.h"

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

namespace
{
using PlatformSemanticValidator::AssignedSegment;
using PlatformSemanticValidator::CandidateCheck;
using PlatformSemanticValidator::CandidateKeyPoint;
using PlatformSemanticValidator::CandidateSelection;
using PlatformSemanticValidator::CornerType;

int gFailures = 0;

void Expect(bool condition, const std::string& name)
{
    std::cout << (condition ? "PASS " : "FAIL ") << name << '\n';
    if (!condition)
    {
        ++gFailures;
    }
}

std::vector<CandidateKeyPoint> MakeCandidate(double platformNoiseScale = 0.0)
{
    const std::vector<double> paths{0, 20, 35, 55, 70, 90, 105, 125, 140, 160};
    const std::vector<CornerType> types{
        CornerType::Outer, CornerType::Outer,
        CornerType::Inner, CornerType::Inner,
        CornerType::Outer, CornerType::Outer,
        CornerType::Inner, CornerType::Inner,
        CornerType::Outer, CornerType::Outer
    };
    const std::vector<double> noisePattern{
        0.6, 0.6, -0.4, -0.4, -0.7,
        -0.7, 0.5, 0.5, 0.2, 0.2
    };
    std::vector<CandidateKeyPoint> points;
    for (std::size_t index = 0; index < paths.size(); ++index)
    {
        const double band = types[index] == CornerType::Inner ? 30.0 : 0.0;
        const double noise = noisePattern[index] * platformNoiseScale;
        points.push_back({
            static_cast<int>(100 + index * 20),
            paths[index],
            band + 0.02 * paths[index] + noise,
            types[index],
            false
        });
    }
    return points;
}

std::vector<CandidateKeyPoint> MakeShiftedSemanticCandidate()
{
    std::vector<CandidateKeyPoint> points = MakeCandidate();
    for (CandidateKeyPoint& point : points)
    {
        point.type = point.type == CornerType::Inner
            ? CornerType::Outer : CornerType::Inner;
    }
    return points;
}

std::vector<AssignedSegment> MakeAssignedSegments()
{
    return {
        {100, 120, 0, 20, 0.0, 0.2, 20.0, "high_platform", false, false},
        {120, 135, 20, 35, 0.2, 30.35, 15.0, "falling_edge", false, false},
        {135, 155, 35, 55, 30.35, 30.55, 20.0, "low_platform", false, false},
        {155, 170, 55, 70, 30.55, 0.70, 15.0, "rising_edge", false, false},
        {170, 190, 70, 90, 0.70, 0.90, 20.0, "high_platform", false, false},
        {190, 205, 90, 105, 0.90, 31.05, 15.0, "falling_edge", false, false},
        {205, 225, 105, 125, 31.05, 31.25, 20.0, "low_platform", false, false}
    };
}
}

int main()
{
    constexpr double flatSlopeThreshold = 0.10;

    const CandidateCheck clean = PlatformSemanticValidator::EvaluateCandidate(
        MakeCandidate(), flatSlopeThreshold);
    Expect(clean.valid, "generated drifted corrugated candidate passes");
    Expect(clean.sufficientEvidence, "generated candidate has two-band evidence");
    Expect(std::abs(clean.bandSeparation - 30.0) < 0.2,
        "drift fit recovers 30 mm platform separation");

    std::vector<CandidateKeyPoint> singleBadSlope = MakeCandidate();
    singleBadSlope[2].profile = singleBadSlope[1].profile + 0.2;
    const CandidateCheck badSlope = PlatformSemanticValidator::EvaluateCandidate(
        singleBadSlope, flatSlopeThreshold);
    Expect(!badSlope.valid, "single abnormal slope is rejected");

    const CandidateCheck noisyOn = PlatformSemanticValidator::EvaluateCandidate(
        MakeCandidate(2.0), flatSlopeThreshold);
    const CandidateCheck cleanOff = PlatformSemanticValidator::EvaluateCandidate(
        MakeCandidate(), flatSlopeThreshold);
    std::cout << "INFO candidate_scores on=" << noisyOn.score
              << " off=" << cleanOff.score << '\n';
    Expect(noisyOn.valid && cleanOff.valid,
        "both synthetic ON and OFF candidates can be locally valid");
    Expect(PlatformSemanticValidator::SelectCandidate(noisyOn, cleanOff)
            == CandidateSelection::RefitOff,
        "clear semantic score selects cleaner OFF candidate");

    const CandidateCheck shifted = PlatformSemanticValidator::EvaluateCandidate(
        MakeShiftedSemanticCandidate(), flatSlopeThreshold);
    Expect(shifted.valid, "opposite label candidate is independently fit-valid");
    Expect(PlatformSemanticValidator::SelectCandidate(clean, shifted)
            == CandidateSelection::Ambiguous,
        "equal-score conflicting semantics fail as ambiguous");
    Expect(PlatformSemanticValidator::SelectCandidate(badSlope, badSlope)
            == CandidateSelection::Neither,
        "two invalid candidates fail closed");

    const auto assignedGood = PlatformSemanticValidator::EvaluateAssignedSegments(
        MakeAssignedSegments(), flatSlopeThreshold);
    Expect(assignedGood.valid, "assigned high/falling/low/rising sequence passes");
    Expect(assignedGood.sufficientEvidence,
        "assigned sequence has repeated high and low platform evidence");
    Expect(std::abs(assignedGood.lowHighSeparation - 30.0) < 0.2,
        "assigned depth model recovers low-farther relationship");

    std::vector<AssignedSegment> wrongDirection = MakeAssignedSegments();
    wrongDirection[3].kind = "falling_edge";
    const auto assignedWrongDirection =
        PlatformSemanticValidator::EvaluateAssignedSegments(
            wrongDirection, flatSlopeThreshold);
    Expect(!assignedWrongDirection.valid,
        "wrong rising/falling attribute is rejected before compensation");

    std::vector<AssignedSegment> missingPlatform = MakeAssignedSegments();
    missingPlatform.erase(missingPlatform.begin() + 2);
    const auto assignedMissingPlatform =
        PlatformSemanticValidator::EvaluateAssignedSegments(
            missingPlatform, flatSlopeThreshold);
    Expect(!assignedMissingPlatform.valid,
        "falling-to-rising sequence without a platform is rejected");

    std::vector<AssignedSegment> swappedPlatforms = MakeAssignedSegments();
    for (AssignedSegment& segment : swappedPlatforms)
    {
        if (segment.kind == "low_platform")
        {
            segment.kind = "high_platform";
        }
        else if (segment.kind == "high_platform")
        {
            segment.kind = "low_platform";
        }
    }
    const auto assignedSwapped = PlatformSemanticValidator::EvaluateAssignedSegments(
        swappedPlatforms, flatSlopeThreshold);
    Expect(!assignedSwapped.valid,
        "globally swapped high/low attributes are rejected");

    std::cout << "SUMMARY failures=" << gFailures << '\n';
    return gFailures == 0 ? 0 : 1;
}
