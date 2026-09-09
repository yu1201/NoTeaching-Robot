#pragma once

#include <string>
#include <vector>

namespace PlatformSemanticValidator
{
enum class CornerType
{
    Other,
    Inner,
    Outer
};

struct CandidateKeyPoint
{
    int rawIndex = -1;
    double path = 0.0;
    double profile = 0.0;
    CornerType type = CornerType::Other;
    bool lapBoundary = false;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct CandidateCheck
{
    bool valid = false;
    bool sufficientEvidence = false;
    int checkedSegmentCount = 0;
    int platformSegmentCount = 0;
    int slopeSegmentCount = 0;
    double bandSeparation = 0.0;
    double residualRms = 0.0;
    double maxResidual = 0.0;
    double score = 0.0;
    int bandDirection = 0;
    std::string fingerprint;
    std::vector<std::string> failures;
    std::vector<std::string> diagnostics;
};

CandidateCheck EvaluateCandidate(
    const std::vector<CandidateKeyPoint>& keyPoints,
    double flatSlopeThreshold);

enum class CandidateSelection
{
    RefitOn,
    RefitOff,
    Ambiguous,
    Neither
};

CandidateSelection SelectCandidate(
    const CandidateCheck& refitOn,
    const CandidateCheck& refitOff);

struct AssignedSegment
{
    int beginRawIndex = -1;
    int endRawIndex = -1;
    double pathBegin = 0.0;
    double pathEnd = 0.0;
    double depthBegin = 0.0;
    double depthEnd = 0.0;
    double lateralSpan = 0.0;
    std::string kind;
    bool lapStep = false;
    bool endpointSegment = false;
};

struct AssignedCheck
{
    bool valid = false;
    bool sufficientEvidence = false;
    int checkedSegmentCount = 0;
    int platformSegmentCount = 0;
    int slopeSegmentCount = 0;
    double lowHighSeparation = 0.0;
    double residualRms = 0.0;
    double maxResidual = 0.0;
    std::string fingerprint;
    std::vector<std::string> failures;
};

AssignedCheck EvaluateAssignedSegments(
    const std::vector<AssignedSegment>& segments,
    double flatSlopeThreshold);
}
