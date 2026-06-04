#pragma once

#include <string>
#include <vector>

namespace mtw_filter_fit
{
struct Point3D
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct IndexedPoint3D
{
    int index = 0;
    Point3D point;
};

enum class SampleAxis
{
    AxisY = 0,
    AxisX = 1
};

enum class FitMode
{
    PreservePath = 0,
    LineFit = 1,
    TrapezoidFit = 2,
    PiecewiseLineFit = 3
};

enum class GeometryStrategy
{
    LegacyGeometry = 0,
    SlopeWaveFiltered = 1,
    RobustSegmentedKeys = 2,
    WorkpieceProjection = 3
};

enum class WeldPointType
{
    Start = 1,
    End = 2,
    InnerCorner = 3,
    OuterCorner = 4,
    Normal = 5,
    Noise = 6
};

enum class TrackPointType
{
    Normal = 0,
    Start = 1,
    End = 2,
    Corner = 3
};

struct FilterFitParams
{
    SampleAxis sampleAxis = SampleAxis::AxisY;
    FitMode fitMode = FitMode::PreservePath;
    GeometryStrategy geometryStrategy = GeometryStrategy::LegacyGeometry;

    // Current measure-then-weld defaults from MeasureThenWeldService::BuildOriginalTrackFitParams.
    double zThreshold = -230.0;
    double zJumpThreshold = 3.0;
    double zContinuityThreshold = 2.0;
    double segmentBreakDistance = 6.0;
    bool keepLongestSegmentOnly = true;
    double sampleStep = 2.0;
    double searchWindow = 8.0;
    int lineFitTrimCount = 0;
    double piecewiseFitTolerance = 4.0;
    int piecewiseMinSegmentPoints = 10;
    int minPointCount = 4;
    int smoothRadius = 3;
};

struct FilterPoint
{
    int index = 0;
    Point3D point;
    std::string source;
};

struct FilterResult
{
    bool ok = false;
    std::string error;
    std::vector<FilterPoint> points;

    int inputPointCount = 0;
    int lowerPointCount = 0;
    int zJumpRejectedCount = 0;
    int zContinuityRejectedCount = 0;
    int segmentRejectedCount = 0;
    int fitSegmentCount = 0;
    int measuredCount = 0;
    int interpolatedCount = 0;
    int extendedCount = 0;
};

struct ClassifiedPoint
{
    int index = 0;
    Point3D point;
    WeldPointType type = WeldPointType::Normal;
    std::string source;
    std::string segmentKindAfter;
};

struct ClassificationResult
{
    bool ok = false;
    std::string error;
    std::vector<ClassifiedPoint> points;

    int startCount = 0;
    int endCount = 0;
    int innerCornerCount = 0;
    int outerCornerCount = 0;
    int normalCount = 0;
    int noiseCount = 0;
};

struct AnalysisResult
{
    bool ok = false;
    std::string error;
    FilterResult filterResult;
    ClassificationResult classificationResult;
    std::vector<ClassifiedPoint> keyPoints;
};

struct ExternalTrackPoint
{
    int index = 0;
    Point3D point;
    TrackPointType type = TrackPointType::Normal;
};

FilterFitParams MeasureThenWeldDefaultParams(
    SampleAxis sampleAxis = SampleAxis::AxisY,
    GeometryStrategy geometryStrategy = GeometryStrategy::LegacyGeometry);

// Median-resample based lower-weld filter used by the older path and by the filter test dialog.
FilterResult FilterLowerWeldPath(
    const std::vector<IndexedPoint3D>& inputPoints,
    const FilterFitParams& params = MeasureThenWeldDefaultParams());

// Latest direct measure-then-weld analysis path: denoise, fit key/corner points, expand to 2 mm path.
AnalysisResult AnalyzeMeasureThenWeldPath(
    const std::vector<IndexedPoint3D>& inputPoints,
    const FilterFitParams& params = MeasureThenWeldDefaultParams());

// "立板投影到底板" pre-step. Feed the returned points into AnalyzeMeasureThenWeldPath.
FilterResult ProjectWorkpieceCloudToLowerWeldPath(
    const std::vector<IndexedPoint3D>& workpieceCloudInput,
    const std::vector<IndexedPoint3D>& seedPathInput,
    const FilterFitParams& params = MeasureThenWeldDefaultParams());

// Use this when an external point-cloud DLL has already returned start/end/corner track points.
std::vector<ExternalTrackPoint> ResampleTrackPoints(
    const std::vector<ExternalTrackPoint>& rawTrackPoints,
    double stepMm = 2.0);

AnalysisResult BuildAnalysisFromExternalTrack(
    const std::vector<ExternalTrackPoint>& trackPoints,
    int originalInputPointCount,
    const FilterFitParams& params = MeasureThenWeldDefaultParams());

int WeldPointTypeCode(WeldPointType type);
const char* WeldPointTypeName(WeldPointType type);
int TrackPointTypeCode(TrackPointType type);
const char* TrackPointTypeName(TrackPointType type);

std::vector<std::string> BuildFilterOutputLines(const FilterResult& result);
std::vector<std::string> BuildClassifiedOutputLines(const ClassificationResult& result);
std::vector<std::string> BuildKeyPointOutputLines(const std::vector<ClassifiedPoint>& keyPoints);
std::vector<std::string> BuildNoiseOutputLines(
    const std::vector<IndexedPoint3D>& inputPoints,
    const FilterResult& fitResult);
std::vector<std::string> BuildExternalTrackOutputLines(
    const std::vector<ExternalTrackPoint>& points,
    const std::string& source);
}
