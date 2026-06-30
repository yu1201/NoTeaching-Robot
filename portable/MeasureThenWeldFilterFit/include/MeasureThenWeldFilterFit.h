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

    // ============================================================================
    // 以下为「波纹板先测后焊」当前在用的几何拐点管线参数，字段名与主程序
    // RobotCalculation::LowerWeldFilterParams 一一对应（见 CHANGELOG）。
    // ============================================================================

    // 输入已去噪标志：SDK 的 is_remove_noise 已开启(基础焊道已是干净稠密点云)时置 true。
    // 置 true 时几何流程：①跳过自身 MAD 去噪/分支去噪与投影平滑(重复处理会削圆尖角、移位拐点)；
    // ②拐点检测改用【方位角法】(下列 azimuth* 参数)而非 Douglas-Peucker。
    bool inputAlreadyDenoised = false;

    // 方位角拐点检测（inputAlreadyDenoised 路径）：左右各拟合局部方向，判方向转折角。
    int azimuthHeadingWindow = 10;            // K：左右两段方向的局部最小二乘拟合跨度（点）
    double azimuthTurnThresholdDeg = 22.0;    // 候选转角阈值（度）：滤掉波纹板侧向小起伏
    double azimuthNmsSpanMm = 12.0;           // 非极大值抑制：同区域合并的主轴弧长
    double azimuthStraightenResidualMm = 6.0; // 直线化兜底：段内偏离弦超此值才补漏检拐点

    // 端区补拐点（相干弓，RefineCornersByCoherentBow）。默认关——已被「平台重算」取代。
    bool cornerRefineEnable = false;
    double azimuthRefineFloorMm = 0.5;        // 端区细化地板(mm)：一致弓向离弦峰值超此才补
    double cornerRefineOneSidedFrac = 0.80;   // 单侧弓出占比门[0.5,1]：挡随机噪声
    double cornerRefineMidMultiple = 3.0;     // 中段地板倍数(≥1)：漏点多在端区的先验
    double cornerRefineEndFrac = 0.20;        // 端区弧长占比[0,0.5]：首尾各此比例用低地板

    // 平台重算（II/OO 结构约束，RefitCornersByPlatformPattern）：每平台从稠密路径重算 2 个边界角。
    bool cornerPatternRefitEnable = true;     // 默认开
    int cornerPlatformMinSegPoints = 8;       // 三段拟合每段最少点数(≥3)

    // 板材搭接 X 错位台阶检测（DetectLapMisalignmentBoundaries）。默认关。
    bool enableLapMisalignmentSplit = false;
    double lapStepHeightThresholdMm = 1.0;    // 台阶高门：中心处两侧拟合线高度差
    double lapStepStationWindowMm = 10.0;     // 单侧拟合窗口长度(主轴 mm)
    double lapStepSideFlatnessMm = 0.12;      // 平台残差 rms 上限(排波纹/噪声)
    double lapStepPlatformSlopeMax = 0.10;    // 平台斜率门(排拐角斜边)

    // 基础焊道首尾段截断（TrimLowerWeldPathByArcLength，拟合提取关键点之前执行）。默认关。
    bool enableEdgeTruncate = false;
    double truncateHeadMm = 0.0;              // 截掉点列开头(首点侧)弧长 mm
    double truncateTailMm = 0.0;              // 截掉点列结尾(末点侧)弧长 mm

    // 端区周期一致性补拐点 / 删错（RecoverEndRegionCornersByPeriod + MergeTooCloseSameTypeCorners）。默认关。
    bool enableEndPeriodCornerRecover = false;
    double endPeriodRatioThreshold = 1.2;     // 端段长/周期L ≥ 此值才判漏拐点
    double endPeriodMinBendDeg = 5.0;         // 补点候选最小弯折角(度)
    double endPeriodMergeFrac = 0.4;          // 删错:相邻同类拐点间距 < 此×周期L 判找错合并

    // 按平台边界重定拐点（SnapCornersToPlatforms）。默认关。
    bool enablePlatformCornerSnap = false;
    double platformSnapFlatSlope = 0.15;      // 侧向局部斜率 < 此判为平台，≥此为坡
    double platformSnapMinFrac = 0.25;        // 平台最小长度 = 此×周期L

    // When true, corner intersections are fitted from the straight core of each
    // neighbouring segment. This mirrors the main program's "直线拟合排除圆弧段"
    // option and helps avoid round/transition samples pulling a straight segment
    // intersection away from the visible line.
    bool useSlopeConsistentCornerFit = false;

    // 调试：把每段拟合“用到的点集”和“拟合出的直线”导出成 CloudCompare 友好的点云，
    // 用来核对分段直线拟合是否正确。仅当 exportFitDebugCloud 为真且 fitDebugDir 非空时导出，
    // 文件写到 fitDebugDir 目录（含 segments 子目录）。
    bool exportFitDebugCloud = false;
    std::string fitDebugDir;
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

// SDK 基础焊道(稠密有序点)拟合前的可选去锯齿预处理：「结构自适应 1D 双边」预平滑。
// 沿弧长参数化，每点用 空间核(σs=windowMm) × 值域核 加权邻点平均，值域距离取「邻点到本点局部切线的
// 垂直偏移」——直线段内垂直偏移≈0→磨掉锯齿；搭接 X 台阶/真实折角处垂直偏移大(>σr=edgeMm)→权重趋零→
// 不跨过去平均，从机制上保住台阶与折角(不会把搭接段圆滑进去)。首末点不动。原地修改 points，返回被移动点数。
// 对应主程序 MeasureThenWeldService::BilateralPresmoothSdkBaseWeld，仅 SdkBaseWeldFit 模式拟合前调用。
int BilateralPresmoothSdkBaseWeld(
    std::vector<IndexedPoint3D>& points,
    double windowMm = 3.0,
    double edgeMm = 0.5);

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
