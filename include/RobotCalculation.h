#pragma once

#include "Const.h"
#include "HandEyeMatrixConfig.h"

#include <QString>
#include <QVector>

#include <Eigen/Dense>

#include <vector>

class RobotCalculation
{
public:
    struct TimestampedRobotPose
    {
        qint64 timestampUs = 0;
        T_ROBOT_COORS pose;
    };

    struct IndexedPoint3D
    {
        int index = 0;
        Eigen::Vector3d point = Eigen::Vector3d::Zero();
    };

    enum class SampleAxis
    {
        AxisY = 0,
        AxisX = 1
    };

    enum class LowerWeldGeometryStrategy
    {
        LegacyGeometry = 0,
        SlopeWaveFiltered = 1,
        RobustSegmentedKeys = 2,
        WorkpieceProjection = 3
    };

    struct LowerWeldFilterParams
    {
        struct CornerCompensation
        {
            double innerToOuterMm = 0.0;
            double innerToInnerMm = 0.0;
            double outerToOuterMm = 0.0;
            double outerToInnerMm = 0.0;
        };

        SampleAxis sampleAxis = SampleAxis::AxisY;
        LowerWeldGeometryStrategy geometryStrategy = LowerWeldGeometryStrategy::LegacyGeometry;
        double zThreshold = -230.0;
        double zJumpThreshold = 5.0;
        double zContinuityThreshold = 3.0;
        double segmentBreakDistance = 12.0;
        bool keepLongestSegmentOnly = true;
        double sampleStep = 2.0;
        double searchWindow = 8.0;
        double piecewiseFitTolerance = 2.0;
        int piecewiseMinSegmentPoints = 4;
        int minPointCount = 3;
        int smoothRadius = 2;
        // 输入已去噪标志：SDK 的 is_remove_noise 已开启(基础焊道已是干净稠密点云)时由调用方置 true。
        // 几何流程随之：①跳过自身的 MAD 去噪/分支去噪与投影平滑(重复处理会削圆尖角、移位拐点)；
        // ②拐点检测改用方位角法(下列参数)——干净输入上 DP「离弦最远点」会把缓变/台阶拐点标偏、
        // 漏检真折角，方位角法判方向转折角度，天然区分真折角与波纹起伏。
        bool inputAlreadyDenoised = false;
        int azimuthHeadingWindow = 10;           // K：左右两段方向的局部最小二乘拟合跨度（点）
        double azimuthTurnThresholdDeg = 22.0;   // 候选转角阈值（度）：滤掉波纹板侧向小起伏
        double azimuthNmsSpanMm = 12.0;          // 非极大值抑制：同区域合并的主轴弧长
        double azimuthStraightenResidualMm = 6.0;// 段内偏离弦超此值才补拐点（漏检兜底）。须大于
                                                 // 波纹起伏幅度，否则把周期起伏误当漏检拐点过补
        // 端区补拐点(起终点先验自适应细化，RefineCornersByCoherentBow 用)。默认关——已被下方"平台重算"取代
        // (补拐点按离弦几何加点、易在端区过补；平台重算按 II/OO 规律重算更稳)。需要纯几何补点时再开。
        bool cornerRefineEnable = false;         // 总开关：默认关(平台重算取代)；开则在拐点检测后做端区补拐点
        double azimuthRefineFloorMm = 0.5;       // 端区细化地板(mm)：端区段内一致弓向离弦峰值超此值才补拐点
        double cornerRefineOneSidedFrac = 0.80;  // 单侧弓出占比门[0.5,1]：段内点同侧占比≥此才算真弓(挡随机噪声)
        double cornerRefineMidMultiple = 3.0;    // 中段地板倍数(≥1)：中段地板=端区地板×此，体现漏点多在端区的先验
        double cornerRefineEndFrac = 0.20;       // 端区弧长占比[0,0.5]：首尾各此比例视为端区(用低地板)
        // 拐点 inner/outer 结构约束(平台重算，RefitCornersByPlatformPattern 用)：按 II/OO 成对规律把拐点归平台，
        // 每平台从稠密路径三段拟合重算 2 个边界角，对源头多检(5→2)/漏检(1→2)免疫。搭接台阶点豁免。默认开。
        bool cornerPatternRefitEnable = true;    // 总开关：默认开
        int cornerPlatformMinSegPoints = 8;      // 三段拟合每段最少点数(≥3)：越大越稳但短平台可能拟合不出
        // 板材搭接 X 错位台阶检测（默认关，仅几何拟合流程）：双侧平台最小二乘判据——候选两侧各取窗口
        // 对侧向 h 做直线拟合，要求两侧斜率近 0（平台，区别于拐角斜边）+ 残差小（排波纹噪声）+ 中心两线
        // 高度差>阈值（台阶高）→ 错位点作硬段边界，两侧点云分别拟合，错位处保留 X 台阶（不做过渡）。
        bool enableLapMisalignmentSplit = false;
        double lapStepHeightThresholdMm = 1.0;   // 台阶高门：中心处两侧拟合线高度差小于此不算错位
        double lapStepStationWindowMm = 10.0;    // 单侧拟合窗口长度(主轴 mm)：越长平台斜率/残差估计越稳
        double lapStepSideFlatnessMm = 0.12;     // 平台残差 rms 上限：两侧拟合残差超此=波纹/噪声，非平台
        double lapStepPlatformSlopeMax = 0.10;   // 平台斜率门(mm侧向/mm主轴)：两侧斜率超此=拐角斜边，非平台
        // 点云投影提取（完整点云→下层焊道轨迹）参数；0 表示自动按滤波参数派生（原硬编码行为）。
        double projectionStationWindowMm = 0.0;     // 站位窗口：0=max(2.5, 采样步长*1.5)
        double projectionTransverseWindowMm = 0.0;  // 横向窗口：0=max(10, 搜索窗口*1.5)
        double projectionZBandBelowMm = 0.0;        // 种子下方Z带：0=max(14, Z连续阈值*4)
        double projectionZBandAboveMm = 0.0;        // 种子上方Z带：0=max(12, Z突变阈值*2.5)
        int projectionMaxCandidatePerSeed = 160;    // 每种子底板候选上限
        double projectionLayerLowPercent = 35.0;    // 底板层位Z分位下界（%）
        double projectionLayerHighPercent = 50.0;   // 底板层位Z分位上界（%）
        int projectionSmoothRadius = 0;             // 投影轮廓平滑半径：0=按滤波平滑半径夹到[1,4]
        bool useSlopeConsistentCornerFit = false;
        // 调试：把每段拟合用到的点集与拟合直线导出成 CloudCompare 友好的点云，
        // 用来核对分段直线拟合是否正确。fitDebugDir 为空时不导出。
        bool exportFitDebugCloud = false;
        QString fitDebugDir;
        bool enableCornerCompensation = false;
        bool validationCoverageEnabled = true;
        int validationMinFinitePointCount = 300;
        double validationMinProjectedSpanMm = 180.0;
        bool validationContinuityEnabled = true;
        double validationMinStationCoverageRatio = 0.55;
        double validationMinLongestContinuousRatio = 0.60;
        bool validationDenoiseRatioEnabled = true;
        double validationMaxRejectedRatio = 0.40;
        bool validationResidualEnabled = true;
        double validationMaxMedianResidualMm = 3.0;
        double validationMaxP95ResidualMm = 8.0;
        double validationResidualInlierThresholdMm = 6.0;
        double validationMinResidualInlierRatio = 0.75;
        bool validationKeyPointEnabled = true;
        int validationMinKeyPointCount = 6;
        int validationMinCornerCount = 4;
        double validationMinSegmentLengthMm = 15.0;
        bool validationOutputEnabled = true;
        int validationMinOutputPointCount = 80;
        double validationMinOutputLengthRatio = 0.70;
        CornerCompensation risingCornerCompensation;
        CornerCompensation fallingCornerCompensation;
    };

    struct LowerWeldFilterPoint
    {
        int index = 0;
        Eigen::Vector3d point = Eigen::Vector3d::Zero();
        QString source;
    };

    struct LowerWeldFilterResult
    {
        bool ok = false;
        QString error;
        QVector<LowerWeldFilterPoint> points;
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

    enum class LowerWeldPointType
    {
        Start = 1,
        End = 2,
        InnerCorner = 3,
        OuterCorner = 4,
        Normal = 5,
        Noise = 6
    };

    struct LowerWeldClassifiedPoint
    {
        int index = 0;
        Eigen::Vector3d point = Eigen::Vector3d::Zero();
        LowerWeldPointType type = LowerWeldPointType::Normal;
        QString source;
        QString segmentKindAfter;
        bool isLapStepBoundary = false;  // 搭接错位台阶的两个端点（不参与求交、不当斜边/拐角补偿）
    };

    struct LowerWeldClassificationResult
    {
        bool ok = false;
        QString error;
        QVector<LowerWeldClassifiedPoint> points;
        int startCount = 0;
        int endCount = 0;
        int innerCornerCount = 0;
        int outerCornerCount = 0;
        int normalCount = 0;
        int noiseCount = 0;
    };

    struct MeasureThenWeldAnalysisResult
    {
        bool ok = false;
        QString error;
        LowerWeldFilterResult filterResult;
        LowerWeldClassificationResult classificationResult;
        QVector<LowerWeldClassifiedPoint> keyPoints;
        LowerWeldClassificationResult cornerCompensatedClassificationResult;
        QVector<LowerWeldClassifiedPoint> cornerCompensatedKeyPoints;
    };

    static T_ROBOT_COORS InterpolateRobotPose(const std::vector<TimestampedRobotPose>& robotSamples, qint64 targetTimestampUs);
    static Eigen::Vector3d CalcLaserPointInRobot(const T_ROBOT_COORS& robotPose, const Eigen::Vector3d& cameraPoint, const HandEyeMatrixConfig& calibration);
    static LowerWeldFilterResult ProjectWorkpieceCloudToLowerWeldPath(
        const QVector<IndexedPoint3D>& workpieceCloudInput,
        const QVector<IndexedPoint3D>& seedPathInput,
        const LowerWeldFilterParams& params);
    static MeasureThenWeldAnalysisResult AnalyzeMeasureThenWeldLowerWeldPathDirect(
        const QVector<IndexedPoint3D>& inputPoints,
        const LowerWeldFilterParams& params);
    static MeasureThenWeldAnalysisResult AnalyzeMeasureThenWeldLowerWeldPathGeometry(
        const QVector<IndexedPoint3D>& inputPoints,
        const LowerWeldFilterParams& params);
    static LowerWeldClassificationResult BuildCornerCompensatedLowerWeldClassification(
        const QVector<LowerWeldClassifiedPoint>& keyPoints,
        const LowerWeldFilterParams& params,
        QVector<LowerWeldClassifiedPoint>* compensatedKeyPoints = nullptr);
    static int LowerWeldPointTypeCode(LowerWeldPointType type);
    static QString LowerWeldPointTypeName(LowerWeldPointType type);
    static QString RobotPoseCsv(qint64 timestampUs, const T_ROBOT_COORS& pose);
    static QString Vector3Csv(qint64 timestampUs, const Eigen::Vector3d& point, const QString& extra = QString());
    static QString RobotPoseIndexedCsv(int index, const T_ROBOT_COORS& pose);
    static QString Vector3IndexedCsv(int index, const Eigen::Vector3d& point, const QString& extra = QString());
    static QString Vector3IndexedSpaceText(int index, const Eigen::Vector3d& point, const QString& extra = QString());
};
