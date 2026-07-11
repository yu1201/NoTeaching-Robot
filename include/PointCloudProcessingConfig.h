#pragma once

#include <QString>

class PointCloudProcessingConfig
{
public:
    // 精测点云处理方法（四种）。旧值兼容：LegacyLaserPath=特征点+拟合，ExternalCorrugatedSheet=SDK全处理。
    enum class Mode
    {
        LegacyLaserPath = 0,         // ④特征点+拟合：相机目标轨迹点 → 滤波拟合
        ExternalCorrugatedSheet = 1, // ①SDK点云算法全处理：完整点云 → DLL → 拐点直接用
        SdkBaseWeldFit = 2,          // ②SDK点云算法+拟合：完整点云 → DLL 基础焊道 → 滤波拟合
        CloudFit = 3                 // ③点云算法+拟合：完整点云 → 滤波拟合
    };

    enum class FeaturePointStrategy
    {
        LegacyGeometry = 0,
        SlopeWaveFiltered = 1,
        RobustSegmentedKeys = 2,
        WorkpieceProjection = 3
    };

    // 滤波拟合的采样/段类判定主轴：默认按扫描起止点方向自动推断。
    enum class SampleAxisMode
    {
        Auto = 0,
        AxisX = 1,
        AxisY = 2
    };

    enum class ValidationPolicy
    {
        Audit = 0,
        Enforce = 1
    };

    static constexpr int CURRENT_VALIDATION_PROFILE_VERSION = 1;

    struct Settings
    {
        Mode mode = Mode::LegacyLaserPath;
        FeaturePointStrategy featurePointStrategy = FeaturePointStrategy::LegacyGeometry;
        QString libraryDir;
        QString configPath;
        double zTruncationValue = 6.0;
        double resampleStepMm = 2.0;
        // SDK 已焊起点截断：开启后 SDK 两种方法用库返回的已焊起点截掉焊道已焊部分（默认关闭）。
        bool sdkUseWeldedStartTruncation = false;
        // 滤波拟合数值参数（管线真实消费；默认值即原管线硬编码值，保证升级后现场行为不变）。
        SampleAxisMode sampleAxisMode = SampleAxisMode::Auto;
        double cloudZThresholdMm = -230.0;
        double cloudZJumpThresholdMm = 3.0;
        double cloudZContinuityThresholdMm = 2.0;
        double cloudSegmentBreakDistanceMm = 6.0;
        bool cloudKeepLongestSegmentOnly = true;
        double fitSampleStepMm = 2.0;
        double fitSearchWindowMm = 8.0;
        double fitPiecewiseToleranceMm = 4.0;
        int fitPiecewiseMinSegmentPoints = 10;
        int fitMinPointCount = 4;
        int fitSmoothRadius = 3;
        // SDK 基础焊道(仅 SdkBaseWeldFit 模式)拟合前预平滑：对 SDK 重建的稠密焊道点做"结构自适应 1D 双边"
        // 去锯齿——按弧长参数化、值域核以"邻点到局部切线的垂直偏移"加权，使跨搭接X台阶/真实折角的邻点
        // 权重趋零，从而只磨平直线段小幅锯齿、不会把搭接台阶或拐角圆滑掉。默认关闭(现场按需开+标定)。
        bool sdkBasePresmoothEnable = false;
        double sdkBasePresmoothWindowMm = 3.0;  // 空间窗 σs(沿弧长mm)：越大越平滑，须远小于最短直线段
        double sdkBasePresmoothEdgeMm = 0.5;     // 保边阈值 σr(mm)：垂直偏移>此的跳变(台阶/折角)保留；须>锯齿幅度且<台阶高度
        // 方位角拐点检测参数（SDK基础点云+滤波拟合流程固定使用；判方向转折角度定拐点，
        // 替代 DP「离弦最远点」——避免缓变/台阶拐角标偏、漏检真折角）。
        double fitAzimuthTurnThresholdDeg = 22.0;    // 转角阈值(度)：区分真折角与波纹起伏的分界
        int fitAzimuthHeadingWindow = 10;            // 左右两段方向的局部最小二乘拟合跨度(点)
        double fitAzimuthNmsSpanMm = 12.0;           // 非极大值抑制弧长：同区域只保留转角最大的拐点
        double fitAzimuthStraightenResidualMm = 6.0; // 直线化兜底残差：段内偏离弦超此值才补拐点
        // 端区补拐点(两条几何路径通用)：界面"端区补拐点"分组开关 + 4 个可调门限。默认关(已被平台重算取代)。
        bool fitCornerRefineEnable = false;          // 总开关(默认关)
        double fitAzimuthRefineFloorMm = 0.5;        // 端区细化地板(mm)：端区一致弓向离弦峰值超此才补拐点
        double fitCornerRefineOneSidedPct = 80.0;    // 单侧弓出门(%)[50,100]：同侧占比≥此才算真弓(挡噪声)
        double fitCornerRefineMidMultiple = 3.0;     // 中段地板倍数(≥1)：中段地板=端区地板×此
        double fitCornerRefineEndFracPct = 20.0;     // 端区占比(%)[0,50]：首尾各此比例视为端区
        // 拐点结构约束(平台重算)：界面"平台重算"开关+参数。按 II/OO 成对规律每平台重算 2 角，修源头多检/漏检。
        bool fitCornerPatternRefitEnable = true;     // 总开关(默认开)
        int fitCornerPlatformMinSegPoints = 8;       // 三段拟合每段最少点数(≥3)
        // 板材搭接 X 错位台阶检测(默认关，仅几何拟合流程)：双侧平台最小二乘→两侧分别拟合+保留台阶
        bool enableLapMisalignmentSplit = false;
        double lapStepHeightThresholdMm = 1.0;   // 台阶高门(中心两线高度差)
        double lapStepStationWindowMm = 10.0;    // 单侧拟合窗口长度(主轴 mm)
        double lapStepSideFlatnessMm = 0.12;     // 平台残差 rms 上限(排波纹/噪声)
        double lapStepPlatformSlopeMax = 0.10;   // 平台斜率门(排拐角斜边)
        // 基础焊道首尾段截断（②③④拟合方案通用，拟合前按点列扫描顺序截掉开头/结尾指定弧长的坏点）。默认关。
        bool fitEdgeTruncateEnable = false;
        double fitTruncateHeadMm = 0.0;  // 截掉点列开头(首点侧)弧长 mm
        double fitTruncateTailMm = 0.0;  // 截掉点列结尾(末点侧)弧长 mm
        // 端区周期一致性补拐点（②③④拟合方案通用，用波纹周期+典型拐角角度补回起/终点段漏掉的拐点）。默认关。
        bool fitEndPeriodRecoverEnable = false;
        double fitEndPeriodRatioThreshold = 1.2;  // 端段长/周期 ≥ 此值判漏拐点
        double fitEndPeriodMinBendDeg = 5.0;       // 补点候选最小弯折角(度)
        double fitEndPeriodMergeFrac = 0.4;        // 删错:相邻同类拐点间距 < 此×周期 判找错合并
        // 按平台边界重定拐点（②③④拟合方案通用，检测平台、把拐点归位到平台两端、删平台内放错的角）。默认关。
        bool fitPlatformSnapEnable = false;
        double fitPlatformSnapFlatSlope = 0.15;    // 侧向斜率 < 此判为平台(平)，≥此为坡
        double fitPlatformSnapMinFrac = 0.25;      // 平台最小长度 = 此×周期
        // 点云投影提取参数（方法③点云算法+拟合专用；0=按滤波参数自动派生，等同原硬编码行为）。
        double projectionStationWindowMm = 0.0;
        double projectionTransverseWindowMm = 0.0;
        double projectionZBandBelowMm = 0.0;
        double projectionZBandAboveMm = 0.0;
        int projectionMaxCandidatePerSeed = 160;
        double projectionLayerLowPercent = 35.0;
        double projectionLayerHighPercent = 50.0;
        int projectionSmoothRadius = 0;
        bool slopeConsistentCornerFit = false;
        // 调试：导出每段拟合点集与拟合直线为 CloudCompare 点云（默认开启，存数据库不写 ini）。
        bool exportFitDebugCloud = true;
        // 调试：导出完整点云逐帧文件(每点带帧号/相机原始坐标/机器人位姿/时间戳，647MB级)排查散点。
        // 默认关闭——仅排查相机散点时手动勾选，否则每次扫描生成大文件会明显拖慢流程。
        bool exportWorkpieceFrameDebug = false;
        ValidationPolicy validationPolicy = ValidationPolicy::Enforce;
        int validationProfileVersion = CURRENT_VALIDATION_PROFILE_VERSION;
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
    };

    static QString DefaultLibraryDir();
    static QString DefaultConfigPath();
    // 算法参数主文件：Data/CorrugatedSheetPointCloudEctration.ini（现场拥有，DLL 参数更新直接替换）。
    static QString DataConfigPath();
    static Settings Load();
    static bool Save(const Settings& settings, QString* error = nullptr);
    static void SetRuntimeModeOverride(Mode mode);
    static void SetRuntimeScanDirectionOverride(double x, double y, double z);
    static bool RuntimeScanDirectionOverride(double* x, double* y, double* z);
    static void ClearRuntimeOverrides();
    static QString ModeDisplayName(Mode mode);
    static QString ModeConfigValue(Mode mode);
    static Mode ModeFromConfigValue(const QString& value);
    static QString FeaturePointStrategyDisplayName(FeaturePointStrategy strategy);
    static QString FeaturePointStrategyConfigValue(FeaturePointStrategy strategy);
    static FeaturePointStrategy FeaturePointStrategyFromConfigValue(const QString& value);
    static QString ValidationPolicyConfigValue(ValidationPolicy policy);
    static ValidationPolicy ValidationPolicyFromConfigValue(const QString& value);
    static QString SampleAxisModeConfigValue(SampleAxisMode mode);
    static SampleAxisMode SampleAxisModeFromConfigValue(const QString& value);
};
