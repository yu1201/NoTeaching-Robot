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
        int fitLineFitTrimCount = 0;
        double fitPiecewiseToleranceMm = 4.0;
        int fitPiecewiseMinSegmentPoints = 10;
        int fitMinPointCount = 4;
        int fitSmoothRadius = 3;
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
    static QString SampleAxisModeConfigValue(SampleAxisMode mode);
    static SampleAxisMode SampleAxisModeFromConfigValue(const QString& value);
};
