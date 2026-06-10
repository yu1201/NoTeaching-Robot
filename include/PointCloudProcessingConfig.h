#pragma once

#include <QString>

class PointCloudProcessingConfig
{
public:
    enum class Mode
    {
        LegacyLaserPath = 0,
        ExternalCorrugatedSheet = 1
    };

    enum class FeaturePointStrategy
    {
        LegacyGeometry = 0,
        SlopeWaveFiltered = 1,
        RobustSegmentedKeys = 2,
        WorkpieceProjection = 3
    };

    struct Settings
    {
        Mode mode = Mode::LegacyLaserPath;
        FeaturePointStrategy featurePointStrategy = FeaturePointStrategy::LegacyGeometry;
        QString libraryDir;
        QString configPath;
        double zTruncationValue = 6.0;
        double resampleStepMm = 2.0;
        bool fallbackToLegacy = true;
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
    static void SetRuntimeFallbackToLegacyOverride(bool fallbackToLegacy);
    static void SetRuntimeScanDirectionOverride(double x, double y, double z);
    static bool RuntimeScanDirectionOverride(double* x, double* y, double* z);
    static void ClearRuntimeOverrides();
    static QString ModeDisplayName(Mode mode);
    static QString ModeConfigValue(Mode mode);
    static Mode ModeFromConfigValue(const QString& value);
    static QString FeaturePointStrategyDisplayName(FeaturePointStrategy strategy);
    static QString FeaturePointStrategyConfigValue(FeaturePointStrategy strategy);
    static FeaturePointStrategy FeaturePointStrategyFromConfigValue(const QString& value);
};
