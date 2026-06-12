#include "PointCloudProcessingConfig.h"

#include "ConfigDatabase.h"
#include "RobotDataHelper.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>

#include <algorithm>

namespace
{
constexpr auto SETTINGS_GROUP = "PointCloudProcessing";
bool g_hasRuntimeModeOverride = false;
PointCloudProcessingConfig::Mode g_runtimeModeOverride =
    PointCloudProcessingConfig::Mode::LegacyLaserPath;
bool g_hasRuntimeScanDirectionOverride = false;
double g_runtimeScanDirectionX = 1.0;
double g_runtimeScanDirectionY = 0.0;
double g_runtimeScanDirectionZ = 0.0;

QString ReadSetting(const QString& key, const QString& defaultValue = QString())
{
    QString value;
    if (ConfigDatabase::ReadScopedSetting(QStringLiteral("global"), QString(), SETTINGS_GROUP, key, &value))
    {
        return value;
    }
    return defaultValue;
}

bool WriteSetting(const QString& key, const QString& value, QString* error)
{
    if (ConfigDatabase::WriteScopedSetting(QStringLiteral("global"), QString(), SETTINGS_GROUP, key, value))
    {
        return true;
    }

    if (error != nullptr)
    {
        *error = QString("写入点云处理配置失败：%1").arg(key);
    }
    return false;
}

bool ReadBoolSetting(const QString& key, bool defaultValue)
{
    const QString value = ReadSetting(key, defaultValue ? "1" : "0").trimmed().toLower();
    if (value.isEmpty())
    {
        return defaultValue;
    }
    return value == "1" || value == "true" || value == "yes";
}

int ReadIntSetting(const QString& key, int defaultValue)
{
    bool ok = false;
    const int value = ReadSetting(key, QString::number(defaultValue)).trimmed().toInt(&ok);
    return ok ? value : defaultValue;
}

double ReadDoubleSetting(const QString& key, double defaultValue)
{
    bool ok = false;
    const double value = ReadSetting(key, QString::number(defaultValue, 'f', 6)).trimmed().toDouble(&ok);
    return ok ? value : defaultValue;
}

QString ExistingProjectFilePath(const QString& relativePath)
{
    const QString path = RobotDataHelper::FindProjectFilePath(relativePath);
    if (!path.isEmpty())
    {
        return path;
    }
    return RobotDataHelper::BuildProjectPath(relativePath);
}
}

QString PointCloudProcessingConfig::DefaultLibraryDir()
{
    const QString dllPath = ExistingProjectFilePath("SDK/PointCloudExtration/PointCloudExtration.dll");
    return QFileInfo(dllPath).absolutePath();
}

QString PointCloudProcessingConfig::DefaultConfigPath()
{
    return ExistingProjectFilePath("SDK/PointCloudExtration/config/CorrugatedSheetPointCloudEctration.ini");
}

QString PointCloudProcessingConfig::DataConfigPath()
{
    return RobotDataHelper::BuildProjectPath(QStringLiteral("Data/CorrugatedSheetPointCloudEctration.ini"));
}

PointCloudProcessingConfig::Settings PointCloudProcessingConfig::Load()
{
    Settings settings;
    settings.mode = ModeFromConfigValue(ReadSetting("General/ProcessingMode", ModeConfigValue(settings.mode)));
    settings.featurePointStrategy = FeaturePointStrategyFromConfigValue(
        ReadSetting("FeaturePoint/Strategy", FeaturePointStrategyConfigValue(settings.featurePointStrategy)));
    settings.libraryDir = ReadSetting("External/LibraryDir", DefaultLibraryDir()).trimmed();
    settings.configPath = ReadSetting("External/ConfigPath", DefaultConfigPath()).trimmed();
    settings.zTruncationValue = ReadSetting("External/ZTruncationValue", "6.0").toDouble();
    settings.resampleStepMm = ReadSetting("External/ResampleStepMm", "2.0").toDouble();
    settings.sdkUseWeldedStartTruncation = ReadBoolSetting("External/UseWeldedStartTruncation", false);
    settings.sampleAxisMode = SampleAxisModeFromConfigValue(
        ReadSetting("Fit/SampleAxis", SampleAxisModeConfigValue(settings.sampleAxisMode)));
    settings.cloudZThresholdMm = ReadDoubleSetting("CloudAlgo/ZThresholdMm", settings.cloudZThresholdMm);
    settings.cloudZJumpThresholdMm = ReadDoubleSetting("CloudAlgo/ZJumpThresholdMm", settings.cloudZJumpThresholdMm);
    settings.cloudZContinuityThresholdMm = ReadDoubleSetting("CloudAlgo/ZContinuityThresholdMm", settings.cloudZContinuityThresholdMm);
    settings.cloudSegmentBreakDistanceMm = ReadDoubleSetting("CloudAlgo/SegmentBreakDistanceMm", settings.cloudSegmentBreakDistanceMm);
    settings.cloudKeepLongestSegmentOnly = ReadBoolSetting("CloudAlgo/KeepLongestSegmentOnly", settings.cloudKeepLongestSegmentOnly);
    settings.fitSampleStepMm = ReadDoubleSetting("Fit/SampleStepMm", settings.fitSampleStepMm);
    settings.fitSearchWindowMm = ReadDoubleSetting("Fit/SearchWindowMm", settings.fitSearchWindowMm);
    settings.fitPiecewiseToleranceMm = ReadDoubleSetting("Fit/PiecewiseToleranceMm", settings.fitPiecewiseToleranceMm);
    settings.fitPiecewiseMinSegmentPoints = ReadIntSetting("Fit/PiecewiseMinSegmentPoints", settings.fitPiecewiseMinSegmentPoints);
    settings.fitMinPointCount = ReadIntSetting("Fit/MinPointCount", settings.fitMinPointCount);
    settings.fitSmoothRadius = ReadIntSetting("Fit/SmoothRadius", settings.fitSmoothRadius);
    settings.projectionStationWindowMm = ReadDoubleSetting("CloudProjection/StationWindowMm", settings.projectionStationWindowMm);
    settings.projectionTransverseWindowMm = ReadDoubleSetting("CloudProjection/TransverseWindowMm", settings.projectionTransverseWindowMm);
    settings.projectionZBandBelowMm = ReadDoubleSetting("CloudProjection/ZBandBelowMm", settings.projectionZBandBelowMm);
    settings.projectionZBandAboveMm = ReadDoubleSetting("CloudProjection/ZBandAboveMm", settings.projectionZBandAboveMm);
    settings.projectionMaxCandidatePerSeed = ReadIntSetting("CloudProjection/MaxCandidatePerSeed", settings.projectionMaxCandidatePerSeed);
    settings.projectionLayerLowPercent = ReadDoubleSetting("CloudProjection/LayerLowPercent", settings.projectionLayerLowPercent);
    settings.projectionLayerHighPercent = ReadDoubleSetting("CloudProjection/LayerHighPercent", settings.projectionLayerHighPercent);
    settings.projectionSmoothRadius = ReadIntSetting("CloudProjection/SmoothRadius", settings.projectionSmoothRadius);
    settings.slopeConsistentCornerFit = ReadBoolSetting("FeaturePoint/SlopeConsistentCornerFit", false);
    settings.exportFitDebugCloud = ReadBoolSetting("FeaturePoint/ExportFitDebugCloud", true);
    settings.validationCoverageEnabled = ReadBoolSetting("Validation/CoverageEnabled", settings.validationCoverageEnabled);
    settings.validationMinFinitePointCount = ReadIntSetting("Validation/MinFinitePointCount", settings.validationMinFinitePointCount);
    settings.validationMinProjectedSpanMm = ReadDoubleSetting("Validation/MinProjectedSpanMm", settings.validationMinProjectedSpanMm);
    settings.validationContinuityEnabled = ReadBoolSetting("Validation/ContinuityEnabled", settings.validationContinuityEnabled);
    settings.validationMinStationCoverageRatio = ReadDoubleSetting("Validation/MinStationCoverageRatio", settings.validationMinStationCoverageRatio);
    settings.validationMinLongestContinuousRatio = ReadDoubleSetting("Validation/MinLongestContinuousRatio", settings.validationMinLongestContinuousRatio);
    settings.validationDenoiseRatioEnabled = ReadBoolSetting("Validation/DenoiseRatioEnabled", settings.validationDenoiseRatioEnabled);
    settings.validationMaxRejectedRatio = ReadDoubleSetting("Validation/MaxRejectedRatio", settings.validationMaxRejectedRatio);
    settings.validationResidualEnabled = ReadBoolSetting("Validation/ResidualEnabled", settings.validationResidualEnabled);
    settings.validationMaxMedianResidualMm = ReadDoubleSetting("Validation/MaxMedianResidualMm", settings.validationMaxMedianResidualMm);
    settings.validationMaxP95ResidualMm = ReadDoubleSetting("Validation/MaxP95ResidualMm", settings.validationMaxP95ResidualMm);
    settings.validationResidualInlierThresholdMm = ReadDoubleSetting("Validation/ResidualInlierThresholdMm", settings.validationResidualInlierThresholdMm);
    settings.validationMinResidualInlierRatio = ReadDoubleSetting("Validation/MinResidualInlierRatio", settings.validationMinResidualInlierRatio);
    settings.validationKeyPointEnabled = ReadBoolSetting("Validation/KeyPointEnabled", settings.validationKeyPointEnabled);
    settings.validationMinKeyPointCount = ReadIntSetting("Validation/MinKeyPointCount", settings.validationMinKeyPointCount);
    settings.validationMinCornerCount = ReadIntSetting("Validation/MinCornerCount", settings.validationMinCornerCount);
    settings.validationMinSegmentLengthMm = ReadDoubleSetting("Validation/MinSegmentLengthMm", settings.validationMinSegmentLengthMm);
    settings.validationOutputEnabled = ReadBoolSetting("Validation/OutputEnabled", settings.validationOutputEnabled);
    settings.validationMinOutputPointCount = ReadIntSetting("Validation/MinOutputPointCount", settings.validationMinOutputPointCount);
    settings.validationMinOutputLengthRatio = ReadDoubleSetting("Validation/MinOutputLengthRatio", settings.validationMinOutputLengthRatio);
    if (g_hasRuntimeModeOverride)
    {
        settings.mode = g_runtimeModeOverride;
    }

    if (settings.libraryDir.isEmpty())
    {
        settings.libraryDir = DefaultLibraryDir();
    }
    // 算法参数主文件放 Data/（现场拥有、升级不覆盖，DLL 厂商发新参数直接替换该文件）。
    // 不存在时从出厂默认播种一份；Data 文件存在则始终优先；被删则回退出厂默认，永不失效。
    const QString dataConfigPath = DataConfigPath();
    if (!QFileInfo::exists(dataConfigPath))
    {
        QString seedPath = settings.configPath.trimmed();
        if (seedPath.isEmpty() || !QFileInfo::exists(seedPath))
        {
            seedPath = DefaultConfigPath();
        }
        if (QFileInfo::exists(seedPath))
        {
            QFile::copy(seedPath, dataConfigPath);
        }
    }
    if (QFileInfo::exists(dataConfigPath))
    {
        settings.configPath = dataConfigPath;
    }
    else if (settings.configPath.isEmpty() || !QFileInfo::exists(settings.configPath))
    {
        settings.configPath = DefaultConfigPath();
    }
    if (settings.resampleStepMm <= 0.0)
    {
        settings.resampleStepMm = 2.0;
    }
    // 数值滤波参数钳制：负值回退到原管线硬编码默认；0 在阈值类参数上表示"关闭"，保留。
    if (settings.cloudZJumpThresholdMm < 0.0)
    {
        settings.cloudZJumpThresholdMm = 3.0;
    }
    if (settings.cloudZContinuityThresholdMm < 0.0)
    {
        settings.cloudZContinuityThresholdMm = 2.0;
    }
    if (settings.cloudSegmentBreakDistanceMm < 0.0)
    {
        settings.cloudSegmentBreakDistanceMm = 6.0;
    }
    if (settings.fitSampleStepMm <= 0.0)
    {
        settings.fitSampleStepMm = 2.0;
    }
    if (settings.fitSearchWindowMm < 0.0)
    {
        settings.fitSearchWindowMm = 8.0;
    }
    if (settings.fitPiecewiseToleranceMm <= 0.0)
    {
        settings.fitPiecewiseToleranceMm = 4.0;
    }
    settings.fitPiecewiseMinSegmentPoints = std::max(2, settings.fitPiecewiseMinSegmentPoints);
    settings.fitMinPointCount = std::max(2, settings.fitMinPointCount);
    settings.fitSmoothRadius = std::max(0, settings.fitSmoothRadius);
    // 投影提取参数：负值视为 0（自动派生）；候选上限非正回退默认；层位分位夹到 [0,100] 且上界≥下界。
    settings.projectionStationWindowMm = std::max(0.0, settings.projectionStationWindowMm);
    settings.projectionTransverseWindowMm = std::max(0.0, settings.projectionTransverseWindowMm);
    settings.projectionZBandBelowMm = std::max(0.0, settings.projectionZBandBelowMm);
    settings.projectionZBandAboveMm = std::max(0.0, settings.projectionZBandAboveMm);
    if (settings.projectionMaxCandidatePerSeed <= 0)
    {
        settings.projectionMaxCandidatePerSeed = 160;
    }
    settings.projectionLayerLowPercent = std::clamp(settings.projectionLayerLowPercent, 0.0, 100.0);
    settings.projectionLayerHighPercent = std::clamp(
        std::max(settings.projectionLayerHighPercent, settings.projectionLayerLowPercent), 0.0, 100.0);
    settings.projectionSmoothRadius = std::max(0, settings.projectionSmoothRadius);
    settings.validationMinFinitePointCount = std::max(0, settings.validationMinFinitePointCount);
    settings.validationMinProjectedSpanMm = std::max(0.0, settings.validationMinProjectedSpanMm);
    settings.validationMinStationCoverageRatio = std::clamp(settings.validationMinStationCoverageRatio, 0.0, 1.0);
    settings.validationMinLongestContinuousRatio = std::clamp(settings.validationMinLongestContinuousRatio, 0.0, 1.0);
    settings.validationMaxRejectedRatio = std::clamp(settings.validationMaxRejectedRatio, 0.0, 1.0);
    settings.validationMaxMedianResidualMm = std::max(0.0, settings.validationMaxMedianResidualMm);
    settings.validationMaxP95ResidualMm = std::max(0.0, settings.validationMaxP95ResidualMm);
    settings.validationResidualInlierThresholdMm = std::max(0.0, settings.validationResidualInlierThresholdMm);
    settings.validationMinResidualInlierRatio = std::clamp(settings.validationMinResidualInlierRatio, 0.0, 1.0);
    settings.validationMinKeyPointCount = std::max(0, settings.validationMinKeyPointCount);
    settings.validationMinCornerCount = std::max(0, settings.validationMinCornerCount);
    settings.validationMinSegmentLengthMm = std::max(0.0, settings.validationMinSegmentLengthMm);
    settings.validationMinOutputPointCount = std::max(0, settings.validationMinOutputPointCount);
    settings.validationMinOutputLengthRatio = std::max(0.0, settings.validationMinOutputLengthRatio);
    return settings;
}

bool PointCloudProcessingConfig::Save(const Settings& settings, QString* error)
{
    QString localError;
    const auto write = [&localError](const QString& key, const QString& value)
    {
        return WriteSetting(key, value, &localError);
    };

    const bool ok =
        write("General/ProcessingMode", ModeConfigValue(settings.mode))
        && write("FeaturePoint/Strategy", FeaturePointStrategyConfigValue(settings.featurePointStrategy))
        && write("External/LibraryDir", QDir::toNativeSeparators(settings.libraryDir))
        && write("External/ConfigPath", QDir::toNativeSeparators(settings.configPath))
        && write("External/ZTruncationValue", QString::number(settings.zTruncationValue, 'f', 6))
        && write("External/ResampleStepMm", QString::number(settings.resampleStepMm, 'f', 6))
        && write("External/UseWeldedStartTruncation", settings.sdkUseWeldedStartTruncation ? "1" : "0")
        && write("FeaturePoint/SlopeConsistentCornerFit", settings.slopeConsistentCornerFit ? "1" : "0")
        && write("FeaturePoint/ExportFitDebugCloud", settings.exportFitDebugCloud ? "1" : "0")
        && write("Fit/SampleAxis", SampleAxisModeConfigValue(settings.sampleAxisMode))
        && write("CloudAlgo/ZThresholdMm", QString::number(settings.cloudZThresholdMm, 'f', 6))
        && write("CloudAlgo/ZJumpThresholdMm", QString::number(settings.cloudZJumpThresholdMm, 'f', 6))
        && write("CloudAlgo/ZContinuityThresholdMm", QString::number(settings.cloudZContinuityThresholdMm, 'f', 6))
        && write("CloudAlgo/SegmentBreakDistanceMm", QString::number(settings.cloudSegmentBreakDistanceMm, 'f', 6))
        && write("CloudAlgo/KeepLongestSegmentOnly", settings.cloudKeepLongestSegmentOnly ? "1" : "0")
        && write("Fit/SampleStepMm", QString::number(settings.fitSampleStepMm, 'f', 6))
        && write("Fit/SearchWindowMm", QString::number(settings.fitSearchWindowMm, 'f', 6))
        && write("Fit/PiecewiseToleranceMm", QString::number(settings.fitPiecewiseToleranceMm, 'f', 6))
        && write("Fit/PiecewiseMinSegmentPoints", QString::number(settings.fitPiecewiseMinSegmentPoints))
        && write("Fit/MinPointCount", QString::number(settings.fitMinPointCount))
        && write("Fit/SmoothRadius", QString::number(settings.fitSmoothRadius))
        && write("CloudProjection/StationWindowMm", QString::number(settings.projectionStationWindowMm, 'f', 6))
        && write("CloudProjection/TransverseWindowMm", QString::number(settings.projectionTransverseWindowMm, 'f', 6))
        && write("CloudProjection/ZBandBelowMm", QString::number(settings.projectionZBandBelowMm, 'f', 6))
        && write("CloudProjection/ZBandAboveMm", QString::number(settings.projectionZBandAboveMm, 'f', 6))
        && write("CloudProjection/MaxCandidatePerSeed", QString::number(settings.projectionMaxCandidatePerSeed))
        && write("CloudProjection/LayerLowPercent", QString::number(settings.projectionLayerLowPercent, 'f', 6))
        && write("CloudProjection/LayerHighPercent", QString::number(settings.projectionLayerHighPercent, 'f', 6))
        && write("CloudProjection/SmoothRadius", QString::number(settings.projectionSmoothRadius))
        && write("Validation/CoverageEnabled", settings.validationCoverageEnabled ? "1" : "0")
        && write("Validation/MinFinitePointCount", QString::number(settings.validationMinFinitePointCount))
        && write("Validation/MinProjectedSpanMm", QString::number(settings.validationMinProjectedSpanMm, 'f', 6))
        && write("Validation/ContinuityEnabled", settings.validationContinuityEnabled ? "1" : "0")
        && write("Validation/MinStationCoverageRatio", QString::number(settings.validationMinStationCoverageRatio, 'f', 6))
        && write("Validation/MinLongestContinuousRatio", QString::number(settings.validationMinLongestContinuousRatio, 'f', 6))
        && write("Validation/DenoiseRatioEnabled", settings.validationDenoiseRatioEnabled ? "1" : "0")
        && write("Validation/MaxRejectedRatio", QString::number(settings.validationMaxRejectedRatio, 'f', 6))
        && write("Validation/ResidualEnabled", settings.validationResidualEnabled ? "1" : "0")
        && write("Validation/MaxMedianResidualMm", QString::number(settings.validationMaxMedianResidualMm, 'f', 6))
        && write("Validation/MaxP95ResidualMm", QString::number(settings.validationMaxP95ResidualMm, 'f', 6))
        && write("Validation/ResidualInlierThresholdMm", QString::number(settings.validationResidualInlierThresholdMm, 'f', 6))
        && write("Validation/MinResidualInlierRatio", QString::number(settings.validationMinResidualInlierRatio, 'f', 6))
        && write("Validation/KeyPointEnabled", settings.validationKeyPointEnabled ? "1" : "0")
        && write("Validation/MinKeyPointCount", QString::number(settings.validationMinKeyPointCount))
        && write("Validation/MinCornerCount", QString::number(settings.validationMinCornerCount))
        && write("Validation/MinSegmentLengthMm", QString::number(settings.validationMinSegmentLengthMm, 'f', 6))
        && write("Validation/OutputEnabled", settings.validationOutputEnabled ? "1" : "0")
        && write("Validation/MinOutputPointCount", QString::number(settings.validationMinOutputPointCount))
        && write("Validation/MinOutputLengthRatio", QString::number(settings.validationMinOutputLengthRatio, 'f', 6));
    if (!ok && error != nullptr)
    {
        *error = localError;
    }
    return ok;
}

void PointCloudProcessingConfig::SetRuntimeModeOverride(Mode mode)
{
    g_runtimeModeOverride = mode;
    g_hasRuntimeModeOverride = true;
}

void PointCloudProcessingConfig::SetRuntimeScanDirectionOverride(double x, double y, double z)
{
    g_runtimeScanDirectionX = x;
    g_runtimeScanDirectionY = y;
    g_runtimeScanDirectionZ = z;
    g_hasRuntimeScanDirectionOverride = true;
}

bool PointCloudProcessingConfig::RuntimeScanDirectionOverride(double* x, double* y, double* z)
{
    if (!g_hasRuntimeScanDirectionOverride)
    {
        return false;
    }
    if (x != nullptr)
    {
        *x = g_runtimeScanDirectionX;
    }
    if (y != nullptr)
    {
        *y = g_runtimeScanDirectionY;
    }
    if (z != nullptr)
    {
        *z = g_runtimeScanDirectionZ;
    }
    return true;
}

void PointCloudProcessingConfig::ClearRuntimeOverrides()
{
    g_hasRuntimeModeOverride = false;
    g_hasRuntimeScanDirectionOverride = false;
}

QString PointCloudProcessingConfig::ModeDisplayName(Mode mode)
{
    switch (mode)
    {
    case Mode::ExternalCorrugatedSheet:
        return "SDK点云算法全处理";
    case Mode::SdkBaseWeldFit:
        return "SDK点云算法+拟合";
    case Mode::CloudFit:
        return "点云算法+拟合";
    case Mode::LegacyLaserPath:
    default:
        return "特征点+拟合";
    }
}

QString PointCloudProcessingConfig::ModeConfigValue(Mode mode)
{
    switch (mode)
    {
    case Mode::ExternalCorrugatedSheet:
        return "ExternalCorrugatedSheet";
    case Mode::SdkBaseWeldFit:
        return "SdkBaseWeldFit";
    case Mode::CloudFit:
        return "CloudFit";
    case Mode::LegacyLaserPath:
    default:
        return "LegacyLaserPath";
    }
}

PointCloudProcessingConfig::Mode PointCloudProcessingConfig::ModeFromConfigValue(const QString& value)
{
    const QString normalized = value.trimmed().toLower();
    if (normalized == "1"
        || normalized == "external"
        || normalized == "externalcorrugatedsheet"
        || normalized == "new"
        || normalized == "sdkfull")
    {
        return Mode::ExternalCorrugatedSheet;
    }
    if (normalized == "2"
        || normalized == "sdkbaseweldfit"
        || normalized == "sdkfit"
        || normalized == "sdkbaseweld")
    {
        return Mode::SdkBaseWeldFit;
    }
    if (normalized == "3"
        || normalized == "cloudfit"
        || normalized == "cloud")
    {
        return Mode::CloudFit;
    }
    return Mode::LegacyLaserPath;
}

QString PointCloudProcessingConfig::FeaturePointStrategyDisplayName(FeaturePointStrategy strategy)
{
    switch (strategy)
    {
    case FeaturePointStrategy::WorkpieceProjection:
        return "方案三：立板投影到底板";
    case FeaturePointStrategy::RobustSegmentedKeys:
        return "方案二：鲁棒分段关键点";
    case FeaturePointStrategy::SlopeWaveFiltered:
        return "方案一：斜面波动滤波";
    case FeaturePointStrategy::LegacyGeometry:
    default:
        return "旧版几何拟合";
    }
}

QString PointCloudProcessingConfig::FeaturePointStrategyConfigValue(FeaturePointStrategy strategy)
{
    switch (strategy)
    {
    case FeaturePointStrategy::WorkpieceProjection:
        return "WorkpieceProjection";
    case FeaturePointStrategy::RobustSegmentedKeys:
        return "RobustSegmentedKeys";
    case FeaturePointStrategy::SlopeWaveFiltered:
        return "SlopeWaveFiltered";
    case FeaturePointStrategy::LegacyGeometry:
    default:
        return "LegacyGeometry";
    }
}

QString PointCloudProcessingConfig::SampleAxisModeConfigValue(SampleAxisMode mode)
{
    switch (mode)
    {
    case SampleAxisMode::AxisX:
        return "x";
    case SampleAxisMode::AxisY:
        return "y";
    case SampleAxisMode::Auto:
    default:
        return "auto";
    }
}

PointCloudProcessingConfig::SampleAxisMode PointCloudProcessingConfig::SampleAxisModeFromConfigValue(const QString& value)
{
    const QString normalized = value.trimmed().toLower();
    if (normalized == "x" || normalized == "axisx" || normalized == "1")
    {
        return SampleAxisMode::AxisX;
    }
    if (normalized == "y" || normalized == "axisy" || normalized == "2")
    {
        return SampleAxisMode::AxisY;
    }
    return SampleAxisMode::Auto;
}

PointCloudProcessingConfig::FeaturePointStrategy PointCloudProcessingConfig::FeaturePointStrategyFromConfigValue(
    const QString& value)
{
    const QString normalized = value.trimmed().toLower();
    if (normalized == "2"
        || normalized == "robust"
        || normalized == "robustsegmented"
        || normalized == "robustsegmentedkeys"
        || normalized == "scheme2")
    {
        return FeaturePointStrategy::RobustSegmentedKeys;
    }
    if (normalized == "3"
        || normalized == "projection"
        || normalized == "workpieceprojection"
        || normalized == "verticalprojection"
        || normalized == "scheme3")
    {
        return FeaturePointStrategy::WorkpieceProjection;
    }
    if (normalized == "1"
        || normalized == "slope"
        || normalized == "slopewave"
        || normalized == "slopewavefiltered"
        || normalized == "scheme1")
    {
        return FeaturePointStrategy::SlopeWaveFiltered;
    }
    return FeaturePointStrategy::LegacyGeometry;
}
