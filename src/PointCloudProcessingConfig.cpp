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
bool g_hasRuntimeFallbackOverride = false;
bool g_runtimeFallbackOverride = true;
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
    settings.slopeConsistentCornerFit = ReadBoolSetting("FeaturePoint/SlopeConsistentCornerFit", false);
    settings.exportFitDebugCloud = ReadBoolSetting("FeaturePoint/ExportFitDebugCloud", true);
    settings.fallbackToLegacy = ReadBoolSetting("External/FallbackToLegacy", true);
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
    if (g_hasRuntimeFallbackOverride)
    {
        settings.fallbackToLegacy = g_runtimeFallbackOverride;
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
        && write("FeaturePoint/SlopeConsistentCornerFit", settings.slopeConsistentCornerFit ? "1" : "0")
        && write("FeaturePoint/ExportFitDebugCloud", settings.exportFitDebugCloud ? "1" : "0")
        && write("External/FallbackToLegacy", settings.fallbackToLegacy ? "1" : "0")
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

void PointCloudProcessingConfig::SetRuntimeFallbackToLegacyOverride(bool fallbackToLegacy)
{
    g_runtimeFallbackOverride = fallbackToLegacy;
    g_hasRuntimeFallbackOverride = true;
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
    g_hasRuntimeFallbackOverride = false;
    g_hasRuntimeScanDirectionOverride = false;
}

QString PointCloudProcessingConfig::ModeDisplayName(Mode mode)
{
    switch (mode)
    {
    case Mode::ExternalCorrugatedSheet:
        return "新版精测点云库";
    case Mode::LegacyLaserPath:
    default:
        return "旧版目标点处理";
    }
}

QString PointCloudProcessingConfig::ModeConfigValue(Mode mode)
{
    switch (mode)
    {
    case Mode::ExternalCorrugatedSheet:
        return "ExternalCorrugatedSheet";
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
        || normalized == "new")
    {
        return Mode::ExternalCorrugatedSheet;
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
