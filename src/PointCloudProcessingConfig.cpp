#include "PointCloudProcessingConfig.h"

#include "ConfigDatabase.h"
#include "RobotDataHelper.h"

#include <QDir>
#include <QFileInfo>
#include <QSettings>

namespace
{
constexpr auto SETTINGS_RELATIVE_PATH = "Data/PointCloudProcessing.ini";

QString ReadSetting(const QString& key, const QString& defaultValue = QString())
{
    QString value;
    if (ConfigDatabase::ReadSetting(PointCloudProcessingConfig::SettingsFilePath(), key, &value))
    {
        return value;
    }

    QSettings settings(PointCloudProcessingConfig::SettingsFilePath(), QSettings::IniFormat);
    return settings.value(key, defaultValue).toString();
}

bool WriteSetting(const QString& key, const QString& value, QString* error)
{
    const bool databaseOk = ConfigDatabase::WriteSetting(PointCloudProcessingConfig::SettingsFilePath(), key, value);

    QSettings settings(PointCloudProcessingConfig::SettingsFilePath(), QSettings::IniFormat);
    settings.setValue(key, value);
    settings.sync();
    const bool fileOk = settings.status() == QSettings::NoError;
    if (databaseOk || fileOk)
    {
        return true;
    }

    if (error != nullptr)
    {
        *error = QString("写入点云处理配置失败：%1 / %2")
            .arg(PointCloudProcessingConfig::SettingsFilePath(), key);
    }
    return false;
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

QString PointCloudProcessingConfig::SettingsFilePath()
{
    return RobotDataHelper::BuildProjectPath(SETTINGS_RELATIVE_PATH);
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

    const QString fallbackText = ReadSetting("External/FallbackToLegacy", "1").trimmed().toLower();
    settings.fallbackToLegacy = fallbackText.isEmpty()
        || fallbackText == "1"
        || fallbackText == "true"
        || fallbackText == "yes";

    if (settings.libraryDir.isEmpty())
    {
        settings.libraryDir = DefaultLibraryDir();
    }
    if (settings.configPath.isEmpty())
    {
        settings.configPath = DefaultConfigPath();
    }
    if (settings.resampleStepMm <= 0.0)
    {
        settings.resampleStepMm = 2.0;
    }
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
        && write("External/FallbackToLegacy", settings.fallbackToLegacy ? "1" : "0");
    if (!ok && error != nullptr)
    {
        *error = localError;
    }
    return ok;
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
