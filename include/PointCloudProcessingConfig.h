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
    };

    static QString SettingsFilePath();
    static QString DefaultLibraryDir();
    static QString DefaultConfigPath();
    static Settings Load();
    static bool Save(const Settings& settings, QString* error = nullptr);
    static QString ModeDisplayName(Mode mode);
    static QString ModeConfigValue(Mode mode);
    static Mode ModeFromConfigValue(const QString& value);
    static QString FeaturePointStrategyDisplayName(FeaturePointStrategy strategy);
    static QString FeaturePointStrategyConfigValue(FeaturePointStrategy strategy);
    static FeaturePointStrategy FeaturePointStrategyFromConfigValue(const QString& value);
};
