#include "PointCloudProcessingConfig.h"

#include "ConfigDatabase.h"
#include "RobotDataHelper.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QMap>

#include <algorithm>
#include <cmath>

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
thread_local const QMap<QString, QString>* g_activeSettingsSnapshot = nullptr;

class SettingsSnapshotScope
{
public:
    explicit SettingsSnapshotScope(const QMap<QString, QString>& snapshot)
        : m_previous(g_activeSettingsSnapshot)
    {
        g_activeSettingsSnapshot = &snapshot;
    }

    ~SettingsSnapshotScope()
    {
        g_activeSettingsSnapshot = m_previous;
    }

private:
    const QMap<QString, QString>* m_previous = nullptr;
};

void NormalizeFiniteLoadValues(PointCloudProcessingConfig::Settings& settings)
{
    const PointCloudProcessingConfig::Settings defaults;
    const auto useDefaultIfNonFinite = [](double& value, double defaultValue)
    {
        if (!std::isfinite(value))
        {
            value = defaultValue;
        }
    };

    useDefaultIfNonFinite(settings.zTruncationValue, defaults.zTruncationValue);
    useDefaultIfNonFinite(settings.resampleStepMm, defaults.resampleStepMm);
    useDefaultIfNonFinite(settings.fitSampleStepMm, defaults.fitSampleStepMm);
    useDefaultIfNonFinite(settings.fitEndPeriodMergeFrac, defaults.fitEndPeriodMergeFrac);
    useDefaultIfNonFinite(settings.fitSameTypeFlatSlope, defaults.fitSameTypeFlatSlope);

    // std::clamp/std::min/std::max do not sanitize NaN. Restore every floating-point
    // quality threshold before the range clamps and the Enforce safety floors run.
    useDefaultIfNonFinite(settings.validationMinProjectedSpanMm, defaults.validationMinProjectedSpanMm);
    useDefaultIfNonFinite(settings.validationMinStationCoverageRatio, defaults.validationMinStationCoverageRatio);
    useDefaultIfNonFinite(settings.validationMinLongestContinuousRatio, defaults.validationMinLongestContinuousRatio);
    useDefaultIfNonFinite(settings.validationMaxRejectedRatio, defaults.validationMaxRejectedRatio);
    useDefaultIfNonFinite(settings.validationMaxMedianResidualMm, defaults.validationMaxMedianResidualMm);
    useDefaultIfNonFinite(settings.validationMaxP95ResidualMm, defaults.validationMaxP95ResidualMm);
    useDefaultIfNonFinite(settings.validationResidualInlierThresholdMm, defaults.validationResidualInlierThresholdMm);
    useDefaultIfNonFinite(settings.validationMinResidualInlierRatio, defaults.validationMinResidualInlierRatio);
    useDefaultIfNonFinite(settings.validationMinSegmentLengthMm, defaults.validationMinSegmentLengthMm);
    useDefaultIfNonFinite(settings.validationMinOutputLengthRatio, defaults.validationMinOutputLengthRatio);
    useDefaultIfNonFinite(
        settings.validationMinNonLapSegmentHardMm,
        defaults.validationMinNonLapSegmentHardMm);
    useDefaultIfNonFinite(
        settings.validationMinLapOrEndpointSegmentHardMm,
        defaults.validationMinLapOrEndpointSegmentHardMm);
    useDefaultIfNonFinite(
        settings.validationMaxFinalPositionStepMm,
        defaults.validationMaxFinalPositionStepMm);
    useDefaultIfNonFinite(
        settings.validationMaxFinalControllerEulerStepDeg,
        defaults.validationMaxFinalControllerEulerStepDeg);
    useDefaultIfNonFinite(
        settings.validationMaxFinalPhysicalOrientationStepDeg,
        defaults.validationMaxFinalPhysicalOrientationStepDeg);
    useDefaultIfNonFinite(
        settings.validationMinFinalToPreCompLengthRatio,
        defaults.validationMinFinalToPreCompLengthRatio);
    useDefaultIfNonFinite(
        settings.validationMaxFinalToPreCompLengthRatio,
        defaults.validationMaxFinalToPreCompLengthRatio);
    useDefaultIfNonFinite(
        settings.validationMinFinalMatchedArcRatio,
        defaults.validationMinFinalMatchedArcRatio);
    useDefaultIfNonFinite(
        settings.validationMinFinalSourceUniqueCoverageRatio,
        defaults.validationMinFinalSourceUniqueCoverageRatio);
    useDefaultIfNonFinite(
        settings.validationMinFinalSourceArcSpanRatio,
        defaults.validationMinFinalSourceArcSpanRatio);
    useDefaultIfNonFinite(
        settings.validationMaxFinalSourceDisplacementMm,
        defaults.validationMaxFinalSourceDisplacementMm);
    useDefaultIfNonFinite(
        settings.validationMaxFinalSourcePhysicalOrientationDeltaDeg,
        defaults.validationMaxFinalSourcePhysicalOrientationDeltaDeg);
}

void NormalizeConfigurableWeldValidationThresholds(
    PointCloudProcessingConfig::Settings& settings)
{
    settings.validationMinNonLapSegmentHardMm =
        std::clamp(settings.validationMinNonLapSegmentHardMm, 0.0, 9999.0);
    settings.validationMinLapOrEndpointSegmentHardMm =
        std::clamp(settings.validationMinLapOrEndpointSegmentHardMm, 0.0, 9999.0);
    settings.validationMaxFinalPositionStepMm =
        std::clamp(settings.validationMaxFinalPositionStepMm, 0.001, 999999.0);
    settings.validationMaxFinalControllerEulerStepDeg =
        std::clamp(settings.validationMaxFinalControllerEulerStepDeg, 0.001, 360.0);
    settings.validationMaxFinalPhysicalOrientationStepDeg =
        std::clamp(settings.validationMaxFinalPhysicalOrientationStepDeg, 0.001, 180.0);
    settings.validationMinFinalToPreCompLengthRatio =
        std::clamp(settings.validationMinFinalToPreCompLengthRatio, 0.0, 10.0);
    settings.validationMaxFinalToPreCompLengthRatio =
        std::clamp(
            std::max(
                settings.validationMaxFinalToPreCompLengthRatio,
                settings.validationMinFinalToPreCompLengthRatio),
            0.0,
            10.0);
    settings.validationMinFinalMatchedArcRatio =
        std::clamp(settings.validationMinFinalMatchedArcRatio, 0.0, 1.0);
    settings.validationMinFinalSourceUniqueCoverageRatio =
        std::clamp(settings.validationMinFinalSourceUniqueCoverageRatio, 0.0, 1.0);
    settings.validationMinFinalSourceArcSpanRatio =
        std::clamp(settings.validationMinFinalSourceArcSpanRatio, 0.0, 1.0);
    settings.validationMaxFinalSourceDisplacementMm =
        std::clamp(settings.validationMaxFinalSourceDisplacementMm, 0.001, 999999.0);
    settings.validationMaxFinalSourcePhysicalOrientationDeltaDeg =
        std::clamp(
            settings.validationMaxFinalSourcePhysicalOrientationDeltaDeg,
            0.001,
            180.0);
}

void ApplyEnforceValidationSafetyBounds(PointCloudProcessingConfig::Settings& settings)
{
    if (settings.validationPolicy != PointCloudProcessingConfig::ValidationPolicy::Enforce)
    {
        return;
    }
    if (settings.validationCoverageEnabled)
    {
        settings.validationMinFinitePointCount = std::max(300, settings.validationMinFinitePointCount);
        settings.validationMinProjectedSpanMm = std::max(180.0, settings.validationMinProjectedSpanMm);
    }
    if (settings.validationContinuityEnabled)
    {
        settings.validationMinStationCoverageRatio = std::max(0.55, settings.validationMinStationCoverageRatio);
        settings.validationMinLongestContinuousRatio = std::max(0.60, settings.validationMinLongestContinuousRatio);
    }
    if (settings.validationDenoiseRatioEnabled)
    {
        settings.validationMaxRejectedRatio = std::min(0.40, settings.validationMaxRejectedRatio);
    }
    if (settings.validationResidualEnabled)
    {
        settings.validationMaxMedianResidualMm =
            settings.validationMaxMedianResidualMm <= 0.0
            ? 3.0
            : std::min(3.0, settings.validationMaxMedianResidualMm);
        settings.validationMaxP95ResidualMm =
            settings.validationMaxP95ResidualMm <= 0.0
            ? 8.0
            : std::min(8.0, settings.validationMaxP95ResidualMm);
        settings.validationResidualInlierThresholdMm =
            settings.validationResidualInlierThresholdMm <= 0.0
            ? 6.0
            : std::min(6.0, settings.validationResidualInlierThresholdMm);
        settings.validationMinResidualInlierRatio = std::max(0.75, settings.validationMinResidualInlierRatio);
    }
    if (settings.validationKeyPointEnabled)
    {
        settings.validationMinKeyPointCount = std::max(6, settings.validationMinKeyPointCount);
        settings.validationMinCornerCount = std::max(4, settings.validationMinCornerCount);
    }
    if (settings.validationOutputEnabled)
    {
        settings.validationMinOutputPointCount = std::max(80, settings.validationMinOutputPointCount);
        settings.validationMinOutputLengthRatio = std::max(0.70, settings.validationMinOutputLengthRatio);
    }
}

QString ReadSetting(const QString& key, const QString& defaultValue = QString())
{
    if (g_activeSettingsSnapshot != nullptr)
    {
        const auto it = g_activeSettingsSnapshot->constFind(key);
        return it == g_activeSettingsSnapshot->cend() ? defaultValue : it.value();
    }
    QString value;
    if (ConfigDatabase::ReadScopedSetting(QStringLiteral("global"), QString(), SETTINGS_GROUP, key, &value))
    {
        return value;
    }
    return defaultValue;
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
    return ok && std::isfinite(value) ? value : defaultValue;
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

bool PointCloudProcessingConfig::CoreSafetyGatesEnabled(const Settings& settings)
{
    return settings.safetyGateProofIntegrityEnabled
        && settings.safetyGateProductionPurposeEnabled
        && settings.safetyGateRobotNameBindingEnabled
        && settings.safetyGateCaseBindingEnabled
        && settings.safetyGateEndpointBindingEnabled
        && settings.safetyGateCameraHandEyeBindingEnabled
        && settings.safetyGateFreshnessEnabled
        && settings.safetyGatePolicySnapshotEnabled
        && settings.safetyGateInputEvidenceEnabled
        && settings.safetyGateAuthorizedPoseIdentityEnabled
        && settings.safetyGateTrajectoryStructureEnabled
        && settings.safetyGateMotionPrecheckEnabled;
}

bool PointCloudProcessingConfig::HasDisabledCoreSafetyGate(const Settings& settings)
{
    return !CoreSafetyGatesEnabled(settings);
}

PointCloudProcessingConfig::Settings PointCloudProcessingConfig::Load()
{
    const QMap<QString, QString> settingsSnapshot = ConfigDatabase::ReadScopedSettings(
        QStringLiteral("global"), QString(), SETTINGS_GROUP);
    const SettingsSnapshotScope snapshotScope(settingsSnapshot);
    Settings settings;
    settings.mode = ModeFromConfigValue(ReadSetting("General/ProcessingMode", ModeConfigValue(settings.mode)));
    settings.featurePointStrategy = FeaturePointStrategyFromConfigValue(
        ReadSetting("FeaturePoint/Strategy", FeaturePointStrategyConfigValue(settings.featurePointStrategy)));
    settings.libraryDir = ReadSetting("External/LibraryDir", DefaultLibraryDir()).trimmed();
    settings.configPath = ReadSetting("External/ConfigPath", DefaultConfigPath()).trimmed();
    settings.zTruncationValue = ReadDoubleSetting("External/ZTruncationValue", settings.zTruncationValue);
    settings.resampleStepMm = ReadDoubleSetting("External/ResampleStepMm", settings.resampleStepMm);
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
    settings.sdkBasePresmoothEnable = ReadBoolSetting("Fit/SdkBasePresmoothEnable", settings.sdkBasePresmoothEnable);
    settings.sdkBasePresmoothWindowMm = ReadDoubleSetting("Fit/SdkBasePresmoothWindowMm", settings.sdkBasePresmoothWindowMm);
    settings.sdkBasePresmoothEdgeMm = ReadDoubleSetting("Fit/SdkBasePresmoothEdgeMm", settings.sdkBasePresmoothEdgeMm);
    settings.fitAzimuthTurnThresholdDeg = ReadDoubleSetting("Fit/AzimuthTurnThresholdDeg", settings.fitAzimuthTurnThresholdDeg);
    settings.fitAzimuthHeadingWindow = ReadIntSetting("Fit/AzimuthHeadingWindow", settings.fitAzimuthHeadingWindow);
    settings.fitAzimuthNmsSpanMm = ReadDoubleSetting("Fit/AzimuthNmsSpanMm", settings.fitAzimuthNmsSpanMm);
    settings.fitAzimuthStraightenResidualMm = ReadDoubleSetting("Fit/AzimuthStraightenResidualMm", settings.fitAzimuthStraightenResidualMm);
    settings.fitCornerRefineEnable = ReadBoolSetting("Fit/CornerRefineEnable", settings.fitCornerRefineEnable);
    settings.fitAzimuthRefineFloorMm = ReadDoubleSetting("Fit/AzimuthRefineFloorMm", settings.fitAzimuthRefineFloorMm);
    settings.fitCornerRefineOneSidedPct = ReadDoubleSetting("Fit/CornerRefineOneSidedPct", settings.fitCornerRefineOneSidedPct);
    settings.fitCornerRefineMidMultiple = ReadDoubleSetting("Fit/CornerRefineMidMultiple", settings.fitCornerRefineMidMultiple);
    settings.fitCornerRefineEndFracPct = ReadDoubleSetting("Fit/CornerRefineEndFracPct", settings.fitCornerRefineEndFracPct);
    settings.fitCornerPatternRefitEnable = ReadBoolSetting("Fit/CornerPatternRefitEnable", settings.fitCornerPatternRefitEnable);
    settings.fitCornerPlatformMinSegPoints = ReadIntSetting("Fit/CornerPlatformMinSegPoints", settings.fitCornerPlatformMinSegPoints);
    settings.enableLapMisalignmentSplit = ReadBoolSetting("Fit/EnableLapMisalignmentSplit", settings.enableLapMisalignmentSplit);
    settings.lapStepHeightThresholdMm = ReadDoubleSetting("Fit/LapStepHeightThresholdMm", settings.lapStepHeightThresholdMm);
    settings.lapStepStationWindowMm = ReadDoubleSetting("Fit/LapStepStationWindowMm", settings.lapStepStationWindowMm);
    settings.lapStepSideFlatnessMm = ReadDoubleSetting("Fit/LapStepSideFlatnessMm", settings.lapStepSideFlatnessMm);
    settings.lapStepPlatformSlopeMax = ReadDoubleSetting("Fit/LapStepPlatformSlopeMax", settings.lapStepPlatformSlopeMax);
    settings.fitEdgeTruncateEnable = ReadBoolSetting("Fit/EdgeTruncateEnable", settings.fitEdgeTruncateEnable);
    settings.fitTruncateHeadMm = ReadDoubleSetting("Fit/TruncateHeadMm", settings.fitTruncateHeadMm);
    settings.fitTruncateTailMm = ReadDoubleSetting("Fit/TruncateTailMm", settings.fitTruncateTailMm);
    settings.fitEndPeriodRecoverEnable = ReadBoolSetting("Fit/EndPeriodRecoverEnable", settings.fitEndPeriodRecoverEnable);
    settings.fitEndPeriodRatioThreshold = ReadDoubleSetting("Fit/EndPeriodRatioThreshold", settings.fitEndPeriodRatioThreshold);
    settings.fitEndPeriodMinBendDeg = ReadDoubleSetting("Fit/EndPeriodMinBendDeg", settings.fitEndPeriodMinBendDeg);
    settings.fitEndPeriodMergeFrac = ReadDoubleSetting("Fit/EndPeriodMergeFrac", settings.fitEndPeriodMergeFrac);
    settings.fitPlatformSnapEnable = ReadBoolSetting("Fit/PlatformSnapEnable", settings.fitPlatformSnapEnable);
    settings.fitPlatformSnapFlatSlope = ReadDoubleSetting("Fit/PlatformSnapFlatSlope", settings.fitPlatformSnapFlatSlope);
    settings.fitPlatformSnapMinFrac = ReadDoubleSetting("Fit/PlatformSnapMinFrac", settings.fitPlatformSnapMinFrac);
    // 兼容旧库：独立开关首次出现前，短段合并与 EndPeriodRecover 共用开关、斜率门共用 PlatformSnap 参数。
    // 新键不存在时沿用旧行为；保存一次后改为各自独立。
    settings.fitSameTypeShortMergeEnable =
        ReadBoolSetting("Fit/SameTypeShortMergeEnable", settings.fitEndPeriodRecoverEnable);
    settings.fitSameTypeMinReferenceSegments =
        ReadIntSetting("Fit/SameTypeMinReferenceSegments", settings.fitSameTypeMinReferenceSegments);
    settings.fitSameTypeFlatSlope =
        ReadDoubleSetting("Fit/SameTypeFlatSlope", settings.fitPlatformSnapFlatSlope);
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
    settings.exportWorkpieceFrameDebug = ReadBoolSetting("FeaturePoint/ExportWorkpieceFrameDebug", false);
    const int storedValidationProfileVersion =
        ReadIntSetting("Validation/ProfileVersion", 0);
    settings.validationProfileVersion = storedValidationProfileVersion;
    settings.validationPolicy = ValidationPolicyFromConfigValue(
        ReadSetting("Validation/Policy", ValidationPolicyConfigValue(settings.validationPolicy)));
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
    settings.validationSegmentHardLimitsEnabled =
        ReadBoolSetting("Validation/SegmentHardLimitsEnabled", settings.validationSegmentHardLimitsEnabled);
    settings.validationMinNonLapSegmentHardMm =
        ReadDoubleSetting("Validation/MinNonLapSegmentHardMm", settings.validationMinNonLapSegmentHardMm);
    settings.validationMinLapOrEndpointSegmentHardMm =
        ReadDoubleSetting(
            "Validation/MinLapOrEndpointSegmentHardMm",
            settings.validationMinLapOrEndpointSegmentHardMm);
    settings.validationFinalTrajectoryStepEnabled =
        ReadBoolSetting("Validation/FinalTrajectoryStepEnabled", settings.validationFinalTrajectoryStepEnabled);
    settings.validationMaxFinalPositionStepMm =
        ReadDoubleSetting("Validation/MaxFinalPositionStepMm", settings.validationMaxFinalPositionStepMm);
    settings.validationMaxFinalControllerEulerStepDeg =
        ReadDoubleSetting(
            "Validation/MaxFinalControllerEulerStepDeg",
            settings.validationMaxFinalControllerEulerStepDeg);
    settings.validationMaxFinalPhysicalOrientationStepDeg =
        ReadDoubleSetting(
            "Validation/MaxFinalPhysicalOrientationStepDeg",
            settings.validationMaxFinalPhysicalOrientationStepDeg);
    settings.validationFinalLengthBindingEnabled =
        ReadBoolSetting("Validation/FinalLengthBindingEnabled", settings.validationFinalLengthBindingEnabled);
    settings.validationMinFinalToPreCompLengthRatio =
        ReadDoubleSetting(
            "Validation/MinFinalToPreCompLengthRatio",
            settings.validationMinFinalToPreCompLengthRatio);
    settings.validationMaxFinalToPreCompLengthRatio =
        ReadDoubleSetting(
            "Validation/MaxFinalToPreCompLengthRatio",
            settings.validationMaxFinalToPreCompLengthRatio);
    settings.validationFinalTopologyBindingEnabled =
        ReadBoolSetting("Validation/FinalTopologyBindingEnabled", settings.validationFinalTopologyBindingEnabled);
    settings.validationMinFinalMatchedArcRatio =
        ReadDoubleSetting(
            "Validation/MinFinalMatchedArcRatio",
            settings.validationMinFinalMatchedArcRatio);
    settings.validationMinFinalSourceUniqueCoverageRatio =
        ReadDoubleSetting(
            "Validation/MinFinalSourceUniqueCoverageRatio",
            settings.validationMinFinalSourceUniqueCoverageRatio);
    settings.validationMinFinalSourceArcSpanRatio =
        ReadDoubleSetting(
            "Validation/MinFinalSourceArcSpanRatio",
            settings.validationMinFinalSourceArcSpanRatio);
    settings.validationFinalSourceBindingEnabled =
        ReadBoolSetting("Validation/FinalSourceBindingEnabled", settings.validationFinalSourceBindingEnabled);
    settings.validationMaxFinalSourceDisplacementMm =
        ReadDoubleSetting(
            "Validation/MaxFinalSourceDisplacementMm",
            settings.validationMaxFinalSourceDisplacementMm);
    settings.validationMaxFinalSourcePhysicalOrientationDeltaDeg =
        ReadDoubleSetting(
            "Validation/MaxFinalSourcePhysicalOrientationDeltaDeg",
            settings.validationMaxFinalSourcePhysicalOrientationDeltaDeg);
    settings.validationFinalSemanticIntegrityEnabled =
        ReadBoolSetting("Validation/FinalSemanticIntegrityEnabled", settings.validationFinalSemanticIntegrityEnabled);
    settings.safetyGateProofIntegrityEnabled =
        ReadBoolSetting("SafetyGates/ProofIntegrityEnabled", settings.safetyGateProofIntegrityEnabled);
    settings.safetyGateProductionPurposeEnabled =
        ReadBoolSetting("SafetyGates/ProductionPurposeEnabled", settings.safetyGateProductionPurposeEnabled);
    settings.safetyGateRobotNameBindingEnabled =
        ReadBoolSetting("SafetyGates/RobotNameBindingEnabled", settings.safetyGateRobotNameBindingEnabled);
    settings.safetyGateCaseBindingEnabled =
        ReadBoolSetting("SafetyGates/CaseBindingEnabled", settings.safetyGateCaseBindingEnabled);
    settings.safetyGateEndpointBindingEnabled =
        ReadBoolSetting("SafetyGates/EndpointBindingEnabled", settings.safetyGateEndpointBindingEnabled);
    settings.safetyGateCameraHandEyeBindingEnabled =
        ReadBoolSetting("SafetyGates/CameraHandEyeBindingEnabled", settings.safetyGateCameraHandEyeBindingEnabled);
    settings.safetyGateFreshnessEnabled =
        ReadBoolSetting("SafetyGates/FreshnessEnabled", settings.safetyGateFreshnessEnabled);
    settings.safetyGatePolicySnapshotEnabled =
        ReadBoolSetting("SafetyGates/PolicySnapshotEnabled", settings.safetyGatePolicySnapshotEnabled);
    settings.safetyGateInputEvidenceEnabled =
        ReadBoolSetting("SafetyGates/InputEvidenceEnabled", settings.safetyGateInputEvidenceEnabled);
    settings.safetyGateAuthorizedPoseIdentityEnabled =
        ReadBoolSetting("SafetyGates/AuthorizedPoseIdentityEnabled", settings.safetyGateAuthorizedPoseIdentityEnabled);
    settings.safetyGateTrajectoryStructureEnabled =
        ReadBoolSetting("SafetyGates/TrajectoryStructureEnabled", settings.safetyGateTrajectoryStructureEnabled);
    settings.safetyGateMotionPrecheckEnabled =
        ReadBoolSetting("SafetyGates/MotionPrecheckEnabled", settings.safetyGateMotionPrecheckEnabled);
    const int storedSafetyGateBehaviorVersion =
        ReadIntSetting("SafetyGates/BehaviorVersion", 0);

    // 旧现场数据库曾把六类门禁全部持久化为 0，安装/OTA 又会保留 Data，单靠 C++ 默认值无法恢复。
    // Profile v1 已用 101 组历史语料完成阈值回算；仅旧配置升级时进入 Enforce 并打开全部六类指标。
    // v1+ 配置中的开关是操作员显式选择，必须按原值恢复。
    if (storedValidationProfileVersion < CURRENT_VALIDATION_PROFILE_VERSION)
    {
        settings.validationPolicy = ValidationPolicy::Enforce;
        settings.validationCoverageEnabled = true;
        settings.validationContinuityEnabled = true;
        settings.validationDenoiseRatioEnabled = true;
        settings.validationResidualEnabled = true;
        settings.validationKeyPointEnabled = true;
        settings.validationOutputEnabled = true;
    }
    settings.validationProfileVersion = CURRENT_VALIDATION_PROFILE_VERSION;
    // 兼容修复：BehaviorVersion 0 曾在任一核心记录开关关闭时把 Policy 强制持久化为
    // Audit。仅迁移该最早版本；v1 记录型开关和 v2 实际启停开关均保留操作员选择。
    if (storedSafetyGateBehaviorVersion < 1
        && settings.validationPolicy == ValidationPolicy::Audit
        && HasDisabledCoreSafetyGate(settings))
    {
        settings.validationPolicy = ValidationPolicy::Enforce;
    }
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
    NormalizeFiniteLoadValues(settings);
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
    settings.fitAzimuthTurnThresholdDeg = std::max(1.0, std::min(90.0, settings.fitAzimuthTurnThresholdDeg));
    settings.fitAzimuthHeadingWindow = std::max(2, settings.fitAzimuthHeadingWindow);
    settings.fitAzimuthNmsSpanMm = std::max(1.0, settings.fitAzimuthNmsSpanMm);
    settings.fitAzimuthStraightenResidualMm = std::max(0.5, settings.fitAzimuthStraightenResidualMm);
    settings.fitAzimuthRefineFloorMm = std::max(0.0, settings.fitAzimuthRefineFloorMm);  // <=0 表示关闭细化
    settings.fitCornerRefineOneSidedPct = std::clamp(settings.fitCornerRefineOneSidedPct, 50.0, 100.0);
    settings.fitCornerRefineMidMultiple = std::max(1.0, settings.fitCornerRefineMidMultiple);
    settings.fitCornerRefineEndFracPct = std::clamp(settings.fitCornerRefineEndFracPct, 0.0, 50.0);
    settings.fitCornerPlatformMinSegPoints = std::max(3, settings.fitCornerPlatformMinSegPoints);
    settings.lapStepHeightThresholdMm = std::max(0.3, settings.lapStepHeightThresholdMm);
    settings.lapStepStationWindowMm = std::max(2.0, settings.lapStepStationWindowMm);
    settings.lapStepSideFlatnessMm = std::max(0.02, settings.lapStepSideFlatnessMm);
    settings.lapStepPlatformSlopeMax = std::max(0.02, settings.lapStepPlatformSlopeMax);
    settings.fitEndPeriodMergeFrac = std::clamp(settings.fitEndPeriodMergeFrac, 0.05, 0.90);
    settings.fitSameTypeMinReferenceSegments = std::max(2, settings.fitSameTypeMinReferenceSegments);
    settings.fitSameTypeFlatSlope = std::clamp(settings.fitSameTypeFlatSlope, 0.03, 0.30);
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
    NormalizeConfigurableWeldValidationThresholds(settings);
    ApplyEnforceValidationSafetyBounds(settings);
    return settings;
}

bool PointCloudProcessingConfig::Save(const Settings& settings, QString* error)
{
    Settings normalizedSettings = settings;
    normalizedSettings.validationProfileVersion = CURRENT_VALIDATION_PROFILE_VERSION;
    NormalizeFiniteLoadValues(normalizedSettings);
    NormalizeConfigurableWeldValidationThresholds(normalizedSettings);
    ApplyEnforceValidationSafetyBounds(normalizedSettings);
    QMap<QString, QString> pendingValues;
    const auto write = [&pendingValues](const QString& key, const QString& value)
    {
        pendingValues.insert(key, value);
        return true;
    };

    const bool valuesPrepared =
        write("General/ProcessingMode", ModeConfigValue(settings.mode))
        && write("FeaturePoint/Strategy", FeaturePointStrategyConfigValue(settings.featurePointStrategy))
        && write("External/LibraryDir", QDir::toNativeSeparators(settings.libraryDir))
        && write("External/ConfigPath", QDir::toNativeSeparators(settings.configPath))
        && write("External/ZTruncationValue", QString::number(settings.zTruncationValue, 'f', 6))
        && write("External/ResampleStepMm", QString::number(settings.resampleStepMm, 'f', 6))
        && write("External/UseWeldedStartTruncation", settings.sdkUseWeldedStartTruncation ? "1" : "0")
        && write("FeaturePoint/SlopeConsistentCornerFit", settings.slopeConsistentCornerFit ? "1" : "0")
        && write("FeaturePoint/ExportFitDebugCloud", settings.exportFitDebugCloud ? "1" : "0")
        && write("FeaturePoint/ExportWorkpieceFrameDebug", settings.exportWorkpieceFrameDebug ? "1" : "0")
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
        && write("Fit/SdkBasePresmoothEnable", settings.sdkBasePresmoothEnable ? "1" : "0")
        && write("Fit/SdkBasePresmoothWindowMm", QString::number(settings.sdkBasePresmoothWindowMm, 'f', 6))
        && write("Fit/SdkBasePresmoothEdgeMm", QString::number(settings.sdkBasePresmoothEdgeMm, 'f', 6))
        && write("Fit/AzimuthTurnThresholdDeg", QString::number(settings.fitAzimuthTurnThresholdDeg, 'f', 6))
        && write("Fit/AzimuthHeadingWindow", QString::number(settings.fitAzimuthHeadingWindow))
        && write("Fit/AzimuthNmsSpanMm", QString::number(settings.fitAzimuthNmsSpanMm, 'f', 6))
        && write("Fit/AzimuthStraightenResidualMm", QString::number(settings.fitAzimuthStraightenResidualMm, 'f', 6))
        && write("Fit/CornerRefineEnable", settings.fitCornerRefineEnable ? "1" : "0")
        && write("Fit/AzimuthRefineFloorMm", QString::number(settings.fitAzimuthRefineFloorMm, 'f', 6))
        && write("Fit/CornerRefineOneSidedPct", QString::number(settings.fitCornerRefineOneSidedPct, 'f', 6))
        && write("Fit/CornerRefineMidMultiple", QString::number(settings.fitCornerRefineMidMultiple, 'f', 6))
        && write("Fit/CornerRefineEndFracPct", QString::number(settings.fitCornerRefineEndFracPct, 'f', 6))
        && write("Fit/CornerPatternRefitEnable", settings.fitCornerPatternRefitEnable ? "1" : "0")
        && write("Fit/CornerPlatformMinSegPoints", QString::number(settings.fitCornerPlatformMinSegPoints))
        && write("Fit/EnableLapMisalignmentSplit", settings.enableLapMisalignmentSplit ? "1" : "0")
        && write("Fit/LapStepHeightThresholdMm", QString::number(settings.lapStepHeightThresholdMm, 'f', 6))
        && write("Fit/LapStepStationWindowMm", QString::number(settings.lapStepStationWindowMm, 'f', 6))
        && write("Fit/LapStepSideFlatnessMm", QString::number(settings.lapStepSideFlatnessMm, 'f', 6))
        && write("Fit/LapStepPlatformSlopeMax", QString::number(settings.lapStepPlatformSlopeMax, 'f', 6))
        && write("Fit/EdgeTruncateEnable", settings.fitEdgeTruncateEnable ? "1" : "0")
        && write("Fit/TruncateHeadMm", QString::number(settings.fitTruncateHeadMm, 'f', 6))
        && write("Fit/TruncateTailMm", QString::number(settings.fitTruncateTailMm, 'f', 6))
        && write("Fit/EndPeriodRecoverEnable", settings.fitEndPeriodRecoverEnable ? "1" : "0")
        && write("Fit/EndPeriodRatioThreshold", QString::number(settings.fitEndPeriodRatioThreshold, 'f', 6))
        && write("Fit/EndPeriodMinBendDeg", QString::number(settings.fitEndPeriodMinBendDeg, 'f', 6))
        && write("Fit/EndPeriodMergeFrac", QString::number(settings.fitEndPeriodMergeFrac, 'f', 6))
        && write("Fit/SameTypeShortMergeEnable", settings.fitSameTypeShortMergeEnable ? "1" : "0")
        && write("Fit/SameTypeMinReferenceSegments", QString::number(settings.fitSameTypeMinReferenceSegments))
        && write("Fit/SameTypeFlatSlope", QString::number(settings.fitSameTypeFlatSlope, 'f', 6))
        && write("Fit/PlatformSnapEnable", settings.fitPlatformSnapEnable ? "1" : "0")
        && write("Fit/PlatformSnapFlatSlope", QString::number(settings.fitPlatformSnapFlatSlope, 'f', 6))
        && write("Fit/PlatformSnapMinFrac", QString::number(settings.fitPlatformSnapMinFrac, 'f', 6))
        && write("CloudProjection/StationWindowMm", QString::number(settings.projectionStationWindowMm, 'f', 6))
        && write("CloudProjection/TransverseWindowMm", QString::number(settings.projectionTransverseWindowMm, 'f', 6))
        && write("CloudProjection/ZBandBelowMm", QString::number(settings.projectionZBandBelowMm, 'f', 6))
        && write("CloudProjection/ZBandAboveMm", QString::number(settings.projectionZBandAboveMm, 'f', 6))
        && write("CloudProjection/MaxCandidatePerSeed", QString::number(settings.projectionMaxCandidatePerSeed))
        && write("CloudProjection/LayerLowPercent", QString::number(settings.projectionLayerLowPercent, 'f', 6))
        && write("CloudProjection/LayerHighPercent", QString::number(settings.projectionLayerHighPercent, 'f', 6))
        && write("CloudProjection/SmoothRadius", QString::number(settings.projectionSmoothRadius))
        && write("Validation/ProfileVersion", QString::number(CURRENT_VALIDATION_PROFILE_VERSION))
        && write("Validation/Policy", ValidationPolicyConfigValue(normalizedSettings.validationPolicy))
        && write("Validation/CoverageEnabled", normalizedSettings.validationCoverageEnabled ? "1" : "0")
        && write("Validation/MinFinitePointCount", QString::number(normalizedSettings.validationMinFinitePointCount))
        && write("Validation/MinProjectedSpanMm", QString::number(normalizedSettings.validationMinProjectedSpanMm, 'f', 6))
        && write("Validation/ContinuityEnabled", normalizedSettings.validationContinuityEnabled ? "1" : "0")
        && write("Validation/MinStationCoverageRatio", QString::number(normalizedSettings.validationMinStationCoverageRatio, 'f', 6))
        && write("Validation/MinLongestContinuousRatio", QString::number(normalizedSettings.validationMinLongestContinuousRatio, 'f', 6))
        && write("Validation/DenoiseRatioEnabled", normalizedSettings.validationDenoiseRatioEnabled ? "1" : "0")
        && write("Validation/MaxRejectedRatio", QString::number(normalizedSettings.validationMaxRejectedRatio, 'f', 6))
        && write("Validation/ResidualEnabled", normalizedSettings.validationResidualEnabled ? "1" : "0")
        && write("Validation/MaxMedianResidualMm", QString::number(normalizedSettings.validationMaxMedianResidualMm, 'f', 6))
        && write("Validation/MaxP95ResidualMm", QString::number(normalizedSettings.validationMaxP95ResidualMm, 'f', 6))
        && write("Validation/ResidualInlierThresholdMm", QString::number(normalizedSettings.validationResidualInlierThresholdMm, 'f', 6))
        && write("Validation/MinResidualInlierRatio", QString::number(normalizedSettings.validationMinResidualInlierRatio, 'f', 6))
        && write("Validation/KeyPointEnabled", normalizedSettings.validationKeyPointEnabled ? "1" : "0")
        && write("Validation/MinKeyPointCount", QString::number(normalizedSettings.validationMinKeyPointCount))
        && write("Validation/MinCornerCount", QString::number(normalizedSettings.validationMinCornerCount))
        && write("Validation/MinSegmentLengthMm", QString::number(normalizedSettings.validationMinSegmentLengthMm, 'f', 6))
        && write("Validation/OutputEnabled", normalizedSettings.validationOutputEnabled ? "1" : "0")
        && write("Validation/MinOutputPointCount", QString::number(normalizedSettings.validationMinOutputPointCount))
        && write("Validation/MinOutputLengthRatio", QString::number(normalizedSettings.validationMinOutputLengthRatio, 'f', 6))
        && write("Validation/SegmentHardLimitsEnabled", normalizedSettings.validationSegmentHardLimitsEnabled ? "1" : "0")
        && write("Validation/MinNonLapSegmentHardMm", QString::number(normalizedSettings.validationMinNonLapSegmentHardMm, 'f', 6))
        && write("Validation/MinLapOrEndpointSegmentHardMm", QString::number(normalizedSettings.validationMinLapOrEndpointSegmentHardMm, 'f', 6))
        && write("Validation/FinalTrajectoryStepEnabled", normalizedSettings.validationFinalTrajectoryStepEnabled ? "1" : "0")
        && write("Validation/MaxFinalPositionStepMm", QString::number(normalizedSettings.validationMaxFinalPositionStepMm, 'f', 6))
        && write("Validation/MaxFinalControllerEulerStepDeg", QString::number(normalizedSettings.validationMaxFinalControllerEulerStepDeg, 'f', 6))
        && write("Validation/MaxFinalPhysicalOrientationStepDeg", QString::number(normalizedSettings.validationMaxFinalPhysicalOrientationStepDeg, 'f', 6))
        && write("Validation/FinalLengthBindingEnabled", normalizedSettings.validationFinalLengthBindingEnabled ? "1" : "0")
        && write("Validation/MinFinalToPreCompLengthRatio", QString::number(normalizedSettings.validationMinFinalToPreCompLengthRatio, 'f', 6))
        && write("Validation/MaxFinalToPreCompLengthRatio", QString::number(normalizedSettings.validationMaxFinalToPreCompLengthRatio, 'f', 6))
        && write("Validation/FinalTopologyBindingEnabled", normalizedSettings.validationFinalTopologyBindingEnabled ? "1" : "0")
        && write("Validation/MinFinalMatchedArcRatio", QString::number(normalizedSettings.validationMinFinalMatchedArcRatio, 'f', 6))
        && write("Validation/MinFinalSourceUniqueCoverageRatio", QString::number(normalizedSettings.validationMinFinalSourceUniqueCoverageRatio, 'f', 6))
        && write("Validation/MinFinalSourceArcSpanRatio", QString::number(normalizedSettings.validationMinFinalSourceArcSpanRatio, 'f', 6))
        && write("Validation/FinalSourceBindingEnabled", normalizedSettings.validationFinalSourceBindingEnabled ? "1" : "0")
        && write("Validation/MaxFinalSourceDisplacementMm", QString::number(normalizedSettings.validationMaxFinalSourceDisplacementMm, 'f', 6))
        && write("Validation/MaxFinalSourcePhysicalOrientationDeltaDeg", QString::number(normalizedSettings.validationMaxFinalSourcePhysicalOrientationDeltaDeg, 'f', 6))
        && write("Validation/FinalSemanticIntegrityEnabled", normalizedSettings.validationFinalSemanticIntegrityEnabled ? "1" : "0")
        && write("SafetyGates/ProofIntegrityEnabled", settings.safetyGateProofIntegrityEnabled ? "1" : "0")
        && write("SafetyGates/ProductionPurposeEnabled", settings.safetyGateProductionPurposeEnabled ? "1" : "0")
        && write("SafetyGates/RobotNameBindingEnabled", settings.safetyGateRobotNameBindingEnabled ? "1" : "0")
        && write("SafetyGates/CaseBindingEnabled", settings.safetyGateCaseBindingEnabled ? "1" : "0")
        && write("SafetyGates/EndpointBindingEnabled", settings.safetyGateEndpointBindingEnabled ? "1" : "0")
        && write("SafetyGates/CameraHandEyeBindingEnabled", settings.safetyGateCameraHandEyeBindingEnabled ? "1" : "0")
        && write("SafetyGates/FreshnessEnabled", settings.safetyGateFreshnessEnabled ? "1" : "0")
        && write("SafetyGates/PolicySnapshotEnabled", settings.safetyGatePolicySnapshotEnabled ? "1" : "0")
        && write("SafetyGates/InputEvidenceEnabled", settings.safetyGateInputEvidenceEnabled ? "1" : "0")
        && write("SafetyGates/AuthorizedPoseIdentityEnabled", settings.safetyGateAuthorizedPoseIdentityEnabled ? "1" : "0")
        && write("SafetyGates/TrajectoryStructureEnabled", settings.safetyGateTrajectoryStructureEnabled ? "1" : "0")
        && write("SafetyGates/MotionPrecheckEnabled", settings.safetyGateMotionPrecheckEnabled ? "1" : "0")
        && write("SafetyGates/BehaviorVersion", QString::number(CURRENT_SAFETY_GATE_BEHAVIOR_VERSION));
    const bool ok = valuesPrepared && ConfigDatabase::WriteScopedSettings(
        QStringLiteral("global"), QString(), SETTINGS_GROUP, pendingValues);
    if (!ok && error != nullptr)
    {
        *error = QStringLiteral("原子写入点云处理配置失败，数据库已回滚，未留下混合版本。");
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

QString PointCloudProcessingConfig::ValidationPolicyConfigValue(ValidationPolicy policy)
{
    return policy == ValidationPolicy::Audit ? "Audit" : "Enforce";
}

PointCloudProcessingConfig::ValidationPolicy PointCloudProcessingConfig::ValidationPolicyFromConfigValue(
    const QString& value)
{
    const QString normalized = value.trimmed().toLower();
    return normalized == "audit" || normalized == "0"
        ? ValidationPolicy::Audit
        : ValidationPolicy::Enforce;
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
