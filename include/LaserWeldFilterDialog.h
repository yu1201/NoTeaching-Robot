#pragma once

#include "PointCloudProcessingConfig.h"

#include <QDialog>

#include <functional>

class QComboBox;
class QDoubleSpinBox;
class QCheckBox;
class QGroupBox;
class QLineEdit;
class QPlainTextEdit;
class QPushButton;
class QSpinBox;
class QTabWidget;

class LaserWeldFilterDialog : public QDialog
{
public:
    explicit LaserWeldFilterDialog(
        std::function<bool()> liveSessionGuard,
        QWidget* parent = nullptr);

private:
    void BuildUi();
    void ApplyStyle();
    void LoadSettings();
    bool SaveSettings(QString* error = nullptr) const;
    void BrowseExternalLibraryDir();
    void BrowseExternalConfigFile();
    void LoadExternalAlgorithmConfig();
    void SaveExternalAlgorithmConfig(QString* error = nullptr) const;
    void ApplyMethodEnableState();
    void AppendLog(const QString& text);
    PointCloudProcessingConfig::Mode CurrentProcessingMode() const;
    PointCloudProcessingConfig::FeaturePointStrategy CurrentFeaturePointStrategy() const;

private:
    std::function<bool()> m_liveSessionGuard;
    QComboBox* m_pProcessingModeCombo = nullptr;
    QTabWidget* m_pAlgorithmTabWidget = nullptr;
    QLineEdit* m_pExternalLibraryDirEdit = nullptr;
    QLineEdit* m_pExternalConfigPathEdit = nullptr;
    QDoubleSpinBox* m_pExternalZTruncationSpin = nullptr;
    QDoubleSpinBox* m_pExternalResampleStepSpin = nullptr;
    QCheckBox* m_pSdkWeldedStartCheck = nullptr;
    QCheckBox* m_pCloudUprightCheck = nullptr;
    QDoubleSpinBox* m_pCloudPlateThicknessSpin = nullptr;
    QDoubleSpinBox* m_pCloudRemoveFloorZSpin = nullptr;
    QSpinBox* m_pCloudThreadNumberSpin = nullptr;
    QDoubleSpinBox* m_pCloudMoveDistanceSpin = nullptr;
    QCheckBox* m_pCloudDebugLogCheck = nullptr;
    QLineEdit* m_pCloudLogPathEdit = nullptr;
    QDoubleSpinBox* m_pCloudPlaneThresholdSpin = nullptr;
    QDoubleSpinBox* m_pCloudMergeLinesAngleSpin = nullptr;
    QDoubleSpinBox* m_pCloudMergeLinesDistanceSpin = nullptr;
    QDoubleSpinBox* m_pCloudClusterToleranceSpin = nullptr;
    QCheckBox* m_pCloudClusterCheck = nullptr;
    QCheckBox* m_pCloudRemoveNoiseCheck = nullptr;  // is_remove_noise：SDK 库内置去噪（20260617 版新增）
    QCheckBox* m_pCloudSampleCheck = nullptr;       // is_sample：SDK 库内降采样找平面（20260624 版新增，提速/降崩溃）
    QDoubleSpinBox* m_pCloudSampleSizeSpin = nullptr; // sample_size：降采样体素大小
    QDoubleSpinBox* m_pCloudAboveZSpin = nullptr;     // above_z：抬高值
    QSpinBox* m_pCloudPixelNumSpin = nullptr;         // pixel_num：骨架短毛刺/短支路清理长度（像素）
    QSpinBox* m_pCloudDiscreteValueSpin = nullptr;
    QSpinBox* m_pCloudDilateValueSpin = nullptr;
    QSpinBox* m_pCloudErodeValueSpin = nullptr;
    QDoubleSpinBox* m_pCloudLinesDisThresholdSpin = nullptr;
    QDoubleSpinBox* m_pCloudLineLengthSpin = nullptr;
    QGroupBox* m_pSdkParamGroup = nullptr;
    QGroupBox* m_pSdkInnerGroup = nullptr;
    QGroupBox* m_pProjectionGroup = nullptr;
    QDoubleSpinBox* m_pProjStationWindowSpin = nullptr;
    QDoubleSpinBox* m_pProjTransverseWindowSpin = nullptr;
    QDoubleSpinBox* m_pProjZBandBelowSpin = nullptr;
    QDoubleSpinBox* m_pProjZBandAboveSpin = nullptr;
    QDoubleSpinBox* m_pProjLayerLowSpin = nullptr;
    QDoubleSpinBox* m_pProjLayerHighSpin = nullptr;
    QSpinBox* m_pProjMaxCandidateSpin = nullptr;
    QSpinBox* m_pProjSmoothRadiusSpin = nullptr;
    QComboBox* m_pFeaturePointStrategyCombo = nullptr;
    QComboBox* m_pAxisCombo = nullptr;
    QDoubleSpinBox* m_pZThresholdSpin = nullptr;  // 兼容旧配置，当前算法未消费，不再显示
    QDoubleSpinBox* m_pZJumpThresholdSpin = nullptr;
    QDoubleSpinBox* m_pZContinuityThresholdSpin = nullptr;
    QDoubleSpinBox* m_pSegmentBreakDistanceSpin = nullptr;
    QCheckBox* m_pKeepLongestSegmentCheck = nullptr;  // 兼容旧配置，当前算法未消费，不再显示
    QDoubleSpinBox* m_pStepSpin = nullptr;
    QDoubleSpinBox* m_pWindowSpin = nullptr;
    QDoubleSpinBox* m_pPiecewiseToleranceSpin = nullptr;
    QSpinBox* m_pPiecewiseMinSegmentSpin = nullptr;
    QSpinBox* m_pMinPointSpin = nullptr;
    QSpinBox* m_pSmoothRadiusSpin = nullptr;
    QDoubleSpinBox* m_pAzimuthTurnThresholdSpin = nullptr;       // 拐点转角阈值(度)
    QSpinBox* m_pAzimuthHeadingWindowSpin = nullptr;             // 拐点拟合窗口(点)
    QDoubleSpinBox* m_pAzimuthNmsSpanSpin = nullptr;             // 拐点NMS弧长(mm)
    QDoubleSpinBox* m_pAzimuthStraightenResidualSpin = nullptr;  // 直线化兜底残差(mm)
    QCheckBox* m_pCornerRefineEnableCheck = nullptr;            // 端区补拐点总开关
    QDoubleSpinBox* m_pAzimuthRefineFloorSpin = nullptr;         // 端区细化地板(mm)
    QDoubleSpinBox* m_pCornerRefineOneSidedSpin = nullptr;       // 单侧弓出门(%)
    QDoubleSpinBox* m_pCornerRefineMidMultipleSpin = nullptr;    // 中段地板倍数
    QDoubleSpinBox* m_pCornerRefineEndFracSpin = nullptr;        // 端区占比(%)
    QCheckBox* m_pCornerPatternRefitCheck = nullptr;            // 平台重算(II/OO结构约束)总开关
    QSpinBox* m_pCornerPlatformMinSegSpin = nullptr;             // 三段拟合每段最少点数
    QCheckBox* m_pLapSplitCheck = nullptr;                       // 启用搭接错位检测
    QDoubleSpinBox* m_pLapStepHeightSpin = nullptr;             // 错位阶跃阈值(mm)
    QDoubleSpinBox* m_pLapStepStationSpin = nullptr;            // 错位主轴窗口(mm)
    QDoubleSpinBox* m_pLapStepFlatnessSpin = nullptr;          // 平台残差上限(mm)
    QDoubleSpinBox* m_pLapStepSlopeSpin = nullptr;            // 平台斜率上限(mm/mm)
    QCheckBox* m_pSdkBasePresmoothCheck = nullptr;            // SdkBase 拟合前双边预平滑(去锯齿)总开关
    QDoubleSpinBox* m_pSdkBasePresmoothWindowSpin = nullptr;  // 预平滑空间窗 σs(mm)
    QDoubleSpinBox* m_pSdkBasePresmoothEdgeSpin = nullptr;    // 预平滑保边阈值 σr(mm)
    QCheckBox* m_pEdgeTruncateCheck = nullptr;               // 基础焊道首尾截断总开关(②③④通用)
    QDoubleSpinBox* m_pTruncateHeadSpin = nullptr;           // 开头截断弧长(mm)
    QDoubleSpinBox* m_pTruncateTailSpin = nullptr;           // 结尾截断弧长(mm)
    QCheckBox* m_pEndPeriodRecoverCheck = nullptr;           // 端区周期补拐点总开关(②③④通用)
    QDoubleSpinBox* m_pEndPeriodRatioSpin = nullptr;         // 端段长/周期 判漏阈值
    QDoubleSpinBox* m_pEndPeriodMinBendSpin = nullptr;       // 补点最小弯折角(度)
    QCheckBox* m_pSameTypeShortMergeCheck = nullptr;         // 同类短段多余拐点合并总开关(②③④通用)
    QDoubleSpinBox* m_pEndPeriodMergeSpin = nullptr;         // 删错:候选短段/同类段中位 阈值
    QSpinBox* m_pSameTypeMinReferenceSpin = nullptr;         // 每类最少完整参考段数
    QDoubleSpinBox* m_pSameTypeFlatSlopeSpin = nullptr;      // 短段复核用坡/平台斜率分界
    QCheckBox* m_pPlatformSnapCheck = nullptr;               // 按平台边界重定拐点总开关(②③④通用)
    QDoubleSpinBox* m_pPlatformSnapFlatSlopeSpin = nullptr;  // 平台判定斜率上限
    QDoubleSpinBox* m_pPlatformSnapMinFracSpin = nullptr;    // 平台最小长度/周期
    QCheckBox* m_pSlopeConsistentCornerFitCheck = nullptr;
    QCheckBox* m_pExportFitDebugCloudCheck = nullptr;
    QCheckBox* m_pExportWorkpieceFrameDebugCheck = nullptr;
    QCheckBox* m_pValidationAuditOnlyCheck = nullptr;
    QCheckBox* m_pValidationCoverageCheck = nullptr;
    QSpinBox* m_pValidationMinFinitePointSpin = nullptr;
    QDoubleSpinBox* m_pValidationMinProjectedSpanSpin = nullptr;
    QCheckBox* m_pValidationContinuityCheck = nullptr;
    QDoubleSpinBox* m_pValidationMinStationCoverageSpin = nullptr;
    QDoubleSpinBox* m_pValidationMinLongestContinuousSpin = nullptr;
    QCheckBox* m_pValidationDenoiseRatioCheck = nullptr;
    QDoubleSpinBox* m_pValidationMaxRejectedRatioSpin = nullptr;
    QCheckBox* m_pValidationResidualCheck = nullptr;
    QDoubleSpinBox* m_pValidationMaxMedianResidualSpin = nullptr;
    QDoubleSpinBox* m_pValidationMaxP95ResidualSpin = nullptr;
    QDoubleSpinBox* m_pValidationResidualInlierThresholdSpin = nullptr;
    QDoubleSpinBox* m_pValidationMinResidualInlierRatioSpin = nullptr;
    QCheckBox* m_pValidationKeyPointCheck = nullptr;
    QSpinBox* m_pValidationMinKeyPointSpin = nullptr;
    QSpinBox* m_pValidationMinCornerSpin = nullptr;
    QDoubleSpinBox* m_pValidationMinSegmentLengthSpin = nullptr;
    QCheckBox* m_pValidationOutputCheck = nullptr;
    QSpinBox* m_pValidationMinOutputPointSpin = nullptr;
    QDoubleSpinBox* m_pValidationMinOutputLengthRatioSpin = nullptr;
    QCheckBox* m_pValidationSegmentHardLimitsCheck = nullptr;
    QDoubleSpinBox* m_pValidationMinNonLapSegmentHardSpin = nullptr;
    QDoubleSpinBox* m_pValidationMinLapOrEndpointSegmentHardSpin = nullptr;
    QCheckBox* m_pValidationFinalTrajectoryStepCheck = nullptr;
    QDoubleSpinBox* m_pValidationMaxFinalPositionStepSpin = nullptr;
    QDoubleSpinBox* m_pValidationMaxFinalControllerEulerStepSpin = nullptr;
    QDoubleSpinBox* m_pValidationMaxFinalPhysicalOrientationStepSpin = nullptr;
    QCheckBox* m_pValidationFinalLengthBindingCheck = nullptr;
    QDoubleSpinBox* m_pValidationMinFinalLengthRatioSpin = nullptr;
    QDoubleSpinBox* m_pValidationMaxFinalLengthRatioSpin = nullptr;
    QCheckBox* m_pValidationFinalTopologyBindingCheck = nullptr;
    QDoubleSpinBox* m_pValidationMinFinalMatchedArcRatioSpin = nullptr;
    QDoubleSpinBox* m_pValidationMinFinalSourceUniqueCoverageRatioSpin = nullptr;
    QDoubleSpinBox* m_pValidationMinFinalSourceArcSpanRatioSpin = nullptr;
    QCheckBox* m_pValidationFinalSourceBindingCheck = nullptr;
    QDoubleSpinBox* m_pValidationMaxFinalSourceDisplacementSpin = nullptr;
    QDoubleSpinBox* m_pValidationMaxFinalSourcePhysicalOrientationDeltaSpin = nullptr;
    QCheckBox* m_pValidationFinalSemanticIntegrityCheck = nullptr;
    QPlainTextEdit* m_pLogText = nullptr;
};
