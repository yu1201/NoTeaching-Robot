#pragma once

#include "PointCloudProcessingConfig.h"

#include <QDialog>

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
    explicit LaserWeldFilterDialog(QWidget* parent = nullptr);

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
    QDoubleSpinBox* m_pZThresholdSpin = nullptr;
    QDoubleSpinBox* m_pZJumpThresholdSpin = nullptr;
    QDoubleSpinBox* m_pZContinuityThresholdSpin = nullptr;
    QDoubleSpinBox* m_pSegmentBreakDistanceSpin = nullptr;
    QCheckBox* m_pKeepLongestSegmentCheck = nullptr;
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
    QCheckBox* m_pLapSplitCheck = nullptr;                       // 启用搭接错位检测
    QDoubleSpinBox* m_pLapStepHeightSpin = nullptr;             // 错位阶跃阈值(mm)
    QDoubleSpinBox* m_pLapStepStationSpin = nullptr;            // 错位主轴窗口(mm)
    QDoubleSpinBox* m_pLapStepFlatnessSpin = nullptr;          // 平台残差上限(mm)
    QDoubleSpinBox* m_pLapStepSlopeSpin = nullptr;            // 平台斜率上限(mm/mm)
    QCheckBox* m_pSlopeConsistentCornerFitCheck = nullptr;
    QCheckBox* m_pExportFitDebugCloudCheck = nullptr;
    QCheckBox* m_pExportWorkpieceFrameDebugCheck = nullptr;
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
    QPlainTextEdit* m_pLogText = nullptr;
};
