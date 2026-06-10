#pragma once

#include "PointCloudProcessingConfig.h"
#include "RobotCalculation.h"

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
    QCheckBox* m_pExternalFallbackCheck = nullptr;
    QComboBox* m_pCloudAxisCombo = nullptr;
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
    QSpinBox* m_pCloudDiscreteValueSpin = nullptr;
    QSpinBox* m_pCloudDilateValueSpin = nullptr;
    QSpinBox* m_pCloudErodeValueSpin = nullptr;
    QDoubleSpinBox* m_pCloudLinesDisThresholdSpin = nullptr;
    QDoubleSpinBox* m_pCloudLineLengthSpin = nullptr;
    QGroupBox* m_pSdkParamGroup = nullptr;
    QGroupBox* m_pSdkInnerGroup = nullptr;
    QGroupBox* m_pCloudAlgoGroup = nullptr;
    QComboBox* m_pFeaturePointStrategyCombo = nullptr;
    QComboBox* m_pAxisCombo = nullptr;
    QComboBox* m_pFitModeCombo = nullptr;
    QDoubleSpinBox* m_pZThresholdSpin = nullptr;
    QDoubleSpinBox* m_pZJumpThresholdSpin = nullptr;
    QDoubleSpinBox* m_pZContinuityThresholdSpin = nullptr;
    QDoubleSpinBox* m_pSegmentBreakDistanceSpin = nullptr;
    QCheckBox* m_pKeepLongestSegmentCheck = nullptr;
    QDoubleSpinBox* m_pStepSpin = nullptr;
    QDoubleSpinBox* m_pWindowSpin = nullptr;
    QSpinBox* m_pLineFitTrimSpin = nullptr;
    QDoubleSpinBox* m_pPiecewiseToleranceSpin = nullptr;
    QSpinBox* m_pPiecewiseMinSegmentSpin = nullptr;
    QSpinBox* m_pMinPointSpin = nullptr;
    QSpinBox* m_pSmoothRadiusSpin = nullptr;
    QCheckBox* m_pSlopeConsistentCornerFitCheck = nullptr;
    QCheckBox* m_pExportFitDebugCloudCheck = nullptr;
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
