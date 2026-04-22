#pragma once

#include "RobotCalculation.h"

#include <QDialog>

class QComboBox;
class QDoubleSpinBox;
class QCheckBox;
class QLineEdit;
class QPlainTextEdit;
class QPushButton;
class QSpinBox;

class LaserWeldFilterDialog : public QDialog
{
public:
    explicit LaserWeldFilterDialog(QWidget* parent = nullptr);

private:
    void BuildUi();
    void ApplyStyle();
    void LoadSettings();
    void SaveSettings() const;
    void BrowseInputFile();
    void BrowseOutputFile();
    void UpdateSuggestedOutputPath();
    void RunFilter();
    void AppendLog(const QString& text);
    RobotCalculation::LowerWeldFilterParams CurrentParams() const;

private:
    QLineEdit* m_pInputPathEdit = nullptr;
    QLineEdit* m_pOutputPathEdit = nullptr;
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
    QPushButton* m_pRunButton = nullptr;
    QPlainTextEdit* m_pLogText = nullptr;
};
