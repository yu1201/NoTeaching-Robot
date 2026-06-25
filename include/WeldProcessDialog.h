#pragma once

#include <QDialog>
#include <QString>

#include "WeldProcessFile.h"

QT_BEGIN_NAMESPACE
namespace Ui
{
class WeldProcessDialog;
}
QT_END_NAMESPACE

class QDoubleSpinBox;
class QCloseEvent;
class QCheckBox;
class QComboBox;
class ContralUnit;
class QFormLayout;
class QGridLayout;
class QGroupBox;
class QLineEdit;
class QListWidget;
class QPushButton;
class QSpinBox;
class QStackedWidget;

class WeldProcessDialog : public QDialog
{
    Q_OBJECT

public:
    explicit WeldProcessDialog(const T_CONTRAL_UNIT& unitInfo, ContralUnit* pContralUnit, QWidget* parent = nullptr);
    ~WeldProcessDialog();

protected:
    void showEvent(QShowEvent* event) override;

private slots:
    void ReloadData();
    bool SaveData();
    void OnWeldSelectionChanged(int row);
    void OnBeadSelectionChanged(int row);
    void OnRobotChanged(int index);
    void OnWeldGroupsReordered();
    void AddWeldGroup();
    void RemoveWeldGroup();
    void AddBead();
    void RemoveBead();

private:
    void closeEvent(QCloseEvent* event) override;
    void BuildEditorUi();
    void ApplyDialogStyle();
    void LoadToUi(int preferredGroupRow = -1, int preferredBeadRow = -1);
    void UpdateWeaveShapeAvailability();   // 摆动实现=pointwise 时：摆弧形状只留正弦/三角/纵向往复可选，L摆及其它原生波形置灰
    void PopulateWeldList(int preferredGroupRow);
    void PopulateBeadList(int preferredBeadRow);
    void ApplySelectionToUi(int weldIndex);
    int GroupCount() const;
    QString BuildWeldDisplayName(int index, const T_WELD_PARA& item) const;
    QString BuildGroupKey(const T_WELD_PARA& item) const;
    std::vector<int> GetGroupIndices(int groupRow) const;
    int CurrentWeldIndex() const;
    QLineEdit* AddTextField(QGridLayout* layout, const QString& label);
    QDoubleSpinBox* AddDoubleField(QGridLayout* layout, const QString& label, int decimals = 3);
    QDoubleSpinBox* AddDoubleFieldWithUnit(QGridLayout* layout, const QString& label, const QString& unit, int decimals = 3);
    QDoubleSpinBox* AddEnabledDoubleField(QGridLayout* layout, const QString& label, QCheckBox** enableCheck, int decimals = 3);
    QDoubleSpinBox* AddEnabledDoubleFieldWithUnit(QGridLayout* layout, const QString& label, QCheckBox** enableCheck, const QString& unit, int decimals = 3);
    QSpinBox* AddIntField(QGridLayout* layout, const QString& label, int minimum = -999999, int maximum = 999999);
    QSpinBox* AddIntFieldWithUnit(QGridLayout* layout, const QString& label, const QString& unit, int minimum = -999999, int maximum = 999999);
    bool CollectWeaveFromUi(T_WeaveDate& out) const;
    bool CollectWeldFromUi(T_WELD_PARA& out) const;
    void ShowError(const QString& message) const;
    QLineEdit* AddSingleTextField(QFormLayout* layout, const QString& label);
    QDoubleSpinBox* AddSingleDoubleField(QFormLayout* layout, const QString& label, int decimals = 3);
    QDoubleSpinBox* AddSingleDoubleFieldWithUnit(QFormLayout* layout, const QString& label, const QString& unit, int decimals = 3);
    QSpinBox* AddSingleIntField(QFormLayout* layout, const QString& label, int minimum = -999999, int maximum = 999999);
    QSpinBox* AddSingleIntFieldWithUnit(QFormLayout* layout, const QString& label, const QString& unit, int minimum = -999999, int maximum = 999999);
    QComboBox* AddSingleComboField(QFormLayout* layout, const QString& label);
    QCheckBox* AddSingleSwitchField(QFormLayout* layout, const QString& label);
    void UpdateFeaturePageEnabled();
    bool HasUnsavedChanges() const;
    QString BuildSnapshot() const;
    void MarkCleanSnapshot();

private:
    Ui::WeldProcessDialog* ui = nullptr;
    WeldProcessFile m_file;
    bool m_isLoading = false;

    QListWidget* m_weldListWidget = nullptr;
    QListWidget* m_beadListWidget = nullptr;
    QPushButton* m_addWeldButton = nullptr;
    QPushButton* m_removeWeldButton = nullptr;
    QPushButton* m_addBeadButton = nullptr;
    QPushButton* m_removeBeadButton = nullptr;
    QPushButton* m_normalPageButton = nullptr;
    QPushButton* m_weavePageButton = nullptr;
    QPushButton* m_trackPageButton = nullptr;
    QPushButton* m_wrapParamButton = nullptr;
    QPushButton* m_cornerTransitionParamButton = nullptr;
    QStackedWidget* m_editorStack = nullptr;
    QStackedWidget* m_specialParamStack = nullptr;

    QLineEdit* m_workPeaceEdit = nullptr;
    QLineEdit* m_weldTypeEdit = nullptr;
    QComboBox* m_arcModeCombo = nullptr;
    QComboBox* m_weldPostureCombo = nullptr;
    QComboBox* m_dynamicModeCombo = nullptr;   // 动态特性(WLin DYNAMIC)：0=NULL 1=ntdyn0，复用工艺 nWeldMethod 存储
    QDoubleSpinBox* m_weldOverlapSpin = nullptr;
    QDoubleSpinBox* m_weldAngleSizeSpin = nullptr;
    QCheckBox* m_weaveEnableCheck = nullptr;
    QCheckBox* m_trackEnableCheck = nullptr;

    QDoubleSpinBox* m_startArcCurrentSpin = nullptr;
    QDoubleSpinBox* m_startArcVoltageSpin = nullptr;
    QDoubleSpinBox* m_startWaitTimeSpin = nullptr;

    QDoubleSpinBox* m_trackCurrentSpin = nullptr;
    QDoubleSpinBox* m_trackVoltageSpin = nullptr;
    QDoubleSpinBox* m_weldVelocitySpin = nullptr;
    QDoubleSpinBox* m_crosswiseOffsetSpin = nullptr;
    QDoubleSpinBox* m_verticalOffsetSpin = nullptr;
    QDoubleSpinBox* m_weldAngleSpin = nullptr;
    QDoubleSpinBox* m_weldDipAngleSpin = nullptr;

    QDoubleSpinBox* m_stopArcCurrentSpin = nullptr;
    QDoubleSpinBox* m_stopArcVoltageSpin = nullptr;
    QDoubleSpinBox* m_stopWaitTimeSpin = nullptr;

    QDoubleSpinBox* m_wrapCurrent1Spin = nullptr;
    QDoubleSpinBox* m_wrapVoltage1Spin = nullptr;
    QDoubleSpinBox* m_wrapWaitTime1Spin = nullptr;
    QDoubleSpinBox* m_wrapCurrent2Spin = nullptr;
    QDoubleSpinBox* m_wrapVoltage2Spin = nullptr;
    QDoubleSpinBox* m_wrapWaitTime2Spin = nullptr;
    QDoubleSpinBox* m_wrapCurrent3Spin = nullptr;
    QDoubleSpinBox* m_wrapVoltage3Spin = nullptr;
    QDoubleSpinBox* m_wrapWaitTime3Spin = nullptr;
    QCheckBox* m_wrapCurrent1EnableCheck = nullptr;
    QCheckBox* m_wrapVoltage1EnableCheck = nullptr;
    QCheckBox* m_wrapWaitTime1EnableCheck = nullptr;
    QCheckBox* m_wrapCurrent2EnableCheck = nullptr;
    QCheckBox* m_wrapVoltage2EnableCheck = nullptr;
    QCheckBox* m_wrapWaitTime2EnableCheck = nullptr;
    QCheckBox* m_wrapCurrent3EnableCheck = nullptr;
    QCheckBox* m_wrapVoltage3EnableCheck = nullptr;
    QCheckBox* m_wrapWaitTime3EnableCheck = nullptr;

    QDoubleSpinBox* m_cornerTransitionRadiusSpin = nullptr;
    QDoubleSpinBox* m_cornerTransitionSpeedSpin = nullptr;
    QDoubleSpinBox* m_cornerTransitionCurrentSpin = nullptr;
    QDoubleSpinBox* m_cornerTransitionVoltageSpin = nullptr;
    QCheckBox* m_cornerTransitionRadiusEnableCheck = nullptr;
    QCheckBox* m_cornerTransitionSpeedEnableCheck = nullptr;
    QCheckBox* m_cornerTransitionCurrentEnableCheck = nullptr;
    QCheckBox* m_cornerTransitionVoltageEnableCheck = nullptr;
    QComboBox* m_cornerTransitionScopeCombo = nullptr;
    QDoubleSpinBox* m_finalWeldStepSpin = nullptr;  // 实际焊道点间距(mm)
    QComboBox* m_weldDirectionCombo = nullptr;      // 焊接方向(1=起点到终点/-1=终点到起点)
    QString m_unitName;

    ContralUnit* m_pContralUnit = nullptr;   // 机器人单元列表来源（工艺界面切换机器人用）
    QComboBox* m_robotCombo = nullptr;        // 机器人选择下拉
    QComboBox* m_weaveTypeCombo = nullptr;
    QComboBox* m_weaveImplCombo = nullptr;   // 摆动实现：0=机器人原生 1=上位机自建pointwise(存 T_WELD_PARA.nWrapConditionNo)
    QComboBox* m_weaveShapeCombo = nullptr;
    QComboBox* m_weavePointsCombo = nullptr;  // pointwise每周期采样点数(4/8/12/16/24/32，存工艺 nStandWeldDir 复用死字段)
    QDoubleSpinBox* m_weaveFrequencySpin = nullptr;
    QDoubleSpinBox* m_weaveAmplitudeSpin = nullptr;
    QDoubleSpinBox* m_swingDirectionSpin = nullptr;
    QDoubleSpinBox* m_weavePlaneAngleSpin = nullptr;
    QDoubleSpinBox* m_spaceAngleSpin = nullptr;
    QSpinBox* m_pauseTime1Spin = nullptr;
    QSpinBox* m_pauseTime2Spin = nullptr;
    QSpinBox* m_pauseTime3Spin = nullptr;
    QSpinBox* m_pauseTime4Spin = nullptr;
    QComboBox* m_pauseContinueCombo = nullptr;
    QDoubleSpinBox* m_endLengthSpin = nullptr;
    QDoubleSpinBox* m_endWidthSpin = nullptr;
    QDoubleSpinBox* m_centerHeightSpin = nullptr;
    QGroupBox* m_weaveParamGroup = nullptr;

    QSpinBox* m_lateralBeginCycleSpin = nullptr;
    QDoubleSpinBox* m_lateralGainSpin = nullptr;
    QDoubleSpinBox* m_leftAreaCoefficientSpin = nullptr;
    QDoubleSpinBox* m_rightAreaCoefficientSpin = nullptr;
    QComboBox* m_verticalModeCombo = nullptr;
    QDoubleSpinBox* m_verticalReferenceCurrentSpin = nullptr;
    QSpinBox* m_verticalBeginCycleSpin = nullptr;
    QSpinBox* m_verticalSustainCycleSpin = nullptr;
    QDoubleSpinBox* m_verticalCycleLengthSpin = nullptr;
    QDoubleSpinBox* m_verticalGainSpin = nullptr;
    QComboBox* m_trackIntervalModeCombo = nullptr;
    QSpinBox* m_timeIntervalSpin = nullptr;
    QSpinBox* m_distanceIntervalSpin = nullptr;
    QDoubleSpinBox* m_lateralMinCompSpin = nullptr;
    QDoubleSpinBox* m_lateralMaxCompSpin = nullptr;
    QDoubleSpinBox* m_lateralTotalMaxCompSpin = nullptr;
    QDoubleSpinBox* m_lateralAsymmetrySpin = nullptr;
    QDoubleSpinBox* m_verticalMinCompSpin = nullptr;
    QDoubleSpinBox* m_verticalMaxCompSpin = nullptr;
    QDoubleSpinBox* m_verticalTotalMaxCompSpin = nullptr;
    QDoubleSpinBox* m_verticalAsymmetrySpin = nullptr;
    QGroupBox* m_trackParamGroup = nullptr;
    QString m_cleanSnapshot;
};
