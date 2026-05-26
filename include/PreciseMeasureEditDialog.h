#pragma once

#include "Const.h"
#include "ContralUnit.h"

#include <QDialog>
#include <QList>
#include <QMap>
#include <QString>
#include <functional>

class QComboBox;
class QCloseEvent;
class QCheckBox;
class QDoubleSpinBox;
class QGridLayout;
class QGroupBox;
class QLabel;
class QLineEdit;
class QPlainTextEdit;
class QPushButton;
class QSplitter;
class RobotDriverAdaptor;

// 测量焊接参数界面：选择机器人和参数组后读取/修改 MeasureWeldParam.ini。
// 安全姿态保留脉冲点，扫描起终点改为直角坐标点。
class PreciseMeasureEditDialog : public QDialog
{
public:
    explicit PreciseMeasureEditDialog(
        ContralUnit* pContralUnit,
        QWidget* parent = nullptr,
        bool positionTeachOnly = false,
        std::function<void()> openCameraPreviewCallback = nullptr);

protected:
    void closeEvent(QCloseEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;

private:
    void BuildUi();
    void LoadRobotList();
    void OnRobotChanged(int index);
    void LoadParamGroups();
    void OnParamGroupChanged(int index);
    void AddZeroParamGroup();
    void CopyCurrentParamGroup();
    void DeleteCurrentParamGroup();
    void TeachStartSafePulse();
    void TeachStartPos();
    void TeachEndPos();
    void TeachEndSafePulse();
    void OpenPositionTeachDialog();
    void ReloadCurrentParam();
    bool SaveAllParamEdits();
    void SaveManualStartSafePulse();
    void SaveManualStartPos();
    void SaveManualEndPos();
    void SaveManualEndSafePulse();
    void SaveOtherParamEdit();
    void SaveOtherParamComboEdit(QComboBox* combo);

    RobotDriverAdaptor* GetSelectedRobotDriver();
    QString CurrentRobotName() const;
    QString CurrentParamFilePath() const;
    QString CurrentSectionName(QString* error = nullptr) const;
    QString CurrentWeldSectionName(QString* error = nullptr) const;
    int CurrentGroupIndex() const;
    QString CurrentGroupName() const;
    bool LoadCurrentParam();
    bool ReadPulse(const QString& prefix, T_ANGLE_PULSE& pulse, QString& error) const;
    bool WritePulse(const QString& prefix, const T_ANGLE_PULSE& pulse, QString& error) const;
    bool ReadCoors(const QString& prefix, T_ROBOT_COORS& coors, QString& error) const;
    bool WriteCoors(const QString& prefix, const T_ROBOT_COORS& coors, QString& error) const;
    bool LoadOtherParams();
    QWidget* CreateScanSafeParamPanel();
    bool LoadScanSafeParams();
    bool SaveScanSafeParams(QString& error) const;
    void UpdateScanSafePreview();
    void UpdateComputedScanSafeUiState();
    bool WriteParamValue(const QString& sectionName, const QString& key, const QString& value, QString& error) const;
    bool SaveGroupMetadata(QString& error) const;
    bool CreateParamGroup(bool copyCurrent, QString& error);
    QGroupBox* CreatePulseGroup(const QString& title, const QString& groupName, const QString& teachText, void (PreciseMeasureEditDialog::*teachSlot)(), void (PreciseMeasureEditDialog::*saveSlot)());
    QGroupBox* CreateCoorsGroup(const QString& title, const QString& groupName, const QString& teachText, void (PreciseMeasureEditDialog::*teachSlot)(), void (PreciseMeasureEditDialog::*saveSlot)());
    void SetPulseEditors(const QString& groupName, const T_ANGLE_PULSE& pulse);
    void SetCoorsEditors(const QString& groupName, const T_ROBOT_COORS& coors);
    bool GetPulseFromEditors(const QString& groupName, T_ANGLE_PULSE& pulse, QString& error) const;
    bool GetCoorsFromEditors(const QString& groupName, T_ROBOT_COORS& coors, QString& error) const;
    void SetEditorsBlocked(bool blocked);
    void ClearOtherParamEditors();
    void SwitchOtherParamPage(bool showScanPage);
    void UpdateOtherParamPageVisibility();
    void UpdateAdaptiveLayout();
    void RebuildPulseGroupLayout(bool wide);
    bool HasUnsavedChanges() const;
    QString BuildSnapshot() const;
    void MarkCleanSnapshot();
    void AppendLog(const QString& text);

private:
    ContralUnit* m_pContralUnit = nullptr;
    bool m_bPositionTeachOnly = false;
    std::function<void()> m_openCameraPreviewCallback;
    QComboBox* m_pRobotCombo = nullptr;
    QComboBox* m_pGroupCombo = nullptr;
    QLineEdit* m_pGroupNameEdit = nullptr;
    QSplitter* m_pContentSplitter = nullptr;
    QWidget* m_pPulsePanel = nullptr;
    QWidget* m_pOtherPanel = nullptr;
    QGridLayout* m_pPulseGroupsLayout = nullptr;
    QGridLayout* m_pOtherParamLayout = nullptr;
    QPushButton* m_pScanParamTabBtn = nullptr;
    QPushButton* m_pWeldParamTabBtn = nullptr;
    QPlainTextEdit* m_pLogText = nullptr;
    QGroupBox* m_pStartSafePulseGroup = nullptr;
    QGroupBox* m_pEndSafePulseGroup = nullptr;
    QWidget* m_pScanSafeParamPanel = nullptr;
    QCheckBox* m_pUseComputedScanSafeCheck = nullptr;
    QDoubleSpinBox* m_pScanSafeOffsetSpin = nullptr;
    QDoubleSpinBox* m_pScanSafeGunAngleSpin = nullptr;
    QComboBox* m_pScanSafeXDirectionCombo = nullptr;
    QDoubleSpinBox* m_pScanSafeLiftSpin = nullptr;
    QDoubleSpinBox* m_pScanSafeFlipWarnSpin = nullptr;
    QLabel* m_pScanSafePreviewLabel = nullptr;
    QLabel* m_pScanSafePathLabel = nullptr;
    QList<QWidget*> m_pulseGroupWidgets;
    QList<QWidget*> m_otherParamSectionWidgets;
    QMap<QString, QLineEdit*> m_editors;
    QMap<QString, QLineEdit*> m_otherParamEditors;
    QMap<QString, QComboBox*> m_otherParamComboEditors;
    T_ANGLE_PULSE m_taughtStartPulse;
    bool m_hasTaughtStartPulse = false;
    bool m_bWideAdaptiveLayout = true;
    bool m_bLoading = false;
    bool m_showScanParamPage = true;
    QString m_cleanSnapshot;
};
