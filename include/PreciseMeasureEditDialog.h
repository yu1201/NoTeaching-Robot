#pragma once

#include "Const.h"
#include "ContralUnit.h"

#include <QDialog>
#include <QList>
#include <QMap>
#include <QString>

class QComboBox;
class QGridLayout;
class QGroupBox;
class QLineEdit;
class QPlainTextEdit;
class QSplitter;
class RobotDriverAdaptor;

// 精测量数据修改界面：选择机器人后读取/修改 PreciseMeasureParam.ini。
// 安全姿态保留脉冲点，扫描起终点改为直角坐标点。
class PreciseMeasureEditDialog : public QDialog
{
public:
    explicit PreciseMeasureEditDialog(ContralUnit* pContralUnit, QWidget* parent = nullptr);

protected:
    void resizeEvent(QResizeEvent* event) override;

private:
    void BuildUi();
    void LoadRobotList();
    void OnRobotChanged(int index);
    void TeachStartSafePulse();
    void TeachStartPos();
    void TeachEndPos();
    void TeachEndSafePulse();
    void ReloadCurrentParam();
    void SaveAllParamEdits();
    void SaveManualStartSafePulse();
    void SaveManualStartPos();
    void SaveManualEndPos();
    void SaveManualEndSafePulse();
    void SaveOtherParamEdit();

    RobotDriverAdaptor* GetSelectedRobotDriver();
    QString CurrentRobotName() const;
    QString CurrentParamFilePath() const;
    QString CurrentSectionName(QString* error = nullptr) const;
    bool LoadCurrentParam();
    bool ReadPulse(const QString& prefix, T_ANGLE_PULSE& pulse, QString& error) const;
    bool WritePulse(const QString& prefix, const T_ANGLE_PULSE& pulse, QString& error) const;
    bool ReadCoors(const QString& prefix, T_ROBOT_COORS& coors, QString& error) const;
    bool WriteCoors(const QString& prefix, const T_ROBOT_COORS& coors, QString& error) const;
    bool LoadOtherParams();
    bool WriteParamValue(const QString& key, const QString& value, QString& error) const;
    QGroupBox* CreatePulseGroup(const QString& title, const QString& groupName, const QString& teachText, void (PreciseMeasureEditDialog::*teachSlot)(), void (PreciseMeasureEditDialog::*saveSlot)());
    QGroupBox* CreateCoorsGroup(const QString& title, const QString& groupName, const QString& teachText, void (PreciseMeasureEditDialog::*teachSlot)(), void (PreciseMeasureEditDialog::*saveSlot)());
    void SetPulseEditors(const QString& groupName, const T_ANGLE_PULSE& pulse);
    void SetCoorsEditors(const QString& groupName, const T_ROBOT_COORS& coors);
    bool GetPulseFromEditors(const QString& groupName, T_ANGLE_PULSE& pulse, QString& error) const;
    bool GetCoorsFromEditors(const QString& groupName, T_ROBOT_COORS& coors, QString& error) const;
    void SetEditorsBlocked(bool blocked);
    void ClearOtherParamEditors();
    void UpdateAdaptiveLayout();
    void RebuildPulseGroupLayout(bool wide);
    void AppendLog(const QString& text);

private:
    ContralUnit* m_pContralUnit = nullptr;
    QComboBox* m_pRobotCombo = nullptr;
    QSplitter* m_pContentSplitter = nullptr;
    QWidget* m_pPulsePanel = nullptr;
    QWidget* m_pOtherPanel = nullptr;
    QGridLayout* m_pPulseGroupsLayout = nullptr;
    QGridLayout* m_pOtherParamLayout = nullptr;
    QPlainTextEdit* m_pLogText = nullptr;
    QList<QWidget*> m_pulseGroupWidgets;
    QMap<QString, QLineEdit*> m_editors;
    QMap<QString, QLineEdit*> m_otherParamEditors;
    bool m_bWideAdaptiveLayout = true;
    bool m_bLoading = false;
};
