#pragma once

#include "ContralUnit.h"

#include <QDialog>
#include <QString>
#include <QVector>

class QButtonGroup;
class QComboBox;
class QCloseEvent;
class QLabel;
class QLineEdit;
class QPlainTextEdit;
class QPushButton;
class QWidget;

class WeldSeamCompDialog : public QDialog
{
public:
    explicit WeldSeamCompDialog(ContralUnit* pContralUnit, QWidget* parent = nullptr);

public:
    enum class CompMode
    {
        Pose,
        Seam
    };

    struct CompType
    {
        QString displayName;
        QString segmentKind;
    };

    struct PoseCompRow
    {
        QString sectionName;
        QString name;
        QString segmentKind;
        double poseRx = 0.0;
        double poseRy = 30.0;
        double poseRz = 180.0;
        double compX = 0.0;
        double compY = 0.0;
        double compZ = 0.0;
    };

    struct SeamCompRow
    {
        QString sectionName;
        QString name;
        QString segmentKind;
        double weldZComp = 0.0;
        double weldGunDirComp = 0.0;
        double weldSeamDirComp = 0.0;
    };

private:
    void closeEvent(QCloseEvent* event) override;
    void BuildUi();
    void LoadRobotList();
    void LoadCurrentParam();
    void LoadPoseParam(const QString& path);
    void LoadSeamParam(const QString& path);
    bool SaveCurrentParam();
    bool SavePoseParam(const QString& path, QString& error) const;
    bool SaveSeamParam(const QString& path, QString& error) const;
    void SetMode(CompMode mode);
    void RefreshTypeCombo();
    void RefreshEditor();
    bool StoreEditorValues(bool validate, QString& error);
    bool HasUnsavedChanges() const;
    QString BuildSnapshot() const;
    void MarkCleanSnapshot();
    void AppendLog(const QString& text);
    QString CurrentRobotName() const;
    QString CurrentPoseParamPath() const;
    QString CurrentSeamParamPath() const;
    QString CurrentParamPath() const;
    int CurrentRowCount() const;
    int CurrentTypeIndex() const;

private:
    ContralUnit* m_pContralUnit = nullptr;
    QComboBox* m_pRobotCombo = nullptr;
    QButtonGroup* m_pModeGroup = nullptr;
    QPushButton* m_pPoseModeBtn = nullptr;
    QPushButton* m_pSeamModeBtn = nullptr;
    QComboBox* m_pTypeCombo = nullptr;
    QLabel* m_pPathLabel = nullptr;
    QLabel* m_pHintLabel = nullptr;
    QWidget* m_pPoseDisplayWidget = nullptr;
    QLabel* m_pEditLabels[3] = { nullptr, nullptr, nullptr };
    QLineEdit* m_pEditValues[3] = { nullptr, nullptr, nullptr };
    QLineEdit* m_pPoseValues[3] = { nullptr, nullptr, nullptr };
    QPlainTextEdit* m_pLogText = nullptr;
    QPushButton* m_pReloadBtn = nullptr;
    QPushButton* m_pSaveBtn = nullptr;
    QVector<CompType> m_poseTypes;
    QVector<PoseCompRow> m_poseRows;
    QVector<SeamCompRow> m_seamRows;
    double m_poseMatchMaxErrorDeg = 5.0;
    CompMode m_mode = CompMode::Pose;
    int m_currentTypeIndex = 0;
    bool m_bLoading = false;
    QString m_cleanSnapshot;
};
