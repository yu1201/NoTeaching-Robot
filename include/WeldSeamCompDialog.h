#pragma once

#include "ContralUnit.h"
#include "MeasureThenWeldService.h"

#include <QDialog>
#include <QString>
#include <QVector>

class QButtonGroup;
class QCheckBox;
class QComboBox;
class QCloseEvent;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QListWidget;
class QPlainTextEdit;
class QAbstractButton;
class QPushButton;
class QWidget;
class QTimer;
class QSplitter;

namespace pcview { class PointCloud3DView; }

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
        double innerToOuterCornerComp = 0.0;
        double innerToInnerCornerComp = 0.0;
        double outerToOuterCornerComp = 0.0;
        double outerToInnerCornerComp = 0.0;
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
    void LoadPoseCornerCompensationFromDatabase(const QString& robotName);
    void LoadSeamParam(const QString& path);
    bool SaveCurrentParam();
    bool SavePoseParam(const QString& path, QString& error) const;
    bool SavePoseCornerCompensationToDatabase(const QString& robotName, QString& error) const;
    bool SaveSeamParam(const QString& path, QString& error) const;
    void RefreshCornerCompensationEditor();
    void SetMode(CompMode mode);
    void RefreshGroupCombo();
    void RefreshTypeCombo();
    void RefreshEditor();
    bool StoreEditorValues(bool validate, QString& error);
    void AddCurrentGroup();
    void RenameCurrentGroup();
    void DeleteCurrentGroup();
    bool EditGroupName(QString& name, const QString& title);
    QString ModeGroupName() const;
    QString DefaultGroupName(int index) const;
    QString DefaultPoseRowName(int index) const;
    QString DefaultPoseSegmentKind(int index) const;
    PoseCompRow MakeDefaultPoseRow(int groupIndex, int segmentIndex) const;
    SeamCompRow MakeDefaultSeamRow(int groupIndex, int segmentIndex) const;
    int CurrentPoseGroupMatchMode() const;
    void SetCurrentPoseGroupMatchMode(int mode);
    int CurrentGroupCount() const;
    int CurrentFlatRowIndex() const;
    bool HasUnsavedChanges() const;
    QString BuildSnapshot() const;
    void MarkCleanSnapshot();
    void AppendLog(const QString& text);
    QString CurrentRobotName() const;
    QString CurrentPoseParamPath() const;
    QString CurrentSeamParamPath() const;
    int CurrentRowCount() const;
    int CurrentTypeIndex() const;

    // 补偿前 / 补偿后 焊道可视化对比
    QWidget* CreateCompPreviewPanel();
    void ChooseCompPreviewDirectory();
    void ScheduleCompPreview();
    void RecomputeCompPreview();
    void ApplyCompPreviewLayerVisibility();
    MeasureThenWeldService::CompPreviewEditValues CollectCompPreviewEditValues() const;
    MeasureThenWeldService::CompPreviewEditValues CollectSavedPoseCompEdits() const;
    void BackupOldCompFiles(const QString& backupRoot, QString& summary) const;
    int CurrentRobotType() const;

private:
    ContralUnit* m_pContralUnit = nullptr;
    QComboBox* m_pRobotCombo = nullptr;
    QButtonGroup* m_pModeGroup = nullptr;
    QPushButton* m_pPoseModeBtn = nullptr;
    QPushButton* m_pSeamModeBtn = nullptr;
    QListWidget* m_pGroupList = nullptr;
    QComboBox* m_pTypeCombo = nullptr;
    QPushButton* m_pNewGroupBtn = nullptr;
    QPushButton* m_pRenameGroupBtn = nullptr;
    QPushButton* m_pDeleteGroupBtn = nullptr;
    QLabel* m_pPathLabel = nullptr;
    QLabel* m_pHintLabel = nullptr;
    QWidget* m_pPoseDisplayWidget = nullptr;
    QLabel* m_pPoseMatchModeLabel = nullptr;
    QComboBox* m_pPoseMatchModeCombo = nullptr;
    QLabel* m_pEditLabels[3] = { nullptr, nullptr, nullptr };
    QLineEdit* m_pEditValues[3] = { nullptr, nullptr, nullptr };
    QLineEdit* m_pPoseValues[3] = { nullptr, nullptr, nullptr };
    QWidget* m_pCornerCompensationWidget = nullptr;
    QCheckBox* m_pCornerCompensationCheck = nullptr;
    QDoubleSpinBox* m_pCornerCompensationValues[4] = { nullptr, nullptr, nullptr, nullptr };
    QPlainTextEdit* m_pLogText = nullptr;
    QPushButton* m_pReloadBtn = nullptr;
    QPushButton* m_pSaveBtn = nullptr;
    QVector<CompType> m_poseTypes;
    QVector<QString> m_poseGroupNames;
    QVector<int> m_poseGroupMatchModes;
    QVector<bool> m_poseGroupCornerCompEnabled;
    QVector<QString> m_seamGroupNames;
    QVector<PoseCompRow> m_poseRows;
    QVector<SeamCompRow> m_seamRows;
    double m_poseMatchMaxErrorDeg = 5.0;
    int m_poseCompMatchMode = 0;
    CompMode m_mode = CompMode::Pose;
    int m_currentGroupIndex = 0;
    int m_currentPoseGroupIndex = 0;
    int m_currentSeamGroupIndex = 0;
    int m_currentTypeIndex = 0;
    bool m_bLoading = false;
    QString m_cleanSnapshot;

    // 补偿前 / 补偿后 焊道可视化对比
    pcview::PointCloud3DView* m_pCompPreviewView = nullptr;
    QLineEdit* m_pCompPreviewDirEdit = nullptr;
    QLabel* m_pCompPreviewInfoLabel = nullptr;
    QTimer* m_pCompPreviewTimer = nullptr;
    MeasureThenWeldService m_compPreviewService;
    QVector<MeasureThenWeldService::CompPreviewPoint> m_compPreviewBaseline;
    QVector<MeasureThenWeldService::CompPreviewPoint> m_compPreviewOriginal;
    QVector<MeasureThenWeldService::CompPreviewPoint> m_compPreviewRaw;
    QVector<PoseCompRow> m_savedPoseRows;   // 加载/保存后的姿态补偿快照（姿态补偿阶段按 delta 计算）
    QString m_compPreviewDir;
    QString m_compPreviewBaselineDir;
    // 五阶段图层开关：0=原始数据 1=原始焊道 2=姿态补偿 3=焊道补偿 4=圆弧过渡
    QAbstractButton* m_pStageToggles[5] = { nullptr, nullptr, nullptr, nullptr, nullptr };
};
