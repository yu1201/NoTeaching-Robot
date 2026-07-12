#pragma once

#include "ContralUnit.h"
#include "MeasureThenWeldService.h"

#include <QDialog>
#include <QString>
#include <QStringList>
#include <QVector>

#include <atomic>
#include <memory>
#include <thread>
#include <vector>

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
class QProgressDialog;

namespace pcview { class PointCloud3DView; }

class WeldSeamCompDialog : public QDialog
{
public:
    explicit WeldSeamCompDialog(ContralUnit* pContralUnit, QWidget* parent = nullptr);
    ~WeldSeamCompDialog() override;

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
    void showEvent(QShowEvent* event) override;
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
    QWidget* CreateWeldProcessPanel();
    void LoadWeldProcessArea();
    void ApplySelectedProcessToEditors();
    bool SaveWeldProcessArea(QString& error);
    void ChooseCompPreviewDirectory();
    void SetCompPreviewDirectory(const QString& dir);
    void RefreshCompPreviewScanLine();
    void RebuildCompPreviewBaseline();
    bool ExecuteCompPreviewRebuild(bool showErrorBox);
    void AutoSelectLatestCompPreviewDirectory();
    void ScheduleCompPreview();
    void RecomputeCompPreview();
    // 由 stages 构建六阶段图层并刷新 3D 视图/箭头/信息（编辑实时预览与异步载入共用的"显示"部分）。
    void ApplyComputedStages(const MeasureThenWeldService::CompPreviewStages& stages);
    // 打开/切换目录时后台载入补偿预览：可选重算基准焊道 → 读焊道数据 → 算补偿，进度条+不阻塞 UI。
    struct CompPreviewLoadResult
    {
        QVector<MeasureThenWeldService::CompPreviewPoint> original;
        QVector<MeasureThenWeldService::CompPreviewPoint> raw;
        QVector<MeasureThenWeldService::CompPreviewPoint> baseline;
        MeasureThenWeldService::CompPreviewStages stages;
        QStringList logs;
        bool baselineOk = false;
        QString baselineError;
        bool canceled = false;
    };
    void OpenCompPreviewDirectoryAsync(const QString& dir, bool forceRebuild);
    void OnCompPreviewLoaded(const CompPreviewLoadResult& result);
    void StopAndJoinPreviewWorker();
    void EnsurePreviewProgress();
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
    QVector<MeasureThenWeldService::CompPreviewArrow> m_compPreviewArrows;  // 全量箭头缓存，按开关过滤显示
    QVector<PoseCompRow> m_savedPoseRows;   // 加载/保存后的姿态补偿快照（姿态补偿阶段按 delta 计算）
    QString m_compPreviewDir;
    QString m_compPreviewBaselineDir;
    // 焊接方向（工艺值优先、测量页旧值回退，1=起点到终点 / -1=终点到起点），用于预览方向箭头。
    int m_compPreviewWeldDirection = 1;
    // 扫描轨迹起止点 XY（当前启用组），方向箭头放在扫描位置那一侧（与焊接方向无关）。
    bool m_compPreviewScanLineValid = false;
    QPointF m_compPreviewScanStartXY;
    QPointF m_compPreviewScanEndXY;
    // 防止"载入目录→缺方法文件→自动重算→重新载入"递归。
    bool m_bAutoRebuildingCompPreview = false;
    // 后台异步载入补偿预览：进度框 + 可取消、可 join 的所有权线程。
    // 析构先发出 stop token，再 join；不再 detach+计数忙等。
    QProgressDialog* m_pPreviewProgress = nullptr;
    bool m_bPreviewLoading = false;
    QMetaObject::Connection m_previewProgressCancelConn;
    std::shared_ptr<std::atomic_bool> m_destroyed = std::make_shared<std::atomic_bool>(false);
    std::shared_ptr<std::atomic_bool> m_previewCancel;
    std::thread m_previewWorker;
    // 六阶段图层开关：0=原始数据 1=原始焊道 2=姿态补偿 3=焊道补偿 4=圆弧过渡 5=实际焊道
    QAbstractButton* m_pStageToggles[6] = { nullptr, nullptr, nullptr, nullptr, nullptr, nullptr };

    // 工艺区域（圆弧过渡 + 实际焊道点间距 + 焊接方向，编辑实时联动预览、保存落盘）
    QComboBox* m_pProcessCombo = nullptr;
    QCheckBox* m_pArcEnableCheck = nullptr;
    QDoubleSpinBox* m_pArcRadiusSpin = nullptr;
    QDoubleSpinBox* m_pFinalStepSpin = nullptr;
    QComboBox* m_pWeldDirectionCombo = nullptr;
    QCheckBox* m_pSimplifyTrajectoryCheck = nullptr;  // 精简轨迹(只保留特殊点)开关
    QLabel* m_pProcessHintLabel = nullptr;
    std::vector<T_WELD_PARA> m_weldProcessList;
    int m_weldProcessSelectedIndex = -1;
    bool m_weldProcessLoaded = false;
    bool m_bLoadingProcessArea = false;
    bool m_simplifyTrajectoryEnabled = false;  // 精简轨迹标志(存 WeldSeamCompParam/ALLWeldSeamComp，落盘+预览联动+真实焊接生效)
};
