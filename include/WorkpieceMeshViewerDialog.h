#pragma once

#include "WorkpieceMeshBuilder.h"

#include <QDialog>

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

class QLabel;
class QLineEdit;
class QProgressDialog;
class QPushButton;
class QToolButton;
class QScrollArea;
class QStackedWidget;
class WorkpieceMeshGLWidget;

// 工件模型查看器：加载 LaserPoint 目录的网格模型缓存（一次生成的二进制 PLY），
// 提供 实体面 / 点云 两种 3D 模式（QOpenGLWidget 硬件渲染，全量数据）与 2.5D 高度图模式。
// 框选删除：加载原始完整点云进入编辑模式，屏幕矩形框选删除噪声点（可撤销），
// 用剩余点重新生成网格并覆写模型缓存——人工剔除算法无法区分的贴面噪声毛刺。
class WorkpieceMeshViewerDialog : public QDialog
{
public:
    explicit WorkpieceMeshViewerDialog(QWidget* parent = nullptr);
    ~WorkpieceMeshViewerDialog() override;

    // 异步加载目录的模型缓存：缓存有效直接加载（亚秒）；否则后台线程从
    // _WorkpieceCloud.txt 生成（进度条 + 可取消，不阻塞界面），完成后自动显示。
    void StartLoad(const QString& laserDir);

private:
    void BuildUi();
    void ApplyStyle();
    void BrowseResultDirectory();
    void ShowHeightMap();
    void FinishLoadFromCache();
    void OnBuildFinished(bool ok, const QString& error);
    void EnsureProgressDialog();
    void EnterEditMode();
    void ExitEditMode();
    void ShowEditCloud();
    void OnEditPointsRemoved(const QVector<RobotCalculation::IndexedPoint3D>& removed);
    void UndoRemove();
    void RegenerateFromEditedCloud();
    void RestoreOriginalModel();
    void UpdateEditInfo();
    void ShowSorPanel();        // SOR 统计离群参数面板（非模态，预览→删除）
    void ShowClusterPanel();    // 连通域去飞点参数面板（非模态，预览→删除）
    // 后台去噪预览：在后台线程跑 computeFn（只读坐标快照），进度条可取消，完成回 UI
    // 线程把离群/飞点 mask 并入选区。computeFn 命中取消返回空 vector。
    void RunDenoisePreviewAsync(const QString& title,
        std::function<std::vector<std::uint8_t>(const float*, qsizetype,
            std::atomic<qsizetype>*, const std::atomic_bool*)> computeFn);
    void OnDenoisePreviewFinished(const std::vector<std::uint8_t>& mask, bool canceled);
    // 分离图层（固定双层）：选中点切到另一图层单独处理（如可疑区单独去噪），最后可合并。
    void SeparateSelectedToSuspect();
    void SwitchActiveLayer();
    void MergeLayers();
    void SetSuspectLayerVisible(bool on);
    void ResetEditState();

    WorkpieceMeshGLWidget* m_pGlWidget = nullptr;
    QStackedWidget* m_pViewStack = nullptr;
    QLabel* m_pHeightMapLabel = nullptr;
    QScrollArea* m_pHeightMapScroll = nullptr;
    QLabel* m_pInfoLabel = nullptr;
    QProgressDialog* m_pProgress = nullptr;
    QLineEdit* m_pDirEdit = nullptr;
    QWidget* m_pEditBar = nullptr;       // 第二行上下文条容器（编辑模式才显示）
    QPushButton* m_pSolidBtn = nullptr;
    QPushButton* m_pPointBtn = nullptr;
    QPushButton* m_pHeightBtn = nullptr;
    QPushButton* m_pEditBtn = nullptr;
    QPushButton* m_pDeleteSelBtn = nullptr;
    QPushButton* m_pInvertBtn = nullptr;
    QPushButton* m_pDeselectBtn = nullptr;
    QPushButton* m_pIsolateBtn = nullptr;
    QPushButton* m_pLassoBtn = nullptr;
    QToolButton* m_pCleanBtn = nullptr;  // 「清理▾」下拉（SOR/连通域）
    QToolButton* m_pLayerBtn = nullptr;  // 「图层▾」下拉（分离/切换/合并/可见）
    QPushButton* m_pUndoBtn = nullptr;
    QPushButton* m_pRegenBtn = nullptr;
    QPushButton* m_pRestoreBtn = nullptr;
    bool m_bBuilding = false;
    bool m_bEditBusy = false;
    WorkpieceMeshBuilder::Mesh m_mesh;
    QImage m_heightMapCache;
    QString m_laserDir;
    QString m_meshInfoText;
    // 编辑态：原始点云副本 + 撤销栈（每步存被删点，撤销=追加回去）。
    QVector<RobotCalculation::IndexedPoint3D> m_editCloud;       // 活动编辑层（GLWidget 裸指针固定指向它）
    QVector<RobotCalculation::IndexedPoint3D> m_suspectCloud;    // 另一图层（分离出的可疑区，背景暗灰）
    bool m_suspectVisible = true;       // 另一图层是否显示/参与重建
    bool m_editingSuspect = false;      // 当前活动层是否为「可疑层」（仅 UI 显示用）
    QVector<QVector<RobotCalculation::IndexedPoint3D>> m_undoStack;
    // 后台线程生命周期守卫：detached 线程经令牌检查后才 invokeMethod(this)，
    // 析构置 destroyed 并等待计数归零，杜绝退出时对悬垂 this 的调用。
    std::shared_ptr<std::atomic_bool> m_destroyed = std::make_shared<std::atomic_bool>(false);
    std::shared_ptr<std::atomic_int> m_workerCount = std::make_shared<std::atomic_int>(0);
    // 当前在飞的后台去噪取消标志（同时只有一个，m_bEditBusy 串行）；析构置位让计算尽快收尾，
    // 否则退出时若有大点云 SOR 在算，忙等会冻结 UI 数秒。
    std::shared_ptr<std::atomic_bool> m_activeDenoiseCancel;
    QMetaObject::Connection m_progressCancelConn;
};
