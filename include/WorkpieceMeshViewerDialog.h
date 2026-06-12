#pragma once

#include "WorkpieceMeshBuilder.h"

#include <QDialog>

#include <atomic>
#include <memory>

class QLabel;
class QLineEdit;
class QProgressDialog;
class QPushButton;
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

    WorkpieceMeshGLWidget* m_pGlWidget = nullptr;
    QStackedWidget* m_pViewStack = nullptr;
    QLabel* m_pHeightMapLabel = nullptr;
    QScrollArea* m_pHeightMapScroll = nullptr;
    QLabel* m_pInfoLabel = nullptr;
    QProgressDialog* m_pProgress = nullptr;
    QLineEdit* m_pDirEdit = nullptr;
    QPushButton* m_pSolidBtn = nullptr;
    QPushButton* m_pPointBtn = nullptr;
    QPushButton* m_pHeightBtn = nullptr;
    QPushButton* m_pEditBtn = nullptr;
    QPushButton* m_pDeleteSelBtn = nullptr;
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
    QVector<RobotCalculation::IndexedPoint3D> m_editCloud;
    QVector<QVector<RobotCalculation::IndexedPoint3D>> m_undoStack;
    // 后台线程生命周期守卫：detached 线程经令牌检查后才 invokeMethod(this)，
    // 析构置 destroyed 并等待计数归零，杜绝退出时对悬垂 this 的调用。
    std::shared_ptr<std::atomic_bool> m_destroyed = std::make_shared<std::atomic_bool>(false);
    std::shared_ptr<std::atomic_int> m_workerCount = std::make_shared<std::atomic_int>(0);
    QMetaObject::Connection m_progressCancelConn;
};
