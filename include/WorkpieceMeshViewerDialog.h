#pragma once

#include "WorkpieceMeshBuilder.h"

#include <QDialog>

class QLabel;
class QProgressDialog;
class QPushButton;
class QScrollArea;
class QStackedWidget;
class WorkpieceMeshGLWidget;

// 工件模型查看器：加载 LaserPoint 目录的网格模型缓存（一次生成的二进制 PLY），
// 提供 实体面 / 点云 两种 3D 模式（QOpenGLWidget 硬件渲染，全量数据）与 2.5D 高度图模式。
class WorkpieceMeshViewerDialog : public QDialog
{
public:
    explicit WorkpieceMeshViewerDialog(QWidget* parent = nullptr);

    // 异步加载目录的模型缓存：缓存有效直接加载（亚秒）；否则后台线程从
    // _WorkpieceCloud.txt 生成（进度条 + 可取消，不阻塞界面），完成后自动显示。
    void StartLoad(const QString& laserDir);

private:
    void BuildUi();
    void ApplyStyle();
    void ShowHeightMap();
    void FinishLoadFromCache();
    void OnBuildFinished(bool ok, const QString& error);

    WorkpieceMeshGLWidget* m_pGlWidget = nullptr;
    QStackedWidget* m_pViewStack = nullptr;
    QLabel* m_pHeightMapLabel = nullptr;
    QScrollArea* m_pHeightMapScroll = nullptr;
    QLabel* m_pInfoLabel = nullptr;
    QProgressDialog* m_pProgress = nullptr;
    bool m_bBuilding = false;
    WorkpieceMeshBuilder::Mesh m_mesh;
    QImage m_heightMapCache;
    QString m_laserDir;
};
