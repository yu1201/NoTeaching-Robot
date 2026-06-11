#pragma once

#include "WorkpieceMeshBuilder.h"

#include <QDialog>

class QLabel;
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

    // 加载目录的模型缓存；缓存不存在时从 _WorkpieceCloud.txt 自动生成一次（耗时数秒，带等待光标）。
    bool LoadFromLaserDir(const QString& laserDir, QString& error);

private:
    void BuildUi();
    void ApplyStyle();
    void ShowHeightMap();

    WorkpieceMeshGLWidget* m_pGlWidget = nullptr;
    QStackedWidget* m_pViewStack = nullptr;
    QLabel* m_pHeightMapLabel = nullptr;
    QScrollArea* m_pHeightMapScroll = nullptr;
    QLabel* m_pInfoLabel = nullptr;
    WorkpieceMeshBuilder::Mesh m_mesh;
    QImage m_heightMapCache;
    QString m_laserDir;
};
