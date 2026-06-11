#include "WorkpieceMeshViewerDialog.h"

#include "WindowStyleHelper.h"

#include <QApplication>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QMessageBox>
#include <QProgressDialog>

#include <atomic>
#include <memory>
#include <thread>
#include <QHBoxLayout>
#include <QLabel>
#include <QMatrix4x4>
#include <QMouseEvent>
#include <QOpenGLBuffer>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLWidget>
#include <QPushButton>
#include <QScrollArea>
#include <QStackedWidget>
#include <QSurfaceFormat>
#include <QVBoxLayout>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>

// ===================== OpenGL 网格/点云视图 =====================
//
// 全量网格（百万级三角形）走 GPU：单 VBO（pos+normal 交错）+ EBO，Lambert 光照，
// 顶点按高度伪彩。相机为轨迹球式（旋转/平移/缩放），与 PointCloud3DView 手感一致。
class WorkpieceMeshGLWidget : public QOpenGLWidget, protected QOpenGLFunctions
{
public:
    explicit WorkpieceMeshGLWidget(QWidget* parent = nullptr)
        : QOpenGLWidget(parent)
    {
        QSurfaceFormat format;
        format.setVersion(3, 3);
        format.setProfile(QSurfaceFormat::CoreProfile);
        format.setDepthBufferSize(24);
        format.setSamples(4);
        setFormat(format);
        setMinimumSize(480, 360);
        setFocusPolicy(Qt::StrongFocus);
    }

    ~WorkpieceMeshGLWidget() override
    {
        makeCurrent();
        m_vertexBuffer.destroy();
        m_indexBuffer.destroy();
        m_vao.destroy();
        delete m_pProgram;
        doneCurrent();
    }

    void SetMesh(const WorkpieceMeshBuilder::Mesh& mesh)
    {
        m_pendingMesh = mesh;
        m_meshDirty = true;
        ComputeBounds(mesh);
        ResetView();
        if (isValid())
        {
            update();
        }
    }

    void SetSolidMode(bool solid)
    {
        m_solidMode = solid;
        update();
    }

    void ResetView()
    {
        m_yawDeg = -35.0f;
        m_pitchDeg = -55.0f;
        m_panX = 0.0f;
        m_panY = 0.0f;
        m_distance = m_boundRadius * 2.4f;
        update();
    }

    void SetTopView()
    {
        m_yawDeg = 0.0f;
        m_pitchDeg = -89.0f;
        m_panX = 0.0f;
        m_panY = 0.0f;
        m_distance = m_boundRadius * 2.4f;
        update();
    }

protected:
    void initializeGL() override
    {
        initializeOpenGLFunctions();
        glClearColor(0.043f, 0.071f, 0.094f, 1.0f);  // 与深色主题一致 #0B1218
        glEnable(GL_DEPTH_TEST);

        m_pProgram = new QOpenGLShaderProgram();
        m_pProgram->addShaderFromSourceCode(QOpenGLShader::Vertex,
            "#version 330 core\n"
            "layout(location=0) in vec3 inPos;\n"
            "layout(location=1) in vec3 inNormal;\n"
            "uniform mat4 uMvp;\n"
            "uniform int uColorAxis;\n"  // 伪彩轴=包围盒跨度最小的轴（表面偏移方向，姿态无关）
            "uniform float uZMin;\n"
            "uniform float uZRange;\n"
            "out vec3 vNormal;\n"
            "out float vHeight;\n"
            "void main(){\n"
            "  gl_Position = uMvp * vec4(inPos, 1.0);\n"
            "  vNormal = inNormal;\n"
            "  vHeight = clamp((inPos[uColorAxis] - uZMin) / uZRange, 0.0, 1.0);\n"
            "  gl_PointSize = 2.0;\n"
            "}\n");
        m_pProgram->addShaderFromSourceCode(QOpenGLShader::Fragment,
            "#version 330 core\n"
            "in vec3 vNormal;\n"
            "in float vHeight;\n"
            "uniform vec3 uLightDir;\n"
            "uniform float uUseLight;\n"
            "out vec4 fragColor;\n"
            "vec3 turbo(float t){\n"  // 高度伪彩（近似 turbo 渐变）
            "  vec3 a = vec3(0.190, 0.072, 0.232);\n"
            "  vec3 b = vec3(0.058, 0.640, 0.844);\n"
            "  vec3 c = vec3(0.500, 0.930, 0.250);\n"
            "  vec3 d = vec3(0.980, 0.730, 0.090);\n"
            "  vec3 e = vec3(0.730, 0.090, 0.090);\n"
            "  if (t < 0.25) return mix(a, b, t / 0.25);\n"
            "  if (t < 0.50) return mix(b, c, (t - 0.25) / 0.25);\n"
            "  if (t < 0.75) return mix(c, d, (t - 0.50) / 0.25);\n"
            "  return mix(d, e, (t - 0.75) / 0.25);\n"
            "}\n"
            "void main(){\n"
            "  vec3 base = turbo(vHeight);\n"
            "  float diff = abs(dot(normalize(vNormal), normalize(uLightDir)));\n"  // 双面光照（绕序/朝向无关）
            "  float k = mix(1.0, 0.30 + 0.70 * diff, uUseLight);\n"
            "  fragColor = vec4(base * k, 1.0);\n"
            "}\n");
        m_pProgram->link();

        m_vao.create();
        m_vertexBuffer = QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
        m_vertexBuffer.create();
        m_indexBuffer = QOpenGLBuffer(QOpenGLBuffer::IndexBuffer);
        m_indexBuffer.create();
        glEnable(GL_PROGRAM_POINT_SIZE);
    }

    void paintGL() override
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        if (m_meshDirty)
        {
            UploadMesh();
        }
        if (m_indexCount == 0 || m_pProgram == nullptr)
        {
            return;
        }

        QMatrix4x4 projection;
        const float aspect = height() > 0 ? float(width()) / float(height()) : 1.0f;
        projection.perspective(40.0f, aspect, m_boundRadius * 0.01f, m_boundRadius * 20.0f);

        QMatrix4x4 view;
        view.translate(m_panX, m_panY, -m_distance);
        view.rotate(m_pitchDeg, 1.0f, 0.0f, 0.0f);
        view.rotate(m_yawDeg, 0.0f, 0.0f, 1.0f);
        view.translate(-m_center);

        m_pProgram->bind();
        m_pProgram->setUniformValue("uMvp", projection * view);
        m_pProgram->setUniformValue("uColorAxis", m_colorAxis);
        m_pProgram->setUniformValue("uZMin", m_zMin);
        m_pProgram->setUniformValue("uZRange", std::max(m_zMax - m_zMin, 1e-3f));
        m_pProgram->setUniformValue("uLightDir", QVector3D(-0.4f, -0.3f, 1.0f));
        m_pProgram->setUniformValue("uUseLight", m_solidMode ? 1.0f : 0.0f);

        m_vao.bind();
        if (m_solidMode)
        {
            glDrawElements(GL_TRIANGLES, m_indexCount, GL_UNSIGNED_INT, nullptr);
        }
        else
        {
            glDrawArrays(GL_POINTS, 0, m_vertexCount);
        }
        m_vao.release();
        m_pProgram->release();
    }

    void mousePressEvent(QMouseEvent* event) override
    {
        m_lastMousePos = event->position();
    }

    void mouseMoveEvent(QMouseEvent* event) override
    {
        const QPointF delta = event->position() - m_lastMousePos;
        m_lastMousePos = event->position();
        if (event->buttons() & Qt::LeftButton)
        {
            m_yawDeg += float(delta.x()) * 0.4f;
            m_pitchDeg = std::clamp(m_pitchDeg + float(delta.y()) * 0.4f, -179.0f, -1.0f);
            update();
        }
        else if (event->buttons() & (Qt::RightButton | Qt::MiddleButton))
        {
            const float scale = m_distance * 0.0016f;
            m_panX += float(delta.x()) * scale;
            m_panY -= float(delta.y()) * scale;
            update();
        }
    }

    void mouseDoubleClickEvent(QMouseEvent*) override
    {
        ResetView();
    }

    void wheelEvent(QWheelEvent* event) override
    {
        const float factor = event->angleDelta().y() > 0 ? 0.85f : 1.18f;
        m_distance = std::clamp(m_distance * factor, m_boundRadius * 0.05f, m_boundRadius * 20.0f);
        update();
    }

private:
    void ComputeBounds(const WorkpieceMeshBuilder::Mesh& mesh)
    {
        if (mesh.vertices.isEmpty())
        {
            m_center = QVector3D();
            m_boundRadius = 100.0f;
            m_zMin = 0.0f;
            m_zMax = 1.0f;
            return;
        }
        Eigen::Vector3f minBound = mesh.vertices.first();
        Eigen::Vector3f maxBound = mesh.vertices.first();
        for (const auto& v : mesh.vertices)
        {
            minBound = minBound.cwiseMin(v);
            maxBound = maxBound.cwiseMax(v);
        }
        const Eigen::Vector3f center = (minBound + maxBound) * 0.5f;
        m_center = QVector3D(center.x(), center.y(), center.z());
        m_boundRadius = std::max(1.0f, (maxBound - minBound).norm() * 0.5f);
        // 伪彩轴 = 包围盒跨度最小的轴（表面偏移方向），工件摆放姿态无关。
        const Eigen::Vector3f span = maxBound - minBound;
        m_colorAxis = 0;
        if (span.y() <= span.x() && span.y() <= span.z())
        {
            m_colorAxis = 1;
        }
        else if (span.z() <= span.x() && span.z() <= span.y())
        {
            m_colorAxis = 2;
        }
        m_zMin = minBound[m_colorAxis];
        m_zMax = maxBound[m_colorAxis];
    }

    void UploadMesh()
    {
        m_meshDirty = false;
        m_vertexCount = 0;
        m_indexCount = 0;
        if (!m_pendingMesh.IsValid())
        {
            return;
        }

        QVector<float> interleaved;
        interleaved.reserve(m_pendingMesh.vertices.size() * 6);
        for (qsizetype i = 0; i < m_pendingMesh.vertices.size(); ++i)
        {
            const auto& v = m_pendingMesh.vertices[i];
            const auto& n = m_pendingMesh.normals[i];
            interleaved << v.x() << v.y() << v.z() << n.x() << n.y() << n.z();
        }

        m_vao.bind();
        m_vertexBuffer.bind();
        m_vertexBuffer.allocate(interleaved.constData(), int(interleaved.size() * sizeof(float)));
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), nullptr);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
            reinterpret_cast<void*>(3 * sizeof(float)));
        m_indexBuffer.bind();
        m_indexBuffer.allocate(m_pendingMesh.indices.constData(),
            int(m_pendingMesh.indices.size() * sizeof(quint32)));
        m_vao.release();

        m_vertexCount = int(m_pendingMesh.vertices.size());
        m_indexCount = int(m_pendingMesh.indices.size());
        m_pendingMesh = WorkpieceMeshBuilder::Mesh();  // 上传后释放内存副本
    }

    QOpenGLShaderProgram* m_pProgram = nullptr;
    QOpenGLVertexArrayObject m_vao;
    QOpenGLBuffer m_vertexBuffer;
    QOpenGLBuffer m_indexBuffer;
    WorkpieceMeshBuilder::Mesh m_pendingMesh;
    bool m_meshDirty = false;
    int m_vertexCount = 0;
    int m_indexCount = 0;
    bool m_solidMode = true;

    QVector3D m_center;
    float m_boundRadius = 100.0f;
    int m_colorAxis = 2;
    float m_zMin = 0.0f;
    float m_zMax = 1.0f;
    float m_yawDeg = -35.0f;
    float m_pitchDeg = -55.0f;
    float m_panX = 0.0f;
    float m_panY = 0.0f;
    float m_distance = 300.0f;
    QPointF m_lastMousePos;
};

// ===================== 查看器对话框 =====================

WorkpieceMeshViewerDialog::WorkpieceMeshViewerDialog(QWidget* parent)
    : QDialog(parent)
{
    setWindowTitle("工件模型");
    resize(1100, 760);
    BuildUi();
    ApplyStyle();
}

void WorkpieceMeshViewerDialog::BuildUi()
{
    QVBoxLayout* rootLayout = new QVBoxLayout(this);
    rootLayout->setContentsMargins(10, 10, 10, 10);
    rootLayout->setSpacing(8);

    QHBoxLayout* toolbar = new QHBoxLayout();
    QPushButton* solidBtn = new QPushButton("实体面");
    QPushButton* pointBtn = new QPushButton("点云");
    QPushButton* heightBtn = new QPushButton("高度图");
    QPushButton* topBtn = new QPushButton("俯视");
    QPushButton* resetBtn = new QPushButton("重置");
    QPushButton* exportStlBtn = new QPushButton("导出 STL");
    exportStlBtn->setToolTip("导出二进制 STL 网格（SolidWorks/UG 等 CAD 软件可直接导入）。");
    for (QPushButton* button : { solidBtn, pointBtn, heightBtn, topBtn, resetBtn, exportStlBtn })
    {
        button->setMinimumWidth(72);
        toolbar->addWidget(button);
    }
    toolbar->addStretch(1);
    rootLayout->addLayout(toolbar);

    m_pViewStack = new QStackedWidget(this);
    m_pGlWidget = new WorkpieceMeshGLWidget(this);
    m_pViewStack->addWidget(m_pGlWidget);

    m_pHeightMapScroll = new QScrollArea(this);
    m_pHeightMapLabel = new QLabel(this);
    m_pHeightMapLabel->setAlignment(Qt::AlignCenter);
    m_pHeightMapScroll->setWidget(m_pHeightMapLabel);
    m_pHeightMapScroll->setWidgetResizable(true);
    m_pViewStack->addWidget(m_pHeightMapScroll);
    rootLayout->addWidget(m_pViewStack, 1);

    m_pInfoLabel = new QLabel("加载工件模型缓存后显示统计信息。");
    m_pInfoLabel->setWordWrap(true);
    rootLayout->addWidget(m_pInfoLabel);

    connect(solidBtn, &QPushButton::clicked, this, [this]()
        {
            m_pViewStack->setCurrentIndex(0);
            m_pGlWidget->SetSolidMode(true);
        });
    connect(pointBtn, &QPushButton::clicked, this, [this]()
        {
            m_pViewStack->setCurrentIndex(0);
            m_pGlWidget->SetSolidMode(false);
        });
    connect(heightBtn, &QPushButton::clicked, this, [this]() { ShowHeightMap(); });
    connect(topBtn, &QPushButton::clicked, this, [this]()
        {
            m_pViewStack->setCurrentIndex(0);
            m_pGlWidget->SetTopView();
        });
    connect(resetBtn, &QPushButton::clicked, this, [this]()
        {
            m_pViewStack->setCurrentIndex(0);
            m_pGlWidget->ResetView();
        });
    connect(exportStlBtn, &QPushButton::clicked, this, [this]()
        {
            if (!m_mesh.IsValid())
            {
                QMessageBox::information(this, "导出 STL", "尚未加载工件模型。");
                return;
            }
            const QString defaultPath = m_laserDir.isEmpty()
                ? QStringLiteral("WorkpieceMesh.stl")
                : QDir(m_laserDir).filePath("PreciseLaserPoint_WorkpieceMesh.stl");
            const QString path = QFileDialog::getSaveFileName(
                this, "导出 STL", defaultPath, "STL 网格 (*.stl)");
            if (path.isEmpty())
            {
                return;
            }
            QString error;
            QApplication::setOverrideCursor(Qt::WaitCursor);
            const bool ok = WorkpieceMeshBuilder::SaveMeshStl(path, m_mesh, error);
            QApplication::restoreOverrideCursor();
            if (!ok)
            {
                QMessageBox::warning(this, "导出 STL", error);
                return;
            }
            m_pInfoLabel->setText(QString("已导出 STL：%1（三角形 %2）")
                .arg(QDir::toNativeSeparators(path))
                .arg(m_mesh.indices.size() / 3));
        });
}

void WorkpieceMeshViewerDialog::ApplyStyle()
{
    setStyleSheet(QString(
        "QDialog { background: #101820; color: #E8F1F2; }"
        "QLabel { color: #B8C7CC; }"
        "QPushButton { background: #1F3542; color: #F4FAFA; border: 1px solid #3C6475; border-radius: 8px; padding: 6px 12px; }"
        "QPushButton:hover { background: #2C5364; border-color: #63C7D1; }"
        "QScrollArea { border: 1px solid #2E4656; border-radius: 8px; background: #0B1117; }"));
}

void WorkpieceMeshViewerDialog::StartLoad(const QString& laserDir)
{
    if (m_bBuilding)
    {
        return;  // 正在后台生成，避免重入
    }
    m_laserDir = laserDir;
    m_heightMapCache = QImage();
    setWindowTitle(QString("工件模型 - %1").arg(QFileInfo(laserDir).dir().dirName()));

    const QString cachePath = WorkpieceMeshBuilder::MeshCachePath(laserDir);
    if (WorkpieceMeshBuilder::IsMeshCacheValid(cachePath))
    {
        FinishLoadFromCache();  // 二进制缓存亚秒级，同步加载
        return;
    }

    const QString cloudPath = QDir(laserDir).filePath("PreciseLaserPoint_WorkpieceCloud.txt");
    if (!QFileInfo::exists(cloudPath))
    {
        m_pInfoLabel->setText(QString("缺少完整点云文件，无法生成工件模型：%1")
            .arg(QDir::toNativeSeparators(cloudPath)));
        return;
    }

    // 首次/旧版本缓存：后台线程生成（解析 150MB 级文本最耗时），进度条 + 可取消，不阻塞界面。
    m_bBuilding = true;
    m_pInfoLabel->setText("正在后台生成工件模型缓存…");
    if (m_pProgress == nullptr)
    {
        m_pProgress = new QProgressDialog(this);
        m_pProgress->setWindowTitle("生成工件模型");
        m_pProgress->setWindowModality(Qt::WindowModal);
        m_pProgress->setCancelButtonText("取消");
        m_pProgress->setRange(0, 100);
        m_pProgress->setMinimumDuration(0);
        m_pProgress->setAutoClose(false);
        m_pProgress->setAutoReset(false);
    }
    m_pProgress->setLabelText("正在生成工件模型（首次需要解析完整点云）…");
    m_pProgress->setValue(0);
    m_pProgress->show();

    auto cancelFlag = std::make_shared<std::atomic_bool>(false);
    // UniqueConnection 不支持 lambda 槽（Debug 下断言），复用进度框时先断开旧连接再接新取消标志。
    disconnect(m_pProgress, &QProgressDialog::canceled, nullptr, nullptr);
    connect(m_pProgress, &QProgressDialog::canceled, this,
        [cancelFlag]() { cancelFlag->store(true); });

    std::thread([this, laserDir, cloudPath, cancelFlag]()
        {
            // 进度回调在后台线程触发，更新 UI 一律排队投递；this 销毁后排队调用自动丢弃。
            const WorkpieceMeshBuilder::ProgressCallback progressCb =
                [this, cancelFlag](int percent, const QString& stage) -> bool
                {
                    if (cancelFlag->load())
                    {
                        return false;
                    }
                    QMetaObject::invokeMethod(this, [this, percent, stage]()
                        {
                            if (m_pProgress != nullptr)
                            {
                                m_pProgress->setValue(percent);
                                m_pProgress->setLabelText(QString("正在生成工件模型：%1…").arg(stage));
                            }
                        }, Qt::QueuedConnection);
                    return true;
                };
            QString buildError;
            const bool ok = WorkpieceMeshBuilder::EnsureMeshCache(laserDir, cloudPath, buildError, progressCb);
            QMetaObject::invokeMethod(this, [this, ok, buildError]()
                {
                    OnBuildFinished(ok, buildError);
                }, Qt::QueuedConnection);
        }).detach();
}

void WorkpieceMeshViewerDialog::OnBuildFinished(bool ok, const QString& error)
{
    m_bBuilding = false;
    if (m_pProgress != nullptr)
    {
        m_pProgress->hide();
    }
    if (!ok)
    {
        if (error == QStringLiteral("已取消"))
        {
            m_pInfoLabel->setText("已取消生成工件模型。");
        }
        else
        {
            m_pInfoLabel->setText("生成工件模型失败：" + error);
            QMessageBox::warning(this, "工件模型", error);
        }
        return;
    }
    FinishLoadFromCache();
}

void WorkpieceMeshViewerDialog::FinishLoadFromCache()
{
    const QString cachePath = WorkpieceMeshBuilder::MeshCachePath(m_laserDir);
    QString error;
    QApplication::setOverrideCursor(Qt::WaitCursor);
    const bool loaded = WorkpieceMeshBuilder::LoadMeshPly(cachePath, m_mesh, error);
    QApplication::restoreOverrideCursor();
    if (!loaded)
    {
        m_pInfoLabel->setText("加载模型缓存失败：" + error);
        QMessageBox::warning(this, "工件模型", error);
        return;
    }
    m_pGlWidget->SetMesh(m_mesh);
    m_pViewStack->setCurrentIndex(0);
    m_pInfoLabel->setText(QString("模型缓存：%1（顶点 %2 / 三角形 %3）— 左键旋转 / 右键平移 / 滚轮缩放 / 双击复位")
        .arg(QDir::toNativeSeparators(cachePath))
        .arg(m_mesh.vertices.size())
        .arg(m_mesh.indices.size() / 3));
}

void WorkpieceMeshViewerDialog::ShowHeightMap()
{
    if (m_heightMapCache.isNull() && m_mesh.IsValid())
    {
        QApplication::setOverrideCursor(Qt::WaitCursor);
        m_heightMapCache = WorkpieceMeshBuilder::RenderHeightMap(m_mesh);
        QApplication::restoreOverrideCursor();
    }
    if (!m_heightMapCache.isNull())
    {
        m_pHeightMapLabel->setPixmap(QPixmap::fromImage(m_heightMapCache));
        m_pHeightMapLabel->adjustSize();
        m_pViewStack->setCurrentIndex(1);
    }
}
