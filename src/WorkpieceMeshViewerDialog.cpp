#include "WorkpieceMeshViewerDialog.h"

#include "WindowStyleHelper.h"

#include <QApplication>
#include <QDir>
#include <QFile>
#include <QFileDialog>
#include <QDoubleSpinBox>
#include <QFileInfo>
#include <QFormLayout>
#include <QInputDialog>
#include <QMenu>
#include <QMessageBox>
#include <QSpinBox>
#include <QToolButton>
#include <QProgressDialog>

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>
#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QKeyEvent>
#include <QMatrix4x4>
#include <QMouseEvent>
#include <QOpenGLBuffer>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLWidget>
#include <QPainter>
#include <QPaintEvent>
#include <QPolygon>
#include <QPushButton>
#include <QRubberBand>
#include <QScrollArea>
#include <QStackedWidget>
#include <QSurfaceFormat>
#include <QVBoxLayout>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <vector>

// 套索选区覆盖层：透明、鼠标穿透的子控件，只负责把当前套索路径画成红色虚线
// 闭合多边形。与 QRubberBand 同思路——选区绘制走这层 2D overlay，绝不触发底层
// 百万级点云的 GL 重绘（之前用 paintGL+QPainter 拖一下卡一下）。
class LassoSelectionOverlay : public QWidget
{
public:
    explicit LassoSelectionOverlay(QWidget* parent) : QWidget(parent)
    {
        setAttribute(Qt::WA_TransparentForMouseEvents, true);  // 鼠标事件穿透到 GL 控件
        setAttribute(Qt::WA_NoSystemBackground, true);
        setAttribute(Qt::WA_TranslucentBackground, true);
        hide();
    }
    void SetPath(const QVector<QPoint>& pts)
    {
        m_pts = pts;
        update();
    }
    void ClearPath()
    {
        m_pts.clear();
        update();
    }

protected:
    void paintEvent(QPaintEvent*) override
    {
        if (m_pts.size() < 2)
        {
            return;
        }
        QPainter p(this);
        p.setRenderHint(QPainter::Antialiasing, true);
        QPen pen(QColor(255, 70, 70), 1.5);
        pen.setStyle(Qt::DashLine);
        p.setPen(pen);
        p.setBrush(QColor(255, 70, 70, 40));  // 半透明红填充提示选区范围
        p.drawPolygon(QPolygon(m_pts));
    }

private:
    QVector<QPoint> m_pts;
};

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
        m_pLassoOverlay = new LassoSelectionOverlay(this);
    }

    ~WorkpieceMeshGLWidget() override
    {
        makeCurrent();
        m_vertexBuffer.destroy();
        m_indexBuffer.destroy();
        m_vao.destroy();
        m_editVertexBuffer.destroy();
        m_editFlagBuffer.destroy();
        m_editVao.destroy();
        delete m_pProgram;
        doneCurrent();
    }

    void SetMesh(const WorkpieceMeshBuilder::Mesh& mesh, bool keepView = false)
    {
        m_pendingMesh = mesh;
        m_meshDirty = true;
        ComputeBounds(mesh);
        if (!keepView)
        {
            ResetView();
        }
        if (isValid())
        {
            update();
        }
    }

    // 进入/退出框选编辑：cloud 指向对话框持有的原始点云（删除时原地压缩），
    // nullptr 退出编辑回到网格显示。不动相机——用户正对着毛刺的视角直接框选。
    // 交互：左键拖框=选中（亮红，可多框累加），Delete/按钮=删除所选，Esc=清空选择。
    void SetEditCloud(QVector<RobotCalculation::IndexedPoint3D>* cloud,
        std::function<void(const QVector<RobotCalculation::IndexedPoint3D>&)> onRemoved = nullptr,
        std::function<void(qsizetype)> onSelectionChanged = nullptr)
    {
        m_pEditCloud = cloud;
        m_onEditRemoved = std::move(onRemoved);
        m_onSelectionChanged = std::move(onSelectionChanged);
        m_rubberActive = false;
        m_editDirty = (cloud != nullptr);
        update();
    }

    // 编辑点云内容变化（撤销恢复）后重新上传 GPU（选中状态同时清空）。
    void RefreshEditCloud()
    {
        m_editDirty = true;
        update();
    }

    qsizetype SelectedCount() const { return m_selectedCount; }

    // 删除所有选中点：原地压缩点云 + 通知对话框（撤销栈/统计），重传 GPU。
    void DeleteSelectedEditPoints()
    {
        if (m_pEditCloud == nullptr || m_selectedCount <= 0
            || m_editFlags.size() != m_pEditCloud->size())
        {
            return;
        }
        QApplication::setOverrideCursor(Qt::WaitCursor);
        QVector<RobotCalculation::IndexedPoint3D>& cloud = *m_pEditCloud;
        QVector<RobotCalculation::IndexedPoint3D> removed;
        removed.reserve(int(m_selectedCount));
        qsizetype keep = 0;
        for (qsizetype i = 0; i < cloud.size(); ++i)
        {
            if (m_editFlags[i] > 0.5f)
            {
                removed.append(cloud[i]);
            }
            else
            {
                cloud[keep++] = cloud[i];
            }
        }
        cloud.resize(keep);
        m_editDirty = true;  // 全量重传（pos+flags 清零）
        QApplication::restoreOverrideCursor();
        update();
        if (!removed.isEmpty() && m_onEditRemoved)
        {
            m_onEditRemoved(removed);
        }
    }

    void ClearSelection()
    {
        if (m_selectedCount <= 0)
        {
            return;
        }
        std::fill(m_editFlags.begin(), m_editFlags.end(), 0.0f);
        m_selectedCount = 0;
        m_flagsDirty = true;
        update();
        if (m_onSelectionChanged)
        {
            m_onSelectionChanged(0);
        }
    }

    // 反选：当前选区取反（红↔灰）。框多了先框「该保留的」再反选，或直接反选删外部。
    void InvertSelection()
    {
        if (m_pEditCloud == nullptr || m_editFlags.isEmpty()) return;
        qsizetype cnt = 0;
        for (float& f : m_editFlags)
        {
            f = (f > 0.5f) ? 0.0f : 1.0f;
            if (f > 0.5f) ++cnt;
        }
        m_selectedCount = cnt;
        m_flagsDirty = true;
        update();
        if (m_onSelectionChanged) m_onSelectionChanged(m_selectedCount);
    }

    // 隔离显示：仅渲染选中（红）点、其余隐藏——精修时不被周围点遮挡；纯视图态、可逆。
    void SetIsolateMode(bool on)
    {
        m_isolateMode = on;
        update();
    }

    // 套索模式开关：开启后左键拖动画自由多边形，松开按 point-in-polygon 选中；
    // 与矩形框选互斥（同一左键拖动二选一）。关闭时立即收起覆盖层与残留路径。
    void SetLassoMode(bool on)
    {
        m_lassoMode = on;
        if (!on)
        {
            m_lassoActive = false;
            m_lassoPath.clear();
            if (m_pLassoOverlay != nullptr)
            {
                m_pLassoOverlay->ClearPath();
                m_pLassoOverlay->hide();
            }
        }
    }

    // 统计离群删除（SOR）：把离群点追加到选中集（不删，复用 Delete 让用户确认）。
    // k=邻居数，stdMul=标准差倍数；点的 k 近邻平均距离 > 全局均值 + stdMul·std 判离群。
    // 返回本次新增选中数。纯 Eigen + 均匀网格哈希 + 多线程，无第三方依赖。
    qsizetype SelectStatisticalOutliers(int k, double stdMul)
    {
        if (m_pEditCloud == nullptr || m_editFlags.size() != m_pEditCloud->size()
            || m_editPositions.size() != m_pEditCloud->size() * 3)
        {
            return 0;
        }
        const qsizetype n = m_pEditCloud->size();
        if (k < 1 || n < qsizetype(k) + 1) return 0;
        const float* pos = m_editPositions.constData();

        Eigen::Vector3f lo(pos[0], pos[1], pos[2]);
        Eigen::Vector3f hi = lo;
        for (qsizetype i = 1; i < n; ++i)
        {
            const Eigen::Vector3f p(pos[i * 3], pos[i * 3 + 1], pos[i * 3 + 2]);
            lo = lo.cwiseMin(p);
            hi = hi.cwiseMax(p);
        }
        const Eigen::Vector3f span = hi - lo;
        double cell = double(span.norm()) / std::cbrt(double(n)) * 2.0;  // ≈2× 平均点间距
        if (cell < 1e-6) cell = 1.0;
        const int nx = std::max(1, int(span.x() / cell) + 1);
        const int ny = std::max(1, int(span.y() / cell) + 1);
        const int nz = std::max(1, int(span.z() / cell) + 1);
        if (static_cast<long long>(nx) * ny * nz > 16LL * 1024 * 1024) return 0;  // 防爆内存
        std::vector<std::vector<int>> grid(static_cast<size_t>(static_cast<long long>(nx) * ny * nz));
        auto cellOf = [&](float px, float py, float pz, int& cx, int& cy, int& cz)
        {
            cx = std::clamp(int((px - lo.x()) / cell), 0, nx - 1);
            cy = std::clamp(int((py - lo.y()) / cell), 0, ny - 1);
            cz = std::clamp(int((pz - lo.z()) / cell), 0, nz - 1);
        };
        for (qsizetype i = 0; i < n; ++i)
        {
            int cx, cy, cz;
            cellOf(pos[i * 3], pos[i * 3 + 1], pos[i * 3 + 2], cx, cy, cz);
            grid[static_cast<size_t>((cz * ny + cy) * nx + cx)].push_back(int(i));
        }

        std::vector<float> meanDist(size_t(n), 0.0f);
        const int threads = std::clamp(int(std::thread::hardware_concurrency()), 1, 16);
        const qsizetype chunk = (n + threads - 1) / threads;
        const int maxRing = std::max(nx, std::max(ny, nz));
        std::vector<std::thread> workers;
        for (int t = 0; t < threads; ++t)
        {
            const qsizetype b = qsizetype(t) * chunk;
            const qsizetype e = std::min(n, b + chunk);
            if (b >= e) break;
            workers.emplace_back([=, &grid, &meanDist]()
            {
                std::vector<float> d2;
                for (qsizetype i = b; i < e; ++i)
                {
                    const float px = pos[i * 3], py = pos[i * 3 + 1], pz = pos[i * 3 + 2];
                    int cx, cy, cz;
                    cellOf(px, py, pz, cx, cy, cz);
                    d2.clear();
                    for (int r = 1; r <= maxRing; ++r)  // 邻域逐圈扩，直到候选够 k
                    {
                        d2.clear();
                        for (int z = std::max(0, cz - r); z <= std::min(nz - 1, cz + r); ++z)
                            for (int y = std::max(0, cy - r); y <= std::min(ny - 1, cy + r); ++y)
                                for (int x = std::max(0, cx - r); x <= std::min(nx - 1, cx + r); ++x)
                                    for (int j : grid[static_cast<size_t>((z * ny + y) * nx + x)])
                                    {
                                        if (qsizetype(j) == i) continue;
                                        const float dx = pos[j * 3] - px;
                                        const float dy = pos[j * 3 + 1] - py;
                                        const float dz = pos[j * 3 + 2] - pz;
                                        d2.push_back(dx * dx + dy * dy + dz * dz);
                                    }
                        if (qsizetype(d2.size()) >= qsizetype(k) || r >= maxRing) break;
                    }
                    if (d2.empty()) { meanDist[size_t(i)] = 0.0f; continue; }
                    const int kk = int(std::min(qsizetype(k), qsizetype(d2.size())));
                    std::nth_element(d2.begin(), d2.begin() + kk, d2.end());
                    double s = 0.0;
                    for (int mIdx = 0; mIdx < kk; ++mIdx) s += std::sqrt(double(d2[size_t(mIdx)]));
                    meanDist[size_t(i)] = float(s / kk);
                }
            });
        }
        for (auto& w : workers) w.join();

        double sum = 0.0;
        for (float d : meanDist) sum += d;
        const double mean = sum / double(n);
        double var = 0.0;
        for (float d : meanDist) var += (d - mean) * (d - mean);
        const double sd = std::sqrt(var / double(n));
        const double thresh = mean + stdMul * sd;

        qsizetype added = 0;
        for (qsizetype i = 0; i < n; ++i)
            if (meanDist[size_t(i)] > thresh && m_editFlags[i] < 0.5f)
            {
                m_editFlags[i] = 1.0f;
                ++added;
            }
        m_selectedCount += added;
        m_flagsDirty = true;
        update();
        if (m_onSelectionChanged) m_onSelectionChanged(m_selectedCount);
        return added;
    }

    // 连通域去飞点：把「与主体不连通的小簇」标记为选中（不删，复用 Delete 确认）。
    // voxel=体素边长（同体素或 26 邻体素内有点即视为连通）；minClusterPts=保留簇的最小点数，
    // 小于此的连通簇判为飞点。返回本次新增选中数。纯 Eigen + 体素哈希并查集，无第三方依赖。
    qsizetype SelectSmallClusters(double voxel, int minClusterPts)
    {
        if (m_pEditCloud == nullptr || m_editFlags.size() != m_pEditCloud->size()
            || m_editPositions.size() != m_pEditCloud->size() * 3)
        {
            return 0;
        }
        const qsizetype n = m_pEditCloud->size();
        if (n < 1 || voxel <= 1e-6) return 0;
        const float* pos = m_editPositions.constData();

        Eigen::Vector3f lo(pos[0], pos[1], pos[2]);
        Eigen::Vector3f hi = lo;
        for (qsizetype i = 1; i < n; ++i)
        {
            const Eigen::Vector3f p(pos[i * 3], pos[i * 3 + 1], pos[i * 3 + 2]);
            lo = lo.cwiseMin(p);
            hi = hi.cwiseMax(p);
        }
        const Eigen::Vector3f span = hi - lo;
        const int nx = std::max(1, int(span.x() / voxel) + 1);
        const int ny = std::max(1, int(span.y() / voxel) + 1);
        const int nz = std::max(1, int(span.z() / voxel) + 1);
        if (static_cast<long long>(nx) * ny * nz > 64LL * 1024 * 1024) return 0;
        auto voxelOf = [&](float x, float y, float z) -> long long
        {
            const int cx = std::clamp(int((x - lo.x()) / voxel), 0, nx - 1);
            const int cy = std::clamp(int((y - lo.y()) / voxel), 0, ny - 1);
            const int cz = std::clamp(int((z - lo.z()) / voxel), 0, nz - 1);
            return (static_cast<long long>(cz) * ny + cy) * nx + cx;
        };

        std::unordered_map<long long, int> voxelCount;   // 体素 → 点数
        std::vector<long long> pointVoxel(static_cast<size_t>(n));
        voxelCount.reserve(size_t(n));
        for (qsizetype i = 0; i < n; ++i)
        {
            const long long v = voxelOf(pos[i * 3], pos[i * 3 + 1], pos[i * 3 + 2]);
            pointVoxel[size_t(i)] = v;
            ++voxelCount[v];
        }

        // 并查集（体素粒度）：26 邻接的非空体素合并。
        std::unordered_map<long long, long long> parent;
        parent.reserve(voxelCount.size());
        for (const auto& kv : voxelCount) parent[kv.first] = kv.first;
        auto findRoot = [&parent](long long a) -> long long
        {
            while (parent[a] != a) { parent[a] = parent[parent[a]]; a = parent[a]; }
            return a;
        };
        for (const auto& kv : voxelCount)
        {
            const long long v = kv.first;
            const int cz = int(v / (static_cast<long long>(nx) * ny));
            const long long rem = v % (static_cast<long long>(nx) * ny);
            const int cy = int(rem / nx);
            const int cx = int(rem % nx);
            for (int dz = -1; dz <= 1; ++dz)
                for (int dy = -1; dy <= 1; ++dy)
                    for (int dx = -1; dx <= 1; ++dx)
                    {
                        if (dx == 0 && dy == 0 && dz == 0) continue;
                        const int x = cx + dx, y = cy + dy, z = cz + dz;
                        if (x < 0 || x >= nx || y < 0 || y >= ny || z < 0 || z >= nz) continue;
                        const long long nv = (static_cast<long long>(z) * ny + y) * nx + x;
                        if (voxelCount.find(nv) == voxelCount.end()) continue;
                        const long long ra = findRoot(v), rb = findRoot(nv);
                        if (ra != rb) parent[ra] = rb;
                    }
        }

        std::unordered_map<long long, int> clusterCount;  // 簇根 → 总点数
        clusterCount.reserve(voxelCount.size());
        for (const auto& kv : voxelCount) clusterCount[findRoot(kv.first)] += kv.second;

        qsizetype added = 0;
        for (qsizetype i = 0; i < n; ++i)
        {
            if (m_editFlags[i] > 0.5f) continue;
            if (clusterCount[findRoot(pointVoxel[size_t(i)])] < minClusterPts)
            {
                m_editFlags[i] = 1.0f;
                ++added;
            }
        }
        m_selectedCount += added;
        m_flagsDirty = true;
        update();
        if (m_onSelectionChanged) m_onSelectionChanged(m_selectedCount);
        return added;
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
            "layout(location=2) in float inSelected;\n"  // 编辑模式框选标记（网格 VAO 不启用，恒 0）
            "uniform mat4 uMvp;\n"
            "uniform int uColorAxis;\n"  // 伪彩轴=包围盒跨度最小的轴（表面偏移方向，姿态无关）
            "uniform float uZMin;\n"
            "uniform float uZRange;\n"
            "out vec3 vNormal;\n"
            "out float vHeight;\n"
            "out float vSelected;\n"
            "void main(){\n"
            "  gl_Position = uMvp * vec4(inPos, 1.0);\n"
            "  vNormal = inNormal;\n"
            "  vHeight = clamp((inPos[uColorAxis] - uZMin) / uZRange, 0.0, 1.0);\n"
            "  vSelected = inSelected;\n"
            "  gl_PointSize = 2.0;\n"
            "}\n");
        m_pProgram->addShaderFromSourceCode(QOpenGLShader::Fragment,
            "#version 330 core\n"
            "in vec3 vNormal;\n"
            "in float vHeight;\n"
            "in float vSelected;\n"
            "uniform vec3 uLightDir;\n"
            "uniform float uUseLight;\n"
            "uniform float uIsolateMode;\n"
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
            "  if (uIsolateMode > 0.5 && vSelected < 0.5) discard;\n"
            "  vec3 base = turbo(vHeight);\n"
            "  float diff = abs(dot(normalize(vNormal), normalize(uLightDir)));\n"  // 双面光照（绕序/朝向无关）
            "  float k = mix(1.0, 0.30 + 0.70 * diff, uUseLight);\n"
            "  vec3 color = base * k;\n"
            "  if (vSelected > 0.5) { color = vec3(1.0, 0.28, 0.28); }\n"  // 选中点亮红
            "  fragColor = vec4(color, 1.0);\n"
            "}\n");
        m_pProgram->link();

        m_vao.create();
        m_vertexBuffer = QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
        m_vertexBuffer.create();
        m_indexBuffer = QOpenGLBuffer(QOpenGLBuffer::IndexBuffer);
        m_indexBuffer.create();
        m_editVao.create();
        m_editVertexBuffer = QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
        m_editVertexBuffer.create();
        m_editFlagBuffer = QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
        m_editFlagBuffer.create();
        glEnable(GL_PROGRAM_POINT_SIZE);
    }

    void paintGL() override
    {
        // 防御性重申依赖的 GL 开关（Qt 内部若用 QPainter 混合渲染会关掉
        // GL_MULTISAMPLE 等且不恢复；橡皮筋已改 QRubberBand overlay，不再混合）。
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_PROGRAM_POINT_SIZE);
        glEnable(GL_MULTISAMPLE);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        if (m_meshDirty)
        {
            UploadMesh();
        }
        if (m_editDirty)
        {
            UploadEditCloud();
        }
        if (m_flagsDirty)
        {
            UploadEditFlags();
        }
        if (m_pProgram == nullptr)
        {
            return;
        }

        m_pProgram->bind();
        m_pProgram->setUniformValue("uMvp", BuildMvpMatrix());
        m_pProgram->setUniformValue("uColorAxis", m_colorAxis);
        m_pProgram->setUniformValue("uZMin", m_zMin);
        m_pProgram->setUniformValue("uZRange", std::max(m_zMax - m_zMin, 1e-3f));
        m_pProgram->setUniformValue("uLightDir", QVector3D(-0.4f, -0.3f, 1.0f));
        m_pProgram->setUniformValue("uIsolateMode", (m_pEditCloud != nullptr && m_isolateMode) ? 1.0f : 0.0f);

        if (m_pEditCloud != nullptr)
        {
            if (m_editPointCount > 0)
            {
                m_pProgram->setUniformValue("uUseLight", 0.0f);
                glVertexAttrib3f(1, 0.0f, 0.0f, 1.0f);  // 编辑 VBO 只有坐标，法线给常量占位
                m_editVao.bind();
                glDrawArrays(GL_POINTS, 0, m_editPointCount);
                m_editVao.release();
            }
        }
        else if (m_indexCount > 0)
        {
            m_pProgram->setUniformValue("uUseLight", m_solidMode ? 1.0f : 0.0f);
            glVertexAttrib1f(2, 0.0f);  // 网格 VAO 不带选中属性，常量 0
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
        }
        m_pProgram->release();
    }

    void mousePressEvent(QMouseEvent* event) override
    {
        m_lastMousePos = event->position();
        // 编辑模式：纯左键按下开始框选；Ctrl+左键保留旋转。
        // 橡皮筋用 QRubberBand overlay 子控件——拖动只移动覆盖层，
        // 完全不触发百万级点云的 GL 重绘（之前用 paintGL+QPainter 拖一下卡一下）。
        if (m_pEditCloud != nullptr && event->button() == Qt::LeftButton
            && !(event->modifiers() & Qt::ControlModifier))
        {
            if (m_lassoMode)
            {
                m_lassoActive = true;
                m_lassoSubtract = (event->modifiers() & Qt::ShiftModifier) != 0;  // Shift+套索=减选
                m_lassoPath.clear();
                m_lassoPath.append(event->position().toPoint());
                if (m_pLassoOverlay != nullptr)
                {
                    m_pLassoOverlay->setGeometry(rect());  // 对齐当前控件尺寸
                    m_pLassoOverlay->raise();
                    m_pLassoOverlay->ClearPath();
                    m_pLassoOverlay->show();
                }
            }
            else
            {
                m_rubberActive = true;
                m_rubberSubtract = (event->modifiers() & Qt::ShiftModifier) != 0;  // Shift+拖框=减选
                m_rubberStart = event->position().toPoint();
                if (m_pRubberBand == nullptr)
                {
                    m_pRubberBand = new QRubberBand(QRubberBand::Rectangle, this);
                }
                m_pRubberBand->setGeometry(QRect(m_rubberStart, QSize()));
                m_pRubberBand->show();
            }
        }
    }

    void mouseMoveEvent(QMouseEvent* event) override
    {
        const QPointF delta = event->position() - m_lastMousePos;
        m_lastMousePos = event->position();
        if (m_lassoActive && (event->buttons() & Qt::LeftButton))
        {
            const QPoint p = event->position().toPoint();
            // 抽稀：与上一采样点间距 ≥3px 才记录，避免路径点过密。
            if (m_lassoPath.isEmpty() || (p - m_lassoPath.back()).manhattanLength() >= 3)
            {
                m_lassoPath.append(p);
                if (m_pLassoOverlay != nullptr)
                {
                    m_pLassoOverlay->SetPath(m_lassoPath);
                }
            }
            return;
        }
        if (m_rubberActive && (event->buttons() & Qt::LeftButton))
        {
            m_pRubberBand->setGeometry(
                QRect(m_rubberStart, event->position().toPoint()).normalized());
            return;
        }
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

    void mouseReleaseEvent(QMouseEvent* event) override
    {
        if (m_lassoActive && event->button() == Qt::LeftButton)
        {
            m_lassoActive = false;
            if (m_pLassoOverlay != nullptr)
            {
                m_pLassoOverlay->ClearPath();
                m_pLassoOverlay->hide();
            }
            if (m_lassoPath.size() >= 3)
            {
                SelectPointsInPolygon(m_lassoPath, m_lassoSubtract);
            }
            m_lassoPath.clear();
            return;
        }
        if (m_rubberActive && event->button() == Qt::LeftButton)
        {
            m_rubberActive = false;
            if (m_pRubberBand != nullptr)
            {
                m_pRubberBand->hide();
            }
            const QRect rect = QRect(m_rubberStart, event->position().toPoint()).normalized();
            if (rect.width() >= 4 && rect.height() >= 4)
            {
                SelectPointsInRect(rect, m_rubberSubtract);
            }
        }
    }

    void keyPressEvent(QKeyEvent* event) override
    {
        if (m_pEditCloud != nullptr)
        {
            if (event->key() == Qt::Key_Delete || event->key() == Qt::Key_Backspace)
            {
                event->accept();
                DeleteSelectedEditPoints();
                return;
            }
            if (event->key() == Qt::Key_Escape)
            {
                event->accept();  // 编辑模式下 Esc 只清选择，不让对话框关闭
                ClearSelection();
                return;
            }
        }
        QOpenGLWidget::keyPressEvent(event);
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
    QMatrix4x4 BuildMvpMatrix() const
    {
        QMatrix4x4 projection;
        const float aspect = height() > 0 ? float(width()) / float(height()) : 1.0f;
        projection.perspective(40.0f, aspect, m_boundRadius * 0.01f, m_boundRadius * 20.0f);
        QMatrix4x4 view;
        view.translate(m_panX, m_panY, -m_distance);
        view.rotate(m_pitchDeg, 1.0f, 0.0f, 0.0f);
        view.rotate(m_yawDeg, 0.0f, 0.0f, 1.0f);
        view.translate(-m_center);
        return projection * view;
    }

    void UploadEditCloud()
    {
        m_editDirty = false;
        m_flagsDirty = false;
        m_editPointCount = 0;
        m_selectedCount = 0;
        if (m_pEditCloud == nullptr || m_pEditCloud->isEmpty())
        {
            m_editFlags.clear();
            m_editPositions.clear();
            return;
        }
        // float 坐标数组保留为成员：框选投影直接扫连续 float 内存，
        // 免去每次从 Eigen double 转换的开销。
        m_editPositions.clear();
        m_editPositions.reserve(m_pEditCloud->size() * 3);
        for (const auto& p : *m_pEditCloud)
        {
            m_editPositions << float(p.point.x()) << float(p.point.y()) << float(p.point.z());
        }
        m_editFlags.fill(0.0f, m_pEditCloud->size());
        m_editVao.bind();
        m_editVertexBuffer.bind();
        m_editVertexBuffer.allocate(m_editPositions.constData(),
            int(m_editPositions.size() * sizeof(float)));
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
        glDisableVertexAttribArray(1);
        m_editFlagBuffer.bind();
        m_editFlagBuffer.allocate(m_editFlags.constData(), int(m_editFlags.size() * sizeof(float)));
        glEnableVertexAttribArray(2);
        glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, sizeof(float), nullptr);
        m_editVao.release();
        m_editPointCount = int(m_pEditCloud->size());
    }

    // 选中标记变化（框选/清选）只重传 flag 缓冲（每点 4 字节），不动坐标数据；
    // 容量够时用 write 原地覆写，避免 glBufferData 重分配。
    void UploadEditFlags()
    {
        m_flagsDirty = false;
        if (m_editPointCount <= 0 || m_editFlags.size() != m_editPointCount)
        {
            return;
        }
        const int bytes = int(m_editFlags.size() * sizeof(float));
        m_editFlagBuffer.bind();
        if (m_editFlagBuffer.size() >= bytes)
        {
            m_editFlagBuffer.write(0, m_editFlags.constData(), bytes);
        }
        else
        {
            m_editFlagBuffer.allocate(m_editFlags.constData(), bytes);
        }
        m_editFlagBuffer.release();
    }

    // 屏幕矩形框选：与渲染同一 MVP 把点投到窗口坐标，框内的点标记选中（穿透
    // 全部深度，与 CloudCompare 的 segment 工具一致——旋转到毛刺侧影方向再框）。
    // 多次框选累加；删除由 Delete/按钮显式触发。
    // 百万级点的投影判定按硬件线程数分块并行，矩阵乘手动展开（列主序），
    // 每线程只写自己区间的 flags，无锁。
    void SelectPointsInRect(const QRect& rect, bool subtract)
    {
        if (m_pEditCloud == nullptr || m_pEditCloud->isEmpty()
            || m_editFlags.size() != m_pEditCloud->size()
            || m_editPositions.size() != m_pEditCloud->size() * 3)
        {
            return;
        }
        const QMatrix4x4 mvp = BuildMvpMatrix();
        const float* m = mvp.constData();  // QMatrix4x4 内部列主序
        const float w = float(width());
        const float h = float(height());
        const float sxMin = float(rect.left());
        const float sxMax = float(rect.right() + 1);
        const float syMin = float(rect.top());
        const float syMax = float(rect.bottom() + 1);
        const qsizetype total = m_pEditCloud->size();
        const float* pos = m_editPositions.constData();
        float* flags = m_editFlags.data();
        const int threadCount = std::clamp(int(std::thread::hardware_concurrency()), 1, 16);
        const qsizetype chunk = (total + threadCount - 1) / threadCount;
        std::vector<qsizetype> chunkCounts(size_t(threadCount), 0);
        std::vector<std::thread> workers;
        workers.reserve(size_t(threadCount));
        for (int t = 0; t < threadCount; ++t)
        {
            const qsizetype begin = qsizetype(t) * chunk;
            const qsizetype end = std::min(total, begin + chunk);
            if (begin >= end)
            {
                break;
            }
            workers.emplace_back([=, &chunkCounts]()
                {
                    qsizetype localCount = 0;
                    for (qsizetype i = begin; i < end; ++i)
                    {
                        // 加选只看未选点、减选只看已选点（其余状态不变，跳过）。
                        if (subtract ? (flags[i] < 0.5f) : (flags[i] > 0.5f))
                        {
                            continue;
                        }
                        const float x = pos[i * 3];
                        const float y = pos[i * 3 + 1];
                        const float z = pos[i * 3 + 2];
                        const float cw = m[3] * x + m[7] * y + m[11] * z + m[15];
                        if (cw <= 1e-6f)
                        {
                            continue;
                        }
                        const float inv = 1.0f / cw;
                        const float sx = ((m[0] * x + m[4] * y + m[8] * z + m[12]) * inv * 0.5f + 0.5f) * w;
                        if (sx < sxMin || sx >= sxMax)
                        {
                            continue;
                        }
                        const float sy = (0.5f - (m[1] * x + m[5] * y + m[9] * z + m[13]) * inv * 0.5f) * h;
                        if (sy < syMin || sy >= syMax)
                        {
                            continue;
                        }
                        flags[i] = subtract ? 0.0f : 1.0f;
                        ++localCount;
                    }
                    chunkCounts[size_t(t)] = localCount;
                });
        }
        for (auto& worker : workers)
        {
            worker.join();
        }
        qsizetype newlySelected = 0;
        for (const qsizetype c : chunkCounts)
        {
            newlySelected += c;
        }
        if (newlySelected == 0)
        {
            return;
        }
        m_selectedCount += subtract ? -newlySelected : newlySelected;
        m_flagsDirty = true;
        update();
        if (m_onSelectionChanged)
        {
            m_onSelectionChanged(m_selectedCount);
        }
    }

    // 套索/多边形选区：把每个点投影到屏幕，按奇偶射线法（PNPoly）判断是否落在
    // 多边形内。投影、减选语义、多线程分块均同 SelectPointsInRect；先用多边形屏幕
    // 包围盒做粗筛，落框内者再走 point-in-polygon 精判。
    void SelectPointsInPolygon(const QVector<QPoint>& poly, bool subtract)
    {
        if (m_pEditCloud == nullptr || m_pEditCloud->isEmpty()
            || m_editFlags.size() != m_pEditCloud->size()
            || m_editPositions.size() != m_pEditCloud->size() * 3
            || poly.size() < 3)
        {
            return;
        }
        // 多边形顶点拍平成 float 数组 + 屏幕包围盒（粗筛）。
        const int pn = poly.size();
        std::vector<float> px(static_cast<size_t>(pn));
        std::vector<float> py(static_cast<size_t>(pn));
        float bbMinX = float(poly[0].x());
        float bbMaxX = bbMinX;
        float bbMinY = float(poly[0].y());
        float bbMaxY = bbMinY;
        for (int i = 0; i < pn; ++i)
        {
            px[size_t(i)] = float(poly[i].x());
            py[size_t(i)] = float(poly[i].y());
            bbMinX = std::min(bbMinX, px[size_t(i)]);
            bbMaxX = std::max(bbMaxX, px[size_t(i)]);
            bbMinY = std::min(bbMinY, py[size_t(i)]);
            bbMaxY = std::max(bbMaxY, py[size_t(i)]);
        }
        const QMatrix4x4 mvp = BuildMvpMatrix();
        const float* m = mvp.constData();  // QMatrix4x4 内部列主序
        const float w = float(width());
        const float h = float(height());
        const qsizetype total = m_pEditCloud->size();
        const float* pos = m_editPositions.constData();
        float* flags = m_editFlags.data();
        const int threadCount = std::clamp(int(std::thread::hardware_concurrency()), 1, 16);
        const qsizetype chunk = (total + threadCount - 1) / threadCount;
        std::vector<qsizetype> chunkCounts(size_t(threadCount), 0);
        std::vector<std::thread> workers;
        workers.reserve(size_t(threadCount));
        for (int t = 0; t < threadCount; ++t)
        {
            const qsizetype begin = qsizetype(t) * chunk;
            const qsizetype end = std::min(total, begin + chunk);
            if (begin >= end)
            {
                break;
            }
            workers.emplace_back([=, &chunkCounts]()
                {
                    qsizetype localCount = 0;
                    for (qsizetype i = begin; i < end; ++i)
                    {
                        if (subtract ? (flags[i] < 0.5f) : (flags[i] > 0.5f))
                        {
                            continue;
                        }
                        const float x = pos[i * 3];
                        const float y = pos[i * 3 + 1];
                        const float z = pos[i * 3 + 2];
                        const float cw = m[3] * x + m[7] * y + m[11] * z + m[15];
                        if (cw <= 1e-6f)
                        {
                            continue;
                        }
                        const float inv = 1.0f / cw;
                        const float sx = ((m[0] * x + m[4] * y + m[8] * z + m[12]) * inv * 0.5f + 0.5f) * w;
                        if (sx < bbMinX || sx > bbMaxX)
                        {
                            continue;
                        }
                        const float sy = (0.5f - (m[1] * x + m[5] * y + m[9] * z + m[13]) * inv * 0.5f) * h;
                        if (sy < bbMinY || sy > bbMaxY)
                        {
                            continue;
                        }
                        // 奇偶射线法：自该点水平向右的射线与多边形各边交点数为奇 → 内部。
                        bool inside = false;
                        for (int a = 0, b = pn - 1; a < pn; b = a++)
                        {
                            const float yai = py[size_t(a)];
                            const float ybi = py[size_t(b)];
                            if ((yai > sy) != (ybi > sy))
                            {
                                const float xcross = px[size_t(a)]
                                    + (sy - yai) / (ybi - yai) * (px[size_t(b)] - px[size_t(a)]);
                                if (sx < xcross)
                                {
                                    inside = !inside;
                                }
                            }
                        }
                        if (!inside)
                        {
                            continue;
                        }
                        flags[i] = subtract ? 0.0f : 1.0f;
                        ++localCount;
                    }
                    chunkCounts[size_t(t)] = localCount;
                });
        }
        for (auto& worker : workers)
        {
            worker.join();
        }
        qsizetype newlySelected = 0;
        for (const qsizetype c : chunkCounts)
        {
            newlySelected += c;
        }
        if (newlySelected == 0)
        {
            return;
        }
        m_selectedCount += subtract ? -newlySelected : newlySelected;
        m_flagsDirty = true;
        update();
        if (m_onSelectionChanged)
        {
            m_onSelectionChanged(m_selectedCount);
        }
    }

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

    // 框选编辑态（点云归对话框所有，这里只持引用指针）。
    QVector<RobotCalculation::IndexedPoint3D>* m_pEditCloud = nullptr;
    std::function<void(const QVector<RobotCalculation::IndexedPoint3D>&)> m_onEditRemoved;
    std::function<void(qsizetype)> m_onSelectionChanged;
    QOpenGLVertexArrayObject m_editVao;
    QOpenGLBuffer m_editVertexBuffer;
    QOpenGLBuffer m_editFlagBuffer;
    QVector<float> m_editFlags;      // 与编辑点云一一对应的选中标记（0/1，直接作 attrib）
    QVector<float> m_editPositions;  // float 坐标缓存（x y z 连续），框选投影直接扫
    qsizetype m_selectedCount = 0;
    bool m_editDirty = false;
    bool m_flagsDirty = false;
    bool m_isolateMode = false;     // 隔离显示：仅渲染选中点
    int m_editPointCount = 0;
    bool m_rubberActive = false;
    bool m_rubberSubtract = false;  // 本次框选是否为减选（Shift+拖框）
    QPoint m_rubberStart;
    QRubberBand* m_pRubberBand = nullptr;
    bool m_lassoMode = false;        // 套索模式（与矩形框选互斥）
    bool m_lassoActive = false;      // 正在拖套索
    bool m_lassoSubtract = false;    // 本次套索为减选（Shift+套索）
    QVector<QPoint> m_lassoPath;     // 套索屏幕路径（逻辑像素）
    LassoSelectionOverlay* m_pLassoOverlay = nullptr;

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

WorkpieceMeshViewerDialog::~WorkpieceMeshViewerDialog()
{
    // 应用退出连锁析构时可能仍有 detached 后台线程（建缓存/加载点云/重新生成）。
    // 置销毁标记让线程跳过对 this 的 invokeMethod 并尽快收尾（进度回调返回 false
    // 取消），等计数归零后才继续析构——此前线程对 this 的排队调用仍然安全
    //（析构函数体执行期间 QObject 基类尚未析构）。
    m_destroyed->store(true);
    while (m_workerCount->load() > 0)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

void WorkpieceMeshViewerDialog::BuildUi()
{
    QVBoxLayout* rootLayout = new QVBoxLayout(this);
    rootLayout->setContentsMargins(10, 10, 10, 10);
    rootLayout->setSpacing(8);

    // 第一行：结果目录（独立页面自己选目录，不再依赖补偿界面传入）。
    QHBoxLayout* dirRow = new QHBoxLayout();
    dirRow->setSpacing(8);
    QPushButton* browseBtn = new QPushButton("打开结果…");
    browseBtn->setObjectName("PrimaryButton");
    browseBtn->setToolTip("选择扫描结果目录（Result\\<机器人>\\<结果> 或其 LaserPoint 子目录）。");
    m_pDirEdit = new QLineEdit();
    m_pDirEdit->setReadOnly(true);
    m_pDirEdit->setPlaceholderText("尚未选择扫描结果目录…");
    dirRow->addWidget(browseBtn);
    dirRow->addWidget(m_pDirEdit, 1);
    rootLayout->addLayout(dirRow);

    // 第二行：工具栏，按 视图 / 相机 / 编辑 / 导出 分组，组间竖线分隔。
    QHBoxLayout* toolbar = new QHBoxLayout();
    toolbar->setSpacing(6);
    auto makeSeparator = [this]() -> QFrame*
        {
            QFrame* line = new QFrame(this);
            line->setFrameShape(QFrame::VLine);
            line->setObjectName("ToolbarSeparator");
            line->setFixedWidth(2);
            return line;
        };

    // 视图组：互斥高亮，当前显示模式一目了然。
    m_pSolidBtn = new QPushButton("实体面");
    m_pPointBtn = new QPushButton("点云");
    m_pHeightBtn = new QPushButton("高度图");
    for (QPushButton* button : { m_pSolidBtn, m_pPointBtn, m_pHeightBtn })
    {
        button->setCheckable(true);
        button->setAutoExclusive(true);
    }
    m_pSolidBtn->setChecked(true);

    QPushButton* topBtn = new QPushButton("俯视");
    QPushButton* resetBtn = new QPushButton("复位视角");

    m_pEditBtn = new QPushButton("框选编辑");
    m_pEditBtn->setCheckable(true);
    m_pEditBtn->setToolTip("进入编辑：显示原始点云，左键拖矩形选中点（亮红、可多框累加），"
        "Delete 或'删除所选'执行删除，Esc 清空选择；Ctrl+左键旋转 / 右键平移 / 滚轮缩放。"
        "删完后用'重新生成'重建模型。");
    m_pDeleteSelBtn = new QPushButton("删除所选");
    m_pDeleteSelBtn->setObjectName("DangerButton");
    m_pDeleteSelBtn->setToolTip("删除当前选中（亮红）的点，等同按 Delete 键。");
    m_pLassoBtn = new QPushButton("套索选");
    m_pLassoBtn->setCheckable(true);
    m_pLassoBtn->setToolTip("套索选：开启后左键拖动画自由多边形圈选（适合不规则边界）；关闭则恢复矩形框选。Shift+拖动=从选区减去。");
    m_pInvertBtn = new QPushButton("反选");
    m_pInvertBtn->setToolTip("当前选区取反：框多了先框'该保留的'再反选，或反选后删外部。");
    m_pDeselectBtn = new QPushButton("全不选");
    m_pDeselectBtn->setToolTip("清空当前选区（等同 Esc）。");
    m_pIsolateBtn = new QPushButton("隔离显示");
    m_pIsolateBtn->setCheckable(true);
    m_pIsolateBtn->setToolTip("只显示选中(红)点、隐藏其余，方便精修；再按取消。提示：Shift+拖框=从选区减去。");
    m_pCleanBtn = new QToolButton();
    m_pCleanBtn->setText("清理 ▾");
    m_pCleanBtn->setToolButtonStyle(Qt::ToolButtonTextOnly);
    m_pCleanBtn->setToolTip("自动去噪：SOR 统计离群 / 连通域去飞点——选中后按 Delete 确认删除。");
    m_pCleanBtn->setPopupMode(QToolButton::InstantPopup);
    {
        QMenu* cleanMenu = new QMenu(m_pCleanBtn);
        connect(cleanMenu->addAction("SOR 统计离群…"), &QAction::triggered, this, [this]() { ShowSorPanel(); });
        connect(cleanMenu->addAction("连通域去飞点…"), &QAction::triggered, this, [this]() { ShowClusterPanel(); });
        m_pCleanBtn->setMenu(cleanMenu);
    }
    m_pUndoBtn = new QPushButton("撤销");
    m_pRegenBtn = new QPushButton("重新生成");
    m_pRegenBtn->setObjectName("PrimaryButton");
    m_pRegenBtn->setToolTip("用删除后剩余的点重新生成网格模型，覆写模型缓存，"
        "并把裁剪后的点云保存为 _WorkpieceCloud_Edited.txt（下次打开沿用）。");
    m_pRestoreBtn = new QPushButton("恢复原始");
    m_pRestoreBtn->setObjectName("DangerButton");
    m_pRestoreBtn->setToolTip("丢弃全部框选删除结果（含已保存的裁剪点云），从原始完整点云重新生成模型缓存。");
    QPushButton* exportStlBtn = new QPushButton("导出 STL");
    exportStlBtn->setToolTip("导出二进制 STL 网格（SolidWorks/UG 等 CAD 软件可直接导入）。");

    // 第一行（常驻）：视图模式 + 视角 + 进入编辑 + 导出。其余编辑工具进第二行上下文条。
    toolbar->addWidget(m_pSolidBtn);
    toolbar->addWidget(m_pPointBtn);
    toolbar->addWidget(m_pHeightBtn);
    toolbar->addWidget(makeSeparator());
    toolbar->addWidget(topBtn);
    toolbar->addWidget(resetBtn);
    toolbar->addWidget(makeSeparator());
    toolbar->addWidget(m_pEditBtn);
    toolbar->addStretch(1);
    toolbar->addWidget(exportStlBtn);
    rootLayout->addLayout(toolbar);

    // 第二行（上下文条）：仅「框选编辑」激活时显示（仿 CloudCompare/Blender 模式专属工具条）。
    // 左=选择/清理/撤销（常用、非破坏）；右=破坏性操作（红色、靠右物理隔离，防误触）。
    m_pEditBar = new QWidget(this);
    QHBoxLayout* editBar = new QHBoxLayout(m_pEditBar);
    editBar->setContentsMargins(0, 0, 0, 0);
    editBar->setSpacing(6);
    editBar->addWidget(m_pLassoBtn);
    editBar->addWidget(makeSeparator());
    editBar->addWidget(m_pInvertBtn);
    editBar->addWidget(m_pDeselectBtn);
    editBar->addWidget(m_pIsolateBtn);
    editBar->addWidget(makeSeparator());
    editBar->addWidget(m_pCleanBtn);
    editBar->addWidget(makeSeparator());
    editBar->addWidget(m_pUndoBtn);
    editBar->addStretch(1);
    editBar->addWidget(m_pRegenBtn);
    editBar->addWidget(m_pDeleteSelBtn);   // DangerButton 红
    editBar->addWidget(m_pRestoreBtn);     // DangerButton 红
    m_pEditBar->hide();  // 初始隐藏；进入编辑模式才显示
    rootLayout->addWidget(m_pEditBar);

    // 初始禁用编辑相关按钮（加载模型后启用「框选编辑」，进入编辑模式才启用其余）。
    m_pEditBtn->setEnabled(false);
    m_pDeleteSelBtn->setEnabled(false);
    m_pLassoBtn->setEnabled(false);
    m_pInvertBtn->setEnabled(false);
    m_pDeselectBtn->setEnabled(false);
    m_pIsolateBtn->setEnabled(false);
    m_pCleanBtn->setEnabled(false);
    m_pUndoBtn->setEnabled(false);
    m_pRegenBtn->setEnabled(false);
    m_pRestoreBtn->setEnabled(false);

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

    m_pInfoLabel = new QLabel("点击'打开结果…'选择扫描结果目录（首次自动从完整点云生成模型缓存，之后秒开）。");
    m_pInfoLabel->setWordWrap(true);
    rootLayout->addWidget(m_pInfoLabel);

    connect(browseBtn, &QPushButton::clicked, this, [this]() { BrowseResultDirectory(); });
    connect(m_pSolidBtn, &QPushButton::clicked, this, [this]()
        {
            m_pEditBtn->setChecked(false);
            m_pViewStack->setCurrentIndex(0);
            m_pGlWidget->SetSolidMode(true);
        });
    connect(m_pPointBtn, &QPushButton::clicked, this, [this]()
        {
            m_pEditBtn->setChecked(false);
            m_pViewStack->setCurrentIndex(0);
            m_pGlWidget->SetSolidMode(false);
        });
    connect(m_pHeightBtn, &QPushButton::clicked, this, [this]()
        {
            m_pEditBtn->setChecked(false);
            ShowHeightMap();
        });
    connect(topBtn, &QPushButton::clicked, this, [this]()
        {
            m_pGlWidget->SetTopView();
        });
    connect(resetBtn, &QPushButton::clicked, this, [this]()
        {
            m_pGlWidget->ResetView();
        });
    connect(m_pEditBtn, &QPushButton::toggled, this, [this](bool on)
        {
            if (on)
            {
                EnterEditMode();
            }
            else
            {
                ExitEditMode();
            }
        });
    connect(m_pDeleteSelBtn, &QPushButton::clicked, this, [this]()
        {
            m_pGlWidget->DeleteSelectedEditPoints();
        });
    connect(m_pLassoBtn, &QPushButton::toggled, this, [this](bool on) { m_pGlWidget->SetLassoMode(on); });
    connect(m_pInvertBtn, &QPushButton::clicked, this, [this]() { m_pGlWidget->InvertSelection(); });
    connect(m_pDeselectBtn, &QPushButton::clicked, this, [this]() { m_pGlWidget->ClearSelection(); });
    connect(m_pIsolateBtn, &QPushButton::toggled, this, [this](bool on) { m_pGlWidget->SetIsolateMode(on); });
    connect(m_pUndoBtn, &QPushButton::clicked, this, [this]() { UndoRemove(); });
    connect(m_pRegenBtn, &QPushButton::clicked, this, [this]() { RegenerateFromEditedCloud(); });
    connect(m_pRestoreBtn, &QPushButton::clicked, this, [this]() { RestoreOriginalModel(); });
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
    // 参考通用 3D 查看器工具栏：扁平深色按钮 + 当前模式高亮（checked）+
    // 主操作青色 / 危险操作红色点缀 + 组间细分隔线。
    setStyleSheet(QString(
        "QDialog { background: #101820; color: #E8F1F2; }"
        "QLabel { color: #B8C7CC; }"
        "QLineEdit { background: #0D161D; color: #9ED8DB; border: 1px solid #2E4656;"
        " border-radius: 8px; padding: 6px 10px; }"
        "QPushButton { background: #1B2B36; color: #D9E8EA; border: 1px solid #31505F;"
        " border-radius: 8px; padding: 7px 14px; min-height: 22px; }"
        "QPushButton:hover { background: #24465A; border-color: #63C7D1; }"
        "QPushButton:pressed { background: #152530; }"
        "QPushButton:checked { background: #2C6173; border-color: #79D6DE; color: #FFFFFF; font-weight: 600; }"
        "QPushButton:disabled { background: #141F27; color: #546770; border-color: #243640; }"
        "QPushButton#PrimaryButton { border-color: #3F7E8C; color: #A8E8EE; }"
        "QPushButton#PrimaryButton:hover { background: #1F5160; border-color: #79D6DE; }"
        "QPushButton#DangerButton { border-color: #6B3642; color: #F0B9C2; }"
        "QPushButton#DangerButton:hover { background: #54222E; border-color: #E07A8A; }"
        "QPushButton#DangerButton:disabled { color: #546770; border-color: #243640; }"
        "QFrame#ToolbarSeparator { background: #26404E; border: none; margin: 4px 2px; }"
        "QScrollArea { border: 1px solid #2E4656; border-radius: 8px; background: #0B1117; }"));
}

void WorkpieceMeshViewerDialog::BrowseResultDirectory()
{
    if (m_bBuilding || m_bEditBusy)
    {
        m_pInfoLabel->setText("后台任务进行中，请稍候再切换目录。");
        return;
    }
    const QString startDir = m_laserDir.isEmpty()
        ? QDir::current().filePath("Result")
        : QFileInfo(m_laserDir).path();
    const QString chosen = QFileDialog::getExistingDirectory(this, "选择扫描结果目录", startDir);
    if (chosen.isEmpty())
    {
        return;
    }
    // 兼容直接选结果目录（自动进 LaserPoint 子目录）或选到 LaserPoint 本身。
    QString laserDir = chosen;
    QDir dir(chosen);
    if (dir.dirName().compare(QStringLiteral("LaserPoint"), Qt::CaseInsensitive) != 0
        && dir.exists(QStringLiteral("LaserPoint")))
    {
        laserDir = dir.filePath(QStringLiteral("LaserPoint"));
    }
    const bool hasCloud = QFileInfo::exists(QDir(laserDir).filePath("PreciseLaserPoint_WorkpieceCloud.txt"))
        || QFileInfo::exists(WorkpieceMeshBuilder::EditedCloudPath(laserDir));
    const bool hasCache = WorkpieceMeshBuilder::IsMeshCacheValid(WorkpieceMeshBuilder::MeshCachePath(laserDir));
    if (!hasCloud && !hasCache)
    {
        QMessageBox::information(this, "工件模型",
            "所选目录没有完整点云文件或模型缓存。\n请选择 Result\\<机器人>\\<结果> 目录（或其 LaserPoint 子目录）。");
        return;
    }
    StartLoad(laserDir);
}

void WorkpieceMeshViewerDialog::StartLoad(const QString& laserDir)
{
    if (m_bBuilding || m_bEditBusy)
    {
        // 正在后台生成/编辑作业，不能换上下文——明确提示而不是静默吞掉，
        // 避免用户把窗口里上一个目录的模型当成新选目录的结果。
        m_pInfoLabel->setText("后台任务进行中，当前显示内容未更新，请稍候重新打开。");
        return;
    }
    // 无论同/异目录都先退出编辑模式：复用实例重开时若编辑态残留，界面显示
    // 网格统计提示而交互仍是框选删除，用户按提示"左键旋转"会误删点。
    m_pEditBtn->setChecked(false);
    if (laserDir != m_laserDir)
    {
        // 换了结果目录：上一工件的编辑态全部作废。
        m_editCloud.clear();
        m_undoStack.clear();
        m_pEditBtn->setEnabled(false);
        m_pRestoreBtn->setEnabled(false);
    }
    m_laserDir = laserDir;
    m_heightMapCache = QImage();
    setWindowTitle(QString("工件模型 - %1").arg(QFileInfo(laserDir).dir().dirName()));
    if (m_pDirEdit != nullptr)
    {
        m_pDirEdit->setText(QDir::toNativeSeparators(laserDir));
    }

    const QString cachePath = WorkpieceMeshBuilder::MeshCachePath(laserDir);
    if (WorkpieceMeshBuilder::IsMeshCacheValid(cachePath))
    {
        FinishLoadFromCache();  // 二进制缓存亚秒级，同步加载
        return;
    }

    // 优先用框选裁剪后的点云（上次"重新生成"保存），没有才用原始完整点云。
    const QString cloudPath = WorkpieceMeshBuilder::ResolveCloudSourcePath(laserDir);
    if (!QFileInfo::exists(cloudPath))
    {
        m_pInfoLabel->setText(QString("缺少完整点云文件，无法生成工件模型：%1")
            .arg(QDir::toNativeSeparators(cloudPath)));
        return;
    }

    // 首次/旧版本缓存：后台线程生成（解析 150MB 级文本最耗时），进度条 + 可取消，不阻塞界面。
    m_bBuilding = true;
    m_pInfoLabel->setText("正在后台生成工件模型缓存…");
    EnsureProgressDialog();
    m_pProgress->setCancelButtonText("取消");  // regen 路径会移除取消按钮，这里恢复
    m_pProgress->setLabelText("正在生成工件模型（首次需要解析完整点云）…");
    m_pProgress->setValue(0);
    m_pProgress->show();

    auto cancelFlag = std::make_shared<std::atomic_bool>(false);
    // 精确断开上一次的取消连接：通配 disconnect 会把 QProgressDialog 内部连接一并
    // 误杀；UniqueConnection 不支持 lambda 槽（Debug 下断言）。
    if (m_progressCancelConn)
    {
        disconnect(m_progressCancelConn);
    }
    m_progressCancelConn = connect(m_pProgress, &QProgressDialog::canceled, this,
        [cancelFlag]() { cancelFlag->store(true); });

    auto destroyed = m_destroyed;
    auto workerCount = m_workerCount;
    workerCount->fetch_add(1);
    std::thread([this, laserDir, cloudPath, cancelFlag, destroyed, workerCount]()
        {
            // 进度回调在后台线程触发，更新 UI 一律排队投递；对话框销毁由 destroyed
            // 令牌拦截（析构等待计数归零，等待期间的排队调用安全）。
            const WorkpieceMeshBuilder::ProgressCallback progressCb =
                [this, cancelFlag, destroyed](int percent, const QString& stage) -> bool
                {
                    if (cancelFlag->load() || destroyed->load())
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
            if (!destroyed->load())
            {
                QMetaObject::invokeMethod(this, [this, ok, buildError]()
                    {
                        OnBuildFinished(ok, buildError);
                    }, Qt::QueuedConnection);
            }
            workerCount->fetch_sub(1);
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
    m_meshInfoText = QString("模型缓存：%1（顶点 %2 / 三角形 %3）— 左键旋转 / 右键平移 / 滚轮缩放 / 双击复位")
        .arg(QDir::toNativeSeparators(cachePath))
        .arg(m_mesh.vertices.size())
        .arg(m_mesh.indices.size() / 3);
    m_pInfoLabel->setText(m_meshInfoText);
    m_pEditBtn->setEnabled(true);
    m_pRestoreBtn->setEnabled(true);
}

void WorkpieceMeshViewerDialog::EnsureProgressDialog()
{
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
}

void WorkpieceMeshViewerDialog::EnterEditMode()
{
    if (m_bBuilding || m_bEditBusy)
    {
        m_pEditBtn->setChecked(false);
        return;
    }
    if (!m_editCloud.isEmpty())
    {
        ShowEditCloud();  // 再次进入：沿用已加载点云与撤销栈
        return;
    }
    // 优先加载裁剪后的点云（上次"重新生成"保存），编辑可跨会话延续。
    const QString cloudPath = WorkpieceMeshBuilder::ResolveCloudSourcePath(m_laserDir);
    if (!QFileInfo::exists(cloudPath))
    {
        QMessageBox::information(this, "框选编辑",
            QString("缺少完整点云文件，无法编辑：%1").arg(QDir::toNativeSeparators(cloudPath)));
        m_pEditBtn->setChecked(false);
        return;
    }

    // 首次进入：后台加载完整点云（150MB 级文本），进度条 + 可取消。
    m_bEditBusy = true;
    EnsureProgressDialog();
    m_pProgress->setCancelButtonText("取消");
    m_pProgress->setRange(0, 100);
    m_pProgress->setValue(0);
    m_pProgress->setLabelText("正在加载完整点云用于编辑…");
    m_pProgress->show();

    auto cancelFlag = std::make_shared<std::atomic_bool>(false);
    if (m_progressCancelConn)
    {
        disconnect(m_progressCancelConn);
    }
    m_progressCancelConn = connect(m_pProgress, &QProgressDialog::canceled, this,
        [cancelFlag]() { cancelFlag->store(true); });

    auto destroyed = m_destroyed;
    auto workerCount = m_workerCount;
    workerCount->fetch_add(1);
    std::thread([this, cloudPath, cancelFlag, destroyed, workerCount]()
        {
            const WorkpieceMeshBuilder::ProgressCallback progressCb =
                [this, cancelFlag, destroyed](int percent, const QString& stage) -> bool
                {
                    if (cancelFlag->load() || destroyed->load())
                    {
                        return false;
                    }
                    QMetaObject::invokeMethod(this, [this, percent, stage]()
                        {
                            if (m_pProgress != nullptr)
                            {
                                m_pProgress->setValue(percent);
                                m_pProgress->setLabelText(QString("正在加载完整点云：%1…").arg(stage));
                            }
                        }, Qt::QueuedConnection);
                    return true;
                };
            QVector<RobotCalculation::IndexedPoint3D> points;
            QString loadError;
            const bool ok = WorkpieceMeshBuilder::LoadCloudPointsWithProgress(
                cloudPath, points, loadError, progressCb);
            if (!destroyed->load())
            {
                QMetaObject::invokeMethod(this, [this, ok, loadError, points]()
                    {
                        m_bEditBusy = false;
                        if (m_pProgress != nullptr)
                        {
                            m_pProgress->hide();
                        }
                        if (!ok)
                        {
                            if (loadError != QStringLiteral("已取消"))
                            {
                                QMessageBox::warning(this, "框选删除", loadError);
                            }
                            m_pEditBtn->setChecked(false);
                            return;
                        }
                        m_editCloud = points;
                        if (m_pEditBtn->isChecked())  // 加载期间用户可能已手动退出
                        {
                            ShowEditCloud();
                        }
                    }, Qt::QueuedConnection);
            }
            workerCount->fetch_sub(1);
        }).detach();
}

void WorkpieceMeshViewerDialog::ShowEditCloud()
{
    m_pViewStack->setCurrentIndex(0);
    m_pGlWidget->SetEditCloud(&m_editCloud,
        [this](const QVector<RobotCalculation::IndexedPoint3D>& removed)
        {
            OnEditPointsRemoved(removed);
        },
        [this](qsizetype) { UpdateEditInfo(); });
    m_pGlWidget->setFocus();  // Delete/Esc 快捷键直达
    m_pEditBar->show();   // 显示编辑上下文条
    m_pDeleteSelBtn->setEnabled(true);
    m_pLassoBtn->setEnabled(true);
    m_pInvertBtn->setEnabled(true);
    m_pDeselectBtn->setEnabled(true);
    m_pIsolateBtn->setEnabled(true);
    m_pCleanBtn->setEnabled(true);
    m_pUndoBtn->setEnabled(!m_undoStack.isEmpty());
    m_pRegenBtn->setEnabled(true);
    UpdateEditInfo();
}

void WorkpieceMeshViewerDialog::ExitEditMode()
{
    // 保留 m_editCloud 与撤销栈——再次进入编辑可继续；切回网格显示。
    m_pGlWidget->SetEditCloud(nullptr);
    m_pEditBar->hide();   // 隐藏编辑上下文条
    m_pIsolateBtn->setChecked(false);   // 退出编辑重置隔离显示
    m_pGlWidget->SetIsolateMode(false);
    m_pLassoBtn->setChecked(false);     // 退出编辑重置套索模式
    m_pDeleteSelBtn->setEnabled(false);
    m_pLassoBtn->setEnabled(false);
    m_pInvertBtn->setEnabled(false);
    m_pDeselectBtn->setEnabled(false);
    m_pIsolateBtn->setEnabled(false);
    m_pCleanBtn->setEnabled(false);
    m_pUndoBtn->setEnabled(false);
    m_pRegenBtn->setEnabled(false);
    if (!m_meshInfoText.isEmpty())
    {
        m_pInfoLabel->setText(m_meshInfoText);
    }
}

void WorkpieceMeshViewerDialog::OnEditPointsRemoved(
    const QVector<RobotCalculation::IndexedPoint3D>& removed)
{
    m_undoStack.append(removed);
    while (m_undoStack.size() > 20)  // 撤销栈封顶（更早的删除可用"恢复原始"找回）
    {
        m_undoStack.removeFirst();
    }
    m_pUndoBtn->setEnabled(true);
    UpdateEditInfo();
}

void WorkpieceMeshViewerDialog::UndoRemove()
{
    if (m_undoStack.isEmpty())
    {
        return;
    }
    m_editCloud += m_undoStack.takeLast();
    m_pGlWidget->RefreshEditCloud();
    m_pUndoBtn->setEnabled(!m_undoStack.isEmpty());
    UpdateEditInfo();
}

void WorkpieceMeshViewerDialog::UpdateEditInfo()
{
    m_pInfoLabel->setText(QString(
        "框选编辑：左键拖矩形选中（已选 %1 / 剩余 %2 点，可撤销 %3 步）→ Delete 删除，Esc 清选；"
        "Ctrl+左键旋转 / 右键平移 / 滚轮缩放。删完后点'重新生成'重建模型。")
        .arg(m_pGlWidget->SelectedCount())
        .arg(m_editCloud.size())
        .arg(m_undoStack.size()));
}

void WorkpieceMeshViewerDialog::RegenerateFromEditedCloud()
{
    if (m_bBuilding || m_bEditBusy)
    {
        return;
    }
    if (m_editCloud.size() < 16)
    {
        QMessageBox::warning(this, "重新生成", "剩余点数过少，无法生成网格。可先'撤销删除'或'恢复原始'。");
        return;
    }
    // 先退出编辑模式再拍快照：否则进度框被关闭按钮提前隐藏时用户还能继续框选，
    // 新删除进不了正在生成的网格，编辑点云与写盘缓存静默分叉。
    m_pEditBtn->setChecked(false);
    m_bEditBusy = true;
    EnsureProgressDialog();
    if (m_progressCancelConn)
    {
        disconnect(m_progressCancelConn);
        m_progressCancelConn = QMetaObject::Connection();
    }
    m_pProgress->setCancelButton(nullptr);  // 网格化不可取消，移除取消按钮防误解
    m_pProgress->setRange(0, 0);  // 网格化无细分进度，用不确定模式
    m_pProgress->setLabelText("正在用剩余点重新生成模型并更新缓存…");
    m_pProgress->show();

    const QString cachePath = WorkpieceMeshBuilder::MeshCachePath(m_laserDir);
    const QString editedCloudPath = WorkpieceMeshBuilder::EditedCloudPath(m_laserDir);
    auto destroyed = m_destroyed;
    auto workerCount = m_workerCount;
    workerCount->fetch_add(1);
    std::thread([this, cachePath, editedCloudPath, cloud = m_editCloud, destroyed, workerCount]()
        {
            WorkpieceMeshBuilder::Mesh mesh;
            QString buildError;
            bool ok = WorkpieceMeshBuilder::BuildFromScanlineCloud(cloud, mesh, buildError);
            if (ok)
            {
                ok = WorkpieceMeshBuilder::SaveMeshPly(cachePath, mesh, buildError);
            }
            QString cloudSaveError;
            bool cloudSaved = false;
            if (ok)
            {
                // 裁剪结果同时持久化为 txt：下次打开/再次编辑直接沿用，不用重新框选。
                cloudSaved = WorkpieceMeshBuilder::SaveCloudPointsTxt(editedCloudPath, cloud, cloudSaveError);
            }
            if (!destroyed->load())
            {
                QMetaObject::invokeMethod(this, [this, ok, buildError, mesh, cloudSaved, cloudSaveError]()
                    {
                        m_bEditBusy = false;
                        if (m_pProgress != nullptr)
                        {
                            m_pProgress->hide();
                            m_pProgress->setRange(0, 100);
                        }
                        if (!ok)
                        {
                            QMessageBox::warning(this, "重新生成", buildError);
                            return;
                        }
                        m_mesh = mesh;
                        m_heightMapCache = QImage();
                        m_pGlWidget->SetMesh(m_mesh, true);  // 保持当前视角，方便直接检查框选效果
                        m_meshInfoText = QString(
                            "已用剩余点重新生成模型并更新缓存（顶点 %1 / 三角形 %2）%3。"
                            "如仍有噪声可再次'框选编辑'，或'恢复原始'重来。")
                            .arg(m_mesh.vertices.size())
                            .arg(m_mesh.indices.size() / 3)
                            .arg(cloudSaved ? QStringLiteral("，裁剪点云已保存")
                                            : QStringLiteral("，但裁剪点云保存失败：") + cloudSaveError);
                        m_pInfoLabel->setText(m_meshInfoText);
                    }, Qt::QueuedConnection);
            }
            workerCount->fetch_sub(1);
        }).detach();
}

void WorkpieceMeshViewerDialog::RestoreOriginalModel()
{
    if (m_bBuilding || m_bEditBusy)
    {
        return;
    }
    // 先确认源点云还在，再删模型缓存——否则源 txt 已被清理时缓存删了无法重建，
    // 唯一产物永久丢失。
    const QString cloudPath = QDir(m_laserDir).filePath("PreciseLaserPoint_WorkpieceCloud.txt");
    if (!QFileInfo::exists(cloudPath))
    {
        QMessageBox::warning(this, "恢复原始",
            QString("完整点云文件已不存在，无法重新生成，已保留当前模型缓存：%1")
                .arg(QDir::toNativeSeparators(cloudPath)));
        return;
    }
    if (QMessageBox::question(this, "恢复原始",
            "将丢弃全部框选删除结果，从原始完整点云重新生成模型缓存。继续？")
        != QMessageBox::Yes)
    {
        return;
    }
    m_pEditBtn->setChecked(false);
    m_editCloud.clear();
    m_undoStack.clear();
    m_pUndoBtn->setEnabled(false);
    QFile::remove(WorkpieceMeshBuilder::EditedCloudPath(m_laserDir));  // 裁剪点云一并作废
    QFile::remove(WorkpieceMeshBuilder::MeshCachePath(m_laserDir));
    StartLoad(m_laserDir);  // 缓存已删 → 走后台从原始点云重建
}

// SOR 统计离群：非模态小面板，调参→预览标红→确认删除。与「删除」解耦——
// 预览只改红旗选区，真正销毁点仍走 DeleteSelectedEditPoints（可撤销）。
void WorkpieceMeshViewerDialog::ShowSorPanel()
{
    if (m_pGlWidget == nullptr) return;
    QDialog* dlg = new QDialog(this);
    dlg->setWindowTitle("SOR 统计离群去噪");
    dlg->setAttribute(Qt::WA_DeleteOnClose);
    dlg->setModal(false);

    QVBoxLayout* root = new QVBoxLayout(dlg);
    QLabel* hint = new QLabel(
        "按每点到 k 个最近邻的平均距离统计：偏离全局均值超过设定倍数的点判为离群。\n"
        "先「预览」标红，确认无误后「删除选中」（删除可撤销）。", dlg);
    hint->setWordWrap(true);
    root->addWidget(hint);

    QFormLayout* form = new QFormLayout();
    QSpinBox* kSpin = new QSpinBox(dlg);
    kSpin->setRange(1, 200);
    kSpin->setValue(6);
    QDoubleSpinBox* sdSpin = new QDoubleSpinBox(dlg);
    sdSpin->setRange(0.1, 10.0);
    sdSpin->setSingleStep(0.1);
    sdSpin->setDecimals(2);
    sdSpin->setValue(1.0);
    form->addRow("邻居数 k", kSpin);
    form->addRow("标准差倍数（越小删越多）", sdSpin);
    root->addLayout(form);

    QHBoxLayout* btns = new QHBoxLayout();
    QPushButton* previewBtn = new QPushButton("预览选中", dlg);
    QPushButton* clearBtn = new QPushButton("清除选区", dlg);
    QPushButton* applyBtn = new QPushButton("删除选中", dlg);
    applyBtn->setObjectName("DangerButton");
    btns->addWidget(previewBtn);
    btns->addWidget(clearBtn);
    btns->addStretch(1);
    btns->addWidget(applyBtn);
    root->addLayout(btns);

    connect(previewBtn, &QPushButton::clicked, this, [this, kSpin, sdSpin]()
        {
            m_pGlWidget->ClearSelection();
            const qsizetype n = m_pGlWidget->SelectStatisticalOutliers(kSpin->value(), sdSpin->value());
            m_pInfoLabel->setText(QString("SOR 预览：标记离群点 %1 个（红）。满意→删除选中；否则调参重试。").arg(n));
        });
    connect(clearBtn, &QPushButton::clicked, this, [this]() { m_pGlWidget->ClearSelection(); });
    connect(applyBtn, &QPushButton::clicked, this, [this, dlg]()
        {
            m_pGlWidget->DeleteSelectedEditPoints();
            dlg->close();
        });

    dlg->show();
}

// 连通域去飞点：体素栅格 26 邻接聚类，点数少于阈值的孤立小簇判为悬空飞点。
void WorkpieceMeshViewerDialog::ShowClusterPanel()
{
    if (m_pGlWidget == nullptr) return;
    QDialog* dlg = new QDialog(this);
    dlg->setWindowTitle("连通域去飞点");
    dlg->setAttribute(Qt::WA_DeleteOnClose);
    dlg->setModal(false);

    QVBoxLayout* root = new QVBoxLayout(dlg);
    QLabel* hint = new QLabel(
        "按体素栅格做 26 邻接连通域聚类，把点数少于阈值的孤立小簇（悬空飞点）标红。\n"
        "先「预览」标红，确认无误后「删除选中」（删除可撤销）。", dlg);
    hint->setWordWrap(true);
    root->addWidget(hint);

    QFormLayout* form = new QFormLayout();
    QDoubleSpinBox* voxSpin = new QDoubleSpinBox(dlg);
    voxSpin->setRange(0.1, 50.0);
    voxSpin->setSingleStep(0.5);
    voxSpin->setDecimals(2);
    voxSpin->setValue(3.0);
    voxSpin->setSuffix(" mm");
    QSpinBox* minSpin = new QSpinBox(dlg);
    minSpin->setRange(1, 100000);
    minSpin->setValue(30);
    form->addRow("体素尺寸", voxSpin);
    form->addRow("最小簇点数（小于此判为飞点）", minSpin);
    root->addLayout(form);

    QHBoxLayout* btns = new QHBoxLayout();
    QPushButton* previewBtn = new QPushButton("预览选中", dlg);
    QPushButton* clearBtn = new QPushButton("清除选区", dlg);
    QPushButton* applyBtn = new QPushButton("删除选中", dlg);
    applyBtn->setObjectName("DangerButton");
    btns->addWidget(previewBtn);
    btns->addWidget(clearBtn);
    btns->addStretch(1);
    btns->addWidget(applyBtn);
    root->addLayout(btns);

    connect(previewBtn, &QPushButton::clicked, this, [this, voxSpin, minSpin]()
        {
            m_pGlWidget->ClearSelection();
            const qsizetype n = m_pGlWidget->SelectSmallClusters(voxSpin->value(), minSpin->value());
            m_pInfoLabel->setText(QString("连通域预览：标记飞点 %1 个（红）。满意→删除选中；否则调参重试。").arg(n));
        });
    connect(clearBtn, &QPushButton::clicked, this, [this]() { m_pGlWidget->ClearSelection(); });
    connect(applyBtn, &QPushButton::clicked, this, [this, dlg]()
        {
            m_pGlWidget->DeleteSelectedEditPoints();
            dlg->close();
        });

    dlg->show();
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
    else if (m_pSolidBtn != nullptr)
    {
        m_pSolidBtn->setChecked(true);  // 没有模型可渲染，视图高亮回退实体面
    }
}
