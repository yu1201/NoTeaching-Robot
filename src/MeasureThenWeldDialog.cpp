#include "MeasureThenWeldDialog.h"

#include "CameraFrameCache.h"
#include "ConfigDatabase.h"
#include "FANUCRobotDriver.h"
#include "STEPRobotDriver.h"
#include "HandEyeMatrixConfig.h"
#include "MeasureThenWeldService.h"
#include "MeasureThenWeldRuntimeConfig.h"
#include "OPini.h"
#include "RobotDriverAdaptor.h"
#include "RobotDataHelper.h"
#include "RobotLog.h"
#include "RobotMessage.h"
#include "RobotOperationLease.h"
#include "WeldProcessFile.h"
#include "WeldResumePlanner.h"
#include "WindowStyleHelper.h"
#include "groove/framebuffer.h"

#include <QCloseEvent>
#include <QCheckBox>
#include <QComboBox>
#include <QCoreApplication>
#include <QDateTime>
#include <QDoubleSpinBox>
#include <QDir>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLayout>
#include <QMessageBox>
#include <QMutexLocker>
#include <QPlainTextEdit>
#include <QPointer>
#include <QProgressBar>
#include <QPainter>
#include <QPushButton>
#include <QRegularExpression>
#include <QScrollArea>
#include <QSizePolicy>
#include <QSignalBlocker>
#include <QStringList>
#include <QTextDocument>
#include <QTextStream>
#include <QThread>
#include <QTimer>
#include <QUuid>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>
#include <mutex>
#include <thread>

namespace
{
constexpr auto RAW_LASER_FILE_NAME = "PreciseLaserPoint.txt";
constexpr auto WORKPIECE_CLOUD_FILE_NAME = "PreciseLaserPoint_WorkpieceCloud.txt";
constexpr auto PRESERVE_PATH_FILE_NAME = "PreciseLaserPoint_PreservePath_2mm.txt";
constexpr auto WELD_POSE_FILE_NAME = "PreciseLaserPoint_WeldPose_2mm.txt";
constexpr auto WELD_POSE_SEAM_COMP_FILE_NAME = "PreciseLaserPoint_WeldPose_2mm_SeamComp.txt";
constexpr int COMP_SEGMENT_COUNT = 4;
constexpr char POSE_GROUP_COUNT_KEY[] = "PoseCompGroupCount";
constexpr char POSE_ACTIVE_GROUP_INDEX_KEY[] = "ActivePoseCompGroupIndex";
constexpr char SEAM_GROUP_COUNT_KEY[] = "SeamCompGroupCount";
constexpr char SEAM_ACTIVE_GROUP_INDEX_KEY[] = "ActiveSeamCompGroupIndex";
constexpr char WELD_BREAKPOINT_SECTION[] = "Breakpoint";
constexpr char WELD_BREAKPOINT_RECORD_V2_KEY[] = "RecordV2";
std::recursive_mutex g_weldBreakpointRecordMutex;

std::string ToUtf8StdString(const QString& text)
{
    const QByteArray bytes = text.toUtf8();
    return std::string(bytes.constData(), static_cast<size_t>(bytes.size()));
}

double SafeSpeed(double value, double fallback)
{
    return value > 0.0 ? value : fallback;
}

void ShowNonModalFlowResult(
    QWidget* parent,
    QMessageBox::Icon icon,
    const QString& title,
    const QString& text)
{
    QMessageBox* message = new QMessageBox(icon, title, text, QMessageBox::Ok, parent);
    message->setAttribute(Qt::WA_DeleteOnClose);
    message->setWindowModality(Qt::NonModal);
    message->show();
}

QString WeldBreakpointStoragePath(const QString& robotName)
{
    return RobotDataHelper::BuildProjectPath(
        QString("Data/%1/WeldBreakpoint.ini").arg(robotName));
}

bool WriteBreakpointRecordValue(
    const QString& robotName,
    const QString& encoded,
    QString* error)
{
    const std::lock_guard<std::recursive_mutex> lock(g_weldBreakpointRecordMutex);
    COPini ini;
    const QString path = WeldBreakpointStoragePath(robotName);
    if (!ini.SetFileName(ToUtf8StdString(path))
        || !ini.SetSectionName(WELD_BREAKPOINT_SECTION)
        || !ini.WriteString(WELD_BREAKPOINT_RECORD_V2_KEY, ToUtf8StdString(encoded)))
    {
        if (error != nullptr)
        {
            *error = QString("写入断点V2记录失败：%1").arg(path);
        }
        return false;
    }
    return true;
}

bool DisableLegacyBreakpoint(const QString& robotName, QString* error)
{
    const std::lock_guard<std::recursive_mutex> lock(g_weldBreakpointRecordMutex);
    COPini ini;
    const QString path = WeldBreakpointStoragePath(robotName);
    if (!ini.SetFileName(ToUtf8StdString(path))
        || !ini.SetSectionName(WELD_BREAKPOINT_SECTION)
        || !ini.WriteString("Valid", 0))
    {
        if (error != nullptr)
        {
            *error = QString("关闭旧版断点标志失败：%1").arg(path);
        }
        return false;
    }
    return true;
}

bool ReadBreakpointRecord(
    const QString& robotName,
    WeldResumePlanner::CheckpointRecord& record,
    QString* encoded,
    QString* error)
{
    const std::lock_guard<std::recursive_mutex> lock(g_weldBreakpointRecordMutex);
    COPini ini;
    const QString path = WeldBreakpointStoragePath(robotName);
    std::string stored;
    if (!ini.SetFileName(ToUtf8StdString(path))
        || !ini.SetSectionName(WELD_BREAKPOINT_SECTION)
        || ini.ReadString(false, WELD_BREAKPOINT_RECORD_V2_KEY, stored) <= 0
        || stored.empty())
    {
        if (error != nullptr)
        {
            *error = QStringLiteral(
                "没有可验证的V2断点。旧版仅含位姿的断点禁止自动选择最新案例，请在新版本中重新暂停生成断点。");
        }
        return false;
    }
    const QString value = QString::fromUtf8(stored.data(), static_cast<int>(stored.size()));
    if (value.startsWith(QStringLiteral("invalidated:v2:")))
    {
        if (error != nullptr)
        {
            *error = QStringLiteral("旧断点已被随后启动的完整焊接失效，禁止继续使用。");
        }
        return false;
    }
    if (!WeldResumePlanner::DecodeRecord(value, record, error))
    {
        return false;
    }
    if (encoded != nullptr)
    {
        *encoded = value;
    }
    return true;
}

bool SavePausedBreakpointRecord(
    const WeldResumePlanner::CheckpointRecord& source,
    QString* error)
{
    const std::lock_guard<std::recursive_mutex> lock(g_weldBreakpointRecordMutex);
    WeldResumePlanner::CheckpointRecord writing = source;
    writing.state = QStringLiteral("writing");
    QString encodeError;
    const QString writingValue = WeldResumePlanner::EncodeRecord(writing, &encodeError);
    if (writingValue.isEmpty()
        || !WriteBreakpointRecordValue(source.robotName, writingValue, error))
    {
        if (error != nullptr && error->isEmpty())
        {
            *error = encodeError;
        }
        return false;
    }
    if (!DisableLegacyBreakpoint(source.robotName, error))
    {
        return false;
    }

    WeldResumePlanner::CheckpointRecord paused = source;
    paused.state = QStringLiteral("paused");
    const QString pausedValue = WeldResumePlanner::EncodeRecord(paused, &encodeError);
    if (pausedValue.isEmpty()
        || !WriteBreakpointRecordValue(source.robotName, pausedValue, error))
    {
        if (error != nullptr && error->isEmpty())
        {
            *error = encodeError;
        }
        return false;
    }
    return true;
}

bool TransitionBreakpointRecordState(
    const QString& robotName,
    const QString& expectedCheckpointId,
    const QString& expectedState,
    const QString& newState,
    QString* error)
{
    const std::lock_guard<std::recursive_mutex> lock(g_weldBreakpointRecordMutex);
    WeldResumePlanner::CheckpointRecord record;
    if (!ReadBreakpointRecord(robotName, record, nullptr, error))
    {
        return false;
    }
    if (record.checkpointId != expectedCheckpointId || record.state != expectedState)
    {
        if (error != nullptr)
        {
            *error = QString("断点状态已变化，拒绝覆盖。Expected=%1/%2 Current=%3/%4")
                .arg(expectedCheckpointId, expectedState, record.checkpointId, record.state);
        }
        return false;
    }
    record.state = newState;
    QString encodeError;
    const QString value = WeldResumePlanner::EncodeRecord(record, &encodeError);
    if (value.isEmpty() || !WriteBreakpointRecordValue(robotName, value, error))
    {
        if (error != nullptr && error->isEmpty())
        {
            *error = encodeError;
        }
        return false;
    }
    return DisableLegacyBreakpoint(robotName, error);
}

QString RobotDriverTypeName(RobotDriverAdaptor* driver)
{
    if (dynamic_cast<STEPRobotCtrl*>(driver) != nullptr)
    {
        return QStringLiteral("STEP");
    }
    if (dynamic_cast<FANUCRobotCtrl*>(driver) != nullptr)
    {
        return QStringLiteral("FANUC");
    }
    return QStringLiteral("UNKNOWN");
}

// 扫描实时激光线视图：把当前帧 XData/YData 画成散点（自动缩放），与坡口相机预览同源数据、轻量版绘制。
class LaserLineLiveView final : public QWidget
{
public:
    explicit LaserLineLiveView(QWidget* parent = nullptr)
        : QWidget(parent)
    {
        setMinimumHeight(170);
    }

    void SetFrame(const udpDataShow& frame)
    {
        m_x = frame.XData;
        m_y = frame.YData;
        m_hasFrame = m_x.size() >= 2 && m_x.size() == m_y.size();
        if (m_hasFrame)
        {
            double minY = m_y[0], maxY = m_y[0];
            for (int i = 1; i < m_y.size(); ++i)
            {
                minY = std::min(minY, m_y[i]); maxY = std::max(maxY, m_y[i]);
            }
            const double dataCenterY = (minY + maxY) * 0.5;
            // Y 中心只锁定一次（固定标尺）：之后深度变化表现为线上下移动，画面不重新定标。
            // 仅当数据即将跑出视野(偏离超半窗80%)才重锁一次。
            const double halfSpanY = m_halfSpanXmm * (height() > 0 && width() > 0 ? double(height()) / width() : 1.0);
            if (!m_hasLockedCenterY || std::abs(dataCenterY - m_lockedCenterY) > halfSpanY * 0.8)
            {
                m_lockedCenterY = dataCenterY;
                m_hasLockedCenterY = true;
            }
        }
        update();
    }

    void ClearFrame()
    {
        m_hasFrame = false;
        m_hasLockedCenterY = false;  // 下次扫描重新锁定标尺中心
        update();
    }

    void SetPointSize(double pointSize)
    {
        m_pointSize = std::clamp(pointSize, 0.5, 6.0);
        update();
    }

    void SetViewHalfSpan(double halfSpanMm)
    {
        m_halfSpanXmm = std::clamp(halfSpanMm, 20.0, 500.0);
        m_hasLockedCenterY = false;  // 视野变了重新锁定中心
        update();
    }

protected:
    void paintEvent(QPaintEvent*) override
    {
        QPainter painter(this);
        painter.fillRect(rect(), QColor(0x05, 0x08, 0x0B));  // 黑底（与坡口相机预览一致）
        painter.setPen(QColor(0x2B, 0x45, 0x52));
        painter.drawRect(rect().adjusted(0, 0, -1, -1));
        if (!m_hasFrame)
        {
            painter.setPen(QColor(0x6E, 0x88, 0x94));
            painter.drawText(rect(), Qt::AlignCenter, QStringLiteral("激光线点云：等待相机帧..."));
            return;
        }
        // 固定标尺（等比）：X 窗口恒定 ±kHalfSpanXmm、中心恒 0；Y 同一比例尺、中心为锁定值。
        // 标尺不随帧数据变化——线的移动/形状变化即真实几何变化。
        const QRectF area = rect().adjusted(10, 10, -10, -10);
        const double scale = area.width() / (2.0 * m_halfSpanXmm);
        const double centerX = 0.0;
        const double centerY = m_hasLockedCenterY ? m_lockedCenterY : 0.0;

        // 网格 + 物理坐标标注（与预览同风格）：按像素等分，标签为对应的物理 mm 值。
        QPen gridPen(QColor(0x24, 0x38, 0x42));
        gridPen.setStyle(Qt::DashLine);
        painter.setPen(gridPen);
        constexpr int kGridCount = 4;
        QFont labelFont = painter.font();
        labelFont.setPointSize(9);
        painter.setFont(labelFont);
        for (int index = 1; index < kGridCount; ++index)
        {
            const double gx = area.left() + area.width() * index / kGridCount;
            const double gy = area.top() + area.height() * index / kGridCount;
            painter.drawLine(QPointF(gx, area.top()), QPointF(gx, area.bottom()));
            painter.drawLine(QPointF(area.left(), gy), QPointF(area.right(), gy));
        }
        painter.setPen(QColor(0x5E, 0x78, 0x84));
        for (int index = 1; index < kGridCount; ++index)
        {
            const double gx = area.left() + area.width() * index / kGridCount;
            const double gy = area.top() + area.height() * index / kGridCount;
            const double dataX = centerX + (gx - area.center().x()) / scale;
            const double dataY = centerY - (gy - area.center().y()) / scale;
            painter.drawText(QPointF(gx + 3.0, area.bottom() - 4.0), QString::number(dataX, 'f', 0));
            painter.drawText(QPointF(area.left() + 4.0, gy - 3.0), QString::number(dataY, 'f', 0));
        }

        painter.setPen(QPen(QColor(0x72, 0xD4, 0xDD), m_pointSize));
        for (int i = 0; i < m_x.size(); ++i)
        {
            const double px = area.center().x() + (m_x[i] - centerX) * scale;
            const double py = area.center().y() - (m_y[i] - centerY) * scale;  // Y 向上为正
            painter.drawPoint(QPointF(px, py));
        }
    }

private:
    QVector<double> m_x;
    QVector<double> m_y;
    bool m_hasFrame = false;
    bool m_hasLockedCenterY = false;
    double m_lockedCenterY = 0.0;
    double m_halfSpanXmm = 120.0;  // 视野半宽(mm)：默认覆盖相机±70视野留余量，可经工具条调节
    double m_pointSize = 1.0;      // 点大小(px)，可经工具条调节
};

// 运行监控最大化窗口：流程启动时弹出。流程步骤/进度、实时激光线+相机图像、日志、暂停/继续/断点续焊按钮。
// 无 Q_OBJECT（按钮用 lambda 连接），照 GroovePointCloudDialog 定义在 cpp 内的先例。
class RunMonitorDialog final : public QDialog
{
public:
    explicit RunMonitorDialog(QWidget* parent = nullptr)
        : QDialog(parent)
    {
        setWindowTitle("运行监控");
        setWindowFlags(windowFlags() | Qt::WindowMinMaxButtonsHint);
        QVBoxLayout* root = new QVBoxLayout(this);
        root->setContentsMargins(14, 12, 14, 12);
        root->setSpacing(8);

        m_pStepLabel = new QLabel("等待流程启动...", this);
        m_pStepLabel->setStyleSheet("font-size: 20px; font-weight: 600; color: #9ED8DB;");
        root->addWidget(m_pStepLabel);
        m_pProgress = new QProgressBar(this);
        m_pProgress->setRange(0, 100);
        root->addWidget(m_pProgress);

        // 显示调节工具条（平板触控友好的大按钮）：点大小 − / +，视野 放大 / 缩小，实时生效。
        QHBoxLayout* viewToolbar = new QHBoxLayout();
        viewToolbar->setSpacing(8);
        auto makeToolButton = [this](const QString& text)
            {
                QPushButton* button = new QPushButton(text, this);
                button->setMinimumSize(64, 44);
                return button;
            };
        viewToolbar->addWidget(new QLabel("点大小:", this));
        QPushButton* pointSmallerBtn = makeToolButton("−");
        m_pPointSizeLabel = new QLabel("1.0", this);
        m_pPointSizeLabel->setAlignment(Qt::AlignCenter);
        m_pPointSizeLabel->setMinimumWidth(44);
        QPushButton* pointBiggerBtn = makeToolButton("＋");
        viewToolbar->addWidget(pointSmallerBtn);
        viewToolbar->addWidget(m_pPointSizeLabel);
        viewToolbar->addWidget(pointBiggerBtn);
        viewToolbar->addSpacing(24);
        viewToolbar->addWidget(new QLabel("视野:", this));
        QPushButton* zoomInBtn = makeToolButton("放大");
        m_pViewSpanLabel = new QLabel("240 mm", this);
        m_pViewSpanLabel->setAlignment(Qt::AlignCenter);
        m_pViewSpanLabel->setMinimumWidth(76);
        QPushButton* zoomOutBtn = makeToolButton("缩小");
        viewToolbar->addWidget(zoomInBtn);
        viewToolbar->addWidget(m_pViewSpanLabel);
        viewToolbar->addWidget(zoomOutBtn);
        viewToolbar->addStretch(1);
        root->addLayout(viewToolbar);
        connect(pointSmallerBtn, &QPushButton::clicked, this, [this]() { AdjustPointSize(-0.5); });
        connect(pointBiggerBtn, &QPushButton::clicked, this, [this]() { AdjustPointSize(0.5); });
        connect(zoomInBtn, &QPushButton::clicked, this, [this]() { AdjustViewSpan(-40.0); });
        connect(zoomOutBtn, &QPushButton::clicked, this, [this]() { AdjustViewSpan(40.0); });

        QHBoxLayout* live = new QHBoxLayout();
        live->setSpacing(10);
        m_pLineView = new LaserLineLiveView(this);
        m_pLineView->setMinimumHeight(320);
        m_pImageLabel = new QLabel("相机图像：等待图像帧...\n（需相机图像口支持）", this);
        m_pImageLabel->setAlignment(Qt::AlignCenter);
        m_pImageLabel->setMinimumHeight(320);
        m_pImageLabel->setStyleSheet("QLabel { background: #101C24; color: #6E8894; border: 1px solid #2B4552; border-radius: 8px; }");
        // Ignored：布局忽略 pixmap 撑起的 sizeHint，空间纯按 stretch 分配——否则 setPixmap(按size缩放)
        // 会反馈膨胀（图越来越大、点云被挤越来越小）。
        m_pImageLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
        live->addWidget(m_pLineView, 1);
        live->addWidget(m_pImageLabel, 1);
        root->addLayout(live, 1);

        m_pLog = new QPlainTextEdit(this);
        m_pLog->setReadOnly(true);
        m_pLog->document()->setMaximumBlockCount(800);
        m_pLog->setMaximumHeight(180);
        root->addWidget(m_pLog);

        QHBoxLayout* buttons = new QHBoxLayout();
        buttons->addStretch(1);
        m_pPauseBtn = new QPushButton("暂停", this);
        m_pResumeWeldBtn = new QPushButton("断点续焊", this);
        m_pPauseBtn->setMinimumSize(180, 52);
        m_pResumeWeldBtn->setMinimumSize(180, 52);
        m_pPauseBtn->setEnabled(false);
        m_pResumeWeldBtn->setEnabled(false);
        buttons->addWidget(m_pPauseBtn);
        buttons->addWidget(m_pResumeWeldBtn);
        root->addLayout(buttons);
    }

    void SetStep(const QString& text) { m_pStepLabel->setText(text); }
    void SetProgressValue(int value) { m_pProgress->setValue(std::clamp(value, 0, 100)); }
    void AppendLogLine(const QString& text) { m_pLog->appendPlainText(text); }
    void SetLine(const udpDataShow& frame) { m_pLineView->SetFrame(frame); }
    void ClearLine() { m_pLineView->ClearFrame(); }
    void SetImage(const QImage& image)
    {
        if (image.isNull())
        {
            return;
        }
        m_pImageLabel->setPixmap(QPixmap::fromImage(image).scaled(
            m_pImageLabel->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    }
    QPushButton* PauseButton() const { return m_pPauseBtn; }
    QPushButton* ResumeWeldButton() const { return m_pResumeWeldBtn; }

    void AdjustPointSize(double delta)
    {
        m_pointSize = std::clamp(m_pointSize + delta, 0.5, 6.0);
        m_pLineView->SetPointSize(m_pointSize);
        m_pPointSizeLabel->setText(QString::number(m_pointSize, 'f', 1));
    }

    void AdjustViewSpan(double delta)
    {
        m_viewSpanMm = std::clamp(m_viewSpanMm + delta, 40.0, 1000.0);
        m_pLineView->SetViewHalfSpan(m_viewSpanMm * 0.5);
        m_pViewSpanLabel->setText(QString("%1 mm").arg(m_viewSpanMm, 0, 'f', 0));
    }

protected:
    void closeEvent(QCloseEvent* event) override
    {
        // 流程仍在跑时只隐藏不销毁（监控窗口可随时经主界面流程再次弹出）。
        event->ignore();
        hide();
    }

private:
    QLabel* m_pStepLabel = nullptr;
    QProgressBar* m_pProgress = nullptr;
    LaserLineLiveView* m_pLineView = nullptr;
    QLabel* m_pImageLabel = nullptr;
    QPlainTextEdit* m_pLog = nullptr;
    QPushButton* m_pPauseBtn = nullptr;
    QPushButton* m_pResumeWeldBtn = nullptr;
    QLabel* m_pPointSizeLabel = nullptr;
    QLabel* m_pViewSpanLabel = nullptr;
    double m_pointSize = 1.0;
    double m_viewSpanMm = 240.0;
};

// ===== 相机时间补偿自动标定 =====
// 原理：时间偏差 δ 造成的轨迹偏移 = 扫描速度 × δ，方向沿运动方向。同一工件正/反向各扫一次，
// 拐点在扫描方向上的分裂量 S = 2·v·δ，故 δ = S/(2v)；建议补偿 = 当前补偿 − δ。
// 前置：统计时间对齐(方案A)已消除随机抖动(<2ms)，否则分裂量会被 ±60ms 随机项污染、标不出真值。
struct TimeOffsetCalibrationResult
{
    bool valid = false;
    QString error;
    int axisIndex = 1;              // 扫描方向轴 0=X 1=Y 2=Z
    int pairCount = 0;              // 参与统计的拐点配对数
    double splitMm = 0.0;           // 正向-反向 拐点中位分裂
    double madMm = 0.0;             // 配对差的中位绝对偏差(离散度)
    double delta0Ms = 0.0;          // 解算出的固有延迟
    double suggestedOffsetMs = 0.0; // 建议写入的 CameraTimeOffsetMs
};

// savedPath 为 ScanMoveAndCollect 输出（正常是 LaserPoint 下的焊接姿态文件路径；容错目录形态）。
QString ResolveCalibKeyPointsPath(const QString& savedPath)
{
    const QFileInfo info(savedPath);
    QDir dir = info.isFile() ? info.dir() : QDir(savedPath);
    if (!QFileInfo::exists(dir.filePath("PreciseLaserPoint_KeyPoints.txt"))
        && QFileInfo::exists(dir.filePath("LaserPoint/PreciseLaserPoint_KeyPoints.txt")))
    {
        dir = QDir(dir.filePath("LaserPoint"));
    }
    return dir.filePath("PreciseLaserPoint_KeyPoints.txt");
}

// 读取拐点在指定轴上的坐标。typeCode 传 "3"/"4" 只取内/外拐点，传空取两类全部。
// KeyPoints 行格式：source_index x y z type_code type_name source。
std::vector<double> ReadCornerAxisCoords(const QString& keyPointsPath, int axisIndex, const QString& typeCode, QString* error)
{
    std::vector<double> coords;
    QFile file(keyPointsPath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        if (error != nullptr)
        {
            *error = QString("打开拐点文件失败：%1").arg(keyPointsPath);
        }
        return coords;
    }
    QTextStream stream(&file);
    while (!stream.atEnd())
    {
        const QString line = stream.readLine().trimmed();
        if (line.isEmpty() || line.startsWith('#'))
        {
            continue;
        }
        const QStringList parts = line.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
        if (parts.size() < 5)
        {
            continue;
        }
        const QString type = parts[4];
        if (type != "3" && type != "4")
        {
            continue;
        }
        if (!typeCode.isEmpty() && type != typeCode)
        {
            continue;
        }
        bool okValue = false;
        const double value = parts[1 + axisIndex].toDouble(&okValue);
        if (okValue)
        {
            coords.push_back(value);
        }
    }
    return coords;
}

// 单一类型拐点的最近邻(±6mm)配对差集合：内拐只配内拐、外拐只配外拐，防止短坡工件上跨类型错配。
void AppendNearestPairDeltas(const std::vector<double>& forward, const std::vector<double>& reverse, std::vector<double>& deltas)
{
    for (const double value : forward)
    {
        double best = 0.0;
        double bestDistance = 1e18;
        for (const double candidate : reverse)
        {
            const double distance = std::abs(value - candidate);
            if (distance < bestDistance)
            {
                bestDistance = distance;
                best = candidate;
            }
        }
        if (bestDistance < 6.0)
        {
            deltas.push_back(value - best);
        }
    }
}

TimeOffsetCalibrationResult ComputeTimeOffsetCalibration(
    const QString& savedForward,
    const QString& savedReverse,
    const T_PRECISE_MEASURE_PARAM& param)
{
    TimeOffsetCalibrationResult result;

    // 带符号的行进方向分量：split = 2·v·δ·u_a，u_a 为起点→终点单位向量在所选轴上的分量。
    // 漏掉符号时，起点轴坐标大于终点的合法示教会解出反号补偿、越标越偏。
    const double signedDx = param.tEndPos.dX - param.tStartPos.dX;
    const double signedDy = param.tEndPos.dY - param.tStartPos.dY;
    const double signedDz = param.tEndPos.dZ - param.tStartPos.dZ;
    const double dx = std::abs(signedDx);
    const double dy = std::abs(signedDy);
    const double dz = std::abs(signedDz);
    result.axisIndex = (dx >= dy && dx >= dz) ? 0 : (dy >= dz ? 1 : 2);
    const double scanDistanceMm = std::sqrt(signedDx * signedDx + signedDy * signedDy + signedDz * signedDz);
    if (scanDistanceMm < 1.0)
    {
        result.error = "扫描起终点距离无效。";
        return result;
    }
    const double signedAxisDeltas[3] = { signedDx, signedDy, signedDz };
    const double axisComponent = signedAxisDeltas[result.axisIndex] / scanDistanceMm;
    if (std::abs(axisComponent) < 0.5)
    {
        result.error = "扫描方向与主轴夹角过大（斜扫），拐点轴向分裂无法可靠换算时间补偿。";
        return result;
    }

    QString readError;
    const QString forwardKeyPoints = ResolveCalibKeyPointsPath(savedForward);
    const QString reverseKeyPoints = ResolveCalibKeyPointsPath(savedReverse);
    const std::vector<double> forwardInner = ReadCornerAxisCoords(forwardKeyPoints, result.axisIndex, QStringLiteral("3"), &readError);
    const std::vector<double> forwardOuter = ReadCornerAxisCoords(forwardKeyPoints, result.axisIndex, QStringLiteral("4"), &readError);
    const std::vector<double> reverseInner = ReadCornerAxisCoords(reverseKeyPoints, result.axisIndex, QStringLiteral("3"), &readError);
    const std::vector<double> reverseOuter = ReadCornerAxisCoords(reverseKeyPoints, result.axisIndex, QStringLiteral("4"), &readError);
    const std::size_t forwardTotal = forwardInner.size() + forwardOuter.size();
    const std::size_t reverseTotal = reverseInner.size() + reverseOuter.size();
    if (forwardTotal < 3 || reverseTotal < 3)
    {
        result.error = readError.isEmpty()
            ? QString("拐点数量不足（正向=%1，反向=%2，各需≥3），无法标定。请确认工件在扫描范围内且能稳定识别拐点。")
                .arg(static_cast<int>(forwardTotal))
                .arg(static_cast<int>(reverseTotal))
            : readError;
        return result;
    }

    // 内拐配内拐、外拐配外拐（各自最近邻 ±6mm），取全部配对差的中位数为分裂量。
    std::vector<double> deltas;
    deltas.reserve(forwardTotal);
    AppendNearestPairDeltas(forwardInner, reverseInner, deltas);
    AppendNearestPairDeltas(forwardOuter, reverseOuter, deltas);
    result.pairCount = static_cast<int>(deltas.size());
    if (result.pairCount < 6)
    {
        result.error = QString("拐点配对数不足（%1<6），无法可靠标定。").arg(result.pairCount);
        return result;
    }
    std::sort(deltas.begin(), deltas.end());
    result.splitMm = deltas[deltas.size() / 2];
    std::vector<double> absDeviations;
    absDeviations.reserve(deltas.size());
    for (const double delta : deltas)
    {
        absDeviations.push_back(std::abs(delta - result.splitMm));
    }
    std::sort(absDeviations.begin(), absDeviations.end());
    result.madMm = absDeviations[absDeviations.size() / 2];

    const double speedMmPerSec = param.dScanSpeed / 60.0;
    if (speedMmPerSec <= 0.0)
    {
        result.error = "扫描速度无效，无法换算时间补偿。";
        return result;
    }
    result.delta0Ms = result.splitMm / (2.0 * speedMmPerSec * axisComponent) * 1000.0;
    result.suggestedOffsetMs = param.dCameraTimeOffsetMs - result.delta0Ms;

    if (result.madMm > 1.0)
    {
        result.error = QString("拐点配对离散度过大（MAD=%1 mm > 1.0 mm），疑似工件/夹持在两次扫描间移动，标定结果不可信。")
            .arg(result.madMm, 0, 'f', 2);
        return result;
    }
    // 可检测上限由拐点配对门限(±6mm)决定：|δ|max = 6mm/(2v)。超出即意味着分裂大于配对门限、结果已不可信。
    const double maxDetectableMs = 6.0 / (2.0 * speedMmPerSec) * 1000.0;
    if (std::abs(result.delta0Ms) > maxDetectableMs)
    {
        result.error = QString("解算延迟 %1 ms 超出当前速度下的可检测范围（±%2 ms，由拐点配对门限决定），请先排查时间轴/对齐设置或降低扫描速度后重试。")
            .arg(result.delta0Ms, 0, 'f', 1)
            .arg(maxDetectableMs, 0, 'f', 0);
        return result;
    }
    result.valid = true;
    return result;
}

// 把标定出的补偿值写入该机器人全部测量参数组（延迟是相机/链路属性，不随参数组变）。
bool WriteCameraTimeOffsetToAllScanGroups(const QString& robotName, double valueMs, int* groupsWritten, QString* error)
{
    COPini ini;
    if (!ini.SetFileName(ToUtf8StdString(RobotDataHelper::MeasureWeldParamPath(robotName))))
    {
        if (error != nullptr)
        {
            *error = "打开测量焊接参数失败。";
        }
        return false;
    }
    ini.SetSectionName("MeasureWeldGroups");
    int groupCount = 0;
    ini.ReadString(false, "GroupCount", &groupCount);
    if (groupCount <= 0)
    {
        if (error != nullptr)
        {
            *error = "未找到任何测量参数组。";
        }
        return false;
    }
    int written = 0;
    for (int index = 0; index < groupCount; ++index)
    {
        ini.SetSectionName(RobotDataHelper::MeasureWeldScanSectionName(index).toStdString());
        if (ini.WriteString("CameraTimeOffsetMs", valueMs, 3))
        {
            ++written;
        }
    }
    if (groupsWritten != nullptr)
    {
        *groupsWritten = written;
    }
    if (written != groupCount && error != nullptr)
    {
        *error = QString("部分参数组写入失败（%1/%2）。").arg(written).arg(groupCount);
    }
    return written == groupCount;
}

bool HasSkipScanRebuildInput(const QDir& dir)
{
    return QFileInfo::exists(dir.filePath(RAW_LASER_FILE_NAME))
        || QFileInfo::exists(dir.filePath(PRESERVE_PATH_FILE_NAME))
        || QFileInfo::exists(dir.filePath(WORKPIECE_CLOUD_FILE_NAME));
}

bool HasSkipScanExecutableHistory(const QDir& dir)
{
    return QFileInfo::exists(dir.filePath(WELD_POSE_SEAM_COMP_FILE_NAME))
        || QFileInfo::exists(dir.filePath(WELD_POSE_FILE_NAME));
}

QString ResolveLaserPointDirFromSelection(const QString& selectedDir)
{
    const QFileInfo selectedInfo(selectedDir);
    if (!selectedInfo.exists() || !selectedInfo.isDir())
    {
        return QString();
    }

    const QDir dir(selectedInfo.absoluteFilePath());
    if (HasSkipScanRebuildInput(dir) || HasSkipScanExecutableHistory(dir))
    {
        return dir.absolutePath();
    }

    const QString laserDir = dir.filePath("LaserPoint");
    const QDir nestedLaserDir(laserDir);
    if (HasSkipScanRebuildInput(nestedLaserDir) || HasSkipScanExecutableHistory(nestedLaserDir))
    {
        return nestedLaserDir.absolutePath();
    }

    return QString();
}

QString BuildWeldProcessGroupKey(const T_WELD_PARA& weld)
{
    return QString("%1|%2")
        .arg(DecodeRobotMessageText(weld.strWorkPeace))
        .arg(weld.dWeldAngleSize, 0, 'f', 3);
}

int CountWeldProcessLayers(const std::vector<T_WELD_PARA>& weldList, const QString& groupKey)
{
    int layerCount = 0;
    for (const T_WELD_PARA& weld : weldList)
    {
        if (BuildWeldProcessGroupKey(weld) == groupKey)
        {
            ++layerCount;
        }
    }
    return std::max(1, layerCount);
}

QString BuildPoseCompParamPath(const QString& robotName)
{
    return RobotDataHelper::BuildProjectPath(QString("Data/%1/WeldPoseCompParam.ini").arg(robotName));
}

QString BuildSeamCompParamPath(const QString& robotName)
{
    return RobotDataHelper::BuildProjectPath(QString("Data/%1/WeldSeamCompParam.ini").arg(robotName));
}

void LoadCompGroupCombo(
    QComboBox* combo,
    const QString& path,
    const QString& allSection,
    const QString& rowCountKey,
    const QString& groupCountKey,
    const QString& activeGroupIndexKey,
    const QString& groupSectionPrefix,
    const QString& defaultNamePrefix)
{
    if (combo == nullptr)
    {
        return;
    }

    const QSignalBlocker blocker(combo);
    combo->clear();

    int activeIndex = 0;
    int groupCount = 1;
    const bool hasConfig = ConfigDatabase::HasIniFile(path);
    if (hasConfig)
    {
        COPini ini;
        if (ini.SetFileName(ToUtf8StdString(path)))
        {
            ini.SetSectionName(ToUtf8StdString(allSection));
            ini.ReadString(false, ToUtf8StdString(activeGroupIndexKey), &activeIndex);
            int configuredGroupCount = 0;
            if (ini.ReadString(false, ToUtf8StdString(groupCountKey), &configuredGroupCount) > 0)
            {
                groupCount = std::max(1, configuredGroupCount);
            }
            else
            {
                int rowCount = COMP_SEGMENT_COUNT;
                ini.ReadString(false, ToUtf8StdString(rowCountKey), &rowCount);
                groupCount = std::max(1, (std::max(0, rowCount) + COMP_SEGMENT_COUNT - 1) / COMP_SEGMENT_COUNT);
            }

            activeIndex = std::clamp(activeIndex, 0, groupCount - 1);
            for (int index = 0; index < groupCount; ++index)
            {
                QString groupName = QString("%1%2").arg(defaultNamePrefix).arg(index + 1);
                std::string encodedName;
                ini.SetSectionName(ToUtf8StdString(QString("%1%2").arg(groupSectionPrefix).arg(index)));
                if (ini.ReadString(false, "Name", encodedName) > 0)
                {
                    const QString decodedName = DecodeRobotMessageText(encodedName).trimmed();
                    if (!decodedName.isEmpty())
                    {
                        groupName = decodedName;
                    }
                }
                combo->addItem(QString("%1 / Group%2").arg(groupName).arg(index), index);
            }
        }
    }

    if (combo->count() <= 0)
    {
        combo->addItem(QString("%1%2 / Group0").arg(defaultNamePrefix).arg(1), 0);
        activeIndex = 0;
    }
    combo->setCurrentIndex(std::clamp(activeIndex, 0, combo->count() - 1));
    combo->setEnabled(combo->count() > 0);
}

bool SaveActiveCompGroupIndex(
    const QString& path,
    const QString& allSection,
    const QString& activeGroupIndexKey,
    int activeIndex,
    QString& error)
{
    if (path.isEmpty())
    {
        error = "补偿参数配置库不可用。";
        return false;
    }
    if (!ConfigDatabase::HasIniFile(path) && activeIndex <= 0)
    {
        return true;
    }

    COPini ini;
    if (!ini.SetFileName(false, ToUtf8StdString(path)))
    {
        error = "打开补偿参数失败。";
        return false;
    }
    ini.SetSectionName(ToUtf8StdString(allSection));
    if (!ini.WriteString(ToUtf8StdString(activeGroupIndexKey), std::max(0, activeIndex)))
    {
        error = QStringLiteral("写入补偿组选择失败。");
        return false;
    }
    return true;
}
}

MeasureThenWeldDialog::MeasureThenWeldDialog(ContralUnit* pContralUnit, int unitIndex, StartCameraFunc startCamera, StopCameraFunc stopCamera, CameraCacheFunc cameraCacheForUnit, QWidget* parent)
    : QDialog(parent)
    , m_pContralUnit(pContralUnit)
    , m_unitIndex(unitIndex)
    , m_pService(new MeasureThenWeldService())
    , m_startCamera(startCamera)
    , m_stopCamera(stopCamera)
    , m_cameraCacheForUnit(cameraCacheForUnit)
    , m_pCameraCache(nullptr)
{
    setWindowTitle("先测后焊");
    ApplyUnifiedWindowChrome(this);
    ResizeWindowForAvailableGeometry(this, QSize(760, 500), 0.72, 0.66);
    setStyleSheet(
        "QDialog { background: #111820; color: #ECF3F4; }"
        "QPushButton { background: #233645; color: #F5FAFA; border: 1px solid #3C6173; border-radius: 12px; padding: 12px 18px; font-size: 16px; }"
        "QPushButton:hover { background: #2D5465; border-color: #72D4DD; }"
        "QPushButton:pressed { background: #18303B; }"
        "QPushButton:disabled { background: #27323A; color: #7D8B91; border-color: #364650; }"
        "QCheckBox { color: #BFE8EC; spacing: 8px; font-size: 15px; }"
        "QCheckBox::indicator { width: 18px; height: 18px; border: 1px solid #5C7A8B; background: #0B1117; }"
        "QCheckBox::indicator:checked { background: #1E8AA0; border-color: #72D4DD; }"
        "QDoubleSpinBox { background: #05090D; color: #F5FAFA; border: 1px solid #36586A; padding: 4px 8px; min-height: 26px; }"
        "QComboBox { background: #05090D; color: #F5FAFA; border: 1px solid #36586A; padding: 4px 8px; min-height: 26px; }"
        "QComboBox::drop-down { width: 34px; border-left: 1px solid #36586A; background: #05090D; }"
        "QComboBox::down-arrow { image: url(:/QtWidgetsApplication4/icons/chevron-down.svg); width: 12px; height: 8px; }"
        "QComboBox QAbstractItemView { background: #081018; color: #F5FAFA; selection-background-color: #2D7D8C; }"
        "QPlainTextEdit { background: #081018; color: #BFE8EC; border: 1px solid #2C4653; border-radius: 10px; padding: 8px; }"
        "QProgressBar { background: #081018; color: #F5FAFA; border: 1px solid #2C4653; border-radius: 8px; text-align: center; min-height: 18px; }"
        "QProgressBar::chunk { background: #2D8DA0; border-radius: 7px; }"
        "QLabel { color: #BACBD1; }");

    QVBoxLayout* rootLayout = new QVBoxLayout(this);

    QLabel* titleLabel = new QLabel("先测后焊功能");
    titleLabel->setStyleSheet("font-size: 22px; font-weight: bold; color: #F7FCFC;");
    rootLayout->addWidget(titleLabel);

    QLabel* hintLabel = new QLabel("预设参数：读取配置库中的当前测量焊接参数组，并执行安全姿态、扫描起点、扫描终点、收枪姿态；扫描段采集相机三维点，并在扫描后自动执行 PreservePath 拟合、焊道分类、焊接姿态生成和焊道补偿。也可以跳过扫描，直接选历史结果文件夹焊接。");
    hintLabel->setWordWrap(true);
    rootLayout->addWidget(hintLabel);

    QScrollArea* flowScrollArea = new QScrollArea(this);
    flowScrollArea->setObjectName("AdaptiveWindowScrollArea");
    ConfigureResponsiveScrollArea(flowScrollArea);

    QWidget* flowContent = new QWidget(flowScrollArea);
    flowContent->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
    QVBoxLayout* flowLayout = new QVBoxLayout(flowContent);
    flowLayout->setContentsMargins(0, 0, 8, 0);
    flowLayout->setSpacing(12);
    flowLayout->setSizeConstraint(QLayout::SetMinAndMaxSize);

    QGridLayout* selectorLayout = new QGridLayout();
    selectorLayout->setHorizontalSpacing(14);
    selectorLayout->setVerticalSpacing(8);
    selectorLayout->addWidget(new QLabel("机器人："), 0, 0);
    m_pRobotCombo = new QComboBox();
    m_pRobotCombo->setMinimumWidth(220);
    selectorLayout->addWidget(m_pRobotCombo, 0, 1);
    selectorLayout->addWidget(new QLabel("位置类型："), 0, 2);
    m_pParamGroupCombo = new QComboBox();
    m_pParamGroupCombo->setMinimumWidth(210);
    selectorLayout->addWidget(m_pParamGroupCombo, 0, 3);
    selectorLayout->addWidget(new QLabel("焊接工艺："), 1, 0);
    m_pWeldProcessCombo = new QComboBox();
    m_pWeldProcessCombo->setMinimumWidth(360);
    selectorLayout->addWidget(m_pWeldProcessCombo, 1, 1, 1, 3);
    selectorLayout->addWidget(new QLabel("姿态补偿组："), 2, 0);
    m_pPoseCompGroupCombo = new QComboBox();
    m_pPoseCompGroupCombo->setMinimumWidth(210);
    selectorLayout->addWidget(m_pPoseCompGroupCombo, 2, 1);
    selectorLayout->addWidget(new QLabel("焊道补偿组："), 2, 2);
    m_pSeamCompGroupCombo = new QComboBox();
    m_pSeamCompGroupCombo->setMinimumWidth(210);
    selectorLayout->addWidget(m_pSeamCompGroupCombo, 2, 3);
    selectorLayout->setColumnStretch(4, 1);
    flowLayout->addLayout(selectorLayout);

    QHBoxLayout* modeLayout = new QHBoxLayout();
    QLabel* modeLabel = new QLabel("运行模式：");
    modeLabel->setStyleSheet("font-weight: bold; color: #9ED8DB;");
    m_pActualWeldCheck = new QCheckBox("实际焊接");
    m_pActualWeldCheck->setChecked(true);
    QLabel* modeHintLabel = new QLabel("取消勾选后只空跑轨迹，使用焊接参数里的空跑速度。");
    // 最终轨迹点间距已统一到焊道补偿/工艺界面调整（此处原编辑框重复，已移除）。
    m_pSkipConfirmCheck = new QCheckBox("流程免确认");
    m_bSkipFlowConfirms.store(MeasureThenWeldRuntimeConfig::LoadSkipFlowConfirms());
    m_pSkipConfirmCheck->setChecked(m_bSkipFlowConfirms.load());
    m_pSkipConfirmCheck->setToolTip(
        "勾选后流程自动跳过这些确认弹窗：移动到扫描起点、扫描终点并采集、扫描收枪、\n"
        "扫描完成、焊前确认、焊后确认、补偿完成。\n"
        "以下确认不受影响，始终弹出：首次运动（扫描下枪安全位置）、\n"
        "进入焊接段（移动到焊接下枪安全位置）、扫描姿态翻转风险告警、跳过扫描的目录核对。");
    modeLayout->addWidget(modeLabel);
    modeLayout->addWidget(m_pActualWeldCheck);
    modeLayout->addWidget(m_pSkipConfirmCheck);
    modeLayout->addWidget(modeHintLabel);
    modeLayout->addStretch();
    flowLayout->addLayout(modeLayout);

    QGridLayout* buttonLayout = new QGridLayout();
    m_pPresetParamBtn = new QPushButton("预设参数");
    m_pSkipScanWeldBtn = new QPushButton("跳过扫描焊接");
    m_pLineScanProcessBtn = new QPushButton("线扫处理");
    m_pTimeOffsetCalibBtn = new QPushButton("相机时间补偿标定");
    m_pTimeOffsetCalibBtn->setToolTip("同一工件自动正/反向各扫一次，按两次拐点在扫描方向的分裂量解算相机链路固有延迟，确认后写入该机器人全部参数组的「相机时间补偿(ms)」。标定前请确认工件/夹持不会移动。");
    m_pPresetParamBtn->setMinimumHeight(64);
    m_pSkipScanWeldBtn->setMinimumHeight(64);
    m_pLineScanProcessBtn->setMinimumHeight(64);
    m_pTimeOffsetCalibBtn->setMinimumHeight(64);
    buttonLayout->addWidget(m_pPresetParamBtn, 0, 0);
    buttonLayout->addWidget(m_pLineScanProcessBtn, 0, 1);
    buttonLayout->addWidget(m_pSkipScanWeldBtn, 1, 0);
    buttonLayout->addWidget(m_pTimeOffsetCalibBtn, 1, 1);
    QPushButton* resumeWeldFlowBtn = new QPushButton("断点续焊");
    resumeWeldFlowBtn->setMinimumHeight(64);
    resumeWeldFlowBtn->setToolTip("从上次暂停落盘的V2断点继续焊接：只使用断点绑定的案例与实际执行轨迹，校验机器人端点、参数/工艺指纹和SHA256，再按毫米弧长精确回退。旧版断点不会自动匹配最新结果。");
    buttonLayout->addWidget(resumeWeldFlowBtn, 2, 0, 1, 2);
    connect(resumeWeldFlowBtn, &QPushButton::clicked, this, &MeasureThenWeldDialog::RunResumeWeldFlow);
    flowLayout->addLayout(buttonLayout);

    // 运行监控最大化窗口（实时点云/图像/日志/暂停控制都在里面，不挤主界面）；数据泵随流程启停。
    m_pRunMonitor = new RunMonitorDialog(this);
    connect(static_cast<RunMonitorDialog*>(m_pRunMonitor)->PauseButton(), &QPushButton::clicked,
        this, [this]() { OnPauseResumeClicked(); });
    connect(static_cast<RunMonitorDialog*>(m_pRunMonitor)->ResumeWeldButton(), &QPushButton::clicked,
        this, [this]() { OnResumeWeldClicked(); });

    m_pLiveViewTimer = new QTimer(this);
    m_pLiveViewTimer->setInterval(100);
    connect(m_pLiveViewTimer, &QTimer::timeout, this, [this]()
        {
            RunMonitorDialog* monitor = static_cast<RunMonitorDialog*>(m_pRunMonitor);
            if (m_pCameraCache == nullptr || monitor == nullptr)
            {
                return;
            }
            udpDataShow latestFrame;
            if (m_pCameraCache->Latest(latestFrame))
            {
                monitor->SetLine(latestFrame);
            }
            monitor->SetImage(m_pCameraCache->LatestImage());
        });

    QVBoxLayout* progressLayout = new QVBoxLayout();
    progressLayout->setSpacing(6);
    m_pProgressLabel = new QLabel("等待操作");
    m_pProgressLabel->setStyleSheet("color: #9ED8DB; font-weight: 600;");
    m_pProgressBar = new QProgressBar();
    m_pProgressBar->setRange(0, 100);
    m_pProgressBar->setValue(0);
    progressLayout->addWidget(m_pProgressLabel);
    progressLayout->addWidget(m_pProgressBar);
    flowLayout->addLayout(progressLayout);
    m_pProgressLabel->hide();
    m_pProgressBar->hide();
    flowLayout->addStretch(1);
    flowScrollArea->setWidget(flowContent);
    rootLayout->addWidget(flowScrollArea, 1);

    m_pProgressAnimationTimer = new QTimer(this);
    m_pProgressAnimationTimer->setInterval(300);
    connect(m_pProgressAnimationTimer, &QTimer::timeout, this, &MeasureThenWeldDialog::UpdateProgressAnimation);

    m_pLogText = new QPlainTextEdit();
    m_pLogText->setReadOnly(true);
    m_pLogText->document()->setMaximumBlockCount(1600);
    m_pLogText->setPlainText("流程日志：等待操作...");
    m_pLogText->setMinimumHeight(150);
    m_pLogText->setMaximumHeight(240);
    rootLayout->addWidget(m_pLogText);

    connect(m_pPresetParamBtn, &QPushButton::clicked, this, &MeasureThenWeldDialog::RunPresetParamFlow);
    connect(m_pSkipScanWeldBtn, &QPushButton::clicked, this, &MeasureThenWeldDialog::RunSkipScanWeldFlow);
    connect(m_pLineScanProcessBtn, &QPushButton::clicked, this, &MeasureThenWeldDialog::RunLineScanProcess);
    connect(m_pTimeOffsetCalibBtn, &QPushButton::clicked, this, &MeasureThenWeldDialog::RunCameraTimeOffsetCalibrationFlow);
    connect(m_pActualWeldCheck, &QCheckBox::toggled, this, &MeasureThenWeldDialog::SaveWeldModeToParam);
    connect(m_pSkipConfirmCheck, &QCheckBox::toggled, this, [this](bool checked)
        {
            m_bSkipFlowConfirms.store(checked);
            MeasureThenWeldRuntimeConfig::SaveSkipFlowConfirms(checked);
            AppendLog(checked
                ? "流程免确认已开启：中间步骤与信息类确认将自动继续（首次运动/进入焊接/风险告警/目录核对仍会弹出）。"
                : "流程免确认已关闭：所有流程确认弹窗恢复。");
        });
    connect(m_pRobotCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, &MeasureThenWeldDialog::OnRobotChanged);
    connect(m_pParamGroupCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, &MeasureThenWeldDialog::OnParamGroupChanged);
    connect(m_pWeldProcessCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, &MeasureThenWeldDialog::OnWeldProcessChanged);
    connect(m_pPoseCompGroupCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int)
        {
            if (m_bLoadingSelectors || m_bRunning)
            {
                return;
            }
            QString error;
            if (!SaveCurrentCompGroupSelections(error))
            {
                AppendLog("切换姿态补偿组失败：" + error);
                return;
            }
            AppendLog(QString("当前姿态补偿组已切换为：%1").arg(m_pPoseCompGroupCombo != nullptr ? m_pPoseCompGroupCombo->currentText() : QString()));
        });
    connect(m_pSeamCompGroupCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int)
        {
            if (m_bLoadingSelectors || m_bRunning)
            {
                return;
            }
            QString error;
            if (!SaveCurrentCompGroupSelections(error))
            {
                AppendLog("切换焊道补偿组失败：" + error);
                return;
            }
            AppendLog(QString("当前焊道补偿组已切换为：%1").arg(m_pSeamCompGroupCombo != nullptr ? m_pSeamCompGroupCombo->currentText() : QString()));
        });
    // 先显示界面，组合框数据(读配置库工艺/参数)延后一拍加载，主入口秒开、不在构造里阻塞。
    QTimer::singleShot(0, this, [this]() { LoadRobotList(); });
}

bool MeasureThenWeldDialog::IsRunning() const
{
    return m_bRunning;
}

void MeasureThenWeldDialog::ReloadSelectors()
{
    if (m_bRunning)
    {
        return;
    }
    LoadRobotList();
}

void MeasureThenWeldDialog::closeEvent(QCloseEvent* event)
{
    // 主栈中的 close 仅隐藏缓存页；流程线程/租约继续存活。允许返回主页使用
    // 不可删除的安全停止入口，真正析构和控制单元重载仍由全局租约门禁拦截。
    QDialog::closeEvent(event);
}

void MeasureThenWeldDialog::LoadRobotList()
{
    if (m_pRobotCombo == nullptr)
    {
        return;
    }

    m_bLoadingSelectors = true;
    m_pRobotCombo->clear();

    const QVector<RobotDataHelper::RobotInfo> robots = RobotDataHelper::LoadRobotList(m_pContralUnit);
    int selectedIndex = -1;
    for (const RobotDataHelper::RobotInfo& info : robots)
    {
        const int row = m_pRobotCombo->count();
        m_pRobotCombo->addItem(info.displayName, info.unitIndex);
        m_pRobotCombo->setItemData(row, info.robotName, Qt::UserRole + 1);
        if (info.unitIndex == m_unitIndex)
        {
            selectedIndex = row;
        }
    }

    if (selectedIndex < 0 && m_pRobotCombo->count() > 0)
    {
        selectedIndex = 0;
    }
    if (selectedIndex >= 0)
    {
        m_pRobotCombo->setCurrentIndex(selectedIndex);
        m_unitIndex = m_pRobotCombo->currentData().toInt();
    }

    m_bLoadingSelectors = false;
    LoadParamGroups();
    LoadWeldProcessList();
    LoadCompGroupLists();
    RefreshWeldModeFromParam();
}

void MeasureThenWeldDialog::LoadParamGroups()
{
    if (m_pParamGroupCombo == nullptr)
    {
        return;
    }

    m_bLoadingSelectors = true;
    m_pParamGroupCombo->clear();

    const QString robotName = CurrentRobotName();
    QString error;
    if (robotName.isEmpty() || !RobotDataHelper::EnsureMeasureWeldParamFile(robotName, &error))
    {
        if (!error.isEmpty())
        {
            AppendLog("读取位置类型失败：" + error);
        }
        m_bLoadingSelectors = false;
        return;
    }

    COPini ini;
    const QString path = RobotDataHelper::MeasureWeldParamPath(robotName);
    if (!ini.SetFileName(path.toLocal8Bit().constData()))
    {
        AppendLog("读取位置类型失败：打开参数数据失败：" + path);
        m_bLoadingSelectors = false;
        return;
    }

    int groupCount = 1;
    int useNo = 0;
    ini.SetSectionName("MeasureWeldGroups");
    ini.ReadString(false, "GroupCount", &groupCount);
    ini.ReadString(false, "UseGroupNo", &useNo);
    groupCount = std::max(1, groupCount);
    useNo = std::clamp(useNo, 0, groupCount - 1);
    for (int index = 0; index < groupCount; ++index)
    {
        std::string groupName;
        ini.ReadString(false, QString("Group%1Name").arg(index).toStdString(), groupName);
        const QString displayName = groupName.empty()
            ? QString("参数组%1").arg(index + 1)
            : DecodeRobotMessageText(groupName);
        m_pParamGroupCombo->addItem(QString("%1 / Group%2").arg(displayName).arg(index), index);
    }
    m_pParamGroupCombo->setCurrentIndex(useNo);
    m_bLoadingSelectors = false;
}

void MeasureThenWeldDialog::LoadWeldProcessList()
{
    if (m_pWeldProcessCombo == nullptr)
    {
        return;
    }

    m_bLoadingSelectors = true;
    m_pWeldProcessCombo->clear();
    m_pWeldProcessCombo->setEnabled(false);

    if (m_pContralUnit == nullptr
        || m_unitIndex < 0
        || m_unitIndex >= static_cast<int>(m_pContralUnit->m_vtContralUnitInfo.size()))
    {
        m_bLoadingSelectors = false;
        return;
    }

    WeldProcessFile processFile(*m_pContralUnit, m_unitIndex);
    if (!processFile.Init())
    {
        m_pWeldProcessCombo->addItem("请先在工艺界面重新创建工艺", -1);
        m_bLoadingSelectors = false;
        return;
    }
    const std::vector<T_WELD_PARA>& weldList = processFile.GetWeldParaList();
    QString selectedKey;
    if (!weldList.empty())
    {
        const int useIndex = std::clamp(processFile.GetUseWeldParaNo(), 0, static_cast<int>(weldList.size()) - 1);
        selectedKey = BuildWeldProcessGroupKey(weldList[useIndex]);
    }
    int selectedComboIndex = 0;
    QStringList seenKeys;
    for (int index = 0; index < static_cast<int>(weldList.size()); ++index)
    {
        const T_WELD_PARA& weld = weldList[index];
        const QString groupKey = BuildWeldProcessGroupKey(weld);
        if (seenKeys.contains(groupKey))
        {
            continue;
        }
        seenKeys.append(groupKey);
        const QString workpiece = DecodeRobotMessageText(weld.strWorkPeace);
        const int layerCount = CountWeldProcessLayers(weldList, groupKey);
        const QString layerText = layerCount > 1 ? QString(" | 共%1层").arg(layerCount) : QString();
        const QString displayName = QString("%1. %2 | 焊脚%3%4")
            .arg(m_pWeldProcessCombo->count() + 1)
            .arg(workpiece.isEmpty() ? QStringLiteral("未命名工艺") : workpiece)
            .arg(weld.dWeldAngleSize, 0, 'f', 1)
            .arg(layerText);
        m_pWeldProcessCombo->addItem(displayName, index);
        if (groupKey == selectedKey)
        {
            selectedComboIndex = m_pWeldProcessCombo->count() - 1;
        }
    }

    if (m_pWeldProcessCombo->count() > 0)
    {
        m_pWeldProcessCombo->setEnabled(true);
        m_pWeldProcessCombo->setCurrentIndex(std::clamp(selectedComboIndex, 0, m_pWeldProcessCombo->count() - 1));
    }
    else
    {
        const QString errorText = DecodeRobotMessageText(processFile.GetLastError());
        m_pWeldProcessCombo->addItem(errorText.isEmpty() ? QStringLiteral("未读取到焊接工艺") : errorText, -1);
    }

    m_bLoadingSelectors = false;
}

void MeasureThenWeldDialog::LoadCompGroupLists()
{
    const QString robotName = CurrentRobotName();
    m_bLoadingSelectors = true;
    LoadCompGroupCombo(
        m_pPoseCompGroupCombo,
        BuildPoseCompParamPath(robotName),
        "ALLWeldPoseComp",
        "PoseCompCount",
        POSE_GROUP_COUNT_KEY,
        POSE_ACTIVE_GROUP_INDEX_KEY,
        "WeldPoseCompGroup",
        "姿态补偿组");
    LoadCompGroupCombo(
        m_pSeamCompGroupCombo,
        BuildSeamCompParamPath(robotName),
        "ALLWeldSeamComp",
        "SeamCompCount",
        SEAM_GROUP_COUNT_KEY,
        SEAM_ACTIVE_GROUP_INDEX_KEY,
        "WeldSeamCompGroup",
        "焊道补偿组");
    m_bLoadingSelectors = false;
}

void MeasureThenWeldDialog::OnRobotChanged(int index)
{
    Q_UNUSED(index);
    if (m_bLoadingSelectors || m_pRobotCombo == nullptr)
    {
        return;
    }

    m_unitIndex = m_pRobotCombo->currentData().toInt();
    m_pCameraCache = ResolveCameraCacheForUnit(m_unitIndex);
    LoadParamGroups();
    LoadWeldProcessList();
    LoadCompGroupLists();
    RefreshWeldModeFromParam();
    AppendLog(QString("当前机器人已切换为：%1").arg(m_pRobotCombo->currentText()));
}

void MeasureThenWeldDialog::OnParamGroupChanged(int index)
{
    Q_UNUSED(index);
    if (m_bLoadingSelectors || m_bRunning)
    {
        return;
    }

    QString error;
    if (!SaveCurrentParamGroupSelection(error))
    {
        AppendLog("切换位置类型失败：" + error);
        return;
    }
    RefreshWeldModeFromParam();
    AppendLog(QString("当前位置类型已切换为：%1").arg(m_pParamGroupCombo != nullptr ? m_pParamGroupCombo->currentText() : QString()));
}

void MeasureThenWeldDialog::OnWeldProcessChanged(int index)
{
    Q_UNUSED(index);
    if (m_bLoadingSelectors || m_bRunning)
    {
        return;
    }

    QString error;
    if (!SaveCurrentWeldProcessSelection(error))
    {
        AppendLog("切换焊接工艺失败：" + error);
        return;
    }
    AppendLog(QString("当前焊接工艺已切换为：%1").arg(m_pWeldProcessCombo != nullptr ? m_pWeldProcessCombo->currentText() : QString()));
}

QString MeasureThenWeldDialog::CurrentRobotName() const
{
    if (m_pRobotCombo != nullptr && m_pRobotCombo->currentIndex() >= 0)
    {
        const QString robotName = m_pRobotCombo->currentData(Qt::UserRole + 1).toString();
        if (!robotName.trimmed().isEmpty())
        {
            return robotName.trimmed();
        }
    }

    RobotDriverAdaptor* driver = RobotDataHelper::GetRobotDriver(m_pContralUnit, m_unitIndex);
    if (driver != nullptr && !driver->m_sRobotName.empty())
    {
        return QString::fromStdString(driver->m_sRobotName);
    }
    return QString();
}

int MeasureThenWeldDialog::CurrentParamGroupIndex() const
{
    if (m_pParamGroupCombo != nullptr && m_pParamGroupCombo->currentIndex() >= 0)
    {
        return std::max(0, m_pParamGroupCombo->currentData().toInt());
    }
    return 0;
}

int MeasureThenWeldDialog::CurrentPoseCompGroupIndex() const
{
    if (m_pPoseCompGroupCombo != nullptr && m_pPoseCompGroupCombo->currentIndex() >= 0)
    {
        return std::max(0, m_pPoseCompGroupCombo->currentData().toInt());
    }
    return 0;
}

int MeasureThenWeldDialog::CurrentSeamCompGroupIndex() const
{
    if (m_pSeamCompGroupCombo != nullptr && m_pSeamCompGroupCombo->currentIndex() >= 0)
    {
        return std::max(0, m_pSeamCompGroupCombo->currentData().toInt());
    }
    return 0;
}

bool MeasureThenWeldDialog::SaveCurrentParamGroupSelection(QString& error) const
{
    const QString robotName = CurrentRobotName();
    if (robotName.isEmpty())
    {
        error = "未选择机器人。";
        return false;
    }
    return RobotDataHelper::WriteParamValue(
        RobotDataHelper::MeasureWeldParamPath(robotName),
        "MeasureWeldGroups",
        "UseGroupNo",
        QString::number(CurrentParamGroupIndex()),
        &error);
}

bool MeasureThenWeldDialog::SaveCurrentWeldProcessSelection(QString& error) const
{
    if (m_pWeldProcessCombo == nullptr || m_pWeldProcessCombo->currentIndex() < 0)
    {
        error = "未选择焊接工艺。";
        return false;
    }
    if (m_pContralUnit == nullptr
        || m_unitIndex < 0
        || m_unitIndex >= static_cast<int>(m_pContralUnit->m_vtContralUnitInfo.size()))
    {
        error = "未选择有效机器人。";
        return false;
    }

    const int weldProcessIndex = m_pWeldProcessCombo->currentData().toInt();
    if (weldProcessIndex < 0)
    {
        error = "当前焊接工艺无效。";
        return false;
    }

    WeldProcessFile processFile(*m_pContralUnit, m_unitIndex);
    if (!processFile.Init())
    {
        error = DecodeRobotMessageText(processFile.GetLastError());
        if (error.isEmpty())
        {
            error = "读取焊接工艺失败，请先在工艺界面重新创建工艺。";
        }
        return false;
    }
    if (!processFile.UpdateUseWeldParaNo(weldProcessIndex))
    {
        error = DecodeRobotMessageText(processFile.GetLastError());
        if (error.isEmpty())
        {
            error = "写入焊接工艺选择失败。";
        }
        return false;
    }
    return true;
}

bool MeasureThenWeldDialog::SaveCurrentCompGroupSelections(QString& error) const
{
    const QString robotName = CurrentRobotName();
    if (robotName.isEmpty())
    {
        error = "未选择机器人。";
        return false;
    }

    if (!SaveActiveCompGroupIndex(
        BuildPoseCompParamPath(robotName),
        "ALLWeldPoseComp",
        POSE_ACTIVE_GROUP_INDEX_KEY,
        CurrentPoseCompGroupIndex(),
        error))
    {
        return false;
    }

    if (!SaveActiveCompGroupIndex(
        BuildSeamCompParamPath(robotName),
        "ALLWeldSeamComp",
        SEAM_ACTIVE_GROUP_INDEX_KEY,
        CurrentSeamCompGroupIndex(),
        error))
    {
        return false;
    }
    return true;
}

CameraFrameCache* MeasureThenWeldDialog::ResolveCameraCacheForUnit(int unitIndex)
{
    if (m_cameraCacheForUnit)
    {
        return m_cameraCacheForUnit(unitIndex);
    }
    return m_pCameraCache;
}

RobotDriverAdaptor* MeasureThenWeldDialog::GetRobotDriver()
{
    if (m_pRobotCombo != nullptr && m_pRobotCombo->currentIndex() >= 0)
    {
        m_unitIndex = m_pRobotCombo->currentData().toInt();
    }

    if (m_pContralUnit == nullptr || m_unitIndex < 0 || m_unitIndex >= static_cast<int>(m_pContralUnit->m_vtContralUnitInfo.size()))
    {
        QMessageBox::warning(this, "先测后焊", "未找到可用的控制单元。");
        return nullptr;
    }

    RobotDriverAdaptor* pRobotDriverAdaptor = static_cast<RobotDriverAdaptor*>(m_pContralUnit->m_vtContralUnitInfo[m_unitIndex].pUnitDriver);
    if (pRobotDriverAdaptor == nullptr)
    {
        QMessageBox::warning(this, "先测后焊", "当前控制单元未创建驱动。");
        return nullptr;
    }

    return pRobotDriverAdaptor;
}

bool MeasureThenWeldDialog::LoadPresetParam(RobotDriverAdaptor* pRobotDriver, T_PRECISE_MEASURE_PARAM& param, QString& error)
{
    if (!SaveCurrentParamGroupSelection(error))
    {
        return false;
    }
    if (m_pWeldProcessCombo != nullptr && m_pWeldProcessCombo->isEnabled() && !SaveCurrentWeldProcessSelection(error))
    {
        return false;
    }
    if (!SaveCurrentCompGroupSelections(error))
    {
        return false;
    }
    return m_pService != nullptr && m_pService->LoadPresetParam(pRobotDriver, param, error);
}

bool MeasureThenWeldDialog::ReadPulse(COPini& ini, const std::string& prefix, T_ANGLE_PULSE& pulse, QString& error) const
{
    return m_pService != nullptr && m_pService->ReadPulse(ini, prefix, pulse, error);
}

bool MeasureThenWeldDialog::ReadCoors(COPini& ini, const std::string& prefix, T_ROBOT_COORS& coors, QString& error) const
{
    return m_pService != nullptr && m_pService->ReadCoors(ini, prefix, coors, error);
}

bool MeasureThenWeldDialog::ReadPulseList(COPini& ini, const std::string& countKey, const std::string& prefix, std::vector<T_ANGLE_PULSE>& pulses, QString& error) const
{
    return m_pService != nullptr && m_pService->ReadPulseList(ini, countKey, prefix, pulses, error);
}

bool MeasureThenWeldDialog::MovePulseAndWait(RobotDriverAdaptor* pRobotDriver, const T_ANGLE_PULSE& pulse, double speed, const QString& name)
{
    return m_pService != nullptr && m_pService->MovePulseAndWait(
        pRobotDriver,
        pulse,
        speed,
        name,
        [this](const QString& text) { AppendLog(text); },
        [this](const QString& text) { SetFlowStep(text); });
}

bool MeasureThenWeldDialog::MovePulseListAndWait(RobotDriverAdaptor* pRobotDriver, const std::vector<T_ANGLE_PULSE>& pulses, double speed, const QString& name)
{
    return m_pService != nullptr && m_pService->MovePulseListAndWait(
        pRobotDriver,
        pulses,
        speed,
        name,
        [this](const QString& text) { AppendLog(text); },
        [this](const QString& text) { SetFlowStep(text); });
}

bool MeasureThenWeldDialog::MoveCoorsAndWait(RobotDriverAdaptor* pRobotDriver, const T_ROBOT_COORS& coors, double speed, const QString& name)
{
    return m_pService != nullptr && m_pService->MoveCoorsAndWait(
        pRobotDriver,
        coors,
        speed,
        name,
        [this](const QString& text) { AppendLog(text); },
        [this](const QString& text) { SetFlowStep(text); });
}

bool MeasureThenWeldDialog::MoveScanStartSafeAndWait(RobotDriverAdaptor* pRobotDriver, const T_PRECISE_MEASURE_PARAM& param, double speed)
{
    return m_pService != nullptr && m_pService->MoveScanStartSafeAndWait(
        pRobotDriver,
        param,
        speed,
        [this](const QString& text) { AppendLog(text); },
        [this](const QString& text) { SetFlowStep(text); },
        [this](const QString& title, const QString& detail) { return ShowCheckpointDialog(title, detail); });
}

bool MeasureThenWeldDialog::MoveScanEndSafeAndWait(RobotDriverAdaptor* pRobotDriver, const T_PRECISE_MEASURE_PARAM& param, double speed)
{
    return m_pService != nullptr && m_pService->MoveScanEndSafeAndWait(
        pRobotDriver,
        param,
        speed,
        [this](const QString& text) { AppendLog(text); },
        [this](const QString& text) { SetFlowStep(text); });
}

bool MeasureThenWeldDialog::ScanMoveAndCollect(RobotDriverAdaptor* pRobotDriver, const T_PRECISE_MEASURE_PARAM& param, QString& savedPath)
{
    return m_pService != nullptr && m_pService->ScanMoveAndCollect(
        pRobotDriver,
        param,
        savedPath,
        [this](const QString& text) { AppendLog(text); },
        [this](const QString& text)
        {
            SetFlowStep(text);
            if (text.contains("扫描运动中"))
            {
                SetProgressBusy(40, text);
            }
            else if (text.contains("扫描完成"))
            {
                SetProgressBusy(62, text);
            }
            else if (text.contains("重新计算") || text.contains("特征分析") || text.contains("焊接姿态"))
            {
                SetProgressBusy(68, text);
            }
        },
        m_pCameraCache);
}

QString MeasureThenWeldDialog::BuildResultDir(const std::string& robotName) const
{
    return m_pService != nullptr ? m_pService->BuildResultDir(robotName) : QString();
}

bool MeasureThenWeldDialog::SaveTextLines(const QString& filePath, const std::vector<QString>& lines, QString& error) const
{
    return m_pService != nullptr && m_pService->SaveTextLines(filePath, lines, error);
}

void MeasureThenWeldDialog::SetScanDataUploadHook(std::function<void(const QString&)> hook)
{
    m_scanDataUploadHook = std::move(hook);
}

void MeasureThenWeldDialog::NotifyFlowResultForUpload(const QString& poseFilePath)
{
    if (!m_scanDataUploadHook || poseFilePath.isEmpty())
    {
        return;
    }
    // poseFilePath = Result/<机器人>/<案例>/LaserPoint/xxx.txt → 上传单位是案例目录。
    QDir caseDir = QFileInfo(poseFilePath).dir();
    if (!caseDir.cdUp())
    {
        return;
    }
    const QString casePath = caseDir.absolutePath();
    const auto hook = m_scanDataUploadHook;
    QMetaObject::invokeMethod(qApp, [hook, casePath]() { hook(casePath); }, Qt::QueuedConnection);
}

bool MeasureThenWeldDialog::HasLiveSession(const QString& actionName)
{
    if (!m_liveSessionGuard || !m_liveSessionGuard())
    {
        const QString message = QStringLiteral("账号会话已失效，已拒绝继续：%1").arg(actionName);
        AppendLog(message);
        SetFlowStep(message);
        return false;
    }
    return true;
}

bool MeasureThenWeldDialog::ConfirmContinue(const QString& actionName)
{
    if (QThread::currentThread() != thread())
    {
        // 流程在线程里跑，确认框必须切回 UI 线程并阻塞等待用户选择。
        bool confirmed = false;
        QPointer<MeasureThenWeldDialog> self(this);
        QMetaObject::invokeMethod(qApp, [self, actionName, &confirmed]()
            {
                if (self == nullptr)
                {
                    confirmed = false;
                    return;
                }
                confirmed = self->ConfirmContinue(actionName);
            }, Qt::BlockingQueuedConnection);
        return confirmed;
    }

    if (!HasLiveSession(actionName))
    {
        return false;
    }
    // 流程免确认只放行中间衔接运动；首次运动（扫描下枪安全位置）与
    // 进入焊接段（移动到焊接下枪安全位置并执行焊接轨迹）不在白名单，始终弹框。
    static const QStringList kSkippableActions = {
        QStringLiteral("移动到扫描起点"),
        QStringLiteral("扫描终点并采集相机点"),
        QStringLiteral("扫描收枪安全位置"),
    };
    if (m_bSkipFlowConfirms.load() && kSkippableActions.contains(actionName))
    {
        AppendLog(QString("（免确认）自动继续：%1").arg(actionName));
        return true;
    }

    SetFlowStep(QString("等待确认：%1").arg(actionName));

    const QMessageBox::StandardButton ret = QMessageBox::question(
        this,
        "先测后焊确认",
        QString("即将执行：%1\n\n请确认机器人周围安全，是否继续移动？\n选择“取消”将退出当前流程。").arg(actionName),
        QMessageBox::Ok | QMessageBox::Cancel,
        QMessageBox::Cancel);
    const bool confirmed = (ret == QMessageBox::Ok);
    AppendLog(QString("%1：%2").arg(actionName).arg(confirmed ? "已确认" : "已取消"));
    return confirmed;
}

bool MeasureThenWeldDialog::ShowCheckpointDialog(const QString& title, const QString& detail)
{
    if (QThread::currentThread() != thread())
    {
        bool confirmed = false;
        QPointer<MeasureThenWeldDialog> self(this);
        QMetaObject::invokeMethod(qApp, [self, title, detail, &confirmed]()
            {
                if (self == nullptr)
                {
                    confirmed = false;
                    return;
                }
                confirmed = self->ShowCheckpointDialog(title, detail);
            }, Qt::BlockingQueuedConnection);
        return confirmed;
    }

    if (!HasLiveSession(title))
    {
        return false;
    }
    // 流程免确认按标题白名单放行；不在白名单的（跳过扫描确认、扫描姿态翻转
    // 风险提醒等）一律照常弹框——新增的关键节点默认落在保留侧，安全优先。
    static const QStringList kSkippableTitles = {
        QStringLiteral("扫描完成"),
        QStringLiteral("补偿完成"),
        QStringLiteral("焊前确认"),
        QStringLiteral("焊后确认"),
    };
    if (m_bSkipFlowConfirms.load() && kSkippableTitles.contains(title))
    {
        AppendLog(QString("（免确认）关键节点[%1]自动继续。%2").arg(title).arg(detail));
        return true;
    }

    SetFlowStep(QString("关键节点确认：%1").arg(title));

    const QMessageBox::StandardButton ret = QMessageBox::question(
        this,
        title,
        detail + "\n\n选择“确定”继续，选择“取消”终止当前流程。",
        QMessageBox::Ok | QMessageBox::Cancel,
        QMessageBox::Ok);
    const bool confirmed = (ret == QMessageBox::Ok);
    AppendLog(QString("关键节点[%1]：%2").arg(title).arg(confirmed ? "已确认继续" : "已取消流程"));
    return confirmed;
}

bool MeasureThenWeldDialog::BlockedByOtherFlow(const QString& title)
{
    // 互锁：流程测试等其他流程正在驱动同一台机器人时，拦下本次启动。
    if (m_preStartGuard)
    {
        QString reason;
        if (!m_preStartGuard(reason))
        {
            QMessageBox::warning(this, title,
                reason.isEmpty() ? QStringLiteral("当前有流程正在运行，无法开始。") : reason);
            return true;
        }
    }
    return false;
}

void MeasureThenWeldDialog::RunPresetParamFlow()
{
    if (m_bRunning)
    {
        QMessageBox::information(this, "先测后焊", "流程正在运行。");
        return;
    }
    if (BlockedByOtherFlow("先测后焊"))
    {
        return;
    }

    RobotDriverAdaptor* pRobotDriver = GetRobotDriver();
    if (pRobotDriver == nullptr)
    {
        return;
    }

    T_PRECISE_MEASURE_PARAM param;
    QString error;
    if (!LoadPresetParam(pRobotDriver, param, error))
    {
        QMessageBox::warning(this, "预设参数", error);
        return;
    }
    param.bDoActualWeld = IsActualWeldModeChecked();
    if (!HasLiveSession(QStringLiteral("启动先测后焊预设流程")))
    {
        return;
    }
    QString leaseError;
    const auto operationLease = RobotOperationLease::TryAcquire(
        pRobotDriver, QStringLiteral("先测后焊预设流程"), &leaseError);
    if (!operationLease)
    {
        QMessageBox::warning(this, "先测后焊", leaseError);
        return;
    }
    QString invalidateError;
    if (!MeasureThenWeldService::InvalidateStoredWeldResumeCheckpoint(
        QString::fromStdString(param.sRobotName), invalidateError))
    {
        QMessageBox::warning(this, "先测后焊",
            invalidateError + QStringLiteral("；未开始相机或机器人动作。"));
        return;
    }
    const int unitIndexForRun = m_unitIndex;
    m_pCameraCache = nullptr;  // 相机启动成功后再解析，避免 runtime 重建期间保留旧缓存指针。
    SetRunning(true);
    ResetProgress("读取预设参数完成，准备启动相机");
    SetProgress(5, "读取预设参数完成");
    SetFlowStep("读取预设参数完成，准备启动相机");
    AppendLog(QString("已读取参数：%1，位置类型=%2 [%3]")
        .arg(QString::fromStdString(param.sIniFilePath))
        .arg(param.sParamGroupName)
        .arg(QString::fromStdString(param.sSectionName)));
    AppendLog(QString("焊接执行模式：%1，焊接速度=%2 mm/min，空跑速度=%3 mm/min，安全位速度=%4 mm/min")
        .arg(param.bDoActualWeld ? QStringLiteral("实际焊接") : QStringLiteral("空跑"))
        .arg(param.dWeldSpeedMmPerMin, 0, 'f', 3)
        .arg(param.dDryRunSpeedMmPerMin, 0, 'f', 3)
        .arg(param.dWeldSafeMoveSpeedMmPerMin, 0, 'f', 3));

    // 机器人运动和扫描采集放到后台线程，避免 UI 被 CheckRobotDone 和文件保存卡住。
    QPointer<MeasureThenWeldDialog> self(this);
    std::thread([self, pRobotDriver, param, unitIndexForRun, operationLease]()
        {
            bool ok = true;
            QString message;
            QString cameraIP;
            QString savedPath;
            QString executeSummary;
            CameraFrameCache* cameraCacheForRun = nullptr;

            if (self != nullptr)
            {
                self->SetProgressBusy(8, "正在启动扫描相机");
            }
            QMetaObject::invokeMethod(qApp, [self, &cameraIP, &ok, &cameraCacheForRun, unitIndexForRun]()
                {
                    // 相机 UDP 线程由主界面统一管理，这里通过回调启动。
                    if (self == nullptr)
                    {
                        ok = false;
                        return;
                    }
                    ok = self->m_startCamera ? self->m_startCamera(unitIndexForRun, cameraIP) : false;
                    if (ok)
                    {
                        // 启动回调在共享接收模式下可能重建 runtime；必须在成功后重新取得本轮缓存。
                        cameraCacheForRun = self->ResolveCameraCacheForUnit(unitIndexForRun);
                        self->m_pCameraCache = cameraCacheForRun;
                        ok = cameraCacheForRun != nullptr;
                    }
                }, Qt::BlockingQueuedConnection);

            if (!ok)
            {
                message = "相机启动失败，流程中止。";
            }
            else
            {
                QMetaObject::invokeMethod(qApp, [self, cameraIP]()
                    {
                        if (self != nullptr)
                        {
                            self->SetFlowStep(QString("相机接收已启动：%1，准备扫描下枪安全位置").arg(cameraIP));
                            self->SetProgress(12, "相机接收已启动");
                            self->AppendLog(QString("相机接收已启动：%1").arg(cameraIP));
                        }
                    }, Qt::QueuedConnection);

                MeasureThenWeldService::ScanCycleResult scanCycle;
                auto beforeScanAction = [self](const QString& action) -> bool
                    {
                        if (self == nullptr || !self->ConfirmContinue(action))
                        {
                            return false;
                        }
                        if (action == QStringLiteral("扫描下枪安全位置"))
                        {
                            self->SetFlowStep("准备移动到扫描下枪安全位置");
                            self->SetProgressBusy(18, "移动到扫描下枪安全位置");
                        }
                        else if (action == QStringLiteral("移动到扫描起点"))
                        {
                            self->SetFlowStep("准备移动到扫描起点");
                            self->SetProgressBusy(28, "移动到扫描起点");
                        }
                        else if (action == QStringLiteral("扫描终点并采集相机点"))
                        {
                            self->SetFlowStep("准备扫描终点并采集相机点");
                            self->SetProgressBusy(40, "扫描运动中，正在采集点云");
                        }
                        return true;
                    };
                ok = self != nullptr
                    && self->m_pService != nullptr
                    && self->m_pService->RunScanCycle(
                        pRobotDriver,
                        param,
                        SafeSpeed(param.dRunSpeed, 1.0),
                        cameraCacheForRun,
                        scanCycle,
                        [self](const QString& text) { if (self != nullptr) self->AppendLog(text); },
                        [self](const QString& text) { if (self != nullptr) self->SetFlowStep(text); },
                        [self](const QString& title, const QString& detail) -> bool
                        {
                            return self != nullptr && self->ShowCheckpointDialog(title, detail);
                        },
                        beforeScanAction);
                savedPath = scanCycle.weldPosePath;
                if (!ok && message.isEmpty())
                {
                    message = scanCycle.error.isEmpty()
                        ? QStringLiteral("扫描流程失败或已取消。")
                        : scanCycle.error;
                }
                if (ok)
                {
                    const QFileInfo weldPoseFileInfo(savedPath);
                    if (!weldPoseFileInfo.isFile())
                    {
                        if (self != nullptr)
                        {
                            self->AppendLog(QString("未生成可下发的焊接姿态文件，当前结果=%1").arg(savedPath));
                        }
                        ok = false;
                        message = "预设参数流程已完成测量，但未生成可下发的焊接姿态文件。";
                    }
                }
                if (ok)
                {
                    ok = self != nullptr && self->ShowCheckpointDialog(
                        "扫描完成",
                        QString("扫描、拟合、焊道分类和焊接姿态生成已完成。\n焊接姿态文件：%1").arg(savedPath));
                }
                if (ok)
                {
                    ok = self != nullptr && self->ConfirmContinue("移动到焊接下枪安全位置并执行焊接轨迹");
                }
                if (ok)
                {
                    QString executeError;
                    T_ROBOT_COORS startSafeCoors;
                    T_ROBOT_COORS endSafeCoors;
                    if (self != nullptr)
                    {
                        self->SetFlowStep("准备执行焊接轨迹");
                        self->SetProgressBusy(84, "执行焊接轨迹");
                        self->AppendLog(QString("开始执行焊接轨迹：%1").arg(savedPath));
                    }

                    ok = self != nullptr
                        && self->m_pService != nullptr
                        && self->m_pService->ExecuteWeldPoseFileWithSafePos(
                            pRobotDriver,
                            savedPath,
                            param,
                            executeSummary,
                            executeError,
                            &startSafeCoors,
                            &endSafeCoors,
                            [self](const QString& text) { if (self != nullptr) self->AppendLog(text); },
                            [self](const QString& text) { if (self != nullptr) self->SetFlowStep(text); },
                            [self](const QString& title, const QString& detail) -> bool
                            {
                                return self != nullptr && self->ShowCheckpointDialog(title, detail);
                            },
                            0.0,
                            true,
                            MeasureThenWeldService::WeldPoseSource::PointCloudProduction,
                            -1.0,
                            false,
                            [pRobotDriver]() { return RobotOperationLease::IsCancellationRequested(pRobotDriver); },
                            [self, pRobotDriver, param](const MeasureThenWeldService::WeldExecutionIdentity& identity, QString& prepareError)
                            {
                                return self != nullptr && self->PrepareActiveWeldCheckpoint(
                                    pRobotDriver,
                                    param,
                                    identity.qualityProofPosePath,
                                    identity.qualityProofPoseSha256,
                                    identity.sampledPosePath,
                                    identity.sampledPoseSha256,
                                    identity.sampledPoseSize,
                                    identity.programName,
                                    identity.sampledPointCount,
                                    identity.effectiveFinalStepMm,
                                    identity.parameterFingerprint,
                                    identity.resumeCheckpointSupported,
                                    identity.resumeUnsupportedReason,
                                    prepareError);
                            },
                            [self](bool completed)
                            {
                                if (self != nullptr)
                                {
                                    self->FinishActiveWeldExecution(completed);
                                }
                            });
                    if (ok)
                    {
                        if (self != nullptr)
                        {
                            self->AppendLog(QString("焊接轨迹执行完成：%1").arg(executeSummary));
                            self->NotifyFlowResultForUpload(savedPath);
                        }
                        message = QString("预设参数流程完成。\n结果位置：%1\n执行结果：%2\n下枪安全位置：%3\n收枪安全位置：%4")
                            .arg(savedPath)
                            .arg(executeSummary)
                            .arg(QString("%1, %2, %3")
                                .arg(startSafeCoors.dX, 0, 'f', 3)
                                .arg(startSafeCoors.dY, 0, 'f', 3)
                                .arg(startSafeCoors.dZ, 0, 'f', 3))
                            .arg(QString("%1, %2, %3")
                                .arg(endSafeCoors.dX, 0, 'f', 3)
                                .arg(endSafeCoors.dY, 0, 'f', 3)
                                .arg(endSafeCoors.dZ, 0, 'f', 3));
                    }
                    else
                    {
                        if (self != nullptr)
                        {
                            self->AppendLog(QString("焊接轨迹执行失败：%1").arg(executeError));
                        }
                        message = QString("预设参数流程已完成测量，但焊接轨迹执行失败。\n%1").arg(executeError);
                    }
                }
                else if (message.isEmpty())
                {
                    message = "预设参数流程失败，请查看流程日志。";
                }
            }

            QMetaObject::invokeMethod(qApp, [self, message, ok]()
                {
                    if (self == nullptr)
                    {
                        return;
                    }
                    if (self->m_stopCamera)
                    {
                        // 流程正常完成、失败或用户取消，都会释放流程侧相机占用。
                        self->m_stopCamera();
                    }
                    self->SetFlowStep(ok ? "流程完成" : "流程失败，请查看流程日志");
                    self->FinishProgress(ok, ok ? QStringLiteral("流程完成") : QStringLiteral("流程失败，请查看流程日志"));
                    self->AppendLog(ok ? "流程完成。" : "流程失败。");
                    self->SetRunning(false);
                    ShowNonModalFlowResult(
                        self,
                        ok ? QMessageBox::Information : QMessageBox::Warning,
                        "预设参数",
                        message);
                }, Qt::QueuedConnection);
        }).detach();
}

void MeasureThenWeldDialog::RunSkipScanWeldFlow()
{
    if (m_bRunning)
    {
        QMessageBox::information(this, "先测后焊", "流程正在运行。");
        return;
    }
    if (BlockedByOtherFlow("先测后焊"))
    {
        return;
    }

    RobotDriverAdaptor* pRobotDriver = GetRobotDriver();
    if (pRobotDriver == nullptr)
    {
        return;
    }

    T_PRECISE_MEASURE_PARAM param;
    QString error;
    if (!LoadPresetParam(pRobotDriver, param, error))
    {
        QMessageBox::warning(this, "跳过扫描焊接", error);
        return;
    }
    param.bDoActualWeld = IsActualWeldModeChecked();

    const QString defaultDir = RobotDataHelper::BuildProjectPath(
        QString("Result/%1").arg(QString::fromStdString(param.sRobotName)));
    const QString selectedDir = QFileDialog::getExistingDirectory(
        this,
        "选择先测后焊结果文件夹",
        defaultDir,
        QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    if (selectedDir.isEmpty())
    {
        return;
    }

    const QString laserDir = ResolveLaserPointDirFromSelection(selectedDir);
    if (laserDir.isEmpty())
    {
        QMessageBox::warning(
            this,
            "跳过扫描焊接",
            QString("在所选目录中未找到原始激光点、PreservePath、局部完整点云、历史姿态文件或历史补偿文件。\n请选择结果目录本身，或其下的 LaserPoint 目录。"));
        return;
    }

    const QString preservePath = QDir(laserDir).filePath(PRESERVE_PATH_FILE_NAME);
    const QString poseFilePath = QDir(laserDir).filePath(WELD_POSE_FILE_NAME);
    const QString seamCompPath = QDir(laserDir).filePath(WELD_POSE_SEAM_COMP_FILE_NAME);
    const bool canRebuildFromLaser = HasSkipScanRebuildInput(QDir(laserDir));
    const bool hasExistingPoseFile = QFileInfo::exists(poseFilePath);
    const bool hasExistingSeamCompFile = QFileInfo::exists(seamCompPath);

    if (!canRebuildFromLaser && !hasExistingPoseFile && !hasExistingSeamCompFile)
    {
        QMessageBox::warning(
            this,
            "跳过扫描焊接",
            QString("所选目录缺少可重建或可执行的历史文件。\n需要至少包含 %1、%2、%3、%4 或 %5。")
                .arg(RAW_LASER_FILE_NAME)
                .arg(PRESERVE_PATH_FILE_NAME)
                .arg(WORKPIECE_CLOUD_FILE_NAME)
                .arg(WELD_POSE_FILE_NAME)
                .arg(WELD_POSE_SEAM_COMP_FILE_NAME));
        return;
    }

    if (!HasLiveSession(QStringLiteral("启动跳过扫描焊接流程")))
    {
        return;
    }
    QString leaseError;
    const auto operationLease = RobotOperationLease::TryAcquire(
        pRobotDriver, QStringLiteral("跳过扫描焊接流程"), &leaseError);
    if (!operationLease)
    {
        QMessageBox::warning(this, "跳过扫描焊接", leaseError);
        return;
    }
    QString invalidateError;
    if (!MeasureThenWeldService::InvalidateStoredWeldResumeCheckpoint(
        QString::fromStdString(param.sRobotName), invalidateError))
    {
        QMessageBox::warning(this, "跳过扫描焊接",
            invalidateError + QStringLiteral("；未开始处理或机器人动作。"));
        return;
    }

    SetRunning(true);
    ResetProgress("已选择历史结果，准备重新计算");
    SetProgress(8, "已选择历史结果");
    SetFlowStep("已选择历史结果，准备重新计算三份焊接文件");
    AppendLog(QString("跳过扫描模式：结果目录=%1").arg(selectedDir));
    AppendLog(QString("LaserPoint目录=%1").arg(laserDir));
    AppendLog(QString("PreservePath文件将输出到=%1").arg(preservePath));
    AppendLog(QString("姿态文件=%1").arg(poseFilePath));
    AppendLog(QString("补偿后文件将输出到=%1").arg(seamCompPath));
    AppendLog(QString("焊接执行模式：%1，焊接速度=%2 mm/min，空跑速度=%3 mm/min，安全位速度=%4 mm/min，示教姿态=%5，RZ增益=%6 deg，爬坡RZ夹紧=[%7, %8] deg")
        .arg(param.bDoActualWeld ? QStringLiteral("实际焊接") : QStringLiteral("空跑"))
        .arg(param.dWeldSpeedMmPerMin, 0, 'f', 3)
        .arg(param.dDryRunSpeedMmPerMin, 0, 'f', 3)
        .arg(param.dWeldSafeMoveSpeedMmPerMin, 0, 'f', 3)
        .arg(param.bUseTaughtWeldPose ? QStringLiteral("启用") : QStringLiteral("未启用"))
        .arg(param.dWeldRzGainDeg, 0, 'f', 3)
        .arg(param.dSlopeRzMinDeg, 0, 'f', 3)
        .arg(param.dSlopeRzMaxDeg, 0, 'f', 3));

    QPointer<MeasureThenWeldDialog> self(this);
    std::thread([self,
                 pRobotDriver,
                 param,
                 selectedDir,
                 laserDir,
                 preservePath = QString(preservePath),
                 poseFilePath = QString(poseFilePath),
                 seamCompPath = QString(seamCompPath),
                 canRebuildFromLaser,
                 hasExistingPoseFile,
                 hasExistingSeamCompFile,
                 operationLease]() mutable
        {
            bool ok = true;
            QString message;
            QString processError;
            QString rebuildSummary;
            QString executeSummary;
            QString rebuildPreservePath = preservePath;
            QString rebuildPoseFilePath = poseFilePath;
            QString rebuildSeamCompPath = seamCompPath;
            T_ROBOT_COORS startSafeCoors;
            T_ROBOT_COORS endSafeCoors;

            if (self != nullptr)
            {
                ok = self->ShowCheckpointDialog(
                    "跳过扫描确认",
                    QString("已选择结果目录：%1\n将按当前参数重新计算：\n%2\n%3\n%4")
                        .arg(selectedDir)
                        .arg(preservePath)
                        .arg(poseFilePath)
                        .arg(seamCompPath));
            }

            if (ok && canRebuildFromLaser)
            {
                if (self != nullptr)
                {
                    self->SetFlowStep("正在从历史LaserPoint重新计算三份焊接文件");
                    self->SetProgressBusy(28, "正在从历史LaserPoint重新计算焊接文件");
                }
                ok = self != nullptr
                    && self->m_pService != nullptr
                    && self->m_pService->RebuildWeldFilesFromLaserDir(
                        param,
                        laserDir,
                        rebuildPreservePath,
                        rebuildPoseFilePath,
                        rebuildSeamCompPath,
                        rebuildSummary,
                        processError,
                        [self](const QString& text) { if (self != nullptr) self->AppendLog(text); },
                        [self](const QString& text) { if (self != nullptr) self->SetFlowStep(text); });
                if (self != nullptr)
                {
                    if (ok)
                    {
                        self->AppendLog(QString("跳过扫描重建完成：%1").arg(rebuildSummary));
                        preservePath = rebuildPreservePath;
                        poseFilePath = rebuildPoseFilePath;
                        seamCompPath = rebuildSeamCompPath;
                    }
                    else
                    {
                        self->AppendLog(QString("跳过扫描重建文件失败：%1").arg(processError));
                    }
                }
            }
            else if (ok && hasExistingSeamCompFile)
            {
                rebuildSummary = QString("未找到原始激光点/PreservePath/局部完整点云，跳过重建，直接使用已有补偿文件：%1")
                    .arg(seamCompPath);
                if (self != nullptr)
                {
                    self->SetFlowStep("使用已有历史补偿文件，准备执行焊接轨迹");
                    self->SetProgress(48, "使用已有补偿文件");
                    self->AppendLog(rebuildSummary);
                }
            }
            else if (ok && hasExistingPoseFile)
            {
                if (self != nullptr)
                {
                    self->SetFlowStep("正在从历史姿态文件重新生成焊道补偿文件");
                    self->SetProgressBusy(42, "正在生成焊道补偿文件");
                    self->AppendLog(QString("未找到原始激光点/PreservePath/局部完整点云，改用历史姿态文件重新生成补偿：%1").arg(poseFilePath));
                }

                QString seamCompSummary;
                ok = self != nullptr
                    && self->m_pService != nullptr
                    && self->m_pService->ApplyWeldSeamCompToPoseFile(
                        QString::fromStdString(param.sRobotName),
                        poseFilePath,
                        seamCompPath,
                        seamCompSummary,
                        processError);
                if (ok)
                {
                    rebuildSummary = QString("历史姿态文件补偿完成：Pose=%1；SeamComp=%2；%3")
                        .arg(poseFilePath, seamCompPath, seamCompSummary);
                    if (self != nullptr)
                    {
                        self->AppendLog(rebuildSummary);
                    }
                }
                else if (self != nullptr)
                {
                    self->AppendLog(QString("历史姿态文件补偿失败：%1").arg(processError));
                }
            }

            if (ok)
            {
                ok = self != nullptr && self->ShowCheckpointDialog(
                    "补偿完成",
                    QString("PreservePath：%1\n姿态文件：%2\n补偿文件：%3\n%4")
                        .arg(preservePath)
                        .arg(poseFilePath)
                        .arg(seamCompPath)
                        .arg(rebuildSummary));
            }

            if (ok)
            {
                ok = self != nullptr && self->ConfirmContinue("移动到焊接下枪安全位置并执行焊接轨迹");
            }

            if (ok)
            {
                if (self != nullptr)
                {
                    self->SetFlowStep("准备执行跳过扫描后的焊接轨迹");
                    self->SetProgressBusy(74, "执行焊接轨迹");
                    self->AppendLog(QString("开始执行焊接轨迹：%1").arg(seamCompPath));
                }

                ok = self != nullptr
                    && self->m_pService != nullptr
                    && self->m_pService->ExecuteWeldPoseFileWithSafePos(
                        pRobotDriver,
                        seamCompPath,
                        param,
                        executeSummary,
                        processError,
                        &startSafeCoors,
                        &endSafeCoors,
                        [self](const QString& text) { if (self != nullptr) self->AppendLog(text); },
                        [self](const QString& text) { if (self != nullptr) self->SetFlowStep(text); },
                        [self](const QString& title, const QString& detail) -> bool
                        {
                            return self != nullptr && self->ShowCheckpointDialog(title, detail);
                        },
                        0.0,
                        true,
                        MeasureThenWeldService::WeldPoseSource::PointCloudProduction,
                        -1.0,
                        false,
                        [pRobotDriver]() { return RobotOperationLease::IsCancellationRequested(pRobotDriver); },
                        [self, pRobotDriver, param](const MeasureThenWeldService::WeldExecutionIdentity& identity, QString& prepareError)
                        {
                            return self != nullptr && self->PrepareActiveWeldCheckpoint(
                                pRobotDriver,
                                param,
                                identity.qualityProofPosePath,
                                identity.qualityProofPoseSha256,
                                identity.sampledPosePath,
                                identity.sampledPoseSha256,
                                identity.sampledPoseSize,
                                identity.programName,
                                identity.sampledPointCount,
                                identity.effectiveFinalStepMm,
                                identity.parameterFingerprint,
                                identity.resumeCheckpointSupported,
                                identity.resumeUnsupportedReason,
                                prepareError);
                        },
                        [self](bool completed)
                        {
                            if (self != nullptr)
                            {
                                self->FinishActiveWeldExecution(completed);
                            }
                        });
                if (ok)
                {
                    if (self != nullptr)
                    {
                        self->AppendLog(QString("焊接轨迹执行完成：%1").arg(executeSummary));
                    }
                    message = QString("跳过扫描焊接完成。\n结果目录：%1\n姿态文件：%2\n补偿文件：%3\n执行结果：%4\n下枪安全位置：%5\n收枪安全位置：%6")
                        .arg(selectedDir)
                        .arg(poseFilePath)
                        .arg(seamCompPath)
                        .arg(executeSummary)
                        .arg(QString("%1, %2, %3")
                            .arg(startSafeCoors.dX, 0, 'f', 3)
                            .arg(startSafeCoors.dY, 0, 'f', 3)
                            .arg(startSafeCoors.dZ, 0, 'f', 3))
                        .arg(QString("%1, %2, %3")
                            .arg(endSafeCoors.dX, 0, 'f', 3)
                            .arg(endSafeCoors.dY, 0, 'f', 3)
                            .arg(endSafeCoors.dZ, 0, 'f', 3));
                }
                else
                {
                    if (self != nullptr)
                    {
                        self->AppendLog(QString("焊接轨迹执行失败：%1").arg(processError));
                    }
                    message = QString("跳过扫描后焊接失败。\n%1").arg(processError);
                }
            }
            else if (message.isEmpty())
            {
                message = processError.isEmpty()
                    ? QString("跳过扫描焊接流程失败，请查看流程日志。")
                    : QString("跳过扫描焊接流程失败。\n%1").arg(processError);
            }

            QMetaObject::invokeMethod(qApp, [self, message, ok]()
                {
                    if (self == nullptr)
                    {
                        return;
                    }
                    self->SetFlowStep(ok ? "流程完成" : "流程失败，请查看流程日志");
                    self->FinishProgress(ok, ok ? QStringLiteral("跳过扫描焊接完成") : QStringLiteral("跳过扫描焊接失败"));
                    self->AppendLog(ok ? "跳过扫描焊接流程完成。" : "跳过扫描焊接流程失败。");
                    self->SetRunning(false);
                    ShowNonModalFlowResult(
                        self,
                        ok ? QMessageBox::Information : QMessageBox::Warning,
                        "跳过扫描焊接",
                        message);
                }, Qt::QueuedConnection);
        }).detach();
}

void MeasureThenWeldDialog::RunLineScanProcess()
{
    SetFlowStep("大线扫粗定位功能暂未接入");
    AppendLog("线扫处理是整体大范围扫描获取多个焊道粗定位，当前精测点云库不接到这里。");
    QMessageBox::information(
        this,
        "线扫处理",
        "线扫处理用于整体大范围扫描和多个焊道粗定位。\n当前新增的点云库已接入预设参数后的局部精测量流程。");
}

bool MeasureThenWeldDialog::PrepareActiveWeldCheckpoint(
    RobotDriverAdaptor* pRobotDriver,
    const T_PRECISE_MEASURE_PARAM& param,
    const QString& sourcePosePath,
    const QString& sourcePoseSha256,
    const QString& sampledPosePath,
    const QString& sampledPoseSha256,
    qint64 sampledPoseSize,
    const QString& programName,
    int sampledPointCount,
    double effectiveFinalStepMm,
    const QString& parameterFingerprint,
    bool resumeCheckpointSupported,
    const QString& resumeUnsupportedReason,
    QString& error)
{
    error.clear();
    // FANUC 暂停尚未接入；不能因为 STEP 的断点能力阻断 FANUC 原有焊接流程。
    if (dynamic_cast<STEPRobotCtrl*>(pRobotDriver) == nullptr)
    {
        ClearActiveWeldCheckpoint();
        return true;
    }

    const QString activeOwner = RobotOperationLease::CurrentOwner(pRobotDriver);
    const bool isResumeFlow = activeOwner == QStringLiteral("断点续焊流程");
    if (activeOwner != QStringLiteral("先测后焊预设流程")
        && activeOwner != QStringLiteral("跳过扫描焊接流程")
        && !isResumeFlow)
    {
        error = QStringLiteral("冻结STEP焊接断点上下文失败：当前页面未持有允许的硬件操作租约。");
        return false;
    }
    if (!resumeCheckpointSupported && isResumeFlow)
    {
        error = resumeUnsupportedReason.isEmpty()
            ? QStringLiteral("当前焊接工艺无法绑定真实执行轨迹，暂停/断点续焊已禁用。")
            : resumeUnsupportedReason;
        return false;
    }

    static const QRegularExpression sha256Pattern(QStringLiteral("^[0-9a-fA-F]{64}$"));
    if (pRobotDriver == nullptr || programName.trimmed().isEmpty()
        || sampledPointCount < 2
        || sampledPoseSize <= 0
        || !sha256Pattern.match(sourcePoseSha256).hasMatch()
        || !sha256Pattern.match(sampledPoseSha256).hasMatch()
        || !std::isfinite(effectiveFinalStepMm) || effectiveFinalStepMm <= 0.0
        || parameterFingerprint.size() != 64)
    {
        error = QStringLiteral("冻结STEP焊接断点上下文失败：程序、轨迹快照、点数、点距或工艺指纹无效。");
        return false;
    }

    WeldResumePlanner::CheckpointRecord record;
    record.state = QStringLiteral("prepared");
    record.checkpointId = QUuid::createUuid().toString(QUuid::WithoutBraces);
    record.createdAtUtc = QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs);
    record.robotName = QString::fromStdString(param.sRobotName).trimmed();
    record.robotType = RobotDriverTypeName(pRobotDriver);
    record.robotEndpoint = RobotOperationLease::PersistentEndpointIdentity(pRobotDriver);
    record.paramGroupIndex = param.nParamGroupIndex;
    record.paramGroupName = param.sParamGroupName;
    record.scanSection = QString::fromStdString(param.sSectionName);
    record.weldSection = QString::fromStdString(param.sWeldSectionName);
    record.parameterFingerprint = parameterFingerprint;
    record.programName = programName.trimmed();
    record.weldDirection = param.nWeldDirection < 0 ? -1 : 1;
    record.actualWeld = param.bDoActualWeld;
    record.finalStepMm = effectiveFinalStepMm;
    record.backtrackMm = param.dResumeBacktrackMm;

    if (record.robotName.isEmpty() || record.robotType != QStringLiteral("STEP")
        || record.robotEndpoint.isEmpty()
        || !std::isfinite(record.backtrackMm) || record.backtrackMm < 0.0)
    {
        error = QStringLiteral("冻结STEP焊接断点上下文失败：机器人持久端点或回退距离无效。");
        return false;
    }

    const QString projectRoot = RobotDataHelper::FindProjectRootPath();
    if (!WeldResumePlanner::BindTrajectoryIdentity(
        projectRoot, sampledPosePath, record.robotName, record, &error))
    {
        return false;
    }
    if (record.trajectorySha256.compare(sampledPoseSha256, Qt::CaseInsensitive) != 0
        || record.trajectorySize != sampledPoseSize
        || record.trajectoryPointCount != sampledPointCount)
    {
        error = QStringLiteral(
            "最终执行轨迹在保存与冻结之间发生变化：SHA256、大小或点数与 Service 快照不一致。");
        return false;
    }

    const QFileInfo sourceInfo(sourcePosePath);
    const QString sourceAbsolute = QDir::cleanPath(QDir::fromNativeSeparators(
        sourceInfo.canonicalFilePath().isEmpty()
            ? sourceInfo.absoluteFilePath()
            : sourceInfo.canonicalFilePath()));
    const QString sourceRelative = QDir::cleanPath(QDir::fromNativeSeparators(
        QDir(projectRoot).relativeFilePath(sourceAbsolute)));
    if (!sourceInfo.isFile() || QDir::isAbsolutePath(sourceRelative)
        || sourceRelative == QStringLiteral("..")
        || sourceRelative.startsWith(QStringLiteral("../"))
        || !sourceRelative.startsWith(record.caseRelativeDir + QLatin1Char('/'), Qt::CaseInsensitive))
    {
        error = QString("源轨迹不在绑定案例目录内：%1").arg(sourceAbsolute);
        return false;
    }
    const QString currentSourceSha256 = WeldResumePlanner::ComputeFileSha256(sourceAbsolute, &error);
    if (currentSourceSha256.compare(sourcePoseSha256, Qt::CaseInsensitive) != 0)
    {
        if (error.isEmpty())
        {
            error = QStringLiteral("质量证明源轨迹在授权与冻结之间发生变化，拒绝生成断点。");
        }
        return false;
    }
    record.sourceTrajectoryRelativePath = sourceRelative;
    record.sourceTrajectorySha256 = sourcePoseSha256.toLower();

    QString encodeError;
    const QString encoded = WeldResumePlanner::EncodeRecord(record, &encodeError);
    if (encoded.isEmpty())
    {
        error = encodeError;
        return false;
    }

    // 启动一条新的完整焊接会使旧 paused 断点失去业务意义；在 START 前用当前有效身份
    // 写入不可续焊 tombstone，避免新焊道完成后仍能误用旧断点重复焊接。
    if (!isResumeFlow)
    {
        WeldResumePlanner::CheckpointRecord superseded = record;
        superseded.state = QStringLiteral("superseded");
        const QString supersededValue = WeldResumePlanner::EncodeRecord(superseded, &encodeError);
        QString storageError;
        if (supersededValue.isEmpty()
            || !WriteBreakpointRecordValue(record.robotName, supersededValue, &storageError)
            || !DisableLegacyBreakpoint(record.robotName, &storageError))
        {
            error = storageError.isEmpty()
                ? QStringLiteral("使旧断点失效失败，已在START前中止新焊接。")
                : storageError;
            return false;
        }
    }
    if (!resumeCheckpointSupported)
    {
        ClearActiveWeldCheckpoint();
        const QString reason = resumeUnsupportedReason.isEmpty()
            ? QStringLiteral("当前焊接工艺无法绑定真实执行轨迹，暂停/断点续焊已禁用。")
            : resumeUnsupportedReason;
        AppendLog(reason + QStringLiteral(" 旧断点已失效，正常焊接继续执行。"));
        return true;
    }
    {
        QMutexLocker lock(&m_activeWeldCheckpointMutex);
        m_activeWeldCheckpointRecord = encoded;
    }
    SetWeldPauseAvailable(true);
    AppendLog(QString("已冻结可暂停焊接上下文：Case=%1 Program=%2 SHA256=%3…")
        .arg(record.caseId, record.programName, record.trajectorySha256.left(12)));
    return true;
}

QString MeasureThenWeldDialog::ActiveWeldCheckpointRecord() const
{
    QMutexLocker lock(&m_activeWeldCheckpointMutex);
    return m_activeWeldCheckpointRecord;
}

void MeasureThenWeldDialog::ClearActiveWeldCheckpoint()
{
    {
        QMutexLocker lock(&m_activeWeldCheckpointMutex);
        m_activeWeldCheckpointRecord.clear();
    }
    SetWeldPauseAvailable(false);
}

void MeasureThenWeldDialog::FinishActiveWeldExecution(bool programCompleted)
{
    const QString encoded = ActiveWeldCheckpointRecord();
    WeldResumePlanner::CheckpointRecord record;
    QString decodeError;
    if (!encoded.isEmpty() && WeldResumePlanner::DecodeRecord(encoded, record, &decodeError)
        && programCompleted
        && (record.state == QStringLiteral("paused")
            || record.state == QStringLiteral("continuing")))
    {
        QString transitionError;
        if (!TransitionBreakpointRecordState(
            record.robotName,
            record.checkpointId,
            record.state,
            QStringLiteral("finished"),
            &transitionError))
        {
            AppendLog(QString("焊接已完成，但断点终态写回失败（保持闭锁）：%1").arg(transitionError));
        }
    }
    ClearActiveWeldCheckpoint();
    if (programCompleted)
    {
        QPointer<MeasureThenWeldDialog> self(this);
        QMetaObject::invokeMethod(this, [self]()
            {
                if (self != nullptr && self->m_pRunMonitor != nullptr)
                {
                    static_cast<RunMonitorDialog*>(self->m_pRunMonitor)
                        ->ResumeWeldButton()->setEnabled(false);
                }
            }, Qt::QueuedConnection);
    }
}

void MeasureThenWeldDialog::SetWeldPauseAvailable(bool available)
{
    if (QThread::currentThread() != thread())
    {
        QPointer<MeasureThenWeldDialog> self(this);
        QMetaObject::invokeMethod(this, [self, available]()
            {
                if (self != nullptr)
                {
                    self->SetWeldPauseAvailable(available);
                }
            }, Qt::QueuedConnection);
        return;
    }
    RunMonitorDialog* monitor = static_cast<RunMonitorDialog*>(m_pRunMonitor);
    if (monitor == nullptr)
    {
        return;
    }
    monitor->PauseButton()->setEnabled(available && m_bRunning);
    if (!available)
    {
        monitor->PauseButton()->setText(QStringLiteral("暂停"));
    }
}

void MeasureThenWeldDialog::OnPauseResumeClicked()
{
    RunMonitorDialog* monitor = static_cast<RunMonitorDialog*>(m_pRunMonitor);
    RobotDriverAdaptor* pRobotDriver = GetRobotDriver();
    STEPRobotCtrl* pStepDriver = dynamic_cast<STEPRobotCtrl*>(pRobotDriver);
    if (monitor == nullptr || pStepDriver == nullptr)
    {
        QMessageBox::information(this, "暂停/继续", "暂停功能当前仅支持 STEP 机器人。");
        return;
    }

    const QString activeOwner = RobotOperationLease::CurrentOwner(pRobotDriver);
    const bool ownedByThisFlow = activeOwner == QStringLiteral("先测后焊预设流程")
        || activeOwner == QStringLiteral("跳过扫描焊接流程")
        || activeOwner == QStringLiteral("断点续焊流程");
    if (!m_bRunning || !ownedByThisFlow)
    {
        QMessageBox::warning(this, "暂停/继续",
            "当前焊接程序不再由本页面持有硬件租约，禁止发送 STOP/START。");
        SetWeldPauseAvailable(false);
        return;
    }

    WeldResumePlanner::CheckpointRecord activeRecord;
    QString recordError;
    const QString activeEncoded = ActiveWeldCheckpointRecord();
    if (!WeldResumePlanner::DecodeRecord(activeEncoded, activeRecord, &recordError)
        || (activeRecord.state != QStringLiteral("prepared")
            && activeRecord.state != QStringLiteral("paused")
            && activeRecord.state != QStringLiteral("continuing"))
        || activeRecord.robotType != QStringLiteral("STEP")
        || activeRecord.robotName.compare(CurrentRobotName(), Qt::CaseInsensitive) != 0
        || activeRecord.robotEndpoint != RobotOperationLease::PersistentEndpointIdentity(pRobotDriver))
    {
        QMessageBox::warning(this, "暂停/继续",
            recordError.isEmpty()
                ? QStringLiteral("当前没有与本次STEP焊接程序一致的可验证断点上下文，禁止暂停。")
                : recordError);
        SetWeldPauseAvailable(false);
        return;
    }

    if (monitor->PauseButton()->text() == QStringLiteral("暂停"))
    {
        std::string pausedProject;
        std::string pausedProgram;
        T_ROBOT_COORS stablePose{};
        int stableProgramLine = -1;
        if (!pStepDriver->PauseTrackedProgramAndWait(
            ToUtf8StdString(activeRecord.programName),
            stableProgramLine,
            stablePose,
            &pausedProject,
            &pausedProgram))
        {
            AppendLog(QString("暂停失败：%1").arg(DecodeRobotMessageText(pStepDriver->GetLastRobotError())));
            return;
        }
        if (QString::fromStdString(pausedProgram) != activeRecord.programName)
        {
            AppendLog(QString("暂停后程序身份变化，拒绝生成断点：Expected=%1 Actual=%2")
                .arg(activeRecord.programName, QString::fromStdString(pausedProgram)));
            monitor->PauseButton()->setText(QStringLiteral("继续"));
            monitor->ResumeWeldButton()->setEnabled(false);
            return;
        }

        m_pauseProgramLine = stableProgramLine;
        m_pausePose = stablePose;
        m_hasPausePose = true;
        activeRecord.state = QStringLiteral("paused");
        activeRecord.createdAtUtc = QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs);
        activeRecord.programLine = m_pauseProgramLine;
        activeRecord.x = m_pausePose.dX;
        activeRecord.y = m_pausePose.dY;
        activeRecord.z = m_pausePose.dZ;
        activeRecord.rx = m_pausePose.dRX;
        activeRecord.ry = m_pausePose.dRY;
        activeRecord.rz = m_pausePose.dRZ;

        QString saveError;
        const bool saved = SavePausedBreakpointRecord(activeRecord, &saveError);
        if (saved)
        {
            QString encodeError;
            const QString pausedEncoded = WeldResumePlanner::EncodeRecord(activeRecord, &encodeError);
            {
                QMutexLocker lock(&m_activeWeldCheckpointMutex);
                m_activeWeldCheckpointRecord = pausedEncoded;
            }
            monitor->ResumeWeldButton()->setEnabled(true);
        }
        else
        {
            monitor->ResumeWeldButton()->setEnabled(false);
            AppendLog(QString("机器人已暂停，但V2断点写入失败；旧断点已保持无效，禁止断点续焊：%1")
                .arg(saveError));
            ShowNonModalFlowResult(this, QMessageBox::Warning, "暂停/继续",
                QString("机器人已稳定暂停，但断点保存失败。可以点击“继续”恢复本程序；禁止关闭后使用断点续焊。\n%1")
                    .arg(saveError));
        }
        monitor->PauseButton()->setText(QStringLiteral("继续"));
        AppendLog(QString("已稳定暂停：程序=%1，行=%2，断点位姿 X=%3 Y=%4 Z=%5，V2落盘=%6。恢复前请勿移动工件。")
            .arg(activeRecord.programName)
            .arg(m_pauseProgramLine)
            .arg(m_pausePose.dX, 0, 'f', 1)
            .arg(m_pausePose.dY, 0, 'f', 1)
            .arg(m_pausePose.dZ, 0, 'f', 1)
            .arg(saved ? QStringLiteral("成功") : QStringLiteral("失败")));
        return;
    }

    // START 前先把可自动续焊的 paused 原子切为 continuing。若进程在 START 后崩溃，
    // 磁盘记录保持不可自动续焊，避免重启后从旧断点重复焊接。
    if (activeRecord.state == QStringLiteral("paused"))
    {
        QString transitionError;
        if (!TransitionBreakpointRecordState(
            activeRecord.robotName,
            activeRecord.checkpointId,
            QStringLiteral("paused"),
            QStringLiteral("continuing"),
            &transitionError))
        {
            QMessageBox::warning(this, "继续运行",
                QString("断点状态切换失败，未发送START：%1").arg(transitionError));
            return;
        }
        activeRecord.state = QStringLiteral("continuing");
        QString encodeError;
        const QString continuingEncoded = WeldResumePlanner::EncodeRecord(activeRecord, &encodeError);
        if (continuingEncoded.isEmpty())
        {
            AppendLog(QString("断点已闭锁为continuing，但内存上下文更新失败：%1").arg(encodeError));
            SetWeldPauseAvailable(false);
            return;
        }
        QMutexLocker lock(&m_activeWeldCheckpointMutex);
        m_activeWeldCheckpointRecord = continuingEncoded;
    }
    double positionDeviationMm = 0.0;
    double angleDeviationDeg = 0.0;
	if (!pStepDriver->ResumeTrackedProgramFromPause(
        ToUtf8StdString(activeRecord.programName),
        m_pausePose,
        2.0,
        2.0,
        &positionDeviationMm,
        &angleDeviationDeg))
    {
        AppendLog(QString("继续命令发送失败；断点保持continuing闭锁：%1")
            .arg(DecodeRobotMessageText(pStepDriver->GetLastRobotError())));
        return;
    }
    monitor->PauseButton()->setText("暂停");
    AppendLog(QString("已从断点继续（程序行=%1，偏差=%2 mm/%3 deg）。")
        .arg(m_pauseProgramLine)
        .arg(positionDeviationMm, 0, 'f', 3)
        .arg(angleDeviationDeg, 0, 'f', 3));
}

void MeasureThenWeldDialog::OnResumeWeldClicked()
{
    // 监控窗口里的按钮与主界面「断点续焊」同一流程；当前流程仍在运行时提示先结束。
    if (m_bRunning)
    {
        QMessageBox::information(this, "断点续焊",
            "断点已落盘。请先等待/结束当前流程，再点击主界面「断点续焊」从断点继续。");
        return;
    }
    RunResumeWeldFlow();
}

void MeasureThenWeldDialog::RunResumeWeldFlow()
{
    if (m_bRunning)
    {
        QMessageBox::information(this, "断点续焊", "流程正在运行。");
        return;
    }
    if (BlockedByOtherFlow("断点续焊"))
    {
        return;
    }
    RobotDriverAdaptor* pRobotDriver = GetRobotDriver();
    if (pRobotDriver == nullptr)
    {
        return;
    }
    const QString robotName = CurrentRobotName().trimmed();
    WeldResumePlanner::CheckpointRecord checkpointRecord;
    QString error;
    if (!ReadBreakpointRecord(robotName, checkpointRecord, nullptr, &error))
    {
        QMessageBox::warning(this, "断点续焊", error);
        return;
    }
    if (checkpointRecord.state != QStringLiteral("paused"))
    {
        QMessageBox::warning(this, "断点续焊",
            QString("断点状态为 %1，不允许自动续焊。只有完整落盘的 paused 状态可执行；resuming 状态需人工复核。")
                .arg(checkpointRecord.state));
        return;
    }

    T_PRECISE_MEASURE_PARAM param;
    if (!LoadPresetParam(pRobotDriver, param, error))
    {
        QMessageBox::warning(this, "断点续焊", error);
        return;
    }
    param.bDoActualWeld = IsActualWeldModeChecked();

    const QString currentEndpoint = RobotOperationLease::PersistentEndpointIdentity(pRobotDriver);
    const QString currentType = RobotDriverTypeName(pRobotDriver);
    if (checkpointRecord.robotName.compare(robotName, Qt::CaseInsensitive) != 0
        || checkpointRecord.robotType != currentType
        || currentType != QStringLiteral("STEP")
        || currentEndpoint.isEmpty()
        || checkpointRecord.robotEndpoint != currentEndpoint)
    {
        QMessageBox::warning(this, "断点续焊",
            QString("断点机器人身份不一致，已中止。\nRecord=%1 / %2 / %3\nCurrent=%4 / %5 / %6")
                .arg(checkpointRecord.robotName, checkpointRecord.robotType, checkpointRecord.robotEndpoint,
                    robotName, currentType, currentEndpoint));
        return;
    }
    if (checkpointRecord.paramGroupIndex != param.nParamGroupIndex
        || checkpointRecord.paramGroupName != param.sParamGroupName
        || checkpointRecord.scanSection != QString::fromStdString(param.sSectionName)
        || checkpointRecord.weldSection != QString::fromStdString(param.sWeldSectionName)
        || checkpointRecord.weldDirection != (param.nWeldDirection < 0 ? -1 : 1)
        || checkpointRecord.actualWeld != param.bDoActualWeld
        || std::abs(checkpointRecord.backtrackMm - param.dResumeBacktrackMm) > 1e-9)
    {
        QMessageBox::warning(this, "断点续焊",
            "当前参数组、焊接方向、运行模式或回退距离已与暂停时不同，已中止。请恢复暂停时配置后重试。");
        return;
    }

    QString currentFingerprint;
    double currentFinalStepMm = 0.0;
    bool resumeCheckpointSupported = false;
    QString resumeUnsupportedReason;
    if (m_pService == nullptr
        || !m_pService->ResolveWeldExecutionParameters(
            param,
            0.0,
            currentFingerprint,
            currentFinalStepMm,
            error,
            &resumeCheckpointSupported,
            &resumeUnsupportedReason))
    {
        QMessageBox::warning(this, "断点续焊", error);
        return;
    }
    if (!resumeCheckpointSupported)
    {
        QMessageBox::warning(this, "断点续焊",
            resumeUnsupportedReason.isEmpty()
                ? QStringLiteral("当前工艺无法绑定真实执行轨迹，禁止自动断点续焊。")
                : resumeUnsupportedReason);
        return;
    }
    if (currentFingerprint != checkpointRecord.parameterFingerprint
        || std::abs(currentFinalStepMm - checkpointRecord.finalStepMm) > 1e-9)
    {
        QMessageBox::warning(this, "断点续焊",
            "当前焊接工艺（电流/电压/速度/摆动/跟踪/点距）与暂停时指纹不一致，已中止。");
        return;
    }

    const QString projectRoot = RobotDataHelper::FindProjectRootPath();
    QString posePath;
    QString qualityProofSourcePosePath;
    if (!WeldResumePlanner::ResolveBoundTrajectory(
        projectRoot, robotName, checkpointRecord, posePath, &error))
    {
        QMessageBox::warning(this, "断点续焊", error);
        return;
    }
    if (!checkpointRecord.sourceTrajectoryRelativePath.isEmpty())
    {
        const QString sourceRelative = QDir::cleanPath(QDir::fromNativeSeparators(
            checkpointRecord.sourceTrajectoryRelativePath));
        if (QDir::isAbsolutePath(sourceRelative)
            || sourceRelative == QStringLiteral("..")
            || sourceRelative.startsWith(QStringLiteral("../"))
            || !sourceRelative.startsWith(
                checkpointRecord.caseRelativeDir + QLatin1Char('/'), Qt::CaseInsensitive)
            || checkpointRecord.sourceTrajectorySha256.size() != 64)
        {
            QMessageBox::warning(this, "断点续焊", "断点中的源轨迹相对路径或SHA256无效，已中止。");
            return;
        }
        const QString sourcePath = QDir(projectRoot).filePath(
            sourceRelative);
        qualityProofSourcePosePath = sourcePath;
        const QString sourceHash = WeldResumePlanner::ComputeFileSha256(sourcePath, &error);
        if (sourceHash.isEmpty() || sourceHash != checkpointRecord.sourceTrajectorySha256)
        {
            QMessageBox::warning(this, "断点续焊",
                error.isEmpty() ? QStringLiteral("暂停时源轨迹的SHA256已变化，已中止。") : error);
            return;
        }
    }
    if (qualityProofSourcePosePath.isEmpty())
    {
        QMessageBox::warning(this, "断点续焊",
            "断点没有绑定最初的 SeamComp 源轨迹，无法验证点云质量证明，已中止。");
        return;
    }

    WeldResumePlanner::ResumePlan resumePlan;
    if (!WeldResumePlanner::PlanFromPausedPoseBound(
        posePath,
        checkpointRecord,
        checkpointRecord.x,
        checkpointRecord.y,
        checkpointRecord.z,
        checkpointRecord.backtrackMm,
        resumePlan,
        &error))
    {
        QMessageBox::warning(this, "断点续焊", error);
        return;
    }
    if (!std::isfinite(resumePlan.matchDistanceMm) || resumePlan.matchDistanceMm > 20.0)
    {
        QMessageBox::warning(this, "断点续焊",
            QString("暂停位姿距绑定轨迹 %1 mm（允许上限 20 mm），已中止。")
                .arg(resumePlan.matchDistanceMm, 0, 'f', 3));
        return;
    }

    const QMessageBox::StandardButton confirm = QMessageBox::question(
        this,
        "断点续焊",
        QString("断点时间：%1\n案例：%2\n绑定轨迹：%3\nSHA256：%4\n"
                "暂停弧长：%5 / %6 mm（匹配偏差 %7 mm）\n"
                "按执行顺序回退：%8 mm → 从弧长 %9 mm 精确插值续焊。\n\n"
                "机器人将先经安全位到续焊起点，并按暂停时工艺重新起弧。确认续焊？")
            .arg(checkpointRecord.createdAtUtc)
            .arg(checkpointRecord.caseId)
            .arg(QDir::toNativeSeparators(posePath))
            .arg(checkpointRecord.trajectorySha256)
            .arg(resumePlan.pauseArcMm, 0, 'f', 3)
            .arg(resumePlan.totalArcMm, 0, 'f', 3)
            .arg(resumePlan.matchDistanceMm, 0, 'f', 3)
            .arg(resumePlan.actualBacktrackMm, 0, 'f', 3)
            .arg(resumePlan.resumeArcMm, 0, 'f', 3),
        QMessageBox::Yes | QMessageBox::No,
        QMessageBox::No);
    if (confirm != QMessageBox::Yes)
    {
        return;
    }

    if (!HasLiveSession(QStringLiteral("启动断点续焊流程")))
    {
        return;
    }
    QString leaseError;
    const auto operationLease = RobotOperationLease::TryAcquire(
        pRobotDriver, QStringLiteral("断点续焊流程"), &leaseError);
    if (!operationLease)
    {
        QMessageBox::warning(this, "断点续焊", leaseError);
        return;
    }
    QString revalidatedPosePath;
    QString revalidateError;
    if (RobotOperationLease::PersistentEndpointIdentity(pRobotDriver) != checkpointRecord.robotEndpoint
        || RobotDriverTypeName(pRobotDriver) != checkpointRecord.robotType
        || !WeldResumePlanner::ResolveBoundTrajectory(
            projectRoot, robotName, checkpointRecord, revalidatedPosePath, &revalidateError)
        || QFileInfo(revalidatedPosePath).canonicalFilePath().compare(
            QFileInfo(posePath).canonicalFilePath(), Qt::CaseInsensitive) != 0)
    {
        QMessageBox::warning(this, "断点续焊",
            revalidateError.isEmpty()
                ? QStringLiteral("取得硬件租约后机器人端点或绑定轨迹发生变化，未启动机器人。")
                : revalidateError);
        return;
    }
    QString transitionError;
    if (!TransitionBreakpointRecordState(
        robotName,
        checkpointRecord.checkpointId,
        QStringLiteral("paused"),
        QStringLiteral("resuming"),
        &transitionError))
    {
        QMessageBox::warning(this, "断点续焊",
            QString("断点状态原子切换失败，未启动机器人：%1").arg(transitionError));
        return;
    }

    m_pCameraCache = ResolveCameraCacheForUnit(m_unitIndex);
    SetRunning(true);
    ResetProgress("断点续焊：准备执行");
    AppendLog(QString("断点续焊开始：Case=%1，轨迹=%2，PauseArc=%3 mm，ResumeArc=%4 mm，回退=%5 mm。")
        .arg(checkpointRecord.caseId, posePath)
        .arg(resumePlan.pauseArcMm, 0, 'f', 3)
        .arg(resumePlan.resumeArcMm, 0, 'f', 3)
        .arg(resumePlan.actualBacktrackMm, 0, 'f', 3));

    QPointer<MeasureThenWeldDialog> self(this);
    std::thread([self, pRobotDriver, param, posePath, qualityProofSourcePosePath,
        robotName, checkpointRecord, resumePlan, operationLease]()
        {
            QString summary;
            QString execError;
            const auto validateResumeIdentity =
                [pRobotDriver, param, checkpointRecord, posePath](const MeasureThenWeldService::WeldExecutionIdentity& identity, QString& prepareError)
                {
                    if (pRobotDriver == nullptr)
                    {
                        prepareError = QStringLiteral("续焊复核失败：机器人驱动为空。");
                        return false;
                    }
                    const QString startEndpoint = RobotOperationLease::PersistentEndpointIdentity(pRobotDriver);
                    const QString startRobotType = RobotDriverTypeName(pRobotDriver);
                    const QString startRobotName = QString::fromStdString(param.sRobotName).trimmed();
                    if (startRobotName.compare(checkpointRecord.robotName, Qt::CaseInsensitive) != 0
                        || startRobotType != checkpointRecord.robotType
                        || startEndpoint != checkpointRecord.robotEndpoint)
                    {
                        prepareError = QStringLiteral("续焊运动/START前机器人名称、类型或端点身份发生变化，已中止。");
                        return false;
                    }
                    if (identity.parameterFingerprint != checkpointRecord.parameterFingerprint
                        || std::abs(identity.effectiveFinalStepMm - checkpointRecord.finalStepMm) > 1e-9)
                    {
                        prepareError = QStringLiteral("续焊运动/START前工艺指纹或最终点距发生变化，已中止。");
                        return false;
                    }
                    const QFileInfo identitySourceInfo(QDir::fromNativeSeparators(identity.sourcePosePath));
                    const QFileInfo expectedSourceInfo(QDir::fromNativeSeparators(posePath));
                    const QString identitySourcePath = QDir::cleanPath(
                        identitySourceInfo.canonicalFilePath().isEmpty()
                            ? identitySourceInfo.absoluteFilePath()
                            : identitySourceInfo.canonicalFilePath());
                    const QString expectedSourcePath = QDir::cleanPath(
                        expectedSourceInfo.canonicalFilePath().isEmpty()
                            ? expectedSourceInfo.absoluteFilePath()
                            : expectedSourceInfo.canonicalFilePath());
                    if (identitySourcePath.compare(expectedSourcePath, Qt::CaseInsensitive) != 0
                        || identity.sourcePoseSha256.compare(
                            checkpointRecord.trajectorySha256, Qt::CaseInsensitive) != 0
                        || identity.sourcePoseSize != checkpointRecord.trajectorySize)
                    {
                        prepareError = QStringLiteral(
                            "续焊运动/START前 Service 实际解析的轨迹路径、SHA256或大小与断点绑定不一致，已中止。");
                        return false;
                    }
                    return true;
                };
            bool ok = self != nullptr && self->m_pService != nullptr
                && self->m_pService->ExecuteWeldPoseFileWithSafePos(
                    pRobotDriver, posePath, param, summary, execError,
                    nullptr, nullptr,
                    [self](const QString& text) { if (self != nullptr) self->AppendLog(text); },
                    [self](const QString& text) { if (self != nullptr) self->SetFlowStep(text); },
                    [self](const QString& title, const QString& detail)
                        { return self != nullptr && self->ShowCheckpointDialog(title, detail); },
                    checkpointRecord.finalStepMm,
                    true,
                    MeasureThenWeldService::WeldPoseSource::PointCloudProduction,
                    resumePlan.resumeArcMm,
                    true,
                    [pRobotDriver, expectedEndpoint = checkpointRecord.robotEndpoint]()
                    {
                        return RobotOperationLease::IsCancellationRequested(pRobotDriver)
                            || RobotOperationLease::PersistentEndpointIdentity(pRobotDriver) != expectedEndpoint;
                    },
                    [self, pRobotDriver, param, checkpointRecord, validateResumeIdentity](const MeasureThenWeldService::WeldExecutionIdentity& identity, QString& prepareError)
                    {
                        if (!validateResumeIdentity(identity, prepareError))
                        {
                            return false;
                        }
                        return self != nullptr && self->PrepareActiveWeldCheckpoint(
                            pRobotDriver,
                            param,
                            identity.qualityProofPosePath,
                            identity.qualityProofPoseSha256,
                            identity.sampledPosePath,
                            identity.sampledPoseSha256,
                            identity.sampledPoseSize,
                            identity.programName,
                            identity.sampledPointCount,
                            identity.effectiveFinalStepMm,
                            identity.parameterFingerprint,
                            identity.resumeCheckpointSupported,
                            identity.resumeUnsupportedReason,
                            prepareError);
                    },
                    [self](bool completed)
                    {
                        if (self != nullptr)
                        {
                            self->FinishActiveWeldExecution(completed);
                        }
                    },
                    checkpointRecord.trajectorySha256,
                    validateResumeIdentity,
                    qualityProofSourcePosePath);

            if (ok)
            {
                QString finishError;
                if (!TransitionBreakpointRecordState(
                    robotName,
                    checkpointRecord.checkpointId,
                    QStringLiteral("resuming"),
                    QStringLiteral("finished"),
                    &finishError))
                {
                    WeldResumePlanner::CheckpointRecord current;
                    QString readError;
                    const bool replacedByNewCheckpoint = ReadBreakpointRecord(
                        robotName, current, nullptr, &readError)
                        && current.checkpointId != checkpointRecord.checkpointId;
                    if (!replacedByNewCheckpoint)
                    {
                        ok = false;
                        execError = QString("续焊程序已完成，但旧断点终态写回失败，保持闭锁：%1")
                            .arg(finishError);
                    }
                }
            }

            QMetaObject::invokeMethod(qApp, [self, ok, summary, execError]()
                {
                    if (self == nullptr)
                    {
                        return;
                    }
                    self->SetFlowStep(ok ? "断点续焊完成" : "断点续焊失败");
                    self->FinishProgress(ok, ok ? QStringLiteral("断点续焊完成") : QStringLiteral("断点续焊失败"));
                    self->SetRunning(false);
                    ShowNonModalFlowResult(
                        self,
                        ok ? QMessageBox::Information : QMessageBox::Warning,
                        "断点续焊",
                        ok
                            ? (summary.isEmpty() ? QStringLiteral("续焊完成。") : summary)
                            : (execError.isEmpty() ? QStringLiteral("续焊失败，请查看日志。") : execError));
                }, Qt::QueuedConnection);
        }).detach();
}

void MeasureThenWeldDialog::RunCameraTimeOffsetCalibrationFlow()
{
    if (m_bRunning)
    {
        QMessageBox::information(this, "相机时间补偿标定", "流程正在运行。");
        return;
    }
    if (BlockedByOtherFlow("相机时间补偿标定"))
    {
        return;
    }

    RobotDriverAdaptor* pRobotDriver = GetRobotDriver();
    if (pRobotDriver == nullptr)
    {
        return;
    }

    T_PRECISE_MEASURE_PARAM param;
    QString error;
    if (!LoadPresetParam(pRobotDriver, param, error))
    {
        QMessageBox::warning(this, "相机时间补偿标定", error);
        return;
    }
    param.bDoActualWeld = false;  // 标定只扫描，不进入焊接

    const double scanDistanceMm = std::sqrt(
        std::pow(param.tStartPos.dX - param.tEndPos.dX, 2.0)
        + std::pow(param.tStartPos.dY - param.tEndPos.dY, 2.0)
        + std::pow(param.tStartPos.dZ - param.tEndPos.dZ, 2.0));
    if (scanDistanceMm < 50.0)
    {
        QMessageBox::warning(this, "相机时间补偿标定",
            QString("扫描起点和终点距离过短（%1 mm < 50 mm），请先示教有效的扫描起点/终点。").arg(scanDistanceMm, 0, 'f', 1));
        return;
    }

    const QMessageBox::StandardButton confirm = QMessageBox::question(
        this,
        "相机时间补偿标定",
        QString("机器人将自动执行两次扫描（正向 + 起终点互换反向），全程无逐步确认。\n"
            "参数组：%1\n扫描距离≈%2 mm，扫描速度=%3 mm/min，当前补偿=%4 ms。\n\n"
            "工件要求：沿扫描方向有明显起伏特征（如波纹板，拐点≥6 个）；平板/无特征直缝无法标定。\n"
            "正向扫描后会自动预检拐点数量，不足会立即中止。\n\n"
            "请确认：工件与夹持在两次扫描期间不会移动、扫描路径无障碍。开始标定？")
            .arg(param.sParamGroupName)
            .arg(scanDistanceMm, 0, 'f', 0)
            .arg(param.dScanSpeed, 0, 'f', 0)
            .arg(param.dCameraTimeOffsetMs, 0, 'f', 1),
        QMessageBox::Yes | QMessageBox::No,
        QMessageBox::No);
    if (confirm != QMessageBox::Yes)
    {
        return;
    }

    if (!HasLiveSession(QStringLiteral("启动相机时间补偿标定")))
    {
        return;
    }
    QString leaseError;
    const auto operationLease = RobotOperationLease::TryAcquire(
        pRobotDriver, QStringLiteral("相机时间补偿标定"), &leaseError);
    if (!operationLease)
    {
        QMessageBox::warning(this, "相机时间补偿标定", leaseError);
        return;
    }

    const int unitIndexForRun = m_unitIndex;
    // 上一轮相机 runtime 可能已经被主窗口重建；SetRunning(true) 会立即访问该成员，
    // 因此先清掉旧指针，待本轮启动成功后再在 UI 线程解析并赋回新缓存。
    m_pCameraCache = nullptr;
    SetRunning(true);
    ResetProgress("相机时间补偿标定：准备启动相机");
    SetProgress(5, "标定开始");
    AppendLog(QString("相机时间补偿标定开始：参数组=%1 [%2]，扫描速度=%3 mm/min，当前补偿=%4 ms。")
        .arg(param.sParamGroupName)
        .arg(QString::fromStdString(param.sSectionName))
        .arg(param.dScanSpeed, 0, 'f', 1)
        .arg(param.dCameraTimeOffsetMs, 0, 'f', 1));

    QPointer<MeasureThenWeldDialog> self(this);
    std::thread([self, pRobotDriver, param, unitIndexForRun, operationLease]()
        {
            bool ok = true;
            QString message;
            QString cameraIP;
            QString savedForward;
            QString savedReverse;
            CameraFrameCache* cameraCacheForRun = nullptr;
            TimeOffsetCalibrationResult calib;

            QMetaObject::invokeMethod(qApp, [self, &cameraIP, &ok, &cameraCacheForRun, unitIndexForRun]()
                {
                    if (self == nullptr)
                    {
                        ok = false;
                        return;
                    }
                    ok = self->m_startCamera ? self->m_startCamera(unitIndexForRun, cameraIP) : false;
                    if (ok)
                    {
                        // 启动相机可能重建当前单元的 runtime；必须在启动成功后重新取得缓存，
                        // 禁止沿用启动前的旧指针。
                        cameraCacheForRun = self->ResolveCameraCacheForUnit(unitIndexForRun);
                        self->m_pCameraCache = cameraCacheForRun;
                        ok = cameraCacheForRun != nullptr;
                    }
                }, Qt::BlockingQueuedConnection);
            if (!ok)
            {
                message = "相机启动失败，标定中止。";
            }

            auto runCalibrationScan = [self, pRobotDriver, cameraCacheForRun](
                const T_PRECISE_MEASURE_PARAM& scanParam,
                const QString& phaseName,
                int progressBase,
                QString& savedPath,
                QString& scanError) -> bool
                {
                    if (self == nullptr || self->m_pService == nullptr || cameraCacheForRun == nullptr)
                    {
                        scanError = QStringLiteral("标定扫描运行环境已失效。");
                        return false;
                    }

                    MeasureThenWeldService::ScanCycleResult scanCycle;
                    const bool scanOk = self->m_pService->RunScanCycle(
                        pRobotDriver,
                        scanParam,
                        SafeSpeed(scanParam.dRunSpeed, 1.0),
                        cameraCacheForRun,
                        scanCycle,
                        [self, phaseName](const QString& text)
                        {
                            if (self != nullptr)
                            {
                                self->AppendLog(QString("%1：%2").arg(phaseName, text));
                            }
                        },
                        [self, phaseName, progressBase](const QString& text)
                        {
                            if (self != nullptr)
                            {
                                self->SetFlowStep(QString("%1：%2").arg(phaseName, text));
                                self->SetProgressBusy(progressBase, text);
                            }
                        },
                        [self](const QString& title, const QString& detail) -> bool
                        {
                            return self != nullptr && self->ShowCheckpointDialog(title, detail);
                        });
                    // 标定只需要 KeyPoints；部分合法处理路径只返回案例目录而不生成最终姿态文件。
                    // 保留旧 ScanMoveAndCollect 的“文件或案例目录”兼容语义。
                    savedPath = !scanCycle.weldPosePath.isEmpty()
                        ? scanCycle.weldPosePath
                        : scanCycle.caseDir;
                    if (!scanOk)
                    {
                        scanError = scanCycle.error.isEmpty()
                            ? QStringLiteral("扫描循环失败，请查看日志。")
                            : scanCycle.error;
                    }
                    return scanOk;
                };

            // ---- 第一次：正向扫描 ----
            if (ok)
            {
                if (self != nullptr)
                {
                    self->SetFlowStep("标定 1/2：正向扫描");
                    self->SetProgressBusy(15, "正向：移动到下枪安全位置");
                }
                QString scanError;
                ok = runCalibrationScan(param, QStringLiteral("标定 1/2 正向"), 30, savedForward, scanError);
                if (!ok && message.isEmpty())
                {
                    message = QString("正向扫描失败，标定中止：%1").arg(scanError);
                }
                // 工件适配性预检：正向拐点太少说明工件没有足够特征，立即中止、不再白跑反向扫描。
                if (ok)
                {
                    const double dx = std::abs(param.tStartPos.dX - param.tEndPos.dX);
                    const double dy = std::abs(param.tStartPos.dY - param.tEndPos.dY);
                    const double dz = std::abs(param.tStartPos.dZ - param.tEndPos.dZ);
                    const int axisIndex = (dx >= dy && dx >= dz) ? 0 : (dy >= dz ? 1 : 2);
                    QString precheckError;
                    const std::vector<double> forwardCorners =
                        ReadCornerAxisCoords(ResolveCalibKeyPointsPath(savedForward), axisIndex, QString(), &precheckError);
                    if (static_cast<int>(forwardCorners.size()) < 6)
                    {
                        ok = false;
                        message = QString("正向扫描仅识别到 %1 个拐点（需≥6），当前工件不适合时间补偿标定——"
                            "请改用沿扫描方向有明显起伏特征的工件（如波纹板）。已中止，未执行反向扫描。")
                            .arg(static_cast<int>(forwardCorners.size()));
                        if (self != nullptr)
                        {
                            self->AppendLog(message);
                        }
                    }
                    else if (self != nullptr)
                    {
                        self->AppendLog(QString("标定预检通过：正向扫描识别到 %1 个拐点。")
                            .arg(static_cast<int>(forwardCorners.size())));
                    }
                }
            }

            // ---- 第二次：起终点互换反向扫描 ----
            if (ok)
            {
                T_PRECISE_MEASURE_PARAM paramReverse = param;
                std::swap(paramReverse.tStartPos, paramReverse.tEndPos);
                // 示教安全位模式(bUseComputedScanSafe=false)：start 列表是[远→近]接近序列、end 列表是[近→远]撤离序列，
                // 互换后语义互逆，必须各自倒序——否则反向段会按错序脉冲走出示教之外的直线路径。推算模式不受影响。
                std::swap(paramReverse.vtStartSafePulse, paramReverse.vtEndSafePulse);
                std::reverse(paramReverse.vtStartSafePulse.begin(), paramReverse.vtStartSafePulse.end());
                std::reverse(paramReverse.vtEndSafePulse.begin(), paramReverse.vtEndSafePulse.end());
                paramReverse.bHasStartPulse = false;  // 起点侧手腕参考不再适用，跳过该项检查

                if (self != nullptr)
                {
                    self->SetFlowStep("标定 2/2：反向扫描（起终点互换）");
                    self->SetProgressBusy(50, "反向：移动到下枪安全位置");
                }
                QString scanError;
                ok = runCalibrationScan(paramReverse, QStringLiteral("标定 2/2 反向"), 64, savedReverse, scanError);
                if (!ok && message.isEmpty())
                {
                    message = QString("反向扫描失败，标定中止：%1").arg(scanError);
                }
            }

            // ---- 解算 ----
            if (ok)
            {
                if (self != nullptr)
                {
                    self->SetProgressBusy(92, "解算时间补偿");
                }
                calib = ComputeTimeOffsetCalibration(savedForward, savedReverse, param);
                if (self != nullptr)
                {
                    const char axisNames[3] = { 'X', 'Y', 'Z' };
                    self->AppendLog(QString("标定解算：扫描方向轴=%1，拐点配对=%2，正-反分裂=%3 mm（MAD=%4 mm），固有延迟=%5 ms，建议补偿=%6 ms（当前=%7 ms）。%8")
                        .arg(QChar(axisNames[calib.axisIndex]))
                        .arg(calib.pairCount)
                        .arg(calib.splitMm, 0, 'f', 3)
                        .arg(calib.madMm, 0, 'f', 3)
                        .arg(calib.delta0Ms, 0, 'f', 2)
                        .arg(calib.suggestedOffsetMs, 0, 'f', 2)
                        .arg(param.dCameraTimeOffsetMs, 0, 'f', 2)
                        .arg(calib.valid ? QString() : QString("未通过质量门禁：%1").arg(calib.error)));
                }
                if (!calib.valid)
                {
                    ok = false;
                    message = QString("标定质量不达标：%1").arg(calib.error);
                }
            }

            // ---- 主线程：结果确认 + 写入全部参数组 ----
            QMetaObject::invokeMethod(qApp, [self, ok, message, calib, param]()
                {
                    if (self == nullptr)
                    {
                        return;
                    }
                    if (self->m_stopCamera)
                    {
                        self->m_stopCamera();
                    }
                    bool finished = ok;
                    QString finishText = message;
                    if (ok)
                    {
                        const QMessageBox::StandardButton apply = QMessageBox::question(
                            self,
                            "相机时间补偿标定",
                            QString("标定完成。\n\n正-反向拐点分裂：%1 mm（配对 %2 个，MAD %3 mm）\n"
                                "解算固有延迟：%4 ms\n当前补偿：%5 ms\n建议补偿：%6 ms\n\n"
                                "是否将建议值写入该机器人【全部】测量参数组的「相机时间补偿(ms)」？")
                                .arg(calib.splitMm, 0, 'f', 3)
                                .arg(calib.pairCount)
                                .arg(calib.madMm, 0, 'f', 3)
                                .arg(calib.delta0Ms, 0, 'f', 2)
                                .arg(param.dCameraTimeOffsetMs, 0, 'f', 2)
                                .arg(calib.suggestedOffsetMs, 0, 'f', 2),
                            QMessageBox::Yes | QMessageBox::No,
                            QMessageBox::Yes);
                        if (apply == QMessageBox::Yes)
                        {
                            int groupsWritten = 0;
                            QString writeError;
                            if (WriteCameraTimeOffsetToAllScanGroups(
                                QString::fromStdString(param.sRobotName), calib.suggestedOffsetMs, &groupsWritten, &writeError))
                            {
                                finishText = QString("标定完成：补偿 %1 ms 已写入 %2 个参数组。建议再正反各扫一次验证（分裂应 <0.2 mm）。")
                                    .arg(calib.suggestedOffsetMs, 0, 'f', 2)
                                    .arg(groupsWritten);
                                self->AppendLog(finishText);
                            }
                            else
                            {
                                finished = false;
                                finishText = QString("补偿写入失败：%1").arg(writeError);
                                self->AppendLog(finishText);
                            }
                        }
                        else
                        {
                            finishText = QString("标定完成，未写入（建议值 %1 ms 已记录在流程日志）。")
                                .arg(calib.suggestedOffsetMs, 0, 'f', 2);
                        }
                    }
                    self->SetFlowStep(finished ? "标定完成" : "标定失败，请查看流程日志");
                    self->FinishProgress(finished, finished ? QStringLiteral("标定完成") : QStringLiteral("标定失败"));
                    self->SetRunning(false);
                    ShowNonModalFlowResult(
                        self,
                        finished ? QMessageBox::Information : QMessageBox::Warning,
                        "相机时间补偿标定",
                        finishText);
                }, Qt::QueuedConnection);
        }).detach();
}

void MeasureThenWeldDialog::RefreshWeldModeFromParam()
{
    if (m_pActualWeldCheck == nullptr || m_pService == nullptr)
    {
        return;
    }

    RobotDriverAdaptor* pRobotDriver = GetRobotDriver();
    if (pRobotDriver == nullptr)
    {
        return;
    }

    T_PRECISE_MEASURE_PARAM param;
    QString error;
    if (!m_pService->LoadPresetParam(pRobotDriver, param, error))
    {
        return;
    }

    if (m_pActualWeldCheck != nullptr)
    {
        const QSignalBlocker blocker(m_pActualWeldCheck);
        m_pActualWeldCheck->setChecked(param.bDoActualWeld);
        m_pActualWeldCheck->setToolTip(QString("勾选后按焊接速度运行：%1 mm/min\n取消勾选后按空跑速度运行：%2 mm/min\n下枪/收枪安全位置速度：%3 mm/min")
            .arg(param.dWeldSpeedMmPerMin, 0, 'f', 3)
            .arg(param.dDryRunSpeedMmPerMin, 0, 'f', 3)
            .arg(param.dWeldSafeMoveSpeedMmPerMin, 0, 'f', 3));
    }
}

void MeasureThenWeldDialog::SaveWeldModeToParam(bool doActualWeld)
{
    if (m_bRunning || m_pService == nullptr)
    {
        return;
    }

    RobotDriverAdaptor* pRobotDriver = GetRobotDriver();
    if (pRobotDriver == nullptr)
    {
        return;
    }

    T_PRECISE_MEASURE_PARAM param;
    QString error;
    if (!m_pService->LoadPresetParam(pRobotDriver, param, error))
    {
        AppendLog("保存焊接/空跑模式失败：" + error);
        return;
    }

    const QString paramPath = QString::fromStdString(param.sWeldParamFilePath.empty()
        ? param.sIniFilePath
        : param.sWeldParamFilePath);
    if (!RobotDataHelper::WriteParamValue(
        paramPath,
        QString::fromStdString(param.sWeldSectionName),
        "WeldEnable",
        doActualWeld ? "1" : "0",
        &error))
    {
        AppendLog("保存焊接/空跑模式失败：" + error);
        return;
    }

    AppendLog(QString("运行模式已切换为：%1").arg(doActualWeld ? QStringLiteral("实际焊接") : QStringLiteral("空跑")));
}


bool MeasureThenWeldDialog::IsActualWeldModeChecked() const
{
    return m_pActualWeldCheck == nullptr || m_pActualWeldCheck->isChecked();
}

void MeasureThenWeldDialog::AppendLog(const QString& text)
{
    if (QThread::currentThread() != thread())
    {
        QPointer<MeasureThenWeldDialog> self(this);
        QMetaObject::invokeMethod(qApp, [self, text]()
            {
                if (self != nullptr)
                {
                    self->AppendLog(text);
                }
            }, Qt::QueuedConnection);
        return;
    }

    if (m_pRunMonitor != nullptr)
    {
        static_cast<RunMonitorDialog*>(m_pRunMonitor)->AppendLogLine(text);
    }
    if (m_pLogText != nullptr)
    {
        m_pLogText->appendPlainText(QString("[%1] %2").arg(QDateTime::currentDateTime().toString("HH:mm:ss.zzz")).arg(text));
    }

    // 同步把界面扫描日志保存到文件（统一走 RobotLog：按天归档 Log/<日期>/、线程安全、自动毫秒时间戳），
    // 便于扫描后离线分析耗时/写盘速率、排查焊缝文件未生成等问题。
    static RobotLog fileLogger("Log/MeasureThenWeldLog.txt", false);
    fileLogger.writeLine(text.toStdString());
}

void MeasureThenWeldDialog::SetFlowStep(const QString& text)
{
    if (QThread::currentThread() != thread())
    {
        QPointer<MeasureThenWeldDialog> self(this);
        QMetaObject::invokeMethod(qApp, [self, text]()
            {
                if (self != nullptr)
                {
                    self->SetFlowStep(text);
                }
            }, Qt::QueuedConnection);
        return;
    }

    if (m_pRunMonitor != nullptr)
    {
        static_cast<RunMonitorDialog*>(m_pRunMonitor)->SetStep(text);
    }
    emit FlowStepChanged(QString("先测后焊流程进行中，目前：%1").arg(text));
}

void MeasureThenWeldDialog::ResetProgress(const QString& text)
{
    if (QThread::currentThread() != thread())
    {
        QPointer<MeasureThenWeldDialog> self(this);
        QMetaObject::invokeMethod(qApp, [self, text]()
            {
                if (self != nullptr)
                {
                    self->ResetProgress(text);
                }
            }, Qt::QueuedConnection);
        return;
    }

    m_bProgressBusy = false;
    m_nProgressValue = 0;
    m_sProgressText = text.trimmed().isEmpty() ? QStringLiteral("准备开始") : text.trimmed();
    if (m_pProgressAnimationTimer != nullptr)
    {
        m_pProgressAnimationTimer->stop();
    }
    if (m_pProgressLabel != nullptr)
    {
        m_pProgressLabel->setText(m_sProgressText);
        m_pProgressLabel->show();
    }
    if (m_pProgressBar != nullptr)
    {
        m_pProgressBar->setRange(0, 100);
        m_pProgressBar->setValue(0);
        m_pProgressBar->show();
    }
}

void MeasureThenWeldDialog::SetProgress(int value, const QString& text)
{
    if (QThread::currentThread() != thread())
    {
        QPointer<MeasureThenWeldDialog> self(this);
        QMetaObject::invokeMethod(qApp, [self, value, text]()
            {
                if (self != nullptr)
                {
                    self->SetProgress(value, text);
                }
            }, Qt::QueuedConnection);
        return;
    }

    if (m_pRunMonitor != nullptr)
    {
        static_cast<RunMonitorDialog*>(m_pRunMonitor)->SetProgressValue(value);
    }
    m_bProgressBusy = false;
    m_nProgressValue = std::clamp(value, 0, 100);
    m_sProgressText = text.trimmed();
    if (m_pProgressAnimationTimer != nullptr)
    {
        m_pProgressAnimationTimer->stop();
    }
    if (m_pProgressLabel != nullptr)
    {
        m_pProgressLabel->setText(m_sProgressText.isEmpty() ? QStringLiteral("处理中") : m_sProgressText);
        m_pProgressLabel->show();
    }
    if (m_pProgressBar != nullptr)
    {
        m_pProgressBar->setRange(0, 100);
        m_pProgressBar->setValue(m_nProgressValue);
        m_pProgressBar->show();
    }
}

void MeasureThenWeldDialog::SetProgressBusy(int baseValue, const QString& text)
{
    if (QThread::currentThread() != thread())
    {
        QPointer<MeasureThenWeldDialog> self(this);
        QMetaObject::invokeMethod(qApp, [self, baseValue, text]()
            {
                if (self != nullptr)
                {
                    self->SetProgressBusy(baseValue, text);
                }
            }, Qt::QueuedConnection);
        return;
    }

    m_bProgressBusy = true;
    m_nProgressValue = (std::max)(m_nProgressValue, std::clamp(baseValue, 0, 96));
    m_sProgressText = text.trimmed().isEmpty() ? QStringLiteral("处理中") : text.trimmed();
    if (m_pProgressLabel != nullptr)
    {
        m_pProgressLabel->setText(m_sProgressText);
        m_pProgressLabel->show();
    }
    if (m_pProgressBar != nullptr)
    {
        m_pProgressBar->setRange(0, 100);
        m_pProgressBar->setValue(m_nProgressValue);
        m_pProgressBar->show();
    }
    if (m_pProgressAnimationTimer != nullptr && !m_pProgressAnimationTimer->isActive())
    {
        m_pProgressAnimationTimer->start();
    }
}

void MeasureThenWeldDialog::FinishProgress(bool ok, const QString& text)
{
    if (QThread::currentThread() != thread())
    {
        QPointer<MeasureThenWeldDialog> self(this);
        QMetaObject::invokeMethod(qApp, [self, ok, text]()
            {
                if (self != nullptr)
                {
                    self->FinishProgress(ok, text);
                }
            }, Qt::QueuedConnection);
        return;
    }

    m_bProgressBusy = false;
    m_nProgressValue = ok ? 100 : (std::max)(m_nProgressValue, 1);
    m_sProgressText = text.trimmed().isEmpty()
        ? (ok ? QStringLiteral("流程完成") : QStringLiteral("流程失败"))
        : text.trimmed();
    if (m_pProgressAnimationTimer != nullptr)
    {
        m_pProgressAnimationTimer->stop();
    }
    if (m_pProgressLabel != nullptr)
    {
        m_pProgressLabel->setText(m_sProgressText);
        m_pProgressLabel->show();
    }
    if (m_pProgressBar != nullptr)
    {
        m_pProgressBar->setRange(0, 100);
        m_pProgressBar->setValue(m_nProgressValue);
        m_pProgressBar->show();
    }
}

void MeasureThenWeldDialog::UpdateProgressAnimation()
{
    if (!m_bProgressBusy)
    {
        if (m_pProgressAnimationTimer != nullptr)
        {
            m_pProgressAnimationTimer->stop();
        }
        return;
    }

    m_nProgressValue = (std::min)(95, m_nProgressValue + 1);
    static int dotTick = 0;
    dotTick = (dotTick + 1) % 4;
    if (m_pProgressLabel != nullptr)
    {
        m_pProgressLabel->setText(QString("%1%2").arg(m_sProgressText, QString(dotTick, QLatin1Char('.'))));
    }
    if (m_pProgressBar != nullptr)
    {
        m_pProgressBar->setValue(m_nProgressValue);
    }
}

void MeasureThenWeldDialog::SetRunning(bool running)
{
    if (running)
    {
        ClearActiveWeldCheckpoint();
        if (m_pRunMonitor != nullptr)
        {
            static_cast<RunMonitorDialog*>(m_pRunMonitor)->ResumeWeldButton()->setEnabled(false);
        }
    }
    else
    {
        SetWeldPauseAvailable(false);
    }
    m_bRunning = running;
    m_pPresetParamBtn->setEnabled(!running);
    m_pSkipScanWeldBtn->setEnabled(!running);
    m_pLineScanProcessBtn->setEnabled(!running);
    if (m_pTimeOffsetCalibBtn != nullptr)
    {
        m_pTimeOffsetCalibBtn->setEnabled(!running);
    }
    // 实时显示数据泵随流程启停；开启时让 SKJ worker 同步取相机图像，并弹出运行监控最大化窗口。
    if (m_pLiveViewTimer != nullptr)
    {
        if (running)
        {
            if (m_pCameraCache != nullptr)
            {
                m_pCameraCache->SetLiveImageEnabled(true);
            }
            m_pLiveViewTimer->start();
            if (m_pRunMonitor != nullptr)
            {
                static_cast<RunMonitorDialog*>(m_pRunMonitor)->PauseButton()->setText("暂停");
                m_pRunMonitor->showMaximized();
            }
        }
        else
        {
            m_pLiveViewTimer->stop();
            if (m_pCameraCache != nullptr)
            {
                m_pCameraCache->SetLiveImageEnabled(false);
            }
        }
    }
    if (m_pActualWeldCheck != nullptr)
    {
        m_pActualWeldCheck->setEnabled(!running);
    }
    if (m_pRobotCombo != nullptr)
    {
        m_pRobotCombo->setEnabled(!running);
    }
    if (m_pParamGroupCombo != nullptr)
    {
        m_pParamGroupCombo->setEnabled(!running);
    }
    if (m_pWeldProcessCombo != nullptr)
    {
        m_pWeldProcessCombo->setEnabled(!running && m_pWeldProcessCombo->count() > 0 && m_pWeldProcessCombo->currentData().toInt() >= 0);
    }
    if (m_pPoseCompGroupCombo != nullptr)
    {
        m_pPoseCompGroupCombo->setEnabled(!running && m_pPoseCompGroupCombo->count() > 0);
    }
    if (m_pSeamCompGroupCombo != nullptr)
    {
        m_pSeamCompGroupCombo->setEnabled(!running && m_pSeamCompGroupCombo->count() > 0);
    }
}
