#include "groove/scancameraskjworker.h"

#include "CameraFrameCache.h"
#include "RobotLog.h"
#include "groove/framebuffer.h"

#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QFileInfo>
#include <QLibrary>
#include <QTimer>

#include <limits>

namespace
{
// 与 protocol.h 的 SKJ_* 返回码一致（不直接 include SDK 头，避免 dllimport 声明干扰动态加载）。
constexpr int kSkjOk = 0;
constexpr int kSkjErrNoData = -106;        // 暂无新帧
constexpr int kSkjErrDataDuplicate = -107; // 最新帧已取过

// 轮询间隔：相机约 60fps（~16ms 一帧），10ms 轮询可跟上，无新帧时返回 DUPLICATE/NO_DATA 属正常。
constexpr int kPollIntervalMs = 10;
constexpr int kConnectTimeoutMs = 3000;
constexpr int kDiagnosticEmitStride = 30;

// 与 SDK protocol.h 的 Point3d/Point2d 二进制布局一致（均为 double 字段）。
struct SkjPoint3d
{
    double x;
    double y;
    double z;
};

struct SkjPoint2d
{
    double x;
    double y;
};

void WriteCameraSkjLog(const QString& text)
{
    // 统一走 RobotLog：按天归档 Log/<日期>/、自带写入锁、自动毫秒时间戳。
    static RobotLog logger("Log/CameraSkjClient.txt", false);
    logger.writeLine(text.toStdString());
}
}

ScanCameraSkjWorker::ScanCameraSkjWorker(CameraFrameCache* frameCache, QObject* parent)
    : QObject(parent)
    , m_frameCache(frameCache)
{
}

ScanCameraSkjWorker::~ScanCameraSkjWorker()
{
    stopClient();
    if (m_library != nullptr)
    {
        m_library->unload();
        delete m_library;
        m_library = nullptr;
    }
}

QString ScanCameraSkjWorker::resolveDllPath() const
{
    const QString candidate = QDir(QCoreApplication::applicationDirPath()).filePath("SKJCamera.dll");
    if (QFileInfo::exists(candidate))
    {
        return candidate;
    }
    // 退回让 QLibrary 按系统搜索路径定位（含工作目录）。
    return QStringLiteral("SKJCamera");
}

void* ScanCameraSkjWorker::resolve(const char* name)
{
    if (m_library == nullptr)
    {
        return nullptr;
    }
    return reinterpret_cast<void*>(m_library->resolve(name));
}

bool ScanCameraSkjWorker::loadSdk(QString* error)
{
    if (m_library != nullptr && m_library->isLoaded() && m_getLatestFrame != nullptr)
    {
        return true;
    }
    if (m_library == nullptr)
    {
        m_library = new QLibrary(resolveDllPath(), this);
    }
    if (!m_library->load())
    {
        if (error != nullptr)
        {
            *error = QString("加载 SKJCamera.dll 失败：%1\n路径：%2")
                .arg(m_library->errorString(), resolveDllPath());
        }
        return false;
    }

    m_create = reinterpret_cast<CreateFn>(resolve("SKJCamera_Create"));
    m_destroy = reinterpret_cast<DestroyFn>(resolve("SKJCamera_Destroy"));
    m_connect = reinterpret_cast<ConnectFn>(resolve("SKJCamera_Connect"));
    m_disconnect = reinterpret_cast<DisconnectFn>(resolve("SKJCamera_Disconnect"));
    m_isConnected = reinterpret_cast<IsConnectedFn>(resolve("SKJCamera_IsConnected"));
    m_setConnectTimeout = reinterpret_cast<SetTimeoutFn>(resolve("SKJCamera_SetConnectTimeout"));
    m_getLatestFrame = reinterpret_cast<GetLatestFrameFn>(resolve("SKJCamera_GetLatestFrame"));
    m_frameRelease = reinterpret_cast<FrameReleaseFn>(resolve("SKJFrame_Release"));
    m_frameTimestamp = reinterpret_cast<FrameTimestampFn>(resolve("SKJFrame_GetTimestamp"));
    m_framePoint3DCount = reinterpret_cast<FrameIntFn>(resolve("SKJFrame_GetPoint3DCount"));
    m_framePoint3DData = reinterpret_cast<FramePtrFn>(resolve("SKJFrame_GetPoint3DData"));
    m_framePoint2DCount = reinterpret_cast<FrameIntFn>(resolve("SKJFrame_GetPoint2DCount"));
    m_framePoint2DData = reinterpret_cast<FramePtrFn>(resolve("SKJFrame_GetPoint2DData"));
    m_frameResultPoint = reinterpret_cast<FrameResultFn>(resolve("SKJFrame_GetResultPointValue"));
    m_frameFpsPointCloud = reinterpret_cast<FrameFloatFn>(resolve("SKJFrame_GetFpsPointCloud"));
    m_frameErrorText = reinterpret_cast<FrameTextFn>(resolve("SKJFrame_GetErrorText"));
    m_errorString = reinterpret_cast<ErrorStringFn>(resolve("SKJCamera_GetErrorString"));

    if (m_create == nullptr || m_destroy == nullptr || m_connect == nullptr
        || m_disconnect == nullptr || m_getLatestFrame == nullptr || m_frameRelease == nullptr
        || m_framePoint3DCount == nullptr || m_framePoint3DData == nullptr)
    {
        if (error != nullptr)
        {
            *error = "SKJCamera.dll 缺少点云数据接口导出函数（需新版含 SKJCamera_GetLatestFrame）。";
        }
        return false;
    }
    return true;
}

void ScanCameraSkjWorker::startClient(const QString& serverIP, int serverPort, int pollIntervalMs)
{
    m_serverIP = serverIP.trimmed();
    m_serverPort = serverPort;
    m_pollIntervalMs = (pollIntervalMs > 0) ? pollIntervalMs : kPollIntervalMs;  // 无效则回退默认
    m_running = true;
    m_loggedFirstFrame = false;
    m_pollCount = 0;
    m_noNewFrameCount = 0;
    m_decodedFrameCount = 0;
    m_decodeFailedCount = 0;
    m_appendedFrameCount = 0;

    QString error;
    if (!loadSdk(&error))
    {
        WriteCameraSkjLog(QString("loadSdk failed: %1").arg(error));
        emitDiagnostic(QString("SKJ SDK 加载失败：%1").arg(error));
        return;
    }

    teardownHandle();
    m_handle = (m_create != nullptr) ? m_create() : nullptr;
    if (m_handle == nullptr)
    {
        WriteCameraSkjLog("SKJCamera_Create returned null");
        emitDiagnostic("创建 SKJCamera 句柄失败。");
        return;
    }
    if (m_setConnectTimeout != nullptr)
    {
        m_setConnectTimeout(m_handle, kConnectTimeoutMs);
    }

    const QByteArray ipBytes = m_serverIP.toLocal8Bit();
    const int ret = m_connect(m_handle, ipBytes.constData(), m_serverPort);
    if (ret != kSkjOk)
    {
        const QString reason = (m_errorString != nullptr)
            ? QString::fromUtf8(m_errorString(ret))
            : QString::number(ret);
        WriteCameraSkjLog(QString("connect failed target=%1:%2 ret=%3 (%4)")
            .arg(m_serverIP).arg(m_serverPort).arg(ret).arg(reason));
        emitDiagnostic(QString("连接相机失败：%1").arg(reason));
        teardownHandle();
        return;
    }
    WriteCameraSkjLog(QString("connected target=%1:%2").arg(m_serverIP).arg(m_serverPort));

    if (m_pollTimer == nullptr)
    {
        m_pollTimer = new QTimer(this);
        m_pollTimer->setTimerType(Qt::PreciseTimer);
        connect(m_pollTimer, &QTimer::timeout, this, &ScanCameraSkjWorker::pollFrame);
    }
    m_pollTimer->start(m_pollIntervalMs);
    emitDiagnostic(QString("SKJ SDK 已连接 %1:%2，开始取帧（轮询间隔 %3 ms）。").arg(m_serverIP).arg(m_serverPort).arg(m_pollIntervalMs));
}

void ScanCameraSkjWorker::pollFrame()
{
    if (!m_running || m_handle == nullptr || m_getLatestFrame == nullptr)
    {
        return;
    }

    ++m_pollCount;
    void* frame = nullptr;
    const int ret = m_getLatestFrame(m_handle, &frame);

    // 逐帧记录 SDK 取帧状态（含 0=取到、-106 无新帧、-107 重复、其它错误），供扫描结束后做
    // 数据完整性判定（长时间无新帧→终止流程）和写入匹配明细文件。
    if (m_frameCache != nullptr)
    {
        m_frameCache->RecordPollStatus(ret);
    }

    if (ret == kSkjErrNoData || ret == kSkjErrDataDuplicate)
    {
        ++m_noNewFrameCount;
        if (m_pollCount % kDiagnosticEmitStride == 0)
        {
            emitDiagnostic("SKJ SDK 收帧中（等待新帧）。");
        }
        return;
    }
    if (ret != kSkjOk || frame == nullptr)
    {
        ++m_decodeFailedCount;
        if (frame != nullptr && m_frameRelease != nullptr)
        {
            m_frameRelease(frame);
        }
        const QString reason = (m_errorString != nullptr)
            ? QString::fromUtf8(m_errorString(ret))
            : QString::number(ret);
        emitDiagnostic(QString("取帧失败：%1").arg(reason));
        return;
    }

    udpDataShow data;
    const int count3d = (m_framePoint3DCount != nullptr) ? m_framePoint3DCount(frame) : 0;
    const SkjPoint3d* points3d = static_cast<const SkjPoint3d*>(
        (m_framePoint3DData != nullptr) ? m_framePoint3DData(frame) : nullptr);
    if (count3d > 0 && points3d != nullptr)
    {
        data.XData.reserve(count3d);
        data.YData.reserve(count3d);
        data.allResultPoint.reserve(count3d);
        for (int i = 0; i < count3d; ++i)
        {
            // 与旧 BuildUdpFrame 一致：绘图取 (y, z)；allResultPoint 保留原始三维点。
            data.XData.append(points3d[i].y);
            data.YData.append(points3d[i].z);
            data.allResultPoint.push_back(cv::Point3d(points3d[i].x, points3d[i].y, points3d[i].z));
        }
    }

    const int count2d = (m_framePoint2DCount != nullptr) ? m_framePoint2DCount(frame) : 0;
    const SkjPoint2d* points2d = static_cast<const SkjPoint2d*>(
        (m_framePoint2DData != nullptr) ? m_framePoint2DData(frame) : nullptr);
    if (count2d > 0 && points2d != nullptr)
    {
        data.fitLineX.reserve(count2d);
        data.fitLineY.reserve(count2d);
        for (int i = 0; i < count2d; ++i)
        {
            data.fitLineX.append(points2d[i].x);
            data.fitLineY.append(points2d[i].y);
        }
    }

    cv::Point3d targetPoint(0.0, 0.0, 0.0);
    if (m_frameResultPoint != nullptr)
    {
        SkjPoint3d result{0.0, 0.0, 0.0};
        if (m_frameResultPoint(frame, &result) == kSkjOk)
        {
            targetPoint = cv::Point3d(result.x, result.y, result.z);
        }
    }
    // 与 TCP 链路一致：未识别到目标(0,0,0)替换为 NaN，避免 0 值伪目标点混入下游。
    if (targetPoint == cv::Point3d(0.0, 0.0, 0.0))
    {
        const double nan = std::numeric_limits<double>::quiet_NaN();
        targetPoint = cv::Point3d(nan, nan, nan);
    }
    data.targetPoint = targetPoint;
    data.targetX.append(targetPoint.y);
    data.targetY.append(-targetPoint.z);

    data.mFps = (m_frameFpsPointCloud != nullptr) ? m_frameFpsPointCloud(frame) : 0.0f;
    // SDK 时间戳【实测为微秒】，与 CameraFrameCache 的 cameraTimestampUs 同单位，直接填入。
    // 依据：两次会话首帧时间戳差 52,951,358 与墙钟差 52.95s 完全一致（若为毫秒则相差 1000 倍）；
    // 60fps 下相邻帧差≈16667us。SDK 文档标称"毫秒"有误，以实测为准——切勿再 ×1000。
    const long long timestampUs = (m_frameTimestamp != nullptr) ? m_frameTimestamp(frame) : 0;
    data.timestamp = static_cast<qulonglong>(timestampUs);
    if (m_frameErrorText != nullptr)
    {
        const char* err = m_frameErrorText(frame);
        if (err != nullptr)
        {
            data.errorMessage = QString::fromUtf8(err);
        }
    }

    if (m_frameRelease != nullptr)
    {
        m_frameRelease(frame);
    }

    ++m_decodedFrameCount;
    if (m_frameCache != nullptr)
    {
        m_frameCache->AppendFrame(data);
        ++m_appendedFrameCount;
    }

    if (!m_loggedFirstFrame)
    {
        m_loggedFirstFrame = true;
        WriteCameraSkjLog(QString("first frame target=%1:%2 timestampUs=%3 points=%4 fitPoints=%5 fps=%6")
            .arg(m_serverIP).arg(m_serverPort).arg(data.timestamp)
            .arg(count3d).arg(count2d).arg(static_cast<double>(data.mFps)));
        emitDiagnostic("SKJ SDK 已取到点云帧。");
    }
    else if (m_decodedFrameCount % kDiagnosticEmitStride == 0)
    {
        emitDiagnostic("SKJ SDK 收帧中。");
    }
}

void ScanCameraSkjWorker::stopClient()
{
    m_running = false;
    if (m_pollTimer != nullptr)
    {
        m_pollTimer->stop();
    }
    teardownHandle();
    emitDiagnostic("SKJ SDK 已停止。");
}

void ScanCameraSkjWorker::teardownHandle()
{
    if (m_handle != nullptr)
    {
        if (m_disconnect != nullptr)
        {
            m_disconnect(m_handle);
        }
        if (m_destroy != nullptr)
        {
            m_destroy(m_handle);
        }
        m_handle = nullptr;
    }
}

void ScanCameraSkjWorker::emitDiagnostic(const QString& statusText)
{
    emit diagnosticChanged(m_pollCount, m_noNewFrameCount, m_decodedFrameCount,
        m_decodeFailedCount, m_appendedFrameCount, statusText);
}
