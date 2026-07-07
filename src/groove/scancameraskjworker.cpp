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

#include <chrono>

#include <limits>

namespace
{
// 与 protocol.h 的 SKJ_* 返回码一致（不直接 include SDK 头，避免 dllimport 声明干扰动态加载）。
constexpr int kSkjOk = 0;
constexpr int kSkjErrNoData = -106;        // 暂无新帧
constexpr int kSkjErrDataDuplicate = -107; // 最新帧已取过

// 轮询间隔：相机约 60fps（~16ms 一帧），10ms 轮询可跟上，无新帧时返回 DUPLICATE/NO_DATA 属正常。
constexpr int kPollIntervalMs = 10;
constexpr int kImageServerPort = 50001;      // 图像传输独立端口（SDK v1.2.0，与点云 50006 分开）
constexpr int kImagePollIntervalMs = 30;     // 图像轮询周期：视频流一般 ≤30fps，30ms 足够
constexpr std::size_t kImageQueueMax = 8;    // 保存队列上限：写盘跟不上时丢最旧帧，保护内存
constexpr int kImageWebpQuality = 80;        // WebP 有损质量：同观感比 JPG85 小约三成

qint64 SkjSteadyNowUs()
{
    return static_cast<qint64>(std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}
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
    m_setFrameBufferCount = reinterpret_cast<SetFrameBufferFn>(resolve("SKJCamera_SetFrameBufferCount"));
    m_getLatestFrame = reinterpret_cast<GetLatestFrameFn>(resolve("SKJCamera_GetLatestFrame"));
    m_frameRelease = reinterpret_cast<FrameReleaseFn>(resolve("SKJFrame_Release"));
    m_frameTimestamp = reinterpret_cast<FrameTimestampFn>(resolve("SKJFrame_GetTimestamp"));
    m_framePoint3DCount = reinterpret_cast<FrameIntFn>(resolve("SKJFrame_GetPoint3DCount"));
    m_framePoint3DData = reinterpret_cast<FramePtrFn>(resolve("SKJFrame_GetPoint3DData"));
    m_framePoint2DCount = reinterpret_cast<FrameIntFn>(resolve("SKJFrame_GetPoint2DCount"));
    m_framePoint2DData = reinterpret_cast<FramePtrFn>(resolve("SKJFrame_GetPoint2DData"));
    m_frameResultPoint = reinterpret_cast<FrameResultFn>(resolve("SKJFrame_GetResultPointValue"));
    m_frameFpsPointCloud = reinterpret_cast<FrameFloatFn>(resolve("SKJFrame_GetFpsPointCloud"));
    m_frameChannel = reinterpret_cast<FrameIntFn>(resolve("SKJFrame_GetChannel"));
    m_frameErrorText = reinterpret_cast<FrameTextFn>(resolve("SKJFrame_GetErrorText"));
    m_errorString = reinterpret_cast<ErrorStringFn>(resolve("SKJCamera_GetErrorString"));

    // 图像传输接口（SDK v1.2.0）：全部可选解析——旧 DLL 无这些导出时保持 null，图像功能自动禁用，不影响点云。
    m_imageConnect = reinterpret_cast<ImageConnectFn>(resolve("SKJCamera_ConnectImage"));
    m_imageDisconnect = reinterpret_cast<ImageDisconnectFn>(resolve("SKJCamera_DisconnectImage"));
    m_getLatestImage = reinterpret_cast<GetLatestImageFn>(resolve("SKJCamera_GetLatestImage"));
    m_imageRelease = reinterpret_cast<ImageReleaseFn>(resolve("SKJImage_Release"));
    m_imageWidth = reinterpret_cast<ImageIntFn>(resolve("SKJImage_GetWidth"));
    m_imageHeight = reinterpret_cast<ImageIntFn>(resolve("SKJImage_GetHeight"));
    m_imageStride = reinterpret_cast<ImageIntFn>(resolve("SKJImage_GetStride"));
    m_imageData = reinterpret_cast<ImageDataFn>(resolve("SKJImage_GetData"));
    m_imageTimestamp = reinterpret_cast<ImageTimestampFn>(resolve("SKJImage_GetTimestamp"));

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

    // v1.2.0 FIFO 环形缓冲：加深到 8 帧（默认 2），取帧线程短暂变慢也不丢中间帧；旧 DLL 无此接口自动跳过。
    if (m_setFrameBufferCount != nullptr)
    {
        const int bufferRet = m_setFrameBufferCount(m_handle, 8);
        WriteCameraSkjLog(QString("SetFrameBufferCount(8) ret=%1").arg(bufferRet));
    }
    m_tsUnitMultiplier = 0;  // 重连后重新判定时间戳单位
    m_lastRawTsForUnitDetect = 0;

    if (m_pollTimer == nullptr)
    {
        m_pollTimer = new QTimer(this);
        m_pollTimer->setTimerType(Qt::PreciseTimer);
        connect(m_pollTimer, &QTimer::timeout, this, &ScanCameraSkjWorker::pollFrame);
    }
    m_pollTimer->start(m_pollIntervalMs);
    emitDiagnostic(QString("SKJ SDK 已连接 %1:%2，开始取帧（轮询间隔 %3 ms）。").arg(m_serverIP).arg(m_serverPort).arg(m_pollIntervalMs));

    // 图像传输连接（独立端口，失败只告警不影响点云取帧）。总开关(预览页按钮)关闭时不连。
    m_imageConnected = false;
    if (m_frameCache == nullptr || m_frameCache->ImageTransportEnabled())
    {
        connectImageTransport();
    }
    else
    {
        WriteCameraSkjLog("image transport disabled by user switch, skip image connect");
    }
}

void ScanCameraSkjWorker::connectImageTransport()
{
    if (m_imageConnected || m_handle == nullptr)
    {
        return;
    }
    if (m_imageConnect == nullptr || m_getLatestImage == nullptr || m_imageData == nullptr || m_imageRelease == nullptr)
    {
        WriteCameraSkjLog("image transfer APIs not exported by SKJCamera.dll, image capture disabled (old SDK)");
        return;
    }
    const QByteArray ipBytes = m_serverIP.toLocal8Bit();
    const int imageRet = m_imageConnect(m_handle, ipBytes.constData(), kImageServerPort);
    if (imageRet == kSkjOk)
    {
        m_imageConnected = true;
        startImageSaveThread();
        if (m_imageTimer == nullptr)
        {
            m_imageTimer = new QTimer(this);
            m_imageTimer->setTimerType(Qt::CoarseTimer);
            connect(m_imageTimer, &QTimer::timeout, this, &ScanCameraSkjWorker::pollImage);
        }
        m_imageTimer->start(kImagePollIntervalMs);
        WriteCameraSkjLog(QString("image connected target=%1:%2").arg(m_serverIP).arg(kImageServerPort));
    }
    else
    {
        const QString reason = (m_errorString != nullptr)
            ? QString::fromUtf8(m_errorString(imageRet))
            : QString::number(imageRet);
        WriteCameraSkjLog(QString("image connect failed target=%1:%2 ret=%3 (%4)")
            .arg(m_serverIP).arg(kImageServerPort).arg(imageRet).arg(reason));
    }
}

void ScanCameraSkjWorker::disconnectImageTransport()
{
    if (m_imageTimer != nullptr)
    {
        m_imageTimer->stop();
    }
    if (m_imageConnected && m_imageDisconnect != nullptr && m_handle != nullptr)
    {
        m_imageDisconnect(m_handle);
        WriteCameraSkjLog("image transport disconnected by user switch");
    }
    m_imageConnected = false;
}

void ScanCameraSkjWorker::setImageTransportEnabled(bool enabled)
{
    if (m_frameCache != nullptr)
    {
        m_frameCache->SetImageTransportEnabled(enabled);
    }
    if (enabled)
    {
        if (m_running)
        {
            connectImageTransport();
        }
    }
    else
    {
        disconnectImageTransport();
    }
}

void ScanCameraSkjWorker::pollFrame()
{
    // v1.2.0 起 GetLatestFrame 为 FIFO 弹最旧帧：每个 tick 排空队列（上限=FIFO最大深度16，防死循环），
    // 保证既不丢帧也不积压滞后；旧 DLL(单槽最新帧)下循环第二次即返回空，行为不变。
    for (int drained = 0; drained < 16; ++drained)
    {
        if (!pollFrameOnce())
        {
            break;
        }
    }
}

bool ScanCameraSkjWorker::pollFrameOnce()
{
    if (!m_running || m_handle == nullptr || m_getLatestFrame == nullptr)
    {
        return false;
    }

    ++m_pollCount;
    void* frame = nullptr;
    const int ret = m_getLatestFrame(m_handle, &frame);

    if (ret == kSkjErrNoData || ret == kSkjErrDataDuplicate)
    {
        // 无新帧/重复：记录一次 SDK 取帧调用（无帧数据）。诊断"帧时间戳跳变"时，若跳变窗口内持续
        // 出现这类调用，说明 SDK 当时确实没有新帧可给（上游少给），而非我方漏取。
        if (m_frameCache != nullptr)
        {
            m_frameCache->RecordPollStatus(ret, 0, -1);
        }
        ++m_noNewFrameCount;
        if (m_pollCount % kDiagnosticEmitStride == 0)
        {
            emitDiagnostic("SKJ SDK 收帧中（等待新帧）。");
        }
        return false;
    }
    if (ret != kSkjOk || frame == nullptr)
    {
        if (m_frameCache != nullptr)
        {
            m_frameCache->RecordPollStatus(ret, 0, -1);
        }
        ++m_decodeFailedCount;
        if (frame != nullptr && m_frameRelease != nullptr)
        {
            m_frameRelease(frame);
        }
        const QString reason = (m_errorString != nullptr)
            ? QString::fromUtf8(m_errorString(ret))
            : QString::number(ret);
        emitDiagnostic(QString("取帧失败：%1").arg(reason));
        return false;
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
    // SDK 时间戳单位运行时自适应：旧版 DLL 实测为微秒(文档误标毫秒)，v1.2.0「修改时间戳获取方式」后
    // 单位未知——用相邻帧差判定一次：差<2000(60fps≈16.7、250fps≈4) → 毫秒×1000 归一；差≥2000(≈16667us) → 微秒原样。
    const int frameChannel = (m_frameChannel != nullptr) ? m_frameChannel(frame) : -1;  // 帧号须在 Release 前读取
    const long long rawTimestamp = (m_frameTimestamp != nullptr) ? m_frameTimestamp(frame) : 0;
    if (m_tsUnitMultiplier == 0 && rawTimestamp > 0)
    {
        if (m_lastRawTsForUnitDetect > 0 && rawTimestamp > m_lastRawTsForUnitDetect)
        {
            const long long unitDetectDelta = rawTimestamp - m_lastRawTsForUnitDetect;
            m_tsUnitMultiplier = (unitDetectDelta < 2000) ? 1000 : 1;
            WriteCameraSkjLog(QString("timestamp unit detected: frame delta=%1 -> %2")
                .arg(unitDetectDelta)
                .arg(m_tsUnitMultiplier == 1000 ? "ms(x1000)" : "us(x1)"));
        }
        m_lastRawTsForUnitDetect = rawTimestamp;
    }
    const long long timestampUs = rawTimestamp * (m_tsUnitMultiplier > 0 ? m_tsUnitMultiplier : 1);
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
        // 记录本次成功取帧调用：带 SDK 帧时间戳、三维点数与帧号，供 SdkPollLog 逐帧对齐/精确数丢帧。
        m_frameCache->RecordPollStatus(kSkjOk, static_cast<qint64>(timestampUs), count3d, frameChannel);
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
    return true;
}

void ScanCameraSkjWorker::pollImage()
{
    if (!m_running || !m_imageConnected || m_handle == nullptr
        || m_getLatestImage == nullptr || m_imageData == nullptr || m_imageRelease == nullptr
        || m_frameCache == nullptr)
    {
        return;
    }
    // 取图条件：扫描采集窗口开启（保存）或实时预览开启（仅显示）。两者都关时不取，避免空转拷贝。
    const QString captureDir = m_frameCache->ImageCaptureDir();
    const bool liveEnabled = m_frameCache->LiveImageEnabled();
    if (captureDir.isEmpty() && !liveEnabled)
    {
        return;
    }

    void* image = nullptr;
    const int ret = m_getLatestImage(m_handle, &image);
    if (ret != kSkjOk || image == nullptr)
    {
        if (image != nullptr)
        {
            m_imageRelease(image);
        }
        return;  // 无新图/重复/错误都静默跳过（30ms 高频轮询不刷日志）
    }

    const int width = (m_imageWidth != nullptr) ? m_imageWidth(image) : 0;
    const int height = (m_imageHeight != nullptr) ? m_imageHeight(image) : 0;
    const int stride = (m_imageStride != nullptr) ? m_imageStride(image) : 0;
    const unsigned char* data = m_imageData(image);
    const long long imageTimestamp = (m_imageTimestamp != nullptr) ? m_imageTimestamp(image) : 0;
    if (width <= 0 || height <= 0 || stride <= 0 || data == nullptr
        || (imageTimestamp != 0 && imageTimestamp == m_lastSavedImageTimestamp))
    {
        m_imageRelease(image);
        return;
    }
    m_lastSavedImageTimestamp = imageTimestamp;

    // 抽帧采样只作用于【保存】；实时显示要最新画面，不抽帧。
    const int frameStride = m_frameCache->ImageCaptureFrameStride();
    const qint64 newFrameIndex = m_imageNewFrameCount++;
    const bool saveThisFrame = !captureDir.isEmpty()
        && (frameStride <= 1 || (newFrameIndex % frameStride) == 0);
    if (!saveThisFrame && !liveEnabled)
    {
        m_imageRelease(image);
        return;
    }

    // BGR24 与 QImage::Format_BGR888 内存布局一致；wrap 后深拷贝再释放 SDK 句柄。
    QImage imageCopy = QImage(data, width, height, stride, QImage::Format_BGR888).copy();
    m_imageRelease(image);

    if (liveEnabled)
    {
        m_frameCache->SetLatestImage(imageCopy, imageTimestamp);  // 隐式共享，无额外拷贝
    }
    if (!saveThisFrame)
    {
        return;
    }

    // 文件名带图像 SDK 时间戳 + PC steady 接收时刻（us），与 SdkPollLog/MatchDebug 的时间轴可直接对齐。
    PendingImageSave pending;
    pending.filePath = captureDir + QString("/img_%1_%2.webp").arg(imageTimestamp).arg(SkjSteadyNowUs());
    pending.image = std::move(imageCopy);
    {
        std::lock_guard<std::mutex> locker(m_imageSaveMutex);
        while (m_imageSaveQueue.size() >= kImageQueueMax)
        {
            m_imageSaveQueue.pop_front();  // 写盘跟不上时丢最旧帧，绝不无限占内存
            ++m_imageDroppedCount;
        }
        m_imageSaveQueue.push_back(std::move(pending));
    }
    m_imageSaveCv.notify_one();
}

void ScanCameraSkjWorker::startImageSaveThread()
{
    if (m_imageSaveThread.joinable())
    {
        return;
    }
    m_imageSaveStop.store(false);
    // JPG 编码+写盘放独立线程：单帧编码 5~15ms，若留在 worker 线程会拖慢点云取帧轮询。
    m_imageSaveThread = std::thread([this]()
        {
            for (;;)
            {
                PendingImageSave item;
                {
                    std::unique_lock<std::mutex> locker(m_imageSaveMutex);
                    m_imageSaveCv.wait(locker, [this]()
                        {
                            return m_imageSaveStop.load() || !m_imageSaveQueue.empty();
                        });
                    if (m_imageSaveQueue.empty())
                    {
                        if (m_imageSaveStop.load())
                        {
                            return;  // 队列已排干才退出，扫描尾帧不丢
                        }
                        continue;
                    }
                    item = std::move(m_imageSaveQueue.front());
                    m_imageSaveQueue.pop_front();
                }
                if (item.image.save(item.filePath, "WEBP", kImageWebpQuality))
                {
                    ++m_imageSavedCount;
                }
            }
        });
}

void ScanCameraSkjWorker::stopImageSaveThread()
{
    if (!m_imageSaveThread.joinable())
    {
        return;
    }
    m_imageSaveStop.store(true);
    m_imageSaveCv.notify_all();
    m_imageSaveThread.join();
    std::lock_guard<std::mutex> locker(m_imageSaveMutex);
    m_imageSaveQueue.clear();
}

void ScanCameraSkjWorker::stopClient()
{
    m_running = false;
    if (m_imageTimer != nullptr)
    {
        m_imageTimer->stop();
    }
    stopImageSaveThread();
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
        if (m_imageConnected && m_imageDisconnect != nullptr)
        {
            m_imageDisconnect(m_handle);
        }
        m_imageConnected = false;
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
