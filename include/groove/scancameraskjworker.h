#pragma once

#include "SKJCameraControlClient.h"

#include <QImage>
#include <QObject>
#include <QString>

#include <atomic>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>

class QLibrary;
class QTimer;
class CameraFrameCache;

// 坡口相机数据接收（新协议 "SKJF"）。
// 厂商升级相机后数据帧从旧 0xABCDEF12 协议换成 "SKJF"，旧的 ScanCameraTcpClientWorker 自解析已失效。
// 本 worker 改用厂商 SKJCamera SDK（SKJCamera.dll，QLibrary 动态加载，与 SKJCameraControlClient 同一 DLL）：
// SKJCamera_Connect 建连 + 内部接收线程，本 worker 用 QTimer 轮询 SKJCamera_GetLatestFrame 取帧，
// 转成 udpDataShow 填入 CameraFrameCache（预览/扫描下游不变）。诊断信号与 TCP worker 同构。
class ScanCameraSkjWorker : public QObject
{
    Q_OBJECT
public:
    explicit ScanCameraSkjWorker(CameraFrameCache* frameCache = nullptr, QObject* parent = nullptr);
    ~ScanCameraSkjWorker() override;

public slots:
    void startClient(const QString& serverIP, int serverPort, int pollIntervalMs = 10);
    void stopClient();
    void setImageTransportEnabled(bool enabled);  // 预览页「图像传输」开关：即时连/断图像口(50001)
    void setFrameBufferCount(int count);  // 预览页「接收缓冲帧数」：写缓存共享值，已连接时立即下发 SDK
    // 参数与激光命令必须复用点云连接的同一个 SDK 句柄，并在 worker 所在线程调用。
    // 相机端只允许一条 50006 命令/点云会话；另建控制句柄会抢占现有连接并连带中断图像预览。
    bool readControlParameters(SKJCameraParameterValues* values, QString* error = nullptr);
    bool setControlParameter(SKJCameraControlClient::Parameter parameter, int value, QString* error = nullptr);
    bool setLaserEnabled(bool enabled, QString* error = nullptr);

signals:
    // 与 ScanCameraTcpClientWorker::diagnosticChanged 同构，复用现有预览诊断显示：
    // datagramCount=轮询次数, filteredDatagramCount=无新帧次数(NO_DATA/DUPLICATE),
    // decodedFrameCount=取到帧数, decodeFailedCount=取帧错误数, appendedFrameCount=写入缓存数。
    void diagnosticChanged(qint64 datagramCount,
        qint64 filteredDatagramCount,
        qint64 decodedFrameCount,
        qint64 decodeFailedCount,
        qint64 appendedFrameCount,
        const QString& statusText);

private slots:
    void pollFrame();
    void pollImage();

private:
    bool pollFrameOnce();  // 取一帧；true=取到(FIFO 可能还有,继续排空)，false=队列空/错误

private:
    bool loadSdk(QString* error);
    void* resolve(const char* name);
    void teardownHandle();
    void emitDiagnostic(const QString& statusText);
    QString resolveDllPath() const;
    void startImageSaveThread();
    void stopImageSaveThread();
    void connectImageTransport();
    void disconnectImageTransport();

    CameraFrameCache* m_frameCache = nullptr;
    QLibrary* m_library = nullptr;
    void* m_handle = nullptr;
    QTimer* m_pollTimer = nullptr;
    QTimer* m_imageTimer = nullptr;
    bool m_imageConnected = false;
    // 图像保存异步化：worker 线程只取帧+深拷贝入队，JPG 编码/写盘在独立线程做，避免拖慢点云取帧轮询。
    struct PendingImageSave
    {
        QString filePath;
        QImage image;
    };
    std::deque<PendingImageSave> m_imageSaveQueue;
    std::mutex m_imageSaveMutex;
    std::condition_variable m_imageSaveCv;
    std::thread m_imageSaveThread;
    std::atomic_bool m_imageSaveStop{ false };
    qint64 m_lastSavedImageTimestamp = -1;
    qint64 m_imageNewFrameCount = 0;   // 采集窗口内看到的新图像计数（抽帧用）
    qint64 m_imageSavedCount = 0;
    qint64 m_imageDroppedCount = 0;
    QString m_serverIP;
    int m_serverPort = 0;
    int m_pollIntervalMs = 10;  // 取帧轮询间隔(ms)，由相机参数 CameraReadFps 换算(1000/帧率)后经 startClient 传入
    // SDK v1.2.0「修改时间戳获取方式」后单位未知（旧版实测微秒、新版文档标毫秒）：
    // 首两帧按相邻差自适应判定，0=未判定，1=微秒原样，1000=毫秒×1000 归一到微秒。
    qint64 m_tsUnitMultiplier = 0;
    qint64 m_lastRawTsForUnitDetect = 0;
    bool m_running = false;
    bool m_loggedFirstFrame = false;

    qint64 m_pollCount = 0;
    qint64 m_noNewFrameCount = 0;
    qint64 m_decodedFrameCount = 0;
    qint64 m_decodeFailedCount = 0;
    qint64 m_appendedFrameCount = 0;

    // SKJCamera.dll 导出函数（C ABI，__cdecl；与 SKJCameraControlClient 风格一致）。
    using CreateFn = void*(__cdecl*)();
    using DestroyFn = void(__cdecl*)(void*);
    using ConnectFn = int(__cdecl*)(void*, const char*, int);
    using DisconnectFn = void(__cdecl*)(void*);
    using IsConnectedFn = int(__cdecl*)(void*);
    using SetTimeoutFn = int(__cdecl*)(void*, int);
    using GetIntFn = int(__cdecl*)(void*, int*);
    using SetIntFn = int(__cdecl*)(void*, int);
    using CommandFn = int(__cdecl*)(void*);
    using SetFrameBufferFn = int(__cdecl*)(void*, int);
    using GetLatestFrameFn = int(__cdecl*)(void*, void**);
    using FrameReleaseFn = void(__cdecl*)(void*);
    using FrameTimestampFn = long long(__cdecl*)(const void*);
    using FrameIntFn = int(__cdecl*)(const void*);
    using FramePtrFn = const void*(__cdecl*)(const void*);
    using FrameResultFn = int(__cdecl*)(const void*, void*);
    using FrameFloatFn = float(__cdecl*)(const void*);
    using FrameTextFn = const char*(__cdecl*)(const void*);
    using ErrorStringFn = const char*(__cdecl*)(int);

    // 图像传输接口（SDK v1.2.0 新增，旧 DLL 下解析为 null 即自动禁用图像功能，不影响点云）。
    using ImageConnectFn = int(__cdecl*)(void*, const char*, int);
    using ImageDisconnectFn = void(__cdecl*)(void*);
    using GetLatestImageFn = int(__cdecl*)(void*, void**);
    using ImageReleaseFn = void(__cdecl*)(void*);
    using ImageIntFn = int(__cdecl*)(const void*);
    using ImageDataFn = const unsigned char*(__cdecl*)(const void*);
    using ImageTimestampFn = long long(__cdecl*)(const void*);

    CreateFn m_create = nullptr;
    DestroyFn m_destroy = nullptr;
    ConnectFn m_connect = nullptr;
    DisconnectFn m_disconnect = nullptr;
    ImageConnectFn m_imageConnect = nullptr;
    ImageDisconnectFn m_imageDisconnect = nullptr;
    GetLatestImageFn m_getLatestImage = nullptr;
    ImageReleaseFn m_imageRelease = nullptr;
    ImageIntFn m_imageWidth = nullptr;
    ImageIntFn m_imageHeight = nullptr;
    ImageIntFn m_imageStride = nullptr;
    ImageDataFn m_imageData = nullptr;
    ImageTimestampFn m_imageTimestamp = nullptr;
    IsConnectedFn m_isConnected = nullptr;
    SetTimeoutFn m_setConnectTimeout = nullptr;
    SetTimeoutFn m_setCommandTimeout = nullptr;
    GetIntFn m_getExposure = nullptr;
    SetIntFn m_setExposure = nullptr;
    GetIntFn m_getGain = nullptr;
    SetIntFn m_setGain = nullptr;
    GetIntFn m_getBinarize = nullptr;
    SetIntFn m_setBinarize = nullptr;
    CommandFn m_laserOn = nullptr;
    CommandFn m_laserOff = nullptr;
    SetFrameBufferFn m_setFrameBufferCount = nullptr;  // v1.2.0 FIFO 深度设置，旧 DLL 为 null 自动跳过
    GetLatestFrameFn m_getLatestFrame = nullptr;
    FrameReleaseFn m_frameRelease = nullptr;
    FrameTimestampFn m_frameTimestamp = nullptr;
    FrameIntFn m_framePoint3DCount = nullptr;
    FramePtrFn m_framePoint3DData = nullptr;
    FrameIntFn m_framePoint2DCount = nullptr;
    FramePtrFn m_framePoint2DData = nullptr;
    FrameResultFn m_frameResultPoint = nullptr;
    FrameFloatFn m_frameFpsPointCloud = nullptr;
    FrameIntFn m_frameChannel = nullptr;   // SDK 帧号（厂商临时以 GetChannel 承载，逐帧+1）
    FrameTextFn m_frameErrorText = nullptr;
    ErrorStringFn m_errorString = nullptr;
};
