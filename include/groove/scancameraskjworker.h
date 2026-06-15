#pragma once

#include <QObject>
#include <QString>

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
    void startClient(const QString& serverIP, int serverPort);
    void stopClient();

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

private:
    bool loadSdk(QString* error);
    void* resolve(const char* name);
    void teardownHandle();
    void emitDiagnostic(const QString& statusText);
    QString resolveDllPath() const;

    CameraFrameCache* m_frameCache = nullptr;
    QLibrary* m_library = nullptr;
    void* m_handle = nullptr;
    QTimer* m_pollTimer = nullptr;
    QString m_serverIP;
    int m_serverPort = 0;
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
    using GetLatestFrameFn = int(__cdecl*)(void*, void**);
    using FrameReleaseFn = void(__cdecl*)(void*);
    using FrameTimestampFn = long long(__cdecl*)(const void*);
    using FrameIntFn = int(__cdecl*)(const void*);
    using FramePtrFn = const void*(__cdecl*)(const void*);
    using FrameResultFn = int(__cdecl*)(const void*, void*);
    using FrameFloatFn = float(__cdecl*)(const void*);
    using FrameTextFn = const char*(__cdecl*)(const void*);
    using ErrorStringFn = const char*(__cdecl*)(int);

    CreateFn m_create = nullptr;
    DestroyFn m_destroy = nullptr;
    ConnectFn m_connect = nullptr;
    DisconnectFn m_disconnect = nullptr;
    IsConnectedFn m_isConnected = nullptr;
    SetTimeoutFn m_setConnectTimeout = nullptr;
    GetLatestFrameFn m_getLatestFrame = nullptr;
    FrameReleaseFn m_frameRelease = nullptr;
    FrameTimestampFn m_frameTimestamp = nullptr;
    FrameIntFn m_framePoint3DCount = nullptr;
    FramePtrFn m_framePoint3DData = nullptr;
    FrameIntFn m_framePoint2DCount = nullptr;
    FramePtrFn m_framePoint2DData = nullptr;
    FrameResultFn m_frameResultPoint = nullptr;
    FrameFloatFn m_frameFpsPointCloud = nullptr;
    FrameTextFn m_frameErrorText = nullptr;
    ErrorStringFn m_errorString = nullptr;
};
