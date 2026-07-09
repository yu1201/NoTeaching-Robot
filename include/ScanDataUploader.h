#pragma once

#include <QObject>
#include <QString>
#include <QStringList>

#include <atomic>
#include <mutex>
#include <thread>

class QTimer;
class RobotLog;

// 扫描数据在线上传服务：把 Result/<机器人>/<案例> 目录压成 zip 经 FTP 推到自建服务器
// /data/<设备名>/ 下。扫描流程完成后自动入队（受 OnlineServicesConfig::AutoUploadEnabled
// 开关控制），失败留在待传队列（持久化到 ConfigDatabase），定时重试 + 管理页可手动触发。
// 线程约定：队列与 ConfigDatabase 读写只在 UI 线程；zip 与 FTP 在一次性后台线程，
// 结果经 invokeMethod 回 UI 线程落账。
class ScanDataUploader : public QObject
{
    Q_OBJECT

public:
    explicit ScanDataUploader(QObject* parent = nullptr);
    ~ScanDataUploader() override;

    // 入队并立即尝试上传（UI 线程调用）。caseDir 为 Result 案例目录绝对或工程相对路径。
    void QueueUpload(const QString& caseDir);
    // 手动触发处理当前全部待传项（管理页按钮）。
    void TriggerUploadNow();
    QStringList PendingList() const;
    bool IsBusy() const;

    // 进度快照（任意线程可读，mutex 保护）：退出拦截进度框/界面用。
    struct ProgressSnapshot
    {
        int doneItems = 0;        // 本轮已完成案例数
        int totalItems = 0;       // 本轮总案例数
        QString currentName;      // 当前正在传的案例名
        qint64 sentBytes = 0;     // 当前文件已传字节
        qint64 totalBytes = 0;    // 当前文件总字节
        double bytesPerSec = 0.0; // 当前速度
        int etaSeconds = 0;       // 当前文件预计剩余秒
    };
    ProgressSnapshot CurrentProgress() const;

    // 请求取消当前上传（UI 线程调用，立即返回）：后台线程块间中止并删服务器半截文件。
    void RequestCancel();
    // 请求取消并阻塞等待后台线程收尾（含半截文件删除）后返回——退出程序「强制退出」路径用。
    void CancelAndWait();

signals:
    // 每条进展/结果消息（UI 线程发射），管理页日志区与状态栏共用。
    void uploadStatus(const QString& message);
    void pendingChanged(int count);
    // 上传进度（UI 线程发射，节流 ~200ms）：done/total 案例、当前案例名、当前文件已传/总字节、速度、ETA秒。
    void uploadProgress(int doneItems, int totalItems, const QString& currentName,
        qint64 sentBytes, qint64 totalBytes, double bytesPerSec, int etaSeconds);

private:
    // FTP/设备名配置快照：UI 线程读 ConfigDatabase 装好后传给后台线程（QSQLITE 连接不可跨线程）。
    struct UploadConfig
    {
        std::string host;
        int port = 21;
        std::string user;
        std::string password;
        QString deviceName;
    };

    void LoadPending();
    void SavePending();
    void StartWorkerIfIdle();
    void JoinWorker();
    // 后台线程体：处理传入快照，逐项 zip+FTP；每项结果回 UI 线程 OnItemFinished。
    void WorkerBody(const QStringList& items, const UploadConfig& config);
    void OnItemFinished(const QString& caseDir, bool ok, const QString& message);
    static bool ZipCaseDir(const QString& caseDir, const QString& zipPath, QString* error);

    QStringList m_pending;
    std::thread m_worker;
    std::atomic<bool> m_busy{ false };
    std::atomic<bool> m_cancel{ false };      // 取消标志（UI 置真，worker 与 FtpClient 块间读）
    mutable std::mutex m_progMutex;
    ProgressSnapshot m_prog;                   // 进度快照（worker 写、UI 读，m_progMutex 保护）
    QTimer* m_retryTimer = nullptr;
    RobotLog* m_log = nullptr;
};
