#pragma once

#include <QDialog>
#include <QString>
#include <QStringList>

#include "OnlineServicesConfig.h"
#include "RemoteWorkerLifecycle.h"

#include <functional>
#include <thread>

class QComboBox;
class QFrame;
class QJsonObject;
class QLabel;
class QLineEdit;
class QListWidget;
class QNetworkAccessManager;
class QNetworkReply;
class QPlainTextEdit;
class QProgressBar;
class QPushButton;
class QCheckBox;
class QStackedWidget;
class QTableWidget;
class ScanDataUploader;

// 管理页「在线服务」：OTA 在线升级 + 扫描数据上传 + 服务器配置。
// OTA 源为自建服务器 HTTP 静态目录：{UpdateBaseUrl}/{channel}/latest-v3.json 描述最新版
// （version/notes/file/sha256/publishedAtUtc/expiresAtUtc），channel 按品牌自动取
// neutral 或 brand。客户端只接受验签且处于有效期内的当前 schema 清单。
// 下载完成 SHA256 校验后提示用户确认，退出程序并以 /SILENT 启动 Inno 安装器。
class OnlineServicesDialog : public QDialog
{
    Q_OBJECT

public:
    // uploader 由主窗口持有（常驻，后台自动上传也用它）；flowRunningGuard 返回 true 时禁止安装升级。
    // aboutMode=true 为主页版本号点开的「关于」精简形态：只保留升级区（软件名/当前版本/
    // 可更新版本/更新内容），隐藏上传与服务器配置，打开即自动检查更新。
    // accessLevel 来自刚刚通过服务器验证的固定账号；在线升级对三级账号均开放。
    OnlineServicesDialog(ScanDataUploader* uploader,
        std::function<bool()> flowRunningGuard,
        bool aboutMode = false,
        OnlineServicesConfig::AccessLevel accessLevel = OnlineServicesConfig::AccessLevel::Upload,
        std::function<bool()> privilegedActionGuard = {},
        QWidget* parent = nullptr);
    ~OnlineServicesDialog() override;

    bool IsRemoteOperationBusy() const noexcept;
    void CancelRemoteOperationAndWait(bool permanentShutdown = false);

protected:
    // 关界面时若正在上传：弹「后台继续 / 停止当前上传」，由用户决定。
    void closeEvent(QCloseEvent* event) override;
    // 每次显示自动刷新：正停在「远程数据」页则重新拉取（页面被管理栈缓存复用，构造只跑一次）。
    void showEvent(QShowEvent* event) override;

private:
    void BuildUi();
    void LoadConfigToUi();
    void SaveConfigFromUi();
    QString UpdateChannel() const;   // neutral / brand（按 BrandingConfig 应用名）
    void CheckForUpdate();
    void OnManifestReply(QNetworkReply* reply);
    void StartDownload();
    void OnDownloadFinished(QNetworkReply* reply);
    void FallbackToFullDownload(const QString& reason);
    void InstallDownloadedPackage();
    void AppendLog(const QString& text);
    static int CompareVersions(const QString& lhs, const QString& rhs);  // <0/0/>0

    ScanDataUploader* m_uploader = nullptr;
    std::function<bool()> m_flowRunningGuard;
    std::function<bool()> m_privilegedActionGuard;
    bool m_aboutMode = false;
    OnlineServicesConfig::AccessLevel m_accessLevel = OnlineServicesConfig::AccessLevel::Upload;
    QNetworkAccessManager* m_network = nullptr;

    // 升级区
    QLabel* m_currentVersionLabel = nullptr;
    QLabel* m_latestVersionLabel = nullptr;
    QPlainTextEdit* m_updateNotes = nullptr;
    QPushButton* m_checkUpdateBtn = nullptr;
    QPushButton* m_downloadInstallBtn = nullptr;
    QProgressBar* m_downloadProgress = nullptr;

    // 上传区
    void ShowPickCasesDialog();   // 多选案例上传：列出 Result 下全部案例，多选后按名字顺序入队
    void RefreshUploadUi();
    QCheckBox* m_autoUploadCheck = nullptr;
    QListWidget* m_pendingListWidget = nullptr;
    QPushButton* m_uploadNowBtn = nullptr;
    QPushButton* m_uploadPickBtn = nullptr;
    QLabel* m_uploadStateLabel = nullptr;
    QLabel* m_uploadCurrentLabel = nullptr;
    QLabel* m_uploadDetailLabel = nullptr;
    QLabel* m_uploadQueueLabel = nullptr;
    QProgressBar* m_uploadProgressBar = nullptr;

    // 远程数据区（admin）：恰好一个 owned/joinable FTP worker。析构和主程序退出
    // 都先置取消并 join；worker 只向本对象投递 generation-bound 回调，绝不向 qApp 投递。
    QComboBox* m_remoteDeviceCombo = nullptr;
    QListWidget* m_remoteFileList = nullptr;
    QPushButton* m_remoteRefreshBtn = nullptr;
    QPushButton* m_remoteDownloadBtn = nullptr;
    QPushButton* m_remoteDeleteBtn = nullptr;   // 删除选中数据包（服务器上）
    QPushButton* m_remoteMkdirBtn = nullptr;    // 新建设备目录
    QLabel* m_remoteDownloadStateLabel = nullptr;
    QLabel* m_remoteDownloadCurrentLabel = nullptr;
    QLabel* m_remoteDownloadDetailLabel = nullptr;
    QLabel* m_remoteDownloadQueueLabel = nullptr;
    QProgressBar* m_remoteDownloadProgressBar = nullptr;
    QListWidget* m_remoteDownloadQueueList = nullptr;
    QStringList m_remoteDownloadNames;
    QStringList m_remoteDownloadStates;
    bool m_remoteBusy = false;
    std::thread m_remoteWorker;
    RemoteWorkerLifecycle m_remoteLifecycle;
    void RefreshRemoteDevices();
    void RefreshRemoteFiles();
    void DownloadSelectedRemoteFiles();
    void BeginRemoteDownloadUi(const QStringList& names);
    void UpdateRemoteDownloadUi(int currentIndex,
        int completedItems,
        qulonglong receivedBytes,
        qulonglong totalBytes,
        double bytesPerSec,
        int etaSeconds,
        const QString& phase,
        const QString& itemState = QString());
    void FinishRemoteDownloadUi(int completedItems,
        bool stoppedEarly,
        const QString& summary);
    void RefreshRemoteDownloadQueueUi();
    void DeleteSelectedRemoteFiles();
    void CreateRemoteDeviceDir();
    void SetRemoteBusy(bool busy);
    bool StartRemoteWorker(std::function<void(quint64)> work);
    bool RemoteWorkerCancelled(quint64 generation) const noexcept;
    void FinishRemoteWorker(quint64 generation);
    bool AuthorizeFtpAction(const QString& actionName);
    bool AuthorizePrivilegedAction(const QString& actionName);

    // 仪表盘：左侧导航 + 右侧页面栈（云控制台式布局），总览页放大数字统计卡与设备资源表
    QListWidget* m_navList = nullptr;
    QStackedWidget* m_pagesStack = nullptr;
    int m_remoteNavRow = -1;    // 「远程数据」导航行号（-1=未建）
    int m_accountNavRow = -1;   // 「账号管理」导航行号（-1=未建）
    int m_serverConfigNavRow = -1;
    bool IsUploadOnlyAccount() const;
    bool HasFtpAccess() const;
    bool HasFullAccess() const;
    void UpdateRestrictedNav();
    enum class ServerStatusLevel
    {
        Info,
        Success,
        Error
    };
    void SetServerStatusBanner(const QString& message, ServerStatusLevel level);
    QLabel* m_serverStatusLabel = nullptr;
    QLabel* m_cardDisk = nullptr;       // 磁盘用量百分比（大数字）
    QLabel* m_cardDiskSub = nullptr;    // 已用/总量/剩余
    QProgressBar* m_diskBar = nullptr;  // 磁盘用量进度条
    QLabel* m_cardCloud = nullptr;      // 云端数据总量（大数字）
    QLabel* m_cardDevices = nullptr;    // 设备数（大数字）
    QLabel* m_cardQueue = nullptr;      // 本机待传队列（大数字）
    QTableWidget* m_deviceTable = nullptr;   // 设备资源列表（名称/数据量/文件数/最近上传）
    void RefreshServerStats();
    void RefreshServerStatsViaFtp();
    void UpdateQueueCard();

    // 账号管理（admin，经服务器管理接口：nginx /admin/ 反代 + X-Admin-Token）
    QTableWidget* m_accountTable = nullptr;
    void RefreshAccounts();
    void ShowAddAccountDialog();
    void ChangeSelectedAccountPassword();
    void ToggleSelectedAccountPermission();
    void DeleteSelectedAccount();

    // 管理接口通用请求（UI 线程，QNetworkAccessManager 异步；令牌空时提示并回调失败）
    QString AdminApiBase() const;
    bool CanUseSecureAdminTransport() const;
    void AdminRequest(const QByteArray& verb, const QString& path, const QJsonObject& body,
        std::function<void(bool ok, const QJsonObject& resp)> done);

    // 配置区
    void RequestServerConfigEdit();
    void SetServerConfigEditing(bool editing);
    QLineEdit* m_serverHostEdit = nullptr;
    QLineEdit* m_deviceNameEdit = nullptr;
    QPushButton* m_editServerConfigBtn = nullptr;
    QPushButton* m_saveServerConfigBtn = nullptr;
    bool m_serverConfigEditing = false;

    QPlainTextEdit* m_logText = nullptr;

    // 本次会话发现的新版信息。增量补丁（patch 字段，仅含变更文件的 zip，通常只有主程序
    // 约 2MB）优先于全量安装包（80+MB）；补丁安装 = 退出后解压覆盖安装目录再重启。
    QString m_remoteVersion;
    QString m_remoteFile;
    QString m_remoteSha256;
    qint64 m_remoteSize = 0;
    QString m_remotePatchFile;
    QString m_remotePatchSha256;
    qint64 m_remotePatchSize = 0;
    qint64 m_remoteManifestPublishedAtUtc = 0;
    qint64 m_remoteManifestExpiresAtUtc = 0;
    bool m_usePatch = false;
    QString m_downloadedPath;
    bool m_checkingForUpdate = false;
    bool m_downloading = false;
};
