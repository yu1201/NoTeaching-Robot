#pragma once

#include <QDialog>
#include <QString>

#include <functional>

class QComboBox;
class QLabel;
class QLineEdit;
class QListWidget;
class QNetworkAccessManager;
class QNetworkReply;
class QPlainTextEdit;
class QProgressBar;
class QPushButton;
class QCheckBox;
class ScanDataUploader;

// 管理页「在线服务」：OTA 在线升级 + 扫描数据上传 + 服务器配置。
// OTA 源为自建服务器 HTTP 静态目录：{UpdateBaseUrl}/{channel}/latest.json 描述最新版
// （version/notes/file/sha256），channel 按品牌自动取 neutral 或 brand。
// 下载完成 SHA256 校验后提示用户确认，退出程序并以 /SILENT 启动 Inno 安装器。
class OnlineServicesDialog : public QDialog
{
    Q_OBJECT

public:
    // uploader 由主窗口持有（常驻，后台自动上传也用它）；flowRunningGuard 返回 true 时禁止安装升级。
    // aboutMode=true 为主页版本号点开的「关于」精简形态：只保留升级区（软件名/当前版本/
    // 可更新版本/更新内容），隐藏上传与服务器配置，打开即自动检查更新。
    // remoteBrowseAllowed=true（admin 账户）时显示「远程数据」区：浏览服务器上各设备
    // 上传的数据并下载解压到本地 Result/Remote/<设备>/ 查看。
    OnlineServicesDialog(ScanDataUploader* uploader,
        std::function<bool()> flowRunningGuard,
        bool aboutMode = false,
        bool remoteBrowseAllowed = false,
        QWidget* parent = nullptr);

protected:
    // 关界面时若正在上传：弹「后台继续 / 停止并清理服务器半截文件」，由用户决定。
    void closeEvent(QCloseEvent* event) override;

private:
    void BuildUi();
    void LoadConfigToUi();
    void SaveConfigFromUi();
    QString UpdateChannel() const;   // neutral / brand（按 BrandingConfig 应用名）
    void CheckForUpdate();
    void OnManifestReply(QNetworkReply* reply);
    void StartDownload();
    void OnDownloadFinished(QNetworkReply* reply);
    void InstallDownloadedPackage();
    void AppendLog(const QString& text);
    static int CompareVersions(const QString& lhs, const QString& rhs);  // <0/0/>0

    ScanDataUploader* m_uploader = nullptr;
    std::function<bool()> m_flowRunningGuard;
    bool m_aboutMode = false;
    bool m_remoteBrowseAllowed = false;
    QNetworkAccessManager* m_network = nullptr;

    // 升级区
    QLabel* m_currentVersionLabel = nullptr;
    QLabel* m_latestVersionLabel = nullptr;
    QPlainTextEdit* m_updateNotes = nullptr;
    QPushButton* m_checkUpdateBtn = nullptr;
    QPushButton* m_downloadInstallBtn = nullptr;
    QProgressBar* m_downloadProgress = nullptr;

    // 上传区
    QCheckBox* m_autoUploadCheck = nullptr;
    QListWidget* m_pendingListWidget = nullptr;
    QPushButton* m_uploadNowBtn = nullptr;
    QPushButton* m_uploadPickBtn = nullptr;

    // 远程数据区（admin）：FTP 阻塞操作跑一次性后台线程，回调经 QPointer 防悬空。
    QComboBox* m_remoteDeviceCombo = nullptr;
    QListWidget* m_remoteFileList = nullptr;
    QPushButton* m_remoteRefreshBtn = nullptr;
    QPushButton* m_remoteDownloadBtn = nullptr;
    bool m_remoteBusy = false;
    void RefreshRemoteDevices();
    void RefreshRemoteFiles();
    void DownloadSelectedRemoteFiles();
    void SetRemoteBusy(bool busy);

    // 配置区
    QLineEdit* m_updateBaseUrlEdit = nullptr;
    QLineEdit* m_ftpHostEdit = nullptr;
    QLineEdit* m_ftpPortEdit = nullptr;
    QLineEdit* m_ftpUserEdit = nullptr;
    QLineEdit* m_ftpPasswordEdit = nullptr;
    QLineEdit* m_deviceNameEdit = nullptr;

    QPlainTextEdit* m_logText = nullptr;

    // 本次会话发现的新版信息。增量补丁（patch 字段，仅含变更文件的 zip，通常只有主程序
    // 约 2MB）优先于全量安装包（80+MB）；补丁安装 = 退出后解压覆盖安装目录再重启。
    QString m_remoteVersion;
    QString m_remoteFile;
    QString m_remoteSha256;
    QString m_remotePatchFile;
    QString m_remotePatchSha256;
    qint64 m_remotePatchSize = 0;
    bool m_usePatch = false;
    QString m_downloadedPath;
    bool m_downloading = false;
};
