#pragma once

#include "RobotModelCatalogStore.h"
#include "RobotModelRemoteCatalog.h"

#include <QHash>
#include <QImage>
#include <QDialog>
#include <QList>
#include <QStringList>

#include <atomic>
#include <functional>
#include <thread>

class QLabel;
class QListWidget;
class QProgressBar;
class QPushButton;
class QTableWidget;
class QTextEdit;

// 机器人型号资产库管理。
//
// 本窗口只登记经过适配器完整验证的型号 STEP 与碰撞简模，不选择控制单元、
// 不写 RobotModelId，也不会根据 RobotA/RobotB 等名称猜测型号绑定。
class RobotModelManagerDialog final : public QDialog
{
public:
    explicit RobotModelManagerDialog(QWidget* parent = nullptr);
    ~RobotModelManagerDialog() override;

    void reject() override;

private:
    void BootstrapLegacyCatalog();
    void RefreshCatalog(const QString& preferredModelId = QString());
    void RefreshSelectionDetails();
    void ImportVerifiedModel();
    void SetBusy(bool busy);
    void SetStatus(const QString& text, bool error = false);
    void RefreshLocalPreview();

    void RefreshServerCatalog();
    void RefreshServerSelectionDetails();
    void UploadSelectedModel();
    void DownloadSelectedServerModel();
    void CancelRemoteTransfer();
    bool StartRemoteWorker(std::function<void()> work);
    void FinishRemoteWorker();
    void SetRemoteBusy(bool busy);
    void BeginTransferUi(
        const QString& action,
        const QStringList& itemNames);
    void UpdateTransferUi(
        int currentIndex,
        int completedItems,
        qulonglong transferredBytes,
        qulonglong totalBytes,
        double bytesPerSec,
        int etaSeconds,
        const QString& phase,
        const QString& itemState);
    void FinishTransferUi(
        int completedItems,
        bool stoppedEarly,
        const QString& summary);
    void RefreshTransferQueueUi();
    void ApplyServerCatalog(
        const RobotModelRemoteCatalog::Catalog& catalog,
        const QHash<QString, QImage>& previews,
        const QStringList& previewWarnings);

    bool m_busy = false;
    QList<RobotModelCatalogStore::ModelRecord> m_models;
    QTableWidget* m_modelTable = nullptr;
    QTextEdit* m_detailsEdit = nullptr;
    QLabel* m_statusLabel = nullptr;
    QPushButton* m_importButton = nullptr;
    QPushButton* m_refreshButton = nullptr;
    QLabel* m_localPreviewLabel = nullptr;

    RobotModelRemoteCatalog::Catalog m_serverCatalog;
    QHash<QString, QImage> m_serverPreviews;
    QTableWidget* m_serverTable = nullptr;
    QLabel* m_serverPreviewLabel = nullptr;
    QTextEdit* m_serverDetailsEdit = nullptr;
    QLabel* m_serverStatusLabel = nullptr;
    QPushButton* m_serverRefreshButton = nullptr;
    QPushButton* m_serverUploadButton = nullptr;
    QPushButton* m_serverDownloadButton = nullptr;
    QPushButton* m_serverCancelButton = nullptr;
    QLabel* m_transferStateLabel = nullptr;
    QLabel* m_transferCurrentLabel = nullptr;
    QLabel* m_transferDetailLabel = nullptr;
    QLabel* m_transferQueueLabel = nullptr;
    QProgressBar* m_transferProgressBar = nullptr;
    QListWidget* m_transferQueueList = nullptr;
    QString m_transferAction;
    QStringList m_transferNames;
    QStringList m_transferStates;

    std::thread m_remoteWorker;
    std::atomic<bool> m_remoteBusy{ false };
    std::atomic<bool> m_remoteCancel{ false };
    // 0=不可取消/无任务，1=可取消，2=已请求取消。单个原子状态关闭
    // “取消点击”和下载后本地原子导入之间的竞态窗口。
    std::atomic<int> m_remoteCancelState{ 0 };
};
