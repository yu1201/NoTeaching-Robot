#include "RobotModelManagerDialog.h"

#include "AppPaths.h"
#include "Const.h"
#include "FTPClient.h"
#include "OnlineServicesConfig.h"
#include "RobotCadAssemblyLoader.h"
#include "RobotCollisionEnvelopeStore.h"
#include "RobotLog.h"
#include "RobotModelOriginalPreview.h"
#include "TheoreticalRobotModelStore.h"
#include "WindowStyleHelper.h"

#include <Eigen/Core>

#include <QAbstractItemView>
#include <QCloseEvent>
#include <QDateTime>
#include <QDialogButtonBox>
#include <QDir>
#include <QEventLoop>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QColor>
#include <QGroupBox>
#include <QHeaderView>
#include <QIcon>
#include <QLabel>
#include <QListWidget>
#include <QMessageBox>
#include <QPixmap>
#include <QProgressDialog>
#include <QProgressBar>
#include <QPushButton>
#include <QSplitter>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QTabWidget>
#include <QTextEdit>
#include <QThread>
#include <QTimer>
#include <QTemporaryDir>
#include <QUuid>
#include <QVBoxLayout>
#include <QHBoxLayout>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <exception>
#include <limits>
#include <map>
#include <vector>

namespace
{
constexpr char kModelId[] = "step.sa10-2000h";
constexpr char kAdapterId[] = "step.sa10-2000h.scene-v1";
constexpr char kVerifiedSa10AssemblySha256[] =
    "f8299f6deabc7b6f6ae799592f1f93e2366311d552bf91ac431eccec14bfcaa8";

class NonClosableProgressDialog final : public QProgressDialog
{
public:
    using QProgressDialog::QProgressDialog;

    void reject() override
    {
        // STEP 解析不能安全中断；Esc 不应放开父窗口或制造半完成状态。
    }

protected:
    void closeEvent(QCloseEvent* event) override
    {
        event->ignore();
    }
};

QString ShortSha(const QString& sha256)
{
    return sha256.size() == 64
        ? sha256.left(12) + QStringLiteral("…")
        : QStringLiteral("无效");
}

QString SizeText(qint64 bytes)
{
    if (bytes < 0) return QStringLiteral("未知");
    const double mib = static_cast<double>(bytes) / (1024.0 * 1024.0);
    return mib >= 0.1
        ? QStringLiteral("%1 MiB").arg(mib, 0, 'f', 2)
        : QStringLiteral("%1 字节").arg(bytes);
}

QString MarginText(qint64 micrometres)
{
    if (micrometres < 0) return QStringLiteral("未知");
    return QStringLiteral("%1 mm").arg(
        static_cast<double>(micrometres) / 1000.0, 0, 'f', 3);
}

QString RobotTypeText(int robotType)
{
    if (robotType == ROBOT_TYPE_STEP) return QStringLiteral("新时达 / STEP");
    if (robotType == ROBOT_TYPE_FANUC) return QStringLiteral("FANUC");
    return QStringLiteral("类型 %1").arg(robotType);
}

bool ValidateVerifiedSa10(
    const RobotCadAssemblyLoader::Result& loaded,
    QString& error)
{
    if (loaded.statistics.sourceSha256
        != QString::fromLatin1(kVerifiedSa10AssemblySha256))
    {
        error = QStringLiteral(
            "当前型号库只允许导入已核验的新时达 SA10-2000H 总装 STEP；"
            "不能把其它 J0-J6 模型登记为该型号。");
        return false;
    }
    if (!loaded.base.valid || !loaded.base.j0BoundsMm.valid
        || !loaded.statistics.assemblyBoundsMm.valid
        || (loaded.base.sourceUp - Eigen::Vector3d::UnitY()).norm() > 1.0e-12
        || loaded.statistics.jointComponentCount != 7
        || loaded.statistics.includedComponentCount != 7
        || loaded.statistics.includedPipelineCount != 0)
    {
        error = QStringLiteral(
            "已核验 SA10 总装的 J0-J6、源坐标方向、安装面或装配边界语义不完整。");
        return false;
    }
    std::array<bool, 7> joints{};
    for (const auto& component : loaded.statistics.components)
    {
        if (component.jointIndex < 0 || component.jointIndex > 6) continue;
        const size_t index = static_cast<size_t>(component.jointIndex);
        if (joints[index] || !component.included || !component.boundsMm.valid
            || component.faceCount <= 0)
        {
            error = QStringLiteral("机器人组件 J%1 重复或缺少有效 B-Rep 边界。")
                .arg(component.jointIndex);
            return false;
        }
        joints[index] = true;
    }
    for (int index = 0; index <= 6; ++index)
    {
        if (!joints[static_cast<size_t>(index)])
        {
            error = QStringLiteral("机器人总装缺少唯一有效的 J%1 组件。").arg(index);
            return false;
        }
    }
    if (loaded.statistics.detailedPresentationBuilt
        || loaded.statistics.displayTriangulationPrepared
        || loaded.statistics.displayBlockCount != 0
        || loaded.assemblyShape || loaded.j0Shape || !loaded.displayBlocks.empty())
    {
        error = QStringLiteral("型号库导入意外生成了详细机器人 B-Rep 显示资产。");
        return false;
    }
    error.clear();
    return true;
}

void ConfigureReadOnlyItem(QTableWidgetItem* item)
{
    if (item == nullptr) return;
    item->setFlags(item->flags() & ~Qt::ItemIsEditable);
}

QString HumanBytes(double bytes)
{
    if (!std::isfinite(bytes) || bytes < 0.0)
        return QStringLiteral("未知");
    static const std::array<const char*, 5> units = {
        "B", "KiB", "MiB", "GiB", "TiB"
    };
    int unit = 0;
    while (bytes >= 1024.0 && unit + 1 < static_cast<int>(units.size()))
    {
        bytes /= 1024.0;
        ++unit;
    }
    return QStringLiteral("%1 %2")
        .arg(bytes, 0, unit == 0 ? 'f' : 'f', unit == 0 ? 0 : 1)
        .arg(QString::fromLatin1(units.at(static_cast<size_t>(unit))));
}

std::string OnlineServicesLogPath()
{
    return QDir::toNativeSeparators(AppPaths::WritableChildPath(
        QStringLiteral("Log"), QStringLiteral("OnlineServices.log")))
        .toLocal8Bit()
        .toStdString();
}

std::string Utf8Path(const QString& value)
{
    return value.toUtf8().toStdString();
}

QString JoinRemote(const QString& directory, const QString& name)
{
    return directory.endsWith(QLatin1Char('/'))
        ? directory + name
        : directory + QLatin1Char('/') + name;
}

struct RemoteFtpConfig
{
    std::string host;
    int port = 21;
    std::string user;
    std::string password;
};

bool CurrentRemoteFtpConfig(RemoteFtpConfig& config, QString& error)
{
    config = RemoteFtpConfig();
    error.clear();
    const QString user = OnlineServicesConfig::FtpUser().trimmed();
    const QString password = OnlineServicesConfig::FtpPassword();
    const auto access = OnlineServicesConfig::AccessLevelForAccount(user);
    if (!OnlineServicesConfig::HasFtpAccess(access))
    {
        error = QStringLiteral(
            "当前服务器账号“%1”只有上传权限，不能读取模型列表或下载模型；"
            "请先在在线服务中登录 FTP 权限或全权限账号。").arg(user);
        return false;
    }
    if (password.isEmpty())
    {
        error = QStringLiteral(
            "当前没有已验证的服务器 FTP 密码，请先进入在线服务完成账号验证。");
        return false;
    }
    config.host = OnlineServicesConfig::FtpHost().toUtf8().toStdString();
    config.port = OnlineServicesConfig::FtpPort();
    config.user = user.toUtf8().toStdString();
    config.password = password.toUtf8().toStdString();
    return true;
}

using RemoteFileMap = QHash<QString, qulonglong>;

bool ListRemoteDirectory(
    FtpClient& ftp,
    const QString& directory,
    RemoteFileMap& files,
    std::atomic<bool>* cancel,
    QString& error)
{
    files.clear();
    std::vector<FtpRemoteFileInfo> listed;
    if (!ftp.listFiles(Utf8Path(directory), listed, cancel, 10000))
    {
        error = QStringLiteral("无法读取服务器目录：%1").arg(directory);
        return false;
    }
    for (const FtpRemoteFileInfo& item : listed)
    {
        if (item.isDirectory) continue;
        const QString name = QString::fromUtf8(item.name);
        if (name.isEmpty() || files.contains(name))
        {
            error = QStringLiteral("服务器目录包含空名称或重复文件：%1")
                .arg(directory);
            return false;
        }
        files.insert(name, static_cast<qulonglong>(item.size));
    }
    return true;
}

bool ReadFileBounded(
    const QString& path,
    qint64 maximumBytes,
    QByteArray& bytes,
    QString& error)
{
    bytes.clear();
    const QFileInfo info(path);
    if (!info.exists() || !info.isFile()
#ifdef Q_OS_WIN
        || info.isSymLink() || info.isJunction()
#else
        || info.isSymLink()
#endif
        || info.size() <= 0 || info.size() > maximumBytes)
    {
        error = QStringLiteral("下载文件不是受控范围内的普通文件：%1").arg(path);
        return false;
    }
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly) || file.size() != info.size())
    {
        error = QStringLiteral("无法读取下载文件：%1").arg(file.errorString());
        return false;
    }
    bytes = file.readAll();
    file.close();
    if (bytes.size() != info.size())
    {
        bytes.clear();
        error = QStringLiteral("下载文件读取长度发生变化。");
        return false;
    }
    return true;
}

bool WriteExclusiveFile(
    const QString& path,
    const QByteArray& bytes,
    QString& error)
{
    if (bytes.isEmpty())
    {
        error = QStringLiteral("拒绝写入空的模型传输文件。");
        return false;
    }
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::NewOnly)
        || file.write(bytes) != bytes.size()
        || !file.flush())
    {
        file.close();
        QFile::remove(path);
        error = QStringLiteral("无法创建模型传输临时文件：%1")
            .arg(file.errorString());
        return false;
    }
    file.close();
    return true;
}

bool ReadLatestRemoteCatalog(
    FtpClient& ftp,
    const QString& temporaryRoot,
    std::atomic<bool>* cancel,
    RobotModelRemoteCatalog::Catalog& catalog,
    QString& catalogFileName,
    QString& error,
    const std::function<void(
        qulonglong received, qulonglong total)>& progress = {})
{
    catalog = RobotModelRemoteCatalog::Catalog();
    catalogFileName.clear();
    RemoteFileMap files;
    if (!ListRemoteDirectory(
            ftp, RobotModelRemoteCatalog::RemoteCatalogDirectory(),
            files, cancel, error))
    {
        return false;
    }
    qint64 newestRevision = 0;
    QString newestPayload;
    qulonglong newestSize = 0;
    for (auto it = files.cbegin(); it != files.cend(); ++it)
    {
        qint64 revision = 0;
        QString payload;
        if (!RobotModelRemoteCatalog::ParseCatalogFileName(
                it.key(), revision, payload))
        {
            continue;
        }
        if (it.value() == 0
            || it.value()
                > static_cast<qulonglong>(
                    RobotModelRemoteCatalog::MaximumCatalogBytes))
        {
            error = QStringLiteral("服务器模型清单文件大小越界：%1")
                .arg(it.key());
            return false;
        }
        if (revision > newestRevision)
        {
            newestRevision = revision;
            newestPayload = payload;
            newestSize = it.value();
            catalogFileName = it.key();
        }
        else if (revision == newestRevision && newestRevision > 0
                 && payload != newestPayload)
        {
            error = QStringLiteral(
                "服务器机器人模型库存在同 revision 的冲突清单，已停止读取。");
            return false;
        }
    }
    if (catalogFileName.isEmpty())
    {
        return true;
    }

    const QString localCatalog = QDir(temporaryRoot).filePath(
        QStringLiteral("latest-catalog.json"));
    const QString remoteCatalog = JoinRemote(
        RobotModelRemoteCatalog::RemoteCatalogDirectory(),
        catalogFileName);
    QFile::remove(localCatalog);
    if (!ftp.downloadFileBounded(
            Utf8Path(remoteCatalog),
            QDir::toNativeSeparators(localCatalog).toLocal8Bit().toStdString(),
            newestSize,
            static_cast<unsigned long long>(
                RobotModelRemoteCatalog::MaximumCatalogBytes),
            cancel,
            [progress](unsigned long long received, unsigned long long total)
            {
                if (progress) progress(received, total);
            }))
    {
        error = QStringLiteral("下载服务器机器人模型清单失败：%1")
            .arg(catalogFileName);
        return false;
    }
    QByteArray bytes;
    if (!ReadFileBounded(
            localCatalog, RobotModelRemoteCatalog::MaximumCatalogBytes,
            bytes, error)
        || !RobotModelRemoteCatalog::Parse(bytes, catalog, error)
        || catalog.revisionUtcMs != newestRevision
        || catalog.payloadSha256 != newestPayload)
    {
        if (error.isEmpty())
            error = QStringLiteral("服务器机器人模型清单文件名与内容身份不一致。");
        return false;
    }
    return true;
}

bool ResolveRemoteFile(
    const RemoteFileMap& files,
    const QString& name,
    qint64 expectedBytes,
    const QString& label,
    QString& error)
{
    const auto found = files.constFind(name);
    if (found == files.cend())
    {
        error = QStringLiteral("服务器缺少%1：%2").arg(label, name);
        return false;
    }
    if (expectedBytes <= 0
        || found.value() != static_cast<qulonglong>(expectedBytes))
    {
        error = QStringLiteral("服务器%1大小与清单不一致：%2")
            .arg(label, name);
        return false;
    }
    return true;
}
}

RobotModelManagerDialog::RobotModelManagerDialog(QWidget* parent)
    : QDialog(parent)
{
    setWindowTitle(QStringLiteral("机器人模型库管理"));
    setModal(true);
    ApplyUnifiedWindowChrome(this);
    ResizeWindowForAvailableGeometry(this, QSize(1120, 720), 0.92, 0.90);

    QVBoxLayout* root = new QVBoxLayout(this);
    root->setContentsMargins(12, 12, 12, 12);
    root->setSpacing(10);

    QLabel* boundary = new QLabel(QStringLiteral(
        "这里只管理经过验证的机器人型号资产，不绑定具体 RobotA/RobotB，也不修改控制单元。"
        "当前仅支持型号 step.sa10-2000h（新时达 SA10-2000H）；控制单元中的机器人型号是流程唯一选择来源。"));
    boundary->setWordWrap(true);
    boundary->setStyleSheet(QStringLiteral(
        "QLabel { background:#173247; color:#b3e5fc; border:1px solid #39789d; "
        "padding:8px; border-radius:4px; }"));
    root->addWidget(boundary);

    QTabWidget* tabs = new QTabWidget(this);
    tabs->setDocumentMode(true);

    QWidget* localPage = new QWidget(tabs);
    QVBoxLayout* localLayout = new QVBoxLayout(localPage);
    localLayout->setContentsMargins(8, 8, 8, 8);
    localLayout->setSpacing(8);
    QHBoxLayout* actions = new QHBoxLayout();
    m_importButton = new QPushButton(QStringLiteral("导入 SA10 总装并生成简模"));
    m_refreshButton = new QPushButton(QStringLiteral("刷新本地型号库"));
    m_importButton->setObjectName(QStringLiteral("RobotModelCatalogImportButton"));
    m_refreshButton->setObjectName(QStringLiteral("RobotModelCatalogRefreshButton"));
    actions->addWidget(m_importButton);
    actions->addWidget(m_refreshButton);
    actions->addStretch(1);
    localLayout->addLayout(actions);

    m_modelTable = new QTableWidget(0, 6, localPage);
    m_modelTable->setObjectName(QStringLiteral("RobotModelCatalogTable"));
    m_modelTable->setHorizontalHeaderLabels({
        QStringLiteral("型号ID"),
        QStringLiteral("显示名称"),
        QStringLiteral("驱动类型"),
        QStringLiteral("STEP SHA"),
        QStringLiteral("碰撞简模"),
        QStringLiteral("登记时间")
    });
    m_modelTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_modelTable->setSelectionMode(QAbstractItemView::SingleSelection);
    m_modelTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_modelTable->verticalHeader()->setVisible(false);
    m_modelTable->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    m_modelTable->horizontalHeader()->setStretchLastSection(true);
    m_modelTable->setMinimumHeight(220);
    localLayout->addWidget(m_modelTable, 1);

    QSplitter* localDetailsSplitter = new QSplitter(Qt::Horizontal, localPage);
    QGroupBox* detailsGroup = new QGroupBox(
        QStringLiteral("选中型号详情与完整性状态"), localDetailsSplitter);
    QVBoxLayout* detailsLayout = new QVBoxLayout(detailsGroup);
    m_detailsEdit = new QTextEdit();
    m_detailsEdit->setObjectName(QStringLiteral("RobotModelCatalogDetails"));
    m_detailsEdit->setReadOnly(true);
    m_detailsEdit->setMinimumHeight(140);
    m_detailsEdit->setPlaceholderText(QStringLiteral("型号库为空。"));
    detailsLayout->addWidget(m_detailsEdit);

    QGroupBox* localPreviewGroup = new QGroupBox(
        QStringLiteral("碰撞简模预览"), localDetailsSplitter);
    QVBoxLayout* localPreviewLayout = new QVBoxLayout(localPreviewGroup);
    m_localPreviewLabel = new QLabel(
        QStringLiteral("选择本地型号后显示预览"), localPreviewGroup);
    m_localPreviewLabel->setAlignment(Qt::AlignCenter);
    m_localPreviewLabel->setMinimumSize(300, 180);
    m_localPreviewLabel->setStyleSheet(QStringLiteral(
        "QLabel { background:#081822; color:#78909c; border:1px solid #29485a; }"));
    localPreviewLayout->addWidget(m_localPreviewLabel);
    localDetailsSplitter->addWidget(detailsGroup);
    localDetailsSplitter->addWidget(localPreviewGroup);
    localDetailsSplitter->setStretchFactor(0, 2);
    localDetailsSplitter->setStretchFactor(1, 1);
    localLayout->addWidget(localDetailsSplitter);

    m_statusLabel = new QLabel(QStringLiteral("正在读取本地型号库…"), localPage);
    m_statusLabel->setObjectName(QStringLiteral("RobotModelCatalogStatus"));
    m_statusLabel->setWordWrap(true);
    m_statusLabel->setMinimumHeight(36);
    localLayout->addWidget(m_statusLabel);
    tabs->addTab(localPage, QStringLiteral("本地模型"));

    QWidget* serverPage = new QWidget(tabs);
    QVBoxLayout* serverLayout = new QVBoxLayout(serverPage);
    serverLayout->setContentsMargins(8, 8, 8, 8);
    serverLayout->setSpacing(8);
    QHBoxLayout* serverActions = new QHBoxLayout();
    m_serverRefreshButton = new QPushButton(
        QStringLiteral("刷新服务器模型列表"), serverPage);
    m_serverUploadButton = new QPushButton(
        QStringLiteral("上传选中本地模型"), serverPage);
    m_serverDownloadButton = new QPushButton(
        QStringLiteral("下载选中服务器模型"), serverPage);
    m_serverCancelButton = new QPushButton(
        QStringLiteral("取消传输"), serverPage);
    m_serverCancelButton->setEnabled(false);
    serverActions->addWidget(m_serverRefreshButton);
    serverActions->addWidget(m_serverUploadButton);
    serverActions->addWidget(m_serverDownloadButton);
    serverActions->addWidget(m_serverCancelButton);
    serverActions->addStretch(1);
    serverLayout->addLayout(serverActions);

    QSplitter* serverSplitter = new QSplitter(Qt::Horizontal, serverPage);
    m_serverTable = new QTableWidget(0, 7, serverSplitter);
    m_serverTable->setObjectName(QStringLiteral("RobotModelRemoteTable"));
    m_serverTable->setHorizontalHeaderLabels({
        QStringLiteral("预览"),
        QStringLiteral("型号ID"),
        QStringLiteral("显示名称"),
        QStringLiteral("驱动类型"),
        QStringLiteral("STEP大小"),
        QStringLiteral("简模型号"),
        QStringLiteral("服务器发布时间")
    });
    m_serverTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_serverTable->setSelectionMode(QAbstractItemView::SingleSelection);
    m_serverTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_serverTable->verticalHeader()->setVisible(false);
    m_serverTable->setIconSize(QSize(96, 64));
    m_serverTable->verticalHeader()->setDefaultSectionSize(72);
    m_serverTable->horizontalHeader()->setSectionResizeMode(
        QHeaderView::ResizeToContents);
    m_serverTable->horizontalHeader()->setStretchLastSection(true);

    QWidget* serverPreviewPane = new QWidget(serverSplitter);
    QVBoxLayout* serverPreviewLayout = new QVBoxLayout(serverPreviewPane);
    serverPreviewLayout->setContentsMargins(4, 4, 4, 4);
    QLabel* serverPreviewTitle = new QLabel(
        QStringLiteral("服务器模型预览"), serverPreviewPane);
    serverPreviewTitle->setStyleSheet(QStringLiteral(
        "QLabel { color:#b3e5fc; font-weight:600; }"));
    m_serverPreviewLabel = new QLabel(
        QStringLiteral("选择服务器模型后显示预览"), serverPreviewPane);
    m_serverPreviewLabel->setAlignment(Qt::AlignCenter);
    m_serverPreviewLabel->setMinimumSize(320, 215);
    m_serverPreviewLabel->setStyleSheet(QStringLiteral(
        "QLabel { background:#081822; color:#78909c; border:1px solid #29485a; }"));
    m_serverDetailsEdit = new QTextEdit(serverPreviewPane);
    m_serverDetailsEdit->setReadOnly(true);
    m_serverDetailsEdit->setMinimumHeight(125);
    m_serverDetailsEdit->setPlaceholderText(QStringLiteral("服务器模型库为空。"));
    serverPreviewLayout->addWidget(serverPreviewTitle);
    serverPreviewLayout->addWidget(m_serverPreviewLabel);
    serverPreviewLayout->addWidget(m_serverDetailsEdit, 1);
    serverSplitter->addWidget(m_serverTable);
    serverSplitter->addWidget(serverPreviewPane);
    serverSplitter->setStretchFactor(0, 2);
    serverSplitter->setStretchFactor(1, 1);
    serverLayout->addWidget(serverSplitter, 1);

    QGroupBox* transferGroup = new QGroupBox(
        QStringLiteral("模型传输队列与进度"), serverPage);
    QVBoxLayout* transferLayout = new QVBoxLayout(transferGroup);
    m_transferStateLabel = new QLabel(QStringLiteral("当前没有模型传输任务"));
    m_transferCurrentLabel = new QLabel(QStringLiteral("当前文件：无"));
    m_transferProgressBar = new QProgressBar(transferGroup);
    m_transferProgressBar->setRange(0, 100);
    m_transferProgressBar->setValue(0);
    m_transferProgressBar->setFormat(QStringLiteral("0%"));
    m_transferDetailLabel = new QLabel(
        QStringLiteral("上传和下载会显示文件队列、实时速度与预计剩余时间。"));
    m_transferDetailLabel->setWordWrap(true);
    m_transferQueueLabel = new QLabel(QStringLiteral("传输队列：0 项"));
    m_transferQueueList = new QListWidget(transferGroup);
    m_transferQueueList->setMaximumHeight(100);
    m_transferQueueList->addItem(QStringLiteral("当前没有模型传输任务"));
    transferLayout->addWidget(m_transferStateLabel);
    transferLayout->addWidget(m_transferCurrentLabel);
    transferLayout->addWidget(m_transferProgressBar);
    transferLayout->addWidget(m_transferDetailLabel);
    transferLayout->addWidget(m_transferQueueLabel);
    transferLayout->addWidget(m_transferQueueList);
    serverLayout->addWidget(transferGroup);

    m_serverStatusLabel = new QLabel(
        QStringLiteral("尚未读取服务器模型列表。"), serverPage);
    m_serverStatusLabel->setWordWrap(true);
    m_serverStatusLabel->setMinimumHeight(34);
    m_serverStatusLabel->setStyleSheet(QStringLiteral(
        "QLabel { color:#b3e5fc; padding:5px; }"));
    serverLayout->addWidget(m_serverStatusLabel);
    tabs->addTab(serverPage, QStringLiteral("服务器模型"));
    root->addWidget(tabs, 1);

    QDialogButtonBox* buttons = new QDialogButtonBox(QDialogButtonBox::Close);
    connect(buttons, &QDialogButtonBox::rejected, this, [this]() { reject(); });
    root->addWidget(buttons);

    connect(m_importButton, &QPushButton::clicked,
        this, [this]() { ImportVerifiedModel(); });
    connect(m_refreshButton, &QPushButton::clicked,
        this, [this]() { BootstrapLegacyCatalog(); });
    connect(m_modelTable, &QTableWidget::currentCellChanged,
        this, [this](int, int, int, int) { RefreshSelectionDetails(); });
    connect(m_serverRefreshButton, &QPushButton::clicked,
        this, [this]() { RefreshServerCatalog(); });
    connect(m_serverUploadButton, &QPushButton::clicked,
        this, [this]() { UploadSelectedModel(); });
    connect(m_serverDownloadButton, &QPushButton::clicked,
        this, [this]() { DownloadSelectedServerModel(); });
    connect(m_serverCancelButton, &QPushButton::clicked,
        this, [this]() { CancelRemoteTransfer(); });
    connect(m_serverTable, &QTableWidget::currentCellChanged,
        this, [this](int, int, int, int) { RefreshServerSelectionDetails(); });

    QTimer::singleShot(0, this, [this]() { BootstrapLegacyCatalog(); });
    QTimer::singleShot(200, this, [this]() { RefreshServerCatalog(); });
}

RobotModelManagerDialog::~RobotModelManagerDialog()
{
    m_remoteCancelState.store(2);
    m_remoteCancel.store(true);
    if (m_remoteWorker.joinable()) m_remoteWorker.join();
}

void RobotModelManagerDialog::reject()
{
    if (m_busy || m_remoteBusy.load()) return;
    QDialog::reject();
}

void RobotModelManagerDialog::SetBusy(bool busy)
{
    m_busy = busy;
    if (m_importButton != nullptr) m_importButton->setEnabled(!busy);
    if (m_refreshButton != nullptr) m_refreshButton->setEnabled(!busy);
    if (m_modelTable != nullptr) m_modelTable->setEnabled(!busy);
    if (m_serverUploadButton != nullptr)
        m_serverUploadButton->setEnabled(!busy && !m_remoteBusy.load());
}

void RobotModelManagerDialog::SetStatus(const QString& text, bool error)
{
    if (m_statusLabel == nullptr) return;
    m_statusLabel->setText(text);
    m_statusLabel->setStyleSheet(error
        ? QStringLiteral("QLabel { color:#ff8a80; padding:5px; }")
        : QStringLiteral("QLabel { color:#a5d6a7; padding:5px; }"));
}

void RobotModelManagerDialog::BootstrapLegacyCatalog()
{
    if (m_busy) return;
    SetBusy(true);
    SetStatus(QStringLiteral("正在安全检查旧版当前 SA10 资产并刷新型号库…"));

    RobotModelCatalogStore::ModelRecord bootstrapModel;
    bool availableInCatalog = false;
    bool ok = false;
    QString workerError;
    QEventLoop waitLoop;
    QThread* worker = QThread::create([&]()
        {
            try
            {
                ok = RobotModelCatalogStore::BootstrapVerifiedSa10FromLegacy(
                    bootstrapModel, availableInCatalog, workerError);
            }
            catch (const std::exception& exception)
            {
                workerError = QStringLiteral("刷新机器人型号库线程发生异常：%1")
                    .arg(QString::fromLocal8Bit(exception.what()).simplified());
                ok = false;
            }
            catch (...)
            {
                workerError = QStringLiteral("刷新机器人型号库线程发生未知异常。");
                ok = false;
            }
        });
    NonClosableProgressDialog progress(
        QStringLiteral("正在检查旧版 SA10 资产并读取型号库…"),
        QString(), 0, 0, this);
    progress.setWindowTitle(QStringLiteral("刷新机器人型号库"));
    progress.setWindowModality(Qt::WindowModal);
    progress.setCancelButton(nullptr);
    progress.setMinimumDuration(300);
    progress.setAutoClose(false);
    progress.setAutoReset(false);
    progress.setWindowFlag(Qt::WindowCloseButtonHint, false);
    QTimer progressDelay;
    progressDelay.setSingleShot(true);
    connect(&progressDelay, &QTimer::timeout, &progress, &QProgressDialog::show);
    connect(worker, &QThread::finished, &waitLoop, &QEventLoop::quit);
    connect(worker, &QThread::finished, &progress, &QProgressDialog::accept);
    progressDelay.start(300);
    worker->start();
    waitLoop.exec();
    progressDelay.stop();
    worker->wait();
    delete worker;
    progress.accept();
    SetBusy(false);

    if (!ok)
    {
        m_models.clear();
        m_modelTable->setRowCount(0);
        m_detailsEdit->clear();
        SetStatus(QStringLiteral("机器人型号库不可用：%1").arg(workerError), true);
        QMessageBox::warning(this, QStringLiteral("刷新机器人型号库"), workerError);
        return;
    }
    RefreshCatalog(availableInCatalog ? bootstrapModel.modelId : QString());
    if (availableInCatalog)
    {
        SetStatus(QStringLiteral(
            "已确认旧版当前 SA10 资产安全登记为型号 %1；未绑定任何具体机器人。")
            .arg(bootstrapModel.modelId));
    }
}

void RobotModelManagerDialog::RefreshCatalog(const QString& preferredModelId)
{
    if (m_busy) return;
    QString previousId = preferredModelId;
    if (previousId.isEmpty() && m_modelTable->currentRow() >= 0)
    {
        QTableWidgetItem* currentIdItem =
            m_modelTable->item(m_modelTable->currentRow(), 0);
        if (currentIdItem != nullptr)
            previousId = currentIdItem->data(Qt::UserRole).toString();
    }

    QList<RobotModelCatalogStore::ModelRecord> models;
    QString error;
    if (!RobotModelCatalogStore::ListModels(models, error))
    {
        m_models.clear();
        m_modelTable->setRowCount(0);
        m_detailsEdit->clear();
        SetStatus(QStringLiteral("读取机器人型号库失败：%1").arg(error), true);
        return;
    }
    std::sort(models.begin(), models.end(),
        [](const auto& left, const auto& right)
        {
            const int displayOrder = QString::compare(
                left.displayName, right.displayName, Qt::CaseInsensitive);
            return displayOrder != 0 ? displayOrder < 0 : left.modelId < right.modelId;
        });
    m_models = models;
    m_modelTable->setRowCount(m_models.size());

    int selectedRow = -1;
    int unavailableCount = 0;
    QString firstUnavailableReason;
    for (qsizetype index = 0; index < m_models.size(); ++index)
    {
        const auto& model = m_models.at(index);
        const int row = static_cast<int>(index);
        RobotModelCatalogStore::Eligibility eligibility;
        QString eligibilityError;
        const bool eligibilityRead = RobotModelCatalogStore::ResolveModelEligibility(
            model.modelId, model.sourceRobotType, eligibility, eligibilityError);
        const bool available = eligibilityRead && eligibility.eligible;
        const QString state = available
            ? QStringLiteral("可用（%1）").arg(MarginText(model.collision.safetyMarginMicrometres))
            : eligibilityRead
                ? QStringLiteral("不可用：%1").arg(eligibility.reason)
                : QStringLiteral("校验失败：%1").arg(eligibilityError);
        if (!available)
        {
            ++unavailableCount;
            if (firstUnavailableReason.isEmpty()) firstUnavailableReason = state;
        }

        QList<QTableWidgetItem*> items = {
            new QTableWidgetItem(model.modelId),
            new QTableWidgetItem(model.displayName),
            new QTableWidgetItem(RobotTypeText(model.sourceRobotType)),
            new QTableWidgetItem(ShortSha(model.sourceStep.sha256)),
            new QTableWidgetItem(state),
            new QTableWidgetItem(model.registeredUtc)
        };
        items[0]->setData(Qt::UserRole, model.modelId);
        for (int column = 0; column < items.size(); ++column)
        {
            ConfigureReadOnlyItem(items.at(column));
            items.at(column)->setToolTip(column == 4 ? state : items.at(column)->text());
            if (column == 4)
                items.at(column)->setForeground(available ? QColor(102, 187, 106)
                                                         : QColor(239, 83, 80));
            m_modelTable->setItem(row, column, items.at(column));
        }
        if (model.modelId == previousId) selectedRow = row;
    }

    if (selectedRow < 0 && !m_models.isEmpty()) selectedRow = 0;
    if (selectedRow >= 0)
    {
        m_modelTable->selectRow(selectedRow);
        m_modelTable->setCurrentCell(selectedRow, 0);
    }
    else
    {
        m_detailsEdit->clear();
    }
    RefreshSelectionDetails();
    if (m_models.isEmpty())
    {
        SetStatus(QStringLiteral(
            "型号库为空。可导入已核验的 SA10-2000H 总装并生成简模。"));
    }
    else if (unavailableCount > 0)
    {
        SetStatus(QStringLiteral(
            "已读取 %1 个型号，其中 %2 个当前不可用；首个问题：%3")
            .arg(m_models.size()).arg(unavailableCount).arg(firstUnavailableReason), true);
    }
    else
    {
        SetStatus(QStringLiteral(
            "已读取 %1 个机器人型号；列表刷新不会解析大型 STEP。")
            .arg(m_models.size()));
    }
}

void RobotModelManagerDialog::RefreshSelectionDetails()
{
    if (m_detailsEdit == nullptr || m_modelTable == nullptr) return;
    const int row = m_modelTable->currentRow();
    if (row < 0 || row >= m_models.size())
    {
        m_detailsEdit->clear();
        if (m_localPreviewLabel != nullptr)
        {
            m_localPreviewLabel->setPixmap(QPixmap());
            m_localPreviewLabel->setText(QStringLiteral("没有可预览的本地型号"));
        }
        return;
    }
    const auto& model = m_models.at(row);
    QString state = m_modelTable->item(row, 4) != nullptr
        ? m_modelTable->item(row, 4)->text() : QStringLiteral("未知");
    const QString text = QStringLiteral(
        "型号ID：%1\n"
        "显示名称：%2\n"
        "场景适配器：%3\n"
        "驱动类型：%4\n"
        "源文件：%5（%6）\n"
        "STEP SHA-256：%7\n"
        "碰撞简模 profile：%8\n"
        "碰撞 payload SHA-256：%9\n"
        "安全余量：%10\n"
        "登记时间：%11\n"
        "当前状态：%12\n\n"
        "说明：碰撞简模为 J0-J6 静态保守 AABB，未标定且不代表实时关节姿态。")
        .arg(model.modelId)
        .arg(model.displayName)
        .arg(model.adapterId)
        .arg(RobotTypeText(model.sourceRobotType))
        .arg(model.sourceStep.originalDisplayName)
        .arg(SizeText(model.sourceStep.sizeBytes))
        .arg(model.sourceStep.sha256)
        .arg(model.collision.profileKeySha256)
        .arg(model.collisionPayloadSha256)
        .arg(MarginText(model.collision.safetyMarginMicrometres))
        .arg(model.registeredUtc)
        .arg(state);
    m_detailsEdit->setPlainText(text);
    RefreshLocalPreview();
}

void RobotModelManagerDialog::RefreshLocalPreview()
{
    if (m_localPreviewLabel == nullptr
        || m_modelTable == nullptr
        || m_modelTable->currentRow() < 0
        || m_modelTable->currentRow() >= m_models.size())
    {
        return;
    }
    const auto& model = m_models.at(m_modelTable->currentRow());
    RobotCollisionEnvelopeStore::EnvelopeSet envelope;
    QString error;
    if (!RobotCollisionEnvelopeStore::LoadAsset(
            model.collision, envelope, error))
    {
        m_localPreviewLabel->setPixmap(QPixmap());
        m_localPreviewLabel->setText(
            QStringLiteral("本地简模预览不可用\n%1").arg(error));
        return;
    }
    QImage image;
    if (!RobotModelRemoteCatalog::RenderPreview(
            envelope, model.displayName, image, error))
    {
        m_localPreviewLabel->setPixmap(QPixmap());
        m_localPreviewLabel->setText(
            QStringLiteral("生成本地预览失败\n%1").arg(error));
        return;
    }
    m_localPreviewLabel->setText(QString());
    m_localPreviewLabel->setPixmap(QPixmap::fromImage(image).scaled(
        m_localPreviewLabel->size(),
        Qt::KeepAspectRatio,
        Qt::SmoothTransformation));
}

void RobotModelManagerDialog::ImportVerifiedModel()
{
    if (m_busy) return;
    const QString sourcePath = QFileDialog::getOpenFileName(
        this,
        QStringLiteral("导入新时达 SA10-2000H 总装 STEP"),
        QString(),
        QStringLiteral("STEP模型 (*.step *.stp)"));
    if (sourcePath.isEmpty()) return;
    if (QMessageBox::question(
            this,
            QStringLiteral("确认机器人型号"),
            QStringLiteral(
                "所选文件将按固定型号 step.sa10-2000h（新时达 SA10-2000H）严格校验。\n\n"
                "只有已核验总装的精确 SHA-256、J0-J6、毫米单位、坐标方向和安装面全部通过，"
                "才会在最后一步登记到型号库。继续吗？"),
            QMessageBox::Yes | QMessageBox::No,
            QMessageBox::No) != QMessageBox::Yes)
    {
        return;
    }

    SetBusy(true);
    SetStatus(QStringLiteral(
        "正在以 bounds-only 模式解析总装、生成 30 mm 碰撞简模并做登记前回读…"));

    RobotModelCatalogStore::ModelRecord registeredModel;
    QString workerError;
    bool workerOk = false;
    QEventLoop waitLoop;
    QThread* worker = QThread::create([&]()
        {
            try
            {
                RobotCadAssemblyLoader::Options options;
                options.buildDetailedPresentation = false;
                options.prepareDisplayTriangulation = false;
                RobotCadAssemblyLoader::Result loaded;
                TheoreticalRobotModelStore::Asset sourceAsset;
                RobotCollisionEnvelopeStore::EnvelopeSet envelope;
                RobotCollisionEnvelopeStore::StoredAsset collisionAsset;
                RobotCollisionEnvelopeStore::GenerationParameters generation;

                workerOk = RobotCadAssemblyLoader::LoadFile(
                        sourcePath, loaded, workerError, &options)
                    && ValidateVerifiedSa10(loaded, workerError)
                    && TheoreticalRobotModelStore::ImportStepFile(
                        sourcePath, sourceAsset, workerError, false);
                if (workerOk && sourceAsset.sha256 != loaded.statistics.sourceSha256)
                {
                    workerError = QStringLiteral(
                        "机器人 STEP 在语义解析和受控复制之间发生变化，未登记型号。");
                    workerOk = false;
                }
                if (workerOk)
                {
                    workerOk = RobotCollisionEnvelopeStore::Generate(
                            loaded, generation, envelope, workerError)
                        && RobotCollisionEnvelopeStore::Persist(
                            envelope, collisionAsset, workerError);
                    loaded = RobotCadAssemblyLoader::Result();
                }
                if (workerOk)
                {
                    workerOk = RobotModelCatalogStore::RegisterValidatedModel(
                        QString::fromLatin1(kModelId),
                        QStringLiteral("新时达 SA10-2000H"),
                        QString::fromLatin1(kAdapterId),
                        ROBOT_TYPE_STEP,
                        sourceAsset,
                        collisionAsset,
                        registeredModel,
                        workerError);
                }
            }
            catch (const std::exception& exception)
            {
                workerError = QStringLiteral("导入机器人型号线程发生异常：%1")
                    .arg(QString::fromLocal8Bit(exception.what()).simplified());
                workerOk = false;
            }
            catch (...)
            {
                workerError = QStringLiteral("导入机器人型号线程发生未知异常。");
                workerOk = false;
            }
        });

    NonClosableProgressDialog progress(
        QStringLiteral(
            "正在校验 SA10 总装并生成 J0-J6 碰撞简模；首次可能需要 1–2 分钟…"),
        QString(), 0, 0, this);
    progress.setWindowTitle(QStringLiteral("导入机器人型号"));
    progress.setWindowModality(Qt::WindowModal);
    progress.setCancelButton(nullptr);
    progress.setMinimumDuration(0);
    progress.setAutoClose(false);
    progress.setAutoReset(false);
    progress.setMinimumWidth(600);
    progress.setWindowFlag(Qt::WindowCloseButtonHint, false);
    connect(worker, &QThread::finished, &waitLoop, &QEventLoop::quit);
    connect(worker, &QThread::finished, &progress, &QProgressDialog::accept);
    progress.show();
    worker->start();
    waitLoop.exec();
    worker->wait();
    delete worker;
    progress.accept();
    SetBusy(false);

    if (!workerOk)
    {
        SetStatus(QStringLiteral("机器人型号导入失败，目录未发布新记录：%1")
            .arg(workerError), true);
        QMessageBox::warning(this, QStringLiteral("导入机器人型号"), workerError);
        return;
    }
    RefreshCatalog(registeredModel.modelId);
    SetStatus(QStringLiteral(
        "型号 %1 已完整回读并登记；未绑定或修改任何控制单元机器人。")
        .arg(registeredModel.modelId));
}

bool RobotModelManagerDialog::StartRemoteWorker(std::function<void()> work)
{
    if (m_remoteBusy.load() || !work) return false;
    if (m_remoteWorker.joinable()) m_remoteWorker.join();
    m_remoteCancel.store(false);
    m_remoteCancelState.store(1);
    m_remoteBusy.store(true);
    SetRemoteBusy(true);
    try
    {
        m_remoteWorker = std::thread(
            [work = std::move(work)]()
            {
                work();
            });
    }
    catch (const std::exception& exception)
    {
        m_remoteBusy.store(false);
        m_remoteCancelState.store(0);
        SetRemoteBusy(false);
        if (m_serverStatusLabel != nullptr)
        {
            m_serverStatusLabel->setText(
                QStringLiteral("无法启动模型传输线程：%1")
                    .arg(QString::fromLocal8Bit(exception.what()).simplified()));
        }
        return false;
    }
    catch (...)
    {
        m_remoteBusy.store(false);
        m_remoteCancelState.store(0);
        SetRemoteBusy(false);
        if (m_serverStatusLabel != nullptr)
            m_serverStatusLabel->setText(QStringLiteral("无法启动模型传输线程。"));
        return false;
    }
    return true;
}

void RobotModelManagerDialog::FinishRemoteWorker()
{
    if (m_remoteWorker.joinable()) m_remoteWorker.join();
    m_remoteCancelState.store(0);
    m_remoteBusy.store(false);
    SetRemoteBusy(false);
}

void RobotModelManagerDialog::SetRemoteBusy(bool busy)
{
    if (m_serverRefreshButton != nullptr)
        m_serverRefreshButton->setEnabled(!busy);
    if (m_serverUploadButton != nullptr)
        m_serverUploadButton->setEnabled(!busy && !m_busy);
    if (m_serverDownloadButton != nullptr)
        m_serverDownloadButton->setEnabled(!busy);
    if (m_serverCancelButton != nullptr)
        m_serverCancelButton->setEnabled(
            busy && m_remoteCancelState.load() == 1);
    if (m_serverTable != nullptr)
        m_serverTable->setEnabled(!busy);
}

void RobotModelManagerDialog::CancelRemoteTransfer()
{
    if (!m_remoteBusy.load()) return;
    int expected = 1;
    if (!m_remoteCancelState.compare_exchange_strong(expected, 2))
        return;
    m_remoteCancel.store(true);
    if (m_transferStateLabel != nullptr)
        m_transferStateLabel->setText(QStringLiteral("正在安全停止模型传输…"));
    if (m_serverCancelButton != nullptr)
        m_serverCancelButton->setEnabled(false);
}

void RobotModelManagerDialog::BeginTransferUi(
    const QString& action,
    const QStringList& itemNames)
{
    m_transferAction = action;
    m_transferNames = itemNames;
    m_transferStates.clear();
    for (int index = 0; index < itemNames.size(); ++index)
        m_transferStates << QStringLiteral("等待");
    if (m_transferStateLabel != nullptr)
        m_transferStateLabel->setText(
            QStringLiteral("%1队列已建立").arg(action));
    if (m_transferCurrentLabel != nullptr)
        m_transferCurrentLabel->setText(QStringLiteral("当前文件：等待开始"));
    if (m_transferProgressBar != nullptr)
    {
        m_transferProgressBar->setValue(0);
        m_transferProgressBar->setFormat(QStringLiteral("0%"));
    }
    if (m_transferDetailLabel != nullptr)
        m_transferDetailLabel->setText(
            QStringLiteral("共 %1 项，按顺序传输并校验。")
                .arg(itemNames.size()));
    RefreshTransferQueueUi();
}

void RobotModelManagerDialog::UpdateTransferUi(
    int currentIndex,
    int completedItems,
    qulonglong transferredBytes,
    qulonglong totalBytes,
    double bytesPerSec,
    int etaSeconds,
    const QString& phase,
    const QString& itemState)
{
    if (currentIndex < 0 || currentIndex >= m_transferNames.size()) return;
    if (!itemState.isEmpty()
        && currentIndex < m_transferStates.size()
        && m_transferStates.at(currentIndex) != itemState)
    {
        m_transferStates[currentIndex] = itemState;
        RefreshTransferQueueUi();
    }
    if (m_transferStateLabel != nullptr)
        m_transferStateLabel->setText(
            QStringLiteral("正在%1").arg(m_transferAction));
    if (m_transferCurrentLabel != nullptr)
        m_transferCurrentLabel->setText(
            QStringLiteral("当前文件：%1").arg(
                m_transferNames.at(currentIndex)));
    const bool hasTotal = totalBytes > 0;
    const int percent = hasTotal
        ? qBound(
            0,
            static_cast<int>(transferredBytes * 100 / totalBytes),
            100)
        : 0;
    if (m_transferProgressBar != nullptr)
    {
        m_transferProgressBar->setValue(percent);
        m_transferProgressBar->setFormat(
            QStringLiteral("%1%").arg(percent));
    }
    if (m_transferQueueLabel != nullptr)
        m_transferQueueLabel->setText(
            QStringLiteral("%1队列：%2 项；完成 %3 项")
                .arg(m_transferAction)
                .arg(m_transferNames.size())
                .arg(completedItems));
    if (m_transferDetailLabel != nullptr)
    {
        QString detail = QStringLiteral("第 %1 / %2 项    %3")
            .arg(currentIndex + 1)
            .arg(m_transferNames.size())
            .arg(phase);
        if (hasTotal)
        {
            detail += QStringLiteral("    %1 / %2")
                .arg(HumanBytes(static_cast<double>(transferredBytes)))
                .arg(HumanBytes(static_cast<double>(totalBytes)));
        }
        if (bytesPerSec > 1.0)
            detail += QStringLiteral("    %1/s").arg(HumanBytes(bytesPerSec));
        if (etaSeconds > 0)
            detail += QStringLiteral("    预计剩余 %1 秒").arg(etaSeconds);
        m_transferDetailLabel->setText(detail);
    }
}

void RobotModelManagerDialog::FinishTransferUi(
    int completedItems,
    bool stoppedEarly,
    const QString& summary)
{
    for (int index = 0; index < m_transferStates.size(); ++index)
    {
        if (m_transferStates.at(index) == QStringLiteral("等待"))
            m_transferStates[index] = stoppedEarly
                ? QStringLiteral("未执行")
                : QStringLiteral("失败");
    }
    RefreshTransferQueueUi();
    if (m_transferStateLabel != nullptr)
        m_transferStateLabel->setText(stoppedEarly
            ? QStringLiteral("模型传输已安全停止")
            : QStringLiteral("模型传输已完成"));
    if (m_transferCurrentLabel != nullptr)
        m_transferCurrentLabel->setText(QStringLiteral("当前文件：无"));
    if (m_transferProgressBar != nullptr)
    {
        m_transferProgressBar->setValue(stoppedEarly ? 0 : 100);
        m_transferProgressBar->setFormat(stoppedEarly
            ? QStringLiteral("已停止")
            : QStringLiteral("100%"));
    }
    if (m_transferQueueLabel != nullptr)
        m_transferQueueLabel->setText(
            QStringLiteral("%1队列：%2 项；完成 %3 项")
                .arg(m_transferAction)
                .arg(m_transferNames.size())
                .arg(completedItems));
    if (m_transferDetailLabel != nullptr)
        m_transferDetailLabel->setText(summary);
}

void RobotModelManagerDialog::RefreshTransferQueueUi()
{
    if (m_transferQueueList == nullptr) return;
    m_transferQueueList->clear();
    for (int index = 0; index < m_transferNames.size(); ++index)
    {
        const QString state = index < m_transferStates.size()
            ? m_transferStates.at(index)
            : QStringLiteral("等待");
        m_transferQueueList->addItem(
            QStringLiteral("%1  %2.  %3")
                .arg(state)
                .arg(index + 1)
                .arg(m_transferNames.at(index)));
    }
    if (m_transferNames.isEmpty())
        m_transferQueueList->addItem(
            QStringLiteral("当前没有模型传输任务"));
}

void RobotModelManagerDialog::ApplyServerCatalog(
    const RobotModelRemoteCatalog::Catalog& catalog,
    const QHash<QString, QImage>& previews,
    const QStringList& previewWarnings)
{
    QString previousModelId;
    if (m_serverTable != nullptr && m_serverTable->currentRow() >= 0)
    {
        QTableWidgetItem* item =
            m_serverTable->item(m_serverTable->currentRow(), 1);
        if (item != nullptr)
            previousModelId = item->data(Qt::UserRole).toString();
    }
    m_serverCatalog = catalog;
    m_serverPreviews = previews;
    m_serverTable->setRowCount(catalog.models.size());
    int selectedRow = -1;
    for (qsizetype index = 0; index < catalog.models.size(); ++index)
    {
        const auto& record = catalog.models.at(index);
        const auto& model = record.model;
        const int row = static_cast<int>(index);
        QTableWidgetItem* previewItem = new QTableWidgetItem();
        const QImage image = previews.value(model.modelId);
        if (!image.isNull())
            previewItem->setIcon(QIcon(QPixmap::fromImage(image)));
        else if (record.preview.sourceKind
                 != RobotModelRemoteCatalog::OriginalStepPreviewKind())
            previewItem->setText(QStringLiteral("需更新原图"));
        else
            previewItem->setText(QStringLiteral("无预览"));
        QList<QTableWidgetItem*> items = {
            previewItem,
            new QTableWidgetItem(model.modelId),
            new QTableWidgetItem(model.displayName),
            new QTableWidgetItem(RobotTypeText(model.sourceRobotType)),
            new QTableWidgetItem(SizeText(model.sourceStep.sizeBytes)),
            new QTableWidgetItem(ShortSha(model.collision.profileKeySha256)),
            new QTableWidgetItem(catalog.publishedUtc)
        };
        items[1]->setData(Qt::UserRole, model.modelId);
        for (int column = 0; column < items.size(); ++column)
        {
            ConfigureReadOnlyItem(items.at(column));
            m_serverTable->setItem(row, column, items.at(column));
        }
        if (model.modelId == previousModelId) selectedRow = row;
    }
    if (selectedRow < 0 && !catalog.models.isEmpty()) selectedRow = 0;
    if (selectedRow >= 0)
    {
        m_serverTable->selectRow(selectedRow);
        m_serverTable->setCurrentCell(selectedRow, 1);
    }
    else
    {
        m_serverDetailsEdit->clear();
        m_serverPreviewLabel->setPixmap(QPixmap());
        m_serverPreviewLabel->setText(QStringLiteral("服务器模型库为空"));
    }
    RefreshServerSelectionDetails();
    if (catalog.models.isEmpty())
    {
        m_serverStatusLabel->setText(
            QStringLiteral("服务器模型库为空，可上传当前本地已验证型号。"));
        m_serverStatusLabel->setStyleSheet(QStringLiteral(
            "QLabel { color:#b3e5fc; padding:5px; }"));
    }
    else if (!previewWarnings.isEmpty())
    {
        m_serverStatusLabel->setText(
            QStringLiteral("已读取 %1 个服务器型号；%2")
                .arg(catalog.models.size())
                .arg(previewWarnings.join(QStringLiteral("；"))));
        m_serverStatusLabel->setStyleSheet(QStringLiteral(
            "QLabel { color:#ffcc80; padding:5px; }"));
    }
    else
    {
        m_serverStatusLabel->setText(
            QStringLiteral(
                "已读取 %1 个服务器型号，模型号、简模型号和原始 STEP 缩略图均已校验。")
                .arg(catalog.models.size()));
        m_serverStatusLabel->setStyleSheet(QStringLiteral(
            "QLabel { color:#a5d6a7; padding:5px; }"));
    }
}

void RobotModelManagerDialog::RefreshServerSelectionDetails()
{
    if (m_serverTable == nullptr
        || m_serverDetailsEdit == nullptr
        || m_serverPreviewLabel == nullptr)
        return;
    const int row = m_serverTable->currentRow();
    if (row < 0 || row >= m_serverCatalog.models.size())
    {
        m_serverDetailsEdit->clear();
        m_serverPreviewLabel->setPixmap(QPixmap());
        m_serverPreviewLabel->setText(
            QStringLiteral("没有可预览的服务器型号"));
        return;
    }
    const auto& record = m_serverCatalog.models.at(row);
    const auto& model = record.model;
    const QImage image = m_serverPreviews.value(model.modelId);
    if (image.isNull())
    {
        m_serverPreviewLabel->setPixmap(QPixmap());
        m_serverPreviewLabel->setText(
            record.preview.sourceKind
                    != RobotModelRemoteCatalog::OriginalStepPreviewKind()
                ? QStringLiteral(
                    "服务器仍是旧版简模预览\n请重新上传同一型号以生成原始 STEP 缩略图")
                : QStringLiteral("服务器原始 STEP 缩略图缺失或校验失败"));
    }
    else
    {
        m_serverPreviewLabel->setText(QString());
        m_serverPreviewLabel->setPixmap(QPixmap::fromImage(image).scaled(
            m_serverPreviewLabel->size(),
            Qt::KeepAspectRatio,
            Qt::SmoothTransformation));
    }
    m_serverDetailsEdit->setPlainText(QStringLiteral(
        "型号ID：%1\n"
        "显示名称：%2\n"
        "场景适配器：%3\n"
        "驱动类型：%4\n"
        "STEP：%5（%6）\n"
        "STEP SHA-256：%7\n"
        "简模型号：%8\n"
        "简模 payload SHA-256：%9\n"
        "安全余量：%10\n"
        "原始 STEP 缩略图 SHA-256：%11\n"
        "服务器清单发布时间：%12\n\n"
        "下载后仍会执行软件内置可信型号、STEP、J0-J6 和碰撞简模完整性门禁。")
        .arg(model.modelId)
        .arg(model.displayName)
        .arg(model.adapterId)
        .arg(RobotTypeText(model.sourceRobotType))
        .arg(model.sourceStep.originalDisplayName)
        .arg(SizeText(model.sourceStep.sizeBytes))
        .arg(model.sourceStep.sha256)
        .arg(model.collision.profileKeySha256)
        .arg(model.collisionPayloadSha256)
        .arg(MarginText(model.collision.safetyMarginMicrometres))
        .arg(record.preview.sha256)
        .arg(m_serverCatalog.publishedUtc));
}

void RobotModelManagerDialog::RefreshServerCatalog()
{
    if (m_remoteBusy.load()) return;
    RemoteFtpConfig config;
    QString configError;
    if (!CurrentRemoteFtpConfig(config, configError))
    {
        m_serverStatusLabel->setText(configError);
        m_serverStatusLabel->setStyleSheet(QStringLiteral(
            "QLabel { color:#ff8a80; padding:5px; }"));
        return;
    }
    BeginTransferUi(
        QStringLiteral("读取"),
        { QStringLiteral("服务器模型清单"),
          QStringLiteral("模型预览图") });
    m_serverStatusLabel->setText(QStringLiteral("正在连接服务器模型库…"));
    StartRemoteWorker([this, config]()
        {
            RobotModelRemoteCatalog::Catalog catalog;
            QHash<QString, QImage> previews;
            QStringList warnings;
            QString error;
            bool ok = false;
            int completed = 0;
            try
            {
                const QString transferRoot = AppPaths::WritablePath(
                    QStringLiteral("Temp/RobotModelTransfers"));
                if (transferRoot.isEmpty() || !QDir().mkpath(transferRoot))
                {
                    error = QStringLiteral("无法创建机器人模型传输临时目录。");
                }
                QTemporaryDir temporary(
                    QDir(transferRoot).filePath(QStringLiteral("refresh-XXXXXX")));
                if (error.isEmpty() && !temporary.isValid())
                    error = QStringLiteral("无法创建唯一模型列表 staging。");
                RobotLog log(OnlineServicesLogPath(), false);
                FtpClient ftp(
                    &log, config.host, config.port,
                    config.user, config.password);
                ftp.setMessageBoxesEnabled(false);
                QString catalogName;
                const auto started = std::chrono::steady_clock::now();
                if (error.isEmpty())
                {
                    ok = ReadLatestRemoteCatalog(
                        ftp, temporary.path(), &m_remoteCancel,
                        catalog, catalogName, error,
                        [this, started](qulonglong received, qulonglong total)
                        {
                            if (m_remoteCancel.load()) return;
                            const double elapsed = std::max(
                                0.001,
                                std::chrono::duration<double>(
                                    std::chrono::steady_clock::now() - started)
                                    .count());
                            const double speed =
                                static_cast<double>(received) / elapsed;
                            const int eta = speed > 1.0 && received < total
                                ? static_cast<int>(std::ceil(
                                    static_cast<double>(total - received)
                                    / speed))
                                : 0;
                            QMetaObject::invokeMethod(
                                this,
                                [this, received, total, speed, eta]()
                                {
                                    UpdateTransferUi(
                                        0, 0, received, total, speed, eta,
                                        QStringLiteral("FTP 清单下载"),
                                        QStringLiteral("正在读取"));
                                },
                                Qt::QueuedConnection);
                        });
                }
                if (ok)
                {
                    completed = 1;
                    QMetaObject::invokeMethod(
                        this,
                        [this]()
                        {
                            UpdateTransferUi(
                                0, 1, 1, 1, 0.0, 0,
                                QStringLiteral("清单完整性已校验"),
                                QStringLiteral("完成"));
                        },
                        Qt::QueuedConnection);
                }
                RemoteFileMap previewFiles;
                if (ok)
                {
                    ok = ListRemoteDirectory(
                        ftp, RobotModelRemoteCatalog::RemotePreviewDirectory(),
                        previewFiles, &m_remoteCancel, error);
                }
                qulonglong totalPreviewBytes = 0;
                if (ok)
                {
                    for (const auto& record : catalog.models)
                    {
                        if (record.preview.sourceKind
                            != RobotModelRemoteCatalog::
                                OriginalStepPreviewKind())
                        {
                            continue;
                        }
                        const auto found =
                            previewFiles.constFind(record.preview.storedFileName);
                        if (found != previewFiles.cend()
                            && found.value()
                                == static_cast<qulonglong>(
                                    record.preview.sizeBytes))
                        {
                            totalPreviewBytes += found.value();
                        }
                    }
                }
                qulonglong completedPreviewBytes = 0;
                for (qsizetype index = 0;
                     ok && index < catalog.models.size();
                     ++index)
                {
                    if (m_remoteCancel.load())
                    {
                        ok = false;
                        error = QStringLiteral("用户取消读取服务器模型列表。");
                        break;
                    }
                    const auto& record = catalog.models.at(index);
                    if (record.preview.sourceKind
                        != RobotModelRemoteCatalog::
                            OriginalStepPreviewKind())
                    {
                        warnings << QStringLiteral(
                            "%1 仍使用旧版简模预览，已停止显示；重新上传同一型号可更新原图")
                            .arg(record.model.modelId);
                        continue;
                    }
                    const auto found =
                        previewFiles.constFind(record.preview.storedFileName);
                    if (found == previewFiles.cend()
                        || found.value()
                            != static_cast<qulonglong>(
                                record.preview.sizeBytes))
                    {
                        warnings << QStringLiteral("%1 的预览图缺失或大小不符")
                            .arg(record.model.modelId);
                        continue;
                    }
                    const QString localPreview = QDir(temporary.path()).filePath(
                        QStringLiteral("preview-%1.png").arg(index));
                    const qulonglong beforeBytes = completedPreviewBytes;
                    const auto previewStarted =
                        std::chrono::steady_clock::now();
                    const bool downloaded = ftp.downloadFileBounded(
                        Utf8Path(JoinRemote(
                            RobotModelRemoteCatalog::RemotePreviewDirectory(),
                            record.preview.storedFileName)),
                        QDir::toNativeSeparators(localPreview)
                            .toLocal8Bit().toStdString(),
                        found.value(),
                        static_cast<unsigned long long>(
                            RobotModelRemoteCatalog::MaximumPreviewBytes),
                        &m_remoteCancel,
                        [this, beforeBytes, totalPreviewBytes, previewStarted](
                            unsigned long long received,
                            unsigned long long)
                        {
                            if (m_remoteCancel.load()) return;
                            const qulonglong aggregate = beforeBytes + received;
                            const double elapsed = std::max(
                                0.001,
                                std::chrono::duration<double>(
                                    std::chrono::steady_clock::now()
                                        - previewStarted).count());
                            const double speed =
                                static_cast<double>(received) / elapsed;
                            const int eta =
                                speed > 1.0
                                    && aggregate < totalPreviewBytes
                                ? static_cast<int>(std::ceil(
                                    static_cast<double>(
                                        totalPreviewBytes - aggregate)
                                    / speed))
                                : 0;
                            QMetaObject::invokeMethod(
                                this,
                                [this, aggregate, totalPreviewBytes, speed, eta]()
                                {
                                    UpdateTransferUi(
                                        1, 1, aggregate, totalPreviewBytes,
                                        speed, eta,
                                        QStringLiteral(
                                            "下载并校验原始 STEP 缩略图"),
                                        QStringLiteral("正在读取"));
                                },
                                Qt::QueuedConnection);
                        });
                    if (!downloaded)
                    {
                        warnings << QStringLiteral(
                            "%1 的原始 STEP 缩略图下载失败")
                            .arg(record.model.modelId);
                        continue;
                    }
                    QByteArray pngBytes;
                    QImage image;
                    QString previewError;
                    if (!ReadFileBounded(
                            localPreview,
                            RobotModelRemoteCatalog::MaximumPreviewBytes,
                            pngBytes, previewError)
                        || !RobotModelRemoteCatalog::ValidatePreview(
                            pngBytes, record.preview, image, previewError))
                    {
                        warnings << QStringLiteral(
                            "%1 的原始 STEP 缩略图校验失败：%2")
                            .arg(record.model.modelId, previewError);
                        continue;
                    }
                    previews.insert(record.model.modelId, image);
                    completedPreviewBytes += found.value();
                }
                if (ok)
                {
                    completed = 2;
                    QMetaObject::invokeMethod(
                        this,
                        [this, totalPreviewBytes]()
                        {
                            UpdateTransferUi(
                                1, 2,
                                totalPreviewBytes > 0 ? totalPreviewBytes : 1,
                                totalPreviewBytes > 0 ? totalPreviewBytes : 1,
                                0.0, 0,
                                QStringLiteral("预览图读取完成"),
                                QStringLiteral("完成"));
                        },
                        Qt::QueuedConnection);
                }
            }
            catch (const std::exception& exception)
            {
                ok = false;
                error = QStringLiteral("读取服务器模型库发生异常：%1")
                    .arg(QString::fromLocal8Bit(exception.what()).simplified());
            }
            catch (...)
            {
                ok = false;
                error = QStringLiteral("读取服务器模型库发生未知异常。");
            }
            const bool cancelled = m_remoteCancel.load();
            QMetaObject::invokeMethod(
                this,
                [this, ok, cancelled, completed, catalog, previews,
                    warnings, error]()
                {
                    FinishRemoteWorker();
                    if (ok)
                    {
                        ApplyServerCatalog(catalog, previews, warnings);
                        FinishTransferUi(
                            completed, false,
                            QStringLiteral(
                                "服务器模型列表读取完成：%1 个型号。")
                                .arg(catalog.models.size()));
                    }
                    else
                    {
                        const QString message = cancelled
                            ? QStringLiteral("服务器模型列表读取已取消。")
                            : QStringLiteral("服务器模型列表读取失败：%1")
                                .arg(error);
                        m_serverStatusLabel->setText(message);
                        m_serverStatusLabel->setStyleSheet(QStringLiteral(
                            "QLabel { color:#ff8a80; padding:5px; }"));
                        FinishTransferUi(completed, true, message);
                    }
                },
                Qt::QueuedConnection);
        });
}

void RobotModelManagerDialog::UploadSelectedModel()
{
    if (m_remoteBusy.load() || m_modelTable == nullptr) return;
    const int row = m_modelTable->currentRow();
    if (row < 0 || row >= m_models.size())
    {
        QMessageBox::information(
            this, QStringLiteral("上传机器人模型"),
            QStringLiteral("请先在“本地模型”中选择一个已验证型号。"));
        return;
    }
    RemoteFtpConfig config;
    QString configError;
    if (!CurrentRemoteFtpConfig(config, configError))
    {
        QMessageBox::warning(
            this, QStringLiteral("上传机器人模型"), configError);
        return;
    }
    const RobotModelCatalogStore::ModelRecord model = m_models.at(row);
    if (QMessageBox::question(
            this,
            QStringLiteral("确认上传机器人模型"),
            QStringLiteral(
                "将把以下已验证型号上传到服务器独立目录：\n\n"
                "型号：%1\n显示名称：%2\nSTEP：%3\n简模型号：%4\n\n"
                "服务器会保存 STEP、碰撞简模、原始 STEP 缩略图和不可变版本清单。继续吗？")
                .arg(model.modelId)
                .arg(model.displayName)
                .arg(SizeText(model.sourceStep.sizeBytes))
                .arg(model.collision.profileKeySha256),
            QMessageBox::Yes | QMessageBox::No,
            QMessageBox::No) != QMessageBox::Yes)
    {
        return;
    }

    const QString sourcePath = QDir(
        TheoreticalRobotModelStore::StoreDirectory())
        .filePath(model.sourceStep.storedFileName);
    const QString collisionPath = QDir(
        RobotCollisionEnvelopeStore::StoreDirectory())
        .filePath(model.collision.storedFileName);
    BeginTransferUi(
        QStringLiteral("上传"),
        {
            QStringLiteral("STEP：%1").arg(model.sourceStep.storedFileName),
            QStringLiteral("碰撞简模：%1").arg(
                model.collision.storedFileName),
            QStringLiteral("原始 STEP 缩略图"),
            QStringLiteral("服务器模型清单")
        });
    m_serverStatusLabel->setText(
        QStringLiteral("正在准备上传型号 %1…").arg(model.modelId));

    StartRemoteWorker(
        [this, config, model, sourcePath, collisionPath]()
        {
            bool ok = false;
            bool cancelled = false;
            int completed = 0;
            QString error;
            RobotModelRemoteCatalog::Catalog publishedCatalog;
            try
            {
                RobotModelCatalogStore::Eligibility eligibility;
                if (!RobotModelCatalogStore::ResolveModelEligibility(
                        model.modelId, model.sourceRobotType,
                        eligibility, error)
                    || !eligibility.eligible
                    || eligibility.model.sourceStep.sha256
                        != model.sourceStep.sha256)
                {
                    if (error.isEmpty())
                        error = eligibility.reason.isEmpty()
                            ? QStringLiteral("本地型号未通过生产资格校验。")
                            : eligibility.reason;
                }
                QString sourceSha;
                qint64 sourceBytes = 0;
                if (error.isEmpty()
                    && (!RobotModelRemoteCatalog::HashFileStable(
                            sourcePath,
                            TheoreticalRobotModelStore::MaximumAssetBytes,
                            sourceSha, sourceBytes, error)
                        || sourceSha != model.sourceStep.sha256
                        || sourceBytes != model.sourceStep.sizeBytes))
                {
                    if (error.isEmpty())
                        error = QStringLiteral(
                            "本地 STEP 大小或 SHA-256 与型号目录不一致。");
                }
                RobotCollisionEnvelopeStore::EnvelopeSet envelope;
                if (error.isEmpty()
                    && (!RobotCollisionEnvelopeStore::LoadAsset(
                            model.collision, envelope, error)
                        || envelope.payloadSha256
                            != model.collisionPayloadSha256))
                {
                    if (error.isEmpty())
                        error = QStringLiteral(
                            "本地碰撞简模与型号目录不一致。");
                }

                QImage previewImage;
                QByteArray previewBytes;
                RobotModelRemoteCatalog::PreviewAsset previewAsset;
                if (error.isEmpty())
                {
                    QMetaObject::invokeMethod(
                        this,
                        [this]()
                        {
                            UpdateTransferUi(
                                2, 0, 0, 1, 0.0, 0,
                                QStringLiteral(
                                    "解析原始 STEP/B-Rep 并生成缩略图"),
                                QStringLiteral("正在生成"));
                        },
                        Qt::QueuedConnection);
                }
                if (error.isEmpty()
                    && (!RobotModelOriginalPreview::RenderStepFile(
                            sourcePath, model.displayName,
                            previewImage, error)
                        || !RobotModelRemoteCatalog::EncodePreview(
                            previewImage,
                            RobotModelRemoteCatalog::
                                OriginalStepPreviewKind(),
                            previewBytes,
                            previewAsset, error)))
                {
                    // error 已由预览生成器填写。
                }

                const QString transferRoot = AppPaths::WritablePath(
                    QStringLiteral("Temp/RobotModelTransfers"));
                if (error.isEmpty()
                    && (transferRoot.isEmpty()
                        || !QDir().mkpath(transferRoot)))
                {
                    error = QStringLiteral(
                        "无法创建机器人模型上传临时目录。");
                }
                QTemporaryDir temporary(
                    QDir(transferRoot).filePath(
                        QStringLiteral("upload-XXXXXX")));
                if (error.isEmpty() && !temporary.isValid())
                    error = QStringLiteral("无法创建唯一模型上传 staging。");
                const QString previewPath =
                    QDir(temporary.path()).filePath(
                        previewAsset.storedFileName);
                if (error.isEmpty()
                    && !WriteExclusiveFile(
                        previewPath, previewBytes, error))
                {
                    // error 已填写。
                }

                RobotLog log(OnlineServicesLogPath(), false);
                FtpClient ftp(
                    &log, config.host, config.port,
                    config.user, config.password);
                ftp.setMessageBoxesEnabled(false);
                RobotModelRemoteCatalog::Catalog currentCatalog;
                QString currentCatalogName;
                if (error.isEmpty()
                    && !ReadLatestRemoteCatalog(
                        ftp, temporary.path(), &m_remoteCancel,
                        currentCatalog, currentCatalogName, error))
                {
                    // error 已填写。
                }

                RobotModelRemoteCatalog::ModelRecord remoteRecord;
                remoteRecord.model = model;
                remoteRecord.preview = previewAsset;
                bool publishNewCatalog = false;
                if (error.isEmpty())
                {
                    auto found = std::find_if(
                        currentCatalog.models.begin(),
                        currentCatalog.models.end(),
                        [&model](const auto& record)
                        {
                            return record.model.modelId == model.modelId;
                        });
                    if (found == currentCatalog.models.end())
                    {
                        currentCatalog.models.append(remoteRecord);
                        publishNewCatalog = true;
                    }
                    else
                    {
                        const auto& existing = found->model;
                        if (existing.sourceStep.sha256
                                != model.sourceStep.sha256
                            || existing.collision.profileKeySha256
                                != model.collision.profileKeySha256
                            || existing.collisionPayloadSha256
                                != model.collisionPayloadSha256
                            || existing.adapterId != model.adapterId
                            || existing.sourceRobotType
                                != model.sourceRobotType)
                        {
                            error = QStringLiteral(
                                "服务器型号 %1 已绑定不同的 STEP、简模或适配器，"
                                "禁止静默覆盖。").arg(model.modelId);
                        }
                        else if (found->preview.sha256
                                     != previewAsset.sha256
                                 || found->model.displayName
                                     != model.displayName)
                        {
                            *found = remoteRecord;
                            publishNewCatalog = true;
                        }
                    }
                }

                QByteArray catalogBytes;
                QString catalogPath;
                QString catalogFileName;
                if (error.isEmpty() && publishNewCatalog)
                {
                    RobotModelRemoteCatalog::Catalog candidate =
                        currentCatalog;
                    candidate.revisionUtcMs = std::max(
                        QDateTime::currentMSecsSinceEpoch(),
                        currentCatalog.revisionUtcMs + 1);
                    candidate.publishedUtc =
                        QDateTime::currentDateTimeUtc()
                            .toString(Qt::ISODateWithMs);
                    candidate.previousCatalogSha256 =
                        currentCatalog.payloadSha256;
                    candidate.payloadSha256.clear();
                    if (!RobotModelRemoteCatalog::Serialize(
                            candidate, catalogBytes,
                            publishedCatalog, error))
                    {
                        // error 已填写。
                    }
                    else
                    {
                        catalogFileName =
                            RobotModelRemoteCatalog::CatalogFileName(
                                publishedCatalog);
                        catalogPath = QDir(temporary.path()).filePath(
                            catalogFileName);
                        if (catalogFileName.isEmpty()
                            || !WriteExclusiveFile(
                                catalogPath, catalogBytes, error))
                        {
                            if (error.isEmpty())
                                error = QStringLiteral(
                                    "无法生成服务器模型清单文件名。");
                        }
                    }
                }
                else if (error.isEmpty())
                {
                    publishedCatalog = currentCatalog;
                }

                RemoteFileMap assetFiles;
                RemoteFileMap collisionFiles;
                RemoteFileMap previewFiles;
                RemoteFileMap catalogFiles;
                if (error.isEmpty()
                    && (!ListRemoteDirectory(
                            ftp,
                            RobotModelRemoteCatalog::RemoteAssetsDirectory(),
                            assetFiles, &m_remoteCancel, error)
                        || !ListRemoteDirectory(
                            ftp,
                            RobotModelRemoteCatalog::RemoteCollisionDirectory(),
                            collisionFiles, &m_remoteCancel, error)
                        || !ListRemoteDirectory(
                            ftp,
                            RobotModelRemoteCatalog::RemotePreviewDirectory(),
                            previewFiles, &m_remoteCancel, error)
                        || !ListRemoteDirectory(
                            ftp,
                            RobotModelRemoteCatalog::RemoteCatalogDirectory(),
                            catalogFiles, &m_remoteCancel, error)))
                {
                    // error 已填写。
                }

                auto uploadOne = [
                    this, &ftp, &completed, &error
                ](
                    int index,
                    const QString& localPath,
                    const QString& remoteDirectory,
                    const QString& fileName,
                    qint64 expectedBytes,
                    RemoteFileMap& remoteFiles) -> bool
                {
                    const auto existing = remoteFiles.constFind(fileName);
                    if (existing != remoteFiles.cend())
                    {
                        if (existing.value()
                            != static_cast<qulonglong>(expectedBytes))
                        {
                            error = QStringLiteral(
                                "服务器已有同名模型资产但大小不一致：%1")
                                .arg(fileName);
                            return false;
                        }
                        ++completed;
                        QMetaObject::invokeMethod(
                            this,
                            [this, index, completed, expectedBytes]()
                            {
                                UpdateTransferUi(
                                    index, completed,
                                    expectedBytes, expectedBytes,
                                    0.0, 0,
                                    QStringLiteral("服务器已有相同内容寻址文件"),
                                    QStringLiteral("服务器已有"));
                            },
                            Qt::QueuedConnection);
                        return true;
                    }
                    QMetaObject::invokeMethod(
                        this,
                        [this, index]()
                        {
                            UpdateTransferUi(
                                index, 0, 0, 0, 0.0, 0,
                                QStringLiteral("准备 FTP 上传"),
                                QStringLiteral("正在上传"));
                        },
                        Qt::QueuedConnection);
                    const auto started = std::chrono::steady_clock::now();
                    auto lastUi =
                        std::chrono::steady_clock::time_point{};
                    const bool uploaded = ftp.uploadFileWithProgress(
                        QDir::toNativeSeparators(localPath)
                            .toLocal8Bit().toStdString(),
                        Utf8Path(JoinRemote(remoteDirectory, fileName)),
                        &m_remoteCancel,
                        [this, index, started, &lastUi](
                            long long sent, long long total)
                        {
                            if (m_remoteCancel.load()) return;
                            const auto now =
                                std::chrono::steady_clock::now();
                            if (lastUi
                                    != std::chrono::steady_clock::time_point{}
                                && sent < total
                                && std::chrono::duration_cast<
                                    std::chrono::milliseconds>(
                                    now - lastUi).count() < 120)
                            {
                                return;
                            }
                            lastUi = now;
                            const double elapsed = std::max(
                                0.001,
                                std::chrono::duration<double>(
                                    now - started).count());
                            const double speed =
                                static_cast<double>(sent) / elapsed;
                            const int eta = speed > 1.0 && sent < total
                                ? static_cast<int>(std::ceil(
                                    static_cast<double>(total - sent)
                                    / speed))
                                : 0;
                            QMetaObject::invokeMethod(
                                this,
                                [this, index, sent, total, speed, eta]()
                                {
                                    UpdateTransferUi(
                                        index, 0,
                                        static_cast<qulonglong>(sent),
                                        static_cast<qulonglong>(total),
                                        speed, eta,
                                        QStringLiteral("FTP 传输中"),
                                        QStringLiteral("正在上传"));
                                },
                                Qt::QueuedConnection);
                        },
                        false);
                    if (!uploaded)
                    {
                        error = m_remoteCancel.load()
                            ? QStringLiteral("用户取消模型上传。")
                            : QStringLiteral("FTP 上传失败：%1")
                                .arg(fileName);
                        return false;
                    }
                    RemoteFileMap verified;
                    if (!ListRemoteDirectory(
                            ftp, remoteDirectory, verified,
                            &m_remoteCancel, error)
                        || !ResolveRemoteFile(
                            verified, fileName, expectedBytes,
                            QStringLiteral("上传资产"), error))
                    {
                        return false;
                    }
                    remoteFiles = verified;
                    ++completed;
                    QMetaObject::invokeMethod(
                        this,
                        [this, index, completed, expectedBytes]()
                        {
                            UpdateTransferUi(
                                index, completed,
                                expectedBytes, expectedBytes,
                                0.0, 0,
                                QStringLiteral("服务器大小回读通过"),
                                QStringLiteral("完成"));
                        },
                        Qt::QueuedConnection);
                    return true;
                };

                if (error.isEmpty()
                    && !uploadOne(
                        0, sourcePath,
                        RobotModelRemoteCatalog::RemoteAssetsDirectory(),
                        model.sourceStep.storedFileName,
                        model.sourceStep.sizeBytes, assetFiles))
                {
                    // error 已填写。
                }
                if (error.isEmpty()
                    && !uploadOne(
                        1, collisionPath,
                        RobotModelRemoteCatalog::RemoteCollisionDirectory(),
                        model.collision.storedFileName,
                        model.collision.sizeBytes, collisionFiles))
                {
                    // error 已填写。
                }
                if (error.isEmpty()
                    && !uploadOne(
                        2, previewPath,
                        RobotModelRemoteCatalog::RemotePreviewDirectory(),
                        previewAsset.storedFileName,
                        previewAsset.sizeBytes, previewFiles))
                {
                    // error 已填写。
                }
                if (error.isEmpty())
                {
                    if (catalogFileName.isEmpty())
                    {
                        ++completed;
                        QMetaObject::invokeMethod(
                            this,
                            [this, completed]()
                            {
                                UpdateTransferUi(
                                    3, completed, 1, 1, 0.0, 0,
                                    QStringLiteral("型号已在最新服务器清单中"),
                                    QStringLiteral("无需更新"));
                            },
                            Qt::QueuedConnection);
                    }
                    else if (!uploadOne(
                        3, catalogPath,
                        RobotModelRemoteCatalog::RemoteCatalogDirectory(),
                        catalogFileName,
                        catalogBytes.size(), catalogFiles))
                    {
                        // error 已填写。
                    }
                }

                if (error.isEmpty())
                {
                    RobotModelRemoteCatalog::Catalog readback;
                    QString readbackName;
                    if (!ReadLatestRemoteCatalog(
                            ftp, temporary.path(), &m_remoteCancel,
                            readback, readbackName, error)
                        || readback.payloadSha256
                            != publishedCatalog.payloadSha256)
                    {
                        if (error.isEmpty())
                            error = QStringLiteral(
                                "服务器最新模型清单回读身份不一致。");
                    }
                    else
                    {
                        const auto found = std::find_if(
                            readback.models.cbegin(),
                            readback.models.cend(),
                            [&model](const auto& record)
                            {
                                return record.model.modelId
                                        == model.modelId
                                    && record.model.sourceStep.sha256
                                        == model.sourceStep.sha256
                                    && record.model.collision.profileKeySha256
                                        == model.collision.profileKeySha256;
                            });
                        if (found == readback.models.cend())
                            error = QStringLiteral(
                                "服务器清单回读后找不到刚上传的型号。");
                    }
                }
                ok = error.isEmpty() && !m_remoteCancel.load();
                cancelled = m_remoteCancel.load();
            }
            catch (const std::exception& exception)
            {
                error = QStringLiteral("上传机器人模型发生异常：%1")
                    .arg(QString::fromLocal8Bit(exception.what()).simplified());
            }
            catch (...)
            {
                error = QStringLiteral("上传机器人模型发生未知异常。");
            }
            cancelled = cancelled || m_remoteCancel.load();
            QMetaObject::invokeMethod(
                this,
                [this, ok, cancelled, completed, model, error]()
                {
                    FinishRemoteWorker();
                    const QString message = ok
                        ? QStringLiteral(
                            "型号 %1 已上传，服务器模型清单与资产大小回读通过。")
                            .arg(model.modelId)
                        : cancelled
                            ? QStringLiteral("型号 %1 上传已取消。")
                                .arg(model.modelId)
                            : QStringLiteral("型号 %1 上传失败：%2")
                                .arg(model.modelId, error);
                    m_serverStatusLabel->setText(message);
                    m_serverStatusLabel->setStyleSheet(ok
                        ? QStringLiteral(
                            "QLabel { color:#a5d6a7; padding:5px; }")
                        : QStringLiteral(
                            "QLabel { color:#ff8a80; padding:5px; }"));
                    FinishTransferUi(completed, !ok, message);
                    if (ok)
                        QTimer::singleShot(
                            600, this,
                            [this]() { RefreshServerCatalog(); });
                },
                Qt::QueuedConnection);
        });
}

void RobotModelManagerDialog::DownloadSelectedServerModel()
{
    if (m_remoteBusy.load() || m_serverTable == nullptr) return;
    const int row = m_serverTable->currentRow();
    if (row < 0 || row >= m_serverCatalog.models.size())
    {
        QMessageBox::information(
            this, QStringLiteral("下载机器人模型"),
            QStringLiteral("请先选择一个服务器机器人型号。"));
        return;
    }
    RemoteFtpConfig config;
    QString configError;
    if (!CurrentRemoteFtpConfig(config, configError))
    {
        QMessageBox::warning(
            this, QStringLiteral("下载机器人模型"), configError);
        return;
    }
    const RobotModelRemoteCatalog::ModelRecord selected =
        m_serverCatalog.models.at(row);
    if (QMessageBox::question(
            this,
            QStringLiteral("确认下载机器人模型"),
            QStringLiteral(
                "将下载并验证以下服务器型号：\n\n"
                "型号：%1\n显示名称：%2\nSTEP：%3\n简模型号：%4\n\n"
                "下载完成后仍需通过软件内置可信型号和简模门禁，"
                "才会登记到本地型号库。继续吗？")
                .arg(selected.model.modelId)
                .arg(selected.model.displayName)
                .arg(SizeText(selected.model.sourceStep.sizeBytes))
                .arg(selected.model.collision.profileKeySha256),
            QMessageBox::Yes | QMessageBox::No,
            QMessageBox::No) != QMessageBox::Yes)
    {
        return;
    }

    BeginTransferUi(
        QStringLiteral("下载"),
        {
            QStringLiteral("STEP：%1")
                .arg(selected.model.sourceStep.storedFileName),
            QStringLiteral("碰撞简模：%1")
                .arg(selected.model.collision.storedFileName),
            QStringLiteral("预览图：%1")
                .arg(selected.preview.storedFileName)
        });
    m_serverStatusLabel->setText(
        QStringLiteral("正在准备下载型号 %1…")
            .arg(selected.model.modelId));

    StartRemoteWorker(
        [this, config, selected]()
        {
            bool ok = false;
            bool cancelled = false;
            int completed = 0;
            QString error;
            RobotModelCatalogStore::ModelRecord registeredModel;
            try
            {
                const QString transferRoot = AppPaths::WritablePath(
                    QStringLiteral("Temp/RobotModelTransfers"));
                if (transferRoot.isEmpty() || !QDir().mkpath(transferRoot))
                    error = QStringLiteral(
                        "无法创建机器人模型下载临时目录。");
                QTemporaryDir temporary(
                    QDir(transferRoot).filePath(
                        QStringLiteral("download-XXXXXX")));
                if (error.isEmpty() && !temporary.isValid())
                    error = QStringLiteral("无法创建唯一模型下载 staging。");

                RobotLog log(OnlineServicesLogPath(), false);
                FtpClient ftp(
                    &log, config.host, config.port,
                    config.user, config.password);
                ftp.setMessageBoxesEnabled(false);
                RobotModelRemoteCatalog::Catalog latestCatalog;
                QString latestCatalogName;
                if (error.isEmpty()
                    && !ReadLatestRemoteCatalog(
                        ftp, temporary.path(), &m_remoteCancel,
                        latestCatalog, latestCatalogName, error))
                {
                    // error 已填写。
                }
                RobotModelRemoteCatalog::ModelRecord latestRecord;
                bool foundLatest = false;
                if (error.isEmpty())
                {
                    const auto found = std::find_if(
                        latestCatalog.models.cbegin(),
                        latestCatalog.models.cend(),
                        [&selected](const auto& record)
                        {
                            return record.model.modelId
                                == selected.model.modelId;
                        });
                    if (found == latestCatalog.models.cend())
                    {
                        error = QStringLiteral(
                            "服务器最新清单已不包含所选型号，请刷新后重试。");
                    }
                    else
                    {
                        latestRecord = *found;
                        foundLatest = true;
                        if (latestRecord.model.sourceStep.sha256
                                != selected.model.sourceStep.sha256
                            || latestRecord.model.collision.profileKeySha256
                                != selected.model.collision.profileKeySha256
                            || latestRecord.model.collisionPayloadSha256
                                != selected.model.collisionPayloadSha256
                            || latestRecord.preview.sha256
                                != selected.preview.sha256)
                        {
                            error = QStringLiteral(
                                "服务器型号清单在选择后发生变化，请刷新并重新确认。");
                        }
                    }
                }

                RemoteFileMap assetFiles;
                RemoteFileMap collisionFiles;
                RemoteFileMap previewFiles;
                if (error.isEmpty()
                    && foundLatest
                    && (!ListRemoteDirectory(
                            ftp,
                            RobotModelRemoteCatalog::RemoteAssetsDirectory(),
                            assetFiles, &m_remoteCancel, error)
                        || !ListRemoteDirectory(
                            ftp,
                            RobotModelRemoteCatalog::RemoteCollisionDirectory(),
                            collisionFiles, &m_remoteCancel, error)
                        || !ListRemoteDirectory(
                            ftp,
                            RobotModelRemoteCatalog::RemotePreviewDirectory(),
                            previewFiles, &m_remoteCancel, error)
                        || !ResolveRemoteFile(
                            assetFiles,
                            latestRecord.model.sourceStep.storedFileName,
                            latestRecord.model.sourceStep.sizeBytes,
                            QStringLiteral("STEP"), error)
                        || !ResolveRemoteFile(
                            collisionFiles,
                            latestRecord.model.collision.storedFileName,
                            latestRecord.model.collision.sizeBytes,
                            QStringLiteral("碰撞简模"), error)
                        || !ResolveRemoteFile(
                            previewFiles,
                            latestRecord.preview.storedFileName,
                            latestRecord.preview.sizeBytes,
                            QStringLiteral("预览图"), error)))
                {
                    // error 已填写。
                }

                const QString localStep = QDir(temporary.path()).filePath(
                    latestRecord.model.modelId + QStringLiteral(".step"));
                const QString localCollision = QDir(temporary.path()).filePath(
                    QStringLiteral("collision.robot-aabb.json"));
                const QString localPreview = QDir(temporary.path()).filePath(
                    QStringLiteral("preview.png"));

                auto downloadOne = [
                    this, &ftp, &completed, &error
                ](
                    int index,
                    const QString& remoteDirectory,
                    const QString& fileName,
                    const QString& localPath,
                    qint64 expectedBytes,
                    qint64 maximumBytes) -> bool
                {
                    QMetaObject::invokeMethod(
                        this,
                        [this, index]()
                        {
                            UpdateTransferUi(
                                index, 0, 0, 0, 0.0, 0,
                                QStringLiteral("准备受限 FTP 下载"),
                                QStringLiteral("正在下载"));
                        },
                        Qt::QueuedConnection);
                    const auto started = std::chrono::steady_clock::now();
                    auto lastUi =
                        std::chrono::steady_clock::time_point{};
                    const bool downloaded = ftp.downloadFileBounded(
                        Utf8Path(JoinRemote(remoteDirectory, fileName)),
                        QDir::toNativeSeparators(localPath)
                            .toLocal8Bit().toStdString(),
                        static_cast<unsigned long long>(expectedBytes),
                        static_cast<unsigned long long>(maximumBytes),
                        &m_remoteCancel,
                        [this, index, started, &lastUi](
                            unsigned long long received,
                            unsigned long long total)
                        {
                            if (m_remoteCancel.load()) return;
                            const auto now =
                                std::chrono::steady_clock::now();
                            if (lastUi
                                    != std::chrono::steady_clock::time_point{}
                                && received < total
                                && std::chrono::duration_cast<
                                    std::chrono::milliseconds>(
                                    now - lastUi).count() < 120)
                            {
                                return;
                            }
                            lastUi = now;
                            const double elapsed = std::max(
                                0.001,
                                std::chrono::duration<double>(
                                    now - started).count());
                            const double speed =
                                static_cast<double>(received) / elapsed;
                            const int eta = speed > 1.0 && received < total
                                ? static_cast<int>(std::ceil(
                                    static_cast<double>(total - received)
                                    / speed))
                                : 0;
                            QMetaObject::invokeMethod(
                                this,
                                [this, index, received, total, speed, eta]()
                                {
                                    UpdateTransferUi(
                                        index, 0, received, total,
                                        speed, eta,
                                        QStringLiteral("FTP 传输中"),
                                        QStringLiteral("正在下载"));
                                },
                                Qt::QueuedConnection);
                        });
                    if (!downloaded)
                    {
                        error = m_remoteCancel.load()
                            ? QStringLiteral("用户取消模型下载。")
                            : QStringLiteral("FTP 下载失败：%1")
                                .arg(fileName);
                        return false;
                    }
                    ++completed;
                    QMetaObject::invokeMethod(
                        this,
                        [this, index, completed, expectedBytes]()
                        {
                            UpdateTransferUi(
                                index, completed,
                                expectedBytes, expectedBytes,
                                0.0, 0,
                                QStringLiteral("下载大小回读通过"),
                                QStringLiteral("正在验证"));
                        },
                        Qt::QueuedConnection);
                    return true;
                };

                if (error.isEmpty()
                    && !downloadOne(
                        0,
                        RobotModelRemoteCatalog::RemoteAssetsDirectory(),
                        latestRecord.model.sourceStep.storedFileName,
                        localStep,
                        latestRecord.model.sourceStep.sizeBytes,
                        TheoreticalRobotModelStore::MaximumAssetBytes))
                {
                    // error 已填写。
                }
                QString stepSha;
                qint64 stepBytes = 0;
                if (error.isEmpty()
                    && (!RobotModelRemoteCatalog::HashFileStable(
                            localStep,
                            TheoreticalRobotModelStore::MaximumAssetBytes,
                            stepSha, stepBytes, error)
                        || stepSha
                            != latestRecord.model.sourceStep.sha256
                        || stepBytes
                            != latestRecord.model.sourceStep.sizeBytes))
                {
                    if (error.isEmpty())
                        error = QStringLiteral(
                            "下载 STEP 的大小或 SHA-256 与服务器清单不一致。");
                }
                if (error.isEmpty())
                {
                    QMetaObject::invokeMethod(
                        this,
                        [this, stepBytes]()
                        {
                            UpdateTransferUi(
                                0, 1, stepBytes, stepBytes,
                                0.0, 0,
                                QStringLiteral("STEP SHA-256 校验通过"),
                                QStringLiteral("完成"));
                        },
                        Qt::QueuedConnection);
                }

                if (error.isEmpty()
                    && !downloadOne(
                        1,
                        RobotModelRemoteCatalog::RemoteCollisionDirectory(),
                        latestRecord.model.collision.storedFileName,
                        localCollision,
                        latestRecord.model.collision.sizeBytes,
                        RobotCollisionEnvelopeStore::MaximumEnvelopeFileBytes))
                {
                    // error 已填写。
                }
                QByteArray collisionBytes;
                if (error.isEmpty()
                    && !ReadFileBounded(
                        localCollision,
                        RobotCollisionEnvelopeStore::MaximumEnvelopeFileBytes,
                        collisionBytes, error))
                {
                    // error 已填写。
                }

                if (error.isEmpty()
                    && !downloadOne(
                        2,
                        RobotModelRemoteCatalog::RemotePreviewDirectory(),
                        latestRecord.preview.storedFileName,
                        localPreview,
                        latestRecord.preview.sizeBytes,
                        RobotModelRemoteCatalog::MaximumPreviewBytes))
                {
                    // error 已填写。
                }
                QByteArray previewBytes;
                QImage previewImage;
                if (error.isEmpty()
                    && (!ReadFileBounded(
                            localPreview,
                            RobotModelRemoteCatalog::MaximumPreviewBytes,
                            previewBytes, error)
                        || !RobotModelRemoteCatalog::ValidatePreview(
                            previewBytes, latestRecord.preview,
                            previewImage, error)))
                {
                    // error 已填写。
                }
                if (error.isEmpty())
                {
                    QMetaObject::invokeMethod(
                        this,
                        [this, latestRecord]()
                        {
                            UpdateTransferUi(
                                2, 3,
                                latestRecord.preview.sizeBytes,
                                latestRecord.preview.sizeBytes,
                                0.0, 0,
                                QStringLiteral("预览图 SHA-256 与 PNG 校验通过"),
                                QStringLiteral("完成"));
                        },
                        Qt::QueuedConnection);
                }

                TheoreticalRobotModelStore::Asset importedStep;
                RobotCollisionEnvelopeStore::StoredAsset importedCollision;
                RobotCollisionEnvelopeStore::EnvelopeSet importedEnvelope;
                const int cancelState =
                    m_remoteCancelState.exchange(0);
                if (error.isEmpty()
                    && (cancelState == 2 || m_remoteCancel.load()))
                    error = QStringLiteral("用户取消模型下载。");
                if (error.isEmpty())
                {
                    QMetaObject::invokeMethod(
                        this,
                        [this]()
                        {
                            m_transferStateLabel->setText(
                                QStringLiteral("正在执行本地可信型号门禁…"));
                            m_transferDetailLabel->setText(
                                QStringLiteral(
                                    "传输已完成，正在原子导入 STEP、简模并登记型号。"));
                            m_serverCancelButton->setEnabled(false);
                        },
                        Qt::QueuedConnection);
                }
                if (error.isEmpty()
                    && (!TheoreticalRobotModelStore::ImportStepFile(
                            localStep, importedStep, error, false)
                        || importedStep.sha256
                            != latestRecord.model.sourceStep.sha256
                        || importedStep.sizeBytes
                            != latestRecord.model.sourceStep.sizeBytes))
                {
                    if (error.isEmpty())
                        error = QStringLiteral(
                            "下载 STEP 导入本地受控库后的身份不一致。");
                }
                if (error.isEmpty()
                    && (!RobotCollisionEnvelopeStore::ImportFile(
                            localCollision,
                            importedCollision, importedEnvelope, error)
                        || importedCollision.profileKeySha256
                            != latestRecord.model.collision.profileKeySha256
                        || importedCollision.sourceStepSha256
                            != latestRecord.model.sourceStep.sha256
                        || importedEnvelope.payloadSha256
                            != latestRecord.model.collisionPayloadSha256))
                {
                    if (error.isEmpty())
                        error = QStringLiteral(
                            "下载碰撞简模导入本地库后的身份不一致。");
                }
                if (error.isEmpty()
                    && !RobotModelCatalogStore::RegisterValidatedModel(
                        latestRecord.model.modelId,
                        latestRecord.model.displayName,
                        latestRecord.model.adapterId,
                        latestRecord.model.sourceRobotType,
                        importedStep,
                        importedCollision,
                        registeredModel,
                        error))
                {
                    // error 已填写；未通过代码内可信 revision 时不会登记。
                }
                ok = error.isEmpty() && !m_remoteCancel.load();
                cancelled = m_remoteCancel.load();
            }
            catch (const std::exception& exception)
            {
                error = QStringLiteral("下载机器人模型发生异常：%1")
                    .arg(QString::fromLocal8Bit(exception.what()).simplified());
            }
            catch (...)
            {
                error = QStringLiteral("下载机器人模型发生未知异常。");
            }
            cancelled = cancelled || m_remoteCancel.load();
            QMetaObject::invokeMethod(
                this,
                [this, ok, cancelled, completed, selected,
                    registeredModel, error]()
                {
                    FinishRemoteWorker();
                    const QString message = ok
                        ? QStringLiteral(
                            "型号 %1 已下载并通过本地可信型号、STEP 和碰撞简模门禁。")
                            .arg(registeredModel.modelId)
                        : cancelled
                            ? QStringLiteral("型号 %1 下载已取消。")
                                .arg(selected.model.modelId)
                            : QStringLiteral("型号 %1 下载失败：%2")
                                .arg(selected.model.modelId, error);
                    m_serverStatusLabel->setText(message);
                    m_serverStatusLabel->setStyleSheet(ok
                        ? QStringLiteral(
                            "QLabel { color:#a5d6a7; padding:5px; }")
                        : QStringLiteral(
                            "QLabel { color:#ff8a80; padding:5px; }"));
                    FinishTransferUi(completed, !ok, message);
                    if (ok)
                        RefreshCatalog(registeredModel.modelId);
                },
                Qt::QueuedConnection);
        });
}
