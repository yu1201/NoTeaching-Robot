#include "OnlineServicesDialog.h"

#include "BrandingConfig.h"
#include "FtpClient.h"
#include "OnlineServicesConfig.h"
#include "RobotLog.h"
#include "ScanDataUploader.h"

#include <QComboBox>
#include <QFileInfo>
#include <QPointer>

#include <thread>

#include <QtCore/private/qzipreader_p.h>

#include <QApplication>
#include <QCheckBox>
#include <QCryptographicHash>
#include <QDir>
#include <QFile>
#include <QFileDialog>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QCloseEvent>
#include <QMessageBox>
#include <QNetworkAccessManager>
#include <QNetworkProxy>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QPlainTextEdit>
#include <QProcess>
#include <QProgressBar>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>

namespace
{
	QString DownloadTempDir()
	{
		return QStringLiteral("Temp/OnlineUpdate");
	}
}

OnlineServicesDialog::OnlineServicesDialog(ScanDataUploader* uploader,
	std::function<bool()> flowRunningGuard,
	bool aboutMode,
	bool remoteBrowseAllowed,
	QWidget* parent)
	: QDialog(parent)
	, m_uploader(uploader)
	, m_flowRunningGuard(std::move(flowRunningGuard))
	, m_aboutMode(aboutMode)
	, m_remoteBrowseAllowed(remoteBrowseAllowed)
{
	setWindowTitle(m_aboutMode ? QStringLiteral("关于") : QStringLiteral("在线服务"));
	m_network = new QNetworkAccessManager(this);
	// 自建 OTA 服务器是直连 IP，绝不走系统代理：现场设备若装了代理软件(clash/v2ray 等)，
	// Qt 默认吃系统代理会把请求转给代理，代理连不到自建服务器就回 502 Bad Gateway。
	// 显式 NoProxy 让检查更新/下载都直连服务器。（FTP 上传走 WinINet DIRECT，本就不走代理。）
	m_network->setProxy(QNetworkProxy::NoProxy);
	BuildUi();
	LoadConfigToUi();

	if (m_uploader != nullptr)
	{
		connect(m_uploader, &ScanDataUploader::uploadStatus, this, &OnlineServicesDialog::AppendLog);
		connect(m_uploader, &ScanDataUploader::pendingChanged, this, [this](int)
			{
				if (m_pendingListWidget != nullptr && m_uploader != nullptr)
				{
					m_pendingListWidget->clear();
					m_pendingListWidget->addItems(m_uploader->PendingList());
				}
			});
	}

	if (m_aboutMode)
	{
		// 关于页打开即自动查一次新版（结果落在「最新版本/更新说明」区，失败只记日志）。
		QTimer::singleShot(0, this, [this]() { CheckForUpdate(); });
	}
}

void OnlineServicesDialog::BuildUi()
{
	QVBoxLayout* mainLayout = new QVBoxLayout(this);
	mainLayout->setContentsMargins(16, 14, 16, 14);
	mainLayout->setSpacing(10);

	if (m_aboutMode)
	{
		// 关于页头部：软件名 + 安装目录（升级区自身展示当前/最新版本与更新内容）。
		QLabel* appNameLabel = new QLabel(QApplication::applicationName(), this);
		appNameLabel->setStyleSheet("font-size: 22px; font-weight: bold; color: #9ED8DB;");
		QLabel* installDirLabel = new QLabel(
			QStringLiteral("安装目录：%1").arg(QDir::toNativeSeparators(QApplication::applicationDirPath())), this);
		installDirLabel->setStyleSheet("color: #7E9AA6; font-size: 12px;");
		mainLayout->addWidget(appNameLabel);
		mainLayout->addWidget(installDirLabel);
	}

	// —— 在线升级 ——
	QGroupBox* updateGroup = new QGroupBox(QStringLiteral("在线升级"), this);
	QGridLayout* updateLayout = new QGridLayout(updateGroup);
	m_currentVersionLabel = new QLabel(QStringLiteral("当前版本：%1（%2通道）")
		.arg(QApplication::applicationVersion())
		.arg(UpdateChannel() == QStringLiteral("brand") ? QStringLiteral("品牌") : QStringLiteral("中性")), this);
	m_latestVersionLabel = new QLabel(QStringLiteral("最新版本：未检查"), this);
	m_checkUpdateBtn = new QPushButton(QStringLiteral("检查更新"), this);
	m_checkUpdateBtn->setMinimumHeight(44);
	m_downloadInstallBtn = new QPushButton(QStringLiteral("下载并安装"), this);
	m_downloadInstallBtn->setMinimumHeight(44);
	m_downloadInstallBtn->setEnabled(false);
	m_downloadProgress = new QProgressBar(this);
	m_downloadProgress->setRange(0, 100);
	m_downloadProgress->setValue(0);
	m_updateNotes = new QPlainTextEdit(this);
	m_updateNotes->setReadOnly(true);
	m_updateNotes->setPlaceholderText(QStringLiteral("更新说明将在检查后显示…"));
	m_updateNotes->setMaximumHeight(110);
	updateLayout->addWidget(m_currentVersionLabel, 0, 0);
	updateLayout->addWidget(m_latestVersionLabel, 0, 1);
	updateLayout->addWidget(m_checkUpdateBtn, 0, 2);
	updateLayout->addWidget(m_downloadInstallBtn, 0, 3);
	updateLayout->addWidget(m_downloadProgress, 1, 0, 1, 4);
	updateLayout->addWidget(m_updateNotes, 2, 0, 1, 4);
	mainLayout->addWidget(updateGroup);

	// —— 数据上传 ——
	QGroupBox* uploadGroup = new QGroupBox(QStringLiteral("扫描数据上传"), this);
	QGridLayout* uploadLayout = new QGridLayout(uploadGroup);
	m_autoUploadCheck = new QCheckBox(QStringLiteral("扫描流程完成后自动上传"), this);
	m_uploadNowBtn = new QPushButton(QStringLiteral("立即上传待传项"), this);
	m_uploadNowBtn->setMinimumHeight(40);
	m_uploadPickBtn = new QPushButton(QStringLiteral("选择案例上传…"), this);
	m_uploadPickBtn->setMinimumHeight(40);
	m_pendingListWidget = new QListWidget(this);
	m_pendingListWidget->setMaximumHeight(110);
	uploadLayout->addWidget(m_autoUploadCheck, 0, 0);
	uploadLayout->addWidget(m_uploadNowBtn, 0, 1);
	uploadLayout->addWidget(m_uploadPickBtn, 0, 2);
	uploadLayout->addWidget(new QLabel(QStringLiteral("待上传队列："), this), 1, 0);
	uploadLayout->addWidget(m_pendingListWidget, 2, 0, 1, 3);
	uploadGroup->setVisible(!m_aboutMode);
	mainLayout->addWidget(uploadGroup);

	// —— 服务器配置 ——
	QGroupBox* configGroup = new QGroupBox(QStringLiteral("服务器配置"), this);
	QGridLayout* configLayout = new QGridLayout(configGroup);
	m_updateBaseUrlEdit = new QLineEdit(this);
	m_ftpHostEdit = new QLineEdit(this);
	m_ftpPortEdit = new QLineEdit(this);
	m_ftpPortEdit->setMaximumWidth(90);
	m_ftpUserEdit = new QLineEdit(this);
	m_ftpPasswordEdit = new QLineEdit(this);
	m_ftpPasswordEdit->setEchoMode(QLineEdit::Password);
	m_deviceNameEdit = new QLineEdit(this);
	QPushButton* saveConfigBtn = new QPushButton(QStringLiteral("保存配置"), this);
	saveConfigBtn->setMinimumHeight(40);
	configLayout->addWidget(new QLabel(QStringLiteral("升级源地址"), this), 0, 0);
	configLayout->addWidget(m_updateBaseUrlEdit, 0, 1, 1, 3);
	configLayout->addWidget(new QLabel(QStringLiteral("FTP 服务器"), this), 1, 0);
	configLayout->addWidget(m_ftpHostEdit, 1, 1);
	configLayout->addWidget(new QLabel(QStringLiteral("端口"), this), 1, 2);
	configLayout->addWidget(m_ftpPortEdit, 1, 3);
	configLayout->addWidget(new QLabel(QStringLiteral("FTP 账号"), this), 2, 0);
	configLayout->addWidget(m_ftpUserEdit, 2, 1);
	configLayout->addWidget(new QLabel(QStringLiteral("FTP 密码"), this), 2, 2);
	configLayout->addWidget(m_ftpPasswordEdit, 2, 3);
	configLayout->addWidget(new QLabel(QStringLiteral("设备名"), this), 3, 0);
	configLayout->addWidget(m_deviceNameEdit, 3, 1);
	configLayout->addWidget(saveConfigBtn, 3, 3);
	configGroup->setVisible(!m_aboutMode);
	mainLayout->addWidget(configGroup);

	// —— 远程数据（admin）：浏览/下载各设备上传到服务器的扫描数据 ——
	if (!m_aboutMode && m_remoteBrowseAllowed)
	{
		QGroupBox* remoteGroup = new QGroupBox(QStringLiteral("远程数据（管理员）"), this);
		QGridLayout* remoteLayout = new QGridLayout(remoteGroup);
		m_remoteRefreshBtn = new QPushButton(QStringLiteral("刷新设备列表"), this);
		m_remoteRefreshBtn->setMinimumHeight(40);
		m_remoteDeviceCombo = new QComboBox(this);
		m_remoteDeviceCombo->setMinimumWidth(220);
		m_remoteFileList = new QListWidget(this);
		m_remoteFileList->setSelectionMode(QAbstractItemView::ExtendedSelection);
		m_remoteFileList->setMaximumHeight(130);
		m_remoteDownloadBtn = new QPushButton(QStringLiteral("下载选中到本地"), this);
		m_remoteDownloadBtn->setMinimumHeight(40);
		m_remoteDownloadBtn->setToolTip(QStringLiteral("下载并自动解压到 Result\\Remote\\<设备>\\，可用点云查看等工具直接打开。"));
		remoteLayout->addWidget(m_remoteRefreshBtn, 0, 0);
		remoteLayout->addWidget(new QLabel(QStringLiteral("设备："), this), 0, 1);
		remoteLayout->addWidget(m_remoteDeviceCombo, 0, 2);
		remoteLayout->addWidget(m_remoteDownloadBtn, 0, 3);
		remoteLayout->addWidget(m_remoteFileList, 1, 0, 1, 4);
		mainLayout->addWidget(remoteGroup);

		connect(m_remoteRefreshBtn, &QPushButton::clicked, this, [this]()
			{
				SaveConfigFromUi();
				RefreshRemoteDevices();
			});
		connect(m_remoteDeviceCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int)
			{
				RefreshRemoteFiles();
			});
		connect(m_remoteDownloadBtn, &QPushButton::clicked, this, [this]()
			{
				DownloadSelectedRemoteFiles();
			});
	}

	m_logText = new QPlainTextEdit(this);
	m_logText->setReadOnly(true);
	m_logText->setPlaceholderText(QStringLiteral("在线服务日志…"));
	mainLayout->addWidget(m_logText, 1);

	connect(m_checkUpdateBtn, &QPushButton::clicked, this, [this]() { CheckForUpdate(); });
	connect(m_downloadInstallBtn, &QPushButton::clicked, this, [this]()
		{
			if (m_downloadedPath.isEmpty())
			{
				StartDownload();
			}
			else
			{
				InstallDownloadedPackage();
			}
		});
	connect(m_autoUploadCheck, &QCheckBox::toggled, this, [this](bool checked)
		{
			OnlineServicesConfig::SetAutoUploadEnabled(checked);
			AppendLog(checked ? QStringLiteral("已开启扫描完成自动上传。") : QStringLiteral("已关闭扫描完成自动上传。"));
		});
	connect(m_uploadNowBtn, &QPushButton::clicked, this, [this]()
		{
			SaveConfigFromUi();  // 顺手保存，避免填了账号没点保存直接传
			if (m_uploader != nullptr)
			{
				m_uploader->TriggerUploadNow();
			}
		});
	connect(m_uploadPickBtn, &QPushButton::clicked, this, [this]()
		{
			const QString dir = QFileDialog::getExistingDirectory(this,
				QStringLiteral("选择要上传的结果案例目录"), QStringLiteral("Result"));
			if (!dir.isEmpty() && m_uploader != nullptr)
			{
				SaveConfigFromUi();
				m_uploader->QueueUpload(dir);
			}
		});
	connect(saveConfigBtn, &QPushButton::clicked, this, [this]()
		{
			SaveConfigFromUi();
			AppendLog(QStringLiteral("服务器配置已保存。"));
		});

	resize(m_aboutMode ? 640 : 860, m_aboutMode ? 560 : 780);
}

void OnlineServicesDialog::LoadConfigToUi()
{
	m_updateBaseUrlEdit->setText(OnlineServicesConfig::UpdateBaseUrl());
	m_ftpHostEdit->setText(OnlineServicesConfig::FtpHost());
	m_ftpPortEdit->setText(QString::number(OnlineServicesConfig::FtpPort()));
	m_ftpUserEdit->setText(OnlineServicesConfig::FtpUser());
	m_ftpPasswordEdit->setText(OnlineServicesConfig::FtpPassword());
	m_deviceNameEdit->setText(OnlineServicesConfig::DeviceName());
	m_autoUploadCheck->setChecked(OnlineServicesConfig::AutoUploadEnabled());
	if (m_uploader != nullptr && m_pendingListWidget != nullptr)
	{
		m_pendingListWidget->clear();
		m_pendingListWidget->addItems(m_uploader->PendingList());
	}
}

void OnlineServicesDialog::SaveConfigFromUi()
{
	OnlineServicesConfig::SetUpdateBaseUrl(m_updateBaseUrlEdit->text().trimmed());
	OnlineServicesConfig::SetFtpHost(m_ftpHostEdit->text().trimmed());
	bool ok = false;
	const int port = m_ftpPortEdit->text().trimmed().toInt(&ok);
	OnlineServicesConfig::SetFtpPort(ok ? port : 21);
	OnlineServicesConfig::SetFtpUser(m_ftpUserEdit->text().trimmed());
	OnlineServicesConfig::SetFtpPassword(m_ftpPasswordEdit->text());
	OnlineServicesConfig::SetDeviceName(m_deviceNameEdit->text().trimmed());
}

QString OnlineServicesDialog::UpdateChannel() const
{
	// 通道按品牌覆盖是否生效判定：品牌包随安装包分发 branding/branding.ini（打包脚本仅在
	// branding/ 被 git 跟踪的品牌分支才拷入），装到设备后 IsActive()=true → brand 通道；
	// 中性包不含 branding/ → neutral。exe 文件名两通道相同(QtWidgetsApplication4.exe)不可用作判据。
	// 若品牌设备 branding/ 被误删，程序整体已退化为中性(名称/图标全默认)，走 neutral 通道装中性包与之一致。
	return BrandingConfig::IsActive() ? QStringLiteral("brand") : QStringLiteral("neutral");
}

void OnlineServicesDialog::CheckForUpdate()
{
	SaveConfigFromUi();
	const QString url = OnlineServicesConfig::UpdateBaseUrl() + "/" + UpdateChannel() + "/latest.json";
	AppendLog(QStringLiteral("检查更新：%1").arg(url));
	m_checkUpdateBtn->setEnabled(false);

	QNetworkRequest request{ QUrl(url) };
	request.setTransferTimeout(15000);
	QNetworkReply* reply = m_network->get(request);
	connect(reply, &QNetworkReply::finished, this, [this, reply]() { OnManifestReply(reply); });
}

void OnlineServicesDialog::OnManifestReply(QNetworkReply* reply)
{
	reply->deleteLater();
	m_checkUpdateBtn->setEnabled(true);
	if (reply->error() != QNetworkReply::NoError)
	{
		m_latestVersionLabel->setText(QStringLiteral("最新版本：检查失败"));
		AppendLog(QStringLiteral("检查更新失败：%1").arg(reply->errorString()));
		return;
	}

	const QJsonObject obj = QJsonDocument::fromJson(reply->readAll()).object();
	m_remoteVersion = obj.value(QStringLiteral("version")).toString();
	m_remoteFile = obj.value(QStringLiteral("file")).toString();
	m_remoteSha256 = obj.value(QStringLiteral("sha256")).toString().toLower();
	const QString notes = obj.value(QStringLiteral("notes")).toString();

	// 增量补丁字段（可选）：仅含变更文件的 zip（当前=主程序 exe），优先于全量安装包。
	const QJsonObject patch = obj.value(QStringLiteral("patch")).toObject();
	m_remotePatchFile = patch.value(QStringLiteral("file")).toString();
	m_remotePatchSha256 = patch.value(QStringLiteral("sha256")).toString().toLower();
	m_remotePatchSize = static_cast<qint64>(patch.value(QStringLiteral("size")).toDouble());
	// baseMinVersion：补丁只含 exe，仅当本机版本 >= 此基线（=非 exe 载荷/DLL 与目标一致的最低版本）
	// 时增量才安全；更老的设备 DLL 可能不一致，自动回退全量安装。缺该字段时按旧行为（有补丁即用）。
	const QString patchBaseMinVersion = patch.value(QStringLiteral("baseMinVersion")).toString().trimmed();
	m_usePatch = !m_remotePatchFile.isEmpty();
	if (m_usePatch && !patchBaseMinVersion.isEmpty()
		&& CompareVersions(QApplication::applicationVersion(), patchBaseMinVersion) < 0)
	{
		m_usePatch = false;
		AppendLog(QStringLiteral("本机版本 %1 低于增量补丁基线 %2（DLL 可能不一致），改用全量安装。")
			.arg(QApplication::applicationVersion(), patchBaseMinVersion));
	}

	if (m_remoteVersion.isEmpty() || m_remoteFile.isEmpty())
	{
		AppendLog(QStringLiteral("更新清单格式错误（缺 version/file 字段）。"));
		return;
	}

	m_latestVersionLabel->setText(QStringLiteral("最新版本：%1").arg(m_remoteVersion));
	m_updateNotes->setPlainText(notes);
	m_downloadedPath.clear();

	const int cmp = CompareVersions(m_remoteVersion, QApplication::applicationVersion());
	if (cmp > 0)
	{
		m_downloadInstallBtn->setEnabled(true);
		m_downloadInstallBtn->setText(m_usePatch
			? QStringLiteral("增量升级（约 %1 MB）").arg(QString::number(m_remotePatchSize / 1048576.0, 'f', 1))
			: QStringLiteral("下载并安装"));
		AppendLog(QStringLiteral("发现新版本 %1（当前 %2）%3。")
			.arg(m_remoteVersion)
			.arg(QApplication::applicationVersion())
			.arg(m_usePatch ? QStringLiteral("，支持增量升级") : QString()));
	}
	else
	{
		m_downloadInstallBtn->setEnabled(false);
		AppendLog(QStringLiteral("已是最新版本。"));
	}
}

void OnlineServicesDialog::StartDownload()
{
	if (m_downloading || m_remoteFile.isEmpty())
	{
		return;
	}
	const QString fileName = m_usePatch ? m_remotePatchFile : m_remoteFile;
	const QString url = OnlineServicesConfig::UpdateBaseUrl() + "/" + UpdateChannel() + "/" + fileName;
	AppendLog(QStringLiteral("开始下载%1：%2").arg(m_usePatch ? QStringLiteral("增量补丁") : QStringLiteral("安装包")).arg(url));
	m_downloading = true;
	m_downloadInstallBtn->setEnabled(false);
	m_downloadProgress->setValue(0);

	QNetworkRequest request{ QUrl(url) };
	request.setTransferTimeout(10 * 60 * 1000);  // 大文件：10 分钟超时
	QNetworkReply* reply = m_network->get(request);
	connect(reply, &QNetworkReply::downloadProgress, this, [this](qint64 received, qint64 total)
		{
			if (total > 0)
			{
				m_downloadProgress->setValue(static_cast<int>(received * 100 / total));
			}
		});
	connect(reply, &QNetworkReply::finished, this, [this, reply]() { OnDownloadFinished(reply); });
}

void OnlineServicesDialog::OnDownloadFinished(QNetworkReply* reply)
{
	reply->deleteLater();
	m_downloading = false;
	if (reply->error() != QNetworkReply::NoError)
	{
		m_downloadInstallBtn->setEnabled(true);
		AppendLog(QStringLiteral("下载失败：%1").arg(reply->errorString()));
		return;
	}

	const QByteArray data = reply->readAll();

	// SHA256 校验（清单里提供时才校验；不提供也放行但记日志）。
	const QString expectedSha = m_usePatch ? m_remotePatchSha256 : m_remoteSha256;
	if (!expectedSha.isEmpty())
	{
		const QString actual = QString::fromLatin1(
			QCryptographicHash::hash(data, QCryptographicHash::Sha256).toHex()).toLower();
		if (actual != expectedSha)
		{
			m_downloadInstallBtn->setEnabled(true);
			AppendLog(QStringLiteral("SHA256 校验失败，文件可能损坏或被篡改，已丢弃。"));
			return;
		}
		AppendLog(QStringLiteral("SHA256 校验通过。"));
	}
	else
	{
		AppendLog(QStringLiteral("清单未提供 SHA256，跳过校验。"));
	}

	QDir().mkpath(DownloadTempDir());
	m_downloadedPath = DownloadTempDir() + "/" + (m_usePatch ? m_remotePatchFile : m_remoteFile);
	QFile out(m_downloadedPath);
	if (!out.open(QIODevice::WriteOnly))
	{
		m_downloadedPath.clear();
		m_downloadInstallBtn->setEnabled(true);
		AppendLog(QStringLiteral("保存安装包失败：无法写入 %1").arg(m_downloadedPath));
		return;
	}
	out.write(data);
	out.close();

	m_downloadProgress->setValue(100);
	m_downloadInstallBtn->setEnabled(true);
	m_downloadInstallBtn->setText(QStringLiteral("立即安装并重启"));
	AppendLog(QStringLiteral("下载完成：%1（%2 MB）。点击「立即安装并重启」完成升级。")
		.arg(m_downloadedPath).arg(QString::number(data.size() / 1048576.0, 'f', 1)));
	InstallDownloadedPackage();
}

void OnlineServicesDialog::InstallDownloadedPackage()
{
	if (m_downloadedPath.isEmpty() || !QFile::exists(m_downloadedPath))
	{
		AppendLog(QStringLiteral("没有已下载的安装包。"));
		return;
	}
	if (m_flowRunningGuard && m_flowRunningGuard())
	{
		QMessageBox::warning(this, QStringLiteral("在线升级"),
			QStringLiteral("焊接/扫描流程正在运行，禁止此时安装升级。\n请等流程结束后再点「立即安装并重启」。"));
		return;
	}

	const QMessageBox::StandardButton ret = QMessageBox::question(
		this,
		QStringLiteral("在线升级"),
		(m_usePatch
			? QStringLiteral("即将增量升级到 %1：\n程序将退出并替换更新文件（约几秒），然后自动重新启动。\n"
				"现场数据（Data/Result/Log）不受影响。\n\n确认现在升级？")
			: QStringLiteral("即将升级到 %1：\n程序将退出并静默安装新版本，安装完成后自动重新启动。\n"
				"现场数据（Data/Result/Log）不受影响。\n\n确认现在升级？")).arg(m_remoteVersion),
		QMessageBox::Ok | QMessageBox::Cancel,
		QMessageBox::Cancel);
	if (ret != QMessageBox::Ok)
	{
		return;
	}

	const QString payload = QDir::toNativeSeparators(QFileInfo(m_downloadedPath).absoluteFilePath());
	const QString appPath = QDir::toNativeSeparators(QCoreApplication::applicationFilePath());
	const QString bootstrapPath = QDir::toNativeSeparators(
		QFileInfo(DownloadTempDir() + "/apply_update.cmd").absoluteFilePath());
	QString script;
	if (m_usePatch)
	{
		// 增量：等本进程退出（按映像名轮询）→ tar 解压补丁 zip 覆盖安装目录（Win10+ 自带 bsdtar 支持 zip）→ 重启。
		const QString exeName = QFileInfo(appPath).fileName();
		const QString appDir = QDir::toNativeSeparators(QCoreApplication::applicationDirPath());
		script = QString(
			"@echo off\r\n"
			":wait\r\n"
			"tasklist /FI \"IMAGENAME eq %1\" | find /I \"%1\" >nul && (timeout /t 1 /nobreak >nul & goto wait)\r\n"
			"tar -xf \"%2\" -C \"%3\"\r\n"
			"start \"\" \"%4\"\r\n")
			.arg(exeName).arg(payload).arg(appDir).arg(appPath);
	}
	else
	{
		// 全量：/SILENT 静默安装；Inno [Run] postinstall 在 silent 下被跳过，装完由批处理拉起新版。
		script = QString("@echo off\r\n\"%1\" /SILENT\r\nstart \"\" \"%2\"\r\n").arg(payload).arg(appPath);
	}

	QFile bootstrap(bootstrapPath);
	if (!bootstrap.open(QIODevice::WriteOnly | QIODevice::Truncate))
	{
		AppendLog(QStringLiteral("写入升级引导脚本失败，请到 %1 手动处理。").arg(payload));
		return;
	}
	// 批处理按系统 ANSI 代码页写（cmd 默认按 ANSI 解析，路径含中文时 toLatin1 会坏）。
	bootstrap.write(script.toLocal8Bit());
	bootstrap.close();

	// 记下目标版本：重启后主窗口自检「实际版本 == 目标?」，不符即告警（补丁/安装没生效）。
	OnlineServicesConfig::SetPendingUpdateTargetVersion(m_remoteVersion);

	AppendLog(QStringLiteral("开始升级：%1").arg(payload));
	if (!QProcess::startDetached(QStringLiteral("cmd.exe"), { QStringLiteral("/c"), bootstrapPath }))
	{
		OnlineServicesConfig::SetPendingUpdateTargetVersion(QString());  // 没启动成功，撤销记录
		AppendLog(QStringLiteral("启动升级脚本失败，请到 %1 手动处理。").arg(payload));
		return;
	}
	QApplication::quit();
}

void OnlineServicesDialog::AppendLog(const QString& text)
{
	if (m_logText != nullptr)
	{
		m_logText->appendPlainText(text);
	}
}

void OnlineServicesDialog::closeEvent(QCloseEvent* event)
{
	// 关界面时若正在上传：问用户后台继续还是停止（停止则清服务器半截文件）。
	if (m_uploader != nullptr && m_uploader->IsBusy())
	{
		const QMessageBox::StandardButton ret = QMessageBox::question(
			this,
			QStringLiteral("上传进行中"),
			QStringLiteral("扫描数据还在上传。\n\n「后台继续」：关闭本界面，上传转后台继续。\n"
				"「停止上传」：中止上传并删除服务器上未传完的半截文件。"),
			QMessageBox::Yes | QMessageBox::No,
			QMessageBox::Yes);
		if (ret == QMessageBox::No)
		{
			m_uploader->RequestCancel();  // 后台线程块间中止并删半截文件；案例留队列
			AppendLog(QStringLiteral("已请求停止上传，服务器半截文件将被清除。"));
		}
		// 无论后台继续还是停止，都放行关闭（停止是异步清理，不阻塞关窗）。
	}
	QDialog::closeEvent(event);
}

int OnlineServicesDialog::CompareVersions(const QString& lhs, const QString& rhs)
{
	// 版本形如 2026.07.08.1031：按 '.' 分段数值比较，段数不足按 0。
	const QStringList a = lhs.split('.');
	const QStringList b = rhs.split('.');
	const int n = qMax(a.size(), b.size());
	for (int i = 0; i < n; ++i)
	{
		const int va = i < a.size() ? a.at(i).toInt() : 0;
		const int vb = i < b.size() ? b.at(i).toInt() : 0;
		if (va != vb)
		{
			return va < vb ? -1 : 1;
		}
	}
	return 0;
}

// ============ 远程数据（admin）：FTP 阻塞操作跑一次性 detach 线程，回调经 QPointer 防悬空 ============

namespace
{
	struct RemoteFtpConfig
	{
		std::string host;
		int port = 21;
		std::string user;
		std::string password;
	};

	// UI 线程读配置快照（ConfigDatabase 不可跨线程）。
	RemoteFtpConfig CurrentRemoteFtpConfig()
	{
		RemoteFtpConfig cfg;
		cfg.host = OnlineServicesConfig::FtpHost().toStdString();
		cfg.port = OnlineServicesConfig::FtpPort();
		cfg.user = OnlineServicesConfig::FtpUser().toStdString();
		cfg.password = OnlineServicesConfig::FtpPassword().toStdString();
		return cfg;
	}
}

void OnlineServicesDialog::SetRemoteBusy(bool busy)
{
	m_remoteBusy = busy;
	if (m_remoteRefreshBtn != nullptr)
	{
		m_remoteRefreshBtn->setEnabled(!busy);
	}
	if (m_remoteDeviceCombo != nullptr)
	{
		m_remoteDeviceCombo->setEnabled(!busy);
	}
	if (m_remoteDownloadBtn != nullptr)
	{
		m_remoteDownloadBtn->setEnabled(!busy);
	}
}

void OnlineServicesDialog::RefreshRemoteDevices()
{
	if (m_remoteBusy)
	{
		return;
	}
	const RemoteFtpConfig cfg = CurrentRemoteFtpConfig();
	if (cfg.user.empty())
	{
		AppendLog(QStringLiteral("请先填写 FTP 账号并保存配置。"));
		return;
	}
	SetRemoteBusy(true);
	AppendLog(QStringLiteral("正在获取设备列表…"));
	QPointer<OnlineServicesDialog> self(this);
	std::thread([self, cfg]()
		{
			RobotLog log(".//Log//OnlineServices.txt", false);
			FtpClient ftp(&log, cfg.host, cfg.port, cfg.user, cfg.password);
			ftp.setMessageBoxesEnabled(false);
			std::vector<FtpRemoteFileInfo> entries;
			const bool ok = ftp.listFiles("/data", entries);
			QStringList devices;
			for (const FtpRemoteFileInfo& e : entries)
			{
				if (e.isDirectory)
				{
					devices << QString::fromStdString(e.name);
				}
			}
			QMetaObject::invokeMethod(qApp, [self, ok, devices]()
				{
					if (self == nullptr)
					{
						return;
					}
					self->SetRemoteBusy(false);
					if (!ok)
					{
						self->AppendLog(QStringLiteral("获取设备列表失败（检查 FTP 配置与网络）。"));
						return;
					}
					self->m_remoteDeviceCombo->clear();
					self->m_remoteDeviceCombo->addItems(devices);
					self->AppendLog(QStringLiteral("设备列表已刷新：共 %1 台。").arg(devices.size()));
				}, Qt::QueuedConnection);
		}).detach();
}

void OnlineServicesDialog::RefreshRemoteFiles()
{
	if (m_remoteFileList == nullptr || m_remoteDeviceCombo == nullptr)
	{
		return;
	}
	m_remoteFileList->clear();
	const QString device = m_remoteDeviceCombo->currentText().trimmed();
	if (device.isEmpty() || m_remoteBusy)
	{
		return;
	}
	const RemoteFtpConfig cfg = CurrentRemoteFtpConfig();
	SetRemoteBusy(true);
	QPointer<OnlineServicesDialog> self(this);
	std::thread([self, cfg, device]()
		{
			RobotLog log(".//Log//OnlineServices.txt", false);
			FtpClient ftp(&log, cfg.host, cfg.port, cfg.user, cfg.password);
			ftp.setMessageBoxesEnabled(false);
			std::vector<FtpRemoteFileInfo> entries;
			const std::string remoteDir = "/data/" + device.toStdString();
			const bool ok = ftp.listFiles(remoteDir, entries);
			QStringList names;
			QStringList labels;
			for (const FtpRemoteFileInfo& e : entries)
			{
				if (!e.isDirectory)
				{
					names << QString::fromStdString(e.name);
					labels << QStringLiteral("%1（%2 MB）")
						.arg(QString::fromStdString(e.name))
						.arg(QString::number(e.size / 1048576.0, 'f', 1));
				}
			}
			QMetaObject::invokeMethod(qApp, [self, ok, device, names, labels]()
				{
					if (self == nullptr)
					{
						return;
					}
					self->SetRemoteBusy(false);
					if (!ok)
					{
						self->AppendLog(QStringLiteral("获取 %1 的文件列表失败。").arg(device));
						return;
					}
					self->m_remoteFileList->clear();
					for (int i = 0; i < names.size(); ++i)
					{
						QListWidgetItem* item = new QListWidgetItem(labels.at(i), self->m_remoteFileList);
						item->setData(Qt::UserRole, names.at(i));
					}
					self->AppendLog(QStringLiteral("设备 %1：共 %2 个数据包。").arg(device).arg(names.size()));
				}, Qt::QueuedConnection);
		}).detach();
}

void OnlineServicesDialog::DownloadSelectedRemoteFiles()
{
	if (m_remoteBusy || m_remoteFileList == nullptr || m_remoteDeviceCombo == nullptr)
	{
		return;
	}
	const QString device = m_remoteDeviceCombo->currentText().trimmed();
	const QList<QListWidgetItem*> selected = m_remoteFileList->selectedItems();
	if (device.isEmpty() || selected.isEmpty())
	{
		AppendLog(QStringLiteral("请先选择设备和要下载的数据包。"));
		return;
	}
	QStringList names;
	for (const QListWidgetItem* item : selected)
	{
		names << item->data(Qt::UserRole).toString();
	}
	const RemoteFtpConfig cfg = CurrentRemoteFtpConfig();
	const QString localDir = QStringLiteral("Result/Remote/") + device;
	QDir().mkpath(localDir);
	SetRemoteBusy(true);
	AppendLog(QStringLiteral("开始下载 %1 个数据包到 %2 …").arg(names.size()).arg(QDir::toNativeSeparators(localDir)));
	QPointer<OnlineServicesDialog> self(this);
	std::thread([self, cfg, device, names, localDir]()
		{
			RobotLog log(".//Log//OnlineServices.txt", false);
			FtpClient ftp(&log, cfg.host, cfg.port, cfg.user, cfg.password);
			ftp.setMessageBoxesEnabled(false);
			int okCount = 0;
			for (const QString& name : names)
			{
				const std::string remotePath = "/data/" + device.toStdString() + "/" + name.toStdString();
				const QString localZip = localDir + "/" + name;
				const bool downloaded = ftp.downloadFile(remotePath, localZip.toStdString());
				bool extracted = false;
				if (downloaded)
				{
					// zip 内已带 <机器人>/<案例>/ 前缀，解到设备目录即可分层落地。
					QZipReader zr(localZip);
					extracted = zr.isReadable() && zr.extractAll(localDir);
					zr.close();
					if (extracted)
					{
						QFile::remove(localZip);
						++okCount;
					}
				}
				QMetaObject::invokeMethod(qApp, [self, name, downloaded, extracted]()
					{
						if (self != nullptr)
						{
							self->AppendLog(downloaded && extracted
								? QStringLiteral("已下载并解压：%1").arg(name)
								: QStringLiteral("下载失败：%1%2").arg(name)
									.arg(downloaded ? QStringLiteral("（解压失败，zip 保留）") : QString()));
						}
					}, Qt::QueuedConnection);
			}
			QMetaObject::invokeMethod(qApp, [self, okCount, names, localDir]()
				{
					if (self == nullptr)
					{
						return;
					}
					self->SetRemoteBusy(false);
					self->AppendLog(QStringLiteral("下载完成：成功 %1/%2，数据在 %3（点云查看等工具可直接打开）。")
						.arg(okCount).arg(names.size()).arg(QDir::toNativeSeparators(localDir)));
				}, Qt::QueuedConnection);
		}).detach();
}
