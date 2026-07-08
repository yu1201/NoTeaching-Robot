#include "ScanDataUploader.h"

#include "FtpClient.h"
#include "OnlineServicesConfig.h"
#include "RobotLog.h"

#include <QDir>
#include <QDirIterator>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QMetaObject>
#include <QTcpSocket>
#include <QTimer>

#include <QtCore/private/qzipwriter_p.h>

namespace
{
	constexpr int kRetryIntervalMs = 5 * 60 * 1000;  // 失败重试间隔：5 分钟

	QString UploadTempDir()
	{
		return QStringLiteral("Temp/OnlineUpload");
	}
}

ScanDataUploader::ScanDataUploader(QObject* parent)
	: QObject(parent)
{
	m_log = new RobotLog(".//Log//OnlineServices.txt", false);
	LoadPending();

	m_retryTimer = new QTimer(this);
	m_retryTimer->setInterval(kRetryIntervalMs);
	connect(m_retryTimer, &QTimer::timeout, this, [this]()
		{
			if (!m_pending.isEmpty() && !m_busy.load())
			{
				emit uploadStatus(QStringLiteral("定时重试：%1 个待传案例。").arg(m_pending.size()));
				StartWorkerIfIdle();
			}
		});
	m_retryTimer->start();
}

ScanDataUploader::~ScanDataUploader()
{
	JoinWorker();
	delete m_log;
	m_log = nullptr;
}

void ScanDataUploader::QueueUpload(const QString& caseDir)
{
	const QString normalized = QDir::cleanPath(caseDir);
	if (normalized.isEmpty() || !QDir(normalized).exists())
	{
		emit uploadStatus(QStringLiteral("上传入队失败：目录不存在 %1").arg(normalized));
		return;
	}
	if (!m_pending.contains(normalized))
	{
		m_pending.append(normalized);
		SavePending();
		emit pendingChanged(m_pending.size());
	}
	StartWorkerIfIdle();
}

void ScanDataUploader::TriggerUploadNow()
{
	if (m_pending.isEmpty())
	{
		emit uploadStatus(QStringLiteral("没有待上传的案例。"));
		return;
	}
	StartWorkerIfIdle();
}

QStringList ScanDataUploader::PendingList() const
{
	return m_pending;
}

bool ScanDataUploader::IsBusy() const
{
	return m_busy.load();
}

void ScanDataUploader::LoadPending()
{
	m_pending.clear();
	const QJsonDocument doc = QJsonDocument::fromJson(OnlineServicesConfig::PendingUploads().toUtf8());
	for (const auto& item : doc.array())
	{
		const QString path = item.toString();
		// 启动加载时清掉已被现场删除的目录，避免永远重试。
		if (!path.isEmpty() && QDir(path).exists())
		{
			m_pending.append(path);
		}
	}
}

void ScanDataUploader::SavePending()
{
	QJsonArray array;
	for (const QString& path : m_pending)
	{
		array.append(path);
	}
	OnlineServicesConfig::SetPendingUploads(QString::fromUtf8(QJsonDocument(array).toJson(QJsonDocument::Compact)));
}

void ScanDataUploader::StartWorkerIfIdle()
{
	if (m_busy.load() || m_pending.isEmpty())
	{
		return;
	}
	JoinWorker();
	m_busy.store(true);
	const QStringList snapshot = m_pending;
	// 配置在 UI 线程读好快照传入（ConfigDatabase 的 QSQLITE 连接不可跨线程使用）。
	UploadConfig config;
	config.host = OnlineServicesConfig::FtpHost().toStdString();
	config.port = OnlineServicesConfig::FtpPort();
	config.user = OnlineServicesConfig::FtpUser().toStdString();
	config.password = OnlineServicesConfig::FtpPassword().toStdString();
	config.deviceName = OnlineServicesConfig::DeviceName();
	m_worker = std::thread([this, snapshot, config]() { WorkerBody(snapshot, config); });
}

void ScanDataUploader::JoinWorker()
{
	if (m_worker.joinable())
	{
		m_worker.join();
	}
}

void ScanDataUploader::WorkerBody(const QStringList& items, const UploadConfig& config)
{
	const std::string& host = config.host;
	const int port = config.port;
	const std::string& user = config.user;
	const std::string& password = config.password;
	const QString& deviceName = config.deviceName;

	if (user.empty())
	{
		QMetaObject::invokeMethod(this, [this]()
			{
				m_busy.store(false);
				emit uploadStatus(QStringLiteral("上传未配置：请在管理页「在线服务」填写 FTP 账号。"));
			}, Qt::QueuedConnection);
		return;
	}

	// 联网预检：3 秒内探测 FTP 端口可达性。没联网/服务器不可达时快速失败，避免 WinINet
	// 连接长超时拖住后台线程与 CLI 退出前的收尾等待——数据留队列，联网后自动重试。
	{
		QTcpSocket probe;
		probe.connectToHost(QString::fromStdString(host), static_cast<quint16>(port));
		const bool reachable = probe.waitForConnected(3000);
		probe.abort();
		if (!reachable)
		{
			QMetaObject::invokeMethod(this, [this]()
				{
					m_busy.store(false);
					emit uploadStatus(QStringLiteral("服务器不可达（未联网？），本次跳过；%1 个案例留队列，联网后自动重试。")
						.arg(m_pending.size()));
				}, Qt::QueuedConnection);
			return;
		}
	}

	FtpClient ftp(m_log, host, port, user, password);
	ftp.setMessageBoxesEnabled(false);  // 后台上传不弹窗，结果走状态信号

	const std::string remoteBase = "/data/" + deviceName.toStdString();
	bool remoteDirReady = ftp.createRemoteDirRecursive(remoteBase);

	for (const QString& caseDir : items)
	{
		if (!remoteDirReady)
		{
			QMetaObject::invokeMethod(this, [this, caseDir]()
				{ OnItemFinished(caseDir, false, QStringLiteral("FTP 连接或远端目录创建失败")); }, Qt::QueuedConnection);
			continue;
		}

		// zip 名：<机器人>_<案例>.zip（Result/<Robot>/<case> 取两级）
		const QFileInfo caseInfo(caseDir);
		const QString robotName = caseInfo.dir().dirName();
		const QString zipName = QString("%1_%2.zip").arg(robotName).arg(caseInfo.fileName());
		QDir().mkpath(UploadTempDir());
		const QString zipPath = UploadTempDir() + "/" + zipName;

		QString zipError;
		if (!ZipCaseDir(caseDir, zipPath, &zipError))
		{
			QMetaObject::invokeMethod(this, [this, caseDir, zipError]()
				{ OnItemFinished(caseDir, false, QStringLiteral("压缩失败：%1").arg(zipError)); }, Qt::QueuedConnection);
			continue;
		}

		const std::string remotePath = remoteBase + "/" + zipName.toStdString();
		const bool uploaded = ftp.uploadFile(zipPath.toStdString(), remotePath, true);
		QFile::remove(zipPath);

		QMetaObject::invokeMethod(this, [this, caseDir, uploaded, zipName]()
			{
				OnItemFinished(caseDir, uploaded,
					uploaded ? QStringLiteral("已上传 %1").arg(zipName)
					         : QStringLiteral("FTP 上传失败（将定时重试）"));
			}, Qt::QueuedConnection);

		if (!uploaded)
		{
			remoteDirReady = false;  // 连接大概率已断，本轮剩余项直接留待重试
		}
	}

	QMetaObject::invokeMethod(this, [this]()
		{
			m_busy.store(false);
			if (!m_pending.isEmpty())
			{
				emit uploadStatus(QStringLiteral("本轮结束，仍有 %1 个待传案例（5 分钟后自动重试）。").arg(m_pending.size()));
			}
		}, Qt::QueuedConnection);
}

void ScanDataUploader::OnItemFinished(const QString& caseDir, bool ok, const QString& message)
{
	if (ok)
	{
		m_pending.removeAll(caseDir);
		SavePending();
		emit pendingChanged(m_pending.size());
	}
	emit uploadStatus(QStringLiteral("[%1] %2").arg(QFileInfo(caseDir).fileName()).arg(message));
}

bool ScanDataUploader::ZipCaseDir(const QString& caseDir, const QString& zipPath, QString* error)
{
	// 照 ResultArchiveDialog 的打包方式：zip 内路径保留 <机器人>/<案例>/… 两级前缀。
	const QFileInfo caseInfo(caseDir);
	const QDir rootDir(QDir::cleanPath(caseInfo.dir().absolutePath() + "/.."));

	QZipWriter zw(zipPath);
	zw.setCompressionPolicy(QZipWriter::AlwaysCompress);
	if (zw.status() != QZipWriter::NoError)
	{
		if (error != nullptr)
		{
			*error = QStringLiteral("无法创建压缩包(status=%1)").arg(int(zw.status()));
		}
		return false;
	}

	QDirIterator it(caseDir, QDir::Files | QDir::NoSymLinks, QDirIterator::Subdirectories);
	while (it.hasNext())
	{
		it.next();
		const QFileInfo fi = it.fileInfo();
		const QString rel = rootDir.relativeFilePath(fi.absoluteFilePath());
		QFile f(fi.absoluteFilePath());
		if (f.open(QIODevice::ReadOnly))
		{
			zw.addFile(rel, &f);
			f.close();
		}
	}
	zw.close();

	if (zw.status() != QZipWriter::NoError)
	{
		if (error != nullptr)
		{
			*error = QStringLiteral("压缩过程出错(status=%1)").arg(int(zw.status()));
		}
		QFile::remove(zipPath);
		return false;
	}
	return true;
}
