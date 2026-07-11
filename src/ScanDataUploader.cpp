#include "ScanDataUploader.h"

#include "AppPaths.h"
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

#include <chrono>
#include <QTimer>

#include <QtCore/private/qzipwriter_p.h>

namespace
{
	constexpr int kRetryIntervalMs = 5 * 60 * 1000;  // 失败重试间隔：5 分钟

	QString UploadTempDir()
	{
		return AppPaths::WritablePath(QStringLiteral("Temp/OnlineUpload"));
	}

	bool ResolveSafeResultCaseDir(
		const QString& caseDir,
		QString* resolvedPath = nullptr,
		QString* robotName = nullptr,
		QString* caseName = nullptr)
	{
		const QString resultRoot = AppPaths::WritablePath(QStringLiteral("Result"));
		const QString canonicalDataRoot = QDir(AppPaths::DataRootPath()).canonicalPath();
		const QString canonicalRoot = QDir(resultRoot).canonicalPath();
		const QFileInfo inputInfo(QDir::fromNativeSeparators(caseDir));
		const QString absoluteInput = inputInfo.isAbsolute()
			? inputInfo.absoluteFilePath()
			: QDir(AppPaths::DataRootPath()).absoluteFilePath(inputInfo.filePath());
		const QFileInfo absoluteInfo(absoluteInput);
		const QString canonicalInput = absoluteInfo.canonicalFilePath();
		const QString resultRelativeToData = QDir(canonicalDataRoot)
			.relativeFilePath(canonicalRoot)
			.replace(QLatin1Char('\\'), QLatin1Char('/'));
		if (canonicalDataRoot.isEmpty()
			|| canonicalRoot.isEmpty()
			|| resultRelativeToData.compare(QStringLiteral("Result"), Qt::CaseInsensitive) != 0
			|| canonicalInput.isEmpty()
			|| !absoluteInfo.isDir())
		{
			return false;
		}
		const QString relative = QDir(canonicalRoot)
			.relativeFilePath(canonicalInput)
			.replace(QLatin1Char('\\'), QLatin1Char('/'));
		const QStringList components = relative.split(QLatin1Char('/'), Qt::KeepEmptyParts);
		if (components.size() != 2
			|| !AppPaths::IsSafePathComponent(components.at(0))
			|| !AppPaths::IsSafePathComponent(components.at(1)))
		{
			return false;
		}
		if (resolvedPath != nullptr)
		{
			*resolvedPath = QDir::cleanPath(canonicalInput);
		}
		if (robotName != nullptr)
		{
			*robotName = components.at(0);
		}
		if (caseName != nullptr)
		{
			*caseName = components.at(1);
		}
		return true;
	}

	std::string OnlineServicesLogPath()
	{
		return QDir::toNativeSeparators(
			AppPaths::WritablePath(QStringLiteral("Log/OnlineServices.txt")))
			.toLocal8Bit().toStdString();
	}
}

ScanDataUploader::ScanDataUploader(QObject* parent)
	: QObject(parent)
{
	m_log = new RobotLog(OnlineServicesLogPath(), false);
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
	m_cancel.store(true);  // 先请求取消，避免析构卡在长 FTP 传输上
	JoinWorker();
	delete m_log;
	m_log = nullptr;
}

void ScanDataUploader::QueueUpload(const QString& caseDir)
{
	QString normalized;
	if (!ResolveSafeResultCaseDir(caseDir, &normalized))
	{
		emit uploadStatus(QStringLiteral("上传入队失败：案例必须是 data root 下安全的 Result/<机器人>/<案例> 目录：%1")
			.arg(QDir::toNativeSeparators(caseDir)));
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
		QString path;
		// 启动加载时清掉已被现场删除的目录，避免永远重试。
		if (ResolveSafeResultCaseDir(item.toString(), &path))
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
	m_cancel.store(false);  // 新一轮开始清取消标志，避免上一轮取消残留
	const QStringList snapshot = m_pending;
	// 配置在 UI 线程读好快照传入（ConfigDatabase 的 QSQLITE 连接不可跨线程使用）。
	UploadConfig config;
	config.host = OnlineServicesConfig::FtpHost().toStdString();
	config.port = OnlineServicesConfig::FtpPort();
	config.user = OnlineServicesConfig::FtpUser().toStdString();
	config.password = OnlineServicesConfig::FtpPassword().toStdString();
	config.deviceName = OnlineServicesConfig::DeviceName().trimmed();
	if (!AppPaths::IsSafePathComponent(config.deviceName))
	{
		m_busy.store(false);
		emit uploadStatus(QStringLiteral("上传未配置：设备名必须是安全的单一目录名。"));
		return;
	}
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
	if (!AppPaths::IsSafePathComponent(deviceName))
	{
		QMetaObject::invokeMethod(this, [this]()
			{
				m_busy.store(false);
				emit uploadStatus(QStringLiteral("上传已拒绝：设备名包含路径字符或保留设备名。"));
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
	// 预检分两步、失败原因分开报：先登录（账号/密码/网络），再建设备目录（写权限）。
	// 注意：FtpClient 构造函数不建立连接，必须先 connect() 再 createRemoteDirRecursive()。
	QString preflightError;
	if (!ftp.connect())
	{
		preflightError = QStringLiteral("FTP 登录失败：账号或密码不正确、或服务器拒绝连接（账号「%1」，服务器 %2:%3）")
			.arg(QString::fromStdString(user), QString::fromStdString(host)).arg(port);
	}
	else if (!ftp.createRemoteDirRecursive(remoteBase))
	{
		preflightError = QStringLiteral("FTP 建设备目录失败：账号「%1」可能没有写权限（目录 %2）")
			.arg(QString::fromStdString(user), QString::fromStdString(remoteBase));
	}
	bool remoteDirReady = preflightError.isEmpty();

	const int totalItems = static_cast<int>(items.size());
	int doneItems = 0;
	{
		std::lock_guard<std::mutex> lk(m_progMutex);
		m_prog = ProgressSnapshot{};
		m_prog.totalItems = totalItems;
	}

	for (const QString& caseDir : items)
	{
		if (m_cancel.load())
		{
			break;  // 已取消：不再处理剩余案例（当前未传完的留队列下次重传）
		}
		if (!remoteDirReady)
		{
			QMetaObject::invokeMethod(this, [this, caseDir, preflightError]()
				{ OnItemFinished(caseDir, false, preflightError); }, Qt::QueuedConnection);
			continue;
		}

		// zip 名：<机器人>_<案例>.zip（Result/<Robot>/<case> 取两级）
		QString safeCaseDir;
		QString robotName;
		QString caseName;
		if (!ResolveSafeResultCaseDir(caseDir, &safeCaseDir, &robotName, &caseName))
		{
			QMetaObject::invokeMethod(this, [this, caseDir]()
				{ OnItemFinished(caseDir, false, QStringLiteral("案例目录已失效或逃出 Result 根目录")); },
				Qt::QueuedConnection);
			continue;
		}
		const QString zipName = QString("%1_%2.zip").arg(robotName, caseName);
		const QString uploadTempDir = UploadTempDir();
		const QString zipPath = AppPaths::WritableChildPath(QStringLiteral("Temp/OnlineUpload"), zipName);
		if (!AppPaths::IsSafePathComponent(zipName)
			|| zipPath.isEmpty()
			|| !QDir().mkpath(uploadTempDir))
		{
			QMetaObject::invokeMethod(this, [this, caseDir]()
				{ OnItemFinished(caseDir, false, QStringLiteral("上传临时文件路径不安全")); },
				Qt::QueuedConnection);
			continue;
		}

		{
			std::lock_guard<std::mutex> lk(m_progMutex);
			m_prog.currentName = zipName;
			m_prog.doneItems = doneItems;
			m_prog.sentBytes = 0;
			m_prog.totalBytes = 0;
			m_prog.bytesPerSec = 0.0;
			m_prog.etaSeconds = 0;
		}

		QString zipError;
		if (!ZipCaseDir(safeCaseDir, zipPath, &zipError))
		{
			QMetaObject::invokeMethod(this, [this, caseDir, zipError]()
				{ OnItemFinished(caseDir, false, QStringLiteral("压缩失败：%1").arg(zipError)); }, Qt::QueuedConnection);
			continue;
		}

		// 分块上传 + 进度回调（节流 200ms 发信号 + 更新快照 + 估算速度/ETA）+ 可取消。
		auto lastEmit = std::chrono::steady_clock::now();
		long long lastSent = 0;
		double speed = 0.0;
		const std::string remotePath = remoteBase + "/" + zipName.toStdString();
		const QByteArray zipPathBytes = QDir::toNativeSeparators(zipPath).toLocal8Bit();
		const bool uploaded = ftp.uploadFileWithProgress(zipPathBytes.toStdString(), remotePath, &m_cancel,
			[this, &lastEmit, &lastSent, &speed, zipName, doneItems, totalItems](long long sent, long long total)
			{
				const auto now = std::chrono::steady_clock::now();
				const double dt = std::chrono::duration<double>(now - lastEmit).count();
				if (dt < 0.2 && sent != total)
				{
					return;  // 节流：非完成时 200ms 内不重复上报
				}
				if (dt > 0.0)
				{
					speed = static_cast<double>(sent - lastSent) / dt;
				}
				lastEmit = now;
				lastSent = sent;
				const int eta = (speed > 1.0 && total > sent)
					? static_cast<int>((total - sent) / speed) : 0;
				{
					std::lock_guard<std::mutex> lk(m_progMutex);
					m_prog.sentBytes = sent;
					m_prog.totalBytes = total;
					m_prog.bytesPerSec = speed;
					m_prog.etaSeconds = eta;
				}
				QMetaObject::invokeMethod(this, [this, doneItems, totalItems, zipName, sent, total, speed, eta]()
					{ emit uploadProgress(doneItems, totalItems, zipName, sent, total, speed, eta); },
					Qt::QueuedConnection);
			}, true);
		QFile::remove(zipPath);

		if (m_cancel.load())
		{
			// 被取消：半截文件已由 FtpClient 删除，本案例留队列，不计完成、不再处理后续。
			QMetaObject::invokeMethod(this, [this, caseDir]()
				{ OnItemFinished(caseDir, false, QStringLiteral("已取消上传（服务器半截文件已清除）")); }, Qt::QueuedConnection);
			break;
		}

		if (uploaded)
		{
			++doneItems;
			{
				std::lock_guard<std::mutex> lk(m_progMutex);
				m_prog.doneItems = doneItems;
			}
		}

		QMetaObject::invokeMethod(this, [this, caseDir, uploaded, zipName]()
			{
				OnItemFinished(caseDir, uploaded,
					uploaded ? QStringLiteral("已上传 %1").arg(zipName)
					         : QStringLiteral("FTP 上传失败（将定时重试）"));
			}, Qt::QueuedConnection);

		if (!uploaded)
		{
			remoteDirReady = false;  // 连接大概率已断，本轮剩余项直接留待重试
			if (preflightError.isEmpty())
			{
				preflightError = QStringLiteral("FTP 连接中断，本轮剩余项留待下次重试");
			}
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

ScanDataUploader::ProgressSnapshot ScanDataUploader::CurrentProgress() const
{
	std::lock_guard<std::mutex> lk(m_progMutex);
	return m_prog;
}

void ScanDataUploader::RequestCancel()
{
	m_cancel.store(true);  // 后台线程与 FtpClient 块间读到后中止并删服务器半截文件
}

void ScanDataUploader::CancelAndWait()
{
	m_cancel.store(true);
	JoinWorker();          // 阻塞等后台线程收尾（含半截文件删除）后返回
	m_busy.store(false);
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
