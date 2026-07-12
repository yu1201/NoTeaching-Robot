#include "ScanDataUploader.h"

#include "AppPaths.h"
#include "FtpClient.h"
#include "OnlineServicesConfig.h"
#include "RobotLog.h"

#include <QDateTime>
#include <QCoreApplication>
#include <QCryptographicHash>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QEvent>
#include <QHash>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonParseError>
#include <QMetaObject>
#include <QProcess>
#include <QSet>
#include <QStorageInfo>
#include <QTcpSocket>
#include <QTimer>
#include <QUuid>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <limits>
#include <vector>

#ifdef Q_OS_WIN
#include <windows.h>
#endif

#include <QtCore/private/qzipreader_p.h>

namespace
{
	constexpr int kRetryIntervalMs = 5 * 60 * 1000;  // 失败重试间隔：5 分钟
	constexpr int kMaximumRemoteComponentUtf8Bytes = 240;
	constexpr int kMaximumPendingItems = 512;
	constexpr int kMaximumItemsPerWorkerRun = 8;
	constexpr int kMaximumPendingJsonBytes = 1024 * 1024;
	constexpr int kMaximumArchiveFileCount = 4096;
	constexpr int kMaximumTempEntriesScanned = 2048;
	constexpr int kMaximumStaleArchivesDeletedPerPass = 128;
	// ZIP32 偏移不能贴近 4 GiB 边界。压缩由可强制终止的系统
	// bsdtar 子进程执行，避免 QZipWriter::addFile(QIODevice*) 内部 readAll()
	// 导致单个大文件压缩期间无法响应取消。
	constexpr qint64 kMaximumSingleFileBytes = 768LL * 1024 * 1024;
	constexpr qint64 kMaximumUncompressedBytes = 2LL * 1024 * 1024 * 1024;
	constexpr qint64 kMaximumZipBytes = 2LL * 1024 * 1024 * 1024;
	constexpr qint64 kHashReadChunkBytes = 1024 * 1024;
	constexpr qint64 kMinimumFreeDiskReserveBytes = 2LL * 1024 * 1024 * 1024;
	constexpr int kArchiveProcessPollMs = 50;

	struct ArchiveFileSnapshot
	{
		QString absolutePath;
		QString archivePath;
		qint64 size = 0;
		qint64 modifiedMs = 0;
	};

	void SetArchiveError(QString* error, const QString& message)
	{
		if (error != nullptr)
		{
			*error = message;
		}
	}

	bool ArchiveCancelRequested(const std::atomic<bool>* cancel)
	{
		return cancel != nullptr && cancel->load(std::memory_order_relaxed);
	}

	QString FixedSystemTarProgram()
	{
#ifdef Q_OS_WIN
		std::vector<wchar_t> windowsDir(32768, L'\0');
		const UINT length = GetSystemWindowsDirectoryW(
			windowsDir.data(), static_cast<UINT>(windowsDir.size()));
		if (length == 0 || length >= windowsDir.size())
		{
			return QString();
		}
		const QString candidate = QDir(QString::fromWCharArray(windowsDir.data(), int(length)))
			.filePath(QStringLiteral("System32/tar.exe"));
		const std::wstring native = QDir::toNativeSeparators(candidate).toStdWString();
		const DWORD attributes = GetFileAttributesW(native.c_str());
		if (attributes == INVALID_FILE_ATTRIBUTES
			|| (attributes & FILE_ATTRIBUTE_DIRECTORY) != 0
			|| (attributes & FILE_ATTRIBUTE_REPARSE_POINT) != 0)
		{
			return QString();
		}
		const QFileInfo info(candidate);
		if (!info.isFile() || info.isSymLink() || info.isJunction())
		{
			return QString();
		}
		return info.absoluteFilePath();
#else
		return QString();
#endif
	}

	std::filesystem::path ToFileSystemPath(const QString& path)
	{
#ifdef Q_OS_WIN
		return std::filesystem::path(QDir::toNativeSeparators(path).toStdWString());
#else
		return std::filesystem::path(path.toStdString());
#endif
	}

	QString FromFileSystemPath(const std::filesystem::path& path)
	{
#ifdef Q_OS_WIN
		return QString::fromStdWString(path.wstring());
#else
		return QString::fromStdString(path.string());
#endif
	}

	bool CollectArchiveFiles(
		const QString& caseDir,
		std::vector<ArchiveFileSnapshot>* files,
		qint64* totalBytes,
		QString* error,
		const std::atomic<bool>* cancel)
	{
		if (files == nullptr || totalBytes == nullptr)
		{
			SetArchiveError(error, QStringLiteral("压缩内部参数无效"));
			return false;
		}
		files->clear();
		*totalBytes = 0;

		const QFileInfo caseInfo(caseDir);
		const QString canonicalCase = caseInfo.canonicalFilePath();
		const QString canonicalArchiveRoot = QDir(
			QDir::cleanPath(caseInfo.dir().absolutePath() + QStringLiteral("/.."))).canonicalPath();
		if (canonicalCase.isEmpty() || canonicalArchiveRoot.isEmpty() || !caseInfo.isDir())
		{
			SetArchiveError(error, QStringLiteral("案例目录不存在或无法解析"));
			return false;
		}
		const QString caseRelative = QDir(canonicalArchiveRoot)
			.relativeFilePath(canonicalCase).replace(QLatin1Char('\\'), QLatin1Char('/'));
		if (caseRelative == QStringLiteral("..")
			|| caseRelative.startsWith(QStringLiteral("../"))
			|| QDir::isAbsolutePath(caseRelative))
		{
			SetArchiveError(error, QStringLiteral("案例目录逃出归档根目录"));
			return false;
		}

		std::error_code ec;
		const std::filesystem::path fsRoot = ToFileSystemPath(canonicalCase);
		std::filesystem::recursive_directory_iterator it(
			fsRoot, std::filesystem::directory_options::none, ec);
		const std::filesystem::recursive_directory_iterator end;
		if (ec)
		{
			SetArchiveError(error, QStringLiteral("无法遍历案例目录：%1")
				.arg(QString::fromLocal8Bit(ec.message().c_str())));
			return false;
		}

		while (it != end)
		{
			if (ArchiveCancelRequested(cancel))
			{
				SetArchiveError(error, QStringLiteral("已取消压缩（目录清单阶段）"));
				return false;
			}
			const std::filesystem::directory_entry entry = *it;
			const std::filesystem::file_status status = entry.symlink_status(ec);
			if (ec)
			{
				SetArchiveError(error, QStringLiteral("无法读取目录项状态：%1")
					.arg(QString::fromLocal8Bit(ec.message().c_str())));
				return false;
			}
			const QString entryPath = FromFileSystemPath(entry.path());
			const QFileInfo entryInfo(entryPath);
			bool isLinkLike = std::filesystem::is_symlink(status) || entryInfo.isSymLink();
#ifdef Q_OS_WIN
			isLinkLike = isLinkLike || entryInfo.isJunction();
#endif
			if (isLinkLike)
			{
				SetArchiveError(error, QStringLiteral("案例目录包含不允许的链接/联接点：%1")
					.arg(QDir::toNativeSeparators(entryPath)));
				return false;
			}
			if (std::filesystem::is_directory(status))
			{
				it.increment(ec);
				if (ec)
				{
					SetArchiveError(error, QStringLiteral("目录遍历中断：%1")
						.arg(QString::fromLocal8Bit(ec.message().c_str())));
					return false;
				}
				continue;
			}
			if (!std::filesystem::is_regular_file(status))
			{
				SetArchiveError(error, QStringLiteral("案例目录包含非常规文件：%1")
					.arg(QDir::toNativeSeparators(entryPath)));
				return false;
			}

			const std::uintmax_t rawSize = entry.file_size(ec);
			if (ec || rawSize > static_cast<std::uintmax_t>(std::numeric_limits<qint64>::max()))
			{
				SetArchiveError(error, QStringLiteral("无法获取文件大小：%1")
					.arg(QDir::toNativeSeparators(entryPath)));
				return false;
			}
			const qint64 fileBytes = static_cast<qint64>(rawSize);
			if (fileBytes > kMaximumSingleFileBytes)
			{
				SetArchiveError(error, QStringLiteral("单文件超过 768 MiB 上限：%1")
					.arg(QDir::toNativeSeparators(entryPath)));
				return false;
			}
			if (fileBytes > kMaximumUncompressedBytes - *totalBytes)
			{
				SetArchiveError(error, QStringLiteral("案例未压缩总字节超过 2 GiB 上限"));
				return false;
			}
			if (files->size() >= static_cast<size_t>(kMaximumArchiveFileCount))
			{
				SetArchiveError(error, QStringLiteral("案例文件数超过 %1 上限")
					.arg(kMaximumArchiveFileCount));
				return false;
			}

			const QString canonicalFile = entryInfo.canonicalFilePath();
			const QString relative = QDir(canonicalArchiveRoot)
				.relativeFilePath(canonicalFile).replace(QLatin1Char('\\'), QLatin1Char('/'));
			if (canonicalFile.isEmpty()
				|| relative == QStringLiteral("..")
				|| relative.startsWith(QStringLiteral("../"))
				|| QDir::isAbsolutePath(relative))
			{
				SetArchiveError(error, QStringLiteral("文件路径无法安全加入归档：%1")
					.arg(QDir::toNativeSeparators(entryPath)));
				return false;
			}
			files->push_back({ canonicalFile, relative, fileBytes,
				entryInfo.lastModified().toMSecsSinceEpoch() });
			*totalBytes += fileBytes;

			it.increment(ec);
			if (ec)
			{
				SetArchiveError(error, QStringLiteral("目录遍历中断：%1")
					.arg(QString::fromLocal8Bit(ec.message().c_str())));
				return false;
			}
		}

		std::sort(files->begin(), files->end(), [](const ArchiveFileSnapshot& left, const ArchiveFileSnapshot& right)
			{
				return left.archivePath.compare(right.archivePath, Qt::CaseSensitive) < 0;
			});
		if (files->empty())
		{
			SetArchiveError(error, QStringLiteral("案例目录中没有可上传文件"));
			return false;
		}
		return true;
	}

	bool HashFileExact(
		const ArchiveFileSnapshot& snapshot,
		QByteArray* digest,
		QString* error,
		const std::atomic<bool>* cancel)
	{
		QFile file(snapshot.absolutePath);
		if (!file.open(QIODevice::ReadOnly))
		{
			SetArchiveError(error, QStringLiteral("无法读取文件：%1（%2）")
				.arg(QDir::toNativeSeparators(snapshot.absolutePath), file.errorString()));
			return false;
		}
		QCryptographicHash hash(QCryptographicHash::Sha256);
		QByteArray buffer(static_cast<qsizetype>(kHashReadChunkBytes), Qt::Uninitialized);
		qint64 readBytes = 0;
		while (true)
		{
			if (ArchiveCancelRequested(cancel))
			{
				SetArchiveError(error, QStringLiteral("已取消压缩（文件校验阶段）"));
				return false;
			}
			const qint64 count = file.read(buffer.data(), buffer.size());
			if (count < 0)
			{
				SetArchiveError(error, QStringLiteral("读取文件失败：%1（%2）")
					.arg(QDir::toNativeSeparators(snapshot.absolutePath), file.errorString()));
				return false;
			}
			if (count == 0)
			{
				break;
			}
			readBytes += count;
			if (readBytes > snapshot.size)
			{
				SetArchiveError(error, QStringLiteral("压缩期间文件变大：%1")
					.arg(QDir::toNativeSeparators(snapshot.absolutePath)));
				return false;
			}
			hash.addData(QByteArrayView(buffer.constData(), static_cast<qsizetype>(count)));
		}
		if (file.error() != QFileDevice::NoError || readBytes != snapshot.size)
		{
			SetArchiveError(error, QStringLiteral("文件未完整读取或压缩期间变化：%1")
				.arg(QDir::toNativeSeparators(snapshot.absolutePath)));
			return false;
		}
		if (digest != nullptr)
		{
			*digest = hash.result();
		}
		return true;
	}

	bool SameArchiveSnapshot(
		const std::vector<ArchiveFileSnapshot>& before,
		const std::vector<ArchiveFileSnapshot>& after)
	{
		if (before.size() != after.size())
		{
			return false;
		}
		for (size_t index = 0; index < before.size(); ++index)
		{
			if (before[index].archivePath != after[index].archivePath
				|| before[index].size != after[index].size
				|| before[index].modifiedMs != after[index].modifiedMs)
			{
				return false;
			}
		}
		return true;
	}

	qint64 EstimatedWorstCaseZipWriteBytes(const std::vector<ArchiveFileSnapshot>& files)
	{
		// zlib compressBound 的保守公式 + ZIP 本地/中心目录头与两份 UTF-8 文件名。
		qint64 estimated = 22;
		for (const ArchiveFileSnapshot& file : files)
		{
			const qint64 compressedBound = file.size
				+ (file.size >> 12)
				+ (file.size >> 14)
				+ (file.size >> 25)
				+ 13;
			const qint64 nameBytes = file.archivePath.toUtf8().size();
			estimated += compressedBound + 76 + 2 * nameBytes;
		}
		return estimated;
	}

	bool HasArchiveStorageHeadroom(
		const QString& zipPath,
		const std::vector<ArchiveFileSnapshot>& files,
		QString* error)
	{
		QStorageInfo storage(QFileInfo(zipPath).absolutePath());
		storage.refresh();
		if (!storage.isValid() || !storage.isReady() || storage.bytesAvailable() < 0)
		{
			SetArchiveError(error, QStringLiteral("无法确认上传临时盘可用空间，已拒绝压缩"));
			return false;
		}
		const qint64 estimatedWriteBytes = EstimatedWorstCaseZipWriteBytes(files);
		const qint64 requiredBytes = estimatedWriteBytes + kMinimumFreeDiskReserveBytes;
		if (storage.bytesAvailable() < requiredBytes)
		{
			SetArchiveError(error,
				QStringLiteral("上传临时盘空间不足：可用 %1 字节，本次最坏写入 %2 字节，必须额外保留 2 GiB")
					.arg(storage.bytesAvailable()).arg(estimatedWriteBytes));
			return false;
		}
		return true;
	}

	QString TruncateToUtf8Bytes(const QString& value, int maximumBytes)
	{
		QString result;
		result.reserve(value.size());
		int usedBytes = 0;
		for (int index = 0; index < value.size();)
		{
			int unitLength = 1;
			if (value.at(index).isHighSurrogate()
				&& index + 1 < value.size()
				&& value.at(index + 1).isLowSurrogate())
			{
				unitLength = 2;
			}
			const QString unit = value.mid(index, unitLength);
			const int unitBytes = unit.toUtf8().size();
			if (usedBytes + unitBytes > maximumBytes)
			{
				break;
			}
			result += unit;
			usedBytes += unitBytes;
			index += unitLength;
		}
		return result;
	}

	QString UploadTempDir()
	{
		return AppPaths::WritablePath(QStringLiteral("Temp/OnlineUpload"));
	}

	bool CleanupStaleUploadArchives(QString* error, int* deletedCount = nullptr)
	{
		if (deletedCount != nullptr)
		{
			*deletedCount = 0;
		}
		const QString tempDir = UploadTempDir();
		const QFileInfo rootInfo(tempDir);
		if (!rootInfo.exists())
		{
			return true;
		}
		bool rootIsLinkLike = rootInfo.isSymLink();
#ifdef Q_OS_WIN
		rootIsLinkLike = rootIsLinkLike || rootInfo.isJunction();
#endif
		const QString canonicalRoot = QDir(tempDir).canonicalPath();
		if (!rootInfo.isDir() || rootIsLinkLike || canonicalRoot.isEmpty())
		{
			SetArchiveError(error, QStringLiteral("上传临时目录不是安全的普通目录，已拒绝清理和上传"));
			return false;
		}

		std::error_code ec;
		std::filesystem::directory_iterator it(
			ToFileSystemPath(canonicalRoot), std::filesystem::directory_options::none, ec);
		const std::filesystem::directory_iterator end;
		if (ec)
		{
			SetArchiveError(error, QStringLiteral("无法扫描上传临时目录：%1")
				.arg(QString::fromLocal8Bit(ec.message().c_str())));
			return false;
		}

		int scanned = 0;
		int deleted = 0;
		const qint64 nowUtcMs = QDateTime::currentDateTimeUtc().toMSecsSinceEpoch();
		while (it != end)
		{
			if (++scanned > kMaximumTempEntriesScanned)
			{
				SetArchiveError(error, QStringLiteral("上传临时目录超过 %1 个目录项扫描上限，已失败关闭")
					.arg(kMaximumTempEntriesScanned));
				return false;
			}
			const QString entryPath = FromFileSystemPath(it->path());
			const QFileInfo info(entryPath);
			const QString name = info.fileName();
			if (ScanDataUploadPolicy::IsOwnedTempArchiveName(name)
				|| ScanDataUploadPolicy::IsOwnedTempArchiveListName(name))
			{
				bool isLinkLike = info.isSymLink();
#ifdef Q_OS_WIN
				isLinkLike = isLinkLike || info.isJunction();
#endif
				if (!info.isFile() || isLinkLike)
				{
					SetArchiveError(error, QStringLiteral("发现占用本工具临时命名的非普通文件，已拒绝删除：%1")
						.arg(QDir::toNativeSeparators(entryPath)));
					return false;
				}
				const QString canonicalEntry = info.canonicalFilePath();
				const QString relative = QDir(canonicalRoot).relativeFilePath(canonicalEntry)
					.replace(QLatin1Char('\\'), QLatin1Char('/'));
				if (canonicalEntry.isEmpty()
					|| relative.compare(name, Qt::CaseSensitive) != 0
					|| relative.contains(QLatin1Char('/')))
				{
					SetArchiveError(error, QStringLiteral("上传临时文件解析后逃出目录边界，已拒绝删除"));
					return false;
				}
				const qint64 modifiedUtcMs = info.lastModified().toUTC().toMSecsSinceEpoch();
				if (ScanDataUploadPolicy::ShouldDeleteTempArchive(
						name, true, false, modifiedUtcMs, nowUtcMs)
					|| ScanDataUploadPolicy::ShouldDeleteTempArchiveList(
						name, true, false, modifiedUtcMs, nowUtcMs))
				{
					if (deleted >= kMaximumStaleArchivesDeletedPerPass)
					{
						SetArchiveError(error, QStringLiteral("本轮已清理 %1 个过期临时 ZIP，仍有积压；已停止本轮上传")
							.arg(kMaximumStaleArchivesDeletedPerPass));
						if (deletedCount != nullptr)
						{
							*deletedCount = deleted;
						}
						return false;
					}
					if (!QFile::remove(entryPath))
					{
						SetArchiveError(error, QStringLiteral("无法删除超过 24 小时的上传临时 ZIP：%1")
							.arg(QDir::toNativeSeparators(entryPath)));
						return false;
					}
					++deleted;
				}
			}

			it.increment(ec);
			if (ec)
			{
				SetArchiveError(error, QStringLiteral("上传临时目录扫描中断：%1")
					.arg(QString::fromLocal8Bit(ec.message().c_str())));
				return false;
			}
		}
		if (deletedCount != nullptr)
		{
			*deletedCount = deleted;
		}
		return true;
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
	int startupDeleted = 0;
	if (!CleanupStaleUploadArchives(&m_startupCleanupError, &startupDeleted))
	{
		m_log->writeLine(m_startupCleanupError.toUtf8().toStdString());
	}
	else if (startupDeleted > 0)
	{
		m_log->writeLine(QStringLiteral("启动清理 %1 个超过 24 小时的上传临时 ZIP。")
			.arg(startupDeleted).toUtf8().toStdString());
	}
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
	if (!m_pendingStoreError.isEmpty())
	{
		QTimer::singleShot(0, this, [this]() { emit uploadStatus(m_pendingStoreError); });
	}
	if (!m_startupCleanupError.isEmpty())
	{
		QTimer::singleShot(0, this, [this]() { emit uploadStatus(m_startupCleanupError); });
	}
}

ScanDataUploader::~ScanDataUploader()
{
	m_cancel.store(true);  // 先请求取消，避免析构卡在长 FTP 传输上
	JoinWorker();
	// worker 已停止，不会再投递；仅冲刷发给本对象的 queued MetaCall，确保成功项先持久化出队。
	QCoreApplication::sendPostedEvents(this, QEvent::MetaCall);
	delete m_log;
	m_log = nullptr;
}

void ScanDataUploader::QueueUpload(const QString& caseDir)
{
	if (m_pendingStoreBlocked)
	{
		emit uploadStatus(m_pendingStoreError);
		return;
	}
	QString normalized;
	if (!ResolveSafeResultCaseDir(caseDir, &normalized))
	{
		emit uploadStatus(QStringLiteral("上传入队失败：案例必须是 data root 下安全的 Result/<机器人>/<案例> 目录：%1")
			.arg(QDir::toNativeSeparators(caseDir)));
		return;
	}
	if (!m_pending.contains(normalized))
	{
		if (m_pending.size() >= kMaximumPendingItems)
		{
			emit uploadStatus(QStringLiteral("上传入队失败：待传队列已达 %1 条硬上限，请先处理现有队列。")
				.arg(kMaximumPendingItems));
			return;
		}
		m_pending.append(normalized);
		SavePending();
		emit pendingChanged(m_pending.size());
	}
	StartWorkerIfIdle();
}

void ScanDataUploader::TriggerUploadNow()
{
	if (m_pendingStoreBlocked)
	{
		emit uploadStatus(m_pendingStoreError);
		return;
	}
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
	m_pendingStoreBlocked = false;
	m_pendingStoreError.clear();
	const QByteArray pendingJson = OnlineServicesConfig::PendingUploads().toUtf8();
	if (pendingJson.size() > kMaximumPendingJsonBytes)
	{
		m_pendingStoreBlocked = true;
		m_pendingStoreError = QStringLiteral("待传队列存储超过 1 MiB 硬上限，已暂停上传且不改写原队列；请管理员人工核查 ConfigStore。");
		return;
	}
	QJsonParseError parseError;
	const QJsonDocument doc = QJsonDocument::fromJson(pendingJson, &parseError);
	if (parseError.error != QJsonParseError::NoError || !doc.isArray())
	{
		m_pendingStoreBlocked = true;
		m_pendingStoreError = QStringLiteral("待传队列存储已损坏，已暂停上传且不改写原队列：%1")
			.arg(parseError.errorString());
		return;
	}
	if (doc.array().size() > kMaximumPendingItems)
	{
		m_pendingStoreBlocked = true;
		m_pendingStoreError = QStringLiteral("待传队列超过 %1 条硬上限，已暂停上传且不改写原队列；请管理员先处理积压。")
			.arg(kMaximumPendingItems);
		return;
	}
	QSet<QString> uniquePaths;
	for (const auto& item : doc.array())
	{
		QString path;
		// 启动加载时清掉已被现场删除的目录，避免永远重试。
		if (item.isString()
			&& ResolveSafeResultCaseDir(item.toString(), &path)
			&& !uniquePaths.contains(path))
		{
			uniquePaths.insert(path);
			m_pending.append(path);
		}
	}
}

void ScanDataUploader::SavePending()
{
	if (m_pendingStoreBlocked || m_pending.size() > kMaximumPendingItems)
	{
		return;  // 失败关闭：绝不用截断后的队列覆盖原存储
	}
	QJsonArray array;
	for (const QString& path : m_pending)
	{
		array.append(path);
	}
	OnlineServicesConfig::SetPendingUploads(QString::fromUtf8(QJsonDocument(array).toJson(QJsonDocument::Compact)));
}

void ScanDataUploader::StartWorkerIfIdle()
{
	if (m_pendingStoreBlocked || m_busy.load() || m_pending.isEmpty())
	{
		return;
	}
	QString cleanupError;
	int deletedCount = 0;
	if (!CleanupStaleUploadArchives(&cleanupError, &deletedCount))
	{
		emit uploadStatus(QStringLiteral("上传已失败关闭：%1").arg(cleanupError));
		return;
	}
	if (deletedCount > 0)
	{
		emit uploadStatus(QStringLiteral("已清理 %1 个超过 24 小时的上传临时 ZIP。")
			.arg(deletedCount));
	}
	JoinWorker();
	m_busy.store(true);
	m_cancel.store(false);  // 新一轮开始清取消标志，避免上一轮取消残留
	const QStringList snapshot = m_pending.mid(0, kMaximumItemsPerWorkerRun);
	// 配置在 UI 线程读好快照传入（ConfigDatabase 的 QSQLITE 连接不可跨线程使用）。
	UploadConfig config;
	config.host = OnlineServicesConfig::FtpHost().toStdString();
	config.port = OnlineServicesConfig::FtpPort();
	config.user = OnlineServicesConfig::FtpUser().toStdString();
	config.password = OnlineServicesConfig::FtpPassword().toStdString();
	config.deviceName = OnlineServicesConfig::DeviceName().trimmed();
	if (!OnlineServicesConfig::IsServerAccountName(config.deviceName))
	{
		m_busy.store(false);
		emit uploadStatus(QStringLiteral(
			"上传未配置：设备名必须匹配 ^[a-z][a-z0-9_-]{2,31}$。"));
		return;
	}
	QString identityError;
	if (!OnlineServicesConfig::HasDeviceBoundUploadIdentity(&identityError))
	{
		m_busy.store(false);
		emit uploadStatus(QStringLiteral("上传已失败关闭：%1。请在服务端创建用户名与设备名完全一致的专用 FTP 账号，并将其 ACL 限制为仅写 /data/%2；客户端无法代替该服务端隔离。")
			.arg(identityError, config.deviceName));
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
	if (password.empty())
	{
		QMetaObject::invokeMethod(this, [this]()
			{
				m_busy.store(false);
				emit uploadStatus(QStringLiteral("上传未配置：请在管理页「在线服务」填写 FTP 密码。"));
			}, Qt::QueuedConnection);
		return;
	}
	const QString snapshotUser = QString::fromStdString(user).trimmed();
	QString snapshotIdentityError;
	if (!OnlineServicesConfig::IsDeviceBoundUploadIdentity(
		snapshotUser, deviceName, &snapshotIdentityError))
	{
		QMetaObject::invokeMethod(this, [this, snapshotIdentityError]()
			{
				m_busy.store(false);
				emit uploadStatus(QStringLiteral("上传已拒绝：%1。").arg(snapshotIdentityError));
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
		// 本地临时名只用随机尝试 ID，避免同一 data root 中的多个上传器/进程互相覆盖。
		const QString localZipName = QStringLiteral("upload_%1.zip")
			.arg(QUuid::createUuid().toString(QUuid::WithoutBraces).remove(QLatin1Char('-')));
		const QString zipPath = AppPaths::WritableChildPath(
			QStringLiteral("Temp/OnlineUpload"), localZipName);
		if (!AppPaths::IsSafePathComponent(zipName)
			|| !AppPaths::IsSafePathComponent(localZipName)
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
		if (!ZipCaseDir(safeCaseDir, zipPath, &zipError, &m_cancel))
		{
			QMetaObject::invokeMethod(this, [this, caseDir, zipError]()
				{ OnItemFinished(caseDir, false, QStringLiteral("压缩失败：%1").arg(zipError)); }, Qt::QueuedConnection);
			continue;
		}

		// 分块上传 + 进度回调（节流 200ms 发信号 + 更新快照 + 估算速度/ETA）+ 可取消。
		auto lastEmit = std::chrono::steady_clock::now();
		long long lastSent = 0;
		double speed = 0.0;
		// 设备专用账号每次尝试都使用唯一远端名，
		// 避免 STOR 重试覆盖任何既有文件；失败残件由服务器按策略清理。
		const QString attemptId = QDateTime::currentDateTimeUtc().toString(QStringLiteral("yyyyMMddTHHmmsszzz"))
			+ QLatin1Char('_')
			+ QUuid::createUuid().toString(QUuid::WithoutBraces).remove(QLatin1Char('-')).left(12);
		const qint64 archiveBytes = QFileInfo(zipPath).size();
		const QString archiveStem = QFileInfo(zipName).completeBaseName();
		const QString remoteSuffix = QLatin1Char('_') + attemptId + QLatin1Char('_')
			+ QString::number(archiveBytes) + QStringLiteral(".zip");
		const int maximumStemBytes = kMaximumRemoteComponentUtf8Bytes - remoteSuffix.toUtf8().size();
		const QString remoteStem = TruncateToUtf8Bytes(archiveStem, maximumStemBytes);
		const QString remoteZipName = remoteStem + remoteSuffix;
		if (archiveBytes <= 0
			|| remoteStem.isEmpty()
			|| remoteZipName.toUtf8().size() > kMaximumRemoteComponentUtf8Bytes
			|| !AppPaths::IsSafePathComponent(remoteZipName))
		{
			QFile::remove(zipPath);
			QMetaObject::invokeMethod(this, [this, caseDir]()
				{ OnItemFinished(caseDir, false, QStringLiteral("上传远端文件名不安全")); }, Qt::QueuedConnection);
			continue;
		}
		const std::string remotePath = remoteBase + "/" + remoteZipName.toStdString();
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
			}, false);
		QFile::remove(zipPath);

		if (uploaded)
		{
			++doneItems;
			{
				std::lock_guard<std::mutex> lk(m_progMutex);
				m_prog.doneItems = doneItems;
			}
		}

		if (m_cancel.load())
		{
			// 最后一块完成后才收到取消时，本案例已可靠落盘并应出队；只停止后续项。
			QMetaObject::invokeMethod(this, [this, caseDir, uploaded, zipName]()
				{
					OnItemFinished(caseDir, uploaded,
						uploaded ? QStringLiteral("已上传 %1；已停止后续上传").arg(zipName)
							     : QStringLiteral("已取消上传；不完整唯一残件可能暂存至服务器定期清理"));
				}, Qt::QueuedConnection);
			break;
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
	m_cancel.store(true);  // 后台线程与 FtpClient 块间读到后中止；唯一残件不展示并定期清理
}

void ScanDataUploader::CancelAndWait()
{
	m_cancel.store(true);
	JoinWorker();          // 阻塞等后台线程收尾后返回
	// 不能依赖退出路径之后的事件循环：只处理本对象的 worker 回调，避免已上传案例下次重复上传。
	QCoreApplication::sendPostedEvents(this, QEvent::MetaCall);
	m_busy.store(false);
}

bool ScanDataUploader::ZipCaseDir(
	const QString& caseDir,
	const QString& zipPath,
	QString* error,
	const std::atomic<bool>* cancel)
{
	// 先完整遍历并固化清单：遍历/查状态任何失败都不能静默漏文件后仍上传。
	std::vector<ArchiveFileSnapshot> beforeFiles;
	qint64 beforeTotalBytes = 0;
	if (!CollectArchiveFiles(caseDir, &beforeFiles, &beforeTotalBytes, error, cancel))
	{
		return false;
	}
	if (ArchiveCancelRequested(cancel))
	{
		SetArchiveError(error, QStringLiteral("已取消压缩（存储预检前）"));
		return false;
	}
	if (!HasArchiveStorageHeadroom(zipPath, beforeFiles, error))
	{
		return false;  // 在独占创建 ZIP 之前拒绝，不消耗临时盘、不动队列
	}

	// 先对每个源文件分块计算摘要，每 1 MiB 检查一次取消。
	std::vector<QByteArray> sourceDigests;
	sourceDigests.reserve(beforeFiles.size());
	for (const ArchiveFileSnapshot& snapshot : beforeFiles)
	{
		QByteArray digest;
		QString readError;
		if (!HashFileExact(snapshot, &digest, &readError, cancel))
		{
			SetArchiveError(error, readError);
			return false;
		}
		sourceDigests.push_back(digest);
	}

	// Windows 自带 bsdtar 可从 NUL 分隔清单以 UTF-8 相对路径生成 ZIP。
	// 它在独立进程里压缩整个单文件；取消时 kill() 不必等待
	// QZipWriter::readAll()/deflate 返回，因而最大终止检查间隔为 50 ms。
	const QString tarProgram = FixedSystemTarProgram();
	if (tarProgram.isEmpty() || !QFileInfo(tarProgram).isFile())
	{
		SetArchiveError(error, QStringLiteral("系统可终止 ZIP 压缩器不可用：%1")
			.arg(QDir::toNativeSeparators(tarProgram)));
		return false;
	}

	const QFileInfo caseInfo(caseDir);
	const QString archiveRoot = QDir(
		QDir::cleanPath(caseInfo.dir().absolutePath() + QStringLiteral("/.."))).canonicalPath();
	if (archiveRoot.isEmpty())
		{
		SetArchiveError(error, QStringLiteral("无法解析归档根目录"));
		return false;
	}

	const QString listPath = zipPath + QStringLiteral(".files");
	QFile listFile(listPath);
	if (!listFile.open(QIODevice::WriteOnly | QIODevice::NewOnly))
	{
		SetArchiveError(error, QStringLiteral("无法独占创建压缩清单：%1").arg(listFile.errorString()));
		return false;
	}
	for (const ArchiveFileSnapshot& snapshot : beforeFiles)
	{
		if (ArchiveCancelRequested(cancel))
		{
			listFile.close();
			QFile::remove(listPath);
			SetArchiveError(error, QStringLiteral("已取消压缩（生成文件清单阶段）"));
			return false;
		}
		const QByteArray encodedPath = snapshot.archivePath.toUtf8();
		if (encodedPath.isEmpty()
			|| listFile.write(encodedPath) != encodedPath.size()
			|| !listFile.putChar('\0'))
		{
			const QString writeError = listFile.errorString();
			listFile.close();
			QFile::remove(listPath);
			SetArchiveError(error, QStringLiteral("写入压缩清单失败：%1").arg(writeError));
			return false;
		}
	}
	listFile.close();
	if (listFile.error() != QFileDevice::NoError)
	{
		const QString writeError = listFile.errorString();
		QFile::remove(listPath);
		SetArchiveError(error, QStringLiteral("关闭压缩清单失败：%1").arg(writeError));
		return false;
	}

	QProcess archiveProcess;
	archiveProcess.setProgram(tarProgram);
	archiveProcess.setArguments(QStringList()
		<< QStringLiteral("-c") << QStringLiteral("--format") << QStringLiteral("zip")
		<< QStringLiteral("--options") << QStringLiteral("hdrcharset=UTF-8")
		<< QStringLiteral("-f") << QDir::toNativeSeparators(zipPath)
		<< QStringLiteral("-C") << QDir::toNativeSeparators(archiveRoot)
		<< QStringLiteral("--null") << QStringLiteral("--no-recursion")
		<< QStringLiteral("-T") << QDir::toNativeSeparators(listPath));
	archiveProcess.setProcessChannelMode(QProcess::SeparateChannels);
	QByteArray boundedArchiveError;
	boundedArchiveError.reserve(4096);
	const auto drainProcessOutput = [&archiveProcess, &boundedArchiveError]()
	{
		archiveProcess.readAllStandardOutput();
		const QByteArray chunk = archiveProcess.readAllStandardError();
		const qsizetype remaining = 4096 - boundedArchiveError.size();
		if (remaining > 0)
		{
			boundedArchiveError += chunk.left(remaining);
		}
	};
	archiveProcess.start();
	if (!archiveProcess.waitForStarted(5000))
	{
		QFile::remove(listPath);
		QFile::remove(zipPath);
		SetArchiveError(error, QStringLiteral("无法启动系统 ZIP 压缩器：%1")
			.arg(archiveProcess.errorString()));
		return false;
	}
	while (archiveProcess.state() != QProcess::NotRunning)
	{
		if (ArchiveCancelRequested(cancel))
		{
			archiveProcess.kill();
			if (!archiveProcess.waitForFinished(5000))
			{
				// Windows TerminateProcess 后应立即返回；仍以无限等待确认
				// 进程真正消失，绝不在子进程可能继续写时删临时包。
				archiveProcess.kill();
				archiveProcess.waitForFinished(-1);
			}
			QFile::remove(listPath);
			QFile::remove(zipPath);
			SetArchiveError(error, QStringLiteral("已取消压缩（压缩子进程已终止）"));
			return false;
		}
		archiveProcess.waitForFinished(kArchiveProcessPollMs);
		// 两个管道都持续 drain，避免 4096 文件错误逐条输出塞满
		// stderr 反压死锁。诊断只保留前 4096 字节，其余丢弃。
		drainProcessOutput();
	}
	drainProcessOutput();
	const QByteArray archiveError = boundedArchiveError;
	QFile::remove(listPath);
	if (archiveProcess.exitStatus() != QProcess::NormalExit || archiveProcess.exitCode() != 0)
	{
		QFile::remove(zipPath);
		SetArchiveError(error, QStringLiteral("系统 ZIP 压缩失败（退出码 %1）：%2")
			.arg(archiveProcess.exitCode())
			.arg(QString::fromLocal8Bit(archiveError).trimmed()));
		return false;
	}

	// 不信任 tar 的参数/编码转换结果：回读 ZIP central directory，
	// 与预冻结的相对路径、项目数和未压缩大小逐项严格相等。
	QHash<QString, qint64> expectedEntries;
	expectedEntries.reserve(static_cast<qsizetype>(beforeFiles.size()));
	for (const ArchiveFileSnapshot& snapshot : beforeFiles)
	{
		expectedEntries.insert(snapshot.archivePath, snapshot.size);
	}
	QZipReader archiveReader(zipPath);
	const QList<QZipReader::FileInfo> actualEntries = archiveReader.fileInfoList();
	if (archiveReader.status() != QZipReader::NoError
		|| actualEntries.size() != static_cast<qsizetype>(beforeFiles.size()))
	{
		archiveReader.close();
		QFile::remove(zipPath);
		SetArchiveError(error, QStringLiteral("ZIP 中央目录回读失败或项目数不符：预期 %1，实际 %2")
			.arg(beforeFiles.size()).arg(actualEntries.size()));
		return false;
	}
	QSet<QString> seenEntries;
	seenEntries.reserve(actualEntries.size());
	for (const QZipReader::FileInfo& entry : actualEntries)
	{
		if (ArchiveCancelRequested(cancel))
		{
			archiveReader.close();
			QFile::remove(zipPath);
			SetArchiveError(error, QStringLiteral("已取消压缩（ZIP 中央目录复核）"));
			return false;
		}
		const QString normalizedPath = QDir::fromNativeSeparators(entry.filePath);
		if (!entry.isFile || entry.isDir || entry.isSymLink
			|| !expectedEntries.contains(normalizedPath)
			|| expectedEntries.value(normalizedPath) != entry.size
			|| seenEntries.contains(normalizedPath))
		{
			archiveReader.close();
			QFile::remove(zipPath);
			SetArchiveError(error, QStringLiteral("ZIP 中央目录包含非预期/重复/大小不符项：%1")
				.arg(normalizedPath));
			return false;
		}
		seenEntries.insert(normalizedPath);
	}
	archiveReader.close();

	const qint64 zipBytes = QFileInfo(zipPath).size();
	if (zipBytes <= 0 || zipBytes > kMaximumZipBytes)
	{
		QFile::remove(zipPath);
		SetArchiveError(error, QStringLiteral("最终 ZIP 大小无效或超过 2 GiB 上限（%1 字节）")
			.arg(zipBytes));
		return false;
	}

	// 压缩后重读源文件并重新遍历：内容、文件数、路径、大小或时间戳在归档期间
	// 任意变化都删除 ZIP 并失败，避免上传混合时点的不完整案例。
	for (size_t index = 0; index < beforeFiles.size(); ++index)
	{
		QByteArray afterDigest;
		QString readError;
		if (!HashFileExact(beforeFiles[index], &afterDigest, &readError, cancel)
			|| afterDigest != sourceDigests[index])
		{
			QFile::remove(zipPath);
			SetArchiveError(error, readError.isEmpty()
				? QStringLiteral("压缩期间文件内容变化：%1")
					.arg(QDir::toNativeSeparators(beforeFiles[index].absolutePath))
				: readError);
			return false;
		}
	}
	std::vector<ArchiveFileSnapshot> afterFiles;
	qint64 afterTotalBytes = 0;
	QString rescanError;
	if (!CollectArchiveFiles(caseDir, &afterFiles, &afterTotalBytes, &rescanError, cancel)
		|| afterTotalBytes != beforeTotalBytes
		|| !SameArchiveSnapshot(beforeFiles, afterFiles))
	{
		QFile::remove(zipPath);
		SetArchiveError(error, rescanError.isEmpty()
			? QStringLiteral("压缩期间案例文件清单或元数据变化")
			: rescanError);
		return false;
	}
	return true;
}
