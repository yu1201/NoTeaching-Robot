#include "OnlineServicesDialog.h"

#include "AppPaths.h"
#include "BrandingConfig.h"
#include "FtpClient.h"
#include "OnlineServicesConfig.h"
#include "RobotOperationLease.h"
#include "RobotLog.h"
#include "ScanDataUploader.h"

#include <QComboBox>
#include <QDate>
#include <QFileInfo>
#include <QPointer>

#include <algorithm>
#include <cmath>
#include <thread>

#ifdef Q_OS_WIN
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#include <bcrypt.h>
#pragma comment(lib, "bcrypt.lib")
#endif

#include <QtCore/private/qzipreader_p.h>

#include <QApplication>
#include <QCheckBox>
#include <QCryptographicHash>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileDialog>
#include <QSaveFile>
#include <QDialogButtonBox>
#include <QFormLayout>
#include <QFrame>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QHostAddress>
#include <QInputDialog>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QCloseEvent>
#include <QShowEvent>
#include <QMessageBox>
#include <QStackedWidget>
#include <QTableWidget>
#include <QNetworkAccessManager>
#include <QNetworkProxy>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QPlainTextEdit>
#include <QProcess>
#include <QProcessEnvironment>
#include <QProgressBar>
#include <QPushButton>
#include <QRegularExpression>
#include <QSignalBlocker>
#include <QTimer>
#include <QUrl>
#include <QVBoxLayout>

namespace
{
	constexpr qint64 kMaximumManifestBytes = 256 * 1024;
	constexpr qint64 kMaximumUpdatePayloadBytes = 512LL * 1024 * 1024;
	constexpr char kOtaSignatureAlgorithm[] = "RSA-PKCS1-SHA256";
	// RSA-3072 公钥；对应私钥仅以 CurrentUser DPAPI 形式保存在仓库外。
	constexpr char kOtaReleasePublicKeyBlobBase64[] =
		"UlNBMQAMAAADAAAAgAEAAAAAAAAAAAAAAQAB4ax2i7VFMjmJhtYWOJvaLLN+sAE/nXCUimEtrdo9l1co8mT3rYz2vy5lsB+ztcN+c+iXZ+G6YMy1Xrfp2AN3jKd6ZfXNG9z0UPXDlS/0AlUnONobSyVSkMtarODxrPNKb9Kq7+XkF/sgOxYzfgg9QVU8lfqD4pInm54C8+6OQDD8WpckvmUqOZ4jHeqgEzvavPiRyI+IR0WYDJdFh/NhQRpxGmzgNFvzhvzHyvALJ+KxKh7+YW0/r3+YvRaJeTD+bFJxO0q/ZeoMjtTz+WY9WgCHB+VnH7VKDtpUd2lPxYXc8y1wIkc78FYMQZFWcXGV5GVn7Lf8jG5QVmrOncg+GvkNErdAUlz+B1cntccuVhBQpeXVVK8J9gz802O4dDXU1xKSsHScEDYC4MtwsR/M9YWcackdYsBnMAPSu5fYqzTDWkQR2HbfYRom55QVh8cmVO3fK4DZ0Iyehky8Vi3UWusSbzkYhAsfLrn/c2eY7Jx4COFK/ZVyZWMRgS79sjnz";
	constexpr char kOtaKnownAnswerPayloadSha256[] =
		"2903794ff9ac838ef8db7c14a7b808fc5aba37e95f5cf330c00af08180f0a80f";
	constexpr char kOtaKnownAnswerSignatureBase64[] =
		"TXkCqU69V5nldnVs6V3J3J/4MoL/ziTvq/cI6JIrwZLiDR/q4I6N6P4FlRZWfTydY/d4HVWzGy8O6O01XcjGb858RteHjNtgajFWkUYSlu3CPdntJLg+QA/GyIDjucb0X9p859bRzhw1aRlQ9+dLVOYqdNeNKpFndeJVjERD+SLx2YHZTZmx+mIqGXlddNa56tchJOC4Ow8ZC+t7hYPznfMVH437w1s/YscTZRWIENV05JEFghtfRstcEXObuaXUvB0Eh2gFEtQT+6vLR3ItNJWDjx95x4ZiOdt09QR3YhPKl8DFMZHDJ2vT2Dia7UDV8y6XO9tG4urm//p+kPDLhLtKDpqg+bSbneHToVA0gpORg4q14BVagREGnyZNXql27QRi5d3nrfvsv4wQPev2g++xwKmTrdrmey16QpGQEYLnu0P+vPPR65Vb9fyM4GkkhCX4e2rNd3mZS7dNoFMAKK+g6BCibv4tL8/PPwJgleBCc3/PeHcRSV1UjpuLw25K";

	bool IsStrictOtaVersion(const QString& value)
	{
		static const QRegularExpression pattern(
			QStringLiteral(R"(^\d{4}\.\d{2}\.\d{2}\.\d{4}$)"));
		if (!pattern.match(value).hasMatch())
		{
			return false;
		}
		const QStringList parts = value.split(QLatin1Char('.'));
		bool yearOk = false;
		bool monthOk = false;
		bool dayOk = false;
		bool timeOk = false;
		const int year = parts.at(0).toInt(&yearOk);
		const int month = parts.at(1).toInt(&monthOk);
		const int day = parts.at(2).toInt(&dayOk);
		const int hhmm = parts.at(3).toInt(&timeOk);
		return yearOk && monthOk && dayOk && timeOk
			&& year >= 2000
			&& QDate(year, month, day).isValid()
			&& hhmm / 100 >= 0 && hhmm / 100 <= 23
			&& hhmm % 100 >= 0 && hhmm % 100 <= 59;
	}

	bool IsStrictSha256(const QString& value)
	{
		static const QRegularExpression pattern(QStringLiteral(R"(^[0-9a-f]{64}$)"));
		return pattern.match(value).hasMatch();
	}

	bool TryReadManifestSize(const QJsonValue& value, qint64* size)
	{
		if (size != nullptr)
		{
			*size = 0;
		}
		if (!value.isDouble())
		{
			return false;
		}
		const double number = value.toDouble();
		if (!std::isfinite(number)
			|| std::floor(number) != number
			|| number <= 0.0
			|| number > static_cast<double>(kMaximumUpdatePayloadBytes))
		{
			return false;
		}
		if (size != nullptr)
		{
			*size = static_cast<qint64>(number);
		}
		return true;
	}

	bool HasOnlyObjectKeys(
		const QJsonObject& object,
		std::initializer_list<const char*> allowedKeys)
	{
		for (auto it = object.constBegin(); it != object.constEnd(); ++it)
		{
			bool allowed = false;
			for (const char* key : allowedKeys)
			{
				if (it.key() == QLatin1String(key))
				{
					allowed = true;
					break;
				}
			}
			if (!allowed)
			{
				return false;
			}
		}
		return true;
	}

	bool IsStrictManifestShape(const QJsonObject& manifest)
	{
		if (!HasOnlyObjectKeys(manifest,
			{ "schemaVersion", "channel", "version", "file", "sha256", "size",
				"notes", "patch", "signatureAlgorithm", "signature" })
			|| manifest.size() < 9
			|| !manifest.value(QStringLiteral("schemaVersion")).isDouble()
			|| manifest.value(QStringLiteral("schemaVersion")).toDouble() != 2.0
			|| !manifest.value(QStringLiteral("channel")).isString()
			|| !manifest.value(QStringLiteral("version")).isString()
			|| !manifest.value(QStringLiteral("file")).isString()
			|| !manifest.value(QStringLiteral("sha256")).isString()
			|| !manifest.value(QStringLiteral("size")).isDouble()
			|| !manifest.value(QStringLiteral("notes")).isString()
			|| !manifest.value(QStringLiteral("signatureAlgorithm")).isString()
			|| !manifest.value(QStringLiteral("signature")).isString())
		{
			return false;
		}

		if (!manifest.contains(QStringLiteral("patch")))
		{
			return true;
		}
		const QJsonValue patchValue = manifest.value(QStringLiteral("patch"));
		if (!patchValue.isObject())
		{
			return false;
		}
		const QJsonObject patch = patchValue.toObject();
		return patch.size() == 4
			&& HasOnlyObjectKeys(patch, { "file", "sha256", "size", "baseMinVersion" })
			&& patch.value(QStringLiteral("file")).isString()
			&& patch.value(QStringLiteral("sha256")).isString()
			&& patch.value(QStringLiteral("size")).isDouble()
			&& patch.value(QStringLiteral("baseMinVersion")).isString();
	}

	QString HashFileSha256(const QString& path)
	{
		QFile file(path);
		if (!file.open(QIODevice::ReadOnly))
		{
			return {};
		}
		QCryptographicHash hash(QCryptographicHash::Sha256);
		while (!file.atEnd())
		{
			const QByteArray chunk = file.read(1024 * 1024);
			if (chunk.isEmpty() && file.error() != QFileDevice::NoError)
			{
				return {};
			}
			hash.addData(chunk);
		}
		return QString::fromLatin1(hash.result().toHex()).toLower();
	}

	QByteArray BuildOtaManifestSignaturePayload(const QJsonObject& manifest)
	{
		const QString notes = manifest.value(QStringLiteral("notes")).toString();
		const QString notesSha256 = QString::fromLatin1(
			QCryptographicHash::hash(notes.toUtf8(), QCryptographicHash::Sha256).toHex());
		const QJsonObject patch = manifest.value(QStringLiteral("patch")).toObject();
		const auto integerText = [](const QJsonValue& value) -> QByteArray
			{
				const double number = value.toDouble(0.0);
				return std::isfinite(number) && std::floor(number) == number
					? QByteArray::number(static_cast<qint64>(number))
					: QByteArrayLiteral("0");
			};
		QByteArray payload("NoTeaching-Robot OTA Manifest Signature v2\n");
		const auto appendField = [&payload](const char* name, const QByteArray& value)
			{
				payload.append(name);
				payload.append('=');
				payload.append(value);
				payload.append('\n');
			};
		appendField("schemaVersion", QByteArrayLiteral("2"));
		appendField("channel", manifest.value(QStringLiteral("channel")).toString().toUtf8());
		appendField("version", manifest.value(QStringLiteral("version")).toString().toUtf8());
		appendField("file", manifest.value(QStringLiteral("file")).toString().toUtf8());
		appendField("sha256", manifest.value(QStringLiteral("sha256")).toString().toUtf8());
		appendField("size", integerText(manifest.value(QStringLiteral("size"))));
		appendField("notesSha256", notesSha256.toLatin1());
		appendField("patch.file", patch.value(QStringLiteral("file")).toString().toUtf8());
		appendField("patch.sha256", patch.value(QStringLiteral("sha256")).toString().toUtf8());
		appendField("patch.size", integerText(patch.value(QStringLiteral("size"))));
		appendField("patch.baseMinVersion",
			patch.value(QStringLiteral("baseMinVersion")).toString().toUtf8());
		return payload;
	}

	bool VerifyOtaRsaSignature(const QJsonObject& manifest, const QByteArray& signature)
	{
#ifdef Q_OS_WIN
		const QByteArray publicBlob = QByteArray::fromBase64(kOtaReleasePublicKeyBlobBase64);
		const QByteArray digest = QCryptographicHash::hash(
			BuildOtaManifestSignaturePayload(manifest), QCryptographicHash::Sha256);
		BCRYPT_ALG_HANDLE algorithm = nullptr;
		BCRYPT_KEY_HANDLE key = nullptr;
		NTSTATUS status = BCryptOpenAlgorithmProvider(
			&algorithm, BCRYPT_RSA_ALGORITHM, nullptr, 0);
		if (status >= 0)
		{
			status = BCryptImportKeyPair(
				algorithm,
				nullptr,
				BCRYPT_RSAPUBLIC_BLOB,
				&key,
				reinterpret_cast<PUCHAR>(const_cast<char*>(publicBlob.constData())),
				static_cast<ULONG>(publicBlob.size()),
				0);
		}
		if (status >= 0)
		{
			BCRYPT_PKCS1_PADDING_INFO paddingInfo{ BCRYPT_SHA256_ALGORITHM };
			status = BCryptVerifySignature(
				key,
				&paddingInfo,
				reinterpret_cast<PUCHAR>(const_cast<char*>(digest.constData())),
				static_cast<ULONG>(digest.size()),
				reinterpret_cast<PUCHAR>(const_cast<char*>(signature.constData())),
				static_cast<ULONG>(signature.size()),
				BCRYPT_PAD_PKCS1);
		}
		if (key != nullptr)
		{
			BCryptDestroyKey(key);
		}
		if (algorithm != nullptr)
		{
			BCryptCloseAlgorithmProvider(algorithm, 0);
		}
		return status >= 0;
#else
		Q_UNUSED(manifest);
		return false;
#endif
	}

	bool OtaManifestKnownAnswerSelfTest()
	{
		const QJsonObject patch{
			{ QStringLiteral("file"), QStringLiteral("HK-Pathlynx-CORPLA-Patch-v2026.07.12.0030.zip") },
			{ QStringLiteral("sha256"), QString(64, QLatin1Char('b')) },
			{ QStringLiteral("size"), 3456789 },
			{ QStringLiteral("baseMinVersion"), QStringLiteral("2026.07.10.1750") }
		};
		const QJsonObject manifest{
			{ QStringLiteral("schemaVersion"), 2 },
			{ QStringLiteral("channel"), QStringLiteral("brand") },
			{ QStringLiteral("version"), QStringLiteral("2026.07.12.0030") },
			{ QStringLiteral("file"), QStringLiteral("HK-Pathlynx-CORPLA-Setup-v2026.07.12.0030.exe") },
			{ QStringLiteral("sha256"), QString(64, QLatin1Char('a')) },
			{ QStringLiteral("size"), 86657296 },
			{ QStringLiteral("notes"), QStringLiteral("签名回归") },
			{ QStringLiteral("patch"), patch },
			{ QStringLiteral("signatureAlgorithm"), QString::fromLatin1(kOtaSignatureAlgorithm) }
		};
		const QByteArray payloadDigest = QCryptographicHash::hash(
			BuildOtaManifestSignaturePayload(manifest), QCryptographicHash::Sha256).toHex();
		const QByteArray signature = QByteArray::fromBase64(kOtaKnownAnswerSignatureBase64);
		return payloadDigest == QByteArray(kOtaKnownAnswerPayloadSha256)
			&& signature.toBase64() == QByteArray(kOtaKnownAnswerSignatureBase64)
			&& VerifyOtaRsaSignature(manifest, signature);
	}

	bool VerifyOtaManifestSignature(const QJsonObject& manifest)
	{
		static const bool knownAnswerPassed = OtaManifestKnownAnswerSelfTest();
		if (!knownAnswerPassed
			|| manifest.value(QStringLiteral("schemaVersion")).toDouble() != 2.0
			|| manifest.value(QStringLiteral("signatureAlgorithm")).toString()
				!= QString::fromLatin1(kOtaSignatureAlgorithm))
		{
			return false;
		}
		const QByteArray encodedSignature =
			manifest.value(QStringLiteral("signature")).toString().toLatin1();
		const QByteArray signature = QByteArray::fromBase64(encodedSignature);
		return !signature.isEmpty()
			&& signature.toBase64() == encodedSignature
			&& VerifyOtaRsaSignature(manifest, signature);
	}

	QString ExpectedInstallerName(const QString& channel, const QString& version)
	{
		return channel == QStringLiteral("brand")
			? QStringLiteral("HK-Pathlynx-CORPLA-Setup-v%1.exe").arg(version)
			: QStringLiteral("NoTeaching-Robot-Setup-v%1.exe").arg(version);
	}

	QString ExpectedPatchName(const QString& version)
	{
		return QStringLiteral("HK-Pathlynx-CORPLA-Patch-v%1.zip").arg(version);
	}

	QString EscapeBatchLiteral(QString value)
	{
		// cmd.exe 会在双引号内仍展开 %VAR%；路径中的百分号必须成对转义。
		value.replace(QStringLiteral("%"), QStringLiteral("%%"));
		return value;
	}

	bool IsAllowedOtaUrl(const QUrl& url)
	{
		const QString scheme = url.scheme().toLower();
		return url.isValid()
			&& (scheme == QStringLiteral("http") || scheme == QStringLiteral("https"))
			&& !url.host().isEmpty()
			&& url.userName().isEmpty()
			&& url.password().isEmpty();
	}

	QString DownloadTempDir()
	{
		return AppPaths::WritablePath(QStringLiteral("Temp/OnlineUpdate"));
	}

	QString PatchFailureMarkerPath(const QString& version)
	{
		if (!IsStrictOtaVersion(version))
		{
			return {};
		}
		return AppPaths::WritableChildPath(
			QStringLiteral("Temp/OnlineUpdate"),
			QStringLiteral("patch_failed_%1.flag").arg(version));
	}

	QString PatchHealthMarkerPath(const QString& version)
	{
		if (!IsStrictOtaVersion(version))
		{
			return {};
		}
		return AppPaths::WritableChildPath(
			QStringLiteral("Temp/OnlineUpdate"),
			QStringLiteral("patch_healthy_%1.flag").arg(version));
	}

	bool PatchFailedForVersion(const QString& version)
	{
		const QString markerPath = PatchFailureMarkerPath(version);
		return !markerPath.isEmpty() && QFile::exists(markerPath);
	}

	bool IsSafeRemotePathComponent(const QString& value)
	{
		return AppPaths::IsSafePathComponent(value);
	}

	bool IsCompleteRemoteArchive(const QString& fileName, qulonglong actualBytes)
	{
		if (!fileName.endsWith(QStringLiteral(".zip"), Qt::CaseInsensitive))
		{
			return false;
		}
		// 新写一次上传名包含声明长度；不完整 STOR 即使被服务器当作 226，
		// 也不会出现在数据列表。旧版本归档没有该后缀，保持向后可见。
		static const QRegularExpression writeOncePattern(QStringLiteral(
			R"(_\d{8}T\d{9}_[0-9a-f]{12}_(\d+)\.zip$)"),
			QRegularExpression::CaseInsensitiveOption);
		const QRegularExpressionMatch match = writeOncePattern.match(fileName);
		if (!match.hasMatch())
		{
			return true;
		}
		bool sizeOk = false;
		const qulonglong expectedBytes = match.captured(1).toULongLong(&sizeOk);
		return sizeOk && expectedBytes > 0 && expectedBytes == actualBytes;
	}

	bool IsSafeArchiveEntry(const QZipReader::FileInfo& entry)
	{
		if (!entry.isValid() || entry.isSymLink)
		{
			return false;
		}
		QString normalized = QDir::fromNativeSeparators(entry.filePath);
		while (normalized.endsWith(QLatin1Char('/')))
		{
			normalized.chop(1);
		}
		if (normalized.isEmpty() || QDir::isAbsolutePath(normalized))
		{
			return false;
		}
		const QStringList components = normalized.split(QLatin1Char('/'), Qt::KeepEmptyParts);
		return std::all_of(components.cbegin(), components.cend(), [](const QString& component)
			{
				return AppPaths::IsSafePathComponent(component);
			});
	}

	bool HasOnlySafeArchiveEntries(const QZipReader& archive)
	{
		const QList<QZipReader::FileInfo> entries = archive.fileInfoList();
		return archive.isReadable()
			&& !entries.isEmpty()
			&& std::all_of(entries.cbegin(), entries.cend(), IsSafeArchiveEntry);
	}

	bool IsSafeExecutableOnlyPatch(const QString& archivePath)
	{
		QZipReader archive(archivePath);
		const QList<QZipReader::FileInfo> entries = archive.fileInfoList();
		const QString expectedExeName = QFileInfo(QCoreApplication::applicationFilePath()).fileName();
		const bool safe = archive.isReadable()
			&& entries.size() == 1
			&& entries.first().isFile
			&& !entries.first().isSymLink
			&& entries.first().filePath == expectedExeName
			&& IsSafeRemotePathComponent(expectedExeName)
			&& IsSafeArchiveEntry(entries.first());
		archive.close();
		return safe;
	}

	bool BytesContainVersion(const QByteArray& bytes, const QString& version)
	{
		const QByteArray ascii = version.toLatin1();
		QByteArray utf16;
		utf16.reserve(version.size() * 2);
		for (const QChar character : version)
		{
			const ushort value = character.unicode();
			utf16.append(static_cast<char>(value & 0xff));
			utf16.append(static_cast<char>((value >> 8) & 0xff));
		}
		return bytes.contains(ascii) || bytes.contains(utf16);
	}

	bool ExtractExecutablePatchToStaging(
		const QString& archivePath,
		const QString& expectedVersion,
		const QString& stagingPath,
		QString* stagedSha256,
		qint64* stagedSize)
	{
		if (stagedSha256 != nullptr)
		{
			stagedSha256->clear();
		}
		if (stagedSize != nullptr)
		{
			*stagedSize = 0;
		}
		QZipReader archive(archivePath);
		const QList<QZipReader::FileInfo> entries = archive.fileInfoList();
		const QString expectedExeName = QFileInfo(QCoreApplication::applicationFilePath()).fileName();
		if (!archive.isReadable()
			|| entries.size() != 1
			|| !entries.first().isFile
			|| entries.first().isSymLink
			|| entries.first().filePath != expectedExeName
			|| entries.first().size <= 0
			|| entries.first().size > kMaximumUpdatePayloadBytes
			|| !IsSafeArchiveEntry(entries.first()))
		{
			archive.close();
			return false;
		}
		const QByteArray executableBytes = archive.fileData(expectedExeName);
		archive.close();
		if (executableBytes.size() != entries.first().size
			|| !BytesContainVersion(executableBytes, expectedVersion))
		{
			return false;
		}

		QSaveFile staging(stagingPath);
		if (!staging.open(QIODevice::WriteOnly)
			|| staging.write(executableBytes) != executableBytes.size()
			|| !staging.commit())
		{
			staging.cancelWriting();
			return false;
		}
		const QString digest = HashFileSha256(stagingPath);
		if (!IsStrictSha256(digest) || QFileInfo(stagingPath).size() != executableBytes.size())
		{
			QFile::remove(stagingPath);
			return false;
		}
		if (stagedSha256 != nullptr)
		{
			*stagedSha256 = digest;
		}
		if (stagedSize != nullptr)
		{
			*stagedSize = executableBytes.size();
		}
		return true;
	}

	std::string OnlineServicesLogPath()
	{
		return QDir::toNativeSeparators(
			AppPaths::WritablePath(QStringLiteral("Log/OnlineServices.txt")))
			.toLocal8Bit().toStdString();
	}

	QString HumanBytes(double bytes)
	{
		if (bytes >= 1024.0 * 1024.0 * 1024.0)
		{
			return QStringLiteral("%1 GB").arg(QString::number(bytes / (1024.0 * 1024.0 * 1024.0), 'f', 1));
		}
		if (bytes >= 1024.0 * 1024.0)
		{
			return QStringLiteral("%1 MB").arg(QString::number(bytes / (1024.0 * 1024.0), 'f', 1));
		}
		return QStringLiteral("%1 KB").arg(QString::number(bytes / 1024.0, 'f', 0));
	}
}

OnlineServicesDialog::OnlineServicesDialog(ScanDataUploader* uploader,
	std::function<bool()> flowRunningGuard,
	bool aboutMode,
	bool remoteBrowseAllowed,
	std::function<bool()> privilegedActionGuard,
	QWidget* parent)
	: QDialog(parent)
	, m_uploader(uploader)
	, m_flowRunningGuard(std::move(flowRunningGuard))
	, m_privilegedActionGuard(std::move(privilegedActionGuard))
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
				UpdateQueueCard();
			});
	}

	if (!m_aboutMode)
	{
		UpdateQueueCard();
		// 配了管理令牌才自动拉服务器统计/账号（没配就静默，卡片显示「待刷新」，避免开页报错刷屏）。
		if (!OnlineServicesConfig::AdminToken().trimmed().isEmpty())
		{
			QTimer::singleShot(0, this, [this]()
				{
					if (m_accountTable != nullptr)
					{
						RefreshAccounts();
					}
				});
		}
	}

	if (m_aboutMode)
	{
		// 关于页打开即自动查一次新版（结果落在「最新版本/更新说明」区，失败只记日志）。
		QTimer::singleShot(0, this, [this]() { CheckForUpdate(); });
	}
}

void OnlineServicesDialog::showEvent(QShowEvent* event)
{
	QDialog::showEvent(event);
	if (m_aboutMode)
	{
		return;
	}
	// 页面被管理栈缓存复用，构造只跑一次：每次重新显示都把数据刷成最新。
	UpdateQueueCard();
	if (!OnlineServicesConfig::AdminToken().trimmed().isEmpty())
	{
		RefreshServerStats();
	}
	if (m_navList != nullptr && m_remoteNavRow >= 0 && m_navList->currentRow() == m_remoteNavRow)
	{
		RefreshRemoteDevices();   // 上次停在「远程数据」页：重开界面也自动刷新
	}
}

void OnlineServicesDialog::BuildUi()
{
	// 统一控件样式（深色控制台风格）：普通/主操作/危险按钮、分组框、表格、输入框、下拉。
	// 按钮用动态属性 kind=primary/danger 区分主次；qproperty 变化后需 unpolish/polish，但这里创建时即设，无需刷新。
	if (!m_aboutMode)
	{
		setStyleSheet(QStringLiteral(
			"QGroupBox { border: 1px solid #294049; border-radius: 10px; margin-top: 10px; background: #101E24; }"
			"QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 6px; color: #9ED8DB; font-weight: 600; }"
			"QPushButton { background: #1F3542; color: #E7F3F5; border: 1px solid #3A5A69; border-radius: 7px;"
			"  padding: 7px 16px; font-size: 13px; min-height: 20px; }"
			"QPushButton:hover { background: #2A4756; border-color: #4E7889; }"
			"QPushButton:pressed { background: #18303B; }"
			"QPushButton:disabled { background: #1A2A31; color: #5E7982; border-color: #263C44; }"
			"QPushButton[kind=\"primary\"] { background: #1F5A46; border-color: #3E8E70; color: #EAFBF3; font-weight: 600; }"
			"QPushButton[kind=\"primary\"]:hover { background: #2A7358; }"
			"QPushButton[kind=\"danger\"] { background: #5A2323; border-color: #8E3E3E; color: #FBEAEA; }"
			"QPushButton[kind=\"danger\"]:hover { background: #733030; }"
			"QLineEdit { background: #0E1A1F; color: #E7F3F5; border: 1px solid #2E4A57; border-radius: 6px; padding: 6px 8px; selection-background-color: #2A7358; }"
			"QLineEdit:focus { border-color: #72D4DD; }"
			"QComboBox { background: #0E1A1F; color: #E7F3F5; border: 1px solid #2E4A57; border-radius: 6px; padding: 5px 8px; }"
			"QComboBox:focus { border-color: #72D4DD; }"
			"QComboBox QAbstractItemView { background: #0E1A1F; color: #E7F3F5; selection-background-color: #2A4756; }"
			"QTableWidget { background: #0E1A1F; alternate-background-color: #12222A; color: #D6E7EA; gridline-color: #22383F;"
			"  border: 1px solid #294049; border-radius: 8px; selection-background-color: #1F5A46; selection-color: #EAFBF3; }"
			"QTableWidget::item { padding: 4px 8px; }"
			"QHeaderView::section { background: #16262E; color: #9ED8DB; border: none; border-bottom: 1px solid #294049;"
			"  padding: 8px; font-weight: 600; }"
			"QListWidget { background: #0E1A1F; color: #D6E7EA; border: 1px solid #294049; border-radius: 8px; }"
			"QListWidget::item { padding: 6px 8px; }"
			"QListWidget::item:selected { background: #1F5A46; color: #EAFBF3; border-left: 3px solid #7BE0A2; }"
			"QListWidget::item:hover { background: #16262E; }"));
	}

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
	m_checkUpdateBtn->setProperty("kind", "primary");
	m_downloadInstallBtn = new QPushButton(QStringLiteral("下载并安装"), this);
	m_downloadInstallBtn->setMinimumHeight(44);
	m_downloadInstallBtn->setProperty("kind", "primary");
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
	updateLayout->setRowStretch(3, 1);   // 内容贴顶：多余高度给底部空行，不再把控件竖向摊开
	if (m_aboutMode)
	{
		mainLayout->addWidget(updateGroup);   // 关于页保持精简：只有升级区（仪表盘模式挂进页签）
	}

	// —— 数据上传 ——
	QGroupBox* uploadGroup = new QGroupBox(QStringLiteral("扫描数据上传"), this);
	QGridLayout* uploadLayout = new QGridLayout(uploadGroup);
	m_autoUploadCheck = new QCheckBox(QStringLiteral("扫描流程完成后自动上传"), this);
	m_uploadNowBtn = new QPushButton(QStringLiteral("立即上传待传项"), this);
	m_uploadNowBtn->setMinimumHeight(40);
	m_uploadNowBtn->setProperty("kind", "primary");
	m_uploadPickBtn = new QPushButton(QStringLiteral("选择案例上传…"), this);
	m_uploadPickBtn->setMinimumHeight(40);
	m_pendingListWidget = new QListWidget(this);
	// 队列列表占满页面剩余高度（不设上限），内容自然贴顶。
	uploadLayout->addWidget(m_autoUploadCheck, 0, 0);
	uploadLayout->addWidget(m_uploadNowBtn, 0, 1);
	uploadLayout->addWidget(m_uploadPickBtn, 0, 2);
	uploadLayout->addWidget(new QLabel(QStringLiteral("待上传队列："), this), 1, 0);
	uploadLayout->addWidget(m_pendingListWidget, 2, 0, 1, 3);
	uploadLayout->setRowStretch(2, 1);
	uploadGroup->setVisible(!m_aboutMode);

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
	// 管理令牌：服务器管理接口（账号管理/磁盘统计）鉴权用，管理员手填、混淆存储。
	m_adminTokenEdit = new QLineEdit(this);
	m_adminTokenEdit->setEchoMode(QLineEdit::Password);
	m_adminTokenEdit->setPlaceholderText(QStringLiteral("管理令牌（账号管理/服务器统计用，管理员填写）"));
	configLayout->addWidget(new QLabel(QStringLiteral("管理令牌"), this), 4, 0);
	configLayout->addWidget(m_adminTokenEdit, 4, 1, 1, 2);
	configLayout->addWidget(saveConfigBtn, 4, 3);
	configLayout->setRowStretch(5, 1);   // 配置行贴顶
	// 升级源、FTP 目的端和管理令牌都会改变软件/数据的信任边界，
	// 只允许持有实时本地 admin 会话的页面查看和修改。
	configGroup->setVisible(!m_aboutMode && m_remoteBrowseAllowed);

	// —— 远程数据（admin）：浏览/下载/删除各设备上传到服务器的扫描数据、新建设备目录 ——
	QGroupBox* remoteGroup = nullptr;
	if (!m_aboutMode && m_remoteBrowseAllowed)
	{
		// 全权限账号可浏览/下载全部设备；只上传账号(uploader)自动锁定为仅本机设备、禁下载。
		remoteGroup = new QGroupBox(QStringLiteral("远程数据"), this);
		// 纵向布局：顶行操作条(靠左) + 文件列表(占满) + 底行按钮(靠右)。
		// 勿用无列拉伸的 QGridLayout：文件列表撑满页宽后多余宽度会分进中间列，标签和下拉间被拉出大空。
		QVBoxLayout* remoteLayout = new QVBoxLayout(remoteGroup);
		remoteLayout->setSpacing(10);
		m_remoteRefreshBtn = new QPushButton(QStringLiteral("刷新设备列表"), this);
		m_remoteRefreshBtn->setMinimumHeight(40);
		m_remoteDeviceCombo = new QComboBox(this);
		m_remoteDeviceCombo->setFixedWidth(300);
		m_remoteFileList = new QListWidget(this);
		m_remoteFileList->setSelectionMode(QAbstractItemView::ExtendedSelection);
		// 文件列表占满页面剩余高度，内容贴顶。
		m_remoteDownloadBtn = new QPushButton(QStringLiteral("下载选中到本地"), this);
		m_remoteDownloadBtn->setMinimumHeight(40);
		m_remoteDownloadBtn->setToolTip(QStringLiteral("下载并自动解压到 Result\\Remote\\<设备>\\，可用点云查看等工具直接打开。"));
		m_remoteDownloadBtn->setProperty("kind", "primary");
		m_remoteDeleteBtn = new QPushButton(QStringLiteral("删除选中（服务器）"), this);
		m_remoteDeleteBtn->setMinimumHeight(40);
		m_remoteDeleteBtn->setProperty("kind", "danger");
		m_remoteDeleteBtn->setToolTip(QStringLiteral("从服务器上永久删除选中的数据包（不影响设备本地数据）。"));
		m_remoteMkdirBtn = new QPushButton(QStringLiteral("新建设备目录…"), this);
		m_remoteMkdirBtn->setMinimumHeight(40);
		QHBoxLayout* remoteTopRow = new QHBoxLayout();
		remoteTopRow->setSpacing(8);
		remoteTopRow->addWidget(m_remoteRefreshBtn);
		remoteTopRow->addWidget(new QLabel(QStringLiteral("设备："), this));
		remoteTopRow->addWidget(m_remoteDeviceCombo);
		remoteTopRow->addWidget(m_remoteDownloadBtn);
		remoteTopRow->addStretch(1);
		remoteLayout->addLayout(remoteTopRow);
		remoteLayout->addWidget(m_remoteFileList, 1);
		QHBoxLayout* remoteBottomRow = new QHBoxLayout();
		remoteBottomRow->setSpacing(8);
		remoteBottomRow->addStretch(1);
		remoteBottomRow->addWidget(m_remoteDeleteBtn);
		remoteBottomRow->addWidget(m_remoteMkdirBtn);
		remoteLayout->addLayout(remoteBottomRow);

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
		connect(m_remoteDeleteBtn, &QPushButton::clicked, this, [this]()
			{
				DeleteSelectedRemoteFiles();
			});
		connect(m_remoteMkdirBtn, &QPushButton::clicked, this, [this]()
			{
				CreateRemoteDeviceDir();
			});
	}

	// —— 账号管理（admin，经服务器管理接口：nginx /admin/ + X-Admin-Token）——
	QGroupBox* accountGroup = nullptr;
	if (!m_aboutMode && m_remoteBrowseAllowed)
	{
		accountGroup = new QGroupBox(QStringLiteral("FTP 账号管理（管理员）"), this);
		QVBoxLayout* accLayout = new QVBoxLayout(accountGroup);
		accLayout->setSpacing(10);
		// 工具条：主操作（添加）+ 选中项操作分组，左对齐紧凑，后置 stretch 不撑满整行。
		QHBoxLayout* accToolbar = new QHBoxLayout();
		accToolbar->setSpacing(8);
		QPushButton* accAddBtn = new QPushButton(QStringLiteral("＋ 添加账号"), this);
		accAddBtn->setProperty("kind", "primary");
		QPushButton* accPwBtn = new QPushButton(QStringLiteral("改密码"), this);
		QPushButton* accPermBtn = new QPushButton(QStringLiteral("切换权限"), this);
		QPushButton* accDelBtn = new QPushButton(QStringLiteral("删除账号"), this);
		accDelBtn->setProperty("kind", "danger");
		QPushButton* accRefreshBtn = new QPushButton(QStringLiteral("刷新"), this);
		accToolbar->addWidget(accAddBtn);
		accToolbar->addSpacing(6);
		accToolbar->addWidget(accPwBtn);
		accToolbar->addWidget(accPermBtn);
		accToolbar->addWidget(accDelBtn);
		accToolbar->addStretch();
		accToolbar->addWidget(accRefreshBtn);
		accLayout->addLayout(accToolbar);

		m_accountTable = new QTableWidget(0, 3, this);
		m_accountTable->setHorizontalHeaderLabels({ QStringLiteral("账号"), QStringLiteral("权限"), QStringLiteral("说明") });
		m_accountTable->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Fixed);
		m_accountTable->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Fixed);
		m_accountTable->horizontalHeader()->setSectionResizeMode(2, QHeaderView::Stretch);
		m_accountTable->setColumnWidth(0, 200);
		m_accountTable->setColumnWidth(1, 120);
		m_accountTable->verticalHeader()->setVisible(false);
		m_accountTable->verticalHeader()->setDefaultSectionSize(38);
		m_accountTable->setSelectionBehavior(QAbstractItemView::SelectRows);
		m_accountTable->setSelectionMode(QAbstractItemView::SingleSelection);
		m_accountTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
		m_accountTable->setAlternatingRowColors(true);
		accLayout->addWidget(m_accountTable, 1);
		QLabel* accHint = new QLabel(QStringLiteral(
			"「仅上传」账号能上传但下载不到任何数据（现场设备默认）；「全权限」可上传下载浏览全部设备。"
			"uploader/devicedata 为系统保护账号不可删除；改 uploader 密码会让所有用默认账号的现场设备断传，慎改。"), this);
		accHint->setWordWrap(true);
		accHint->setStyleSheet("color: #7E9AA6; font-size: 11px;");
		accLayout->addWidget(accHint);

		connect(accRefreshBtn, &QPushButton::clicked, this, [this]() { SaveConfigFromUi(); RefreshAccounts(); });
		connect(accAddBtn, &QPushButton::clicked, this, [this]() { ShowAddAccountDialog(); });
		connect(accPwBtn, &QPushButton::clicked, this, [this]() { ChangeSelectedAccountPassword(); });
		connect(accPermBtn, &QPushButton::clicked, this, [this]() { ToggleSelectedAccountPermission(); });
		connect(accDelBtn, &QPushButton::clicked, this, [this]() { DeleteSelectedAccount(); });
	}

	// —— 仪表盘组装：左侧导航 + 右侧页面栈（云控制台式布局，关于页不走这里）——
	if (!m_aboutMode)
	{
		// 总览页：大数字统计卡 ×4 + 设备资源列表。
		QWidget* overviewPage = new QWidget(this);
		QVBoxLayout* ovLayout = new QVBoxLayout(overviewPage);
		ovLayout->setContentsMargins(0, 0, 0, 0);
		ovLayout->setSpacing(10);

		QHBoxLayout* ovHeader = new QHBoxLayout();
		QLabel* ovTitle = new QLabel(QStringLiteral("服务器总览"), overviewPage);
		ovTitle->setStyleSheet("font-size: 16px; font-weight: bold; color: #9ED8DB;");
		QLabel* ovVersion = new QLabel(QStringLiteral("软件 v%1（%2通道）")
			.arg(QApplication::applicationVersion())
			.arg(UpdateChannel() == QStringLiteral("brand") ? QStringLiteral("品牌") : QStringLiteral("中性")), overviewPage);
		ovVersion->setStyleSheet("color: #7E9AA6; font-size: 12px;");
		QPushButton* statsRefreshBtn = new QPushButton(QStringLiteral("刷新状态"), overviewPage);
		statsRefreshBtn->setMinimumHeight(38);
		connect(statsRefreshBtn, &QPushButton::clicked, this, [this]()
			{
				SaveConfigFromUi();
				RefreshServerStats();
				UpdateQueueCard();
			});
		ovHeader->addWidget(ovTitle);
		ovHeader->addSpacing(14);
		ovHeader->addWidget(ovVersion);
		ovHeader->addStretch();
		ovHeader->addWidget(statsRefreshBtn);
		ovLayout->addLayout(ovHeader);

		// 大数字统计卡：标题小字 + 数值大字 + 说明小字（+磁盘卡带用量进度条）。
		auto makeStatCard = [](QWidget* parent, const QString& caption, QLabel*& valueLabel,
			QLabel** subLabel, const QString& accentColor) -> QFrame*
			{
				QFrame* card = new QFrame(parent);
				card->setStyleSheet(
					"QFrame { background: #13232B; border: 1px solid #2E4A57; border-radius: 12px; }"
					"QLabel { border: none; background: transparent; }");
				QVBoxLayout* cardLayout = new QVBoxLayout(card);
				cardLayout->setContentsMargins(14, 12, 14, 12);
				cardLayout->setSpacing(4);
				QLabel* cap = new QLabel(caption, card);
				cap->setStyleSheet("color: #7E9AA6; font-size: 12px;");
				valueLabel = new QLabel(QStringLiteral("--"), card);
				valueLabel->setStyleSheet(QStringLiteral("color: %1; font-size: 26px; font-weight: bold;").arg(accentColor));
				cardLayout->addWidget(cap);
				cardLayout->addWidget(valueLabel);
				if (subLabel != nullptr)
				{
					*subLabel = new QLabel(QString(), card);
					(*subLabel)->setStyleSheet("color: #8FB0BC; font-size: 11px;");
					(*subLabel)->setWordWrap(true);
					cardLayout->addWidget(*subLabel);
				}
				return card;
			};
		QHBoxLayout* cardsRow = new QHBoxLayout();
		cardsRow->setSpacing(10);
		QFrame* diskCard = makeStatCard(overviewPage, QStringLiteral("服务器磁盘"), m_cardDisk, &m_cardDiskSub, QStringLiteral("#72D4DD"));
		m_diskBar = new QProgressBar(diskCard);
		m_diskBar->setRange(0, 100);
		m_diskBar->setValue(0);
		m_diskBar->setTextVisible(false);
		m_diskBar->setFixedHeight(6);
		m_diskBar->setStyleSheet(
			"QProgressBar { background: #0E1A1F; border: none; border-radius: 3px; }"
			"QProgressBar::chunk { background: #72D4DD; border-radius: 3px; }");
		static_cast<QVBoxLayout*>(diskCard->layout())->addWidget(m_diskBar);
		cardsRow->addWidget(diskCard, 1);
		cardsRow->addWidget(makeStatCard(overviewPage, QStringLiteral("云端数据"), m_cardCloud, nullptr, QStringLiteral("#7BE0A2")), 1);
		cardsRow->addWidget(makeStatCard(overviewPage, QStringLiteral("设备数"), m_cardDevices, nullptr, QStringLiteral("#E8D28A")), 1);
		cardsRow->addWidget(makeStatCard(overviewPage, QStringLiteral("本机待传队列"), m_cardQueue, nullptr, QStringLiteral("#F0A0A0")), 1);
		ovLayout->addLayout(cardsRow);

		QLabel* devTitle = new QLabel(QStringLiteral("设备资源列表"), overviewPage);
		devTitle->setStyleSheet("color: #9ED8DB; font-size: 13px; font-weight: 600;");
		ovLayout->addWidget(devTitle);
		m_deviceTable = new QTableWidget(0, 4, overviewPage);
		m_deviceTable->setHorizontalHeaderLabels({ QStringLiteral("设备名"), QStringLiteral("数据量"),
			QStringLiteral("文件数"), QStringLiteral("最近上传") });
		m_deviceTable->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
		m_deviceTable->verticalHeader()->setVisible(false);
		m_deviceTable->setSelectionBehavior(QAbstractItemView::SelectRows);
		m_deviceTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
		ovLayout->addWidget(m_deviceTable, 1);

		// 左侧导航 + 右侧页面栈。
		m_navList = new QListWidget(this);
		m_navList->setFixedWidth(150);
		m_navList->setStyleSheet(
			"QListWidget { background: #0E1A1F; border: 1px solid #294049; border-radius: 10px; outline: none; }"
			"QListWidget::item { color: #AFC8CE; padding: 12px 14px; font-size: 14px; }"
			"QListWidget::item:selected { background: #1F3542; color: #F4FAFA; border-left: 3px solid #72D4DD; }"
			"QListWidget::item:hover { background: #16262E; }");
		m_pagesStack = new QStackedWidget(this);

		auto addNavPage = [this](const QString& title, QWidget* page)
			{
				m_navList->addItem(title);
				m_pagesStack->addWidget(page);
			};
		addNavPage(QStringLiteral("总览"), overviewPage);
		addNavPage(QStringLiteral("在线升级"), updateGroup);
		addNavPage(QStringLiteral("数据上传"), uploadGroup);
		if (remoteGroup != nullptr)
		{
			m_remoteNavRow = m_navList->count();
			addNavPage(QStringLiteral("远程数据"), remoteGroup);
		}
		if (accountGroup != nullptr)
		{
			m_accountNavRow = m_navList->count();
			addNavPage(QStringLiteral("账号管理"), accountGroup);
		}
		addNavPage(QStringLiteral("服务器配置"), configGroup);
		connect(m_navList, &QListWidget::currentRowChanged, this, [this](int row)
			{
				if (m_pagesStack == nullptr || row < 0 || row >= m_pagesStack->count())
				{
					return;
				}
				// 受限账号：即便被程序化选中到禁用页，也弹回总览，绝不显示别的设备数据。
				QListWidgetItem* item = m_navList->item(row);
				if (item != nullptr && !(item->flags() & Qt::ItemIsEnabled))
				{
					m_navList->setCurrentRow(0);
					return;
				}
				m_pagesStack->setCurrentIndex(row);
				if (row == m_remoteNavRow)
				{
					RefreshRemoteDevices();   // 每次进入「远程数据」自动刷新（受限=锁定本机并列出自己的文件）
				}
			});
		m_navList->setCurrentRow(0);

		QHBoxLayout* bodyRow = new QHBoxLayout();
		bodyRow->setSpacing(12);
		bodyRow->addWidget(m_navList);
		bodyRow->addWidget(m_pagesStack, 1);
		mainLayout->addLayout(bodyRow, 1);
	}

	m_logText = new QPlainTextEdit(this);
	m_logText->setReadOnly(true);
	m_logText->setPlaceholderText(QStringLiteral("在线服务日志…"));
	if (!m_aboutMode)
	{
		m_logText->setMaximumHeight(150);   // 仪表盘模式日志固定高度，空间让给页签内容
	}
	mainLayout->addWidget(m_logText, m_aboutMode ? 1 : 0);

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
			ShowPickCasesDialog();
		});
	connect(saveConfigBtn, &QPushButton::clicked, this, [this]()
		{
			SaveConfigFromUi();
			AppendLog(m_remoteBusy
				? QStringLiteral("其它服务器配置已保存；FTP 配置请在远程数据操作完成后再次保存。")
				: QStringLiteral("服务器配置已保存。"));
		});

	resize(m_aboutMode ? 640 : 860, m_aboutMode ? 560 : 780);
}

void OnlineServicesDialog::LoadConfigToUi()
{
	if (m_remoteBrowseAllowed)
	{
		m_updateBaseUrlEdit->setText(OnlineServicesConfig::UpdateBaseUrl());
		m_ftpHostEdit->setText(OnlineServicesConfig::FtpHost());
		m_ftpPortEdit->setText(QString::number(OnlineServicesConfig::FtpPort()));
		m_ftpUserEdit->setText(OnlineServicesConfig::FtpUser());
		m_ftpPasswordEdit->setText(OnlineServicesConfig::FtpPassword());
		m_deviceNameEdit->setText(OnlineServicesConfig::DeviceName());
		if (m_adminTokenEdit != nullptr)
		{
			m_adminTokenEdit->setText(OnlineServicesConfig::AdminToken());
		}
	}
	m_autoUploadCheck->setChecked(OnlineServicesConfig::AutoUploadEnabled());
	if (m_uploader != nullptr && m_pendingListWidget != nullptr)
	{
		m_pendingListWidget->clear();
		m_pendingListWidget->addItems(m_uploader->PendingList());
	}
	UpdateRestrictedNav();
}

bool OnlineServicesDialog::IsUploadOnlyAccount() const
{
	// 现场设备开箱即随包的只上传账号 uploader（服务器端 download_enable=NO，看不到也下不了别人）。
	// 权限判断必须与实际 FTP 请求使用同一份「已保存配置」，不能读取尚未保存的输入框文本。
	const QString user = OnlineServicesConfig::FtpUser().trimmed();
	return user.compare(QStringLiteral("uploader"), Qt::CaseInsensitive) == 0;
}

void OnlineServicesDialog::UpdateRestrictedNav()
{
	const bool restricted = IsUploadOnlyAccount();
	const bool unrestrictedIdle = !m_remoteBusy && !restricted;

	// 「账号管理」：只上传账号整页禁用（无令牌也无权限）。「远程数据」保持可进，
	// 但下方把设备锁定为本机、禁下载/建目录——只能看到自己设备上传的文件。
	auto setNavRowEnabled = [this](int row, bool enabled, const QString& tip)
		{
			if (m_navList == nullptr || row < 0 || row >= m_navList->count())
			{
				return;
			}
			QListWidgetItem* item = m_navList->item(row);
			if (item == nullptr)
			{
				return;
			}
			item->setFlags(enabled ? (item->flags() | Qt::ItemIsEnabled)
				: (item->flags() & ~Qt::ItemIsEnabled));
			item->setToolTip(enabled ? QString() : tip);
			if (!enabled && m_navList->currentRow() == row)
			{
				m_navList->setCurrentRow(0);   // 正停在被禁页 → 弹回总览
			}
		};
	setNavRowEnabled(m_accountNavRow, !restricted,
		QStringLiteral("当前为只上传账号（uploader），无账号管理权限；改用全权限账号后可用。"));
	setNavRowEnabled(m_remoteNavRow, true, QString());

	// 远程数据控件级限制（about 模式/非 admin 登录时这些控件不存在，判空跳过）。
	if (m_remoteDeviceCombo != nullptr)
	{
		m_remoteDeviceCombo->setEnabled(unrestrictedIdle);   // 忙碌或受限时均不可切换设备
		m_remoteDeviceCombo->setToolTip(restricted
			? QStringLiteral("只上传账号仅能查看本机设备的数据。") : QString());
	}
	if (m_remoteMkdirBtn != nullptr)
	{
		m_remoteMkdirBtn->setEnabled(unrestrictedIdle);
		m_remoteMkdirBtn->setToolTip(restricted
			? QStringLiteral("只上传账号不能新建其他设备目录。") : QString());
	}
	if (m_remoteDownloadBtn != nullptr)
	{
		m_remoteDownloadBtn->setEnabled(unrestrictedIdle);
		m_remoteDownloadBtn->setToolTip(restricted
			? QStringLiteral("只上传账号服务器端禁止下载（download_enable=NO），需下载请改用全权限账号。")
			: QStringLiteral("下载并自动解压到 Result\\Remote\\<设备>\\，可用点云查看等工具直接打开。"));
	}
	// 受限且设备列表还不是「只有本机」时重置（清掉之前全权限账号刷出的全部设备）。
	if (restricted && m_remoteDeviceCombo != nullptr)
	{
		const QString selfDevice = OnlineServicesConfig::DeviceName().trimmed();
		if (m_remoteDeviceCombo->count() != 1 || m_remoteDeviceCombo->currentText() != selfDevice)
		{
			RefreshRemoteDevices();
		}
	}
}

void OnlineServicesDialog::SaveConfigFromUi()
{
	if (!m_remoteBrowseAllowed
		|| !m_privilegedActionGuard
		|| !m_privilegedActionGuard())
	{
		if (!m_aboutMode)
		{
			AppendLog(QStringLiteral("升级源、FTP 与管理令牌仅允许有效的本地管理员会话修改，本次未保存。"));
		}
		return;
	}
	OnlineServicesConfig::SetUpdateBaseUrl(m_updateBaseUrlEdit->text().trimmed());
	if (!m_remoteBusy)
	{
		OnlineServicesConfig::SetFtpHost(m_ftpHostEdit->text().trimmed());
		bool ok = false;
		const int port = m_ftpPortEdit->text().trimmed().toInt(&ok);
		OnlineServicesConfig::SetFtpPort(ok ? port : 21);
		OnlineServicesConfig::SetFtpUser(m_ftpUserEdit->text().trimmed());
		OnlineServicesConfig::SetFtpPassword(m_ftpPasswordEdit->text());
		const QString editedDeviceName = m_deviceNameEdit->text().trimmed();
		if (editedDeviceName.isEmpty() || IsSafeRemotePathComponent(editedDeviceName))
		{
			OnlineServicesConfig::SetDeviceName(editedDeviceName);
		}
		else
		{
			AppendLog(QStringLiteral("设备名未保存：只能使用安全的单一目录名，不能含路径字符或保留设备名。"));
		}
	}
	else
	{
		// 远程 FTP 请求仍持有旧配置快照时，不允许把预先编辑但未保存的新值写入配置。
		// 等请求结束后再次保存即可，避免旧回调把另一台服务器/账号的数据落到当前页面。
		bool portOk = false;
		const int editedPort = m_ftpPortEdit->text().trimmed().toInt(&portOk);
		const bool hasPendingFtpChanges =
			m_ftpHostEdit->text().trimmed() != OnlineServicesConfig::FtpHost()
			|| !portOk || editedPort != OnlineServicesConfig::FtpPort()
			|| m_ftpUserEdit->text().trimmed() != OnlineServicesConfig::FtpUser()
			|| m_ftpPasswordEdit->text() != OnlineServicesConfig::FtpPassword()
			|| m_deviceNameEdit->text().trimmed() != OnlineServicesConfig::DeviceName();
		if (hasPendingFtpChanges)
		{
			AppendLog(QStringLiteral("远程数据操作进行中，FTP 配置暂未保存；请在操作完成后重试。"));
		}
	}
	if (m_adminTokenEdit != nullptr)
	{
		OnlineServicesConfig::SetAdminToken(m_adminTokenEdit->text().trimmed());
	}
	UpdateRestrictedNav();   // 账号可能改了，重新评估「远程数据/账号管理」是否可用
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
	if (m_checkingForUpdate)
	{
		AppendLog(QStringLiteral("更新清单正在检查，请勿重复请求。"));
		return;
	}
	if (m_downloading)
	{
		AppendLog(QStringLiteral("更新文件正在下载，完成或失败前禁止刷新清单。"));
		return;
	}
	SaveConfigFromUi();
	const QString url = OnlineServicesConfig::UpdateBaseUrl() + "/" + UpdateChannel() + "/latest.json";
	const QUrl manifestUrl(url);
	// 一旦开始刷新，旧 offer 立即失效。网络失败、服务端撤回或签名失败都不能继续使用
	// 上一次清单留下的下载按钮/载荷。
	m_remoteVersion.clear();
	m_remoteFile.clear();
	m_remoteSha256.clear();
	m_remoteSize = 0;
	m_remotePatchFile.clear();
	m_remotePatchSha256.clear();
	m_remotePatchSize = 0;
	m_usePatch = false;
	if (!m_downloadedPath.isEmpty())
	{
		QFile::remove(m_downloadedPath);
		m_downloadedPath.clear();
	}
	m_downloadInstallBtn->setEnabled(false);
	m_downloadProgress->setValue(0);
	if (!IsAllowedOtaUrl(manifestUrl))
	{
		AppendLog(QStringLiteral("升级源 URL 无效：只允许无内嵌账号的 HTTP/HTTPS 地址。"));
		return;
	}
	AppendLog(QStringLiteral("检查更新：%1").arg(url));
	m_checkingForUpdate = true;
	m_checkUpdateBtn->setEnabled(false);

	QNetworkRequest request{ manifestUrl };
	request.setTransferTimeout(15000);
	request.setAttribute(
		QNetworkRequest::RedirectPolicyAttribute,
		QNetworkRequest::SameOriginRedirectPolicy);
	QNetworkReply* reply = m_network->get(request);
	reply->setReadBufferSize(kMaximumManifestBytes + 1);
	reply->setProperty("otaManifestBytes", QByteArray());
	connect(reply, &QNetworkReply::metaDataChanged, this, [reply]()
		{
			bool contentLengthOk = false;
			const qint64 contentLength =
				reply->header(QNetworkRequest::ContentLengthHeader).toLongLong(&contentLengthOk);
			if (contentLengthOk && contentLength > kMaximumManifestBytes)
			{
				reply->setProperty("otaManifestSizeRejected", true);
				reply->abort();
			}
		});
	connect(reply, &QNetworkReply::readyRead, this, [reply]()
		{
			QByteArray bytes = reply->property("otaManifestBytes").toByteArray();
			const qint64 remaining = kMaximumManifestBytes + 1 - bytes.size();
			if (remaining > 0)
			{
				bytes.append(reply->read(remaining));
			}
			if (bytes.size() > kMaximumManifestBytes || reply->bytesAvailable() > 0)
			{
				reply->setProperty("otaManifestSizeRejected", true);
				reply->abort();
				return;
			}
			reply->setProperty("otaManifestBytes", bytes);
		});
	connect(reply, &QNetworkReply::finished, this, [this, reply]() { OnManifestReply(reply); });
}

void OnlineServicesDialog::OnManifestReply(QNetworkReply* reply)
{
	reply->deleteLater();
	m_checkingForUpdate = false;
	m_checkUpdateBtn->setEnabled(true);
	if (reply->error() != QNetworkReply::NoError)
	{
		m_latestVersionLabel->setText(QStringLiteral("最新版本：检查失败"));
		AppendLog(reply->property("otaManifestSizeRejected").toBool()
			? QStringLiteral("更新清单响应超过 256 KiB，已在接收过程中中止。")
			: QStringLiteral("检查更新失败：%1").arg(reply->errorString()));
		return;
	}

	QByteArray manifestBytes = reply->property("otaManifestBytes").toByteArray();
	manifestBytes.append(reply->read(kMaximumManifestBytes + 1 - manifestBytes.size()));
	if (manifestBytes.isEmpty() || manifestBytes.size() > kMaximumManifestBytes)
	{
		AppendLog(QStringLiteral("更新清单为空或超过 256 KiB，已拒绝。"));
		return;
	}
	QJsonParseError parseError;
	const QJsonDocument document = QJsonDocument::fromJson(manifestBytes, &parseError);
	if (parseError.error != QJsonParseError::NoError || !document.isObject())
	{
		AppendLog(QStringLiteral("更新清单不是有效的 JSON 对象，已拒绝：%1").arg(parseError.errorString()));
		return;
	}
	const QJsonObject obj = document.object();
	if (!IsStrictManifestShape(obj))
	{
		AppendLog(QStringLiteral("更新清单字段、类型或 schema 不严格，已拒绝。"));
		return;
	}
	const QString channel = UpdateChannel();
	if (channel == QStringLiteral("neutral") && obj.contains(QStringLiteral("patch")))
	{
		AppendLog(QStringLiteral("中性通道清单禁止包含 patch，已拒绝。"));
		return;
	}
	const QString remoteVersion = obj.value(QStringLiteral("version")).toString();
	const QString remoteFile = obj.value(QStringLiteral("file")).toString();
	const QString remoteSha256 = obj.value(QStringLiteral("sha256")).toString();
	qint64 remoteSize = 0;
	if (!IsStrictOtaVersion(remoteVersion)
		|| remoteFile != ExpectedInstallerName(channel, remoteVersion)
		|| !IsSafeRemotePathComponent(remoteFile)
		|| !IsStrictSha256(remoteSha256)
		|| !TryReadManifestSize(obj.value(QStringLiteral("size")), &remoteSize))
	{
		AppendLog(QStringLiteral(
			"更新清单格式错误：全量包必须提供严格版本、通道绑定文件名、64 位 SHA256 和正整数 size。"));
		return;
	}
	if (obj.value(QStringLiteral("channel")).toString() != channel
		|| !VerifyOtaManifestSignature(obj))
	{
		AppendLog(QStringLiteral(
			"更新清单通道或 RSA 签名验证失败，可能被篡改或不是正式发布，已拒绝。"));
		return;
	}
	const QString currentVersion = QApplication::applicationVersion().trimmed();
	if (!IsStrictOtaVersion(currentVersion))
	{
		AppendLog(QStringLiteral("本机内嵌版本格式无效，无法安全比较 OTA 版本，已拒绝更新。"));
		return;
	}
	QString highestSeenVersion = OnlineServicesConfig::HighestSeenUpdateVersion(channel);
	if (!IsStrictOtaVersion(highestSeenVersion))
	{
		highestSeenVersion = currentVersion;
	}
	if (CompareVersions(remoteVersion, highestSeenVersion) < 0)
	{
		AppendLog(QStringLiteral(
			"签名清单版本 %1 低于本机已见最高版本 %2，疑似旧清单回放，已拒绝。")
			.arg(remoteVersion, highestSeenVersion));
		return;
	}
	if (CompareVersions(remoteVersion, highestSeenVersion) > 0)
	{
		OnlineServicesConfig::SetHighestSeenUpdateVersion(channel, remoteVersion);
		if (OnlineServicesConfig::HighestSeenUpdateVersion(channel) != remoteVersion)
		{
			AppendLog(QStringLiteral("无法持久化 OTA 防回放版本水位，已拒绝本次更新。"));
			return;
		}
	}

	m_remoteVersion = remoteVersion;
	m_remoteFile = remoteFile;
	m_remoteSha256 = remoteSha256;
	m_remoteSize = remoteSize;
	const QString notes = obj.value(QStringLiteral("notes")).toString();
	if (notes.size() > 4000)
	{
		AppendLog(QStringLiteral("更新清单 notes 超过 4000 字符，已拒绝。"));
		return;
	}

	// 中性通道固定只走全量。品牌补丁是可选优化，但只要任一字段缺失/非法就
	// fail closed 回退全量，绝不沿用历史“缺 baseMinVersion 也直接打补丁”的行为。
	const QJsonValue patchValue = obj.value(QStringLiteral("patch"));
	if (channel == QStringLiteral("brand") && !patchValue.isUndefined() && !patchValue.isNull())
	{
		const QJsonObject patch = patchValue.toObject();
		const QString patchFile = patch.value(QStringLiteral("file")).toString();
		const QString patchSha256 = patch.value(QStringLiteral("sha256")).toString();
		const QString patchBaseMinVersion =
			patch.value(QStringLiteral("baseMinVersion")).toString();
		qint64 patchSize = 0;
		const bool validPatch = patchValue.isObject()
			&& patchFile == ExpectedPatchName(remoteVersion)
			&& IsSafeRemotePathComponent(patchFile)
			&& IsStrictSha256(patchSha256)
			&& TryReadManifestSize(patch.value(QStringLiteral("size")), &patchSize)
			&& IsStrictOtaVersion(patchBaseMinVersion)
			&& CompareVersions(patchBaseMinVersion, remoteVersion) <= 0;
		if (!validPatch)
		{
			AppendLog(QStringLiteral("品牌增量补丁字段不完整或不合法，已安全回退全量安装。"));
		}
		else if (CompareVersions(currentVersion, patchBaseMinVersion) < 0)
		{
			AppendLog(QStringLiteral("本机版本 %1 低于增量补丁基线 %2（DLL 可能不一致），改用全量安装。")
				.arg(currentVersion, patchBaseMinVersion));
		}
		else if (PatchFailedForVersion(remoteVersion))
		{
			AppendLog(QStringLiteral(
				"版本 %1 的增量替换曾在退出后失败，本次强制改用全量安装包。")
				.arg(remoteVersion));
		}
		else
		{
			m_remotePatchFile = patchFile;
			m_remotePatchSha256 = patchSha256;
			m_remotePatchSize = patchSize;
			m_usePatch = true;
		}
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
	const bool usePatch = m_usePatch;
	const QString downloadVersion = m_remoteVersion;
	const QString channel = UpdateChannel();
	const QString fileName = usePatch ? m_remotePatchFile : m_remoteFile;
	const QString expectedSha = usePatch ? m_remotePatchSha256 : m_remoteSha256;
	const qint64 expectedSize = usePatch ? m_remotePatchSize : m_remoteSize;
	if (!IsSafeRemotePathComponent(fileName))
	{
		AppendLog(QStringLiteral("更新文件名不安全，已拒绝下载。"));
		m_downloadInstallBtn->setEnabled(false);
		return;
	}
	if (expectedSize <= 0 || expectedSize > kMaximumUpdatePayloadBytes)
	{
		AppendLog(QStringLiteral("更新清单中的文件大小无效，已拒绝下载。"));
		m_downloadInstallBtn->setEnabled(false);
		return;
	}
	const QString url = OnlineServicesConfig::UpdateBaseUrl() + "/" + channel + "/" + fileName;
	const QUrl payloadUrl(url);
	if (!IsAllowedOtaUrl(payloadUrl))
	{
		AppendLog(QStringLiteral("更新载荷 URL 无效，已拒绝下载。"));
		m_downloadInstallBtn->setEnabled(false);
		return;
	}
	const QString tempDir = DownloadTempDir();
	const QString targetPath = AppPaths::WritableChildPath(QStringLiteral("Temp/OnlineUpdate"), fileName);
	if (targetPath.isEmpty() || !QDir().mkpath(tempDir))
	{
		m_downloadInstallBtn->setEnabled(false);
		AppendLog(QStringLiteral("更新目录或文件名无效，已拒绝下载。"));
		return;
	}
	const QString partPath = targetPath + QStringLiteral(".part");
	QFile::remove(partPath);
	QFile* sink = new QFile(partPath);
	if (!sink->open(QIODevice::WriteOnly | QIODevice::Truncate))
	{
		delete sink;
		m_downloadInstallBtn->setEnabled(true);
		AppendLog(QStringLiteral("无法创建更新临时文件：%1").arg(partPath));
		return;
	}
	sink->setObjectName(QStringLiteral("otaPayloadSink"));
	AppendLog(QStringLiteral("开始下载%1：%2").arg(usePatch ? QStringLiteral("增量补丁") : QStringLiteral("安装包")).arg(url));
	m_downloading = true;
	m_checkUpdateBtn->setEnabled(false);
	m_downloadInstallBtn->setEnabled(false);
	m_downloadProgress->setValue(0);

	QNetworkRequest request{ payloadUrl };
	request.setTransferTimeout(10 * 60 * 1000);  // 大文件：10 分钟超时
	request.setAttribute(
		QNetworkRequest::RedirectPolicyAttribute,
		QNetworkRequest::SameOriginRedirectPolicy);
	QNetworkReply* reply = m_network->get(request);
	sink->setParent(reply);
	connect(sink, &QObject::destroyed, [partPath]()
		{
			// 对话框/QNAM 在下载中被销毁时 finished 可能不再到达；QFile 析构关句柄后
			// 无条件清理 .part。成功路径已 rename，此处删除不存在文件是幂等的。
			QFile::remove(partPath);
		});
	reply->setReadBufferSize(1024 * 1024);
	reply->setProperty("otaExpectedSize", expectedSize);
	reply->setProperty("otaWasPatch", usePatch);
	reply->setProperty("otaVersion", downloadVersion);
	reply->setProperty("otaChannel", channel);
	reply->setProperty("otaFileName", fileName);
	reply->setProperty("otaExpectedSha256", expectedSha);
	reply->setProperty("otaTransferredSize", static_cast<qint64>(0));
	reply->setProperty("otaPartPath", partPath);
	reply->setProperty("otaTargetPath", targetPath);
	connect(reply, &QNetworkReply::metaDataChanged, this, [reply, expectedSize]()
		{
			bool contentLengthOk = false;
			const qint64 contentLength =
				reply->header(QNetworkRequest::ContentLengthHeader).toLongLong(&contentLengthOk);
			if (contentLengthOk && contentLength != expectedSize)
			{
				reply->setProperty("otaSizeRejected", true);
				reply->abort();
			}
		});
	connect(reply, &QNetworkReply::readyRead, this, [reply, sink, expectedSize]()
		{
			const QByteArray chunk = reply->readAll();
			const qint64 transferred = reply->property("otaTransferredSize").toLongLong();
			if (chunk.size() > expectedSize - transferred)
			{
				reply->setProperty("otaSizeRejected", true);
				reply->abort();
				return;
			}
			if (!sink->isOpen() || sink->write(chunk) != chunk.size())
			{
				reply->setProperty("otaWriteRejected", true);
				reply->abort();
				return;
			}
			reply->setProperty("otaTransferredSize", transferred + chunk.size());
		});
	connect(reply, &QNetworkReply::downloadProgress, this, [this, reply, expectedSize](qint64 received, qint64 total)
		{
			if ((total > 0 && total != expectedSize) || received > expectedSize)
			{
				reply->setProperty("otaSizeRejected", true);
				reply->abort();
				return;
			}
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
	m_checkUpdateBtn->setEnabled(true);
	QFile* sink = reply->findChild<QFile*>(QStringLiteral("otaPayloadSink"), Qt::FindDirectChildrenOnly);
	const QString partPath = reply->property("otaPartPath").toString();
	const QString targetPath = reply->property("otaTargetPath").toString();
	const bool wasPatch = reply->property("otaWasPatch").toBool();
	const QString downloadVersion = reply->property("otaVersion").toString();
	const QString downloadChannel = reply->property("otaChannel").toString();
	const QString downloadFileName = reply->property("otaFileName").toString();
	const QString expectedSha = reply->property("otaExpectedSha256").toString();
	qint64 transferred = reply->property("otaTransferredSize").toLongLong();
	bool streamOk = sink != nullptr && sink->isOpen();
	const QByteArray tail = reply->readAll();
	const qint64 expectedSize = reply->property("otaExpectedSize").toLongLong();
	if (streamOk && tail.size() <= expectedSize - transferred)
	{
		streamOk = sink->write(tail) == tail.size();
		transferred += tail.size();
	}
	else if (!tail.isEmpty())
	{
		streamOk = false;
	}
	if (sink != nullptr)
	{
		streamOk = sink->flush() && streamOk;
		sink->close();
	}

	const auto rejectDownload = [this, &partPath, wasPatch, &downloadVersion](const QString& reason)
		{
			QFile::remove(partPath);
			if (wasPatch && m_remoteVersion == downloadVersion)
			{
				FallbackToFullDownload(reason);
			}
			else
			{
				m_downloadInstallBtn->setEnabled(true);
				AppendLog(reason);
			}
		};
	if (reply->error() != QNetworkReply::NoError)
	{
		rejectDownload(reply->property("otaSizeRejected").toBool()
			? QStringLiteral("下载响应大小与签发清单不一致，已中止并丢弃。")
			: (reply->property("otaWriteRejected").toBool()
				? QStringLiteral("下载过程中写入临时文件失败，已丢弃。")
				: QStringLiteral("下载失败：%1").arg(reply->errorString())));
		return;
	}

	bool contentLengthOk = false;
	const qint64 contentLength =
		reply->header(QNetworkRequest::ContentLengthHeader).toLongLong(&contentLengthOk);
	if (!contentLengthOk || contentLength != expectedSize)
	{
		rejectDownload(QStringLiteral("服务器 Content-Length 与签发清单 size 不一致，已拒绝安装。"));
		return;
	}
	if (!streamOk || transferred != expectedSize || QFileInfo(partPath).size() != expectedSize)
	{
		rejectDownload(QStringLiteral("实际落盘字节数与签发清单 size 不一致，已丢弃。"));
		return;
	}

	// SHA256 是清单硬字段；清单解析阶段已强制 64 位小写十六进制。
	if (!IsStrictSha256(expectedSha))
	{
		rejectDownload(QStringLiteral("签发清单 SHA256 非法，已拒绝安装。"));
		return;
	}
	const QString actual = HashFileSha256(partPath);
	if (actual != expectedSha)
	{
		rejectDownload(QStringLiteral("SHA256 校验失败，文件可能损坏或被篡改，已丢弃。"));
		return;
	}
	AppendLog(QStringLiteral("Content-Length、size 与 SHA256 三项校验通过。"));
	if (downloadVersion != m_remoteVersion
		|| downloadChannel != UpdateChannel()
		|| downloadFileName != QFileInfo(targetPath).fileName())
	{
		rejectDownload(QStringLiteral("下载期间更新会话身份发生变化，已丢弃载荷。"));
		return;
	}

	if ((!QFile::remove(targetPath) && QFile::exists(targetPath))
		|| !QFile::rename(partPath, targetPath))
	{
		rejectDownload(QStringLiteral("保存已验证更新文件失败，已停止安装。"));
		return;
	}
	m_downloadedPath = targetPath;

	m_downloadProgress->setValue(100);
	m_downloadInstallBtn->setEnabled(true);
	m_downloadInstallBtn->setText(QStringLiteral("立即安装并重启"));
	AppendLog(QStringLiteral("下载完成：%1（%2 MB）。点击「立即安装并重启」完成升级。")
		.arg(m_downloadedPath).arg(QString::number(expectedSize / 1048576.0, 'f', 1)));
	InstallDownloadedPackage();
}

void OnlineServicesDialog::FallbackToFullDownload(const QString& reason)
{
	if (!m_usePatch)
	{
		m_downloadInstallBtn->setEnabled(true);
		AppendLog(reason);
		return;
	}

	m_usePatch = false;
	m_remotePatchFile.clear();
	m_remotePatchSha256.clear();
	m_remotePatchSize = 0;
	m_downloadedPath.clear();
	m_downloadProgress->setValue(0);
	m_downloadInstallBtn->setEnabled(true);
	m_downloadInstallBtn->setText(QStringLiteral("下载全量安装包"));
	AppendLog(QStringLiteral("%1 已自动切换为签发清单中的全量安装包。").arg(reason));
	QTimer::singleShot(0, this, [this]()
		{
			if (!m_downloading && !m_usePatch && !m_remoteFile.isEmpty())
			{
				StartDownload();
			}
		});
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
	if (m_usePatch && !IsSafeExecutableOnlyPatch(m_downloadedPath))
	{
		const QString rejectedPath = m_downloadedPath;
		m_downloadedPath.clear();
		QFile::remove(rejectedPath);
		QMessageBox::warning(this, QStringLiteral("在线升级"),
			QStringLiteral("增量补丁包含路径、符号链接或额外文件，已拒绝安装；将自动改用全量安装包。"));
		FallbackToFullDownload(
			QStringLiteral("增量补丁内容不安全：仅允许包含当前程序 exe 的单文件 zip，已删除。"));
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
	if (!RobotOperationLease::NewOperationsAllowed())
	{
		QMessageBox::warning(this, QStringLiteral("在线升级"),
			QStringLiteral("当前会话已禁止新的机器人操作，不能进入升级退出阶段。"));
		return;
	}
	struct ScopedNewOperationBlock final
	{
		ScopedNewOperationBlock()
		{
			token = RobotOperationLease::AddNewOperationsBlock(
				QStringLiteral("在线升级即将退出程序，禁止启动新的机器人操作。"));
		}
		~ScopedNewOperationBlock()
		{
			RobotOperationLease::RemoveNewOperationsBlock(token);
		}
		void KeepBlocked() { token = 0; }
		RobotOperationLease::NewOperationBlockToken token = 0;
	} operationBlock;
	if ((m_flowRunningGuard && m_flowRunningGuard()) || RobotOperationLease::AnyActive())
	{
		QMessageBox::warning(this, QStringLiteral("在线升级"),
			QStringLiteral("确认期间有焊接、扫描或机器人操作启动，已取消本次升级。"));
		return;
	}

	const bool usePatch = m_usePatch;
	const qint64 signedPayloadSize = usePatch ? m_remotePatchSize : m_remoteSize;
	const QString signedPayloadSha = usePatch ? m_remotePatchSha256 : m_remoteSha256;
	const QString downloadedPath = QFileInfo(m_downloadedPath).absoluteFilePath();
	const auto rejectChangedPayload = [this, usePatch, &downloadedPath](const QString& reason)
		{
			QFile::remove(downloadedPath);
			m_downloadedPath.clear();
			m_downloadProgress->setValue(0);
			QMessageBox::warning(this, QStringLiteral("在线升级"), reason);
			if (usePatch)
			{
				FallbackToFullDownload(reason);
			}
			else
			{
				m_downloadInstallBtn->setEnabled(true);
				m_downloadInstallBtn->setText(QStringLiteral("重新下载全量安装包"));
				AppendLog(reason);
			}
		};
	if (signedPayloadSize <= 0
		|| !IsStrictSha256(signedPayloadSha)
		|| QFileInfo(downloadedPath).size() != signedPayloadSize
		|| HashFileSha256(downloadedPath) != signedPayloadSha
		|| (usePatch && !IsSafeExecutableOnlyPatch(downloadedPath)))
	{
		rejectChangedPayload(QStringLiteral("确认期间更新文件已损坏或被替换，重新校验失败，已删除。"));
		return;
	}

	QString payloadSourcePath = downloadedPath;
	QString payloadSha = signedPayloadSha;
	qint64 payloadSize = signedPayloadSize;
	if (usePatch)
	{
		const QString expectedExeName = QFileInfo(QCoreApplication::applicationFilePath()).fileName();
		const QString stagedPatchPath = AppPaths::WritableChildPath(
			QStringLiteral("Temp/OnlineUpdate"), expectedExeName + QStringLiteral(".ota-staged"));
		QString stagedSha;
		qint64 stagedSize = 0;
		QFile::remove(stagedPatchPath);
		if (stagedPatchPath.isEmpty()
			|| !ExtractExecutablePatchToStaging(
				downloadedPath, m_remoteVersion, stagedPatchPath, &stagedSha, &stagedSize)
			// 解压前后再次验证 zip；避免确认后的可写 Temp 文件在读取窗口被替换。
			|| QFileInfo(downloadedPath).size() != signedPayloadSize
			|| HashFileSha256(downloadedPath) != signedPayloadSha)
		{
			QFile::remove(stagedPatchPath);
			rejectChangedPayload(QStringLiteral("增量补丁预解压或二次校验失败，已删除并改用全量安装包。"));
			return;
		}
		payloadSourcePath = stagedPatchPath;
		payloadSha = stagedSha;
		payloadSize = stagedSize;
	}

	const QString rawAppPath = QDir::toNativeSeparators(QCoreApplication::applicationFilePath());
	const QString payload = EscapeBatchLiteral(QDir::toNativeSeparators(payloadSourcePath));
	const QString appPath = EscapeBatchLiteral(rawAppPath);
	const QString appDir = EscapeBatchLiteral(
		QDir::toNativeSeparators(QCoreApplication::applicationDirPath()));
	const QString bootstrapFile = AppPaths::WritableChildPath(
		QStringLiteral("Temp/OnlineUpdate"), QStringLiteral("apply_update.cmd"));
	const QString failureLogFile = AppPaths::WritableChildPath(
		QStringLiteral("Temp/OnlineUpdate"), QStringLiteral("install_failed.txt"));
	const QString patchFailureMarkerFile = PatchFailureMarkerPath(m_remoteVersion);
	const QString patchHealthMarkerFile = PatchHealthMarkerPath(m_remoteVersion);
	if (bootstrapFile.isEmpty() || failureLogFile.isEmpty()
		|| patchFailureMarkerFile.isEmpty() || patchHealthMarkerFile.isEmpty())
	{
		AppendLog(QStringLiteral("升级启动脚本路径无效，已停止安装。"));
		return;
	}
	const QString bootstrapPath = QDir::toNativeSeparators(
		QFileInfo(bootstrapFile).absoluteFilePath());
	const QString failureLogPath = EscapeBatchLiteral(QDir::toNativeSeparators(
		QFileInfo(failureLogFile).absoluteFilePath()));
	const QString patchFailureMarkerPath = EscapeBatchLiteral(QDir::toNativeSeparators(
		QFileInfo(patchFailureMarkerFile).absoluteFilePath()));
	const QString patchHealthMarkerPath = EscapeBatchLiteral(QDir::toNativeSeparators(
		QFileInfo(patchHealthMarkerFile).absoluteFilePath()));
	const qint64 currentPid = QCoreApplication::applicationPid();
	const QString waitForCurrentProcess = QString(
		"powershell.exe -NoLogo -NoProfile -NonInteractive -Command \"try { "
		"$process = Get-Process -Id %1 -ErrorAction SilentlyContinue; "
		"if ($null -ne $process) { $process.WaitForExit() }; exit 0 } catch { exit 1 }\"\r\n")
		.arg(currentPid);
	const QString verifyPayload = QString(
		"powershell.exe -NoLogo -NoProfile -NonInteractive -Command \"try { "
		"$item = Get-Item -LiteralPath $env:NO_TEACHING_OTA_PAYLOAD -ErrorAction Stop; "
		"if ($item.Length -ne %1) { exit 2 }; "
		"$actual = (Get-FileHash -LiteralPath $env:NO_TEACHING_OTA_PAYLOAD -Algorithm SHA256 -ErrorAction Stop).Hash.ToLowerInvariant(); "
		"if ($actual -ceq '%2') { exit 0 } else { exit 3 } } catch { exit 4 }\"\r\n")
		.arg(payloadSize)
		.arg(payloadSha);
	QString script;
	if (usePatch)
	{
		// 增量包在退出前已预解压并验证。退出后先复制到安装目录同卷临时名，随后用
		// File.Replace 原子替换并自动备份旧 exe；任何失败都保持/恢复旧 exe，绝不 tar 直覆。
		script = QString(
			"@echo off\r\n"
			"setlocal DisableDelayedExpansion\r\n"
			"set \"NO_TEACHING_OTA_PAYLOAD=%2\"\r\n"
			"set \"OTA_APP=%5\"\r\n"
			"set \"OTA_NEW=%5.ota-new\"\r\n"
			"set \"OTA_BACKUP=%5.ota-backup\"\r\n"
			"set \"OTA_FAILED=%5.ota-failed\"\r\n"
			"set \"OTA_PATCH_FAILURE_MARKER=%7\"\r\n"
			"set \"OTA_HEALTH=%8\"\r\n"
			"%1"
			"if errorlevel 1 goto :wait_failed\r\n"
			"if not errorlevel 0 goto :wait_failed\r\n"
			"%6"
			"if errorlevel 1 goto :verify_failed\r\n"
			"if not errorlevel 0 goto :verify_failed\r\n"
			">\"%OTA_PATCH_FAILURE_MARKER%\" echo patch_in_progress\r\n"
			"del /f /q \"%OTA_NEW%\" \"%OTA_BACKUP%\" \"%OTA_FAILED%\" \"%OTA_HEALTH%\" 2>nul\r\n"
			"copy /b /y \"%NO_TEACHING_OTA_PAYLOAD%\" \"%OTA_NEW%\" >nul\r\n"
			"if errorlevel 1 goto :copy_failed\r\n"
			"if not errorlevel 0 goto :copy_failed\r\n"
			"set \"NO_TEACHING_OTA_PAYLOAD=%OTA_NEW%\"\r\n"
			"%6"
			"if errorlevel 1 goto :copy_verify_failed\r\n"
			"if not errorlevel 0 goto :copy_verify_failed\r\n"
			"powershell.exe -NoLogo -NoProfile -NonInteractive -Command \"try { "
			"[System.IO.File]::Replace($env:OTA_NEW, $env:OTA_APP, $env:OTA_BACKUP, $true); exit 0 } catch { exit 1 }\"\r\n"
			"if errorlevel 1 goto :replace_failed\r\n"
			"if not errorlevel 0 goto :replace_failed\r\n"
			"powershell.exe -NoLogo -NoProfile -NonInteractive -Command \"try { "
			"$process = Start-Process -FilePath $env:OTA_APP -PassThru -ErrorAction Stop; "
			"$deadline = [DateTime]::UtcNow.AddSeconds(30); while ([DateTime]::UtcNow -lt $deadline) { "
			"if ($process.HasExited) { exit 1 }; if (Test-Path -LiteralPath $env:OTA_HEALTH) { exit 0 }; "
			"Start-Sleep -Seconds 1 }; exit 2 } catch { exit 3 }\"\r\n"
			"if errorlevel 1 goto :restart_failed\r\n"
			"if not errorlevel 0 goto :restart_failed\r\n"
			"del /f /q \"%OTA_PATCH_FAILURE_MARKER%\" \"%OTA_HEALTH%\" \"%OTA_BACKUP%\" \"%OTA_FAILED%\" 2>nul\r\n"
			"exit /b 0\r\n"
			":wait_failed\r\n"
			">\"%4\" echo Waiting for the running application failed.\r\n"
			"exit /b 1\r\n"
			":verify_failed\r\n"
			">\"%OTA_PATCH_FAILURE_MARKER%\" echo patch_failed\r\n"
			">\"%4\" echo Staged patch hash or size verification failed.\r\n"
			"start \"\" \"%OTA_APP%\"\r\n"
			"exit /b 2\r\n"
			":copy_failed\r\n"
			">\"%OTA_PATCH_FAILURE_MARKER%\" echo patch_failed\r\n"
			">\"%4\" echo Copying the staged executable failed; original was preserved.\r\n"
			"start \"\" \"%OTA_APP%\"\r\n"
			"exit /b 3\r\n"
			":copy_verify_failed\r\n"
			">\"%OTA_PATCH_FAILURE_MARKER%\" echo patch_failed\r\n"
			">\"%4\" echo Copied executable verification failed; original was preserved.\r\n"
			"del /f /q \"%OTA_NEW%\" 2>nul\r\n"
			"start \"\" \"%OTA_APP%\"\r\n"
			"exit /b 4\r\n"
			":replace_failed\r\n"
			">\"%OTA_PATCH_FAILURE_MARKER%\" echo patch_failed\r\n"
			">\"%4\" echo Atomic executable replacement failed; original was preserved.\r\n"
			"del /f /q \"%OTA_NEW%\" 2>nul\r\n"
			"start \"\" \"%OTA_APP%\"\r\n"
			"exit /b 5\r\n"
			":restart_failed\r\n"
			">\"%OTA_PATCH_FAILURE_MARKER%\" echo patch_failed\r\n"
			"powershell.exe -NoLogo -NoProfile -NonInteractive -Command \"try { "
			"[System.IO.File]::Replace($env:OTA_BACKUP, $env:OTA_APP, $env:OTA_FAILED, $true); exit 0 } catch { exit 1 }\"\r\n"
			"if errorlevel 1 goto :rollback_failed\r\n"
			"if not errorlevel 0 goto :rollback_failed\r\n"
			">\"%4\" echo Updated application start failed; old executable was restored.\r\n"
			"start \"\" \"%OTA_APP%\"\r\n"
			"exit /b 6\r\n"
			":rollback_failed\r\n"
			">\"%OTA_PATCH_FAILURE_MARKER%\" echo patch_failed\r\n"
			">\"%4\" echo Updated application start failed and rollback failed.\r\n"
			"exit /b 7\r\n")
			.arg(waitForCurrentProcess, payload, appDir, failureLogPath, appPath, verifyPayload,
				patchFailureMarkerPath, patchHealthMarkerPath);
	}
	else
	{
		// 全量同样必须先等当前 PID 退出，并检查 Inno 退出码；失败时不拉起旧程序。
		// /DIR 强制装回当前程序目录：Inno 按 AppId 查注册表定位先前安装，手工拷贝部署（无注册表记录）
		// 或历史 AppId 对不上时会落到 DefaultDirName={localappdata}，在别的盘装出第二份（2026-07-10 事故）。
		script = QString(
			"@echo off\r\n"
			"setlocal DisableDelayedExpansion\r\n"
			"set \"NO_TEACHING_OTA_PAYLOAD=%2\"\r\n"
			"set \"OTA_APP=%5\"\r\n"
			"set \"OTA_HEALTH=%8\"\r\n"
			"del /f /q \"%OTA_HEALTH%\" 2>nul\r\n"
			"%1"
			"if errorlevel 1 goto :wait_failed\r\n"
			"if not errorlevel 0 goto :wait_failed\r\n"
			"%6"
			"if errorlevel 1 goto :verify_failed\r\n"
			"if not errorlevel 0 goto :verify_failed\r\n"
			"start \"\" /wait \"%2\" /VERYSILENT /SUPPRESSMSGBOXES /NORESTART /DIR=\"%3\"\r\n"
			"if errorlevel 1 goto :install_failed\r\n"
			"if not errorlevel 0 goto :install_failed\r\n"
			"powershell.exe -NoLogo -NoProfile -NonInteractive -Command \"try { "
			"$process = Start-Process -FilePath $env:OTA_APP -PassThru -ErrorAction Stop; "
			"$deadline = [DateTime]::UtcNow.AddSeconds(30); while ([DateTime]::UtcNow -lt $deadline) { "
			"if ($process.HasExited) { exit 1 }; if (Test-Path -LiteralPath $env:OTA_HEALTH) { exit 0 }; "
			"Start-Sleep -Seconds 1 }; exit 2 } catch { exit 3 }\"\r\n"
			"if errorlevel 1 goto :restart_failed\r\n"
			"if not errorlevel 0 goto :restart_failed\r\n"
			"del /f /q \"%7\" \"%OTA_HEALTH%\" \"%5.ota-backup\" \"%5.ota-failed\" 2>nul\r\n"
			"exit /b 0\r\n"
			":wait_failed\r\n"
			">\"%4\" echo Waiting for the running application failed.\r\n"
			"exit /b 1\r\n"
			":verify_failed\r\n"
			">\"%4\" echo Full installer hash or size verification failed.\r\n"
			"start \"\" \"%5\"\r\n"
			"exit /b 2\r\n"
			":install_failed\r\n"
			">\"%4\" echo Full installer failed.\r\n"
			"exit /b 3\r\n"
			":restart_failed\r\n"
			">\"%7\" echo full_failed\r\n"
			">\"%4\" echo Updated application restart failed.\r\n"
			"exit /b 4\r\n")
			.arg(waitForCurrentProcess, payload, appDir, failureLogPath, appPath, verifyPayload,
				patchFailureMarkerPath, patchHealthMarkerPath);
	}

	QFile bootstrap(bootstrapPath);
	if (!bootstrap.open(QIODevice::WriteOnly | QIODevice::Truncate))
	{
		AppendLog(QStringLiteral("写入升级引导脚本失败，请到 %1 手动处理。").arg(payload));
		return;
	}
	// 批处理按系统 ANSI 代码页写（cmd 默认按 ANSI 解析，路径含中文时 toLatin1 会坏）。
	const QByteArray scriptBytes = script.toLocal8Bit();
	if (bootstrap.write(scriptBytes) != scriptBytes.size() || !bootstrap.flush())
	{
		bootstrap.close();
		QFile::remove(bootstrapPath);
		AppendLog(QStringLiteral("升级引导脚本未完整落盘，已停止安装。"));
		return;
	}
	bootstrap.close();

	// 记下目标版本：重启后主窗口自检「实际版本 == 目标?」，不符即告警（补丁/安装没生效）。
	OnlineServicesConfig::SetPendingUpdateTargetVersion(m_remoteVersion);

	AppendLog(QStringLiteral("开始升级：%1").arg(payload));
	QProcess launcher;
	QProcessEnvironment environment = QProcessEnvironment::systemEnvironment();
	environment.remove(QStringLiteral("ERRORLEVEL"));
	environment.insert(QStringLiteral("NO_TEACHING_OTA_BOOTSTRAP"), bootstrapPath);
	// 原进程可能由 --data-root 启动；updater 重启时没有原 argv，因此把已经解析、规范化的
	// 当前真源显式传给子进程环境，保证 health marker、配置和现场数据仍落在同一目录。
	environment.insert(QStringLiteral("QTWIDGETSAPP4_DATA_ROOT"), AppPaths::DataRootPath());
	launcher.setProcessEnvironment(environment);
	launcher.setProgram(QStringLiteral("cmd.exe"));
#ifdef Q_OS_WIN
	// cmd 不把反斜杠当作引号转义；QProcess 的普通参数拼接会把 /C 后的命令错误地
	// 变成 \"path\"。Windows 原生命令行必须保持 cmd 要求的双层引号形式。
	launcher.setNativeArguments(
		QStringLiteral("/D /V:OFF /S /C \"\"%NO_TEACHING_OTA_BOOTSTRAP%\"\""));
#else
	launcher.setArguments({ QStringLiteral("/D"), QStringLiteral("/V:OFF"),
		QStringLiteral("/S"), QStringLiteral("/C"),
		QStringLiteral("\"%NO_TEACHING_OTA_BOOTSTRAP%\"") });
#endif
	if (!launcher.startDetached())
	{
		OnlineServicesConfig::SetPendingUpdateTargetVersion(QString());  // 没启动成功，撤销记录
		AppendLog(QStringLiteral("启动升级脚本失败，请到 %1 手动处理。").arg(payload));
		return;
	}
	operationBlock.KeepBlocked();
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
	// 关界面时若正在上传：问用户后台继续还是停止当前传输。
	if (m_uploader != nullptr && m_uploader->IsBusy())
	{
		const QMessageBox::StandardButton ret = QMessageBox::question(
			this,
			QStringLiteral("上传进行中"),
			QStringLiteral("扫描数据还在上传。\n\n「后台继续」：关闭本界面，上传转后台继续。\n"
				"「停止上传」：中止当前传输；不完整唯一件不会显示，并由服务器定期清理。"),
			QMessageBox::Yes | QMessageBox::No,
			QMessageBox::Yes);
		if (ret == QMessageBox::No)
		{
			m_uploader->RequestCancel();  // 后台线程块间中止；未完整案例留队列
			AppendLog(QStringLiteral("已请求停止上传；不完整唯一件保持隐藏并由服务器定期清理。"));
		}
		// 无论后台继续还是停止，都放行关闭（停止是异步清理，不阻塞关窗）。
	}
	QDialog::closeEvent(event);
}

int OnlineServicesDialog::CompareVersions(const QString& lhs, const QString& rhs)
{
	// 版本必须是经过严格日期/HHMM 校验的四段格式；非法值一律不可用于升级判定。
	if (!IsStrictOtaVersion(lhs) || !IsStrictOtaVersion(rhs))
	{
		return 0;
	}
	const QStringList a = lhs.split('.');
	const QStringList b = rhs.split('.');
	for (int i = 0; i < 4; ++i)
	{
		const int va = a.at(i).toInt();
		const int vb = b.at(i).toInt();
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

bool OnlineServicesDialog::AuthorizePrivilegedAction(const QString& actionName)
{
	if (!m_remoteBrowseAllowed
		|| !m_privilegedActionGuard
		|| !m_privilegedActionGuard())
	{
		const QString message = QStringLiteral("本地管理员会话已失效，已拒绝%1。请重新登录。").arg(actionName);
		AppendLog(message);
		QMessageBox::warning(this, QStringLiteral("权限已失效"), message);
		return false;
	}
	return true;
}

void OnlineServicesDialog::SetRemoteBusy(bool busy)
{
	m_remoteBusy = busy;
	if (m_remoteRefreshBtn != nullptr)
	{
		m_remoteRefreshBtn->setEnabled(!busy);
	}
	if (m_remoteDeleteBtn != nullptr)
	{
		m_remoteDeleteBtn->setEnabled(!busy);
	}
	// FTP 请求使用保存时的配置快照。请求期间锁住这些输入，避免旧请求完成后把
	// 另一套账号/主机的结果落到当前界面；其它 OTA/管理配置不受影响。
	for (QLineEdit* edit : { m_ftpHostEdit, m_ftpPortEdit, m_ftpUserEdit, m_ftpPasswordEdit, m_deviceNameEdit })
	{
		if (edit != nullptr)
		{
			edit->setEnabled(!busy);
		}
	}
	// 设备选择/下载/建目录同时受「后台忙碌」和账号权限约束；统一重算，
	// 避免异步操作结束时 setEnabled(true) 把 uploader 的限制意外撤销。
	UpdateRestrictedNav();
}

void OnlineServicesDialog::RefreshRemoteDevices()
{
	if (!AuthorizePrivilegedAction(QStringLiteral("刷新远程设备")))
	{
		return;
	}
	if (m_remoteBusy)
	{
		return;
	}
	// 只上传账号：不列服务器全部设备目录，设备锁定为本机设备名（只看自己传的数据）。
	if (IsUploadOnlyAccount())
	{
		if (m_remoteDeviceCombo == nullptr)
		{
			return;
		}
		const QString selfDevice = OnlineServicesConfig::DeviceName().trimmed();
		m_remoteDeviceCombo->clear();
		if (!IsSafeRemotePathComponent(selfDevice))
		{
			AppendLog(QStringLiteral("请先填写安全的设备名（单一目录名，不能含路径字符）。"));
			return;
		}
		m_remoteDeviceCombo->addItem(selfDevice);   // addItem 触发 currentIndexChanged → 自动刷新文件列表
		AppendLog(QStringLiteral("只上传账号：仅显示本机设备「%1」的数据。").arg(selfDevice));
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
			RobotLog log(OnlineServicesLogPath(), false);
			FtpClient ftp(&log, cfg.host, cfg.port, cfg.user, cfg.password);
			ftp.setMessageBoxesEnabled(false);
			std::vector<FtpRemoteFileInfo> entries;
			const bool ok = ftp.listFiles("/data", entries);
			QStringList devices;
			for (const FtpRemoteFileInfo& e : entries)
			{
				const QString deviceName = QString::fromStdString(e.name);
				if (e.isDirectory && IsSafeRemotePathComponent(deviceName))
				{
					devices << deviceName;
				}
			}
			QMetaObject::invokeMethod(qApp, [self, ok, devices]()
				{
					if (self == nullptr)
					{
						return;
					}
					if (!ok)
					{
						self->SetRemoteBusy(false);
						self->AppendLog(QStringLiteral("获取设备列表失败（检查 FTP 配置与网络）。"));
						return;
					}
					{
						// 先静默落地最终列表，再解除 busy 并刷新一次文件；避免 clear/addItems
						// 中间信号触发嵌套请求，也确保 uploader 最终只能看到本机设备。
						const QSignalBlocker blocker(self->m_remoteDeviceCombo);
						self->m_remoteDeviceCombo->clear();
						if (self->IsUploadOnlyAccount())
						{
							const QString selfDevice = OnlineServicesConfig::DeviceName().trimmed();
							if (IsSafeRemotePathComponent(selfDevice))
							{
								self->m_remoteDeviceCombo->addItem(selfDevice);
							}
						}
						else
						{
							self->m_remoteDeviceCombo->addItems(devices);
						}
					}
					self->SetRemoteBusy(false);
					self->AppendLog(QStringLiteral("设备列表已刷新：共 %1 台。")
						.arg(self->m_remoteDeviceCombo->count()));
					self->RefreshRemoteFiles();
				}, Qt::QueuedConnection);
		}).detach();
}

void OnlineServicesDialog::RefreshRemoteFiles()
{
	if (!AuthorizePrivilegedAction(QStringLiteral("刷新远程文件")))
	{
		return;
	}
	if (m_remoteFileList == nullptr || m_remoteDeviceCombo == nullptr)
	{
		return;
	}
	m_remoteFileList->clear();
	const QString device = m_remoteDeviceCombo->currentText().trimmed();
	if (m_remoteBusy)
	{
		return;
	}
	if (!IsSafeRemotePathComponent(device))
	{
		AppendLog(QStringLiteral("远程设备名不安全，已拒绝访问。"));
		return;
	}
	if (IsUploadOnlyAccount()
		&& device.compare(OnlineServicesConfig::DeviceName().trimmed(), Qt::CaseSensitive) != 0)
	{
		AppendLog(QStringLiteral("只上传账号检测到非本机设备，已重置远程数据列表。"));
		RefreshRemoteDevices();
		return;
	}
	const RemoteFtpConfig cfg = CurrentRemoteFtpConfig();
	SetRemoteBusy(true);
	QPointer<OnlineServicesDialog> self(this);
	std::thread([self, cfg, device]()
		{
			RobotLog log(OnlineServicesLogPath(), false);
			FtpClient ftp(&log, cfg.host, cfg.port, cfg.user, cfg.password);
			ftp.setMessageBoxesEnabled(false);
			std::vector<FtpRemoteFileInfo> entries;
			const std::string remoteDir = "/data/" + device.toStdString();
			const bool ok = ftp.listFiles(remoteDir, entries);
			QStringList names;
			QStringList labels;
			for (const FtpRemoteFileInfo& e : entries)
			{
				const QString fileName = QString::fromStdString(e.name);
				if (!e.isDirectory
					&& IsSafeRemotePathComponent(fileName)
					&& IsCompleteRemoteArchive(fileName, static_cast<qulonglong>(e.size)))
				{
					names << fileName;
					labels << QStringLiteral("%1（%2 MB）")
						.arg(fileName)
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
	if (!AuthorizePrivilegedAction(QStringLiteral("下载远程数据")))
	{
		return;
	}
	if (m_remoteBusy || m_remoteFileList == nullptr || m_remoteDeviceCombo == nullptr)
	{
		return;
	}
	if (IsUploadOnlyAccount())
	{
		AppendLog(QStringLiteral("当前为只上传账号，服务器禁止下载；请改用全权限账号。"));
		return;
	}
	const QString device = m_remoteDeviceCombo->currentText().trimmed();
	const QList<QListWidgetItem*> selected = m_remoteFileList->selectedItems();
	if (!IsSafeRemotePathComponent(device) || selected.isEmpty())
	{
		AppendLog(QStringLiteral("请选择安全的设备名和数据包。"));
		return;
	}
	QStringList names;
	for (const QListWidgetItem* item : selected)
	{
		const QString name = item->data(Qt::UserRole).toString();
		if (!IsSafeRemotePathComponent(name))
		{
			AppendLog(QStringLiteral("远程文件名不安全，已拒绝下载：%1").arg(name));
			return;
		}
		names << name;
	}
	const RemoteFtpConfig cfg = CurrentRemoteFtpConfig();
	const QString localDir = AppPaths::WritableChildPath(QStringLiteral("Result/Remote"), device);
	if (localDir.isEmpty() || !QDir().mkpath(localDir))
	{
		AppendLog(QStringLiteral("远程数据本地目录无效，已拒绝下载。"));
		return;
	}
	SetRemoteBusy(true);
	AppendLog(QStringLiteral("开始下载 %1 个数据包到 %2 …").arg(names.size()).arg(QDir::toNativeSeparators(localDir)));
	QPointer<OnlineServicesDialog> self(this);
	std::thread([self, cfg, device, names, localDir]()
		{
			RobotLog log(OnlineServicesLogPath(), false);
			FtpClient ftp(&log, cfg.host, cfg.port, cfg.user, cfg.password);
			ftp.setMessageBoxesEnabled(false);
			int okCount = 0;
			for (const QString& name : names)
			{
				const std::string remotePath = "/data/" + device.toStdString() + "/" + name.toStdString();
				const QString localZip = AppPaths::WritableChildPath(
					QStringLiteral("Result/Remote/%1").arg(device), name);
				const QByteArray localZipBytes = QDir::toNativeSeparators(localZip).toLocal8Bit();
				const bool transferSucceeded = !localZip.isEmpty()
					&& ftp.downloadFile(remotePath, localZipBytes.toStdString());
				const bool downloaded = transferSucceeded
					&& IsCompleteRemoteArchive(name, static_cast<qulonglong>(QFileInfo(localZip).size()));
				if (transferSucceeded && !downloaded)
				{
					QFile::remove(localZip);
				}
				bool extracted = false;
				if (downloaded)
				{
					// zip 内已带 <机器人>/<案例>/ 前缀，解到设备目录即可分层落地。
					QZipReader zr(localZip);
					const bool safeArchive = HasOnlySafeArchiveEntries(zr);
					extracted = safeArchive && zr.extractAll(localDir);
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

void OnlineServicesDialog::DeleteSelectedRemoteFiles()
{
	if (!AuthorizePrivilegedAction(QStringLiteral("删除远程数据")))
	{
		return;
	}
	if (m_remoteBusy || m_remoteFileList == nullptr || m_remoteDeviceCombo == nullptr)
	{
		return;
	}
	const QString device = m_remoteDeviceCombo->currentText().trimmed();
	if (IsUploadOnlyAccount()
		&& device.compare(OnlineServicesConfig::DeviceName().trimmed(), Qt::CaseSensitive) != 0)
	{
		AppendLog(QStringLiteral("只上传账号不能删除其它设备的数据，已重置为本机设备。"));
		RefreshRemoteDevices();
		return;
	}
	const QList<QListWidgetItem*> selected = m_remoteFileList->selectedItems();
	if (!IsSafeRemotePathComponent(device) || selected.isEmpty())
	{
		AppendLog(QStringLiteral("请选择安全的设备名和要删除的数据包。"));
		return;
	}
	const QMessageBox::StandardButton ret = QMessageBox::warning(this, QStringLiteral("删除服务器数据"),
		QStringLiteral("将从服务器永久删除设备 %1 的 %2 个数据包（不影响设备本地数据）。\n\n确定删除？")
			.arg(device).arg(selected.size()),
		QMessageBox::Ok | QMessageBox::Cancel, QMessageBox::Cancel);
	if (ret != QMessageBox::Ok)
	{
		return;
	}
	if (!AuthorizePrivilegedAction(QStringLiteral("删除远程数据")))
	{
		return;
	}
	QStringList names;
	for (const QListWidgetItem* item : selected)
	{
		const QString name = item->data(Qt::UserRole).toString();
		if (!IsSafeRemotePathComponent(name))
		{
			AppendLog(QStringLiteral("远程文件名不安全，已拒绝删除：%1").arg(name));
			return;
		}
		names << name;
	}
	const RemoteFtpConfig cfg = CurrentRemoteFtpConfig();
	SetRemoteBusy(true);
	QPointer<OnlineServicesDialog> self(this);
	std::thread([self, cfg, device, names]()
		{
			RobotLog log(OnlineServicesLogPath(), false);
			FtpClient ftp(&log, cfg.host, cfg.port, cfg.user, cfg.password);
			ftp.setMessageBoxesEnabled(false);
			int okCount = 0;
			for (const QString& name : names)
			{
				const std::string remotePath = "/data/" + device.toStdString() + "/" + name.toStdString();
				if (ftp.deleteFile(remotePath, /*askConfirm=*/false))
				{
					++okCount;
				}
			}
			QMetaObject::invokeMethod(qApp, [self, okCount, names]()
				{
					if (self == nullptr)
					{
						return;
					}
					self->SetRemoteBusy(false);
					self->AppendLog(QStringLiteral("服务器数据删除完成：成功 %1/%2。").arg(okCount).arg(names.size()));
					self->RefreshRemoteFiles();
				}, Qt::QueuedConnection);
		}).detach();
}

void OnlineServicesDialog::CreateRemoteDeviceDir()
{
	if (!AuthorizePrivilegedAction(QStringLiteral("新建远程设备目录")))
	{
		return;
	}
	if (m_remoteBusy)
	{
		return;
	}
	if (IsUploadOnlyAccount())
	{
		AppendLog(QStringLiteral("当前为只上传账号，不能新建设备目录；请改用全权限账号。"));
		return;
	}
	bool okInput = false;
	const QString name = QInputDialog::getText(this, QStringLiteral("新建设备目录"),
		QStringLiteral("目录名（建在服务器 /data/ 下，支持中文，不能含斜杠）："),
		QLineEdit::Normal, QString(), &okInput).trimmed();
	if (!okInput || name.isEmpty())
	{
		return;
	}
	if (!IsSafeRemotePathComponent(name))
	{
		AppendLog(QStringLiteral("目录名必须是安全的单一名称，不能含路径字符或保留设备名。"));
		return;
	}
	if (!AuthorizePrivilegedAction(QStringLiteral("新建远程设备目录")))
	{
		return;
	}
	const RemoteFtpConfig cfg = CurrentRemoteFtpConfig();
	SetRemoteBusy(true);
	QPointer<OnlineServicesDialog> self(this);
	std::thread([self, cfg, name]()
		{
			RobotLog log(OnlineServicesLogPath(), false);
			FtpClient ftp(&log, cfg.host, cfg.port, cfg.user, cfg.password);
			ftp.setMessageBoxesEnabled(false);
			const bool ok = ftp.createRemoteDirRecursive("/data/" + name.toStdString());
			QMetaObject::invokeMethod(qApp, [self, ok, name]()
				{
					if (self == nullptr)
					{
						return;
					}
					self->SetRemoteBusy(false);
					self->AppendLog(ok ? QStringLiteral("目录已创建：/data/%1").arg(name)
						: QStringLiteral("创建目录失败：/data/%1").arg(name));
					if (ok)
					{
						self->RefreshRemoteDevices();
					}
				}, Qt::QueuedConnection);
		}).detach();
}

// ============ 服务器管理接口（统计/账号）：nginx 8090 的 /admin/ 反代，X-Admin-Token 鉴权 ============

QString OnlineServicesDialog::AdminApiBase() const
{
	// 从升级源地址推导：http://<host>:8090/ota → http://<host>:8090/admin/api（同端口经 nginx 反代，不开新端口）。
	QString base = OnlineServicesConfig::UpdateBaseUrl().trimmed();
	while (base.endsWith(QLatin1Char('/')))
	{
		base.chop(1);
	}
	if (base.endsWith(QStringLiteral("/ota")))
	{
		base.chop(4);
	}
	return base + QStringLiteral("/admin/api");
}

void OnlineServicesDialog::AdminRequest(const QByteArray& verb, const QString& path,
	const QJsonObject& body, std::function<void(bool ok, const QJsonObject& resp)> done)
{
	if (!AuthorizePrivilegedAction(QStringLiteral("执行服务器管理请求")))
	{
		if (done)
		{
			done(false, QJsonObject());
		}
		return;
	}
	const QUrl adminUrl(AdminApiBase() + path);
	const QString scheme = adminUrl.scheme().toLower();
	const QString host = adminUrl.host().toLower();
	const QHostAddress address(host);
	const bool isLoopbackHost = host == QStringLiteral("localhost")
		|| (!address.isNull() && address.isLoopback());
	if (!adminUrl.isValid() || (scheme != QStringLiteral("https")
		&& !(scheme == QStringLiteral("http") && isLoopbackHost)))
	{
		AppendLog(QStringLiteral(
			"管理接口已拒绝：令牌和账号变更只能通过 HTTPS 或本机 SSH 隧道发送，禁止公网明文 HTTP。"));
		if (done)
		{
			done(false, QJsonObject());
		}
		return;
	}
	const QString token = OnlineServicesConfig::AdminToken().trimmed();
	if (token.isEmpty())
	{
		AppendLog(QStringLiteral("未配置管理令牌：请在「服务器配置」页签填写管理令牌并保存。"));
		if (done)
		{
			done(false, QJsonObject());
		}
		return;
	}
	QNetworkRequest request{ adminUrl };
	request.setTransferTimeout(20000);
	request.setAttribute(QNetworkRequest::RedirectPolicyAttribute, QNetworkRequest::SameOriginRedirectPolicy);
	request.setRawHeader("X-Admin-Token", token.toUtf8());
	request.setHeader(QNetworkRequest::ContentTypeHeader, QStringLiteral("application/json"));
	const QByteArray payload = body.isEmpty() ? QByteArray() : QJsonDocument(body).toJson(QJsonDocument::Compact);
	QNetworkReply* reply = m_network->sendCustomRequest(request, verb, payload);
	connect(reply, &QNetworkReply::finished, this, [this, reply, done]()
		{
			reply->deleteLater();
			const QJsonObject obj = QJsonDocument::fromJson(reply->readAll()).object();
			const bool ok = reply->error() == QNetworkReply::NoError && obj.value(QStringLiteral("ok")).toBool();
			if (!ok)
			{
				const QString err = obj.value(QStringLiteral("error")).toString();
				AppendLog(QStringLiteral("管理接口请求失败：%1").arg(err.isEmpty() ? reply->errorString() : err));
			}
			if (done)
			{
				done(ok, obj);
			}
		});
}

void OnlineServicesDialog::RefreshServerStats()
{
	if (m_cardDisk == nullptr || m_cardCloud == nullptr)
	{
		return;
	}
	AdminRequest("GET", QStringLiteral("/stats"), QJsonObject(), [this](bool ok, const QJsonObject& resp)
		{
			if (!ok)
			{
				m_cardDisk->setText(QStringLiteral("--"));
				if (m_cardDiskSub != nullptr)
				{
					m_cardDiskSub->setText(QStringLiteral("获取失败（查管理令牌/网络）"));
				}
				m_cardCloud->setText(QStringLiteral("--"));
				if (m_cardDevices != nullptr)
				{
					m_cardDevices->setText(QStringLiteral("--"));
				}
				return;
			}
			const QJsonObject disk = resp.value(QStringLiteral("disk")).toObject();
			const double total = disk.value(QStringLiteral("totalBytes")).toDouble();
			const double used = disk.value(QStringLiteral("usedBytes")).toDouble();
			const int percent = total > 0.0 ? static_cast<int>(used * 100.0 / total + 0.5) : 0;
			m_cardDisk->setText(QStringLiteral("%1%").arg(percent));
			if (m_cardDiskSub != nullptr)
			{
				m_cardDiskSub->setText(QStringLiteral("已用 %1 / %2 · 剩 %3")
					.arg(HumanBytes(used), HumanBytes(total),
						HumanBytes(disk.value(QStringLiteral("freeBytes")).toDouble())));
			}
			if (m_diskBar != nullptr)
			{
				m_diskBar->setValue(percent);
			}
			m_cardCloud->setText(HumanBytes(resp.value(QStringLiteral("dataBytes")).toDouble()));
			const QJsonArray devices = resp.value(QStringLiteral("devices")).toArray();
			if (m_cardDevices != nullptr)
			{
				m_cardDevices->setText(QStringLiteral("%1 台").arg(devices.size()));
			}
			if (m_deviceTable != nullptr)
			{
				m_deviceTable->setRowCount(0);
				for (const QJsonValue& value : devices)
				{
					const QJsonObject device = value.toObject();
					const int row = m_deviceTable->rowCount();
					m_deviceTable->insertRow(row);
					m_deviceTable->setItem(row, 0, new QTableWidgetItem(device.value(QStringLiteral("name")).toString()));
					m_deviceTable->setItem(row, 1, new QTableWidgetItem(HumanBytes(device.value(QStringLiteral("bytes")).toDouble())));
					m_deviceTable->setItem(row, 2, new QTableWidgetItem(QString::number(device.value(QStringLiteral("files")).toInt())));
					const qint64 lastEpoch = static_cast<qint64>(device.value(QStringLiteral("lastUploadEpoch")).toDouble());
					m_deviceTable->setItem(row, 3, new QTableWidgetItem(lastEpoch > 0
						? QDateTime::fromSecsSinceEpoch(lastEpoch).toString(QStringLiteral("yyyy-MM-dd HH:mm"))
						: QStringLiteral("—")));
				}
			}
		});
}

void OnlineServicesDialog::UpdateQueueCard()
{
	if (m_cardQueue == nullptr)
	{
		return;
	}
	const int pending = m_uploader != nullptr ? m_uploader->PendingList().size() : 0;
	const bool busy = m_uploader != nullptr && m_uploader->IsBusy();
	m_cardQueue->setText(QStringLiteral("%1 项%2")
		.arg(pending)
		.arg(busy ? QStringLiteral(" ↑") : QString()));
}

void OnlineServicesDialog::RefreshAccounts()
{
	if (m_accountTable == nullptr)
	{
		return;
	}
	AdminRequest("GET", QStringLiteral("/accounts"), QJsonObject(), [this](bool ok, const QJsonObject& resp)
		{
			if (!ok || m_accountTable == nullptr)
			{
				return;
			}
			const QJsonArray accounts = resp.value(QStringLiteral("accounts")).toArray();
			m_accountTable->setRowCount(0);
			for (const QJsonValue& value : accounts)
			{
				const QJsonObject account = value.toObject();
				const QString name = account.value(QStringLiteral("name")).toString();
				const QString perm = account.value(QStringLiteral("permission")).toString();
				const bool isProtected = account.value(QStringLiteral("protected")).toBool();
				const int row = m_accountTable->rowCount();
				m_accountTable->insertRow(row);
				QTableWidgetItem* nameItem = new QTableWidgetItem(name);
				nameItem->setData(Qt::UserRole, perm);
				m_accountTable->setItem(row, 0, nameItem);
				m_accountTable->setItem(row, 1, new QTableWidgetItem(
					perm == QStringLiteral("upload") ? QStringLiteral("仅上传") : QStringLiteral("全权限")));
				m_accountTable->setItem(row, 2, new QTableWidgetItem(
					isProtected ? QStringLiteral("系统保护（不可删）") : QString()));
			}
			AppendLog(QStringLiteral("账号列表已刷新：共 %1 个。").arg(accounts.size()));
		});
}

void OnlineServicesDialog::ShowAddAccountDialog()
{
	QDialog dlg(this);
	dlg.setWindowTitle(QStringLiteral("添加 FTP 账号"));
	QFormLayout* form = new QFormLayout(&dlg);
	QLineEdit* nameEdit = new QLineEdit(&dlg);
	nameEdit->setPlaceholderText(QStringLiteral("小写字母开头，3-32 位（小写字母/数字/_-）"));
	QLineEdit* pwEdit = new QLineEdit(&dlg);
	pwEdit->setEchoMode(QLineEdit::Password);
	pwEdit->setPlaceholderText(QStringLiteral("至少 8 位"));
	QComboBox* permCombo = new QComboBox(&dlg);
	permCombo->addItem(QStringLiteral("仅上传（现场设备用）"), QStringLiteral("upload"));
	permCombo->addItem(QStringLiteral("全权限（可下载浏览全部设备）"), QStringLiteral("full"));
	form->addRow(QStringLiteral("账号名："), nameEdit);
	form->addRow(QStringLiteral("密码："), pwEdit);
	form->addRow(QStringLiteral("权限："), permCombo);
	QDialogButtonBox* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dlg);
	form->addRow(buttons);
	connect(buttons, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
	connect(buttons, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
	if (dlg.exec() != QDialog::Accepted)
	{
		return;
	}
	QJsonObject body;
	body.insert(QStringLiteral("name"), nameEdit->text().trimmed());
	body.insert(QStringLiteral("password"), pwEdit->text());
	body.insert(QStringLiteral("permission"), permCombo->currentData().toString());
	AdminRequest("POST", QStringLiteral("/accounts"), body, [this](bool ok, const QJsonObject&)
		{
			if (ok)
			{
				AppendLog(QStringLiteral("账号已创建。"));
				RefreshAccounts();
			}
		});
}

void OnlineServicesDialog::ChangeSelectedAccountPassword()
{
	if (m_accountTable == nullptr || m_accountTable->currentRow() < 0)
	{
		AppendLog(QStringLiteral("请先在账号表中选中一个账号。"));
		return;
	}
	const QString name = m_accountTable->item(m_accountTable->currentRow(), 0)->text();
	if (name == QStringLiteral("uploader"))
	{
		const QMessageBox::StandardButton ret = QMessageBox::warning(this, QStringLiteral("改密码"),
			QStringLiteral("uploader 是随安装包分发的默认上传账号，改密码会让所有未手动配置账号的现场设备断传。\n\n确定要改？"),
			QMessageBox::Ok | QMessageBox::Cancel, QMessageBox::Cancel);
		if (ret != QMessageBox::Ok)
		{
			return;
		}
	}
	bool okInput = false;
	const QString pw = QInputDialog::getText(this, QStringLiteral("改密码"),
		QStringLiteral("账号 %1 的新密码（至少 8 位）：").arg(name), QLineEdit::Password, QString(), &okInput);
	if (!okInput || pw.isEmpty())
	{
		return;
	}
	QJsonObject body;
	body.insert(QStringLiteral("password"), pw);
	AdminRequest("PATCH", QStringLiteral("/accounts/") + name, body, [this, name](bool ok, const QJsonObject&)
		{
			if (ok)
			{
				AppendLog(QStringLiteral("账号 %1 密码已修改。").arg(name));
			}
		});
}

void OnlineServicesDialog::ToggleSelectedAccountPermission()
{
	if (m_accountTable == nullptr || m_accountTable->currentRow() < 0)
	{
		AppendLog(QStringLiteral("请先在账号表中选中一个账号。"));
		return;
	}
	QTableWidgetItem* nameItem = m_accountTable->item(m_accountTable->currentRow(), 0);
	const QString name = nameItem->text();
	if (name == QStringLiteral("uploader") || name == QStringLiteral("devicedata"))
	{
		AppendLog(QStringLiteral("系统账号权限固定：uploader 永远仅上传，devicedata 永远全权限。"));
		return;
	}
	const QString curPerm = nameItem->data(Qt::UserRole).toString();
	const QString newPerm = curPerm == QStringLiteral("upload") ? QStringLiteral("full") : QStringLiteral("upload");
	QJsonObject body;
	body.insert(QStringLiteral("permission"), newPerm);
	AdminRequest("PATCH", QStringLiteral("/accounts/") + name, body, [this, name, newPerm](bool ok, const QJsonObject&)
		{
			if (ok)
			{
				AppendLog(QStringLiteral("账号 %1 权限已切换为%2。").arg(name)
					.arg(newPerm == QStringLiteral("upload") ? QStringLiteral("「仅上传」") : QStringLiteral("「全权限」")));
				RefreshAccounts();
			}
		});
}

void OnlineServicesDialog::DeleteSelectedAccount()
{
	if (m_accountTable == nullptr || m_accountTable->currentRow() < 0)
	{
		AppendLog(QStringLiteral("请先在账号表中选中一个账号。"));
		return;
	}
	const QString name = m_accountTable->item(m_accountTable->currentRow(), 0)->text();
	const QMessageBox::StandardButton ret = QMessageBox::warning(this, QStringLiteral("删除账号"),
		QStringLiteral("将删除服务器 FTP 账号 %1（其已上传的数据保留）。\n\n确定删除？").arg(name),
		QMessageBox::Ok | QMessageBox::Cancel, QMessageBox::Cancel);
	if (ret != QMessageBox::Ok)
	{
		return;
	}
	AdminRequest("DELETE", QStringLiteral("/accounts/") + name, QJsonObject(), [this, name](bool ok, const QJsonObject&)
		{
			if (ok)
			{
				AppendLog(QStringLiteral("账号 %1 已删除。").arg(name));
				RefreshAccounts();
			}
		});
}

void OnlineServicesDialog::ShowPickCasesDialog()
{
	// 列出 Result/<机器人>/<案例> 全部案例目录（跳过 Remote 远程下载区/Archives 打包区），
	// 支持 Ctrl/Shift 多选；确定后按名字升序依次入队（上传器串行处理=依次打包上传）。
	QList<QPair<QString, QString>> cases;   // <显示名 Robot / case, 绝对路径>
	const QDir resultDir(AppPaths::WritablePath(QStringLiteral("Result")));
	const QStringList robots = resultDir.entryList(QDir::Dirs | QDir::NoDotAndDotDot, QDir::Name);
	for (const QString& robot : robots)
	{
		if (robot.compare(QStringLiteral("Remote"), Qt::CaseInsensitive) == 0
			|| robot.compare(QStringLiteral("Archives"), Qt::CaseInsensitive) == 0)
		{
			continue;
		}
		const QDir robotDir(resultDir.filePath(robot));
		const QStringList caseNames = robotDir.entryList(QDir::Dirs | QDir::NoDotAndDotDot, QDir::Name);
		for (const QString& caseName : caseNames)
		{
			cases.append(qMakePair(robot + QStringLiteral(" / ") + caseName,
				robotDir.absoluteFilePath(caseName)));
		}
	}
	if (cases.isEmpty())
	{
		AppendLog(QStringLiteral("Result 下没有可上传的案例目录。"));
		return;
	}

	QDialog dlg(this);
	dlg.setWindowTitle(QStringLiteral("选择要上传的案例（可多选）"));
	QVBoxLayout* layout = new QVBoxLayout(&dlg);
	QLabel* hint = new QLabel(QStringLiteral("Ctrl 单个加选 / Shift 连续选；确定后按名字顺序依次打包上传。"), &dlg);
	hint->setStyleSheet("color: #7E9AA6; font-size: 12px;");
	layout->addWidget(hint);
	QListWidget* list = new QListWidget(&dlg);
	list->setSelectionMode(QAbstractItemView::ExtendedSelection);
	for (const auto& c : cases)
	{
		QListWidgetItem* item = new QListWidgetItem(c.first, list);
		item->setData(Qt::UserRole, c.second);
	}
	layout->addWidget(list, 1);
	QHBoxLayout* btnRow = new QHBoxLayout();
	QPushButton* selectAllBtn = new QPushButton(QStringLiteral("全选"), &dlg);
	connect(selectAllBtn, &QPushButton::clicked, list, &QListWidget::selectAll);
	btnRow->addWidget(selectAllBtn);
	btnRow->addStretch();
	QDialogButtonBox* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dlg);
	buttons->button(QDialogButtonBox::Ok)->setText(QStringLiteral("上传选中"));
	buttons->button(QDialogButtonBox::Ok)->setProperty("kind", "primary");
	buttons->button(QDialogButtonBox::Cancel)->setText(QStringLiteral("取消"));
	btnRow->addWidget(buttons);
	layout->addLayout(btnRow);
	connect(buttons, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
	connect(buttons, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
	dlg.resize(520, 480);
	if (dlg.exec() != QDialog::Accepted)
	{
		return;
	}

	QStringList picked;
	const QList<QListWidgetItem*> selected = list->selectedItems();
	for (const QListWidgetItem* item : selected)
	{
		picked << item->data(Qt::UserRole).toString();
	}
	if (picked.isEmpty() || m_uploader == nullptr)
	{
		return;
	}
	picked.sort(Qt::CaseInsensitive);   // 按名字升序依次入队
	SaveConfigFromUi();
	for (const QString& dir : picked)
	{
		m_uploader->QueueUpload(dir);
	}
	AppendLog(QStringLiteral("已选择 %1 个案例，按名字顺序加入上传队列。").arg(picked.size()));
}
