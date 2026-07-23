#include "TheoreticalRobotModelStore.h"

#include "AppPaths.h"

#include <QByteArrayView>
#include <QCryptographicHash>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLockFile>
#include <QMutex>
#include <QMutexLocker>
#include <QSaveFile>
#include <QSet>
#include <QTemporaryFile>

#include <algorithm>


namespace
{
constexpr int kManifestSchemaVersion = 1;
constexpr qint64 kMaximumManifestBytes = 1024 * 1024;
constexpr qsizetype kMaximumManifestAssets = 1024;
const QString kStoreRelativePath = QStringLiteral("Data/RobotModels");
const QString kManifestName = QStringLiteral("assets.json");
const QString kMutationLockName = QStringLiteral("store.lock");

struct Manifest
{
    QList<TheoreticalRobotModelStore::Asset> assets;
    QString activeSha256;
};

struct FileIdentity
{
    QString sha256;
    qint64 sizeBytes = 0;
    qint64 modifiedMs = 0;
    QByteArray head;
    QByteArray tail;
};

QMutex& StoreMutationMutex()
{
    static QMutex mutex;
    return mutex;
}

bool IsLinkOrJunction(const QFileInfo& info)
{
#ifdef Q_OS_WIN
    return info.isSymLink() || info.isJunction();
#else
    return info.isSymLink();
#endif
}

bool IsLowerSha256(const QString& value)
{
    if (value.size() != 64) return false;
    for (const QChar ch : value)
    {
        if (!((ch >= QLatin1Char('0') && ch <= QLatin1Char('9'))
              || (ch >= QLatin1Char('a') && ch <= QLatin1Char('f'))))
        {
            return false;
        }
    }
    return true;
}

bool IsSafeDisplayName(const QString& value)
{
    if (value.isEmpty() || value.size() > 1024) return false;
    for (const QChar ch : value)
    {
        if (ch.unicode() < 0x20 || ch.unicode() == 0x7f) return false;
    }
    return true;
}

bool HasStepEnvelope(const FileIdentity& identity)
{
    QByteArray head = identity.head;
    if (head.startsWith("\xEF\xBB\xBF")) head.remove(0, 3);
    while (!head.isEmpty()
           && (head.front() == ' ' || head.front() == '\t'
               || head.front() == '\r' || head.front() == '\n'))
    {
        head.remove(0, 1);
    }
    return head.startsWith("ISO-10303-21;")
        && identity.tail.trimmed().endsWith("END-ISO-10303-21;");
}

bool ReadStableIdentity(
    const QString& path,
    qint64 maximumBytes,
    FileIdentity& identity,
    QString& error)
{
    identity = FileIdentity();
    const QFileInfo before(path);
    if (!before.exists() || !before.isFile() || IsLinkOrJunction(before))
    {
        error = QStringLiteral("STEP 资产不是可读取的普通文件：%1")
            .arg(QDir::toNativeSeparators(path));
        return false;
    }
    if (before.size() <= 0 || before.size() > maximumBytes)
    {
        error = QStringLiteral("STEP 资产必须大于 0 且不超过 %1 MiB：%2")
            .arg(TheoreticalRobotModelStore::MaximumAssetBytes / (1024 * 1024))
            .arg(QDir::toNativeSeparators(path));
        return false;
    }

    const qint64 expectedSize = before.size();
    const qint64 expectedModifiedMs = before.lastModified().toMSecsSinceEpoch();
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly) || file.size() != expectedSize)
    {
        error = QStringLiteral("无法稳定打开 STEP 资产：%1")
            .arg(file.errorString().isEmpty() ? QDir::toNativeSeparators(path)
                                              : file.errorString());
        return false;
    }

    QCryptographicHash hash(QCryptographicHash::Sha256);
    QByteArray block(1024 * 1024, Qt::Uninitialized);
    qint64 totalBytes = 0;
    while (!file.atEnd())
    {
        const qint64 count = file.read(block.data(), block.size());
        if (count < 0)
        {
            error = QStringLiteral("读取 STEP 资产失败：%1").arg(file.errorString());
            return false;
        }
        if (count == 0) continue;
        totalBytes += count;
        if (totalBytes > expectedSize || totalBytes > maximumBytes)
        {
            error = QStringLiteral("STEP 资产读取长度越界，已拒绝导入。");
            return false;
        }
        hash.addData(QByteArrayView(block.constData(), static_cast<qsizetype>(count)));
        if (identity.head.size() < 4096)
        {
            const qsizetype take = (std::min)(
                static_cast<qsizetype>(count), 4096 - identity.head.size());
            identity.head.append(block.constData(), take);
        }
        identity.tail.append(block.constData(), static_cast<qsizetype>(count));
        if (identity.tail.size() > 4096) identity.tail = identity.tail.right(4096);
    }
    file.close();

    QFileInfo after(path);
    after.refresh();
    if (totalBytes != expectedSize
        || !after.exists() || !after.isFile() || IsLinkOrJunction(after)
        || after.size() != expectedSize
        || after.lastModified().toMSecsSinceEpoch() != expectedModifiedMs)
    {
        error = QStringLiteral("STEP 资产在读取过程中发生变化，已拒绝使用。");
        identity = FileIdentity();
        return false;
    }

    identity.sha256 = QString::fromLatin1(hash.result().toHex()).toLower();
    identity.sizeBytes = expectedSize;
    identity.modifiedMs = expectedModifiedMs;
    return true;
}

bool EnsureStoreDirectory(QString& error)
{
    const QString path = TheoreticalRobotModelStore::StoreDirectory();
    if (path.isEmpty())
    {
        error = QStringLiteral("无法解析理论机器人模型库目录。");
        return false;
    }
    const QString dataRoot = QDir::cleanPath(AppPaths::DataRootPath());
    const QString dataDirectory = QDir(dataRoot).filePath(QStringLiteral("Data"));
    for (const QString& componentPath : { dataDirectory, path })
    {
        const QFileInfo component(componentPath);
        if (IsLinkOrJunction(component))
        {
            error = QStringLiteral("理论机器人模型库路径包含符号链接或联接点：%1")
                .arg(QDir::toNativeSeparators(componentPath));
            return false;
        }
    }

    QFileInfo info(path);
    if (info.exists() && (!info.isDir() || IsLinkOrJunction(info)))
    {
        error = QStringLiteral("理论机器人模型库路径不是普通目录：%1")
            .arg(QDir::toNativeSeparators(path));
        return false;
    }
    if (!info.exists() && !QDir().mkpath(path))
    {
        error = QStringLiteral("无法创建理论机器人模型库：%1")
            .arg(QDir::toNativeSeparators(path));
        return false;
    }
    info.refresh();
    if (!info.exists() || !info.isDir() || IsLinkOrJunction(info))
    {
        error = QStringLiteral("理论机器人模型库目录校验失败。");
        return false;
    }
    const QString canonicalRoot = QFileInfo(dataRoot).canonicalFilePath();
    const QString canonicalStore = info.canonicalFilePath();
    const QString relative = QDir(canonicalRoot).relativeFilePath(canonicalStore)
        .replace(QLatin1Char('\\'), QLatin1Char('/'));
    if (canonicalRoot.isEmpty() || canonicalStore.isEmpty()
        || relative == QStringLiteral("..")
        || relative.startsWith(QStringLiteral("../"))
        || QFileInfo(relative).isAbsolute() || QDir::isAbsolutePath(relative))
    {
        error = QStringLiteral("理论机器人模型库的真实路径越出数据根目录。");
        return false;
    }
    return true;
}

QString StoredAssetPath(const QString& fileName)
{
    if (!AppPaths::IsSafePathComponent(fileName)) return QString();
    const QString path = AppPaths::WritableChildPath(kStoreRelativePath, fileName);
    const QString store = TheoreticalRobotModelStore::StoreDirectory();
    if (path.isEmpty() || store.isEmpty()) return QString();
    const QString relative = QDir(store).relativeFilePath(path)
        .replace(QLatin1Char('\\'), QLatin1Char('/'));
    if (relative != fileName) return QString();
    return path;
}

bool ValidateAssetRecord(
    const TheoreticalRobotModelStore::Asset& asset,
    QString& error)
{
    if (!IsLowerSha256(asset.sha256)
        || asset.storedFileName != asset.sha256 + QStringLiteral(".step")
        || StoredAssetPath(asset.storedFileName).isEmpty()
        || !IsSafeDisplayName(asset.originalDisplayName)
        || asset.sizeBytes <= 0
        || asset.sizeBytes > TheoreticalRobotModelStore::MaximumAssetBytes)
    {
        error = QStringLiteral("理论机器人资产清单包含无效或越界记录。");
        return false;
    }
    const QDateTime imported = QDateTime::fromString(asset.importedUtc, Qt::ISODateWithMs);
    if (!imported.isValid())
    {
        error = QStringLiteral("理论机器人资产清单中的导入时间无效。");
        return false;
    }
    return true;
}

bool ReadManifest(Manifest& manifest, QString& error)
{
    manifest = Manifest();
    if (!EnsureStoreDirectory(error)) return false;
    const QString path = TheoreticalRobotModelStore::ManifestFilePath();
    if (path.isEmpty())
    {
        error = QStringLiteral("无法解析理论机器人资产清单路径。");
        return false;
    }
    const QFileInfo before(path);
    if (IsLinkOrJunction(before))
    {
        error = QStringLiteral("理论机器人资产清单路径是符号链接或联接点。");
        return false;
    }
    if (!before.exists()) return true;
    if (!before.isFile()
        || before.size() <= 0 || before.size() > kMaximumManifestBytes)
    {
        error = QStringLiteral("理论机器人资产清单不是有效的普通小文件。");
        return false;
    }

    QFile file(path);
    if (!file.open(QIODevice::ReadOnly) || file.size() != before.size())
    {
        error = QStringLiteral("无法读取理论机器人资产清单：%1").arg(file.errorString());
        return false;
    }
    const QByteArray bytes = file.readAll();
    file.close();
    QFileInfo after(path);
    after.refresh();
    if (bytes.size() != before.size()
        || !after.exists() || !after.isFile() || IsLinkOrJunction(after)
        || after.size() != before.size()
        || after.lastModified().toMSecsSinceEpoch()
            != before.lastModified().toMSecsSinceEpoch())
    {
        error = QStringLiteral("理论机器人资产清单在读取过程中发生变化。");
        return false;
    }

    QJsonParseError parseError;
    const QJsonDocument document = QJsonDocument::fromJson(bytes, &parseError);
    if (parseError.error != QJsonParseError::NoError || !document.isObject())
    {
        error = QStringLiteral("理论机器人资产清单 JSON 无效：%1")
            .arg(parseError.errorString());
        return false;
    }
    const QJsonObject root = document.object();
    if (root.value(QStringLiteral("schemaVersion")).toInt(-1)
            != kManifestSchemaVersion
        || !root.value(QStringLiteral("assets")).isArray()
        || !root.value(QStringLiteral("activeSha256")).isString())
    {
        error = QStringLiteral("理论机器人资产清单版本或字段无效。");
        return false;
    }
    const QJsonArray assets = root.value(QStringLiteral("assets")).toArray();
    if (assets.size() > kMaximumManifestAssets)
    {
        error = QStringLiteral("理论机器人资产清单条目过多。");
        return false;
    }

    QSet<QString> uniqueHashes;
    for (const QJsonValue& value : assets)
    {
        if (!value.isObject())
        {
            error = QStringLiteral("理论机器人资产清单包含非对象条目。");
            return false;
        }
        const QJsonObject object = value.toObject();
        TheoreticalRobotModelStore::Asset asset;
        asset.sha256 = object.value(QStringLiteral("sha256")).toString();
        asset.storedFileName = object.value(QStringLiteral("storedFileName")).toString();
        asset.originalDisplayName = object.value(QStringLiteral("originalDisplayName")).toString();
        asset.sizeBytes = object.value(QStringLiteral("sizeBytes")).toInteger(-1);
        asset.importedUtc = object.value(QStringLiteral("importedUtc")).toString();
        if (!ValidateAssetRecord(asset, error) || uniqueHashes.contains(asset.sha256))
        {
            if (error.isEmpty())
                error = QStringLiteral("理论机器人资产清单包含重复 SHA-256。");
            return false;
        }
        uniqueHashes.insert(asset.sha256);
        manifest.assets.append(asset);
    }
    manifest.activeSha256 = root.value(QStringLiteral("activeSha256")).toString();
    if (!manifest.activeSha256.isEmpty()
        && (!IsLowerSha256(manifest.activeSha256)
            || !uniqueHashes.contains(manifest.activeSha256)))
    {
        error = QStringLiteral("理论机器人资产清单的当前资产引用无效。");
        return false;
    }
    return true;
}

bool WriteManifest(const Manifest& manifest, QString& error)
{
    if (manifest.assets.size() > kMaximumManifestAssets)
    {
        error = QStringLiteral("理论机器人资产清单条目超过持久化上限。");
        return false;
    }
    QSet<QString> uniqueHashes;
    QJsonArray assets;
    for (const TheoreticalRobotModelStore::Asset& asset : manifest.assets)
    {
        if (!ValidateAssetRecord(asset, error) || uniqueHashes.contains(asset.sha256))
        {
            if (error.isEmpty()) error = QStringLiteral("拒绝写入重复的机器人资产记录。");
            return false;
        }
        uniqueHashes.insert(asset.sha256);
        assets.append(QJsonObject{
            { QStringLiteral("sha256"), asset.sha256 },
            { QStringLiteral("storedFileName"), asset.storedFileName },
            { QStringLiteral("originalDisplayName"), asset.originalDisplayName },
            { QStringLiteral("sizeBytes"), asset.sizeBytes },
            { QStringLiteral("importedUtc"), asset.importedUtc }
        });
    }
    if (!manifest.activeSha256.isEmpty()
        && !uniqueHashes.contains(manifest.activeSha256))
    {
        error = QStringLiteral("拒绝写入不存在的当前机器人资产引用。");
        return false;
    }

    const QString path = TheoreticalRobotModelStore::ManifestFilePath();
    const QFileInfo existing(path);
    if (IsLinkOrJunction(existing))
    {
        error = QStringLiteral("理论机器人资产清单路径是符号链接或联接点。");
        return false;
    }
    if (existing.exists() && !existing.isFile())
    {
        error = QStringLiteral("理论机器人资产清单路径被非普通文件占用。");
        return false;
    }
    const QJsonDocument document(QJsonObject{
        { QStringLiteral("schemaVersion"), kManifestSchemaVersion },
        { QStringLiteral("activeSha256"), manifest.activeSha256 },
        { QStringLiteral("assets"), assets }
    });
    const QByteArray bytes = document.toJson(QJsonDocument::Indented);
    if (bytes.isEmpty() || bytes.size() > kMaximumManifestBytes)
    {
        error = QStringLiteral("理论机器人资产清单超过持久化上限。");
        return false;
    }

    QSaveFile file(path);
    file.setDirectWriteFallback(false);
    if (!file.open(QIODevice::WriteOnly)
        || file.write(bytes) != bytes.size()
        || !file.commit())
    {
        file.cancelWriting();
        error = QStringLiteral("无法原子写入理论机器人资产清单：%1")
            .arg(file.errorString());
        return false;
    }
    return true;
}

bool AcquireMutationLock(QLockFile& lock, QString& error)
{
    lock.setStaleLockTime(0);
    if (lock.tryLock(10000)) return true;
    error = QStringLiteral("理论机器人资产库正在被另一个进程修改，请稍后重试。");
    return false;
}

bool ValidateStoredAsset(
    const TheoreticalRobotModelStore::Asset& asset,
    QString& path,
    QString& error)
{
    path.clear();
    if (!ValidateAssetRecord(asset, error)) return false;
    const QString candidate = StoredAssetPath(asset.storedFileName);
    if (candidate.isEmpty())
    {
        error = QStringLiteral("理论机器人资产路径解析越界，已拒绝使用。");
        return false;
    }
    FileIdentity identity;
    if (!ReadStableIdentity(candidate, TheoreticalRobotModelStore::MaximumAssetBytes,
                            identity, error))
        return false;
    if (!HasStepEnvelope(identity))
    {
        error = QStringLiteral("理论机器人资产不包含有效的 STEP 文件边界。");
        return false;
    }
    if (identity.sizeBytes != asset.sizeBytes || identity.sha256 != asset.sha256)
    {
        error = QStringLiteral("理论机器人资产大小或 SHA-256 与清单不一致。");
        return false;
    }
    path = candidate;
    return true;
}

bool CopySourceToNewAsset(
    const QString& sourcePath,
    const FileIdentity& expected,
    const QString& targetPath,
    QString& error)
{
    const QFileInfo sourceBefore(sourcePath);
    if (!sourceBefore.exists() || !sourceBefore.isFile() || IsLinkOrJunction(sourceBefore)
        || sourceBefore.size() != expected.sizeBytes
        || sourceBefore.lastModified().toMSecsSinceEpoch() != expected.modifiedMs)
    {
        error = QStringLiteral("STEP 源文件在复制前发生变化。");
        return false;
    }

    QTemporaryFile temporary(
        QDir(TheoreticalRobotModelStore::StoreDirectory())
            .filePath(QStringLiteral("import-XXXXXX.tmp")));
    temporary.setAutoRemove(true);
    if (!temporary.open())
    {
        error = QStringLiteral("无法在机器人资产库创建同盘暂存文件：%1")
            .arg(temporary.errorString());
        return false;
    }
    QFile source(sourcePath);
    if (!source.open(QIODevice::ReadOnly) || source.size() != expected.sizeBytes)
    {
        error = QStringLiteral("无法稳定打开待复制的 STEP：%1").arg(source.errorString());
        return false;
    }

    QCryptographicHash copiedHash(QCryptographicHash::Sha256);
    QByteArray block(1024 * 1024, Qt::Uninitialized);
    qint64 copiedBytes = 0;
    while (!source.atEnd())
    {
        const qint64 count = source.read(block.data(), block.size());
        if (count < 0)
        {
            error = QStringLiteral("复制 STEP 时读取失败：%1").arg(source.errorString());
            return false;
        }
        if (count == 0) continue;
        copiedBytes += count;
        if (copiedBytes > expected.sizeBytes
            || copiedBytes > TheoreticalRobotModelStore::MaximumAssetBytes)
        {
            error = QStringLiteral("复制 STEP 时读取长度越界。");
            return false;
        }
        copiedHash.addData(QByteArrayView(block.constData(), static_cast<qsizetype>(count)));
        qint64 offset = 0;
        while (offset < count)
        {
            const qint64 written = temporary.write(block.constData() + offset, count - offset);
            if (written <= 0)
            {
                error = QStringLiteral("复制 STEP 时写入暂存文件失败：%1")
                    .arg(temporary.errorString());
                return false;
            }
            offset += written;
        }
    }
    source.close();
    if (copiedBytes != expected.sizeBytes
        || QString::fromLatin1(copiedHash.result().toHex()).toLower() != expected.sha256
        || !temporary.flush())
    {
        error = QStringLiteral("STEP 源文件在复制过程中发生变化或暂存写入失败。");
        return false;
    }

    QFileInfo sourceAfter(sourcePath);
    sourceAfter.refresh();
    if (!sourceAfter.exists() || !sourceAfter.isFile() || IsLinkOrJunction(sourceAfter)
        || sourceAfter.size() != expected.sizeBytes
        || sourceAfter.lastModified().toMSecsSinceEpoch() != expected.modifiedMs)
    {
        error = QStringLiteral("STEP 源文件在复制过程中发生变化。");
        return false;
    }
    if (QFileInfo::exists(targetPath))
    {
        error = QStringLiteral("哈希命名的机器人资产文件已被占用。");
        return false;
    }
    if (!temporary.rename(targetPath))
    {
        // 另一个遵守同一规则的进程可能刚完成同内容导入。只在目标完整匹配时接受。
        FileIdentity racedTarget;
        QString racedError;
        if (ReadStableIdentity(targetPath, TheoreticalRobotModelStore::MaximumAssetBytes,
                               racedTarget, racedError)
            && racedTarget.sizeBytes == expected.sizeBytes
            && racedTarget.sha256 == expected.sha256
            && HasStepEnvelope(racedTarget))
        {
            return true;
        }
        error = QStringLiteral("无法原子发布理论机器人 STEP 资产：%1")
            .arg(temporary.errorString());
        return false;
    }
    temporary.setAutoRemove(false);
    return true;
}
}

QString TheoreticalRobotModelStore::StoreDirectory()
{
    if (!AppPaths::IsInitialized()) return QString();
    return AppPaths::WritablePath(kStoreRelativePath);
}

QString TheoreticalRobotModelStore::ManifestFilePath()
{
    if (!AppPaths::IsInitialized()) return QString();
    return AppPaths::WritableChildPath(kStoreRelativePath, kManifestName);
}

bool TheoreticalRobotModelStore::ImportStepFile(
    const QString& sourcePath,
    Asset& importedAsset,
    QString& error,
    bool activate)
{
    importedAsset = Asset();
    error.clear();
    const QFileInfo sourceInfo(sourcePath);
    const QString suffix = sourceInfo.suffix().toLower();
    if (suffix != QStringLiteral("step") && suffix != QStringLiteral("stp"))
    {
        error = QStringLiteral("理论机器人模型只接受 .step 或 .stp 文件。");
        return false;
    }
    if (!IsSafeDisplayName(sourceInfo.fileName()))
    {
        error = QStringLiteral("STEP 原始显示名为空、过长或包含控制字符。");
        return false;
    }

    FileIdentity sourceIdentity;
    if (!ReadStableIdentity(sourcePath, MaximumAssetBytes, sourceIdentity, error))
        return false;
    if (!HasStepEnvelope(sourceIdentity))
    {
        error = QStringLiteral("所选文件不包含有效的 STEP 文件边界。");
        return false;
    }
    if (!EnsureStoreDirectory(error)) return false;

    QMutexLocker<QMutex> guard(&StoreMutationMutex());
    const QString lockPath = StoredAssetPath(kMutationLockName);
    if (lockPath.isEmpty())
    {
        error = QStringLiteral("机器人资产库锁路径解析失败。");
        return false;
    }
    const QFileInfo lockInfo(lockPath);
    if (IsLinkOrJunction(lockInfo) || (lockInfo.exists() && !lockInfo.isFile()))
    {
        error = QStringLiteral("机器人资产库锁路径被非普通文件占用。");
        return false;
    }
    QLockFile lock(lockPath);
    if (!AcquireMutationLock(lock, error)) return false;

    Manifest manifest;
    if (!ReadManifest(manifest, error)) return false;

    const QString storedFileName = sourceIdentity.sha256 + QStringLiteral(".step");
    const QString targetPath = StoredAssetPath(storedFileName);
    if (targetPath.isEmpty())
    {
        error = QStringLiteral("哈希命名的机器人资产路径解析失败。");
        return false;
    }
    const bool alreadyListed = std::any_of(
        manifest.assets.cbegin(), manifest.assets.cend(),
        [&candidateHash = sourceIdentity.sha256](const Asset& asset)
        {
            return asset.sha256 == candidateHash;
        });
    if (!alreadyListed && manifest.assets.size() >= kMaximumManifestAssets)
    {
        error = QStringLiteral("理论机器人资产清单已达到条目上限。");
        return false;
    }

    Asset candidate;
    candidate.sha256 = sourceIdentity.sha256;
    candidate.storedFileName = storedFileName;
    candidate.originalDisplayName = sourceInfo.fileName();
    candidate.sizeBytes = sourceIdentity.sizeBytes;
    candidate.importedUtc = QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs);

    const QFileInfo existingTarget(targetPath);
    if (IsLinkOrJunction(existingTarget))
    {
        error = QStringLiteral("哈希命名的机器人资产路径是符号链接或联接点。");
        return false;
    }
    if (existingTarget.exists())
    {
        QString verifiedPath;
        if (!ValidateStoredAsset(candidate, verifiedPath, error))
        {
            error = QStringLiteral("库中同 SHA 文件名已存在但内容不同，已保留原文件：%1")
                .arg(error);
            return false;
        }
    }
    else if (!CopySourceToNewAsset(sourcePath, sourceIdentity, targetPath, error))
    {
        return false;
    }

    int existingIndex = -1;
    for (int index = 0; index < manifest.assets.size(); ++index)
    {
        if (manifest.assets.at(index).sha256 == candidate.sha256)
        {
            existingIndex = index;
            break;
        }
    }
    if (existingIndex >= 0)
    {
        const Asset& existing = manifest.assets.at(existingIndex);
        if (existing.storedFileName != candidate.storedFileName
            || existing.sizeBytes != candidate.sizeBytes)
        {
            error = QStringLiteral("清单中的同 SHA 机器人资产记录不一致。");
            return false;
        }
        importedAsset = existing; // 同内容重复导入时保留第一次的原始显示名和时间。
    }
    else
    {
        manifest.assets.append(candidate);
        importedAsset = candidate;
    }
    if (activate) manifest.activeSha256 = candidate.sha256;
    if (!WriteManifest(manifest, error))
    {
        importedAsset = Asset();
        return false;
    }
    return true;
}

bool TheoreticalRobotModelStore::ListAssets(QList<Asset>& assets, QString& error)
{
    assets.clear();
    error.clear();
    Manifest manifest;
    if (!ReadManifest(manifest, error)) return false;
    assets = manifest.assets;
    return true;
}

bool TheoreticalRobotModelStore::ReadActiveRecord(
    Asset& activeAsset,
    bool& hasActive,
    QString& error)
{
    activeAsset = Asset();
    hasActive = false;
    error.clear();

    Manifest manifest;
    if (!ReadManifest(manifest, error)) return false;
    if (manifest.activeSha256.isEmpty()) return true;

    for (const Asset& asset : manifest.assets)
    {
        if (asset.sha256 == manifest.activeSha256)
        {
            activeAsset = asset;
            hasActive = true;
            return true;
        }
    }
    // ReadManifest 已经检查过引用完整性；保留防御性关闭失败，避免未来修改
    // 清单解析逻辑时把悬空 active 记录泄漏给快速启动路径。
    error = QStringLiteral("当前理论机器人资产引用不存在。");
    return false;
}

bool TheoreticalRobotModelStore::CompareExchangeActive(
    const QString& expectedSha256,
    const QString& desiredSha256,
    QString& error)
{
    error.clear();
    const QString expected = expectedSha256.trimmed().toLower();
    const QString desired = desiredSha256.trimmed().toLower();
    if ((!expected.isEmpty() && !IsLowerSha256(expected))
        || (!desired.isEmpty() && !IsLowerSha256(desired)))
    {
        error = QStringLiteral("理论机器人资产 SHA-256 无效。");
        return false;
    }
    if (!EnsureStoreDirectory(error)) return false;

    QMutexLocker<QMutex> guard(&StoreMutationMutex());
    const QString lockPath = StoredAssetPath(kMutationLockName);
    if (lockPath.isEmpty())
    {
        error = QStringLiteral("机器人资产库锁路径解析失败。");
        return false;
    }
    const QFileInfo lockInfo(lockPath);
    if (IsLinkOrJunction(lockInfo) || (lockInfo.exists() && !lockInfo.isFile()))
    {
        error = QStringLiteral("机器人资产库锁路径被非普通文件占用。");
        return false;
    }
    QLockFile lock(lockPath);
    if (!AcquireMutationLock(lock, error)) return false;

    Manifest manifest;
    if (!ReadManifest(manifest, error)) return false;
    if (manifest.activeSha256 != expected)
    {
        error = QStringLiteral(
            "当前机器人资产已被其它实例切换，条件更新已拒绝（预期 %1，实际 %2）。")
            .arg(expected.isEmpty() ? QStringLiteral("空") : expected.left(12),
                 manifest.activeSha256.isEmpty()
                     ? QStringLiteral("空") : manifest.activeSha256.left(12));
        return false;
    }

    if (!desired.isEmpty())
    {
        const Asset* selected = nullptr;
        for (const Asset& asset : manifest.assets)
        {
            if (asset.sha256 == desired)
            {
                selected = &asset;
                break;
            }
        }
        if (selected == nullptr)
        {
            error = QStringLiteral("指定的理论机器人资产不在当前清单中。");
            return false;
        }
        QString verifiedPath;
        if (!ValidateStoredAsset(*selected, verifiedPath, error)) return false;
    }

    if (manifest.activeSha256 == desired) return true;
    manifest.activeSha256 = desired;
    return WriteManifest(manifest, error);
}

bool TheoreticalRobotModelStore::SetActive(const QString& sha256, QString& error)
{
    error.clear();
    const QString normalized = sha256.trimmed().toLower();
    if (!IsLowerSha256(normalized))
    {
        error = QStringLiteral("理论机器人资产 SHA-256 无效。");
        return false;
    }
    if (!EnsureStoreDirectory(error)) return false;

    QMutexLocker<QMutex> guard(&StoreMutationMutex());
    const QString lockPath = StoredAssetPath(kMutationLockName);
    if (lockPath.isEmpty())
    {
        error = QStringLiteral("机器人资产库锁路径解析失败。");
        return false;
    }
    const QFileInfo lockInfo(lockPath);
    if (IsLinkOrJunction(lockInfo) || (lockInfo.exists() && !lockInfo.isFile()))
    {
        error = QStringLiteral("机器人资产库锁路径被非普通文件占用。");
        return false;
    }
    QLockFile lock(lockPath);
    if (!AcquireMutationLock(lock, error)) return false;

    Manifest manifest;
    if (!ReadManifest(manifest, error)) return false;
    const Asset* selected = nullptr;
    for (const Asset& asset : manifest.assets)
    {
        if (asset.sha256 == normalized)
        {
            selected = &asset;
            break;
        }
    }
    if (selected == nullptr)
    {
        error = QStringLiteral("指定的理论机器人资产不在当前清单中。");
        return false;
    }
    QString verifiedPath;
    if (!ValidateStoredAsset(*selected, verifiedPath, error)) return false;
    manifest.activeSha256 = normalized;
    return WriteManifest(manifest, error);
}

bool TheoreticalRobotModelStore::ResolveActive(
    QString& stepPath,
    Asset& activeAsset,
    QString& error)
{
    stepPath.clear();
    activeAsset = Asset();
    error.clear();

    Manifest manifest;
    if (!ReadManifest(manifest, error)) return false;
    if (manifest.activeSha256.isEmpty())
    {
        error = QStringLiteral("尚未选择理论机器人 STEP 资产。");
        return false;
    }
    const Asset* selected = nullptr;
    for (const Asset& asset : manifest.assets)
    {
        if (asset.sha256 == manifest.activeSha256)
        {
            selected = &asset;
            break;
        }
    }
    if (selected == nullptr)
    {
        error = QStringLiteral("当前理论机器人资产引用不存在。");
        return false;
    }

    QString verifiedPath;
    if (!ValidateStoredAsset(*selected, verifiedPath, error)) return false;
    stepPath = verifiedPath;
    activeAsset = *selected;
    return true;
}
