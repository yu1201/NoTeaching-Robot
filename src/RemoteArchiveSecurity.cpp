#include "RemoteArchiveSecurity.h"

#include "AppPaths.h"

#include <QDir>
#include <QDirIterator>
#include <QDateTime>
#include <QElapsedTimer>
#include <QFile>
#include <QFileInfo>
#include <QHash>
#include <QRegularExpression>
#include <QProcess>
#include <QProcessEnvironment>
#include <QSet>
#include <QStorageInfo>
#include <QUuid>
#include <QtCore/private/qzipreader_p.h>

#ifdef Q_OS_WIN
#include <windows.h>
#endif

#include <limits>
#include <iterator>

namespace
{
    struct ArchiveEntrySnapshot
    {
        QString path;
        qint64 size = 0;
    };

    struct DetailedInspection
    {
        RemoteArchiveSecurity::ArchiveInspection summary;
        QHash<QString, ArchiveEntrySnapshot> filesByFoldedPath;
    };

    void SetError(QString* error, const QString& message);

    struct ZipStructurePreflight
    {
        int entryCount = 0;
        quint32 centralDirectoryOffset = 0;
        quint32 centralDirectorySize = 0;
    };

    constexpr quint32 kLocalHeaderSignature = 0x04034b50u;
    constexpr quint32 kCentralHeaderSignature = 0x02014b50u;
    constexpr quint32 kEndOfCentralDirectorySignature = 0x06054b50u;
    constexpr quint32 kZip64EndSignature = 0x06064b50u;
    constexpr quint32 kZip64LocatorSignature = 0x07064b50u;
    constexpr quint32 kMaximumCentralDirectoryBytes = 16u * 1024u * 1024u;
    constexpr quint16 kMaximumZipEntryNameBytes = 1024;
    constexpr quint16 kMaximumZipExtraBytes = 4096;
    constexpr quint16 kMaximumZipCommentBytes = 4096;

    quint16 ReadLe16(const char* data)
    {
        return static_cast<quint16>(static_cast<unsigned char>(data[0]))
            | (static_cast<quint16>(static_cast<unsigned char>(data[1])) << 8);
    }

    quint32 ReadLe32(const char* data)
    {
        return static_cast<quint32>(static_cast<unsigned char>(data[0]))
            | (static_cast<quint32>(static_cast<unsigned char>(data[1])) << 8)
            | (static_cast<quint32>(static_cast<unsigned char>(data[2])) << 16)
            | (static_cast<quint32>(static_cast<unsigned char>(data[3])) << 24);
    }

    bool ReadExact(QFile* file, qint64 offset, qint64 size, QByteArray* output)
    {
        if (file == nullptr || output == nullptr || offset < 0 || size < 0
            || !file->seek(offset))
        {
            return false;
        }
        *output = file->read(size);
        return output->size() == size;
    }

    bool ExtraFieldIsBoundedAndNonZip64(const QByteArray& extra)
    {
        int cursor = 0;
        while (cursor < extra.size())
        {
            if (extra.size() - cursor < 4)
            {
                return false;
            }
            const quint16 headerId = ReadLe16(extra.constData() + cursor);
            const quint16 dataSize = ReadLe16(extra.constData() + cursor + 2);
            cursor += 4;
            if (headerId == 0x0001u || dataSize > extra.size() - cursor)
            {
                return false;
            }
            cursor += dataSize;
        }
        return cursor == extra.size();
    }

    bool IsStrictEntryNameBytes(const QByteArray& nameBytes, quint16 flags)
    {
        if (nameBytes.isEmpty()
            || nameBytes.size() > kMaximumZipEntryNameBytes
            || nameBytes.contains('\0'))
        {
            return false;
        }
        const bool utf8Flag = (flags & (1u << 11)) != 0;
        if (!utf8Flag)
        {
            return std::all_of(nameBytes.cbegin(), nameBytes.cend(), [](char value)
                { return static_cast<unsigned char>(value) < 0x80u; });
        }
        const QString decoded = QString::fromUtf8(nameBytes.constData(), nameBytes.size());
        return !decoded.contains(QChar(0xfffd))
            && decoded.toUtf8() == nameBytes;
    }

    bool PreflightZipStructure(
        const QString& archivePath,
        ZipStructurePreflight* result,
        QString* error)
    {
        if (result == nullptr)
        {
            SetError(error, QStringLiteral("ZIP 结构预检输出参数为空"));
            return false;
        }
        *result = ZipStructurePreflight{};
        QFile file(archivePath);
        if (!file.open(QIODevice::ReadOnly))
        {
            SetError(error, QStringLiteral("无法打开 ZIP 做 EOCD 预检"));
            return false;
        }
        const qint64 fileSize = file.size();
        const qint64 tailSize = std::min<qint64>(fileSize, 22 + 65535);
        QByteArray tail;
        if (fileSize < 22 || !ReadExact(&file, fileSize - tailSize, tailSize, &tail))
        {
            SetError(error, QStringLiteral("ZIP 太小或无法读取 EOCD 尾部"));
            return false;
        }

        qint64 eocdOffset = -1;
        int eocdCandidateCount = 0;
        const qint64 tailBase = fileSize - tailSize;
        for (int index = tail.size() - 22; index >= 0; --index)
        {
            const char* candidate = tail.constData() + index;
            if (ReadLe32(candidate) != kEndOfCentralDirectorySignature)
            {
                continue;
            }
            const quint16 commentBytes = ReadLe16(candidate + 20);
            if (commentBytes <= kMaximumZipCommentBytes
                && tailBase + index + 22 + commentBytes == fileSize)
            {
                eocdOffset = tailBase + index;
                ++eocdCandidateCount;
            }
        }
        if (eocdOffset < 0 || eocdCandidateCount != 1)
        {
            SetError(error, QStringLiteral("ZIP EOCD 缺失、重复、注释过长或尾部存在歧义数据"));
            return false;
        }
        QByteArray eocd;
        if (!ReadExact(&file, eocdOffset, 22, &eocd))
        {
            SetError(error, QStringLiteral("ZIP EOCD 读取失败"));
            return false;
        }
        const quint16 diskNumber = ReadLe16(eocd.constData() + 4);
        const quint16 centralDisk = ReadLe16(eocd.constData() + 6);
        const quint16 diskEntries = ReadLe16(eocd.constData() + 8);
        const quint16 totalEntries = ReadLe16(eocd.constData() + 10);
        const quint32 centralSize = ReadLe32(eocd.constData() + 12);
        const quint32 centralOffset = ReadLe32(eocd.constData() + 16);
        if (diskNumber != 0
            || centralDisk != 0
            || diskEntries != totalEntries
            || totalEntries == 0
            || totalEntries == 0xffffu
            || totalEntries > RemoteArchiveSecurity::MaximumCentralDirectoryEntries
            || centralSize == 0
            || centralSize == 0xffffffffu
            || centralSize > kMaximumCentralDirectoryBytes
            || centralOffset == 0xffffffffu
            || static_cast<quint64>(centralOffset) + centralSize
                != static_cast<quint64>(eocdOffset))
        {
            SetError(error, QStringLiteral(
                "ZIP64/多盘/超量 central directory 或 offset/size 边界非法"));
            return false;
        }
        if (eocdOffset >= 4)
        {
            QByteArray priorSignature;
            if (ReadExact(&file, eocdOffset - 4, 4, &priorSignature)
                && (ReadLe32(priorSignature.constData()) == kZip64LocatorSignature
                    || ReadLe32(priorSignature.constData()) == kZip64EndSignature))
            {
                SetError(error, QStringLiteral("ZIP64 结构已拒绝"));
                return false;
            }
        }
        if (eocdOffset >= 20)
        {
            QByteArray locator;
            if (ReadExact(&file, eocdOffset - 20, 4, &locator)
                && ReadLe32(locator.constData()) == kZip64LocatorSignature)
            {
                SetError(error, QStringLiteral("ZIP64 locator 已拒绝"));
                return false;
            }
        }

        qint64 cursor = centralOffset;
        QSet<quint32> localOffsets;
        quint32 minimumLocalOffset = std::numeric_limits<quint32>::max();
        for (quint16 entryIndex = 0; entryIndex < totalEntries; ++entryIndex)
        {
            QByteArray fixedCentral;
            if (!ReadExact(&file, cursor, 46, &fixedCentral)
                || ReadLe32(fixedCentral.constData()) != kCentralHeaderSignature)
            {
                SetError(error, QStringLiteral("ZIP central header 数量/签名不一致"));
                return false;
            }
            const quint16 versionMadeBy = ReadLe16(fixedCentral.constData() + 4);
            const quint16 flags = ReadLe16(fixedCentral.constData() + 8);
            const quint16 method = ReadLe16(fixedCentral.constData() + 10);
            const quint32 compressedBytes = ReadLe32(fixedCentral.constData() + 20);
            const quint32 uncompressedBytes = ReadLe32(fixedCentral.constData() + 24);
            const quint16 nameBytes = ReadLe16(fixedCentral.constData() + 28);
            const quint16 extraBytes = ReadLe16(fixedCentral.constData() + 30);
            const quint16 commentBytes = ReadLe16(fixedCentral.constData() + 32);
            const quint16 startingDisk = ReadLe16(fixedCentral.constData() + 34);
            const quint32 externalAttributes = ReadLe32(fixedCentral.constData() + 38);
            const quint32 localOffset = ReadLe32(fixedCentral.constData() + 42);
            const quint32 unixMode = externalAttributes >> 16;
            const bool unixSymlink = (versionMadeBy >> 8) == 3
                && (unixMode & 0170000u) == 0120000u;
            if ((flags & ((1u << 0) | (1u << 6) | (1u << 13))) != 0
                || (method != 0 && method != 8)
                || compressedBytes == 0xffffffffu
                || uncompressedBytes == 0xffffffffu
                || localOffset == 0xffffffffu
                || startingDisk != 0
                || nameBytes == 0
                || nameBytes > kMaximumZipEntryNameBytes
                || extraBytes > kMaximumZipExtraBytes
                || commentBytes > kMaximumZipCommentBytes
                || unixSymlink
                || localOffsets.contains(localOffset))
            {
                SetError(error, QStringLiteral(
                    "ZIP central entry 使用加密/ZIP64/多盘/链接/非法压缩或字段越界"));
                return false;
            }
            const qint64 variableSize = static_cast<qint64>(nameBytes)
                + extraBytes + commentBytes;
            if (cursor + 46 + variableSize
                > static_cast<qint64>(centralOffset) + centralSize)
            {
                SetError(error, QStringLiteral("ZIP central entry 逃出声明目录边界"));
                return false;
            }
            QByteArray variableCentral;
            if (!ReadExact(&file, cursor + 46, variableSize, &variableCentral))
            {
                SetError(error, QStringLiteral("ZIP central entry 变量字段读取失败"));
                return false;
            }
            const QByteArray centralName = variableCentral.left(nameBytes);
            const QByteArray centralExtra = variableCentral.mid(nameBytes, extraBytes);
            if (!IsStrictEntryNameBytes(centralName, flags)
                || !ExtraFieldIsBoundedAndNonZip64(centralExtra))
            {
                SetError(error, QStringLiteral("ZIP central entry 名称编码或 extra field 非法"));
                return false;
            }

            QByteArray fixedLocal;
            if (!ReadExact(&file, localOffset, 30, &fixedLocal)
                || ReadLe32(fixedLocal.constData()) != kLocalHeaderSignature)
            {
                SetError(error, QStringLiteral("ZIP local header offset/签名非法"));
                return false;
            }
            const quint16 localFlags = ReadLe16(fixedLocal.constData() + 6);
            const quint16 localMethod = ReadLe16(fixedLocal.constData() + 8);
            const quint32 localCompressedBytes = ReadLe32(fixedLocal.constData() + 18);
            const quint32 localUncompressedBytes = ReadLe32(fixedLocal.constData() + 22);
            const quint16 localNameBytes = ReadLe16(fixedLocal.constData() + 26);
            const quint16 localExtraBytes = ReadLe16(fixedLocal.constData() + 28);
            if (localFlags != flags
                || localMethod != method
                || localNameBytes != nameBytes
                || localNameBytes > kMaximumZipEntryNameBytes
                || localExtraBytes > kMaximumZipExtraBytes
                || ((flags & (1u << 3)) == 0
                    && (localCompressedBytes != compressedBytes
                        || localUncompressedBytes != uncompressedBytes)))
            {
                SetError(error, QStringLiteral("ZIP local/central header 字段不一致"));
                return false;
            }
            const qint64 localVariableSize = static_cast<qint64>(localNameBytes) + localExtraBytes;
            QByteArray variableLocal;
            if (!ReadExact(&file, static_cast<qint64>(localOffset) + 30,
                    localVariableSize, &variableLocal))
            {
                SetError(error, QStringLiteral("ZIP local header 变量字段读取失败"));
                return false;
            }
            const QByteArray localName = variableLocal.left(localNameBytes);
            const QByteArray localExtra = variableLocal.mid(localNameBytes, localExtraBytes);
            const quint64 dataOffset = static_cast<quint64>(localOffset) + 30
                + localVariableSize;
            if (localName != centralName
                || !ExtraFieldIsBoundedAndNonZip64(localExtra)
                || dataOffset > centralOffset
                || compressedBytes > centralOffset - dataOffset)
            {
                SetError(error, QStringLiteral(
                    "ZIP local name/extra/data range 与 central directory 不一致"));
                return false;
            }
            localOffsets.insert(localOffset);
            minimumLocalOffset = std::min(minimumLocalOffset, localOffset);
            cursor += 46 + variableSize;
        }
        if (cursor != static_cast<qint64>(centralOffset) + centralSize
            || minimumLocalOffset != 0)
        {
            SetError(error, QStringLiteral("ZIP central directory 未被精确消费或含前置 polyglot 数据"));
            return false;
        }
        result->entryCount = totalEntries;
        result->centralDirectoryOffset = centralOffset;
        result->centralDirectorySize = centralSize;
        return true;
    }

    void SetError(QString* error, const QString& message)
    {
        if (error != nullptr)
        {
            *error = message;
        }
    }

    bool IsCancelled(const std::atomic_bool* cancelFlag)
    {
        return cancelFlag != nullptr && cancelFlag->load();
    }

    QString NormalizedEntryPath(const QZipReader::FileInfo& entry)
    {
        if (entry.filePath.contains(QLatin1Char('\\')))
        {
            return {};
        }
        QString normalized = entry.filePath;
        while (normalized.endsWith(QLatin1Char('/')))
        {
            normalized.chop(1);
        }
        return normalized;
    }

    bool ParseArchive(
        const QString& archivePath,
        DetailedInspection* detailed,
        QString* error)
    {
        if (detailed == nullptr)
        {
            SetError(error, QStringLiteral("归档检查输出参数为空"));
            return false;
        }
        *detailed = DetailedInspection{};
        const QFileInfo archiveInfo(archivePath);
        if (!archiveInfo.isFile()
            || archiveInfo.isSymLink()
#ifdef Q_OS_WIN
            || archiveInfo.isJunction()
#endif
            || archiveInfo.size() <= 0
            || archiveInfo.size() > RemoteArchiveSecurity::MaximumArchiveBytes)
        {
            SetError(error, QStringLiteral("ZIP 不存在、是链接、为空或超过 2 GiB"));
            return false;
        }

        ZipStructurePreflight structuralPreflight;
        if (!PreflightZipStructure(archivePath, &structuralPreflight, error))
        {
            return false;
        }

        QZipReader archive(archivePath);
        const QList<QZipReader::FileInfo> entries = archive.fileInfoList();
        if (!archive.isReadable()
            || entries.isEmpty()
            || entries.size() != structuralPreflight.entryCount
            || entries.size() > RemoteArchiveSecurity::MaximumCentralDirectoryEntries)
        {
            archive.close();
            SetError(error, QStringLiteral("ZIP central directory 不可读、为空或条目数超过 %1")
                .arg(RemoteArchiveSecurity::MaximumCentralDirectoryEntries));
            return false;
        }

        QString robotName;
        QString caseName;
        QSet<QString> allFoldedPaths;
        QSet<QString> fileFoldedPaths;
        QList<QPair<QString, bool>> normalizedEntries;
        qint64 totalBytes = 0;
        int fileCount = 0;
        for (const QZipReader::FileInfo& entry : entries)
        {
            const QString normalized = NormalizedEntryPath(entry);
            const QStringList components = normalized.split(QLatin1Char('/'), Qt::KeepEmptyParts);
            if (!entry.isValid()
                || entry.isSymLink
                || (!entry.isFile && !entry.isDir)
                || normalized.isEmpty()
                || normalized.toUtf8().size() > 1024
                || components.size() > 32
                || QDir::isAbsolutePath(normalized))
            {
                archive.close();
                SetError(error, QStringLiteral("ZIP 含非法类型、链接、绝对/过深/过长路径"));
                return false;
            }
            for (const QString& component : components)
            {
                if (!AppPaths::IsSafePathComponent(component))
                {
                    archive.close();
                    SetError(error, QStringLiteral("ZIP 含不安全路径组件：%1").arg(normalized));
                    return false;
                }
            }
            const QString folded = normalized.toCaseFolded();
            if (allFoldedPaths.contains(folded))
            {
                archive.close();
                SetError(error, QStringLiteral("ZIP 含重复或仅大小写不同的路径：%1").arg(normalized));
                return false;
            }
            allFoldedPaths.insert(folded);
            normalizedEntries.push_back(qMakePair(normalized, entry.isFile));

            if (!entry.isFile)
            {
                if (entry.size != 0)
                {
                    archive.close();
                    SetError(error, QStringLiteral("ZIP 目录条目声明了非零大小"));
                    return false;
                }
                continue;
            }
            if (components.size() < 3
                || entry.size < 0
                || entry.size > RemoteArchiveSecurity::MaximumSingleFileBytes
                || entry.size > RemoteArchiveSecurity::MaximumUncompressedBytes - totalBytes)
            {
                archive.close();
                SetError(error, QStringLiteral("ZIP 文件不在 Robot/Case 下，或单项/总解压大小越界"));
                return false;
            }
            if (robotName.isEmpty())
            {
                robotName = components.at(0);
                caseName = components.at(1);
            }
            else if (components.at(0) != robotName || components.at(1) != caseName)
            {
                archive.close();
                SetError(error, QStringLiteral("ZIP 必须且只能包含一个 Robot/Case 案例树"));
                return false;
            }
            ++fileCount;
            totalBytes += entry.size;
            fileFoldedPaths.insert(folded);
            detailed->filesByFoldedPath.insert(folded, { normalized, entry.size });
        }
        archive.close();
        if (fileCount <= 0 || robotName.isEmpty() || caseName.isEmpty())
        {
            SetError(error, QStringLiteral("ZIP 没有可用案例文件"));
            return false;
        }

        // Reject file/directory prefix conflicts and empty trees outside the one case.
        for (const auto& normalizedEntry : normalizedEntries)
        {
            const QStringList components = normalizedEntry.first.split(QLatin1Char('/'));
            if (components.at(0) != robotName
                || (components.size() >= 2 && components.at(1) != caseName)
                || (components.size() == 1 && normalizedEntry.second)
                || (components.size() == 2 && normalizedEntry.second))
            {
                SetError(error, QStringLiteral("ZIP central directory 含案例树之外的条目"));
                return false;
            }
            QString ancestor;
            for (int index = 0; index + 1 < components.size(); ++index)
            {
                if (!ancestor.isEmpty())
                {
                    ancestor += QLatin1Char('/');
                }
                ancestor += components.at(index);
                if (fileFoldedPaths.contains(ancestor.toCaseFolded()))
                {
                    SetError(error, QStringLiteral("ZIP 含文件/目录前缀冲突：%1")
                        .arg(normalizedEntry.first));
                    return false;
                }
            }
        }

        detailed->summary.robotName = robotName;
        detailed->summary.caseName = caseName;
        detailed->summary.totalUncompressedBytes = totalBytes;
        detailed->summary.fileCount = fileCount;
        detailed->summary.centralDirectoryEntryCount = entries.size();
        return true;
    }

    bool IsLinkLike(const QFileInfo& info)
    {
        return info.isSymLink()
#ifdef Q_OS_WIN
            || info.isJunction()
#endif
            ;
    }

    const QRegularExpression& StrictStagingNamePattern()
    {
        static const QRegularExpression pattern(
            QStringLiteral(R"(^remote-staging-[0-9a-f]{32}$)"));
        return pattern;
    }

    bool ValidateDeviceRootIdentity(
        const QString& deviceRoot,
        QString* absolutePath,
        QString* canonicalPath,
        QString* error)
    {
        const QFileInfo deviceInfo(deviceRoot);
        const QString deviceAbsolute = QDir::cleanPath(deviceInfo.absoluteFilePath());
        if (!deviceInfo.exists() || !deviceInfo.isDir() || IsLinkLike(deviceInfo))
        {
            SetError(error, QStringLiteral("device 根目录不存在或不是安全普通目录"));
            return false;
        }
        const QString deviceCanonical = QDir::cleanPath(deviceInfo.canonicalFilePath());
        if (deviceCanonical.isEmpty())
        {
            SetError(error, QStringLiteral("无法回读 device 根目录的规范身份"));
            return false;
        }
        if (absolutePath != nullptr)
        {
            *absolutePath = deviceAbsolute;
        }
        if (canonicalPath != nullptr)
        {
            *canonicalPath = deviceCanonical;
        }
        return true;
    }

    bool ValidateTransactionRootIdentity(
        const QString& transactionRoot,
        const QString& deviceRoot,
        bool allowMissing,
        QString* transactionAbsolutePath,
        QString* error)
    {
        QString deviceAbsolute;
        QString deviceCanonical;
        if (!ValidateDeviceRootIdentity(
            deviceRoot, &deviceAbsolute, &deviceCanonical, error))
        {
            return false;
        }

        const QFileInfo transactionInfo(transactionRoot);
        const QString transactionAbsolute = QDir::cleanPath(transactionInfo.absoluteFilePath());
        if (!StrictStagingNamePattern().match(transactionInfo.fileName()).hasMatch()
            || QDir::cleanPath(transactionInfo.absolutePath()) != deviceAbsolute
            || transactionAbsolute == deviceAbsolute
            || IsLinkLike(transactionInfo))
        {
            SetError(error, QStringLiteral(
                "staging 必须是 device 根目录直属的 remote-staging-<32 lowercase hex> 普通目录"));
            return false;
        }
        if (!transactionInfo.exists())
        {
            if (!allowMissing)
            {
                SetError(error, QStringLiteral("staging 目录不存在"));
                return false;
            }
            if (transactionAbsolutePath != nullptr)
            {
                *transactionAbsolutePath = transactionAbsolute;
            }
            return true;
        }
        if (!transactionInfo.isDir())
        {
            SetError(error, QStringLiteral("staging 名称被非常规目录对象占用"));
            return false;
        }
        const QString transactionCanonical =
            QDir::cleanPath(transactionInfo.canonicalFilePath());
        if (transactionCanonical.isEmpty()
            || QDir::cleanPath(QFileInfo(transactionCanonical).absolutePath()) != deviceCanonical)
        {
            SetError(error, QStringLiteral("staging 规范身份逃出 device 根目录"));
            return false;
        }
        if (transactionAbsolutePath != nullptr)
        {
            *transactionAbsolutePath = transactionAbsolute;
        }
        return true;
    }

    bool AtomicRenameDirectoryNoReplace(const QString& source, const QString& destination)
    {
        if (QFileInfo::exists(destination))
        {
            return false;
        }
#ifdef Q_OS_WIN
        return MoveFileExW(
            reinterpret_cast<LPCWSTR>(source.utf16()),
            reinterpret_cast<LPCWSTR>(destination.utf16()),
            MOVEFILE_WRITE_THROUGH) != FALSE;
#else
        return QDir().rename(source, destination);
#endif
    }

    bool VerifyExtractedFiles(
        const QString& extractionRoot,
        const DetailedInspection& expected,
        const std::atomic_bool* cancelFlag,
        QString* error)
    {
        QHash<QString, ArchiveEntrySnapshot> actual;
        qint64 totalBytes = 0;
        QDirIterator iterator(
            extractionRoot,
            QDir::AllEntries | QDir::NoDotAndDotDot | QDir::Hidden | QDir::System,
            QDirIterator::Subdirectories);
        while (iterator.hasNext())
        {
            if (IsCancelled(cancelFlag))
            {
                SetError(error, QStringLiteral("远程 ZIP 解压验证已取消"));
                return false;
            }
            iterator.next();
            const QFileInfo info = iterator.fileInfo();
            if (info.isSymLink()
#ifdef Q_OS_WIN
                || info.isJunction()
#endif
                || (!info.isDir() && !info.isFile()))
            {
                SetError(error, QStringLiteral("解压结果含链接或非常规文件"));
                return false;
            }
            if (!info.isFile())
            {
                continue;
            }
            const QString relative = QDir(extractionRoot).relativeFilePath(info.absoluteFilePath())
                .replace(QLatin1Char('\\'), QLatin1Char('/'));
            const QString folded = relative.toCaseFolded();
            if (actual.contains(folded)
                || info.size() < 0
                || info.size() > RemoteArchiveSecurity::MaximumSingleFileBytes
                || info.size() > RemoteArchiveSecurity::MaximumUncompressedBytes - totalBytes)
            {
                SetError(error, QStringLiteral("解压结果存在重复路径或实际大小越界"));
                return false;
            }
            actual.insert(folded, { relative, info.size() });
            totalBytes += info.size();
        }
        if (actual.size() != expected.filesByFoldedPath.size()
            || totalBytes != expected.summary.totalUncompressedBytes)
        {
            SetError(error, QStringLiteral("解压后的文件数/总大小与 central directory 不一致"));
            return false;
        }
        for (auto it = expected.filesByFoldedPath.cbegin(); it != expected.filesByFoldedPath.cend(); ++it)
        {
            const auto actualIt = actual.constFind(it.key());
            if (actualIt == actual.cend()
                || actualIt->path != it->path
                || actualIt->size != it->size)
            {
                SetError(error, QStringLiteral("解压文件路径/大小与 central directory 不一致：%1")
                    .arg(it->path));
                return false;
            }
        }
        return true;
    }

    bool ExtractWithCancellableSystemTar(
        const QString& archivePath,
        const QString& extractionRoot,
        const std::atomic_bool* cancelFlag,
        QString* error)
    {
#ifndef Q_OS_WIN
        Q_UNUSED(archivePath);
        Q_UNUSED(extractionRoot);
        Q_UNUSED(cancelFlag);
        SetError(error, QStringLiteral("受控远程 ZIP 解压只允许 Windows System32 tar.exe"));
        return false;
#else
        wchar_t systemDirectoryBuffer[MAX_PATH] = {};
        const UINT systemDirectoryLength = GetSystemDirectoryW(
            systemDirectoryBuffer,
            static_cast<UINT>(std::size(systemDirectoryBuffer)));
        if (systemDirectoryLength == 0
            || systemDirectoryLength >= std::size(systemDirectoryBuffer))
        {
            SetError(error, QStringLiteral("无法定位固定 Windows System32 目录"));
            return false;
        }
        const QString systemDirectory = QString::fromWCharArray(
            systemDirectoryBuffer,
            static_cast<int>(systemDirectoryLength));
        const QFileInfo tarInfo(QDir(systemDirectory).filePath(QStringLiteral("tar.exe")));
        const QString canonicalTar = tarInfo.canonicalFilePath();
        const QString canonicalSystemDirectory = QFileInfo(systemDirectory).canonicalFilePath();
        if (!tarInfo.isFile()
            || tarInfo.isSymLink()
            || tarInfo.isJunction()
            || canonicalTar.isEmpty()
            || canonicalSystemDirectory.isEmpty()
            || QDir::cleanPath(QFileInfo(canonicalTar).absolutePath())
                != QDir::cleanPath(canonicalSystemDirectory))
        {
            SetError(error, QStringLiteral("固定 System32 tar.exe 缺失或路径身份异常"));
            return false;
        }

        QProcess extractor;
        extractor.setProgram(canonicalTar);
        extractor.setArguments({
            QStringLiteral("-xf"),
            QDir::toNativeSeparators(archivePath),
            QStringLiteral("-C"),
            QDir::toNativeSeparators(extractionRoot),
        });
        extractor.setWorkingDirectory(canonicalSystemDirectory);
        QProcessEnvironment childEnvironment;
        childEnvironment.insert(QStringLiteral("PATH"), canonicalSystemDirectory);
        extractor.setProcessEnvironment(childEnvironment);
        extractor.setProcessChannelMode(QProcess::MergedChannels);
        extractor.setCreateProcessArgumentsModifier(
            [](QProcess::CreateProcessArguments* arguments)
            {
                arguments->flags |= CREATE_NO_WINDOW;
            });
        extractor.start(QIODevice::ReadOnly);
        if (!extractor.waitForStarted(5000))
        {
            SetError(error, QStringLiteral("无法启动固定 System32 tar.exe：%1")
                .arg(extractor.errorString()));
            return false;
        }

        QElapsedTimer elapsed;
        elapsed.start();
        qint64 lastResourceScanMs = -1000;
        bool cancelled = false;
        bool timedOut = false;
        bool resourceLimitExceeded = false;
        QString resourceError;
        QByteArray diagnosticTail;
        auto drainDiagnostics = [&]()
            {
                diagnosticTail.append(extractor.readAll());
                if (diagnosticTail.size() > 4096)
                {
                    diagnosticTail = diagnosticTail.right(4096);
                }
            };
        while (true)
        {
            const bool finished = extractor.waitForFinished(100);
            // Drain continuously: thousands of rejected-entry diagnostics must never
            // fill the child pipe and turn cancellation/join into a deadlock.
            drainDiagnostics();
            if (finished)
            {
                break;
            }
            cancelled = IsCancelled(cancelFlag);
            timedOut = elapsed.elapsed() > 30LL * 60 * 1000;
            if (!cancelled && !timedOut
                && elapsed.elapsed() - lastResourceScanMs >= 250)
            {
                lastResourceScanMs = elapsed.elapsed();
                qint64 totalBytes = 0;
                int fileCount = 0;
                QDirIterator runtimeIterator(
                    extractionRoot,
                    QDir::AllEntries | QDir::NoDotAndDotDot | QDir::Hidden | QDir::System,
                    QDirIterator::Subdirectories);
                while (runtimeIterator.hasNext())
                {
                    runtimeIterator.next();
                    const QFileInfo info = runtimeIterator.fileInfo();
                    if (info.isSymLink() || info.isJunction()
                        || (!info.isDir() && !info.isFile()))
                    {
                        resourceLimitExceeded = true;
                        resourceError = QStringLiteral("解压子进程产生链接或非常规文件");
                        break;
                    }
                    if (!info.isFile())
                    {
                        continue;
                    }
                    ++fileCount;
                    if (fileCount > RemoteArchiveSecurity::MaximumCentralDirectoryEntries
                        || info.size() < 0
                        || info.size() > RemoteArchiveSecurity::MaximumSingleFileBytes
                        || info.size() > RemoteArchiveSecurity::MaximumUncompressedBytes - totalBytes)
                    {
                        resourceLimitExceeded = true;
                        resourceError = QStringLiteral("解压实际文件数/单项/总输出超过硬上限");
                        break;
                    }
                    totalBytes += info.size();
                }
                if (!resourceLimitExceeded
                    && !RemoteArchiveSecurity::HasWorkspaceCapacity(
                        extractionRoot, 0, &resourceError))
                {
                    resourceLimitExceeded = true;
                }
            }
            if (!cancelled && !timedOut && !resourceLimitExceeded)
            {
                continue;
            }
            extractor.terminate();
            if (!extractor.waitForFinished(1500))
            {
                extractor.kill();
                if (!extractor.waitForFinished(5000))
                {
                    // Do not let transaction cleanup race a still-live extractor.
                    // On Windows QProcess::kill uses TerminateProcess; the unbounded
                    // wait is only after that hard termination request.
                    extractor.kill();
                    extractor.waitForFinished(-1);
                }
            }
            drainDiagnostics();
            break;
        }
        while (extractor.state() != QProcess::NotRunning)
        {
            extractor.kill();
            extractor.waitForFinished(-1);
            drainDiagnostics();
        }
        if (cancelled || timedOut || resourceLimitExceeded)
        {
            SetError(error,
                cancelled
                    ? QStringLiteral("远程 ZIP 子进程解压已取消并终止")
                    : (timedOut
                        ? QStringLiteral("远程 ZIP 子进程解压超过 30 分钟硬上限并终止")
                        : QStringLiteral("远程 ZIP 子进程因运行时资源门禁终止：%1")
                            .arg(resourceError)));
            return false;
        }
        if (extractor.exitStatus() != QProcess::NormalExit || extractor.exitCode() != 0)
        {
            drainDiagnostics();
            const QString detail = QString::fromLocal8Bit(diagnosticTail).trimmed().right(1000);
            SetError(error, QStringLiteral("System32 tar.exe 解压失败(exit=%1)：%2")
                .arg(extractor.exitCode()).arg(detail));
            return false;
        }
        return true;
#endif
    }
}

namespace RemoteArchiveSecurity
{
    bool ValidateListedArchive(const QString& fileName, qulonglong listedBytes, QString* error)
    {
        if (!AppPaths::IsSafePathComponent(fileName)
            || !fileName.endsWith(QStringLiteral(".zip"), Qt::CaseInsensitive)
            || listedBytes == 0
            || listedBytes > static_cast<qulonglong>(MaximumArchiveBytes))
        {
            SetError(error, QStringLiteral("远端 ZIP 名称或 LIST 大小无效/超过 2 GiB"));
            return false;
        }
        static const QRegularExpression writeOncePattern(QStringLiteral(
            R"(_\d{8}T\d{9}_[0-9a-f]{12}_(\d+)\.zip$)"),
            QRegularExpression::CaseInsensitiveOption);
        const QRegularExpressionMatch match = writeOncePattern.match(fileName);
        if (!match.hasMatch())
        {
            return true;
        }
        bool sizeOk = false;
        const qulonglong embeddedBytes = match.captured(1).toULongLong(&sizeOk);
        if (!sizeOk || embeddedBytes == 0 || embeddedBytes != listedBytes)
        {
            SetError(error, QStringLiteral("写一次 ZIP 文件名声明大小与 FTP LIST 不一致"));
            return false;
        }
        return true;
    }

    bool ValidateOpenedArchiveSize(
        const QString& fileName,
        qulonglong listedBytes,
        qulonglong openedBytes,
        QString* error)
    {
        if (!ValidateListedArchive(fileName, listedBytes, error)
            || openedBytes != listedBytes)
        {
            if (error != nullptr && error->isEmpty())
            {
                *error = QStringLiteral("FTP 打开后的远端大小与先前 LIST 声明不一致");
            }
            return false;
        }
        return true;
    }

    bool HasWorkspaceCapacity(
        const QString& pathOnTargetVolume,
        qulonglong additionalBytes,
        QString* error)
    {
        if (additionalBytes > static_cast<qulonglong>(
            std::numeric_limits<qint64>::max() - MinimumFreeDiskReserveBytes))
        {
            SetError(error, QStringLiteral("远程归档磁盘预算溢出"));
            return false;
        }
        const QFileInfo targetInfo(pathOnTargetVolume);
        const QString probePath = targetInfo.isDir()
            ? targetInfo.absoluteFilePath()
            : targetInfo.absolutePath();
        QStorageInfo storage(probePath);
        storage.refresh();
        const qint64 required = static_cast<qint64>(additionalBytes) + MinimumFreeDiskReserveBytes;
        if (!storage.isValid() || !storage.isReady() || storage.bytesAvailable() < required)
        {
            SetError(error, QStringLiteral("目标磁盘空间不足：需要新增 %1 字节并保留 2 GiB 安全余量")
                .arg(additionalBytes));
            return false;
        }
        return true;
    }

    bool InspectArchive(
        const QString& archivePath,
        ArchiveInspection* inspection,
        QString* error)
    {
        DetailedInspection detailed;
        if (!ParseArchive(archivePath, &detailed, error))
        {
            return false;
        }
        if (inspection != nullptr)
        {
            *inspection = detailed.summary;
        }
        return true;
    }

    bool CleanupStagingTransaction(
        const QString& transactionRoot,
        const QString& deviceRoot,
        QString* error)
    {
        QString transactionAbsolute;
        if (!ValidateTransactionRootIdentity(
            transactionRoot, deviceRoot, true, &transactionAbsolute, error))
        {
            return false;
        }
        const QFileInfo before(transactionAbsolute);
        if (!before.exists())
        {
            return true;
        }
        const bool removed = QDir(transactionAbsolute).removeRecursively();
        const QFileInfo after(transactionAbsolute);
        if (!removed || after.exists() || IsLinkLike(after))
        {
            SetError(error, QStringLiteral("staging 递归删除失败或删除后路径仍可见：%1")
                .arg(QDir::toNativeSeparators(transactionAbsolute)));
            return false;
        }
        return true;
    }

    bool AuditAndCleanupStaging(
        const QString& deviceRoot,
        StagingAuditResult* result,
        QString* error,
        qint64 nowUtcMs)
    {
        if (result != nullptr)
        {
            *result = StagingAuditResult{};
        }
        QString deviceAbsolute;
        QString deviceCanonical;
        if (!ValidateDeviceRootIdentity(
            deviceRoot, &deviceAbsolute, &deviceCanonical, error))
        {
            return false;
        }
        Q_UNUSED(deviceCanonical);

        const qint64 auditNow = nowUtcMs >= 0
            ? nowUtcMs
            : QDateTime::currentDateTimeUtc().toMSecsSinceEpoch();
        QStringList deletionCandidates;
        int scannedEntries = 0;
        int stagingDirectories = 0;
        QDirIterator iterator(
            deviceAbsolute,
            QDir::AllEntries | QDir::NoDotAndDotDot | QDir::Hidden | QDir::System,
            QDirIterator::NoIteratorFlags);
        while (iterator.hasNext())
        {
            iterator.next();
            ++scannedEntries;
            if (scannedEntries > MaximumDeviceRootEntriesScanned)
            {
                SetError(error, QStringLiteral("device 根目录直属项超过审计上限 %1")
                    .arg(MaximumDeviceRootEntriesScanned));
                return false;
            }

            const QFileInfo info = iterator.fileInfo();
            if (!info.fileName().startsWith(
                QStringLiteral("remote-staging-"), Qt::CaseInsensitive))
            {
                continue;
            }
            ++stagingDirectories;
            if (stagingDirectories > MaximumStagingDirectoriesPerAudit)
            {
                SetError(error, QStringLiteral("staging 残留数量超过审计上限 %1")
                    .arg(MaximumStagingDirectoriesPerAudit));
                return false;
            }

            QString validatedPath;
            QString validationError;
            if (!ValidateTransactionRootIdentity(
                info.absoluteFilePath(), deviceAbsolute, false, &validatedPath, &validationError))
            {
                SetError(error, QStringLiteral("发现歧义 staging 残留：%1（%2）")
                    .arg(info.fileName(), validationError));
                return false;
            }
            const QDateTime lastModified = info.lastModified().toUTC();
            if (!lastModified.isValid())
            {
                SetError(error, QStringLiteral("无法回读 staging 修改时间：%1")
                    .arg(info.fileName()));
                return false;
            }
            const qint64 modifiedMs = lastModified.toMSecsSinceEpoch();
            if (modifiedMs > auditNow
                || auditNow - modifiedMs < StagingCleanupMinimumAgeMs)
            {
                SetError(error, QStringLiteral(
                    "发现未达到 24 小时安全清理年龄的 staging，已禁止启动批次：%1")
                    .arg(info.fileName()));
                return false;
            }
            deletionCandidates.push_back(validatedPath);
        }

        if (deletionCandidates.size() > MaximumStagingDirectoriesDeletedPerAudit)
        {
            SetError(error, QStringLiteral("本次 staging 清理候选超过删除预算 %1")
                .arg(MaximumStagingDirectoriesDeletedPerAudit));
            return false;
        }

        int deletedDirectories = 0;
        for (const QString& candidate : deletionCandidates)
        {
            QString cleanupError;
            if (!CleanupStagingTransaction(candidate, deviceAbsolute, &cleanupError))
            {
                SetError(error, QStringLiteral("staging 审计清理失败：%1")
                    .arg(cleanupError));
                return false;
            }
            ++deletedDirectories;
        }
        if (result != nullptr)
        {
            result->scannedEntries = scannedEntries;
            result->stagingDirectories = stagingDirectories;
            result->deletedDirectories = deletedDirectories;
        }
        return true;
    }

    PromotionResult ExtractValidateAndPromote(
        const QString& archivePath,
        const QString& deviceRoot,
        const std::atomic_bool* cancelFlag)
    {
        PromotionResult result;
        const QFileInfo archiveInfo(archivePath);
        const QString transactionRoot = QDir::cleanPath(archiveInfo.absolutePath());
        QString deviceAbsolute;
        QString deviceCanonical;
        QString identityError;
        const bool safeDevice = ValidateDeviceRootIdentity(
            deviceRoot, &deviceAbsolute, &deviceCanonical, &identityError);
        Q_UNUSED(deviceCanonical);
        auto fail = [&](const QString& message) -> PromotionResult
            {
                QString cleanupError;
                const bool cleaned = safeDevice
                    && CleanupStagingTransaction(
                        transactionRoot, deviceAbsolute, &cleanupError);
                result.status = PromotionStatus::Failed;
                result.promotedCasePath.clear();
                result.stagingCleanupSucceeded = cleaned;
                result.message = cleaned
                    ? message
                    : message + QStringLiteral("；且 staging 清理失败，已禁止继续：%1")
                        .arg(cleanupError.isEmpty() ? identityError : cleanupError);
                return result;
            };
        QString transactionAbsolute;
        if (!safeDevice
            || !ValidateTransactionRootIdentity(
                transactionRoot,
                safeDevice ? deviceAbsolute : deviceRoot,
                false,
                &transactionAbsolute,
                &identityError)
            || QDir::cleanPath(archiveInfo.absolutePath()) != transactionAbsolute)
        {
            return fail(QStringLiteral("远程 ZIP 不在受控 device staging 目录内：%1")
                .arg(identityError));
        }
        if (IsCancelled(cancelFlag))
        {
            return fail(QStringLiteral("远程 ZIP 处理已取消"));
        }

        DetailedInspection detailed;
        QString operationError;
        if (!ParseArchive(archivePath, &detailed, &operationError))
        {
            return fail(operationError.isEmpty()
                ? QStringLiteral("ZIP central directory 验证失败")
                : operationError);
        }
        if (!HasWorkspaceCapacity(
            deviceAbsolute,
            static_cast<qulonglong>(detailed.summary.totalUncompressedBytes),
            &operationError))
        {
            return fail(operationError.isEmpty()
                ? QStringLiteral("解压磁盘空间不足")
                : operationError);
        }

        const QString extractionRoot = QDir(transactionRoot).filePath(QStringLiteral("extracted"));
        if (!QDir().mkpath(extractionRoot))
        {
            return fail(QStringLiteral("无法创建受控解压 staging"));
        }
        QString extractionError;
        if (!ExtractWithCancellableSystemTar(
            archivePath, extractionRoot, cancelFlag, &extractionError))
        {
            return fail(extractionError);
        }
        if (IsCancelled(cancelFlag))
        {
            return fail(QStringLiteral("远程 ZIP 解压后已取消，未提升任何文件"));
        }
        if (!VerifyExtractedFiles(
            extractionRoot, detailed, cancelFlag, &operationError))
        {
            return fail(operationError.isEmpty()
                ? QStringLiteral("解压结果复验失败")
                : operationError);
        }

        const QString stagedRobot = QDir(extractionRoot).filePath(detailed.summary.robotName);
        const QString stagedCase = QDir(stagedRobot).filePath(detailed.summary.caseName);
        const QString finalRobot = QDir(deviceAbsolute).filePath(detailed.summary.robotName);
        const QString finalCase = QDir(finalRobot).filePath(detailed.summary.caseName);
        if (IsCancelled(cancelFlag))
        {
            return fail(QStringLiteral("远程 ZIP 提升前已取消"));
        }
        if (QFileInfo::exists(finalCase))
        {
            return fail(QStringLiteral("目标案例已存在，禁止覆盖：%1")
                .arg(QDir::toNativeSeparators(finalCase)));
        }
        bool promoted = false;
        if (!QFileInfo::exists(finalRobot))
        {
            promoted = AtomicRenameDirectoryNoReplace(stagedRobot, finalRobot);
        }
        else
        {
            const QFileInfo robotInfo(finalRobot);
            if (!robotInfo.isDir()
                || robotInfo.isSymLink()
#ifdef Q_OS_WIN
                || robotInfo.isJunction()
#endif
                )
            {
                return fail(QStringLiteral("目标机器人目录不是安全普通目录"));
            }
            promoted = AtomicRenameDirectoryNoReplace(stagedCase, finalCase);
        }
        if (!promoted)
        {
            return fail(QStringLiteral("案例 staging 原子提升失败，目标未修改"));
        }

        // Commit boundary: publish the final path and committed state before attempting
        // best-effort cleanup.  A payload lock must never make a landed case look failed.
        result.status = PromotionStatus::PromotedCleanupFailed;
        result.promotedCasePath = finalCase;
        result.stagingCleanupSucceeded = false;
        result.message = QStringLiteral("案例已原子提升，正在回读 staging 清理结果");
        QString cleanupError;
        if (!CleanupStagingTransaction(
            transactionRoot, deviceAbsolute, &cleanupError))
        {
            result.message = QStringLiteral("案例已落地但 staging 清理失败：%1")
                .arg(cleanupError);
            return result;
        }
        result.status = PromotionStatus::Promoted;
        result.stagingCleanupSucceeded = true;
        result.message = QStringLiteral("案例已原子提升且 staging 清理完成");
        return result;
    }
}
