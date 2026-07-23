#include "ReferenceModelLibrary.h"

#include "RobotDataHelper.h"
#include "StepModelImporter.h"

#include <QCryptographicHash>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QList>
#include <QLockFile>
#include <QMutex>
#include <QMutexLocker>
#include <QPair>
#include <QRegularExpression>
#include <QSaveFile>
#include <QThread>
#include <QUuid>

namespace
{
const QString kLibSubdir = QStringLiteral("Data/ReferenceModels");

QMutex& LibraryMutationMutex()
{
    static QMutex mutex;
    return mutex;
}

QString StableFileSha256(const QString& path, QString& error)
{
    const QFileInfo before(path);
    if (!before.exists() || !before.isFile() || before.isSymLink())
    {
        error = QStringLiteral("文件不是可读取的普通文件：%1").arg(path);
        return QString();
    }
    const qint64 expectedSize = before.size();
    const qint64 expectedModifiedMs = before.lastModified().toMSecsSinceEpoch();
    if (expectedSize <= 0)
    {
        error = QStringLiteral("文件为空：%1").arg(path);
        return QString();
    }

    QFile file(path);
    if (!file.open(QIODevice::ReadOnly))
    {
        error = QStringLiteral("无法读取文件：%1").arg(file.errorString());
        return QString();
    }
    if (file.size() != expectedSize)
    {
        error = QStringLiteral("文件在读取前已变化：%1").arg(path);
        return QString();
    }

    QCryptographicHash hash(QCryptographicHash::Sha256);
    QByteArray block;
    block.resize(1024 * 1024);
    while (!file.atEnd())
    {
        const qint64 count = file.read(block.data(), block.size());
        if (count < 0)
        {
            error = QStringLiteral("读取文件失败：%1").arg(file.errorString());
            return QString();
        }
        if (count > 0)
        {
            hash.addData(QByteArrayView(block.constData(), static_cast<qsizetype>(count)));
        }
    }
    file.close();

    QFileInfo after(path);
    after.refresh();
    if (!after.exists() || !after.isFile() || after.isSymLink()
        || after.size() != expectedSize
        || after.lastModified().toMSecsSinceEpoch() != expectedModifiedMs)
    {
        error = QStringLiteral("文件在读取过程中发生变化：%1").arg(path);
        return QString();
    }
    return QString::fromLatin1(hash.result().toHex()).toLower();
}

QJsonArray VectorJson(const Eigen::Vector3d& value)
{
    return QJsonArray{ value.x(), value.y(), value.z() };
}

QJsonArray StringListJson(const QStringList& values)
{
    QJsonArray result;
    for (const QString& value : values) result.append(value);
    return result;
}

bool ReadJsonObjectFile(const QString& path, QJsonObject& object, QString& error)
{
    object = QJsonObject();
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly))
    {
        error = QStringLiteral("无法读取JSON文件：%1").arg(path);
        return false;
    }
    QJsonParseError parseError;
    const QJsonDocument document = QJsonDocument::fromJson(file.readAll(), &parseError);
    if (parseError.error != QJsonParseError::NoError || !document.isObject())
    {
        error = QStringLiteral("JSON文件格式无效：%1").arg(path);
        return false;
    }
    object = document.object();
    return true;
}

bool IsStepMetadata(const QJsonObject& object)
{
    return object.value(QStringLiteral("schemaVersion")).toInt() == 1
        && object.value(QStringLiteral("recipeVersion")).toString()
            == QStringLiteral("occt-step-mesh-v1");
}

bool AcquireLibraryMutationLock(QLockFile& lock, QString& error)
{
    // STEP 解析/三角化可能远超 30 秒；不能把仍在工作的长事务误判为陈旧锁。
    lock.setStaleLockTime(0);
    if (lock.tryLock(10000)) return true;
    error = QStringLiteral("模型库正在被另一个程序修改，请稍后重试。");
    return false;
}

bool RemoveFileWithRetry(const QString& path)
{
    if (!QFileInfo::exists(path)) return true;
    // QFile::copy 会保留源 STEP 的只读属性；库内副本必须可由模型管理功能删除。
    const QFileDevice::Permissions permissions = QFile::permissions(path);
    QFile::setPermissions(path, permissions | QFileDevice::WriteOwner | QFileDevice::WriteUser);
    // Defender/索引器仍可能短暂持有刚复制的 CAD 文件；总等待上限 3 秒。
    constexpr int kAttempts = 30;
    for (int attempt = 0; attempt < kAttempts; ++attempt)
    {
        if (QFile::remove(path) || !QFileInfo::exists(path)) return true;
        if (attempt + 1 < kAttempts) QThread::msleep(100);
    }
    return false;
}

bool HasStepArtifacts(const QString& name)
{
    return QFileInfo::exists(ReferenceModelLibrary::SourceStepPath(name))
        || QFileInfo::exists(ReferenceModelLibrary::ModelMetadataPath(name));
}

struct PendingStepState
{
    bool pending = false;
    bool canConfirm = false;
    QString summary;
    QString confirmationToken;
};

QString PendingStepSnapshotToken(const QString& name, QString& error)
{
    QCryptographicHash snapshot(QCryptographicHash::Sha256);
    const QList<QPair<QByteArray, QString>> artifacts = {
        { QByteArrayLiteral("ply"), ReferenceModelLibrary::ModelPath(name) },
        { QByteArrayLiteral("source"), ReferenceModelLibrary::SourceStepPath(name) },
        { QByteArrayLiteral("metadata"), ReferenceModelLibrary::ModelMetadataPath(name) }
    };
    for (const auto& artifact : artifacts)
    {
        snapshot.addData(artifact.first);
        const QFileInfo before(artifact.second);
        if (!before.exists())
        {
            snapshot.addData(QByteArrayLiteral(":missing;"));
            continue;
        }
        if (!before.isFile() || before.isSymLink())
        {
            error = QStringLiteral("待处理 STEP 记录包含非普通文件，拒绝生成撤销标识：%1")
                .arg(artifact.second);
            return QString();
        }
        const qint64 expectedSize = before.size();
        const qint64 expectedModifiedMs = before.lastModified().toMSecsSinceEpoch();
        QFile file(artifact.second);
        if (!file.open(QIODevice::ReadOnly))
        {
            error = QStringLiteral("无法读取待处理 STEP 记录以生成撤销标识：%1")
                .arg(file.errorString());
            return QString();
        }
        QCryptographicHash fileHash(QCryptographicHash::Sha256);
        QByteArray block(1024 * 1024, Qt::Uninitialized);
        while (!file.atEnd())
        {
            const qint64 count = file.read(block.data(), block.size());
            if (count < 0)
            {
                error = QStringLiteral("读取待处理 STEP 记录失败：%1").arg(file.errorString());
                return QString();
            }
            if (count > 0)
                fileHash.addData(QByteArrayView(block.constData(), static_cast<qsizetype>(count)));
        }
        file.close();
        QFileInfo after(artifact.second);
        after.refresh();
        if (!after.exists() || !after.isFile() || after.isSymLink()
            || after.size() != expectedSize
            || after.lastModified().toMSecsSinceEpoch() != expectedModifiedMs)
        {
            error = QStringLiteral("待处理 STEP 记录在检查过程中发生变化，请重新检查。");
            return QString();
        }
        snapshot.addData(QByteArrayLiteral(":"));
        snapshot.addData(QByteArray::number(expectedSize));
        snapshot.addData(QByteArrayLiteral(":"));
        snapshot.addData(QByteArray::number(expectedModifiedMs));
        snapshot.addData(QByteArrayLiteral(":"));
        snapshot.addData(fileHash.result().toHex());
        snapshot.addData(QByteArrayLiteral(";"));
    }
    return QStringLiteral("snapshot-%1")
        .arg(QString::fromLatin1(snapshot.result().toHex()).toLower());
}

QString StepSummaryFromMetadata(const QJsonObject& metadata)
{
    QStringList units;
    for (const QJsonValue& value : metadata.value(QStringLiteral("sourceLengthUnits")).toArray())
    {
        const QString unit = value.toString().trimmed();
        if (!unit.isEmpty()) units.append(unit);
    }
    if (units.isEmpty()) units.append(QStringLiteral("未知"));

    const QJsonArray dimensions = metadata.value(QStringLiteral("bounds"))
        .toObject().value(QStringLiteral("dimensionsMm")).toArray();
    const double x = dimensions.size() > 0 ? dimensions.at(0).toDouble() : 0.0;
    const double y = dimensions.size() > 1 ? dimensions.at(1).toDouble() : 0.0;
    const double z = dimensions.size() > 2 ? dimensions.at(2).toDouble() : 0.0;
    const double deflection = metadata.value(QStringLiteral("triangulation"))
        .toObject().value(QStringLiteral("linearDeflectionMm")).toDouble();
    return QStringLiteral(
        "声明单位 %1；尺寸 %2 × %3 × %4 mm；%5 个曲面，%6 个顶点，%7 个三角形；离散误差 %8 mm。")
        .arg(units.join(QStringLiteral(" / ")))
        .arg(x, 0, 'f', 2)
        .arg(y, 0, 'f', 2)
        .arg(z, 0, 'f', 2)
        .arg(metadata.value(QStringLiteral("meshedFaceCount")).toInteger())
        .arg(metadata.value(QStringLiteral("vertexCount")).toInteger())
        .arg(metadata.value(QStringLiteral("triangleCount")).toInteger())
        .arg(deflection, 0, 'f', 3);
}

bool InspectPendingStepImportUnlocked(
    const QString& name,
    PendingStepState& state,
    QString& error)
{
    state = PendingStepState();
    error.clear();
    const QString plyPath = ReferenceModelLibrary::ModelPath(name);
    const QString sourcePath = ReferenceModelLibrary::SourceStepPath(name);
    const QString metadataPath = ReferenceModelLibrary::ModelMetadataPath(name);
    const bool hasPly = QFileInfo::exists(plyPath);
    const bool hasSource = QFileInfo::exists(sourcePath);
    const bool hasMetadata = QFileInfo::exists(metadataPath);

    // 没有 STEP sidecar 时，无论 PLY 是否存在，均不是待确认 STEP 事务。
    if (!hasSource && !hasMetadata) return true;
    state.pending = true;
    const auto useSnapshotActionToken = [&]()
    {
        state.confirmationToken.clear();
        state.confirmationToken = PendingStepSnapshotToken(name, error);
        return !state.confirmationToken.isEmpty();
    };
    if (!hasMetadata)
    {
        state.summary = QStringLiteral("上次 STEP 导入只留下源文件，事务不完整，不能确认。");
        return useSnapshotActionToken();
    }

    QJsonObject metadata;
    QString metadataError;
    if (!ReadJsonObjectFile(metadataPath, metadata, metadataError) || !IsStepMetadata(metadata))
    {
        state.summary = QStringLiteral("上次 STEP 导入的追溯元数据损坏，不能确认：%1")
            .arg(metadataError.isEmpty() ? QStringLiteral("元数据类型无效") : metadataError);
        return useSnapshotActionToken();
    }
    state.summary = StepSummaryFromMetadata(metadata);
    state.confirmationToken = metadata.value(QStringLiteral("transactionId")).toString();

    if (metadata.value(QStringLiteral("dimensionsConfirmed")).toBool(false))
    {
        // 完整且已确认的模型不是 pending；缺件时则作为可显式清理的损坏残留处理。
        if (hasPly && hasSource)
        {
            state.pending = false;
            state.summary.clear();
            state.confirmationToken.clear();
        }
        else
        {
            state.summary += QStringLiteral("\n已确认记录缺少 PLY 或 STEP 源文件，不能恢复。");
            if (!useSnapshotActionToken()) return false;
        }
        return true;
    }
    if (!hasPly || !hasSource)
    {
        state.summary += QStringLiteral("\n上次导入未完整提交，缺少 PLY 或 STEP 源文件，不能确认。");
        if (!useSnapshotActionToken()) return false;
        return true;
    }
    if (state.confirmationToken.isEmpty())
    {
        state.summary += QStringLiteral("\n上次导入没有事务标识，不能确认。");
        return useSnapshotActionToken();
    }

    QString hashError;
    const QString sourceHash = StableFileSha256(sourcePath, hashError);
    if (sourceHash.isEmpty()
        || sourceHash != metadata.value(QStringLiteral("sourceSha256")).toString())
    {
        state.summary += QStringLiteral("\nSTEP 源文件完整性校验失败，不能确认：%1")
            .arg(sourceHash.isEmpty() ? hashError : QStringLiteral("SHA-256 不一致"));
        return useSnapshotActionToken();
    }
    const QString plyHash = StableFileSha256(plyPath, hashError);
    if (plyHash.isEmpty()
        || plyHash != metadata.value(QStringLiteral("plySha256")).toString())
    {
        state.summary += QStringLiteral("\nPLY 网格完整性校验失败，不能确认：%1")
            .arg(plyHash.isEmpty() ? hashError : QStringLiteral("SHA-256 不一致"));
        return useSnapshotActionToken();
    }
    state.canConfirm = true;
    return true;
}
}

QString ReferenceModelLibrary::LibraryDir()
{
    const QString dir = RobotDataHelper::BuildProjectPath(kLibSubdir);
    QDir().mkpath(dir);  // 不存在则递归创建
    return dir;
}

QString ReferenceModelLibrary::ModelPath(const QString& name)
{
    return QDir(LibraryDir()).filePath(name + QStringLiteral(".ply"));
}

QString ReferenceModelLibrary::SourceStepPath(const QString& name)
{
    return QDir(LibraryDir()).filePath(name + QStringLiteral(".source.step"));
}

QString ReferenceModelLibrary::ModelMetadataPath(const QString& name)
{
    return QDir(LibraryDir()).filePath(name + QStringLiteral(".model.json"));
}

bool ReferenceModelLibrary::ResolveConfirmedStepDisplaySource(
    const QString& name,
    const QString& expectedPlySha256,
    QString& sourceStepPath,
    QString& sourceSha256,
    QString& error)
{
    QMutexLocker<QMutex> mutationLock(&LibraryMutationMutex());
    sourceStepPath.clear();
    sourceSha256.clear();
    error.clear();
    if (!IsValidName(name))
    {
        error = QStringLiteral("模型名非法，无法解析 STEP 外显源。");
        return false;
    }
    const QString expectedPly = expectedPlySha256.trimmed().toLower();
    static const QRegularExpression sha256Pattern(QStringLiteral("^[0-9a-f]{64}$"));
    if (!sha256Pattern.match(expectedPly).hasMatch())
    {
        error = QStringLiteral("模板没有有效的计算 PLY SHA-256，禁止显示未绑定的 STEP。");
        return false;
    }

    QLockFile crossProcessLock(QDir(LibraryDir()).filePath(
        QStringLiteral(".reference-model-library.lock")));
    if (!AcquireLibraryMutationLock(crossProcessLock, error)) return false;

    const QString plyPath = ModelPath(name);
    const QString stepPath = SourceStepPath(name);
    const QString metadataPath = ModelMetadataPath(name);
    if (!QFileInfo::exists(plyPath) || !QFileInfo::exists(stepPath)
        || !QFileInfo::exists(metadataPath))
    {
        error = QStringLiteral(
            "该模型没有完整的已确认 STEP、计算 PLY 和追溯元数据；请重新从 STEP 导入。");
        return false;
    }

    QJsonObject metadata;
    if (!ReadJsonObjectFile(metadataPath, metadata, error) || !IsStepMetadata(metadata))
    {
        if (error.isEmpty()) error = QStringLiteral("STEP 模型追溯元数据无效。");
        return false;
    }
    if (!metadata.value(QStringLiteral("dimensionsConfirmed")).toBool(false))
    {
        error = QStringLiteral("STEP 模型尚未完成人工单位/尺寸确认，禁止外显。");
        return false;
    }
    const QString metadataPlySha = metadata.value(QStringLiteral("plySha256"))
        .toString().trimmed().toLower();
    if (metadataPlySha != expectedPly)
    {
        error = QStringLiteral("原始 STEP 追溯记录与模板绑定的计算 PLY 不一致，禁止外显。");
        return false;
    }
    const QString metadataSourceSha = metadata.value(QStringLiteral("sourceSha256"))
        .toString().trimmed().toLower();
    if (!sha256Pattern.match(metadataSourceSha).hasMatch())
    {
        error = QStringLiteral("STEP 追溯记录没有有效的源文件 SHA-256。");
        return false;
    }

    QString hashError;
    const QString actualSourceSha = StableFileSha256(stepPath, hashError);
    if (actualSourceSha.isEmpty() || actualSourceSha != metadataSourceSha)
    {
        error = actualSourceSha.isEmpty()
            ? hashError
            : QStringLiteral("原始 STEP 已变化，与追溯元数据不一致，禁止外显。");
        return false;
    }
    const QString actualPlySha = StableFileSha256(plyPath, hashError);
    if (actualPlySha.isEmpty() || actualPlySha != metadataPlySha)
    {
        error = actualPlySha.isEmpty()
            ? hashError
            : QStringLiteral("内部计算 PLY 已变化，与追溯元数据不一致，禁止外显。");
        return false;
    }

    sourceStepPath = stepPath;
    sourceSha256 = actualSourceSha;
    return true;
}

QStringList ReferenceModelLibrary::ListModels()
{
    QStringList names;
    const QDir d(LibraryDir());
    const QFileInfoList items = d.entryInfoList(QStringList() << QStringLiteral("*.ply"),
                                                QDir::Files, QDir::Name);
    for (const QFileInfo& fi : items)
    {
        const QString name = fi.completeBaseName();
        if (Exists(name)) names << name;
    }
    return names;
}

bool ReferenceModelLibrary::Exists(const QString& name)
{
    if (!IsValidName(name) || !QFileInfo::exists(ModelPath(name))) return false;
    const bool hasSource = QFileInfo::exists(SourceStepPath(name));
    const bool hasMetadata = QFileInfo::exists(ModelMetadataPath(name));
    if (!hasSource && !hasMetadata) return true;  // 原有纯 PLY 模型。
    if (!hasSource || !hasMetadata) return false;

    QJsonObject metadata;
    QString ignoredError;
    return ReadJsonObjectFile(ModelMetadataPath(name), metadata, ignoredError)
        && IsStepMetadata(metadata)
        && metadata.value(QStringLiteral("dimensionsConfirmed")).toBool(false);
}

bool ReferenceModelLibrary::IsValidName(const QString& name)
{
    if (name.isEmpty() || name.length() > 80) return false;
    if (name == QStringLiteral(".") || name == QStringLiteral("..")) return false;
    static const QRegularExpression bad(QStringLiteral("[\\\\/:*?\"<>|\\x00-\\x1f]"));
    return !name.contains(bad);
}

bool ReferenceModelLibrary::ImportFromMesh(const QString& name, const WorkpieceMeshBuilder::Mesh& mesh, QString& error)
{
    QMutexLocker<QMutex> mutationLock(&LibraryMutationMutex());
    QLockFile crossProcessLock(QDir(LibraryDir()).filePath(QStringLiteral(".reference-model-library.lock")));
    if (!AcquireLibraryMutationLock(crossProcessLock, error)) return false;
    if (!IsValidName(name)) { error = QStringLiteral("模型名非法（含路径分隔符或非法字符）"); return false; }
    if (!mesh.IsValid()) { error = QStringLiteral("网格无效（顶点/三角形不足）"); return false; }
    if (HasStepArtifacts(name))
    {
        error = QStringLiteral("同名模型带有STEP源文件和追溯元数据；请先删除旧模型，禁止仅覆盖PLY造成身份不一致。");
        return false;
    }
    return WorkpieceMeshBuilder::SaveMeshPly(ModelPath(name), mesh, error);
}

bool ReferenceModelLibrary::ImportFromFile(const QString& name, const QString& srcPlyPath, QString& error)
{
    QMutexLocker<QMutex> mutationLock(&LibraryMutationMutex());
    QLockFile crossProcessLock(QDir(LibraryDir()).filePath(QStringLiteral(".reference-model-library.lock")));
    if (!AcquireLibraryMutationLock(crossProcessLock, error)) return false;
    if (!IsValidName(name)) { error = QStringLiteral("模型名非法（含路径分隔符或非法字符）"); return false; }
    if (HasStepArtifacts(name))
    {
        error = QStringLiteral("同名模型带有STEP源文件和追溯元数据；请先删除旧模型，禁止仅覆盖PLY造成身份不一致。");
        return false;
    }
    WorkpieceMeshBuilder::Mesh mesh;
    if (!WorkpieceMeshBuilder::LoadMeshPly(srcPlyPath, mesh, error)) return false;  // 先校验是合法网格
    if (!mesh.IsValid()) { error = QStringLiteral("源文件不是有效网格"); return false; }
    return WorkpieceMeshBuilder::SaveMeshPly(ModelPath(name), mesh, error);
}

bool ReferenceModelLibrary::ImportFromStepFile(
    const QString& name,
    const QString& srcStepPath,
    QString& error,
    QString* summary,
    QString* confirmationToken)
{
    QMutexLocker<QMutex> mutationLock(&LibraryMutationMutex());
    error.clear();
    if (summary != nullptr) summary->clear();
    if (confirmationToken != nullptr) confirmationToken->clear();
    if (!IsValidName(name))
    {
        error = QStringLiteral("模型名非法（含路径分隔符或非法字符）");
        return false;
    }
    QLockFile crossProcessLock(QDir(LibraryDir()).filePath(QStringLiteral(".reference-model-library.lock")));
    if (!AcquireLibraryMutationLock(crossProcessLock, error)) return false;

    const QString finalPlyPath = ModelPath(name);
    const QString finalSourcePath = SourceStepPath(name);
    const QString finalMetadataPath = ModelMetadataPath(name);
    if (QFileInfo::exists(finalPlyPath) || QFileInfo::exists(finalSourcePath)
        || QFileInfo::exists(finalMetadataPath))
    {
        error = QStringLiteral(
            "同名模型或上次待确认的 STEP 记录已经存在；程序不会自动覆盖或删除，"
            "请先恢复确认、显式撤销，或更换名称。");
        return false;
    }

    QString hashError;
    const QString firstSourceSha256 = StableFileSha256(srcStepPath, hashError);
    if (firstSourceSha256.isEmpty())
    {
        error = hashError;
        return false;
    }

    StepModelImporter::Options options;
    StepModelImporter::Statistics statistics;
    WorkpieceMeshBuilder::Mesh mesh;
    if (!StepModelImporter::ImportFile(srcStepPath, mesh, error, &statistics, &options))
    {
        return false;
    }

    const QString secondSourceSha256 = StableFileSha256(srcStepPath, hashError);
    if (secondSourceSha256.isEmpty() || secondSourceSha256 != firstSourceSha256)
    {
        error = secondSourceSha256.isEmpty()
            ? hashError
            : QStringLiteral("STEP源文件在转换期间发生变化，已放弃导入结果。");
        return false;
    }

    const QString transactionId = QUuid::createUuid().toString(QUuid::WithoutBraces).toLower();
    const QString stagedPlyPath = finalPlyPath + QStringLiteral(".import-") + transactionId;
    const QString stagedSourcePath = finalSourcePath + QStringLiteral(".import-") + transactionId;
    const QString stagedMetadataPath = finalMetadataPath + QStringLiteral(".import-") + transactionId;
    const auto cleanupStaging = [&]()
    {
        QFile::remove(stagedPlyPath + QStringLiteral(".tmp"));
        QFile::remove(stagedPlyPath);
        QFile::remove(stagedSourcePath);
        QFile::remove(stagedMetadataPath);
    };

    if (!WorkpieceMeshBuilder::SaveMeshPly(stagedPlyPath, mesh, error))
    {
        cleanupStaging();
        return false;
    }
    mesh = WorkpieceMeshBuilder::Mesh();

    WorkpieceMeshBuilder::Mesh verifiedMesh;
    if (!WorkpieceMeshBuilder::LoadMeshPly(stagedPlyPath, verifiedMesh, error))
    {
        error = QStringLiteral("STEP转换网格写后回读失败：%1").arg(error);
        cleanupStaging();
        return false;
    }
    if (verifiedMesh.vertices.size() != statistics.vertexCount
        || verifiedMesh.indices.size() / 3 != statistics.triangleCount)
    {
        error = QStringLiteral("STEP转换网格写后回读的顶点或三角形数量不一致。");
        cleanupStaging();
        return false;
    }
    verifiedMesh = WorkpieceMeshBuilder::Mesh();

    const QString plySha256 = StableFileSha256(stagedPlyPath, hashError);
    if (plySha256.isEmpty())
    {
        error = hashError;
        cleanupStaging();
        return false;
    }
    if (!QFile::copy(srcStepPath, stagedSourcePath))
    {
        error = QStringLiteral("复制STEP源文件到模型库暂存区失败。");
        cleanupStaging();
        return false;
    }
    if (!QFile::setPermissions(
            stagedSourcePath,
            QFile::permissions(stagedSourcePath)
                | QFileDevice::WriteOwner | QFileDevice::WriteUser))
    {
        error = QStringLiteral("无法移除模型库 STEP 副本的只读属性。");
        cleanupStaging();
        return false;
    }
    const QString copiedSourceSha256 = StableFileSha256(stagedSourcePath, hashError);
    if (copiedSourceSha256.isEmpty() || copiedSourceSha256 != firstSourceSha256)
    {
        error = copiedSourceSha256.isEmpty()
            ? hashError
            : QStringLiteral("模型库中的STEP源文件副本与导入源不一致。");
        cleanupStaging();
        return false;
    }

    const Eigen::Vector3d dimensions = statistics.boundsMaxMm - statistics.boundsMinMm;
    QJsonObject triangulation;
    triangulation.insert(QStringLiteral("linearDeflectionMm"), statistics.linearDeflectionMm);
    triangulation.insert(QStringLiteral("angularDeflectionRadians"), options.angularDeflectionRadians);
    triangulation.insert(QStringLiteral("relative"), false);

    QJsonObject bounds;
    bounds.insert(QStringLiteral("minimumMm"), VectorJson(statistics.boundsMinMm));
    bounds.insert(QStringLiteral("maximumMm"), VectorJson(statistics.boundsMaxMm));
    bounds.insert(QStringLiteral("dimensionsMm"), VectorJson(dimensions));

    QJsonObject metadata;
    metadata.insert(QStringLiteral("schemaVersion"), 1);
    metadata.insert(QStringLiteral("recipeVersion"), QStringLiteral("occt-step-mesh-v1"));
    metadata.insert(QStringLiteral("transactionId"), transactionId);
    metadata.insert(QStringLiteral("createdUtc"),
        QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs));
    metadata.insert(QStringLiteral("sourceFileName"), QFileInfo(srcStepPath).fileName());
    metadata.insert(QStringLiteral("sourceSha256"), firstSourceSha256);
    metadata.insert(QStringLiteral("sourceSizeBytes"), static_cast<double>(QFileInfo(stagedSourcePath).size()));
    metadata.insert(QStringLiteral("sourceLengthUnits"), StringListJson(statistics.sourceLengthUnits));
    metadata.insert(QStringLiteral("outputLengthUnit"), QStringLiteral("mm"));
    metadata.insert(QStringLiteral("coordinatePolicy"), QStringLiteral("preserve-source-axis-and-origin"));
    metadata.insert(QStringLiteral("dimensionsConfirmed"), false);
    metadata.insert(QStringLiteral("occtVersion"), statistics.occtVersion);
    metadata.insert(QStringLiteral("triangulation"), triangulation);
    metadata.insert(QStringLiteral("bounds"), bounds);
    metadata.insert(QStringLiteral("sourceFaceCount"), static_cast<double>(statistics.sourceFaceCount));
    metadata.insert(QStringLiteral("meshedFaceCount"), static_cast<double>(statistics.meshedFaceCount));
    metadata.insert(QStringLiteral("vertexCount"), static_cast<double>(statistics.vertexCount));
    metadata.insert(QStringLiteral("triangleCount"), static_cast<double>(statistics.triangleCount));
    metadata.insert(QStringLiteral("skippedDegenerateTriangles"),
        static_cast<double>(statistics.skippedDegenerateTriangles));
    metadata.insert(QStringLiteral("plySha256"), plySha256);

    const QByteArray metadataJson = QJsonDocument(metadata).toJson(QJsonDocument::Indented);
    QSaveFile metadataFile(stagedMetadataPath);
    if (!metadataFile.open(QIODevice::WriteOnly)
        || metadataFile.write(metadataJson) != metadataJson.size()
        || !metadataFile.commit())
    {
        error = QStringLiteral("写入STEP模型追溯元数据失败：%1").arg(metadataFile.errorString());
        cleanupStaging();
        return false;
    }
    QFile metadataVerificationFile(stagedMetadataPath);
    QJsonParseError metadataParseError;
    if (!metadataVerificationFile.open(QIODevice::ReadOnly))
    {
        error = QStringLiteral("STEP模型追溯元数据写后无法读取。");
        cleanupStaging();
        return false;
    }
    const QJsonDocument verifiedMetadata = QJsonDocument::fromJson(
        metadataVerificationFile.readAll(), &metadataParseError);
    metadataVerificationFile.close();
    if (metadataParseError.error != QJsonParseError::NoError
        || !verifiedMetadata.isObject()
        || verifiedMetadata.object().value(QStringLiteral("sourceSha256")).toString()
            != firstSourceSha256
        || verifiedMetadata.object().value(QStringLiteral("plySha256")).toString()
            != plySha256
        || verifiedMetadata.object().value(QStringLiteral("transactionId")).toString()
            != transactionId)
    {
        error = QStringLiteral("STEP模型追溯元数据写后回读校验失败。");
        cleanupStaging();
        return false;
    }

    // PLY 是 ListModels/模板身份的可见提交点，因此最后发布。
    if (!QFile::rename(stagedSourcePath, finalSourcePath))
    {
        error = QStringLiteral("发布STEP源文件失败。");
        cleanupStaging();
        return false;
    }
    if (!QFile::rename(stagedMetadataPath, finalMetadataPath))
    {
        QFile::remove(finalSourcePath);
        error = QStringLiteral("发布STEP追溯元数据失败。");
        cleanupStaging();
        return false;
    }
    if (!QFile::rename(stagedPlyPath, finalPlyPath))
    {
        QFile::remove(finalSourcePath);
        QFile::remove(finalMetadataPath);
        error = QStringLiteral("发布STEP转换网格失败。");
        cleanupStaging();
        return false;
    }

    if (summary != nullptr)
    {
        *summary = QStringLiteral(
            "声明单位 %1；尺寸 %2 × %3 × %4 mm；%5 个曲面，%6 个顶点，%7 个三角形；离散误差 %8 mm。")
            .arg(statistics.sourceLengthUnits.join(QStringLiteral(" / ")))
            .arg(dimensions.x(), 0, 'f', 2)
            .arg(dimensions.y(), 0, 'f', 2)
            .arg(dimensions.z(), 0, 'f', 2)
            .arg(statistics.meshedFaceCount)
            .arg(statistics.vertexCount)
            .arg(statistics.triangleCount)
            .arg(statistics.linearDeflectionMm, 0, 'f', 3);
    }
    if (confirmationToken != nullptr) *confirmationToken = transactionId;
    return true;
}

bool ReferenceModelLibrary::InspectPendingStepImport(
    const QString& name,
    bool& pending,
    bool& canConfirm,
    QString& summary,
    QString& confirmationToken,
    QString& error)
{
    QMutexLocker<QMutex> mutationLock(&LibraryMutationMutex());
    pending = false;
    canConfirm = false;
    summary.clear();
    confirmationToken.clear();
    error.clear();
    if (!IsValidName(name))
    {
        error = QStringLiteral("模型名非法。");
        return false;
    }
    PendingStepState state;
    if (!InspectPendingStepImportUnlocked(name, state, error)) return false;
    pending = state.pending;
    canConfirm = state.canConfirm;
    summary = state.summary;
    confirmationToken = state.confirmationToken;
    return true;
}

bool ReferenceModelLibrary::ConfirmStepImport(
    const QString& name,
    const QString& confirmationToken,
    QString& error)
{
    QMutexLocker<QMutex> mutationLock(&LibraryMutationMutex());
    error.clear();
    if (!IsValidName(name))
    {
        error = QStringLiteral("模型名非法。");
        return false;
    }
    if (confirmationToken.trimmed().isEmpty())
    {
        error = QStringLiteral("STEP 确认事务标识为空，拒绝确认。");
        return false;
    }
    QLockFile crossProcessLock(QDir(LibraryDir()).filePath(QStringLiteral(".reference-model-library.lock")));
    if (!AcquireLibraryMutationLock(crossProcessLock, error)) return false;
    const QString plyPath = ModelPath(name);
    const QString sourcePath = SourceStepPath(name);
    const QString metadataPath = ModelMetadataPath(name);
    if (!QFileInfo::exists(plyPath) || !QFileInfo::exists(sourcePath)
        || !QFileInfo::exists(metadataPath))
    {
        error = QStringLiteral("待确认STEP模型的PLY、源文件或元数据不完整。");
        return false;
    }

    QJsonObject metadata;
    if (!ReadJsonObjectFile(metadataPath, metadata, error) || !IsStepMetadata(metadata))
    {
        if (error.isEmpty()) error = QStringLiteral("STEP模型追溯元数据类型无效。");
        return false;
    }
    if (metadata.value(QStringLiteral("transactionId")).toString() != confirmationToken)
    {
        error = QStringLiteral("待确认 STEP 已不是界面刚才展示的那次导入，拒绝误确认；请重新核对。");
        return false;
    }

    QString hashError;
    const QString sourceSha256 = StableFileSha256(sourcePath, hashError);
    if (sourceSha256.isEmpty()
        || sourceSha256 != metadata.value(QStringLiteral("sourceSha256")).toString())
    {
        error = sourceSha256.isEmpty()
            ? hashError : QStringLiteral("待确认STEP源文件SHA-256与元数据不一致。");
        return false;
    }
    const QString plySha256 = StableFileSha256(plyPath, hashError);
    if (plySha256.isEmpty()
        || plySha256 != metadata.value(QStringLiteral("plySha256")).toString())
    {
        error = plySha256.isEmpty()
            ? hashError : QStringLiteral("待确认STEP网格SHA-256与元数据不一致。");
        return false;
    }
    if (metadata.value(QStringLiteral("dimensionsConfirmed")).toBool(false)) return true;

    metadata.insert(QStringLiteral("dimensionsConfirmed"), true);
    metadata.insert(QStringLiteral("dimensionsConfirmedUtc"),
        QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs));
    const QByteArray confirmedJson = QJsonDocument(metadata).toJson(QJsonDocument::Indented);
    QSaveFile metadataFile(metadataPath);
    metadataFile.setDirectWriteFallback(false);
    if (!metadataFile.open(QIODevice::WriteOnly)
        || metadataFile.write(confirmedJson) != confirmedJson.size()
        || !metadataFile.commit())
    {
        error = QStringLiteral("提交STEP单位/尺寸人工确认失败：%1").arg(metadataFile.errorString());
        return false;
    }

    QJsonObject verifiedMetadata;
    if (!ReadJsonObjectFile(metadataPath, verifiedMetadata, error)
        || !IsStepMetadata(verifiedMetadata)
        || !verifiedMetadata.value(QStringLiteral("dimensionsConfirmed")).toBool(false))
    {
        if (error.isEmpty()) error = QStringLiteral("STEP人工确认写后回读失败。");
        return false;
    }
    return true;
}

bool ReferenceModelLibrary::DiscardPendingStepImport(
    const QString& name,
    const QString& expectedPendingToken,
    QString& error)
{
    QMutexLocker<QMutex> mutationLock(&LibraryMutationMutex());
    error.clear();
    if (!IsValidName(name))
    {
        error = QStringLiteral("模型名非法。");
        return false;
    }
    if (expectedPendingToken.trimmed().isEmpty())
    {
        error = QStringLiteral("待撤销 STEP 的事务/快照标识为空，拒绝删除。");
        return false;
    }
    QLockFile crossProcessLock(QDir(LibraryDir()).filePath(QStringLiteral(".reference-model-library.lock")));
    if (!AcquireLibraryMutationLock(crossProcessLock, error)) return false;

    PendingStepState state;
    if (!InspectPendingStepImportUnlocked(name, state, error)) return false;
    if (!state.pending)
    {
        error = QStringLiteral("没有可撤销的待确认 STEP 记录；已确认模型不会由此接口删除。");
        return false;
    }
    if (state.confirmationToken != expectedPendingToken)
    {
        error = QStringLiteral("待确认 STEP 已在界面核对后发生变化，拒绝误删；请重新检查。");
        return false;
    }

    // 必须先删除可见提交点 PLY；如果失败立即停止，绝不能留下无 sidecar 的未确认 PLY。
    const QString plyPath = ModelPath(name);
    if (!RemoveFileWithRetry(plyPath))
    {
        error = QStringLiteral("无法隐藏待确认 STEP 的 PLY，未删除任何追溯文件：%1").arg(plyPath);
        return false;
    }
    QStringList leftovers;
    for (const QString& path : { SourceStepPath(name), ModelMetadataPath(name) })
        if (!RemoveFileWithRetry(path)) leftovers.append(path);
    if (!leftovers.isEmpty())
    {
        error = QStringLiteral("撤销待确认 STEP 时以下文件删除失败：%1")
            .arg(leftovers.join(QStringLiteral("；")));
        return false;
    }
    return true;
}

bool ReferenceModelLibrary::LoadModel(
    const QString& name,
    WorkpieceMeshBuilder::Mesh& mesh,
    QString& error,
    const WorkpieceMeshBuilder::CancelCallback& cancelRequested)
{
    QMutexLocker<QMutex> mutationLock(&LibraryMutationMutex());
    QLockFile crossProcessLock(QDir(LibraryDir()).filePath(QStringLiteral(".reference-model-library.lock")));
    if (!AcquireLibraryMutationLock(crossProcessLock, error)) return false;
    if (!Exists(name)) { error = QStringLiteral("基准模型不存在：%1").arg(name); return false; }
    if (HasStepArtifacts(name))
    {
        QJsonObject metadata;
        if (!ReadJsonObjectFile(ModelMetadataPath(name), metadata, error)
            || !IsStepMetadata(metadata))
        {
            if (error.isEmpty()) error = QStringLiteral("STEP 模型追溯元数据无效。");
            return false;
        }
        QString hashError;
        const QString sourceHash = StableFileSha256(SourceStepPath(name), hashError);
        if (sourceHash.isEmpty()
            || sourceHash != metadata.value(QStringLiteral("sourceSha256")).toString())
        {
            error = sourceHash.isEmpty()
                ? hashError : QStringLiteral("STEP 源文件已变化，与追溯元数据不一致。");
            return false;
        }
        const QString plyHash = StableFileSha256(ModelPath(name), hashError);
        if (plyHash.isEmpty()
            || plyHash != metadata.value(QStringLiteral("plySha256")).toString())
        {
            error = plyHash.isEmpty()
                ? hashError : QStringLiteral("STEP 转换网格已变化，与追溯元数据不一致。");
            return false;
        }
    }
    return WorkpieceMeshBuilder::LoadMeshPly(ModelPath(name), mesh, error, cancelRequested);
}

bool ReferenceModelLibrary::DeleteModel(const QString& name, QString& error)
{
    QMutexLocker<QMutex> mutationLock(&LibraryMutationMutex());
    QLockFile crossProcessLock(QDir(LibraryDir()).filePath(QStringLiteral(".reference-model-library.lock")));
    if (!AcquireLibraryMutationLock(crossProcessLock, error)) return false;
    if (!Exists(name))
    {
        error = QStringLiteral("基准模型不存在：%1").arg(name);
        return false;
    }
    if (!RemoveFileWithRetry(ModelPath(name))) { error = QStringLiteral("删除失败：%1").arg(name); return false; }
    QStringList leftovers;
    for (const QString& sidecar : { SourceStepPath(name), ModelMetadataPath(name) })
    {
        if (!RemoveFileWithRetry(sidecar)) leftovers.append(sidecar);
    }
    if (!leftovers.isEmpty())
    {
        error = QStringLiteral("模型PLY已删除，但以下STEP追溯文件删除失败：%1")
            .arg(leftovers.join(QStringLiteral("；")));
        return false;
    }
    return true;
}

bool ReferenceModelLibrary::RenameModel(const QString& fromName, const QString& toName, QString& error)
{
    QMutexLocker<QMutex> mutationLock(&LibraryMutationMutex());
    QLockFile crossProcessLock(QDir(LibraryDir()).filePath(QStringLiteral(".reference-model-library.lock")));
    if (!AcquireLibraryMutationLock(crossProcessLock, error)) return false;
    if (!Exists(fromName)) { error = QStringLiteral("源模型不存在：%1").arg(fromName); return false; }
    if (!IsValidName(toName)) { error = QStringLiteral("新名非法"); return false; }
    if (Exists(toName)) { error = QStringLiteral("目标名已存在：%1").arg(toName); return false; }

    const QString fromPly = ModelPath(fromName);
    const QString toPly = ModelPath(toName);
    const QString fromSource = SourceStepPath(fromName);
    const QString toSource = SourceStepPath(toName);
    const QString fromMetadata = ModelMetadataPath(fromName);
    const QString toMetadata = ModelMetadataPath(toName);
    const bool hasSource = QFileInfo::exists(fromSource);
    const bool hasMetadata = QFileInfo::exists(fromMetadata);
    if (!hasSource && !hasMetadata)
    {
        if (!QFile::rename(fromPly, toPly))
        {
            error = QStringLiteral("重命名模型PLY失败");
            return false;
        }
        return true;
    }
    if (!hasSource || !hasMetadata)
    {
        error = QStringLiteral("STEP模型追溯文件不完整，拒绝重命名。");
        return false;
    }
    if (QFileInfo::exists(toPly) || QFileInfo::exists(toSource)
        || QFileInfo::exists(toMetadata))
    {
        error = QStringLiteral("目标名称存在模型或STEP追溯残留，拒绝覆盖。");
        return false;
    }

    QJsonObject metadata;
    if (!ReadJsonObjectFile(fromMetadata, metadata, error)
        || !IsStepMetadata(metadata)
        || !metadata.value(QStringLiteral("dimensionsConfirmed")).toBool(false))
    {
        if (error.isEmpty()) error = QStringLiteral("STEP模型尚未确认或元数据无效。");
        return false;
    }

    const QString transactionId = QUuid::createUuid().toString(QUuid::WithoutBraces).toLower();
    const QString stagedPly = toPly + QStringLiteral(".rename-") + transactionId;
    const auto cleanupTarget = [&]()
    {
        QFile::remove(stagedPly);
        QFile::remove(toPly);
        QFile::remove(toSource);
        QFile::remove(toMetadata);
    };
    if (!QFile::copy(fromSource, toSource)
        || !QFile::copy(fromMetadata, toMetadata)
        || !QFile::copy(fromPly, stagedPly))
    {
        cleanupTarget();
        error = QStringLiteral("复制STEP模型重命名事务文件失败。");
        return false;
    }
    if (!QFile::setPermissions(
            toSource,
            QFile::permissions(toSource)
                | QFileDevice::WriteOwner | QFileDevice::WriteUser))
    {
        cleanupTarget();
        error = QStringLiteral("无法移除重命名后 STEP 副本的只读属性。");
        return false;
    }

    QString hashError;
    const QString copiedSourceHash = StableFileSha256(toSource, hashError);
    const QString copiedPlyHash = StableFileSha256(stagedPly, hashError);
    WorkpieceMeshBuilder::Mesh verifiedMesh;
    if (copiedSourceHash != metadata.value(QStringLiteral("sourceSha256")).toString()
        || copiedPlyHash != metadata.value(QStringLiteral("plySha256")).toString()
        || !WorkpieceMeshBuilder::LoadMeshPly(stagedPly, verifiedMesh, hashError))
    {
        cleanupTarget();
        error = QStringLiteral("STEP模型重命名副本校验失败：%1").arg(hashError);
        return false;
    }
    verifiedMesh = WorkpieceMeshBuilder::Mesh();
    if (!QFile::rename(stagedPly, toPly))
    {
        cleanupTarget();
        error = QStringLiteral("发布重命名后的STEP模型失败。");
        return false;
    }

    // 新名称完整可见后再隐藏旧名称；任意崩溃点最多留下两个完整、同哈希的模型，
    // 不会出现丢失 provenance 的旧纯 PLY 被重新公开。
    if (!RemoveFileWithRetry(fromPly))
    {
        cleanupTarget();
        error = QStringLiteral("新模型已验证，但无法移除旧模型；已尝试回滚新名称。");
        return false;
    }
    const bool sourceRemoved = RemoveFileWithRetry(fromSource);
    const bool metadataRemoved = RemoveFileWithRetry(fromMetadata);
    if (!sourceRemoved || !metadataRemoved)
    {
        // 旧 PLY 已隐藏，残留 sidecar 不会进入模型列表；再次使用旧名导入时会自动恢复。
        error = QStringLiteral("重命名成功，但旧名称下存在可恢复的STEP追溯残留。");
    }
    return true;
}
