#include "AppPaths.h"
#include "RobotCollisionEnvelopeStore.h"

#include <QCoreApplication>
#include <QCryptographicHash>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTemporaryDir>
#include <QTextStream>

#include <cmath>

namespace
{
int Fail(const QString& message)
{
    QTextStream(stderr) << "FAIL: " << message << Qt::endl;
    return 1;
}

bool Require(bool condition, const QString& message)
{
    if (!condition) QTextStream(stderr) << "FAIL: " << message << Qt::endl;
    return condition;
}

bool WriteBytes(const QString& path, const QByteArray& bytes)
{
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) return false;
    const bool ok = file.write(bytes) == bytes.size();
    file.close();
    return ok;
}

QByteArray ReadBytes(const QString& path)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) return QByteArray();
    return file.readAll();
}

RobotCadAssemblyLoader::Result Fixture()
{
    RobotCadAssemblyLoader::Result result;
    result.statistics.sourceSha256 = QStringLiteral(
        "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef");
    result.statistics.sourceLengthUnits = { QStringLiteral("millimetre") };
    for (int index = 0; index <= 6; ++index)
    {
        RobotCadAssemblyLoader::ComponentStatistics component;
        component.assemblyPath = QStringLiteral("Robot/J%1").arg(index);
        component.instanceName = QStringLiteral("instance J%1").arg(index);
        component.productName = QStringLiteral("SA10-J%1").arg(index);
        component.jointIndex = index;
        component.included = true;
        component.boundsMm.valid = true;
        component.boundsMm.minimumMm = Eigen::Vector3d(
            -100.0 - index, -50.0 + index, -30.0 - 2.0 * index);
        component.boundsMm.maximumMm = Eigen::Vector3d(
            100.0 + index, 120.0 + 10.0 * index, 80.0 + index);
        result.statistics.components.append(component);
    }
    RobotCadAssemblyLoader::ComponentStatistics workspace;
    workspace.assemblyPath = QStringLiteral("Robot/3D workspace");
    workspace.jointIndex = -1;
    workspace.included = false;
    workspace.boundsMm.valid = true;
    workspace.boundsMm.minimumMm = Eigen::Vector3d::Constant(-100000.0);
    workspace.boundsMm.maximumMm = Eigen::Vector3d::Constant(100000.0);
    result.statistics.components.append(workspace);
    result.base.valid = true;
    result.base.sourceUp = Eigen::Vector3d::UnitY();
    result.base.minimumYmm = result.statistics.components.at(0).boundsMm.minimumMm.y();
    result.base.conservativeBaseCenterMm = Eigen::Vector3d(0.0, -50.0, 0.0);
    result.base.j0BoundsMm = result.statistics.components.at(0).boundsMm;
    result.statistics.assemblyBoundsMm.valid = true;
    result.statistics.assemblyBoundsMm.minimumMm =
        result.statistics.components.at(0).boundsMm.minimumMm;
    result.statistics.assemblyBoundsMm.maximumMm =
        result.statistics.components.at(0).boundsMm.maximumMm;
    for (int index = 1; index <= 6; ++index)
    {
        result.statistics.assemblyBoundsMm.minimumMm =
            result.statistics.assemblyBoundsMm.minimumMm.cwiseMin(
                result.statistics.components.at(index).boundsMm.minimumMm);
        result.statistics.assemblyBoundsMm.maximumMm =
            result.statistics.assemblyBoundsMm.maximumMm.cwiseMax(
                result.statistics.components.at(index).boundsMm.maximumMm);
    }
    return result;
}

bool IsHashFileName(const QString& name)
{
    const QString suffix = QStringLiteral(".robot-aabb.json");
    if (name.size() != 64 + suffix.size() || !name.endsWith(suffix)) return false;
    for (int index = 0; index < 64; ++index)
    {
        const QChar ch = name.at(index);
        if (!((ch >= QLatin1Char('0') && ch <= QLatin1Char('9'))
              || (ch >= QLatin1Char('a') && ch <= QLatin1Char('f'))))
            return false;
    }
    return true;
}
}

int main(int argc, char* argv[])
{
    QCoreApplication application(argc, argv);
    if (application.arguments().contains(QStringLiteral("--preinit-probe")))
    {
        RobotCollisionEnvelopeStore::EnvelopeSet envelope;
        RobotCollisionEnvelopeStore::StoredAsset asset;
        QString error;
        bool missing = true;
        const bool rejected = !RobotCollisionEnvelopeStore::Load(
            Fixture().statistics.sourceSha256, {}, envelope, asset, error);
        QString missingError;
        const bool missingQueryRejected = !RobotCollisionEnvelopeStore::IsMissing(
            Fixture().statistics.sourceSha256, {}, missing, missingError);
        if (!Require(!AppPaths::IsInitialized()
                         && RobotCollisionEnvelopeStore::StoreDirectory().isEmpty()
                         && rejected && envelope.joints.isEmpty()
                         && asset.storedFileName.isEmpty() && !error.isEmpty()
                         && missingQueryRejected && !missing && !missingError.isEmpty(),
                     QStringLiteral("pre-initialization access did not fail closed")))
            return 1;
        QTextStream(stdout) << "PASS: collision envelope pre-init fails closed" << Qt::endl;
        return 0;
    }

    QTemporaryDir dataRoot;
    if (!dataRoot.isValid()) return Fail(QStringLiteral("cannot create temporary data root"));
    QString error;
    if (!AppPaths::Initialize(
            QStringList{ application.applicationFilePath(),
                         QStringLiteral("--data-root"), dataRoot.path() }, &error))
        return Fail(QStringLiteral("cannot initialize AppPaths: %1").arg(error));

    RobotCadAssemblyLoader::Result source = Fixture();
    RobotCollisionEnvelopeStore::GenerationParameters parameters;
    parameters.safetyMarginMm = 25.0;
    RobotCollisionEnvelopeStore::EnvelopeSet generated;
    if (!Require(RobotCollisionEnvelopeStore::Generate(
                     source, parameters, generated, error),
                 QStringLiteral("generation failed: %1").arg(error))
        || !Require(generated.joints.size() == 7
                        && generated.safetyMarginMicrometres == 25000,
                    QStringLiteral("generated identity/joint count is wrong")))
        return 1;

    for (int index = 0; index <= 6; ++index)
    {
        const auto& component = source.statistics.components.at(index);
        const auto& joint = generated.joints.at(index);
        for (int axis = 0; axis < 3; ++axis)
        {
            const double expectedMinimum = (index == 0 && axis == 1)
                ? source.base.minimumYmm
                : component.boundsMm.minimumMm[axis] - 25.0;
            if (!Require(joint.collisionBoundsMm.minimumMm[axis]
                                <= component.boundsMm.minimumMm[axis]
                            && joint.collisionBoundsMm.maximumMm[axis]
                                >= component.boundsMm.maximumMm[axis]
                            && std::abs(joint.collisionBoundsMm.minimumMm[axis]
                                        - expectedMinimum)
                                < 1.0e-12
                            && std::abs(joint.collisionBoundsMm.maximumMm[axis]
                                        - (component.boundsMm.maximumMm[axis] + 25.0))
                                < 1.0e-12,
                        QStringLiteral("J%1 AABB does not conservatively cover source bounds")
                            .arg(index)))
                return 1;
        }
    }
    if (!Require(generated.coordinateFrame == QStringLiteral("sourceAssembly")
                     && generated.staticOnly
                     && generated.sourceUp.isApprox(Eigen::Vector3d::UnitY())
                     && generated.baseMinimumYmm == source.base.minimumYmm
                     && generated.joints.at(0).collisionBoundsMm.minimumMm.y()
                        == source.base.minimumYmm
                     && generated.overallExpandedBoundsMm.minimumMm.y()
                        <= generated.overallSourceBoundsMm.minimumMm.y(),
                 QStringLiteral("coordinate/base/overall metadata is invalid")))
        return 1;
    if (!Require(RobotCollisionEnvelopeStore::ValidateForUse(generated, error),
                 QStringLiteral("generated envelope failed public validation: %1")
                    .arg(error)))
        return 1;

    bool profileMissing = false;
    if (!Require(RobotCollisionEnvelopeStore::IsMissing(
                     source.statistics.sourceSha256,
                     parameters,
                     profileMissing,
                     error)
                    && profileMissing,
                 QStringLiteral("absent deterministic profile was not reported missing: %1")
                    .arg(error)))
        return 1;

    RobotCollisionEnvelopeStore::StoredAsset saved;
    if (!Require(RobotCollisionEnvelopeStore::Persist(generated, saved, error),
                 QStringLiteral("atomic persist failed: %1").arg(error))
        || !Require(IsHashFileName(saved.storedFileName)
                        && saved.profileKeySha256 == generated.profileKeySha256
                        && saved.sourceStepSha256 == generated.sourceStepSha256
                        && saved.sizeBytes > 0,
                    QStringLiteral("persisted asset metadata/path is invalid")))
        return 1;
    if (!Require(RobotCollisionEnvelopeStore::IsMissing(
                     source.statistics.sourceSha256,
                     parameters,
                     profileMissing,
                     error)
                    && !profileMissing,
                 QStringLiteral("persisted deterministic profile was reported missing: %1")
                    .arg(error)))
        return 1;

    RobotCollisionEnvelopeStore::EnvelopeSet loaded;
    RobotCollisionEnvelopeStore::StoredAsset resolved;
    if (!Require(RobotCollisionEnvelopeStore::Load(
                     source.statistics.sourceSha256, parameters, loaded, resolved, error),
                 QStringLiteral("round-trip load failed: %1").arg(error))
        || !Require(loaded.payloadSha256 == generated.payloadSha256
                        && loaded.joints.size() == 7
                        && resolved.storedFileName == saved.storedFileName,
                    QStringLiteral("round-trip identity/content changed")))
        return 1;

    // 非精确到 0.001 mm 的余量、越界余量、重复/缺失 J 都必须关闭失败。
    RobotCollisionEnvelopeStore::GenerationParameters badMargin;
    badMargin.safetyMarginMm = 1.0004;
    RobotCollisionEnvelopeStore::EnvelopeSet rejected;
    if (!Require(!RobotCollisionEnvelopeStore::Generate(
                     source, badMargin, rejected, error) && rejected.joints.isEmpty(),
                 QStringLiteral("over-precision margin was accepted")))
        return 1;
    badMargin.safetyMarginMm = 501.0;
    if (!Require(!RobotCollisionEnvelopeStore::Generate(
                     source, badMargin, rejected, error),
                 QStringLiteral("out-of-range margin was accepted")))
        return 1;
    RobotCadAssemblyLoader::Result duplicate = source;
    duplicate.statistics.components.append(duplicate.statistics.components.first());
    if (!Require(!RobotCollisionEnvelopeStore::Generate(
                     duplicate, parameters, rejected, error),
                 QStringLiteral("duplicate J0 was accepted")))
        return 1;
    RobotCadAssemblyLoader::Result flat = source;
    flat.statistics.components[0].boundsMm.maximumMm.x() =
        flat.statistics.components[0].boundsMm.minimumMm.x();
    flat.base.j0BoundsMm = flat.statistics.components[0].boundsMm;
    RobotCollisionEnvelopeStore::GenerationParameters zeroMargin;
    zeroMargin.safetyMarginMm = 0.0;
    if (!Require(!RobotCollisionEnvelopeStore::Generate(
                     flat, zeroMargin, rejected, error),
                 QStringLiteral("zero-volume collision AABB was accepted")))
        return 1;

    const QString storedPath = QDir(RobotCollisionEnvelopeStore::StoreDirectory())
        .filePath(saved.storedFileName);
    const QByteArray validBytes = ReadBytes(storedPath);
    if (validBytes.isEmpty()) return Fail(QStringLiteral("cannot snapshot envelope JSON"));

    // 服务器下载入口必须先完整解析任意 staging 文件，再按内容寻址原子落库并回读。
    const QString downloadedPath =
        dataRoot.filePath(QStringLiteral("downloaded-envelope.json"));
    if (!WriteBytes(downloadedPath, validBytes) || !QFile::remove(storedPath))
        return Fail(QStringLiteral("cannot prepare downloaded-envelope import"));
    RobotCollisionEnvelopeStore::StoredAsset importedAsset;
    RobotCollisionEnvelopeStore::EnvelopeSet importedEnvelope;
    if (!Require(
            RobotCollisionEnvelopeStore::ImportFile(
                downloadedPath, importedAsset, importedEnvelope, error)
                && importedAsset.profileKeySha256 == saved.profileKeySha256
                && importedAsset.sourceStepSha256 == saved.sourceStepSha256
                && importedAsset.storedFileName == saved.storedFileName
                && importedEnvelope.payloadSha256 == generated.payloadSha256
                && ReadBytes(storedPath) == validBytes,
            QStringLiteral("downloaded envelope import did not validate/persist/read back: %1")
                .arg(error)))
        return 1;
    QJsonObject invalidImportRoot =
        QJsonDocument::fromJson(validBytes).object();
    invalidImportRoot.insert(QStringLiteral("unexpected"), true);
    if (!WriteBytes(
            downloadedPath,
            QJsonDocument(invalidImportRoot).toJson(QJsonDocument::Compact))
        || !Require(
            !RobotCollisionEnvelopeStore::ImportFile(
                downloadedPath, importedAsset, importedEnvelope, error)
                && importedAsset.storedFileName.isEmpty()
                && importedEnvelope.joints.isEmpty()
                && ReadBytes(storedPath) == validBytes,
            QStringLiteral("invalid downloaded envelope was accepted or changed the store")))
        return 1;

    // schema 不要求数组物理顺序；J0 基座校验必须按 jointIndex 查找。
    QJsonObject reorderedRoot = QJsonDocument::fromJson(validBytes).object();
    QJsonArray reorderedJoints = reorderedRoot.value(QStringLiteral("joints")).toArray();
    QJsonArray reversedJoints;
    for (qsizetype index = reorderedJoints.size(); index > 0; --index)
        reversedJoints.append(reorderedJoints.at(index - 1));
    reorderedRoot.insert(QStringLiteral("joints"), reversedJoints);
    reorderedRoot.remove(QStringLiteral("payloadSha256"));
    const QString reorderedPayloadSha = QString::fromLatin1(
        QCryptographicHash::hash(
            QJsonDocument(reorderedRoot).toJson(QJsonDocument::Compact),
            QCryptographicHash::Sha256).toHex());
    reorderedRoot.insert(QStringLiteral("payloadSha256"), reorderedPayloadSha);
    if (!WriteBytes(storedPath, QJsonDocument(reorderedRoot).toJson(QJsonDocument::Compact)))
        return Fail(QStringLiteral("cannot write reordered-joints envelope"));
    RobotCollisionEnvelopeStore::StoredAsset reorderedAsset = saved;
    reorderedAsset.sizeBytes = 0;
    if (!Require(RobotCollisionEnvelopeStore::LoadAsset(reorderedAsset, loaded, error)
                    && loaded.joints.first().jointIndex == 6,
                 QStringLiteral("valid reordered J0-J6 envelope was rejected: %1").arg(error)))
        return 1;
    if (!WriteBytes(storedPath, validBytes))
        return Fail(QStringLiteral("cannot restore JSON after reordered-joints probe"));

    // 未知字段即违反严格 schema；输出必须清空。
    QJsonDocument strictDocument = QJsonDocument::fromJson(validBytes);
    QJsonObject strictRoot = strictDocument.object();
    strictRoot.insert(QStringLiteral("unexpected"), true);
    if (!WriteBytes(storedPath, QJsonDocument(strictRoot).toJson(QJsonDocument::Compact)))
        return Fail(QStringLiteral("cannot write strict-schema tamper"));
    loaded = generated;
    if (!Require(!RobotCollisionEnvelopeStore::LoadAsset(saved, loaded, error)
                     && loaded.joints.isEmpty(),
                 QStringLiteral("unknown JSON field was accepted or stale output leaked")))
        return 1;

    // 有效 schema 内的数值篡改也必须被 payload 哈希/覆盖关系拒绝。
    if (!WriteBytes(storedPath, validBytes)) return Fail(QStringLiteral("cannot restore JSON"));
    QJsonObject tampered = QJsonDocument::fromJson(validBytes).object();
    QJsonArray joints = tampered.value(QStringLiteral("joints")).toArray();
    QJsonObject j0 = joints.at(0).toObject();
    QJsonObject bounds = j0.value(QStringLiteral("collisionBoundsMm")).toObject();
    QJsonArray maximum = bounds.value(QStringLiteral("maximumMm")).toArray();
    maximum[0] = maximum.at(0).toDouble() - 100.0;
    bounds.insert(QStringLiteral("maximumMm"), maximum);
    j0.insert(QStringLiteral("collisionBoundsMm"), bounds);
    joints[0] = j0;
    tampered.insert(QStringLiteral("joints"), joints);
    if (!WriteBytes(storedPath, QJsonDocument(tampered).toJson(QJsonDocument::Compact)))
        return Fail(QStringLiteral("cannot write payload tamper"));
    if (!Require(!RobotCollisionEnvelopeStore::LoadAsset(saved, loaded, error)
                     && loaded.joints.isEmpty(),
                 QStringLiteral("tampered collision bounds were accepted")))
        return 1;
    if (!Require(RobotCollisionEnvelopeStore::IsMissing(
                     source.statistics.sourceSha256,
                     parameters,
                     profileMissing,
                     error)
                    && !profileMissing,
                 QStringLiteral("existing tampered JSON was misclassified as missing")))
        return 1;

    // 上层资产记录不能传入路径或替换 STEP 身份。
    if (!WriteBytes(storedPath, validBytes)) return Fail(QStringLiteral("cannot restore JSON twice"));
    RobotCollisionEnvelopeStore::StoredAsset malicious = saved;
    malicious.storedFileName = QStringLiteral("../escape.robot-aabb.json");
    if (!Require(!RobotCollisionEnvelopeStore::LoadAsset(malicious, loaded, error),
                 QStringLiteral("path traversal asset was accepted")))
        return 1;
    malicious = saved;
    malicious.sourceStepSha256 = QString(64, QLatin1Char('f'));
    if (!Require(!RobotCollisionEnvelopeStore::LoadAsset(malicious, loaded, error),
                 QStringLiteral("mismatched source STEP SHA was accepted")))
        return 1;

    // 普通文件路径被目录占用时，读取必须关闭失败且不得越界。
    if (!QFile::remove(storedPath) || !QDir().mkpath(storedPath))
        return Fail(QStringLiteral("cannot prepare non-file path probe"));
    loaded = generated;
    if (!Require(!RobotCollisionEnvelopeStore::LoadAsset(saved, loaded, error)
                     && loaded.joints.isEmpty(),
                 QStringLiteral("directory at envelope path was accepted")))
        return 1;
    if (!Require(!RobotCollisionEnvelopeStore::IsMissing(
                     source.statistics.sourceSha256,
                     parameters,
                     profileMissing,
                     error),
                 QStringLiteral("non-file profile path was misclassified as missing")))
        return 1;

    QTextStream(stdout)
        << "PASS: J0-J6 conservative source-AABB generation, margin, atomic round-trip, "
           "strict-schema/tamper/SHA/path fail-closed tests"
        << Qt::endl;
    return 0;
}
