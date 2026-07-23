#include "AppPaths.h"
#include "Const.h"
#include "RobotModelCatalogStore.h"

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

QByteArray StepFixture(const QByteArray& identity)
{
    return QByteArray("ISO-10303-21;\nHEADER;\n"
                      "FILE_DESCRIPTION(('robot model catalog test'),'2;1');\n"
                      "FILE_NAME('")
        + identity
        + QByteArray("','','',(''),(''),'','');\n"
                     "FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));\nENDSEC;\nDATA;\n/* ")
        + identity
        + QByteArray(" */\nENDSEC;\nEND-ISO-10303-21;\n");
}

RobotCadAssemblyLoader::Result RobotFixture(const QString& sourceSha256)
{
    RobotCadAssemblyLoader::Result result;
    result.statistics.sourceSha256 = sourceSha256;
    result.statistics.sourceLengthUnits = { QStringLiteral("mm") };
    for (int index = 0; index <= 6; ++index)
    {
        RobotCadAssemblyLoader::ComponentStatistics component;
        component.assemblyPath = QStringLiteral("Robot/J%1").arg(index);
        component.instanceName = QStringLiteral("J%1 instance").arg(index);
        component.productName = QStringLiteral("J%1").arg(index);
        component.jointIndex = index;
        component.included = true;
        component.boundsMm.valid = true;
        component.boundsMm.minimumMm = Eigen::Vector3d(
            -100.0 - index, -40.0 + index, -70.0 - index);
        component.boundsMm.maximumMm = Eigen::Vector3d(
            100.0 + index, 150.0 + index * 20.0, 80.0 + index);
        result.statistics.components.append(component);
    }
    result.base.valid = true;
    result.base.sourceUp = Eigen::Vector3d::UnitY();
    result.base.j0BoundsMm = result.statistics.components.first().boundsMm;
    result.base.minimumYmm = result.base.j0BoundsMm.minimumMm.y();
    result.base.conservativeBaseCenterMm = Eigen::Vector3d(
        0.0, result.base.minimumYmm, 0.0);
    result.statistics.assemblyBoundsMm.valid = true;
    result.statistics.assemblyBoundsMm.minimumMm =
        result.statistics.components.first().boundsMm.minimumMm;
    result.statistics.assemblyBoundsMm.maximumMm =
        result.statistics.components.first().boundsMm.maximumMm;
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

bool CreateAssets(
    const QString& sourcePath,
    TheoreticalRobotModelStore::Asset& step,
    RobotCollisionEnvelopeStore::StoredAsset& collision,
    RobotCollisionEnvelopeStore::EnvelopeSet& envelope,
    QString& error,
    bool activate = false)
{
    if (!TheoreticalRobotModelStore::ImportStepFile(
            sourcePath, step, error, activate))
        return false;
    RobotCadAssemblyLoader::Result fixture = RobotFixture(step.sha256);
    RobotCollisionEnvelopeStore::GenerationParameters parameters;
    return RobotCollisionEnvelopeStore::Generate(
               fixture, parameters, envelope, error)
        && RobotCollisionEnvelopeStore::Persist(envelope, collision, error);
}

QString PayloadSha(const QJsonObject& objectWithoutHash)
{
    return QString::fromLatin1(QCryptographicHash::hash(
        QJsonDocument(objectWithoutHash).toJson(QJsonDocument::Compact),
        QCryptographicHash::Sha256).toHex());
}

QJsonObject WithUpdatedPayloadSha(QJsonObject root)
{
    root.remove(QStringLiteral("payloadSha256"));
    root.insert(QStringLiteral("payloadSha256"), PayloadSha(root));
    return root;
}

int RunPreinitProbe()
{
    QList<RobotModelCatalogStore::ModelRecord> models{
        RobotModelCatalogStore::ModelRecord{ QStringLiteral("stale") }
    };
    RobotModelCatalogStore::Eligibility eligibility;
    eligibility.eligible = true;
    QString error;
    if (!Require(!AppPaths::IsInitialized(),
                 QStringLiteral("AppPaths unexpectedly initialized"))
        || !Require(RobotModelCatalogStore::StoreDirectory().isEmpty()
                        && RobotModelCatalogStore::CatalogFilePath().isEmpty(),
                    QStringLiteral("catalog exposed paths before AppPaths init"))
        || !Require(!RobotModelCatalogStore::ListModels(models, error)
                        && models.isEmpty() && !error.isEmpty(),
                    QStringLiteral("preinit ListModels did not fail closed"))
        || !Require(!RobotModelCatalogStore::ResolveModelEligibility(
                        QStringLiteral("step.sa10-2000h"), ROBOT_TYPE_STEP,
                        eligibility, error)
                        && !eligibility.eligible && !error.isEmpty(),
                    QStringLiteral("preinit eligibility did not fail closed")))
        return 1;
    QTextStream(stdout) << "PASS: robot model catalog preinit fails closed"
                        << Qt::endl;
    return 0;
}

int RunRealSa10Bootstrap(
    QCoreApplication& application,
    const QString& stepPath,
    const QString& envelopePath)
{
    QTemporaryDir dataRoot;
    if (!dataRoot.isValid()) return Fail(QStringLiteral("cannot create bootstrap root"));
    QString error;
    if (!AppPaths::Initialize(
            { application.applicationFilePath(), QStringLiteral("--data-root"),
              dataRoot.path() }, &error))
        return Fail(QStringLiteral("bootstrap AppPaths init failed: %1").arg(error));

    TheoreticalRobotModelStore::Asset imported;
    if (!TheoreticalRobotModelStore::ImportStepFile(
            stepPath, imported, error, true))
        return Fail(QStringLiteral("bootstrap real STEP import failed: %1").arg(error));
    const QString destination = QDir(RobotModelCatalogStore::StoreDirectory())
        .filePath(QFileInfo(envelopePath).fileName());
    if (!QFile::copy(envelopePath, destination))
        return Fail(QStringLiteral("cannot stage real SA10 envelope"));

    RobotModelCatalogStore::ModelRecord model;
    bool registered = false;
    if (!RobotModelCatalogStore::BootstrapVerifiedSa10FromLegacy(
            model, registered, error)
        || !registered || model.modelId != QStringLiteral("step.sa10-2000h")
        || model.sourceStep.sha256 != imported.sha256)
        return Fail(QStringLiteral("real SA10 bootstrap failed: %1").arg(error));
    RobotModelCatalogStore::ModelRecord repeated;
    registered = false;
    if (!RobotModelCatalogStore::BootstrapVerifiedSa10FromLegacy(
            repeated, registered, error)
        || !registered || repeated.registeredUtc != model.registeredUtc)
        return Fail(QStringLiteral("idempotent SA10 bootstrap failed: %1").arg(error));
    QList<RobotModelCatalogStore::ModelRecord> models;
    if (!RobotModelCatalogStore::ListModels(models, error) || models.size() != 1)
        return Fail(QStringLiteral("real SA10 bootstrap list failed: %1").arg(error));
    QTextStream(stdout) << "PASS: real SA10 legacy assets bootstrapped unbound"
                        << Qt::endl;
    return 0;
}
}

int main(int argc, char* argv[])
{
    QCoreApplication application(argc, argv);
    if (application.arguments().contains(QStringLiteral("--preinit-probe")))
        return RunPreinitProbe();
    const int realIndex = application.arguments().indexOf(
        QStringLiteral("--real-sa10-bootstrap"));
    if (realIndex >= 0)
    {
        if (realIndex + 2 >= application.arguments().size())
            return Fail(QStringLiteral("real bootstrap needs STEP and envelope paths"));
        return RunRealSa10Bootstrap(
            application, application.arguments().at(realIndex + 1),
            application.arguments().at(realIndex + 2));
    }

    QTemporaryDir sourceRoot;
    QTemporaryDir dataRoot;
    if (!sourceRoot.isValid() || !dataRoot.isValid())
        return Fail(QStringLiteral("cannot create temporary roots"));
    QString error;
    if (!AppPaths::Initialize(
            { application.applicationFilePath(), QStringLiteral("--data-root"),
              dataRoot.path() }, &error))
        return Fail(QStringLiteral("AppPaths init failed: %1").arg(error));

    QList<RobotModelCatalogStore::ModelRecord> models;
    if (!Require(RobotModelCatalogStore::ListModels(models, error) && models.isEmpty(),
                 QStringLiteral("new catalog is not empty: %1").arg(error)))
        return 1;
    RobotModelCatalogStore::ModelRecord skipped;
    bool bootstrapped = true;
    if (!Require(RobotModelCatalogStore::BootstrapVerifiedSa10FromLegacy(
                     skipped, bootstrapped, error)
                     && !bootstrapped && skipped.modelId.isEmpty(),
                 QStringLiteral("non-SA10/empty legacy active was not skipped safely: %1")
                     .arg(error)))
        return 1;

    const QString firstSource = sourceRoot.filePath(QStringLiteral("SA10 fixture.step"));
    if (!WriteBytes(firstSource, StepFixture("model-one")))
        return Fail(QStringLiteral("cannot write first STEP"));
    TheoreticalRobotModelStore::Asset firstStep;
    RobotCollisionEnvelopeStore::StoredAsset firstCollision;
    RobotCollisionEnvelopeStore::EnvelopeSet firstEnvelope;
    if (!CreateAssets(
            firstSource, firstStep, firstCollision, firstEnvelope, error))
        return Fail(QStringLiteral("cannot create first assets: %1").arg(error));

    RobotModelCatalogStore::ModelRecord firstModel;
    if (!Require(RobotModelCatalogStore::RegisterValidatedModel(
                     QStringLiteral("step.sa10-2000h"),
                     QStringLiteral("新时达 SA10-2000H"),
                     QStringLiteral("step.sa10-2000h.scene-v1"),
                     ROBOT_TYPE_STEP,
                     firstStep,
                     firstCollision,
                     firstModel,
                     error),
                 QStringLiteral("first registration failed: %1").arg(error))
        || !Require(firstModel.modelId == QStringLiteral("step.sa10-2000h")
                        && firstModel.sourceStep.sha256 == firstStep.sha256
                        && firstModel.collisionPayloadSha256
                            == firstEnvelope.payloadSha256,
                    QStringLiteral("registered model identity is wrong")))
        return 1;

    const QString catalogPath = RobotModelCatalogStore::CatalogFilePath();
    const QByteArray validCatalogBytes = ReadBytes(catalogPath);
    const QJsonObject validRoot = QJsonDocument::fromJson(validCatalogBytes).object();
    if (!Require(QFileInfo(catalogPath).fileName()
                        == QStringLiteral("robot-model-catalog.json")
                    && validRoot.value(QStringLiteral("models")).toArray().size() == 1
                    && !validRoot.contains(QStringLiteral("bindings")),
                QStringLiteral("catalog path/schema unexpectedly stores bindings")))
        return 1;

    RobotModelCatalogStore::Eligibility eligibility;
    if (!Require(RobotModelCatalogStore::ResolveModelEligibility(
                     QStringLiteral("step.sa10-2000h"), ROBOT_TYPE_STEP,
                     eligibility, error)
                     && eligibility.eligible
                     && eligibility.collisionEnvelope.joints.size() == 7,
                 QStringLiteral("registered model is not eligible: %1 / %2")
                    .arg(error, eligibility.reason)))
        return 1;

    // catalog 的 payload SHA 可由同一用户重算，它不是授权签名。即使攻击者把
    // 记录改到结构和 payload 完全自洽，代码内未登记的型号、适配器和类型也必须
    // 在访问资产之前被正常门禁拒绝。
    const auto expectTrustRejection = [&catalogPath, &validCatalogBytes](
        QJsonObject root,
        const QString& requestedModelId,
        int actualRobotType,
        const QString& probeName)
    {
        const bool wrote = WriteBytes(
            catalogPath,
            QJsonDocument(WithUpdatedPayloadSha(root)).toJson());
        RobotModelCatalogStore::Eligibility tamperedEligibility;
        QString tamperedError;
        const bool resolved = wrote
            && RobotModelCatalogStore::ResolveModelEligibility(
                requestedModelId, actualRobotType,
                tamperedEligibility, tamperedError);
        const bool rejected = resolved && !tamperedEligibility.eligible
            && tamperedEligibility.reason.contains(QStringLiteral("未受信"));
        const bool restored = WriteBytes(catalogPath, validCatalogBytes);
        return Require(
            wrote && rejected && restored,
            QStringLiteral("self-consistent %1 catalog tamper was not rejected: %2 / %3")
                .arg(probeName, tamperedError, tamperedEligibility.reason));
    };

    QJsonObject modelIdTamper = validRoot;
    QJsonArray modelIdModels = modelIdTamper.value(QStringLiteral("models")).toArray();
    QJsonObject modelIdModel = modelIdModels.first().toObject();
    modelIdModel.insert(QStringLiteral("modelId"), QStringLiteral("step.attacker"));
    modelIdModels[0] = modelIdModel;
    modelIdTamper.insert(QStringLiteral("models"), modelIdModels);
    if (!expectTrustRejection(
            modelIdTamper, QStringLiteral("step.attacker"), ROBOT_TYPE_STEP,
            QStringLiteral("modelId")))
        return 1;

    QJsonObject adapterTamper = validRoot;
    QJsonArray adapterModels = adapterTamper.value(QStringLiteral("models")).toArray();
    QJsonObject adapterModel = adapterModels.first().toObject();
    adapterModel.insert(
        QStringLiteral("adapterId"), QStringLiteral("step.attacker.scene-v1"));
    adapterModels[0] = adapterModel;
    adapterTamper.insert(QStringLiteral("models"), adapterModels);
    if (!expectTrustRejection(
            adapterTamper, QStringLiteral("step.sa10-2000h"), ROBOT_TYPE_STEP,
            QStringLiteral("adapterId")))
        return 1;

    QJsonObject typeTamper = validRoot;
    QJsonArray typeModels = typeTamper.value(QStringLiteral("models")).toArray();
    QJsonObject typeModel = typeModels.first().toObject();
    typeModel.insert(QStringLiteral("sourceRobotType"), ROBOT_TYPE_FANUC);
    typeModels[0] = typeModel;
    typeTamper.insert(QStringLiteral("models"), typeModels);
    if (!expectTrustRejection(
            typeTamper, QStringLiteral("step.sa10-2000h"), ROBOT_TYPE_FANUC,
            QStringLiteral("sourceRobotType")))
        return 1;

    // 同一受信源 STEP 使用另一个完全有效的余量/profile/payload 简模，也不能
    // 靠同步重算 catalog payload 取得资格；受信对象是完整 revision 元组。
    RobotCadAssemblyLoader::Result alternateFixture = RobotFixture(firstStep.sha256);
    RobotCollisionEnvelopeStore::GenerationParameters alternateGeneration;
    alternateGeneration.safetyMarginMm = 31.0;
    RobotCollisionEnvelopeStore::EnvelopeSet alternateEnvelope;
    RobotCollisionEnvelopeStore::StoredAsset alternateCollision;
    if (!RobotCollisionEnvelopeStore::Generate(
            alternateFixture, alternateGeneration, alternateEnvelope, error)
        || !RobotCollisionEnvelopeStore::Persist(
            alternateEnvelope, alternateCollision, error))
        return Fail(QStringLiteral("cannot create alternate-margin collision: %1")
            .arg(error));
    QJsonObject alternateCollisionCatalog = validRoot;
    QJsonArray alternateCollisionModels =
        alternateCollisionCatalog.value(QStringLiteral("models")).toArray();
    QJsonObject alternateCollisionModel =
        alternateCollisionModels.first().toObject();
    alternateCollisionModel.insert(QStringLiteral("collision"), QJsonObject{
        { QStringLiteral("profileKeySha256"),
          alternateCollision.profileKeySha256 },
        { QStringLiteral("sourceStepSha256"),
          alternateCollision.sourceStepSha256 },
        { QStringLiteral("storedFileName"), alternateCollision.storedFileName },
        { QStringLiteral("sizeBytes"), alternateCollision.sizeBytes },
        { QStringLiteral("safetyMarginMicrometres"),
          alternateCollision.safetyMarginMicrometres },
        { QStringLiteral("payloadSha256"), alternateEnvelope.payloadSha256 }
    });
    alternateCollisionModels[0] = alternateCollisionModel;
    alternateCollisionCatalog.insert(
        QStringLiteral("models"), alternateCollisionModels);
    if (!expectTrustRejection(
            alternateCollisionCatalog, QStringLiteral("step.sa10-2000h"),
            ROBOT_TYPE_STEP, QStringLiteral("collision profile/margin/payload")))
        return 1;

    // 更强的攻击探针：保持受信 source/profile/30 mm 不变，手工修改碰撞 JSON，
    // 同时重算碰撞 payload、文件大小和 catalog payload。结构与全部普通 SHA 均
    // 自洽，但 payload 不在代码内受信 revision 元组中，仍必须被拒绝。
    const QString collisionPath = QDir(RobotModelCatalogStore::StoreDirectory())
        .filePath(firstCollision.storedFileName);
    const QByteArray validCollisionBytes = ReadBytes(collisionPath);
    QJsonObject forgedCollisionRoot =
        QJsonDocument::fromJson(validCollisionBytes).object();
    QJsonArray forgedJoints =
        forgedCollisionRoot.value(QStringLiteral("joints")).toArray();
    QJsonObject forgedJoint0 = forgedJoints.first().toObject();
    forgedJoint0.insert(QStringLiteral("assemblyPath"), QStringLiteral("Robot/K0"));
    forgedJoints[0] = forgedJoint0;
    forgedCollisionRoot.insert(QStringLiteral("joints"), forgedJoints);
    forgedCollisionRoot.remove(QStringLiteral("payloadSha256"));
    const QString forgedCollisionPayload = PayloadSha(forgedCollisionRoot);
    forgedCollisionRoot.insert(
        QStringLiteral("payloadSha256"), forgedCollisionPayload);
    const QByteArray forgedCollisionBytes =
        QJsonDocument(forgedCollisionRoot).toJson(QJsonDocument::Indented);

    QJsonObject forgedCollisionCatalog = validRoot;
    QJsonArray forgedCollisionModels =
        forgedCollisionCatalog.value(QStringLiteral("models")).toArray();
    QJsonObject forgedCollisionModel = forgedCollisionModels.first().toObject();
    QJsonObject forgedCollisionRecord =
        forgedCollisionModel.value(QStringLiteral("collision")).toObject();
    forgedCollisionRecord.insert(
        QStringLiteral("sizeBytes"), forgedCollisionBytes.size());
    forgedCollisionRecord.insert(
        QStringLiteral("payloadSha256"), forgedCollisionPayload);
    forgedCollisionModel.insert(
        QStringLiteral("collision"), forgedCollisionRecord);
    forgedCollisionModels[0] = forgedCollisionModel;
    forgedCollisionCatalog.insert(
        QStringLiteral("models"), forgedCollisionModels);
    const bool forgedCollisionWritten =
        WriteBytes(collisionPath, forgedCollisionBytes)
        && WriteBytes(
            catalogPath,
            QJsonDocument(WithUpdatedPayloadSha(forgedCollisionCatalog)).toJson());
    RobotCollisionEnvelopeStore::StoredAsset forgedCollisionAsset = firstCollision;
    forgedCollisionAsset.sizeBytes = forgedCollisionBytes.size();
    RobotCollisionEnvelopeStore::EnvelopeSet forgedEnvelope;
    QString forgedAssetError;
    const bool forgedAssetValid = forgedCollisionWritten
        && RobotCollisionEnvelopeStore::LoadAsset(
            forgedCollisionAsset, forgedEnvelope, forgedAssetError)
        && forgedEnvelope.payloadSha256 == forgedCollisionPayload;
    RobotModelCatalogStore::Eligibility forgedEligibility;
    QString forgedError;
    const bool forgedResolved = forgedCollisionWritten
        && RobotModelCatalogStore::ResolveModelEligibility(
            QStringLiteral("step.sa10-2000h"), ROBOT_TYPE_STEP,
            forgedEligibility, forgedError);
    const bool forgedRejected = forgedResolved && !forgedEligibility.eligible
        && forgedEligibility.reason.contains(QStringLiteral("未受信"));
    const bool forgedRestored = WriteBytes(catalogPath, validCatalogBytes)
        && WriteBytes(collisionPath, validCollisionBytes);
    if (!Require(
            forgedCollisionWritten && forgedAssetValid
                && forgedRejected && forgedRestored,
            QStringLiteral(
                "self-consistent collision JSON + catalog forgery was not rejected: "
                "%1 / %2 / %3")
                .arg(forgedAssetError, forgedError, forgedEligibility.reason)))
        return 1;

    if (!Require(RobotModelCatalogStore::ResolveModelEligibility(
                     QStringLiteral("step.sa10-2000h"), ROBOT_TYPE_FANUC,
                     eligibility, error)
                     && !eligibility.eligible
                     && eligibility.reason.contains(QStringLiteral("类型")),
                 QStringLiteral("wrong robot type was not rejected"))
        || !Require(RobotModelCatalogStore::ResolveModelEligibility(
                        QStringLiteral("fanuc.unknown"), ROBOT_TYPE_FANUC,
                        eligibility, error)
                        && !eligibility.eligible
                        && eligibility.reason.contains(QStringLiteral("尚未导入")),
                    QStringLiteral("unknown model was not rejected normally")))
        return 1;

    // 相同型号、相同 revision 和相同适配资产必须幂等，不得追加第二条。
    RobotModelCatalogStore::ModelRecord repeated;
    if (!Require(RobotModelCatalogStore::RegisterValidatedModel(
                     QStringLiteral("step.sa10-2000h"),
                     QStringLiteral("ignored repeat display name"),
                     QStringLiteral("step.sa10-2000h.scene-v1"),
                     ROBOT_TYPE_STEP,
                     firstStep,
                     firstCollision,
                     repeated,
                     error)
                     && repeated.registeredUtc == firstModel.registeredUtc,
                 QStringLiteral("same revision registration was not idempotent: %1")
                    .arg(error))
        || !Require(RobotModelCatalogStore::ListModels(models, error)
                        && models.size() == 1,
                    QStringLiteral("idempotent registration duplicated model")))
        return 1;

    // 其它 STEP revision 不能静默替换同一个可读型号键。
    const QString secondSource = sourceRoot.filePath(QStringLiteral("second.step"));
    if (!WriteBytes(secondSource, StepFixture("model-two")))
        return Fail(QStringLiteral("cannot write second STEP"));
    TheoreticalRobotModelStore::Asset secondStep;
    RobotCollisionEnvelopeStore::StoredAsset secondCollision;
    RobotCollisionEnvelopeStore::EnvelopeSet secondEnvelope;
    if (!CreateAssets(
            secondSource, secondStep, secondCollision, secondEnvelope, error))
        return Fail(QStringLiteral("cannot create second assets: %1").arg(error));

    // 源身份篡改测试使用另一份结构完整、受控存储且拥有有效碰撞简模的 STEP，
    // 并同步改写 catalog 的所有关联字段；它仍不能绕过代码内 revision 白名单。
    const QString untrustedSource =
        sourceRoot.filePath(QStringLiteral("untrusted.step"));
    if (!WriteBytes(untrustedSource, StepFixture("model-untrusted")))
        return Fail(QStringLiteral("cannot write untrusted STEP"));
    TheoreticalRobotModelStore::Asset untrustedStep;
    RobotCollisionEnvelopeStore::StoredAsset untrustedCollision;
    RobotCollisionEnvelopeStore::EnvelopeSet untrustedEnvelope;
    if (!CreateAssets(
            untrustedSource, untrustedStep, untrustedCollision,
            untrustedEnvelope, error))
        return Fail(QStringLiteral("cannot create untrusted assets: %1").arg(error));
    QJsonObject sourceTamper = validRoot;
    QJsonArray sourceModels = sourceTamper.value(QStringLiteral("models")).toArray();
    QJsonObject sourceModel = sourceModels.first().toObject();
    sourceModel.insert(QStringLiteral("sourceStep"), QJsonObject{
        { QStringLiteral("sha256"), untrustedStep.sha256 },
        { QStringLiteral("storedFileName"), untrustedStep.storedFileName },
        { QStringLiteral("originalDisplayName"), untrustedStep.originalDisplayName },
        { QStringLiteral("sizeBytes"), untrustedStep.sizeBytes },
        { QStringLiteral("importedUtc"), untrustedStep.importedUtc }
    });
    sourceModel.insert(QStringLiteral("collision"), QJsonObject{
        { QStringLiteral("profileKeySha256"), untrustedCollision.profileKeySha256 },
        { QStringLiteral("sourceStepSha256"), untrustedCollision.sourceStepSha256 },
        { QStringLiteral("storedFileName"), untrustedCollision.storedFileName },
        { QStringLiteral("sizeBytes"), untrustedCollision.sizeBytes },
        { QStringLiteral("safetyMarginMicrometres"),
          untrustedCollision.safetyMarginMicrometres },
        { QStringLiteral("payloadSha256"), untrustedEnvelope.payloadSha256 }
    });
    sourceModels[0] = sourceModel;
    sourceTamper.insert(QStringLiteral("models"), sourceModels);
    if (!expectTrustRejection(
            sourceTamper, QStringLiteral("step.sa10-2000h"), ROBOT_TYPE_STEP,
            QStringLiteral("source STEP SHA-256")))
        return 1;

    RobotModelCatalogStore::ModelRecord rejected;
    if (!Require(!RobotModelCatalogStore::RegisterValidatedModel(
                     QStringLiteral("step.sa10-2000h"), QStringLiteral("replacement"),
                     QStringLiteral("step.sa10-2000h.scene-v1"), ROBOT_TYPE_STEP,
                     secondStep, secondCollision, rejected, error)
                     && rejected.modelId.isEmpty()
                     && error.contains(QStringLiteral("显式升级")),
                 QStringLiteral("different revision silently replaced model"))
        || !Require(RobotModelCatalogStore::ListModels(models, error)
                        && models.size() == 1
                        && models.first().sourceStep.sha256 == firstStep.sha256,
                    QStringLiteral("failed replacement changed catalog")))
        return 1;

    // 同一 revision 也不能通过另一个可读型号键建立歧义别名。
    error.clear();
    if (!Require(!RobotModelCatalogStore::RegisterValidatedModel(
                     QStringLiteral("step.alias"), QStringLiteral("alias"),
                     QStringLiteral("step.sa10-2000h.scene-v1"), ROBOT_TYPE_STEP,
                     firstStep, firstCollision, rejected, error)
                     && error.contains(QStringLiteral("已登记")),
                 QStringLiteral("same STEP revision was aliased")))
        return 1;

    // 注册阶段必须完整哈希 STEP；同尺寸篡改不能成为新的 catalog 条目。
    const QString secondStored = QDir(RobotModelCatalogStore::StoreDirectory())
        .filePath(secondStep.storedFileName);
    QByteArray tamperedStep = ReadBytes(secondStored);
    if (tamperedStep.size() < 20)
        return Fail(QStringLiteral("second STEP fixture unexpectedly short"));
    tamperedStep[tamperedStep.size() / 2] =
        tamperedStep.at(tamperedStep.size() / 2) == 'X' ? 'Y' : 'X';
    if (!WriteBytes(secondStored, tamperedStep))
        return Fail(QStringLiteral("cannot tamper second STEP"));
    error.clear();
    if (!Require(!RobotModelCatalogStore::RegisterValidatedModel(
                     QStringLiteral("step.second"), QStringLiteral("second"),
                     QStringLiteral("step.second.scene-v1"), ROBOT_TYPE_STEP,
                     secondStep, secondCollision, rejected, error)
                     && !error.isEmpty(),
                 QStringLiteral("registration accepted same-size STEP tamper")))
        return 1;

    // 资格快速门禁不重新哈希 104 MB STEP；已验证简模仍可独立使用。
    const QString firstStored = QDir(RobotModelCatalogStore::StoreDirectory())
        .filePath(firstStep.storedFileName);
    const QByteArray firstValidBytes = ReadBytes(firstStored);
    QByteArray firstSameSizeTamper = firstValidBytes;
    firstSameSizeTamper[firstSameSizeTamper.size() / 2] =
        firstSameSizeTamper.at(firstSameSizeTamper.size() / 2) == 'Q' ? 'R' : 'Q';
    if (!WriteBytes(firstStored, firstSameSizeTamper)
        || !Require(RobotModelCatalogStore::ResolveModelEligibility(
                        QStringLiteral("step.sa10-2000h"), ROBOT_TYPE_STEP,
                        eligibility, error)
                        && eligibility.eligible,
                    QStringLiteral("eligibility unexpectedly rehashed large STEP"))
        || !WriteBytes(firstStored, firstValidBytes))
        return Fail(QStringLiteral("fast STEP gate probe failed: %1").arg(error));

    // 小型碰撞 JSON 每次资格查询均完整校验，篡改必须关闭失败。
    QJsonObject collisionRoot = QJsonDocument::fromJson(validCollisionBytes).object();
    collisionRoot.insert(QStringLiteral("unexpected"), true);
    if (!WriteBytes(collisionPath, QJsonDocument(collisionRoot).toJson())
        || !Require(RobotModelCatalogStore::ResolveModelEligibility(
                        QStringLiteral("step.sa10-2000h"), ROBOT_TYPE_STEP,
                        eligibility, error)
                        && !eligibility.eligible
                        && eligibility.reason.contains(QStringLiteral("简模")),
                    QStringLiteral("tampered collision JSON remained eligible"))
        || !WriteBytes(collisionPath, validCollisionBytes))
        return Fail(QStringLiteral("collision tamper probe failed"));

    // catalog 是严格 schema 且带 payload；即使重新计算 payload，路径注入也拒绝。
    QJsonObject maliciousRoot = validRoot;
    QJsonArray maliciousModels = maliciousRoot.value(QStringLiteral("models")).toArray();
    QJsonObject maliciousModel = maliciousModels.first().toObject();
    QJsonObject maliciousSource = maliciousModel.value(QStringLiteral("sourceStep")).toObject();
    maliciousSource.insert(QStringLiteral("storedFileName"), QStringLiteral("../escape.step"));
    maliciousModel.insert(QStringLiteral("sourceStep"), maliciousSource);
    maliciousModels[0] = maliciousModel;
    maliciousRoot.insert(QStringLiteral("models"), maliciousModels);
    maliciousRoot.remove(QStringLiteral("payloadSha256"));
    maliciousRoot.insert(QStringLiteral("payloadSha256"), PayloadSha(maliciousRoot));
    if (!WriteBytes(catalogPath, QJsonDocument(maliciousRoot).toJson())
        || !Require(!RobotModelCatalogStore::ListModels(models, error)
                        && models.isEmpty(),
                    QStringLiteral("catalog path traversal was accepted"))
        || !WriteBytes(catalogPath, validCatalogBytes))
        return Fail(QStringLiteral("catalog traversal probe failed"));

    // 未知字段且旧 payload 必须失败，不得泄漏上一次列表。
    QJsonObject unknownRoot = validRoot;
    unknownRoot.insert(QStringLiteral("bindings"), QJsonArray());
    models = { firstModel };
    if (!WriteBytes(catalogPath, QJsonDocument(unknownRoot).toJson())
        || !Require(!RobotModelCatalogStore::ListModels(models, error)
                        && models.isEmpty(),
                    QStringLiteral("unknown catalog field was accepted or stale output leaked"))
        || !WriteBytes(catalogPath, validCatalogBytes))
        return Fail(QStringLiteral("strict schema probe failed"));

    QTextStream(stdout)
        << "PASS: centralized model catalog, immutable STEP revision, strict schema, "
           "atomic registration, fast eligibility and collision/path tamper gates"
        << Qt::endl;
    return 0;
}
