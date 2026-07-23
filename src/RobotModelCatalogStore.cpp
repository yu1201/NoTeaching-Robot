#include "RobotModelCatalogStore.h"

#include "AppPaths.h"
#include "Const.h"

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

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
constexpr int kSchemaVersion = 1;
const QString kKind = QStringLiteral("robot-model-catalog");
const QString kStoreRelativePath = QStringLiteral("Data/RobotModels");
const QString kCatalogFileName = QStringLiteral("robot-model-catalog.json");
const QString kMutationLockName = QStringLiteral("robot-model-catalog.lock");

const QString kVerifiedSa10ModelId = QStringLiteral("step.sa10-2000h");
const QString kVerifiedSa10DisplayName = QStringLiteral("新时达 SA10-2000H");
const QString kVerifiedSa10AdapterId =
    QStringLiteral("step.sa10-2000h.scene-v1");
const QString kVerifiedSa10AssemblySha256 = QStringLiteral(
    "f8299f6deabc7b6f6ae799592f1f93e2366311d552bf91ac431eccec14bfcaa8");

// catalog 及其 payload SHA-256 都是可由本机用户修改的普通数据文件，
// 因而只能检测意外损坏，不能授权新的型号、适配器或 STEP revision。
// 真正可进入模型焊接流程的身份必须来自随程序发布的代码内注册表。
//
// 当前导入器已经对 SA10 总装及其确定性碰撞简模做精确校验，所以这里把源
// STEP、profileKey、碰撞 payload 和安全余量作为不可拆分的 revision 元组。
// 后续若厂家提供新的合法 revision，应在完成相同语义审核后加入完整元组；
// 如果以后改用厂家签名清单，则应在这里校验内置公钥签名，而不能把另一个
// 普通 payload SHA 或可由同一用户生成的本地“证明”当作签名。
struct TrustedRevisionDefinition
{
    QString sourceStepSha256;
    QString collisionProfileKeySha256;
    QString collisionPayloadSha256;
    qint64 safetyMarginMicrometres = 0;
};

struct TrustedModelDefinition
{
    QString modelId;
    QString adapterId;
    int sourceRobotType = -1;
    QList<TrustedRevisionDefinition> trustedRevisions;
};

const QList<TrustedModelDefinition>& TrustedModelDefinitions()
{
    static const QList<TrustedModelDefinition> definitions = []
    {
        QList<TrustedRevisionDefinition> sa10Revisions{
            TrustedRevisionDefinition{
                kVerifiedSa10AssemblySha256,
                QStringLiteral(
                    "1a8f6c2389970fcfaf7bc4e8854a63a6bda2f5c85a16449ab12c550dda2905dd"),
                QStringLiteral(
                    "4cca84e0f1a9c8b280fc49879b6d347504e3e359764c89f8efad3827a6224794"),
                30000
            }
        };
#ifdef ROBOT_MODEL_CATALOG_ENABLE_TEST_TRUST
        // Standalone store tests use tiny deterministic STEP fixtures. These
        // tuples are compiled only into the test executable, never the product.
        sa10Revisions.append(TrustedRevisionDefinition{
            QStringLiteral(
                "5192ac51e44a587ff2d7d2d8b3780d6f4445503d189539f1efc6ab04ed56cf53"),
            QStringLiteral(
                "a35350bd57bfc26ecfb330161433d1904d00a5584ffad9eb7f840c74cf3d0ef6"),
            QStringLiteral(
                "933d2eda5aea04ed9edf33c9a68de42d56303506985bcc4e5591ebebc99e0c1a"),
            30000
        });
#endif
        return QList<TrustedModelDefinition>{ TrustedModelDefinition{
            kVerifiedSa10ModelId,
            kVerifiedSa10AdapterId,
            ROBOT_TYPE_STEP,
            sa10Revisions
        } };
    }();
    return definitions;
}

const TrustedModelDefinition* FindTrustedModelDefinition(
    const QString& modelId)
{
    const auto& definitions = TrustedModelDefinitions();
    const auto found = std::find_if(
        definitions.cbegin(), definitions.cend(),
        [&modelId](const TrustedModelDefinition& definition)
        {
            return definition.modelId == modelId;
        });
    return found == definitions.cend() ? nullptr : &(*found);
}

bool ValidateTrustedModelIdentity(
    const RobotModelCatalogStore::ModelRecord& model,
    QString& error)
{
    const TrustedModelDefinition* definition =
        FindTrustedModelDefinition(model.modelId);
    if (definition == nullptr)
    {
        error = QStringLiteral(
            "型号 %1 没有代码内受信适配规则；catalog payload SHA-256 "
            "不能授权新型号。").arg(model.modelId);
        return false;
    }
    if (model.adapterId != definition->adapterId
        || model.sourceRobotType != definition->sourceRobotType)
    {
        error = QStringLiteral(
            "型号 %1 的适配器或机器人类型不符合代码内受信规则。")
            .arg(model.modelId);
        return false;
    }
    const auto revision = std::find_if(
        definition->trustedRevisions.cbegin(),
        definition->trustedRevisions.cend(),
        [&model](const TrustedRevisionDefinition& candidate)
        {
            return candidate.sourceStepSha256 == model.sourceStep.sha256;
        });
    if (revision == definition->trustedRevisions.cend())
    {
        error = QStringLiteral(
            "型号 %1 的 STEP revision 未列入代码内受信源身份。")
            .arg(model.modelId);
        return false;
    }
    if (model.collision.profileKeySha256
            != revision->collisionProfileKeySha256
        || model.collisionPayloadSha256 != revision->collisionPayloadSha256
        || model.collision.safetyMarginMicrometres
            != revision->safetyMarginMicrometres)
    {
        error = QStringLiteral(
            "型号 %1 的碰撞 profile、payload 或安全余量不符合代码内受信 "
            "revision 元组。").arg(model.modelId);
        return false;
    }
    error.clear();
    return true;
}

struct Catalog
{
    qint64 revision = 0;
    QList<RobotModelCatalogStore::ModelRecord> models;
};

QMutex& MutationMutex()
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
            return false;
    }
    return true;
}

bool IsSafeText(
    const QString& value,
    qsizetype maximumLength,
    bool allowEmpty = false)
{
    if ((!allowEmpty && value.isEmpty()) || value.size() > maximumLength)
        return false;
    for (const QChar ch : value)
    {
        if (ch.unicode() < 0x20 || ch.unicode() == 0x7f) return false;
    }
    return true;
}

bool IsStableIdentifier(const QString& value)
{
    if (value.isEmpty() || value.size() > 128
        || value.front() == QLatin1Char('.')
        || value.back() == QLatin1Char('.'))
        return false;
    for (const QChar ch : value)
    {
        if (!((ch >= QLatin1Char('a') && ch <= QLatin1Char('z'))
              || (ch >= QLatin1Char('0') && ch <= QLatin1Char('9'))
              || ch == QLatin1Char('.') || ch == QLatin1Char('-')
              || ch == QLatin1Char('_')))
            return false;
    }
    return true;
}

bool HasExactKeys(
    const QJsonObject& object,
    const std::initializer_list<QString>& keys)
{
    if (object.size() != static_cast<qsizetype>(keys.size())) return false;
    for (const QString& key : keys)
    {
        if (!object.contains(key)) return false;
    }
    return true;
}

bool IsExactInteger(const QJsonValue& value, qint64 minimum, qint64 maximum)
{
    if (!value.isDouble()) return false;
    const double number = value.toDouble(std::numeric_limits<double>::quiet_NaN());
    return std::isfinite(number) && std::floor(number) == number
        && number >= static_cast<double>(minimum)
        && number <= static_cast<double>(maximum);
}

bool IsValidUtc(const QString& value)
{
    const QDateTime parsed = QDateTime::fromString(value, Qt::ISODateWithMs);
    return parsed.isValid() && parsed.timeSpec() == Qt::UTC;
}

bool SourceAssetsEqual(
    const TheoreticalRobotModelStore::Asset& left,
    const TheoreticalRobotModelStore::Asset& right)
{
    return left.sha256 == right.sha256
        && left.storedFileName == right.storedFileName
        && left.originalDisplayName == right.originalDisplayName
        && left.sizeBytes == right.sizeBytes
        && left.importedUtc == right.importedUtc;
}

bool CollisionAssetsEqual(
    const RobotCollisionEnvelopeStore::StoredAsset& left,
    const RobotCollisionEnvelopeStore::StoredAsset& right)
{
    return left.profileKeySha256 == right.profileKeySha256
        && left.sourceStepSha256 == right.sourceStepSha256
        && left.storedFileName == right.storedFileName
        && left.sizeBytes == right.sizeBytes
        && left.safetyMarginMicrometres == right.safetyMarginMicrometres;
}

bool ValidateSourceRecord(
    const TheoreticalRobotModelStore::Asset& asset,
    QString& error)
{
    if (!IsLowerSha256(asset.sha256)
        || asset.storedFileName != asset.sha256 + QStringLiteral(".step")
        || !AppPaths::IsSafePathComponent(asset.storedFileName)
        || !IsSafeText(asset.originalDisplayName, 1024)
        || asset.sizeBytes <= 0
        || asset.sizeBytes > TheoreticalRobotModelStore::MaximumAssetBytes
        || !IsValidUtc(asset.importedUtc))
    {
        error = QStringLiteral("机器人型号目录中的 STEP 资产记录无效。");
        return false;
    }
    return true;
}

bool ValidateCollisionRecord(
    const RobotCollisionEnvelopeStore::StoredAsset& asset,
    const QString& payloadSha256,
    QString& error)
{
    const QString suffix = QStringLiteral(".robot-aabb.json");
    if (!IsLowerSha256(asset.profileKeySha256)
        || !IsLowerSha256(asset.sourceStepSha256)
        || asset.storedFileName != asset.profileKeySha256 + suffix
        || !AppPaths::IsSafePathComponent(asset.storedFileName)
        || asset.sizeBytes <= 0
        || asset.sizeBytes > RobotCollisionEnvelopeStore::MaximumEnvelopeFileBytes
        || asset.safetyMarginMicrometres < 0
        || asset.safetyMarginMicrometres
            > static_cast<qint64>(
                RobotCollisionEnvelopeStore::MaximumSafetyMarginMm * 1000.0)
        || !IsLowerSha256(payloadSha256))
    {
        error = QStringLiteral("机器人型号目录中的碰撞简模资产记录无效。");
        return false;
    }
    return true;
}

bool ValidateModelRecord(
    const RobotModelCatalogStore::ModelRecord& model,
    QString& error)
{
    if (!IsStableIdentifier(model.modelId)
        || !IsSafeText(model.displayName, 256)
        || !IsStableIdentifier(model.adapterId)
        || model.sourceRobotType <= 0
        || model.sourceRobotType > 1000000
        || !IsValidUtc(model.registeredUtc)
        || !ValidateSourceRecord(model.sourceStep, error)
        || !ValidateCollisionRecord(
            model.collision, model.collisionPayloadSha256, error))
    {
        if (error.isEmpty())
            error = QStringLiteral("机器人型号目录包含无效型号字段。");
        return false;
    }
    if (model.collision.sourceStepSha256 != model.sourceStep.sha256)
    {
        error = QStringLiteral("机器人型号的总装和碰撞简模 STEP 身份不一致。");
        return false;
    }
    return true;
}

QJsonObject SourceToJson(const TheoreticalRobotModelStore::Asset& asset)
{
    return QJsonObject{
        { QStringLiteral("sha256"), asset.sha256 },
        { QStringLiteral("storedFileName"), asset.storedFileName },
        { QStringLiteral("originalDisplayName"), asset.originalDisplayName },
        { QStringLiteral("sizeBytes"), asset.sizeBytes },
        { QStringLiteral("importedUtc"), asset.importedUtc }
    };
}

QJsonObject CollisionToJson(
    const RobotCollisionEnvelopeStore::StoredAsset& asset,
    const QString& payloadSha256)
{
    return QJsonObject{
        { QStringLiteral("profileKeySha256"), asset.profileKeySha256 },
        { QStringLiteral("sourceStepSha256"), asset.sourceStepSha256 },
        { QStringLiteral("storedFileName"), asset.storedFileName },
        { QStringLiteral("sizeBytes"), asset.sizeBytes },
        { QStringLiteral("safetyMarginMicrometres"),
          asset.safetyMarginMicrometres },
        { QStringLiteral("payloadSha256"), payloadSha256 }
    };
}

QJsonObject ModelToJson(const RobotModelCatalogStore::ModelRecord& model)
{
    return QJsonObject{
        { QStringLiteral("modelId"), model.modelId },
        { QStringLiteral("displayName"), model.displayName },
        { QStringLiteral("adapterId"), model.adapterId },
        { QStringLiteral("sourceRobotType"), model.sourceRobotType },
        { QStringLiteral("sourceStep"), SourceToJson(model.sourceStep) },
        { QStringLiteral("collision"), CollisionToJson(
            model.collision, model.collisionPayloadSha256) },
        { QStringLiteral("registeredUtc"), model.registeredUtc }
    };
}

QJsonObject CatalogPayload(const Catalog& catalog)
{
    QJsonArray models;
    for (const auto& model : catalog.models) models.append(ModelToJson(model));
    return QJsonObject{
        { QStringLiteral("schemaVersion"), kSchemaVersion },
        { QStringLiteral("kind"), kKind },
        { QStringLiteral("revision"), catalog.revision },
        { QStringLiteral("models"), models }
    };
}

QString JsonPayloadSha256(const QJsonObject& payload)
{
    return QString::fromLatin1(QCryptographicHash::hash(
        QJsonDocument(payload).toJson(QJsonDocument::Compact),
        QCryptographicHash::Sha256).toHex());
}

bool ParseSource(
    const QJsonValue& value,
    TheoreticalRobotModelStore::Asset& asset,
    QString& error)
{
    asset = TheoreticalRobotModelStore::Asset();
    if (!value.isObject())
    {
        error = QStringLiteral("机器人型号的 STEP 资产不是对象。");
        return false;
    }
    const QJsonObject object = value.toObject();
    if (!HasExactKeys(object, {
            QStringLiteral("sha256"), QStringLiteral("storedFileName"),
            QStringLiteral("originalDisplayName"), QStringLiteral("sizeBytes"),
            QStringLiteral("importedUtc") })
        || !object.value(QStringLiteral("sha256")).isString()
        || !object.value(QStringLiteral("storedFileName")).isString()
        || !object.value(QStringLiteral("originalDisplayName")).isString()
        || !IsExactInteger(
            object.value(QStringLiteral("sizeBytes")), 1,
            TheoreticalRobotModelStore::MaximumAssetBytes)
        || !object.value(QStringLiteral("importedUtc")).isString())
    {
        error = QStringLiteral("机器人型号的 STEP 资产 schema 无效。");
        return false;
    }
    asset.sha256 = object.value(QStringLiteral("sha256")).toString();
    asset.storedFileName = object.value(QStringLiteral("storedFileName")).toString();
    asset.originalDisplayName =
        object.value(QStringLiteral("originalDisplayName")).toString();
    asset.sizeBytes = object.value(QStringLiteral("sizeBytes")).toInteger(-1);
    asset.importedUtc = object.value(QStringLiteral("importedUtc")).toString();
    return ValidateSourceRecord(asset, error);
}

bool ParseCollision(
    const QJsonValue& value,
    RobotCollisionEnvelopeStore::StoredAsset& asset,
    QString& payloadSha256,
    QString& error)
{
    asset = RobotCollisionEnvelopeStore::StoredAsset();
    payloadSha256.clear();
    if (!value.isObject())
    {
        error = QStringLiteral("机器人型号的碰撞简模资产不是对象。");
        return false;
    }
    const QJsonObject object = value.toObject();
    if (!HasExactKeys(object, {
            QStringLiteral("profileKeySha256"),
            QStringLiteral("sourceStepSha256"),
            QStringLiteral("storedFileName"), QStringLiteral("sizeBytes"),
            QStringLiteral("safetyMarginMicrometres"),
            QStringLiteral("payloadSha256") })
        || !object.value(QStringLiteral("profileKeySha256")).isString()
        || !object.value(QStringLiteral("sourceStepSha256")).isString()
        || !object.value(QStringLiteral("storedFileName")).isString()
        || !IsExactInteger(
            object.value(QStringLiteral("sizeBytes")), 1,
            RobotCollisionEnvelopeStore::MaximumEnvelopeFileBytes)
        || !IsExactInteger(
            object.value(QStringLiteral("safetyMarginMicrometres")), 0,
            static_cast<qint64>(
                RobotCollisionEnvelopeStore::MaximumSafetyMarginMm * 1000.0))
        || !object.value(QStringLiteral("payloadSha256")).isString())
    {
        error = QStringLiteral("机器人型号的碰撞简模资产 schema 无效。");
        return false;
    }
    asset.profileKeySha256 =
        object.value(QStringLiteral("profileKeySha256")).toString();
    asset.sourceStepSha256 =
        object.value(QStringLiteral("sourceStepSha256")).toString();
    asset.storedFileName =
        object.value(QStringLiteral("storedFileName")).toString();
    asset.sizeBytes = object.value(QStringLiteral("sizeBytes")).toInteger(-1);
    asset.safetyMarginMicrometres =
        object.value(QStringLiteral("safetyMarginMicrometres")).toInteger(-1);
    payloadSha256 = object.value(QStringLiteral("payloadSha256")).toString();
    return ValidateCollisionRecord(asset, payloadSha256, error);
}

bool ParseModel(
    const QJsonValue& value,
    RobotModelCatalogStore::ModelRecord& model,
    QString& error)
{
    model = RobotModelCatalogStore::ModelRecord();
    if (!value.isObject())
    {
        error = QStringLiteral("机器人型号目录包含非对象条目。");
        return false;
    }
    const QJsonObject object = value.toObject();
    if (!HasExactKeys(object, {
            QStringLiteral("modelId"), QStringLiteral("displayName"),
            QStringLiteral("adapterId"), QStringLiteral("sourceRobotType"),
            QStringLiteral("sourceStep"), QStringLiteral("collision"),
            QStringLiteral("registeredUtc") })
        || !object.value(QStringLiteral("modelId")).isString()
        || !object.value(QStringLiteral("displayName")).isString()
        || !object.value(QStringLiteral("adapterId")).isString()
        || !IsExactInteger(
            object.value(QStringLiteral("sourceRobotType")), 1, 1000000)
        || !object.value(QStringLiteral("registeredUtc")).isString())
    {
        error = QStringLiteral("机器人型号条目 schema 无效。");
        return false;
    }
    model.modelId = object.value(QStringLiteral("modelId")).toString();
    model.displayName = object.value(QStringLiteral("displayName")).toString();
    model.adapterId = object.value(QStringLiteral("adapterId")).toString();
    model.sourceRobotType =
        static_cast<int>(object.value(QStringLiteral("sourceRobotType")).toInteger(-1));
    model.registeredUtc =
        object.value(QStringLiteral("registeredUtc")).toString();
    if (!ParseSource(
            object.value(QStringLiteral("sourceStep")), model.sourceStep, error)
        || !ParseCollision(
            object.value(QStringLiteral("collision")), model.collision,
            model.collisionPayloadSha256, error))
        return false;
    return ValidateModelRecord(model, error);
}

bool EnsureStoreDirectory(QString& error)
{
    const QString store = RobotModelCatalogStore::StoreDirectory();
    if (store.isEmpty())
    {
        error = QStringLiteral("无法解析机器人型号库目录。");
        return false;
    }
    const QString dataRoot = QDir::cleanPath(AppPaths::DataRootPath());
    const QString dataDirectory = QDir(dataRoot).filePath(QStringLiteral("Data"));
    for (const QString& component : { dataDirectory, store })
    {
        const QFileInfo info(component);
        if (IsLinkOrJunction(info))
        {
            error = QStringLiteral("机器人型号库路径包含符号链接或联接点：%1")
                .arg(QDir::toNativeSeparators(component));
            return false;
        }
    }
    QFileInfo storeInfo(store);
    if (storeInfo.exists() && (!storeInfo.isDir() || IsLinkOrJunction(storeInfo)))
    {
        error = QStringLiteral("机器人型号库路径不是普通目录。");
        return false;
    }
    if (!storeInfo.exists() && !QDir().mkpath(store))
    {
        error = QStringLiteral("无法创建机器人型号库目录。");
        return false;
    }
    storeInfo.refresh();
    const QString canonicalRoot = QFileInfo(dataRoot).canonicalFilePath();
    const QString canonicalStore = storeInfo.canonicalFilePath();
    const QString relative = QDir(canonicalRoot).relativeFilePath(canonicalStore)
        .replace(QLatin1Char('\\'), QLatin1Char('/'));
    if (!storeInfo.exists() || !storeInfo.isDir() || IsLinkOrJunction(storeInfo)
        || canonicalRoot.isEmpty() || canonicalStore.isEmpty()
        || relative == QStringLiteral("..") || relative.startsWith(QStringLiteral("../"))
        || QFileInfo(relative).isAbsolute() || QDir::isAbsolutePath(relative))
    {
        error = QStringLiteral("机器人型号库真实路径越出数据根目录。");
        return false;
    }
    return true;
}

QString ControlledPath(const QString& fileName)
{
    if (!AppPaths::IsSafePathComponent(fileName)) return QString();
    const QString path = AppPaths::WritableChildPath(kStoreRelativePath, fileName);
    const QString store = RobotModelCatalogStore::StoreDirectory();
    if (path.isEmpty() || store.isEmpty()) return QString();
    const QString relative = QDir(store).relativeFilePath(path)
        .replace(QLatin1Char('\\'), QLatin1Char('/'));
    return relative == fileName ? path : QString();
}

bool ReadStableCatalogBytes(
    const QString& path,
    QByteArray& bytes,
    bool& missing,
    QString& error)
{
    bytes.clear();
    missing = false;
    const QFileInfo before(path);
    if (IsLinkOrJunction(before))
    {
        error = QStringLiteral("机器人型号目录文件是符号链接或联接点。");
        return false;
    }
    if (!before.exists())
    {
        missing = true;
        return true;
    }
    if (!before.isFile() || before.size() <= 0
        || before.size() > RobotModelCatalogStore::MaximumCatalogFileBytes)
    {
        error = QStringLiteral("机器人型号目录不是有效的普通小文件。");
        return false;
    }
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly) || file.size() != before.size())
    {
        error = QStringLiteral("无法稳定读取机器人型号目录：%1")
            .arg(file.errorString());
        return false;
    }
    bytes = file.readAll();
    file.close();
    QFileInfo after(path);
    after.refresh();
    if (bytes.size() != before.size() || !after.exists() || !after.isFile()
        || IsLinkOrJunction(after) || after.size() != before.size()
        || after.lastModified().toMSecsSinceEpoch()
            != before.lastModified().toMSecsSinceEpoch())
    {
        bytes.clear();
        error = QStringLiteral("机器人型号目录在读取过程中发生变化。");
        return false;
    }
    return true;
}

bool ValidateCatalog(const Catalog& catalog, QString& error)
{
    if (catalog.revision < 0
        || catalog.models.size() > RobotModelCatalogStore::MaximumModels)
    {
        error = QStringLiteral("机器人型号目录 revision 或条目数量越界。");
        return false;
    }
    QSet<QString> modelIds;
    QSet<QString> sourceHashes;
    for (const auto& model : catalog.models)
    {
        if (!ValidateModelRecord(model, error)
            || modelIds.contains(model.modelId)
            || sourceHashes.contains(model.sourceStep.sha256))
        {
            if (error.isEmpty())
                error = QStringLiteral("机器人型号目录包含重复型号或 STEP revision。");
            return false;
        }
        modelIds.insert(model.modelId);
        sourceHashes.insert(model.sourceStep.sha256);
    }
    return true;
}

bool ReadCatalog(Catalog& catalog, QString& error)
{
    catalog = Catalog();
    if (!EnsureStoreDirectory(error)) return false;
    const QString path = RobotModelCatalogStore::CatalogFilePath();
    if (path.isEmpty())
    {
        error = QStringLiteral("无法解析机器人型号目录文件路径。");
        return false;
    }
    QByteArray bytes;
    bool missing = false;
    if (!ReadStableCatalogBytes(path, bytes, missing, error)) return false;
    if (missing) return true;

    QJsonParseError parseError;
    const QJsonDocument document = QJsonDocument::fromJson(bytes, &parseError);
    if (parseError.error != QJsonParseError::NoError || !document.isObject())
    {
        error = QStringLiteral("机器人型号目录 JSON 无效：%1")
            .arg(parseError.errorString());
        return false;
    }
    const QJsonObject root = document.object();
    if (!HasExactKeys(root, {
            QStringLiteral("schemaVersion"), QStringLiteral("kind"),
            QStringLiteral("revision"), QStringLiteral("models"),
            QStringLiteral("payloadSha256") })
        || root.value(QStringLiteral("schemaVersion")).toInt(-1) != kSchemaVersion
        || root.value(QStringLiteral("kind")).toString() != kKind
        || !IsExactInteger(
            root.value(QStringLiteral("revision")), 0,
            std::numeric_limits<qint64>::max())
        || !root.value(QStringLiteral("models")).isArray()
        || !root.value(QStringLiteral("payloadSha256")).isString())
    {
        error = QStringLiteral("机器人型号目录 schema 或字段类型无效。");
        return false;
    }
    const QString storedPayloadSha =
        root.value(QStringLiteral("payloadSha256")).toString();
    QJsonObject payload = root;
    payload.remove(QStringLiteral("payloadSha256"));
    if (!IsLowerSha256(storedPayloadSha)
        || JsonPayloadSha256(payload) != storedPayloadSha)
    {
        error = QStringLiteral("机器人型号目录 payload SHA-256 校验失败。");
        return false;
    }
    const QJsonArray models = root.value(QStringLiteral("models")).toArray();
    if (models.size() > RobotModelCatalogStore::MaximumModels)
    {
        error = QStringLiteral("机器人型号目录条目过多。");
        return false;
    }
    Catalog parsed;
    parsed.revision = root.value(QStringLiteral("revision")).toInteger(-1);
    for (const QJsonValue& value : models)
    {
        RobotModelCatalogStore::ModelRecord model;
        if (!ParseModel(value, model, error)) return false;
        parsed.models.append(model);
    }
    if (!ValidateCatalog(parsed, error)) return false;
    catalog = parsed;
    return true;
}

bool WriteCatalog(const Catalog& catalog, QString& error)
{
    if (!ValidateCatalog(catalog, error) || !EnsureStoreDirectory(error)) return false;
    QJsonObject root = CatalogPayload(catalog);
    root.insert(QStringLiteral("payloadSha256"), JsonPayloadSha256(root));
    const QByteArray bytes = QJsonDocument(root).toJson(QJsonDocument::Indented);
    if (bytes.isEmpty()
        || bytes.size() > RobotModelCatalogStore::MaximumCatalogFileBytes)
    {
        error = QStringLiteral("机器人型号目录超过持久化大小上限。");
        return false;
    }
    const QString path = RobotModelCatalogStore::CatalogFilePath();
    const QFileInfo existing(path);
    if (IsLinkOrJunction(existing)
        || (existing.exists() && !existing.isFile()))
    {
        error = QStringLiteral("机器人型号目录目标路径被非普通文件占用。");
        return false;
    }
    QSaveFile file(path);
    file.setDirectWriteFallback(false);
    if (!file.open(QIODevice::WriteOnly)
        || file.write(bytes) != bytes.size()
        || !file.commit())
    {
        file.cancelWriting();
        error = QStringLiteral("无法原子写入机器人型号目录：%1")
            .arg(file.errorString());
        return false;
    }
    return true;
}

bool AcquireMutationLock(QLockFile& lock, QString& error)
{
    lock.setStaleLockTime(0);
    if (lock.tryLock(10000)) return true;
    error = QStringLiteral("机器人型号目录正由另一个进程修改，请稍后重试。");
    return false;
}

bool FastValidateSourceAsset(
    const TheoreticalRobotModelStore::Asset& expected,
    QString& path,
    QString& error)
{
    path.clear();
    if (!ValidateSourceRecord(expected, error)) return false;
    QList<TheoreticalRobotModelStore::Asset> assets;
    if (!TheoreticalRobotModelStore::ListAssets(assets, error)) return false;
    const auto found = std::find_if(
        assets.cbegin(), assets.cend(),
        [&expected](const auto& candidate)
        {
            return SourceAssetsEqual(candidate, expected);
        });
    if (found == assets.cend())
    {
        error = QStringLiteral("机器人型号引用的 STEP 不在受控资产清单中。");
        return false;
    }
    const QString candidate = ControlledPath(expected.storedFileName);
    const QFileInfo info(candidate);
    if (candidate.isEmpty() || !info.exists() || !info.isFile()
        || IsLinkOrJunction(info) || info.size() != expected.sizeBytes)
    {
        error = QStringLiteral("机器人型号 STEP 缺失、大小不符或不是普通文件。");
        return false;
    }
    const QString canonicalStore = QFileInfo(
        RobotModelCatalogStore::StoreDirectory()).canonicalFilePath();
    const QString canonicalFile = info.canonicalFilePath();
    const QString relative = QDir(canonicalStore).relativeFilePath(canonicalFile)
        .replace(QLatin1Char('\\'), QLatin1Char('/'));
    if (canonicalStore.isEmpty() || canonicalFile.isEmpty()
        || relative != expected.storedFileName)
    {
        error = QStringLiteral("机器人型号 STEP 真实路径越出受控目录。");
        return false;
    }
    path = candidate;
    return true;
}

bool StrictValidateSourceAsset(
    const TheoreticalRobotModelStore::Asset& expected,
    QString& error)
{
    QString path;
    if (!FastValidateSourceAsset(expected, path, error)) return false;
    const QFileInfo before(path);
    const qint64 expectedModified = before.lastModified().toMSecsSinceEpoch();
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly) || file.size() != expected.sizeBytes)
    {
        error = QStringLiteral("无法稳定打开机器人型号 STEP：%1")
            .arg(file.errorString());
        return false;
    }
    QCryptographicHash hash(QCryptographicHash::Sha256);
    QByteArray head;
    QByteArray tail;
    QByteArray block(1024 * 1024, Qt::Uninitialized);
    qint64 total = 0;
    while (!file.atEnd())
    {
        const qint64 count = file.read(block.data(), block.size());
        if (count < 0)
        {
            error = QStringLiteral("读取机器人型号 STEP 失败：%1")
                .arg(file.errorString());
            return false;
        }
        if (count == 0) continue;
        total += count;
        if (total > expected.sizeBytes
            || total > TheoreticalRobotModelStore::MaximumAssetBytes)
        {
            error = QStringLiteral("机器人型号 STEP 读取长度越界。");
            return false;
        }
        hash.addData(QByteArrayView(block.constData(), static_cast<qsizetype>(count)));
        if (head.size() < 4096)
        {
            const qsizetype take = (std::min)(
                static_cast<qsizetype>(count), 4096 - head.size());
            head.append(block.constData(), take);
        }
        tail.append(block.constData(), static_cast<qsizetype>(count));
        if (tail.size() > 4096) tail = tail.right(4096);
    }
    file.close();
    QFileInfo after(path);
    after.refresh();
    if (total != expected.sizeBytes || !after.exists() || !after.isFile()
        || IsLinkOrJunction(after) || after.size() != expected.sizeBytes
        || after.lastModified().toMSecsSinceEpoch() != expectedModified
        || QString::fromLatin1(hash.result().toHex()).toLower() != expected.sha256)
    {
        error = QStringLiteral("机器人型号 STEP 大小、SHA-256 或稳定性校验失败。");
        return false;
    }
    if (head.startsWith("\xEF\xBB\xBF")) head.remove(0, 3);
    while (!head.isEmpty()
           && (head.front() == ' ' || head.front() == '\t'
               || head.front() == '\r' || head.front() == '\n'))
        head.remove(0, 1);
    if (!head.startsWith("ISO-10303-21;")
        || !tail.trimmed().endsWith("END-ISO-10303-21;"))
    {
        error = QStringLiteral("机器人型号资产不包含有效 STEP 文件边界。");
        return false;
    }
    return true;
}

bool ValidateCollisionAsset(
    const RobotModelCatalogStore::ModelRecord& model,
    RobotCollisionEnvelopeStore::EnvelopeSet& envelope,
    QString& error)
{
    envelope = RobotCollisionEnvelopeStore::EnvelopeSet();
    if (!RobotCollisionEnvelopeStore::LoadAsset(
            model.collision, envelope, error))
        return false;
    if (envelope.sourceStepSha256 != model.sourceStep.sha256
        || envelope.profileKeySha256 != model.collision.profileKeySha256
        || envelope.safetyMarginMicrometres
            != model.collision.safetyMarginMicrometres
        || envelope.payloadSha256 != model.collisionPayloadSha256)
    {
        envelope = RobotCollisionEnvelopeStore::EnvelopeSet();
        error = QStringLiteral("机器人型号与碰撞简模的 SHA、余量或 payload 不一致。");
        return false;
    }
    return true;
}
}

QString RobotModelCatalogStore::StoreDirectory()
{
    if (!AppPaths::IsInitialized()) return QString();
    return AppPaths::WritablePath(kStoreRelativePath);
}

QString RobotModelCatalogStore::CatalogFilePath()
{
    if (!AppPaths::IsInitialized()) return QString();
    return AppPaths::WritableChildPath(kStoreRelativePath, kCatalogFileName);
}

bool RobotModelCatalogStore::ListModels(
    QList<ModelRecord>& models,
    QString& error)
{
    models.clear();
    error.clear();
    Catalog catalog;
    if (!ReadCatalog(catalog, error)) return false;
    models = catalog.models;
    return true;
}

bool RobotModelCatalogStore::RegisterValidatedModel(
    const QString& modelId,
    const QString& displayName,
    const QString& adapterId,
    int sourceRobotType,
    const TheoreticalRobotModelStore::Asset& sourceStep,
    const RobotCollisionEnvelopeStore::StoredAsset& collision,
    ModelRecord& registeredModel,
    QString& error)
{
    registeredModel = ModelRecord();
    error.clear();
    ModelRecord candidate;
    candidate.modelId = modelId.trimmed();
    candidate.displayName = displayName.trimmed();
    candidate.adapterId = adapterId.trimmed();
    candidate.sourceRobotType = sourceRobotType;
    candidate.sourceStep = sourceStep;
    candidate.collision = collision;
    candidate.registeredUtc =
        QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs);

    // 先验证大型 STEP 和小型碰撞资产；在此之前不会创建 catalog 可见记录。
    if (!StrictValidateSourceAsset(candidate.sourceStep, error)) return false;
    RobotCollisionEnvelopeStore::EnvelopeSet envelope;
    if (!RobotCollisionEnvelopeStore::LoadAsset(
            candidate.collision, envelope, error))
        return false;
    candidate.collisionPayloadSha256 = envelope.payloadSha256;
    if (!ValidateModelRecord(candidate, error)
        || envelope.sourceStepSha256 != candidate.sourceStep.sha256
        || envelope.profileKeySha256 != candidate.collision.profileKeySha256
        || envelope.safetyMarginMicrometres
            != candidate.collision.safetyMarginMicrometres)
    {
        if (error.isEmpty())
            error = QStringLiteral("机器人型号与已验证碰撞简模身份不一致。");
        return false;
    }

    if (!EnsureStoreDirectory(error)) return false;
    QMutexLocker<QMutex> guard(&MutationMutex());
    const QString lockPath = ControlledPath(kMutationLockName);
    if (lockPath.isEmpty())
    {
        error = QStringLiteral("机器人型号目录锁路径解析失败。");
        return false;
    }
    const QFileInfo lockInfo(lockPath);
    if (IsLinkOrJunction(lockInfo)
        || (lockInfo.exists() && !lockInfo.isFile()))
    {
        error = QStringLiteral("机器人型号目录锁路径被非普通文件占用。");
        return false;
    }
    QLockFile lock(lockPath);
    if (!AcquireMutationLock(lock, error)) return false;
    Catalog catalog;
    if (!ReadCatalog(catalog, error)) return false;

    for (const ModelRecord& existing : catalog.models)
    {
        if (existing.modelId == candidate.modelId)
        {
            if (existing.sourceStep.sha256 != candidate.sourceStep.sha256)
            {
                error = QStringLiteral(
                    "型号 %1 已绑定其它 STEP revision；请使用显式升级流程，"
                    "不能静默替换。").arg(candidate.modelId);
                return false;
            }
            if (!ValidateTrustedModelIdentity(candidate, error)) return false;
            if (existing.adapterId != candidate.adapterId
                || existing.sourceRobotType != candidate.sourceRobotType
                || !SourceAssetsEqual(existing.sourceStep, candidate.sourceStep)
                || !CollisionAssetsEqual(existing.collision, candidate.collision)
                || existing.collisionPayloadSha256
                    != candidate.collisionPayloadSha256)
            {
                error = QStringLiteral(
                    "同一型号和 STEP revision 的适配器或简模记录不一致。");
                return false;
            }
            registeredModel = existing;
            return true;
        }
        if (existing.sourceStep.sha256 == candidate.sourceStep.sha256)
        {
            error = QStringLiteral("同一 STEP revision 已登记为型号 %1。")
                .arg(existing.modelId);
            return false;
        }
    }
    if (!ValidateTrustedModelIdentity(candidate, error)) return false;
    if (catalog.models.size() >= MaximumModels)
    {
        error = QStringLiteral("机器人型号目录已达到条目上限。");
        return false;
    }
    catalog.models.append(candidate);
    std::sort(
        catalog.models.begin(), catalog.models.end(),
        [](const ModelRecord& left, const ModelRecord& right)
        {
            return left.modelId < right.modelId;
        });
    if (catalog.revision == std::numeric_limits<qint64>::max())
    {
        error = QStringLiteral("机器人型号目录 revision 已耗尽。");
        return false;
    }
    ++catalog.revision;
    if (!WriteCatalog(catalog, error)) return false;

    // 原子提交后立即回读小型 catalog，防止向调用方泄漏未发布结果。
    Catalog verified;
    if (!ReadCatalog(verified, error)) return false;
    const auto found = std::find_if(
        verified.models.cbegin(), verified.models.cend(),
        [&candidate](const ModelRecord& value)
        {
            return value.modelId == candidate.modelId
                && value.sourceStep.sha256 == candidate.sourceStep.sha256;
        });
    if (found == verified.models.cend())
    {
        error = QStringLiteral("机器人型号目录提交后回读身份不一致。");
        return false;
    }
    registeredModel = *found;
    return true;
}

bool RobotModelCatalogStore::ResolveModelEligibility(
    const QString& modelId,
    int actualRobotType,
    Eligibility& eligibility,
    QString& error)
{
    eligibility = Eligibility();
    error.clear();
    const QString normalizedModelId = modelId.trimmed();
    if (!IsStableIdentifier(normalizedModelId))
    {
        eligibility.reason = QStringLiteral("控制单元未配置有效的机器人型号。");
        return true;
    }
    Catalog catalog;
    if (!ReadCatalog(catalog, error)) return false;
    const auto found = std::find_if(
        catalog.models.cbegin(), catalog.models.cend(),
        [&normalizedModelId](const ModelRecord& model)
        {
            return model.modelId == normalizedModelId;
        });
    if (found == catalog.models.cend())
    {
        eligibility.reason = QStringLiteral("机器人型号 %1 尚未导入适配模型。")
            .arg(normalizedModelId);
        return true;
    }
    eligibility.model = *found;
    QString trustedIdentityError;
    if (!ValidateTrustedModelIdentity(*found, trustedIdentityError))
    {
        eligibility.reason = QStringLiteral("机器人型号记录未受信：%1")
            .arg(trustedIdentityError);
        return true;
    }
    if (actualRobotType != found->sourceRobotType)
    {
        eligibility.reason = QStringLiteral(
            "控制单元机器人类型 %1 与型号库要求的类型 %2 不一致。")
            .arg(actualRobotType).arg(found->sourceRobotType);
        return true;
    }
    QString stepPath;
    QString assetError;
    if (!FastValidateSourceAsset(found->sourceStep, stepPath, assetError))
    {
        eligibility.reason = QStringLiteral("机器人总装资产不可用：%1")
            .arg(assetError);
        return true;
    }
    RobotCollisionEnvelopeStore::EnvelopeSet envelope;
    if (!ValidateCollisionAsset(*found, envelope, assetError))
    {
        eligibility.reason = QStringLiteral("机器人碰撞简模不可用：%1")
            .arg(assetError);
        return true;
    }
    eligibility.collisionEnvelope = envelope;
    eligibility.eligible = true;
    eligibility.reason = QStringLiteral("机器人型号及预生成碰撞简模可用。");
    return true;
}

bool RobotModelCatalogStore::BootstrapVerifiedSa10FromLegacy(
    ModelRecord& model,
    bool& registered,
    QString& error)
{
    model = ModelRecord();
    registered = false;
    error.clear();

    // Bootstrap 只允许首次迁移触碰 104 MB STEP。型号已经登记后走正常的
    // 小文件/大小快速门禁，管理窗口每次刷新都不会重新哈希总装。
    QList<ModelRecord> existingModels;
    if (!ListModels(existingModels, error)) return false;
    for (const ModelRecord& existing : existingModels)
    {
        if (existing.modelId != kVerifiedSa10ModelId) continue;
        if (existing.sourceStep.sha256 != kVerifiedSa10AssemblySha256
            || existing.adapterId != kVerifiedSa10AdapterId
            || existing.sourceRobotType != ROBOT_TYPE_STEP)
        {
            error = QStringLiteral(
                "现有 SA10 型号键已被其它 revision、适配器或机器人类型占用。");
            return false;
        }
        Eligibility eligibility;
        if (!ResolveModelEligibility(
                kVerifiedSa10ModelId, ROBOT_TYPE_STEP, eligibility, error))
            return false;
        if (!eligibility.eligible)
        {
            error = eligibility.reason;
            return false;
        }
        model = existing;
        registered = true;
        return true;
    }

    TheoreticalRobotModelStore::Asset active;
    bool hasActive = false;
    if (!TheoreticalRobotModelStore::ReadActiveRecord(
            active, hasActive, error))
        return false;
    if (!hasActive || active.sha256 != kVerifiedSa10AssemblySha256)
        return true;

    QString controlledStepPath;
    TheoreticalRobotModelStore::Asset verified;
    if (!TheoreticalRobotModelStore::ResolveActive(
            controlledStepPath, verified, error)
        || verified.sha256 != kVerifiedSa10AssemblySha256)
    {
        if (error.isEmpty())
            error = QStringLiteral("现有 SA10 active STEP 身份复核失败。");
        return false;
    }
    RobotCollisionEnvelopeStore::GenerationParameters generation;
    RobotCollisionEnvelopeStore::EnvelopeSet envelope;
    RobotCollisionEnvelopeStore::StoredAsset collision;
    if (!RobotCollisionEnvelopeStore::Load(
            verified.sha256, generation, envelope, collision, error))
        return false;
    if (!RegisterValidatedModel(
            kVerifiedSa10ModelId,
            kVerifiedSa10DisplayName,
            kVerifiedSa10AdapterId,
            ROBOT_TYPE_STEP,
            verified,
            collision,
            model,
            error))
        return false;
    registered = true;
    return true;
}
