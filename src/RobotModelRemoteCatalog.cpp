#include "RobotModelRemoteCatalog.h"

#include "AppPaths.h"
#include "TheoreticalRobotModelStore.h"

#include <QBuffer>
#include <QByteArrayView>
#include <QCryptographicHash>
#include <QDateTime>
#include <QFile>
#include <QFileInfo>
#include <QFontDatabase>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QPainter>
#include <QRegularExpression>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace
{
constexpr int kSchemaVersion = 1;
const QString kKind = QStringLiteral("robot-model-server-catalog");
const QString kRemoteRoot =
    QStringLiteral("/模型文件/机器人模型文件");
const QString kCatalogPrefix = QStringLiteral("catalog-v1-");
const QString kCatalogSuffix = QStringLiteral(".json");
const QString kOriginalStepPreviewKind =
    QStringLiteral("original-step-v1");
const QString kCollisionEnvelopePreviewKind =
    QStringLiteral("collision-envelope-v1");

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
    const double number =
        value.toDouble(std::numeric_limits<double>::quiet_NaN());
    return std::isfinite(number) && std::floor(number) == number
        && number >= static_cast<double>(minimum)
        && number <= static_cast<double>(maximum);
}

bool IsValidUtc(const QString& value)
{
    const QDateTime parsed = QDateTime::fromString(value, Qt::ISODateWithMs);
    return parsed.isValid() && parsed.timeSpec() == Qt::UTC;
}

bool ValidatePreviewRecord(
    const RobotModelRemoteCatalog::PreviewAsset& preview,
    QString& error)
{
    if ((preview.sourceKind != kOriginalStepPreviewKind
            && preview.sourceKind != kCollisionEnvelopePreviewKind)
        || !IsLowerSha256(preview.sha256)
        || preview.storedFileName
            != QStringLiteral("preview-%1.png").arg(preview.sha256)
        || !AppPaths::IsSafePathComponent(preview.storedFileName)
        || preview.sizeBytes <= 0
        || preview.sizeBytes > RobotModelRemoteCatalog::MaximumPreviewBytes
        || preview.width != RobotModelRemoteCatalog::PreviewWidth
        || preview.height != RobotModelRemoteCatalog::PreviewHeight)
    {
        error = QStringLiteral("服务器机器人模型预览图记录无效。");
        return false;
    }
    return true;
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
        error = QStringLiteral("服务器机器人模型 STEP 记录无效。");
        return false;
    }
    return true;
}

bool ValidateCollisionRecord(
    const RobotModelCatalogStore::ModelRecord& model,
    QString& error)
{
    const auto& collision = model.collision;
    if (!IsLowerSha256(collision.profileKeySha256)
        || !IsLowerSha256(collision.sourceStepSha256)
        || collision.storedFileName
            != collision.profileKeySha256
                + QStringLiteral(".robot-aabb.json")
        || !AppPaths::IsSafePathComponent(collision.storedFileName)
        || collision.sizeBytes <= 0
        || collision.sizeBytes
            > RobotCollisionEnvelopeStore::MaximumEnvelopeFileBytes
        || collision.safetyMarginMicrometres < 0
        || collision.safetyMarginMicrometres
            > static_cast<qint64>(
                RobotCollisionEnvelopeStore::MaximumSafetyMarginMm * 1000.0)
        || !IsLowerSha256(model.collisionPayloadSha256)
        || collision.sourceStepSha256 != model.sourceStep.sha256)
    {
        error = QStringLiteral("服务器机器人碰撞简模记录无效。");
        return false;
    }
    return true;
}

bool ValidateModelRecord(
    const RobotModelRemoteCatalog::ModelRecord& record,
    QString& error)
{
    const auto& model = record.model;
    if (!IsStableIdentifier(model.modelId)
        || !IsSafeText(model.displayName, 256)
        || !IsStableIdentifier(model.adapterId)
        || model.sourceRobotType <= 0
        || model.sourceRobotType > 1000000
        || !IsValidUtc(model.registeredUtc)
        || !ValidateSourceRecord(model.sourceStep, error)
        || !ValidateCollisionRecord(model, error)
        || !ValidatePreviewRecord(record.preview, error))
    {
        if (error.isEmpty())
            error = QStringLiteral("服务器机器人型号记录字段无效。");
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
    const RobotModelCatalogStore::ModelRecord& model)
{
    const auto& asset = model.collision;
    return QJsonObject{
        { QStringLiteral("profileKeySha256"), asset.profileKeySha256 },
        { QStringLiteral("sourceStepSha256"), asset.sourceStepSha256 },
        { QStringLiteral("storedFileName"), asset.storedFileName },
        { QStringLiteral("sizeBytes"), asset.sizeBytes },
        { QStringLiteral("safetyMarginMicrometres"),
          asset.safetyMarginMicrometres },
        { QStringLiteral("payloadSha256"), model.collisionPayloadSha256 }
    };
}

QJsonObject PreviewToJson(
    const RobotModelRemoteCatalog::PreviewAsset& preview)
{
    QJsonObject object{
        { QStringLiteral("sha256"), preview.sha256 },
        { QStringLiteral("storedFileName"), preview.storedFileName },
        { QStringLiteral("sizeBytes"), preview.sizeBytes },
        { QStringLiteral("width"), preview.width },
        { QStringLiteral("height"), preview.height }
    };
    if (preview.sourceKindDeclared)
        object.insert(QStringLiteral("sourceKind"), preview.sourceKind);
    return object;
}

QJsonObject ModelToJson(
    const RobotModelRemoteCatalog::ModelRecord& record)
{
    const auto& model = record.model;
    return QJsonObject{
        { QStringLiteral("modelId"), model.modelId },
        { QStringLiteral("displayName"), model.displayName },
        { QStringLiteral("adapterId"), model.adapterId },
        { QStringLiteral("sourceRobotType"), model.sourceRobotType },
        { QStringLiteral("sourceStep"), SourceToJson(model.sourceStep) },
        { QStringLiteral("collision"), CollisionToJson(model) },
        { QStringLiteral("preview"), PreviewToJson(record.preview) },
        { QStringLiteral("registeredUtc"), model.registeredUtc }
    };
}

QJsonObject CatalogPayload(
    const RobotModelRemoteCatalog::Catalog& catalog)
{
    QJsonArray models;
    for (const auto& model : catalog.models) models.append(ModelToJson(model));
    return QJsonObject{
        { QStringLiteral("schemaVersion"), kSchemaVersion },
        { QStringLiteral("kind"), kKind },
        { QStringLiteral("revisionUtcMs"), catalog.revisionUtcMs },
        { QStringLiteral("publishedUtc"), catalog.publishedUtc },
        { QStringLiteral("previousCatalogSha256"),
          catalog.previousCatalogSha256 },
        { QStringLiteral("models"), models }
    };
}

bool ParseSource(
    const QJsonValue& value,
    TheoreticalRobotModelStore::Asset& asset,
    QString& error)
{
    asset = TheoreticalRobotModelStore::Asset();
    if (!value.isObject())
    {
        error = QStringLiteral("服务器机器人 STEP 记录不是对象。");
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
        error = QStringLiteral("服务器机器人 STEP schema 无效。");
        return false;
    }
    asset.sha256 = object.value(QStringLiteral("sha256")).toString();
    asset.storedFileName =
        object.value(QStringLiteral("storedFileName")).toString();
    asset.originalDisplayName =
        object.value(QStringLiteral("originalDisplayName")).toString();
    asset.sizeBytes =
        object.value(QStringLiteral("sizeBytes")).toInteger(-1);
    asset.importedUtc =
        object.value(QStringLiteral("importedUtc")).toString();
    return ValidateSourceRecord(asset, error);
}

bool ParseCollision(
    const QJsonValue& value,
    RobotModelCatalogStore::ModelRecord& model,
    QString& error)
{
    if (!value.isObject())
    {
        error = QStringLiteral("服务器机器人简模记录不是对象。");
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
        error = QStringLiteral("服务器机器人简模 schema 无效。");
        return false;
    }
    model.collision.profileKeySha256 =
        object.value(QStringLiteral("profileKeySha256")).toString();
    model.collision.sourceStepSha256 =
        object.value(QStringLiteral("sourceStepSha256")).toString();
    model.collision.storedFileName =
        object.value(QStringLiteral("storedFileName")).toString();
    model.collision.sizeBytes =
        object.value(QStringLiteral("sizeBytes")).toInteger(-1);
    model.collision.safetyMarginMicrometres =
        object.value(QStringLiteral("safetyMarginMicrometres"))
            .toInteger(-1);
    model.collisionPayloadSha256 =
        object.value(QStringLiteral("payloadSha256")).toString();
    return ValidateCollisionRecord(model, error);
}

bool ParsePreview(
    const QJsonValue& value,
    RobotModelRemoteCatalog::PreviewAsset& preview,
    QString& error)
{
    preview = RobotModelRemoteCatalog::PreviewAsset();
    if (!value.isObject())
    {
        error = QStringLiteral("服务器机器人预览图记录不是对象。");
        return false;
    }
    const QJsonObject object = value.toObject();
    const bool legacySchema = HasExactKeys(object, {
        QStringLiteral("sha256"), QStringLiteral("storedFileName"),
        QStringLiteral("sizeBytes"), QStringLiteral("width"),
        QStringLiteral("height") });
    const bool typedSchema = HasExactKeys(object, {
        QStringLiteral("sourceKind"),
        QStringLiteral("sha256"), QStringLiteral("storedFileName"),
        QStringLiteral("sizeBytes"), QStringLiteral("width"),
        QStringLiteral("height") });
    if ((!legacySchema && !typedSchema)
        || (typedSchema
            && !object.value(QStringLiteral("sourceKind")).isString())
        || !object.value(QStringLiteral("sha256")).isString()
        || !object.value(QStringLiteral("storedFileName")).isString()
        || !IsExactInteger(
            object.value(QStringLiteral("sizeBytes")), 1,
            RobotModelRemoteCatalog::MaximumPreviewBytes)
        || !IsExactInteger(
            object.value(QStringLiteral("width")),
            RobotModelRemoteCatalog::PreviewWidth,
            RobotModelRemoteCatalog::PreviewWidth)
        || !IsExactInteger(
            object.value(QStringLiteral("height")),
            RobotModelRemoteCatalog::PreviewHeight,
            RobotModelRemoteCatalog::PreviewHeight))
    {
        error = QStringLiteral("服务器机器人预览图 schema 无效。");
        return false;
    }
    preview.sourceKind = legacySchema
        ? kCollisionEnvelopePreviewKind
        : object.value(QStringLiteral("sourceKind")).toString();
    preview.sourceKindDeclared = typedSchema;
    preview.sha256 = object.value(QStringLiteral("sha256")).toString();
    preview.storedFileName =
        object.value(QStringLiteral("storedFileName")).toString();
    preview.sizeBytes =
        object.value(QStringLiteral("sizeBytes")).toInteger(-1);
    preview.width = object.value(QStringLiteral("width")).toInt(-1);
    preview.height = object.value(QStringLiteral("height")).toInt(-1);
    return ValidatePreviewRecord(preview, error);
}

bool ParseModel(
    const QJsonValue& value,
    RobotModelRemoteCatalog::ModelRecord& record,
    QString& error)
{
    record = RobotModelRemoteCatalog::ModelRecord();
    if (!value.isObject())
    {
        error = QStringLiteral("服务器机器人型号条目不是对象。");
        return false;
    }
    const QJsonObject object = value.toObject();
    if (!HasExactKeys(object, {
            QStringLiteral("modelId"), QStringLiteral("displayName"),
            QStringLiteral("adapterId"), QStringLiteral("sourceRobotType"),
            QStringLiteral("sourceStep"), QStringLiteral("collision"),
            QStringLiteral("preview"), QStringLiteral("registeredUtc") })
        || !object.value(QStringLiteral("modelId")).isString()
        || !object.value(QStringLiteral("displayName")).isString()
        || !object.value(QStringLiteral("adapterId")).isString()
        || !IsExactInteger(
            object.value(QStringLiteral("sourceRobotType")), 1, 1000000)
        || !object.value(QStringLiteral("registeredUtc")).isString())
    {
        error = QStringLiteral("服务器机器人型号 schema 无效。");
        return false;
    }
    auto& model = record.model;
    model.modelId = object.value(QStringLiteral("modelId")).toString();
    model.displayName = object.value(QStringLiteral("displayName")).toString();
    model.adapterId = object.value(QStringLiteral("adapterId")).toString();
    model.sourceRobotType =
        object.value(QStringLiteral("sourceRobotType")).toInt(-1);
    model.registeredUtc =
        object.value(QStringLiteral("registeredUtc")).toString();
    return ParseSource(
            object.value(QStringLiteral("sourceStep")),
            model.sourceStep, error)
        && ParseCollision(
            object.value(QStringLiteral("collision")), model, error)
        && ParsePreview(
            object.value(QStringLiteral("preview")), record.preview, error)
        && ValidateModelRecord(record, error);
}

struct ProjectedPoint
{
    QPointF point;
    double depth = 0.0;
};

ProjectedPoint Project(
    const Eigen::Vector3d& value,
    const Eigen::Vector3d& center)
{
    const Eigen::Vector3d p = value - center;
    return {
        QPointF(
            0.866025403784 * (p.x() - p.z()),
            -p.y() + 0.5 * (p.x() + p.z())),
        p.x() + p.y() + p.z()
    };
}

QFont PreviewFont(const QFont& base)
{
    QFont font(base);
    const QStringList families = QFontDatabase::families();
    const QStringList candidates = {
        QStringLiteral("Microsoft YaHei UI"),
        QStringLiteral("Microsoft YaHei"),
        QStringLiteral("微软雅黑"),
        QStringLiteral("SimHei"),
        QStringLiteral("黑体"),
        QStringLiteral("SimSun"),
        QStringLiteral("宋体"),
        QStringLiteral("Noto Sans CJK SC"),
        QStringLiteral("Arial Unicode MS"),
        QStringLiteral("Arial")
    };
    for (const QString& candidate : candidates)
    {
        if (!families.contains(candidate, Qt::CaseInsensitive))
            continue;
        font.setFamily(candidate);
        break;
    }
    return font;
}
}

QString RobotModelRemoteCatalog::RemoteRoot()
{
    return kRemoteRoot;
}

QString RobotModelRemoteCatalog::RemoteAssetsDirectory()
{
    return kRemoteRoot + QStringLiteral("/assets");
}

QString RobotModelRemoteCatalog::RemoteCollisionDirectory()
{
    return kRemoteRoot + QStringLiteral("/collision");
}

QString RobotModelRemoteCatalog::RemotePreviewDirectory()
{
    return kRemoteRoot + QStringLiteral("/previews");
}

QString RobotModelRemoteCatalog::RemoteCatalogDirectory()
{
    return kRemoteRoot + QStringLiteral("/catalog");
}

QString RobotModelRemoteCatalog::OriginalStepPreviewKind()
{
    return kOriginalStepPreviewKind;
}

QString RobotModelRemoteCatalog::CollisionEnvelopePreviewKind()
{
    return kCollisionEnvelopePreviewKind;
}

QString RobotModelRemoteCatalog::CatalogFileName(const Catalog& catalog)
{
    if (catalog.revisionUtcMs <= 0
        || !IsLowerSha256(catalog.payloadSha256))
        return QString();
    return QStringLiteral("%1%2-%3%4")
        .arg(kCatalogPrefix)
        .arg(catalog.revisionUtcMs, 13, 10, QLatin1Char('0'))
        .arg(catalog.payloadSha256)
        .arg(kCatalogSuffix);
}

bool RobotModelRemoteCatalog::ParseCatalogFileName(
    const QString& fileName,
    qint64& revisionUtcMs,
    QString& payloadSha256)
{
    revisionUtcMs = 0;
    payloadSha256.clear();
    static const QRegularExpression pattern(
        QStringLiteral(
            R"(^catalog-v1-([0-9]{13})-([0-9a-f]{64})\.json$)"));
    const QRegularExpressionMatch match = pattern.match(fileName);
    if (!match.hasMatch()) return false;
    bool revisionOk = false;
    const qint64 revision = match.captured(1).toLongLong(&revisionOk);
    if (!revisionOk || revision <= 0) return false;
    revisionUtcMs = revision;
    payloadSha256 = match.captured(2);
    return true;
}

bool RobotModelRemoteCatalog::Serialize(
    Catalog catalog,
    QByteArray& bytes,
    Catalog& normalized,
    QString& error)
{
    bytes.clear();
    normalized = Catalog();
    error.clear();
    if (catalog.revisionUtcMs <= 0
        || !IsValidUtc(catalog.publishedUtc)
        || (!catalog.previousCatalogSha256.isEmpty()
            && !IsLowerSha256(catalog.previousCatalogSha256))
        || catalog.models.size() > RobotModelCatalogStore::MaximumModels)
    {
        error = QStringLiteral("服务器机器人模型清单头字段无效。");
        return false;
    }
    std::sort(
        catalog.models.begin(), catalog.models.end(),
        [](const ModelRecord& left, const ModelRecord& right)
        {
            return left.model.modelId < right.model.modelId;
        });
    QString previousId;
    for (const ModelRecord& record : catalog.models)
    {
        if (!ValidateModelRecord(record, error)
            || record.model.modelId == previousId)
        {
            if (error.isEmpty())
                error = QStringLiteral("服务器机器人模型清单包含重复型号。");
            return false;
        }
        previousId = record.model.modelId;
    }
    const QJsonObject payload = CatalogPayload(catalog);
    catalog.payloadSha256 = Sha256(
        QJsonDocument(payload).toJson(QJsonDocument::Compact));
    QJsonObject root = payload;
    root.insert(QStringLiteral("payloadSha256"), catalog.payloadSha256);
    bytes = QJsonDocument(root).toJson(QJsonDocument::Compact);
    if (bytes.isEmpty() || bytes.size() > MaximumCatalogBytes)
    {
        bytes.clear();
        error = QStringLiteral("服务器机器人模型清单超过大小上限。");
        return false;
    }
    normalized = catalog;
    return true;
}

bool RobotModelRemoteCatalog::Parse(
    const QByteArray& bytes,
    Catalog& catalog,
    QString& error)
{
    catalog = Catalog();
    error.clear();
    if (bytes.isEmpty() || bytes.size() > MaximumCatalogBytes)
    {
        error = QStringLiteral("服务器机器人模型清单为空或超过大小上限。");
        return false;
    }
    QJsonParseError parseError;
    const QJsonDocument document = QJsonDocument::fromJson(bytes, &parseError);
    if (parseError.error != QJsonParseError::NoError || !document.isObject())
    {
        error = QStringLiteral("服务器机器人模型清单 JSON 无效：%1")
            .arg(parseError.errorString());
        return false;
    }
    const QJsonObject root = document.object();
    if (!HasExactKeys(root, {
            QStringLiteral("schemaVersion"), QStringLiteral("kind"),
            QStringLiteral("revisionUtcMs"), QStringLiteral("publishedUtc"),
            QStringLiteral("previousCatalogSha256"),
            QStringLiteral("models"), QStringLiteral("payloadSha256") })
        || !IsExactInteger(
            root.value(QStringLiteral("schemaVersion")),
            kSchemaVersion, kSchemaVersion)
        || root.value(QStringLiteral("kind")).toString() != kKind
        || !IsExactInteger(
            root.value(QStringLiteral("revisionUtcMs")), 1,
            std::numeric_limits<qint64>::max())
        || !root.value(QStringLiteral("publishedUtc")).isString()
        || !root.value(QStringLiteral("previousCatalogSha256")).isString()
        || !root.value(QStringLiteral("models")).isArray()
        || !root.value(QStringLiteral("payloadSha256")).isString())
    {
        error = QStringLiteral("服务器机器人模型清单 schema 无效。");
        return false;
    }

    Catalog parsed;
    parsed.revisionUtcMs =
        root.value(QStringLiteral("revisionUtcMs")).toInteger(0);
    parsed.publishedUtc =
        root.value(QStringLiteral("publishedUtc")).toString();
    parsed.previousCatalogSha256 =
        root.value(QStringLiteral("previousCatalogSha256")).toString();
    parsed.payloadSha256 =
        root.value(QStringLiteral("payloadSha256")).toString();
    if (!IsValidUtc(parsed.publishedUtc)
        || (!parsed.previousCatalogSha256.isEmpty()
            && !IsLowerSha256(parsed.previousCatalogSha256))
        || !IsLowerSha256(parsed.payloadSha256))
    {
        error = QStringLiteral("服务器机器人模型清单身份字段无效。");
        return false;
    }
    const QJsonArray models = root.value(QStringLiteral("models")).toArray();
    if (models.size() > RobotModelCatalogStore::MaximumModels)
    {
        error = QStringLiteral("服务器机器人模型清单条目过多。");
        return false;
    }
    for (const QJsonValue& value : models)
    {
        ModelRecord record;
        if (!ParseModel(value, record, error)) return false;
        parsed.models.append(record);
    }
    QList<ModelRecord> sorted = parsed.models;
    std::sort(
        sorted.begin(), sorted.end(),
        [](const ModelRecord& left, const ModelRecord& right)
        {
            return left.model.modelId < right.model.modelId;
        });
    for (qsizetype index = 0; index < sorted.size(); ++index)
    {
        if (sorted.at(index).model.modelId
                != parsed.models.at(index).model.modelId
            || (index > 0
                && sorted.at(index - 1).model.modelId
                    == sorted.at(index).model.modelId))
        {
            error = QStringLiteral(
                "服务器机器人模型清单未按型号排序或包含重复型号。");
            return false;
        }
    }
    const QString expectedPayload =
        Sha256(QJsonDocument(CatalogPayload(parsed))
            .toJson(QJsonDocument::Compact));
    if (expectedPayload != parsed.payloadSha256)
    {
        error = QStringLiteral(
            "服务器机器人模型清单 payload SHA-256 校验失败。");
        return false;
    }
    catalog = parsed;
    return true;
}

bool RobotModelRemoteCatalog::RenderPreview(
    const RobotCollisionEnvelopeStore::EnvelopeSet& envelope,
    const QString& displayName,
    QImage& image,
    QString& error)
{
    image = QImage();
    error.clear();
    if (!RobotCollisionEnvelopeStore::ValidateForUse(envelope, error))
        return false;
    if (!IsSafeText(displayName, 256))
    {
        error = QStringLiteral("机器人预览名称无效。");
        return false;
    }

    QImage rendered(
        PreviewWidth, PreviewHeight, QImage::Format_ARGB32_Premultiplied);
    rendered.fill(QColor(8, 24, 34));
    QPainter painter(&rendered);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setRenderHint(QPainter::TextAntialiasing, true);
    painter.setFont(PreviewFont(painter.font()));

    QLinearGradient background(0, 0, 0, PreviewHeight);
    background.setColorAt(0.0, QColor(14, 55, 72));
    background.setColorAt(1.0, QColor(4, 15, 23));
    painter.fillRect(rendered.rect(), background);

    const Eigen::Vector3d minimum =
        envelope.overallExpandedBoundsMm.minimumMm;
    const Eigen::Vector3d maximum =
        envelope.overallExpandedBoundsMm.maximumMm;
    const Eigen::Vector3d center = (minimum + maximum) * 0.5;
    std::array<QPointF, 8> projectedOverall{};
    int pointIndex = 0;
    for (int x = 0; x < 2; ++x)
        for (int y = 0; y < 2; ++y)
            for (int z = 0; z < 2; ++z)
            {
                const Eigen::Vector3d point(
                    x ? maximum.x() : minimum.x(),
                    y ? maximum.y() : minimum.y(),
                    z ? maximum.z() : minimum.z());
                projectedOverall[static_cast<size_t>(pointIndex++)] =
                    Project(point, center).point;
            }
    QRectF sourceBounds;
    for (const QPointF& point : projectedOverall)
        sourceBounds |= QRectF(point, QSizeF(0.01, 0.01));
    if (sourceBounds.width() <= 1.0e-9 || sourceBounds.height() <= 1.0e-9)
    {
        error = QStringLiteral("机器人简模投影范围退化，无法生成预览。");
        return false;
    }
    const QRectF target(38.0, 44.0, PreviewWidth - 76.0, PreviewHeight - 82.0);
    const double scale = std::min(
        target.width() / sourceBounds.width(),
        target.height() / sourceBounds.height());
    const QPointF targetCenter = target.center();
    const QPointF sourceCenter = sourceBounds.center();
    auto mapPoint = [&](const QPointF& point)
    {
        return targetCenter + (point - sourceCenter) * scale;
    };

    struct JointDrawing
    {
        int jointIndex = -1;
        double depth = 0.0;
        std::array<QPointF, 8> points{};
    };
    QList<JointDrawing> drawings;
    for (const auto& joint : envelope.joints)
    {
        JointDrawing drawing;
        drawing.jointIndex = joint.jointIndex;
        const auto& bounds = joint.sourceBoundsMm;
        int index = 0;
        double depth = 0.0;
        for (int x = 0; x < 2; ++x)
            for (int y = 0; y < 2; ++y)
                for (int z = 0; z < 2; ++z)
                {
                    const Eigen::Vector3d point(
                        x ? bounds.maximumMm.x() : bounds.minimumMm.x(),
                        y ? bounds.maximumMm.y() : bounds.minimumMm.y(),
                        z ? bounds.maximumMm.z() : bounds.minimumMm.z());
                    const ProjectedPoint projected = Project(point, center);
                    drawing.points[static_cast<size_t>(index++)] =
                        mapPoint(projected.point);
                    depth += projected.depth;
                }
        drawing.depth = depth / 8.0;
        drawings.append(drawing);
    }
    std::sort(
        drawings.begin(), drawings.end(),
        [](const JointDrawing& left, const JointDrawing& right)
        {
            return left.depth < right.depth;
        });

    static const std::array<QColor, 7> colors = {
        QColor(38, 198, 218), QColor(102, 187, 106),
        QColor(255, 202, 40), QColor(255, 112, 67),
        QColor(171, 71, 188), QColor(66, 165, 245),
        QColor(236, 64, 122)
    };
    static const std::array<std::array<int, 2>, 12> edges = {{
        {{0, 1}}, {{0, 2}}, {{0, 4}}, {{1, 3}},
        {{1, 5}}, {{2, 3}}, {{2, 6}}, {{3, 7}},
        {{4, 5}}, {{4, 6}}, {{5, 7}}, {{6, 7}}
    }};
    for (const JointDrawing& drawing : drawings)
    {
        const QColor color =
            colors.at(static_cast<size_t>(drawing.jointIndex));
        QPolygonF hull;
        for (const QPointF& point : drawing.points) hull << point;
        painter.setBrush(QColor(color.red(), color.green(), color.blue(), 22));
        painter.setPen(QPen(QColor(color.red(), color.green(), color.blue(), 80), 1.0));
        painter.drawRect(hull.boundingRect());
        painter.setBrush(Qt::NoBrush);
        painter.setPen(QPen(color, 2.0));
        for (const auto& edge : edges)
            painter.drawLine(
                drawing.points.at(static_cast<size_t>(edge[0])),
                drawing.points.at(static_cast<size_t>(edge[1])));
        QPointF jointCenter;
        for (const QPointF& point : drawing.points) jointCenter += point;
        jointCenter /= 8.0;
        painter.setBrush(color);
        painter.setPen(QPen(QColor(245, 251, 255), 1.0));
        painter.drawEllipse(jointCenter, 11.0, 11.0);
        painter.setPen(QColor(4, 15, 23));
        QFont jointFont = painter.font();
        jointFont.setBold(true);
        jointFont.setPointSize(8);
        painter.setFont(jointFont);
        painter.drawText(
            QRectF(jointCenter.x() - 12.0, jointCenter.y() - 10.0, 24.0, 20.0),
            Qt::AlignCenter,
            QStringLiteral("J%1").arg(drawing.jointIndex));
    }

    painter.setPen(QColor(226, 242, 248));
    QFont titleFont = painter.font();
    titleFont.setBold(true);
    titleFont.setPointSize(12);
    painter.setFont(titleFont);
    painter.drawText(
        QRectF(18.0, 10.0, PreviewWidth - 36.0, 28.0),
        Qt::AlignLeft | Qt::AlignVCenter,
        displayName);
    QFont noteFont = painter.font();
    noteFont.setBold(false);
    noteFont.setPointSize(8);
    painter.setFont(noteFont);
    painter.setPen(QColor(144, 202, 224));
    painter.drawText(
        QRectF(18.0, PreviewHeight - 30.0, PreviewWidth - 36.0, 20.0),
        Qt::AlignLeft | Qt::AlignVCenter,
        QStringLiteral("J0–J6 碰撞简模预览 · %1 mm 安全余量")
            .arg(
                static_cast<double>(envelope.safetyMarginMicrometres)
                    / 1000.0,
                0, 'f', 1));
    painter.end();
    image = rendered;
    return true;
}

bool RobotModelRemoteCatalog::EncodePreview(
    const QImage& image,
    const QString& sourceKind,
    QByteArray& pngBytes,
    PreviewAsset& asset,
    QString& error)
{
    pngBytes.clear();
    asset = PreviewAsset();
    error.clear();
    if ((sourceKind != kOriginalStepPreviewKind
            && sourceKind != kCollisionEnvelopePreviewKind)
        || image.isNull()
        || image.width() != PreviewWidth
        || image.height() != PreviewHeight)
    {
        error = QStringLiteral("机器人预览图尺寸无效。");
        return false;
    }
    QBuffer buffer(&pngBytes);
    if (!buffer.open(QIODevice::WriteOnly)
        || !image.save(&buffer, "PNG", 9)
        || pngBytes.isEmpty()
        || pngBytes.size() > MaximumPreviewBytes)
    {
        pngBytes.clear();
        error = QStringLiteral("机器人预览图 PNG 编码失败或超过大小上限。");
        return false;
    }
    asset.sourceKind = sourceKind;
    asset.sourceKindDeclared = true;
    asset.sha256 = Sha256(pngBytes);
    asset.storedFileName =
        QStringLiteral("preview-%1.png").arg(asset.sha256);
    asset.sizeBytes = pngBytes.size();
    asset.width = image.width();
    asset.height = image.height();
    return ValidatePreviewRecord(asset, error);
}

bool RobotModelRemoteCatalog::ValidatePreview(
    const QByteArray& pngBytes,
    const PreviewAsset& expected,
    QImage& image,
    QString& error)
{
    image = QImage();
    error.clear();
    if (!ValidatePreviewRecord(expected, error)
        || pngBytes.size() != expected.sizeBytes
        || Sha256(pngBytes) != expected.sha256)
    {
        if (error.isEmpty())
            error = QStringLiteral("机器人预览图大小或 SHA-256 不一致。");
        return false;
    }
    QImage decoded = QImage::fromData(pngBytes, "PNG");
    if (decoded.isNull()
        || decoded.width() != expected.width
        || decoded.height() != expected.height)
    {
        error = QStringLiteral("机器人预览图不是声明尺寸的有效 PNG。");
        return false;
    }
    image = decoded;
    return true;
}

QString RobotModelRemoteCatalog::Sha256(const QByteArray& bytes)
{
    return QString::fromLatin1(
        QCryptographicHash::hash(bytes, QCryptographicHash::Sha256).toHex());
}

bool RobotModelRemoteCatalog::HashFileStable(
    const QString& path,
    qint64 maximumBytes,
    QString& sha256,
    qint64& sizeBytes,
    QString& error)
{
    sha256.clear();
    sizeBytes = 0;
    error.clear();
    const QFileInfo before(path);
    if (!before.exists() || !before.isFile()
#ifdef Q_OS_WIN
        || before.isSymLink() || before.isJunction()
#else
        || before.isSymLink()
#endif
        || before.size() <= 0 || before.size() > maximumBytes)
    {
        error = QStringLiteral("待传模型资产不是受控范围内的普通文件。");
        return false;
    }
    const qint64 expectedModified =
        before.lastModified().toMSecsSinceEpoch();
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly) || file.size() != before.size())
    {
        error = QStringLiteral("无法稳定打开待传模型资产：%1")
            .arg(file.errorString());
        return false;
    }
    QCryptographicHash hash(QCryptographicHash::Sha256);
    QByteArray block(1024 * 1024, Qt::Uninitialized);
    qint64 total = 0;
    while (!file.atEnd())
    {
        const qint64 count = file.read(block.data(), block.size());
        if (count < 0)
        {
            error = QStringLiteral("读取待传模型资产失败：%1")
                .arg(file.errorString());
            return false;
        }
        if (count == 0) continue;
        total += count;
        if (total > before.size() || total > maximumBytes)
        {
            error = QStringLiteral("待传模型资产读取长度越界。");
            return false;
        }
        hash.addData(
            QByteArrayView(block.constData(), static_cast<qsizetype>(count)));
    }
    file.close();
    QFileInfo after(path);
    after.refresh();
    if (total != before.size()
        || !after.exists() || !after.isFile()
#ifdef Q_OS_WIN
        || after.isSymLink() || after.isJunction()
#else
        || after.isSymLink()
#endif
        || after.size() != before.size()
        || after.lastModified().toMSecsSinceEpoch() != expectedModified)
    {
        error = QStringLiteral("待传模型资产在读取过程中发生变化。");
        return false;
    }
    sha256 = QString::fromLatin1(hash.result().toHex());
    sizeBytes = total;
    return true;
}
