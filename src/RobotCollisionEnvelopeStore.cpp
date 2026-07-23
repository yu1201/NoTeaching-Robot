#include "RobotCollisionEnvelopeStore.h"

#include "AppPaths.h"

#include <QByteArrayView>
#include <QCryptographicHash>
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
#include <array>
#include <cmath>
#include <limits>

namespace
{
constexpr int kSchemaVersion = 1;
constexpr qint64 kMicrometresPerMillimetre = 1000;
const QString kStoreRelativePath = QStringLiteral("Data/RobotModels");
const QString kFileSuffix = QStringLiteral(".robot-aabb.json");
const QString kMutationLockName = QStringLiteral("collision-envelope.lock");
const QString kKind = QStringLiteral("robot-joint-collision-envelope");
const QString kAlgorithm = QStringLiteral("source-axis-aligned-bounding-box");
const QString kLengthUnit = QStringLiteral("mm");
const QString kCoordinateFrame = QStringLiteral("sourceAssembly");

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

bool HasExactKeys(const QJsonObject& object, const std::initializer_list<QString>& keys)
{
    if (object.size() != static_cast<qsizetype>(keys.size())) return false;
    for (const QString& key : keys)
    {
        if (!object.contains(key)) return false;
    }
    return true;
}

bool IsSafeText(const QString& value, qsizetype maximumLength, bool allowEmpty = false)
{
    if ((!allowEmpty && value.isEmpty()) || value.size() > maximumLength) return false;
    for (const QChar ch : value)
    {
        if (ch.unicode() < 0x20 || ch.unicode() == 0x7f) return false;
    }
    return true;
}

bool IsFiniteBounds(const RobotCollisionEnvelopeStore::AxisAlignedBounds& bounds)
{
    for (int axis = 0; axis < 3; ++axis)
    {
        if (!std::isfinite(bounds.minimumMm[axis])
            || !std::isfinite(bounds.maximumMm[axis])
            || bounds.minimumMm[axis] > bounds.maximumMm[axis])
            return false;
    }
    return true;
}

bool MarginMicrometres(
    const RobotCollisionEnvelopeStore::GenerationParameters& parameters,
    qint64& value,
    QString& error)
{
    value = 0;
    if (!std::isfinite(parameters.safetyMarginMm)
        || parameters.safetyMarginMm < 0.0
        || parameters.safetyMarginMm
            > RobotCollisionEnvelopeStore::MaximumSafetyMarginMm)
    {
        error = QStringLiteral("机器人碰撞安全余量必须在 0 到 %1 mm 之间。")
            .arg(RobotCollisionEnvelopeStore::MaximumSafetyMarginMm);
        return false;
    }
    const double scaled = parameters.safetyMarginMm * kMicrometresPerMillimetre;
    const qint64 rounded = static_cast<qint64>(std::llround(scaled));
    if (std::abs(scaled - static_cast<double>(rounded)) > 1.0e-7)
    {
        error = QStringLiteral("机器人碰撞安全余量最多保留 0.001 mm 精度。");
        return false;
    }
    value = rounded;
    return true;
}

QString ProfileKey(const QString& sourceSha256, qint64 marginMicrometres)
{
    if (!IsLowerSha256(sourceSha256)
        || marginMicrometres < 0
        || marginMicrometres
            > static_cast<qint64>(RobotCollisionEnvelopeStore::MaximumSafetyMarginMm
                                  * kMicrometresPerMillimetre))
        return QString();
    QByteArray identity("robot-joint-source-aabb-v1\0", 27);
    identity.append(sourceSha256.toLatin1());
    identity.append('\0');
    identity.append(QByteArray::number(marginMicrometres));
    return QString::fromLatin1(
        QCryptographicHash::hash(identity, QCryptographicHash::Sha256).toHex());
}

QString FileNameForProfile(const QString& profileKey)
{
    return IsLowerSha256(profileKey) ? profileKey + kFileSuffix : QString();
}

bool IsMillimetreUnit(const QString& raw)
{
    const QString value = raw.trimmed().toLower();
    return value == QStringLiteral("mm")
        || value == QStringLiteral("millimetre")
        || value == QStringLiteral("millimetres")
        || value == QStringLiteral("millimeter")
        || value == QStringLiteral("millimeters");
}

bool EnsureStoreDirectory(QString& error)
{
    const QString store = RobotCollisionEnvelopeStore::StoreDirectory();
    if (store.isEmpty())
    {
        error = QStringLiteral("无法解析机器人碰撞包络库目录。");
        return false;
    }
    const QString dataRoot = QDir::cleanPath(AppPaths::DataRootPath());
    const QString dataDirectory = QDir(dataRoot).filePath(QStringLiteral("Data"));
    for (const QString& component : { dataDirectory, store })
    {
        const QFileInfo info(component);
        if (IsLinkOrJunction(info))
        {
            error = QStringLiteral("机器人碰撞包络库路径包含符号链接或联接点：%1")
                .arg(QDir::toNativeSeparators(component));
            return false;
        }
    }
    QFileInfo storeInfo(store);
    if (storeInfo.exists() && (!storeInfo.isDir() || IsLinkOrJunction(storeInfo)))
    {
        error = QStringLiteral("机器人碰撞包络库路径不是普通目录。");
        return false;
    }
    if (!storeInfo.exists() && !QDir().mkpath(store))
    {
        error = QStringLiteral("无法创建机器人碰撞包络库。");
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
        error = QStringLiteral("机器人碰撞包络库真实路径越出数据根目录。");
        return false;
    }
    return true;
}

QString StoredPath(const QString& fileName)
{
    if (!AppPaths::IsSafePathComponent(fileName)) return QString();
    const QString path = AppPaths::WritableChildPath(kStoreRelativePath, fileName);
    const QString store = RobotCollisionEnvelopeStore::StoreDirectory();
    if (path.isEmpty() || store.isEmpty()) return QString();
    const QString relative = QDir(store).relativeFilePath(path)
        .replace(QLatin1Char('\\'), QLatin1Char('/'));
    return relative == fileName ? path : QString();
}

QJsonArray VectorToJson(const Eigen::Vector3d& value)
{
    return QJsonArray{ value.x(), value.y(), value.z() };
}

QJsonObject BoundsToJson(const RobotCollisionEnvelopeStore::AxisAlignedBounds& bounds)
{
    return QJsonObject{
        { QStringLiteral("minimumMm"), VectorToJson(bounds.minimumMm) },
        { QStringLiteral("maximumMm"), VectorToJson(bounds.maximumMm) }
    };
}

QJsonObject PayloadObject(const RobotCollisionEnvelopeStore::EnvelopeSet& envelope)
{
    QJsonArray joints;
    for (const auto& joint : envelope.joints)
    {
        joints.append(QJsonObject{
            { QStringLiteral("jointIndex"), joint.jointIndex },
            { QStringLiteral("jointName"), joint.jointName },
            { QStringLiteral("assemblyPath"), joint.assemblyPath },
            { QStringLiteral("sourceBoundsMm"), BoundsToJson(joint.sourceBoundsMm) },
            { QStringLiteral("collisionBoundsMm"), BoundsToJson(joint.collisionBoundsMm) }
        });
    }
    return QJsonObject{
        { QStringLiteral("schemaVersion"), kSchemaVersion },
        { QStringLiteral("kind"), kKind },
        { QStringLiteral("profileKeySha256"), envelope.profileKeySha256 },
        { QStringLiteral("sourceStepSha256"), envelope.sourceStepSha256 },
        { QStringLiteral("sourceLengthUnit"), envelope.sourceLengthUnit },
        { QStringLiteral("coordinateFrame"), envelope.coordinateFrame },
        { QStringLiteral("staticOnly"), envelope.staticOnly },
        { QStringLiteral("generation"), QJsonObject{
            { QStringLiteral("algorithm"), envelope.algorithm },
            { QStringLiteral("safetyMarginMicrometres"),
              envelope.safetyMarginMicrometres }
        } },
        { QStringLiteral("sourceUp"), VectorToJson(envelope.sourceUp) },
        { QStringLiteral("baseGeometry"), QJsonObject{
            { QStringLiteral("minimumYmm"), envelope.baseMinimumYmm },
            { QStringLiteral("centerMm"), VectorToJson(envelope.baseCenterMm) }
        } },
        { QStringLiteral("overallSourceBoundsMm"),
          BoundsToJson(envelope.overallSourceBoundsMm) },
        { QStringLiteral("overallExpandedBoundsMm"),
          BoundsToJson(envelope.overallExpandedBoundsMm) },
        { QStringLiteral("joints"), joints }
    };
}

QString PayloadSha256(const QJsonObject& payload)
{
    const QByteArray bytes = QJsonDocument(payload).toJson(QJsonDocument::Compact);
    return QString::fromLatin1(
        QCryptographicHash::hash(bytes, QCryptographicHash::Sha256).toHex());
}

bool ValidateEnvelope(
    const RobotCollisionEnvelopeStore::EnvelopeSet& envelope,
    QString& error)
{
    const qint64 maximumMargin = static_cast<qint64>(
        RobotCollisionEnvelopeStore::MaximumSafetyMarginMm
        * kMicrometresPerMillimetre);
    if (!IsLowerSha256(envelope.sourceStepSha256)
        || envelope.sourceLengthUnit != kLengthUnit
        || envelope.coordinateFrame != kCoordinateFrame
        || !envelope.staticOnly
        || envelope.algorithm != kAlgorithm
        || envelope.safetyMarginMicrometres < 0
        || envelope.safetyMarginMicrometres > maximumMargin
        || envelope.profileKeySha256
            != ProfileKey(envelope.sourceStepSha256,
                          envelope.safetyMarginMicrometres)
        || envelope.joints.size() != 7
        || !std::isfinite(envelope.baseMinimumYmm)
        || !envelope.sourceUp.allFinite() || !envelope.baseCenterMm.allFinite()
        || std::abs(envelope.sourceUp.norm() - 1.0) > 1.0e-9
        || !IsFiniteBounds(envelope.overallSourceBoundsMm)
        || !IsFiniteBounds(envelope.overallExpandedBoundsMm))
    {
        error = QStringLiteral("机器人碰撞包络身份、单位、算法或关节数量无效。");
        return false;
    }
    std::array<bool, 7> seen{};
    const double marginMm = static_cast<double>(envelope.safetyMarginMicrometres)
        / kMicrometresPerMillimetre;
    constexpr double tolerance = 1.0e-9;
    RobotCollisionEnvelopeStore::AxisAlignedBounds recomputedSource;
    recomputedSource.minimumMm = Eigen::Vector3d::Constant(
        std::numeric_limits<double>::infinity());
    recomputedSource.maximumMm = Eigen::Vector3d::Constant(
        -std::numeric_limits<double>::infinity());
    RobotCollisionEnvelopeStore::AxisAlignedBounds recomputedExpanded;
    recomputedExpanded.minimumMm = Eigen::Vector3d::Constant(
        std::numeric_limits<double>::infinity());
    recomputedExpanded.maximumMm = Eigen::Vector3d::Constant(
        -std::numeric_limits<double>::infinity());
    for (const auto& joint : envelope.joints)
    {
        if (joint.jointIndex < 0 || joint.jointIndex > 6
            || seen[static_cast<size_t>(joint.jointIndex)]
            || joint.jointName != QStringLiteral("J%1").arg(joint.jointIndex)
            || !IsSafeText(joint.assemblyPath, 4096)
            || !IsFiniteBounds(joint.sourceBoundsMm)
            || !IsFiniteBounds(joint.collisionBoundsMm))
        {
            error = QStringLiteral("机器人碰撞包络包含重复或无效的关节记录。");
            return false;
        }
        seen[static_cast<size_t>(joint.jointIndex)] = true;
        for (int axis = 0; axis < 3; ++axis)
        {
            if (joint.collisionBoundsMm.minimumMm[axis]
                >= joint.collisionBoundsMm.maximumMm[axis])
            {
                error = QStringLiteral("J%1 的碰撞 AABB 没有有限体积。")
                    .arg(joint.jointIndex);
                return false;
            }
            double expectedMinimum = joint.sourceBoundsMm.minimumMm[axis] - marginMm;
            if (joint.jointIndex == 0 && axis == 1)
                expectedMinimum = (std::max)(expectedMinimum, envelope.baseMinimumYmm);
            const double expectedMaximum = joint.sourceBoundsMm.maximumMm[axis] + marginMm;
            if (std::abs(joint.collisionBoundsMm.minimumMm[axis] - expectedMinimum)
                    > tolerance
                || std::abs(joint.collisionBoundsMm.maximumMm[axis] - expectedMaximum)
                    > tolerance)
            {
                error = QStringLiteral("J%1 的碰撞 AABB 未按安全余量保守覆盖源边界。")
                    .arg(joint.jointIndex);
                return false;
            }
            if (joint.sourceBoundsMm.minimumMm[axis]
                    < envelope.overallSourceBoundsMm.minimumMm[axis] - tolerance
                || joint.sourceBoundsMm.maximumMm[axis]
                    > envelope.overallSourceBoundsMm.maximumMm[axis] + tolerance)
            {
                error = QStringLiteral("J%1 的源 AABB 越出机器人整体源边界。")
                    .arg(joint.jointIndex);
                return false;
            }
            recomputedExpanded.minimumMm[axis] = (std::min)(
                recomputedExpanded.minimumMm[axis],
                joint.collisionBoundsMm.minimumMm[axis]);
            recomputedExpanded.maximumMm[axis] = (std::max)(
                recomputedExpanded.maximumMm[axis],
                joint.collisionBoundsMm.maximumMm[axis]);
            recomputedSource.minimumMm[axis] = (std::min)(
                recomputedSource.minimumMm[axis], joint.sourceBoundsMm.minimumMm[axis]);
            recomputedSource.maximumMm[axis] = (std::max)(
                recomputedSource.maximumMm[axis], joint.sourceBoundsMm.maximumMm[axis]);
        }
    }
    const auto j0 = std::find_if(
        envelope.joints.cbegin(),
        envelope.joints.cend(),
        [](const RobotCollisionEnvelopeStore::JointEnvelope& joint)
        {
            return joint.jointIndex == 0;
        });
    if (j0 == envelope.joints.cend()
        || std::abs(envelope.baseMinimumYmm
                    - j0->sourceBoundsMm.minimumMm.y()) > tolerance
        || std::abs(j0->collisionBoundsMm.minimumMm.y()
                    - envelope.baseMinimumYmm) > tolerance)
    {
        error = QStringLiteral("J0 基座安装面与 J0 源/碰撞边界不一致。");
        return false;
    }
    for (int axis = 0; axis < 3; ++axis)
    {
        if (std::abs(envelope.overallSourceBoundsMm.minimumMm[axis]
                     - recomputedSource.minimumMm[axis]) > tolerance
            || std::abs(envelope.overallSourceBoundsMm.maximumMm[axis]
                        - recomputedSource.maximumMm[axis]) > tolerance
            || std::abs(envelope.overallExpandedBoundsMm.minimumMm[axis]
                     - recomputedExpanded.minimumMm[axis]) > tolerance
            || std::abs(envelope.overallExpandedBoundsMm.maximumMm[axis]
                        - recomputedExpanded.maximumMm[axis]) > tolerance)
        {
            error = QStringLiteral("机器人整体源/扩张 AABB 与 J0...J6 并集不一致。");
            return false;
        }
    }
    if (!IsLowerSha256(envelope.payloadSha256)
        || envelope.payloadSha256 != PayloadSha256(PayloadObject(envelope)))
    {
        error = QStringLiteral("机器人碰撞包络 payload SHA-256 无效。");
        return false;
    }
    return true;
}

bool JsonToVector(const QJsonValue& value, Eigen::Vector3d& output)
{
    if (!value.isArray()) return false;
    const QJsonArray array = value.toArray();
    if (array.size() != 3) return false;
    for (int axis = 0; axis < 3; ++axis)
    {
        if (!array.at(axis).isDouble()) return false;
        output[axis] = array.at(axis).toDouble(std::numeric_limits<double>::quiet_NaN());
        if (!std::isfinite(output[axis])) return false;
    }
    return true;
}

bool JsonToBounds(
    const QJsonValue& value,
    RobotCollisionEnvelopeStore::AxisAlignedBounds& output)
{
    if (!value.isObject()) return false;
    const QJsonObject object = value.toObject();
    return HasExactKeys(object, { QStringLiteral("minimumMm"),
                                  QStringLiteral("maximumMm") })
        && JsonToVector(object.value(QStringLiteral("minimumMm")), output.minimumMm)
        && JsonToVector(object.value(QStringLiteral("maximumMm")), output.maximumMm)
        && IsFiniteBounds(output);
}

bool ParseDocument(
    const QByteArray& bytes,
    RobotCollisionEnvelopeStore::EnvelopeSet& output,
    QString& error)
{
    output = RobotCollisionEnvelopeStore::EnvelopeSet();
    QJsonParseError parseError;
    const QJsonDocument document = QJsonDocument::fromJson(bytes, &parseError);
    if (parseError.error != QJsonParseError::NoError || !document.isObject())
    {
        error = QStringLiteral("机器人碰撞包络 JSON 无效：%1")
            .arg(parseError.errorString());
        return false;
    }
    const QJsonObject root = document.object();
    if (!HasExactKeys(root, {
            QStringLiteral("schemaVersion"), QStringLiteral("kind"),
            QStringLiteral("profileKeySha256"), QStringLiteral("sourceStepSha256"),
            QStringLiteral("sourceLengthUnit"), QStringLiteral("coordinateFrame"),
            QStringLiteral("staticOnly"), QStringLiteral("generation"),
            QStringLiteral("sourceUp"), QStringLiteral("baseGeometry"),
            QStringLiteral("overallSourceBoundsMm"),
            QStringLiteral("overallExpandedBoundsMm"),
            QStringLiteral("joints"), QStringLiteral("payloadSha256") })
        || !root.value(QStringLiteral("schemaVersion")).isDouble()
        || root.value(QStringLiteral("schemaVersion")).toDouble(-1.0)
            != static_cast<double>(kSchemaVersion)
        || !root.value(QStringLiteral("kind")).isString()
        || root.value(QStringLiteral("kind")).toString() != kKind
        || !root.value(QStringLiteral("profileKeySha256")).isString()
        || !root.value(QStringLiteral("sourceStepSha256")).isString()
        || !root.value(QStringLiteral("sourceLengthUnit")).isString()
        || !root.value(QStringLiteral("coordinateFrame")).isString()
        || !root.value(QStringLiteral("staticOnly")).isBool()
        || !root.value(QStringLiteral("generation")).isObject()
        || !root.value(QStringLiteral("sourceUp")).isArray()
        || !root.value(QStringLiteral("baseGeometry")).isObject()
        || !root.value(QStringLiteral("overallSourceBoundsMm")).isObject()
        || !root.value(QStringLiteral("overallExpandedBoundsMm")).isObject()
        || !root.value(QStringLiteral("joints")).isArray()
        || !root.value(QStringLiteral("payloadSha256")).isString())
    {
        error = QStringLiteral("机器人碰撞包络 JSON schema 或字段类型无效。");
        return false;
    }
    const QJsonObject generation = root.value(QStringLiteral("generation")).toObject();
    if (!HasExactKeys(generation, { QStringLiteral("algorithm"),
                                    QStringLiteral("safetyMarginMicrometres") })
        || !generation.value(QStringLiteral("algorithm")).isString()
        || !generation.value(QStringLiteral("safetyMarginMicrometres")).isDouble())
    {
        error = QStringLiteral("机器人碰撞包络生成参数 schema 无效。");
        return false;
    }
    const double marginNumber = generation.value(
        QStringLiteral("safetyMarginMicrometres")).toDouble(-1.0);
    if (!std::isfinite(marginNumber) || std::floor(marginNumber) != marginNumber
        || marginNumber < 0.0
        || marginNumber > RobotCollisionEnvelopeStore::MaximumSafetyMarginMm
            * kMicrometresPerMillimetre)
    {
        error = QStringLiteral("机器人碰撞包络安全余量字段无效。");
        return false;
    }

    RobotCollisionEnvelopeStore::EnvelopeSet parsed;
    parsed.profileKeySha256 = root.value(QStringLiteral("profileKeySha256")).toString();
    parsed.sourceStepSha256 = root.value(QStringLiteral("sourceStepSha256")).toString();
    parsed.sourceLengthUnit = root.value(QStringLiteral("sourceLengthUnit")).toString();
    parsed.coordinateFrame = root.value(QStringLiteral("coordinateFrame")).toString();
    parsed.staticOnly = root.value(QStringLiteral("staticOnly")).toBool(false);
    parsed.algorithm = generation.value(QStringLiteral("algorithm")).toString();
    parsed.safetyMarginMicrometres = static_cast<qint64>(marginNumber);
    parsed.payloadSha256 = root.value(QStringLiteral("payloadSha256")).toString();
    const QJsonObject baseGeometry = root.value(QStringLiteral("baseGeometry")).toObject();
    if (!HasExactKeys(baseGeometry, { QStringLiteral("minimumYmm"),
                                      QStringLiteral("centerMm") })
        || !baseGeometry.value(QStringLiteral("minimumYmm")).isDouble()
        || !std::isfinite(baseGeometry.value(QStringLiteral("minimumYmm")).toDouble())
        || !JsonToVector(root.value(QStringLiteral("sourceUp")), parsed.sourceUp)
        || !JsonToVector(baseGeometry.value(QStringLiteral("centerMm")),
                         parsed.baseCenterMm)
        || !JsonToBounds(root.value(QStringLiteral("overallSourceBoundsMm")),
                         parsed.overallSourceBoundsMm)
        || !JsonToBounds(root.value(QStringLiteral("overallExpandedBoundsMm")),
                         parsed.overallExpandedBoundsMm))
    {
        error = QStringLiteral("机器人碰撞包络基座或整体边界 schema 无效。");
        return false;
    }
    parsed.baseMinimumYmm = baseGeometry.value(QStringLiteral("minimumYmm")).toDouble();
    const QJsonArray joints = root.value(QStringLiteral("joints")).toArray();
    if (joints.size() != 7)
    {
        error = QStringLiteral("机器人碰撞包络必须包含且只包含 J0...J6。");
        return false;
    }
    for (const QJsonValue& value : joints)
    {
        if (!value.isObject())
        {
            error = QStringLiteral("机器人碰撞包络关节条目不是对象。");
            return false;
        }
        const QJsonObject object = value.toObject();
        if (!HasExactKeys(object, {
                QStringLiteral("jointIndex"), QStringLiteral("jointName"),
                QStringLiteral("assemblyPath"), QStringLiteral("sourceBoundsMm"),
                QStringLiteral("collisionBoundsMm") })
            || !object.value(QStringLiteral("jointIndex")).isDouble()
            || !object.value(QStringLiteral("jointName")).isString()
            || !object.value(QStringLiteral("assemblyPath")).isString())
        {
            error = QStringLiteral("机器人碰撞包络关节 schema 无效。");
            return false;
        }
        const double indexNumber = object.value(QStringLiteral("jointIndex")).toDouble(-1.0);
        if (std::floor(indexNumber) != indexNumber || indexNumber < 0.0 || indexNumber > 6.0)
        {
            error = QStringLiteral("机器人碰撞包络关节编号无效。");
            return false;
        }
        RobotCollisionEnvelopeStore::JointEnvelope joint;
        joint.jointIndex = static_cast<int>(indexNumber);
        joint.jointName = object.value(QStringLiteral("jointName")).toString();
        joint.assemblyPath = object.value(QStringLiteral("assemblyPath")).toString();
        if (!JsonToBounds(object.value(QStringLiteral("sourceBoundsMm")),
                          joint.sourceBoundsMm)
            || !JsonToBounds(object.value(QStringLiteral("collisionBoundsMm")),
                             joint.collisionBoundsMm))
        {
            error = QStringLiteral("机器人碰撞包络关节边界无效。");
            return false;
        }
        parsed.joints.append(joint);
    }
    if (!ValidateEnvelope(parsed, error)) return false;
    output = parsed;
    return true;
}

bool ReadStableSmallFile(const QString& path, QByteArray& bytes, QString& error)
{
    bytes.clear();
    const QFileInfo before(path);
    if (!before.exists() || !before.isFile() || IsLinkOrJunction(before)
        || before.size() <= 0
        || before.size() > RobotCollisionEnvelopeStore::MaximumEnvelopeFileBytes)
    {
        error = QStringLiteral("机器人碰撞包络不是受控的普通小文件。");
        return false;
    }
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly) || file.size() != before.size())
    {
        error = QStringLiteral("无法稳定打开机器人碰撞包络：%1").arg(file.errorString());
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
        error = QStringLiteral("机器人碰撞包络在读取过程中发生变化。");
        return false;
    }
    return true;
}

bool ValidateExpectedAsset(
    const RobotCollisionEnvelopeStore::StoredAsset& asset,
    QString& error)
{
    const QString expectedName = FileNameForProfile(asset.profileKeySha256);
    if (!IsLowerSha256(asset.profileKeySha256)
        || !IsLowerSha256(asset.sourceStepSha256)
        || expectedName.isEmpty() || asset.storedFileName != expectedName
        || StoredPath(asset.storedFileName).isEmpty()
        || asset.safetyMarginMicrometres < 0
        || asset.safetyMarginMicrometres
            > static_cast<qint64>(RobotCollisionEnvelopeStore::MaximumSafetyMarginMm
                                  * kMicrometresPerMillimetre)
        || asset.profileKeySha256
            != ProfileKey(asset.sourceStepSha256, asset.safetyMarginMicrometres)
        || asset.sizeBytes < 0
        || asset.sizeBytes > RobotCollisionEnvelopeStore::MaximumEnvelopeFileBytes)
    {
        error = QStringLiteral("机器人碰撞包络资产路径或身份记录无效。");
        return false;
    }
    return true;
}
}

QString RobotCollisionEnvelopeStore::StoreDirectory()
{
    if (!AppPaths::IsInitialized()) return QString();
    return AppPaths::WritablePath(kStoreRelativePath);
}

bool RobotCollisionEnvelopeStore::ValidateForUse(
    const EnvelopeSet& envelope,
    QString& error)
{
    error.clear();
    return ValidateEnvelope(envelope, error);
}

bool RobotCollisionEnvelopeStore::Generate(
    const RobotCadAssemblyLoader::Result& loadedRobot,
    const GenerationParameters& parameters,
    EnvelopeSet& output,
    QString& error)
{
    output = EnvelopeSet();
    error.clear();
    qint64 marginMicrometres = 0;
    if (!MarginMicrometres(parameters, marginMicrometres, error)) return false;
    const auto& statistics = loadedRobot.statistics;
    if (!IsLowerSha256(statistics.sourceSha256))
    {
        error = QStringLiteral("机器人 STEP SHA-256 无效，不能生成碰撞包络。");
        return false;
    }
    if (statistics.sourceLengthUnits.isEmpty())
    {
        error = QStringLiteral("机器人 STEP 未报告长度单位，不能假定毫米。");
        return false;
    }
    if (!loadedRobot.base.valid || !loadedRobot.base.j0BoundsMm.valid
        || !loadedRobot.base.sourceUp.allFinite()
        || std::abs(loadedRobot.base.sourceUp.norm() - 1.0) > 1.0e-9
        || !std::isfinite(loadedRobot.base.minimumYmm)
        || !loadedRobot.base.conservativeBaseCenterMm.allFinite())
    {
        error = QStringLiteral("机器人 loader 未提供有效基座和整体源边界。");
        return false;
    }
    for (const QString& unit : statistics.sourceLengthUnits)
    {
        if (!IsMillimetreUnit(unit))
        {
            error = QStringLiteral("机器人 STEP 不是已确认的毫米单位：%1").arg(unit);
            return false;
        }
    }

    std::array<const RobotCadAssemblyLoader::ComponentStatistics*, 7> selected{};
    for (const auto& component : statistics.components)
    {
        if (component.jointIndex < 0 || component.jointIndex > 6) continue;
        const size_t index = static_cast<size_t>(component.jointIndex);
        if (!component.included || selected[index] != nullptr)
        {
            error = QStringLiteral("机器人装配统计中的 J%1 重复或未被 loader 纳入。")
                .arg(component.jointIndex);
            return false;
        }
        selected[index] = &component;
    }

    EnvelopeSet generated;
    generated.sourceStepSha256 = statistics.sourceSha256;
    generated.sourceLengthUnit = kLengthUnit;
    generated.coordinateFrame = kCoordinateFrame;
    generated.staticOnly = true;
    generated.algorithm = kAlgorithm;
    generated.safetyMarginMicrometres = marginMicrometres;
    generated.profileKeySha256 = ProfileKey(
        generated.sourceStepSha256, marginMicrometres);
    generated.sourceUp = loadedRobot.base.sourceUp;
    generated.baseMinimumYmm = loadedRobot.base.minimumYmm;
    generated.baseCenterMm = loadedRobot.base.conservativeBaseCenterMm;
    generated.overallSourceBoundsMm.minimumMm = Eigen::Vector3d::Constant(
        std::numeric_limits<double>::infinity());
    generated.overallSourceBoundsMm.maximumMm = Eigen::Vector3d::Constant(
        -std::numeric_limits<double>::infinity());
    generated.overallExpandedBoundsMm.minimumMm = Eigen::Vector3d::Constant(
        std::numeric_limits<double>::infinity());
    generated.overallExpandedBoundsMm.maximumMm = Eigen::Vector3d::Constant(
        -std::numeric_limits<double>::infinity());
    const double marginMm = static_cast<double>(marginMicrometres)
        / kMicrometresPerMillimetre;
    for (int jointIndex = 0; jointIndex <= 6; ++jointIndex)
    {
        const auto* component = selected[static_cast<size_t>(jointIndex)];
        if (component == nullptr || !component->boundsMm.valid)
        {
            error = QStringLiteral("机器人装配统计缺少 J%1 的有效边界。")
                .arg(jointIndex);
            return false;
        }
        JointEnvelope joint;
        joint.jointIndex = jointIndex;
        joint.jointName = QStringLiteral("J%1").arg(jointIndex);
        joint.assemblyPath = component->assemblyPath;
        joint.sourceBoundsMm.minimumMm = component->boundsMm.minimumMm;
        joint.sourceBoundsMm.maximumMm = component->boundsMm.maximumMm;
        if (!IsSafeText(joint.assemblyPath, 4096)
            || !IsFiniteBounds(joint.sourceBoundsMm))
        {
            error = QStringLiteral("J%1 的源 AABB 或装配路径无效。").arg(jointIndex);
            return false;
        }
        joint.collisionBoundsMm.minimumMm = joint.sourceBoundsMm.minimumMm
            - Eigen::Vector3d::Constant(marginMm);
        joint.collisionBoundsMm.maximumMm = joint.sourceBoundsMm.maximumMm
            + Eigen::Vector3d::Constant(marginMm);
        if (jointIndex == 0)
        {
            joint.collisionBoundsMm.minimumMm.y() = (std::max)(
                joint.collisionBoundsMm.minimumMm.y(), generated.baseMinimumYmm);
        }
        generated.overallSourceBoundsMm.minimumMm =
            generated.overallSourceBoundsMm.minimumMm.cwiseMin(
                joint.sourceBoundsMm.minimumMm);
        generated.overallSourceBoundsMm.maximumMm =
            generated.overallSourceBoundsMm.maximumMm.cwiseMax(
                joint.sourceBoundsMm.maximumMm);
        generated.overallExpandedBoundsMm.minimumMm =
            generated.overallExpandedBoundsMm.minimumMm.cwiseMin(
                joint.collisionBoundsMm.minimumMm);
        generated.overallExpandedBoundsMm.maximumMm =
            generated.overallExpandedBoundsMm.maximumMm.cwiseMax(
                joint.collisionBoundsMm.maximumMm);
        generated.joints.append(joint);
    }
    generated.payloadSha256 = PayloadSha256(PayloadObject(generated));
    if (!ValidateEnvelope(generated, error)) return false;
    output = generated;
    return true;
}

bool RobotCollisionEnvelopeStore::Persist(
    const EnvelopeSet& envelope,
    StoredAsset& asset,
    QString& error)
{
    asset = StoredAsset();
    error.clear();
    if (!ValidateEnvelope(envelope, error) || !EnsureStoreDirectory(error)) return false;
    QJsonObject root = PayloadObject(envelope);
    root.insert(QStringLiteral("payloadSha256"), envelope.payloadSha256);
    const QByteArray bytes = QJsonDocument(root).toJson(QJsonDocument::Indented);
    if (bytes.isEmpty() || bytes.size() > MaximumEnvelopeFileBytes)
    {
        error = QStringLiteral("机器人碰撞包络超过持久化大小上限。");
        return false;
    }
    const QString fileName = FileNameForProfile(envelope.profileKeySha256);
    const QString path = StoredPath(fileName);
    const QString lockPath = StoredPath(kMutationLockName);
    if (path.isEmpty() || lockPath.isEmpty())
    {
        error = QStringLiteral("机器人碰撞包络受控路径解析失败。");
        return false;
    }

    QMutexLocker<QMutex> processGuard(&MutationMutex());
    const QFileInfo lockInfo(lockPath);
    if (IsLinkOrJunction(lockInfo) || (lockInfo.exists() && !lockInfo.isFile()))
    {
        error = QStringLiteral("机器人碰撞包络锁路径被非普通文件占用。");
        return false;
    }
    QLockFile lock(lockPath);
    lock.setStaleLockTime(0);
    if (!lock.tryLock(10000))
    {
        error = QStringLiteral("机器人碰撞包络库正由另一个进程修改。");
        return false;
    }
    const QFileInfo existing(path);
    if (IsLinkOrJunction(existing) || (existing.exists() && !existing.isFile()))
    {
        error = QStringLiteral("机器人碰撞包络目标路径被非普通文件占用。");
        return false;
    }
    if (existing.exists())
    {
        QByteArray existingBytes;
        EnvelopeSet existingEnvelope;
        if (!ReadStableSmallFile(path, existingBytes, error)
            || !ParseDocument(existingBytes, existingEnvelope, error)
            || existingEnvelope.payloadSha256 != envelope.payloadSha256)
        {
            error = QStringLiteral("同 profileKey 的碰撞包络已存在但内容不一致：%1")
                .arg(error);
            return false;
        }
    }
    else
    {
        QSaveFile file(path);
        file.setDirectWriteFallback(false);
        if (!file.open(QIODevice::WriteOnly)
            || file.write(bytes) != bytes.size()
            || !file.commit())
        {
            file.cancelWriting();
            error = QStringLiteral("无法原子写入机器人碰撞包络：%1")
                .arg(file.errorString());
            return false;
        }
    }

    StoredAsset saved;
    saved.profileKeySha256 = envelope.profileKeySha256;
    saved.sourceStepSha256 = envelope.sourceStepSha256;
    saved.storedFileName = fileName;
    saved.sizeBytes = QFileInfo(path).size();
    saved.safetyMarginMicrometres = envelope.safetyMarginMicrometres;
    EnvelopeSet verified;
    if (!LoadAsset(saved, verified, error)
        || verified.payloadSha256 != envelope.payloadSha256)
    {
        asset = StoredAsset();
        return false;
    }
    asset = saved;
    return true;
}

bool RobotCollisionEnvelopeStore::Load(
    const QString& sourceStepSha256,
    const GenerationParameters& parameters,
    EnvelopeSet& envelope,
    StoredAsset& asset,
    QString& error)
{
    envelope = EnvelopeSet();
    asset = StoredAsset();
    error.clear();
    qint64 marginMicrometres = 0;
    if (!IsLowerSha256(sourceStepSha256)
        || !MarginMicrometres(parameters, marginMicrometres, error))
    {
        if (error.isEmpty()) error = QStringLiteral("机器人 STEP SHA-256 无效。");
        return false;
    }
    StoredAsset expected;
    expected.profileKeySha256 = ProfileKey(sourceStepSha256, marginMicrometres);
    expected.sourceStepSha256 = sourceStepSha256;
    expected.storedFileName = FileNameForProfile(expected.profileKeySha256);
    expected.safetyMarginMicrometres = marginMicrometres;
    if (!LoadAsset(expected, envelope, error)) return false;
    expected.sizeBytes = QFileInfo(StoredPath(expected.storedFileName)).size();
    asset = expected;
    return true;
}

bool RobotCollisionEnvelopeStore::IsMissing(
    const QString& sourceStepSha256,
    const GenerationParameters& parameters,
    bool& missing,
    QString& error)
{
    missing = false;
    error.clear();
    qint64 marginMicrometres = 0;
    if (!IsLowerSha256(sourceStepSha256)
        || !MarginMicrometres(parameters, marginMicrometres, error))
    {
        if (error.isEmpty()) error = QStringLiteral("机器人 STEP SHA-256 无效。");
        return false;
    }
    if (!EnsureStoreDirectory(error)) return false;
    const QString profileKey = ProfileKey(sourceStepSha256, marginMicrometres);
    const QString path = StoredPath(FileNameForProfile(profileKey));
    if (path.isEmpty())
    {
        error = QStringLiteral("机器人碰撞包络受控路径解析失败。");
        return false;
    }
    const QFileInfo info(path);
    if (IsLinkOrJunction(info) || (info.exists() && !info.isFile()))
    {
        error = QStringLiteral("机器人碰撞包络目标路径不是普通文件。");
        return false;
    }
    missing = !info.exists();
    return true;
}

bool RobotCollisionEnvelopeStore::LoadAsset(
    const StoredAsset& expectedAsset,
    EnvelopeSet& envelope,
    QString& error)
{
    envelope = EnvelopeSet();
    error.clear();
    if (!ValidateExpectedAsset(expectedAsset, error) || !EnsureStoreDirectory(error))
        return false;
    const QString path = StoredPath(expectedAsset.storedFileName);
    QByteArray bytes;
    if (!ReadStableSmallFile(path, bytes, error)) return false;
    if (expectedAsset.sizeBytes > 0 && bytes.size() != expectedAsset.sizeBytes)
    {
        error = QStringLiteral("机器人碰撞包络大小与资产记录不一致。");
        return false;
    }
    EnvelopeSet parsed;
    if (!ParseDocument(bytes, parsed, error)
        || parsed.profileKeySha256 != expectedAsset.profileKeySha256
        || parsed.sourceStepSha256 != expectedAsset.sourceStepSha256
        || parsed.safetyMarginMicrometres != expectedAsset.safetyMarginMicrometres)
    {
        if (error.isEmpty()) error = QStringLiteral("机器人碰撞包络与资产身份不一致。");
        return false;
    }
    envelope = parsed;
    return true;
}
