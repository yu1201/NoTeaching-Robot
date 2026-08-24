#include "ModelWeldingWorkflow.h"

#include "ConfigDatabase.h"
#include "HandEyeMatrixConfig.h"

#include <QCryptographicHash>
#include <QByteArrayView>
#include <QDateTime>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QMap>
#include <QMutex>
#include <QMutexLocker>
#include <QUuid>

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
constexpr qsizetype kMaximumTemplateCount = 512;
constexpr qsizetype kMaximumStationCount = 64;
constexpr qsizetype kMaximumSeamCount = 32;
constexpr qsizetype kMaximumSeamPointCount = 4096;
constexpr qsizetype kMaximumTotalSeamPointCount = 8192;
constexpr qsizetype kMaximumRecordBytes = 512 * 1024;
constexpr quint64 kMaximumExactJsonInteger = 9007199254740991ULL;
const QString kTemplateScope = QStringLiteral("model_weld_template");
const QString kTeachingScope = QStringLiteral("model_weld_teaching");
const QString kDefinitionModule = QStringLiteral("DefinitionV1");
const QString kRecordKey = QStringLiteral("RecordJson");

QMutex& StoreMutex()
{
    static QMutex mutex;
    return mutex;
}

bool Finite(double value)
{
    return std::isfinite(value);
}

bool Finite(const Eigen::Vector3d& value)
{
    return value.allFinite();
}

bool Finite(const Eigen::Matrix3d& value)
{
    return value.allFinite();
}

bool FinitePose(const T_ROBOT_COORS& pose)
{
    return Finite(pose.dX) && Finite(pose.dY) && Finite(pose.dZ)
        && Finite(pose.dRX) && Finite(pose.dRY) && Finite(pose.dRZ)
        && Finite(pose.dBX) && Finite(pose.dBY) && Finite(pose.dBZ);
}

bool IsSha256(const QString& value)
{
    if (value.size() != 64)
    {
        return false;
    }
    for (const QChar ch : value)
    {
        const char16_t code = ch.unicode();
        const bool asciiDigit = code >= u'0' && code <= u'9';
        const bool asciiLowerHex = code >= u'a' && code <= u'f';
        const bool asciiUpperHex = code >= u'A' && code <= u'F';
        if (!asciiDigit && !asciiLowerHex && !asciiUpperHex)
        {
            return false;
        }
    }
    return true;
}

bool IsStableId(const QString& value)
{
    const QUuid uuid(value);
    return !uuid.isNull() && value == uuid.toString(QUuid::WithoutBraces).toLower();
}

bool IsStableIdentifier(const QString& value)
{
    if (value.isEmpty() || value.size() > 128
        || value.front() == QLatin1Char('.')
        || value.back() == QLatin1Char('.'))
    {
        return false;
    }
    for (const QChar ch : value)
    {
        if (!((ch >= QLatin1Char('a') && ch <= QLatin1Char('z'))
              || (ch >= QLatin1Char('0') && ch <= QLatin1Char('9'))
              || ch == QLatin1Char('.') || ch == QLatin1Char('-')
              || ch == QLatin1Char('_')))
        {
            return false;
        }
    }
    return true;
}

QString RoleText(ModelWeldingStationRole role)
{
    switch (role)
    {
    case ModelWeldingStationRole::Verify:
        return QStringLiteral("verify");
    case ModelWeldingStationRole::Backup:
        return QStringLiteral("backup");
    default:
        return QStringLiteral("solve");
    }
}

QString SeamSourceText(ModelWeldingSeamSource source)
{
    switch (source)
    {
    case ModelWeldingSeamSource::CadShapeIntersection:
        return QStringLiteral("cad_shape_intersection");
    case ModelWeldingSeamSource::CadCorrugatedButtJoint:
        return QStringLiteral("cad_corrugated_butt_joint");
    case ModelWeldingSeamSource::CadCorrugatedBaseJoint:
        return QStringLiteral("cad_corrugated_base_joint");
    case ModelWeldingSeamSource::ReverseMeshSeedProjection:
        return QStringLiteral("reverse_mesh_seed_projection");
    default:
        return QStringLiteral("cad_shared_edge");
    }
}

bool ParseSeamSource(const QString& text, ModelWeldingSeamSource& source)
{
    if (text == QStringLiteral("cad_shared_edge"))
    {
        source = ModelWeldingSeamSource::CadSharedEdge;
        return true;
    }
    if (text == QStringLiteral("cad_shape_intersection"))
    {
        source = ModelWeldingSeamSource::CadShapeIntersection;
        return true;
    }
    if (text == QStringLiteral("cad_corrugated_butt_joint"))
    {
        source = ModelWeldingSeamSource::CadCorrugatedButtJoint;
        return true;
    }
    if (text == QStringLiteral("cad_corrugated_base_joint"))
    {
        source = ModelWeldingSeamSource::CadCorrugatedBaseJoint;
        return true;
    }
    if (text == QStringLiteral("reverse_mesh_seed_projection"))
    {
        source = ModelWeldingSeamSource::ReverseMeshSeedProjection;
        return true;
    }
    return false;
}

bool ParseRole(const QString& text, ModelWeldingStationRole& role)
{
    if (text == QStringLiteral("solve"))
    {
        role = ModelWeldingStationRole::Solve;
        return true;
    }
    if (text == QStringLiteral("verify"))
    {
        role = ModelWeldingStationRole::Verify;
        return true;
    }
    if (text == QStringLiteral("backup"))
    {
        role = ModelWeldingStationRole::Backup;
        return true;
    }
    return false;
}

QJsonArray VectorJson(const Eigen::Vector3d& value)
{
    return QJsonArray{ value.x(), value.y(), value.z() };
}

bool ParseVector(const QJsonValue& json, Eigen::Vector3d& value)
{
    if (!json.isArray())
    {
        return false;
    }
    const QJsonArray array = json.toArray();
    if (array.size() != 3)
    {
        return false;
    }
    for (int i = 0; i < 3; ++i)
    {
        if (!array.at(i).isDouble() || !Finite(array.at(i).toDouble()))
        {
            return false;
        }
        value[i] = array.at(i).toDouble();
    }
    return true;
}

QJsonArray MatrixJson(const Eigen::Matrix3d& value)
{
    QJsonArray rows;
    for (int row = 0; row < 3; ++row)
    {
        rows.append(QJsonArray{ value(row, 0), value(row, 1), value(row, 2) });
    }
    return rows;
}

bool ParseMatrix(const QJsonValue& json, Eigen::Matrix3d& value)
{
    if (!json.isArray() || json.toArray().size() != 3)
    {
        return false;
    }
    const QJsonArray rows = json.toArray();
    for (int row = 0; row < 3; ++row)
    {
        const QJsonValue rowValue = rows.at(row);
        if (!rowValue.isArray() || rowValue.toArray().size() != 3)
        {
            return false;
        }
        const QJsonArray columns = rowValue.toArray();
        for (int column = 0; column < 3; ++column)
        {
            if (!columns.at(column).isDouble() || !Finite(columns.at(column).toDouble()))
            {
                return false;
            }
            value(row, column) = columns.at(column).toDouble();
        }
    }
    return true;
}

QJsonObject PoseJson(const T_ROBOT_COORS& pose)
{
    QJsonObject result;
    result.insert(QStringLiteral("x"), pose.dX);
    result.insert(QStringLiteral("y"), pose.dY);
    result.insert(QStringLiteral("z"), pose.dZ);
    result.insert(QStringLiteral("rx"), pose.dRX);
    result.insert(QStringLiteral("ry"), pose.dRY);
    result.insert(QStringLiteral("rz"), pose.dRZ);
    result.insert(QStringLiteral("bx"), pose.dBX);
    result.insert(QStringLiteral("by"), pose.dBY);
    result.insert(QStringLiteral("bz"), pose.dBZ);
    return result;
}

bool ParsePose(const QJsonValue& json, T_ROBOT_COORS& pose)
{
    if (!json.isObject())
    {
        return false;
    }
    const QJsonObject object = json.toObject();
    const QStringList keys = {
        QStringLiteral("x"), QStringLiteral("y"), QStringLiteral("z"),
        QStringLiteral("rx"), QStringLiteral("ry"), QStringLiteral("rz"),
        QStringLiteral("bx"), QStringLiteral("by"), QStringLiteral("bz")
    };
    double values[9] = {};
    for (int i = 0; i < keys.size(); ++i)
    {
        const QJsonValue value = object.value(keys.at(i));
        if (!value.isDouble() || !Finite(value.toDouble()))
        {
            return false;
        }
        values[i] = value.toDouble();
    }
    pose = T_ROBOT_COORS(
        values[0], values[1], values[2], values[3], values[4], values[5],
        values[6], values[7], values[8]);
    return true;
}

QJsonArray PulseJson(const T_ANGLE_PULSE& pulse)
{
    return QJsonArray{
        static_cast<double>(pulse.nSPulse), static_cast<double>(pulse.nLPulse),
        static_cast<double>(pulse.nUPulse), static_cast<double>(pulse.nRPulse),
        static_cast<double>(pulse.nBPulse), static_cast<double>(pulse.nTPulse),
        static_cast<double>(pulse.lBXPulse), static_cast<double>(pulse.lBYPulse),
        static_cast<double>(pulse.lBZPulse)
    };
}

bool ParsePulse(const QJsonValue& json, T_ANGLE_PULSE& pulse)
{
    if (!json.isArray() || json.toArray().size() != 9)
    {
        return false;
    }
    const QJsonArray array = json.toArray();
    long values[9] = {};
    for (int i = 0; i < 9; ++i)
    {
        if (!array.at(i).isDouble() || !Finite(array.at(i).toDouble()))
        {
            return false;
        }
        const double rounded = std::round(array.at(i).toDouble());
        if (std::abs(rounded - array.at(i).toDouble()) > 1.0e-6
            || rounded < static_cast<double>((std::numeric_limits<long>::min)())
            || rounded > static_cast<double>((std::numeric_limits<long>::max)()))
        {
            return false;
        }
        values[i] = static_cast<long>(rounded);
    }
    pulse = T_ANGLE_PULSE(
        values[0], values[1], values[2], values[3], values[4], values[5],
        values[6], values[7], values[8]);
    return true;
}

bool JsonString(const QJsonObject& object, const QString& key, QString& value, bool allowEmpty = false)
{
    const QJsonValue json = object.value(key);
    if (!json.isString())
    {
        return false;
    }
    value = json.toString();
    return allowEmpty || !value.trimmed().isEmpty();
}

bool JsonBool(const QJsonObject& object, const QString& key, bool& value)
{
    const QJsonValue json = object.value(key);
    if (!json.isBool())
    {
        return false;
    }
    value = json.toBool();
    return true;
}

bool JsonRevision(const QJsonObject& object, const QString& key, quint64& value, bool allowZero = false)
{
    const QJsonValue json = object.value(key);
    if (!json.isDouble())
    {
        return false;
    }
    const double number = json.toDouble();
    if (!Finite(number) || number < (allowZero ? 0.0 : 1.0)
        || number > 9007199254740991.0 || std::floor(number) != number)
    {
        return false;
    }
    value = static_cast<quint64>(number);
    return true;
}

bool IsOrthonormalRightHanded(const Eigen::Matrix3d& axes)
{
    if (!Finite(axes))
    {
        return false;
    }
    const double orthogonalityError =
        (axes.transpose() * axes - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff();
    return orthogonalityError <= 1.0e-5 && std::abs(axes.determinant() - 1.0) <= 1.0e-5;
}

double PoseDistanceMm(const T_ROBOT_COORS& left, const T_ROBOT_COORS& right)
{
    const double dx = left.dX - right.dX;
    const double dy = left.dY - right.dY;
    const double dz = left.dZ - right.dZ;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

QByteArray CompactJson(const QJsonObject& object)
{
    return QJsonDocument(object).toJson(QJsonDocument::Compact);
}

bool ReadStoredJson(const QString& scopeType, const QString& scopeId, QByteArray& json, QString& error)
{
    error.clear();
    json.clear();
    QString text;
    const ConfigDatabase::ReadStatus status = ConfigDatabase::ReadScopedSettingStatus(
        scopeType, scopeId, kDefinitionModule, kRecordKey, &text);
    if (status == ConfigDatabase::ReadStatus::NotFound)
    {
        return false;
    }
    if (status != ConfigDatabase::ReadStatus::Found)
    {
        error = QStringLiteral("配置数据库读取失败。");
        return false;
    }
    json = text.toUtf8();
    if (json.isEmpty() || json.size() > kMaximumRecordBytes)
    {
        error = QStringLiteral("配置记录为空或超过大小上限。");
        return false;
    }
    return true;
}

bool CompareAndSwapStoredJson(
    const QString& scopeType,
    const QString& scopeId,
    const QByteArray* expectedJson,
    const QByteArray& newJson,
    QString& error)
{
    if (newJson.isEmpty() || newJson.size() > kMaximumRecordBytes)
    {
        error = QStringLiteral("配置记录为空或超过大小上限。");
        return false;
    }
    QString expectedText;
    const QString* expectedTextPointer = nullptr;
    if (expectedJson != nullptr)
    {
        expectedText = QString::fromUtf8(*expectedJson);
        expectedTextPointer = &expectedText;
    }
    bool conflict = false;
    if (!ConfigDatabase::CompareAndSwapScopedSetting(
        scopeType,
        scopeId,
        kDefinitionModule,
        kRecordKey,
        expectedTextPointer,
        QString::fromUtf8(newJson),
        QStringLiteral("json"),
        false,
        &conflict,
        &error))
    {
        if (conflict)
        {
            error = QStringLiteral("配置记录已被其他进程或操作更新，保存已拒绝。");
        }
        else if (error.isEmpty())
        {
            error = QStringLiteral("配置数据库原子写入失败。");
        }
        return false;
    }
    return true;
}

bool CompareAndSwapStoredJsonWithWitness(
    const QString& witnessScopeType,
    const QString& witnessScopeId,
    const QByteArray& expectedWitnessJson,
    const QString& targetScopeType,
    const QString& targetScopeId,
    const QByteArray* expectedTargetJson,
    const QByteArray& newTargetJson,
    QString& error)
{
    if (expectedWitnessJson.isEmpty() || expectedWitnessJson.size() > kMaximumRecordBytes
        || newTargetJson.isEmpty() || newTargetJson.size() > kMaximumRecordBytes)
    {
        error = QStringLiteral("witness或target配置记录为空或超过大小上限。");
        return false;
    }
    QString expectedTargetText;
    const QString* expectedTargetTextPointer = nullptr;
    if (expectedTargetJson != nullptr)
    {
        expectedTargetText = QString::fromUtf8(*expectedTargetJson);
        expectedTargetTextPointer = &expectedTargetText;
    }
    bool conflict = false;
    if (!ConfigDatabase::CompareAndSwapScopedSettingWithWitness(
        witnessScopeType,
        witnessScopeId,
        kDefinitionModule,
        kRecordKey,
        QString::fromUtf8(expectedWitnessJson),
        targetScopeType,
        targetScopeId,
        kDefinitionModule,
        kRecordKey,
        expectedTargetTextPointer,
        QString::fromUtf8(newTargetJson),
        QStringLiteral("json"),
        false,
        &conflict,
        &error))
    {
        if (conflict)
        {
            error = QStringLiteral("模型模板或机器人示教已被其他进程更新，保存已拒绝。");
        }
        else if (error.isEmpty())
        {
            error = QStringLiteral("带模板版本见证的配置原子写入失败。");
        }
        return false;
    }
    return true;
}

double Rmse(const QVector<double>& values)
{
    if (values.isEmpty())
    {
        return 0.0;
    }
    double squared = 0.0;
    for (double value : values)
    {
        squared += value * value;
    }
    return std::sqrt(squared / static_cast<double>(values.size()));
}

double Maximum(const QVector<double>& values)
{
    double maximum = 0.0;
    for (double value : values)
    {
        maximum = std::max(maximum, value);
    }
    return maximum;
}

QByteArray CanonicalDouble(const char* name, double raw)
{
    double value = raw;
    if (value == 0.0)
    {
        value = 0.0;
    }
    return QByteArray(name) + '=' + QByteArray::number(value, 'g', 17) + '\n';
}
}

QString ModelWeldingWorkflow::CreateStableId()
{
    return QUuid::createUuid().toString(QUuid::WithoutBraces).toLower();
}

QString ModelWeldingWorkflow::ComputeFileSha256(const QString& filePath, QString* error)
{
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly))
    {
        if (error != nullptr)
        {
            *error = QStringLiteral("无法读取模型文件：%1").arg(file.errorString());
        }
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
            if (error != nullptr)
            {
                *error = QStringLiteral("读取模型文件失败：%1").arg(file.errorString());
            }
            return QString();
        }
        if (count > 0)
        {
            hash.addData(QByteArrayView(
                block.constData(), static_cast<qsizetype>(count)));
        }
    }
    return QString::fromLatin1(hash.result().toHex()).toLower();
}

QString ModelWeldingWorkflow::ComputeHandEyeSha256(const HandEyeMatrixConfig& config)
{
    QByteArray canonical;
    canonical.reserve(512);
    canonical += "robotType=" + QByteArray::number(config.robotType) + '\n';
    for (int row = 0; row < 3; ++row)
    {
        for (int column = 0; column < 3; ++column)
        {
            const QByteArray name = "r" + QByteArray::number(row) + QByteArray::number(column);
            canonical += CanonicalDouble(name.constData(), config.rotation(row, column));
        }
    }
    for (int index = 0; index < 3; ++index)
    {
        const QByteArray name = "t" + QByteArray::number(index);
        canonical += CanonicalDouble(name.constData(), config.translation(index));
    }
    return QString::fromLatin1(QCryptographicHash::hash(
        canonical, QCryptographicHash::Sha256).toHex()).toLower();
}

QString ModelWeldingWorkflow::ComputeTool1Sha256(const T_ROBOT_COORS& tool1)
{
    QByteArray canonical("toolNumber=1\ncanonicalVersion=1\n");
    canonical += CanonicalDouble("x", tool1.dX);
    canonical += CanonicalDouble("y", tool1.dY);
    canonical += CanonicalDouble("z", tool1.dZ);
    canonical += CanonicalDouble("rx", tool1.dRX);
    canonical += CanonicalDouble("ry", tool1.dRY);
    canonical += CanonicalDouble("rz", tool1.dRZ);
    return QString::fromLatin1(QCryptographicHash::hash(
        canonical, QCryptographicHash::Sha256).toHex()).toLower();
}

bool ModelWeldingWorkflow::GeneratePlacementGuide(
    const WorkpieceMeshBuilder::Mesh& mesh,
    double longLengthMm,
    double shortLengthMm,
    ModelWeldingPlacementGuide& guide,
    QString& error)
{
    if (!mesh.IsValid() || mesh.vertices.size() < 8)
    {
        error = QStringLiteral("模型网格无效或顶点过少，无法生成放置引导。");
        return false;
    }
    if (!Finite(longLengthMm) || !Finite(shortLengthMm)
        || longLengthMm <= 0.0 || shortLengthMm <= 0.0 || longLengthMm <= shortLengthMm)
    {
        error = QStringLiteral("V型槽必须满足长边长度大于短边长度，且两者均为正数。");
        return false;
    }

    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    qsizetype finiteCount = 0;
    for (const Eigen::Vector3f& vertex : mesh.vertices)
    {
        const Eigen::Vector3d point = vertex.cast<double>();
        if (Finite(point))
        {
            center += point;
            ++finiteCount;
        }
    }
    if (finiteCount < 8)
    {
        error = QStringLiteral("模型有效顶点过少。");
        return false;
    }
    center /= static_cast<double>(finiteCount);

    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (const Eigen::Vector3f& vertex : mesh.vertices)
    {
        const Eigen::Vector3d point = vertex.cast<double>();
        if (Finite(point))
        {
            const Eigen::Vector3d delta = point - center;
            covariance += delta * delta.transpose();
        }
    }
    covariance /= static_cast<double>(finiteCount);
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(covariance);
    if (eigen.info() != Eigen::Success || !eigen.eigenvectors().allFinite()
        || !eigen.eigenvalues().allFinite())
    {
        error = QStringLiteral("模型主方向计算失败。");
        return false;
    }
    const Eigen::Vector3d eigenvalues = eigen.eigenvalues().cwiseMax(0.0);
    if (eigenvalues.z() <= 1.0e-12
        || eigenvalues.y() / eigenvalues.z() <= 1.0e-6
        || std::abs(eigenvalues.z() - eigenvalues.y()) / eigenvalues.z() < 1.0e-3
        || std::abs(eigenvalues.y() - eigenvalues.x()) / eigenvalues.y() < 1.0e-3)
    {
        error = QStringLiteral("模型主方向退化或长短方向近似对称，需人工指定V型槽长边方向。");
        return false;
    }

    Eigen::Vector3d xAxis = eigen.eigenvectors().col(2).normalized();
    Eigen::Vector3d yAxis = eigen.eigenvectors().col(1).normalized();
    Eigen::Vector3d zAxis = xAxis.cross(yAxis).normalized();
    if (!Finite(xAxis) || !Finite(yAxis) || !Finite(zAxis))
    {
        error = QStringLiteral("模型主方向退化，无法生成右手坐标系。");
        return false;
    }

    auto canonicalSign = [](const Eigen::Vector3d& axis) -> double
        {
            Eigen::Index component = 0;
            axis.cwiseAbs().maxCoeff(&component);
            return axis[component] < 0.0 ? -1.0 : 1.0;
        };
    if (canonicalSign(xAxis) < 0.0)
    {
        xAxis = -xAxis;
        yAxis = -yAxis;
    }

    double minZ = (std::numeric_limits<double>::max)();
    double maxZ = (std::numeric_limits<double>::lowest)();
    QVector<double> zValues;
    zValues.reserve(static_cast<qsizetype>(finiteCount));
    for (const Eigen::Vector3f& vertex : mesh.vertices)
    {
        const Eigen::Vector3d point = vertex.cast<double>();
        if (!Finite(point))
        {
            continue;
        }
        const double projection = (point - center).dot(zAxis);
        zValues.push_back(projection);
        minZ = std::min(minZ, projection);
        maxZ = std::max(maxZ, projection);
    }
    const double zBand = std::max(0.5, (maxZ - minZ) * 0.01);
    int nearMin = 0;
    int nearMax = 0;
    for (double projection : zValues)
    {
        nearMin += projection <= minZ + zBand ? 1 : 0;
        nearMax += projection >= maxZ - zBand ? 1 : 0;
    }
    if (nearMax > nearMin)
    {
        yAxis = -yAxis;
        zAxis = -zAxis;
    }

    Eigen::Matrix3d axes;
    axes.col(0) = xAxis;
    axes.col(1) = yAxis;
    axes.col(2) = zAxis;
    if (!IsOrthonormalRightHanded(axes))
    {
        error = QStringLiteral("生成的模型放置坐标系无效。");
        return false;
    }

    Eigen::Vector3d minimum = Eigen::Vector3d::Constant((std::numeric_limits<double>::max)());
    Eigen::Vector3d maximum = Eigen::Vector3d::Constant((std::numeric_limits<double>::lowest)());
    for (const Eigen::Vector3f& vertex : mesh.vertices)
    {
        const Eigen::Vector3d point = vertex.cast<double>();
        if (!Finite(point))
        {
            continue;
        }
        const Eigen::Vector3d projection = axes.transpose() * (point - center);
        minimum = minimum.cwiseMin(projection);
        maximum = maximum.cwiseMax(projection);
    }
    const Eigen::Vector3d span = (maximum - minimum).cwiseMax(Eigen::Vector3d::Constant(1.0e-6));
    double bestScore = (std::numeric_limits<double>::max)();
    Eigen::Vector3d bestPoint = center + axes * minimum;
    for (const Eigen::Vector3f& vertex : mesh.vertices)
    {
        const Eigen::Vector3d point = vertex.cast<double>();
        if (!Finite(point))
        {
            continue;
        }
        const Eigen::Vector3d normalized =
            (axes.transpose() * (point - center) - minimum).cwiseQuotient(span);
        const double score = normalized.squaredNorm();
        if (score < bestScore)
        {
            bestScore = score;
            bestPoint = point;
        }
    }

    guide.anchorModelMm = bestPoint;
    guide.axesModel = axes;
    guide.vSlotYawDegrees = 0.0;
    guide.longLengthMm = longLengthMm;
    guide.shortLengthMm = shortLengthMm;
    error.clear();
    return true;
}

QVector<ModelWeldingFeatureStation> ModelWeldingWorkflow::GenerateDraftStations(
    const WorkpieceMeshBuilder::Mesh& mesh,
    int solveCount,
    int verifyCount)
{
    QVector<ModelWeldingFeatureStation> result;
    const int clampedSolveCount = std::clamp(solveCount, 0, 12);
    const int clampedVerifyCount = std::clamp(verifyCount, 0, 4);
    const int requested = clampedSolveCount + clampedVerifyCount;
    if (!mesh.IsValid() || requested <= 0 || mesh.vertices.isEmpty())
    {
        return result;
    }

    QVector<Eigen::Vector3d> sample;
    const qsizetype step = std::max<qsizetype>(1, mesh.vertices.size() / 50000);
    for (qsizetype i = 0; i < mesh.vertices.size(); i += step)
    {
        const Eigen::Vector3d point = mesh.vertices.at(i).cast<double>();
        if (Finite(point))
        {
            sample.push_back(point);
        }
    }
    if (sample.isEmpty())
    {
        return result;
    }

    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    for (const Eigen::Vector3d& point : sample)
    {
        center += point;
    }
    center /= static_cast<double>(sample.size());

    QVector<Eigen::Vector3d> selected;
    for (int stationIndex = 0; stationIndex < requested; ++stationIndex)
    {
        double bestDistance = -1.0;
        Eigen::Vector3d bestPoint = sample.first();
        for (const Eigen::Vector3d& candidate : sample)
        {
            double distance = (candidate - center).squaredNorm();
            if (!selected.isEmpty())
            {
                distance = (std::numeric_limits<double>::max)();
                for (const Eigen::Vector3d& existing : selected)
                {
                    distance = std::min(distance, (candidate - existing).squaredNorm());
                }
            }
            if (distance > bestDistance)
            {
                bestDistance = distance;
                bestPoint = candidate;
            }
        }
        selected.push_back(bestPoint);

        ModelWeldingFeatureStation station;
        if (stationIndex < clampedSolveCount)
        {
            station.stationId = QStringLiteral("S%1").arg(stationIndex + 1, 2, 10, QLatin1Char('0'));
            station.displayName = QStringLiteral("求解候选 %1").arg(stationIndex + 1);
            station.role = ModelWeldingStationRole::Solve;
        }
        else
        {
            const int verifyIndex = stationIndex - clampedSolveCount + 1;
            station.stationId = QStringLiteral("V%1").arg(verifyIndex, 2, 10, QLatin1Char('0'));
            station.displayName = QStringLiteral("验证候选 %1").arg(verifyIndex);
            station.role = ModelWeldingStationRole::Verify;
        }
        station.anchorModelMm = bestPoint;
        const Eigen::Vector3d direction = bestPoint - center;
        station.scanDirectionModel = direction.norm() > 1.0e-9
            ? direction.normalized()
            : Eigen::Vector3d::UnitX();
        station.candidateConfirmed = false;
        result.push_back(station);
    }
    return result;
}

bool ModelWeldingWorkflow::ValidateTemplateStructure(
    const ModelWeldingFlowTemplate& value,
    QString& error,
    bool requireProductionReady)
{
    if (value.schemaVersion != ModelWeldingFlowTemplate::SchemaVersion)
    {
        error = QStringLiteral("模型流程模板版本不支持。");
        return false;
    }
    if (!IsStableId(value.templateId) || value.revision < 1
        || value.revision > kMaximumExactJsonInteger)
    {
        error = QStringLiteral("模型流程模板ID或修订号无效。");
        return false;
    }
    if (value.displayName.trimmed().isEmpty() || value.displayName.size() > 120
        || value.modelLibraryName.trimmed().isEmpty() || value.modelLibraryName.size() > 120
        || !IsSha256(value.modelSha256)
        || value.units != QStringLiteral("mm"))
    {
        error = QStringLiteral("模型名称、模型SHA-256或单位无效。");
        return false;
    }
    if (!Finite(value.placement.anchorModelMm)
        || !IsOrthonormalRightHanded(value.placement.axesModel)
        || !Finite(value.placement.vSlotYawDegrees)
        || value.placement.vSlotYawDegrees < -180.0
        || value.placement.vSlotYawDegrees >= 180.0
        || !Finite(value.placement.longLengthMm)
        || !Finite(value.placement.shortLengthMm)
        || value.placement.longLengthMm <= value.placement.shortLengthMm
        || value.placement.shortLengthMm <= 0.0
        || value.placement.longLengthMm > 10000.0)
    {
        error = QStringLiteral("V型槽放置引导无效。");
        return false;
    }
    if (value.inheritedFromTemplateId.isEmpty())
    {
        if (value.inheritedFromRevision != 0 || !value.inheritedFromRecordSha256.isEmpty())
        {
            error = QStringLiteral("相似模型继承来源字段不完整。");
            return false;
        }
    }
    else if (!IsStableId(value.inheritedFromTemplateId)
        || value.inheritedFromTemplateId == value.templateId
        || value.inheritedFromRevision < 1
        || value.inheritedFromRevision > kMaximumExactJsonInteger
        || !IsSha256(value.inheritedFromRecordSha256))
    {
        error = QStringLiteral("相似模型继承来源ID、修订或哈希无效。");
        return false;
    }
    if (value.stations.size() > kMaximumStationCount)
    {
        error = QStringLiteral("扫描站数量超过上限。");
        return false;
    }

    if (value.seams.size() > kMaximumSeamCount)
    {
        error = QStringLiteral("焊缝定义数量超过上限。");
        return false;
    }

    QMap<QString, bool> ids;
    QVector<Eigen::Vector3d> confirmedSolvePoints;
    int confirmedVerifyCount = 0;
    for (const ModelWeldingFeatureStation& station : value.stations)
    {
        const QString id = station.stationId.trimmed();
        if (id.isEmpty() || id != station.stationId || id.size() > 24 || ids.contains(id))
        {
            error = QStringLiteral("扫描站编号为空、过长或重复：%1").arg(id);
            return false;
        }
        ids.insert(id, true);
        if (!Finite(station.anchorModelMm) || !Finite(station.scanDirectionModel)
            || !Finite(station.roiHalfExtentMm)
            || station.scanDirectionModel.norm() < 1.0e-6
            || station.roiHalfExtentMm.minCoeff() <= 0.0
            || station.roiHalfExtentMm.maxCoeff() > 5000.0)
        {
            error = QStringLiteral("扫描站 %1 的模型位置、方向或ROI无效。").arg(id);
            return false;
        }
        if (station.candidateConfirmed && station.role == ModelWeldingStationRole::Solve)
        {
            confirmedSolvePoints.push_back(station.anchorModelMm);
        }
        if (station.candidateConfirmed && station.role == ModelWeldingStationRole::Verify)
        {
            ++confirmedVerifyCount;
        }
    }

    QMap<QString, bool> seamIds;
    qsizetype totalSeamPoints = 0;
    int confirmedSeamCount = 0;
    for (const ModelWeldingSeamDefinition& seam : value.seams)
    {
        const QString id = seam.seamId.trimmed();
        if (!IsStableIdentifier(id) || id != seam.seamId || seamIds.contains(id))
        {
            error = QStringLiteral("焊缝编号为空、格式无效或重复：%1").arg(id);
            return false;
        }
        seamIds.insert(id, true);
        if (!IsSha256(seam.sourceGeometrySha256)
            || (!seam.seedPathSha256.isEmpty() && !IsSha256(seam.seedPathSha256))
            || seam.pathModelMm.size() < 2
            || seam.pathModelMm.size() > kMaximumSeamPointCount
            || !Finite(seam.lengthMm) || seam.lengthMm <= 0.0)
        {
            error = QStringLiteral("焊缝 %1 的源身份、点数或长度无效。").arg(id);
            return false;
        }
        const bool reverseMesh = seam.source
            == ModelWeldingSeamSource::ReverseMeshSeedProjection;
        if ((reverseMesh && !IsSha256(seam.seedPathSha256))
            || (!reverseMesh && !seam.seedPathSha256.isEmpty()))
        {
            error = QStringLiteral("焊缝 %1 的种子身份与来源类型不一致。").arg(id);
            return false;
        }
        totalSeamPoints += seam.pathModelMm.size();
        if (totalSeamPoints > kMaximumTotalSeamPointCount)
        {
            error = QStringLiteral("模板焊缝总点数超过上限。");
            return false;
        }
        double calculatedLength = 0.0;
        for (int pointIndex = 0; pointIndex < seam.pathModelMm.size(); ++pointIndex)
        {
            if (!Finite(seam.pathModelMm.at(pointIndex)))
            {
                error = QStringLiteral("焊缝 %1 含非有限坐标。").arg(id);
                return false;
            }
            if (pointIndex > 0)
            {
                const double step = (seam.pathModelMm.at(pointIndex)
                    - seam.pathModelMm.at(pointIndex - 1)).norm();
                if (!Finite(step) || step <= 1.0e-9 || step > 1000.0)
                {
                    error = QStringLiteral("焊缝 %1 含重复点或异常大步长。").arg(id);
                    return false;
                }
                calculatedLength += step;
            }
        }
        const double allowedLengthError = std::max(0.05, calculatedLength * 1.0e-6);
        if (std::abs(calculatedLength - seam.lengthMm) > allowedLengthError)
        {
            error = QStringLiteral("焊缝 %1 的记录长度与点列不一致。").arg(id);
            return false;
        }
        if (seam.humanConfirmed) ++confirmedSeamCount;
    }

    if (requireProductionReady)
    {
        if (!value.humanDatumConfirmed)
        {
            error = QStringLiteral("尚未人工确认V型槽+X、+Y和地面+Z方向。");
            return false;
        }
        if (!value.humanCollisionChecked)
        {
            error = QStringLiteral("尚未完成人工碰撞范围检查。");
            return false;
        }
        if (!value.inheritedFromTemplateId.isEmpty()
            && (!value.humanSameFixtureConfirmed
                || !value.humanDatumConfirmed
                || !value.humanScanAreaConfirmed))
        {
            error = QStringLiteral("相似模型继承尚未完成同工装、基准和扫描区域人工复核。");
            return false;
        }
        if (confirmedSolvePoints.size() < 3 || confirmedVerifyCount < 1)
        {
            error = QStringLiteral("生产流程至少需要3个已确认求解站和1个已确认验证站。");
            return false;
        }
        if (confirmedSeamCount < 1)
        {
            error = QStringLiteral("生产流程至少需要1条已人工确认焊缝。");
            return false;
        }
        Eigen::Vector3d center = Eigen::Vector3d::Zero();
        for (const Eigen::Vector3d& point : confirmedSolvePoints)
        {
            center += point;
        }
        center /= static_cast<double>(confirmedSolvePoints.size());
        Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
        for (const Eigen::Vector3d& point : confirmedSolvePoints)
        {
            const Eigen::Vector3d delta = point - center;
            covariance += delta * delta.transpose();
        }
        const Eigen::JacobiSVD<Eigen::Matrix3d> svd(covariance);
        const Eigen::Vector3d singular = svd.singularValues();
        if (!singular.allFinite() || singular.x() <= 1.0e-12
            || singular.y() / singular.x() < 1.0e-4)
        {
            error = QStringLiteral("已确认求解站近共线，无法稳定求解六自由度刚体矩阵。");
            return false;
        }
    }
    error.clear();
    return true;
}

bool ModelWeldingWorkflow::ValidateTeachingStructure(
    const ModelWeldingRobotTeaching& value,
    const ModelWeldingFlowTemplate* modelTemplate,
    QString& error,
    bool requireProductionReady)
{
    if (value.schemaVersion != ModelWeldingRobotTeaching::SchemaVersion
        || !IsStableId(value.teachingId) || value.revision < 1
        || value.revision > kMaximumExactJsonInteger
        || !IsStableId(value.templateId) || value.templateRevision < 1
        || value.templateRevision > kMaximumExactJsonInteger
        || !IsSha256(value.templateRecordSha256)
        || value.robotName.trimmed().isEmpty() || value.robotName.size() > 120
        || value.robotName != value.robotName.trimmed()
        || value.robotEndpoint.size() > 512
        || (!value.robotEndpoint.isEmpty()
            && value.robotEndpoint != value.robotEndpoint.trimmed())
        || value.robotModelId.size() > 128
        || (!value.robotModelId.isEmpty()
            && !IsStableIdentifier(value.robotModelId))
        || (!value.sourceStepSha256.isEmpty()
            && !IsSha256(value.sourceStepSha256))
        || (!value.collisionProfileSha256.isEmpty()
            && !IsSha256(value.collisionProfileSha256))
        || value.cameraSection.size() > 120
        || (!value.cameraSection.isEmpty()
            && value.cameraSection != value.cameraSection.trimmed())
        || (!value.handEyeSha256.isEmpty() && !IsSha256(value.handEyeSha256))
        || (!value.tool1Sha256.isEmpty() && !IsSha256(value.tool1Sha256)))
    {
        error = QStringLiteral("机器人示教记录身份或绑定信息无效。");
        return false;
    }
    const bool hasAnyRobotModelIdentity = !value.robotModelId.isEmpty()
        || !value.sourceStepSha256.isEmpty()
        || !value.collisionProfileSha256.isEmpty();
    if (hasAnyRobotModelIdentity
        && (value.robotModelId.isEmpty()
            || value.sourceStepSha256.isEmpty()
            || value.collisionProfileSha256.isEmpty()))
    {
        error = QStringLiteral("机器人示教记录的型号、源STEP和碰撞简模身份必须成组保存。");
        return false;
    }
    if (value.scans.size() > kMaximumStationCount)
    {
        error = QStringLiteral("机器人示教扫描站数量超过上限。");
        return false;
    }
    if (modelTemplate != nullptr
        && (modelTemplate->templateId != value.templateId
            || modelTemplate->revision != value.templateRevision))
    {
        error = QStringLiteral("机器人示教记录与模型模板修订不匹配。");
        return false;
    }
    if (modelTemplate != nullptr)
    {
        QString hashError;
        const QString expectedTemplateHash = TemplateRecordSha256(*modelTemplate, hashError);
        if (expectedTemplateHash.isEmpty() || expectedTemplateHash != value.templateRecordSha256.toLower())
        {
            error = hashError.isEmpty()
                ? QStringLiteral("机器人示教记录绑定的模型模板哈希不匹配。")
                : QStringLiteral("模型模板哈希计算失败：%1").arg(hashError);
            return false;
        }
    }

    if (requireProductionReady)
    {
        if (modelTemplate == nullptr)
        {
            error = QStringLiteral("生产就绪检查必须提供模型流程模板。");
            return false;
        }
        QString templateError;
        if (!ValidateTemplateStructure(*modelTemplate, templateError, true))
        {
            error = QStringLiteral("模型流程模板未达到生产就绪条件：%1").arg(templateError);
            return false;
        }
        if (value.robotEndpoint.trimmed().isEmpty()
            || !IsStableIdentifier(value.robotModelId)
            || !IsSha256(value.sourceStepSha256)
            || !IsSha256(value.collisionProfileSha256)
            || value.cameraSection.trimmed().isEmpty()
            || !IsSha256(value.handEyeSha256)
            || !IsSha256(value.tool1Sha256))
        {
            error = QStringLiteral(
                "生产运行前必须绑定机器人端点、机器人型号资产、相机、手眼参数哈希和Tool1参数哈希。");
            return false;
        }
    }

    QMap<QString, bool> ids;
    for (const ModelWeldingScanTeaching& scan : value.scans)
    {
        const QString id = scan.stationId.trimmed();
        if (id.isEmpty() || id != scan.stationId || ids.contains(id))
        {
            error = QStringLiteral("示教扫描站编号为空或重复：%1").arg(id);
            return false;
        }
        ids.insert(id, true);
        bool requiredForProduction = requireProductionReady;
        if (modelTemplate != nullptr)
        {
            const auto stationIt = std::find_if(
                modelTemplate->stations.cbegin(),
                modelTemplate->stations.cend(),
                [&id](const ModelWeldingFeatureStation& station)
                {
                    return station.stationId == id;
                });
            if (stationIt == modelTemplate->stations.cend())
            {
                error = QStringLiteral("示教记录包含模板中不存在的扫描站：%1").arg(id);
                return false;
            }
            requiredForProduction = requireProductionReady
                && stationIt->candidateConfirmed
                && stationIt->role != ModelWeldingStationRole::Backup;
        }
        if (!Finite(scan.runSpeedMmPerMin) || !Finite(scan.scanSpeedMmPerMin)
            || scan.runSpeedMmPerMin <= 0.0 || scan.scanSpeedMmPerMin <= 0.0
            || scan.runSpeedMmPerMin > 100000.0 || scan.scanSpeedMmPerMin > 100000.0)
        {
            error = QStringLiteral("扫描站 %1 的速度无效。").arg(id);
            return false;
        }
        if (!FinitePose(scan.startPose) || !FinitePose(scan.endPose))
        {
            error = QStringLiteral("扫描站 %1 的示教位姿包含非有限值。").arg(id);
            return false;
        }
        if (requiredForProduction
            && (!scan.startTaught || !scan.endTaught
                || !scan.startPulseTaught
                || PoseDistanceMm(scan.startPose, scan.endPose) < 10.0))
        {
            error = QStringLiteral("扫描站 %1 未完成有效起终点/起点关节示教，或扫描长度小于10mm。").arg(id);
            return false;
        }
    }

    if (requireProductionReady && modelTemplate != nullptr)
    {
        for (const ModelWeldingFeatureStation& station : modelTemplate->stations)
        {
            if (!station.candidateConfirmed || station.role == ModelWeldingStationRole::Backup)
            {
                continue;
            }
            if (!ids.contains(station.stationId))
            {
                error = QStringLiteral("缺少必需扫描站 %1 的机器人示教记录。").arg(station.stationId);
                return false;
            }
        }
    }
    error.clear();
    return true;
}

QByteArray ModelWeldingWorkflow::EncodeTemplate(const ModelWeldingFlowTemplate& value, QString& error)
{
    if (!ValidateTemplateStructure(value, error, false))
    {
        return QByteArray();
    }
    QJsonObject placement;
    placement.insert(QStringLiteral("anchorModelMm"), VectorJson(value.placement.anchorModelMm));
    placement.insert(QStringLiteral("axesModel"), MatrixJson(value.placement.axesModel));
    if (std::abs(value.placement.vSlotYawDegrees) > 1.0e-12)
    {
        placement.insert(QStringLiteral("vSlotYawDegrees"), value.placement.vSlotYawDegrees);
    }
    placement.insert(QStringLiteral("longLengthMm"), value.placement.longLengthMm);
    placement.insert(QStringLiteral("shortLengthMm"), value.placement.shortLengthMm);

    QJsonArray stations;
    for (const ModelWeldingFeatureStation& station : value.stations)
    {
        QJsonObject object;
        object.insert(QStringLiteral("stationId"), station.stationId);
        object.insert(QStringLiteral("displayName"), station.displayName);
        object.insert(QStringLiteral("role"), RoleText(station.role));
        object.insert(QStringLiteral("anchorModelMm"), VectorJson(station.anchorModelMm));
        object.insert(QStringLiteral("scanDirectionModel"), VectorJson(station.scanDirectionModel));
        object.insert(QStringLiteral("roiHalfExtentMm"), VectorJson(station.roiHalfExtentMm));
        object.insert(QStringLiteral("candidateConfirmed"), station.candidateConfirmed);
        stations.append(object);
    }

    QJsonArray seams;
    for (const ModelWeldingSeamDefinition& seam : value.seams)
    {
        QJsonArray points;
        for (const Eigen::Vector3d& point : seam.pathModelMm)
            points.append(VectorJson(point));
        QJsonObject object;
        object.insert(QStringLiteral("seamId"), seam.seamId);
        object.insert(QStringLiteral("source"), SeamSourceText(seam.source));
        object.insert(QStringLiteral("sourceGeometrySha256"),
            seam.sourceGeometrySha256.toLower());
        object.insert(QStringLiteral("seedPathSha256"), seam.seedPathSha256.toLower());
        object.insert(QStringLiteral("pathModelMm"), points);
        object.insert(QStringLiteral("lengthMm"), seam.lengthMm);
        object.insert(QStringLiteral("humanConfirmed"), seam.humanConfirmed);
        seams.append(object);
    }

    QJsonObject root;
    root.insert(QStringLiteral("schemaVersion"), value.schemaVersion);
    root.insert(QStringLiteral("templateId"), value.templateId);
    root.insert(QStringLiteral("revision"), static_cast<double>(value.revision));
    root.insert(QStringLiteral("displayName"), value.displayName);
    root.insert(QStringLiteral("modelLibraryName"), value.modelLibraryName);
    root.insert(QStringLiteral("modelSha256"), value.modelSha256.toLower());
    root.insert(QStringLiteral("units"), value.units);
    root.insert(QStringLiteral("placement"), placement);
    root.insert(QStringLiteral("stations"), stations);
    root.insert(QStringLiteral("seams"), seams);
    root.insert(QStringLiteral("inheritedFromTemplateId"), value.inheritedFromTemplateId);
    root.insert(QStringLiteral("inheritedFromRevision"), static_cast<double>(value.inheritedFromRevision));
    root.insert(QStringLiteral("inheritedFromRecordSha256"), value.inheritedFromRecordSha256.toLower());
    root.insert(QStringLiteral("humanSameFixtureConfirmed"), value.humanSameFixtureConfirmed);
    root.insert(QStringLiteral("humanDatumConfirmed"), value.humanDatumConfirmed);
    root.insert(QStringLiteral("humanScanAreaConfirmed"), value.humanScanAreaConfirmed);
    root.insert(QStringLiteral("humanCollisionChecked"), value.humanCollisionChecked);
    root.insert(QStringLiteral("createdAtUtc"), value.createdAtUtc);
    root.insert(QStringLiteral("updatedAtUtc"), value.updatedAtUtc);
    const QByteArray json = CompactJson(root);
    if (json.size() > kMaximumRecordBytes)
    {
        error = QStringLiteral("模型流程模板超过大小上限。");
        return QByteArray();
    }
    return json;
}

bool ModelWeldingWorkflow::DecodeTemplate(
    const QByteArray& json,
    ModelWeldingFlowTemplate& value,
    QString& error)
{
    if (json.isEmpty() || json.size() > kMaximumRecordBytes)
    {
        error = QStringLiteral("模型流程模板为空或超过大小上限。");
        return false;
    }
    QJsonParseError parseError;
    const QJsonDocument document = QJsonDocument::fromJson(json, &parseError);
    if (parseError.error != QJsonParseError::NoError || !document.isObject())
    {
        error = QStringLiteral("模型流程模板JSON无效：%1").arg(parseError.errorString());
        return false;
    }
    const QJsonObject root = document.object();
    ModelWeldingFlowTemplate decoded;
    const QJsonValue schemaValue = root.value(QStringLiteral("schemaVersion"));
    const int storedSchemaVersion = schemaValue.isDouble()
        ? schemaValue.toInt(-1) : -1;
    if (!schemaValue.isDouble()
        || (storedSchemaVersion != 1
            && storedSchemaVersion != ModelWeldingFlowTemplate::SchemaVersion)
        || !JsonString(root, QStringLiteral("templateId"), decoded.templateId)
        || !JsonRevision(root, QStringLiteral("revision"), decoded.revision)
        || !JsonString(root, QStringLiteral("displayName"), decoded.displayName)
        || !JsonString(root, QStringLiteral("modelLibraryName"), decoded.modelLibraryName)
        || !JsonString(root, QStringLiteral("modelSha256"), decoded.modelSha256)
        || !JsonString(root, QStringLiteral("units"), decoded.units)
        || !root.value(QStringLiteral("placement")).isObject()
        || !root.value(QStringLiteral("stations")).isArray())
    {
        error = QStringLiteral("模型流程模板缺少必需字段或字段类型错误。");
        return false;
    }
    decoded.modelSha256 = decoded.modelSha256.toLower();
    decoded.schemaVersion = ModelWeldingFlowTemplate::SchemaVersion;
    const QJsonObject placement = root.value(QStringLiteral("placement")).toObject();
    if (!ParseVector(placement.value(QStringLiteral("anchorModelMm")), decoded.placement.anchorModelMm)
        || !ParseMatrix(placement.value(QStringLiteral("axesModel")), decoded.placement.axesModel)
        || !placement.value(QStringLiteral("longLengthMm")).isDouble()
        || !placement.value(QStringLiteral("shortLengthMm")).isDouble())
    {
        error = QStringLiteral("模型流程模板的V型槽放置引导字段无效。");
        return false;
    }
    decoded.placement.longLengthMm = placement.value(QStringLiteral("longLengthMm")).toDouble();
    decoded.placement.shortLengthMm = placement.value(QStringLiteral("shortLengthMm")).toDouble();
    const QJsonValue vSlotYaw = placement.value(QStringLiteral("vSlotYawDegrees"));
    if (!vSlotYaw.isUndefined())
    {
        if (!vSlotYaw.isDouble())
        {
            error = QStringLiteral("模型流程模板的V型槽偏航角字段无效。");
            return false;
        }
        decoded.placement.vSlotYawDegrees = vSlotYaw.toDouble();
    }

    const QJsonArray stations = root.value(QStringLiteral("stations")).toArray();
    if (stations.size() > kMaximumStationCount)
    {
        error = QStringLiteral("模型流程模板扫描站数量超过上限。");
        return false;
    }
    for (const QJsonValue& stationValue : stations)
    {
        if (!stationValue.isObject())
        {
            error = QStringLiteral("模型流程模板扫描站字段不是对象。");
            return false;
        }
        const QJsonObject object = stationValue.toObject();
        ModelWeldingFeatureStation station;
        QString role;
        if (!JsonString(object, QStringLiteral("stationId"), station.stationId)
            || !JsonString(object, QStringLiteral("displayName"), station.displayName, true)
            || !JsonString(object, QStringLiteral("role"), role)
            || !ParseRole(role, station.role)
            || !ParseVector(object.value(QStringLiteral("anchorModelMm")), station.anchorModelMm)
            || !ParseVector(object.value(QStringLiteral("scanDirectionModel")), station.scanDirectionModel)
            || !ParseVector(object.value(QStringLiteral("roiHalfExtentMm")), station.roiHalfExtentMm)
            || !JsonBool(object, QStringLiteral("candidateConfirmed"), station.candidateConfirmed))
        {
            error = QStringLiteral("模型流程模板扫描站字段无效。");
            return false;
        }
        decoded.stations.push_back(station);
    }

    const QJsonValue seamsValue = root.value(QStringLiteral("seams"));
    if (storedSchemaVersion >= 2 && !seamsValue.isArray())
    {
        error = QStringLiteral("模型流程模板缺少焊缝定义数组。");
        return false;
    }
    if (seamsValue.isArray())
    {
        const QJsonArray seams = seamsValue.toArray();
        if (seams.size() > kMaximumSeamCount)
        {
            error = QStringLiteral("模型流程模板焊缝定义数量超过上限。");
            return false;
        }
        for (const QJsonValue& seamValue : seams)
        {
            if (!seamValue.isObject())
            {
                error = QStringLiteral("模型流程模板焊缝定义不是对象。");
                return false;
            }
            const QJsonObject object = seamValue.toObject();
            ModelWeldingSeamDefinition seam;
            QString source;
            if (!JsonString(object, QStringLiteral("seamId"), seam.seamId)
                || !JsonString(object, QStringLiteral("source"), source)
                || !ParseSeamSource(source, seam.source)
                || !JsonString(object, QStringLiteral("sourceGeometrySha256"),
                    seam.sourceGeometrySha256)
                || !JsonString(object, QStringLiteral("seedPathSha256"),
                    seam.seedPathSha256, true)
                || !object.value(QStringLiteral("pathModelMm")).isArray()
                || !object.value(QStringLiteral("lengthMm")).isDouble()
                || !JsonBool(object, QStringLiteral("humanConfirmed"), seam.humanConfirmed))
            {
                error = QStringLiteral("模型流程模板焊缝定义字段无效。");
                return false;
            }
            seam.sourceGeometrySha256 = seam.sourceGeometrySha256.toLower();
            seam.seedPathSha256 = seam.seedPathSha256.toLower();
            seam.lengthMm = object.value(QStringLiteral("lengthMm")).toDouble();
            const QJsonArray points = object.value(QStringLiteral("pathModelMm")).toArray();
            if (points.size() < 2 || points.size() > kMaximumSeamPointCount)
            {
                error = QStringLiteral("模型流程模板焊缝点数无效。");
                return false;
            }
            seam.pathModelMm.reserve(points.size());
            for (const QJsonValue& pointValue : points)
            {
                Eigen::Vector3d point;
                if (!ParseVector(pointValue, point))
                {
                    error = QStringLiteral("模型流程模板焊缝坐标无效。");
                    return false;
                }
                seam.pathModelMm.push_back(point);
            }
            decoded.seams.push_back(seam);
        }
    }

    if (!JsonString(root, QStringLiteral("inheritedFromTemplateId"), decoded.inheritedFromTemplateId, true)
        || !JsonRevision(root, QStringLiteral("inheritedFromRevision"), decoded.inheritedFromRevision, true)
        || !JsonString(root, QStringLiteral("inheritedFromRecordSha256"), decoded.inheritedFromRecordSha256, true)
        || !JsonBool(root, QStringLiteral("humanSameFixtureConfirmed"), decoded.humanSameFixtureConfirmed)
        || !JsonBool(root, QStringLiteral("humanDatumConfirmed"), decoded.humanDatumConfirmed)
        || !JsonBool(root, QStringLiteral("humanScanAreaConfirmed"), decoded.humanScanAreaConfirmed)
        || !JsonBool(root, QStringLiteral("humanCollisionChecked"), decoded.humanCollisionChecked)
        || !JsonString(root, QStringLiteral("createdAtUtc"), decoded.createdAtUtc, true)
        || !JsonString(root, QStringLiteral("updatedAtUtc"), decoded.updatedAtUtc, true))
    {
        error = QStringLiteral("模型流程模板继承或审计字段无效。");
        return false;
    }
    decoded.inheritedFromRecordSha256 = decoded.inheritedFromRecordSha256.toLower();
    if ((!decoded.inheritedFromTemplateId.isEmpty() && !IsStableId(decoded.inheritedFromTemplateId))
        || (!decoded.inheritedFromRecordSha256.isEmpty() && !IsSha256(decoded.inheritedFromRecordSha256)))
    {
        error = QStringLiteral("模型流程模板继承来源无效。");
        return false;
    }
    if (!ValidateTemplateStructure(decoded, error, false))
    {
        return false;
    }
    value = decoded;
    return true;
}

QByteArray ModelWeldingWorkflow::EncodeTeaching(const ModelWeldingRobotTeaching& value, QString& error)
{
    if (!ValidateTeachingStructure(value, nullptr, error, false))
    {
        return QByteArray();
    }
    QJsonArray scans;
    for (const ModelWeldingScanTeaching& scan : value.scans)
    {
        QJsonObject object;
        object.insert(QStringLiteral("stationId"), scan.stationId);
        object.insert(QStringLiteral("startTaught"), scan.startTaught);
        object.insert(QStringLiteral("endTaught"), scan.endTaught);
        object.insert(QStringLiteral("startPose"), PoseJson(scan.startPose));
        object.insert(QStringLiteral("endPose"), PoseJson(scan.endPose));
        object.insert(QStringLiteral("startPulseTaught"), scan.startPulseTaught);
        object.insert(QStringLiteral("startPulse"), PulseJson(scan.startPulse));
        object.insert(QStringLiteral("runSpeedMmPerMin"), scan.runSpeedMmPerMin);
        object.insert(QStringLiteral("scanSpeedMmPerMin"), scan.scanSpeedMmPerMin);
        scans.append(object);
    }
    QJsonObject root;
    root.insert(QStringLiteral("schemaVersion"), value.schemaVersion);
    root.insert(QStringLiteral("teachingId"), value.teachingId);
    root.insert(QStringLiteral("revision"), static_cast<double>(value.revision));
    root.insert(QStringLiteral("templateId"), value.templateId);
    root.insert(QStringLiteral("templateRevision"), static_cast<double>(value.templateRevision));
    root.insert(QStringLiteral("templateRecordSha256"), value.templateRecordSha256.toLower());
    root.insert(QStringLiteral("robotName"), value.robotName);
    root.insert(QStringLiteral("robotEndpoint"), value.robotEndpoint);
    root.insert(QStringLiteral("robotModelId"), value.robotModelId);
    root.insert(QStringLiteral("sourceStepSha256"), value.sourceStepSha256.toLower());
    root.insert(QStringLiteral("collisionProfileSha256"), value.collisionProfileSha256.toLower());
    root.insert(QStringLiteral("cameraSection"), value.cameraSection);
    root.insert(QStringLiteral("handEyeSha256"), value.handEyeSha256.toLower());
    root.insert(QStringLiteral("tool1Sha256"), value.tool1Sha256.toLower());
    root.insert(QStringLiteral("scans"), scans);
    root.insert(QStringLiteral("createdAtUtc"), value.createdAtUtc);
    root.insert(QStringLiteral("updatedAtUtc"), value.updatedAtUtc);
    const QByteArray json = CompactJson(root);
    if (json.size() > kMaximumRecordBytes)
    {
        error = QStringLiteral("机器人示教记录超过大小上限。");
        return QByteArray();
    }
    return json;
}

bool ModelWeldingWorkflow::DecodeTeaching(
    const QByteArray& json,
    ModelWeldingRobotTeaching& value,
    QString& error)
{
    if (json.isEmpty() || json.size() > kMaximumRecordBytes)
    {
        error = QStringLiteral("机器人示教记录为空或超过大小上限。");
        return false;
    }
    QJsonParseError parseError;
    const QJsonDocument document = QJsonDocument::fromJson(json, &parseError);
    if (parseError.error != QJsonParseError::NoError || !document.isObject())
    {
        error = QStringLiteral("机器人示教JSON无效：%1").arg(parseError.errorString());
        return false;
    }
    const QJsonObject root = document.object();
    ModelWeldingRobotTeaching decoded;
    const QJsonValue schemaValue = root.value(QStringLiteral("schemaVersion"));
    const double storedSchemaNumber = schemaValue.isDouble() ? schemaValue.toDouble(-1.0) : -1.0;
    const int storedSchemaVersion = schemaValue.isDouble() ? schemaValue.toInt(-1) : -1;
    if (!schemaValue.isDouble()
        || !std::isfinite(storedSchemaNumber)
        || storedSchemaNumber != static_cast<double>(storedSchemaVersion)
        || storedSchemaVersion < 1
        || storedSchemaVersion > ModelWeldingRobotTeaching::SchemaVersion
        || !JsonString(root, QStringLiteral("teachingId"), decoded.teachingId)
        || !JsonRevision(root, QStringLiteral("revision"), decoded.revision)
        || !JsonString(root, QStringLiteral("templateId"), decoded.templateId)
        || !JsonRevision(root, QStringLiteral("templateRevision"), decoded.templateRevision)
        || !JsonString(root, QStringLiteral("templateRecordSha256"), decoded.templateRecordSha256)
        || !JsonString(root, QStringLiteral("robotName"), decoded.robotName)
        || !JsonString(root, QStringLiteral("cameraSection"), decoded.cameraSection, true)
        || !JsonString(root, QStringLiteral("handEyeSha256"), decoded.handEyeSha256, true)
        || !JsonString(root, QStringLiteral("tool1Sha256"), decoded.tool1Sha256, true)
        || !root.value(QStringLiteral("scans")).isArray()
        || !JsonString(root, QStringLiteral("createdAtUtc"), decoded.createdAtUtc, true)
        || !JsonString(root, QStringLiteral("updatedAtUtc"), decoded.updatedAtUtc, true))
    {
        error = QStringLiteral("机器人示教记录缺少必需字段或字段类型错误。");
        return false;
    }
    if (storedSchemaVersion >= 2
        && !JsonString(root, QStringLiteral("robotEndpoint"), decoded.robotEndpoint, true))
    {
        error = QStringLiteral("机器人示教记录缺少机器人端点字段或字段类型错误。");
        return false;
    }
    if (storedSchemaVersion >= 3
        && (!JsonString(root, QStringLiteral("robotModelId"), decoded.robotModelId, true)
            || !JsonString(root, QStringLiteral("sourceStepSha256"), decoded.sourceStepSha256, true)
            || !JsonString(root, QStringLiteral("collisionProfileSha256"), decoded.collisionProfileSha256, true)))
    {
        error = QStringLiteral("机器人示教记录缺少机器人型号资产身份字段或字段类型错误。");
        return false;
    }
    decoded.schemaVersion = ModelWeldingRobotTeaching::SchemaVersion;
    decoded.templateRecordSha256 = decoded.templateRecordSha256.toLower();
    decoded.robotModelId = decoded.robotModelId.toLower();
    decoded.sourceStepSha256 = decoded.sourceStepSha256.toLower();
    decoded.collisionProfileSha256 = decoded.collisionProfileSha256.toLower();
    decoded.handEyeSha256 = decoded.handEyeSha256.toLower();
    decoded.tool1Sha256 = decoded.tool1Sha256.toLower();
    const QJsonArray scans = root.value(QStringLiteral("scans")).toArray();
    if (scans.size() > kMaximumStationCount)
    {
        error = QStringLiteral("机器人示教扫描站数量超过上限。");
        return false;
    }
    for (const QJsonValue& scanValue : scans)
    {
        if (!scanValue.isObject())
        {
            error = QStringLiteral("机器人示教扫描站字段不是对象。");
            return false;
        }
        const QJsonObject object = scanValue.toObject();
        ModelWeldingScanTeaching scan;
        if (!JsonString(object, QStringLiteral("stationId"), scan.stationId)
            || !JsonBool(object, QStringLiteral("startTaught"), scan.startTaught)
            || !JsonBool(object, QStringLiteral("endTaught"), scan.endTaught)
            || !ParsePose(object.value(QStringLiteral("startPose")), scan.startPose)
            || !ParsePose(object.value(QStringLiteral("endPose")), scan.endPose)
            || !JsonBool(object, QStringLiteral("startPulseTaught"), scan.startPulseTaught)
            || !ParsePulse(object.value(QStringLiteral("startPulse")), scan.startPulse)
            || !object.value(QStringLiteral("runSpeedMmPerMin")).isDouble()
            || !object.value(QStringLiteral("scanSpeedMmPerMin")).isDouble())
        {
            error = QStringLiteral("机器人示教扫描站字段无效。");
            return false;
        }
        scan.runSpeedMmPerMin = object.value(QStringLiteral("runSpeedMmPerMin")).toDouble();
        scan.scanSpeedMmPerMin = object.value(QStringLiteral("scanSpeedMmPerMin")).toDouble();
        decoded.scans.push_back(scan);
    }
    if (!ValidateTeachingStructure(decoded, nullptr, error, false))
    {
        return false;
    }
    value = decoded;
    return true;
}

QString ModelWeldingWorkflow::TemplateRecordSha256(const ModelWeldingFlowTemplate& value, QString& error)
{
    const QByteArray json = EncodeTemplate(value, error);
    if (json.isEmpty())
    {
        return QString();
    }
    return QString::fromLatin1(QCryptographicHash::hash(json, QCryptographicHash::Sha256).toHex()).toLower();
}

bool ModelWeldingWorkflow::SaveTemplate(
    const ModelWeldingFlowTemplate& value,
    quint64 expectedPreviousRevision,
    QString& error)
{
    QMutexLocker lock(&StoreMutex());
    if (!ValidateTemplateStructure(value, error, false))
    {
        return false;
    }
    QByteArray inheritanceSourceJson;
    if (expectedPreviousRevision == 0 && !value.inheritedFromTemplateId.isEmpty())
    {
        if (!ReadStoredJson(
            kTemplateScope, value.inheritedFromTemplateId, inheritanceSourceJson, error))
        {
            if (error.isEmpty())
            {
                error = QStringLiteral("相似模型继承的来源模板已不存在。");
            }
            return false;
        }
        ModelWeldingFlowTemplate sourceTemplate;
        QString sourceHashError;
        if (!DecodeTemplate(inheritanceSourceJson, sourceTemplate, error))
        {
            return false;
        }
        if (sourceTemplate.templateId != value.inheritedFromTemplateId)
        {
            error = QStringLiteral("继承来源配置作用域ID与JSON内部ID不一致。");
            return false;
        }
        const QString sourceHash = TemplateRecordSha256(sourceTemplate, sourceHashError);
        if (sourceTemplate.revision != value.inheritedFromRevision
            || sourceHash.isEmpty()
            || sourceHash != value.inheritedFromRecordSha256.toLower())
        {
            error = sourceHashError.isEmpty()
                ? QStringLiteral("相似模型继承来源已更新，请重新执行继承审核。")
                : QStringLiteral("继承来源哈希计算失败：%1").arg(sourceHashError);
            return false;
        }
    }
    QByteArray existingJson;
    QString readError;
    const bool exists = ReadStoredJson(kTemplateScope, value.templateId, existingJson, readError);
    if (!exists && !readError.isEmpty())
    {
        error = readError;
        return false;
    }
    if (exists)
    {
        ModelWeldingFlowTemplate existing;
        if (!DecodeTemplate(existingJson, existing, error))
        {
            return false;
        }
        if (existing.templateId != value.templateId)
        {
            error = QStringLiteral("模板配置作用域ID与JSON内部ID不一致。");
            return false;
        }
        if (existing.revision != expectedPreviousRevision
            || value.revision != existing.revision + 1)
        {
            error = QStringLiteral("模板已被其他操作更新，修订冲突。");
            return false;
        }
    }
    else if (expectedPreviousRevision != 0 || value.revision != 1)
    {
        error = QStringLiteral("新模板必须从修订1开始。");
        return false;
    }
    const QByteArray json = EncodeTemplate(value, error);
    const QByteArray* expectedJson = exists ? &existingJson : nullptr;
    if (json.isEmpty())
    {
        return false;
    }
    if (!inheritanceSourceJson.isEmpty())
    {
        return CompareAndSwapStoredJsonWithWitness(
            kTemplateScope,
            value.inheritedFromTemplateId,
            inheritanceSourceJson,
            kTemplateScope,
            value.templateId,
            expectedJson,
            json,
            error);
    }
    return CompareAndSwapStoredJson(
        kTemplateScope, value.templateId, expectedJson, json, error);
}

ModelWeldingWorkflow::LoadStatus ModelWeldingWorkflow::LoadTemplate(
    const QString& templateId,
    ModelWeldingFlowTemplate& value,
    QString& error)
{
    QMutexLocker lock(&StoreMutex());
    error.clear();
    if (!IsStableId(templateId))
    {
        error = QStringLiteral("模板ID无效。");
        return LoadStatus::Error;
    }
    QByteArray json;
    if (!ReadStoredJson(kTemplateScope, templateId, json, error))
    {
        return error.isEmpty() ? LoadStatus::NotFound : LoadStatus::Error;
    }
    ModelWeldingFlowTemplate loaded;
    if (!DecodeTemplate(json, loaded, error))
    {
        return LoadStatus::Error;
    }
    if (loaded.templateId != templateId)
    {
        error = QStringLiteral("模板配置作用域ID与JSON内部ID不一致。");
        return LoadStatus::Error;
    }
    value = loaded;
    return LoadStatus::Found;
}

bool ModelWeldingWorkflow::ListTemplates(QVector<ModelWeldingFlowTemplate>& values, QString& error)
{
    QMutexLocker lock(&StoreMutex());
    QStringList ids;
    if (!ConfigDatabase::TryListScopedSettingIdsBounded(
        kTemplateScope, kDefinitionModule, kMaximumTemplateCount, 64, &ids))
    {
        error = QStringLiteral("无法枚举模型流程模板。");
        return false;
    }
    QVector<ModelWeldingFlowTemplate> loaded;
    for (const QString& id : ids)
    {
        QByteArray json;
        if (!ReadStoredJson(kTemplateScope, id, json, error))
        {
            return false;
        }
        ModelWeldingFlowTemplate value;
        if (!DecodeTemplate(json, value, error))
        {
            return false;
        }
        if (value.templateId != id)
        {
            error = QStringLiteral("模板配置作用域ID与JSON内部ID不一致。");
            return false;
        }
        loaded.push_back(value);
    }
    std::sort(loaded.begin(), loaded.end(), [](const auto& left, const auto& right)
        {
            return QString::localeAwareCompare(left.displayName, right.displayName) < 0;
        });
    values = loaded;
    error.clear();
    return true;
}

bool ModelWeldingWorkflow::SaveTeaching(
    const ModelWeldingRobotTeaching& value,
    quint64 expectedPreviousRevision,
    QString& error)
{
    QMutexLocker lock(&StoreMutex());
    if (!ValidateTeachingStructure(value, nullptr, error, false))
    {
        return false;
    }
    if (!IsStableIdentifier(value.robotModelId)
        || !IsSha256(value.sourceStepSha256)
        || !IsSha256(value.collisionProfileSha256))
    {
        error = QStringLiteral(
            "保存机器人示教前必须绑定机器人型号、源STEP和碰撞简模配置身份；"
            "旧版记录只能作为迁移草稿读取。");
        return false;
    }
    QByteArray templateJson;
    if (!ReadStoredJson(kTemplateScope, value.templateId, templateJson, error))
    {
        if (error.isEmpty())
        {
            error = QStringLiteral("机器人示教引用的模型模板不存在。");
        }
        return false;
    }
    ModelWeldingFlowTemplate referencedTemplate;
    if (!DecodeTemplate(templateJson, referencedTemplate, error)
        || !ValidateTeachingStructure(value, &referencedTemplate, error, false))
    {
        return false;
    }
    QByteArray existingJson;
    QString readError;
    const bool exists = ReadStoredJson(kTeachingScope, value.teachingId, existingJson, readError);
    if (!exists && !readError.isEmpty())
    {
        error = readError;
        return false;
    }
    if (exists)
    {
        ModelWeldingRobotTeaching existing;
        if (!DecodeTeaching(existingJson, existing, error))
        {
            return false;
        }
        if (existing.teachingId != value.teachingId)
        {
            error = QStringLiteral("示教配置作用域ID与JSON内部ID不一致。");
            return false;
        }
        const bool existingHasRobotModelIdentity = !existing.robotModelId.isEmpty()
            || !existing.sourceStepSha256.isEmpty()
            || !existing.collisionProfileSha256.isEmpty();
        if (existingHasRobotModelIdentity
            && (existing.robotModelId != value.robotModelId
                || existing.sourceStepSha256 != value.sourceStepSha256
                || existing.collisionProfileSha256 != value.collisionProfileSha256))
        {
            error = QStringLiteral(
                "已保存示教绑定的机器人型号资产身份不可改写，请为新型号重新示教。");
            return false;
        }
        if (existing.revision != expectedPreviousRevision
            || value.revision != existing.revision + 1)
        {
            error = QStringLiteral("示教记录已被其他操作更新，修订冲突。");
            return false;
        }
    }
    else if (expectedPreviousRevision != 0 || value.revision != 1)
    {
        error = QStringLiteral("新示教记录必须从修订1开始。");
        return false;
    }
    const QByteArray json = EncodeTeaching(value, error);
    const QByteArray* expectedJson = exists ? &existingJson : nullptr;
    return !json.isEmpty()
        && CompareAndSwapStoredJsonWithWitness(
            kTemplateScope,
            value.templateId,
            templateJson,
            kTeachingScope,
            value.teachingId,
            expectedJson,
            json,
            error);
}

ModelWeldingWorkflow::LoadStatus ModelWeldingWorkflow::LoadTeaching(
    const QString& teachingId,
    ModelWeldingRobotTeaching& value,
    QString& error)
{
    QMutexLocker lock(&StoreMutex());
    error.clear();
    if (!IsStableId(teachingId))
    {
        error = QStringLiteral("示教记录ID无效。");
        return LoadStatus::Error;
    }
    QByteArray json;
    if (!ReadStoredJson(kTeachingScope, teachingId, json, error))
    {
        return error.isEmpty() ? LoadStatus::NotFound : LoadStatus::Error;
    }
    ModelWeldingRobotTeaching loaded;
    if (!DecodeTeaching(json, loaded, error))
    {
        return LoadStatus::Error;
    }
    if (loaded.teachingId != teachingId)
    {
        error = QStringLiteral("示教配置作用域ID与JSON内部ID不一致。");
        return LoadStatus::Error;
    }
    value = loaded;
    return LoadStatus::Found;
}

bool ModelWeldingWorkflow::ListTeachings(QVector<ModelWeldingRobotTeaching>& values, QString& error)
{
    QMutexLocker lock(&StoreMutex());
    QStringList ids;
    if (!ConfigDatabase::TryListScopedSettingIdsBounded(
        kTeachingScope, kDefinitionModule, kMaximumTemplateCount, 64, &ids))
    {
        error = QStringLiteral("无法枚举机器人示教记录。");
        return false;
    }
    QVector<ModelWeldingRobotTeaching> loaded;
    for (const QString& id : ids)
    {
        QByteArray json;
        if (!ReadStoredJson(kTeachingScope, id, json, error))
        {
            return false;
        }
        ModelWeldingRobotTeaching value;
        if (!DecodeTeaching(json, value, error))
        {
            return false;
        }
        if (value.teachingId != id)
        {
            error = QStringLiteral("示教配置作用域ID与JSON内部ID不一致。");
            return false;
        }
        loaded.push_back(value);
    }
    values = loaded;
    error.clear();
    return true;
}

ModelWeldingRigidFitResult ModelWeldingWorkflow::SolveRigidPointPairs(
    const QVector<ModelWeldingRigidPointPair>& pairs,
    const ModelWeldingRigidFitOptions& options)
{
    ModelWeldingRigidFitResult result;
    const double optionValues[] = {
        options.minimumSecondToFirstSingularRatio,
        options.maximumSolveRmseMm,
        options.maximumSolveResidualMm,
        options.maximumVerifyResidualMm,
        options.maximumPairDistanceErrorMm,
        options.maximumScaleDeviation
    };
    for (double value : optionValues)
    {
        if (!Finite(value) || value < 0.0)
        {
            result.rejectionReason = QStringLiteral("刚体定位质量门限包含无效数值。");
            return result;
        }
    }
    QVector<ModelWeldingRigidPointPair> solvePairs;
    QVector<ModelWeldingRigidPointPair> verifyPairs;
    QMap<QString, bool> ids;
    for (const ModelWeldingRigidPointPair& pair : pairs)
    {
        const QString featureId = pair.featureId.trimmed();
        if (featureId.isEmpty() || ids.contains(featureId)
            || !Finite(pair.modelPointMm) || !Finite(pair.measuredBasePointMm))
        {
            result.rejectionReason = QStringLiteral("定位点编号重复、为空或包含非有限坐标。");
            return result;
        }
        ids.insert(featureId, true);
        (pair.useForSolve ? solvePairs : verifyPairs).push_back(pair);
    }
    if (solvePairs.size() < 3)
    {
        result.rejectionReason = QStringLiteral("刚体定位至少需要3个求解点。");
        return result;
    }

    Eigen::Vector3d sourceCenter = Eigen::Vector3d::Zero();
    Eigen::Vector3d targetCenter = Eigen::Vector3d::Zero();
    for (const ModelWeldingRigidPointPair& pair : solvePairs)
    {
        sourceCenter += pair.modelPointMm;
        targetCenter += pair.measuredBasePointMm;
    }
    sourceCenter /= static_cast<double>(solvePairs.size());
    targetCenter /= static_cast<double>(solvePairs.size());

    Eigen::Matrix3d sourceScatter = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    double sourceSquared = 0.0;
    double targetSquared = 0.0;
    for (const ModelWeldingRigidPointPair& pair : solvePairs)
    {
        const Eigen::Vector3d source = pair.modelPointMm - sourceCenter;
        const Eigen::Vector3d target = pair.measuredBasePointMm - targetCenter;
        sourceScatter += source * source.transpose();
        covariance += source * target.transpose();
        sourceSquared += source.squaredNorm();
        targetSquared += target.squaredNorm();
    }

    const Eigen::JacobiSVD<Eigen::Matrix3d> geometrySvd(sourceScatter);
    result.singularValues = geometrySvd.singularValues();
    if (!result.singularValues.allFinite() || result.singularValues.x() <= 1.0e-12
        || result.singularValues.y() / result.singularValues.x()
            < options.minimumSecondToFirstSingularRatio)
    {
        result.rejectionReason = QStringLiteral("求解点重复、共线或空间分布过差。");
        return result;
    }

    const Eigen::JacobiSVD<Eigen::Matrix3d> svd(
        covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d rotation = svd.matrixV() * svd.matrixU().transpose();
    if (rotation.determinant() < 0.0)
    {
        Eigen::Matrix3d adjustedV = svd.matrixV();
        adjustedV.col(2) *= -1.0;
        rotation = adjustedV * svd.matrixU().transpose();
    }
    const Eigen::Vector3d translation = targetCenter - rotation * sourceCenter;
    if (!IsOrthonormalRightHanded(rotation) || !Finite(translation))
    {
        result.rejectionReason = QStringLiteral("刚体求解产生了无效旋转或平移。");
        return result;
    }

    result.baseFromModel.setIdentity();
    result.baseFromModel.block<3, 3>(0, 0) = rotation;
    result.baseFromModel.block<3, 1>(0, 3) = translation;
    for (const ModelWeldingRigidPointPair& pair : solvePairs)
    {
        result.solveResidualsMm.push_back(
            (rotation * pair.modelPointMm + translation - pair.measuredBasePointMm).norm());
    }
    for (const ModelWeldingRigidPointPair& pair : verifyPairs)
    {
        result.verifyResidualsMm.push_back(
            (rotation * pair.modelPointMm + translation - pair.measuredBasePointMm).norm());
    }
    result.solveRmseMm = Rmse(result.solveResidualsMm);
    result.maximumSolveResidualMm = Maximum(result.solveResidualsMm);
    result.maximumVerifyResidualMm = Maximum(result.verifyResidualsMm);
    result.observedScaleRatio = sourceSquared > 1.0e-12
        ? std::sqrt(targetSquared / sourceSquared)
        : 1.0;

    double pairErrorSum = 0.0;
    int pairErrorCount = 0;
    for (int left = 0; left < solvePairs.size(); ++left)
    {
        for (int right = left + 1; right < solvePairs.size(); ++right)
        {
            const double modelDistance =
                (solvePairs.at(left).modelPointMm - solvePairs.at(right).modelPointMm).norm();
            const double measuredDistance =
                (solvePairs.at(left).measuredBasePointMm - solvePairs.at(right).measuredBasePointMm).norm();
            const double error = std::abs(modelDistance - measuredDistance);
            pairErrorSum += error;
            result.maximumPairDistanceErrorMm =
                std::max(result.maximumPairDistanceErrorMm, error);
            ++pairErrorCount;
        }
    }
    result.meanPairDistanceErrorMm = pairErrorCount > 0
        ? pairErrorSum / static_cast<double>(pairErrorCount)
        : 0.0;
    result.solved = true;

    if (result.solveRmseMm > options.maximumSolveRmseMm)
    {
        result.rejectionReason = QStringLiteral("求解点RMSE超限：%1 mm。").arg(result.solveRmseMm, 0, 'f', 3);
    }
    else if (result.maximumSolveResidualMm > options.maximumSolveResidualMm)
    {
        result.rejectionReason = QStringLiteral("求解点最大残差超限：%1 mm。").arg(result.maximumSolveResidualMm, 0, 'f', 3);
    }
    else if (!verifyPairs.isEmpty()
        && result.maximumVerifyResidualMm > options.maximumVerifyResidualMm)
    {
        result.rejectionReason = QStringLiteral("独立验证点最大残差超限：%1 mm。").arg(result.maximumVerifyResidualMm, 0, 'f', 3);
    }
    else if (result.maximumPairDistanceErrorMm > options.maximumPairDistanceErrorMm)
    {
        result.rejectionReason = QStringLiteral("点对距离一致性超限：%1 mm。").arg(result.maximumPairDistanceErrorMm, 0, 'f', 3);
    }
    else if (std::abs(result.observedScaleRatio - 1.0) > options.maximumScaleDeviation)
    {
        result.rejectionReason = QStringLiteral("检测到比例变化：%1；刚体定位禁止缩放。")
            .arg(result.observedScaleRatio, 0, 'f', 6);
    }
    else
    {
        result.accepted = true;
        result.rejectionReason.clear();
    }
    return result;
}
