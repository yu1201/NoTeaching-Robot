#include "WeldResumePlanner.h"

#include <QByteArray>
#include <QCryptographicHash>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QRegularExpression>
#include <QTextStream>

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
constexpr auto EXPECTED_TRAJECTORY_PREFIX = "PreciseLaserPoint_WeldPose_2mm_SeamComp";
constexpr auto EXPECTED_TRAJECTORY_SUFFIX = "_FinalSampled.txt";
constexpr double MIN_SEGMENT_LENGTH_MM = 1e-9;
constexpr double AMBIGUOUS_DISTANCE_EPSILON_MM = 0.5;
constexpr double SAME_LOCAL_ARC_POSITION_EPSILON_MM = AMBIGUOUS_DISTANCE_EPSILON_MM;

void SetError(QString* error, const QString& value)
{
    if (error != nullptr)
    {
        *error = value;
    }
}

bool IsFinite(double value)
{
    return std::isfinite(value);
}

bool PathEquals(const QString& left, const QString& right)
{
#ifdef Q_OS_WIN
    return left.compare(right, Qt::CaseInsensitive) == 0;
#else
    return left == right;
#endif
}

QString CleanAbsolutePath(const QString& path)
{
    QFileInfo info(path);
    const QString canonical = info.canonicalFilePath();
    return QDir::cleanPath(QDir::fromNativeSeparators(
        canonical.isEmpty() ? info.absoluteFilePath() : canonical));
}

bool IsSafeRelativePath(const QString& path)
{
    const QString normalized = QDir::cleanPath(QDir::fromNativeSeparators(path.trimmed()));
    return !normalized.isEmpty()
        && normalized != QStringLiteral(".")
        && !QDir::isAbsolutePath(normalized)
        && normalized != QStringLiteral("..")
        && !normalized.startsWith(QStringLiteral("../"));
}

double PointDistance(
    const WeldResumePlanner::TrajectoryPoint& left,
    const WeldResumePlanner::TrajectoryPoint& right)
{
    const double dx = left.x - right.x;
    const double dy = left.y - right.y;
    const double dz = left.z - right.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

QJsonObject RecordToJson(const WeldResumePlanner::CheckpointRecord& record)
{
    QJsonObject root;
    root.insert(QStringLiteral("schemaVersion"), record.schemaVersion);
    root.insert(QStringLiteral("state"), record.state);
    root.insert(QStringLiteral("checkpointId"), record.checkpointId);
    root.insert(QStringLiteral("createdAtUtc"), record.createdAtUtc);

    QJsonObject robot;
    robot.insert(QStringLiteral("name"), record.robotName);
    robot.insert(QStringLiteral("type"), record.robotType);
    robot.insert(QStringLiteral("endpoint"), record.robotEndpoint);
    root.insert(QStringLiteral("robot"), robot);

    QJsonObject parameters;
    parameters.insert(QStringLiteral("groupIndex"), record.paramGroupIndex);
    parameters.insert(QStringLiteral("groupName"), record.paramGroupName);
    parameters.insert(QStringLiteral("scanSection"), record.scanSection);
    parameters.insert(QStringLiteral("weldSection"), record.weldSection);
    parameters.insert(QStringLiteral("fingerprint"), record.parameterFingerprint);
    parameters.insert(QStringLiteral("weldDirection"), record.weldDirection);
    parameters.insert(QStringLiteral("actualWeld"), record.actualWeld);
    parameters.insert(QStringLiteral("finalStepMm"), record.finalStepMm);
    parameters.insert(QStringLiteral("backtrackMm"), record.backtrackMm);
    root.insert(QStringLiteral("parameters"), parameters);

    QJsonObject runCase;
    runCase.insert(QStringLiteral("caseId"), record.caseId);
    runCase.insert(QStringLiteral("relativeDir"), record.caseRelativeDir);
    root.insert(QStringLiteral("case"), runCase);

    QJsonObject trajectory;
    trajectory.insert(QStringLiteral("sourceRelativePath"), record.sourceTrajectoryRelativePath);
    trajectory.insert(QStringLiteral("sourceSha256"), record.sourceTrajectorySha256);
    trajectory.insert(QStringLiteral("relativePath"), record.trajectoryRelativePath);
    trajectory.insert(QStringLiteral("sha256"), record.trajectorySha256);
    trajectory.insert(QStringLiteral("size"), static_cast<double>(record.trajectorySize));
    trajectory.insert(QStringLiteral("pointCount"), record.trajectoryPointCount);
    trajectory.insert(QStringLiteral("executionOrder"), record.trajectoryInExecutionOrder);
    root.insert(QStringLiteral("trajectory"), trajectory);

    QJsonObject program;
    program.insert(QStringLiteral("name"), record.programName);
    program.insert(QStringLiteral("line"), record.programLine);
    root.insert(QStringLiteral("program"), program);

    QJsonObject pose;
    pose.insert(QStringLiteral("x"), record.x);
    pose.insert(QStringLiteral("y"), record.y);
    pose.insert(QStringLiteral("z"), record.z);
    pose.insert(QStringLiteral("rx"), record.rx);
    pose.insert(QStringLiteral("ry"), record.ry);
    pose.insert(QStringLiteral("rz"), record.rz);
    root.insert(QStringLiteral("pose"), pose);
    return root;
}

bool JsonString(const QJsonObject& object, const QString& key, QString& value)
{
    const QJsonValue jsonValue = object.value(key);
    if (!jsonValue.isString())
    {
        return false;
    }
    value = jsonValue.toString();
    return true;
}

bool JsonInt(const QJsonObject& object, const QString& key, int& value)
{
    const QJsonValue jsonValue = object.value(key);
    if (!jsonValue.isDouble())
    {
        return false;
    }
    const double number = jsonValue.toDouble(std::numeric_limits<double>::quiet_NaN());
    if (!IsFinite(number)
        || number < static_cast<double>(std::numeric_limits<int>::min())
        || number > static_cast<double>(std::numeric_limits<int>::max())
        || std::floor(number) != number)
    {
        return false;
    }
    value = static_cast<int>(number);
    return true;
}

bool JsonDouble(const QJsonObject& object, const QString& key, double& value)
{
    const QJsonValue jsonValue = object.value(key);
    if (!jsonValue.isDouble())
    {
        return false;
    }
    value = jsonValue.toDouble(std::numeric_limits<double>::quiet_NaN());
    return IsFinite(value);
}

bool JsonBool(const QJsonObject& object, const QString& key, bool& value)
{
    const QJsonValue jsonValue = object.value(key);
    if (!jsonValue.isBool())
    {
        return false;
    }
    value = jsonValue.toBool();
    return true;
}
}

QString WeldResumePlanner::EncodeRecord(const CheckpointRecord& record, QString* error)
{
    if (record.schemaVersion != SchemaVersion
        || record.checkpointId.trimmed().isEmpty()
        || record.robotName.trimmed().isEmpty())
    {
        SetError(error, QStringLiteral("断点V2记录缺少版本、记录ID或机器人身份。"));
        return QString();
    }

    const QByteArray json = QJsonDocument(RecordToJson(record)).toJson(QJsonDocument::Compact);
    return QStringLiteral("b64:v2:") + QString::fromLatin1(
        json.toBase64(QByteArray::Base64UrlEncoding | QByteArray::OmitTrailingEquals));
}

bool WeldResumePlanner::DecodeRecord(const QString& encoded, CheckpointRecord& record, QString* error)
{
    record = CheckpointRecord();
    const QString prefix = QStringLiteral("b64:v2:");
    if (!encoded.startsWith(prefix))
    {
        SetError(error, QStringLiteral("不存在可验证的V2断点记录；旧版断点禁止自动匹配最新案例。"));
        return false;
    }

    const QByteArray jsonBytes = QByteArray::fromBase64(
        encoded.mid(prefix.size()).toLatin1(), QByteArray::Base64UrlEncoding);
    QJsonParseError parseError;
    const QJsonDocument document = QJsonDocument::fromJson(jsonBytes, &parseError);
    if (parseError.error != QJsonParseError::NoError || !document.isObject())
    {
        SetError(error, QString("断点V2记录JSON损坏：%1").arg(parseError.errorString()));
        return false;
    }

    const QJsonObject root = document.object();
    const QJsonObject robot = root.value(QStringLiteral("robot")).toObject();
    const QJsonObject parameters = root.value(QStringLiteral("parameters")).toObject();
    const QJsonObject runCase = root.value(QStringLiteral("case")).toObject();
    const QJsonObject trajectory = root.value(QStringLiteral("trajectory")).toObject();
    const QJsonObject program = root.value(QStringLiteral("program")).toObject();
    const QJsonObject pose = root.value(QStringLiteral("pose")).toObject();

    double trajectorySize = -1.0;
    if (!JsonInt(root, QStringLiteral("schemaVersion"), record.schemaVersion)
        || !JsonString(root, QStringLiteral("state"), record.state)
        || !JsonString(root, QStringLiteral("checkpointId"), record.checkpointId)
        || !JsonString(root, QStringLiteral("createdAtUtc"), record.createdAtUtc)
        || !JsonString(robot, QStringLiteral("name"), record.robotName)
        || !JsonString(robot, QStringLiteral("type"), record.robotType)
        || !JsonString(robot, QStringLiteral("endpoint"), record.robotEndpoint)
        || !JsonInt(parameters, QStringLiteral("groupIndex"), record.paramGroupIndex)
        || !JsonString(parameters, QStringLiteral("groupName"), record.paramGroupName)
        || !JsonString(parameters, QStringLiteral("scanSection"), record.scanSection)
        || !JsonString(parameters, QStringLiteral("weldSection"), record.weldSection)
        || !JsonString(parameters, QStringLiteral("fingerprint"), record.parameterFingerprint)
        || !JsonInt(parameters, QStringLiteral("weldDirection"), record.weldDirection)
        || !JsonBool(parameters, QStringLiteral("actualWeld"), record.actualWeld)
        || !JsonDouble(parameters, QStringLiteral("finalStepMm"), record.finalStepMm)
        || !JsonDouble(parameters, QStringLiteral("backtrackMm"), record.backtrackMm)
        || !JsonString(runCase, QStringLiteral("caseId"), record.caseId)
        || !JsonString(runCase, QStringLiteral("relativeDir"), record.caseRelativeDir)
        || !JsonString(trajectory, QStringLiteral("sourceRelativePath"), record.sourceTrajectoryRelativePath)
        || !JsonString(trajectory, QStringLiteral("sourceSha256"), record.sourceTrajectorySha256)
        || !JsonString(trajectory, QStringLiteral("relativePath"), record.trajectoryRelativePath)
        || !JsonString(trajectory, QStringLiteral("sha256"), record.trajectorySha256)
        || !JsonDouble(trajectory, QStringLiteral("size"), trajectorySize)
        || !JsonInt(trajectory, QStringLiteral("pointCount"), record.trajectoryPointCount)
        || !JsonBool(trajectory, QStringLiteral("executionOrder"), record.trajectoryInExecutionOrder)
        || !JsonString(program, QStringLiteral("name"), record.programName)
        || !JsonInt(program, QStringLiteral("line"), record.programLine)
        || !JsonDouble(pose, QStringLiteral("x"), record.x)
        || !JsonDouble(pose, QStringLiteral("y"), record.y)
        || !JsonDouble(pose, QStringLiteral("z"), record.z)
        || !JsonDouble(pose, QStringLiteral("rx"), record.rx)
        || !JsonDouble(pose, QStringLiteral("ry"), record.ry)
        || !JsonDouble(pose, QStringLiteral("rz"), record.rz))
    {
        SetError(error, QStringLiteral("断点V2记录缺少必需字段或字段类型错误。"));
        return false;
    }

    static const QRegularExpression sha256Pattern(QStringLiteral("^[0-9a-fA-F]{64}$"));
    const bool knownState = record.state == QStringLiteral("prepared")
        || record.state == QStringLiteral("writing")
        || record.state == QStringLiteral("paused")
        || record.state == QStringLiteral("continuing")
        || record.state == QStringLiteral("resuming")
        || record.state == QStringLiteral("superseded")
        || record.state == QStringLiteral("finished");
    const bool sourceIdentityConsistent =
        (record.sourceTrajectoryRelativePath.isEmpty() && record.sourceTrajectorySha256.isEmpty())
        || (IsSafeRelativePath(record.sourceTrajectoryRelativePath)
            && sha256Pattern.match(record.sourceTrajectorySha256).hasMatch());
    if (record.schemaVersion != SchemaVersion
        || trajectorySize < 0.0
        || trajectorySize > static_cast<double>(std::numeric_limits<qint64>::max())
        || !knownState
        || record.checkpointId.trimmed().isEmpty()
        || record.createdAtUtc.trimmed().isEmpty()
        || record.robotName.trimmed().isEmpty()
        || record.robotType.trimmed().isEmpty()
        || record.robotEndpoint.trimmed().isEmpty()
        || record.paramGroupIndex < 0
        || record.paramGroupName.trimmed().isEmpty()
        || record.scanSection.trimmed().isEmpty()
        || record.weldSection.trimmed().isEmpty()
        || record.caseId.trimmed().isEmpty()
        || !IsSafeRelativePath(record.caseRelativeDir)
        || !IsSafeRelativePath(record.trajectoryRelativePath)
        || !sha256Pattern.match(record.trajectorySha256).hasMatch()
        || !sha256Pattern.match(record.parameterFingerprint).hasMatch()
        || !sourceIdentityConsistent
        || record.trajectoryPointCount < 2
        || !record.trajectoryInExecutionOrder
        || record.programName.trimmed().isEmpty()
        || (record.weldDirection != 1 && record.weldDirection != -1)
        || record.finalStepMm <= 0.0
        || record.backtrackMm < 0.0
        || record.backtrackMm > 1000.0)
    {
        SetError(error, QStringLiteral("断点V2记录版本或安全约束无效。"));
        return false;
    }
    record.trajectorySize = static_cast<qint64>(trajectorySize);
    return true;
}

QString WeldResumePlanner::ComputeFileSha256(const QString& filePath, QString* error)
{
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly))
    {
        SetError(error, QString("无法读取轨迹文件以计算SHA256：%1").arg(filePath));
        return QString();
    }

    QCryptographicHash hash(QCryptographicHash::Sha256);
    if (!hash.addData(&file))
    {
        SetError(error, QString("读取轨迹文件计算SHA256失败：%1").arg(filePath));
        return QString();
    }
    return QString::fromLatin1(hash.result().toHex()).toLower();
}

QString WeldResumePlanner::BuildParameterFingerprint(const QStringList& orderedFields)
{
    QJsonArray values;
    for (const QString& field : orderedFields)
    {
        values.append(field);
    }
    return QString::fromLatin1(QCryptographicHash::hash(
        QJsonDocument(values).toJson(QJsonDocument::Compact),
        QCryptographicHash::Sha256).toHex()).toLower();
}

bool WeldResumePlanner::BindTrajectoryIdentity(
    const QString& projectRoot,
    const QString& trajectoryPath,
    const QString& robotName,
    CheckpointRecord& record,
    QString* error)
{
    const QString root = CleanAbsolutePath(projectRoot);
    const QString absoluteTrajectory = CleanAbsolutePath(trajectoryPath);
    const QFileInfo trajectoryInfo(absoluteTrajectory);
    if (!trajectoryInfo.isFile())
    {
        SetError(error, QString("实际执行轨迹不存在：%1").arg(trajectoryPath));
        return false;
    }

    const QString relative = QDir::cleanPath(QDir::fromNativeSeparators(
        QDir(root).relativeFilePath(absoluteTrajectory)));
    if (!IsSafeRelativePath(relative))
    {
        SetError(error, QString("实际执行轨迹不在工程目录内：%1").arg(absoluteTrajectory));
        return false;
    }

    const QStringList parts = relative.split(QLatin1Char('/'), Qt::SkipEmptyParts);
    if (parts.size() != 5
        || parts[0].compare(QStringLiteral("Result"), Qt::CaseInsensitive) != 0
        || parts[1].compare(robotName, Qt::CaseInsensitive) != 0
        || parts[3].compare(QStringLiteral("LaserPoint"), Qt::CaseInsensitive) != 0
        || !parts[4].startsWith(QString::fromLatin1(EXPECTED_TRAJECTORY_PREFIX), Qt::CaseInsensitive)
        || !parts[4].endsWith(QString::fromLatin1(EXPECTED_TRAJECTORY_SUFFIX), Qt::CaseInsensitive))
    {
        SetError(error, QString("实际执行轨迹不符合 Result/<robot>/<case>/LaserPoint/FinalSampled 结构：%1")
            .arg(relative));
        return false;
    }

    QString hashError;
    const QString sha256 = ComputeFileSha256(absoluteTrajectory, &hashError);
    if (sha256.isEmpty())
    {
        SetError(error, hashError);
        return false;
    }

    QVector<TrajectoryPoint> points;
    QString pointError;
    if (!LoadExecutionTrajectory(absoluteTrajectory, points, &pointError))
    {
        SetError(error, pointError);
        return false;
    }

    record.robotName = robotName;
    record.caseId = parts[2];
    record.caseRelativeDir = parts.mid(0, 3).join(QLatin1Char('/'));
    record.trajectoryRelativePath = relative;
    record.trajectorySha256 = sha256;
    record.trajectorySize = trajectoryInfo.size();
    record.trajectoryPointCount = points.size();
    record.trajectoryInExecutionOrder = true;
    return true;
}

bool WeldResumePlanner::ResolveBoundTrajectory(
    const QString& projectRoot,
    const QString& expectedRobotName,
    const CheckpointRecord& record,
    QString& trajectoryPath,
    QString* error)
{
    trajectoryPath.clear();
    if (record.robotName.compare(expectedRobotName, Qt::CaseInsensitive) != 0
        || !IsSafeRelativePath(record.trajectoryRelativePath)
        || !IsSafeRelativePath(record.caseRelativeDir))
    {
        SetError(error, QStringLiteral("断点机器人或案例相对路径与当前上下文不一致。"));
        return false;
    }

    const QString candidate = QDir(projectRoot).filePath(record.trajectoryRelativePath);
    CheckpointRecord current;
    QString bindError;
    if (!BindTrajectoryIdentity(projectRoot, candidate, expectedRobotName, current, &bindError))
    {
        SetError(error, bindError);
        return false;
    }

    if (!PathEquals(current.caseId, record.caseId)
        || !PathEquals(current.caseRelativeDir, record.caseRelativeDir)
        || !PathEquals(current.trajectoryRelativePath, record.trajectoryRelativePath)
        || current.trajectorySha256.compare(record.trajectorySha256, Qt::CaseInsensitive) != 0
        || current.trajectorySize != record.trajectorySize
        || current.trajectoryPointCount != record.trajectoryPointCount)
    {
        SetError(error, QStringLiteral("断点绑定的案例、轨迹大小、点数或SHA256已变化，拒绝续焊。"));
        return false;
    }

    trajectoryPath = CleanAbsolutePath(candidate);
    return true;
}

bool WeldResumePlanner::LoadExecutionTrajectory(
    const QString& trajectoryPath,
    QVector<TrajectoryPoint>& points,
    QString* error)
{
    points.clear();
    QFile file(trajectoryPath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        SetError(error, QString("无法打开实际执行轨迹：%1").arg(trajectoryPath));
        return false;
    }

    QTextStream stream(&file);
    static const QRegularExpression separator(QStringLiteral("[\\s,]+"));
    int lineNumber = 0;
    while (!stream.atEnd())
    {
        ++lineNumber;
        const QString line = stream.readLine().trimmed();
        if (line.isEmpty() || line.startsWith(QLatin1Char('#')))
        {
            continue;
        }

        const QStringList parts = line.split(separator, Qt::SkipEmptyParts);
        if (parts.size() >= 5
            && parts[0].compare(QStringLiteral("weld_index"), Qt::CaseInsensitive) == 0)
        {
            continue;
        }
        if (parts.size() < 5)
        {
            SetError(error, QString("实际执行轨迹第%1行字段不足，期望 weldIndex rawIndex X Y Z ...")
                .arg(lineNumber));
            return false;
        }

        bool weldIndexOk = false;
        bool rawIndexOk = false;
        bool xOk = false;
        bool yOk = false;
        bool zOk = false;
        parts[0].toInt(&weldIndexOk);
        parts[1].toInt(&rawIndexOk);
        TrajectoryPoint point;
        point.x = parts[2].toDouble(&xOk);
        point.y = parts[3].toDouble(&yOk);
        point.z = parts[4].toDouble(&zOk);
        if (!weldIndexOk || !rawIndexOk || !xOk || !yOk || !zOk
            || !IsFinite(point.x) || !IsFinite(point.y) || !IsFinite(point.z))
        {
            SetError(error, QString("实际执行轨迹第%1行索引或XYZ无效。").arg(lineNumber));
            return false;
        }
        points.push_back(point);
    }

    if (points.size() < 2)
    {
        SetError(error, QString("实际执行轨迹有效点不足：%1").arg(points.size()));
        points.clear();
        return false;
    }
    return true;
}

bool WeldResumePlanner::PlanFromPausedPose(
    const QString& trajectoryPath,
    double pauseX,
    double pauseY,
    double pauseZ,
    double backtrackMm,
    ResumePlan& plan,
    QString* error)
{
    plan = ResumePlan();
    if (!IsFinite(pauseX) || !IsFinite(pauseY) || !IsFinite(pauseZ)
        || !IsFinite(backtrackMm) || backtrackMm < 0.0)
    {
        SetError(error, QStringLiteral("断点位姿或毫米回退距离无效。"));
        return false;
    }
    if (!LoadExecutionTrajectory(trajectoryPath, plan.points, error))
    {
        return false;
    }
    plan.sourcePointCount = plan.points.size();

    struct Candidate
    {
        int segment = -1;
        double ratio = 0.0;
        double distance = std::numeric_limits<double>::infinity();
        double arc = 0.0;
    };
    QVector<Candidate> candidates;
    double cumulativeArc = 0.0;
    for (int index = 0; index + 1 < plan.points.size(); ++index)
    {
        const TrajectoryPoint& begin = plan.points[index];
        const TrajectoryPoint& end = plan.points[index + 1];
        const double dx = end.x - begin.x;
        const double dy = end.y - begin.y;
        const double dz = end.z - begin.z;
        const double lengthSquared = dx * dx + dy * dy + dz * dz;
        const double length = std::sqrt(lengthSquared);
        if (length <= MIN_SEGMENT_LENGTH_MM)
        {
            continue;
        }

        const double projection = std::clamp(
            ((pauseX - begin.x) * dx + (pauseY - begin.y) * dy + (pauseZ - begin.z) * dz)
                / lengthSquared,
            0.0,
            1.0);
        TrajectoryPoint projected;
        projected.x = begin.x + dx * projection;
        projected.y = begin.y + dy * projection;
        projected.z = begin.z + dz * projection;
        TrajectoryPoint pause;
        pause.x = pauseX;
        pause.y = pauseY;
        pause.z = pauseZ;

        Candidate candidate;
        candidate.segment = index;
        candidate.ratio = projection;
        candidate.distance = PointDistance(projected, pause);
        candidate.arc = cumulativeArc + length * projection;
        candidates.push_back(candidate);
        cumulativeArc += length;
    }
    plan.totalArcMm = cumulativeArc;
    if (candidates.isEmpty() || cumulativeArc <= MIN_SEGMENT_LENGTH_MM)
    {
        SetError(error, QStringLiteral("实际执行轨迹没有有效非零长度线段。"));
        return false;
    }

    std::sort(candidates.begin(), candidates.end(), [](const Candidate& left, const Candidate& right)
        {
            return left.distance < right.distance;
        });
    const Candidate best = candidates.front();
    for (int index = 1; index < candidates.size(); ++index)
    {
        const Candidate& other = candidates[index];
        if (other.distance > best.distance + AMBIGUOUS_DISTANCE_EPSILON_MM)
        {
            break;
        }
        if (std::abs(other.arc - best.arc) > SAME_LOCAL_ARC_POSITION_EPSILON_MM)
        {
            SetError(error, QString("断点位姿在轨迹上存在多个近似候选（弧长%1/%2mm，距离%3/%4mm），拒绝猜测。")
                .arg(best.arc, 0, 'f', 3)
                .arg(other.arc, 0, 'f', 3)
                .arg(best.distance, 0, 'f', 3)
                .arg(other.distance, 0, 'f', 3));
            return false;
        }
    }

    plan.matchedSegmentIndex = best.segment;
    plan.matchedSegmentRatio = best.ratio;
    plan.matchDistanceMm = best.distance;
    plan.pauseArcMm = best.arc;
    plan.resumeArcMm = std::max(0.0, best.arc - backtrackMm);
    plan.actualBacktrackMm = best.arc - plan.resumeArcMm;
    return true;
}
