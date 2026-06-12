#include "WeldPoseAverageUpdater.h"

#ifndef NOMINMAX
#define NOMINMAX
#endif

#include "ConfigDatabase.h"
#include "OPini.h"
#include "RobotDataHelper.h"
#include "RobotMessage.h"

#include <QDateTime>
#include <QDir>
#include <QDirIterator>
#include <QFile>
#include <QFileInfo>
#include <QRegularExpression>
#include <QSet>
#include <QTextStream>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace
{
constexpr auto WELD_POSE_FILE_NAME = "PreciseLaserPoint_WeldPose_2mm.txt";
constexpr auto REPORT_FILE_NAME = "PreciseLaserPoint_WeldPose_2mm_PoseAverageReport.txt";
constexpr double DEFAULT_POSE_MATCH_MAX_ERROR_DEG = 5.0;
constexpr double MIN_OUTLIER_THRESHOLD_DEG = 2.0;

struct SegmentDef
{
    const char* displayName;
    const char* segmentKind;
};

const SegmentDef kDefaultSegments[] = {
    { "姿态0 / 低平台", "low_platform" },
    { "姿态1 / 上升边", "rising_edge" },
    { "姿态2 / 高平台", "high_platform" },
    { "姿态3 / 下降边", "falling_edge" }
};
constexpr int kDefaultSegmentCount = sizeof(kDefaultSegments) / sizeof(kDefaultSegments[0]);

struct WeldPoseRecord
{
    double rx = 0.0;
    double ry = 0.0;
    double rz = 0.0;
    double bx = 0.0;
    double by = 0.0;
    double bz = 0.0;
    QString pointType;
    QString segmentKind;
};

struct PoseSample
{
    double rx = 0.0;
    double ry = 0.0;
    double rz = 0.0;
};

struct PoseLibrarySlot
{
    QString name;
    QString segmentKind;
    double rx = 0.0;
    double ry = 30.0;
    double rz = 180.0;
    double compX = 0.0;
    double compY = 0.0;
    double compZ = 0.0;
};

struct PoseLibrary
{
    QVector<PoseLibrarySlot> poseSlots;
    double matchMaxErrorDeg = DEFAULT_POSE_MATCH_MAX_ERROR_DEG;
};

QString LocalPath(const QString& path)
{
    return QDir::toNativeSeparators(QFileInfo(path).absoluteFilePath());
}

std::string LocalString(const QString& text)
{
    const QByteArray bytes = text.toUtf8();
    return std::string(bytes.constData(), static_cast<size_t>(bytes.size()));
}

double NormalizeAngleDeg(double angle)
{
    while (angle > 180.0)
    {
        angle -= 360.0;
    }
    while (angle <= -180.0)
    {
        angle += 360.0;
    }
    return angle;
}

double NormalizeAngleNear(double angle, double reference)
{
    double normalized = NormalizeAngleDeg(angle);
    const double normalizedReference = NormalizeAngleDeg(reference);
    while (normalized - normalizedReference > 180.0)
    {
        normalized -= 360.0;
    }
    while (normalized - normalizedReference < -180.0)
    {
        normalized += 360.0;
    }
    return normalized;
}

double AngleDistanceDeg(double left, double right)
{
    return std::abs(NormalizeAngleNear(left, right) - NormalizeAngleDeg(right));
}

double PoseDistanceDeg(
    double leftRx,
    double leftRy,
    double leftRz,
    double rightRx,
    double rightRy,
    double rightRz)
{
    const double dRx = leftRx - rightRx;
    const double dRy = leftRy - rightRy;
    const double dRz = AngleDistanceDeg(leftRz, rightRz);
    return std::sqrt(dRx * dRx + dRy * dRy + dRz * dRz);
}

double Median(QVector<double> values)
{
    if (values.isEmpty())
    {
        return 0.0;
    }

    std::sort(values.begin(), values.end());
    const int middle = values.size() / 2;
    if ((values.size() % 2) == 1)
    {
        return values[middle];
    }
    return (values[middle - 1] + values[middle]) * 0.5;
}

bool ResolveAveragePose(
    const QVector<PoseSample>& samples,
    double& rx,
    double& ry,
    double& rz)
{
    if (samples.isEmpty())
    {
        return false;
    }

    const double rzReference = samples.front().rz;
    double rxTotal = 0.0;
    double ryTotal = 0.0;
    double rzTotal = 0.0;
    for (const PoseSample& sample : samples)
    {
        rxTotal += sample.rx;
        ryTotal += sample.ry;
        rzTotal += NormalizeAngleNear(sample.rz, rzReference);
    }

    const double count = static_cast<double>(samples.size());
    rx = rxTotal / count;
    ry = ryTotal / count;
    rz = NormalizeAngleDeg(rzTotal / count);
    return true;
}

bool IsDefaultSegmentKind(const QString& segmentKind)
{
    for (const SegmentDef& def : kDefaultSegments)
    {
        if (segmentKind.compare(def.segmentKind, Qt::CaseInsensitive) == 0)
        {
            return true;
        }
    }
    return false;
}

QString InferRobotNameFromResultPath(const QString& inputFilePath)
{
    const QString normalizedPath = QDir::fromNativeSeparators(
        QFileInfo(inputFilePath).absoluteFilePath());
    const QStringList pathParts = normalizedPath.split('/', Qt::SkipEmptyParts);
    for (int index = 0; index + 1 < pathParts.size(); ++index)
    {
        if (pathParts[index].compare("Result", Qt::CaseInsensitive) == 0)
        {
            const QString robotName = pathParts[index + 1].trimmed();
            if (!robotName.isEmpty())
            {
                return robotName;
            }
        }
    }
    return "RobotA";
}

QString ResolvePoseFilePath(
    const QString& inputPathOrResultDir,
    QStringList& warnings,
    QString& error)
{
    QString normalizedInputPath = QDir::fromNativeSeparators(inputPathOrResultDir.trimmed());
    if (normalizedInputPath.isEmpty())
    {
        error = "输入路径为空。";
        return QString();
    }

    QFileInfo inputInfo(normalizedInputPath);
    if (!inputInfo.isAbsolute())
    {
        inputInfo = QFileInfo(QDir::current().filePath(normalizedInputPath));
    }
    if (!inputInfo.exists())
    {
        error = "输入路径不存在：" + LocalPath(inputInfo.absoluteFilePath());
        return QString();
    }
    if (inputInfo.isFile())
    {
        return inputInfo.absoluteFilePath();
    }

    QDir dir(inputInfo.absoluteFilePath());
    const QStringList directCandidates = {
        dir.filePath(WELD_POSE_FILE_NAME),
        dir.filePath(QString("LaserPoint/%1").arg(WELD_POSE_FILE_NAME))
    };
    for (const QString& candidate : directCandidates)
    {
        if (QFileInfo::exists(candidate))
        {
            return QFileInfo(candidate).absoluteFilePath();
        }
    }

    QVector<QFileInfo> matches;
    QDirIterator it(
        dir.absolutePath(),
        QStringList() << WELD_POSE_FILE_NAME,
        QDir::Files,
        QDirIterator::Subdirectories);
    while (it.hasNext())
    {
        matches.push_back(QFileInfo(it.next()));
    }

    if (matches.isEmpty())
    {
        error = QString("结果目录中未找到 %1：%2")
            .arg(WELD_POSE_FILE_NAME, LocalPath(dir.absolutePath()));
        return QString();
    }

    std::sort(matches.begin(), matches.end(), [](const QFileInfo& left, const QFileInfo& right)
        {
            return left.lastModified() > right.lastModified();
        });
    if (matches.size() > 1)
    {
        warnings << QString("输入目录下找到 %1 个姿态文件，已使用修改时间最新的文件。").arg(matches.size());
    }
    return matches.front().absoluteFilePath();
}

bool TryParseWeldPoseRecord(const QString& line, WeldPoseRecord& record)
{
    QString normalizedLine = line;
    normalizedLine.remove('"');
    const QStringList parts = normalizedLine.contains(',')
        ? normalizedLine.split(',', Qt::SkipEmptyParts)
        : normalizedLine.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
    if (parts.size() < 13)
    {
        return false;
    }

    bool valuesOk[11] = {};
    parts[0].trimmed().toInt(&valuesOk[0]);
    parts[1].trimmed().toInt(&valuesOk[1]);
    parts[2].trimmed().toDouble(&valuesOk[2]);
    parts[3].trimmed().toDouble(&valuesOk[3]);
    parts[4].trimmed().toDouble(&valuesOk[4]);
    record.rx = parts[5].trimmed().toDouble(&valuesOk[5]);
    record.ry = parts[6].trimmed().toDouble(&valuesOk[6]);
    record.rz = parts[7].trimmed().toDouble(&valuesOk[7]);
    record.bx = parts[8].trimmed().toDouble(&valuesOk[8]);
    record.by = parts[9].trimmed().toDouble(&valuesOk[9]);
    record.bz = parts[10].trimmed().toDouble(&valuesOk[10]);

    for (bool ok : valuesOk)
    {
        if (!ok)
        {
            return false;
        }
    }

    record.pointType = parts[11].trimmed();
    record.segmentKind = parts[12].trimmed();
    return std::isfinite(record.rx)
        && std::isfinite(record.ry)
        && std::isfinite(record.rz)
        && std::isfinite(record.bx)
        && std::isfinite(record.by)
        && std::isfinite(record.bz);
}

bool LoadWeldPoseRecords(
    const QString& filePath,
    QVector<WeldPoseRecord>& records,
    QString& error)
{
    records.clear();
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        error = "打开焊道姿态文件失败：" + LocalPath(filePath);
        return false;
    }

    QTextStream stream(&file);
    int lineNumber = 0;
    while (!stream.atEnd())
    {
        const QString line = stream.readLine().trimmed();
        ++lineNumber;
        if (line.isEmpty() || line.startsWith('#'))
        {
            continue;
        }

        WeldPoseRecord record;
        if (!TryParseWeldPoseRecord(line, record))
        {
            if (records.isEmpty())
            {
                continue;
            }

            error = QString("解析焊道姿态文件失败，第 %1 行格式无效：%2")
                .arg(lineNumber)
                .arg(line);
            return false;
        }
        records.push_back(record);
    }

    if (records.isEmpty())
    {
        error = "焊道姿态文件中没有有效点：" + LocalPath(filePath);
        return false;
    }
    return true;
}

void InitializeDefaultPoseSlots(QVector<PoseLibrarySlot>& poseSlots)
{
    for (int index = 0; index < poseSlots.size() && index < kDefaultSegmentCount; ++index)
    {
        poseSlots[index].name = QString::fromUtf8(kDefaultSegments[index].displayName);
        poseSlots[index].segmentKind = QString::fromLatin1(kDefaultSegments[index].segmentKind);
        poseSlots[index].rx = 0.0;
        poseSlots[index].ry = 30.0;
        poseSlots[index].rz = 180.0;
    }
}

bool ReadIniDouble(COPini& ini, const std::string& key, double& value)
{
    return ini.ReadString(false, key, &value) > 0;
}

PoseLibrary LoadPoseLibrary(const QString& path)
{
    PoseLibrary library;
    int poseCompCount = kDefaultSegmentCount;
    if (!ConfigDatabase::HasIniFile(path))
    {
        library.poseSlots.resize(poseCompCount);
        InitializeDefaultPoseSlots(library.poseSlots);
        return library;
    }

    COPini ini;
    ini.SetFileName(LocalString(path));
    ini.SetSectionName("ALLWeldPoseComp");
    ini.ReadString(false, "PoseCompCount", &poseCompCount);
    ReadIniDouble(ini, "PoseMatchMaxErrorDeg", library.matchMaxErrorDeg);
    library.matchMaxErrorDeg = std::max(0.0, library.matchMaxErrorDeg);

    library.poseSlots.resize(std::max(0, poseCompCount));
    InitializeDefaultPoseSlots(library.poseSlots);
    for (int index = 0; index < library.poseSlots.size(); ++index)
    {
        PoseLibrarySlot& slot = library.poseSlots[index];
        ini.SetSectionName(LocalString(QString("WeldPoseComp%1").arg(index)));

        std::string text;
        if (ini.ReadString(false, "Name", text) > 0)
        {
            slot.name = DecodeRobotMessageText(text);
        }
        if (ini.ReadString(false, "SegmentKind", text) > 0)
        {
            slot.segmentKind = DecodeRobotMessageText(text);
        }
        ReadIniDouble(ini, "Rx", slot.rx);
        ReadIniDouble(ini, "Ry", slot.ry);
        ReadIniDouble(ini, "Rz", slot.rz);
        ReadIniDouble(ini, "CompX", slot.compX);
        ReadIniDouble(ini, "CompY", slot.compY);
        ReadIniDouble(ini, "CompZ", slot.compZ);
        slot.rz = NormalizeAngleDeg(slot.rz);
    }

    return library;
}

bool SavePoseLibrary(const QString& path, const PoseLibrary& library, QString& error)
{
    COPini ini;
    ini.SetFileName(false, LocalString(path));
    ini.SetSectionName("ALLWeldPoseComp");
    if (!ini.WriteString("PoseCompCount", static_cast<int>(library.poseSlots.size()))
        || !ini.WriteString("PoseMatchMaxErrorDeg", library.matchMaxErrorDeg, 6))
    {
        error = "写入姿态补偿库总配置失败：" + LocalPath(path);
        return false;
    }

    for (int index = 0; index < library.poseSlots.size(); ++index)
    {
        const PoseLibrarySlot& slot = library.poseSlots[index];
        ini.SetSectionName(LocalString(QString("WeldPoseComp%1").arg(index)));
        if (!ini.WriteString("Name", LocalString(slot.name))
            || !ini.WriteString("SegmentKind", LocalString(slot.segmentKind))
            || !ini.WriteString("Rx", slot.rx, 6)
            || !ini.WriteString("Ry", slot.ry, 6)
            || !ini.WriteString("Rz", NormalizeAngleDeg(slot.rz), 6)
            || !ini.WriteString("CompX", slot.compX, 6)
            || !ini.WriteString("CompY", slot.compY, 6)
            || !ini.WriteString("CompZ", slot.compZ, 6))
        {
            error = QString("写入姿态补偿库槽 %1 失败：%2")
                .arg(index)
                .arg(LocalPath(path));
            return false;
        }
    }
    return true;
}

int FindBestPoseSlot(
    const PoseLibrary& library,
    double rx,
    double ry,
    double rz,
    double& bestDistanceDeg)
{
    int bestIndex = -1;
    bestDistanceDeg = std::numeric_limits<double>::max();
    for (int index = 0; index < library.poseSlots.size(); ++index)
    {
        const PoseLibrarySlot& slot = library.poseSlots[index];
        const double distance = PoseDistanceDeg(rx, ry, rz, slot.rx, slot.ry, slot.rz);
        if (distance > library.matchMaxErrorDeg)
        {
            continue;
        }
        if (distance < bestDistanceDeg)
        {
            bestDistanceDeg = distance;
            bestIndex = index;
        }
    }
    return bestIndex;
}

int FindMatchingSegmentPoseSlot(
    const PoseLibrary& library,
    const QString& segmentKind,
    double rx,
    double ry,
    double rz,
    double& bestDistanceDeg)
{
    int bestIndex = -1;
    bestDistanceDeg = std::numeric_limits<double>::max();
    for (int index = 0; index < library.poseSlots.size(); ++index)
    {
        const PoseLibrarySlot& slot = library.poseSlots[index];
        if (slot.segmentKind.compare(segmentKind, Qt::CaseInsensitive) != 0)
        {
            continue;
        }

        const double distance = PoseDistanceDeg(rx, ry, rz, slot.rx, slot.ry, slot.rz);
        if (distance > library.matchMaxErrorDeg)
        {
            continue;
        }
        if (distance < bestDistanceDeg)
        {
            bestDistanceDeg = distance;
            bestIndex = index;
        }
    }
    return bestIndex;
}

WeldPoseAverageUpdater::GroupReport BuildGroupReport(
    const SegmentDef& def,
    const QVector<PoseSample>& samples)
{
    WeldPoseAverageUpdater::GroupReport report;
    report.displayName = QString::fromUtf8(def.displayName);
    report.segmentKind = QString::fromLatin1(def.segmentKind);
    report.sourceCount = samples.size();
    if (samples.isEmpty())
    {
        return report;
    }

    double baseRx = 0.0;
    double baseRy = 0.0;
    double baseRz = 0.0;
    ResolveAveragePose(samples, baseRx, baseRy, baseRz);

    QVector<double> distances;
    distances.reserve(samples.size());
    for (const PoseSample& sample : samples)
    {
        distances.push_back(PoseDistanceDeg(sample.rx, sample.ry, sample.rz, baseRx, baseRy, baseRz));
    }

    const double medianDistance = Median(distances);
    QVector<double> deviations;
    deviations.reserve(distances.size());
    for (double distance : distances)
    {
        deviations.push_back(std::abs(distance - medianDistance));
    }

    const double mad = Median(deviations);
    report.outlierThresholdDeg = samples.size() >= 3
        ? std::max(MIN_OUTLIER_THRESHOLD_DEG, medianDistance + 3.0 * 1.4826 * mad)
        : std::numeric_limits<double>::infinity();

    QVector<PoseSample> filteredSamples;
    filteredSamples.reserve(samples.size());
    for (int index = 0; index < samples.size(); ++index)
    {
        if (distances[index] <= report.outlierThresholdDeg)
        {
            filteredSamples.push_back(samples[index]);
        }
    }
    if (filteredSamples.isEmpty())
    {
        filteredSamples = samples;
    }

    report.usedCount = filteredSamples.size();
    report.filteredCount = report.sourceCount - report.usedCount;
    report.hasAverage = ResolveAveragePose(filteredSamples, report.rx, report.ry, report.rz);
    return report;
}

bool WriteSeamAverageMetadata(
    const QString& path,
    const WeldPoseAverageUpdater::UpdateResult& result,
    QString& error)
{
    COPini ini;
    ini.SetFileName(false, LocalString(path));
    ini.SetSectionName("WeldSeamPoseAverage");
    if (!ini.WriteString("LastUpdateTime", LocalString(QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss")))
        || !ini.WriteString("SourcePoseFile", LocalString(result.inputPoseFilePath))
        || !ini.WriteString("PoseCompParamPath", LocalString(result.poseCompParamPath))
        || !ini.WriteString("GroupCount", static_cast<int>(result.groups.size()))
        || !ini.WriteString("AnyOutlierFiltered", result.anyOutlierFiltered))
    {
        error = "写入焊道补偿参数姿态均值摘要失败：" + LocalPath(path);
        return false;
    }

    for (int index = 0; index < result.groups.size(); ++index)
    {
        const WeldPoseAverageUpdater::GroupReport& group = result.groups[index];
        ini.SetSectionName(LocalString(QString("WeldSeamPoseAverage%1").arg(index)));
        if (!ini.WriteString("Name", LocalString(group.displayName))
            || !ini.WriteString("SegmentKind", LocalString(group.segmentKind))
            || !ini.WriteString("PoseCompIndex", group.poseCompIndex)
            || !ini.WriteString("SampleCount", group.sourceCount)
            || !ini.WriteString("UsedSampleCount", group.usedCount)
            || !ini.WriteString("FilteredOutlierCount", group.filteredCount)
            || !ini.WriteString("Rx", group.rx, 6)
            || !ini.WriteString("Ry", group.ry, 6)
            || !ini.WriteString("Rz", NormalizeAngleDeg(group.rz), 6)
            || !ini.WriteString("AddedNewSlot", group.addedNewSlot)
            || !ini.WriteString("WrotePoseToSlot", group.wrotePoseToSlot)
            || !ini.WriteString("MatchedDistanceDeg", group.matchedDistanceDeg, 6))
        {
            error = QString("写入焊道补偿参数姿态均值分组 %1 失败：%2")
                .arg(index)
                .arg(LocalPath(path));
            return false;
        }
    }

    return true;
}

QString FormatFiniteThreshold(double threshold)
{
    if (!std::isfinite(threshold))
    {
        return "n/a";
    }
    return QString::number(threshold, 'f', 3);
}

QString FormatPose(double rx, double ry, double rz)
{
    return QString("RX=%1, RY=%2, RZ=%3")
        .arg(rx, 0, 'f', 3)
        .arg(ry, 0, 'f', 3)
        .arg(NormalizeAngleDeg(rz), 0, 'f', 3);
}

QString BuildReportPath(const QString& inputPoseFilePath)
{
    return QFileInfo(inputPoseFilePath).dir().filePath(REPORT_FILE_NAME);
}

void BuildReportLines(
    WeldPoseAverageUpdater::UpdateResult& result,
    const QStringList& warnings,
    int previousPoseCompCount,
    double matchMaxErrorDeg)
{
    QStringList lines;
    lines << "一键更新焊道姿态均值报告";
    lines << QString("更新时间：%1").arg(QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss"));
    lines << QString("机器人：%1").arg(result.robotName);
    lines << QString("输入姿态文件：%1").arg(LocalPath(result.inputPoseFilePath));
    lines << QString("姿态补偿库：%1").arg(LocalPath(result.poseCompParamPath));
    lines << QString("焊道补偿配置：%1").arg(LocalPath(result.seamCompParamPath));
    lines << QString("总姿态点数：%1").arg(result.totalRecordCount);
    lines << QString("姿态复用阈值：%1 deg").arg(matchMaxErrorDeg, 0, 'f', 3);
    lines << QString("姿态库数量：%1 -> %2").arg(previousPoseCompCount).arg(previousPoseCompCount + result.addedSlotCount);

    for (const QString& warning : warnings)
    {
        lines << "提示：" + warning;
    }

    lines << "";
    lines << "分组统计：";
    for (const WeldPoseAverageUpdater::GroupReport& group : result.groups)
    {
        if (!group.hasAverage)
        {
            lines << QString("- %1 [%2]：无有效样本，未更新。")
                .arg(group.displayName, group.segmentKind);
            continue;
        }

        QString action;
        if (group.addedNewSlot)
        {
            action = QString("新增姿态组 WeldPoseComp%1").arg(group.poseCompIndex);
        }
        else if (group.wrotePoseToSlot)
        {
            action = QString("复用并更新 WeldPoseComp%1，距离=%2 deg")
                .arg(group.poseCompIndex)
                .arg(group.matchedDistanceDeg, 0, 'f', 3);
        }
        else
        {
            action = QString("直接复用 WeldPoseComp%1，距离=%2 deg，本次不重复覆盖")
                .arg(group.poseCompIndex)
                .arg(group.matchedDistanceDeg, 0, 'f', 3);
        }
        lines << QString("- %1 [%2]：样本=%3，使用=%4，过滤异常=%5，异常阈值=%6 deg，%7，%8")
            .arg(group.displayName)
            .arg(group.segmentKind)
            .arg(group.sourceCount)
            .arg(group.usedCount)
            .arg(group.filteredCount)
            .arg(FormatFiniteThreshold(group.outlierThresholdDeg))
            .arg(FormatPose(group.rx, group.ry, group.rz))
            .arg(action);

        lines << QString("  槽位更新前：%1 [%2] %3")
            .arg(group.previousName.isEmpty() ? QString("空槽") : group.previousName)
            .arg(group.previousSegmentKind.isEmpty() ? QString("unassigned") : group.previousSegmentKind)
            .arg(FormatPose(group.previousRx, group.previousRy, group.previousRz));
        if (group.wrotePoseToSlot)
        {
            lines << QString("  槽位更新后：WeldPoseComp%1 %2").arg(group.poseCompIndex).arg(FormatPose(group.rx, group.ry, group.rz));
        }
        else
        {
            lines << QString("  本组均值：%1；复用槽位保持：WeldPoseComp%2 %3")
                .arg(FormatPose(group.rx, group.ry, group.rz))
                .arg(group.poseCompIndex)
                .arg(FormatPose(group.previousRx, group.previousRy, group.previousRz));
        }
    }

    lines << "";
    lines << QString("异常值过滤：%1").arg(result.anyOutlierFiltered ? "有" : "无");
    lines << QString("新增姿态组：%1，复用姿态组：%2").arg(result.addedSlotCount).arg(result.reusedSlotCount);
    lines << QString("报告文件：%1").arg(LocalPath(result.reportPath));
    result.reportLines = lines;
}
}


bool WeldPoseAverageUpdater::UpdateFromInput(
    const QString& inputPathOrResultDir,
    const QString& robotNameHint,
    UpdateResult& result,
    QString& error)
{
    result = UpdateResult();
    error.clear();

    QStringList warnings;
    const QString poseFilePath = ResolvePoseFilePath(inputPathOrResultDir, warnings, error);
    if (poseFilePath.isEmpty())
    {
        return false;
    }

    QVector<WeldPoseRecord> records;
    if (!LoadWeldPoseRecords(poseFilePath, records, error))
    {
        return false;
    }

    result.inputPoseFilePath = QFileInfo(poseFilePath).absoluteFilePath();
    result.robotName = robotNameHint.trimmed().isEmpty()
        ? InferRobotNameFromResultPath(result.inputPoseFilePath)
        : robotNameHint.trimmed();
    result.poseCompParamPath = RobotDataHelper::BuildProjectPath(
        QString("Data/%1/WeldPoseCompParam.ini").arg(result.robotName));
    result.seamCompParamPath = RobotDataHelper::BuildProjectPath(
        QString("Data/%1/WeldSeamCompParam.ini").arg(result.robotName));
    result.reportPath = BuildReportPath(result.inputPoseFilePath);
    result.totalRecordCount = records.size();

    QVector<QVector<PoseSample>> groupedSamples(kDefaultSegmentCount);
    for (const WeldPoseRecord& record : records)
    {
        if (!IsDefaultSegmentKind(record.segmentKind))
        {
            continue;
        }
        for (int index = 0; index < kDefaultSegmentCount; ++index)
        {
            if (record.segmentKind.compare(kDefaultSegments[index].segmentKind, Qt::CaseInsensitive) == 0)
            {
                groupedSamples[index].push_back({ record.rx, record.ry, record.rz });
                break;
            }
        }
    }

    PoseLibrary library = LoadPoseLibrary(result.poseCompParamPath);
    const int previousPoseCompCount = library.poseSlots.size();
    const double matchMaxErrorDeg = library.matchMaxErrorDeg;
    QSet<int> writtenPoseSlotIndexes;

    for (int index = 0; index < kDefaultSegmentCount; ++index)
    {
        GroupReport report = BuildGroupReport(kDefaultSegments[index], groupedSamples[index]);
        if (!report.hasAverage)
        {
            result.groups.push_back(report);
            continue;
        }

        double bestDistanceDeg = std::numeric_limits<double>::max();
        int bestIndex = FindMatchingSegmentPoseSlot(
            library,
            report.segmentKind,
            report.rx,
            report.ry,
            report.rz,
            bestDistanceDeg);
        if (bestIndex < 0)
        {
            bestIndex = FindBestPoseSlot(library, report.rx, report.ry, report.rz, bestDistanceDeg);
        }
        if (bestIndex >= 0)
        {
            PoseLibrarySlot& slot = library.poseSlots[bestIndex];
            const bool alreadyWritten = writtenPoseSlotIndexes.contains(bestIndex);
            report.poseCompIndex = bestIndex;
            report.reusedExistingSlot = true;
            report.wrotePoseToSlot = !alreadyWritten;
            report.matchedDistanceDeg = bestDistanceDeg;
            report.previousName = slot.name;
            report.previousSegmentKind = slot.segmentKind;
            report.previousRx = slot.rx;
            report.previousRy = slot.ry;
            report.previousRz = slot.rz;

            if (!alreadyWritten)
            {
                slot.rx = report.rx;
                slot.ry = report.ry;
                slot.rz = report.rz;
                if (slot.name.trimmed().isEmpty())
                {
                    slot.name = report.displayName;
                }
                if (slot.segmentKind.trimmed().isEmpty())
                {
                    slot.segmentKind = report.segmentKind;
                }
                writtenPoseSlotIndexes.insert(bestIndex);
            }
            ++result.reusedSlotCount;
        }
        else
        {
            PoseLibrarySlot slot;
            slot.name = QString("离线姿态 / %1").arg(report.displayName);
            slot.segmentKind = report.segmentKind;
            slot.rx = report.rx;
            slot.ry = report.ry;
            slot.rz = report.rz;
            library.poseSlots.push_back(slot);

            report.poseCompIndex = library.poseSlots.size() - 1;
            report.addedNewSlot = true;
            report.wrotePoseToSlot = true;
            report.matchedDistanceDeg = 0.0;
            report.previousName.clear();
            report.previousSegmentKind.clear();
            report.previousRx = 0.0;
            report.previousRy = 0.0;
            report.previousRz = 0.0;
            writtenPoseSlotIndexes.insert(report.poseCompIndex);
            ++result.addedSlotCount;
        }

        if (report.filteredCount > 0)
        {
            result.anyOutlierFiltered = true;
        }
        result.groups.push_back(report);
    }

    if (!SavePoseLibrary(result.poseCompParamPath, library, error))
    {
        return false;
    }

    if (!WriteSeamAverageMetadata(result.seamCompParamPath, result, error))
    {
        return false;
    }

    BuildReportLines(result, warnings, previousPoseCompCount, matchMaxErrorDeg);
    if (!RobotDataHelper::SaveTextFileLines(result.reportPath, result.reportLines, &error))
    {
        return false;
    }

    return true;
}
