#include "PointCloudExtractionProcessor.h"

#include <QByteArray>
#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QProcess>
#include <QRegularExpression>
#include <QStringList>
#include <QTextStream>
#include <QtGlobal>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#ifdef Q_OS_WIN
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#endif

namespace
{
struct ExternalPoint3D
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

enum class ExternalPointType
{
    Start,
    End,
    Corner
};

struct ExternalTrackPoint
{
    ExternalPoint3D pt;
    ExternalPointType type = ExternalPointType::Corner;
};

#ifdef Q_OS_WIN
// 20260609 版 SDK：新增 terminal 引用出参，返回已焊段端点（新焊接的起点），无已焊段时为零点。
using ExtractFn = ExternalTrackPoint* (__cdecl*)(
    ExternalPoint3D*,
    int,
    int*,
    ExternalPoint3D&,
    double,
    ExternalPoint3D,
    const char*);
using ReleaseTrackPointsFn = void (__cdecl*)(ExternalTrackPoint**);

constexpr char EXTRACT_EXPORT_NAME[] =
    "?CorrugatedSheetPointCloudExtration@@YAPEAUTrackPointsPosition@@PEAUPoint_3D@@HPEAHAEAU2@NU2@PEBD@Z";
constexpr char RELEASE_TRACK_POINTS_EXPORT_NAME[] =
    "?ReleaseTrackPoints@@YAXPEAPEAUTrackPointsPosition@@@Z";

// SDK DLL（含 PCL kdtree 等第三方库）在异常输入/参数下可能发生段错误(0xC0000005)而拖垮整个上位机进程
// （现场实测：Z 截断改 -410 后 SDK 返回点→构建 KdTree→pcl_kdtree_release.dll 崩溃）。用 SEH 包裹其调用，
// DLL 内部崩溃时拦截为可报告错误，进程不挂。__try/__except 不能与需要栈展开的 C++ RAII 对象共存于同一
// 函数(C2712)，故单列此无 RAII 局部对象的薄封装。
static ExternalTrackPoint* CallSdkExtractGuarded(
    ExtractFn extract,
    ExternalPoint3D* input,
    int inputCount,
    int* trackCount,
    ExternalPoint3D& terminal,
    double zTrunc,
    ExternalPoint3D scanDir,
    const char* configPath,
    bool* crashed)
{
    *crashed = false;
    __try
    {
        return extract(input, inputCount, trackCount, terminal, zTrunc, scanDir, configPath);
    }
    __except (EXCEPTION_EXECUTE_HANDLER)
    {
        *crashed = true;
        return nullptr;
    }
}
#endif

bool IsFinitePoint(const Eigen::Vector3d& point)
{
    return std::isfinite(point.x()) && std::isfinite(point.y()) && std::isfinite(point.z());
}

ExternalPoint3D ToExternalPoint(const Eigen::Vector3d& point)
{
    ExternalPoint3D external;
    external.x = point.x();
    external.y = point.y();
    external.z = point.z();
    return external;
}

Eigen::Vector3d FromExternalPoint(const ExternalPoint3D& point)
{
    return Eigen::Vector3d(point.x, point.y, point.z);
}

PointCloudExtractionProcessor::TrackPointType FromExternalType(ExternalPointType type)
{
    switch (type)
    {
    case ExternalPointType::Start:
        return PointCloudExtractionProcessor::TrackPointType::Start;
    case ExternalPointType::End:
        return PointCloudExtractionProcessor::TrackPointType::End;
    case ExternalPointType::Corner:
    default:
        return PointCloudExtractionProcessor::TrackPointType::Corner;
    }
}

double PrimaryValue(const Eigen::Vector3d& point, RobotCalculation::SampleAxis axis)
{
    return axis == RobotCalculation::SampleAxis::AxisY ? point.x() : point.y();
}

double PathAxisValue(const Eigen::Vector3d& point, RobotCalculation::SampleAxis axis)
{
    return axis == RobotCalculation::SampleAxis::AxisY ? point.y() : point.x();
}

QVector<int> BuildKeyPointIndexes(const QVector<PointCloudExtractionProcessor::TrackPoint>& points)
{
    using TrackType = PointCloudExtractionProcessor::TrackPointType;

    QVector<int> indexes;
    indexes.reserve(points.size());
    for (int index = 0; index < points.size(); ++index)
    {
        if (points[index].type != TrackType::Normal)
        {
            indexes.push_back(index);
        }
    }
    if (indexes.isEmpty() || indexes.front() != 0)
    {
        indexes.push_front(0);
    }
    if (indexes.back() != points.size() - 1)
    {
        indexes.push_back(points.size() - 1);
    }
    return indexes;
}

QString SegmentKindFromGeometry(
    const Eigen::Vector3d& begin,
    const Eigen::Vector3d& end,
    RobotCalculation::SampleAxis axis,
    double lowHighMidpoint,
    double profileRange)
{
    const double dPath = PathAxisValue(end, axis) - PathAxisValue(begin, axis);
    const double dProfile = PrimaryValue(end, axis) - PrimaryValue(begin, axis);
    const double slopeThresholdMm = std::max(1.0, profileRange * 0.12);
    const double slopeRatioThreshold = 0.25;
    const bool isSlope =
        std::abs(dProfile) >= slopeThresholdMm
        && std::abs(dProfile) >= std::abs(dPath) * slopeRatioThreshold;
    if (isSlope)
    {
        return dProfile >= 0.0 ? "rising_edge" : "falling_edge";
    }

    const double middleProfile = (PrimaryValue(begin, axis) + PrimaryValue(end, axis)) * 0.5;
    return middleProfile <= lowHighMidpoint ? "low_platform" : "high_platform";
}

QVector<QString> AssignSegmentKindsFromTrackGeometry(
    const QVector<PointCloudExtractionProcessor::TrackPoint>& points,
    const QVector<int>& keyIndexes,
    RobotCalculation::SampleAxis axis)
{
    QVector<QString> segmentKinds;
    if (keyIndexes.size() < 2)
    {
        return segmentKinds;
    }

    double minProfile = std::numeric_limits<double>::max();
    double maxProfile = std::numeric_limits<double>::lowest();
    for (const PointCloudExtractionProcessor::TrackPoint& point : points)
    {
        const double profile = PrimaryValue(point.point, axis);
        minProfile = std::min(minProfile, profile);
        maxProfile = std::max(maxProfile, profile);
    }

    if (!std::isfinite(minProfile) || !std::isfinite(maxProfile))
    {
        segmentKinds.fill("low_platform", keyIndexes.size() - 1);
        return segmentKinds;
    }

    const double profileRange = std::max(0.0, maxProfile - minProfile);
    const double lowHighMidpoint = (minProfile + maxProfile) * 0.5;
    segmentKinds.reserve(keyIndexes.size() - 1);
    for (int index = 0; index + 1 < keyIndexes.size(); ++index)
    {
        segmentKinds.push_back(SegmentKindFromGeometry(
            points[keyIndexes[index]].point,
            points[keyIndexes[index + 1]].point,
            axis,
            lowHighMidpoint,
            profileRange));
    }
    return segmentKinds;
}

RobotCalculation::LowerWeldPointType CornerTypeFromAdjacentSegments(
    const QVector<QString>& segmentKinds,
    int keyOrdinal)
{
    const QString previousKind = keyOrdinal > 0 && keyOrdinal - 1 < segmentKinds.size()
        ? segmentKinds[keyOrdinal - 1].trimmed().toLower()
        : QString();
    const QString nextKind = keyOrdinal < segmentKinds.size()
        ? segmentKinds[keyOrdinal].trimmed().toLower()
        : QString();

    if (previousKind == "rising_edge"
        || previousKind == "high_platform"
        || nextKind == "high_platform"
        || nextKind == "falling_edge")
    {
        return RobotCalculation::LowerWeldPointType::OuterCorner;
    }
    return RobotCalculation::LowerWeldPointType::InnerCorner;
}

RobotCalculation::LowerWeldPointType ClassifiedTypeForTrackPoint(
    const PointCloudExtractionProcessor::TrackPoint& point,
    bool isFirst,
    bool isLast,
    const QVector<int>& keyIndexes,
    const QVector<QString>& segmentKinds,
    int pointIndex)
{
    using OutputType = RobotCalculation::LowerWeldPointType;
    using TrackType = PointCloudExtractionProcessor::TrackPointType;

    if (isFirst || point.type == TrackType::Start)
    {
        return OutputType::Start;
    }
    if (isLast || point.type == TrackType::End)
    {
        return OutputType::End;
    }
    if (point.type == TrackType::Corner)
    {
        const int keyOrdinal = keyIndexes.indexOf(pointIndex);
        if (keyOrdinal >= 0)
        {
            return CornerTypeFromAdjacentSegments(segmentKinds, keyOrdinal);
        }
        return OutputType::InnerCorner;
    }
    return OutputType::Normal;
}

QString SourceNameForTrackPoint(PointCloudExtractionProcessor::TrackPointType type)
{
    using TrackType = PointCloudExtractionProcessor::TrackPointType;
    switch (type)
    {
    case TrackType::Start:
        return "external_start";
    case TrackType::End:
        return "external_end";
    case TrackType::Corner:
        return "external_corner";
    case TrackType::Normal:
    default:
        return "external_2mm";
    }
}

QByteArray ConfigLineValue(const QByteArray& content, const QByteArray& key)
{
    const QList<QByteArray> lines = content.split('\n');
    const QByteArray normalizedKey = key.trimmed().toLower();
    for (QByteArray line : lines)
    {
        line = line.trimmed();
        if (line.startsWith("//") || line.startsWith("#"))
        {
            continue;
        }
        const int equalIndex = line.indexOf('=');
        if (equalIndex < 0)
        {
            continue;
        }
        const QByteArray lineKey = line.left(equalIndex).trimmed().toLower();
        if (lineKey == normalizedKey)
        {
            return line.mid(equalIndex + 1).trimmed();
        }
    }
    return QByteArray();
}

bool ConfigValueIsTrue(const QByteArray& content, const QByteArray& key)
{
    const QByteArray value = ConfigLineValue(content, key).toLower();
    return value == "1" || value == "true" || value == "yes";
}

void ReplaceConfigValue(QByteArray* content, const QByteArray& key, const QByteArray& value)
{
    if (content == nullptr)
    {
        return;
    }

    QList<QByteArray> lines = content->split('\n');
    const QByteArray normalizedKey = key.trimmed().toLower();
    bool replaced = false;
    for (QByteArray& line : lines)
    {
        const QByteArray trimmedLine = line.trimmed();
        if (trimmedLine.startsWith("//") || trimmedLine.startsWith("#"))
        {
            continue;
        }
        const int equalIndex = line.indexOf('=');
        if (equalIndex < 0)
        {
            continue;
        }
        const QByteArray lineKey = line.left(equalIndex).trimmed().toLower();
        if (lineKey == normalizedKey)
        {
            line = key + " = " + value;
            replaced = true;
            break;
        }
    }

    if (!replaced)
    {
        lines.push_back(key + " = " + value);
    }
    *content = lines.join('\n');
}


QByteArray ConfigIntegerValue(double value)
{
    const int rounded = std::max(1, static_cast<int>(std::lround(value)));
    return QByteArray::number(rounded);
}

QString RuntimeConfigPathForOutput(const QString& baseWeldOutputPath)
{
    if (!baseWeldOutputPath.trimmed().isEmpty())
    {
        return QFileInfo(baseWeldOutputPath).absoluteDir().filePath("CorrugatedSheetPointCloudEctration.runtime.ini");
    }
    return QDir::temp().filePath("QtWidgetsApplication4_PointCloudExtration/CorrugatedSheetPointCloudEctration.runtime.ini");
}

QString PrepareRuntimeExternalConfigPath(
    const QString& configPath,
    const QString& baseWeldOutputPath,
    double baseWeldStepMm,
    QString* error)
{
    QFile file(configPath);
    if (!file.open(QIODevice::ReadOnly))
    {
        return configPath;
    }
    QByteArray content = file.readAll();
    file.close();

    bool changed = false;
    if (!ConfigValueIsTrue(content, "DEBUGLOG"))
    {
        changed = false;
    }
    else
    {
        const QString logPath = QString::fromLocal8Bit(ConfigLineValue(content, "LOGPATH")).trimmed();
        if (logPath.isEmpty() || !QDir().mkpath(QDir::fromNativeSeparators(logPath)))
        {
            ReplaceConfigValue(&content, "DEBUGLOG", "false");
            changed = true;
        }
    }

    // 20260609 版 DLL 要求 LOGPATH 必须有值（空值直接弹 "Para_name LOGPATH not found"），
    // 配置里留空时向运行时副本注入项目 Log 目录下的默认输出目录（工作目录启动时已设到项目根）。
    if (QString::fromLocal8Bit(ConfigLineValue(content, "LOGPATH")).trimmed().isEmpty())
    {
        const QString fallbackLogPath = QDir::toNativeSeparators(
            QDir::current().absoluteFilePath(QStringLiteral("Log/PointCloudExtration")));
        QDir().mkpath(QDir::fromNativeSeparators(fallbackLogPath));
        ReplaceConfigValue(&content, "LOGPATH", fallbackLogPath.toLocal8Bit());
        changed = true;
    }

    // 20260617 版 DLL 强制要求 is_remove_noise 存在，缺失即报 "Para_name is_remove_noise not found"。
    // 现场配置可能仍指向不含该键的旧 ini（用户自定义 config 路径或历史模板），这里兜底注入默认
    // 开启；已显式配置过（界面写入 true/false）则 ConfigLineValue 非空、保留不覆盖。
    if (ConfigLineValue(content, "is_remove_noise").isEmpty())
    {
        ReplaceConfigValue(&content, "is_remove_noise", "true");
        changed = true;
    }

    const QString normalizedBaseWeldPath = baseWeldOutputPath.trimmed();
    if (!normalizedBaseWeldPath.isEmpty())
    {
        const QFileInfo outputInfo(normalizedBaseWeldPath);
        const QString outputDir = outputInfo.absolutePath();
        if (!QDir().mkpath(outputDir))
        {
            if (error != nullptr)
            {
                *error = QString("创建SDK基础焊道输出目录失败：%1").arg(outputDir);
            }
            return QString();
        }
        ReplaceConfigValue(
            &content,
            "Save_File_Name",
            QDir::toNativeSeparators(outputInfo.absoluteFilePath()).toLocal8Bit());
        ReplaceConfigValue(
            &content,
            "Step",
            ConfigIntegerValue(baseWeldStepMm > 0.0 ? baseWeldStepMm : 2.0));
        changed = true;
    }

    if (!changed)
    {
        return configPath;
    }

    const QString runtimeConfigPath = RuntimeConfigPathForOutput(normalizedBaseWeldPath);
    const QString runtimeDir = QFileInfo(runtimeConfigPath).absolutePath();
    if (!QDir().mkpath(runtimeDir))
    {
        if (error != nullptr)
        {
            *error = QString("创建SDK运行配置目录失败：%1").arg(runtimeDir);
        }
        return QString();
    }

    QFile runtimeConfig(runtimeConfigPath);
    if (!runtimeConfig.open(QIODevice::WriteOnly | QIODevice::Truncate))
    {
        if (error != nullptr)
        {
            *error = QString("写入SDK运行配置失败：%1").arg(runtimeConfigPath);
        }
        return QString();
    }
    runtimeConfig.write(content);
    runtimeConfig.close();
    return runtimeConfigPath;
}

PointCloudExtractionProcessor::TrackPointType SdkBaseWeldPointTypeFromText(const QString& text)
{
    const QString normalized = text.trimmed().toLower();
    if (normalized == "start")
    {
        return PointCloudExtractionProcessor::TrackPointType::Start;
    }
    if (normalized == "end")
    {
        return PointCloudExtractionProcessor::TrackPointType::End;
    }
    return PointCloudExtractionProcessor::TrackPointType::Normal;
}

bool LoadSdkBaseWeldFile(
    const QString& filePath,
    QVector<PointCloudExtractionProcessor::TrackPoint>* points,
    QString* error)
{
    if (points == nullptr)
    {
        return false;
    }
    points->clear();

    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        if (error != nullptr)
        {
            *error = QString("读取SDK基础焊道文件失败：%1").arg(filePath);
        }
        return false;
    }

    QTextStream stream(&file);
    int nextIndex = 1;
    int lineNumber = 0;
    while (!stream.atEnd())
    {
        ++lineNumber;
        const QString line = stream.readLine().trimmed();
        if (line.isEmpty() || line.startsWith('#') || line.startsWith("index", Qt::CaseInsensitive))
        {
            continue;
        }

        const QStringList fields = line.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
        if (fields.size() < 4)
        {
            continue;
        }

        bool xOk = false;
        bool yOk = false;
        bool zOk = false;
        bool firstFieldIsIndex = false;
        fields.front().toInt(&firstFieldIsIndex);
        const int coordinateOffset = firstFieldIsIndex && fields.size() >= 4 ? 1 : 0;
        if (fields.size() < coordinateOffset + 3)
        {
            continue;
        }
        const double x = fields.value(coordinateOffset).toDouble(&xOk);
        const double y = fields.value(coordinateOffset + 1).toDouble(&yOk);
        const double z = fields.value(coordinateOffset + 2).toDouble(&zOk);
        if (!xOk || !yOk || !zOk)
        {
            if (error != nullptr)
            {
                *error = QString("SDK基础焊道文件第%1行坐标无效：%2").arg(lineNumber).arg(line);
            }
            points->clear();
            return false;
        }

        PointCloudExtractionProcessor::TrackPoint point;
        point.index = nextIndex++;
        point.point = Eigen::Vector3d(x, y, z);
        point.type = fields.size() > coordinateOffset + 3
            ? SdkBaseWeldPointTypeFromText(fields.value(coordinateOffset + 3))
            : PointCloudExtractionProcessor::TrackPointType::Normal;
        points->push_back(point);
    }

    if (points->size() < 2)
    {
        if (error != nullptr)
        {
            *error = QString("SDK基础焊道文件点数过少：%1，文件=%2").arg(points->size()).arg(filePath);
        }
        points->clear();
        return false;
    }

    points->front().type = PointCloudExtractionProcessor::TrackPointType::Start;
    points->back().type = PointCloudExtractionProcessor::TrackPointType::End;
    return true;
}

void ApplyReturnedKeyPointTypes(
    QVector<PointCloudExtractionProcessor::TrackPoint>* densePoints,
    const QVector<PointCloudExtractionProcessor::TrackPoint>& keyPoints)
{
    if (densePoints == nullptr || densePoints->isEmpty())
    {
        return;
    }

    for (const PointCloudExtractionProcessor::TrackPoint& keyPoint : keyPoints)
    {
        if (keyPoint.type == PointCloudExtractionProcessor::TrackPointType::Normal)
        {
            continue;
        }

        int bestIndex = -1;
        double bestDistanceSquared = std::numeric_limits<double>::max();
        for (int index = 0; index < densePoints->size(); ++index)
        {
            const double distanceSquared = ((*densePoints)[index].point - keyPoint.point).squaredNorm();
            if (distanceSquared < bestDistanceSquared)
            {
                bestDistanceSquared = distanceSquared;
                bestIndex = index;
            }
        }
        if (bestIndex >= 0)
        {
            (*densePoints)[bestIndex].type = keyPoint.type;
        }
    }

    densePoints->front().type = PointCloudExtractionProcessor::TrackPointType::Start;
    densePoints->back().type = PointCloudExtractionProcessor::TrackPointType::End;
}

void CountClassifiedPoints(RobotCalculation::LowerWeldClassificationResult& result)
{
    result.startCount = 0;
    result.endCount = 0;
    result.innerCornerCount = 0;
    result.outerCornerCount = 0;
    result.normalCount = 0;
    result.noiseCount = 0;

    for (const RobotCalculation::LowerWeldClassifiedPoint& point : result.points)
    {
        switch (point.type)
        {
        case RobotCalculation::LowerWeldPointType::Start:
            ++result.startCount;
            break;
        case RobotCalculation::LowerWeldPointType::End:
            ++result.endCount;
            break;
        case RobotCalculation::LowerWeldPointType::InnerCorner:
            ++result.innerCornerCount;
            break;
        case RobotCalculation::LowerWeldPointType::OuterCorner:
            ++result.outerCornerCount;
            break;
        case RobotCalculation::LowerWeldPointType::Noise:
            ++result.noiseCount;
            break;
        case RobotCalculation::LowerWeldPointType::Normal:
        default:
            ++result.normalCount;
            break;
        }
    }
}

void AppendTrackPoint(
    QVector<PointCloudExtractionProcessor::TrackPoint>& points,
    int& nextIndex,
    const Eigen::Vector3d& point,
    PointCloudExtractionProcessor::TrackPointType type)
{
    PointCloudExtractionProcessor::TrackPoint output;
    output.index = nextIndex++;
    output.point = point;
    output.type = type;
    points.push_back(output);
}

QVector<PointCloudExtractionProcessor::TrackPoint> ResampleTrackPoints(
    const QVector<PointCloudExtractionProcessor::TrackPoint>& source,
    double stepMm)
{
    QVector<PointCloudExtractionProcessor::TrackPoint> output;
    if (source.isEmpty())
    {
        return output;
    }

    const double safeStep = stepMm > 0.0 ? stepMm : 2.0;
    output.reserve(source.size() * 2);
    int nextIndex = 1;
    AppendTrackPoint(output, nextIndex, source.front().point, source.front().type);

    for (int index = 0; index + 1 < source.size(); ++index)
    {
        const PointCloudExtractionProcessor::TrackPoint& begin = source[index];
        const PointCloudExtractionProcessor::TrackPoint& end = source[index + 1];
        const Eigen::Vector3d delta = end.point - begin.point;
        const double length = delta.norm();
        if (length > safeStep)
        {
            for (double distance = safeStep; distance < length - 1e-9; distance += safeStep)
            {
                AppendTrackPoint(
                    output,
                    nextIndex,
                    begin.point + delta * (distance / length),
                    PointCloudExtractionProcessor::TrackPointType::Normal);
            }
        }
        AppendTrackPoint(output, nextIndex, end.point, end.type);
    }

    if (!output.isEmpty())
    {
        output.front().type = PointCloudExtractionProcessor::TrackPointType::Start;
        output.back().type = PointCloudExtractionProcessor::TrackPointType::End;
    }
    return output;
}
}

PointCloudExtractionProcessor::ExtractionResult PointCloudExtractionProcessor::ExtractCorrugatedSheet(
    const QVector<RobotCalculation::IndexedPoint3D>& inputPoints,
    const PointCloudProcessingConfig::Settings& settings,
    const Eigen::Vector3d& scanDirection,
    const QString& baseWeldOutputPath)
{
    ExtractionResult result;
    result.inputPointCount = inputPoints.size();

#ifndef Q_OS_WIN
    result.error = "新版精测点云库当前只支持 Windows DLL。";
    return result;
#else
    if (inputPoints.isEmpty())
    {
        result.error = "局部完整点云为空，无法调用新版精测点云库。";
        return result;
    }

    QString libraryDir = settings.libraryDir.trimmed().isEmpty()
        ? PointCloudProcessingConfig::DefaultLibraryDir()
        : settings.libraryDir.trimmed();
    QString dllPath = QDir(libraryDir).filePath("PointCloudExtration.dll");
    // 配置目录可能是旧部署/别的盘符遗留（如配置库残留 B:\NoTeaching-Robot\...），在其下找不到 DLL 时
    // 回退到工程内默认目录，避免换机器/换盘符后因数据库里的绝对路径报"未找到精测点云库"。
    if (!QFileInfo::exists(dllPath))
    {
        const QString fallbackDll = QDir(PointCloudProcessingConfig::DefaultLibraryDir()).filePath("PointCloudExtration.dll");
        if (QFileInfo::exists(fallbackDll))
        {
            dllPath = fallbackDll;
        }
    }
    result.dllPath = QDir::toNativeSeparators(QFileInfo(dllPath).absoluteFilePath());
    QString configPath = settings.configPath.trimmed().isEmpty()
        ? PointCloudProcessingConfig::DefaultConfigPath()
        : settings.configPath.trimmed();
    if (!QFileInfo::exists(configPath))
    {
        const QString fallbackConfig = PointCloudProcessingConfig::DefaultConfigPath();
        if (QFileInfo::exists(fallbackConfig))
        {
            configPath = fallbackConfig;
        }
    }
    result.configPath = QDir::toNativeSeparators(configPath);
    if (!baseWeldOutputPath.trimmed().isEmpty())
    {
        result.baseWeldPath = QDir::toNativeSeparators(QFileInfo(baseWeldOutputPath).absoluteFilePath());
    }

    if (!QFileInfo::exists(result.dllPath))
    {
        result.error = "未找到新版精测点云库：" + result.dllPath;
        return result;
    }
    if (!QFileInfo::exists(result.configPath))
    {
        result.error = "未找到新版精测点云配置：" + result.configPath;
        return result;
    }
    QString runtimeConfigError;
    result.configPath = QDir::toNativeSeparators(
        QFileInfo(PrepareRuntimeExternalConfigPath(
            result.configPath,
            result.baseWeldPath,
            settings.resampleStepMm,
            &runtimeConfigError)).absoluteFilePath());
    if (!runtimeConfigError.isEmpty())
    {
        result.error = runtimeConfigError;
        return result;
    }
    if (!QFileInfo::exists(result.configPath))
    {
        result.error = "未找到新版精测点云运行配置：" + result.configPath;
        return result;
    }
    if (!result.baseWeldPath.isEmpty()
        && QFileInfo::exists(result.baseWeldPath)
        && !QFile::remove(result.baseWeldPath))
    {
        result.error = "清理旧SDK基础焊道文件失败：" + result.baseWeldPath;
        return result;
    }

    std::vector<ExternalPoint3D> externalInput;
    externalInput.reserve(inputPoints.size());
    for (const RobotCalculation::IndexedPoint3D& point : inputPoints)
    {
        if (IsFinitePoint(point.point))
        {
            externalInput.push_back(ToExternalPoint(point.point));
        }
    }
    if (externalInput.empty())
    {
        result.error = "局部完整点云中没有有效点，无法调用新版精测点云库。";
        return result;
    }

    Eigen::Vector3d safeScanDirection = scanDirection;
    if (!IsFinitePoint(safeScanDirection) || safeScanDirection.norm() <= std::numeric_limits<double>::epsilon())
    {
        safeScanDirection = Eigen::Vector3d::UnitX();
    }
    safeScanDirection.normalize();

    HMODULE library = LoadLibraryExW(
        reinterpret_cast<LPCWSTR>(result.dllPath.utf16()),
        nullptr,
        LOAD_WITH_ALTERED_SEARCH_PATH);
    if (library == nullptr)
    {
        result.error = QString("加载新版精测点云库失败：%1，Win32错误=%2")
            .arg(result.dllPath)
            .arg(static_cast<unsigned long>(GetLastError()));
        return result;
    }

    auto releaseLibrary = [&library]()
    {
        if (library != nullptr)
        {
            FreeLibrary(library);
            library = nullptr;
        }
    };

    const auto extract = reinterpret_cast<ExtractFn>(GetProcAddress(library, EXTRACT_EXPORT_NAME));
    const auto releaseTrackPoints = reinterpret_cast<ReleaseTrackPointsFn>(
        GetProcAddress(library, RELEASE_TRACK_POINTS_EXPORT_NAME));
    if (extract == nullptr || releaseTrackPoints == nullptr)
    {
        result.error = QString("新版精测点云库导出函数不完整：extract=%1 release=%2")
            .arg(extract == nullptr ? "missing" : "ok")
            .arg(releaseTrackPoints == nullptr ? "missing" : "ok");
        releaseLibrary();
        return result;
    }

    int trackPointCount = 0;
    const QByteArray configPathBytes = result.configPath.toLocal8Bit();
    ExternalPoint3D weldedTerminal{};
    bool sdkCrashed = false;
    ExternalTrackPoint* rawTrackPoints = CallSdkExtractGuarded(
        extract,
        externalInput.data(),
        static_cast<int>(externalInput.size()),
        &trackPointCount,
        weldedTerminal,
        settings.zTruncationValue,
        ToExternalPoint(safeScanDirection),
        configPathBytes.constData(),
        &sdkCrashed);
    if (sdkCrashed)
    {
        result.error = QString("新版精测点云库内部崩溃(异常 0xC0000005，出错模块 pcl_kdtree)，已拦截避免主程序崩溃。"
            "常见诱因：Z 截断值=%1 不当导致截断后点集为空/退化，或输入点云含异常坐标(NaN/Inf)。请检查 Z 截断值与点云。")
            .arg(settings.zTruncationValue, 0, 'f', 1);
        releaseLibrary();
        return result;
    }

    if (rawTrackPoints == nullptr || trackPointCount <= 0)
    {
        result.error = QString("新版精测点云库未返回有效焊道点：输出点数=%1").arg(trackPointCount);
        if (rawTrackPoints != nullptr)
        {
            releaseTrackPoints(&rawTrackPoints);
        }
        releaseLibrary();
        return result;
    }

    QVector<TrackPoint> rawPoints;
    rawPoints.reserve(trackPointCount);
    for (int index = 0; index < trackPointCount; ++index)
    {
        const Eigen::Vector3d point = FromExternalPoint(rawTrackPoints[index].pt);
        if (!IsFinitePoint(point))
        {
            continue;
        }

        TrackPoint output;
        output.index = rawPoints.size() + 1;
        output.point = point;
        output.type = FromExternalType(rawTrackPoints[index].type);
        rawPoints.push_back(output);
    }

    releaseTrackPoints(&rawTrackPoints);
    releaseLibrary();

    // 已焊起点：SDK 检测到工件上已焊段时返回新焊接的起点，零点/非有限值视为无已焊段。
    const Eigen::Vector3d weldedStart = FromExternalPoint(weldedTerminal);
    if (IsFinitePoint(weldedStart) && weldedStart.norm() > 1e-9)
    {
        result.hasWeldedStartPoint = true;
        result.weldedStartPoint = weldedStart;
    }

    result.rawPoints = rawPoints;
    result.keyPointExpandedPoints = ResampleTrackPoints(rawPoints, settings.resampleStepMm);
    if (!result.baseWeldPath.isEmpty())
    {
        QString baseWeldError;
        QVector<TrackPoint> baseWeldPoints;
        if (!LoadSdkBaseWeldFile(result.baseWeldPath, &baseWeldPoints, &baseWeldError))
        {
            result.error = baseWeldError;
            return result;
        }
        ApplyReturnedKeyPointTypes(&baseWeldPoints, rawPoints);
        result.points = baseWeldPoints;
        result.usedBaseWeldFile = true;
    }
    else
    {
        result.points = result.keyPointExpandedPoints;
    }
    if (result.points.size() < 2)
    {
        result.error = QString("新版精测点云库输出点过少：%1").arg(result.points.size());
        result.points.clear();
        return result;
    }

    result.ok = true;
    return result;
#endif
}

RobotCalculation::MeasureThenWeldAnalysisResult PointCloudExtractionProcessor::BuildAnalysisResult(
    const ExtractionResult& extraction,
    const RobotCalculation::LowerWeldFilterParams& params)
{
    RobotCalculation::MeasureThenWeldAnalysisResult result;
    result.filterResult.inputPointCount = extraction.inputPointCount;

    if (!extraction.ok || extraction.points.size() < 2)
    {
        result.error = extraction.error.isEmpty()
            ? "新版精测点云库输出无效。"
            : extraction.error;
        return result;
    }

    result.filterResult.points.reserve(extraction.points.size());
    result.classificationResult.points.reserve(extraction.points.size());
    result.keyPoints.reserve(extraction.points.size());
    const QVector<int> keyIndexes = BuildKeyPointIndexes(extraction.points);
    const QVector<QString> segmentKinds =
        AssignSegmentKindsFromTrackGeometry(extraction.points, keyIndexes, params.sampleAxis);

    for (int index = 0; index < extraction.points.size(); ++index)
    {
        const TrackPoint& trackPoint = extraction.points[index];
        const bool isFirst = index == 0;
        const bool isLast = index == extraction.points.size() - 1;
        const RobotCalculation::LowerWeldPointType pointType =
            ClassifiedTypeForTrackPoint(trackPoint, isFirst, isLast, keyIndexes, segmentKinds, index);

        RobotCalculation::LowerWeldFilterPoint filterPoint;
        filterPoint.index = trackPoint.index;
        filterPoint.point = trackPoint.point;
        filterPoint.source = SourceNameForTrackPoint(trackPoint.type);
        result.filterResult.points.push_back(filterPoint);

        RobotCalculation::LowerWeldClassifiedPoint classifiedPoint;
        classifiedPoint.index = trackPoint.index;
        classifiedPoint.point = trackPoint.point;
        classifiedPoint.type = pointType;
        classifiedPoint.source = filterPoint.source;
        const int keyOrdinal = keyIndexes.indexOf(index);
        if (keyOrdinal >= 0 && keyOrdinal < segmentKinds.size())
        {
            classifiedPoint.segmentKindAfter = segmentKinds[keyOrdinal];
        }
        result.classificationResult.points.push_back(classifiedPoint);

        if (pointType != RobotCalculation::LowerWeldPointType::Normal
            && pointType != RobotCalculation::LowerWeldPointType::Noise)
        {
            result.keyPoints.push_back(classifiedPoint);
        }
    }

    const int outputPointCount = static_cast<int>(extraction.points.size());
    const int keyPointCount = static_cast<int>(result.keyPoints.size());
    result.filterResult.ok = true;
    result.filterResult.lowerPointCount = outputPointCount;
    result.filterResult.fitSegmentCount = std::max(1, keyPointCount - 1);
    result.filterResult.measuredCount = keyPointCount;
    result.filterResult.interpolatedCount = std::max(0, outputPointCount - keyPointCount);
    result.filterResult.extendedCount = result.filterResult.interpolatedCount;

    result.classificationResult.ok = true;
    CountClassifiedPoints(result.classificationResult);
    result.cornerCompensatedClassificationResult =
        RobotCalculation::BuildCornerCompensatedLowerWeldClassification(
            result.keyPoints,
            params,
            &result.cornerCompensatedKeyPoints);

    if (result.classificationResult.startCount <= 0 || result.classificationResult.endCount <= 0)
    {
        result.error = "新版精测点云库输出缺少起点或终点。";
        return result;
    }

    result.ok = true;
    return result;
}

namespace
{
// ====== 进程隔离：点云/结果文件序列化(坐标用 'g',12 保 double 精度) ======
bool WriteWorkerCloudFile(const QString& path, const QVector<RobotCalculation::IndexedPoint3D>& pts)
{
    QFile f(path);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        return false;
    }
    QTextStream out(&f);
    out << pts.size() << '\n';
    for (const RobotCalculation::IndexedPoint3D& p : pts)
    {
        out << QString::number(p.point.x(), 'g', 12) << ' '
            << QString::number(p.point.y(), 'g', 12) << ' '
            << QString::number(p.point.z(), 'g', 12) << '\n';
    }
    return true;
}

QVector<RobotCalculation::IndexedPoint3D> ReadWorkerCloudFile(const QString& path)
{
    QVector<RobotCalculation::IndexedPoint3D> pts;
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        return pts;
    }
    QTextStream in(&f);
    bool first = true;
    int idx = 0;
    while (!in.atEnd())
    {
        const QString line = in.readLine();
        if (first)
        {
            pts.reserve(line.trimmed().toInt());
            first = false;
            continue;
        }
        const QStringList t = line.split(' ', Qt::SkipEmptyParts);
        if (t.size() < 3)
        {
            continue;
        }
        RobotCalculation::IndexedPoint3D p;
        p.index = idx++;
        p.point = Eigen::Vector3d(t[0].toDouble(), t[1].toDouble(), t[2].toDouble());
        pts.push_back(p);
    }
    return pts;
}

void WriteWorkerTrackArray(QTextStream& out, const QVector<PointCloudExtractionProcessor::TrackPoint>& a)
{
    out << a.size() << '\n';
    for (const PointCloudExtractionProcessor::TrackPoint& p : a)
    {
        out << QString::number(p.point.x(), 'g', 12) << ' '
            << QString::number(p.point.y(), 'g', 12) << ' '
            << QString::number(p.point.z(), 'g', 12) << ' '
            << static_cast<int>(p.type) << '\n';
    }
}

QVector<PointCloudExtractionProcessor::TrackPoint> ReadWorkerTrackArray(QTextStream& in)
{
    QVector<PointCloudExtractionProcessor::TrackPoint> a;
    const int n = in.readLine().trimmed().toInt();
    a.reserve(n);
    for (int i = 0; i < n; ++i)
    {
        const QStringList t = in.readLine().split(' ', Qt::SkipEmptyParts);
        if (t.size() < 4)
        {
            continue;
        }
        PointCloudExtractionProcessor::TrackPoint p;
        p.index = i;
        p.point = Eigen::Vector3d(t[0].toDouble(), t[1].toDouble(), t[2].toDouble());
        p.type = static_cast<PointCloudExtractionProcessor::TrackPointType>(t[3].toInt());
        a.push_back(p);
    }
    return a;
}

bool WriteWorkerResultFile(const QString& path, const PointCloudExtractionProcessor::ExtractionResult& r)
{
    QFile f(path);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        return false;
    }
    QTextStream out(&f);
    out << (r.ok ? 1 : 0) << ' ' << r.inputPointCount << ' '
        << (r.usedBaseWeldFile ? 1 : 0) << ' ' << (r.hasWeldedStartPoint ? 1 : 0) << ' '
        << QString::number(r.weldedStartPoint.x(), 'g', 12) << ' '
        << QString::number(r.weldedStartPoint.y(), 'g', 12) << ' '
        << QString::number(r.weldedStartPoint.z(), 'g', 12) << '\n';
    out << QString(r.error).replace('\n', QLatin1Char(' ')) << '\n';  // error 压成单行
    out << r.dllPath << '\n';
    out << r.configPath << '\n';
    out << r.baseWeldPath << '\n';
    WriteWorkerTrackArray(out, r.points);
    WriteWorkerTrackArray(out, r.rawPoints);
    WriteWorkerTrackArray(out, r.keyPointExpandedPoints);
    return true;
}

PointCloudExtractionProcessor::ExtractionResult ReadWorkerResultFile(const QString& path, bool* fileOk)
{
    PointCloudExtractionProcessor::ExtractionResult r;
    *fileOk = false;
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        return r;
    }
    QTextStream in(&f);
    const QStringList head = in.readLine().split(' ', Qt::SkipEmptyParts);
    if (head.size() < 7)
    {
        return r;
    }
    r.ok = head[0].toInt() != 0;
    r.inputPointCount = head[1].toInt();
    r.usedBaseWeldFile = head[2].toInt() != 0;
    r.hasWeldedStartPoint = head[3].toInt() != 0;
    r.weldedStartPoint = Eigen::Vector3d(head[4].toDouble(), head[5].toDouble(), head[6].toDouble());
    r.error = in.readLine();
    r.dllPath = in.readLine();
    r.configPath = in.readLine();
    r.baseWeldPath = in.readLine();
    r.points = ReadWorkerTrackArray(in);
    r.rawPoints = ReadWorkerTrackArray(in);
    r.keyPointExpandedPoints = ReadWorkerTrackArray(in);
    *fileOk = true;
    return r;
}
}  // namespace

PointCloudExtractionProcessor::ExtractionResult PointCloudExtractionProcessor::ExtractCorrugatedSheetIsolated(
    const QVector<RobotCalculation::IndexedPoint3D>& inputPoints,
    const PointCloudProcessingConfig::Settings& settings,
    const Eigen::Vector3d& scanDirection,
    const QString& baseWeldOutputPath)
{
    ExtractionResult result;
    result.inputPointCount = inputPoints.size();

    const QString workDir = QDir::temp().filePath(QStringLiteral("QtWidgetsApplication4_sdkworker"));
    QDir().mkpath(workDir);
    const QString cloudFile = QDir(workDir).filePath(QStringLiteral("input_cloud.txt"));
    const QString resultFile = QDir(workDir).filePath(QStringLiteral("extract_result.txt"));
    QFile::remove(resultFile);

    if (!WriteWorkerCloudFile(cloudFile, inputPoints))
    {
        result.error = "无法写入 SDK 子进程输入点云临时文件：" + cloudFile;
        return result;
    }

    QStringList args;
    args << QStringLiteral("--pointcloud-extract-worker")
         << cloudFile
         << QString::number(scanDirection.x(), 'g', 12)
         << QString::number(scanDirection.y(), 'g', 12)
         << QString::number(scanDirection.z(), 'g', 12)
         << (baseWeldOutputPath.isEmpty() ? QStringLiteral("-") : baseWeldOutputPath)
         << resultFile;

    QProcess proc;
    proc.setProgram(QCoreApplication::applicationFilePath());
    proc.setArguments(args);
    proc.start();
    if (!proc.waitForStarted(15000))
    {
        result.error = "无法启动 SDK 点云子进程（进程隔离）。";
        return result;
    }
    if (!proc.waitForFinished(300000))  // SDK 多线程提取通常数秒，给 5 分钟上限防卡死
    {
        proc.kill();
        proc.waitForFinished(3000);
        result.error = "SDK 点云子进程超时未返回（已终止，主程序未受影响）。请检查点云规模或 SDK 配置。";
        return result;
    }
    if (proc.exitStatus() == QProcess::CrashExit)
    {
        result.error = QString("SDK 点云库子进程崩溃，已被进程隔离拦截、主程序未受影响。"
            "崩溃模块通常为 pcl_kdtree(SDK 内部多线程)，常见诱因：Z 截断值=%1 导致点集为空/退化，或点云含异常坐标。"
            "请调整 Z 截断值/点云，或改用『特征点+拟合』。").arg(settings.zTruncationValue, 0, 'f', 1);
        return result;
    }
    if (proc.exitCode() != 0)
    {
        result.error = QString("SDK 点云子进程异常退出(退出码=%1)，主程序未受影响。").arg(proc.exitCode());
        return result;
    }
    bool fileOk = false;
    const ExtractionResult parsed = ReadWorkerResultFile(resultFile, &fileOk);
    if (!fileOk)
    {
        result.error = "SDK 子进程已结束但结果文件无法解析。";
        return result;
    }
    return parsed;
}

int PointCloudExtractionProcessor::RunExtractWorker(const QStringList& workerArgs)
{
    if (workerArgs.size() < 6)
    {
        return 2;
    }
    const QString cloudFile = workerArgs[0];
    const Eigen::Vector3d scanDir(
        workerArgs[1].toDouble(), workerArgs[2].toDouble(), workerArgs[3].toDouble());
    const QString baseWeldOut = (workerArgs[4] == QStringLiteral("-")) ? QString() : workerArgs[4];
    const QString resultFile = workerArgs[5];

    const QVector<RobotCalculation::IndexedPoint3D> inputPoints = ReadWorkerCloudFile(cloudFile);
    const PointCloudProcessingConfig::Settings settings = PointCloudProcessingConfig::Load();

    // 真正的 SDK 调用：若 SDK(pcl_kdtree 多线程)在此段错误，本子进程崩溃退出，主进程的
    // ExtractCorrugatedSheetIsolated 会识别为 CrashExit 并报错，主程序(GUI/机器人)不受影响。
    const ExtractionResult r = ExtractCorrugatedSheet(inputPoints, settings, scanDir, baseWeldOut);
    WriteWorkerResultFile(resultFile, r);
    return 0;
}
