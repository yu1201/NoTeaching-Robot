#include "PointCloudExtractionProcessor.h"

#include <QByteArray>
#include <QDir>
#include <QFileInfo>

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
using ExtractFn = ExternalTrackPoint* (__cdecl*)(
    ExternalPoint3D*,
    int,
    int*,
    double,
    ExternalPoint3D,
    const char*);
using ReleaseTrackPointsFn = void (__cdecl*)(ExternalTrackPoint**);

constexpr char EXTRACT_EXPORT_NAME[] =
    "?CorrugatedSheetPointCloudExtration@@YAPEAUTrackPointsPosition@@PEAUPoint_3D@@HPEAHNU2@PEBD@Z";
constexpr char RELEASE_TRACK_POINTS_EXPORT_NAME[] =
    "?ReleaseTrackPoints@@YAXPEAPEAUTrackPointsPosition@@@Z";
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
    const Eigen::Vector3d& scanDirection)
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

    const QString libraryDir = settings.libraryDir.trimmed().isEmpty()
        ? PointCloudProcessingConfig::DefaultLibraryDir()
        : settings.libraryDir.trimmed();
    const QString dllPath = QDir(libraryDir).filePath("PointCloudExtration.dll");
    result.dllPath = QDir::toNativeSeparators(QFileInfo(dllPath).absoluteFilePath());
    result.configPath = QDir::toNativeSeparators(settings.configPath.trimmed().isEmpty()
        ? PointCloudProcessingConfig::DefaultConfigPath()
        : settings.configPath.trimmed());

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
    ExternalTrackPoint* rawTrackPoints = extract(
        externalInput.data(),
        static_cast<int>(externalInput.size()),
        &trackPointCount,
        settings.zTruncationValue,
        ToExternalPoint(safeScanDirection),
        configPathBytes.constData());

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

    result.rawPoints = rawPoints;
    result.points = ResampleTrackPoints(rawPoints, settings.resampleStepMm);
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

    if (result.classificationResult.startCount <= 0 || result.classificationResult.endCount <= 0)
    {
        result.error = "新版精测点云库输出缺少起点或终点。";
        return result;
    }

    result.ok = true;
    return result;
}
