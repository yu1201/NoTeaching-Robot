#pragma once

#include "PointCloudProcessingConfig.h"
#include "RobotCalculation.h"

#include <Eigen/Dense>

#include <QVector>

#include <functional>

class PointCloudExtractionProcessor
{
public:
    enum class TrackPointType
    {
        Normal = 0,
        Start = 1,
        End = 2,
        Corner = 3
    };

    struct TrackPoint
    {
        int index = 0;
        Eigen::Vector3d point = Eigen::Vector3d::Zero();
        TrackPointType type = TrackPointType::Normal;
    };

    struct ExtractionResult
    {
        bool ok = false;
        QString error;
        QVector<TrackPoint> rawPoints;
        QVector<TrackPoint> keyPointExpandedPoints;
        QVector<TrackPoint> points;
        int inputPointCount = 0;
        int finiteInputPointCount = 0;
        int invalidInputPointCount = 0;
        QString dllPath;
        QString configPath;
        QString baseWeldPath;
        bool usedBaseWeldFile = false;
        // 20260609 版 SDK：检测到工件已焊段时返回新焊接的起点（用于焊道截断），无已焊段时无效。
        bool hasWeldedStartPoint = false;
        Eigen::Vector3d weldedStartPoint = Eigen::Vector3d::Zero();
    };

    static ExtractionResult ExtractCorrugatedSheet(
        const QVector<RobotCalculation::IndexedPoint3D>& inputPoints,
        const PointCloudProcessingConfig::Settings& settings,
        const Eigen::Vector3d& scanDirection,
        const QString& baseWeldOutputPath,
        const QString& runtimeConfigDir);

    // 进程隔离版：把上面的 SDK 调用放到子进程执行。SDK(PointCloudExtration→pcl_kdtree)在多线程下
    // 可能段错误(0xC0000005)拖垮整个上位机；子进程崩溃时本函数检测到崩溃退出、返回可报告错误，
    // 主进程(GUI/机器人)不受影响。逻辑等价 ExtractCorrugatedSheet，仅多一层进程边界。
    static ExtractionResult ExtractCorrugatedSheetIsolated(
        const QVector<RobotCalculation::IndexedPoint3D>& inputPoints,
        const PointCloudProcessingConfig::Settings& settings,
        const Eigen::Vector3d& scanDirection,
        const QString& baseWeldOutputPath = QString(),
        const std::function<bool()>& stopRequested = std::function<bool()>());

    // 子进程入口：由 main() 在构造主窗口前拦截 --pointcloud-extract-worker 调用。
    // workerArgs = [inputCloudFile, scanX, scanY, scanZ, baseWeldOutputPath, resultFile,
    //               parentOwnedRuntimeDir, settingsSnapshotFile]。所有临时文件均放在父进程持有的 QTemporaryDir 下，
    // 即使 worker 因 SDK 硬崩溃而无法析构，父进程仍会递归清理。
    static int RunExtractWorker(const QStringList& workerArgs);

    static RobotCalculation::MeasureThenWeldAnalysisResult BuildAnalysisResult(
        const ExtractionResult& extraction,
        const RobotCalculation::LowerWeldFilterParams& params);
};
