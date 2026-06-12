#pragma once

#include "PointCloudProcessingConfig.h"
#include "RobotCalculation.h"

#include <Eigen/Dense>

#include <QVector>

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
        const QString& baseWeldOutputPath = QString());

    static RobotCalculation::MeasureThenWeldAnalysisResult BuildAnalysisResult(
        const ExtractionResult& extraction,
        const RobotCalculation::LowerWeldFilterParams& params);
};
