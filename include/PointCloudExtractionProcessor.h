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
        QVector<TrackPoint> points;
        int inputPointCount = 0;
        QString dllPath;
        QString configPath;
    };

    static ExtractionResult ExtractCorrugatedSheet(
        const QVector<RobotCalculation::IndexedPoint3D>& inputPoints,
        const PointCloudProcessingConfig::Settings& settings,
        const Eigen::Vector3d& scanDirection);

    static RobotCalculation::MeasureThenWeldAnalysisResult BuildAnalysisResult(
        const ExtractionResult& extraction,
        const RobotCalculation::LowerWeldFilterParams& params);
};
