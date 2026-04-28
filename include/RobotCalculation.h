#pragma once

#include "Const.h"
#include "HandEyeMatrixConfig.h"

#include <QString>
#include <QVector>

#include <Eigen/Dense>

#include <vector>

class RobotCalculation
{
public:
    struct TimestampedRobotPose
    {
        qint64 timestampUs = 0;
        T_ROBOT_COORS pose;
    };

    struct IndexedPoint3D
    {
        int index = 0;
        Eigen::Vector3d point = Eigen::Vector3d::Zero();
    };

    enum class SampleAxis
    {
        AxisY = 0,
        AxisX = 1
    };

    enum class LowerWeldFitMode
    {
        PreservePath = 0,
        LineFit = 1,
        TrapezoidFit = 2,
        PiecewiseLineFit = 3
    };

    struct LowerWeldFilterParams
    {
        SampleAxis sampleAxis = SampleAxis::AxisY;
        LowerWeldFitMode fitMode = LowerWeldFitMode::PreservePath;
        double zThreshold = -230.0;
        double zJumpThreshold = 5.0;
        double zContinuityThreshold = 3.0;
        double segmentBreakDistance = 12.0;
        bool keepLongestSegmentOnly = true;
        double sampleStep = 2.0;
        double searchWindow = 8.0;
        int lineFitTrimCount = 0;
        double piecewiseFitTolerance = 2.0;
        int piecewiseMinSegmentPoints = 4;
        int minPointCount = 3;
        int smoothRadius = 2;
    };

    struct LowerWeldFilterPoint
    {
        int index = 0;
        Eigen::Vector3d point = Eigen::Vector3d::Zero();
        QString source;
    };

    struct LowerWeldFilterResult
    {
        bool ok = false;
        QString error;
        QVector<LowerWeldFilterPoint> points;
        int inputPointCount = 0;
        int lowerPointCount = 0;
        int zJumpRejectedCount = 0;
        int zContinuityRejectedCount = 0;
        int segmentRejectedCount = 0;
        int fitSegmentCount = 0;
        int measuredCount = 0;
        int interpolatedCount = 0;
        int extendedCount = 0;
    };

    enum class LowerWeldPointType
    {
        Start = 1,
        End = 2,
        InnerCorner = 3,
        OuterCorner = 4,
        Normal = 5,
        Noise = 6
    };

    struct LowerWeldClassifiedPoint
    {
        int index = 0;
        Eigen::Vector3d point = Eigen::Vector3d::Zero();
        LowerWeldPointType type = LowerWeldPointType::Normal;
        QString source;
    };

    struct LowerWeldClassificationResult
    {
        bool ok = false;
        QString error;
        QVector<LowerWeldClassifiedPoint> points;
        int startCount = 0;
        int endCount = 0;
        int innerCornerCount = 0;
        int outerCornerCount = 0;
        int normalCount = 0;
        int noiseCount = 0;
    };

    static T_ROBOT_COORS InterpolateRobotPose(const std::vector<TimestampedRobotPose>& robotSamples, qint64 targetTimestampUs);
    static Eigen::Vector3d CalcLaserPointInRobot(const T_ROBOT_COORS& robotPose, const Eigen::Vector3d& cameraPoint, const HandEyeMatrixConfig& calibration);
    static LowerWeldFilterResult FilterLowerWeldPath(const QVector<IndexedPoint3D>& inputPoints, const LowerWeldFilterParams& params);
    static LowerWeldClassificationResult ClassifyLowerWeldPoints(
        const LowerWeldFilterResult& filterResult,
        SampleAxis sampleAxis);
    static int LowerWeldPointTypeCode(LowerWeldPointType type);
    static QString LowerWeldPointTypeName(LowerWeldPointType type);
    static QString RobotPoseCsv(qint64 timestampUs, const T_ROBOT_COORS& pose);
    static QString Vector3Csv(qint64 timestampUs, const Eigen::Vector3d& point, const QString& extra = QString());
    static QString RobotPoseIndexedCsv(int index, const T_ROBOT_COORS& pose);
    static QString Vector3IndexedCsv(int index, const Eigen::Vector3d& point, const QString& extra = QString());
    static QString Vector3IndexedSpaceText(int index, const Eigen::Vector3d& point, const QString& extra = QString());

private:
    static Eigen::Matrix3d RotX(double w);
    static Eigen::Matrix3d RotY(double p);
    static Eigen::Matrix3d RotZ(double r);
    static Eigen::Matrix3d FanucRot(double w, double p, double r);
};
