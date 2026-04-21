#pragma once

#include "Const.h"
#include "HandEyeMatrixConfig.h"

#include <QString>

#include <Eigen/Dense>

#include <vector>

class RobotCalculation
{
public:
    struct TimestampedRobotPose
    {
        qint64 timestampMs = 0;
        T_ROBOT_COORS pose;
    };

    static T_ROBOT_COORS InterpolateRobotPose(const std::vector<TimestampedRobotPose>& robotSamples, qint64 targetTimestampMs);
    static Eigen::Vector3d CalcLaserPointInRobot(const T_ROBOT_COORS& robotPose, const Eigen::Vector3d& cameraPoint, const HandEyeMatrixConfig& calibration);
    static QString RobotPoseCsv(qint64 timestampMs, const T_ROBOT_COORS& pose);
    static QString Vector3Csv(qint64 timestampMs, const Eigen::Vector3d& point, const QString& extra = QString());
    static QString RobotPoseIndexedCsv(int index, const T_ROBOT_COORS& pose);
    static QString Vector3IndexedCsv(int index, const Eigen::Vector3d& point, const QString& extra = QString());

private:
    static Eigen::Matrix3d RotX(double w);
    static Eigen::Matrix3d RotY(double p);
    static Eigen::Matrix3d RotZ(double r);
    static Eigen::Matrix3d FanucRot(double w, double p, double r);
};
