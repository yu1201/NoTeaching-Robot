#pragma once

#include "Const.h"

#include <Eigen/Dense>

#include <cmath>

namespace RobotPoseTransform
{
constexpr double kPi = 3.14159265358979323846;

inline int NormalizeRobotType(int robotType)
{
    return robotType == ROBOT_TYPE_STEP ? ROBOT_TYPE_STEP : ROBOT_TYPE_FANUC;
}

inline Eigen::Matrix3d RotXDeg(double angleDeg)
{
    const double angleRad = angleDeg * kPi / 180.0;
    const double c = std::cos(angleRad);
    const double s = std::sin(angleRad);
    return (Eigen::Matrix3d() << 1.0, 0.0, 0.0,
                                  0.0, c, -s,
                                  0.0, s, c).finished();
}

inline Eigen::Matrix3d RotYDeg(double angleDeg)
{
    const double angleRad = angleDeg * kPi / 180.0;
    const double c = std::cos(angleRad);
    const double s = std::sin(angleRad);
    return (Eigen::Matrix3d() << c, 0.0, s,
                                  0.0, 1.0, 0.0,
                                  -s, 0.0, c).finished();
}

inline Eigen::Matrix3d RotZDeg(double angleDeg)
{
    const double angleRad = angleDeg * kPi / 180.0;
    const double c = std::cos(angleRad);
    const double s = std::sin(angleRad);
    return (Eigen::Matrix3d() << c, -s, 0.0,
                                  s, c, 0.0,
                                  0.0, 0.0, 1.0).finished();
}

inline Eigen::Matrix3d RotationFromAnglesDeg(double rxDeg, double ryDeg, double rzDeg, int robotType)
{
    if (NormalizeRobotType(robotType) == ROBOT_TYPE_STEP)
    {
        return RotXDeg(rxDeg) * RotYDeg(ryDeg) * RotZDeg(rzDeg);
    }

    return RotZDeg(rzDeg) * RotYDeg(ryDeg) * RotXDeg(rxDeg);
}

inline Eigen::Matrix3d RotationFromPose(const T_ROBOT_COORS& pose, int robotType)
{
    return RotationFromAnglesDeg(pose.dRX, pose.dRY, pose.dRZ, robotType);
}
}
