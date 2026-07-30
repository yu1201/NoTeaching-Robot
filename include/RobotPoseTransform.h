#pragma once

#include "Const.h"

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

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

inline double AngleDegNear(double angleDeg, double referenceDeg)
{
    if (!std::isfinite(angleDeg) || !std::isfinite(referenceDeg))
    {
        return angleDeg;
    }
    return referenceDeg + std::remainder(angleDeg - referenceDeg, 360.0);
}

inline Eigen::Vector3d AnglesFromRotationDegNear(
    const Eigen::Matrix3d& rotation,
    int robotType,
    const Eigen::Vector3d& referenceDeg)
{
    if (!rotation.allFinite() || !referenceDeg.allFinite())
    {
        return referenceDeg;
    }

    constexpr double kGimbalEpsilon = 1e-9;
    const double radiansToDegrees = 180.0 / kPi;
    const int normalizedRobotType = NormalizeRobotType(robotType);
    double rxRad = 0.0;
    double ryRad = 0.0;
    double rzRad = 0.0;
    bool atGimbalLock = false;

    if (normalizedRobotType == ROBOT_TYPE_STEP)
    {
        ryRad = std::asin(std::clamp(rotation(0, 2), -1.0, 1.0));
        const double cosRy = std::cos(ryRad);
        atGimbalLock = std::abs(cosRy) <= kGimbalEpsilon;
        if (!atGimbalLock)
        {
            rxRad = std::atan2(-rotation(1, 2), rotation(2, 2));
            rzRad = std::atan2(-rotation(0, 1), rotation(0, 0));
        }
        else
        {
            rxRad = referenceDeg.x() / radiansToDegrees;
            if (ryRad >= 0.0)
            {
                const double rxPlusRz = std::atan2(rotation(1, 0), rotation(1, 1));
                rzRad = rxPlusRz - rxRad;
            }
            else
            {
                const double rzMinusRx = std::atan2(rotation(1, 0), rotation(1, 1));
                rzRad = rzMinusRx + rxRad;
            }
        }
    }
    else
    {
        ryRad = std::asin(std::clamp(-rotation(2, 0), -1.0, 1.0));
        const double cosRy = std::cos(ryRad);
        atGimbalLock = std::abs(cosRy) <= kGimbalEpsilon;
        if (!atGimbalLock)
        {
            rxRad = std::atan2(rotation(2, 1), rotation(2, 2));
            rzRad = std::atan2(rotation(1, 0), rotation(0, 0));
        }
        else
        {
            rxRad = referenceDeg.x() / radiansToDegrees;
            if (ryRad >= 0.0)
            {
                const double rxMinusRz = std::atan2(rotation(0, 1), rotation(0, 2));
                rzRad = rxRad - rxMinusRz;
            }
            else
            {
                const double rxPlusRz = std::atan2(-rotation(0, 1), rotation(1, 1));
                rzRad = rxPlusRz - rxRad;
            }
        }
    }

    const Eigen::Vector3d primaryDeg(
        rxRad * radiansToDegrees,
        ryRad * radiansToDegrees,
        rzRad * radiansToDegrees);
    const auto unwrapNearReference =
        [&referenceDeg](const Eigen::Vector3d& anglesDeg)
        {
            return Eigen::Vector3d(
                AngleDegNear(anglesDeg.x(), referenceDeg.x()),
                AngleDegNear(anglesDeg.y(), referenceDeg.y()),
                AngleDegNear(anglesDeg.z(), referenceDeg.z()));
        };

    std::array<Eigen::Vector3d, 2> candidates = {
        unwrapNearReference(primaryDeg),
        unwrapNearReference(Eigen::Vector3d(
            primaryDeg.x() + 180.0,
            180.0 - primaryDeg.y(),
            primaryDeg.z() + 180.0))
    };
    const int candidateCount = atGimbalLock ? 1 : 2;
    double bestDistanceSquared = std::numeric_limits<double>::infinity();
    Eigen::Vector3d best = candidates.front();
    for (int index = 0; index < candidateCount; ++index)
    {
        const Eigen::Vector3d& candidate = candidates[index];
        const Eigen::Matrix3d reconstructed = RotationFromAnglesDeg(
            candidate.x(), candidate.y(), candidate.z(), normalizedRobotType);
        if (!reconstructed.allFinite()
            || (reconstructed - rotation).cwiseAbs().maxCoeff() > 1e-6)
        {
            continue;
        }

        const double distanceSquared = (candidate - referenceDeg).squaredNorm();
        if (distanceSquared < bestDistanceSquared)
        {
            bestDistanceSquared = distanceSquared;
            best = candidate;
        }
    }
    return best;
}
}
