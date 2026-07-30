#include "RobotPoseTransform.h"

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <vector>

namespace
{
double RotationDistanceDeg(
    const Eigen::Matrix3d& left,
    const Eigen::Matrix3d& right)
{
    const Eigen::Matrix3d relative = left.transpose() * right;
    const double cosine = std::clamp((relative.trace() - 1.0) * 0.5, -1.0, 1.0);
    return std::acos(cosine) * 180.0 / RobotPoseTransform::kPi;
}

bool Near(double left, double right, double tolerance = 1e-8)
{
    return std::abs(left - right) <= tolerance;
}

bool CheckRoundTrip(
    const Eigen::Vector3d& inputDeg,
    int robotType,
    const Eigen::Vector3d& referenceDeg,
    const char* label)
{
    const Eigen::Matrix3d expected = RobotPoseTransform::RotationFromAnglesDeg(
        inputDeg.x(), inputDeg.y(), inputDeg.z(), robotType);
    const Eigen::Vector3d output = RobotPoseTransform::AnglesFromRotationDegNear(
        expected, robotType, referenceDeg);
    const Eigen::Matrix3d reconstructed = RobotPoseTransform::RotationFromAnglesDeg(
        output.x(), output.y(), output.z(), robotType);
    const double errorDeg = RotationDistanceDeg(expected, reconstructed);
    if (!std::isfinite(errorDeg) || errorDeg > 1e-6)
    {
        std::cerr << label << " round trip failed: input="
                  << inputDeg.transpose() << " output=" << output.transpose()
                  << " physical error=" << errorDeg << " deg\n";
        return false;
    }
    return true;
}

bool CheckContinuousWorldZRotation(
    const Eigen::Vector3d& taughtDeg,
    int robotType,
    const char* label)
{
    const Eigen::Matrix3d taughtRotation =
        RobotPoseTransform::RotationFromAnglesDeg(
            taughtDeg.x(), taughtDeg.y(), taughtDeg.z(), robotType);
    for (double direction : { -1.0, 1.0 })
    {
        Eigen::Vector3d previousEuler = taughtDeg;
        Eigen::Matrix3d previousRotation = taughtRotation;
        for (double magnitudeDeg = 0.5;
             magnitudeDeg <= 90.0 + 1e-9;
             magnitudeDeg += 0.5)
        {
            const double deltaDeg = direction * magnitudeDeg;
            const Eigen::Matrix3d expected =
                RobotPoseTransform::RotZDeg(deltaDeg) * taughtRotation;
            const Eigen::Vector3d output =
                RobotPoseTransform::AnglesFromRotationDegNear(
                    expected, robotType, previousEuler);
            const Eigen::Matrix3d reconstructed =
                RobotPoseTransform::RotationFromAnglesDeg(
                    output.x(), output.y(), output.z(), robotType);
            const double roundTripErrorDeg =
                RotationDistanceDeg(expected, reconstructed);
            const double stepRotationDeg =
                RotationDistanceDeg(previousRotation, reconstructed);
            if (!std::isfinite(roundTripErrorDeg)
                || roundTripErrorDeg > 2e-6
                || stepRotationDeg > 0.5001)
            {
                std::cerr << label << " continuity failed at delta=" << deltaDeg
                          << " output=" << output.transpose()
                          << " roundTrip=" << roundTripErrorDeg
                          << " stepRotation=" << stepRotationDeg << "\n";
                return false;
            }
            previousEuler = output;
            previousRotation = reconstructed;
        }
    }
    return true;
}
}

int main()
{
    bool ok = true;
    const std::array<Eigen::Vector3d, 6> representativePoses = {
        Eigen::Vector3d(110.708088, 80.519879, 64.364493),
        Eigen::Vector3d(-180.0, 7.0, 20.0),
        Eigen::Vector3d(-180.0, 30.0, 10.5),
        Eigen::Vector3d(0.0, 30.0, 0.0),
        Eigen::Vector3d(45.0, -89.999, 170.0),
        Eigen::Vector3d(-120.0, 89.999, -175.0)
    };

    for (int robotType : { ROBOT_TYPE_STEP, ROBOT_TYPE_FANUC })
    {
        for (const Eigen::Vector3d& pose : representativePoses)
        {
            ok = CheckRoundTrip(pose, robotType, pose, "representative") && ok;
            ok = CheckContinuousWorldZRotation(
                pose,
                robotType,
                robotType == ROBOT_TYPE_STEP ? "STEP" : "FANUC") && ok;
        }
    }

    const Eigen::Vector3d fieldTaught(110.708088, 80.519879, 64.364493);
    const Eigen::Matrix3d fieldRotation =
        RobotPoseTransform::RotationFromAnglesDeg(
            fieldTaught.x(),
            fieldTaught.y(),
            fieldTaught.z(),
            ROBOT_TYPE_STEP);
    const Eigen::Vector3d unchanged =
        RobotPoseTransform::AnglesFromRotationDegNear(
            fieldRotation,
            ROBOT_TYPE_STEP,
            fieldTaught);
    if (!Near(unchanged.x(), fieldTaught.x())
        || !Near(unchanged.y(), fieldTaught.y())
        || !Near(unchanged.z(), fieldTaught.z()))
    {
        std::cerr << "platform pose changed: " << unchanged.transpose() << "\n";
        ok = false;
    }

    const Eigen::Matrix3d worldRotated =
        RobotPoseTransform::RotZDeg(-30.0) * fieldRotation;
    const Eigen::Matrix3d oldRzOnly =
        RobotPoseTransform::RotationFromAnglesDeg(
            fieldTaught.x(),
            fieldTaught.y(),
            fieldTaught.z() + 30.0,
            ROBOT_TYPE_STEP);
    const double oldVsToolFrameDeg =
        RotationDistanceDeg(worldRotated, oldRzOnly);
    if (oldVsToolFrameDeg < 1.0)
    {
        std::cerr << "old RZ-only posture unexpectedly matches tool-frame posture: "
                  << oldVsToolFrameDeg << " deg\n";
        ok = false;
    }

    if (!ok)
    {
        return 1;
    }
    std::cout << "robot pose transform tests passed; field old/new physical difference="
              << oldVsToolFrameDeg << " deg\n";
    return 0;
}
