#include "RobotCalculation.h"

#include <algorithm>
#include <cmath>

namespace
{
constexpr double kMeasurePi = 3.14159265358979323846;
}

Eigen::Matrix3d RobotCalculation::RotX(double w)
{
    const double c = std::cos(w);
    const double s = std::sin(w);
    return (Eigen::Matrix3d() << 1.0, 0.0, 0.0,
                                  0.0, c, -s,
                                  0.0, s, c).finished();
}

Eigen::Matrix3d RobotCalculation::RotY(double p)
{
    const double c = std::cos(p);
    const double s = std::sin(p);
    return (Eigen::Matrix3d() << c, 0.0, s,
                                  0.0, 1.0, 0.0,
                                  -s, 0.0, c).finished();
}

Eigen::Matrix3d RobotCalculation::RotZ(double r)
{
    const double c = std::cos(r);
    const double s = std::sin(r);
    return (Eigen::Matrix3d() << c, -s, 0.0,
                                  s, c, 0.0,
                                  0.0, 0.0, 1.0).finished();
}

Eigen::Matrix3d RobotCalculation::FanucRot(double w, double p, double r)
{
    return RotZ(r) * RotY(p) * RotX(w);
}

T_ROBOT_COORS RobotCalculation::InterpolateRobotPose(const std::vector<TimestampedRobotPose>& robotSamples, qint64 targetTimestampMs)
{
    if (robotSamples.empty())
    {
        return T_ROBOT_COORS();
    }

    if (targetTimestampMs <= robotSamples.front().timestampMs)
    {
        return robotSamples.front().pose;
    }
    if (targetTimestampMs >= robotSamples.back().timestampMs)
    {
        return robotSamples.back().pose;
    }

    const auto upper = std::lower_bound(robotSamples.begin(), robotSamples.end(), targetTimestampMs,
        [](const TimestampedRobotPose& sample, qint64 timestamp)
        {
            return sample.timestampMs < timestamp;
        });

    if (upper == robotSamples.begin())
    {
        return upper->pose;
    }

    const auto lower = upper - 1;
    const qint64 dt = upper->timestampMs - lower->timestampMs;
    const double ratio = dt > 0 ? static_cast<double>(targetTimestampMs - lower->timestampMs) / static_cast<double>(dt) : 0.0;

    auto lerp = [ratio](double a, double b)
        {
            return a + (b - a) * ratio;
        };

    return T_ROBOT_COORS(
        lerp(lower->pose.dX, upper->pose.dX),
        lerp(lower->pose.dY, upper->pose.dY),
        lerp(lower->pose.dZ, upper->pose.dZ),
        lerp(lower->pose.dRX, upper->pose.dRX),
        lerp(lower->pose.dRY, upper->pose.dRY),
        lerp(lower->pose.dRZ, upper->pose.dRZ),
        lerp(lower->pose.dBX, upper->pose.dBX),
        lerp(lower->pose.dBY, upper->pose.dBY),
        lerp(lower->pose.dBZ, upper->pose.dBZ));
}

Eigen::Vector3d RobotCalculation::CalcLaserPointInRobot(const T_ROBOT_COORS& robotPose,
    const Eigen::Vector3d& cameraPoint,
    const HandEyeMatrixConfig& calibration)
{
    const double w = robotPose.dRX * kMeasurePi / 180.0;
    const double p = robotPose.dRY * kMeasurePi / 180.0;
    const double r = robotPose.dRZ * kMeasurePi / 180.0;
    const Eigen::Matrix3d robotRotation = FanucRot(w, p, r);
    return robotRotation * (calibration.rotation * cameraPoint + calibration.translation)
        + Eigen::Vector3d(robotPose.dX, robotPose.dY, robotPose.dZ);
}

QString RobotCalculation::RobotPoseCsv(qint64 timestampMs, const T_ROBOT_COORS& pose)
{
    return QString("%1,%2,%3,%4,%5,%6,%7,%8,%9,%10")
        .arg(QString::number(timestampMs))
        .arg(pose.dX, 0, 'f', 6)
        .arg(pose.dY, 0, 'f', 6)
        .arg(pose.dZ, 0, 'f', 6)
        .arg(pose.dRX, 0, 'f', 6)
        .arg(pose.dRY, 0, 'f', 6)
        .arg(pose.dRZ, 0, 'f', 6)
        .arg(pose.dBX, 0, 'f', 6)
        .arg(pose.dBY, 0, 'f', 6)
        .arg(pose.dBZ, 0, 'f', 6);
}

QString RobotCalculation::Vector3Csv(qint64 timestampMs, const Eigen::Vector3d& point, const QString& extra)
{
    QString line = QString("%1,%2,%3,%4")
        .arg(QString::number(timestampMs))
        .arg(point.x(), 0, 'f', 6)
        .arg(point.y(), 0, 'f', 6)
        .arg(point.z(), 0, 'f', 6);
    if (!extra.isEmpty())
    {
        line += "," + extra;
    }
    return line;
}

QString RobotCalculation::RobotPoseIndexedCsv(int index, const T_ROBOT_COORS& pose)
{
    return QString("%1,%2,%3,%4,%5,%6,%7,%8,%9,%10")
        .arg(index)
        .arg(pose.dX, 0, 'f', 6)
        .arg(pose.dY, 0, 'f', 6)
        .arg(pose.dZ, 0, 'f', 6)
        .arg(pose.dRX, 0, 'f', 6)
        .arg(pose.dRY, 0, 'f', 6)
        .arg(pose.dRZ, 0, 'f', 6)
        .arg(pose.dBX, 0, 'f', 6)
        .arg(pose.dBY, 0, 'f', 6)
        .arg(pose.dBZ, 0, 'f', 6);
}

QString RobotCalculation::Vector3IndexedCsv(int index, const Eigen::Vector3d& point, const QString& extra)
{
    QString line = QString("%1,%2,%3,%4")
        .arg(index)
        .arg(point.x(), 0, 'f', 6)
        .arg(point.y(), 0, 'f', 6)
        .arg(point.z(), 0, 'f', 6);
    if (!extra.isEmpty())
    {
        line += "," + extra;
    }
    return line;
}
