#include "RobotKinematicsModel.h"

#include <Eigen/Geometry>
#include <atomic>
#include <cmath>
#include <iostream>
#include <random>

namespace
{
Eigen::Matrix4d Link(double x, double z, double rxDeg)
{
    constexpr double pi = 3.14159265358979323846;
    Eigen::Matrix4d value = Eigen::Matrix4d::Identity();
    value.topLeftCorner<3, 3>() =
        Eigen::AngleAxisd(rxDeg * pi / 180., Eigen::Vector3d::UnitX()).toRotationMatrix();
    value(0, 3) = x;
    value(2, 3) = z;
    return value;
}
}

int main()
{
    using namespace RobotKinematicsModel;
    Model truth;
    truth.nominal = {Link(0, 300, 90), Link(350, 0, 0), Link(300, 0, 0),
        Link(0, 120, 90), Link(0, 100, -90), Link(0, 80, 0)};
    truth.geometry.setZero();
    truth.featureGeometry.setZero();
    truth.geometry[0] = .22;
    truth.geometry[7] = -.18;
    truth.geometry[14] = .12;
    truth.geometry[27] = .15;
    truth.featureGeometry = truth.geometry;

    std::mt19937 random(20260907);
    std::uniform_real_distribution<double> angle(-80., 80.);
    std::vector<Sample> training;
    std::vector<Sample> validation;
    for (int i = 0; i < 120; ++i)
    {
        Joint q;
        for (int axis = 0; axis < 6; ++axis) { q[axis] = angle(random); }
        (i < 90 ? training : validation).push_back({q, Forward(truth, q)});
    }
    Model fitted;
    fitted.nominal = truth.nominal;
    std::atomic_bool cancel{false};
    std::string error;
    if (!Fit(training, fitted, cancel, {}, error))
    {
        std::cerr << error << '\n';
        return 1;
    }
    const Metrics metrics = Evaluate(fitted, validation);
    if (metrics.positionMax > .002 || metrics.angleMax > .001)
    {
        std::cerr << "heldout error mm=" << metrics.positionMax
            << " deg=" << metrics.angleMax << '\n';
        return 2;
    }
    Joint expected;
    expected << 15, -20, 25, -10, 5, 18;
    const Eigen::Matrix4d target = Forward(fitted, expected);
    Joint solved = expected + Joint::Constant(.5);
    Eigen::Matrix<double, 6, 2> limits;
    limits.col(0).setConstant(-170);
    limits.col(1).setConstant(170);
    if (!Inverse(fitted, target, solved, limits)
        || Evaluate(fitted, {{solved, target}}).positionMax > 1e-4)
    {
        std::cerr << "inverse closure failed\n";
        return 3;
    }
    cancel.store(true);
    Model cancelled;
    cancelled.nominal = truth.nominal;
    if (Fit(training, cancelled, cancel, {}, error)) { return 4; }
    std::cout << "PASS: persistent kinematics fit, heldout validation and inverse closure\n";
    return 0;
}
