#pragma once
#include "RobotCalibrationTypes.h"

// Equivalent geometry + smooth static-gravity compliance; not factory parameter
// identification or a statement about physical positioning accuracy.
namespace RobotKinematicsModel
{
using Joint = Eigen::Matrix<double, 6, 1>;
struct Sample { Joint q; Eigen::Matrix4d flange; };
struct Model
{
    std::array<Eigen::Matrix4d, 6> nominal;
    Eigen::VectorXd geometry = Eigen::VectorXd::Zero(42);
    Eigen::VectorXd featureGeometry = Eigen::VectorXd::Zero(42);
    std::array<Eigen::MatrixXd, 6> mappings;
    std::array<Eigen::VectorXd, 6> compliance;
};
struct Metrics { double positionMax = 0, positionRms = 0, angleMax = 0; };
Eigen::Matrix4d Forward(const Model& model, const Joint& q);
bool Inverse(const Model& model, const Eigen::Matrix4d& target, Joint& seed,
    const Eigen::Matrix<double, 6, 2>& limits);
bool Fit(const std::vector<Sample>& training, Model& model, std::atomic_bool& cancel,
    const RobotCalibrationProgress& progress, std::string& error);
Metrics Evaluate(const Model& model, const std::vector<Sample>& samples);
bool Rigid(const Eigen::Matrix4d& value);
}
