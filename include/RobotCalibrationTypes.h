#pragma once
#include <Eigen/Dense>
#include <array>
#include <atomic>
#include <functional>
#include <string>
#include <vector>

// Brand-neutral calculation contract. mm, radians in matrices, degrees in joints.
// A calculation must NOT select a tool/work object, log in, acquire a controller
// permit, enable a servo, or issue a motion command.
struct RobotKinematicsProfile
{
    int tool = 0;
    int workobject = 0;
    int load = 0;
};
struct RobotKinematicsReference
{
    std::string identity; // Endpoint + model + firmware + parameter/profile hashes.
    std::string source;
    RobotKinematicsProfile profile;
    std::array<Eigen::Matrix4d, 6> nominal;
    Eigen::Matrix<double, 6, 2> limits;
    Eigen::Matrix4d tool = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d work = Eigen::Matrix4d::Identity();
    Eigen::Matrix<double, 6, 1> current = Eigen::Matrix<double, 6, 1>::Zero();
};
struct RobotKinematicsPoint
{
    Eigen::Matrix<double, 6, 1> joints = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity(); // TCP in explicit work object.
    std::array<int, 4> configuration{};
    std::string request;
    std::string response;
};
struct RobotCalibrationAsset
{
    std::string kind;
    std::string name;
    std::string source;
    std::string fingerprint;
    std::string status; // acquired / candidate / invalid / unsupported / missing
    std::string detail;
    std::string raw; // Bounded non-secret source data, persisted in ConfigStore.
};
struct RobotCalibrationDiscovery
{
    std::string recipeRevision;
    std::string identity;
    std::vector<RobotCalibrationAsset> assets;
};
struct RobotKinematicsOptimizationOptions
{
    RobotKinematicsProfile profile;
    int trainingCount = 240;
    int validationCount = 80;
    double maxPositionErrorMm = 0.05;
    double maxOrientationErrorDeg = 0.01;
    unsigned int seed = 0; // Zero: new random seed; persisted before sampling.
};
struct RobotCalibrationRunResult
{
    bool passed = false;
    std::string runId;
    std::string report;
};
using RobotCalibrationProgress = std::function<void(const std::string&)>;

struct RobotControllerHandEye
{
    int sensorIndex = -1;
    int toolIndex = -1;
    std::string cameraAddress;
    std::string source;
    std::string sourceFingerprint;
    std::string controllerIdentity;
    std::string toolFingerprint;
    Eigen::Matrix4d cameraToTool = Eigen::Matrix4d::Identity();
    Eigen::Matrix<double,6,1> reportedError = Eigen::Matrix<double,6,1>::Zero();
    double sampleResidualMaxMm = 0;
    std::string evidence;
};
