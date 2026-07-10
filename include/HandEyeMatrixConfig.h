#pragma once

#include "Const.h"

#include <QString>
#include <QVector>

#include <Eigen/Dense>

// ===== 数据结构 =====

struct HandEyeMatrixConfig
{
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();
    int robotType = ROBOT_TYPE_FANUC;
};

struct HandEyeCalibrationSample
{
    bool valid = false;
    T_ROBOT_COORS robotPose;
    Eigen::Vector3d cameraPoint = Eigen::Vector3d::Zero();
};

struct HandEyeCalibrationConfig
{
    T_ROBOT_COORS tcpPoint;
    QVector<HandEyeCalibrationSample> samples;
};

constexpr int kHandEyeCalibrationSampleCount = 6;

// ===== 默认值 / 文件路径 =====

QString GetMeasureCameraSectionName(const QString& robotName);
QString GetHandEyeMatrixIniPath(const QString& robotName, const QString& cameraSection);
QString GetHandEyeCalibrationIniPath(const QString& robotName, const QString& cameraSection);
HandEyeMatrixConfig GetDefaultHandEyeMatrixConfig();
HandEyeCalibrationConfig GetDefaultHandEyeCalibrationConfig();

// ===== 文件读写 =====

bool EnsureHandEyeMatrixIni(const QString& robotName, const QString& cameraSection, QString* error = nullptr, QString* filePathOut = nullptr);
bool LoadHandEyeMatrixConfig(const QString& robotName, const QString& cameraSection, HandEyeMatrixConfig& config, QString* error = nullptr, QString* filePathOut = nullptr);
// 扫描/运动专用严格加载：不自动创建默认矩阵，要求显式就绪标记并校验旋转矩阵有效性。
bool LoadExistingValidatedHandEyeMatrixConfig(const QString& robotName, const QString& cameraSection, HandEyeMatrixConfig& config, QString* error = nullptr, QString* filePathOut = nullptr);
bool SaveHandEyeMatrixConfig(const QString& robotName, const QString& cameraSection, const HandEyeMatrixConfig& config, QString* error = nullptr, QString* filePathOut = nullptr);
bool EnsureHandEyeCalibrationIni(const QString& robotName, const QString& cameraSection, QString* error = nullptr, QString* filePathOut = nullptr);
bool LoadHandEyeCalibrationConfig(const QString& robotName, const QString& cameraSection, HandEyeCalibrationConfig& config, QString* error = nullptr, QString* filePathOut = nullptr);
bool SaveHandEyeCalibrationConfig(const QString& robotName, const QString& cameraSection, const HandEyeCalibrationConfig& config, QString* error = nullptr, QString* filePathOut = nullptr);
bool ComputeHandEyeMatrixFromCalibration(const HandEyeCalibrationConfig& calibration, HandEyeMatrixConfig& config, QString* error = nullptr);
bool ComputeHandEyeMatrixFromCalibration(const QString& robotName, const HandEyeCalibrationConfig& calibration, HandEyeMatrixConfig& config, QString* error = nullptr);
bool ComputeHandEyeMatrixFromCalibration(int robotType, const HandEyeCalibrationConfig& calibration, HandEyeMatrixConfig& config, QString* error = nullptr);
