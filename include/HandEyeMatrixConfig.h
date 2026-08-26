#pragma once

#include "Const.h"
#include "ConfigSection.h"

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

// ===== 默认值 / 数据库位置 =====

QString GetMeasureCameraSectionName(const QString& robotName);
ConfigLocation GetHandEyeMatrixLocation(const QString& robotName, const QString& cameraSection);
ConfigLocation GetHandEyeCalibrationLocation(const QString& robotName, const QString& cameraSection);
QString GetHandEyeStorageLabel(const ConfigLocation& location);
HandEyeMatrixConfig GetDefaultHandEyeMatrixConfig();
HandEyeCalibrationConfig GetDefaultHandEyeCalibrationConfig();

// ===== 数据库读写 =====

bool EnsureHandEyeMatrixConfig(const QString& robotName, const QString& cameraSection, QString* error = nullptr, QString* storageLabelOut = nullptr);
bool LoadHandEyeMatrixConfig(const QString& robotName, const QString& cameraSection, HandEyeMatrixConfig& config, QString* error = nullptr, QString* storageLabelOut = nullptr);
// 扫描/运动专用严格加载：不自动创建默认矩阵，要求显式就绪标记并校验旋转矩阵有效性。
bool LoadExistingValidatedHandEyeMatrixConfig(const QString& robotName, const QString& cameraSection, HandEyeMatrixConfig& config, QString* error = nullptr, QString* storageLabelOut = nullptr);
bool SaveHandEyeMatrixConfig(const QString& robotName, const QString& cameraSection, const HandEyeMatrixConfig& config, QString* error = nullptr, QString* storageLabelOut = nullptr);
bool EnsureHandEyeCalibrationConfig(const QString& robotName, const QString& cameraSection, QString* error = nullptr, QString* storageLabelOut = nullptr);
bool LoadHandEyeCalibrationConfig(const QString& robotName, const QString& cameraSection, HandEyeCalibrationConfig& config, QString* error = nullptr, QString* storageLabelOut = nullptr);
bool SaveHandEyeCalibrationConfig(const QString& robotName, const QString& cameraSection, const HandEyeCalibrationConfig& config, QString* error = nullptr, QString* storageLabelOut = nullptr);
bool ComputeHandEyeMatrixFromCalibration(const HandEyeCalibrationConfig& calibration, HandEyeMatrixConfig& config, QString* error = nullptr);
bool ComputeHandEyeMatrixFromCalibration(const QString& robotName, const HandEyeCalibrationConfig& calibration, HandEyeMatrixConfig& config, QString* error = nullptr);
bool ComputeHandEyeMatrixFromCalibration(int robotType, const HandEyeCalibrationConfig& calibration, HandEyeMatrixConfig& config, QString* error = nullptr);
