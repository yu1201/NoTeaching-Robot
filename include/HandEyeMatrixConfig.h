#pragma once

#include <QString>

#include <Eigen/Dense>

// ===== 数据结构 =====

struct HandEyeMatrixConfig
{
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();
};

// ===== 默认值 / 文件路径 =====

QString GetMeasureCameraSectionName(const QString& robotName);
QString GetHandEyeMatrixIniPath(const QString& robotName, const QString& cameraSection);
HandEyeMatrixConfig GetDefaultHandEyeMatrixConfig();

// ===== 文件读写 =====

bool EnsureHandEyeMatrixIni(const QString& robotName, const QString& cameraSection, QString* error = nullptr, QString* filePathOut = nullptr);
bool LoadHandEyeMatrixConfig(const QString& robotName, const QString& cameraSection, HandEyeMatrixConfig& config, QString* error = nullptr, QString* filePathOut = nullptr);
bool SaveHandEyeMatrixConfig(const QString& robotName, const QString& cameraSection, const HandEyeMatrixConfig& config, QString* error = nullptr, QString* filePathOut = nullptr);
