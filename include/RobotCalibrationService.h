#pragma once
#include "RobotCalibrationTypes.h"
#include "RobotKinematicsModel.h"
#include <QJsonObject>
#include <QString>
class RobotDriverAdaptor;
namespace RobotCalibrationService
{
QJsonObject EncodeModel(const RobotKinematicsModel::Model& model);
bool DecodeModel(const QJsonObject& json,RobotKinematicsModel::Model& model);
QString LatestReport(const std::string& robot);
}
