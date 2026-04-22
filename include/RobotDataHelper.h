#pragma once

#include "Const.h"
#include "RobotCalculation.h"

#include <QPair>
#include <QString>
#include <QStringList>
#include <QVector>

class ContralUnit;
class RobotDriverAdaptor;

class RobotDataHelper
{
public:
    // ===== 基础数据结构 =====
    struct RobotInfo
    {
        int unitIndex = -1;
        QString robotName;
        QString displayName;
    };

    struct CameraInfo
    {
        QString sectionName;
        QString displayName;
    };

    struct CameraParamData
    {
        QString sectionName = "CAMERA0";
        QString deviceAddress;
        QString devicePort;
        QString exposureTime;
        QString gainLevel;
        QString cameraType;
    };

    // ===== 工程路径 =====
    static QString FindProjectRootPath();
    static QString FindProjectFilePath(const QString& relativePath);
    static QString BuildProjectPath(const QString& relativePath);

    // ===== 机器人列表 / 驱动 =====
    static QVector<RobotInfo> LoadRobotList(ContralUnit* pContralUnit);
    static RobotDriverAdaptor* GetRobotDriver(ContralUnit* pContralUnit, int unitIndex);

    // ===== 相机参数 =====
    static QString CameraParamPath(const QString& robotName);
    static QVector<CameraInfo> LoadCameraList(const QString& robotName, int* pSelectedIndex = nullptr);
    static QString MeasureCameraSection(const QString& robotName);
    static bool LoadCameraParam(const QString& robotName, const QString& cameraSection, CameraParamData& param, QString* error = nullptr);
    static bool SaveCameraParam(const QString& robotName, const CameraParamData& param, QString* error = nullptr);

    // ===== 精测量参数 =====
    static QString PreciseMeasureParamPath(const QString& robotName);
    static QString PreciseMeasureSectionName(const QString& robotName, QString* error = nullptr);
    static bool ReadPrecisePulse(const QString& robotName, const QString& prefix, T_ANGLE_PULSE& pulse, QString* error = nullptr);
    static bool WritePrecisePulse(const QString& robotName, const QString& prefix, const T_ANGLE_PULSE& pulse, QString* error = nullptr);
    static bool ReadPreciseCoors(const QString& robotName, const QString& prefix, T_ROBOT_COORS& coors, QString* error = nullptr);
    static bool WritePreciseCoors(const QString& robotName, const QString& prefix, const T_ROBOT_COORS& coors, QString* error = nullptr);
    static bool WritePreciseParamValue(const QString& robotName, const QString& key, const QString& value, QString* error = nullptr);

    // ===== 激光点文件 =====
    static bool LoadIndexedPoint3DFile(const QString& filePath, QVector<RobotCalculation::IndexedPoint3D>& points, QString* error = nullptr);
    static bool SaveTextFileLines(const QString& filePath, const QStringList& lines, QString* error = nullptr);

    // ===== 底层 ini 读写 =====
    static bool ReadPulse(const QString& filePath, const QString& sectionName, const QString& prefix, T_ANGLE_PULSE& pulse, QString* error = nullptr);
    static bool WritePulse(const QString& filePath, const QString& sectionName, const QString& prefix, const T_ANGLE_PULSE& pulse, QString* error = nullptr);
    static bool ReadCoors(const QString& filePath, const QString& sectionName, const QString& prefix, T_ROBOT_COORS& coors, QString* error = nullptr);
    static bool WriteCoors(const QString& filePath, const QString& sectionName, const QString& prefix, const T_ROBOT_COORS& coors, QString* error = nullptr);
    static bool WriteParamValue(const QString& filePath, const QString& sectionName, const QString& key, const QString& value, QString* error = nullptr);
};
