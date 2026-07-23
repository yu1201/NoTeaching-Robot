#pragma once

#include "Const.h"
#include "RobotCalculation.h"

#include <QPair>
#include <QString>
#include <QStringList>
#include <QVector>

#include <functional>

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
        // 控制单元 RobotPara.ini 中显式保存的驱动族（STEP/FANUC）与具体机器人型号。
        // robotType 保留配置值，调用方需要将它与当前运行时驱动类型独立核对。
        // unitIndex 只用于本次运行，模型焊接资格必须使用 robotModelId，
        // 不能根据 robotType、IP 或机器人名称猜测具体型号。
        int robotType = -1;
        QString robotModelId;
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
        QString readFps;   // 相机读取帧率(fps)：驱动取帧轮询(间隔=1000/帧率)；空/无效则按默认 100
        QString imageCaptureEnable;  // 扫描时保存相机图像开关：1=开(默认) 0=关
        QString imageCaptureStride;  // 图像抽帧间隔：每 N 张新图像存 1 张，默认 5（约6fps，控制磁盘用量）
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
    static QString MeasureWeldParamPath(const QString& robotName);
    static bool EnsureMeasureWeldParamFile(const QString& robotName, QString* error = nullptr);
    static QString MeasureWeldScanSectionName(int groupIndex);
    static QString MeasureWeldWeldSectionName(int groupIndex);
    static int MeasureWeldCurrentGroupIndex(const QString& robotName, QString* error = nullptr);
    static QString MeasureWeldCurrentScanSectionName(const QString& robotName, QString* error = nullptr);
    static QString MeasureWeldCurrentWeldSectionName(const QString& robotName, QString* error = nullptr);
    // 测量焊接参数页当前启用组的实际焊道点间距；工艺里还没存过该字段时作预填/回退值（默认 4mm）。
    static double ReadActiveFinalWeldStepFallbackMm(const QString& robotName);
    // 测量焊接参数里的旧焊接方向；工艺未设置焊接方向时的预填/回退值（1=起点到终点，-1=终点到起点）。
    static int ReadActiveWeldDirectionFallback(const QString& robotName);

    // ===== 激光点文件 =====
    static bool LoadIndexedPoint3DFile(
        const QString& filePath,
        QVector<RobotCalculation::IndexedPoint3D>& points,
        QString* error = nullptr,
        const std::function<bool()>& stopRequested = std::function<bool()>());
    static bool SaveTextFileLines(const QString& filePath, const QStringList& lines, QString* error = nullptr);

    // ===== 底层 ini 读写 =====
    static bool ReadPulse(const QString& filePath, const QString& sectionName, const QString& prefix, T_ANGLE_PULSE& pulse, QString* error = nullptr);
    static bool WritePulse(const QString& filePath, const QString& sectionName, const QString& prefix, const T_ANGLE_PULSE& pulse, QString* error = nullptr);
    static bool ReadCoors(const QString& filePath, const QString& sectionName, const QString& prefix, T_ROBOT_COORS& coors, QString* error = nullptr);
    static bool WriteCoors(const QString& filePath, const QString& sectionName, const QString& prefix, const T_ROBOT_COORS& coors, QString* error = nullptr);
    static bool WriteParamValue(const QString& filePath, const QString& sectionName, const QString& key, const QString& value, QString* error = nullptr);
};
