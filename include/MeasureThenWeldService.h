#pragma once

#include "MeasureThenWeldDialog.h"
#include "RobotCalculation.h"

#include <functional>
#include <vector>

class RobotDriverAdaptor;
class CameraFrameCache;

class MeasureThenWeldService
{
public:
    using LogCallback = std::function<void(const QString&)>;
    using StepCallback = std::function<void(const QString&)>;
    using CheckpointCallback = std::function<bool(const QString&, const QString&)>;

    bool LoadPresetParam(RobotDriverAdaptor* pRobotDriver, T_PRECISE_MEASURE_PARAM& param, QString& error) const;
    bool MovePulseAndWait(RobotDriverAdaptor* pRobotDriver, const T_ANGLE_PULSE& pulse, double speed, const QString& name, const LogCallback& appendLog, const StepCallback& setFlowStep) const;
    bool MovePulseListAndWait(RobotDriverAdaptor* pRobotDriver, const std::vector<T_ANGLE_PULSE>& pulses, double speed, const QString& name, const LogCallback& appendLog, const StepCallback& setFlowStep) const;
    bool MoveCoorsAndWait(RobotDriverAdaptor* pRobotDriver, const T_ROBOT_COORS& coors, double speed, const QString& name, const LogCallback& appendLog, const StepCallback& setFlowStep) const;
    bool MoveScanStartSafeAndWait(RobotDriverAdaptor* pRobotDriver, const T_PRECISE_MEASURE_PARAM& param, double speed, const LogCallback& appendLog, const StepCallback& setFlowStep, const CheckpointCallback& checkpoint) const;
    bool MoveScanEndSafeAndWait(RobotDriverAdaptor* pRobotDriver, const T_PRECISE_MEASURE_PARAM& param, double speed, const LogCallback& appendLog, const StepCallback& setFlowStep) const;
    bool ScanMoveAndCollect(RobotDriverAdaptor* pRobotDriver, const T_PRECISE_MEASURE_PARAM& param, QString& savedPath, const LogCallback& appendLog, const StepCallback& setFlowStep, CameraFrameCache* cameraCache) const;
    bool RebuildWeldFilesFromLaserDir(
        const T_PRECISE_MEASURE_PARAM& param,
        const QString& laserDir,
        QString& preservePath,
        QString& weldPosePath,
        QString& seamCompPath,
        QString& summary,
        QString& error,
        const LogCallback& appendLog = LogCallback(),
        const StepCallback& setFlowStep = StepCallback()) const;

    QString BuildResultDir(const std::string& robotName) const;
    bool SaveTextLines(const QString& filePath, const std::vector<QString>& lines, QString& error) const;
    bool ApplyWeldSeamCompToPoseFile(
        const QString& robotName,
        const QString& inputPath,
        const QString& outputPath,
        QString& summary,
        QString& error) const;
    bool GenerateStepWeldProgramFiles(
        const QString& robotName,
        const QString& poseFilePath,
        const QString& outputDir,
        bool actualWeld,
        double weldSpeedMmPerMin,
        QString& programName,
        QString& srpPath,
        QString& srdPath,
        QString& summary,
        QString& error) const;
    bool DownlinkWeldPoseFile(
        RobotDriverAdaptor* pRobotDriver,
        const QString& poseFilePath,
        double linearSpeedConfigMmPerMin,
        QString& summary,
        QString& error) const;
    bool ExecuteWeldPoseFileWithSafePos(
        RobotDriverAdaptor* pRobotDriver,
        const QString& poseFilePath,
        const T_PRECISE_MEASURE_PARAM& param,
        QString& summary,
        QString& error,
        T_ROBOT_COORS* pStartSafeCoors = nullptr,
        T_ROBOT_COORS* pEndSafeCoors = nullptr,
        const LogCallback& appendLog = LogCallback(),
        const StepCallback& setFlowStep = StepCallback(),
        const CheckpointCallback& checkpoint = CheckpointCallback()) const;
    bool ReadPulse(COPini& ini, const std::string& prefix, T_ANGLE_PULSE& pulse, QString& error) const;
    bool ReadCoors(COPini& ini, const std::string& prefix, T_ROBOT_COORS& coors, QString& error) const;
    bool ReadPulseList(COPini& ini, const std::string& countKey, const std::string& prefix, std::vector<T_ANGLE_PULSE>& pulses, QString& error) const;

    // ===== 补偿前后焊道可视化预览（供 WeldSeamCompDialog 实时对比，单一事实源复用真实补偿数学）=====
    enum class CompPreviewKind
    {
        Seam,   // 焊道补偿：Z向 / 枪反向 / 焊道方向（整段刚性平移）
        Pose,   // 姿态补偿：compX/Y/Z 在工具系按该点姿态旋到世界后叠加
        Corner  // 拐点补偿：上升/下降边内外拐点沿切线移动
    };

    struct CompPreviewPoint
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double rx = 0.0;       // 姿态补偿需要每点输出姿态做工具系旋转
        double ry = 0.0;
        double rz = 0.0;
        int typeCode = 5;      // 分类码 1=start 2=end 3=inner_corner 4=outer_corner 5=normal 6=noise（拐点补偿用）
        QString segmentKind;   // low_platform/rising_edge/high_platform/falling_edge（可能带 _transition/_arc）
        QString pointType;
    };

    // 对话框当前编辑的 4 段补偿值，段下标硬映射：0=低平台 1=上升边 2=高平台 3=下降边。
    struct CompPreviewEditValues
    {
        // 焊道补偿(Seam)
        double weldZComp[4] = { 0.0, 0.0, 0.0, 0.0 };
        double weldGunDirComp[4] = { 0.0, 0.0, 0.0, 0.0 };
        double weldSeamDirComp[4] = { 0.0, 0.0, 0.0, 0.0 };
        // 姿态补偿(Pose)
        double poseRx[4] = { 0.0, 0.0, 0.0, 0.0 };
        double poseRy[4] = { 0.0, 0.0, 0.0, 0.0 };
        double poseRz[4] = { 0.0, 0.0, 0.0, 0.0 };
        double compX[4] = { 0.0, 0.0, 0.0, 0.0 };
        double compY[4] = { 0.0, 0.0, 0.0, 0.0 };
        double compZ[4] = { 0.0, 0.0, 0.0, 0.0 };
        int poseMatchMode = 0;            // 0=按姿态匹配 1=按段属性
        double poseMatchMaxErrorDeg = 5.0;// 按姿态匹配时的最大角度误差
        int robotType = ROBOT_TYPE_FANUC; // 品牌旋转合成（FANUC=Rz·Ry·Rx / STEP 反序）
        // 拐点补偿(Corner)
        bool cornerEnabled = false;
        double risingInnerToOuter = 0.0;
        double risingInnerToInner = 0.0;
        double risingOuterToOuter = 0.0;
        double risingOuterToInner = 0.0;
        double fallingInnerToOuter = 0.0;
        double fallingInnerToInner = 0.0;
        double fallingOuterToOuter = 0.0;
        double fallingOuterToInner = 0.0;
    };

    // 一根方向箭头（世界坐标），由 service 按补偿类型产出，对话框只负责渲染。
    struct CompPreviewArrow
    {
        double origin[3] = { 0.0, 0.0, 0.0 };
        double vector[3] = { 0.0, 0.0, 0.0 };  // 已含长度
        QString label;
        // 0=Z向蓝 1=枪反向黄 2=焊道方向绿 3=工具X红 4=工具Y绿 5=工具Z蓝 6=拐点位移橙
        int colorId = 0;
        bool doubleHeaded = true;
    };

    struct CompPreviewResult
    {
        bool ok = false;
        QString error;
        QVector<CompPreviewPoint> before;   // 补偿前焊道
        QVector<CompPreviewPoint> after;    // 补偿后焊道（按当前编辑值实时算出）
        QVector<CompPreviewArrow> arrows;   // 正负影响方向箭头（按补偿类型）
        double seamAxis[3] = { 0.0, 0.0, 0.0 };  // 焊道方向（世界单位向量）
        double gunAxis[3] = { 0.0, 0.0, 0.0 };   // 枪反向（垂直 Z 与焊道方向）
    };

    // 从扫描结果目录读取补偿前基准焊道（Seam/Pose 读 _WeldPose_2mm.txt；Corner 读 _KeyPoints.txt）。
    bool LoadCompPreviewBaseline(CompPreviewKind kind, const QString& laserDir, QVector<CompPreviewPoint>& baseline, QString& error) const;
    // 按当前编辑的补偿值，对基准焊道实时重算补偿后焊道（零复刻，复用管线真实补偿数学）。
    CompPreviewResult RecomputeCompPreview(CompPreviewKind kind, const QString& robotName, const QVector<CompPreviewPoint>& baseline, const CompPreviewEditValues& edits) const;
    // 读取原始焊道（分类后几何 _Classified.txt）作为对照图层。
    bool LoadCompPreviewOriginalTrack(const QString& laserDir, QVector<CompPreviewPoint>& points, QString& error) const;

private:
    static double SafeSpeed(double value, double fallback);
};
