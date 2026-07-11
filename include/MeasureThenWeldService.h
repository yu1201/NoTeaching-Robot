#pragma once

#include "MeasureThenWeldDialog.h"
#include "PointCloudProcessingConfig.h"
#include "RobotCalculation.h"

#include <functional>
#include <vector>

class RobotDriverAdaptor;
class CameraFrameCache;

class MeasureThenWeldService
{
public:
    enum class WeldPoseSource
    {
        PointCloudProduction = 0,
        SyntheticVirtualTest = 1
    };

    using LogCallback = std::function<void(const QString&)>;
    using StepCallback = std::function<void(const QString&)>;
    using CheckpointCallback = std::function<bool(const QString&, const QString&)>;
    using BeforeActionCallback = std::function<bool(const QString&)>;
    using StopRequestedCallback = std::function<bool()>;

    struct WeldExecutionIdentity
    {
        QString sourcePosePath;
        QString sourcePoseSha256;
        qint64 sourcePoseSize = -1;
        QString qualityProofPosePath;
        QString qualityProofPoseSha256;
        qint64 qualityProofPoseSize = -1;
        QString sampledPosePath;
        QString sampledPoseSha256;
        qint64 sampledPoseSize = -1;
        QString programName;
        QString localProgramPath;
        int sampledPointCount = 0;
        double effectiveFinalStepMm = 0.0;
        QString parameterFingerprint;
        bool trajectoryInExecutionOrder = true;
        bool resumeCheckpointSupported = true;
        QString resumeUnsupportedReason;
    };
    using WeldExecutionPreparedCallback =
        std::function<bool(const WeldExecutionIdentity&, QString&)>;
    // 续焊等高风险入口可在任何机器人运动前复核冻结身份；此时程序名可能尚未生成。
    using WeldExecutionPreMotionCallback =
        std::function<bool(const WeldExecutionIdentity&, QString&)>;
    // executionPrepared 成功后，无论启动失败、流程取消还是程序正常完成都恰好回调一次；
    // true 表示焊接程序已确认完成，false 表示尚未完成便退出。调用方据此及时关闭暂停窗口。
    using WeldExecutionFinishedCallback = std::function<void(bool)>;

    enum class ScanCycleStatus
    {
        Success,
        Stopped,
        Failed
    };

    enum class ScanCyclePhase
    {
        NotStarted,
        AtStartSafe,
        AtScanStart,
        ScanMotionStarted,
        AtScanEnd,
        AtEndSafe
    };

    struct ScanMotionProgress
    {
        bool commandAccepted = false;
        bool motionStarted = false;
        bool motionCompleted = false;
    };

    struct ScanCycleResult
    {
        ScanCycleStatus status = ScanCycleStatus::Failed;
        ScanCyclePhase lastPhase = ScanCyclePhase::NotStarted;
        QString caseDir;
        QString weldPosePath;
        QString error;
        bool motionFailure = false;
        bool fatalFailure = false;
        bool scanAttempted = false;
        bool scanDataSucceeded = false;
        bool poseGenerated = false;
        bool safelyRetracted = false;
        bool stopRequestedDuringCycle = false;
    };

    // 启动任何会产生新焊道身份的完整流程前调用；失败时必须在第一条机器人运动前中止。
    static bool InvalidateStoredWeldResumeCheckpoint(const QString& robotName, QString& error);
    bool LoadPresetParam(RobotDriverAdaptor* pRobotDriver, T_PRECISE_MEASURE_PARAM& param, QString& error) const;
    bool ResolveWeldExecutionParameters(
        const T_PRECISE_MEASURE_PARAM& param,
        double overrideFinalStepMm,
        QString& fingerprint,
        double& effectiveFinalStepMm,
        QString& error,
        bool* resumeCheckpointSupported = nullptr,
        QString* resumeUnsupportedReason = nullptr) const;
    bool MovePulseAndWait(RobotDriverAdaptor* pRobotDriver, const T_ANGLE_PULSE& pulse, double speed, const QString& name, const LogCallback& appendLog, const StepCallback& setFlowStep) const;
    bool MovePulseListAndWait(RobotDriverAdaptor* pRobotDriver, const std::vector<T_ANGLE_PULSE>& pulses, double speed, const QString& name, const LogCallback& appendLog, const StepCallback& setFlowStep) const;
    bool MoveCoorsAndWait(RobotDriverAdaptor* pRobotDriver, const T_ROBOT_COORS& coors, double speed, const QString& name, const LogCallback& appendLog, const StepCallback& setFlowStep) const;
    bool MoveScanStartSafeAndWait(RobotDriverAdaptor* pRobotDriver, const T_PRECISE_MEASURE_PARAM& param, double speed, const LogCallback& appendLog, const StepCallback& setFlowStep, const CheckpointCallback& checkpoint) const;
    bool MoveScanEndSafeAndWait(RobotDriverAdaptor* pRobotDriver, const T_PRECISE_MEASURE_PARAM& param, double speed, const LogCallback& appendLog, const StepCallback& setFlowStep) const;
    bool RunScanCycle(
        RobotDriverAdaptor* pRobotDriver,
        const T_PRECISE_MEASURE_PARAM& param,
        double runSpeed,
        CameraFrameCache* cameraCache,
        ScanCycleResult& result,
        const LogCallback& appendLog,
        const StepCallback& setFlowStep,
        const CheckpointCallback& safetyCheckpoint = CheckpointCallback(),
        const BeforeActionCallback& beforeAction = BeforeActionCallback(),
        const StopRequestedCallback& stopRequested = StopRequestedCallback()) const;
    bool ScanMoveAndCollect(
        RobotDriverAdaptor* pRobotDriver,
        const T_PRECISE_MEASURE_PARAM& param,
        QString& savedPath,
        const LogCallback& appendLog,
        const StepCallback& setFlowStep,
        CameraFrameCache* cameraCache,
        ScanMotionProgress* progress = nullptr,
        const HandEyeMatrixConfig* validatedCalibration = nullptr) const;
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
        QString& error,
        QString* generatedSha256 = nullptr,
        qint64* generatedSize = nullptr) const;
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
        QString& error,
        double overrideFinalStepMm = 0.0,         // >0 时强制覆盖最终轨迹点间距（虚拟焊道测试用）
        bool allowPointwiseWeave = true,
        WeldPoseSource poseSource = WeldPoseSource::PointCloudProduction) const;

    // 调试用：从机器人当前位姿沿 ±Y 造一条干净的虚拟直线焊道，保持当前焊枪姿态，
    // 不走点云拟合/姿态补偿/焊缝补偿/起终裁剪/拐点处理，直接生成 srp/srd（摆动/速度/姿态仍读保存的工艺）。
    // 长度与点间距均可调；点间距=机器人最终逐点执行的轨迹间距（覆盖工艺值，便于测密集点摆动）。
    // 产出的 weldPosePath 既是查看文件，也是下发执行文件（执行不再叠加补偿）。
    bool GenerateVirtualStraightWeldFiles(
        const QString& robotName,
        const T_ROBOT_COORS& startCoors,
        double lengthMm,
        double pointStepMm,
        int directionSign,        // +1 = 基坐标 +Y，-1 = -Y
        bool actualWeld,
        QString& outputDir,       // 入空则自动建 Result/<robot>/VirtualWeld_<时间>；返回解析后的目录
        QString& weldPosePath,
        QString& srpPath,
        QString& srdPath,
        QString& programName,
        QString& summary,
        QString& error,
        const LogCallback& appendLog = LogCallback()) const;
    bool DownlinkWeldPoseFile(
        RobotDriverAdaptor* pRobotDriver,
        const QString& poseFilePath,
        double linearSpeedConfigMmPerMin,
        QString& summary,
        QString& error,
        WeldPoseSource poseSource = WeldPoseSource::PointCloudProduction) const;
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
        const CheckpointCallback& checkpoint = CheckpointCallback(),
        double overrideFinalStepMm = 0.0,         // >0 时强制覆盖最终轨迹点间距（虚拟焊道测试用）
        bool allowPointwiseWeave = true,          // pointwise 自定义摆动默认放行(含先测后焊)；传 false 可禁用(保留钩子)
        WeldPoseSource poseSource = WeldPoseSource::PointCloudProduction,
        double resumeStartArcMm = -1.0,           // 断点续焊：执行顺序轨迹上的精确起始弧长；<0 表示普通全轨迹执行
        bool inputAlreadyInExecutionOrder = false,// V2续焊传实际 FinalSampled 时为 true，禁止再次按 WeldDirection 反转
        const StopRequestedCallback& stopRequested = StopRequestedCallback(),
        const WeldExecutionPreparedCallback& executionPrepared = WeldExecutionPreparedCallback(),
        const WeldExecutionFinishedCallback& executionFinished = WeldExecutionFinishedCallback(),
        const QString& expectedSourceSha256 = QString(),
        const WeldExecutionPreMotionCallback& executionPreMotion = WeldExecutionPreMotionCallback(),
        const QString& qualityProofSourcePosePath = QString()) const;
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
        double bx = 0.0;       // 基坐标/外部轴（焊缝后处理保真需原样保留）
        double by = 0.0;
        double bz = 0.0;
        int weldIndex = 0;     // 原焊接点序（焊缝后处理保真用）
        int rawIndex = 0;      // 原始 raw_index（拐点恢复按它匹配，必须保留以与下发一致）
        int typeCode = 5;      // 分类码 1=start 2=end 3=inner_corner 4=outer_corner 5=normal 6=noise（拐点补偿用）
        QString segmentKind;   // low_platform/rising_edge/high_platform/falling_edge（可能带 _transition/_arc）
        QString pointType;
        bool isLapStep = false;  // 搭接错位台阶端点：预览须带它，否则搭接横移被自交/锐角裁剪裁掉、搭接收敛不触发
    };

    // 对话框当前编辑的 4 段补偿值，段下标硬映射：0=低平台 1=上升边 2=高平台 3=下降边。
    struct CompPreviewEditValues
    {
        // 焊道补偿(Seam)；seamSegmentKind 为各槽位真实段类（来自配置，可能是 CorrugatedPlate），
        // 让预览的槽位匹配/回退与下发 FindSeamCompSlotForRecord 完全一致。
        double weldZComp[4] = { 0.0, 0.0, 0.0, 0.0 };
        double weldGunDirComp[4] = { 0.0, 0.0, 0.0, 0.0 };
        double weldSeamDirComp[4] = { 0.0, 0.0, 0.0, 0.0 };
        QString seamSegmentKind[4];
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
        // 工艺区域试调覆盖（仅预览联动，不落盘）：圆弧过渡与实际焊道点间距
        bool processOverrideValid = false;  // true=用下面三个值覆盖真实工艺
        bool arcEnabled = false;
        double arcRadiusMm = 0.0;
        double processFinalStepMm = 0.0;    // 0=未设→回退测量参数页的值
        bool keepAnchorsOnly = false;       // 精简轨迹：最终抽样只保留特殊点(起终/拐点/段边界/圆弧边界/搭接)，预览"实际焊道"阶段联动
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
    // 四种处理方法各自的基础焊道文件名（处理成功时落盘到 LaserPoint 目录；
    // 文件存在即表示该目录已按该方法完成焊道生成）。
    static QString MethodBaseTrackFileName(PointCloudProcessingConfig::Mode mode);
    // 由精测点云配置构造几何拟合参数的唯一来源（真机流程与 CLI 共用，杜绝两处手抄漂移）。
    // sampleAxisMode=Auto 时采用 fallbackSampleAxis；拐点补偿/调试目录等调用方差异由调用方追加。
    static RobotCalculation::LowerWeldFilterParams BuildTrackFitParamsFromSettings(
        const PointCloudProcessingConfig::Settings& settings,
        RobotCalculation::SampleAxis fallbackSampleAxis);
    // 读取"原始数据"对照图层：当前方法的基础焊道文件；未生成时回退相机目标点轨迹。
    bool LoadCompPreviewRawCloud(
        const QString& laserDir,
        QVector<CompPreviewPoint>& points,
        QString& error,
        QString* sourceDescription = nullptr) const;

    // ===== 五阶段流水线预览：原始数据→原始焊道→姿态补偿→焊道补偿→圆弧过渡 =====
    // 基准 = _WeldPose_2mm.txt（已烘焙扫描时保存的姿态补偿）。
    // 姿态补偿阶段按 delta 计算（当前值−已保存值），当前=已保存时与文件一致；
    // 焊道补偿与圆弧过渡逐级链式计算，复用管线真实数学，与下发 _SeamComp 一致。
    struct CompPreviewStages
    {
        bool ok = false;
        QString error;
        QVector<CompPreviewPoint> poseComp;   // 姿态补偿后（基准 + 姿态补偿增量）
        QVector<CompPreviewPoint> seamComp;   // 焊道补偿后（纯补偿平移）
        QVector<CompPreviewPoint> arc;        // 圆弧过渡后（完整后处理，2mm 稠密执行文件）
        QVector<CompPreviewPoint> actual;     // 实际焊道（按点间距最终抽样 = 机器人逐点执行的轨迹）
        QVector<CompPreviewArrow> arrows;     // 正负方向箭头
    };
    CompPreviewStages ComputeCompPreviewStages(
        const QString& robotName,
        const QVector<CompPreviewPoint>& baseline,
        const CompPreviewEditValues& currentEdits,
        const CompPreviewEditValues& savedEdits,
        bool includePoseArrows) const;

private:
    static double SafeSpeed(double value, double fallback);
};
