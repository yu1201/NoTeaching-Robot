#pragma once

#include "MeasureThenWeldDialog.h"
#include "PointCloudProcessingConfig.h"
#include "RobotCalculation.h"
#include "WeldExecutionSafety.h"

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
        SyntheticVirtualTest = 1,
        ScanPoseVariationDryRun = 2
    };

    using LogCallback = std::function<void(const QString&)>;
    using StepCallback = std::function<void(const QString&)>;
    using CheckpointCallback = std::function<bool(const QString&, const QString&)>;
    using BeforeActionCallback = std::function<bool(const QString&)>;
    using StopRequestedCallback = std::function<bool()>;
    using ScanProgressCallback = std::function<void(double)>;
    using ScanPauseAvailabilityCallback = std::function<void(bool, const QString&)>;
    // 扫描运动及末端采样冻结后、百万级点云后处理前执行。RunScanCycle 用它优先完成安全收枪。
    using ScanMotionCompletedCallback = std::function<bool()>;

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
        T_ROBOT_COORS requiredEndSafePose;
        double safeMoveSpeedMmPerMin = 0.0;
    };

    // 可跨线程按值传递的只读生产绑定。只能在拥有当前机器人驱动的线程中通过
    // CapturePointCloudProductionExpectation 创建；后台重建不得持有/解引用裸驱动指针。
    struct PointCloudProductionExpectation
    {
        QString robotName;
        QString robotEndpoint;
        QString cameraSection;
        QString handEyeSha256;
    };
    using WeldExecutionPreparedCallback =
        std::function<bool(const WeldExecutionIdentity&, QString&)>;
    // 续焊等高风险入口可在任何机器人运动前复核冻结身份；此时程序名可能尚未生成。
    using WeldExecutionPreMotionCallback =
        std::function<bool(const WeldExecutionIdentity&, QString&)>;
    // executionPrepared 成功后报告安全终态。程序完成后先同步报告
    // ProgramCompletedUnretracted 以持久化恢复门禁；只有强制回撤并验证到位后才报告
    // SafelyRetracted。启动前退出或启动结果不确定报告 Incomplete。
    using WeldExecutionFinishedCallback =
        std::function<bool(const WeldExecutionTerminalResult&, QString&)>;

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

    // 单次扫描采集完成后的处理范围。生产先测后焊默认保持 CorrugatedBoard；
    // 调试流程可选择只保存点云，或从相机逐帧特征点生成平滑曲线。
    enum class ScanPostProcessMode
    {
        None = 0,
        FeaturePointSmoothCurve = 1,
        CorrugatedBoard = 2
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

    // 扫描变姿态精度测试使用的四段周期。空间位置始终沿示教起终点直线；
    // 姿态按“下平台(基准) -> 上坡(左转) -> 上平台(基准) -> 下坡(右转)”循环。
    // 左右旋转输入允许 -60~60 deg：正值保持名称所示方向，负值反向；下坡右旋
    // 对应的实际 RotZ 角为 -rightRotationDeg。旋转合成口径为 RotZ(angle) * taughtRotation。
    struct ScanPoseVariationParams
    {
        double lowPlatformLengthMm = 30.0;
        double risingLengthMm = 30.0;
        double highPlatformLengthMm = 30.0;
        double fallingLengthMm = 30.0;
        double leftRotationDeg = 10.0;
        double rightRotationDeg = 10.0;
        double transitionLengthMm = 10.0;
        double pointStepMm = 2.0;
    };

    struct ScanPoseVariationPoint
    {
        int index = 0;
        double distanceMm = 0.0;
        double cycleDistanceMm = 0.0;
        double commandedRotationDeg = 0.0;
        QString phase;
        T_ROBOT_COORS pose;
    };

    // 启动任何会产生新焊道身份的完整流程前调用；失败时必须在第一条机器人运动前中止。
    static bool InvalidateStoredWeldResumeCheckpoint(const QString& robotName, QString& error);
    static bool CapturePointCloudProductionExpectation(
        const RobotDriverAdaptor* pRobotDriver,
        const QString& expectedRobotName,
        PointCloudProductionExpectation& expectation,
        QString& error);
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
    bool VerifyRobotAtSafePose(
        RobotDriverAdaptor* pRobotDriver,
        const T_ROBOT_COORS& expected,
        T_ROBOT_COORS& observed,
        QString& error) const;
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
        const StopRequestedCallback& stopRequested = StopRequestedCallback(),
        const ScanProgressCallback& scanProgress = ScanProgressCallback(),
        const ScanPauseAvailabilityCallback& scanPauseAvailability =
            ScanPauseAvailabilityCallback(),
        const std::vector<T_ROBOT_COORS>* scanTrajectory = nullptr,
        const QString& cameraSectionOverride = QString(),
        ScanPostProcessMode postProcessMode =
            ScanPostProcessMode::CorrugatedBoard,
        const QString& pointCloudSdkLibraryDirOverride = QString()) const;
    bool ScanMoveAndCollect(
        RobotDriverAdaptor* pRobotDriver,
        const T_PRECISE_MEASURE_PARAM& param,
        QString& savedPath,
        const LogCallback& appendLog,
        const StepCallback& setFlowStep,
        CameraFrameCache* cameraCache,
        ScanMotionProgress* progress = nullptr,
        const HandEyeMatrixConfig* validatedCalibration = nullptr,
        const ScanProgressCallback& scanProgress = ScanProgressCallback(),
        const ScanPauseAvailabilityCallback& scanPauseAvailability =
            ScanPauseAvailabilityCallback(),
        const std::vector<T_ROBOT_COORS>* scanTrajectory = nullptr,
        const QString& cameraSectionOverride = QString(),
        const ScanMotionCompletedCallback& motionCompleted =
            ScanMotionCompletedCallback(),
        ScanPostProcessMode postProcessMode =
            ScanPostProcessMode::CorrugatedBoard,
        const QString& pointCloudSdkLibraryDirOverride = QString()) const;
    bool SaveScanPoseVariationTrajectory(
        const QString& filePath,
        const QVector<ScanPoseVariationPoint>& trajectory,
        QString& error) const;
    bool GenerateScanPoseVariationTrajectory(
        const T_ROBOT_COORS& taughtBasePose,
        const T_ROBOT_COORS& taughtStartPose,
        const T_ROBOT_COORS& taughtEndPose,
        int robotType,
        const ScanPoseVariationParams& params,
        QVector<ScanPoseVariationPoint>& trajectory,
        QString& summary,
        QString& error) const;
    bool RebuildWeldFilesFromLaserDir(
        RobotDriverAdaptor* pRobotDriver,
        const T_PRECISE_MEASURE_PARAM& param,
        const QString& laserDir,
        QString& preservePath,
        QString& weldPosePath,
        QString& seamCompPath,
        QString& summary,
        QString& error,
        const LogCallback& appendLog = LogCallback(),
        const StepCallback& setFlowStep = StepCallback(),
        const PointCloudProductionExpectation& productionExpectation =
            PointCloudProductionExpectation(),
        const StopRequestedCallback& stopRequested = StopRequestedCallback()) const;

    QString BuildResultDir(const std::string& robotName) const;
    bool SaveTextLines(
        const QString& filePath,
        const std::vector<QString>& lines,
        QString& error,
        const StopRequestedCallback& stopRequested = StopRequestedCallback()) const;
    bool ApplyWeldSeamCompToPoseFile(
        const QString& robotName,
        const QString& inputPath,
        const QString& outputPath,
        QString& summary,
        QString& error,
        QString* generatedSha256 = nullptr,
        qint64* generatedSize = nullptr,
        const StopRequestedCallback& stopRequested = StopRequestedCallback()) const;
    bool GenerateRobotWeldProgramFiles(
        RobotDriverAdaptor* pRobotDriver,
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
        WeldPoseSource poseSource = WeldPoseSource::PointCloudProduction,
        const PointCloudProductionExpectation& authorizationExpectation =
            PointCloudProductionExpectation(),
        bool allowActiveProofReplacement = false) const;

    // 调试用：从机器人当前位姿沿 ±Y 造一条干净的虚拟直线焊道，保持当前焊枪姿态，
    // 不走点云拟合/姿态补偿/焊缝补偿/起终裁剪/拐点处理，直接通过适配层生成控制器程序文件。
    // 长度与点间距均可调；点间距=机器人最终逐点执行的轨迹间距（覆盖工艺值，便于测密集点摆动）。
    // 产出的 weldPosePath 既是查看文件，也是下发执行文件（执行不再叠加补偿）。
    bool GenerateVirtualStraightWeldFiles(
        RobotDriverAdaptor* pRobotDriver,
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
    // 扫描变姿态测试的“直线处理”结果是基坐标 XYZ 曲线，不含机器人姿态。
    // 本入口把每个曲线点直接作为 Tool1 TCP，统一使用已示教基础姿态，生成固定 2mm
    // 空跑姿态文件及控制器程序；不会起弧、摆动或应用焊道/姿态补偿。
    bool GenerateScanPoseVariationDryRunFiles(
        RobotDriverAdaptor* pRobotDriver,
        const QString& featureCurvePath,
        const T_ROBOT_COORS& basePose,
        double dryRunSpeedMmPerMin,
        T_ROBOT_COORS& curveStartPose,
        T_ROBOT_COORS& curveEndPose,
        QString& posePath,
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
    bool ReadPulse(ConfigSection& ini, const std::string& prefix, T_ANGLE_PULSE& pulse, QString& error) const;
    bool ReadCoors(ConfigSection& ini, const std::string& prefix, T_ROBOT_COORS& coors, QString& error) const;
    bool ReadPulseList(ConfigSection& ini, const std::string& countKey, const std::string& prefix, std::vector<T_ANGLE_PULSE>& pulses, QString& error) const;

    // ===== 补偿前后焊道可视化预览（供 WeldSeamCompDialog 实时对比，单一事实源复用真实补偿数学）=====
    enum class CompPreviewKind
    {
        Seam,   // 焊道补偿：Z向 / 枪反向 / 焊道方向（整段刚性平移）
        Pose,   // 姿态补偿：四类物理段统一使用焊道基准(X切向/Y法向/Z世界Z)
        Corner  // 拐点补偿：上升/下降边内外拐点沿切线移动
    };

    struct CompPreviewPoint
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double rx = 0.0;       // 保留输出姿态，用于按姿态匹配补偿槽和焊枪轨迹预览
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

    // 对话框当前编辑值：焊道补偿整条统一；姿态补偿仍按四段映射。
    struct CompPreviewEditValues
    {
        // 焊道补偿(Seam)：一套三方向数值统一应用于整条焊道。
        double weldZComp = 0.0;
        double weldGunDirComp = 0.0;
        double weldSeamDirComp = 0.0;
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
        int posePreviewSegmentIndex = 0;  // UI 当前选中的姿态段；仅用于方向箭头/提示，不参与生产补偿
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
        // 0=Z向蓝 1=枪反向黄 2=焊道方向绿
        // 3/4/5=当前姿态段补偿方向（切向红/法向绿/世界Z蓝）
        // 6=拐点位移橙 7/8/9=当前焊道补偿方向（世界Z蓝/枪反向黄/焊道方向绿）
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
    bool LoadCompPreviewBaseline(
        CompPreviewKind kind,
        const QString& laserDir,
        QVector<CompPreviewPoint>& baseline,
        QString& error,
        const StopRequestedCallback& stopRequested = StopRequestedCallback()) const;
    // 按当前编辑的补偿值，对基准焊道实时重算补偿后焊道（零复刻，复用管线真实补偿数学）。
    CompPreviewResult RecomputeCompPreview(CompPreviewKind kind, const QString& robotName, const QVector<CompPreviewPoint>& baseline, const CompPreviewEditValues& edits) const;
    // 读取原始焊道（分类后几何 _Classified.txt）作为对照图层。
    bool LoadCompPreviewOriginalTrack(
        const QString& laserDir,
        QVector<CompPreviewPoint>& points,
        QString& error,
        const StopRequestedCallback& stopRequested = StopRequestedCallback()) const;
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
        QString* sourceDescription = nullptr,
        const StopRequestedCallback& stopRequested = StopRequestedCallback()) const;

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
        QVector<CompPreviewArrow> arrows;     // 焊道坐标轴 + 当前模式的实际补偿方向
        int selectedPoseSegmentIndex = -1;
        QString selectedPoseSegmentKind;
        bool selectedPoseSegmentMatched = false;
        bool selectedPoseDirectionValid = false;
        double selectedPoseCompLocal[3] = { 0.0, 0.0, 0.0 };
        double selectedPoseCompWorld[3] = { 0.0, 0.0, 0.0 };
        bool selectedSeamDirectionValid = false;
        // 顺序：世界Z、枪反向、焊道方向。
        double selectedSeamCompLocal[3] = { 0.0, 0.0, 0.0 };
        double selectedSeamCompWorld[3] = { 0.0, 0.0, 0.0 };
    };
    CompPreviewStages ComputeCompPreviewStages(
        const QString& robotName,
        const QVector<CompPreviewPoint>& baseline,
        const CompPreviewEditValues& currentEdits,
        const CompPreviewEditValues& savedEdits,
        bool showPoseSelection,
        const StopRequestedCallback& stopRequested = StopRequestedCallback()) const;

private:
    static double SafeSpeed(double value, double fallback);
};
