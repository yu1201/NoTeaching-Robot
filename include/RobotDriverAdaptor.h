#pragma once
#include "Const.h"


#include <string>   // 必须包含，否则无法使用std::string
#include <iostream> // 用于输出string（cout）
#include <sstream>  // 用于string和数字的转换（stringstream）
#include <atomic>
#include <cstdint>
#include <deque>
#include <initializer_list>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>
#include <KDL/frames.hpp>
#include <KDL/chain.hpp>
#include <KDL/chainfksolverpos_recursive.hpp>
#include <KDL/chainiksolverpos_nr.hpp>
#include <KDL/chainiksolvervel_pinv.hpp>
// 引入日志头文件
#include "RobotLog.h"
#include "ConfigSection.h"


enum class RobotDriverFamily
{
    Unknown = 0,
    Fanuc,
    Step,
    Inovance,
};

enum class RobotDriverCapability : std::uint64_t
{
    None = 0,
    PassiveState = 1ULL << 0,
    RobotTimestamp = 1ULL << 1,
    LinearMotion = 1ULL << 2,
    JointMotion = 1ULL << 3,
    ContinuousTrajectory = 1ULL << 4,
    ContinuousJog = 1ULL << 5,
    PauseResume = 1ULL << 6,
    PersistentProgramRecovery = 1ULL << 7,
    OperationModeControl = 1ULL << 8,
    NativeProgramUpload = 1ULL << 9,
    DiagnosticCommand = 1ULL << 10,
    CartesianRegister = 1ULL << 11,
    VerifiedProgramCompletion = 1ULL << 12,
    VerifiedSafeAbort = 1ULL << 13,
    ActualArcWeld = 1ULL << 14,
    ExternalAxis = 1ULL << 15,
    HandEyeProgramSupport = 1ULL << 16,
    OfflineTrajectoryExport = 1ULL << 17,
    ConnectionControl = 1ULL << 18,
    AlarmReset = 1ULL << 19,
    ServoPowerControl = 1ULL << 20,
    ToolDataRead = 1ULL << 21,
    IntegerRegister = 1ULL << 22,
    TeachPendantSpeedControl = 1ULL << 23,
    NativeProgramExecution = 1ULL << 24,
    FtpFileTransfer = 1ULL << 25,
    HandEyeMatrixRead = 1ULL << 26,
    HandEyeSupportProgramInstall = 1ULL << 27,
};

constexpr std::uint64_t RobotDriverCapabilityBit(RobotDriverCapability capability)
{
    return static_cast<std::uint64_t>(capability);
}

constexpr std::uint64_t operator|(RobotDriverCapability left, RobotDriverCapability right)
{
    return RobotDriverCapabilityBit(left) | RobotDriverCapabilityBit(right);
}

enum class RobotMotionState
{
    Unknown = 0,
    Idle,
    Starting,
    Running,
    Paused,
    Completed,
    Interrupted,
    Faulted,
};

struct RobotMotionStatus
{
    RobotMotionState state = RobotMotionState::Unknown;
    int rawCode = -1;
    bool terminalVerified = false;
    std::string detail;
};

enum class RobotOperationMode
{
    Manual = 1,
    Automatic = 2,
    ExternalAutomatic = 3,
    Start = 4,
};

enum class RobotTrajectoryPurpose
{
    ScanDryRun = 0,
    WeldDryRun,
    ActualWeld,
};

enum class RobotPersistentRecoveryStrategy
{
    Unsupported = 0,
    ExactProgramIdentity,
    AbortUnknownCurrentProgram,
};

struct RobotDriverDescriptor
{
    RobotDriverFamily family = RobotDriverFamily::Unknown;
    int typeCode = 0;
    int poseConventionType = ROBOT_TYPE_FANUC;
    std::string typeName = "UNKNOWN";
    std::string displayName = "Unknown robot";
};

struct RobotConnectionEndpoint
{
    std::string host;
    int port = 0;

    bool IsValid() const
    {
        return !host.empty() && port > 0 && port <= 65535;
    }
};

struct RobotControllerFileInfo
{
    std::string name;
    std::string path;
    std::string modifiedTime;
    std::uint64_t size = 0;
    bool isDirectory = false;
};

// 只包含业务显示和文件选择所需的品牌无关信息，不包含 FTP 凭据、控制器协议或底层对象。
struct RobotFileTransferProfile
{
    std::string robotName;
    std::string endpointDisplay;
    std::string defaultRemoteDirectory;
    std::string defaultLocalDirectory;
    std::vector<std::string> localFileFilters;
};

struct RobotProgramInventoryResult
{
    std::string robotName;
    std::string remoteDirectory;
    std::size_t entryCount = 0;
    std::size_t programCount = 0;
};

// 适配层只定义 FTP 文件功能；FtpClient、账号、目录规则和程序格式由品牌驱动接入的底层实现持有。
// 会话可安全移交后台线程，业务层不持有机器人驱动指针或任何 FTP 凭据。
class RobotFileTransferSession
{
public:
    virtual ~RobotFileTransferSession() = default;
    virtual const RobotFileTransferProfile& Profile() const = 0;
    virtual bool ListProgramFiles(
        const std::string& remoteDirectory,
        std::vector<RobotControllerFileInfo>& entries,
        int timeoutMs = 10000) = 0;
    virtual bool UploadProgramFile(
        const std::string& localPath,
        const std::string& remotePath,
        bool replaceExisting = true) = 0;
    virtual bool DownloadProgramFile(
        const std::string& remotePath,
        const std::string& localPath) = 0;
    virtual bool DeleteProgramFile(const std::string& remotePath) = 0;
    virtual bool QueryProgramInventory(
        RobotProgramInventoryResult& result,
        int timeoutMs = 10000) = 0;
    virtual std::string LastError() const = 0;
};

struct RobotTrajectoryHandle
{
    std::string programName;
    std::string localProgramPath;
    std::string localDataPath;
    std::string remoteProgramPath;
    std::string remoteDataPath;
    std::string programContentSha256;
    std::string dataContentSha256;
    std::uint64_t programContentSize = 0;
    std::uint64_t dataContentSize = 0;
    bool prepared = false;
    bool started = false;
};

class RobotDriverAdaptor
{
public:
    RobotDriverAdaptor(std::string sUnitName, RobotLog* pRobotLog);
    virtual ~RobotDriverAdaptor();
    virtual bool InitRobotDriver(std::string strUnitName) = 0;

    // 唯一的业务层机器人契约。品牌、SDK、寄存器、程序格式和原生速度单位
    // 必须由派生驱动在本层以下消化，业务代码不得 dynamic_cast 具体驱动。
    virtual RobotDriverDescriptor DriverDescriptor() const = 0;
    virtual std::uint64_t DriverCapabilities() const = 0;
    virtual RobotConnectionEndpoint ControlEndpoint() const = 0;
    virtual bool Connect() = 0;
    virtual bool Disconnect() = 0;
    virtual RobotFileTransferProfile FileTransferProfile() const = 0;
    virtual std::shared_ptr<RobotFileTransferSession> CreateFileTransferSession(
        std::string* error = nullptr) const = 0;
    const std::string& RobotName() const noexcept;
    const std::string& CustomName() const noexcept;
    int RobotType() const noexcept;
    int ExternalAxleType() const noexcept;
    int RobotAxisCount() const noexcept;
    E_ROBOT_BRAND RobotBrand() const noexcept;
    const T_KINEMATICS& KinematicsParameters() const noexcept;
    const T_AXISUNIT& AxisUnit() const noexcept;
    const T_AXISLIMITANGLE& AxisLimitAngles() const noexcept;
    const T_ROBOT_TOOLS& Tools() const noexcept;
    const T_ROBOT_COORS& FirstTool() const noexcept;
    const T_ANGLE_PULSE& HomePulse() const noexcept;
    void SetConfiguredGunTool(const T_ROBOT_COORS& tool);
    bool HasLogSink() const noexcept;
    void WriteLog(LogColor color, const char* format, ...) const;
    bool Supports(RobotDriverCapability capability) const;
    bool SupportsMask(std::uint64_t requiredMask) const;
    bool SupportsAll(std::initializer_list<RobotDriverCapability> capabilities) const;
    std::string MissingCapabilitiesText(std::uint64_t requiredMask) const;
    std::string MissingCapabilitiesText(
        std::initializer_list<RobotDriverCapability> capabilities) const;
    static const char* CapabilityDisplayName(RobotDriverCapability capability);
    virtual bool ValidateLinearSpeedMmPerMin(double speedMmPerMin, std::string* error = nullptr) const = 0;
    virtual bool MoveLinearMmPerMin(
        const T_ROBOT_COORS& target,
        double speedMmPerMin,
        int externalAxleType,
        const int* configuration = nullptr) = 0;
    virtual bool MoveJointPercent(
        const T_ANGLE_PULSE& target,
        double speedPercent,
        int externalAxleType) = 0;
    virtual RobotMotionStatus ReadMotionStatus() = 0;
    virtual RobotMotionStatus ReadMotionStatusPassive(
        long long* pRobotMs = nullptr,
        long long* pPcRecvMs = nullptr) = 0;
    virtual bool ReserveTrajectory(
        RobotTrajectoryPurpose purpose,
        RobotTrajectoryHandle& handle) = 0;
    // moveInfos 的 MOVL 速度统一为 mm/min；派生驱动负责转换为原生单位。
    virtual bool DownlinkTrajectory(
        const std::vector<T_ROBOT_MOVE_INFO>& moveInfos,
        RobotTrajectoryPurpose purpose,
        RobotTrajectoryHandle& handle) = 0;
    virtual bool ExportTrajectoryProgramFiles(
        const std::vector<T_ROBOT_MOVE_INFO>& moveInfos,
        RobotTrajectoryPurpose purpose,
        const std::string& outputDirectory,
        RobotTrajectoryHandle& handle,
        std::string* error = nullptr) = 0;
    virtual bool StartTrajectory(
        const std::vector<T_ROBOT_MOVE_INFO>& moveInfos,
        RobotTrajectoryPurpose purpose,
        RobotTrajectoryHandle& handle) = 0;
    virtual bool WaitTrajectory(
        const RobotTrajectoryHandle& handle,
        int pollDelayMs,
        int runTimeoutMs,
        RobotMotionStatus* terminalStatus = nullptr) = 0;
    virtual bool GetTrackedMotionIdentity(
        std::string& projectName,
        std::string& programName,
        bool* alreadyStopped = nullptr) = 0;
    virtual bool PauseTrackedMotion(
        const std::string& expectedProgramName,
        int& programLine,
        T_ROBOT_COORS& pausedPose,
        std::string* projectName = nullptr,
        std::string* programName = nullptr) = 0;
    virtual bool ResumeTrackedMotion(
        const std::string& expectedProgramName,
        const T_ROBOT_COORS& checkpointPose,
        double maxPositionDeviationMm,
        double maxAngleDeviationDeg,
        double* positionDeviationMm = nullptr,
        double* angleDeviationDeg = nullptr) = 0;
    virtual RobotPersistentRecoveryStrategy PersistentRecoveryStrategy() const = 0;
    virtual bool AbortPersistedMotion(const std::string& expectedProgramName) = 0;
    virtual bool SetOperationMode(RobotOperationMode mode) = 0;
    virtual bool InitializeAfterConnect(std::string* summary = nullptr) = 0;
    virtual bool ShutdownBeforeDisconnect() = 0;
    virtual void ReloadRuntimeConfiguration() = 0;
    virtual bool PrepareNativeProgramUpload() = 0;
    // 连续直角点动速度为 mm/min，连续关节点动速度为百分比；派生驱动负责原生单位转换。
    virtual bool StartContinuousJog(int moveType, double canonicalSpeed) = 0;
    virtual bool PushContinuousJogPoint(const T_ROBOT_COORS& target, double speedMmPerMin) = 0;
    virtual bool PushContinuousJogPoint(const T_ANGLE_PULSE& target, double speedPercent) = 0;
    virtual void RequestEndContinuousJog() = 0;
    virtual void EndContinuousJog() = 0;
    virtual bool IsContinuousJogRunning() const = 0;
    virtual int UploadNativeProgramSource(
        const std::string& localPath,
        const std::string& remoteDirectory = std::string()) = 0;
    virtual std::string SendDiagnosticCommand(const std::string& command) = 0;
    virtual bool WriteCartesianRegister(
        int index,
        const double pose[8],
        int config[7]) = 0;
    virtual bool RunProgramAndWait(
        const std::string& programName,
        int startTimeoutMs,
        int finishTimeoutMs,
        int pollDelayMs,
        RobotMotionStatus* terminalStatus = nullptr) = 0;
    // 手眼辅助程序的格式、名称、寄存器和完成见证全部封装在品牌驱动内。
    // 安装辅助程序与执行机器人侧矩阵验证是两个独立能力，业务层不得混为一个开关。
    virtual bool InstallHandEyeSupportPrograms(std::string* summary = nullptr) = 0;
    virtual bool RunHandEyeValidation(
        const T_ROBOT_COORS& robotPose,
        T_ROBOT_COORS& robotCalculatedPoint) = 0;

    void LoadRobotKinematicsPara(std::string strRobotName, T_KINEMATICS& tKinematics, T_AXISUNIT& tAxisUnit, T_AXISLIMITANGLE& tAxisLimitAngle);
    void LoadRobotExternalAxlePara(std::string strRobotName);
    int CalculateRobotAxisCountByExternalAxleType(int externalAxleType) const;

    virtual bool RobotKinematics(T_ANGLE_PULSE tRobotPulse, T_ROBOT_COORS tToolCoors, T_ROBOT_COORS& tRobotCoors);
    virtual bool RobotInverseKinematics(T_ROBOT_COORS tRobotCoors, T_ROBOT_COORS tToolCoors, std::vector<T_ANGLE_PULSE>& vtResultPulse);
    bool RunKinematicsSelfTest(const T_ANGLE_PULSE& inputPulse, const T_ROBOT_COORS& toolCoors, T_ANGLE_PULSE* pBestResult = nullptr);


    virtual bool IsConnected() = 0;
    // 后台状态监控线程的"首次连接"钩子：默认空(FANUC 惰性连接无需)；STEP 重写为发起一次连接。
    // 配合 s_connectDriversAtConstruct：GUI 模式构造不连，改由监控线程在后台连，避免连不上拖慢主窗口显示。
    virtual void EnsureConnectionForMonitor() {}
    // 驱动构造时是否同步连接机器人：默认 true(旧行为/CLI 用，保证命令执行时已连)；GUI 启动
    // (main 检测到非 --no-show)置 false → 构造不连、监控线程后台连，主窗口立即可见。
    static std::atomic<bool> s_connectDriversAtConstruct;
    // 纯文件离线 CLI 不需要状态采样，也不允许后台监控线程触发控制器连接。
    static std::atomic<bool> s_startStateMonitorsAtConstruct;
    virtual bool cleanAlarm() = 0;
    virtual bool ServoOn() = 0;
    void ClearLastRobotError();
    void SetLastRobotError(const std::string& error);
    std::string GetLastRobotError() const;
    virtual std::string GetRobotStatusText() = 0;
    virtual std::string GetStateMonitorSourceText() const = 0;
    virtual double GetCurrentPos(int nAxisNo) = 0;
    virtual T_ROBOT_COORS GetCurrentPos() = 0;
    // 严格读取接口：失败与“真实零位”必须可区分，运动规划不得把失败返回的零值当当前位置。
    virtual bool TryGetCurrentPos(T_ROBOT_COORS& pos) = 0;
    virtual double GetCurrentPulse(int nAxisNo) = 0;
    virtual T_ANGLE_PULSE GetCurrentPulse() = 0;
    virtual bool TryGetCurrentPulse(T_ANGLE_PULSE& pulse) = 0;
    virtual T_ROBOT_COORS GetCurrentPosPassive(long long* pRobotMs = nullptr, long long* pPcRecvMs = nullptr) = 0;
    virtual T_ANGLE_PULSE GetCurrentPulsePassive(long long* pRobotMs = nullptr, long long* pPcRecvMs = nullptr) = 0;
    virtual int CheckDonePassive(long long* pRobotMs = nullptr, long long* pPcRecvMs = nullptr) = 0;
    struct StateSnapshot
    {
        std::uint64_t sequence = 0;
        long long robotMs = 0;
        long long pcRecvMs = 0;
        T_ROBOT_COORS pose;
        T_ANGLE_PULSE pulse;
        int done = -1;
        bool valid = false;
    };
    bool StartStateMonitor(int intervalMs = 50);
    void StopStateMonitor();
    bool IsStateMonitorRunning() const;
    void ClearStateMonitorSnapshots();
    bool LatestStateSnapshot(StateSnapshot& snapshot) const;
    std::vector<StateSnapshot> StateSnapshotsBetween(std::uint64_t beginExclusive, std::uint64_t endInclusive) const;
    std::uint64_t StateMonitorMark() const;
    int StateMonitorCachedCount() const;
    // 通用(品牌无关)：把已优化的中心线点位序列(只直线直连)，按 T_WeaveDate
    // (nWeaveType==kWeaveTypeAppPointwise) 沿弧长展开成密集摆动点(正弦/三角/L摆/纵向往复)。
    // 非 pointwise 或点数<2 时原样返回。输出点已关 bHasWeaveParam、dOverlapRel=0(精确过点)。
    // error!=nullptr 时，速度/频率无效等失败原因写入 *error 并返回空。
    static std::vector<T_ROBOT_MOVE_INFO> ExpandMoveInfosByPointwiseWeave(
        const std::vector<T_ROBOT_MOVE_INFO>& centerline, std::string* error = nullptr);
    // 摆动速度补偿：摆动后 TCP 路径变长，按全局 k=Σ摆动点距/Σ中心线点距 把每点运动速度字段
    // (dWeldSpeedMmPerMin 给 STEP/ARCDATA、tSpeed.dSpeed 给 FANUC)乘 k，维持沿焊缝行进速度。
    // maxLinearSpeedMmPerSec>0 时限幅(防超机器人/焊机上限)；info!=nullptr 输出 k 与限幅告警。
    // 未摆动(k≈1)或无法算 k 时原样返回。
    static std::vector<T_ROBOT_MOVE_INFO> ApplyWeaveSpeedCompensation(
        const std::vector<T_ROBOT_MOVE_INFO>& centerline,
        const std::vector<T_ROBOT_MOVE_INFO>& weaveMoveInfo,
        double maxLinearSpeedMmPerSec = 0.0,
        std::string* info = nullptr);
    virtual int CheckDone() = 0;
    // 阻塞等待任务终态；runTimeoutMs 是活动运行预算，必须为有限正值。
    virtual int CheckRobotDone(int nDelayTime = 200, int runTimeoutMs = 1800000) = 0;
    // 必须执行不可恢复的程序中止并稳定回读真实终态；普通暂停不得返回成功。
    virtual bool AbortCurrentProgramSafely() = 0;
    virtual bool SetTpSpeed(int speed) = 0;
    virtual bool GetToolData(int nToolNo, T_ROBOT_COORS& robotToolData) = 0;
    virtual bool TryGetIntVar(int nIndex, int& value, const char* cStrPreFix = "INT") = 0;
    virtual int GetIntVar(int nIndex, const char* cStrPreFix = "INT") = 0;
    virtual bool SetIntVar(int nIndex, int nValue, int score = 2, const char* cStrPreFix = "INT") = 0;
    virtual bool SetIntVar(const char* name, int value, int score = 2) = 0;
    virtual bool SetRealVar(int nIndex, double value, const char* cStrPreFix = "REAL", int score = 1) = 0;
    virtual int GetPosVar(long lPvarIndex, double array[6], int config[7] = { 0 }, int MoveType = POSVAR) = 0;
    // 从机器人变量中读取手眼矩阵。rotation 为行优先 3x3，translation 为 mm 单位平移。
    virtual bool GetHandEyeMatrixVariable(const char* variableName, double rotation[9], double translation[3], std::string* error = nullptr) = 0;
private:
    void CreateKinematicsChain();
    void StateMonitorWorker(int intervalMs);
    void StoreStateSnapshot(const StateSnapshot& snapshot);
    void rotationMatrixToRPY(const KDL::Rotation& rot, double& rx, double& ry, double& rz);
    void CoorsToKDLFrame(const T_ROBOT_COORS& coors, KDL::Frame& frame);
    KDL::Frame CalculateFlangeFrame(const T_ROBOT_COORS& tcp_target, const T_ROBOT_COORS& tool_coors);
    // 单组初始值求解逆解（返回关节角度，度）
    bool SolveSingleIK(const KDL::Chain& chain, const KDL::Frame& flange_frame,
        const std::vector<double>& init_angles_deg, std::vector<double>& joint_angles_deg);

    // 求解所有有效逆解（返回关节角度列表，度）
    std::vector<std::vector<double>> SolveAllValidIK(const KDL::Chain& chain, const KDL::Frame& flange_frame);
    // 5. 筛选有效关节角（在限位范围内）
    bool IsJointAngleValid(const std::vector<double>& joint_angles_deg);
    // 6. 逆解去重（误差<0.1度视为同一解）
    bool IsDuplicateSolution(const std::vector<double>& new_sol, const std::vector<std::vector<double>>& exist_sols);
    // 4. 关节角度→脉冲转换（适配T_ANGLE_PULSE）
    void JointAngleToPulse(const std::vector<double>& joint_angles_deg, T_ANGLE_PULSE& pulse);

protected:
    virtual void PrepareStateMonitor();

//----------------------------------------变量类--------------------------------------------//
protected:
	T_KINEMATICS m_tKinematics;
	T_AXISUNIT m_tAxisUnit;
	T_AXISLIMITANGLE m_tAxisLimitAngle;

	T_ROBOT_TOOLS m_tTools;
	T_ROBOT_COORS m_tFirstTool;							//关节臂一号工具
	T_ANGLE_PULSE m_tHomePulse;							//关节臂非运行状态时的安全位置

	int m_nRobotNo;										//关节臂编号
	std::string m_sRobotName;								//关节臂名称（参数调取，程序内部用）
	std::string m_sCustomName;							//关节臂名称（显示用）
	int m_nRobotType;									//关节臂类型（按工作种类划分）
    int m_nExternalAxleType;                           // 外部轴类型，来自配置库 [ExternalAxle]
    int m_nRobotAxisCount;                             // 机器人轴数，默认 6，外部轴启用后累加
	E_ROBOT_BRAND m_eRobotBrand;						//机器人品牌
	//----------------------------------------KDL运动学部分------------------------------------//
	KDL::Chain m_kinematicsChain;
    //----------------------------------------日志相关----------------------------------------//
	RobotLog* m_pRobotLog; // 日志实例（默认路径：Log/robot_log.txt，开启控制台输出）
    mutable std::mutex m_lastRobotErrorMutex;
    std::string m_sLastRobotError;

private:
    static constexpr std::size_t kStateMonitorMaxFrames = 200;
    mutable std::mutex m_stateMonitorMutex;
    std::deque<StateSnapshot> m_stateMonitorFrames;
    std::thread m_stateMonitorThread;
    std::atomic_bool m_stateMonitorRunning;
    std::uint64_t m_stateMonitorNextSequence;
};
