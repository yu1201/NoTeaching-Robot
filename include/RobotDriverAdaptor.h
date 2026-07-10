#pragma once
#include "Const.h"


#include <string>   // 必须包含，否则无法使用std::string
#include <iostream> // 用于输出string（cout）
#include <sstream>  // 用于string和数字的转换（stringstream）
#include <atomic>
#include <cstdint>
#include <deque>
#include <mutex>
#include <thread>
#include <vector>
#include <KDL/frames.hpp>
#include <KDL/chain.hpp>
#include <KDL/chainfksolverpos_recursive.hpp>
#include <KDL/chainiksolverpos_nr.hpp>
#include <KDL/chainiksolvervel_pinv.hpp>
#include "FTPClient.h"

// 引入日志头文件
#include "RobotLog.h"
#include "OPini.h"


class RobotDriverAdaptor
{
public:
    RobotDriverAdaptor(std::string sUnitName, RobotLog* pRobotLog);
    virtual ~RobotDriverAdaptor();
    virtual bool InitRobotDriver(std::string strUnitName);

    void LoadRobotKinematicsPara(std::string strRobotName, T_KINEMATICS& tKinematics, T_AXISUNIT& tAxisUnit, T_AXISLIMITANGLE& tAxisLimitAngle);
    void LoadRobotExternalAxlePara(std::string strRobotName);
    int CalculateRobotAxisCountByExternalAxleType(int externalAxleType) const;

    virtual bool RobotKinematics(T_ANGLE_PULSE tRobotPulse, T_ROBOT_COORS tToolCoors, T_ROBOT_COORS& tRobotCoors);
    virtual bool RobotInverseKinematics(T_ROBOT_COORS tRobotCoors, T_ROBOT_COORS tToolCoors, std::vector<T_ANGLE_PULSE>& vtResultPulse);
    bool RunKinematicsSelfTest(const T_ANGLE_PULSE& inputPulse, const T_ROBOT_COORS& toolCoors, T_ANGLE_PULSE* pBestResult = nullptr);


    virtual bool InitSocket(const char* ip, unsigned short Port, bool ifRecode = false);
    virtual bool CloseSocket();
    virtual bool IsConnected();
    // 后台状态监控线程的"首次连接"钩子：默认空(FANUC 惰性连接无需)；STEP 重写为发起一次连接。
    // 配合 s_connectDriversAtConstruct：GUI 模式构造不连，改由监控线程在后台连，避免连不上拖慢主窗口显示。
    virtual void EnsureConnectionForMonitor() {}
    // 驱动构造时是否同步连接机器人：默认 true(旧行为/CLI 用，保证命令执行时已连)；GUI 启动
    // (main 检测到非 --no-show)置 false → 构造不连、监控线程后台连，主窗口立即可见。
    static std::atomic<bool> s_connectDriversAtConstruct;
    virtual bool cleanAlarm();
    virtual bool ServoOn();
    void ClearLastRobotError();
    void SetLastRobotError(const std::string& error);
    std::string GetLastRobotError() const;
    virtual std::string GetRobotStatusText();
    virtual std::string GetStateMonitorSourceText() const;
    virtual double GetCurrentPos(int nAxisNo);
    virtual T_ROBOT_COORS GetCurrentPos();
    virtual double GetCurrentPulse(int nAxisNo);
    virtual T_ANGLE_PULSE GetCurrentPulse();
    virtual T_ROBOT_COORS GetCurrentPosPassive(long long* pRobotMs = nullptr, long long* pPcRecvMs = nullptr);
    virtual T_ANGLE_PULSE GetCurrentPulsePassive(long long* pRobotMs = nullptr, long long* pPcRecvMs = nullptr);
    virtual int CheckDonePassive(long long* pRobotMs = nullptr, long long* pPcRecvMs = nullptr);
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
    bool LatestStateSnapshot(StateSnapshot& snapshot) const;
    std::vector<StateSnapshot> StateSnapshotsBetween(std::uint64_t beginExclusive, std::uint64_t endInclusive) const;
    std::uint64_t StateMonitorMark() const;
    int StateMonitorCachedCount() const;
    virtual int ContiMoveAny(const std::vector<T_ROBOT_MOVE_INFO>& vtRobotMoveInfo);
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
    virtual int CheckDone();
    virtual int CheckRobotDone(int nDelayTime = 200);
    // 必须执行不可恢复的程序中止并稳定回读真实终态；普通暂停不得返回成功。
    virtual bool AbortCurrentProgramSafely();
    virtual bool CallJob(std::string sJobName);
    virtual int InitFtp();
    virtual int UploadFile(std::string LocalFilePath, std::string RemoteFilePath);
    virtual int DownloadFile(std::string RemoteFilePath, std::string LocalFilePath);
    virtual bool SetTpSpeed(int speed);
    virtual bool GetToolData(int nToolNo, T_ROBOT_COORS& robotToolData);
    virtual int GetIntVar(int nIndex, const char* cStrPreFix = "INT");
    virtual bool SetIntVar(int nIndex, int nValue, int score = 2, const char* cStrPreFix = "INT");
    virtual bool SetIntVar(const char* name, int value, int score = 2);
    virtual bool SetRealVar(int nIndex, double value, const char* cStrPreFix = "REAL", int score = 1);
    virtual int GetPosVar(long lPvarIndex, double array[6], int config[7] = { 0 }, int MoveType = POSVAR);
    // 从机器人变量中读取手眼矩阵。rotation 为行优先 3x3，translation 为 mm 单位平移。
    virtual bool GetHandEyeMatrixVariable(const char* variableName, double rotation[9], double translation[3], std::string* error = nullptr);
    virtual bool MoveByJob(T_ROBOT_COORS tRobotJointCoord, T_ROBOT_MOVE_SPEED tPulseMove, int nExternalAxleType, std::string JobName = "MOVL", int isconfig = 1, int config[7] = { 0 });
    virtual bool MoveByJob(T_ANGLE_PULSE tRobotJointCoord, T_ROBOT_MOVE_SPEED tPulseMove, int nExternalAxleType, std::string JobName = "MOVJ");
    virtual bool MoveByJob(double* dRobotJointCoord, T_ROBOT_MOVE_SPEED tPulseMove, int nExternalAxleType, int nPVarType = PULSEVAR, std::string JobName = "MOVJ", int config[7] = { 0 });

private:
    void CreateFanucChain();
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
public:
	T_KINEMATICS m_tKinematics;
	T_AXISUNIT m_tAxisUnit;
	T_AXISLIMITANGLE m_tAxisLimitAngle;

	T_ROBOT_TOOLS m_tTools;
	T_ROBOT_COORS m_tFirstTool;							//关节臂一号工具
	T_ANGLE_PULSE m_tHomePulse;							//关节臂非运行状态时的安全位置

	int m_nRobotNo;										//关节臂编号
	std::string m_sRobotName;								//关节臂名称（参数调取，程序内部用）
	std::string m_sCustomName;							//关节臂名称（显示用）
    std::string m_sSocketIP;
    std::string m_sFTPIP;
    int m_nFTPPort;
    std::string m_sFTPUser;
    std::string m_sFTPPassWord;
    // 未成功加载配置时保持无效端口，避免租约端点身份或连接代码读取未初始化值。
    int m_nSocketPort = 0;
	int m_nRobotType;									//关节臂类型（按工作种类划分）
    int m_nExternalAxleType;                           // 外部轴类型，来自配置库 [ExternalAxle]
    int m_nRobotAxisCount;                             // 机器人轴数，默认 6，外部轴启用后累加
	E_ROBOT_BRAND m_eRobotBrand;						//机器人品牌
	//----------------------------------------KDL运动学部分------------------------------------//
	KDL::Chain m_pFanucChain;
    //----------------------------------------日志相关----------------------------------------//
	RobotLog* m_pRobotLog; // 日志实例（默认路径：Log/robot_log.txt，开启控制台输出）
    FtpClient* m_pFTP;
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
