#pragma once
#include "Const.h"


#include <string>   // 必须包含，否则无法使用std::string
#include <iostream> // 用于输出string（cout）
#include <sstream>  // 用于string和数字的转换（stringstream）
#include <mutex>
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


    virtual bool InitSocket(const char* ip, u_short Port, bool ifRecode = false);
    virtual bool CloseSocket();
    virtual bool IsConnected();
    virtual bool cleanAlarm();
    virtual bool ServoOn();
    void ClearLastRobotError();
    void SetLastRobotError(const std::string& error);
    std::string GetLastRobotError() const;
    virtual std::string GetRobotStatusText();
    virtual double GetCurrentPos(int nAxisNo);
    virtual T_ROBOT_COORS GetCurrentPos();
    virtual double GetCurrentPulse(int nAxisNo);
    virtual T_ANGLE_PULSE GetCurrentPulse();
    virtual T_ROBOT_COORS GetCurrentPosPassive(long long* pRobotMs = nullptr, long long* pPcRecvMs = nullptr);
    virtual T_ANGLE_PULSE GetCurrentPulsePassive(long long* pRobotMs = nullptr, long long* pPcRecvMs = nullptr);
    virtual int CheckDonePassive(long long* pRobotMs = nullptr, long long* pPcRecvMs = nullptr);
    virtual int ContiMoveAny(const std::vector<T_ROBOT_MOVE_INFO>& vtRobotMoveInfo);
    virtual int CheckDone();
    virtual int CheckRobotDone(int nDelayTime = 200);
    virtual bool CallJob(std::string sJobName);
    virtual int InitFtp();
    virtual int UploadFile(std::string LocalFilePath, std::string RemoteFilePath);
    virtual int DownloadFile(std::string RemoteFilePath, std::string LocalFilePath);
    virtual bool SetTpSpeed(int speed);
    virtual int GetIntVar(int nIndex, const char* cStrPreFix = "INT");
    virtual bool SetIntVar(int nIndex, int nValue, int score = 2, const char* cStrPreFix = "INT");
    virtual bool SetIntVar(const char* name, int value, int score = 2);
    virtual bool SetRealVar(int nIndex, double value, const char* cStrPreFix = "REAL", int score = 1);
    virtual int GetPosVar(long lPvarIndex, double array[6], int config[7] = { 0 }, int MoveType = POSVAR);
    virtual bool MoveByJob(T_ROBOT_COORS tRobotJointCoord, T_ROBOT_MOVE_SPEED tPulseMove, int nExternalAxleType, std::string JobName = "MOVL", int isconfig = 1, int config[7] = { 0 });
    virtual bool MoveByJob(T_ANGLE_PULSE tRobotJointCoord, T_ROBOT_MOVE_SPEED tPulseMove, int nExternalAxleType, std::string JobName = "MOVJ");
    virtual bool MoveByJob(double* dRobotJointCoord, T_ROBOT_MOVE_SPEED tPulseMove, int nExternalAxleType, int nPVarType = PULSEVAR, std::string JobName = "MOVJ", int config[7] = { 0 });

private:
    void CreateFanucChain();
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

//----------------------------------------变量类--------------------------------------------//
public:
// 脉冲当量
    //double m_dPulseEqS;
    //double m_dPulseEqL;
    //double m_dPulseEqU;
    //double m_dPulseEqR;
    //double m_dPulseEqB;
    //double m_dPulseEqT;
    //double m_dPulseEqEX;
    //double m_dPulseEqEY;
    //double m_dPulseEqEZ;

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
    int m_nSocketPort;
	int m_nRobotType;									//关节臂类型（按工作种类划分）
    int m_nExternalAxleType;                           // 外部轴类型，来自 RobotPara.ini [ExternalAxle]
    int m_nRobotAxisCount;                             // 机器人轴数，默认 6，外部轴启用后累加
	E_ROBOT_BRAND m_eRobotBrand;						//机器人品牌
	//----------------------------------------KDL运动学部分------------------------------------//
	KDL::Chain m_pFanucChain;
    //----------------------------------------日志相关----------------------------------------//
	RobotLog* m_pRobotLog; // 日志实例（默认路径：Log/robot_log.txt，开启控制台输出）
    FtpClient* m_pFTP;
    mutable std::mutex m_lastRobotErrorMutex;
    std::string m_sLastRobotError;
};
