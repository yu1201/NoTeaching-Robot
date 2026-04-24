#pragma once

#include "Const.h"
#include "RobotCom.hpp"
#include "RobotLog.h"
#include "RobotMessage.h"
#include "RobotDriverAdaptor.h"

#include <cstdint>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>
#include <vector>

#ifndef __FANUC_ROBOT_CTRL
#define __FANUC_ROBOT_CTRL

class FANUCRobotCtrl : public RobotDriverAdaptor
{
public:
	FANUCRobotCtrl(std::string strUnitName, RobotLog* pLog);
	~FANUCRobotCtrl() override;

public:
	// 初始化与控制连接：机器人侧S4作为TCP服务器，上位机作为客户端请求/应答。
	bool InitSocket(const char* ip, u_short Port, bool ifRecode = false) override;
	bool CloseSocket() override;
	bool InitRobotDriver(std::string strUnitName) override;

	// 主动状态读取：通过控制socket实时请求机器人数据，会占用S4控制通道。
	double GetCurrentPos(int nAxisNo) override;
	T_ROBOT_COORS GetCurrentPos() override;
	double GetCurrentPulse(int nAxisNo) override;
	T_ANGLE_PULSE GetCurrentPulse() override;
	int CheckDone();
	int CheckRobotDone(int nDelayTime = 200);

	// 被动状态读取：读取S5监控线程最后一帧缓存，不占用控制socket；时间戳同时返回机器人累计毫秒和PC接收毫秒。
	T_ROBOT_COORS GetCurrentPosPassive(long long* pRobotMs = nullptr, long long* pPcRecvMs = nullptr);
	T_ANGLE_PULSE GetCurrentPulsePassive(long long* pRobotMs = nullptr, long long* pPcRecvMs = nullptr);
	int CheckDonePassive(long long* pRobotMs = nullptr, long long* pPcRecvMs = nullptr);

	// 程序调用与监控通道：CallJob走S4控制通道，Monitor走S5独立推送通道。
	bool CallJob(std::string sJobName);
	// 通用TP完成检测：约定程序启动后写入运行态(默认10/20)，完成时写入完成态(默认1)。
	bool CallJobAndWaitStateDone(
		std::string sJobName,
		int nStateReg = 92,
		int nDoneState = 1,
		int nStartStateA = 10,
		int nStartStateB = 20,
		int nStartTimeoutMs = 5000,
		int nFinishTimeoutMs = 10000,
		int nDelayTime = 100,
		int* pLastState = nullptr,
		bool bResetStateBeforeCall = true);
	// 通用寄存器完成检测：适合程序已启动、只需要等待状态寄存器从运行态进入完成态的场景。
	bool WaitStateDone(
		int nStateReg = 93,
		int nDoneState = 1,
		int nStartStateA = 10,
		int nStartStateB = 20,
		int nStartTimeoutMs = 3000,
		int nFinishTimeoutMs = 120000,
		int nDelayTime = 100,
		int* pLastState = nullptr);
	bool StopRobotServices();
	bool StartMonitor(int nPort = 0);
	void StopMonitor();
	std::string GetMonitorText();

	// 连续运动：生成并上传临时程序，适合多点路径/特殊运动；与下方固定TP单点运动区分。
	int ContiMoveAny(const std::vector<T_ROBOT_MOVE_INFO>& vtRobotMoveInfo) override;
	int UploadMultiPointTpProgram(
		const std::vector<T_ROBOT_MOVE_INFO>& vtRobotMoveInfo,
		std::string* pProgramName = nullptr,
		std::string* pLocalLsPath = nullptr,
		std::string* pRemoteTpPath = nullptr);
	bool StartContinuousMoveQueue(int nMoveType, double dSpeed);
	bool PushContinuousMovePoint(const T_ROBOT_MOVE_INFO& moveInfo);
	bool PushContinuousMovePoint(const T_ROBOT_COORS& target, double dSpeed);
	bool PushContinuousMovePoint(const T_ANGLE_PULSE& target, double dSpeed);
	void RequestEndContinuousMoveQueue();
	void EndContinuousMoveQueue();
	bool IsContinuousMoveQueueRunning() const;

	// 工艺程序上传：发送预置焊接/摆焊KAREL程序，并写入所需寄存器参数。
	bool SendWeldTriangleWeaveProgram(int nWeldTrackNum);
	bool SendWeldLWeaveProgram(int nWeldTrackNum);
	bool SendWeldProgram(int nWeldTrackNum);

	// 文件传输：KL会先编译成PC上传，LS会先编译成TP上传。
	int UploadKlFile(std::string localKlPath, std::string remoteDir = "/md/");
	int UploadLsFile(std::string localLsPath, std::string remoteDir = "/md/");

	// FTP通用接口：供固定程序/特殊程序上传下载复用。
	int InitFtp();
	int UploadFile(std::string LocalFilePath, std::string RemoteFilePath);
	int DownloadFile(std::string RemoteFilePath, std::string LocalFilePath);

	// 机器人基础控制：当前大多通过常驻服务命令转发，未实现的命令由机器人侧返回或占位OK。
	bool ServoOff();
	bool ServoOn();
	bool cleanAlarm();
	bool SetSysMode(int mode);
	bool SetTpSpeed(int speed);

	// 程序状态查询：用于兼容既有STEP接口命名。
	std::string GetUserProgram();
	std::string GetUserProject();
	std::string SendRawCommandForTest(const std::string& command);
	bool LoadUserProgram(std::string projName, std::string progName);
	bool UnLoadUserProgramer();
	bool Prog_startRun_Py();
	bool Prog_stop_Py();

	// 工具与位置寄存器：SetPosVar最终由KAREL服务写PR，Cartesian写SET_POS_REG，Joint写SET_JPOS_REG。
	bool SetRobotToolNo(int nToolNo);
	bool GetToolData(int unToolNo, T_ROBOT_COORS adRobotToolData);
	bool SetPosVar(int nIndex, double pos[8], int nPVarType, int isconfig, int config[7] = { 0 }, int scoper = ENGINEEVAR, int Coord = POSVAR);
	void SetPosVar(int nIndex, T_ROBOT_COORS tRobotCoors, int isconfig = 1, int config[7] = { 0 }, int scoper = ENGINEEVAR);
	bool SetPosVar(int nIndex, T_ANGLE_PULSE tRobotPulse, int scoper = ENGINEEVAR);
	//bool SetPosVar(int nIndex, AXISPOS eRobotCoors, int scoper = ENGINEEVAR);
	//bool SetPosVar(int nIndex, JointsPos eRobotCoors, int scoper = ENGINEEVAR);

	int GetPosVar(long lPvarIndex, double array[6], int config[7] = { 0 }, int MoveType = POSVAR);

	// 速度与寄存器：R[17]作为固定TP速度寄存器，INT/REAL用于工艺参数或兼容接口。
	bool SetSpeed(const char* name, double* speed, int scord = ENGINEEVAR);
	bool SetSpeed(int nIndex, double adSpeed[5]);
	//bool SetSpeed(int nIndex, SDynamicPercent adSpeed);
	int GetIntVar(int nIndex, const char* cStrPreFix = "INT");
	bool SetIntVar(int nIndex, int nValue, int score = 2, const char* cStrPreFix = "INT");
	bool SetIntVar(const char* name, int value, int score = 2);
	bool SetRealVar(int nIndex, double value, const char* cStrPreFix = "REAL", int score = 1);

	// 运动命令兼容层：旧接口仍保留；单点MOVL/MOVJ使用固定TP以避免重复编译。
	bool AxisPulseMove(int nAxisNo, long lDist, long lRobotSpd, int nCoorType = POSVAR, int nMovtype = RELVAR, int nToolNo = 1, long lCoordFrm = 1);
	bool PosMove(int nAxisNo, double dDist, long lRobotSpd, int nCoorType = POSVAR, int nMovtype = RELVAR, int config[7] = { 0 }, int nToolNo = 1, long lCoordFrm = 0);
	bool PosMove(int nAxisNo, double dDist, long lRobotSpd, int nCoorType = POSVAR, int nMovtype = RELVAR, int nToolNo = 1, long lCoordFrm = 0);
	bool MoveByJob(int Axis, double Distence, int config[7], double speed, int ifAbsoluteM, int ifJoint);
	bool MoveByJob(double Distence[8], int config[7], double speed, int ifAbsolutM, int ifJoint);
	bool MoveByJob(double* dRobotJointCoord, T_ROBOT_MOVE_SPEED tPulseMove, int nExternalAxleType, int nPVarType = PULSEVAR, std::string JobName = "MOVJ", int config[7] = { 0 });
	bool MoveByJob(T_ROBOT_COORS tRobotJointCoord, T_ROBOT_MOVE_SPEED tPulseMove, int nExternalAxleType, std::string JobName = "MOVL", int isconfig = 1, int config[7] = { 0 });
	bool MoveByJob(T_ANGLE_PULSE tRobotJointCoord, T_ROBOT_MOVE_SPEED tPulseMove, int nExternalAxleType, std::string JobName = "MOVJ");

	// 运行资源：S4控制socket、S5监控socket、FTP客户端和从ini读取的机器人参数。
	HANDLE m_hMutex;
	bool m_bLocalDebugMark;
	std::uintptr_t m_uSocketHandle;
	bool m_bSocketConnected;
	bool m_bWSAStarted;
	// 监控线程缓存：由S5监控通道更新，UI/业务线程只读缓存，减少对控制通道S4的抢占。
	std::uintptr_t m_uMonitorSocketHandle;
	std::atomic_bool m_bMonitorRunning;
	bool m_bMonitorWSAStarted;
	int m_nMonitorPort;
	std::thread m_monitorThread;
	std::mutex m_monitorMutex;
	std::string m_sMonitorText;
	T_ROBOT_COORS m_tMonitorPos;
	T_ANGLE_PULSE m_tMonitorPulse;
	int m_nMonitorDone;
	int m_nMonitorDoneRaw;
	int m_nMonitorDoneCandidate;
	int m_nMonitorDoneStableCount;
	long long m_llMonitorRobotMs;
	long long m_llMonitorPcRecvMs;
	std::atomic<long long> m_llLastCallJobPcMs;

private:
	void ContinuousMoveWorker();
	bool UploadContinuousStartBufferToRobot(const std::vector<T_ROBOT_MOVE_INFO>& startBuffer);
	bool WriteContinuousMovePointToRobot(int prIndex, const T_ROBOT_MOVE_INFO& moveInfo);
	int ReadContinuousDoneCount();

	std::thread m_continuousMoveThread;
	mutable std::mutex m_continuousMoveLifecycleMutex;
	mutable std::mutex m_continuousMoveMutex;
	std::condition_variable m_continuousMoveCv;
	std::deque<T_ROBOT_MOVE_INFO> m_continuousMoveQueue;
	std::atomic_bool m_continuousMoveRunning;
	bool m_continuousMoveStopRequested;
	bool m_continuousMoveRobotStarted;
	int m_continuousMoveType;
	double m_continuousMoveSpeed;
	long long m_continuousWrittenCount;
	long long m_continuousConsumedCount;
	T_ROBOT_MOVE_INFO m_continuousLastPoint;
};

#endif
