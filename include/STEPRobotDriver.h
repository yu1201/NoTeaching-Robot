#pragma once
#include "Const.h"
#include "RobotCom.hpp"
#include "RobotLog.h"

#include "RobotMessage.h"
#include "RobotDriverAdaptor.h"

#ifndef __STEP_ROBOT_CTRL
#define __STEP_ROBOT_CTRL

inline const char* GetErrorText(int nErrCode)
{
	switch (nErrCode)
	{
	case 0:      return "执行成功";
	case -1:     return "指令执行失败";
	case -2:     return "数据收发异常";
	case -3:     return "连接断开";
	default:     return "未知错误";
	}
}

inline const char* GetModeText(int nMode)
{
	switch (nMode)
	{
	case 0:      return "手动";
	case 1:     return "自动";
	case 2:     return "外部自动";
	case 3:     return "开始";
	case 4:     return "停止";
	case 5:     return "停止点动";
	default:     return "未知错误";
	}
}



using namespace STEPROBOTSDK;
class STEPRobotCtrl : public RobotDriverAdaptor
{
public:
	STEPRobotCtrl(std::string strUnitName, RobotLog* pLog);
	~STEPRobotCtrl() override;
public:

	bool InitSocket(const char* ip, u_short Port, bool ifRecode = false) override;
	bool CloseSocket() override;
	bool InitRobotDriver(std::string strUnitName) override;


	double GetCurrentPos(int nAxisNo) override;
	T_ROBOT_COORS GetCurrentPos() override;
	double GetCurrentPulse(int nAxisNo) override;
	T_ANGLE_PULSE GetCurrentPulse() override;


	int CheckDone();
	int CheckRobotDone(int nDelayTime = 200);

	bool CallJob(std::string sJobName);

	//任意点集 坐标类型、插补方式、速度 的连续运动
	int ContiMoveAny(const std::vector<T_ROBOT_MOVE_INFO>& vtRobotMoveInfo) override;

	bool SendWeldTriangleWeaveProgram(int nWeldTrackNum);

	bool SendWeldLWeaveProgram(int nWeldTrackNum);

	bool SendWeldProgram(int nWeldTrackNum);

	//初始化ftp
	int InitFtp();
	//上传文件给埃斯顿机器人，埃斯顿为RemoteFilePath，本地为LocalFilePath    //  .//MultiPos_Mv1.erd 
	int UploadFile(std::string LocalFilePath, std::string RemoteFilePath);
	//下载文件,埃斯顿为RemoteFilePath，本地为LocalFilePath
	int DownloadFile(std::string RemoteFilePath, std::string LocalFilePath);

	bool ServoOff();
	bool ServoOn();

	//清除报警信息+
	bool cleanAlarm();
	//设置当前模式 0-手动模式，1-自动模式，2-远程模式，-1-报错
	bool SetSysMode(int mode);
	//设置示教器速度
	bool SetTpSpeed(int speed);//设置TP示教器上的速度%

	//获取当前程序名
	std::string GetUserProgram();

	//获取当前工程名
	std::string GetUserProject();

	//加载程序 设置变量前要先加载程序
	bool LoadUserProgram(std::string projName, std::string progName);
	//卸载程序 卸载的是目前加载的程序
	bool UnLoadUserProgramer();


	//运行程序 运行的是目前加载的程序
	bool Prog_startRun_Py();
	//停止程序 停止的是目前加载的程序
	bool Prog_stop_Py();


	//设置当前工具 !+
	bool SetRobotToolNo(int nToolNo);

	/****************************************************信息获取****************************************************/
	//获取工具信息	+		
	bool GetToolData(int unToolNo, T_ROBOT_COORS adRobotToolData);

	/****************************************************变量读写****************************************************/
	//,int nToolNo = 1 scoper,0-系统，1-全局，2-工程，3-程序

	//pos[9]:x y z a b c 7 8   , scoper,0-系统，1-全局，2-工程，3-程序 , Coord,0-直角，1-关节
	//pos[8]:a1 a2 a3 a4 a5 a6 a7 a8,设置关节
	//config[0]:mode,config[1];cf1,config[2];cf2,config[3];cf3,config[4];cf4,config[5];cf5,config[6];cf6
	//对mP,mJP赋值
	//(int nIndex, double adPosVar[8], int nPVarType, int isconfig, int config[7], int scoper, int Coord)
	bool SetPosVar(int nIndex, double pos[8], int nPVarType, int isconfig, int config[7] = { 0 }, int scoper = ENGINEEVAR, int Coord = POSVAR);
	void SetPosVar(int nIndex, T_ROBOT_COORS tRobotCoors, int isconfig = 1, int config[7] = { 0 }, int scoper = ENGINEEVAR);//发送点没有外部轴
	bool SetPosVar(int nIndex, T_ANGLE_PULSE tRobotPulse, int scoper = ENGINEEVAR);  //发送点没有外部轴
	bool SetPosVar(int nIndex, AXISPOS eRobotCoors, int scoper = ENGINEEVAR);
	bool SetPosVar(int nIndex, JointsPos eRobotCoors, int scoper = ENGINEEVAR);

	int GetPosVar(long lPvarIndex, double array[6], int config[7] = { 0 }, int MoveType = POSVAR);

	// 埃斯顿机器人专用: 速度变量读写
	bool SetSpeed(const char* name, double* speed, int scord = ENGINEEVAR); //char设置单个速度
	bool SetSpeed(int nIndex, double adSpeed[5]); // 设置单个速度
	bool SetSpeed(int nIndex, SDynamicPercent adSpeed); // 设置单个速度

	////摆焊参数
	//bool SetWeaveDate(const char* name, ESTUN_WeaveDate WeaveDate, int scope = 2);
	//获取一个指定I变量
	int GetIntVar(int nIndex, const char* cStrPreFix = "INT");
	//设置一个指定I变量
	bool SetIntVar(int nIndex, int nValue, int score = 2, const char* cStrPreFix = "INT");
	//设置一个指定I变量
	bool SetIntVar(const char* name, int value, int score = 2);
	//设置Real变量
	bool SetRealVar(int nIndex, double value, const char* cStrPreFix = "REAL", int score = 1);//主要用于发送电流电压 scoper,0-系统，1-全局，2-工程，3-程序

	/****************************************************运动函数****************************************************/

//关节坐标系单轴运动函数(暂无外部轴)++  (用当前用户和工具)	/*1 - 10000 representing 0.01 to 100.0 %),推荐500*/
	bool AxisPulseMove(int nAxisNo, long lDist, long lRobotSpd, int nCoorType = POSVAR, int nMovtype = RELVAR, int nToolNo = 1, long lCoordFrm = 1);
	//直角坐标系单轴运动函数(暂无外部轴)
	bool PosMove(int nAxisNo, double dDist, long lRobotSpd, int nCoorType = POSVAR, int nMovtype = RELVAR, int config[7] = { 0 }, int nToolNo = 1, long lCoordFrm = 0);
	bool PosMove(int nAxisNo, double dDist, long lRobotSpd, int nCoorType = POSVAR, int nMovtype = RELVAR, int nToolNo = 1, long lCoordFrm = 0);
	//基础移动函数
	bool MoveByJob(int Axis, double Distence, int config[7], double speed, int ifAbsoluteM, int ifJoint);////默认 mode值Cf值为零 ifAbsoluteM 是否相对值，ifJoint 是否为直角坐标
	bool MoveByJob(double Distence[8], int config[7], double speed, int ifAbsolutM, int ifJoint);

	//通用移动函数	
	bool MoveByJob(double* dRobotJointCoord, T_ROBOT_MOVE_SPEED tPulseMove, int nExternalAxleType, int nPVarType = PULSEVAR, std::string JobName = "MOVJ", int config[7] = { 0 });
	bool MoveByJob(T_ROBOT_COORS tRobotJointCoord, T_ROBOT_MOVE_SPEED tPulseMove, int nExternalAxleType, std::string JobName = "MOVL", int isconfig = 1, int config[7] = { 0 });////默认 mode值Cf值为零	

	bool MoveByJob(T_ANGLE_PULSE tRobotJointCoord, T_ROBOT_MOVE_SPEED tPulseMove, int nExternalAxleType, std::string JobName = "MOVJ");



	HANDLE m_hMutex;
	bool m_bLocalDebugMark;
	RobotComClient* m_pSTEPRobotClient;

};
#endif
