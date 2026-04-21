#pragma once
#ifndef CONST_H
#define CONST_H


#include <cstdio>
#include <cstdlib>  
#include <cmath>    
#include <string>
#include <vector>
#include <sstream>  // 字符串拼接

#include <cstdarg>   // C++ 推荐写法

//using namespace std; // 可选，避免每次写std::string
// 定义 PI 常量（如果系统未定义）
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define MOVL 0
#define MOVJ 1

#define POSVAR 0				//直角坐标
#define PULSEVAR 1				//关节坐标

#define ENGINEEVAR 0			//工程变量
#define	PROGRAMVAR 1			//程序变量

#define RELVAR	0				//相对值
#define ABSVAR	1				//绝对值

#define ROBOT_TYPE_STEP 1
#define ROBOT_TYPE_FANUC 2

struct T_ROBOT_COORS {
	double dX;
	double dY;
	double dZ;
	double dRX;
	double dRY;
	double dRZ;
	double dBX;
	double dBY;
	double dBZ;

	// 构造函数，默认初始化为全0
	T_ROBOT_COORS();
	// 构造函数
	T_ROBOT_COORS(double dX, double dY, double dZ, double dRX, double dRY, double dRZ, double dBX, double dBY, double dBZ);
	// 加法，所有值一一对应相加
	T_ROBOT_COORS operator+(const T_ROBOT_COORS& tPoint);
	// 减法，所有值一一对应相减
	T_ROBOT_COORS operator-(const T_ROBOT_COORS& tPoint);
	// 乘法，所有值一一乘以dNum
	T_ROBOT_COORS operator*(double dNum);
	// 将外部轴坐标加到机械臂坐标里，然后清空外部轴坐标
	void TransToWorld();
	// 输出到文件
	int fprintf(FILE* file, const char* sFormat = "%12.4lf %12.4lf %12.4lf %12.4lf %12.4lf %12.4lf %12.4lf %12.4lf %12.4lf\n");
	int fprintf(FILE* file, int nIndex, const char* sFormat = "%4d %12.4lf %12.4lf %12.4lf %12.4lf %12.4lf %12.4lf %12.4lf %12.4lf %12.4lf\n");
	// 比较函数
	bool Compare(const T_ROBOT_COORS& tCompare, const T_ROBOT_COORS& tLimit);
};

struct T_ANGLE_PULSE {
	long nSPulse;
	long nLPulse;
	long nUPulse;
	long nRPulse;
	long nBPulse;
	long nTPulse;
	long lBXPulse;
	long lBYPulse;
	long lBZPulse;

	// 构造函数，默认初始化为全0
	T_ANGLE_PULSE();
	// 构造函数
	T_ANGLE_PULSE(long nSPulse, long nLPulse, long nUPulse, long nRPulse, long nBPulse, long nTPulse, long lBXPulse, long lBYPulse, long lBZPulse);
	// 输出到文件
	int fprintf(FILE* file, const char* sFormat = "%10ld %10ld %10ld %10ld %10ld %10ld %10ld %10ld %10ld\n");
	int fprintf(FILE* file, int nIndex, const char* sFormat = "%4d %10ld %10ld %10ld %10ld %10ld %10ld %10ld %10ld %10ld\n");
	// 比较函数
	bool Compare(const T_ANGLE_PULSE& tCompare, const T_ANGLE_PULSE& tLimit);
};

typedef struct
{
	T_ROBOT_COORS tGunTool;
	T_ROBOT_COORS tMagnetTool;
	T_ROBOT_COORS tPolisherTool;
	T_ROBOT_COORS tCameraTool;
}T_ROBOT_TOOLS;

typedef enum
{
	ROBOT_BRAND_YASKAWA = 0x01,			//安川
	ROBOT_BRAND_ESTUN = 0x02,		//埃斯顿
	ROBOT_BRAND_CRP = 0x03			//卡诺普
}E_ROBOT_BRAND;

typedef struct T_KINEMATICS
{
	double dA1;   //连杆长度
	double dAL1;    //连杆扭转角  
	double dD1;     //连杆偏距
	double dTH1;    //关节角

	double dA2;
	double dAL2;
	double dD2;
	double dTH2;

	double dA3;
	double dAL3;
	double dD3;
	double dTH3;

	double dA4;
	double dAL4;
	double dD4;
	double dTH4;

	double dA5;
	double dAL5;
	double dD5;
	double dTH5;

	double dA6;
	double dAL6;
	double dD6;
	double dTH6;

	T_KINEMATICS()
	{
		memset(this, 0, sizeof(T_KINEMATICS));
	}
}T_KINEMATICS;

struct T_AXISUNIT
{
	double dSPulseUnit;
	double dLPulseUnit;
	double dUPulseUnit;
	double dRPulseUnit;
	double dBPulseUnit;
	double dTPulseUnit;
	double dBXPulseUnit;
	double dBYPulseUnit;
	double dBZPulseUnit;
	T_AXISUNIT()
	{
		memset(this, 0, sizeof(T_AXISUNIT));
	}
	double GetValueByIndex(int index) const
	{
		const int MEMBER_COUNT = 8;
		if (index < 0 || index > MEMBER_COUNT)
		{
			//g_robotLogger.write(LogColor::ERR,
			//    "T_AXISUNIT::GetValueByIndex 越界！非法序号：%d，合法范围：1-%d，结构体地址：0x%p",
			//    index, MEMBER_COUNT, static_cast<const void*>(this));
			return -9999.0;
		}

		const double* pMemberBase = reinterpret_cast<const double*>(this);
		return *(pMemberBase + index);
	}

};

struct T_AXISLIMITANGLE
{
	double dMaxSAngle;
	double dMinSAngle;

	double dMaxLAngle;
	double dMinLAngle;

	double dMaxUAngle;
	double dMinUAngle;

	double dMaxRAngle;
	double dMinRAngle;

	double dMaxBAngle;
	double dMinBAngle;

	double dMaxTAngle;
	double dMinTAngle;

	double dMaxBXAngle;
	double dMinBXAngle;

	double dMaxBYAngle;
	double dMinBYAngle;

	double dMaxBZAngle;
	double dMinBZAngle;
	T_AXISLIMITANGLE()
	{
		memset(this, 0, sizeof(T_AXISLIMITANGLE));
	}

	double GetMaxAngleByIndex(int axisIndex) const
	{
		if (axisIndex < 0 || axisIndex > 8) {
			//g_robotLogger.write(LogColor::ERR, "GetMaxAngle越界！序号：%d（合法1-6）", axisIndex);
			return -9999.0;
		}
		return *((double*)this + axisIndex * 2);
	}

	double GetMinAngleByIndex(int axisIndex) const
	{
		if (axisIndex < 0 || axisIndex > 8) {
			//g_robotLogger.write(LogColor::ERR, "GetMinAngle越界！序号：%d（合法1-6）", axisIndex);
			return -9999.0;
		}
		return *((double*)this + axisIndex * 2 + 1);
	}

};

struct T_CONTRAL_UNIT  // 给结构体命名
{
	// 成员变量声明（不赋默认值）
	int nUnitNo;
	std::string sUnitName;
	std::string sChineseName;
	std::string sContralUnitType;
	int nsUnitType;
	void* pUnitDriver;

	T_CONTRAL_UNIT()
		: nUnitNo(-1)
		, nsUnitType(0)
		, pUnitDriver(nullptr)
	{
	}
};

struct T_ROBOT_MOVE_SPEED
{
	double dSpeed = 0;
	double dACC = 0;
	double dDEC = 0;

	T_ROBOT_MOVE_SPEED(double dSpeed, double dACC, double dDEC)
	{
		this->dSpeed = dSpeed;
		this->dACC = dACC;
		this->dDEC = dDEC;
	}
	T_ROBOT_MOVE_SPEED()
	{

	}
};

struct T_ROBOT_MOVE_INFO
{
	T_ROBOT_COORS tCoord;	//坐标
	T_ANGLE_PULSE tPulse;	//坐标
	int nMoveType = MOVJ;	//移动方式
	T_ROBOT_MOVE_SPEED tSpeed;	//移动速度
	int nMoveDevice = -1;
	int nTrackNo = -1;
	double adBasePosVar[3]; //外部轴坐标(MP_USR_VAR_INFO中最多支持8轴)
};

typedef struct
{
	int Type;
	double Freq;
	double Amp_L;
	double Amp_R;
	int StopTime_L;
	int StopTime_C;
	int StopTime_R;
	double RotAngle_X;
	double RotAngle_Z;
	int DelayType_L;
	int DelayType_C;
	int DelayType_R;
	double RotAngle_L;
	double RotAngle_R;
}T_WeaveDate;

typedef struct
{
	std::string strWorkPeace;
	std::string strWeldType;
	double dWeldAngleSize; // 焊脚尺寸 仅支持整数尺寸
	int nLayerNo; // 多层多道层号
	double dStartArcCurrent;
	double dStartArcVoltage;
	double dStartWaitTime;
	double dTrackCurrent;
	double dTrackVoltage;
	double WeldVelocity;
	double dStopArcCurrent;
	double dStopArcVoltage;
	double dStopWaitTime;

	double dWrapCurrentt1;
	double dWrapVoltage1;
	double dWrapWaitTime1;
	double dWrapCurrentt2;
	double dWrapVoltage2;
	double dWrapWaitTime2;
	double dWrapCurrentt3;
	double dWrapVoltage3;
	double dWrapWaitTime3;

	double CrosswiseOffset;//横向补偿
	double verticalOffset;//竖直方向补偿
	int nWrapConditionNo; // 摆动条件号
	double dWeldAngle; // 焊接角度 焊丝和底板或立焊侧板夹角
	double dWeldDipAngle; // 焊接倾角 药芯焊丝(拉) 实芯焊丝(推) 倾角不同
	int nStandWeldDir; // 立焊焊接方向：0:立向下  1:立向上
	int nWeaveTypeNo; // 关联的摆动类型号

	// CO2/混合气 实心/药芯 脉冲/直流 1.2/1.4 等不同组合
	int nWeldMethod; // 焊接方法 1脉冲分别/0直流分别 （0立焊 1平焊）

	T_WeaveDate tWeaveParam; // 摆动参数(埃斯顿摆动需要动态写入使用)
}T_WELD_PARA;

// 线扫参数中的起点/终点脉冲姿态信息
struct T_SCAN_WELDING_PARAM
{
	std::string sRobotName;      // 机器人名称，从 RobotDriverAdaptor 中获取
	std::string sSectionName;    // ini 分组名，例如 Table1
	std::string sIniFilePath;    // DATA/<RobotName>/LineScanParam.ini

	T_ANGLE_PULSE tStartPulse;   // 机器人扫描起点脉冲姿态
	T_ANGLE_PULSE tEndPulse;     // 机器人扫描终点脉冲姿态

	bool bValid = false;
};

// 单个料台的线扫粗定位参数，对应 ini 中一个 [TableN] 分组
struct T_COARSE_SCAN_PARAM
{
	int nTableNo = -1;
	std::string sUnitName;
	double dYMaxCar = 0.0;
	double dYMinCar = 0.0;
	double dYMaxRobot = 0.0;
	double dYMinRobot = 0.0;
	double dXMax = 0.0;
	double dXMin = 0.0;
	double dZMax = 0.0;
	double dZMin = 0.0;

	double dScanSpeed = 0.0;
	double dRunSpeed = 0.0;
	double dAcc = 0.0;
	double dDec = 0.0;

	double dTableY = 0.0;
	double dTableZ = 0.0;
	T_ANGLE_PULSE tStartPulse;
	T_ANGLE_PULSE tEndPulse;

	double dScanStartCarLoction = 0.0;
	double dScanEndtCarLoction = 0.0;
	double dScanLength = 0.0;
	int nTableScanDir = 0;
	int nExAxisEnable = 0;

	double dRangeXMax = 0.0;
	double dRangeXMin = 0.0;
	double dRangeYMax = 0.0;
	double dRangeYMin = 0.0;
	double dRangeZMax = 0.0;
	double dRangeZMin = 0.0;
	int nTableImgStartX = 0;
	int nTableImgEndX = 0;
};

//typedef enum {
//    MANUAL = 1,
//    AUTO,
//    AUTO_EXT
//} eMode;


const std::string DATA_PATH = ".\\Data\\";
const std::string ROBOT_PARA_INI = "\\RobotPara.ini";
const std::string CONTRAL_UNIT_INFO_INI = ".\\Data\\ContralUnitInfo.ini";

std::string GetStr(const char* format, ...);
#endif
