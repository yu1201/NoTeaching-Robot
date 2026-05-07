#include "STEPRobotDriver.h"

#include <algorithm>
#include <cmath>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <string>

namespace
{
	std::string StepMakeProgramName()
	{
		return "ContiMoveAny";
	}

	long StepClampPositiveLong(double value, long defaultValue)
	{
		if (value <= 0.0)
		{
			return defaultValue;
		}
		return static_cast<long>(value);
	}

	std::string StepBuildDynamicName(size_t index)
	{
		return GetStr("dyn%u", static_cast<unsigned>(index));
	}

	std::string StepBuildCartPosName(size_t index)
	{
		return GetStr("cp%u", static_cast<unsigned>(index));
	}

	std::string StepBuildAxisPosName(size_t index)
	{
		return GetStr("ap%u", static_cast<unsigned>(index));
	}

	double StepPulseToJoint(long pulse, double pulseUnit)
	{
		return pulseUnit == 0.0 ? static_cast<double>(pulse) : static_cast<double>(pulse) * pulseUnit;
	}

	long StepJointToPulse(double joint, double pulseUnit)
	{
		return pulseUnit == 0.0 ? static_cast<long>(std::lround(joint)) : static_cast<long>(std::lround(joint / pulseUnit));
	}

	AXISPOS StepToAxisPos(const T_ANGLE_PULSE& tRobotPulse, const T_AXISUNIT& axisUnit)
	{
		AXISPOS value = {};
		value.m_Joint[0] = StepPulseToJoint(tRobotPulse.nSPulse, axisUnit.dSPulseUnit);
		value.m_Joint[1] = StepPulseToJoint(tRobotPulse.nLPulse, axisUnit.dLPulseUnit);
		value.m_Joint[2] = StepPulseToJoint(tRobotPulse.nUPulse, axisUnit.dUPulseUnit);
		value.m_Joint[3] = StepPulseToJoint(tRobotPulse.nRPulse, axisUnit.dRPulseUnit);
		value.m_Joint[4] = StepPulseToJoint(tRobotPulse.nBPulse, axisUnit.dBPulseUnit);
		value.m_Joint[5] = StepPulseToJoint(tRobotPulse.nTPulse, axisUnit.dTPulseUnit);
		value.m_AuxJoint[0] = StepPulseToJoint(tRobotPulse.lBXPulse, axisUnit.dBXPulseUnit);
		value.m_AuxJoint[1] = StepPulseToJoint(tRobotPulse.lBYPulse, axisUnit.dBYPulseUnit);
		value.m_AuxJoint[2] = StepPulseToJoint(tRobotPulse.lBZPulse, axisUnit.dBZPulseUnit);
		return value;
	}

	std::string StepBuildSrdContent(const std::vector<T_ROBOT_MOVE_INFO>& moveInfos, const T_AXISUNIT& axisUnit)
	{
		std::ostringstream oss;
		oss << std::fixed << std::setprecision(6);
		oss << "OVERLAPABS ola0 := {  0, 0, 0,0 }" << "\n";

		for (size_t i = 0; i < moveInfos.size(); ++i)
		{
			const T_ROBOT_MOVE_INFO& info = moveInfos[i];
			const std::string cartName = StepBuildCartPosName(i);
			const std::string axisName = StepBuildAxisPosName(i);
			const std::string dynName = StepBuildDynamicName(i);

			oss << "CARTPOS " << cartName << " := {  "
				<< info.tCoord.dX << ", " << info.tCoord.dY << ", " << info.tCoord.dZ << ", "
				<< info.tCoord.dRX << ", " << info.tCoord.dRY << ", " << info.tCoord.dRZ << ", "
				<< info.adBasePosVar[0] << ", " << info.adBasePosVar[1] << ", " << info.adBasePosVar[2]
				<< ", 0.0, 0.0, 0.0,0 }" << "\n";

			oss << "AXISPOS " << axisName << " := {  "
				<< StepPulseToJoint(info.tPulse.nSPulse, axisUnit.dSPulseUnit) << ", "
				<< StepPulseToJoint(info.tPulse.nLPulse, axisUnit.dLPulseUnit) << ", "
				<< StepPulseToJoint(info.tPulse.nUPulse, axisUnit.dUPulseUnit) << ", "
				<< StepPulseToJoint(info.tPulse.nRPulse, axisUnit.dRPulseUnit) << ", "
				<< StepPulseToJoint(info.tPulse.nBPulse, axisUnit.dBPulseUnit) << ", "
				<< StepPulseToJoint(info.tPulse.nTPulse, axisUnit.dTPulseUnit) << ", "
				<< StepPulseToJoint(info.tPulse.lBXPulse, axisUnit.dBXPulseUnit) << ", "
				<< StepPulseToJoint(info.tPulse.lBYPulse, axisUnit.dBYPulseUnit) << ", "
				<< StepPulseToJoint(info.tPulse.lBZPulse, axisUnit.dBZPulseUnit)
				<< ", 0.0, 0.0,0.0 }" << "\n";

			const long speed = StepClampPositiveLong(info.tSpeed.dSpeed, 1000);
			const long acc = StepClampPositiveLong(info.tSpeed.dACC, speed * 2);
			const long dec = StepClampPositiveLong(info.tSpeed.dDEC, speed * 2);

			oss << "DYNAMIC " << dynName << " := {  "
				<< speed << ", " << acc << ", " << dec
				<< ", 50000, 90, 270, 270, 2700, 100, 200, 200,200 }" << "\n";
		}

		return oss.str();
	}

	std::string StepBuildSrpContent(const std::vector<T_ROBOT_MOVE_INFO>& moveInfos)
	{
		std::ostringstream oss;

		for (size_t i = 0; i < moveInfos.size(); ++i)
		{
			const T_ROBOT_MOVE_INFO& info = moveInfos[i];
			const std::string cartName = StepBuildCartPosName(i);
			const std::string axisName = StepBuildAxisPosName(i);
			const std::string dynName = StepBuildDynamicName(i);

			if (info.nMoveType == MOVL)
			{
				oss << "Lin(" << cartName << "," << dynName << ",ola0,tool1,WORLD);" << "\n";
			}
			else
			{
				oss << "PTP(" << axisName << "," << dynName << ",ola0,tool1,WORLD);" << "\n";
			}
		}

		return oss.str();
	}

	bool StepWriteTextFile(const std::string& filePath, const std::string& content)
	{
		std::ofstream out(filePath, std::ios::out | std::ios::trunc);
		if (!out.is_open())
		{
			return false;
		}

		out << content;
		return out.good();
	}

	std::string StepGetProgramScopeName(STEPRobotCtrl* ctrl, int scoper)
	{
		if (ctrl == nullptr)
		{
			return std::string();
		}
		return scoper == PROGRAMVAR ? ctrl->GetUserProgram() : std::string();
	}

	std::string StepBuildPosVarName(int nIndex, int nPVarType)
	{
		return nPVarType == PULSEVAR ? GetStr("ap%d", nIndex) : GetStr("cp%d", nIndex);
	}

	CARTPOS StepToCartPos(const T_ROBOT_COORS& tRobotCoors)
	{
		CARTPOS value = {};
		value.m_CartPos.cart[0] = tRobotCoors.dX;
		value.m_CartPos.cart[1] = tRobotCoors.dY;
		value.m_CartPos.cart[2] = tRobotCoors.dZ;
		value.m_CartPos.cart[3] = tRobotCoors.dRX;
		value.m_CartPos.cart[4] = tRobotCoors.dRY;
		value.m_CartPos.cart[5] = tRobotCoors.dRZ;
		value.m_CartPos.m_Mode = 0;
		value.m_AuxPos[0] = tRobotCoors.dBX;
		value.m_AuxPos[1] = tRobotCoors.dBY;
		value.m_AuxPos[2] = tRobotCoors.dBZ;
		return value;
	}

	SDynamicPercent StepToDynamicPercent(double vel, double acc, double dec, double jerk = -1.0, double tjolt = -1.0)
	{
		SDynamicPercent value = {};
		value.m_SegmentDynamic.m_Vel = vel;
		value.m_SegmentDynamic.m_Acc = acc;
		value.m_SegmentDynamic.m_Dec = dec;
		value.m_SegmentDynamic.m_Jerk = jerk;
		value.m_SegmentDynamic.m_Tjolt = tjolt;
		value.m_OriDynamic = value.m_SegmentDynamic;
		value.m_JointPercent = value.m_SegmentDynamic;
		return value;
	}

	int StepMoveTypeFromJobName(const std::string& jobName)
	{
		std::string upper = jobName;
		std::transform(upper.begin(), upper.end(), upper.begin(),
			[](unsigned char ch) { return static_cast<char>(std::toupper(ch)); });
		return upper == "MOVL" || upper == "LIN" || upper == "LINE" ? MOVL : MOVJ;
	}

	int StepSpeedToPercent(double speed)
	{
		if (speed <= 0.0)
		{
			return -1;
		}
		return std::max(1, std::min(100, static_cast<int>(std::lround(speed))));
	}

	std::string StepCurrentToolName(STEPRobotCtrl* ctrl)
	{
		if (ctrl == nullptr || ctrl->m_pSTEPRobotClient == nullptr)
		{
			return "Default";
		}
		const std::string tool = ctrl->m_pSTEPRobotClient->getToolName();
		return tool.empty() ? "Default" : tool;
	}

	std::string StepCurrentRefName(STEPRobotCtrl* ctrl)
	{
		if (ctrl == nullptr || ctrl->m_pSTEPRobotClient == nullptr)
		{
			return "WORLD";
		}
		const std::string ref = ctrl->m_pSTEPRobotClient->getTeachRefSysNam();
		return ref.empty() ? "WORLD" : ref;
	}

	HMIPos StepMakeAxisHmiPos(const T_ANGLE_PULSE& pulse, const T_AXISUNIT& axisUnit, bool hasExternalAxis)
	{
		HMIPos pos = {};
		pos.m_PosType = hasExternalAxis ? eAxisPos : eRobotAxisPos;
		pos.m_Pos.m_Joint[0] = StepPulseToJoint(pulse.nSPulse, axisUnit.dSPulseUnit);
		pos.m_Pos.m_Joint[1] = StepPulseToJoint(pulse.nLPulse, axisUnit.dLPulseUnit);
		pos.m_Pos.m_Joint[2] = StepPulseToJoint(pulse.nUPulse, axisUnit.dUPulseUnit);
		pos.m_Pos.m_Joint[3] = StepPulseToJoint(pulse.nRPulse, axisUnit.dRPulseUnit);
		pos.m_Pos.m_Joint[4] = StepPulseToJoint(pulse.nBPulse, axisUnit.dBPulseUnit);
		pos.m_Pos.m_Joint[5] = StepPulseToJoint(pulse.nTPulse, axisUnit.dTPulseUnit);
		pos.m_Pos.m_AuxPos[0] = StepPulseToJoint(pulse.lBXPulse, axisUnit.dBXPulseUnit);
		pos.m_Pos.m_AuxPos[1] = StepPulseToJoint(pulse.lBYPulse, axisUnit.dBYPulseUnit);
		pos.m_Pos.m_AuxPos[2] = StepPulseToJoint(pulse.lBZPulse, axisUnit.dBZPulseUnit);
		return pos;
	}

	HMIPos StepMakeCartHmiPos(const T_ROBOT_COORS& coors, bool hasExternalAxis)
	{
		HMIPos pos = {};
		pos.m_PosType = hasExternalAxis ? eCartPos : eRobotCartPos;
		pos.m_Pos.m_CartPos.m_X = coors.dX;
		pos.m_Pos.m_CartPos.m_Y = coors.dY;
		pos.m_Pos.m_CartPos.m_Z = coors.dZ;
		pos.m_Pos.m_CartPos.m_A = coors.dRX;
		pos.m_Pos.m_CartPos.m_B = coors.dRY;
		pos.m_Pos.m_CartPos.m_C = coors.dRZ;
		pos.m_Pos.m_CartPos.m_Mode = 0;
		pos.m_Pos.m_AuxPos[0] = coors.dBX;
		pos.m_Pos.m_AuxPos[1] = coors.dBY;
		pos.m_Pos.m_AuxPos[2] = coors.dBZ;
		return pos;
	}

	SDKOverlap StepNoOverlap()
	{
		SDKOverlap ovl = {};
		ovl.m_tcp_distance = -1.0;
		ovl.m_Percent = -1.0;
		ovl.m_OveTyp = static_cast<OVERLAPTYPE>(0);
		return ovl;
	}

	bool StepWaitProgramName(STEPRobotCtrl* ctrl, const std::string& expected, int timeoutMs)
	{
		if (ctrl == nullptr || ctrl->m_pSTEPRobotClient == nullptr)
		{
			return false;
		}
		for (int elapsed = 0; elapsed <= timeoutMs; elapsed += 50)
		{
			if (ctrl->m_pSTEPRobotClient->getProgramName() == expected)
			{
				return true;
			}
			Sleep(50);
		}
		return false;
	}

	bool StepWaitProgramState(STEPRobotCtrl* ctrl, PROGRAMSTATE expected, int timeoutMs)
	{
		if (ctrl == nullptr || ctrl->m_pSTEPRobotClient == nullptr)
		{
			return false;
		}
		for (int elapsed = 0; elapsed <= timeoutMs; elapsed += 50)
		{
			if (ctrl->m_pSTEPRobotClient->getProgramState() == expected)
			{
				return true;
			}
			Sleep(50);
		}
		return false;
	}

	bool StepUnloadProgram(STEPRobotCtrl* ctrl)
	{
		if (ctrl == nullptr || ctrl->m_pSTEPRobotClient == nullptr)
		{
			return false;
		}

		const std::string program = ctrl->GetUserProgram();
		if (program.empty())
		{
			return true;
		}

		int ret = 0;
		if (program == "SDK")
		{
			ret = ctrl->m_pSTEPRobotClient->SDKSimulateProgramKill(true);
		}
		else
		{
			ret = ctrl->m_pSTEPRobotClient->ProgramKillCmd(ctrl->GetUserProject(), program, true);
		}

		if (ret != 0)
		{
			if (ctrl->m_pRobotLog != nullptr)
			{
				ctrl->m_pRobotLog->write(LogColor::ERR, "STEP 卸载当前程序失败：%s,原因:%s", program.c_str(), GetErrorText(ret));
			}
			return false;
		}
		return StepWaitProgramName(ctrl, std::string(), 3000);
	}

	bool StepPrepareSdkProgram(STEPRobotCtrl* ctrl)
	{
		if (ctrl == nullptr || ctrl->m_pSTEPRobotClient == nullptr)
		{
			return false;
		}
		if (ctrl->m_pSTEPRobotClient->getProgramState() == eRun)
		{
			if (ctrl->m_pRobotLog != nullptr)
			{
				ctrl->m_pRobotLog->write(LogColor::ERR, "STEP 运动下发失败：机器人程序仍在运行");
			}
			return false;
		}

		if (!StepUnloadProgram(ctrl))
		{
			return false;
		}

		int ret = ctrl->m_pSTEPRobotClient->SDKSimulateProgramLoad(true);
		if (ret != 0)
		{
			if (ctrl->m_pRobotLog != nullptr)
			{
				ctrl->m_pRobotLog->write(LogColor::ERR, "STEP 加载SDK模拟程序失败,原因:%s", GetErrorText(ret));
			}
			return false;
		}
		if (!StepWaitProgramName(ctrl, "SDK", 3000))
		{
			if (ctrl->m_pRobotLog != nullptr)
			{
				ctrl->m_pRobotLog->write(LogColor::ERR, "STEP 等待SDK模拟程序加载超时");
			}
			return false;
		}

		if (!ctrl->Prog_startRun_Py())
		{
			return false;
		}
		return StepWaitProgramState(ctrl, eRun, 3000);
	}

	bool StepRunSimulateMove(STEPRobotCtrl* ctrl, const T_ROBOT_MOVE_INFO& moveInfo)
	{
		if (ctrl == nullptr || ctrl->m_pSTEPRobotClient == nullptr)
		{
			return false;
		}
		if (!StepPrepareSdkProgram(ctrl))
		{
			return false;
		}

		const bool hasExternalAxis = ctrl->m_nExternalAxleType != 0;
		const int dyn = StepSpeedToPercent(moveInfo.tSpeed.dSpeed);
		const SDKOverlap ovl = StepNoOverlap();
		const std::string tool = StepCurrentToolName(ctrl);
		const std::string ref = StepCurrentRefName(ctrl);
		const int moveType = moveInfo.nMoveType == MOVL ? MOVL : MOVJ;

		int ret = 0;
		if (moveType == MOVL)
		{
			HMIPos endPos = StepMakeCartHmiPos(moveInfo.tCoord, hasExternalAxis);
			ret = ctrl->m_pSTEPRobotClient->SDKSimulateLinProCmd(endPos, dyn, ovl, eOriNULL, tool, ref, 1);
		}
		else
		{
			HMIPos endPos = StepMakeAxisHmiPos(moveInfo.tPulse, ctrl->m_tAxisUnit, hasExternalAxis);
			ret = ctrl->m_pSTEPRobotClient->SDKSimulatePTPProCmd(endPos, dyn, ovl, tool, ref, 1);
		}

		if (ret != 0)
		{
			if (ctrl->m_pRobotLog != nullptr)
			{
				ctrl->m_pRobotLog->write(LogColor::ERR, "STEP 写入%s语句失败,原因:%s", moveType == MOVL ? "Lin" : "PTP", GetErrorText(ret));
			}
			return false;
		}

		ret = ctrl->m_pSTEPRobotClient->SDKSimulateEOFCmd(2);
		if (ret != 0)
		{
			if (ctrl->m_pRobotLog != nullptr)
			{
				ctrl->m_pRobotLog->write(LogColor::ERR, "STEP 写入EOF语句失败,原因:%s", GetErrorText(ret));
			}
			return false;
		}

		int errorLine = 0;
		ret = ctrl->m_pSTEPRobotClient->SDKSimulateProgramSend(errorLine);
		if (ret != 0)
		{
			if (ctrl->m_pRobotLog != nullptr)
			{
				ctrl->m_pRobotLog->write(LogColor::ERR, "STEP 发送运动语句失败,原因:%s,错误行:%d", GetErrorText(ret), errorLine);
			}
			return false;
		}

		if (ctrl->m_pRobotLog != nullptr)
		{
			ctrl->m_pRobotLog->write(LogColor::SUCCESS, "STEP %s运动已下发 | Speed=%d | Tool=%s | Ref=%s",
				moveType == MOVL ? "Lin" : "PTP", dyn, tool.c_str(), ref.c_str());
		}
		return true;
	}

	int StepNormalizeAxisIndex(int axisNo)
	{
		return axisNo >= 1 && axisNo <= 9 ? axisNo - 1 : axisNo;
	}

	void StepSetPulseByIndex(T_ANGLE_PULSE& pulse, int axisIndex, long value)
	{
		switch (axisIndex)
		{
		case 0: pulse.nSPulse = value; break;
		case 1: pulse.nLPulse = value; break;
		case 2: pulse.nUPulse = value; break;
		case 3: pulse.nRPulse = value; break;
		case 4: pulse.nBPulse = value; break;
		case 5: pulse.nTPulse = value; break;
		case 6: pulse.lBXPulse = value; break;
		case 7: pulse.lBYPulse = value; break;
		case 8: pulse.lBZPulse = value; break;
		default: break;
		}
	}

	long StepGetPulseByIndex(const T_ANGLE_PULSE& pulse, int axisIndex)
	{
		switch (axisIndex)
		{
		case 0: return pulse.nSPulse;
		case 1: return pulse.nLPulse;
		case 2: return pulse.nUPulse;
		case 3: return pulse.nRPulse;
		case 4: return pulse.nBPulse;
		case 5: return pulse.nTPulse;
		case 6: return pulse.lBXPulse;
		case 7: return pulse.lBYPulse;
		case 8: return pulse.lBZPulse;
		default: return 0;
		}
	}

	void StepSetCoorsByIndex(T_ROBOT_COORS& coors, int axisIndex, double value)
	{
		switch (axisIndex)
		{
		case 0: coors.dX = value; break;
		case 1: coors.dY = value; break;
		case 2: coors.dZ = value; break;
		case 3: coors.dRX = value; break;
		case 4: coors.dRY = value; break;
		case 5: coors.dRZ = value; break;
		case 6: coors.dBX = value; break;
		case 7: coors.dBY = value; break;
		case 8: coors.dBZ = value; break;
		default: break;
		}
	}

	double StepGetCoorsByIndex(const T_ROBOT_COORS& coors, int axisIndex)
	{
		switch (axisIndex)
		{
		case 0: return coors.dX;
		case 1: return coors.dY;
		case 2: return coors.dZ;
		case 3: return coors.dRX;
		case 4: return coors.dRY;
		case 5: return coors.dRZ;
		case 6: return coors.dBX;
		case 7: return coors.dBY;
		case 8: return coors.dBZ;
		default: return 0.0;
		}
	}
}


STEPRobotCtrl::STEPRobotCtrl(std::string strUnitName, RobotLog* pLog)
	: RobotDriverAdaptor(strUnitName, pLog)

{
	m_pSTEPRobotClient = new RobotComClient();
	m_bLocalDebugMark = false;
	InitRobotDriver(strUnitName);
	m_hMutex = CreateMutexA(NULL, FALSE, "Mutex");
	//InitSocket(m_sSocketIP.c_str(), m_nSocketPort);
}


STEPRobotCtrl::~STEPRobotCtrl()
{
}

bool STEPRobotCtrl::InitRobotDriver(std::string strUnitName)
{
	COPini cIni;

	cIni.SetFileName(DATA_PATH + strUnitName + ROBOT_PARA_INI);
	cIni.SetSectionName("BaseParam");
	cIni.ReadString("RobotName", m_sRobotName);
	cIni.ReadString("CustomName", m_sCustomName);
	cIni.ReadString("SocketIP", m_sSocketIP);
	cIni.ReadString("SocketPort", &m_nSocketPort);
	cIni.ReadString("RobotType", &m_nRobotType);
	cIni.ReadString("RobotBrand", (int*)&m_eRobotBrand);
	cIni.ReadString("FTPIP", m_sFTPIP);
	cIni.ReadString("FTPPort", &m_nFTPPort);
	cIni.ReadString("FTPUser", m_sFTPUser);
	cIni.ReadString("FTPPassWord", m_sFTPPassWord);

	cIni.SetSectionName("Tool");
	cIni.ReadString("PolisherTool_d", "", m_tTools.tPolisherTool, T_ROBOT_COORS(1, 1, 1, 1, 1, 1, -1, -1, -1));
	cIni.ReadString("MagnetTool_d", "", m_tTools.tMagnetTool, T_ROBOT_COORS(1, 1, 1, 1, 1, 1, -1, -1, -1));
	cIni.ReadString("GunTool_d", "", m_tTools.tGunTool, T_ROBOT_COORS(1, 1, 1, 1, 1, 1, -1, -1, -1));
	cIni.ReadString("CameraTool_d", "", m_tTools.tCameraTool, T_ROBOT_COORS(1, 1, 1, 1, 1, 1, -1, -1, -1));

	InitSocket(m_sSocketIP.c_str(), m_nSocketPort);
	return true;
}

bool STEPRobotCtrl::InitSocket(const char* ip, u_short Port, bool ifRecord)
{
	int nRet = 0;
	if (!m_bLocalDebugMark)
	{
		nRet = m_pSTEPRobotClient->init(ip, Port);
		if (0 != nRet)
		{
			return false;
		}
	}
	return true;
}

bool STEPRobotCtrl::CloseSocket()
{
	int nRet = 0;
	if (!m_bLocalDebugMark)
	{
		nRet = m_pSTEPRobotClient->close();
		if (0 != nRet)
		{
			return false;
		}
	}
	return true;
}

double STEPRobotCtrl::GetCurrentPos(int nAxisNo) 
{
	RobotCartPos cartposworld = m_pSTEPRobotClient->getCartPosWorld();

	double dPos = cartposworld.cart[nAxisNo]; //坐标
	return dPos;
}

T_ROBOT_COORS STEPRobotCtrl::GetCurrentPos() 
{
	T_ROBOT_COORS stCoors = T_ROBOT_COORS(); // 初始化所有值为0

	// 直接调用封装好的GetCurrentPos(int)函数，依次获取6个轴的坐标
	stCoors.dX = GetCurrentPos(0);
	stCoors.dY = GetCurrentPos(1);
	stCoors.dZ = GetCurrentPos(2);
	stCoors.dRX = GetCurrentPos(3);
	stCoors.dRY = GetCurrentPos(4);
	stCoors.dRZ = GetCurrentPos(5);

	return stCoors;
}

double STEPRobotCtrl::GetCurrentPulse(int nAxisNo) 
{
	AXISPOS currentaxispos = m_pSTEPRobotClient->getAxisPos(); //获取当前笛卡尔位置
	double dPulse = 0;
	if (nAxisNo >= 0 && nAxisNo < 6)
	{
		dPulse = currentaxispos.m_Joint[nAxisNo]; //X坐标
	}
	else if (nAxisNo > 5)
	{
		dPulse = currentaxispos.m_AuxJoint[nAxisNo - 6]; //附加轴1位置
	}
	else
	{
		return 0;
	}
	return static_cast<double>(StepJointToPulse(dPulse, m_tAxisUnit.GetValueByIndex(nAxisNo)));
}

T_ANGLE_PULSE STEPRobotCtrl::GetCurrentPulse()
{
	T_ANGLE_PULSE tPulse = T_ANGLE_PULSE();
	tPulse.nSPulse = GetCurrentPulse(0);
	tPulse.nLPulse = GetCurrentPulse(1);
	tPulse.nUPulse = GetCurrentPulse(2);
	tPulse.nRPulse = GetCurrentPulse(3);
	tPulse.nBPulse = GetCurrentPulse(4);
	tPulse.nTPulse = GetCurrentPulse(5);
	tPulse.lBXPulse = GetCurrentPulse(6);
	tPulse.lBYPulse = GetCurrentPulse(7);
	tPulse.lBZPulse = GetCurrentPulse(8);
	return tPulse;
}

int STEPRobotCtrl::CheckDone()
{
	//PROGRAMSTATE getProgramState();
	int nRet = m_pSTEPRobotClient->getProgramState();   //0	运行，1 暂停，2 停止，3 未知
	return nRet;
}
int STEPRobotCtrl::CheckRobotDone(int nDelayTime)
{
	int nRet = -1;
	while (1)
	{
		nRet = CheckDone();
		if (0 != nRet)
		{
			Sleep(nDelayTime);
			nRet = CheckDone();
			if (0 != nRet)
			{
				return nRet;
			}
		}
		Sleep(nDelayTime);
	}
	return -1;
}

std::string STEPRobotCtrl::GetUserProgram()
{
	std::string sProgramName = m_pSTEPRobotClient->getProgramName();
	return sProgramName;
}

std::string STEPRobotCtrl::GetUserProject()
{
	std::string sProgramName = m_pSTEPRobotClient->getProjectName();
	return sProgramName;
}

bool STEPRobotCtrl::LoadUserProgram(std::string sProjName, std::string sProgName)
{
	std::string sNowProject, sNowProgram;
	sNowProgram = GetUserProgram();
	sNowProject = GetUserProject();
	int nRet = 0;
	std::vector<int> vnErrLine;
	if (sProjName != sNowProject ||sProgName!=sNowProgram)
	{
		nRet = m_pSTEPRobotClient->ProgramLoadCmd(sProjName,sProgName,vnErrLine,true);
		if (nRet != 0)
		{
			showErrorMessage(
				nullptr,
				"加载程序失败,失败原因:%s",
				GetErrorText(nRet)   // 直接用全局错误库
			);
			return false;
		}

		if (!vnErrLine.empty())
		{
			std::stringstream ss;
			ss << "程序包含语法错误，\n 错误行号：";

			// 把所有错误行拼起来
			for (size_t i = 0; i < vnErrLine.size(); ++i)
			{
				if (i > 0)
					ss << ", ";

				ss << vnErrLine[i];
			}

			// 弹出所有错误行
			showWarnMessage(nullptr, "程序语法错误", ss.str().c_str());
		}
	}
	return true;
}

bool STEPRobotCtrl::UnLoadUserProgramer()
{
	std::string sNowProject, sNowProgram;
	sNowProgram = GetUserProgram();
	sNowProject = GetUserProject();
	int nRet = 0;
	nRet = m_pSTEPRobotClient->ProgramKillCmd(sNowProject, sNowProgram, true);
	if (nRet != 0)
	{
		showErrorMessage(
			nullptr,
			"卸载程序失败,失败原因:%s",
			GetErrorText(nRet)   // 直接用全局错误库
		);
		return false;
	}
	return true;
}

//设置当前模式：兼容旧调用的 0/1/2，也支持直接传 SDK 的 MODEKEY 值。
bool STEPRobotCtrl::SetSysMode(int nMode)
{
	MODEKEY eMode = MANUAL;
	if (nMode == 0)
	{
		eMode = MANUAL;
	}
	else if (nMode == 1)
	{
		eMode = AUTO;
	}
	else if (nMode == 2)
	{
		eMode = AUTO_EXT;
	}
	else if (nMode == MANUAL || nMode == AUTO || nMode == AUTO_EXT || nMode == START || nMode == STOP || nMode == MSTOP)
	{
		eMode = static_cast<MODEKEY>(nMode);
	}
	else
	{
		showErrorMessage(
			nullptr,
			"设置当前模式:%d失败,失败原因:模式选择错误",nMode
		);
		return false;
	}
	int nRet = m_pSTEPRobotClient->SetModeCmd(eMode,true);
	if (nRet != 0)
	{
		showErrorMessage(
			nullptr,
			"设置当前模式:%d失败,失败原因:%s", nMode,
			GetErrorText(nRet)   // 直接用全局错误库
		);
		return false;
	}
	return true;
}

//运行程序 运行的是目前加载的程序
bool STEPRobotCtrl::Prog_startRun_Py()
{
	int nRet = m_pSTEPRobotClient->SetModeCmd(MODEKEY::START, true);
	if (nRet != 0)
	{
		showErrorMessage(
			nullptr,
			"运行程序失败,失败原因:%s",
			GetErrorText(nRet)   // 直接用全局错误库
		);
		return false;
	}
	return true;
}

//停止程序 停止的是目前加载的程序
bool STEPRobotCtrl::Prog_stop_Py()
{
	int nRet = m_pSTEPRobotClient->SetModeCmd(MODEKEY::STOP, true);
	if (nRet != 0)
	{
		showErrorMessage(
			nullptr,
			"停止程序失败,失败原因:%s",
			GetErrorText(nRet)   // 直接用全局错误库
		);
		return false;
	}
	return true;
}

bool STEPRobotCtrl::CallJob(std::string sJobName)
{
	std::string sNowProjName = GetUserProject();
	
	if (0 != LoadUserProgram(sNowProjName, sJobName))
	{
		return false;
	}
	if (0 != Prog_startRun_Py())
	{
		return false;
	}
	return true;
}

int STEPRobotCtrl::ContiMoveAny(const std::vector<T_ROBOT_MOVE_INFO>& vtRobotMoveInfo)
{
	if (vtRobotMoveInfo.empty())
	{
		m_pRobotLog->write(LogColor::ERR, "STEP ContiMoveAny 失败：轨迹点为空");
		return -1;
	}

	std::string sProjectName = GetUserProject();
	if (sProjectName.empty())
	{
		sProjectName = m_sRobotName.empty() ? "DefaultProject" : m_sRobotName;
	}

	const std::string sProgramName = StepMakeProgramName();
	const std::string sLocalDir = ".\\Job\\STEP";
	const std::string sLocalProgramFile = sLocalDir + "\\" + sProgramName + ".srp";
	const std::string sLocalDataFile = sLocalDir + "\\" + sProgramName + ".srd";
	const std::string sRemoteBaseDir = "/UserPrograms/" + sProjectName + ".sr";
	const std::string sRemoteProgramFile = sRemoteBaseDir + "/" + sProgramName + ".srp";
	const std::string sRemoteDataFile = sRemoteBaseDir + "/" + sProgramName + ".srd";

	try
	{
		std::filesystem::create_directories(sLocalDir);
	}
	catch (const std::exception& e)
	{
		m_pRobotLog->write(LogColor::ERR, "STEP ContiMoveAny 创建本地目录失败：%s", e.what());
		return -2;
	}

	const std::string sSrpContent = StepBuildSrpContent(vtRobotMoveInfo);
	const std::string sSrdContent = StepBuildSrdContent(vtRobotMoveInfo, m_tAxisUnit);

	if (!StepWriteTextFile(sLocalProgramFile, sSrpContent))
	{
		m_pRobotLog->write(LogColor::ERR, "STEP ContiMoveAny 写入SRP失败：%s", sLocalProgramFile.c_str());
		return -3;
	}
	if (!StepWriteTextFile(sLocalDataFile, sSrdContent))
	{
		m_pRobotLog->write(LogColor::ERR, "STEP ContiMoveAny 写入SRD失败：%s", sLocalDataFile.c_str());
		return -4;
	}

	m_pRobotLog->write(LogColor::SUCCESS,
		"STEP ContiMoveAny 已生成程序 | Project=%s | Program=%s | PointCount=%d",
		sProjectName.c_str(), sProgramName.c_str(), static_cast<int>(vtRobotMoveInfo.size()));

	if (m_pFTP == nullptr)
	{
		InitFtp();
	}
	if (m_pFTP == nullptr)
	{
		m_pRobotLog->write(LogColor::ERR, "STEP ContiMoveAny 初始化FTP失败");
		return -5;
	}

	if (!m_pFTP->uploadFile(sLocalProgramFile, sRemoteProgramFile))
	{
		m_pRobotLog->write(LogColor::ERR, "STEP ContiMoveAny 上传SRP失败：%s", sRemoteProgramFile.c_str());
		return -6;
	}
	if (!m_pFTP->uploadFile(sLocalDataFile, sRemoteDataFile))
	{
		m_pRobotLog->write(LogColor::ERR, "STEP ContiMoveAny 上传SRD失败：%s", sRemoteDataFile.c_str());
		return -7;
	}

	m_pRobotLog->write(LogColor::SUCCESS,
		"STEP ContiMoveAny 上传完成 | SRP=%s | SRD=%s",
		sRemoteProgramFile.c_str(), sRemoteDataFile.c_str());

	const std::string sCurrentProgram = GetUserProgram();
	if (!sCurrentProgram.empty())
	{
		if (!UnLoadUserProgramer())
		{
			m_pRobotLog->write(LogColor::ERR, "STEP ContiMoveAny 卸载当前程序失败：%s", sCurrentProgram.c_str());
			return -8;
		}
		m_pRobotLog->write(LogColor::SUCCESS, "STEP ContiMoveAny 已卸载当前程序：%s", sCurrentProgram.c_str());
	}

	if (!LoadUserProgram(sProjectName, sProgramName))
	{
		m_pRobotLog->write(LogColor::ERR,
			"STEP ContiMoveAny 加载程序失败 | Project=%s | Program=%s",
			sProjectName.c_str(), sProgramName.c_str());
		return -9;
	}

	m_pRobotLog->write(LogColor::SUCCESS,
		"STEP ContiMoveAny 已加载程序 | Project=%s | Program=%s",
		sProjectName.c_str(), sProgramName.c_str());

	if (!Prog_startRun_Py())
	{
		m_pRobotLog->write(LogColor::ERR, "STEP ContiMoveAny 启动程序失败：%s", sProgramName.c_str());
		return -10;
	}

	m_pRobotLog->write(LogColor::SUCCESS, "STEP ContiMoveAny 已启动程序：%s", sProgramName.c_str());

	return 0;
}

bool STEPRobotCtrl::ServoOff()
{
	int nRet = 0;
	//当前机器人未使能状态
	if (m_pSTEPRobotClient->getMotorEnableState() == 1)
	{
		nRet = m_pSTEPRobotClient->EnableMotorCmd();
	}
	if (nRet != 0)
	{
		showErrorMessage(
			nullptr,
			"下使能失败,失败原因:%s",
			GetErrorText(nRet)   // 直接用全局错误库
		);
		return false;
	}
	return true;
}

bool STEPRobotCtrl::ServoOn()
{
	int nRet = 0;
	//当前机器人未使能状态
	if (m_pSTEPRobotClient->getMotorEnableState() == 0)
	{
		nRet = m_pSTEPRobotClient->EnableMotorCmd();
	}
	if (nRet != 0)
	{
		showErrorMessage(
			nullptr,
			"下使能失败,失败原因:%s",
			GetErrorText(nRet)   // 直接用全局错误库
		);
		return false;
	}
	return true;
}

bool STEPRobotCtrl::SetRobotToolNo(int nToolNo)
{
	int nRet = 0;
	std::string sToolName = GetStr("TOOL%d",nToolNo);
	nRet = m_pSTEPRobotClient->ToolSetCmd(sToolName);

	if (nRet != 0)
	{
		showErrorMessage(
			nullptr,
			"工具设置失败,失败原因:%s",
			GetErrorText(nRet)   // 直接用全局错误库
		);
		return false;
	}
	return true;
}

bool STEPRobotCtrl::GetToolData(int nToolNo, T_ROBOT_COORS adRobotToolData)
{
	int nRet = 0;
	adRobotToolData = T_ROBOT_COORS();
	std::string sToolName = GetStr("TOOL%d", nToolNo);
	STEPROBOTSDK::Tool tTool = { false,0,0,0,0,0,0 };
	nRet = m_pSTEPRobotClient->VariableToolReadCmd(sToolName, tTool);

	if (nRet != 0)
	{
		showErrorMessage(
			nullptr,
			"工具获取失败,失败原因:%s",
			GetErrorText(nRet)   // 直接用全局错误库
		);
		return false;
	}

	adRobotToolData.dX = tTool.X;
	adRobotToolData.dY = tTool.Y;
	adRobotToolData.dZ = tTool.Z;
	adRobotToolData.dRX = tTool.A;
	adRobotToolData.dRY = tTool.B;
	adRobotToolData.dRZ = tTool.C;
	return true;
}

//FTP建立连接
int STEPRobotCtrl::InitFtp()
{
	m_pFTP = new FtpClient(m_pRobotLog, m_sFTPIP, m_nFTPPort, m_sFTPUser, m_sFTPPassWord);
	return 0;
}

//上传文件给埃斯顿机器人，埃斯顿为RemoteFilePath，本地为LocalFilePath    //  .//MultiPos_Mv1.erd 
int STEPRobotCtrl::UploadFile(std::string LocalFilePath, std::string RemoteFilePath)
{
	return m_pFTP->uploadFile(LocalFilePath, RemoteFilePath);
}
//下载文件,埃斯顿为RemoteFilePath，本地为LocalFilePath
int STEPRobotCtrl::DownloadFile(std::string RemoteFilePath, std::string LocalFilePath)
{
	return m_pFTP->downloadFile(RemoteFilePath,LocalFilePath);
}

//清除报警信息+
bool STEPRobotCtrl::cleanAlarm()
{
	int nRet = 0;
	nRet = m_pSTEPRobotClient->AllAlarmConfirmCmd();

	if (nRet != 0)
	{
		showErrorMessage(
			nullptr,
			"清除报错失败,原因:%s",
			GetErrorText(nRet)   // 直接用全局错误库
		);
		return false;
	}

	return true;
}

bool STEPRobotCtrl::SetTpSpeed(int speed)
{
	if (speed < 0 || speed > 100)
	{
		showErrorMessage(nullptr, "设置示教器速度失败,失败原因:速度范围错误(%d)", speed);
		return false;
	}

	const int nRet = m_pSTEPRobotClient->OverrideSetCmd(static_cast<double>(speed));
	if (nRet != 0)
	{
		showErrorMessage(nullptr, "设置示教器速度失败,失败原因:%s", GetErrorText(nRet));
		return false;
	}
	return true;
}

bool STEPRobotCtrl::SetPosVar(int nIndex, double pos[8], int nPVarType, int isconfig, int config[7], int scoper, int Coord)
{
	(void)isconfig;
	(void)config;
	(void)Coord;

	const std::string sProjectName = GetUserProject();
	const std::string sProgramName = StepGetProgramScopeName(this, scoper);
	const std::string sVarName = StepBuildPosVarName(nIndex, nPVarType);

	if (nPVarType == PULSEVAR)
	{
		T_ANGLE_PULSE tPulse;
		tPulse.nSPulse = static_cast<long>(pos[0]);
		tPulse.nLPulse = static_cast<long>(pos[1]);
		tPulse.nUPulse = static_cast<long>(pos[2]);
		tPulse.nRPulse = static_cast<long>(pos[3]);
		tPulse.nBPulse = static_cast<long>(pos[4]);
		tPulse.nTPulse = static_cast<long>(pos[5]);
		tPulse.lBXPulse = static_cast<long>(pos[6]);
		tPulse.lBYPulse = static_cast<long>(pos[7]);
		tPulse.lBZPulse = 0;
		return SetPosVar(nIndex, tPulse, scoper);
	}

	T_ROBOT_COORS tRobotCoors;
	tRobotCoors.dX = pos[0];
	tRobotCoors.dY = pos[1];
	tRobotCoors.dZ = pos[2];
	tRobotCoors.dRX = pos[3];
	tRobotCoors.dRY = pos[4];
	tRobotCoors.dRZ = pos[5];
	tRobotCoors.dBX = pos[6];
	tRobotCoors.dBY = pos[7];
	tRobotCoors.dBZ = 0;

	const CARTPOS value = StepToCartPos(tRobotCoors);
	const int nRet = m_pSTEPRobotClient->VariableCartposModifyCmd(sProjectName, sProgramName, sVarName, value);
	if (nRet != 0)
	{
		showErrorMessage(nullptr, "设置位置变量失败:%s,原因:%s", sVarName.c_str(), GetErrorText(nRet));
		return false;
	}
	return true;
}

void STEPRobotCtrl::SetPosVar(int nIndex, T_ROBOT_COORS tRobotCoors, int isconfig, int config[7], int scoper)
{
	double pos[8] = {
		tRobotCoors.dX, tRobotCoors.dY, tRobotCoors.dZ,
		tRobotCoors.dRX, tRobotCoors.dRY, tRobotCoors.dRZ,
		tRobotCoors.dBX, tRobotCoors.dBY
	};
	(void)SetPosVar(nIndex, pos, POSVAR, isconfig, config, scoper, POSVAR);
}

bool STEPRobotCtrl::SetPosVar(int nIndex, T_ANGLE_PULSE tRobotPulse, int scoper)
{
	const std::string sProjectName = GetUserProject();
	const std::string sProgramName = StepGetProgramScopeName(this, scoper);
	const std::string sVarName = StepBuildPosVarName(nIndex, PULSEVAR);
	const AXISPOS value = StepToAxisPos(tRobotPulse, m_tAxisUnit);

	const int nRet = m_pSTEPRobotClient->VariableAxisposModifyCmd(sProjectName, sProgramName, sVarName, value);
	if (nRet != 0)
	{
		showErrorMessage(nullptr, "设置关节位置变量失败:%s,原因:%s", sVarName.c_str(), GetErrorText(nRet));
		return false;
	}
	return true;
}

bool STEPRobotCtrl::SetPosVar(int nIndex, AXISPOS eRobotCoors, int scoper)
{
	const std::string sProjectName = GetUserProject();
	const std::string sProgramName = StepGetProgramScopeName(this, scoper);
	const std::string sVarName = StepBuildPosVarName(nIndex, PULSEVAR);
	const int nRet = m_pSTEPRobotClient->VariableAxisposModifyCmd(sProjectName, sProgramName, sVarName, eRobotCoors);
	if (nRet != 0)
	{
		showErrorMessage(nullptr, "设置AXISPOS变量失败:%s,原因:%s", sVarName.c_str(), GetErrorText(nRet));
		return false;
	}
	return true;
}

bool STEPRobotCtrl::SetPosVar(int nIndex, JointsPos eRobotCoors, int scoper)
{
	const std::string sProjectName = GetUserProject();
	const std::string sProgramName = StepGetProgramScopeName(this, scoper);
	const std::string sVarName = StepBuildPosVarName(nIndex, PULSEVAR);
	const int nRet = m_pSTEPRobotClient->VariableRobotAxisposModifyCmd(sProjectName, sProgramName, sVarName, eRobotCoors);
	if (nRet != 0)
	{
		showErrorMessage(nullptr, "设置JointsPos变量失败:%s,原因:%s", sVarName.c_str(), GetErrorText(nRet));
		return false;
	}
	return true;
}

int STEPRobotCtrl::GetPosVar(long lPvarIndex, double array[6], int config[7], int MoveType)
{
	(void)config;
	const std::string sProjectName = GetUserProject();
	const std::string sProgramName = GetUserProgram();
	const std::string sVarName = StepBuildPosVarName(static_cast<int>(lPvarIndex), MoveType);

	if (MoveType == PULSEVAR)
	{
		AXISPOS value = {};
		const int nRet = m_pSTEPRobotClient->VariableAxisposReadCmd(sProjectName, sProgramName, sVarName, value);
		if (nRet != 0)
		{
			return nRet;
		}
		for (int i = 0; i < 6; ++i)
		{
			array[i] = value.m_Joint[i];
		}
		return 0;
	}

	CARTPOS value = {};
	const int nRet = m_pSTEPRobotClient->VariableCartposReadCmd(sProjectName, sProgramName, sVarName, value);
	if (nRet != 0)
	{
		return nRet;
	}
	for (int i = 0; i < 6; ++i)
	{
		array[i] = value.m_CartPos.cart[i];
	}
	return 0;
}

bool STEPRobotCtrl::SetSpeed(const char* name, double* speed, int scord)
{
	if (name == nullptr || speed == nullptr)
	{
		return false;
	}

	const SDynamicPercent value = StepToDynamicPercent(speed[0], speed[1], speed[2], speed[3], speed[4]);
	const std::string sProjectName = GetUserProject();
	const std::string sProgramName = StepGetProgramScopeName(this, scord);
	const int nRet = m_pSTEPRobotClient->VariableDynamicModifyCmd(sProjectName, sProgramName, name, value);
	if (nRet != 0)
	{
		showErrorMessage(nullptr, "设置速度变量失败:%s,原因:%s", name, GetErrorText(nRet));
		return false;
	}
	return true;
}

bool STEPRobotCtrl::SetSpeed(int nIndex, double adSpeed[5])
{
	const std::string sVarName = GetStr("dyn%d", nIndex);
	return SetSpeed(sVarName.c_str(), adSpeed, PROGRAMVAR);
}

bool STEPRobotCtrl::SetSpeed(int nIndex, SDynamicPercent adSpeed)
{
	const std::string sVarName = GetStr("dyn%d", nIndex);
	const std::string sProjectName = GetUserProject();
	const std::string sProgramName = GetUserProgram();
	const int nRet = m_pSTEPRobotClient->VariableDynamicModifyCmd(sProjectName, sProgramName, sVarName, adSpeed);
	if (nRet != 0)
	{
		showErrorMessage(nullptr, "设置动态速度变量失败:%s,原因:%s", sVarName.c_str(), GetErrorText(nRet));
		return false;
	}
	return true;
}

int STEPRobotCtrl::GetIntVar(int nIndex, const char* cStrPreFix)
{
	const std::string sVarName = GetStr("%s%d", cStrPreFix, nIndex);
	const std::string sProjectName = GetUserProject();
	const std::string sProgramName = GetUserProgram();
	int value = 0;
	const int nRet = m_pSTEPRobotClient->VariableIntReadCmd(sProjectName, sProgramName, sVarName, value);
	return nRet == 0 ? value : 0;
}

bool STEPRobotCtrl::SetIntVar(int nIndex, int nValue, int score, const char* cStrPreFix)
{
	const std::string sVarName = GetStr("%s%d", cStrPreFix, nIndex);
	return SetIntVar(sVarName.c_str(), nValue, score);
}

bool STEPRobotCtrl::SetIntVar(const char* name, int value, int score)
{
	if (name == nullptr)
	{
		return false;
	}
	const std::string sProjectName = GetUserProject();
	const std::string sProgramName = StepGetProgramScopeName(this, score);
	const int nRet = m_pSTEPRobotClient->VariableIntModifyCmd(sProjectName, sProgramName, name, value);
	if (nRet != 0)
	{
		showErrorMessage(nullptr, "设置INT变量失败:%s,原因:%s", name, GetErrorText(nRet));
		return false;
	}
	return true;
}

bool STEPRobotCtrl::SetRealVar(int nIndex, double value, const char* cStrPreFix, int score)
{
	const std::string sVarName = GetStr("%s%d", cStrPreFix, nIndex);
	const std::string sProjectName = GetUserProject();
	const std::string sProgramName = StepGetProgramScopeName(this, score);
	const int nRet = m_pSTEPRobotClient->VariableRealModifyCmd(sProjectName, sProgramName, sVarName, value);
	if (nRet != 0)
	{
		showErrorMessage(nullptr, "设置REAL变量失败:%s,原因:%s", sVarName.c_str(), GetErrorText(nRet));
		return false;
	}
	return true;
}

bool STEPRobotCtrl::WaitStateDone(
	int nStateReg,
	int nDoneState,
	int nStartStateA,
	int nStartStateB,
	int nStartTimeoutMs,
	int nFinishTimeoutMs,
	int nDelayTime,
	int* pLastState)
{
	(void)nStateReg;
	const int delay = std::max(1, nDelayTime);
	int state = CheckDone();
	if (pLastState != nullptr)
	{
		*pLastState = state;
	}

	bool observedStart = state == nStartStateA || state == nStartStateB;
	for (int elapsed = 0; !observedStart && elapsed < nStartTimeoutMs; elapsed += delay)
	{
		Sleep(delay);
		state = CheckDone();
		if (pLastState != nullptr)
		{
			*pLastState = state;
		}
		observedStart = state == nStartStateA || state == nStartStateB;
	}

	if (!observedStart && nStartTimeoutMs > 0)
	{
		if (m_pRobotLog != nullptr)
		{
			m_pRobotLog->write(LogColor::WARNING,
				"STEP WaitStateDone 未观察到启动状态，继续等待完成 | LastState=%d",
				state);
		}
	}

	for (int elapsed = 0; elapsed < nFinishTimeoutMs; elapsed += delay)
	{
		state = CheckDone();
		if (pLastState != nullptr)
		{
			*pLastState = state;
		}
		if (state == nDoneState)
		{
			return true;
		}
		Sleep(delay);
	}

	if (m_pRobotLog != nullptr)
	{
		m_pRobotLog->write(LogColor::ERR,
			"STEP WaitStateDone 等待完成超时 | DoneState=%d | LastState=%d",
			nDoneState, state);
	}
	return false;
}

bool STEPRobotCtrl::CallJobAndWaitStateDone(
	std::string sJobName,
	int nStateReg,
	int nDoneState,
	int nStartStateA,
	int nStartStateB,
	int nStartTimeoutMs,
	int nFinishTimeoutMs,
	int nDelayTime,
	int* pLastState,
	bool bResetStateBeforeCall)
{
	(void)bResetStateBeforeCall;
	if (!CallJob(sJobName))
	{
		return false;
	}
	return WaitStateDone(
		nStateReg,
		nDoneState,
		nStartStateA,
		nStartStateB,
		nStartTimeoutMs,
		nFinishTimeoutMs,
		nDelayTime,
		pLastState);
}

bool STEPRobotCtrl::AxisPulseMove(int nAxisNo, long lDist, long lRobotSpd, int nCoorType, int nMovtype, int nToolNo, long lCoordFrm)
{
	(void)nCoorType;
	(void)nToolNo;
	(void)lCoordFrm;

	const int axisIndex = StepNormalizeAxisIndex(nAxisNo);
	if (axisIndex < 0 || axisIndex > 8)
	{
		showErrorMessage(nullptr, "STEP 关节单轴运动失败,轴号越界:%d", nAxisNo);
		return false;
	}

	T_ANGLE_PULSE target = GetCurrentPulse();
	const long current = StepGetPulseByIndex(target, axisIndex);
	StepSetPulseByIndex(target, axisIndex, nMovtype == ABSVAR ? lDist : current + lDist);

	return MoveByJob(target, T_ROBOT_MOVE_SPEED(static_cast<double>(lRobotSpd), 0.0, 0.0), m_nExternalAxleType, "MOVJ");
}

bool STEPRobotCtrl::PosMove(int nAxisNo, double dDist, long lRobotSpd, int nCoorType, int nMovtype, int config[7], int nToolNo, long lCoordFrm)
{
	(void)config;
	(void)nToolNo;
	(void)lCoordFrm;

	if (nCoorType == PULSEVAR)
	{
		return AxisPulseMove(nAxisNo, static_cast<long>(std::lround(dDist)), lRobotSpd, nCoorType, nMovtype, nToolNo, lCoordFrm);
	}

	const int axisIndex = StepNormalizeAxisIndex(nAxisNo);
	if (axisIndex < 0 || axisIndex > 8)
	{
		showErrorMessage(nullptr, "STEP 直角单轴运动失败,轴号越界:%d", nAxisNo);
		return false;
	}

	T_ROBOT_COORS target = GetCurrentPos();
	const double current = StepGetCoorsByIndex(target, axisIndex);
	StepSetCoorsByIndex(target, axisIndex, nMovtype == ABSVAR ? dDist : current + dDist);

	return MoveByJob(target, T_ROBOT_MOVE_SPEED(static_cast<double>(lRobotSpd), 0.0, 0.0), m_nExternalAxleType, "MOVL");
}

bool STEPRobotCtrl::PosMove(int nAxisNo, double dDist, long lRobotSpd, int nCoorType, int nMovtype, int nToolNo, long lCoordFrm)
{
	int config[7] = { 0 };
	return PosMove(nAxisNo, dDist, lRobotSpd, nCoorType, nMovtype, config, nToolNo, lCoordFrm);
}

bool STEPRobotCtrl::MoveByJob(int Axis, double Distence, int config[7], double speed, int ifAbsoluteM, int ifJoint)
{
	const int moveType = ifAbsoluteM ? ABSVAR : RELVAR;
	if (ifJoint)
	{
		return AxisPulseMove(Axis, static_cast<long>(std::lround(Distence)), static_cast<long>(std::lround(speed)), PULSEVAR, moveType);
	}
	return PosMove(Axis, Distence, static_cast<long>(std::lround(speed)), POSVAR, moveType, config);
}

bool STEPRobotCtrl::MoveByJob(double Distence[8], int config[7], double speed, int ifAbsolutM, int ifJoint)
{
	(void)config;
	if (Distence == nullptr)
	{
		return false;
	}

	if (ifJoint)
	{
		T_ANGLE_PULSE target = ifAbsolutM ? T_ANGLE_PULSE() : GetCurrentPulse();
		const long values[8] =
		{
			static_cast<long>(std::lround(Distence[0])),
			static_cast<long>(std::lround(Distence[1])),
			static_cast<long>(std::lround(Distence[2])),
			static_cast<long>(std::lround(Distence[3])),
			static_cast<long>(std::lround(Distence[4])),
			static_cast<long>(std::lround(Distence[5])),
			static_cast<long>(std::lround(Distence[6])),
			static_cast<long>(std::lround(Distence[7]))
		};
		for (int i = 0; i < 8; ++i)
		{
			const long current = ifAbsolutM ? 0 : StepGetPulseByIndex(target, i);
			StepSetPulseByIndex(target, i, current + values[i]);
		}
		return MoveByJob(target, T_ROBOT_MOVE_SPEED(speed, 0.0, 0.0), m_nExternalAxleType, "MOVJ");
	}

	T_ROBOT_COORS target = ifAbsolutM ? T_ROBOT_COORS() : GetCurrentPos();
	for (int i = 0; i < 8; ++i)
	{
		const double current = ifAbsolutM ? 0.0 : StepGetCoorsByIndex(target, i);
		StepSetCoorsByIndex(target, i, current + Distence[i]);
	}
	return MoveByJob(target, T_ROBOT_MOVE_SPEED(speed, 0.0, 0.0), m_nExternalAxleType, "MOVL");
}

bool STEPRobotCtrl::MoveByJob(double* dRobotJointCoord, T_ROBOT_MOVE_SPEED tPulseMove, int nExternalAxleType, int nPVarType, std::string JobName, int config[7])
{
	(void)nExternalAxleType;
	(void)config;
	if (dRobotJointCoord == nullptr)
	{
		return false;
	}

	const int moveType = StepMoveTypeFromJobName(JobName);
	if (nPVarType == POSVAR || moveType == MOVL)
	{
		T_ROBOT_COORS target(
			dRobotJointCoord[0], dRobotJointCoord[1], dRobotJointCoord[2],
			dRobotJointCoord[3], dRobotJointCoord[4], dRobotJointCoord[5],
			dRobotJointCoord[6], dRobotJointCoord[7], 0.0);
		return MoveByJob(target, tPulseMove, nExternalAxleType, "MOVL");
	}

	T_ANGLE_PULSE target(
		static_cast<long>(std::lround(dRobotJointCoord[0])),
		static_cast<long>(std::lround(dRobotJointCoord[1])),
		static_cast<long>(std::lround(dRobotJointCoord[2])),
		static_cast<long>(std::lround(dRobotJointCoord[3])),
		static_cast<long>(std::lround(dRobotJointCoord[4])),
		static_cast<long>(std::lround(dRobotJointCoord[5])),
		static_cast<long>(std::lround(dRobotJointCoord[6])),
		static_cast<long>(std::lround(dRobotJointCoord[7])),
		0);
	return MoveByJob(target, tPulseMove, nExternalAxleType, "MOVJ");
}

bool STEPRobotCtrl::MoveByJob(T_ROBOT_COORS tRobotJointCoord, T_ROBOT_MOVE_SPEED tPulseMove, int nExternalAxleType, std::string JobName, int isconfig, int config[7])
{
	(void)nExternalAxleType;
	(void)JobName;
	(void)isconfig;
	(void)config;

	T_ROBOT_MOVE_INFO moveInfo;
	moveInfo.nMoveType = MOVL;
	moveInfo.tCoord = tRobotJointCoord;
	moveInfo.tSpeed = tPulseMove;
	return StepRunSimulateMove(this, moveInfo);
}

bool STEPRobotCtrl::MoveByJob(T_ANGLE_PULSE tRobotJointCoord, T_ROBOT_MOVE_SPEED tPulseMove, int nExternalAxleType, std::string JobName)
{
	(void)nExternalAxleType;
	(void)JobName;

	T_ROBOT_MOVE_INFO moveInfo;
	moveInfo.nMoveType = MOVJ;
	moveInfo.tPulse = tRobotJointCoord;
	moveInfo.tSpeed = tPulseMove;
	return StepRunSimulateMove(this, moveInfo);
}

bool STEPRobotCtrl::SendWeldTriangleWeaveProgram(int nWeldTrackNum)
{
	(void)nWeldTrackNum;
	if (m_pRobotLog != nullptr)
	{
		m_pRobotLog->write(LogColor::WARNING, "STEP 三角摆焊程序下发暂未绑定具体工艺参数，已拒绝执行");
	}
	return false;
}

bool STEPRobotCtrl::SendWeldLWeaveProgram(int nWeldTrackNum)
{
	(void)nWeldTrackNum;
	if (m_pRobotLog != nullptr)
	{
		m_pRobotLog->write(LogColor::WARNING, "STEP L型摆焊程序下发暂未绑定具体工艺参数，已拒绝执行");
	}
	return false;
}

bool STEPRobotCtrl::SendWeldProgram(int nWeldTrackNum)
{
	(void)nWeldTrackNum;
	if (m_pRobotLog != nullptr)
	{
		m_pRobotLog->write(LogColor::WARNING, "STEP 普通焊接程序下发暂未绑定具体工艺参数，已拒绝执行");
	}
	return false;
}
