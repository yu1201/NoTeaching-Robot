#include "STEPRobotDriver.h"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

namespace
{
	std::string StepMakeProgramName()
	{
		return "ContiMoveAny";
	}

	constexpr const char* kStepDynamicJobProjectName = "PCRobot";
	constexpr const char* kStepProjectVariableProgramName = "_project";

	double StepClampPositiveDouble(double value, double defaultValue)
	{
		if (!std::isfinite(value) || value <= 0.0)
		{
			return defaultValue;
		}
		return value;
	}

	double StepLinearSpeedMmPerSecFromConfig(double speedMmPerMin, double fallbackMmPerSec = 1.0)
	{
		if (!std::isfinite(speedMmPerMin) || speedMmPerMin <= 0.0)
		{
			return fallbackMmPerSec;
		}
		const double converted = speedMmPerMin / 60.0;
		return converted > 0.0 ? converted : fallbackMmPerSec;
	}

	double StepJointSpeedPercent(double speed)
	{
		if (!std::isfinite(speed) || speed <= 0.0)
		{
			return 20.0;
		}

		double percent = speed;
		if (percent > 100.0)
		{
			percent /= 100.0;
		}
		return std::clamp(percent, 1.0, 100.0);
	}

	struct StepDynamicValues
	{
		double segmentVel = 1.0;
		double segmentAcc = 3.0;
		double segmentDec = 3.0;
		double segmentJerk = 50000.0;
		double oriVel = 90.0;
		double oriAcc = 270.0;
		double oriDec = 270.0;
		double oriJerk = 2700.0;
		double jointVel = 20.0;
		double jointAcc = 40.0;
		double jointDec = 40.0;
		double jointJerk = 200.0;
	};

	StepDynamicValues StepBuildDynamicValues(const T_ROBOT_MOVE_SPEED& speed)
	{
		StepDynamicValues value;
		value.segmentVel = StepLinearSpeedMmPerSecFromConfig(speed.dSpeed, 1.0);
		value.segmentAcc = speed.dACC > 0.0
			? StepLinearSpeedMmPerSecFromConfig(speed.dACC, value.segmentVel * 3.0)
			: value.segmentVel * 3.0;
		value.segmentDec = speed.dDEC > 0.0
			? StepLinearSpeedMmPerSecFromConfig(speed.dDEC, value.segmentVel * 3.0)
			: value.segmentVel * 3.0;
		value.segmentJerk = std::max(50000.0, value.segmentAcc * 10.0);

		value.jointVel = StepJointSpeedPercent(speed.dSpeed);
		value.jointAcc = std::clamp(value.jointVel * 2.0, 1.0, 200.0);
		value.jointDec = value.jointAcc;
		value.jointJerk = 200.0;
		return value;
	}

	std::string StepBuildDynamicName(size_t index)
	{
		return GetStr("ntdyn%u", static_cast<unsigned>(index));
	}

	bool StepHasSrSuffix(const std::string& value)
	{
		if (value.size() < 3)
		{
			return false;
		}

		const char dot = value[value.size() - 3];
		const char s = value[value.size() - 2];
		const char r = value[value.size() - 1];
		return dot == '.'
			&& (s == 's' || s == 'S')
			&& (r == 'r' || r == 'R');
	}

	std::string StepNormalizeProjectName(std::string projectName)
	{
		while (StepHasSrSuffix(projectName))
		{
			projectName.resize(projectName.size() - 3);
		}
		return projectName;
	}

	std::string StepBuildRemoteProjectDir(const std::string& projectName)
	{
		return "/UserPrograms/" + StepNormalizeProjectName(projectName) + ".sr";
	}

	std::string StepCString(const char* text, size_t capacity)
	{
		if (text == nullptr)
		{
			return std::string();
		}
		size_t length = 0;
		while (length < capacity && text[length] != '\0')
		{
			++length;
		}
		return std::string(text, length);
	}

	const char* StepProgramStateText(int state)
	{
		switch (state)
		{
		case STEPROBOTSDK::eRun: return "运行";
		case STEPROBOTSDK::ePause: return "暂停";
		case STEPROBOTSDK::eStop: return "停止";
		case STEPROBOTSDK::eReturn: return "未知";
		default: return "未识别";
		}
	}

	const char* StepMessageTypeText(STEPROBOTSDK::MESSAGETYPE type)
	{
		switch (type)
		{
		case STEPROBOTSDK::eInfo: return "Info";
		case STEPROBOTSDK::eWarning: return "Warning";
		case STEPROBOTSDK::eError: return "Error";
		default: return "Unknown";
		}
	}

	std::string StepBuildCartPosName(size_t index)
	{
		return GetStr("ntcp%u", static_cast<unsigned>(index));
	}

	std::string StepBuildAxisPosName(size_t index)
	{
		return GetStr("ntap%u", static_cast<unsigned>(index));
	}

	std::vector<std::string> StepBuildToolNameCandidates(int toolNo)
	{
		return {
			GetStr("tool%d", toolNo),
			GetStr("TOOL%d", toolNo)
		};
	}

	bool StepIsInvalidExternalPulse(long pulse)
	{
		return pulse <= static_cast<long>(std::numeric_limits<int>::min()) + 1L;
	}

	double StepPulseToPosition(long pulse, double pulseUnit)
	{
		if (!std::isfinite(pulseUnit) || std::abs(pulseUnit) < 1e-12)
		{
			return static_cast<double>(pulse);
		}
		return static_cast<double>(pulse) * pulseUnit;
	}

	double StepExternalPulseToPosition(long pulse, double pulseUnit)
	{
		if (StepIsInvalidExternalPulse(pulse))
		{
			return 0.0;
		}
		return StepPulseToPosition(pulse, pulseUnit);
	}

	std::string StepBuildOverlapName(size_t index)
	{
		return GetStr("ntolr%u", static_cast<unsigned>(index));
	}

	std::string StepBuildSrdContent(const std::vector<T_ROBOT_MOVE_INFO>& moveInfos, const T_AXISUNIT& axisUnit)
	{
		std::ostringstream oss;
		oss << std::fixed << std::setprecision(6);

		for (size_t i = 0; i < moveInfos.size(); ++i)
		{
			const T_ROBOT_MOVE_INFO& info = moveInfos[i];
			const std::string cartName = StepBuildCartPosName(i);
			const std::string axisName = StepBuildAxisPosName(i);
			const std::string dynName = StepBuildDynamicName(i);

			if (info.nPosType == PULSEVAR)
			{
				oss << "AXISPOS " << axisName << " := {  "
					<< StepPulseToPosition(info.tPulse.nSPulse, axisUnit.dSPulseUnit) << ", "
					<< StepPulseToPosition(info.tPulse.nLPulse, axisUnit.dLPulseUnit) << ", "
					<< StepPulseToPosition(info.tPulse.nUPulse, axisUnit.dUPulseUnit) << ", "
					<< StepPulseToPosition(info.tPulse.nRPulse, axisUnit.dRPulseUnit) << ", "
					<< StepPulseToPosition(info.tPulse.nBPulse, axisUnit.dBPulseUnit) << ", "
					<< StepPulseToPosition(info.tPulse.nTPulse, axisUnit.dTPulseUnit) << ", "
					<< StepExternalPulseToPosition(info.tPulse.lBXPulse, axisUnit.dBXPulseUnit) << ", "
					<< StepExternalPulseToPosition(info.tPulse.lBYPulse, axisUnit.dBYPulseUnit) << ", "
					<< StepExternalPulseToPosition(info.tPulse.lBZPulse, axisUnit.dBZPulseUnit)
					<< ", 0.0, 0.0,0.0 }" << "\n";
			}
			else
			{
				oss << "CARTPOS " << cartName << " := {  "
					<< info.tCoord.dX << ", " << info.tCoord.dY << ", " << info.tCoord.dZ << ", "
					<< info.tCoord.dRX << ", " << info.tCoord.dRY << ", " << info.tCoord.dRZ << ", "
					<< info.adBasePosVar[0] << ", " << info.adBasePosVar[1] << ", " << info.adBasePosVar[2]
					<< ", 0.0, 0.0, 0.0,0 }" << "\n";
			}

			const StepDynamicValues dyn = StepBuildDynamicValues(info.tSpeed);

			oss << "DYNAMIC " << dynName << " := {  "
				<< StepClampPositiveDouble(dyn.segmentVel, 1.0) << ", "
				<< StepClampPositiveDouble(dyn.segmentAcc, 3.0) << ", "
				<< StepClampPositiveDouble(dyn.segmentDec, 3.0) << ", "
				<< StepClampPositiveDouble(dyn.segmentJerk, 50000.0) << ", "
				<< dyn.oriVel << ", " << dyn.oriAcc << ", " << dyn.oriDec << ", " << dyn.oriJerk << ", "
				<< dyn.jointVel << ", " << dyn.jointAcc << ", " << dyn.jointDec << "," << dyn.jointJerk
				<< " }" << "\n";
			oss << "OVERLAPREL " << StepBuildOverlapName(i) << " := 20" << "\n";
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
			const std::string overlapName = StepBuildOverlapName(i);
			const std::string targetName = info.nPosType == PULSEVAR ? axisName : cartName;

			if (info.nMoveType == MOVL)
			{
				oss << "Lin(" << targetName << "," << dynName << "," << overlapName << ",NULL,tool1,WORLD);" << "\n";
			}
			else
			{
				oss << "PTP(" << targetName << "," << dynName << "," << overlapName << ",tool1,WORLD);" << "\n";
			}
		}

		return oss.str();
	}

	bool StepWriteTextFile(const std::string& filePath, const std::string& content)
	{
		std::ofstream out(filePath, std::ios::out | std::ios::binary | std::ios::trunc);
		if (!out.is_open())
		{
			return false;
		}

		std::string normalized;
		normalized.reserve(content.size());
		for (size_t i = 0; i < content.size(); ++i)
		{
			if (content[i] == '\r')
			{
				if (i + 1 < content.size() && content[i + 1] == '\n')
				{
					continue;
				}
				normalized.push_back('\n');
				continue;
			}
			normalized.push_back(content[i]);
		}

		out.write(normalized.data(), static_cast<std::streamsize>(normalized.size()));
		return out.good();
	}

	std::string StepGetProgramScopeName(STEPRobotCtrl* ctrl, int scoper)
	{
		if (ctrl == nullptr)
		{
			return std::string();
		}
		return scoper == PROGRAMVAR ? ctrl->GetUserProgram() : std::string(kStepProjectVariableProgramName);
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

	AXISPOS StepToAxisPos(const T_ANGLE_PULSE& tRobotPulse, const T_AXISUNIT& axisUnit)
	{
		AXISPOS value = {};
		value.m_Joint[0] = StepPulseToPosition(tRobotPulse.nSPulse, axisUnit.dSPulseUnit);
		value.m_Joint[1] = StepPulseToPosition(tRobotPulse.nLPulse, axisUnit.dLPulseUnit);
		value.m_Joint[2] = StepPulseToPosition(tRobotPulse.nUPulse, axisUnit.dUPulseUnit);
		value.m_Joint[3] = StepPulseToPosition(tRobotPulse.nRPulse, axisUnit.dRPulseUnit);
		value.m_Joint[4] = StepPulseToPosition(tRobotPulse.nBPulse, axisUnit.dBPulseUnit);
		value.m_Joint[5] = StepPulseToPosition(tRobotPulse.nTPulse, axisUnit.dTPulseUnit);
		value.m_AuxJoint[0] = StepExternalPulseToPosition(tRobotPulse.lBXPulse, axisUnit.dBXPulseUnit);
		value.m_AuxJoint[1] = StepExternalPulseToPosition(tRobotPulse.lBYPulse, axisUnit.dBYPulseUnit);
		value.m_AuxJoint[2] = StepExternalPulseToPosition(tRobotPulse.lBZPulse, axisUnit.dBZPulseUnit);
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
}


STEPRobotCtrl::STEPRobotCtrl(std::string strUnitName, RobotLog* pLog)
	: RobotDriverAdaptor(strUnitName, pLog)
	, m_hMutex(nullptr)
	, m_bLocalDebugMark(false)
	, m_bSocketConnected(false)
	, m_sStepProjectName()
	, m_pSTEPRobotClient(nullptr)

{
	m_pSTEPRobotClient = new RobotComClient();
	InitRobotDriver(strUnitName);
	m_hMutex = CreateMutexA(NULL, FALSE, "Mutex");
	//InitSocket(m_sSocketIP.c_str(), m_nSocketPort);
}


STEPRobotCtrl::~STEPRobotCtrl()
{
	StopStateMonitor();
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
	cIni.ReadString("StepProjectName", m_sStepProjectName);
	if (m_sStepProjectName.empty())
	{
		cIni.ReadString("ProjectName", m_sStepProjectName);
	}
	m_sStepProjectName = StepNormalizeProjectName(m_sStepProjectName);
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
	ClearLastRobotError();
	int nRet = 0;
	if (!m_bLocalDebugMark)
	{
		nRet = m_pSTEPRobotClient->init(ip, Port);
		if (0 != nRet)
		{
			m_bSocketConnected = false;
			SetLastRobotError(GetStr("STEP连接失败：IP=%s Port=%hu 原因=%s(%d)", ip == nullptr ? "" : ip, Port, GetErrorText(nRet), nRet));
			return false;
		}
	}
	m_bSocketConnected = true;
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
	m_bSocketConnected = false;
	return true;
}

bool STEPRobotCtrl::IsConnected()
{
	if (m_bLocalDebugMark)
	{
		return true;
	}
	if (m_pSTEPRobotClient == nullptr || !m_bSocketConnected)
	{
		return false;
	}
	const int sdkConnectStatus = m_pSTEPRobotClient->ConnectStatus();
	return sdkConnectStatus >= 0;
}

std::string STEPRobotCtrl::GetRobotStatusText()
{
	if (m_pSTEPRobotClient == nullptr)
	{
		return "STEP状态：客户端未初始化";
	}

	const int connectStatus = m_bLocalDebugMark ? 1 : m_pSTEPRobotClient->ConnectStatus();
	const int programState = static_cast<int>(m_pSTEPRobotClient->getProgramState());
	const int motorState = m_pSTEPRobotClient->getMotorEnableState();
	const STEPROBOTSDK::MessageData message = m_pSTEPRobotClient->getMessageData();
	const std::string messageText = StepCString(message.m_MessageString, sizeof(message.m_MessageString));
	const std::string messageSource = StepCString(message.m_MessageSource, sizeof(message.m_MessageSource));

	std::ostringstream oss;
	oss << "STEP状态：本地连接=" << (m_bSocketConnected ? "已连接" : "未连接")
		<< "，SDK连接码=" << connectStatus
		<< "，程序=" << StepProgramStateText(programState) << "(" << programState << ")"
		<< "，使能=" << (motorState == 1 ? "上使能" : "未使能") << "(" << motorState << ")"
		<< "，消息类型=" << StepMessageTypeText(message.m_MessageType)
		<< "，消息ID=" << message.m_MessageID;
	if (!messageSource.empty())
	{
		oss << "，来源=" << messageSource;
	}
	if (!messageText.empty())
	{
		oss << "，消息=" << messageText;
	}
	return oss.str();
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
	dPulse = dPulse / m_tAxisUnit.GetValueByIndex(nAxisNo);
	return dPulse;
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
	if (m_pSTEPRobotClient == nullptr)
	{
		SetLastRobotError("STEP完成状态检测失败：客户端未初始化");
		return -1;
	}
	//PROGRAMSTATE getProgramState();
	int nRet = m_pSTEPRobotClient->getProgramState();   //0	运行，1 暂停，2 停止，3 未知
	return nRet;
}
int STEPRobotCtrl::CheckRobotDone(int nDelayTime)
{
	if (nDelayTime <= 0)
	{
		nDelayTime = 200;
	}
	int nRet = -1;
	while (1)
	{
		nRet = CheckDone();
		if (STEPROBOTSDK::eRun != nRet)
		{
			Sleep(nDelayTime);
			nRet = CheckDone();
			if (STEPROBOTSDK::eRun != nRet)
			{
				const STEPROBOTSDK::MessageData message = m_pSTEPRobotClient != nullptr
					? m_pSTEPRobotClient->getMessageData()
					: STEPROBOTSDK::MessageData();
				if (nRet != STEPROBOTSDK::eStop || message.m_MessageType == STEPROBOTSDK::eError)
				{
					const std::string statusText = GetRobotStatusText();
					SetLastRobotError("STEP运动未正常完成：" + statusText);
					if (m_pRobotLog != nullptr)
					{
						m_pRobotLog->write(LogColor::ERR, "%s", GetLastRobotError().c_str());
					}
					return -1000 - nRet;
				}
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

bool STEPRobotCtrl::LoadUserProgram(std::string sProjName, std::string sProgName, bool bForceReload)
{
	std::string sNowProject, sNowProgram;
	sNowProgram = GetUserProgram();
	sNowProject = GetUserProject();
	int nRet = 0;
	std::vector<int> vnErrLine;
	if (bForceReload || sProjName != sNowProject || sProgName != sNowProgram)
	{
		nRet = m_pSTEPRobotClient->ProgramLoadCmd(sProjName,sProgName,vnErrLine,true);
		if (nRet != 0)
		{
			SetLastRobotError(GetStr("STEP加载程序失败：Project=%s Program=%s 原因=%s(%d)",
				sProjName.c_str(), sProgName.c_str(), GetErrorText(nRet), nRet));
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
			SetLastRobotError(GetStr("STEP加载程序失败：Project=%s Program=%s 存在语法错误，错误行数=%d",
				sProjName.c_str(), sProgName.c_str(), static_cast<int>(vnErrLine.size())));
			return false;
		}

		for (int i = 0; i < 40; ++i)
		{
			if (GetUserProgram() == sProgName)
			{
				return true;
			}
			Sleep(50);
		}

		SetLastRobotError(GetStr("STEP加载程序超时：Project=%s Program=%s 当前Program=%s",
			sProjName.c_str(), sProgName.c_str(), GetUserProgram().c_str()));
		return false;
	}
	return true;
}

bool STEPRobotCtrl::UnLoadUserProgramer()
{
	std::string sNowProject, sNowProgram;
	sNowProgram = GetUserProgram();
	sNowProject = StepNormalizeProjectName(GetUserProject());
	if (sNowProgram.empty() || sNowProject.empty())
	{
		return true;
	}
	int nRet = 0;
	nRet = m_pSTEPRobotClient->ProgramKillCmd(sNowProject, sNowProgram, true);
	if (nRet != 0)
	{
		SetLastRobotError(GetStr("STEP卸载程序失败：Project=%s Program=%s 原因=%s(%d)",
			sNowProject.c_str(), sNowProgram.c_str(), GetErrorText(nRet), nRet));
		showErrorMessage(
			nullptr,
			"卸载程序失败,失败原因:%s",
			GetErrorText(nRet)   // 直接用全局错误库
		);
		return false;
	}
	for (int i = 0; i < 40; ++i)
	{
		if (GetUserProgram().empty())
		{
			return true;
		}
		Sleep(50);
	}

	SetLastRobotError(GetStr("STEP卸载程序超时：Project=%s Program=%s 当前Program=%s",
		sNowProject.c_str(), sNowProgram.c_str(), GetUserProgram().c_str()));
	m_pRobotLog->write(LogColor::ERR, "STEP卸载程序超时：Project=%s Program=%s",
		sNowProject.c_str(), sNowProgram.c_str());
	return false;
}

//设置当前模式 0-手动模式，1-自动模式，3-外部自动
bool STEPRobotCtrl::SetSysMode(int nMode)
{
	const bool validMode =
		nMode == MODEKEY::MANUAL
		|| nMode == MODEKEY::AUTO
		|| nMode == MODEKEY::AUTO_EXT
		|| nMode == MODEKEY::START
		|| nMode == MODEKEY::STOP
		|| nMode == MODEKEY::MSTOP;
	if (!validMode)
	{
		SetLastRobotError(GetStr("STEP设置模式失败：模式=%d 超出范围", nMode));
		showErrorMessage(
			nullptr,
			"设置当前模式:%d失败,失败原因:模式选择错误",nMode
		);
		return false;
	}
	MODEKEY eMode = MODEKEY(nMode);
	int nRet = m_pSTEPRobotClient->SetModeCmd(eMode,true);
	if (nRet != 0)
	{
		SetLastRobotError(GetStr("STEP设置模式失败：模式=%d 原因=%s(%d)", nMode, GetErrorText(nRet), nRet));
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
		SetLastRobotError(GetStr("STEP启动程序失败：原因=%s(%d)，%s", GetErrorText(nRet), nRet, GetRobotStatusText().c_str()));
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
		SetLastRobotError(GetStr("STEP停止程序失败：原因=%s(%d)，%s", GetErrorText(nRet), nRet, GetRobotStatusText().c_str()));
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
	
	if (!LoadUserProgram(sNowProjName, sJobName))
	{
		return false;
	}
	if (!Prog_startRun_Py())
	{
		return false;
	}
	return true;
}

int STEPRobotCtrl::ContiMoveAny(const std::vector<T_ROBOT_MOVE_INFO>& vtRobotMoveInfo)
{
	ClearLastRobotError();
	if (vtRobotMoveInfo.empty())
	{
		SetLastRobotError("STEP连续运动失败：轨迹点为空");
		m_pRobotLog->write(LogColor::ERR, "STEP ContiMoveAny 失败：轨迹点为空");
		return -1;
	}

	const std::string sProjectName = StepNormalizeProjectName(kStepDynamicJobProjectName);

	const std::string sProgramName = StepMakeProgramName();
	const std::string sLocalDir = ".\\Job\\STEP";
	const std::string sLocalProgramFile = sLocalDir + "\\" + sProgramName + ".srp";
	const std::string sLocalDataFile = sLocalDir + "\\" + sProgramName + ".srd";
	const std::string sRemoteBaseDir = StepBuildRemoteProjectDir(sProjectName);
	const std::string sRemoteProgramFile = sRemoteBaseDir + "/" + sProgramName + ".srp";
	const std::string sRemoteDataFile = sRemoteBaseDir + "/" + sProgramName + ".srd";

	try
	{
		std::filesystem::create_directories(sLocalDir);
	}
	catch (const std::exception& e)
	{
		SetLastRobotError(GetStr("STEP连续运动失败：创建本地目录失败，%s", e.what()));
		m_pRobotLog->write(LogColor::ERR, "STEP ContiMoveAny 创建本地目录失败：%s", e.what());
		return -2;
	}

	const std::string sSrpContent = StepBuildSrpContent(vtRobotMoveInfo);
	const std::string sSrdContent = StepBuildSrdContent(vtRobotMoveInfo, m_tAxisUnit);

	if (!StepWriteTextFile(sLocalProgramFile, sSrpContent))
	{
		SetLastRobotError(GetStr("STEP连续运动失败：写入SRP失败，%s", sLocalProgramFile.c_str()));
		m_pRobotLog->write(LogColor::ERR, "STEP ContiMoveAny 写入SRP失败：%s", sLocalProgramFile.c_str());
		return -3;
	}
	if (!StepWriteTextFile(sLocalDataFile, sSrdContent))
	{
		SetLastRobotError(GetStr("STEP连续运动失败：写入SRD失败，%s", sLocalDataFile.c_str()));
		m_pRobotLog->write(LogColor::ERR, "STEP ContiMoveAny 写入SRD失败：%s", sLocalDataFile.c_str());
		return -4;
	}

	m_pRobotLog->write(LogColor::SUCCESS,
		"STEP ContiMoveAny 已生成程序 | Project=%s | Program=%s | PointCount=%d",
		sProjectName.c_str(), sProgramName.c_str(), static_cast<int>(vtRobotMoveInfo.size()));

	const std::string sCurrentProgram = GetUserProgram();
	if (!sCurrentProgram.empty())
	{
		const int nProgramState = CheckDone();
		if (nProgramState == STEPROBOTSDK::eRun || nProgramState == STEPROBOTSDK::ePause)
		{
			if (!Prog_stop_Py())
			{
				SetLastRobotError(GetStr("STEP连续运动失败：停止当前程序失败，%s，%s",
					sCurrentProgram.c_str(), GetRobotStatusText().c_str()));
				m_pRobotLog->write(LogColor::ERR, "STEP ContiMoveAny 停止当前程序失败：%s", sCurrentProgram.c_str());
				return -5;
			}

			bool bStopped = false;
			for (int i = 0; i < 40; ++i)
			{
				if (CheckDone() == STEPROBOTSDK::eStop)
				{
					bStopped = true;
					break;
				}
				Sleep(50);
			}
			if (!bStopped)
			{
				SetLastRobotError(GetStr("STEP连续运动失败：等待当前程序停止超时，%s，%s",
					sCurrentProgram.c_str(), GetRobotStatusText().c_str()));
				m_pRobotLog->write(LogColor::ERR, "STEP ContiMoveAny 等待当前程序停止超时：%s", sCurrentProgram.c_str());
				return -5;
			}
		}

		if (!UnLoadUserProgramer())
		{
			SetLastRobotError(GetStr("STEP连续运动失败：卸载当前程序失败，%s，%s",
				sCurrentProgram.c_str(), GetRobotStatusText().c_str()));
			m_pRobotLog->write(LogColor::ERR, "STEP ContiMoveAny 卸载当前程序失败：%s", sCurrentProgram.c_str());
			return -5;
		}
		m_pRobotLog->write(LogColor::SUCCESS, "STEP ContiMoveAny 已卸载当前程序：%s", sCurrentProgram.c_str());
	}

	if (m_pFTP != nullptr)
	{
		m_pRobotLog->write(LogColor::DEFAULT, "STEP ContiMoveAny 上传前重建FTP客户端，避免复用失效会话");
		delete m_pFTP;
		m_pFTP = nullptr;
	}

	if (m_pFTP == nullptr)
	{
		InitFtp();
	}
	if (m_pFTP == nullptr)
	{
		SetLastRobotError("STEP连续运动失败：初始化FTP失败");
		m_pRobotLog->write(LogColor::ERR, "STEP ContiMoveAny 初始化FTP失败");
		return -6;
	}

	if (!m_pFTP->uploadFile(sLocalProgramFile, sRemoteProgramFile, false))
	{
		SetLastRobotError(GetStr("STEP连续运动失败：上传SRP失败，%s", sRemoteProgramFile.c_str()));
		m_pRobotLog->write(LogColor::ERR, "STEP ContiMoveAny 上传SRP失败：%s", sRemoteProgramFile.c_str());
		return -7;
	}
	if (!m_pFTP->uploadFile(sLocalDataFile, sRemoteDataFile, false))
	{
		SetLastRobotError(GetStr("STEP连续运动失败：上传SRD失败，%s", sRemoteDataFile.c_str()));
		m_pRobotLog->write(LogColor::ERR, "STEP ContiMoveAny 上传SRD失败：%s", sRemoteDataFile.c_str());
		return -8;
	}

	m_pRobotLog->write(LogColor::SUCCESS,
		"STEP ContiMoveAny 上传完成 | SRP=%s | SRD=%s",
		sRemoteProgramFile.c_str(), sRemoteDataFile.c_str());

	if (!LoadUserProgram(sProjectName, sProgramName, true))
	{
		SetLastRobotError(GetStr("STEP连续运动失败：加载程序失败，Project=%s Program=%s，%s",
			sProjectName.c_str(), sProgramName.c_str(), GetRobotStatusText().c_str()));
		m_pRobotLog->write(LogColor::ERR,
			"STEP ContiMoveAny 加载程序失败 | Project=%s | Program=%s",
			sProjectName.c_str(), sProgramName.c_str());
		return -9;
	}

	m_pRobotLog->write(LogColor::SUCCESS,
		"STEP ContiMoveAny 已加载程序 | Project=%s | Program=%s",
		sProjectName.c_str(), sProgramName.c_str());

	const auto failRunPrepare = [this, &sProgramName](const char* stepName, int ret) -> int
		{
			SetLastRobotError(GetStr("STEP连续运动失败：启动前%s失败，Program=%s，返回=%d，%s",
				stepName, sProgramName.c_str(), ret, GetRobotStatusText().c_str()));
			m_pRobotLog->write(LogColor::ERR, "%s", GetLastRobotError().c_str());
			return -10;
		};

	int nPrepareRet = m_pSTEPRobotClient->AllAlarmConfirmCmd();
	if (nPrepareRet != 0)
	{
		return failRunPrepare("清除报警", nPrepareRet);
	}

	if (m_pSTEPRobotClient->getOperationMode() != STEPROBOTSDK::eAutomatic)
	{
		nPrepareRet = m_pSTEPRobotClient->SetModeCmd(STEPROBOTSDK::MODEKEY::AUTO, true);
		if (nPrepareRet != 0)
		{
			return failRunPrepare("切换自动模式", nPrepareRet);
		}

		bool bAutoMode = false;
		for (int i = 0; i < 20; ++i)
		{
			if (m_pSTEPRobotClient->getOperationMode() == STEPROBOTSDK::eAutomatic)
			{
				bAutoMode = true;
				break;
			}
			Sleep(50);
		}
		if (!bAutoMode)
		{
			return failRunPrepare("等待自动模式", 0);
		}
	}

	if (m_pSTEPRobotClient->getMotorEnableState() == 0)
	{
		nPrepareRet = m_pSTEPRobotClient->EnableMotorCmd();
		if (nPrepareRet != 0)
		{
			return failRunPrepare("上使能", nPrepareRet);
		}

		bool bEnabled = false;
		for (int i = 0; i < 20; ++i)
		{
			if (m_pSTEPRobotClient->getMotorEnableState() == 1)
			{
				bEnabled = true;
				break;
			}
			Sleep(50);
		}
		if (!bEnabled)
		{
			return failRunPrepare("等待使能", 0);
		}
	}

	if (m_pSTEPRobotClient->getProgramMode() != STEPROBOTSDK::eContinue)
	{
		nPrepareRet = m_pSTEPRobotClient->ProgramRunModeCmd(static_cast<int>(STEPROBOTSDK::eContinue));
		if (nPrepareRet != 0)
		{
			return failRunPrepare("切换连续运行模式", nPrepareRet);
		}

		bool bContinueMode = false;
		for (int i = 0; i < 20; ++i)
		{
			if (m_pSTEPRobotClient->getProgramMode() == STEPROBOTSDK::eContinue)
			{
				bContinueMode = true;
				break;
			}
			Sleep(50);
		}
		if (!bContinueMode)
		{
			return failRunPrepare("等待连续运行模式", 0);
		}
	}

	if (!Prog_startRun_Py())
	{
		SetLastRobotError(GetStr("STEP连续运动失败：启动程序失败，Program=%s，%s",
			sProgramName.c_str(), GetRobotStatusText().c_str()));
		m_pRobotLog->write(LogColor::ERR, "STEP ContiMoveAny 启动程序失败：%s", sProgramName.c_str());
		return -10;
	}

	bool bStarted = false;
	for (int i = 0; i < 50; ++i)
	{
		const int nProgramState = CheckDone();
		if (nProgramState == STEPROBOTSDK::eRun)
		{
			bStarted = true;
			break;
		}

		const STEPROBOTSDK::MessageData message = m_pSTEPRobotClient != nullptr
			? m_pSTEPRobotClient->getMessageData()
			: STEPROBOTSDK::MessageData();
		if (message.m_MessageType == STEPROBOTSDK::eError)
		{
			SetLastRobotError(GetStr("STEP连续运动失败：启动后控制器报错，Program=%s，%s",
				sProgramName.c_str(), GetRobotStatusText().c_str()));
			m_pRobotLog->write(LogColor::ERR, "%s", GetLastRobotError().c_str());
			return -10;
		}

		Sleep(20);
	}
	if (!bStarted)
	{
		SetLastRobotError(GetStr("STEP连续运动失败：启动后未进入运行态，Program=%s，%s",
			sProgramName.c_str(), GetRobotStatusText().c_str()));
		m_pRobotLog->write(LogColor::ERR, "%s", GetLastRobotError().c_str());
		return -10;
	}

	m_pRobotLog->write(LogColor::SUCCESS, "STEP ContiMoveAny 已启动程序：%s", sProgramName.c_str());

	return 0;
}

bool STEPRobotCtrl::ServoOff()
{
	int nRet = 0;
	//当前机器人使能时执行下使能
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
	//当前机器人未使能时执行上使能
	if (m_pSTEPRobotClient->getMotorEnableState() == 0)
	{
		nRet = m_pSTEPRobotClient->EnableMotorCmd();
	}
	if (nRet != 0)
	{
		showErrorMessage(
			nullptr,
			"上使能失败,失败原因:%s",
			GetErrorText(nRet)   // 直接用全局错误库
		);
		return false;
	}
	return true;
}

bool STEPRobotCtrl::SetRobotToolNo(int nToolNo)
{
	if (m_pSTEPRobotClient == nullptr)
	{
		SetLastRobotError("STEP工具设置失败：SDK客户端未初始化。");
		return false;
	}

	int lastRet = 0;
	for (const std::string& toolName : StepBuildToolNameCandidates(nToolNo))
	{
		lastRet = m_pSTEPRobotClient->ToolSetCmd(toolName);
		if (lastRet == 0)
		{
			ClearLastRobotError();
			return true;
		}
		if (m_pRobotLog != nullptr)
		{
			m_pRobotLog->write(LogColor::WARNING, "STEP工具设置失败 | tool=%s ret=%d reason=%s", toolName.c_str(), lastRet, GetErrorText(lastRet));
		}
	}

	SetLastRobotError(GetStr("STEP工具设置失败：tool%d/TOOL%d 均失败，最后错误=%s(%d)",
		nToolNo,
		nToolNo,
		GetErrorText(lastRet),
		lastRet));
	return false;
}

bool STEPRobotCtrl::GetToolData(int nToolNo, T_ROBOT_COORS& adRobotToolData)
{
	adRobotToolData = T_ROBOT_COORS();
	if (m_pSTEPRobotClient == nullptr)
	{
		SetLastRobotError("STEP工具读取失败：SDK客户端未初始化。");
		return false;
	}

	int lastRet = 0;
	std::string lastToolName;
	STEPROBOTSDK::Tool tTool = { false,0,0,0,0,0,0 };
	for (const std::string& toolName : StepBuildToolNameCandidates(nToolNo))
	{
		tTool = STEPROBOTSDK::Tool{ false,0,0,0,0,0,0 };
		lastToolName = toolName;
		lastRet = m_pSTEPRobotClient->VariableToolReadCmd(toolName, tTool);
		if (lastRet == 0)
		{
			adRobotToolData.dX = tTool.X;
			adRobotToolData.dY = tTool.Y;
			adRobotToolData.dZ = tTool.Z;
			adRobotToolData.dRX = tTool.A;
			adRobotToolData.dRY = tTool.B;
			adRobotToolData.dRZ = tTool.C;
			ClearLastRobotError();
			if (m_pRobotLog != nullptr)
			{
				m_pRobotLog->write(
					LogColor::SUCCESS,
					"STEP工具读取成功 | tool=%s X=%.6f Y=%.6f Z=%.6f A=%.6f B=%.6f C=%.6f fixed=%d",
					toolName.c_str(),
					tTool.X,
					tTool.Y,
					tTool.Z,
					tTool.A,
					tTool.B,
					tTool.C,
					tTool.Fixed ? 1 : 0);
			}
			return true;
		}
		if (m_pRobotLog != nullptr)
		{
			m_pRobotLog->write(LogColor::WARNING, "STEP工具读取失败 | tool=%s ret=%d reason=%s", toolName.c_str(), lastRet, GetErrorText(lastRet));
		}
	}

	SetLastRobotError(GetStr("STEP工具读取失败：tool%d/TOOL%d 均失败，最后尝试=%s，错误=%s(%d)",
		nToolNo,
		nToolNo,
		lastToolName.c_str(),
		GetErrorText(lastRet),
		lastRet));
	return false;
}

//FTP建立连接
int STEPRobotCtrl::InitFtp()
{
	if (m_pFTP == nullptr)
	{
		m_pFTP = new FtpClient(m_pRobotLog, m_sFTPIP, m_nFTPPort, m_sFTPUser, m_sFTPPassWord);
	}
	return 0;
}

//上传文件给埃斯顿机器人，埃斯顿为RemoteFilePath，本地为LocalFilePath    //  .//MultiPos_Mv1.erd 
int STEPRobotCtrl::UploadFile(std::string LocalFilePath, std::string RemoteFilePath)
{
	if (m_pFTP == nullptr)
	{
		InitFtp();
	}
	if (m_pFTP == nullptr)
	{
		return -1;
	}
	return m_pFTP->uploadFile(LocalFilePath, RemoteFilePath, false) ? 0 : -1;
}
//下载文件,埃斯顿为RemoteFilePath，本地为LocalFilePath
int STEPRobotCtrl::DownloadFile(std::string RemoteFilePath, std::string LocalFilePath)
{
	if (m_pFTP == nullptr)
	{
		InitFtp();
	}
	if (m_pFTP == nullptr)
	{
		return -1;
	}
	return m_pFTP->downloadFile(RemoteFilePath, LocalFilePath) ? 0 : -1;
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
	const std::string sProjectName = StepNormalizeProjectName(kStepDynamicJobProjectName);
	const std::string sProgramName = kStepProjectVariableProgramName;
	const std::string sVarName = StepBuildPosVarName(static_cast<int>(lPvarIndex), MoveType);

	if (MoveType == PULSEVAR)
	{
		AXISPOS value = {};
		int nRet = m_pSTEPRobotClient->VariableAxisposReadCmd(sProjectName, sProgramName, sVarName, value);
		if (nRet != 0)
		{
			const int fallbackRet = m_pSTEPRobotClient->VariableAxisposReadCmd(sProjectName, std::string(), sVarName, value);
			if (fallbackRet == 0)
			{
				nRet = 0;
			}
		}
		if (nRet != 0)
		{
			SetLastRobotError(GetStr("STEP读取工程关节变量失败：Project=%s Program=%s Var=%s 原因=%s(%d)。请确认 PCRobot/_project.srd 中已定义 ap%d。",
				sProjectName.c_str(), sProgramName.c_str(), sVarName.c_str(), GetErrorText(nRet), nRet, static_cast<int>(lPvarIndex)));
			return nRet;
		}
		for (int i = 0; i < 6; ++i)
		{
			array[i] = value.m_Joint[i];
		}
		return 0;
	}

	CARTPOS value = {};
	int nRet = m_pSTEPRobotClient->VariableCartposReadCmd(sProjectName, sProgramName, sVarName, value);
	if (nRet != 0)
	{
		const int fallbackRet = m_pSTEPRobotClient->VariableCartposReadCmd(sProjectName, std::string(), sVarName, value);
		if (fallbackRet == 0)
		{
			nRet = 0;
		}
	}
	if (nRet != 0)
	{
		SetLastRobotError(GetStr("STEP读取工程直角变量失败：Project=%s Program=%s Var=%s 原因=%s(%d)。请确认 PCRobot/_project.srd 中已定义 cp%d。",
			sProjectName.c_str(), sProgramName.c_str(), sVarName.c_str(), GetErrorText(nRet), nRet, static_cast<int>(lPvarIndex)));
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

bool STEPRobotCtrl::MoveByJob(T_ROBOT_COORS tRobotJointCoord, T_ROBOT_MOVE_SPEED tPulseMove, int nExternalAxleType, std::string JobName, int isconfig, int config[7])
{
	(void)nExternalAxleType;
	(void)isconfig;
	(void)config;

	ClearLastRobotError();
	T_ROBOT_MOVE_INFO moveInfo = {};
	moveInfo.nMoveType = (JobName == "MOVJ" || JobName == "movj") ? MOVJ : MOVL;
	moveInfo.nPosType = POSVAR;
	moveInfo.tCoord = tRobotJointCoord;
	moveInfo.tSpeed = tPulseMove;
	moveInfo.nMoveDevice = 0;
	moveInfo.nTrackNo = 0;
	moveInfo.adBasePosVar[0] = tRobotJointCoord.dBX;
	moveInfo.adBasePosVar[1] = tRobotJointCoord.dBY;
	moveInfo.adBasePosVar[2] = tRobotJointCoord.dBZ;

	const int ret = ContiMoveAny(std::vector<T_ROBOT_MOVE_INFO>{ moveInfo });
	if (ret != 0 && GetLastRobotError().empty())
	{
		SetLastRobotError(GetStr("STEP直角运动下发失败：ret=%d", ret));
	}
	return ret == 0;
}

bool STEPRobotCtrl::MoveByJob(T_ANGLE_PULSE tRobotJointCoord, T_ROBOT_MOVE_SPEED tPulseMove, int nExternalAxleType, std::string JobName)
{
	(void)nExternalAxleType;

	ClearLastRobotError();
	T_ROBOT_MOVE_INFO moveInfo = {};
	moveInfo.nMoveType = (JobName == "MOVL" || JobName == "movl") ? MOVL : MOVJ;
	moveInfo.nPosType = PULSEVAR;
	moveInfo.tPulse = tRobotJointCoord;
	moveInfo.tSpeed = tPulseMove;
	moveInfo.nMoveDevice = 0;
	moveInfo.nTrackNo = 0;
	moveInfo.adBasePosVar[0] = static_cast<double>(tRobotJointCoord.lBXPulse);
	moveInfo.adBasePosVar[1] = static_cast<double>(tRobotJointCoord.lBYPulse);
	moveInfo.adBasePosVar[2] = static_cast<double>(tRobotJointCoord.lBZPulse);

	const int ret = ContiMoveAny(std::vector<T_ROBOT_MOVE_INFO>{ moveInfo });
	if (ret != 0 && GetLastRobotError().empty())
	{
		SetLastRobotError(GetStr("STEP关节运动下发失败：ret=%d", ret));
	}
	return ret == 0;
}

bool STEPRobotCtrl::MoveByJob(double* dRobotJointCoord, T_ROBOT_MOVE_SPEED tPulseMove, int nExternalAxleType, int nPVarType, std::string JobName, int config[7])
{
	(void)config;
	if (dRobotJointCoord == nullptr)
	{
		SetLastRobotError("STEP运动失败：目标点为空");
		return false;
	}

	if (nPVarType == POSVAR || JobName == "MOVL" || JobName == "movl")
	{
		T_ROBOT_COORS coors;
		coors.dX = dRobotJointCoord[0];
		coors.dY = dRobotJointCoord[1];
		coors.dZ = dRobotJointCoord[2];
		coors.dRX = dRobotJointCoord[3];
		coors.dRY = dRobotJointCoord[4];
		coors.dRZ = dRobotJointCoord[5];
		coors.dBX = dRobotJointCoord[6];
		coors.dBY = dRobotJointCoord[7];
		return MoveByJob(coors, tPulseMove, nExternalAxleType, JobName);
	}

	T_ANGLE_PULSE pulse;
	pulse.nSPulse = static_cast<long>(dRobotJointCoord[0]);
	pulse.nLPulse = static_cast<long>(dRobotJointCoord[1]);
	pulse.nUPulse = static_cast<long>(dRobotJointCoord[2]);
	pulse.nRPulse = static_cast<long>(dRobotJointCoord[3]);
	pulse.nBPulse = static_cast<long>(dRobotJointCoord[4]);
	pulse.nTPulse = static_cast<long>(dRobotJointCoord[5]);
	pulse.lBXPulse = static_cast<long>(dRobotJointCoord[6]);
	pulse.lBYPulse = static_cast<long>(dRobotJointCoord[7]);
	return MoveByJob(pulse, tPulseMove, nExternalAxleType, JobName);
}
