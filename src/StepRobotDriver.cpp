#include "STEPRobotDriver.h"

#include "MeasureThenWeldRuntimeConfig.h"
#include "RobotOperationLease.h"
#include "RobotPoseTransform.h"
#include "StepSdkBuildConfig.h"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <limits>
#include <sstream>
#include <utility>
#include <vector>

namespace
{
	std::string StepMakeProgramName()
	{
		return "ContiMoveAny";
	}

	std::string StepSanitizeProgramName(std::string programName)
	{
		if (programName.empty())
		{
			programName = StepMakeProgramName();
		}

		for (char& ch : programName)
		{
			const unsigned char value = static_cast<unsigned char>(ch);
			if (value < 32
				|| ch == '<'
				|| ch == '>'
				|| ch == ':'
				|| ch == '"'
				|| ch == '/'
				|| ch == '\\'
				|| ch == '|'
				|| ch == '?'
				|| ch == '*'
				|| ch == '.')
			{
				ch = '_';
			}
		}
		return programName.empty() ? StepMakeProgramName() : programName;
	}

	constexpr const char* kStepDynamicJobProjectName = "PCRobot";
	constexpr const char* kStepProjectVariableProgramName = "_project";
	constexpr const char* kStepArcOnDataName = "arcon0";
	constexpr const char* kStepArcDataName = "arc0";
	constexpr const char* kStepTransitionArcDataName = "arctrans0";
	constexpr const char* kStepArcOffDataName = "arcoff0";
	constexpr const char* kStepArcStartIntName = "int0";
	constexpr const char* kStepArcRetryDataName = "retry0";
	constexpr const char* kStepArcRealName = "real0";
	constexpr const char* kStepWeaveDataName = "wd0";
	constexpr const char* kStepTrackDataName = "td0";

	long long StepRobotSteadyNowMs()
	{
		return std::chrono::duration_cast<std::chrono::milliseconds>(
			std::chrono::steady_clock::now().time_since_epoch()).count();
	}

	void StepFillPcPassiveTimestamp(long long* pRobotMs, long long* pPcRecvMs, long long pcMs)
	{
		if (pRobotMs != nullptr)
		{
			*pRobotMs = pcMs;
		}
		if (pPcRecvMs != nullptr)
		{
			*pPcRecvMs = pcMs;
		}
	}

	// 接口模式进程级缓存：-1=未加载 0=旧接口 1=时间戳接口。状态监控线程 50ms
	// 周期调用本函数，逐次读配置库会引入每秒 20 次 SQLite 查询与采样抖动；
	// 管理界面切换模式时经 InvalidateStepSdkInterfaceModeCache 作废重读。
	std::atomic<int> g_stepSdkInterfaceModeCache{ -1 };

	bool StepUseTimestampSdkInterface()
	{
#if STEP_SDK_HAS_TIMESTAMP
		int cached = g_stepSdkInterfaceModeCache.load();
		if (cached < 0)
		{
			cached = (MeasureThenWeldRuntimeConfig::LoadStepSdkInterfaceMode()
				== MeasureThenWeldRuntimeConfig::StepSdkInterfaceMode::Timestamp) ? 1 : 0;
			g_stepSdkInterfaceModeCache.store(cached);
		}
		return cached == 1;
#else
		return false;
#endif
	}

	T_ROBOT_COORS StepRobotCartPosToCoors(const STEPROBOTSDK::RobotCartPos& cartPos)
	{
		T_ROBOT_COORS coors = T_ROBOT_COORS();
		coors.dX = cartPos.cart[0];
		coors.dY = cartPos.cart[1];
		coors.dZ = cartPos.cart[2];
		coors.dRX = cartPos.cart[3];
		coors.dRY = cartPos.cart[4];
		coors.dRZ = cartPos.cart[5];
		return coors;
	}

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

	int StepNormalizeArcMode(int mode)
	{
		return mode >= 0 && mode <= 7 ? mode : 4;
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

	std::string StepToLower(std::string text)
	{
		std::transform(text.begin(), text.end(), text.begin(), [](unsigned char ch) {
			return static_cast<char>(std::tolower(ch));
		});
		return text;
	}

	std::string StepInternetLastErrorText()
	{
		DWORD errorCode = GetLastError();
		DWORD responseCode = errorCode;
		char response[512] = {};
		DWORD responseSize = static_cast<DWORD>(sizeof(response));
		if (InternetGetLastResponseInfoA(&responseCode, response, &responseSize) && response[0] != '\0')
		{
			return GetStr("WinINet错误=%lu，FTP响应=%s", responseCode, response);
		}
		return GetStr("WinINet错误=%lu", errorCode);
	}

	bool StepDownloadFtpTextFile(
		const std::string& host,
		int port,
		const std::string& user,
		const std::string& password,
		const std::string& remotePath,
		std::string& content,
		std::string* errorText)
	{
		content.clear();
		if (host.empty())
		{
			if (errorText != nullptr)
			{
				*errorText = "FTP地址为空。";
			}
			return false;
		}

		HINTERNET internet = InternetOpenA("QtWidgetsApplication4_STEP_Eye", INTERNET_OPEN_TYPE_DIRECT, nullptr, nullptr, 0);
		if (internet == nullptr)
		{
			if (errorText != nullptr)
			{
				*errorText = "FTP初始化失败：" + StepInternetLastErrorText();
			}
			return false;
		}

		HINTERNET ftp = InternetConnectA(
			internet,
			host.c_str(),
			static_cast<INTERNET_PORT>(port > 0 ? port : 21),
			user.empty() ? nullptr : user.c_str(),
			password.empty() ? nullptr : password.c_str(),
			INTERNET_SERVICE_FTP,
			INTERNET_FLAG_PASSIVE,
			0);
		if (ftp == nullptr)
		{
			const std::string detail = StepInternetLastErrorText();
			InternetCloseHandle(internet);
			if (errorText != nullptr)
			{
				*errorText = GetStr("FTP连接失败：%s:%d，%s", host.c_str(), port > 0 ? port : 21, detail.c_str());
			}
			return false;
		}

		HINTERNET file = FtpOpenFileA(ftp, remotePath.c_str(), GENERIC_READ, FTP_TRANSFER_TYPE_ASCII | INTERNET_FLAG_RELOAD, 0);
		if (file == nullptr)
		{
			const std::string detail = StepInternetLastErrorText();
			InternetCloseHandle(ftp);
			InternetCloseHandle(internet);
			if (errorText != nullptr)
			{
				*errorText = GetStr("FTP打开文件失败：%s，%s", remotePath.c_str(), detail.c_str());
			}
			return false;
		}

		char buffer[4096] = {};
		DWORD bytesRead = 0;
		while (InternetReadFile(file, buffer, sizeof(buffer), &bytesRead))
		{
			if (bytesRead == 0)
			{
				break;
			}
			content.append(buffer, buffer + bytesRead);
		}
		if (bytesRead != 0)
		{
			if (errorText != nullptr)
			{
				*errorText = "FTP读取文件失败：" + StepInternetLastErrorText();
			}
			InternetCloseHandle(file);
			InternetCloseHandle(ftp);
			InternetCloseHandle(internet);
			return false;
		}

		InternetCloseHandle(file);
		InternetCloseHandle(ftp);
		InternetCloseHandle(internet);
		return true;
	}

	bool StepParseHandEyeSrd(
		const std::string& content,
		const std::string& variableName,
		double rotation[9],
		double translation[3],
		std::string* errorText)
	{
		const std::string loweredContent = StepToLower(content);
		const std::string loweredName = StepToLower(variableName);
		size_t searchPos = 0;
		while ((searchPos = loweredContent.find("handeye", searchPos)) != std::string::npos)
		{
			size_t nameStart = searchPos + 7;
			while (nameStart < loweredContent.size() && std::isspace(static_cast<unsigned char>(loweredContent[nameStart])))
			{
				++nameStart;
			}

			size_t nameEnd = nameStart;
			while (nameEnd < loweredContent.size()
				&& !std::isspace(static_cast<unsigned char>(loweredContent[nameEnd]))
				&& loweredContent[nameEnd] != ':'
				&& loweredContent[nameEnd] != '=')
			{
				++nameEnd;
			}

			if (loweredContent.substr(nameStart, nameEnd - nameStart) != loweredName)
			{
				searchPos = nameEnd;
				continue;
			}

			const size_t openBrace = loweredContent.find('{', nameEnd);
			const size_t closeBrace = openBrace == std::string::npos ? std::string::npos : loweredContent.find('}', openBrace);
			if (openBrace == std::string::npos || closeBrace == std::string::npos)
			{
				break;
			}

			std::string valuesText = content.substr(openBrace + 1, closeBrace - openBrace - 1);
			for (char& ch : valuesText)
			{
				if (ch == ',')
				{
					ch = ' ';
				}
			}

			std::istringstream stream(valuesText);
			std::vector<double> values;
			double value = 0.0;
			while (stream >> value)
			{
				values.push_back(value);
			}

			if (values.size() < 12)
			{
				if (errorText != nullptr)
				{
					*errorText = GetStr("HANDEYE %s 数据数量不足：需要12个，实际%zu个。", variableName.c_str(), values.size());
				}
				return false;
			}

			// STEP 示教器的 HANDEYE 文件按列保存 3x3 旋转矩阵；界面配置按行显示 R00..R22。
			for (int row = 0; row < 3; ++row)
			{
				for (int col = 0; col < 3; ++col)
				{
					rotation[row * 3 + col] = values[static_cast<size_t>(col * 3 + row)];
				}
			}
			for (int index = 0; index < 3; ++index)
			{
				translation[index] = values[static_cast<size_t>(9 + index)];
			}
			return true;
		}

		if (errorText != nullptr)
		{
			*errorText = GetStr("未在全局变量文件中找到 HANDEYE %s。", variableName.c_str());
		}
		return false;
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

	// pointwise 摆动停留：每个停留点一个 INT 等待变量(ms)，srp 的 WaitTime 与 srd 的 INT 定义共用此名
	std::string StepBuildWaitIntName(size_t index)
	{
		return GetStr("nwait%u", static_cast<unsigned>(index));
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

	void StepAppendFileComment(std::ostringstream& oss, const char* text)
	{
		(void)oss;
		(void)text;
	}

	void StepAppendFileComment(std::ostringstream& oss, const std::string& text)
	{
		StepAppendFileComment(oss, text.c_str());
	}

	void StepAppendCommand(std::ostringstream& oss, const char* command)
	{
		oss << command << "\n";
	}

	void StepAppendCommand(std::ostringstream& oss, const std::string& command)
	{
		StepAppendCommand(oss, command.c_str());
	}

	bool StepNearlyEqual(double left, double right)
	{
		return std::abs(left - right) <= 1e-9;
	}

	bool StepDynamicValuesEqual(const StepDynamicValues& left, const StepDynamicValues& right)
	{
		return StepNearlyEqual(left.segmentVel, right.segmentVel)
			&& StepNearlyEqual(left.segmentAcc, right.segmentAcc)
			&& StepNearlyEqual(left.segmentDec, right.segmentDec)
			&& StepNearlyEqual(left.segmentJerk, right.segmentJerk)
			&& StepNearlyEqual(left.oriVel, right.oriVel)
			&& StepNearlyEqual(left.oriAcc, right.oriAcc)
			&& StepNearlyEqual(left.oriDec, right.oriDec)
			&& StepNearlyEqual(left.oriJerk, right.oriJerk)
			&& StepNearlyEqual(left.jointVel, right.jointVel)
			&& StepNearlyEqual(left.jointAcc, right.jointAcc)
			&& StepNearlyEqual(left.jointDec, right.jointDec)
			&& StepNearlyEqual(left.jointJerk, right.jointJerk);
	}

	struct StepVariablePlan
	{
		std::vector<size_t> dynamicIndexes;
		std::vector<StepDynamicValues> dynamicValues;
	};

	StepVariablePlan StepBuildVariablePlan(const std::vector<T_ROBOT_MOVE_INFO>& moveInfos)
	{
		StepVariablePlan plan;
		plan.dynamicIndexes.reserve(moveInfos.size());
		for (const T_ROBOT_MOVE_INFO& info : moveInfos)
		{
			const StepDynamicValues dyn = StepBuildDynamicValues(info.tSpeed);
			size_t dynamicIndex = plan.dynamicValues.size();
			for (size_t index = 0; index < plan.dynamicValues.size(); ++index)
			{
				if (StepDynamicValuesEqual(plan.dynamicValues[index], dyn))
				{
					dynamicIndex = index;
					break;
				}
			}
			if (dynamicIndex == plan.dynamicValues.size())
			{
				plan.dynamicValues.push_back(dyn);
			}
			plan.dynamicIndexes.push_back(dynamicIndex);
		}
		return plan;
	}

	void StepAppendDynamicLine(std::ostringstream& oss, const std::string& dynName, const StepDynamicValues& dyn)
	{
		oss << "DYNAMIC " << dynName << " := {  "
			<< StepClampPositiveDouble(dyn.segmentVel, 1.0) << ", "
			<< StepClampPositiveDouble(dyn.segmentAcc, 3.0) << ", "
			<< StepClampPositiveDouble(dyn.segmentDec, 3.0) << ", "
			<< StepClampPositiveDouble(dyn.segmentJerk, 50000.0) << ", "
			<< dyn.oriVel << ", " << dyn.oriAcc << ", " << dyn.oriDec << ", " << dyn.oriJerk << ", "
			<< dyn.jointVel << ", " << dyn.jointAcc << ", " << dyn.jointDec << "," << dyn.jointJerk
			<< " }" << "\n";
	}

	double StepOverlapRelValue(const std::vector<T_ROBOT_MOVE_INFO>& moveInfos)
	{
		for (const T_ROBOT_MOVE_INFO& info : moveInfos)
		{
			if (std::isfinite(info.dOverlapRel))
			{
				return std::max(0.0, info.dOverlapRel);
			}
		}
		return 20.0;
	}

	// STEP srp Lin/WLin 第4参(姿态/位形)：0=NULL 1=可变(eVAR) 2=恒定(eCONST) 3=腕关节(eWRIST)。来自基础工艺参数。
	const char* StepPostureName(int postureType)
	{
		switch (postureType)
		{
		case 0: return "NULL";
		case 2: return "eCONST";
		case 3: return "eWRIST";
		case 1:
		default: return "eVAR";
		}
	}

	bool StepHasWeldProcess(const std::vector<T_ROBOT_MOVE_INFO>& moveInfos)
	{
		return std::any_of(moveInfos.begin(), moveInfos.end(),
			[](const T_ROBOT_MOVE_INFO& info)
			{
				return info.bWeldProcessEnabled;
			});
	}

	bool StepHasTransitionWeldProcess(const std::vector<T_ROBOT_MOVE_INFO>& moveInfos)
	{
		return std::any_of(moveInfos.begin(), moveInfos.end(),
			[](const T_ROBOT_MOVE_INFO& info)
			{
				return info.bWeldProcessEnabled && info.bUseTransitionWeldParams;
			});
	}

	const T_ROBOT_MOVE_INFO* StepFirstWeldProcessInfo(const std::vector<T_ROBOT_MOVE_INFO>& moveInfos)
	{
		for (const T_ROBOT_MOVE_INFO& info : moveInfos)
		{
			if (info.bWeldProcessEnabled)
			{
				return &info;
			}
		}
		return nullptr;
	}

	const T_ROBOT_MOVE_INFO* StepFirstNormalWeldProcessInfo(const std::vector<T_ROBOT_MOVE_INFO>& moveInfos)
	{
		for (const T_ROBOT_MOVE_INFO& info : moveInfos)
		{
			if (info.bWeldProcessEnabled && !info.bUseTransitionWeldParams)
			{
				return &info;
			}
		}
		return StepFirstWeldProcessInfo(moveInfos);
	}

	double StepFiniteOrDefault(double value, double fallback)
	{
		return std::isfinite(value) ? value : fallback;
	}

	double StepWeldDataSpeedMmPerSec(const T_ROBOT_MOVE_INFO& info)
	{
		if (std::isfinite(info.dWeldSpeedMmPerMin) && info.dWeldSpeedMmPerMin > 0.0)
		{
			return StepLinearSpeedMmPerSecFromConfig(info.dWeldSpeedMmPerMin, 0.0);
		}
		return StepLinearSpeedMmPerSecFromConfig(info.tSpeed.dSpeed, 0.0);
	}

	std::string StepFormatCompactNumber(double value)
	{
		const double safeValue = StepFiniteOrDefault(value, 0.0);
		const double rounded = std::round(safeValue);
		if (std::fabs(safeValue - rounded) < 0.000001)
		{
			return std::to_string(static_cast<long long>(rounded));
		}

		std::ostringstream oss;
		oss << std::fixed << std::setprecision(6) << safeValue;
		std::string text = oss.str();
		while (text.size() > 1 && text.back() == '0')
		{
			text.pop_back();
		}
		if (!text.empty() && text.back() == '.')
		{
			text.pop_back();
		}
		return text;
	}

	unsigned StepWaitTimeMs(double waitTimeSeconds)
	{
		const double safeValue = StepFiniteOrDefault(waitTimeSeconds, 0.0);
		if (safeValue <= 0.0)
		{
			return 0U;
		}
		return static_cast<unsigned>(std::max(0.0, std::round(safeValue * 1000.0)));
	}

	std::string StepFormatPaddedWaitTimeMs(double waitTimeSeconds)
	{
		std::ostringstream oss;
		oss << std::setw(6) << std::setfill('0') << StepWaitTimeMs(waitTimeSeconds);
		return oss.str();
	}

	std::string StepBuildArcDataLine(
		const char* name,
		double current,
		double voltage,
		double speedMmPerSec)
	{
		std::ostringstream oss;
		oss << "ARCDATA " << name << " := {  "
			<< StepFormatCompactNumber(current) << ", "
			<< StepFormatCompactNumber(voltage) << ", "
			<< StepFormatCompactNumber(speedMmPerSec) << ",0.000000 }" << "\n";
		return oss.str();
	}

	std::string StepEnumOrNumber(int value, const std::vector<std::pair<int, const char*>>& names)
	{
		for (const auto& item : names)
		{
			if (item.first == value)
			{
				return item.second;
			}
		}
		return std::to_string(value);
	}

	std::string StepWeaveTypeText(int value)
	{
		return StepEnumOrNumber(value, {
			{0, "eTCPWeave"},
			{1, "eWristWeave"},
			{2, "e45JointWeave"}
		});
	}

	std::string StepWeaveShapeText(int value)
	{
		return StepEnumOrNumber(value, {
			{0, "eNoWeave"},
			{5, "eSin"},
			{6, "eSinFreq"},
			{7, "eSpiral"},
			{8, "eObliqueTriangle"},
			{9, "eSpaceTriangle"},
			{10, "eLTriangle"},
			{11, "eBackForward"},
			{12, "eConstPoint"},
			{13, "eSpiralFreq"},
			{14, "eObliqueTriangleFreq"},
			{15, "eSpaceTriangleFreq"},
			{16, "eLTriangleFreq"},
			{17, "eBackForwordFreq"},
			{18, "eHalfSin"}
		});
	}

	std::string StepVerticalModeText(int value)
	{
		return StepEnumOrNumber(value, {
			{0, "eConst"},
			{1, "eSample"}
		});
	}

	std::string StepTrackIntervalModeText(int value)
	{
		return StepEnumOrNumber(value, {
			{0, "eDistance"},
			{1, "eTime"}
		});
	}

	std::string StepBuildWeaveDataLine(const char* name, const T_WeaveDate& weave)
	{
		std::ostringstream oss;
		oss << "WEAVEDATA " << name << " := {  "
			<< StepWeaveTypeText(weave.nWeaveType) << ", "
			<< StepWeaveShapeText(weave.nWeaveShape) << ", "
			<< StepFormatCompactNumber(weave.dWeaveFrequencyHz) << ", "
			<< StepFormatCompactNumber(weave.dWeaveAmplitudeMm) << ", "
			<< StepFormatCompactNumber(weave.dSwingDirectionDeg) << ", "
			<< StepFormatCompactNumber(weave.dWeavePlaneAngleDeg) << ", "
			<< StepFormatCompactNumber(weave.dSpaceAngleDeg) << ", "
			<< weave.nPauseTime1Ms << ", "
			<< weave.nPauseTime2Ms << ", "
			<< weave.nPauseTime3Ms << ", "
			<< weave.nPauseTime4Ms << ", "
			<< (weave.nPauseContinue != 0 ? "TRUE" : "FALSE") << ", "
			<< StepFormatCompactNumber(weave.dEndLengthMm) << ", "
			<< StepFormatCompactNumber(weave.dEndWidthMm) << ","
			<< StepFormatCompactNumber(weave.dCenterHeightMm) << " }" << "\n";
		return oss.str();
	}

	std::string StepBuildTrackDataLine(const char* name, const T_TrackData& track)
	{
		std::ostringstream oss;
		oss << "TRACKDATA " << name << " := {  "
			<< track.nLateralBeginCycle << ", "
			<< StepFormatCompactNumber(track.dLateralGain) << ", "
			<< StepFormatCompactNumber(track.dLeftAreaCoefficient) << ", "
			<< StepFormatCompactNumber(track.dRightAreaCoefficient) << ", "
			<< StepVerticalModeText(track.nVerticalModeFlag) << ", "
			<< StepFormatCompactNumber(track.dVerticalReferenceCurrent) << ", "
			<< track.nVerticalBeginCycle << ", "
			<< track.nVerticalSustainCycle << ", "
			<< StepFormatCompactNumber(track.dVerticalCycleLength) << ", "
			<< StepFormatCompactNumber(track.dVerticalGain) << ", "
			<< StepTrackIntervalModeText(track.nTimeOrDistanceMode) << ", "
			<< track.nTimeIntervalMs << ", "
			<< track.nDistanceIntervalMm << ", "
			<< StepFormatCompactNumber(track.dLateralMinCompPerCycle) << ", "
			<< StepFormatCompactNumber(track.dLateralMaxCompPerCycle) << ", "
			<< StepFormatCompactNumber(track.dLateralMaxCompTotal) << ", "
			<< StepFormatCompactNumber(track.dLateralAsymmetryCoefficient) << ", "
			<< StepFormatCompactNumber(track.dLateralReserved6) << ", "
			<< StepFormatCompactNumber(track.dLateralReserved5) << ", "
			<< StepFormatCompactNumber(track.dLateralReserved4) << ", "
			<< StepFormatCompactNumber(track.dLateralReserved3) << ", "
			<< StepFormatCompactNumber(track.dLateralReserved2) << ", "
			<< StepFormatCompactNumber(track.dLateralReserved1) << ", "
			<< StepFormatCompactNumber(track.dVerticalMinCompPerCycle) << ", "
			<< StepFormatCompactNumber(track.dVerticalMaxCompPerCycle) << ", "
			<< StepFormatCompactNumber(track.dVerticalMaxCompTotal) << ", "
			<< StepFormatCompactNumber(track.dVerticalAsymmetryCoefficient) << ", "
			<< StepFormatCompactNumber(track.dVerticalReserved6) << ", "
			<< StepFormatCompactNumber(track.dVerticalReserved5) << ", "
			<< StepFormatCompactNumber(track.dVerticalReserved4) << ", "
			<< StepFormatCompactNumber(track.dVerticalReserved3) << ", "
			<< StepFormatCompactNumber(track.dVerticalReserved2) << ","
			<< StepFormatCompactNumber(track.dVerticalReserved1) << " }" << "\n";
		return oss.str();
	}

	std::string StepBuildSrdContent(const std::vector<T_ROBOT_MOVE_INFO>& moveInfos, const T_AXISUNIT& axisUnit, bool actualWeld)
	{
		std::ostringstream oss;
		oss << std::fixed << std::setprecision(6);
		const StepVariablePlan variablePlan = StepBuildVariablePlan(moveInfos);

		StepAppendFileComment(oss, "点位数据：SRP运动语句使用的TCP/AP变量");
		for (size_t i = 0; i < moveInfos.size(); ++i)
		{
			const T_ROBOT_MOVE_INFO& info = moveInfos[i];
			const std::string cartName = StepBuildCartPosName(i);
			const std::string axisName = StepBuildAxisPosName(i);

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

			// pointwise 摆动停留点：生成 INT 等待变量(ms)，供 srp 的 WaitTime 语句引用
			if (info.nDwellMs > 0)
			{
				oss << "INT " << StepBuildWaitIntName(i) << " := " << info.nDwellMs << "\n";
			}
		}

		StepAppendFileComment(oss, "速度数据：共享DYNAMIC变量");
		for (size_t index = 0; index < variablePlan.dynamicValues.size(); ++index)
		{
			StepAppendDynamicLine(oss, StepBuildDynamicName(index), variablePlan.dynamicValues[index]);
		}
		if (!moveInfos.empty())
		{
			StepAppendFileComment(oss, "连续过渡比例：共享OVERLAPREL变量");
			oss << "OVERLAPREL " << StepBuildOverlapName(0) << " := " << StepOverlapRelValue(moveInfos) << "\n";
		}

		if (StepHasWeldProcess(moveInfos))
		{
			const T_ROBOT_MOVE_INFO* processInfo = StepFirstWeldProcessInfo(moveInfos);
			const T_ROBOT_MOVE_INFO* normalProcessInfo = StepFirstNormalWeldProcessInfo(moveInfos);
			if (processInfo != nullptr && normalProcessInfo != nullptr)
			{
				StepAppendFileComment(oss, actualWeld
					? "实际焊接：SRP直接生成ARCON/ARCSET/ARCOFF，不再使用IF判断"
					: "空跑模式：SRP不生成ARCON/ARCSET/ARCOFF焊接指令");
				StepAppendFileComment(oss, "焊接数据：起弧参数");
				oss << "ARCONDATA " << kStepArcOnDataName << " := {  0, "
					<< StepFormatCompactNumber(processInfo->dArcStartCurrent) << ", "
					<< StepFormatCompactNumber(processInfo->dArcStartVoltage) << ", "
					<< StepFormatPaddedWaitTimeMs(processInfo->dArcStartWaitTime) << ", 0,0 }" << "\n";
				oss << "ARCOFFDATA " << kStepArcOffDataName << " := {  0, 0, "
					<< StepFormatCompactNumber(processInfo->dArcEndCurrent) << ", "
					<< StepFormatCompactNumber(processInfo->dArcEndVoltage) << ","
					<< StepWaitTimeMs(processInfo->dArcEndWaitTime) << " }" << "\n";
				oss << "INT " << kStepArcStartIntName << " := 0" << "\n";
				StepAppendFileComment(oss, "正常焊接参数：电流/电压/速度");
				oss << StepBuildArcDataLine(
					kStepArcDataName,
					normalProcessInfo->dWeldCurrent,
					normalProcessInfo->dWeldVoltage,
					StepWeldDataSpeedMmPerSec(*normalProcessInfo));

				if (StepHasTransitionWeldProcess(moveInfos))
				{
					const auto transitionIt = std::find_if(moveInfos.begin(), moveInfos.end(),
						[](const T_ROBOT_MOVE_INFO& info)
						{
							return info.bWeldProcessEnabled && info.bUseTransitionWeldParams;
					});
					if (transitionIt != moveInfos.end())
					{
						StepAppendFileComment(oss, "拐点过渡参数：电流/电压/速度");
						oss << StepBuildArcDataLine(
							kStepTransitionArcDataName,
							transitionIt->dWeldCurrent,
							transitionIt->dWeldVoltage,
							StepWeldDataSpeedMmPerSec(*transitionIt));
					}
				}

				StepAppendFileComment(oss, "焊接辅助数据：重试/实数");
				oss << "ARCRETRYDATA " << kStepArcRetryDataName << " := {  1000, 0, 1000, 50, 0, 0, FALSE, 20, 50,5 }" << "\n";
				oss << "REAL " << kStepArcRealName << " := 0.0" << "\n";
				if (processInfo->bHasWeaveParam)
				{
					StepAppendFileComment(oss, "摆动参数：WEAVEDATA");
					oss << StepBuildWeaveDataLine(kStepWeaveDataName, processInfo->tWeaveParam);
				}
				if (processInfo->bHasTrackParam)
				{
					StepAppendFileComment(oss, "跟踪参数：TRACKDATA");
					oss << StepBuildTrackDataLine(kStepTrackDataName, processInfo->tTrackParam);
				}
			}
		}

		return oss.str();
	}

	std::string StepBuildSrpContent(const std::vector<T_ROBOT_MOVE_INFO>& moveInfos, bool actualWeld)
	{
		std::ostringstream oss;
		const StepVariablePlan variablePlan = StepBuildVariablePlan(moveInfos);
		const std::string sharedOverlapName = StepBuildOverlapName(0);
		const bool hasWeldProcess = StepHasWeldProcess(moveInfos);
		const bool emitWeldCommands = hasWeldProcess && actualWeld;
		if (emitWeldCommands)
		{
			const T_ROBOT_MOVE_INFO* processInfo = StepFirstWeldProcessInfo(moveInfos);
			const int arcMode = processInfo != nullptr ? StepNormalizeArcMode(processInfo->nArcMode) : 4;
			StepAppendCommand(oss, GetStr("ARCMODE(%d);", arcMode));
			StepAppendFileComment(oss, "焊接开始：使用起弧参数起弧");
			StepAppendCommand(oss, std::string("ARCON(")
				+ kStepArcOnDataName + "," + kStepArcDataName + "," + kStepArcStartIntName + ","
				+ kStepArcRetryDataName + "," + kStepArcRealName + ");");
			StepAppendFileComment(oss, "切换正常焊接参数");
			StepAppendCommand(oss, std::string("ARCSET(") + kStepArcDataName + ");");
		}
		else if (hasWeldProcess)
		{
			StepAppendFileComment(oss, "空跑模式：不生成ARCON/ARCSET/ARCOFF焊接指令");
		}

		bool usingTransitionWeldParams = false;

		for (size_t i = 0; i < moveInfos.size(); ++i)
		{
			const T_ROBOT_MOVE_INFO& info = moveInfos[i];
			const std::string cartName = StepBuildCartPosName(i);
			const std::string axisName = StepBuildAxisPosName(i);
			const size_t dynamicIndex = i < variablePlan.dynamicIndexes.size() ? variablePlan.dynamicIndexes[i] : 0;
			const std::string dynName = StepBuildDynamicName(dynamicIndex);
			const std::string targetName = info.nPosType == PULSEVAR ? axisName : cartName;

			if (hasWeldProcess && info.bWeldProcessEnabled)
			{
				const bool needTransition = info.bUseTransitionWeldParams;
				if (emitWeldCommands && needTransition)
				{
					if (!usingTransitionWeldParams)
					{
						// 进入拐点过渡段时切一次过渡电流电压，段内连续点沿用该工艺。
						StepAppendFileComment(oss, "进入拐点过渡参数");
						StepAppendCommand(oss, std::string("ARCSET(") + kStepTransitionArcDataName + ");");
					}
				}
				else if (emitWeldCommands && usingTransitionWeldParams)
				{
					// 离开拐点过渡段时恢复正常焊接电流电压。
					StepAppendFileComment(oss, "退出拐点过渡参数，恢复正常焊接参数");
					StepAppendCommand(oss, std::string("ARCSET(") + kStepArcDataName + ");");
				}
				usingTransitionWeldParams = needTransition;
				const char* weaveName = info.bHasWeaveParam ? kStepWeaveDataName : "NULL";
				const char* trackName = info.bHasTrackParam ? kStepTrackDataName : "NULL";
				// 焊接动态特性(DYNAMIC)：nDynamicMode!=0 用程序速度 dynName(ntdyn0)，否则 NULL(机器人默认动态)。
				const std::string weldDynamicArg = (info.nDynamicMode != 0) ? dynName : std::string("NULL");
				oss << "WLin(" << targetName << "," << weldDynamicArg << "," << sharedOverlapName << "," << StepPostureName(info.nPostureType) << ","
					<< weaveName << "," << trackName << ",tool1,WORLD);" << "\n";
			}
			else if (info.nMoveType == MOVL)
			{
				oss << "Lin(" << targetName << "," << dynName << "," << sharedOverlapName << ","
					<< StepPostureName(info.nPostureType) << ",tool1,WORLD);" << "\n";
			}
			else
			{
				oss << "PTP(" << targetName << "," << dynName << "," << sharedOverlapName << ",tool1,WORLD);" << "\n";
			}

			// pointwise 摆动相位点停留：该点运动后插 WaitTime 完全停 nDwellMs 毫秒(INT 变量定义在 .srd)
			if (info.nDwellMs > 0)
			{
				oss << "WaitTime(" << StepBuildWaitIntName(i) << ",TRUE);" << "\n";
			}
		}

		if (emitWeldCommands)
		{
			StepAppendFileComment(oss, "焊接结束：使用停弧参数停弧");
			StepAppendCommand(oss, std::string("ARCOFF(") + kStepArcOffDataName + ");");
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
		std::string line;
		const auto appendCleanLine = [&normalized, &line]() {
			const size_t commentPos = line.find("//");
			if (commentPos != std::string::npos)
			{
				line.erase(commentPos);
			}
			while (!line.empty() && (line.back() == ' ' || line.back() == '\t'))
			{
				line.pop_back();
			}
			if (line.find_first_not_of(" \t") != std::string::npos)
			{
				normalized.append(line);
				normalized.push_back('\n');
			}
			line.clear();
		};
		for (size_t i = 0; i < content.size(); ++i)
		{
			if (content[i] == '\r')
			{
				if (i + 1 < content.size() && content[i + 1] == '\n')
				{
					++i;
				}
				appendCleanLine();
				continue;
			}
			if (content[i] == '\n')
			{
				appendCleanLine();
				continue;
			}
			line.push_back(content[i]);
		}
		if (!line.empty())
		{
			appendCleanLine();
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

std::string STEPRobotCtrl::MakeTimestampWeldProgramName()
{
	const auto now = std::chrono::system_clock::now();
	const std::time_t nowTime = std::chrono::system_clock::to_time_t(now);
	const auto nowMs = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
	std::tm localTime = {};
#if defined(_WIN32)
	localtime_s(&localTime, &nowTime);
#else
	localtime_r(&nowTime, &localTime);
#endif

	std::ostringstream oss;
	oss << "Weld_" << std::put_time(&localTime, "%Y%m%d_%H%M%S")
		<< "_" << std::setw(3) << std::setfill('0') << nowMs.count();
	return StepSanitizeProgramName(oss.str());
}

bool STEPRobotCtrl::WriteContiMoveAnyFiles(
	const std::vector<T_ROBOT_MOVE_INFO>& vtRobotMoveInfo,
	const std::string& localDir,
	const std::string& programName,
	const T_AXISUNIT& axisUnit,
	std::string* localProgramFile,
	std::string* localDataFile,
	std::string* errorText,
	bool actualWeld)
{
	if (vtRobotMoveInfo.empty())
	{
		if (errorText != nullptr)
		{
			*errorText = "轨迹点为空";
		}
		return false;
	}

	// pointwise 自定义摆动：把中心线点位沿弧长展开成密集摆动点(非 pointwise 原样返回)
	std::string weaveError;
	std::vector<T_ROBOT_MOVE_INFO> weaveMoveInfo =
		RobotDriverAdaptor::ExpandMoveInfosByPointwiseWeave(vtRobotMoveInfo, &weaveError);
	if (!weaveError.empty())
	{
		if (errorText != nullptr) { *errorText = weaveError; }
		return false;
	}
	// 摆动后 TCP 路径变长，按 k 补偿速度维持沿焊缝行进速度(未摆动时原样返回)
	weaveMoveInfo = RobotDriverAdaptor::ApplyWeaveSpeedCompensation(vtRobotMoveInfo, weaveMoveInfo);

	const std::string safeProgramName = StepSanitizeProgramName(programName);
	const std::string targetDir = localDir.empty() ? ".\\Job\\STEP" : localDir;
	const std::filesystem::path dirPath(targetDir);
	const std::filesystem::path srpPath = dirPath / (safeProgramName + ".srp");
	const std::filesystem::path srdPath = dirPath / (safeProgramName + ".srd");

	try
	{
		std::filesystem::create_directories(dirPath);
	}
	catch (const std::exception& e)
	{
		if (errorText != nullptr)
		{
			*errorText = std::string("创建本地目录失败：") + e.what();
		}
		return false;
	}

	const std::string srpContent = StepBuildSrpContent(weaveMoveInfo, actualWeld);
	const std::string srdContent = StepBuildSrdContent(weaveMoveInfo, axisUnit, actualWeld);
	const std::string srpPathText = srpPath.string();
	const std::string srdPathText = srdPath.string();

	if (!StepWriteTextFile(srpPathText, srpContent))
	{
		if (errorText != nullptr)
		{
			*errorText = "写入SRP失败：" + srpPathText;
		}
		return false;
	}
	if (!StepWriteTextFile(srdPathText, srdContent))
	{
		if (errorText != nullptr)
		{
			*errorText = "写入SRD失败：" + srdPathText;
		}
		return false;
	}

	if (localProgramFile != nullptr)
	{
		*localProgramFile = srpPathText;
	}
	if (localDataFile != nullptr)
	{
		*localDataFile = srdPathText;
	}
	if (errorText != nullptr)
	{
		errorText->clear();
	}
	return true;
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
	// 先停监控线程(join，等其可能正在进行的后台首连返回)再 CloseSocket，
	// 否则 worker 的 InitSocket 可能与析构的 CloseSocket 并发操作 m_pSTEPRobotClient。
	StopStateMonitor();
	CloseSocket();
	if (m_pSTEPRobotClient != nullptr)
	{
		delete m_pSTEPRobotClient;
		m_pSTEPRobotClient = nullptr;
	}
	if (m_hMutex != nullptr)
	{
		CloseHandle(m_hMutex);
		m_hMutex = nullptr;
	}
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

	// GUI 启动(s_connectDriversAtConstruct=false)：构造不在此同步连接，改由状态监控线程后台首连，
	// 避免机器人离线时这里挂满 OS connect 超时(~5s/台)拖慢主窗口显示。CLI(--no-show)保持同步连接，
	// 确保 CLI 命令执行时已连上(零行为回归)。两种情况监控线程的 EnsureConnectionForMonitor 都会兜底。
	if (s_connectDriversAtConstruct.load())
	{
		InitSocket(m_sSocketIP.c_str(), m_nSocketPort);
	}
	return true;
}

void STEPRobotCtrl::EnsureConnectionForMonitor()
{
	if (m_bLocalDebugMark)
	{
		m_bSocketConnected.store(true);
		return;
	}
	bool connected = false;
	if (m_pSTEPRobotClient != nullptr && m_bSocketConnected.load())
	{
		const int status = WithSdkCommand([&]() { return m_pSTEPRobotClient->ConnectStatus(); });
		connected = status >= 0;
		m_bSocketConnected.store(connected);
	}
	// 真实探测和可能阻塞的重连只发生在后台监控线程；GUI 只读 atomic 快照。
	if (!connected)
	{
		InitSocket(m_sSocketIP.c_str(), m_nSocketPort);
	}
}

bool STEPRobotCtrl::InitSocket(const char* ip, unsigned short Port, bool ifRecord)
{
	ClearLastRobotError();
	int nRet = 0;
	if (!m_bLocalDebugMark)
	{
		// 纯连接不会启动程序；sticky STOP 下也必须允许重连，才能在断线恢复后
		// 再次调用 AbortCurrentProgram 做真实停机回读。加载/START/运动入口仍严格拒绝取消态。
		nRet = WithSdkCommand([&]() { return m_pSTEPRobotClient->init(ip, Port); });
		if (0 != nRet)
		{
			m_bSocketConnected = false;
			SetLastRobotError(GetStr("STEP连接失败：IP=%s Port=%hu 原因=%s(%d)", ip == nullptr ? "" : ip, Port, GetErrorText(nRet), nRet));
			return false;
		}
	}
	m_bSocketConnected = true;
	// 新连接会话：时间轴锁定重置，由首个状态样本重新决定。
	m_nTimestampAxisLatch.store(0);
	m_lastValidRobotMs.store(0);
	m_bTimestampFallbackLogged.store(false);
	return true;
}

void STEPRobotCtrl::InvalidateStepSdkInterfaceModeCache()
{
	g_stepSdkInterfaceModeCache.store(-1);
}

bool STEPRobotCtrl::CloseSocket()
{
	int nRet = 0;
	const bool wasConnected = m_bSocketConnected;
	m_bSocketConnected = false;
	if (m_pSTEPRobotClient == nullptr)
	{
		return true;
	}
	if (!m_bLocalDebugMark)
	{
		nRet = WithSdkCommand([&]() { return m_pSTEPRobotClient->close(); });
		if (0 != nRet)
		{
			return false;
		}
	}
	return wasConnected || nRet == 0;
}

bool STEPRobotCtrl::IsConnected()
{
	if (m_bLocalDebugMark)
	{
		return true;
	}
	// UI 定时器和红色 STOP 可达性不能等待正在超时重连的 SDK mutex。
	return m_pSTEPRobotClient != nullptr && m_bSocketConnected.load();
}

std::string STEPRobotCtrl::GetRobotStatusText()
{
	if (m_pSTEPRobotClient == nullptr)
	{
		return "STEP状态：客户端未初始化";
	}

	const int connectStatus = m_bLocalDebugMark ? 1
		: WithSdkCommand([&]() { return m_pSTEPRobotClient->ConnectStatus(); });
	const std::string sdkVersion = WithSdkCommand([&]() { return m_pSTEPRobotClient->SDKVersion(); });
	const int programState = WithSdkCommand([&]() { return static_cast<int>(m_pSTEPRobotClient->getProgramState()); });
	const int motorState = WithSdkCommand([&]() { return m_pSTEPRobotClient->getMotorEnableState(); });
	const STEPROBOTSDK::MessageData message = WithSdkCommand([&]() { return m_pSTEPRobotClient->getMessageData(); });
	const std::string messageText = StepCString(message.m_MessageString, sizeof(message.m_MessageString));
	const std::string messageSource = StepCString(message.m_MessageSource, sizeof(message.m_MessageSource));

	std::ostringstream oss;
	oss << "STEP状态：本地连接=" << (m_bSocketConnected ? "已连接" : "未连接")
		<< "，SDK连接码=" << connectStatus
		<< "，SDK版本=" << (sdkVersion.empty() ? "未知" : sdkVersion)
		<< "，接口=" << (StepUseTimestampSdkInterface() ? "新版时间戳" : "旧版兼容")
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

std::string STEPRobotCtrl::GetStateMonitorSourceText() const
{
#if STEP_SDK_HAS_TIMESTAMP
	return StepUseTimestampSdkInterface()
		? "STEP SDK新版getTimestamp()(位姿+robot_ms)，脉冲=getAxisPos，完成状态=getProgramState"
		: "STEP SDK旧版接口getCartPosWorld/getAxisPos/getProgramState，时间轴=PC接收时间";
#else
	return "STEP SDK旧版库构建getCartPosWorld/getAxisPos/getProgramState，时间轴=PC接收时间";
#endif
}

double STEPRobotCtrl::GetCurrentPos(int nAxisNo) 
{
	if (m_pSTEPRobotClient == nullptr || nAxisNo < 0 || nAxisNo >= 6)
	{
		return 0.0;
	}

	RobotCartPos cartposworld = WithSdkCommand([&]() { return m_pSTEPRobotClient->getCartPosWorld(); });

	double dPos = cartposworld.cart[nAxisNo]; //坐标
	return dPos;
}

T_ROBOT_COORS STEPRobotCtrl::GetCurrentPos()
{
	if (m_pSTEPRobotClient == nullptr)
	{
		return T_ROBOT_COORS();
	}

	RobotCartPos cartposworld = WithSdkCommand([&]() { return m_pSTEPRobotClient->getCartPosWorld(); });
	return StepRobotCartPosToCoors(cartposworld);
}

T_ROBOT_COORS STEPRobotCtrl::GetCurrentPosPassive(long long* pRobotMs, long long* pPcRecvMs)
{
	if (m_pSTEPRobotClient == nullptr)
	{
		const long long pcMs = StepRobotSteadyNowMs();
		StepFillPcPassiveTimestamp(pRobotMs, pPcRecvMs, pcMs);
		return T_ROBOT_COORS();
	}

	if (m_bLocalDebugMark || !m_bSocketConnected || !StepUseTimestampSdkInterface())
	{
		T_ROBOT_COORS pose = GetCurrentPos();
		const long long pcMs = StepRobotSteadyNowMs();
		StepFillPcPassiveTimestamp(pRobotMs, pPcRecvMs, pcMs);
		return pose;
	}

#if STEP_SDK_HAS_TIMESTAMP
	const TimestampAddCartpos timestampedPos = WithSdkCommand([&]() { return m_pSTEPRobotClient->getTimestamp(); });
	const long long pcRecvMs = StepRobotSteadyNowMs();
	const unsigned long long robotTimestampMs = static_cast<unsigned long long>(timestampedPos.m_TimeStamp_ms);

	// 时间轴会话锁定：首个样本决定本次连接走机器人时间戳还是 PC 接收时间，
	// 之后不再切换——两种纪元完全不同，混进同一扫描序列会破坏时间插值。
	int axisLatch = m_nTimestampAxisLatch.load();
	if (axisLatch == 0)
	{
		axisLatch = robotTimestampMs > 0 ? 1 : 2;
		m_nTimestampAxisLatch.store(axisLatch);
		if (axisLatch == 2 && !m_bTimestampFallbackLogged.exchange(true) && m_pRobotLog != nullptr)
		{
			m_pRobotLog->write(LogColor::ERR,
				"STEP 控制器未提供时间戳（getTimestamp 返回 0），本次连接状态时间轴回退 PC 接收时间。");
		}
	}
	long long robotMs;
	if (axisLatch == 1)
	{
		if (robotTimestampMs > 0)
		{
			robotMs = static_cast<long long>(robotTimestampMs);
			m_lastValidRobotMs.store(robotMs);
		}
		else
		{
			// 已锁定机器人时间轴时偶发 0 值：沿用上一有效时间戳（时间冻结一帧），
			// 不回退 PC 纪元。
			robotMs = m_lastValidRobotMs.load();
		}
	}
	else
	{
		robotMs = pcRecvMs;
	}

	if (pRobotMs != nullptr)
	{
		*pRobotMs = robotMs;
	}
	if (pPcRecvMs != nullptr)
	{
		*pPcRecvMs = pcRecvMs;
	}
	return StepRobotCartPosToCoors(timestampedPos.m_CartPos.m_CartPos);
#else
	const long long pcMs = StepRobotSteadyNowMs();
	StepFillPcPassiveTimestamp(pRobotMs, pPcRecvMs, pcMs);
	return GetCurrentPos();
#endif
}

double STEPRobotCtrl::GetCurrentPulse(int nAxisNo)
{
	if (m_pSTEPRobotClient == nullptr || nAxisNo < 0 || nAxisNo >= 9)
	{
		return 0.0;
	}

	AXISPOS currentaxispos = WithSdkCommand([&]() { return m_pSTEPRobotClient->getAxisPos(); }); //获取当前笛卡尔位置
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

T_ANGLE_PULSE STEPRobotCtrl::GetCurrentPulsePassive(long long* pRobotMs, long long* pPcRecvMs)
{
	const long long pcMs = StepRobotSteadyNowMs();
	T_ANGLE_PULSE pulse = GetCurrentPulse();
	StepFillPcPassiveTimestamp(pRobotMs, pPcRecvMs, pcMs);
	return pulse;
}

int STEPRobotCtrl::CheckDone()
{
	if (m_pSTEPRobotClient == nullptr)
	{
		SetLastRobotError("STEP完成状态检测失败：客户端未初始化");
		return -1;
	}
	//PROGRAMSTATE getProgramState();
	std::lock_guard<std::recursive_mutex> modeLock(m_sdkCommandMutex);
	int nRet = m_pSTEPRobotClient->getProgramState();   //0	运行，1 暂停，2 停止，3 未知
	return nRet;
}

int STEPRobotCtrl::CheckDonePassive(long long* pRobotMs, long long* pPcRecvMs)
{
	const long long pcMs = StepRobotSteadyNowMs();
	const int done = CheckDone();
	StepFillPcPassiveTimestamp(pRobotMs, pPcRecvMs, pcMs);
	return done;
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
		if (RobotOperationLease::IsCancellationRequested(this))
		{
			SetLastRobotError("STEP硬件操作已被安全停止取消，禁止把停止态判作正常完成。");
			return -20000;
		}
		nRet = CheckDone();
		// 暂停是可恢复的运行态：外层流程和其硬件租约必须继续存活，等待用户 START。
		// 若把 ePause 当失败返回，流程会释放租约，而机器人程序仍可被界面无租约地继续。
		if (nRet == STEPROBOTSDK::eRun || nRet == STEPROBOTSDK::ePause)
		{
			Sleep(nDelayTime);
			continue;
		}
		if (STEPROBOTSDK::eRun != nRet)
		{
			Sleep(nDelayTime);
			nRet = CheckDone();
			if (nRet == STEPROBOTSDK::eRun || nRet == STEPROBOTSDK::ePause)
			{
				Sleep(nDelayTime);
				continue;
			}
			if (STEPROBOTSDK::eRun != nRet)
			{
				STEPROBOTSDK::MessageData message;
				{
					std::lock_guard<std::recursive_mutex> modeLock(m_sdkCommandMutex);
					message = m_pSTEPRobotClient != nullptr
						? m_pSTEPRobotClient->getMessageData()
						: STEPROBOTSDK::MessageData();
				}
				if (nRet != STEPROBOTSDK::eStop || message.m_MessageType == STEPROBOTSDK::eError)
				{
					const std::string statusText = GetRobotStatusText();
					const std::string waitError = "STEP运动未正常完成：" + statusText;
					if (m_pRobotLog != nullptr)
					{
						m_pRobotLog->write(LogColor::ERR, "%s", waitError.c_str());
					}
					const bool stopped = RobotOperationLease::StopAndConfirmUnverifiedMotion(this);
					const std::string stopDetail = GetLastRobotError();
					SetLastRobotError(waitError + (stopped
						? "；已自动中止并确认机器人程序不可恢复。"
						: "；自动中止未确认：" + stopDetail));
					return -1000 - nRet;
				}
				{
					std::lock_guard<std::recursive_mutex> modeLock(m_sdkCommandMutex);
					RobotOperationLease::MarkMotionCompleted(this);
					m_motionTrackedProjectName.clear();
					m_motionTrackedProgramName.clear();
				}
				return nRet;
			}
		}
		Sleep(nDelayTime);
	}
	return -1;
}

bool STEPRobotCtrl::AbortCurrentProgramSafely()
{
	return AbortCurrentProgram();
}

std::string STEPRobotCtrl::GetUserProgram()
{
	std::string sProgramName = WithSdkCommand([&]() { return m_pSTEPRobotClient->getProgramName(); });
	return sProgramName;
}

std::string STEPRobotCtrl::GetUserProject()
{
	std::string sProgramName = WithSdkCommand([&]() { return m_pSTEPRobotClient->getProjectName(); });
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
		bool cancelledBeforeLoad = false;
		nRet = WithSdkCommand([&]()
			{
				// 与 ProgramLoadCmd 共用同一把 SDK 锁，关闭“安全终止已确认后，
				// 旧流程才继续加载可由下一次 START 恢复的程序”竞态。
				if (RobotOperationLease::IsCancellationRequested(this))
				{
					cancelledBeforeLoad = true;
					return -1;
				}
				return m_pSTEPRobotClient->ProgramLoadCmd(sProjName, sProgName, vnErrLine, true);
			});
		if (cancelledBeforeLoad)
		{
			SetLastRobotError(GetStr(
				"STEP硬件操作已被安全停止取消，拒绝加载后续程序：Project=%s Program=%s",
				sProjName.c_str(), sProgName.c_str()));
			return false;
		}
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
			if (RobotOperationLease::IsCancellationRequested(this))
			{
				SetLastRobotError(GetStr(
					"STEP硬件操作已被安全停止取消，停止等待程序加载：Project=%s Program=%s",
					sProjName.c_str(), sProgName.c_str()));
				return false;
			}
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
	nRet = WithSdkCommand([&]() { return m_pSTEPRobotClient->ProgramKillCmd(sNowProject, sNowProgram, true); });
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
	const bool safetyStop = nMode == MODEKEY::STOP || nMode == MODEKEY::MSTOP;
	std::unique_lock<std::recursive_mutex> modeLock(m_sdkCommandMutex);
	if (!safetyStop && RobotOperationLease::IsCancellationRequested(this))
	{
		SetLastRobotError("STEP硬件操作已被安全停止取消，拒绝恢复或切换到新的运行模式。");
		return false;
	}
	const bool startsProgram = nMode == MODEKEY::START;
	if (startsProgram)
	{
		const std::string startProject = m_pSTEPRobotClient != nullptr
			? StepNormalizeProjectName(m_pSTEPRobotClient->getProjectName())
			: std::string();
		const std::string startProgram = m_pSTEPRobotClient != nullptr
			? m_pSTEPRobotClient->getProgramName()
			: std::string();
		if (startProgram.empty())
		{
			SetLastRobotError("STEP START 已拒绝：当前没有已加载程序。");
			return false;
		}
		QString motionError;
		if (!RobotOperationLease::MarkMotionStarted(this, false, &motionError))
		{
			const std::string armError = motionError.toStdString();
			const bool priorMotionPending = RobotOperationLease::MotionCompletionPending(this);
			modeLock.unlock();
			if (priorMotionPending)
			{
				const bool stopped = RobotOperationLease::StopAndConfirmUnverifiedMotion(this);
				const std::string stopDetail = GetLastRobotError();
				SetLastRobotError(armError + (stopped
					? "；已自动中止并确认上一程序不可恢复。"
					: "；自动中止上一程序未确认：" + stopDetail));
			}
			else
			{
				SetLastRobotError(armError);
			}
			return false;
		}
		m_motionTrackedProjectName = startProject;
		m_motionTrackedProgramName = startProgram;
	}
	MODEKEY eMode = MODEKEY(nMode);
	int nRet = m_pSTEPRobotClient->SetModeCmd(eMode,true);
	if (nRet != 0)
	{
		const std::string modeError = GetStr(
			"STEP设置模式失败：模式=%d 原因=%s(%d)", nMode, GetErrorText(nRet), nRet);
		modeLock.unlock();
		if (startsProgram)
		{
			const bool stopped = RobotOperationLease::StopAndConfirmUnverifiedMotion(this);
			const std::string stopDetail = GetLastRobotError();
			SetLastRobotError(modeError + (stopped
				? "；START结果不明，已自动中止并确认程序不可恢复。"
				: "；START结果不明，自动中止未确认：" + stopDetail));
		}
		else
		{
			SetLastRobotError(modeError);
			showErrorMessage(
				nullptr,
				"设置当前模式:%d失败,失败原因:%s", nMode,
				GetErrorText(nRet));
		}
		return false;
	}
	return true;
}

//运行程序 运行的是目前加载的程序
bool STEPRobotCtrl::Prog_startRun_Py(bool resumeExisting)
{
	std::unique_lock<std::recursive_mutex> modeLock(m_sdkCommandMutex);
	if (RobotOperationLease::IsCancellationRequested(this))
	{
		SetLastRobotError("STEP硬件操作已被安全停止取消，拒绝重新启动程序。");
		return false;
	}
	const std::string startProject = m_pSTEPRobotClient != nullptr
		? StepNormalizeProjectName(m_pSTEPRobotClient->getProjectName())
		: std::string();
	const std::string startProgram = m_pSTEPRobotClient != nullptr
		? m_pSTEPRobotClient->getProgramName()
		: std::string();
	if (startProgram.empty())
	{
		SetLastRobotError("STEP START 已拒绝：当前没有已加载程序。");
		return false;
	}
	if (resumeExisting
		&& (!RobotOperationLease::MotionCompletionPending(this)
			|| m_pSTEPRobotClient == nullptr
			|| m_pSTEPRobotClient->getProgramState() != STEPROBOTSDK::ePause
			|| m_motionTrackedProjectName != startProject
			|| m_motionTrackedProgramName != startProgram))
	{
		const std::string resumeError = GetStr(
			"STEP继续运行失败：暂停程序身份已变化或不再可恢复。Tracked=%s/%s Current=%s/%s",
			m_motionTrackedProjectName.c_str(),
			m_motionTrackedProgramName.c_str(),
			startProject.c_str(),
			startProgram.c_str());
		modeLock.unlock();
		const bool stopped = RobotOperationLease::StopAndConfirmUnverifiedMotion(this);
		const std::string stopDetail = GetLastRobotError();
		SetLastRobotError(resumeError + (stopped
			? "；原程序已自动中止并确认不可恢复。"
			: "；程序身份异常且自动中止未确认：" + stopDetail));
		return false;
	}
	QString motionError;
	if (!RobotOperationLease::MarkMotionStarted(this, resumeExisting, &motionError))
	{
		const std::string armError = motionError.toStdString();
		const bool priorMotionPending = RobotOperationLease::MotionCompletionPending(this);
		modeLock.unlock();
		if (priorMotionPending)
		{
			const bool stopped = RobotOperationLease::StopAndConfirmUnverifiedMotion(this);
			const std::string stopDetail = GetLastRobotError();
			SetLastRobotError(armError + (stopped
				? "；已自动中止并确认上一程序不可恢复。"
				: "；自动中止上一程序未确认：" + stopDetail));
		}
		else
		{
			SetLastRobotError(armError);
		}
		return false;
	}
	if (!resumeExisting)
	{
		m_motionTrackedProjectName = startProject;
		m_motionTrackedProgramName = startProgram;
	}
	int nRet = m_pSTEPRobotClient->SetModeCmd(MODEKEY::START, true);
	if (nRet != 0)
	{
		const std::string startError = GetStr(
			"STEP启动程序失败：原因=%s(%d)，%s",
			GetErrorText(nRet), nRet, GetRobotStatusText().c_str());
		modeLock.unlock();
		const bool stopped = RobotOperationLease::StopAndConfirmUnverifiedMotion(this);
		const std::string stopDetail = GetLastRobotError();
		SetLastRobotError(startError + (stopped
			? "；START结果不明，已自动中止并确认程序不可恢复。"
			: "；START结果不明，自动中止未确认：" + stopDetail));
		return false;
	}
	return true;
}

//停止程序 停止的是目前加载的程序
bool STEPRobotCtrl::Prog_stop_Py()
{
	std::lock_guard<std::recursive_mutex> modeLock(m_sdkCommandMutex);
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

bool STEPRobotCtrl::AbortCurrentProgram()
{
	std::lock_guard<std::recursive_mutex> sdkLock(m_sdkCommandMutex);
	if (m_pSTEPRobotClient == nullptr)
	{
		SetLastRobotError("STEP安全取消失败：SDK客户端未初始化。");
		return false;
	}
	if (!m_bLocalDebugMark)
	{
		const int connectStatus = m_bSocketConnected.load()
			? m_pSTEPRobotClient->ConnectStatus()
			: -1;
		if (connectStatus < 0)
		{
			// 安全停止专用重连：sticky cancellation 会阻止普通动作租约，但不能阻止
			// 恢复通信后再次 STOP/Kill 并完成真实停机确认。
			const int initRet = m_pSTEPRobotClient->init(m_sSocketIP.c_str(), m_nSocketPort);
			if (initRet != 0)
			{
				m_bSocketConnected.store(false);
				SetLastRobotError(GetStr(
					"STEP安全取消重连失败：IP=%s Port=%d 原因=%s(%d)",
					m_sSocketIP.c_str(), m_nSocketPort, GetErrorText(initRet), initRet));
				return false;
			}
			m_bSocketConnected.store(true);
			m_nTimestampAxisLatch.store(0);
			m_lastValidRobotMs.store(0);
			m_bTimestampFallbackLogged.store(false);
		}
	}
	const bool appMotionPending = RobotOperationLease::MotionCompletionPending(this);
	if (!appMotionPending)
	{
		// 红色按钮只停止本软件已登记的运动。普通寄存器/FTP 等租约被取消时，
		// 不得误杀示教器或外部通道当前加载的程序。
		m_motionTrackedProjectName.clear();
		m_motionTrackedProgramName.clear();
		ClearLastRobotError();
		return true;
	}
	const std::string currentProject = StepNormalizeProjectName(m_pSTEPRobotClient->getProjectName());
	const std::string currentProgram = m_pSTEPRobotClient->getProgramName();
	if (m_motionTrackedProgramName.empty()
		|| currentProject != m_motionTrackedProjectName
		|| currentProgram != m_motionTrackedProgramName)
	{
		SetLastRobotError(GetStr(
			"STEP安全取消未执行：当前程序身份与本软件启动记录不一致，拒绝误停外部程序。Tracked=%s/%s Current=%s/%s",
			m_motionTrackedProjectName.c_str(),
			m_motionTrackedProgramName.c_str(),
			currentProject.c_str(),
			currentProgram.c_str()));
		return false;
	}

	// STOP=23 只是可恢复暂停；随后必须 ProgramKillCmd 并回读程序名为空，
	// 才能确认后续 START 不会恢复已脱离高层流程的旧程序。
	const int stopRet = m_pSTEPRobotClient->SetModeCmd(MODEKEY::STOP, true);
	const std::string projectName = m_motionTrackedProjectName;
	const std::string programName = m_motionTrackedProgramName;
	int killRet = 0;
	if (!projectName.empty() && !programName.empty())
	{
		killRet = m_pSTEPRobotClient->ProgramKillCmd(projectName, programName, true);
	}
	if (!programName.empty() && killRet != 0)
	{
		SetLastRobotError(GetStr(
			"STEP安全取消未确认：已加载程序的 ProgramKillCmd 失败，STOP=%d，Kill=%d，Project=%s，Program=%s",
			stopRet, killRet, projectName.c_str(), programName.c_str()));
		return false;
	}

	int stableStoppedCount = 0;
	for (int retry = 0; retry < 40; ++retry)
	{
		const std::string remainingProgram = m_pSTEPRobotClient->getProgramName();
		const int state = m_pSTEPRobotClient->getProgramState();
		if (remainingProgram.empty() && state == STEPROBOTSDK::eStop)
		{
			++stableStoppedCount;
			if (stableStoppedCount >= 4)
			{
				RobotOperationLease::MarkMotionCompleted(this);
				m_motionTrackedProjectName.clear();
				m_motionTrackedProgramName.clear();
				ClearLastRobotError();
				return true;
			}
		}
		else
		{
			// ePause/eUnknown 或任意通信异常态都不得当成不可恢复终止。
			stableStoppedCount = 0;
		}
		Sleep(50);
	}

	SetLastRobotError(GetStr(
		"STEP安全取消未确认：STOP=%d，Kill=%d，Project=%s，Program=%s，Remaining=%s，State=%d",
		stopRet,
		killRet,
		projectName.c_str(),
		programName.c_str(),
		m_pSTEPRobotClient->getProgramName().c_str(),
		static_cast<int>(m_pSTEPRobotClient->getProgramState())));
	return false;
}

int STEPRobotCtrl::GetCurrentProgramLine()
{
	if (m_pSTEPRobotClient == nullptr)
	{
		return -1;
	}
	return WithSdkCommand([&]() { return m_pSTEPRobotClient->getCurrentLine(); });
}

bool STEPRobotCtrl::SetProgramLine(int nLine)
{
	if (m_pSTEPRobotClient == nullptr || nLine < 0)
	{
		return false;
	}
	const int nRet = WithSdkCommand([&]() { return m_pSTEPRobotClient->SetpcCmd(nLine); });
	if (nRet != 0)
	{
		SetLastRobotError(GetStr("STEP设置程序行号失败：行=%d 原因=%s(%d)", nLine, GetErrorText(nRet), nRet));
		return false;
	}
	return true;
}

bool STEPRobotCtrl::CallJob(std::string sJobName)
{
	if (RobotOperationLease::IsCancellationRequested(this))
	{
		SetLastRobotError("STEP硬件操作已被安全停止取消，拒绝调用后续程序：" + sJobName);
		return false;
	}
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
	return ContiMoveAnyWithProgramName(vtRobotMoveInfo, StepMakeProgramName());
}

int STEPRobotCtrl::ContiMoveAnyWithProgramName(const std::vector<T_ROBOT_MOVE_INFO>& vtRobotMoveInfo, const std::string& programName)
{
	ClearLastRobotError();
	if (RobotOperationLease::IsCancellationRequested(this))
	{
		SetLastRobotError("STEP硬件操作已被安全停止取消，拒绝下发后续轨迹。");
		return -20000;
	}
	if (vtRobotMoveInfo.empty())
	{
		SetLastRobotError("STEP连续运动失败：轨迹点为空");
		m_pRobotLog->write(LogColor::ERR, "STEP ContiMoveAny 失败：轨迹点为空");
		return -1;
	}

	// pointwise 自定义摆动：把中心线点位沿弧长展开成密集摆动点(非 pointwise 原样返回)
	std::string weaveError;
	std::vector<T_ROBOT_MOVE_INFO> weaveMoveInfo =
		RobotDriverAdaptor::ExpandMoveInfosByPointwiseWeave(vtRobotMoveInfo, &weaveError);
	if (!weaveError.empty())
	{
		SetLastRobotError(weaveError);
		m_pRobotLog->write(LogColor::ERR, "STEP ContiMoveAny pointwise 摆动失败：%s", weaveError.c_str());
		return -1;
	}
	// 摆动后 TCP 路径变长，按 k 补偿速度维持沿焊缝行进速度(未摆动时原样返回)
	std::string weaveSpeedInfo;
	weaveMoveInfo = RobotDriverAdaptor::ApplyWeaveSpeedCompensation(vtRobotMoveInfo, weaveMoveInfo, 0.0, &weaveSpeedInfo);
	if (!weaveSpeedInfo.empty())
	{
		m_pRobotLog->write(LogColor::DEFAULT, "STEP %s", weaveSpeedInfo.c_str());
	}

	const std::string sProjectName = StepNormalizeProjectName(kStepDynamicJobProjectName);

	const std::string sProgramName = StepSanitizeProgramName(programName);
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

	const std::string sSrpContent = StepBuildSrpContent(weaveMoveInfo, true);
	const std::string sSrdContent = StepBuildSrdContent(weaveMoveInfo, m_tAxisUnit, true);

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
			// 暂停态(1)的程序已处于停止键状态，再发 STOP 无效/报错——现场“暂停状态下启动流程失败”的根因。
			// 暂停态跳过停止步骤直接卸载(ProgramKillCmd 对暂停程序有效)；运行态仍先停止并等待到停止。
			if (nProgramState == STEPROBOTSDK::eRun)
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
			else
			{
				m_pRobotLog->write(LogColor::WARNING,
					"STEP ContiMoveAny 当前程序处于暂停态，跳过停止直接卸载：%s", sCurrentProgram.c_str());
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
	const auto getOperationMode = [this]()
		{
			std::lock_guard<std::recursive_mutex> modeLock(m_sdkCommandMutex);
			return m_pSTEPRobotClient->getOperationMode();
		};
	const auto getMotorEnableState = [this]()
		{
			std::lock_guard<std::recursive_mutex> modeLock(m_sdkCommandMutex);
			return m_pSTEPRobotClient->getMotorEnableState();
		};
	const auto getProgramMode = [this]()
		{
			std::lock_guard<std::recursive_mutex> modeLock(m_sdkCommandMutex);
			return m_pSTEPRobotClient->getProgramMode();
		};

	int nPrepareRet = 0;
	{
		std::lock_guard<std::recursive_mutex> modeLock(m_sdkCommandMutex);
		if (RobotOperationLease::IsCancellationRequested(this))
		{
			return failRunPrepare("安全停止后取消启动", -20000);
		}
		nPrepareRet = m_pSTEPRobotClient->AllAlarmConfirmCmd();
	}
	if (nPrepareRet != 0)
	{
		return failRunPrepare("清除报警", nPrepareRet);
	}

	if (getOperationMode() != STEPROBOTSDK::eAutomatic)
	{
		const auto trySwitchAutoMode = [this]() -> bool
			{
				{
					std::lock_guard<std::recursive_mutex> modeLock(m_sdkCommandMutex);
					if (RobotOperationLease::IsCancellationRequested(this)
						|| m_pSTEPRobotClient->SetModeCmd(STEPROBOTSDK::MODEKEY::AUTO, true) != 0)
					{
						return false;
					}
				}
				for (int i = 0; i < 20; ++i)
				{
					{
						std::lock_guard<std::recursive_mutex> modeLock(m_sdkCommandMutex);
						if (m_pSTEPRobotClient->getOperationMode() == STEPROBOTSDK::eAutomatic)
						{
							return true;
						}
					}
					if (RobotOperationLease::IsCancellationRequested(this))
					{
						return false;
					}
					Sleep(50);
				}
				return false;
			};

		if (!trySwitchAutoMode())
		{
			if (RobotOperationLease::IsCancellationRequested(this))
			{
				return failRunPrepare("安全停止后取消切换自动模式", -20000);
			}
			// 现场经验：SDK 会话异常时切自动命令发了也不生效，重连一次后重试才有效。
			m_pRobotLog->write(LogColor::WARNING,
				"STEP ContiMoveAny 切换自动模式未生效，重连机器人后重试");
			if (RobotOperationLease::IsCancellationRequested(this))
			{
				return failRunPrepare("安全停止后禁止重连", -20000);
			}
			WithSdkCommand([&]() { return m_pSTEPRobotClient->close(); });
			Sleep(200);
			if (RobotOperationLease::IsCancellationRequested(this))
			{
				return failRunPrepare("安全停止后禁止重连", -20000);
			}
			if (!InitSocket(m_sSocketIP.c_str(), m_nSocketPort))
			{
				return failRunPrepare("切换自动模式失败后重连机器人", 0);
			}
			if (!trySwitchAutoMode())
			{
				return failRunPrepare("重连后切换自动模式", 0);
			}
			m_pRobotLog->write(LogColor::SUCCESS, "STEP ContiMoveAny 重连后已切换自动模式");
		}
	}

	if (getMotorEnableState() == 0)
	{
		{
			std::lock_guard<std::recursive_mutex> modeLock(m_sdkCommandMutex);
			if (RobotOperationLease::IsCancellationRequested(this))
			{
				return failRunPrepare("安全停止后取消上使能", -20000);
			}
			nPrepareRet = m_pSTEPRobotClient->EnableMotorCmd();
		}
		if (nPrepareRet != 0)
		{
			return failRunPrepare("上使能", nPrepareRet);
		}

		bool bEnabled = false;
		for (int i = 0; i < 20; ++i)
		{
			if (getMotorEnableState() == 1)
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

	if (getProgramMode() != STEPROBOTSDK::eContinue)
	{
		{
			std::lock_guard<std::recursive_mutex> modeLock(m_sdkCommandMutex);
			if (RobotOperationLease::IsCancellationRequested(this))
			{
				return failRunPrepare("安全停止后取消切换连续运行模式", -20000);
			}
			nPrepareRet = m_pSTEPRobotClient->ProgramRunModeCmd(static_cast<int>(STEPROBOTSDK::eContinue));
		}
		if (nPrepareRet != 0)
		{
			return failRunPrepare("切换连续运行模式", nPrepareRet);
		}

		bool bContinueMode = false;
		for (int i = 0; i < 20; ++i)
		{
			if (getProgramMode() == STEPROBOTSDK::eContinue)
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

		STEPROBOTSDK::MessageData message;
		{
			std::lock_guard<std::recursive_mutex> modeLock(m_sdkCommandMutex);
			message = m_pSTEPRobotClient != nullptr
				? m_pSTEPRobotClient->getMessageData()
				: STEPROBOTSDK::MessageData();
		}
		if (message.m_MessageType == STEPROBOTSDK::eError)
		{
			const std::string startError = GetStr(
				"STEP连续运动失败：启动后控制器报错，Program=%s，%s",
				sProgramName.c_str(), GetRobotStatusText().c_str());
			const bool stopped = RobotOperationLease::StopAndConfirmUnverifiedMotion(this);
			const std::string stopDetail = GetLastRobotError();
			SetLastRobotError(startError + (stopped
				? "；已自动中止并确认程序不可恢复。"
				: "；自动中止未确认：" + stopDetail));
			m_pRobotLog->write(LogColor::ERR, "%s", GetLastRobotError().c_str());
			return -10;
		}

		Sleep(20);
	}
	if (!bStarted)
	{
		const std::string startError = GetStr(
			"STEP连续运动失败：启动后未进入运行态，Program=%s，%s",
			sProgramName.c_str(), GetRobotStatusText().c_str());
		const bool stopped = RobotOperationLease::StopAndConfirmUnverifiedMotion(this);
		const std::string stopDetail = GetLastRobotError();
		SetLastRobotError(startError + (stopped
			? "；已自动中止并确认程序不可恢复。"
			: "；自动中止未确认：" + stopDetail));
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
	if (WithSdkCommand([&]() { return m_pSTEPRobotClient->getMotorEnableState(); }) == 1)
	{
		nRet = WithSdkCommand([&]() { return m_pSTEPRobotClient->EnableMotorCmd(); });
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
	if (WithSdkCommand([&]() { return m_pSTEPRobotClient->getMotorEnableState(); }) == 0)
	{
		nRet = WithSdkCommand([&]() { return m_pSTEPRobotClient->EnableMotorCmd(); });
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
		lastRet = WithSdkCommand([&]() { return m_pSTEPRobotClient->ToolSetCmd(toolName); });
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
		lastRet = WithSdkCommand([&]() { return m_pSTEPRobotClient->VariableToolReadCmd(toolName, tTool); });
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
	nRet = WithSdkCommand([&]() { return m_pSTEPRobotClient->AllAlarmConfirmCmd(); });

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

	const int nRet = WithSdkCommand([&]() { return m_pSTEPRobotClient->OverrideSetCmd(static_cast<double>(speed)); });
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
	const int nRet = WithSdkCommand([&]() { return m_pSTEPRobotClient->VariableCartposModifyCmd(sProjectName, sProgramName, sVarName, value); });
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

	const int nRet = WithSdkCommand([&]() { return m_pSTEPRobotClient->VariableAxisposModifyCmd(sProjectName, sProgramName, sVarName, value); });
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
	const int nRet = WithSdkCommand([&]() { return m_pSTEPRobotClient->VariableAxisposModifyCmd(sProjectName, sProgramName, sVarName, eRobotCoors); });
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
	const int nRet = WithSdkCommand([&]() { return m_pSTEPRobotClient->VariableRobotAxisposModifyCmd(sProjectName, sProgramName, sVarName, eRobotCoors); });
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
		int nRet = WithSdkCommand([&]() { return m_pSTEPRobotClient->VariableAxisposReadCmd(sProjectName, sProgramName, sVarName, value); });
		if (nRet != 0)
		{
			const int fallbackRet = WithSdkCommand([&]() { return m_pSTEPRobotClient->VariableAxisposReadCmd(sProjectName, std::string(), sVarName, value); });
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
	int nRet = WithSdkCommand([&]() { return m_pSTEPRobotClient->VariableCartposReadCmd(sProjectName, sProgramName, sVarName, value); });
	if (nRet != 0)
	{
		const int fallbackRet = WithSdkCommand([&]() { return m_pSTEPRobotClient->VariableCartposReadCmd(sProjectName, std::string(), sVarName, value); });
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

bool STEPRobotCtrl::GetHandEyeMatrixVariable(const char* variableName, double rotation[9], double translation[3], std::string* error)
{
	if (variableName == nullptr || variableName[0] == '\0')
	{
		const std::string message = "STEP读取手眼矩阵失败：变量名为空。";
		SetLastRobotError(message);
		if (error != nullptr)
		{
			*error = message;
		}
		return false;
	}
	if (rotation == nullptr || translation == nullptr)
	{
		const std::string message = "STEP读取手眼矩阵失败：输出缓存为空。";
		SetLastRobotError(message);
		if (error != nullptr)
		{
			*error = message;
		}
		return false;
	}
	const std::string projectName = "_global";
	const std::string programName = "_project";
	const std::string globalProjectDataPath = "/UserPrograms/_global.sr/_project.srd";
	const std::string ftpHost = !m_sFTPIP.empty() ? m_sFTPIP : m_sSocketIP;
	std::string ftpContent;
	std::string ftpDownloadError;
	std::string ftpParseError;
	if (StepDownloadFtpTextFile(ftpHost, m_nFTPPort, m_sFTPUser, m_sFTPPassWord, globalProjectDataPath, ftpContent, &ftpDownloadError))
	{
		if (StepParseHandEyeSrd(ftpContent, variableName, rotation, translation, &ftpParseError))
		{
			ClearLastRobotError();
			if (m_pRobotLog != nullptr)
			{
				m_pRobotLog->write(
					LogColor::SUCCESS,
					"STEP已通过FTP读取全局手眼变量：%s/%s Var=%s",
					projectName.c_str(),
					programName.c_str(),
					variableName);
			}
			return true;
		}
	}

	const auto fillFromCart = [rotation, translation](const double cart[6]) {
		T_ROBOT_COORS pose;
		pose.dX = cart[0];
		pose.dY = cart[1];
		pose.dZ = cart[2];
		pose.dRX = cart[3];
		pose.dRY = cart[4];
		pose.dRZ = cart[5];
		const Eigen::Matrix3d matrix = RobotPoseTransform::RotationFromPose(pose, ROBOT_TYPE_STEP);
		for (int row = 0; row < 3; ++row)
		{
			for (int col = 0; col < 3; ++col)
			{
				rotation[row * 3 + col] = matrix(row, col);
			}
		}
		translation[0] = pose.dX;
		translation[1] = pose.dY;
		translation[2] = pose.dZ;
	};

	int lastRobotCartRet = 0;
	int lastCartRet = 0;
	if (m_pSTEPRobotClient != nullptr && IsConnected())
	{
		std::vector<std::string> names;
		names.push_back(variableName);
		std::string upperName = variableName;
		std::transform(upperName.begin(), upperName.end(), upperName.begin(), [](unsigned char ch) {
			return static_cast<char>(std::toupper(ch));
		});
		if (upperName != names.front())
		{
			names.push_back(upperName);
		}

		for (const std::string& name : names)
		{
			RobotCartPos robotCart = {};
			lastRobotCartRet = WithSdkCommand([&]() { return m_pSTEPRobotClient->VariableRobotCartposReadCmd(projectName, programName, name, robotCart); });
			if (lastRobotCartRet == 0)
			{
				fillFromCart(robotCart.cart);
				ClearLastRobotError();
				return true;
			}

			CARTPOS cart = {};
			lastCartRet = WithSdkCommand([&]() { return m_pSTEPRobotClient->VariableCartposReadCmd(projectName, programName, name, cart); });
			if (lastCartRet == 0)
			{
				fillFromCart(cart.m_CartPos.cart);
				ClearLastRobotError();
				return true;
			}
		}
	}
	else
	{
		ftpParseError += ftpParseError.empty() ? "" : "；";
		ftpParseError += "SDK兜底未执行：机器人SDK客户端未连接。";
	}

	const std::string message = GetStr(
		"STEP读取全局手眼变量失败：SDK未提供HANDEYE专用读取接口；已尝试FTP %s。FTP下载错误=%s，FTP解析错误=%s，Project=%s Program=%s Var=%s，RobotCartPos错误=%s(%d)，CARTPOS错误=%s(%d)。",
		globalProjectDataPath.c_str(),
		ftpDownloadError.empty() ? "无" : ftpDownloadError.c_str(),
		ftpParseError.empty() ? "无" : ftpParseError.c_str(),
		projectName.c_str(),
		programName.c_str(),
		variableName,
		GetErrorText(lastRobotCartRet),
		lastRobotCartRet,
		GetErrorText(lastCartRet),
		lastCartRet);
	SetLastRobotError(message);
	if (error != nullptr)
	{
		*error = message;
	}
	return false;
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
	const int nRet = WithSdkCommand([&]() { return m_pSTEPRobotClient->VariableDynamicModifyCmd(sProjectName, sProgramName, name, value); });
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
	const int nRet = WithSdkCommand([&]() { return m_pSTEPRobotClient->VariableDynamicModifyCmd(sProjectName, sProgramName, sVarName, adSpeed); });
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
	const int nRet = WithSdkCommand([&]() { return m_pSTEPRobotClient->VariableIntReadCmd(sProjectName, sProgramName, sVarName, value); });
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
	const int nRet = WithSdkCommand([&]() { return m_pSTEPRobotClient->VariableIntModifyCmd(sProjectName, sProgramName, name, value); });
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
	const int nRet = WithSdkCommand([&]() { return m_pSTEPRobotClient->VariableRealModifyCmd(sProjectName, sProgramName, sVarName, value); });
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
