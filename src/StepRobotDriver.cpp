#include "STEPRobotDriver.h"
#include "FTPClient.h"
#include "RobotDriverRegistry.h"
#include "RobotFtpFileTransfer.h"

#include "AppPaths.h"
#include "MeasureThenWeldRuntimeConfig.h"
#include "RobotOperationLease.h"
#include "RobotRecoverySafetyPolicy.h"
#include "WeldProcessValidation.h"
#include "RobotPoseTransform.h"
#include "StepSdkBuildConfig.h"

#include <QByteArray>
#include <QCryptographicHash>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QString>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <limits>
#include <mutex>
#include <sstream>
#include <thread>
#include <utility>
#include <vector>

namespace
{
	constexpr size_t kStepMaxProgramNameLength = 31; // SDK m_ProgramName[32] reserves the last byte for NUL.
	std::atomic<std::uint64_t> g_stepGeneratedNameSequence{ 0 };
	std::mutex g_stepGeneratedFilePairMutex;

	std::uint64_t StepFnv1a64(const std::string& text)
	{
		std::uint64_t value = 1469598103934665603ULL;
		for (const unsigned char ch : text)
		{
			value ^= static_cast<std::uint64_t>(ch);
			value *= 1099511628211ULL;
		}
		return value;
	}

	std::string StepBase36(std::uint64_t value, size_t width)
	{
		static constexpr char kDigits[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
		std::string result(width, '0');
		for (size_t index = width; index > 0; --index)
		{
			result[index - 1] = kDigits[value % 36ULL];
			value /= 36ULL;
		}
		return result;
	}

	std::uint64_t StepProcessNonce()
	{
		static const std::uint64_t nonce = []()
			{
				std::ostringstream identity;
				identity << GetCurrentProcessId() << '|'
					<< std::chrono::high_resolution_clock::now().time_since_epoch().count() << '|'
					<< std::chrono::steady_clock::now().time_since_epoch().count() << '|'
					<< reinterpret_cast<std::uintptr_t>(&g_stepGeneratedNameSequence);
				return StepFnv1a64(identity.str());
			}();
		return nonce;
	}

	std::string StepMakeUniqueToken(const std::string& identity)
	{
		const std::uint64_t sequence = g_stepGeneratedNameSequence.fetch_add(1, std::memory_order_relaxed) + 1;
		const std::uint64_t identityHash = StepFnv1a64(identity);
		// Multiplication by an odd constant keeps the per-process sequence bijective in 64 bits;
		// endpoint/instance identity and a process nonce isolate independent controllers/processes.
		const std::uint64_t mixedLow = StepProcessNonce()
			^ identityHash
			^ (sequence * 0x9E3779B97F4A7C15ULL);
		const std::uint64_t mixedHigh = StepFnv1a64(
			identity + "|" + std::to_string(sequence) + "|" + std::to_string(StepProcessNonce()));
		// 17 base36 characters retain about 84 bits while fitting the STEP char[32]
		// program field together with the readable weld timestamp.
		return StepBase36(mixedHigh, 4) + StepBase36(mixedLow, 13);
	}

	QString StepDecodeLocalPath(const std::string& text)
	{
		return QString::fromLocal8Bit(text.data(), static_cast<int>(text.size()));
	}

	std::filesystem::path StepFileSystemPath(const QString& path)
	{
		return std::filesystem::path(QDir::toNativeSeparators(path).toStdWString());
	}

	std::string StepLocalPathBytes(const std::filesystem::path& path)
	{
		const QString text = QDir::toNativeSeparators(QString::fromStdWString(path.wstring()));
		const QByteArray bytes = text.toLocal8Bit();
		return std::string(bytes.constData(), static_cast<size_t>(bytes.size()));
	}

	std::filesystem::path StepResolveOutputDirectory(const std::string& localDir)
	{
		if (localDir.empty())
		{
			return StepFileSystemPath(AppPaths::WritablePath(QStringLiteral("Job/STEP")));
		}

		const QString decoded = StepDecodeLocalPath(localDir);
		const QFileInfo info(QDir::fromNativeSeparators(decoded));
		const QString absolute = info.isAbsolute()
			? info.absoluteFilePath()
			: AppPaths::CommandLinePath(decoded);
		return StepFileSystemPath(absolute);
	}

	std::string StepControllerIdentity(const STEPRobotCtrl* ctrl)
	{
		if (ctrl == nullptr)
		{
			return "STEP|NO_CONTROLLER";
		}
		const RobotConnectionEndpoint controlEndpoint = ctrl->ControlEndpoint();
		const RobotFileTransferProfile fileTransfer = ctrl->FileTransferProfile();
		std::ostringstream identity;
		identity << "STEP|" << ctrl->RobotName()
			<< '|' << controlEndpoint.host << ':' << controlEndpoint.port
			<< '|' << fileTransfer.endpointDisplay
			<< "|PID=" << GetCurrentProcessId()
			<< "|INSTANCE=" << reinterpret_cast<std::uintptr_t>(ctrl);
		return identity.str();
	}

	std::filesystem::path StepInstanceOutputDirectory(const STEPRobotCtrl* ctrl)
	{
		const std::string identity = StepControllerIdentity(ctrl);
		std::ostringstream folder;
		folder << "endpoint_" << std::uppercase << std::hex << std::setw(16) << std::setfill('0')
			<< StepFnv1a64(identity)
			<< "_p" << std::dec << GetCurrentProcessId()
			<< "_i" << std::uppercase << std::hex << reinterpret_cast<std::uintptr_t>(ctrl);
		return StepResolveOutputDirectory(std::string()) / folder.str();
	}

	std::string StepMakeProgramName(const STEPRobotCtrl* ctrl = nullptr)
	{
		return "MOVE_" + StepMakeUniqueToken(StepControllerIdentity(ctrl));
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
			if (!std::isalnum(value) && ch != '_')
			{
				ch = '_';
			}
		}
		if (programName.empty())
		{
			programName = StepMakeProgramName();
		}
		if (!std::isalpha(static_cast<unsigned char>(programName.front())))
		{
			programName.insert(0, "P_");
		}
		if (programName.size() > kStepMaxProgramNameLength)
		{
			programName.resize(kStepMaxProgramNameLength);
		}
		return programName;
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
	constexpr const char* kStepCompletionWitnessName = "ntdone";
	constexpr const char* kStepCompletionWitnessHoldName = "ntdonewait";
	constexpr int kStepCompletionWitnessHoldMs = 1500;
	constexpr int kStepPauseWaitTimeoutMs = 1800000;
	constexpr int kStepCompletionWitnessSettleTimeoutMs = 1000;
	constexpr int kStepCompletionWitnessPollIntervalMs = 50;
	constexpr int kStepCompletionWitnessStableReads = 3;

	std::string StepBuildCompletionMessageToken(const std::string& programName)
	{
		return "NTDONE|" + StepSanitizeProgramName(programName);
	}

	std::string StepMessageText(const STEPROBOTSDK::MessageData& message)
	{
		const char* const begin = message.m_MessageString;
		const char* const end = std::find(
			begin, begin + sizeof(message.m_MessageString), '\0');
		return std::string(begin, end);
	}

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

	std::string StepTrim(std::string text)
	{
		const auto isSpace = [](unsigned char ch) { return std::isspace(ch) != 0; };
		const auto begin = std::find_if_not(text.begin(), text.end(), isSpace);
		const auto end = std::find_if_not(text.rbegin(), text.rend(), isSpace).base();
		return begin < end ? std::string(begin, end) : std::string();
	}

	std::string StepStripNativeProgramExtension(std::string programName)
	{
		std::string lower = programName;
		std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char ch) {
			return static_cast<char>(std::tolower(ch));
		});
		for (const std::string& extension : { std::string(".srp"), std::string(".srd"), std::string(".sr") })
		{
			if (lower.size() > extension.size()
				&& lower.compare(lower.size() - extension.size(), extension.size(), extension) == 0)
			{
				programName.resize(programName.size() - extension.size());
				break;
			}
		}
		return programName;
	}

	bool StepResolveNativeProgramIdentity(
		const std::string& requestedIdentity,
		const std::string& defaultProject,
		std::string& projectName,
		std::string& programName,
		std::string& error)
	{
		projectName.clear();
		programName.clear();
		error.clear();
		std::string identity = StepTrim(requestedIdentity);
		std::replace(identity.begin(), identity.end(), '\\', '/');
		while (!identity.empty() && identity.back() == '/')
		{
			identity.pop_back();
		}
		if (identity.empty())
		{
			error = "程序名为空。";
			return false;
		}

		const std::size_t slash = identity.find_last_of('/');
		if (slash != std::string::npos)
		{
			programName = identity.substr(slash + 1);
			std::string projectPath = identity.substr(0, slash);
			const std::size_t projectSlash = projectPath.find_last_of('/');
			projectName = projectSlash == std::string::npos
				? projectPath : projectPath.substr(projectSlash + 1);
		}
		else
		{
			const std::size_t separator = identity.find(':');
			if (separator != std::string::npos)
			{
				projectName = identity.substr(0, separator);
				programName = identity.substr(separator + 1);
			}
			else
			{
				projectName = defaultProject;
				programName = identity;
			}
		}

		projectName = StepNormalizeProjectName(StepTrim(projectName));
		programName = StepStripNativeProgramExtension(StepTrim(programName));
		if (projectName.empty() || programName.empty())
		{
			error = "工程名或程序名为空。支持 Program、Project/Program、Project:Program。";
			return false;
		}
		if (projectName.find_first_of("/\\:") != std::string::npos
			|| programName.find_first_of("/\\:") != std::string::npos
			|| projectName == "." || projectName == ".."
			|| programName == "." || programName == "..")
		{
			error = "工程名或程序名包含非法路径字符。";
			return false;
		}
		if (programName.size() > kStepMaxProgramNameLength)
		{
			error = GetStr("程序名超过STEP控制器上限：%u > %u。",
				static_cast<unsigned>(programName.size()),
				static_cast<unsigned>(kStepMaxProgramNameLength));
			return false;
		}
		return true;
	}

	bool StepResolveNativeRemoteDirectory(
		const std::string& requestedDirectory,
		const std::string& defaultProject,
		std::string& remoteDirectory,
		std::string& error)
	{
		remoteDirectory = StepTrim(requestedDirectory);
		error.clear();
		std::replace(remoteDirectory.begin(), remoteDirectory.end(), '\\', '/');
		if (remoteDirectory.empty())
		{
			remoteDirectory = StepBuildRemoteProjectDir(defaultProject);
		}
		else if (remoteDirectory.find('/') == std::string::npos)
		{
			remoteDirectory = StepBuildRemoteProjectDir(remoteDirectory);
		}
		else if (remoteDirectory.front() != '/')
		{
			remoteDirectory.insert(remoteDirectory.begin(), '/');
		}
		while (remoteDirectory.size() > 1 && remoteDirectory.back() == '/')
		{
			remoteDirectory.pop_back();
		}
		if (remoteDirectory.find("..") != std::string::npos)
		{
			error = "远端目录包含非法上级路径。";
			return false;
		}
		return true;
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

	std::string StepBuildSrdContent(
		const std::vector<T_ROBOT_MOVE_INFO>& moveInfos,
		const T_AXISUNIT& axisUnit,
		bool actualWeld,
		std::string* validationError = nullptr)
	{
		if (validationError != nullptr)
		{
			validationError->clear();
		}
		if (actualWeld)
		{
			std::string error;
			if (!WeldProcessValidation::ValidateActualMoveInfos(moveInfos, error))
			{
				if (validationError != nullptr)
				{
					*validationError = "STEP实际焊接工艺安全校验失败：" + error;
				}
				return std::string();
			}
		}
		std::ostringstream oss;
		oss << std::fixed << std::setprecision(6);
		const StepVariablePlan variablePlan = StepBuildVariablePlan(moveInfos);
		StepAppendFileComment(oss, "程序自然完成见证：START前和首行清零，物理完成后置1并留出上位机锁存窗口");
		oss << "INT " << kStepCompletionWitnessName << " := 0" << "\n";
		oss << "INT " << kStepCompletionWitnessHoldName << " := "
			<< kStepCompletionWitnessHoldMs << "\n";

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

	std::string StepBuildSrpContent(
		const std::vector<T_ROBOT_MOVE_INFO>& moveInfos,
		bool actualWeld,
		const std::string& programName)
	{
		std::ostringstream oss;
		const StepVariablePlan variablePlan = StepBuildVariablePlan(moveInfos);
		const std::string sharedOverlapName = StepBuildOverlapName(0);
		const bool hasWeldProcess = StepHasWeldProcess(moveInfos);
		const bool emitWeldCommands = hasWeldProcess && actualWeld;
		StepAppendFileComment(oss, "任何新运行先清零自然完成见证，禁止沿用上一次结果");
		StepAppendCommand(oss, std::string(kStepCompletionWitnessName) + ":=0;");
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
		StepAppendFileComment(oss, "打断预读并等待末段物理运动及收弧真正完成");
		StepAppendCommand(oss, "WaitIsFinished();");
		StepAppendFileComment(oss, "仅在全部运动以及可选ARCOFF完成后写入自然完成见证");
		StepAppendCommand(oss, std::string(kStepCompletionWitnessName) + ":=1;");
		StepAppendFileComment(oss, "发送本次唯一完成令牌，并保持运行态供SDK锁存；不写控制器磁盘");
		StepAppendCommand(oss,
			"Message(\"" + StepBuildCompletionMessageToken(programName)
			+ "\",eInfo," + kStepCompletionWitnessName + "," + kStepCompletionWitnessName + ");");
		StepAppendCommand(oss,
			std::string("WaitTime(") + kStepCompletionWitnessHoldName + ",TRUE);");

		return oss.str();
	}

	bool StepAtomicReplaceTextFile(const std::filesystem::path& filePath, const std::string& content)
	{
		for (int attempt = 0; attempt < 8; ++attempt)
		{
			const std::string temporaryToken = StepMakeUniqueToken(StepLocalPathBytes(filePath));
			std::filesystem::path temporaryPath = filePath;
			temporaryPath += std::filesystem::path(
				L".tmp." + std::to_wstring(GetCurrentProcessId()) + L"."
				+ std::wstring(temporaryToken.begin(), temporaryToken.end()));

			HANDLE fileHandle = CreateFileW(
				temporaryPath.c_str(),
				GENERIC_WRITE,
				0,
				nullptr,
				CREATE_NEW,
				FILE_ATTRIBUTE_TEMPORARY,
				nullptr);
			if (fileHandle == INVALID_HANDLE_VALUE)
			{
				if (GetLastError() == ERROR_FILE_EXISTS || GetLastError() == ERROR_ALREADY_EXISTS)
				{
					continue;
				}
				return false;
			}

			bool writeOk = true;
			size_t offset = 0;
			while (offset < content.size())
			{
				const DWORD chunkSize = static_cast<DWORD>(std::min<size_t>(
					content.size() - offset,
					static_cast<size_t>(std::numeric_limits<DWORD>::max())));
				DWORD written = 0;
				if (!WriteFile(fileHandle, content.data() + offset, chunkSize, &written, nullptr)
					|| written == 0)
				{
					writeOk = false;
					break;
				}
				offset += written;
			}
			if (writeOk && !FlushFileBuffers(fileHandle))
			{
				writeOk = false;
			}
			if (!CloseHandle(fileHandle))
			{
				writeOk = false;
			}
			if (writeOk && MoveFileExW(
				temporaryPath.c_str(),
				filePath.c_str(),
				MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH))
			{
				return true;
			}
			DeleteFileW(temporaryPath.c_str());
			return false;
		}
		return false;
	}

	bool StepWriteTextFile(const std::filesystem::path& filePath, const std::string& content)
	{
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

		return StepAtomicReplaceTextFile(filePath, normalized);
	}

	constexpr qint64 kStepMaximumGeneratedProgramBytes = 128LL * 1024LL * 1024LL;

	bool StepReadProgramContentIdentity(
		const std::filesystem::path& filePath,
		RobotRecoverySafetyPolicy::ProgramContentIdentity& identity,
		std::string& error)
	{
		identity = {};
		error.clear();
		QFile file(QString::fromStdWString(filePath.wstring()));
		if (!file.open(QIODevice::ReadOnly))
		{
			error = "无法只读打开生成程序文件：" + StepLocalPathBytes(filePath);
			return false;
		}
		const qint64 expectedSize = file.size();
		if (expectedSize < 0 || expectedSize > kStepMaximumGeneratedProgramBytes)
		{
			error = "生成程序文件大小无效或超过128MiB上限：" + StepLocalPathBytes(filePath);
			return false;
		}
		QCryptographicHash hash(QCryptographicHash::Sha256);
		qint64 totalRead = 0;
		while (!file.atEnd())
		{
			const QByteArray chunk = file.read(64 * 1024);
			if (chunk.isEmpty() && file.error() != QFileDevice::NoError)
			{
				error = "读取生成程序文件失败：" + StepLocalPathBytes(filePath);
				return false;
			}
			hash.addData(chunk);
			totalRead += chunk.size();
		}
		if (totalRead != expectedSize || file.size() != expectedSize)
		{
			error = "生成程序文件在身份计算期间发生变化：" + StepLocalPathBytes(filePath);
			return false;
		}
		identity.size = expectedSize;
		identity.sha256 = hash.result().toHex().toLower();
		if (!identity.IsValid())
		{
			error = "生成程序文件内容身份无效：" + StepLocalPathBytes(filePath);
			return false;
		}
		return true;
	}

	struct StepTemporaryFileCleanup
	{
		std::filesystem::path programPath;
		std::filesystem::path dataPath;
		~StepTemporaryFileCleanup()
		{
			std::error_code ignored;
			std::filesystem::remove(programPath, ignored);
			ignored.clear();
			std::filesystem::remove(dataPath, ignored);
		}
	};

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
	std::tm localTime = {};
#if defined(_WIN32)
	localtime_s(&localTime, &nowTime);
#else
	localtime_r(&nowTime, &localTime);
#endif

	std::ostringstream oss;
	// Preserve the established Weld_ prefix while fitting a readable hour stamp
	// and an 84-bit process/sequence token in the STEP char[32] field.
	oss << "Weld_" << std::put_time(&localTime, "%y%m%d%H")
		<< "_" << StepMakeUniqueToken("STEP_WELD_PROGRAM");
	return StepSanitizeProgramName(oss.str());
}

void STEPRobotCtrl::ClearGeneratedProgramContentWitnessLocked()
{
	if (++m_contentWitnessGeneration == 0)
	{
		++m_contentWitnessGeneration;
	}
	m_contentWitnessProjectName.clear();
	m_contentWitnessProgramName.clear();
	m_contentWitnessRemoteProgramPath.clear();
	m_contentWitnessRemoteDataPath.clear();
	m_contentWitnessProgramIdentity = {};
	m_contentWitnessDataIdentity = {};
}

void STEPRobotCtrl::ClearGeneratedProgramCompletionWitnessLocked(bool preserveContentWitness)
{
	m_completionWitnessProjectName.clear();
	m_completionWitnessProgramName.clear();
	m_completionWitnessMessageToken.clear();
	m_completionWitnessRuntimeLatched = false;
	if (!preserveContentWitness)
	{
		ClearGeneratedProgramContentWitnessLocked();
	}
}

bool STEPRobotCtrl::VerifyGeneratedProgramRemoteContentLocked(std::string& error)
{
	RobotRecoverySafetyPolicy::ProgramContentWitnessSnapshot snapshot;
	snapshot.generation = m_contentWitnessGeneration;
	snapshot.projectName = m_contentWitnessProjectName;
	snapshot.programName = m_contentWitnessProgramName;
	snapshot.remoteProgramPath = m_contentWitnessRemoteProgramPath;
	snapshot.remoteDataPath = m_contentWitnessRemoteDataPath;
	snapshot.programIdentity = m_contentWitnessProgramIdentity;
	snapshot.dataIdentity = m_contentWitnessDataIdentity;
	auto cancelToken = std::make_shared<std::atomic<bool>>(false);
	return VerifyGeneratedProgramRemoteContentSnapshot(snapshot, cancelToken, error);
}

bool STEPRobotCtrl::VerifyGeneratedProgramRemoteContentSnapshot(
	const RobotRecoverySafetyPolicy::ProgramContentWitnessSnapshot& snapshot,
	const RobotRecoverySafetyPolicy::RemoteContentVerificationGate::Token& cancelToken,
	std::string& error)
{
	error.clear();
	if (m_pFTP == nullptr || cancelToken == nullptr || !snapshot.IsValid())
	{
		error = "STEP远端程序内容见证未建立。";
		return false;
	}
	std::filesystem::path verifyDirectory;
	try
	{
		verifyDirectory = StepInstanceOutputDirectory(this) / "verify";
		std::filesystem::create_directories(verifyDirectory);
	}
	catch (const std::exception& e)
	{
		error = std::string("STEP远端内容回读临时目录创建失败：") + e.what();
		return false;
	}
	const std::string token = StepMakeUniqueToken(
		snapshot.projectName + "|" + snapshot.programName + "|REMOTE_VERIFY");
	StepTemporaryFileCleanup cleanup;
	cleanup.programPath = verifyDirectory / (token + ".srp.readback.tmp");
	cleanup.dataPath = verifyDirectory / (token + ".srd.readback.tmp");
	m_pFTP->setMessageBoxesEnabled(false);
	if (!RobotRecoverySafetyPolicy::ProgramContentWithinLimit(
			snapshot.programIdentity, kStepMaximumGeneratedProgramBytes)
		|| !RobotRecoverySafetyPolicy::ProgramContentWithinLimit(
			snapshot.dataIdentity, kStepMaximumGeneratedProgramBytes))
	{
		error = "STEP远端SRP/SRD声明大小为空或超过128MiB上限，读取主体前拒绝。";
		return false;
	}
	if (!m_pFTP->downloadFileBounded(
			snapshot.remoteProgramPath,
			StepLocalPathBytes(cleanup.programPath),
			static_cast<unsigned long long>(snapshot.programIdentity.size),
			static_cast<unsigned long long>(kStepMaximumGeneratedProgramBytes),
			cancelToken.get())
		|| !m_pFTP->downloadFileBounded(
			snapshot.remoteDataPath,
			StepLocalPathBytes(cleanup.dataPath),
			static_cast<unsigned long long>(snapshot.dataIdentity.size),
			static_cast<unsigned long long>(kStepMaximumGeneratedProgramBytes),
			cancelToken.get()))
	{
		error = "STEP远端SRP/SRD下载回读失败，无法建立内容身份。";
		return false;
	}
	RobotRecoverySafetyPolicy::ProgramContentIdentity observedProgram;
	RobotRecoverySafetyPolicy::ProgramContentIdentity observedData;
	std::string identityError;
	if (!StepReadProgramContentIdentity(cleanup.programPath, observedProgram, identityError)
		|| !StepReadProgramContentIdentity(cleanup.dataPath, observedData, identityError))
	{
		error = "STEP远端SRP/SRD回读身份计算失败：" + identityError;
		return false;
	}
	if (!RobotRecoverySafetyPolicy::SameProgramContent(
			snapshot.programIdentity, observedProgram)
		|| !RobotRecoverySafetyPolicy::SameProgramContent(
			snapshot.dataIdentity, observedData))
	{
		error = GetStr(
			"STEP远端SRP/SRD内容身份变化：Program=%s SRP=%lld/%lld:%s/%s SRD=%lld/%lld:%s/%s",
			snapshot.programName.c_str(),
			static_cast<long long>(snapshot.programIdentity.size),
			static_cast<long long>(observedProgram.size),
			snapshot.programIdentity.sha256.constData(), observedProgram.sha256.constData(),
			static_cast<long long>(snapshot.dataIdentity.size),
			static_cast<long long>(observedData.size),
			snapshot.dataIdentity.sha256.constData(), observedData.sha256.constData());
		return false;
	}
	return true;
}

bool STEPRobotCtrl::CaptureGeneratedProgramContentSnapshotLocked(
	const std::string& expectedProject,
	const std::string& expectedProgram,
	RobotRecoverySafetyPolicy::ProgramContentWitnessSnapshot& snapshot,
	std::string& error) const
{
	snapshot = {};
	if (m_pSTEPRobotClient == nullptr || m_pFTP == nullptr
		|| expectedProject.empty() || expectedProgram.empty())
	{
		error = "STEP START后内容复核缺少SDK、FTP或程序身份。";
		return false;
	}
	if (!m_bLocalDebugMark
		&& (!m_bSocketConnected.load() || m_pSTEPRobotClient->ConnectStatus() < 0))
	{
		error = "STEP START后内容复核时机器人连接不可用。";
		return false;
	}
	snapshot.generation = m_contentWitnessGeneration;
	snapshot.projectName = m_contentWitnessProjectName;
	snapshot.programName = m_contentWitnessProgramName;
	snapshot.remoteProgramPath = m_contentWitnessRemoteProgramPath;
	snapshot.remoteDataPath = m_contentWitnessRemoteDataPath;
	snapshot.programIdentity = m_contentWitnessProgramIdentity;
	snapshot.dataIdentity = m_contentWitnessDataIdentity;
	snapshot.observedProjectName = StepNormalizeProjectName(
		m_pSTEPRobotClient->getProjectName());
	snapshot.observedProgramName = m_pSTEPRobotClient->getProgramName();
	snapshot.observedProgramState = static_cast<int>(
		m_pSTEPRobotClient->getProgramState());
	if (!snapshot.IsValid()
		|| snapshot.projectName != expectedProject
		|| snapshot.programName != expectedProgram
		|| snapshot.observedProjectName != expectedProject
		|| snapshot.observedProgramName != expectedProgram)
	{
		error = GetStr(
			"STEP START后内容复核快照身份不一致。Witness=%s/%s Current=%s/%s Expected=%s/%s",
			snapshot.projectName.c_str(), snapshot.programName.c_str(),
			snapshot.observedProjectName.c_str(), snapshot.observedProgramName.c_str(),
			expectedProject.c_str(), expectedProgram.c_str());
		return false;
	}
	return true;
}

void STEPRobotCtrl::ClearGeneratedProgramContentWitnessIfSnapshotLocked(
	const RobotRecoverySafetyPolicy::ProgramContentWitnessSnapshot& snapshot)
{
	RobotRecoverySafetyPolicy::ProgramContentWitnessSnapshot current;
	current.generation = m_contentWitnessGeneration;
	current.projectName = m_contentWitnessProjectName;
	current.programName = m_contentWitnessProgramName;
	current.remoteProgramPath = m_contentWitnessRemoteProgramPath;
	current.remoteDataPath = m_contentWitnessRemoteDataPath;
	current.programIdentity = m_contentWitnessProgramIdentity;
	current.dataIdentity = m_contentWitnessDataIdentity;
	if (RobotRecoverySafetyPolicy::SameProgramContentWitness(snapshot, current)
		&& m_completionWitnessProjectName == snapshot.projectName
		&& m_completionWitnessProgramName == snapshot.programName)
	{
		ClearGeneratedProgramCompletionWitnessLocked();
	}
}

void STEPRobotCtrl::CancelActiveRemoteContentVerification()
{
	m_remoteContentVerificationGate.CancelActive();
}

bool STEPRobotCtrl::VerifyGeneratedProgramAfterStartWithSdkUnlock(
	std::unique_lock<std::recursive_mutex>& sdkLock,
	const std::string& expectedProject,
	const std::string& expectedProgram,
	std::string& error)
{
	error.clear();
	if (!sdkLock.owns_lock())
	{
		error = "STEP START后内容复核调用未持SDK锁。";
		return false;
	}
	RobotRecoverySafetyPolicy::ProgramContentWitnessSnapshot snapshot;
	const auto failStartedProgramLocked = [&](const std::string& detail)
		{
			const bool sameProgramStillLoaded = m_pSTEPRobotClient != nullptr
				&& StepNormalizeProjectName(m_pSTEPRobotClient->getProjectName()) == expectedProject
				&& m_pSTEPRobotClient->getProgramName() == expectedProgram;
			const bool stopped = sameProgramStillLoaded
				&& StopAndUnloadGeneratedProgramLocked(expectedProject, expectedProgram);
			if (stopped)
			{
				RobotOperationLease::MarkMotionCompleted(this);
			}
			if (snapshot.IsValid())
			{
				ClearGeneratedProgramContentWitnessIfSnapshotLocked(snapshot);
			}
			error = detail + (stopped
				? "；原快照绑定程序已STOP/Kill并稳定确认。"
				: "；未确认原快照绑定程序已终止，运动闭锁保持有效。");
			return false;
		};
	if (!CaptureGeneratedProgramContentSnapshotLocked(
			expectedProject, expectedProgram, snapshot, error))
	{
		const std::string captureError = error;
		return failStartedProgramLocked(captureError);
	}
	// 极短/零距离程序可能在 START 返回后立刻越过物理完成屏障。先在当前
	// SDK 快照中尝试锁存一次，再进入不持 SDK 锁的 FTP 内容复核窗口。
	{
		std::string ignoredWitnessError;
		VerifyGeneratedProgramCompletionWitnessLocked(ignoredWitnessError);
	}
	const auto cancelToken = m_remoteContentVerificationGate.TryBegin();
	if (cancelToken == nullptr)
	{
		return failStartedProgramLocked(
			"STEP START后已有另一内容复核正在执行，已失败关闭。");
	}

	// 关键：WinINet 回读期间不持 m_sdkCommandMutex。红色 STOP 可立即取得
	// SDK 锁执行 STOP/Kill，并通过独立 token 让下载在下一取消点尽快结束。
	// 同时用一个短生命周期观察线程锁存运行态完成令牌，避免慢 FTP 吃掉
	// SRP 末尾 1.5s 的令牌保持窗口；线程只读 SDK 状态并在返回前必定 join。
	sdkLock.unlock();
	std::atomic<bool> stopCompletionObserver{ false };
	std::thread completionObserver;
	try
	{
		completionObserver = std::thread([this, &stopCompletionObserver, &cancelToken]()
			{
				try
				{
					while (!stopCompletionObserver.load(std::memory_order_acquire)
						&& !cancelToken->load(std::memory_order_acquire))
					{
						{
							std::lock_guard<std::recursive_mutex> observerLock(m_sdkCommandMutex);
							std::string ignoredWitnessError;
							VerifyGeneratedProgramCompletionWitnessLocked(ignoredWitnessError);
						}
						Sleep(kStepCompletionWitnessPollIntervalMs);
					}
				}
				catch (...)
				{
					// 观察器异常不得越过线程边界；主线程随后仍会按未见证状态失败关闭。
				}
			});
	}
	catch (const std::exception& e)
	{
		sdkLock.lock();
		m_remoteContentVerificationGate.End(cancelToken);
		return failStartedProgramLocked(
			std::string("STEP START后完成令牌观察器创建失败：") + e.what());
	}
	std::string downloadError;
	bool downloadVerified = false;
	try
	{
		downloadVerified = VerifyGeneratedProgramRemoteContentSnapshot(
			snapshot, cancelToken, downloadError);
	}
	catch (const std::exception& e)
	{
		downloadError = std::string("STEP START后远端内容复核异常：") + e.what();
	}
	catch (...)
	{
		downloadError = "STEP START后远端内容复核发生未知异常。";
	}
	stopCompletionObserver.store(true, std::memory_order_release);
	completionObserver.join();
	sdkLock.lock();
	m_remoteContentVerificationGate.End(cancelToken);

	RobotRecoverySafetyPolicy::ProgramContentWitnessSnapshot current;
	std::string currentError;
	const bool currentCaptured = CaptureGeneratedProgramContentSnapshotLocked(
		expectedProject, expectedProgram, current, currentError);
	const int currentState = current.observedProgramState;
	// START 返回后只接受仍在运行或已自然到达停止态；若又变为暂停，
	// 说明下载窗口中发生了外部状态切换，必须失败关闭。
	const bool currentStateAllowed = currentState == STEPROBOTSDK::eRun
		|| currentState == STEPROBOTSDK::eStop;
	const bool snapshotStateKnown = snapshot.observedProgramState == STEPROBOTSDK::eRun
		|| snapshot.observedProgramState == STEPROBOTSDK::ePause
		|| snapshot.observedProgramState == STEPROBOTSDK::eStop;
	const bool snapshotStillBound = currentCaptured
		&& RobotRecoverySafetyPolicy::SameProgramContentWitness(snapshot, current)
		&& m_motionTrackedProjectName == expectedProject
		&& m_motionTrackedProgramName == expectedProgram
		&& m_completionWitnessProjectName == expectedProject
		&& m_completionWitnessProgramName == expectedProgram
		&& snapshotStateKnown
		&& currentStateAllowed;
	const bool cancelled = cancelToken->load(std::memory_order_acquire)
		|| RobotOperationLease::IsCancellationRequested(this);
	if (downloadVerified && snapshotStillBound && !cancelled)
	{
		return true;
	}

	const std::string detail = !downloadVerified
		? downloadError
		: (!currentError.empty() ? currentError
			: (cancelled ? "红色STOP已取消内容复核。"
				: "下载期间见证、当前程序、状态或运动身份已变化。"));
	// 只能停止/清除本次冻结的同一程序和见证；下载期间若被其他程序
	// 替换，绝不误杀或误清，且 MotionCompletionPending 保持 sticky。
	return failStartedProgramLocked(detail);
}

bool STEPRobotCtrl::ArmGeneratedProgramContentWitness(
	const std::string& projectName,
	const std::string& programName,
	const std::string& remoteProgramPath,
	const std::string& remoteDataPath,
	const RobotRecoverySafetyPolicy::ProgramContentIdentity& programIdentity,
	const RobotRecoverySafetyPolicy::ProgramContentIdentity& dataIdentity,
	std::string& error)
{
	std::lock_guard<std::recursive_mutex> sdkLock(m_sdkCommandMutex);
	ClearGeneratedProgramContentWitnessLocked();
	if (projectName.empty() || programName.empty()
		|| remoteProgramPath.empty() || remoteDataPath.empty()
		|| !programIdentity.IsValid() || !dataIdentity.IsValid())
	{
		error = "STEP生成程序内容见证参数无效。";
		return false;
	}
	m_contentWitnessProjectName = projectName;
	m_contentWitnessProgramName = programName;
	m_contentWitnessRemoteProgramPath = remoteProgramPath;
	m_contentWitnessRemoteDataPath = remoteDataPath;
	m_contentWitnessProgramIdentity = programIdentity;
	m_contentWitnessDataIdentity = dataIdentity;
	if (!VerifyGeneratedProgramRemoteContentLocked(error))
	{
		ClearGeneratedProgramContentWitnessLocked();
		return false;
	}
	return true;
}

bool STEPRobotCtrl::StopAndUnloadGeneratedProgramLocked(
	const std::string& projectName,
	const std::string& programName)
{
	if (m_pSTEPRobotClient == nullptr
		|| programName.empty()
		|| StepNormalizeProjectName(m_pSTEPRobotClient->getProjectName()) != projectName
		|| m_pSTEPRobotClient->getProgramName() != programName)
	{
		return false;
	}
	m_pSTEPRobotClient->SetModeCmd(MODEKEY::STOP, true);
	const int killRet = m_pSTEPRobotClient->ProgramKillCmd(projectName, programName, true);
	if (killRet != 0)
	{
		return false;
	}
	int stableCount = 0;
	for (int retry = 0; retry < 40; ++retry)
	{
		if (m_pSTEPRobotClient->getProgramName().empty()
			&& m_pSTEPRobotClient->getProgramState() == STEPROBOTSDK::eStop)
		{
			if (++stableCount >= 4)
			{
				return true;
			}
		}
		else
		{
			stableCount = 0;
		}
		Sleep(50);
	}
	return false;
}

bool STEPRobotCtrl::ArmGeneratedProgramCompletionWitness(
	const std::string& projectName,
	const std::string& programName,
	std::string& error)
{
	error.clear();
	std::lock_guard<std::recursive_mutex> sdkLock(m_sdkCommandMutex);
	ClearGeneratedProgramCompletionWitnessLocked(true);
	if (m_pSTEPRobotClient == nullptr || projectName.empty() || programName.empty())
	{
		error = "STEP自然完成见证准备失败：SDK客户端、工程或程序身份无效。";
		return false;
	}
	if (!m_bLocalDebugMark
		&& (!m_bSocketConnected.load() || m_pSTEPRobotClient->ConnectStatus() < 0))
	{
		error = "STEP自然完成见证准备失败：机器人连接不可用。";
		return false;
	}
	const std::string currentProject = StepNormalizeProjectName(m_pSTEPRobotClient->getProjectName());
	const std::string currentProgram = m_pSTEPRobotClient->getProgramName();
	const int initialLine = m_pSTEPRobotClient->getCurrentLine();
	const int initialState = static_cast<int>(m_pSTEPRobotClient->getProgramState());
	if (currentProject != projectName || currentProgram != programName
		|| initialLine < 0 || initialLine > 1
		|| initialState != STEPROBOTSDK::eStop)
	{
		error = GetStr(
			"STEP自然完成见证准备失败：加载身份、初始程序行或状态异常。Expected=%s/%s Current=%s/%s Line=%d State=%d",
			projectName.c_str(), programName.c_str(), currentProject.c_str(), currentProgram.c_str(),
			initialLine, initialState);
		return false;
	}
	const int writeRet = m_pSTEPRobotClient->VariableIntModifyCmd(
		projectName, programName, kStepCompletionWitnessName, 0);
	int readValue = -1;
	const int readRet = writeRet == 0
		? m_pSTEPRobotClient->VariableIntReadCmd(
			projectName, programName, kStepCompletionWitnessName, readValue)
		: -1;
	const std::string expectedMessageToken = StepBuildCompletionMessageToken(programName);
	const STEPROBOTSDK::MessageData initialMessage = m_pSTEPRobotClient->getMessageData();
	const std::string initialMessageText = StepMessageText(initialMessage);
	if (writeRet != 0 || readRet != 0 || readValue != 0
		|| initialMessageText == expectedMessageToken)
	{
		error = GetStr(
			"STEP自然完成见证清零/负检查失败：Program=%s Variable=%s WriteRet=%d ReadRet=%d Value=%d MessageCollision=%d",
			programName.c_str(), kStepCompletionWitnessName, writeRet, readRet, readValue,
			initialMessageText == expectedMessageToken ? 1 : 0);
		return false;
	}
	m_completionWitnessProjectName = projectName;
	m_completionWitnessProgramName = programName;
	m_completionWitnessMessageToken = expectedMessageToken;
	m_completionWitnessRuntimeLatched = false;
	return true;
}

bool STEPRobotCtrl::VerifyGeneratedProgramCompletionWitnessLocked(std::string& error)
{
	error.clear();
	if (m_pSTEPRobotClient == nullptr)
	{
		error = "SDK客户端未初始化";
		return false;
	}
	if (!m_bLocalDebugMark
		&& (!m_bSocketConnected.load() || m_pSTEPRobotClient->ConnectStatus() < 0))
	{
		error = "机器人连接不可用，状态和变量回读可能陈旧";
		return false;
	}
	const std::string currentProject = StepNormalizeProjectName(m_pSTEPRobotClient->getProjectName());
	const std::string currentProgram = m_pSTEPRobotClient->getProgramName();
	if (m_completionWitnessProjectName.empty() || m_completionWitnessProgramName.empty()
		|| m_completionWitnessMessageToken.empty()
		|| m_completionWitnessProjectName != m_motionTrackedProjectName
		|| m_completionWitnessProgramName != m_motionTrackedProgramName
		|| currentProject != m_motionTrackedProjectName
		|| currentProgram != m_motionTrackedProgramName)
	{
		error = GetStr(
			"缺少同一程序的自然完成见证。Witness=%s/%s Tracked=%s/%s Current=%s/%s",
			m_completionWitnessProjectName.c_str(), m_completionWitnessProgramName.c_str(),
			m_motionTrackedProjectName.c_str(), m_motionTrackedProgramName.c_str(),
			currentProject.c_str(), currentProgram.c_str());
		return false;
	}
	if (m_completionWitnessRuntimeLatched)
	{
		return true;
	}
	int witnessValue = -1;
	const int readRet = m_pSTEPRobotClient->VariableIntReadCmd(
		currentProject, currentProgram, kStepCompletionWitnessName, witnessValue);
	const STEPROBOTSDK::MessageData message = m_pSTEPRobotClient->getMessageData();
	const std::string messageText = StepMessageText(message);
	const bool exactMessageWitness = message.m_MessageType == STEPROBOTSDK::eInfo
		&& messageText == m_completionWitnessMessageToken;
	const bool runtimeVariableWitness = readRet == 0 && witnessValue == 1;
	if (!exactMessageWitness && !runtimeVariableWitness)
	{
		error = GetStr(
			"自然完成运行态见证未观察到：Program=%s Variable=%s ReadRet=%d Value=%d MessageType=%d Message=%s Line=%d",
			currentProgram.c_str(), kStepCompletionWitnessName, readRet, witnessValue,
			static_cast<int>(message.m_MessageType), messageText.c_str(),
			m_pSTEPRobotClient->getCurrentLine());
		return false;
	}
	m_completionWitnessRuntimeLatched = true;
	return true;
}

bool STEPRobotCtrl::VerifyGeneratedProgramReadyForStartLocked(
	const std::string& projectName,
	const std::string& programName,
	std::string& error)
{
	error.clear();
	if (m_pSTEPRobotClient == nullptr
		|| m_completionWitnessProjectName != projectName
		|| m_completionWitnessProgramName != programName
		|| StepNormalizeProjectName(m_pSTEPRobotClient->getProjectName()) != projectName
		|| m_pSTEPRobotClient->getProgramName() != programName)
	{
		error = GetStr(
			"STEP START前程序身份与完成见证不一致。Witness=%s/%s Expected=%s/%s Current=%s/%s",
			m_completionWitnessProjectName.c_str(), m_completionWitnessProgramName.c_str(),
			projectName.c_str(), programName.c_str(),
			m_pSTEPRobotClient != nullptr
				? StepNormalizeProjectName(m_pSTEPRobotClient->getProjectName()).c_str() : "",
			m_pSTEPRobotClient != nullptr ? m_pSTEPRobotClient->getProgramName().c_str() : "");
		return false;
	}
	if (!m_bLocalDebugMark
		&& (!m_bSocketConnected.load() || m_pSTEPRobotClient->ConnectStatus() < 0))
	{
		error = "STEP START前机器人连接不可用。";
		return false;
	}
	std::string remoteContentError;
	if (!VerifyGeneratedProgramRemoteContentLocked(remoteContentError))
	{
		const bool stopped = StopAndUnloadGeneratedProgramLocked(projectName, programName);
		error = "STEP START前远端SRP/SRD内容复核失败，已拒绝START："
			+ remoteContentError
			+ (stopped ? "；已STOP/Kill并稳定确认卸载。" : "；STOP/Kill未能稳定确认。" );
		ClearGeneratedProgramContentWitnessLocked();
		return false;
	}
	const int currentLine = m_pSTEPRobotClient->getCurrentLine();
	const int currentState = static_cast<int>(m_pSTEPRobotClient->getProgramState());
	int witnessValue = -1;
	const int witnessRet = m_pSTEPRobotClient->VariableIntReadCmd(
		projectName, programName, kStepCompletionWitnessName, witnessValue);
	const STEPROBOTSDK::MessageData currentMessage = m_pSTEPRobotClient->getMessageData();
	const std::string currentMessageText = StepMessageText(currentMessage);
	const std::string expectedMessageToken = StepBuildCompletionMessageToken(programName);
	if (currentLine < 0 || currentLine > 1
		|| currentState != STEPROBOTSDK::eStop
		|| witnessRet != 0 || witnessValue != 0
		|| m_completionWitnessRuntimeLatched
		|| m_completionWitnessMessageToken != expectedMessageToken
		|| currentMessageText == expectedMessageToken)
	{
		error = GetStr(
			"STEP START前程序不在可验证初始态：Program=%s Line=%d State=%d WitnessRet=%d Witness=%d Latched=%d TokenBound=%d MessageCollision=%d",
			programName.c_str(), currentLine, currentState, witnessRet, witnessValue,
			m_completionWitnessRuntimeLatched ? 1 : 0,
			m_completionWitnessMessageToken == expectedMessageToken ? 1 : 0,
			currentMessageText == expectedMessageToken ? 1 : 0);
		return false;
	}
	return true;
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
	const std::filesystem::path dirPath = StepResolveOutputDirectory(localDir);
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

	const std::string srpContent = StepBuildSrpContent(
		weaveMoveInfo, actualWeld, safeProgramName);
	std::string processValidationError;
	const std::string srdContent = StepBuildSrdContent(
		weaveMoveInfo, axisUnit, actualWeld, &processValidationError);
	if (!processValidationError.empty())
	{
		if (errorText != nullptr)
		{
			*errorText = processValidationError;
		}
		return false;
	}
	const std::string srpPathText = StepLocalPathBytes(srpPath);
	const std::string srdPathText = StepLocalPathBytes(srdPath);

	{
		// Keep the SRP/SRD pair coherent even when two offline generation requests
		// target the same explicit name. Each individual replacement is atomic.
		std::lock_guard<std::mutex> filePairLock(g_stepGeneratedFilePairMutex);
		if (!StepWriteTextFile(srpPath, srpContent))
		{
			if (errorText != nullptr)
			{
				*errorText = "写入SRP失败：" + srpPathText;
			}
			return false;
		}
		if (!StepWriteTextFile(srdPath, srdContent))
		{
			if (errorText != nullptr)
			{
				*errorText = "写入SRD失败：" + srdPathText;
			}
			return false;
		}
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
	delete m_pFTP;
	m_pFTP = nullptr;
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

RobotDriverDescriptor STEPRobotCtrl::DriverDescriptor() const
{
	return RobotDriverDescriptor{
		RobotDriverFamily::Step,
		ROBOT_TYPE_STEP,
		ROBOT_TYPE_STEP,
		"STEP",
		"STEP"
	};
}

RobotConnectionEndpoint STEPRobotCtrl::ControlEndpoint() const
{
	return RobotConnectionEndpoint{ m_sSocketIP, m_nSocketPort };
}

bool STEPRobotCtrl::Connect()
{
	const RobotConnectionEndpoint endpoint = ControlEndpoint();
	if (!endpoint.IsValid())
	{
		SetLastRobotError("STEP连接参数不完整。");
		return false;
	}
	return InitSocket(endpoint.host.c_str(), static_cast<unsigned short>(endpoint.port));
}

bool STEPRobotCtrl::Disconnect()
{
	return CloseSocket();
}

std::uint64_t STEPRobotCtrl::DriverCapabilities() const
{
	return RobotDriverCapabilityBit(RobotDriverCapability::PassiveState)
		| RobotDriverCapabilityBit(RobotDriverCapability::RobotTimestamp)
		| RobotDriverCapabilityBit(RobotDriverCapability::LinearMotion)
		| RobotDriverCapabilityBit(RobotDriverCapability::JointMotion)
		| RobotDriverCapabilityBit(RobotDriverCapability::ContinuousTrajectory)
		| RobotDriverCapabilityBit(RobotDriverCapability::PauseResume)
		| RobotDriverCapabilityBit(RobotDriverCapability::PersistentProgramRecovery)
		| RobotDriverCapabilityBit(RobotDriverCapability::OperationModeControl)
		| RobotDriverCapabilityBit(RobotDriverCapability::NativeProgramUpload)
		| RobotDriverCapabilityBit(RobotDriverCapability::DiagnosticCommand)
		| RobotDriverCapabilityBit(RobotDriverCapability::CartesianRegister)
		| RobotDriverCapabilityBit(RobotDriverCapability::VerifiedProgramCompletion)
		| RobotDriverCapabilityBit(RobotDriverCapability::VerifiedSafeAbort)
		| RobotDriverCapabilityBit(RobotDriverCapability::ActualArcWeld)
		| RobotDriverCapabilityBit(RobotDriverCapability::ExternalAxis)
		| RobotDriverCapabilityBit(RobotDriverCapability::OfflineTrajectoryExport)
		| RobotDriverCapabilityBit(RobotDriverCapability::ConnectionControl)
		| RobotDriverCapabilityBit(RobotDriverCapability::AlarmReset)
		| RobotDriverCapabilityBit(RobotDriverCapability::ServoPowerControl)
		| RobotDriverCapabilityBit(RobotDriverCapability::ToolDataRead)
		| RobotDriverCapabilityBit(RobotDriverCapability::IntegerRegister)
		| RobotDriverCapabilityBit(RobotDriverCapability::TeachPendantSpeedControl)
		| RobotDriverCapabilityBit(RobotDriverCapability::NativeProgramExecution)
		| RobotDriverCapabilityBit(RobotDriverCapability::FtpFileTransfer)
		| RobotDriverCapabilityBit(RobotDriverCapability::HandEyeMatrixRead)
		| RobotDriverCapabilityBit(RobotDriverCapability::HandEyeSupportProgramInstall);
}

RobotFileTransferProfile STEPRobotCtrl::FileTransferProfile() const
{
	RobotFileTransferProfile profile;
	profile.robotName = m_sRobotName;
	profile.endpointDisplay = m_sFTPIP;
	if (!profile.endpointDisplay.empty() && m_nFTPPort > 0)
	{
		profile.endpointDisplay += ":" + std::to_string(m_nFTPPort);
	}
	profile.defaultRemoteDirectory = StepBuildRemoteProjectDir(
		m_sStepProjectName.empty() ? kStepDynamicJobProjectName : m_sStepProjectName);
	profile.defaultLocalDirectory = "Job/STEP";
	profile.localFileFilters = { "*.srp", "*.srd", "*.sr" };
	return profile;
}

std::shared_ptr<RobotFileTransferSession> STEPRobotCtrl::CreateFileTransferSession(
	std::string* error) const
{
	if (m_sFTPIP.empty() || m_nFTPPort <= 0 || m_nFTPPort > 65535)
	{
		if (error != nullptr)
		{
			*error = "STEP机器人FTP参数不完整。";
		}
		return {};
	}
	if (error != nullptr) { error->clear(); }
	return std::make_shared<RobotFtpFileTransfer>(
		m_sFTPIP,
		m_nFTPPort,
		m_sFTPUser,
		m_sFTPPassWord,
		FileTransferProfile(),
		std::vector<std::string>{ ".srp", ".srd", ".sr" },
		std::vector<std::string>{ ".srp" },
		"Log/StepRobotFtp.log");
}

bool STEPRobotCtrl::ValidateLinearSpeedMmPerMin(double speedMmPerMin, std::string* error) const
{
	if (!std::isfinite(speedMmPerMin) || speedMmPerMin <= 0.0)
	{
		if (error != nullptr)
		{
			*error = GetStr("STEP线速度无效：%.6f mm/min", speedMmPerMin);
		}
		return false;
	}
	if (error != nullptr)
	{
		error->clear();
	}
	return true;
}

bool STEPRobotCtrl::MoveLinearMmPerMin(
	const T_ROBOT_COORS& target,
	double speedMmPerMin,
	int externalAxleType,
	const int* configuration)
{
	std::string error;
	if (!ValidateLinearSpeedMmPerMin(speedMmPerMin, &error))
	{
		SetLastRobotError(error);
		return false;
	}
	int nativeConfiguration[7] = {};
	if (configuration != nullptr)
	{
		std::copy(configuration, configuration + 7, nativeConfiguration);
	}
	return MoveByJob(
		target,
		T_ROBOT_MOVE_SPEED(speedMmPerMin, 0.0, 0.0),
		externalAxleType,
		"MOVL",
		1,
		nativeConfiguration);
}

bool STEPRobotCtrl::MoveJointPercent(
	const T_ANGLE_PULSE& target,
	double speedPercent,
	int externalAxleType)
{
	if (!std::isfinite(speedPercent) || speedPercent <= 0.0 || speedPercent > 100.0)
	{
		SetLastRobotError(GetStr("STEP关节速度百分比无效：%.6f", speedPercent));
		return false;
	}
	return MoveByJob(
		target,
		T_ROBOT_MOVE_SPEED(speedPercent, 0.0, 0.0),
		externalAxleType,
		"MOVJ");
}

namespace
{
	RobotMotionStatus StepNormalizedMotionStatus(int rawCode, const std::string& detail)
	{
		RobotMotionStatus status;
		status.rawCode = rawCode;
		status.detail = detail;
		switch (rawCode)
		{
		case STEPROBOTSDK::eRun:
			status.state = RobotMotionState::Running;
			break;
		case STEPROBOTSDK::ePause:
			status.state = RobotMotionState::Paused;
			break;
		case STEPROBOTSDK::eStop:
			status.state = RobotMotionState::Completed;
			break;
		default:
			status.state = RobotMotionState::Unknown;
			break;
		}
		return status;
	}
}

RobotMotionStatus STEPRobotCtrl::ReadMotionStatus()
{
	return StepNormalizedMotionStatus(CheckDone(), GetRobotStatusText());
}

RobotMotionStatus STEPRobotCtrl::ReadMotionStatusPassive(long long* pRobotMs, long long* pPcRecvMs)
{
	return StepNormalizedMotionStatus(
		CheckDonePassive(pRobotMs, pPcRecvMs),
		GetStateMonitorSourceText());
}

bool STEPRobotCtrl::ReserveTrajectory(
	RobotTrajectoryPurpose purpose,
	RobotTrajectoryHandle& handle)
{
	(void)purpose;
	handle = RobotTrajectoryHandle{};
	handle.programName = MakeTimestampWeldProgramName();
	return !handle.programName.empty();
}

bool STEPRobotCtrl::DownlinkTrajectory(
	const std::vector<T_ROBOT_MOVE_INFO>& moveInfos,
	RobotTrajectoryPurpose purpose,
	RobotTrajectoryHandle& handle)
{
	if (moveInfos.empty())
	{
		SetLastRobotError("STEP轨迹下发失败：轨迹为空。");
		return false;
	}
	if (handle.programName.empty() && !ReserveTrajectory(purpose, handle))
	{
		SetLastRobotError("STEP轨迹身份预留失败。");
		return false;
	}
	const std::filesystem::path localDirectory = StepInstanceOutputDirectory(this);
	std::string localProgramFile;
	std::string localDataFile;
	std::string generateError;
	if (!WriteContiMoveAnyFiles(
		moveInfos,
		StepLocalPathBytes(localDirectory),
		handle.programName,
		m_tAxisUnit,
		&localProgramFile,
		&localDataFile,
		&generateError,
		purpose == RobotTrajectoryPurpose::ActualWeld))
	{
		SetLastRobotError("STEP轨迹文件生成失败：" + generateError);
		return false;
	}
	const std::string projectName = StepNormalizeProjectName(kStepDynamicJobProjectName);
	const std::string remoteBase = StepBuildRemoteProjectDir(projectName) + "/";
	const std::string remoteProgram = remoteBase + handle.programName + ".srp";
	const std::string remoteData = remoteBase + handle.programName + ".srd";
	RobotRecoverySafetyPolicy::ProgramContentIdentity programIdentity;
	RobotRecoverySafetyPolicy::ProgramContentIdentity dataIdentity;
	std::string identityError;
	if (!StepReadProgramContentIdentity(
			std::filesystem::path(localProgramFile), programIdentity, identityError)
		|| !StepReadProgramContentIdentity(
			std::filesystem::path(localDataFile), dataIdentity, identityError))
	{
		SetLastRobotError("STEP轨迹下发失败：本地SRP/SRD内容身份计算失败，" + identityError);
		return false;
	}
	if (UploadFile(localProgramFile, remoteProgram) != 0
		|| UploadFile(localDataFile, remoteData) != 0)
	{
		SetLastRobotError("STEP轨迹文件上传失败：" + handle.programName);
		return false;
	}
	handle.localProgramPath = localProgramFile;
	handle.localDataPath = localDataFile;
	handle.remoteProgramPath = remoteProgram;
	handle.remoteDataPath = remoteData;
	handle.programContentSha256 = programIdentity.sha256.toStdString();
	handle.dataContentSha256 = dataIdentity.sha256.toStdString();
	handle.programContentSize = static_cast<std::uint64_t>(programIdentity.size);
	handle.dataContentSize = static_cast<std::uint64_t>(dataIdentity.size);
	handle.prepared = true;
	return true;
}

bool STEPRobotCtrl::ExportTrajectoryProgramFiles(
	const std::vector<T_ROBOT_MOVE_INFO>& moveInfos,
	RobotTrajectoryPurpose purpose,
	const std::string& outputDirectory,
	RobotTrajectoryHandle& handle,
	std::string* error)
{
	if (handle.programName.empty() && !ReserveTrajectory(purpose, handle))
	{
		const std::string detail = "STEP离线轨迹程序身份预留失败。";
		SetLastRobotError(detail);
		if (error != nullptr) { *error = detail; }
		return false;
	}
	std::string programPath;
	std::string dataPath;
	std::string writeError;
	if (!WriteContiMoveAnyFiles(
		moveInfos,
		outputDirectory,
		handle.programName,
		m_tAxisUnit,
		&programPath,
		&dataPath,
		&writeError,
		purpose == RobotTrajectoryPurpose::ActualWeld))
	{
		SetLastRobotError("STEP离线轨迹程序导出失败：" + writeError);
		if (error != nullptr) { *error = writeError; }
		return false;
	}
	handle.localProgramPath = programPath;
	handle.localDataPath = dataPath;
	handle.prepared = true;
	if (error != nullptr) { error->clear(); }
	return true;
}

bool STEPRobotCtrl::StartTrajectory(
	const std::vector<T_ROBOT_MOVE_INFO>& moveInfos,
	RobotTrajectoryPurpose purpose,
	RobotTrajectoryHandle& handle)
{
	(void)purpose;
	if (!handle.prepared || handle.programName.empty()
		|| handle.localProgramPath.empty() || handle.localDataPath.empty()
		|| handle.remoteProgramPath.empty() || handle.remoteDataPath.empty()
		|| handle.programContentSha256.size() != 64 || handle.dataContentSha256.size() != 64)
	{
		SetLastRobotError("STEP轨迹尚未通过DownlinkTrajectory完成生成和上传，禁止启动。");
		return false;
	}
	const int ret = ContiMoveAnyWithProgramName(moveInfos, handle.programName, &handle);
	if (ret != 0)
	{
		return false;
	}
	handle.started = true;
	return true;
}

bool STEPRobotCtrl::WaitTrajectory(
	const RobotTrajectoryHandle& handle,
	int pollDelayMs,
	int runTimeoutMs,
	RobotMotionStatus* terminalStatus)
{
	if (!handle.started)
	{
		SetLastRobotError("STEP轨迹尚未启动，禁止等待完成。");
		return false;
	}
	const int ret = CheckRobotDone(pollDelayMs, runTimeoutMs);
	if (terminalStatus != nullptr)
	{
		*terminalStatus = StepNormalizedMotionStatus(
			ret > 0 ? STEPROBOTSDK::eStop : ret,
			GetRobotStatusText());
		terminalStatus->terminalVerified = ret > 0;
	}
	return ret > 0;
}

bool STEPRobotCtrl::PauseTrackedMotion(
	const std::string& expectedProgramName,
	int& programLine,
	T_ROBOT_COORS& pausedPose,
	std::string* projectName,
	std::string* programName)
{
	return PauseTrackedProgramAndWait(
		expectedProgramName,
		programLine,
		pausedPose,
		projectName,
		programName);
}

bool STEPRobotCtrl::ResumeTrackedMotion(
	const std::string& expectedProgramName,
	const T_ROBOT_COORS& checkpointPose,
	double maxPositionDeviationMm,
	double maxAngleDeviationDeg,
	double* positionDeviationMm,
	double* angleDeviationDeg)
{
	return ResumeTrackedProgramFromPause(
		expectedProgramName,
		checkpointPose,
		maxPositionDeviationMm,
		maxAngleDeviationDeg,
		positionDeviationMm,
		angleDeviationDeg);
}

RobotPersistentRecoveryStrategy STEPRobotCtrl::PersistentRecoveryStrategy() const
{
	return RobotPersistentRecoveryStrategy::ExactProgramIdentity;
}

bool STEPRobotCtrl::AbortPersistedMotion(const std::string& expectedProgramName)
{
	return AbortPersistedProgramForRecovery(expectedProgramName);
}

bool STEPRobotCtrl::SetOperationMode(RobotOperationMode mode)
{
	return SetSysMode(static_cast<int>(mode));
}

bool STEPRobotCtrl::InitializeAfterConnect(std::string* summary)
{
	const bool alarmOk = cleanAlarm();
	const bool modeOk = SetOperationMode(RobotOperationMode::Automatic);
	const bool servoOk = ServoOn();
	if (summary != nullptr)
	{
		*summary = GetStr("STEP清报警=%d 自动模式=%d 上使能=%d",
			alarmOk ? 1 : 0,
			modeOk ? 1 : 0,
			servoOk ? 1 : 0);
	}
	return alarmOk && modeOk && servoOk;
}

bool STEPRobotCtrl::ShutdownBeforeDisconnect()
{
	return true;
}

void STEPRobotCtrl::ReloadRuntimeConfiguration()
{
	InvalidateStepSdkInterfaceModeCache();
}

bool STEPRobotCtrl::PrepareNativeProgramUpload()
{
	std::string error;
	const std::shared_ptr<RobotFileTransferSession> session = CreateFileTransferSession(&error);
	if (session == nullptr)
	{
		SetLastRobotError("STEP原生程序上传准备失败：" + error);
		return false;
	}
	ClearLastRobotError();
	return true;
}

bool STEPRobotCtrl::StartContinuousJog(int moveType, double nativeSpeed)
{
	(void)moveType;
	(void)nativeSpeed;
	SetLastRobotError("STEP驱动未实现连续点动队列。");
	return false;
}

bool STEPRobotCtrl::PushContinuousJogPoint(const T_ROBOT_COORS& target, double nativeSpeed)
{
	(void)target;
	(void)nativeSpeed;
	SetLastRobotError("STEP驱动未实现连续点动队列。");
	return false;
}

bool STEPRobotCtrl::PushContinuousJogPoint(const T_ANGLE_PULSE& target, double nativeSpeed)
{
	(void)target;
	(void)nativeSpeed;
	SetLastRobotError("STEP驱动未实现连续点动队列。");
	return false;
}

void STEPRobotCtrl::RequestEndContinuousJog()
{
}

void STEPRobotCtrl::EndContinuousJog()
{
}

bool STEPRobotCtrl::IsContinuousJogRunning() const
{
	return false;
}

int STEPRobotCtrl::UploadNativeProgramSource(
	const std::string& localPath,
	const std::string& remoteDirectory)
{
	const std::filesystem::path localFile(localPath);
	std::error_code fileError;
	if (localPath.empty() || !std::filesystem::is_regular_file(localFile, fileError))
	{
		SetLastRobotError("STEP原生程序上传失败：本地文件不存在或不可读：" + localPath);
		return -1;
	}
	std::string extension = localFile.extension().string();
	std::transform(extension.begin(), extension.end(), extension.begin(), [](unsigned char ch) {
		return static_cast<char>(std::tolower(ch));
	});
	if (extension != ".srp" && extension != ".srd" && extension != ".sr")
	{
		SetLastRobotError("STEP原生程序上传仅支持SRP、SRD或SR文件：" + localPath);
		return -1;
	}

	const std::string defaultProject = m_sStepProjectName.empty()
		? StepNormalizeProjectName(kStepDynamicJobProjectName)
		: StepNormalizeProjectName(m_sStepProjectName);
	std::string resolvedDirectory;
	std::string directoryError;
	if (!StepResolveNativeRemoteDirectory(
		remoteDirectory, defaultProject, resolvedDirectory, directoryError))
	{
		SetLastRobotError("STEP原生程序上传失败：" + directoryError);
		return -1;
	}

	std::string sessionError;
	const std::shared_ptr<RobotFileTransferSession> session =
		CreateFileTransferSession(&sessionError);
	if (session == nullptr)
	{
		SetLastRobotError("STEP原生程序上传失败：" + sessionError);
		return -1;
	}
	const std::string remotePath = resolvedDirectory + "/" + localFile.filename().string();
	if (!session->UploadProgramFile(localPath, remotePath, true))
	{
		SetLastRobotError("STEP原生程序上传失败：" + session->LastError()
			+ "，Remote=" + remotePath);
		return -1;
	}
	if (m_pRobotLog != nullptr)
	{
		m_pRobotLog->write(LogColor::SUCCESS,
			"STEP原生程序已通过FTP上传：Local=%s Remote=%s",
			localPath.c_str(), remotePath.c_str());
	}
	ClearLastRobotError();
	return 0;
}

std::string STEPRobotCtrl::SendDiagnosticCommand(const std::string& command)
{
	const std::string trimmedCommand = StepTrim(command);
	if (trimmedCommand.empty())
	{
		SetLastRobotError("STEP诊断命令为空。");
		return {};
	}
	if (m_pSTEPRobotClient == nullptr || (!m_bLocalDebugMark && !IsConnected()))
	{
		SetLastRobotError("STEP诊断命令失败：机器人未连接。");
		return {};
	}

	std::string normalizedCommand = trimmedCommand;
	std::transform(normalizedCommand.begin(), normalizedCommand.end(), normalizedCommand.begin(),
		[](unsigned char ch) { return static_cast<char>(std::toupper(ch)); });
	std::ostringstream response;
	if (normalizedCommand == "GET_INFO" || normalizedCommand == "INFO")
	{
		std::lock_guard<std::recursive_mutex> lock(m_sdkCommandMutex);
		response << "INFO=" << m_pSTEPRobotClient->getInfo()
			<< ";SDK=" << m_pSTEPRobotClient->SDKVersion()
			<< ";CONNECT=" << m_pSTEPRobotClient->ConnectStatus();
	}
	else if (normalizedCommand == "GET_STATUS" || normalizedCommand == "STATUS")
	{
		std::lock_guard<std::recursive_mutex> lock(m_sdkCommandMutex);
		const STEPROBOTSDK::MessageData message = m_pSTEPRobotClient->getMessageData();
		response << "PROJECT=" << StepNormalizeProjectName(m_pSTEPRobotClient->getProjectName())
			<< ";PROGRAM=" << m_pSTEPRobotClient->getProgramName()
			<< ";STATE=" << StepProgramStateText(m_pSTEPRobotClient->getProgramState())
			<< "(" << static_cast<int>(m_pSTEPRobotClient->getProgramState()) << ")"
			<< ";MODE=" << static_cast<int>(m_pSTEPRobotClient->getProgramMode())
			<< ";LINE=" << m_pSTEPRobotClient->getCurrentLine()
			<< ";MOTOR=" << m_pSTEPRobotClient->getMotorEnableState()
			<< ";OVERRIDE=" << m_pSTEPRobotClient->getOverride()
			<< ";MESSAGE_TYPE=" << StepMessageTypeText(message.m_MessageType)
			<< ";MESSAGE_ID=" << message.m_MessageID
			<< ";MESSAGE=" << StepMessageText(message);
	}
	else if (normalizedCommand == "GET_USER_PROGRAM")
	{
		std::lock_guard<std::recursive_mutex> lock(m_sdkCommandMutex);
		response << "PROJECT=" << StepNormalizeProjectName(m_pSTEPRobotClient->getProjectName())
			<< ";PROGRAM=" << m_pSTEPRobotClient->getProgramName();
	}
	else if (normalizedCommand == "GET_CUR_POS")
	{
		T_ROBOT_COORS pose;
		if (!TryGetCurrentPos(pose))
		{
			return {};
		}
		response << std::fixed << std::setprecision(6)
			<< "POS:" << pose.dX << ',' << pose.dY << ',' << pose.dZ << ','
			<< pose.dRX << ',' << pose.dRY << ',' << pose.dRZ << ','
			<< pose.dBX << ',' << pose.dBY << ',' << pose.dBZ;
	}
	else if (normalizedCommand == "GET_CUR_PULSE")
	{
		T_ANGLE_PULSE pulse;
		if (!TryGetCurrentPulse(pulse))
		{
			return {};
		}
		response << "PULSE:" << pulse.nSPulse << ',' << pulse.nLPulse << ','
			<< pulse.nUPulse << ',' << pulse.nRPulse << ',' << pulse.nBPulse << ','
			<< pulse.nTPulse << ',' << pulse.lBXPulse << ',' << pulse.lBYPulse << ','
			<< pulse.lBZPulse;
	}
	else if (normalizedCommand == "CHECK_DONE")
	{
		const int state = CheckDone();
		response << "STATE=" << StepProgramStateText(state) << "(" << state << ")";
	}
	else if (normalizedCommand == "GET_MESSAGE")
	{
		const STEPROBOTSDK::MessageData message = WithSdkCommand(
			[&]() { return m_pSTEPRobotClient->getMessageData(); });
		response << "TYPE=" << StepMessageTypeText(message.m_MessageType)
			<< ";ID=" << message.m_MessageID
			<< ";SOURCE=" << StepCString(message.m_MessageSource, sizeof(message.m_MessageSource))
			<< ";MESSAGE=" << StepMessageText(message);
	}
	else
	{
		int testRet = 0;
		STEPROBOTSDK::MessageData message = {};
		int state = -1;
		{
			std::lock_guard<std::recursive_mutex> lock(m_sdkCommandMutex);
			testRet = m_pSTEPRobotClient->test(trimmedCommand.c_str());
			message = m_pSTEPRobotClient->getMessageData();
			state = static_cast<int>(m_pSTEPRobotClient->getProgramState());
		}
		response << "TEST_RET=" << testRet
			<< ";STATE=" << StepProgramStateText(state) << "(" << state << ")"
			<< ";MESSAGE_TYPE=" << StepMessageTypeText(message.m_MessageType)
			<< ";MESSAGE_ID=" << message.m_MessageID
			<< ";MESSAGE=" << StepMessageText(message);
		if (testRet != 0 || message.m_MessageType == STEPROBOTSDK::eError)
		{
			SetLastRobotError("STEP原生诊断命令返回错误：" + response.str());
			return response.str();
		}
	}
	ClearLastRobotError();
	return response.str();
}

bool STEPRobotCtrl::WriteCartesianRegister(int index, const double pose[8], int config[7])
{
	double writablePose[8] = {};
	std::copy(pose, pose + 8, writablePose);
	return SetPosVar(index, writablePose, POSVAR, 1, config, ENGINEEVAR, POSVAR);
}

bool STEPRobotCtrl::RunProgramAndWait(
	const std::string& programName,
	int startTimeoutMs,
	int finishTimeoutMs,
	int pollDelayMs,
	RobotMotionStatus* terminalStatus)
{
	if (terminalStatus != nullptr)
	{
		*terminalStatus = RobotMotionStatus{};
		terminalStatus->state = RobotMotionState::Faulted;
	}
	if (startTimeoutMs <= 0 || finishTimeoutMs <= 0)
	{
		SetLastRobotError("STEP原生程序执行超时参数必须为有限正值。");
		if (terminalStatus != nullptr) { terminalStatus->detail = GetLastRobotError(); }
		return false;
	}
	pollDelayMs = std::clamp(pollDelayMs, 20, 1000);
	if (m_pSTEPRobotClient == nullptr || (!m_bLocalDebugMark && !IsConnected()))
	{
		SetLastRobotError("STEP原生程序执行失败：机器人未连接。");
		if (terminalStatus != nullptr) { terminalStatus->detail = GetLastRobotError(); }
		return false;
	}
	if (RobotOperationLease::MotionCompletionPending(this))
	{
		SetLastRobotError("STEP原生程序执行失败：上一运动尚未获得稳定终态见证。");
		if (terminalStatus != nullptr) { terminalStatus->detail = GetLastRobotError(); }
		return false;
	}

	const std::string defaultProject = m_sStepProjectName.empty()
		? StepNormalizeProjectName(kStepDynamicJobProjectName)
		: StepNormalizeProjectName(m_sStepProjectName);
	std::string requestedProject;
	std::string requestedProgram;
	std::string identityError;
	if (!StepResolveNativeProgramIdentity(
		programName, defaultProject, requestedProject, requestedProgram, identityError))
	{
		SetLastRobotError("STEP原生程序身份无效：" + identityError);
		if (terminalStatus != nullptr) { terminalStatus->detail = GetLastRobotError(); }
		return false;
	}

	struct NativeProgramSnapshot
	{
		int connectStatus = -1;
		std::string project;
		std::string program;
		int state = -1;
		int line = -1;
		STEPROBOTSDK::MessageData message = {};
	};
	const auto readSnapshot = [this]()
		{
			NativeProgramSnapshot snapshot;
			std::lock_guard<std::recursive_mutex> lock(m_sdkCommandMutex);
			if (m_pSTEPRobotClient == nullptr)
			{
				return snapshot;
			}
			snapshot.connectStatus = m_bLocalDebugMark
				? 0 : m_pSTEPRobotClient->ConnectStatus();
			snapshot.project = StepNormalizeProjectName(m_pSTEPRobotClient->getProjectName());
			snapshot.program = m_pSTEPRobotClient->getProgramName();
			snapshot.state = static_cast<int>(m_pSTEPRobotClient->getProgramState());
			snapshot.line = m_pSTEPRobotClient->getCurrentLine();
			snapshot.message = m_pSTEPRobotClient->getMessageData();
			return snapshot;
		};
	const auto sameMessage = [](const STEPROBOTSDK::MessageData& left,
		const STEPROBOTSDK::MessageData& right)
		{
			return left.m_MessageType == right.m_MessageType
				&& left.m_MessageID == right.m_MessageID
				&& left.m_MessageTime == right.m_MessageTime
				&& StepMessageText(left) == StepMessageText(right)
				&& StepCString(left.m_MessageSource, sizeof(left.m_MessageSource))
					== StepCString(right.m_MessageSource, sizeof(right.m_MessageSource));
		};
	bool motionMarked = false;
	const auto failRun = [this, terminalStatus, &motionMarked](
		const std::string& reason,
		int rawState,
		RobotMotionState state = RobotMotionState::Faulted)
		{
			std::string detail = reason;
			if (motionMarked)
			{
				const bool stopped = RobotOperationLease::StopAndConfirmUnverifiedMotion(this);
				const std::string stopDetail = GetLastRobotError();
				detail += stopped
					? "；已STOP/Kill并稳定确认程序不可恢复。"
					: "；STOP/Kill未确认：" + stopDetail;
			}
			SetLastRobotError(detail);
			if (terminalStatus != nullptr)
			{
				terminalStatus->state = state;
				terminalStatus->rawCode = rawState;
				terminalStatus->terminalVerified = false;
				terminalStatus->detail = detail;
			}
			return false;
		};

	const NativeProgramSnapshot beforeLoad = readSnapshot();
	if ((!m_bLocalDebugMark && beforeLoad.connectStatus < 0)
		|| beforeLoad.state == STEPROBOTSDK::eRun
		|| beforeLoad.state == STEPROBOTSDK::ePause)
	{
		return failRun(GetStr(
			"STEP原生程序执行已拒绝：控制器当前程序仍在运行或暂停。Current=%s/%s State=%d",
			beforeLoad.project.c_str(), beforeLoad.program.c_str(), beforeLoad.state),
			beforeLoad.state);
	}
	if (RobotOperationLease::IsCancellationRequested(this))
	{
		return failRun("STEP原生程序执行已由安全停止取消，未加载程序。",
			beforeLoad.state, RobotMotionState::Interrupted);
	}
	{
		std::lock_guard<std::recursive_mutex> lock(m_sdkCommandMutex);
		m_motionTrackedProjectName.clear();
		m_motionTrackedProgramName.clear();
		// 任意原生程序由控制器的加载返回、精确程序身份、运行进度和稳定eStop闭环；
		// 自动生成焊接轨迹仍继续使用SRP/SRD内容哈希与ntdone自然完成见证。
		ClearGeneratedProgramCompletionWitnessLocked();
	}
	if (!LoadUserProgram(requestedProject, requestedProgram, true))
	{
		return failRun("STEP原生程序加载失败：" + GetLastRobotError(), -1);
	}

	const NativeProgramSnapshot loaded = readSnapshot();
	if (loaded.project != requestedProject || loaded.program != requestedProgram
		|| loaded.state != STEPROBOTSDK::eStop)
	{
		return failRun(GetStr(
			"STEP原生程序加载后身份或初始状态不一致。Expected=%s/%s Current=%s/%s State=%d",
			requestedProject.c_str(), requestedProgram.c_str(),
			loaded.project.c_str(), loaded.program.c_str(), loaded.state), loaded.state);
	}
	if (RobotOperationLease::IsCancellationRequested(this))
	{
		return failRun("STEP原生程序已加载但在START前被安全停止取消。",
			loaded.state, RobotMotionState::Interrupted);
	}

	int startRet = -1;
	{
		std::lock_guard<std::recursive_mutex> lock(m_sdkCommandMutex);
		if (RobotOperationLease::IsCancellationRequested(this)
			|| StepNormalizeProjectName(m_pSTEPRobotClient->getProjectName()) != requestedProject
			|| m_pSTEPRobotClient->getProgramName() != requestedProgram
			|| m_pSTEPRobotClient->getProgramState() != STEPROBOTSDK::eStop)
		{
			return failRun("STEP原生程序START前身份或状态发生变化。", -1);
		}
		QString motionError;
		if (!RobotOperationLease::MarkMotionStarted(this, false, &motionError))
		{
			return failRun("STEP原生程序START登记失败：" + motionError.toStdString(), -1);
		}
		motionMarked = true;
		m_motionTrackedProjectName = requestedProject;
		m_motionTrackedProgramName = requestedProgram;
		startRet = m_pSTEPRobotClient->SetModeCmd(MODEKEY::START, true);
	}
	if (startRet != 0)
	{
		return failRun(GetStr(
			"STEP原生程序START失败：Project=%s Program=%s Ret=%d Message=%s",
			requestedProject.c_str(), requestedProgram.c_str(), startRet,
			StepMessageText(readSnapshot().message).c_str()), startRet);
	}

	const auto controllerErrorAfter = [&sameMessage, &loaded](const NativeProgramSnapshot& snapshot)
		{
			return snapshot.message.m_MessageType == STEPROBOTSDK::eError
				&& !sameMessage(snapshot.message, loaded.message);
		};
	const auto validateSnapshot = [this, &requestedProject, &requestedProgram,
		&controllerErrorAfter, &failRun](const NativeProgramSnapshot& snapshot)
		{
			if (!m_bLocalDebugMark && snapshot.connectStatus < 0)
			{
				return failRun("STEP原生程序执行期间连接断开。", snapshot.state);
			}
			if (snapshot.project != requestedProject || snapshot.program != requestedProgram)
			{
				return failRun(GetStr(
					"STEP原生程序执行期间身份变化。Expected=%s/%s Current=%s/%s",
					requestedProject.c_str(), requestedProgram.c_str(),
					snapshot.project.c_str(), snapshot.program.c_str()), snapshot.state);
			}
			if (controllerErrorAfter(snapshot))
			{
				return failRun(GetStr(
					"STEP控制器报告程序错误：Type=%s ID=%d Message=%s",
					StepMessageTypeText(snapshot.message.m_MessageType),
					snapshot.message.m_MessageID,
					StepMessageText(snapshot.message).c_str()), snapshot.state);
			}
			return true;
		};
	const auto completeRun = [this, terminalStatus, &motionMarked,
		&requestedProject, &requestedProgram, &failRun](const NativeProgramSnapshot& snapshot)
		{
			if (!RobotOperationLease::MarkMotionCompleted(this))
			{
				return failRun("STEP原生程序已稳定eStop，但适配层未能解除运动完成待确认状态。",
					snapshot.state);
			}
			motionMarked = false;
			{
				std::lock_guard<std::recursive_mutex> lock(m_sdkCommandMutex);
				m_motionTrackedProjectName.clear();
				m_motionTrackedProgramName.clear();
				ClearGeneratedProgramCompletionWitnessLocked();
			}
			const std::string detail = GetStr(
				"STEP原生程序自然完成：Project=%s Program=%s Line=%d，已稳定回读eStop。",
				requestedProject.c_str(), requestedProgram.c_str(), snapshot.line);
			if (m_pRobotLog != nullptr)
			{
				m_pRobotLog->write(LogColor::SUCCESS, "%s", detail.c_str());
			}
			ClearLastRobotError();
			if (terminalStatus != nullptr)
			{
				terminalStatus->state = RobotMotionState::Completed;
				terminalStatus->rawCode = STEPROBOTSDK::eStop;
				terminalStatus->terminalVerified = true;
				terminalStatus->detail = detail;
			}
			return true;
		};

	bool executionObserved = false;
	bool lineProgressObserved = false;
	int stableStoppedReads = 0;
	const auto startDeadline = std::chrono::steady_clock::now()
		+ std::chrono::milliseconds(startTimeoutMs);
	while (std::chrono::steady_clock::now() < startDeadline)
	{
		if (RobotOperationLease::IsCancellationRequested(this))
		{
			return failRun("STEP原生程序启动等待已由安全停止取消。",
				-1, RobotMotionState::Interrupted);
		}
		const NativeProgramSnapshot snapshot = readSnapshot();
		if (!validateSnapshot(snapshot))
		{
			return false;
		}
		lineProgressObserved = lineProgressObserved || snapshot.line != loaded.line;
		if (snapshot.state == STEPROBOTSDK::eRun
			|| snapshot.state == STEPROBOTSDK::ePause)
		{
			executionObserved = true;
			break;
		}
		if (snapshot.state == STEPROBOTSDK::eStop && lineProgressObserved)
		{
			if (++stableStoppedReads >= 3)
			{
				return completeRun(snapshot);
			}
		}
		else if (snapshot.state != STEPROBOTSDK::eStop)
		{
			return failRun(GetStr("STEP原生程序启动后返回未知状态：%d。", snapshot.state),
				snapshot.state);
		}
		Sleep(pollDelayMs);
	}
	if (!executionObserved)
	{
		return failRun(GetStr(
			"STEP原生程序在%dms内未观察到eRun/ePause或行号进展后的稳定eStop。",
			startTimeoutMs), STEPROBOTSDK::eStop);
	}

	stableStoppedReads = 0;
	const auto finishDeadline = std::chrono::steady_clock::now()
		+ std::chrono::milliseconds(finishTimeoutMs);
	while (std::chrono::steady_clock::now() < finishDeadline)
	{
		if (RobotOperationLease::IsCancellationRequested(this))
		{
			return failRun("STEP原生程序完成等待已由安全停止取消。",
				-1, RobotMotionState::Interrupted);
		}
		const NativeProgramSnapshot snapshot = readSnapshot();
		if (!validateSnapshot(snapshot))
		{
			return false;
		}
		if (snapshot.state == STEPROBOTSDK::eStop)
		{
			if (++stableStoppedReads >= 3)
			{
				return completeRun(snapshot);
			}
		}
		else if (snapshot.state == STEPROBOTSDK::eRun
			|| snapshot.state == STEPROBOTSDK::ePause)
		{
			stableStoppedReads = 0;
		}
		else
		{
			return failRun(GetStr("STEP原生程序运行中返回未知状态：%d。", snapshot.state),
				snapshot.state);
		}
		Sleep(pollDelayMs);
	}
	return failRun(GetStr("STEP原生程序在%dms内未完成。", finishTimeoutMs),
		-1, RobotMotionState::Interrupted);
}

bool STEPRobotCtrl::InstallHandEyeSupportPrograms(std::string* summary)
{
	QString programPath = AppPaths::WritablePath("Job/STEP/handeyetest.srp");
	QString dataPath = AppPaths::WritablePath("Job/STEP/handeyetest.srd");
	if (!QFileInfo::exists(programPath))
	{
		programPath = AppPaths::FindResourcePath("Job/STEP/handeyetest.srp");
	}
	if (!QFileInfo::exists(dataPath))
	{
		dataPath = AppPaths::FindResourcePath("Job/STEP/handeyetest.srd");
	}
	if (programPath.isEmpty() || dataPath.isEmpty())
	{
		SetLastRobotError("STEP手眼辅助程序资源不完整，缺少handeyetest.srp或handeyetest.srd。");
		return false;
	}
	const QByteArray programBytes = programPath.toLocal8Bit();
	const QByteArray dataBytes = dataPath.toLocal8Bit();
	if (UploadNativeProgramSource(programBytes.constData()) != 0)
	{
		SetLastRobotError("STEP手眼辅助SRP上传失败：" + GetLastRobotError());
		return false;
	}
	if (UploadNativeProgramSource(dataBytes.constData()) != 0)
	{
		SetLastRobotError("STEP手眼辅助SRD上传失败：" + GetLastRobotError());
		return false;
	}
	const std::string projectName = m_sStepProjectName.empty()
		? StepNormalizeProjectName(kStepDynamicJobProjectName)
		: StepNormalizeProjectName(m_sStepProjectName);
	if (summary != nullptr)
	{
		*summary = "STEP handeyetest.srp/handeyetest.srd已上传到"
			+ StepBuildRemoteProjectDir(projectName)
			+ "，可在示教器加载使用。";
	}
	ClearLastRobotError();
	return true;
}

bool STEPRobotCtrl::RunHandEyeValidation(
	const T_ROBOT_COORS& robotPose,
	T_ROBOT_COORS& robotCalculatedPoint)
{
	(void)robotPose;
	(void)robotCalculatedPoint;
	SetLastRobotError(
		"STEP手眼辅助运动程序已支持安装，但当前辅助程序不输出机器人侧矩阵计算点；"
		"本地手眼矩阵检测仍可使用，机器人侧结果对比保持关闭。");
	return false;
}

bool STEPRobotCtrl::InitRobotDriver(std::string strUnitName)
{
	ConfigSection cIni;
	if (const RobotDriverSetupProfile* setup =
		RobotDriverRegistry::SetupProfile(ROBOT_TYPE_STEP))
	{
		m_nSocketPort = setup->defaultSocketPort;
		m_sFTPIP = setup->defaultFtpHost;
		m_nFTPPort = setup->defaultFtpPort;
		m_sFTPUser = setup->defaultFtpUser;
		m_sFTPPassWord = setup->defaultFtpPassword;
	}

	cIni.SetLocation(ConfigLocation::Robot(QString::fromUtf8(strUnitName.c_str()), QStringLiteral("RobotPara")));
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
	if (m_sStepProjectName.empty())
	{
		if (const RobotDriverSetupProfile* setup =
			RobotDriverRegistry::SetupProfile(ROBOT_TYPE_STEP))
		{
			m_sStepProjectName = setup->defaultControllerProject;
		}
	}
	m_sStepProjectName = StepNormalizeProjectName(m_sStepProjectName);
	cIni.ReadString("FTPIP", m_sFTPIP);
	cIni.ReadString("FTPPort", &m_nFTPPort);
	cIni.ReadString("FTPUser", m_sFTPUser);
	cIni.ReadString("FTPPassWord", m_sFTPPassWord);
	if (m_sFTPIP.empty())
	{
		m_sFTPIP = m_sSocketIP;
	}

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
	// Copy before SDK use: callers commonly pass m_sSocketIP.c_str(), and assigning
	// that aliased pointer back into the same std::string is not a safe update path.
	const std::string connectedIp = ip != nullptr ? std::string(ip) : std::string();
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
	if (!connectedIp.empty())
	{
		m_sSocketIP = connectedIp;
	}
	if (Port > 0)
	{
		m_nSocketPort = static_cast<int>(Port);
	}
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
	ClearLastRobotError();
	int nRet = 0;
	const bool wasConnected = m_bSocketConnected.load();
	if (m_pSTEPRobotClient == nullptr)
	{
		m_bSocketConnected.store(false);
		return true;
	}
	if (!m_bLocalDebugMark)
	{
		nRet = WithSdkCommand([&]() { return m_pSTEPRobotClient->close(); });
	}
	// SDK 调用结束后再次落定连接快照，避免其他线程看到“已断开”后排队重连，
	// 又被本次 close 的尾部状态覆盖。
	m_bSocketConnected.store(false);
	if (0 != nRet)
	{
		const std::string error = GetStr(
			"STEP断开失败：原因=%s(%d)", GetErrorText(nRet), nRet);
		SetLastRobotError(error);
		if (m_pRobotLog != nullptr)
		{
			m_pRobotLog->write(LogColor::ERR,
				"STEP断开失败 | ret=%d reason=%s connected_before=%d",
				nRet,
				GetErrorText(nRet),
				wasConnected ? 1 : 0);
		}
		return false;
	}
	if (m_pRobotLog != nullptr)
	{
		m_pRobotLog->write(LogColor::SUCCESS,
			"STEP连接已断开 | ret=%d connected_before=%d",
			nRet,
			wasConnected ? 1 : 0);
	}
	return true;
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
	T_ROBOT_COORS pos;
	return TryGetCurrentPos(pos) ? pos : T_ROBOT_COORS();
}

bool STEPRobotCtrl::TryGetCurrentPos(T_ROBOT_COORS& pos)
{
	pos = T_ROBOT_COORS();
	if (m_pSTEPRobotClient == nullptr
		|| (!m_bLocalDebugMark && !m_bSocketConnected.load()))
	{
		SetLastRobotError("STEP当前位置读取失败：机器人未连接。");
		return false;
	}

	const int beforeStatus = m_bLocalDebugMark
		? 0
		: WithSdkCommand([&]() { return m_pSTEPRobotClient->ConnectStatus(); });
	if (!m_bLocalDebugMark && beforeStatus < 0)
	{
		m_bSocketConnected.store(false);
		SetLastRobotError("STEP当前位置读取失败：SDK连接状态无效。");
		return false;
	}
	RobotCartPos cartposworld = WithSdkCommand([&]() { return m_pSTEPRobotClient->getCartPosWorld(); });
	const int afterStatus = m_bLocalDebugMark
		? 0
		: WithSdkCommand([&]() { return m_pSTEPRobotClient->ConnectStatus(); });
	if (!m_bLocalDebugMark && (!m_bSocketConnected.load() || afterStatus < 0))
	{
		m_bSocketConnected.store(false);
		SetLastRobotError("STEP当前位置读取期间机器人连接已断开。");
		return false;
	}
	pos = StepRobotCartPosToCoors(cartposworld);
	const bool finite = std::isfinite(pos.dX) && std::isfinite(pos.dY) && std::isfinite(pos.dZ)
		&& std::isfinite(pos.dRX) && std::isfinite(pos.dRY) && std::isfinite(pos.dRZ)
		&& std::isfinite(pos.dBX) && std::isfinite(pos.dBY) && std::isfinite(pos.dBZ);
	if (!finite)
	{
		SetLastRobotError("STEP当前位置读取失败：SDK返回非有限值。");
		pos = T_ROBOT_COORS();
		return false;
	}
	const bool allZero = std::abs(pos.dX) < 1e-12 && std::abs(pos.dY) < 1e-12
		&& std::abs(pos.dZ) < 1e-12 && std::abs(pos.dRX) < 1e-12
		&& std::abs(pos.dRY) < 1e-12 && std::abs(pos.dRZ) < 1e-12
		&& std::abs(pos.dBX) < 1e-12 && std::abs(pos.dBY) < 1e-12
		&& std::abs(pos.dBZ) < 1e-12;
	if (!m_bLocalDebugMark && allZero)
	{
		SetLastRobotError(
			"STEP当前位置读取失败：SDK返回全零哨兵值，禁止将其当作真实当前位置。");
		pos = T_ROBOT_COORS();
		return false;
	}
	return true;
}

T_ROBOT_COORS STEPRobotCtrl::GetCurrentPosPassive(long long* pRobotMs, long long* pPcRecvMs)
{
	if (m_pSTEPRobotClient == nullptr)
	{
		if (pRobotMs != nullptr) *pRobotMs = 0;
		if (pPcRecvMs != nullptr) *pPcRecvMs = 0;
		return T_ROBOT_COORS();
	}

	if (m_bLocalDebugMark || !m_bSocketConnected || !StepUseTimestampSdkInterface())
	{
		T_ROBOT_COORS pose;
		if (!TryGetCurrentPos(pose))
		{
			if (pRobotMs != nullptr) *pRobotMs = 0;
			if (pPcRecvMs != nullptr) *pPcRecvMs = 0;
			return T_ROBOT_COORS();
		}
		const long long pcMs = StepRobotSteadyNowMs();
		StepFillPcPassiveTimestamp(pRobotMs, pPcRecvMs, pcMs);
		return pose;
	}

#if STEP_SDK_HAS_TIMESTAMP
	const int beforeStatus = WithSdkCommand([&]() { return m_pSTEPRobotClient->ConnectStatus(); });
	if (beforeStatus < 0)
	{
		m_bSocketConnected.store(false);
		if (pRobotMs != nullptr) *pRobotMs = 0;
		if (pPcRecvMs != nullptr) *pPcRecvMs = 0;
		SetLastRobotError("STEP被动位姿读取失败：SDK连接状态无效。");
		return T_ROBOT_COORS();
	}
	const TimestampAddCartpos timestampedPos = WithSdkCommand([&]() { return m_pSTEPRobotClient->getTimestamp(); });
	const long long pcRecvMs = StepRobotSteadyNowMs();
	const int afterStatus = WithSdkCommand([&]() { return m_pSTEPRobotClient->ConnectStatus(); });
	if (!m_bSocketConnected.load() || afterStatus < 0)
	{
		m_bSocketConnected.store(false);
		if (pRobotMs != nullptr) *pRobotMs = 0;
		if (pPcRecvMs != nullptr) *pPcRecvMs = 0;
		SetLastRobotError("STEP被动位姿读取期间机器人连接已断开。");
		return T_ROBOT_COORS();
	}
	const unsigned long long robotTimestampMs = static_cast<unsigned long long>(timestampedPos.m_TimeStamp_ms);
	const T_ROBOT_COORS pose = StepRobotCartPosToCoors(timestampedPos.m_CartPos.m_CartPos);
	const bool finite = std::isfinite(pose.dX) && std::isfinite(pose.dY) && std::isfinite(pose.dZ)
		&& std::isfinite(pose.dRX) && std::isfinite(pose.dRY) && std::isfinite(pose.dRZ)
		&& std::isfinite(pose.dBX) && std::isfinite(pose.dBY) && std::isfinite(pose.dBZ);
	const bool allZero = std::abs(pose.dX) < 1e-12 && std::abs(pose.dY) < 1e-12
		&& std::abs(pose.dZ) < 1e-12 && std::abs(pose.dRX) < 1e-12
		&& std::abs(pose.dRY) < 1e-12 && std::abs(pose.dRZ) < 1e-12
		&& std::abs(pose.dBX) < 1e-12 && std::abs(pose.dBY) < 1e-12
		&& std::abs(pose.dBZ) < 1e-12;
	if (!finite || allZero)
	{
		if (pRobotMs != nullptr) *pRobotMs = 0;
		if (pPcRecvMs != nullptr) *pPcRecvMs = 0;
		SetLastRobotError(!finite
			? "STEP被动位姿读取失败：SDK返回非有限值。"
			: "STEP被动位姿读取失败：SDK返回全零哨兵值。");
		return T_ROBOT_COORS();
	}

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
			// 已锁定机器人时间轴后，0 不再沿用旧帧；旧位姿配旧时间戳会被误作
			// 本轮扫描的新样本，必须让状态快照标为无效并等待下一帧。
			if (pRobotMs != nullptr) *pRobotMs = 0;
			if (pPcRecvMs != nullptr) *pPcRecvMs = 0;
			SetLastRobotError("STEP被动位姿读取失败：机器人时间戳在锁定后返回0。");
			return T_ROBOT_COORS();
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
	return pose;
#else
	T_ROBOT_COORS pose;
	if (!TryGetCurrentPos(pose))
	{
		if (pRobotMs != nullptr) *pRobotMs = 0;
		if (pPcRecvMs != nullptr) *pPcRecvMs = 0;
		return T_ROBOT_COORS();
	}
	const long long pcMs = StepRobotSteadyNowMs();
	StepFillPcPassiveTimestamp(pRobotMs, pPcRecvMs, pcMs);
	return pose;
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
	T_ANGLE_PULSE pulse;
	return TryGetCurrentPulse(pulse) ? pulse : T_ANGLE_PULSE();
}

bool STEPRobotCtrl::TryGetCurrentPulse(T_ANGLE_PULSE& pulse)
{
	pulse = T_ANGLE_PULSE();
	if (m_pSTEPRobotClient == nullptr
		|| (!m_bLocalDebugMark && !m_bSocketConnected.load()))
	{
		SetLastRobotError("STEP当前关节读取失败：机器人未连接。");
		return false;
	}
	const int beforeStatus = m_bLocalDebugMark
		? 0
		: WithSdkCommand([&]() { return m_pSTEPRobotClient->ConnectStatus(); });
	if (!m_bLocalDebugMark && beforeStatus < 0)
	{
		m_bSocketConnected.store(false);
		SetLastRobotError("STEP当前关节读取失败：SDK连接状态无效。");
		return false;
	}
	const AXISPOS axisPos = WithSdkCommand([&]() { return m_pSTEPRobotClient->getAxisPos(); });
	const int afterStatus = m_bLocalDebugMark
		? 0
		: WithSdkCommand([&]() { return m_pSTEPRobotClient->ConnectStatus(); });
	if (!m_bLocalDebugMark && (!m_bSocketConnected.load() || afterStatus < 0))
	{
		m_bSocketConnected.store(false);
		SetLastRobotError("STEP当前关节读取期间机器人连接已断开。");
		return false;
	}

	long converted[9] = {};
	bool allZero = true;
	const int activeAxisCount = std::clamp(m_nRobotAxisCount, 6, 9);
	for (int axis = 0; axis < activeAxisCount; ++axis)
	{
		const double position = axis < 6
			? axisPos.m_Joint[axis]
			: axisPos.m_AuxJoint[axis - 6];
		const double unit = m_tAxisUnit.GetValueByIndex(axis);
		if (!std::isfinite(position) || !std::isfinite(unit) || std::abs(unit) < 1e-15)
		{
			SetLastRobotError(GetStr(
				"STEP当前关节读取失败：轴%d数值或脉冲当量无效。", axis + 1));
			return false;
		}
		const double scaled = position / unit;
		if (!std::isfinite(scaled)
			|| scaled < static_cast<double>(std::numeric_limits<long>::lowest())
			|| scaled > static_cast<double>(std::numeric_limits<long>::max()))
		{
			SetLastRobotError(GetStr(
				"STEP当前关节读取失败：轴%d换算结果超出脉冲范围。", axis + 1));
			return false;
		}
		converted[axis] = static_cast<long>(std::lround(scaled));
		allZero = allZero && std::abs(position) < 1e-12;
	}
	if (!m_bLocalDebugMark && allZero)
	{
		T_ROBOT_COORS cartesian;
		if (!TryGetCurrentPos(cartesian))
		{
			SetLastRobotError(
				"STEP当前关节读取失败：关节与直角接口同时返回全零哨兵值。");
			return false;
		}
	}
	pulse = T_ANGLE_PULSE(
		converted[0], converted[1], converted[2],
		converted[3], converted[4], converted[5],
		converted[6], converted[7], converted[8]);
	return true;
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

int STEPRobotCtrl::CheckRobotDone(int nDelayTime, int runTimeoutMs)
{
	if (nDelayTime <= 0)
	{
		nDelayTime = 200;
	}
	if (runTimeoutMs <= 0)
	{
		runTimeoutMs = 1800000;
	}
	const auto failUnverified = [this](const std::string& reason, int code)
		{
			if (m_pRobotLog != nullptr)
			{
				m_pRobotLog->write(LogColor::ERR, "%s", reason.c_str());
			}
			const bool stopped = RobotOperationLease::StopAndConfirmUnverifiedMotion(this);
			const std::string stopDetail = GetLastRobotError();
			SetLastRobotError(reason + (stopped
				? "；已自动中止并确认机器人程序不可恢复。"
				: "；自动中止未确认：" + stopDetail));
			return code;
		};
	int nRet = -1;
	auto lastPollTime = std::chrono::steady_clock::now();
	std::chrono::milliseconds activeRunElapsed(0);
	std::chrono::milliseconds pauseElapsed(0);
	while (1)
	{
		if (RobotOperationLease::IsCancellationRequested(this))
		{
			SetLastRobotError("STEP硬件操作已被安全停止取消，禁止把停止态判作正常完成。");
			return -20000;
		}
		if (!m_bLocalDebugMark
			&& (m_pSTEPRobotClient == nullptr
				|| !m_bSocketConnected.load()
				|| WithSdkCommand([&]() { return m_pSTEPRobotClient->ConnectStatus(); }) < 0))
		{
			return failUnverified("STEP等待运动完成失败：机器人连接已断开。", -32000);
		}
		nRet = CheckDone();
		const auto now = std::chrono::steady_clock::now();
		if (nRet == STEPROBOTSDK::eRun || nRet == STEPROBOTSDK::ePause)
		{
			bool newlyLatched = false;
			{
				std::lock_guard<std::recursive_mutex> modeLock(m_sdkCommandMutex);
				const bool wasLatched = m_completionWitnessRuntimeLatched;
				std::string ignoredWitnessError;
				VerifyGeneratedProgramCompletionWitnessLocked(ignoredWitnessError);
				newlyLatched = !wasLatched && m_completionWitnessRuntimeLatched;
			}
			if (newlyLatched && m_pRobotLog != nullptr)
			{
				m_pRobotLog->write(LogColor::SUCCESS,
					"STEP自然完成运行态令牌已锁存，等待同一程序进入eStop");
			}
		}
		if (nRet == STEPROBOTSDK::eRun)
		{
			activeRunElapsed += std::chrono::duration_cast<std::chrono::milliseconds>(now - lastPollTime);
		}
		else if (nRet == STEPROBOTSDK::ePause)
		{
			pauseElapsed += std::chrono::duration_cast<std::chrono::milliseconds>(now - lastPollTime);
		}
		lastPollTime = now;
		if (activeRunElapsed.count() >= runTimeoutMs)
		{
			std::string trackedProgram;
			{
				std::lock_guard<std::recursive_mutex> modeLock(m_sdkCommandMutex);
				trackedProgram = m_motionTrackedProgramName;
			}
			return failUnverified(GetStr(
				"STEP运动等待超时：活动运行已达%lldms，上限=%dms，Program=%s",
				static_cast<long long>(activeRunElapsed.count()), runTimeoutMs,
				trackedProgram.c_str()), -30000);
		}
		if (pauseElapsed.count() >= kStepPauseWaitTimeoutMs)
		{
			return failUnverified(GetStr(
				"STEP暂停等待超时：累计暂停已达%lldms，上限=%dms。",
				static_cast<long long>(pauseElapsed.count()), kStepPauseWaitTimeoutMs), -30001);
		}
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
					return failUnverified(waitError, -31000);
				}
				// 控制器的程序状态和变量监控通道不是同一快照：自然结束时 eStop
				// 可能先可见，末行 ntdone 稍后才可读。仅在同一受跟踪程序持续 eStop、
				// 没有控制器错误时给变量一个很短的稳定窗口；外部 STOP 的 ntdone
				// 会始终保持 0，因此不会被放宽成成功。
				std::string witnessError;
				bool naturallyCompleted = false;
				bool returnedToRunningState = false;
				int stableWitnessReads = 0;
				int witnessAttempts = 0;
				const auto witnessDeadline = std::chrono::steady_clock::now()
					+ std::chrono::milliseconds(kStepCompletionWitnessSettleTimeoutMs);
				while (std::chrono::steady_clock::now() <= witnessDeadline)
				{
					if (RobotOperationLease::IsCancellationRequested(this))
					{
						SetLastRobotError(
							"STEP硬件操作已被安全停止取消，完成见证等待已终止。");
						return -20000;
					}
					if (!m_bLocalDebugMark
						&& (m_pSTEPRobotClient == nullptr
							|| !m_bSocketConnected.load()
							|| WithSdkCommand([&]() { return m_pSTEPRobotClient->ConnectStatus(); }) < 0))
					{
						return failUnverified(
							"STEP等待自然完成见证失败：机器人连接已断开。", -32000);
					}

					int witnessState = -1;
					STEPROBOTSDK::MessageData witnessMessage;
					bool witnessReadOk = false;
					{
						std::lock_guard<std::recursive_mutex> modeLock(m_sdkCommandMutex);
						witnessState = m_pSTEPRobotClient != nullptr
							? static_cast<int>(m_pSTEPRobotClient->getProgramState())
							: -1;
						witnessMessage = m_pSTEPRobotClient != nullptr
							? m_pSTEPRobotClient->getMessageData()
							: STEPROBOTSDK::MessageData();
						if (witnessState == STEPROBOTSDK::eStop
							&& witnessMessage.m_MessageType != STEPROBOTSDK::eError)
						{
							witnessReadOk = VerifyGeneratedProgramCompletionWitnessLocked(witnessError);
						}
					}
					++witnessAttempts;

					if (witnessState == STEPROBOTSDK::eRun
						|| witnessState == STEPROBOTSDK::ePause)
					{
						returnedToRunningState = true;
						break;
					}
					if (witnessState != STEPROBOTSDK::eStop
						|| witnessMessage.m_MessageType == STEPROBOTSDK::eError)
					{
						return failUnverified(
							"STEP自然完成见证等待期间程序状态/控制器消息异常："
							+ GetRobotStatusText(), -31000);
					}
					if (witnessReadOk)
					{
						++stableWitnessReads;
						if (stableWitnessReads >= kStepCompletionWitnessStableReads)
						{
							naturallyCompleted = true;
							break;
						}
					}
					else
					{
						stableWitnessReads = 0;
					}
					Sleep(kStepCompletionWitnessPollIntervalMs);
				}
				if (returnedToRunningState)
				{
					Sleep(nDelayTime);
					continue;
				}
				if (!naturallyCompleted)
				{
					return failUnverified(
						GetStr(
							"STEP程序进入eStop但在%dms稳定窗口内没有自然完成见证，按提前STOP/中止处理："
							"Attempts=%d StableReads=%d %s",
							kStepCompletionWitnessSettleTimeoutMs,
							witnessAttempts,
							stableWitnessReads,
							witnessError.c_str()),
						-31001);
				}
				if (m_pRobotLog != nullptr)
				{
					m_pRobotLog->write(LogColor::SUCCESS,
						"STEP自然完成见证稳定确认 | Attempts=%d StableReads=%d",
						witnessAttempts,
						stableWitnessReads);
				}
				{
					std::lock_guard<std::recursive_mutex> modeLock(m_sdkCommandMutex);
					RobotOperationLease::MarkMotionCompleted(this);
					m_motionTrackedProjectName.clear();
					m_motionTrackedProgramName.clear();
					ClearGeneratedProgramCompletionWitnessLocked();
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
		bool contentIdentityChanged = false;
		std::string contentIdentityError;
		nRet = WithSdkCommand([&]()
			{
				// 与 ProgramLoadCmd 共用同一把 SDK 锁，关闭“安全终止已确认后，
				// 旧流程才继续加载可由下一次 START 恢复的程序”竞态。
				if (RobotOperationLease::IsCancellationRequested(this))
				{
					cancelledBeforeLoad = true;
					return -1;
				}
				const bool contentWitnessApplies =
					m_contentWitnessProjectName == StepNormalizeProjectName(sProjName)
					&& m_contentWitnessProgramName == sProgName;
				if (!m_contentWitnessProgramName.empty()
					&& (!contentWitnessApplies
						|| !VerifyGeneratedProgramRemoteContentLocked(contentIdentityError)))
				{
					contentIdentityChanged = true;
					ClearGeneratedProgramContentWitnessLocked();
					return -1;
				}
				const int loadRet = m_pSTEPRobotClient->ProgramLoadCmd(
					sProjName, sProgName, vnErrLine, true);
				if (loadRet == 0 && contentWitnessApplies
					&& !VerifyGeneratedProgramRemoteContentLocked(contentIdentityError))
				{
					contentIdentityChanged = true;
					StopAndUnloadGeneratedProgramLocked(
						StepNormalizeProjectName(sProjName), sProgName);
					ClearGeneratedProgramContentWitnessLocked();
					return -1;
				}
				return loadRet;
			});
		if (cancelledBeforeLoad)
		{
			SetLastRobotError(GetStr(
				"STEP硬件操作已被安全停止取消，拒绝加载后续程序：Project=%s Program=%s",
				sProjName.c_str(), sProgName.c_str()));
			return false;
		}
		if (contentIdentityChanged)
		{
			SetLastRobotError(GetStr(
				"STEP加载程序前后远端SRP/SRD内容身份无效或已变化，拒绝加载/START：Project=%s Program=%s Detail=%s",
				sProjName.c_str(), sProgName.c_str(), contentIdentityError.c_str()));
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
	if (safetyStop)
	{
		CancelActiveRemoteContentVerification();
	}
	std::unique_lock<std::recursive_mutex> modeLock(m_sdkCommandMutex);
	if (safetyStop)
	{
		CancelActiveRemoteContentVerification();
	}
	if (!safetyStop && RobotOperationLease::IsCancellationRequested(this))
	{
		SetLastRobotError("STEP硬件操作已被安全停止取消，拒绝恢复或切换到新的运行模式。");
		return false;
	}
	const bool startsProgram = nMode == MODEKEY::START;
	std::string startProject;
	std::string startProgram;
	if (startsProgram)
	{
		startProject = m_pSTEPRobotClient != nullptr
			? StepNormalizeProjectName(m_pSTEPRobotClient->getProjectName())
			: std::string();
		startProgram = m_pSTEPRobotClient != nullptr
			? m_pSTEPRobotClient->getProgramName()
			: std::string();
		if (startProgram.empty())
		{
			SetLastRobotError("STEP START 已拒绝：当前没有已加载程序。");
			return false;
		}
		std::string witnessError;
		if (!VerifyGeneratedProgramReadyForStartLocked(startProject, startProgram, witnessError))
		{
			SetLastRobotError(
				"STEP START 已拒绝：通用入口不能把任意已加载程序升级为可信程序；"
				"只有本软件刚生成/上传并保留来源见证的程序可启动，" + witnessError);
			ClearGeneratedProgramCompletionWitnessLocked();
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
	if (startsProgram)
	{
		std::string postStartContentError;
		if (!VerifyGeneratedProgramAfterStartWithSdkUnlock(
				modeLock, startProject, startProgram, postStartContentError))
		{
			SetLastRobotError(
				"STEP START后远端SRP/SRD内容复核失败，已立即STOP/Kill："
				+ postStartContentError);
			return false;
		}
	}
	return true;
}

//运行程序 运行的是目前加载的程序
bool STEPRobotCtrl::Prog_startRun_Py(bool resumeExisting)
{
	std::unique_lock<std::recursive_mutex> modeLock(m_sdkCommandMutex);
	return ProgStartRunWithSdkLock(modeLock, resumeExisting);
}

bool STEPRobotCtrl::ProgStartRunWithSdkLock(
	std::unique_lock<std::recursive_mutex>& modeLock,
	bool resumeExisting)
{
	if (!modeLock.owns_lock())
	{
		SetLastRobotError("STEP START内部调用未持SDK锁。");
		return false;
	}
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
	if (!resumeExisting)
	{
		std::string witnessError;
		if (!VerifyGeneratedProgramReadyForStartLocked(startProject, startProgram, witnessError))
		{
			SetLastRobotError("STEP START前程序身份已变化或不在可验证初始态，拒绝启动：" + witnessError);
			ClearGeneratedProgramCompletionWitnessLocked();
			return false;
		}
	}
	else
	{
		std::string resumeContentError;
		if (!VerifyGeneratedProgramRemoteContentLocked(resumeContentError))
		{
			const bool stopped = StopAndUnloadGeneratedProgramLocked(startProject, startProgram);
			if (stopped)
			{
				RobotOperationLease::MarkMotionCompleted(this);
			}
			SetLastRobotError(
				"STEP暂停恢复START前远端SRP/SRD内容已变化，拒绝继续："
				+ resumeContentError
				+ (stopped ? "；已STOP/Kill并稳定确认。" : "；STOP/Kill未确认。"));
			ClearGeneratedProgramCompletionWitnessLocked();
			return false;
		}
		int witnessValue = -1;
		const int witnessRet = m_pSTEPRobotClient->VariableIntReadCmd(
			startProject, startProgram, kStepCompletionWitnessName, witnessValue);
		const STEPROBOTSDK::MessageData resumeMessage = m_pSTEPRobotClient->getMessageData();
		const bool exactMessageWitness = resumeMessage.m_MessageType == STEPROBOTSDK::eInfo
			&& StepMessageText(resumeMessage) == m_completionWitnessMessageToken;
		const bool witnessIdentityMatches = m_completionWitnessProjectName == startProject
			&& m_completionWitnessProgramName == startProgram
			&& m_completionWitnessMessageToken == StepBuildCompletionMessageToken(startProgram);
		const bool initialWitnessState = witnessValue == 0
			&& !m_completionWitnessRuntimeLatched && !exactMessageWitness;
		const bool completedBarrierState = (witnessValue == 0 || witnessValue == 1)
			&& (m_completionWitnessRuntimeLatched || exactMessageWitness);
		if (!witnessIdentityMatches || witnessRet != 0
			|| (!initialWitnessState && !completedBarrierState))
		{
			const std::string resumeWitnessError = GetStr(
				"STEP继续运行失败：暂停程序完成见证无效。Witness=%s/%s Current=%s/%s ReadRet=%d Value=%d Latched=%d ExactMessage=%d",
				m_completionWitnessProjectName.c_str(), m_completionWitnessProgramName.c_str(),
				startProject.c_str(), startProgram.c_str(), witnessRet, witnessValue,
				m_completionWitnessRuntimeLatched ? 1 : 0, exactMessageWitness ? 1 : 0);
			modeLock.unlock();
			const bool stopped = RobotOperationLease::StopAndConfirmUnverifiedMotion(this);
			const std::string stopDetail = GetLastRobotError();
			SetLastRobotError(resumeWitnessError + (stopped
				? "；已自动中止并确认暂停程序不可恢复。"
				: "；自动中止暂停程序未确认：" + stopDetail));
			return false;
		}
		if (completedBarrierState)
		{
			m_completionWitnessRuntimeLatched = true;
		}
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
	std::string postStartContentError;
	if (!VerifyGeneratedProgramAfterStartWithSdkUnlock(
			modeLock, startProject, startProgram, postStartContentError))
	{
		SetLastRobotError(
			"STEP START后远端SRP/SRD内容复核失败，已立即STOP/Kill："
			+ postStartContentError);
		return false;
	}
	return true;
}

//停止程序 停止的是目前加载的程序
bool STEPRobotCtrl::Prog_stop_Py()
{
	CancelActiveRemoteContentVerification();
	std::lock_guard<std::recursive_mutex> modeLock(m_sdkCommandMutex);
	CancelActiveRemoteContentVerification();
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

bool STEPRobotCtrl::GetTrackedMotionIdentity(
	std::string& projectName,
	std::string& programName,
	bool* alreadyStopped)
{
	projectName.clear();
	programName.clear();
	if (alreadyStopped != nullptr)
	{
		*alreadyStopped = false;
	}
	std::lock_guard<std::recursive_mutex> sdkLock(m_sdkCommandMutex);
	if (m_pSTEPRobotClient == nullptr)
	{
		SetLastRobotError("STEP运动身份读取失败：SDK客户端未初始化。");
		return false;
	}
	if (!m_bLocalDebugMark
		&& (!m_bSocketConnected.load() || m_pSTEPRobotClient->ConnectStatus() < 0))
	{
		SetLastRobotError("STEP运动身份读取失败：机器人连接不可用。");
		return false;
	}
	if (RobotOperationLease::IsCancellationRequested(this)
		|| !RobotOperationLease::MotionCompletionPending(this))
	{
		SetLastRobotError("STEP运动身份读取失败：当前没有受跟踪的未完成运动。");
		return false;
	}

	const std::string currentProject =
		StepNormalizeProjectName(m_pSTEPRobotClient->getProjectName());
	const std::string currentProgram = m_pSTEPRobotClient->getProgramName();
	const int currentState = static_cast<int>(m_pSTEPRobotClient->getProgramState());
	if (m_motionTrackedProjectName.empty()
		|| m_motionTrackedProgramName.empty()
		|| currentProject != m_motionTrackedProjectName
		|| currentProgram != m_motionTrackedProgramName
		|| (currentState != STEPROBOTSDK::eRun
			&& currentState != STEPROBOTSDK::ePause
			&& currentState != STEPROBOTSDK::eStop))
	{
		SetLastRobotError(GetStr(
			"STEP运动身份读取失败：跟踪身份或运行状态不一致。Tracked=%s/%s Current=%s/%s State=%d",
			m_motionTrackedProjectName.c_str(),
			m_motionTrackedProgramName.c_str(),
			currentProject.c_str(),
			currentProgram.c_str(),
			currentState));
		return false;
	}

	projectName = m_motionTrackedProjectName;
	programName = m_motionTrackedProgramName;
	if (alreadyStopped != nullptr)
	{
		*alreadyStopped = currentState == STEPROBOTSDK::eStop;
	}
	ClearLastRobotError();
	return true;
}

bool STEPRobotCtrl::PauseTrackedProgramAndWait(
	const std::string& expectedProgramName,
	int& programLine,
	T_ROBOT_COORS& pausedPose,
	std::string* projectName,
	std::string* programName)
{
	programLine = -1;
	pausedPose = T_ROBOT_COORS();
	std::lock_guard<std::recursive_mutex> sdkLock(m_sdkCommandMutex);
	if (m_pSTEPRobotClient == nullptr)
	{
		SetLastRobotError("STEP暂停失败：SDK客户端未初始化。");
		return false;
	}
	const auto connectionUsable = [this]()
		{
			return m_bLocalDebugMark
				|| (m_bSocketConnected.load() && m_pSTEPRobotClient->ConnectStatus() >= 0);
		};
	if (!connectionUsable())
	{
		SetLastRobotError("STEP暂停失败：机器人连接不可用，禁止使用陈旧程序状态或位姿生成断点。");
		return false;
	}
	if (RobotOperationLease::IsCancellationRequested(this))
	{
		SetLastRobotError("STEP暂停失败：当前硬件操作已被安全停止取消。");
		return false;
	}
	if (!RobotOperationLease::MotionCompletionPending(this))
	{
		SetLastRobotError("STEP暂停失败：本软件没有正在跟踪的可恢复程序。");
		return false;
	}

	const std::string currentProject = StepNormalizeProjectName(m_pSTEPRobotClient->getProjectName());
	const std::string currentProgram = m_pSTEPRobotClient->getProgramName();
	if (expectedProgramName.empty()
		|| m_motionTrackedProjectName.empty()
		|| m_motionTrackedProgramName.empty()
		|| m_motionTrackedProgramName != expectedProgramName
		|| currentProject != m_motionTrackedProjectName
		|| currentProgram != expectedProgramName)
	{
		SetLastRobotError(GetStr(
			"STEP暂停失败：当前程序身份与断点上下文不一致。Expected=%s Tracked=%s/%s Current=%s/%s",
			expectedProgramName.c_str(),
			m_motionTrackedProjectName.c_str(),
			m_motionTrackedProgramName.c_str(),
			currentProject.c_str(),
			currentProgram.c_str()));
		return false;
	}

	int state = static_cast<int>(m_pSTEPRobotClient->getProgramState());
	if (state != STEPROBOTSDK::ePause)
	{
		if (state != STEPROBOTSDK::eRun)
		{
			SetLastRobotError(GetStr(
				"STEP暂停失败：跟踪程序不在运行/暂停态，State=%d Program=%s",
				state, currentProgram.c_str()));
			return false;
		}
		const int stopRet = m_pSTEPRobotClient->SetModeCmd(MODEKEY::STOP, true);
		if (stopRet != 0)
		{
			SetLastRobotError(GetStr(
				"STEP暂停命令失败：原因=%s(%d)，Program=%s",
				GetErrorText(stopRet), stopRet, currentProgram.c_str()));
			return false;
		}
	}

	int stablePauseCount = 0;
	for (int retry = 0; retry < 60; ++retry)
	{
		if (!connectionUsable())
		{
			SetLastRobotError("STEP暂停确认失败：等待暂停期间机器人连接已断开。");
			return false;
		}
		if (RobotOperationLease::IsCancellationRequested(this))
		{
			SetLastRobotError("STEP暂停确认已让位给安全停止请求。");
			return false;
		}
		const std::string observedProject = StepNormalizeProjectName(m_pSTEPRobotClient->getProjectName());
		const std::string observedProgram = m_pSTEPRobotClient->getProgramName();
		state = static_cast<int>(m_pSTEPRobotClient->getProgramState());
		if (observedProject != m_motionTrackedProjectName
			|| observedProgram != m_motionTrackedProgramName)
		{
			SetLastRobotError(GetStr(
				"STEP暂停确认失败：等待期间程序身份变化。Tracked=%s/%s Current=%s/%s",
				m_motionTrackedProjectName.c_str(),
				m_motionTrackedProgramName.c_str(),
				observedProject.c_str(),
				observedProgram.c_str()));
			return false;
		}
		if (state == STEPROBOTSDK::ePause)
		{
			++stablePauseCount;
			if (stablePauseCount >= 4)
			{
				break;
			}
		}
		else if (state == STEPROBOTSDK::eRun)
		{
			stablePauseCount = 0;
		}
		else
		{
			SetLastRobotError(GetStr(
				"STEP暂停确认失败：程序进入不可恢复状态，State=%d Program=%s",
				state, observedProgram.c_str()));
			return false;
		}
		Sleep(50);
	}
	if (stablePauseCount < 4)
	{
		SetLastRobotError(GetStr(
			"STEP暂停确认超时：未获得连续稳定 ePause，State=%d Program=%s",
			state, currentProgram.c_str()));
		return false;
	}

	if (!connectionUsable())
	{
		SetLastRobotError("STEP暂停快照失败：机器人连接已断开。");
		return false;
	}
	if (RobotOperationLease::IsCancellationRequested(this))
	{
		SetLastRobotError("STEP暂停快照已让位给安全停止请求。");
		return false;
	}
	const int firstProgramLine = m_pSTEPRobotClient->getCurrentLine();
	const T_ROBOT_COORS firstPose = StepRobotCartPosToCoors(m_pSTEPRobotClient->getCartPosWorld());
	Sleep(50);
	if (!connectionUsable())
	{
		SetLastRobotError("STEP暂停快照失败：两次位姿读取之间机器人连接已断开。");
		return false;
	}
	if (RobotOperationLease::IsCancellationRequested(this))
	{
		SetLastRobotError("STEP暂停快照已让位给安全停止请求。");
		return false;
	}
	const int secondProgramLine = m_pSTEPRobotClient->getCurrentLine();
	const T_ROBOT_COORS secondPose = StepRobotCartPosToCoors(m_pSTEPRobotClient->getCartPosWorld());
	auto poseFinite = [](const T_ROBOT_COORS& pose)
		{
			return std::isfinite(pose.dX) && std::isfinite(pose.dY) && std::isfinite(pose.dZ)
				&& std::isfinite(pose.dRX) && std::isfinite(pose.dRY) && std::isfinite(pose.dRZ);
		};
	const double dx = secondPose.dX - firstPose.dX;
	const double dy = secondPose.dY - firstPose.dY;
	const double dz = secondPose.dZ - firstPose.dZ;
	const double poseDriftMm = std::sqrt(dx * dx + dy * dy + dz * dz);
	const auto wrappedAngleDelta = [](double left, double right)
		{
			return std::abs(std::remainder(left - right, 360.0));
		};
	const double poseAngleDriftDeg = (std::max)({
		wrappedAngleDelta(secondPose.dRX, firstPose.dRX),
		wrappedAngleDelta(secondPose.dRY, firstPose.dRY),
		wrappedAngleDelta(secondPose.dRZ, firstPose.dRZ) });
	const bool identityStable =
		connectionUsable()
		&& StepNormalizeProjectName(m_pSTEPRobotClient->getProjectName()) == m_motionTrackedProjectName
		&& m_pSTEPRobotClient->getProgramName() == m_motionTrackedProgramName
		&& m_pSTEPRobotClient->getProgramState() == STEPROBOTSDK::ePause;
	if (firstProgramLine < 0 || firstProgramLine != secondProgramLine
		|| !poseFinite(firstPose) || !poseFinite(secondPose)
		|| !std::isfinite(poseDriftMm) || poseDriftMm > 0.2
		|| !std::isfinite(poseAngleDriftDeg) || poseAngleDriftDeg > 0.2
		|| !identityStable)
	{
		SetLastRobotError(GetStr(
			"STEP暂停快照未稳定：Line=%d/%d Drift=%.3fmm/%.3fdeg IdentityStable=%d Program=%s",
			firstProgramLine, secondProgramLine, poseDriftMm, poseAngleDriftDeg,
			identityStable ? 1 : 0, currentProgram.c_str()));
		return false;
	}

	programLine = secondProgramLine;
	pausedPose = secondPose;
	if (projectName != nullptr)
	{
		*projectName = m_motionTrackedProjectName;
	}
	if (programName != nullptr)
	{
		*programName = m_motionTrackedProgramName;
	}
	ClearLastRobotError();
	return true;
}

bool STEPRobotCtrl::ResumeTrackedProgramFromPause(
	const std::string& expectedProgramName,
	const T_ROBOT_COORS& checkpointPose,
	double maxPositionDeviationMm,
	double maxAngleDeviationDeg,
	double* positionDeviationMm,
	double* angleDeviationDeg)
{
	if (positionDeviationMm != nullptr)
	{
		*positionDeviationMm = std::numeric_limits<double>::infinity();
	}
	if (angleDeviationDeg != nullptr)
	{
		*angleDeviationDeg = std::numeric_limits<double>::infinity();
	}
	std::unique_lock<std::recursive_mutex> sdkLock(m_sdkCommandMutex);
	if (m_pSTEPRobotClient == nullptr)
	{
		SetLastRobotError("STEP继续失败：SDK客户端未初始化。");
		return false;
	}
	if (!m_bLocalDebugMark
		&& (!m_bSocketConnected.load() || m_pSTEPRobotClient->ConnectStatus() < 0))
	{
		SetLastRobotError("STEP继续失败：机器人连接不可用，禁止使用陈旧位姿发送START。");
		return false;
	}
	if (RobotOperationLease::IsCancellationRequested(this)
		|| !RobotOperationLease::MotionCompletionPending(this))
	{
		SetLastRobotError("STEP继续失败：当前程序已被安全取消或不再处于受跟踪运行态。");
		return false;
	}
	if (expectedProgramName.empty()
		|| m_motionTrackedProjectName.empty()
		|| m_motionTrackedProgramName != expectedProgramName
		|| StepNormalizeProjectName(m_pSTEPRobotClient->getProjectName()) != m_motionTrackedProjectName
		|| m_pSTEPRobotClient->getProgramName() != expectedProgramName
		|| m_pSTEPRobotClient->getProgramState() != STEPROBOTSDK::ePause)
	{
		SetLastRobotError(GetStr(
			"STEP继续失败：当前暂停程序身份不一致。Expected=%s Tracked=%s/%s Current=%s/%s State=%d",
			expectedProgramName.c_str(),
			m_motionTrackedProjectName.c_str(),
			m_motionTrackedProgramName.c_str(),
			StepNormalizeProjectName(m_pSTEPRobotClient->getProjectName()).c_str(),
			m_pSTEPRobotClient->getProgramName().c_str(),
			static_cast<int>(m_pSTEPRobotClient->getProgramState())));
		return false;
	}
	if (!std::isfinite(maxPositionDeviationMm) || maxPositionDeviationMm < 0.0
		|| !std::isfinite(maxAngleDeviationDeg) || maxAngleDeviationDeg < 0.0)
	{
		SetLastRobotError("STEP继续失败：断点偏差阈值无效。");
		return false;
	}

	const auto poseFinite = [](const T_ROBOT_COORS& pose)
		{
			return std::isfinite(pose.dX) && std::isfinite(pose.dY) && std::isfinite(pose.dZ)
				&& std::isfinite(pose.dRX) && std::isfinite(pose.dRY) && std::isfinite(pose.dRZ);
		};
	const auto wrappedAngleDelta = [](double left, double right)
		{
			return std::abs(std::remainder(left - right, 360.0));
		};
	if (!poseFinite(checkpointPose))
	{
		SetLastRobotError("STEP继续失败：落盘断点位姿无效。");
		return false;
	}

	const T_ROBOT_COORS firstPose = StepRobotCartPosToCoors(m_pSTEPRobotClient->getCartPosWorld());
	Sleep(50);
	if (RobotOperationLease::IsCancellationRequested(this))
	{
		SetLastRobotError("STEP继续位姿核对已让位给安全停止请求。");
		return false;
	}
	const T_ROBOT_COORS secondPose = StepRobotCartPosToCoors(m_pSTEPRobotClient->getCartPosWorld());
	const double stableDx = secondPose.dX - firstPose.dX;
	const double stableDy = secondPose.dY - firstPose.dY;
	const double stableDz = secondPose.dZ - firstPose.dZ;
	const double stablePositionDriftMm = std::sqrt(
		stableDx * stableDx + stableDy * stableDy + stableDz * stableDz);
	const double stableAngleDriftDeg = (std::max)({
		wrappedAngleDelta(secondPose.dRX, firstPose.dRX),
		wrappedAngleDelta(secondPose.dRY, firstPose.dRY),
		wrappedAngleDelta(secondPose.dRZ, firstPose.dRZ) });
	const bool identityStillPaused =
		StepNormalizeProjectName(m_pSTEPRobotClient->getProjectName()) == m_motionTrackedProjectName
		&& m_pSTEPRobotClient->getProgramName() == expectedProgramName
		&& m_pSTEPRobotClient->getProgramState() == STEPROBOTSDK::ePause;
	if (!poseFinite(firstPose) || !poseFinite(secondPose)
		|| !std::isfinite(stablePositionDriftMm) || stablePositionDriftMm > 0.2
		|| !std::isfinite(stableAngleDriftDeg) || stableAngleDriftDeg > 0.2
		|| !identityStillPaused)
	{
		SetLastRobotError(GetStr(
			"STEP继续失败：暂停位姿/程序不稳定，Drift=%.3fmm/%.3fdeg IdentityStable=%d。",
			stablePositionDriftMm, stableAngleDriftDeg, identityStillPaused ? 1 : 0));
		return false;
	}

	const double dx = secondPose.dX - checkpointPose.dX;
	const double dy = secondPose.dY - checkpointPose.dY;
	const double dz = secondPose.dZ - checkpointPose.dZ;
	const double positionDeviation = std::sqrt(dx * dx + dy * dy + dz * dz);
	const double angleDeviation = (std::max)({
		wrappedAngleDelta(secondPose.dRX, checkpointPose.dRX),
		wrappedAngleDelta(secondPose.dRY, checkpointPose.dRY),
		wrappedAngleDelta(secondPose.dRZ, checkpointPose.dRZ) });
	if (positionDeviationMm != nullptr)
	{
		*positionDeviationMm = positionDeviation;
	}
	if (angleDeviationDeg != nullptr)
	{
		*angleDeviationDeg = angleDeviation;
	}
	if (!std::isfinite(positionDeviation) || positionDeviation > maxPositionDeviationMm
		|| !std::isfinite(angleDeviation) || angleDeviation > maxAngleDeviationDeg)
	{
		SetLastRobotError(GetStr(
			"STEP继续被拒绝：当前位置偏离落盘断点 %.3fmm/%.3fdeg，允许上限 %.3fmm/%.3fdeg。请手动回到断点附近后重试。",
			positionDeviation, angleDeviation, maxPositionDeviationMm, maxAngleDeviationDeg));
		return false;
	}

	// 复用当前唯一 SDK lock，关闭位姿核对与 START 的竞态；START 后内容回读会
	// 显式释放这一把锁，因此红色 STOP 不会被外层递归锁继续阻塞。
	return ProgStartRunWithSdkLock(sdkLock, true);
}

bool STEPRobotCtrl::AbortCurrentProgram()
{
	CancelActiveRemoteContentVerification();
	std::lock_guard<std::recursive_mutex> sdkLock(m_sdkCommandMutex);
	CancelActiveRemoteContentVerification();
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
		ClearGeneratedProgramCompletionWitnessLocked();
		ClearLastRobotError();
		return true;
	}
	const std::string currentProject = StepNormalizeProjectName(m_pSTEPRobotClient->getProjectName());
	const std::string currentProgram = m_pSTEPRobotClient->getProgramName();
	if (currentProgram.empty())
	{
		int stableStoppedCount = 0;
		for (int retry = 0; retry < 40; ++retry)
		{
			if (m_pSTEPRobotClient->getProgramName().empty()
				&& m_pSTEPRobotClient->getProgramState() == STEPROBOTSDK::eStop)
			{
				if (++stableStoppedCount >= 4)
				{
					RobotOperationLease::MarkMotionCompleted(this);
					m_motionTrackedProjectName.clear();
					m_motionTrackedProgramName.clear();
					ClearGeneratedProgramCompletionWitnessLocked();
					ClearLastRobotError();
					return true;
				}
			}
			else
			{
				stableStoppedCount = 0;
			}
			Sleep(50);
		}
		SetLastRobotError(
			"STEP安全取消未确认：程序已卸载，但未稳定回读到空程序+eStop终态。");
		return false;
	}
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
				ClearGeneratedProgramCompletionWitnessLocked();
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

bool STEPRobotCtrl::AbortPersistedProgramForRecovery(const std::string& expectedProgramName)
{
	CancelActiveRemoteContentVerification();
	std::lock_guard<std::recursive_mutex> sdkLock(m_sdkCommandMutex);
	CancelActiveRemoteContentVerification();
	if (m_pSTEPRobotClient == nullptr || expectedProgramName.empty())
	{
		SetLastRobotError("STEP持久恢复终止失败：SDK客户端或记录绑定程序名无效。");
		return false;
	}
	if (!m_bLocalDebugMark)
	{
		const int connectStatus = m_bSocketConnected.load()
			? m_pSTEPRobotClient->ConnectStatus() : -1;
		if (connectStatus < 0)
		{
			const int initRet = m_pSTEPRobotClient->init(m_sSocketIP.c_str(), m_nSocketPort);
			if (initRet != 0)
			{
				m_bSocketConnected.store(false);
				SetLastRobotError(GetStr(
					"STEP持久恢复终止重连失败：IP=%s Port=%d 原因=%s(%d)",
					m_sSocketIP.c_str(), m_nSocketPort, GetErrorText(initRet), initRet));
				return false;
			}
			m_bSocketConnected.store(true);
			m_nTimestampAxisLatch.store(0);
			m_lastValidRobotMs.store(0);
			m_bTimestampFallbackLogged.store(false);
		}
	}

	const auto stableEmptyStopped = [this]()
		{
			int stableCount = 0;
			for (int retry = 0; retry < 40; ++retry)
			{
				if (!m_bLocalDebugMark
					&& (!m_bSocketConnected.load()
						|| m_pSTEPRobotClient->ConnectStatus() < 0))
				{
					return false;
				}
				if (m_pSTEPRobotClient->getProgramName().empty()
					&& m_pSTEPRobotClient->getProgramState() == STEPROBOTSDK::eStop)
				{
					if (++stableCount >= 4)
					{
						return true;
					}
				}
				else
				{
					stableCount = 0;
				}
				Sleep(50);
			}
			return false;
		};

	const std::string currentProgram = m_pSTEPRobotClient->getProgramName();
	const std::string currentProject = StepNormalizeProjectName(m_pSTEPRobotClient->getProjectName());
	const int currentState = static_cast<int>(m_pSTEPRobotClient->getProgramState());
	RobotRecoverySafetyPolicy::ObservedProgramState observedState =
		RobotRecoverySafetyPolicy::ObservedProgramState::Unknown;
	if (currentState == STEPROBOTSDK::eRun)
	{
		observedState = RobotRecoverySafetyPolicy::ObservedProgramState::Running;
	}
	else if (currentState == STEPROBOTSDK::ePause)
	{
		observedState = RobotRecoverySafetyPolicy::ObservedProgramState::Paused;
	}
	else if (currentState == STEPROBOTSDK::eStop)
	{
		observedState = RobotRecoverySafetyPolicy::ObservedProgramState::Stopped;
	}
	const bool alreadyStable = currentProgram.empty() && stableEmptyStopped();
	const auto action = RobotRecoverySafetyPolicy::ResolveStepPersistedProgramAction(
		QString::fromStdString(expectedProgramName), QString::fromStdString(currentProgram),
		observedState, alreadyStable);
	if (action == RobotRecoverySafetyPolicy::PersistedProgramAction::AlreadyStopped)
	{
		m_motionTrackedProjectName.clear();
		m_motionTrackedProgramName.clear();
		ClearGeneratedProgramCompletionWitnessLocked();
		ClearLastRobotError();
		return true;
	}
	if (action == RobotRecoverySafetyPolicy::PersistedProgramAction::Reject)
	{
		SetLastRobotError(GetStr(
			"STEP持久恢复终止已拒绝：记录程序与控制器当前程序/状态不一致。Expected=%s Current=%s/%s State=%d",
			expectedProgramName.c_str(), currentProject.c_str(), currentProgram.c_str(), currentState));
		return false;
	}
	if (currentProject.empty())
	{
		SetLastRobotError("STEP持久恢复终止失败：同名程序已加载但当前工程名为空，拒绝无身份 Kill。");
		return false;
	}
	if (action == RobotRecoverySafetyPolicy::PersistedProgramAction::StopThenKill)
	{
		const int stopRet = m_pSTEPRobotClient->SetModeCmd(MODEKEY::STOP, true);
		if (stopRet != 0)
		{
			SetLastRobotError(GetStr(
				"STEP持久恢复 STOP 失败：Program=%s Ret=%d", currentProgram.c_str(), stopRet));
			return false;
		}
	}
	const int killRet = m_pSTEPRobotClient->ProgramKillCmd(
		currentProject, currentProgram, true);
	if (killRet != 0)
	{
		SetLastRobotError(GetStr(
			"STEP持久恢复 Kill 失败：Project=%s Program=%s Ret=%d",
			currentProject.c_str(), currentProgram.c_str(), killRet));
		return false;
	}
	if (!stableEmptyStopped())
	{
		SetLastRobotError(GetStr(
			"STEP持久恢复终止未确认：Kill 后未稳定回读空程序+eStop。Expected=%s Remaining=%s State=%d",
			expectedProgramName.c_str(), m_pSTEPRobotClient->getProgramName().c_str(),
			static_cast<int>(m_pSTEPRobotClient->getProgramState())));
		return false;
	}
	m_motionTrackedProjectName.clear();
	m_motionTrackedProgramName.clear();
	ClearGeneratedProgramCompletionWitnessLocked();
	ClearLastRobotError();
	return true;
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
	SetLastRobotError(
		"STEP拒绝通过通用CallJob启动来源未验证的程序：" + sJobName
		+ "。请使用本软件生成/上传且带WaitIsFinished+ntdone契约的受验证入口。");
	return false;
}

int STEPRobotCtrl::ContiMoveAny(const std::vector<T_ROBOT_MOVE_INFO>& vtRobotMoveInfo)
{
	return ContiMoveAnyWithProgramName(vtRobotMoveInfo, StepMakeProgramName(this));
}

int STEPRobotCtrl::ContiMoveAnyWithProgramName(
	const std::vector<T_ROBOT_MOVE_INFO>& vtRobotMoveInfo,
	const std::string& programName,
	const RobotTrajectoryHandle* preparedHandle)
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
	const bool usePreparedUpload = preparedHandle != nullptr;
	if (usePreparedUpload
		&& (!preparedHandle->prepared || preparedHandle->programName.empty()
			|| preparedHandle->localProgramPath.empty() || preparedHandle->localDataPath.empty()
			|| preparedHandle->remoteProgramPath.empty() || preparedHandle->remoteDataPath.empty()
			|| preparedHandle->programContentSha256.size() != 64
			|| preparedHandle->dataContentSha256.size() != 64))
	{
		SetLastRobotError("STEP启动已下发轨迹失败：轨迹句柄不完整。");
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
	// Remote names are controller-safe and globally unique; local artifacts add
	// endpoint + process + driver-instance isolation so two robots never share files.
	const std::filesystem::path localDirPath = usePreparedUpload
		? std::filesystem::path(preparedHandle->localProgramPath).parent_path()
		: StepInstanceOutputDirectory(this);
	const std::filesystem::path localProgramPath = usePreparedUpload
		? std::filesystem::path(preparedHandle->localProgramPath)
		: localDirPath / (sProgramName + ".srp");
	const std::filesystem::path localDataPath = usePreparedUpload
		? std::filesystem::path(preparedHandle->localDataPath)
		: localDirPath / (sProgramName + ".srd");
	const std::string sLocalProgramFile = StepLocalPathBytes(localProgramPath);
	const std::string sLocalDataFile = StepLocalPathBytes(localDataPath);
	const std::string sRemoteBaseDir = StepBuildRemoteProjectDir(sProjectName);
	const std::string expectedRemoteProgramFile = sRemoteBaseDir + "/" + sProgramName + ".srp";
	const std::string expectedRemoteDataFile = sRemoteBaseDir + "/" + sProgramName + ".srd";
	const std::string sRemoteProgramFile = usePreparedUpload
		? preparedHandle->remoteProgramPath : expectedRemoteProgramFile;
	const std::string sRemoteDataFile = usePreparedUpload
		? preparedHandle->remoteDataPath : expectedRemoteDataFile;
	if (usePreparedUpload
		&& (StepSanitizeProgramName(preparedHandle->programName) != sProgramName
			|| sRemoteProgramFile != expectedRemoteProgramFile
			|| sRemoteDataFile != expectedRemoteDataFile))
	{
		SetLastRobotError("STEP启动已下发轨迹失败：句柄中的程序身份或远端路径不匹配。");
		return -1;
	}
	{
		std::lock_guard<std::recursive_mutex> sdkLock(m_sdkCommandMutex);
		ClearGeneratedProgramCompletionWitnessLocked();
	}

	RobotRecoverySafetyPolicy::ProgramContentIdentity localProgramIdentity;
	RobotRecoverySafetyPolicy::ProgramContentIdentity localDataIdentity;
	if (!usePreparedUpload)
	{
		try
		{
			std::filesystem::create_directories(localDirPath);
		}
		catch (const std::exception& e)
		{
			SetLastRobotError(GetStr("STEP连续运动失败：创建本地目录失败，%s", e.what()));
			m_pRobotLog->write(LogColor::ERR, "STEP ContiMoveAny 创建本地目录失败：%s", e.what());
			return -2;
		}

		const std::string sSrpContent = StepBuildSrpContent(
			weaveMoveInfo, true, sProgramName);
		std::string processValidationError;
		const std::string sSrdContent = StepBuildSrdContent(
			weaveMoveInfo, m_tAxisUnit, true, &processValidationError);
		if (!processValidationError.empty())
		{
			SetLastRobotError(processValidationError);
			m_pRobotLog->write(LogColor::ERR, "STEP ContiMoveAny 实际焊接工艺校验失败：%s",
				processValidationError.c_str());
			return -2;
		}

		{
			std::lock_guard<std::mutex> filePairLock(g_stepGeneratedFilePairMutex);
			if (!StepWriteTextFile(localProgramPath, sSrpContent))
			{
				SetLastRobotError(GetStr("STEP连续运动失败：写入SRP失败，%s", sLocalProgramFile.c_str()));
				m_pRobotLog->write(LogColor::ERR, "STEP ContiMoveAny 写入SRP失败：%s", sLocalProgramFile.c_str());
				return -3;
			}
			if (!StepWriteTextFile(localDataPath, sSrdContent))
			{
				SetLastRobotError(GetStr("STEP连续运动失败：写入SRD失败，%s", sLocalDataFile.c_str()));
				m_pRobotLog->write(LogColor::ERR, "STEP ContiMoveAny 写入SRD失败：%s", sLocalDataFile.c_str());
				return -4;
			}
			std::string identityError;
			if (!StepReadProgramContentIdentity(
					localProgramPath, localProgramIdentity, identityError)
				|| !StepReadProgramContentIdentity(
					localDataPath, localDataIdentity, identityError))
			{
				SetLastRobotError("STEP连续运动失败：本地SRP/SRD写后身份计算失败，" + identityError);
				return -4;
			}
		}

		m_pRobotLog->write(LogColor::SUCCESS,
			"STEP ContiMoveAny 已生成程序 | Project=%s | Program=%s | PointCount=%d",
			sProjectName.c_str(), sProgramName.c_str(), static_cast<int>(vtRobotMoveInfo.size()));
	}
	else
	{
		std::string identityError;
		std::lock_guard<std::mutex> filePairLock(g_stepGeneratedFilePairMutex);
		if (!StepReadProgramContentIdentity(localProgramPath, localProgramIdentity, identityError)
			|| !StepReadProgramContentIdentity(localDataPath, localDataIdentity, identityError))
		{
			SetLastRobotError("STEP启动已下发轨迹失败：本地SRP/SRD身份读取失败，" + identityError);
			return -4;
		}
		if (localProgramIdentity.sha256.toStdString() != preparedHandle->programContentSha256
			|| localDataIdentity.sha256.toStdString() != preparedHandle->dataContentSha256
			|| static_cast<std::uint64_t>(localProgramIdentity.size) != preparedHandle->programContentSize
			|| static_cast<std::uint64_t>(localDataIdentity.size) != preparedHandle->dataContentSize)
		{
			SetLastRobotError("STEP启动已下发轨迹失败：本地SRP/SRD内容已偏离Downlink冻结身份。");
			return -4;
		}
		m_pRobotLog->write(LogColor::SUCCESS,
			"STEP ContiMoveAny 使用已下发程序 | Project=%s | Program=%s",
			sProjectName.c_str(), sProgramName.c_str());
	}

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
		m_pRobotLog->write(LogColor::DEFAULT,
			usePreparedUpload
			? "STEP ContiMoveAny 启动前重建FTP客户端，用于远端内容身份复核"
			: "STEP ContiMoveAny 上传前重建FTP客户端，避免复用失效会话");
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
	RobotRecoverySafetyPolicy::ProgramContentIdentity beforeUploadProgram;
	RobotRecoverySafetyPolicy::ProgramContentIdentity beforeUploadData;
	std::string localIdentityError;
	if (!StepReadProgramContentIdentity(
			localProgramPath, beforeUploadProgram, localIdentityError)
		|| !StepReadProgramContentIdentity(
			localDataPath, beforeUploadData, localIdentityError)
		|| !RobotRecoverySafetyPolicy::SameProgramContent(
			localProgramIdentity, beforeUploadProgram)
		|| !RobotRecoverySafetyPolicy::SameProgramContent(
			localDataIdentity, beforeUploadData))
	{
		SetLastRobotError("STEP连续运动失败：启动前本地SRP/SRD内容发生变化，" + localIdentityError);
		return -6;
	}

	if (!usePreparedUpload)
	{
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
		RobotRecoverySafetyPolicy::ProgramContentIdentity afterUploadProgram;
		RobotRecoverySafetyPolicy::ProgramContentIdentity afterUploadData;
		if (!StepReadProgramContentIdentity(
				localProgramPath, afterUploadProgram, localIdentityError)
			|| !StepReadProgramContentIdentity(
				localDataPath, afterUploadData, localIdentityError)
			|| !RobotRecoverySafetyPolicy::SameProgramContent(
				localProgramIdentity, afterUploadProgram)
			|| !RobotRecoverySafetyPolicy::SameProgramContent(
				localDataIdentity, afterUploadData))
		{
			SetLastRobotError("STEP连续运动失败：上传后本地SRP/SRD内容发生变化，" + localIdentityError);
			return -8;
		}
	}
	std::string contentWitnessError;
	if (!ArmGeneratedProgramContentWitness(
			sProjectName, sProgramName,
			sRemoteProgramFile, sRemoteDataFile,
			localProgramIdentity, localDataIdentity,
			contentWitnessError))
	{
		SetLastRobotError("STEP连续运动失败：远端SRP/SRD上传回读身份不一致，" + contentWitnessError);
		m_pRobotLog->write(LogColor::ERR, "%s", GetLastRobotError().c_str());
		return -8;
	}

	m_pRobotLog->write(LogColor::SUCCESS,
		usePreparedUpload
		? "STEP ContiMoveAny 已下发程序身份复核完成，未重复上传 | SRP=%s | SRD=%s"
		: "STEP ContiMoveAny 上传完成 | SRP=%s | SRD=%s",
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
	std::string completionWitnessError;
	if (!ArmGeneratedProgramCompletionWitness(
		sProjectName, sProgramName, completionWitnessError))
	{
		SetLastRobotError(completionWitnessError);
		m_pRobotLog->write(LogColor::ERR, "%s", completionWitnessError.c_str());
		return -10;
	}

	const auto failRunPrepare = [this, &sProgramName](const char* stepName, int ret) -> int
		{
			{
				std::lock_guard<std::recursive_mutex> modeLock(m_sdkCommandMutex);
				ClearGeneratedProgramCompletionWitnessLocked();
			}
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

	// 控制器已经处于连续模式时禁止重复写运行模式。ProgramRunModeCmd 只表示命令已受理，
	// 若紧接着 START，异步模式命令可能在机器人运行后才到达并触发 4009。
	// 只有明确观察到非连续模式时，才在程序稳定 eStop 的窗口内切换。
	const int continueProgramMode = static_cast<int>(STEPROBOTSDK::eContinue);
	const int programModeBefore = static_cast<int>(getProgramMode());
	bool programModeCommandIssued = false;
	int programStateBeforeModeCommand = STEPROBOTSDK::eStop;
	if (programModeBefore != continueProgramMode)
	{
		{
			std::lock_guard<std::recursive_mutex> modeLock(m_sdkCommandMutex);
			if (RobotOperationLease::IsCancellationRequested(this))
			{
				return failRunPrepare("安全停止后取消切换连续运行模式", -20000);
			}
			programStateBeforeModeCommand = m_pSTEPRobotClient != nullptr
				? static_cast<int>(m_pSTEPRobotClient->getProgramState())
				: -1;
			if (programStateBeforeModeCommand == STEPROBOTSDK::eStop)
			{
				programModeCommandIssued = true;
				nPrepareRet = m_pSTEPRobotClient->ProgramRunModeCmd(continueProgramMode);
			}
		}
		if (programStateBeforeModeCommand != STEPROBOTSDK::eStop)
		{
			m_pRobotLog->write(LogColor::ERR,
				"STEP ContiMoveAny 拒绝切换连续运行模式：程序未停止 | Before=%d State=%d",
				programModeBefore,
				programStateBeforeModeCommand);
			return failRunPrepare("切换连续运行模式前程序未稳定停止",
				programStateBeforeModeCommand);
		}
		if (nPrepareRet != 0)
		{
			m_pRobotLog->write(LogColor::ERR,
				"STEP ContiMoveAny 设置连续运行模式失败 | Before=%d CommandRet=%d",
				programModeBefore,
				nPrepareRet);
			return failRunPrepare("切换连续运行模式", nPrepareRet);
		}

		// 非连续 -> 连续必须观察到真实状态边沿；命令返回成功不能直接放行 START。
		Sleep(50);
	}

	// 无论是否发过命令，都要求连续三次稳定回读 eContinue。已是连续模式时这里只读不写，
	// 避免重复模式命令与随后 START 竞争。
	int stableContinueReads = 0;
	int programModeAfter = static_cast<int>(getProgramMode());
	for (int i = 0; i < 20; ++i)
	{
		if (RobotOperationLease::IsCancellationRequested(this))
		{
			return failRunPrepare("安全停止后取消等待连续运行模式", -20000);
		}
		programModeAfter = static_cast<int>(getProgramMode());
		if (programModeAfter == continueProgramMode)
		{
			++stableContinueReads;
			if (stableContinueReads >= 3)
			{
				break;
			}
		}
		else
		{
			stableContinueReads = 0;
		}
		Sleep(50);
	}
	m_pRobotLog->write(
		stableContinueReads >= 3 ? LogColor::SUCCESS : LogColor::ERR,
		"STEP ContiMoveAny 连续运行模式确认 | Before=%d CommandIssued=%d CommandRet=%d After=%d StableReads=%d",
		programModeBefore,
		programModeCommandIssued ? 1 : 0,
		nPrepareRet,
		programModeAfter,
		stableContinueReads);
	if (stableContinueReads < 3)
	{
		return failRunPrepare("等待连续运行模式", programModeAfter);
	}

	if (!Prog_startRun_Py())
	{
		SetLastRobotError(GetStr("STEP连续运动失败：启动程序失败，Program=%s，%s",
			sProgramName.c_str(), GetRobotStatusText().c_str()));
		m_pRobotLog->write(LogColor::ERR, "STEP ContiMoveAny 启动程序失败：%s", sProgramName.c_str());
		return -10;
	}

	bool bStarted = false;
	bool bCompletedBeforeRunConfirmation = false;
	for (int i = 0; i < 50; ++i)
	{
		int nProgramState = -1;
		STEPROBOTSDK::MessageData message;
		bool completionWitnessed = false;
		{
			std::lock_guard<std::recursive_mutex> modeLock(m_sdkCommandMutex);
			nProgramState = m_pSTEPRobotClient != nullptr
				? static_cast<int>(m_pSTEPRobotClient->getProgramState())
				: -1;
			message = m_pSTEPRobotClient != nullptr
				? m_pSTEPRobotClient->getMessageData()
				: STEPROBOTSDK::MessageData();
			if (nProgramState == STEPROBOTSDK::eStop
				&& message.m_MessageType != STEPROBOTSDK::eError)
			{
				std::string ignoredWitnessError;
				completionWitnessed = VerifyGeneratedProgramCompletionWitnessLocked(
					ignoredWitnessError);
			}
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
		if (nProgramState == STEPROBOTSDK::eRun)
		{
			bStarted = true;
			break;
		}
		// 极短/零距离程序可能在 START 后的远端内容复核窗口内已经自然完成。
		// 只有同一工程/程序、无控制器错误且运行态令牌已锁存时才接受 eStop；
		// 随后的 CheckRobotDone 仍会执行稳定终态和同一见证确认。
		if (nProgramState == STEPROBOTSDK::eStop && completionWitnessed)
		{
			bStarted = true;
			bCompletedBeforeRunConfirmation = true;
			break;
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

	m_pRobotLog->write(LogColor::SUCCESS,
		bCompletedBeforeRunConfirmation
			? "STEP ContiMoveAny 已启动且在启动确认窗口内自然完成：%s"
			: "STEP ContiMoveAny 已启动程序：%s",
		sProgramName.c_str());

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

bool STEPRobotCtrl::TryGetIntVar(int nIndex, int& value, const char* cStrPreFix)
{
	const std::string sVarName = GetStr("%s%d", cStrPreFix, nIndex);
	const std::string sProjectName = GetUserProject();
	const std::string sProgramName = GetUserProgram();
	const int nRet = WithSdkCommand([&]() { return m_pSTEPRobotClient->VariableIntReadCmd(sProjectName, sProgramName, sVarName, value); });
	if (nRet != 0)
	{
		value = 0;
		SetLastRobotError(GetStr("STEP读取整数变量失败：%s，原因=%s", sVarName.c_str(), GetErrorText(nRet)));
		return false;
	}
	return true;
}

int STEPRobotCtrl::GetIntVar(int nIndex, const char* cStrPreFix)
{
	int value = 0;
	return TryGetIntVar(nIndex, value, cStrPreFix) ? value : 0;
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
