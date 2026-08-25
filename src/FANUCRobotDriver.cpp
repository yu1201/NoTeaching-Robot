#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>

#include "AppPaths.h"
#include "FanucTpContentIdentity.h"
#include "FANUCRobotDriver.h"
#include "FTPClient.h"
#include "RobotFtpFileTransfer.h"
#include "RobotOperationLease.h"

#include <QByteArray>
#include <QDir>
#include <QFileInfo>
#include <QString>
#include <QTemporaryDir>

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <charconv>
#include <cerrno>
#include <cctype>
#include <cstdlib>
#include <cmath>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <thread>
#include <unordered_map>
#include <vector>

#pragma comment(lib, "ws2_32.lib")

namespace
{
	const int FANUC_SOCKET_TIMEOUT_MS = 3000;
	const size_t FANUC_SOCKET_MAX_LINE_SIZE = 4096;
	const int FANUC_DEFAULT_MOTION_STATE_REG = 93;
	const int FANUC_WELD_PATH_CNT_VALUE = 50;
	const int FANUC_ACTUAL_WELD_CONTRACT_MISSING = -20;
	const int FANUC_DRY_RUN_CONTAINS_WELD_METADATA = -21;
	constexpr const char* FANUC_ACTUAL_WELD_UNAVAILABLE_REASON =
		"FANUC实际焊接已失败关闭：当前工程没有绑定经现场验证的ArcTool LS模板、"
		"起弧/焊接/过渡/收弧schedule映射和灭弧状态回读契约；当前仅允许空跑轨迹。";
	constexpr size_t FANUC_MAX_GENERATED_PROGRAM_NAME = 31;
	std::atomic<std::uint64_t> g_fanucGeneratedNameSequence{ 0 };
	std::mutex g_fanucGeneratedFilePairMutex;
	std::mutex g_fanucCompilerMutex;
	std::mutex g_fanucFixedEndpointMutexRegistryMutex;
	std::unordered_map<std::string, std::weak_ptr<std::mutex>> g_fanucFixedEndpointMutexes;

	bool FanucRequest(
		FANUCRobotCtrl* ctrl,
		const std::string& command,
		std::string& response,
		bool* commandMayHaveBeenSent = nullptr);
	std::string FanucResponsePayload(const std::string& response);

	std::shared_ptr<std::mutex> FanucFixedEndpointTransferMutex(const std::string& endpointKey)
	{
		std::lock_guard<std::mutex> registryLock(g_fanucFixedEndpointMutexRegistryMutex);
		std::shared_ptr<std::mutex> endpointMutex = g_fanucFixedEndpointMutexes[endpointKey].lock();
		if (endpointMutex == nullptr)
		{
			endpointMutex = std::make_shared<std::mutex>();
			g_fanucFixedEndpointMutexes[endpointKey] = endpointMutex;
		}
		return endpointMutex;
	}

	std::uint64_t FanucFnv1a64(const std::string& text)
	{
		std::uint64_t value = 1469598103934665603ULL;
		for (const unsigned char ch : text)
		{
			value ^= static_cast<std::uint64_t>(ch);
			value *= 1099511628211ULL;
		}
		return value;
	}

	std::string FanucBase36(std::uint64_t value, size_t width)
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

	std::uint64_t FanucProcessNonce()
	{
		static const std::uint64_t nonce = []()
			{
				std::ostringstream identity;
				identity << GetCurrentProcessId() << '|'
					<< std::chrono::high_resolution_clock::now().time_since_epoch().count() << '|'
					<< std::chrono::steady_clock::now().time_since_epoch().count() << '|'
					<< reinterpret_cast<std::uintptr_t>(&g_fanucGeneratedNameSequence);
				return FanucFnv1a64(identity.str());
			}();
		return nonce;
	}

	std::string FanucMakeUniqueToken(const std::string& identity)
	{
		const std::uint64_t sequence = g_fanucGeneratedNameSequence.fetch_add(1, std::memory_order_relaxed) + 1;
		const std::uint64_t mixedLow = FanucProcessNonce()
			^ FanucFnv1a64(identity)
			^ (sequence * 0x9E3779B97F4A7C15ULL);
		const std::uint64_t mixedHigh = FanucFnv1a64(
			identity + "|" + std::to_string(sequence) + "|" + std::to_string(FanucProcessNonce()));
		return FanucBase36(mixedHigh, 4) + FanucBase36(mixedLow, 13);
	}

	std::string FanucControllerIdentity(const FANUCRobotCtrl* ctrl, bool includeInstance = true)
	{
		if (ctrl == nullptr)
		{
			return "FANUC|NO_CONTROLLER";
		}
		const RobotConnectionEndpoint controlEndpoint = ctrl->ControlEndpoint();
		const RobotFileTransferProfile fileTransfer = ctrl->FileTransferProfile();
		std::ostringstream identity;
		identity << "FANUC|" << ctrl->RobotName()
			<< '|' << controlEndpoint.host << ':' << controlEndpoint.port
			<< '|' << fileTransfer.endpointDisplay;
		if (includeInstance)
		{
			identity << "|PID=" << GetCurrentProcessId()
				<< "|INSTANCE=" << reinterpret_cast<std::uintptr_t>(ctrl);
		}
		return identity.str();
	}

	std::filesystem::path FanucGeneratedProgramDirectory(const FANUCRobotCtrl* ctrl)
	{
		const std::string identity = FanucControllerIdentity(ctrl);
		std::ostringstream folder;
		folder << "endpoint_" << std::uppercase << std::hex << std::setw(16) << std::setfill('0')
			<< FanucFnv1a64(identity)
			<< "_p" << std::dec << GetCurrentProcessId()
			<< "_i" << std::uppercase << std::hex << reinterpret_cast<std::uintptr_t>(ctrl);
		const QString basePath = QDir::toNativeSeparators(
			AppPaths::WritablePath(QStringLiteral("Job/FANUC")));
		return std::filesystem::path(basePath.toStdWString()) / folder.str();
	}

	QString FanucDecodeLocalPath(const std::string& text)
	{
		return QString::fromLocal8Bit(text.data(), static_cast<int>(text.size()));
	}

	QString FanucDecodeConfigText(const std::string& text)
	{
		QString value = QString::fromUtf8(text.data(), static_cast<int>(text.size()));
		if (value.contains(QChar(0xfffd)))
		{
			value = FanucDecodeLocalPath(text);
		}
		return value;
	}

	std::filesystem::path FanucFileSystemPath(const QString& path)
	{
		return std::filesystem::path(QDir::toNativeSeparators(path).toStdWString());
	}

	std::string FanucLocalPathBytes(const QString& path)
	{
		const QByteArray bytes = QDir::toNativeSeparators(path).toLocal8Bit();
		return std::string(bytes.constData(), static_cast<size_t>(bytes.size()));
	}

	std::string FanucLocalPathBytes(const std::filesystem::path& path)
	{
		return FanucLocalPathBytes(QString::fromStdWString(path.wstring()));
	}

	std::filesystem::path FanucResolveLocalPath(const std::string& pathText)
	{
		const QString decoded = FanucDecodeLocalPath(pathText);
		const QFileInfo info(QDir::fromNativeSeparators(decoded));
		const QString absolute = info.isAbsolute()
			? info.absoluteFilePath()
			: AppPaths::CommandLinePath(decoded);
		return FanucFileSystemPath(absolute);
	}

	bool FanucReadFileIdentity(
		const std::string& filePath,
		FanucTpContentIdentity::Identity& identity)
	{
		return FanucTpContentIdentity::Read(FanucResolveLocalPath(filePath), identity);
	}

	bool FanucContainsWeldMetadata(const T_ROBOT_MOVE_INFO& info)
	{
		const auto hasProcessNumber = [](double value)
			{
				return !std::isfinite(value) || std::abs(value) > 1e-9;
			};
		return info.bWeldProcessEnabled
			|| info.bArcStartBeforeMove
			|| info.bArcEndAfterMove
			|| info.bUseTransitionWeldParams
			|| info.bHasWeaveParam
			|| info.bAppPointwiseWeave
			|| info.bHasTrackParam
			|| hasProcessNumber(info.dArcStartCurrent)
			|| hasProcessNumber(info.dArcStartVoltage)
			|| hasProcessNumber(info.dArcStartWaitTime)
			|| hasProcessNumber(info.dWeldCurrent)
			|| hasProcessNumber(info.dWeldVoltage)
			|| hasProcessNumber(info.dWeldSpeedMmPerMin)
			|| hasProcessNumber(info.dArcEndCurrent)
			|| hasProcessNumber(info.dArcEndVoltage)
			|| hasProcessNumber(info.dArcEndWaitTime);
	}

	bool FanucContainsExternalAxisTarget(const T_ROBOT_MOVE_INFO& info)
	{
		if (info.nMoveType == MOVL)
		{
			return std::abs(info.tCoord.dBX) > 1e-9
				|| std::abs(info.tCoord.dBY) > 1e-9
				|| std::abs(info.tCoord.dBZ) > 1e-9;
		}
		return info.tPulse.lBXPulse != 0
			|| info.tPulse.lBYPulse != 0
			|| info.tPulse.lBZPulse != 0;
	}

	long long FanucElapsedMs(std::chrono::steady_clock::time_point start)
	{
		return std::chrono::duration_cast<std::chrono::milliseconds>(
			std::chrono::steady_clock::now() - start).count();
	}

	bool FanucInitWinSock()
	{
		WSADATA wsaData = {};
		return WSAStartup(MAKEWORD(2, 2), &wsaData) == 0;
	}

	void FanucCleanupWinSock()
	{
		WSACleanup();
	}

	SOCKET FanucInvalidSocket()
	{
		return INVALID_SOCKET;
	}

	SOCKET FanucGetSocket(std::uintptr_t handle)
	{
		return static_cast<SOCKET>(handle);
	}

	bool FanucSetSocketTimeout(SOCKET sock, int timeoutMs)
	{
		const DWORD timeout = static_cast<DWORD>(timeoutMs);
		return setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&timeout), sizeof(timeout)) != SOCKET_ERROR &&
			setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, reinterpret_cast<const char*>(&timeout), sizeof(timeout)) != SOCKET_ERROR;
	}

	bool FanucConnectSocket(SOCKET& sock, const std::string& ip, unsigned short port)
	{
		sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		if (sock == INVALID_SOCKET)
		{
			return false;
		}

		sockaddr_in addr = {};
		addr.sin_family = AF_INET;
		addr.sin_port = htons(port);
		if (inet_pton(AF_INET, ip.c_str(), &addr.sin_addr) != 1)
		{
			closesocket(sock);
			sock = INVALID_SOCKET;
			return false;
		}

		if (connect(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == SOCKET_ERROR)
		{
			closesocket(sock);
			sock = INVALID_SOCKET;
			return false;
		}

		FanucSetSocketTimeout(sock, FANUC_SOCKET_TIMEOUT_MS);
		return true;
	}

	bool FanucSendLine(SOCKET sock, const std::string& text)
	{
		const std::string payload = text + "\n";
		int totalSent = 0;
		const int totalSize = static_cast<int>(payload.size());
		while (totalSent < totalSize)
		{
			const int sent = send(sock, payload.c_str() + totalSent, totalSize - totalSent, 0);
			if (sent == SOCKET_ERROR)
			{
				return false;
			}
			totalSent += sent;
		}
		return true;
	}

	bool FanucWaitReadable(SOCKET sock, int timeoutMs)
	{
		fd_set readSet;
		FD_ZERO(&readSet);
		FD_SET(sock, &readSet);

		timeval tv = {};
		tv.tv_sec = timeoutMs / 1000;
		tv.tv_usec = (timeoutMs % 1000) * 1000;

		const int result = select(0, &readSet, nullptr, nullptr, &tv);
		return result > 0 && FD_ISSET(sock, &readSet);
	}

	bool FanucReceiveLine(SOCKET sock, std::string& out, int timeoutMs = FANUC_SOCKET_TIMEOUT_MS)
	{
		out.clear();
		if (timeoutMs <= 0)
		{
			return false;
		}

		// The timeout is a deadline for the complete line, not an idle timeout that
		// restarts after every byte.  Otherwise a peer can keep this worker alive
		// indefinitely by trickling one byte before each select() timeout.
		const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
		size_t rawBytesReceived = 0;
		char ch = 0;

		while (true)
		{
			const auto now = std::chrono::steady_clock::now();
			if (now >= deadline)
			{
				return false;
			}

			const auto remainingMilliseconds =
				std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count();
			const int remainingTimeoutMs = static_cast<int>(std::max<std::int64_t>(1, remainingMilliseconds));
			if (!FanucWaitReadable(sock, remainingTimeoutMs))
			{
				return false;
			}

			const int recved = recv(sock, &ch, 1, 0);
			if (recved <= 0)
			{
				return false;
			}
			++rawBytesReceived;
			if (rawBytesReceived > FANUC_SOCKET_MAX_LINE_SIZE)
			{
				return false;
			}

			if (ch == '\n')
			{
				return true;
			}

			if (ch != '\r')
			{
				out.push_back(ch);
			}
		}
	}

	std::vector<std::string> FanucSplit(const std::string& text, char delimiter)
	{
		std::vector<std::string> parts;
		std::stringstream ss(text);
		std::string item;
		while (std::getline(ss, item, delimiter))
		{
			parts.push_back(item);
		}
		return parts;
	}

	std::string FanucJoin(const std::vector<std::string>& parts, char delimiter)
	{
		std::ostringstream oss;
		for (size_t i = 0; i < parts.size(); ++i)
		{
			if (i > 0)
			{
				oss << delimiter;
			}
			oss << parts[i];
		}
		return oss.str();
	}

	bool FanucStartsWith(const std::string& text, const std::string& prefix)
	{
		return text.size() >= prefix.size() && text.compare(0, prefix.size(), prefix) == 0;
	}

	bool FanucParseDoubles(const std::string& text, double* values, size_t count)
	{
		if (values == nullptr)
		{
			return false;
		}

		const std::vector<std::string> parts = FanucSplit(text, ',');
		if (parts.size() < count)
		{
			return false;
		}

		for (size_t i = 0; i < count; ++i)
		{
			const std::string& part = parts[i];
			const char* first = part.c_str();
			while (*first != '\0' && std::isspace(static_cast<unsigned char>(*first)))
			{
				++first;
			}
			errno = 0;
			char* end = nullptr;
			const double parsed = std::strtod(first, &end);
			while (end != nullptr && *end != '\0'
				&& std::isspace(static_cast<unsigned char>(*end)))
			{
				++end;
			}
			if (first == end || end == nullptr || *end != '\0'
				|| errno == ERANGE || !std::isfinite(parsed))
			{
				return false;
			}
			values[i] = parsed;
		}
		return true;
	}

	bool FanucParseLongs(const std::string& text, long* values, size_t count)
	{
		if (values == nullptr)
		{
			return false;
		}

		const std::vector<std::string> parts = FanucSplit(text, ',');
		if (parts.size() < count)
		{
			return false;
		}

		for (size_t i = 0; i < count; ++i)
		{
			const std::string& part = parts[i];
			const char* first = part.c_str();
			while (*first != '\0' && std::isspace(static_cast<unsigned char>(*first)))
			{
				++first;
			}
			errno = 0;
			char* end = nullptr;
			const long parsed = std::strtol(first, &end, 10);
			while (end != nullptr && *end != '\0'
				&& std::isspace(static_cast<unsigned char>(*end)))
			{
				++end;
			}
			if (first == end || end == nullptr || *end != '\0' || errno == ERANGE)
			{
				return false;
			}
			values[i] = parsed;
		}
		return true;
	}

	long long FanucSteadyMs()
	{
		return std::chrono::duration_cast<std::chrono::milliseconds>(
			std::chrono::steady_clock::now().time_since_epoch()).count();
	}

	long FanucPositionToPulse(double position, double pulseUnit)
	{
		if (!std::isfinite(position) || !std::isfinite(pulseUnit)
			|| std::abs(pulseUnit) < 1e-15)
		{
			return 0;
		}
		const double scaled = position / pulseUnit;
		if (!std::isfinite(scaled)
			|| scaled < static_cast<double>(std::numeric_limits<long>::lowest())
			|| scaled > static_cast<double>(std::numeric_limits<long>::max()))
		{
			return 0;
		}
		return static_cast<long>(std::lround(scaled));
	}

	std::string FanucBuildConfigText(const int config[7])
	{
		if (config == nullptr)
		{
			return "0,0,0,0,0,0,0";
		}

		std::vector<std::string> parts;
		for (int i = 0; i < 7; ++i)
		{
			parts.push_back(std::to_string(config[i]));
		}
		return FanucJoin(parts, ',');
	}

	std::string FanucMakeTimestamp()
	{
		SYSTEMTIME st = {};
		GetLocalTime(&st);
		return GetStr("%04d%02d%02d_%02d%02d%02d",
			st.wYear, st.wMonth, st.wDay,
			st.wHour, st.wMinute, st.wSecond);
	}

	std::string FanucMakeControllerProgramName(const FANUCRobotCtrl* ctrl, const char* prefix)
	{
		const std::string safePrefix = prefix != nullptr && prefix[0] != '\0' ? prefix : "F";
		std::string programName = safePrefix + FanucMakeUniqueToken(FanucControllerIdentity(ctrl));
		if (programName.size() > FANUC_MAX_GENERATED_PROGRAM_NAME)
		{
			programName.resize(FANUC_MAX_GENERATED_PROGRAM_NAME);
		}
		return programName;
	}

	std::string FanucMakeProgramName(const FANUCRobotCtrl* ctrl)
	{
		return FanucMakeControllerProgramName(ctrl, "FK");
	}

	std::string FanucMakeTpProgramName(const FANUCRobotCtrl* ctrl)
	{
		return FanucMakeControllerProgramName(ctrl, "FT");
	}

	bool FanucIsValidControllerProgramName(const std::string& programName)
	{
		if (programName.empty() || programName.size() > FANUC_MAX_GENERATED_PROGRAM_NAME
			|| std::isalpha(static_cast<unsigned char>(programName.front())) == 0)
		{
			return false;
		}
		return std::all_of(
			programName.begin(),
			programName.end(),
			[](unsigned char ch)
			{
				return std::isalnum(ch) != 0 || ch == '_';
			});
	}

	std::string FanucMoveTypeText(int moveType)
	{
		return moveType == MOVL ? "MOVL" : "MOVJ";
	}

	int FanucMoveTypeFromJobName(const std::string& jobName)
	{
		return jobName == "MOVL" ? MOVL : MOVJ;
	}

	double FanucPulseToPosition(long pulse, double pulseUnit)
	{
		return static_cast<double>(pulse) * pulseUnit;
	}

	int FanucSpeedPercent(double speed)
	{
		if (!std::isfinite(speed) || speed <= 0.0)
		{
			return 0;
		}

		// FANUC项目里同时存在两种传参口径：
		// 1. 直接传百分比，例如 1 / 5 / 20
		// 2. 配置文件里按百分比*100保存，例如 2000 表示 20%
		// 这里统一兼容，避免 MOVJ 安全姿态把 2000 误解释成 100%。
		double percent = speed;
		if (percent > 100.0)
		{
			percent /= 100.0;
		}
		if (percent < 1.0)
		{
			return 0;
		}
		if (percent > 100.0)
		{
			return 100;
		}
		return static_cast<int>(std::floor(percent));
	}

	bool FanucLinearSpeedRegister(double speed, int& speedRegister)
	{
		speedRegister = 0;
		if (!std::isfinite(speed) || speed < 1.0
			|| speed > static_cast<double>(std::numeric_limits<int>::max()))
		{
			return false;
		}
		speedRegister = static_cast<int>(std::floor(speed));
		return speedRegister >= 1;
	}

	std::string FanucBuildLsHeader(const std::string& programName, size_t lineCount, const char* comment)
	{
		SYSTEMTIME st = {};
		GetLocalTime(&st);

		std::ostringstream oss;
		oss << "/PROG  " << programName << "\n";
		oss << "/ATTR" << "\n";
		oss << "OWNER\t\t= MNEDITOR;" << "\n";
		oss << "COMMENT\t\t= \"" << comment << "\";" << "\n";
		oss << "PROG_SIZE\t= 0;" << "\n";
		oss << "CREATE\t\t= DATE " << GetStr("%02d-%02d-%02d", st.wYear % 100, st.wMonth, st.wDay)
			<< "  TIME " << GetStr("%02d:%02d:%02d", st.wHour, st.wMinute, st.wSecond) << ";" << "\n";
		oss << "MODIFIED\t= DATE " << GetStr("%02d-%02d-%02d", st.wYear % 100, st.wMonth, st.wDay)
			<< "  TIME " << GetStr("%02d:%02d:%02d", st.wHour, st.wMinute, st.wSecond) << ";" << "\n";
		oss << "FILE_NAME\t= ;" << "\n";
		oss << "VERSION\t\t= 0;" << "\n";
		oss << "LINE_COUNT\t= " << lineCount << ";" << "\n";
		oss << "MEMORY_SIZE\t= 0;" << "\n";
		oss << "PROTECT\t\t= READ_WRITE;" << "\n";
		oss << "TCD:  STACK_SIZE\t= 0," << "\n";
		oss << "      TASK_PRIORITY\t= 50," << "\n";
		oss << "      TIME_SLICE\t= 0," << "\n";
		oss << "      BUSY_LAMP_OFF\t= 0," << "\n";
		oss << "      ABORT_REQUEST\t= 0," << "\n";
		oss << "      PAUSE_REQUEST\t= 0;" << "\n";
		oss << "DEFAULT_GROUP\t= 1,*,*,*,*;" << "\n";
		oss << "CONTROL_CODE\t= 00000000 00000000;" << "\n";
		return oss.str();
	}

	std::string FanucBuildCartesianPoint(const T_ROBOT_COORS& coord, size_t index)
	{
		std::ostringstream oss;
		oss << std::fixed << std::setprecision(3);
		oss << "P[" << index << "]{" << "\n";
		oss << "   GP1:" << "\n";
		oss << "\tUF : 0, UT : 1,\t\tCONFIG : 'N U T, 0, 0, 0'," << "\n";
		oss << "\tX = " << coord.dX << "  mm,\tY = " << coord.dY << "  mm,\tZ = " << coord.dZ << "  mm," << "\n";
		oss << "\tW = " << coord.dRX << " deg,\tP = " << coord.dRY << " deg,\tR = " << coord.dRZ << " deg" << "\n";
		oss << "};" << "\n";
		return oss.str();
	}

	std::string FanucBuildJointPoint(const T_ANGLE_PULSE& pulse, const T_AXISUNIT& axisUnit, size_t index)
	{
		std::ostringstream oss;
		oss << std::fixed << std::setprecision(3);
		oss << "P[" << index << "]{" << "\n";
		oss << "   GP1:" << "\n";
		oss << "\tUF : 0, UT : 1," << "\n";
		oss << "\tJ1 = " << FanucPulseToPosition(pulse.nSPulse, axisUnit.dSPulseUnit) << " deg,\t"
			<< "J2 = " << FanucPulseToPosition(pulse.nLPulse, axisUnit.dLPulseUnit) << " deg,\t"
			<< "J3 = " << FanucPulseToPosition(pulse.nUPulse, axisUnit.dUPulseUnit) << " deg," << "\n";
		oss << "\tJ4 = " << FanucPulseToPosition(pulse.nRPulse, axisUnit.dRPulseUnit) << " deg,\t"
			<< "J5 = " << FanucPulseToPosition(pulse.nBPulse, axisUnit.dBPulseUnit) << " deg,\t"
			<< "J6 = " << FanucPulseToPosition(pulse.nTPulse, axisUnit.dTPulseUnit) << " deg" << "\n";
		oss << "};" << "\n";
		return oss.str();
	}

	std::string FanucBuildTpMoveLsContent(
		const std::string& programName,
		const std::vector<T_ROBOT_MOVE_INFO>& moveInfos,
		const T_AXISUNIT& axisUnit)
	{
		const size_t lineCount = moveInfos.size() + 5;
		std::ostringstream oss;
		oss << FanucBuildLsHeader(programName, lineCount, "Auto FANUC move");
		oss << "/MN" << "\n";
		oss << "   1:  UFRAME_NUM=0 ;" << "\n";
		oss << "   2:  UTOOL_NUM=1 ;" << "\n";
		oss << GetStr("%4u:  R[%d]=10 ;",
			static_cast<unsigned>(3),
			FANUC_DEFAULT_MOTION_STATE_REG) << "\n";

		for (size_t i = 0; i < moveInfos.size(); ++i)
		{
			const T_ROBOT_MOVE_INFO& info = moveInfos[i];
			const size_t pointIndex = i + 1;
			const size_t lineIndex = i + 4;
			// 搭接台阶等零过渡点必须精确到位；普通点继续沿用现场既有 CNT50，
			// 避免把 STEP 的 OVERLAPREL 数值直接误当作 FANUC CNT 等级。
			const bool useFine = (i == 0)
				|| (i + 1 == moveInfos.size())
				|| !std::isfinite(info.dOverlapRel)
				|| info.dOverlapRel <= 0.0;
			const std::string termination = useFine
				? "FINE"
				: GetStr("CNT%d", FANUC_WELD_PATH_CNT_VALUE);
			if (info.nMoveType == MOVL)
			{
				int linearSpeed = 0;
				FanucLinearSpeedRegister(info.tSpeed.dSpeed, linearSpeed);
				oss << GetStr("%4u:  L P[%u] %dmm/sec %s ;",
					static_cast<unsigned>(lineIndex),
					static_cast<unsigned>(pointIndex),
					linearSpeed,
					termination.c_str()) << "\n";
			}
			else
			{
				oss << GetStr("%4u:  J P[%u] %d%% %s ;",
					static_cast<unsigned>(lineIndex),
					static_cast<unsigned>(pointIndex),
					FanucSpeedPercent(info.tSpeed.dSpeed),
					termination.c_str()) << "\n";
			}
		}

		oss << GetStr("%4u:  R[%d]=1 ;",
			static_cast<unsigned>(moveInfos.size() + 4),
			FANUC_DEFAULT_MOTION_STATE_REG) << "\n";
		oss << GetStr("%4u:  END ;", static_cast<unsigned>(moveInfos.size() + 5)) << "\n";
		oss << "/POS" << "\n";
		for (size_t i = 0; i < moveInfos.size(); ++i)
		{
			if (moveInfos[i].nMoveType == MOVL)
			{
				oss << FanucBuildCartesianPoint(moveInfos[i].tCoord, i + 1);
			}
			else
			{
				oss << FanucBuildJointPoint(moveInfos[i].tPulse, axisUnit, i + 1);
			}
		}
		oss << "/END" << "\n";
		return oss.str();
	}

	void FanucLogMovePoint(RobotDriverAdaptor* driver, const char* prefix, int index, const T_ROBOT_MOVE_INFO& info, const T_AXISUNIT* axisUnit = nullptr)
	{
		if (driver == nullptr || prefix == nullptr)
		{
			return;
		}

		if (info.nMoveType == MOVL)
		{
			driver->WriteLog(LogColor::DEFAULT,
				"%s MOVL P[%d]: X=%.3f Y=%.3f Z=%.3f RX=%.3f RY=%.3f RZ=%.3f BX=%.3f BY=%.3f BZ=%.3f Speed=%.3f",
				prefix, index,
				info.tCoord.dX, info.tCoord.dY, info.tCoord.dZ,
				info.tCoord.dRX, info.tCoord.dRY, info.tCoord.dRZ,
				info.tCoord.dBX, info.tCoord.dBY, info.tCoord.dBZ,
				info.tSpeed.dSpeed);
			return;
		}

		if (axisUnit != nullptr)
		{
			driver->WriteLog(LogColor::DEFAULT,
				"%s MOVJ P[%d]: J1=%.3f J2=%.3f J3=%.3f J4=%.3f J5=%.3f J6=%.3f Pulse=(%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld) Speed=%.3f",
				prefix, index,
				FanucPulseToPosition(info.tPulse.nSPulse, axisUnit->dSPulseUnit),
				FanucPulseToPosition(info.tPulse.nLPulse, axisUnit->dLPulseUnit),
				FanucPulseToPosition(info.tPulse.nUPulse, axisUnit->dUPulseUnit),
				FanucPulseToPosition(info.tPulse.nRPulse, axisUnit->dRPulseUnit),
				FanucPulseToPosition(info.tPulse.nBPulse, axisUnit->dBPulseUnit),
				FanucPulseToPosition(info.tPulse.nTPulse, axisUnit->dTPulseUnit),
				info.tPulse.nSPulse, info.tPulse.nLPulse, info.tPulse.nUPulse,
				info.tPulse.nRPulse, info.tPulse.nBPulse, info.tPulse.nTPulse,
				info.tPulse.lBXPulse, info.tPulse.lBYPulse, info.tPulse.lBZPulse,
				info.tSpeed.dSpeed);
		}
		else
		{
			driver->WriteLog(LogColor::DEFAULT,
				"%s MOVJ P[%d]: Pulse=(%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld) Speed=%.3f",
				prefix, index,
				info.tPulse.nSPulse, info.tPulse.nLPulse, info.tPulse.nUPulse,
				info.tPulse.nRPulse, info.tPulse.nBPulse, info.tPulse.nTPulse,
				info.tPulse.lBXPulse, info.tPulse.lBYPulse, info.tPulse.lBZPulse,
				info.tSpeed.dSpeed);
		}
	}

	std::string FanucBuildVarContent(const std::string& programName, const std::vector<T_ROBOT_MOVE_INFO>& moveInfos)
	{
		std::ostringstream oss;
		oss << "# FANUC continuous move data" << "\n";
		oss << "PROGRAM=" << programName << "\n";
		oss << "POINT_COUNT=" << moveInfos.size() << "\n";
		oss << std::fixed << std::setprecision(3);

		for (size_t i = 0; i < moveInfos.size(); ++i)
		{
			const T_ROBOT_MOVE_INFO& info = moveInfos[i];
			const size_t pointIndex = i + 1;

			oss << "\n";
			oss << "[POINT" << pointIndex << "]" << "\n";
			oss << "MOVE_TYPE=" << FanucMoveTypeText(info.nMoveType) << "\n";
			oss << "MOVE_DEVICE=" << info.nMoveDevice << "\n";
			oss << "TRACK_NO=" << info.nTrackNo << "\n";
			oss << "SPEED=" << info.tSpeed.dSpeed << "\n";
			oss << "ACC=" << info.tSpeed.dACC << "\n";
			oss << "DEC=" << info.tSpeed.dDEC << "\n";
			oss << "COORD="
				<< info.tCoord.dX << ","
				<< info.tCoord.dY << ","
				<< info.tCoord.dZ << ","
				<< info.tCoord.dRX << ","
				<< info.tCoord.dRY << ","
				<< info.tCoord.dRZ << "\n";
			oss << "PULSE="
				<< info.tPulse.nSPulse << ","
				<< info.tPulse.nLPulse << ","
				<< info.tPulse.nUPulse << ","
				<< info.tPulse.nRPulse << ","
				<< info.tPulse.nBPulse << ","
				<< info.tPulse.nTPulse << ","
				<< info.tPulse.lBXPulse << ","
				<< info.tPulse.lBYPulse << ","
				<< info.tPulse.lBZPulse << "\n";
			oss << "BASE="
				<< info.adBasePosVar[0] << ","
				<< info.adBasePosVar[1] << ","
				<< info.adBasePosVar[2] << "\n";
		}

		return oss.str();
	}

	std::string FanucBuildKlContent(const std::string& programName, const std::string& varFileName, const std::vector<T_ROBOT_MOVE_INFO>& moveInfos)
	{
		std::ostringstream oss;
		oss << "PROGRAM " << programName << "\n";
		oss << "%COMMENT = 'Auto generated by QtWidgetsApplication4'" << "\n";
		oss << "%NOLOCKGROUP" << "\n";
		oss << "\n";
		oss << "CONST" << "\n";
		oss << "   VAR_FILE_NAME = '" << varFileName << "'" << "\n";
		oss << "   POINT_COUNT = " << moveInfos.size() << "\n";
		oss << "\n";
		oss << "VAR" << "\n";
		oss << "   idx : INTEGER" << "\n";
		oss << "\n";
		oss << "BEGIN" << "\n";
		oss << "   WRITE('AUTO GENERATED FANUC PATH', CR)" << "\n";
		oss << "   WRITE('VAR FILE: ', VAR_FILE_NAME, CR)" << "\n";
		oss << "   WRITE('POINT COUNT: ', POINT_COUNT, CR)" << "\n";
		oss << "   FOR idx = 1 TO POINT_COUNT DO" << "\n";
		oss << "      WRITE('POINT ', idx, ' READY', CR)" << "\n";
		oss << "   ENDFOR" << "\n";
		oss << "END " << programName << "\n";
		oss << "\n";
		oss << "-- Path preview generated from host application" << "\n";

		oss << std::fixed << std::setprecision(3);
		for (size_t i = 0; i < moveInfos.size(); ++i)
		{
			const T_ROBOT_MOVE_INFO& info = moveInfos[i];
			oss << "-- P" << (i + 1)
				<< " " << FanucMoveTypeText(info.nMoveType)
				<< " SPD=" << info.tSpeed.dSpeed
				<< " XYZWPR=("
				<< info.tCoord.dX << ","
				<< info.tCoord.dY << ","
				<< info.tCoord.dZ << ","
				<< info.tCoord.dRX << ","
				<< info.tCoord.dRY << ","
				<< info.tCoord.dRZ << ")"
				<< "\n";
		}

		return oss.str();
	}

	bool FanucWriteTextFile(const std::string& filePath, const std::string& content)
	{
		const std::filesystem::path resolvedPath = FanucResolveLocalPath(filePath);
		for (int attempt = 0; attempt < 8; ++attempt)
		{
			const std::string temporaryToken = FanucMakeUniqueToken(filePath);
			std::filesystem::path temporaryPath = resolvedPath;
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
				resolvedPath.c_str(),
				MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH))
			{
				return true;
			}
			DeleteFileW(temporaryPath.c_str());
			return false;
		}
		return false;
	}

	std::filesystem::path FanucFindCompilerToolPath(const std::string& fileName)
	{
		const QString fileNameText = QString::fromLatin1(fileName.data(), static_cast<int>(fileName.size()));
		std::vector<std::filesystem::path> candidates =
		{
			FanucFileSystemPath(AppPaths::ResourcePath(
				QStringLiteral("Tools/FANUC/WinOLPC/bin/") + fileNameText)),
			FanucFileSystemPath(AppPaths::ResourcePath(
				QStringLiteral("SDK/FANUC/WinOLPC/bin/") + fileNameText)),
			std::filesystem::path(L"C:\\Program Files (x86)\\FANUC\\WinOLPC\\bin") / fileNameText.toStdWString(),
			std::filesystem::path(L"C:\\Program Files\\FANUC\\WinOLPC\\bin") / fileNameText.toStdWString()
		};

		for (const auto& candidate : candidates)
		{
			std::error_code ec;
			if (std::filesystem::exists(candidate, ec))
			{
				return candidate;
			}
		}

		return candidates.front();
	}

	std::filesystem::path FanucGetKtransPath()
	{
		return FanucFindCompilerToolPath("ktrans.exe");
	}

	std::filesystem::path FanucGetMaketpPath()
	{
		return FanucFindCompilerToolPath("maketp.exe");
	}

	bool FanucFileExists(const std::filesystem::path& filePath)
	{
		std::error_code ec;
		return std::filesystem::exists(filePath, ec);
	}

	bool FanucFileExists(const std::string& filePath)
	{
		return FanucFileExists(FanucResolveLocalPath(filePath));
	}

	std::filesystem::path FanucAbsolutePath(const std::string& filePath)
	{
		return FanucResolveLocalPath(filePath);
	}

	std::string FanucTrim(const std::string& text)
	{
		const size_t begin = text.find_first_not_of(" \t\r\n");
		if (begin == std::string::npos)
		{
			return std::string();
		}
		const size_t end = text.find_last_not_of(" \t\r\n");
		return text.substr(begin, end - begin + 1);
	}

	bool FanucParseIntStrict(const std::string& text, int& value)
	{
		const std::string trimmed = FanucTrim(text);
		if (trimmed.empty())
		{
			return false;
		}
		int parsed = 0;
		const char* begin = trimmed.data();
		const char* end = begin + trimmed.size();
		const std::from_chars_result result = std::from_chars(begin, end, parsed);
		if (result.ec != std::errc() || result.ptr != end)
		{
			return false;
		}
		value = parsed;
		return true;
	}

	struct FanucTaskSnapshot
	{
		std::string programName;
		int status = -999;
		int line = -1;
	};

	bool FanucParseTaskSnapshot(const std::string& response, FanucTaskSnapshot& snapshot)
	{
		if (!FanucStartsWith(response, "TASK:"))
		{
			return false;
		}
		const std::vector<std::string> parts = FanucSplit(response.substr(5), ',');
		if (parts.size() != 3 || parts[0].empty()
			|| !FanucParseIntStrict(parts[1], snapshot.status)
			|| !FanucParseIntStrict(parts[2], snapshot.line))
		{
			return false;
		}
		snapshot.programName = parts[0];
		return true;
	}

	bool FanucPoseIsFinite(const T_ROBOT_COORS& pose)
	{
		return std::isfinite(pose.dX) && std::isfinite(pose.dY) && std::isfinite(pose.dZ)
			&& std::isfinite(pose.dRX) && std::isfinite(pose.dRY) && std::isfinite(pose.dRZ);
	}

	double FanucPositionDeviationMm(const T_ROBOT_COORS& left, const T_ROBOT_COORS& right)
	{
		const double dx = left.dX - right.dX;
		const double dy = left.dY - right.dY;
		const double dz = left.dZ - right.dZ;
		return std::sqrt(dx * dx + dy * dy + dz * dz);
	}

	double FanucAngleDeviationDeg(const T_ROBOT_COORS& left, const T_ROBOT_COORS& right)
	{
		const auto delta = [](double a, double b)
			{
				return std::abs(std::remainder(a - b, 360.0));
			};
		return (std::max)({
			delta(left.dRX, right.dRX),
			delta(left.dRY, right.dRY),
			delta(left.dRZ, right.dRZ) });
	}

	bool FanucRotationMatrixIsValid(const double rotation[9])
	{
		if (rotation == nullptr)
		{
			return false;
		}
		for (int index = 0; index < 9; ++index)
		{
			if (!std::isfinite(rotation[index]))
			{
				return false;
			}
		}
		for (int row = 0; row < 3; ++row)
		{
			double normSquared = 0.0;
			for (int column = 0; column < 3; ++column)
			{
				normSquared += rotation[row * 3 + column] * rotation[row * 3 + column];
			}
			if (std::abs(normSquared - 1.0) > 0.05)
			{
				return false;
			}
		}
		for (int firstRow = 0; firstRow < 3; ++firstRow)
		{
			for (int secondRow = firstRow + 1; secondRow < 3; ++secondRow)
			{
				double dot = 0.0;
				for (int column = 0; column < 3; ++column)
				{
					dot += rotation[firstRow * 3 + column] * rotation[secondRow * 3 + column];
				}
				if (std::abs(dot) > 0.05)
				{
					return false;
				}
			}
		}
		const double determinant =
			rotation[0] * (rotation[4] * rotation[8] - rotation[5] * rotation[7])
			- rotation[1] * (rotation[3] * rotation[8] - rotation[5] * rotation[6])
			+ rotation[2] * (rotation[3] * rotation[7] - rotation[4] * rotation[6]);
		return std::isfinite(determinant) && std::abs(determinant - 1.0) <= 0.05;
	}

	std::string FanucToLower(std::string text)
	{
		std::transform(text.begin(), text.end(), text.begin(),
			[](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
		return text;
	}

	std::filesystem::path FanucReadWinOlpcOutputDir()
	{
		const std::filesystem::path robotIniPath = FanucFindCompilerToolPath("robot.ini");
		std::ifstream input(robotIniPath);
		if (!input.is_open())
		{
			return std::filesystem::path();
		}

		std::string line;
		while (std::getline(input, line))
		{
			if (!FanucStartsWith(line, "Output="))
			{
				continue;
			}

			const std::string outputDir = FanucTrim(line.substr(std::string("Output=").size()));
			if (!outputDir.empty())
			{
				const std::filesystem::path configuredPath = FanucFileSystemPath(
					FanucDecodeLocalPath(outputDir));
				return configuredPath.is_absolute()
					? configuredPath
					: (robotIniPath.parent_path() / configuredPath).lexically_normal();
			}
		}

		return std::filesystem::path();
	}

	std::filesystem::path FanucFindCompiledTpInOutputDir(const std::filesystem::path& requestedTpPath)
	{
		const std::filesystem::path outputDir = FanucReadWinOlpcOutputDir();
		std::error_code ec;
		if (outputDir.empty() || !std::filesystem::exists(outputDir, ec))
		{
			return std::filesystem::path();
		}

		const std::string wantedStem = FanucToLower(requestedTpPath.stem().string());
		for (const auto& entry : std::filesystem::directory_iterator(outputDir, ec))
		{
			if (ec || !entry.is_regular_file())
			{
				continue;
			}

			const std::filesystem::path candidate = entry.path();
			if (FanucToLower(candidate.stem().string()) == wantedStem
				&& FanucToLower(candidate.extension().string()) == ".tp")
			{
				return candidate;
			}
		}

		return std::filesystem::path();
	}

	std::string FanucBuildProgramPath(const std::string& unitName, const std::string& fileName)
	{
		const QString unitNameText = FanucDecodeConfigText(unitName);
		const QString fileNameText = FanucDecodeConfigText(fileName);
		const std::vector<QString> candidates =
		{
			AppPaths::WritablePath(QStringLiteral("Result/%1/%2").arg(unitNameText, fileNameText)),
			AppPaths::WritablePath(QStringLiteral("Job/FANUC/%1").arg(fileNameText)),
			AppPaths::FindResourcePath(QStringLiteral("SDK/FANUC/%1").arg(fileNameText))
		};
		for (const QString& path : candidates)
		{
			if (!path.isEmpty() && QFileInfo::exists(path))
			{
				return FanucLocalPathBytes(path);
			}
		}
		return std::string();
	}

	std::string FanucFixedMoveProgramName(int moveType)
	{
		return moveType == MOVL ? "FANUC_MOVL" : "FANUC_MOVJ";
	}

	std::string FanucFixedMoveTpPath(const std::string& unitName, int moveType)
	{
		return FanucBuildProgramPath(unitName, FanucFixedMoveProgramName(moveType) + ".tp");
	}

	bool FanucCompileKlToPc(const std::string& klPath, const std::string& pcPath, RobotLog* pLog)
	{
		// WinOLPC tools share robot.ini/output state and are not safe to run concurrently.
		std::lock_guard<std::mutex> compilerLock(g_fanucCompilerMutex);
		const auto compileStart = std::chrono::steady_clock::now();
		const std::filesystem::path ktransPath = FanucGetKtransPath();
		const std::filesystem::path absoluteKlPath = FanucAbsolutePath(klPath);
		const std::filesystem::path absolutePcPath = FanucAbsolutePath(pcPath);
		if (!FanucFileExists(ktransPath))
		{
			if (pLog != nullptr)
			{
				const std::string ktransPathText = FanucLocalPathBytes(ktransPath);
				pLog->write(LogColor::ERR, "FANUC 编译失败：未找到 ktrans.exe，路径=%s | 耗时=%lldms",
					ktransPathText.c_str(), FanucElapsedMs(compileStart));
			}
			return false;
		}

		std::error_code ec;
		std::filesystem::remove(absolutePcPath, ec);

		const std::filesystem::path ktransWorkDir = ktransPath.parent_path();
		const std::wstring exePathW = ktransPath.wstring();
		const std::wstring workDirW = ktransWorkDir.wstring();
		const std::wstring commandTextW =
			L"\"" + exePathW + L"\" \"" +
			absoluteKlPath.wstring() + L"\" \"" +
			absolutePcPath.wstring() + L"\"";
		std::vector<wchar_t> commandLine(commandTextW.begin(), commandTextW.end());
		commandLine.push_back(L'\0');

		STARTUPINFOW si = {};
		si.cb = sizeof(si);
		si.dwFlags = STARTF_USESHOWWINDOW;
		si.wShowWindow = SW_HIDE;
		PROCESS_INFORMATION pi = {};

		const BOOL created = CreateProcessW(
			nullptr,
			commandLine.data(),
			nullptr,
			nullptr,
			FALSE,
			0,
			nullptr,
			workDirW.c_str(),
			&si,
			&pi);

		if (!created)
		{
			if (pLog != nullptr)
			{
				pLog->write(LogColor::ERR, "FANUC 编译失败：启动 ktrans.exe 失败，错误码=%lu | 耗时=%lldms",
					GetLastError(), FanucElapsedMs(compileStart));
			}
			return false;
		}

		WaitForSingleObject(pi.hProcess, INFINITE);

		DWORD exitCode = 0;
		GetExitCodeProcess(pi.hProcess, &exitCode);
		CloseHandle(pi.hThread);
		CloseHandle(pi.hProcess);

		if (exitCode != 0 || !FanucFileExists(absolutePcPath))
		{
			if (pLog != nullptr)
			{
				const std::string klPathText = FanucLocalPathBytes(absoluteKlPath);
				const std::string pcPathText = FanucLocalPathBytes(absolutePcPath);
				const std::string workDirText = FanucLocalPathBytes(ktransWorkDir);
				pLog->write(LogColor::ERR,
					"FANUC 编译失败：ktrans 返回码=%lu，KL=%s，PC=%s，WorkDir=%s | 耗时=%lldms",
					static_cast<unsigned long>(exitCode), klPathText.c_str(), pcPathText.c_str(), workDirText.c_str(),
					FanucElapsedMs(compileStart));
			}
			return false;
		}

		if (pLog != nullptr)
		{
			const std::string pcPathText = FanucLocalPathBytes(absolutePcPath);
			pLog->write(LogColor::SUCCESS, "FANUC 编译成功：%s | 耗时=%lldms",
				pcPathText.c_str(), FanucElapsedMs(compileStart));
		}
		return true;
	}

	bool FanucCompileLsToTp(const std::string& lsPath, const std::string& tpPath, RobotLog* pLog)
	{
		std::lock_guard<std::mutex> compilerLock(g_fanucCompilerMutex);
		const auto compileStart = std::chrono::steady_clock::now();
		const std::filesystem::path maketpPath = FanucGetMaketpPath();
		const std::filesystem::path absoluteLsPath = FanucAbsolutePath(lsPath);
		const std::filesystem::path absoluteTpPath = FanucAbsolutePath(tpPath);
		if (!FanucFileExists(maketpPath))
		{
			if (pLog != nullptr)
			{
				const std::string maketpPathText = FanucLocalPathBytes(maketpPath);
				pLog->write(LogColor::ERR, "FANUC TP编译失败：未找到 maketp.exe，路径=%s | 耗时=%lldms",
					maketpPathText.c_str(), FanucElapsedMs(compileStart));
			}
			return false;
		}

		std::error_code ec;
		std::filesystem::create_directories(absoluteTpPath.parent_path(), ec);
		std::filesystem::remove(absoluteTpPath, ec);

		const std::filesystem::path maketpWorkDir = maketpPath.parent_path();
		const std::wstring exePathW = maketpPath.wstring();
		const std::wstring workDirW = maketpWorkDir.wstring();
		const std::wstring commandTextW =
			L"\"" + exePathW + L"\" \"" +
			absoluteLsPath.wstring() + L"\" \"" +
			absoluteTpPath.wstring() + L"\"";
		std::vector<wchar_t> commandLine(commandTextW.begin(), commandTextW.end());
		commandLine.push_back(L'\0');

		STARTUPINFOW si = {};
		si.cb = sizeof(si);
		si.dwFlags = STARTF_USESHOWWINDOW;
		si.wShowWindow = SW_HIDE;
		PROCESS_INFORMATION pi = {};

		const BOOL created = CreateProcessW(
			nullptr,
			commandLine.data(),
			nullptr,
			nullptr,
			FALSE,
			0,
			nullptr,
			workDirW.c_str(),
			&si,
			&pi);

		if (!created)
		{
			if (pLog != nullptr)
			{
				pLog->write(LogColor::ERR, "FANUC TP编译失败：启动 maketp.exe 失败，错误码=%lu | 耗时=%lldms",
					GetLastError(), FanucElapsedMs(compileStart));
			}
			return false;
		}

		WaitForSingleObject(pi.hProcess, INFINITE);

		DWORD exitCode = 0;
		GetExitCodeProcess(pi.hProcess, &exitCode);
		CloseHandle(pi.hThread);
		CloseHandle(pi.hProcess);

		if (exitCode == 0 && !FanucFileExists(absoluteTpPath))
		{
			const std::filesystem::path fallbackTpPath = FanucFindCompiledTpInOutputDir(absoluteTpPath);
			if (!fallbackTpPath.empty())
			{
				std::filesystem::copy_file(
					fallbackTpPath,
					absoluteTpPath,
					std::filesystem::copy_options::overwrite_existing,
					ec);
				if (!ec && pLog != nullptr)
				{
					const std::string fallbackPathText = FanucLocalPathBytes(fallbackTpPath);
					const std::string tpPathText = FanucLocalPathBytes(absoluteTpPath);
					pLog->write(LogColor::DEFAULT,
						"FANUC TP编译产物已从 WinOLPC 输出目录复制：%s -> %s",
						fallbackPathText.c_str(),
						tpPathText.c_str());
				}
			}
		}

		if (exitCode != 0 || !FanucFileExists(absoluteTpPath))
		{
			if (pLog != nullptr)
			{
				const std::string lsPathText = FanucLocalPathBytes(absoluteLsPath);
				const std::string tpPathText = FanucLocalPathBytes(absoluteTpPath);
				const std::string workDirText = FanucLocalPathBytes(maketpWorkDir);
				pLog->write(LogColor::ERR,
					"FANUC TP编译失败：maketp 返回码=%lu，LS=%s，TP=%s，WorkDir=%s | 耗时=%lldms",
					static_cast<unsigned long>(exitCode), lsPathText.c_str(), tpPathText.c_str(), workDirText.c_str(),
					FanucElapsedMs(compileStart));
			}
			return false;
		}

		if (pLog != nullptr)
		{
			const std::string tpPathText = FanucLocalPathBytes(absoluteTpPath);
			pLog->write(LogColor::SUCCESS, "FANUC TP编译成功：%s | 耗时=%lldms",
				tpPathText.c_str(), FanucElapsedMs(compileStart));
		}
		return true;
	}
}

FANUCRobotCtrl::FANUCRobotCtrl(std::string strUnitName, RobotLog* pLog)
	: RobotDriverAdaptor(strUnitName, pLog),
	m_hMutex(nullptr),
	m_bLocalDebugMark(false),
	m_uSocketHandle(static_cast<std::uintptr_t>(FanucInvalidSocket())),
	m_bSocketConnected(false),
	m_bWSAStarted(false),
	m_uMonitorSocketHandle(static_cast<std::uintptr_t>(FanucInvalidSocket())),
	m_bMonitorRunning(false),
	m_bMonitorWSAStarted(false),
	m_nMonitorPort(9001),
	m_sMonitorText("状态: 监控未连接"),
	m_nMonitorDone(-1),
	m_nMonitorDoneRaw(-1),
	m_nMonitorDoneCandidate(-1),
	m_nMonitorDoneStableCount(0),
	m_llMonitorRobotMs(0),
	m_llMonitorPcRecvMs(0),
	m_llLastCallJobPcMs(0),
	m_continuousMoveRunning(false),
	m_continuousMoveStopRequested(false),
	m_continuousMoveRobotStarted(false),
	m_continuousMoveType(MOVJ),
	m_continuousMoveSpeed(1.0),
	m_continuousWrittenCount(0),
	m_continuousConsumedCount(0)
{
	InitRobotDriver(strUnitName);
	// 每个 driver 独立串行化自己的 S4 socket；不使用全局命名 mutex，避免不同机器人无谓互斥。
	m_hMutex = CreateMutexA(nullptr, FALSE, nullptr);
	if (m_hMutex == nullptr && m_pRobotLog != nullptr)
	{
		m_pRobotLog->write(LogColor::ERR, "FANUC 控制 socket 互斥锁创建失败，后续命令将 fail-closed");
	}
}

FANUCRobotCtrl::~FANUCRobotCtrl()
{
	StopStateMonitor();
	EndContinuousMoveQueue();
	StopMonitor();
	CloseSocket();
	delete m_pFTP;
	m_pFTP = nullptr;

	if (m_hMutex != nullptr)
	{
		CloseHandle(m_hMutex);
		m_hMutex = nullptr;
	}
}

RobotDriverDescriptor FANUCRobotCtrl::DriverDescriptor() const
{
	return RobotDriverDescriptor{
		RobotDriverFamily::Fanuc,
		ROBOT_TYPE_FANUC,
		ROBOT_TYPE_FANUC,
		"FANUC",
		"FANUC"
	};
}

RobotConnectionEndpoint FANUCRobotCtrl::ControlEndpoint() const
{
	return RobotConnectionEndpoint{ m_sSocketIP, m_nSocketPort };
}

bool FANUCRobotCtrl::Connect()
{
	const RobotConnectionEndpoint endpoint = ControlEndpoint();
	if (!endpoint.IsValid())
	{
		SetLastRobotError("FANUC连接参数不完整。");
		return false;
	}
	return InitSocket(endpoint.host.c_str(), static_cast<unsigned short>(endpoint.port));
}

bool FANUCRobotCtrl::Disconnect()
{
	return CloseSocket();
}

std::uint64_t FANUCRobotCtrl::DriverCapabilities() const
{
	return RobotDriverCapabilityBit(RobotDriverCapability::PassiveState)
		| RobotDriverCapabilityBit(RobotDriverCapability::RobotTimestamp)
		| RobotDriverCapabilityBit(RobotDriverCapability::LinearMotion)
		| RobotDriverCapabilityBit(RobotDriverCapability::JointMotion)
		| RobotDriverCapabilityBit(RobotDriverCapability::ContinuousTrajectory)
		| RobotDriverCapabilityBit(RobotDriverCapability::ContinuousJog)
		| RobotDriverCapabilityBit(RobotDriverCapability::PauseResume)
		| RobotDriverCapabilityBit(RobotDriverCapability::PersistentProgramRecovery)
		| RobotDriverCapabilityBit(RobotDriverCapability::NativeProgramUpload)
		| RobotDriverCapabilityBit(RobotDriverCapability::DiagnosticCommand)
		| RobotDriverCapabilityBit(RobotDriverCapability::CartesianRegister)
		| RobotDriverCapabilityBit(RobotDriverCapability::VerifiedProgramCompletion)
		| RobotDriverCapabilityBit(RobotDriverCapability::VerifiedSafeAbort)
		| RobotDriverCapabilityBit(RobotDriverCapability::HandEyeProgramSupport)
		| RobotDriverCapabilityBit(RobotDriverCapability::OfflineTrajectoryExport)
		| RobotDriverCapabilityBit(RobotDriverCapability::ConnectionControl)
		| RobotDriverCapabilityBit(RobotDriverCapability::ToolDataRead)
		| RobotDriverCapabilityBit(RobotDriverCapability::IntegerRegister)
		| RobotDriverCapabilityBit(RobotDriverCapability::TeachPendantSpeedControl)
		| RobotDriverCapabilityBit(RobotDriverCapability::NativeProgramExecution)
		| RobotDriverCapabilityBit(RobotDriverCapability::FtpFileTransfer)
		| RobotDriverCapabilityBit(RobotDriverCapability::HandEyeMatrixRead)
		| RobotDriverCapabilityBit(RobotDriverCapability::HandEyeSupportProgramInstall);
}

RobotFileTransferProfile FANUCRobotCtrl::FileTransferProfile() const
{
	RobotFileTransferProfile profile;
	profile.robotName = m_sRobotName;
	profile.endpointDisplay = m_sFTPIP;
	if (!profile.endpointDisplay.empty() && m_nFTPPort > 0)
	{
		profile.endpointDisplay += ":" + std::to_string(m_nFTPPort);
	}
	profile.defaultRemoteDirectory = "/md";
	profile.defaultLocalDirectory = "Job/FANUC";
	profile.localFileFilters = { "*.ls", "*.tp", "*.kl", "*.pc", "*.var", "*.vr", "*.dt" };
	return profile;
}

std::shared_ptr<RobotFileTransferSession> FANUCRobotCtrl::CreateFileTransferSession(
	std::string* error) const
{
	if (m_sFTPIP.empty() || m_nFTPPort <= 0 || m_nFTPPort > 65535)
	{
		if (error != nullptr)
		{
			*error = "FANUC机器人FTP参数不完整。";
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
		std::vector<std::string>{ ".ls", ".tp", ".kl", ".pc", ".var", ".vr", ".dt" },
		std::vector<std::string>{ ".tp", ".pc" },
		"Log/FanucRobotFtp.log");
}

bool FANUCRobotCtrl::ValidateLinearSpeedMmPerMin(double speedMmPerMin, std::string* error) const
{
	if (!std::isfinite(speedMmPerMin) || speedMmPerMin < 60.0)
	{
		if (error != nullptr)
		{
			*error = GetStr(
				"FANUC线速度无法表示：请求=%.6f mm/min，固定TP最低为60 mm/min。",
				speedMmPerMin);
		}
		return false;
	}
	if (error != nullptr)
	{
		error->clear();
	}
	return true;
}

bool FANUCRobotCtrl::MoveLinearMmPerMin(
	const T_ROBOT_COORS& target,
	double speedMmPerMin,
	int externalAxleType,
	const int* configuration)
{
	if (externalAxleType != 0 || std::abs(target.dBX) > 1e-9
		|| std::abs(target.dBY) > 1e-9 || std::abs(target.dBZ) > 1e-9)
	{
		SetLastRobotError("FANUC直线运动已限制：当前底层只验证了GP1六轴机器人，不能执行外部轴目标。");
		return false;
	}
	std::string error;
	if (!ValidateLinearSpeedMmPerMin(speedMmPerMin, &error))
	{
		SetLastRobotError(error);
		return false;
	}
	const double nativeMmPerSec = std::floor(speedMmPerMin / 60.0);
	int nativeConfiguration[7] = {};
	if (configuration != nullptr)
	{
		std::copy(configuration, configuration + 7, nativeConfiguration);
	}
	return MoveByJob(
		target,
		T_ROBOT_MOVE_SPEED(nativeMmPerSec, 0.0, 0.0),
		externalAxleType,
		"MOVL",
		1,
		nativeConfiguration);
}

bool FANUCRobotCtrl::MoveJointPercent(
	const T_ANGLE_PULSE& target,
	double speedPercent,
	int externalAxleType)
{
	if (externalAxleType != 0 || target.lBXPulse != 0
		|| target.lBYPulse != 0 || target.lBZPulse != 0)
	{
		SetLastRobotError("FANUC关节运动已限制：当前底层只验证了GP1/J1-J6，不能执行外部轴目标。");
		return false;
	}
	if (!std::isfinite(speedPercent) || speedPercent <= 0.0 || speedPercent > 100.0)
	{
		SetLastRobotError(GetStr("FANUC关节速度百分比无效：%.6f", speedPercent));
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
	RobotMotionStatus FanucNormalizedMotionStatus(int rawCode, const std::string& detail)
	{
		RobotMotionStatus status;
		status.rawCode = rawCode;
		status.detail = detail;
		if (rawCode == 0)
		{
			status.state = RobotMotionState::Running;
		}
		else if (rawCode == 1)
		{
			status.state = RobotMotionState::Completed;
		}
		else
		{
			status.state = RobotMotionState::Unknown;
		}
		return status;
	}

	bool FanucCanonicalTrajectoryToNative(
		const FANUCRobotCtrl* driver,
		const std::vector<T_ROBOT_MOVE_INFO>& source,
		std::vector<T_ROBOT_MOVE_INFO>& native,
		std::string& error)
	{
		native = source;
		for (std::size_t index = 0; index < native.size(); ++index)
		{
			T_ROBOT_MOVE_INFO& info = native[index];
			if (info.nMoveType != MOVL)
			{
				continue;
			}
			if (!driver->ValidateLinearSpeedMmPerMin(info.tSpeed.dSpeed, &error))
			{
				error = GetStr("FANUC轨迹点%u速度无效：%s",
					static_cast<unsigned>(index), error.c_str());
				return false;
			}
			info.tSpeed.dSpeed = std::floor(info.tSpeed.dSpeed / 60.0);
		}
		return true;
	}
}

RobotMotionStatus FANUCRobotCtrl::ReadMotionStatus()
{
	return FanucNormalizedMotionStatus(CheckDone(), GetRobotStatusText());
}

RobotMotionStatus FANUCRobotCtrl::ReadMotionStatusPassive(long long* pRobotMs, long long* pPcRecvMs)
{
	return FanucNormalizedMotionStatus(
		CheckDonePassive(pRobotMs, pPcRecvMs),
		GetStateMonitorSourceText());
}

bool FANUCRobotCtrl::ReserveTrajectory(
	RobotTrajectoryPurpose purpose,
	RobotTrajectoryHandle& handle)
{
	(void)purpose;
	handle = RobotTrajectoryHandle{};
	handle.programName = FanucMakeTpProgramName(this);
	return !handle.programName.empty();
}

bool FANUCRobotCtrl::DownlinkTrajectory(
	const std::vector<T_ROBOT_MOVE_INFO>& moveInfos,
	RobotTrajectoryPurpose purpose,
	RobotTrajectoryHandle& handle)
{
	if (handle.programName.empty() && !ReserveTrajectory(purpose, handle))
	{
		SetLastRobotError("FANUC轨迹身份预留失败。");
		return false;
	}
	std::vector<T_ROBOT_MOVE_INFO> nativeMoveInfos;
	std::string conversionError;
	if (!FanucCanonicalTrajectoryToNative(this, moveInfos, nativeMoveInfos, conversionError))
	{
		SetLastRobotError(conversionError);
		return false;
	}
	std::string programName;
	std::string localPath;
	std::string remotePath;
	const TrajectoryProgramMode mode = purpose == RobotTrajectoryPurpose::ActualWeld
		? TrajectoryProgramMode::ActualWeld
		: TrajectoryProgramMode::DryRun;
	const int ret = UploadMultiPointTpProgram(
		nativeMoveInfos,
		mode,
		&programName,
		&localPath,
		&remotePath,
		handle.programName);
	if (ret != 0)
	{
		return false;
	}
	handle.programName = programName;
	handle.localProgramPath = localPath;
	handle.remoteProgramPath = remotePath;
	handle.prepared = true;
	return true;
}

bool FANUCRobotCtrl::ExportTrajectoryProgramFiles(
	const std::vector<T_ROBOT_MOVE_INFO>& moveInfos,
	RobotTrajectoryPurpose purpose,
	const std::string& outputDirectory,
	RobotTrajectoryHandle& handle,
	std::string* error)
{
	const auto fail = [this, error](const std::string& detail)
		{
			SetLastRobotError(detail);
			if (error != nullptr) { *error = detail; }
			return false;
		};

	if (moveInfos.empty())
	{
		return fail("FANUC离线轨迹导出失败：轨迹点为空。");
	}
	if (outputDirectory.empty())
	{
		return fail("FANUC离线轨迹导出失败：输出目录为空。");
	}
	if (purpose == RobotTrajectoryPurpose::ActualWeld)
	{
		std::string reason;
		HasVerifiedArcWeldContract(&reason);
		return fail(reason);
	}
	const auto weldMetadataIt = std::find_if(
		moveInfos.begin(),
		moveInfos.end(),
		[](const T_ROBOT_MOVE_INFO& info) { return FanucContainsWeldMetadata(info); });
	if (weldMetadataIt != moveInfos.end())
	{
		return fail(GetStr(
			"FANUC离线空跑轨迹包含焊接元数据，已拒绝导出普通TP：Index=%u。",
			static_cast<unsigned>(std::distance(moveInfos.begin(), weldMetadataIt))));
	}
	const auto externalAxisIt = std::find_if(
		moveInfos.begin(),
		moveInfos.end(),
		[](const T_ROBOT_MOVE_INFO& info) { return FanucContainsExternalAxisTarget(info); });
	if (externalAxisIt != moveInfos.end())
	{
		return fail(GetStr(
			"FANUC离线轨迹包含外部轴目标，但当前LS/TP生成器只实现GP1/J1-J6：Index=%u。",
			static_cast<unsigned>(std::distance(moveInfos.begin(), externalAxisIt))));
	}
	if (handle.programName.empty() && !ReserveTrajectory(purpose, handle))
	{
		return fail("FANUC离线轨迹程序身份预留失败。");
	}
	if (!FanucIsValidControllerProgramName(handle.programName))
	{
		return fail("FANUC离线轨迹导出失败：程序名必须以字母开头，仅含字母、数字或下划线，且不超过31字符。");
	}

	std::vector<T_ROBOT_MOVE_INFO> nativeMoveInfos;
	std::string conversionError;
	if (!FanucCanonicalTrajectoryToNative(this, moveInfos, nativeMoveInfos, conversionError))
	{
		return fail(conversionError);
	}
	for (std::size_t index = 0; index < nativeMoveInfos.size(); ++index)
	{
		const T_ROBOT_MOVE_INFO& info = nativeMoveInfos[index];
		int speedRegister = 0;
		const bool speedOk = info.nMoveType == MOVL
			? FanucLinearSpeedRegister(info.tSpeed.dSpeed, speedRegister)
			: (speedRegister = FanucSpeedPercent(info.tSpeed.dSpeed)) > 0;
		if (!speedOk)
		{
			return fail(GetStr(
				"FANUC离线轨迹点速度无法安全表示：Index=%u Mode=%s Requested=%.6f",
				static_cast<unsigned>(index),
				info.nMoveType == MOVL ? "MOVL" : "MOVJ",
				info.tSpeed.dSpeed));
		}
	}

	const std::filesystem::path outputPath = FanucResolveLocalPath(outputDirectory);
	std::error_code directoryError;
	std::filesystem::create_directories(outputPath, directoryError);
	if (directoryError || !std::filesystem::is_directory(outputPath, directoryError))
	{
		return fail("FANUC离线轨迹导出失败：无法创建输出目录，" + outputDirectory);
	}
	const std::filesystem::path lsPath = outputPath / (handle.programName + ".ls");
	const std::filesystem::path tpPath = outputPath / (handle.programName + ".tp");
	const std::string lsPathText = FanucLocalPathBytes(lsPath);
	const std::string tpPathText = FanucLocalPathBytes(tpPath);
	const std::string lsContent = FanucBuildTpMoveLsContent(
		handle.programName,
		nativeMoveInfos,
		m_tAxisUnit);
	{
		std::lock_guard<std::mutex> filePairLock(g_fanucGeneratedFilePairMutex);
		if (!FanucWriteTextFile(lsPathText, lsContent))
		{
			return fail("FANUC离线轨迹导出失败：无法写入LS文件，" + lsPathText);
		}
		if (!FanucCompileLsToTp(lsPathText, tpPathText, m_pRobotLog))
		{
			return fail("FANUC离线轨迹导出失败：LS已生成，但MakeTP未能生成控制器TP文件。");
		}
	}

	handle.localProgramPath = lsPathText;
	handle.localDataPath = tpPathText;
	handle.remoteProgramPath.clear();
	handle.remoteDataPath.clear();
	handle.prepared = true;
	handle.started = false;
	ClearLastRobotError();
	if (error != nullptr) { error->clear(); }
	if (m_pRobotLog != nullptr)
	{
		m_pRobotLog->write(LogColor::SUCCESS,
			"FANUC离线轨迹导出完成 | Program=%s | LS=%s | TP=%s",
			handle.programName.c_str(), lsPathText.c_str(), tpPathText.c_str());
	}
	return true;
}

bool FANUCRobotCtrl::StartTrajectory(
	const std::vector<T_ROBOT_MOVE_INFO>& moveInfos,
	RobotTrajectoryPurpose purpose,
	RobotTrajectoryHandle& handle)
{
	(void)moveInfos;
	(void)purpose;
	if (!handle.prepared || handle.programName.empty() || handle.remoteProgramPath.empty())
	{
		SetLastRobotError("FANUC轨迹尚未通过DownlinkTrajectory完成生成和上传，禁止启动。");
		return false;
	}
	if (!CallJobWithCompletionState(handle.programName, FANUC_DEFAULT_MOTION_STATE_REG, 1))
	{
		return false;
	}
	handle.started = true;
	return true;
}

bool FANUCRobotCtrl::WaitTrajectory(
	const RobotTrajectoryHandle& handle,
	int pollDelayMs,
	int runTimeoutMs,
	RobotMotionStatus* terminalStatus)
{
	if (!handle.started)
	{
		SetLastRobotError("FANUC轨迹尚未启动，禁止等待完成。");
		return false;
	}
	const int ret = CheckRobotDone(pollDelayMs, runTimeoutMs);
	if (terminalStatus != nullptr)
	{
		*terminalStatus = FanucNormalizedMotionStatus(ret > 0 ? 1 : ret, GetRobotStatusText());
		terminalStatus->terminalVerified = ret > 0;
	}
	return ret > 0;
}

bool FANUCRobotCtrl::GetTrackedMotionIdentity(
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
	if (!HasServiceCapability("PAUSE_V1"))
	{
		return false;
	}
	if (!RobotOperationLease::MotionCompletionPending(this))
	{
		SetLastRobotError("FANUC运动身份读取失败：本软件没有正在跟踪的未完成运动。");
		return false;
	}
	std::string response;
	FanucTaskSnapshot snapshot;
	if (!FanucRequest(this, "GET_ACTIVE_TASK", response)
		|| !FanucParseTaskSnapshot(response, snapshot)
		|| snapshot.programName == "NONE")
	{
		SetLastRobotError("FANUC运动身份读取失败：活动任务响应无效，RSP=" + response);
		return false;
	}
	if (snapshot.status != 0 && snapshot.status != 1 && snapshot.status != 2)
	{
		SetLastRobotError(GetStr(
			"FANUC运动身份读取失败：任务状态不可跟踪，Program=%s Status=%d",
			snapshot.programName.c_str(), snapshot.status));
		return false;
	}
	projectName = "MD";
	programName = snapshot.programName;
	if (alreadyStopped != nullptr)
	{
		*alreadyStopped = snapshot.status == 2;
	}
	ClearLastRobotError();
	return true;
}

bool FANUCRobotCtrl::PauseTrackedMotion(
	const std::string& expectedProgramName,
	int& programLine,
	T_ROBOT_COORS& pausedPose,
	std::string* projectName,
	std::string* programName)
{
	programLine = -1;
	pausedPose = T_ROBOT_COORS();
	if (projectName != nullptr) { projectName->clear(); }
	if (programName != nullptr) { programName->clear(); }
	if (!HasServiceCapability("PAUSE_V1"))
	{
		return false;
	}
	if (expectedProgramName.empty()
		|| RobotOperationLease::IsCancellationRequested(this)
		|| !RobotOperationLease::MotionCompletionPending(this))
	{
		SetLastRobotError("FANUC暂停失败：预期程序为空、运动已取消或没有受跟踪的未完成运动。");
		return false;
	}

	std::string response;
	FanucTaskSnapshot beforePause;
	if (!FanucRequest(this, "GET_ACTIVE_TASK", response)
		|| !FanucParseTaskSnapshot(response, beforePause)
		|| beforePause.programName != expectedProgramName
		|| (beforePause.status != 0 && beforePause.status != 1))
	{
		SetLastRobotError("FANUC暂停失败：当前活动任务身份或状态不匹配，Expected="
			+ expectedProgramName + " RSP=" + response);
		return false;
	}

	if (!FanucRequest(this, "PAUSE_TASK:" + expectedProgramName, response)
		|| !FanucStartsWith(response, "PAUSED:"))
	{
		SetLastRobotError("FANUC暂停命令未获得已暂停证明，RSP=" + response);
		return false;
	}
	const std::vector<std::string> pausedFields = FanucSplit(FanucResponsePayload(response), ',');
	int reportedLine = -1;
	if (pausedFields.size() != 2 || pausedFields[0] != expectedProgramName
		|| !FanucParseIntStrict(pausedFields[1], reportedLine) || reportedLine < 0)
	{
		SetLastRobotError("FANUC暂停响应格式或任务身份无效，RSP=" + response);
		return false;
	}

	T_ROBOT_COORS firstPose;
	T_ROBOT_COORS secondPose;
	if (!TryGetCurrentPos(firstPose))
	{
		SetLastRobotError("FANUC暂停快照失败：第一次位姿读取失败。");
		return false;
	}
	Sleep(100);
	if (RobotOperationLease::IsCancellationRequested(this) || !TryGetCurrentPos(secondPose))
	{
		SetLastRobotError("FANUC暂停快照失败：安全取消或第二次位姿读取失败。");
		return false;
	}

	FanucTaskSnapshot afterPause;
	if (!FanucRequest(this, "GET_ACTIVE_TASK", response)
		|| !FanucParseTaskSnapshot(response, afterPause)
		|| afterPause.programName != expectedProgramName
		|| afterPause.status != 1
		|| afterPause.line != reportedLine)
	{
		SetLastRobotError("FANUC暂停确认失败：位姿采样后任务身份、暂停态或行号变化，RSP=" + response);
		return false;
	}
	const double positionDrift = FanucPositionDeviationMm(firstPose, secondPose);
	const double angleDrift = FanucAngleDeviationDeg(firstPose, secondPose);
	if (!FanucPoseIsFinite(firstPose) || !FanucPoseIsFinite(secondPose)
		|| !std::isfinite(positionDrift) || positionDrift > 0.2
		|| !std::isfinite(angleDrift) || angleDrift > 0.2)
	{
		SetLastRobotError(GetStr(
			"FANUC暂停快照未稳定：Program=%s Line=%d Drift=%.3fmm/%.3fdeg",
			expectedProgramName.c_str(), reportedLine, positionDrift, angleDrift));
		return false;
	}

	programLine = reportedLine;
	pausedPose = secondPose;
	if (projectName != nullptr) { *projectName = "MD"; }
	if (programName != nullptr) { *programName = expectedProgramName; }
	ClearLastRobotError();
	return true;
}

bool FANUCRobotCtrl::ResumeTrackedMotion(
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
	if (!HasServiceCapability("PAUSE_V1"))
	{
		return false;
	}
	if (expectedProgramName.empty()
		|| RobotOperationLease::IsCancellationRequested(this)
		|| !RobotOperationLease::MotionCompletionPending(this)
		|| !FanucPoseIsFinite(checkpointPose)
		|| !std::isfinite(maxPositionDeviationMm) || maxPositionDeviationMm < 0.0
		|| !std::isfinite(maxAngleDeviationDeg) || maxAngleDeviationDeg < 0.0)
	{
		SetLastRobotError("FANUC继续失败：程序身份、运动跟踪状态、断点位姿或偏差阈值无效。");
		return false;
	}

	std::string response;
	FanucTaskSnapshot firstSnapshot;
	if (!FanucRequest(this, "GET_ACTIVE_TASK", response)
		|| !FanucParseTaskSnapshot(response, firstSnapshot)
		|| firstSnapshot.programName != expectedProgramName
		|| firstSnapshot.status != 1)
	{
		SetLastRobotError("FANUC继续失败：当前任务不是预期暂停任务，Expected="
			+ expectedProgramName + " RSP=" + response);
		return false;
	}

	T_ROBOT_COORS firstPose;
	T_ROBOT_COORS secondPose;
	if (!TryGetCurrentPos(firstPose))
	{
		SetLastRobotError("FANUC继续失败：第一次暂停位姿读取失败。");
		return false;
	}
	Sleep(100);
	if (RobotOperationLease::IsCancellationRequested(this) || !TryGetCurrentPos(secondPose))
	{
		SetLastRobotError("FANUC继续失败：安全取消或第二次暂停位姿读取失败。");
		return false;
	}

	FanucTaskSnapshot secondSnapshot;
	if (!FanucRequest(this, "GET_ACTIVE_TASK", response)
		|| !FanucParseTaskSnapshot(response, secondSnapshot)
		|| secondSnapshot.programName != expectedProgramName
		|| secondSnapshot.status != 1
		|| secondSnapshot.line != firstSnapshot.line)
	{
		SetLastRobotError("FANUC继续失败：位姿核对期间暂停任务身份、状态或行号变化，RSP=" + response);
		return false;
	}

	const double stablePositionDrift = FanucPositionDeviationMm(firstPose, secondPose);
	const double stableAngleDrift = FanucAngleDeviationDeg(firstPose, secondPose);
	if (!FanucPoseIsFinite(firstPose) || !FanucPoseIsFinite(secondPose)
		|| !std::isfinite(stablePositionDrift) || stablePositionDrift > 0.2
		|| !std::isfinite(stableAngleDrift) || stableAngleDrift > 0.2)
	{
		SetLastRobotError(GetStr(
			"FANUC继续失败：暂停位姿不稳定，Drift=%.3fmm/%.3fdeg",
			stablePositionDrift, stableAngleDrift));
		return false;
	}

	const double positionDeviation = FanucPositionDeviationMm(secondPose, checkpointPose);
	const double angleDeviation = FanucAngleDeviationDeg(secondPose, checkpointPose);
	if (positionDeviationMm != nullptr) { *positionDeviationMm = positionDeviation; }
	if (angleDeviationDeg != nullptr) { *angleDeviationDeg = angleDeviation; }
	if (!std::isfinite(positionDeviation) || positionDeviation > maxPositionDeviationMm
		|| !std::isfinite(angleDeviation) || angleDeviation > maxAngleDeviationDeg)
	{
		SetLastRobotError(GetStr(
			"FANUC继续被拒绝：当前位置偏离断点 %.3fmm/%.3fdeg，允许 %.3fmm/%.3fdeg。",
			positionDeviation, angleDeviation, maxPositionDeviationMm, maxAngleDeviationDeg));
		return false;
	}

	if (!FanucRequest(this, "RESUME_TASK:" + expectedProgramName, response)
		|| !FanucStartsWith(response, "RESUMED:"))
	{
		SetLastRobotError("FANUC继续命令未获得运行态证明，RSP=" + response);
		return false;
	}
	const std::vector<std::string> resumedFields = FanucSplit(FanucResponsePayload(response), ',');
	int resumedStatus = -999;
	if (resumedFields.size() != 2 || resumedFields[0] != expectedProgramName
		|| !FanucParseIntStrict(resumedFields[1], resumedStatus)
		|| (resumedStatus != 0 && resumedStatus != -1))
	{
		SetLastRobotError("FANUC继续响应格式、身份或状态无效，RSP=" + response);
		return false;
	}
	ClearLastRobotError();
	return true;
}

RobotPersistentRecoveryStrategy FANUCRobotCtrl::PersistentRecoveryStrategy() const
{
	return RobotPersistentRecoveryStrategy::AbortUnknownCurrentProgram;
}

bool FANUCRobotCtrl::AbortPersistedMotion(const std::string& expectedProgramName)
{
	if (expectedProgramName.empty())
	{
		SetLastRobotError("FANUC持久恢复缺少预期程序身份。");
		return false;
	}
	return AbortCurrentProgramSafely();
}

bool FANUCRobotCtrl::SetOperationMode(RobotOperationMode mode)
{
	return SetSysMode(static_cast<int>(mode));
}

bool FANUCRobotCtrl::InitializeAfterConnect(std::string* summary)
{
	if (summary != nullptr)
	{
		*summary = "FANUC连接已建立；控制器模式和伺服状态保持现场设置。";
	}
	return true;
}

bool FANUCRobotCtrl::ShutdownBeforeDisconnect()
{
	bool ok = true;
	if (IsConnected())
	{
		ok = StopRobotServices();
	}
	StopMonitor();
	return ok;
}

void FANUCRobotCtrl::ReloadRuntimeConfiguration()
{
}

bool FANUCRobotCtrl::PrepareNativeProgramUpload()
{
	return StopRobotServices();
}

bool FANUCRobotCtrl::StartContinuousJog(int moveType, double canonicalSpeed)
{
	if (moveType == MOVL)
	{
		std::string error;
		if (!ValidateLinearSpeedMmPerMin(canonicalSpeed, &error))
		{
			SetLastRobotError(error);
			return false;
		}
		canonicalSpeed = std::floor(canonicalSpeed / 60.0);
	}
	else if (!std::isfinite(canonicalSpeed) || canonicalSpeed <= 0.0 || canonicalSpeed > 100.0)
	{
		SetLastRobotError(GetStr("FANUC连续关节点动速度百分比无效：%.6f", canonicalSpeed));
		return false;
	}
	return StartContinuousMoveQueue(moveType, canonicalSpeed);
}

bool FANUCRobotCtrl::PushContinuousJogPoint(const T_ROBOT_COORS& target, double speedMmPerMin)
{
	std::string error;
	if (!ValidateLinearSpeedMmPerMin(speedMmPerMin, &error))
	{
		SetLastRobotError(error);
		return false;
	}
	return PushContinuousMovePoint(target, std::floor(speedMmPerMin / 60.0));
}

bool FANUCRobotCtrl::PushContinuousJogPoint(const T_ANGLE_PULSE& target, double speedPercent)
{
	if (!std::isfinite(speedPercent) || speedPercent <= 0.0 || speedPercent > 100.0)
	{
		SetLastRobotError(GetStr("FANUC连续关节点动速度百分比无效：%.6f", speedPercent));
		return false;
	}
	return PushContinuousMovePoint(target, speedPercent);
}

void FANUCRobotCtrl::RequestEndContinuousJog()
{
	RequestEndContinuousMoveQueue();
}

void FANUCRobotCtrl::EndContinuousJog()
{
	EndContinuousMoveQueue();
}

bool FANUCRobotCtrl::IsContinuousJogRunning() const
{
	return IsContinuousMoveQueueRunning();
}

int FANUCRobotCtrl::UploadNativeProgramSource(
	const std::string& localPath,
	const std::string& remoteDirectory)
{
	std::string extension = std::filesystem::path(localPath).extension().string();
	std::transform(extension.begin(), extension.end(), extension.begin(),
		[](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
	const std::string remote = remoteDirectory.empty() ? "/md/" : remoteDirectory;
	if (extension == ".kl")
	{
		return UploadKlFile(localPath, remote);
	}
	if (extension == ".ls")
	{
		return UploadLsFile(localPath, remote);
	}
	if (extension == ".pc" || extension == ".tp")
	{
		std::string remoteDirectoryPath = remote;
		if (remoteDirectoryPath.back() != '/')
		{
			remoteDirectoryPath.push_back('/');
		}
		const std::string remotePath = remoteDirectoryPath
			+ std::filesystem::path(localPath).filename().string();
		return UploadFile(localPath, remotePath);
	}
	SetLastRobotError("FANUC原生程序上传仅支持KL、PC、LS或TP文件：" + localPath);
	return -1;
}

std::string FANUCRobotCtrl::SendDiagnosticCommand(const std::string& command)
{
	return SendRawCommandForTest(command);
}

bool FANUCRobotCtrl::WriteCartesianRegister(int index, const double pose[8], int config[7])
{
	double writablePose[8] = {};
	std::copy(pose, pose + 8, writablePose);
	return SetPosVar(index, writablePose, POSVAR, 1, config, ENGINEEVAR, POSVAR);
}

bool FANUCRobotCtrl::RunProgramAndWait(
	const std::string& programName,
	int startTimeoutMs,
	int finishTimeoutMs,
	int pollDelayMs,
	RobotMotionStatus* terminalStatus)
{
	int rawState = -1;
	const bool ok = CallJobAndWaitStateDone(
		programName,
		FANUC_DEFAULT_MOTION_STATE_REG,
		1,
		10,
		20,
		startTimeoutMs,
		finishTimeoutMs,
		pollDelayMs,
		&rawState,
		true);
	if (terminalStatus != nullptr)
	{
		*terminalStatus = FanucNormalizedMotionStatus(ok ? 1 : rawState, GetRobotStatusText());
		terminalStatus->terminalVerified = ok;
	}
	return ok;
}

bool FANUCRobotCtrl::InstallHandEyeSupportPrograms(std::string* summary)
{
	const QString autoProgramPath = AppPaths::FindResourcePath("SDK/FANUC/FANUC_HECALIB.ls");
	const QString validationProgramPath = AppPaths::FindResourcePath("SDK/FANUC/FANUC_HECHECK.ls");
	if (autoProgramPath.isEmpty() || validationProgramPath.isEmpty())
	{
		SetLastRobotError("FANUC手眼辅助程序资源不完整，缺少FANUC_HECALIB.ls或FANUC_HECHECK.ls。");
		return false;
	}
	const int autoRet = UploadLsFile(autoProgramPath.toLocal8Bit().constData(), "/md/");
	if (autoRet != 0)
	{
		SetLastRobotError(GetStr("FANUC自动标定程序上传失败，返回码=%d。", autoRet));
		return false;
	}
	const int validationRet = UploadLsFile(validationProgramPath.toLocal8Bit().constData(), "/md/");
	if (validationRet != 0)
	{
		SetLastRobotError(GetStr("FANUC手眼验证程序上传失败，返回码=%d。", validationRet));
		return false;
	}
	if (summary != nullptr)
	{
		*summary = "FANUC_HECALIB与FANUC_HECHECK已上传到控制器。";
	}
	return true;
}

bool FANUCRobotCtrl::RunHandEyeValidation(
	const T_ROBOT_COORS& robotPose,
	T_ROBOT_COORS& robotCalculatedPoint)
{
	constexpr int startRegister = 79;
	constexpr int resultRegister = 80;
	constexpr int stateRegister = 92;
	constexpr const char* programName = "FANUC_HECHECK";
	int config[7] = {};
	double pose[8] =
	{
		robotPose.dX, robotPose.dY, robotPose.dZ,
		robotPose.dRX, robotPose.dRY, robotPose.dRZ,
		robotPose.dBX, robotPose.dBY
	};
	if (!WriteCartesianRegister(startRegister, pose, config))
	{
		return false;
	}
	int rawState = -1;
	if (!CallJobAndWaitStateDone(
		programName, stateRegister, 1, 10, 20, 5000, 10000, 100, &rawState, true))
	{
		return false;
	}
	double result[6] = {};
	if (GetPosVar(resultRegister, result, config, POSVAR) != 0)
	{
		SetLastRobotError("FANUC手眼验证程序完成，但读取PR[80]失败。");
		return false;
	}
	robotCalculatedPoint = T_ROBOT_COORS();
	robotCalculatedPoint.dX = result[0];
	robotCalculatedPoint.dY = result[1];
	robotCalculatedPoint.dZ = result[2];
	return true;
}

namespace
{
	constexpr long long FANUC_DONE_STARTUP_GUARD_MS = 5000;
	constexpr int FANUC_DONE_PASSIVE_STABLE_COUNT = 3;
	constexpr int FANUC_DONE_ACTIVE_STABLE_COUNT = 4;

	class FanucMutexGuard final
	{
	public:
		explicit FanucMutexGuard(HANDLE handle) : m_handle(handle) {}
		bool Lock(DWORD timeoutMs)
		{
			if (m_handle == nullptr)
			{
				return false;
			}
			const DWORD result = WaitForSingleObject(m_handle, timeoutMs);
			m_locked = result == WAIT_OBJECT_0 || result == WAIT_ABANDONED;
			return m_locked;
		}
		~FanucMutexGuard()
		{
			if (m_locked)
			{
				ReleaseMutex(m_handle);
			}
		}
		FanucMutexGuard(const FanucMutexGuard&) = delete;
		FanucMutexGuard& operator=(const FanucMutexGuard&) = delete;
	private:
		HANDLE m_handle = nullptr;
		bool m_locked = false;
	};

	bool FanucEnsureSocket(FANUCRobotCtrl* ctrl)
	{
		if (ctrl == nullptr)
		{
			return false;
		}
		if (ctrl->m_bSocketConnected.load())
		{
			return true;
		}
		return ctrl->Connect();
	}

	bool FanucRequest(
		FANUCRobotCtrl* ctrl,
		const std::string& command,
		std::string& response,
		bool* commandMayHaveBeenSent)
	{
		response.clear();
		if (commandMayHaveBeenSent != nullptr)
		{
			*commandMayHaveBeenSent = false;
		}
		if (ctrl == nullptr || ctrl->m_hMutex == nullptr)
		{
			if (ctrl != nullptr)
			{
				ctrl->SetLastRobotError("FANUC请求失败：socket互斥锁不可用，拒绝无串行保护的命令，CMD=" + command);
			}
			return false;
		}

		FanucMutexGuard socketGuard(ctrl->m_hMutex);
		if (!socketGuard.Lock(FANUC_SOCKET_TIMEOUT_MS))
		{
			if (ctrl->HasLogSink())
			{
				ctrl->WriteLog(LogColor::ERR, "FANUC Socket CMD=%s 等待互斥锁超时", command.c_str());
			}
			ctrl->SetLastRobotError("FANUC请求失败：等待socket互斥锁超时，CMD=" + command);
			return false;
		}

		// 连接、句柄读取、请求应答及失败关闭全部受同一锁保护，避免等待线程
		// 使用已被另一请求关闭的预缓存 socket 句柄。
		if (!FanucEnsureSocket(ctrl))
		{
			ctrl->SetLastRobotError("FANUC请求失败：控制socket未连接，CMD=" + command);
			return false;
		}

		SOCKET sock = FanucGetSocket(ctrl->m_uSocketHandle);
		if (sock == INVALID_SOCKET)
		{
			ctrl->SetLastRobotError("FANUC请求失败：socket句柄无效，CMD=" + command);
			return false;
		}

		// 对会启动运动/程序的命令，在拿到与 PROGRAM_STOP 共用的 socket mutex 后
		// 再检查一次取消锁存，消除“先检查→STOP→随后才发送 CALL_JOB”的 TOCTOU。
		const bool startsProgramOrMotion =
			FanucStartsWith(command, "CALL_JOB:")
			|| FanucStartsWith(command, "RESUME_TASK:")
			|| command == "PROGRAM_START"
			|| FanucStartsWith(command, "AXIS_PULSE_MOVE:")
			|| FanucStartsWith(command, "POS_MOVE:")
			|| FanucStartsWith(command, "MOVE_BY_JOB_");
		if (startsProgramOrMotion && RobotOperationLease::IsCancellationRequested(ctrl))
		{
			ctrl->SetLastRobotError("FANUC硬件操作已被安全停止取消，运动命令未发送：" + command);
			return false;
		}

		// 一旦进入 send，失败也可能已经发送了部分字节，必须按“结果未知”处理。
		if (commandMayHaveBeenSent != nullptr)
		{
			*commandMayHaveBeenSent = true;
		}
		const bool sent = FanucSendLine(sock, command);
		const bool recvOk = sent && FanucReceiveLine(sock, response);

		if (ctrl->HasLogSink())
		{
			ctrl->WriteLog(sent && recvOk ? LogColor::DEFAULT : LogColor::ERR,
				"FANUC Socket CMD=%s RSP=%s",
				command.c_str(), response.c_str());
		}

		if (!sent || !recvOk)
		{
			ctrl->SetLastRobotError("FANUC请求失败：CMD=" + command + " RSP=" + response);
			ctrl->Disconnect();
		}

		return sent && recvOk;
	}

	bool FanucIsOkResponse(const std::string& response)
	{
		return response == "OK" || FanucStartsWith(response, "OK:");
	}

	std::string FanucResponsePayload(const std::string& response)
	{
		const size_t pos = response.find(':');
		return pos == std::string::npos ? std::string() : response.substr(pos + 1);
	}

	bool FanucParseMonitorFrame(
		const std::string& line,
		T_ROBOT_COORS& pos,
		T_ANGLE_PULSE& pulse,
		int& done,
		long long& robotMs,
		const T_AXISUNIT& axisUnit)
	{
		// Monitor frame format:
		// MON:seq,robot_ms,done,x,y,z,w,p,r,j1,j2,j3,j4,j5,j6,e1,e2,e3
		// robot_ms is generated on the robot side; pc_recv_ms is appended when the frame reaches the PC.
		if (!FanucStartsWith(line, "MON:"))
		{
			return false;
		}

		double values[18] = {};
		if (!FanucParseDoubles(line.substr(4), values, 18))
		{
			return false;
		}

		robotMs = static_cast<long long>(values[1]);
		done = static_cast<int>(values[2]);
		pos = T_ROBOT_COORS(values[3], values[4], values[5], values[6], values[7], values[8], 0, 0, 0);
		pulse = T_ANGLE_PULSE(
			FanucPositionToPulse(values[9], axisUnit.dSPulseUnit),
			FanucPositionToPulse(values[10], axisUnit.dLPulseUnit),
			FanucPositionToPulse(values[11], axisUnit.dUPulseUnit),
			FanucPositionToPulse(values[12], axisUnit.dRPulseUnit),
			FanucPositionToPulse(values[13], axisUnit.dBPulseUnit),
			FanucPositionToPulse(values[14], axisUnit.dTPulseUnit),
			FanucPositionToPulse(values[15], axisUnit.dBXPulseUnit),
			FanucPositionToPulse(values[16], axisUnit.dBYPulseUnit),
			FanucPositionToPulse(values[17], axisUnit.dBZPulseUnit));
		return true;
	}
}

// ===================== 初始化与控制通道 =====================

// 读取RobotPara.ini中的FANUC基础参数、控制端口、监控端口、FTP参数和工具参数。
bool FANUCRobotCtrl::InitRobotDriver(std::string strUnitName)
{
	COPini cIni;
	cIni.SetFileName(DATA_PATH + strUnitName + ROBOT_PARA_INI);
	cIni.SetSectionName("BaseParam");
	cIni.ReadString("RobotName", m_sRobotName);
	cIni.ReadString("CustomName", m_sCustomName);
	cIni.ReadString("SocketIP", m_sSocketIP);
	cIni.ReadString("SocketPort", &m_nSocketPort);
	cIni.ReadString(false, "MonitorPort", &m_nMonitorPort);
	if (m_nMonitorPort <= 0)
	{
		m_nMonitorPort = 9001;
	}
	cIni.ReadString("RobotType", &m_nRobotType);
	cIni.ReadString("RobotBrand", (int*)&m_eRobotBrand);
	cIni.ReadString("FTPIP", m_sFTPIP);
	cIni.ReadString("FTPPort", &m_nFTPPort);
	cIni.ReadString("FTPUser", m_sFTPUser);
	cIni.ReadString("FTPPassWord", m_sFTPPassWord);

	LoadRobotExternalAxlePara(strUnitName);

	cIni.SetSectionName("Tool");
	cIni.ReadString("PolisherTool_d", "", m_tTools.tPolisherTool, T_ROBOT_COORS(1, 1, 1, 1, 1, 1, -1, -1, -1));
	cIni.ReadString("MagnetTool_d", "", m_tTools.tMagnetTool, T_ROBOT_COORS(1, 1, 1, 1, 1, 1, -1, -1, -1));
	cIni.ReadString("GunTool_d", "", m_tTools.tGunTool, T_ROBOT_COORS(1, 1, 1, 1, 1, 1, -1, -1, -1));
	cIni.ReadString("CameraTool_d", "", m_tTools.tCameraTool, T_ROBOT_COORS(1, 1, 1, 1, 1, 1, -1, -1, -1));

	InvalidateFixedMoveUploadCache();
	return true;
}

// 建立S4控制socket连接，并通过HELLO确认机器人常驻服务可用。
bool FANUCRobotCtrl::InitSocket(const char* ip, unsigned short Port, bool ifRecode)
{
	(void)ifRecode;
	FanucMutexGuard socketGuard(m_hMutex);
	if (!socketGuard.Lock(FANUC_SOCKET_TIMEOUT_MS))
	{
		SetLastRobotError("FANUC InitSocket 失败：无法取得 socket 生命周期互斥锁。");
		return false;
	}
	const std::string socketIp = (ip != nullptr && ip[0] != '\0') ? ip : m_sSocketIP;
	const unsigned short socketPort = Port > 0 ? Port : static_cast<unsigned short>(m_nSocketPort);

	if (socketIp.empty() || socketPort == 0)
	{
		if (m_pRobotLog != nullptr)
		{
			m_pRobotLog->write(LogColor::ERR, "FANUC InitSocket 失败：IP或端口为空");
		}
		return false;
	}

	if (!m_bWSAStarted)
	{
		if (!FanucInitWinSock())
		{
			if (m_pRobotLog != nullptr)
			{
				m_pRobotLog->write(LogColor::ERR, "FANUC InitSocket 失败：WSAStartup 失败");
			}
			return false;
		}
		m_bWSAStarted = true;
	}

	if (m_bSocketConnected.load())
	{
		CloseSocket();
		if (!m_bWSAStarted)
		{
			if (!FanucInitWinSock())
			{
				if (m_pRobotLog != nullptr)
				{
					m_pRobotLog->write(LogColor::ERR, "FANUC InitSocket 失败：重新初始化 WinSock 失败");
				}
				return false;
			}
			m_bWSAStarted = true;
		}
	}

	SOCKET sock = FanucInvalidSocket();
	if (!FanucConnectSocket(sock, socketIp, socketPort))
	{
		if (m_pRobotLog != nullptr)
		{
			m_pRobotLog->write(LogColor::ERR,
				"FANUC InitSocket 失败：无法连接 %s:%u，错误码=%d",
				socketIp.c_str(), static_cast<unsigned>(socketPort), WSAGetLastError());
		}
		return false;
	}

	m_uSocketHandle = static_cast<std::uintptr_t>(sock);
	m_bSocketConnected.store(true);

	if (m_pRobotLog != nullptr)
	{
		m_pRobotLog->write(LogColor::SUCCESS,
			"FANUC Socket 已连接：%s:%u",
			socketIp.c_str(), static_cast<unsigned>(socketPort));
	}

	std::string response;
	if (FanucSendLine(sock, "HELLO") && FanucReceiveLine(sock, response))
	{
		if (m_pRobotLog != nullptr)
		{
			m_pRobotLog->write(LogColor::DEFAULT, "FANUC Socket 握手返回：%s", response.c_str());
		}
	}
	else
	{
		if (m_pRobotLog != nullptr)
		{
			m_pRobotLog->write(LogColor::ERR, "FANUC InitSocket 失败：HELLO握手超时或连接已断开");
		}
		CloseSocket();
		return false;
	}

	// The runtime endpoint is authoritative for per-controller caches and local
	// artifact isolation, including callers that reconnect to a non-INI endpoint.
	m_sSocketIP = socketIp;
	m_nSocketPort = static_cast<int>(socketPort);
	InvalidateFixedMoveUploadCache();
	return true;
}

// 关闭S4控制socket；通信失败时也会调用它来触发下次自动重连。
bool FANUCRobotCtrl::CloseSocket()
{
	FanucMutexGuard socketGuard(m_hMutex);
	if (!socketGuard.Lock(FANUC_SOCKET_TIMEOUT_MS))
	{
		SetLastRobotError("FANUC CloseSocket 失败：无法取得 socket 生命周期互斥锁。");
		return false;
	}
	if (m_bSocketConnected.load())
	{
		SOCKET sock = FanucGetSocket(m_uSocketHandle);
		if (sock != INVALID_SOCKET)
		{
			closesocket(sock);
		}
		m_uSocketHandle = static_cast<std::uintptr_t>(FanucInvalidSocket());
		m_bSocketConnected.store(false);
	}

	if (m_bWSAStarted)
	{
		FanucCleanupWinSock();
		m_bWSAStarted = false;
	}

	// A later reconnect must re-verify/re-upload the fixed TP on that controller.
	InvalidateFixedMoveUploadCache();
	return true;
}

bool FANUCRobotCtrl::IsConnected()
{
	return m_bSocketConnected.load();
}

// 主动读取单轴笛卡尔坐标，内部会请求完整当前位置后取对应分量。
double FANUCRobotCtrl::GetCurrentPos(int nAxisNo)
{
	T_ROBOT_COORS pos = GetCurrentPos();
	switch (nAxisNo)
	{
	case 0: return pos.dX;
	case 1: return pos.dY;
	case 2: return pos.dZ;
	case 3: return pos.dRX;
	case 4: return pos.dRY;
	case 5: return pos.dRZ;
	case 6: return pos.dBX;
	case 7: return pos.dBY;
	case 8: return pos.dBZ;
	default: return 0.0;
	}
}

// 主动读取当前TCP位姿，协议命令为GET_CUR_POS。
T_ROBOT_COORS FANUCRobotCtrl::GetCurrentPos()
{
	T_ROBOT_COORS pos;
	return TryGetCurrentPos(pos) ? pos : T_ROBOT_COORS();
}

bool FANUCRobotCtrl::TryGetCurrentPos(T_ROBOT_COORS& pos)
{
	pos = T_ROBOT_COORS();
	std::string response;
	double values[6] = {};
	if (!FanucRequest(this, "GET_CUR_POS", response))
	{
		return false;
	}

	if (!FanucStartsWith(response, "POS:") || !FanucParseDoubles(FanucResponsePayload(response), values, 6))
	{
		SetLastRobotError("FANUC当前位置响应无效：" + response);
		return false;
	}
	for (double value : values)
	{
		if (!std::isfinite(value))
		{
			SetLastRobotError("FANUC当前位置包含非有限值：" + response);
			return false;
		}
	}
	pos = T_ROBOT_COORS(values[0], values[1], values[2], values[3], values[4], values[5], 0, 0, 0);
	return true;
}

// 被动读取当前TCP位姿缓存，同时可取机器人侧robot_ms和PC侧pc_recv_ms。
T_ROBOT_COORS FANUCRobotCtrl::GetCurrentPosPassive(long long* pRobotMs, long long* pPcRecvMs)
{
	std::lock_guard<std::mutex> lock(m_monitorMutex);
	const bool monitorConnected = m_bMonitorRunning.load()
		&& m_uMonitorSocketHandle.load()
			!= static_cast<std::uintptr_t>(FanucInvalidSocket());
	if (!monitorConnected)
	{
		if (pRobotMs != nullptr)
		{
			*pRobotMs = 0;
		}
		if (pPcRecvMs != nullptr)
		{
			*pPcRecvMs = 0;
		}
		return T_ROBOT_COORS();
	}
	if (pRobotMs != nullptr)
	{
		*pRobotMs = m_llMonitorRobotMs;
	}
	if (pPcRecvMs != nullptr)
	{
		*pPcRecvMs = m_llMonitorPcRecvMs;
	}
	return m_tMonitorPos;
}

std::string FANUCRobotCtrl::GetStateMonitorSourceText() const
{
	return "FANUC S5监控缓存(MON robot_ms)，不占用S4控制通道";
}

// 主动读取单轴脉冲，内部会请求完整关节值后按轴当量换算。
double FANUCRobotCtrl::GetCurrentPulse(int nAxisNo)
{
	T_ANGLE_PULSE pulse = GetCurrentPulse();
	switch (nAxisNo)
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
	default: return 0.0;
	}
}

// 主动读取当前关节位置；机器人返回角度/外部轴位置，PC侧按ini里的轴当量换算为脉冲。
T_ANGLE_PULSE FANUCRobotCtrl::GetCurrentPulse()
{
	T_ANGLE_PULSE pulse;
	return TryGetCurrentPulse(pulse) ? pulse : T_ANGLE_PULSE();
}

bool FANUCRobotCtrl::TryGetCurrentPulse(T_ANGLE_PULSE& pulse)
{
	pulse = T_ANGLE_PULSE();
	std::string response;
	double values[9] = {};
	const std::string command = "GET_CUR_PULSE:" + std::to_string(m_nRobotAxisCount);
	if (!FanucRequest(this, command, response))
	{
		return false;
	}

	if (!FanucStartsWith(response, "PULSE:") || !FanucParseDoubles(FanucResponsePayload(response), values, 9))
	{
		SetLastRobotError("FANUC当前关节响应无效：" + response);
		return false;
	}
	const int activeAxisCount = std::clamp(m_nRobotAxisCount, 6, 9);
	long converted[9] = {};
	for (int axis = 0; axis < activeAxisCount; ++axis)
	{
		const double unit = m_tAxisUnit.GetValueByIndex(axis);
		if (!std::isfinite(values[axis]) || !std::isfinite(unit) || std::abs(unit) < 1e-15)
		{
			SetLastRobotError(GetStr(
				"FANUC当前关节读取失败：轴%d数值或脉冲当量无效。", axis + 1));
			return false;
		}
		const double scaled = values[axis] / unit;
		if (!std::isfinite(scaled)
			|| scaled < static_cast<double>(std::numeric_limits<long>::lowest())
			|| scaled > static_cast<double>(std::numeric_limits<long>::max()))
		{
			SetLastRobotError(GetStr(
				"FANUC当前关节读取失败：轴%d换算结果超出脉冲范围。", axis + 1));
			return false;
		}
		converted[axis] = static_cast<long>(std::lround(scaled));
	}
	pulse = T_ANGLE_PULSE(
		converted[0], converted[1], converted[2],
		converted[3], converted[4], converted[5],
		converted[6], converted[7], converted[8]);
	return true;
}

// 被动读取关节脉冲缓存，同时可取这一帧的机器人侧和PC侧时间戳。
T_ANGLE_PULSE FANUCRobotCtrl::GetCurrentPulsePassive(long long* pRobotMs, long long* pPcRecvMs)
{
	std::lock_guard<std::mutex> lock(m_monitorMutex);
	if (pRobotMs != nullptr)
	{
		*pRobotMs = m_llMonitorRobotMs;
	}
	if (pPcRecvMs != nullptr)
	{
		*pPcRecvMs = m_llMonitorPcRecvMs;
	}
	return m_tMonitorPulse;
}

// 主动读取最近一次 CALL_JOB 程序状态：0=程序运行中，1=程序停止/完成，-1=通信或解析失败。
int FANUCRobotCtrl::CheckDone()
{
	std::string response;
	if (!FanucRequest(this, "CHECK_DONE", response))
	{
		if (GetLastRobotError().empty())
		{
			SetLastRobotError("FANUC完成状态检测失败：CHECK_DONE无响应");
		}
		return -1;
	}
	if (!FanucStartsWith(response, "DONE:"))
	{
		SetLastRobotError("FANUC完成状态检测失败：响应格式异常，RSP=" + response);
		return -1;
	}
	const std::string payload = FanucResponsePayload(response);
	int done = -1;
	if (FanucParseIntStrict(payload, done) && (done == 0 || done == 1))
	{
		return done;
	}
	SetLastRobotError("FANUC完成状态检测失败：DONE仅允许0/1，RSP=" + response);
	return -1;
}

// 被动读取机器人状态缓存，不占用S4控制通道。
int FANUCRobotCtrl::CheckDonePassive(long long* pRobotMs, long long* pPcRecvMs)
{
	std::lock_guard<std::mutex> lock(m_monitorMutex);
	if (pRobotMs != nullptr)
	{
		*pRobotMs = m_llMonitorRobotMs;
	}
	if (pPcRecvMs != nullptr)
	{
		*pPcRecvMs = m_llMonitorPcRecvMs;
	}
	return m_nMonitorDone;
}

// 阻塞等待最近一次 CALL_JOB 程序结束；通信失败会返回-1，避免断线时永久卡住。
int FANUCRobotCtrl::CheckRobotDone(int nDelayTime, int runTimeoutMs)
{
	if (nDelayTime <= 0)
	{
		nDelayTime = 200;
	}
	if (runTimeoutMs <= 0)
	{
		runTimeoutMs = 1800000;
	}

	int nRet = -1;
	int nStableDoneCount = 0;
	bool observedRunning = false;
	const auto waitStartTime = std::chrono::steady_clock::now();
	auto lastLogTime = std::chrono::steady_clock::now();

	while (true)
	{
		if (RobotOperationLease::IsCancellationRequested(this))
		{
			SetLastRobotError("FANUC硬件操作已被安全停止取消，禁止把停机状态判作正常完成。");
			return -20000;
		}
		if (FanucElapsedMs(waitStartTime) >= runTimeoutMs)
		{
			const std::string timeoutError = GetStr(
				"FANUC等待运动完成超时：已等待%lldms，上限=%dms。",
				FanucElapsedMs(waitStartTime), runTimeoutMs);
			if (m_pRobotLog != nullptr)
			{
				m_pRobotLog->write(LogColor::ERR, "%s", timeoutError.c_str());
			}
			const bool stopped = RobotOperationLease::StopAndConfirmUnverifiedMotion(this);
			const std::string stopDetail = GetLastRobotError();
			SetLastRobotError(timeoutError + (stopped
				? "；已自动中止并确认机器人任务终态。"
				: "；自动中止未确认：" + stopDetail));
			return -30000;
		}
		nRet = CheckDone();
		if (nRet < 0)
		{
			const std::string waitError =
				"FANUC等待运动完成失败：" + GetRobotStatusText() + "，最近错误=" + GetLastRobotError();
			if (m_pRobotLog != nullptr)
			{
				m_pRobotLog->write(LogColor::ERR, "FANUC CheckRobotDone 检测失败，返回=%d", nRet);
			}
			const bool stopped = RobotOperationLease::StopAndConfirmUnverifiedMotion(this);
			const std::string stopDetail = GetLastRobotError();
			SetLastRobotError(waitError + (stopped
				? "；已自动中止并确认机器人任务终态。"
				: "；自动中止未确认：" + stopDetail));
			return nRet;
		}
		if (nRet == 0)
		{
			observedRunning = true;
		}

		const long long lastCallJobPcMs = m_llLastCallJobPcMs.load();
		const long long witnessCallMs = m_llCompletionWitnessCallJobPcMs.load();
		const int witnessReg = m_nCompletionWitnessStateReg.load();
		const int witnessDoneState = m_nCompletionWitnessDoneState.load();
		int witnessValue = 0;
		const bool witnessReadOk = witnessReg >= 0
			&& TryGetIntVarStrict(witnessReg, "INT", witnessValue);
		const bool completionWitnessed = lastCallJobPcMs > 0
			&& witnessCallMs == lastCallJobPcMs
			&& witnessReg >= 0
			&& witnessReadOk
			&& witnessValue == witnessDoneState;
		const bool startupGuardActive = nRet != 0
			&& !completionWitnessed
			&& !observedRunning
			&& lastCallJobPcMs > 0
			&& (FanucSteadyMs() - lastCallJobPcMs) < FANUC_DONE_STARTUP_GUARD_MS;

		if (nRet != 0 && !startupGuardActive)
		{
			++nStableDoneCount;
			if (nStableDoneCount >= FANUC_DONE_ACTIVE_STABLE_COUNT)
			{
				if (!completionWitnessed)
				{
					const std::string unknownCompletion = GetStr(
						"FANUC任务已终止，但没有同一次CALL_JOB的程序完成见证：Call=%lld WitnessCall=%lld R[%d]=%d Expected=%d",
						lastCallJobPcMs, witnessCallMs, witnessReg, witnessValue, witnessDoneState);
					const bool stopped = RobotOperationLease::StopAndConfirmUnverifiedMotion(this);
					const std::string stopDetail = GetLastRobotError();
					SetLastRobotError(unknownCompletion + (stopped
						? "；已确认任务终态，但不得报告程序自然完成。"
						: "；终态二次确认失败：" + stopDetail));
					return -31001;
				}
				m_llCompletionWitnessCallJobPcMs.store(0);
				m_nCompletionWitnessStateReg.store(-1);
				m_nCompletionWitnessDoneState.store(0);
				RobotOperationLease::MarkMotionCompleted(this);
				return nRet;
			}
		}
		else
		{
			nStableDoneCount = 0;
			if (m_pRobotLog != nullptr && FanucElapsedMs(lastLogTime) >= 5000)
			{
				m_pRobotLog->write(LogColor::DEFAULT, "FANUC CheckRobotDone 等待机器人程序运行结束...");
				lastLogTime = std::chrono::steady_clock::now();
			}
		}

		Sleep(nDelayTime);
	}
}

bool FANUCRobotCtrl::AbortCurrentProgramSafely()
{
	return Prog_stop_Py();
}

bool FANUCRobotCtrl::HasServiceCapability(const char* capabilityToken)
{
	if (capabilityToken == nullptr || capabilityToken[0] == '\0')
	{
		SetLastRobotError("FANUC机器人侧服务能力标识为空。");
		return false;
	}
	std::string response;
	if (!FanucRequest(this, "GET_USER_PROGRAM", response))
	{
		return false;
	}
	const std::string requiredCapability = capabilityToken;
	if (!FanucStartsWith(response, "PROGRAM:")
		|| response.find("LIB=20260825_ADAPTOR_V3") == std::string::npos
		|| response.find(requiredCapability) == std::string::npos)
	{
		SetLastRobotError(
			"FANUC机器人侧服务缺少适配层所需能力 " + requiredCapability
			+ "，请上传并重启本版本 FanucServiceLib/STARTALL。RSP="
			+ response);
		return false;
	}
	return true;
}

bool FANUCRobotCtrl::HasVerifiedProgramStopCapability()
{
	return HasServiceCapability("ABORT_V2");
}

// 通用程序没有可验证的自然完成契约，公开入口必须 fail-closed。
bool FANUCRobotCtrl::CallJob(std::string sJobName)
{
	SetLastRobotError(
		"FANUC拒绝启动无完成寄存器契约的程序：" + sJobName
		+ "。请使用带完成状态寄存器的受验证入口，或连续点动专用入口。");
	return false;
}

bool FANUCRobotCtrl::CallJobWithCompletionState(
	std::string sJobName,
	int nStateReg,
	int nDoneState)
{
	if (nStateReg < 0 || nDoneState == 0)
	{
		SetLastRobotError(GetStr(
			"FANUC程序完成契约无效：Program=%s R[%d] Expected=%d",
			sJobName.c_str(), nStateReg, nDoneState));
		return false;
	}
	if (RobotOperationLease::IsCancellationRequested(this))
	{
		SetLastRobotError("FANUC硬件操作已被安全停止取消，拒绝启动后续程序：" + sJobName);
		return false;
	}

	std::lock_guard<std::mutex> startLock(m_callJobStartMutex);
	if (RobotOperationLease::MotionCompletionPending(this))
	{
		SetLastRobotError(
			"FANUC拒绝覆盖尚未确认终态的程序完成契约：" + sJobName);
		return false;
	}
	if (!SetIntVar(nStateReg, 0))
	{
		SetLastRobotError(GetStr(
			"FANUC程序启动前清零完成寄存器失败：Program=%s R[%d]",
			sJobName.c_str(), nStateReg));
		return false;
	}
	int resetValue = 0;
	if (!TryGetIntVarStrict(nStateReg, "INT", resetValue) || resetValue != 0)
	{
		SetLastRobotError(GetStr(
			"FANUC程序启动前完成寄存器回读未清零：Program=%s R[%d]=%d",
			sJobName.c_str(), nStateReg, resetValue));
		return false;
	}
	return CallJobInternal(sJobName, nStateReg, nDoneState, false);
}

// 调用方必须持有 m_callJobStartMutex；连续点动使用自身 R80/R81/R82 协议，
// 其完成证明不允许泄漏到公开的通用 CALL_JOB 入口。
bool FANUCRobotCtrl::CallJobInternal(
	const std::string& sJobName,
	int nCompletionStateReg,
	int nCompletionDoneState,
	bool allowManagedUnwitnessed)
{
	const bool hasCompletionWitness = nCompletionStateReg >= 0 && nCompletionDoneState != 0;
	if (!hasCompletionWitness && !allowManagedUnwitnessed)
	{
		SetLastRobotError("FANUC拒绝启动无完成见证的内部程序：" + sJobName);
		return false;
	}
	if (allowManagedUnwitnessed
		&& sJobName != "FANUC_JOGL"
		&& sJobName != "FANUC_JOGJ")
	{
		SetLastRobotError("FANUC拒绝把非连续点动程序当作受管流任务启动：" + sJobName);
		return false;
	}
	if (RobotOperationLease::IsCancellationRequested(this))
	{
		SetLastRobotError("FANUC硬件操作已被安全停止取消，拒绝启动后续程序：" + sJobName);
		return false;
	}

	QString motionError;
	if (!RobotOperationLease::MarkMotionStarted(this, false, &motionError))
	{
		const std::string armError = motionError.toStdString();
		if (RobotOperationLease::MotionCompletionPending(this))
		{
			const bool stopped = RobotOperationLease::StopAndConfirmUnverifiedMotion(this);
			const std::string stopDetail = GetLastRobotError();
			SetLastRobotError(armError + (stopped
				? "；已自动中止并确认上一任务终态。"
				: "；自动中止上一任务未确认：" + stopDetail));
		}
		else
		{
			SetLastRobotError(armError);
		}
		return false;
	}
	m_llCompletionWitnessCallJobPcMs.store(0);
	m_nCompletionWitnessStateReg.store(hasCompletionWitness ? nCompletionStateReg : -1);
	m_nCompletionWitnessDoneState.store(hasCompletionWitness ? nCompletionDoneState : 0);
	// 只有机器人侧真实实现 ABORT_TASK(cancel_mtn=TRUE) 的服务版本才允许启动任务。
	// 旧服务的 PROGRAM_STOP 只返回 OK、并不会停止运动，必须 fail-closed。
	if (!HasVerifiedProgramStopCapability())
	{
		// 尚未发送 CALL_JOB，能力检查失败可安全撤销本次 armed 状态。
		m_llCompletionWitnessCallJobPcMs.store(0);
		m_nCompletionWitnessStateReg.store(-1);
		m_nCompletionWitnessDoneState.store(0);
		RobotOperationLease::MarkMotionCompleted(this);
		return false;
	}
	std::string response;
	bool commandMayHaveBeenSent = false;
	const bool requestOk = FanucRequest(
		this,
		"CALL_JOB:" + sJobName,
		response,
		&commandMayHaveBeenSent);
	const std::string requestError = GetLastRobotError();
	const bool commandDefinitelyNotSent = !requestOk && !commandMayHaveBeenSent;
	const bool ok = requestOk && FanucIsOkResponse(response);
	if (ok)
	{
		const long long callJobPcMs = FanucSteadyMs();
		m_llLastCallJobPcMs.store(callJobPcMs);
		m_llCompletionWitnessCallJobPcMs.store(hasCompletionWitness ? callJobPcMs : 0);
		std::lock_guard<std::mutex> lock(m_monitorMutex);
		m_nMonitorDone = 0;
		m_nMonitorDoneRaw = 0;
		m_nMonitorDoneCandidate = 0;
		m_nMonitorDoneStableCount = 0;
	}
	else
	{
		m_llCompletionWitnessCallJobPcMs.store(0);
		m_nCompletionWitnessStateReg.store(-1);
		m_nCompletionWitnessDoneState.store(0);
		if (commandDefinitelyNotSent)
		{
			// socket层已证明 CALL_JOB 未进入 send；可撤销 armed，
			// 无需用 NO_ACTIVE_CALL_JOB 伪造停止证明。
			RobotOperationLease::MarkMotionCompleted(this);
			if (RobotOperationLease::IsCancellationRequested(this))
			{
				RobotOperationLease::ConfirmCancellationHandled(this);
			}
			SetLastRobotError(requestError);
			return false;
		}
		// 服务明确返回 ERR 时 RUN_TASK 未被接受，可撤销本次 armed 状态；
		// 请求无响应时则保留 pending，异常收尾会尝试安全中止。
		if (requestOk)
		{
			RobotOperationLease::MarkMotionCompleted(this);
		}
		const std::string startError =
			"FANUC调用程序失败：Program=" + sJobName + " RSP=" + response + "，" + GetRobotStatusText();
		if (!requestOk)
		{
			const bool stopped = RobotOperationLease::StopAndConfirmUnverifiedMotion(this);
			const std::string stopDetail = GetLastRobotError();
			SetLastRobotError(startError + (stopped
				? "；命令结果不明，已自动中止并确认机器人任务终态。"
				: "；命令结果不明，自动中止未确认：" + stopDetail));
		}
		else
		{
			SetLastRobotError(startError);
		}
	}
	return ok;
}

bool FANUCRobotCtrl::CallJobAndWaitStateDone(
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
	if (pLastState != nullptr)
	{
		*pLastState = 0;
	}

	if (nDelayTime <= 0)
	{
		nDelayTime = 100;
	}

	if (nStartTimeoutMs <= 0)
	{
		nStartTimeoutMs = 5000;
	}

	if (nFinishTimeoutMs <= 0)
	{
		nFinishTimeoutMs = 10000;
	}

	if (!bResetStateBeforeCall)
	{
		SetLastRobotError(
			"FANUC拒绝复用未清零的完成寄存器；CallJobAndWaitStateDone 必须在本轮启动前清零并回读。");
		return false;
	}

	if (!CallJobWithCompletionState(sJobName, nStateReg, nDoneState))
	{
		if (m_pRobotLog != nullptr)
		{
			m_pRobotLog->write(LogColor::ERR,
				"FANUC CallJobAndWaitStateDone 调用程序失败：Program=%s",
				sJobName.c_str());
		}
		return false;
	}

	if (m_pRobotLog != nullptr)
	{
		m_pRobotLog->write(LogColor::DEFAULT,
			"FANUC CallJobAndWaitStateDone 已调用程序：%s，等待 R[%d] 进入运行态(%d/%d)或完成态(%d)。",
			sJobName.c_str(), nStateReg, nStartStateA, nStartStateB, nDoneState);
	}

	int lastState = 0;
	if (!WaitStateDone(
		nStateReg,
		nDoneState,
		nStartStateA,
		nStartStateB,
		nStartTimeoutMs,
		nFinishTimeoutMs,
		nDelayTime,
		&lastState))
	{
		if (m_pRobotLog != nullptr)
		{
			m_pRobotLog->write(LogColor::ERR,
				"FANUC CallJobAndWaitStateDone 等待状态寄存器失败：Program=%s R[%d]=%d",
				sJobName.c_str(), nStateReg, lastState);
		}
		if (pLastState != nullptr)
		{
			*pLastState = lastState;
		}
		if (!RobotOperationLease::IsCancellationRequested(this))
		{
			const std::string waitError = GetLastRobotError();
			const bool stopped = RobotOperationLease::StopAndConfirmUnverifiedMotion(this);
			const std::string stopDetail = GetLastRobotError();
			SetLastRobotError(waitError + (stopped
				? "；等待失败后已自动中止并确认任务终态。"
				: "；等待失败后的自动中止未确认：" + stopDetail));
		}
		return false;
	}

	const int jobDone = CheckRobotDone(nDelayTime, 30000);
	if (jobDone <= 0)
	{
		if (m_pRobotLog != nullptr)
		{
			m_pRobotLog->write(LogColor::ERR,
				"FANUC CallJobAndWaitStateDone 程序已写完成态，但任务退出失败：Program=%s CheckRobotDone=%d",
				sJobName.c_str(), jobDone);
		}
		if (pLastState != nullptr)
		{
			*pLastState = lastState;
		}
		return false;
	}

	if (m_pRobotLog != nullptr)
	{
		m_pRobotLog->write(LogColor::DEFAULT,
			"FANUC CallJobAndWaitStateDone 完成：Program=%s R[%d]=%d CheckRobotDone=%d",
			sJobName.c_str(), nStateReg, lastState, jobDone);
	}

	if (pLastState != nullptr)
	{
		*pLastState = lastState;
	}
	return true;
}

bool FANUCRobotCtrl::WaitStateDone(
	int nStateReg,
	int nDoneState,
	int nStartStateA,
	int nStartStateB,
	int nStartTimeoutMs,
	int nFinishTimeoutMs,
	int nDelayTime,
	int* pLastState)
{
	if (pLastState != nullptr)
	{
		*pLastState = 0;
	}

	if (nDelayTime <= 0)
	{
		nDelayTime = 100;
	}

	if (nStartTimeoutMs <= 0)
	{
		nStartTimeoutMs = 3000;
	}

	if (nFinishTimeoutMs <= 0)
	{
		nFinishTimeoutMs = 1800000;
	}

	int lastState = 0;
	bool hasStarted = false;
	const auto startDeadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(nStartTimeoutMs);
	while (std::chrono::steady_clock::now() < startDeadline)
	{
		if (RobotOperationLease::IsCancellationRequested(this))
		{
			SetLastRobotError("FANUC等待运动启动期间收到安全停止，流程已取消。");
			return false;
		}
		if (!TryGetIntVarStrict(nStateReg, "INT", lastState))
		{
			SetLastRobotError(GetStr(
				"FANUC等待运动启动时读取状态寄存器失败：R[%d]", nStateReg));
			return false;
		}
		if (lastState == nStartStateA || lastState == nStartStateB || lastState == nDoneState)
		{
			hasStarted = true;
			break;
		}
		Sleep(nDelayTime);
	}

	if (!hasStarted)
	{
		SetLastRobotError(GetStr("FANUC等待运动启动失败：R[%d]=%d，期望运行态=%d/%d 或完成态=%d，%s",
			nStateReg, lastState, nStartStateA, nStartStateB, nDoneState, GetRobotStatusText().c_str()));
		if (m_pRobotLog != nullptr)
		{
			m_pRobotLog->write(LogColor::ERR,
				"FANUC WaitStateDone 未检测到程序启动：R[%d]=%d",
				nStateReg, lastState);
		}
		if (pLastState != nullptr)
		{
			*pLastState = lastState;
		}
		return false;
	}

	if (lastState != nDoneState)
	{
		const auto finishDeadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(nFinishTimeoutMs);
		const auto finishMonitorStart = std::chrono::steady_clock::now();
		bool taskObservedRunning = false;
		int stableStoppedBeforeDone = 0;
		while (std::chrono::steady_clock::now() < finishDeadline)
		{
			if (RobotOperationLease::IsCancellationRequested(this))
			{
				SetLastRobotError("FANUC等待运动完成期间收到安全停止，流程已取消。");
				return false;
			}
			if (!TryGetIntVarStrict(nStateReg, "INT", lastState))
			{
				SetLastRobotError(GetStr(
					"FANUC等待运动完成时读取状态寄存器失败：R[%d]", nStateReg));
				return false;
			}
			if (lastState == nDoneState)
			{
				break;
			}
			const int taskDone = CheckDone();
			if (taskDone < 0)
			{
				SetLastRobotError(GetStr(
					"FANUC等待R[%d]完成期间任务状态读取失败：R[%d]=%d，%s",
					nStateReg, nStateReg, lastState, GetRobotStatusText().c_str()));
				return false;
			}
			if (taskDone == 0)
			{
				taskObservedRunning = true;
				stableStoppedBeforeDone = 0;
			}
			else
			{
				const bool startupGraceExpired = FanucElapsedMs(finishMonitorStart)
					>= FANUC_DONE_STARTUP_GUARD_MS;
				if (taskObservedRunning || startupGraceExpired)
				{
					if (++stableStoppedBeforeDone >= FANUC_DONE_ACTIVE_STABLE_COUNT)
					{
						SetLastRobotError(GetStr(
							"FANUC任务在完成寄存器置位前已终止：R[%d]=%d Expected=%d。",
							nStateReg, lastState, nDoneState));
						return false;
					}
				}
			}
			Sleep(nDelayTime);
		}
	}

	if (lastState != nDoneState)
	{
		SetLastRobotError(GetStr("FANUC等待运动完成失败：R[%d]=%d，期望=%d，%s",
			nStateReg, lastState, nDoneState, GetRobotStatusText().c_str()));
		if (m_pRobotLog != nullptr)
		{
			m_pRobotLog->write(LogColor::ERR,
				"FANUC WaitStateDone 未检测到完成态：R[%d]=%d 期望=%d",
				nStateReg, lastState, nDoneState);
		}
		if (pLastState != nullptr)
		{
			*pLastState = lastState;
		}
		return false;
	}

	if (pLastState != nullptr)
	{
		*pLastState = lastState;
	}
	return true;
}

// 请求机器人侧常驻/监控服务自行退出；随后关闭本地S4/S5连接。
bool FANUCRobotCtrl::StopRobotServices()
{
	// S5 被动通道停掉前先结束基类采样线程；否则其 running 标志保持 true，
	// 后续重连的 StartStateMonitor 会误以为已启动并永久复用旧缓存。
	StopStateMonitor();
	std::string response;
	const bool requestOk = FanucRequest(this, "STOP_SERVICE", response) && FanucIsOkResponse(response);
	CloseSocket();
	StopMonitor();
	return requestOk;
}

namespace
{
	const int FANUC_STREAM_FIRST_PR = 20;
	const int FANUC_STREAM_BUFFER_COUNT = 20;
	const int FANUC_STREAM_START_POINT_COUNT = 20;
	const int FANUC_STREAM_RUN_REG = 80;
	const int FANUC_STREAM_DONE_COUNT_REG = 81;
	const int FANUC_STREAM_END_COUNT_REG = 82;
	const int FANUC_STREAM_LOAD_BUFFER_REG = 83;
	const int FANUC_STREAM_LOAD_STATUS_REG = 84;
	const int FANUC_STREAM_TERMINAL_REG = 85;
	const int FANUC_STREAM_SAFE_GAP = 3;
	const int FANUC_STREAM_STOP_COUNT_TIMEOUT_MS = 5000;
	const int FANUC_STREAM_STOP_JOB_TIMEOUT_MS = 5000;

	int FanucPositiveModulo(long long value, int mod)
	{
		const int result = static_cast<int>(value % mod);
		return result < 0 ? result + mod : result;
	}
}

bool FANUCRobotCtrl::StartContinuousMoveQueue(int nMoveType, double dSpeed)
{
	std::unique_lock<std::mutex> lifecycleLock(m_continuousMoveLifecycleMutex);
	m_continuousMoveLifecycleCv.wait(lifecycleLock, [this]()
		{
			return !m_continuousMoveJoinInProgress;
		});
	if (RobotOperationLease::IsCancellationRequested(this))
	{
		SetLastRobotError("FANUC硬件操作已被安全停止取消，拒绝启动连续点动。");
		return false;
	}
	int requestedSpeedRegister = 0;
	const bool speedRepresentable = nMoveType == MOVL
		? FanucLinearSpeedRegister(dSpeed, requestedSpeedRegister)
		: (requestedSpeedRegister = FanucSpeedPercent(dSpeed)) > 0;
	if (!speedRepresentable)
	{
		SetLastRobotError(GetStr(
			"FANUC连续点动速度无法安全表示：Mode=%s Requested=%.6f。MOVL最低为1mm/sec，且禁止向上取整。",
			nMoveType == MOVL ? "MOVL" : "MOVJ", dSpeed));
		return false;
	}

	// running=true 既表示 worker 正在运行，也表示上一轮停止未得到机器人完成确认。
	// 这两种情况都禁止用“重启队列”覆盖状态，否则会让新命令与仍在运动的机器人重叠。
	if (m_continuousMoveRunning.load())
	{
		if (m_pRobotLog != nullptr)
		{
			m_pRobotLog->write(LogColor::ERR,
				"FANUC 连续运动队列仍在运行或停止未确认，拒绝启动新队列");
		}
		return false;
	}
	if (m_continuousMoveThread.joinable())
	{
		m_continuousMoveThread.join();
	}

	{
		std::lock_guard<std::mutex> lock(m_continuousMoveMutex);
		m_continuousMoveQueue.clear();
		m_continuousMoveStopRequested = false;
		m_continuousMoveRobotStarted = false;
		m_continuousMoveType = nMoveType == MOVL ? MOVL : MOVJ;
		m_continuousMoveSpeed = dSpeed;
		m_continuousWrittenCount = 0;
		m_continuousConsumedCount = 0;
	}

	m_continuousMoveRunning.store(true);
	m_continuousMoveThread = std::thread(&FANUCRobotCtrl::ContinuousMoveWorker, this);
	return true;
}

bool FANUCRobotCtrl::PushContinuousMovePoint(const T_ROBOT_MOVE_INFO& moveInfo)
{
	if (!m_continuousMoveRunning.load())
	{
		return false;
	}

	{
		std::lock_guard<std::mutex> lock(m_continuousMoveMutex);
		m_continuousMoveQueue.push_back(moveInfo);
		m_continuousLastPoint = moveInfo;
	}
	m_continuousMoveCv.notify_one();
	return true;
}

bool FANUCRobotCtrl::PushContinuousMovePoint(const T_ROBOT_COORS& target, double dSpeed)
{
	T_ROBOT_MOVE_INFO moveInfo;
	moveInfo.nMoveType = MOVL;
	moveInfo.tCoord = target;
	moveInfo.tSpeed = T_ROBOT_MOVE_SPEED(dSpeed, 0.0, 0.0);
	return PushContinuousMovePoint(moveInfo);
}

bool FANUCRobotCtrl::PushContinuousMovePoint(const T_ANGLE_PULSE& target, double dSpeed)
{
	T_ROBOT_MOVE_INFO moveInfo;
	moveInfo.nMoveType = MOVJ;
	moveInfo.tPulse = target;
	moveInfo.tSpeed = T_ROBOT_MOVE_SPEED(dSpeed, 0.0, 0.0);
	return PushContinuousMovePoint(moveInfo);
}

void FANUCRobotCtrl::RequestEndContinuousMoveQueue()
{
	if (!m_continuousMoveRunning.load())
	{
		return;
	}

	{
		std::lock_guard<std::mutex> lock(m_continuousMoveMutex);
		m_continuousMoveStopRequested = true;
	}
	m_continuousMoveCv.notify_all();
}

void FANUCRobotCtrl::EndContinuousMoveQueue()
{
	std::thread threadToJoin;
	{
		std::unique_lock<std::mutex> lifecycleLock(m_continuousMoveLifecycleMutex);
		m_continuousMoveLifecycleCv.wait(lifecycleLock, [this]()
			{
				return !m_continuousMoveJoinInProgress;
			});
		if (!m_continuousMoveRunning.load() && !m_continuousMoveThread.joinable())
		{
			return;
		}

		{
			std::lock_guard<std::mutex> lock(m_continuousMoveMutex);
			m_continuousMoveStopRequested = true;
		}
		if (m_continuousMoveThread.joinable()
			&& m_continuousMoveThread.get_id() != std::this_thread::get_id())
		{
			m_continuousMoveJoinInProgress = true;
			m_continuousMoveJoiningThreadId = m_continuousMoveThread.get_id();
			threadToJoin = std::move(m_continuousMoveThread);
		}
	}
	m_continuousMoveCv.notify_all();

	// 绝不能持 lifecycle mutex 等 worker；worker 的异常停止路径也会进入 Finalize。
	if (threadToJoin.joinable())
	{
		threadToJoin.join();
		{
			std::lock_guard<std::mutex> lifecycleLock(m_continuousMoveLifecycleMutex);
			m_continuousMoveJoinInProgress = false;
			m_continuousMoveJoiningThreadId = std::thread::id();
		}
		m_continuousMoveLifecycleCv.notify_all();
	}
	// worker 只有在机器人侧任务真实结束后才会清 running；通信失败/停止未确认时保持 true，
	// 上层租约因而不会释放，控制单元重载与后续运动继续 fail-closed。
	if (m_continuousMoveRunning.load() && m_pRobotLog != nullptr)
	{
		m_pRobotLog->write(LogColor::ERR,
			"FANUC 连续运动停止未得到机器人完成确认，保持互锁占用");
	}
}

bool FANUCRobotCtrl::IsContinuousMoveQueueRunning() const
{
	return m_continuousMoveRunning.load();
}

void FANUCRobotCtrl::FinalizeContinuousMoveAfterVerifiedStop()
{
	// PROGRAM_STOP 已通过机器人侧任务终态稳定回读。此处只负责收拢本地
	// 连续点动 worker，解除“停止未确认”留下的本地 running 闭锁。
	std::thread threadToJoin;
	bool ownsJoin = false;
	{
		std::unique_lock<std::mutex> lifecycleLock(m_continuousMoveLifecycleMutex);
		const bool currentWorkerIsBeingJoined = m_continuousMoveJoinInProgress
			&& m_continuousMoveJoiningThreadId == std::this_thread::get_id();
		if (!currentWorkerIsBeingJoined)
		{
			m_continuousMoveLifecycleCv.wait(lifecycleLock, [this]()
				{
					return !m_continuousMoveJoinInProgress;
				});
		}
		{
			std::lock_guard<std::mutex> lock(m_continuousMoveMutex);
			m_continuousMoveStopRequested = true;
			m_continuousMoveQueue.clear();
		}
		if (m_continuousMoveThread.joinable()
			&& m_continuousMoveThread.get_id() != std::this_thread::get_id())
		{
			m_continuousMoveJoinInProgress = true;
			m_continuousMoveJoiningThreadId = m_continuousMoveThread.get_id();
			threadToJoin = std::move(m_continuousMoveThread);
			ownsJoin = true;
		}
	}
	m_continuousMoveCv.notify_all();
	if (threadToJoin.joinable())
	{
		threadToJoin.join();
	}
	{
		std::lock_guard<std::mutex> lifecycleLock(m_continuousMoveLifecycleMutex);
		m_continuousMoveRobotStarted.store(false);
		m_llCompletionWitnessCallJobPcMs.store(0);
		m_nCompletionWitnessStateReg.store(-1);
		m_nCompletionWitnessDoneState.store(0);
		RobotOperationLease::MarkMotionCompleted(this);
		m_continuousMoveRunning.store(false);
		if (ownsJoin)
		{
			m_continuousMoveJoinInProgress = false;
			m_continuousMoveJoiningThreadId = std::thread::id();
		}
	}
	if (ownsJoin)
	{
		m_continuousMoveLifecycleCv.notify_all();
	}
}

bool FANUCRobotCtrl::WriteContinuousMovePointToRobot(int prIndex, const T_ROBOT_MOVE_INFO& moveInfo)
{
	bool ok = false;
	if (moveInfo.nMoveType == MOVL)
	{
		int config[7] = { 0 };
		double pos[8] =
		{
			moveInfo.tCoord.dX, moveInfo.tCoord.dY, moveInfo.tCoord.dZ,
			moveInfo.tCoord.dRX, moveInfo.tCoord.dRY, moveInfo.tCoord.dRZ,
			moveInfo.tCoord.dBX, moveInfo.tCoord.dBY
		};
		if (m_pRobotLog != nullptr)
		{
			m_pRobotLog->write(LogColor::DEFAULT,
				"FANUC 最终写入连续MOVL点 PR[%d] written=%lld consumed=%lld: X=%.3f Y=%.3f Z=%.3f RX=%.3f RY=%.3f RZ=%.3f BX=%.3f BY=%.3f",
				prIndex,
				m_continuousWrittenCount,
				m_continuousConsumedCount,
				pos[0], pos[1], pos[2], pos[3], pos[4], pos[5], pos[6], pos[7]);
		}
		ok = SetPosVar(prIndex, pos, POSVAR, 1, config, ENGINEEVAR, POSVAR);
	}
	else
	{
		if (m_pRobotLog != nullptr)
		{
			m_pRobotLog->write(LogColor::DEFAULT,
				"FANUC 最终写入连续MOVJ点 PR[%d] written=%lld consumed=%lld: J1=%ld J2=%ld J3=%ld J4=%ld J5=%ld J6=%ld EX1=%ld EX2=%ld EX3=%ld",
				prIndex,
				m_continuousWrittenCount,
				m_continuousConsumedCount,
				moveInfo.tPulse.nSPulse,
				moveInfo.tPulse.nLPulse,
				moveInfo.tPulse.nUPulse,
				moveInfo.tPulse.nRPulse,
				moveInfo.tPulse.nBPulse,
				moveInfo.tPulse.nTPulse,
				moveInfo.tPulse.lBXPulse,
				moveInfo.tPulse.lBYPulse,
				moveInfo.tPulse.lBZPulse);
		}
		ok = SetPosVar(prIndex, moveInfo.tPulse, ENGINEEVAR);
	}

	return ok;
}

bool FANUCRobotCtrl::UploadContinuousStartBufferToRobot(const std::vector<T_ROBOT_MOVE_INFO>& startBuffer)
{
	if (startBuffer.size() != FANUC_STREAM_BUFFER_COUNT)
	{
		return false;
	}

	const std::filesystem::path localDirPath = FanucGeneratedProgramDirectory(this);
	const std::filesystem::path localFilePath = localDirPath / "JOGBUF.DT";
	const std::string localPath = FanucLocalPathBytes(localFilePath);
	const std::string remotePath = "JOGBUF.DT";

	try
	{
		std::filesystem::create_directories(localDirPath);
	}
	catch (...)
	{
		return false;
	}

	std::ostringstream oss;
	oss << std::fixed << std::setprecision(6);
	for (int i = 0; i < FANUC_STREAM_BUFFER_COUNT; ++i)
	{
		const T_ROBOT_MOVE_INFO& info = startBuffer[static_cast<size_t>(i)];
		const int prIndex = FANUC_STREAM_FIRST_PR + i;
		const int posType = info.nMoveType == MOVL ? 0 : 1;
		oss << prIndex << "," << posType;
		if (posType == 0)
		{
			oss << "," << info.tCoord.dX
				<< "," << info.tCoord.dY
				<< "," << info.tCoord.dZ
				<< "," << info.tCoord.dRX
				<< "," << info.tCoord.dRY
				<< "," << info.tCoord.dRZ
				<< "," << info.tCoord.dBX
				<< "," << info.tCoord.dBY
				<< ",0";
			if (m_pRobotLog != nullptr)
			{
				m_pRobotLog->write(LogColor::DEFAULT,
					"FANUC FTP预装连续MOVL点 buffer[%d] PR[%d]: X=%.3f Y=%.3f Z=%.3f RX=%.3f RY=%.3f RZ=%.3f BX=%.3f BY=%.3f BZ=%.3f",
					i,
					prIndex,
					info.tCoord.dX, info.tCoord.dY, info.tCoord.dZ,
					info.tCoord.dRX, info.tCoord.dRY, info.tCoord.dRZ,
					info.tCoord.dBX, info.tCoord.dBY, info.tCoord.dBZ);
			}
		}
		else
		{
			oss << "," << info.tPulse.nSPulse
				<< "," << info.tPulse.nLPulse
				<< "," << info.tPulse.nUPulse
				<< "," << info.tPulse.nRPulse
				<< "," << info.tPulse.nBPulse
				<< "," << info.tPulse.nTPulse
				<< "," << info.tPulse.lBXPulse
				<< "," << info.tPulse.lBYPulse
				<< "," << info.tPulse.lBZPulse;
			if (m_pRobotLog != nullptr)
			{
				m_pRobotLog->write(LogColor::DEFAULT,
					"FANUC FTP预装连续MOVJ点 buffer[%d] PR[%d]: J1=%ld J2=%ld J3=%ld J4=%ld J5=%ld J6=%ld EX1=%ld EX2=%ld EX3=%ld",
					i,
					prIndex,
					info.tPulse.nSPulse,
					info.tPulse.nLPulse,
					info.tPulse.nUPulse,
					info.tPulse.nRPulse,
					info.tPulse.nBPulse,
					info.tPulse.nTPulse,
					info.tPulse.lBXPulse,
					info.tPulse.lBYPulse,
					info.tPulse.lBZPulse);
			}
		}
		oss << "\n";
	}

	if (!FanucWriteTextFile(localPath, oss.str()))
	{
		return false;
	}

	if (m_pFTP == nullptr)
	{
		InitFtp();
	}
	if (m_pFTP == nullptr || !m_pFTP->uploadFile(localPath, remotePath))
	{
		return false;
	}
	delete m_pFTP;
	m_pFTP = nullptr;
	if (m_pRobotLog != nullptr)
	{
		m_pRobotLog->write(LogColor::DEFAULT, "FANUC 点动缓冲FTP上传后已释放连接，等待机器人文件系统刷新");
	}
	std::this_thread::sleep_for(std::chrono::milliseconds(300));

	const bool ok = SetIntVar(FANUC_STREAM_LOAD_BUFFER_REG, 1);
	if (m_pRobotLog != nullptr)
	{
		if (ok)
		{
			m_pRobotLog->write(LogColor::SUCCESS, "FANUC 点动初始缓冲FTP上传完成");
		}
		else
		{
			m_pRobotLog->write(LogColor::ERR, "FANUC 点动初始缓冲FTP标志写入失败");
		}
	}
	return ok;
}

int FANUCRobotCtrl::ReadContinuousDoneCount()
{
	// GetIntVar 的历史失败返回值是 0，无法区分“尚未完成任何点”和断线。
	// 连续点动停机确认必须保留失败语义，因此在这里直接校验协议响应。
	std::string response;
	if (!FanucRequest(this,
		"GET_INT:INT," + std::to_string(FANUC_STREAM_DONE_COUNT_REG), response)
		|| !FanucStartsWith(response, "INT:"))
	{
		return -1;
	}
	const std::string payload = FanucResponsePayload(response);
	char* end = nullptr;
	const long parsed = std::strtol(payload.c_str(), &end, 10);
	if (end == payload.c_str() || end == nullptr || *end != '\0' || parsed < 0 || parsed > 1000000000L)
	{
		SetLastRobotError("FANUC连续点动完成计数响应无效：" + response);
		return -1;
	}
	return static_cast<int>(parsed);
}

void FANUCRobotCtrl::ContinuousMoveWorker()
{
	bool ok = true;
	bool jobAccepted = false;
	bool robotStopConfirmed = false;
	bool hasLastPoint = false;
	T_ROBOT_MOVE_INFO lastPoint;
	const auto waitForStableJobDone = [this](int timeoutMs) -> bool
		{
			int stableDoneCount = 0;
			const auto deadline = std::chrono::steady_clock::now()
				+ std::chrono::milliseconds(timeoutMs);
			while (std::chrono::steady_clock::now() < deadline)
			{
				const int done = CheckDone();
				if (done < 0)
				{
					return false;
				}
				int terminalMarker = 0;
				const bool markerReadOk = TryGetIntVarStrict(
					FANUC_STREAM_TERMINAL_REG, "INT", terminalMarker);
				const long long lastCallJobPcMs = m_llLastCallJobPcMs.load();
				const bool sameCallWitness = markerReadOk
					&& terminalMarker == 1
					&& m_nCompletionWitnessStateReg.load() == FANUC_STREAM_TERMINAL_REG
					&& m_nCompletionWitnessDoneState.load() == 1
					&& m_llCompletionWitnessCallJobPcMs.load() == lastCallJobPcMs
					&& lastCallJobPcMs > 0;
				if (done > 0 && sameCallWitness)
				{
					if (++stableDoneCount >= FANUC_DONE_ACTIVE_STABLE_COUNT)
					{
						m_llCompletionWitnessCallJobPcMs.store(0);
						m_nCompletionWitnessStateReg.store(-1);
						m_nCompletionWitnessDoneState.store(0);
						RobotOperationLease::MarkMotionCompleted(this);
						return true;
					}
				}
				else
				{
					stableDoneCount = 0;
				}
				std::this_thread::sleep_for(std::chrono::milliseconds(50));
			}
			return false;
		};
	const auto setAndVerifyStreamReg = [this](int reg, int expected) -> bool
		{
			if (!SetIntVar(reg, expected))
			{
				SetLastRobotError(GetStr(
					"FANUC连续点动寄存器写入失败：R[%d]=%d", reg, expected));
				return false;
			}
			int actual = 0;
			if (!TryGetIntVarStrict(reg, "INT", actual) || actual != expected)
			{
				SetLastRobotError(GetStr(
					"FANUC连续点动寄存器回读不一致：R[%d]=%d Expected=%d",
					reg, actual, expected));
				return false;
			}
			return true;
		};
	const auto startManagedStreamJob = [this, &setAndVerifyStreamReg](const std::string& programName) -> bool
		{
			std::lock_guard<std::mutex> startLock(m_callJobStartMutex);
			// R[85] 是本次流式 TP 的自然完成见证。清零、严格回读、arm 与 CALL
			// 必须紧邻且共用启动锁；TP 首行也再次清零，以覆盖机器人端残留旧程序
			// 在主机回读之后、CALL 之前才写入旧终态的极窄窗口。
			return setAndVerifyStreamReg(FANUC_STREAM_TERMINAL_REG, 0)
				&& CallJobInternal(programName, FANUC_STREAM_TERMINAL_REG, 1, true);
		};

	const bool initRunOk = setAndVerifyStreamReg(FANUC_STREAM_RUN_REG, 0);
	const bool initDoneOk = setAndVerifyStreamReg(FANUC_STREAM_DONE_COUNT_REG, 0);
	const bool initEndOk = setAndVerifyStreamReg(FANUC_STREAM_END_COUNT_REG, 0);
	const bool initLoadBufferOk = setAndVerifyStreamReg(FANUC_STREAM_LOAD_BUFFER_REG, 0);
	const bool initLoadStatusOk = setAndVerifyStreamReg(FANUC_STREAM_LOAD_STATUS_REG, 0);
	const bool initTerminalOk = setAndVerifyStreamReg(FANUC_STREAM_TERMINAL_REG, 0);
	ok = initRunOk && initDoneOk && initEndOk && initLoadBufferOk && initLoadStatusOk
		&& initTerminalOk;

	std::vector<T_ROBOT_MOVE_INFO> startBuffer;
	startBuffer.reserve(FANUC_STREAM_START_POINT_COUNT);
	while (static_cast<int>(startBuffer.size()) < FANUC_STREAM_START_POINT_COUNT && ok)
	{
		T_ROBOT_MOVE_INFO nextPoint;
		bool hasPoint = false;
		{
			std::unique_lock<std::mutex> lock(m_continuousMoveMutex);
			m_continuousMoveCv.wait(lock, [this]()
				{
					return m_continuousMoveStopRequested || !m_continuousMoveQueue.empty();
				});

			if (!m_continuousMoveQueue.empty())
			{
				nextPoint = m_continuousMoveQueue.front();
				m_continuousMoveQueue.pop_front();
				hasPoint = true;
			}
			else if (m_continuousMoveStopRequested && hasLastPoint)
			{
				nextPoint = lastPoint;
				hasPoint = true;
			}
		}

		if (!hasPoint)
		{
			ok = false;
			break;
		}

		startBuffer.push_back(nextPoint);
		lastPoint = nextPoint;
		hasLastPoint = true;
	}

	if (ok && hasLastPoint)
	{
		ok = UploadContinuousStartBufferToRobot(startBuffer);
		if (!ok)
		{
			for (int i = 0; i < static_cast<int>(startBuffer.size()); ++i)
			{
				ok = WriteContinuousMovePointToRobot(FANUC_STREAM_FIRST_PR + i, startBuffer[static_cast<size_t>(i)]);
				if (!ok)
				{
					break;
				}
			}
		}
		if (ok)
		{
			m_continuousWrittenCount = static_cast<long long>(startBuffer.size());
		}
	}

	if (ok && hasLastPoint)
	{
		int speedReg = 0;
		const bool speedRepresentable = m_continuousMoveType == MOVL
			? FanucLinearSpeedRegister(m_continuousMoveSpeed, speedReg)
			: (speedReg = FanucSpeedPercent(m_continuousMoveSpeed)) > 0;
		const bool speedWriteOk = speedRepresentable && SetTpSpeed(speedReg);
		int speedReadback = 0;
		const bool speedReadbackOk = speedWriteOk
			&& TryGetIntVarStrict(17, "INT", speedReadback)
			&& speedReadback == speedReg;
		if (!speedReadbackOk)
		{
			SetLastRobotError(GetStr(
				"FANUC连续点动速度未可靠写入：R[17]=%d Expected=%d",
				speedReadback, speedReg));
		}
		const bool loadStatusResetOk = speedReadbackOk
			&& setAndVerifyStreamReg(FANUC_STREAM_LOAD_STATUS_REG, 0);
		const bool runFlagOk = loadStatusResetOk
			&& setAndVerifyStreamReg(FANUC_STREAM_RUN_REG, 1);
		jobAccepted = speedReadbackOk && runFlagOk
			&& startManagedStreamJob(m_continuousMoveType == MOVL ? "FANUC_JOGL" : "FANUC_JOGJ");
		ok = jobAccepted;

		int loadStatus = 0;
		if (ok)
		{
			for (int retry = 0; retry < 50; ++retry)
			{
				loadStatus = GetIntVar(FANUC_STREAM_LOAD_STATUS_REG);
				if (loadStatus != 0 && loadStatus != -100)
				{
					break;
				}
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
			}

			if (m_pRobotLog != nullptr)
			{
				m_pRobotLog->write(loadStatus == 1 ? LogColor::SUCCESS : LogColor::ERR,
					"FANUC 连续运动LOADJOGBUF状态: R[%d]=%d",
					FANUC_STREAM_LOAD_STATUS_REG,
					loadStatus);
			}

			if (loadStatus == -201)
			{
				if (m_pRobotLog != nullptr)
				{
					m_pRobotLog->write(LogColor::WARNING, "FANUC 点动缓冲文件打开失败，切换为直接写PR兜底模式");
				}

				SetIntVar(FANUC_STREAM_RUN_REG, 0);
				// 第一轮 TP 已被 CALL_JOB 接受，必须先证明它已经退出，才能再次
				// CALL_JOB 同名程序；否则服务端 active task 会被覆盖且两任务可能并行。
				ok = !jobAccepted || waitForStableJobDone(FANUC_STREAM_STOP_JOB_TIMEOUT_MS);
				if (ok)
				{
					jobAccepted = false;
				}
				for (int i = 0; i < static_cast<int>(startBuffer.size()); ++i)
				{
					if (!ok)
					{
						break;
					}
					ok = WriteContinuousMovePointToRobot(FANUC_STREAM_FIRST_PR + i, startBuffer[static_cast<size_t>(i)]);
					if (!ok)
					{
						break;
					}
				}

				if (ok)
				{
					const bool fallbackLoadModeOk = setAndVerifyStreamReg(FANUC_STREAM_LOAD_BUFFER_REG, 2);
					const bool fallbackStatusOk = fallbackLoadModeOk
						&& setAndVerifyStreamReg(FANUC_STREAM_LOAD_STATUS_REG, 0);
					const bool fallbackTerminalOk = fallbackStatusOk
						&& setAndVerifyStreamReg(FANUC_STREAM_TERMINAL_REG, 0);
					const bool fallbackRunFlagOk = fallbackTerminalOk
						&& setAndVerifyStreamReg(FANUC_STREAM_RUN_REG, 1);
					jobAccepted = fallbackLoadModeOk && fallbackStatusOk
						&& fallbackTerminalOk && fallbackRunFlagOk
						&& startManagedStreamJob(m_continuousMoveType == MOVL ? "FANUC_JOGL" : "FANUC_JOGJ");
					ok = jobAccepted;
				}

				if (ok)
				{
					loadStatus = 0;
					for (int retry = 0; retry < 50; ++retry)
					{
						loadStatus = GetIntVar(FANUC_STREAM_LOAD_STATUS_REG);
						if (loadStatus != 0 && loadStatus != -100)
						{
							break;
						}
						std::this_thread::sleep_for(std::chrono::milliseconds(100));
					}
					if (m_pRobotLog != nullptr)
					{
						m_pRobotLog->write(loadStatus == 1 ? LogColor::SUCCESS : LogColor::ERR,
							"FANUC 连续运动直接写PR兜底LOADJOGBUF状态: R[%d]=%d",
							FANUC_STREAM_LOAD_STATUS_REG,
							loadStatus);
					}
				}
			}

			if (loadStatus == 1 && m_pRobotLog != nullptr)
			{
				std::string pr20Response;
				if (FanucRequest(this, "GET_POS_VAR:20,0", pr20Response))
				{
					m_pRobotLog->write(LogColor::DEFAULT,
						"FANUC 连续运动LOADJOGBUF后PR20诊断: %s",
						pr20Response.c_str());
				}
				else
				{
					m_pRobotLog->write(LogColor::ERR, "FANUC 连续运动LOADJOGBUF后PR20诊断读取失败");
				}
			}

			if (loadStatus != 1)
			{
				SetIntVar(FANUC_STREAM_RUN_REG, 0);
				ok = false;
			}
		}
		m_continuousMoveRobotStarted = jobAccepted;
		m_continuousConsumedCount = std::max(0, ReadContinuousDoneCount());
	}

	while (ok && jobAccepted)
	{
		bool stopRequested = false;
		{
			std::lock_guard<std::mutex> lock(m_continuousMoveMutex);
			stopRequested = m_continuousMoveStopRequested;
		}
		stopRequested = stopRequested || RobotOperationLease::IsCancellationRequested(this);

		if (stopRequested)
		{
			// 松手后不再消费尚未写入机器人侧的 UI 队列。
			{
				std::lock_guard<std::mutex> lock(m_continuousMoveMutex);
				m_continuousMoveQueue.clear();
			}
			const int observedDoneCount = ReadContinuousDoneCount();
			if (observedDoneCount >= 0)
			{
				m_continuousConsumedCount = std::max<long long>(
					m_continuousConsumedCount, observedDoneCount);
			}
			// R80 变为 0 后，TP 会在每个运动点前检查 R81>=R82。目标只取当前
			// 已完成计数的下一点（并受已写点数上限约束），避免把预装的 20 点全部跑完。
			const long long expectedDoneCount = std::min<long long>(
				m_continuousWrittenCount,
				std::max<long long>(0, m_continuousConsumedCount) + 1);
			if (m_pRobotLog != nullptr)
			{
				m_pRobotLog->write(LogColor::DEFAULT,
					"FANUC 连续运动收到停止请求: written=%lld endCount=%lld consumed=%lld observed=%d",
					m_continuousWrittenCount,
					expectedDoneCount,
					m_continuousConsumedCount,
					observedDoneCount);
			}
			const bool endCountOk = SetIntVar(
				FANUC_STREAM_END_COUNT_REG, static_cast<int>(expectedDoneCount));
			const bool stopOk = SetIntVar(FANUC_STREAM_RUN_REG, 0);
			if ((!endCountOk || !stopOk) && m_pRobotLog != nullptr)
			{
				m_pRobotLog->write(LogColor::ERR,
					"FANUC 连续点动停止寄存器写入不完整：endCount=%d runFlag=%d；继续以任务终态回读兜底",
					endCountOk ? 1 : 0,
					stopOk ? 1 : 0);
			}

			// TP 程序会把已完成点数写入 R[81]，直到追上 R[82] 才退出。
			// 有限时等待，避免断线令 worker/destructor 永久 join；超时仍保持 running=true。
			bool countConfirmed = false;
			int consecutiveReadFailures = 0;
			const auto countDeadline = std::chrono::steady_clock::now()
				+ std::chrono::milliseconds(FANUC_STREAM_STOP_COUNT_TIMEOUT_MS);
			while (std::chrono::steady_clock::now() < countDeadline)
			{
				const int doneCount = ReadContinuousDoneCount();
				if (doneCount >= expectedDoneCount)
				{
					m_continuousConsumedCount = doneCount;
					countConfirmed = true;
					break;
				}
				if (doneCount < 0 && ++consecutiveReadFailures >= 3)
				{
					break;
				}
				if (doneCount >= 0)
				{
					consecutiveReadFailures = 0;
					m_continuousConsumedCount = std::max<long long>(m_continuousConsumedCount, doneCount);
				}
				std::this_thread::sleep_for(std::chrono::milliseconds(20));
			}

			// R[81] 只用于诊断点消费边界；只要 TP 任务终态得到稳定确认，
			// 即使计数回读失败也可以安全释放连续点动租约。
			robotStopConfirmed = waitForStableJobDone(FANUC_STREAM_STOP_JOB_TIMEOUT_MS);
			if (!countConfirmed && m_pRobotLog != nullptr)
			{
				m_pRobotLog->write(robotStopConfirmed ? LogColor::WARNING : LogColor::ERR,
					"FANUC 连续点动完成计数未确认，任务终态确认=%d",
					robotStopConfirmed ? 1 : 0);
			}
			ok = robotStopConfirmed;
			if (robotStopConfirmed)
			{
				jobAccepted = false;
			}
			break;
		}

		const int doneCount = ReadContinuousDoneCount();
		if (doneCount >= 0 && doneCount > m_continuousConsumedCount)
		{
			m_continuousConsumedCount = doneCount;
		}

		T_ROBOT_MOVE_INFO nextPoint;
		bool hasPoint = false;
		{
			std::lock_guard<std::mutex> lock(m_continuousMoveMutex);
			const long long outstanding = m_continuousWrittenCount - m_continuousConsumedCount;
			if (!m_continuousMoveQueue.empty() &&
				outstanding < FANUC_STREAM_BUFFER_COUNT - FANUC_STREAM_SAFE_GAP)
			{
				nextPoint = m_continuousMoveQueue.front();
				m_continuousMoveQueue.pop_front();
				hasPoint = true;
			}
		}

		if (hasPoint)
		{
			const int slot = FanucPositiveModulo(m_continuousWrittenCount, FANUC_STREAM_BUFFER_COUNT);
			ok = WriteContinuousMovePointToRobot(FANUC_STREAM_FIRST_PR + slot, nextPoint);
			if (ok)
			{
				lastPoint = nextPoint;
				hasLastPoint = true;
				++m_continuousWrittenCount;
			}
		}
		else
		{
			std::unique_lock<std::mutex> lock(m_continuousMoveMutex);
			m_continuousMoveCv.wait_for(lock, std::chrono::milliseconds(20));
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}

	// CALL_JOB 一旦被接受，后续任意握手/写点失败都不能按普通失败退出；
	// 先尝试停止标志并稳定读取任务终态，未确认则保持 running/lease。
	if (jobAccepted && !robotStopConfirmed)
	{
		SetIntVar(FANUC_STREAM_RUN_REG, 0);
		robotStopConfirmed = waitForStableJobDone(FANUC_STREAM_STOP_JOB_TIMEOUT_MS);
		if (robotStopConfirmed)
		{
			jobAccepted = false;
		}
	}

	if (!ok)
	{
		SetIntVar(FANUC_STREAM_RUN_REG, 0);
		if (m_pRobotLog != nullptr)
		{
			m_pRobotLog->write(LogColor::ERR, "FANUC 连续运动队列异常结束");
		}
	}
	else if (m_pRobotLog != nullptr)
	{
		m_pRobotLog->write(LogColor::DEFAULT,
			"FANUC 连续运动线程结束: written=%lld consumed=%lld robotStarted=%d",
			m_continuousWrittenCount,
			m_continuousConsumedCount,
			m_continuousMoveRobotStarted.load() ? 1 : 0);
	}

	m_continuousMoveRobotStarted = jobAccepted;
	if (!jobAccepted || robotStopConfirmed)
	{
		m_continuousMoveRunning.store(false);
	}
	else
	{
		// 无法证明机器人侧任务已经结束时保持 true；上层不得释放租约或开始下一动作。
		m_continuousMoveRunning.store(true);
		if (m_pRobotLog != nullptr)
		{
			m_pRobotLog->write(LogColor::ERR,
				"FANUC 连续运动线程退出但机器人停止未确认，保持运行互锁");
		}
	}
}

// ===================== 被动监控通道 =====================

void FANUCRobotCtrl::PrepareStateMonitor()
{
	StartMonitor();
}

// 启动S5监控线程；端口默认来自RobotPara.ini的MonitorPort。
bool FANUCRobotCtrl::StartMonitor(int nPort)
{
	// The monitor channel is intentionally separate from the control socket.
	// UI/status polling reads this cache so high-frequency display updates do not block motion commands.
	if (m_bMonitorRunning)
	{
		return true;
	}
	if (nPort <= 0)
	{
		nPort = m_nMonitorPort > 0 ? m_nMonitorPort : 9001;
	}
	if (m_sSocketIP.empty())
	{
		return false;
	}

	if (!FanucInitWinSock())
	{
		if (m_pRobotLog != nullptr)
		{
			m_pRobotLog->write(LogColor::ERR, "FANUC Monitor 启动失败：WSAStartup失败");
		}
		return false;
	}
	m_bMonitorWSAStarted = true;

	m_bMonitorRunning = true;
	m_monitorThread = std::thread([this, nPort]()
		{
			while (m_bMonitorRunning)
			{
				SOCKET sock = FanucInvalidSocket();
				if (!FanucConnectSocket(sock, m_sSocketIP, static_cast<unsigned short>(nPort)))
				{
					{
						std::lock_guard<std::mutex> lock(m_monitorMutex);
						m_sMonitorText = GetStr("状态: 监控连接失败 %s:%d", m_sSocketIP.c_str(), nPort);
					}
					Sleep(1000);
					continue;
				}

				m_uMonitorSocketHandle.store(static_cast<std::uintptr_t>(sock));
				if (m_pRobotLog != nullptr)
				{
					m_pRobotLog->write(LogColor::SUCCESS, "FANUC Monitor 已连接：%s:%d", m_sSocketIP.c_str(), nPort);
				}

				while (m_bMonitorRunning)
				{
					std::string line;
					if (!FanucReceiveLine(sock, line, 1000))
					{
						break;
					}

					const long long pcRecvMs = FanucSteadyMs();
					std::lock_guard<std::mutex> lock(m_monitorMutex);
					m_sMonitorText = GetStr("%s\npc_recv_ms=%lld", line.c_str(), pcRecvMs);
					// Keep the last complete monitor frame as a passive-read cache.
					T_ROBOT_COORS pos;
					T_ANGLE_PULSE pulse;
					int done = -1;
					long long robotMs = 0;
					if (FanucParseMonitorFrame(line, pos, pulse, done, robotMs, m_tAxisUnit))
					{
						m_tMonitorPos = pos;
						m_tMonitorPulse = pulse;
						m_nMonitorDoneRaw = done;
						m_llMonitorRobotMs = robotMs;
						m_llMonitorPcRecvMs = pcRecvMs;

						if (done < 0)
						{
							m_nMonitorDone = -1;
							m_nMonitorDoneCandidate = -1;
							m_nMonitorDoneStableCount = 0;
						}
						else if (done == 0)
						{
							m_nMonitorDone = 0;
							m_nMonitorDoneCandidate = 0;
							m_nMonitorDoneStableCount = 0;
						}
						else
						{
							if (m_nMonitorDoneCandidate == done)
							{
								++m_nMonitorDoneStableCount;
							}
							else
							{
								m_nMonitorDoneCandidate = done;
								m_nMonitorDoneStableCount = 1;
							}

							const long long lastCallJobPcMs = m_llLastCallJobPcMs.load();
							const bool startupGuardActive = (lastCallJobPcMs > 0 && (pcRecvMs - lastCallJobPcMs) < FANUC_DONE_STARTUP_GUARD_MS);
							if (!startupGuardActive && m_nMonitorDoneStableCount >= FANUC_DONE_PASSIVE_STABLE_COUNT)
							{
								m_nMonitorDone = done;
							}
							else
							{
								m_nMonitorDone = 0;
							}
						}
					}
				}

				std::uintptr_t expectedHandle = static_cast<std::uintptr_t>(sock);
				const std::uintptr_t invalidHandle = static_cast<std::uintptr_t>(FanucInvalidSocket());
				if (m_uMonitorSocketHandle.compare_exchange_strong(expectedHandle, invalidHandle))
				{
					closesocket(sock);
				}
				if (m_bMonitorRunning)
				{
					std::lock_guard<std::mutex> lock(m_monitorMutex);
					m_sMonitorText = "状态: 监控连接断开，正在重连...";
					m_nMonitorDone = -1;
					m_nMonitorDoneRaw = -1;
					m_nMonitorDoneCandidate = -1;
					m_nMonitorDoneStableCount = 0;
					Sleep(500);
				}
			}
		});

	return true;
}

// 停止S5监控线程并关闭监控socket。
void FANUCRobotCtrl::StopMonitor()
{
	m_bMonitorRunning = false;
	const std::uintptr_t invalidHandle = static_cast<std::uintptr_t>(FanucInvalidSocket());
	SOCKET sock = FanucGetSocket(m_uMonitorSocketHandle.exchange(invalidHandle));
	if (sock != INVALID_SOCKET)
	{
		shutdown(sock, SD_BOTH);
		closesocket(sock);
	}
	if (m_monitorThread.joinable())
	{
		m_monitorThread.join();
	}
	if (m_bMonitorWSAStarted)
	{
		FanucCleanupWinSock();
		m_bMonitorWSAStarted = false;
	}
}

// 返回最后一帧监控原始文本，主要用于UI快速显示/现场排查。
std::string FANUCRobotCtrl::GetMonitorText()
{
	std::lock_guard<std::mutex> lock(m_monitorMutex);
	return m_sMonitorText;
}

std::string FANUCRobotCtrl::GetRobotStatusText()
{
	std::ostringstream oss;
	oss << "FANUC状态：连接=" << (m_bSocketConnected.load() ? "已连接" : "未连接");
	{
		std::lock_guard<std::mutex> lock(m_monitorMutex);
		oss << "，被动Done=" << m_nMonitorDone
			<< "，原始Done=" << m_nMonitorDoneRaw
			<< "，robot_ms=" << m_llMonitorRobotMs
			<< "，pc_recv_ms=" << m_llMonitorPcRecvMs;
		if (!m_sMonitorText.empty())
		{
			oss << "，监控=" << m_sMonitorText;
		}
	}
	return oss.str();
}

// ===================== 连续运动与特殊程序上传 =====================

// 多点连续运动：生成临时KL/VAR文件并上传，只允许纯运动数据。
// FANUC 实焊必须走独立且已验证的 ArcTool 契约，禁止从旧接口旁路。
int FANUCRobotCtrl::ContiMoveAny(const std::vector<T_ROBOT_MOVE_INFO>& vtRobotMoveInfo)
{
	if (vtRobotMoveInfo.empty())
	{
		if (m_pRobotLog != nullptr)
		{
			m_pRobotLog->write(LogColor::ERR, "FANUC ContiMoveAny 失败：轨迹点为空");
		}
		return -1;
	}
	const auto weldMetadataIt = std::find_if(
		vtRobotMoveInfo.begin(),
		vtRobotMoveInfo.end(),
		[](const T_ROBOT_MOVE_INFO& info) { return FanucContainsWeldMetadata(info); });
	if (weldMetadataIt != vtRobotMoveInfo.end())
	{
		const size_t index = static_cast<size_t>(std::distance(vtRobotMoveInfo.begin(), weldMetadataIt));
		const std::string reason = GetStr(
			"FANUC旧连续运动接口包含焊接元数据，已拒绝生成KL/VAR：Index=%u。实焊不得旁路ArcTool契约。",
			static_cast<unsigned>(index));
		SetLastRobotError(reason);
		if (m_pRobotLog != nullptr)
		{
			m_pRobotLog->write(LogColor::ERR, "%s", reason.c_str());
		}
		return FANUC_DRY_RUN_CONTAINS_WELD_METADATA;
	}

	const auto externalAxisIt = std::find_if(
		vtRobotMoveInfo.begin(),
		vtRobotMoveInfo.end(),
		[](const T_ROBOT_MOVE_INFO& info) { return FanucContainsExternalAxisTarget(info); });
	if (externalAxisIt != vtRobotMoveInfo.end())
	{
		const size_t index = static_cast<size_t>(std::distance(vtRobotMoveInfo.begin(), externalAxisIt));
		SetLastRobotError(GetStr(
			"FANUC轨迹包含外部轴目标，但当前LS/TP生成器只实现GP1/J1-J6：Index=%u。",
			static_cast<unsigned>(index)));
		return -7;
	}

	const std::string timestamp = FanucMakeTimestamp();
	const std::string programName = FanucMakeProgramName(this);
	const std::filesystem::path localDirPath = FanucGeneratedProgramDirectory(this);
	const std::string klFileName = programName + ".kl";
	const std::string pcFileName = programName + ".pc";
	const std::string varFileName = programName + ".var";
	const std::string localKlPath = FanucLocalPathBytes(localDirPath / klFileName);
	const std::string localPcPath = FanucLocalPathBytes(localDirPath / pcFileName);
	const std::string localVarPath = FanucLocalPathBytes(localDirPath / varFileName);
	const std::string remotePcPath = "/md/" + pcFileName;
	const std::string remoteVarPath = "/md/" + varFileName;

	try
	{
		std::filesystem::create_directories(localDirPath);
	}
	catch (const std::exception& e)
	{
		if (m_pRobotLog != nullptr)
		{
			m_pRobotLog->write(LogColor::ERR, "FANUC ContiMoveAny 创建目录失败：%s", e.what());
		}
		return -2;
	}

	const std::string klContent = FanucBuildKlContent(programName, varFileName, vtRobotMoveInfo);
	const std::string varContent = FanucBuildVarContent(programName, vtRobotMoveInfo);

	{
		std::lock_guard<std::mutex> filePairLock(g_fanucGeneratedFilePairMutex);
		if (!FanucWriteTextFile(localKlPath, klContent) || !FanucWriteTextFile(localVarPath, varContent))
		{
			if (m_pRobotLog != nullptr)
			{
				m_pRobotLog->write(LogColor::ERR, "FANUC ContiMoveAny 生成文件失败：%s 或 %s", localKlPath.c_str(), localVarPath.c_str());
			}
			return -3;
		}
	}

	if (m_pRobotLog != nullptr)
	{
		for (size_t i = 0; i < vtRobotMoveInfo.size(); ++i)
		{
			FanucLogMovePoint(this, "FANUC ContiMoveAny生成点", static_cast<int>(i + 1), vtRobotMoveInfo[i], &m_tAxisUnit);
		}
		m_pRobotLog->write(LogColor::SUCCESS,
			"FANUC ContiMoveAny 已生成轨迹文件 | Program=%s | PointCount=%d | Time=%s",
			programName.c_str(), static_cast<int>(vtRobotMoveInfo.size()), timestamp.c_str());
		m_pRobotLog->write(LogColor::DEFAULT, "本地KL文件：%s", localKlPath.c_str());
		m_pRobotLog->write(LogColor::DEFAULT, "本地VAR文件：%s", localVarPath.c_str());
	}

	if (!FanucCompileKlToPc(localKlPath, localPcPath, m_pRobotLog))
	{
		return -4;
	}

	if (m_pFTP == nullptr)
	{
		InitFtp();
	}
	if (m_pFTP == nullptr)
	{
		if (m_pRobotLog != nullptr)
		{
			m_pRobotLog->write(LogColor::ERR, "FANUC ContiMoveAny 初始化FTP失败");
		}
		return -5;
	}

	if (!m_pFTP->uploadFile(localPcPath, remotePcPath))
	{
		if (m_pRobotLog != nullptr)
		{
			m_pRobotLog->write(LogColor::ERR, "FANUC ContiMoveAny 上传PC失败：%s", remotePcPath.c_str());
		}
		return -6;
	}

	if (!m_pFTP->uploadFile(localVarPath, remoteVarPath))
	{
		if (m_pRobotLog != nullptr)
		{
			m_pRobotLog->write(LogColor::ERR, "FANUC ContiMoveAny 上传VAR失败：%s", remoteVarPath.c_str());
		}
		return -7;
	}

	if (m_pRobotLog != nullptr)
	{
		m_pRobotLog->write(LogColor::SUCCESS,
			"FANUC ContiMoveAny 上传完成 | PC=%s | VAR=%s",
			remotePcPath.c_str(), remoteVarPath.c_str());
	}

	return 0;
}

bool FANUCRobotCtrl::HasVerifiedArcWeldContract(std::string* pReason) const
{
	if (pReason != nullptr)
	{
		*pReason = FANUC_ACTUAL_WELD_UNAVAILABLE_REASON;
	}
	return false;
}

int FANUCRobotCtrl::UploadMultiPointTpProgram(
	const std::vector<T_ROBOT_MOVE_INFO>& vtRobotMoveInfo,
	TrajectoryProgramMode mode,
	std::string* pProgramName,
	std::string* pLocalLsPath,
	std::string* pRemoteTpPath,
	const std::string& requestedProgramName)
{
	if (vtRobotMoveInfo.empty())
	{
		if (m_pRobotLog != nullptr)
		{
			m_pRobotLog->write(LogColor::ERR, "FANUC UploadMultiPointTpProgram 失败：轨迹点为空");
		}
		return -1;
	}

	if (mode == TrajectoryProgramMode::ActualWeld)
	{
		std::string reason;
		HasVerifiedArcWeldContract(&reason);
		SetLastRobotError(reason);
		if (m_pRobotLog != nullptr)
		{
			m_pRobotLog->write(LogColor::ERR, "%s", reason.c_str());
		}
		return FANUC_ACTUAL_WELD_CONTRACT_MISSING;
	}

	const auto weldMetadataIt = std::find_if(
		vtRobotMoveInfo.begin(),
		vtRobotMoveInfo.end(),
		[](const T_ROBOT_MOVE_INFO& info) { return FanucContainsWeldMetadata(info); });
	if (weldMetadataIt != vtRobotMoveInfo.end())
	{
		const size_t index = static_cast<size_t>(std::distance(vtRobotMoveInfo.begin(), weldMetadataIt));
		const std::string reason = GetStr(
			"FANUC空跑轨迹包含焊接元数据，已拒绝生成普通TP：Index=%u。禁止静默丢弃起弧/收弧/摆动/跟踪字段。",
			static_cast<unsigned>(index));
		SetLastRobotError(reason);
		if (m_pRobotLog != nullptr)
		{
			m_pRobotLog->write(LogColor::ERR, "%s", reason.c_str());
		}
		return FANUC_DRY_RUN_CONTAINS_WELD_METADATA;
	}

	const auto tpExternalAxisIt = std::find_if(
		vtRobotMoveInfo.begin(),
		vtRobotMoveInfo.end(),
		[](const T_ROBOT_MOVE_INFO& info) { return FanucContainsExternalAxisTarget(info); });
	if (tpExternalAxisIt != vtRobotMoveInfo.end())
	{
		const size_t index = static_cast<size_t>(std::distance(vtRobotMoveInfo.begin(), tpExternalAxisIt));
		SetLastRobotError(GetStr(
			"FANUC轨迹包含外部轴目标，但当前LS/TP生成器只实现GP1/J1-J6：Index=%u。",
			static_cast<unsigned>(index)));
		return -7;
	}

	for (size_t index = 0; index < vtRobotMoveInfo.size(); ++index)
	{
		const T_ROBOT_MOVE_INFO& info = vtRobotMoveInfo[index];
		int speedRegister = 0;
		const bool speedOk = info.nMoveType == MOVL
			? FanucLinearSpeedRegister(info.tSpeed.dSpeed, speedRegister)
			: (speedRegister = FanucSpeedPercent(info.tSpeed.dSpeed)) > 0;
		if (!speedOk)
		{
			SetLastRobotError(GetStr(
				"FANUC轨迹点速度无法安全表示：Index=%u Mode=%s Requested=%.6f",
				static_cast<unsigned>(index),
				info.nMoveType == MOVL ? "MOVL" : "MOVJ",
				info.tSpeed.dSpeed));
			return -6;
		}
	}

	const std::string programName = requestedProgramName.empty()
		? FanucMakeTpProgramName(this)
		: requestedProgramName;
	if (!FanucIsValidControllerProgramName(programName))
	{
		SetLastRobotError("FANUC轨迹程序名非法：必须以字母开头，仅含字母、数字或下划线，且不超过31字符。");
		return -8;
	}
	const std::filesystem::path localDirPath = FanucGeneratedProgramDirectory(this);
	const std::string lsFileName = programName + ".ls";
	const std::string localLsPath = FanucLocalPathBytes(localDirPath / lsFileName);
	const std::string remoteTpPath = "/md/" + programName + ".tp";

	try
	{
		std::filesystem::create_directories(localDirPath);
	}
	catch (const std::exception& e)
	{
		if (m_pRobotLog != nullptr)
		{
			m_pRobotLog->write(LogColor::ERR, "FANUC UploadMultiPointTpProgram 创建目录失败：%s", e.what());
		}
		return -2;
	}

	const std::string lsContent = FanucBuildTpMoveLsContent(programName, vtRobotMoveInfo, m_tAxisUnit);
	if (!FanucWriteTextFile(localLsPath, lsContent))
	{
		if (m_pRobotLog != nullptr)
		{
			m_pRobotLog->write(LogColor::ERR, "FANUC UploadMultiPointTpProgram 生成LS失败：%s", localLsPath.c_str());
		}
		return -3;
	}

	if (m_pRobotLog != nullptr)
	{
		for (size_t i = 0; i < vtRobotMoveInfo.size(); ++i)
		{
			FanucLogMovePoint(this, "FANUC 多点TP下发", static_cast<int>(i + 1), vtRobotMoveInfo[i], &m_tAxisUnit);
		}
		m_pRobotLog->write(LogColor::SUCCESS,
			"FANUC 多点TP轨迹已生成 | Program=%s | PointCount=%d | LocalLS=%s",
			programName.c_str(),
			static_cast<int>(vtRobotMoveInfo.size()),
			localLsPath.c_str());
	}

	const int uploadRet = UploadLsFile(localLsPath, "/md/");
	if (uploadRet != 0)
	{
		if (m_pRobotLog != nullptr)
		{
			m_pRobotLog->write(LogColor::ERR,
				"FANUC UploadMultiPointTpProgram 上传失败 | Program=%s | Ret=%d",
				programName.c_str(),
				uploadRet);
		}
		return uploadRet;
	}

	if (pProgramName != nullptr)
	{
		*pProgramName = programName;
	}
	if (pLocalLsPath != nullptr)
	{
		*pLocalLsPath = localLsPath;
	}
	if (pRemoteTpPath != nullptr)
	{
		*pRemoteTpPath = remoteTpPath;
	}

	if (m_pRobotLog != nullptr)
	{
		m_pRobotLog->write(LogColor::SUCCESS,
			"FANUC 多点TP下发完成 | Program=%s | RemoteTP=%s | 当前仅下发，不自动执行",
			programName.c_str(),
			remoteTpPath.c_str());
	}
	return 0;
}

// 这三个历史 KAREL 文件自带 "placeholder"，没有任何焊接动作。
// 禁止把它们作为可用焊接程序上传，避免界面或后续调用者误判能力已闭环。
bool FANUCRobotCtrl::SendWeldTriangleWeaveProgram(int nWeldTrackNum)
{
	(void)nWeldTrackNum;
	const std::string reason =
		"FANUC三角摆焊程序已禁止下发：WeldTriangleWeave.kl/.pc仍是占位文件，未绑定现场ArcTool契约。";
	SetLastRobotError(reason);
	if (m_pRobotLog != nullptr)
	{
		m_pRobotLog->write(LogColor::ERR, "%s", reason.c_str());
	}
	return false;
}

bool FANUCRobotCtrl::SendWeldLWeaveProgram(int nWeldTrackNum)
{
	(void)nWeldTrackNum;
	const std::string reason =
		"FANUCL形摆焊程序已禁止下发：WeldLWeave.kl/.pc仍是占位文件，未绑定现场ArcTool契约。";
	SetLastRobotError(reason);
	if (m_pRobotLog != nullptr)
	{
		m_pRobotLog->write(LogColor::ERR, "%s", reason.c_str());
	}
	return false;
}

bool FANUCRobotCtrl::SendWeldProgram(int nWeldTrackNum)
{
	(void)nWeldTrackNum;
	const std::string reason =
		"FANUC普通焊接程序已禁止下发：WeldProgram.kl/.pc仍是占位文件，未绑定现场ArcTool契约。";
	SetLastRobotError(reason);
	if (m_pRobotLog != nullptr)
	{
		m_pRobotLog->write(LogColor::ERR, "%s", reason.c_str());
	}
	return false;
}

// 编译KL为PC后上传到机器人；KL源码本身不上传。
int FANUCRobotCtrl::UploadKlFile(std::string localKlPath, std::string remoteDir)
{
	if (!FanucFileExists(localKlPath))
	{
		if (m_pRobotLog != nullptr)
		{
			m_pRobotLog->write(LogColor::ERR, "FANUC UploadKlFile 失败：本地KL不存在，路径=%s", localKlPath.c_str());
		}
		return -1;
	}

	std::filesystem::path klPath(localKlPath);
	const std::string klFileName = klPath.filename().string();
	const std::string pcFileName = klPath.stem().string() + ".pc";
	const std::string localPcPath = (klPath.parent_path() / pcFileName).string();

	if (remoteDir.empty())
	{
		remoteDir = "/md/";
	}
	else if (remoteDir.back() != '/')
	{
		remoteDir += "/";
	}

	const std::string remoteKlPath = remoteDir + klFileName;
	const std::string remotePcPath = remoteDir + pcFileName;

	if (!FanucCompileKlToPc(localKlPath, localPcPath, m_pRobotLog))
	{
		return -2;
	}

	if (m_pFTP == nullptr)
	{
		InitFtp();
	}
	if (m_pFTP == nullptr)
	{
		if (m_pRobotLog != nullptr)
		{
			m_pRobotLog->write(LogColor::ERR, "FANUC UploadKlFile 失败：初始化FTP失败");
		}
		return -3;
	}

	if (!m_pFTP->uploadFile(localPcPath, remotePcPath))
	{
		if (m_pRobotLog != nullptr)
		{
			m_pRobotLog->write(LogColor::ERR, "FANUC UploadKlFile 失败：上传PC失败，远程路径=%s", remotePcPath.c_str());
		}
		return -4;
	}

	if (m_pRobotLog != nullptr)
	{
		m_pRobotLog->write(LogColor::SUCCESS,
			"FANUC UploadKlFile 完成 | PC=%s",
			remotePcPath.c_str());
	}

	return 0;
}

// 编译LS为TP后上传到机器人；用于离线TP程序下发。
int FANUCRobotCtrl::UploadLsFile(std::string localLsPath, std::string remoteDir)
{
	if (!FanucFileExists(localLsPath))
	{
		if (m_pRobotLog != nullptr)
		{
			m_pRobotLog->write(LogColor::ERR, "FANUC UploadLsFile 失败：本地LS不存在，路径=%s", localLsPath.c_str());
		}
		return -1;
	}

	std::filesystem::path lsPath(localLsPath);
	const std::string tpFileName = lsPath.stem().string() + ".tp";
	const std::string localTpPath = (lsPath.parent_path() / tpFileName).string();

	if (remoteDir.empty())
	{
		remoteDir = "/md/";
	}
	else if (remoteDir.back() != '/')
	{
		remoteDir += "/";
	}

	const std::string remoteTpPath = remoteDir + tpFileName;

	if (!FanucCompileLsToTp(localLsPath, localTpPath, m_pRobotLog))
	{
		return -2;
	}

	if (m_pFTP == nullptr)
	{
		InitFtp();
	}
	if (m_pFTP == nullptr)
	{
		if (m_pRobotLog != nullptr)
		{
			m_pRobotLog->write(LogColor::ERR, "FANUC UploadLsFile 失败：初始化FTP失败");
		}
		return -3;
	}

	if (!m_pFTP->uploadFile(localTpPath, remoteTpPath))
	{
		if (m_pRobotLog != nullptr)
		{
			m_pRobotLog->write(LogColor::ERR, "FANUC UploadLsFile 失败：上传TP失败，远程路径=%s", remoteTpPath.c_str());
		}
		return -4;
	}

	if (m_pRobotLog != nullptr)
	{
		m_pRobotLog->write(LogColor::SUCCESS,
			"FANUC UploadLsFile 完成 | TP=%s",
			remoteTpPath.c_str());
	}

	return 0;
}

// ===================== FTP文件传输 =====================

// 初始化FTP客户端；连接复用由FtpClient内部处理。
int FANUCRobotCtrl::InitFtp()
{
	if (m_pFTP == nullptr)
	{
		m_pFTP = new FtpClient(m_pRobotLog, m_sFTPIP, m_nFTPPort, m_sFTPUser, m_sFTPPassWord);
	}
	return 0;
}

// 上传任意已准备好的文件，固定TP和编译产物都走这个接口。
int FANUCRobotCtrl::UploadFile(std::string LocalFilePath, std::string RemoteFilePath)
{
	if (m_pFTP == nullptr)
	{
		InitFtp();
	}
	if (m_pFTP == nullptr)
	{
		return -1;
	}
	return m_pFTP->uploadFile(LocalFilePath, RemoteFilePath) ? 0 : -1;
}

// 从机器人FTP下载文件到本地。
int FANUCRobotCtrl::DownloadFile(std::string RemoteFilePath, std::string LocalFilePath)
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

// ===================== 基础控制命令 =====================

// 当前常驻服务没有可验证的伺服下电与状态回读契约，必须失败关闭。
bool FANUCRobotCtrl::ServoOff()
{
	SetLastRobotError("FANUC伺服下电未适配：机器人侧服务尚无真实动作和状态回读契约。");
	return false;
}

// 当前常驻服务没有可验证的伺服上电与状态回读契约，必须失败关闭。
bool FANUCRobotCtrl::ServoOn()
{
	SetLastRobotError("FANUC伺服上电未适配：机器人侧服务尚无真实动作和状态回读契约。");
	return false;
}

// 当前常驻服务没有可验证的报警复位和报警状态回读契约，必须失败关闭。
bool FANUCRobotCtrl::cleanAlarm()
{
	SetLastRobotError("FANUC报警复位未适配：机器人侧服务尚无真实复位动作和报警状态回读契约。");
	return false;
}

// 当前常驻服务没有可验证的模式切换与模式回读契约，必须失败关闭。
bool FANUCRobotCtrl::SetSysMode(int mode)
{
	(void)mode;
	SetLastRobotError("FANUC运行模式切换未适配：机器人侧服务尚无真实切换动作和模式回读契约。");
	return false;
}

// 设置固定TP使用的速度寄存器R[17]。
bool FANUCRobotCtrl::SetTpSpeed(int speed)
{
	std::string response;
	return FanucRequest(this, "SET_TP_SPEED:" + std::to_string(speed), response) && FanucIsOkResponse(response);
}

// 获取当前用户程序名；FANUC常驻服务模式下返回固定服务程序名。
std::string FANUCRobotCtrl::GetUserProgram()
{
	std::string response;
	if (!FanucRequest(this, "GET_USER_PROGRAM", response) || !FanucStartsWith(response, "PROGRAM:"))
	{
		return std::string();
	}
	return FanucResponsePayload(response);
}

// 获取当前项目名；FANUC控制器无STEP项目概念，这里用于兼容上层接口。
std::string FANUCRobotCtrl::GetUserProject()
{
	std::string response;
	if (!FanucRequest(this, "GET_USER_PROJECT", response) || !FanucStartsWith(response, "PROJECT:"))
	{
		return std::string();
	}
	return FanucResponsePayload(response);
}

std::string FANUCRobotCtrl::SendRawCommandForTest(const std::string& command)
{
	std::string response;
	if (!FanucRequest(this, command, response))
	{
		return "REQUEST_FAIL";
	}
	return response;
}

// 兼容STEP的加载程序接口；FANUC侧通过服务命令处理。
bool FANUCRobotCtrl::LoadUserProgram(std::string projName, std::string progName)
{
	std::string response;
	return FanucRequest(this, "LOAD_USER_PROGRAM:" + projName + "," + progName, response) && FanucIsOkResponse(response);
}

// 兼容STEP的卸载程序接口；FANUC侧通常不需要显式卸载。
bool FANUCRobotCtrl::UnLoadUserProgramer()
{
	std::string response;
	return FanucRequest(this, "UNLOAD_USER_PROGRAM", response) && FanucIsOkResponse(response);
}

// 兼容STEP的程序启动接口。
bool FANUCRobotCtrl::Prog_startRun_Py()
{
	std::string response;
	return FanucRequest(this, "PROGRAM_START", response) && FanucIsOkResponse(response);
}

// 兼容STEP的程序停止接口。
bool FANUCRobotCtrl::Prog_stop_Py()
{
	if (!HasVerifiedProgramStopCapability())
	{
		return false;
	}
	std::string response;
	if (!FanucRequest(this, "PROGRAM_STOP", response))
	{
		SetLastRobotError("FANUC PROGRAM_STOP 通信失败，无法确认中止：" + response);
		return false;
	}
	const bool abortAccepted = FanucIsOkResponse(response);
	const bool noTrackedCallJob = response == "ERR:NO_ACTIVE_CALL_JOB";
	if (noTrackedCallJob)
	{
		// 服务重启会丢失 RAM 中的 active task identity，但此前 RUN_TASK 启动的
		// TP 仍可能独立运行或暂停。存在本软件 pending 时，NO_ACTIVE 绝不是停止证明。
		if (RobotOperationLease::MotionCompletionPending(this))
		{
			SetLastRobotError(
				"FANUC PROGRAM_STOP 无法确认中止：机器人服务未保留活动任务身份，"
				"但本软件仍有未确认运动。保持安全互锁，禁止继续运动。");
			return false;
		}
		// 本来就没有本软件 pending 时，NO_ACTIVE 仅作为幂等 no-op 收拢本地线程。
		FinalizeContinuousMoveAfterVerifiedStop();
		ClearLastRobotError();
		return true;
	}
	if (!abortAccepted && !noTrackedCallJob)
	{
		SetLastRobotError("FANUC PROGRAM_STOP 未接受中止请求：" + response);
		return false;
	}

	// ABORT_TASK 返回后再通过任务状态做稳定回读；只收到 OK 不能宣称停机。
	int stableStoppedCount = 0;
	const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
	while (std::chrono::steady_clock::now() < deadline)
	{
		const int done = CheckDone();
		if (done < 0)
		{
			return false;
		}
		if (done > 0)
		{
			if (++stableStoppedCount >= FANUC_DONE_ACTIVE_STABLE_COUNT)
			{
				FinalizeContinuousMoveAfterVerifiedStop();
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
	SetLastRobotError("FANUC PROGRAM_STOP 已发送，但 5 秒内未能回读确认机器人停止。");
	return false;
}

// 动态工具号由机器人侧写入并回读 $MNUTOOLNUM[1] 后才报告成功。
bool FANUCRobotCtrl::SetRobotToolNo(int nToolNo)
{
	if (!HasServiceCapability("TOOL_DATA_V1"))
	{
		return false;
	}
	if (nToolNo < 1 || nToolNo > 10)
	{
		SetLastRobotError(GetStr("FANUC工具号无效：UT=%d，当前支持1~10。", nToolNo));
		return false;
	}
	std::string response;
	if (!FanucRequest(this, "SET_TOOL_NO:" + std::to_string(nToolNo), response)
		|| response != "TOOL_NO:" + std::to_string(nToolNo))
	{
		SetLastRobotError("FANUC工具号设置或回读验证失败，RSP=" + response);
		return false;
	}
	ClearLastRobotError();
	return true;
}

// 读取机器人系统变量 $MNUTOOL[1,n].$X/$Y/$Z/$W/$P/$R。
bool FANUCRobotCtrl::GetToolData(int unToolNo, T_ROBOT_COORS& adRobotToolData)
{
	adRobotToolData = T_ROBOT_COORS();
	if (!HasServiceCapability("TOOL_DATA_V1"))
	{
		return false;
	}
	if (unToolNo < 1 || unToolNo > 10)
	{
		SetLastRobotError(GetStr("FANUC工具数据读取失败：UT=%d超出1~10。", unToolNo));
		return false;
	}
	std::string response;
	double values[6] = {};
	if (!FanucRequest(this, "GET_TOOL_DATA:" + std::to_string(unToolNo), response)
		|| !FanucStartsWith(response, "TOOL:")
		|| FanucSplit(FanucResponsePayload(response), ',').size() != 6
		|| !FanucParseDoubles(FanucResponsePayload(response), values, 6))
	{
		SetLastRobotError("FANUC工具数据响应无效，RSP=" + response);
		return false;
	}
	adRobotToolData = T_ROBOT_COORS(
		values[0], values[1], values[2], values[3], values[4], values[5], 0, 0, 0);
	ClearLastRobotError();
	return true;
}

// ===================== 寄存器与变量读写 =====================

// 写位置寄存器：POSVAR走Cartesian PR，PULSEVAR走Joint PR。
bool FANUCRobotCtrl::SetPosVar(int nIndex, double pos[8], int nPVarType, int isconfig, int config[7], int scoper, int Coord)
{
	(void)isconfig;
	(void)scoper;
	(void)Coord;
	std::vector<std::string> parts =
	{
		std::to_string(nIndex),
		std::to_string(nPVarType)
	};
	for (int i = 0; i < 8; ++i)
	{
		parts.push_back(GetStr("%.6f", pos[i]));
	}
	parts.push_back(FanucBuildConfigText(config));

	std::string response;
	return FanucRequest(this, "SET_POS_VAR:" + FanucJoin(parts, ','), response) && FanucIsOkResponse(response);
}

// 写Cartesian位置寄存器的便捷重载。
void FANUCRobotCtrl::SetPosVar(int nIndex, T_ROBOT_COORS tRobotCoors, int isconfig, int config[7], int scoper)
{
	double pos[8] =
	{
		tRobotCoors.dX, tRobotCoors.dY, tRobotCoors.dZ,
		tRobotCoors.dRX, tRobotCoors.dRY, tRobotCoors.dRZ,
		tRobotCoors.dBX, tRobotCoors.dBY
	};
	if (m_pRobotLog != nullptr)
	{
		m_pRobotLog->write(LogColor::DEFAULT,
			"FANUC 发送MOVL点 PR[%d]: X=%.3f Y=%.3f Z=%.3f RX=%.3f RY=%.3f RZ=%.3f BX=%.3f BY=%.3f",
			nIndex, pos[0], pos[1], pos[2], pos[3], pos[4], pos[5], pos[6], pos[7]);
	}
	SetPosVar(nIndex, pos, POSVAR, isconfig, config, scoper, POSVAR);
}

// 写Joint位置寄存器的便捷重载；先按轴当量把脉冲换算为角度/外部轴位置。
bool FANUCRobotCtrl::SetPosVar(int nIndex, T_ANGLE_PULSE tRobotPulse, int scoper)
{
	(void)scoper;
	double pos[8] =
	{
		FanucPulseToPosition(tRobotPulse.nSPulse, m_tAxisUnit.dSPulseUnit),
		FanucPulseToPosition(tRobotPulse.nLPulse, m_tAxisUnit.dLPulseUnit),
		FanucPulseToPosition(tRobotPulse.nUPulse, m_tAxisUnit.dUPulseUnit),
		FanucPulseToPosition(tRobotPulse.nRPulse, m_tAxisUnit.dRPulseUnit),
		FanucPulseToPosition(tRobotPulse.nBPulse, m_tAxisUnit.dBPulseUnit),
		FanucPulseToPosition(tRobotPulse.nTPulse, m_tAxisUnit.dTPulseUnit),
		FanucPulseToPosition(tRobotPulse.lBXPulse, m_tAxisUnit.dBXPulseUnit),
		FanucPulseToPosition(tRobotPulse.lBYPulse, m_tAxisUnit.dBYPulseUnit)
	};
	int config[7] = { 0 };
	return SetPosVar(nIndex, pos, PULSEVAR, 0, config, ENGINEEVAR, PULSEVAR);
}

// 读取位置寄存器；当前协议仅保留基础返回，后续可按现场需要补完整解析。
int FANUCRobotCtrl::GetPosVar(long lPvarIndex, double array[6], int config[7], int MoveType)
{
	std::string response;
	if (!FanucRequest(this, "GET_POS_VAR:" + std::to_string(lPvarIndex) + "," + std::to_string(MoveType), response))
	{
		return -1;
	}
	if (!FanucStartsWith(response, "POSVAR:"))
	{
		return -1;
	}

	const std::vector<std::string> parts = FanucSplit(FanucResponsePayload(response), ',');
	if (parts.size() < 6)
	{
		return -1;
	}

	size_t valueOffset = 0;
	if (parts.size() >= 9)
	{
		// FANUC 常驻服务返回格式：index, type, cfgFlag, X, Y, Z, W, P, R, N U T, ...
		// 这里跳过前三个元信息字段，只取后面的 XYZWPR。
		valueOffset = 3;
	}
	if (parts.size() < valueOffset + 6)
	{
		return -1;
	}

	for (size_t i = 0; i < 6; ++i)
	{
		array[i] = atof(parts[valueOffset + i].c_str());
	}

	if (config != nullptr)
	{
		for (int i = 0; i < 7; ++i)
		{
			config[i] = 0;
		}
	}
	return 0;
}

bool FANUCRobotCtrl::GetHandEyeMatrixVariable(
	const char* variableName,
	double rotation[9],
	double translation[3],
	std::string* error)
{
	const auto fail = [this, error](const std::string& detail)
		{
			SetLastRobotError(detail);
			if (error != nullptr) { *error = detail; }
			return false;
		};
	if (variableName == nullptr || std::string(variableName) != "eye"
		|| rotation == nullptr || translation == nullptr)
	{
		return fail("FANUC手眼矩阵读取失败：当前仅支持变量名eye，且输出缓冲区不能为空。");
	}
	if (!HasServiceCapability("HAND_EYE_R100_V1"))
	{
		const std::string detail = GetLastRobotError();
		if (error != nullptr) { *error = detail; }
		return false;
	}

	std::string response;
	double values[12] = {};
	if (!FanucRequest(this, "GET_HE_MATRIX:eye", response)
		|| !FanucStartsWith(response, "HE_MATRIX:")
		|| FanucSplit(FanucResponsePayload(response), ',').size() != 12
		|| !FanucParseDoubles(FanucResponsePayload(response), values, 12))
	{
		return fail("FANUC手眼矩阵响应无效，RSP=" + response);
	}
	for (int index = 0; index < 9; ++index)
	{
		rotation[index] = values[index];
	}
	for (int index = 0; index < 3; ++index)
	{
		translation[index] = values[9 + index];
	}
	if (!FanucRotationMatrixIsValid(rotation)
		|| !std::isfinite(translation[0])
		|| !std::isfinite(translation[1])
		|| !std::isfinite(translation[2]))
	{
		return fail(
			"FANUC手眼矩阵R[100]~R[111]无效：旋转必须为正交右手矩阵，平移必须为有限毫米值。");
	}
	ClearLastRobotError();
	if (error != nullptr) { error->clear(); }
	return true;
}

// 设置命名速度变量；兼容旧接口，当前由常驻服务命令转发。
bool FANUCRobotCtrl::SetSpeed(const char* name, double* speed, int scord)
{
	(void)scord;
	std::vector<std::string> parts = { name == nullptr ? "" : name };
	for (int i = 0; i < 5; ++i)
	{
		parts.push_back(GetStr("%.6f", speed[i]));
	}
	std::string response;
	return FanucRequest(this, "SET_SPEED:" + FanucJoin(parts, ','), response) && FanucIsOkResponse(response);
}

// 按编号设置速度变量，内部转成SPD<n>命名。
bool FANUCRobotCtrl::SetSpeed(int nIndex, double adSpeed[5])
{
	const std::string speedName = GetStr("SPD%d", nIndex);
	return SetSpeed(speedName.c_str(), adSpeed, ENGINEEVAR);
}

//bool FANUCRobotCtrl::SetSpeed(int nIndex, SDynamicPercent adSpeed)
//{
//	(void)nIndex;
//	(void)adSpeed;
//	return false;
//}

// 读取整数寄存器，默认读取INT<n>。
bool FANUCRobotCtrl::TryGetIntVar(int nIndex, int& value, const char* cStrPreFix)
{
	return TryGetIntVarStrict(nIndex, cStrPreFix, value);
}

int FANUCRobotCtrl::GetIntVar(int nIndex, const char* cStrPreFix)
{
	int value = 0;
	return TryGetIntVarStrict(nIndex, cStrPreFix, value) ? value : 0;
}

bool FANUCRobotCtrl::TryGetIntVarStrict(int nIndex, const char* cStrPreFix, int& value)
{
	value = 0;
	std::string response;
	const std::string prefix = cStrPreFix == nullptr ? "INT" : cStrPreFix;
	if (!FanucRequest(this, "GET_INT:" + prefix + "," + std::to_string(nIndex), response))
	{
		return false;
	}
	if (!FanucStartsWith(response, "INT:"))
	{
		SetLastRobotError("FANUC整数变量响应格式无效：" + response);
		return false;
	}
	if (!FanucParseIntStrict(FanucResponsePayload(response), value))
	{
		SetLastRobotError("FANUC整数变量响应无效：" + response);
		return false;
	}
	return true;
}

// 按编号写整数寄存器，默认写INT<n>。
bool FANUCRobotCtrl::SetIntVar(int nIndex, int nValue, int score, const char* cStrPreFix)
{
	(void)score;
	const std::string prefix = cStrPreFix == nullptr ? "INT" : cStrPreFix;
	return SetIntVar((prefix + std::to_string(nIndex)).c_str(), nValue, score);
}

// 按名字写整数变量/寄存器，例如WELD_TRACK_NO会映射到机器人侧固定寄存器。
bool FANUCRobotCtrl::SetIntVar(const char* name, int value, int score)
{
	(void)score;
	std::string response;
	return FanucRequest(this, "SET_INT:" + std::string(name == nullptr ? "" : name) + "," + std::to_string(value), response) && FanucIsOkResponse(response);
}

// 写实数变量/寄存器，保留给工艺参数扩展。
bool FANUCRobotCtrl::SetRealVar(int nIndex, double value, const char* cStrPreFix, int score)
{
	(void)score;
	const std::string prefix = cStrPreFix == nullptr ? "REAL" : cStrPreFix;
	std::string response;
	return FanucRequest(this, "SET_REAL:" + prefix + std::to_string(nIndex) + "," + GetStr("%.6f", value), response) && FanucIsOkResponse(response);
}

// ===================== 运动接口兼容层 =====================

// 单轴脉冲运动兼容入口；当前转发给机器人侧服务处理。
bool FANUCRobotCtrl::AxisPulseMove(int nAxisNo, long lDist, long lRobotSpd, int nCoorType, int nMovtype, int nToolNo, long lCoordFrm)
{
	std::vector<std::string> parts =
	{
		std::to_string(nAxisNo),
		std::to_string(lDist),
		std::to_string(lRobotSpd),
		std::to_string(nCoorType),
		std::to_string(nMovtype),
		std::to_string(nToolNo),
		std::to_string(lCoordFrm)
	};
	std::string response;
	return FanucRequest(this, "AXIS_PULSE_MOVE:" + FanucJoin(parts, ','), response) && FanucIsOkResponse(response);
}

// 单轴笛卡尔/位置运动兼容入口，带config参数。
bool FANUCRobotCtrl::PosMove(int nAxisNo, double dDist, long lRobotSpd, int nCoorType, int nMovtype, int config[7], int nToolNo, long lCoordFrm)
{
	std::vector<std::string> parts =
	{
		std::to_string(nAxisNo),
		GetStr("%.6f", dDist),
		std::to_string(lRobotSpd),
		std::to_string(nCoorType),
		std::to_string(nMovtype),
		std::to_string(nToolNo),
		std::to_string(lCoordFrm),
		FanucBuildConfigText(config)
	};
	std::string response;
	return FanucRequest(this, "POS_MOVE:" + FanucJoin(parts, ','), response) && FanucIsOkResponse(response);
}

// 单轴笛卡尔/位置运动兼容入口，默认config为0。
bool FANUCRobotCtrl::PosMove(int nAxisNo, double dDist, long lRobotSpd, int nCoorType, int nMovtype, int nToolNo, long lCoordFrm)
{
	int config[7] = { 0 };
	return PosMove(nAxisNo, dDist, lRobotSpd, nCoorType, nMovtype, config, nToolNo, lCoordFrm);
}

void FANUCRobotCtrl::InvalidateFixedMoveUploadCache()
{
	std::lock_guard<std::mutex> cacheLock(m_fixedMoveUploadMutex);
	m_fixedMoveUploadEndpointKey.clear();
	m_fixedMovlUploaded = false;
	m_fixedMovjUploaded = false;
	m_fixedMovlLocalIdentity = {};
	m_fixedMovjLocalIdentity = {};
}

bool FANUCRobotCtrl::EnsureFixedMoveTpUploaded(
	int moveType,
	const std::string& localTpPath,
	const std::string& programName)
{
	std::lock_guard<std::mutex> cacheLock(m_fixedMoveUploadMutex);
	const std::string endpointKey = FanucControllerIdentity(this, false);
	FanucTpContentIdentity::Identity localIdentity;
	if (!FanucReadFileIdentity(localTpPath, localIdentity))
	{
		return false;
	}
	if (m_fixedMoveUploadEndpointKey != endpointKey)
	{
		m_fixedMoveUploadEndpointKey = endpointKey;
		m_fixedMovlUploaded = false;
		m_fixedMovjUploaded = false;
		m_fixedMovlLocalIdentity = {};
		m_fixedMovjLocalIdentity = {};
	}

	bool& uploaded = moveType == MOVL ? m_fixedMovlUploaded : m_fixedMovjUploaded;
	FanucTpContentIdentity::Identity& cachedIdentity = moveType == MOVL
		? m_fixedMovlLocalIdentity
		: m_fixedMovjLocalIdentity;
	const std::string remoteFileName = programName + ".tp";
	const std::string remoteTpPath = "/md/" + remoteFileName;
	// Use a cache-local FTP client built from the current endpoint. The shared
	// m_pFTP may belong to another in-flight transfer or to pre-reload settings.
	FtpClient fixedMoveFtp(
		m_pRobotLog,
		m_sFTPIP,
		m_nFTPPort,
		m_sFTPUser,
		m_sFTPPassWord);
	fixedMoveFtp.setMessageBoxesEnabled(false);

	const auto remoteFileMatchesContent = [&]() -> bool
		{
			std::vector<FtpRemoteFileInfo> files;
			if (!fixedMoveFtp.listFiles("/md/", files))
			{
				return false;
			}
			const bool declaredFilePresent = std::any_of(files.begin(), files.end(), [&](const FtpRemoteFileInfo& file)
				{
					return !file.isDirectory
						&& file.size == localIdentity.size
						&& _stricmp(file.name.c_str(), remoteFileName.c_str()) == 0;
				});
			if (!declaredFilePresent)
			{
				return false;
			}

			// 保持机器人既有程序名（以及 CALL_JOB 语义）不变，避免把哈希塞进受限的
			// FANUC 文件名。每次执行前把控制器二进制下载到独立临时目录并比较 SHA-256；
			// 因此同名、同长度替换也绝不会被缓存命中。
			QTemporaryDir verificationDir;
			if (!verificationDir.isValid())
			{
				return false;
			}
			const QString downloadedPath = verificationDir.filePath(QStringLiteral("remote.tp"));
			if (!fixedMoveFtp.downloadFile(remoteTpPath, FanucLocalPathBytes(downloadedPath)))
			{
				return false;
			}
			FanucTpContentIdentity::Identity remoteIdentity;
			return FanucTpContentIdentity::Read(FanucFileSystemPath(downloadedPath), remoteIdentity)
				&& FanucTpContentIdentity::Matches(localIdentity, remoteIdentity);
		};

	if (uploaded
		&& FanucTpContentIdentity::Matches(cachedIdentity, localIdentity)
		&& remoteFileMatchesContent())
	{
		return true;
	}
	if (uploaded && m_pRobotLog != nullptr)
	{
		m_pRobotLog->write(LogColor::WARNING,
			"FANUC 固定TP缓存已失效，远端文件缺失或不可验证，将重新上传：%s",
			remoteTpPath.c_str());
	}
	uploaded = false;
	cachedIdentity = {};

	if (!fixedMoveFtp.uploadFile(localTpPath, remoteTpPath))
	{
		return false;
	}
	// 上传期间本地受信任 TP 也不得被并发替换；远端与最初读取的身份必须同时保持一致。
	FanucTpContentIdentity::Identity localIdentityAfterUpload;
	if (!FanucReadFileIdentity(localTpPath, localIdentityAfterUpload)
		|| !FanucTpContentIdentity::Matches(localIdentity, localIdentityAfterUpload)
		|| !remoteFileMatchesContent())
	{
		if (m_pRobotLog != nullptr)
		{
			m_pRobotLog->write(LogColor::ERR,
				"FANUC 固定TP上传后远端回读失败：%s",
				remoteTpPath.c_str());
		}
		return false;
	}
	uploaded = true;
	cachedIdentity = localIdentity;
	if (m_pRobotLog != nullptr)
	{
		m_pRobotLog->write(LogColor::SUCCESS,
			"FANUC 固定TP已上传并回读确认：%s | Endpoint=%s:%d",
			remoteTpPath.c_str(), m_sFTPIP.c_str(), m_nFTPPort);
	}
	return true;
}

// 固定TP单点运动公共流程：设置目标PR/速度R，必要时上传固定TP，然后调用任务。
bool FANUCRobotCtrl::CreateUploadRunTpMove(const std::vector<T_ROBOT_MOVE_INFO>& moveInfos)
{
	FANUCRobotCtrl* ctrl = this;
	std::lock_guard<std::mutex> executionLock(m_fixedMoveExecutionMutex);
	// Single-point MOVL/MOVJ uses fixed TP programs:
	// update PR[1] and R[17], upload the fixed TP once, then CALL_JOB.
	// This avoids invoking maketp.exe for every small jog command.
	if (moveInfos.empty())
	{
		ctrl->ClearLastRobotError();
		ctrl->SetLastRobotError("FANUC固定TP运动失败：目标点为空");
		return false;
	}
	ctrl->ClearLastRobotError();
	if (moveInfos.size() != 1)
	{
		ctrl->SetLastRobotError(GetStr("FANUC固定TP运动失败：当前只支持单点调用，PointCount=%d",
			static_cast<int>(moveInfos.size())));
		if (ctrl->HasLogSink())
		{
			ctrl->WriteLog(LogColor::ERR,
				"FANUC 固定TP运动当前只支持单点调用，PointCount=%d",
				static_cast<int>(moveInfos.size()));
		}
		return false;
	}
	if (ctrl->RobotAxisCount() > 6 && ctrl->HasLogSink())
	{
		ctrl->WriteLog(LogColor::WARNING,
			"FANUC TP运动提示：当前轴数=%d，生成的LS暂按6轴主机器人点位写入",
			ctrl->RobotAxisCount());
	}

	const T_ROBOT_MOVE_INFO& moveInfo = moveInfos[0];
	const int moveType = moveInfo.nMoveType == MOVL ? MOVL : MOVJ;
	const std::string programName = FanucFixedMoveProgramName(moveType);
	FanucLogMovePoint(ctrl, "FANUC 固定TP目标点", 1, moveInfo, &ctrl->AxisUnit());
	const std::string localTpPath = FanucFixedMoveTpPath(ctrl->RobotName(), moveType);
	if (localTpPath.empty())
	{
		ctrl->SetLastRobotError("FANUC固定TP运动失败：固定TP不存在，Program=" + programName);
		if (ctrl->HasLogSink())
		{
			ctrl->WriteLog(LogColor::ERR, "FANUC 固定TP不存在：%s.tp", programName.c_str());
		}
		return false;
	}

	int speed = 0;
	const bool speedRepresentable = moveType == MOVL
		? FanucLinearSpeedRegister(moveInfo.tSpeed.dSpeed, speed)
		: (speed = FanucSpeedPercent(moveInfo.tSpeed.dSpeed)) > 0;
	if (!speedRepresentable)
	{
		ctrl->SetLastRobotError(GetStr(
			"FANUC固定TP运动失败：速度无法安全表示，Mode=%s Requested=%.6f；MOVL最低1mm/sec且禁止向上取整。",
			moveType == MOVL ? "MOVL" : "MOVJ", moveInfo.tSpeed.dSpeed));
		return false;
	}

	if (!ctrl->SetTpSpeed(speed) || ctrl->GetIntVar(17) != speed)
	{
		ctrl->SetLastRobotError(GetStr("FANUC固定TP运动失败：设置速度失败，Speed=%d，%s",
			speed, ctrl->GetRobotStatusText().c_str()));
		if (ctrl->HasLogSink())
		{
			ctrl->WriteLog(LogColor::ERR, "FANUC 固定TP运动设置速度失败：%d", speed);
		}
		return false;
	}

	bool setTargetOk = false;
	if (moveType == MOVL)
	{
		int config[7] = { 0 };
		double pos[8] =
		{
			moveInfo.tCoord.dX, moveInfo.tCoord.dY, moveInfo.tCoord.dZ,
			moveInfo.tCoord.dRX, moveInfo.tCoord.dRY, moveInfo.tCoord.dRZ,
			moveInfo.tCoord.dBX, moveInfo.tCoord.dBY
		};
		if (ctrl->HasLogSink())
		{
			ctrl->WriteLog(LogColor::DEFAULT,
				"FANUC 发送固定TP MOVL点 PR[1]: X=%.3f Y=%.3f Z=%.3f RX=%.3f RY=%.3f RZ=%.3f BX=%.3f BY=%.3f",
				pos[0], pos[1], pos[2], pos[3], pos[4], pos[5], pos[6], pos[7]);
		}
		setTargetOk = ctrl->SetPosVar(1, pos, POSVAR, 1, config, ENGINEEVAR, POSVAR);
	}
	else
	{
		setTargetOk = ctrl->SetPosVar(1, moveInfo.tPulse, ENGINEEVAR);
	}
	if (!setTargetOk)
	{
		ctrl->SetLastRobotError("FANUC固定TP运动失败：设置PR[1]失败，Program=" + programName + "，" + ctrl->GetRobotStatusText());
		if (ctrl->HasLogSink())
		{
			ctrl->WriteLog(LogColor::ERR, "FANUC 固定TP运动设置PR[1]失败：%s", programName.c_str());
		}
		return false;
	}

	const std::string remoteTpPath = "/md/" + programName + ".tp";
	// 同一控制器的“远端内容回读 -> CALL_JOB”必须是一个不可插入区间；否则另一 driver
	// 可能在哈希验证后、启动前替换同名程序。全局单实例 + endpoint mutex 同时封住进程间/进程内窗口。
	const std::string endpointKey = FanucControllerIdentity(ctrl, false);
	const std::shared_ptr<std::mutex> endpointMutex = FanucFixedEndpointTransferMutex(endpointKey);
	std::lock_guard<std::mutex> endpointVerifiedCallLock(*endpointMutex);
	if (!ctrl->EnsureFixedMoveTpUploaded(moveType, localTpPath, programName))
	{
		ctrl->SetLastRobotError("FANUC固定TP运动失败：上传或远端回读TP失败，Remote=" + remoteTpPath + "，" + ctrl->GetRobotStatusText());
		if (ctrl->HasLogSink())
		{
			ctrl->WriteLog(LogColor::ERR, "FANUC 固定TP上传/回读失败：%s", remoteTpPath.c_str());
		}
		return false;
	}

	if (!ctrl->CallJobWithCompletionState(programName, 93, 1))
	{
		// Do not auto-retry CALL_JOB: a transport failure may have started motion.
		// Invalidate only, so the next explicit user operation re-verifies/re-uploads.
		ctrl->InvalidateFixedMoveUploadCache();
		ctrl->SetLastRobotError("FANUC固定TP运动失败：启动程序失败，Program=" + programName + "，" + ctrl->GetRobotStatusText());
		if (ctrl->HasLogSink())
		{
			ctrl->WriteLog(LogColor::ERR, "FANUC TP运动启动失败：%s", programName.c_str());
		}
		return false;
	}

	if (ctrl->HasLogSink())
	{
		ctrl->WriteLog(LogColor::SUCCESS, "FANUC TP运动已启动：%s", programName.c_str());
	}
	return true;
}

// 按double数组执行单点MOVL/MOVJ：POSVAR解释为Cartesian，PULSEVAR解释为Joint脉冲。
bool FANUCRobotCtrl::MoveByJob(double* dRobotJointCoord, T_ROBOT_MOVE_SPEED tPulseMove, int nExternalAxleType, int nPVarType, std::string JobName, int config[7])
{
	(void)nExternalAxleType;
	(void)config;
	if (dRobotJointCoord == nullptr)
	{
		return false;
	}

	T_ROBOT_MOVE_INFO moveInfo;
	moveInfo.nMoveType = FanucMoveTypeFromJobName(JobName);
	moveInfo.tSpeed = tPulseMove;

	if (nPVarType == POSVAR || moveInfo.nMoveType == MOVL)
	{
		moveInfo.nMoveType = MOVL;
		moveInfo.tCoord = T_ROBOT_COORS(
			dRobotJointCoord[0], dRobotJointCoord[1], dRobotJointCoord[2],
			dRobotJointCoord[3], dRobotJointCoord[4], dRobotJointCoord[5],
			dRobotJointCoord[6], dRobotJointCoord[7], 0.0);
	}
	else
	{
		moveInfo.nMoveType = MOVJ;
		moveInfo.tPulse = T_ANGLE_PULSE(
			static_cast<long>(std::lround(dRobotJointCoord[0])),
			static_cast<long>(std::lround(dRobotJointCoord[1])),
			static_cast<long>(std::lround(dRobotJointCoord[2])),
			static_cast<long>(std::lround(dRobotJointCoord[3])),
			static_cast<long>(std::lround(dRobotJointCoord[4])),
			static_cast<long>(std::lround(dRobotJointCoord[5])),
			static_cast<long>(std::lround(dRobotJointCoord[6])),
			static_cast<long>(std::lround(dRobotJointCoord[7])),
			0);
	}

	return CreateUploadRunTpMove(std::vector<T_ROBOT_MOVE_INFO>{ moveInfo });
}

// Cartesian单点MOVL：写PR[1]为XYZWPR后调用FANUC_MOVL固定TP。
bool FANUCRobotCtrl::MoveByJob(T_ROBOT_COORS tRobotJointCoord, T_ROBOT_MOVE_SPEED tPulseMove, int nExternalAxleType, std::string JobName, int isconfig, int config[7])
{
	(void)nExternalAxleType;
	(void)isconfig;
	(void)config;

	T_ROBOT_MOVE_INFO moveInfo;
	moveInfo.nMoveType = MOVL;
	moveInfo.tCoord = tRobotJointCoord;
	moveInfo.tSpeed = tPulseMove;
	return CreateUploadRunTpMove(std::vector<T_ROBOT_MOVE_INFO>{ moveInfo });
}

// Joint单点MOVJ：写PR[1]为JOINTPOS后调用FANUC_MOVJ固定TP。
bool FANUCRobotCtrl::MoveByJob(T_ANGLE_PULSE tRobotJointCoord, T_ROBOT_MOVE_SPEED tPulseMove, int nExternalAxleType, std::string JobName)
{
	(void)nExternalAxleType;
	(void)JobName;

	T_ROBOT_MOVE_INFO moveInfo;
	moveInfo.nMoveType = MOVJ;
	moveInfo.tPulse = tRobotJointCoord;
	moveInfo.tSpeed = tPulseMove;
	return CreateUploadRunTpMove(std::vector<T_ROBOT_MOVE_INFO>{ moveInfo });
}
