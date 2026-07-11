#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>

#include "FANUCRobotDriver.h"
#include "RobotOperationLease.h"

#include <algorithm>
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
#include <mutex>
#include <sstream>
#include <thread>
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
		char ch = 0;

		while (true)
		{
			if (!FanucWaitReadable(sock, timeoutMs))
			{
				return false;
			}

			const int recved = recv(sock, &ch, 1, 0);
			if (recved <= 0)
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
				if (out.size() >= FANUC_SOCKET_MAX_LINE_SIZE)
				{
					return false;
				}
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

	std::string FanucMakeProgramName()
	{
		SYSTEMTIME st = {};
		GetLocalTime(&st);
		return GetStr("FM%02d%02d%02d%02d",
			st.wMonth, st.wDay, st.wHour, st.wMinute);
	}

	std::string FanucMakeTpProgramName()
	{
		SYSTEMTIME st = {};
		GetLocalTime(&st);
		return GetStr("FM%02d%02d%02d", st.wHour, st.wMinute, st.wSecond);
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

	void FanucLogMovePoint(RobotLog* log, const char* prefix, int index, const T_ROBOT_MOVE_INFO& info, const T_AXISUNIT* axisUnit = nullptr)
	{
		if (log == nullptr || prefix == nullptr)
		{
			return;
		}

		if (info.nMoveType == MOVL)
		{
			log->write(LogColor::DEFAULT,
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
			log->write(LogColor::DEFAULT,
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
			log->write(LogColor::DEFAULT,
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
		std::ofstream out(filePath, std::ios::out | std::ios::trunc);
		if (!out.is_open())
		{
			return false;
		}

		out << content;
		return out.good();
	}

	std::filesystem::path FanucGetExecutableDir()
	{
		char modulePath[MAX_PATH] = {};
		const DWORD length = GetModuleFileNameA(nullptr, modulePath, MAX_PATH);
		if (length == 0 || length >= MAX_PATH)
		{
			std::error_code ec;
			return std::filesystem::current_path(ec);
		}

		return std::filesystem::path(std::string(modulePath, length)).parent_path();
	}

	std::string FanucFindCompilerToolPath(const std::string& fileName)
	{
		const std::filesystem::path exeDir = FanucGetExecutableDir();
		std::vector<std::filesystem::path> candidates =
		{
			exeDir / "Tools" / "FANUC" / "WinOLPC" / "bin" / fileName,
			exeDir / "SDK" / "FANUC" / "WinOLPC" / "bin" / fileName,
			std::filesystem::path("C:\\Program Files (x86)\\FANUC\\WinOLPC\\bin") / fileName,
			std::filesystem::path("C:\\Program Files\\FANUC\\WinOLPC\\bin") / fileName
		};

		for (const auto& candidate : candidates)
		{
			std::error_code ec;
			if (std::filesystem::exists(candidate, ec))
			{
				return candidate.string();
			}
		}

		return candidates.front().string();
	}

	std::string FanucGetKtransPath()
	{
		return FanucFindCompilerToolPath("ktrans.exe");
	}

	std::string FanucGetMaketpPath()
	{
		return FanucFindCompilerToolPath("maketp.exe");
	}

	bool FanucFileExists(const std::string& filePath)
	{
		std::error_code ec;
		return std::filesystem::exists(filePath, ec);
	}

	std::filesystem::path FanucAbsolutePath(const std::string& filePath)
	{
		std::error_code ec;
		const std::filesystem::path path(filePath);
		const std::filesystem::path absolutePath = std::filesystem::absolute(path, ec);
		return ec ? path : absolutePath;
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

	std::string FanucToLower(std::string text)
	{
		std::transform(text.begin(), text.end(), text.begin(),
			[](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
		return text;
	}

	std::filesystem::path FanucReadWinOlpcOutputDir()
	{
		const std::string robotIniPath = FanucFindCompilerToolPath("robot.ini");
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
				return std::filesystem::path(outputDir);
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
		std::vector<std::string> candidates =
		{
			".\\Result\\" + unitName + "\\" + fileName,
			".\\Job\\FANUC\\" + fileName,
			".\\SDK\\FANUC\\" + fileName
		};
		for (const std::string& path : candidates)
		{
			if (FanucFileExists(path))
			{
				return path;
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
		const auto compileStart = std::chrono::steady_clock::now();
		const std::string ktransPath = FanucGetKtransPath();
		const std::filesystem::path absoluteKlPath = FanucAbsolutePath(klPath);
		const std::filesystem::path absolutePcPath = FanucAbsolutePath(pcPath);
		if (!FanucFileExists(ktransPath))
		{
			if (pLog != nullptr)
			{
				pLog->write(LogColor::ERR, "FANUC 编译失败：未找到 ktrans.exe，路径=%s | 耗时=%lldms",
					ktransPath.c_str(), FanucElapsedMs(compileStart));
			}
			return false;
		}

		std::error_code ec;
		std::filesystem::remove(absolutePcPath, ec);

		const std::string ktransWorkDir = std::filesystem::path(ktransPath).parent_path().string();
		const std::wstring exePathW(ktransPath.begin(), ktransPath.end());
		const std::wstring workDirW(ktransWorkDir.begin(), ktransWorkDir.end());
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

		if (exitCode != 0 || !FanucFileExists(absolutePcPath.string()))
		{
			if (pLog != nullptr)
			{
				pLog->write(LogColor::ERR,
					"FANUC 编译失败：ktrans 返回码=%lu，KL=%s，PC=%s，WorkDir=%s | 耗时=%lldms",
					static_cast<unsigned long>(exitCode), absoluteKlPath.string().c_str(), absolutePcPath.string().c_str(), ktransWorkDir.c_str(),
					FanucElapsedMs(compileStart));
			}
			return false;
		}

		if (pLog != nullptr)
		{
			pLog->write(LogColor::SUCCESS, "FANUC 编译成功：%s | 耗时=%lldms",
				absolutePcPath.string().c_str(), FanucElapsedMs(compileStart));
		}
		return true;
	}

	bool FanucCompileLsToTp(const std::string& lsPath, const std::string& tpPath, RobotLog* pLog)
	{
		const auto compileStart = std::chrono::steady_clock::now();
		const std::string maketpPath = FanucGetMaketpPath();
		const std::filesystem::path absoluteLsPath = FanucAbsolutePath(lsPath);
		const std::filesystem::path absoluteTpPath = FanucAbsolutePath(tpPath);
		if (!FanucFileExists(maketpPath))
		{
			if (pLog != nullptr)
			{
				pLog->write(LogColor::ERR, "FANUC TP编译失败：未找到 maketp.exe，路径=%s | 耗时=%lldms",
					maketpPath.c_str(), FanucElapsedMs(compileStart));
			}
			return false;
		}

		std::error_code ec;
		std::filesystem::create_directories(absoluteTpPath.parent_path(), ec);
		std::filesystem::remove(absoluteTpPath, ec);

		const std::string maketpWorkDir = std::filesystem::path(maketpPath).parent_path().string();
		const std::wstring exePathW(maketpPath.begin(), maketpPath.end());
		const std::wstring workDirW(maketpWorkDir.begin(), maketpWorkDir.end());
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

		if (exitCode == 0 && !FanucFileExists(absoluteTpPath.string()))
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
					pLog->write(LogColor::DEFAULT,
						"FANUC TP编译产物已从 WinOLPC 输出目录复制：%s -> %s",
						fallbackTpPath.string().c_str(),
						absoluteTpPath.string().c_str());
				}
			}
		}

		if (exitCode != 0 || !FanucFileExists(absoluteTpPath.string()))
		{
			if (pLog != nullptr)
			{
				pLog->write(LogColor::ERR,
					"FANUC TP编译失败：maketp 返回码=%lu，LS=%s，TP=%s，WorkDir=%s | 耗时=%lldms",
					static_cast<unsigned long>(exitCode), absoluteLsPath.string().c_str(), absoluteTpPath.string().c_str(), maketpWorkDir.c_str(),
					FanucElapsedMs(compileStart));
			}
			return false;
		}

		if (pLog != nullptr)
		{
			pLog->write(LogColor::SUCCESS, "FANUC TP编译成功：%s | 耗时=%lldms",
				absoluteTpPath.string().c_str(), FanucElapsedMs(compileStart));
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

	if (m_hMutex != nullptr)
	{
		CloseHandle(m_hMutex);
		m_hMutex = nullptr;
	}
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
		return ctrl->InitSocket(ctrl->m_sSocketIP.c_str(), static_cast<unsigned short>(ctrl->m_nSocketPort));
	}

	bool FanucRequest(
		FANUCRobotCtrl* ctrl,
		const std::string& command,
		std::string& response,
		bool* commandMayHaveBeenSent = nullptr)
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
			if (ctrl->m_pRobotLog != nullptr)
			{
				ctrl->m_pRobotLog->write(LogColor::ERR, "FANUC Socket CMD=%s 等待互斥锁超时", command.c_str());
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

		if (ctrl->m_pRobotLog != nullptr)
		{
			ctrl->m_pRobotLog->write(sent && recvOk ? LogColor::DEFAULT : LogColor::ERR,
				"FANUC Socket CMD=%s RSP=%s",
				command.c_str(), response.c_str());
		}

		if (!sent || !recvOk)
		{
			ctrl->SetLastRobotError("FANUC请求失败：CMD=" + command + " RSP=" + response);
			ctrl->CloseSocket();
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

bool FANUCRobotCtrl::HasVerifiedProgramStopCapability()
{
	std::string response;
	if (!FanucRequest(this, "GET_USER_PROGRAM", response))
	{
		return false;
	}
	static const std::string requiredCapability = "LIB=20260710_ABORT_TASK_STATE_V2";
	if (response.find(requiredCapability) == std::string::npos)
	{
		SetLastRobotError(
			"FANUC机器人侧服务不支持可验证的运动停止，请先上传并重启本版本 FanucServiceLib/STARTALL。RSP="
			+ response);
		return false;
	}
	return true;
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

	const std::string localDir = ".\\Job\\FANUC";
	const std::string localPath = localDir + "\\JOGBUF.DT";
	const std::string remotePath = "JOGBUF.DT";

	try
	{
		std::filesystem::create_directories(localDir);
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

	const std::string timestamp = FanucMakeTimestamp();
	const std::string programName = FanucMakeProgramName();
	const std::string localDir = ".\\Job\\FANUC";
	const std::string klFileName = programName + ".kl";
	const std::string pcFileName = programName + ".pc";
	const std::string varFileName = programName + ".var";
	const std::string localKlPath = localDir + "\\" + klFileName;
	const std::string localPcPath = localDir + "\\" + pcFileName;
	const std::string localVarPath = localDir + "\\" + varFileName;
	const std::string remotePcPath = "/md/" + pcFileName;
	const std::string remoteVarPath = "/md/" + varFileName;

	try
	{
		std::filesystem::create_directories(localDir);
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

	if (!FanucWriteTextFile(localKlPath, klContent) || !FanucWriteTextFile(localVarPath, varContent))
	{
		if (m_pRobotLog != nullptr)
		{
			m_pRobotLog->write(LogColor::ERR, "FANUC ContiMoveAny 生成文件失败：%s 或 %s", localKlPath.c_str(), localVarPath.c_str());
		}
		return -3;
	}

	if (m_pRobotLog != nullptr)
	{
		for (size_t i = 0; i < vtRobotMoveInfo.size(); ++i)
		{
			FanucLogMovePoint(m_pRobotLog, "FANUC ContiMoveAny生成点", static_cast<int>(i + 1), vtRobotMoveInfo[i], &m_tAxisUnit);
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
	std::string* pRemoteTpPath)
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

	const std::string programName = FanucMakeTpProgramName();
	const std::string localDir = ".\\Job\\FANUC";
	const std::string lsFileName = programName + ".ls";
	const std::string localLsPath = localDir + "\\" + lsFileName;
	const std::string remoteTpPath = "/md/" + programName + ".tp";

	try
	{
		std::filesystem::create_directories(localDir);
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
			FanucLogMovePoint(m_pRobotLog, "FANUC 多点TP下发", static_cast<int>(i + 1), vtRobotMoveInfo[i], &m_tAxisUnit);
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

// 请求机器人伺服下电；实际动作由常驻服务命令实现。
bool FANUCRobotCtrl::ServoOff()
{
	std::string response;
	return FanucRequest(this, "SERVO_OFF", response) && FanucIsOkResponse(response);
}

// 请求机器人伺服上电；实际动作由常驻服务命令实现。
bool FANUCRobotCtrl::ServoOn()
{
	std::string response;
	return FanucRequest(this, "SERVO_ON", response) && FanucIsOkResponse(response);
}

// 清除机器人报警；当前通过常驻服务命令转发。
bool FANUCRobotCtrl::cleanAlarm()
{
	std::string response;
	return FanucRequest(this, "CLEAR_ALARM", response) && FanucIsOkResponse(response);
}

// 设置系统/运行模式；保留与通用RobotDriver接口一致的入口。
bool FANUCRobotCtrl::SetSysMode(int mode)
{
	std::string response;
	return FanucRequest(this, "SET_SYS_MODE:" + std::to_string(mode), response) && FanucIsOkResponse(response);
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

// 设置工具号；固定TP当前默认UT=1，后续可扩展为动态工具号。
bool FANUCRobotCtrl::SetRobotToolNo(int nToolNo)
{
	std::string response;
	return FanucRequest(this, "SET_TOOL_NO:" + std::to_string(nToolNo), response) && FanucIsOkResponse(response);
}

// 获取工具数据；机器人侧服务需要返回 TOOL:x,y,z,rx,ry,rz。
bool FANUCRobotCtrl::GetToolData(int unToolNo, T_ROBOT_COORS& adRobotToolData)
{
	adRobotToolData = T_ROBOT_COORS();
	std::string response;
	if (!FanucRequest(this, "GET_TOOL_DATA:" + std::to_string(unToolNo), response))
	{
		return false;
	}

	double values[6] = {};
	if (!FanucStartsWith(response, "TOOL:") || !FanucParseDoubles(FanucResponsePayload(response), values, 6))
	{
		SetLastRobotError("FANUC工具读取失败：机器人服务未返回 TOOL:x,y,z,rx,ry,rz，RSP=" + response);
		return false;
	}

	adRobotToolData = T_ROBOT_COORS(values[0], values[1], values[2], values[3], values[4], values[5], 0, 0, 0);
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

// 固定TP单点运动公共流程：设置目标PR/速度R，必要时上传固定TP，然后调用任务。
static bool FanucCreateUploadRunTpMove(FANUCRobotCtrl* ctrl, const std::vector<T_ROBOT_MOVE_INFO>& moveInfos)
{
	// Single-point MOVL/MOVJ uses fixed TP programs:
	// update PR[1] and R[17], upload the fixed TP once, then CALL_JOB.
	// This avoids invoking maketp.exe for every small jog command.
	if (ctrl == nullptr || moveInfos.empty())
	{
		if (ctrl != nullptr)
		{
			ctrl->ClearLastRobotError();
			ctrl->SetLastRobotError("FANUC固定TP运动失败：目标点为空");
		}
		return false;
	}
	ctrl->ClearLastRobotError();
	if (moveInfos.size() != 1)
	{
		ctrl->SetLastRobotError(GetStr("FANUC固定TP运动失败：当前只支持单点调用，PointCount=%d",
			static_cast<int>(moveInfos.size())));
		if (ctrl->m_pRobotLog != nullptr)
		{
			ctrl->m_pRobotLog->write(LogColor::ERR,
				"FANUC 固定TP运动当前只支持单点调用，PointCount=%d",
				static_cast<int>(moveInfos.size()));
		}
		return false;
	}
	if (ctrl->m_nRobotAxisCount > 6 && ctrl->m_pRobotLog != nullptr)
	{
		ctrl->m_pRobotLog->write(LogColor::WARNING,
			"FANUC TP运动提示：当前轴数=%d，生成的LS暂按6轴主机器人点位写入",
			ctrl->m_nRobotAxisCount);
	}

	const T_ROBOT_MOVE_INFO& moveInfo = moveInfos[0];
	const int moveType = moveInfo.nMoveType == MOVL ? MOVL : MOVJ;
	const std::string programName = FanucFixedMoveProgramName(moveType);
	FanucLogMovePoint(ctrl->m_pRobotLog, "FANUC 固定TP目标点", 1, moveInfo, &ctrl->m_tAxisUnit);
	const std::string localTpPath = FanucFixedMoveTpPath(ctrl->m_sRobotName, moveType);
	if (localTpPath.empty())
	{
		ctrl->SetLastRobotError("FANUC固定TP运动失败：固定TP不存在，Program=" + programName);
		if (ctrl->m_pRobotLog != nullptr)
		{
			ctrl->m_pRobotLog->write(LogColor::ERR, "FANUC 固定TP不存在：%s.tp", programName.c_str());
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
		if (ctrl->m_pRobotLog != nullptr)
		{
			ctrl->m_pRobotLog->write(LogColor::ERR, "FANUC 固定TP运动设置速度失败：%d", speed);
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
		if (ctrl->m_pRobotLog != nullptr)
		{
			ctrl->m_pRobotLog->write(LogColor::DEFAULT,
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
		if (ctrl->m_pRobotLog != nullptr)
		{
			ctrl->m_pRobotLog->write(LogColor::ERR, "FANUC 固定TP运动设置PR[1]失败：%s", programName.c_str());
		}
		return false;
	}

	static bool movjUploaded = false;
	static bool movlUploaded = false;
	bool& uploaded = moveType == MOVL ? movlUploaded : movjUploaded;
	if (!uploaded)
	{
		const std::string remoteTpPath = "/md/" + programName + ".tp";
		if (ctrl->UploadFile(localTpPath, remoteTpPath) != 0)
		{
			ctrl->SetLastRobotError("FANUC固定TP运动失败：上传TP失败，Remote=" + remoteTpPath + "，" + ctrl->GetRobotStatusText());
			if (ctrl->m_pRobotLog != nullptr)
			{
				ctrl->m_pRobotLog->write(LogColor::ERR, "FANUC 固定TP上传失败：%s", remoteTpPath.c_str());
			}
			return false;
		}
		uploaded = true;
		if (ctrl->m_pRobotLog != nullptr)
		{
			ctrl->m_pRobotLog->write(LogColor::SUCCESS, "FANUC 固定TP已上传：%s", remoteTpPath.c_str());
		}
	}

	if (!ctrl->CallJobWithCompletionState(programName, 93, 1))
	{
		ctrl->SetLastRobotError("FANUC固定TP运动失败：启动程序失败，Program=" + programName + "，" + ctrl->GetRobotStatusText());
		if (ctrl->m_pRobotLog != nullptr)
		{
			ctrl->m_pRobotLog->write(LogColor::ERR, "FANUC TP运动启动失败：%s", programName.c_str());
		}
		return false;
	}

	if (ctrl->m_pRobotLog != nullptr)
	{
		ctrl->m_pRobotLog->write(LogColor::SUCCESS, "FANUC TP运动已启动：%s", programName.c_str());
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

	return FanucCreateUploadRunTpMove(this, std::vector<T_ROBOT_MOVE_INFO>{ moveInfo });
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
	return FanucCreateUploadRunTpMove(this, std::vector<T_ROBOT_MOVE_INFO>{ moveInfo });
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
	return FanucCreateUploadRunTpMove(this, std::vector<T_ROBOT_MOVE_INFO>{ moveInfo });
}
