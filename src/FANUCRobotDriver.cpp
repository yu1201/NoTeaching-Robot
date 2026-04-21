#include <winsock2.h>
#include <ws2tcpip.h>

#include "FANUCRobotDriver.h"

#include <cstdint>
#include <cstdlib>
#include <cmath>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <thread>
#include <vector>

#pragma comment(lib, "ws2_32.lib")

namespace
{
	const int FANUC_SOCKET_TIMEOUT_MS = 3000;
	const size_t FANUC_SOCKET_MAX_LINE_SIZE = 4096;

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

	bool FanucConnectSocket(SOCKET& sock, const std::string& ip, u_short port)
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
			values[i] = atof(parts[i].c_str());
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
			values[i] = atol(parts[i].c_str());
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
		if (pulseUnit == 0.0)
		{
			return 0;
		}
		return static_cast<long>(std::lround(position / pulseUnit));
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
		if (speed <= 0.0)
		{
			return 20;
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
			return 1;
		}
		if (percent > 100.0)
		{
			return 100;
		}
		return static_cast<int>(std::lround(percent));
	}

	double FanucLinearSpeed(double speed)
	{
		if (speed <= 0.0)
		{
			return 100.0;
		}
		return speed;
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
		const size_t lineCount = moveInfos.size() + 3;
		std::ostringstream oss;
		oss << FanucBuildLsHeader(programName, lineCount, "Auto FANUC move");
		oss << "/MN" << "\n";
		oss << "   1:  UFRAME_NUM=0 ;" << "\n";
		oss << "   2:  UTOOL_NUM=1 ;" << "\n";

		for (size_t i = 0; i < moveInfos.size(); ++i)
		{
			const T_ROBOT_MOVE_INFO& info = moveInfos[i];
			const size_t pointIndex = i + 1;
			const size_t lineIndex = i + 3;
			if (info.nMoveType == MOVL)
			{
				oss << GetStr("%4u:  L P[%u] %.0fmm/sec FINE ;",
					static_cast<unsigned>(lineIndex),
					static_cast<unsigned>(pointIndex),
					FanucLinearSpeed(info.tSpeed.dSpeed)) << "\n";
			}
			else
			{
				oss << GetStr("%4u:  J P[%u] %d%% FINE ;",
					static_cast<unsigned>(lineIndex),
					static_cast<unsigned>(pointIndex),
					FanucSpeedPercent(info.tSpeed.dSpeed)) << "\n";
			}
		}

		oss << GetStr("%4u:  END ;", static_cast<unsigned>(moveInfos.size() + 3)) << "\n";
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

	std::string FanucGetKtransPath()
	{
		return "C:\\Program Files (x86)\\FANUC\\WinOLPC\\bin\\ktrans.exe";
	}

	std::string FanucGetMaketpPath()
	{
		return "C:\\Program Files (x86)\\FANUC\\WinOLPC\\bin\\maketp.exe";
	}

	bool FanucFileExists(const std::string& filePath)
	{
		std::error_code ec;
		return std::filesystem::exists(filePath, ec);
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
		std::filesystem::remove(pcPath, ec);

		const std::string ktransWorkDir = std::filesystem::path(ktransPath).parent_path().string();
		const std::wstring exePathW(ktransPath.begin(), ktransPath.end());
		const std::wstring workDirW(ktransWorkDir.begin(), ktransWorkDir.end());
		const std::wstring commandTextW =
			L"\"" + exePathW + L"\" \"" +
			std::wstring(klPath.begin(), klPath.end()) + L"\" \"" +
			std::wstring(pcPath.begin(), pcPath.end()) + L"\"";
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

		if (exitCode != 0 || !FanucFileExists(pcPath))
		{
			if (pLog != nullptr)
			{
				pLog->write(LogColor::ERR,
					"FANUC 编译失败：ktrans 返回码=%lu，KL=%s，PC=%s，WorkDir=%s | 耗时=%lldms",
					static_cast<unsigned long>(exitCode), klPath.c_str(), pcPath.c_str(), ktransWorkDir.c_str(),
					FanucElapsedMs(compileStart));
			}
			return false;
		}

		if (pLog != nullptr)
		{
			pLog->write(LogColor::SUCCESS, "FANUC 编译成功：%s | 耗时=%lldms",
				pcPath.c_str(), FanucElapsedMs(compileStart));
		}
		return true;
	}

	bool FanucCompileLsToTp(const std::string& lsPath, const std::string& tpPath, RobotLog* pLog)
	{
		const auto compileStart = std::chrono::steady_clock::now();
		const std::string maketpPath = FanucGetMaketpPath();
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
		std::filesystem::remove(tpPath, ec);

		const std::string maketpWorkDir = std::filesystem::path(maketpPath).parent_path().string();
		const std::wstring exePathW(maketpPath.begin(), maketpPath.end());
		const std::wstring workDirW(maketpWorkDir.begin(), maketpWorkDir.end());
		const std::wstring commandTextW =
			L"\"" + exePathW + L"\" \"" +
			std::wstring(lsPath.begin(), lsPath.end()) + L"\" \"" +
			std::wstring(tpPath.begin(), tpPath.end()) + L"\"";
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

		if (exitCode != 0 || !FanucFileExists(tpPath))
		{
			if (pLog != nullptr)
			{
				pLog->write(LogColor::ERR,
					"FANUC TP编译失败：maketp 返回码=%lu，LS=%s，TP=%s，WorkDir=%s | 耗时=%lldms",
					static_cast<unsigned long>(exitCode), lsPath.c_str(), tpPath.c_str(), maketpWorkDir.c_str(),
					FanucElapsedMs(compileStart));
			}
			return false;
		}

		if (pLog != nullptr)
		{
			pLog->write(LogColor::SUCCESS, "FANUC TP编译成功：%s | 耗时=%lldms",
				tpPath.c_str(), FanucElapsedMs(compileStart));
		}
		return true;
	}
}

FANUCRobotCtrl::FANUCRobotCtrl(std::string strUnitName, RobotLog* pLog)
	: RobotDriverAdaptor(strUnitName, pLog),
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
	m_llMonitorRobotMs(0),
	m_llMonitorPcRecvMs(0),
	m_continuousMoveRunning(false),
	m_continuousMoveStopRequested(false),
	m_continuousMoveRobotStarted(false),
	m_continuousMoveType(MOVJ),
	m_continuousMoveSpeed(1.0),
	m_continuousWrittenCount(0),
	m_continuousConsumedCount(0)
{
	InitRobotDriver(strUnitName);
	m_hMutex = CreateMutexA(nullptr, FALSE, "FANUCRobotMutex");
}

FANUCRobotCtrl::~FANUCRobotCtrl()
{
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
	bool FanucEnsureSocket(FANUCRobotCtrl* ctrl)
	{
		if (ctrl == nullptr)
		{
			return false;
		}
		if (ctrl->m_bSocketConnected)
		{
			return true;
		}
		return ctrl->InitSocket(ctrl->m_sSocketIP.c_str(), static_cast<u_short>(ctrl->m_nSocketPort));
	}

	bool FanucRequest(FANUCRobotCtrl* ctrl, const std::string& command, std::string& response)
	{
		response.clear();
		if (!FanucEnsureSocket(ctrl))
		{
			return false;
		}

		SOCKET sock = FanucGetSocket(ctrl->m_uSocketHandle);
		if (sock == INVALID_SOCKET)
		{
			return false;
		}

		if (ctrl->m_hMutex != nullptr)
		{
			const DWORD waitResult = WaitForSingleObject(ctrl->m_hMutex, FANUC_SOCKET_TIMEOUT_MS);
			if (waitResult != WAIT_OBJECT_0 && waitResult != WAIT_ABANDONED)
			{
				if (ctrl->m_pRobotLog != nullptr)
				{
					ctrl->m_pRobotLog->write(LogColor::ERR, "FANUC Socket CMD=%s 等待互斥锁超时", command.c_str());
				}
				return false;
			}
		}

		const bool sent = FanucSendLine(sock, command);
		const bool recvOk = sent && FanucReceiveLine(sock, response);

		if (ctrl->m_hMutex != nullptr)
		{
			ReleaseMutex(ctrl->m_hMutex);
		}

		if (ctrl->m_pRobotLog != nullptr)
		{
			ctrl->m_pRobotLog->write(sent && recvOk ? LogColor::DEFAULT : LogColor::ERR,
				"FANUC Socket CMD=%s RSP=%s",
				command.c_str(), response.c_str());
		}

		if (!sent || !recvOk)
		{
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
bool FANUCRobotCtrl::InitSocket(const char* ip, u_short Port, bool ifRecode)
{
	(void)ifRecode;
	const std::string socketIp = (ip != nullptr && ip[0] != '\0') ? ip : m_sSocketIP;
	const u_short socketPort = Port > 0 ? Port : static_cast<u_short>(m_nSocketPort);

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

	if (m_bSocketConnected)
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
	m_bSocketConnected = true;

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
	if (m_bSocketConnected)
	{
		SOCKET sock = FanucGetSocket(m_uSocketHandle);
		if (sock != INVALID_SOCKET)
		{
			closesocket(sock);
		}
		m_uSocketHandle = static_cast<std::uintptr_t>(FanucInvalidSocket());
		m_bSocketConnected = false;
	}

	if (m_bWSAStarted)
	{
		FanucCleanupWinSock();
		m_bWSAStarted = false;
	}

	return true;
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
	std::string response;
	double values[6] = {};
	if (!FanucRequest(this, "GET_CUR_POS", response))
	{
		return T_ROBOT_COORS();
	}

	if (!FanucStartsWith(response, "POS:") || !FanucParseDoubles(FanucResponsePayload(response), values, 6))
	{
		return T_ROBOT_COORS();
	}

	return T_ROBOT_COORS(values[0], values[1], values[2], values[3], values[4], values[5], 0, 0, 0);
}

// 被动读取当前TCP位姿缓存，同时可取机器人侧robot_ms和PC侧pc_recv_ms。
T_ROBOT_COORS FANUCRobotCtrl::GetCurrentPosPassive(long long* pRobotMs, long long* pPcRecvMs)
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
	return m_tMonitorPos;
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
	std::string response;
	double values[9] = {};
	const std::string command = "GET_CUR_PULSE:" + std::to_string(m_nRobotAxisCount);
	if (!FanucRequest(this, command, response))
	{
		return T_ANGLE_PULSE();
	}

	if (!FanucStartsWith(response, "PULSE:") || !FanucParseDoubles(FanucResponsePayload(response), values, 9))
	{
		return T_ANGLE_PULSE();
	}

	return T_ANGLE_PULSE(
		FanucPositionToPulse(values[0], m_tAxisUnit.dSPulseUnit),
		FanucPositionToPulse(values[1], m_tAxisUnit.dLPulseUnit),
		FanucPositionToPulse(values[2], m_tAxisUnit.dUPulseUnit),
		FanucPositionToPulse(values[3], m_tAxisUnit.dRPulseUnit),
		FanucPositionToPulse(values[4], m_tAxisUnit.dBPulseUnit),
		FanucPositionToPulse(values[5], m_tAxisUnit.dTPulseUnit),
		FanucPositionToPulse(values[6], m_tAxisUnit.dBXPulseUnit),
		FanucPositionToPulse(values[7], m_tAxisUnit.dBYPulseUnit),
		FanucPositionToPulse(values[8], m_tAxisUnit.dBZPulseUnit));
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

// 主动读取机器人运行状态：0=运行中，1=停止/完成，-1=通信或解析失败。
int FANUCRobotCtrl::CheckDone()
{
	std::string response;
	if (!FanucRequest(this, "CHECK_DONE", response))
	{
		return -1;
	}
	if (!FanucStartsWith(response, "DONE:"))
	{
		return -1;
	}
	return atoi(FanucResponsePayload(response).c_str());
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

// 阻塞等待机器人运动结束；通信失败会返回-1，避免断线时永久卡住。
int FANUCRobotCtrl::CheckRobotDone(int nDelayTime)
{
	if (nDelayTime <= 0)
	{
		nDelayTime = 200;
	}

	int nRet = -1;
	int nStableDoneCount = 0;
	auto lastLogTime = std::chrono::steady_clock::now();

	while (true)
	{
		nRet = CheckDone();
		if (nRet < 0)
		{
			if (m_pRobotLog != nullptr)
			{
				m_pRobotLog->write(LogColor::ERR, "FANUC CheckRobotDone 检测失败，返回=%d", nRet);
			}
			return nRet;
		}

		if (nRet != 0)
		{
			++nStableDoneCount;
			if (nStableDoneCount >= 2)
			{
				return nRet;
			}
		}
		else
		{
			nStableDoneCount = 0;
			if (m_pRobotLog != nullptr && FanucElapsedMs(lastLogTime) >= 5000)
			{
				m_pRobotLog->write(LogColor::DEFAULT, "FANUC CheckRobotDone 等待机器人运行结束...");
				lastLogTime = std::chrono::steady_clock::now();
			}
		}

		Sleep(nDelayTime);
	}
}

// 通过常驻服务异步启动指定TP/KAREL程序。
bool FANUCRobotCtrl::CallJob(std::string sJobName)
{
	std::string response;
	return FanucRequest(this, "CALL_JOB:" + sJobName, response) && FanucIsOkResponse(response);
}

// 请求机器人侧常驻/监控服务自行退出；随后关闭本地S4/S5连接。
bool FANUCRobotCtrl::StopRobotServices()
{
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
	const int FANUC_STREAM_SAFE_GAP = 3;

	int FanucPositiveModulo(long long value, int mod)
	{
		const int result = static_cast<int>(value % mod);
		return result < 0 ? result + mod : result;
	}
}

bool FANUCRobotCtrl::StartContinuousMoveQueue(int nMoveType, double dSpeed)
{
	std::lock_guard<std::mutex> lifecycleLock(m_continuousMoveLifecycleMutex);

	if (m_continuousMoveRunning.load() || m_continuousMoveThread.joinable())
	{
		{
			std::lock_guard<std::mutex> lock(m_continuousMoveMutex);
			m_continuousMoveStopRequested = true;
		}
		m_continuousMoveCv.notify_all();

		if (m_continuousMoveThread.joinable())
		{
			m_continuousMoveThread.join();
		}
		m_continuousMoveRunning.store(false);
	}

	{
		std::lock_guard<std::mutex> lock(m_continuousMoveMutex);
		m_continuousMoveQueue.clear();
		m_continuousMoveStopRequested = false;
		m_continuousMoveRobotStarted = false;
		m_continuousMoveType = nMoveType == MOVL ? MOVL : MOVJ;
		m_continuousMoveSpeed = dSpeed <= 0.0 ? 1.0 : dSpeed;
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
	std::lock_guard<std::mutex> lifecycleLock(m_continuousMoveLifecycleMutex);

	if (!m_continuousMoveRunning.load() && !m_continuousMoveThread.joinable())
	{
		return;
	}

	{
		std::lock_guard<std::mutex> lock(m_continuousMoveMutex);
		m_continuousMoveStopRequested = true;
	}
	m_continuousMoveCv.notify_all();

	if (m_continuousMoveThread.joinable())
	{
		if (m_continuousMoveThread.get_id() != std::this_thread::get_id())
		{
			m_continuousMoveThread.join();
		}
	}
	m_continuousMoveRunning.store(false);
}

bool FANUCRobotCtrl::IsContinuousMoveQueueRunning() const
{
	return m_continuousMoveRunning.load();
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
	const int doneCount = GetIntVar(FANUC_STREAM_DONE_COUNT_REG);
	if (doneCount < 0)
	{
		return -1;
	}
	return doneCount;
}

void FANUCRobotCtrl::ContinuousMoveWorker()
{
	bool ok = true;
	bool hasLastPoint = false;
	T_ROBOT_MOVE_INFO lastPoint;

	SetIntVar(FANUC_STREAM_RUN_REG, 0);
	SetIntVar(FANUC_STREAM_DONE_COUNT_REG, 0);
	SetIntVar(FANUC_STREAM_END_COUNT_REG, 0);
	SetIntVar(FANUC_STREAM_LOAD_BUFFER_REG, 0);
	SetIntVar(FANUC_STREAM_LOAD_STATUS_REG, 0);

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
		SetTpSpeed(static_cast<int>(std::max(1.0, m_continuousMoveSpeed)));
		SetIntVar(FANUC_STREAM_LOAD_STATUS_REG, 0);
		ok = SetIntVar(FANUC_STREAM_RUN_REG, 1) &&
			CallJob(m_continuousMoveType == MOVL ? "FANUC_JOGL" : "FANUC_JOGJ");

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
				ok = true;
				for (int i = 0; i < static_cast<int>(startBuffer.size()); ++i)
				{
					ok = WriteContinuousMovePointToRobot(FANUC_STREAM_FIRST_PR + i, startBuffer[static_cast<size_t>(i)]);
					if (!ok)
					{
						break;
					}
				}

				if (ok)
				{
					SetIntVar(FANUC_STREAM_LOAD_BUFFER_REG, 2);
					SetIntVar(FANUC_STREAM_LOAD_STATUS_REG, 0);
					ok = SetIntVar(FANUC_STREAM_RUN_REG, 1) &&
						CallJob(m_continuousMoveType == MOVL ? "FANUC_JOGL" : "FANUC_JOGJ");
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
		m_continuousMoveRobotStarted = ok;
		m_continuousConsumedCount = std::max(0, ReadContinuousDoneCount());
	}

	while (ok && m_continuousMoveRobotStarted)
	{
		bool stopRequested = false;
		{
			std::lock_guard<std::mutex> lock(m_continuousMoveMutex);
			stopRequested = m_continuousMoveStopRequested;
		}

		if (stopRequested)
		{
			if (m_pRobotLog != nullptr)
			{
				m_pRobotLog->write(LogColor::DEFAULT,
					"FANUC 连续运动收到停止请求: endCount=%lld consumed=%lld",
					m_continuousWrittenCount,
					m_continuousConsumedCount);
			}
			SetIntVar(FANUC_STREAM_END_COUNT_REG, static_cast<int>(m_continuousWrittenCount));
			SetIntVar(FANUC_STREAM_RUN_REG, 0);
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
			m_continuousMoveRobotStarted ? 1 : 0);
	}

	m_continuousMoveRunning.store(false);
}

// ===================== 被动监控通道 =====================

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
				if (!FanucConnectSocket(sock, m_sSocketIP, static_cast<u_short>(nPort)))
				{
					{
						std::lock_guard<std::mutex> lock(m_monitorMutex);
						m_sMonitorText = GetStr("状态: 监控连接失败 %s:%d", m_sSocketIP.c_str(), nPort);
					}
					Sleep(1000);
					continue;
				}

				m_uMonitorSocketHandle = static_cast<std::uintptr_t>(sock);
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
						m_nMonitorDone = done;
						m_llMonitorRobotMs = robotMs;
						m_llMonitorPcRecvMs = pcRecvMs;
					}
				}

				closesocket(sock);
				m_uMonitorSocketHandle = static_cast<std::uintptr_t>(FanucInvalidSocket());
				if (m_bMonitorRunning)
				{
					std::lock_guard<std::mutex> lock(m_monitorMutex);
					m_sMonitorText = "状态: 监控连接断开，正在重连...";
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
	SOCKET sock = FanucGetSocket(m_uMonitorSocketHandle);
	if (sock != INVALID_SOCKET)
	{
		closesocket(sock);
		m_uMonitorSocketHandle = static_cast<std::uintptr_t>(FanucInvalidSocket());
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

// ===================== 连续运动与特殊程序上传 =====================

// 多点连续运动：生成临时KL/VAR文件并上传，适合路径/焊接等非固定单点动作。
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

// 上传三角摆焊KAREL程序，并写入焊缝编号参数。
bool FANUCRobotCtrl::SendWeldTriangleWeaveProgram(int nWeldTrackNum)
{
	const std::string filePath = FanucBuildProgramPath(m_sRobotName, "WeldTriangleWeave.kl");
	if (filePath.empty())
	{
		return false;
	}
	return UploadKlFile(filePath) == 0 && SetIntVar("WELD_TRACK_NO", nWeldTrackNum);
}

// 上传L形摆焊KAREL程序，并写入焊缝编号参数。
bool FANUCRobotCtrl::SendWeldLWeaveProgram(int nWeldTrackNum)
{
	const std::string filePath = FanucBuildProgramPath(m_sRobotName, "WeldLWeave.kl");
	if (filePath.empty())
	{
		return false;
	}
	return UploadKlFile(filePath) == 0 && SetIntVar("WELD_TRACK_NO", nWeldTrackNum);
}

// 上传普通焊接KAREL程序，并写入焊缝编号参数。
bool FANUCRobotCtrl::SendWeldProgram(int nWeldTrackNum)
{
	const std::string filePath = FanucBuildProgramPath(m_sRobotName, "WeldProgram.kl");
	if (filePath.empty())
	{
		return false;
	}
	return UploadKlFile(filePath) == 0 && SetIntVar("WELD_TRACK_NO", nWeldTrackNum);
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
	std::string response;
	return FanucRequest(this, "PROGRAM_STOP", response) && FanucIsOkResponse(response);
}

// 设置工具号；固定TP当前默认UT=1，后续可扩展为动态工具号。
bool FANUCRobotCtrl::SetRobotToolNo(int nToolNo)
{
	std::string response;
	return FanucRequest(this, "SET_TOOL_NO:" + std::to_string(nToolNo), response) && FanucIsOkResponse(response);
}

// 获取工具数据；当前预留协议入口。
bool FANUCRobotCtrl::GetToolData(int unToolNo, T_ROBOT_COORS adRobotToolData)
{
	(void)adRobotToolData;
	std::string response;
	return FanucRequest(this, "GET_TOOL_DATA:" + std::to_string(unToolNo), response) && FanucStartsWith(response, "TOOL:");
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

//bool FANUCRobotCtrl::SetPosVar(int nIndex, AXISPOS eRobotCoors, int scoper)
//{
//	(void)nIndex;
//	(void)eRobotCoors;
//	(void)scoper;
//	return false;
//}
//
//bool FANUCRobotCtrl::SetPosVar(int nIndex, JointsPos eRobotCoors, int scoper)
//{
//	(void)nIndex;
//	(void)eRobotCoors;
//	(void)scoper;
//	return false;
//}

// 读取位置寄存器；当前协议仅保留基础返回，后续可按现场需要补完整解析。
int FANUCRobotCtrl::GetPosVar(long lPvarIndex, double array[6], int config[7], int MoveType)
{
	(void)config;
	std::string response;
	if (!FanucRequest(this, "GET_POS_VAR:" + std::to_string(lPvarIndex) + "," + std::to_string(MoveType), response))
	{
		return -1;
	}
	if (!FanucStartsWith(response, "POSVAR:") || !FanucParseDoubles(FanucResponsePayload(response), array, 6))
	{
		return -1;
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
	std::string response;
	const std::string prefix = cStrPreFix == nullptr ? "INT" : cStrPreFix;
	if (!FanucRequest(this, "GET_INT:" + prefix + "," + std::to_string(nIndex), response))
	{
		return 0;
	}
	if (!FanucStartsWith(response, "INT:"))
	{
		return 0;
	}
	return atoi(FanucResponsePayload(response).c_str());
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

// 旧版按单轴距离调用任务接口；保留给既有上层代码。
bool FANUCRobotCtrl::MoveByJob(int Axis, double Distence, int config[7], double speed, int ifAbsoluteM, int ifJoint)
{
	std::vector<std::string> parts =
	{
		std::to_string(Axis),
		GetStr("%.6f", Distence),
		GetStr("%.6f", speed),
		std::to_string(ifAbsoluteM),
		std::to_string(ifJoint),
		FanucBuildConfigText(config)
	};
	std::string response;
	return FanucRequest(this, "MOVE_BY_JOB_AXIS:" + FanucJoin(parts, ','), response) && FanucIsOkResponse(response);
}

// 旧版按数组位置调用任务接口；保留给既有上层代码。
bool FANUCRobotCtrl::MoveByJob(double Distence[8], int config[7], double speed, int ifAbsolutM, int ifJoint)
{
	std::vector<std::string> parts =
	{
		GetStr("%.6f", Distence[0]), GetStr("%.6f", Distence[1]), GetStr("%.6f", Distence[2]), GetStr("%.6f", Distence[3]),
		GetStr("%.6f", Distence[4]), GetStr("%.6f", Distence[5]), GetStr("%.6f", Distence[6]), GetStr("%.6f", Distence[7]),
		GetStr("%.6f", speed),
		std::to_string(ifAbsolutM),
		std::to_string(ifJoint),
		FanucBuildConfigText(config)
	};
	std::string response;
	return FanucRequest(this, "MOVE_BY_JOB_POS:" + FanucJoin(parts, ','), response) && FanucIsOkResponse(response);
}

// 固定TP单点运动公共流程：设置目标PR/速度R，必要时上传固定TP，然后调用任务。
static bool FanucCreateUploadRunTpMove(FANUCRobotCtrl* ctrl, const std::vector<T_ROBOT_MOVE_INFO>& moveInfos)
{
	// Single-point MOVL/MOVJ uses fixed TP programs:
	// update PR[1] and R[17], upload the fixed TP once, then CALL_JOB.
	// This avoids invoking maketp.exe for every small jog command.
	if (ctrl == nullptr || moveInfos.empty())
	{
		return false;
	}
	if (moveInfos.size() != 1)
	{
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
		if (ctrl->m_pRobotLog != nullptr)
		{
			ctrl->m_pRobotLog->write(LogColor::ERR, "FANUC 固定TP不存在：%s.tp", programName.c_str());
		}
		return false;
	}

	const int speed = moveType == MOVL
		? static_cast<int>(std::lround(FanucLinearSpeed(moveInfo.tSpeed.dSpeed)))
		: FanucSpeedPercent(moveInfo.tSpeed.dSpeed);

	if (!ctrl->SetTpSpeed(speed))
	{
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

	if (!ctrl->CallJob(programName))
	{
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
