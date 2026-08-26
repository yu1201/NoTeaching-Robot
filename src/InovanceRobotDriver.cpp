#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>

#include "InovanceRobotDriver.h"
#include "RobotDriverRegistry.h"
#include "RobotFtpFileTransfer.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cerrno>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <sstream>
#include <thread>

#pragma comment(lib, "ws2_32.lib")

namespace
{
constexpr int kDefaultTimeoutMs = 3000;
constexpr std::size_t kMaxProtocolResponse = 32768;
constexpr double kMaxLinearSpeedMmPerMin = 120000.0;

SOCKET ToSocket(std::uintptr_t handle)
{
    return static_cast<SOCKET>(handle);
}

std::uintptr_t FromSocket(SOCKET socket)
{
    return static_cast<std::uintptr_t>(socket);
}

std::string Trim(std::string value)
{
    const auto notSpace = [](unsigned char ch) { return !std::isspace(ch); };
    value.erase(value.begin(), std::find_if(value.begin(), value.end(), notSpace));
    value.erase(std::find_if(value.rbegin(), value.rend(), notSpace).base(), value.end());
    return value;
}

std::string ValuePart(const std::string& response)
{
    std::string value = Trim(response);
    if (!value.empty() && value.front() == '=')
    {
        value.erase(value.begin());
    }
    return Trim(value);
}

std::vector<double> ParseNumbers(std::string value)
{
    for (char& ch : value)
    {
        if (ch == ',' || ch == ';' || ch == '[' || ch == ']')
        {
            ch = ' ';
        }
    }
    std::istringstream stream(value);
    std::vector<double> result;
    double number = 0.0;
    while (stream >> number)
    {
        result.push_back(number);
    }
    return result;
}

std::string FormatDouble(double value)
{
    std::ostringstream stream;
    // Vendor string examples use millimetre/degree values with 3 decimals. This also
    // keeps a complete ROB_POS parameter inside the documented 128-character limit.
    stream << std::fixed << std::setprecision(3) << value;
    return stream.str();
}

long PulseAt(const T_ANGLE_PULSE& pulse, int index)
{
    switch (index)
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

double PoseAt(const T_ROBOT_COORS& pose, int index)
{
    switch (index)
    {
    case 0: return pose.dX;
    case 1: return pose.dY;
    case 2: return pose.dZ;
    case 3: return pose.dRX;
    case 4: return pose.dRY;
    case 5: return pose.dRZ;
    case 6: return pose.dBX;
    case 7: return pose.dBY;
    case 8: return pose.dBZ;
    default: return 0.0;
    }
}

double NormalizeAngleDifference(double left, double right)
{
    double difference = left - right;
    while (difference > 180.0) { difference -= 360.0; }
    while (difference < -180.0) { difference += 360.0; }
    return difference;
}

bool SameProgramHandle(
    const RobotTrajectoryHandle& left,
    const RobotTrajectoryHandle& right)
{
    return !left.programName.empty() && left.programName == right.programName;
}

bool NormalizeInovanceRemotePath(
    std::string path,
    std::string& normalized,
    std::string& error)
{
    normalized.clear();
    error.clear();
    path = Trim(std::move(path));
    std::replace(path.begin(), path.end(), '\\', '/');
    if (path.empty() || path.find('\0') != std::string::npos)
    {
        error = "远端路径为空或包含无效字符。";
        return false;
    }

    std::vector<std::string> components;
    std::size_t begin = 0;
    while (begin <= path.size())
    {
        const std::size_t end = path.find('/', begin);
        const std::string component = path.substr(
            begin, end == std::string::npos ? std::string::npos : end - begin);
        if (!component.empty() && component != ".")
        {
            if (component == "..")
            {
                error = "远端路径不允许包含上级目录。";
                return false;
            }
            components.push_back(component);
        }
        if (end == std::string::npos) { break; }
        begin = end + 1;
    }
    if (components.empty())
    {
        error = "远端路径没有有效目录或文件名。";
        return false;
    }

    for (const std::string& component : components)
    {
        normalized.push_back('/');
        normalized += component;
    }
    return true;
}

bool InovanceActiveProjectDirectory(
    const std::string& taskProgramPath,
    std::string& directory,
    std::string& error)
{
    std::string normalizedPath;
    if (!NormalizeInovanceRemotePath(taskProgramPath, normalizedPath, error))
    {
        return false;
    }
    std::string lowerPath = normalizedPath;
    std::transform(lowerPath.begin(), lowerPath.end(), lowerPath.begin(),
        [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    if (lowerPath.rfind("/teachprogram/", 0) != 0
        || lowerPath.size() <= std::strlen("/teachprogram/x.pro")
        || lowerPath.substr(lowerPath.size() - 4) != ".pro")
    {
        error = "主任务路径不是 TeachProgram/工程名/*.pro，无法确定安全上传目录："
            + normalizedPath;
        return false;
    }
    const std::size_t slash = normalizedPath.find_last_of('/');
    if (slash == std::string::npos || slash <= std::strlen("/TeachProgram"))
    {
        error = "主任务路径缺少工程目录：" + normalizedPath;
        return false;
    }
    directory = normalizedPath.substr(0, slash);
    return true;
}
}

InovanceRobotCtrl::InovanceRobotCtrl(std::string unitName, RobotLog* log)
    : RobotDriverAdaptor(unitName, log)
{
    InitRobotDriver(std::move(unitName));
}

InovanceRobotCtrl::~InovanceRobotCtrl()
{
    StopStateMonitor();
    EndContinuousJog();
    if (IsConnected())
    {
        ShutdownBeforeDisconnect();
    }
    Disconnect();
}

long long InovanceRobotCtrl::SteadyMs()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

bool InovanceRobotCtrl::InitRobotDriver(std::string unitName)
{
    if (const RobotDriverSetupProfile* setup =
        RobotDriverRegistry::SetupProfile(ROBOT_TYPE_INOVANCE))
    {
        m_socketPort = setup->defaultSocketPort;
        m_ftpIp = setup->defaultFtpHost;
        m_ftpPort = setup->defaultFtpPort;
        m_ftpUser = setup->defaultFtpUser;
        m_ftpPassword = setup->defaultFtpPassword;
    }

    ConfigSection ini;
    ini.SetLocation(ConfigLocation::Robot(QString::fromUtf8(unitName.c_str()), QStringLiteral("RobotPara")));
    ini.SetSectionName("BaseParam");
    ini.ReadString("RobotName", m_sRobotName);
    ini.ReadString("CustomName", m_sCustomName);
    ini.ReadString("SocketIP", m_socketIp);
    ini.ReadString(false, "SocketPort", &m_socketPort);
    ini.ReadString(false, "RobotType", &m_nRobotType);
    ini.ReadString(false, "RobotBrand", reinterpret_cast<int*>(&m_eRobotBrand));
    ini.ReadString(false, "ToolNo", &m_toolNo);
    ini.ReadString(false, "WobjNo", &m_wobjNo);
    ini.ReadString(false, "MaxBufferedCommands", &m_maxBufferedCommands);
    ini.ReadString(false, "ForceControlPermit", &m_forceControlPermit);
    ini.ReadString(false, "ApiUserLevel", &m_apiUserLevel);
    ini.ReadString(false, "ApiPassword", &m_apiPassword);
    ini.ReadString(false, "FTPIP", &m_ftpIp);
    ini.ReadString(false, "FTPPort", &m_ftpPort);
    ini.ReadString(false, "FTPUser", &m_ftpUser);
    ini.ReadString(false, "FTPPassWord", &m_ftpPassword);

    if (m_socketPort <= 0 || m_socketPort > 65535) { m_socketPort = 2222; }
    if (m_ftpIp.empty()) { m_ftpIp = m_socketIp; }
    if (m_ftpPort <= 0 || m_ftpPort > 65535) { m_ftpPort = 7777; }
    if (m_nRobotType == 0) { m_nRobotType = ROBOT_TYPE_INOVANCE; }
    m_toolNo = std::clamp(m_toolNo, 0, 15);
    m_wobjNo = std::clamp(m_wobjNo, 0, 15);
    m_maxBufferedCommands = std::clamp(m_maxBufferedCommands, 1, 32);
    m_apiUserLevel = std::clamp(m_apiUserLevel, 0, 3);

    LoadRobotExternalAxlePara(unitName);

    ini.SetSectionName("Tool");
    ini.ReadString("PolisherTool_d", "", m_tTools.tPolisherTool,
        T_ROBOT_COORS(1, 1, 1, 1, 1, 1, -1, -1, -1));
    ini.ReadString("MagnetTool_d", "", m_tTools.tMagnetTool,
        T_ROBOT_COORS(1, 1, 1, 1, 1, 1, -1, -1, -1));
    ini.ReadString("GunTool_d", "", m_tTools.tGunTool,
        T_ROBOT_COORS(1, 1, 1, 1, 1, 1, -1, -1, -1));
    ini.ReadString("CameraTool_d", "", m_tTools.tCameraTool,
        T_ROBOT_COORS(1, 1, 1, 1, 1, 1, -1, -1, -1));
    return true;
}

RobotDriverDescriptor InovanceRobotCtrl::DriverDescriptor() const
{
    // 汇川原始姿态为 X,Y,Z,A,B,C；手册寄存器说明 A/B/C 对应 Rz/Ry/Rx。
    // 映射到通用 dRX/dRY/dRZ 后采用项目既有 Rz*Ry*Rx 约定。
    return RobotDriverDescriptor{
        RobotDriverFamily::Inovance,
        ROBOT_TYPE_INOVANCE,
        ROBOT_TYPE_FANUC,
        "INOVANCE",
        "汇川 Inovance"
    };
}

std::uint64_t InovanceRobotCtrl::DriverCapabilities() const
{
    std::uint64_t capabilities = RobotDriverCapabilityBit(RobotDriverCapability::PassiveState)
        | RobotDriverCapabilityBit(RobotDriverCapability::LinearMotion)
        | RobotDriverCapabilityBit(RobotDriverCapability::ContinuousTrajectory)
        | RobotDriverCapabilityBit(RobotDriverCapability::ContinuousJog)
        | RobotDriverCapabilityBit(RobotDriverCapability::PauseResume)
        | RobotDriverCapabilityBit(RobotDriverCapability::OperationModeControl)
        | RobotDriverCapabilityBit(RobotDriverCapability::NativeProgramUpload)
        | RobotDriverCapabilityBit(RobotDriverCapability::DiagnosticCommand)
        | RobotDriverCapabilityBit(RobotDriverCapability::CartesianRegister)
        | RobotDriverCapabilityBit(RobotDriverCapability::VerifiedProgramCompletion)
        | RobotDriverCapabilityBit(RobotDriverCapability::VerifiedSafeAbort)
        | RobotDriverCapabilityBit(RobotDriverCapability::ConnectionControl)
        | RobotDriverCapabilityBit(RobotDriverCapability::AlarmReset)
        | RobotDriverCapabilityBit(RobotDriverCapability::ServoPowerControl)
        | RobotDriverCapabilityBit(RobotDriverCapability::ToolDataRead)
        | RobotDriverCapabilityBit(RobotDriverCapability::TeachPendantSpeedControl)
        | RobotDriverCapabilityBit(RobotDriverCapability::FtpFileTransfer);
    const double mainAxisUnits[6] = {
        m_tAxisUnit.dSPulseUnit, m_tAxisUnit.dLPulseUnit, m_tAxisUnit.dUPulseUnit,
        m_tAxisUnit.dRPulseUnit, m_tAxisUnit.dBPulseUnit, m_tAxisUnit.dTPulseUnit
    };
    const bool jointUnitsReady = std::all_of(
        std::begin(mainAxisUnits), std::end(mainAxisUnits),
        [](double unit) { return std::isfinite(unit) && std::abs(unit) >= 1e-15; });
    if (jointUnitsReady)
    {
        capabilities |= RobotDriverCapabilityBit(RobotDriverCapability::JointMotion);
    }
    if (m_nExternalAxleType != 0)
    {
        capabilities |= RobotDriverCapabilityBit(RobotDriverCapability::ExternalAxis);
    }
    return capabilities;
}

RobotConnectionEndpoint InovanceRobotCtrl::ControlEndpoint() const
{
    return RobotConnectionEndpoint{ m_socketIp, m_socketPort };
}

bool InovanceRobotCtrl::CloseSocketLocked()
{
    const SOCKET socket = ToSocket(m_socketHandle);
    if (socket != INVALID_SOCKET)
    {
        shutdown(socket, SD_BOTH);
        closesocket(socket);
    }
    m_socketHandle = FromSocket(INVALID_SOCKET);
    m_connected.store(false);
    m_permitOwned.store(false);
    m_dataStreamEnabled.store(false);
    if (m_wsaStarted)
    {
        WSACleanup();
        m_wsaStarted = false;
    }
    return true;
}

bool InovanceRobotCtrl::Connect()
{
    ClearLastRobotError();
    const RobotConnectionEndpoint endpoint = ControlEndpoint();
    if (!endpoint.IsValid())
    {
        SetLastRobotError("汇川连接参数不完整，控制端口应为控制器远程以太网端口2222。");
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(m_socketMutex);
        if (m_connected.load()) { return true; }
        if (!m_wsaStarted)
        {
            WSADATA data = {};
            if (WSAStartup(MAKEWORD(2, 2), &data) != 0)
            {
                SetLastRobotError("汇川连接失败：WSAStartup失败。");
                return false;
            }
            m_wsaStarted = true;
        }

        addrinfo hints = {};
        hints.ai_family = AF_UNSPEC;
        hints.ai_socktype = SOCK_STREAM;
        hints.ai_protocol = IPPROTO_TCP;
        addrinfo* addresses = nullptr;
        const std::string port = std::to_string(endpoint.port);
        const int lookup = getaddrinfo(endpoint.host.c_str(), port.c_str(), &hints, &addresses);
        if (lookup != 0)
        {
            CloseSocketLocked();
            SetLastRobotError("汇川连接失败：控制器地址无法解析。");
            return false;
        }

        SOCKET connectedSocket = INVALID_SOCKET;
        for (addrinfo* address = addresses; address != nullptr; address = address->ai_next)
        {
            SOCKET candidate = socket(address->ai_family, address->ai_socktype, address->ai_protocol);
            if (candidate == INVALID_SOCKET) { continue; }
            const DWORD timeout = kDefaultTimeoutMs;
            setsockopt(candidate, SOL_SOCKET, SO_RCVTIMEO,
                reinterpret_cast<const char*>(&timeout), sizeof(timeout));
            setsockopt(candidate, SOL_SOCKET, SO_SNDTIMEO,
                reinterpret_cast<const char*>(&timeout), sizeof(timeout));
            if (connect(candidate, address->ai_addr, static_cast<int>(address->ai_addrlen)) == 0)
            {
                connectedSocket = candidate;
                break;
            }
            closesocket(candidate);
        }
        freeaddrinfo(addresses);
        if (connectedSocket == INVALID_SOCKET)
        {
            const int socketError = WSAGetLastError();
            CloseSocketLocked();
            SetLastRobotError("汇川连接失败：无法连接 " + endpoint.host + ":"
                + std::to_string(endpoint.port) + "，Winsock=" + std::to_string(socketError) + "。");
            return false;
        }
        m_socketHandle = FromSocket(connectedSocket);
        m_connected.store(true);
    }

    int connectionState = 0;
    if (!QueryInt("Get_ConnectState", connectionState) || connectionState != 1)
    {
        SetLastRobotError("汇川TCP已建立，但Get_ConnectState未确认上位机连接状态为1。");
        Disconnect();
        return false;
    }
    ClearLastRobotError();
    if (m_pRobotLog != nullptr)
    {
        m_pRobotLog->write(LogColor::SUCCESS,
            "汇川远程以太网已连接：%s:%d", endpoint.host.c_str(), endpoint.port);
    }
    return true;
}

bool InovanceRobotCtrl::Disconnect()
{
    std::lock_guard<std::mutex> lock(m_socketMutex);
    return CloseSocketLocked();
}

void InovanceRobotCtrl::EnsureConnectionForMonitor()
{
    if (IsConnected()) { return; }
    const long long now = SteadyMs();
    long long previous = m_lastConnectAttemptMs.load();
    if (now - previous < 2000) { return; }
    if (!m_lastConnectAttemptMs.compare_exchange_strong(previous, now))
    {
        return;
    }
    Connect();
}

std::string InovanceRobotCtrl::ProtocolErrorText(const std::string& response)
{
    std::string code = Trim(response);
    const std::size_t end = code.find_first_of(" ,;:");
    if (end != std::string::npos) { code.resize(end); }
    if (code == "e11") return "运动缓存仍有未完成指令";
    if (code == "e16") return "当前状态不允许暂停";
    if (code == "e18") return "控制器模式冲突";
    if (code == "e24") return "当前连接没有控制许可";
    if (code == "e25") return "控制设备不是远程以太网";
    if (code == "e27") return "当前用户级别不足";
    if (code == "e28") return "控制许可被其它客户端占用";
    if (code == "e36") return "示教器按键控制权阻止远程手动模式";
    return "控制器返回错误";
}

bool InovanceRobotCtrl::SendCommand(
    const std::string& command,
    std::string& response,
    int timeoutMs)
{
    response.clear();
    if (command.empty() || command.size() > 16384
        || command.find("@@") != std::string::npos
        || command.find("$$") != std::string::npos)
    {
        SetLastRobotError("汇川命令格式无效：适配层只接受不含协议包头包尾的单条命令。");
        return false;
    }

    std::lock_guard<std::mutex> lock(m_socketMutex);
    if (!m_connected.load() || ToSocket(m_socketHandle) == INVALID_SOCKET)
    {
        SetLastRobotError("汇川命令失败：机器人未连接。");
        return false;
    }

    SOCKET socket = ToSocket(m_socketHandle);
    const DWORD timeout = static_cast<DWORD>(std::clamp(timeoutMs, 100, 60000));
    setsockopt(socket, SOL_SOCKET, SO_RCVTIMEO,
        reinterpret_cast<const char*>(&timeout), sizeof(timeout));
    setsockopt(socket, SOL_SOCKET, SO_SNDTIMEO,
        reinterpret_cast<const char*>(&timeout), sizeof(timeout));

    const std::string request = "@@" + command + "$$";
    std::size_t sent = 0;
    while (sent < request.size())
    {
        const int count = send(socket, request.data() + sent,
            static_cast<int>(request.size() - sent), 0);
        if (count <= 0)
        {
            const int socketError = WSAGetLastError();
            CloseSocketLocked();
            SetLastRobotError("汇川命令发送失败，连接已关闭，Winsock="
                + std::to_string(socketError) + "。");
            return false;
        }
        sent += static_cast<std::size_t>(count);
    }

    std::string framed;
    std::array<char, 2048> buffer = {};
    while (framed.find("$$") == std::string::npos)
    {
        const int count = recv(socket, buffer.data(), static_cast<int>(buffer.size()), 0);
        if (count <= 0)
        {
            const int socketError = WSAGetLastError();
            CloseSocketLocked();
            SetLastRobotError("汇川命令应答超时或连接中断，连接已关闭，Winsock="
                + std::to_string(socketError) + "。");
            return false;
        }
        framed.append(buffer.data(), static_cast<std::size_t>(count));
        if (framed.size() > kMaxProtocolResponse)
        {
            CloseSocketLocked();
            SetLastRobotError("汇川命令应答超过32KiB安全上限，连接已关闭。");
            return false;
        }
    }

    const std::size_t begin = framed.find("##");
    const std::size_t finish = framed.find("$$", begin == std::string::npos ? 0 : begin + 2);
    if (begin == std::string::npos || finish == std::string::npos)
    {
        CloseSocketLocked();
        SetLastRobotError("汇川命令应答缺少##/$$协议边界，连接已关闭。");
        return false;
    }
    response = Trim(framed.substr(begin + 2, finish - (begin + 2)));
    if (!response.empty() && (response.front() == 'e' || response.front() == 'E'))
    {
        const std::string operation = command.substr(0, command.find(' '));
        SetLastRobotError("汇川命令 " + operation + " 失败："
            + ProtocolErrorText(response) + "（" + response + "）。");
        return false;
    }
    ClearLastRobotError();
    return true;
}

bool InovanceRobotCtrl::QueryInt(const std::string& command, int& value)
{
    std::string response;
    if (!SendCommand(command, response)) { return false; }
    const std::string text = ValuePart(response);
    char* end = nullptr;
    errno = 0;
    const long parsed = std::strtol(text.c_str(), &end, 0);
    if (errno != 0 || end == text.c_str()
        || parsed < std::numeric_limits<int>::min()
        || parsed > std::numeric_limits<int>::max())
    {
        SetLastRobotError("汇川命令 " + command + " 返回的整数格式无效。");
        return false;
    }
    value = static_cast<int>(parsed);
    return true;
}

bool InovanceRobotCtrl::QueryDoubles(
    const std::string& command,
    std::vector<double>& values,
    std::size_t minimumCount)
{
    std::string response;
    if (!SendCommand(command, response)) { return false; }
    values = ParseNumbers(ValuePart(response));
    if (values.size() < minimumCount
        || !std::all_of(values.cbegin(), values.cend(), [](double value) { return std::isfinite(value); }))
    {
        SetLastRobotError("汇川命令 " + command + " 返回的数值数量或格式无效。");
        return false;
    }
    return true;
}

RobotFileTransferProfile InovanceRobotCtrl::FileTransferProfile() const
{
    RobotFileTransferProfile profile;
    profile.robotName = m_sRobotName;
    profile.endpointDisplay = m_ftpIp.empty()
        ? std::string()
        : m_ftpIp + ":" + std::to_string(m_ftpPort);
    profile.defaultRemoteDirectory = "/TeachProgram";
    profile.defaultLocalDirectory = "Job/Inovance";
    profile.localFileFilters = { "*.pro", "*.prj", "*.pts", "*.jsn" };
    return profile;
}

std::shared_ptr<RobotFileTransferSession> InovanceRobotCtrl::CreateFileTransferSession(
    std::string* error) const
{
    if (m_ftpIp.empty() || m_ftpPort <= 0 || m_ftpPort > 65535)
    {
        if (error != nullptr)
        {
            *error = "汇川机器人FTP参数不完整。";
        }
        return {};
    }
    if (error != nullptr) { error->clear(); }
    return std::make_shared<RobotFtpFileTransfer>(
        m_ftpIp,
        m_ftpPort,
        m_ftpUser,
        m_ftpPassword,
        FileTransferProfile(),
        std::vector<std::string>{ ".pro", ".prj", ".pts", ".jsn" },
        std::vector<std::string>{ ".pro" },
        "Log/InovanceRobotFtp.log");
}

bool InovanceRobotCtrl::EnsureControlPermit()
{
    int controlDevice = -1;
    if (!QueryInt("CurCtrlDev", controlDevice)) { return false; }
    if (controlDevice != 2)
    {
        SetLastRobotError("汇川控制权不属于远程以太网设备：请在示教器/InoRobotLab中切换到远程以太网控制后重试。");
        return false;
    }

    int owner = -1;
    if (!QueryInt("CurPermit", owner)) { return false; }
    if (owner == 1)
    {
        m_permitOwned.store(true);
        return true;
    }
    if (owner == 2 && !m_forceControlPermit)
    {
        SetLastRobotError("汇川控制许可正被其它远程客户端占用；默认禁止强抢。"
            "确认现场安全后可在机器人BaseParam设置ForceControlPermit=1。");
        return false;
    }

    std::string response;
    const char* command = owner == 2 ? "AcqPermit forcibly" : "AcqPermit";
    if (!SendCommand(command, response) || response != "ok")
    {
        if (GetLastRobotError().empty())
        {
            SetLastRobotError("汇川获取远程控制许可失败。");
        }
        return false;
    }
    if (!QueryInt("CurPermit", owner) || owner != 1)
    {
        SetLastRobotError("汇川控制器未确认当前连接拥有控制许可。");
        return false;
    }
    m_permitOwned.store(true);
    return true;
}

bool InovanceRobotCtrl::EnsureMotionReady()
{
    if (!IsConnected() && !Connect()) { return false; }
    if (!EnsureControlPermit()) { return false; }

    int emergencyStop = -1;
    if (!QueryInt("Get_EStopSts", emergencyStop)) { return false; }
    if (emergencyStop != 0)
    {
        SetLastRobotError("汇川机器人处于急停状态，禁止启动运动。");
        return false;
    }
    int systemError = -1;
    if (!QueryInt("Get_SysErrSts", systemError)) { return false; }
    if (systemError != 0)
    {
        SetLastRobotError("汇川控制器存在报警或警告，需先复位并确认故障状态为0。");
        return false;
    }
    int motor = -1;
    if (!QueryInt("Get_MotorSts", motor)) { return false; }
    if (motor != 1)
    {
        SetLastRobotError("汇川伺服尚未上电；业务层需先通过适配层ServoOn并确认回读。");
        return false;
    }
    return true;
}

bool InovanceRobotCtrl::SetDataStreamMode(const char* action, int expectedMode)
{
    if (action == nullptr)
    {
        SetLastRobotError("汇川数据流模式动作为空。");
        return false;
    }
    int currentMode = -1;
    if (QueryInt("Get_DsMode", currentMode) && currentMode == expectedMode)
    {
        m_dataStreamEnabled.store(currentMode != 0);
        return true;
    }
    std::string response;
    if (!SendCommand(std::string("Dsmode ") + action, response) || response != "ok")
    {
        return false;
    }
    int mode = -1;
    for (int attempt = 0; attempt < 20; ++attempt)
    {
        if (QueryInt("Get_DsMode", mode) && mode == expectedMode)
        {
            m_dataStreamEnabled.store(mode != 0);
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
    SetLastRobotError("汇川数据流模式动作未通过Get_DsMode回读确认：期望="
        + std::to_string(expectedMode) + "，实际=" + std::to_string(mode) + "。");
    return false;
}

bool InovanceRobotCtrl::InitializeAfterConnect(std::string* summary)
{
    if (!IsConnected() && !Connect()) { return false; }
    if (!EnsureControlPermit()) { return false; }

    if (m_apiUserLevel > 0)
    {
        if (m_apiPassword.empty())
        {
            SetLastRobotError("汇川配置要求登录编辑/管理模式，但ApiPassword为空。");
            return false;
        }
        std::string response;
        if (!SendCommand("UserLogin " + std::to_string(m_apiUserLevel) + " " + m_apiPassword, response)
            || response != "ok")
        {
            return false;
        }
        int actualLevel = -1;
        if (!QueryInt("CurUserType", actualLevel) || actualLevel < m_apiUserLevel)
        {
            SetLastRobotError("汇川用户级别登录后未通过CurUserType回读确认。");
            return false;
        }
        m_userLoggedIn.store(true);
    }

    // 工具/工件切换属于管理级控制动作；仅在配置明确指定时设置并逐项回读。
    if (m_toolNo != 0 || m_apiUserLevel >= 2)
    {
        std::string response;
        int actual = -1;
        if (!SendCommand("Set_ToolCNum " + std::to_string(m_toolNo), response)
            || response != "ok" || !QueryInt("Get_ToolCNum", actual) || actual != m_toolNo)
        {
            SetLastRobotError("汇川激活工具号设置或回读失败；确认用户级别及ToolNo配置。");
            return false;
        }
    }
    int activeTool = -1;
    if (!QueryInt("Get_ToolCNum", activeTool) || activeTool != m_toolNo)
    {
        SetLastRobotError("汇川当前激活工具号与ToolNo配置不一致；"
            "请在示教器切换工具，或配置ApiUserLevel=2及密码后由驱动设置。");
        return false;
    }
    if (m_wobjNo != 0 || m_apiUserLevel >= 2)
    {
        std::string response;
        int actual = -1;
        if (!SendCommand("Set_WobjNum " + std::to_string(m_wobjNo), response)
            || response != "ok" || !QueryInt("Get_WobjNum", actual) || actual != m_wobjNo)
        {
            SetLastRobotError("汇川激活工件号设置或回读失败；确认用户级别及WobjNo配置。");
            return false;
        }
    }
    int activeWobj = -1;
    if (!QueryInt("Get_WobjNum", activeWobj) || activeWobj != m_wobjNo)
    {
        SetLastRobotError("汇川当前激活工件号与WobjNo配置不一致；"
            "请在示教器切换工件，或配置ApiUserLevel=2及密码后由驱动设置。");
        return false;
    }

    if (summary != nullptr)
    {
        *summary = "汇川2222远程以太网连接和控制许可已确认；工具="
            + std::to_string(m_toolNo) + "，工件=" + std::to_string(m_wobjNo)
            + "。伺服与运行模式保持现场状态。";
    }
    ClearLastRobotError();
    return true;
}

bool InovanceRobotCtrl::ShutdownBeforeDisconnect()
{
    bool ok = true;
    std::vector<std::string> failures;
    const auto rememberFailure = [this, &failures](const char* fallback)
    {
        const std::string error = GetLastRobotError();
        failures.push_back(error.empty() ? std::string(fallback) : error);
    };
    if (!IsConnected()) { return true; }

    int motion = -1;
    int dataStreamMode = -1;
    if (QueryInt("Get_MotionSts", motion)
        && QueryInt("Get_DsMode", dataStreamMode)
        && (motion != 0 || dataStreamMode != 0))
    {
        if (!AbortCurrentProgramSafely())
        {
            rememberFailure("汇川断开前安全中止失败。");
            ok = false;
        }
    }

    std::string response;
    if (!SendCommand("Motor OFF", response) || response != "ok")
    {
        rememberFailure("汇川断开前Motor OFF失败。");
        ok = false;
    }
    else
    {
        int motor = -1;
        for (int attempt = 0; attempt < 20; ++attempt)
        {
            if (QueryInt("Get_MotorSts", motor) && motor == 0) { break; }
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
        }
        if (motor != 0)
        {
            SetLastRobotError("汇川断开前Motor OFF未通过状态回读确认。");
            rememberFailure("汇川断开前Motor OFF回读失败。");
            ok = false;
        }
    }
    if (m_userLoggedIn.load())
    {
        if (!SendCommand("UserLogout", response) || response != "ok")
        {
            rememberFailure("汇川断开前UserLogout失败。");
            ok = false;
        }
        m_userLoggedIn.store(false);
    }
    if (m_permitOwned.load())
    {
        if (!SendCommand("RemovePermit", response) || response != "ok")
        {
            rememberFailure("汇川断开前RemovePermit失败。");
            ok = false;
        }
        m_permitOwned.store(false);
    }
    if (!ok)
    {
        std::ostringstream error;
        for (std::size_t index = 0; index < failures.size(); ++index)
        {
            if (index > 0) { error << " | "; }
            error << failures[index];
        }
        SetLastRobotError(error.str().empty()
            ? "汇川断开前安全收尾未完整通过。" : error.str());
    }
    return ok;
}

void InovanceRobotCtrl::ReloadRuntimeConfiguration()
{
    InitRobotDriver(m_sRobotName);
}

bool InovanceRobotCtrl::SetOperationMode(RobotOperationMode mode)
{
    int rawMode = 0;
    if (mode == RobotOperationMode::Manual) { rawMode = 1; }
    else if (mode == RobotOperationMode::Automatic
        || mode == RobotOperationMode::ExternalAutomatic) { rawMode = 2; }
    else
    {
        SetLastRobotError("汇川远程以太网只支持Set_Mode 1(手动)或2(自动)，Start不是运行模式。");
        return false;
    }
    if (!EnsureControlPermit()) { return false; }
    std::string response;
    if (!SendCommand("Set_Mode " + std::to_string(rawMode), response) || response != "ok")
    {
        return false;
    }
    int actual = -1;
    if (!QueryInt("Get_Mode", actual) || actual != rawMode)
    {
        SetLastRobotError("汇川运行模式切换未通过Get_Mode回读确认。"
            "若返回e36，请先释放示教器按键控制权。");
        return false;
    }
    return true;
}

bool InovanceRobotCtrl::cleanAlarm()
{
    if (!EnsureControlPermit()) { return false; }
    std::string response;
    if (!SendCommand("ResetErr", response) || response != "ok") { return false; }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    int status = -1;
    if (!QueryInt("Get_SysErrSts", status) || status != 0)
    {
        SetLastRobotError("汇川报警复位后Get_SysErrSts仍非0，请在示教器查看具体故障。");
        return false;
    }
    return true;
}

bool InovanceRobotCtrl::ServoOn()
{
    if (!EnsureControlPermit()) { return false; }
    std::string response;
    if (!SendCommand("Motor ON", response) || response != "ok") { return false; }
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    int motor = -1;
    if (!QueryInt("Get_MotorSts", motor) || motor != 1)
    {
        SetLastRobotError("汇川Motor ON已发送，但300ms后Get_MotorSts未确认伺服使能。");
        return false;
    }
    return true;
}

bool InovanceRobotCtrl::SetTpSpeed(int speed)
{
    if (speed < 1 || speed > 100)
    {
        SetLastRobotError("汇川全局速度范围为1..100。");
        return false;
    }
    if (!EnsureControlPermit()) { return false; }
    std::string response;
    if (!SendCommand("Set_Vel " + std::to_string(speed), response) || response != "ok")
    {
        return false;
    }
    int actual = -1;
    if (!QueryInt("Get_Vel", actual) || actual != speed)
    {
        SetLastRobotError("汇川全局速度设置未通过Get_Vel回读确认。");
        return false;
    }
    return true;
}

bool InovanceRobotCtrl::IsConnected()
{
    return m_connected.load();
}

std::string InovanceRobotCtrl::GetRobotStatusText()
{
    if (!IsConnected())
    {
        const std::string error = GetLastRobotError();
        return error.empty() ? "汇川：未连接" : "汇川：" + error;
    }
    int emergencyStop = -1;
    int systemError = -1;
    int motor = -1;
    int mode = -1;
    int motion = -1;
    if (!QueryInt("Get_EStopSts", emergencyStop)
        || !QueryInt("Get_SysErrSts", systemError)
        || !QueryInt("Get_MotorSts", motor)
        || !QueryInt("Get_Mode", mode)
        || !QueryInt("Get_MotionSts", motion))
    {
        return "汇川：" + GetLastRobotError();
    }
    std::ostringstream text;
    text << "汇川：急停=" << emergencyStop
        << " 故障=" << systemError
        << " 伺服=" << motor
        << " 模式=" << mode
        << " 运动=" << motion;
    return text.str();
}

std::string InovanceRobotCtrl::GetStateMonitorSourceText() const
{
    return "汇川2222远程以太网轮询(Get_RobPHere/Get_PosHerePulse/Get_MotionSts)，时间轴为PC steady ms";
}

void InovanceRobotCtrl::StorePassivePose(const T_ROBOT_COORS& pose, long long pcRecvMs)
{
    std::lock_guard<std::mutex> lock(m_passiveMutex);
    m_passivePose = pose;
    m_passivePosePcMs = pcRecvMs;
    m_passivePoseValid = true;
}

void InovanceRobotCtrl::StorePassivePulse(const T_ANGLE_PULSE& pulse, long long pcRecvMs)
{
    std::lock_guard<std::mutex> lock(m_passiveMutex);
    m_passivePulse = pulse;
    m_passivePulsePcMs = pcRecvMs;
    m_passivePulseValid = true;
}

void InovanceRobotCtrl::StorePassiveMotion(const RobotMotionStatus& status, long long pcRecvMs)
{
    std::lock_guard<std::mutex> lock(m_passiveMutex);
    m_passiveMotion = status;
    m_passiveMotionPcMs = pcRecvMs;
    m_passiveMotionValid = true;
}

bool InovanceRobotCtrl::ReadCartesianPosition(T_ROBOT_COORS& pos, int armConfig[4])
{
    std::vector<double> values;
    if (!QueryDoubles("Get_RobPHere", values, 16)) { return false; }
    // 手册：ROB_POS = X,Y,Z,A,B,C; Arm[4]; E1..E6，且A/B/C语义为Rz/Ry/Rx。
    pos = T_ROBOT_COORS();
    pos.dX = values[0];
    pos.dY = values[1];
    pos.dZ = values[2];
    pos.dRZ = values[3];
    pos.dRY = values[4];
    pos.dRX = values[5];
    pos.dBX = values[10];
    pos.dBY = values[11];
    pos.dBZ = values[12];

    {
        std::lock_guard<std::mutex> lock(m_passiveMutex);
        for (int index = 0; index < 4; ++index)
        {
            m_armConfig[index] = static_cast<int>(std::llround(values[6 + index]));
            if (armConfig != nullptr) { armConfig[index] = m_armConfig[index]; }
        }
        for (int index = 0; index < 6; ++index)
        {
            m_externalValues[index] = values[10 + index];
        }
    }
    StorePassivePose(pos, SteadyMs());
    return true;
}

bool InovanceRobotCtrl::TryGetCurrentPos(T_ROBOT_COORS& pos)
{
    return ReadCartesianPosition(pos, nullptr);
}

T_ROBOT_COORS InovanceRobotCtrl::GetCurrentPos()
{
    T_ROBOT_COORS pos;
    TryGetCurrentPos(pos);
    return pos;
}

double InovanceRobotCtrl::GetCurrentPos(int axisNo)
{
    T_ROBOT_COORS pos;
    return TryGetCurrentPos(pos) ? PoseAt(pos, axisNo) : 0.0;
}

bool InovanceRobotCtrl::TryGetCurrentPulse(T_ANGLE_PULSE& pulse)
{
    std::vector<double> values;
    if (!QueryDoubles("Get_PosHerePulse", values, 6)) { return false; }
    pulse = T_ANGLE_PULSE(
        static_cast<long>(std::llround(values[0])),
        static_cast<long>(std::llround(values[1])),
        static_cast<long>(std::llround(values[2])),
        static_cast<long>(std::llround(values[3])),
        static_cast<long>(std::llround(values[4])),
        static_cast<long>(std::llround(values[5])),
        0, 0, 0);

    if (m_nExternalAxleType != 0)
    {
        std::vector<double> joints;
        if (!QueryDoubles("Get_RobJPHere", joints, 14)) { return false; }
        const double units[3] = {
            m_tAxisUnit.dBXPulseUnit,
            m_tAxisUnit.dBYPulseUnit,
            m_tAxisUnit.dBZPulseUnit
        };
        long* outputs[3] = { &pulse.lBXPulse, &pulse.lBYPulse, &pulse.lBZPulse };
        for (int index = 0; index < 3; ++index)
        {
            if ((m_nExternalAxleType & (1 << index)) == 0) { continue; }
            if (!std::isfinite(units[index]) || std::abs(units[index]) < 1e-15)
            {
                SetLastRobotError("汇川外部轴已启用，但ExternalAxle脉冲单位未配置，无法返回严格脉冲值。");
                return false;
            }
            *outputs[index] = static_cast<long>(std::llround(joints[8 + index] / units[index]));
        }
    }
    StorePassivePulse(pulse, SteadyMs());
    return true;
}

T_ANGLE_PULSE InovanceRobotCtrl::GetCurrentPulse()
{
    T_ANGLE_PULSE pulse;
    TryGetCurrentPulse(pulse);
    return pulse;
}

double InovanceRobotCtrl::GetCurrentPulse(int axisNo)
{
    T_ANGLE_PULSE pulse;
    return TryGetCurrentPulse(pulse) ? static_cast<double>(PulseAt(pulse, axisNo)) : 0.0;
}

T_ROBOT_COORS InovanceRobotCtrl::GetCurrentPosPassive(
    long long* robotMs, long long* pcRecvMs)
{
    T_ROBOT_COORS active;
    TryGetCurrentPos(active);
    std::lock_guard<std::mutex> lock(m_passiveMutex);
    if (robotMs != nullptr) { *robotMs = m_passivePosePcMs; }
    if (pcRecvMs != nullptr) { *pcRecvMs = m_passivePosePcMs; }
    return m_passivePoseValid ? m_passivePose : T_ROBOT_COORS();
}

T_ANGLE_PULSE InovanceRobotCtrl::GetCurrentPulsePassive(
    long long* robotMs, long long* pcRecvMs)
{
    T_ANGLE_PULSE active;
    TryGetCurrentPulse(active);
    std::lock_guard<std::mutex> lock(m_passiveMutex);
    if (robotMs != nullptr) { *robotMs = m_passivePulsePcMs; }
    if (pcRecvMs != nullptr) { *pcRecvMs = m_passivePulsePcMs; }
    return m_passivePulseValid ? m_passivePulse : T_ANGLE_PULSE();
}

RobotMotionStatus InovanceRobotCtrl::ReadMotionStatus()
{
    RobotMotionStatus status;
    int motion = -1;
    if (!QueryInt("Get_MotionSts", motion))
    {
        status.state = RobotMotionState::Unknown;
        status.detail = GetLastRobotError();
        StorePassiveMotion(status, SteadyMs());
        return status;
    }
    status.rawCode = motion;

    int dataStreamMode = 0;
    if (!QueryInt("Get_DsMode", dataStreamMode))
    {
        status.state = RobotMotionState::Unknown;
        status.detail = GetLastRobotError();
    }
    else if (dataStreamMode == 2)
    {
        status.state = RobotMotionState::Paused;
        status.detail = "汇川数据流已暂停";
    }
    else if (motion == 1)
    {
        status.state = RobotMotionState::Running;
        status.detail = "汇川机器人运动中";
    }
    else if (motion == 2)
    {
        status.state = RobotMotionState::Interrupted;
        status.detail = "汇川机器人运动中断";
    }
    else if (motion == 0)
    {
        int finalCommandId = -1;
        bool hasActive = false;
        {
            std::lock_guard<std::mutex> lock(m_trajectoryMutex);
            finalCommandId = m_finalCommandId;
            hasActive = m_activeHandle.started;
        }
        int commandDone = 0;
        if (hasActive && finalCommandId >= 0
            && QueryInt("Get_CmdSts " + std::to_string(finalCommandId), commandDone)
            && commandDone == 1)
        {
            status.state = RobotMotionState::Completed;
            status.terminalVerified = true;
            status.detail = "汇川末条数据流指令已精确到位";
        }
        else if (hasActive)
        {
            status.state = RobotMotionState::Starting;
            status.detail = "汇川数据流指令已受理，等待到位见证";
        }
        else
        {
            status.state = RobotMotionState::Idle;
            status.terminalVerified = true;
            status.detail = "汇川机器人已停止";
        }
    }
    else
    {
        status.state = RobotMotionState::Unknown;
        status.detail = "汇川返回未知运动状态=" + std::to_string(motion);
    }
    StorePassiveMotion(status, SteadyMs());
    return status;
}

RobotMotionStatus InovanceRobotCtrl::ReadMotionStatusPassive(
    long long* robotMs, long long* pcRecvMs)
{
    ReadMotionStatus();
    std::lock_guard<std::mutex> lock(m_passiveMutex);
    if (robotMs != nullptr) { *robotMs = m_passiveMotionPcMs; }
    if (pcRecvMs != nullptr) { *pcRecvMs = m_passiveMotionPcMs; }
    return m_passiveMotionValid ? m_passiveMotion : RobotMotionStatus{};
}

int InovanceRobotCtrl::CheckDone()
{
    const RobotMotionStatus status = ReadMotionStatus();
    return (status.state == RobotMotionState::Idle
        || status.state == RobotMotionState::Completed) ? 1 : 0;
}

int InovanceRobotCtrl::CheckDonePassive(long long* robotMs, long long* pcRecvMs)
{
    const RobotMotionStatus status = ReadMotionStatusPassive(robotMs, pcRecvMs);
    return (status.state == RobotMotionState::Idle
        || status.state == RobotMotionState::Completed) ? 1 : 0;
}

int InovanceRobotCtrl::CheckRobotDone(int delayMs, int runTimeoutMs)
{
    if (runTimeoutMs <= 0)
    {
        SetLastRobotError("汇川完成等待必须使用有限正超时。");
        return 0;
    }
    delayMs = std::clamp(delayMs, 20, 1000);
    const long long deadline = SteadyMs() + runTimeoutMs;
    while (SteadyMs() < deadline)
    {
        const RobotMotionStatus status = ReadMotionStatus();
        if ((status.state == RobotMotionState::Completed && status.terminalVerified)
            || status.state == RobotMotionState::Idle)
        {
            return 1;
        }
        if (status.state == RobotMotionState::Interrupted
            || status.state == RobotMotionState::Faulted
            || status.state == RobotMotionState::Unknown)
        {
            return 0;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(delayMs));
    }
    SetLastRobotError("汇川等待运动完成超时，未获得精确到位见证。");
    return 0;
}

bool InovanceRobotCtrl::ValidateLinearSpeedMmPerMin(
    double speedMmPerMin, std::string* error) const
{
    if (!std::isfinite(speedMmPerMin)
        || speedMmPerMin <= 0.0
        || speedMmPerMin > kMaxLinearSpeedMmPerMin)
    {
        if (error != nullptr)
        {
            *error = "汇川直线速度必须为有限正值且不超过120000 mm/min；"
                "驱动会转换为MovLRobP数值型TCP mm/s。";
        }
        return false;
    }
    if (error != nullptr) { error->clear(); }
    return true;
}

bool InovanceRobotCtrl::SendCartesianMove(
    const T_ROBOT_COORS& target,
    double speedMmPerMin,
    int zone,
    const int* configuration,
    int* commandId)
{
    std::string validationError;
    if (!ValidateLinearSpeedMmPerMin(speedMmPerMin, &validationError))
    {
        SetLastRobotError(validationError);
        return false;
    }
    const double components[9] = {
        target.dX, target.dY, target.dZ,
        target.dRX, target.dRY, target.dRZ,
        target.dBX, target.dBY, target.dBZ
    };
    if (!std::all_of(std::begin(components), std::end(components),
        [](double value) { return std::isfinite(value); }))
    {
        SetLastRobotError("汇川直线运动目标包含非有限坐标。");
        return false;
    }

    int arm[4] = {};
    double external[6] = {};
    {
        std::lock_guard<std::mutex> lock(m_passiveMutex);
        std::copy(std::begin(m_armConfig), std::end(m_armConfig), arm);
        std::copy(std::begin(m_externalValues), std::end(m_externalValues), external);
    }
    // 通用 configuration 没有汇川 ArmType 的品牌语义，只有驱动内部通过当前位置回读的
    // ArmType 可安全复用；位置寄存器接口另行显式接受4个Arm值。
    (void)configuration;
    if (m_nExternalAxleType & 1) { external[0] = target.dBX; }
    if (m_nExternalAxleType & 2) { external[1] = target.dBY; }
    if (m_nExternalAxleType & 4) { external[2] = target.dBZ; }

    const double speedMmPerSecond = speedMmPerMin / 60.0;
    std::ostringstream targetParameter;
    targetParameter << '['
        << FormatDouble(target.dX) << ',' << FormatDouble(target.dY) << ','
        << FormatDouble(target.dZ) << ','
        // 汇川原始顺序A,B,C对应通用RZ,RY,RX。
        << FormatDouble(target.dRZ) << ',' << FormatDouble(target.dRY) << ','
        << FormatDouble(target.dRX) << "; "
        << arm[0] << ',' << arm[1] << ',' << arm[2] << ',' << arm[3] << "; "
        << FormatDouble(external[0]) << ',' << FormatDouble(external[1]) << ','
        << FormatDouble(external[2]) << ',' << FormatDouble(external[3]) << ','
        << FormatDouble(external[4]) << ',' << FormatDouble(external[5]) << ']';
    if (targetParameter.str().size() > 128)
    {
        SetLastRobotError("汇川MovLRobP目标参数超过手册规定的128字符上限。");
        return false;
    }
    std::ostringstream command;
    command << "MovLRobP " << targetParameter.str() << ' '
        // type=1数值速度；百分比保留100；static=1表示不受全局百分比影响。
        << "1,100,1," << FormatDouble(speedMmPerSecond)
        << ",180.000000,2.000000,1.000000 "
        << std::clamp(zone, -2, 200) << " 0";

    int before = -1;
    QueryInt("Get_CurCmdNum", before);
    std::string response;
    if (!SendCommand(command.str(), response) || response != "ok") { return false; }

    int after = before;
    for (int attempt = 0; attempt < 40; ++attempt)
    {
        if (QueryInt("Get_CurCmdNum", after) && after != before) { break; }
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
    if (after < 0 || after == before)
    {
        SetLastRobotError("汇川MovLRobP返回ok，但Get_CurCmdNum未产生新的指令编号，拒绝伪造完成身份。");
        return false;
    }
    if (commandId != nullptr) { *commandId = after; }
    return true;
}

bool InovanceRobotCtrl::SendJointMove(
    const T_ANGLE_PULSE& target,
    double speedPercent,
    int zone,
    int* commandId)
{
    if (!std::isfinite(speedPercent) || speedPercent < 1.0 || speedPercent > 100.0)
    {
        SetLastRobotError("汇川关节速度必须为1..100百分比。");
        return false;
    }
    const double units[9] = {
        m_tAxisUnit.dSPulseUnit, m_tAxisUnit.dLPulseUnit, m_tAxisUnit.dUPulseUnit,
        m_tAxisUnit.dRPulseUnit, m_tAxisUnit.dBPulseUnit, m_tAxisUnit.dTPulseUnit,
        m_tAxisUnit.dBXPulseUnit, m_tAxisUnit.dBYPulseUnit, m_tAxisUnit.dBZPulseUnit
    };
    for (int axis = 0; axis < 6; ++axis)
    {
        if (!std::isfinite(units[axis]) || std::abs(units[axis]) < 1e-15)
        {
            SetLastRobotError("汇川关节运动缺少AxisUnit脉冲到角度换算参数，禁止发送错误关节角。");
            return false;
        }
    }

    double external[6] = {};
    {
        std::lock_guard<std::mutex> lock(m_passiveMutex);
        std::copy(std::begin(m_externalValues), std::end(m_externalValues), external);
    }
    for (int axis = 0; axis < 3; ++axis)
    {
        if ((m_nExternalAxleType & (1 << axis)) == 0) { continue; }
        if (!std::isfinite(units[6 + axis]) || std::abs(units[6 + axis]) < 1e-15)
        {
            SetLastRobotError("汇川外部轴运动缺少ExternalAxle脉冲单位配置。");
            return false;
        }
        external[axis] = static_cast<double>(PulseAt(target, 6 + axis)) * units[6 + axis];
    }

    std::ostringstream targetParameter;
    targetParameter << '[';
    for (int axis = 0; axis < 6; ++axis)
    {
        if (axis > 0) { targetParameter << ','; }
        targetParameter << FormatDouble(static_cast<double>(PulseAt(target, axis)) * units[axis]);
    }
    targetParameter << ",0.000,0.000; ";
    for (int axis = 0; axis < 6; ++axis)
    {
        if (axis > 0) { targetParameter << ','; }
        targetParameter << FormatDouble(external[axis]);
    }
    targetParameter << ']';
    if (targetParameter.str().size() > 128)
    {
        SetLastRobotError("汇川MovJAbsRobJP目标参数超过手册规定的128字符上限。");
        return false;
    }
    std::ostringstream command;
    command << "MovJAbsRobJP " << targetParameter.str()
        << " 0," << std::clamp(static_cast<int>(std::lround(speedPercent)), 1, 100)
        << ",0,0.000000,0.000000,2.000000,1.000000 "
        << std::clamp(zone, -2, 200) << " 0";

    int before = -1;
    QueryInt("Get_CurCmdNum", before);
    std::string response;
    if (!SendCommand(command.str(), response) || response != "ok") { return false; }
    int after = before;
    for (int attempt = 0; attempt < 40; ++attempt)
    {
        if (QueryInt("Get_CurCmdNum", after) && after != before) { break; }
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
    if (after < 0 || after == before)
    {
        SetLastRobotError("汇川MovJAbsRobJP返回ok，但Get_CurCmdNum未产生新的指令编号。");
        return false;
    }
    if (commandId != nullptr) { *commandId = after; }
    return true;
}

bool InovanceRobotCtrl::WaitForCommandDone(int commandId, int pollDelayMs, int timeoutMs)
{
    if (commandId < 0 || timeoutMs <= 0)
    {
        SetLastRobotError("汇川运动完成见证参数无效。");
        return false;
    }
    pollDelayMs = std::clamp(pollDelayMs, 20, 1000);
    const long long deadline = SteadyMs() + timeoutMs;
    int stableDone = 0;
    while (SteadyMs() < deadline)
    {
        int done = 0;
        if (!QueryInt("Get_CmdSts " + std::to_string(commandId), done)) { return false; }
        int motion = -1;
        if (!QueryInt("Get_MotionSts", motion)) { return false; }
        if (motion == 2)
        {
            SetLastRobotError("汇川运动在等待到位期间被中断。");
            return false;
        }
        stableDone = (done == 1 && motion == 0) ? stableDone + 1 : 0;
        if (stableDone >= 2) { return true; }
        std::this_thread::sleep_for(std::chrono::milliseconds(pollDelayMs));
    }
    SetLastRobotError("汇川运动等待Get_CmdSts精确到位超时。");
    return false;
}

bool InovanceRobotCtrl::MoveLinearMmPerMin(
    const T_ROBOT_COORS& target,
    double speedMmPerMin,
    int externalAxleType,
    const int* configuration)
{
    if (externalAxleType != m_nExternalAxleType)
    {
        SetLastRobotError("汇川直线运动外部轴类型与当前驱动配置不一致。");
        return false;
    }
    if (!EnsureMotionReady() || !SetDataStreamMode("ON", 1)) { return false; }
    // 首次运动前必须取得真实ArmType和未暴露的E4..E6，防止以默认值覆盖控制器位形。
    T_ROBOT_COORS current;
    if (!ReadCartesianPosition(current, nullptr))
    {
        SetDataStreamMode("OFF", 0);
        return false;
    }
    int commandId = -1;
    const bool sent = SendCartesianMove(target, speedMmPerMin, -1, configuration, &commandId);
    const bool completed = sent && WaitForCommandDone(commandId, 25, 1800000);
    const std::string motionError = completed ? std::string() : GetLastRobotError();
    const bool stopped = SetDataStreamMode("OFF", 0);
    if (!completed && !motionError.empty()) { SetLastRobotError(motionError); }
    return completed && stopped;
}

bool InovanceRobotCtrl::MoveJointPercent(
    const T_ANGLE_PULSE& target,
    double speedPercent,
    int externalAxleType)
{
    if (externalAxleType != m_nExternalAxleType)
    {
        SetLastRobotError("汇川关节运动外部轴类型与当前驱动配置不一致。");
        return false;
    }
    if (!EnsureMotionReady() || !SetDataStreamMode("ON", 1)) { return false; }
    T_ROBOT_COORS current;
    if (!ReadCartesianPosition(current, nullptr))
    {
        SetDataStreamMode("OFF", 0);
        return false;
    }
    int commandId = -1;
    const bool sent = SendJointMove(target, speedPercent, -1, &commandId);
    const bool completed = sent && WaitForCommandDone(commandId, 25, 1800000);
    const std::string motionError = completed ? std::string() : GetLastRobotError();
    const bool stopped = SetDataStreamMode("OFF", 0);
    if (!completed && !motionError.empty()) { SetLastRobotError(motionError); }
    return completed && stopped;
}

std::uint64_t InovanceRobotCtrl::FingerprintMoveInfos(
    const std::vector<T_ROBOT_MOVE_INFO>& moveInfos,
    RobotTrajectoryPurpose purpose) const
{
    std::uint64_t hash = 1469598103934665603ULL;
    const auto mixBytes = [&hash](const void* data, std::size_t size)
    {
        const auto* bytes = static_cast<const unsigned char*>(data);
        for (std::size_t index = 0; index < size; ++index)
        {
            hash ^= bytes[index];
            hash *= 1099511628211ULL;
        }
    };
    const int purposeValue = static_cast<int>(purpose);
    mixBytes(&purposeValue, sizeof(purposeValue));
    const std::uint64_t count = static_cast<std::uint64_t>(moveInfos.size());
    mixBytes(&count, sizeof(count));
    for (const T_ROBOT_MOVE_INFO& move : moveInfos)
    {
        const double values[] = {
            move.tCoord.dX, move.tCoord.dY, move.tCoord.dZ,
            move.tCoord.dRX, move.tCoord.dRY, move.tCoord.dRZ,
            move.tCoord.dBX, move.tCoord.dBY, move.tCoord.dBZ,
            move.tSpeed.dSpeed, move.dOverlapRel, move.dWeldSpeedMmPerMin
        };
        mixBytes(values, sizeof(values));
        const long pulses[] = {
            move.tPulse.nSPulse, move.tPulse.nLPulse, move.tPulse.nUPulse,
            move.tPulse.nRPulse, move.tPulse.nBPulse, move.tPulse.nTPulse,
            move.tPulse.lBXPulse, move.tPulse.lBYPulse, move.tPulse.lBZPulse
        };
        mixBytes(pulses, sizeof(pulses));
        const int integers[] = { move.nMoveType, move.nPosType, move.nDwellMs };
        mixBytes(integers, sizeof(integers));
        const bool flags[] = {
            move.bWeldProcessEnabled,
            move.bArcStartBeforeMove,
            move.bArcEndAfterMove,
            move.bHasWeaveParam,
            move.bHasTrackParam
        };
        mixBytes(flags, sizeof(flags));
    }
    return hash;
}

bool InovanceRobotCtrl::ValidateMoveInfos(
    const std::vector<T_ROBOT_MOVE_INFO>& moveInfos,
    RobotTrajectoryPurpose purpose,
    std::string& error) const
{
    if (moveInfos.empty())
    {
        error = "汇川轨迹为空。";
        return false;
    }
    if (moveInfos.size() > 100000)
    {
        error = "汇川轨迹点数超过100000安全上限。";
        return false;
    }
    if (purpose == RobotTrajectoryPurpose::ActualWeld)
    {
        error = "汇川手册证明了数据流运动和运动IO，但当前未配置焊机DO/模拟量映射、"
            "起弧成功反馈和灭弧终态，因此真实焊接保持关闭。";
        return false;
    }
    for (std::size_t index = 0; index < moveInfos.size(); ++index)
    {
        const T_ROBOT_MOVE_INFO& move = moveInfos[index];
        if (move.bWeldProcessEnabled || move.bArcStartBeforeMove || move.bArcEndAfterMove)
        {
            error = "汇川第" + std::to_string(index + 1)
                + "点包含焊接/起弧标志，但驱动尚无现场焊机反馈契约。";
            return false;
        }
        if (move.nDwellMs > 0)
        {
            error = "汇川数据流适配暂未把轨迹停留转换为经验证的控制器等待指令。";
            return false;
        }
        if (move.nMoveType == MOVL)
        {
            const double speed = move.dWeldSpeedMmPerMin > 0.0
                ? move.dWeldSpeedMmPerMin : move.tSpeed.dSpeed;
            if (!ValidateLinearSpeedMmPerMin(speed, &error))
            {
                error = "汇川第" + std::to_string(index + 1) + "个直线点：" + error;
                return false;
            }
        }
        else if (move.nMoveType == MOVJ)
        {
            if (!std::isfinite(move.tSpeed.dSpeed)
                || move.tSpeed.dSpeed < 1.0 || move.tSpeed.dSpeed > 100.0)
            {
                error = "汇川第" + std::to_string(index + 1)
                    + "个关节点速度必须为1..100百分比。";
                return false;
            }
            const double units[6] = {
                m_tAxisUnit.dSPulseUnit, m_tAxisUnit.dLPulseUnit, m_tAxisUnit.dUPulseUnit,
                m_tAxisUnit.dRPulseUnit, m_tAxisUnit.dBPulseUnit, m_tAxisUnit.dTPulseUnit
            };
            if (!std::all_of(std::begin(units), std::end(units),
                [](double unit) { return std::isfinite(unit) && std::abs(unit) >= 1e-15; }))
            {
                error = "汇川轨迹包含关节点，但当前机器人未配置真实AxisUnit；JointMotion能力未开放。";
                return false;
            }
        }
        else
        {
            error = "汇川轨迹只支持MOVL和MOVJ。";
            return false;
        }
    }
    error.clear();
    return true;
}

bool InovanceRobotCtrl::ReserveTrajectory(
    RobotTrajectoryPurpose purpose,
    RobotTrajectoryHandle& handle)
{
    if (purpose == RobotTrajectoryPurpose::ActualWeld)
    {
        SetLastRobotError("汇川真实焊接能力未声明：需先完成焊机IO映射、起弧/灭弧反馈和现场验证。");
        return false;
    }
    std::lock_guard<std::mutex> lock(m_trajectoryMutex);
    if (m_trajectoryRunning.load())
    {
        SetLastRobotError("汇川上一条数据流轨迹仍在运行，禁止覆盖轨迹身份。");
        return false;
    }
    handle = RobotTrajectoryHandle{};
    handle.programName = "INOVANCE_STREAM_" + std::to_string(++m_trajectoryCounter);
    m_preparedMoveInfos.clear();
    m_preparedPurpose = purpose;
    m_preparedFingerprint = 0;
    m_activeHandle = RobotTrajectoryHandle{};
    m_finalCommandId = -1;
    return true;
}

bool InovanceRobotCtrl::DownlinkTrajectory(
    const std::vector<T_ROBOT_MOVE_INFO>& moveInfos,
    RobotTrajectoryPurpose purpose,
    RobotTrajectoryHandle& handle)
{
    std::string error;
    if (!ValidateMoveInfos(moveInfos, purpose, error))
    {
        SetLastRobotError(error);
        return false;
    }
    if (handle.programName.empty() && !ReserveTrajectory(purpose, handle))
    {
        return false;
    }
    std::lock_guard<std::mutex> lock(m_trajectoryMutex);
    if (m_trajectoryRunning.load())
    {
        SetLastRobotError("汇川轨迹下传时上一轨迹仍在运行。");
        return false;
    }
    m_preparedMoveInfos = moveInfos;
    m_preparedPurpose = purpose;
    m_preparedFingerprint = FingerprintMoveInfos(moveInfos, purpose);
    handle.prepared = true;
    handle.started = false;
    return true;
}

bool InovanceRobotCtrl::ExportTrajectoryProgramFiles(
    const std::vector<T_ROBOT_MOVE_INFO>& moveInfos,
    RobotTrajectoryPurpose purpose,
    const std::string& outputDirectory,
    RobotTrajectoryHandle& handle,
    std::string* error)
{
    (void)moveInfos;
    (void)purpose;
    (void)outputDirectory;
    (void)handle;
    const std::string message = "汇川当前通过2222数据流直接下发轨迹，手册未定义可离线上传的原生程序文件格式；离线导出保持关闭。";
    SetLastRobotError(message);
    if (error != nullptr) { *error = message; }
    return false;
}

bool InovanceRobotCtrl::StartTrajectory(
    const std::vector<T_ROBOT_MOVE_INFO>& moveInfos,
    RobotTrajectoryPurpose purpose,
    RobotTrajectoryHandle& handle)
{
    std::string error;
    if (!ValidateMoveInfos(moveInfos, purpose, error))
    {
        SetLastRobotError(error);
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(m_trajectoryMutex);
        if (!handle.prepared
            || m_preparedMoveInfos.empty()
            || purpose != m_preparedPurpose
            || FingerprintMoveInfos(moveInfos, purpose) != m_preparedFingerprint)
        {
            SetLastRobotError("汇川StartTrajectory输入与DownlinkTrajectory冻结的轨迹身份不一致。");
            return false;
        }
        if (m_trajectoryRunning.load())
        {
            SetLastRobotError("汇川已有数据流轨迹在运行。");
            return false;
        }
    }

    if (!EnsureMotionReady() || !SetDataStreamMode("ON", 1)) { return false; }
    T_ROBOT_COORS current;
    if (!ReadCartesianPosition(current, nullptr))
    {
        SetDataStreamMode("OFF", 0);
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(m_trajectoryMutex);
        handle.started = true;
        m_activeHandle = handle;
        m_finalCommandId = -1;
        m_trajectoryRunning.store(true);
        m_trajectoryPaused.store(false);
    }

    bool ok = true;
    int lastCommandId = -1;
    for (std::size_t index = 0; index < moveInfos.size(); ++index)
    {
        int cacheCount = m_maxBufferedCommands;
        const long long cacheDeadline = SteadyMs() + 10000;
        while (SteadyMs() < cacheDeadline)
        {
            if (QueryInt("Get_CurCmdCacheNum", cacheCount)
                && cacheCount < m_maxBufferedCommands)
            {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        if (cacheCount >= m_maxBufferedCommands)
        {
            SetLastRobotError("汇川运动缓存持续达到配置上限，停止继续下发轨迹。");
            ok = false;
            break;
        }

        const T_ROBOT_MOVE_INFO& move = moveInfos[index];
        const int zone = move.dOverlapRel <= 0.0
            ? -1 : std::clamp(static_cast<int>(std::lround(move.dOverlapRel)), 0, 200);
        if (move.nMoveType == MOVL)
        {
            const double speed = move.dWeldSpeedMmPerMin > 0.0
                ? move.dWeldSpeedMmPerMin : move.tSpeed.dSpeed;
            ok = SendCartesianMove(move.tCoord, speed, zone, nullptr, &lastCommandId);
        }
        else
        {
            ok = SendJointMove(move.tPulse, move.tSpeed.dSpeed, zone, &lastCommandId);
        }
        if (!ok) { break; }
        {
            std::lock_guard<std::mutex> lock(m_trajectoryMutex);
            m_finalCommandId = lastCommandId;
        }
    }

    if (!ok)
    {
        const std::string startError = GetLastRobotError();
        AbortCurrentProgramSafely();
        if (!startError.empty()) { SetLastRobotError(startError); }
        return false;
    }
    handle.started = true;
    ClearLastRobotError();
    return true;
}

bool InovanceRobotCtrl::WaitTrajectory(
    const RobotTrajectoryHandle& handle,
    int pollDelayMs,
    int runTimeoutMs,
    RobotMotionStatus* terminalStatus)
{
    if (terminalStatus != nullptr) { *terminalStatus = RobotMotionStatus{}; }
    int finalCommandId = -1;
    {
        std::lock_guard<std::mutex> lock(m_trajectoryMutex);
        if (!SameProgramHandle(handle, m_activeHandle)
            || !m_activeHandle.started || m_finalCommandId < 0)
        {
            SetLastRobotError("汇川WaitTrajectory句柄与当前数据流轨迹身份不一致。");
            if (terminalStatus != nullptr)
            {
                terminalStatus->state = RobotMotionState::Faulted;
                terminalStatus->detail = GetLastRobotError();
            }
            return false;
        }
        finalCommandId = m_finalCommandId;
    }
    const bool completed = WaitForCommandDone(finalCommandId, pollDelayMs, runTimeoutMs);
    if (!completed)
    {
        const std::string completionError = GetLastRobotError();
        AbortCurrentProgramSafely();
        if (!completionError.empty()) { SetLastRobotError(completionError); }
        if (terminalStatus != nullptr)
        {
            terminalStatus->state = RobotMotionState::Faulted;
            terminalStatus->detail = GetLastRobotError();
        }
        return false;
    }
    if (!SetDataStreamMode("OFF", 0))
    {
        if (terminalStatus != nullptr)
        {
            terminalStatus->state = RobotMotionState::Faulted;
            terminalStatus->detail = GetLastRobotError();
        }
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(m_trajectoryMutex);
        m_trajectoryRunning.store(false);
        m_trajectoryPaused.store(false);
        m_activeHandle.started = false;
    }
    if (terminalStatus != nullptr)
    {
        terminalStatus->state = RobotMotionState::Completed;
        terminalStatus->rawCode = 0;
        terminalStatus->terminalVerified = true;
        terminalStatus->detail = "汇川Get_CmdSts和Get_MotionSts连续回读确认轨迹完成";
    }
    ClearLastRobotError();
    return true;
}

bool InovanceRobotCtrl::GetTrackedMotionIdentity(
    std::string& projectName,
    std::string& programName,
    bool* alreadyStopped)
{
    std::lock_guard<std::mutex> lock(m_trajectoryMutex);
    if (m_activeHandle.programName.empty())
    {
        SetLastRobotError("汇川当前没有由本适配层启动的数据流轨迹身份。");
        return false;
    }
    projectName = "DATA_STREAM";
    programName = m_activeHandle.programName;
    if (alreadyStopped != nullptr) { *alreadyStopped = !m_trajectoryRunning.load(); }
    return true;
}

bool InovanceRobotCtrl::PauseTrackedMotion(
    const std::string& expectedProgramName,
    int& programLine,
    T_ROBOT_COORS& pausedPose,
    std::string* projectName,
    std::string* programName)
{
    {
        std::lock_guard<std::mutex> lock(m_trajectoryMutex);
        if (!m_trajectoryRunning.load()
            || m_activeHandle.programName != expectedProgramName)
        {
            SetLastRobotError("汇川暂停被拒绝：期望程序身份与当前数据流轨迹不一致。");
            return false;
        }
    }
    if (!SetDataStreamMode("PAUSE", 2)) { return false; }
    int motion = -1;
    int stableStopped = 0;
    for (int attempt = 0; attempt < 80; ++attempt)
    {
        if (!QueryInt("Get_MotionSts", motion)) { return false; }
        stableStopped = motion != 1 ? stableStopped + 1 : 0;
        if (stableStopped >= 3) { break; }
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
    if (stableStopped < 3)
    {
        SetLastRobotError("汇川Dsmode PAUSE后运动状态未稳定停止。");
        return false;
    }
    T_ROBOT_COORS first;
    T_ROBOT_COORS second;
    if (!TryGetCurrentPos(first)) { return false; }
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    if (!TryGetCurrentPos(second)) { return false; }
    const double positionDeviation = std::sqrt(
        std::pow(first.dX - second.dX, 2.0)
        + std::pow(first.dY - second.dY, 2.0)
        + std::pow(first.dZ - second.dZ, 2.0));
    const double angleDeviation = std::max({
        std::abs(NormalizeAngleDifference(first.dRX, second.dRX)),
        std::abs(NormalizeAngleDifference(first.dRY, second.dRY)),
        std::abs(NormalizeAngleDifference(first.dRZ, second.dRZ))
    });
    if (positionDeviation > 0.2 || angleDeviation > 0.2)
    {
        SetLastRobotError("汇川暂停后的两次位姿回读不稳定，拒绝生成续传检查点。");
        return false;
    }
    int currentCommand = -1;
    if (!QueryInt("Get_CurCmdNum", currentCommand)) { return false; }
    programLine = currentCommand;
    pausedPose = second;
    if (projectName != nullptr) { *projectName = "DATA_STREAM"; }
    if (programName != nullptr) { *programName = expectedProgramName; }
    m_trajectoryPaused.store(true);
    return true;
}

bool InovanceRobotCtrl::ResumeTrackedMotion(
    const std::string& expectedProgramName,
    const T_ROBOT_COORS& checkpointPose,
    double maxPositionDeviationMm,
    double maxAngleDeviationDeg,
    double* positionDeviationMm,
    double* angleDeviationDeg)
{
    if (!std::isfinite(maxPositionDeviationMm) || maxPositionDeviationMm < 0.0
        || !std::isfinite(maxAngleDeviationDeg) || maxAngleDeviationDeg < 0.0)
    {
        SetLastRobotError("汇川恢复运动的位姿偏差阈值无效。");
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(m_trajectoryMutex);
        if (!m_trajectoryRunning.load() || !m_trajectoryPaused.load()
            || m_activeHandle.programName != expectedProgramName)
        {
            SetLastRobotError("汇川恢复被拒绝：轨迹身份或暂停状态不一致。");
            return false;
        }
    }
    int mode = -1;
    if (!QueryInt("Get_DsMode", mode) || mode != 2)
    {
        SetLastRobotError("汇川恢复前Get_DsMode未确认PAUSE状态。");
        return false;
    }
    T_ROBOT_COORS current;
    if (!TryGetCurrentPos(current)) { return false; }
    const double position = std::sqrt(
        std::pow(current.dX - checkpointPose.dX, 2.0)
        + std::pow(current.dY - checkpointPose.dY, 2.0)
        + std::pow(current.dZ - checkpointPose.dZ, 2.0));
    const double angle = std::max({
        std::abs(NormalizeAngleDifference(current.dRX, checkpointPose.dRX)),
        std::abs(NormalizeAngleDifference(current.dRY, checkpointPose.dRY)),
        std::abs(NormalizeAngleDifference(current.dRZ, checkpointPose.dRZ))
    });
    if (positionDeviationMm != nullptr) { *positionDeviationMm = position; }
    if (angleDeviationDeg != nullptr) { *angleDeviationDeg = angle; }
    if (position > maxPositionDeviationMm || angle > maxAngleDeviationDeg)
    {
        SetLastRobotError("汇川恢复检查点偏差超限：位置=" + FormatDouble(position)
            + "mm，姿态=" + FormatDouble(angle) + "deg。");
        return false;
    }
    if (!SetDataStreamMode("CONTINUE", 1)) { return false; }
    m_trajectoryPaused.store(false);
    return true;
}

RobotPersistentRecoveryStrategy InovanceRobotCtrl::PersistentRecoveryStrategy() const
{
    return RobotPersistentRecoveryStrategy::Unsupported;
}

bool InovanceRobotCtrl::AbortPersistedMotion(const std::string& expectedProgramName)
{
    (void)expectedProgramName;
    SetLastRobotError("汇川数据流指令编号和轨迹身份不跨上位机重启持久化；"
        "当前只支持本连接会话内的安全中止，不能声明断电/重启后精确恢复。请在示教器确认机器人静止后重新开始流程。");
    return false;
}

bool InovanceRobotCtrl::AbortCurrentProgramSafely()
{
    if (!IsConnected())
    {
        SetLastRobotError("汇川安全中止失败：机器人未连接，无法取得停止见证。");
        return false;
    }
    int dataStreamMode = -1;
    int motion = -1;
    if (!QueryInt("Get_DsMode", dataStreamMode)
        || !QueryInt("Get_MotionSts", motion))
    {
        return false;
    }
    if (dataStreamMode == 0 && motion == 1)
    {
        SetLastRobotError("汇川检测到非本驱动数据流的机器人运动；禁止用Dsmode伪装安全中止，需由原生工程或示教器停止。");
        return false;
    }
    if (dataStreamMode == 1)
    {
        if (!SetDataStreamMode("PAUSE", 2)) { return false; }
    }
    int stableStopped = 0;
    for (int attempt = 0; attempt < 120; ++attempt)
    {
        if (!QueryInt("Get_MotionSts", motion)) { return false; }
        stableStopped = motion != 1 ? stableStopped + 1 : 0;
        if (stableStopped >= 3) { break; }
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
    if (stableStopped < 3)
    {
        SetLastRobotError("汇川安全中止未获得连续三次非运动状态见证。");
        return false;
    }
    if (dataStreamMode != 0 && !SetDataStreamMode("OFF", 0)) { return false; }
    stableStopped = 0;
    for (int attempt = 0; attempt < 20; ++attempt)
    {
        int mode = -1;
        if (!QueryInt("Get_DsMode", mode) || !QueryInt("Get_MotionSts", motion))
        {
            return false;
        }
        stableStopped = (mode == 0 && motion == 0) ? stableStopped + 1 : 0;
        if (stableStopped >= 3) { break; }
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
    if (stableStopped < 3)
    {
        SetLastRobotError("汇川数据流关闭后未获得Get_DsMode=0且Get_MotionSts=0的稳定终态。");
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(m_trajectoryMutex);
        m_trajectoryRunning.store(false);
        m_trajectoryPaused.store(false);
        m_activeHandle.started = false;
    }
    m_continuousJogRunning.store(false);
    ClearLastRobotError();
    return true;
}

bool InovanceRobotCtrl::StartContinuousJog(int moveType, double canonicalSpeed)
{
    if (moveType != MOVL && moveType != MOVJ)
    {
        SetLastRobotError("汇川连续点动只支持MOVL或MOVJ。");
        return false;
    }
    if (moveType == MOVL)
    {
        std::string error;
        if (!ValidateLinearSpeedMmPerMin(canonicalSpeed, &error))
        {
            SetLastRobotError(error);
            return false;
        }
    }
    else if (!std::isfinite(canonicalSpeed) || canonicalSpeed < 1.0 || canonicalSpeed > 100.0)
    {
        SetLastRobotError("汇川连续关节点动速度必须为1..100百分比。");
        return false;
    }
    bool expected = false;
    if (!m_continuousJogRunning.compare_exchange_strong(expected, true))
    {
        SetLastRobotError("汇川连续点动已经运行。");
        return false;
    }
    if (!EnsureMotionReady() || !SetDataStreamMode("ON", 1))
    {
        m_continuousJogRunning.store(false);
        return false;
    }
    T_ROBOT_COORS current;
    if (!ReadCartesianPosition(current, nullptr))
    {
        m_continuousJogRunning.store(false);
        SetDataStreamMode("OFF", 0);
        return false;
    }
    m_continuousJogMoveType = moveType;
    m_continuousJogSpeed = canonicalSpeed;
    m_continuousJogStopRequested.store(false);
    return true;
}

bool InovanceRobotCtrl::PushContinuousJogPoint(
    const T_ROBOT_COORS& target, double speedMmPerMin)
{
    if (!m_continuousJogRunning.load() || m_continuousJogStopRequested.load()
        || m_continuousJogMoveType != MOVL)
    {
        SetLastRobotError("汇川直角连续点动未启动或已请求结束。");
        return false;
    }
    int commandId = -1;
    if (!SendCartesianMove(target, speedMmPerMin, 0, nullptr, &commandId)) { return false; }
    std::lock_guard<std::mutex> lock(m_trajectoryMutex);
    m_finalCommandId = commandId;
    return true;
}

bool InovanceRobotCtrl::PushContinuousJogPoint(
    const T_ANGLE_PULSE& target, double speedPercent)
{
    if (!m_continuousJogRunning.load() || m_continuousJogStopRequested.load()
        || m_continuousJogMoveType != MOVJ)
    {
        SetLastRobotError("汇川关节连续点动未启动或已请求结束。");
        return false;
    }
    int commandId = -1;
    if (!SendJointMove(target, speedPercent, 0, &commandId)) { return false; }
    std::lock_guard<std::mutex> lock(m_trajectoryMutex);
    m_finalCommandId = commandId;
    return true;
}

void InovanceRobotCtrl::RequestEndContinuousJog()
{
    m_continuousJogStopRequested.store(true);
}

void InovanceRobotCtrl::EndContinuousJog()
{
    if (!m_continuousJogRunning.exchange(false)) { return; }
    m_continuousJogStopRequested.store(true);
    int finalCommandId = -1;
    {
        std::lock_guard<std::mutex> lock(m_trajectoryMutex);
        finalCommandId = m_finalCommandId;
    }
    if (finalCommandId >= 0)
    {
        WaitForCommandDone(finalCommandId, 25, 10000);
    }
    if (IsConnected())
    {
        SetDataStreamMode("OFF", 0);
    }
}

bool InovanceRobotCtrl::IsContinuousJogRunning() const
{
    return m_continuousJogRunning.load();
}

bool InovanceRobotCtrl::PrepareNativeProgramUpload()
{
    std::string error;
    const std::shared_ptr<RobotFileTransferSession> session =
        CreateFileTransferSession(&error);
    if (session == nullptr)
    {
        SetLastRobotError("汇川原生程序上传准备失败：" + error);
        return false;
    }
    ClearLastRobotError();
    return true;
}

int InovanceRobotCtrl::UploadNativeProgramSource(
    const std::string& localPath,
    const std::string& remoteDirectory)
{
    const std::filesystem::path localFile(localPath);
    std::error_code fileError;
    if (localPath.empty() || !std::filesystem::is_regular_file(localFile, fileError))
    {
        SetLastRobotError("汇川原生程序上传失败：本地文件不存在或不可读：" + localPath);
        return -1;
    }

    std::string extension = localFile.extension().string();
    std::transform(extension.begin(), extension.end(), extension.begin(),
        [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    if (extension != ".pro" && extension != ".prj"
        && extension != ".pts" && extension != ".jsn")
    {
        SetLastRobotError("汇川原生程序上传仅支持PRO、PRJ、PTS或JSN文件：" + localPath);
        return -1;
    }

    std::string sessionError;
    const std::shared_ptr<RobotFileTransferSession> session =
        CreateFileTransferSession(&sessionError);
    if (session == nullptr)
    {
        SetLastRobotError("汇川原生程序上传失败：" + sessionError);
        return -1;
    }

    std::string resolvedDirectory;
    std::string directoryError;
    if (remoteDirectory.empty())
    {
        if (!IsConnected())
        {
            SetLastRobotError("汇川原生程序上传失败：未连接2222控制通道，"
                "无法通过 Get_TaskPrgPath 0 确认当前工程目录；也可以显式指定远端目录。");
            return -1;
        }
        int taskStatus = -1;
        if (!QueryInt("Get_TaskRunSts 0", taskStatus))
        {
            return -1;
        }
        if (taskStatus == 1)
        {
            SetLastRobotError("汇川原生程序上传已拒绝：主任务正在运行，请停止后再覆盖工程文件。");
            return -1;
        }
        std::string taskPathResponse;
        if (!SendCommand("Get_TaskPrgPath 0", taskPathResponse)
            || !InovanceActiveProjectDirectory(
                ValuePart(taskPathResponse), resolvedDirectory, directoryError))
        {
            if (!directoryError.empty())
            {
                SetLastRobotError("汇川原生程序上传失败：" + directoryError);
            }
            return -1;
        }
    }
    else
    {
        if (!NormalizeInovanceRemotePath(
            remoteDirectory, resolvedDirectory, directoryError))
        {
            SetLastRobotError("汇川原生程序上传失败：" + directoryError);
            return -1;
        }
        if (IsConnected())
        {
            int taskStatus = -1;
            if (!QueryInt("Get_TaskRunSts 0", taskStatus))
            {
                return -1;
            }
            if (taskStatus == 1)
            {
                SetLastRobotError("汇川原生程序上传已拒绝：主任务正在运行，请停止后再覆盖工程文件。");
                return -1;
            }
        }
    }

    const std::string remotePath = resolvedDirectory + "/"
        + localFile.filename().string();
    if (!session->UploadProgramFile(localPath, remotePath, true))
    {
        SetLastRobotError("汇川原生程序上传失败：" + session->LastError()
            + "，Remote=" + remotePath);
        return -1;
    }
    if (m_pRobotLog != nullptr)
    {
        m_pRobotLog->write(LogColor::SUCCESS,
            "汇川原生程序已通过FTP上传：Local=%s Remote=%s",
            localPath.c_str(), remotePath.c_str());
    }
    ClearLastRobotError();
    return 0;
}

std::string InovanceRobotCtrl::SendDiagnosticCommand(const std::string& command)
{
    std::string response;
    if (!SendCommand(command, response))
    {
        return "ERROR:" + GetLastRobotError();
    }
    return response;
}

bool InovanceRobotCtrl::WriteCartesianRegister(
    int index, const double pose[8], int config[7])
{
    if (index < 0 || index > 9999 || pose == nullptr || config == nullptr)
    {
        SetLastRobotError("汇川位置寄存器参数无效，P序号范围为0..9999。");
        return false;
    }
    for (int valueIndex = 0; valueIndex < 8; ++valueIndex)
    {
        if (!std::isfinite(pose[valueIndex]))
        {
            SetLastRobotError("汇川位置寄存器包含非有限坐标。");
            return false;
        }
    }
    bool hasPassivePose = false;
    {
        std::lock_guard<std::mutex> lock(m_passiveMutex);
        hasPassivePose = m_passivePoseValid;
    }
    if (!hasPassivePose)
    {
        // Generic config[7] has no safe one-to-one meaning for Inovance ArmType.
        // Preserve the controller's current ArmType instead of inventing a mapping.
        T_ROBOT_COORS current;
        if (!ReadCartesianPosition(current, nullptr)) { return false; }
    }

    int arm[4] = {};
    double external[6] = {};
    {
        std::lock_guard<std::mutex> lock(m_passiveMutex);
        std::copy(std::begin(m_armConfig), std::end(m_armConfig), arm);
        std::copy(std::begin(m_externalValues), std::end(m_externalValues), external);
    }
    external[0] = pose[6];
    external[1] = pose[7];
    std::ostringstream command;
    command << "SetMemRobP " << index << ' '
        << FormatDouble(pose[0]) << ',' << FormatDouble(pose[1]) << ','
        << FormatDouble(pose[2]) << ','
        // 通用数组为X,Y,Z,RX,RY,RZ,BX,BY；汇川写入A,B,C=RZ,RY,RX。
        << FormatDouble(pose[5]) << ',' << FormatDouble(pose[4]) << ','
        << FormatDouble(pose[3]) << "; "
        << arm[0] << ',' << arm[1] << ',' << arm[2] << ',' << arm[3] << "; ";
    for (int externalIndex = 0; externalIndex < 6; ++externalIndex)
    {
        if (externalIndex > 0) { command << ','; }
        command << FormatDouble(external[externalIndex]);
    }
    const std::string commandText = command.str();
    const std::string registerPrefix = "SetMemRobP " + std::to_string(index) + " ";
    if (commandText.size() < registerPrefix.size()
        || commandText.size() - registerPrefix.size() > 128)
    {
        SetLastRobotError("汇川SetMemRobP位置参数超过手册规定的128字符上限。");
        return false;
    }
    std::string response;
    if (!SendCommand(commandText, response) || response != "ok") { return false; }

    T_ROBOT_COORS verify;
    int verifyArm[4] = {};
    if (!ReadPositionRegister(index, verify, verifyArm)) { return false; }
    const double expected[6] = { pose[0], pose[1], pose[2], pose[3], pose[4], pose[5] };
    const double actual[6] = { verify.dX, verify.dY, verify.dZ, verify.dRX, verify.dRY, verify.dRZ };
    for (int valueIndex = 0; valueIndex < 6; ++valueIndex)
    {
        if (std::abs(expected[valueIndex] - actual[valueIndex]) > 0.001)
        {
            SetLastRobotError("汇川SetMemRobP写入后Get_RobP回读不一致。");
            return false;
        }
    }
    for (int armIndex = 0; armIndex < 4; ++armIndex)
    {
        if (verifyArm[armIndex] != arm[armIndex])
        {
            SetLastRobotError("汇川SetMemRobP写入后ArmType回读不一致。");
            return false;
        }
    }
    return true;
}

bool InovanceRobotCtrl::ReadPositionRegister(
    int index, T_ROBOT_COORS& pos, int armConfig[4])
{
    if (index < 0 || index > 9999)
    {
        SetLastRobotError("汇川P位置寄存器序号范围为0..9999。");
        return false;
    }
    std::vector<double> values;
    if (!QueryDoubles("Get_RobP " + std::to_string(index), values, 16)) { return false; }
    pos = T_ROBOT_COORS();
    pos.dX = values[0];
    pos.dY = values[1];
    pos.dZ = values[2];
    pos.dRZ = values[3];
    pos.dRY = values[4];
    pos.dRX = values[5];
    pos.dBX = values[10];
    pos.dBY = values[11];
    pos.dBZ = values[12];
    if (armConfig != nullptr)
    {
        for (int armIndex = 0; armIndex < 4; ++armIndex)
        {
            armConfig[armIndex] = static_cast<int>(std::llround(values[6 + armIndex]));
        }
    }
    return true;
}

int InovanceRobotCtrl::GetPosVar(
    long index, double array[6], int config[7], int moveType)
{
    if (moveType != POSVAR || array == nullptr || config == nullptr
        || index < 0 || index > 9999)
    {
        SetLastRobotError("汇川GetPosVar只支持P0..P9999直角位置变量。");
        return -1;
    }
    T_ROBOT_COORS pos;
    int arm[4] = {};
    if (!ReadPositionRegister(static_cast<int>(index), pos, arm)) { return -1; }
    array[0] = pos.dX;
    array[1] = pos.dY;
    array[2] = pos.dZ;
    array[3] = pos.dRX;
    array[4] = pos.dRY;
    array[5] = pos.dRZ;
    std::fill(config, config + 7, 0);
    std::copy(std::begin(arm), std::end(arm), config);
    return 0;
}

bool InovanceRobotCtrl::RunProgramAndWait(
    const std::string& programName,
    int startTimeoutMs,
    int finishTimeoutMs,
    int pollDelayMs,
    RobotMotionStatus* terminalStatus)
{
    (void)programName;
    (void)startTimeoutMs;
    (void)finishTimeoutMs;
    (void)pollDelayMs;
    SetLastRobotError("汇川Prg Start只能启动控制器当前工程，手册没有证明如何按调用参数选择并绑定程序身份；"
        "为防止执行错误工程，按名称原生程序执行保持关闭。");
    if (terminalStatus != nullptr)
    {
        *terminalStatus = RobotMotionStatus{};
        terminalStatus->state = RobotMotionState::Faulted;
        terminalStatus->detail = GetLastRobotError();
    }
    return false;
}

bool InovanceRobotCtrl::GetToolData(int toolNo, T_ROBOT_COORS& robotToolData)
{
    if (toolNo < 0 || toolNo > 15)
    {
        SetLastRobotError("汇川工具号范围为0..15。");
        return false;
    }
    std::vector<double> values;
    if (!QueryDoubles("Get_ToolData " + std::to_string(toolNo), values, 17))
    {
        return false;
    }
    // 返回=RobHold; X,Y,Z,A,B,C; Load[10]。
    robotToolData = T_ROBOT_COORS();
    robotToolData.dX = values[1];
    robotToolData.dY = values[2];
    robotToolData.dZ = values[3];
    robotToolData.dRZ = values[4];
    robotToolData.dRY = values[5];
    robotToolData.dRX = values[6];
    return true;
}

bool InovanceRobotCtrl::TryGetIntVar(
    int index, int& value, const char* prefix)
{
    if (index < 0 || index > 255 || prefix == nullptr)
    {
        SetLastRobotError("汇川PLC DInt变量序号范围为0..255。");
        return false;
    }
    std::string type(prefix);
    std::transform(type.begin(), type.end(), type.begin(),
        [](unsigned char ch) { return static_cast<char>(std::toupper(ch)); });
    if (type != "INT" && type != "DINT")
    {
        SetLastRobotError("汇川TryGetIntVar当前只映射PLC DInt变量。");
        return false;
    }
    return QueryInt("Get_PlcVar DInt " + std::to_string(index), value);
}

int InovanceRobotCtrl::GetIntVar(int index, const char* prefix)
{
    int value = 0;
    TryGetIntVar(index, value, prefix);
    return value;
}

bool InovanceRobotCtrl::SetIntVar(
    int index, int value, int scope, const char* prefix)
{
    (void)index;
    (void)value;
    (void)scope;
    (void)prefix;
    SetLastRobotError("汇川字符串API表只证明PLC DInt读取，未给出对应写指令；整数变量写入保持关闭。");
    return false;
}

bool InovanceRobotCtrl::SetIntVar(const char* name, int value, int scope)
{
    (void)name;
    (void)value;
    (void)scope;
    SetLastRobotError("汇川字符串API表未证明按名称写整数变量的语义。");
    return false;
}

bool InovanceRobotCtrl::SetRealVar(
    int index, double value, const char* prefix, int scope)
{
    (void)index;
    (void)value;
    (void)prefix;
    (void)scope;
    SetLastRobotError("汇川字符串API表未证明通用REAL变量写入语义。");
    return false;
}

bool InovanceRobotCtrl::InstallHandEyeSupportPrograms(std::string* summary)
{
    const std::string message = "汇川尚未接入经验证的原生工程上传通道，无法安装机器人侧手眼辅助程序。";
    SetLastRobotError(message);
    if (summary != nullptr) { *summary = message; }
    return false;
}

bool InovanceRobotCtrl::RunHandEyeValidation(
    const T_ROBOT_COORS& robotPose,
    T_ROBOT_COORS& robotCalculatedPoint)
{
    (void)robotPose;
    (void)robotCalculatedPoint;
    SetLastRobotError("汇川手册未定义本程序所需的手眼辅助程序、输入寄存器和完成见证契约。");
    return false;
}

bool InovanceRobotCtrl::GetHandEyeMatrixVariable(
    const char* variableName,
    double rotation[9],
    double translation[3],
    std::string* error)
{
    (void)variableName;
    (void)rotation;
    (void)translation;
    const std::string message = "汇川手册未定义可读取的3x3旋转+毫米平移手眼矩阵变量契约。";
    SetLastRobotError(message);
    if (error != nullptr) { *error = message; }
    return false;
}
