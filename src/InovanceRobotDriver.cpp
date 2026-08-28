#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>

#include "AppPaths.h"
#include "InovanceRobotDriver.h"
#include "RobotDriverRegistry.h"
#include "RobotFtpFileTransfer.h"
#include "RobotOperationLease.h"

#include <QCryptographicHash>

#include <algorithm>
#include <array>
#include <chrono>
#include <cerrno>
#include <cctype>
#include <cmath>
#include <ctime>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <regex>
#include <sstream>
#include <thread>

#pragma comment(lib, "ws2_32.lib")

namespace
{
constexpr int kDefaultTimeoutMs = 3000;
constexpr std::size_t kMaxProtocolResponse = 32768;
constexpr double kMaxLinearSpeedMmPerMin = 120000.0;
constexpr int kInovanceProgramFileLimit = 16;
constexpr int kInovanceProgramInstructionLimit = 2000;
constexpr const char* kInovanceManagedTrajectoryModule = "HK_WELD_JOB";
// B255 为汇川型号底层保留的原生程序状态字节：0=待启动、1=已进入、10=自然完成。
// 使用 B 而不是 R，避免原生程序执行依赖编辑级登录；通用整数寄存器仍映射到 R。
constexpr int kInovanceNativeProgramStateByte = 255;
constexpr std::size_t kMaxNativeProgramBytes = 4U * 1024U * 1024U;
constexpr const char* kInovanceDispatcherMarker =
    "QTWIDGETSAPP4_INOVANCE_DISPATCHER_V1";

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

std::string FormatProgramDouble(double value)
{
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(6) << value;
    return stream.str();
}

std::string InovanceContentSha256(const std::string& content)
{
    const QByteArray bytes(content.data(), static_cast<int>(content.size()));
    return QCryptographicHash::hash(bytes, QCryptographicHash::Sha256)
        .toHex().toStdString();
}

std::string InovancePcTimestamp()
{
    const std::time_t now = std::time(nullptr);
    std::tm localTime{};
    localtime_s(&localTime, &now);
    std::ostringstream stamp;
    stamp << std::put_time(&localTime, "%Y%m%d_%H%M%S");
    return stamp.str();
}

const char* InovanceIoValue(int value)
{
    return value == 0 ? "OFF" : "ON";
}

bool IsInovanceNativeTrajectoryPurpose(RobotTrajectoryPurpose purpose)
{
    return purpose == RobotTrajectoryPurpose::WeldDryRun
        || purpose == RobotTrajectoryPurpose::ActualWeld;
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

std::string LowerAscii(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(),
        [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    return value;
}

bool IsInovanceProgramIdentifier(const std::string& value)
{
    if (value.empty() || value.size() > 28
        || !std::isalpha(static_cast<unsigned char>(value.front())))
    {
        return false;
    }
    return std::all_of(value.cbegin(), value.cend(), [](unsigned char ch)
        { return std::isalnum(ch) || ch == '_'; });
}

bool ParseInovanceProgramRequest(
    std::string request,
    std::string& requestedProject,
    std::string& moduleName,
    std::string& error)
{
    requestedProject.clear();
    moduleName.clear();
    error.clear();
    request = Trim(std::move(request));
    std::replace(request.begin(), request.end(), '\\', '/');
    while (!request.empty() && request.front() == '/') { request.erase(request.begin()); }
    while (!request.empty() && request.back() == '/') { request.pop_back(); }
    if (request.empty() || request.find("..") != std::string::npos
        || request.find('\0') != std::string::npos)
    {
        error = "程序身份为空或包含上级目录。";
        return false;
    }

    const std::string teachPrefix = "teachprogram/";
    if (LowerAscii(request).rfind(teachPrefix, 0) == 0)
    {
        request.erase(0, teachPrefix.size());
    }
    const std::size_t slash = request.find('/');
    if (slash != std::string::npos)
    {
        if (request.find('/', slash + 1) != std::string::npos)
        {
            error = "程序身份只允许 模块名 或 工程名/模块名。";
            return false;
        }
        requestedProject = request.substr(0, slash);
        request.erase(0, slash + 1);
        if (requestedProject.empty())
        {
            error = "程序身份中的工程名为空。";
            return false;
        }
    }

    const std::size_t dot = request.find_last_of('.');
    if (dot != std::string::npos)
    {
        if (LowerAscii(request.substr(dot)) != ".pro")
        {
            error = "汇川可执行模块必须是 .pro 文件。";
            return false;
        }
        request.erase(dot);
    }
    if (!IsInovanceProgramIdentifier(request))
    {
        error = "汇川模块名必须以字母开头，只含字母、数字、下划线，含.pro总长不超过32字符。";
        return false;
    }
    if (LowerAscii(request) == "main")
    {
        error = "main.pro 是适配层调度入口，业务程序必须放在独立公共模块中。";
        return false;
    }
    moduleName = request;
    return true;
}

bool InovanceActiveMainProgram(
    const std::string& taskProgramPath,
    std::string& projectDirectory,
    std::string& projectName,
    std::string& error)
{
    if (!InovanceActiveProjectDirectory(taskProgramPath, projectDirectory, error))
    {
        return false;
    }
    std::string normalizedPath;
    if (!NormalizeInovanceRemotePath(taskProgramPath, normalizedPath, error))
    {
        return false;
    }
    const std::size_t programSlash = normalizedPath.find_last_of('/');
    if (programSlash == std::string::npos
        || LowerAscii(normalizedPath.substr(programSlash + 1)) != "main.pro")
    {
        error = "当前主任务入口不是固定的 main.pro：" + normalizedPath;
        return false;
    }
    const std::size_t projectSlash = projectDirectory.find_last_of('/');
    if (projectSlash == std::string::npos || projectSlash + 1 >= projectDirectory.size())
    {
        error = "当前任务路径缺少工程名：" + normalizedPath;
        return false;
    }
    projectName = projectDirectory.substr(projectSlash + 1);
    return true;
}

bool ReadBoundedTextFile(
    const std::filesystem::path& path,
    std::string& content,
    std::string& error)
{
    content.clear();
    error.clear();
    std::error_code sizeError;
    const std::uintmax_t size = std::filesystem::file_size(path, sizeError);
    if (sizeError || size == 0 || size > kMaxNativeProgramBytes)
    {
        error = "程序文件为空、不可读或超过4MiB安全上限。";
        return false;
    }
    std::ifstream input(path, std::ios::binary);
    if (!input)
    {
        error = "无法打开程序文件。";
        return false;
    }
    content.assign(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
    if (!input.good() && !input.eof())
    {
        error = "读取程序文件失败。";
        content.clear();
        return false;
    }
    return true;
}

bool ValidateInovanceCallableModule(const std::string& content, std::string& error)
{
    const std::regex runFunction(
        R"((^|[\r\n])[\t ]*Func[\t ]*Run[\t ]*\([\t ]*\))",
        std::regex_constants::icase);
    const std::regex startEntry(
        R"((^|[\r\n])[\t ]*Start[\t ]*;)",
        std::regex_constants::icase);
    const std::regex mainEntry(
        R"((^|[\r\n])[\t ]*Main[\t ]*\()",
        std::regex_constants::icase);
    if (!std::regex_search(content, runFunction))
    {
        error = "模块缺少适配层约定的无参数公共入口 Func Run()。";
        return false;
    }
    if (std::regex_search(content, startEntry) || std::regex_search(content, mainEntry))
    {
        error = "公共模块包含 Start/Main 任务入口；同一工程只能由 main.pro 保留入口函数。";
        return false;
    }
    error.clear();
    return true;
}

bool WriteInovanceDispatcher(
    const std::filesystem::path& path,
    const std::string& moduleName,
    std::string& content,
    std::string& error)
{
    std::ostringstream source;
    source << "// " << kInovanceDispatcherMarker << "\r\n"
        << "Include \"" << moduleName << ".pro\";\r\n"
        << "Start;\r\n"
        << "B[" << kInovanceNativeProgramStateByte << "] = 1;\r\n"
        << moduleName << ".Run();\r\n"
        << "B[" << kInovanceNativeProgramStateByte << "] = 10;\r\n"
        << "End;\r\n";
    content = source.str();
    std::ofstream output(path, std::ios::binary | std::ios::trunc);
    if (!output)
    {
        error = "无法创建汇川 main.pro 调度器。";
        return false;
    }
    output.write(content.data(), static_cast<std::streamsize>(content.size()));
    output.flush();
    if (!output)
    {
        error = "写入汇川 main.pro 调度器失败。";
        return false;
    }
    error.clear();
    return true;
}

bool ParseInovanceIndexedVariableName(
    std::string name,
    std::string& prefix,
    int& index)
{
    prefix.clear();
    index = -1;
    name = Trim(std::move(name));
    name.erase(std::remove_if(name.begin(), name.end(), [](unsigned char ch)
        { return std::isspace(ch); }), name.end());
    if (name.empty()) { return false; }
    const std::size_t open = name.find('[');
    std::string indexText;
    if (open != std::string::npos)
    {
        if (name.back() != ']' || open == 0 || name.find('[', open + 1) != std::string::npos)
        {
            return false;
        }
        prefix = name.substr(0, open);
        indexText = name.substr(open + 1, name.size() - open - 2);
    }
    else
    {
        const auto firstDigit = std::find_if(name.cbegin(), name.cend(), [](unsigned char ch)
            { return std::isdigit(ch); });
        if (firstDigit == name.cend()) { return false; }
        prefix.assign(name.cbegin(), firstDigit);
        indexText.assign(firstDigit, name.cend());
    }
    if (prefix.empty() || indexText.empty()
        || !std::all_of(indexText.cbegin(), indexText.cend(), [](unsigned char ch)
            { return std::isdigit(ch); }))
    {
        return false;
    }
    prefix = LowerAscii(prefix);
    char* end = nullptr;
    const long parsed = std::strtol(indexText.c_str(), &end, 10);
    if (end == indexText.c_str() || *end != '\0' || parsed < 0 || parsed > 255)
    {
        return false;
    }
    index = static_cast<int>(parsed);
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
    // 原生轨迹工作线程持有 this；必须在关闭Socket和析构成员前收敛。
    if (m_nativeTrajectoryFuture.valid())
    {
        m_nativeTrajectoryFuture.wait();
        try { (void)m_nativeTrajectoryFuture.get(); }
        catch (...) {}
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
    m_weldJobEnabled = false;
    m_weldArcEnableDo = -1;
    m_weldArcEnableActiveValue = 1;
    m_weldReadyDi = -1;
    m_weldReadyActiveValue = 1;
    m_weldArcEstablishedDi = -1;
    m_weldArcEstablishedActiveValue = 1;
    m_weldCurrentDa = -1;
    m_weldCurrentDaGain = 0.0;
    m_weldCurrentDaOffset = 0.0;
    m_weldCurrentDaMin = 0.0;
    m_weldCurrentDaMax = 0.0;
    m_weldVoltageDa = -1;
    m_weldVoltageDaGain = 0.0;
    m_weldVoltageDaOffset = 0.0;
    m_weldVoltageDaMin = 0.0;
    m_weldVoltageDaMax = 0.0;
    m_weldReadyTimeoutMs = 10000;
    m_weldArcStartTimeoutMs = 10000;
    m_weldArcEndTimeoutMs = 10000;
    m_weldAlarmIndex = 0;
    m_weldArcInterruptId = -1;

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

    ini.SetSectionName("WeldJob");
    ini.ReadString(false, "Enabled", &m_weldJobEnabled);
    ini.ReadString(false, "ArcEnableDO", &m_weldArcEnableDo);
    ini.ReadString(false, "ArcEnableActiveValue", &m_weldArcEnableActiveValue);
    ini.ReadString(false, "ReadyDI", &m_weldReadyDi);
    ini.ReadString(false, "ReadyActiveValue", &m_weldReadyActiveValue);
    ini.ReadString(false, "ArcEstablishedDI", &m_weldArcEstablishedDi);
    ini.ReadString(false, "ArcEstablishedActiveValue", &m_weldArcEstablishedActiveValue);
    ini.ReadString(false, "CurrentDA", &m_weldCurrentDa);
    ini.ReadString(false, "CurrentDAGain", &m_weldCurrentDaGain);
    ini.ReadString(false, "CurrentDAOffset", &m_weldCurrentDaOffset);
    ini.ReadString(false, "CurrentDAMin", &m_weldCurrentDaMin);
    ini.ReadString(false, "CurrentDAMax", &m_weldCurrentDaMax);
    ini.ReadString(false, "VoltageDA", &m_weldVoltageDa);
    ini.ReadString(false, "VoltageDAGain", &m_weldVoltageDaGain);
    ini.ReadString(false, "VoltageDAOffset", &m_weldVoltageDaOffset);
    ini.ReadString(false, "VoltageDAMin", &m_weldVoltageDaMin);
    ini.ReadString(false, "VoltageDAMax", &m_weldVoltageDaMax);
    ini.ReadString(false, "ReadyTimeoutMs", &m_weldReadyTimeoutMs);
    ini.ReadString(false, "ArcStartTimeoutMs", &m_weldArcStartTimeoutMs);
    ini.ReadString(false, "ArcEndTimeoutMs", &m_weldArcEndTimeoutMs);
    ini.ReadString(false, "AlarmIndex", &m_weldAlarmIndex);
    ini.ReadString(false, "ArcInterruptId", &m_weldArcInterruptId);

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
        | RobotDriverCapabilityBit(RobotDriverCapability::OperationModeControl)
        | RobotDriverCapabilityBit(RobotDriverCapability::NativeProgramUpload)
        | RobotDriverCapabilityBit(RobotDriverCapability::NativeProgramExecution)
        | RobotDriverCapabilityBit(RobotDriverCapability::DiagnosticCommand)
        | RobotDriverCapabilityBit(RobotDriverCapability::CartesianRegister)
        | RobotDriverCapabilityBit(RobotDriverCapability::IntegerRegister)
        | RobotDriverCapabilityBit(RobotDriverCapability::VerifiedProgramCompletion)
        | RobotDriverCapabilityBit(RobotDriverCapability::VerifiedSafeAbort)
        | RobotDriverCapabilityBit(RobotDriverCapability::ConnectionControl)
        | RobotDriverCapabilityBit(RobotDriverCapability::AlarmReset)
        | RobotDriverCapabilityBit(RobotDriverCapability::ServoPowerControl)
        | RobotDriverCapabilityBit(RobotDriverCapability::ToolDataRead)
        | RobotDriverCapabilityBit(RobotDriverCapability::TeachPendantSpeedControl)
        | RobotDriverCapabilityBit(RobotDriverCapability::FtpFileTransfer)
        | RobotDriverCapabilityBit(RobotDriverCapability::OfflineTrajectoryExport);
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
    if (HasVerifiedWeldJobContract(nullptr))
    {
        capabilities |= RobotDriverCapabilityBit(RobotDriverCapability::ActualArcWeld);
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
    m_nativeProgramRunning.store(false);
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
    // 与安全 STOP 共用 socket 互斥锁后再检查取消锁存，关闭
    // “检查通过 -> STOP -> 随后才发送 Prg Start”的竞态窗口。
    if (command == "Prg Start" && RobotOperationLease::IsCancellationRequested(this))
    {
        SetLastRobotError("汇川硬件操作已被安全停止取消，Prg Start 未发送。");
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
    while (end != nullptr && *end != '\0'
        && std::isspace(static_cast<unsigned char>(*end)))
    {
        ++end;
    }
    if (errno != 0 || end == text.c_str()
        || (end != nullptr && *end != '\0')
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
    int taskStatus = -1;
    if (QueryInt("Get_MotionSts", motion)
        && QueryInt("Get_DsMode", dataStreamMode)
        && QueryInt("Get_TaskRunSts 0", taskStatus)
        && (motion != 0 || dataStreamMode != 0 || taskStatus == 1
            || m_nativeProgramRunning.load()))
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
    if (m_trajectoryRunning.load() || m_nativeProgramRunning.load())
    {
        SetLastRobotError("汇川运动或原生JOB运行期间禁止重载机器人配置；"
            "避免焊接IO/DA映射与已启动JOB发生变化。");
        if (m_pRobotLog != nullptr)
        {
            m_pRobotLog->write(LogColor::ERR, "%s", GetLastRobotError().c_str());
        }
        return;
    }
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
    else if (m_nativeProgramRunning.load())
    {
        int taskStatus = -1;
        int stateByte = -1;
        int fault = -1;
        if (!QueryInt("Get_TaskRunSts 0", taskStatus)
            || !TryGetIntVar(kInovanceNativeProgramStateByte, stateByte, "B")
            || !QueryInt("Get_SysErrSts", fault))
        {
            status.state = RobotMotionState::Unknown;
            status.detail = GetLastRobotError();
        }
        else if (fault != 0)
        {
            status.state = RobotMotionState::Faulted;
            status.detail = "汇川原生JOB运行中控制器故障=" + std::to_string(fault);
        }
        else if (stateByte == 10 && taskStatus != 1 && motion != 1)
        {
            status.state = RobotMotionState::Completed;
            status.terminalVerified = true;
            status.detail = "汇川原生JOB已写入B255=10且任务/运动停止";
        }
        else if (taskStatus == 1 || stateByte == 1 || motion == 1)
        {
            status.state = RobotMotionState::Running;
            status.detail = "汇川原生JOB运行中";
        }
        else
        {
            status.state = RobotMotionState::Interrupted;
            status.detail = "汇川原生JOB已停止但缺少B255=10自然完成见证";
        }
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
            move.tSpeed.dSpeed, move.dOverlapRel, move.dWeldSpeedMmPerMin,
            move.dArcStartCurrent, move.dArcStartVoltage, move.dArcStartWaitTime,
            move.dWeldCurrent, move.dWeldVoltage,
            move.dArcEndCurrent, move.dArcEndVoltage, move.dArcEndWaitTime
        };
        mixBytes(values, sizeof(values));
        const long pulses[] = {
            move.tPulse.nSPulse, move.tPulse.nLPulse, move.tPulse.nUPulse,
            move.tPulse.nRPulse, move.tPulse.nBPulse, move.tPulse.nTPulse,
            move.tPulse.lBXPulse, move.tPulse.lBYPulse, move.tPulse.lBZPulse
        };
        mixBytes(pulses, sizeof(pulses));
        const int integers[] = {
            move.nMoveType, move.nPosType, move.nDwellMs,
            move.nPostureType, move.nDynamicMode, move.nMoveDevice,
            move.nTrackNo, move.nArcMode, move.nWeavePointsPerCycle
        };
        mixBytes(integers, sizeof(integers));
        const bool flags[] = {
            move.bWeldProcessEnabled,
            move.bArcStartBeforeMove,
            move.bArcEndAfterMove,
            move.bUseTransitionWeldParams,
            move.bHasWeaveParam,
            move.bAppPointwiseWeave,
            move.bHasTrackParam
        };
        mixBytes(flags, sizeof(flags));
    }
    if (purpose == RobotTrajectoryPurpose::ActualWeld)
    {
        const int weldIntegers[] = {
            m_weldJobEnabled ? 1 : 0,
            m_weldArcEnableDo, m_weldArcEnableActiveValue,
            m_weldReadyDi, m_weldReadyActiveValue,
            m_weldArcEstablishedDi, m_weldArcEstablishedActiveValue,
            m_weldCurrentDa, m_weldVoltageDa,
            m_weldReadyTimeoutMs, m_weldArcStartTimeoutMs,
            m_weldArcEndTimeoutMs, m_weldAlarmIndex, m_weldArcInterruptId
        };
        const double weldValues[] = {
            m_weldCurrentDaGain, m_weldCurrentDaOffset,
            m_weldCurrentDaMin, m_weldCurrentDaMax,
            m_weldVoltageDaGain, m_weldVoltageDaOffset,
            m_weldVoltageDaMin, m_weldVoltageDaMax
        };
        mixBytes(weldIntegers, sizeof(weldIntegers));
        mixBytes(weldValues, sizeof(weldValues));
    }
    return hash;
}

bool InovanceRobotCtrl::HasVerifiedWeldJobContract(std::string* error) const
{
    std::vector<std::string> missing;
    const auto require = [&missing](bool condition, const char* name)
        {
            if (!condition) { missing.emplace_back(name); }
        };
    const auto validBit = [](int value) { return value == 0 || value == 1; };
    const auto validDa = [](int channel)
        { return (channel >= 0 && channel <= 15) || (channel >= 64 && channel <= 79); };
    const auto validScale = [](double gain, double offset, double minimum, double maximum)
        {
            return std::isfinite(gain) && std::abs(gain) >= 1e-12
                && std::isfinite(offset) && std::isfinite(minimum)
                && std::isfinite(maximum) && minimum < maximum;
        };

    require(m_weldJobEnabled, "Enabled=1");
    require(m_weldArcEnableDo >= 0 && m_weldArcEnableDo <= 13823, "ArcEnableDO");
    require(validBit(m_weldArcEnableActiveValue), "ArcEnableActiveValue(0/1)");
    require(m_weldReadyDi >= 0 && m_weldReadyDi <= 13823, "ReadyDI");
    require(validBit(m_weldReadyActiveValue), "ReadyActiveValue(0/1)");
    require(m_weldArcEstablishedDi >= 0 && m_weldArcEstablishedDi <= 13823,
        "ArcEstablishedDI");
    require(validBit(m_weldArcEstablishedActiveValue),
        "ArcEstablishedActiveValue(0/1)");
    require(validDa(m_weldCurrentDa), "CurrentDA(0..15/64..79)");
    require(validScale(m_weldCurrentDaGain, m_weldCurrentDaOffset,
        m_weldCurrentDaMin, m_weldCurrentDaMax),
        "CurrentDAGain/Offset/Min/Max");
    require(validDa(m_weldVoltageDa), "VoltageDA(0..15/64..79)");
    require(validScale(m_weldVoltageDaGain, m_weldVoltageDaOffset,
        m_weldVoltageDaMin, m_weldVoltageDaMax),
        "VoltageDAGain/Offset/Min/Max");
    require(m_weldCurrentDa != m_weldVoltageDa, "CurrentDA!=VoltageDA");
    require(m_weldReadyTimeoutMs > 0 && m_weldReadyTimeoutMs <= 65535000,
        "ReadyTimeoutMs");
    require(m_weldArcStartTimeoutMs > 0 && m_weldArcStartTimeoutMs <= 65535000,
        "ArcStartTimeoutMs");
    require(m_weldArcEndTimeoutMs > 0 && m_weldArcEndTimeoutMs <= 65535000,
        "ArcEndTimeoutMs");
    require(m_weldAlarmIndex >= 0 && m_weldAlarmIndex <= 15, "AlarmIndex(0..15)");
    require(m_weldArcInterruptId >= 0 && m_weldArcInterruptId <= 127,
        "ArcInterruptId(0..127)");

    if (!missing.empty())
    {
        if (error != nullptr)
        {
            std::ostringstream detail;
            detail << "汇川实际焊接JOB映射未完成，数据库 Robot/" << m_sRobotName
                << "/RobotPara/WeldJob 缺少或无效：";
            for (std::size_t index = 0; index < missing.size(); ++index)
            {
                if (index > 0) { detail << ", "; }
                detail << missing[index];
            }
            detail << "。未完成现场映射时只允许空跑JOB。";
            *error = detail.str();
        }
        return false;
    }
    if (error != nullptr) { error->clear(); }
    return true;
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
    const bool nativeJob = IsInovanceNativeTrajectoryPurpose(purpose);
    const bool actualWeld = purpose == RobotTrajectoryPurpose::ActualWeld;
    if (actualWeld && !HasVerifiedWeldJobContract(&error))
    {
        return false;
    }
    const T_ROBOT_MOVE_INFO& firstMove = moveInfos.front();
    const T_ROBOT_MOVE_INFO& lastMove = moveInfos.back();
    if (actualWeld
        && (!firstMove.bArcStartBeforeMove || !lastMove.bArcEndAfterMove))
    {
        error = "汇川实际焊接JOB要求首点明确起弧、末点明确收弧。";
        return false;
    }
    const auto mappedDaInRange = [](double processValue, double gain,
        double offset, double minimum, double maximum)
        {
            const double output = processValue * gain + offset;
            return std::isfinite(processValue) && std::isfinite(output)
                && output >= minimum && output <= maximum;
        };
    for (std::size_t index = 0; index < moveInfos.size(); ++index)
    {
        const T_ROBOT_MOVE_INFO& move = moveInfos[index];
        const double poseValues[] = {
            move.tCoord.dX, move.tCoord.dY, move.tCoord.dZ,
            move.tCoord.dRX, move.tCoord.dRY, move.tCoord.dRZ,
            move.tCoord.dBX, move.tCoord.dBY, move.tCoord.dBZ
        };
        if (move.nMoveType == MOVL
            && !std::all_of(std::begin(poseValues), std::end(poseValues),
                [](double value) { return std::isfinite(value); }))
        {
            error = "汇川第" + std::to_string(index + 1) + "个直线点包含非有限位姿。";
            return false;
        }
        if (!nativeJob
            && (move.bWeldProcessEnabled || move.bArcStartBeforeMove
                || move.bArcEndAfterMove))
        {
            error = "汇川第" + std::to_string(index + 1)
                + "点包含焊接/起弧标志，扫描数据流禁止携带焊接动作。";
            return false;
        }
        if (!nativeJob && move.nDwellMs > 0)
        {
            error = "汇川数据流适配暂未把轨迹停留转换为经验证的控制器等待指令。";
            return false;
        }
        if (nativeJob && move.bHasTrackParam)
        {
            error = "汇川原生JOB尚未实现跟踪参数，已拒绝忽略轨迹跟踪语义。";
            return false;
        }
        if (nativeJob && move.bHasWeaveParam && !move.bAppPointwiseWeave)
        {
            error = "汇川原生JOB尚未实现控制器原生摆动；请改用上位机已展开的pointwise摆动轨迹。";
            return false;
        }
        if (actualWeld)
        {
            if (move.nMoveType != MOVL || !move.bWeldProcessEnabled)
            {
                error = "汇川实际焊接JOB的所有焊道点必须是启用焊接工艺的MOVL点。";
                return false;
            }
            if ((index != 0 && move.bArcStartBeforeMove)
                || (index + 1 != moveInfos.size() && move.bArcEndAfterMove))
            {
                error = "汇川实际焊接JOB只允许首点起弧、末点收弧。";
                return false;
            }
            if (move.bUseTransitionWeldParams)
            {
                error = "汇川JOB中的Set DA会打断相邻运动预处理；过渡电流/电压尚无连续切换证明，实际焊接保持限制。";
                return false;
            }
            if (!mappedDaInRange(move.dArcStartCurrent,
                    m_weldCurrentDaGain, m_weldCurrentDaOffset,
                    m_weldCurrentDaMin, m_weldCurrentDaMax)
                || !mappedDaInRange(move.dArcEndCurrent,
                    m_weldCurrentDaGain, m_weldCurrentDaOffset,
                    m_weldCurrentDaMin, m_weldCurrentDaMax)
                || !mappedDaInRange(move.dWeldCurrent,
                    m_weldCurrentDaGain, m_weldCurrentDaOffset,
                    m_weldCurrentDaMin, m_weldCurrentDaMax)
                || !mappedDaInRange(move.dArcStartVoltage,
                    m_weldVoltageDaGain, m_weldVoltageDaOffset,
                    m_weldVoltageDaMin, m_weldVoltageDaMax)
                || !mappedDaInRange(move.dArcEndVoltage,
                    m_weldVoltageDaGain, m_weldVoltageDaOffset,
                    m_weldVoltageDaMin, m_weldVoltageDaMax)
                || !mappedDaInRange(move.dWeldVoltage,
                    m_weldVoltageDaGain, m_weldVoltageDaOffset,
                    m_weldVoltageDaMin, m_weldVoltageDaMax)
                || !std::isfinite(move.dArcStartWaitTime)
                || move.dArcStartWaitTime < 0.0 || move.dArcStartWaitTime > 65535.0
                || !std::isfinite(move.dArcEndWaitTime)
                || move.dArcEndWaitTime < 0.0 || move.dArcEndWaitTime > 65535.0)
            {
                error = "汇川第" + std::to_string(index + 1)
                    + "点的焊接电流/电压映射越过配置DA范围，或起收弧等待时间无效。";
                return false;
            }
            if (index > 0)
            {
                const T_ROBOT_MOVE_INFO& reference = moveInfos.front();
                const auto differs = [](double left, double right)
                    { return std::abs(left - right) > 1e-9; };
                if (differs(move.dArcStartCurrent, reference.dArcStartCurrent)
                    || differs(move.dArcStartVoltage, reference.dArcStartVoltage)
                    || differs(move.dArcStartWaitTime, reference.dArcStartWaitTime)
                    || differs(move.dWeldCurrent, reference.dWeldCurrent)
                    || differs(move.dWeldVoltage, reference.dWeldVoltage)
                    || differs(move.dArcEndCurrent, reference.dArcEndCurrent)
                    || differs(move.dArcEndVoltage, reference.dArcEndVoltage)
                    || differs(move.dArcEndWaitTime, reference.dArcEndWaitTime))
                {
                    error = "汇川实际焊接JOB当前只允许一组稳定起弧/焊接/收弧参数；检测到点间工艺参数变化。";
                    return false;
                }
            }
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

bool InovanceRobotCtrl::WriteTrajectoryJobFile(
    const std::vector<T_ROBOT_MOVE_INFO>& moveInfos,
    RobotTrajectoryPurpose purpose,
    const std::string& outputDirectory,
    RobotTrajectoryHandle& handle,
    std::string& error)
{
    if (!ValidateMoveInfos(moveInfos, purpose, error)) { return false; }
    if (outputDirectory.empty())
    {
        error = "汇川原生JOB输出目录为空。";
        return false;
    }
    if (handle.programName.empty() && !ReserveTrajectory(purpose, handle))
    {
        error = GetLastRobotError();
        return false;
    }
    if (!IsInovanceProgramIdentifier(handle.programName))
    {
        error = "汇川原生JOB程序名不符合PRO模块命名规则：" + handle.programName;
        return false;
    }

    bool needsCartesianArm = false;
    for (const T_ROBOT_MOVE_INFO& move : moveInfos)
    {
        needsCartesianArm = needsCartesianArm || move.nMoveType == MOVL;
    }
    if (needsCartesianArm)
    {
        bool armReady = false;
        {
            std::lock_guard<std::mutex> lock(m_passiveMutex);
            armReady = m_passivePoseValid;
        }
        if (!armReady)
        {
            T_ROBOT_COORS current;
            if (!IsConnected() || !ReadCartesianPosition(current, nullptr))
            {
                error = "汇川原生JOB生成前无法取得当前ArmType；请先连接机器人并完成一次位姿回读。";
                return false;
            }
        }
    }

    int arm[4] = {};
    double passiveExternal[6] = {};
    {
        std::lock_guard<std::mutex> lock(m_passiveMutex);
        std::copy(std::begin(m_armConfig), std::end(m_armConfig), arm);
        std::copy(std::begin(m_externalValues), std::end(m_externalValues), passiveExternal);
    }
    const double axisUnits[9] = {
        m_tAxisUnit.dSPulseUnit, m_tAxisUnit.dLPulseUnit, m_tAxisUnit.dUPulseUnit,
        m_tAxisUnit.dRPulseUnit, m_tAxisUnit.dBPulseUnit, m_tAxisUnit.dTPulseUnit,
        m_tAxisUnit.dBXPulseUnit, m_tAxisUnit.dBYPulseUnit, m_tAxisUnit.dBZPulseUnit
    };

    const bool actualWeld = purpose == RobotTrajectoryPurpose::ActualWeld;
    const int arcInactiveValue = m_weldArcEnableActiveValue == 0 ? 1 : 0;
    const int arcLostValue = m_weldArcEstablishedActiveValue == 0 ? 1 : 0;
    const auto mappedCurrent = [this](double current)
        { return current * m_weldCurrentDaGain + m_weldCurrentDaOffset; };
    const auto mappedVoltage = [this](double voltage)
        { return voltage * m_weldVoltageDaGain + m_weldVoltageDaOffset; };
    const auto timeoutSeconds = [](int timeoutMs)
        { return static_cast<double>(timeoutMs) / 1000.0; };

    std::ostringstream source;
    source << "// QTWIDGETSAPP4_INOVANCE_TRAJECTORY_JOB_V1\r\n"
        << "// PC_TIMESTAMP=" << InovancePcTimestamp() << "\r\n"
        << "Func Run()\r\n";
    for (std::size_t index = 0; index < moveInfos.size(); ++index)
    {
        const T_ROBOT_MOVE_INFO& move = moveInfos[index];
        if (move.nMoveType == MOVL)
        {
            const double external[6] = {
                move.tCoord.dBX, move.tCoord.dBY, move.tCoord.dBZ,
                passiveExternal[3], passiveExternal[4], passiveExternal[5]
            };
            source << "LP[" << index << "] = {(" << FormatProgramDouble(move.tCoord.dX)
                << ',' << FormatProgramDouble(move.tCoord.dY)
                << ',' << FormatProgramDouble(move.tCoord.dZ)
                // 汇川PRO使用A,B,C；通用适配层使用RX,RY,RZ。
                << ',' << FormatProgramDouble(move.tCoord.dRZ)
                << ',' << FormatProgramDouble(move.tCoord.dRY)
                << ',' << FormatProgramDouble(move.tCoord.dRX) << "),("
                << arm[0] << ',' << arm[1] << ',' << arm[2] << ',' << arm[3] << "),("
                << FormatProgramDouble(external[0]) << ',' << FormatProgramDouble(external[1])
                << ',' << FormatProgramDouble(external[2]) << ',' << FormatProgramDouble(external[3])
                << ',' << FormatProgramDouble(external[4]) << ',' << FormatProgramDouble(external[5])
                << ")};\r\n";
        }
        else
        {
            source << "JP[" << index << "] = {("
                << FormatProgramDouble(static_cast<double>(move.tPulse.nSPulse) * axisUnits[0]) << ','
                << FormatProgramDouble(static_cast<double>(move.tPulse.nLPulse) * axisUnits[1]) << ','
                << FormatProgramDouble(static_cast<double>(move.tPulse.nUPulse) * axisUnits[2]) << ','
                << FormatProgramDouble(static_cast<double>(move.tPulse.nRPulse) * axisUnits[3]) << ','
                << FormatProgramDouble(static_cast<double>(move.tPulse.nBPulse) * axisUnits[4]) << ','
                << FormatProgramDouble(static_cast<double>(move.tPulse.nTPulse) * axisUnits[5]) << "),("
                << FormatProgramDouble(static_cast<double>(move.tPulse.lBXPulse) * axisUnits[6]) << ','
                << FormatProgramDouble(static_cast<double>(move.tPulse.lBYPulse) * axisUnits[7]) << ','
                << FormatProgramDouble(static_cast<double>(move.tPulse.lBZPulse) * axisUnits[8])
                << ",0.000000,0.000000,0.000000)};\r\n";
        }
    }

    if (actualWeld)
    {
        const T_ROBOT_MOVE_INFO& process = moveInfos.front();
        source << "Set Out[" << m_weldArcEnableDo << "],"
            << InovanceIoValue(arcInactiveValue) << ";\r\n"
            << "Wait In[" << m_weldReadyDi << "] == "
            << InovanceIoValue(m_weldReadyActiveValue) << ",T["
            << FormatProgramDouble(timeoutSeconds(m_weldReadyTimeoutMs))
            << "],Goto L[900];\r\n"
            << "Set DA[" << m_weldCurrentDa << "],"
            << FormatProgramDouble(mappedCurrent(process.dArcStartCurrent)) << ";\r\n"
            << "Set DA[" << m_weldVoltageDa << "],"
            << FormatProgramDouble(mappedVoltage(process.dArcStartVoltage)) << ";\r\n"
            << "IDelete " << m_weldArcInterruptId << ";\r\n"
            << "IConnect " << m_weldArcInterruptId << ",ArcLostTrap();\r\n"
            << "ISigIn(ONCE," << m_weldArcInterruptId << ','
            << m_weldArcEstablishedDi << ',' << InovanceIoValue(arcLostValue)
            << ");\r\n"
            << "IActive " << m_weldArcInterruptId << ";\r\n"
            << "IEnable;\r\n"
            << "Set Out[" << m_weldArcEnableDo << "],"
            << InovanceIoValue(m_weldArcEnableActiveValue) << ";\r\n"
            << "Wait In[" << m_weldArcEstablishedDi << "] == "
            << InovanceIoValue(m_weldArcEstablishedActiveValue) << ",T["
            << FormatProgramDouble(timeoutSeconds(m_weldArcStartTimeoutMs))
            << "],Goto L[901];\r\n";
        if (process.dArcStartWaitTime > 0.0)
        {
            source << "Wait T[" << FormatProgramDouble(process.dArcStartWaitTime) << "];\r\n";
        }
        source << "Set DA[" << m_weldCurrentDa << "],"
            << FormatProgramDouble(mappedCurrent(process.dWeldCurrent)) << ";\r\n"
            << "Set DA[" << m_weldVoltageDa << "],"
            << FormatProgramDouble(mappedVoltage(process.dWeldVoltage)) << ";\r\n";
    }

    for (std::size_t index = 0; index < moveInfos.size(); ++index)
    {
        const T_ROBOT_MOVE_INFO& move = moveInfos[index];
        const bool exact = index == 0 || index + 1 == moveInfos.size()
            || !std::isfinite(move.dOverlapRel) || move.dOverlapRel <= 0.0;
        const std::string zone = exact ? std::string("Fine")
            : "ZR[" + std::to_string(std::clamp(
                static_cast<int>(std::lround(move.dOverlapRel)), 1, 200)) + "]";
        if (move.nMoveType == MOVL)
        {
            const double speedMmPerMin = move.dWeldSpeedMmPerMin > 0.0
                ? move.dWeldSpeedMmPerMin : move.tSpeed.dSpeed;
            source << "Movl LP[" << index << "],Speed["
                << FormatProgramDouble(speedMmPerMin / 60.0)
                << "].Bstatic:1," << zone << ",Tool[" << m_toolNo
                << "],Wobj[" << m_wobjNo << "];\r\n";
        }
        else
        {
            source << "MovAbsJ JP[" << index << "],V["
                << std::clamp(static_cast<int>(std::lround(move.tSpeed.dSpeed)), 1, 100)
                << "]," << zone << ",Tool[" << m_toolNo << "],Wobj["
                << m_wobjNo << "];\r\n";
        }
        if (move.nDwellMs > 0)
        {
            source << "Wait T["
                << FormatProgramDouble(static_cast<double>(move.nDwellMs) / 1000.0)
                << "];\r\n";
        }
    }

    if (actualWeld)
    {
        const T_ROBOT_MOVE_INFO& process = moveInfos.back();
        source << "Set DA[" << m_weldCurrentDa << "],"
            << FormatProgramDouble(mappedCurrent(process.dArcEndCurrent)) << ";\r\n"
            << "Set DA[" << m_weldVoltageDa << "],"
            << FormatProgramDouble(mappedVoltage(process.dArcEndVoltage)) << ";\r\n";
        if (process.dArcEndWaitTime > 0.0)
        {
            source << "Wait T[" << FormatProgramDouble(process.dArcEndWaitTime) << "];\r\n";
        }
        source << "IDeactive " << m_weldArcInterruptId << ";\r\n"
            << "IDisable;\r\n"
            << "IDelete " << m_weldArcInterruptId << ";\r\n"
            << "Set Out[" << m_weldArcEnableDo << "],"
            << InovanceIoValue(arcInactiveValue) << ";\r\n"
            << "Wait In[" << m_weldArcEstablishedDi << "] == "
            << InovanceIoValue(arcLostValue) << ",T["
            << FormatProgramDouble(timeoutSeconds(m_weldArcEndTimeoutMs))
            << "],Goto L[903];\r\n"
            << "Goto L[999];\r\n"
            << "L[900]:\r\nSet Out[" << m_weldArcEnableDo << "],"
            << InovanceIoValue(arcInactiveValue)
            << ";\r\nPrint \"WELDER NOT READY\";\r\nAlarm[" << m_weldAlarmIndex << "];\r\n"
            << "L[901]:\r\nIDeactive " << m_weldArcInterruptId << ";\r\n"
            << "IDisable;\r\n"
            << "IDelete " << m_weldArcInterruptId << ";\r\n"
            << "Set Out[" << m_weldArcEnableDo << "],"
            << InovanceIoValue(arcInactiveValue) << ";\r\n"
            << "Print \"ARC START TIMEOUT\";\r\nAlarm[" << m_weldAlarmIndex << "];\r\n"
            << "L[903]:\r\nSet Out[" << m_weldArcEnableDo << "],"
            << InovanceIoValue(arcInactiveValue)
            << ";\r\nPrint \"ARC OFF TIMEOUT\";\r\nAlarm[" << m_weldAlarmIndex << "];\r\n"
            << "L[999]:\r\n";
    }
    source << "EndFunc;\r\n";
    if (actualWeld)
    {
        source << "Trap ArcLostTrap()\r\n"
            << "Set Out[" << m_weldArcEnableDo << "],"
            << InovanceIoValue(arcInactiveValue) << ";\r\n"
            << "Print \"ARC LOST\";\r\n"
            << "Alarm[" << m_weldAlarmIndex << "];\r\n"
            << "EndTrap;\r\n";
    }
    const std::string content = source.str();
    const int lineCount = static_cast<int>(std::count(content.cbegin(), content.cend(), '\n'));
    if (lineCount > kInovanceProgramInstructionLimit)
    {
        error = "汇川原生JOB生成后共" + std::to_string(lineCount)
            + "行，超过单个PRO最多2000行限制；需提高采样间距或拆分焊道。";
        return false;
    }
    if (content.size() > kMaxNativeProgramBytes)
    {
        error = "汇川原生JOB超过4MiB适配层安全上限。";
        return false;
    }

    const std::filesystem::path directory(outputDirectory);
    std::error_code directoryError;
    std::filesystem::create_directories(directory, directoryError);
    if (directoryError || !std::filesystem::is_directory(directory, directoryError))
    {
        error = "无法创建汇川原生JOB输出目录：" + outputDirectory;
        return false;
    }
    const std::filesystem::path programPath = directory / (handle.programName + ".pro");
    std::ofstream output(programPath, std::ios::binary | std::ios::trunc);
    if (!output)
    {
        error = "无法创建汇川原生JOB文件：" + programPath.string();
        return false;
    }
    output.write(content.data(), static_cast<std::streamsize>(content.size()));
    output.flush();
    if (!output)
    {
        error = "写入汇川原生JOB文件失败：" + programPath.string();
        return false;
    }

    handle.localProgramPath = programPath.string();
    handle.localDataPath.clear();
    handle.remoteProgramPath.clear();
    handle.remoteDataPath.clear();
    handle.programContentSha256 = InovanceContentSha256(content);
    handle.dataContentSha256.clear();
    handle.programContentSize = static_cast<std::uint64_t>(content.size());
    handle.dataContentSize = 0;
    handle.prepared = true;
    handle.started = false;
    error.clear();
    return true;
}

bool InovanceRobotCtrl::UploadTrajectoryJob(
    RobotTrajectoryHandle& handle,
    std::string& error)
{
    if (!handle.prepared || handle.localProgramPath.empty()
        || handle.programContentSha256.size() != 64 || handle.programContentSize == 0)
    {
        error = "汇川原生JOB尚未完成本地生成和内容身份冻结。";
        return false;
    }
    if (!IsConnected())
    {
        error = "汇川原生JOB上传前2222控制通道未连接。";
        return false;
    }
    int taskStatus = -1;
    if (!QueryInt("Get_TaskRunSts 0", taskStatus) || taskStatus == 1)
    {
        error = "汇川原生JOB上传前主任务未停止。";
        return false;
    }
    std::string taskPathResponse;
    std::string activeDirectory;
    std::string activeProject;
    if (!SendCommand("Get_TaskPrgPath 0", taskPathResponse)
        || !InovanceActiveMainProgram(ValuePart(taskPathResponse),
            activeDirectory, activeProject, error))
    {
        if (error.empty()) { error = GetLastRobotError(); }
        return false;
    }
    std::string sessionError;
    const std::shared_ptr<RobotFileTransferSession> session =
        CreateFileTransferSession(&sessionError);
    if (session == nullptr)
    {
        error = "汇川原生JOB无法建立FTP底层：" + sessionError;
        return false;
    }
    std::vector<RobotControllerFileInfo> entries;
    if (!session->ListProgramFiles(activeDirectory, entries, 10000))
    {
        error = "汇川原生JOB上传前无法读取当前工程文件清单：" + session->LastError();
        return false;
    }
    int proCount = 0;
    bool mainFound = false;
    bool targetFound = false;
    const std::string targetFile = handle.programName + ".pro";
    for (const RobotControllerFileInfo& entry : entries)
    {
        if (entry.isDirectory) { continue; }
        const std::string lower = LowerAscii(entry.name);
        if (lower.size() < 4 || lower.substr(lower.size() - 4) != ".pro") { continue; }
        ++proCount;
        mainFound = mainFound || lower == "main.pro";
        targetFound = targetFound || lower == LowerAscii(targetFile);
    }
    if (!mainFound)
    {
        error = "汇川当前工程缺少固定入口main.pro，禁止写入轨迹模块。";
        return false;
    }
    if (proCount + (targetFound ? 0 : 1) > kInovanceProgramFileLimit)
    {
        error = "汇川当前工程没有可用PRO槽位：已有" + std::to_string(proCount)
            + "个，控制器上限为16个。";
        return false;
    }

    const std::string remotePath = activeDirectory + "/" + targetFile;
    if (!session->UploadProgramFile(handle.localProgramPath, remotePath, true))
    {
        error = "汇川原生JOB上传失败：" + session->LastError();
        return false;
    }
    const std::filesystem::path verifyPath =
        std::filesystem::path(handle.localProgramPath).parent_path()
        / (handle.programName + "_uploaded_verify.pro");
    if (!session->DownloadProgramFile(remotePath, verifyPath.string()))
    {
        error = "汇川原生JOB上传后无法回读：" + session->LastError();
        return false;
    }
    std::string uploadedContent;
    std::string readError;
    if (!ReadBoundedTextFile(verifyPath, uploadedContent, readError)
        || uploadedContent.size() != handle.programContentSize
        || InovanceContentSha256(uploadedContent) != handle.programContentSha256)
    {
        error = "汇川原生JOB上传后字节身份不一致："
            + (readError.empty() ? std::string("SHA-256或大小不一致。") : readError);
        return false;
    }
    handle.remoteProgramPath = remotePath;
    handle.remoteDataPath.clear();
    error.clear();
    return true;
}

bool InovanceRobotCtrl::VerifyTrajectoryJobRemoteIdentity(
    const RobotTrajectoryHandle& handle,
    std::string& error) const
{
    if (handle.remoteProgramPath.empty() || handle.localProgramPath.empty()
        || handle.programContentSha256.size() != 64 || handle.programContentSize == 0)
    {
        error = "汇川原生JOB缺少远端路径或冻结内容身份。";
        return false;
    }
    std::string sessionError;
    const std::shared_ptr<RobotFileTransferSession> session =
        CreateFileTransferSession(&sessionError);
    if (session == nullptr)
    {
        error = "汇川原生JOB启动前无法建立FTP复核：" + sessionError;
        return false;
    }
    const std::filesystem::path verifyPath =
        std::filesystem::path(handle.localProgramPath).parent_path()
        / (handle.programName + "_start_verify.pro");
    if (!session->DownloadProgramFile(handle.remoteProgramPath, verifyPath.string()))
    {
        error = "汇川原生JOB启动前远端回读失败：" + session->LastError();
        return false;
    }
    std::string content;
    std::string readError;
    if (!ReadBoundedTextFile(verifyPath, content, readError)
        || content.size() != handle.programContentSize
        || InovanceContentSha256(content) != handle.programContentSha256)
    {
        error = "汇川原生JOB在下发后、启动前发生内容变化："
            + (readError.empty() ? std::string("SHA-256或大小不一致。") : readError);
        return false;
    }
    error.clear();
    return true;
}

bool InovanceRobotCtrl::PrepareWeldJobHardware(std::string& error)
{
    if (!HasVerifiedWeldJobContract(&error) || !EnsureMotionReady())
    {
        if (error.empty()) { error = GetLastRobotError(); }
        return false;
    }
    int arcDoCfg = 0;
    int currentDaCfg = 0;
    int voltageDaCfg = 0;
    if (!QueryInt("Get_DOCfg " + std::to_string(m_weldArcEnableDo), arcDoCfg)
        || !QueryInt("Get_DACfg " + std::to_string(m_weldCurrentDa), currentDaCfg)
        || !QueryInt("Get_DACfg " + std::to_string(m_weldVoltageDa), voltageDaCfg)
        || arcDoCfg != 1 || currentDaCfg != 1 || voltageDaCfg != 1)
    {
        error = "汇川焊接JOB要求ArcEnableDO、CurrentDA、VoltageDA均由RC控制；现场配置权回读未通过。";
        return false;
    }
    return ConfirmWeldArcOutputOff(error);
}

bool InovanceRobotCtrl::ConfirmWeldArcOutputOff(std::string& error)
{
    if (m_weldArcEnableDo < 0
        || (m_weldArcEnableActiveValue != 0 && m_weldArcEnableActiveValue != 1))
    {
        error = "汇川焊接JOB没有有效的ArcEnableDO关弧映射。";
        return false;
    }
    const int inactiveValue = m_weldArcEnableActiveValue == 0 ? 1 : 0;
    std::string response;
    if (!SendCommand("Set_DO " + std::to_string(m_weldArcEnableDo)
            + " " + std::to_string(inactiveValue), response)
        || response != "ok")
    {
        error = "汇川ArcEnableDO安全关断命令未确认：" + GetLastRobotError();
        return false;
    }
    int stableOff = 0;
    int actual = -1;
    for (int attempt = 0; attempt < 40; ++attempt)
    {
        if (!QueryInt("Get_DO " + std::to_string(m_weldArcEnableDo), actual))
        {
            error = GetLastRobotError();
            return false;
        }
        stableOff = actual == inactiveValue ? stableOff + 1 : 0;
        if (stableOff >= 3)
        {
            error.clear();
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
    error = "汇川ArcEnableDO安全关断后未获得连续三次OFF回读，实际="
        + std::to_string(actual) + "。";
    return false;
}

bool InovanceRobotCtrl::ReserveTrajectory(
    RobotTrajectoryPurpose purpose,
    RobotTrajectoryHandle& handle)
{
    if (purpose == RobotTrajectoryPurpose::ActualWeld)
    {
        std::string contractError;
        if (!HasVerifiedWeldJobContract(&contractError))
        {
            SetLastRobotError(contractError);
            return false;
        }
    }
    std::lock_guard<std::mutex> lock(m_trajectoryMutex);
    if (m_trajectoryRunning.load())
    {
        SetLastRobotError("汇川上一条轨迹仍在运行，禁止覆盖轨迹身份。");
        return false;
    }
    handle = RobotTrajectoryHandle{};
    handle.programName = IsInovanceNativeTrajectoryPurpose(purpose)
        ? std::string(kInovanceManagedTrajectoryModule)
        : "INOVANCE_STREAM_" + std::to_string(++m_trajectoryCounter);
    m_preparedMoveInfos.clear();
    m_preparedPurpose = purpose;
    m_preparedFingerprint = 0;
    m_activeHandle = RobotTrajectoryHandle{};
    m_finalCommandId = -1;
    m_nativeTrajectoryResultCached = false;
    m_nativeTrajectoryCachedResult = NativeTrajectoryResult{};
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
    if (IsInovanceNativeTrajectoryPurpose(purpose))
    {
        const QString auditRoot = AppPaths::WritablePath(
            QStringLiteral("Job/Inovance/Generated"));
        const std::filesystem::path outputDirectory =
            std::filesystem::path(auditRoot.toStdWString())
            / ("job_" + InovancePcTimestamp() + "_" + std::to_string(SteadyMs()));
        if (!WriteTrajectoryJobFile(
                moveInfos, purpose, outputDirectory.string(), handle, error))
        {
            SetLastRobotError("汇川原生轨迹JOB生成失败：" + error);
            return false;
        }
        if (!UploadTrajectoryJob(handle, error))
        {
            SetLastRobotError("汇川原生轨迹JOB下发失败：" + error);
            return false;
        }
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
    m_activeHandle = RobotTrajectoryHandle{};
    m_finalCommandId = -1;
    ClearLastRobotError();
    return true;
}

bool InovanceRobotCtrl::ExportTrajectoryProgramFiles(
    const std::vector<T_ROBOT_MOVE_INFO>& moveInfos,
    RobotTrajectoryPurpose purpose,
    const std::string& outputDirectory,
    RobotTrajectoryHandle& handle,
    std::string* error)
{
    std::string detail;
    if (!WriteTrajectoryJobFile(
            moveInfos, purpose, outputDirectory, handle, detail))
    {
        SetLastRobotError("汇川离线轨迹JOB导出失败：" + detail);
        if (error != nullptr) { *error = detail; }
        return false;
    }
    if (m_pRobotLog != nullptr)
    {
        m_pRobotLog->write(LogColor::SUCCESS,
            "汇川离线轨迹JOB已生成：Program=%s Local=%s SHA256=%s",
            handle.programName.c_str(), handle.localProgramPath.c_str(),
            handle.programContentSha256.c_str());
    }
    ClearLastRobotError();
    if (error != nullptr) { error->clear(); }
    return true;
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
            SetLastRobotError("汇川已有轨迹在运行。");
            return false;
        }
    }

    if (IsInovanceNativeTrajectoryPurpose(purpose))
    {
        if (handle.localProgramPath.empty() || handle.remoteProgramPath.empty()
            || handle.programContentSha256.size() != 64 || handle.programContentSize == 0)
        {
            SetLastRobotError("汇川原生轨迹JOB尚未完成生成、上传和内容身份冻结，禁止启动。");
            return false;
        }
        if (m_nativeTrajectoryFuture.valid())
        {
            SetLastRobotError("汇川上一原生轨迹工作线程尚未收敛，禁止并发启动。");
            return false;
        }
        if (!VerifyTrajectoryJobRemoteIdentity(handle, error))
        {
            SetLastRobotError(error);
            return false;
        }
        if (purpose == RobotTrajectoryPurpose::ActualWeld
            && !PrepareWeldJobHardware(error))
        {
            SetLastRobotError("汇川实际焊接JOB启动前硬件闭环检查失败：" + error);
            return false;
        }

        handle.started = true;
        {
            std::lock_guard<std::mutex> lock(m_trajectoryMutex);
            m_activeHandle = handle;
            m_activeHandle.started = true;
            m_trajectoryRunning.store(true);
            m_trajectoryPaused.store(false);
            m_nativeTrajectoryResultCached = false;
            m_nativeTrajectoryCachedResult = NativeTrajectoryResult{};
        }
        try
        {
            const std::string programName = handle.programName;
            m_nativeTrajectoryFuture = std::async(std::launch::async,
                [this, programName]()
                {
                    NativeTrajectoryResult result;
                    result.success = RunProgramAndWait(
                        programName, 10000, 24 * 60 * 60 * 1000, 100,
                        &result.terminalStatus);
                    if (!result.success)
                    {
                        result.error = result.terminalStatus.detail.empty()
                            ? GetLastRobotError() : result.terminalStatus.detail;
                    }
                    return result;
                });
        }
        catch (const std::exception& exception)
        {
            std::lock_guard<std::mutex> lock(m_trajectoryMutex);
            m_trajectoryRunning.store(false);
            m_activeHandle.started = false;
            handle.started = false;
            SetLastRobotError("汇川原生轨迹JOB工作线程启动失败：" + std::string(exception.what()));
            return false;
        }

        // StartTrajectory只在观察到主任务进入、本次B255进入值或极短JOB自然完成后返回。
        // FTP调度器准备也计入60秒握手上限，避免界面无边界等待。
        const auto handshakeDeadline = std::chrono::steady_clock::now()
            + std::chrono::seconds(60);
        while (std::chrono::steady_clock::now() < handshakeDeadline)
        {
            if (m_nativeTrajectoryFuture.wait_for(std::chrono::milliseconds(0))
                == std::future_status::ready)
            {
                NativeTrajectoryResult result;
                try { result = m_nativeTrajectoryFuture.get(); }
                catch (const std::exception& exception)
                {
                    result.error = exception.what();
                }
                if (!result.success)
                {
                    std::lock_guard<std::mutex> lock(m_trajectoryMutex);
                    m_trajectoryRunning.store(false);
                    m_activeHandle.started = false;
                    handle.started = false;
                    SetLastRobotError("汇川原生轨迹JOB启动失败：" + result.error);
                    return false;
                }
                {
                    std::lock_guard<std::mutex> lock(m_trajectoryMutex);
                    m_nativeTrajectoryResultCached = true;
                    m_nativeTrajectoryCachedResult = result;
                }
                ClearLastRobotError();
                return true;
            }
            if (m_nativeProgramRunning.load())
            {
                int taskStatus = -1;
                int stateByte = -1;
                if (QueryInt("Get_TaskRunSts 0", taskStatus)
                    && TryGetIntVar(kInovanceNativeProgramStateByte, stateByte, "B")
                    && (taskStatus == 1 || stateByte == 1 || stateByte == 10))
                {
                    ClearLastRobotError();
                    return true;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
        }

        RobotOperationLease::RequestCancellation(this);
        const std::string handshakeError =
            "汇川原生轨迹JOB在60秒内未获得任务进入见证，已触发安全中止。";
        AbortCurrentProgramSafely();
        m_nativeTrajectoryFuture.wait();
        try { (void)m_nativeTrajectoryFuture.get(); }
        catch (...) {}
        {
            std::lock_guard<std::mutex> lock(m_trajectoryMutex);
            m_trajectoryRunning.store(false);
            m_activeHandle.started = false;
        }
        handle.started = false;
        SetLastRobotError(handshakeError);
        return false;
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
    bool nativeJob = false;
    {
        std::lock_guard<std::mutex> lock(m_trajectoryMutex);
        nativeJob = IsInovanceNativeTrajectoryPurpose(m_preparedPurpose);
        if (nativeJob
            && (!SameProgramHandle(handle, m_activeHandle) || !m_activeHandle.started))
        {
            SetLastRobotError("汇川WaitTrajectory句柄与当前原生JOB身份不一致。");
            if (terminalStatus != nullptr)
            {
                terminalStatus->state = RobotMotionState::Faulted;
                terminalStatus->detail = GetLastRobotError();
            }
            return false;
        }
    }
    if (nativeJob)
    {
        NativeTrajectoryResult result;
        bool hasResult = false;
        {
            std::lock_guard<std::mutex> lock(m_trajectoryMutex);
            if (m_nativeTrajectoryResultCached)
            {
                result = m_nativeTrajectoryCachedResult;
                m_nativeTrajectoryResultCached = false;
                m_nativeTrajectoryCachedResult = NativeTrajectoryResult{};
                hasResult = true;
            }
        }
        if (!hasResult)
        {
            if (!m_nativeTrajectoryFuture.valid())
            {
                SetLastRobotError("汇川原生轨迹JOB没有可等待的执行工作线程。");
                return false;
            }
            if (pollDelayMs <= 0 || runTimeoutMs <= 0)
            {
                SetLastRobotError("汇川原生轨迹JOB轮询和完成超时必须为正数。");
                return false;
            }
            if (m_nativeTrajectoryFuture.wait_for(std::chrono::milliseconds(runTimeoutMs))
                != std::future_status::ready)
            {
                RobotOperationLease::RequestCancellation(this);
                const std::string timeoutError = "汇川原生轨迹JOB在"
                    + std::to_string(runTimeoutMs) + "ms内未完成，已触发安全中止。";
                const bool abortOk = AbortCurrentProgramSafely();
                m_nativeTrajectoryFuture.wait();
                try { result = m_nativeTrajectoryFuture.get(); }
                catch (const std::exception& exception) { result.error = exception.what(); }
                result.success = false;
                result.error = timeoutError + (abortOk
                    ? "安全停止及关弧输出已确认。"
                    : "安全停止未完整确认：" + GetLastRobotError());
                result.terminalStatus.state = RobotMotionState::Interrupted;
                result.terminalStatus.terminalVerified = abortOk;
                result.terminalStatus.detail = result.error;
            }
            else
            {
                try { result = m_nativeTrajectoryFuture.get(); }
                catch (const std::exception& exception)
                {
                    result.success = false;
                    result.error = exception.what();
                }
            }
        }

        if (result.success && m_preparedPurpose == RobotTrajectoryPurpose::ActualWeld)
        {
            std::string arcOffError;
            if (!ConfirmWeldArcOutputOff(arcOffError))
            {
                result.success = false;
                result.error = "汇川原生焊接JOB已结束，但ArcEnableDO最终关断见证失败："
                    + arcOffError;
                result.terminalStatus.state = RobotMotionState::Faulted;
                result.terminalStatus.terminalVerified = false;
                result.terminalStatus.detail = result.error;
            }
        }
        {
            std::lock_guard<std::mutex> lock(m_trajectoryMutex);
            m_trajectoryRunning.store(false);
            m_trajectoryPaused.store(false);
            m_activeHandle.started = false;
        }
        if (terminalStatus != nullptr) { *terminalStatus = result.terminalStatus; }
        if (!result.success)
        {
            SetLastRobotError(result.error.empty()
                ? std::string("汇川原生轨迹JOB未获得自然完成见证。") : result.error);
            return false;
        }
        ClearLastRobotError();
        return true;
    }

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
        SetLastRobotError("汇川当前没有由本适配层启动的轨迹身份。");
        return false;
    }
    projectName = IsInovanceNativeTrajectoryPurpose(m_preparedPurpose)
        ? "NATIVE_JOB" : "DATA_STREAM";
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
        if (IsInovanceNativeTrajectoryPurpose(m_preparedPurpose))
        {
            SetLastRobotError("汇川2222协议只提供Prg Start/Stop，没有原生程序暂停/续行命令；"
                "焊接JOB运行中禁止伪造PauseResume，安全停止后只能重新走焊接授权流程。 ");
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
        if (IsInovanceNativeTrajectoryPurpose(m_preparedPurpose))
        {
            SetLastRobotError("汇川原生焊接JOB不支持通过2222协议从中断行续行；"
                "必须确认关弧和静止后重新生成并审核剩余焊道。");
            return false;
        }
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
    int taskStatus = -1;
    if (!QueryInt("Get_DsMode", dataStreamMode)
        || !QueryInt("Get_MotionSts", motion)
        || !QueryInt("Get_TaskRunSts 0", taskStatus))
    {
        return false;
    }

    const bool nativeProgramTracked = m_nativeProgramRunning.load() || taskStatus == 1;
    if (dataStreamMode == 0 && (motion == 1 || taskStatus == 1))
    {
        std::string response;
        if (!SendCommand("Prg Stop", response) || response != "ok")
        {
            SetLastRobotError("汇川原生程序Prg Stop未被控制器确认：" + GetLastRobotError());
            return false;
        }
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
    for (int attempt = 0; attempt < 120; ++attempt)
    {
        if (!QueryInt("Get_TaskRunSts 0", taskStatus)
            || !QueryInt("Get_MotionSts", motion))
        {
            return false;
        }
        stableStopped = (taskStatus != 1 && motion != 1) ? stableStopped + 1 : 0;
        if (stableStopped >= 3) { break; }
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
    if (stableStopped < 3)
    {
        SetLastRobotError("汇川Prg Stop后未获得任务非运行且机器人非运动的连续三次见证。");
        return false;
    }
    if (m_weldJobEnabled && m_weldArcEnableDo >= 0)
    {
        std::string arcOffError;
        if (!ConfirmWeldArcOutputOff(arcOffError))
        {
            SetLastRobotError("汇川安全中止已停止运动，但焊机关弧输出未确认：" + arcOffError);
            return false;
        }
    }
    if (nativeProgramTracked)
    {
        std::string response;
        if (!SendCommand("BackStartLine", response) || response != "ok")
        {
            SetLastRobotError("汇川原生程序停止后无法回到起始行，禁止解除安全互锁。");
            return false;
        }
    }

    stableStopped = 0;
    for (int attempt = 0; attempt < 20; ++attempt)
    {
        int mode = -1;
        if (!QueryInt("Get_DsMode", mode)
            || !QueryInt("Get_MotionSts", motion)
            || !QueryInt("Get_TaskRunSts 0", taskStatus))
        {
            return false;
        }
        stableStopped = (mode == 0 && motion == 0 && taskStatus != 1)
            ? stableStopped + 1 : 0;
        if (stableStopped >= 3) { break; }
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
    if (stableStopped < 3)
    {
        SetLastRobotError("汇川安全中止后未获得Get_DsMode=0、Get_MotionSts=0且主任务非运行的稳定终态。");
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(m_trajectoryMutex);
        m_trajectoryRunning.store(false);
        m_trajectoryPaused.store(false);
        m_activeHandle.started = false;
    }
    m_continuousJogRunning.store(false);
    m_nativeProgramRunning.store(false);
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
    if (terminalStatus != nullptr) { *terminalStatus = RobotMotionStatus{}; }
    std::unique_lock<std::mutex> nativeLock(m_nativeProgramMutex, std::try_to_lock);
    const auto failWithoutMotion = [this, terminalStatus](
        const std::string& reason,
        int rawState = -1,
        RobotMotionState state = RobotMotionState::Faulted)
        {
            SetLastRobotError(reason);
            if (terminalStatus != nullptr)
            {
                terminalStatus->state = state;
                terminalStatus->rawCode = rawState;
                terminalStatus->terminalVerified = false;
                terminalStatus->detail = reason;
            }
            return false;
        };
    if (!nativeLock.owns_lock())
    {
        return failWithoutMotion("汇川已有原生程序调度正在执行，禁止并发覆盖 main.pro。");
    }
    if (startTimeoutMs <= 0 || finishTimeoutMs <= 0 || pollDelayMs <= 0)
    {
        return failWithoutMotion("汇川原生程序启动、完成和轮询超时必须为正数。");
    }
    pollDelayMs = std::clamp(pollDelayMs, 20, 1000);

    std::string requestedProject;
    std::string requestedModule;
    std::string parseError;
    if (!ParseInovanceProgramRequest(
        programName, requestedProject, requestedModule, parseError))
    {
        return failWithoutMotion("汇川原生程序身份无效：" + parseError);
    }
    if (RobotOperationLease::MotionCompletionPending(this))
    {
        return failWithoutMotion("汇川上一项运动仍缺少可验证终态，禁止覆盖并启动新的原生程序。");
    }
    if (RobotOperationLease::IsCancellationRequested(this))
    {
        return failWithoutMotion("汇川原生程序执行已由安全停止取消，未更新 main.pro。",
            -1, RobotMotionState::Interrupted);
    }
    if (!IsConnected() || !EnsureMotionReady())
    {
        return failWithoutMotion("汇川原生程序执行前安全条件未通过：" + GetLastRobotError());
    }

    int dataStreamMode = -1;
    int taskStatus = -1;
    if (!QueryInt("Get_DsMode", dataStreamMode)
        || !QueryInt("Get_TaskRunSts 0", taskStatus))
    {
        return failWithoutMotion("汇川原生程序执行前状态回读失败：" + GetLastRobotError());
    }
    if (dataStreamMode != 0 || taskStatus == 1)
    {
        return failWithoutMotion("汇川数据流或主任务仍在运行，禁止更新 main.pro。", taskStatus);
    }
    if (taskStatus != 0 && taskStatus != 10)
    {
        return failWithoutMotion("汇川主任务未处于停止/就绪状态，Get_TaskRunSts 0="
            + std::to_string(taskStatus) + "。", taskStatus);
    }

    std::string taskPathResponse;
    std::string activeDirectory;
    std::string activeProject;
    std::string pathError;
    if (!SendCommand("Get_TaskPrgPath 0", taskPathResponse)
        || !InovanceActiveMainProgram(
            ValuePart(taskPathResponse), activeDirectory, activeProject, pathError))
    {
        return failWithoutMotion("汇川无法确认当前激活工程的固定 main.pro："
            + (pathError.empty() ? GetLastRobotError() : pathError));
    }
    if (!requestedProject.empty()
        && LowerAscii(requestedProject) != LowerAscii(activeProject))
    {
        return failWithoutMotion("汇川程序身份指定工程 " + requestedProject
            + "，但当前激活工程是 " + activeProject + "；本方案禁止自动切换工程。");
    }

    std::string sessionError;
    const std::shared_ptr<RobotFileTransferSession> session =
        CreateFileTransferSession(&sessionError);
    if (session == nullptr)
    {
        return failWithoutMotion("汇川原生程序执行无法建立FTP底层：" + sessionError);
    }
    std::vector<RobotControllerFileInfo> entries;
    if (!session->ListProgramFiles(activeDirectory, entries, 10000))
    {
        return failWithoutMotion("汇川无法读取当前工程程序清单：" + session->LastError());
    }
    int programFileCount = 0;
    int requestedMatches = 0;
    bool mainFound = false;
    std::string actualModuleFile;
    const std::string requestedFileLower = LowerAscii(requestedModule + ".pro");
    for (const RobotControllerFileInfo& entry : entries)
    {
        if (entry.isDirectory || entry.name.empty()) { continue; }
        const std::string lowerName = LowerAscii(entry.name);
        if (lowerName.size() < 4 || lowerName.substr(lowerName.size() - 4) != ".pro")
        {
            continue;
        }
        ++programFileCount;
        if (lowerName == "main.pro") { mainFound = true; }
        if (lowerName == requestedFileLower)
        {
            ++requestedMatches;
            actualModuleFile = entry.name;
        }
    }
    if (programFileCount > kInovanceProgramFileLimit)
    {
        return failWithoutMotion("汇川当前工程包含 " + std::to_string(programFileCount)
            + " 个PRO文件，超过控制器16个程序文件上限。");
    }
    if (!mainFound || requestedMatches != 1
        || actualModuleFile.find('/') != std::string::npos
        || actualModuleFile.find('\\') != std::string::npos)
    {
        return failWithoutMotion("汇川当前工程必须恰好包含目标公共模块 "
            + requestedModule + ".pro 和固定 main.pro；当前匹配数="
            + std::to_string(requestedMatches) + "。");
    }
    const std::string actualModuleName = actualModuleFile.substr(
        0, actualModuleFile.size() - std::strlen(".pro"));
    if (!IsInovanceProgramIdentifier(actualModuleName))
    {
        return failWithoutMotion("汇川FTP返回的目标模块文件名不符合控制器命名规范。");
    }

    const long long runStamp = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    const QString auditRoot = AppPaths::WritablePath(
        QStringLiteral("Job/Inovance/DispatcherRuns"));
    const std::filesystem::path runDirectory = std::filesystem::path(auditRoot.toStdWString())
        / ("run_" + std::to_string(runStamp) + "_" + std::to_string(SteadyMs())
            + "_" + LowerAscii(actualModuleName));
    const std::filesystem::path backupMainPath = runDirectory / "main_before.pro";
    const std::filesystem::path moduleCopyPath = runDirectory / actualModuleFile;
    const std::filesystem::path dispatcherPath = runDirectory / "main.pro";
    const std::filesystem::path verifiedDispatcherPath = runDirectory / "main_uploaded_verify.pro";
    const std::filesystem::path restoredMainVerifyPath = runDirectory / "main_restored_verify.pro";
    std::error_code directoryError;
    std::filesystem::create_directories(runDirectory, directoryError);
    if (directoryError)
    {
        return failWithoutMotion("汇川无法创建调度审计目录："
            + runDirectory.string() + "。");
    }

    const std::string remoteMainPath = activeDirectory + "/main.pro";
    const std::string remoteModulePath = activeDirectory + "/" + actualModuleFile;
    if (!session->DownloadProgramFile(remoteMainPath, backupMainPath.string()))
    {
        return failWithoutMotion("汇川覆盖 main.pro 前无法建立本地备份："
            + session->LastError());
    }
    std::string backupMainContent;
    std::string backupError;
    if (!ReadBoundedTextFile(backupMainPath, backupMainContent, backupError))
    {
        return failWithoutMotion("汇川覆盖 main.pro 前的本地备份无效：" + backupError);
    }
    if (!session->DownloadProgramFile(remoteModulePath, moduleCopyPath.string()))
    {
        return failWithoutMotion("汇川无法下载并校验目标公共模块：" + session->LastError());
    }
    std::string moduleContent;
    std::string moduleError;
    if (!ReadBoundedTextFile(moduleCopyPath, moduleContent, moduleError)
        || !ValidateInovanceCallableModule(moduleContent, moduleError))
    {
        return failWithoutMotion("汇川目标模块 " + actualModuleFile
            + " 不满足适配层公共模块契约：" + moduleError);
    }
    std::string expectedTrajectorySha256;
    std::uint64_t expectedTrajectorySize = 0;
    {
        std::lock_guard<std::mutex> lock(m_trajectoryMutex);
        if (m_activeHandle.started
            && LowerAscii(m_activeHandle.programName) == LowerAscii(requestedModule))
        {
            expectedTrajectorySha256 = m_activeHandle.programContentSha256;
            expectedTrajectorySize = m_activeHandle.programContentSize;
        }
    }
    if (!expectedTrajectorySha256.empty()
        && (moduleContent.size() != expectedTrajectorySize
            || InovanceContentSha256(moduleContent) != expectedTrajectorySha256))
    {
        return failWithoutMotion("汇川原生轨迹模块在StartTrajectory冻结后发生变化，"
            "远端PRO的SHA-256或大小与句柄不一致。");
    }
    if (RobotOperationLease::IsCancellationRequested(this))
    {
        return failWithoutMotion("汇川原生程序调度准备期间已收到安全停止，未覆盖main.pro。",
            taskStatus, RobotMotionState::Interrupted);
    }

    // B255 先由PC写0并回读，随后只能由本次调度器写1和10，排除上次运行遗留的伪完成值。
    if (!SetIntVar(kInovanceNativeProgramStateByte, 0, 2, "B"))
    {
        return failWithoutMotion("汇川原生程序状态字节B255初始化失败："
            + GetLastRobotError());
    }
    std::string dispatcherContent;
    std::string dispatcherError;
    if (!WriteInovanceDispatcher(
        dispatcherPath, actualModuleName, dispatcherContent, dispatcherError))
    {
        return failWithoutMotion("汇川生成 main.pro 调度器失败：" + dispatcherError);
    }

    bool dispatcherInstalled = false;
    bool mainRestoreVerified = true;
    const auto restoreOriginalMain = [&]()
        {
            if (!dispatcherInstalled) { return std::string(); }
            if (!session->UploadProgramFile(backupMainPath.string(), remoteMainPath, true))
            {
                mainRestoreVerified = false;
                return std::string("；原main.pro自动恢复失败：") + session->LastError()
                    + "，备份位于 " + backupMainPath.string();
            }
            if (!session->DownloadProgramFile(remoteMainPath, restoredMainVerifyPath.string()))
            {
                mainRestoreVerified = false;
                return std::string("；原main.pro已回传但回读验证失败：")
                    + session->LastError() + "，备份位于 " + backupMainPath.string();
            }
            std::string restoredContent;
            std::string restoredError;
            if (!ReadBoundedTextFile(restoredMainVerifyPath, restoredContent, restoredError)
                || restoredContent != backupMainContent)
            {
                mainRestoreVerified = false;
                return std::string("；原main.pro恢复后的字节身份未确认：")
                    + (restoredError.empty() ? "内容不一致。" : restoredError)
                    + "，备份位于 " + backupMainPath.string();
            }
            dispatcherInstalled = false;
            mainRestoreVerified = true;
            return std::string("；原main.pro已由本地备份恢复并完成字节回读确认。");
        };
    if (!session->UploadProgramFile(dispatcherPath.string(), remoteMainPath, true))
    {
        return failWithoutMotion("汇川 main.pro 调度器上传失败：" + session->LastError());
    }
    dispatcherInstalled = true;
    if (!session->DownloadProgramFile(remoteMainPath, verifiedDispatcherPath.string()))
    {
        const std::string downloadError = session->LastError();
        const std::string restore = restoreOriginalMain();
        return failWithoutMotion("汇川 main.pro 上传后无法回读验证："
            + downloadError + restore);
    }
    std::string verifiedDispatcher;
    std::string verifyError;
    if (!ReadBoundedTextFile(verifiedDispatcherPath, verifiedDispatcher, verifyError)
        || verifiedDispatcher != dispatcherContent)
    {
        const std::string restore = restoreOriginalMain();
        return failWithoutMotion("汇川 main.pro 上传后内容身份不一致："
            + (verifyError.empty() ? std::string("字节内容不一致。") : verifyError) + restore);
    }

    std::string response;
    if (!SendCommand("BackStartLine", response) || response != "ok")
    {
        const std::string error = GetLastRobotError();
        const std::string restore = restoreOriginalMain();
        return failWithoutMotion("汇川调度器上传后无法回到程序起始行：" + error + restore);
    }
    if (RobotOperationLease::IsCancellationRequested(this))
    {
        const std::string restore = restoreOriginalMain();
        return failWithoutMotion("汇川Prg Start前已收到安全停止，未启动原生程序。" + restore,
            taskStatus, RobotMotionState::Interrupted);
    }
    int beforeLine = -1;
    if (!QueryInt("Get_TaskRunSts 0", taskStatus)
        || !QueryInt("Get_TaskProgramLine 0", beforeLine)
        || taskStatus == 1)
    {
        const std::string error = GetLastRobotError();
        const std::string restore = restoreOriginalMain();
        return failWithoutMotion("汇川Prg Start前任务状态/行号复核失败：" + error + restore,
            taskStatus);
    }
    std::string verifyTaskPathResponse;
    std::string verifyDirectory;
    std::string verifyProject;
    std::string verifyPathError;
    if (!SendCommand("Get_TaskPrgPath 0", verifyTaskPathResponse)
        || !InovanceActiveMainProgram(ValuePart(verifyTaskPathResponse),
            verifyDirectory, verifyProject, verifyPathError)
        || LowerAscii(verifyDirectory) != LowerAscii(activeDirectory)
        || LowerAscii(verifyProject) != LowerAscii(activeProject))
    {
        const std::string restore = restoreOriginalMain();
        return failWithoutMotion("汇川Prg Start前激活工程身份发生变化："
            + (verifyPathError.empty() ? GetLastRobotError() : verifyPathError) + restore);
    }

    bool motionMarked = false;
    const auto failRun = [this, terminalStatus, &motionMarked,
        &restoreOriginalMain, &mainRestoreVerified](
        const std::string& reason,
        int rawState,
        RobotMotionState state = RobotMotionState::Faulted)
        {
            std::string detail = reason;
            bool stopped = !motionMarked;
            if (motionMarked)
            {
                stopped = RobotOperationLease::StopAndConfirmUnverifiedMotion(this);
                const std::string stopError = GetLastRobotError();
                detail += stopped
                    ? "；已Prg Stop、BackStartLine并稳定确认任务与机器人停止。"
                    : "；原生程序安全停止未确认：" + stopError;
            }
            if (stopped)
            {
                detail += restoreOriginalMain();
                if (!mainRestoreVerified)
                {
                    detail += "；原main.pro未恢复，禁止再次启动原生程序。";
                }
            }
            else
            {
                detail += "；未取得停止见证，未覆盖当前main.pro；本地备份保留。";
            }
            m_nativeProgramRunning.store(false);
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
    QString motionError;
    if (!RobotOperationLease::MarkMotionStarted(this, false, &motionError))
    {
        const std::string restore = restoreOriginalMain();
        return failWithoutMotion("汇川原生程序START登记失败："
            + motionError.toStdString() + restore);
    }
    motionMarked = true;
    m_nativeProgramRunning.store(true);
    if (!SendCommand("Prg Start", response) || response != "ok")
    {
        return failRun("汇川Prg Start失败或结果未知：" + GetLastRobotError(), taskStatus);
    }

    struct ProgramSnapshot
    {
        int task = -1;
        int stateByte = -1;
        int fault = -1;
        int motion = -1;
        int line = -1;
    };
    const auto readSnapshot = [this, &activeDirectory, &activeProject](
        ProgramSnapshot& snapshot)
        {
            if (!QueryInt("Get_TaskRunSts 0", snapshot.task)
                || !TryGetIntVar(kInovanceNativeProgramStateByte, snapshot.stateByte, "B")
                || !QueryInt("Get_SysErrSts", snapshot.fault)
                || !QueryInt("Get_MotionSts", snapshot.motion)
                || !QueryInt("Get_TaskProgramLine 0", snapshot.line))
            {
                return false;
            }
            if ((snapshot.task != 0 && snapshot.task != 1 && snapshot.task != 10)
                || (snapshot.stateByte != 0 && snapshot.stateByte != 1
                    && snapshot.stateByte != 10)
                || snapshot.motion < 0 || snapshot.motion > 2
                || snapshot.motion == 2)
            {
                SetLastRobotError("汇川原生程序返回未允许的任务/状态字节/运动状态：Task="
                    + std::to_string(snapshot.task) + " B255="
                    + std::to_string(snapshot.stateByte) + " Motion="
                    + std::to_string(snapshot.motion) + "。");
                return false;
            }
            std::string pathResponse;
            std::string directory;
            std::string project;
            std::string error;
            if (!SendCommand("Get_TaskPrgPath 0", pathResponse)
                || !InovanceActiveMainProgram(
                    ValuePart(pathResponse), directory, project, error)
                || LowerAscii(directory) != LowerAscii(activeDirectory)
                || LowerAscii(project) != LowerAscii(activeProject))
            {
                SetLastRobotError(error.empty()
                    ? "汇川原生程序执行期间激活工程身份发生变化。" : error);
                return false;
            }
            return true;
        };
    const auto completeRun = [this, terminalStatus, &motionMarked,
        &activeProject, &actualModuleName, &backupMainPath](const ProgramSnapshot& snapshot)
        {
            if (!RobotOperationLease::MarkMotionCompleted(this))
            {
                SetLastRobotError("汇川原生程序已自然完成，但适配层未能解除运动完成待确认状态。");
                return false;
            }
            motionMarked = false;
            m_nativeProgramRunning.store(false);
            const std::string detail = "汇川原生程序自然完成：Project=" + activeProject
                + " Module=" + actualModuleName + ".pro Line="
                + std::to_string(snapshot.line)
                + "；B255=10且任务/运动连续稳定停止；原main已恢复，备份="
                + backupMainPath.string();
            if (m_pRobotLog != nullptr)
            {
                m_pRobotLog->write(LogColor::SUCCESS, "%s", detail.c_str());
            }
            ClearLastRobotError();
            if (terminalStatus != nullptr)
            {
                terminalStatus->state = RobotMotionState::Completed;
                terminalStatus->rawCode = snapshot.task;
                terminalStatus->terminalVerified = true;
                terminalStatus->detail = detail;
            }
            return true;
        };

    bool executionObserved = false;
    int stableCompleted = 0;
    int stoppedWithoutCompletion = 0;
    ProgramSnapshot snapshot;
    const auto startDeadline = std::chrono::steady_clock::now()
        + std::chrono::milliseconds(startTimeoutMs);
    while (std::chrono::steady_clock::now() < startDeadline)
    {
        if (RobotOperationLease::IsCancellationRequested(this))
        {
            return failRun("汇川原生程序启动等待已由安全停止取消。",
                snapshot.task, RobotMotionState::Interrupted);
        }
        if (!readSnapshot(snapshot))
        {
            return failRun("汇川原生程序启动状态回读失败：" + GetLastRobotError(), snapshot.task);
        }
        if (snapshot.fault != 0)
        {
            return failRun("汇川控制器在原生程序启动期间报告故障，Get_SysErrSts="
                + std::to_string(snapshot.fault) + "。", snapshot.task);
        }
        executionObserved = executionObserved || snapshot.task == 1
            || snapshot.stateByte == 1 || snapshot.stateByte == 10
            || snapshot.line != beforeLine;
        stableCompleted = (snapshot.stateByte == 10
            && snapshot.task != 1 && snapshot.motion != 1) ? stableCompleted + 1 : 0;
        if (stableCompleted >= 3)
        {
            const std::string restore = restoreOriginalMain();
            if (!mainRestoreVerified)
            {
                return failRun("汇川原生程序已自然完成，但原main.pro恢复未通过验证"
                    + restore, snapshot.task);
            }
            if (!completeRun(snapshot))
            {
                return failRun(GetLastRobotError(), snapshot.task);
            }
            return true;
        }
        stoppedWithoutCompletion = (executionObserved && snapshot.task != 1
            && snapshot.stateByte != 10) ? stoppedWithoutCompletion + 1 : 0;
        if (stoppedWithoutCompletion >= 3)
        {
            return failRun("汇川原生程序已停止，但未写入B255=10自然完成见证；"
                "请在示教器查看SRD/PRO语法或运行错误。", snapshot.task);
        }
        if (executionObserved) { break; }
        std::this_thread::sleep_for(std::chrono::milliseconds(pollDelayMs));
    }
    if (!executionObserved)
    {
        return failRun("汇川原生程序在 " + std::to_string(startTimeoutMs)
            + "ms 内未观察到任务运行、B255进入值或程序行进展。", snapshot.task);
    }

    stableCompleted = 0;
    stoppedWithoutCompletion = 0;
    const auto finishDeadline = std::chrono::steady_clock::now()
        + std::chrono::milliseconds(finishTimeoutMs);
    while (std::chrono::steady_clock::now() < finishDeadline)
    {
        if (RobotOperationLease::IsCancellationRequested(this))
        {
            return failRun("汇川原生程序完成等待已由安全停止取消。",
                snapshot.task, RobotMotionState::Interrupted);
        }
        if (!readSnapshot(snapshot))
        {
            return failRun("汇川原生程序完成状态回读失败：" + GetLastRobotError(), snapshot.task);
        }
        if (snapshot.fault != 0)
        {
            return failRun("汇川控制器在原生程序运行期间报告故障，Get_SysErrSts="
                + std::to_string(snapshot.fault) + "。", snapshot.task);
        }
        stableCompleted = (snapshot.stateByte == 10
            && snapshot.task != 1 && snapshot.motion != 1) ? stableCompleted + 1 : 0;
        if (stableCompleted >= 3)
        {
            const std::string restore = restoreOriginalMain();
            if (!mainRestoreVerified)
            {
                return failRun("汇川原生程序已自然完成，但原main.pro恢复未通过验证"
                    + restore, snapshot.task);
            }
            if (!completeRun(snapshot))
            {
                return failRun(GetLastRobotError(), snapshot.task);
            }
            return true;
        }
        stoppedWithoutCompletion = (snapshot.task != 1 && snapshot.stateByte != 10)
            ? stoppedWithoutCompletion + 1 : 0;
        if (stoppedWithoutCompletion >= 3)
        {
            return failRun("汇川原生程序已停止，但B255未达到10；"
                "示教器会保留具体程序错误，适配层拒绝把停止误报为完成。", snapshot.task);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(pollDelayMs));
    }
    return failRun("汇川原生程序在 " + std::to_string(finishTimeoutMs)
        + "ms 内未获得B255=10和任务/运动稳定停止见证。",
        snapshot.task, RobotMotionState::Interrupted);
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
        SetLastRobotError("汇川整数/字节变量序号范围为0..255。");
        return false;
    }
    const std::string type = LowerAscii(Trim(prefix));
    if (type == "int" || type == "r")
    {
        return QueryInt("Get_R " + std::to_string(index), value);
    }
    if (type == "b" || type == "byte")
    {
        return QueryInt("Get_B " + std::to_string(index), value);
    }
    if (type == "dint" || type == "plc_dint")
    {
        return QueryInt("Get_PlcVar DInt " + std::to_string(index), value);
    }
    SetLastRobotError("汇川整数读取只支持全局R、全局B或只读PLC_DINT变量。");
    return false;
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
    (void)scope;
    if (index < 0 || index > 255 || prefix == nullptr)
    {
        SetLastRobotError("汇川整数/字节变量序号范围为0..255。");
        return false;
    }
    const std::string type = LowerAscii(Trim(prefix));
    std::string command;
    const char* verifyPrefix = nullptr;
    if (type == "int" || type == "r")
    {
        if (value == std::numeric_limits<int>::min())
        {
            SetLastRobotError("汇川全局R变量最小值为-2147483647。");
            return false;
        }
        command = "Set_R " + std::to_string(index) + " " + std::to_string(value);
        verifyPrefix = "R";
    }
    else if (type == "b" || type == "byte")
    {
        if (value < 0 || value > 255)
        {
            SetLastRobotError("汇川全局B变量值范围为0..255。");
            return false;
        }
        command = "Set_B " + std::to_string(index) + " " + std::to_string(value);
        verifyPrefix = "B";
    }
    else if (type == "dint" || type == "plc_dint")
    {
        SetLastRobotError("汇川远程以太网手册只提供PLC_DINT读取；写入请使用全局R变量。");
        return false;
    }
    else
    {
        SetLastRobotError("汇川整数写入只支持全局R或全局B变量。");
        return false;
    }

    std::string response;
    if (!SendCommand(command, response) || response != "ok")
    {
        return false;
    }
    int verified = 0;
    if (!TryGetIntVar(index, verified, verifyPrefix) || verified != value)
    {
        SetLastRobotError("汇川整数变量写入后回读不一致：Index="
            + std::to_string(index) + " Expected=" + std::to_string(value)
            + " Actual=" + std::to_string(verified) + "。");
        return false;
    }
    ClearLastRobotError();
    return true;
}

bool InovanceRobotCtrl::SetIntVar(const char* name, int value, int scope)
{
    if (name == nullptr)
    {
        SetLastRobotError("汇川整数变量名为空。");
        return false;
    }
    std::string prefix;
    int index = -1;
    if (!ParseInovanceIndexedVariableName(name, prefix, index))
    {
        SetLastRobotError("汇川整数变量名只支持 R[0]、R0、INT[0]、B[0] 或 BYTE[0] 格式。");
        return false;
    }
    return SetIntVar(index, value, scope, prefix.c_str());
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
    const std::string message = "汇川原生程序上传与同工程模块执行已接入，但当前没有经验证的汇川手眼辅助PRO模块、输入输出寄存器和完成契约。";
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
