#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>

#include "AppPaths.h"
#include "FTPClient.h"
#include "InovanceRobotDriver.h"
#include "InovanceUserLogin.h"
#include "RobotDriverRegistry.h"
#include "RobotFtpFileTransfer.h"
#include "RobotOperationLease.h"
#include "ConfigDatabase.h"

#include <QCryptographicHash>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QFile>
#include <QTemporaryDir>
#include <QDateTime>

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
#include <set>
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
constexpr const char* kInovanceCallableFunction = "func1";
constexpr int kInovanceArcDataIndex = 0;
constexpr int kInovanceWeaveDataIndex = 0;
constexpr int kInovanceRpmIndex = 0;
// B255 为汇川型号底层保留的原生程序状态字节：0=待启动、1=已进入、10=自然完成。
// 使用 B 而不是 R，避免原生程序执行依赖编辑级登录；通用整数寄存器仍映射到 R。
constexpr int kInovanceNativeProgramStateByte = 255;
constexpr std::size_t kMaxNativeProgramBytes = 4U * 1024U * 1024U;
constexpr const char* kInovanceMachineParametersPath =
    "/RobotParams/MachineParams.json";
constexpr std::uint64_t kMaxInovanceMachineParametersBytes = 1024U * 1024U;

bool JsonDoubleArray(
    const QJsonObject& object,
    const char* key,
    int minimumSize,
    std::vector<double>& values)
{
    values.clear();
    const QJsonArray array = object.value(QString::fromLatin1(key)).toArray();
    if (array.size() < minimumSize) { return false; }
    values.reserve(static_cast<std::size_t>(array.size()));
    for (const QJsonValue& value : array)
    {
        if (!value.isDouble() || !std::isfinite(value.toDouble())) { return false; }
        values.push_back(value.toDouble());
    }
    return true;
}

bool NearlyEqualArray(
    const std::vector<double>& protocol,
    const std::vector<double>& file,
    int count,
    double tolerance)
{
    if (protocol.size() < static_cast<std::size_t>(count)
        || file.size() < static_cast<std::size_t>(count)) { return false; }
    for (int index = 0; index < count; ++index)
    {
        if (std::abs(protocol[index] - file[index]) > tolerance) { return false; }
    }
    return true;
}

KDL::Frame InovancePoseFrame(const T_ROBOT_COORS& pose)
{
    return KDL::Frame(
        KDL::Rotation::RPY(
            pose.dRX * M_PI / 180.0,
            pose.dRY * M_PI / 180.0,
            pose.dRZ * M_PI / 180.0),
        KDL::Vector(pose.dX / 1000.0, pose.dY / 1000.0, pose.dZ / 1000.0));
}

T_ROBOT_COORS InovanceFramePose(const KDL::Frame& frame)
{
    T_ROBOT_COORS pose;
    pose.dX = frame.p.x() * 1000.0;
    pose.dY = frame.p.y() * 1000.0;
    pose.dZ = frame.p.z() * 1000.0;
    double rx = 0.0;
    double ry = 0.0;
    double rz = 0.0;
    frame.M.GetRPY(rx, ry, rz);
    pose.dRX = rx * 180.0 / M_PI;
    pose.dRY = ry * 180.0 / M_PI;
    pose.dRZ = rz * 180.0 / M_PI;
    return pose;
}

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

std::string FormatProgramNumber(double value)
{
    std::string text = FormatProgramDouble(value);
    while (text.size() > 1 && text.back() == '0') { text.pop_back(); }
    if (!text.empty() && text.back() == '.') { text.pop_back(); }
    return text == "-0" ? std::string("0") : text;
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

std::string InovancePcProgramTimestamp()
{
    const auto now = std::chrono::system_clock::now();
    const std::time_t current = std::chrono::system_clock::to_time_t(now);
    std::tm localTime{};
    localtime_s(&localTime, &current);
    const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % std::chrono::seconds(1);
    std::ostringstream stamp;
    stamp << std::put_time(&localTime, "%Y-%m-%d %H:%M:%S") << '.'
        << std::setw(3) << std::setfill('0') << milliseconds.count();
    return stamp.str();
}

std::string InovanceProgramInfo(const std::string& robotName)
{
    std::string safeRobotName = robotName;
    std::replace_if(safeRobotName.begin(), safeRobotName.end(), [](unsigned char ch)
        { return ch < 0x20 || ch == '"' || ch == '\\'; }, '_');
    std::ostringstream source;
    source << "ProgramInfo\r\n"
        << "    Version = \"S4.24\"\r\n"
        << "    VRC = \"V4R24C1\"\r\n"
        << "    Model = \"\"\r\n"
        << "    Time = \"" << InovancePcProgramTimestamp() << "\"\r\n"
        << "    RobotName = \"" << safeRobotName << "\"\r\n"
        << "EndProgramInfo\r\n";
    return source.str();
}

bool ParseInovanceProgramRobotName(
    const std::string& content,
    std::string& robotName,
    std::string& error)
{
    robotName.clear();
    const std::regex declaration(
        R"((^|[\r\n])[\t ]*RobotName[\t ]*=[\t ]*\"([^\"\r\n]+)\"[\t ]*)",
        std::regex_constants::icase);
    std::sregex_iterator match(content.begin(), content.end(), declaration);
    const std::sregex_iterator end;
    if (match == end)
    {
        error = "PRO的ProgramInfo缺少RobotName。";
        return false;
    }
    robotName = Trim((*match)[2].str());
    ++match;
    if (match != end)
    {
        error = "PRO包含多个RobotName声明。";
        robotName.clear();
        return false;
    }
    if (robotName.empty() || robotName.size() > 128)
    {
        error = "PRO的RobotName为空或超过128字节。";
        robotName.clear();
        return false;
    }
    error.clear();
    return true;
}

bool IsInovanceNativeTrajectoryPurpose(RobotTrajectoryPurpose purpose)
{
    return purpose == RobotTrajectoryPurpose::WeldDryRun
        || purpose == RobotTrajectoryPurpose::ActualWeld;
}

bool InovanceNativeWeaveShape(int commonShape, int& controllerShape)
{
    switch (static_cast<EWeaveShape>(commonShape))
    {
    case EWeaveShape::eSin:
    case EWeaveShape::eSinFreq:
        controllerShape = 1;
        return true;
    default:
        controllerShape = -1;
        return false;
    }
}

bool SameProgramValue(double left, double right)
{
    return std::abs(left - right) <= 1e-9;
}

bool SameInovanceWeldParameters(
    const T_ROBOT_MOVE_INFO& left,
    const T_ROBOT_MOVE_INFO& right)
{
    return SameProgramValue(left.dWeldCurrent, right.dWeldCurrent)
        && SameProgramValue(left.dWeldVoltage, right.dWeldVoltage)
        && SameProgramValue(left.dWeldSpeedMmPerMin, right.dWeldSpeedMmPerMin);
}

bool SameInovanceWeaveParameters(const T_WeaveDate& left, const T_WeaveDate& right)
{
    return left.nWeaveType == right.nWeaveType
        && left.nWeaveShape == right.nWeaveShape
        && SameProgramValue(left.dWeaveFrequencyHz, right.dWeaveFrequencyHz)
        && SameProgramValue(left.dWeaveAmplitudeMm, right.dWeaveAmplitudeMm)
        && left.nPauseTime1Ms == right.nPauseTime1Ms
        && left.nPauseTime2Ms == right.nPauseTime2Ms;
}

void AppendInovanceWeaveCommand(
    std::ostringstream& source,
    const char* command,
    const T_WeaveDate& weave)
{
    int shape = 1;
    (void)InovanceNativeWeaveShape(weave.nWeaveShape, shape);
    source << command << " WeaveData[" << kInovanceWeaveDataIndex << "],Shape["
        << shape << "],Freq[" << FormatProgramNumber(weave.dWeaveFrequencyHz)
        << "],RAmp[" << FormatProgramNumber(weave.dWeaveAmplitudeMm)
        << "],LAmp[" << FormatProgramNumber(weave.dWeaveAmplitudeMm)
        << "],RT[" << FormatProgramNumber(static_cast<double>(weave.nPauseTime1Ms) / 1000.0)
        << "],LT[" << FormatProgramNumber(static_cast<double>(weave.nPauseTime2Ms) / 1000.0)
        << "];\r\n";
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
    constexpr const char* teachPrefix = "/TeachProgram/";
    const std::size_t projectBegin = std::strlen(teachPrefix);
    const std::size_t projectEnd = normalizedPath.find('/', projectBegin);
    if (projectBegin >= normalizedPath.size())
    {
        error = "当前任务路径缺少工程名：" + normalizedPath;
        return false;
    }
    projectName = normalizedPath.substr(projectBegin,
        projectEnd == std::string::npos ? std::string::npos : projectEnd - projectBegin);
    if (projectName.empty())
    {
        error = "当前任务路径缺少工程名：" + normalizedPath;
        return false;
    }
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

bool RegisterInovanceProgramInProject(
    const std::string& projectContent,
    const std::string& programFile,
    std::string& updatedContent,
    bool& changed,
    std::string& error)
{
    updatedContent.clear();
    changed = false;
    error.clear();
    QJsonParseError parseError;
    const QJsonDocument document = QJsonDocument::fromJson(
        QByteArray(projectContent.data(), static_cast<int>(projectContent.size())),
        &parseError);
    if (parseError.error != QJsonParseError::NoError || !document.isObject())
    {
        error = "PRJ不是有效JSON：" + parseError.errorString().toStdString();
        return false;
    }
    QJsonObject root = document.object();
    if (root.value(QStringLiteral("FileType")).toString()
            != QStringLiteral("RobotProjectConfigFile")
        || root.value(QStringLiteral("Company")).toString().compare(
            QStringLiteral("Inovance"), Qt::CaseInsensitive) != 0)
    {
        error = "PRJ缺少汇川RobotProjectConfigFile身份。";
        return false;
    }

    const QString requested = QString::fromUtf8(programFile.c_str());
    const auto updateProgramFiles = [&requested, &changed, &error](
        QJsonObject& owner) -> bool
        {
            QJsonArray files = owner.value(QStringLiteral("ProgramFiles")).toArray();
            bool mainFound = false;
            bool targetFound = false;
            for (const QJsonValue& value : files)
            {
                const QString name = value.toString();
                mainFound = mainFound || name.compare(
                    QStringLiteral("main.pro"), Qt::CaseInsensitive) == 0;
                targetFound = targetFound || name.compare(requested, Qt::CaseInsensitive) == 0;
            }
            if (!mainFound)
            {
                error = "PRJ程序清单未登记固定入口main.pro。";
                return false;
            }
            if (!targetFound)
            {
                if (files.size() >= kInovanceProgramFileLimit)
                {
                    error = "PRJ程序清单已达到16个文件上限。";
                    return false;
                }
                files.append(requested);
                changed = true;
            }
            if (owner.value(QStringLiteral("ProgramFiles")) != files
                || owner.value(QStringLiteral("ProgramFilesCount")).toInt(-1) != files.size())
            {
                changed = true;
            }
            owner.insert(QStringLiteral("ProgramFiles"), files);
            owner.insert(QStringLiteral("ProgramFilesCount"), files.size());
            return true;
        };

    QJsonArray tasks = root.value(QStringLiteral("MultiTaskInfos")).toArray();
    if (!tasks.isEmpty())
    {
        int activeTaskIndex = -1;
        for (int index = 0; index < tasks.size(); ++index)
        {
            const QJsonObject task = tasks.at(index).toObject();
            if (task.value(QStringLiteral("TaskId")).toInt(-1) == 0
                && task.value(QStringLiteral("EnterProgramFile")).toString().compare(
                    QStringLiteral("main.pro"), Qt::CaseInsensitive) == 0)
            {
                activeTaskIndex = index;
                break;
            }
        }
        if (activeTaskIndex < 0)
        {
            error = "V3 PRJ未找到Task0/main.pro任务清单。";
            return false;
        }
        QJsonObject task = tasks.at(activeTaskIndex).toObject();
        // Some field projects contain both project-level modules at the root and a
        // Task0 record without ProgramFiles/ProgramFilesCount.  Root ProgramFiles
        // must stay at the project root (its PRO files are stored there); only repair
        // the missing Task0 list with the task entry program.  Never move root modules
        // into Task0 because that changes their controller path.
        if (!task.value(QStringLiteral("ProgramFiles")).isArray())
        {
            const QString enterProgram = task.value(
                QStringLiteral("EnterProgramFile")).toString().trimmed();
            if (enterProgram.isEmpty())
            {
                error = "V3 PRJ的Task0缺少ProgramFiles，且EnterProgramFile为空，无法安全修复。";
                return false;
            }
            QJsonArray migratedFiles;
            migratedFiles.append(enterProgram);
            task.insert(QStringLiteral("ProgramFiles"), migratedFiles);
            task.insert(QStringLiteral("ProgramFilesCount"), migratedFiles.size());
            changed = true;
        }
        if (!updateProgramFiles(task)) { return false; }
        tasks.replace(activeTaskIndex, task);
        root.insert(QStringLiteral("MultiTaskInfos"), tasks);
        if (root.value(QStringLiteral("MultiTaskCount")).toInt(-1) != tasks.size())
        {
            root.insert(QStringLiteral("MultiTaskCount"), tasks.size());
            changed = true;
        }
        const bool hasRootProgramFiles = root.value(
            QStringLiteral("ProgramFiles")).isArray();
        std::set<std::string> rootPrograms;
        for (const QJsonValue& fileValue :
            root.value(QStringLiteral("ProgramFiles")).toArray())
        {
            const std::string name = LowerAscii(fileValue.toString().toStdString());
            if (!name.empty()) { rootPrograms.insert(name); }
        }
        std::set<std::string> projectPrograms = rootPrograms;
        for (const QJsonValue& taskValue : tasks)
        {
            for (const QJsonValue& fileValue :
                taskValue.toObject().value(QStringLiteral("ProgramFiles")).toArray())
            {
                projectPrograms.insert(LowerAscii(fileValue.toString().toStdString()));
            }
        }
        if (projectPrograms.size() > kInovanceProgramFileLimit)
        {
            error = "PRJ程序清单合计超过16个文件上限。";
            return false;
        }
        // Normal V3 exports omit root ProgramFiles and use the root count as the
        // aggregate task count.  Hybrid exports pair a root ProgramFiles array
        // with project-level modules stored outside Task0; in that form the count
        // belongs to that root array.  Preserve the controller's two valid forms.
        const int projectProgramCount = static_cast<int>(
            hasRootProgramFiles ? rootPrograms.size() : projectPrograms.size());
        if (root.value(QStringLiteral("ProgramFilesCount")).toInt(-1)
            != projectProgramCount)
        {
            changed = true;
        }
        root.insert(QStringLiteral("ProgramFilesCount"), projectProgramCount);
    }
    else
    {
        if (!updateProgramFiles(root)) { return false; }
    }

    const QByteArray serialized = QJsonDocument(root).toJson(QJsonDocument::Indented);
    if (serialized.isEmpty())
    {
        error = "PRJ序列化结果为空。";
        return false;
    }
    updatedContent.assign(serialized.constData(), static_cast<std::size_t>(serialized.size()));
    return true;
}

bool ValidateInovanceCallableModule(const std::string& content, std::string& error)
{
    const std::regex moduleScopeComment(
        R"(EndProgramInfo(?:[\t ]*\r?\n)*[\t ]*//)",
        std::regex_constants::icase);
    if (std::regex_search(content, moduleScopeComment))
    {
        error = "ProgramInfo后、局部点声明前不能放置模块级注释。";
        return false;
    }
    const std::regex callableFunction(
        R"((^|[\r\n])[\t ]*Func[\t ]*func1[\t ]*\([\t ]*\))",
        std::regex_constants::icase);
    const std::regex startEntry(
        R"((^|[\r\n])[\t ]*Start[\t ]*;)",
        std::regex_constants::icase);
    const std::regex mainEntry(
        R"((^|[\r\n])[\t ]*Main[\t ]*\()",
        std::regex_constants::icase);
    if (!std::regex_search(content, callableFunction))
    {
        error = "模块缺少汇川Call调度约定的无参数入口 Func func1()。";
        return false;
    }
    if (std::regex_search(content, startEntry) || std::regex_search(content, mainEntry))
    {
        error = "公共模块包含 Start/Main 任务入口；同一工程只能由 main.pro 保留入口函数。";
        return false;
    }
    const std::regex localPointDeclaration(
        R"((^|[\r\n])[\t ]*LP[\t ]*\[[\t ]*([0-9]+)[\t ]*\][\t ]*=)",
        std::regex_constants::icase);
    std::set<int> localPointIndexes;
    for (std::sregex_iterator match(
             content.begin(), content.end(), localPointDeclaration), end;
         match != end; ++match)
    {
        int index = -1;
        try
        {
            index = std::stoi((*match)[2].str());
        }
        catch (...)
        {
            error = "公共模块包含无法解析的LP变量序号。";
            return false;
        }
        if (index < 0 || index > 9999)
        {
            error = "公共模块LP[" + std::to_string(index)
                + "]超出0..9999范围。";
            return false;
        }
        if (!localPointIndexes.insert(index).second)
        {
            error = "公共模块重复声明LP[" + std::to_string(index)
                + "]；同一PRO内局部点序号必须唯一。";
            return false;
        }
    }
    error.clear();
    return true;
}

bool WriteInovanceDispatcher(
    const std::filesystem::path& path,
    const std::string& moduleName,
    const std::string& robotName,
    std::string& content,
    std::string& error)
{
    std::ostringstream source;
    source << InovanceProgramInfo(robotName)
        << "Start;\r\n"
        << "B[" << kInovanceNativeProgramStateByte << "] = 1;\r\n"
        << "Call \"" << moduleName << ".pro\",\""
        << kInovanceCallableFunction << "\";\r\n"
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
    m_kinematicsSession.Invalidate();
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
    int configuredUserLevel = 0;
    std::string configuredPassword;
    ini.ReadString(false, "ApiUserLevel", &configuredUserLevel);
    ini.ReadString(false, "ApiPassword", &configuredPassword);
    const auto login = InovanceUserLogin::Resolve(configuredUserLevel, configuredPassword);
    m_apiUserLevel = login.level;
    m_apiPassword = login.password;
    ini.ReadString(false, "FTPIP", &m_ftpIp);
    ini.ReadString(false, "FTPPort", &m_ftpPort);
    ini.ReadString(false, "FTPUser", &m_ftpUser);
    ini.ReadString(false, "FTPPassWord", &m_ftpPassword);

    if (m_socketPort <= 0 || m_socketPort > 65535) { m_socketPort = 2222; }
    if (m_ftpIp.empty()) { m_ftpIp = m_socketIp; }
    if (m_ftpPort <= 0 || m_ftpPort > 65535) { m_ftpPort = 7777; }
    if (m_nRobotType == 0) { m_nRobotType = ROBOT_TYPE_INOVANCE; }
    // 早期汇川模板曾把现场坐标错误写成 Tool0/Wobj0。当前控制器标定、扫描、
    // 手眼和原生JOB均绑定 Tool[1] + Wobj[1]；加载时修复旧数据并回写数据库。
    if (m_toolNo == 0)
    {
        m_toolNo = kApplicationGunToolNumber;
        if (!ini.WriteString("ToolNo", m_toolNo) && m_pRobotLog != nullptr)
        {
            m_pRobotLog->write(LogColor::WARNING,
                "汇川旧Tool0配置已在本次运行按Tool1使用，但回写数据库失败 | unit=%s",
                unitName.c_str());
        }
    }
    if (m_wobjNo == 0)
    {
        m_wobjNo = 1;
        if (!ini.WriteString("WobjNo", m_wobjNo) && m_pRobotLog != nullptr)
        {
            m_pRobotLog->write(LogColor::WARNING,
                "汇川旧Wobj0配置已在本次运行按Wobj1使用，但回写数据库失败 | unit=%s",
                unitName.c_str());
        }
    }
    m_toolNo = std::clamp(m_toolNo, 1, 15);
    m_wobjNo = std::clamp(m_wobjNo, 1, 15);
    m_maxBufferedCommands = std::clamp(m_maxBufferedCommands, 1, 32);

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
        | RobotDriverCapabilityBit(RobotDriverCapability::HandEyeMatrixRead)
        | RobotDriverCapabilityBit(RobotDriverCapability::OfflineTrajectoryExport)
        | RobotDriverCapabilityBit(RobotDriverCapability::CircularMotion)
        | RobotDriverCapabilityBit(RobotDriverCapability::RealRegister)
        | RobotDriverCapabilityBit(RobotDriverCapability::StructuredControllerStatus)
        | RobotDriverCapabilityBit(RobotDriverCapability::ControllerKinematicsRead)
        | RobotDriverCapabilityBit(RobotDriverCapability::ControllerKinematicsCalculate)
        | RobotDriverCapabilityBit(RobotDriverCapability::CalibrationAssetDiscovery);
    if (m_connectionReady.load() && m_kinematicsSession.Ready())
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
    m_connectionReady.store(false);
    m_kinematicsSession.Invalidate();
    {
        std::lock_guard<std::mutex> passiveLock(m_passiveMutex);
        m_passivePulseValid = false;
    }
    m_userLoggedIn.store(false);
    m_modeConnectionEpoch.fetch_add(1);
    m_dataStreamEntryMode.store(-1);
    m_dataStreamEntryMotor.store(-1);
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
    // An explicit connection action permits one new authentication attempt.
    return ConnectWithPolicy(true);
}

bool InovanceRobotCtrl::ConnectWithPolicy(bool explicitRetry)
{
    // Keep the TCP handshake and authentication atomic against all commands,
    // concurrent Connect calls and Disconnect. IsConnected is false until verified.
    std::unique_lock<std::mutex> lock(m_socketMutex);
    if (m_connectionReady.load()) { return true; }
    std::string blockedError;
    if (!m_loginRetry.Begin(explicitRetry, blockedError))
    {
        SetLastRobotError(blockedError);
        return false;
    }
    ClearLastRobotError();
    const RobotConnectionEndpoint endpoint = ControlEndpoint();
    if (!endpoint.IsValid())
    {
        SetLastRobotError("汇川连接参数不完整，控制端口应为控制器远程以太网端口2222。");
        return false;
    }

    {
        if (m_connectionReady.load()) { return true; }
        if (m_connected.load()) { CloseSocketLocked(); }
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
    if (!QueryIntLocked("Get_ConnectState", connectionState) || connectionState != 1)
    {
        SetLastRobotError("汇川TCP已建立，但Get_ConnectState未确认上位机连接状态为1。");
        CloseSocketLocked();
        return false;
    }
    if (!LoginUserLocked())
    {
        CloseSocketLocked();
        return false;
    }
    m_connectionReady.store(true);
    ClearLastRobotError();
    if (m_pRobotLog != nullptr)
    {
        m_pRobotLog->write(LogColor::SUCCESS,
            "汇川远程以太网已连接：%s:%d，用户级别=%d（CurUserType已确认）",
            endpoint.host.c_str(), endpoint.port, m_apiUserLevel);
    }
    // Live read-only acquisition, not a cached AxisUnit grant. Release the
    // command lock first: the fixed recipe issues ordinary query commands.
    lock.unlock();
    // Restore the selected recipe only. This never runs a test, changes mode,
    // acquires control permit, enables the motor or starts the data stream.
    RestoreModePreparation();
    RobotKinematicsValidationResult kinematics;
    if (!RefreshKinematicsFromController(kinematics))
    {
        if (m_pRobotLog != nullptr)
        { m_pRobotLog->write(LogColor::ERR, "汇川控制连接保留，关节运动未就绪：%s", GetLastRobotError().c_str()); }
        return IsConnected();
    }
    if (m_pRobotLog != nullptr)
    { m_pRobotLog->write(LogColor::SUCCESS, "汇川连接后运动学已实时校验并保存数据库：%s", kinematics.acquisitionSummary.c_str()); }
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
    ConnectWithPolicy(false);
}

std::string InovanceRobotCtrl::ProtocolErrorText(const std::string& response)
{
    std::string code = Trim(response);
    const std::size_t end = code.find_first_of(" ,;:");
    if (end != std::string::npos) { code.resize(end); }
    if (code == "e1") return "指令语法错误";
    if (code == "e2") return "参数数量错误";
    if (code == "e3") return "参数值不合法";
    if (code == "e4") return "当前模式不允许此操作";
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
    std::lock_guard<std::mutex> lock(m_socketMutex);
    if (!m_connectionReady.load())
    {
        response.clear();
        SetLastRobotError(m_loginRetry.Error().empty()
            ? "汇川命令未发送：连接或自动登录尚未完成。" : m_loginRetry.Error());
        return false;
    }
    return SendCommandLocked(command, response, timeoutMs);
}

bool InovanceRobotCtrl::SendCommandLocked(
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

    if (!m_connected.load() || ToSocket(m_socketHandle) == INVALID_SOCKET)
    {
        SetLastRobotError("汇川命令失败：机器人未连接。");
        return false;
    }
    // Startup acquisition has no high-level operation lease (stage 6 already
    // owns one). Enforce exclusion here as well; safety Stop/OFF still pass.
    if (m_kinematicsReadInProgress.load()
        && (command.rfind("Mov", 0) == 0 || command.rfind("Set_", 0) == 0
            || command == "Prg Start" || command == "Motor ON"
            || command == "Dsmode ON" || command == "Dsmode CONTINUE"))
    {
        SetLastRobotError("汇川正在只读校验运动学资产，暂不接受运动、上电、程序启动或参数写入；安全停止仍可使用。");
        return false;
    }
    // 与安全 STOP 共用 socket 互斥锁后再检查取消锁存，关闭
    // “检查通过 -> STOP -> 随后才发送 Prg Start”的竞态窗口。
    if ((command == "Prg Start" || command == "Motor ON" || command == "Dsmode ON"
        || command == "Dsmode CONTINUE") && RobotOperationLease::IsCancellationRequested(this))
    {
        SetLastRobotError("汇川硬件操作已被安全停止取消，" + command + " 未发送。");
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
        if (command.rfind("UserLogin ", 0) == 0)
        {
            SetLastRobotError("汇川UserLogin失败：" + InovanceUserLogin::RejectionDetail(response));
            return false;
        }
        const std::string operation = command.rfind("Dsmode ", 0) == 0
            || command.rfind("Motor ", 0) == 0 || command.rfind("Set_Mode ", 0) == 0
            ? command : command.substr(0, command.find(' '));
        SetLastRobotError("汇川命令 " + operation + " 失败："
            + ProtocolErrorText(response) + "（" + response + "）。");
        return false;
    }
    ClearLastRobotError();
    return true;
}

bool InovanceRobotCtrl::QueryInt(const std::string& command, int& value)
{
    std::lock_guard<std::mutex> lock(m_socketMutex);
    if (!m_connectionReady.load())
    {
        SetLastRobotError(m_loginRetry.Error().empty()
            ? "汇川查询未发送：连接或自动登录尚未完成。" : m_loginRetry.Error());
        return false;
    }
    return QueryIntLocked(command, value);
}

bool InovanceRobotCtrl::QueryIntLocked(const std::string& command, int& value)
{
    std::string response;
    if (!SendCommandLocked(command, response)) { return false; }
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

bool InovanceRobotCtrl::QueryLeadingInt(const std::string& command, int& value)
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
        SetLastRobotError("汇川命令 " + command + " 返回值缺少有效的首个整数。");
        return false;
    }
    value = static_cast<int>(parsed);
    return true;
}

bool InovanceRobotCtrl::QuerySystemErrorCode(int& value)
{
    std::string response;
    if (!SendCommand("Get_SysErr", response)) { return false; }
    std::string text = Trim(ValuePart(response));
    int base = 0;
    if (!text.empty() && (text.back() == 'h' || text.back() == 'H'))
    {
        text.pop_back();
        base = 16;
    }
    char* end = nullptr;
    errno = 0;
    const long parsed = std::strtol(text.c_str(), &end, base);
    while (end != nullptr && *end != '\0'
        && std::isspace(static_cast<unsigned char>(*end)))
    {
        ++end;
    }
    if (errno != 0 || end == text.c_str() || (end != nullptr && *end != '\0')
        || parsed < std::numeric_limits<int>::min()
        || parsed > std::numeric_limits<int>::max())
    {
        SetLastRobotError("汇川命令 Get_SysErr 返回的十六进制故障码格式无效。");
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
    profile.localFileFilters = { "*.pro", "*.prj", "*.pts", "*.jsn", "*.dat" };
    profile.acceptanceProgramExtensions = { ".pro" };
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
        std::vector<std::string>{ ".pro", ".prj", ".pts", ".jsn", ".dat" },
        std::vector<std::string>{ ".pro" },
        "Log/InovanceRobotFtp.log");
}

bool InovanceRobotCtrl::RefreshKinematicsFromController(
    RobotKinematicsValidationResult& result)
{
    std::lock_guard<std::mutex> refreshLock(m_kinematicsRefreshMutex);
    InovanceKinematicsReadScope readOnlyScope(m_kinematicsReadInProgress);
    const std::uint64_t generation = m_kinematicsSession.Invalidate();
    {
        std::lock_guard<std::mutex> passiveLock(m_passiveMutex);
        m_passivePulseValid = false;
    }
    result = {};
    const auto fail = [this](const std::string& message)
    {
        SetLastRobotError("汇川运动学资产读取失败：" + message);
        return false;
    };
    if (!IsConnected()) { return fail("2222控制通道未连接。"); }
    int motion = -1;
    if (m_trajectoryRunning.load() || m_nativeProgramRunning.load()
        || m_continuousJogRunning.load() || RobotOperationLease::MotionCompletionPending(this)
        || !QueryInt("Get_MotionSts", motion) || motion != 0)
    { return fail("机器人未确认静止，暂不装载轴单位；停止后请重新连接或执行流程6。"); }

    std::string modelResponse;
    std::string firmwareResponse;
    if (!SendCommand("Get_RobotType", modelResponse)
        || !SendCommand("Get_FwVersion", firmwareResponse)) { return false; }
    result.modelName = ValuePart(modelResponse);
    if (result.modelName.empty() || result.modelName.size() > 128)
    { return fail("Get_RobotType返回无效。"); }
    if (ValuePart(firmwareResponse).empty()) { return fail("Get_FwVersion返回无效。"); }

    std::vector<double> structure;
    std::vector<double> angularCompensation;
    std::vector<double> reductionRatios;
    std::vector<double> couplingMaster;
    std::vector<double> couplingSlave;
    std::vector<double> absoluteZero;
    if (!QueryDoubles("Get_StrPara", structure, 6)
        || !QueryDoubles("Get_StrParaComp", angularCompensation, 6)
        || !QueryDoubles("Get_RdctRatio", reductionRatios, 6)
        || !QueryDoubles("Get_CpParaM", couplingMaster, 6)
        || !QueryDoubles("Get_CpParaS", couplingSlave, 6)
        || !QueryDoubles("Get_ZeroPos", absoluteZero, 6))
    {
        return false;
    }

    T_AXISLIMITANGLE limits;
    for (int axis = 0; axis < 6; ++axis)
    {
        std::vector<double> negative;
        std::vector<double> positive;
        const std::string name = "J" + std::to_string(axis + 1);
        if (!QueryDoubles("Get_AxisNLim " + name, negative, 1)
            || !QueryDoubles("Get_AxisPLim " + name, positive, 1)
            || negative[0] >= positive[0])
        {
            return fail("关节" + name + "限位读取或范围校验失败。");
        }
        double* limitStorage = reinterpret_cast<double*>(&limits);
        limitStorage[axis * 2] = positive[0];
        limitStorage[axis * 2 + 1] = negative[0];
    }

    // 远程协议没有返回编码器位数和d3/d5/a4/a5长度补偿；只读下载厂商
    // MachineParams.json补齐。该文件只作本次内存解析，随后立即删除临时副本。
    const QString localDirectory = AppPaths::WritablePath(
        QStringLiteral("Temp/InovanceKinematics"));
    std::error_code fileError;
    std::filesystem::create_directories(
        std::filesystem::path(localDirectory.toStdWString()), fileError);
    if (fileError) { return fail("无法创建参数只读下载临时目录。"); }
    // Each robot/refresh owns its temporary copy; parallel RobotA/RobotC reads
    // cannot delete or parse one another's MachineParams.json.
    QTemporaryDir downloadDirectory(localDirectory + QStringLiteral("/read-XXXXXX"));
    if (!downloadDirectory.isValid()) { return fail("无法创建独立参数下载目录。"); }
    const QString localFile = downloadDirectory.filePath(QStringLiteral("MachineParams.json"));

    FtpClient ftp(m_pRobotLog, m_ftpIp, m_ftpPort, m_ftpUser, m_ftpPassword);
    ftp.setMessageBoxesEnabled(false);
    if (!ftp.connect()) { return fail("FTP连接失败，无法补齐编码器和长度补偿参数。"); }
    std::vector<FtpRemoteFileInfo> parameterFiles;
    if (!ftp.listFiles("/RobotParams", parameterFiles, nullptr, 256))
    { return fail("FTP无法列出/RobotParams。"); }
    const auto machineFile = std::find_if(parameterFiles.cbegin(), parameterFiles.cend(),
        [](const FtpRemoteFileInfo& entry)
        {
            return !entry.isDirectory && LowerAscii(entry.name) == "machineparams.json";
        });
    if (machineFile == parameterFiles.cend() || machineFile->size == 0
        || machineFile->size > kMaxInovanceMachineParametersBytes)
    { return fail("MachineParams.json不存在、为空或超过1MiB上限。"); }
    std::atomic_bool cancelDownload{ false };
    if (!ftp.downloadFileBounded(
        kInovanceMachineParametersPath,
        localFile.toStdString(),
        machineFile->size,
        kMaxInovanceMachineParametersBytes,
        &cancelDownload))
    { return fail("MachineParams.json只读下载失败。"); }

    QFile input(localFile);
    if (!input.open(QIODevice::ReadOnly))
    {
        std::filesystem::remove(std::filesystem::path(localFile.toStdWString()), fileError);
        return fail("MachineParams.json临时副本无法读取。");
    }
    const QByteArray jsonBytes = input.readAll();
    input.close();
    std::filesystem::remove(std::filesystem::path(localFile.toStdWString()), fileError);
    QJsonParseError jsonError;
    const QJsonDocument document = QJsonDocument::fromJson(jsonBytes, &jsonError);
    if (jsonError.error != QJsonParseError::NoError || !document.isObject())
    { return fail("MachineParams.json格式无效。"); }

    const QJsonObject root = document.object();
    const QJsonObject body = root.value(QStringLiteral("stRobotBody")).toObject();
    if (body.value(QStringLiteral("cRobotName")).toString().toStdString() != result.modelName
        || !body.value(QStringLiteral("cRobotName")).toString().startsWith(QStringLiteral("IR-R"))
        || body.value(QStringLiteral("RobotType")).toInt() != 6
        || body.value(QStringLiteral("stBase")).toObject().value(QStringLiteral("i32AxisNum")).toInt() != 6)
    { return fail("TCP/FTP型号不一致或不是已支持的IR-R六轴机构。"); }
    const QJsonObject joint = root.value(QStringLiteral("stJoint")).toObject();
    const QJsonObject kinematicsFile = root.value(QStringLiteral("stRobotBody")).toObject()
        .value(QStringLiteral("stKinematics")).toObject();
    const QJsonObject install = root.value(QStringLiteral("stMotion")).toObject()
        .value(QStringLiteral("stSpace")).toObject()
        .value(QStringLiteral("stInstall")).toObject()
        .value(QStringLiteral("stInstallMode")).toObject();
    std::vector<double> encoderBits;
    std::vector<double> fileRatios;
    std::vector<double> fileZero;
    std::vector<double> filePositiveLimits;
    std::vector<double> fileNegativeLimits;
    std::vector<double> fileStructure;
    std::vector<double> fileCouplingMaster;
    std::vector<double> fileCouplingSlave;
    if (!JsonDoubleArray(joint, "i32EncBit", 6, encoderBits)
        || !JsonDoubleArray(joint, "dRatio", 6, fileRatios)
        || !JsonDoubleArray(joint, "dAbsZero", 6, fileZero)
        || !JsonDoubleArray(joint, "dPosLimit", 6, filePositiveLimits)
        || !JsonDoubleArray(joint, "dNegLimit", 6, fileNegativeLimits)
        || !JsonDoubleArray(joint, "dCoupParamMaster", 6, fileCouplingMaster)
        || !JsonDoubleArray(joint, "dCoupParamSlave", 6, fileCouplingSlave)
        || !JsonDoubleArray(kinematicsFile, "dRobotStructureParam", 6, fileStructure))
    { return fail("MachineParams.json缺少机械参数字段。"); }

    const double fileAnglesRaw[6] = {
        install.value(QStringLiteral("alpha1")).toDouble(std::numeric_limits<double>::quiet_NaN()),
        install.value(QStringLiteral("alpha2")).toDouble(std::numeric_limits<double>::quiet_NaN()),
        install.value(QStringLiteral("alpha3")).toDouble(std::numeric_limits<double>::quiet_NaN()),
        install.value(QStringLiteral("alpha4")).toDouble(std::numeric_limits<double>::quiet_NaN()),
        install.value(QStringLiteral("alpha5")).toDouble(std::numeric_limits<double>::quiet_NaN()),
        install.value(QStringLiteral("beta2")).toDouble(std::numeric_limits<double>::quiet_NaN())
    };
    const std::vector<double> fileAngles(std::begin(fileAnglesRaw), std::end(fileAnglesRaw));
    const double d3 = install.value(QStringLiteral("d3")).toDouble(
        std::numeric_limits<double>::quiet_NaN());
    const double d5 = install.value(QStringLiteral("d5")).toDouble(
        std::numeric_limits<double>::quiet_NaN());
    const double a4 = install.value(QStringLiteral("a4")).toDouble(
        std::numeric_limits<double>::quiet_NaN());
    const double a5 = install.value(QStringLiteral("a5")).toDouble(
        std::numeric_limits<double>::quiet_NaN());
    if (!std::all_of(fileAngles.cbegin(), fileAngles.cend(),
            [](double value) { return std::isfinite(value); })
        || !std::isfinite(d3) || !std::isfinite(d5)
        || !std::isfinite(a4) || !std::isfinite(a5))
    { return fail("MachineParams.json缺少完整安装补偿参数。"); }
    std::vector<double> lengthCompensation;
    if (!QueryDoubles("Get_SupplementaryStrParamComp", lengthCompensation, 4)
        || !NearlyEqualArray(lengthCompensation, {d3, d5, a4, a5}, 4, 0.002))
    { return fail("2222长度补偿与FTP参数不一致，拒绝装载混合设备或旧备份模型。"); }

    // 同一控制器的两条只读来源必须一致，防止拿到备份文件或其它本体参数。
    if (!NearlyEqualArray(structure, fileStructure, 6, 0.002)
        || !NearlyEqualArray(angularCompensation, fileAngles, 6, 0.002)
        || !NearlyEqualArray(reductionRatios, fileRatios, 6, 0.002)
        || !NearlyEqualArray(absoluteZero, fileZero, 6, 0.51)
        || !NearlyEqualArray(couplingMaster, fileCouplingMaster, 6, 0.002)
        || !NearlyEqualArray(couplingSlave, fileCouplingSlave, 6, 0.002))
    { return fail("2222接口值与FTP MachineParams.json不一致，拒绝装载混合模型。"); }
    for (int axis = 0; axis < 6; ++axis)
    {
        if (std::abs(limits.GetMaxAngleByIndex(axis) - filePositiveLimits[axis]) > 0.002
            || std::abs(limits.GetMinAngleByIndex(axis) - fileNegativeLimits[axis]) > 0.002)
        { return fail("2222关节限位与FTP参数不一致。"); }
    }

    T_AXISUNIT units = m_tAxisUnit; // Preserve separately configured external axes.
    double* unitStorage = reinterpret_cast<double*>(&units);
    for (int axis = 0; axis < 6; ++axis)
    {
        const int bits = static_cast<int>(std::llround(encoderBits[axis]));
        if (bits < 8 || bits > 32 || encoderBits[axis] != bits || fileRatios[axis] <= 0.0)
        { return fail("编码器位数或减速比范围无效。"); }
        unitStorage[axis] = 360.0 / (std::ldexp(1.0, bits) * fileRatios[axis]);
    }

    // 汇川IR-R六轴模型：结构长度为a1/a2/a3/d4/d6/d1；安装补偿给出
    // alpha1..alpha5/beta2及d3/d5/a4/a5，J2固定+90度零位偏置。
    T_KINEMATICS model;
    model.dA1 = fileStructure[0]; model.dAL1 = fileAngles[0]; model.dD1 = fileStructure[5]; model.dTH1 = 0.0;
    model.dA2 = fileStructure[1]; model.dAL2 = fileAngles[1]; model.dD2 = 0.0; model.dTH2 = 90.0;
    model.dA3 = fileStructure[2]; model.dAL3 = fileAngles[2]; model.dD3 = d3; model.dTH3 = 0.0;
    model.dA4 = a4; model.dAL4 = fileAngles[3]; model.dD4 = fileStructure[3]; model.dTH4 = 0.0;
    model.dA5 = a5; model.dAL5 = fileAngles[4]; model.dD5 = d5; model.dTH5 = 0.0;
    model.dA6 = 0.0; model.dAL6 = fileAngles[5]; model.dD6 = fileStructure[4]; model.dTH6 = 0.0;

    std::vector<double> joints;
    std::vector<double> rawPulses;
    if (!QueryInt("Get_MotionSts", motion) || motion != 0
        || !QueryDoubles("Get_RobJPHere", joints, 14)
        || !QueryDoubles("Get_PosHerePulse", rawPulses, 6))
    { return false; }
    for (int axis = 0; axis < 6; ++axis)
    { result.currentJointDegrees[axis] = joints[axis]; }

    const QJsonArray couplingMatrix = joint.value(QStringLiteral("dCoupParam")).toArray();
    if (couplingMatrix.size() < 6 || couplingMatrix.at(5).toArray().size() < 6)
    { return fail("MachineParams.json缺少完整关节耦合矩阵。"); }
    const double j6FromJ5 = couplingMatrix.at(5).toArray().at(4).toDouble(
        std::numeric_limits<double>::quiet_NaN());
    if (!std::isfinite(j6FromJ5)) { return fail("J5/J6耦合系数无效。"); }
    double maxPulseJointError = 0.0;
    for (int axis = 0; axis < 6; ++axis)
    {
        double reconstructed = (rawPulses[axis] - fileZero[axis]) * unitStorage[axis];
        if (axis == 5) { reconstructed -= j6FromJ5 * joints[4]; }
        if (!std::isfinite(reconstructed) || !std::isfinite(reconstructed - joints[axis]))
        { return fail("脉冲/关节换算出现非有限结果。"); }
        maxPulseJointError = std::max(maxPulseJointError,
            std::abs(reconstructed - joints[axis]));
    }
    if (maxPulseJointError > 0.01)
    { return fail("绝对零点/编码器/耦合换算与Get_RobJPHere不一致。"); }

    int activeTool = -1;
    int activeWobj = -1;
    if (!QueryInt("Get_ToolCNum", activeTool)
        || !QueryInt("Get_WobjNum", activeWobj))
    { return false; }
    T_ROBOT_COORS tool;
    if (!GetToolData(activeTool, tool)) { return false; }
    std::vector<double> wobjValues;
    if (!QueryDoubles("Get_WobjData " + std::to_string(activeWobj), wobjValues, 14))
    { return false; }
    if (std::llround(wobjValues[1]) != 1)
    { return fail("当前工件坐标系不是固定工件，接口测试暂不支持关联外部机械单元。"); }
    T_ROBOT_COORS userFrame;
    userFrame.dX = wobjValues[2]; userFrame.dY = wobjValues[3]; userFrame.dZ = wobjValues[4];
    userFrame.dRZ = wobjValues[5]; userFrame.dRY = wobjValues[6]; userFrame.dRX = wobjValues[7];
    T_ROBOT_COORS objectFrame;
    objectFrame.dX = wobjValues[8]; objectFrame.dY = wobjValues[9]; objectFrame.dZ = wobjValues[10];
    objectFrame.dRZ = wobjValues[11]; objectFrame.dRY = wobjValues[12]; objectFrame.dRX = wobjValues[13];
    if (!ReadCartesianPosition(result.controllerTcpInActiveWorkobject, nullptr))
    { return false; }
    std::vector<double> finalJoints;
    int finalTool = -1;
    int finalWobj = -1;
    std::string finalModel;
    std::string finalFirmware;
    if (!QueryDoubles("Get_RobJPHere", finalJoints, 14)
        || !QueryInt("Get_MotionSts", motion) || motion != 0
        || !InovanceKinematicsSession::StationarySample(joints, finalJoints)
        || !QueryInt("Get_ToolCNum", finalTool) || finalTool != activeTool
        || !QueryInt("Get_WobjNum", finalWobj) || finalWobj != activeWobj
        || !SendCommand("Get_RobotType", finalModel) || finalModel != modelResponse
        || !SendCommand("Get_FwVersion", finalFirmware) || finalFirmware != firmwareResponse)
    { return fail("采样期间关节、工具、工件或设备身份发生变化，拒绝混合时刻校验结果。"); }
    const KDL::Frame controllerFlange = InovancePoseFrame(userFrame)
        * InovancePoseFrame(objectFrame)
        * InovancePoseFrame(result.controllerTcpInActiveWorkobject)
        * InovancePoseFrame(tool).Inverse();
    result.controllerFlangeInBase = InovanceFramePose(controllerFlange);

    KDL::Chain chain;
    const double* dh = reinterpret_cast<const double*>(&model);
    for (int axis = 0; axis < 6; ++axis)
    {
        chain.addSegment(KDL::Segment(
            KDL::Joint(KDL::Joint::RotZ),
            KDL::Frame::DH(
                dh[axis * 4] / 1000.0,
                dh[axis * 4 + 1] * M_PI / 180.0,
                dh[axis * 4 + 2] / 1000.0,
                dh[axis * 4 + 3] * M_PI / 180.0)));
    }
    KDL::JntArray jointArray(6);
    for (int axis = 0; axis < 6; ++axis)
    { jointArray(axis) = joints[axis] * M_PI / 180.0; }
    KDL::ChainFkSolverPos_recursive fk(chain);
    KDL::Frame calculatedFlange;
    if (fk.JntToCart(jointArray, calculatedFlange) < 0)
    { return fail("当前关节正运动学计算失败。"); }
    result.calculatedFlangeInBase = InovanceFramePose(calculatedFlange);
    result.positionErrorMm = (calculatedFlange.p - controllerFlange.p).Norm() * 1000.0;
    result.orientationErrorDeg = KDL::diff(controllerFlange, calculatedFlange).rot.Norm()
        * 180.0 / M_PI;
    if (!std::isfinite(result.positionErrorMm) || !std::isfinite(result.orientationErrorDeg)
        || result.positionErrorMm > 2.0 || result.orientationErrorDeg > 0.1)
    {
        std::ostringstream error;
        error << std::fixed << std::setprecision(4)
            << "当前关节/直角闭环超差：位置=" << result.positionErrorMm
            << "mm，姿态=" << result.orientationErrorDeg << "deg。";
        return fail(error.str());
    }

    std::ostringstream summary;
    summary << std::fixed << std::setprecision(4)
        << "2222:Get_RobotType/Get_StrPara/Get_StrParaComp/Get_RdctRatio/"
        << "Get_CpParaM/Get_CpParaS/Get_ZeroPos/Get_AxisNLim/Get_AxisPLim/"
        << "Get_RobJPHere/Get_RobPHere/Get_PosHerePulse/Get_ToolData/Get_WobjData；"
        << "FTP:" << kInovanceMachineParametersPath
        << "(i32EncBit,d3,d5,a4,a5,dCoupParam)；"
        << "活动Tool=" << activeTool << "，Wobj=" << activeWobj
        << "，脉冲关节最大误差=" << maxPulseJointError << "deg。";
    result.acquisitionSummary = summary.str();
    // Persist an independent branded evidence snapshot, never write the shared
    // RobotPara template and never restore a cache without the live recipe above.
    const auto jsonArray = [](const double* values, int count)
    {
        QJsonArray array;
        for (int index = 0; index < count; ++index) { array.append(values[index]); }
        return array;
    };
    QJsonObject snapshot;
    snapshot.insert(QStringLiteral("schema"), QStringLiteral("InovanceLiveKinematics-v1"));
    snapshot.insert(QStringLiteral("robot"), QString::fromStdString(m_sRobotName));
    snapshot.insert(QStringLiteral("controlHost"), QString::fromStdString(m_socketIp));
    snapshot.insert(QStringLiteral("controlPort"), m_socketPort);
    snapshot.insert(QStringLiteral("ftpHost"), QString::fromStdString(m_ftpIp));
    snapshot.insert(QStringLiteral("ftpPort"), m_ftpPort);
    snapshot.insert(QStringLiteral("model"), QString::fromStdString(result.modelName));
    snapshot.insert(QStringLiteral("firmware"), QString::fromStdString(ValuePart(firmwareResponse)));
    snapshot.insert(QStringLiteral("sourcePath"), QString::fromLatin1(kInovanceMachineParametersPath));
    snapshot.insert(QStringLiteral("sourceSha256"), QString::fromLatin1(
        QCryptographicHash::hash(jsonBytes, QCryptographicHash::Sha256).toHex()));
    snapshot.insert(QStringLiteral("sourceMachineParameters"), root);
    snapshot.insert(QStringLiteral("dh"), jsonArray(reinterpret_cast<const double*>(&model), 24));
    snapshot.insert(QStringLiteral("axisUnit"), jsonArray(unitStorage, 9));
    snapshot.insert(QStringLiteral("axisLimits"), jsonArray(reinterpret_cast<const double*>(&limits), 12));
    snapshot.insert(QStringLiteral("pulseJointErrorDeg"), maxPulseJointError);
    snapshot.insert(QStringLiteral("positionErrorMm"), result.positionErrorMm);
    snapshot.insert(QStringLiteral("orientationErrorDeg"), result.orientationErrorDeg);
    snapshot.insert(QStringLiteral("acquisitionSummary"), QString::fromStdString(result.acquisitionSummary));
    snapshot.insert(QStringLiteral("validatedAt"), QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs));
    snapshot.insert(QStringLiteral("requiresLiveValidationOnConnect"), true);
    const QString serialized = QString::fromUtf8(QJsonDocument(snapshot).toJson(QJsonDocument::Compact));

    // Database I/O can wait on SQLite; never hold the command/STOP socket lock
    // while persisting. Snapshot publication is rechecked under that lock below.
    if (!m_connectionReady.load() || generation != m_kinematicsSession.Generation())
    { return fail("运动学采集期间连接已改变，本次结果已作废。"); }
    const QString robot = QString::fromStdString(m_sRobotName);
    const QString module = QStringLiteral("InovanceKinematics");
    const QString key = QStringLiteral("ValidatedSnapshot");
    QString readback;
    if (!ConfigDatabase::WriteScopedSetting(QStringLiteral("robot"), robot, module, key, serialized, QStringLiteral("json"))
        || !ConfigDatabase::ReadScopedSetting(QStringLiteral("robot"), robot, module, key, &readback)
        || readback != serialized)
    { return fail("实时验证通过，但数据库快照保存/回读失败，关节运动保持关闭。"); }
    std::lock_guard<std::mutex> socketLock(m_socketMutex);
    if (!m_connectionReady.load() || generation != m_kinematicsSession.Generation())
    { return fail("数据库保存期间连接已改变，快照仅供诊断，本会话关节运动保持关闭。"); }
    std::string installError;
    if (!InstallValidatedKinematicsModel(model, units, limits, &installError))
    { return fail(installError); }
    if (!m_kinematicsSession.Publish(generation))
    { return fail("运动学结果不属于当前连接，拒绝启用关节运动。"); }
    result.valid = true;
    ClearLastRobotError();
    return true;
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
    if (!QueryLeadingInt("CurPermit", owner)) { return false; }
    if (owner == 1)
    {
        m_permitOwned.store(true);
        return true;
    }
    if (owner != 0 && owner != 2)
    {
        SetLastRobotError("汇川控制许可归属回读无效，未申请许可或发送控制指令。");
        return false;
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
    if (!QueryLeadingInt("CurPermit", owner) || owner != 1)
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

InovanceModeSequence::Ops InovanceRobotCtrl::ModeSequenceOps(std::uint64_t expectedEpoch)
{
    InovanceModeSequence::Ops ops;
    const auto epoch = expectedEpoch;
    ops.read = [this, epoch](InovanceModeSequence::State& state, std::string& error)
    {
        std::lock_guard<std::mutex> socketLock(m_socketMutex);
        if (!IsConnected() || epoch != m_modeConnectionEpoch.load())
        { error = "组合测试期间连接已变化，禁止跨连接继续测试或恢复。"; return false; }
        const bool ok = QueryIntLocked("Get_Mode", state.mode)
            && QueryIntLocked("Get_MotorSts", state.motor) && QueryIntLocked("Get_DsMode", state.stream)
            && QueryIntLocked("Get_MotionSts", state.motion) && QueryIntLocked("Get_TaskRunSts 0", state.task)
            && QueryIntLocked("Get_EStopSts", state.estop) && QueryIntLocked("Get_SysErrSts", state.fault)
            && IsConnected() && epoch == m_modeConnectionEpoch.load();
        error = ok ? std::string() : GetLastRobotError();
        return ok;
    };
    ops.send = [this, epoch](const std::string& command, std::string& reply)
    {
        std::lock_guard<std::mutex> socketLock(m_socketMutex);
        if (!IsConnected() || epoch != m_modeConnectionEpoch.load())
        { reply = "组合测试期间连接已变化，命令未发送。"; return false; }
        std::string response;
        const bool ok = SendCommandLocked(command, response) && response == "ok";
        reply = ok ? response : (GetLastRobotError().empty() ? "非预期应答：" + response : GetLastRobotError());
        return ok;
    };
    ops.cancelled = [this, epoch]() { return epoch != m_modeConnectionEpoch.load()
        || RobotOperationLease::IsCancellationRequested(this); };
    ops.delay = []() { std::this_thread::sleep_for(std::chrono::milliseconds(25)); };
    return ops;
}

void InovanceRobotCtrl::RestoreModePreparation()
{
    RobotModePreparationStore::Binding binding;
    std::uint64_t epoch = 0;
    std::string identityError;
    {
        // Capture identity on a single live connection. Do not hold the socket
        // mutex during SQLite I/O or take the socket mutex under the mode mutex.
        std::lock_guard<std::mutex> socketLock(m_socketMutex);
        if (!IsConnected()) { return; }
        epoch = m_modeConnectionEpoch.load();
        binding.robotName = QString::fromUtf8(m_sRobotName.c_str());
        binding.driver = QStringLiteral("Inovance");
        binding.host = QString::fromUtf8(m_socketIp.c_str());
        binding.port = m_socketPort;
        binding.revision = InovanceModeSequence::kStrategyRevision;
        std::string model, firmware;
        if (!SendCommandLocked("Get_RobotType", model)
            || !SendCommandLocked("Get_FwVersion", firmware))
        { identityError = "控制器型号/固件读取失败：" + GetLastRobotError(); }
        else
        {
            binding.controllerModel = QString::fromUtf8(ValuePart(model).c_str()).trimmed();
            binding.firmware = QString::fromUtf8(ValuePart(firmware).c_str()).trimmed();
            if (binding.controllerModel.isEmpty() || binding.controllerModel.size() > 128
                || binding.firmware.isEmpty() || binding.firmware.size() > 128)
            { identityError = "控制器型号/固件返回无效，不能恢复已固化组合。"; }
        }
    }
    RobotModePreparationStore::Record record;
    QString error;
    const auto status = identityError.empty()
        ? RobotModePreparationStore::Load(binding, record, error)
        : RobotModePreparationStore::LoadStatus::Error;
    if (!identityError.empty()) { error = QString::fromUtf8(identityError.c_str()); }
    InovanceModeSequence::Plan plan;
    const bool restored = status == RobotModePreparationStore::LoadStatus::Found
        && InovanceModeSequence::Find(record.planId.toStdString(), plan);
    if (status == RobotModePreparationStore::LoadStatus::Found && !restored)
    { error = QStringLiteral("已固化组合不在当前驱动支持列表中，需重新测试并固化。"); }
    if (!restored && error.isEmpty())
    { error = QStringLiteral("尚无已固化组合；旧验收报告不能代替策略配置，请在流程4测试通过后选用固化一次。"); }
    {
        std::lock_guard<std::mutex> modeLock(m_modePreparationMutex);
        if (!IsConnected() || epoch != m_modeConnectionEpoch.load()) { return; }
        m_verifiedModePreparations.clear();
        m_modePreparationTestRecords.clear();
        m_activeModePreparation.clear();
        m_persistedModePreparation.clear();
        m_modePreparationBinding = binding;
        m_modePreparationBindingEpoch = identityError.empty() ? epoch : 0;
        m_modePreparationLoadError = restored ? std::string() : error.toStdString();
        if (restored)
        {
            m_activeModePreparation = plan.id;
            m_persistedModePreparation = plan.id;
        }
    }
    if (m_pRobotLog != nullptr)
    {
        if (restored)
        { m_pRobotLog->write(LogColor::SUCCESS, "汇川已恢复固化数据流组合：%s；仅恢复策略，未上电/开流，运行时仍检查安全状态。", plan.id.c_str()); }
        else
        { m_pRobotLog->write(LogColor::ERR, "汇川数据流策略未恢复：%s", error.toStdString().c_str()); }
    }
}

std::vector<RobotModePreparationTestCase> InovanceRobotCtrl::ModePreparationTestCases() const
{
    std::vector<RobotModePreparationTestCase> result;
    std::lock_guard<std::mutex> lock(m_modePreparationMutex);
    for (const auto& plan : InovanceModeSequence::Plans())
    {
        const auto found = m_verifiedModePreparations.find(plan.id);
        const bool verified = m_connected.load() && found != m_verifiedModePreparations.end()
            && found->second == m_modeConnectionEpoch.load();
        const bool persisted = m_connectionReady.load()
            && m_modePreparationBindingEpoch == m_modeConnectionEpoch.load()
            && m_persistedModePreparation == plan.id && m_activeModePreparation == plan.id;
        result.push_back({ plan.id, plan.name + (persisted
            ? (verified ? "【本连接通过·已固化】" : "【已恢复固化策略】")
            : (verified ? "【本连接已通过·待固化】" : "【未验证】")) });
    }
    return result;
}

bool InovanceRobotCtrl::RunModePreparationTestCase(const std::string& id, RobotModePreparationTestResult& result)
{
    result = {};
    InovanceModeSequence::Plan plan;
    if (!InovanceModeSequence::Find(id, plan) || !IsConnected() || !EnsureControlPermit())
    { result.evidence = "测试组合无效、未连接或没有控制许可：" + GetLastRobotError(); return false; }
    const auto epoch = m_modeConnectionEpoch.load();
    {
        std::lock_guard<std::mutex> lock(m_modePreparationMutex);
        m_verifiedModePreparations.erase(id);
        m_modePreparationTestRecords.erase(id);
        if (m_modePreparationBindingEpoch != epoch)
        {
            result.evidence = "当前连接型号/固件身份未确认，不能测试并固化组合；请重新连接。";
            SetLastRobotError(result.evidence);
            return false;
        }
        if (m_persistedModePreparation == id)
        {
            // Retesting withdraws the old PASS durably before touching the
            // controller. A failed retest must not resurrect it after restart.
            QString revokeError;
            m_activeModePreparation.clear();
            if (!RobotModePreparationStore::Revoke(m_modePreparationBinding, revokeError))
            {
                result.evidence = "旧组合撤销保存失败，未开始测试：" + revokeError.toStdString();
                SetLastRobotError(result.evidence);
                return false;
            }
            m_persistedModePreparation.clear();
            m_modePreparationLoadError = "已固化组合已进入重测，需测试通过并再次选用固化。";
        }
    }
    if (!IsConnected() || epoch != m_modeConnectionEpoch.load())
    { result.evidence = "组合测试准备期间连接已变化，未开始测试。"; return false; }
    result.evidence = "组合 " + id + "：" + plan.name + "（无位移、不启动JOB）\n";
    result.passed = InovanceModeSequence::Test(ModeSequenceOps(epoch), plan, result.restoreVerified, result.evidence);
    result.restoreVerified = result.restoreVerified && epoch == m_modeConnectionEpoch.load() && IsConnected();
    result.passed = result.passed && result.restoreVerified;
    if (result.passed)
    {
        std::lock_guard<std::mutex> lock(m_modePreparationMutex);
        if (IsConnected() && epoch == m_modeConnectionEpoch.load()
            && m_modePreparationBindingEpoch == epoch)
        {
            m_verifiedModePreparations[id] = epoch;
            RobotModePreparationStore::Record record;
            record.planId = QString::fromStdString(id);
            record.verifiedAtUtc = QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs);
            record.evidence = QString::fromUtf8(result.evidence.c_str());
            m_modePreparationTestRecords[id] = std::move(record);
        }
        else { result.passed = false; result.restoreVerified = false; result.evidence += "测试结果发布时连接已变化，未授予选用资格。\n"; }
    }
    if (!result.passed) { SetLastRobotError(result.evidence); }
    return result.passed;
}

bool InovanceRobotCtrl::UseVerifiedModePreparation(const std::string& id)
{
    // Keep selection atomic against a new business operation starting after
    // the UI's busy check. Never replace a recipe while its exit order is in use.
    struct SelectionBlock
    {
        RobotOperationLease::NewOperationBlockToken token =
            RobotOperationLease::AddNewOperationsBlock(QStringLiteral("正在固化机器人运动准备策略，请稍后启动操作。"));
        ~SelectionBlock() { RobotOperationLease::RemoveNewOperationsBlock(token); }
    } selectionBlock;
    if (RobotOperationLease::AnyActive() || RobotOperationLease::MotionCompletionPending(this)
        || m_dataStreamEnabled.load() || m_dataStreamEntryMode.load() >= 0
        || m_trajectoryRunning.load() || m_nativeProgramRunning.load() || m_continuousJogRunning.load())
    { SetLastRobotError("机器人存在活动操作或未完成的数据流，禁止更换固化策略；停止并确认恢复后再选用。"); return false; }
    std::lock_guard<std::mutex> lock(m_modePreparationMutex);
    const auto epoch = m_modeConnectionEpoch.load();
    if (!IsConnected() || m_modePreparationBindingEpoch != epoch)
    { SetLastRobotError("当前连接型号/固件尚未核对，不能固化组合。"); return false; }
    if (m_persistedModePreparation == id && m_activeModePreparation == id)
    { return true; }
    const auto found = m_verifiedModePreparations.find(id);
    const auto record = m_modePreparationTestRecords.find(id);
    if (found == m_verifiedModePreparations.end() || found->second != epoch
        || record == m_modePreparationTestRecords.end())
    { SetLastRobotError("该组合尚未在当前连接中通过准备和恢复验证，不能固化；历史报告不会自动转换为策略。"); return false; }
    QString error;
    if (!RobotModePreparationStore::SaveVerified(m_modePreparationBinding, record->second, error))
    { SetLastRobotError("组合固化保存/回读失败，未选用：" + error.toStdString()); return false; }
    // Disk contains a verified recipe, not a grant for a disconnected session.
    if (!IsConnected() || epoch != m_modeConnectionEpoch.load())
    { SetLastRobotError("组合已保存，但连接已变化；本连接未激活，重连后将重新核对身份。"); return false; }
    m_activeModePreparation = id;
    m_persistedModePreparation = id;
    m_modePreparationLoadError.clear();
    return true;
}

std::string InovanceRobotCtrl::ActiveModePreparationId() const
{
    std::lock_guard<std::mutex> lock(m_modePreparationMutex);
    return m_connectionReady.load() && m_modePreparationBindingEpoch == m_modeConnectionEpoch.load()
        && !m_persistedModePreparation.empty() && m_activeModePreparation == m_persistedModePreparation
        ? m_activeModePreparation : std::string();
}

bool InovanceRobotCtrl::SetDataStreamMode(const char* action, int expectedMode)
{
    const auto epoch = m_modeConnectionEpoch.load();
    if (action == nullptr)
    {
        SetLastRobotError("汇川数据流模式动作为空。");
        return false;
    }
    const auto queryStream = [this, epoch](int& mode)
    {
        std::lock_guard<std::mutex> socketLock(m_socketMutex);
        if (!IsConnected() || epoch != m_modeConnectionEpoch.load())
        { SetLastRobotError("数据流操作期间连接已变化，未跨连接执行。"); return false; }
        return QueryIntLocked("Get_DsMode", mode);
    };
    int currentMode = -1;
    if (!queryStream(currentMode)) { return false; }
    if (std::string(action) == "ON")
    {
        if (!EnsureControlPermit()) { return false; }
        InovanceModeSequence::Plan plan;
        if (!InovanceModeSequence::Find(ActiveModePreparationId(), plan))
        {
            std::lock_guard<std::mutex> modeLock(m_modePreparationMutex);
            SetLastRobotError("汇川数据流准备策略不可用：" + m_modePreparationLoadError
                + " 请在适配验收流程4测试通过后选用固化；已固化策略在同一配置下重启/重连会自动恢复。");
            return false;
        }
        if (currentMode == 1 && m_dataStreamEntryMode.load() >= 0)
        {
            InovanceModeSequence::State owned;
            std::string trace;
            const auto ops = ModeSequenceOps(epoch);
            if (InovanceModeSequence::Read(ops, owned, trace, "已归属数据流回读")
                && InovanceModeSequence::RuntimeReady(owned, plan.mode) && owned.stream == 1)
            { return IsConnected() && epoch == m_modeConnectionEpoch.load(); }
            SetLastRobotError("汇川已归属数据流不再满足已验收模式/伺服常驻运行条件：\n" + trace);
            return false;
        }
        if (currentMode != 0)
        { SetLastRobotError("汇川存在未归属本次准备的数据流，禁止接管或清空已有运动队列。"); return false; }
        auto ops = ModeSequenceOps(epoch);
        InovanceModeSequence::State before;
        std::string trace;
        if (plan.streamFirst)
        {
            SetLastRobotError("汇川当前固化组合要求先开数据流再上电，不能用于伺服常驻生产运行；"
                "请在流程4选用同一模式的“上电→开数据流”已通过组合。");
            return false;
        }
        if (!InovanceModeSequence::Read(ops, before, trace, "数据流进入前")
            || !before.Safe() || before.stream != 0)
        { SetLastRobotError(trace); return false; }
        if (!InovanceModeSequence::OpenRuntimeStream(ops, plan.mode, trace))
        {
            SetLastRobotError("汇川生产数据流准备失败；验收证据组合=" + plan.id
                + "。连接基线保持自动和上电；直连运动按该组合切换一次模式并保持伺服，"
                "JOB执行前会独立恢复自动模式：\n" + trace);
            return false;
        }
        {
            std::lock_guard<std::mutex> socketLock(m_socketMutex);
            if (!IsConnected() || epoch != m_modeConnectionEpoch.load())
            { SetLastRobotError("数据流准备完成时连接已变化，未发布旧连接准备结果。"); return false; }
            m_dataStreamEntryMode.store(plan.mode);
            m_dataStreamEntryMotor.store(1);
            m_dataStreamEnabled.store(true);
        }
        return true;
    }
    if (expectedMode == 0 && m_dataStreamEntryMode.load() >= 0
        && m_dataStreamEntryMotor.load() == 1)
    {
        std::string trace;
        const int runtimeMode = m_dataStreamEntryMode.load();
        if (!InovanceModeSequence::CloseRuntimeStream(ModeSequenceOps(epoch), runtimeMode, trace))
        {
            SetLastRobotError("汇川生产数据流关闭失败；未执行模式切换或伺服下电：\n" + trace);
            return false;
        }
        std::lock_guard<std::mutex> socketLock(m_socketMutex);
        if (!IsConnected() || epoch != m_modeConnectionEpoch.load()) { return false; }
        m_dataStreamEnabled.store(false);
        m_dataStreamEntryMode.store(-1);
        m_dataStreamEntryMotor.store(-1);
        return true;
    }
    if (expectedMode == 0 && m_dataStreamEntryMode.load() >= 0
        && m_dataStreamEntryMotor.load() == 0)
    {
        InovanceModeSequence::Plan plan;
        if (!InovanceModeSequence::Find(ActiveModePreparationId(), plan))
        { SetLastRobotError("汇川退出数据流时组合身份失效，需安全中止并检查连接。"); return false; }
        if (plan.powerOffFirst && currentMode != 0)
        {
            std::string trace;
            const auto ops = ModeSequenceOps(epoch);
            if (!InovanceModeSequence::Step(ops, "Motor OFF",
                [](const InovanceModeSequence::State& state) { return state.motor == 0; }, trace, true))
            {
                const bool restored = InovanceModeSequence::RestoreOff(ops, m_dataStreamEntryMode.load(), trace);
                SetLastRobotError("汇川选定的先下电退出顺序失败，恢复="
                    + std::string(restored ? "OK\n" : "FAIL\n") + trace);
                return false;
            }
        }
    }
    if (currentMode == expectedMode)
    {
        std::lock_guard<std::mutex> socketLock(m_socketMutex);
        if (!IsConnected() || epoch != m_modeConnectionEpoch.load()) { return false; }
        m_dataStreamEnabled.store(currentMode != 0);
    }
    else
    {
        std::string response;
        if (!ModeSequenceOps(epoch).send(std::string("Dsmode ") + action, response))
        { SetLastRobotError(response); return false; }
        bool verified = false;
        for (int attempt = 0; attempt < 20; ++attempt)
        {
            if (!queryStream(currentMode)) { return false; }
            if (currentMode == expectedMode)
            { verified = true; break; }
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
        }
        if (!verified)
        { SetLastRobotError("汇川数据流模式动作未通过Get_DsMode回读确认：期望=" + std::to_string(expectedMode)
            + "，实际=" + std::to_string(currentMode) + "。"); return false; }
        std::lock_guard<std::mutex> socketLock(m_socketMutex);
        if (!IsConnected() || epoch != m_modeConnectionEpoch.load()) { return false; }
        m_dataStreamEnabled.store(currentMode != 0);
    }
    if (expectedMode == 0 && m_dataStreamEntryMode.load() >= 0)
    {
        const int originalMode = m_dataStreamEntryMode.load();
        std::string trace;
        // A test started with the motor OFF must power down before restoring
        // its mode, exactly as witnessed in the no-motion combination test.
        const bool restored = m_dataStreamEntryMotor.load() == 0
            ? InovanceModeSequence::RestoreOff(ModeSequenceOps(epoch), originalMode, trace)
            : InovanceModeSequence::SetMode(ModeSequenceOps(epoch), originalMode, trace, true);
        if (!restored)
        { SetLastRobotError("汇川数据流关闭后模式恢复失败：" + trace); return false; }
        std::lock_guard<std::mutex> socketLock(m_socketMutex);
        if (!IsConnected() || epoch != m_modeConnectionEpoch.load()) { return false; }
        m_dataStreamEntryMode.store(-1);
        m_dataStreamEntryMotor.store(-1);
    }
    return true;
}

bool InovanceRobotCtrl::LoginUserLocked()
{
    m_userLoggedIn.store(false);
    InovanceUserLogin::Ops ops;
    ops.send = [this](const std::string& command, std::string& response)
    { return SendCommandLocked(command, response); };
    ops.query = [this](const std::string& command, int& value)
    { return QueryIntLocked(command, value); };
    std::string error;
    if (!InovanceUserLogin::Login({m_apiUserLevel, m_apiPassword}, ops, error))
    {
        // Diagnostic queries are read-only and never establish login success.
        // Keep the original failure before these queries change LastRobotError.
        std::string firmware;
        std::string response;
        if (SendCommandLocked("Get_FwVersion", response))
        {
            const auto value = ValuePart(response);
            if (value.size() >= 2 && value.size() <= 64 && value.front() == 'V'
                && value.find_first_not_of("ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789._-+")
                    == std::string::npos) { firmware = value; }
        }
        int controlDevice = -1;
        int currentUser = -1;
        int permitOwner = -1;
        if (!QueryIntLocked("CurCtrlDev", controlDevice) || controlDevice < 0 || controlDevice > 2)
        { controlDevice = -1; }
        if (!QueryIntLocked("CurUserType", currentUser) || currentUser < 0 || currentUser > 3)
        { currentUser = -1; }
        if (SendCommandLocked("CurPermit", response))
        {
            std::istringstream input(ValuePart(response));
            int value = -1;
            if (input >> value && value >= 0 && value <= 2) { permitOwner = value; }
        }
        error += "\n登录诊断：固件=" + (firmware.empty() ? std::string("读取失败") : firmware)
            + "，控制设备=" + std::to_string(controlDevice)
            + "，API用户级别=" + std::to_string(currentUser)
            + "，许可归属=" + std::to_string(permitOwner)
            + "（-1表示读取失败；API登录独立于示教器登录）。";
        m_loginRetry.Block(error);
        SetLastRobotError(m_loginRetry.Error());
        if (m_pRobotLog != nullptr)
        { m_pRobotLog->write(LogColor::ERR, "%s", m_loginRetry.Error().c_str()); }
        return false;
    }
    m_userLoggedIn.store(true);
    return true;
}

bool InovanceRobotCtrl::InitializeAfterConnect(std::string* summary)
{
    if (summary != nullptr) { summary->clear(); }
    if (!IsConnected())
    { SetLastRobotError("汇川连接后初始化要求已建立并验证2222连接；不会在初始化中自动重连。 "); return false; }
    if (RobotOperationLease::CurrentOwner(this).isEmpty())
    {
        SetLastRobotError("汇川连接后初始化只能由持有机器人操作租约的显式连接流程调用；后台重连不会自动上电。");
        return false;
    }
    const std::uint64_t epoch = m_modeConnectionEpoch.load();
    InovanceConnectionPreparation::Ops ops;
    ops.cancelled = [this, epoch]()
    {
        return epoch != m_modeConnectionEpoch.load() || !IsConnected()
            || RobotOperationLease::IsCancellationRequested(this);
    };
    ops.delay = []() { std::this_thread::sleep_for(std::chrono::milliseconds(25)); };
    ops.read = [this, epoch](InovanceConnectionPreparation::State& state, std::string& error)
    {
        std::lock_guard<std::mutex> socketLock(m_socketMutex);
        if (!IsConnected() || epoch != m_modeConnectionEpoch.load())
        { error = "连接已变化，禁止在新连接继续旧初始化。"; return false; }
        std::string permitReply;
        int permit = -1;
        bool permitOk = SendCommandLocked("CurPermit", permitReply);
        if (permitOk)
        {
            std::istringstream input(ValuePart(permitReply));
            permitOk = static_cast<bool>(input >> permit) && permit >= 0 && permit <= 2;
        }
        const bool ok = QueryIntLocked("Get_Mode", state.mode)
            && QueryIntLocked("Get_MotorSts", state.motor)
            && QueryIntLocked("Get_DsMode", state.stream)
            && QueryIntLocked("Get_MotionSts", state.motion)
            && QueryIntLocked("Get_TaskRunSts 0", state.task)
            && QueryIntLocked("Get_EStopSts", state.estop)
            && QueryIntLocked("Get_SysErrSts", state.fault)
            && QueryIntLocked("CurCtrlDev", state.controlDevice)
            && permitOk && IsConnected() && epoch == m_modeConnectionEpoch.load();
        state.permit = permit;
        if (permitOk) { m_permitOwned.store(permit == 1); }
        error = ok ? std::string() : GetLastRobotError();
        if (!permitOk && error.empty()) { error = "CurPermit返回格式无效。"; }
        return ok;
    };
    ops.send = [this, epoch](const std::string& command, std::string& reply)
    {
        std::lock_guard<std::mutex> socketLock(m_socketMutex);
        if (!IsConnected() || epoch != m_modeConnectionEpoch.load())
        { reply = "连接已变化，命令未发送。"; return false; }
        std::string response;
        const bool ok = SendCommandLocked(command, response) && response == "ok"
            && IsConnected() && epoch == m_modeConnectionEpoch.load();
        reply = ok ? response : (GetLastRobotError().empty() ? response : GetLastRobotError());
        return ok;
    };
    ops.prepareCoordinates = [this, epoch](std::string& evidence)
    {
        if (m_toolNo != kApplicationGunToolNumber)
        {
            evidence = "应用焊枪固定使用已标定Tool1，但数据库ToolNo="
                + std::to_string(m_toolNo) + "；拒绝选择其他工具。";
            return false;
        }
        if (m_wobjNo != 1)
        {
            evidence = "汇川现场流程固定使用已标定Wobj1，但数据库WobjNo="
                + std::to_string(m_wobjNo) + "；拒绝选择其他工件坐标。";
            return false;
        }
        std::lock_guard<std::mutex> socketLock(m_socketMutex);
        const auto stillSafe = [this, epoch](std::string& error)
        {
            int motion = -1, task = -1, stream = -1, estop = -1, fault = -1, controlDevice = -1;
            std::string permitReply;
            int permit = -1;
            if (!IsConnected() || epoch != m_modeConnectionEpoch.load()
                || !QueryIntLocked("Get_MotionSts", motion)
                || !QueryIntLocked("Get_TaskRunSts 0", task)
                || !QueryIntLocked("Get_DsMode", stream)
                || !QueryIntLocked("Get_EStopSts", estop)
                || !QueryIntLocked("Get_SysErrSts", fault)
                || !QueryIntLocked("CurCtrlDev", controlDevice)
                || !SendCommandLocked("CurPermit", permitReply))
            { error = GetLastRobotError(); return false; }
            std::istringstream input(ValuePart(permitReply));
            if (!(input >> permit) || motion != 0 || (task != 0 && task != 10)
                || stream != 0 || estop != 0 || fault != 0 || controlDevice != 2 || permit != 1
                || epoch != m_modeConnectionEpoch.load())
            { error = "坐标设置前实时状态不再满足静止、无急停/报警、远程许可及数据流关闭条件。"; return false; }
            return true;
        };
        if (!stillSafe(evidence)) { return false; }
        const auto setAndVerify = [this, epoch, &stillSafe](const char* setName, const char* getName,
            int configured, const char* label, std::string& error)
        {
            if (!stillSafe(error)) { return false; }
            std::string response;
            if (!SendCommandLocked(std::string(setName) + " " + std::to_string(configured), response)
                || response != "ok")
            { error = std::string(label) + "设置失败：" + GetLastRobotError(); return false; }
            int actual = -1;
            if (!QueryIntLocked(getName, actual) || actual != configured
                || epoch != m_modeConnectionEpoch.load())
            { error = std::string(label) + "回读与配置不一致，期望=" + std::to_string(configured)
                    + "，实际=" + std::to_string(actual) + "。"; return false; }
            return true;
        };
        if (!setAndVerify("Set_ToolCNum", "Get_ToolCNum", m_toolNo, "工具号", evidence)
            || !setAndVerify("Set_WobjNum", "Get_WobjNum", m_wobjNo, "工件号", evidence))
        { return false; }
        evidence = "Tool=" + std::to_string(m_toolNo) + "，Wobj=" + std::to_string(m_wobjNo) + "，写后回读一致。";
        return true;
    };
    ops.prepareKinematics = [this, epoch](std::string& evidence)
    {
        if (!IsConnected() || epoch != m_modeConnectionEpoch.load())
        {
            evidence = "连接已变化，未读取运动学资产。";
            return false;
        }
        if (m_kinematicsSession.Ready())
        {
            evidence = "本连接的运动学资产已经过实时校验，SKIP。";
            return true;
        }
        RobotKinematicsValidationResult validation;
        if (!RefreshKinematicsFromController(validation))
        {
            evidence = GetLastRobotError().empty()
                ? "中断态恢复后运动学资产实时校验失败。" : GetLastRobotError();
            return false;
        }
        evidence = validation.acquisitionSummary.empty()
            ? "中断态恢复后已重新实时校验。" : validation.acquisitionSummary;
        return IsConnected() && epoch == m_modeConnectionEpoch.load();
    };

    std::string evidence;
    const bool ok = InovanceConnectionPreparation::Prepare(ops, evidence);
    if (summary != nullptr) { *summary = evidence; }
    if (!ok)
    {
        SetLastRobotError("汇川连接已建立，但前置初始化失败（未启动运动）：\n" + evidence);
        return false;
    }
    m_permitOwned.store(true);
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

    std::string response;
    if (!ServoOff())
    {
        rememberFailure("汇川断开前安全停止和Motor OFF失败。");
        ok = false;
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
    if (m_trajectoryRunning.load() || m_nativeProgramRunning.load()
        || m_continuousJogRunning.load() || RobotOperationLease::MotionCompletionPending(this)
        || !RobotOperationLease::CurrentOwner(this).isEmpty())
    {
        SetLastRobotError("汇川运动或原生JOB运行期间禁止重载机器人配置；"
            "避免工具、工件坐标和可选关弧IO见证与已启动JOB发生变化。");
        if (m_pRobotLog != nullptr)
        {
            m_pRobotLog->write(LogColor::ERR, "%s", GetLastRobotError().c_str());
        }
        return;
    }
    bool wasConnected = false;
    {
        std::lock_guard<std::mutex> refreshLock(m_kinematicsRefreshMutex);
        InovanceKinematicsReadScope readOnlyScope(m_kinematicsReadInProgress);
        std::lock_guard<std::mutex> socketLock(m_socketMutex);
        wasConnected = m_connectionReady.load();
        int motion = -1;
        if (m_trajectoryRunning.load() || m_nativeProgramRunning.load()
            || m_continuousJogRunning.load() || RobotOperationLease::MotionCompletionPending(this)
            || !RobotOperationLease::CurrentOwner(this).isEmpty()
            || (wasConnected && (!QueryIntLocked("Get_MotionSts", motion) || motion != 0)))
        {
            SetLastRobotError("汇川仍有活动操作或未确认停止，拒绝断开并重载配置。");
            return;
        }
        // Endpoint/authentication/FTP changes must never leave the old live
        // socket paired with the newly loaded calibration source.
        CloseSocketLocked();
        InitRobotDriver(m_sRobotName);
    }
    if (wasConnected) { ConnectWithPolicy(false); }
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

bool InovanceRobotCtrl::ServoOff()
{
    if (!IsConnected() && !Connect()) { return false; }
    if (!EnsureControlPermit()) { return false; }
    const auto epoch = m_modeConnectionEpoch.load();

    int motion = -1;
    int dataStreamMode = -1;
    int taskStatus = -1;
    if (!QueryInt("Get_MotionSts", motion)
        || !QueryInt("Get_DsMode", dataStreamMode)
        || !QueryInt("Get_TaskRunSts 0", taskStatus))
    {
        return false;
    }
    if (motion != 0 || dataStreamMode != 0 || taskStatus == 1
        || m_nativeProgramRunning.load())
    {
        if (!AbortCurrentProgramSafely())
        {
            if (GetLastRobotError().empty())
            {
                SetLastRobotError("汇川伺服下电前无法确认运动和程序已经安全中止。");
            }
            return false;
        }
    }

    int originalMode = -1;
    if (!QueryInt("Get_Mode", originalMode) || (originalMode != 1 && originalMode != 2))
    { SetLastRobotError("汇川下电前运行模式未知，未尝试切换模式。"); return false; }
    std::string trace;
    if (!InovanceModeSequence::RestoreOff(ModeSequenceOps(epoch), originalMode, trace))
    { SetLastRobotError("汇川伺服下电/恢复失败：\n" + trace); return false; }
    std::lock_guard<std::mutex> socketLock(m_socketMutex);
    if (!IsConnected() || epoch != m_modeConnectionEpoch.load()) { return false; }
    m_dataStreamEntryMode.store(-1);
    m_dataStreamEntryMotor.store(-1);
    ClearLastRobotError();
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
    return m_connected.load() && m_connectionReady.load();
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
        << " 运动=" << motion
        << " 关节运动学=" << (m_kinematicsSession.Ready() ? "实时校验就绪" : "未就绪（停止后重连或执行流程6）");
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
    // Monitoring must not wait behind the FTP calibration download.
    std::unique_lock<std::mutex> refreshLock(m_kinematicsRefreshMutex, std::try_to_lock);
    if (!refreshLock.owns_lock() || !m_connectionReady.load() || !m_kinematicsSession.Ready())
    {
        SetLastRobotError("汇川当前连接的轴单位尚未实时校验，无法返回通用关节脉冲；请停止后重连或执行流程6。厂商绝对编码器脉冲不能代替通用脉冲。");
        return false;
    }
    const std::uint64_t generation = m_kinematicsSession.Generation();
    const double mainUnits[6] = {
        m_tAxisUnit.dSPulseUnit, m_tAxisUnit.dLPulseUnit, m_tAxisUnit.dUPulseUnit,
        m_tAxisUnit.dRPulseUnit, m_tAxisUnit.dBPulseUnit, m_tAxisUnit.dTPulseUnit
    };
    const bool normalizedUnitsReady = std::all_of(
        std::begin(mainUnits), std::end(mainUnits),
        [](double unit) { return std::isfinite(unit) && std::abs(unit) >= 1e-15; });
    std::vector<double> joints;
    if (normalizedUnitsReady)
    {
        // 通用T_ANGLE_PULSE是“关节角/AxisUnit”的零点相对表示。汇川
        // Get_PosHerePulse返回绝对编码器脉冲，含零点和腕部耦合，不能直接泄漏给业务层。
        if (!QueryDoubles("Get_RobJPHere", joints, 14)) { return false; }
        for (int axis = 0; axis < 6; ++axis)
        {
            const double converted = joints[axis] / mainUnits[axis];
            if (!std::isfinite(converted) || converted < (std::numeric_limits<long>::min)()
                || converted > (std::numeric_limits<long>::max)())
            { SetLastRobotError("汇川关节值超出通用脉冲整数范围。"); return false; }
        }
        pulse = T_ANGLE_PULSE(
            static_cast<long>(std::llround(joints[0] / mainUnits[0])),
            static_cast<long>(std::llround(joints[1] / mainUnits[1])),
            static_cast<long>(std::llround(joints[2] / mainUnits[2])),
            static_cast<long>(std::llround(joints[3] / mainUnits[3])),
            static_cast<long>(std::llround(joints[4] / mainUnits[4])),
            static_cast<long>(std::llround(joints[5] / mainUnits[5])),
            0, 0, 0);
    }
    else
    {
        SetLastRobotError("汇川已验证轴单位无效，拒绝返回错误的通用关节脉冲。");
        return false;
    }

    if (m_nExternalAxleType != 0)
    {
        if (joints.empty() && !QueryDoubles("Get_RobJPHere", joints, 14)) { return false; }
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
    {
        std::lock_guard<std::mutex> socketLock(m_socketMutex);
        if (!m_connectionReady.load() || !m_kinematicsSession.Ready()
            || generation != m_kinematicsSession.Generation())
        { SetLastRobotError("汇川关节读取期间连接已改变，本次脉冲结果已作废。"); return false; }
        StorePassivePulse(pulse, SteadyMs());
    }
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
    const bool valid = m_kinematicsSession.Ready() && m_passivePulseValid;
    if (robotMs != nullptr) { *robotMs = valid ? m_passivePulsePcMs : 0; }
    if (pcRecvMs != nullptr) { *pcRecvMs = valid ? m_passivePulsePcMs : 0; }
    return valid ? m_passivePulse : T_ANGLE_PULSE();
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

RobotControllerStatus InovanceRobotCtrl::ReadControllerStatus()
{
    RobotControllerStatus status;
    status.connected = IsConnected();
    status.pcRecvMs = SteadyMs();
    if (!status.connected)
    {
        status.detail = "汇川控制器未连接。";
        return status;
    }

    int rawMode = -1;
    int emergencyStop = -1;
    int motor = -1;
    int rawMotion = -1;
    int systemErrorStatus = -1;
    int systemErrorCode = -1;
    int controlOwner = -1;
    int permit = -1;
    if (!QueryInt("Get_Mode", rawMode)
        || !QueryInt("Get_EStopSts", emergencyStop)
        || !QueryInt("Get_MotorSts", motor)
        || !QueryInt("Get_MotionSts", rawMotion)
        || !QueryInt("Get_SysErrSts", systemErrorStatus)
        || !QuerySystemErrorCode(systemErrorCode)
        || !QueryInt("CurCtrlDev", controlOwner)
        || !QueryLeadingInt("CurPermit", permit))
    {
        status.detail = GetLastRobotError().empty()
            ? "汇川结构化状态读取不完整。" : GetLastRobotError();
        return status;
    }

    status.rawOperationMode = rawMode;
    if (rawMode == 1) { status.operationMode = RobotOperationMode::Manual; }
    else if (rawMode == 2) { status.operationMode = RobotOperationMode::Automatic; }
    status.emergencyStopKnown = true;
    status.emergencyStop = emergencyStop != 0;
    status.servoPowerKnown = true;
    status.servoPowered = motor == 1;
    status.systemFaultKnown = true;
    status.systemFault = (systemErrorStatus & 0x1) != 0;
    status.systemWarning = (systemErrorStatus & 0x2) != 0;
    status.systemErrorCode = systemErrorCode;
    status.controlOwnerKnown = true;
    status.controlOwnedByApi = controlOwner == 2;
    status.rawControlOwner = controlOwner;
    status.controlPermitKnown = true;
    status.hasControlPermit = permit == 1;
    status.rawPermitState = permit;
    status.motion.rawCode = rawMotion;
    if (rawMotion == 0)
    {
        status.motion.state = RobotMotionState::Idle;
        status.motion.terminalVerified = true;
        status.motion.detail = "汇川机器人已停止";
    }
    else if (rawMotion == 1)
    {
        status.motion.state = RobotMotionState::Running;
        status.motion.detail = "汇川机器人运动中";
    }
    else if (rawMotion == 2)
    {
        status.motion.state = RobotMotionState::Interrupted;
        status.motion.detail = "汇川机器人运动中断";
    }
    else
    {
        status.motion.state = RobotMotionState::Unknown;
        status.motion.detail = "汇川未知运动状态=" + std::to_string(rawMotion);
    }
    status.valid = true;
    std::ostringstream detail;
    detail << "模式=" << rawMode
        << " 急停=" << emergencyStop
        << " 伺服=" << motor
        << " 运动=" << rawMotion
        << " 故障状态=" << systemErrorStatus
        << " 故障码=0x" << std::hex << std::uppercase << systemErrorCode
        << std::dec << " 控制设备=" << controlOwner
        << " 许可=" << permit;
    status.detail = detail.str();
    ClearLastRobotError();
    return status;
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

    // Direct Move* and data-stream trajectories share the same frozen final
    // command identity. Wait on that identity here (not inside Move*) so scan
    // and calibration workers can collect data while the robot is moving.
    int trackedCommandId = -1;
    {
        std::lock_guard<std::mutex> lock(m_trajectoryMutex);
        if (m_activeHandle.started
            && !IsInovanceNativeTrajectoryPurpose(m_preparedPurpose))
        {
            trackedCommandId = m_finalCommandId;
        }
    }
    if (trackedCommandId >= 0)
    {
        if (!WaitForCommandDone(trackedCommandId, delayMs, runTimeoutMs))
        {
            return 0;
        }
        return FinalizeCompletedDataStreamMotion() ? 1 : 0;
    }

    const long long deadline = SteadyMs() + runTimeoutMs;
    while (SteadyMs() < deadline)
    {
        const RobotMotionStatus status = ReadMotionStatus();
        if ((status.state == RobotMotionState::Completed && status.terminalVerified)
            || status.state == RobotMotionState::Idle)
        {
            if (!FinalizeCompletedDataStreamMotion())
            {
                return 0;
            }
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

bool InovanceRobotCtrl::SendCircularMove(
    const T_ROBOT_COORS& via,
    const T_ROBOT_COORS& target,
    double speedMmPerMin,
    int zone,
    const int* viaConfiguration,
    const int* targetConfiguration,
    int* commandId)
{
    std::string validationError;
    if (!ValidateLinearSpeedMmPerMin(speedMmPerMin, &validationError))
    {
        SetLastRobotError(validationError);
        return false;
    }
    const double components[18] = {
        via.dX, via.dY, via.dZ, via.dRX, via.dRY, via.dRZ,
        via.dBX, via.dBY, via.dBZ,
        target.dX, target.dY, target.dZ, target.dRX, target.dRY, target.dRZ,
        target.dBX, target.dBY, target.dBZ
    };
    if (!std::all_of(std::begin(components), std::end(components),
        [](double value) { return std::isfinite(value); }))
    {
        SetLastRobotError("汇川圆弧运动中间点或目标点包含非有限坐标。");
        return false;
    }

    int arm[4] = {};
    double baseExternal[6] = {};
    {
        std::lock_guard<std::mutex> lock(m_passiveMutex);
        std::copy(std::begin(m_armConfig), std::end(m_armConfig), arm);
        std::copy(std::begin(m_externalValues), std::end(m_externalValues), baseExternal);
    }
    // 通用 configuration 不表达汇川 ArmType；与 MOVL 相同，复用当前位置的真实 ArmType。
    (void)viaConfiguration;
    (void)targetConfiguration;

    const auto makeParameter = [this, &arm, &baseExternal](const T_ROBOT_COORS& pose)
    {
        double external[6] = {};
        std::copy(std::begin(baseExternal), std::end(baseExternal), external);
        if (m_nExternalAxleType & 1) { external[0] = pose.dBX; }
        if (m_nExternalAxleType & 2) { external[1] = pose.dBY; }
        if (m_nExternalAxleType & 4) { external[2] = pose.dBZ; }
        std::ostringstream parameter;
        parameter << '['
            << FormatDouble(pose.dX) << ',' << FormatDouble(pose.dY) << ','
            << FormatDouble(pose.dZ) << ','
            << FormatDouble(pose.dRZ) << ',' << FormatDouble(pose.dRY) << ','
            << FormatDouble(pose.dRX) << "; "
            << arm[0] << ',' << arm[1] << ',' << arm[2] << ',' << arm[3] << "; "
            << FormatDouble(external[0]) << ',' << FormatDouble(external[1]) << ','
            << FormatDouble(external[2]) << ',' << FormatDouble(external[3]) << ','
            << FormatDouble(external[4]) << ',' << FormatDouble(external[5]) << ']';
        return parameter.str();
    };
    const std::string viaParameter = makeParameter(via);
    const std::string targetParameter = makeParameter(target);
    if (viaParameter.size() > 128 || targetParameter.size() > 128)
    {
        SetLastRobotError("汇川MovCRobP中间点或目标点参数超过手册规定的128字符上限。");
        return false;
    }

    const double speedMmPerSecond = speedMmPerMin / 60.0;
    std::ostringstream command;
    command << "MovCRobP " << viaParameter << ' ' << targetParameter << ' '
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
        SetLastRobotError("汇川MovCRobP返回ok，但Get_CurCmdNum未产生新的指令编号，拒绝伪造完成身份。");
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
    std::unique_lock<std::mutex> refreshLock(m_kinematicsRefreshMutex, std::try_to_lock);
    if (!refreshLock.owns_lock() || !m_connectionReady.load() || !m_kinematicsSession.Ready())
    {
        SetLastRobotError("汇川当前连接未完成轴单位实时校验，关节运动未发送。");
        return false;
    }
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
    {
        std::lock_guard<std::mutex> socketLock(m_socketMutex);
        if (!m_connectionReady.load() || !m_kinematicsSession.Ready())
        { SetLastRobotError("汇川关节运动发送前连接已改变，目标已作废。"); return false; }
        if (!SendCommandLocked(command.str(), response) || response != "ok") { return false; }
    }
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
    int stableInterrupted = 0;
    while (SteadyMs() < deadline)
    {
        int done = 0;
        if (!QueryInt("Get_CmdSts " + std::to_string(commandId), done)) { return false; }
        int motion = -1;
        if (!QueryInt("Get_MotionSts", motion)) { return false; }
        if (motion == 2)
        {
            // The controller can expose a single stale/intermediate interrupted
            // sample while the data stream is closed and reopened between two
            // direct moves.  Do not release the frozen command identity on one
            // sample: the field trace showed the same command return to running
            // immediately afterwards.  A real interruption remains fail-closed
            // after three consecutive confirmations and no completion witness.
            ++stableInterrupted;
            stableDone = 0;
            if (stableInterrupted >= 3)
            {
                SetLastRobotError(
                    "汇川运动中断状态已连续确认3次，指令编号="
                    + std::to_string(commandId)
                    + "，Get_CmdSts=" + std::to_string(done)
                    + "，Get_MotionSts=2。");
                return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(pollDelayMs));
            continue;
        }
        stableInterrupted = 0;
        stableDone = (done == 1 && motion == 0) ? stableDone + 1 : 0;
        if (stableDone >= 2) { return true; }
        std::this_thread::sleep_for(std::chrono::milliseconds(pollDelayMs));
    }
    SetLastRobotError("汇川运动等待Get_CmdSts精确到位超时。");
    return false;
}

bool InovanceRobotCtrl::BeginTrackedDirectMotion(
    int commandId,
    const char* operationName)
{
    if (commandId < 0)
    {
        SetLastRobotError("汇川单点运动已受理但没有取得有效指令编号，禁止报告启动成功。");
        return false;
    }

    std::lock_guard<std::mutex> lock(m_trajectoryMutex);
    if (m_trajectoryRunning.load() || m_nativeProgramRunning.load()
        || m_continuousJogRunning.load())
    {
        SetLastRobotError("汇川上一项运动仍由适配层跟踪，禁止覆盖单点运动身份。");
        return false;
    }

    RobotTrajectoryHandle handle;
    handle.programName = "INOVANCE_DIRECT_"
        + std::string(operationName != nullptr ? operationName : "MOVE")
        + "_" + std::to_string(++m_trajectoryCounter);
    handle.prepared = true;
    handle.started = true;
    m_preparedMoveInfos.clear();
    m_preparedPurpose = RobotTrajectoryPurpose::ScanDryRun;
    m_preparedFingerprint = 0;
    m_activeHandle = handle;
    m_finalCommandId = commandId;
    m_trajectoryRunning.store(true);
    m_trajectoryPaused.store(false);
    ClearLastRobotError();
    return true;
}

bool InovanceRobotCtrl::FinalizeCompletedDataStreamMotion()
{
    bool hasTrackedDataStreamMotion = false;
    {
        std::lock_guard<std::mutex> lock(m_trajectoryMutex);
        hasTrackedDataStreamMotion = m_activeHandle.started
            && !IsInovanceNativeTrajectoryPurpose(m_preparedPurpose)
            && m_finalCommandId >= 0;
    }
    if (!hasTrackedDataStreamMotion)
    {
        return true;
    }
    if (!SetDataStreamMode("OFF", 0))
    {
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(m_trajectoryMutex);
        m_trajectoryRunning.store(false);
        m_trajectoryPaused.store(false);
        m_activeHandle.started = false;
        m_finalCommandId = -1;
    }
    ClearLastRobotError();
    return true;
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
    if (!SetDataStreamMode("ON", 1) || !EnsureMotionReady()) { return false; }
    // 首次运动前必须取得真实ArmType和未暴露的E4..E6，防止以默认值覆盖控制器位形。
    T_ROBOT_COORS current;
    if (!ReadCartesianPosition(current, nullptr))
    {
        SetDataStreamMode("OFF", 0);
        return false;
    }
    int commandId = -1;
    const bool sent = SendCartesianMove(target, speedMmPerMin, -1, configuration, &commandId);
    if (sent && BeginTrackedDirectMotion(commandId, "MOVL"))
    {
        return true;
    }
    const std::string motionError = GetLastRobotError();
    if (sent)
    {
        const bool stopped = AbortCurrentProgramSafely();
        SetLastRobotError(motionError + (stopped
            ? "；运动身份冻结失败后已执行可验证安全中止。"
            : "；运动身份冻结失败且安全中止未确认：" + GetLastRobotError()));
    }
    else
    {
        SetDataStreamMode("OFF", 0);
        if (!motionError.empty()) { SetLastRobotError(motionError); }
    }
    return false;
}

bool InovanceRobotCtrl::MoveCircularMmPerMin(
    const T_ROBOT_COORS& via,
    const T_ROBOT_COORS& target,
    double speedMmPerMin,
    int externalAxleType,
    const int* viaConfiguration,
    const int* targetConfiguration)
{
    if (externalAxleType != m_nExternalAxleType)
    {
        SetLastRobotError("汇川圆弧运动外部轴类型与当前驱动配置不一致。");
        return false;
    }
    if (!SetDataStreamMode("ON", 1) || !EnsureMotionReady()) { return false; }
    T_ROBOT_COORS current;
    if (!ReadCartesianPosition(current, nullptr))
    {
        SetDataStreamMode("OFF", 0);
        return false;
    }
    int commandId = -1;
    const bool sent = SendCircularMove(
        via, target, speedMmPerMin, -1,
        viaConfiguration, targetConfiguration, &commandId);
    if (sent && BeginTrackedDirectMotion(commandId, "MOVC"))
    {
        return true;
    }
    const std::string motionError = GetLastRobotError();
    if (sent)
    {
        const bool stopped = AbortCurrentProgramSafely();
        SetLastRobotError(motionError + (stopped
            ? "；运动身份冻结失败后已执行可验证安全中止。"
            : "；运动身份冻结失败且安全中止未确认：" + GetLastRobotError()));
    }
    else
    {
        SetDataStreamMode("OFF", 0);
        if (!motionError.empty()) { SetLastRobotError(motionError); }
    }
    return false;
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
    if (!SetDataStreamMode("ON", 1) || !EnsureMotionReady()) { return false; }
    T_ROBOT_COORS current;
    if (!ReadCartesianPosition(current, nullptr))
    {
        SetDataStreamMode("OFF", 0);
        return false;
    }
    int commandId = -1;
    const bool sent = SendJointMove(target, speedPercent, -1, &commandId);
    if (sent && BeginTrackedDirectMotion(commandId, "MOVJ"))
    {
        return true;
    }
    const std::string motionError = GetLastRobotError();
    if (sent)
    {
        const bool stopped = AbortCurrentProgramSafely();
        SetLastRobotError(motionError + (stopped
            ? "；运动身份冻结失败后已执行可验证安全中止。"
            : "；运动身份冻结失败且安全中止未确认：" + GetLastRobotError()));
    }
    else
    {
        SetDataStreamMode("OFF", 0);
        if (!motionError.empty()) { SetLastRobotError(motionError); }
    }
    return false;
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
        if (move.bHasWeaveParam)
        {
            const int weaveIntegers[] = {
                move.tWeaveParam.nWeaveType,
                move.tWeaveParam.nWeaveShape,
                move.tWeaveParam.nPauseTime1Ms,
                move.tWeaveParam.nPauseTime2Ms,
                move.tWeaveParam.nPauseTime3Ms,
                move.tWeaveParam.nPauseTime4Ms,
                move.tWeaveParam.nPauseContinue
            };
            const double weaveValues[] = {
                move.tWeaveParam.dWeaveFrequencyHz,
                move.tWeaveParam.dWeaveAmplitudeMm,
                move.tWeaveParam.dSwingDirectionDeg,
                move.tWeaveParam.dWeavePlaneAngleDeg,
                move.tWeaveParam.dSpaceAngleDeg,
                move.tWeaveParam.dEndLengthMm,
                move.tWeaveParam.dEndWidthMm,
                move.tWeaveParam.dCenterHeightMm
            };
            mixBytes(weaveIntegers, sizeof(weaveIntegers));
            mixBytes(weaveValues, sizeof(weaveValues));
        }
        if (move.bHasTrackParam)
        {
            const int trackIntegers[] = {
                move.tTrackParam.nLateralBeginCycle,
                move.tTrackParam.nVerticalModeFlag,
                move.tTrackParam.nVerticalBeginCycle,
                move.tTrackParam.nVerticalSustainCycle,
                move.tTrackParam.nTimeOrDistanceMode,
                move.tTrackParam.nTimeIntervalMs,
                move.tTrackParam.nDistanceIntervalMm
            };
            const double trackValues[] = {
                move.tTrackParam.dLateralGain,
                move.tTrackParam.dLeftAreaCoefficient,
                move.tTrackParam.dRightAreaCoefficient,
                move.tTrackParam.dVerticalReferenceCurrent,
                move.tTrackParam.dVerticalCycleLength,
                move.tTrackParam.dVerticalGain,
                move.tTrackParam.dLateralMinCompPerCycle,
                move.tTrackParam.dLateralMaxCompPerCycle,
                move.tTrackParam.dLateralMaxCompTotal,
                move.tTrackParam.dLateralAsymmetryCoefficient,
                move.tTrackParam.dVerticalMinCompPerCycle,
                move.tTrackParam.dVerticalMaxCompPerCycle,
                move.tTrackParam.dVerticalMaxCompTotal,
                move.tTrackParam.dVerticalAsymmetryCoefficient
            };
            mixBytes(trackIntegers, sizeof(trackIntegers));
            mixBytes(trackValues, sizeof(trackValues));
        }
    }
    const int nativeProgramSettings[] = { m_toolNo, m_wobjNo };
    mixBytes(nativeProgramSettings, sizeof(nativeProgramSettings));
    return hash;
}

bool InovanceRobotCtrl::HasVerifiedWeldJobContract(std::string* error) const
{
    if (m_toolNo != kApplicationGunToolNumber || m_wobjNo != 1)
    {
        if (error != nullptr)
        {
            *error = "汇川原生焊接JOB必须使用已标定Tool1和Wobj1。";
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
    bool arcActive = false;
    int arcSegments = 0;
    for (std::size_t index = 0; index < moveInfos.size(); ++index)
    {
        const T_ROBOT_MOVE_INFO& move = moveInfos[index];
        const double poseValues[] = {
            move.tCoord.dX, move.tCoord.dY, move.tCoord.dZ,
            move.tCoord.dRX, move.tCoord.dRY, move.tCoord.dRZ,
            move.tCoord.dBX, move.tCoord.dBY, move.tCoord.dBZ
        };
        if ((move.nMoveType == MOVL || (nativeJob && move.nPosType == POSVAR))
            && !std::all_of(std::begin(poseValues), std::end(poseValues),
                [](double value) { return std::isfinite(value); }))
        {
            error = "汇川第" + std::to_string(index + 1) + "个直线点包含非有限位姿。";
            return false;
        }
        if (move.nMoveType == MOVL && move.nPosType != POSVAR)
        {
            error = "汇川第" + std::to_string(index + 1)
                + "个MOVL点必须提供笛卡尔POSVAR位姿。";
            return false;
        }
        if (nativeJob && move.nMoveType == MOVJ && move.nPosType != POSVAR)
        {
            error = "汇川原生JOB暂不覆盖工程全局JP.pts；MOVJ请提供POSVAR，底层将生成Movj LP局部点。";
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
            error = "汇川现场已确认ArcTrackData引用指令，但尚未取得ArcTrackData变量文件格式；"
                "当前拒绝忽略通用TRACKDATA参数，请先关闭跟踪或补充控制器导出的跟踪数据文件。";
            return false;
        }
        if (nativeJob && move.bHasWeaveParam && !move.bAppPointwiseWeave)
        {
            int shape = -1;
            const T_WeaveDate& weave = move.tWeaveParam;
            const bool unsupportedFields = weave.nWeaveType != 0
                || weave.nPauseTime3Ms != 0 || weave.nPauseTime4Ms != 0
                || weave.nPauseContinue != 0
                || !SameProgramValue(weave.dSwingDirectionDeg, 0.0)
                || !SameProgramValue(weave.dWeavePlaneAngleDeg, 0.0)
                || !SameProgramValue(weave.dSpaceAngleDeg, 0.0)
                || !SameProgramValue(weave.dEndLengthMm, 0.0)
                || !SameProgramValue(weave.dEndWidthMm, 0.0)
                || !SameProgramValue(weave.dCenterHeightMm, 0.0);
            if (!InovanceNativeWeaveShape(weave.nWeaveShape, shape)
                || unsupportedFields
                || !std::isfinite(weave.dWeaveFrequencyHz)
                || weave.dWeaveFrequencyHz <= 0.0
                || !std::isfinite(weave.dWeaveAmplitudeMm)
                || weave.dWeaveAmplitudeMm < 0.0
                || weave.nPauseTime1Ms < 0 || weave.nPauseTime2Ms < 0)
            {
                error = "汇川当前已接入现场验证的对称正弦原生摆动：Shape[1]、Freq、"
                    "RAmp=LAmp、RT、LT；第" + std::to_string(index + 1)
                    + "点包含尚无等价汇川JOB字段的摆动参数。";
                return false;
            }
        }
        if (actualWeld)
        {
            if (move.bArcStartBeforeMove)
            {
                if (arcActive || !move.bWeldProcessEnabled)
                {
                    error = "汇川第" + std::to_string(index + 1)
                        + "点的起弧标志重复，或该点未启用焊接工艺。";
                    return false;
                }
                arcActive = true;
                ++arcSegments;
            }
            if (move.bWeldProcessEnabled != arcActive)
            {
                error = "汇川第" + std::to_string(index + 1)
                    + "点的焊接启用状态与起收弧状态不一致。";
                return false;
            }
            if (move.bWeldProcessEnabled)
            {
                const double processValues[] = {
                    move.dArcStartCurrent, move.dArcStartVoltage,
                    move.dWeldCurrent, move.dWeldVoltage,
                    move.dArcEndCurrent, move.dArcEndVoltage
                };
                if (move.nMoveType != MOVL
                    || !std::all_of(std::begin(processValues), std::end(processValues),
                        [](double value) { return std::isfinite(value) && value >= 0.0; })
                    || !std::isfinite(move.dArcStartWaitTime)
                    || move.dArcStartWaitTime < 0.0 || move.dArcStartWaitTime > 65535.0
                    || !std::isfinite(move.dArcEndWaitTime)
                    || move.dArcEndWaitTime < 0.0 || move.dArcEndWaitTime > 65.535)
                {
                    error = "汇川第" + std::to_string(index + 1)
                        + "个焊接点必须是MOVL，电流/电压须为非负有限值，"
                        "起弧等待须为0..65535秒，ArcOffT须为0..65535毫秒。";
                    return false;
                }
            }
            if (move.bArcEndAfterMove)
            {
                if (!arcActive)
                {
                    error = "汇川第" + std::to_string(index + 1) + "点收弧时电弧未开启。";
                    return false;
                }
                arcActive = false;
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
            if (!nativeJob)
            {
                if (!m_connectionReady.load() || !m_kinematicsSession.Ready())
                {
                    error = "汇川轨迹包含关节点，但当前机器人未配置真实AxisUnit；JointMotion能力未开放。";
                    return false;
                }
            }
        }
        else
        {
            error = "汇川轨迹只支持MOVL和MOVJ。";
            return false;
        }
    }
    if (actualWeld && (arcSegments == 0 || arcActive))
    {
        error = arcSegments == 0
            ? "汇川实际焊接JOB没有任何明确的起弧段。"
            : "汇川实际焊接JOB结束时电弧仍处于开启状态，缺少收弧标志。";
        return false;
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
        needsCartesianArm = needsCartesianArm || move.nPosType == POSVAR;
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
    const bool actualWeld = purpose == RobotTrajectoryPurpose::ActualWeld;

    std::string controllerRobotName;
    if (!ReadControllerProgramRobotName(controllerRobotName, error))
    {
        error = "汇川原生JOB无法绑定控制器真实机型：" + error;
        return false;
    }

    std::ostringstream source;
    // The pendant parser expects local-point declarations to follow
    // ProgramInfo directly.  Module-scope comments before the first LP are
    // displayed as instructions and make the generated module unusable.
    source << InovanceProgramInfo(controllerRobotName);
    for (std::size_t index = 0; index < moveInfos.size(); ++index)
    {
        const T_ROBOT_MOVE_INFO& move = moveInfos[index];
        const double external[6] = {
            move.tCoord.dBX, move.tCoord.dBY, move.tCoord.dBZ,
            passiveExternal[3], passiveExternal[4], passiveExternal[5]
        };
        // A controller-created local-point variable is serialized in PRO as
        // three semicolon-delimited fields without braces or parentheses:
        //   LP[n] = X,Y,Z,A,B,C; arm0,arm1,arm2,arm3; E1,...,E6;
        // This deliberately mirrors the controller export instead of applying
        // C/C++ aggregate-initializer syntax to the built-in LP variable.
        source << "LP[" << index << "] =  " << FormatProgramDouble(move.tCoord.dX)
            << ", " << FormatProgramDouble(move.tCoord.dY)
            << ", " << FormatProgramDouble(move.tCoord.dZ)
            // 汇川PRO使用A,B,C；通用适配层使用RX,RY,RZ。
            << ", " << FormatProgramDouble(move.tCoord.dRZ)
            << ", " << FormatProgramDouble(move.tCoord.dRY)
            << ", " << FormatProgramDouble(move.tCoord.dRX) << "; "
            << arm[0] << ", " << arm[1] << ", " << arm[2] << ", " << arm[3] << "; "
            << FormatProgramDouble(external[0]) << ", " << FormatProgramDouble(external[1])
            << ", " << FormatProgramDouble(external[2]) << ", " << FormatProgramDouble(external[3])
            << ", " << FormatProgramDouble(external[4]) << ", " << FormatProgramDouble(external[5])
            << ";\r\n";
    }
    source << "Func " << kInovanceCallableFunction << "()\r\n";

    bool arcActive = false;
    bool nativeWeaveActive = false;
    const T_ROBOT_MOVE_INFO* activeWeldParameters = nullptr;
    const T_WeaveDate* activeWeaveParameters = nullptr;
    const auto weldSpeedMmPerSecond = [](const T_ROBOT_MOVE_INFO& move)
        {
            const double mmPerMinute = move.dWeldSpeedMmPerMin > 0.0
                ? move.dWeldSpeedMmPerMin : move.tSpeed.dSpeed;
            return mmPerMinute / 60.0;
        };
    const auto appendWeldSet = [&source, &weldSpeedMmPerSecond](
        const T_ROBOT_MOVE_INFO& move)
        {
            source << "WeldSet ArcData[" << kInovanceArcDataIndex << "],AC["
                << FormatProgramNumber(move.dWeldCurrent) << "],AV["
                << FormatProgramNumber(move.dWeldVoltage) << "],WS["
                << FormatProgramNumber(weldSpeedMmPerSecond(move)) << "];\r\n";
        };

    for (std::size_t index = 0; index < moveInfos.size(); ++index)
    {
        const T_ROBOT_MOVE_INFO& move = moveInfos[index];
        // Match the pendant-authored instruction form exactly.  In particular,
        // the field controller's editor accepts integer Speed[] and Fine, but the
        // current controller rejects the configured blend zone in generated
        // trajectory programs. Keep
        // every point exact until a controller-verified blending form is available.
        // The field-verified controller form binds every move explicitly to the
        // calibrated Tool[1] and Wobj[1]; do not rely on a previous instruction's
        // active coordinate state.
        const std::string zone = "Fine";

        if (actualWeld && move.bArcStartBeforeMove)
        {
            source << "WeldOn ArcData[" << kInovanceArcDataIndex << "],AC["
                << FormatProgramNumber(move.dArcStartCurrent) << "],AV["
                << FormatProgramNumber(move.dArcStartVoltage) << "],WS["
                << FormatProgramNumber(weldSpeedMmPerSecond(move)) << "],RPM["
                << kInovanceRpmIndex << "];\r\n";
            if (move.dArcStartWaitTime > 0.0)
            {
                source << "Wait T[" << FormatProgramNumber(move.dArcStartWaitTime)
                    << "];\r\n";
            }
            appendWeldSet(move);
            activeWeldParameters = &move;
            arcActive = true;
        }
        else if (actualWeld && arcActive && move.bWeldProcessEnabled
            && (activeWeldParameters == nullptr
                || !SameInovanceWeldParameters(*activeWeldParameters, move)))
        {
            appendWeldSet(move);
            activeWeldParameters = &move;
        }

        const bool wantsNativeWeave = move.bHasWeaveParam && !move.bAppPointwiseWeave;
        if (wantsNativeWeave && !nativeWeaveActive)
        {
            AppendInovanceWeaveCommand(source, "WeaveOn", move.tWeaveParam);
            activeWeaveParameters = &move.tWeaveParam;
            nativeWeaveActive = true;
        }
        else if (wantsNativeWeave && activeWeaveParameters != nullptr
            && !SameInovanceWeaveParameters(*activeWeaveParameters, move.tWeaveParam))
        {
            AppendInovanceWeaveCommand(source, "WeaveSet", move.tWeaveParam);
            activeWeaveParameters = &move.tWeaveParam;
        }
        else if (!wantsNativeWeave && nativeWeaveActive)
        {
            source << "WeaveOff;\r\n";
            nativeWeaveActive = false;
            activeWeaveParameters = nullptr;
        }

        if (move.nMoveType == MOVL)
        {
            const double speedMmPerMin = move.dWeldSpeedMmPerMin > 0.0
                ? move.dWeldSpeedMmPerMin : move.tSpeed.dSpeed;
            const int speedMmPerSecond = std::clamp(
                static_cast<int>(std::lround(speedMmPerMin / 60.0)), 1, 15000);
            source << "Movl LP[" << index << "],Speed["
                << speedMmPerSecond << "]," << zone << ",Tool[" << m_toolNo
                << "],Wobj[" << m_wobjNo << "];\r\n";
        }
        else
        {
            source << "Movj LP[" << index << "],V["
                << std::clamp(static_cast<int>(std::lround(move.tSpeed.dSpeed)), 1, 100)
                << "]," << zone << ",Tool[" << m_toolNo
                << "],Wobj[" << m_wobjNo << "];\r\n";
        }
        if (move.nDwellMs > 0)
        {
            source << "Wait T["
                << FormatProgramNumber(static_cast<double>(move.nDwellMs) / 1000.0)
                << "];\r\n";
        }

        if (actualWeld && move.bArcEndAfterMove)
        {
            const long long arcOffMilliseconds = std::llround(move.dArcEndWaitTime * 1000.0);
            source << "WeldOff ArcData[" << kInovanceArcDataIndex << "],AC["
                << FormatProgramNumber(move.dArcEndCurrent) << "],AV["
                << FormatProgramNumber(move.dArcEndVoltage) << "],ArcOffT["
                << arcOffMilliseconds << "];\r\n";
            arcActive = false;
            activeWeldParameters = nullptr;
            if (nativeWeaveActive)
            {
                source << "WeaveOff;\r\n";
                nativeWeaveActive = false;
                activeWeaveParameters = nullptr;
            }
        }
    }
    if (nativeWeaveActive) { source << "WeaveOff;\r\n"; }
    source << "EndFunc;\r\n";
    const std::string content = source.str();
    std::string generatedModuleError;
    if (!ValidateInovanceCallableModule(content, generatedModuleError))
    {
        error = "汇川原生JOB生成内容不满足公共模块契约：" + generatedModuleError;
        return false;
    }
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

bool InovanceRobotCtrl::ReadControllerProgramRobotName(
    std::string& robotName,
    std::string& error)
{
    robotName.clear();
    if (!IsConnected())
    {
        error = "2222控制通道未连接。";
        return false;
    }
    std::string response;
    if (!SendCommand("Get_RobotType", response))
    {
        error = GetLastRobotError();
        if (error.empty()) { error = "Get_RobotType调用失败。"; }
        return false;
    }
    robotName = Trim(ValuePart(response));
    if (robotName.empty() || robotName.size() > 128
        || robotName.rfind("IR-R", 0) != 0
        || robotName.find_first_of("\"\\\r\n") != std::string::npos)
    {
        error = "Get_RobotType返回的真实机型不能用于PRO的RobotName：" + robotName;
        robotName.clear();
        return false;
    }
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
    std::string localContent;
    std::string localError;
    if (!ReadBoundedTextFile(handle.localProgramPath, localContent, localError)
        || localContent.size() != handle.programContentSize
        || InovanceContentSha256(localContent) != handle.programContentSha256
        || !ValidateInovanceCallableModule(localContent, localError))
    {
        error = "汇川原生JOB上传前本地PRO身份或模块契约无效："
            + (localError.empty() ? std::string("SHA-256或大小不一致。") : localError);
        return false;
    }
    std::string moduleRobotName;
    if (!ParseInovanceProgramRobotName(localContent, moduleRobotName, localError))
    {
        error = "汇川原生JOB上传前无法确认PRO机型：" + localError;
        return false;
    }
    std::string controllerRobotName;
    if (!ReadControllerProgramRobotName(controllerRobotName, localError))
    {
        error = "汇川原生JOB上传前无法确认控制器真实机型：" + localError;
        return false;
    }
    if (moduleRobotName != controllerRobotName)
    {
        error = "汇川原生JOB的RobotName与当前控制器不一致：PRO="
            + moduleRobotName + "，Controller=" + controllerRobotName + "。";
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
    const std::string targetFile = handle.programName + ".pro";
    const std::size_t projectDirectoryEnd = activeDirectory.find(
        '/', std::strlen("/TeachProgram/"));
    const std::string activeProjectDirectory = projectDirectoryEnd == std::string::npos
        ? activeDirectory : activeDirectory.substr(0, projectDirectoryEnd);
    const std::string remoteProjectPath = activeProjectDirectory + "/"
        + activeProject + ".prj";
    const std::filesystem::path auditDirectory =
        std::filesystem::path(handle.localProgramPath).parent_path();
    const std::filesystem::path projectBackupPath = auditDirectory
        / (activeProject + "_before.prj");
    const std::filesystem::path registeredProjectPath = auditDirectory
        / (activeProject + "_registered.prj");
    const std::filesystem::path projectVerifyPath = auditDirectory
        / (activeProject + "_uploaded_verify.prj");
    if (!session->DownloadProgramFile(remoteProjectPath, projectBackupPath.string()))
    {
        error = "汇川原生JOB上传前无法备份当前PRJ：" + session->LastError();
        return false;
    }
    std::string projectContent;
    std::string projectReadError;
    if (!ReadBoundedTextFile(projectBackupPath, projectContent, projectReadError))
    {
        error = "汇川当前PRJ备份无效：" + projectReadError;
        return false;
    }
    std::string registeredProjectContent;
    bool projectChanged = false;
    std::string registrationError;
    if (!RegisterInovanceProgramInProject(
        projectContent, targetFile, registeredProjectContent,
        projectChanged, registrationError))
    {
        error = "汇川轨迹模块无法登记到当前PRJ：" + registrationError;
        return false;
    }
    if (!projectChanged)
    {
        registeredProjectContent = projectContent;
    }
    else
    {
        std::ofstream projectOutput(
            registeredProjectPath, std::ios::binary | std::ios::trunc);
        if (!projectOutput)
        {
            error = "汇川无法创建已登记轨迹模块的本地PRJ。";
            return false;
        }
        projectOutput.write(registeredProjectContent.data(),
            static_cast<std::streamsize>(registeredProjectContent.size()));
        projectOutput.flush();
        if (!projectOutput)
        {
            error = "汇川写入已登记轨迹模块的本地PRJ失败。";
            return false;
        }
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
    bool dynamicDirectoryFound = false;
    for (const RobotControllerFileInfo& entry : entries)
    {
        const std::string lower = LowerAscii(entry.name);
        if (entry.isDirectory)
        {
            dynamicDirectoryFound = dynamicDirectoryFound || lower == "dynamiccall";
            continue;
        }
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
    const std::string dynamicDirectory = activeDirectory + "/DynamicCall";
    const std::string dynamicRemotePath = dynamicDirectory + "/" + targetFile;
    const std::filesystem::path previousProgramPath = auditDirectory
        / (handle.programName + "_before.pro");
    const std::filesystem::path previousDynamicProgramPath = auditDirectory
        / (handle.programName + "_dynamic_before.pro");
    bool dynamicTargetFound = false;
    if (dynamicDirectoryFound)
    {
        std::vector<RobotControllerFileInfo> dynamicEntries;
        if (!session->ListProgramFiles(dynamicDirectory, dynamicEntries, 10000))
        {
            error = "汇川原生JOB上传前无法读取DynamicCall文件清单："
                + session->LastError();
            return false;
        }
        for (const RobotControllerFileInfo& entry : dynamicEntries)
        {
            dynamicTargetFound = dynamicTargetFound
                || (!entry.isDirectory && LowerAscii(entry.name) == LowerAscii(targetFile));
        }
    }
    if (targetFound
        && !session->DownloadProgramFile(remotePath, previousProgramPath.string()))
    {
        error = "汇川覆盖既有轨迹模块前无法建立PRO备份：" + session->LastError();
        return false;
    }
    if (dynamicTargetFound
        && !session->DownloadProgramFile(
            dynamicRemotePath, previousDynamicProgramPath.string()))
    {
        error = "汇川覆盖DynamicCall轨迹模块前无法建立PRO备份："
            + session->LastError();
        return false;
    }
    const auto rollbackTargetProgram = [&]() -> bool
        {
            return targetFound
                ? session->UploadProgramFile(
                    previousProgramPath.string(), remotePath, false)
                : session->DeleteProgramFile(remotePath);
        };
    const auto rollbackDynamicProgram = [&]() -> bool
        {
            return dynamicTargetFound
                ? session->UploadProgramFile(
                    previousDynamicProgramPath.string(), dynamicRemotePath, false)
                : session->DeleteProgramFile(dynamicRemotePath);
        };
    if (!session->UploadProgramFile(handle.localProgramPath, remotePath, false))
    {
        const std::string uploadError = session->LastError();
        const bool restored = rollbackTargetProgram();
        error = "汇川原生JOB上传失败：" + uploadError
            + (restored ? "；已恢复上传前PRO状态。" : "；PRO状态恢复失败。 ");
        return false;
    }
    const std::filesystem::path& frozenProjectPath = projectChanged
        ? registeredProjectPath : projectBackupPath;
    const std::filesystem::path verifyPath =
        std::filesystem::path(handle.localProgramPath).parent_path()
        / (handle.programName + "_uploaded_verify.pro");
    if (!session->DownloadProgramFile(remotePath, verifyPath.string()))
    {
        const std::string verifyError = session->LastError();
        const bool restored = rollbackTargetProgram();
        error = "汇川原生JOB上传后无法回读：" + verifyError
            + (restored ? "；已恢复上传前PRO状态。" : "；PRO状态恢复失败。 ");
        return false;
    }
    std::string uploadedContent;
    std::string readError;
    if (!ReadBoundedTextFile(verifyPath, uploadedContent, readError)
        || uploadedContent.size() != handle.programContentSize
        || InovanceContentSha256(uploadedContent) != handle.programContentSha256)
    {
        const bool restored = rollbackTargetProgram();
        error = "汇川原生JOB上传后字节身份不一致："
            + (readError.empty() ? std::string("SHA-256或大小不一致。") : readError)
            + (restored ? "；已恢复上传前PRO状态。" : "；PRO状态恢复失败。 ");
        return false;
    }
    if (!session->UploadProgramFile(
            handle.localProgramPath, dynamicRemotePath, false))
    {
        const std::string uploadError = session->LastError();
        const bool dynamicRestored = rollbackDynamicProgram();
        const bool programRestored = rollbackTargetProgram();
        error = "汇川原生JOB无法同步到Call执行目录DynamicCall：" + uploadError
            + (dynamicRestored ? "；已恢复DynamicCall上传前状态。"
                : "；DynamicCall状态恢复失败。")
            + (programRestored ? "；已恢复工程根目录PRO状态。"
                : "；工程根目录PRO状态恢复失败。 ");
        return false;
    }
    const std::filesystem::path dynamicVerifyPath = auditDirectory
        / (handle.programName + "_dynamic_uploaded_verify.pro");
    if (!session->DownloadProgramFile(
            dynamicRemotePath, dynamicVerifyPath.string()))
    {
        const std::string verifyError = session->LastError();
        const bool dynamicRestored = rollbackDynamicProgram();
        const bool programRestored = rollbackTargetProgram();
        error = "汇川原生JOB同步到DynamicCall后无法回读：" + verifyError
            + (dynamicRestored ? "；已恢复DynamicCall上传前状态。"
                : "；DynamicCall状态恢复失败。")
            + (programRestored ? "；已恢复工程根目录PRO状态。"
                : "；工程根目录PRO状态恢复失败。 ");
        return false;
    }
    std::string dynamicUploadedContent;
    if (!ReadBoundedTextFile(dynamicVerifyPath, dynamicUploadedContent, readError)
        || dynamicUploadedContent.size() != handle.programContentSize
        || InovanceContentSha256(dynamicUploadedContent) != handle.programContentSha256
        || dynamicUploadedContent != uploadedContent)
    {
        const bool dynamicRestored = rollbackDynamicProgram();
        const bool programRestored = rollbackTargetProgram();
        error = "汇川DynamicCall轨迹模块回读身份不一致："
            + (readError.empty() ? std::string("根目录与执行目录内容不一致。") : readError)
            + (dynamicRestored ? "；已恢复DynamicCall上传前状态。"
                : "；DynamicCall状态恢复失败。")
            + (programRestored ? "；已恢复工程根目录PRO状态。"
                : "；工程根目录PRO状态恢复失败。 ");
        return false;
    }
    if (projectChanged
        && !session->UploadProgramFile(
            frozenProjectPath.string(), remoteProjectPath, false))
    {
        const std::string uploadError = session->LastError();
        const bool restored = session->UploadProgramFile(
            projectBackupPath.string(), remoteProjectPath, false);
        const bool dynamicRestored = rollbackDynamicProgram();
        const bool programRestored = rollbackTargetProgram();
        error = "汇川轨迹模块PRO已上传并验证，但登记当前工程PRJ失败："
            + uploadError + (restored
                ? "；已回传原PRJ备份。"
                : "；原PRJ恢复失败，备份位于 " + projectBackupPath.string())
            + (dynamicRestored ? "；已恢复DynamicCall上传前状态。"
                : "；DynamicCall状态恢复失败。")
            + (programRestored ? "；已恢复上传前PRO状态。" : "；PRO状态恢复失败。 ");
        return false;
    }
    if (!session->DownloadProgramFile(remoteProjectPath, projectVerifyPath.string()))
    {
        const std::string verifyError = session->LastError();
        const bool restored = !projectChanged || session->UploadProgramFile(
            projectBackupPath.string(), remoteProjectPath, false);
        const bool dynamicRestored = rollbackDynamicProgram();
        const bool programRestored = rollbackTargetProgram();
        error = "汇川轨迹模块上传后无法回读PRJ：" + verifyError
            + (!projectChanged ? std::string()
                : restored ? "；已回传原PRJ备份。"
                : "；原PRJ恢复失败，备份位于 " + projectBackupPath.string())
            + (dynamicRestored ? "；已恢复DynamicCall上传前状态。"
                : "；DynamicCall状态恢复失败。")
            + (programRestored ? "；已恢复上传前PRO状态。" : "；PRO状态恢复失败。 ");
        return false;
    }
    std::string uploadedProjectContent;
    if (!ReadBoundedTextFile(projectVerifyPath, uploadedProjectContent, readError)
        || uploadedProjectContent != registeredProjectContent)
    {
        const bool restored = !projectChanged || session->UploadProgramFile(
            projectBackupPath.string(), remoteProjectPath, false);
        const bool dynamicRestored = rollbackDynamicProgram();
        const bool programRestored = rollbackTargetProgram();
        error = "汇川轨迹模块登记后的PRJ字节身份不一致："
            + (readError.empty() ? std::string("内容不一致。") : readError)
            + (!projectChanged ? std::string()
                : restored ? "；已回传原PRJ备份。"
                : "；原PRJ恢复失败，备份位于 " + projectBackupPath.string())
            + (dynamicRestored ? "；已恢复DynamicCall上传前状态。"
                : "；DynamicCall状态恢复失败。")
            + (programRestored ? "；已恢复上传前PRO状态。" : "；PRO状态恢复失败。 ");
        return false;
    }
    handle.remoteProgramPath = dynamicRemotePath;
    handle.localDataPath = frozenProjectPath.string();
    handle.remoteDataPath = remoteProjectPath;
    handle.dataContentSha256 = InovanceContentSha256(registeredProjectContent);
    handle.dataContentSize = static_cast<std::uint64_t>(registeredProjectContent.size());
    error.clear();
    return true;
}

bool InovanceRobotCtrl::VerifyTrajectoryJobRemoteIdentity(
    const RobotTrajectoryHandle& handle,
    std::string& error) const
{
    if (handle.remoteProgramPath.empty() || handle.localProgramPath.empty()
        || handle.programContentSha256.size() != 64 || handle.programContentSize == 0
        || handle.remoteDataPath.empty() || handle.localDataPath.empty()
        || handle.dataContentSha256.size() != 64 || handle.dataContentSize == 0)
    {
        error = "汇川原生JOB缺少PRO/PRJ远端路径或冻结内容身份。";
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
    const std::filesystem::path projectVerifyPath =
        std::filesystem::path(handle.localProgramPath).parent_path()
        / (handle.programName + "_project_start_verify.prj");
    if (!session->DownloadProgramFile(handle.remoteDataPath, projectVerifyPath.string()))
    {
        error = "汇川原生JOB启动前PRJ回读失败：" + session->LastError();
        return false;
    }
    std::string projectContent;
    if (!ReadBoundedTextFile(projectVerifyPath, projectContent, readError)
        || projectContent.size() != handle.dataContentSize
        || InovanceContentSha256(projectContent) != handle.dataContentSha256)
    {
        error = "汇川原生JOB启动前PRJ登记身份发生变化："
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
    // 原生WeldOn/WeldSet/WeldOff由控制器的ArcData负责焊机握手。
    // WeldJob IO/DA配置仅保留为可选的额外关弧见证，不再是实际焊接能力前提。
    if (!m_weldJobEnabled)
    {
        error.clear();
        return true;
    }
    if (m_weldArcEnableDo < 0 || m_weldCurrentDa < 0 || m_weldVoltageDa < 0)
    {
        error = "汇川已启用可选WeldJob硬件见证，但ArcEnableDO/CurrentDA/VoltageDA不完整。";
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
        error = "汇川可选WeldJob硬件见证要求ArcEnableDO、CurrentDA、VoltageDA均由RC控制；"
            "现场配置权回读未通过。";
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
            || handle.programContentSha256.size() != 64 || handle.programContentSize == 0
            || handle.localDataPath.empty() || handle.remoteDataPath.empty()
            || handle.dataContentSha256.size() != 64 || handle.dataContentSize == 0)
        {
            SetLastRobotError("汇川原生轨迹JOB尚未完成PRO生成、PRJ登记、上传和内容身份冻结，禁止启动。");
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

    if (!SetDataStreamMode("ON", 1) || !EnsureMotionReady()) { return false; }
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

        if (result.success && m_preparedPurpose == RobotTrajectoryPurpose::ActualWeld
            && m_weldJobEnabled && m_weldArcEnableDo >= 0)
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
    bool returnedToStart = false;
    if (motion == 2)
    {
        // Get_MotionSts=2 is a stopped-but-interrupted controller state, not a
        // natural terminal witness. Once the data stream and task are both
        // stopped, BackStartLine is the documented no-motion project reset.
        // Never clear the software interlock until motion=0 is read back three
        // consecutive times.
        std::string response;
        if (!SendCommand("BackStartLine", response) || response != "ok")
        {
            SetLastRobotError("汇川安全中止后的运动中断态清理失败：BackStartLine未被控制器确认。");
            return false;
        }
        returnedToStart = true;
        stableStopped = 0;
        for (int attempt = 0; attempt < 40; ++attempt)
        {
            if (!QueryInt("Get_TaskRunSts 0", taskStatus)
                || !QueryInt("Get_MotionSts", motion))
            {
                return false;
            }
            stableStopped = (taskStatus != 1 && motion == 0) ? stableStopped + 1 : 0;
            if (stableStopped >= 3) { break; }
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
        }
        if (stableStopped < 3)
        {
            SetLastRobotError("汇川BackStartLine后未连续确认Get_MotionSts=0，中断态仍未清除。");
            return false;
        }
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
    if (nativeProgramTracked && !returnedToStart)
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
    if (!SetDataStreamMode("ON", 1) || !EnsureMotionReady())
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
        && extension != ".pts" && extension != ".jsn" && extension != ".dat")
    {
        SetLastRobotError("汇川原生程序上传仅支持PRO、PRJ、PTS、JSN或DAT文件：" + localPath);
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
    if (!EnsureControlPermit()) { return false; }
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
    const std::filesystem::path projectCopyPath = runDirectory / "project_before_start.prj";
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
    // Call resolves its target from Task0/DynamicCall on this controller. The
    // root copy remains registered for pendant visibility, while this byte-identical
    // copy is the executable identity verified immediately before dispatch.
    const std::string remoteModulePath = activeDirectory
        + "/DynamicCall/" + actualModuleFile;
    const std::size_t projectDirectoryEnd = activeDirectory.find(
        '/', std::strlen("/TeachProgram/"));
    const std::string activeProjectDirectory = projectDirectoryEnd == std::string::npos
        ? activeDirectory : activeDirectory.substr(0, projectDirectoryEnd);
    const std::string remoteProjectPath = activeProjectDirectory + "/"
        + activeProject + ".prj";
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
    std::string moduleRobotName;
    if (!ParseInovanceProgramRobotName(moduleContent, moduleRobotName, moduleError))
    {
        return failWithoutMotion("汇川目标模块 " + actualModuleFile
            + " 无法确认PRO机型：" + moduleError);
    }
    std::string controllerRobotName;
    if (!ReadControllerProgramRobotName(controllerRobotName, moduleError))
    {
        return failWithoutMotion("汇川原生程序启动前无法确认控制器真实机型："
            + moduleError);
    }
    if (moduleRobotName != controllerRobotName)
    {
        return failWithoutMotion("汇川目标模块的RobotName与当前控制器不一致：PRO="
            + moduleRobotName + "，Controller=" + controllerRobotName + "。");
    }
    std::string mainRobotName;
    if (!ParseInovanceProgramRobotName(backupMainContent, mainRobotName, moduleError)
        || mainRobotName != controllerRobotName)
    {
        return failWithoutMotion("汇川当前main.pro与控制器真实机型不一致："
            + (moduleError.empty()
                ? "Main=" + mainRobotName + "，Controller=" + controllerRobotName + "。"
                : moduleError));
    }
    if (!session->DownloadProgramFile(remoteProjectPath, projectCopyPath.string()))
    {
        return failWithoutMotion("汇川无法下载并校验当前PRJ程序登记："
            + session->LastError());
    }
    std::string projectContent;
    std::string projectError;
    if (!ReadBoundedTextFile(projectCopyPath, projectContent, projectError))
    {
        return failWithoutMotion("汇川当前PRJ内容无效：" + projectError);
    }
    std::string registeredProjectContent;
    bool projectWouldChange = false;
    if (!RegisterInovanceProgramInProject(
            projectContent, actualModuleFile, registeredProjectContent,
            projectWouldChange, projectError)
        || projectWouldChange)
    {
        return failWithoutMotion(projectWouldChange
            ? "汇川Call目标未登记在当前PRJ的ProgramFiles中，禁止启动。"
            : "汇川当前PRJ程序清单无效：" + projectError);
    }
    std::string expectedTrajectorySha256;
    std::uint64_t expectedTrajectorySize = 0;
    std::string expectedProjectSha256;
    std::uint64_t expectedProjectSize = 0;
    std::string expectedProjectRemotePath;
    {
        std::lock_guard<std::mutex> lock(m_trajectoryMutex);
        if (m_activeHandle.started
            && LowerAscii(m_activeHandle.programName) == LowerAscii(requestedModule))
        {
            expectedTrajectorySha256 = m_activeHandle.programContentSha256;
            expectedTrajectorySize = m_activeHandle.programContentSize;
            expectedProjectSha256 = m_activeHandle.dataContentSha256;
            expectedProjectSize = m_activeHandle.dataContentSize;
            expectedProjectRemotePath = m_activeHandle.remoteDataPath;
        }
    }
    if (!expectedTrajectorySha256.empty()
        && (moduleContent.size() != expectedTrajectorySize
            || InovanceContentSha256(moduleContent) != expectedTrajectorySha256))
    {
        return failWithoutMotion("汇川原生轨迹模块在StartTrajectory冻结后发生变化，"
            "远端PRO的SHA-256或大小与句柄不一致。");
    }
    if (!expectedProjectSha256.empty())
    {
        if (LowerAscii(expectedProjectRemotePath) != LowerAscii(remoteProjectPath))
        {
            return failWithoutMotion("汇川原生轨迹句柄绑定的PRJ路径与当前激活工程不一致。");
        }
        if (projectContent.size() != expectedProjectSize
            || InovanceContentSha256(projectContent) != expectedProjectSha256)
        {
            return failWithoutMotion("汇川原生轨迹模块在StartTrajectory冻结后发生变化，"
                "远端PRJ的SHA-256或大小与句柄不一致："
                "内容不一致。");
        }
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
        dispatcherPath, actualModuleName, controllerRobotName,
        dispatcherContent, dispatcherError))
    {
        return failWithoutMotion("汇川生成 main.pro 调度器失败：" + dispatcherError);
    }

    bool dispatcherInstalled = false;
    bool keepPermanentDispatcher = false;
    bool mainRestoreVerified = true;
    const auto restoreOriginalMain = [&]()
        {
            if (!dispatcherInstalled) { return std::string(); }
            if (keepPermanentDispatcher)
            {
                return std::string("；main.pro永久Call调度器已保留。 ");
            }
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
    // Once the dispatcher has passed byte-for-byte readback it becomes the
    // permanent Task0 entry.  The backup is retained for audit/recovery only;
    // normal completion, stop and fault paths must not replace it with the old
    // main.pro.
    keepPermanentDispatcher = true;

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
    // Native JOB execution has its own automatic-mode preparation, independent
    // of the selected data-stream recipe. Mode transitions are asynchronous on
    // the field controller, so wait for mode 2 to settle and then (re-)enable
    // servo before START instead of trusting the first Get_Mode read.
    std::string automaticPreparationTrace;
    if (!EnsureControlPermit()
        || !InovanceModeSequence::EnsureRuntimeReady(
            ModeSequenceOps(m_modeConnectionEpoch.load()), 2, automaticPreparationTrace))
    {
        const std::string error = GetLastRobotError();
        return failWithoutMotion("汇川原生JOB自动模式准备失败："
            + (error.empty() ? automaticPreparationTrace : error + "\n" + automaticPreparationTrace)
            + restoreOriginalMain());
    }
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
    const auto controllerFaultDetail = [this](const char* phase, int faultStatus)
        {
            std::string faultCode;
            const bool codeRead = SendCommand("Get_SysErr", faultCode);
            return std::string("汇川控制器在原生程序") + phase
                + "期间报告故障，Get_SysErrSts=" + std::to_string(faultStatus)
                + "，Get_SysErr=" + (codeRead && !faultCode.empty()
                    ? faultCode : std::string("读取失败：") + GetLastRobotError())
                + "。";
        };
    const auto completeRun = [this, terminalStatus, &motionMarked,
        &activeProject, &actualModuleName](const ProgramSnapshot& snapshot)
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
                + "；B255=10且任务/运动连续稳定停止；main.pro永久Call调度器已保留。";
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
            return failRun(controllerFaultDetail("启动", snapshot.fault), snapshot.task);
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
            return failRun(controllerFaultDetail("运行", snapshot.fault), snapshot.task);
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

    if (!EnsureControlPermit()) { return false; }
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
    if (index < 0 || index > 255)
    {
        SetLastRobotError("汇川全局D实数变量索引范围为0..255。");
        return false;
    }
    const std::string type = LowerAscii(Trim(prefix == nullptr ? "REAL" : prefix));
    if (type != "real" && type != "d")
    {
        SetLastRobotError("汇川实数变量只支持REAL或D前缀，并映射到全局D[0..255]。");
        return false;
    }
    if (scope != 1)
    {
        SetLastRobotError("汇川远程以太网Set_D只提供全局D变量；适配层scope必须为1。");
        return false;
    }
    if (!std::isfinite(value) || value < -9999999.999 || value > 9999999.999)
    {
        SetLastRobotError("汇川全局D实数变量值范围为-9999999.999..9999999.999。");
        return false;
    }
    if (!EnsureControlPermit()) { return false; }
    std::string response;
    if (!SendCommand("Set_D " + std::to_string(index) + " "
        + FormatProgramNumber(value), response) || response != "ok")
    {
        return false;
    }
    double verified = 0.0;
    if (!TryGetRealVar(index, verified, "D", 1)
        || std::abs(verified - value) > 1.1e-6)
    {
        SetLastRobotError("汇川D变量写入后回读不一致：Index="
            + std::to_string(index) + " Expected=" + FormatProgramNumber(value)
            + " Actual=" + FormatProgramNumber(verified) + "。");
        return false;
    }
    ClearLastRobotError();
    return true;
}

bool InovanceRobotCtrl::TryGetRealVar(
    int index, double& value, const char* prefix, int scope)
{
    value = 0.0;
    if (index < 0 || index > 255)
    {
        SetLastRobotError("汇川全局D实数变量索引范围为0..255。");
        return false;
    }
    const std::string type = LowerAscii(Trim(prefix == nullptr ? "REAL" : prefix));
    if (type != "real" && type != "d")
    {
        SetLastRobotError("汇川实数变量读取只支持REAL或D前缀。");
        return false;
    }
    if (scope != 1)
    {
        SetLastRobotError("汇川远程以太网Get_D只提供全局D变量；适配层scope必须为1。");
        return false;
    }
    std::vector<double> values;
    if (!QueryDoubles("Get_D " + std::to_string(index), values, 1)
        || values.size() != 1)
    {
        if (GetLastRobotError().empty())
        {
            SetLastRobotError("汇川Get_D必须返回且只返回一个实数。");
        }
        return false;
    }
    value = values.front();
    ClearLastRobotError();
    return true;
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
    if (rotation == nullptr || translation == nullptr)
    {
        const std::string message = "手眼矩阵输出缓冲区为空。";
        SetLastRobotError(message);
        if (error != nullptr) { *error = message; }
        return false;
    }
    const std::string name = variableName == nullptr ? std::string() : std::string(variableName);
    if (name != "eye" && name != "laser0" && name != "sensor0")
    {
        const std::string message = "汇川手眼名称仅支持 eye/laser0/sensor0，固定映射到弧焊激光传感器0。";
        SetLastRobotError(message);
        if (error != nullptr) { *error = message; }
        return false;
    }
    RobotControllerHandEye value;
    std::string message;
    if (!ReadControllerHandEye(0, value, message))
    {
        SetLastRobotError(message);
        if (error != nullptr) { *error = message; }
        return false;
    }
    for (int row = 0; row < 3; ++row)
    {
        for (int col = 0; col < 3; ++col)
        {
            rotation[row * 3 + col] = value.cameraToTool(row, col);
        }
        translation[row] = value.cameraToTool(row, 3);
    }
    ClearLastRobotError();
    if (error != nullptr)
    {
        *error = "来源=" + value.source + "；相机=" + value.cameraAddress
            + "；绑定Tool=" + std::to_string(value.toolIndex) + "；参考系=camera-to-tool-tcp";
    }
    return true;
}
