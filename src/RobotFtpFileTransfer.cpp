#include "RobotFtpFileTransfer.h"

#include "FTPClient.h"
#include "RobotLog.h"

#include <algorithm>
#include <cctype>
#include <deque>
#include <filesystem>
#include <set>
#include <utility>

namespace
{
std::string Lower(std::string text)
{
    std::transform(text.begin(), text.end(), text.begin(), [](unsigned char ch)
        { return static_cast<char>(std::tolower(ch)); });
    return text;
}

std::string NormalizeExtension(std::string extension)
{
    extension = Lower(std::move(extension));
    if (!extension.empty() && extension.front() != '.')
    {
        extension.insert(extension.begin(), '.');
    }
    return extension;
}

std::string FileExtension(const std::string& path)
{
    const std::string name = std::filesystem::path(path).filename().string();
    const std::size_t dot = name.find_last_of('.');
    return dot == std::string::npos ? std::string() : NormalizeExtension(name.substr(dot));
}

bool ContainsExtension(const std::vector<std::string>& extensions, const std::string& path)
{
    const std::string extension = FileExtension(path);
    return !extension.empty() && std::any_of(
        extensions.cbegin(), extensions.cend(),
        [&extension](const std::string& candidate)
        {
            return NormalizeExtension(candidate) == extension;
        });
}

std::size_t CountProgramUnits(
    const std::vector<FtpRemoteFileInfo>& entries,
    const std::vector<std::string>& extensions)
{
    std::set<std::string> acceptedExtensions;
    for (const std::string& extension : extensions)
    {
        const std::string normalized = NormalizeExtension(extension);
        if (normalized.size() > 1)
        {
            acceptedExtensions.insert(normalized);
        }
    }

    std::set<std::string> programNames;
    for (const FtpRemoteFileInfo& entry : entries)
    {
        if (entry.isDirectory || entry.name.empty())
        {
            continue;
        }
        const std::string normalizedName = Lower(
            entry.path.empty() ? entry.name : entry.path);
        const std::size_t dot = normalizedName.find_last_of('.');
        if (dot == std::string::npos || dot == 0 || dot + 1 >= normalizedName.size())
        {
            continue;
        }
        if (acceptedExtensions.find(normalizedName.substr(dot)) != acceptedExtensions.end())
        {
            programNames.insert(normalizedName.substr(0, dot));
        }
    }
    return programNames.size();
}
}

RobotFtpFileTransfer::RobotFtpFileTransfer(
    std::string host,
    int port,
    std::string user,
    std::string password,
    RobotFileTransferProfile profile,
    std::vector<std::string> listedProgramExtensions,
    std::vector<std::string> inventoryProgramExtensions,
    std::string logPath)
    : m_host(std::move(host))
    , m_port(port)
    , m_user(std::move(user))
    , m_password(std::move(password))
    , m_profile(std::move(profile))
    , m_listedProgramExtensions(std::move(listedProgramExtensions))
    , m_inventoryProgramExtensions(std::move(inventoryProgramExtensions))
    , m_logPath(std::move(logPath))
{
}

RobotFtpFileTransfer::~RobotFtpFileTransfer() = default;

const RobotFileTransferProfile& RobotFtpFileTransfer::Profile() const
{
    return m_profile;
}

bool RobotFtpFileTransfer::EnsureClient()
{
    if (m_client != nullptr)
    {
        return true;
    }
    if (m_host.empty() || m_port <= 0 || m_port > 65535)
    {
        SetError("机器人FTP连接参数不完整。");
        return false;
    }
    if (m_log == nullptr)
    {
        m_log = std::make_unique<RobotLog>(m_logPath.empty() ? "Log/RobotFtpFileTransfer.log" : m_logPath, false);
    }
    m_client = std::make_unique<FtpClient>(m_log.get(), m_host, m_port, m_user, m_password);
    m_client->setMessageBoxesEnabled(false);
    return true;
}

bool RobotFtpFileTransfer::IsListedProgramFile(const std::string& fileName) const
{
    return ContainsExtension(m_listedProgramExtensions, fileName);
}

bool RobotFtpFileTransfer::ValidateProgramPath(const std::string& path, const char* operation)
{
    if (path.empty() || !IsListedProgramFile(path))
    {
        SetError(std::string(operation) + "失败：文件类型不属于当前机器人品牌声明的程序格式。");
        return false;
    }
    return true;
}

bool RobotFtpFileTransfer::ListProgramFiles(
    const std::string& remoteDirectory,
    std::vector<RobotControllerFileInfo>& entries,
    int timeoutMs)
{
    entries.clear();
    if (!EnsureClient())
    {
        return false;
    }
    const std::string directory = remoteDirectory.empty()
        ? m_profile.defaultRemoteDirectory : remoteDirectory;
    std::vector<FtpRemoteFileInfo> rawEntries;
    if (!m_client->listFiles(directory, rawEntries, nullptr, timeoutMs))
    {
        SetError("机器人FTP目录读取失败：" + directory);
        return false;
    }
    entries.reserve(rawEntries.size());
    for (const FtpRemoteFileInfo& entry : rawEntries)
    {
        if (!entry.isDirectory && !IsListedProgramFile(entry.name))
        {
            continue;
        }
        entries.push_back(RobotControllerFileInfo{
            entry.name,
            entry.path,
            entry.modifiedTime,
            static_cast<std::uint64_t>(entry.size),
            entry.isDirectory });
    }
    SetError({});
    return true;
}

bool RobotFtpFileTransfer::UploadProgramFile(
    const std::string& localPath,
    const std::string& remotePath,
    bool replaceExisting)
{
    if (!ValidateProgramPath(localPath, "上传")
        || !ValidateProgramPath(remotePath, "上传")
        || !EnsureClient())
    {
        return false;
    }
    if (!m_client->uploadFile(localPath, remotePath, replaceExisting))
    {
        SetError("机器人程序上传失败：" + remotePath);
        return false;
    }
    SetError({});
    return true;
}

bool RobotFtpFileTransfer::DownloadProgramFile(
    const std::string& remotePath,
    const std::string& localPath)
{
    if (!ValidateProgramPath(remotePath, "下载") || !EnsureClient())
    {
        return false;
    }
    if (!m_client->downloadFile(remotePath, localPath))
    {
        SetError("机器人程序下载失败：" + remotePath);
        return false;
    }
    SetError({});
    return true;
}

bool RobotFtpFileTransfer::DeleteProgramFile(const std::string& remotePath)
{
    if (!ValidateProgramPath(remotePath, "删除") || !EnsureClient())
    {
        return false;
    }
    if (!m_client->deleteFile(remotePath, false))
    {
        SetError("机器人程序删除失败：" + remotePath);
        return false;
    }
    SetError({});
    return true;
}

bool RobotFtpFileTransfer::QueryProgramInventory(
    RobotProgramInventoryResult& result,
    int timeoutMs)
{
    result = {};
    result.robotName = m_profile.robotName;
    result.remoteDirectory = m_profile.defaultRemoteDirectory;
    if (!EnsureClient())
    {
        return false;
    }
    // 品牌默认目录通常只是工程根目录（例如汇川 /TeachProgram/cs/Task0）。
    // 库存必须有限递归读取，否则根目录只有文件夹时会误报“程序=0”。
    constexpr int kMaximumDepth = 4;
    constexpr std::size_t kMaximumDirectories = 128;
    constexpr std::size_t kMaximumTotalEntries = 2000;
    const std::size_t maximumPerDirectory = static_cast<std::size_t>(
        std::clamp(timeoutMs, 1, 10000));
    std::deque<std::pair<std::string, int>> pending;
    std::set<std::string> visitedDirectories;
    std::vector<FtpRemoteFileInfo> allEntries;
    pending.emplace_back(result.remoteDirectory, 0);
    while (!pending.empty())
    {
        const auto [directory, depth] = pending.front();
        pending.pop_front();
        if (!visitedDirectories.insert(Lower(directory)).second)
        {
            continue;
        }
        if (visitedDirectories.size() > kMaximumDirectories)
        {
            SetError("机器人FTP程序库存目录超过128个，已停止有限递归读取。");
            return false;
        }
        std::vector<FtpRemoteFileInfo> entries;
        if (!m_client->listFiles(directory, entries, nullptr, maximumPerDirectory))
        {
            SetError("机器人FTP程序库存读取失败：" + directory);
            return false;
        }
        if (allEntries.size() + entries.size() > kMaximumTotalEntries)
        {
            SetError("机器人FTP程序库存条目超过2000个，已停止有限递归读取。");
            return false;
        }
        allEntries.insert(allEntries.end(), entries.cbegin(), entries.cend());
        if (depth >= kMaximumDepth)
        {
            continue;
        }
        for (const FtpRemoteFileInfo& entry : entries)
        {
            if (entry.isDirectory && !entry.path.empty())
            {
                pending.emplace_back(entry.path, depth + 1);
            }
        }
    }
    result.entryCount = allEntries.size();
    result.programCount = CountProgramUnits(allEntries, m_inventoryProgramExtensions);
    SetError({});
    return true;
}

std::string RobotFtpFileTransfer::LastError() const
{
    return m_lastError;
}

void RobotFtpFileTransfer::SetError(std::string error)
{
    m_lastError = std::move(error);
    if (!m_lastError.empty() && m_log != nullptr)
    {
        m_log->write(LogColor::ERR, "%s", m_lastError.c_str());
    }
}
