#include "FtpClient.h"
#include <stdexcept>
#include <fstream>
#include <sstream>
#include <chrono>
#include <algorithm>
#include <cstdio>
#include <cstring>

namespace
{
    long long ElapsedMs(std::chrono::steady_clock::time_point start)
    {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start).count();
    }

    std::string NormalizeRemoteDir(std::string remoteDir)
    {
        std::replace(remoteDir.begin(), remoteDir.end(), '\\', '/');
        while (remoteDir.size() > 1 && remoteDir.back() == '/')
        {
            remoteDir.pop_back();
        }
        if (remoteDir == "/")
        {
            return "";
        }
        return remoteDir;
    }

    std::string BuildRemoteListPattern(const std::string& remoteDir)
    {
        const std::string normalizedDir = NormalizeRemoteDir(remoteDir);
        return normalizedDir.empty() ? "*" : normalizedDir + "/*";
    }

    std::string JoinRemotePath(const std::string& remoteDir, const std::string& name)
    {
        const std::string normalizedDir = NormalizeRemoteDir(remoteDir);
        return normalizedDir.empty() ? name : normalizedDir + "/" + name;
    }

    unsigned long long FileSizeFromFindData(const WIN32_FIND_DATAA& data)
    {
        return (static_cast<unsigned long long>(data.nFileSizeHigh) << 32)
            | static_cast<unsigned long long>(data.nFileSizeLow);
    }

    std::string FormatFileTimeText(const FILETIME& fileTime)
    {
        FILETIME localFileTime{};
        SYSTEMTIME systemTime{};
        if (!FileTimeToLocalFileTime(&fileTime, &localFileTime)
            || !FileTimeToSystemTime(&localFileTime, &systemTime))
        {
            return "";
        }

        char buffer[32] = {};
        std::snprintf(buffer, sizeof(buffer), "%04u-%02u-%02u %02u:%02u:%02u",
            static_cast<unsigned>(systemTime.wYear),
            static_cast<unsigned>(systemTime.wMonth),
            static_cast<unsigned>(systemTime.wDay),
            static_cast<unsigned>(systemTime.wHour),
            static_cast<unsigned>(systemTime.wMinute),
            static_cast<unsigned>(systemTime.wSecond));
        return buffer;
    }
}

// 构造函数实现
FtpClient::FtpClient(
    RobotLog* log,
    const std::string& ftpHost,
    int ftpPort,
    const std::string& ftpUser,
    const std::string& ftpPwd)
    : m_log(log),  // 初始化日志（控制台+文件输出）
    m_ftpHost(ftpHost),
    m_ftpPort(ftpPort),
    m_ftpUser(ftpUser),
    m_ftpPwd(ftpPwd),
    m_hInternet(nullptr),
    m_hFtpSession(nullptr) {
    // 日志记录FTP客户端初始化
    m_log->write(LogColor::SUCCESS, "FTP客户端初始化完成 | 服务器：%s:%d | 用户名：%s",
        ftpHost.c_str(), ftpPort, ftpUser.c_str());
}

// 析构函数：释放所有FTP句柄
FtpClient::~FtpClient() {
    closeFtpSession();
    m_log->write(LogColor::DEFAULT, "FTP客户端已释放所有资源");
}

void FtpClient::setMessageBoxesEnabled(bool enabled) {
    m_messageBoxesEnabled = enabled;
}

std::wstring FtpClient::s2w(const std::string& str) {
    return std::wstring(str.begin(), str.end());
}
// 获取父目录
std::string FtpClient::getParentDir(const std::string& filePath) {
    size_t lastSlash = filePath.find_last_of("/\\");
    if (lastSlash == std::string::npos) return "";
    return filePath.substr(0, lastSlash);
}

// 递归创建本地目录（下载用）
bool FtpClient::createLocalDirRecursive(const std::string& localDir) {
    if (localDir.empty()) return true;
    if (PathIsDirectoryA(localDir.c_str())) return true;

    std::string parent = getParentDir(localDir);
    if (!parent.empty()) createLocalDirRecursive(parent);

    _mkdir(localDir.c_str());
    m_log->write(LogColor::DEFAULT, "本地目录已创建: %s", localDir.c_str());
    return true;
}

// 递归创建远程目录（上传用）
bool FtpClient::createRemoteDirRecursive(const std::string& remoteDir) {
    if (!m_hFtpSession || remoteDir.empty()) return false;

    wchar_t oldDir[MAX_PATH] = { 0 };
    DWORD bufSize = MAX_PATH; // 改为 DWORD
    const BOOL hasOldDir = FtpGetCurrentDirectory(m_hFtpSession, oldDir, &bufSize); // 传地址
    BOOL exist = FtpSetCurrentDirectory(m_hFtpSession, s2w(remoteDir).c_str());

    if (exist) {
        if (hasOldDir) {
            FtpSetCurrentDirectory(m_hFtpSession, oldDir);
        }
        return true;
    }

    std::string parent = getParentDir(remoteDir);
    if (!parent.empty() && !createRemoteDirRecursive(parent)) {
        if (hasOldDir) {
            FtpSetCurrentDirectory(m_hFtpSession, oldDir);
        }
        return false;
    }

    if (FtpCreateDirectory(m_hFtpSession, s2w(remoteDir).c_str())) {
        m_log->write(LogColor::DEFAULT, "远程目录已创建: %s", remoteDir.c_str());
    }
    else if (!FtpSetCurrentDirectory(m_hFtpSession, s2w(remoteDir).c_str())) {
        m_log->write(LogColor::ERR, "远程目录确认失败: %s | %s", remoteDir.c_str(), getFtpErrorMsg().c_str());
        if (hasOldDir) {
            FtpSetCurrentDirectory(m_hFtpSession, oldDir);
        }
        return false;
    }

    if (hasOldDir) {
        FtpSetCurrentDirectory(m_hFtpSession, oldDir);
    }
    return true;
}

// 内部函数：释放FTP会话句柄
void FtpClient::closeFtpSession() {
    if (m_hFtpSession != nullptr) {
        InternetCloseHandle(m_hFtpSession);
        m_hFtpSession = nullptr;
    }
    if (m_hInternet != nullptr) {
        InternetCloseHandle(m_hInternet);
        m_hInternet = nullptr;
    }
}

// 内部函数：获取WinINet错误信息
std::string FtpClient::getFtpErrorMsg() {
    DWORD errCode = GetLastError();
    char errMsg[256] = { 0 };
    FormatMessageA(FORMAT_MESSAGE_FROM_SYSTEM, nullptr, errCode, 0, errMsg, sizeof(errMsg), nullptr);
    return std::string(errMsg);
}

// 内部函数：真实FTP服务器连接逻辑
bool FtpClient::connectFtpServer() {
    try {
        const auto start = std::chrono::steady_clock::now();
        if (m_hFtpSession != nullptr) {
            wchar_t currentDir[MAX_PATH] = { 0 };
            DWORD currentDirSize = MAX_PATH;
            if (FtpGetCurrentDirectory(m_hFtpSession, currentDir, &currentDirSize)) {
                m_log->write(LogColor::DEFAULT, "FTP复用已有连接 | 耗时=%lldms", ElapsedMs(start));
                return true;
            }

            const std::string errMsg = getFtpErrorMsg();
            m_log->write(LogColor::WARNING, "FTP旧连接失效，准备重连 | %s | 耗时=%lldms",
                errMsg.c_str(), ElapsedMs(start));
            closeFtpSession();
        }

        // 1. 初始化WinINet根句柄
        m_hInternet = InternetOpenA("FTP_Client", INTERNET_OPEN_TYPE_DIRECT, nullptr, nullptr, 0);
        if (m_hInternet == nullptr) {
            std::string errMsg = "WinINet初始化失败：" + getFtpErrorMsg();
            m_log->write(LogColor::ERR, errMsg.c_str());
            if (m_messageBoxesEnabled) {
                showErrorMessage("FTP错误", "%s", errMsg.c_str());
            }
            return false;
        }

        // 2. 拼接FTP服务器地址（格式：ftp://xxx:端口）
        std::string ftpUrl = "ftp://" + m_ftpHost + ":" + std::to_string(m_ftpPort);

        // 3. 建立FTP会话
        m_hFtpSession = InternetConnectA(m_hInternet,
            m_ftpHost.c_str(),
            m_ftpPort,
            m_ftpUser.c_str(),
            m_ftpPwd.c_str(),
            INTERNET_SERVICE_FTP,
            INTERNET_FLAG_PASSIVE, // 被动模式（避免端口映射问题）
            0);
        if (m_hFtpSession == nullptr) {
            std::string errMsg = "FTP服务器连接失败：" + m_ftpHost + ":" + std::to_string(m_ftpPort) + " | " + getFtpErrorMsg();
            m_log->write(LogColor::ERR, errMsg.c_str());
            if (m_messageBoxesEnabled) {
                showErrorMessage("FTP错误", "%s", errMsg.c_str());
            }
            closeFtpSession(); // 释放已创建的句柄
            return false;
        }

        m_log->write(LogColor::SUCCESS, "FTP服务器连接成功：%s:%d | 耗时=%lldms",
            m_ftpHost.c_str(), m_ftpPort, ElapsedMs(start));
        return true;
    }
    catch (const std::exception& e) {
        std::string errMsg = "FTP连接异常：" + std::string(e.what());
        m_log->write(LogColor::ERR, errMsg.c_str());
        if (m_messageBoxesEnabled) {
            showErrorMessage("FTP异常", "%s", errMsg.c_str());
        }
        closeFtpSession();
        return false;
    }
}

// 真实文件上传逻辑
bool FtpClient::uploadFile(const std::string& localFilePath, const std::string& remoteFilePath, bool deleteBeforeUpload) {
    const auto totalStart = std::chrono::steady_clock::now();
    // 1. 日志记录上传开始
    m_log->write(LogColor::DEFAULT, "开始上传文件 | 本地：%s | 远程：%s",
        localFilePath.c_str(), remoteFilePath.c_str());

    // 2. 检查并建立FTP连接
    const auto connectStart = std::chrono::steady_clock::now();
    if (!connectFtpServer()) {
        return false;
    }
    m_log->write(LogColor::DEFAULT, "FTP上传阶段耗时 | Connect=%lldms | 远程=%s",
        ElapsedMs(connectStart), remoteFilePath.c_str());

    try {
        // 核心：自动创建远程父目录（如 /UserPrograms/new_dir/ 不存在则创建）
        const auto dirStart = std::chrono::steady_clock::now();
        std::string remoteParentDir = getParentDir(remoteFilePath);
        if (!remoteParentDir.empty()) {
            if (!createRemoteDirRecursive(remoteParentDir)) {
                std::string errMsg = "远程目录确认失败 | 目录：" + remoteParentDir + " | 文件：" + remoteFilePath;
                m_log->write(LogColor::ERR, errMsg.c_str());
                if (m_messageBoxesEnabled) {
                    showErrorMessage("FTP错误", "%s", errMsg.c_str());
                }
                closeFtpSession();
                return false;
            }
        }
        m_log->write(LogColor::DEFAULT, "FTP上传阶段耗时 | EnsureDir=%lldms | Dir=%s",
            ElapsedMs(dirStart), remoteParentDir.c_str());

        if (deleteBeforeUpload) {
            // FANUC 控制器有时不会稳定覆盖同名文件，上传前先静默删除旧文件。
            const auto deleteStart = std::chrono::steady_clock::now();
            if (FtpDeleteFileA(m_hFtpSession, remoteFilePath.c_str())) {
                m_log->write(LogColor::DEFAULT, "上传前已删除远程旧文件：%s | 耗时=%lldms",
                    remoteFilePath.c_str(), ElapsedMs(deleteStart));
            }
            else {
                m_log->write(LogColor::DEFAULT, "上传前未删除到远程旧文件，将继续上传：%s | 耗时=%lldms | %s",
                    remoteFilePath.c_str(), ElapsedMs(deleteStart), getFtpErrorMsg().c_str());
            }
        }
        else {
            m_log->write(LogColor::DEFAULT, "跳过上传前删除，直接覆盖远程文件：%s", remoteFilePath.c_str());
        }

        // 3. 执行FTP上传（WinINet API）
        const auto putStart = std::chrono::steady_clock::now();
        BOOL uploadOk = FtpPutFileA(m_hFtpSession,
            localFilePath.c_str(),
            remoteFilePath.c_str(),
            FTP_TRANSFER_TYPE_BINARY, // 二进制模式（避免文件损坏）
            0);
        const long long putMs = ElapsedMs(putStart);

        if (uploadOk) {
            // 上传成功：只写日志，不弹窗；FTP会话保留给后续文件复用。
            std::string successMsg = "文件上传成功 | 本地：" + localFilePath + " | 远程：" + remoteFilePath;
            m_log->write(LogColor::SUCCESS, successMsg.c_str());
            m_log->write(LogColor::SUCCESS, "FTP上传完成 | PutFile=%lldms | Total=%lldms | 远程=%s",
                putMs, ElapsedMs(totalStart), remoteFilePath.c_str());
            return true;
        }
        else {
            // 上传失败：日志+错误弹窗
            std::string errMsg = "文件上传失败 | 本地：" + localFilePath + " | 远程：" + remoteFilePath + " | " + getFtpErrorMsg();
            m_log->write(LogColor::ERR, errMsg.c_str());
            m_log->write(LogColor::ERR, "FTP上传失败耗时 | PutFile=%lldms | Total=%lldms | 远程=%s",
                putMs, ElapsedMs(totalStart), remoteFilePath.c_str());
            if (m_messageBoxesEnabled) {
                showErrorMessage("FTP错误", "%s", errMsg.c_str());
            }
            closeFtpSession();
            return false;
        }
    }
    catch (const std::exception& e) {
        // 异常处理：日志+错误弹窗
        std::string errMsg = "文件上传异常：" + std::string(e.what()) + " | 本地：" + localFilePath;
        m_log->write(LogColor::ERR, errMsg.c_str());
        if (m_messageBoxesEnabled) {
            showErrorMessage("FTP异常", "%s", errMsg.c_str());
        }
        closeFtpSession();
        return false;
    }
}

bool FtpClient::listFiles(const std::string& remoteDir, std::vector<FtpRemoteFileInfo>& files)
{
    files.clear();
    const auto totalStart = std::chrono::steady_clock::now();
    if (m_log != nullptr)
    {
        m_log->write(LogColor::DEFAULT, "开始读取FTP目录 | 远程目录：%s", remoteDir.c_str());
    }

    if (!connectFtpServer())
    {
        return false;
    }

    WIN32_FIND_DATAA findData{};
    const std::string searchPattern = BuildRemoteListPattern(remoteDir);
    HINTERNET findHandle = FtpFindFirstFileA(
        m_hFtpSession,
        searchPattern.c_str(),
        &findData,
        INTERNET_FLAG_RELOAD,
        0);

    if (findHandle == nullptr)
    {
        const DWORD errorCode = GetLastError();
        if (errorCode == ERROR_NO_MORE_FILES
            || errorCode == ERROR_FILE_NOT_FOUND
            || errorCode == ERROR_PATH_NOT_FOUND)
        {
            if (m_log != nullptr)
            {
                m_log->write(LogColor::WARNING, "FTP目录为空 | 远程目录：%s", remoteDir.c_str());
            }
            closeFtpSession();
            return true;
        }

        std::string errMsg = "读取FTP目录失败 | 目录：" + remoteDir + " | " + getFtpErrorMsg();
        if (m_log != nullptr)
        {
            m_log->write(LogColor::ERR, errMsg.c_str());
        }
        if (m_messageBoxesEnabled) {
            showErrorMessage("FTP错误", "%s", errMsg.c_str());
        }
        closeFtpSession();
        return false;
    }

    DWORD nextError = ERROR_SUCCESS;
    do
    {
        const std::string name = findData.cFileName;
        if (name.empty() || name == "." || name == "..")
        {
            continue;
        }

        FtpRemoteFileInfo info;
        info.name = name;
        info.path = JoinRemotePath(remoteDir, name);
        info.isDirectory = (findData.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) != 0;
        info.size = FileSizeFromFindData(findData);
        info.modifiedTime = FormatFileTimeText(findData.ftLastWriteTime);
        files.push_back(info);
    } while (InternetFindNextFileA(findHandle, &findData));

    nextError = GetLastError();
    InternetCloseHandle(findHandle);
    if (nextError != ERROR_NO_MORE_FILES)
    {
        std::string errMsg = "读取FTP目录中断 | 目录：" + remoteDir + " | " + getFtpErrorMsg();
        if (m_log != nullptr)
        {
            m_log->write(LogColor::ERR, errMsg.c_str());
        }
        if (m_messageBoxesEnabled) {
            showErrorMessage("FTP错误", "%s", errMsg.c_str());
        }
        closeFtpSession();
        return false;
    }

    std::sort(files.begin(), files.end(), [](const FtpRemoteFileInfo& left, const FtpRemoteFileInfo& right)
        {
            if (left.isDirectory != right.isDirectory)
            {
                return left.isDirectory && !right.isDirectory;
            }
            return _stricmp(left.name.c_str(), right.name.c_str()) < 0;
        });

    if (m_log != nullptr)
    {
        m_log->write(LogColor::SUCCESS, "FTP目录读取完成 | 远程目录：%s | 文件数：%d | 耗时：%lldms",
            remoteDir.c_str(), static_cast<int>(files.size()), ElapsedMs(totalStart));
    }
    closeFtpSession();
    return true;
}

// 真实文件下载逻辑
bool FtpClient::downloadFile(const std::string& remoteFilePath, const std::string& localFilePath) {
    // 1. 日志记录下载开始
    m_log->write(LogColor::DEFAULT, "开始下载文件 | 远程：%s | 本地：%s",
        remoteFilePath.c_str(), localFilePath.c_str());

    // 2. 检查并建立FTP连接
    if (!connectFtpServer()) {
        return false;
    }

    try {
        // 核心：自动创建本地父目录（如 C:/robot/new_dir/ 不存在则创建）
        std::string localParentDir = getParentDir(localFilePath);
        if (!localParentDir.empty()) {
            createLocalDirRecursive(localParentDir);
        }
        // 3. 执行FTP下载（WinINet API）
        BOOL downloadOk = FtpGetFileA(m_hFtpSession,
            remoteFilePath.c_str(),
            localFilePath.c_str(),
            FALSE, // 不覆盖已存在的文件（可改为TRUE）
            FILE_ATTRIBUTE_NORMAL,
            FTP_TRANSFER_TYPE_BINARY,
            0);

        if (downloadOk) {
            // 下载成功：日志+信息弹窗
            std::string successMsg = "文件下载成功 | 远程：" + remoteFilePath + " | 本地：" + localFilePath;
            m_log->write(LogColor::SUCCESS, successMsg.c_str());
            if (m_messageBoxesEnabled) {
                showInfoMessage("FTP成功", "%s", successMsg.c_str());
            }
            closeFtpSession();
            return true;
        }
        else {
            // 下载失败：日志+错误弹窗
            std::string errMsg = "文件下载失败 | 远程：" + remoteFilePath + " | 本地：" + localFilePath + " | " + getFtpErrorMsg();
            m_log->write(LogColor::ERR, errMsg.c_str());
            if (m_messageBoxesEnabled) {
                showErrorMessage("FTP错误", "%s", errMsg.c_str());
            }
            closeFtpSession();
            return false;
        }
    }
    catch (const std::exception& e) {
        // 异常处理：日志+错误弹窗
        std::string errMsg = "文件下载异常：" + std::string(e.what()) + " | 远程：" + remoteFilePath;
        m_log->write(LogColor::ERR, errMsg.c_str());
        if (m_messageBoxesEnabled) {
            showErrorMessage("FTP异常", "%s", errMsg.c_str());
        }
        closeFtpSession();
        return false;
    }
}

// 真实文件删除逻辑
bool FtpClient::deleteFile(const std::string& remoteFilePath, bool askConfirm) {
    // 1. 弹窗确认是否删除
    if (askConfirm && m_messageBoxesEnabled) {
        bool confirm = showConfirmMessage("FTP确认", "是否删除FTP服务器文件：%s？", remoteFilePath.c_str());
        if (!confirm) {
            m_log->write(LogColor::WARNING, "用户取消删除FTP文件：%s", remoteFilePath.c_str());
            return false;
        }
    }

    // 2. 检查并建立FTP连接
    if (!connectFtpServer()) {
        return false;
    }

    try {
        // 3. 执行FTP删除（WinINet API）
        BOOL deleteOk = FtpDeleteFileA(m_hFtpSession, remoteFilePath.c_str());

        if (deleteOk) {
            // 删除成功：日志+信息弹窗
            std::string successMsg = "文件删除成功 | 远程：" + remoteFilePath;
            m_log->write(LogColor::SUCCESS, successMsg.c_str());
            if (m_messageBoxesEnabled) {
                showInfoMessage("FTP成功", "%s", successMsg.c_str());
            }
            closeFtpSession();
            return true;
        }
        else {
            // 删除失败：日志+错误弹窗
            std::string errMsg = "文件删除失败 | 远程：" + remoteFilePath + " | " + getFtpErrorMsg();
            m_log->write(LogColor::ERR, errMsg.c_str());
            if (m_messageBoxesEnabled) {
                showErrorMessage("FTP错误", "%s", errMsg.c_str());
            }
            closeFtpSession();
            return false;
        }
    }
    catch (const std::exception& e) {
        // 异常处理：日志+错误弹窗
        std::string errMsg = "文件删除异常：" + std::string(e.what()) + " | 远程：" + remoteFilePath;
        m_log->write(LogColor::ERR, errMsg.c_str());
        if (m_messageBoxesEnabled) {
            showErrorMessage("FTP异常", "%s", errMsg.c_str());
        }
        closeFtpSession();
        return false;
    }
}
