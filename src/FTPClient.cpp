#include "FtpClient.h"
#include <stdexcept>
#include <fstream>
#include <sstream>
#include <chrono>
#include <algorithm>
#include <array>
#include <cstdio>
#include <cstring>

namespace
{
    constexpr DWORD kFtpConnectTimeoutMs = 15 * 1000;
    constexpr DWORD kFtpIoTimeoutMs = 30 * 1000;

    bool SetFtpInternetTimeout(HINTERNET handle, DWORD option, DWORD timeoutMs)
    {
        return handle != nullptr
            && InternetSetOptionA(handle, option, &timeoutMs, sizeof(timeoutMs)) != FALSE;
    }

    bool ConfigureFtpInternetTimeouts(HINTERNET handle)
    {
        // Set on the InternetOpen root before InternetConnect; WinINet inherits
        // these options to every derived FTP connection/file/find handle.
        return SetFtpInternetTimeout(handle, INTERNET_OPTION_CONNECT_TIMEOUT, kFtpConnectTimeoutMs)
            && SetFtpInternetTimeout(handle, INTERNET_OPTION_SEND_TIMEOUT, kFtpIoTimeoutMs)
            && SetFtpInternetTimeout(handle, INTERNET_OPTION_RECEIVE_TIMEOUT, kFtpIoTimeoutMs)
            && SetFtpInternetTimeout(handle, INTERNET_OPTION_DATA_SEND_TIMEOUT, kFtpIoTimeoutMs)
            && SetFtpInternetTimeout(handle, INTERNET_OPTION_DATA_RECEIVE_TIMEOUT, kFtpIoTimeoutMs);
    }

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

bool FtpClient::connect() {
    return connectFtpServer();
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
// 编码约定：整个 FtpClient 的远程路径一律走 A 版 API + UTF-8 字节直通（A 版把字节原样发上 FTP 线，
// Linux/vsftpd 按字节存、UTF-8 环境显示正确，与 FtpOpenFileA/FtpFindFirstFileA 等既有调用一致）。
// 切勿改回 W 版：W 版内部按系统 ANSI 代码页转换，中文设备名会被转成「?」乱码目录，
// 且与 A 版传文件的路径对不上导致上传 550 失败（2026-07-10 现场实证）。
bool FtpClient::createRemoteDirRecursive(const std::string& remoteDir) {
    if (remoteDir.empty()) return false;
    // 会话未建立时先连接：本方法可被外部（如扫描数据上传的建目录预检）在构造后直接调用，
    // 而构造函数并不连接。内部递归及 uploadFile* 调用时会话已就绪，connectFtpServer 会复用不重连。
    if (!m_hFtpSession && !connectFtpServer()) return false;

    char oldDir[MAX_PATH] = { 0 };
    DWORD bufSize = MAX_PATH; // 改为 DWORD
    const BOOL hasOldDir = FtpGetCurrentDirectoryA(m_hFtpSession, oldDir, &bufSize); // 传地址
    BOOL exist = FtpSetCurrentDirectoryA(m_hFtpSession, remoteDir.c_str());

    if (exist) {
        if (hasOldDir) {
            FtpSetCurrentDirectoryA(m_hFtpSession, oldDir);
        }
        return true;
    }

    std::string parent = getParentDir(remoteDir);
    if (!parent.empty() && !createRemoteDirRecursive(parent)) {
        if (hasOldDir) {
            FtpSetCurrentDirectoryA(m_hFtpSession, oldDir);
        }
        return false;
    }

    if (FtpCreateDirectoryA(m_hFtpSession, remoteDir.c_str())) {
        m_log->write(LogColor::DEFAULT, "远程目录已创建: %s", remoteDir.c_str());
    }
    else if (!FtpSetCurrentDirectoryA(m_hFtpSession, remoteDir.c_str())) {
        m_log->write(LogColor::ERR, "远程目录确认失败: %s | %s", remoteDir.c_str(), getFtpErrorMsg().c_str());
        if (hasOldDir) {
            FtpSetCurrentDirectoryA(m_hFtpSession, oldDir);
        }
        return false;
    }

    if (hasOldDir) {
        FtpSetCurrentDirectoryA(m_hFtpSession, oldDir);
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
            char currentDir[MAX_PATH] = { 0 };
            DWORD currentDirSize = MAX_PATH;
            if (FtpGetCurrentDirectoryA(m_hFtpSession, currentDir, &currentDirSize)) {
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
            m_log->write(LogColor::ERR, "%s", errMsg.c_str());
            if (m_messageBoxesEnabled) {
                showErrorMessage("FTP错误", "%s", errMsg.c_str());
            }
            return false;
        }
        if (!ConfigureFtpInternetTimeouts(m_hInternet)) {
            const std::string errMsg = "FTP超时策略设置失败，已拒绝建立无界连接 | " + getFtpErrorMsg();
            m_log->write(LogColor::ERR, "%s", errMsg.c_str());
            if (m_messageBoxesEnabled) {
                showErrorMessage("FTP错误", "%s", errMsg.c_str());
            }
            closeFtpSession();
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
            m_log->write(LogColor::ERR, "%s", errMsg.c_str());
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
        m_log->write(LogColor::ERR, "%s", errMsg.c_str());
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
                m_log->write(LogColor::ERR, "%s", errMsg.c_str());
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
            m_log->write(LogColor::SUCCESS, "%s", successMsg.c_str());
            m_log->write(LogColor::SUCCESS, "FTP上传完成 | PutFile=%lldms | Total=%lldms | 远程=%s",
                putMs, ElapsedMs(totalStart), remoteFilePath.c_str());
            return true;
        }
        else {
            // 上传失败：日志+错误弹窗
            std::string errMsg = "文件上传失败 | 本地：" + localFilePath + " | 远程：" + remoteFilePath + " | " + getFtpErrorMsg();
            m_log->write(LogColor::ERR, "%s", errMsg.c_str());
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
        m_log->write(LogColor::ERR, "%s", errMsg.c_str());
        if (m_messageBoxesEnabled) {
            showErrorMessage("FTP异常", "%s", errMsg.c_str());
        }
        closeFtpSession();
        return false;
    }
}

bool FtpClient::uploadFileWithProgress(const std::string& localFilePath,
    const std::string& remoteFilePath,
    std::atomic<bool>* cancelFlag,
    const std::function<void(long long sent, long long total)>& progressCb,
    bool allowRemoteDelete)
{
    auto cancelled = [cancelFlag]() { return cancelFlag != nullptr && cancelFlag->load(); };

    if (!connectFtpServer()) {
        return false;
    }

    // 打开本地文件，取总大小（进度/ETA 用）。
    FILE* fp = nullptr;
    if (fopen_s(&fp, localFilePath.c_str(), "rb") != 0 || fp == nullptr) {
        m_log->write(LogColor::ERR, "分块上传打不开本地文件：%s", localFilePath.c_str());
        closeFtpSession();
        return false;
    }
    if (_fseeki64(fp, 0, SEEK_END) != 0) {
        m_log->write(LogColor::ERR, "分块上传无法定位本地文件末尾：%s", localFilePath.c_str());
        fclose(fp);
        closeFtpSession();
        return false;
    }
    const long long total = _ftelli64(fp);
    if (total < 0 || _fseeki64(fp, 0, SEEK_SET) != 0) {
        m_log->write(LogColor::ERR, "分块上传无法读取本地文件大小：%s", localFilePath.c_str());
        fclose(fp);
        closeFtpSession();
        return false;
    }

    // 确保远程父目录存在。
    const std::string remoteParentDir = getParentDir(remoteFilePath);
    if (!remoteParentDir.empty() && !createRemoteDirRecursive(remoteParentDir)) {
        m_log->write(LogColor::ERR, "分块上传远程目录确认失败：%s", remoteParentDir.c_str());
        fclose(fp);
        closeFtpSession();
        return false;
    }

    if (allowRemoteDelete) {
        FtpDeleteFileA(m_hFtpSession, remoteFilePath.c_str());  // 静默删旧文件，失败无妨（可能不存在）
    }

    // 打开远程文件写句柄（二进制）。
    HINTERNET hRemote = FtpOpenFileA(m_hFtpSession, remoteFilePath.c_str(),
        GENERIC_WRITE, FTP_TRANSFER_TYPE_BINARY, 0);
    if (hRemote == nullptr) {
        m_log->write(LogColor::ERR, "分块上传打开远程文件失败：%s | %s",
            remoteFilePath.c_str(), getFtpErrorMsg().c_str());
        fclose(fp);
        closeFtpSession();
        return false;
    }

    if (progressCb) {
        progressCb(0, total);
    }

    // 管理员账号可主动删半截文件；写一次 uploader 不发送被服务器拒绝的 DELE。
    // 提前 EOF 仍可能收到 FTP 226，因此残件靠声明长度隐藏并由服务器定期清理。
    auto cleanupPartial = [this, &remoteFilePath, allowRemoteDelete]() {
        return allowRemoteDelete && FtpDeleteFileA(m_hFtpSession, remoteFilePath.c_str()) != FALSE;
    };

    const size_t kChunk = 256 * 1024;  // 256KB/块：足够块间检查取消/更新进度，又不过碎
    std::vector<char> buf(kChunk);
    long long sent = 0;
    bool ok = true;
    while (sent < total) {
        if (cancelled()) {
            ok = false;
            break;
        }
        const size_t remaining = static_cast<size_t>(
            std::min<long long>(static_cast<long long>(kChunk), total - sent));
        const size_t nread = fread(buf.data(), 1, remaining, fp);
        if (nread == 0) {
            m_log->write(LogColor::ERR, ferror(fp)
                ? "分块上传读取本地文件失败 @%lld/%lld：%s"
                : "分块上传本地文件提前结束 @%lld/%lld：%s",
                sent, total, localFilePath.c_str());
            ok = false;
            break;
        }
        DWORD written = 0;
        if (!InternetWriteFile(hRemote, buf.data(), static_cast<DWORD>(nread), &written) || written != nread) {
            m_log->write(LogColor::ERR, "分块上传写入失败 @%lld/%lld：%s",
                sent, total, getFtpErrorMsg().c_str());
            ok = false;
            break;
        }
        sent += static_cast<long long>(written);
        if (progressCb) {
            progressCb(sent, total);
        }
    }

    if (sent != total) {
        ok = false;
    }
    const bool localClosed = fclose(fp) == 0;
    const bool remoteClosed = InternetCloseHandle(hRemote) != FALSE;
    if (!localClosed || !remoteClosed) {
        m_log->write(LogColor::ERR, "分块上传最终关闭失败 @%lld/%lld：%s", sent, total,
            remoteFilePath.c_str());
        ok = false;
    }

    if (!ok) {
        const bool partialDeleted = cleanupPartial();
        m_log->write(cancelled() ? LogColor::DEFAULT : LogColor::ERR,
            partialDeleted
                ? "分块上传%s，已删除服务器半截文件：%s"
                : "分块上传%s，未发送远程删除命令；唯一残件可能暂存至服务器定期清理：%s",
            cancelled() ? "被取消" : "失败", remoteFilePath.c_str());
        return false;
    }

    m_log->write(LogColor::SUCCESS, cancelled()
        ? "分块上传完成（随后收到取消，仅停止后续项） | %lld 字节 | 远程：%s"
        : "分块上传完成 | %lld 字节 | 远程：%s",
        sent, remoteFilePath.c_str());
    return true;
}

bool FtpClient::listFiles(
    const std::string& remoteDir,
    std::vector<FtpRemoteFileInfo>& files,
    std::atomic<bool>* cancelFlag,
    size_t maximumEntries)
{
    files.clear();
    auto cancelled = [cancelFlag]()
        { return cancelFlag != nullptr && cancelFlag->load(); };
    if (maximumEntries == 0 || maximumEntries > 10000 || cancelled())
    {
        return false;
    }
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
            m_log->write(LogColor::ERR, "%s", errMsg.c_str());
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
        if (cancelled())
        {
            nextError = ERROR_OPERATION_ABORTED;
            break;
        }
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
        if (files.size() >= maximumEntries)
        {
            nextError = ERROR_BUFFER_OVERFLOW;
            break;
        }
        files.push_back(info);
    } while (InternetFindNextFileA(findHandle, &findData));

    if (nextError == ERROR_SUCCESS)
    {
        nextError = GetLastError();
    }
    InternetCloseHandle(findHandle);
    if (nextError != ERROR_NO_MORE_FILES)
    {
        SetLastError(nextError);
        std::string errMsg = "读取FTP目录中断 | 目录：" + remoteDir + " | " + getFtpErrorMsg();
        if (m_log != nullptr)
        {
            m_log->write(LogColor::ERR, "%s", errMsg.c_str());
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
            m_log->write(LogColor::SUCCESS, "%s", successMsg.c_str());
            if (m_messageBoxesEnabled) {
                showInfoMessage("FTP成功", "%s", successMsg.c_str());
            }
            closeFtpSession();
            return true;
        }
        else {
            // 下载失败：日志+错误弹窗
            std::string errMsg = "文件下载失败 | 远程：" + remoteFilePath + " | 本地：" + localFilePath + " | " + getFtpErrorMsg();
            m_log->write(LogColor::ERR, "%s", errMsg.c_str());
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
        m_log->write(LogColor::ERR, "%s", errMsg.c_str());
        if (m_messageBoxesEnabled) {
            showErrorMessage("FTP异常", "%s", errMsg.c_str());
        }
        closeFtpSession();
        return false;
    }
}

bool FtpClient::downloadFileBounded(
    const std::string& remoteFilePath,
    const std::string& localFilePath,
    unsigned long long expectedRemoteBytes,
    unsigned long long maximumBytes,
    std::atomic<bool>* cancelFlag)
{
    auto cancelled = [cancelFlag]()
        { return cancelFlag != nullptr && cancelFlag->load(); };
    if (expectedRemoteBytes == 0
        || maximumBytes == 0
        || expectedRemoteBytes > maximumBytes
        || cancelled())
    {
        return false;
    }
    if (m_log != nullptr)
    {
        m_log->write(LogColor::DEFAULT,
            "开始受限FTP下载 | 远程：%s | 声明：%llu | 上限：%llu",
            remoteFilePath.c_str(), expectedRemoteBytes, maximumBytes);
    }
    if (!connectFtpServer() || cancelled())
    {
        closeFtpSession();
        return false;
    }

    HINTERNET remoteFile = FtpOpenFileA(
        m_hFtpSession,
        remoteFilePath.c_str(),
        GENERIC_READ,
        FTP_TRANSFER_TYPE_BINARY | INTERNET_FLAG_RELOAD,
        0);
    if (remoteFile == nullptr)
    {
        closeFtpSession();
        return false;
    }
    DWORD highSize = 0;
    SetLastError(ERROR_SUCCESS);
    const DWORD lowSize = FtpGetFileSize(remoteFile, &highSize);
    const DWORD sizeError = GetLastError();
    const bool sizeKnown = lowSize != INVALID_FILE_SIZE || sizeError == ERROR_SUCCESS;
    const unsigned long long openedBytes =
        (static_cast<unsigned long long>(highSize) << 32)
        | static_cast<unsigned long long>(lowSize);
    if (!sizeKnown
        || openedBytes != expectedRemoteBytes
        || openedBytes > maximumBytes
        || cancelled())
    {
        InternetCloseHandle(remoteFile);
        closeFtpSession();
        if (m_log != nullptr)
        {
            m_log->write(LogColor::ERR,
                "受限FTP下载拒绝：打开后大小与LIST声明不一致或越界 | LIST=%llu | OPEN=%llu",
                expectedRemoteBytes, openedBytes);
        }
        return false;
    }

    const HANDLE localFile = CreateFileA(
        localFilePath.c_str(),
        GENERIC_WRITE,
        0,
        nullptr,
        CREATE_NEW,
        FILE_ATTRIBUTE_NORMAL,
        nullptr);
    if (localFile == INVALID_HANDLE_VALUE)
    {
        InternetCloseHandle(remoteFile);
        closeFtpSession();
        return false;
    }

    bool ok = true;
    unsigned long long received = 0;
    std::array<unsigned char, 64 * 1024> buffer{};
    while (!cancelled())
    {
        DWORD bytesRead = 0;
        if (!InternetReadFile(
            remoteFile,
            buffer.data(),
            static_cast<DWORD>(buffer.size()),
            &bytesRead))
        {
            ok = false;
            break;
        }
        if (bytesRead == 0)
        {
            break;
        }
        if (bytesRead > maximumBytes
            || received > maximumBytes - bytesRead
            || received + bytesRead > expectedRemoteBytes)
        {
            ok = false;
            break;
        }
        DWORD writtenTotal = 0;
        while (writtenTotal < bytesRead)
        {
            DWORD written = 0;
            if (!WriteFile(
                localFile,
                buffer.data() + writtenTotal,
                bytesRead - writtenTotal,
                &written,
                nullptr)
                || written == 0)
            {
                ok = false;
                break;
            }
            writtenTotal += written;
        }
        if (!ok)
        {
            break;
        }
        received += bytesRead;
    }
    ok = ok
        && !cancelled()
        && received == expectedRemoteBytes
        && FlushFileBuffers(localFile) != FALSE;
    CloseHandle(localFile);
    InternetCloseHandle(remoteFile);
    closeFtpSession();
    if (!ok)
    {
        DeleteFileA(localFilePath.c_str());
        if (m_log != nullptr)
        {
            m_log->write(cancelled() ? LogColor::WARNING : LogColor::ERR,
                "受限FTP下载%s | 已收：%llu/%llu | 远程：%s",
                cancelled() ? "取消" : "失败", received, expectedRemoteBytes,
                remoteFilePath.c_str());
        }
        return false;
    }
    if (m_log != nullptr)
    {
        m_log->write(LogColor::SUCCESS,
            "受限FTP下载完成 | %llu 字节 | 远程：%s",
            received, remoteFilePath.c_str());
    }
    return true;
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
            m_log->write(LogColor::SUCCESS, "%s", successMsg.c_str());
            if (m_messageBoxesEnabled) {
                showInfoMessage("FTP成功", "%s", successMsg.c_str());
            }
            closeFtpSession();
            return true;
        }
        else {
            // 删除失败：日志+错误弹窗
            std::string errMsg = "文件删除失败 | 远程：" + remoteFilePath + " | " + getFtpErrorMsg();
            m_log->write(LogColor::ERR, "%s", errMsg.c_str());
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
        m_log->write(LogColor::ERR, "%s", errMsg.c_str());
        if (m_messageBoxesEnabled) {
            showErrorMessage("FTP异常", "%s", errMsg.c_str());
        }
        closeFtpSession();
        return false;
    }
}
