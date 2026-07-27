#pragma once
#ifndef FTPCLIENT_H
#define FTPCLIENT_H

#include "RobotLog.h"
#include "RobotMessage.h"
#include <atomic>
#include <functional>
#include <string>
#include <vector>
#include <windows.h>
#include <wininet.h>

#include <Shlwapi.h>
#include <direct.h>
#pragma comment(lib, "Shlwapi.lib")

// 链接WinINet库（避免手动配置链接器）
#pragma comment(lib, "Wininet.lib")

struct FtpRemoteFileInfo
{
    std::string name;
    std::string path;
    bool isDirectory = false;
    unsigned long long size = 0;
    std::string modifiedTime;
};

class FtpClient {
public:
    // 构造函数：初始化日志和弹窗，指定FTP服务器信息
    FtpClient(RobotLog* log,
        const std::string& ftpHost,
        int ftpPort,
        const std::string& ftpUser,
        const std::string& ftpPwd
       );

    // 显式建立（或复用）FTP 连接：成功后会话可被后续 createRemoteDirRecursive/上传复用。
    // 供上传预检先行探测登录，从而把「登录失败」与「建目录失败」分开报错。
    bool connect();

    // 新增：路径处理
    std::string getParentDir(const std::string& filePath);
    bool createLocalDirRecursive(const std::string& localDir);
    bool createRemoteDirRecursive(const std::string& remoteDir);

    // 核心FTP操作（真实实现）
    bool uploadFile(const std::string& localFilePath, const std::string& remoteFilePath, bool deleteBeforeUpload = true);
    // 分块上传（带进度回调 + 可取消）：progressCb(已发送字节, 总字节) 每块调用一次；
    // cancelFlag 置真则块间中止；allowRemoteDelete=true 时尝试删除服务器半截文件。
    // 写一次上传账号必须传 false；不完整唯一件靠声明长度过滤，并由服务器定期清理。
    // 与 uploadFile(FtpPutFileA) 并存，不影响 STEP 机器人等既有原子上传调用。
    bool uploadFileWithProgress(const std::string& localFilePath,
        const std::string& remoteFilePath,
        std::atomic<bool>* cancelFlag,
        const std::function<void(long long sent, long long total)>& progressCb,
        bool allowRemoteDelete = true);
    bool listFiles(const std::string& remoteDir,
        std::vector<FtpRemoteFileInfo>& files,
        std::atomic<bool>* cancelFlag = nullptr,
        size_t maximumEntries = 10000);
    bool downloadFile(const std::string& remoteFilePath, const std::string& localFilePath);
    // Bounded/cancellable download used by the remote archive browser. The remote
    // size observed after FtpOpenFile must equal the earlier LIST snapshot, including
    // for legacy .zip names without an embedded length. The destination is created
    // exclusively and every failure/cancellation removes the partial file.
    bool downloadFileBounded(const std::string& remoteFilePath,
        const std::string& localFilePath,
        unsigned long long expectedRemoteBytes,
        unsigned long long maximumBytes,
        std::atomic<bool>* cancelFlag,
        const std::function<void(unsigned long long received,
            unsigned long long total)>& progressCb = {});
    bool deleteFile(const std::string& remoteFilePath, bool askConfirm = true);
    void setMessageBoxesEnabled(bool enabled);

    // 析构函数：释放FTP连接
    ~FtpClient();

private:
    // 成员变量
    RobotLog* m_log;               // 日志实例
    std::string m_ftpHost;        // FTP服务器地址
    int m_ftpPort;                // FTP端口
    std::string m_ftpUser;        // FTP用户名
    std::string m_ftpPwd;         // FTP密码
    HINTERNET m_hInternet;        // WinINet根句柄
    HINTERNET m_hFtpSession;      // FTP会话句柄
    bool m_messageBoxesEnabled = true;

    // 内部辅助函数
    bool connectFtpServer();      // 真实FTP连接逻辑
    void closeFtpSession();       // 释放FTP句柄
    std::string getFtpErrorMsg(); // 获取WinINet错误信息
private:
    // 字符串转宽字符串（替代StringToWString）
    // 注：不要再引入 string→wstring 的逐字节拉宽转换配合 W 版 API——中文路径会被系统代码页
    // 转成「?」乱码；远程路径一律 A 版 API + UTF-8 字节直通（见 createRemoteDirRecursive 注释）。
};

#endif // FTPCLIENT_H
