#pragma once
#ifndef FTPCLIENT_H
#define FTPCLIENT_H

#include "RobotLog.h"
#include "RobotMessage.h"
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
        int ftpPort = 21,
        const std::string& ftpUser = "root",
        const std::string& ftpPwd = "STEP_ROBOT_SRH"
       );

    // 新增：路径处理
    std::string getParentDir(const std::string& filePath);
    bool createLocalDirRecursive(const std::string& localDir);
    bool createRemoteDirRecursive(const std::string& remoteDir);

    // 核心FTP操作（真实实现）
    bool uploadFile(const std::string& localFilePath, const std::string& remoteFilePath, bool deleteBeforeUpload = true);
    bool listFiles(const std::string& remoteDir, std::vector<FtpRemoteFileInfo>& files);
    bool downloadFile(const std::string& remoteFilePath, const std::string& localFilePath);
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
    std::wstring s2w(const std::string& str);
};

#endif // FTPCLIENT_H
