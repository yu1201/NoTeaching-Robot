#pragma once

#include "RobotDriverAdaptor.h"

#include <memory>
#include <string>
#include <vector>

class FtpClient;
class RobotLog;

// 独立的 FTP 通信底层。品牌驱动按自身目录和程序格式配置后接入；业务层不可直接依赖本类。
class RobotFtpFileTransfer final : public RobotFileTransferSession
{
public:
    RobotFtpFileTransfer(
        std::string host,
        int port,
        std::string user,
        std::string password,
        RobotFileTransferProfile profile,
        std::vector<std::string> listedProgramExtensions,
        std::vector<std::string> inventoryProgramExtensions,
        std::string logPath);
    ~RobotFtpFileTransfer() override;

    const RobotFileTransferProfile& Profile() const override;
    bool ListProgramFiles(
        const std::string& remoteDirectory,
        std::vector<RobotControllerFileInfo>& entries,
        int timeoutMs = 10000) override;
    bool UploadProgramFile(
        const std::string& localPath,
        const std::string& remotePath,
        bool replaceExisting = true) override;
    bool DownloadProgramFile(
        const std::string& remotePath,
        const std::string& localPath) override;
    bool DeleteProgramFile(const std::string& remotePath) override;
    bool QueryProgramInventory(
        RobotProgramInventoryResult& result,
        int timeoutMs = 10000) override;
    std::string LastError() const override;

private:
    bool EnsureClient();
    bool IsListedProgramFile(const std::string& fileName) const;
    bool ValidateProgramPath(const std::string& path, const char* operation);
    void SetError(std::string error);

    std::string m_host;
    int m_port = 21;
    std::string m_user;
    std::string m_password;
    RobotFileTransferProfile m_profile;
    std::vector<std::string> m_listedProgramExtensions;
    std::vector<std::string> m_inventoryProgramExtensions;
    std::string m_logPath;
    std::unique_ptr<RobotLog> m_log;
    std::unique_ptr<FtpClient> m_client;
    std::string m_lastError;
};
