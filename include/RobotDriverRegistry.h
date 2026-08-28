#pragma once

#include <string>
#include <vector>

class RobotDriverAdaptor;
class RobotLog;

struct RobotDriverSetupProfile
{
    // Stable database identity for the brand template.  This must never be a
    // real control-unit name (RobotA/RobotB/RobotC, etc.).
    const char* templateId = "";
    int defaultSocketPort = 0;
    int defaultMonitorPort = 0;
    bool usesMonitorPort = false;
    int defaultFtpPort = 21;
    const char* defaultFtpHost = "";
    const char* defaultFtpUser = "";
    const char* defaultFtpPassword = "";
    bool usesControllerProject = false;
    const char* defaultControllerProject = "";
    const char* remoteProgramRoot = "";
    bool appendProjectSrDirectory = false;
    const char* localJobSubdirectory = "";
};

struct RobotDriverRegistration
{
    int typeCode = 0;
    const char* displayName = "UNKNOWN";
    RobotDriverAdaptor* (*create)(const std::string& unitName, RobotLog* log) = nullptr;
    RobotDriverSetupProfile setup;
};

// 机器人品牌底层的唯一登记点。业务层只使用 RobotDriverAdaptor；新品牌只有在
// 完成适配层全部纯接口、能力位和现场验证后，才允许在这里登记并出现在配置界面。
class RobotDriverRegistry
{
public:
    static const std::vector<RobotDriverRegistration>& RegisteredTypes();
    static const RobotDriverRegistration* Find(int typeCode);
    static const RobotDriverSetupProfile* SetupProfile(int typeCode);
    static bool IsRegistered(int typeCode);
    static std::string DisplayName(int typeCode);
    static RobotDriverAdaptor* Create(
        int typeCode,
        const std::string& unitName,
        RobotLog* log,
        std::string* error = nullptr);
};
