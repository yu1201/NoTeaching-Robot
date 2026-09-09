#pragma once

#include <cctype>
#include <functional>
#include <string>

// Brand-owned defaults. Never place login credentials in adaptor DTOs or logs.
namespace InovanceUserLogin
{
constexpr int kDefaultLevel = 2; // Administrator, not the robot operation mode.
// Verified on the field controller's API login, independently of pendant login.
constexpr const char* kDefaultPassword = "000000";

struct Credentials
{
    int level = kDefaultLevel;
    std::string password = kDefaultPassword;
};

inline Credentials Resolve(int configuredLevel, const std::string& configuredPassword)
{
    Credentials result;
    // Old templates used 0 to skip login. It now means use the brand default.
    if (configuredLevel != 0) { result.level = configuredLevel; }
    if (!configuredPassword.empty()) { result.password = configuredPassword; }
    return result;
}

struct Ops
{
    std::function<bool(const std::string&, std::string&)> send;
    std::function<bool(const std::string&, int&)> query;
};

// Extract only a bounded protocol code, never the controller's free-form text:
// some firmware versions echo the complete login command, including its password.
inline std::string SafeErrorCode(const std::string& response)
{
    if (response.size() < 2 || (response[0] != 'e' && response[0] != 'E')
        || response[1] < '1' || response[1] > '9') { return {}; }
    std::size_t end = 2;
    if (end < response.size() && response[end] >= '0' && response[end] <= '9') { ++end; }
    if (end < response.size() && response.find_first_of(" :;,\t\r\n", end) != end) { return {}; }
    return "e" + response.substr(1, end - 1);
}

inline std::string RejectionDetail(const std::string& response)
{
    const auto code = SafeErrorCode(response);
    if (code.empty()) { return "控制器应答未通过确认（应答正文已隐藏）"; }
    if (code == "e1") { return "e1：指令语法错误"; }
    if (code == "e2") { return "e2：参数数量错误"; }
    if (code == "e3")
    { return "e3：参数值不合法；该应答不能区分密码、用户级别或固件参数约束，不能直接判定密码错误"; }
    if (code == "e24") { return "e24：当前连接没有控制许可"; }
    if (code == "e25") { return "e25：控制设备不是远程以太网"; }
    if (code == "e27") { return "e27：当前用户级别不足"; }
    return code + "：控制器拒绝登录（应答正文已隐藏）";
}

// Call under the driver's socket mutex. A failed/uncertain authentication must
// not become a password retry loop driven by the two-second state monitor.
class RetryGate
{
public:
    bool Begin(bool explicitRetry, std::string& error)
    {
        if (explicitRetry) { m_error.clear(); }
        if (m_error.empty()) { return true; }
        error = m_error;
        return false;
    }
    void Block(const std::string& error)
    {
        m_error = error + "\n自动登录重试已暂停；核对后请手动点击连接测试重试。";
    }
    const std::string& Error() const { return m_error; }
private:
    std::string m_error;
};

inline bool Login(const Credentials& credentials, const Ops& ops, std::string& error)
{
    error.clear();
    if (credentials.level < 1 || credentials.level > 3 || credentials.password.empty()
        || credentials.password.size() > 8
        || credentials.password.find_first_of("@$") != std::string::npos)
    {
        error = "汇川自动登录参数无效：用户级别应为1..3，密码应为1..8字节且不含协议分隔符。";
        return false;
    }
    for (unsigned char ch : credentials.password)
    {
        if (std::isspace(ch) || std::iscntrl(ch))
        { error = "汇川自动登录密码不能包含空白或控制字符。"; return false; }
    }
    std::string response;
    const bool sent = ops.send("UserLogin " + std::to_string(credentials.level) + " "
        + credentials.password, response);
    if (!sent || response != "ok")
    {
        // Do not echo the command/response: controllers may echo credentials.
        error = "汇川自动登录失败：UserLogin，申请用户级别=" + std::to_string(credentials.level)
            + "；" + (!sent && response.empty()
                ? "通信未确认，登录结果未知；不自动重复提交。" : RejectionDetail(response));
        return false;
    }
    int actualLevel = -1;
    if (!ops.query("CurUserType", actualLevel))
    { error = "汇川自动登录后CurUserType权限回读失败。"; return false; }
    if (actualLevel < 0 || actualLevel > 3)
    { error = "汇川自动登录后CurUserType返回非法用户级别。"; return false; }
    if (actualLevel != credentials.level)
    {
        error = "汇川自动登录权限不匹配：期望=" + std::to_string(credentials.level)
            + "，实际=" + std::to_string(actualLevel) + "。";
        return false;
    }
    return true;
}
}
