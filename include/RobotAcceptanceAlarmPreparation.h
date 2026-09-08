#pragma once

#include <string>

// Business-side sequencing only. All I/O stays behind the robot adaptor.
// The template also lets the exact sequence be tested without a robot or an SDK.
namespace RobotAcceptanceAlarmPreparation
{
template<class Status>
struct Result
{
    bool ready = false;
    bool attempted = false;
    bool commandOk = false;
    bool readbackAttempted = false;
    bool readbackOk = false;
    Status after;
    std::string commandError;
    std::string blockReason;
};

template<class Driver, class Status>
Result<Status> Prepare(Driver& driver, const Status& before,
    bool structuredStatusSupported, bool alarmResetSupported)
{
    Result<Status> result;
    result.after = before;
    // Keep the existing native preparation path for older brands; never blindly
    // reset their alarms when emergency-stop status cannot be read independently.
    if (!structuredStatusSupported)
    {
        result.ready = true;
        return result;
    }
    const auto safeToPrepare = [](const Status& status) -> std::string
    {
        if (!status.valid || !status.connected) { return "控制器状态读取失败或连接无效，禁止自动复位和上电。"; }
        if (!status.emergencyStopKnown) { return "无法确认实体急停状态，禁止自动复位和上电。"; }
        if (status.emergencyStop) { return "实体急停仍处于触发状态，请现场解除；未发送报警复位或上电命令。"; }
        if (!status.systemFaultKnown) { return "无法确认控制器报警状态，禁止自动复位和上电。"; }
        if (!status.motion.terminalVerified) { return "尚未确认机器人停止，禁止自动复位和上电，请现场检查。"; }
        return {};
    };
    result.blockReason = safeToPrepare(before);
    if (!result.blockReason.empty()) { return result; }
    if (!before.systemFault && !before.systemWarning)
    {
        result.ready = true;
        return result;
    }
    if (!alarmResetSupported)
    {
        result.blockReason = "当前品牌未声明报警复位能力，请在示教器复位报警后重试。";
        return result;
    }
    driver.ClearLastRobotError();
    result.attempted = true;
    result.commandOk = driver.cleanAlarm();
    result.commandError = result.commandOk ? std::string() : driver.GetLastRobotError();
    // Always read back, even after a failed ACK. A readback must not turn a failed
    // command into success, and a successful ACK alone is never enough to enable.
    result.readbackAttempted = true;
    result.after = driver.ReadControllerStatus();
    result.blockReason = safeToPrepare(result.after);
    result.readbackOk = result.blockReason.empty()
        && !result.after.systemFault && !result.after.systemWarning;
    if (!result.commandOk)
    {
        result.blockReason = "报警复位接口失败，未继续上电或运动。";
    }
    else if (result.blockReason.empty() && (result.after.systemFault || result.after.systemWarning))
    {
        result.blockReason = "报警复位后控制器仍有报警或警告，未继续上电或运动。";
    }
    result.ready = result.commandOk && result.readbackOk;
    return result;
}
}
