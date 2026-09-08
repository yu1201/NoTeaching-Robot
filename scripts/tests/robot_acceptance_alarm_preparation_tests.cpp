#include "RobotAcceptanceAlarmPreparation.h"

#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

struct Status
{
    bool valid = true;
    bool connected = true;
    bool emergencyStopKnown = true;
    bool emergencyStop = false;
    bool systemFaultKnown = true;
    bool systemFault = false;
    bool systemWarning = false;
    struct Motion { bool terminalVerified = true; } motion;
};

struct MockAdaptor
{
    Status after;
    bool resetOk = true;
    std::string lastError;
    std::vector<std::string> calls;
    void ClearLastRobotError() { calls.push_back("clear-local-error"); lastError.clear(); }
    bool cleanAlarm()
    {
        calls.push_back("reset");
        if (!resetOk) { lastError = "reset rejected"; }
        return resetOk;
    }
    std::string GetLastRobotError() const { return lastError; }
    Status ReadControllerStatus() { calls.push_back("readback"); lastError.clear(); return after; }
};

static void Check(bool condition, const char* label)
{
    if (!condition) { std::cerr << "FAIL: " << label << '\n'; std::exit(1); }
}

int main()
{
    int cases = 0;
    const auto noCommand = [&cases](Status status, bool structured, bool resetCapability,
        bool expectedReady, const char* label)
    {
        MockAdaptor driver;
        const auto result = RobotAcceptanceAlarmPreparation::Prepare(driver, status, structured, resetCapability);
        Check(result.ready == expectedReady && !result.attempted && driver.calls.empty(), label);
        if (!expectedReady) { Check(!result.blockReason.empty(), "blocked preflight includes reason"); }
        ++cases;
    };
    Status clean;
    noCommand(clean, true, true, true, "no alarm: no reset needed");
    noCommand(clean, true, false, true, "no alarm: reset capability not mandatory");
    Status fault; fault.systemFault = true;
    noCommand(fault, false, true, true, "legacy brand: do not blindly reset, preserve native preparation");
    noCommand(fault, true, false, false, "alarm without reset capability blocks enable");
    Status invalid = fault; invalid.valid = false;
    noCommand(invalid, true, true, false, "invalid status blocks reset and enable");
    invalid = fault; invalid.connected = false;
    noCommand(invalid, true, true, false, "disconnected status blocks reset and enable");
    invalid = fault; invalid.emergencyStopKnown = false;
    noCommand(invalid, true, true, false, "unknown emergency stop blocks reset and enable");
    invalid = fault; invalid.emergencyStop = true;
    noCommand(invalid, true, true, false, "physical emergency stop blocks reset and enable");
    invalid = fault; invalid.systemFaultKnown = false;
    noCommand(invalid, true, true, false, "unknown alarm state blocks reset and enable");
    invalid = fault; invalid.motion.terminalVerified = false;
    noCommand(invalid, true, true, false, "unverified stopped state blocks reset and enable");

    const auto resetCase = [&cases](Status before, Status after, bool resetOk, bool expectedReady,
        const char* label)
    {
        MockAdaptor driver;
        driver.after = after;
        driver.resetOk = resetOk;
        const auto result = RobotAcceptanceAlarmPreparation::Prepare(driver, before, true, true);
        Check(result.ready == expectedReady && result.attempted && result.readbackAttempted, label);
        Check(driver.calls == std::vector<std::string>{ "clear-local-error", "reset", "readback" },
            "exactly one reset followed by readback, no retries");
        Check(result.commandOk == resetOk, "preserve reset command result");
        if (!resetOk && after.valid && !after.systemFault)
        { Check(result.readbackOk && !result.ready, "clean readback reported separately from failed reset ACK"); }
        if (!resetOk) { Check(result.commandError == "reset rejected", "readback cannot erase reset error"); }
        if (!expectedReady) { Check(!result.blockReason.empty(), "failed reset includes blocking reason"); }
        ++cases;
    };
    resetCase(fault, clean, true, true, "released emergency stop with latched alarm: reset then verify");
    resetCase(fault, clean, false, false, "failed ACK blocks enable even if readback looks clean");
    resetCase(fault, fault, true, false, "successful ACK with persistent alarm blocks enable");
    invalid = clean; invalid.valid = false;
    resetCase(fault, invalid, true, false, "failed readback blocks enable");
    invalid = clean; invalid.emergencyStop = true;
    resetCase(fault, invalid, true, false, "new emergency stop after reset blocks enable");
    invalid = clean; invalid.emergencyStopKnown = false;
    resetCase(fault, invalid, true, false, "unknown emergency stop after reset blocks enable");
    invalid = clean; invalid.motion.terminalVerified = false;
    resetCase(fault, invalid, true, false, "motion state after reset blocks enable");
    Status warning; warning.systemWarning = true;
    resetCase(warning, clean, true, true, "warning-only reset is verified");
    resetCase(fault, warning, true, false, "persistent warning blocks enable");
    std::cout << "PASS: " << cases << " alarm preparation cases; no robot connection or motion\n";
}
