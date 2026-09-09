#include "InovanceConnectionPreparation.h"

#include <cstdlib>
#include <functional>
#include <iostream>
#include <string>
#include <vector>

using namespace InovanceConnectionPreparation;

static int g_checks = 0;
static void Check(bool value, const char* message)
{
    ++g_checks;
    if (!value) { std::cerr << "FAIL " << message << '\n'; std::exit(1); }
}

struct Mock
{
    State state {1, 0, 0, 0, 0, 0, 0, 2, 1};
    std::vector<std::string> commands;
    bool coordinateOk = true;
    bool kinematicsOk = true;
    std::string failCommand;
    int cancelAfterCommands = -1;
    int reads = 0;
    int delays = 0;
    std::function<void()> beforeRead;

    Ops MakeOps()
    {
        Ops ops;
        ops.cancelled = [this]()
        { return cancelAfterCommands >= 0 && static_cast<int>(commands.size()) >= cancelAfterCommands; };
        ops.delay = [this]() { ++delays; };
        ops.read = [this](State& out, std::string&)
        { ++reads; if (beforeRead) beforeRead(); out = state; return true; };
        ops.prepareCoordinates = [this](std::string& text)
        { text = coordinateOk ? "readback" : "coordinate refused"; return coordinateOk; };
        ops.prepareKinematics = [this](std::string& text)
        { text = kinematicsOk ? "validated" : "kinematics refused"; return kinematicsOk; };
        ops.send = [this](const std::string& command, std::string& reply)
        {
            commands.push_back(command);
            if (command == failCommand) { reply = "mock failure"; return false; }
            if (command == "AcqPermit") state.permit = 1;
            else if (command == "ResetErr") state.fault = 0;
            else if (command == "Prg Stop") { }
            else if (command == "BackStartLine") state.motion = 0;
            else if (command == "Set_Mode 2") state.mode = 2;
            else if (command == "Motor ON") state.motor = 1;
            else { reply = "unexpected command"; return false; }
            reply = "ok";
            return true;
        };
        return ops;
    }
};

static bool Contains(const std::vector<std::string>& commands, const char* value)
{
    for (const auto& command : commands) if (command == value) return true;
    return false;
}

int main()
{
    std::string evidence;
    {
        Mock mock;
        mock.state.fault = 12;
        mock.state.permit = 0;
        Check(Prepare(mock.MakeOps(), evidence), "full preparation succeeds");
        Check(mock.commands == std::vector<std::string>({"AcqPermit", "ResetErr", "Set_Mode 2", "Motor ON"}),
            "full preparation order");
        Check(mock.state.mode == 2 && mock.state.motor == 1 && mock.state.permit == 1,
            "full preparation terminal state");
        Check(evidence.find("未启动运动") != std::string::npos, "success evidence says no motion");
    }
    {
        Mock mock;
        mock.state.mode = 2; mock.state.motor = 1;
        Check(Prepare(mock.MakeOps(), evidence), "already ready is idempotent");
        Check(mock.commands.empty(), "idempotent path sends no command");
    }
    {
        Mock mock;
        int readsAfterModeSwitch = 0;
        mock.beforeRead = [&]()
        {
            if (mock.state.mode == 2 && !Contains(mock.commands, "Motor ON")
                && ++readsAfterModeSwitch == 4) mock.state.motor = 0;
        };
        Check(Prepare(mock.MakeOps(), evidence), "delayed servo drop after mode switch is repaired");
        Check(mock.commands == std::vector<std::string>({"Set_Mode 2", "Motor ON"}),
            "mode must settle before final motor enable");
        Check(mock.state.mode == 2 && mock.state.motor == 1,
            "delayed servo drop terminal state");
    }
    {
        Mock mock;
        mock.state.motion = 2;
        mock.state.permit = 0;
        Check(Prepare(mock.MakeOps(), evidence), "powered-off interrupted baseline is recovered");
        Check(mock.commands == std::vector<std::string>({"AcqPermit", "Prg Stop", "BackStartLine", "Set_Mode 2", "Motor ON"}),
            "interrupted recovery acquires permit, stops and returns to start before preparation");
        Check(mock.state.motion == 0 && mock.state.mode == 2 && mock.state.motor == 1,
            "interrupted recovery reaches normal ready state");
        Check(evidence.find("运动中断态恢复完成") != std::string::npos,
            "interrupted recovery records exact evidence");
    }
    {
        Mock mock;
        mock.state.motion = 2;
        mock.state.motor = 1;
        Check(!Prepare(mock.MakeOps(), evidence), "powered interrupted state is never auto-reset");
        Check(mock.commands.empty(), "powered interrupted state sends no command");
    }
    {
        Mock mock;
        mock.state.motion = 2;
        mock.state.stream = 2;
        Check(!Prepare(mock.MakeOps(), evidence), "paused stream interrupted state is never auto-reset");
        Check(mock.commands.empty(), "paused stream interrupted state sends no command");
    }
    {
        Mock mock;
        mock.state.motion = 2;
        mock.state.permit = 2;
        Check(!Prepare(mock.MakeOps(), evidence), "other-client interrupted state is never taken over");
        Check(mock.commands.empty(), "other-client interrupted state sends no command");
    }
    for (int unsafe = 0; unsafe < 7; ++unsafe)
    {
        Mock mock;
        if (unsafe == 0) mock.state.controlDevice = 0;
        if (unsafe == 1) mock.state.permit = 2;
        if (unsafe == 2) mock.state.estop = 1;
        if (unsafe == 3) mock.state.motion = 1;
        if (unsafe == 4) mock.state.task = 1;
        if (unsafe == 5) mock.state.stream = 1;
        if (unsafe == 6) mock.state.mode = -1;
        Check(!Prepare(mock.MakeOps(), evidence), "unsafe or unknown baseline rejected");
        Check(mock.commands.empty(), "unsafe or unknown baseline sends nothing");
    }
    {
        Mock mock; mock.state.fault = 9; mock.failCommand = "ResetErr";
        Check(!Prepare(mock.MakeOps(), evidence), "alarm reset failure rejects");
        Check(mock.commands.size() == 1 && mock.commands[0] == "ResetErr", "alarm failure stops sequence");
        Check(!Contains(mock.commands, "Set_Mode 2") && !Contains(mock.commands, "Motor ON"),
            "alarm failure never switches or powers");
    }
    {
        Mock mock; mock.coordinateOk = false;
        Check(!Prepare(mock.MakeOps(), evidence), "coordinate failure rejects");
        Check(!Contains(mock.commands, "Set_Mode 2") && !Contains(mock.commands, "Motor ON"),
            "coordinate failure stops before mode and motor");
    }
    {
        Mock mock; mock.kinematicsOk = false;
        Check(!Prepare(mock.MakeOps(), evidence), "kinematics failure rejects");
        Check(!Contains(mock.commands, "Set_Mode 2") && !Contains(mock.commands, "Motor ON"),
            "kinematics failure stops before mode and motor");
    }
    {
        Mock mock; mock.failCommand = "Set_Mode 2";
        Check(!Prepare(mock.MakeOps(), evidence), "mode failure rejects");
        Check(!Contains(mock.commands, "Motor ON"), "mode failure never powers");
    }
    {
        Mock mock; mock.failCommand = "Motor ON";
        Check(!Prepare(mock.MakeOps(), evidence), "motor failure rejects");
        Check(mock.commands.back() == "Motor ON", "motor failure is terminal command");
    }
    {
        Mock mock; mock.cancelAfterCommands = 0;
        Check(!Prepare(mock.MakeOps(), evidence), "pre-cancel rejects");
        Check(mock.commands.empty(), "pre-cancel sends nothing");
    }
    {
        Mock mock; mock.state.permit = 0; mock.cancelAfterCommands = 1;
        Check(!Prepare(mock.MakeOps(), evidence), "cancel after permit rejects");
        Check(mock.commands.size() == 1 && mock.commands[0] == "AcqPermit", "cancel stops subsequent writes");
    }
    {
        Mock mock;
        Ops incomplete;
        Check(!Prepare(incomplete, evidence), "missing callbacks rejected");
    }
    {
        Mock mock;
        Check(Prepare(mock.MakeOps(), evidence), "command allow-list success");
        for (const auto& command : mock.commands)
        {
            Check(command == "AcqPermit" || command == "ResetErr"
                || command == "Prg Stop" || command == "BackStartLine"
                || command == "Set_Mode 2" || command == "Motor ON", "no hidden motion or emergency command");
        }
    }
    std::cout << "PASS: " << g_checks
        << " Inovance explicit connection-preparation checks; no network or real robot\n";
    return 0;
}
