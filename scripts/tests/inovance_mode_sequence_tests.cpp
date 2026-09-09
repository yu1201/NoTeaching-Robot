#include "InovanceModeSequence.h"
#include <algorithm>
#include <iostream>
#include <stdexcept>

using namespace InovanceModeSequence;

struct Controller
{
    State state{1, 0, 0, 0, 0, 0, 0};
    std::vector<std::string> commands;
    std::function<bool(const std::string&, std::string&)> reject;
    std::function<void(const std::string&)> afterSend;
    std::function<void()> beforeRead;
    bool cancelled = false, readFails = false, ignoreCommands = false;
    int reads = 0;
    Ops Operations()
    {
        Ops ops;
        ops.cancelled = [this]() { return cancelled; };
        ops.delay = []() {};
        ops.read = [this](State& value, std::string& error)
        {
            ++reads;
            if (beforeRead) { beforeRead(); }
            if (readFails) { error = "simulated disconnect"; return false; }
            value = state;
            return true;
        };
        ops.send = [this](const std::string& command, std::string& reply)
        {
            commands.push_back(command);
            if (reject && reject(command, reply)) { return false; }
            if (!ignoreCommands)
            {
                if (command == "Set_Mode 1") { state.mode = 1; }
                else if (command == "Set_Mode 2") { state.mode = 2; }
                else if (command == "Motor ON") { state.motor = 1; }
                else if (command == "Motor OFF") { state.motor = 0; if (state.stream == 1) { state.stream = 2; } }
                else if (command == "Dsmode ON") { state.stream = 1; }
                else if (command == "Dsmode OFF") { state.stream = 0; }
                else { throw std::runtime_error("Unexpected command: " + command); }
            }
            if (afterSend) { afterSend(command); }
            reply = "ok";
            return true;
        };
        return ops;
    }
    bool Sent(const std::string& command) const
    { return std::find(commands.begin(), commands.end(), command) != commands.end(); }
};

static int count = 0;
static void Require(bool value, const char* message)
{ if (!value) { throw std::runtime_error(message); } }
template<class Fn> static void Case(const char* name, Fn fn)
{ fn(); ++count; std::cout << "PASS " << name << '\n'; }

int main()
{
    try
    {
        Case("eight complete orderings restore both baseline modes", []()
        {
            Require(Plans().size() == 8, "expected eight cases");
            for (int original : {1, 2}) for (const auto& plan : Plans())
            {
                Controller ctrl;
                ctrl.state.mode = original;
                bool restored = false;
                std::string trace;
                Require(Test(ctrl.Operations(), plan, restored, trace), "permissive controller rejected ordering");
                Require(restored && ctrl.state.mode == original && ctrl.state.motor == 0 && ctrl.state.stream == 0, "baseline not restored");
                const std::string first = plan.streamFirst ? "Dsmode ON" : "Motor ON";
                const std::string second = plan.streamFirst ? "Motor ON" : "Dsmode ON";
                Require(ctrl.commands[1] == first && ctrl.commands[2] == second, "startup order incorrect");
                Require(ctrl.commands[3] == (plan.powerOffFirst ? "Motor OFF" : "Dsmode OFF"), "shutdown order incorrect");
            }
        });
        Case("field e4 failures recover and allow remaining non-motion cases", []()
        {
            int passed = 0, attempted = 0;
            for (const auto& plan : Plans())
            {
                Controller ctrl;
                ctrl.reject = [&](const std::string& command, std::string& reply)
                {
                    if (ctrl.state.mode == 2 && (command == "Dsmode ON" || command == "Motor OFF"))
                    { reply = "e4:not allowed in current mode"; return true; }
                    return false;
                };
                bool restored = false;
                std::string trace;
                if (Test(ctrl.Operations(), plan, restored, trace)) { ++passed; }
                ++attempted;
                Require(restored, "e4 recovery should succeed");
                Require(ctrl.state.motor == 0 && ctrl.state.stream == 0 && ctrl.state.mode == 1, "e4 baseline differs");
            }
            Require(attempted == 8 && passed == 4, "batch acceptance mask incorrect");
        });
        Case("stream first refusal never enables motor", []()
        {
            Controller ctrl;
            ctrl.reject = [](const std::string& command, std::string& reply)
            { reply = "e9:motor not on"; return command == "Dsmode ON"; };
            bool restored = false; std::string trace;
            Require(!Test(ctrl.Operations(), Plans()[2], restored, trace), "rejected command passed");
            Require(restored && !ctrl.Sent("Motor ON"), "continued after stream failure");
        });
        Case("Motor OFF e4 fallback is bounded and restores original auto mode", []()
        {
            Controller ctrl; ctrl.state.mode = 2; ctrl.state.motor = 1;
            ctrl.reject = [&](const std::string& command, std::string& reply)
            { reply = "e4:not allowed in current mode"; return command == "Motor OFF" && ctrl.state.mode == 2; };
            std::string trace;
            Require(RestoreOff(ctrl.Operations(), 2, trace), "manual recovery failed");
            Require(ctrl.commands == std::vector<std::string>{"Motor OFF", "Set_Mode 1", "Motor OFF", "Set_Mode 2"}, "unexpected recovery sequence");
        });
        Case("non-mode Motor OFF error cannot switch mode", []()
        {
            Controller ctrl; ctrl.state.mode = 2; ctrl.state.motor = 1;
            ctrl.reject = [](const std::string&, std::string& reply) { reply = "e7:system fault"; return true; };
            std::string trace;
            Require(!RestoreOff(ctrl.Operations(), 2, trace), "failed motor off recovered falsely");
            Require(ctrl.commands.size() == 1, "mode fallback must only follow e4");
        });
        Case("unrestored stream stops batch after first case", []()
        {
            Controller ctrl;
            ctrl.reject = [](const std::string& command, std::string& reply)
            { reply = "e4:not allowed"; return command == "Dsmode OFF"; };
            int attempted = 0;
            for (const auto& plan : Plans())
            {
                bool restored = false; std::string trace;
                Test(ctrl.Operations(), plan, restored, trace); ++attempted;
                if (!restored) { break; }
            }
            Require(attempted == 1 && !ctrl.Sent("Set_Mode 2"), "continued after unsafe recovery");
        });
        Case("invalid unsafe or nonempty baseline sends no commands", []()
        {
            for (int invalid = 0; invalid < 9; ++invalid)
            {
                Controller ctrl;
                if (invalid == 0) ctrl.state.motor = 1;
                if (invalid == 1) ctrl.state.stream = 2;
                if (invalid == 2) ctrl.state.motion = 1;
                if (invalid == 3) ctrl.state.task = 1;
                if (invalid == 4) ctrl.state.estop = 1;
                if (invalid == 5) ctrl.state.fault = 1;
                if (invalid == 6) ctrl.state.mode = -1;
                if (invalid == 7) ctrl.state.motor = -1;
                if (invalid == 8) ctrl.readFails = true;
                bool restored = false; std::string trace;
                Require(!Test(ctrl.Operations(), Plans()[0], restored, trace), "unsafe baseline passed");
                Require(ctrl.commands.empty(), "unsafe baseline issued command");
            }
        });
        Case("cancel after motor ON only runs safe cleanup", []()
        {
            Controller ctrl;
            ctrl.afterSend = [&](const std::string& command) { if (command == "Motor ON") ctrl.cancelled = true; };
            bool restored = false; std::string trace;
            Require(!Test(ctrl.Operations(), Plans()[0], restored, trace), "cancelled test passed");
            Require(restored && ctrl.Sent("Motor OFF") && !ctrl.Sent("Dsmode ON"), "cancelled startup continued or cleanup missing");
        });
        Case("cancel before startup never energizes", []()
        {
            Controller ctrl; ctrl.cancelled = true;
            bool restored = false; std::string trace;
            Require(!Test(ctrl.Operations(), Plans()[0], restored, trace), "pre-cancelled test passed");
            Require(restored && ctrl.commands.empty(), "pre-cancelled issued command");
        });
        Case("ACK with newly raised fault is never success", []()
        {
            Controller ctrl;
            ctrl.afterSend = [&](const std::string& command) { if (command == "Motor ON") ctrl.state.fault = 1; };
            bool restored = false; std::string trace;
            Require(!Test(ctrl.Operations(), Plans()[0], restored, trace), "fault accepted");
            Require(!restored && !ctrl.Sent("Dsmode ON") && ctrl.state.motor == 0, "fault cleanup or continuation incorrect");
        });
        Case("unexpected motion blocks further commands", []()
        {
            Controller ctrl;
            ctrl.afterSend = [&](const std::string& command) { if (command == "Motor ON") ctrl.state.motion = 1; };
            bool restored = false; std::string trace;
            Require(!Test(ctrl.Operations(), Plans()[0], restored, trace), "unexpected motion accepted");
            Require(!restored && !ctrl.Sent("Dsmode ON"), "commands continued during motion");
        });
        Case("readback timeout is bounded and does not send next startup", []()
        {
            Controller ctrl; ctrl.ignoreCommands = true;
            bool restored = false; std::string trace;
            Require(!Test(ctrl.Operations(), Plans()[0], restored, trace), "unchanged readback passed");
            Require(restored && !ctrl.Sent("Dsmode ON") && ctrl.reads < 65, "timeout unbounded");
        });
        Case("disconnect after enabling prevents further commands", []()
        {
            Controller ctrl;
            ctrl.afterSend = [&](const std::string& command) { if (command == "Motor ON") ctrl.readFails = true; };
            bool restored = false; std::string trace;
            Require(!Test(ctrl.Operations(), Plans()[0], restored, trace), "disconnect accepted");
            Require(!restored && !ctrl.Sent("Dsmode ON"), "commands continued after disconnect");
        });
        Case("runtime stream-first preparation normalizes already-powered baseline", []()
        {
            Controller ctrl; ctrl.state.motor = 1; ctrl.state.mode = 2;
            std::string trace;
            Require(Prepare(ctrl.Operations(), Plans()[2], trace), "runtime prep failed");
            Require(ctrl.commands[0] == "Motor OFF" && ctrl.state.mode == 1 && ctrl.state.motor == 1 && ctrl.state.stream == 1, "runtime order not honored");
        });
        Case("production open switches once to accepted manual recipe and settles delayed servo drop", []()
        {
            Controller ctrl; ctrl.state.mode = 2; ctrl.state.motor = 1;
            int readsAfterModeSwitch = 0;
            ctrl.beforeRead = [&]()
            {
                if (ctrl.state.mode == 1 && !ctrl.Sent("Motor ON")
                    && ++readsAfterModeSwitch == 4) { ctrl.state.motor = 0; }
            };
            std::string trace;
            Require(OpenRuntimeStream(ctrl.Operations(), 1, trace), "production stream open failed");
            Require(ctrl.commands == std::vector<std::string>{"Set_Mode 1", "Motor ON", "Dsmode ON"},
                "production startup did not wait for mode then re-enable");
            Require(ctrl.state.mode == 1 && ctrl.state.motor == 1 && ctrl.state.stream == 1,
                "production manual stream state was not established");
        });
        Case("production open detects stream-induced motor drop and restores selected readiness", []()
        {
            Controller ctrl; ctrl.state.mode = 1; ctrl.state.motor = 1;
            int readsAfterOpen = 0;
            ctrl.afterSend = [&](const std::string& command)
            {
                if (command == "Dsmode ON")
                {
                    ctrl.state.motor = 0;
                    ctrl.state.stream = 2;
                    ++readsAfterOpen;
                }
            };
            std::string trace;
            Require(!OpenRuntimeStream(ctrl.Operations(), 1, trace), "delayed motor drop was accepted");
            Require(ctrl.commands == std::vector<std::string>{"Dsmode ON", "Dsmode OFF", "Motor ON"},
                "runtime recovery must close stream and re-enable without mode/power-off commands");
            Require(ctrl.state.mode == 1 && ctrl.state.motor == 1 && ctrl.state.stream == 0,
                "runtime readiness was not restored");
        });
        Case("production close only stops stream and keeps selected mode and servo resident", []()
        {
            Controller ctrl; ctrl.state.mode = 1; ctrl.state.motor = 1; ctrl.state.stream = 1;
            std::string trace;
            Require(CloseRuntimeStream(ctrl.Operations(), 1, trace), "production stream close failed");
            Require(ctrl.commands == std::vector<std::string>{"Dsmode OFF"}, "production close changed mode or power");
            Require(ctrl.state.mode == 1 && ctrl.state.motor == 1 && ctrl.state.stream == 0,
                "production close did not preserve readiness");
        });
        Case("production open rejects unsafe baseline without sending commands", []()
        {
            for (int invalid = 0; invalid < 2; ++invalid)
            {
                Controller ctrl; ctrl.state.mode = 2; ctrl.state.motor = 1;
                if (invalid == 0) { ctrl.state.motion = 1; }
                else { ctrl.state.stream = 2; }
                std::string trace;
                Require(!OpenRuntimeStream(ctrl.Operations(), 1, trace), "invalid production baseline passed");
                Require(ctrl.commands.empty(), "invalid production baseline issued a command");
            }
        });
        Case("subsequent production segment reuses resident manual power", []()
        {
            Controller ctrl; ctrl.state.mode = 1; ctrl.state.motor = 1;
            std::string trace;
            Require(OpenRuntimeStream(ctrl.Operations(), 1, trace), "resident stream open failed");
            Require(CloseRuntimeStream(ctrl.Operations(), 1, trace), "resident stream close failed");
            Require(ctrl.commands == std::vector<std::string>{"Dsmode ON", "Dsmode OFF"},
                "resident segment should not switch mode or power");
        });
        Case("failed ACK cannot be promoted by successful readback", []()
        {
            Controller ctrl;
            ctrl.reject = [&](const std::string& command, std::string& reply)
            { if (command != "Motor ON") return false; ctrl.state.motor = 1; reply = "ambiguous ACK"; return true; };
            bool restored = false; std::string trace;
            Require(!Test(ctrl.Operations(), Plans()[0], restored, trace), "bad ACK accepted");
            Require(restored && !ctrl.Sent("Dsmode ON") && ctrl.state.motor == 0, "bad ACK cleanup failed");
        });
        std::cout << "PASS: " << count << " mock mode-sequence test groups; no network or real robot\n";
        return 0;
    }
    catch (const std::exception& error)
    { std::cerr << "FAIL: " << error.what() << '\n'; return 1; }
}
