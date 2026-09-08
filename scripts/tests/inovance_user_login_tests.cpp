#include "InovanceUserLogin.h"
#include <iostream>
#include <stdexcept>
#include <vector>

using namespace InovanceUserLogin;

static void Check(bool ok, const char* message)
{ if (!ok) { throw std::runtime_error(message); } }

struct Controller
{
    int actualLevel = 2;
    bool sendOk = true, queryOk = true;
    std::string response = "ok";
    std::vector<std::string> commands;
    Ops Operations()
    {
        return {
            [this](const std::string& command, std::string& reply)
            { commands.push_back(command); reply = response; return sendOk; },
            [this](const std::string& command, int& value)
            { commands.push_back(command); value = actualLevel; return queryOk; }
        };
    }
};

int main()
{
    try
    {
        std::string error;
        const auto defaults = Resolve(0, "");
        Check(defaults.level == 2 && defaults.password == kDefaultPassword,
            "legacy level 0 / missing password must use administrator defaults");
        Check(defaults.password == "000000" && Credentials{}.password == "000000",
            "API default must match the field-verified credential, not the pendant credential");
        for (int level : {1, 2, 3})
        {
            const auto configured = Resolve(level, "custom1");
            Controller controller;
            controller.actualLevel = level;
            Check(configured.level == level && configured.password == "custom1", "override lost");
            Check(Login(configured, controller.Operations(), error), "configured login failed");
            Check(controller.commands.size() == 2
                && controller.commands[0] == "UserLogin " + std::to_string(level) + " custom1"
                && controller.commands[1] == "CurUserType", "unexpected login command sequence");
        }
        Check(Resolve(2, "").password == kDefaultPassword, "empty password fallback lost");
        Check(Resolve(0, "custom1").level == 2, "password override must retain default level");
        Check(Resolve(2, "666666").password == "666666",
            "explicit credentials for other controllers must not be silently rewritten");

        Controller success;
        Check(Login(defaults, success.Operations(), error), "default administrator login failed");
        Check(success.commands.size() == 2 && success.commands[0] == "UserLogin 2 000000"
            && success.commands[1] == "CurUserType", "field-verified default command sequence lost");
        Check(error.empty(), "successful login must clear old error");
        Check(Login(defaults, success.Operations(), error) && success.commands.size() == 4,
            "a new session must authenticate again, not reuse an earlier proof");

        for (int actual : {-1, 0, 1, 3, 99, 666666})
        {
            Controller wrongLevel;
            wrongLevel.actualLevel = actual;
            Check(!Login(defaults, wrongLevel.Operations(), error), "mismatched level accepted");
            Check(error.find(defaults.password) == std::string::npos, "password leaked in level error");
        }
        Controller noReadback;
        noReadback.queryOk = false;
        Check(!Login(defaults, noReadback.Operations(), error), "missing readback accepted");

        for (bool sendOk : {false, true})
        {
            Controller rejected;
            rejected.sendOk = sendOk;
            rejected.response = "e27: echoed credential=" + defaults.password;
            Check(!Login(defaults, rejected.Operations(), error), "rejected login accepted");
            Check(rejected.commands.size() == 1, "readback must not promote failed login ACK");
            Check(error.find(defaults.password) == std::string::npos, "rejection echoed credentials");
            Check(error.find("e27") != std::string::npos, "rejection lost its safe error code");
        }
        for (const char* code : {"e1", "e2", "e3", "e24", "e25", "e27", "e99"})
        {
            Controller rejected;
            rejected.sendOk = false;
            rejected.response = std::string(code) + ": UserLogin 2 " + defaults.password;
            Check(!Login(defaults, rejected.Operations(), error), "controller error accepted");
            Check(error.find(code) != std::string::npos, "safe protocol code lost");
            Check(error.find(defaults.password) == std::string::npos, "controller echo leaked");
            if (std::string(code) == "e3")
            { Check(error.find("不能直接判定密码错误") != std::string::npos, "e3 was misdiagnosed as a wrong password"); }
        }
        for (const char* reply : {"e666666: secret", "e000000: secret", "e03", "e3password", "E",
            "UserLogin 2 666666", "UserLogin 2 000000", "e0"})
        {
            Check(SafeErrorCode(reply).empty(), "unsafe error code accepted");
            Check(RejectionDetail(reply).find(defaults.password) == std::string::npos, "unsafe error code leaked");
        }
        Check(SafeErrorCode("E3: detail") == "e3", "uppercase error code not normalized");
        Controller noAck;
        noAck.sendOk = false;
        noAck.response.clear();
        Check(!Login(defaults, noAck.Operations(), error), "missing ACK accepted");
        Check(error.find("登录结果未知") != std::string::npos, "uncertain login not distinguished");

        RetryGate retry;
        Check(retry.Begin(false, error), "initial monitor connect blocked");
        retry.Block("UserLogin e3");
        const auto originalFailure = retry.Error();
        for (int tick = 0; tick < 100; ++tick)
        {
            error = "unrelated monitor failure";
            Check(!retry.Begin(false, error), "monitor retried rejected login");
            Check(error == originalFailure, "monitor overwrote original login evidence");
        }
        Check(retry.Begin(true, error) && retry.Error().empty(), "explicit retry must clear latch");
        retry.Block("CurUserType readback uncertain");
        Check(!retry.Begin(false, error), "uncertain authentication retried automatically");
        Check(retry.Begin(true, error), "explicit retry after readback failure blocked");
        for (const char* reply : {"", "OK", "=2", "ok unexpected"})
        {
            Controller malformed;
            malformed.response = reply;
            Check(!Login(defaults, malformed.Operations(), error), "ambiguous ACK accepted");
            Check(malformed.commands.size() == 1, "ambiguous ACK continued");
        }
        for (int level : {-1, 4, 99})
        {
            Controller invalid;
            Check(!Login(Resolve(level, "custom1"), invalid.Operations(), error), "invalid level accepted");
            Check(invalid.commands.empty(), "invalid level sent a command");
        }
        for (const std::string& password : std::vector<std::string>{"", "123456789", "a b", "a\nb",
            "a\tb", "a@@b", "a$$b", std::string("a\0b", 3)})
        {
            Controller invalid;
            Check(!Login({2, password}, invalid.Operations(), error), "invalid password accepted");
            Check(invalid.commands.empty(), "invalid password sent a command");
        }
        std::cout << "PASS: Inovance default/override login, exact permission readback, safe e-codes, redaction, uncertainty, 100 blocked monitor retries, explicit retry and input validation (offline only)\n";
        return 0;
    }
    catch (const std::exception& error)
    { std::cerr << "FAIL: " << error.what() << '\n'; return 1; }
}
