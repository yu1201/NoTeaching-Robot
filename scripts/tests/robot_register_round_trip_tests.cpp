#include "RobotRegisterRoundTrip.h"
#include <cmath>
#include <iostream>
#include <future>
#include <chrono>
#include <limits>
#include <set>
#include <stdexcept>
#include <vector>

static void Check(bool ok, const char* message)
{ if (!ok) { throw std::runtime_error(message); } }

struct Controller
{
    int value = 7;
    int calls = 0;
    std::set<int> failAt, throwAt, corruptReadAt;
    bool applyFailedWrite = false;
    std::string lastError;
    std::vector<char> order;
    bool Before(char kind)
    {
        ++calls;
        order.push_back(kind);
        lastError.clear();
        if (throwAt.count(calls)) { throw std::runtime_error("simulated transport exception"); }
        if (failAt.count(calls))
        { lastError = "error-at-" + std::to_string(calls); return false; }
        return true;
    }
    bool Read(int& output)
    {
        if (!Before('R')) { return false; }
        output = corruptReadAt.count(calls) ? value + 100 : value;
        return true;
    }
    bool Write(int next)
    {
        const bool ok = Before('W');
        if (ok || applyFailedWrite) { value = next; }
        return ok;
    }
    RobotRegisterRoundTrip::Result<int> Run()
    {
        return RobotRegisterRoundTrip::Run<int>(
            [this](int& output) { return Read(output); },
            [this](int next) { return Write(next); },
            [this] { return lastError; },
            [](int original) { return original + 1; },
            [](int actual, int expected) { return actual == expected; });
    }
};

int main()
{
    try
    {
        Controller success;
        const auto passed = success.Run();
        Check(passed.Passed() && success.value == 7, "normal round trip did not restore original");
        Check(success.order == std::vector<char>{'R', 'W', 'R', 'W', 'R'}, "wrong callback sequence");

        Controller noOriginal;
        noOriginal.failAt = {1};
        const auto notStarted = noOriginal.Run();
        Check(!notStarted.Passed() && noOriginal.calls == 1, "unknown original was overwritten");
        Check(std::string(RobotRegisterRoundTrip::StatusText(notStarted.write)) == "未执行",
            "unattempted write status is wrong");

        for (int fail : {2, 3, 4, 5})
        {
            Controller broken;
            broken.failAt = {fail};
            const auto result = broken.Run();
            Check(!result.Passed() && broken.calls == 5, "failure lost restoration/readback");
            const RobotRegisterRoundTrip::Step* steps[] = {
                &result.initialRead, &result.write, &result.read, &result.restore, &result.restoreRead};
            Check(steps[fail - 1]->error == "error-at-" + std::to_string(fail), "later operation erased the first error");
            Check(std::string(RobotRegisterRoundTrip::StatusText(*steps[fail - 1])) == "FAIL",
                "attempted failure must not be displayed as not executed");
            if (fail != 4) { Check(broken.value == 7, "recoverable failure did not restore"); }
        }

        Controller denied;
        denied.failAt = {2, 4};
        const auto fieldFailure = denied.Run();
        Check(!fieldFailure.Passed() && fieldFailure.write.error == "error-at-2"
            && fieldFailure.restore.error == "error-at-4", "field e24 failures were not retained separately");
        Check(fieldFailure.originalMatches && !fieldFailure.restore.ok,
            "restored readback must not turn a failed restore command into success");

        for (int fail : {2, 4})
        {
            Controller partial;
            partial.failAt = {fail};
            partial.applyFailedWrite = true;
            const auto result = partial.Run();
            Check(!result.Passed() && partial.value == 7 && result.restoreRead.ok,
                "uncertain write ACK must restore but cannot pass");
        }
        for (int corrupt : {3, 5})
        {
            Controller mismatch;
            mismatch.corruptReadAt = {corrupt};
            const auto result = mismatch.Run();
            Check(!result.Passed(), "mismatched readback accepted");
            Check(mismatch.calls == 5, "mismatch skipped restoration");
        }
        for (int throws : {1, 2, 3, 4, 5})
        {
            Controller broken;
            broken.throwAt = {throws};
            const auto result = broken.Run();
            Check(!result.Passed(), "exception accepted");
            Check(broken.calls == (throws == 1 ? 1 : 5), "exception prevented safe restoration attempts");
        }

        double value = 0.0;
        const auto real = RobotRegisterRoundTrip::Run<double>(
            [&](double& output) { output = value; return true; },
            [&](double next) { value = next; return true; },
            [] { return std::string(); }, [](double original) { return original + 0.125; },
            [](double actual, double expected) { return std::abs(actual - expected) <= 1e-6; });
        Check(real.Passed() && value == 0.0, "REAL round trip failed");
        auto missingRead = real;
        missingRead.restoreRead.ok = false;
        Check(!missingRead.Passed(), "missing final readback was accepted");
        // Actual acceptance path: persist -> write -> wait for physical inspection
        // -> confirmation -> independent readback -> restore -> restore readback.
        for (int scenario = 0; scenario < 6; ++scenario)
        {
            Controller manual;
            bool prompted = false;
            const auto result = RobotRegisterRoundTrip::Run<int>(
                [&](int& output) { return manual.Read(output); },
                [&](int next) { return manual.Write(next); },
                [] { return std::string("manual/checkpoint rejected"); },
                [](int original) { return original + 1; },
                [](int actual, int expected) { return actual == expected; },
                [&](int original, int temporary) {
                    Check(manual.calls == 1 && manual.value == original && temporary == 8,
                        "backup did not precede write");
                    if (scenario == 4) { throw std::runtime_error("database exception"); }
                    return scenario != 1;
                },
                [&](int original, int temporary) {
                    prompted = true;
                    Check(manual.calls == 2 && manual.value == temporary && original == 7,
                        "test value was read/restored before operator confirmation");
                    if (scenario == 3) { throw std::runtime_error("dialog destroyed"); }
                    if (scenario == 5) { manual.value = 100; } // On-site mismatch after acknowledgement.
                    return scenario != 2;
                });
            Check(manual.value == 7, "manual cancel/failure did not restore");
            Check(result.Passed() == (scenario == 0), "manual failure was promoted to pass");
            Check(prompted == (scenario != 1 && scenario != 4), "prompted before durable backup");
            if (scenario == 2 || scenario == 3)
            { Check(!result.read.attempted && result.restore.ok && result.restoreRead.ok,
                "cancel must skip test validation and restore"); }
            if (scenario == 1 || scenario == 4)
            { Check(manual.calls == 1 && !result.write.attempted, "failed durable backup allowed a write"); }
            if (scenario == 5)
            { Check(!result.temporaryMatches && result.originalMatches, "post-confirmation mismatch was ignored"); }
        }
        Controller waiting;
        std::promise<void> atPrompt, humanConfirmed;
        auto atPromptFuture = atPrompt.get_future();
        auto humanFuture = humanConfirmed.get_future();
        auto worker = std::async(std::launch::async, [&] {
            return RobotRegisterRoundTrip::Run<int>(
                [&](int& output) { return waiting.Read(output); },
                [&](int next) { return waiting.Write(next); },
                [] { return std::string(); }, [](int original) { return original + 1; },
                [](int actual, int expected) { return actual == expected; },
                [](int, int) { return true; },
                [&](int, int) { atPrompt.set_value(); humanFuture.wait(); return true; });
        });
        atPromptFuture.wait();
        const bool staysWritten = waiting.calls == 2 && waiting.value == 8
            && worker.wait_for(std::chrono::milliseconds(20)) == std::future_status::timeout;
        humanConfirmed.set_value();
        const auto waited = worker.get();
        Check(staysWritten && waited.Passed() && waiting.value == 7,
            "waiting for operator did not retain value or resume readback/restore");
        std::cout << "PASS: register backup, manual wait/confirmation/cancel, write/read/restore sequence, independent errors, rejected and uncertain ACKs, mismatches, exceptions, status labels and REAL values (offline only)\n";
        return 0;
    }
    catch (const std::exception& error)
    { std::cerr << "FAIL: " << error.what() << '\n'; return 1; }
}
