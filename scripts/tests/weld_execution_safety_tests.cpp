#include "WeldExecutionSafety.h"

#include <cstdlib>
#include <iostream>

namespace
{
void Check(bool condition, const char* message)
{
    if (!condition)
    {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(1);
    }
}
}

int main()
{
    using State = WeldExecutionTerminalState;

    Check(!WeldExecutionSafety::ShouldAttemptMandatoryRetreat(false, false),
        "post-weld Cancel was allowed to start a new retreat motion");
    Check(WeldExecutionSafety::ResolveTerminalState(true, false, false)
            == State::ProgramCompletedUnretracted,
        "post-weld Cancel released a completed/unretracted weld");

    Check(!WeldExecutionSafety::ShouldAttemptMandatoryRetreat(true, true),
        "latched STOP was bypassed for automatic retreat");
    Check(WeldExecutionSafety::ResolveTerminalState(true, false, false)
            == State::ProgramCompletedUnretracted,
        "latched STOP released a completed/unretracted weld");

    Check(WeldExecutionSafety::ShouldAttemptMandatoryRetreat(true, false),
        "confirmed retreat without STOP was not admitted");
    Check(WeldExecutionSafety::ResolveTerminalState(true, true, false)
            == State::ProgramCompletedUnretracted,
        "failed/unverified retreat was reported as safe completion");
    Check(WeldExecutionSafety::ResolveTerminalState(true, true, true)
            == State::SafelyRetracted,
        "verified retreat did not reach safe terminal state");
    Check(WeldExecutionSafety::ResolveTerminalState(false, false, false)
            == State::Incomplete,
        "incomplete weld was misclassified");

    std::cout << "PASS: Cancel, STOP, retreat failure and verified retreat terminal policy\n";
    return 0;
}
