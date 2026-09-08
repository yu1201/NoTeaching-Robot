#pragma once

#include <string>

// Brand-neutral acceptance sequencing. Callbacks use only RobotDriverAdaptor;
// controller-specific permission acquisition belongs in the driver's setters.
namespace RobotRegisterRoundTrip
{
struct Step
{
    bool attempted = false;
    bool ok = false;
    std::string error;
};

inline const char* StatusText(const Step& step)
{ return !step.attempted ? "未执行" : step.ok ? "OK" : "FAIL"; }

template<class T> struct Result
{
    T original{}, temporary{}, readback{}, restored{};
    Step initialRead, backupSaved, write, confirmation, read, restore, restoreRead;
    bool temporaryMatches = false;
    bool originalMatches = false;
    bool Passed() const
    {
        return initialRead.ok && backupSaved.ok && write.ok && confirmation.ok && read.ok && temporaryMatches
            && restore.ok && restoreRead.ok && originalMatches;
    }
};

template<class Operation, class ErrorReader>
void Execute(Step& step, Operation operation, ErrorReader errorReader)
{
    step.attempted = true;
    try
    {
        step.ok = operation();
        if (!step.ok)
        {
            step.error = errorReader();
            if (step.error.empty()) { step.error = "接口返回失败，未提供详细错误。"; }
        }
    }
    catch (...)
    {
        step.ok = false;
        step.error = "接口调用发生异常，结果未确认。";
    }
}

struct ContinueImmediately
{
    template<class T> bool operator()(T, T) const { return true; }
};

template<class T, class Reader, class Writer, class ErrorReader, class Temporary, class Equal,
    class Backup = ContinueImmediately, class Confirm = ContinueImmediately>
Result<T> Run(Reader read, Writer write, ErrorReader errorReader, Temporary temporary, Equal equal,
    Backup backup = {}, Confirm confirm = {})
{
    Result<T> result;
    Execute(result.initialRead, [&] { return read(result.original); }, errorReader);
    if (!result.initialRead.ok) { return result; } // Never overwrite an unknown original.
    result.temporary = temporary(result.original);
    Execute(result.backupSaved, [&] { return backup(result.original, result.temporary); }, errorReader);
    if (!result.backupSaved.ok) { return result; } // Persist originals before any mutation.
    Execute(result.write, [&] { return write(result.temporary); }, errorReader);
    if (result.write.ok)
    { Execute(result.confirmation, [&] { return confirm(result.original, result.temporary); }, errorReader); }
    // A rejected/timed-out setter may still have changed the register. Record
    // readback and restore even on failure; never promote a failed ACK to success.
    // Successful writes stay in place until the operator confirms. Cancellation
    // skips test-value validation, but always attempts restoration below.
    if (!result.write.ok || result.confirmation.ok)
    { Execute(result.read, [&] { return read(result.readback); }, errorReader); }
    result.temporaryMatches = result.read.ok && equal(result.readback, result.temporary);
    Execute(result.restore, [&] { return write(result.original); }, errorReader);
    Execute(result.restoreRead, [&] { return read(result.restored); }, errorReader);
    result.originalMatches = result.restoreRead.ok && equal(result.restored, result.original);
    return result;
}
}
