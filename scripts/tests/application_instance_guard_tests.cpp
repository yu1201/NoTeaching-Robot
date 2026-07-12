#include "ApplicationInstanceGuard.h"

#include <QCoreApplication>
#include <QProcess>
#include <QStringList>
#include <QUuid>

#include <cstdlib>
#include <iostream>

#ifdef Q_OS_WIN
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#endif

namespace
{
constexpr int kLockRejectedExitCode = 23;
constexpr int kMachineLockAccessDeniedExitCode = 24;

void Check(bool condition, const char* message)
{
    if (!condition)
    {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(1);
    }
}

int RunChild(const QString& scope)
{
    QString reason;
    auto guard = ApplicationInstanceGuard::TryAcquire(scope, &reason);
    if (guard)
    {
        return 0;
    }
    return reason.contains(QStringLiteral("无法访问机器级机器人控制锁"))
        ? kMachineLockAccessDeniedExitCode
        : kLockRejectedExitCode;
}

int RunProbe(const QString& executable, const QString& scope)
{
    QProcess child;
    child.start(executable, QStringList() << QStringLiteral("--probe") << scope);
    if (!child.waitForStarted(5000) || !child.waitForFinished(10000))
    {
        return -1;
    }
    return child.exitStatus() == QProcess::NormalExit ? child.exitCode() : -2;
}

#ifdef Q_OS_WIN
int RunNativeMutexOwner(const QString& scope, bool denyReopen)
{
    SECURITY_DESCRIPTOR descriptor {};
    ACL emptyAcl {};
    SECURITY_ATTRIBUTES attributes {};
    SECURITY_ATTRIBUTES* attributesPointer = nullptr;
    if (denyReopen)
    {
        if (!InitializeSecurityDescriptor(&descriptor, SECURITY_DESCRIPTOR_REVISION)
            || !InitializeAcl(&emptyAcl, sizeof(emptyAcl), ACL_REVISION)
            || !SetSecurityDescriptorDacl(&descriptor, TRUE, &emptyAcl, FALSE))
        {
            return 31;
        }
        attributes.nLength = sizeof(attributes);
        attributes.lpSecurityDescriptor = &descriptor;
        attributes.bInheritHandle = FALSE;
        attributesPointer = &attributes;
    }

    const QString name = ApplicationInstanceGuard::MachineMutexName(scope);
    HANDLE mutexHandle = CreateMutexW(
        attributesPointer,
        FALSE,
        reinterpret_cast<LPCWSTR>(name.utf16()));
    if (mutexHandle == nullptr || GetLastError() == ERROR_ALREADY_EXISTS)
    {
        if (mutexHandle != nullptr)
        {
            CloseHandle(mutexHandle);
        }
        return 32;
    }
    std::cout << "READY\n" << std::flush;
    std::cin.get();
    CloseHandle(mutexHandle);
    return 0;
}

void CheckNativeMutexBlocksAcrossProcesses(
    const QString& executable,
    const QString& scope,
    bool denyReopen,
    int expectedProbeExitCode)
{
    QProcess owner;
    QStringList arguments {
        QStringLiteral("--native-owner"),
        scope,
        denyReopen ? QStringLiteral("deny") : QStringLiteral("default")
    };
    owner.start(executable, arguments);
    Check(owner.waitForStarted(5000),
        "native machine-mutex owner process did not start");
    QByteArray output;
    while (!output.contains("READY") && owner.state() != QProcess::NotRunning)
    {
        if (!owner.waitForReadyRead(5000))
        {
            break;
        }
        output += owner.readAllStandardOutput();
    }
    output += owner.readAllStandardOutput();
    Check(output.contains("READY"),
        "native machine-mutex owner did not publish readiness");
    Check(RunProbe(executable, scope) == expectedProbeExitCode,
        denyReopen
            ? "machine mutex access-denied path did not fail closed"
            : "raw Global machine mutex did not block another process");

    owner.write("x");
    owner.closeWriteChannel();
    Check(owner.waitForFinished(5000) && owner.exitCode() == 0,
        "native machine-mutex owner did not release cleanly");
    Check(RunProbe(executable, scope) == 0,
        "machine mutex name remained blocked after the only native handle closed");
}
#endif
}

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);
    const QStringList args = app.arguments();
    if (args.size() == 3 && args.at(1) == QStringLiteral("--probe"))
    {
        return RunChild(args.at(2));
    }
#ifdef Q_OS_WIN
    if (args.size() == 4 && args.at(1) == QStringLiteral("--native-owner"))
    {
        return RunNativeMutexOwner(args.at(2), args.at(3) == QStringLiteral("deny"));
    }
#endif

    QString invalidReason;
    Check(!ApplicationInstanceGuard::TryAcquire(QString(), &invalidReason)
            && !invalidReason.isEmpty(),
        "empty guard scope was not rejected fail-closed");

    const QString scope = QStringLiteral("ApplicationInstanceGuardTests/%1")
        .arg(QUuid::createUuid().toString(QUuid::WithoutBraces));
    QString reason;
    auto owner = ApplicationInstanceGuard::TryAcquire(scope, &reason);
    Check(static_cast<bool>(owner), "parent could not acquire a fresh process guard");

    const QString executable = QCoreApplication::applicationFilePath();
    Check(RunProbe(executable, scope) == kLockRejectedExitCode,
        "second process acquired the same guard while the owner was alive");

    owner.reset();
    Check(RunProbe(executable, scope) == 0,
        "guard was not released after the owner exited its critical section");

#ifdef Q_OS_WIN
    const QString globalScope = QStringLiteral("ApplicationInstanceGuardTests/Global/%1")
        .arg(QUuid::createUuid().toString(QUuid::WithoutBraces));
    CheckNativeMutexBlocksAcrossProcesses(
        executable, globalScope, false, kLockRejectedExitCode);
    const QString deniedScope = QStringLiteral("ApplicationInstanceGuardTests/Denied/%1")
        .arg(QUuid::createUuid().toString(QUuid::WithoutBraces));
    CheckNativeMutexBlocksAcrossProcesses(
        executable, deniedScope, true, kMachineLockAccessDeniedExitCode);
#endif

    std::cout << "PASS: per-user file lock plus Global Windows mutex reject live, cross-process, and access-denied contenders\n";
    return 0;
}
