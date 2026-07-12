#include "RobotRecoverySafetyPolicy.h"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <thread>

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
    using namespace RobotRecoverySafetyPolicy;

    Check(ResolveStepPersistedProgramAction(
        QStringLiteral("Weld_A"), QString(), ObservedProgramState::Stopped, true)
            == PersistedProgramAction::AlreadyStopped,
        "empty program plus stable stop was not idempotent");
    Check(ResolveStepPersistedProgramAction(
        QStringLiteral("Weld_A"), QString(), ObservedProgramState::Unknown, false)
            == PersistedProgramAction::Reject,
        "empty program without stable stop was accepted");
    Check(ResolveStepPersistedProgramAction(
        QStringLiteral("Weld_A"), QStringLiteral("Weld_A"), ObservedProgramState::Running, false)
            == PersistedProgramAction::StopThenKill,
        "same running persisted program did not require STOP+Kill");
    Check(ResolveStepPersistedProgramAction(
        QStringLiteral("Weld_A"), QStringLiteral("Weld_A"), ObservedProgramState::Paused, false)
            == PersistedProgramAction::KillLoaded,
        "same paused persisted program did not require Kill");
    Check(ResolveStepPersistedProgramAction(
        QStringLiteral("Weld_A"), QStringLiteral("OperatorProgram"), ObservedProgramState::Running, false)
            == PersistedProgramAction::Reject,
        "foreign controller program was eligible for STOP/Kill");

    bool moveIssued = false;
    if (MayMoveForSafeRetreat(QStringLiteral("interrupted"), false, false))
    {
        moveIssued = true;
    }
    Check(!moveIssued, "interrupted recovery moved without a verified stop");
    Check(!MayMoveForSafeRetreat(QStringLiteral("interrupted"), false, false),
        "stop failure did not block the first recovery Move");
    Check(MayMoveForSafeRetreat(QStringLiteral("interrupted"), false, true),
        "verified interrupted termination did not admit retreat");
    Check(MayMoveForSafeRetreat(QStringLiteral("unretracted"), true, false),
        "completed/unretracted record could not directly retreat");

    PausedResumeIdentity expected;
    expected.state = QStringLiteral("paused");
    expected.checkpointId = QStringLiteral("checkpoint-1");
    expected.robotName = QStringLiteral("RobotA");
    expected.robotEndpoint = QStringLiteral("tcp:[192.168.1.10]:8193");
    expected.programName = QStringLiteral("Weld_A");
    expected.trajectoryRelativePath = QStringLiteral("Result/RobotA/Case/LaserPoint/Final.txt");
    expected.trajectorySha256 = QString(64, QLatin1Char('a'));
    expected.trajectorySize = 1234;
    PausedResumeIdentity current = expected;
    Check(SamePausedResumeIdentity(expected, current),
        "exact paused binding was not admitted by dedicated policy");
    current.checkpointId = QStringLiteral("checkpoint-2");
    Check(!SamePausedResumeIdentity(expected, current),
        "paused checkpoint TOCTOU was accepted");
    current = expected;
    current.robotEndpoint = QStringLiteral("tcp:[192.168.1.11]:8193");
    Check(!SamePausedResumeIdentity(expected, current),
        "paused endpoint TOCTOU was accepted");
    current = expected;
    current.trajectorySha256 = QString(64, QLatin1Char('b'));
    Check(!SamePausedResumeIdentity(expected, current),
        "paused trajectory TOCTOU was accepted");

    const ProgramContentIdentity original = ContentIdentity(QByteArrayLiteral("ABCD"));
    const ProgramContentIdentity same = ContentIdentity(QByteArrayLiteral("ABCD"));
    const ProgramContentIdentity sameSizeSwap = ContentIdentity(QByteArrayLiteral("WXYZ"));
    Check(SameProgramContent(original, same), "identical STEP content identity did not match");
    Check(original.size == sameSizeSwap.size && !SameProgramContent(original, sameSizeSwap),
        "same-size remote SRP/SRD swap bypassed SHA256 identity");
    ProgramContentIdentity oversized = original;
    oversized.size = 128LL * 1024LL * 1024LL + 1;
    Check(!ProgramContentWithinLimit(oversized, 128LL * 1024LL * 1024LL),
        "oversized remote declaration was admitted before body download");

    ProgramContentWitnessSnapshot liveWitness;
    liveWitness.generation = 1;
    liveWitness.projectName = "Project";
    liveWitness.programName = "Weld_A";
    liveWitness.remoteProgramPath = "/Project/Weld_A.srp";
    liveWitness.remoteDataPath = "/Project/Weld_A.srd";
    liveWitness.programIdentity = original;
    liveWitness.dataIdentity = ContentIdentity(QByteArrayLiteral("DATA"));
    liveWitness.observedProjectName = liveWitness.projectName;
    liveWitness.observedProgramName = liveWitness.programName;
    liveWitness.observedProgramState = 1;
    Check(SameProgramContentWitness(liveWitness, liveWitness),
        "exact content witness snapshot did not match itself");
    ProgramContentWitnessSnapshot replacedWitness = liveWitness;
    ++replacedWitness.generation;
    Check(!SameProgramContentWitness(liveWitness, replacedWitness),
        "witness generation replacement during remote download was accepted");

    // Model a WinINet call that remains blocked even after cancellation. The
    // verifier must have released the SDK mutex, so red STOP can set the
    // independent token, acquire the SDK mutex and complete stop immediately.
    std::recursive_mutex sdkMutex;
    RemoteContentVerificationGate verificationGate;
    RemoteContentVerificationGate::Token verificationToken;
    std::mutex ftpMutex;
    std::condition_variable ftpCondition;
    bool ftpEntered = false;
    bool releaseFtp = false;
    std::atomic<bool> stopCompleted{ false };
    std::atomic<bool> verifierRejected{ false };
    std::thread blockedVerifier([&]()
        {
            std::unique_lock<std::recursive_mutex> sdkLock(sdkMutex);
            const ProgramContentWitnessSnapshot frozen = liveWitness;
            verificationToken = verificationGate.TryBegin();
            Check(verificationToken != nullptr,
                "post-start remote verification token was not acquired");
            sdkLock.unlock();
            {
                std::unique_lock<std::mutex> ftpLock(ftpMutex);
                ftpEntered = true;
                ftpCondition.notify_all();
                ftpCondition.wait(ftpLock, [&]() { return releaseFtp; });
            }
            sdkLock.lock();
            verificationGate.End(verificationToken);
            verifierRejected.store(
                verificationToken->load(std::memory_order_acquire)
                    || !SameProgramContentWitness(frozen, liveWitness),
                std::memory_order_release);
        });
    {
        std::unique_lock<std::mutex> ftpLock(ftpMutex);
        Check(ftpCondition.wait_for(
            ftpLock, std::chrono::seconds(2), [&]() { return ftpEntered; }),
            "post-start fake FTP did not enter its blocking read");
    }
    const auto abortStarted = std::chrono::steady_clock::now();
    std::thread aborter([&]()
        {
            verificationGate.CancelActive();
            const std::lock_guard<std::recursive_mutex> sdkLock(sdkMutex);
            ++liveWitness.generation; // Abort clears/replaces the bound witness.
            stopCompleted.store(true, std::memory_order_release);
        });
    aborter.join();
    const auto abortElapsed = std::chrono::steady_clock::now() - abortStarted;
    Check(stopCompleted.load(std::memory_order_acquire),
        "red STOP did not complete while post-start FTP remained blocked");
    Check(abortElapsed < std::chrono::milliseconds(500),
        "red STOP waited on the post-start FTP verifier SDK lock");
    Check(verificationToken != nullptr
            && verificationToken->load(std::memory_order_acquire),
        "red STOP did not latch the independent FTP cancellation token");
    {
        const std::lock_guard<std::mutex> ftpLock(ftpMutex);
        releaseFtp = true;
    }
    ftpCondition.notify_all();
    blockedVerifier.join();
    Check(verifierRejected.load(std::memory_order_acquire),
        "witness replacement/cancellation during unlocked FTP was accepted");

    std::cout << "PASS: interrupted, paused and STEP remote-content recovery/concurrency policies\n";
    return 0;
}
