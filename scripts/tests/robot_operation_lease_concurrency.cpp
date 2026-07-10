#include "RobotOperationLease.h"

#include <QString>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>
#include <utility>
#include <vector>

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

bool WaitUntil(const std::atomic<int>& value, int target)
{
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (value.load(std::memory_order_acquire) != target
        && std::chrono::steady_clock::now() < deadline)
    {
        std::this_thread::yield();
    }
    return value.load(std::memory_order_acquire) == target;
}
}

int main()
{
    RobotDriverAdaptor firstDriverStorage;
    firstDriverStorage.m_sSocketIP = "192.168.10.21";
    firstDriverStorage.m_nSocketPort = 8193;
    RobotDriverAdaptor secondDriverStorage;
    secondDriverStorage.m_sSocketIP = "192.168.10.22";
    secondDriverStorage.m_nSocketPort = 8193;
    const auto* firstDriver = &firstDriverStorage;
    const auto* secondDriver = &secondDriverStorage;

    constexpr int contenderCount = 32;
    std::atomic<int> ready{ 0 };
    std::atomic<int> attempted{ 0 };
    std::atomic<int> winners{ 0 };
    std::atomic<bool> start{ false };
    std::atomic<bool> releaseWinner{ false };
    std::vector<std::thread> contenders;
    contenders.reserve(contenderCount);
    for (int index = 0; index < contenderCount; ++index)
    {
        contenders.emplace_back([&, index]()
            {
                ready.fetch_add(1, std::memory_order_release);
                while (!start.load(std::memory_order_acquire))
                {
                    std::this_thread::yield();
                }
                auto lease = RobotOperationLease::TryAcquire(
                    firstDriver,
                    QStringLiteral("same-driver-%1").arg(index));
                if (lease)
                {
                    winners.fetch_add(1, std::memory_order_release);
                }
                attempted.fetch_add(1, std::memory_order_release);
                if (lease)
                {
                    while (!releaseWinner.load(std::memory_order_acquire))
                    {
                        std::this_thread::yield();
                    }
                }
            });
    }
    Check(WaitUntil(ready, contenderCount), "contender threads did not become ready");
    start.store(true, std::memory_order_release);
    Check(WaitUntil(attempted, contenderCount), "same-driver TryAcquire calls did not finish");
    Check(winners.load(std::memory_order_acquire) == 1,
        "same driver admitted zero or multiple concurrent leases");
    Check(RobotOperationLease::AnyActive(), "winning same-driver lease is not registered");
    Check(!RobotOperationLease::CurrentOwner(firstDriver).isEmpty(),
        "CurrentOwner did not report the winning lease");
    releaseWinner.store(true, std::memory_order_release);
    for (std::thread& contender : contenders)
    {
        contender.join();
    }
    Check(!RobotOperationLease::AnyActive(), "same-driver lease did not unregister on destruction");

    RobotDriverAdaptor endpointAliasA;
    endpointAliasA.m_sSocketIP = " 192.168.10.30 ";
    endpointAliasA.m_nSocketPort = 8193;
    RobotDriverAdaptor endpointAliasB;
    endpointAliasB.m_sSocketIP = "192.168.10.30";
    endpointAliasB.m_nSocketPort = 8193;
    auto endpointLease = RobotOperationLease::TryAcquire(
        &endpointAliasA, QStringLiteral("same-endpoint-owner"));
    Check(static_cast<bool>(endpointLease), "same-endpoint setup acquire failed");
    QString aliasReason;
    Check(!RobotOperationLease::TryAcquire(
        &endpointAliasB, QStringLiteral("same-endpoint-contender"), &aliasReason),
        "different driver pointers for one TCP endpoint acquired concurrently");
    Check(aliasReason.contains(QStringLiteral("same-endpoint-owner")),
        "same-endpoint rejection did not identify the current owner");
    Check(RobotOperationLease::CurrentOwner(&endpointAliasB) == QStringLiteral("same-endpoint-owner"),
        "CurrentOwner did not resolve an equivalent endpoint alias");
    Check(endpointLease->Matches(&endpointAliasB),
        "Matches did not resolve an equivalent endpoint alias");
    QString cancelledOwner;
    Check(RobotOperationLease::RequestCancellation(&endpointAliasB, &cancelledOwner),
        "equivalent endpoint alias could not request cancellation");
    Check(cancelledOwner == QStringLiteral("same-endpoint-owner")
            && endpointLease->CancellationRequested()
            && RobotOperationLease::IsCancellationRequested(&endpointAliasA),
        "cancellation latch did not propagate across endpoint aliases");

    // 析构按取得时冻结的 identity 释放，不能因 driver 配置字段变化而遗留占用。
    endpointAliasA.m_sSocketIP = "192.168.10.99";
    Check(RobotOperationLease::CurrentOwner(&endpointAliasA) == QStringLiteral("same-endpoint-owner"),
        "CurrentOwner lost an active lease after acquiring driver fields changed");
    Check(!RobotOperationLease::TryAcquire(
        &endpointAliasA, QStringLiteral("same-pointer-after-endpoint-change")),
        "one driver reacquired a second lease after its endpoint fields changed");
    endpointLease.reset();
    Check(RobotOperationLease::IsCancellationRequested(&endpointAliasB),
        "unresolved cancellation latch was lost when the active lease released");
    Check(!RobotOperationLease::MotionCompletionPending(&endpointAliasB),
        "ordinary cancelled lease was misclassified as unresolved robot motion");
    Check(!RobotOperationLease::TryAcquire(
        &endpointAliasA, QStringLiteral("blocked-after-same-pointer-endpoint-change")),
        "same driver bypassed unresolved stop by changing endpoint fields");
    Check(!RobotOperationLease::TryAcquire(
        &endpointAliasB, QStringLiteral("blocked-before-stop-confirmation")),
        "new lease bypassed unresolved safety-stop latch");
    Check(RobotOperationLease::ConfirmCancellationHandled(&endpointAliasA),
        "verified stop could not clear frozen unresolved identity through driver-pointer fallback");
    Check(!RobotOperationLease::AnyActive(),
        "lease destruction did not release the frozen endpoint identity");
    auto aliasReacquired = RobotOperationLease::TryAcquire(
        &endpointAliasB, QStringLiteral("same-endpoint-reacquired"));
    Check(static_cast<bool>(aliasReacquired),
        "equivalent endpoint could not be reacquired after lease destruction");
    aliasReacquired.reset();

    auto globalActiveLease = RobotOperationLease::TryAcquire(
        firstDriver, QStringLiteral("global-active-owner"));
    Check(static_cast<bool>(globalActiveLease), "global cancellation active setup failed");
    const auto globalTargets = RobotOperationLease::LatchGlobalCancellation(
        secondDriver, QStringLiteral("global-required-current"));
    Check(globalTargets.size() == 2,
        "global cancellation did not atomically include active and required-current drivers");
    Check(globalActiveLease->CancellationRequested(),
        "global cancellation did not mark the active lease cancelled");
    Check(!RobotOperationLease::TryAcquire(secondDriver, QStringLiteral("global-required-contender")),
        "required current driver was not pinned by the global cancellation latch");
    Check(RobotOperationLease::IsCancellationRequested(secondDriver),
        "required current driver did not expose the unresolved cancellation to low-level guards");
    globalActiveLease.reset();
    Check(RobotOperationLease::AnyActive(),
        "unresolved global stop was lost when the original active lease ended");
    Check(RobotOperationLease::ConfirmCancellationHandled(firstDriver),
        "could not confirm globally cancelled active driver");
    Check(RobotOperationLease::ConfirmCancellationHandled(secondDriver),
        "could not confirm globally latched required-current driver");
    Check(!RobotOperationLease::AnyActive(),
        "global cancellation confirmations did not clear every pinned target");

    RobotDriverAdaptor ipv6AliasA;
    ipv6AliasA.m_sSocketIP = "[2001:0DB8:0:0:0:0:0:1]";
    ipv6AliasA.m_nSocketPort = 8193;
    RobotDriverAdaptor ipv6AliasB;
    ipv6AliasB.m_sSocketIP = "2001:db8::1";
    ipv6AliasB.m_nSocketPort = 8193;
    auto ipv6Lease = RobotOperationLease::TryAcquire(
        &ipv6AliasA, QStringLiteral("ipv6-owner"));
    Check(static_cast<bool>(ipv6Lease), "IPv6 endpoint setup acquire failed");
    Check(!RobotOperationLease::TryAcquire(&ipv6AliasB, QStringLiteral("ipv6-contender")),
        "equivalent IPv6 spellings did not share one lease identity");
    ipv6Lease.reset();

    std::atomic<int> parallelAcquired{ 0 };
    std::atomic<bool> releaseParallel{ false };
    std::thread first([&]()
        {
            auto lease = RobotOperationLease::TryAcquire(firstDriver, QStringLiteral("parallel-first"));
            if (lease)
            {
                parallelAcquired.fetch_add(1, std::memory_order_release);
                while (!releaseParallel.load(std::memory_order_acquire))
                {
                    std::this_thread::yield();
                }
            }
        });
    std::thread second([&]()
        {
            auto lease = RobotOperationLease::TryAcquire(secondDriver, QStringLiteral("parallel-second"));
            if (lease)
            {
                parallelAcquired.fetch_add(1, std::memory_order_release);
                while (!releaseParallel.load(std::memory_order_acquire))
                {
                    std::this_thread::yield();
                }
            }
        });
    Check(WaitUntil(parallelAcquired, 2), "different drivers could not acquire in parallel");
    const QString summary = RobotOperationLease::ActiveSummary();
    Check(summary.contains(QStringLiteral("parallel-first")), "summary omitted first driver owner");
    Check(summary.contains(QStringLiteral("parallel-second")), "summary omitted second driver owner");
    Check(RobotOperationLease::CurrentOwner(firstDriver) == QStringLiteral("parallel-first"),
        "CurrentOwner returned wrong first-driver owner");
    Check(RobotOperationLease::CurrentOwner(secondDriver) == QStringLiteral("parallel-second"),
        "CurrentOwner returned wrong second-driver owner");
    releaseParallel.store(true, std::memory_order_release);
    first.join();
    second.join();
    Check(!RobotOperationLease::AnyActive(), "parallel leases did not unregister");

    RobotDriverAdaptor emptyEndpointA;
    emptyEndpointA.m_nSocketPort = 8193;
    RobotDriverAdaptor emptyEndpointB;
    emptyEndpointB.m_nSocketPort = 8193;
    auto emptyLeaseA = RobotOperationLease::TryAcquire(
        &emptyEndpointA, QStringLiteral("empty-endpoint-a"));
    auto emptyLeaseB = RobotOperationLease::TryAcquire(
        &emptyEndpointB, QStringLiteral("empty-endpoint-b"));
    Check(static_cast<bool>(emptyLeaseA) && static_cast<bool>(emptyLeaseB),
        "empty endpoints did not fall back to distinct pointer identities");
    emptyLeaseA.reset();
    emptyLeaseB.reset();

    RobotDriverAdaptor invalidEndpointA;
    invalidEndpointA.m_sSocketIP = "192.168.10.40";
    invalidEndpointA.m_nSocketPort = 0;
    RobotDriverAdaptor invalidEndpointB;
    invalidEndpointB.m_sSocketIP = "192.168.10.40";
    invalidEndpointB.m_nSocketPort = 70000;
    auto invalidLeaseA = RobotOperationLease::TryAcquire(
        &invalidEndpointA, QStringLiteral("invalid-endpoint-a"));
    auto invalidLeaseB = RobotOperationLease::TryAcquire(
        &invalidEndpointB, QStringLiteral("invalid-endpoint-b"));
    Check(static_cast<bool>(invalidLeaseA) && static_cast<bool>(invalidLeaseB),
        "invalid endpoints did not fall back to distinct pointer identities");
    invalidLeaseA.reset();
    invalidLeaseB.reset();

    auto crossThread = RobotOperationLease::TryAcquire(firstDriver, QStringLiteral("cross-thread"));
    Check(static_cast<bool>(crossThread), "cross-thread setup acquire failed");
    std::thread destroyer([lease = std::move(crossThread)]() mutable { lease.reset(); });
    destroyer.join();
    Check(!RobotOperationLease::AnyActive(), "cross-thread destruction did not release registry entry");

    auto motionLease = RobotOperationLease::TryAcquire(firstDriver, QStringLiteral("motion-terminal-tracking"));
    Check(static_cast<bool>(motionLease), "motion tracking setup acquire failed");
    QString motionReason;
    Check(RobotOperationLease::MarkMotionStarted(firstDriver, false, &motionReason),
        "first motion could not be armed under its lease");
    Check(RobotOperationLease::MotionCompletionPending(firstDriver),
        "armed motion was not reported pending");
    Check(!RobotOperationLease::MarkMotionStarted(firstDriver, false, &motionReason),
        "second motion bypassed an unverified predecessor");
    Check(RobotOperationLease::MarkMotionStarted(firstDriver, true, &motionReason),
        "explicit same-program resume could not reaffirm pending motion");
    motionLease.reset();
    Check(RobotOperationLease::AnyActive(),
        "unverified motion disappeared when its lease was destroyed");
    Check(RobotOperationLease::MotionCompletionPending(firstDriver),
        "unresolved motion lost its motion-pending provenance after lease destruction");
    Check(!RobotOperationLease::TryAcquire(firstDriver, QStringLiteral("blocked-by-unverified-motion")),
        "new lease bypassed an unverified motion terminal latch");
    Check(RobotOperationLease::MarkMotionCompleted(firstDriver),
        "verified retry stop could not clear unresolved motion provenance");
    Check(!RobotOperationLease::MotionCompletionPending(firstDriver),
        "verified retry stop left unresolved motion provenance set");
    Check(RobotOperationLease::ConfirmCancellationHandled(firstDriver),
        "verified stop could not clear an unverified motion latch");
    Check(!RobotOperationLease::AnyActive(),
        "unverified motion latch remained after verified stop confirmation");

    auto completedMotionLease = RobotOperationLease::TryAcquire(
        firstDriver, QStringLiteral("motion-normal-completion"));
    Check(static_cast<bool>(completedMotionLease), "normal motion completion setup acquire failed");
    Check(RobotOperationLease::MarkMotionStarted(firstDriver),
        "normal motion could not be armed");
    Check(RobotOperationLease::MarkMotionCompleted(firstDriver),
        "stable motion terminal could not clear pending state");
    completedMotionLease.reset();
    Check(!RobotOperationLease::AnyActive(),
        "normally completed motion left a false sticky latch");

    auto stoppedWhileActiveLease = RobotOperationLease::TryAcquire(
        firstDriver, QStringLiteral("motion-stop-confirmed-before-destruction"));
    Check(static_cast<bool>(stoppedWhileActiveLease),
        "active stop-confirmation setup acquire failed");
    Check(RobotOperationLease::MarkMotionStarted(firstDriver),
        "active stop-confirmation motion could not be armed");
    Check(RobotOperationLease::RequestCancellation(firstDriver),
        "active stop-confirmation could not latch cancellation");
    Check(RobotOperationLease::MotionCompletionPending(firstDriver),
        "latched active motion lost pending provenance before terminal proof");
    Check(RobotOperationLease::MarkMotionCompleted(firstDriver),
        "natural stable terminal could not clear latched motion provenance");
    Check(!RobotOperationLease::MotionCompletionPending(firstDriver),
        "stable terminal left unresolved motion provenance set");
    Check(RobotOperationLease::ConfirmCancellationHandled(firstDriver),
        "verified stop could not clear active motion latch");
    stoppedWhileActiveLease.reset();
    Check(!RobotOperationLease::AnyActive(),
        "lease destruction recreated a sticky latch after verified stop");

    auto reacquired = RobotOperationLease::TryAcquire(firstDriver, QStringLiteral("reacquired"));
    Check(static_cast<bool>(reacquired), "driver could not be reacquired after cross-thread destruction");
    reacquired.reset();

    QString nullReason;
    Check(!RobotOperationLease::TryAcquire(nullptr, QStringLiteral("invalid"), &nullReason),
        "null driver unexpectedly acquired a lease");
    Check(!nullReason.isEmpty(), "null-driver rejection did not provide a reason");

    std::cout << "PASS: pointer and normalized-endpoint exclusion, endpoint fallback, parallelism, release, and summary\n";
    return 0;
}
