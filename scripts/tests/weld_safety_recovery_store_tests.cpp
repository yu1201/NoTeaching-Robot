#include "AppPaths.h"
#include "ConfigDatabase.h"
#include "WeldResumePlanner.h"
#include "WeldSafetyRecoveryStore.h"

#include <QCoreApplication>
#include <QDir>

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

WeldResumePlanner::CheckpointRecord CompleteRecord(
    const QString& robotName,
    const QString& endpoint,
    const QString& id,
    int reasonCharacters = 0)
{
    WeldResumePlanner::CheckpointRecord record;
    record.state = QStringLiteral("prepared");
    record.checkpointId = id;
    record.createdAtUtc = QStringLiteral("2026-07-13T01:02:03.000Z");
    record.robotName = robotName;
    record.robotType = QStringLiteral("STEP");
    record.robotEndpoint = endpoint;
    record.paramGroupIndex = 0;
    record.paramGroupName = QStringLiteral("Group 1");
    record.scanSection = QStringLiteral("PreciseMeasureParam1");
    record.weldSection = QStringLiteral("PreciseWeldParam1");
    record.parameterFingerprint = QString(64, QLatin1Char('a'));
    record.caseId = QStringLiteral("Case001");
    record.caseRelativeDir = QStringLiteral("Result/%1/Case001").arg(robotName);
    record.trajectoryRelativePath = QStringLiteral("Result/%1/Case001/LaserPoint/PreciseLaserPoint_FinalSampled.txt")
        .arg(robotName);
    record.trajectorySha256 = QString(64, QLatin1Char('b'));
    record.trajectorySize = 12345;
    record.trajectoryPointCount = 12;
    record.trajectoryInExecutionOrder = true;
    record.programName = QStringLiteral("Weld_TEST_001");
    record.programLine = -1;
    record.weldDirection = 1;
    record.actualWeld = true;
    record.finalStepMm = 4.0;
    record.backtrackMm = 5.0;
    record.safetyObservedAtUtc = record.createdAtUtc;
    record.safetyReason = reasonCharacters > 0
        ? QString(reasonCharacters, QChar(0x710A))
        : QStringLiteral("START前持久门禁测试");
    record.safetyProgramCompleted = false;
    record.safetyMoveSpeedMmPerMin = 600.0;
    record.safeX = 100.0;
    record.safeY = 200.0;
    record.safeZ = 300.0;
    record.safetyWitnessSha256 = WeldResumePlanner::BuildSafeRetreatWitness(record);
    return record;
}

WeldResumePlanner::CheckpointRecord InterruptedRecord(
    const QString& robotName,
    const QString& endpoint,
    const QString& id)
{
    auto record = CompleteRecord(robotName, endpoint, id);
    record.state = QStringLiteral("interrupted");
    record.safetyProgramCompleted = false;
    record.safetyReason = QStringLiteral("START结果未知，必须先终止旧程序。");
    record.safetyWitnessSha256 = WeldResumePlanner::BuildSafeRetreatWitness(record);
    return record;
}

QString Encode(const WeldResumePlanner::CheckpointRecord& record)
{
    QString error;
    const QString encoded = WeldResumePlanner::EncodeRecord(record, &error);
    Check(!encoded.isEmpty(), "could not encode recovery record");
    return encoded;
}

void InitializePaths(const QString& root)
{
    QString error;
    Check(AppPaths::Initialize(
        QStringList{ QStringLiteral("WeldSafetyRecoveryStoreTests"),
            QStringLiteral("--data-root"), QDir::toNativeSeparators(root) },
        &error), "AppPaths initialization failed");
}

void RunSuite(const QString& root)
{
    InitializePaths(root);
    const QString endpointA = QStringLiteral("tcp:[192.168.50.10]:8193");
    QString reason;
    Check(!WeldSafetyRecoveryStore::PersistentAdmissionBlocked(
        QStringLiteral("FreshRobot"), endpointA, &reason),
        "genuine first-use state should be admitted");

    qputenv("QTWIDGETSAPP4_TEST_CONFIG_CURSOR_ERROR", QByteArrayLiteral("1"));
    QString injectedReadback;
    Check(ConfigDatabase::ReadScopedSettingStatus(
        QStringLiteral("robot"), QStringLiteral("CursorErrorRobot"),
        QStringLiteral("WeldBreakpoint/Breakpoint"),
        QStringLiteral("SafeRetreatPending"), &injectedReadback)
            == ConfigDatabase::ReadStatus::Error,
        "injected no-row cursor failure was misclassified as NotFound");
    reason.clear();
    Check(WeldSafetyRecoveryStore::PersistentAdmissionBlocked(
        QStringLiteral("CursorErrorRobot"), QStringLiteral("tcp:[192.168.50.8]:8193"),
        &reason),
        "cursor read failure was admitted as genuine first use");
    qunsetenv("QTWIDGETSAPP4_TEST_CONFIG_CURSOR_ERROR");

    QString error;
    Check(WeldSafetyRecoveryStore::InvalidateIfNoPending(
        QStringLiteral("RobotInvalidated"), error),
        "first-use invalidation could not persist an explicit safe marker");
    reason.clear();
    Check(!WeldSafetyRecoveryStore::PersistentAdmissionBlocked(
        QStringLiteral("RobotAfterInvalidation"), QStringLiteral("tcp:[192.168.50.9]:8193"),
        &reason),
        "safe invalidation tombstone poisoned the global legacy alias scan");

    const auto longRecord = CompleteRecord(
        QStringLiteral("RobotLong"), QStringLiteral("tcp:[192.168.50.11]:8193"),
        QStringLiteral("long-record"), 10000);
    const QString longEncoded = Encode(longRecord);
    Check(longEncoded.toUtf8().size() > 32000 && longEncoded.toUtf8().size() < 64 * 1024,
        "long RecordV2 fixture is outside the intended bound");
    Check(WeldSafetyRecoveryStore::WriteRecord(
        longRecord.robotName, longEncoded, &error), "long RecordV2 write/readback failed");
    WeldResumePlanner::CheckpointRecord longReadback;
    QString encodedReadback;
    Check(WeldSafetyRecoveryStore::ReadRecord(
        longRecord.robotName, longReadback, &encodedReadback, &error),
        "long RecordV2 could not be read through ConfigStore");
    Check(encodedReadback == longEncoded && longReadback.safetyReason == longRecord.safetyReason,
        "long RecordV2 UTF-8 readback changed content");

    const auto missingMarker = CompleteRecord(
        QStringLiteral("RobotMissingMarker"), QStringLiteral("tcp:[192.168.50.12]:8193"),
        QStringLiteral("missing-marker"));
    Check(WeldSafetyRecoveryStore::WriteRecord(
        missingMarker.robotName, Encode(missingMarker), &error),
        "could not seed marker-missing RecordV2");
    reason.clear();
    Check(WeldSafetyRecoveryStore::PersistentAdmissionBlocked(
        missingMarker.robotName, missingMarker.robotEndpoint, &reason),
        "RecordV2 without marker must fail closed");

    const auto pending = CompleteRecord(
        QStringLiteral("RobotPrimary"), endpointA, QStringLiteral("alias-pending"));
    Check(WeldSafetyRecoveryStore::BeginOrUpdatePending(
        pending.robotName, Encode(pending), &error),
        "could not seed endpoint-indexed pending state");
    reason.clear();
    Check(WeldSafetyRecoveryStore::PersistentAdmissionBlocked(
        QStringLiteral("RenamedRobotAlias"), endpointA, &reason),
        "same physical endpoint under another robot name bypassed pending state");

    QString indexedRobot;
    QString indexedEndpoint;
    Check(ConfigDatabase::ReadScopedSettingStatus(
        QStringLiteral("weld_safety_endpoint"), endpointA,
        QStringLiteral("RecoveryIndexV1"), QStringLiteral("RobotName"),
        &indexedRobot) == ConfigDatabase::ReadStatus::Found
        && indexedRobot == pending.robotName,
        "endpoint index was not transactionally written and read back");
    Check(ConfigDatabase::ReadScopedSettingStatus(
        QStringLiteral("weld_safety_endpoint"), endpointA,
        QStringLiteral("RecoveryIndexV1"), QStringLiteral("EndpointIdentity"),
        &indexedEndpoint) == ConfigDatabase::ReadStatus::Found
        && indexedEndpoint == endpointA,
        "endpoint index key/value identity was not canonically recomputed and read back");

    auto paused = CompleteRecord(
        QStringLiteral("RobotPaused"), QStringLiteral("tcp:[192.168.50.14]:8193"),
        QStringLiteral("paused-binding"));
    paused.state = QStringLiteral("paused");
    paused.programLine = 17;
    Check(WeldSafetyRecoveryStore::BeginOrUpdatePending(
        paused.robotName, Encode(paused), &error),
        "could not seed paused resume binding");
    WeldResumePlanner::CheckpointRecord acquiredPaused;
    Check(WeldSafetyRecoveryStore::ReadPausedResumeBinding(
        paused.robotName, paused.robotEndpoint, paused, &acquiredPaused, &error)
        && acquiredPaused.checkpointId == paused.checkpointId,
        "dedicated paused resume binding was not admitted");
    RobotRecoverySafetyPolicy::ExclusiveRecoveryBinding pausedBinding;
    Check(WeldSafetyRecoveryStore::AcquireExclusiveRecoveryBinding(
        paused.robotName, paused.robotEndpoint, paused,
        RobotRecoverySafetyPolicy::RecoveryBindingMode::PausedResume,
        &pausedBinding, &error),
        "unique paused endpoint recovery binding was not acquired");
    RobotRecoverySafetyPolicy::ExclusiveRecoveryBinding duplicateBinding;
    Check(!WeldSafetyRecoveryStore::AcquireExclusiveRecoveryBinding(
        paused.robotName, paused.robotEndpoint, paused,
        RobotRecoverySafetyPolicy::RecoveryBindingMode::PausedResume,
        &duplicateBinding, &error),
        "second Store recovery binding acquired the same endpoint");
    bool stopFailurePending = false;
    Check(WeldSafetyRecoveryStore::ReadPending(
        paused.robotName, stopFailurePending, &error) && stopFailurePending,
        "STOP failure simulation cleared paused pending");
    auto swappedPaused = paused;
    swappedPaused.checkpointId = QStringLiteral("paused-binding-swapped");
    Check(WeldSafetyRecoveryStore::WriteRecord(
        swappedPaused.robotName, Encode(swappedPaused), &error),
        "could not simulate paused binding TOCTOU");
    Check(!WeldSafetyRecoveryStore::RevalidateExclusiveRecoveryBinding(
        pausedBinding, &error),
        "STOP-period paused record replacement retained recovery binding");
    Check(!WeldSafetyRecoveryStore::TransitionBoundRecordState(
        &pausedBinding, QStringLiteral("paused"), QStringLiteral("resuming"), &error),
        "STOP-period paused record replacement transitioned to resuming");
    WeldSafetyRecoveryStore::ReleaseExclusiveRecoveryBinding(
        pausedBinding.endpointIdentity, pausedBinding.token);
    Check(!WeldSafetyRecoveryStore::ReadPausedResumeBinding(
        paused.robotName, paused.robotEndpoint, paused, &acquiredPaused, &error),
        "paused resume accepted a checkpoint changed after confirmation");

    // Simulate a pre-index record. Admission must enumerate the bounded legacy
    // robot scopes and compare canonical physical endpoints, not robot labels.
    const auto legacyAlias = CompleteRecord(
        QStringLiteral("RobotLegacyName"), QStringLiteral("tcp:[ROBOT-CELL.LOCAL.]:8193"),
        QStringLiteral("legacy-alias"));
    Check(WeldSafetyRecoveryStore::WriteRecord(
        legacyAlias.robotName, Encode(legacyAlias), &error),
        "could not seed pre-index legacy pending record");
    Check(ConfigDatabase::WriteScopedSetting(
        QStringLiteral("robot"), legacyAlias.robotName,
        QStringLiteral("WeldBreakpoint/Breakpoint"),
        QStringLiteral("SafeRetreatPending"), QStringLiteral("1"), QStringLiteral("bool")),
        "could not seed pre-index pending marker");
    reason.clear();
    Check(WeldSafetyRecoveryStore::PersistentAdmissionBlocked(
        QStringLiteral("RobotLegacyRenamed"), QStringLiteral("tcp:[robot-cell.local]:8193"),
        &reason),
        "bounded legacy scan missed a canonical endpoint alias");
    auto legacyCurrentPaused = CompleteRecord(
        QStringLiteral("RobotLegacyCurrent"), QStringLiteral("tcp:[robot-cell.local]:8193"),
        QStringLiteral("legacy-current-paused"));
    legacyCurrentPaused.state = QStringLiteral("paused");
    Check(WeldSafetyRecoveryStore::BeginOrUpdatePending(
        legacyCurrentPaused.robotName, Encode(legacyCurrentPaused), &error),
        "could not seed current paused record beside legacy alias");
    RobotRecoverySafetyPolicy::ExclusiveRecoveryBinding aliasBlockedBinding;
    Check(!WeldSafetyRecoveryStore::AcquireExclusiveRecoveryBinding(
        legacyCurrentPaused.robotName, legacyCurrentPaused.robotEndpoint,
        legacyCurrentPaused,
        RobotRecoverySafetyPolicy::RecoveryBindingMode::PausedResume,
        &aliasBlockedBinding, &error),
        "legacy pre-index alias pending did not block current paused recovery");

    const auto caseOnlyScope = CompleteRecord(
        QStringLiteral("RobotCaseScope"), QStringLiteral("tcp:[192.168.50.15]:8193"),
        QStringLiteral("case-only-scope"));
    Check(WeldSafetyRecoveryStore::WriteRecord(
        caseOnlyScope.robotName, Encode(caseOnlyScope), &error),
        "could not seed case-only pre-index pending record");
    Check(ConfigDatabase::WriteScopedSetting(
        QStringLiteral("robot"), caseOnlyScope.robotName,
        QStringLiteral("WeldBreakpoint/Breakpoint"),
        QStringLiteral("SafeRetreatPending"), QStringLiteral("1"), QStringLiteral("bool")),
        "could not seed case-only pending marker");
    reason.clear();
    Check(WeldSafetyRecoveryStore::PersistentAdmissionBlocked(
        QStringLiteral("robotcasescope"), caseOnlyScope.robotEndpoint, &reason),
        "case-only robot scope alias bypassed pre-index pending state");
    auto caseOnlyCurrent = CompleteRecord(
        QStringLiteral("robotcasescope"), caseOnlyScope.robotEndpoint,
        QStringLiteral("case-only-current"));
    caseOnlyCurrent.state = QStringLiteral("paused");
    Check(WeldSafetyRecoveryStore::BeginOrUpdatePending(
        caseOnlyCurrent.robotName, Encode(caseOnlyCurrent), &error),
        "could not seed case-only current paused record");
    Check(!WeldSafetyRecoveryStore::AcquireExclusiveRecoveryBinding(
        caseOnlyCurrent.robotName, caseOnlyCurrent.robotEndpoint, caseOnlyCurrent,
        RobotRecoverySafetyPolicy::RecoveryBindingMode::PausedResume,
        &aliasBlockedBinding, &error),
        "case-only legacy scope did not block exclusive paused recovery");

    const QString safeDualEndpoint = QStringLiteral("tcp:[192.168.50.16]:8193");
    const auto safeDualA = InterruptedRecord(
        QStringLiteral("RobotSafeDualA"), safeDualEndpoint, QStringLiteral("safe-dual-a"));
    const auto safeDualB = InterruptedRecord(
        QStringLiteral("RobotSafeDualB"), safeDualEndpoint, QStringLiteral("safe-dual-b"));
    Check(WeldSafetyRecoveryStore::BeginOrUpdatePending(
        safeDualA.robotName, Encode(safeDualA), &error)
        && WeldSafetyRecoveryStore::BeginOrUpdatePending(
            safeDualB.robotName, Encode(safeDualB), &error),
        "could not seed dual safe-retreat records");
    Check(!WeldSafetyRecoveryStore::AcquireExclusiveRecoveryBinding(
        safeDualB.robotName, safeDualEndpoint, safeDualB,
        RobotRecoverySafetyPolicy::RecoveryBindingMode::SafeRetreat,
        &aliasBlockedBinding, &error),
        "two safe-retreat records on one endpoint were not rejected");

    auto stopSwap = InterruptedRecord(
        QStringLiteral("RobotStopSwap"), QStringLiteral("tcp:[192.168.50.17]:8193"),
        QStringLiteral("stop-swap"));
    Check(WeldSafetyRecoveryStore::BeginOrUpdatePending(
        stopSwap.robotName, Encode(stopSwap), &error),
        "could not seed interrupted STOP-swap record");
    RobotRecoverySafetyPolicy::ExclusiveRecoveryBinding stopSwapBinding;
    Check(WeldSafetyRecoveryStore::AcquireExclusiveRecoveryBinding(
        stopSwap.robotName, stopSwap.robotEndpoint, stopSwap,
        RobotRecoverySafetyPolicy::RecoveryBindingMode::SafeRetreat,
        &stopSwapBinding, &error),
        "could not acquire interrupted recovery binding");
    auto replacedDuringStop = stopSwap;
    replacedDuringStop.safetyReason = QStringLiteral("STOP期间记录被替换");
    replacedDuringStop.safetyWitnessSha256 =
        WeldResumePlanner::BuildSafeRetreatWitness(replacedDuringStop);
    Check(WeldSafetyRecoveryStore::WriteRecord(
        replacedDuringStop.robotName, Encode(replacedDuringStop), &error),
        "could not simulate safe-retreat STOP-period replacement");
    const bool moveAllowedAfterStopSwap =
        WeldSafetyRecoveryStore::RevalidateExclusiveRecoveryBinding(stopSwapBinding, &error)
        && RobotRecoverySafetyPolicy::MayMoveForSafeRetreat(
            stopSwapBinding.record.state,
            stopSwapBinding.record.safetyProgramCompleted,
            true);
    Check(!moveAllowedAfterStopSwap,
        "STOP-period safe-retreat replacement still admitted Move");
    WeldSafetyRecoveryStore::ReleaseExclusiveRecoveryBinding(
        stopSwapBinding.endpointIdentity, stopSwapBinding.token);

    const auto stopFailure = InterruptedRecord(
        QStringLiteral("RobotStopFailure"), QStringLiteral("tcp:[192.168.50.18]:8193"),
        QStringLiteral("stop-failure"));
    Check(WeldSafetyRecoveryStore::BeginOrUpdatePending(
        stopFailure.robotName, Encode(stopFailure), &error),
        "could not seed STOP-failure pending record");
    RobotRecoverySafetyPolicy::ExclusiveRecoveryBinding stopFailureBinding;
    Check(WeldSafetyRecoveryStore::AcquireExclusiveRecoveryBinding(
        stopFailure.robotName, stopFailure.robotEndpoint, stopFailure,
        RobotRecoverySafetyPolicy::RecoveryBindingMode::SafeRetreat,
        &stopFailureBinding, &error),
        "could not acquire STOP-failure recovery binding");
    WeldSafetyRecoveryStore::ReleaseExclusiveRecoveryBinding(
        stopFailureBinding.endpointIdentity, stopFailureBinding.token);
    Check(WeldSafetyRecoveryStore::ReadPending(
        stopFailure.robotName, stopFailurePending, &error) && stopFailurePending,
        "failed STOP cleared persistent recovery pending");

    // Keep this fixture last: an unreadable legacy marker intentionally makes the
    // global alias scan fail closed, so it must not be allowed to mask the alias test.
    const auto invalidMarker = CompleteRecord(
        QStringLiteral("RobotInvalidMarker"), QStringLiteral("tcp:[192.168.50.13]:8193"),
        QStringLiteral("invalid-marker"));
    Check(WeldSafetyRecoveryStore::WriteRecord(
        invalidMarker.robotName, Encode(invalidMarker), &error),
        "could not seed invalid-marker record");
    Check(ConfigDatabase::WriteScopedSetting(
        QStringLiteral("robot"), invalidMarker.robotName,
        QStringLiteral("WeldBreakpoint/Breakpoint"),
        QStringLiteral("SafeRetreatPending"), QStringLiteral("2"), QStringLiteral("bool")),
        "could not seed invalid marker value");
    reason.clear();
    Check(WeldSafetyRecoveryStore::PersistentAdmissionBlocked(
        invalidMarker.robotName, invalidMarker.robotEndpoint, &reason),
        "non-boolean marker must fail closed");

    std::cout << "PASS: WeldSafetyRecoveryStore ConfigStore suite\n";
}

void SeedReadFailure(const QString& root)
{
    InitializePaths(root);
    const auto record = CompleteRecord(
        QStringLiteral("RobotReadFailure"), QStringLiteral("tcp:[192.168.50.20]:8193"),
        QStringLiteral("read-failure"));
    QString error;
    Check(WeldSafetyRecoveryStore::BeginOrUpdatePending(
        record.robotName, Encode(record), &error), "could not seed read-failure state");
    std::cout << "PASS: read-failure fixture seeded\n";
}

void ExpectReadFailureBlocked(const QString& root)
{
    InitializePaths(root);
    QString reason;
    Check(WeldSafetyRecoveryStore::PersistentAdmissionBlocked(
        QStringLiteral("RobotReadFailure"), QStringLiteral("tcp:[192.168.50.20]:8193"),
        &reason), "ConfigStore read failure must fail closed");
    Check(!reason.isEmpty(), "read failure did not produce a diagnostic");
    std::cout << "PASS: ConfigStore read failure blocked admission\n";
}
}

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);
    const QStringList args = app.arguments();
    Check(args.size() == 3, "usage: test <suite|seed-read-failure|expect-read-failure> <data-root>");
    const QString mode = args.at(1);
    const QString root = QDir::cleanPath(QDir::fromNativeSeparators(args.at(2)));
    if (mode == QStringLiteral("suite"))
    {
        RunSuite(root);
    }
    else if (mode == QStringLiteral("seed-read-failure"))
    {
        SeedReadFailure(root);
    }
    else if (mode == QStringLiteral("expect-read-failure"))
    {
        ExpectReadFailureBlocked(root);
    }
    else
    {
        Check(false, "unknown test mode");
    }
    return 0;
}
