#include "AppPaths.h"
#include "RobotAdaptorAcceptanceStore.h"

#include <QCoreApplication>
#include <QDir>
#include <QSqlDatabase>
#include <QSqlQuery>
#include <cstdlib>
#include <iostream>

using namespace RobotAdaptorAcceptanceStore;

static void Check(bool ok, const char* message)
{
    if (!ok) { std::cerr << "FAIL: " << message << '\n'; std::exit(1); }
}

static Record Sample()
{
    Record record = {
        { "Schema", "1" }, { "RobotName", "RobotC" }, { "DriverType", "Inovance" },
        { "SelectedStage", "4" }, { "LinearDistanceMm", "10.000" },
        { "LinearSpeedMmPerMin", "60.000" }, { "ReportJsonPath", "old-report.json" }
    };
    for (int stage = 0; stage < StageCount; ++stage)
    {
        record[QStringLiteral("Stage%1State").arg(stage)] = stage < 4 ? "pass" : "pending";
        record[QStringLiteral("Stage%1Evidence").arg(stage)] = QStringLiteral("阶段%1证据").arg(stage);
    }
    record["Stage4State"] = "fail";
    record["Stage4Evidence"] = QStringLiteral("Motor ON：e7，急停报警 0x0080\n人工备注保留");
    record["ModeCombinationEvidence"] = QStringLiteral(
        "1_motor_ds_off_ds_motor：PASS，恢复PASS\n2_motor_ds_off_ds_motor：Dsmode ON e4，FAIL，恢复PASS\n未执行：后续组合");
    return record;
}

int main(int argc, char** argv)
{
    QCoreApplication app(argc, argv);
    const QStringList args = app.arguments();
    Check(args.size() == 3, "expected isolated root and phase");
    QString error;
    Check(AppPaths::Initialize({ args[0], "--data-root", args[1] }, &error), "initialize isolated root");
    Check(ConfigDatabase::DatabasePath().startsWith(QDir::fromNativeSeparators(args[1])), "isolated database path");
    const QString phase = args[2];
    if (phase == "seed")
    {
        const History legacy = { { "Latest", { { "RunId", "20260904_104327_279" } } },
            { "20260904_104327_279", Sample() } };
        Check(ConfigDatabase::ReplaceScopedModuleSectionsAtomically(
            "robot", "RobotC", "RobotAdaptorAcceptance", legacy, {}, &error), "seed legacy database");
    }
    else if (phase == "restart")
    {
        History history;
        Check(ReadHistory("RobotC", history, &error), "reopen database in second process");
        const QString runId = history.value("Latest").value("RunId");
        Check(runId == "20260904_104327_279", "keep existing latest pointer");
        auto record = history.value(runId);
        Check(record.value("Schema") == "1", "reproduce boolean schema bug");
        Check(NormalizeRecord("RobotC", record, &error), "recover complete legacy record");
        Check(record.value("Stage4Evidence") == Sample().value("Stage4Evidence"), "preserve Chinese evidence");
        Check(Save("RobotC", runId, record, &error), "persist schema repair");
    }
    else if (phase == "verify")
    {
        History history;
        Check(ReadHistory("RobotC", history, &error), "reopen repaired record in third process");
        auto record = history.value("20260904_104327_279");
        Check(record.value("Schema") == Schema(), "schema remains a string after restart");
        Check(record.value("Stage0State") == "pass" && record.value("Stage4State") == "fail", "preserve stage results");
        Check(record.value("SelectedStage") == "4" && record.value("LinearSpeedMmPerMin") == "60.000", "preserve stage selection and parameters");
        Check(record.value("Stage4Evidence") == Sample().value("Stage4Evidence"), "preserve failure evidence after restart");
        Check(record.value("ReportJsonPath") == "old-report.json", "preserve report history");
        Check(record.value("ModeCombinationEvidence") == Sample().value("ModeCombinationEvidence"),
            "preserve every mode combination and skipped reason across restart");

        auto interrupted = record;
        auto oldLinearPass = record;
        oldLinearPass["Stage4State"] = "pass";
        oldLinearPass["PlanRevision"] = "old-plan";
        oldLinearPass.remove("JointMotionState");
        oldLinearPass.remove("JointMotionEvidence");
        Check(NormalizeRecord("RobotC", oldLinearPass, &error), "normalize old linear-only record");
        Check(oldLinearPass.value("JointMotionState") == "pending"
            && oldLinearPass.value("Stage4State") == "pass"
            && oldLinearPass.value("PlanRevision") == "old-plan", "old linear pass never approves joint motion or rewrites original plan");
        for (const QString state : { QString("running"), QString("awaiting_return") })
        {
            auto jointInterrupted = oldLinearPass;
            jointInterrupted["JointMotionState"] = state;
            jointInterrupted["JointMotionEvidence"] = "J1 outward proof";
            Check(NormalizeRecord("RobotC", jointInterrupted, &error), "normalize joint interruption");
            Check(jointInterrupted.value("JointMotionState") == "fail"
                && jointInterrupted.value("JointMotionEvidence").contains("J1 outward proof")
                && jointInterrupted.value("Stage4State") == "pass", "joint interruption fails independently and preserves linear evidence");
        }
        auto registerInterrupted = record;
        registerInterrupted["Stage5State"] = "pass";
        registerInterrupted["RegisterRecovery"] = "R[254] original=7 temporary=8";
        Check(NormalizeRecord("RobotC", registerInterrupted, &error), "normalize pending register recovery");
        Check(registerInterrupted.value("Stage5State") == "fail"
            && registerInterrupted.value("Stage5Evidence").contains("original=7")
            && !registerInterrupted.value("RegisterRecovery").isEmpty(),
            "pending restoration must retain original and override stale pass");
        const QString warning = registerInterrupted.value("Stage5Evidence");
        Check(NormalizeRecord("RobotC", registerInterrupted, &error)
            && registerInterrupted.value("Stage5Evidence") == warning, "recovery warning duplicated on reload");
        interrupted["Stage4State"] = "running";
        Check(Save("RobotC", "20260904_110000_000", interrupted, &error), "persist started stage");
        Check(ReadHistory("RobotC", history, &error), "reload running stage");
        interrupted = history.value("20260904_110000_000");
        Check(NormalizeRecord("RobotC", interrupted, &error), "normalize interrupted stage");
        Check(interrupted.value("Stage4State") == "fail"
            && interrupted.value("Stage4Evidence").contains(QStringLiteral("未自动恢复任何机器人运动")), "interruptions never resume motion or appear passed");
        Check(Save("RobotC", "20260904_110000_000", interrupted, &error), "save interruption status");
        Check(Save("RobotA", "robot_a_run", record, &error), "save second robot independently");
        Check(ReadHistory("RobotC", history, &error), "read independent RobotC history");
        Check(history.size() == 3 && history.contains("20260904_104327_279"), "new run preserves old run; robots isolated");

        auto invalid = record;
        invalid["Schema"] = "future-unknown";
        Check(!NormalizeRecord("RobotC", invalid, &error), "unknown schema rejected");
        invalid = Sample(); invalid.remove("Stage5State");
        Check(!NormalizeRecord("RobotC", invalid, &error), "incomplete legacy record rejected");
        invalid = record;
        Check(!NormalizeRecord("RobotB", invalid, &error), "wrong robot rejected");

        const History before = history;
        {
            QSqlDatabase db = QSqlDatabase::addDatabase("QSQLITE", "acceptance_failure_injection");
            db.setDatabaseName(ConfigDatabase::DatabasePath());
            Check(db.open(), "open test-only failure injection connection");
            QSqlQuery query(db);
            Check(query.exec("CREATE TRIGGER acceptance_test_fail BEFORE INSERT ON settings "
                "WHEN NEW.module='RobotAdaptorAcceptance/zz_failed' "
                "BEGIN SELECT RAISE(ABORT, 'test-only injected write failure'); END"), "create test-only failing write");
            Check(!Save("RobotC", "zz_failed", record, &error), "write failure reported");
            Check(ReadHistory("RobotC", history, &error) && history == before, "rollback both Latest and record atomically");
            Check(query.exec("DROP TRIGGER acceptance_test_fail"), "remove test-only trigger");
            db.close();
        }
        QSqlDatabase::removeDatabase("acceptance_failure_injection");
        Check(!Save("RobotC", "Latest", record, &error), "reserved run id rejected");
    }
    else if (phase == "audit-copy")
    {
        History history;
        Check(ReadHistory("RobotC", history, &error), "read field database copy");
        const QString runId = history.value("Latest").value("RunId");
        auto record = history.value(runId);
        std::cout << "Field copy latest=" << runId.toStdString()
            << " schema=" << record.value("Schema").toStdString()
            << " runs=" << history.size() - 1 << '\n';
        Check(NormalizeRecord("RobotC", record, &error), "recover actual field latest record");
        std::cout << "Recovered stage states:";
        for (int stage = 0; stage < StageCount; ++stage)
        {
            std::cout << ' ' << stage << '=' << record.value(QStringLiteral("Stage%1State").arg(stage)).toStdString();
        }
        std::cout << '\n';
        Check(!record.value("Stage4Evidence").isEmpty(), "field motion evidence preserved");
        Check(Save("RobotC", runId, record, &error), "upgrade field copy only");
        Check(ReadHistory("RobotC", history, &error), "read back upgraded field copy");
        Check(history.value("Latest").value("RunId") == runId
            && history.value(runId).value("Schema") == Schema(), "field run id unchanged and schema repaired");
    }
    else { Check(false, "unknown phase"); }
    std::cout << "PASS: acceptance store " << phase.toStdString() << '\n';
    return 0;
}
