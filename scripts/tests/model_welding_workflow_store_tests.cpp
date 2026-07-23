#include "AppPaths.h"
#include "ConfigDatabase.h"
#include "ModelWeldingWorkflow.h"

#include <QCoreApplication>
#include <QDir>

#include <cmath>
#include <cstdlib>
#include <condition_variable>
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

ModelWeldingFlowTemplate CompleteTemplate()
{
    ModelWeldingFlowTemplate value;
    value.templateId = ModelWeldingWorkflow::CreateStableId();
    value.displayName = QStringLiteral("Store roundtrip template");
    value.modelLibraryName = QStringLiteral("fixture");
    value.modelSha256 = QString(64, QLatin1Char('a'));
    value.placement.vSlotYawDegrees = 32.25;
    value.humanDatumConfirmed = true;
    value.humanCollisionChecked = true;
    value.createdAtUtc = QStringLiteral("2026-07-21T01:02:03.000Z");
    value.updatedAtUtc = value.createdAtUtc;

    const QVector<Eigen::Vector3d> points = {
        Eigen::Vector3d(0.0, 0.0, 0.0),
        Eigen::Vector3d(100.0, 0.0, 0.0),
        Eigen::Vector3d(0.0, 100.0, 0.0),
        Eigen::Vector3d(30.0, 25.0, 45.0)
    };
    for (int index = 0; index < points.size(); ++index)
    {
        ModelWeldingFeatureStation station;
        station.stationId = index < 3
            ? QStringLiteral("S%1").arg(index + 1, 2, 10, QLatin1Char('0'))
            : QStringLiteral("V01");
        station.displayName = station.stationId;
        station.role = index < 3
            ? ModelWeldingStationRole::Solve : ModelWeldingStationRole::Verify;
        station.anchorModelMm = points.at(index);
        station.scanDirectionModel = Eigen::Vector3d::UnitX();
        station.candidateConfirmed = true;
        value.stations.push_back(station);
    }
    return value;
}

ModelWeldingRobotTeaching CompleteTeaching(const ModelWeldingFlowTemplate& modelTemplate)
{
    ModelWeldingRobotTeaching value;
    value.teachingId = ModelWeldingWorkflow::CreateStableId();
    value.templateId = modelTemplate.templateId;
    value.templateRevision = modelTemplate.revision;
    QString error;
    value.templateRecordSha256 =
        ModelWeldingWorkflow::TemplateRecordSha256(modelTemplate, error);
    Check(!value.templateRecordSha256.isEmpty(), "template hash failed");
    value.robotName = QStringLiteral("RobotStoreTest");
    value.robotEndpoint = QStringLiteral("tcp://192.0.2.20:1000");
    value.robotModelId = QStringLiteral("step.sa10-2000h");
    value.sourceStepSha256 = QString(64, QLatin1Char('d'));
    value.collisionProfileSha256 = QString(64, QLatin1Char('e'));
    value.cameraSection = QStringLiteral("CAMERA0");
    value.handEyeSha256 = QString(64, QLatin1Char('b'));
    value.tool1Sha256 = QString(64, QLatin1Char('c'));
    value.createdAtUtc = QStringLiteral("2026-07-21T01:02:03.000Z");
    value.updatedAtUtc = value.createdAtUtc;
    for (int index = 0; index < modelTemplate.stations.size(); ++index)
    {
        ModelWeldingScanTeaching scan;
        scan.stationId = modelTemplate.stations.at(index).stationId;
        scan.startTaught = true;
        scan.endTaught = true;
        scan.startPulseTaught = true;
        scan.startPose = T_ROBOT_COORS(
            index * 50.0, 0.0, 300.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        scan.endPose = T_ROBOT_COORS(
            index * 50.0 + 25.0, 0.0, 300.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        scan.startPulse = T_ANGLE_PULSE(
            index, index + 1, index + 2, 0, 0, 0, 0, 0, 0);
        value.scans.push_back(scan);
    }
    return value;
}

void TestDatabaseCas()
{
    const QString scopeType = QStringLiteral("model_weld_cas_test");
    const QString scopeId = ModelWeldingWorkflow::CreateStableId();
    const QString module = QStringLiteral("DefinitionV1");
    const QString key = QStringLiteral("RecordJson");
    bool conflict = false;
    QString error;
    Check(ConfigDatabase::CompareAndSwapScopedSetting(
        scopeType, scopeId, module, key, nullptr, QStringLiteral("A"),
        QStringLiteral("string"), false, &conflict, &error),
        "CAS create should succeed");
    Check(!conflict, "CAS create should not report conflict");
    Check(!ConfigDatabase::CompareAndSwapScopedSetting(
        scopeType, scopeId, module, key, nullptr, QStringLiteral("bad"),
        QStringLiteral("string"), false, &conflict, &error),
        "second CAS create should fail");
    Check(conflict, "second CAS create should report conflict");
    const QString expectedA = QStringLiteral("A");
    Check(ConfigDatabase::CompareAndSwapScopedSetting(
        scopeType, scopeId, module, key, &expectedA, QStringLiteral("B"),
        QStringLiteral("string"), false, &conflict, &error),
        "CAS update should succeed");
    Check(!ConfigDatabase::CompareAndSwapScopedSetting(
        scopeType, scopeId, module, key, &expectedA, QStringLiteral("stale"),
        QStringLiteral("string"), false, &conflict, &error),
        "stale CAS update should fail");
    Check(conflict, "stale CAS update should report conflict");

    const QString witnessScope = QStringLiteral("model_weld_witness_test");
    const QString witnessId = ModelWeldingWorkflow::CreateStableId();
    const QString targetScope = QStringLiteral("model_weld_witness_target_test");
    const QString targetId = ModelWeldingWorkflow::CreateStableId();
    Check(ConfigDatabase::CompareAndSwapScopedSetting(
        witnessScope, witnessId, module, key, nullptr, QStringLiteral("W1"),
        QStringLiteral("string"), false, &conflict, &error),
        "witness seed should succeed");
    Check(ConfigDatabase::CompareAndSwapScopedSettingWithWitness(
        witnessScope, witnessId, module, key, QStringLiteral("W1"),
        targetScope, targetId, module, key, nullptr, QStringLiteral("T1"),
        QStringLiteral("string"), false, &conflict, &error),
        "witness CAS create should succeed");
    const QString expectedW1 = QStringLiteral("W1");
    Check(ConfigDatabase::CompareAndSwapScopedSetting(
        witnessScope, witnessId, module, key, &expectedW1, QStringLiteral("W2"),
        QStringLiteral("string"), false, &conflict, &error),
        "witness advance should succeed");
    const QString expectedT1 = QStringLiteral("T1");
    Check(!ConfigDatabase::CompareAndSwapScopedSettingWithWitness(
        witnessScope, witnessId, module, key, QStringLiteral("W1"),
        targetScope, targetId, module, key, &expectedT1, QStringLiteral("bad"),
        QStringLiteral("string"), false, &conflict, &error),
        "stale witness must reject target update");
    Check(conflict, "stale witness should report conflict");
    QString targetValue;
    Check(ConfigDatabase::ReadScopedSettingStatus(
        targetScope, targetId, module, key, &targetValue) == ConfigDatabase::ReadStatus::Found
        && targetValue == QStringLiteral("T1"),
        "witness conflict must leave target unchanged");
}

void TestConcurrentDatabaseCas()
{
    const QString scopeType = QStringLiteral("model_weld_concurrent_cas_test");
    const QString scopeId = ModelWeldingWorkflow::CreateStableId();
    const QString module = QStringLiteral("DefinitionV1");
    const QString key = QStringLiteral("RecordJson");
    bool seedConflict = false;
    QString seedError;
    Check(ConfigDatabase::CompareAndSwapScopedSetting(
        scopeType, scopeId, module, key, nullptr, QStringLiteral("R0"),
        QStringLiteral("string"), false, &seedConflict, &seedError),
        "concurrent CAS seed should succeed");

    struct Outcome
    {
        bool success = false;
        bool conflict = false;
        QString error;
    };
    Outcome outcomes[2];
    std::mutex gateMutex;
    std::condition_variable gateCondition;
    int readyCount = 0;
    bool start = false;
    auto worker = [&](int index, const QString& nextValue)
        {
            {
                std::unique_lock<std::mutex> lock(gateMutex);
                ++readyCount;
                gateCondition.notify_all();
                gateCondition.wait(lock, [&start]() { return start; });
            }
            const QString expected = QStringLiteral("R0");
            outcomes[index].success = ConfigDatabase::CompareAndSwapScopedSetting(
                scopeType, scopeId, module, key, &expected, nextValue,
                QStringLiteral("string"), false,
                &outcomes[index].conflict, &outcomes[index].error);
        };
    std::thread first(worker, 0, QStringLiteral("R1"));
    std::thread second(worker, 1, QStringLiteral("R2"));
    {
        std::unique_lock<std::mutex> lock(gateMutex);
        gateCondition.wait(lock, [&readyCount]() { return readyCount == 2; });
        start = true;
    }
    gateCondition.notify_all();
    first.join();
    second.join();

    const int successCount = (outcomes[0].success ? 1 : 0) + (outcomes[1].success ? 1 : 0);
    const int conflictCount = (outcomes[0].conflict ? 1 : 0) + (outcomes[1].conflict ? 1 : 0);
    Check(successCount == 1, "exactly one concurrent CAS writer must commit");
    Check(conflictCount == 1, "losing concurrent CAS writer must report conflict");
    QString stored;
    Check(ConfigDatabase::ReadScopedSettingStatus(
        scopeType, scopeId, module, key, &stored) == ConfigDatabase::ReadStatus::Found
        && (stored == QStringLiteral("R1") || stored == QStringLiteral("R2")),
        "concurrent CAS final value must be one complete winning value");
}

void TestConcurrentWitnessCas()
{
    const QString witnessScope = QStringLiteral("model_weld_concurrent_witness_test");
    const QString witnessId = ModelWeldingWorkflow::CreateStableId();
    const QString targetScope = QStringLiteral("model_weld_concurrent_witness_target_test");
    const QString targetId = ModelWeldingWorkflow::CreateStableId();
    const QString module = QStringLiteral("DefinitionV1");
    const QString key = QStringLiteral("RecordJson");
    bool seedConflict = false;
    QString seedError;
    Check(ConfigDatabase::CompareAndSwapScopedSetting(
        witnessScope, witnessId, module, key, nullptr, QStringLiteral("W"),
        QStringLiteral("string"), false, &seedConflict, &seedError),
        "concurrent witness seed should succeed");
    Check(ConfigDatabase::CompareAndSwapScopedSetting(
        targetScope, targetId, module, key, nullptr, QStringLiteral("T0"),
        QStringLiteral("string"), false, &seedConflict, &seedError),
        "concurrent witness target seed should succeed");

    struct Outcome
    {
        bool success = false;
        bool conflict = false;
        QString error;
    };
    Outcome outcomes[2];
    std::mutex gateMutex;
    std::condition_variable gateCondition;
    int readyCount = 0;
    bool start = false;
    auto worker = [&](int index, const QString& nextValue)
        {
            {
                std::unique_lock<std::mutex> lock(gateMutex);
                ++readyCount;
                gateCondition.notify_all();
                gateCondition.wait(lock, [&start]() { return start; });
            }
            const QString expectedTarget = QStringLiteral("T0");
            outcomes[index].success = ConfigDatabase::CompareAndSwapScopedSettingWithWitness(
                witnessScope, witnessId, module, key, QStringLiteral("W"),
                targetScope, targetId, module, key, &expectedTarget, nextValue,
                QStringLiteral("string"), false,
                &outcomes[index].conflict, &outcomes[index].error);
        };
    std::thread first(worker, 0, QStringLiteral("T1"));
    std::thread second(worker, 1, QStringLiteral("T2"));
    {
        std::unique_lock<std::mutex> lock(gateMutex);
        gateCondition.wait(lock, [&readyCount]() { return readyCount == 2; });
        start = true;
    }
    gateCondition.notify_all();
    first.join();
    second.join();

    const int successCount = (outcomes[0].success ? 1 : 0) + (outcomes[1].success ? 1 : 0);
    const int conflictCount = (outcomes[0].conflict ? 1 : 0) + (outcomes[1].conflict ? 1 : 0);
    Check(successCount == 1, "exactly one concurrent witness CAS writer must commit");
    Check(conflictCount == 1, "losing concurrent witness CAS writer must report conflict");
    QString stored;
    Check(ConfigDatabase::ReadScopedSettingStatus(
        targetScope, targetId, module, key, &stored) == ConfigDatabase::ReadStatus::Found
        && (stored == QStringLiteral("T1") || stored == QStringLiteral("T2")),
        "concurrent witness CAS target must contain one complete winning value");
}

void RunSuite(const QString& root)
{
    QString error;
    Check(AppPaths::Initialize(
        QStringList{ QStringLiteral("ModelWeldingWorkflowStoreTests"),
            QStringLiteral("--data-root"), QDir::toNativeSeparators(root) },
        &error), "AppPaths initialization failed");
    TestDatabaseCas();
    TestConcurrentDatabaseCas();
    TestConcurrentWitnessCas();

    ModelWeldingFlowTemplate value = CompleteTemplate();
    Check(ModelWeldingWorkflow::SaveTemplate(value, 0, error),
        "template create should succeed");
    ModelWeldingFlowTemplate loaded;
    Check(ModelWeldingWorkflow::LoadTemplate(value.templateId, loaded, error)
        == ModelWeldingWorkflow::LoadStatus::Found,
        "template load should succeed");
    Check(loaded.displayName == value.displayName, "template roundtrip mismatch");
    Check(std::abs(loaded.placement.vSlotYawDegrees - value.placement.vSlotYawDegrees) < 1.0e-12,
        "persisted V-slot yaw roundtrip mismatch");

    ModelWeldingFlowTemplate revision2 = loaded;
    revision2.revision = 2;
    revision2.displayName = QStringLiteral("Store roundtrip template r2");
    Check(ModelWeldingWorkflow::SaveTemplate(revision2, 1, error),
        "template revision update should succeed");
    ModelWeldingFlowTemplate stale = loaded;
    stale.revision = 2;
    stale.displayName = QStringLiteral("stale overwrite");
    Check(!ModelWeldingWorkflow::SaveTemplate(stale, 1, error),
        "stale template update must fail");

    ModelWeldingRobotTeaching teaching = CompleteTeaching(revision2);
    Check(ModelWeldingWorkflow::ValidateTeachingStructure(
        teaching, &revision2, error, true),
        "complete teaching should be production-ready");
    ModelWeldingRobotTeaching unboundLegacyDraft = teaching;
    unboundLegacyDraft.teachingId = ModelWeldingWorkflow::CreateStableId();
    unboundLegacyDraft.robotModelId.clear();
    unboundLegacyDraft.sourceStepSha256.clear();
    unboundLegacyDraft.collisionProfileSha256.clear();
    Check(!ModelWeldingWorkflow::SaveTeaching(unboundLegacyDraft, 0, error),
        "teaching persistence must reject legacy records without robot model asset identity");
    Check(ModelWeldingWorkflow::SaveTeaching(teaching, 0, error),
        "teaching create should succeed");
    ModelWeldingRobotTeaching loadedTeaching;
    Check(ModelWeldingWorkflow::LoadTeaching(teaching.teachingId, loadedTeaching, error)
        == ModelWeldingWorkflow::LoadStatus::Found,
        "teaching load should succeed");
    Check(loadedTeaching.templateRecordSha256 == teaching.templateRecordSha256,
        "teaching template binding mismatch");
    Check(loadedTeaching.robotModelId == teaching.robotModelId
        && loadedTeaching.sourceStepSha256 == teaching.sourceStepSha256
        && loadedTeaching.collisionProfileSha256 == teaching.collisionProfileSha256,
        "teaching robot model asset binding mismatch");
    ModelWeldingRobotTeaching teachingRevision2 = loadedTeaching;
    teachingRevision2.revision = 2;
    teachingRevision2.updatedAtUtc = QStringLiteral("2026-07-21T01:02:04.000Z");
    Check(ModelWeldingWorkflow::SaveTeaching(teachingRevision2, 1, error),
        "teaching revision update should succeed");
    ModelWeldingRobotTeaching staleTeaching = loadedTeaching;
    staleTeaching.revision = 2;
    staleTeaching.updatedAtUtc = QStringLiteral("2026-07-21T01:02:05.000Z");
    Check(!ModelWeldingWorkflow::SaveTeaching(staleTeaching, 1, error),
        "stale teaching update must fail");

    ModelWeldingRobotTeaching changedRobotModel = teachingRevision2;
    changedRobotModel.revision = 3;
    changedRobotModel.robotModelId = QStringLiteral("step.other-model");
    changedRobotModel.updatedAtUtc = QStringLiteral("2026-07-21T01:02:06.000Z");
    Check(!ModelWeldingWorkflow::SaveTeaching(changedRobotModel, 2, error),
        "persisted teaching robot model asset identity must be immutable");

    ModelWeldingFlowTemplate inherited = CompleteTemplate();
    inherited.displayName = QStringLiteral("Inherited snapshot");
    inherited.inheritedFromTemplateId = revision2.templateId;
    inherited.inheritedFromRevision = revision2.revision;
    inherited.inheritedFromRecordSha256 =
        ModelWeldingWorkflow::TemplateRecordSha256(revision2, error);
    inherited.humanSameFixtureConfirmed = true;
    inherited.humanScanAreaConfirmed = true;
    Check(ModelWeldingWorkflow::SaveTemplate(inherited, 0, error),
        "inherited template create with source witness should succeed");

    QVector<ModelWeldingFlowTemplate> templates;
    QVector<ModelWeldingRobotTeaching> teachings;
    Check(ModelWeldingWorkflow::ListTemplates(templates, error) && templates.size() == 2,
        "template list should contain source and inherited records");
    Check(ModelWeldingWorkflow::ListTeachings(teachings, error) && teachings.size() == 1,
        "teaching list should contain one record");

    const QString mismatchedScopeId = ModelWeldingWorkflow::CreateStableId();
    const QByteArray mismatchedJson = ModelWeldingWorkflow::EncodeTemplate(inherited, error);
    Check(!mismatchedJson.isEmpty()
        && ConfigDatabase::WriteScopedSetting(
            QStringLiteral("model_weld_template"),
            mismatchedScopeId,
            QStringLiteral("DefinitionV1"),
            QStringLiteral("RecordJson"),
            QString::fromUtf8(mismatchedJson),
            QStringLiteral("json"),
            false),
        "mismatched scope fixture should be written");
    ModelWeldingFlowTemplate mismatched;
    Check(ModelWeldingWorkflow::LoadTemplate(mismatchedScopeId, mismatched, error)
        == ModelWeldingWorkflow::LoadStatus::Error,
        "scope id and internal template id mismatch must fail closed");
    std::cout << "PASS: model welding workflow ConfigStore tests\n";
}
}

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);
    Check(app.arguments().size() == 2, "usage: test <data-root>");
    RunSuite(QDir::cleanPath(QDir::fromNativeSeparators(app.arguments().at(1))));
    return 0;
}
