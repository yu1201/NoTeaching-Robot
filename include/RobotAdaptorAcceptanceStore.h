#pragma once

#include "ConfigDatabase.h"
#include "RobotAdaptorAcceptancePlan.h"

// Acceptance history is database data, not an executable robot recovery point.
namespace RobotAdaptorAcceptanceStore
{
using Record = QMap<QString, QString>;
using History = QMap<QString, Record>;
inline constexpr int StageCount = RobotAdaptorAcceptancePlan::StageCount;

inline QString Schema() { return QStringLiteral("RobotAdaptorAcceptanceV1"); }

inline bool ReadHistory(const QString& robotName, History& history, QString* error)
{
    return ConfigDatabase::ReadScopedModuleSnapshot(
        "robot", robotName, "RobotAdaptorAcceptance", history, error);
}

inline bool NormalizeRecord(const QString& robotName, Record& record, QString* error)
{
    const bool legacyBooleanSchema = record.value("Schema") == "1";
    bool complete = record.value("RobotName") == robotName
        && !record.value("DriverType").isEmpty();
    for (int stage = 0; stage < StageCount; ++stage)
    {
        complete = complete && record.contains(QStringLiteral("Stage%1State").arg(stage))
            && record.contains(QStringLiteral("Stage%1Evidence").arg(stage));
    }
    if (!complete || (!legacyBooleanSchema && record.value("Schema") != Schema()))
    {
        if (error != nullptr)
        {
            *error = QStringLiteral("验收记录格式不支持、机器人不匹配或记录不完整；已保留原记录，未自动新建覆盖。");
        }
        return false;
    }
    // The old const char* argument selected ConfigSection's bool overload.
    // Recover only complete records belonging to this robot, never arbitrary schema=1 data.
    record["Schema"] = Schema();
    // Preserve the plan under which older evidence was actually recorded.
    if (!record.contains("PlanRevision")) { record["PlanRevision"] = "legacy-unrecorded"; }
    if (!record.contains("JointMotionState")) { record["JointMotionState"] = "pending"; }
    if (!record.contains("JointMotionEvidence")) { record["JointMotionEvidence"] = QStringLiteral(""); }
    if (record.value("JointMotionState") == "running"
        || record.value("JointMotionState") == "awaiting_return")
    {
        record["JointMotionState"] = "fail";
        record["JointMotionEvidence"] += QStringLiteral(
            "\n上次关节往返未完成；原点不跨会话恢复，请现场确认后重新测试。");
    }
    if (legacyBooleanSchema) { record["RecoveredBooleanSchema"] = "1"; }
    for (int stage = 0; stage < StageCount; ++stage)
    {
        const QString stateKey = QStringLiteral("Stage%1State").arg(stage);
        if (record.value(stateKey) == "running")
        {
            record[stateKey] = "fail";
            record[QStringLiteral("Stage%1Evidence").arg(stage)] += QStringLiteral(
                "\n上次执行未记录完成结果，可能因退出或断电中断；未自动恢复任何机器人运动，请现场确认后重新测试。");
        }
    }
    if (!record.value("RegisterRecovery").isEmpty())
    {
        record["Stage5State"] = "fail";
        const QString warning = QStringLiteral("\n寄存器原值恢复尚未确认：请按备份核对并恢复，禁止直接重新写入测试值。\n");
        if (!record.value("Stage5Evidence").contains(warning))
        { record["Stage5Evidence"] += warning + record.value("RegisterRecovery"); }
    }
    if (error != nullptr) { error->clear(); }
    return true;
}

inline bool Save(const QString& robotName, const QString& runId, Record record, QString* error)
{
    if (robotName.isEmpty() || runId.isEmpty() || runId == "Latest")
    {
        if (error != nullptr) { *error = QStringLiteral("验收记录机器人或轮次无效。"); }
        return false;
    }
    record["Schema"] = Schema();
    if (!record.contains("EvidencePlanRevision"))
    { record["EvidencePlanRevision"] = record.value("PlanRevision", "legacy-unrecorded"); }
    record["PlanRevision"] = QString::fromUtf8(RobotAdaptorAcceptancePlan::Revision);
    record["RobotName"] = robotName;
    // SQL value_text is NOT NULL. A missing optional evidence string is empty
    // data, not SQL NULL (QString() otherwise binds as NULL with QSQLITE).
    for (auto it = record.begin(); it != record.end(); ++it)
    { if (it.value().isNull()) { it.value() = QStringLiteral(""); } }
    // Keep the complete record and Latest pointer in the same transaction.
    // Empty removeSections preserves every older run and any report metadata.
    const History updates = {
        { "Latest", { { "RunId", runId } } },
        { runId, record }
    };
    return ConfigDatabase::ReplaceScopedModuleSectionsAtomically(
        "robot", robotName, "RobotAdaptorAcceptance", updates, {}, error);
}
}
