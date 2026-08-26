#pragma once

#include "WeldResumePlanner.h"
#include "RobotRecoverySafetyPolicy.h"

#if !defined(WELD_SAFETY_STORE_STORAGE_ONLY_TEST)
#include "MeasureThenWeldService.h"
#endif

#include <memory>

class RobotDriverAdaptor;

// WeldBreakpoint 数据库模块的唯一读写/锁入口。所有 marker + RecordV2 组合更新均按失败关闭顺序写入并回读。
class WeldSafetyRecoveryStore final
{
public:
    static QString StorageLabel(const QString& robotName);
    static bool ReadRecord(
        const QString& robotName,
        WeldResumePlanner::CheckpointRecord& record,
        QString* encoded = nullptr,
        QString* error = nullptr);
    static bool WriteRecord(const QString& robotName, const QString& encoded, QString* error = nullptr);
    static bool DisableLegacy(const QString& robotName, QString* error = nullptr);
    static bool ReadPending(const QString& robotName, bool& pending, QString* error = nullptr);
    static bool BeginOrUpdatePending(
        const QString& robotName,
        const QString& encoded,
        QString* error = nullptr);
    static bool WriteCompletedAndClearPending(
        const QString& robotName,
        const QString& encoded,
        QString* error = nullptr);
    static bool InvalidateIfNoPending(const QString& robotName, QString& error);
    static bool PersistentAdmissionBlocked(
        const QString& robotName,
        const QString& endpointIdentity,
        QString* reason = nullptr);
    // 专用 paused 恢复租约取得后调用：在同一存储锁内原子重读 marker + RecordV2，
    // 严格绑定 checkpoint/端点/程序/轨迹，避免确认页与实际恢复之间的 TOCTOU。
    static bool ReadPausedResumeBinding(
        const QString& robotName,
        const QString& endpointIdentity,
        const WeldResumePlanner::CheckpointRecord& expected,
        WeldResumePlanner::CheckpointRecord* current,
        QString* error = nullptr);
    // 专用恢复必须在硬件租约取得后再建立 Store 端点绑定。绑定期间同一物理端点
    // 只能有一个恢复者，且 index/legacy/case-sensitive scopes 中只能存在唯一 pending 记录。
    static bool AcquireExclusiveRecoveryBinding(
        const QString& robotName,
        const QString& endpointIdentity,
        const WeldResumePlanner::CheckpointRecord& expected,
        RobotRecoverySafetyPolicy::RecoveryBindingMode mode,
        RobotRecoverySafetyPolicy::ExclusiveRecoveryBinding* binding,
        QString* error = nullptr);
    static void ReleaseExclusiveRecoveryBinding(
        const QString& endpointIdentity,
        const QString& token);
    static bool RevalidateExclusiveRecoveryBinding(
        const RobotRecoverySafetyPolicy::ExclusiveRecoveryBinding& binding,
        QString* error = nullptr);
    static bool TransitionBoundRecordState(
        RobotRecoverySafetyPolicy::ExclusiveRecoveryBinding* binding,
        const QString& expectedState,
        const QString& newState,
        QString* error = nullptr);
    static bool TransitionRecordState(
        const QString& robotName,
        const QString& expectedCheckpointId,
        const QString& expectedState,
        const QString& newState,
        QString* error = nullptr);
};

#if !defined(WELD_SAFETY_STORE_STORAGE_ONLY_TEST)
// 不需要暂停 UI 的真实焊接入口（ProcessLoop/Virtual）使用同一持久终态协议。
class WeldSafetyRecoverySession final
{
public:
    WeldSafetyRecoverySession(
        RobotDriverAdaptor* driver,
        const T_PRECISE_MEASURE_PARAM& param);

    bool Prepare(
        const MeasureThenWeldService::WeldExecutionIdentity& identity,
        QString& error);
    bool Finish(const WeldExecutionTerminalResult& terminal, QString& error);

private:
    RobotDriverAdaptor* m_driver = nullptr;
    T_PRECISE_MEASURE_PARAM m_param;
    WeldResumePlanner::CheckpointRecord m_record;
    bool m_prepared = false;
};

using WeldSafetyRecoverySessionPtr = std::shared_ptr<WeldSafetyRecoverySession>;
#endif
