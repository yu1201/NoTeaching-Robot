#pragma once

#include <QString>

#include <cstdint>
#include <memory>
#include <vector>

#if defined(ROBOT_OPERATION_LEASE_TEST_STUB_DRIVER)
#include <string>

// 独立并发 smoke 只需要端点字段，避免为小测试链接完整机器人 SDK。
class RobotDriverAdaptor final
{
public:
    std::string m_sSocketIP;
    int m_nSocketPort = 0;
    bool AbortCurrentProgramSafely() { return false; }
};
#else
class RobotDriverAdaptor;
#endif

// 每台机器人同一时刻只允许一个会改变硬件状态的高层操作持有租约。
// 租约覆盖完整流程（下发、等待完成、收尾），析构自动释放；不同机器人可并行。
class RobotOperationLease final
{
public:
    using Ptr = std::shared_ptr<RobotOperationLease>;

    struct CancellationTarget
    {
        const RobotDriverAdaptor* driver = nullptr;
        QString owner;
    };

    static Ptr TryAcquire(
        const RobotDriverAdaptor* driver,
        const QString& requestedOwner,
        QString* reason = nullptr);
    // 交互式账号会话失效时，统一禁止所有模块启动新的机器人硬件操作。
    // 已持有的租约不受影响，以便原流程仍可执行安全停止和清理。
    static void SetNewOperationsAllowed(
        bool allowed,
        const QString& blockedReason = QString());
    static bool NewOperationsAllowed();
    // 临时阻止新操作的独立 token。释放 token 只移除自己的阻塞，不会把账号会话等
    // 其他 owner 设置的全局禁止误改为允许。
    using NewOperationBlockToken = std::uint64_t;
    static NewOperationBlockToken AddNewOperationsBlock(const QString& blockedReason);
    static void RemoveNewOperationsBlock(NewOperationBlockToken token);
    static QString CurrentOwner(const RobotDriverAdaptor* driver);
    // 可持久化的物理 TCP 端点；端点无效时返回空，绝不返回只在本进程有效的 pointer 身份。
    static QString PersistentEndpointIdentity(const RobotDriverAdaptor* driver);
    // 安全停止不抢占当前租约，而是给当前高层流程锁存取消状态。
    static bool RequestCancellation(const RobotDriverAdaptor* driver, QString* cancelledOwner = nullptr);
    // 在注册表锁内原子地取消全部活动流程，并把 requiredDriver（通常为主页当前机器人）
    // 一并锁存。返回的 driver 在逐个 ConfirmCancellationHandled 前受全局停机闭锁保护。
    static std::vector<CancellationTarget> LatchGlobalCancellation(
        const RobotDriverAdaptor* requiredDriver = nullptr,
        const QString& requiredOwner = QString());
    // 仅在机器人侧真实停止/终止回读成功后调用；失败时保留跨租约停机闭锁。
    static bool ConfirmCancellationHandled(const RobotDriverAdaptor* driver);
    static bool IsCancellationRequested(const RobotDriverAdaptor* driver);
    // 运动命令在真正发送前登记“终态待确认”。正常完成必须由稳定终态回读调用
    // MarkMotionCompleted；否则租约析构会自动转为 sticky stop，且异常路径必须调用
    // StopAndConfirmUnverifiedMotion 尝试真实中止。
    // allowExistingPending 仅用于同一已跟踪程序的暂停后继续，不能用于启动新任务。
    static bool MarkMotionStarted(
        const RobotDriverAdaptor* driver,
        bool allowExistingPending = false,
        QString* reason = nullptr);
    static bool MarkMotionCompleted(const RobotDriverAdaptor* driver);
    static bool MotionCompletionPending(const RobotDriverAdaptor* driver);
    // 异常等待的统一 fail-closed 收尾：先锁存取消，再调用驱动的可验证中止；
    // 只有真实中止成功才清 pending 与 unresolved。可与主页 STOP 并发、结果幂等。
    static bool StopAndConfirmUnverifiedMotion(RobotDriverAdaptor* driver);
    static bool AnyActive();
    static QString ActiveSummary();

    ~RobotOperationLease();

    RobotOperationLease(const RobotOperationLease&) = delete;
    RobotOperationLease& operator=(const RobotOperationLease&) = delete;

    QString Owner() const { return m_owner; }
    bool CancellationRequested() const;
    // 相同 driver 指针或规范化后指向同一 TCP 端点时均视为匹配。
    bool Matches(const RobotDriverAdaptor* driver) const;

private:
    RobotOperationLease(
        const RobotDriverAdaptor* driver,
        QString identityKey,
        std::uint64_t token,
        QString owner);

    const RobotDriverAdaptor* m_driver = nullptr;
    // 析构必须使用取得租约时冻结的 identity；不能受后续配置字段变化影响。
    QString m_identityKey;
    std::uint64_t m_token = 0;
    QString m_owner;
};
