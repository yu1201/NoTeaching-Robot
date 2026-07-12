#include "RobotOperationLease.h"

#if !defined(ROBOT_OPERATION_LEASE_TEST_STUB_DRIVER)
#include "RobotDriverAdaptor.h"
#include "WeldSafetyRecoveryStore.h"
#endif

#include <QHostAddress>
#include <QStringList>

#include <algorithm>
#include <atomic>
#include <map>
#include <mutex>
#include <utility>

namespace
{
struct ActiveOperation
{
    std::uint64_t token = 0;
    QString owner;
    const RobotDriverAdaptor* acquiringDriver = nullptr;
    bool cancellationRequested = false;
    bool motionCompletionPending = false;
};

struct UnresolvedStop
{
    const RobotDriverAdaptor* driver = nullptr;
    // 原租约是否已发送过运动且终态仍未确认；租约析构后仍必须保留，
    // 这样后续红色 STOP 重试不会把它误当成普通配置租约而跳过真实中止。
    bool motionCompletionPending = false;
};

std::mutex g_operationMutex;
// 键为规范化 TCP 端点；只有端点不可用时才退回 driver 指针身份。
std::map<QString, ActiveOperation> g_activeOperations;
// STOP 发出后直到机器人侧真实停机回读成功前持续闭锁；即使原流程先释放租约也不清除。
std::map<QString, UnresolvedStop> g_unresolvedStops;
bool g_newOperationsAllowed = true;
QString g_newOperationsBlockedReason;
std::map<RobotOperationLease::NewOperationBlockToken, QString> g_newOperationBlocks;
std::atomic<std::uint64_t> g_nextOperationToken{ 1 };
std::atomic<std::uint64_t> g_nextOperationBlockToken{ 1 };

QString PointerIdentity(const RobotDriverAdaptor* driver)
{
    return QStringLiteral("pointer:%1")
        .arg(static_cast<qulonglong>(reinterpret_cast<quintptr>(driver)), 0, 16);
}

QString NormalizeSocketHost(const std::string& socketIp)
{
    QString host = QString::fromStdString(socketIp).trimmed().toCaseFolded();
    if (host.size() >= 2 && host.front() == QLatin1Char('[') && host.back() == QLatin1Char(']'))
    {
        host = host.mid(1, host.size() - 2).trimmed();
    }
    while (host.size() > 1 && host.endsWith(QLatin1Char('.')))
    {
        host.chop(1);
    }
    if (host.isEmpty())
    {
        return {};
    }
    for (const QChar ch : host)
    {
        if (ch.isSpace() || ch.isNull())
        {
            return {};
        }
    }

    // 同一个 IPv4/IPv6 地址可能有多种文本写法；解析成功时统一为 Qt 的规范形式。
    QHostAddress address;
    if (address.setAddress(host))
    {
        host = address.toString().toCaseFolded();
    }
    return host;
}

QString ResolveDriverIdentity(const RobotDriverAdaptor* driver)
{
    if (driver == nullptr)
    {
        return {};
    }

    const QString persistentIdentity = RobotOperationLease::PersistentEndpointIdentity(driver);
    return persistentIdentity.isEmpty() ? PointerIdentity(driver) : persistentIdentity;
}
}

QString RobotOperationLease::PersistentEndpointIdentity(const RobotDriverAdaptor* driver)
{
    if (driver == nullptr)
    {
        return {};
    }
    const QString host = NormalizeSocketHost(driver->m_sSocketIP);
    // 先判断 host，避免读取尚未装载配置的 driver 中可能未初始化的端口字段。
    if (host.isEmpty())
    {
        return {};
    }
    const int port = driver->m_nSocketPort;
    if (port <= 0 || port > 65535)
    {
        return {};
    }
    return QStringLiteral("tcp:[%1]:%2").arg(host).arg(port);
}

RobotOperationLease::RobotOperationLease(
    const RobotDriverAdaptor* driver,
    QString identityKey,
    std::uint64_t token,
    QString owner)
    : m_driver(driver)
    , m_identityKey(std::move(identityKey))
    , m_token(token)
    , m_owner(std::move(owner))
{
}

void RobotOperationLease::SetNewOperationsAllowed(
    bool allowed,
    const QString& blockedReason)
{
    std::lock_guard<std::mutex> lock(g_operationMutex);
    g_newOperationsAllowed = allowed;
    g_newOperationsBlockedReason = allowed
        ? QString()
        : blockedReason.trimmed();
    if (!allowed && g_newOperationsBlockedReason.isEmpty())
    {
        g_newOperationsBlockedReason =
            QStringLiteral("账号会话未通过，禁止开始新的机器人硬件操作。");
    }
}

bool RobotOperationLease::NewOperationsAllowed()
{
    std::lock_guard<std::mutex> lock(g_operationMutex);
    return g_newOperationsAllowed && g_newOperationBlocks.empty();
}

RobotOperationLease::NewOperationBlockToken RobotOperationLease::AddNewOperationsBlock(
    const QString& blockedReason)
{
    const NewOperationBlockToken token =
        g_nextOperationBlockToken.fetch_add(1, std::memory_order_relaxed);
    QString reason = blockedReason.trimmed();
    if (reason.isEmpty())
    {
        reason = QStringLiteral("系统正在切换状态，禁止开始新的机器人硬件操作。");
    }
    std::lock_guard<std::mutex> lock(g_operationMutex);
    g_newOperationBlocks.emplace(token, std::move(reason));
    return token;
}

void RobotOperationLease::RemoveNewOperationsBlock(NewOperationBlockToken token)
{
    if (token == 0)
    {
        return;
    }
    std::lock_guard<std::mutex> lock(g_operationMutex);
    g_newOperationBlocks.erase(token);
}

RobotOperationLease::Ptr RobotOperationLease::TryAcquire(
    const RobotDriverAdaptor* driver,
    const QString& requestedOwner,
    QString* reason)
{
    return TryAcquireImpl(driver, requestedOwner, false, reason);
}

RobotOperationLease::Ptr RobotOperationLease::TryAcquireSafetyRecovery(
    const RobotDriverAdaptor* driver,
    const QString& requestedOwner,
    const WeldResumePlanner::CheckpointRecord& expectedRecord,
    RobotRecoverySafetyPolicy::ExclusiveRecoveryBinding* binding,
    QString* reason)
{
    if (binding == nullptr)
    {
        if (reason != nullptr)
        {
            *reason = QStringLiteral("安全回撤恢复缺少 Store 独占绑定输出。");
        }
        return {};
    }
    const Ptr lease = TryAcquireImpl(driver, requestedOwner, true, reason);
    if (!lease)
    {
        return {};
    }
#if !defined(ROBOT_OPERATION_LEASE_TEST_STUB_DRIVER)
    const QString robotName = QString::fromStdString(driver->m_sRobotName).trimmed();
    const QString endpoint = PersistentEndpointIdentity(driver);
    if (!WeldSafetyRecoveryStore::AcquireExclusiveRecoveryBinding(
            robotName,
            endpoint,
            expectedRecord,
            RobotRecoverySafetyPolicy::RecoveryBindingMode::SafeRetreat,
            binding,
            reason))
    {
        return {};
    }
    lease->m_storeRecoveryEndpoint = binding->endpointIdentity;
    lease->m_storeRecoveryToken = binding->token;
    return lease;
#else
    Q_UNUSED(expectedRecord);
    if (reason != nullptr)
    {
        *reason = QStringLiteral("测试桩不支持持久安全回撤绑定。");
    }
    return {};
#endif
}

RobotOperationLease::Ptr RobotOperationLease::TryAcquirePausedResume(
    const RobotDriverAdaptor* driver,
    const QString& requestedOwner,
    const WeldResumePlanner::CheckpointRecord& expectedRecord,
    RobotRecoverySafetyPolicy::ExclusiveRecoveryBinding* binding,
    QString* reason)
{
    if (binding == nullptr)
    {
        if (reason != nullptr)
        {
            *reason = QStringLiteral("paused 恢复缺少原子回读输出，拒绝取得硬件租约。");
        }
        return {};
    }
    const Ptr lease = TryAcquireImpl(driver, requestedOwner, true, reason);
    if (!lease)
    {
        return {};
    }
#if !defined(ROBOT_OPERATION_LEASE_TEST_STUB_DRIVER)
    const QString robotName = QString::fromStdString(driver->m_sRobotName).trimmed();
    const QString endpoint = PersistentEndpointIdentity(driver);
    if (!WeldSafetyRecoveryStore::AcquireExclusiveRecoveryBinding(
            robotName,
            endpoint,
            expectedRecord,
            RobotRecoverySafetyPolicy::RecoveryBindingMode::PausedResume,
            binding,
            reason))
    {
        return {};
    }
    lease->m_storeRecoveryEndpoint = binding->endpointIdentity;
    lease->m_storeRecoveryToken = binding->token;
    return lease;
#else
    Q_UNUSED(expectedRecord);
    if (reason != nullptr)
    {
        *reason = QStringLiteral("测试桩不支持持久 paused 恢复绑定。");
    }
    return {};
#endif
}

RobotOperationLease::Ptr RobotOperationLease::TryAcquireImpl(
    const RobotDriverAdaptor* driver,
    const QString& requestedOwner,
    bool allowPersistentRecovery,
    QString* reason)
{
    if (reason != nullptr)
    {
        reason->clear();
    }
    if (driver == nullptr)
    {
        if (reason != nullptr)
        {
            *reason = QStringLiteral("机器人驱动不可用，无法取得硬件操作租约。");
        }
        return {};
    }

    const QString owner = requestedOwner.trimmed().isEmpty()
        ? QStringLiteral("未命名硬件操作")
        : requestedOwner.trimmed();
    const QString identityKey = ResolveDriverIdentity(driver);
#if !defined(ROBOT_OPERATION_LEASE_TEST_STUB_DRIVER)
    if (!allowPersistentRecovery)
    {
        const QString robotName = QString::fromStdString(driver->m_sRobotName).trimmed();
        QString persistentReason;
        if (WeldSafetyRecoveryStore::PersistentAdmissionBlocked(
            robotName, identityKey, &persistentReason))
        {
            if (reason != nullptr)
            {
                *reason = persistentReason;
            }
            return {};
        }
    }
#else
    Q_UNUSED(allowPersistentRecovery);
#endif
    // 先构造未注册租约以保证异常安全：map 插入失败时不会留下幽灵占用。
    Ptr lease(new RobotOperationLease(driver, identityKey, 0, owner));
    std::lock_guard<std::mutex> lock(g_operationMutex);
    if (!g_newOperationsAllowed || !g_newOperationBlocks.empty())
    {
        if (reason != nullptr)
        {
            if (!g_newOperationsAllowed)
            {
                *reason = g_newOperationsBlockedReason.isEmpty()
                    ? QStringLiteral("账号会话未通过，禁止开始新的机器人硬件操作。")
                    : g_newOperationsBlockedReason;
            }
            else
            {
                *reason = g_newOperationBlocks.cbegin()->second;
            }
        }
        return {};
    }
    const auto unresolved = std::find_if(
        g_unresolvedStops.cbegin(),
        g_unresolvedStops.cend(),
        [driver, &identityKey](const auto& stop)
        {
            return stop.first == identityKey || stop.second.driver == driver;
        });
    if (unresolved != g_unresolvedStops.cend())
    {
        if (reason != nullptr)
        {
            *reason = QStringLiteral("当前机器人上次安全停机尚未得到真实确认，禁止开始“%1”。").arg(owner);
        }
        return {};
    }
    // 同一 driver 在持租约期间即使配置字段被改写，也绝不能借新端点 key 二次取得租约。
    for (const auto& operation : g_activeOperations)
    {
        if (operation.second.acquiringDriver == driver)
        {
            if (reason != nullptr)
            {
                *reason = QStringLiteral("当前机器人正在执行“%1”，不能同时开始“%2”。")
                    .arg(operation.second.owner, owner);
            }
            return {};
        }
    }
    const auto active = g_activeOperations.find(identityKey);
    if (active != g_activeOperations.end())
    {
        if (reason != nullptr)
        {
            *reason = QStringLiteral("当前机器人正在执行“%1”，不能同时开始“%2”。")
                .arg(active->second.owner, owner);
        }
        return {};
    }

    const std::uint64_t token = g_nextOperationToken.fetch_add(1, std::memory_order_relaxed);
    g_activeOperations.emplace(identityKey, ActiveOperation{ token, owner, driver, false, false });
    lease->m_token = token;
    return lease;
}

QString RobotOperationLease::CurrentOwner(const RobotDriverAdaptor* driver)
{
    if (driver == nullptr)
    {
        return {};
    }
    const QString identityKey = ResolveDriverIdentity(driver);
    std::lock_guard<std::mutex> lock(g_operationMutex);
    // 同一指针优先使用取得租约时冻结的记录；即使配置字段意外变化也仍能查询。
    for (const auto& operation : g_activeOperations)
    {
        if (operation.second.acquiringDriver == driver)
        {
            return operation.second.owner;
        }
    }
    const auto active = g_activeOperations.find(identityKey);
    return active == g_activeOperations.end() ? QString() : active->second.owner;
}

bool RobotOperationLease::RequestCancellation(
    const RobotDriverAdaptor* driver,
    QString* cancelledOwner)
{
    if (cancelledOwner != nullptr)
    {
        cancelledOwner->clear();
    }
    if (driver == nullptr)
    {
        return false;
    }
    const QString identityKey = ResolveDriverIdentity(driver);
    std::lock_guard<std::mutex> lock(g_operationMutex);
    for (auto& operation : g_activeOperations)
    {
        if (operation.second.acquiringDriver == driver || operation.first == identityKey)
        {
            operation.second.cancellationRequested = true;
            g_unresolvedStops[operation.first] = UnresolvedStop{
                operation.second.acquiringDriver,
                operation.second.motionCompletionPending
            };
            if (cancelledOwner != nullptr)
            {
                *cancelledOwner = operation.second.owner;
            }
            return true;
        }
    }
    for (const auto& unresolved : g_unresolvedStops)
    {
        if (unresolved.second.driver == driver || unresolved.first == identityKey)
        {
            if (cancelledOwner != nullptr)
            {
                *cancelledOwner = QStringLiteral("安全停机未确认");
            }
            return true;
        }
    }
    return false;
}

std::vector<RobotOperationLease::CancellationTarget> RobotOperationLease::LatchGlobalCancellation(
    const RobotDriverAdaptor* requiredDriver,
    const QString& requiredOwner)
{
    std::lock_guard<std::mutex> lock(g_operationMutex);
    std::vector<CancellationTarget> targets;
    const auto appendTarget = [&targets](const RobotDriverAdaptor* driver, const QString& owner)
    {
        if (driver == nullptr)
        {
            return;
        }
        const auto existing = std::find_if(
            targets.cbegin(),
            targets.cend(),
            [driver](const CancellationTarget& target) { return target.driver == driver; });
        if (existing == targets.cend())
        {
            targets.push_back(CancellationTarget{ driver, owner });
        }
    };

    for (auto& operation : g_activeOperations)
    {
        operation.second.cancellationRequested = true;
        g_unresolvedStops[operation.first] = UnresolvedStop{
            operation.second.acquiringDriver,
            operation.second.motionCompletionPending
        };
        appendTarget(operation.second.acquiringDriver, operation.second.owner);
    }
    for (const auto& unresolved : g_unresolvedStops)
    {
        appendTarget(unresolved.second.driver, QStringLiteral("安全停机未确认"));
    }

    if (requiredDriver != nullptr)
    {
        const QString requiredIdentity = ResolveDriverIdentity(requiredDriver);
        bool alreadyRepresented = false;
        for (const auto& unresolved : g_unresolvedStops)
        {
            if (unresolved.second.driver == requiredDriver || unresolved.first == requiredIdentity)
            {
                alreadyRepresented = true;
                break;
            }
        }
        if (!alreadyRepresented)
        {
            const QString owner = requiredOwner.trimmed().isEmpty()
                ? QStringLiteral("安全停止当前机器人")
                : requiredOwner.trimmed();
            g_unresolvedStops[requiredIdentity] = UnresolvedStop{ requiredDriver, false };
            appendTarget(requiredDriver, owner);
        }
    }
    return targets;
}

bool RobotOperationLease::ConfirmCancellationHandled(const RobotDriverAdaptor* driver)
{
    if (driver == nullptr)
    {
        return false;
    }
    const QString identityKey = ResolveDriverIdentity(driver);
    std::lock_guard<std::mutex> lock(g_operationMutex);
    std::size_t erased = 0;
    for (auto unresolved = g_unresolvedStops.begin(); unresolved != g_unresolvedStops.end();)
    {
        if (unresolved->second.driver == driver || unresolved->first == identityKey)
        {
            unresolved = g_unresolvedStops.erase(unresolved);
            ++erased;
        }
        else
        {
            ++unresolved;
        }
    }
    if (erased > 0)
    {
        // Confirm 的契约本身就是“机器人侧真实停止已回读”；与 unresolved 同一把锁内
        // 原子清 pending，避免 stop 成功与原 lease 析构交错时重新制造闭锁。
        for (auto& operation : g_activeOperations)
        {
            if (operation.second.acquiringDriver == driver || operation.first == identityKey)
            {
                operation.second.motionCompletionPending = false;
            }
        }
    }
    return erased > 0;
}

bool RobotOperationLease::IsCancellationRequested(const RobotDriverAdaptor* driver)
{
    if (driver == nullptr)
    {
        return false;
    }
    const QString identityKey = ResolveDriverIdentity(driver);
    std::lock_guard<std::mutex> lock(g_operationMutex);
    for (const auto& operation : g_activeOperations)
    {
        if ((operation.second.acquiringDriver == driver || operation.first == identityKey)
            && operation.second.cancellationRequested)
        {
            return true;
        }
    }
    for (const auto& unresolved : g_unresolvedStops)
    {
        if (unresolved.second.driver == driver || unresolved.first == identityKey)
        {
            return true;
        }
    }
    return false;
}

bool RobotOperationLease::MarkMotionStarted(
    const RobotDriverAdaptor* driver,
    bool allowExistingPending,
    QString* reason)
{
    if (reason != nullptr)
    {
        reason->clear();
    }
    if (driver == nullptr)
    {
        if (reason != nullptr)
        {
            *reason = QStringLiteral("机器人驱动不可用，拒绝登记运动命令。");
        }
        return false;
    }

    const QString identityKey = ResolveDriverIdentity(driver);
    std::lock_guard<std::mutex> lock(g_operationMutex);
    auto active = std::find_if(
        g_activeOperations.begin(),
        g_activeOperations.end(),
        [driver, &identityKey](const auto& operation)
        {
            return operation.second.acquiringDriver == driver || operation.first == identityKey;
        });
    if (active == g_activeOperations.end())
    {
        if (reason != nullptr)
        {
            *reason = QStringLiteral("当前运动命令没有持有机器人硬件操作租约，已拒绝下发。");
        }
        return false;
    }
    if (active->second.cancellationRequested)
    {
        if (reason != nullptr)
        {
            *reason = QStringLiteral("机器人安全停止已锁存，已拒绝下发新的运动命令。");
        }
        return false;
    }
    if (active->second.motionCompletionPending && !allowExistingPending)
    {
        if (reason != nullptr)
        {
            *reason = QStringLiteral("上一条机器人运动尚未得到稳定终态确认，已拒绝启动下一条运动。");
        }
        return false;
    }
    active->second.motionCompletionPending = true;
    return true;
}

bool RobotOperationLease::MarkMotionCompleted(const RobotDriverAdaptor* driver)
{
    if (driver == nullptr)
    {
        return false;
    }
    const QString identityKey = ResolveDriverIdentity(driver);
    std::lock_guard<std::mutex> lock(g_operationMutex);
    bool found = false;
    auto active = std::find_if(
        g_activeOperations.begin(),
        g_activeOperations.end(),
        [driver, &identityKey](const auto& operation)
        {
            return operation.second.acquiringDriver == driver || operation.first == identityKey;
        });
    if (active != g_activeOperations.end())
    {
        active->second.motionCompletionPending = false;
        found = true;
    }
    // STOP 可能在本轮完成检查之后、Mark 之前锁存 unresolved=true。
    // 可信终态确认必须同步把 provenance 降为 false，但保留取消条目等待 Confirm。
    for (auto& unresolved : g_unresolvedStops)
    {
        if (unresolved.second.driver == driver || unresolved.first == identityKey)
        {
            unresolved.second.motionCompletionPending = false;
            found = true;
        }
    }
    return found;
}

bool RobotOperationLease::MotionCompletionPending(const RobotDriverAdaptor* driver)
{
    if (driver == nullptr)
    {
        return false;
    }
    const QString identityKey = ResolveDriverIdentity(driver);
    std::lock_guard<std::mutex> lock(g_operationMutex);
    const auto active = std::find_if(
        g_activeOperations.cbegin(),
        g_activeOperations.cend(),
        [driver, &identityKey](const auto& operation)
        {
            return operation.second.acquiringDriver == driver || operation.first == identityKey;
        });
    if (active != g_activeOperations.cend() && active->second.motionCompletionPending)
    {
        return true;
    }
    const auto unresolved = std::find_if(
        g_unresolvedStops.cbegin(),
        g_unresolvedStops.cend(),
        [driver, &identityKey](const auto& stop)
        {
            return (stop.second.driver == driver || stop.first == identityKey)
                && stop.second.motionCompletionPending;
        });
    return unresolved != g_unresolvedStops.cend();
}

bool RobotOperationLease::StopAndConfirmUnverifiedMotion(RobotDriverAdaptor* driver)
{
    if (driver == nullptr)
    {
        return false;
    }

    // RequestCancellation 只持 registry mutex；必须在它返回、锁已释放后才进入驱动，
    // 避免 registry -> socket/SDK/join 的反向锁序。
    QString cancelledOwner;
    RequestCancellation(driver, &cancelledOwner);
    const bool stopOk = driver->AbortCurrentProgramSafely();
    if (!stopOk)
    {
        return false;
    }
    MarkMotionCompleted(driver);
    // 并发 STOP 可能已经先清除；第二次返回 false 不改变“驱动已确认停止”的事实。
    ConfirmCancellationHandled(driver);
    return true;
}

bool RobotOperationLease::Matches(const RobotDriverAdaptor* driver) const
{
    if (driver == nullptr)
    {
        return false;
    }
    return driver == m_driver || ResolveDriverIdentity(driver) == m_identityKey;
}

bool RobotOperationLease::CancellationRequested() const
{
    return IsCancellationRequested(m_driver);
}

bool RobotOperationLease::AnyActive()
{
    std::lock_guard<std::mutex> lock(g_operationMutex);
    return !g_activeOperations.empty() || !g_unresolvedStops.empty();
}

QString RobotOperationLease::ActiveSummary()
{
    std::lock_guard<std::mutex> lock(g_operationMutex);
    QStringList owners;
    owners.reserve(static_cast<int>(g_activeOperations.size()));
    for (const auto& active : g_activeOperations)
    {
        if (!owners.contains(active.second.owner))
        {
            owners.push_back(active.second.owner);
        }
    }
    if (!g_unresolvedStops.empty())
    {
        owners.push_back(QStringLiteral("安全停机未确认"));
    }
    return owners.join(QStringLiteral("、"));
}

RobotOperationLease::~RobotOperationLease()
{
#if !defined(ROBOT_OPERATION_LEASE_TEST_STUB_DRIVER)
    if (!m_storeRecoveryEndpoint.isEmpty() && !m_storeRecoveryToken.isEmpty())
    {
        WeldSafetyRecoveryStore::ReleaseExclusiveRecoveryBinding(
            m_storeRecoveryEndpoint, m_storeRecoveryToken);
    }
#endif
    if (m_driver == nullptr || m_identityKey.isEmpty() || m_token == 0)
    {
        return;
    }
    {
        std::lock_guard<std::mutex> lock(g_operationMutex);
        const auto active = g_activeOperations.find(m_identityKey);
        if (active != g_activeOperations.end() && active->second.token == m_token)
        {
            if (active->second.motionCompletionPending)
            {
                // 即使原工作线程因超时/断线先退出，也必须留下跨租约闭锁。
                // 网络/SDK 停机只能由显式失败路径在 registry 锁外执行；析构绝不阻塞或进驱动锁。
                g_unresolvedStops[m_identityKey] = UnresolvedStop{ m_driver, true };
            }
            g_activeOperations.erase(active);
        }
    }
}
