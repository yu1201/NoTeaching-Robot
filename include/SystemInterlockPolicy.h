#pragma once

#include <array>
#include <cstddef>

enum class SystemInterlock : std::size_t
{
    SingleProcess,
    DriverEndpointIdentity,
    AccountSession,
    StateTransition,
    SafeRetreatPending,
    VerifiedStop,
    ExclusiveOperationLease,
    MotionLeaseOwnership,
    MotionTerminal,
    RecoveryIdentity,
    Count
};

constexpr std::size_t SystemInterlockCount = static_cast<std::size_t>(SystemInterlock::Count);

struct SystemInterlockPolicy
{
    // 每项独立存储。未知/缺失的项默认开启；旧总开关不迁移成全部关闭。
    std::array<bool, SystemInterlockCount> enabled{true, true, true, true, true, true, true, true, true, true};
    bool IsEnabled(SystemInterlock gate) const { return enabled.at(static_cast<std::size_t>(gate)); }
    void SetEnabled(SystemInterlock gate, bool value) { enabled.at(static_cast<std::size_t>(gate)) = value; }
    inline static constexpr std::array<const char*, SystemInterlockCount> keys{
        "SystemInterlocks/SingleProcessEnabled",
        "SystemInterlocks/DriverEndpointIdentityEnabled",
        "SystemInterlocks/AccountSessionEnabled",
        "SystemInterlocks/StateTransitionEnabled",
        "SystemInterlocks/SafeRetreatPendingEnabled",
        "SystemInterlocks/VerifiedStopEnabled",
        "SystemInterlocks/ExclusiveOperationLeaseEnabled",
        "SystemInterlocks/MotionLeaseOwnershipEnabled",
        "SystemInterlocks/MotionTerminalEnabled",
        "SystemInterlocks/RecoveryIdentityEnabled"
    };
};
