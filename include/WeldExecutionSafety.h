#pragma once

#include "Const.h"

#include <QString>

enum class WeldExecutionTerminalState
{
    Incomplete = 0,
    ProgramCompletedUnretracted = 1,
    SafelyRetracted = 2
};

struct WeldExecutionTerminalResult
{
    WeldExecutionTerminalState state = WeldExecutionTerminalState::Incomplete;
    bool programStartAttempted = false;
    QString reason;
    QString observedAtUtc;
    T_ROBOT_COORS requiredSafePose;
    T_ROBOT_COORS observedTerminalPose;
    bool observedTerminalPoseValid = false;
    double safeMoveSpeedMmPerMin = 0.0;
};

namespace WeldExecutionSafety
{
    // 焊后“取消”可能表示现场存在障碍，不能把安全回撤当成绕过取消的新运动；
    // 只有明确确认且 STOP 未锁存时才允许自动回撤，其他情况保持 unretracted 门禁。
    inline bool ShouldAttemptMandatoryRetreat(bool postWeldConfirmed, bool stopLatched)
    {
        return postWeldConfirmed && !stopLatched;
    }

    inline WeldExecutionTerminalState ResolveTerminalState(
        bool programCompleted,
        bool retreatAttempted,
        bool retreatVerified)
    {
        if (!programCompleted)
        {
            return WeldExecutionTerminalState::Incomplete;
        }
        return retreatAttempted && retreatVerified
            ? WeldExecutionTerminalState::SafelyRetracted
            : WeldExecutionTerminalState::ProgramCompletedUnretracted;
    }
}
