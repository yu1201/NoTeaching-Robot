#pragma once

#include "Const.h"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>

namespace RobotMotionTimeoutPolicy
{
    constexpr int kMotionTimeoutMs = 1800000;
    constexpr double kAdmissionSafetyFactor = 1.25;
    constexpr double kAdmissionMarginMs = 120000.0;

    inline bool IsFinitePose(const T_ROBOT_COORS& pose)
    {
        return std::isfinite(pose.dX) && std::isfinite(pose.dY) && std::isfinite(pose.dZ)
            && std::isfinite(pose.dRX) && std::isfinite(pose.dRY) && std::isfinite(pose.dRZ)
            && std::isfinite(pose.dBX) && std::isfinite(pose.dBY) && std::isfinite(pose.dBZ);
    }

    inline double TranslationDistanceMm(
        const T_ROBOT_COORS& start,
        const T_ROBOT_COORS& target)
    {
        const double dx = target.dX - start.dX;
        const double dy = target.dY - start.dY;
        const double dz = target.dZ - start.dZ;
        const double dbx = target.dBX - start.dBX;
        const double dby = target.dBY - start.dBY;
        const double dbz = target.dBZ - start.dBZ;
        return std::sqrt(dx * dx + dy * dy + dz * dz
            + dbx * dbx + dby * dby + dbz * dbz);
    }

    inline bool AdmitCartesianMove(
        const T_ROBOT_COORS& start,
        const T_ROBOT_COORS& target,
        double configuredSpeedMmPerMin,
        int& timeoutMs,
        std::string* error = nullptr)
    {
        timeoutMs = 0;
        if (!IsFinitePose(start) || !IsFinitePose(target)
            || !std::isfinite(configuredSpeedMmPerMin)
            || configuredSpeedMmPerMin <= 0.0)
        {
            if (error != nullptr)
            {
                *error = "直角运动准入失败：当前位置、目标或速度无效。";
            }
            return false;
        }

        const double distanceMm = TranslationDistanceMm(start, target);
        const double estimatedMs = distanceMm / configuredSpeedMmPerMin * 60000.0;
        const double requiredMs = estimatedMs * kAdmissionSafetyFactor + kAdmissionMarginMs;
        if (!std::isfinite(requiredMs) || requiredMs > static_cast<double>(kMotionTimeoutMs))
        {
            if (error != nullptr)
            {
                std::ostringstream oss;
                oss << "直角运动预计约" << (estimatedMs / 60000.0)
                    << "分钟，含安全裕量后超过30分钟上限；请提高速度、缩短距离或拆分运动。";
                *error = oss.str();
            }
            return false;
        }

        // 通过准入的计划运动统一保留完整30分钟；该值只作为异常运行的最后边界。
        timeoutMs = kMotionTimeoutMs;
        return true;
    }
}
