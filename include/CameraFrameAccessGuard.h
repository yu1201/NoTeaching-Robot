#pragma once

#include <atomic>

namespace CameraFrameAccess
{
inline std::atomic_bool& MeasureThenWeldExclusiveFlag()
{
    static std::atomic_bool flag{ false };
    return flag;
}

inline bool IsMeasureThenWeldExclusive()
{
    return MeasureThenWeldExclusiveFlag().load(std::memory_order_acquire);
}

inline bool TryBeginMeasureThenWeldExclusive()
{
    bool expected = false;
    return MeasureThenWeldExclusiveFlag().compare_exchange_strong(
        expected,
        true,
        std::memory_order_acq_rel,
        std::memory_order_acquire);
}

inline void EndMeasureThenWeldExclusive()
{
    MeasureThenWeldExclusiveFlag().store(false, std::memory_order_release);
}

class ScopedMeasureThenWeldExclusive
{
public:
    ScopedMeasureThenWeldExclusive()
        : m_acquired(TryBeginMeasureThenWeldExclusive())
    {
    }

    ~ScopedMeasureThenWeldExclusive()
    {
        if (m_acquired)
        {
            EndMeasureThenWeldExclusive();
        }
    }

    bool acquired() const
    {
        return m_acquired;
    }

private:
    bool m_acquired = false;
};
}
