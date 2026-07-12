#pragma once

#include <QtGlobal>

#include <atomic>

// UI-thread-owned lifecycle state for the one OnlineServices remote worker. Worker
// threads only read the atomic cancel/generation fields. A normal page close uses a
// reversible cancel cycle; application exit/destruction permanently prevents restart.
class RemoteWorkerLifecycle
{
public:
    quint64 BeginWorker() noexcept
    {
        if (!CanStart())
        {
            return 0;
        }
        m_cancel.store(false);
        quint64 generation = m_generation.fetch_add(1) + 1;
        if (generation == 0)
        {
            generation = m_generation.fetch_add(1) + 1;
        }
        return generation;
    }

    void BeginCancel(bool permanentShutdown) noexcept
    {
        m_cancelCycle = true;
        m_permanentShutdown = m_permanentShutdown || permanentShutdown;
        m_cancel.store(true);
        m_generation.fetch_add(1);
    }

    void FinishCancel() noexcept
    {
        m_cancelCycle = false;
        if (!m_permanentShutdown)
        {
            m_cancel.store(false);
        }
    }

    bool CanStart() const noexcept
    {
        return !m_cancelCycle && !m_permanentShutdown;
    }

    bool IsCancelled(quint64 generation) const noexcept
    {
        return m_cancel.load() || generation != m_generation.load();
    }

    std::atomic_bool* CancelFlag() noexcept
    {
        return &m_cancel;
    }

private:
    std::atomic_bool m_cancel{ false };
    std::atomic<quint64> m_generation{ 0 };
    bool m_cancelCycle = false;
    bool m_permanentShutdown = false;
};
