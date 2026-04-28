#pragma once

#include "groove/framebuffer.h"

#include <atomic>
#include <cstdint>
#include <deque>
#include <mutex>
#include <thread>
#include <vector>

class CameraFrameCache
{
public:
    static CameraFrameCache& Instance();

    void Start();
    void Stop();
    void Clear();

    std::uint64_t Mark() const;
    bool Latest(udpDataShow& frame) const;
    std::vector<udpDataShow> FramesBetween(std::uint64_t beginExclusive, std::uint64_t endInclusive) const;
    int CachedCount() const;

private:
    struct CachedFrame
    {
        std::uint64_t sequence = 0;
        udpDataShow frame;
    };

    CameraFrameCache() = default;
    ~CameraFrameCache();
    CameraFrameCache(const CameraFrameCache&) = delete;
    CameraFrameCache& operator=(const CameraFrameCache&) = delete;

    void ThreadMain();
    void StoreFrame(const udpDataShow& frame);

    mutable std::mutex m_mutex;
    std::deque<CachedFrame> m_frames;
    std::thread m_thread;
    std::atomic_bool m_running{ false };
    std::uint64_t m_nextSequence = 0;
    std::size_t m_maxCachedFrames = 50000;
};
