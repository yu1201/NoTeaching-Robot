#pragma once

#include "groove/framebuffer.h"

#include <cstdint>
#include <deque>
#include <mutex>
#include <vector>

class CameraFrameCache
{
public:
    struct TimedFrame
    {
        std::uint64_t sequence = 0;
        qint64 cameraTimestampUs = 0;
        qint64 receiveTimestampUs = 0;
        cv::Point3d targetPoint;
        QString errorMessage;
    };

    CameraFrameCache() = default;
    ~CameraFrameCache();
    CameraFrameCache(const CameraFrameCache&) = delete;
    CameraFrameCache& operator=(const CameraFrameCache&) = delete;

    void Start();
    void Stop();
    void Clear();
    void AppendFrame(const udpDataShow& frame);

    std::uint64_t Mark() const;
    bool Latest(udpDataShow& frame) const;
    std::vector<udpDataShow> FramesBetween(std::uint64_t beginExclusive, std::uint64_t endInclusive) const;
    std::vector<TimedFrame> TimedFramesBetween(std::uint64_t beginExclusive, std::uint64_t endInclusive) const;
    int CachedCount() const;

private:
    struct CachedFrame
    {
        std::uint64_t sequence = 0;
        qint64 receiveTimestampUs = 0;
        udpDataShow frame;
    };

    void StoreFrame(const udpDataShow& frame);

    mutable std::mutex m_mutex;
    std::deque<CachedFrame> m_frames;
    std::uint64_t m_nextSequence = 0;
    std::size_t m_maxCachedFrames = 2000;
};
