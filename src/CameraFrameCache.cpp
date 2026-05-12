#include "CameraFrameCache.h"

#include <chrono>

namespace
{
qint64 CameraFrameCacheSteadyNowUs()
{
    return static_cast<qint64>(std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}
}

CameraFrameCache::~CameraFrameCache()
{
    Stop();
}

void CameraFrameCache::Start()
{
    // Camera frames are pushed directly by the per-robot UDP workers.
}

void CameraFrameCache::Stop()
{
    // No shared queue thread is used anymore.
}

void CameraFrameCache::Clear()
{
    std::lock_guard<std::mutex> locker(m_mutex);
    std::deque<CachedFrame>().swap(m_frames);
    m_nextSequence = 0;
}

void CameraFrameCache::AppendFrame(const udpDataShow& frame)
{
    StoreFrame(frame);
}

std::uint64_t CameraFrameCache::Mark() const
{
    std::lock_guard<std::mutex> locker(m_mutex);
    return m_nextSequence;
}

bool CameraFrameCache::Latest(udpDataShow& frame) const
{
    std::lock_guard<std::mutex> locker(m_mutex);
    if (m_frames.empty())
    {
        return false;
    }

    frame = m_frames.back().frame;
    return true;
}

std::vector<udpDataShow> CameraFrameCache::FramesBetween(std::uint64_t beginExclusive, std::uint64_t endInclusive) const
{
    std::vector<udpDataShow> result;
    std::lock_guard<std::mutex> locker(m_mutex);
    result.reserve(m_frames.size());
    for (const CachedFrame& cachedFrame : m_frames)
    {
        if (cachedFrame.sequence > beginExclusive && cachedFrame.sequence <= endInclusive)
        {
            result.push_back(cachedFrame.frame);
        }
    }
    return result;
}

std::vector<CameraFrameCache::TimedFrame> CameraFrameCache::TimedFramesBetween(std::uint64_t beginExclusive, std::uint64_t endInclusive) const
{
    std::vector<TimedFrame> result;
    std::lock_guard<std::mutex> locker(m_mutex);
    result.reserve(m_frames.size());
    for (const CachedFrame& cachedFrame : m_frames)
    {
        if (cachedFrame.sequence > beginExclusive && cachedFrame.sequence <= endInclusive)
        {
            TimedFrame frame;
            frame.sequence = cachedFrame.sequence;
            frame.cameraTimestampUs = static_cast<qint64>(cachedFrame.frame.timestamp);
            frame.receiveTimestampUs = cachedFrame.receiveTimestampUs;
            frame.targetPoint = cachedFrame.frame.targetPoint;
            frame.errorMessage = cachedFrame.frame.errorMessage;
            result.push_back(frame);
        }
    }
    return result;
}

int CameraFrameCache::CachedCount() const
{
    std::lock_guard<std::mutex> locker(m_mutex);
    return static_cast<int>(m_frames.size());
}

void CameraFrameCache::StoreFrame(const udpDataShow& frame)
{
    std::lock_guard<std::mutex> locker(m_mutex);
    CachedFrame cachedFrame;
    cachedFrame.sequence = ++m_nextSequence;
    cachedFrame.receiveTimestampUs = CameraFrameCacheSteadyNowUs();
    cachedFrame.frame = frame;
    m_frames.push_back(cachedFrame);
    while (m_frames.size() > m_maxCachedFrames)
    {
        m_frames.pop_front();
    }
}
