#include "CameraFrameCache.h"

#include "groove/threadsafebuffer.h"

#include <chrono>

CameraFrameCache& CameraFrameCache::Instance()
{
    static CameraFrameCache cache;
    return cache;
}

CameraFrameCache::~CameraFrameCache()
{
    Stop();
}

void CameraFrameCache::Start()
{
    bool expected = false;
    if (!m_running.compare_exchange_strong(expected, true, std::memory_order_acq_rel))
    {
        return;
    }

    m_thread = std::thread(&CameraFrameCache::ThreadMain, this);
}

void CameraFrameCache::Stop()
{
    bool expected = true;
    if (!m_running.compare_exchange_strong(expected, false, std::memory_order_acq_rel))
    {
        return;
    }

    if (m_thread.joinable())
    {
        m_thread.join();
    }
}

void CameraFrameCache::Clear()
{
    std::lock_guard<std::mutex> locker(m_mutex);
    m_frames.clear();
    m_nextSequence = 0;
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

int CameraFrameCache::CachedCount() const
{
    std::lock_guard<std::mutex> locker(m_mutex);
    return static_cast<int>(m_frames.size());
}

void CameraFrameCache::ThreadMain()
{
    while (m_running.load(std::memory_order_acquire))
    {
        udpDataShow frame;
        if (ThreadSafeBuffer<udpDataShow>::Instance().dequeue(frame, 20))
        {
            StoreFrame(frame);
            while (ThreadSafeBuffer<udpDataShow>::Instance().dequeue(frame, 0))
            {
                StoreFrame(frame);
            }
        }
    }

    udpDataShow frame;
    while (ThreadSafeBuffer<udpDataShow>::Instance().dequeue(frame, 0))
    {
        StoreFrame(frame);
    }
}

void CameraFrameCache::StoreFrame(const udpDataShow& frame)
{
    std::lock_guard<std::mutex> locker(m_mutex);
    CachedFrame cachedFrame;
    cachedFrame.sequence = ++m_nextSequence;
    cachedFrame.frame = frame;
    m_frames.push_back(cachedFrame);
    while (m_frames.size() > m_maxCachedFrames)
    {
        m_frames.pop_front();
    }
}
