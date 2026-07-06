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
    // 注意：这里【不】清 m_pollStatus。ScanMoveAndCollect 在运动结束、帧已拉走后会再调一次 Clear()
    // 清帧缓存，若连带清了 poll 日志，会把整场扫描记录 wipe 在快照之前。poll 日志改由 ClearPollStatus() 单独管。
}

void CameraFrameCache::ClearPollStatus()
{
    std::lock_guard<std::mutex> locker(m_mutex);
    std::vector<PollStatus>().swap(m_pollStatus);
}

void CameraFrameCache::AppendFrame(const udpDataShow& frame)
{
    StoreFrame(frame);
}

void CameraFrameCache::RecordPollStatus(int ret, qint64 frameTimestampUs, int pointCount)
{
    std::lock_guard<std::mutex> locker(m_mutex);
    PollStatus status;
    status.receiveTimestampUs = CameraFrameCacheSteadyNowUs();
    status.ret = ret;
    status.frameTimestampUs = frameTimestampUs;
    status.pointCount = pointCount;
    m_pollStatus.push_back(status);
    if (m_pollStatus.size() > m_maxPollStatus)
    {
        // 极端兜底：理论上单次扫描达不到该量级；超限丢弃最旧的一半，避免空闲长跑时内存失控。
        m_pollStatus.erase(m_pollStatus.begin(), m_pollStatus.begin() + static_cast<std::ptrdiff_t>(m_pollStatus.size() / 2));
    }
}

std::vector<CameraFrameCache::PollStatus> CameraFrameCache::PollStatusSnapshot() const
{
    std::lock_guard<std::mutex> locker(m_mutex);
    return m_pollStatus;
}

void CameraFrameCache::SetImageCaptureDir(const QString& dir, int frameStride)
{
    std::lock_guard<std::mutex> locker(m_mutex);
    m_imageCaptureDir = dir;
    m_imageCaptureFrameStride = frameStride > 0 ? frameStride : 1;
}

QString CameraFrameCache::ImageCaptureDir() const
{
    std::lock_guard<std::mutex> locker(m_mutex);
    return m_imageCaptureDir;
}

int CameraFrameCache::ImageCaptureFrameStride() const
{
    std::lock_guard<std::mutex> locker(m_mutex);
    return m_imageCaptureFrameStride;
}

void CameraFrameCache::SetLiveImageEnabled(bool enabled)
{
    std::lock_guard<std::mutex> locker(m_mutex);
    m_liveImageEnabled = enabled;
    if (!enabled)
    {
        m_latestImage = QImage();
        m_latestImageTimestamp = 0;
    }
}

bool CameraFrameCache::LiveImageEnabled() const
{
    std::lock_guard<std::mutex> locker(m_mutex);
    return m_liveImageEnabled;
}

void CameraFrameCache::SetLatestImage(const QImage& image, qint64 imageTimestamp)
{
    std::lock_guard<std::mutex> locker(m_mutex);
    m_latestImage = image;  // QImage 隐式共享，仅引用计数
    m_latestImageTimestamp = imageTimestamp;
}

QImage CameraFrameCache::LatestImage(qint64* imageTimestamp) const
{
    std::lock_guard<std::mutex> locker(m_mutex);
    if (imageTimestamp != nullptr)
    {
        *imageTimestamp = m_latestImageTimestamp;
    }
    return m_latestImage;
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
