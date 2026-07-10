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
    SetConnectionState(ConnectionState::Stopped, QStringLiteral("相机接收已停止。"));
}

void CameraFrameCache::Clear()
{
    std::lock_guard<std::mutex> locker(m_mutex);
    std::deque<CachedFrame>().swap(m_frames);
    // sequence 是 WaitForReadyFrameAfter 的代际标记，缓存生命周期内必须单调递增。
    // Clear 只丢帧内容；若重置为 0，等待者持有的旧 mark 会把重建后的新帧误判为旧帧（ABA）。
    // 注意：这里【不】清 m_pollStatus。ScanMoveAndCollect 在运动结束、帧已拉走后会再调一次 Clear()
    // 清帧缓存，若连带清了 poll 日志，会把整场扫描记录 wipe 在快照之前。poll 日志改由 ClearPollStatus() 单独管。
}

void CameraFrameCache::SetConnectionState(ConnectionState state, const QString& status)
{
    {
        std::lock_guard<std::mutex> locker(m_mutex);
        m_connectionState = state;
        m_connectionStatus = status;
    }
    m_readyCondition.notify_all();
}

CameraFrameCache::ConnectionState CameraFrameCache::GetConnectionState(QString* status) const
{
    std::lock_guard<std::mutex> locker(m_mutex);
    if (status != nullptr)
    {
        *status = m_connectionStatus;
    }
    return m_connectionState;
}

bool CameraFrameCache::WaitForReadyFrameAfter(std::uint64_t beginExclusive, int timeoutMs, QString* error) const
{
    std::unique_lock<std::mutex> locker(m_mutex);
    auto hasUsableFrameAfter = [&](QString* latestRejectReason, int* candidateCount) -> bool
        {
            int seen = 0;
            QString lastReject;
            for (auto it = m_frames.rbegin(); it != m_frames.rend(); ++it)
            {
                if (it->sequence <= beginExclusive)
                {
                    break;
                }
                ++seen;
                const udpDataShow& frame = it->frame;
                // worker 只在成功解码/成功取帧后 AppendFrame；源时间戳是 signed int64，
                // 转入 qulonglong 后必须转回 qint64 检查，避免负值被当成巨大正数放行。
                if (static_cast<qint64>(frame.timestamp) <= 0)
                {
                    if (lastReject.isEmpty())
                    {
                        lastReject = QStringLiteral("帧时间戳小于等于 0");
                    }
                    continue;
                }
                if (candidateCount != nullptr)
                {
                    *candidateCount = seen;
                }
                return true;
            }
            if (latestRejectReason != nullptr)
            {
                *latestRejectReason = lastReject;
            }
            if (candidateCount != nullptr)
            {
                *candidateCount = seen;
            }
            return false;
        };
    const auto readyOrTerminal = [&]()
        {
            return (m_connectionState == ConnectionState::Connected && hasUsableFrameAfter(nullptr, nullptr))
                || m_connectionState == ConnectionState::Failed
                || m_connectionState == ConnectionState::Stopped;
        };
    const int safeTimeoutMs = timeoutMs > 0 ? timeoutMs : 1;
    if (!m_readyCondition.wait_for(locker, std::chrono::milliseconds(safeTimeoutMs), readyOrTerminal))
    {
        if (error != nullptr)
        {
            QString rejectReason;
            int candidateCount = 0;
            hasUsableFrameAfter(&rejectReason, &candidateCount);
            if (candidateCount > 0)
            {
                *error = QString("等待相机有效新帧超时（%1 ms）：已收到 %2 个新帧，但时间戳均无效；最近原因：%3。")
                    .arg(safeTimeoutMs)
                    .arg(candidateCount)
                    .arg(rejectReason.isEmpty() ? QStringLiteral("未知") : rejectReason);
            }
            else
            {
                *error = m_connectionStatus.isEmpty()
                    ? QString("等待相机有效新帧超时（%1 ms），期间未收到新帧。").arg(safeTimeoutMs)
                    : QString("等待相机有效新帧超时（%1 ms）：%2").arg(safeTimeoutMs).arg(m_connectionStatus);
            }
        }
        return false;
    }
    if (m_connectionState == ConnectionState::Connected && hasUsableFrameAfter(nullptr, nullptr))
    {
        return true;
    }
    if (error != nullptr)
    {
        *error = m_connectionStatus.isEmpty() ? QStringLiteral("相机未连接。") : m_connectionStatus;
    }
    return false;
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

void CameraFrameCache::RecordPollStatus(int ret, qint64 frameTimestampUs, int pointCount, int frameChannel)
{
    std::lock_guard<std::mutex> locker(m_mutex);
    PollStatus status;
    status.receiveTimestampUs = CameraFrameCacheSteadyNowUs();
    status.ret = ret;
    status.frameTimestampUs = frameTimestampUs;
    status.pointCount = pointCount;
    status.frameChannel = frameChannel;
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

void CameraFrameCache::SetImageTransportEnabled(bool enabled)
{
    std::lock_guard<std::mutex> locker(m_mutex);
    m_imageTransportEnabled = enabled;
}

bool CameraFrameCache::ImageTransportEnabled() const
{
    std::lock_guard<std::mutex> locker(m_mutex);
    return m_imageTransportEnabled;
}

void CameraFrameCache::SetSkjFrameBufferCount(int count)
{
    std::lock_guard<std::mutex> locker(m_mutex);
    m_skjFrameBufferCount = count < 2 ? 2 : (count > 16 ? 16 : count);
}

int CameraFrameCache::SkjFrameBufferCount() const
{
    std::lock_guard<std::mutex> locker(m_mutex);
    return m_skjFrameBufferCount;
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
    m_readyCondition.notify_all();
}
