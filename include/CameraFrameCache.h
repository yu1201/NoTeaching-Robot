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

    // 单次轮询的 SDK 取帧状态：receiveTimestampUs 为轮询时刻（与帧 receiveTimestampUs 同一 steady 时钟），
    // ret 为 SKJCamera_GetLatestFrame 返回码（0=取到新帧，-106=无新帧，-107=重复帧，其它=取帧错误）。
    struct PollStatus
    {
        qint64 receiveTimestampUs = 0;
        int ret = 0;
    };

    CameraFrameCache() = default;
    ~CameraFrameCache();
    CameraFrameCache(const CameraFrameCache&) = delete;
    CameraFrameCache& operator=(const CameraFrameCache&) = delete;

    void Start();
    void Stop();
    void Clear();
    void AppendFrame(const udpDataShow& frame);
    void RecordPollStatus(int ret);

    std::uint64_t Mark() const;
    bool Latest(udpDataShow& frame) const;
    std::vector<udpDataShow> FramesBetween(std::uint64_t beginExclusive, std::uint64_t endInclusive) const;
    std::vector<TimedFrame> TimedFramesBetween(std::uint64_t beginExclusive, std::uint64_t endInclusive) const;
    int CachedCount() const;
    std::vector<PollStatus> PollStatusSnapshot() const;

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
    std::vector<PollStatus> m_pollStatus;
    std::uint64_t m_nextSequence = 0;
    std::size_t m_maxCachedFrames = 2000;
    std::size_t m_maxPollStatus = 500000;  // 兜底上限：单次扫描 ~100/s×数百秒 远不及此，超限丢弃最旧一半
};
