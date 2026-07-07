#pragma once

#include "groove/framebuffer.h"

#include <QImage>

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
        qint64 receiveTimestampUs = 0;   // 本次轮询(调用 GetLatestFrame)的 PC steady 时刻
        int ret = 0;                     // SKJCamera_GetLatestFrame 返回码
        qint64 frameTimestampUs = 0;     // 取到帧时的 SDK 帧时间戳（ret==0 有效，否则 0）
        int pointCount = -1;             // 取到帧的三维点数 count3d（ret==0 有效，否则 -1）
        int frameChannel = -1;           // SDK 帧号（GetChannel，厂商临时以通道号承载，逐帧+1 递增；差-1=丢帧数）
    };

    CameraFrameCache() = default;
    ~CameraFrameCache();
    CameraFrameCache(const CameraFrameCache&) = delete;
    CameraFrameCache& operator=(const CameraFrameCache&) = delete;

    void Start();
    void Stop();
    void Clear();
    void AppendFrame(const udpDataShow& frame);
    void RecordPollStatus(int ret, qint64 frameTimestampUs, int pointCount, int frameChannel = -1);
    void ClearPollStatus();  // 单独清 poll 日志：Clear() 清帧缓存时不再连带清它，避免运动后清帧把整场日志wipe掉

    // 图像采集窗口（扫描期间随点云一起后台保存相机图像）：Service 在扫描开始设目标目录、结束置空；
    // SKJ worker 每次图像轮询按此判断是否保存。目录为空 = 不采集（默认，防止常开预览把磁盘写爆）。
    void SetImageCaptureDir(const QString& dir, int frameStride = 1);
    QString ImageCaptureDir() const;
    int ImageCaptureFrameStride() const;  // 每 N 张新图像存 1 张（≥1）

    // 实时图像显示通道：worker 每取到新图写入，预览页/扫描页 UI 定时拉取（与激光线帧 Latest() 同模式）。
    // LiveImageEnabled 为真或采集窗口开启时 worker 才取图（预览打开时置真，避免无人看时空转拷贝）。
    void SetLiveImageEnabled(bool enabled);
    bool LiveImageEnabled() const;
    // 图像传输总开关（预览页按钮控制）：关闭时 worker 断开/不再连接图像口(50001)，
    // 用于排查/规避图像流挤占相机端资源导致的点云跳帧。默认开。
    void SetImageTransportEnabled(bool enabled);
    bool ImageTransportEnabled() const;
    void SetLatestImage(const QImage& image, qint64 imageTimestamp);
    QImage LatestImage(qint64* imageTimestamp = nullptr) const;

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
    QString m_imageCaptureDir;
    int m_imageCaptureFrameStride = 1;
    bool m_liveImageEnabled = false;
    bool m_imageTransportEnabled = true;
    QImage m_latestImage;
    qint64 m_latestImageTimestamp = 0;
    std::uint64_t m_nextSequence = 0;
    std::size_t m_maxCachedFrames = 2000;
    std::size_t m_maxPollStatus = 500000;  // 兜底上限：单次扫描 ~100/s×数百秒 远不及此，超限丢弃最旧一半
};
