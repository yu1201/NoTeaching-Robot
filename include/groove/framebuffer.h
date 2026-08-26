#ifndef FRAMEBUFFER_H
#define FRAMEBUFFER_H

#include <QVector>
#include <QtGlobal>
#include <opencv2/opencv.hpp>
#include <vector>

// 相机底层输出给缓存/业务层的统一坐标契约：
// allResultPoint 必须先转换为与 targetPoint 完全相同的设备 XYZ 坐标约定。
// LegacyCloudZOppositeTarget 只描述旧 TCP/UDP 原生协议，转换必须在对应相机底层完成；
// 扫描、标定等业务层不得再按相机品牌修改坐标符号。
enum class CameraNativePointCloudConvention
{
    SameAsTarget,
    LegacyCloudZOppositeTarget
};

inline cv::Point3d CanonicalizeCameraPointCloudPoint(
    const cv::Point3d& point,
    CameraNativePointCloudConvention convention)
{
    if (convention == CameraNativePointCloudConvention::LegacyCloudZOppositeTarget)
    {
        return cv::Point3d(point.x, point.y, -point.z);
    }
    return point;
}

inline std::vector<cv::Point3d> CanonicalizeCameraPointCloud(
    const std::vector<cv::Point3d>& points,
    CameraNativePointCloudConvention convention)
{
    std::vector<cv::Point3d> canonicalPoints;
    canonicalPoints.reserve(points.size());
    for (const cv::Point3d& point : points)
    {
        canonicalPoints.push_back(CanonicalizeCameraPointCloudPoint(point, convention));
    }
    return canonicalPoints;
}

struct udpDataShow
{

    QVector<double> XData;
    QVector<double> YData;
    QVector<double> fitLineX;
    QVector<double> fitLineY;
    QVector<double> targetX;  
    QVector<double> targetY;
    float mFps = 0.0f;
    // 统一后的设备坐标点云，与 targetPoint 同 XYZ 方向和单位。
    std::vector<cv::Point3d> allResultPoint;
    // 生产者完成坐标规范化后必须置 true；非空点云为 false 时扫描业务失败关闭。
    bool allResultPointCanonical = false;
    qulonglong timestamp;

    QString errorMessage;

    cv::Point3d targetPoint;
};

#endif // FRAMEBUFFER_H
