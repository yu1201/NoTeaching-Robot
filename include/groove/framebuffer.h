#ifndef FRAMEBUFFER_H
#define FRAMEBUFFER_H

#include <QVector>
#include <QtGlobal>
#include <opencv2/opencv.hpp>
#include <vector>

struct udpDataShow
{

    QVector<double> XData;
    QVector<double> YData;
    QVector<double> fitLineX;
    QVector<double> fitLineY;
    QVector<double> targetX;  
    QVector<double> targetY;
    float mFps = 0.0f;
    std::vector<cv::Point3d> allResultPoint;
    qulonglong timestamp;

    QString errorMessage;

    cv::Point3d targetPoint;
};

#endif // FRAMEBUFFER_H
