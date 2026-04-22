#ifndef FRAMEBUFFER_H
#define FRAMEBUFFER_H

#include <QVector>
#include <opencv2/core.hpp>

struct udpDataShow
{

    QVector<double> XData;
    QVector<double> YData;
    QVector<double> fitLineX;
    QVector<double> fitLineY;
    QVector<double> targetX;  
    QVector<double> targetY;

    QString errorMessage;

    cv::Point3d targetPoint;
};

#endif // FRAMEBUFFER_H
