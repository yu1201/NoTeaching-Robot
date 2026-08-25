#pragma once

#include "RobotCalculation.h"
#include "WorkpieceMeshBuilder.h"

#include <Eigen/Dense>

#include <QString>
#include <QVector>

// 逆向 PLY/点云模型的“人工粗种子 -> 表面轨迹”入口。
//
// 本类复用工程现有的完整点云投影算法，只输出模型坐标点列；没有种子时明确失败，
// 不把三角网格锐边误当焊缝，也不生成机器人姿态或执行运动。
class ReverseMeshSeamProjector
{
public:
    struct Options
    {
        RobotCalculation::SampleAxis sampleAxis = RobotCalculation::SampleAxis::AxisX;
        double sampleStepMm = 2.0;
        double stationWindowMm = 3.0;
        double transverseWindowMm = 12.0;
        double zBandBelowMm = 14.0;
        double zBandAboveMm = 12.0;
        int maximumOutputPoints = 4096;
    };

    struct Result
    {
        QVector<Eigen::Vector3d> pathModelMm;
        int meshVertexCount = 0;
        int seedPointCount = 0;
        int measuredPointCount = 0;
        int fallbackPointCount = 0;
    };

    static bool Project(
        const WorkpieceMeshBuilder::Mesh& mesh,
        const QVector<Eigen::Vector3d>& seedPathModelMm,
        Result& result,
        QString& error,
        const Options* options = nullptr);
};
