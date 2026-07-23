#pragma once

#include "WorkpieceMeshBuilder.h"

#include <Eigen/Core>

#include <QString>
#include <QStringList>
#include <QtGlobal>

// STEP/STP CAD 模型导入器：使用 Open CASCADE 读取 BRep，并转换为本工程统一的
// 毫米制三角网格。导入过程不会移动、旋转或居中模型，保留 STEP 中的坐标轴与原点。
class StepModelImporter
{
public:
    struct Options
    {
        // 0 表示按模型包围盒对角线自适应：clamp(diagonal / 5000, 0.02, 0.20) mm。
        double linearDeflectionMm = 0.0;
        double angularDeflectionRadians = 0.35;

        // 这些限制可由测试或更严格的现场策略缩小，但不能超过生产硬上限。
        qint64 maximumFileBytes = 256LL * 1024LL * 1024LL;
        qsizetype maximumVertices = 2'000'000;
        qsizetype maximumTriangles = 4'000'000;
    };

    struct Statistics
    {
        qsizetype sourceFaceCount = 0;
        qsizetype meshedFaceCount = 0;
        qsizetype vertexCount = 0;
        qsizetype triangleCount = 0;
        qsizetype skippedDegenerateTriangles = 0;
        double linearDeflectionMm = 0.0;
        Eigen::Vector3d boundsMinMm = Eigen::Vector3d::Zero();
        Eigen::Vector3d boundsMaxMm = Eigen::Vector3d::Zero();
        QStringList sourceLengthUnits;
        QString occtVersion;
    };

    // 成功时 mesh 是可直接交给 WorkpieceMeshBuilder::SaveMeshPly 的完整网格；
    // 失败或抛出任何异常时 mesh 会被清空，并通过 error 返回可展示给用户的中文原因。
    static bool ImportFile(
        const QString& stepFilePath,
        WorkpieceMeshBuilder::Mesh& mesh,
        QString& error,
        Statistics* statistics = nullptr,
        const Options* tighterLimits = nullptr);
};
