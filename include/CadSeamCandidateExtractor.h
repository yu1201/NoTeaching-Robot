#pragma once

#include <Eigen/Dense>

#include <QString>
#include <QVector>

// 只读的 STEP/B-Rep 焊缝候选提取器。
//
// 本模块只生成“待人工确认”的几何候选，不提供焊缝语义、不生成机器人姿态，
// 更不会启动运动。候选点始终保留在源 STEP 的模型坐标（mm）中。
class CadSeamCandidateExtractor
{
public:
    enum class SourceKind
    {
        SharedEdge,
        ShapeIntersection,
        CorrugatedButtJoint,
        CorrugatedBaseJoint
    };

    struct Options
    {
        qint64 maximumFileBytes = 256LL * 1024LL * 1024LL;
        int maximumRoots = 256;
        int maximumFaces = 20000;
        int maximumSharedEdges = 200000;
        int maximumRootPairs = 4096;
        int maximumCandidates = 4096;
        int maximumPointsPerCandidate = 4096;
        double minimumLengthMm = 5.0;
        double samplingStepMm = 2.0;
        double minimumDihedralDegrees = 5.0;
        double maximumDihedralDegrees = 90.0;
        double linearToleranceMm = 0.05;
        // 同一实体的共享锐边通常是钣金折弯棱，不具备焊缝语义。默认只从
        // 不同实体/壳体之间的接触截交线提取；诊断测试可显式重新启用共边。
        bool includeSharedEdges = false;
        bool includeRootIntersections = true;
        bool preferAssemblySeamSemantics = true;
    };

    struct Candidate
    {
        QString candidateId;
        SourceKind sourceKind = SourceKind::SharedEdge;
        QVector<Eigen::Vector3d> pointsModelMm;
        double lengthMm = 0.0;
        double dihedralDegrees = 0.0;
        int firstFaceIndex = -1;
        int secondFaceIndex = -1;
        int firstRootIndex = -1;
        int secondRootIndex = -1;
    };

    struct Statistics
    {
        int rootCount = 0;
        int faceCount = 0;
        int sharedEdgeCount = 0;
        int rootPairCount = 0;
        int intersectedRootPairCount = 0;
        int rejectedShortCount = 0;
        int rejectedTangentCount = 0;
        int duplicateCount = 0;
        int corrugatedComponentCount = 0;
        int corrugatedButtJointCount = 0;
        int corrugatedBaseJointCount = 0;
        bool assemblySemanticsUsed = false;
    };

    static bool ExtractFromStepFile(
        const QString& stepFilePath,
        QVector<Candidate>& candidates,
        QString& error,
        Statistics* statistics = nullptr,
        const Options* tighterLimits = nullptr);
};
