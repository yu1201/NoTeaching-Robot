#pragma once

#include "Const.h"
#include "HandEyeMatrixConfig.h"

#include <QString>
#include <QVector>

#include <Eigen/Dense>

#include <vector>

class RobotCalculation
{
public:
    struct TimestampedRobotPose
    {
        qint64 timestampUs = 0;
        T_ROBOT_COORS pose;
    };

    struct IndexedPoint3D
    {
        int index = 0;
        Eigen::Vector3d point = Eigen::Vector3d::Zero();
    };

    enum class SampleAxis
    {
        AxisY = 0,
        AxisX = 1
    };

    enum class LowerWeldFitMode
    {
        PreservePath = 0,
        LineFit = 1,
        TrapezoidFit = 2,
        PiecewiseLineFit = 3
    };

    enum class LowerWeldGeometryStrategy
    {
        LegacyGeometry = 0,
        SlopeWaveFiltered = 1,
        RobustSegmentedKeys = 2,
        WorkpieceProjection = 3
    };

    struct LowerWeldFilterParams
    {
        struct CornerCompensation
        {
            double innerToOuterMm = 0.0;
            double innerToInnerMm = 0.0;
            double outerToOuterMm = 0.0;
            double outerToInnerMm = 0.0;
        };

        SampleAxis sampleAxis = SampleAxis::AxisY;
        LowerWeldFitMode fitMode = LowerWeldFitMode::PreservePath;
        LowerWeldGeometryStrategy geometryStrategy = LowerWeldGeometryStrategy::LegacyGeometry;
        double zThreshold = -230.0;
        double zJumpThreshold = 5.0;
        double zContinuityThreshold = 3.0;
        double segmentBreakDistance = 12.0;
        bool keepLongestSegmentOnly = true;
        double sampleStep = 2.0;
        double searchWindow = 8.0;
        int lineFitTrimCount = 0;
        double piecewiseFitTolerance = 2.0;
        int piecewiseMinSegmentPoints = 4;
        int minPointCount = 3;
        int smoothRadius = 2;
        // 点云投影提取（完整点云→下层焊道轨迹）参数；0 表示自动按滤波参数派生（原硬编码行为）。
        double projectionStationWindowMm = 0.0;     // 站位窗口：0=max(2.5, 采样步长*1.5)
        double projectionTransverseWindowMm = 0.0;  // 横向窗口：0=max(10, 搜索窗口*1.5)
        double projectionZBandBelowMm = 0.0;        // 种子下方Z带：0=max(14, Z连续阈值*4)
        double projectionZBandAboveMm = 0.0;        // 种子上方Z带：0=max(12, Z突变阈值*2.5)
        int projectionMaxCandidatePerSeed = 160;    // 每种子底板候选上限
        double projectionLayerLowPercent = 35.0;    // 底板层位Z分位下界（%）
        double projectionLayerHighPercent = 50.0;   // 底板层位Z分位上界（%）
        int projectionSmoothRadius = 0;             // 投影轮廓平滑半径：0=按滤波平滑半径夹到[1,4]
        bool useSlopeConsistentCornerFit = false;
        // 调试：把每段拟合用到的点集与拟合直线导出成 CloudCompare 友好的点云，
        // 用来核对分段直线拟合是否正确。fitDebugDir 为空时不导出。
        bool exportFitDebugCloud = false;
        QString fitDebugDir;
        bool enableCornerCompensation = false;
        bool validationCoverageEnabled = true;
        int validationMinFinitePointCount = 300;
        double validationMinProjectedSpanMm = 180.0;
        bool validationContinuityEnabled = true;
        double validationMinStationCoverageRatio = 0.55;
        double validationMinLongestContinuousRatio = 0.60;
        bool validationDenoiseRatioEnabled = true;
        double validationMaxRejectedRatio = 0.40;
        bool validationResidualEnabled = true;
        double validationMaxMedianResidualMm = 3.0;
        double validationMaxP95ResidualMm = 8.0;
        double validationResidualInlierThresholdMm = 6.0;
        double validationMinResidualInlierRatio = 0.75;
        bool validationKeyPointEnabled = true;
        int validationMinKeyPointCount = 6;
        int validationMinCornerCount = 4;
        double validationMinSegmentLengthMm = 15.0;
        bool validationOutputEnabled = true;
        int validationMinOutputPointCount = 80;
        double validationMinOutputLengthRatio = 0.70;
        CornerCompensation risingCornerCompensation;
        CornerCompensation fallingCornerCompensation;
    };

    struct LowerWeldFilterPoint
    {
        int index = 0;
        Eigen::Vector3d point = Eigen::Vector3d::Zero();
        QString source;
    };

    struct LowerWeldFilterResult
    {
        bool ok = false;
        QString error;
        QVector<LowerWeldFilterPoint> points;
        int inputPointCount = 0;
        int lowerPointCount = 0;
        int zJumpRejectedCount = 0;
        int zContinuityRejectedCount = 0;
        int segmentRejectedCount = 0;
        int fitSegmentCount = 0;
        int measuredCount = 0;
        int interpolatedCount = 0;
        int extendedCount = 0;
    };

    enum class LowerWeldPointType
    {
        Start = 1,
        End = 2,
        InnerCorner = 3,
        OuterCorner = 4,
        Normal = 5,
        Noise = 6
    };

    struct LowerWeldClassifiedPoint
    {
        int index = 0;
        Eigen::Vector3d point = Eigen::Vector3d::Zero();
        LowerWeldPointType type = LowerWeldPointType::Normal;
        QString source;
        QString segmentKindAfter;
    };

    struct LowerWeldClassificationResult
    {
        bool ok = false;
        QString error;
        QVector<LowerWeldClassifiedPoint> points;
        int startCount = 0;
        int endCount = 0;
        int innerCornerCount = 0;
        int outerCornerCount = 0;
        int normalCount = 0;
        int noiseCount = 0;
    };

    struct MeasureThenWeldAnalysisResult
    {
        bool ok = false;
        QString error;
        LowerWeldFilterResult filterResult;
        LowerWeldClassificationResult classificationResult;
        QVector<LowerWeldClassifiedPoint> keyPoints;
        LowerWeldClassificationResult cornerCompensatedClassificationResult;
        QVector<LowerWeldClassifiedPoint> cornerCompensatedKeyPoints;
    };

    static T_ROBOT_COORS InterpolateRobotPose(const std::vector<TimestampedRobotPose>& robotSamples, qint64 targetTimestampUs);
    static Eigen::Vector3d CalcLaserPointInRobot(const T_ROBOT_COORS& robotPose, const Eigen::Vector3d& cameraPoint, const HandEyeMatrixConfig& calibration);
    static LowerWeldFilterResult FilterLowerWeldPath(const QVector<IndexedPoint3D>& inputPoints, const LowerWeldFilterParams& params);
    static LowerWeldClassificationResult ClassifyLowerWeldPoints(
        const LowerWeldFilterResult& filterResult,
        SampleAxis sampleAxis);
    static LowerWeldFilterResult ProjectWorkpieceCloudToLowerWeldPath(
        const QVector<IndexedPoint3D>& workpieceCloudInput,
        const QVector<IndexedPoint3D>& seedPathInput,
        const LowerWeldFilterParams& params);
    static MeasureThenWeldAnalysisResult AnalyzeMeasureThenWeldLowerWeldPath(
        const QVector<IndexedPoint3D>& inputPoints,
        const LowerWeldFilterParams& params);
    static MeasureThenWeldAnalysisResult AnalyzeMeasureThenWeldLowerWeldPathDirect(
        const QVector<IndexedPoint3D>& inputPoints,
        const LowerWeldFilterParams& params);
    static MeasureThenWeldAnalysisResult AnalyzeMeasureThenWeldLowerWeldPathGeometry(
        const QVector<IndexedPoint3D>& inputPoints,
        const LowerWeldFilterParams& params);
    static LowerWeldClassificationResult BuildCornerCompensatedLowerWeldClassification(
        const QVector<LowerWeldClassifiedPoint>& keyPoints,
        const LowerWeldFilterParams& params,
        QVector<LowerWeldClassifiedPoint>* compensatedKeyPoints = nullptr);
    static int LowerWeldPointTypeCode(LowerWeldPointType type);
    static QString LowerWeldPointTypeName(LowerWeldPointType type);
    static QString RobotPoseCsv(qint64 timestampUs, const T_ROBOT_COORS& pose);
    static QString Vector3Csv(qint64 timestampUs, const Eigen::Vector3d& point, const QString& extra = QString());
    static QString RobotPoseIndexedCsv(int index, const T_ROBOT_COORS& pose);
    static QString Vector3IndexedCsv(int index, const Eigen::Vector3d& point, const QString& extra = QString());
    static QString Vector3IndexedSpaceText(int index, const Eigen::Vector3d& point, const QString& extra = QString());
};
