#pragma once

#include "Const.h"
#include "WorkpieceMeshBuilder.h"

#include <Eigen/Dense>

#include <QByteArray>
#include <QString>
#include <QVector>

struct HandEyeMatrixConfig;

enum class ModelWeldingStationRole
{
    Solve,
    Verify,
    Backup
};

struct ModelWeldingPlacementGuide
{
    Eigen::Vector3d anchorModelMm = Eigen::Vector3d::Zero();
    // 工件吸附落地后的显示坐标框架；列向量依次是地面内 +X、+Y、地面向上 +Z，
    // 均表达在模型坐标中。V型槽可在该框架内独立绕 +Z 旋转。
    Eigen::Matrix3d axesModel = Eigen::Matrix3d::Identity();
    // V型槽相对 axesModel 绕 +Z 的规范偏航角，范围为 [-180, 180)。
    double vSlotYawDegrees = 0.0;
    double longLengthMm = 300.0;
    double shortLengthMm = 180.0;
};

struct ModelWeldingFeatureStation
{
    QString stationId;                 // S01 / S02 / V01 / B01
    QString displayName;
    ModelWeldingStationRole role = ModelWeldingStationRole::Solve;
    Eigen::Vector3d anchorModelMm = Eigen::Vector3d::Zero();
    Eigen::Vector3d scanDirectionModel = Eigen::Vector3d::UnitX();
    Eigen::Vector3d roiHalfExtentMm = Eigen::Vector3d(25.0, 25.0, 15.0);
    bool candidateConfirmed = false;   // 自动候选必须经人工确认后才能进入生产就绪判断。
};

struct ModelWeldingFlowTemplate
{
    static constexpr int SchemaVersion = 1;

    int schemaVersion = SchemaVersion;
    QString templateId;
    quint64 revision = 1;
    QString displayName;
    QString modelLibraryName;
    QString modelSha256;
    QString units = QStringLiteral("mm");
    ModelWeldingPlacementGuide placement;
    QVector<ModelWeldingFeatureStation> stations;

    // 相似模型继承只保存来源证明；新模板是快照，不会随来源模板继续变化。
    QString inheritedFromTemplateId;
    quint64 inheritedFromRevision = 0;
    QString inheritedFromRecordSha256;
    bool humanSameFixtureConfirmed = false;
    bool humanDatumConfirmed = false;
    bool humanScanAreaConfirmed = false;
    bool humanCollisionChecked = false;

    QString createdAtUtc;
    QString updatedAtUtc;
};

struct ModelWeldingScanTeaching
{
    QString stationId;
    bool startTaught = false;
    bool endTaught = false;
    T_ROBOT_COORS startPose;
    T_ROBOT_COORS endPose;
    bool startPulseTaught = false;
    T_ANGLE_PULSE startPulse;
    double runSpeedMmPerMin = 300.0;
    double scanSpeedMmPerMin = 100.0;
};

struct ModelWeldingRobotTeaching
{
    static constexpr int SchemaVersion = 3;

    int schemaVersion = SchemaVersion;
    QString teachingId;
    quint64 revision = 1;
    QString templateId;
    quint64 templateRevision = 0;
    QString templateRecordSha256;
    QString robotName;
    QString robotEndpoint;
    // 机器人型号资产身份必须与控制单元和目录解析结果逐项一致。旧版记录读取后
    // 三项均为空，只能作为迁移草稿，不能通过生产就绪检查。
    QString robotModelId;
    QString sourceStepSha256;
    QString collisionProfileSha256;
    QString cameraSection;
    QString handEyeSha256;
    QString tool1Sha256;
    QVector<ModelWeldingScanTeaching> scans;
    QString createdAtUtc;
    QString updatedAtUtc;
};

struct ModelWeldingRigidPointPair
{
    QString featureId;
    Eigen::Vector3d modelPointMm = Eigen::Vector3d::Zero();
    Eigen::Vector3d measuredBasePointMm = Eigen::Vector3d::Zero();
    bool useForSolve = true;
};

struct ModelWeldingRigidFitOptions
{
    double minimumSecondToFirstSingularRatio = 1.0e-4;
    double maximumSolveRmseMm = 3.0;
    double maximumSolveResidualMm = 5.0;
    double maximumVerifyResidualMm = 5.0;
    double maximumPairDistanceErrorMm = 3.0;
    double maximumScaleDeviation = 0.01; // 仅作拒绝门禁，求解始终固定 scale=1。
};

struct ModelWeldingRigidFitResult
{
    bool solved = false;
    bool accepted = false;
    Eigen::Matrix4d baseFromModel = Eigen::Matrix4d::Identity();
    Eigen::Vector3d singularValues = Eigen::Vector3d::Zero();
    QVector<double> solveResidualsMm;
    QVector<double> verifyResidualsMm;
    double solveRmseMm = 0.0;
    double maximumSolveResidualMm = 0.0;
    double maximumVerifyResidualMm = 0.0;
    double meanPairDistanceErrorMm = 0.0;
    double maximumPairDistanceErrorMm = 0.0;
    double observedScaleRatio = 1.0;
    QString rejectionReason;
};

class ModelWeldingWorkflow
{
public:
    enum class LoadStatus
    {
        Found,
        NotFound,
        Error
    };

    static QString CreateStableId();
    static QString ComputeFileSha256(const QString& filePath, QString* error = nullptr);
    static QString ComputeHandEyeSha256(const HandEyeMatrixConfig& config);
    static QString ComputeTool1Sha256(const T_ROBOT_COORS& tool1);

    // 自动结果只是粗放置/候选站建议，候选站不会自动标记为生产可用。
    static bool GeneratePlacementGuide(
        const WorkpieceMeshBuilder::Mesh& mesh,
        double longLengthMm,
        double shortLengthMm,
        ModelWeldingPlacementGuide& guide,
        QString& error);
    static QVector<ModelWeldingFeatureStation> GenerateDraftStations(
        const WorkpieceMeshBuilder::Mesh& mesh,
        int solveCount = 3,
        int verifyCount = 1);

    static bool ValidateTemplateStructure(
        const ModelWeldingFlowTemplate& value,
        QString& error,
        bool requireProductionReady = false);
    static bool ValidateTeachingStructure(
        const ModelWeldingRobotTeaching& value,
        const ModelWeldingFlowTemplate* modelTemplate,
        QString& error,
        bool requireProductionReady = false);

    static QByteArray EncodeTemplate(const ModelWeldingFlowTemplate& value, QString& error);
    static bool DecodeTemplate(const QByteArray& json, ModelWeldingFlowTemplate& value, QString& error);
    static QByteArray EncodeTeaching(const ModelWeldingRobotTeaching& value, QString& error);
    static bool DecodeTeaching(const QByteArray& json, ModelWeldingRobotTeaching& value, QString& error);
    static QString TemplateRecordSha256(const ModelWeldingFlowTemplate& value, QString& error);

    static bool SaveTemplate(
        const ModelWeldingFlowTemplate& value,
        quint64 expectedPreviousRevision,
        QString& error);
    static LoadStatus LoadTemplate(
        const QString& templateId,
        ModelWeldingFlowTemplate& value,
        QString& error);
    static bool ListTemplates(QVector<ModelWeldingFlowTemplate>& values, QString& error);

    static bool SaveTeaching(
        const ModelWeldingRobotTeaching& value,
        quint64 expectedPreviousRevision,
        QString& error);
    static LoadStatus LoadTeaching(
        const QString& teachingId,
        ModelWeldingRobotTeaching& value,
        QString& error);
    static bool ListTeachings(QVector<ModelWeldingRobotTeaching>& values, QString& error);

    static ModelWeldingRigidFitResult SolveRigidPointPairs(
        const QVector<ModelWeldingRigidPointPair>& pairs,
        const ModelWeldingRigidFitOptions& options = ModelWeldingRigidFitOptions());
};
