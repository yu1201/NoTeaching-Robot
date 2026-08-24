#include "ModelWeldingWorkflow.h"

#include "ConfigDatabase.h"

#include <Eigen/Geometry>

#include <QByteArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QStringList>

#include <cmath>
#include <iostream>
#include <limits>

namespace
{
int g_failures = 0;

void Check(bool condition, const char* message)
{
    if (!condition)
    {
        std::cerr << "FAIL: " << message << '\n';
        ++g_failures;
    }
}

bool Near(double left, double right, double tolerance)
{
    return std::abs(left - right) <= tolerance;
}

bool Near(const Eigen::Matrix4d& left, const Eigen::Matrix4d& right, double tolerance)
{
    return (left - right).cwiseAbs().maxCoeff() <= tolerance;
}

Eigen::Matrix4d KnownTransform()
{
    const Eigen::Matrix3d rotation =
        (Eigen::AngleAxisd(0.43, Eigen::Vector3d(0.2, -0.6, 0.7).normalized())).toRotationMatrix();
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3, 3>(0, 0) = rotation;
    transform.block<3, 1>(0, 3) = Eigen::Vector3d(420.0, -135.0, 860.0);
    return transform;
}

Eigen::Vector3d Apply(const Eigen::Matrix4d& transform, const Eigen::Vector3d& point)
{
    return transform.block<3, 3>(0, 0) * point + transform.block<3, 1>(0, 3);
}

QVector<ModelWeldingRigidPointPair> ExactPairs(const Eigen::Matrix4d& transform)
{
    const QVector<Eigen::Vector3d> points = {
        Eigen::Vector3d(0.0, 0.0, 0.0),
        Eigen::Vector3d(120.0, 0.0, 5.0),
        Eigen::Vector3d(15.0, 95.0, -8.0),
        Eigen::Vector3d(-30.0, 25.0, 70.0)
    };
    QVector<ModelWeldingRigidPointPair> pairs;
    for (int i = 0; i < points.size(); ++i)
    {
        ModelWeldingRigidPointPair pair;
        pair.featureId = QStringLiteral("S%1").arg(i + 1, 2, 10, QLatin1Char('0'));
        pair.modelPointMm = points.at(i);
        pair.measuredBasePointMm = Apply(transform, points.at(i));
        pair.useForSolve = true;
        pairs.push_back(pair);
    }
    return pairs;
}

void TestExactRigidFit()
{
    const Eigen::Matrix4d expected = KnownTransform();
    QVector<ModelWeldingRigidPointPair> pairs = ExactPairs(expected);
    ModelWeldingRigidPointPair verify;
    verify.featureId = QStringLiteral("V01");
    verify.modelPointMm = Eigen::Vector3d(48.0, -22.0, 31.0);
    verify.measuredBasePointMm = Apply(expected, verify.modelPointMm);
    verify.useForSolve = false;
    pairs.push_back(verify);

    const ModelWeldingRigidFitResult result = ModelWeldingWorkflow::SolveRigidPointPairs(pairs);
    Check(result.solved, "exact rigid fit should be solved");
    Check(result.accepted, "exact rigid fit should be accepted");
    Check(result.rejectionReason.isEmpty(), "accepted exact fit should not have a rejection reason");
    Check(Near(result.baseFromModel, expected, 1.0e-9), "exact transform should be recovered");
    Check(result.solveRmseMm < 1.0e-9, "exact solve RMSE should be near zero");
    Check(result.maximumVerifyResidualMm < 1.0e-9, "exact verify residual should be near zero");
    Check(Near(result.observedScaleRatio, 1.0, 1.0e-12), "exact fit scale ratio should be one");
}

void TestNoisyRigidFit()
{
    const Eigen::Matrix4d expected = KnownTransform();
    QVector<ModelWeldingRigidPointPair> pairs = ExactPairs(expected);
    const QVector<Eigen::Vector3d> noise = {
        Eigen::Vector3d(0.18, -0.12, 0.06),
        Eigen::Vector3d(-0.11, 0.08, -0.04),
        Eigen::Vector3d(0.05, 0.13, -0.09),
        Eigen::Vector3d(-0.07, -0.05, 0.12)
    };
    for (int i = 0; i < pairs.size(); ++i)
    {
        pairs[i].measuredBasePointMm += noise.at(i);
    }

    ModelWeldingRigidPointPair verify;
    verify.featureId = QStringLiteral("V01");
    verify.modelPointMm = Eigen::Vector3d(48.0, -22.0, 31.0);
    verify.measuredBasePointMm = Apply(expected, verify.modelPointMm) + Eigen::Vector3d(0.12, -0.08, 0.05);
    verify.useForSolve = false;
    pairs.push_back(verify);

    const ModelWeldingRigidFitResult result = ModelWeldingWorkflow::SolveRigidPointPairs(pairs);
    Check(result.solved, "noisy rigid fit should be solved");
    Check(result.accepted, "small deterministic noise should remain inside default gates");
    Check(result.solveRmseMm > 0.0 && result.solveRmseMm < 0.3,
        "noisy solve RMSE should be non-zero and small");
    Check(result.maximumVerifyResidualMm < 0.3, "noisy verify residual should be small");
    Check((result.baseFromModel.block<3, 1>(0, 3) - expected.block<3, 1>(0, 3)).norm() < 0.3,
        "noisy translation estimate should remain close");
}

void TestCollinearRejection()
{
    QVector<ModelWeldingRigidPointPair> pairs;
    for (int i = 0; i < 4; ++i)
    {
        ModelWeldingRigidPointPair pair;
        pair.featureId = QStringLiteral("S%1").arg(i + 1);
        pair.modelPointMm = Eigen::Vector3d(i * 40.0, 0.0, 0.0);
        pair.measuredBasePointMm = pair.modelPointMm + Eigen::Vector3d(10.0, 20.0, 30.0);
        pairs.push_back(pair);
    }
    const ModelWeldingRigidFitResult result = ModelWeldingWorkflow::SolveRigidPointPairs(pairs);
    Check(!result.solved, "collinear points should be rejected before solving");
    Check(!result.accepted, "collinear points must not be accepted");
    Check(!result.rejectionReason.isEmpty(), "collinear rejection should explain the failure");
}

void TestScaleRejection()
{
    QVector<ModelWeldingRigidPointPair> pairs;
    const QVector<Eigen::Vector3d> points = {
        Eigen::Vector3d(0.0, 0.0, 0.0),
        Eigen::Vector3d(100.0, 0.0, 0.0),
        Eigen::Vector3d(0.0, 80.0, 0.0),
        Eigen::Vector3d(20.0, 15.0, 60.0)
    };
    for (int i = 0; i < points.size(); ++i)
    {
        ModelWeldingRigidPointPair pair;
        pair.featureId = QStringLiteral("S%1").arg(i + 1);
        pair.modelPointMm = points.at(i);
        pair.measuredBasePointMm = points.at(i) * 1.10 + Eigen::Vector3d(50.0, -25.0, 12.0);
        pairs.push_back(pair);
    }
    ModelWeldingRigidFitOptions options;
    options.maximumSolveRmseMm = 1000.0;
    options.maximumSolveResidualMm = 1000.0;
    options.maximumPairDistanceErrorMm = 1000.0;
    options.maximumScaleDeviation = 0.01;
    const ModelWeldingRigidFitResult result = ModelWeldingWorkflow::SolveRigidPointPairs(pairs, options);
    Check(result.solved, "scaled point set should still produce a rigid least-squares result");
    Check(!result.accepted, "scaled point set must be rejected by the scale gate");
    Check(Near(result.observedScaleRatio, 1.10, 1.0e-12), "observed scale ratio should expose the scale change");
    Check(result.rejectionReason.contains(QStringLiteral("比例")), "scale rejection should identify scale change");
}

void TestVerifyPointRejection()
{
    const Eigen::Matrix4d expected = KnownTransform();
    QVector<ModelWeldingRigidPointPair> pairs = ExactPairs(expected);
    ModelWeldingRigidPointPair verify;
    verify.featureId = QStringLiteral("V01");
    verify.modelPointMm = Eigen::Vector3d(35.0, 40.0, 20.0);
    verify.measuredBasePointMm = Apply(expected, verify.modelPointMm) + Eigen::Vector3d(12.0, 0.0, 0.0);
    verify.useForSolve = false;
    pairs.push_back(verify);

    ModelWeldingRigidFitOptions options;
    options.maximumVerifyResidualMm = 2.0;
    const ModelWeldingRigidFitResult result = ModelWeldingWorkflow::SolveRigidPointPairs(pairs, options);
    Check(result.solved, "bad independent verify point should not prevent solving");
    Check(!result.accepted, "bad independent verify point must reject the fit");
    Check(result.maximumVerifyResidualMm > 11.9, "verify residual should expose the injected offset");
    Check(result.rejectionReason.contains(QStringLiteral("验证点")), "verify rejection should identify the verify gate");
}

ModelWeldingFlowTemplate ValidTemplate()
{
    ModelWeldingFlowTemplate value;
    value.templateId = ModelWeldingWorkflow::CreateStableId();
    value.displayName = QStringLiteral("JSON roundtrip template");
    value.modelLibraryName = QStringLiteral("fixture.ply");
    value.modelSha256 = QString(64, QLatin1Char('a'));
    value.placement.anchorModelMm = Eigen::Vector3d(1.5, 2.5, 3.5);
    value.placement.axesModel =
        Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    value.placement.vSlotYawDegrees = -37.5;
    value.placement.longLengthMm = 320.0;
    value.placement.shortLengthMm = 180.0;
    value.createdAtUtc = QStringLiteral("2026-07-21T01:02:03.000Z");
    value.updatedAtUtc = value.createdAtUtc;

    ModelWeldingFeatureStation solve;
    solve.stationId = QStringLiteral("S01");
    solve.displayName = QStringLiteral("solve one");
    solve.role = ModelWeldingStationRole::Solve;
    solve.anchorModelMm = Eigen::Vector3d(10.0, 20.0, 30.0);
    solve.scanDirectionModel = Eigen::Vector3d(0.0, 1.0, 0.0);
    solve.roiHalfExtentMm = Eigen::Vector3d(15.0, 20.0, 8.0);
    solve.candidateConfirmed = true;
    value.stations.push_back(solve);

    ModelWeldingSeamDefinition seam;
    seam.seamId = QStringLiteral("seam-001");
    seam.source = ModelWeldingSeamSource::CadSharedEdge;
    seam.sourceGeometrySha256 = QString(64, QLatin1Char('f'));
    seam.pathModelMm = {
        Eigen::Vector3d(0.0, 0.0, 0.0),
        Eigen::Vector3d(40.0, 0.0, 0.0),
        Eigen::Vector3d(80.0, 10.0, 0.0)
    };
    seam.lengthMm = 40.0 + std::sqrt(1700.0);
    value.seams.push_back(seam);
    return value;
}

ModelWeldingFlowTemplate ProductionTemplate()
{
    ModelWeldingFlowTemplate value = ValidTemplate();
    value.stations.clear();
    value.humanDatumConfirmed = true;
    value.humanCollisionChecked = true;
    const QVector<Eigen::Vector3d> points = {
        Eigen::Vector3d(0.0, 0.0, 0.0),
        Eigen::Vector3d(100.0, 0.0, 0.0),
        Eigen::Vector3d(0.0, 100.0, 0.0),
        Eigen::Vector3d(25.0, 35.0, 40.0)
    };
    for (int index = 0; index < points.size(); ++index)
    {
        ModelWeldingFeatureStation station;
        station.stationId = index < 3
            ? QStringLiteral("S%1").arg(index + 1, 2, 10, QLatin1Char('0'))
            : QStringLiteral("V01");
        station.role = index < 3
            ? ModelWeldingStationRole::Solve : ModelWeldingStationRole::Verify;
        station.anchorModelMm = points.at(index);
        station.scanDirectionModel = Eigen::Vector3d::UnitX();
        station.candidateConfirmed = true;
        value.stations.push_back(station);
    }
    ModelWeldingFeatureStation unconfirmed;
    unconfirmed.stationId = QStringLiteral("S99");
    unconfirmed.role = ModelWeldingStationRole::Solve;
    unconfirmed.anchorModelMm = Eigen::Vector3d(40.0, 40.0, 20.0);
    unconfirmed.candidateConfirmed = false;
    value.stations.push_back(unconfirmed);
    ModelWeldingFeatureStation backup = unconfirmed;
    backup.stationId = QStringLiteral("B01");
    backup.role = ModelWeldingStationRole::Backup;
    backup.candidateConfirmed = true;
    value.stations.push_back(backup);
    value.seams.first().humanConfirmed = true;
    return value;
}

void TestProductionTeachingRules()
{
    QString error;
    const ModelWeldingFlowTemplate modelTemplate = ProductionTemplate();
    Check(ModelWeldingWorkflow::ValidateTemplateStructure(modelTemplate, error, true),
        "complete production template should pass");

    ModelWeldingFlowTemplate missingConfirmedSeam = modelTemplate;
    missingConfirmedSeam.seams.first().humanConfirmed = false;
    Check(!ModelWeldingWorkflow::ValidateTemplateStructure(
            missingConfirmedSeam, error, true)
            && error.contains(QStringLiteral("人工确认焊缝")),
        "production template must contain a human-confirmed seam");

    ModelWeldingRobotTeaching draft;
    draft.teachingId = ModelWeldingWorkflow::CreateStableId();
    draft.templateId = modelTemplate.templateId;
    draft.templateRevision = modelTemplate.revision;
    draft.templateRecordSha256 =
        ModelWeldingWorkflow::TemplateRecordSha256(modelTemplate, error);
    draft.robotName = QStringLiteral("RobotA");
    draft.robotEndpoint = QStringLiteral("tcp://192.0.2.10:1000");
    draft.robotModelId = QStringLiteral("step.sa10-2000h");
    draft.sourceStepSha256 = QString(64, QLatin1Char('d'));
    draft.collisionProfileSha256 = QString(64, QLatin1Char('e'));
    draft.cameraSection = QStringLiteral("CAMERA0");
    draft.handEyeSha256 = QString(64, QLatin1Char('b'));
    draft.tool1Sha256 = QString(64, QLatin1Char('c'));
    Check(ModelWeldingWorkflow::ValidateTeachingStructure(
        draft, &modelTemplate, error, false),
        "draft validation must not require confirmed stations to be taught");

    for (int index = 0; index < 4; ++index)
    {
        ModelWeldingScanTeaching scan;
        scan.stationId = modelTemplate.stations.at(index).stationId;
        scan.startTaught = true;
        scan.endTaught = true;
        scan.startPulseTaught = true;
        scan.startPose = T_ROBOT_COORS(
            index * 50.0, 0.0, 200.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        scan.endPose = T_ROBOT_COORS(
            index * 50.0 + 20.0, 0.0, 200.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        draft.scans.push_back(scan);
    }
    Check(ModelWeldingWorkflow::ValidateTeachingStructure(
        draft, &modelTemplate, error, true),
        "unconfirmed and backup stations must be exempt from production teaching");

    ModelWeldingRobotTeaching missingRobotModelIdentity = draft;
    missingRobotModelIdentity.robotModelId.clear();
    missingRobotModelIdentity.sourceStepSha256.clear();
    missingRobotModelIdentity.collisionProfileSha256.clear();
    Check(!ModelWeldingWorkflow::ValidateTeachingStructure(
        missingRobotModelIdentity, &modelTemplate, error, true),
        "production teaching must bind the robot model STEP and collision profile identity");

    ModelWeldingRobotTeaching missingEndpoint = draft;
    missingEndpoint.robotEndpoint.clear();
    Check(!ModelWeldingWorkflow::ValidateTeachingStructure(
        missingEndpoint, &modelTemplate, error, true),
        "production teaching must bind a persistent robot endpoint");

    ModelWeldingRobotTeaching wrongBinding = draft;
    wrongBinding.templateRecordSha256 = QString(64, QLatin1Char('d'));
    Check(!ModelWeldingWorkflow::ValidateTeachingStructure(
        wrongBinding, &modelTemplate, error, true),
        "wrong template hash must reject teaching");
}

void TestAdditionalStrictValidation()
{
    QString error;
    ModelWeldingFlowTemplate unicodeHash = ValidTemplate();
    unicodeHash.modelSha256 = QString(63, QLatin1Char('a')) + QChar(0x0661);
    Check(ModelWeldingWorkflow::EncodeTemplate(unicodeHash, error).isEmpty(),
        "non-ASCII digit must not be accepted in SHA-256");

    ModelWeldingFlowTemplate badInheritance = ValidTemplate();
    badInheritance.inheritedFromTemplateId = ModelWeldingWorkflow::CreateStableId();
    badInheritance.inheritedFromRevision = 9007199254740992ULL;
    badInheritance.inheritedFromRecordSha256 = QString(64, QLatin1Char('e'));
    Check(!ModelWeldingWorkflow::ValidateTemplateStructure(
        badInheritance, error, false),
        "inheritance revision outside exact JSON range must fail");

    ModelWeldingFlowTemplate selfInheritance = ValidTemplate();
    selfInheritance.inheritedFromTemplateId = selfInheritance.templateId;
    selfInheritance.inheritedFromRevision = 1;
    selfInheritance.inheritedFromRecordSha256 = QString(64, QLatin1Char('e'));
    Check(!ModelWeldingWorkflow::ValidateTemplateStructure(
        selfInheritance, error, false),
        "template must not inherit from itself");

    ModelWeldingFlowTemplate nonCanonicalYaw = ValidTemplate();
    nonCanonicalYaw.placement.vSlotYawDegrees = 180.0;
    Check(!ModelWeldingWorkflow::ValidateTemplateStructure(
        nonCanonicalYaw, error, false),
        "V-slot yaw at positive 180 degrees must be normalized before storage");
    nonCanonicalYaw.placement.vSlotYawDegrees = -180.0;
    Check(ModelWeldingWorkflow::ValidateTemplateStructure(
        nonCanonicalYaw, error, false),
        "V-slot yaw at negative 180 degrees should be canonical");
    nonCanonicalYaw.placement.vSlotYawDegrees =
        (std::numeric_limits<double>::infinity)();
    Check(!ModelWeldingWorkflow::ValidateTemplateStructure(
        nonCanonicalYaw, error, false),
        "non-finite V-slot yaw must be rejected");

    ModelWeldingFlowTemplate reverseWithoutSeedIdentity = ValidTemplate();
    reverseWithoutSeedIdentity.seams.first().source =
        ModelWeldingSeamSource::ReverseMeshSeedProjection;
    Check(!ModelWeldingWorkflow::ValidateTemplateStructure(
        reverseWithoutSeedIdentity, error, false),
        "reverse-mesh projection must bind the seed file identity");

    error = QStringLiteral("stale error");
    ModelWeldingFlowTemplate missing;
    Check(ModelWeldingWorkflow::LoadTemplate(
        ModelWeldingWorkflow::CreateStableId(), missing, error)
        == ModelWeldingWorkflow::LoadStatus::NotFound,
        "missing template should return NotFound");
    Check(error.isEmpty(), "NotFound must clear stale caller error");

    WorkpieceMeshBuilder::Mesh symmetric;
    for (float x : { -100.0f, 100.0f })
    {
        for (float y : { -10.0f, 10.0f })
        {
            for (float z : { -10.0f, 10.0f })
            {
                symmetric.vertices.push_back(Eigen::Vector3f(x, y, z));
            }
        }
    }
    symmetric.indices = { 0, 1, 2 };
    ModelWeldingPlacementGuide guide;
    Check(!ModelWeldingWorkflow::GeneratePlacementGuide(
        symmetric, 300.0, 180.0, guide, error),
        "axis-symmetric PCA guide must require manual datum direction");
}

void TestJsonRoundTrips()
{
    QString error;
    const ModelWeldingFlowTemplate input = ValidTemplate();
    const QByteArray encoded = ModelWeldingWorkflow::EncodeTemplate(input, error);
    Check(!encoded.isEmpty(), "valid template should encode");
    Check(error.isEmpty(), "valid template encode should not report an error");
    ModelWeldingFlowTemplate decoded;
    Check(ModelWeldingWorkflow::DecodeTemplate(encoded, decoded, error), "encoded template should decode");
    Check(decoded.templateId == input.templateId, "template id should survive JSON roundtrip");
    Check(decoded.modelSha256 == input.modelSha256, "model hash should survive JSON roundtrip");
    Check(decoded.stations.size() == 1 && decoded.stations.first().stationId == QStringLiteral("S01"),
        "station identity should survive JSON roundtrip");
    Check(decoded.seams.size() == 1
            && decoded.seams.first().seamId == QStringLiteral("seam-001")
            && decoded.seams.first().source == ModelWeldingSeamSource::CadSharedEdge
            && decoded.seams.first().sourceGeometrySha256 == QString(64, QLatin1Char('f'))
            && decoded.seams.first().pathModelMm.size() == 3,
        "seam identity, provenance, and path should survive JSON roundtrip");
    for (const ModelWeldingSeamSource source : {
             ModelWeldingSeamSource::CadCorrugatedButtJoint,
             ModelWeldingSeamSource::CadCorrugatedBaseJoint })
    {
        ModelWeldingFlowTemplate semanticTemplate = input;
        semanticTemplate.seams.first().source = source;
        const QByteArray semanticJson = ModelWeldingWorkflow::EncodeTemplate(
            semanticTemplate, error);
        ModelWeldingFlowTemplate semanticDecoded;
        Check(!semanticJson.isEmpty()
                && ModelWeldingWorkflow::DecodeTemplate(
                    semanticJson, semanticDecoded, error)
                && semanticDecoded.seams.size() == 1
                && semanticDecoded.seams.first().source == source,
            "corrugated CAD seam source should survive JSON roundtrip");
    }
    Check((decoded.placement.anchorModelMm - input.placement.anchorModelMm).norm() < 1.0e-12,
        "placement anchor should survive JSON roundtrip");
    Check((decoded.placement.axesModel - input.placement.axesModel).cwiseAbs().maxCoeff() < 1.0e-12,
        "placement axes should survive JSON roundtrip");
    Check(Near(decoded.placement.vSlotYawDegrees, input.placement.vSlotYawDegrees, 1.0e-12),
        "V-slot yaw should survive JSON roundtrip");

    QJsonObject legacyRoot = QJsonDocument::fromJson(encoded).object();
    QJsonObject legacyPlacement = legacyRoot.value(QStringLiteral("placement")).toObject();
    legacyPlacement.remove(QStringLiteral("vSlotYawDegrees"));
    legacyRoot.insert(QStringLiteral("placement"), legacyPlacement);
    ModelWeldingFlowTemplate legacyDecoded;
    Check(ModelWeldingWorkflow::DecodeTemplate(
        QJsonDocument(legacyRoot).toJson(QJsonDocument::Compact), legacyDecoded, error),
        "legacy template without V-slot yaw should decode");
    Check(Near(legacyDecoded.placement.vSlotYawDegrees, 0.0, 1.0e-12),
        "legacy template without V-slot yaw should default to zero");

    QJsonObject schema1Root = QJsonDocument::fromJson(encoded).object();
    schema1Root.insert(QStringLiteral("schemaVersion"), 1);
    schema1Root.remove(QStringLiteral("seams"));
    ModelWeldingFlowTemplate schema1Decoded;
    Check(ModelWeldingWorkflow::DecodeTemplate(
        QJsonDocument(schema1Root).toJson(QJsonDocument::Compact), schema1Decoded, error),
        "schema v1 template without seams should migrate as a draft");
    Check(schema1Decoded.schemaVersion == ModelWeldingFlowTemplate::SchemaVersion
            && schema1Decoded.seams.isEmpty(),
        "schema v1 migration must require fresh seam extraction and confirmation");

    ModelWeldingFlowTemplate zeroYaw = input;
    zeroYaw.placement.vSlotYawDegrees = 0.0;
    const QByteArray zeroYawEncoded = ModelWeldingWorkflow::EncodeTemplate(zeroYaw, error);
    Check(!zeroYawEncoded.isEmpty(), "zero V-slot yaw template should encode");
    const QJsonObject zeroYawRoot = QJsonDocument::fromJson(zeroYawEncoded).object();
    Check(!zeroYawRoot.value(QStringLiteral("placement")).toObject()
            .contains(QStringLiteral("vSlotYawDegrees")),
        "zero V-slot yaw should stay omitted for legacy JSON and hash compatibility");

    QJsonObject invalidYawRoot = legacyRoot;
    QJsonObject invalidYawPlacement = invalidYawRoot.value(QStringLiteral("placement")).toObject();
    invalidYawPlacement.insert(QStringLiteral("vSlotYawDegrees"), QStringLiteral("90"));
    invalidYawRoot.insert(QStringLiteral("placement"), invalidYawPlacement);
    ModelWeldingFlowTemplate invalidYawDecoded;
    Check(!ModelWeldingWorkflow::DecodeTemplate(
        QJsonDocument(invalidYawRoot).toJson(QJsonDocument::Compact), invalidYawDecoded, error),
        "non-numeric V-slot yaw must be rejected");

    ModelWeldingRobotTeaching teaching;
    teaching.teachingId = ModelWeldingWorkflow::CreateStableId();
    teaching.templateId = input.templateId;
    teaching.templateRevision = input.revision;
    teaching.templateRecordSha256 =
        ModelWeldingWorkflow::TemplateRecordSha256(input, error);
    teaching.robotName = QStringLiteral("RobotA");
    teaching.robotEndpoint = QStringLiteral("tcp://192.0.2.10:1000");
    teaching.robotModelId = QStringLiteral("step.sa10-2000h");
    teaching.sourceStepSha256 = QString(64, QLatin1Char('e'));
    teaching.collisionProfileSha256 = QString(64, QLatin1Char('f'));
    teaching.cameraSection = QStringLiteral("CameraA");
    teaching.handEyeSha256 = QString(64, QLatin1Char('c'));
    teaching.tool1Sha256 = QString(64, QLatin1Char('d'));
    teaching.createdAtUtc = input.createdAtUtc;
    teaching.updatedAtUtc = input.updatedAtUtc;
    ModelWeldingScanTeaching scan;
    scan.stationId = QStringLiteral("S01");
    scan.startTaught = true;
    scan.endTaught = true;
    scan.startPulseTaught = true;
    scan.startPose = T_ROBOT_COORS(1, 2, 3, 4, 5, 6, 7, 8, 9);
    scan.endPose = T_ROBOT_COORS(11, 12, 13, 14, 15, 16, 17, 18, 19);
    scan.startPulse = T_ANGLE_PULSE(1, 2, 3, 4, 5, 6, 7, 8, 9);
    scan.runSpeedMmPerMin = 350.0;
    scan.scanSpeedMmPerMin = 125.0;
    teaching.scans.push_back(scan);

    const QByteArray teachingJson = ModelWeldingWorkflow::EncodeTeaching(teaching, error);
    Check(!teachingJson.isEmpty(), "valid teaching should encode");
    ModelWeldingRobotTeaching decodedTeaching;
    Check(ModelWeldingWorkflow::DecodeTeaching(teachingJson, decodedTeaching, error),
        "encoded teaching should decode");
    Check(decodedTeaching.teachingId == teaching.teachingId, "teaching id should survive JSON roundtrip");
    Check(decodedTeaching.robotEndpoint == teaching.robotEndpoint,
        "robot endpoint should survive JSON roundtrip");
    Check(decodedTeaching.robotModelId == teaching.robotModelId
        && decodedTeaching.sourceStepSha256 == teaching.sourceStepSha256
        && decodedTeaching.collisionProfileSha256 == teaching.collisionProfileSha256,
        "robot model asset identity should survive JSON roundtrip");
    Check(decodedTeaching.scans.size() == 1 && decodedTeaching.scans.first().startTaught,
        "teaching scan flags should survive JSON roundtrip");
    Check(Near(decodedTeaching.scans.first().endPose.dRZ, 16.0, 0.0),
        "teaching pose should survive JSON roundtrip");
    Check(decodedTeaching.scans.first().startPulse.nTPulse == 6,
        "teaching pulse should survive JSON roundtrip");

    QJsonObject legacyObject = QJsonDocument::fromJson(teachingJson).object();
    legacyObject.insert(QStringLiteral("schemaVersion"), 1);
    legacyObject.remove(QStringLiteral("robotEndpoint"));
    ModelWeldingRobotTeaching migratedLegacy;
    Check(ModelWeldingWorkflow::DecodeTeaching(
        QJsonDocument(legacyObject).toJson(QJsonDocument::Compact), migratedLegacy, error),
        "legacy schema v1 teaching should decode as a migration draft");
    Check(migratedLegacy.schemaVersion == ModelWeldingRobotTeaching::SchemaVersion
        && migratedLegacy.robotEndpoint.isEmpty()
        && migratedLegacy.robotModelId.isEmpty()
        && migratedLegacy.sourceStepSha256.isEmpty()
        && migratedLegacy.collisionProfileSha256.isEmpty(),
        "legacy v1 teaching migration must remain endpoint and model-asset unbound");
    Check(!ModelWeldingWorkflow::ValidateTeachingStructure(
            migratedLegacy, &input, error, true),
        "legacy v1 teaching must fail closed at production validation");

    QJsonObject schema2Object = QJsonDocument::fromJson(teachingJson).object();
    schema2Object.insert(QStringLiteral("schemaVersion"), 2);
    schema2Object.remove(QStringLiteral("robotModelId"));
    schema2Object.remove(QStringLiteral("sourceStepSha256"));
    schema2Object.remove(QStringLiteral("collisionProfileSha256"));
    ModelWeldingRobotTeaching migratedSchema2;
    Check(ModelWeldingWorkflow::DecodeTeaching(
        QJsonDocument(schema2Object).toJson(QJsonDocument::Compact), migratedSchema2, error),
        "legacy schema v2 teaching should decode as a migration draft");
    Check(migratedSchema2.robotModelId.isEmpty()
        && migratedSchema2.sourceStepSha256.isEmpty()
        && migratedSchema2.collisionProfileSha256.isEmpty(),
        "legacy v2 teaching must remain robot-model-asset unbound");
    Check(!ModelWeldingWorkflow::ValidateTeachingStructure(
            migratedSchema2, &input, error, true),
        "legacy v2 teaching must fail closed at production validation");

    ModelWeldingRobotTeaching paddedEndpoint = teaching;
    paddedEndpoint.robotEndpoint.prepend(QLatin1Char(' '));
    Check(ModelWeldingWorkflow::EncodeTeaching(paddedEndpoint, error).isEmpty(),
        "padded robot endpoint identity must fail strict validation");

    ModelWeldingRobotTeaching partialModelIdentity = teaching;
    partialModelIdentity.collisionProfileSha256.clear();
    Check(ModelWeldingWorkflow::EncodeTeaching(partialModelIdentity, error).isEmpty(),
        "partial robot model asset identity must fail strict validation");
}

void TestInvalidHashes()
{
    QString error;
    ModelWeldingFlowTemplate invalidTemplate = ValidTemplate();
    invalidTemplate.modelSha256 = QString(63, QLatin1Char('a')) + QLatin1Char('g');
    Check(ModelWeldingWorkflow::EncodeTemplate(invalidTemplate, error).isEmpty(),
        "non-hex model SHA-256 must be rejected");
    Check(!error.isEmpty(), "invalid model SHA-256 should report an error");

    ModelWeldingRobotTeaching invalidTeaching;
    invalidTeaching.teachingId = ModelWeldingWorkflow::CreateStableId();
    invalidTeaching.templateId = invalidTemplate.templateId;
    invalidTeaching.templateRevision = 1;
    invalidTeaching.templateRecordSha256 = QString(63, QLatin1Char('f'));
    invalidTeaching.robotName = QStringLiteral("RobotA");
    error.clear();
    Check(ModelWeldingWorkflow::EncodeTeaching(invalidTeaching, error).isEmpty(),
        "wrong-length template record SHA-256 must be rejected");
    Check(!error.isEmpty(), "invalid template record SHA-256 should report an error");
}
}

// The core translation unit references persistence functions even though this suite deliberately
// tests only deterministic algorithms and JSON. Minimal stubs keep the test independent of the
// user's ConfigStore.db and make any accidental persistence call fail closed.
ConfigDatabase::ReadStatus ConfigDatabase::ReadScopedSettingStatus(
    const QString&, const QString&, const QString&, const QString&, QString*)
{
    return ReadStatus::NotFound;
}

bool ConfigDatabase::WriteScopedSetting(
    const QString&, const QString&, const QString&, const QString&, const QString&, const QString&, bool)
{
    return false;
}

bool ConfigDatabase::CompareAndSwapScopedSetting(
    const QString&,
    const QString&,
    const QString&,
    const QString&,
    const QString*,
    const QString&,
    const QString&,
    bool,
    bool* conflict,
    QString* error)
{
    if (conflict != nullptr)
    {
        *conflict = false;
    }
    if (error != nullptr)
    {
        *error = QStringLiteral("persistence disabled in core test");
    }
    return false;
}

bool ConfigDatabase::CompareAndSwapScopedSettingWithWitness(
    const QString&,
    const QString&,
    const QString&,
    const QString&,
    const QString&,
    const QString&,
    const QString&,
    const QString&,
    const QString&,
    const QString*,
    const QString&,
    const QString&,
    bool,
    bool* conflict,
    QString* error)
{
    if (conflict != nullptr)
    {
        *conflict = false;
    }
    if (error != nullptr)
    {
        *error = QStringLiteral("persistence disabled in core test");
    }
    return false;
}

bool ConfigDatabase::TryListScopedSettingIdsBounded(
    const QString&, const QString&, qsizetype, qsizetype, QStringList* ids)
{
    if (ids != nullptr)
    {
        ids->clear();
    }
    return true;
}

int main()
{
    TestExactRigidFit();
    TestNoisyRigidFit();
    TestCollinearRejection();
    TestScaleRejection();
    TestVerifyPointRejection();
    TestJsonRoundTrips();
    TestInvalidHashes();
    TestProductionTeachingRules();
    TestAdditionalStrictValidation();

    if (g_failures != 0)
    {
        std::cerr << g_failures << " model welding workflow test(s) failed\n";
        return 1;
    }
    std::cout << "PASS: model welding workflow core tests\n";
    return 0;
}
