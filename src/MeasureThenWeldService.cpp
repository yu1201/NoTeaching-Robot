#include "MeasureThenWeldService.h"

#include "AppPaths.h"
#include "CameraFrameCache.h"
#include "ConfigDatabase.h"
#include "FANUCRobotDriver.h"
#include "HandEyeMatrixConfig.h"
#include "MeasureThenWeldRuntimeConfig.h"
#include "OPini.h"
#include "PointCloudExtractionProcessor.h"
#include "PointCloudProofIntegrity.h"
#include "PointCloudProcessingConfig.h"
#include "WorkpieceMeshBuilder.h"
#include "RobotDataHelper.h"
#include "RobotMessage.h"
#include "RobotMotionTimeoutPolicy.h"
#include "RobotOperationLease.h"
#include "RobotPoseTransform.h"
#include "STEPRobotDriver.h"
#include "WeldProcessFile.h"
#include "WeldProcessValidation.h"
#include "WeldSafetyRecoveryStore.h"
#include "WeldSeamCompConfig.h"
#include "groove/framebuffer.h"

#include <QCryptographicHash>
#include <QBuffer>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QHash>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMessageAuthenticationCode>
#include <QMutex>
#include <QMutexLocker>
#include <QRandomGenerator>
#include <QRegularExpression>
#include <QSaveFile>
#include <QSet>
#include <QStringList>
#include <QStringConverter>
#include <QTextStream>
#include <QUuid>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <limits>
#include <mutex>
#include <thread>
#include <utility>

namespace
{
constexpr int FANUC_MOTION_STATE_REG = 93;
constexpr double FANUC_WELD_PATH_SPEED_MM_PER_MIN = 400.0;
constexpr double DEFAULT_WELD_SAFE_MOVE_SPEED_MM_PER_MIN = 1000.0;
constexpr double DEFAULT_DRY_RUN_SPEED_MM_PER_MIN = 1000.0;
constexpr double WELD_SAFE_OFFSET_DISTANCE_MM = 70.0;
constexpr int DEFAULT_CAMERA_READ_FPS = 100;  // 相机参数缺省/无效帧率时的回退值（≈10ms 轮询，与 worker 默认一致）
constexpr qint64 ROBOT_SAMPLE_INTERVAL_MS = 50;
constexpr qint64 CAMERA_ROBOT_MATCH_TAIL_WAIT_MS = 500;
constexpr qint64 CAMERA_ROBOT_MATCH_TAIL_POLL_MS = 10;
constexpr int CAMERA_READY_FRAME_TIMEOUT_MS = 5000;
constexpr qint64 CAMERA_NO_FRAME_FAIL_GAP_US = 1000000;  // 扫描期间连续无新帧超过 1s 判数据不完整、终止流程
constexpr auto RAW_LASER_FILE_NAME = "PreciseLaserPoint.txt";
constexpr auto WORKPIECE_CLOUD_FILE_NAME = "PreciseLaserPoint_WorkpieceCloud.txt";
constexpr auto PRESERVE_PATH_FILE_NAME = "PreciseLaserPoint_PreservePath_2mm.txt";
constexpr auto KEY_POINTS_FILE_NAME = "PreciseLaserPoint_KeyPoints.txt";
constexpr auto CLASSIFIED_FILE_NAME = "PreciseLaserPoint_Classified.txt";
constexpr auto CORNER_COMP_KEY_POINTS_FILE_NAME = "PreciseLaserPoint_CornerComp_KeyPoints.txt";
constexpr auto CORNER_COMP_CLASSIFIED_FILE_NAME = "PreciseLaserPoint_CornerComp_Classified.txt";
constexpr auto CLASSIFIED_NOISE_FILE_NAME = "PreciseLaserPoint_Classified_Noise.txt";
constexpr auto WELD_POSE_FILE_NAME = "PreciseLaserPoint_WeldPose_2mm.txt";
constexpr auto WELD_POSE_SEAM_COMP_FILE_NAME = "PreciseLaserPoint_WeldPose_2mm_SeamComp.txt";
constexpr auto WELD_SEGMENT_KIND_DEBUG_FILE_NAME = "PreciseLaserPointSegmentKind.txt";
constexpr auto MATCH_DEBUG_FILE_NAME = "PreciseLaserPoint_MatchDebug.csv";
// 四种处理方法各自的基础焊道文件：处理成功时落盘，文件存在即表示该目录已按该方法完成焊道生成。
constexpr auto METHOD_TRACK_SDK_CLASS_FILE_NAME = "PreciseLaserPoint_SdkClass.txt";   // ①SDK拐点扩充焊道
constexpr auto METHOD_TRACK_SDK_BASE_FILE_NAME = "PreciseLaserPoint_SdkBase.txt";     // ②SDK基础焊道
constexpr auto METHOD_TRACK_POINT_BASE_FILE_NAME = "PreciseLaserPoint_PointBase.txt"; // ③点云投影提取焊道
constexpr auto METHOD_TRACK_POINT_LASER_FILE_NAME = "PreciseLaserPoint_PointLaser.txt"; // ④相机目标点焊道

constexpr auto SDK_POINT_CLOUD_OUTPUT_DIR_NAME = "SdkPointCloud";
constexpr auto SDK_SEAM_EXTRACTED_FILE_NAME = "PreciseLaserPoint_SdkSeamExtracted.txt";
constexpr auto SDK_SEAM_EXTRACTED_2MM_FILE_NAME = "PreciseLaserPoint_SdkSeamExtracted_2mm.txt";
constexpr auto SDK_BASE_WELD_FILE_NAME = "PreciseLaserPoint_SdkBaseWeld.txt";
constexpr auto SDK_SCHEME_COMPARE_DIR_NAME = "SchemeCompare";
constexpr int POSE_COMP_MATCH_BY_POSE = 0;
constexpr int POSE_COMP_MATCH_BY_SEGMENT_CODE = 1;
constexpr char POSE_COMP_MATCH_MODE_KEY[] = "PoseCompMatchMode";
constexpr int POSE_COMP_SEGMENT_COUNT = 4;
constexpr char POSE_GROUP_COUNT_KEY[] = "PoseCompGroupCount";
constexpr char POSE_ACTIVE_GROUP_INDEX_KEY[] = "ActivePoseCompGroupIndex";
constexpr char CORNER_COMP_ENABLED_KEY[] = "Enabled";
constexpr char INNER_TO_OUTER_CORNER_COMP_KEY[] = "InnerToOuter";
constexpr char INNER_TO_INNER_CORNER_COMP_KEY[] = "InnerToInner";
constexpr char OUTER_TO_OUTER_CORNER_COMP_KEY[] = "OuterToOuter";
constexpr char OUTER_TO_INNER_CORNER_COMP_KEY[] = "OuterToInner";
constexpr double DEFAULT_FINAL_WELD_TRAJECTORY_SAMPLE_STEP_MM = 4.0;
constexpr double DEFAULT_RESUME_BACKTRACK_DISTANCE_MM = 5.0;

double NormalizeFinalWeldTrajectorySampleStepMm(double value)
{
    if (!std::isfinite(value) || value <= 0.0)
    {
        return DEFAULT_FINAL_WELD_TRAJECTORY_SAMPLE_STEP_MM;
    }
    return std::clamp(value, 0.5, 100.0);
}

int NormalizePoseCompMatchMode(int mode)
{
    return mode == POSE_COMP_MATCH_BY_SEGMENT_CODE
        ? POSE_COMP_MATCH_BY_SEGMENT_CODE
        : POSE_COMP_MATCH_BY_POSE;
}

QString PoseCompMatchModeDisplayName(int mode)
{
    return NormalizePoseCompMatchMode(mode) == POSE_COMP_MATCH_BY_SEGMENT_CODE
        ? QStringLiteral("按四类段属性")
        : QStringLiteral("按姿态匹配");
}

std::string ToUtf8StdString(const QString& text)
{
    const QByteArray bytes = text.toUtf8();
    return std::string(bytes.constData(), static_cast<size_t>(bytes.size()));
}

bool InvalidateStoredWeldResumeCheckpointImpl(const QString& robotName, QString& error)
{
    return WeldSafetyRecoveryStore::InvalidateIfNoPending(robotName, error);
}

QString ComputeFileSha256ForResumeGate(const QString& filePath, QString& error)
{
    error.clear();
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly))
    {
        error = QString("无法读取V2续焊绑定轨迹：%1").arg(filePath);
        return QString();
    }
    QCryptographicHash hash(QCryptographicHash::Sha256);
    if (!hash.addData(&file))
    {
        error = QString("读取V2续焊绑定轨迹计算SHA256失败：%1").arg(filePath);
        return QString();
    }
    return QString::fromLatin1(hash.result().toHex()).toLower();
}

constexpr auto POINT_CLOUD_QUALITY_GATE_FILE_NAME = "PreciseLaserPoint_QualityGate.json";
constexpr auto POINT_CLOUD_QUALITY_ALGORITHM_REVISION = "pcq-v3-20260712-a";
constexpr int POINT_CLOUD_QUALITY_SCHEMA_VERSION = 3;
constexpr int POINT_CLOUD_PRODUCTION_CONTEXT_REVISION = 1;
constexpr auto POINT_CLOUD_PROOF_SECURITY_MODULE = "PointCloudProofSecurity";
constexpr auto POINT_CLOUD_PROOF_HMAC_KEY_NAME = "HmacKeyV1";
constexpr auto POINT_CLOUD_PROOF_RECEIPT_SCOPE = "pointcloud-proof-receipt";
constexpr auto POINT_CLOUD_PROOF_RECEIPT_MODULE = "QualityGateReceiptV1";
constexpr auto POINT_CLOUD_PROOF_RECEIPT_KEY = "Receipt";
constexpr qint64 POINT_CLOUD_PROOF_MAX_AGE_SECONDS = 24 * 60 * 60;
constexpr qint64 POINT_CLOUD_PROOF_MAX_FUTURE_SKEW_SECONDS = 5 * 60;
constexpr double FINAL_MAX_POSITION_STEP_MM = 50.0;
constexpr double FINAL_MAX_CONTROLLER_EULER_STEP_DEG = 90.0;
constexpr double FINAL_MAX_PHYSICAL_ORIENTATION_STEP_DEG = 90.0;
constexpr double FINAL_MIN_PRECOMP_LENGTH_RATIO = 0.93;
constexpr double FINAL_MAX_PRECOMP_LENGTH_RATIO = 1.25;
constexpr double FINAL_MIN_MATCHED_ARC_RATIO = 0.90;
constexpr double FINAL_MIN_SOURCE_UNIQUE_COVERAGE_RATIO = 0.55;
constexpr double FINAL_MIN_SOURCE_ARC_SPAN_RATIO = 0.90;
constexpr double FINAL_MAX_SOURCE_DISPLACEMENT_MM = 25.0;
constexpr double FINAL_MAX_SOURCE_CONTROLLER_EULER_DELTA_DEG = 60.0;
constexpr double FINAL_MAX_SOURCE_PHYSICAL_ORIENTATION_DELTA_DEG = 60.0;

struct SyntheticPoseAuthorization
{
    QString robotName;
    QString sha256;
    qint64 size = -1;
};

struct PointCloudProductionContext
{
    QString scanRunId;
    QString scanStartedUtc;
    QString robotEndpoint;
    QString cameraSection;
    QString handEyeSha256;
    QString origin;
};

QMutex g_syntheticPoseAuthorizationMutex;
QHash<QString, SyntheticPoseAuthorization> g_syntheticPoseAuthorizations;
QMutex g_pointCloudProofSecurityMutex;

class PointCloudProofReplacementSession
{
public:
    PointCloudProofReplacementSession() = default;
    PointCloudProofReplacementSession(const PointCloudProofReplacementSession&) = delete;
    PointCloudProofReplacementSession& operator=(const PointCloudProofReplacementSession&) = delete;

    ~PointCloudProofReplacementSession()
    {
        if (m_active)
        {
            PointCloudProofIntegrity::AbandonProofReplacement(m_proofPath);
        }
    }

    void Arm(const QString& proofPath)
    {
        m_proofPath = proofPath;
        m_active = true;
    }

    bool Complete(QString& error)
    {
        if (!m_active)
        {
            error = QStringLiteral("点云证明替换会话未激活，禁止解除拒绝闭锁。");
            return false;
        }
        if (!PointCloudProofIntegrity::CompleteProofReplacement(m_proofPath, error))
        {
            return false;
        }
        m_active = false;
        return true;
    }

private:
    QString m_proofPath;
    bool m_active = false;
};

QString PoseAuthorizationPathKey(const QString& filePath)
{
    QString key = QDir::cleanPath(QFileInfo(QDir::fromNativeSeparators(filePath)).absoluteFilePath());
#ifdef Q_OS_WIN
    key = key.toLower();
#endif
    return key;
}

bool IsSha256Text(const QString& value)
{
    static const QRegularExpression pattern(QStringLiteral("^[0-9a-fA-F]{64}$"));
    return pattern.match(value).hasMatch();
}

QString HandEyeMatrixContentSha256(const HandEyeMatrixConfig& calibration)
{
    QByteArray canonical;
    canonical.reserve(512);
    canonical += "robotType=" + QByteArray::number(calibration.robotType) + '\n';
    for (int row = 0; row < 3; ++row)
    {
        for (int column = 0; column < 3; ++column)
        {
            double value = calibration.rotation(row, column);
            if (value == 0.0)
            {
                value = 0.0; // Canonicalize negative zero.
            }
            canonical += "r" + QByteArray::number(row) + QByteArray::number(column)
                + "=" + QByteArray::number(value, 'g', 17) + '\n';
        }
    }
    for (int index = 0; index < 3; ++index)
    {
        double value = calibration.translation(index);
        if (value == 0.0)
        {
            value = 0.0;
        }
        canonical += "t" + QByteArray::number(index)
            + "=" + QByteArray::number(value, 'g', 17) + '\n';
    }
    return QString::fromLatin1(QCryptographicHash::hash(
        canonical, QCryptographicHash::Sha256).toHex()).toLower();
}

PointCloudProductionContext BuildLivePointCloudProductionContext(
    const RobotDriverAdaptor* driver,
    const QString& cameraSection,
    const HandEyeMatrixConfig& calibration)
{
    PointCloudProductionContext context;
    context.scanRunId = QUuid::createUuid().toString(QUuid::WithoutBraces).toLower();
    context.scanStartedUtc = QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs);
    context.robotEndpoint = RobotOperationLease::PersistentEndpointIdentity(driver);
    context.cameraSection = cameraSection.trimmed();
    context.handEyeSha256 = HandEyeMatrixContentSha256(calibration);
    context.origin = QStringLiteral("liveRobotCameraScan");
    return context;
}

QJsonObject PointCloudProductionContextToJson(const PointCloudProductionContext& context)
{
    QJsonObject result;
    result.insert("contextRevision", POINT_CLOUD_PRODUCTION_CONTEXT_REVISION);
    result.insert("origin", context.origin);
    result.insert("scanRunId", context.scanRunId);
    result.insert("scanStartedUtc", context.scanStartedUtc);
    result.insert("robotEndpoint", context.robotEndpoint);
    result.insert("cameraSection", context.cameraSection);
    result.insert("handEyeSha256", context.handEyeSha256);
    return result;
}

PointCloudProductionContext PointCloudProductionContextFromJson(const QJsonObject& object)
{
    PointCloudProductionContext context;
    context.origin = object.value("origin").toString();
    context.scanRunId = object.value("scanRunId").toString();
    context.scanStartedUtc = object.value("scanStartedUtc").toString();
    context.robotEndpoint = object.value("robotEndpoint").toString();
    context.cameraSection = object.value("cameraSection").toString();
    context.handEyeSha256 = object.value("handEyeSha256").toString().toLower();
    return context;
}

bool ParseStrictUtcTimestamp(const QString& text, QDateTime& value)
{
    if (!text.endsWith(QLatin1Char('Z'), Qt::CaseSensitive))
    {
        return false;
    }
    value = QDateTime::fromString(text, Qt::ISODateWithMs);
    if (!value.isValid())
    {
        value = QDateTime::fromString(text, Qt::ISODate);
    }
    return value.isValid() && value.toUTC().toString(Qt::ISODate).endsWith(QLatin1Char('Z'));
}

bool ValidatePointCloudProductionContext(
    const QJsonObject& contextObject,
    const QString& proofCreatedUtc,
    const QString& expectedRobotEndpoint,
    const QString& expectedCameraSection,
    const QString& expectedHandEyeSha256,
    PointCloudProductionContext* validatedContext,
    QString& error)
{
    if (contextObject.value("contextRevision").toInt(-1)
            != POINT_CLOUD_PRODUCTION_CONTEXT_REVISION)
    {
        error = QStringLiteral("点云质量证明缺少受支持的生产上下文版本；旧证明禁止运动，请重新扫描。");
        return false;
    }
    const PointCloudProductionContext context = PointCloudProductionContextFromJson(contextObject);
    const QUuid runId(context.scanRunId);
    if (context.origin != QStringLiteral("liveRobotCameraScan")
        || runId.isNull()
        || runId.toString(QUuid::WithoutBraces).compare(context.scanRunId, Qt::CaseInsensitive) != 0
        || context.robotEndpoint.trimmed().isEmpty()
        || context.cameraSection.trimmed().isEmpty()
        || !IsSha256Text(context.handEyeSha256))
    {
        error = QStringLiteral("点云质量证明的扫描运行、机器人端点或手眼标定上下文不完整；禁止运动，请重新扫描。");
        return false;
    }

    QDateTime createdUtc;
    QDateTime scanStartedUtc;
    if (!ParseStrictUtcTimestamp(proofCreatedUtc, createdUtc)
        || !ParseStrictUtcTimestamp(context.scanStartedUtc, scanStartedUtc))
    {
        error = QStringLiteral("点云质量证明时间不是明确 UTC 时间；禁止运动，请重新扫描/重建。");
        return false;
    }
    createdUtc = createdUtc.toUTC();
    scanStartedUtc = scanStartedUtc.toUTC();
    const QDateTime nowUtc = QDateTime::currentDateTimeUtc();
    const qint64 proofAgeSeconds = createdUtc.secsTo(nowUtc);
    const qint64 scanAgeSeconds = scanStartedUtc.secsTo(nowUtc);
    if (proofAgeSeconds > POINT_CLOUD_PROOF_MAX_AGE_SECONDS
        || scanAgeSeconds > POINT_CLOUD_PROOF_MAX_AGE_SECONDS)
    {
        error = QStringLiteral("点云质量证明或其原始扫描已超过 24 小时新鲜度，禁止运动；请重新扫描。");
        return false;
    }
    if (proofAgeSeconds < -POINT_CLOUD_PROOF_MAX_FUTURE_SKEW_SECONDS
        || scanAgeSeconds < -POINT_CLOUD_PROOF_MAX_FUTURE_SKEW_SECONDS
        || scanStartedUtc > createdUtc.addSecs(POINT_CLOUD_PROOF_MAX_FUTURE_SKEW_SECONDS))
    {
        error = QStringLiteral("点云质量证明时间位于未来或扫描/证明时序异常，禁止运动；请校时后重新扫描。");
        return false;
    }
    if (!expectedRobotEndpoint.trimmed().isEmpty()
        && context.robotEndpoint.compare(expectedRobotEndpoint.trimmed(), Qt::CaseInsensitive) != 0)
    {
        error = QStringLiteral("点云质量证明绑定的机器人持久端点与当前机器人不一致，禁止运动。");
        return false;
    }
    if (context.cameraSection != expectedCameraSection
        || context.handEyeSha256.compare(expectedHandEyeSha256, Qt::CaseInsensitive) != 0)
    {
        error = QStringLiteral("当前测量相机或已验证手眼标定内容与扫描证明不一致；请重新扫描。");
        return false;
    }
    if (validatedContext != nullptr)
    {
        *validatedContext = context;
    }
    return true;
}

bool LoadCurrentPointCloudContextExpectations(
    const QString& robotName,
    const RobotDriverAdaptor* driver,
    bool requireDriverEndpoint,
    QString& robotEndpoint,
    QString& cameraSection,
    QString& handEyeSha256,
    QString& error)
{
    robotEndpoint = RobotOperationLease::PersistentEndpointIdentity(driver);
    if (requireDriverEndpoint && robotEndpoint.isEmpty())
    {
        error = QStringLiteral("当前机器人没有有效的持久 TCP 端点，禁止生成或继承生产质量证明。");
        return false;
    }
    cameraSection = RobotDataHelper::MeasureCameraSection(robotName);
    HandEyeMatrixConfig calibration;
    QString calibrationPath;
    if (!LoadExistingValidatedHandEyeMatrixConfig(
            robotName,
            cameraSection,
            calibration,
            &error,
            &calibrationPath))
    {
        error = QStringLiteral("无法加载当前严格验证的手眼标定，禁止使用点云生产证明：") + error;
        return false;
    }
    handEyeSha256 = HandEyeMatrixContentSha256(calibration);
    if (!IsSha256Text(handEyeSha256))
    {
        error = QStringLiteral("当前手眼标定内容指纹生成失败，禁止使用点云生产证明。");
        return false;
    }
    return true;
}

bool RegisterSyntheticPoseAuthorization(
    const QString& posePath,
    const QString& robotName,
    const QString& sha256,
    qint64 size,
    QString& error)
{
    if (!IsSha256Text(sha256) || size <= 0 || robotName.trimmed().isEmpty())
    {
        error = QStringLiteral("登记虚拟焊道进程内授权失败：路径、机器人、大小或 SHA256 无效。");
        return false;
    }
    SyntheticPoseAuthorization authorization;
    authorization.robotName = robotName.trimmed();
    authorization.sha256 = sha256.toLower();
    authorization.size = size;
    QMutexLocker<QMutex> locker(&g_syntheticPoseAuthorizationMutex);
    g_syntheticPoseAuthorizations.insert(PoseAuthorizationPathKey(posePath), authorization);
    return true;
}

void RevokeSyntheticPoseAuthorization(const QString& posePath)
{
    QMutexLocker<QMutex> locker(&g_syntheticPoseAuthorizationMutex);
    g_syntheticPoseAuthorizations.remove(PoseAuthorizationPathKey(posePath));
}

bool VerifySyntheticPoseAuthorization(
    const QString& posePath,
    const QString& expectedRobotName,
    const QString& loadedSha256,
    qint64 loadedSize,
    QString& error)
{
    QMutexLocker<QMutex> locker(&g_syntheticPoseAuthorizationMutex);
    const auto it = g_syntheticPoseAuthorizations.constFind(PoseAuthorizationPathKey(posePath));
    if (it == g_syntheticPoseAuthorizations.cend())
    {
        error = QStringLiteral("虚拟焊道没有本进程生成授权；重启、换文件或手工创建后必须重新生成。");
        return false;
    }
    const SyntheticPoseAuthorization& authorization = it.value();
    if (authorization.robotName.compare(expectedRobotName.trimmed(), Qt::CaseInsensitive) != 0
        || authorization.size != loadedSize
        || authorization.sha256 != loadedSha256.toLower())
    {
        error = QStringLiteral("虚拟焊道路径、机器人、大小或 SHA256 与本进程生成记录不一致。");
        return false;
    }
    return true;
}

QJsonArray StringListToJsonArray(const QStringList& values)
{
    QJsonArray result;
    for (const QString& value : values)
    {
        result.push_back(value);
    }
    return result;
}

QJsonObject BuildPointCloudQualityThresholds(const PointCloudProcessingConfig::Settings& settings)
{
    QJsonObject thresholds;
    thresholds.insert("profileVersion", PointCloudProcessingConfig::CURRENT_VALIDATION_PROFILE_VERSION);
    thresholds.insert("algorithmRevision", QString::fromLatin1(POINT_CLOUD_QUALITY_ALGORITHM_REVISION));
    thresholds.insert("processingMode", PointCloudProcessingConfig::ModeConfigValue(settings.mode));
    thresholds.insert("sampleStepMm",
        settings.mode == PointCloudProcessingConfig::Mode::ExternalCorrugatedSheet
            ? settings.resampleStepMm
            : settings.fitSampleStepMm);
    thresholds.insert("minFinitePointCount", settings.validationMinFinitePointCount);
    thresholds.insert("minProjectedSpanMm", settings.validationMinProjectedSpanMm);
    thresholds.insert("minStationCoverageRatio", settings.validationMinStationCoverageRatio);
    thresholds.insert("minLongestContinuousRatio", settings.validationMinLongestContinuousRatio);
    thresholds.insert("maxRejectedRatio", settings.validationMaxRejectedRatio);
    thresholds.insert("maxMedianResidualMm", settings.validationMaxMedianResidualMm);
    thresholds.insert("maxP95ResidualMm", settings.validationMaxP95ResidualMm);
    thresholds.insert("residualInlierThresholdMm", settings.validationResidualInlierThresholdMm);
    thresholds.insert("minResidualInlierRatio", settings.validationMinResidualInlierRatio);
    thresholds.insert("minKeyPointCount", settings.validationMinKeyPointCount);
    thresholds.insert("minCornerCount", settings.validationMinCornerCount);
    thresholds.insert("minSegmentWarningMm", settings.validationMinSegmentLengthMm);
    thresholds.insert("minNonLapSegmentHardMm", 3.0);
    thresholds.insert("minLapStepSegmentHardMm", 0.25);
    thresholds.insert("minEndpointAdjacentSegmentHardMm", 0.25);
    thresholds.insert("minOutputPointCount", settings.validationMinOutputPointCount);
    thresholds.insert("minOutputLengthRatio", settings.validationMinOutputLengthRatio);
    thresholds.insert("maxOutputStepHardMm", FINAL_MAX_POSITION_STEP_MM);
    thresholds.insert("maxFinalControllerEulerStepHardDeg", FINAL_MAX_CONTROLLER_EULER_STEP_DEG);
    thresholds.insert("maxFinalPhysicalOrientationStepHardDeg", FINAL_MAX_PHYSICAL_ORIENTATION_STEP_DEG);
    thresholds.insert("minFinalToPreCompLengthRatio", FINAL_MIN_PRECOMP_LENGTH_RATIO);
    thresholds.insert("maxFinalToPreCompLengthRatio", FINAL_MAX_PRECOMP_LENGTH_RATIO);
    thresholds.insert("minFinalMatchedArcRatio", FINAL_MIN_MATCHED_ARC_RATIO);
    thresholds.insert("minFinalSourceUniqueCoverageRatio", FINAL_MIN_SOURCE_UNIQUE_COVERAGE_RATIO);
    thresholds.insert("minFinalSourceArcSpanRatio", FINAL_MIN_SOURCE_ARC_SPAN_RATIO);
    thresholds.insert("maxFinalSourceDisplacementHardMm", FINAL_MAX_SOURCE_DISPLACEMENT_MM);
    thresholds.insert("maxFinalSourceControllerEulerDeltaHardDeg",
        FINAL_MAX_SOURCE_CONTROLLER_EULER_DELTA_DEG);
    thresholds.insert("maxFinalSourcePhysicalOrientationDeltaHardDeg",
        FINAL_MAX_SOURCE_PHYSICAL_ORIENTATION_DELTA_DEG);
    // 保持“全部开启”的既有 proof 阈值对象字节兼容；只有显式关闭时才写入 false。
    if (!settings.validationCoverageEnabled)
    {
        thresholds.insert("coverageEnabled", false);
    }
    if (!settings.validationContinuityEnabled)
    {
        thresholds.insert("continuityEnabled", false);
    }
    if (!settings.validationDenoiseRatioEnabled)
    {
        thresholds.insert("denoiseRatioEnabled", false);
    }
    if (!settings.validationResidualEnabled)
    {
        thresholds.insert("residualEnabled", false);
    }
    if (!settings.validationKeyPointEnabled)
    {
        thresholds.insert("keyPointEnabled", false);
    }
    if (!settings.validationOutputEnabled)
    {
        thresholds.insert("outputEnabled", false);
    }
    return thresholds;
}

QJsonObject BuildSafetyGateRecords(const PointCloudProcessingConfig::Settings& settings)
{
    // 这些开关仅记录操作员配置，不参与策略摘要、证明授权或运动判定。
    // 固定的证明、身份、轨迹和运动前校验始终由生产流程执行。
    QJsonObject records;
    records.insert("behaviorVersion",
        PointCloudProcessingConfig::CURRENT_SAFETY_GATE_BEHAVIOR_VERSION);
    records.insert("proofIntegrityEnabled", settings.safetyGateProofIntegrityEnabled);
    records.insert("productionPurposeEnabled", settings.safetyGateProductionPurposeEnabled);
    records.insert("robotNameBindingEnabled", settings.safetyGateRobotNameBindingEnabled);
    records.insert("caseBindingEnabled", settings.safetyGateCaseBindingEnabled);
    records.insert("endpointBindingEnabled", settings.safetyGateEndpointBindingEnabled);
    records.insert("cameraHandEyeBindingEnabled", settings.safetyGateCameraHandEyeBindingEnabled);
    records.insert("freshnessEnabled", settings.safetyGateFreshnessEnabled);
    records.insert("policySnapshotEnabled", settings.safetyGatePolicySnapshotEnabled);
    records.insert("inputEvidenceEnabled", settings.safetyGateInputEvidenceEnabled);
    records.insert("authorizedPoseIdentityEnabled", settings.safetyGateAuthorizedPoseIdentityEnabled);
    records.insert("trajectoryStructureEnabled", settings.safetyGateTrajectoryStructureEnabled);
    records.insert("motionPrecheckEnabled", settings.safetyGateMotionPrecheckEnabled);
    return records;
}

QString PointCloudQualityPolicyRevision(const PointCloudProcessingConfig::Settings& settings)
{
    QJsonObject policy;
    policy.insert("policy", PointCloudProcessingConfig::ValidationPolicyConfigValue(settings.validationPolicy));
    policy.insert("thresholds", BuildPointCloudQualityThresholds(settings));
    return QString::fromLatin1(QCryptographicHash::hash(
        QJsonDocument(policy).toJson(QJsonDocument::Compact),
        QCryptographicHash::Sha256).toHex()).toLower();
}

QJsonObject PointCloudQualityMetricsToJson(
    const RobotCalculation::MeasureThenWeldAnalysisResult::PointCloudQualityReport& report)
{
    QJsonObject metrics;
    metrics.insert("inputPointCount", report.inputPointCount);
    metrics.insert("finitePointCount", report.finitePointCount);
    metrics.insert("rejectedPointCount", report.rejectedPointCount);
    metrics.insert("keyPointCount", report.keyPointCount);
    metrics.insert("cornerCount", report.cornerCount);
    metrics.insert("outputPointCount", report.outputPointCount);
    metrics.insert("projectedSpanMm", report.projectedSpanMm);
    metrics.insert("stationCoverageRatio", report.stationCoverageRatio);
    metrics.insert("longestContinuousRatio", report.longestContinuousRatio);
    metrics.insert("rejectedRatio", report.rejectedRatio);
    metrics.insert("medianResidualMm", report.medianResidualMm);
    metrics.insert("p95ResidualMm", report.p95ResidualMm);
    metrics.insert("residualInlierRatio", report.residualInlierRatio);
    metrics.insert("minNonLapSegmentLengthMm", report.minNonLapSegmentLengthMm);
    metrics.insert("minLapStepSegmentLengthMm", report.minLapStepSegmentLengthMm);
    metrics.insert("outputLengthMm", report.outputLengthMm);
    metrics.insert("outputLengthRatio", report.outputLengthRatio);
    metrics.insert("maxOutputStepMm", report.maxOutputStepMm);
    return metrics;
}

bool BuildQualityFileEvidence(
    const QString& filePath,
    const QString& laserDir,
    QJsonObject& evidence,
    QString& error,
    const MeasureThenWeldService::StopRequestedCallback& stopRequested =
        MeasureThenWeldService::StopRequestedCallback(),
    qint64* evidenceBytes = nullptr,
    qint64* evidenceLines = nullptr,
    qint64 maximumBytes = PointCloudProofIntegrity::MaximumEvidenceFileBytes,
    qint64 maximumLines = PointCloudProofIntegrity::MaximumEvidenceLinesPerFile)
{
    const QFileInfo fileInfo(QDir::fromNativeSeparators(filePath));
    const QFileInfo laserInfo(QDir::fromNativeSeparators(laserDir));
    const QString canonicalFile = fileInfo.canonicalFilePath();
    const QString canonicalLaser = laserInfo.canonicalFilePath();
    if (!fileInfo.exists() || !fileInfo.isFile()
        || fileInfo.isSymLink()
#ifdef Q_OS_WIN
        || fileInfo.isJunction()
#endif
        || canonicalFile.isEmpty()
        || canonicalLaser.isEmpty()
        || QFileInfo(canonicalFile).dir().absolutePath().compare(
            canonicalLaser, Qt::CaseInsensitive) != 0)
    {
        error = QString("质量证明文件必须是 LaserPoint 的直接普通文件且真实存在：%1").arg(filePath);
        return false;
    }
    PointCloudProofIntegrity::BoundedFileDigest digest;
    if (!PointCloudProofIntegrity::HashFileBounded(
            canonicalFile,
            maximumBytes,
            maximumLines,
            digest,
            error,
            stopRequested))
    {
        error = QStringLiteral("质量证明文件流式证据失败：") + error;
        return false;
    }
    if (evidenceBytes != nullptr)
    {
        *evidenceBytes = digest.size;
    }
    if (evidenceLines != nullptr)
    {
        *evidenceLines = digest.lineCount;
    }
    evidence.insert("relativePath", fileInfo.fileName());
    evidence.insert("size", static_cast<double>(digest.size));
    evidence.insert("sha256", digest.sha256);
    return true;
}

bool ConstantTimeBytesEqual(const QByteArray& left, const QByteArray& right)
{
    return PointCloudProofIntegrity::ConstantTimeEqual(left, right);
}

bool DecodePointCloudProofKey(const QString& encoded, QByteArray& key)
{
    const QByteArray encodedBytes = encoded.trimmed().toLatin1();
    key = QByteArray::fromBase64(encodedBytes, QByteArray::Base64UrlEncoding);
    return key.size() == 32
        && key.toBase64(QByteArray::Base64UrlEncoding | QByteArray::OmitTrailingEquals)
            == encodedBytes;
}

bool LoadPointCloudProofMacKey(bool createIfMissing, QByteArray& key, QString& error)
{
    QMutexLocker locker(&g_pointCloudProofSecurityMutex);
    QString encoded;
    if (ConfigDatabase::ReadScopedSetting(
            QStringLiteral("global"),
            QString(),
            QString::fromLatin1(POINT_CLOUD_PROOF_SECURITY_MODULE),
            QString::fromLatin1(POINT_CLOUD_PROOF_HMAC_KEY_NAME),
            &encoded))
    {
        if (!DecodePointCloudProofKey(encoded, key))
        {
            error = QStringLiteral("点云证明 HMAC 密钥损坏或长度无效，已拒绝授权。");
            return false;
        }
        return true;
    }
    if (!createIfMissing)
    {
        error = QStringLiteral("本机没有可解密的点云证明 HMAC 密钥；旧/异机证明禁止授权。");
        return false;
    }

    QByteArray generated(32, '\0');
    for (qsizetype offset = 0; offset < generated.size(); offset += 4)
    {
        const quint32 randomWord = QRandomGenerator::system()->generate();
        std::memcpy(generated.data() + offset, &randomWord, sizeof(randomWord));
    }
    const QString generatedText = QString::fromLatin1(generated.toBase64(
        QByteArray::Base64UrlEncoding | QByteArray::OmitTrailingEquals));
    if (!ConfigDatabase::WriteScopedSetting(
            QStringLiteral("global"),
            QString(),
            QString::fromLatin1(POINT_CLOUD_PROOF_SECURITY_MODULE),
            QString::fromLatin1(POINT_CLOUD_PROOF_HMAC_KEY_NAME),
            generatedText,
            QStringLiteral("secret"),
            true))
    {
        error = QStringLiteral("无法用 DPAPI CurrentUser 持久化点云证明 HMAC 密钥。");
        return false;
    }
    QString persisted;
    if (!ConfigDatabase::ReadScopedSetting(
            QStringLiteral("global"),
            QString(),
            QString::fromLatin1(POINT_CLOUD_PROOF_SECURITY_MODULE),
            QString::fromLatin1(POINT_CLOUD_PROOF_HMAC_KEY_NAME),
            &persisted)
        || !DecodePointCloudProofKey(persisted, key)
        || !ConstantTimeBytesEqual(key, generated))
    {
        error = QStringLiteral("点云证明 HMAC 密钥写后回读/DPAPI 解密校验失败。");
        key.clear();
        return false;
    }
    return true;
}

bool SignPointCloudQualityProof(QJsonObject& root, QString& error)
{
    QByteArray key;
    if (!LoadPointCloudProofMacKey(true, key, error))
    {
        return false;
    }
    return PointCloudProofIntegrity::SignProof(root, key, error);
}

bool VerifyPointCloudQualityProofMac(const QJsonObject& root, QString& error)
{
    QByteArray key;
    if (!LoadPointCloudProofMacKey(false, key, error))
    {
        return false;
    }
    return PointCloudProofIntegrity::VerifyProofMac(root, key, error);
}

bool BuildPointCloudProofReceiptRecord(
    const QJsonObject& root,
    const QDir& laserDir,
    const QByteArray& proofPayload,
    QString& scanRunId,
    QString& record,
    QString& error)
{
    const QString canonicalCasePath = laserDir.canonicalPath();
    return PointCloudProofIntegrity::BuildReceiptRecord(
        root, canonicalCasePath, proofPayload, scanRunId, record, error);
}

bool RegisterPointCloudProofReceipt(
    const QJsonObject& root,
    const QDir& laserDir,
    const QByteArray& proofPayload,
    QString& error)
{
    QString scanRunId;
    QString record;
    if (!BuildPointCloudProofReceiptRecord(
            root, laserDir, proofPayload, scanRunId, record, error))
    {
        return false;
    }
    if (!ConfigDatabase::WriteScopedSetting(
            QString::fromLatin1(POINT_CLOUD_PROOF_RECEIPT_SCOPE),
            scanRunId,
            QString::fromLatin1(POINT_CLOUD_PROOF_RECEIPT_MODULE),
            QString::fromLatin1(POINT_CLOUD_PROOF_RECEIPT_KEY),
            record,
            QStringLiteral("json"),
            true))
    {
        error = QStringLiteral("无法用 DPAPI CurrentUser 登记点云扫描持久收据。");
        return false;
    }
    QString persisted;
    if (!ConfigDatabase::ReadScopedSetting(
            QString::fromLatin1(POINT_CLOUD_PROOF_RECEIPT_SCOPE),
            scanRunId,
            QString::fromLatin1(POINT_CLOUD_PROOF_RECEIPT_MODULE),
            QString::fromLatin1(POINT_CLOUD_PROOF_RECEIPT_KEY),
            &persisted)
        || !ConstantTimeBytesEqual(persisted.toUtf8(), record.toUtf8()))
    {
        error = QStringLiteral("点云扫描收据写后回读/DPAPI 解密校验失败。");
        return false;
    }
    return true;
}

bool VerifyPointCloudProofReceipt(
    const QJsonObject& root,
    const QDir& laserDir,
    const QByteArray& proofPayload,
    QString& error)
{
    QString scanRunId;
    QString expected;
    if (!BuildPointCloudProofReceiptRecord(
            root, laserDir, proofPayload, scanRunId, expected, error))
    {
        return false;
    }
    QString persisted;
    if (!ConfigDatabase::ReadScopedSetting(
            QString::fromLatin1(POINT_CLOUD_PROOF_RECEIPT_SCOPE),
            scanRunId,
            QString::fromLatin1(POINT_CLOUD_PROOF_RECEIPT_MODULE),
            QString::fromLatin1(POINT_CLOUD_PROOF_RECEIPT_KEY),
            &persisted))
    {
        error = QStringLiteral("本机不存在该 scanRunId 的 DPAPI 持久收据；手写/异机证明禁止授权。");
        return false;
    }
    return PointCloudProofIntegrity::VerifyReceiptRecord(
        root, laserDir.canonicalPath(), proofPayload, persisted, error);
}

bool VerifyPointCloudProofInputs(
    const QJsonObject& root,
    const QDir& laserDir,
    QString& error,
    const MeasureThenWeldService::StopRequestedCallback& stopRequested =
        MeasureThenWeldService::StopRequestedCallback())
{
    const QJsonArray inputs = root.value(QStringLiteral("inputs")).toArray();
    constexpr int maximumInputEvidenceFiles = 8;
    if (inputs.isEmpty() || inputs.size() > maximumInputEvidenceFiles)
    {
        error = QStringLiteral("点云质量证明扫描输入数量必须在 1..%1：当前 %2。")
            .arg(maximumInputEvidenceFiles).arg(inputs.size());
        return false;
    }
    qint64 totalEvidenceBytes = 0;
    qint64 totalEvidenceLines = 0;
    for (const QJsonValue& inputValue : inputs)
    {
        if (stopRequested && stopRequested())
        {
            error = QStringLiteral("扫描输入证据回读已取消。");
            return false;
        }
        if (!inputValue.isObject())
        {
            error = QStringLiteral("点云质量证明包含非对象输入证据。");
            return false;
        }
        const QJsonObject expectedEvidence = inputValue.toObject();
        const QString relativePath = expectedEvidence.value(QStringLiteral("relativePath")).toString();
        if (expectedEvidence.size() != 3
            || relativePath.isEmpty()
            || QFileInfo(relativePath).fileName() != relativePath
            || !IsSha256Text(expectedEvidence.value(QStringLiteral("sha256")).toString())
            || expectedEvidence.value(QStringLiteral("size")).toDouble(-1.0) <= 0.0)
        {
            error = QStringLiteral("点云质量证明的扫描输入证据格式无效。");
            return false;
        }
        QJsonObject currentEvidence;
        qint64 evidenceBytes = 0;
        qint64 evidenceLines = 0;
        if (!BuildQualityFileEvidence(
                laserDir.filePath(relativePath),
                laserDir.absolutePath(),
                currentEvidence,
                error,
                stopRequested,
                &evidenceBytes,
                &evidenceLines))
        {
            error = QStringLiteral("扫描输入证据回读失败：") + error;
            return false;
        }
        if (evidenceBytes > PointCloudProofIntegrity::MaximumEvidenceTotalBytes - totalEvidenceBytes
            || evidenceLines > PointCloudProofIntegrity::MaximumEvidenceTotalLines - totalEvidenceLines)
        {
            error = QStringLiteral("扫描输入证据总字节或总行数超过硬上限。");
            return false;
        }
        totalEvidenceBytes += evidenceBytes;
        totalEvidenceLines += evidenceLines;
        if (currentEvidence != expectedEvidence)
        {
            error = QString("扫描输入 %1 在签发证明后发生变化，禁止下发/执行。").arg(relativePath);
            return false;
        }
    }
    return true;
}

bool LoadValidatedRebuildPointCloudContext(
    const QString& laserDir,
    const QString& expectedRobotName,
    const MeasureThenWeldService::PointCloudProductionExpectation& expectation,
    const PointCloudProcessingConfig::Settings& settings,
    PointCloudProductionContext& context,
    QString& error,
    const MeasureThenWeldService::StopRequestedCallback& stopRequested)
{
    const QDir dir(QDir::fromNativeSeparators(laserDir));
    const QString reportPath = dir.filePath(
        QString::fromLatin1(POINT_CLOUD_QUALITY_GATE_FILE_NAME));
    if (!PointCloudProofIntegrity::VerifyProofNotDenied(reportPath, error))
    {
        error = QStringLiteral("原 live-scan 点云证明已被拒绝闭锁，不能继承重建上下文：") + error;
        return false;
    }
    QByteArray proofPayload;
    if (!PointCloudProofIntegrity::ReadFileBounded(
            reportPath,
            PointCloudProofIntegrity::MaximumProofBytes,
            proofPayload,
            error))
    {
        error = QStringLiteral(
            "生产重建缺少原 live-scan 质量证明上下文；离线目录不能生成可运动授权，请重新扫描：")
            + reportPath + QStringLiteral("；") + error;
        return false;
    }
    QJsonParseError parseError;
    const QJsonDocument document = QJsonDocument::fromJson(proofPayload, &parseError);
    if (parseError.error != QJsonParseError::NoError || !document.isObject())
    {
        error = QStringLiteral("原点云质量证明损坏，不能继承扫描上下文；请重新扫描：")
            + parseError.errorString();
        return false;
    }
    const QJsonObject root = document.object();
    if (root.value("schemaVersion").toInt(-1) != POINT_CLOUD_QUALITY_SCHEMA_VERSION)
    {
        error = QStringLiteral(
            "旧 schema 1/2 点云质量证明没有本机 HMAC/持久收据，禁止重建为生产授权；请重新扫描。");
        return false;
    }
    const QString proofPurpose = root.value("purpose").toString();
    if (proofPurpose != QStringLiteral("production"))
    {
        error = QStringLiteral(
            "原点云质量证明用途为“%1”，不是生产用途 production，禁止继承；请重新扫描。")
            .arg(proofPurpose.isEmpty() ? QStringLiteral("<空>") : proofPurpose);
        return false;
    }
    const QString proofRobotName = root.value("robotName").toString().trimmed();
    const QString currentRobotName = expectedRobotName.trimmed();
    if (proofRobotName.compare(currentRobotName, Qt::CaseInsensitive) != 0)
    {
        error = QStringLiteral(
            "原点云质量证明绑定机器人“%1”，当前控制单元为“%2”，禁止跨机器人继承；"
            "请选择同一机器人的扫描结果或重新扫描。")
            .arg(proofRobotName.isEmpty() ? QStringLiteral("<空>") : proofRobotName,
                currentRobotName.isEmpty() ? QStringLiteral("<空>") : currentRobotName);
        return false;
    }
    const QString expectedCaseId = QFileInfo(dir.absolutePath()).dir().dirName();
    if (root.value("caseId").toString().compare(expectedCaseId, Qt::CaseInsensitive) != 0)
    {
        error = QStringLiteral("原点云质量证明不属于当前案例目录，禁止继承扫描上下文。");
        return false;
    }
    if (!VerifyPointCloudQualityProofMac(root, error)
        || !VerifyPointCloudProofReceipt(root, dir, proofPayload, error))
    {
        return false;
    }

    if (expectation.robotName.compare(expectedRobotName.trimmed(), Qt::CaseInsensitive) != 0
        || expectation.robotEndpoint.trimmed().isEmpty()
        || expectation.cameraSection.trimmed().isEmpty()
        || !IsSha256Text(expectation.handEyeSha256))
    {
        error = QStringLiteral("冻结的生产上下文不完整或机器人不匹配，禁止后台重建生产证明。");
        return false;
    }
    if (!ValidatePointCloudProductionContext(
            root.value("productionContext").toObject(),
            root.value("createdUtc").toString(),
            expectation.robotEndpoint,
            expectation.cameraSection,
            expectation.handEyeSha256,
            &context,
            error))
    {
        return false;
    }

    if (!VerifyPointCloudProofInputs(root, dir, error, stopRequested))
    {
        return false;
    }
    return PointCloudProofIntegrity::VerifyProofNotDenied(reportPath, error);
}

bool WritePointCloudQualityGate(
    const QString& laserDir,
    const QString& robotName,
    const PointCloudProcessingConfig::Settings& settings,
    const RobotCalculation::MeasureThenWeldAnalysisResult::PointCloudQualityReport& report,
    const QStringList& inputPaths,
    const QString& weldPosePath,
    const QString& validatedWeldPoseSha256,
    qint64 validatedWeldPoseSize,
    const QString& authorizedPosePath,
    const QString& validatedAuthorizedPoseSha256,
    qint64 validatedAuthorizedPoseSize,
    const PointCloudProductionContext& productionContext,
    const QString& generationMode,
    QString& error,
    const MeasureThenWeldService::StopRequestedCallback& stopRequested =
        MeasureThenWeldService::StopRequestedCallback())
{
    error.clear();
    const QDir dir(QDir::fromNativeSeparators(laserDir));
    if (!dir.exists() || dir.dirName().compare(QStringLiteral("LaserPoint"), Qt::CaseInsensitive) != 0)
    {
        error = QString("质量报告目录必须是已存在的 LaserPoint：%1").arg(laserDir);
        return false;
    }
    const QString reportPath = dir.filePath(
        QString::fromLatin1(POINT_CLOUD_QUALITY_GATE_FILE_NAME));
    if (!PointCloudProofIntegrity::RequireProofReplacementActive(reportPath, error))
    {
        error = QStringLiteral("点云质量证明写入缺少持久拒绝闭锁：") + error;
        return false;
    }

    const bool enforce = settings.validationPolicy == PointCloudProcessingConfig::ValidationPolicy::Enforce;
    const bool hasValidatedAuthorizedPose = !authorizedPosePath.isEmpty()
        && IsSha256Text(validatedAuthorizedPoseSha256)
        && validatedAuthorizedPoseSize > 0;
    const bool hasValidatedWeldPose = !weldPosePath.isEmpty()
        && IsSha256Text(validatedWeldPoseSha256)
        && validatedWeldPoseSize > 0;
    const QString proofCreatedUtc =
        QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs);
    const QJsonObject productionContextJson =
        PointCloudProductionContextToJson(productionContext);
    QString productionContextError;
    const bool productionContextSemanticallyValid = ValidatePointCloudProductionContext(
        productionContextJson,
        proofCreatedUtc,
        productionContext.robotEndpoint,
        productionContext.cameraSection,
        productionContext.handEyeSha256,
        nullptr,
        productionContextError);
    const bool hasProductionContext = productionContext.origin == QStringLiteral("liveRobotCameraScan")
        && !QUuid(productionContext.scanRunId).isNull()
        && !productionContext.robotEndpoint.trimmed().isEmpty()
        && !productionContext.cameraSection.trimmed().isEmpty()
        && IsSha256Text(productionContext.handEyeSha256)
        && !productionContext.scanStartedUtc.trimmed().isEmpty()
        && (generationMode == QStringLiteral("liveScan")
            || generationMode == QStringLiteral("validatedRebuild"))
        && productionContextSemanticallyValid;
    const bool authorize = enforce && report.evaluated && report.passed
        && hasValidatedAuthorizedPose && hasProductionContext;
    if (enforce && report.evaluated && report.passed
        && hasValidatedAuthorizedPose && !hasProductionContext)
    {
        error = productionContextError.isEmpty()
            ? QStringLiteral(
                "Enforce 质量通过但缺少 live-scan 端点/运行/标定上下文，禁止生成生产授权。")
            : productionContextError;
        return false;
    }
    QJsonObject root;
    root.insert("schemaVersion", POINT_CLOUD_QUALITY_SCHEMA_VERSION);
    root.insert("purpose", "production");
    root.insert("createdUtc", proofCreatedUtc);
    root.insert("generationMode", generationMode);
    root.insert("productionContext", productionContextJson);
    root.insert("robotName", robotName.trimmed());
    root.insert("caseId", dir.dirName().compare(QStringLiteral("LaserPoint"), Qt::CaseInsensitive) == 0
        ? QFileInfo(dir.absolutePath()).dir().dirName()
        : QString());
    root.insert("policy", PointCloudProcessingConfig::ValidationPolicyConfigValue(settings.validationPolicy));
    root.insert("profileVersion", PointCloudProcessingConfig::CURRENT_VALIDATION_PROFILE_VERSION);
    root.insert("algorithmRevision", QString::fromLatin1(POINT_CLOUD_QUALITY_ALGORITHM_REVISION));
    root.insert("processingMode", PointCloudProcessingConfig::ModeConfigValue(settings.mode));
    root.insert("policyRevisionSha256", PointCloudQualityPolicyRevision(settings));
    root.insert("analysisEvaluated", report.evaluated);
    root.insert("qualityPassed", report.passed);
    root.insert("authorized", authorize);
    root.insert("state", authorize ? "authorized" : (enforce ? "rejected" : "audit"));
    root.insert("failures", StringListToJsonArray(report.failures));
    root.insert("warnings", StringListToJsonArray(report.warnings));
    root.insert("metrics", PointCloudQualityMetricsToJson(report));
    root.insert("thresholds", BuildPointCloudQualityThresholds(settings));
    root.insert("safetyGateRecords", BuildSafetyGateRecords(settings));

    QJsonArray inputs;
    constexpr int maximumInputEvidenceFiles = 8;
    if (authorize && inputPaths.isEmpty())
    {
        error = QStringLiteral("生产质量证明没有预期点云输入，禁止生成 authorized 凭证。");
        return false;
    }
    if (inputPaths.size() > maximumInputEvidenceFiles)
    {
        error = QStringLiteral("生产质量证明输入文件超过 %1 个硬上限。")
            .arg(maximumInputEvidenceFiles);
        return false;
    }
    qint64 totalEvidenceBytes = 0;
    qint64 totalEvidenceLines = 0;
    for (const QString& inputPath : inputPaths)
    {
        if (stopRequested && stopRequested())
        {
            error = QStringLiteral("生成点云输入证据已取消。");
            return false;
        }
        if (inputPath.isEmpty() || !QFileInfo::exists(inputPath))
        {
            if (authorize)
            {
                error = QString("生产质量证明的预期点云输入为空或不存在，禁止跳过：%1")
                    .arg(inputPath);
                return false;
            }
            continue;
        }
        QJsonObject evidence;
        QString evidenceError;
        qint64 evidenceBytes = 0;
        qint64 evidenceLines = 0;
        if (!BuildQualityFileEvidence(
                inputPath,
                dir.absolutePath(),
                evidence,
                evidenceError,
                stopRequested,
                &evidenceBytes,
                &evidenceLines))
        {
            if (authorize)
            {
                error = evidenceError;
                return false;
            }
            continue;
        }
        if (evidenceBytes > PointCloudProofIntegrity::MaximumEvidenceTotalBytes - totalEvidenceBytes
            || evidenceLines > PointCloudProofIntegrity::MaximumEvidenceTotalLines - totalEvidenceLines)
        {
            error = QStringLiteral("生产质量证明输入证据总字节或总行数超过硬上限。");
            return false;
        }
        totalEvidenceBytes += evidenceBytes;
        totalEvidenceLines += evidenceLines;
        inputs.push_back(evidence);
    }
    if (authorize && inputs.size() != inputPaths.size())
    {
        error = QStringLiteral("生产质量证明未能为每个预期点云输入建立证据，禁止授权。");
        return false;
    }
    root.insert("inputs", inputs);

    QJsonObject artifacts;
    if (!weldPosePath.isEmpty())
    {
        if (!hasValidatedWeldPose && enforce)
        {
            error = QStringLiteral("补偿前焊道缺少与结构验证同一字节快照的 SHA256/大小。");
            return false;
        }
        QJsonObject currentEvidence;
        if (!BuildQualityFileEvidence(
                weldPosePath,
                dir.absolutePath(),
                currentEvidence,
                error,
                stopRequested,
                nullptr,
                nullptr,
                PointCloudProofIntegrity::MaximumWeldPoseBytes,
                PointCloudProofIntegrity::MaximumWeldPoseLines))
        {
            return false;
        }
        if (hasValidatedWeldPose
            && (currentEvidence.value("sha256").toString().toLower()
                != validatedWeldPoseSha256.toLower()
            || static_cast<qint64>(currentEvidence.value("size").toDouble(-1.0))
                != validatedWeldPoseSize))
        {
            error = QStringLiteral("补偿前焊道在结构验证与质量证明提交之间发生变化，拒绝授权。");
            return false;
        }
        artifacts.insert("weldPose", currentEvidence);
    }
    if (!authorizedPosePath.isEmpty())
    {
        if (!hasValidatedAuthorizedPose && enforce)
        {
            error = QStringLiteral("授权焊道缺少与结构回读同一字节快照的 SHA256/大小。");
            return false;
        }
        QJsonObject currentEvidence;
        if (!BuildQualityFileEvidence(
                authorizedPosePath,
                dir.absolutePath(),
                currentEvidence,
                error,
                stopRequested,
                nullptr,
                nullptr,
                PointCloudProofIntegrity::MaximumWeldPoseBytes,
                PointCloudProofIntegrity::MaximumWeldPoseLines))
        {
            return false;
        }
        if (hasValidatedAuthorizedPose
            && (currentEvidence.value("sha256").toString().toLower()
                != validatedAuthorizedPoseSha256.toLower()
            || static_cast<qint64>(currentEvidence.value("size").toDouble(-1.0))
                != validatedAuthorizedPoseSize))
        {
            error = QStringLiteral("最终焊道在结构回读与质量证明提交之间发生变化，拒绝授权。");
            return false;
        }
        artifacts.insert(authorize ? QStringLiteral("authorizedPose") : QStringLiteral("candidatePose"),
            currentEvidence);
    }
    root.insert("artifacts", artifacts);
    if (!SignPointCloudQualityProof(root, error))
    {
        return false;
    }

    QSaveFile file(reportPath);
    if (!file.open(QIODevice::WriteOnly))
    {
        error = QString("无法写入点云质量报告：%1").arg(reportPath);
        return false;
    }
    const QByteArray payload = QJsonDocument(root).toJson(QJsonDocument::Indented);
    if (payload.isEmpty() || payload.size() > PointCloudProofIntegrity::MaximumProofBytes)
    {
        error = QStringLiteral("点云质量证明超过 %1 字节硬上限，拒绝提交。")
            .arg(PointCloudProofIntegrity::MaximumProofBytes);
        return false;
    }
    if (file.write(payload) != payload.size() || !file.commit())
    {
        error = QString("原子提交点云质量报告失败：%1").arg(reportPath);
        return false;
    }

    QFile verifyFile(reportPath);
    if (!verifyFile.open(QIODevice::ReadOnly))
    {
        error = QString("点云质量报告写后回读失败：%1").arg(reportPath);
        return false;
    }
    const QByteArray verifyPayload = verifyFile.readAll();
    QJsonParseError parseError;
    const QJsonDocument verifyDocument = QJsonDocument::fromJson(verifyPayload, &parseError);
    if (verifyPayload != payload
        || parseError.error != QJsonParseError::NoError
        || !verifyDocument.isObject())
    {
        verifyFile.close();
        QFile::remove(reportPath);
        error = QString("点云质量报告写后解析失败：%1").arg(parseError.errorString());
        return false;
    }
    if (!VerifyPointCloudQualityProofMac(verifyDocument.object(), error))
    {
        verifyFile.close();
        QFile::remove(reportPath);
        return false;
    }
    verifyFile.close();
    if (hasProductionContext
        && !RegisterPointCloudProofReceipt(root, dir, payload, error))
    {
        // 文件和 DB 无法跨介质原子提交；失败时删除新 proof，留下旧/缺失收据只会 fail-closed。
        QFile::remove(reportPath);
        return false;
    }
    if (authorize && !hasProductionContext)
    {
        QFile::remove(reportPath);
        error = QStringLiteral("授权点云证明没有可登记的扫描收据，已删除。");
        return false;
    }
    return true;
}

bool InvalidatePointCloudQualityGate(const QString& laserDir, QString& error)
{
    error.clear();
    const QString path = QDir(QDir::fromNativeSeparators(laserDir))
        .filePath(QString::fromLatin1(POINT_CLOUD_QUALITY_GATE_FILE_NAME));
    if (!PointCloudProofIntegrity::RequireProofReplacementActive(path, error))
    {
        error = QStringLiteral("撤销点云质量证明前拒绝闭锁无效：") + error;
        return false;
    }
    if (QFileInfo::exists(path) && !QFile::remove(path))
    {
        error = QString("无法使旧点云质量证明失效：%1").arg(path);
        return false;
    }
    return true;
}

bool VerifyPointCloudQualityGate(
    const QString& posePath,
    const QString& expectedRobotName,
    const QString& loadedPoseSha256,
    qint64 loadedPoseSize,
    const RobotDriverAdaptor* expectedDriver,
    const MeasureThenWeldService::PointCloudProductionExpectation* frozenExpectation,
    bool allowActiveProofReplacement,
    QString& error)
{
    error.clear();
    const QFileInfo poseInfo(QDir::fromNativeSeparators(posePath));
    const QDir laserDir = poseInfo.dir();
    if (poseInfo.fileName().isEmpty()
        || laserDir.dirName().compare(QStringLiteral("LaserPoint"), Qt::CaseInsensitive) != 0
        || !IsSha256Text(loadedPoseSha256)
        || loadedPoseSize <= 0)
    {
        error = QString("生产焊道必须是 LaserPoint 目录中的直接文件，并提供同一次读取的有效大小/SHA256：%1")
            .arg(posePath);
        return false;
    }
    const QString reportPath = laserDir.filePath(QString::fromLatin1(POINT_CLOUD_QUALITY_GATE_FILE_NAME));
    const auto verifyNotDenied = [&]()
    {
        return allowActiveProofReplacement
            ? PointCloudProofIntegrity::RequireProofReplacementActive(reportPath, error)
            : PointCloudProofIntegrity::VerifyProofNotDenied(reportPath, error);
    };
    if (!verifyNotDenied())
    {
        return false;
    }
    QByteArray proofPayload;
    if (!PointCloudProofIntegrity::ReadFileBounded(
            reportPath,
            PointCloudProofIntegrity::MaximumProofBytes,
            proofPayload,
            error))
    {
        error = QString("缺少、过大或无法完整读取点云质量证明，禁止下发/执行焊道：%1；%2")
            .arg(reportPath, error);
        return false;
    }
    QJsonParseError parseError;
    const QJsonDocument document = QJsonDocument::fromJson(proofPayload, &parseError);
    if (parseError.error != QJsonParseError::NoError || !document.isObject())
    {
        error = QString("点云质量证明损坏：%1").arg(parseError.errorString());
        return false;
    }
    const QJsonObject root = document.object();
    const PointCloudProcessingConfig::Settings currentSettings = PointCloudProcessingConfig::Load();
    if (root.value("schemaVersion").toInt() != POINT_CLOUD_QUALITY_SCHEMA_VERSION
        || root.value("profileVersion").toInt() != PointCloudProcessingConfig::CURRENT_VALIDATION_PROFILE_VERSION
        || root.value("algorithmRevision").toString() != QString::fromLatin1(POINT_CLOUD_QUALITY_ALGORITHM_REVISION)
        || root.value("purpose").toString() != QStringLiteral("production")
        || root.value("policy").toString() != QStringLiteral("Enforce")
        || !root.value("analysisEvaluated").toBool()
        || !root.value("qualityPassed").toBool()
        || !root.value("authorized").toBool()
        || root.value("state").toString() != QStringLiteral("authorized")
        || (root.value("generationMode").toString() != QStringLiteral("liveScan")
            && root.value("generationMode").toString() != QStringLiteral("validatedRebuild")))
    {
        error = QStringLiteral(
            "点云质量证明不是当前 schema 3 的 Enforce/PASS/authorized 证明，禁止下发/执行；旧证明请重新扫描/重建。");
        return false;
    }
    if (!expectedRobotName.trimmed().isEmpty()
        && root.value("robotName").toString().compare(expectedRobotName.trimmed(), Qt::CaseInsensitive) != 0)
    {
        error = QStringLiteral("点云质量证明绑定的机器人与当前机器人不一致。");
        return false;
    }
    const QString expectedCaseId = QFileInfo(laserDir.absolutePath()).dir().dirName();
    if (root.value("caseId").toString().compare(expectedCaseId, Qt::CaseInsensitive) != 0)
    {
        error = QStringLiteral("点云质量证明绑定的案例目录与当前轨迹目录不一致。");
        return false;
    }
    // 先验证不可伪造的本机 MAC 与持久收据，再相信任何 PASS/上下文字段。
    const MeasureThenWeldService::StopRequestedCallback evidenceStopRequested =
        expectedDriver != nullptr
        ? MeasureThenWeldService::StopRequestedCallback([expectedDriver]()
            {
                return RobotOperationLease::IsCancellationRequested(expectedDriver);
            })
        : MeasureThenWeldService::StopRequestedCallback();
    if (!VerifyPointCloudQualityProofMac(root, error)
        || !VerifyPointCloudProofReceipt(root, laserDir, proofPayload, error)
        || !VerifyPointCloudProofInputs(root, laserDir, error, evidenceStopRequested))
    {
        return false;
    }
    QString expectedEndpoint;
    QString expectedCameraSection;
    QString expectedHandEyeSha256;
    if (expectedDriver != nullptr)
    {
        if (!LoadCurrentPointCloudContextExpectations(
                expectedRobotName,
                expectedDriver,
                true,
                expectedEndpoint,
                expectedCameraSection,
                expectedHandEyeSha256,
                error))
        {
            return false;
        }
    }
    else if (frozenExpectation != nullptr
        && frozenExpectation->robotName.compare(
            expectedRobotName.trimmed(), Qt::CaseInsensitive) == 0
        && !frozenExpectation->robotEndpoint.trimmed().isEmpty()
        && !frozenExpectation->cameraSection.trimmed().isEmpty()
        && IsSha256Text(frozenExpectation->handEyeSha256))
    {
        expectedEndpoint = frozenExpectation->robotEndpoint;
        expectedCameraSection = frozenExpectation->cameraSection;
        expectedHandEyeSha256 = frozenExpectation->handEyeSha256;
    }
    else
    {
        error = QStringLiteral(
            "PointCloudProduction 验证缺少当前机器人或 UI 线程冻结的完整生产上下文；离线只能生成 Audit/unauthorized 产物。");
        return false;
    }
    if (!ValidatePointCloudProductionContext(
            root.value("productionContext").toObject(),
            root.value("createdUtc").toString(),
            expectedEndpoint,
            expectedCameraSection,
            expectedHandEyeSha256,
            nullptr,
            error))
    {
        return false;
    }
    if (currentSettings.validationPolicy != PointCloudProcessingConfig::ValidationPolicy::Enforce
        || root.value("processingMode").toString()
            != PointCloudProcessingConfig::ModeConfigValue(currentSettings.mode)
        || root.value("thresholds").toObject() != BuildPointCloudQualityThresholds(currentSettings)
        || root.value("policyRevisionSha256").toString() != PointCloudQualityPolicyRevision(currentSettings))
    {
        error = QStringLiteral("当前点云质量策略/阈值与证明快照不一致，旧证明失效；请从原始点云重新生成焊道。");
        return false;
    }

    const QJsonObject authorizedPose = root.value("artifacts").toObject().value("authorizedPose").toObject();
    if (authorizedPose.value("relativePath").toString() != poseInfo.fileName()
        || static_cast<qint64>(authorizedPose.value("size").toDouble(-1.0)) != loadedPoseSize)
    {
        error = QStringLiteral("点云质量证明绑定的焊道文件名或大小不一致。");
        return false;
    }
    if (loadedPoseSha256.toLower() != authorizedPose.value("sha256").toString().toLower())
    {
        error = QStringLiteral("点云质量证明绑定的焊道 SHA256 与实际解析字节不一致，文件可能已被修改。");
        return false;
    }
    // Recheck after every proof/input/context read. A replacement that begins
    // while verification is in flight must deny the final authorization.
    return verifyNotDenied();
}

bool VerifyWeldPoseAuthorization(
    MeasureThenWeldService::WeldPoseSource poseSource,
    const QString& posePath,
    const QString& expectedRobotName,
    const QString& loadedPoseSha256,
    qint64 loadedPoseSize,
    QString& error,
    const RobotDriverAdaptor* expectedDriver = nullptr,
    const MeasureThenWeldService::PointCloudProductionExpectation* frozenExpectation = nullptr,
    bool allowActiveProofReplacement = false)
{
    return poseSource == MeasureThenWeldService::WeldPoseSource::PointCloudProduction
        ? VerifyPointCloudQualityGate(
            posePath,
            expectedRobotName,
            loadedPoseSha256,
            loadedPoseSize,
            expectedDriver,
            frozenExpectation,
            allowActiveProofReplacement,
            error)
        : VerifySyntheticPoseAuthorization(
            posePath, expectedRobotName, loadedPoseSha256, loadedPoseSize, error);
}

qint64 SteadyNowMs()
{
    return static_cast<qint64>(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}

qint64 SteadyNowUs()
{
    return static_cast<qint64>(std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}

struct TimestampedCameraPoint
{
    int sampleIndex = 0;
    qint64 rawTimestampUs = 0;
    qint64 rawDeltaUs = 0;
    qint64 timestampUs = 0;
    Eigen::Vector3d point = Eigen::Vector3d::Zero();
    std::vector<Eigen::Vector3d> linePoints;
    QString error;
};


struct WeldPosePreset
{
    struct PoseCompSlot
    {
        QString name;
        QString segmentKind;
        double poseRx = 0.0;
        double poseRy = 0.0;
        double poseRz = 0.0;
        double compX = 0.0;
        double compY = 0.0;
        double compZ = 0.0;
        bool hasIniReference = false;
        bool generatedReference = false;
        bool validReference = false;
    };

    struct SeamCompValues
    {
        double weldZComp = 0.0;
        double weldGunDirComp = 0.0;
        double weldSeamDirComp = 0.0;
    };

    QString weldLineFilePath;
    QString weldLineSectionName;
    QString poseCompFilePath;
    QString seamCompFilePath;
    QString robotParaPath;
    int robotType = ROBOT_TYPE_FANUC;
    double rx = 0.0;
    double ry = 0.0;
    double measureReferenceRx = 0.0;
    double measureReferenceRy = 0.0;
    double measureReferenceRz = 0.0;
    double gunToolBaseRz = 180.0;
    double poseMatchMaxErrorDeg = 5.0;
    int poseCompMatchMode = POSE_COMP_MATCH_BY_POSE;
    double cornerTransitionLeadDistance = 10.0;
    double cornerArcRadiusMm = 0.0;
    double finalWeldStepFromProcessMm = 0.0;  // 工艺里的实际焊道点间距(>0 优先于测量参数页的值)
    bool keepAnchorsOnly = false;             // 精简轨迹：最终抽样只保留特殊点(起终/拐点/段边界/圆弧边界/搭接)，丢弃中间普通点
    double weldStartSkipDistance = 10.0;
    double weldEndSkipDistance = 10.0;
    double weldRzGainDeg = 0.0;
    bool useTaughtWeldPose = false;
    double taughtWeldPoseRx = 0.0;
    double taughtWeldPoseRy = 0.0;
    double taughtWeldPoseRz = 0.0;
    double slopeRzMinDeg = -20.0;
    double slopeRzMaxDeg = 20.0;
    double stepOverlapRel = 20.0;
    int weldPostureType = 1; // 焊接姿态/位形(0NULL/1可变/2恒定/3腕关节)，来自基础工艺参数
    int weldDynamicMode = 0; // 动态特性(WLin DYNAMIC)：0=NULL(机器人默认) 非0=ntdyn0(程序速度)，复用工艺 nWeldMethod
    int weldDirection = 1;
    bool weldProcessLoaded = false;
    QString weldProcessLoadError;
    QString weldProcessSafetyError;
    bool cornerArcRadiusFromWeldProcess = false;
    bool transitionSpeedEnabled = false;
    bool transitionCurrentVoltageEnabled = false;
    bool transitionCurrentVoltageEnableMismatch = false;
    int transitionApplyScope = 2;
    double startArcCurrent = 0.0;
    double startArcVoltage = 0.0;
    double startArcWaitTime = 0.0;
    double weldCurrent = 0.0;
    double weldVoltage = 0.0;
    double weldProcessSpeedMmPerMin = 0.0;
    double stopArcCurrent = 0.0;
    double stopArcVoltage = 0.0;
    double stopArcWaitTime = 0.0;
    int arcMode = 4;
    double transitionSpeedMmPerMin = 0.0;
    double transitionCurrent = 0.0;
    double transitionVoltage = 0.0;
    bool weaveEnabled = true;
    bool weaveAppPointwise = false;  // true=上位机自建pointwise摆动(来自工艺 nWrapConditionNo!=0)
    int weavePointsPerCycle = 16;    // pointwise每周期采样点数(来自工艺 nStandWeldDir 复用死字段；0/无效在driver规范化为16)
    bool trackEnabled = true;
    T_WeaveDate weaveParam;
    T_TrackData trackParam;
    std::vector<PoseCompSlot> poseCompSlots;
    SeamCompValues seamComp;
    QString seamCompLoadError;
    QStringList seamCompWarnings;
    bool weldLineFromIni = false;
    bool poseCompFromIni = false;
    bool seamCompFromIni = false;
};

double ResolveEffectiveFinalStepMm(
    const T_PRECISE_MEASURE_PARAM& param,
    const WeldPosePreset& preset,
    double overrideFinalStepMm)
{
    if (std::isfinite(overrideFinalStepMm) && overrideFinalStepMm > 0.0)
    {
        return NormalizeFinalWeldTrajectorySampleStepMm(overrideFinalStepMm);
    }
    if (std::isfinite(preset.finalWeldStepFromProcessMm)
        && preset.finalWeldStepFromProcessMm > 0.0)
    {
        return NormalizeFinalWeldTrajectorySampleStepMm(preset.finalWeldStepFromProcessMm);
    }
    return NormalizeFinalWeldTrajectorySampleStepMm(param.dFinalWeldTrajectoryStepMm);
}

QString BuildEffectiveWeldExecutionFingerprint(
    const T_PRECISE_MEASURE_PARAM& param,
    const WeldPosePreset& preset,
    double effectiveFinalStepMm)
{
    QJsonArray values;
    const auto addText = [&values](const char* key, const QString& value)
        {
            values.append(QString::fromLatin1(key) + QLatin1Char('=') + value);
        };
    const auto addInt = [&addText](const char* key, qint64 value)
        {
            addText(key, QString::number(value));
        };
    const auto addBool = [&addInt](const char* key, bool value)
        {
            addInt(key, value ? 1 : 0);
        };
    const auto addDouble = [&addText](const char* key, double value)
        {
            addText(key, std::isfinite(value)
                ? QString::number(value, 'g', 17)
                : QStringLiteral("non-finite"));
        };

    addText("schema", QStringLiteral("weld-execution-v2"));
    addText("robot", QString::fromStdString(param.sRobotName));
    addInt("groupIndex", param.nParamGroupIndex);
    addText("groupName", param.sParamGroupName);
    addText("scanSection", QString::fromStdString(param.sSectionName));
    addText("weldSection", QString::fromStdString(param.sWeldSectionName));
    addBool("actualWeld", param.bDoActualWeld);
    addDouble("weldSpeed", param.dWeldSpeedMmPerMin);
    addDouble("dryRunSpeed", param.dDryRunSpeedMmPerMin);
    addDouble("safeMoveSpeed", param.dWeldSafeMoveSpeedMmPerMin);
    addDouble("gunBackSafe", param.dGunDownBackSafeDis);
    addInt("safeRetreatDirection", param.nWeldSafeRetreatDirection);
    addDouble("resumeBacktrack", param.dResumeBacktrackMm);
    addInt("weldDirection", preset.weldDirection);
    addDouble("effectiveFinalStep", effectiveFinalStepMm);

    addInt("robotType", preset.robotType);
    addBool("processLoaded", preset.weldProcessLoaded);
    addDouble("processFinalStep", preset.finalWeldStepFromProcessMm);
    addDouble("stepOverlap", preset.stepOverlapRel);
    addInt("postureType", preset.weldPostureType);
    addInt("dynamicMode", preset.weldDynamicMode);
    addDouble("startCurrent", preset.startArcCurrent);
    addDouble("startVoltage", preset.startArcVoltage);
    addDouble("startWait", preset.startArcWaitTime);
    addDouble("weldCurrent", preset.weldCurrent);
    addDouble("weldVoltage", preset.weldVoltage);
    addDouble("processSpeed", preset.weldProcessSpeedMmPerMin);
    addDouble("stopCurrent", preset.stopArcCurrent);
    addDouble("stopVoltage", preset.stopArcVoltage);
    addDouble("stopWait", preset.stopArcWaitTime);
    addInt("arcMode", preset.arcMode);
    addBool("transitionSpeedEnabled", preset.transitionSpeedEnabled);
    addDouble("transitionSpeed", preset.transitionSpeedMmPerMin);
    addBool("transitionCurrentVoltageEnabled", preset.transitionCurrentVoltageEnabled);
    addBool("transitionEnableMismatch", preset.transitionCurrentVoltageEnableMismatch);
    addDouble("transitionCurrent", preset.transitionCurrent);
    addDouble("transitionVoltage", preset.transitionVoltage);
    addInt("transitionScope", preset.transitionApplyScope);
    addBool("keepAnchorsOnly", preset.keepAnchorsOnly);

    addBool("weaveEnabled", preset.weaveEnabled);
    addBool("pointwiseWeave", preset.weaveAppPointwise);
    addInt("weavePointsPerCycle", preset.weavePointsPerCycle);
    const T_WeaveDate& weave = preset.weaveParam;
    addInt("weave.type", weave.nWeaveType);
    addInt("weave.shape", weave.nWeaveShape);
    addDouble("weave.frequency", weave.dWeaveFrequencyHz);
    addDouble("weave.amplitude", weave.dWeaveAmplitudeMm);
    addDouble("weave.swingDirection", weave.dSwingDirectionDeg);
    addDouble("weave.planeAngle", weave.dWeavePlaneAngleDeg);
    addDouble("weave.spaceAngle", weave.dSpaceAngleDeg);
    addInt("weave.pause1", weave.nPauseTime1Ms);
    addInt("weave.pause2", weave.nPauseTime2Ms);
    addInt("weave.pause3", weave.nPauseTime3Ms);
    addInt("weave.pause4", weave.nPauseTime4Ms);
    addInt("weave.pauseContinue", weave.nPauseContinue);
    addDouble("weave.endLength", weave.dEndLengthMm);
    addDouble("weave.endWidth", weave.dEndWidthMm);
    addDouble("weave.centerHeight", weave.dCenterHeightMm);

    addBool("trackEnabled", preset.trackEnabled);
    const T_TrackData& track = preset.trackParam;
    addInt("track.lateralBegin", track.nLateralBeginCycle);
    addDouble("track.lateralGain", track.dLateralGain);
    addDouble("track.leftArea", track.dLeftAreaCoefficient);
    addDouble("track.rightArea", track.dRightAreaCoefficient);
    addInt("track.verticalMode", track.nVerticalModeFlag);
    addDouble("track.verticalReference", track.dVerticalReferenceCurrent);
    addInt("track.verticalBegin", track.nVerticalBeginCycle);
    addInt("track.verticalSustain", track.nVerticalSustainCycle);
    addDouble("track.verticalCycleLength", track.dVerticalCycleLength);
    addDouble("track.verticalGain", track.dVerticalGain);
    addInt("track.intervalMode", track.nTimeOrDistanceMode);
    addInt("track.timeInterval", track.nTimeIntervalMs);
    addInt("track.distanceInterval", track.nDistanceIntervalMm);
    addDouble("track.lateralMinCycle", track.dLateralMinCompPerCycle);
    addDouble("track.lateralMaxCycle", track.dLateralMaxCompPerCycle);
    addDouble("track.lateralMaxTotal", track.dLateralMaxCompTotal);
    addDouble("track.lateralAsymmetry", track.dLateralAsymmetryCoefficient);
    addDouble("track.lateralReserved6", track.dLateralReserved6);
    addDouble("track.lateralReserved5", track.dLateralReserved5);
    addDouble("track.lateralReserved4", track.dLateralReserved4);
    addDouble("track.lateralReserved3", track.dLateralReserved3);
    addDouble("track.lateralReserved2", track.dLateralReserved2);
    addDouble("track.lateralReserved1", track.dLateralReserved1);
    addDouble("track.verticalMinCycle", track.dVerticalMinCompPerCycle);
    addDouble("track.verticalMaxCycle", track.dVerticalMaxCompPerCycle);
    addDouble("track.verticalMaxTotal", track.dVerticalMaxCompTotal);
    addDouble("track.verticalAsymmetry", track.dVerticalAsymmetryCoefficient);
    addDouble("track.verticalReserved6", track.dVerticalReserved6);
    addDouble("track.verticalReserved5", track.dVerticalReserved5);
    addDouble("track.verticalReserved4", track.dVerticalReserved4);
    addDouble("track.verticalReserved3", track.dVerticalReserved3);
    addDouble("track.verticalReserved2", track.dVerticalReserved2);
    addDouble("track.verticalReserved1", track.dVerticalReserved1);

    return QString::fromLatin1(QCryptographicHash::hash(
        QJsonDocument(values).toJson(QJsonDocument::Compact),
        QCryptographicHash::Sha256).toHex()).toLower();
}

struct MeasurementPoseReference
{
    bool valid = false;
    int count = 0;
    QString source;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double rx = 0.0;
    double ry = 0.0;
    double rz = 0.0;
};

bool IsFiniteCameraPoint(const Eigen::Vector3d& point)
{
    return std::isfinite(point.x())
        && std::isfinite(point.y())
        && std::isfinite(point.z());
}

bool ShouldSkipLaserCalc(const TimestampedCameraPoint& sample)
{
    if (!IsFiniteCameraPoint(sample.point))
    {
        return true;
    }

    constexpr double kZeroPointEps = 1e-9;
    return std::abs(sample.point.x()) <= kZeroPointEps
        && std::abs(sample.point.y()) <= kZeroPointEps
        && std::abs(sample.point.z()) <= kZeroPointEps;
}

QString CsvEscape(const QString& value)
{
    QString escaped = value;
    escaped.replace("\"", "\"\"");
    if (escaped.contains(',') || escaped.contains('"') || escaped.contains('\n') || escaped.contains('\r'))
    {
        escaped = "\"" + escaped + "\"";
    }
    return escaped;
}



struct RobotInterpolationWindow
{
    int prevIndex = -1;
    int nextIndex = -1;
    qint64 prevTimestampUs = 0;
    qint64 nextTimestampUs = 0;
    double ratio = 0.0;
};

struct QueuedScanCameraFrame
{
    int sampleIndex = 0;
    qint64 rawTimestampUs = 0;
    qint64 rawDeltaUs = 0;
    qint64 timestampUs = 0;
    udpDataShow frame;
};

struct ProcessedScanWorkpiecePoint
{
    Eigen::Vector3d workpiecePoint = Eigen::Vector3d::Zero();
    // 相机坐标系下的原始点（手眼变换前），仅供完整点云逐帧调试导出排查相机端散点用。
    Eigen::Vector3d cameraPoint = Eigen::Vector3d::Zero();
};

struct ProcessedScanCameraSample
{
    TimestampedCameraPoint sample;
    QString status;
    RobotInterpolationWindow robotWindow;
    bool hasRobotPose = false;
    bool hasLaserPoint = false;
    bool contributedWorkpieceFrame = false;
    T_ROBOT_COORS robotPose;
    Eigen::Vector3d laserPoint = Eigen::Vector3d::Zero();
    std::vector<ProcessedScanWorkpiecePoint> workpiecePoints;
    int skippedWorkpieceCloudPointCount = 0;
};

RobotInterpolationWindow FindRobotInterpolationWindow(
    const std::vector<RobotCalculation::TimestampedRobotPose>& robotSamples,
    qint64 targetTimestampUs)
{
    RobotInterpolationWindow window;
    if (robotSamples.empty())
    {
        return window;
    }

    if (targetTimestampUs <= robotSamples.front().timestampUs)
    {
        window.prevIndex = 1;
        window.nextIndex = robotSamples.size() > 1 ? 2 : 1;
        window.prevTimestampUs = robotSamples.front().timestampUs;
        window.nextTimestampUs = robotSamples[static_cast<std::size_t>(window.nextIndex - 1)].timestampUs;
        return window;
    }

    if (targetTimestampUs >= robotSamples.back().timestampUs)
    {
        window.nextIndex = static_cast<int>(robotSamples.size());
        window.prevIndex = robotSamples.size() > 1 ? window.nextIndex - 1 : window.nextIndex;
        window.prevTimestampUs = robotSamples[static_cast<std::size_t>(window.prevIndex - 1)].timestampUs;
        window.nextTimestampUs = robotSamples.back().timestampUs;
        window.ratio = 1.0;
        return window;
    }

    const auto upper = std::lower_bound(
        robotSamples.begin(),
        robotSamples.end(),
        targetTimestampUs,
        [](const RobotCalculation::TimestampedRobotPose& sample, qint64 timestamp)
        {
            return sample.timestampUs < timestamp;
        });
    const auto lower = upper - 1;
    window.prevIndex = static_cast<int>(std::distance(robotSamples.begin(), lower)) + 1;
    window.nextIndex = static_cast<int>(std::distance(robotSamples.begin(), upper)) + 1;
    window.prevTimestampUs = lower->timestampUs;
    window.nextTimestampUs = upper->timestampUs;
    const qint64 dt = window.nextTimestampUs - window.prevTimestampUs;
    window.ratio = dt == 0 ? 0.0 : static_cast<double>(targetTimestampUs - window.prevTimestampUs) / static_cast<double>(dt);
    return window;
}

RobotCalculation::SampleAxis InferMeasureSampleAxis(const T_PRECISE_MEASURE_PARAM& param)
{
    const double deltaX = std::abs(param.tEndPos.dX - param.tStartPos.dX);
    const double deltaY = std::abs(param.tEndPos.dY - param.tStartPos.dY);
    return deltaX > deltaY
        ? RobotCalculation::SampleAxis::AxisX
        : RobotCalculation::SampleAxis::AxisY;
}

QString SampleAxisName(RobotCalculation::SampleAxis axis)
{
    return axis == RobotCalculation::SampleAxis::AxisX ? "X" : "Y";
}

QString GeometryStrategyName(RobotCalculation::LowerWeldGeometryStrategy strategy)
{
    switch (strategy)
    {
    case RobotCalculation::LowerWeldGeometryStrategy::WorkpieceProjection:
        return PointCloudProcessingConfig::FeaturePointStrategyDisplayName(
            PointCloudProcessingConfig::FeaturePointStrategy::WorkpieceProjection);
    case RobotCalculation::LowerWeldGeometryStrategy::SlopeWaveFiltered:
        return PointCloudProcessingConfig::FeaturePointStrategyDisplayName(
            PointCloudProcessingConfig::FeaturePointStrategy::SlopeWaveFiltered);
    case RobotCalculation::LowerWeldGeometryStrategy::RobustSegmentedKeys:
        return PointCloudProcessingConfig::FeaturePointStrategyDisplayName(
            PointCloudProcessingConfig::FeaturePointStrategy::RobustSegmentedKeys);
    case RobotCalculation::LowerWeldGeometryStrategy::LegacyGeometry:
    default:
        return PointCloudProcessingConfig::FeaturePointStrategyDisplayName(
            PointCloudProcessingConfig::FeaturePointStrategy::LegacyGeometry);
    }
}

QString PoseCornerGroupModule(int groupIndex)
{
    return QStringLiteral("WeldPoseCompParam/CornerCompensation/Group%1").arg(groupIndex);
}

QString PoseCornerSlotModule(int flatIndex)
{
    return QStringLiteral("WeldPoseCompParam/CornerCompensation/Slot%1").arg(flatIndex);
}

bool ReadRobotScopedSetting(const QString& robotName, const QString& module, const QString& key, QString* value)
{
    if (robotName.trimmed().isEmpty())
    {
        return false;
    }
    return ConfigDatabase::ReadScopedSetting(QStringLiteral("robot"), robotName.trimmed(), module, key, value);
}

int ReadRobotScopedInt(const QString& robotName, const QString& module, const QString& key, int defaultValue)
{
    QString value;
    if (!ReadRobotScopedSetting(robotName, module, key, &value))
    {
        return defaultValue;
    }
    bool ok = false;
    const int parsed = value.trimmed().toInt(&ok);
    return ok ? parsed : defaultValue;
}

bool ReadRobotScopedBool(const QString& robotName, const QString& module, const QString& key, bool defaultValue)
{
    QString value;
    if (!ReadRobotScopedSetting(robotName, module, key, &value))
    {
        return defaultValue;
    }
    const QString normalized = value.trimmed().toLower();
    return normalized == QStringLiteral("1")
        || normalized == QStringLiteral("true")
        || normalized == QStringLiteral("yes");
}

double ReadRobotScopedDouble(const QString& robotName, const QString& module, const QString& key, double defaultValue)
{
    QString value;
    if (!ReadRobotScopedSetting(robotName, module, key, &value))
    {
        return defaultValue;
    }
    bool ok = false;
    const double parsed = value.trimmed().toDouble(&ok);
    return ok ? parsed : defaultValue;
}

RobotCalculation::LowerWeldFilterParams::CornerCompensation ReadCornerCompensationSlot(
    const QString& robotName,
    int flatIndex)
{
    RobotCalculation::LowerWeldFilterParams::CornerCompensation comp;
    const QString module = PoseCornerSlotModule(flatIndex);
    comp.innerToOuterMm = ReadRobotScopedDouble(robotName, module, INNER_TO_OUTER_CORNER_COMP_KEY, 0.0);
    comp.innerToInnerMm = ReadRobotScopedDouble(robotName, module, INNER_TO_INNER_CORNER_COMP_KEY, 0.0);
    comp.outerToOuterMm = ReadRobotScopedDouble(robotName, module, OUTER_TO_OUTER_CORNER_COMP_KEY, 0.0);
    comp.outerToInnerMm = ReadRobotScopedDouble(robotName, module, OUTER_TO_INNER_CORNER_COMP_KEY, 0.0);
    return comp;
}

bool HasCornerCompensationValue(const RobotCalculation::LowerWeldFilterParams::CornerCompensation& comp)
{
    return std::abs(comp.innerToOuterMm) > 1e-9
        || std::abs(comp.innerToInnerMm) > 1e-9
        || std::abs(comp.outerToOuterMm) > 1e-9
        || std::abs(comp.outerToInnerMm) > 1e-9;
}

void LoadActivePoseCornerCompensation(
    const QString& robotName,
    RobotCalculation::LowerWeldFilterParams& params)
{
    const QString allModule = QStringLiteral("WeldPoseCompParam/ALLWeldPoseComp");
    int activeGroupIndex = ReadRobotScopedInt(robotName, allModule, POSE_ACTIVE_GROUP_INDEX_KEY, 0);
    const int groupCount = ReadRobotScopedInt(robotName, allModule, POSE_GROUP_COUNT_KEY, activeGroupIndex + 1);
    if (groupCount > 0)
    {
        activeGroupIndex = std::clamp(activeGroupIndex, 0, groupCount - 1);
    }
    else
    {
        activeGroupIndex = 0;
    }

    if (!ReadRobotScopedBool(robotName, PoseCornerGroupModule(activeGroupIndex), CORNER_COMP_ENABLED_KEY, false))
    {
        params.enableCornerCompensation = false;
        params.risingCornerCompensation = {};
        params.fallingCornerCompensation = {};
        return;
    }

    const int offset = activeGroupIndex * POSE_COMP_SEGMENT_COUNT;
    params.risingCornerCompensation = ReadCornerCompensationSlot(robotName, offset + 1);
    params.fallingCornerCompensation = ReadCornerCompensationSlot(robotName, offset + 3);
    params.enableCornerCompensation =
        HasCornerCompensationValue(params.risingCornerCompensation)
        || HasCornerCompensationValue(params.fallingCornerCompensation);
}

RobotCalculation::LowerWeldFilterParams BuildOriginalTrackFitParams(
    const T_PRECISE_MEASURE_PARAM& param,
    const PointCloudProcessingConfig::Settings& settings)
{
    // 参数名册唯一来源在 MeasureThenWeldService::BuildTrackFitParamsFromSettings（CLI 共用）。
    RobotCalculation::LowerWeldFilterParams params = MeasureThenWeldService::BuildTrackFitParamsFromSettings(
        settings,
        InferMeasureSampleAxis(param));
    LoadActivePoseCornerCompensation(QString::fromStdString(param.sRobotName), params);
    return params;
}

Eigen::Vector3d BuildScanDirection(const T_PRECISE_MEASURE_PARAM& param)
{
    double overrideX = 0.0;
    double overrideY = 0.0;
    double overrideZ = 0.0;
    if (PointCloudProcessingConfig::RuntimeScanDirectionOverride(&overrideX, &overrideY, &overrideZ))
    {
        Eigen::Vector3d overrideDirection(overrideX, overrideY, overrideZ);
        if (overrideDirection.norm() <= std::numeric_limits<double>::epsilon())
        {
            return Eigen::Vector3d::UnitX();
        }
        return overrideDirection.normalized();
    }

    Eigen::Vector3d direction(
        param.tEndPos.dX - param.tStartPos.dX,
        param.tEndPos.dY - param.tStartPos.dY,
        param.tEndPos.dZ - param.tStartPos.dZ);
    if (direction.norm() <= std::numeric_limits<double>::epsilon())
    {
        direction = Eigen::Vector3d::UnitX();
    }
    direction.normalize();
    return direction;
}

QVector<RobotCalculation::IndexedPoint3D> ToIndexedInput(
    const QVector<RobotCalculation::LowerWeldFilterPoint>& points);
QVector<RobotCalculation::IndexedPoint3D> ToIndexedInput(
    const QVector<PointCloudExtractionProcessor::TrackPoint>& points);
std::vector<QString> BuildRawLaserOutputLines(const QVector<RobotCalculation::LowerWeldFilterPoint>& points);

// SDK 已焊起点截断：已焊段从上次焊接的起始端延伸，截掉的一侧由"焊接方向"决定——
// 起点到终点焊（weldFromTrackStart=true）已焊段在轨迹头部，保留交界点及其后段并把首点重置为起点；
// 终点到起点焊已焊段在轨迹尾部，保留头部到交界点的段并把尾点重置为终点。
// 两种方向下执行（含方向反转）都恰好从已焊交界处开始接续焊接。
QVector<PointCloudExtractionProcessor::TrackPoint> TruncateTrackAtWeldedStart(
    const QVector<PointCloudExtractionProcessor::TrackPoint>& points,
    const Eigen::Vector3d& weldedStart,
    bool weldFromTrackStart,
    int* removedCount)
{
    if (removedCount != nullptr)
    {
        *removedCount = 0;
    }
    if (points.size() < 2)
    {
        return points;
    }
    int nearestIndex = 0;
    double bestDistance = std::numeric_limits<double>::max();
    for (int index = 0; index < points.size(); ++index)
    {
        const double distance = (points[index].point - weldedStart).norm();
        if (distance < bestDistance)
        {
            bestDistance = distance;
            nearestIndex = index;
        }
    }

    if (weldFromTrackStart)
    {
        // 截掉头部已焊段；截断后至少保留两个点，否则视为截断无效。
        if (nearestIndex <= 0 || points.size() - nearestIndex < 2)
        {
            return points;
        }
        QVector<PointCloudExtractionProcessor::TrackPoint> truncated = points.mid(nearestIndex);
        truncated.first().type = PointCloudExtractionProcessor::TrackPointType::Start;
        if (removedCount != nullptr)
        {
            *removedCount = nearestIndex;
        }
        return truncated;
    }

    // 终点到起点焊：截掉尾部已焊段。
    if (nearestIndex >= points.size() - 1 || nearestIndex < 1)
    {
        return points;
    }
    QVector<PointCloudExtractionProcessor::TrackPoint> truncated = points.mid(0, nearestIndex + 1);
    truncated.last().type = PointCloudExtractionProcessor::TrackPointType::End;
    if (removedCount != nullptr)
    {
        *removedCount = points.size() - truncated.size();
    }
    return truncated;
}

// 方法基础焊道行格式与 PreciseLaserPoint 系列一致：index x y z。
template <typename Container>
std::vector<QString> BuildMethodTrackLines(const Container& points)
{
    std::vector<QString> lines;
    lines.reserve(static_cast<size_t>(points.size()));
    for (const auto& point : points)
    {
        lines.push_back(QString("%1 %2 %3 %4")
            .arg(point.index)
            .arg(point.point.x(), 0, 'f', 6)
            .arg(point.point.y(), 0, 'f', 6)
            .arg(point.point.z(), 0, 'f', 6));
    }
    return lines;
}

bool WriteTextLinesToFile(const QString& path, const std::vector<QString>& lines, QString* error)
{
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text))
    {
        if (error != nullptr)
        {
            *error = QString("无法写入文件：%1（%2）").arg(path, file.errorString());
        }
        return false;
    }

    QTextStream stream(&file);
    for (const QString& line : lines)
    {
        stream << line << '\n';
    }
    stream.flush();
    file.close();
    return true;
}

// 工件模型缓存：处理成功后用内存中的完整点云直接生成（免二次解析 150MB 文本），
// 缓存已存在则跳过；之后查看器/CloudCompare 直接秒开。失败不影响焊接流程。
void EnsureWorkpieceMeshCacheFromCloud(
    const QString& laserDir,
    const QVector<RobotCalculation::IndexedPoint3D>& cloudPoints,
    const MeasureThenWeldService::LogCallback& appendLog)
{
    if (laserDir.trimmed().isEmpty() || cloudPoints.size() < 16)
    {
        return;
    }
    const QString cachePath = WorkpieceMeshBuilder::MeshCachePath(laserDir);
    if (WorkpieceMeshBuilder::IsMeshCacheValid(cachePath))
    {
        return;
    }
    QString meshError;
    WorkpieceMeshBuilder::Mesh mesh;
    if (!WorkpieceMeshBuilder::BuildFromScanlineCloud(cloudPoints, mesh, meshError)
        || !WorkpieceMeshBuilder::SaveMeshPly(cachePath, mesh, meshError))
    {
        if (appendLog)
        {
            appendLog(QString("工件模型缓存生成失败（不影响流程）：%1").arg(meshError));
        }
        return;
    }
    if (appendLog)
    {
        appendLog(QString("已生成工件模型缓存：%1（顶点 %2 / 三角形 %3）")
            .arg(cachePath)
            .arg(mesh.vertices.size())
            .arg(mesh.indices.size() / 3));
    }
}

// 处理成功时把"该方法的基础焊道"落盘到 LaserPoint 目录（文件存在=该方法已完成焊道生成）。
void SaveMethodBaseTrackFile(
    const QString& laserDir,
    PointCloudProcessingConfig::Mode mode,
    const std::vector<QString>& lines,
    const MeasureThenWeldService::LogCallback& appendLog)
{
    if (laserDir.trimmed().isEmpty() || lines.empty())
    {
        return;
    }
    const QString path = QDir(laserDir).filePath(MeasureThenWeldService::MethodBaseTrackFileName(mode));
    QString saveError;
    if (!WriteTextLinesToFile(path, lines, &saveError) && appendLog)
    {
        appendLog(QString("方法基础焊道写入失败（不影响处理）：%1").arg(saveError));
    }
}

// 对 SDK 基础焊道(稠密有序点)做"结构自适应 1D 双边"预平滑去锯齿（仅 SdkBaseWeldFit 模式拟合前调用）。
// 沿弧长参数化；每点用 空间核(σs=windowMm) × 值域核 加权邻点平均，值域距离取"邻点到本点局部切线的
// 垂直偏移"——直线段内垂直偏移≈0→正常平均磨掉锯齿；搭接X台阶/真实折角处垂直偏移大(>σr=edgeMm)→权重
// 趋零→不跨过去平均，从机制上保住台阶与折角(不会把搭接段圆滑进去)。端点(起/终)不动。非迭代单次 O(N·窗)。
// 返回被移动的点数(供日志)。
int BilateralPresmoothSdkBaseWeld(
    QVector<PointCloudExtractionProcessor::TrackPoint>& pts,
    double windowMm,
    double edgeMm,
    const MeasureThenWeldService::StopRequestedCallback& stopRequested)
{
    const int n = pts.size();
    if (n < 5 || windowMm <= 1e-6 || edgeMm <= 1e-6)
    {
        return 0;
    }

    std::vector<double> arc(n, 0.0);
    for (int i = 1; i < n; ++i)
    {
        if ((i & 0x3ff) == 0 && stopRequested && stopRequested())
        {
            return -1;
        }
        arc[i] = arc[i - 1] + (pts[i].point - pts[i - 1].point).norm();
    }

    const int tanWin = 2;  // 局部切线用 ±tanWin 个点中心差分估，抗单点抖动
    auto localTangent = [&](int i) -> Eigen::Vector3d
    {
        const int a = std::max(0, i - tanWin);
        const int b = std::min(n - 1, i + tanWin);
        Eigen::Vector3d t = pts[b].point - pts[a].point;
        const double nrm = t.norm();
        return nrm > 1e-9 ? Eigen::Vector3d(t / nrm) : Eigen::Vector3d(0.0, 1.0, 0.0);
    };

    const double sigmaS2x2 = 2.0 * windowMm * windowMm;
    const double sigmaR2x2 = 2.0 * edgeMm * edgeMm;
    const double halfSpan = 3.0 * windowMm;  // 高斯 3σ 截断

    std::vector<Eigen::Vector3d> out(n);
    int moved = 0;
    for (int i = 0; i < n; ++i)
    {
        if ((i & 0xff) == 0 && stopRequested && stopRequested())
        {
            return -1;
        }
        if (pts[i].type == PointCloudExtractionProcessor::TrackPointType::Start
            || pts[i].type == PointCloudExtractionProcessor::TrackPointType::End)
        {
            out[i] = pts[i].point;
            continue;
        }
        const Eigen::Vector3d ti = localTangent(i);
        Eigen::Vector3d acc = pts[i].point;  // 中心点 ds=0,perp=0 → 权重 1
        double wsum = 1.0;
        for (int j = i - 1; j >= 0; --j)
        {
            const double ds = arc[i] - arc[j];
            if (ds > halfSpan) break;
            const Eigen::Vector3d d = pts[j].point - pts[i].point;
            const double perp = (d - d.dot(ti) * ti).norm();
            const double w = std::exp(-ds * ds / sigmaS2x2) * std::exp(-perp * perp / sigmaR2x2);
            acc += w * pts[j].point;
            wsum += w;
        }
        for (int j = i + 1; j < n; ++j)
        {
            const double ds = arc[j] - arc[i];
            if (ds > halfSpan) break;
            const Eigen::Vector3d d = pts[j].point - pts[i].point;
            const double perp = (d - d.dot(ti) * ti).norm();
            const double w = std::exp(-ds * ds / sigmaS2x2) * std::exp(-perp * perp / sigmaR2x2);
            acc += w * pts[j].point;
            wsum += w;
        }
        out[i] = (wsum > 1e-12) ? Eigen::Vector3d(acc / wsum) : pts[i].point;
        if ((out[i] - pts[i].point).norm() > 1e-6)
        {
            ++moved;
        }
    }
    for (int i = 0; i < n; ++i)
    {
        pts[i].point = out[i];
    }
    return moved;
}

RobotCalculation::MeasureThenWeldAnalysisResult AnalyzeMeasureThenWeldPointCloud(
    const QVector<RobotCalculation::IndexedPoint3D>& legacyLaserInput,
    const QVector<RobotCalculation::IndexedPoint3D>& fullCloudInput,
    const T_PRECISE_MEASURE_PARAM& param,
    const PointCloudProcessingConfig::Settings& settings,
    const RobotCalculation::LowerWeldFilterParams& fitParams,
    const QString& sdkBaseWeldOutputPath,
    const QString& methodTrackOutputDir,
    const MeasureThenWeldService::LogCallback& appendLog,
    bool* usedExternalLibrary = nullptr,
    PointCloudExtractionProcessor::ExtractionResult* externalExtraction = nullptr,
    const MeasureThenWeldService::StopRequestedCallback& stopRequested =
        MeasureThenWeldService::StopRequestedCallback())
{
    const auto canceledResult = []()
    {
        RobotCalculation::MeasureThenWeldAnalysisResult canceled;
        canceled.error = QStringLiteral("已取消点云特征分析。");
        return canceled;
    };
    const auto isCanceled = [&stopRequested]()
    {
        return stopRequested && stopRequested();
    };
    if (isCanceled())
    {
        return canceledResult();
    }
    if (usedExternalLibrary != nullptr)
    {
        *usedExternalLibrary = false;
    }
    if (externalExtraction != nullptr)
    {
        *externalExtraction = PointCloudExtractionProcessor::ExtractionResult();
    }

    if (appendLog)
    {
        appendLog(QString("精测点云处理方式：%1。")
            .arg(PointCloudProcessingConfig::ModeDisplayName(settings.mode)));
        appendLog(QString("特征点拟合方案：%1。")
            .arg(GeometryStrategyName(fitParams.geometryStrategy)));
        if (fitParams.enableEdgeTruncate && (fitParams.truncateHeadMm > 0.0 || fitParams.truncateTailMm > 0.0))
        {
            appendLog(QString("基础焊道首尾截断已启用：开头%1mm / 结尾%2mm（按点列扫描顺序，与焊接方向无关；对②③④拟合方案生效）。")
                .arg(fitParams.truncateHeadMm, 0, 'f', 1)
                .arg(fitParams.truncateTailMm, 0, 'f', 1));
        }
        if (fitParams.enableEndPeriodCornerRecover)
        {
            appendLog(QString("端区周期补拐点已启用：端段长≥%1×周期判漏补、最小确认弯折角%2°（仅补点；对②③④生效）。")
                .arg(fitParams.endPeriodRatioThreshold, 0, 'f', 2)
                .arg(fitParams.endPeriodMinBendDeg, 0, 'f', 1));
        }
        if (fitParams.enableSameTypeShortCornerMerge)
        {
            appendLog(QString("同类短段多余拐点合并已启用：候选段短于%1×同类完整段中位、每类至少%2个参考段、坡/平台斜率分界=%3（按IO/OI/II/OO分别统计，排除首末与搭接段；仅删点；对②③④生效）。")
                .arg(fitParams.endPeriodMergeFrac, 0, 'f', 2)
                .arg(fitParams.sameTypeShortMinReferenceSegments)
                .arg(fitParams.sameTypeShortFlatSlope, 0, 'f', 2));
        }
        if (fitParams.enablePlatformCornerSnap)
        {
            appendLog(QString("按平台边界重定拐点已启用：侧向斜率<%1判为平台、平台最小长度=%2×周期（检测平的段，把拐点归位到平台两端、删平台内放错的角，治拐点放平台中间致平台消失；对正确平台幂等不动；对②③④生效）。")
                .arg(fitParams.platformSnapFlatSlope, 0, 'f', 2)
                .arg(fitParams.platformSnapMinFrac, 0, 'f', 2));
        }
    }

    const bool sdkMode = settings.mode == PointCloudProcessingConfig::Mode::ExternalCorrugatedSheet
        || settings.mode == PointCloudProcessingConfig::Mode::SdkBaseWeldFit;
    if (sdkMode)
    {
        // ①SDK全处理：SDK 拐点(+2mm扩充)直接转结果，不经拟合、不生成基础焊道文件；
        // ②SDK+拟合：SDK 输出基础焊道（稠密），再喂滤波拟合提取特征点。
        // 失败直接报错，不回退其他方法。
        const bool useBaseWeldFit = settings.mode == PointCloudProcessingConfig::Mode::SdkBaseWeldFit;
        if (fullCloudInput.size() < 2)
        {
            RobotCalculation::MeasureThenWeldAnalysisResult failed;
            failed.error = QString("SDK点云算法输入点过少（%1），无法调用外部库。")
                .arg(fullCloudInput.size());
            if (appendLog)
            {
                appendLog(failed.error);
            }
            return failed;
        }
        // 进程隔离调用 SDK：SDK(pcl_kdtree 多线程)崩溃时拦截为可报告错误、主程序不挂(防崩护栏)。
        const PointCloudExtractionProcessor::ExtractionResult extraction =
            PointCloudExtractionProcessor::ExtractCorrugatedSheetIsolated(
                fullCloudInput,
                settings,
                BuildScanDirection(param),
                useBaseWeldFit ? sdkBaseWeldOutputPath : QString(),
                stopRequested);
        if (isCanceled())
        {
            return canceledResult();
        }
        if (!extraction.ok)
        {
            RobotCalculation::MeasureThenWeldAnalysisResult failed;
            failed.error = QString("SDK点云算法处理失败：%1").arg(extraction.error);
            if (appendLog)
            {
                appendLog(failed.error);
            }
            return failed;
        }
        // 已焊起点截断（开关控制）：SDK 检测到已焊段时，按焊接方向截掉焊道已焊部分只焊剩余段。
        PointCloudExtractionProcessor::ExtractionResult workingExtraction = extraction;
        if (settings.sdkUseWeldedStartTruncation)
        {
            if (extraction.hasWeldedStartPoint)
            {
                const bool weldFromTrackStart = param.nWeldDirection >= 0;
                int removedCount = 0;
                workingExtraction.points = TruncateTrackAtWeldedStart(
                    extraction.points, extraction.weldedStartPoint, weldFromTrackStart, &removedCount);
                if (appendLog)
                {
                    if (removedCount > 0)
                    {
                        appendLog(QString("已焊起点截断：交界点(%1, %2, %3)，焊接方向=%4，截除已焊侧 %5 点，保留 %6 点。")
                            .arg(extraction.weldedStartPoint.x(), 0, 'f', 2)
                            .arg(extraction.weldedStartPoint.y(), 0, 'f', 2)
                            .arg(extraction.weldedStartPoint.z(), 0, 'f', 2)
                            .arg(weldFromTrackStart ? "起点到终点" : "终点到起点")
                            .arg(removedCount)
                            .arg(workingExtraction.points.size()));
                    }
                    else
                    {
                        appendLog("已焊起点截断：交界点位于焊道端部或截断后点数不足，焊道未截断。");
                    }
                }
            }
            else if (appendLog)
            {
                appendLog("已焊起点截断：SDK 未检测到已焊段，焊道不截断。");
            }
        }
        RobotCalculation::MeasureThenWeldAnalysisResult analysis;
        if (useBaseWeldFit)
        {
            // SDK 基础点云 + 滤波拟合流程固定走"干净输入"路径：输入是 SDK 重建的稠密有序焊道，
            // 跳过程序自身的去噪/平滑（重复处理会削圆尖角、移位拐点），拐点检测用方位角法
            //（避免 DP 在缓变/台阶拐角标偏、漏检、抄近路）。方位角参数(转角阈值/窗口/NMS/兜底)
            // 随 fitParams 由配置开放，现场可调。只改本次局部拷贝，不动 fitParams 本体。
            RobotCalculation::LowerWeldFilterParams baseWeldParams = fitParams;
            baseWeldParams.inputAlreadyDenoised = true;
            // SDK 基础焊道预平滑(可选, 默认关)：SDK 重建焊道偶有锯齿，拟合第一步前先做结构自适应双边去锯齿；
            // 靠"到局部切线垂直偏移"的值域核跨搭接X台阶/折角自动不平滑。仅本路径，不影响③点云+拟合/④特征点+拟合。
            if (settings.sdkBasePresmoothEnable)
            {
                const int movedCount = BilateralPresmoothSdkBaseWeld(
                    workingExtraction.points,
                    settings.sdkBasePresmoothWindowMm,
                    settings.sdkBasePresmoothEdgeMm,
                    stopRequested);
                if (movedCount < 0)
                {
                    return canceledResult();
                }
                if (appendLog)
                {
                    appendLog(QString("SDK基础焊道预平滑(结构自适应双边)：窗口=%1mm，保边阈值=%2mm，平滑点=%3/%4。")
                        .arg(settings.sdkBasePresmoothWindowMm, 0, 'f', 2)
                        .arg(settings.sdkBasePresmoothEdgeMm, 0, 'f', 2)
                        .arg(movedCount)
                        .arg(workingExtraction.points.size()));
                }
            }
            analysis = RobotCalculation::AnalyzeMeasureThenWeldLowerWeldPathDirect(
                ToIndexedInput(workingExtraction.points), baseWeldParams);
        }
        else
        {
            RobotCalculation::LowerWeldFilterParams sdkDirectParams = fitParams;
            // SDK 直出轨迹按 External/ResampleStepMm 扩充；连续性分箱必须使用同一采样步长，
            // 不能沿用 Fit/SampleStepMm，否则会对完美等距轨迹产生假空洞或掩盖真实空洞。
            sdkDirectParams.sampleStep = std::max(0.1, settings.resampleStepMm);
            analysis = PointCloudExtractionProcessor::BuildAnalysisResult(
                workingExtraction, sdkDirectParams);
        }
        if (!analysis.ok)
        {
            if (appendLog)
            {
                appendLog(QString("SDK点云算法结果%1失败：%2")
                    .arg(useBaseWeldFit ? "拟合" : "转换")
                    .arg(analysis.error));
            }
            return analysis;
        }
        if (usedExternalLibrary != nullptr)
        {
            *usedExternalLibrary = true;
        }
        if (externalExtraction != nullptr)
        {
            *externalExtraction = extraction;
        }
        if (appendLog)
        {
            appendLog(QString("SDK点云算法处理完成（%1）：输入局部完整点云=%2，SDK输出点=%3，DLL=%4，配置=%5，Z截断=%6 mm。")
                .arg(useBaseWeldFit ? "基础焊道+拟合" : "拐点直接使用")
                .arg(extraction.inputPointCount)
                .arg(extraction.points.size())
                .arg(extraction.dllPath)
                .arg(extraction.configPath)
                .arg(settings.zTruncationValue, 0, 'f', 3));
            if (extraction.usedBaseWeldFile)
            {
                appendLog(QString("SDK基础焊道来自库输出文件：%1").arg(extraction.baseWeldPath));
            }
        }
        SaveMethodBaseTrackFile(
            methodTrackOutputDir, settings.mode, BuildMethodTrackLines(workingExtraction.points), appendLog);
        if (isCanceled())
        {
            return canceledResult();
        }
        return analysis;
    }
    if (settings.mode == PointCloudProcessingConfig::Mode::CloudFit)
    {
        // ③点云算法+拟合：立板投影提取（完整点云 + 相机轨迹种子 → 下层轨迹）→ 滤波拟合。
        // 几何链按单条轨迹设计，面状完整点云必须先经投影提取压成轨迹。失败直接报错，不回退。
        if (fullCloudInput.size() < 3)
        {
            RobotCalculation::MeasureThenWeldAnalysisResult failed;
            failed.error = QString("点云算法+拟合输入完整点云点数过少（%1）。").arg(fullCloudInput.size());
            if (appendLog)
            {
                appendLog(failed.error);
            }
            return failed;
        }
        if (legacyLaserInput.size() < 2)
        {
            RobotCalculation::MeasureThenWeldAnalysisResult failed;
            failed.error = QString("点云算法+拟合缺少相机轨迹种子点（%1），无法定位底板候选。")
                .arg(legacyLaserInput.size());
            if (appendLog)
            {
                appendLog(failed.error);
            }
            return failed;
        }
        const RobotCalculation::LowerWeldFilterResult projectedPath =
            RobotCalculation::ProjectWorkpieceCloudToLowerWeldPath(fullCloudInput, legacyLaserInput, fitParams);
        if (isCanceled())
        {
            return canceledResult();
        }
        if (!projectedPath.ok)
        {
            RobotCalculation::MeasureThenWeldAnalysisResult failed;
            failed.error = QString("点云投影提取失败：%1").arg(projectedPath.error);
            if (appendLog)
            {
                appendLog(failed.error);
            }
            return failed;
        }
        if (appendLog)
        {
            appendLog(QString("点云投影提取完成：完整点云=%1，种子点=%2，底板候选点=%3，投影轨迹点=%4。")
                .arg(fullCloudInput.size())
                .arg(legacyLaserInput.size())
                .arg(projectedPath.lowerPointCount)
                .arg(projectedPath.points.size()));
        }
        RobotCalculation::MeasureThenWeldAnalysisResult analysis =
            RobotCalculation::AnalyzeMeasureThenWeldLowerWeldPathDirect(
                ToIndexedInput(projectedPath.points), fitParams);
        if (isCanceled())
        {
            return canceledResult();
        }
        if (!analysis.ok)
        {
            if (appendLog)
            {
                appendLog(QString("点云算法+拟合处理失败：%1").arg(analysis.error));
            }
            return analysis;
        }
        SaveMethodBaseTrackFile(
            methodTrackOutputDir,
            PointCloudProcessingConfig::Mode::CloudFit,
            BuildMethodTrackLines(projectedPath.points),
            appendLog);
        return analysis;
    }

    // ④特征点+拟合：相机目标轨迹点 → 滤波拟合。
    if (appendLog)
    {
        appendLog(QString("特征点+拟合输入轨迹点数：%1。")
            .arg(legacyLaserInput.size()));
    }
    RobotCalculation::MeasureThenWeldAnalysisResult legacyAnalysis =
        RobotCalculation::AnalyzeMeasureThenWeldLowerWeldPathDirect(legacyLaserInput, fitParams);
    if (legacyAnalysis.ok)
    {
        SaveMethodBaseTrackFile(
            methodTrackOutputDir,
            PointCloudProcessingConfig::Mode::LegacyLaserPath,
            BuildMethodTrackLines(legacyLaserInput),
            appendLog);
    }
    return legacyAnalysis;
}

QVector<RobotCalculation::IndexedPoint3D> ToIndexedInput(
    const QVector<RobotCalculation::LowerWeldFilterPoint>& points)
{
    QVector<RobotCalculation::IndexedPoint3D> indexedPoints;
    indexedPoints.reserve(points.size());
    for (const RobotCalculation::LowerWeldFilterPoint& point : points)
    {
        RobotCalculation::IndexedPoint3D indexedPoint;
        indexedPoint.index = point.index;
        indexedPoint.point = point.point;
        indexedPoints.push_back(indexedPoint);
    }
    return indexedPoints;
}

QVector<RobotCalculation::IndexedPoint3D> ToIndexedInput(
    const QVector<PointCloudExtractionProcessor::TrackPoint>& points)
{
    QVector<RobotCalculation::IndexedPoint3D> indexedPoints;
    indexedPoints.reserve(points.size());
    for (const PointCloudExtractionProcessor::TrackPoint& point : points)
    {
        RobotCalculation::IndexedPoint3D indexedPoint;
        indexedPoint.index = point.index;
        indexedPoint.point = point.point;
        indexedPoints.push_back(indexedPoint);
    }
    return indexedPoints;
}

std::vector<QString> BuildFilterOutputLines(const RobotCalculation::LowerWeldFilterResult& result)
{
    std::vector<QString> lines;
    lines.reserve(static_cast<size_t>(result.points.size()) + 1);
    lines.push_back("index x y z source");
    for (const RobotCalculation::LowerWeldFilterPoint& point : result.points)
    {
        lines.push_back(RobotCalculation::Vector3IndexedSpaceText(point.index, point.point, point.source));
    }
    return lines;
}

std::vector<QString> BuildIndexedPointOutputLines(
    const QVector<RobotCalculation::IndexedPoint3D>& points,
    const QString& source)
{
    std::vector<QString> lines;
    lines.reserve(static_cast<size_t>(points.size()) + 1);
    lines.push_back("index x y z source");
    for (const RobotCalculation::IndexedPoint3D& point : points)
    {
        lines.push_back(QString("%1 %2 %3 %4 %5")
            .arg(point.index)
            .arg(point.point.x(), 0, 'f', 6)
            .arg(point.point.y(), 0, 'f', 6)
            .arg(point.point.z(), 0, 'f', 6)
            .arg(source));
    }
    return lines;
}

std::vector<QString> BuildRawLaserOutputLines(const QVector<RobotCalculation::LowerWeldFilterPoint>& points)
{
    std::vector<QString> lines;
    lines.reserve(static_cast<size_t>(points.size()) + 1);
    lines.push_back("index,x,y,z");
    for (const RobotCalculation::LowerWeldFilterPoint& point : points)
    {
        lines.push_back(RobotCalculation::Vector3IndexedCsv(point.index, point.point));
    }
    return lines;
}

QString SdkTrackPointTypeName(PointCloudExtractionProcessor::TrackPointType type)
{
    using TrackType = PointCloudExtractionProcessor::TrackPointType;
    switch (type)
    {
    case TrackType::Start:
        return "start";
    case TrackType::End:
        return "end";
    case TrackType::Corner:
        return "corner";
    case TrackType::Normal:
    default:
        return "normal";
    }
}

int SdkTrackPointTypeCode(PointCloudExtractionProcessor::TrackPointType type)
{
    using TrackType = PointCloudExtractionProcessor::TrackPointType;
    switch (type)
    {
    case TrackType::Start:
        return 1;
    case TrackType::End:
        return 2;
    case TrackType::Corner:
        return 3;
    case TrackType::Normal:
    default:
        return 5;
    }
}

std::vector<QString> BuildSdkTrackOutputLines(
    const QVector<PointCloudExtractionProcessor::TrackPoint>& points,
    const QString& source)
{
    std::vector<QString> lines;
    lines.reserve(static_cast<size_t>(points.size()) + 2);
    lines.push_back("# index x y z sdk_type_code sdk_type_name source");
    lines.push_back("# 1=start 2=end 3=corner 5=normal");
    for (const PointCloudExtractionProcessor::TrackPoint& point : points)
    {
        lines.push_back(QString("%1 %2 %3 %4 %5 %6 %7")
            .arg(point.index)
            .arg(point.point.x(), 0, 'f', 6)
            .arg(point.point.y(), 0, 'f', 6)
            .arg(point.point.z(), 0, 'f', 6)
            .arg(SdkTrackPointTypeCode(point.type))
            .arg(SdkTrackPointTypeName(point.type))
            .arg(source));
    }
    return lines;
}

std::vector<QString> BuildClassifiedOutputLines(const RobotCalculation::LowerWeldClassificationResult& result)
{
    std::vector<QString> lines;
    lines.reserve(static_cast<size_t>(result.points.size()) + 2);
    lines.push_back("# index x y z type_code type_name source");
    lines.push_back("# 1=start 2=end 3=inner_corner 4=outer_corner 5=normal 6=noise");
    for (const RobotCalculation::LowerWeldClassifiedPoint& point : result.points)
    {
        lines.push_back(QString("%1 %2 %3 %4 %5 %6 %7")
            .arg(point.index)
            .arg(point.point.x(), 0, 'f', 6)
            .arg(point.point.y(), 0, 'f', 6)
            .arg(point.point.z(), 0, 'f', 6)
            .arg(RobotCalculation::LowerWeldPointTypeCode(point.type))
            .arg(RobotCalculation::LowerWeldPointTypeName(point.type))
            .arg(point.source));
    }
    return lines;
}

std::vector<QString> BuildKeyPointOutputLines(const QVector<RobotCalculation::LowerWeldClassifiedPoint>& points)
{
    std::vector<QString> lines;
    lines.reserve(static_cast<size_t>(points.size()) + 2);
    lines.push_back("# source_index x y z type_code type_name source");
    lines.push_back("# 1=start 2=end 3=inner_corner 4=outer_corner");
    for (const RobotCalculation::LowerWeldClassifiedPoint& point : points)
    {
        if (point.type == RobotCalculation::LowerWeldPointType::Normal
            || point.type == RobotCalculation::LowerWeldPointType::Noise)
        {
            continue;
        }

        lines.push_back(QString("%1 %2 %3 %4 %5 %6 %7")
            .arg(point.index)
            .arg(point.point.x(), 0, 'f', 6)
            .arg(point.point.y(), 0, 'f', 6)
            .arg(point.point.z(), 0, 'f', 6)
            .arg(RobotCalculation::LowerWeldPointTypeCode(point.type))
            .arg(RobotCalculation::LowerWeldPointTypeName(point.type))
            .arg(point.source.isEmpty() ? "-" : point.source));
    }
    return lines;
}

std::vector<QString> BuildNoiseOutputLines(
    const QVector<RobotCalculation::IndexedPoint3D>& inputPoints,
    const RobotCalculation::LowerWeldFilterResult& fitResult)
{
    std::vector<QString> lines;
    QSet<int> validIndexes;
    validIndexes.reserve(fitResult.points.size());
    for (const RobotCalculation::LowerWeldFilterPoint& point : fitResult.points)
    {
        validIndexes.insert(point.index);
    }

    lines.reserve(static_cast<size_t>(inputPoints.size()) + 2);
    lines.push_back("# index x y z type_code type_name source");
    lines.push_back("# 1=start 2=end 3=inner_corner 4=outer_corner 5=normal 6=noise");
    for (const RobotCalculation::IndexedPoint3D& point : inputPoints)
    {
        if (validIndexes.contains(point.index))
        {
            continue;
        }
        lines.push_back(QString("%1 %2 %3 %4 6 noise raw")
            .arg(point.index)
            .arg(point.point.x(), 0, 'f', 6)
            .arg(point.point.y(), 0, 'f', 6)
            .arg(point.point.z(), 0, 'f', 6));
    }
    return lines;
}

RobotCalculation::LowerWeldFilterParams BuildSchemeCompareFitParams(
    const RobotCalculation::LowerWeldFilterParams& params)
{
    RobotCalculation::LowerWeldFilterParams compareParams = params;
    // 三方案对比是诊断材料，不得阻断其它方案输出；但必须计算同一套指标，供历史审计标定。
    compareParams.validationAuditOnly = true;
    compareParams.validationCoverageEnabled = true;
    compareParams.validationContinuityEnabled = true;
    compareParams.validationDenoiseRatioEnabled = true;
    compareParams.validationResidualEnabled = true;
    compareParams.validationKeyPointEnabled = true;
    compareParams.validationOutputEnabled = true;
    return compareParams;
}

std::vector<QString> BuildSchemeCompareSummaryLines(
    const QString& title,
    const QString& inputDescription,
    const QVector<RobotCalculation::IndexedPoint3D>& inputPoints,
    const RobotCalculation::MeasureThenWeldAnalysisResult& analysis)
{
    std::vector<QString> lines;
    lines.reserve(32);
    lines.push_back(QString("方案=%1").arg(title));
    lines.push_back(QString("输入=%1").arg(inputDescription));
    lines.push_back(QString("输入点数=%1").arg(inputPoints.size()));
    lines.push_back(QString("状态=%1").arg(analysis.ok ? "OK" : "FAIL"));
    if (!analysis.ok)
    {
        lines.push_back(QString("错误=%1").arg(analysis.error));
        return lines;
    }
    lines.push_back(QString("拟合输入点=%1").arg(analysis.filterResult.inputPointCount));
    lines.push_back(QString("保留点=%1").arg(analysis.filterResult.points.size()));
    lines.push_back(QString("关键点=%1").arg(analysis.keyPoints.size()));
    lines.push_back(QString("2mm扩充点=%1").arg(analysis.classificationResult.points.size()));
    lines.push_back(QString("起点=%1").arg(analysis.classificationResult.startCount));
    lines.push_back(QString("终点=%1").arg(analysis.classificationResult.endCount));
    lines.push_back(QString("内拐点=%1").arg(analysis.classificationResult.innerCornerCount));
    lines.push_back(QString("外拐点=%1").arg(analysis.classificationResult.outerCornerCount));
    const auto& quality = analysis.qualityReport;
    if (quality.evaluated)
    {
        lines.push_back(QString("质量策略=Audit"));
        lines.push_back(QString("质量判定=%1").arg(quality.passed ? "PASS" : "WARN"));
        lines.push_back(QString("质量失败项=%1").arg(quality.failures.join(" | ")));
        lines.push_back(QString("质量告警项=%1").arg(quality.warnings.join(" | ")));
        lines.push_back(QString("主轴跨度mm=%1").arg(quality.projectedSpanMm, 0, 'f', 6));
        lines.push_back(QString("站位覆盖率=%1").arg(quality.stationCoverageRatio, 0, 'f', 6));
        lines.push_back(QString("最长连续率=%1").arg(quality.longestContinuousRatio, 0, 'f', 6));
        lines.push_back(QString("剔除率=%1").arg(quality.rejectedRatio, 0, 'f', 6));
        lines.push_back(QString("中位残差mm=%1").arg(quality.medianResidualMm, 0, 'f', 6));
        lines.push_back(QString("P95残差mm=%1").arg(quality.p95ResidualMm, 0, 'f', 6));
        lines.push_back(QString("残差内点率=%1").arg(quality.residualInlierRatio, 0, 'f', 6));
        lines.push_back(QString("最短非搭接段mm=%1").arg(quality.minNonLapSegmentLengthMm, 0, 'f', 6));
        lines.push_back(QString("最短搭接段mm=%1").arg(quality.minLapStepSegmentLengthMm, 0, 'f', 6));
        lines.push_back(QString("输出总长mm=%1").arg(quality.outputLengthMm, 0, 'f', 6));
        lines.push_back(QString("输出最大步长mm=%1").arg(quality.maxOutputStepMm, 0, 'f', 6));
    }
    return lines;
}

bool SaveSchemeAnalysisOutputs(
    const MeasureThenWeldService& service,
    const QString& compareDir,
    const QString& prefix,
    const QString& title,
    const QString& inputDescription,
    const QVector<RobotCalculation::IndexedPoint3D>& inputPoints,
    const RobotCalculation::MeasureThenWeldAnalysisResult& analysis,
    QString& error)
{
    const QDir dir(compareDir);
    if (!service.SaveTextLines(
            dir.filePath(QString("%1_InputPointCloud.txt").arg(prefix)),
            BuildIndexedPointOutputLines(inputPoints, inputDescription),
            error))
    {
        return false;
    }
    if (!service.SaveTextLines(
            dir.filePath(QString("%1_Summary.txt").arg(prefix)),
            BuildSchemeCompareSummaryLines(title, inputDescription, inputPoints, analysis),
            error))
    {
        return false;
    }
    if (!analysis.ok)
    {
        return true;
    }
    return service.SaveTextLines(
            dir.filePath(QString("%1_PreservePath.txt").arg(prefix)),
            BuildFilterOutputLines(analysis.filterResult),
            error)
        && service.SaveTextLines(
            dir.filePath(QString("%1_KeyPoints.txt").arg(prefix)),
            BuildKeyPointOutputLines(analysis.keyPoints),
            error)
        && service.SaveTextLines(
            dir.filePath(QString("%1_Classified_2mm.txt").arg(prefix)),
            BuildClassifiedOutputLines(analysis.classificationResult),
            error);
}

bool SaveSdkSchemeCompareOutputs(
    const MeasureThenWeldService& service,
    const QString& sdkPointCloudDir,
    const QVector<RobotCalculation::IndexedPoint3D>& originalLaserInput,
    const PointCloudExtractionProcessor::ExtractionResult& extraction,
    const RobotCalculation::LowerWeldFilterParams& params,
    QString& error,
    const MeasureThenWeldService::LogCallback& appendLog)
{
    const QString compareDir = QDir(sdkPointCloudDir).filePath(SDK_SCHEME_COMPARE_DIR_NAME);
    if (!QDir().mkpath(compareDir))
    {
        error = QString("创建SDK方案对比目录失败：%1").arg(compareDir);
        return false;
    }

    const QDir dir(compareDir);
    std::vector<QString> featureSummaryLines;
    featureSummaryLines.reserve(5);
    featureSummaryLines.push_back("方案=特征点方案");
    featureSummaryLines.push_back("输入=sdk_feature_point");
    featureSummaryLines.push_back(QString("关键点=%1").arg(extraction.rawPoints.size()));
    featureSummaryLines.push_back(QString("2mm扩充点=%1").arg(extraction.keyPointExpandedPoints.size()));
    if (!service.SaveTextLines(
            dir.filePath("FeaturePoint_KeyPoints.txt"),
            BuildSdkTrackOutputLines(extraction.rawPoints, "sdk_feature_point"),
            error)
        || !service.SaveTextLines(
            dir.filePath("FeaturePoint_KeyPointExpanded_2mm.txt"),
            BuildSdkTrackOutputLines(extraction.keyPointExpandedPoints, "sdk_feature_point_2mm"),
            error)
        || !service.SaveTextLines(
            dir.filePath("FeaturePoint_Summary.txt"),
            featureSummaryLines,
            error))
    {
        return false;
    }

    const RobotCalculation::LowerWeldFilterParams compareParams =
        BuildSchemeCompareFitParams(params);
    const QVector<RobotCalculation::IndexedPoint3D> baseWeldInput = ToIndexedInput(extraction.points);
    const RobotCalculation::MeasureThenWeldAnalysisResult baseWeldFitAnalysis =
        RobotCalculation::AnalyzeMeasureThenWeldLowerWeldPathDirect(baseWeldInput, compareParams);
    if (!SaveSchemeAnalysisOutputs(
            service,
            compareDir,
            "BaseWeldPointCloudFit",
            "点云+拟合方案",
            "sdk_base_weld",
            baseWeldInput,
            baseWeldFitAnalysis,
            error))
    {
        return false;
    }

    const RobotCalculation::MeasureThenWeldAnalysisResult originalPointCloudFitAnalysis =
        RobotCalculation::AnalyzeMeasureThenWeldLowerWeldPathDirect(originalLaserInput, compareParams);
    if (!SaveSchemeAnalysisOutputs(
            service,
            compareDir,
            "OriginalPointCloudFit",
            "点云拟合方案",
            "original_laser_point",
            originalLaserInput,
            originalPointCloudFitAnalysis,
            error))
    {
        return false;
    }

    if (appendLog)
    {
        appendLog(QString("SDK三方案对比输出目录：%1；特征点=%2，点云+拟合关键点=%3，点云拟合关键点=%4")
            .arg(compareDir)
            .arg(extraction.rawPoints.size())
            .arg(baseWeldFitAnalysis.ok ? baseWeldFitAnalysis.keyPoints.size() : 0)
            .arg(originalPointCloudFitAnalysis.ok ? originalPointCloudFitAnalysis.keyPoints.size() : 0));
    }
    return true;
}

QString FilterResultSummary(
    const QString& phaseName,
    const RobotCalculation::LowerWeldFilterParams& params,
    const RobotCalculation::LowerWeldFilterResult& result,
    const QString& outputPath)
{
    return QString("%1完成：采样主轴=%2，输入点=%3，下层候选点=%4，剔除Z突变=%5，剔除Z连续异常=%6，连续段剔除=%7，拟合段数=%8，输出点=%9，结果=%10")
        .arg(phaseName)
        .arg(SampleAxisName(params.sampleAxis))
        .arg(result.inputPointCount)
        .arg(result.lowerPointCount)
        .arg(result.zJumpRejectedCount)
        .arg(result.zContinuityRejectedCount)
        .arg(result.segmentRejectedCount)
        .arg(result.fitSegmentCount)
        .arg(result.points.size())
        .arg(outputPath);
}

constexpr double MAX_REASONABLE_ROBOT_ANGLE_DEG = 3600.0;

bool IsReasonableRobotAngleDeg(double angleDeg)
{
    return std::isfinite(angleDeg)
        && std::abs(angleDeg) <= MAX_REASONABLE_ROBOT_ANGLE_DEG;
}

double NormalizeAngleNear(double angleDeg, double referenceDeg)
{
    if (!IsReasonableRobotAngleDeg(angleDeg)
        || !IsReasonableRobotAngleDeg(referenceDeg))
    {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const double normalizedAngle = std::remainder(angleDeg, 360.0);
    const double normalizedReference = std::remainder(referenceDeg, 360.0);
    const double delta = std::remainder(normalizedAngle - normalizedReference, 360.0);
    const double result = referenceDeg + delta;
    return std::isfinite(result)
        ? result
        : std::numeric_limits<double>::quiet_NaN();
}

double NormalizeAngleToFanucRange(double angleDeg)
{
    if (!IsReasonableRobotAngleDeg(angleDeg))
    {
        return std::numeric_limits<double>::quiet_NaN();
    }
    double normalized = std::fmod(angleDeg, 360.0);
    if (normalized > 180.0)
    {
        normalized -= 360.0;
    }
    if (normalized <= -180.0)
    {
        normalized += 360.0;
    }
    return normalized;
}



double NormalizeRobotRzOutputRange(double angleDeg)
{
    if (!IsReasonableRobotAngleDeg(angleDeg))
    {
        return std::numeric_limits<double>::quiet_NaN();
    }
    double normalized = std::fmod(angleDeg, 360.0);
    if (normalized > 180.0)
    {
        normalized -= 360.0;
    }
    if (normalized < -180.0)
    {
        normalized += 360.0;
    }
    if (std::abs(normalized - 180.0) <= 1e-9)
    {
        return -180.0;
    }
    return normalized;
}

double RobotRzFromGunDirectionDeg(double gunDirectionFromXDeg)
{
    // STEP RZ convention used by the weld posture generator:
    // gun pointing to robot X- is 0 deg, clockwise rotation is positive, and
    // the 180 deg direction is written as -180 deg.
    return NormalizeRobotRzOutputRange(180.0 - gunDirectionFromXDeg);
}

Eigen::Vector3d GunDirectionVectorFromRobotRz(double rzDeg)
{
    const double directionDeg = 180.0 - NormalizeRobotRzOutputRange(rzDeg);
    const double directionRad = directionDeg * M_PI / 180.0;
    return Eigen::Vector3d(std::cos(directionRad), std::sin(directionRad), 0.0);
}

double GunDirectionDegFromVector(const Eigen::Vector3d& direction)
{
    const double xyLength = std::hypot(direction.x(), direction.y());
    if (xyLength <= 1e-9)
    {
        return 180.0;
    }
    return std::atan2(direction.y(), direction.x()) * 180.0 / M_PI;
}

double ComputeLineNormalRobotRz(
    const Eigen::Vector3d& lineVector,
    double referenceRyDeg,
    double referenceRzDeg,
    double* chosenGunDirectionDeg = nullptr,
    double* rejectedRzDeg = nullptr,
    double* referenceDistanceDeg = nullptr)
{
    (void)referenceRyDeg;

    // RZ is driven by the weld normal, not by blending between corner poses.
    // A 2D seam direction has two perpendicular gun-normal candidates; the
    // current measurement posture selects the branch that keeps the gun closest
    // to how the camera saw this workpiece.
    Eigen::Vector3d tangent(lineVector.x(), lineVector.y(), 0.0);
    const double tangentLength = tangent.norm();
    if (tangentLength <= 1e-9)
    {
        if (chosenGunDirectionDeg != nullptr)
        {
            *chosenGunDirectionDeg = GunDirectionDegFromVector(
                GunDirectionVectorFromRobotRz(referenceRzDeg));
        }
        if (rejectedRzDeg != nullptr)
        {
            *rejectedRzDeg = NormalizeRobotRzOutputRange(referenceRzDeg + 180.0);
        }
        if (referenceDistanceDeg != nullptr)
        {
            *referenceDistanceDeg = 0.0;
        }
        return NormalizeRobotRzOutputRange(referenceRzDeg);
    }

    tangent /= tangentLength;
    const Eigen::Vector3d normalA(-tangent.y(), tangent.x(), 0.0);
    const Eigen::Vector3d normalB = -normalA;
    const Eigen::Vector3d referenceNormal = GunDirectionVectorFromRobotRz(referenceRzDeg);
    const bool useA = normalA.dot(referenceNormal) >= normalB.dot(referenceNormal);
    const Eigen::Vector3d selectedNormal = useA ? normalA : normalB;
    const Eigen::Vector3d rejectedNormal = useA ? normalB : normalA;

    const double selectedGunDirectionDeg = GunDirectionDegFromVector(selectedNormal);
    const double rejectedGunDirectionDeg = GunDirectionDegFromVector(rejectedNormal);
    const double selectedRz = RobotRzFromGunDirectionDeg(selectedGunDirectionDeg);
    const double rejectedRz = RobotRzFromGunDirectionDeg(rejectedGunDirectionDeg);
    if (chosenGunDirectionDeg != nullptr)
    {
        *chosenGunDirectionDeg = NormalizeAngleToFanucRange(selectedGunDirectionDeg);
    }
    if (rejectedRzDeg != nullptr)
    {
        *rejectedRzDeg = NormalizeRobotRzOutputRange(rejectedRz);
    }
    if (referenceDistanceDeg != nullptr)
    {
        *referenceDistanceDeg = std::abs(
            NormalizeAngleNear(selectedRz, referenceRzDeg) - referenceRzDeg);
    }
    return NormalizeRobotRzOutputRange(selectedRz);
}

double AngleDistanceDeg(double angleDeg, double referenceDeg)
{
    return std::abs(NormalizeAngleNear(angleDeg, referenceDeg) - referenceDeg);
}

bool TryParseFiniteDouble(const QString& text, double& value)
{
    bool ok = false;
    value = text.trimmed().toDouble(&ok);
    return ok && std::isfinite(value);
}

int CsvColumnIndex(const QStringList& header, const QString& name)
{
    for (int index = 0; index < header.size(); ++index)
    {
        if (header[index].trimmed() == name)
        {
            return index;
        }
    }
    return -1;
}

MeasurementPoseReference MeasurementPoseReferenceFromRobotPose(
    const T_ROBOT_COORS& pose,
    const QString& source,
    int count)
{
    MeasurementPoseReference reference;
    reference.valid = true;
    reference.count = std::max(1, count);
    reference.source = source;
    reference.x = pose.dX;
    reference.y = pose.dY;
    reference.z = pose.dZ;
    reference.rx = pose.dRX;
    reference.ry = pose.dRY;
    reference.rz = NormalizeRobotRzOutputRange(pose.dRZ);
    return reference;
}

MeasurementPoseReference FirstMeasurementPoseReferenceFromProcessedSamples(
    const std::vector<ProcessedScanCameraSample>& samples,
    const QString& source)
{
    int validCount = 0;
    MeasurementPoseReference reference;
    for (const ProcessedScanCameraSample& sample : samples)
    {
        if (!sample.hasRobotPose || !sample.hasLaserPoint)
        {
            continue;
        }

        ++validCount;
        if (!reference.valid)
        {
            reference = MeasurementPoseReferenceFromRobotPose(sample.robotPose, source, validCount);
        }
    }

    if (reference.valid)
    {
        reference.count = validCount;
    }
    return reference;
}

MeasurementPoseReference LoadMeasurementPoseReferenceFromMatchDebug(
    const QString& matchDebugPath,
    QString* error)
{
    if (error != nullptr)
    {
        error->clear();
    }

    QFile file(matchDebugPath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        if (error != nullptr)
        {
            *error = QString("打开相机-机器人-激光匹配明细失败：%1").arg(matchDebugPath);
        }
        return {};
    }

    QTextStream stream(&file);
    stream.setEncoding(QStringConverter::Utf8);
    if (stream.atEnd())
    {
        if (error != nullptr)
        {
            *error = QString("相机-机器人-激光匹配明细为空：%1").arg(matchDebugPath);
        }
        return {};
    }

    const QStringList header = stream.readLine().split(',');
    const int statusCol = CsvColumnIndex(header, "status");
    const int robotXCol = CsvColumnIndex(header, "robot_x");
    const int robotYCol = CsvColumnIndex(header, "robot_y");
    const int robotZCol = CsvColumnIndex(header, "robot_z");
    const int robotRxCol = CsvColumnIndex(header, "robot_rx");
    const int robotRyCol = CsvColumnIndex(header, "robot_ry");
    const int robotRzCol = CsvColumnIndex(header, "robot_rz");
    const int requiredMaxCol = std::max({
        statusCol,
        robotXCol,
        robotYCol,
        robotZCol,
        robotRxCol,
        robotRyCol,
        robotRzCol
    });
    if (statusCol < 0 || robotXCol < 0 || robotYCol < 0 || robotZCol < 0
        || robotRxCol < 0 || robotRyCol < 0 || robotRzCol < 0)
    {
        if (error != nullptr)
        {
            *error = QString("相机-机器人-激光匹配明细缺少机器人姿态列：%1").arg(matchDebugPath);
        }
        return {};
    }

    int validCount = 0;
    MeasurementPoseReference reference;
    while (!stream.atEnd())
    {
        const QString line = stream.readLine().trimmed();
        if (line.isEmpty())
        {
            continue;
        }

        const QStringList fields = line.split(',');
        if (fields.size() <= requiredMaxCol || fields[statusCol].trimmed() != "laser_ok")
        {
            continue;
        }

        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double rx = 0.0;
        double ry = 0.0;
        double rz = 0.0;
        if (!TryParseFiniteDouble(fields[robotXCol], x)
            || !TryParseFiniteDouble(fields[robotYCol], y)
            || !TryParseFiniteDouble(fields[robotZCol], z)
            || !TryParseFiniteDouble(fields[robotRxCol], rx)
            || !TryParseFiniteDouble(fields[robotRyCol], ry)
            || !TryParseFiniteDouble(fields[robotRzCol], rz))
        {
            continue;
        }

        ++validCount;
        if (!reference.valid)
        {
            reference.valid = true;
            reference.source = matchDebugPath;
            reference.x = x;
            reference.y = y;
            reference.z = z;
            reference.rx = rx;
            reference.ry = ry;
            reference.rz = NormalizeRobotRzOutputRange(rz);
        }
    }

    if (reference.valid)
    {
        reference.count = validCount;
        return reference;
    }

    if (error != nullptr)
    {
        *error = QString("相机-机器人-激光匹配明细中没有可用的 laser_ok 测量姿态：%1").arg(matchDebugPath);
    }
    return {};
}

WeldPosePreset ApplyMeasurementPoseReferenceForCalculation(
    WeldPosePreset preset,
    const MeasurementPoseReference& reference,
    const MeasureThenWeldService::LogCallback& appendLog)
{
    if (!reference.valid)
    {
        if (appendLog)
        {
            appendLog(QString("未读取到点云测量姿态，本次焊接姿态仍使用参数组扫描起终点姿态作为参考RZ=%1 deg。")
                .arg(preset.measureReferenceRz, 0, 'f', 3));
        }
        return preset;
    }

    // 测量姿态只用于焊道法向正反、平台深度轴和RZ参考，不能覆盖参数组里固定的焊接RX/RY。
    preset.measureReferenceRx = reference.rx;
    preset.measureReferenceRy = reference.ry;
    preset.measureReferenceRz = reference.rz;
    if (appendLog)
    {
        appendLog(QString("本次计算使用点云测量姿态作为焊道法向参考，不覆盖固定焊接RX/RY：X=%1, Y=%2, Z=%3, 参考RX=%4, 参考RY=%5, 参考RZ=%6, 固定焊接RX=%7, 固定焊接RY=%8, 有效匹配点=%9, 来源=%10")
            .arg(reference.x, 0, 'f', 3)
            .arg(reference.y, 0, 'f', 3)
            .arg(reference.z, 0, 'f', 3)
            .arg(reference.rx, 0, 'f', 3)
            .arg(reference.ry, 0, 'f', 3)
            .arg(reference.rz, 0, 'f', 3)
            .arg(preset.rx, 0, 'f', 3)
            .arg(preset.ry, 0, 'f', 3)
            .arg(reference.count)
            .arg(reference.source));
    }
    return preset;
}

double PoseDistanceDeg(
    double rxDeg,
    double ryDeg,
    double rzDeg,
    double referenceRxDeg,
    double referenceRyDeg,
    double referenceRzDeg)
{
    const double deltaRx = rxDeg - referenceRxDeg;
    const double deltaRy = ryDeg - referenceRyDeg;
    const double deltaRz = AngleDistanceDeg(rzDeg, referenceRzDeg);
    return std::sqrt(deltaRx * deltaRx + deltaRy * deltaRy + deltaRz * deltaRz);
}

Eigen::Vector3d HorizontalUnitOrZero(const Eigen::Vector3d& vector)
{
    Eigen::Vector3d horizontal = vector;
    horizontal.z() = 0.0;
    const double norm = horizontal.head<2>().norm();
    if (norm <= 1e-9)
    {
        return Eigen::Vector3d::Zero();
    }
    horizontal /= norm;
    return horizontal;
}

bool TryReadIniDouble(COPini& ini, const std::string& key, double& value)
{
    return ini.ReadString(false, key, &value) > 0;
}

void NormalizeSlopeRzClamp(double& minDeg, double& maxDeg)
{
    if (!std::isfinite(minDeg))
    {
        minDeg = -20.0;
    }
    if (!std::isfinite(maxDeg))
    {
        maxDeg = 20.0;
    }
    if (minDeg > maxDeg)
    {
        std::swap(minDeg, maxDeg);
    }
}

bool IsSlopeSegmentKind(const QString& segmentKind)
{
    return segmentKind.compare("rising_edge", Qt::CaseInsensitive) == 0
        || segmentKind.compare("falling_edge", Qt::CaseInsensitive) == 0;
}

bool IsPlatformSegmentKind(const QString& segmentKind)
{
    return segmentKind.compare("low_platform", Qt::CaseInsensitive) == 0
        || segmentKind.compare("high_platform", Qt::CaseInsensitive) == 0;
}

int DefaultPoseCompSlotIndex(const QString& segmentKind)
{
    if (segmentKind.compare("low_platform", Qt::CaseInsensitive) == 0)
    {
        return 0;
    }
    if (segmentKind.compare("rising_edge", Qt::CaseInsensitive) == 0)
    {
        return 1;
    }
    if (segmentKind.compare("high_platform", Qt::CaseInsensitive) == 0)
    {
        return 2;
    }
    if (segmentKind.compare("falling_edge", Qt::CaseInsensitive) == 0)
    {
        return 3;
    }
    return -1;
}

QString DefaultPoseCompSlotKind(int index)
{
    switch (index)
    {
    case 0: return "low_platform";
    case 1: return "rising_edge";
    case 2: return "high_platform";
    case 3: return "falling_edge";
    default: return QString();
    }
}

void InitializeDefaultPoseCompSlots(std::vector<WeldPosePreset::PoseCompSlot>& poseCompCollection)
{
    for (int index = 0; index < static_cast<int>(poseCompCollection.size()); ++index)
    {
        poseCompCollection[index].name = QString("姿态补偿%1").arg(index + 1);
        poseCompCollection[index].segmentKind = DefaultPoseCompSlotKind(index);
    }
}

constexpr int kTransitionScopeArc = 0;
constexpr int kTransitionScopeTransition = 1;
constexpr int kTransitionScopeArcAndTransition = 2;

int NormalizeTransitionApplyScope(int scope)
{
    if (scope < kTransitionScopeArc || scope > kTransitionScopeArcAndTransition)
    {
        return kTransitionScopeArcAndTransition;
    }
    return scope;
}

int NormalizeArcMode(int mode)
{
    return mode >= 0 && mode <= 7 ? mode : 4;
}

QString TransitionApplyScopeText(int scope)
{
    switch (NormalizeTransitionApplyScope(scope))
    {
    case kTransitionScopeArc:
        return QStringLiteral("圆弧");
    case kTransitionScopeTransition:
        return QStringLiteral("过渡");
    default:
        return QStringLiteral("圆弧+过渡");
    }
}

bool TryLoadActiveWeldProcessParam(const QString& robotName, T_WELD_PARA& weldPara, QString* error)
{
    if (robotName.trimmed().isEmpty())
    {
        if (error != nullptr)
        {
            *error = "机器人名称为空，无法读取当前工艺。";
        }
        return false;
    }

    // 复用 WeldProcessFile（多键主数据优先 + 文本块回退 + BindWeldToWeave 摆动绑定），
    // 与工艺页同一份读取逻辑，不再维护第二套文本解析器。
    WeldProcessFile processFile(ToUtf8StdString(robotName.trimmed()));
    if (!processFile.Init())
    {
        if (error != nullptr)
        {
            *error = QString::fromUtf8(processFile.GetLastError().c_str());
        }
        return false;
    }

    const T_WELD_PARA* activePara = processFile.GetUseWeldPara();
    if (activePara == nullptr)
    {
        if (error != nullptr)
        {
            *error = QString("工艺数据没有有效条目：%1").arg(robotName.trimmed());
        }
        return false;
    }

    weldPara = *activePara;  // 含 BindWeldToWeave 已灌入的 tWeaveParam
    std::string validationError;
    if (!WeldProcessValidation::ValidateStoredWeldProcess(weldPara, validationError))
    {
        if (error != nullptr)
        {
            *error = QStringLiteral("当前焊接工艺包含非法字段：")
                + QString::fromUtf8(validationError.c_str());
        }
        return false;
    }
    return true;
}

T_PRECISE_MEASURE_PARAM BuildMeasureWeldParamShell(const QString& robotName)
{
    T_PRECISE_MEASURE_PARAM param;
    const QString normalizedRobotName = robotName.trimmed().isEmpty()
        ? QStringLiteral("RobotA")
        : robotName.trimmed();
    param.sRobotName = ToUtf8StdString(normalizedRobotName);

    QString ensureError;
    RobotDataHelper::EnsureMeasureWeldParamFile(normalizedRobotName, &ensureError);
    const QString iniPath = RobotDataHelper::MeasureWeldParamPath(normalizedRobotName);
    param.sIniFilePath = ToUtf8StdString(iniPath);
    param.sWeldParamFilePath = param.sIniFilePath;

    int groupIndex = 0;
    COPini ini;
    if (ini.SetFileName(param.sIniFilePath))
    {
        std::string groupName;
        ini.SetSectionName("MeasureWeldGroups");
        ini.ReadString(false, "UseGroupNo", &groupIndex);
        groupIndex = std::max(0, groupIndex);
        ini.ReadString(false, ToUtf8StdString(QString("Group%1Name").arg(groupIndex)), groupName);
        if (!groupName.empty())
        {
            param.sParamGroupName = QString::fromStdString(groupName);
        }
    }
    param.nParamGroupIndex = groupIndex;
    param.sSectionName = ToUtf8StdString(RobotDataHelper::MeasureWeldScanSectionName(groupIndex));
    param.sWeldSectionName = ToUtf8StdString(RobotDataHelper::MeasureWeldWeldSectionName(groupIndex));
    if (ini.SetFileName(param.sIniFilePath))
    {
        ini.SetSectionName(param.sWeldSectionName);
        ini.ReadString(false, "FinalWeldTrajectoryStepMm", &param.dFinalWeldTrajectoryStepMm);
    }
    param.dFinalWeldTrajectoryStepMm = NormalizeFinalWeldTrajectorySampleStepMm(param.dFinalWeldTrajectoryStepMm);
    return param;
}

void ApplyActiveWeldProcessToPreset(const T_PRECISE_MEASURE_PARAM& param, WeldPosePreset& preset)
{
    T_WELD_PARA weldPara = {};
    QString loadError;
    if (!TryLoadActiveWeldProcessParam(QString::fromStdString(param.sRobotName), weldPara, &loadError))
    {
        preset.weldProcessLoadError = loadError;
        return;
    }

    preset.weldProcessLoaded = true;
    preset.startArcCurrent = weldPara.dStartArcCurrent;
    preset.startArcVoltage = weldPara.dStartArcVoltage;
    preset.startArcWaitTime = weldPara.dStartWaitTime;
    preset.weldCurrent = weldPara.dTrackCurrent;
    preset.weldVoltage = weldPara.dTrackVoltage;
    preset.weldProcessSpeedMmPerMin = weldPara.WeldVelocity;
    preset.stopArcCurrent = weldPara.dStopArcCurrent;
    preset.stopArcVoltage = weldPara.dStopArcVoltage;
    preset.stopArcWaitTime = weldPara.dStopWaitTime;
    preset.arcMode = NormalizeArcMode(weldPara.nArcMode);
    preset.weaveEnabled = weldPara.nWeaveEnable != 0;
    preset.weaveAppPointwise = weldPara.nWrapConditionNo != 0;  // 摆动来源开关：nWrapConditionNo!=0=上位机自建pointwise
    preset.weavePointsPerCycle = weldPara.nStandWeldDir;  // pointwise每周期点数：复用死字段nStandWeldDir(0=旧工艺，driver端规范化为默认16)
    preset.weldDynamicMode = weldPara.nWeldMethod;  // 动态特性：复用 nWeldMethod 死字段，0=WLin用NULL 非0=用ntdyn0(程序速度)
    preset.trackEnabled = weldPara.nTrackEnable != 0;
    preset.weaveParam = weldPara.tWeaveParam;
    preset.trackParam = weldPara.tTrackParam;
    preset.transitionApplyScope = NormalizeTransitionApplyScope(weldPara.nCornerArcTransitionApplyScope);
    // 焊接姿态/位形 + 圆滑(过渡比例) 来自基础工艺参数；本函数在测量参数段之后调用，故圆滑覆盖测量参数页的默认。
    preset.weldPostureType = (weldPara.nWeldPostureType >= 0 && weldPara.nWeldPostureType <= 3)
        ? weldPara.nWeldPostureType
        : 1;
    if (std::isfinite(weldPara.dWeldOverlapRel) && weldPara.dWeldOverlapRel >= 0.0)
    {
        preset.stepOverlapRel = weldPara.dWeldOverlapRel;
    }

    if (weldPara.nCornerArcTransitionRadiusEnable != 0
        && std::isfinite(weldPara.dCornerArcTransitionRadius))
    {
        preset.cornerArcRadiusMm = std::max(2.0, weldPara.dCornerArcTransitionRadius);
        preset.cornerArcRadiusFromWeldProcess = true;
    }

    if (weldPara.nCornerArcTransitionSpeedEnable != 0
        && std::isfinite(weldPara.dCornerArcTransitionSpeed)
        && weldPara.dCornerArcTransitionSpeed > 0.0)
    {
        preset.transitionSpeedEnabled = true;
        preset.transitionSpeedMmPerMin = weldPara.dCornerArcTransitionSpeed;
    }

    const bool transitionCurrentEnabled = weldPara.nCornerArcTransitionCurrentEnable != 0;
    const bool transitionVoltageEnabled = weldPara.nCornerArcTransitionVoltageEnable != 0;
    preset.transitionCurrentVoltageEnableMismatch = transitionCurrentEnabled != transitionVoltageEnabled;
    if (transitionCurrentEnabled
        && transitionVoltageEnabled
        && std::isfinite(weldPara.dCornerArcTransitionCurrent)
        && std::isfinite(weldPara.dCornerArcTransitionVoltage))
    {
        preset.transitionCurrentVoltageEnabled = true;
        preset.transitionCurrent = weldPara.dCornerArcTransitionCurrent;
        preset.transitionVoltage = weldPara.dCornerArcTransitionVoltage;
    }

    // 工艺里的实际焊道点间距（>0 时优先于测量参数页的 FinalWeldTrajectoryStepMm）。
    if (std::isfinite(weldPara.dFinalWeldTrajectoryStepMm)
        && weldPara.dFinalWeldTrajectoryStepMm > 0.0)
    {
        preset.finalWeldStepFromProcessMm = weldPara.dFinalWeldTrajectoryStepMm;
    }
    std::string validationError;
    if (!WeldProcessValidation::ValidateActualWeldProcess(weldPara, validationError))
    {
        preset.weldProcessSafetyError = QStringLiteral("当前焊接工艺未通过实际焊接安全门禁：")
            + QString::fromUtf8(validationError.c_str());
    }
}

WeldPosePreset LoadWeldPosePreset(const T_PRECISE_MEASURE_PARAM& param)
{
    WeldPosePreset preset;
    preset.rx = param.tStartPos.dRX;
    preset.ry = param.tStartPos.dRY;
    preset.weldRzGainDeg = param.dWeldRzGainDeg;
    preset.useTaughtWeldPose = param.bUseTaughtWeldPose;
    preset.taughtWeldPoseRx = std::isfinite(param.dTaughtWeldPoseRxDeg) ? param.dTaughtWeldPoseRxDeg : preset.rx;
    preset.taughtWeldPoseRy = std::isfinite(param.dTaughtWeldPoseRyDeg) ? param.dTaughtWeldPoseRyDeg : preset.ry;
    preset.taughtWeldPoseRz = std::isfinite(param.dTaughtWeldPoseRzDeg) ? param.dTaughtWeldPoseRzDeg : param.tStartPos.dRZ;
    preset.slopeRzMinDeg = param.dSlopeRzMinDeg;
    preset.slopeRzMaxDeg = param.dSlopeRzMaxDeg;
    NormalizeSlopeRzClamp(preset.slopeRzMinDeg, preset.slopeRzMaxDeg);
    preset.stepOverlapRel = std::isfinite(param.dStepOverlapRel) ? std::max(0.0, param.dStepOverlapRel) : 20.0;
    preset.weldDirection = param.nWeldDirection < 0 ? -1 : 1;
    preset.measureReferenceRx = param.tStartPos.dRX;
    const double startRy = param.tStartPos.dRY;
    const double endRyNearStart = NormalizeAngleNear(param.tEndPos.dRY, startRy);
    preset.measureReferenceRy = (startRy + endRyNearStart) * 0.5;
    const double startRz = NormalizeAngleToFanucRange(param.tStartPos.dRZ);
    const double endRzNearStart = NormalizeAngleNear(param.tEndPos.dRZ, startRz);
    preset.measureReferenceRz = NormalizeAngleToFanucRange((startRz + endRzNearStart) * 0.5);
    preset.weldLineSectionName = param.sWeldSectionName.empty()
        ? QStringLiteral("WeldNormalParam0")
        : QString::fromStdString(param.sWeldSectionName);
    preset.weldLineFilePath = param.sWeldParamFilePath.empty()
        ? QString::fromStdString(param.sIniFilePath)
        : QString::fromStdString(param.sWeldParamFilePath);
    preset.poseCompFilePath = RobotDataHelper::BuildProjectPath(
        QString("Data/%1/WeldPoseCompParam.ini").arg(QString::fromStdString(param.sRobotName)));
    preset.seamCompFilePath = RobotDataHelper::BuildProjectPath(
        QString("Data/%1/WeldSeamCompParam.ini").arg(QString::fromStdString(param.sRobotName)));
    preset.robotParaPath = RobotDataHelper::BuildProjectPath(
        QString("Data/%1/RobotPara.ini").arg(QString::fromStdString(param.sRobotName)));
    preset.poseCompSlots.resize(4);
    InitializeDefaultPoseCompSlots(preset.poseCompSlots);

    if (!ConfigDatabase::HasIniFile(preset.weldLineFilePath))
    {
        goto load_pose_comp;
    }

    {
        COPini ini;
        if (ini.SetFileName(ToUtf8StdString(preset.weldLineFilePath)))
        {
            ini.SetSectionName(ToUtf8StdString(preset.weldLineSectionName));
            double rx = preset.rx;
            double ry = preset.ry;
            double cornerTransitionLeadDistance = preset.cornerTransitionLeadDistance;
            double weldStartSkipDistance = preset.weldStartSkipDistance;
            double weldEndSkipDistance = preset.weldEndSkipDistance;
            double weldRzGainDeg = preset.weldRzGainDeg;
            int useTaughtWeldPose = preset.useTaughtWeldPose ? 1 : 0;
            double taughtWeldPoseRx = preset.taughtWeldPoseRx;
            double taughtWeldPoseRy = preset.taughtWeldPoseRy;
            double taughtWeldPoseRz = preset.taughtWeldPoseRz;
            double slopeRzMinDeg = preset.slopeRzMinDeg;
            double slopeRzMaxDeg = preset.slopeRzMaxDeg;
            double stepOverlapRel = preset.stepOverlapRel;
            const bool hasNormalRx = TryReadIniDouble(ini, "NormalWeldRx", rx);
            const bool hasNormalRy = TryReadIniDouble(ini, "NormalWeldRy", ry);
            ini.ReadString(false, "UseTaughtWeldPose", &useTaughtWeldPose);
            TryReadIniDouble(ini, "TaughtWeldPoseRX", taughtWeldPoseRx);
            TryReadIniDouble(ini, "TaughtWeldPoseRY", taughtWeldPoseRy);
            TryReadIniDouble(ini, "TaughtWeldPoseRZ", taughtWeldPoseRz);
            TryReadIniDouble(ini, "CornerTransitionLeadDis", cornerTransitionLeadDistance);
            TryReadIniDouble(ini, "WeldStartSkipDis", weldStartSkipDistance);
            TryReadIniDouble(ini, "WeldEndSkipDis", weldEndSkipDistance);
            TryReadIniDouble(ini, "WeldRzGainDeg", weldRzGainDeg);
            TryReadIniDouble(ini, "SlopeRzMinDeg", slopeRzMinDeg);
            TryReadIniDouble(ini, "SlopeRzMaxDeg", slopeRzMaxDeg);
            TryReadIniDouble(ini, "StepOverlapRel", stepOverlapRel);
            // 焊接顺序以当前测量焊接参数为准，避免旧 ini 字段覆盖界面选择。
            preset.stepOverlapRel = std::isfinite(stepOverlapRel) ? std::max(0.0, stepOverlapRel) : 20.0;
            if (!(hasNormalRx && hasNormalRy))
            {
                rx = preset.rx;
                ry = preset.ry;
                const bool hasFlatRx = TryReadIniDouble(ini, "FlatWeldRx", rx);
                const bool hasFlatRy = TryReadIniDouble(ini, "FlatWeldRy", ry);
                if (!(hasFlatRx && hasFlatRy))
                {
                    rx = preset.rx;
                    ry = preset.ry;
                }
            }

            preset.rx = rx;
            preset.ry = ry;
            preset.cornerTransitionLeadDistance = std::max(0.0, cornerTransitionLeadDistance);
            preset.weldStartSkipDistance = std::max(0.0, weldStartSkipDistance);
            preset.weldEndSkipDistance = std::max(0.0, weldEndSkipDistance);
            preset.weldRzGainDeg = std::isfinite(weldRzGainDeg) ? weldRzGainDeg : 0.0;
            preset.useTaughtWeldPose = useTaughtWeldPose != 0;
            preset.taughtWeldPoseRx = std::isfinite(taughtWeldPoseRx) ? taughtWeldPoseRx : preset.rx;
            preset.taughtWeldPoseRy = std::isfinite(taughtWeldPoseRy) ? taughtWeldPoseRy : preset.ry;
            preset.taughtWeldPoseRz = std::isfinite(taughtWeldPoseRz) ? taughtWeldPoseRz : preset.measureReferenceRz;
            preset.slopeRzMinDeg = slopeRzMinDeg;
            preset.slopeRzMaxDeg = slopeRzMaxDeg;
            NormalizeSlopeRzClamp(preset.slopeRzMinDeg, preset.slopeRzMaxDeg);
            preset.weldLineFromIni = true;
        }
    }

load_pose_comp:
    ApplyActiveWeldProcessToPreset(param, preset);

    if (ConfigDatabase::HasIniFile(preset.poseCompFilePath))
    {
        COPini poseIni;
        if (poseIni.SetFileName(ToUtf8StdString(preset.poseCompFilePath)))
        {
            int poseCompCount = static_cast<int>(preset.poseCompSlots.size());
            poseIni.SetSectionName("ALLWeldPoseComp");
            poseIni.ReadString(false, "PoseCompCount", &poseCompCount);
            int poseGroupCount = 0;
            const bool hasPoseGroups = poseIni.ReadString(false, POSE_GROUP_COUNT_KEY, &poseGroupCount) > 0;
            int activePoseGroupIndex = 0;
            poseIni.ReadString(false, POSE_ACTIVE_GROUP_INDEX_KEY, &activePoseGroupIndex);
            int poseCompMatchMode = preset.poseCompMatchMode;
            if (poseIni.ReadString(false, POSE_COMP_MATCH_MODE_KEY, &poseCompMatchMode) > 0)
            {
                preset.poseCompMatchMode = NormalizePoseCompMatchMode(poseCompMatchMode);
            }
            TryReadIniDouble(poseIni, "PoseMatchMaxErrorDeg", preset.poseMatchMaxErrorDeg);
            preset.poseMatchMaxErrorDeg = std::max(0.0, preset.poseMatchMaxErrorDeg);

            int sourcePoseOffset = 0;
            int loadedPoseCompCount = std::max(0, poseCompCount);
            if (hasPoseGroups && poseGroupCount > 0)
            {
                activePoseGroupIndex = std::clamp(activePoseGroupIndex, 0, poseGroupCount - 1);
                sourcePoseOffset = activePoseGroupIndex * POSE_COMP_SEGMENT_COUNT;
                loadedPoseCompCount = POSE_COMP_SEGMENT_COUNT;
                poseIni.SetSectionName(ToUtf8StdString(QString("WeldPoseCompGroup%1").arg(activePoseGroupIndex)));
                int groupPoseCompMatchMode = preset.poseCompMatchMode;
                if (poseIni.ReadString(false, POSE_COMP_MATCH_MODE_KEY, &groupPoseCompMatchMode) > 0)
                {
                    preset.poseCompMatchMode = NormalizePoseCompMatchMode(groupPoseCompMatchMode);
                }
            }
            preset.poseCompSlots.assign(loadedPoseCompCount, WeldPosePreset::PoseCompSlot());
            InitializeDefaultPoseCompSlots(preset.poseCompSlots);
            for (int index = 0; index < static_cast<int>(preset.poseCompSlots.size()); ++index)
            {
                WeldPosePreset::PoseCompSlot& slot = preset.poseCompSlots[index];
                poseIni.SetSectionName(ToUtf8StdString(QString("WeldPoseComp%1").arg(sourcePoseOffset + index)));

                std::string slotName;
                std::string segmentKind;
                poseIni.ReadString(false, "Name", slotName);
                poseIni.ReadString(false, "SegmentKind", segmentKind);
                if (!slotName.empty())
                {
                    slot.name = QString::fromStdString(slotName);
                }
                if (!segmentKind.empty())
                {
                    slot.segmentKind = QString::fromStdString(segmentKind);
                }

                double poseRx = preset.rx;
                double poseRy = preset.ry;
                double poseRz = preset.gunToolBaseRz;

                const bool hasPoseRx = TryReadIniDouble(poseIni, "Rx", poseRx);
                const bool hasPoseRy = TryReadIniDouble(poseIni, "Ry", poseRy);
                const bool hasPoseRz = TryReadIniDouble(poseIni, "Rz", poseRz);
                TryReadIniDouble(poseIni, "CompX", slot.compX);
                TryReadIniDouble(poseIni, "CompY", slot.compY);
                TryReadIniDouble(poseIni, "CompZ", slot.compZ);

                slot.poseRx = poseRx;
                slot.poseRy = poseRy;
                slot.poseRz = NormalizeAngleToFanucRange(poseRz);
                slot.hasIniReference = hasPoseRx || hasPoseRy || hasPoseRz;
                slot.generatedReference = false;
                slot.validReference = slot.hasIniReference;
            }
            preset.poseCompFromIni = true;
        }
    }

    {
        WeldSeamCompConfig::Document seamDocument;
        QString seamLoadError;
        if (WeldSeamCompConfig::Load(preset.seamCompFilePath, seamDocument, seamLoadError))
        {
            preset.seamCompWarnings = seamDocument.warnings;
            preset.keepAnchorsOnly = seamDocument.simplifyKeepAnchorsOnly;
            if (!seamDocument.groups.isEmpty())
            {
                const int activeIndex = std::clamp(
                    seamDocument.activeGroupIndex, 0, static_cast<int>(seamDocument.groups.size()) - 1);
                const WeldSeamCompConfig::Values& values = seamDocument.groups[activeIndex].values;
                preset.seamComp.weldZComp = values.weldZComp;
                preset.seamComp.weldGunDirComp = values.weldGunDirComp;
                preset.seamComp.weldSeamDirComp = values.weldSeamDirComp;
            }
            preset.seamCompFromIni = seamDocument.sourceExists;
        }
        else
        {
            preset.seamCompLoadError = seamLoadError;
        }
    }

    if (ConfigDatabase::HasIniFile(preset.robotParaPath))
    {
        COPini robotIni;
        if (robotIni.SetFileName(ToUtf8StdString(preset.robotParaPath)))
        {
            int robotType = preset.robotType;
            robotIni.SetSectionName("BaseParam");
            robotIni.ReadString(false, "RobotType", &robotType);
            preset.robotType = RobotPoseTransform::NormalizeRobotType(robotType);

            robotIni.SetSectionName("Tool");
            double gunToolBaseRz = preset.gunToolBaseRz;
            if (TryReadIniDouble(robotIni, "GunTool_dRZ", gunToolBaseRz))
            {
                preset.gunToolBaseRz = NormalizeAngleToFanucRange(gunToolBaseRz);
            }
        }
    }

    return preset;
}

double ComputeDirectionAngleDeg(const Eigen::Vector3d& startPoint, const Eigen::Vector3d& endPoint, bool* pValid = nullptr)
{
    const double deltaX = endPoint.x() - startPoint.x();
    const double deltaY = endPoint.y() - startPoint.y();
    const double length = std::hypot(deltaX, deltaY);
    const bool valid = length > 1e-6;
    if (pValid != nullptr)
    {
        *pValid = valid;
    }
    if (!valid)
    {
        return 0.0;
    }
    return std::atan2(deltaY, deltaX) * 180.0 / M_PI;
}

QString RobotPoseIndexedSpaceText(int index, const T_ROBOT_COORS& pose, const QString& extra = QString())
{
    QString line = QString("%1 %2 %3 %4 %5 %6 %7 %8 %9 %10")
        .arg(index)
        .arg(pose.dX, 0, 'f', 6)
        .arg(pose.dY, 0, 'f', 6)
        .arg(pose.dZ, 0, 'f', 6)
        .arg(pose.dRX, 0, 'f', 6)
        .arg(pose.dRY, 0, 'f', 6)
        .arg(pose.dRZ, 0, 'f', 6)
        .arg(pose.dBX, 0, 'f', 6)
        .arg(pose.dBY, 0, 'f', 6)
        .arg(pose.dBZ, 0, 'f', 6);
    if (!extra.isEmpty())
    {
        line += " " + extra;
    }
    return line;
}

QString LowerWeldSegmentKindText(
    RobotCalculation::LowerWeldPointType beginType,
    RobotCalculation::LowerWeldPointType endType)
{
    using PointType = RobotCalculation::LowerWeldPointType;
    if ((beginType == PointType::Start && endType == PointType::InnerCorner)
        || (beginType == PointType::InnerCorner && endType == PointType::InnerCorner))
    {
        return "low_platform";
    }
    if ((beginType == PointType::Start && endType == PointType::OuterCorner)
        || (beginType == PointType::OuterCorner && endType == PointType::OuterCorner))
    {
        return "high_platform";
    }
    if (beginType == PointType::InnerCorner && endType == PointType::OuterCorner)
    {
        return "rising_edge";
    }
    if (beginType == PointType::OuterCorner && endType == PointType::InnerCorner)
    {
        return "falling_edge";
    }
    if (endType == PointType::End)
    {
        if (beginType == PointType::OuterCorner)
        {
            return "high_platform";
        }
        if (beginType == PointType::InnerCorner)
        {
            return "low_platform";
        }
    }
    return "segment";
}

Eigen::Vector3d UnitVectorOrZero(const Eigen::Vector3d& vector)
{
    const double norm = vector.norm();
    if (!std::isfinite(norm) || norm <= 1e-9)
    {
        return Eigen::Vector3d::Zero();
    }
    return vector / norm;
}

Eigen::Vector3d MeasurementGunTipAxis(const WeldPosePreset& preset)
{
    Eigen::Vector3d axis = RobotPoseTransform::RotationFromAnglesDeg(
        preset.measureReferenceRx,
        preset.measureReferenceRy,
        preset.measureReferenceRz,
        preset.robotType) * Eigen::Vector3d(-1.0, 0.0, 0.0);
    axis = UnitVectorOrZero(axis);
    if (axis.norm() > 1e-9)
    {
        return axis;
    }
    return UnitVectorOrZero(GunDirectionVectorFromRobotRz(preset.measureReferenceRz));
}

QString SegmentKindFromDepthSide(bool beginIsLowPlatform, bool endIsLowPlatform)
{
    if (beginIsLowPlatform == endIsLowPlatform)
    {
        return beginIsLowPlatform ? "low_platform" : "high_platform";
    }
    return beginIsLowPlatform ? "rising_edge" : "falling_edge";
}

bool AssignSegmentKindsByMeasurementGunDepth(
    const QVector<RobotCalculation::LowerWeldClassifiedPoint>& points,
    const std::vector<int>& keyPointPositions,
    const WeldPosePreset& preset,
    double platformFlatSlopeThreshold,
    std::vector<RobotCalculation::LowerWeldPointType>& keyPointTypes,
    QVector<QString>& segmentKinds,
    const MeasureThenWeldService::LogCallback& appendLog)
{
    using PointType = RobotCalculation::LowerWeldPointType;
    segmentKinds.clear();
    if (keyPointPositions.size() < 2 || keyPointTypes.size() != keyPointPositions.size())
    {
        return false;
    }

    const int firstPosition = keyPointPositions.front();
    const int lastPosition = keyPointPositions.back();
    if (firstPosition < 0 || lastPosition < 0
        || firstPosition >= points.size()
        || lastPosition >= points.size())
    {
        return false;
    }

    const Eigen::Vector3d gunAxis = MeasurementGunTipAxis(preset);
    if (gunAxis.norm() <= 1e-9)
    {
        return false;
    }

    const Eigen::Vector3d travelAxis =
        UnitVectorOrZero(points[lastPosition].point - points[firstPosition].point);
    Eigen::Vector3d depthAxis = gunAxis;
    if (travelAxis.norm() > 1e-9)
    {
        depthAxis -= travelAxis * depthAxis.dot(travelAxis);
    }
    depthAxis = UnitVectorOrZero(depthAxis);
    if (depthAxis.norm() <= 1e-9)
    {
        depthAxis = gunAxis;
    }
    if (depthAxis.dot(gunAxis) < 0.0)
    {
        depthAxis = -depthAxis;
    }

    QVector<double> depths;
    depths.reserve(static_cast<int>(keyPointPositions.size()));
    double minDepth = std::numeric_limits<double>::max();
    double maxDepth = std::numeric_limits<double>::lowest();
    for (int keyPosition : keyPointPositions)
    {
        if (keyPosition < 0 || keyPosition >= points.size())
        {
            return false;
        }
        const double depth = points[keyPosition].point.dot(depthAxis);
        if (!std::isfinite(depth))
        {
            return false;
        }
        depths.push_back(depth);
        minDepth = std::min(minDepth, depth);
        maxDepth = std::max(maxDepth, depth);
    }

    const double depthRange = maxDepth - minDepth;
    if (!std::isfinite(depthRange) || depthRange <= 1e-6)
    {
        return false;
    }
    const double depthMidpoint = (minDepth + maxDepth) * 0.5;

    // 完整周期段已经由几何关键点形成四类拓扑：
    // inner->outer / outer->inner 为两种坡，inner->inner / outer->outer 为两种平台。
    // 只用枪深全局中值逐点二分，会让有轻微沿程漂移的长平台两端跨过中值，误判成一条坡。
    // 因此完整中段先保留四类拓扑的“平台/坡面”属性；首末不完整段没有成对角点，
    // 再用局部深度变化量和斜率判断。深度仅负责确定坡向或平台高低。
    const std::vector<PointType> geometryKeyPointTypes = keyPointTypes;
    const double depthSlopeRatioThreshold =
        std::max(0.01, platformFlatSlopeThreshold);
    struct MeasurementDepthSegmentShape
    {
        bool isSlope = false;
        double depthDelta = 0.0;
    };
    std::vector<MeasurementDepthSegmentShape> segmentShapes(
        keyPointPositions.size() - 1);
    int topologyClassifiedCount = 0;
    int lapStepClassifiedCount = 0;
    int endpointGeometryClassifiedCount = 0;
    for (std::size_t index = 0; index + 1 < keyPointPositions.size(); ++index)
    {
        const Eigen::Vector3d segmentDelta =
            points[keyPointPositions[index + 1]].point
            - points[keyPointPositions[index]].point;
        const double segmentLengthSquared = segmentDelta.squaredNorm();
        const double depthDelta = depths[static_cast<int>(index + 1)]
            - depths[static_cast<int>(index)];
        if (!std::isfinite(segmentLengthSquared)
            || segmentLengthSquared <= 1e-12
            || !std::isfinite(depthDelta))
        {
            return false;
        }

        const PointType geometryBeginType = geometryKeyPointTypes[index];
        const PointType geometryEndType = geometryKeyPointTypes[index + 1];
        const bool isInteriorWaveSegment =
            index > 0
            && index + 1 < keyPointPositions.size() - 1
            && (geometryBeginType == PointType::InnerCorner
                || geometryBeginType == PointType::OuterCorner)
            && (geometryEndType == PointType::InnerCorner
                || geometryEndType == PointType::OuterCorner);
        MeasurementDepthSegmentShape& shape = segmentShapes[index];
        shape.depthDelta = depthDelta;
        const bool isLapStepSegment =
            points[keyPointPositions[index]].source == QStringLiteral("geometry_lap_step")
            && points[keyPointPositions[index + 1]].source == QStringLiteral("geometry_lap_step");
        if (isLapStepSegment)
        {
            // 搭接错位段是平台上的刚性横移，不能作为上/下坡参与相邻平台层级传播。
            // 后续分段逻辑仍会让它无条件继承前一平台的 kind。
            shape.isSlope = false;
            ++lapStepClassifiedCount;
            continue;
        }
        if (isInteriorWaveSegment)
        {
            shape.isSlope = geometryBeginType != geometryEndType;
            ++topologyClassifiedCount;
            continue;
        }

        // 首末段可能从平台或坡面中途截断，不能根据 Start/End 类型硬判。
        // 去掉枪深分量后，用沿焊道横向跨度计算局部斜率，与点云平台判据保持同一量纲。
        const double lateralSpanSquared =
            std::max(0.0, segmentLengthSquared - depthDelta * depthDelta);
        const double lateralSpan = std::sqrt(lateralSpanSquared);
        const double depthSlopeRatio = lateralSpan > 1e-9
            ? std::abs(depthDelta) / lateralSpan
            : (std::abs(depthDelta) > 1e-9
                ? std::numeric_limits<double>::infinity()
                : 0.0);
        shape.isSlope = depthSlopeRatio >= depthSlopeRatioThreshold;
        ++endpointGeometryClassifiedCount;
    }

    std::vector<int> lowPlatformVotes(keyPointPositions.size(), 0);
    std::vector<int> highPlatformVotes(keyPointPositions.size(), 0);
    int neighborSlopePlatformCount = 0;
    int midpointFallbackPlatformCount = 0;
    int midpointCrossingPlatformCount = 0;
    segmentKinds.reserve(static_cast<int>(keyPointPositions.size()) - 1);
    for (std::size_t index = 0; index + 1 < keyPointPositions.size(); ++index)
    {
        const MeasurementDepthSegmentShape& shape = segmentShapes[index];
        bool beginIsLowPlatform = false;
        bool endIsLowPlatform = false;
        if (shape.isSlope)
        {
            // 深度越大越靠远侧（低平台）。低->高为 rising，高->低为 falling。
            beginIsLowPlatform = shape.depthDelta < 0.0;
            endIsLowPlatform = !beginIsLowPlatform;
        }
        else
        {
            // 平台优先继承相邻坡的端点层级，避免长程漂移令整段均值越过全局中值：
            // rising 前低后高，falling 前高后低。无相邻坡或两侧冲突时才回退段均值。
            int neighborLowVotes = 0;
            int neighborHighVotes = 0;
            if (index > 0 && segmentShapes[index - 1].isSlope)
            {
                const bool leftSlopeEndIsLow =
                    segmentShapes[index - 1].depthDelta >= 0.0;
                leftSlopeEndIsLow ? ++neighborLowVotes : ++neighborHighVotes;
            }
            if (index + 1 < segmentShapes.size() && segmentShapes[index + 1].isSlope)
            {
                const bool rightSlopeBeginIsLow =
                    segmentShapes[index + 1].depthDelta < 0.0;
                rightSlopeBeginIsLow ? ++neighborLowVotes : ++neighborHighVotes;
            }

            bool segmentIsLowPlatform = false;
            if (neighborLowVotes != neighborHighVotes)
            {
                segmentIsLowPlatform = neighborLowVotes > neighborHighVotes;
                ++neighborSlopePlatformCount;
            }
            else
            {
                const double segmentMiddleDepth =
                    (depths[static_cast<int>(index)]
                        + depths[static_cast<int>(index + 1)]) * 0.5;
                segmentIsLowPlatform = segmentMiddleDepth >= depthMidpoint;
                ++midpointFallbackPlatformCount;
            }
            beginIsLowPlatform = segmentIsLowPlatform;
            endIsLowPlatform = segmentIsLowPlatform;
            const bool rawBeginIsLow =
                depths[static_cast<int>(index)] >= depthMidpoint;
            const bool rawEndIsLow =
                depths[static_cast<int>(index + 1)] >= depthMidpoint;
            if (rawBeginIsLow != rawEndIsLow)
            {
                ++midpointCrossingPlatformCount;
            }
        }

        if (beginIsLowPlatform)
        {
            ++lowPlatformVotes[index];
        }
        else
        {
            ++highPlatformVotes[index];
        }
        if (endIsLowPlatform)
        {
            ++lowPlatformVotes[index + 1];
        }
        else
        {
            ++highPlatformVotes[index + 1];
        }
        segmentKinds.push_back(SegmentKindFromDepthSide(
            beginIsLowPlatform,
            endIsLowPlatform));
    }

    for (std::size_t index = 0; index < keyPointPositions.size(); ++index)
    {
        if (index == 0)
        {
            keyPointTypes[index] = PointType::Start;
        }
        else if (index + 1 == keyPointPositions.size())
        {
            keyPointTypes[index] = PointType::End;
        }
        else
        {
            const bool isLowPlatform =
                lowPlatformVotes[index] == highPlatformVotes[index]
                    ? depths[static_cast<int>(index)] >= depthMidpoint
                    : lowPlatformVotes[index] > highPlatformVotes[index];
            keyPointTypes[index] =
                isLowPlatform ? PointType::InnerCorner : PointType::OuterCorner;
        }
    }

    if (appendLog)
    {
        appendLog(QString("按测量枪姿重判焊道段属性：完整周期段按四类角点拓扑判平台/坡面=%1段，搭接横移段=%2段，"
            "首末不完整段按局部深度几何判定=%3段（枪深/横向跨度阈值=%4）；"
            "平台层级由相邻坡传播=%5段、中值回退=%6段，平台两端跨全局中值但仍保持平台=%7段；"
            "枪尖方向=(%8,%9,%10)，深度轴=(%11,%12,%13)，深度范围=%14 mm，远侧=低平台，近侧=高平台。")
            .arg(topologyClassifiedCount)
            .arg(lapStepClassifiedCount)
            .arg(endpointGeometryClassifiedCount)
            .arg(depthSlopeRatioThreshold, 0, 'f', 3)
            .arg(neighborSlopePlatformCount)
            .arg(midpointFallbackPlatformCount)
            .arg(midpointCrossingPlatformCount)
            .arg(gunAxis.x(), 0, 'f', 3)
            .arg(gunAxis.y(), 0, 'f', 3)
            .arg(gunAxis.z(), 0, 'f', 3)
            .arg(depthAxis.x(), 0, 'f', 3)
            .arg(depthAxis.y(), 0, 'f', 3)
            .arg(depthAxis.z(), 0, 'f', 3)
            .arg(depthRange, 0, 'f', 3));
    }

    return true;
}


struct WeldPoseFileRecord
{
    int weldIndex = 0;
    int rawIndex = 0;
    Eigen::Vector3d point = Eigen::Vector3d::Zero();
    double rx = 0.0;
    double ry = 0.0;
    double rz = 0.0;
    double bx = 0.0;
    double by = 0.0;
    double bz = 0.0;
    QString pointType;
    QString segmentKind;
    bool isLapStep = false;  // 搭接错位台阶端点：下游圆角/平滑/抽样/补偿据此豁免，保住垂直 X 台阶
};

QString BuildWeldPoseFileRecordLine(const WeldPoseFileRecord& record)
{
    return QString("%1 %2 %3 %4 %5 %6 %7 %8 %9 %10 %11 %12 %13 %14")
        .arg(record.weldIndex)
        .arg(record.rawIndex)
        .arg(record.point.x(), 0, 'f', 6)
        .arg(record.point.y(), 0, 'f', 6)
        .arg(record.point.z(), 0, 'f', 6)
        .arg(record.rx, 0, 'f', 6)
        .arg(record.ry, 0, 'f', 6)
        .arg(record.rz, 0, 'f', 6)
        .arg(record.bx, 0, 'f', 6)
        .arg(record.by, 0, 'f', 6)
        .arg(record.bz, 0, 'f', 6)
        .arg(record.pointType)
        .arg(record.segmentKind)
        .arg(record.isLapStep ? 1 : 0);
}

bool TryParseWeldPoseFileRecord(const QString& line, WeldPoseFileRecord& record)
{
    QString normalizedLine = line;
    normalizedLine.remove('"');
    static const QRegularExpression kWhitespaceRe("\\s+");  // 提为静态：避免每行重新编译正则
    const QStringList parts = normalizedLine.contains(',')
        ? normalizedLine.split(',', Qt::SkipEmptyParts)
        : normalizedLine.split(kWhitespaceRe, Qt::SkipEmptyParts);
    if (parts.size() < 13)
    {
        return false;
    }

    bool weldIndexOk = false;
    bool rawIndexOk = false;
    bool xOk = false;
    bool yOk = false;
    bool zOk = false;
    bool rxOk = false;
    bool ryOk = false;
    bool rzOk = false;
    bool bxOk = false;
    bool byOk = false;
    bool bzOk = false;

    record.weldIndex = parts[0].trimmed().toInt(&weldIndexOk);
    record.rawIndex = parts[1].trimmed().toInt(&rawIndexOk);
    const double x = parts[2].trimmed().toDouble(&xOk);
    const double y = parts[3].trimmed().toDouble(&yOk);
    const double z = parts[4].trimmed().toDouble(&zOk);
    record.rx = parts[5].trimmed().toDouble(&rxOk);
    record.ry = parts[6].trimmed().toDouble(&ryOk);
    record.rz = parts[7].trimmed().toDouble(&rzOk);
    record.bx = parts[8].trimmed().toDouble(&bxOk);
    record.by = parts[9].trimmed().toDouble(&byOk);
    record.bz = parts[10].trimmed().toDouble(&bzOk);
    record.pointType = parts[11].trimmed();
    record.segmentKind = parts[12].trimmed();
    record.isLapStep = (parts.size() >= 14 && parts[13].trimmed().toInt() != 0);  // 旧文件无此列→false(向后兼容)

    if (!(weldIndexOk && rawIndexOk && xOk && yOk && zOk
        && rxOk && ryOk && rzOk && bxOk && byOk && bzOk))
    {
        return false;
    }

    if (!(std::isfinite(x) && std::isfinite(y) && std::isfinite(z)
        && std::isfinite(record.rx) && std::isfinite(record.ry) && std::isfinite(record.rz)
        && std::isfinite(record.bx) && std::isfinite(record.by) && std::isfinite(record.bz)))
    {
        return false;
    }
    if (!IsReasonableRobotAngleDeg(record.rx)
        || !IsReasonableRobotAngleDeg(record.ry)
        || !IsReasonableRobotAngleDeg(record.rz))
    {
        return false;
    }

    record.point = Eigen::Vector3d(x, y, z);
    return true;
}

void ApplyWeldDirectionToExecutionRecords(const WeldPosePreset& preset, QVector<WeldPoseFileRecord>& records)
{
    if (preset.weldDirection >= 0 || records.size() < 2)
    {
        return;
    }

    std::reverse(records.begin(), records.end());
    for (int index = 0; index < records.size(); ++index)
    {
        records[index].weldIndex = index + 1;
    }
}

QString WeldDirectionText(const WeldPosePreset& preset)
{
    return preset.weldDirection < 0
        ? QStringLiteral("终点到起点")
        : QStringLiteral("起点到终点");
}

bool LoadWeldPoseFileRecords(
    const QString& filePath,
    QVector<WeldPoseFileRecord>& records,
    QString& error,
    QString* loadedSha256 = nullptr,
    qint64* loadedSize = nullptr,
    const MeasureThenWeldService::StopRequestedCallback& stopRequested =
        MeasureThenWeldService::StopRequestedCallback())
{
    records.clear();
    error.clear();
    if (loadedSha256 != nullptr)
    {
        loadedSha256->clear();
    }
    if (loadedSize != nullptr)
    {
        *loadedSize = -1;
    }

    const QFileInfo poseInfo(QDir::fromNativeSeparators(filePath));
    if (!poseInfo.isFile()
        || poseInfo.isSymLink()
#ifdef Q_OS_WIN
        || poseInfo.isJunction()
#endif
        || poseInfo.size() <= 0
        || poseInfo.size() > PointCloudProofIntegrity::MaximumWeldPoseBytes)
    {
        error = QString("焊道姿态必须是 0..%1 字节范围内的普通文件：%2")
            .arg(PointCloudProofIntegrity::MaximumWeldPoseBytes)
            .arg(poseInfo.absoluteFilePath());
        return false;
    }
    QFile file(poseInfo.absoluteFilePath());
    if (!file.open(QIODevice::ReadOnly))
    {
        error = "打开焊道姿态文件失败：" + QFileInfo(filePath).absoluteFilePath();
        return false;
    }
    const qint64 expectedBytes = file.size();
    QCryptographicHash payloadHash(QCryptographicHash::Sha256);
    qint64 readBytes = 0;
    qint64 lineNumber = 0;
    while (!file.atEnd())
    {
        if (stopRequested && stopRequested())
        {
            error = QStringLiteral("焊道姿态流式读取已取消：") + poseInfo.absoluteFilePath();
            return false;
        }
        const QByteArray rawLine = file.readLine(
            PointCloudProofIntegrity::MaximumWeldPoseLineBytes + 1);
        if (rawLine.isEmpty() && file.error() != QFileDevice::NoError)
        {
            error = QStringLiteral("流式读取焊道姿态失败：") + poseInfo.absoluteFilePath();
            return false;
        }
        if (rawLine.size() > PointCloudProofIntegrity::MaximumWeldPoseLineBytes
            || (!rawLine.endsWith('\n') && !file.atEnd()))
        {
            error = QString("焊道姿态单行超过 %1 字节硬上限：%2")
                .arg(PointCloudProofIntegrity::MaximumWeldPoseLineBytes)
                .arg(poseInfo.absoluteFilePath());
            return false;
        }
        if (readBytes > PointCloudProofIntegrity::MaximumWeldPoseBytes - rawLine.size())
        {
            error = QStringLiteral("焊道姿态流式读取超过总字节硬上限。");
            return false;
        }
        readBytes += rawLine.size();
        payloadHash.addData(rawLine);
        ++lineNumber;
        if (lineNumber > PointCloudProofIntegrity::MaximumWeldPoseLines)
        {
            error = QString("焊道姿态超过 %1 行硬上限：%2")
                .arg(PointCloudProofIntegrity::MaximumWeldPoseLines)
                .arg(poseInfo.absoluteFilePath());
            return false;
        }
        QStringDecoder decoder(QStringDecoder::Utf8);
        const QString decodedLine = decoder(rawLine);
        const QString line = decodedLine.trimmed();
        if (decoder.hasError())
        {
            error = QString("焊道姿态第 %1 行不是严格 UTF-8：%2")
                .arg(lineNumber).arg(poseInfo.absoluteFilePath());
            return false;
        }
        if (line.isEmpty() || line.startsWith('#'))
        {
            continue;
        }

        WeldPoseFileRecord record;
        if (!TryParseWeldPoseFileRecord(line, record))
        {
            if (records.isEmpty())
            {
                continue;
            }

            error = QString("解析焊道姿态文件失败，第 %1 行格式无效：%2")
                .arg(lineNumber)
                .arg(line);
            return false;
        }

        records.push_back(record);
    }
    if (file.error() != QFileDevice::NoError
        || readBytes != expectedBytes
        || file.size() != expectedBytes)
    {
        error = "读取焊道姿态流式快照失败或读取期间文件变化："
            + QFileInfo(filePath).absoluteFilePath();
        return false;
    }

    if (records.size() < 2)
    {
        error = QString("焊道姿态文件有效点不足（%1，至少需要 2 点）：%2")
            .arg(records.size())
            .arg(QFileInfo(filePath).absoluteFilePath());
        return false;
    }

    if (loadedSha256 != nullptr)
    {
        *loadedSha256 = QString::fromLatin1(payloadHash.result().toHex()).toLower();
    }
    if (loadedSize != nullptr)
    {
        *loadedSize = readBytes;
    }

    return true;
}

bool IsKnownWeldPointType(const QString& value)
{
    static const QSet<QString> known = {
        QStringLiteral("start"),
        QStringLiteral("end"),
        QStringLiteral("normal"),
        QStringLiteral("normal_arc"),
        QStringLiteral("inner_corner"),
        QStringLiteral("inner_corner_arc"),
        QStringLiteral("outer_corner"),
        QStringLiteral("outer_corner_arc")
    };
    return known.contains(value.trimmed().toLower());
}

QString WeldSegmentBaseKind(QString value)
{
    value = value.trimmed().toLower();
    constexpr auto transitionSuffix = "_transition";
    constexpr auto arcSuffix = "_arc";
    if (value.endsWith(QString::fromLatin1(transitionSuffix)))
    {
        value.chop(static_cast<int>(std::strlen(transitionSuffix)));
    }
    else if (value.endsWith(QString::fromLatin1(arcSuffix)))
    {
        value.chop(static_cast<int>(std::strlen(arcSuffix)));
    }
    return value;
}

bool IsKnownWeldSegmentKind(const QString& value)
{
    const QString normalized = value.trimmed().toLower();
    const QString base = WeldSegmentBaseKind(normalized);
    static const QSet<QString> knownBases = {
        QStringLiteral("low_platform"),
        QStringLiteral("high_platform"),
        QStringLiteral("rising_edge"),
        QStringLiteral("falling_edge"),
        QStringLiteral("tail"),
        QStringLiteral("segment")
    };
    if (!knownBases.contains(base))
    {
        return false;
    }
    return normalized == base
        || normalized == base + QStringLiteral("_transition")
        || normalized == base + QStringLiteral("_arc");
}

double ControllerEulerDistanceDeg(
    const WeldPoseFileRecord& left,
    const WeldPoseFileRecord& right)
{
    const double dRx = NormalizeAngleNear(right.rx, left.rx) - left.rx;
    const double dRy = NormalizeAngleNear(right.ry, left.ry) - left.ry;
    const double dRz = NormalizeAngleNear(right.rz, left.rz) - left.rz;
    const double distance = std::sqrt(dRx * dRx + dRy * dRy + dRz * dRz);
    return std::isfinite(distance)
        ? distance
        : std::numeric_limits<double>::infinity();
}

double PhysicalOrientationDistanceDeg(
    const WeldPoseFileRecord& left,
    const WeldPoseFileRecord& right,
    int robotType)
{
    const Eigen::Matrix3d leftRotation = RobotPoseTransform::RotationFromAnglesDeg(
        left.rx, left.ry, left.rz, robotType);
    const Eigen::Matrix3d rightRotation = RobotPoseTransform::RotationFromAnglesDeg(
        right.rx, right.ry, right.rz, robotType);
    if (!leftRotation.allFinite() || !rightRotation.allFinite())
    {
        return std::numeric_limits<double>::infinity();
    }
    const Eigen::Matrix3d relative = leftRotation.transpose() * rightRotation;
    const double cosine = std::clamp((relative.trace() - 1.0) * 0.5, -1.0, 1.0);
    const double distance = std::acos(cosine) * 180.0 / RobotPoseTransform::kPi;
    return std::isfinite(distance)
        ? distance
        : std::numeric_limits<double>::infinity();
}

bool ValidateFinalWeldPoseArtifact(
    const QString& filePath,
    const QString& sourcePosePath,
    const QString& expectedGeneratedSha256,
    qint64 expectedGeneratedSize,
    int robotType,
    double declaredStartSkipMm,
    double declaredEndSkipMm,
    const PointCloudProcessingConfig::Settings& settings,
    const RobotCalculation::MeasureThenWeldAnalysisResult::PointCloudQualityReport& qualityReport,
    QString& validatedSourceSha256,
    qint64& validatedSourceSize,
    QString& validatedSha256,
    qint64& validatedSize,
    QString& error,
    const MeasureThenWeldService::StopRequestedCallback& stopRequested =
        MeasureThenWeldService::StopRequestedCallback())
{
    validatedSourceSha256.clear();
    validatedSourceSize = -1;
    validatedSha256.clear();
    validatedSize = -1;
    if (!std::isfinite(declaredStartSkipMm) || declaredStartSkipMm < 0.0
        || !std::isfinite(declaredEndSkipMm) || declaredEndSkipMm < 0.0)
    {
        error = QStringLiteral("最终焊道验证收到的起终点声明裁剪距离无效。");
        return false;
    }
    QVector<WeldPoseFileRecord> sourceRecords;
    if (!LoadWeldPoseFileRecords(
            sourcePosePath,
            sourceRecords,
            error,
            &validatedSourceSha256,
            &validatedSourceSize,
            stopRequested))
    {
        error = QStringLiteral("补偿前焊接姿态回读失败：") + error;
        return false;
    }
    QVector<WeldPoseFileRecord> records;
    if (!LoadWeldPoseFileRecords(
            filePath,
            records,
            error,
            &validatedSha256,
            &validatedSize,
            stopRequested))
    {
        return false;
    }
    if (!IsSha256Text(expectedGeneratedSha256)
        || expectedGeneratedSize <= 0
        || validatedSha256.compare(expectedGeneratedSha256, Qt::CaseInsensitive) != 0
        || validatedSize != expectedGeneratedSize)
    {
        error = QStringLiteral(
            "最终焊道与补偿算法刚生成的完整字节快照不一致，标签、坐标或顺序可能已变化。");
        return false;
    }
    if (!qualityReport.evaluated
        || !std::isfinite(qualityReport.projectedSpanMm)
        || qualityReport.projectedSpanMm <= 0.0)
    {
        error = QStringLiteral("最终焊接姿态缺少本次已完成评估的有效点云跨度报告。");
        return false;
    }
    if (records.size() < settings.validationMinOutputPointCount)
    {
        error = QString("最终焊接姿态回读点数过少：当前 %1，要求至少 %2。")
            .arg(records.size())
            .arg(settings.validationMinOutputPointCount);
        return false;
    }
    double totalLengthMm = 0.0;
    double sourceLengthMm = 0.0;
    double maxStepMm = 0.0;
    double maxControllerEulerStepDeg = 0.0;
    double maxPhysicalOrientationStepDeg = 0.0;
    double matchedFinalArcMm = 0.0;
    double maxSourceDisplacementMm = 0.0;
    double maxSourceControllerEulerDeltaDeg = 0.0;
    double maxSourcePhysicalOrientationDeltaDeg = 0.0;
    QVector<double> sourceArcMm(sourceRecords.size(), 0.0);
    QHash<int, QVector<int>> sourceIndexesByRawIndex;
    QHash<QString, QVector<int>> sourceIndexesBySegmentBase;
    QVector<int> sourceCornerIndexes;
    QVector<int> sourceLapIndexes;
    for (int index = 0; index < sourceRecords.size(); ++index)
    {
        const WeldPoseFileRecord& source = sourceRecords[index];
        if (!IsKnownWeldPointType(source.pointType)
            || !IsKnownWeldSegmentKind(source.segmentKind)
            || !IsReasonableRobotAngleDeg(source.rx)
            || !IsReasonableRobotAngleDeg(source.ry)
            || !IsReasonableRobotAngleDeg(source.rz))
        {
            error = QString("补偿前焊接姿态第 %1 条标签或角度超出允许语义。").arg(index + 1);
            return false;
        }
        sourceIndexesByRawIndex[sourceRecords[index].rawIndex].push_back(index);
        sourceIndexesBySegmentBase[WeldSegmentBaseKind(source.segmentKind)].push_back(index);
        if (source.pointType.contains(QStringLiteral("corner"), Qt::CaseInsensitive))
        {
            sourceCornerIndexes.push_back(index);
        }
        if (source.isLapStep)
        {
            sourceLapIndexes.push_back(index);
        }
    }
    for (int index = 1; index < sourceRecords.size(); ++index)
    {
        const double sourceStepMm =
            (sourceRecords[index].point - sourceRecords[index - 1].point).norm();
        if (!std::isfinite(sourceStepMm))
        {
            error = QString("补偿前焊接姿态第 %1 段长度不是有限值。").arg(index);
            return false;
        }
        sourceLengthMm += sourceStepMm;
        sourceArcMm[index] = sourceLengthMm;
    }
    const double expectedSourceStartArcMm = std::min(declaredStartSkipMm, sourceLengthMm);
    const double expectedSourceEndArcMm = std::max(
        expectedSourceStartArcMm,
        sourceLengthMm - std::min(declaredEndSkipMm, sourceLengthMm));
    const double expectedRetainedSourceLengthMm =
        expectedSourceEndArcMm - expectedSourceStartArcMm;
    if (expectedRetainedSourceLengthMm <= 0.0)
    {
        error = QStringLiteral("声明的起终点裁剪已覆盖全部补偿前轨迹，禁止生成最终焊道。");
        return false;
    }

    const auto hasNearbySource = [&](const QVector<int>& sourceIndexes, const Eigen::Vector3d& point)
        {
            for (int sourceIndex : sourceIndexes)
            {
                if ((point - sourceRecords[sourceIndex].point).norm()
                    <= FINAL_MAX_SOURCE_DISPLACEMENT_MM)
                {
                    return true;
                }
            }
            return false;
        };
    const auto hasNearbySourcePose = [&](const QVector<int>& sourceIndexes,
                                         const WeldPoseFileRecord& record)
        {
            for (int sourceIndex : sourceIndexes)
            {
                const WeldPoseFileRecord& source = sourceRecords[sourceIndex];
                if ((record.point - source.point).norm() <= FINAL_MAX_SOURCE_DISPLACEMENT_MM
                    && ControllerEulerDistanceDeg(source, record)
                        <= FINAL_MAX_SOURCE_CONTROLLER_EULER_DELTA_DEG
                    && PhysicalOrientationDistanceDeg(source, record, robotType)
                        <= FINAL_MAX_SOURCE_PHYSICAL_ORIENTATION_DELTA_DEG)
                {
                    return true;
                }
            }
            return false;
        };

    QVector<int> matchedSourceIndexes(records.size(), -1);
    QSet<int> uniqueMatchedSourceIndexes;
    int lastMatchedSourceIndex = 0;
    for (int index = 0; index < records.size(); ++index)
    {
        const WeldPoseFileRecord& record = records[index];
        if (records[index].weldIndex != index + 1)
        {
            error = QString("最终焊接姿态索引不连续：第 %1 条记录 weld_index=%2，应为 %3。")
                .arg(index + 1)
                .arg(records[index].weldIndex)
                .arg(index + 1);
            return false;
        }
        if (!IsKnownWeldPointType(record.pointType)
            || !IsKnownWeldSegmentKind(record.segmentKind)
            || !IsReasonableRobotAngleDeg(record.rx)
            || !IsReasonableRobotAngleDeg(record.ry)
            || !IsReasonableRobotAngleDeg(record.rz))
        {
            error = QString("最终焊接姿态第 %1 条标签或角度超出允许语义。").arg(index + 1);
            return false;
        }
        const QString pointType = record.pointType.trimmed().toLower();
        if ((pointType == QStringLiteral("start") && index != 0)
            || (pointType == QStringLiteral("end") && index + 1 != records.size()))
        {
            error = QString("最终焊接姿态第 %1 条 start/end 标签位置无效。").arg(index + 1);
            return false;
        }
        const QString segmentBase = WeldSegmentBaseKind(record.segmentKind);
        if (!hasNearbySourcePose(sourceIndexesBySegmentBase.value(segmentBase), record))
        {
            error = QString("最终焊接姿态第 %1 条在补偿前同段语义中没有同时满足 %2mm / %3deg控制器欧拉 / %4deg物理姿态硬门限的候选。")
                .arg(index + 1)
                .arg(FINAL_MAX_SOURCE_DISPLACEMENT_MM, 0, 'f', 1)
                .arg(FINAL_MAX_SOURCE_CONTROLLER_EULER_DELTA_DEG, 0, 'f', 1)
                .arg(FINAL_MAX_SOURCE_PHYSICAL_ORIENTATION_DELTA_DEG, 0, 'f', 1);
            return false;
        }
        if (pointType.contains(QStringLiteral("corner"))
            && !hasNearbySource(sourceCornerIndexes, record.point))
        {
            error = QString("最终焊接姿态第 %1 条拐点标签在补偿前拐点附近没有对应语义。")
                .arg(index + 1);
            return false;
        }
        if (record.isLapStep && !hasNearbySource(sourceLapIndexes, record.point))
        {
            error = QString("最终焊接姿态第 %1 条搭接台阶标签在补偿前台阶附近没有对应语义。")
                .arg(index + 1);
            return false;
        }

        const QVector<int> sourceIndexes = sourceIndexesByRawIndex.value(records[index].rawIndex);
        int bestSourceIndex = -1;
        double bestSourceDistanceMm = std::numeric_limits<double>::infinity();
        double bestSourceControllerEulerDeltaDeg = std::numeric_limits<double>::infinity();
        double bestSourcePhysicalOrientationDeltaDeg = std::numeric_limits<double>::infinity();
        for (int sourceIndex : sourceIndexes)
        {
            if (sourceIndex < lastMatchedSourceIndex)
            {
                continue;
            }
            const WeldPoseFileRecord& source = sourceRecords[sourceIndex];
            const double distanceMm = (records[index].point - source.point).norm();
            const double controllerEulerDeltaDeg = ControllerEulerDistanceDeg(source, record);
            const double physicalOrientationDeltaDeg = PhysicalOrientationDistanceDeg(
                source, record, robotType);
            if (distanceMm <= FINAL_MAX_SOURCE_DISPLACEMENT_MM
                && controllerEulerDeltaDeg <= FINAL_MAX_SOURCE_CONTROLLER_EULER_DELTA_DEG
                && physicalOrientationDeltaDeg <= FINAL_MAX_SOURCE_PHYSICAL_ORIENTATION_DELTA_DEG
                && (distanceMm < bestSourceDistanceMm
                    || (std::abs(distanceMm - bestSourceDistanceMm) <= 1e-9
                        && physicalOrientationDeltaDeg < bestSourcePhysicalOrientationDeltaDeg)))
            {
                bestSourceIndex = sourceIndex;
                bestSourceDistanceMm = distanceMm;
                bestSourceControllerEulerDeltaDeg = controllerEulerDeltaDeg;
                bestSourcePhysicalOrientationDeltaDeg = physicalOrientationDeltaDeg;
            }
        }
        if (bestSourceIndex >= 0)
        {
            matchedSourceIndexes[index] = bestSourceIndex;
            lastMatchedSourceIndex = bestSourceIndex;
            uniqueMatchedSourceIndexes.insert(bestSourceIndex);
            maxSourceDisplacementMm = std::max(maxSourceDisplacementMm, bestSourceDistanceMm);
            maxSourceControllerEulerDeltaDeg = std::max(
                maxSourceControllerEulerDeltaDeg, bestSourceControllerEulerDeltaDeg);
            maxSourcePhysicalOrientationDeltaDeg = std::max(
                maxSourcePhysicalOrientationDeltaDeg, bestSourcePhysicalOrientationDeltaDeg);
        }
        if (index > 0)
        {
            const double stepMm = (records[index].point - records[index - 1].point).norm();
            if (!std::isfinite(stepMm))
            {
                error = QString("最终焊接姿态第 %1 段长度不是有限值。").arg(index);
                return false;
            }
            totalLengthMm += stepMm;
            maxStepMm = std::max(maxStepMm, stepMm);
            maxControllerEulerStepDeg = std::max(
                maxControllerEulerStepDeg,
                ControllerEulerDistanceDeg(records[index - 1], record));
            maxPhysicalOrientationStepDeg = std::max(
                maxPhysicalOrientationStepDeg,
                PhysicalOrientationDistanceDeg(records[index - 1], record, robotType));
            if (matchedSourceIndexes[index - 1] >= 0 && matchedSourceIndexes[index] >= 0)
            {
                matchedFinalArcMm += stepMm;
            }
        }
    }

    if (maxStepMm > FINAL_MAX_POSITION_STEP_MM)
    {
        error = QString("最终焊接姿态最大点距 %1 mm，超过结构硬门限 %2 mm。")
            .arg(maxStepMm, 0, 'f', 3)
            .arg(FINAL_MAX_POSITION_STEP_MM, 0, 'f', 1);
        return false;
    }
    if (maxControllerEulerStepDeg > FINAL_MAX_CONTROLLER_EULER_STEP_DEG
        || maxPhysicalOrientationStepDeg > FINAL_MAX_PHYSICAL_ORIENTATION_STEP_DEG)
    {
        error = QString("最终焊接姿态相邻控制器欧拉跳变/物理旋转=%1/%2 deg，超过硬门限 %3/%4 deg。")
            .arg(maxControllerEulerStepDeg, 0, 'f', 3)
            .arg(maxPhysicalOrientationStepDeg, 0, 'f', 3)
            .arg(FINAL_MAX_CONTROLLER_EULER_STEP_DEG, 0, 'f', 1)
            .arg(FINAL_MAX_PHYSICAL_ORIENTATION_STEP_DEG, 0, 'f', 1);
        return false;
    }

    int firstMatchedSourceIndex = -1;
    int lastMatchedSourceIndexValue = -1;
    for (int sourceIndex : matchedSourceIndexes)
    {
        if (sourceIndex < 0)
        {
            continue;
        }
        if (firstMatchedSourceIndex < 0)
        {
            firstMatchedSourceIndex = sourceIndex;
        }
        lastMatchedSourceIndexValue = sourceIndex;
    }
    const double matchedArcRatio = totalLengthMm > 0.0
        ? matchedFinalArcMm / totalLengthMm
        : 0.0;
    int expectedRetainedSourceRecordCount = 0;
    int matchedRetainedSourceRecordCount = 0;
    for (int sourceIndex = 0; sourceIndex < sourceArcMm.size(); ++sourceIndex)
    {
        if (sourceArcMm[sourceIndex] + 1e-9 < expectedSourceStartArcMm
            || sourceArcMm[sourceIndex] - 1e-9 > expectedSourceEndArcMm)
        {
            continue;
        }
        ++expectedRetainedSourceRecordCount;
        if (uniqueMatchedSourceIndexes.contains(sourceIndex))
        {
            ++matchedRetainedSourceRecordCount;
        }
    }
    const double sourceUniqueCoverageRatio = expectedRetainedSourceRecordCount > 0
        ? static_cast<double>(matchedRetainedSourceRecordCount)
            / static_cast<double>(expectedRetainedSourceRecordCount)
        : 0.0;
    const double matchedSourceStartArcMm = firstMatchedSourceIndex >= 0
        ? sourceArcMm[firstMatchedSourceIndex]
        : 0.0;
    const double matchedSourceEndArcMm = lastMatchedSourceIndexValue >= 0
        ? sourceArcMm[lastMatchedSourceIndexValue]
        : 0.0;
    const double matchedExpectedSourceArcMm = std::max(
        0.0,
        std::min(matchedSourceEndArcMm, expectedSourceEndArcMm)
            - std::max(matchedSourceStartArcMm, expectedSourceStartArcMm));
    const double sourceArcSpanRatio = sourceLengthMm > 0.0
        && firstMatchedSourceIndex >= 0
        && lastMatchedSourceIndexValue >= firstMatchedSourceIndex
        ? matchedExpectedSourceArcMm / expectedRetainedSourceLengthMm
        : 0.0;
    if (matchedArcRatio < FINAL_MIN_MATCHED_ARC_RATIO
        || sourceUniqueCoverageRatio < FINAL_MIN_SOURCE_UNIQUE_COVERAGE_RATIO
        || sourceArcSpanRatio < FINAL_MIN_SOURCE_ARC_SPAN_RATIO
        || maxSourceDisplacementMm > FINAL_MAX_SOURCE_DISPLACEMENT_MM
        || maxSourceControllerEulerDeltaDeg > FINAL_MAX_SOURCE_CONTROLLER_EULER_DELTA_DEG
        || maxSourcePhysicalOrientationDeltaDeg > FINAL_MAX_SOURCE_PHYSICAL_ORIENTATION_DELTA_DEG)
    {
        error = QString("最终焊道与补偿前姿态拓扑不一致：匹配弧长=%1%，源唯一覆盖=%2%，源弧长跨度=%3%，最大位移=%4 mm，控制器/物理姿态差=%5/%6 deg；门限=%7%/%8%/%9%/%10mm/%11deg/%12deg。")
            .arg(matchedArcRatio * 100.0, 0, 'f', 1)
            .arg(sourceUniqueCoverageRatio * 100.0, 0, 'f', 1)
            .arg(sourceArcSpanRatio * 100.0, 0, 'f', 1)
            .arg(maxSourceDisplacementMm, 0, 'f', 3)
            .arg(maxSourceControllerEulerDeltaDeg, 0, 'f', 3)
            .arg(maxSourcePhysicalOrientationDeltaDeg, 0, 'f', 3)
            .arg(FINAL_MIN_MATCHED_ARC_RATIO * 100.0, 0, 'f', 0)
            .arg(FINAL_MIN_SOURCE_UNIQUE_COVERAGE_RATIO * 100.0, 0, 'f', 0)
            .arg(FINAL_MIN_SOURCE_ARC_SPAN_RATIO * 100.0, 0, 'f', 0)
            .arg(FINAL_MAX_SOURCE_DISPLACEMENT_MM, 0, 'f', 1)
            .arg(FINAL_MAX_SOURCE_CONTROLLER_EULER_DELTA_DEG, 0, 'f', 1)
            .arg(FINAL_MAX_SOURCE_PHYSICAL_ORIENTATION_DELTA_DEG, 0, 'f', 1);
        return false;
    }
    const double expectedRetainedPointCloudSpanMm = std::max(
        0.0,
        qualityReport.projectedSpanMm - declaredStartSkipMm - declaredEndSkipMm);
    const double minLengthMm = std::max(
        expectedRetainedPointCloudSpanMm * settings.validationMinOutputLengthRatio,
        expectedRetainedSourceLengthMm * FINAL_MIN_PRECOMP_LENGTH_RATIO);
    const double maxLengthMm = expectedRetainedSourceLengthMm * FINAL_MAX_PRECOMP_LENGTH_RATIO;
    if (totalLengthMm < minLengthMm || totalLengthMm > maxLengthMm)
    {
        error = QString("最终焊接姿态总长 %1 mm，不在本次点云跨度/声明裁剪后补偿前轨迹绑定区间 [%2,%3] mm（点云跨度 %4 mm，补偿前 %5 mm，声明裁剪=%6+%7 mm）。")
            .arg(totalLengthMm, 0, 'f', 3)
            .arg(minLengthMm, 0, 'f', 3)
            .arg(maxLengthMm, 0, 'f', 3)
            .arg(qualityReport.projectedSpanMm, 0, 'f', 3)
            .arg(sourceLengthMm, 0, 'f', 3)
            .arg(declaredStartSkipMm, 0, 'f', 3)
            .arg(declaredEndSkipMm, 0, 'f', 3);
        return false;
    }
    return true;
}

QString BuildFinalSampledWeldPosePath(const QString& poseFilePath, bool uniqueResumeOutput = false)
{
    const QFileInfo poseInfo(QDir::fromNativeSeparators(poseFilePath));
    QString baseName = poseInfo.completeBaseName().isEmpty()
        ? QStringLiteral("WeldPose")
        : poseInfo.completeBaseName();
    // 重复续焊始终从固定 stem 派生短文件名，避免执行后缀在 Windows 路径中无限叠加。
    static const QRegularExpression executionSuffix(
        QStringLiteral("(?:_Resume_[0-9a-f-]+)?_FinalSampled$"),
        QRegularExpression::CaseInsensitiveOption);
    while (baseName.contains(executionSuffix))
    {
        baseName.remove(executionSuffix);
    }
    const QString suffix = uniqueResumeOutput
        ? QString("_Resume_%1_FinalSampled.txt").arg(
            QUuid::createUuid().toString(QUuid::WithoutBraces).remove(QLatin1Char('-')).left(12))
        : QStringLiteral("_FinalSampled.txt");
    return QDir::toNativeSeparators(poseInfo.dir().filePath(baseName + suffix));
}

bool SaveWeldPoseFileRecords(
    const QString& path,
    const QVector<WeldPoseFileRecord>& records,
    QString& error,
    QString* savedSha256 = nullptr,
    qint64* savedSize = nullptr)
{
    error.clear();
    if (savedSha256 != nullptr)
    {
        savedSha256->clear();
    }
    if (savedSize != nullptr)
    {
        *savedSize = -1;
    }
    if (records.isEmpty())
    {
        error = "抽样后没有可保存的焊接姿态点。";
        return false;
    }

    const QFileInfo fileInfo(QDir::fromNativeSeparators(path));
    const QDir parentDir = fileInfo.dir();
    if (!parentDir.exists() && !QDir().mkpath(parentDir.absolutePath()))
    {
        error = QString("创建最终抽样轨迹目录失败：%1").arg(parentDir.absolutePath());
        return false;
    }

    QByteArray payload;
    for (const WeldPoseFileRecord& record : records)
    {
        payload.append(BuildWeldPoseFileRecordLine(record).toUtf8());
        payload.append('\n');
    }

    QSaveFile file(fileInfo.absoluteFilePath());
    if (!file.open(QIODevice::WriteOnly))
    {
        error = QString("保存最终抽样轨迹文件失败：%1").arg(fileInfo.absoluteFilePath());
        return false;
    }
    if (file.write(payload) != payload.size() || !file.commit())
    {
        error = QString("原子提交最终抽样轨迹文件失败：%1").arg(fileInfo.absoluteFilePath());
        return false;
    }
    if (savedSha256 != nullptr)
    {
        *savedSha256 = QString::fromLatin1(
            QCryptographicHash::hash(payload, QCryptographicHash::Sha256).toHex()).toLower();
    }
    if (savedSize != nullptr)
    {
        *savedSize = payload.size();
    }
    return true;
}

QString NormalizeWeldSegmentKind(QString segmentKind)
{
    constexpr auto arcSuffix = "_arc";
    if (segmentKind.endsWith(arcSuffix, Qt::CaseInsensitive))
    {
        segmentKind.chop(static_cast<int>(std::strlen(arcSuffix)));
    }

    constexpr auto transitionSuffix = "_transition";
    if (segmentKind.endsWith(transitionSuffix, Qt::CaseInsensitive))
    {
        segmentKind.chop(static_cast<int>(std::strlen(transitionSuffix)));
    }
    return segmentKind;
}

int WeldSegmentKindCode(const QString& segmentKind)
{
    const QString normalized = NormalizeWeldSegmentKind(segmentKind).trimmed().toLower();
    if (normalized == "low_platform")
    {
        return 0;
    }
    if (normalized == "rising_edge")
    {
        return 1;
    }
    if (normalized == "high_platform")
    {
        return 2;
    }
    if (normalized == "falling_edge")
    {
        return 3;
    }
    return 4;
}

bool IsWeldPoseTransitionRecord(const WeldPoseFileRecord& record)
{
    return record.segmentKind.contains("_transition", Qt::CaseInsensitive)
        || record.pointType.contains("_transition", Qt::CaseInsensitive);
}

std::vector<QString> BuildWeldSegmentKindDebugLines(const QVector<WeldPoseFileRecord>& records)
{
    std::vector<QString> lines;
    lines.reserve(static_cast<size_t>(records.size()) + 1);
    lines.push_back("weld_index raw_index x y z segment_code transition arc");
    for (const WeldPoseFileRecord& record : records)
    {
        const int segmentCode = WeldSegmentKindCode(record.segmentKind);
        const int transitionFlag = IsWeldPoseTransitionRecord(record) ? 1 : 0;
        const int arcFlag = record.segmentKind.contains("_arc", Qt::CaseInsensitive)
            || record.pointType.contains("_arc", Qt::CaseInsensitive)
            ? 1
            : 0;
        lines.push_back(QString("%1 %2 %3 %4 %5 %6 %7 %8")
            .arg(record.weldIndex)
            .arg(record.rawIndex)
            .arg(record.point.x(), 0, 'f', 6)
            .arg(record.point.y(), 0, 'f', 6)
            .arg(record.point.z(), 0, 'f', 6)
            .arg(segmentCode)
            .arg(transitionFlag)
            .arg(arcFlag));
    }
    return lines;
}

Eigen::Vector3d ResolveHorizontalTangentDirection(
    const QVector<Eigen::Vector3d>& points,
    int pointIndex)
{
    if (points.isEmpty() || pointIndex < 0 || pointIndex >= points.size())
    {
        return Eigen::Vector3d::Zero();
    }

    Eigen::Vector3d tangent = Eigen::Vector3d::Zero();
    if (pointIndex > 0 && pointIndex + 1 < points.size())
    {
        tangent = points[pointIndex + 1] - points[pointIndex - 1];
    }
    else if (pointIndex + 1 < points.size())
    {
        tangent = points[pointIndex + 1] - points[pointIndex];
    }
    else if (pointIndex > 0)
    {
        tangent = points[pointIndex] - points[pointIndex - 1];
    }

    return HorizontalUnitOrZero(tangent);
}

Eigen::Vector3d CanonicalHorizontalWeldAxis(Eigen::Vector3d direction)
{
    direction = HorizontalUnitOrZero(direction);
    if (direction.head<2>().norm() <= 1e-9)
    {
        return Eigen::Vector3d::Zero();
    }

    const double absX = std::abs(direction.x());
    const double absY = std::abs(direction.y());
    const bool reverse = absY >= absX
        ? direction.y() < 0.0
        : direction.x() < 0.0;
    return reverse ? -direction : direction;
}

Eigen::Vector3d ResolveOverallHorizontalWeldDirection(
    const QVector<Eigen::Vector3d>& points)
{
    if (points.size() < 2)
    {
        return Eigen::Vector3d::Zero();
    }

    const Eigen::Vector3d startToEnd = HorizontalUnitOrZero(points.back() - points.front());
    if (startToEnd.head<2>().norm() > 1e-9)
    {
        return CanonicalHorizontalWeldAxis(startToEnd);
    }

    double longestSegmentLength = 0.0;
    Eigen::Vector3d longestDirection = Eigen::Vector3d::Zero();
    for (int index = 1; index < points.size(); ++index)
    {
        Eigen::Vector3d segment = points[index] - points[index - 1];
        segment.z() = 0.0;
        const double length = segment.head<2>().norm();
        if (length > longestSegmentLength)
        {
            longestSegmentLength = length;
            longestDirection = segment / length;
        }
    }
    return CanonicalHorizontalWeldAxis(longestDirection);
}

QString RobotCoorsText(const T_ROBOT_COORS& coors)
{
    return QString("X=%1 Y=%2 Z=%3 RX=%4 RY=%5 RZ=%6 BX=%7 BY=%8 BZ=%9")
        .arg(coors.dX, 0, 'f', 3)
        .arg(coors.dY, 0, 'f', 3)
        .arg(coors.dZ, 0, 'f', 3)
        .arg(coors.dRX, 0, 'f', 3)
        .arg(coors.dRY, 0, 'f', 3)
        .arg(coors.dRZ, 0, 'f', 3)
        .arg(coors.dBX, 0, 'f', 3)
        .arg(coors.dBY, 0, 'f', 3)
        .arg(coors.dBZ, 0, 'f', 3);
}

double WrappedAngleDistanceDeg(double left, double right)
{
    double delta = std::fmod(std::abs(left - right), 360.0);
    if (delta > 180.0)
    {
        delta = 360.0 - delta;
    }
    return delta;
}

double DegToRad(double deg)
{
    return deg * M_PI / 180.0;
}

T_ROBOT_COORS BuildScanSafeCoorsFromAnchor(
    const T_ROBOT_COORS& anchor,
    const T_PRECISE_MEASURE_PARAM& param)
{
    const double distance = std::isfinite(param.dScanSafeOffsetDistanceMm) && param.dScanSafeOffsetDistanceMm > 0.0
        ? param.dScanSafeOffsetDistanceMm
        : 150.0;
    const double angleDeg = std::isfinite(param.dScanSafeGunAngleDeg)
        ? param.dScanSafeGunAngleDeg
        : 30.0;
    const double angleRad = DegToRad(angleDeg);
    const double xSign = param.nScanSafeXDirection >= 0 ? 1.0 : -1.0;

    T_ROBOT_COORS safe = anchor;
    safe.dX += xSign * distance * std::sin(angleRad);
    safe.dZ += distance * std::cos(angleRad);
    return safe;
}

double PulseDeltaDeg(long currentPulse, long targetPulse, double pulseUnit)
{
    if (!std::isfinite(pulseUnit) || std::abs(pulseUnit) <= 1e-12)
    {
        return 0.0;
    }
    return std::abs(static_cast<double>(currentPulse - targetPulse) * pulseUnit);
}

double MaxWristDeltaDeg(
    const T_ANGLE_PULSE& currentPulse,
    const T_ANGLE_PULSE& targetPulse,
    const T_AXISUNIT& axisUnit)
{
    const double r = PulseDeltaDeg(currentPulse.nRPulse, targetPulse.nRPulse, axisUnit.dRPulseUnit);
    const double b = PulseDeltaDeg(currentPulse.nBPulse, targetPulse.nBPulse, axisUnit.dBPulseUnit);
    const double t = PulseDeltaDeg(currentPulse.nTPulse, targetPulse.nTPulse, axisUnit.dTPulseUnit);
    return std::max({ r, b, t });
}

int NormalizeWeldSafeRetreatDirectionMode(int mode)
{
    switch (mode)
    {
    case WELD_SAFE_RETREAT_AUTO_LEGACY_X_NEGATIVE:
    case WELD_SAFE_RETREAT_WORLD_X_NEGATIVE:
    case WELD_SAFE_RETREAT_WORLD_X_POSITIVE:
    case WELD_SAFE_RETREAT_WORLD_Y_NEGATIVE:
    case WELD_SAFE_RETREAT_WORLD_Y_POSITIVE:
        return mode;
    default:
        return WELD_SAFE_RETREAT_AUTO_LEGACY_X_NEGATIVE;
    }
}

QString WeldSafeRetreatDirectionText(int mode)
{
    switch (NormalizeWeldSafeRetreatDirectionMode(mode))
    {
    case WELD_SAFE_RETREAT_WORLD_X_NEGATIVE:
        return QStringLiteral("世界 X-");
    case WELD_SAFE_RETREAT_WORLD_X_POSITIVE:
        return QStringLiteral("世界 X+");
    case WELD_SAFE_RETREAT_WORLD_Y_NEGATIVE:
        return QStringLiteral("世界 Y-");
    case WELD_SAFE_RETREAT_WORLD_Y_POSITIVE:
        return QStringLiteral("世界 Y+");
    default:
        return QStringLiteral("自动（兼容旧算法，X-优先）");
    }
}

bool TryBuildWeldSafeCoors(
    const QVector<WeldPoseFileRecord>& records,
    int pointIndex,
    double safeOffsetDistanceMm,
    int retreatDirectionMode,
    int robotType,
    T_ROBOT_COORS& safeCoors,
    QString& error)
{
    error.clear();
    if (records.size() < 2)
    {
        error = "焊接姿态点不足 2 个，无法计算安全位置。";
        return false;
    }
    if (pointIndex < 0 || pointIndex >= records.size())
    {
        error = QString("安全位置锚点越界：%1").arg(pointIndex);
        return false;
    }

    const WeldPoseFileRecord& anchor = records[pointIndex];
    Eigen::Vector3d lateralDirection = Eigen::Vector3d::Zero();
    switch (NormalizeWeldSafeRetreatDirectionMode(retreatDirectionMode))
    {
    case WELD_SAFE_RETREAT_WORLD_X_NEGATIVE:
        lateralDirection = -Eigen::Vector3d::UnitX();
        break;
    case WELD_SAFE_RETREAT_WORLD_X_POSITIVE:
        lateralDirection = Eigen::Vector3d::UnitX();
        break;
    case WELD_SAFE_RETREAT_WORLD_Y_NEGATIVE:
        lateralDirection = -Eigen::Vector3d::UnitY();
        break;
    case WELD_SAFE_RETREAT_WORLD_Y_POSITIVE:
        lateralDirection = Eigen::Vector3d::UnitY();
        break;
    default:
        {
            QVector<Eigen::Vector3d> points;
            points.reserve(records.size());
            for (const WeldPoseFileRecord& record : records)
            {
                points.push_back(record.point);
            }

            const Eigen::Vector3d seamDirection = ResolveHorizontalTangentDirection(points, pointIndex);
            if (seamDirection.head<2>().norm() <= 1e-9)
            {
                error = QString("第 %1 个焊点附近焊道方向无效，无法计算安全位置。").arg(pointIndex + 1);
                return false;
            }

            lateralDirection = HorizontalUnitOrZero(
                Eigen::Vector3d::UnitZ().cross(seamDirection));
            if (lateralDirection.head<2>().norm() <= 1e-9)
            {
                error = QString("第 %1 个焊点附近横向法向无效，无法计算安全位置。").arg(pointIndex + 1);
                return false;
            }

            Eigen::Vector3d safeGunAxis = Eigen::Vector3d::UnitY();
            if (robotType == ROBOT_TYPE_STEP)
            {
                safeGunAxis = -safeGunAxis;
            }
            const Eigen::Vector3d gunDirection = HorizontalUnitOrZero(
                RobotPoseTransform::RotationFromAnglesDeg(anchor.rx, anchor.ry, anchor.rz, robotType)
                * safeGunAxis);
            if (gunDirection.head<2>().norm() > 1e-9
                && lateralDirection.head<2>().dot(gunDirection.head<2>()) < 0.0)
            {
                lateralDirection = -lateralDirection;
            }
            // 兼容旧现场：自动模式继续从世界 X- 侧优先回撤。
            if (lateralDirection.x() > 0.0)
            {
                lateralDirection = -lateralDirection;
            }
            break;
        }
    }

    const double safeOffsetDistance =
        std::isfinite(safeOffsetDistanceMm) && safeOffsetDistanceMm > 0.0
        ? safeOffsetDistanceMm
        : WELD_SAFE_OFFSET_DISTANCE_MM;
    const Eigen::Vector3d safeOffsetDirection =
        (Eigen::Vector3d::UnitZ() + lateralDirection).normalized();
    const Eigen::Vector3d safePoint =
        anchor.point + safeOffsetDirection * safeOffsetDistance;

    safeCoors = T_ROBOT_COORS(
        safePoint.x(),
        safePoint.y(),
        safePoint.z(),
        anchor.rx,
        anchor.ry,
        anchor.rz,
        anchor.bx,
        anchor.by,
        anchor.bz);
    return true;
}

T_ROBOT_COORS BuildWeldPoseCoors(const WeldPoseFileRecord& record)
{
    return T_ROBOT_COORS(
        record.point.x(),
        record.point.y(),
        record.point.z(),
        record.rx,
        record.ry,
        record.rz,
        record.bx,
        record.by,
        record.bz);
}

QVector<WeldPoseFileRecord> SampleFinalWeldTrajectoryRecords(
    const QVector<WeldPoseFileRecord>& records,
    double sampleStepMm,
    bool keepAnchorsOnly = false)
{
    if (records.size() <= 2 || !std::isfinite(sampleStepMm) || sampleStepMm <= 0.0)
    {
        return records;
    }

    // 实际焊道抽样(精简与非精简共用拐角/搭接收敛，只差普通点是否保留)：
    //   · 拐角"跟随圆弧过渡"：圆弧化拐角(corner 点簇含 _arc)保留簇内全部点 = 机器人走圆弧的实际路径；
    //     尖角(corner 点簇无 _arc)收敛为 1 个干净关键节点(取簇中点)，其余冗余点丢弃；
    //   · 搭接台阶簇只保留台阶横向(X)突变前后两点；拐角代表/圆弧路径/搭接两点均强制保留(绕过间距闸门)；
    //   · 普通点：非精简按设定点间距去密保留；精简(keepAnchorsOnly)则全部丢弃，只留上述特殊点。
    const double minGapMm = sampleStepMm;
    const int pointCount = records.size();

    auto isCornerType = [&records](int i) {
        return records[i].pointType.trimmed().toLower().contains(QStringLiteral("corner"));
    };

    // 拐角点簇预处理：forceKeep=强制保留(拐角代表/圆弧路径)，dropPoint=尖角簇内冗余点(整段抽样都不保留)。
    QVector<char> forceKeep(pointCount, 0);
    QVector<char> dropPoint(pointCount, 0);
    for (int i = 0; i < pointCount; )
    {
        if (!isCornerType(i)) { ++i; continue; }
        int j = i;
        bool clusterHasArc = false;
        while (j < pointCount && isCornerType(j))
        {
            if (records[j].pointType.trimmed().toLower().contains(QStringLiteral("_arc")))
            {
                clusterHasArc = true;
            }
            ++j;
        }
        if (clusterHasArc)
        {
            for (int k = i; k < j; ++k) { forceKeep[k] = 1; }            // 圆弧路径：簇内全保留
        }
        else
        {
            const int mid = i + (j - i) / 2;                             // 尖角：收敛为簇中点
            for (int k = i; k < j; ++k) { if (k == mid) { forceKeep[k] = 1; } else { dropPoint[k] = 1; } }
        }
        i = j;
    }

    // 搭接台阶簇预处理(覆盖拐角标记)：搭接错位是垂直焊缝(X)方向的横移台阶，关键节点是台阶两端的角点。
    // 因此搭接簇内只保留 X 最小端 与 X 最大端(=横移台阶前后的两个关键角点)，其余加密/过渡/近重复点全部丢弃，
    // 既保住台阶 X 突变和两端关键节点，又不堆集。
    for (int i = 0; i < pointCount; )
    {
        if (!records[i].isLapStep) { ++i; continue; }
        int j = i;
        while (j < pointCount && records[j].isLapStep) { ++j; }
        for (int k = i; k < j; ++k) { dropPoint[k] = 1; forceKeep[k] = 0; }   // 簇内先全丢
        int loIdx = i;
        int hiIdx = i;
        for (int k = i + 1; k < j; ++k)
        {
            if (records[k].point.x() < records[loIdx].point.x()) { loIdx = k; }
            if (records[k].point.x() > records[hiIdx].point.x()) { hiIdx = k; }
        }
        dropPoint[loIdx] = 0; forceKeep[loIdx] = 1;   // X 最小端(横移台阶一侧关键角)
        dropPoint[hiIdx] = 0; forceKeep[hiIdx] = 1;   // X 最大端(横移台阶另一侧关键角)
        i = j;
    }

    struct KeptPoint
    {
        int index = 0;
        bool anchor = false;
        double arcLengthMm = 0.0;
    };
    QVector<KeptPoint> kept;
    kept.reserve(pointCount);
    kept.push_back({ 0, true, 0.0 });

    double arcLengthMm = 0.0;
    for (int index = 1; index < pointCount - 1; ++index)
    {
        const double segmentLengthMm = (records[index].point - records[index - 1].point).norm();
        if (std::isfinite(segmentLengthMm) && segmentLengthMm > 1e-6)
        {
            arcLengthMm += segmentLengthMm;
        }

        if (dropPoint[index])
        {
            continue;   // 尖角簇内冗余点：丢弃，使拐角只留 1 个干净关键节点
        }

        if (forceKeep[index])
        {
            // 强制保留(拐角代表/圆弧路径/搭接台阶前后两点)：回溯挤掉与其过近的普通采样点，再无条件压入。
            while (kept.size() > 1
                && !kept.back().anchor
                && arcLengthMm - kept.back().arcLengthMm < minGapMm)
            {
                kept.pop_back();
            }
            kept.push_back({ index, true, arcLengthMm });
        }
        else if (!keepAnchorsOnly && arcLengthMm - kept.back().arcLengthMm >= minGapMm)
        {
            // 普通采样点：精简模式不保留(只留特殊点)，非精简按点间距去密保留。
            kept.push_back({ index, false, arcLengthMm });
        }
    }

    // 终点必留且最优先：回溯挤掉与终点间距不足的普通采样点（首点与强制保留点不被吞）。
    const double tailSegmentMm =
        (records[pointCount - 1].point - records[pointCount - 2].point).norm();
    if (std::isfinite(tailSegmentMm) && tailSegmentMm > 1e-6)
    {
        arcLengthMm += tailSegmentMm;
    }
    while (kept.size() > 1
        && !kept.back().anchor   // 强制保留点(拐角/圆弧/搭接)不被终点必留逻辑吞掉
        && arcLengthMm - kept.back().arcLengthMm < minGapMm)
    {
        kept.pop_back();
    }

    QVector<WeldPoseFileRecord> sampled;
    sampled.reserve(kept.size() + 1);
    for (const KeptPoint& keep : kept)
    {
        sampled.push_back(records[keep.index]);
    }
    constexpr double kDuplicateDistanceMm = 1e-6;
    if ((sampled.back().point - records.back().point).norm() <= kDuplicateDistanceMm)
    {
        sampled.back() = records.back();
    }
    else
    {
        sampled.push_back(records.back());
    }
    for (int index = 0; index < sampled.size(); ++index)
    {
        sampled[index].weldIndex = index + 1;
    }
    return sampled;
}

bool BuildWeldPoseMoveInfos(
    const QVector<WeldPoseFileRecord>& records,
    double linearSpeedMmPerSec,
    std::vector<T_ROBOT_MOVE_INFO>& moveInfos,
    QString& error,
    const WeldPosePreset* preset = nullptr,
    double finalTrajectorySampleStepMm = DEFAULT_FINAL_WELD_TRAJECTORY_SAMPLE_STEP_MM,
    double transitionLinearSpeed = 0.0,
    bool enableWeldProcess = false,
    QVector<WeldPoseFileRecord>* executionRecordsOut = nullptr,
    bool preserveInputRecords = false)
{
    moveInfos.clear();
    // 工艺里设置了点间距(>0)时优先用工艺的，否则用测量参数页传入值。
    double effectiveSampleStepMm = finalTrajectorySampleStepMm;
    if (preset != nullptr && preset->finalWeldStepFromProcessMm > 0.0)
    {
        effectiveSampleStepMm = preset->finalWeldStepFromProcessMm;
    }
    const QVector<WeldPoseFileRecord> executionRecords = preserveInputRecords
        ? records
        : SampleFinalWeldTrajectoryRecords(
            records,
            NormalizeFinalWeldTrajectorySampleStepMm(effectiveSampleStepMm),
            preset != nullptr && preset->keepAnchorsOnly);
    if (executionRecordsOut != nullptr)
    {
        *executionRecordsOut = executionRecords;
    }
    moveInfos.reserve(static_cast<size_t>(executionRecords.size()));

    const bool useWeldProcess = enableWeldProcess
        && preset != nullptr
        && preset->weldProcessLoaded;
    if (enableWeldProcess
        && preset != nullptr
        && !preset->weldProcessLoaded
        && !preset->weldProcessLoadError.isEmpty())
    {
        error = preset->weldProcessLoadError;
        return false;
    }
    if (useWeldProcess && preset->transitionCurrentVoltageEnableMismatch)
    {
        error = "拐点过渡电流和过渡电压必须同时启用或同时关闭，请检查当前焊接工艺参数。";
        return false;
    }

    const int transitionApplyScope = preset != nullptr
        ? NormalizeTransitionApplyScope(preset->transitionApplyScope)
        : kTransitionScopeArcAndTransition;
    const auto recordTag = [](const WeldPoseFileRecord& record) -> QString
        {
            return (record.pointType + " " + record.segmentKind).trimmed().toLower();
        };
    const auto isArcRecord = [&](const WeldPoseFileRecord& record) -> bool
        {
            const QString tag = recordTag(record);
            return tag.contains("_arc") || tag.contains(QStringLiteral("圆弧"));
        };
    const auto isTransitionRecord = [&](const WeldPoseFileRecord& record) -> bool
        {
            const QString tag = recordTag(record);
            return tag.contains("transition") || tag.contains(QStringLiteral("过渡"));
        };
    const auto shouldUseTransitionParam = [&](const WeldPoseFileRecord& record) -> bool
        {
            const bool isArc = isArcRecord(record);
            const bool isTransition = isTransitionRecord(record);
            switch (transitionApplyScope)
            {
            case kTransitionScopeArc:
                return isArc;
            case kTransitionScopeTransition:
                return isTransition;
            default:
                return isArc || isTransition;
            }
        };

    int externalAxisPointCount = 0;
    for (const WeldPoseFileRecord& record : records)
    {
        if (std::abs(record.bx) > 1e-6 || std::abs(record.by) > 1e-6 || std::abs(record.bz) > 1e-6)
        {
            ++externalAxisPointCount;
        }
    }
    if (externalAxisPointCount > 0)
    {
        error = QString("焊接姿态文件包含 %1 个外部轴点位，但当前多点 TP 下发只支持 GP1 六轴点位，请先确认 BX/BY/BZ 是否应为 0。")
            .arg(externalAxisPointCount);
        return false;
    }

    for (const WeldPoseFileRecord& record : executionRecords)
    {
        const bool useTransitionParam = shouldUseTransitionParam(record);
        const bool useTransitionSpeed = preset != nullptr
            && preset->transitionSpeedEnabled
            && transitionLinearSpeed > 0.0
            && useTransitionParam;
        const bool useTransitionWeldParams = useWeldProcess
            && preset->transitionCurrentVoltageEnabled
            && useTransitionParam;

        T_ROBOT_MOVE_INFO moveInfo;
        moveInfo.nMoveType = MOVL;
        moveInfo.tCoord = BuildWeldPoseCoors(record);
        moveInfo.tSpeed = T_ROBOT_MOVE_SPEED(
            useTransitionSpeed ? transitionLinearSpeed : linearSpeedMmPerSec,
            0.0,
            0.0);
        // 搭接错位台阶端点用精确转角(overlap=0)：否则连续运动 20% 转弯区会把台阶两个近 90°角混合圆化，
        // 抵消上游保台阶的处理。台阶点精确定位，机器人精确走出垂直 X 台阶。
        moveInfo.dOverlapRel = record.isLapStep
            ? 0.0
            : (preset != nullptr ? preset->stepOverlapRel : 20.0);
        moveInfo.nPostureType = preset != nullptr ? preset->weldPostureType : 1;  // 焊接姿态/位形(srp第4参)
        moveInfo.nDynamicMode = preset != nullptr ? preset->weldDynamicMode : 0;  // 动态特性(WLin第2参DYNAMIC)
        moveInfo.nMoveDevice = 0;
        moveInfo.nTrackNo = 0;
        moveInfo.adBasePosVar[0] = record.bx;
        moveInfo.adBasePosVar[1] = record.by;
        moveInfo.adBasePosVar[2] = record.bz;
        if (useWeldProcess)
        {
            moveInfo.bWeldProcessEnabled = true;
            moveInfo.bUseTransitionWeldParams = useTransitionWeldParams;
            moveInfo.dArcStartCurrent = preset->startArcCurrent;
            moveInfo.dArcStartVoltage = preset->startArcVoltage;
            moveInfo.dArcStartWaitTime = preset->startArcWaitTime;
            moveInfo.dWeldCurrent = useTransitionWeldParams
                ? preset->transitionCurrent
                : preset->weldCurrent;
            moveInfo.dWeldVoltage = useTransitionWeldParams
                ? preset->transitionVoltage
                : preset->weldVoltage;
            moveInfo.dWeldSpeedMmPerMin = (useTransitionSpeed && preset->transitionSpeedMmPerMin > 0.0)
                ? preset->transitionSpeedMmPerMin
                : preset->weldProcessSpeedMmPerMin;
            moveInfo.dArcEndCurrent = preset->stopArcCurrent;
            moveInfo.dArcEndVoltage = preset->stopArcVoltage;
            moveInfo.dArcEndWaitTime = preset->stopArcWaitTime;
            moveInfo.nArcMode = preset->arcMode;
            moveInfo.bHasWeaveParam = preset->weaveEnabled;
            moveInfo.bAppPointwiseWeave = preset->weaveAppPointwise;
            moveInfo.nWeavePointsPerCycle = preset->weavePointsPerCycle;
            if (moveInfo.bHasWeaveParam)
            {
                moveInfo.tWeaveParam = preset->weaveParam;
            }
            moveInfo.bHasTrackParam = preset->trackEnabled;
            if (moveInfo.bHasTrackParam)
            {
                moveInfo.tTrackParam = preset->trackParam;
            }
        }
        moveInfos.push_back(moveInfo);
    }

    if (useWeldProcess && !moveInfos.empty())
    {
        moveInfos.front().bArcStartBeforeMove = true;
        moveInfos.back().bArcEndAfterMove = true;
    }

    return true;
}

double EstimateMoveInfosPathLengthMm(const std::vector<T_ROBOT_MOVE_INFO>& moveInfos)
{
    if (moveInfos.size() < 2)
    {
        return 0.0;
    }

    double totalLengthMm = 0.0;
    for (size_t index = 1; index < moveInfos.size(); ++index)
    {
        const T_ROBOT_COORS& prev = moveInfos[index - 1].tCoord;
        const T_ROBOT_COORS& curr = moveInfos[index].tCoord;
        const double dx = curr.dX - prev.dX;
        const double dy = curr.dY - prev.dY;
        const double dz = curr.dZ - prev.dZ;
        totalLengthMm += std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    return totalLengthMm;
}

double EstimateMoveInfosRunMs(
    const std::vector<T_ROBOT_MOVE_INFO>& moveInfos,
    bool fanucSpeedIsMmPerSec)
{
    if (moveInfos.size() < 2)
    {
        return 0.0;
    }

    double totalRunMs = 0.0;
    for (size_t index = 1; index < moveInfos.size(); ++index)
    {
        const T_ROBOT_COORS& prev = moveInfos[index - 1].tCoord;
        const T_ROBOT_COORS& curr = moveInfos[index].tCoord;
        const double dx = curr.dX - prev.dX;
        const double dy = curr.dY - prev.dY;
        const double dz = curr.dZ - prev.dZ;
        const double segmentLengthMm = std::sqrt(dx * dx + dy * dy + dz * dz);
        const double configuredSpeed = moveInfos[index].tSpeed.dSpeed;
        const double speedMmPerSec = fanucSpeedIsMmPerSec
            ? configuredSpeed
            : configuredSpeed / 60.0;
        if (!std::isfinite(segmentLengthMm) || !std::isfinite(speedMmPerSec)
            || speedMmPerSec <= 0.0)
        {
            return std::numeric_limits<double>::infinity();
        }
        totalRunMs += segmentLengthMm / speedMmPerSec * 1000.0;
    }
    return totalRunMs;
}

struct WeldSeamCompApplyStats
{
    int zAdjustedCount = 0;
    int gunDirAdjustedCount = 0;
    int seamDirAdjustedCount = 0;
    int selfIntersectionTrimCount = 0;
    int selfIntersectionRemovedPointCount = 0;
};

struct WeldPathIntersection
{
    int firstSegmentIndex = -1;
    int secondSegmentIndex = -1;
    double firstRatio = 0.0;
    double secondRatio = 0.0;
    Eigen::Vector2d point = Eigen::Vector2d::Zero();
};

double Cross2d(const Eigen::Vector2d& left, const Eigen::Vector2d& right)
{
    return left.x() * right.y() - left.y() * right.x();
}

bool TryFindFirstWeldPathSelfIntersection(
    const QVector<WeldPoseFileRecord>& records,
    WeldPathIntersection& intersection)
{
    constexpr double kMinSegmentLengthMm = 1e-6;
    constexpr double kIntersectionEpsilon = 1e-6;
    intersection = WeldPathIntersection();

    if (records.size() < 4)
    {
        return false;
    }

    for (int firstIndex = 0; firstIndex + 1 < records.size(); ++firstIndex)
    {
        const Eigen::Vector2d firstBegin(
            records[firstIndex].point.x(),
            records[firstIndex].point.y());
        const Eigen::Vector2d firstEnd(
            records[firstIndex + 1].point.x(),
            records[firstIndex + 1].point.y());
        const Eigen::Vector2d firstDelta = firstEnd - firstBegin;
        if (firstDelta.norm() <= kMinSegmentLengthMm)
        {
            continue;
        }

        for (int secondIndex = firstIndex + 2; secondIndex + 1 < records.size(); ++secondIndex)
        {
            const Eigen::Vector2d secondBegin(
                records[secondIndex].point.x(),
                records[secondIndex].point.y());
            const Eigen::Vector2d secondEnd(
                records[secondIndex + 1].point.x(),
                records[secondIndex + 1].point.y());
            const Eigen::Vector2d secondDelta = secondEnd - secondBegin;
            if (secondDelta.norm() <= kMinSegmentLengthMm)
            {
                continue;
            }

            const double denominator = Cross2d(firstDelta, secondDelta);
            if (std::abs(denominator) <= 1e-9)
            {
                continue;
            }

            const Eigen::Vector2d beginDelta = secondBegin - firstBegin;
            const double firstRatio = Cross2d(beginDelta, secondDelta) / denominator;
            const double secondRatio = Cross2d(beginDelta, firstDelta) / denominator;
            if (firstRatio <= kIntersectionEpsilon
                || firstRatio >= 1.0 - kIntersectionEpsilon
                || secondRatio <= kIntersectionEpsilon
                || secondRatio >= 1.0 - kIntersectionEpsilon)
            {
                continue;
            }

            intersection.firstSegmentIndex = firstIndex;
            intersection.secondSegmentIndex = secondIndex;
            intersection.firstRatio = firstRatio;
            intersection.secondRatio = secondRatio;
            intersection.point = firstBegin + firstDelta * firstRatio;
            return true;
        }
    }

    return false;
}

WeldPoseFileRecord InterpolateWeldPoseRecord(
    const WeldPoseFileRecord& begin,
    const WeldPoseFileRecord& end,
    double ratio)
{
    const double safeRatio = std::clamp(ratio, 0.0, 1.0);
    WeldPoseFileRecord record = begin;
    record.rawIndex = static_cast<int>(std::lround(
        begin.rawIndex + (end.rawIndex - begin.rawIndex) * safeRatio));
    record.point = begin.point + (end.point - begin.point) * safeRatio;
    record.rx = begin.rx + (NormalizeAngleNear(end.rx, begin.rx) - begin.rx) * safeRatio;
    record.ry = begin.ry + (NormalizeAngleNear(end.ry, begin.ry) - begin.ry) * safeRatio;
    record.rz = NormalizeAngleToFanucRange(
        begin.rz + (NormalizeAngleNear(end.rz, begin.rz) - begin.rz) * safeRatio);
    record.bx = begin.bx + (end.bx - begin.bx) * safeRatio;
    record.by = begin.by + (end.by - begin.by) * safeRatio;
    record.bz = begin.bz + (end.bz - begin.bz) * safeRatio;
    if (safeRatio >= 0.5)
    {
        record.pointType = end.pointType;
        record.segmentKind = end.segmentKind;
    }
    return record;
}

void RenumberWeldPoseRecords(QVector<WeldPoseFileRecord>& records);

bool ClipWeldPoseRecordsAtArcLength(
    QVector<WeldPoseFileRecord>& records,
    double resumeStartArcMm,
    double* actualStartArcMm,
    QString& error)
{
    if (actualStartArcMm != nullptr)
    {
        *actualStartArcMm = 0.0;
    }
    if (!std::isfinite(resumeStartArcMm) || resumeStartArcMm < 0.0)
    {
        error = QStringLiteral("断点续焊起始弧长无效。");
        return false;
    }
    if (resumeStartArcMm <= 1e-9)
    {
        return true;
    }

    double cumulative = 0.0;
    for (int index = 0; index + 1 < records.size(); ++index)
    {
        const double segmentLength = (records[index + 1].point - records[index].point).norm();
        if (!std::isfinite(segmentLength) || segmentLength <= 1e-9)
        {
            continue;
        }
        const double segmentEndArc = cumulative + segmentLength;
        if (resumeStartArcMm <= segmentEndArc + 1e-9)
        {
            const double ratio = std::clamp(
                (resumeStartArcMm - cumulative) / segmentLength,
                0.0,
                1.0);
            QVector<WeldPoseFileRecord> clipped;
            clipped.reserve(records.size() - index);
            if (ratio <= 1e-9)
            {
                clipped.push_back(records[index]);
            }
            else if (ratio >= 1.0 - 1e-9)
            {
                clipped.push_back(records[index + 1]);
            }
            else
            {
                clipped.push_back(InterpolateWeldPoseRecord(records[index], records[index + 1], ratio));
            }

            const int remainingBegin = ratio >= 1.0 - 1e-9 ? index + 2 : index + 1;
            for (int remainingIndex = remainingBegin; remainingIndex < records.size(); ++remainingIndex)
            {
                clipped.push_back(records[remainingIndex]);
            }
            if (clipped.size() < 2)
            {
                error = QStringLiteral("断点续焊起始弧长已到轨迹末端，无可执行段。");
                return false;
            }
            RenumberWeldPoseRecords(clipped);
            records = clipped;
            if (actualStartArcMm != nullptr)
            {
                *actualStartArcMm = std::min(resumeStartArcMm, segmentEndArc);
            }
            return true;
        }
        cumulative = segmentEndArc;
    }

    error = QString("断点续焊起始弧长 %1 mm 已到或超过轨迹总长 %2 mm，无可执行段。")
        .arg(resumeStartArcMm, 0, 'f', 3)
        .arg(cumulative, 0, 'f', 3);
    return false;
}

void RenumberWeldPoseRecords(QVector<WeldPoseFileRecord>& records)
{
    for (int index = 0; index < records.size(); ++index)
    {
        records[index].weldIndex = index + 1;
    }
}

void TrimWeldPathSelfIntersections(
    QVector<WeldPoseFileRecord>& records,
    WeldSeamCompApplyStats& stats)
{
    constexpr int kMaxTrimIterations = 256;
    int iteration = 0;

    WeldPathIntersection intersection;
    while (iteration < kMaxTrimIterations
        && TryFindFirstWeldPathSelfIntersection(records, intersection))
    {
        const int firstIndex = intersection.firstSegmentIndex;
        const int secondIndex = intersection.secondSegmentIndex;
        if (firstIndex < 0
            || secondIndex <= firstIndex + 1
            || secondIndex + 1 >= records.size())
        {
            break;
        }

        WeldPoseFileRecord junctionRecord = InterpolateWeldPoseRecord(
            records[secondIndex],
            records[secondIndex + 1],
            intersection.secondRatio);
        junctionRecord.point.x() = intersection.point.x();
        junctionRecord.point.y() = intersection.point.y();

        records[firstIndex + 1] = junctionRecord;
        const int eraseBegin = firstIndex + 2;
        const int eraseEndExclusive = secondIndex + 1;
        if (eraseBegin < eraseEndExclusive)
        {
            stats.selfIntersectionRemovedPointCount += eraseEndExclusive - eraseBegin;
            records.erase(records.begin() + eraseBegin, records.begin() + eraseEndExclusive);
        }

        ++stats.selfIntersectionTrimCount;
        ++iteration;
    }

    if (stats.selfIntersectionTrimCount > 0)
    {
        RenumberWeldPoseRecords(records);
    }
}

struct WeldCornerArcApplyStats
{
    int inputPointCount = 0;
    int outputPointCount = 0;
    int candidateCornerCount = 0;
    int roundedCornerCount = 0;
    int reducedRadiusCornerCount = 0;
    double radiusMm = 0.0;
    double minimumAppliedRadiusMm = 0.0;

    int insertedPointCount() const
    {
        return outputPointCount - inputPointCount;
    }

    int skippedCornerCount() const
    {
        return std::max(0, candidateCornerCount - roundedCornerCount);
    }
};

bool IsWeldCornerPointType(const QString& pointType)
{
    const QString normalized = pointType.trimmed().toLower();
    return normalized.contains("corner") || normalized.contains(QStringLiteral("拐"));
}

bool IsWeldSegmentBoundaryAtCorner(
    const WeldPoseFileRecord& prev,
    const WeldPoseFileRecord& corner,
    const WeldPoseFileRecord& next)
{
    const QString prevKind = NormalizeWeldSegmentKind(prev.segmentKind).trimmed();
    const QString cornerKind = NormalizeWeldSegmentKind(corner.segmentKind).trimmed();
    const QString nextKind = NormalizeWeldSegmentKind(next.segmentKind).trimmed();
    if (prevKind.isEmpty() || cornerKind.isEmpty() || nextKind.isEmpty())
    {
        return false;
    }

    const bool startsNewSegment =
        prevKind.compare(cornerKind, Qt::CaseInsensitive) != 0
        && cornerKind.compare(nextKind, Qt::CaseInsensitive) == 0;
    if (startsNewSegment)
    {
        return true;
    }

    // 段切换前的最后一个点不是几何拐点，避免和真正拐点连续圆滑两次。
    const bool endsOldSegment =
        prevKind.compare(cornerKind, Qt::CaseInsensitive) == 0
        && cornerKind.compare(nextKind, Qt::CaseInsensitive) != 0;
    return !endsOldSegment
        && prevKind.compare(nextKind, Qt::CaseInsensitive) != 0;
}

double EstimateWeldPoseStepMm(const QVector<WeldPoseFileRecord>& records)
{
    QVector<double> lengths;
    lengths.reserve(records.size() > 1 ? records.size() - 1 : 0);
    for (int index = 1; index < records.size(); ++index)
    {
        const double length = (records[index].point - records[index - 1].point).norm();
        if (std::isfinite(length) && length > 1e-6)
        {
            lengths.push_back(length);
        }
    }

    if (lengths.isEmpty())
    {
        return 2.0;
    }

    std::sort(lengths.begin(), lengths.end());
    return std::clamp(lengths[lengths.size() / 2], 0.5, 5.0);
}

QString WeldArcSegmentKind(const QString& segmentKind)
{
    QString normalized = NormalizeWeldSegmentKind(segmentKind).trimmed();
    if (normalized.isEmpty())
    {
        normalized = "arc";
    }
    if (!normalized.endsWith("_arc", Qt::CaseInsensitive))
    {
        normalized += "_arc";
    }
    return normalized;
}

void AppendWeldPoseRecordIfNotDuplicate(
    QVector<WeldPoseFileRecord>& records,
    const WeldPoseFileRecord& record)
{
    constexpr double kDuplicateDistanceMm = 1e-6;
    if (!records.isEmpty()
        && (records.back().point - record.point).norm() <= kDuplicateDistanceMm)
    {
        records.back() = record;
        return;
    }

    records.push_back(record);
}

double WeldPoseTurnAngleRad(const Eigen::Vector3d& incoming, const Eigen::Vector3d& outgoing)
{
    const double incomingLength = incoming.norm();
    const double outgoingLength = outgoing.norm();
    if (incomingLength <= 1e-9 || outgoingLength <= 1e-9)
    {
        return 0.0;
    }

    const double cosTheta = std::clamp(
        incoming.dot(outgoing) / (incomingLength * outgoingLength),
        -0.999999,
        0.999999);
    return std::acos(cosTheta);
}

void TrimSharpEntryPointBeforeWeldArc(
    QVector<WeldPoseFileRecord>& records,
    const Eigen::Vector3d& tangentIn,
    double maxEntryAngleRad,
    double maxTrimDistanceMm)
{
    int trimmedCount = 0;
    while (records.size() >= 2 && trimmedCount < 3)
    {
        const WeldPoseFileRecord& last = records.back();
        if (last.pointType.contains("_arc", Qt::CaseInsensitive))
        {
            break;
        }

        const Eigen::Vector3d incoming = last.point - records[records.size() - 2].point;
        const Eigen::Vector3d outgoing = tangentIn - last.point;
        const double outgoingDistanceMm = outgoing.norm();
        if (outgoingDistanceMm <= 0.5)
        {
            records.removeLast();
            ++trimmedCount;
            continue;
        }

        if (outgoingDistanceMm > maxTrimDistanceMm)
        {
            break;
        }

        if (WeldPoseTurnAngleRad(incoming, outgoing) <= maxEntryAngleRad)
        {
            break;
        }

        records.removeLast();
        ++trimmedCount;
    }
}

bool IsWeldPoseArcRecord(const WeldPoseFileRecord& record)
{
    return record.pointType.contains(QStringLiteral("_arc"), Qt::CaseInsensitive)
        || record.segmentKind.contains(QStringLiteral("_arc"), Qt::CaseInsensitive);
}

void TrimSharpWeldArcEntryPoints(
    QVector<WeldPoseFileRecord>& records,
    double maxEntryAngleRad,
    double maxTrimDistanceMm)
{
    for (int index = 1; index < records.size(); ++index)
    {
        if (!IsWeldPoseArcRecord(records[index])
            || IsWeldPoseArcRecord(records[index - 1]))
        {
            continue;
        }

        int trimmedCount = 0;
        while (index >= 2 && trimmedCount < 3)
        {
            const int previousIndex = index - 1;
            if (IsWeldPoseArcRecord(records[previousIndex]))
            {
                break;
            }

            const Eigen::Vector3d incoming =
                records[previousIndex].point - records[previousIndex - 1].point;
            const Eigen::Vector3d outgoing =
                records[index].point - records[previousIndex].point;
            const double outgoingDistanceMm = outgoing.norm();

            if (outgoingDistanceMm > maxTrimDistanceMm)
            {
                break;
            }

            // 圆弧入口前如果残留一颗很近的原始点，视觉上会变成硬折角。
            if (outgoingDistanceMm > 0.5
                && WeldPoseTurnAngleRad(incoming, outgoing) <= maxEntryAngleRad)
            {
                break;
            }

            records.removeAt(previousIndex);
            --index;
            ++trimmedCount;
        }
    }
}

void TrimSharpWeldArcExitPoints(
    QVector<WeldPoseFileRecord>& records,
    double maxExitAngleRad,
    double maxTrimDistanceMm)
{
    constexpr int kMinimumArcRecordCount = 3;
    for (int index = 1; index < records.size(); ++index)
    {
        if (IsWeldPoseArcRecord(records[index])
            || !IsWeldPoseArcRecord(records[index - 1]))
        {
            continue;
        }

        int trimmedCount = 0;
        while (index >= 2 && trimmedCount < 3)
        {
            const int previousIndex = index - 1;
            if (!IsWeldPoseArcRecord(records[previousIndex]))
            {
                break;
            }

            int arcRunBegin = previousIndex;
            while (arcRunBegin > 0 && IsWeldPoseArcRecord(records[arcRunBegin - 1]))
            {
                --arcRunBegin;
            }
            if (previousIndex - arcRunBegin + 1 <= kMinimumArcRecordCount)
            {
                // 至少保留“切入点 + 中间点 + 切出点”三颗显式圆弧采样。
                // Finalize 会多轮做锐角检查；没有下限时半径较小的圆弧会被逐轮删光，
                // 剩下的弯曲邻接点又没有 _arc 标记，视觉和下发语义都会失真。
                break;
            }

            const Eigen::Vector3d incoming =
                records[previousIndex].point - records[previousIndex - 1].point;
            const Eigen::Vector3d outgoing =
                records[index].point - records[previousIndex].point;
            const double outgoingDistanceMm = outgoing.norm();

            if (outgoingDistanceMm > maxTrimDistanceMm)
            {
                break;
            }

            if (outgoingDistanceMm > 0.5
                && WeldPoseTurnAngleRad(incoming, outgoing) <= maxExitAngleRad)
            {
                break;
            }

            records.removeAt(previousIndex);
            --index;
            ++trimmedCount;
        }
    }
}

struct WeldCornerCandidateInfo
{
    bool isCandidate = false;
    bool markedCorner = false;
    bool explicitCorner = false;
    double theta = 0.0;
    Eigen::Vector3d incomingDir = Eigen::Vector3d::Zero();
    Eigen::Vector3d outgoingDir = Eigen::Vector3d::Zero();
};

struct WeldPolylineCutPoint
{
    bool valid = false;
    int segmentBeginIndex = -1;
    int segmentEndIndex = -1;
    double ratio = 0.0;
    WeldPoseFileRecord record;
};

int FindWeldPoseProbeIndexBefore(
    const QVector<WeldPoseFileRecord>& records,
    int cornerIndex,
    double probeDistanceMm)
{
    double accumulatedMm = 0.0;
    for (int index = cornerIndex; index > 0; --index)
    {
        accumulatedMm += (records[index].point - records[index - 1].point).norm();
        if (accumulatedMm >= probeDistanceMm)
        {
            return index - 1;
        }
    }
    return 0;
}

int FindWeldPoseProbeIndexAfter(
    const QVector<WeldPoseFileRecord>& records,
    int cornerIndex,
    double probeDistanceMm)
{
    double accumulatedMm = 0.0;
    for (int index = cornerIndex; index + 1 < records.size(); ++index)
    {
        accumulatedMm += (records[index + 1].point - records[index].point).norm();
        if (accumulatedMm >= probeDistanceMm)
        {
            return index + 1;
        }
    }
    return records.size() - 1;
}

WeldCornerCandidateInfo EvaluateWeldCornerCandidate(
    const QVector<WeldPoseFileRecord>& records,
    int index,
    double minMarkedCornerAngleRad,
    double maxCornerAngleRad,
    double minSegmentLengthMm,
    double directionProbeDistanceMm)
{
    WeldCornerCandidateInfo info;
    if (index <= 0 || index + 1 >= records.size())
    {
        return info;
    }

    const int prevProbeIndex = FindWeldPoseProbeIndexBefore(
        records,
        index,
        directionProbeDistanceMm);
    const int nextProbeIndex = FindWeldPoseProbeIndexAfter(
        records,
        index,
        directionProbeDistanceMm);
    if (prevProbeIndex >= index || nextProbeIndex <= index)
    {
        return info;
    }

    const WeldPoseFileRecord& prev = records[index - 1];
    const WeldPoseFileRecord& corner = records[index];
    const WeldPoseFileRecord& next = records[index + 1];
    const Eigen::Vector3d incoming = corner.point - records[prevProbeIndex].point;
    const Eigen::Vector3d outgoing = records[nextProbeIndex].point - corner.point;
    const double incomingLength = incoming.norm();
    const double outgoingLength = outgoing.norm();
    if (incomingLength <= minSegmentLengthMm || outgoingLength <= minSegmentLengthMm)
    {
        return info;
    }

    info.incomingDir = incoming / incomingLength;
    info.outgoingDir = outgoing / outgoingLength;
    const double cosTheta = std::clamp(info.incomingDir.dot(info.outgoingDir), -0.999999, 0.999999);
    info.theta = std::acos(cosTheta);
    info.explicitCorner = IsWeldCornerPointType(corner.pointType);
    info.markedCorner = info.explicitCorner
        || IsWeldSegmentBoundaryAtCorner(prev, corner, next);
    // 圆弧只能来自前序折线阶段确认过的明确拐点或段边界。
    // 普通补点即使因离散误差形成较大局部转角，也不能自行制造一个圆弧候选。
    info.isCandidate = info.markedCorner
        && info.theta >= minMarkedCornerAngleRad
        && info.theta <= maxCornerAngleRad
        && !corner.isLapStep;  // 搭接错位台阶端点：禁止圆角化，保住垂直 X 台阶
    return info;
}

double AccumulateWeldPoseDistanceBetween(
    const QVector<WeldPoseFileRecord>& records,
    int fromIndex,
    int toIndex)
{
    if (fromIndex == toIndex)
    {
        return 0.0;
    }

    const int beginIndex = std::min(fromIndex, toIndex);
    const int endIndex = std::max(fromIndex, toIndex);
    double distanceMm = 0.0;
    for (int index = beginIndex; index < endIndex; ++index)
    {
        distanceMm += (records[index + 1].point - records[index].point).norm();
    }
    return distanceMm;
}

void KeepBestWeldCornerCandidateRun(
    QVector<WeldCornerCandidateInfo>& candidates,
    const QVector<int>& runIndices)
{
    if (runIndices.size() <= 1)
    {
        return;
    }

    int bestIndex = runIndices.front();
    double bestScore = -1.0;
    for (const int index : runIndices)
    {
        const WeldCornerCandidateInfo& candidate = candidates[index];
        // 同一物理拐点附近可能同时出现“segmentKind 段边界”和明确 corner 点。
        // 必须优先保留明确 corner，不能让前面的普通边界点先生成圆弧后跳过真拐点。
        const double score = candidate.theta
            + (candidate.explicitCorner ? 100.0 : (candidate.markedCorner ? 10.0 : 0.0));
        if (score > bestScore)
        {
            bestScore = score;
            bestIndex = index;
        }
    }

    for (const int index : runIndices)
    {
        if (index != bestIndex)
        {
            candidates[index].isCandidate = false;
        }
    }
}

void SuppressShortBridgeWeldCornerCandidates(
    const QVector<WeldPoseFileRecord>& records,
    QVector<WeldCornerCandidateInfo>& candidates,
    double mergeDistanceMm)
{
    QVector<int> runIndices;
    int previousCandidate = -1;
    for (int index = 1; index + 1 < records.size(); ++index)
    {
        if (!candidates[index].isCandidate)
        {
            continue;
        }

        if (previousCandidate >= 0
            && AccumulateWeldPoseDistanceBetween(records, previousCandidate, index) > mergeDistanceMm)
        {
            KeepBestWeldCornerCandidateRun(candidates, runIndices);
            runIndices.clear();
        }

        runIndices.push_back(index);
        previousCandidate = index;
    }

    KeepBestWeldCornerCandidateRun(candidates, runIndices);
}

double AccumulateWeldPoseDistanceBackward(
    const QVector<WeldPoseFileRecord>& records,
    int beginIndex,
    int stopIndex)
{
    double distanceMm = 0.0;
    for (int index = beginIndex; index > stopIndex; --index)
    {
        distanceMm += (records[index].point - records[index - 1].point).norm();
    }
    return distanceMm;
}

double AccumulateWeldPoseDistanceForward(
    const QVector<WeldPoseFileRecord>& records,
    int beginIndex,
    int stopIndex)
{
    double distanceMm = 0.0;
    for (int index = beginIndex; index < stopIndex; ++index)
    {
        distanceMm += (records[index + 1].point - records[index].point).norm();
    }
    return distanceMm;
}

struct WeldEndpointTrimStats
{
    int removedStartCount = 0;
    int removedEndCount = 0;
};

void TrimWeldPoseRecordEndpoints(
    const WeldPosePreset& preset,
    QVector<WeldPoseFileRecord>& records,
    WeldEndpointTrimStats& stats)
{
    stats = {};
    if (records.size() < 2)
    {
        return;
    }

    QVector<double> distanceFromStart(records.size(), 0.0);
    for (int index = 1; index < records.size(); ++index)
    {
        distanceFromStart[index] = distanceFromStart[index - 1]
            + (records[index].point - records[index - 1].point).norm();
    }

    QVector<double> distanceFromEnd(records.size(), 0.0);
    for (int index = records.size() - 2; index >= 0; --index)
    {
        distanceFromEnd[index] = distanceFromEnd[index + 1]
            + (records[index + 1].point - records[index].point).norm();
    }

    int startIndex = 0;
    if (preset.weldStartSkipDistance > 1e-6)
    {
        for (int index = 0; index < records.size(); ++index)
        {
            if (distanceFromStart[index] >= preset.weldStartSkipDistance)
            {
                startIndex = index;
                break;
            }
        }
    }

    // 起点不要落在过渡段内部；跳到过渡段结束后的第一个稳定点。
    while (startIndex + 1 < records.size() && IsWeldPoseTransitionRecord(records[startIndex]))
    {
        ++startIndex;
    }

    int endIndex = records.size() - 1;
    if (preset.weldEndSkipDistance > 1e-6)
    {
        for (int index = records.size() - 1; index >= 0; --index)
        {
            if (distanceFromEnd[index] >= preset.weldEndSkipDistance)
            {
                endIndex = index;
                break;
            }
        }
    }

    // 终点同样避开过渡段内部，退回到过渡段开始前的稳定点。
    while (endIndex > startIndex && IsWeldPoseTransitionRecord(records[endIndex]))
    {
        --endIndex;
    }

    if (startIndex <= 0 && endIndex >= records.size() - 1)
    {
        return;
    }

    if (startIndex > endIndex)
    {
        stats.removedStartCount = records.size();
        records.clear();
        return;
    }

    stats.removedStartCount = startIndex;
    stats.removedEndCount = records.size() - 1 - endIndex;
    records = records.mid(startIndex, endIndex - startIndex + 1);
    RenumberWeldPoseRecords(records);
}

bool TryFindWeldPoseCutBefore(
    const QVector<WeldPoseFileRecord>& records,
    int cornerIndex,
    int minIndex,
    double targetDistanceMm,
    WeldPolylineCutPoint& cutPoint)
{
    constexpr double kEpsilon = 1e-9;
    if (targetDistanceMm <= kEpsilon || cornerIndex <= minIndex)
    {
        return false;
    }

    double remainingDistanceMm = targetDistanceMm;
    for (int index = cornerIndex; index > minIndex; --index)
    {
        const double segmentLengthMm = (records[index].point - records[index - 1].point).norm();
        if (segmentLengthMm <= kEpsilon)
        {
            continue;
        }

        if (remainingDistanceMm <= segmentLengthMm + kEpsilon)
        {
            const double ratio = std::clamp(
                (segmentLengthMm - remainingDistanceMm) / segmentLengthMm,
                0.0,
                1.0);
            cutPoint.valid = true;
            cutPoint.segmentBeginIndex = index - 1;
            cutPoint.segmentEndIndex = index;
            cutPoint.ratio = ratio;
            cutPoint.record = InterpolateWeldPoseRecord(records[index - 1], records[index], ratio);
            return true;
        }

        remainingDistanceMm -= segmentLengthMm;
    }

    return false;
}

bool TryFindWeldPoseCutAfter(
    const QVector<WeldPoseFileRecord>& records,
    int cornerIndex,
    int maxIndex,
    double targetDistanceMm,
    WeldPolylineCutPoint& cutPoint)
{
    constexpr double kEpsilon = 1e-9;
    if (targetDistanceMm <= kEpsilon || cornerIndex >= maxIndex)
    {
        return false;
    }

    double remainingDistanceMm = targetDistanceMm;
    for (int index = cornerIndex; index < maxIndex; ++index)
    {
        const double segmentLengthMm = (records[index + 1].point - records[index].point).norm();
        if (segmentLengthMm <= kEpsilon)
        {
            continue;
        }

        if (remainingDistanceMm <= segmentLengthMm + kEpsilon)
        {
            const double ratio = std::clamp(
                remainingDistanceMm / segmentLengthMm,
                0.0,
                1.0);
            cutPoint.valid = true;
            cutPoint.segmentBeginIndex = index;
            cutPoint.segmentEndIndex = index + 1;
            cutPoint.ratio = ratio;
            cutPoint.record = InterpolateWeldPoseRecord(records[index], records[index + 1], ratio);
            return true;
        }

        remainingDistanceMm -= segmentLengthMm;
    }

    return false;
}

WeldCornerArcApplyStats ApplyCornerArcTransitionToWeldPoseRecords(
    const WeldPosePreset& preset,
    QVector<WeldPoseFileRecord>& records)
{
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kMinMarkedCornerAngleRad = 5.0 * kPi / 180.0;
    constexpr double kMaxCornerAngleRad = 178.0 * kPi / 180.0;
    constexpr double kMinEnabledArcRadiusMm = 2.0;
    constexpr double kMinFeasibleArcRadiusMm = 0.25;
    constexpr double kMinSegmentLengthMm = 1e-6;

    WeldCornerArcApplyStats stats;
    stats.inputPointCount = records.size();
    stats.outputPointCount = records.size();
    const double effectiveRadiusMm = preset.cornerArcRadiusMm > 0.0
        ? std::max(kMinEnabledArcRadiusMm, preset.cornerArcRadiusMm)
        : 0.0;
    stats.radiusMm = effectiveRadiusMm;

    if (records.size() < 3 || effectiveRadiusMm <= 0.0)
    {
        return stats;
    }

    const double sampleStepMm = EstimateWeldPoseStepMm(records);
    const double directionProbeDistanceMm = std::clamp(
        std::max(sampleStepMm * 1.5, effectiveRadiusMm * 0.25),
        1.0,
        5.0);
    const double maxEntryTrimAngleRad = 8.0 * kPi / 180.0;
    const double maxEntryTrimDistanceMm = std::max(
        2.0,
        std::max(sampleStepMm * 2.5, effectiveRadiusMm * 0.35));
    const double shortBridgeMergeDistanceMm = std::max({
        2.0,
        sampleStepMm * 3.0,
        effectiveRadiusMm * 2.0
    });
    QVector<WeldCornerCandidateInfo> candidates(records.size());

    for (int index = 1; index + 1 < records.size(); ++index)
    {
        candidates[index] = EvaluateWeldCornerCandidate(
            records,
            index,
            kMinMarkedCornerAngleRad,
            kMaxCornerAngleRad,
            kMinSegmentLengthMm,
            directionProbeDistanceMm);
    }

    SuppressShortBridgeWeldCornerCandidates(
        records,
        candidates,
        shortBridgeMergeDistanceMm);
    for (const WeldCornerCandidateInfo& candidate : candidates)
    {
        if (candidate.isCandidate)
        {
            ++stats.candidateCornerCount;
        }
    }

    QVector<int> previousCandidateIndex(records.size(), 0);
    QVector<int> nextCandidateIndex(records.size(), records.size() - 1);

    int lastCandidateIndex = 0;
    for (int index = 1; index + 1 < records.size(); ++index)
    {
        previousCandidateIndex[index] = lastCandidateIndex;
        if (candidates[index].isCandidate)
        {
            lastCandidateIndex = index;
        }
    }

    lastCandidateIndex = records.size() - 1;
    for (int index = records.size() - 2; index > 0; --index)
    {
        nextCandidateIndex[index] = lastCandidateIndex;
        if (candidates[index].isCandidate)
        {
            lastCandidateIndex = index;
        }
    }

    QVector<WeldPoseFileRecord> roundedRecords;
    roundedRecords.reserve(records.size() + records.size() / 4);
    int copyIndex = 0;

    for (int index = 1; index + 1 < records.size(); ++index)
    {
        const WeldCornerCandidateInfo& candidate = candidates[index];
        if (!candidate.isCandidate || index <= copyIndex)
        {
            continue;
        }

        const WeldPoseFileRecord& corner = records[index];

        const double tanHalf = std::tan(candidate.theta * 0.5);
        if (!std::isfinite(tanHalf) || std::abs(tanHalf) <= 1e-9)
        {
            continue;
        }

        // theta is the path deflection angle. The fillet tangent distance is
        // r * tan(theta / 2), not r / tan(theta / 2).
        double tangentDistanceMm = effectiveRadiusMm * tanHalf;
        const int incomingLimitIndex = std::max(previousCandidateIndex[index], copyIndex);
        const int outgoingLimitIndex = nextCandidateIndex[index];
        const double incomingAvailableMm = AccumulateWeldPoseDistanceBackward(
            records,
            index,
            incomingLimitIndex);
        const double outgoingAvailableMm = AccumulateWeldPoseDistanceForward(
            records,
            index,
            outgoingLimitIndex);
        const double maxTangentDistanceMm = std::min(incomingAvailableMm, outgoingAvailableMm) * 0.45;
        if (maxTangentDistanceMm <= kMinSegmentLengthMm)
        {
            continue;
        }
        if (tangentDistanceMm > maxTangentDistanceMm)
        {
            tangentDistanceMm = maxTangentDistanceMm;
        }

        const double actualRadiusMm = tangentDistanceMm / tanHalf;
        if (!std::isfinite(actualRadiusMm) || actualRadiusMm < kMinFeasibleArcRadiusMm)
        {
            continue;
        }

        WeldPolylineCutPoint tangentInCut;
        WeldPolylineCutPoint tangentOutCut;
        if (!TryFindWeldPoseCutBefore(
                records,
                index,
                incomingLimitIndex,
                tangentDistanceMm,
                tangentInCut)
            || !TryFindWeldPoseCutAfter(
                records,
                index,
                outgoingLimitIndex,
                tangentDistanceMm,
                tangentOutCut)
            || !tangentInCut.valid
            || !tangentOutCut.valid)
        {
            continue;
        }

        const double cosTheta = std::clamp(
            candidate.incomingDir.dot(candidate.outgoingDir),
            -0.999999,
            0.999999);
        const Eigen::Vector3d normalComponent =
            candidate.outgoingDir - candidate.incomingDir * cosTheta;
        const double normalLength = normalComponent.norm();
        if (normalLength <= 1e-9)
        {
            continue;
        }

        const Eigen::Vector3d planeX = candidate.incomingDir;
        const Eigen::Vector3d planeY = normalComponent / normalLength;
        const Eigen::Vector3d tangentIn = tangentInCut.record.point;
        const Eigen::Vector3d tangentOut = tangentOutCut.record.point;
        const Eigen::Vector3d center = tangentIn + planeY * actualRadiusMm;
        const Eigen::Vector3d startVector = tangentIn - center;
        const double startAngle = std::atan2(startVector.dot(planeY), startVector.dot(planeX));
        const double arcLengthMm = actualRadiusMm * candidate.theta;
        const int stepCount = std::max(2, static_cast<int>(std::ceil(arcLengthMm / sampleStepMm)));
        const QString arcSegmentKind = WeldArcSegmentKind(corner.segmentKind);
        const QString arcPointType = corner.pointType.trimmed().isEmpty()
            ? QStringLiteral("arc")
            : (corner.pointType + "_arc");

        while (copyIndex <= tangentInCut.segmentBeginIndex)
        {
            AppendWeldPoseRecordIfNotDuplicate(roundedRecords, records[copyIndex]);
            ++copyIndex;
        }
        // 圆弧切入点前如果残留一个很近的尖角点，会让视觉上仍像没有圆滑过渡。
        TrimSharpEntryPointBeforeWeldArc(
            roundedRecords,
            tangentIn,
            maxEntryTrimAngleRad,
            maxEntryTrimDistanceMm);

        for (int arcIndex = 0; arcIndex <= stepCount; ++arcIndex)
        {
            const double ratio = static_cast<double>(arcIndex) / static_cast<double>(stepCount);
            WeldPoseFileRecord arcRecord = InterpolateWeldPoseRecord(
                tangentInCut.record,
                tangentOutCut.record,
                ratio);
            const double angle = startAngle + candidate.theta * ratio;
            arcRecord.point = center
                + planeX * (std::cos(angle) * actualRadiusMm)
                + planeY * (std::sin(angle) * actualRadiusMm);
            if (arcIndex == stepCount)
            {
                arcRecord.point = tangentOut;
            }
            else if (arcIndex == 0)
            {
                arcRecord.point = tangentIn;
            }
            arcRecord.pointType = arcPointType;
            arcRecord.segmentKind = arcSegmentKind;
            AppendWeldPoseRecordIfNotDuplicate(roundedRecords, arcRecord);
        }

        copyIndex = std::max(copyIndex, tangentOutCut.segmentEndIndex);
        ++stats.roundedCornerCount;
        if (actualRadiusMm + 1e-6 < effectiveRadiusMm)
        {
            ++stats.reducedRadiusCornerCount;
        }
        if (stats.minimumAppliedRadiusMm <= 0.0
            || actualRadiusMm < stats.minimumAppliedRadiusMm)
        {
            stats.minimumAppliedRadiusMm = actualRadiusMm;
        }
    }

    while (copyIndex < records.size())
    {
        AppendWeldPoseRecordIfNotDuplicate(roundedRecords, records[copyIndex]);
        ++copyIndex;
    }

    TrimSharpWeldArcEntryPoints(
        roundedRecords,
        maxEntryTrimAngleRad,
        maxEntryTrimDistanceMm);

    records = std::move(roundedRecords);
    RenumberWeldPoseRecords(records);
    stats.outputPointCount = records.size();
    return stats;
}

// 独立调试预览(不参与主流程，不影响任何实际焊接文件)：只接受已经完成
// 姿态补偿、焊道补偿和最终圆弧过渡的记录，按 _arc 标记上色。
// 禁止从分类点提前自行生成圆弧，否则目录中的中间产物会与真实阶段顺序相互矛盾。
std::vector<QString> BuildArcTransitionPreviewCloudLines(
    const QVector<WeldPoseFileRecord>& records)
{
    std::vector<QString> cloud;
    cloud.push_back(QStringLiteral(
        "// X Y Z R G B  最终后处理结果：绿=显式圆弧过渡段 灰=折线段"));
    for (const WeldPoseFileRecord& record : records)
    {
        const bool isArc = record.pointType.contains(QStringLiteral("arc"))
            || record.segmentKind.contains(QStringLiteral("_arc"));
        const QString rgb = isArc ? QStringLiteral("0 255 0") : QStringLiteral("120 120 120");
        cloud.push_back(QString("%1 %2 %3 %4")
            .arg(record.point.x(), 0, 'f', 6)
            .arg(record.point.y(), 0, 'f', 6)
            .arg(record.point.z(), 0, 'f', 6)
            .arg(rgb));
    }
    return cloud;
}

int DensifyWeldPoseRecordsByStep(
    QVector<WeldPoseFileRecord>& records,
    double maxStepMm)
{
    if (records.size() < 2 || !std::isfinite(maxStepMm) || maxStepMm <= 0.0)
    {
        return 0;
    }

    QVector<WeldPoseFileRecord> denseRecords;
    denseRecords.reserve(records.size());
    denseRecords.push_back(records.front());

    int insertedPointCount = 0;
    for (int index = 1; index < records.size(); ++index)
    {
        const WeldPoseFileRecord& prev = records[index - 1];
        const WeldPoseFileRecord& next = records[index];
        const double distanceMm = (next.point - prev.point).norm();
        if (!std::isfinite(distanceMm) || distanceMm <= 1e-6)
        {
            AppendWeldPoseRecordIfNotDuplicate(denseRecords, next);
            continue;
        }

        const int segmentCount = std::max(
            1,
            static_cast<int>(std::ceil(distanceMm / maxStepMm)));
        for (int segmentIndex = 1; segmentIndex < segmentCount; ++segmentIndex)
        {
            const double ratio = static_cast<double>(segmentIndex)
                / static_cast<double>(segmentCount);
            WeldPoseFileRecord fillRecord = InterpolateWeldPoseRecord(prev, next, ratio);
            // 2mm 加密的填充点一律标 normal：不继承端点(尤其 corner/_arc)的类型，否则单点拐角会被沿线
            // 染成一串假 corner，下游把直线段中部误当拐角顶点。段类 segmentKind 仍保留以维持段/弧归属。
            fillRecord.pointType =
                RobotCalculation::LowerWeldPointTypeName(RobotCalculation::LowerWeldPointType::Normal);
            fillRecord.segmentKind = next.segmentKind;
            AppendWeldPoseRecordIfNotDuplicate(denseRecords, fillRecord);
            ++insertedPointCount;
        }
        AppendWeldPoseRecordIfNotDuplicate(denseRecords, next);
    }

    if (insertedPointCount > 0)
    {
        records = std::move(denseRecords);
        RenumberWeldPoseRecords(records);
    }
    return insertedPointCount;
}

int SmoothRemainingUnroundedWeldCorners(
    QVector<WeldPoseFileRecord>& records,
    double maxStepMm)
{
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kMinSmoothAngleRad = 35.0 * kPi / 180.0;
    if (records.size() < 3)
    {
        return 0;
    }

    const double safeStepMm = std::clamp(maxStepMm, 0.5, 5.0);
    const auto hasNearbyTransitionKind =
        [&records](int centerIndex, int radius) -> bool
    {
        const int beginIndex = std::max(0, centerIndex - radius);
        const int endIndex = std::min(
            static_cast<int>(records.size()) - 1,
            centerIndex + radius);
        for (int index = beginIndex; index <= endIndex; ++index)
        {
            if (records[index].segmentKind.contains(
                    QStringLiteral("_transition"),
                    Qt::CaseInsensitive))
            {
                return true;
            }
        }
        return false;
    };

    int smoothedCount = 0;
    for (int index = 1; index + 1 < records.size(); ++index)
    {
        const bool isMarkedCorner = IsWeldCornerPointType(records[index].pointType);
        const bool isResidualTransitionCorner =
            !isMarkedCorner
            && records[index].pointType.compare(QStringLiteral("normal"), Qt::CaseInsensitive) == 0
            && hasNearbyTransitionKind(index, 2);
        if (records[index].isLapStep   // 搭接错位台阶端点：禁止平滑/桥接，保住垂直 X 台阶
            || IsWeldPoseArcRecord(records[index])
            || (!isMarkedCorner && !isResidualTransitionCorner))
        {
            continue;
        }

        const Eigen::Vector3d incoming = records[index].point - records[index - 1].point;
        const Eigen::Vector3d outgoing = records[index + 1].point - records[index].point;
        if (WeldPoseTurnAngleRad(incoming, outgoing) < kMinSmoothAngleRad)
        {
            continue;
        }

        const WeldPoseFileRecord corner = records[index];
        const WeldPoseFileRecord before = records[index - 1];
        const WeldPoseFileRecord after = records[index + 1];
        const double distanceMm = (after.point - before.point).norm();
        if (!std::isfinite(distanceMm) || distanceMm <= 1e-6 || distanceMm > safeStepMm * 5.0)
        {
            continue;
        }

        const int segmentCount = std::max(
            2,
            static_cast<int>(std::ceil(distanceMm / safeStepMm)));
        QVector<WeldPoseFileRecord> bridgeRecords;
        bridgeRecords.reserve(std::max(0, segmentCount - 1));
        for (int segmentIndex = 1; segmentIndex < segmentCount; ++segmentIndex)
        {
            const double ratio = static_cast<double>(segmentIndex)
                / static_cast<double>(segmentCount);
            WeldPoseFileRecord bridgeRecord = InterpolateWeldPoseRecord(before, after, ratio);
            bridgeRecord.rawIndex = corner.rawIndex;
            bridgeRecord.pointType = isMarkedCorner
                ? (corner.pointType + "_arc")
                : QStringLiteral("normal_arc");
            bridgeRecord.segmentKind = WeldArcSegmentKind(corner.segmentKind);
            bridgeRecords.push_back(bridgeRecord);
        }

        records.erase(records.begin() + index);
        for (int bridgeIndex = 0; bridgeIndex < bridgeRecords.size(); ++bridgeIndex)
        {
            records.insert(index + bridgeIndex, bridgeRecords[bridgeIndex]);
        }
        index += std::max(0, static_cast<int>(bridgeRecords.size()) - 1);
        ++smoothedCount;
    }

    if (smoothedCount > 0)
    {
        RenumberWeldPoseRecords(records);
    }
    return smoothedCount;
}

struct WeldCornerRestoreStats
{
    int missingCornerCount = 0;
    int restoredCornerCount = 0;
    int adjustedCornerCount = 0;
    int skippedCornerCount = 0;
};

QString WeldCornerTypeKey(const QString& pointType)
{
    const QString normalized = pointType.trimmed().toLower();
    if (normalized.contains("inner") || normalized.contains(QStringLiteral("内")))
    {
        return QStringLiteral("inner");
    }
    if (normalized.contains("outer") || normalized.contains(QStringLiteral("外")))
    {
        return QStringLiteral("outer");
    }
    if (IsWeldCornerPointType(pointType))
    {
        return QStringLiteral("corner");
    }
    return QString();
}

bool IsSameWeldCornerType(const QString& lhs, const QString& rhs)
{
    const QString lhsKey = WeldCornerTypeKey(lhs);
    const QString rhsKey = WeldCornerTypeKey(rhs);
    return !lhsKey.isEmpty() && lhsKey == rhsKey;
}

int FindMatchingWeldCornerRecordIndex(
    const QVector<WeldPoseFileRecord>& records,
    const WeldPoseFileRecord& corner)
{
    for (int index = 0; index < records.size(); ++index)
    {
        const WeldPoseFileRecord& record = records[index];
        if (record.rawIndex == corner.rawIndex
            && IsSameWeldCornerType(record.pointType, corner.pointType))
        {
            return index;
        }
    }
    return -1;
}

int FindWeldCornerInsertionIndex(
    const QVector<WeldPoseFileRecord>& records,
    const WeldPoseFileRecord& corner)
{
    for (int index = 0; index < records.size(); ++index)
    {
        if (records[index].rawIndex >= corner.rawIndex)
        {
            return index;
        }
    }
    return records.size();
}

QVector<int> CollectSameKindLineWindowBefore(
    const QVector<WeldPoseFileRecord>& records,
    int endIndex,
    int maxPointCount)
{
    QVector<int> indices;
    if (endIndex < 0 || endIndex >= records.size())
    {
        return indices;
    }

    const QString targetKind = NormalizeWeldSegmentKind(records[endIndex].segmentKind)
        .trimmed()
        .toLower();
    for (int index = endIndex; index >= 0 && indices.size() < maxPointCount; --index)
    {
        if (records[index].isLapStep)
        {
            continue;  // 台阶端点侧向跳变会污染平台直线拟合方向，排除出拟合窗口
        }
        const QString kind = NormalizeWeldSegmentKind(records[index].segmentKind)
            .trimmed()
            .toLower();
        if (!targetKind.isEmpty() && kind != targetKind && indices.size() >= 2)
        {
            break;
        }
        indices.push_front(index);
    }
    return indices;
}

QVector<int> CollectSameKindLineWindowAfter(
    const QVector<WeldPoseFileRecord>& records,
    int beginIndex,
    int maxPointCount)
{
    QVector<int> indices;
    if (beginIndex < 0 || beginIndex >= records.size())
    {
        return indices;
    }

    const QString targetKind = NormalizeWeldSegmentKind(records[beginIndex].segmentKind)
        .trimmed()
        .toLower();
    for (int index = beginIndex; index < records.size() && indices.size() < maxPointCount; ++index)
    {
        if (records[index].isLapStep)
        {
            continue;  // 台阶端点侧向跳变会污染平台直线拟合方向，排除出拟合窗口
        }
        const QString kind = NormalizeWeldSegmentKind(records[index].segmentKind)
            .trimmed()
            .toLower();
        if (!targetKind.isEmpty() && kind != targetKind && indices.size() >= 2)
        {
            break;
        }
        indices.push_back(index);
    }
    return indices;
}

bool TryFitWeldPoseLine2D(
    const QVector<WeldPoseFileRecord>& records,
    const QVector<int>& indices,
    Eigen::Vector2d& point,
    Eigen::Vector2d& direction)
{
    if (indices.size() < 2)
    {
        return false;
    }

    point = Eigen::Vector2d::Zero();
    int validCount = 0;
    for (const int index : indices)
    {
        if (index < 0 || index >= records.size())
        {
            continue;
        }
        point += Eigen::Vector2d(records[index].point.x(), records[index].point.y());
        ++validCount;
    }
    if (validCount < 2)
    {
        return false;
    }
    point /= static_cast<double>(validCount);

    Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
    for (const int index : indices)
    {
        if (index < 0 || index >= records.size())
        {
            continue;
        }
        const Eigen::Vector2d delta =
            Eigen::Vector2d(records[index].point.x(), records[index].point.y()) - point;
        covariance += delta * delta.transpose();
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(covariance);
    if (solver.info() != Eigen::Success || solver.eigenvalues()(1) <= 1e-8)
    {
        return false;
    }

    direction = solver.eigenvectors().col(1).normalized();
    return direction.norm() > 1e-9;
}

double Cross2D(const Eigen::Vector2d& lhs, const Eigen::Vector2d& rhs)
{
    return lhs.x() * rhs.y() - lhs.y() * rhs.x();
}

bool TryIntersectWeldPoseLines2D(
    const Eigen::Vector2d& firstPoint,
    const Eigen::Vector2d& firstDirection,
    const Eigen::Vector2d& secondPoint,
    const Eigen::Vector2d& secondDirection,
    Eigen::Vector2d& intersection)
{
    const double denominator = Cross2D(firstDirection, secondDirection);
    if (std::abs(denominator) <= 1e-8)
    {
        return false;
    }

    const Eigen::Vector2d delta = secondPoint - firstPoint;
    const double firstRatio = Cross2D(delta, secondDirection) / denominator;
    intersection = firstPoint + firstDirection * firstRatio;
    return intersection.allFinite();
}

bool TryBuildLineIntersectionWeldCorner(
    const WeldPoseFileRecord& corner,
    const QVector<WeldPoseFileRecord>& records,
    int beforeEndIndex,
    int afterBeginIndex,
    WeldPoseFileRecord& restoredCorner)
{
    if (records.size() < 4)
    {
        return false;
    }

    if (beforeEndIndex < 0
        || beforeEndIndex >= records.size()
        || afterBeginIndex < 0
        || afterBeginIndex >= records.size()
        || beforeEndIndex >= afterBeginIndex)
    {
        return false;
    }

    const QVector<int> beforeIndices = CollectSameKindLineWindowBefore(
        records,
        beforeEndIndex,
        6);
    const QVector<int> afterIndices = CollectSameKindLineWindowAfter(
        records,
        afterBeginIndex,
        6);

    Eigen::Vector2d firstPoint;
    Eigen::Vector2d firstDirection;
    Eigen::Vector2d secondPoint;
    Eigen::Vector2d secondDirection;
    if (!TryFitWeldPoseLine2D(records, beforeIndices, firstPoint, firstDirection)
        || !TryFitWeldPoseLine2D(records, afterIndices, secondPoint, secondDirection))
    {
        return false;
    }

    Eigen::Vector2d intersection;
    if (!TryIntersectWeldPoseLines2D(
            firstPoint,
            firstDirection,
            secondPoint,
            secondDirection,
            intersection))
    {
        return false;
    }

    const WeldPoseFileRecord& before = records[beforeEndIndex];
    const WeldPoseFileRecord& after = records[afterBeginIndex];
    const Eigen::Vector2d beforePoint(before.point.x(), before.point.y());
    const Eigen::Vector2d afterPoint(after.point.x(), after.point.y());
    const Eigen::Vector2d span = afterPoint - beforePoint;
    const double spanLength = span.norm();
    const double distanceToBefore = (intersection - beforePoint).norm();
    const double distanceToAfter = (intersection - afterPoint).norm();
    const double maxAllowedDistance = std::max(20.0, spanLength * 6.0);
    const Eigen::Vector2d originalCornerPoint(corner.point.x(), corner.point.y());
    if (distanceToBefore > maxAllowedDistance
        || distanceToAfter > maxAllowedDistance
        || (intersection - originalCornerPoint).norm() > std::max(25.0, spanLength * 8.0))
    {
        return false;
    }

    double ratio = 0.5;
    if (span.squaredNorm() > 1e-9)
    {
        ratio = std::clamp((intersection - beforePoint).dot(span) / span.squaredNorm(), 0.0, 1.0);
    }

    restoredCorner = InterpolateWeldPoseRecord(before, after, ratio);
    restoredCorner.rawIndex = corner.rawIndex;
    restoredCorner.point.x() = intersection.x();
    restoredCorner.point.y() = intersection.y();
    restoredCorner.pointType = corner.pointType;
    restoredCorner.segmentKind = corner.segmentKind;
    if ((restoredCorner.point - before.point).norm() <= 1e-6
        || (restoredCorner.point - after.point).norm() <= 1e-6)
    {
        return false;
    }

    return true;
}

bool TryInsertRestoredWeldCorner(
    const WeldPoseFileRecord& corner,
    QVector<WeldPoseFileRecord>& records)
{
    const int insertionIndex = FindWeldCornerInsertionIndex(records, corner);
    if (insertionIndex <= 0 || insertionIndex >= records.size())
    {
        return false;
    }

    WeldPoseFileRecord restoredCorner;
    if (!TryBuildLineIntersectionWeldCorner(
            corner,
            records,
            insertionIndex - 1,
            insertionIndex,
            restoredCorner))
    {
        return false;
    }

    records.insert(insertionIndex, restoredCorner);
    return true;
}

bool TryAdjustExistingWeldCornerByIntersection(
    const WeldPoseFileRecord& sourceCorner,
    int cornerIndex,
    QVector<WeldPoseFileRecord>& records)
{
    if (cornerIndex <= 0 || cornerIndex + 1 >= records.size())
    {
        return false;
    }

    constexpr double kPi = 3.14159265358979323846;
    const double currentTurnRad = WeldPoseTurnAngleRad(
        records[cornerIndex].point - records[cornerIndex - 1].point,
        records[cornerIndex + 1].point - records[cornerIndex].point);
    if (currentTurnRad < 35.0 * kPi / 180.0)
    {
        return false;
    }

    WeldPoseFileRecord restoredCorner;
    if (!TryBuildLineIntersectionWeldCorner(
            sourceCorner,
            records,
            cornerIndex - 1,
            cornerIndex + 1,
            restoredCorner))
    {
        return false;
    }

    if ((restoredCorner.point - records[cornerIndex].point).norm() <= 1e-6)
    {
        return false;
    }

    records[cornerIndex] = restoredCorner;
    return true;
}

WeldCornerRestoreStats RestoreTrimmedWeldCornersByLineIntersection(
    const QVector<WeldPoseFileRecord>& recordsBeforeTrim,
    QVector<WeldPoseFileRecord>& records)
{
    WeldCornerRestoreStats stats;
    if (recordsBeforeTrim.isEmpty() || records.size() < 4)
    {
        return stats;
    }

    int minRawIndex = std::numeric_limits<int>::max();
    int maxRawIndex = std::numeric_limits<int>::min();
    for (const WeldPoseFileRecord& record : records)
    {
        minRawIndex = std::min(minRawIndex, record.rawIndex);
        maxRawIndex = std::max(maxRawIndex, record.rawIndex);
    }

    for (const WeldPoseFileRecord& corner : recordsBeforeTrim)
    {
        if (!IsWeldCornerPointType(corner.pointType)
            || corner.isLapStep   // 搭接错位台阶端点：两侧平行线求交无意义，跳过交点重建
            || corner.rawIndex < minRawIndex
            || corner.rawIndex > maxRawIndex)
        {
            continue;
        }

        const int existingCornerIndex = FindMatchingWeldCornerRecordIndex(records, corner);
        if (existingCornerIndex >= 0)
        {
            if (TryAdjustExistingWeldCornerByIntersection(corner, existingCornerIndex, records))
            {
                ++stats.adjustedCornerCount;
                RenumberWeldPoseRecords(records);
            }
            continue;
        }

        ++stats.missingCornerCount;
        if (TryInsertRestoredWeldCorner(corner, records))
        {
            ++stats.restoredCornerCount;
            RenumberWeldPoseRecords(records);
        }
        else
        {
            ++stats.skippedCornerCount;
        }
    }
    return stats;
}

struct PoseCompSegmentRange
{
    int begin = -1;
    int end = -1;
    QString kind;
};

struct PoseCompJunctionApplyStats
{
    int adjustedJunctionCount = 0;
    int removedPointCount = 0;
};

QVector<PoseCompSegmentRange> CollectPoseCompSegmentRanges(
    const QVector<WeldPoseFileRecord>& records)
{
    QVector<PoseCompSegmentRange> ranges;
    if (records.isEmpty())
    {
        return ranges;
    }

    auto normalizedKindAt = [&records](int index) -> QString
    {
        QString kind = NormalizeWeldSegmentKind(records[index].segmentKind)
            .trimmed()
            .toLower();
        if (kind.isEmpty())
        {
            kind = records[index].segmentKind.trimmed().toLower();
        }
        return kind;
    };

    PoseCompSegmentRange current;
    current.begin = 0;
    current.end = 0;
    current.kind = normalizedKindAt(0);
    for (int index = 1; index < records.size(); ++index)
    {
        const QString kind = normalizedKindAt(index);
        if (kind.compare(current.kind, Qt::CaseInsensitive) == 0)
        {
            current.end = index;
            continue;
        }

        ranges.push_back(current);
        current.begin = index;
        current.end = index;
        current.kind = kind;
    }
    ranges.push_back(current);
    return ranges;
}

struct PoseCompReferencePose
{
    double rx = 0.0;
    double ry = 0.0;
    double rz = 0.0;
};

constexpr double kPoseCompOutputStepMm = 2.0;

QVector<PoseCompReferencePose> ResolvePoseCompReferencePoses(
    const QVector<WeldPoseFileRecord>& records)
{
    QVector<PoseCompReferencePose> references(records.size());
    const QVector<PoseCompSegmentRange> ranges = CollectPoseCompSegmentRanges(records);
    for (const PoseCompSegmentRange& range : ranges)
    {
        if (range.begin < 0 || range.end < range.begin || range.end >= records.size())
        {
            continue;
        }

        // 生成文件把姿态渐变点标为 *_transition。优先取同一物理段内第一个
        // 非渐变点作为固定参考；极短段全部处于渐变区时，则取段首姿态。
        int referenceIndex = range.begin;
        for (int index = range.begin; index <= range.end; ++index)
        {
            const QString kind = records[index].segmentKind.trimmed().toLower();
            if (!kind.endsWith(QStringLiteral("_transition"))
                && !kind.endsWith(QStringLiteral("_arc")))
            {
                referenceIndex = index;
                break;
            }
        }

        const PoseCompReferencePose reference{
            records[referenceIndex].rx,
            records[referenceIndex].ry,
            records[referenceIndex].rz
        };
        for (int index = range.begin; index <= range.end; ++index)
        {
            references[index] = reference;
        }
    }
    return references;
}

QVector<Eigen::Vector3d> ResolvePoseCompSegmentWeldNormals(
    const QVector<WeldPoseFileRecord>& records,
    const QVector<PoseCompReferencePose>& referencePoses)
{
    QVector<Eigen::Vector3d> normals(records.size(), Eigen::Vector3d::Zero());
    if (referencePoses.size() != records.size())
    {
        return normals;
    }

    const QVector<PoseCompSegmentRange> ranges = CollectPoseCompSegmentRanges(records);
    for (const PoseCompSegmentRange& range : ranges)
    {
        const bool isPhysicalSegment =
            IsPlatformSegmentKind(range.kind) || IsSlopeSegmentKind(range.kind);
        if (!isPhysicalSegment
            || range.begin < 0
            || range.end <= range.begin
            || range.end >= records.size())
        {
            continue;
        }

        const Eigen::Vector3d segmentVector =
            records[range.end].point - records[range.begin].point;
        if (HorizontalUnitOrZero(segmentVector).head<2>().norm() <= 1e-9)
        {
            continue;
        }

        const PoseCompReferencePose& reference = referencePoses[range.begin];
        const double normalRz = ComputeLineNormalRobotRz(
            segmentVector,
            reference.ry,
            reference.rz);
        const Eigen::Vector3d weldNormal =
            HorizontalUnitOrZero(GunDirectionVectorFromRobotRz(normalRz));
        for (int index = range.begin; index <= range.end; ++index)
        {
            normals[index] = weldNormal;
        }
    }
    return normals;
}

bool ShouldRebuildPoseCompJunction(const QString& leftKind, const QString& rightKind)
{
    if (leftKind.compare(rightKind, Qt::CaseInsensitive) == 0)
    {
        return false;
    }

    const bool leftPlatform = IsPlatformSegmentKind(leftKind);
    const bool rightPlatform = IsPlatformSegmentKind(rightKind);
    const bool leftSlope = IsSlopeSegmentKind(leftKind);
    const bool rightSlope = IsSlopeSegmentKind(rightKind);
    return (leftPlatform && rightSlope) || (leftSlope && rightPlatform);
}

Eigen::Vector2d WeldPosePoint2D(const WeldPoseFileRecord& record)
{
    return Eigen::Vector2d(record.point.x(), record.point.y());
}

Eigen::Vector2d SegmentTravelDirection2D(
    const QVector<WeldPoseFileRecord>& records,
    const PoseCompSegmentRange& range,
    const Eigen::Vector2d& fallbackDirection)
{
    if (range.begin >= 0
        && range.end >= range.begin
        && range.end < records.size())
    {
        const Eigen::Vector2d delta =
            WeldPosePoint2D(records[range.end]) - WeldPosePoint2D(records[range.begin]);
        if (delta.norm() > 1e-9)
        {
            return delta.normalized();
        }
    }

    if (fallbackDirection.norm() > 1e-9)
    {
        return fallbackDirection.normalized();
    }
    return Eigen::Vector2d(1.0, 0.0);
}

double SegmentProjection(
    const QVector<WeldPoseFileRecord>& records,
    const PoseCompSegmentRange& range,
    const Eigen::Vector2d& travelDirection,
    const Eigen::Vector2d& point)
{
    if (range.begin < 0 || range.begin >= records.size())
    {
        return 0.0;
    }
    return (point - WeldPosePoint2D(records[range.begin])).dot(travelDirection);
}

WeldPoseFileRecord BuildPoseCompJunctionRecord(
    const QVector<WeldPoseFileRecord>& records,
    const PoseCompSegmentRange& ownerRange,
    const PoseCompSegmentRange& elevationRange,
    const Eigen::Vector2d& intersection)
{
    const WeldPoseFileRecord& begin = records[ownerRange.begin];
    const WeldPoseFileRecord& end = records[ownerRange.end];
    const Eigen::Vector2d beginPoint = WeldPosePoint2D(begin);
    const Eigen::Vector2d endPoint = WeldPosePoint2D(end);
    const Eigen::Vector2d span = endPoint - beginPoint;

    double ratio = 0.0;
    if (span.squaredNorm() > 1e-9)
    {
        ratio = (intersection - beginPoint).dot(span) / span.squaredNorm();
    }
    const double clampedRatio = std::clamp(ratio, 0.0, 1.0);
    WeldPoseFileRecord record = InterpolateWeldPoseRecord(begin, end, clampedRatio);
    record.point.x() = intersection.x();
    record.point.y() = intersection.y();

    // 交点记录归右段管理，但高程必须锚定平台段。否则只修改坡面补偿时，
    // 右段 Z 会被写进公共交点，随后反向拉动未修改的平台。
    const WeldPoseFileRecord& elevationBegin = records[elevationRange.begin];
    const WeldPoseFileRecord& elevationEnd = records[elevationRange.end];
    const Eigen::Vector2d elevationBeginPoint = WeldPosePoint2D(elevationBegin);
    const Eigen::Vector2d elevationSpan =
        WeldPosePoint2D(elevationEnd) - elevationBeginPoint;
    double elevationRatio = 0.0;
    if (elevationSpan.squaredNorm() > 1e-9)
    {
        elevationRatio =
            (intersection - elevationBeginPoint).dot(elevationSpan)
            / elevationSpan.squaredNorm();
    }
    if (std::isfinite(elevationRatio) && std::abs(elevationRatio) <= 3.0)
    {
        record.point.z() = elevationBegin.point.z()
            + (elevationEnd.point.z() - elevationBegin.point.z()) * elevationRatio;
    }
    record.pointType = begin.pointType;
    record.segmentKind = begin.segmentKind;
    return record;
}

bool TryApplyPoseCompJunctionIntersection(
    QVector<WeldPoseFileRecord>& records,
    const PoseCompSegmentRange& leftRange,
    const PoseCompSegmentRange& rightRange,
    PoseCompJunctionApplyStats& stats)
{
    constexpr double kMoveEpsilonMm = 1e-5;
    constexpr double kTrimEpsilonMm = 1e-4;
    if (!ShouldRebuildPoseCompJunction(leftRange.kind, rightRange.kind)
        || leftRange.begin < 0
        || leftRange.end < leftRange.begin
        || rightRange.begin <= leftRange.end
        || rightRange.end < rightRange.begin
        || rightRange.end >= records.size())
    {
        return false;
    }

    const QVector<int> leftFitIndices = CollectSameKindLineWindowBefore(
        records,
        leftRange.end,
        8);
    const QVector<int> rightFitIndices = CollectSameKindLineWindowAfter(
        records,
        rightRange.begin,
        8);

    Eigen::Vector2d leftPoint;
    Eigen::Vector2d leftDirection;
    Eigen::Vector2d rightPoint;
    Eigen::Vector2d rightDirection;
    if (!TryFitWeldPoseLine2D(records, leftFitIndices, leftPoint, leftDirection)
        || !TryFitWeldPoseLine2D(records, rightFitIndices, rightPoint, rightDirection))
    {
        return false;
    }

    Eigen::Vector2d intersection;
    if (!TryIntersectWeldPoseLines2D(
            leftPoint,
            leftDirection,
            rightPoint,
            rightDirection,
            intersection))
    {
        return false;
    }

    const Eigen::Vector2d leftEnd = WeldPosePoint2D(records[leftRange.end]);
    const Eigen::Vector2d rightBegin = WeldPosePoint2D(records[rightRange.begin]);
    const double boundaryGap = (rightBegin - leftEnd).norm();
    const double leftLength = (WeldPosePoint2D(records[leftRange.end])
        - WeldPosePoint2D(records[leftRange.begin])).norm();
    const double rightLength = (WeldPosePoint2D(records[rightRange.end])
        - WeldPosePoint2D(records[rightRange.begin])).norm();
    const double maxAllowedDistance = std::max(
        20.0,
        std::max(boundaryGap * 12.0, std::min(leftLength, rightLength) * 2.0));
    if ((intersection - leftEnd).norm() > maxAllowedDistance
        || (intersection - rightBegin).norm() > maxAllowedDistance)
    {
        return false;
    }

    QVector<int> removeIndices;
    const Eigen::Vector2d leftTravelDirection =
        SegmentTravelDirection2D(records, leftRange, leftDirection);
    const double leftIntersectionProjection =
        SegmentProjection(records, leftRange, leftTravelDirection, intersection);
    for (int index = leftRange.end; index > leftRange.begin; --index)
    {
        const double projection = SegmentProjection(
            records,
            leftRange,
            leftTravelDirection,
            WeldPosePoint2D(records[index]));
        if (projection <= leftIntersectionProjection + kTrimEpsilonMm)
        {
            break;
        }
        removeIndices.push_back(index);
    }

    const Eigen::Vector2d rightTravelDirection =
        SegmentTravelDirection2D(records, rightRange, rightDirection);
    const double rightIntersectionProjection =
        SegmentProjection(records, rightRange, rightTravelDirection, intersection);
    for (int index = rightRange.begin + 1; index < rightRange.end; ++index)
    {
        const double projection = SegmentProjection(
            records,
            rightRange,
            rightTravelDirection,
            WeldPosePoint2D(records[index]));
        if (projection >= rightIntersectionProjection - kTrimEpsilonMm)
        {
            break;
        }
        removeIndices.push_back(index);
    }

    // 平台是交界高程的稳定基准：坡面改变后延长到平台，而不是让坡面端点
    // 带动平台。边界正常是一平台一坡面；异常输入则保留右段归属作为回退。
    const PoseCompSegmentRange& elevationRange =
        IsPlatformSegmentKind(leftRange.kind)
        ? leftRange
        : rightRange;
    WeldPoseFileRecord junctionRecord = BuildPoseCompJunctionRecord(
        records,
        rightRange,
        elevationRange,
        intersection);
    const bool moved =
        (records[rightRange.begin].point - junctionRecord.point).norm() > kMoveEpsilonMm;
    if (!moved && removeIndices.isEmpty())
    {
        return false;
    }

    records[rightRange.begin] = junctionRecord;
    std::sort(removeIndices.begin(), removeIndices.end());
    removeIndices.erase(std::unique(removeIndices.begin(), removeIndices.end()), removeIndices.end());
    for (int index = removeIndices.size() - 1; index >= 0; --index)
    {
        records.removeAt(removeIndices[index]);
    }

    ++stats.adjustedJunctionCount;
    stats.removedPointCount += removeIndices.size();
    return true;
}

PoseCompJunctionApplyStats ApplyPoseCompSegmentJunctionIntersections(
    QVector<WeldPoseFileRecord>& records)
{
    PoseCompJunctionApplyStats stats;
    int junctionIndex = 0;
    while (true)
    {
        const QVector<PoseCompSegmentRange> ranges = CollectPoseCompSegmentRanges(records);
        if (junctionIndex + 1 >= ranges.size())
        {
            break;
        }

        // 从左到右，每个物理段边界最多重建一次。重建会删除边界附近的冗余点，
        // 因此每次重新收集索引，但绝不回头反复拟合同一边界，避免误差向相邻直线段传播。
        TryApplyPoseCompJunctionIntersection(
            records,
            ranges[junctionIndex],
            ranges[junctionIndex + 1],
            stats);
        ++junctionIndex;
    }

    if (stats.adjustedJunctionCount > 0)
    {
        RenumberWeldPoseRecords(records);
    }
    return stats;
}

int StraightenPoseCompPhysicalSegments(
    QVector<WeldPoseFileRecord>& records)
{
    constexpr double kMoveEpsilonMm = 1e-6;
    const QVector<PoseCompSegmentRange> ranges =
        CollectPoseCompSegmentRanges(records);
    int adjustedPointCount = 0;

    for (int rangeIndex = 0; rangeIndex < ranges.size(); ++rangeIndex)
    {
        const PoseCompSegmentRange& range = ranges[rangeIndex];
        const int beginIndex = range.begin;
        const int endIndex = rangeIndex + 1 < ranges.size()
            ? ranges[rangeIndex + 1].begin
            : range.end;
        if (beginIndex < 0
            || endIndex <= beginIndex
            || endIndex >= records.size())
        {
            continue;
        }

        if (IsPlatformSegmentKind(range.kind))
        {
            // 平台补偿必须保持刚性平移。相邻坡面平移后，公共边界已经由
            // ApplyPoseCompSegmentJunctionIntersections 通过无限直线延长求交；
            // 这里若再用“平台起点 -> 新交点”重插值整段，会把坡面改动传播到平台。
            // 延长缺口由后续定步长补点填充，越过交点的点已在交点阶段裁掉。
            continue;
        }

        bool containsLapStep = false;
        for (int index = beginIndex; index <= endIndex; ++index)
        {
            if (records[index].isLapStep)
            {
                containsLapStep = true;
                break;
            }
        }
        if (containsLapStep)
        {
            // 搭接台阶必须保留两个独立端点，不能把台阶跨接成斜线。
            continue;
        }

        QVector<double> cumulativeDistance(endIndex - beginIndex + 1, 0.0);
        for (int index = beginIndex + 1; index <= endIndex; ++index)
        {
            cumulativeDistance[index - beginIndex] =
                cumulativeDistance[index - beginIndex - 1]
                + (records[index].point - records[index - 1].point).norm();
        }
        const double totalDistanceMm = cumulativeDistance.back();
        if (!std::isfinite(totalDistanceMm) || totalDistanceMm <= 1e-9)
        {
            continue;
        }

        const Eigen::Vector3d startPoint = records[beginIndex].point;
        const Eigen::Vector3d endPoint = records[endIndex].point;
        for (int index = beginIndex + 1; index < endIndex; ++index)
        {
            const double ratio = std::clamp(
                cumulativeDistance[index - beginIndex] / totalDistanceMm,
                0.0,
                1.0);
            const Eigen::Vector3d straightPoint =
                startPoint + (endPoint - startPoint) * ratio;
            if ((records[index].point - straightPoint).norm() > kMoveEpsilonMm)
            {
                records[index].point = straightPoint;
                ++adjustedPointCount;
            }
        }
    }

    return adjustedPointCount;
}

WeldSeamCompApplyStats ApplyWeldSeamCompToWeldPoseRecords(
    const WeldPosePreset& preset,
    QVector<WeldPoseFileRecord>& records)
{
    WeldSeamCompApplyStats stats;
    if (records.isEmpty())
    {
        return stats;
    }

    QVector<Eigen::Vector3d> basePoints;
    basePoints.reserve(records.size());
    for (const WeldPoseFileRecord& record : records)
    {
        basePoints.push_back(record.point);
    }
    const Eigen::Vector3d overallSeamDirection =
        ResolveOverallHorizontalWeldDirection(basePoints);
    const Eigen::Vector3d overallGunDirection = HorizontalUnitOrZero(
        Eigen::Vector3d::UnitZ().cross(overallSeamDirection));

    const WeldPosePreset::SeamCompValues& seamComp = preset.seamComp;
    for (int index = 0; index < records.size(); ++index)
    {
        WeldPoseFileRecord& record = records[index];
        Eigen::Vector3d horizontalComp = Eigen::Vector3d::Zero();

        if (std::abs(seamComp.weldZComp) > 1e-6)
        {
            record.point.z() += seamComp.weldZComp;
            ++stats.zAdjustedCount;
        }

        if (std::abs(seamComp.weldGunDirComp) > 1e-6)
        {
            // 焊道补偿按不随采样顺序翻转的焊道轴计算；整条轨迹只做同一个刚性平移。
            if (overallGunDirection.head<2>().norm() > 1e-9)
            {
                horizontalComp += overallGunDirection * seamComp.weldGunDirComp;
                ++stats.gunDirAdjustedCount;
            }
        }

        if (std::abs(seamComp.weldSeamDirComp) > 1e-6
            && overallSeamDirection.head<2>().norm() > 1e-9)
        {
            horizontalComp += overallSeamDirection * seamComp.weldSeamDirComp;
            ++stats.seamDirAdjustedCount;
        }

        if (horizontalComp.head<2>().norm() > 1e-9)
        {
            record.point += horizontalComp;
        }
    }

    return stats;
}

// 焊缝补偿平移之后的完整后处理（端点裁剪/自交裁剪/拐点恢复/加密/显式圆弧过渡/锐角裁剪/重编号）。
// 管线 ApplyWeldSeamCompToPoseFile 与补偿预览共用，保证预览贴近实际下发轨迹（单一事实源）。
struct SeamCompFinalizeStats
{
    WeldEndpointTrimStats endpointTrim;
    WeldCornerRestoreStats cornerRestore;
    WeldCornerArcApplyStats arc;
    int densifiedPointCount = 0;
    int postArcDensifiedPointCount = 0;
    int finalDensifiedPointCount = 0;
    int smoothedRemainingCornerCount = 0;
    double densifyStepMm = 2.0;
    bool emptyAfterEndpointTrim = false;
    bool emptyAfterSelfIntersection = false;
};

SeamCompFinalizeStats FinalizeSeamCompedWeldPoseRecords(
    const WeldPosePreset& preset,
    const QVector<WeldPoseFileRecord>& recordsBeforeTrim,
    QVector<WeldPoseFileRecord>& records,
    WeldSeamCompApplyStats& compStats)
{
    SeamCompFinalizeStats stats;
    constexpr double kSharpArcAngleRad = 8.0 * 3.14159265358979323846 / 180.0;

    TrimWeldPoseRecordEndpoints(preset, records, stats.endpointTrim);
    if (records.isEmpty())
    {
        stats.emptyAfterEndpointTrim = true;
        return stats;
    }
    TrimWeldPathSelfIntersections(records, compStats);
    if (records.isEmpty())
    {
        stats.emptyAfterSelfIntersection = true;
        return stats;
    }
    stats.cornerRestore = RestoreTrimmedWeldCornersByLineIntersection(recordsBeforeTrim, records);
    stats.densifyStepMm = std::min(2.0, EstimateWeldPoseStepMm(records));
    stats.densifiedPointCount = DensifyWeldPoseRecordsByStep(records, stats.densifyStepMm);
    stats.arc = ApplyCornerArcTransitionToWeldPoseRecords(preset, records);
    stats.postArcDensifiedPointCount = DensifyWeldPoseRecordsByStep(records, stats.densifyStepMm);
    TrimSharpWeldArcEntryPoints(records, kSharpArcAngleRad, std::max(2.0, stats.densifyStepMm * 2.5));
    TrimSharpWeldArcExitPoints(records, kSharpArcAngleRad, std::max(2.0, stats.densifyStepMm * 2.5));
    stats.finalDensifiedPointCount = DensifyWeldPoseRecordsByStep(records, stats.densifyStepMm);
    TrimSharpWeldArcEntryPoints(records, kSharpArcAngleRad, std::max(2.0, stats.densifyStepMm * 2.5));
    TrimSharpWeldArcExitPoints(records, kSharpArcAngleRad, std::max(2.0, stats.densifyStepMm * 2.5));
    stats.finalDensifiedPointCount += DensifyWeldPoseRecordsByStep(records, stats.densifyStepMm);
    // 非圆弧拐点保持严格的直线-直线连接；只有上面的显式工艺圆弧允许产生曲线。
    stats.smoothedRemainingCornerCount = 0;
    RenumberWeldPoseRecords(records);
    return stats;
}

// 姿态补偿槽位匹配的单一事实源。生产计算与预览方向提示必须共用，
// 否则界面可能展示一个槽位，实际轨迹却匹配到另一个槽位。
int ResolvePoseCompSlotIndex(
    const std::vector<WeldPosePreset::PoseCompSlot>& poseCompSlots,
    int poseCompMatchMode,
    double poseMatchMaxErrorDeg,
    double rx,
    double ry,
    double rz,
    const QString& segmentKind)
{
    if (NormalizePoseCompMatchMode(poseCompMatchMode) == POSE_COMP_MATCH_BY_SEGMENT_CODE)
    {
        // 剥掉 _transition/_arc 后缀再匹配：管线内传入的 segment.kind 本就无后缀（无影响），
        // 预览从 _WeldPose_2mm 文件读回的段类带后缀，不剥会漏补偿过渡点、轨迹出现弯折。
        const int defaultIndex = DefaultPoseCompSlotIndex(NormalizeWeldSegmentKind(segmentKind));
        return defaultIndex >= 0 && defaultIndex < static_cast<int>(poseCompSlots.size())
            ? defaultIndex
            : -1;
    }

    int slotIndex = -1;
    double bestDistance = std::numeric_limits<double>::max();
    for (int index = 0; index < static_cast<int>(poseCompSlots.size()); ++index)
    {
        const WeldPosePreset::PoseCompSlot& slot = poseCompSlots[index];
        if (!slot.validReference)
        {
            continue;
        }
        const double distance = PoseDistanceDeg(rx, ry, rz, slot.poseRx, slot.poseRy, slot.poseRz);
        if (distance > poseMatchMaxErrorDeg)
        {
            continue;
        }
        if (distance < bestDistance)
        {
            bestDistance = distance;
            slotIndex = index;
        }
    }
    return slotIndex;
}

Eigen::Vector3d ResolvePoseCompWorldVector(
    const WeldPosePreset::PoseCompSlot& slot,
    int robotType,
    double rx,
    double ry,
    double rz,
    const QString& segmentKind,
    const Eigen::Vector3d& segmentWeldNormal)
{
    const Eigen::Vector3d poseCompLocal(slot.compX, slot.compY, slot.compZ);
    if (poseCompLocal.norm() <= 1e-9)
    {
        return Eigen::Vector3d::Zero();
    }

    // 四类物理段统一使用焊道自身坐标基准，不能让 RX=±180 或坡面 RZ 夹紧
    // 把局部 Y 旋到焊道切向：Y=焊道法向，X=由该法向唯一确定的稳定切向，
    // Z=世界 Z。法向分支由测量/输出参考姿态选择，正反扫描时不会随点序翻转。
    const QString normalizedKind = NormalizeWeldSegmentKind(segmentKind);
    if (IsPlatformSegmentKind(normalizedKind) || IsSlopeSegmentKind(normalizedKind))
    {
        const Eigen::Vector3d weldNormal = HorizontalUnitOrZero(segmentWeldNormal);
        if (weldNormal.head<2>().norm() > 1e-9)
        {
            const Eigen::Vector3d stableTangent = HorizontalUnitOrZero(
                Eigen::Vector3d::UnitZ().cross(weldNormal));
            return stableTangent * slot.compX
                + weldNormal * slot.compY
                + Eigen::Vector3d::UnitZ() * slot.compZ;
        }
    }

    return RobotPoseTransform::RotationFromAnglesDeg(rx, ry, rz, robotType) * poseCompLocal;
}

// 姿态补偿的「匹配槽位 + 焊道局部基准映射」单一事实源。
// 四类物理段的位置补偿固定映射到段切向、段法向和世界 Z；
// 焊枪输出姿态即使在段尾渐变，也不能旋转同一段的位置补偿方向。
// 管线（BuildSegmentPoseOutputLines）与补偿预览（MeasureThenWeldService::RecomputeCompPreview）共用，
// 确保界面显示的补偿后焊道与实际下发轨迹一致。
Eigen::Vector3d ApplyPoseCompToPoint(
    const std::vector<WeldPosePreset::PoseCompSlot>& poseCompSlots,
    int poseCompMatchMode,
    double poseMatchMaxErrorDeg,
    int robotType,
    const Eigen::Vector3d& point,
    double rx,
    double ry,
    double rz,
    const QString& segmentKind,
    const Eigen::Vector3d& segmentWeldNormal = Eigen::Vector3d::Zero())
{
    const int slotIndex = ResolvePoseCompSlotIndex(
        poseCompSlots,
        poseCompMatchMode,
        poseMatchMaxErrorDeg,
        rx,
        ry,
        rz,
        segmentKind);

    if (slotIndex < 0)
    {
        return point;
    }
    const WeldPosePreset::PoseCompSlot& slot = poseCompSlots[slotIndex];
    const Eigen::Vector3d worldComp = ResolvePoseCompWorldVector(
        slot,
        robotType,
        rx,
        ry,
        rz,
        segmentKind,
        segmentWeldNormal);
    if (worldComp.norm() <= 1e-9)
    {
        return point;
    }
    return point + worldComp;
}

std::vector<QString> BuildSegmentPoseOutputLines(
    const RobotCalculation::LowerWeldClassificationResult& result,
    const T_PRECISE_MEASURE_PARAM& param,
    const WeldPosePreset& preset,
    double platformFlatSlopeThreshold,
    const MeasureThenWeldService::LogCallback& appendLog)
{
    struct SegmentInfo
    {
        int begin = 0;
        int end = 0;
        int nextBegin = 0;
        int transitionBegin = std::numeric_limits<int>::max();
        double beginDistance = 0.0;
        double endDistance = 0.0;
        double transitionBeginDistance = std::numeric_limits<double>::max();
        RobotCalculation::LowerWeldPointType beginType = RobotCalculation::LowerWeldPointType::Normal;
        RobotCalculation::LowerWeldPointType endMarkerType = RobotCalculation::LowerWeldPointType::Normal;
        QString kind;
        double fixedRz = 0.0;
        double directionDeg = 0.0;
        double baseRz = 0.0;
        double normalReferenceDistance = 0.0;
        double rejectedRz = 0.0;
        double rawRzDeviationFromReference = 0.0;
        double clampedRzDeviationFromReference = 0.0;
        bool slopeRzClamped = false;
        bool directionValid = false;
        Eigen::Vector3d poseCompWeldNormal = Eigen::Vector3d::Zero();
        QVector<double> distanceToEnd;
    };

    struct PoseCompSlotAccumulator
    {
        bool hasValue = false;
        double totalWeight = 0.0;
        double totalRx = 0.0;
        double totalRy = 0.0;
        double totalRz = 0.0;
        double rzReference = 0.0;

        void Add(double rx, double ry, double rz, double weight)
        {
            const double safeWeight = std::max(1e-6, weight);
            if (!hasValue)
            {
                rzReference = rz;
                hasValue = true;
            }

            totalWeight += safeWeight;
            totalRx += rx * safeWeight;
            totalRy += ry * safeWeight;
            totalRz += NormalizeAngleNear(rz, rzReference) * safeWeight;
        }

        bool Resolve(double& rx, double& ry, double& rz) const
        {
            if (!hasValue || totalWeight <= 1e-6)
            {
                return false;
            }

            rx = totalRx / totalWeight;
            ry = totalRy / totalWeight;
            rz = NormalizeAngleToFanucRange(totalRz / totalWeight);
            return true;
        }
    };

    std::vector<QString> lines;
    if (appendLog)
    {
        if (!preset.seamCompLoadError.isEmpty())
        {
            appendLog(QStringLiteral("焊道补偿配置读取告警：") + preset.seamCompLoadError);
        }
        for (const QString& warning : preset.seamCompWarnings)
        {
            appendLog(QStringLiteral("焊道补偿配置兼容告警：") + warning);
        }
    }
    if (result.points.isEmpty())
    {
        return lines;
    }

    lines.reserve(static_cast<size_t>(result.points.size()) + 2);
    lines.push_back("weld_index raw_index x y z rx ry rz bx by bz point_type segment_kind is_lap_step");

    std::vector<int> keyPointPositions;
    std::vector<RobotCalculation::LowerWeldPointType> keyPointTypes;
    keyPointPositions.reserve(result.points.size());
    keyPointTypes.reserve(result.points.size());
    for (int index = 0; index < result.points.size(); ++index)
    {
        const RobotCalculation::LowerWeldPointType pointType = result.points[index].type;
        if (pointType == RobotCalculation::LowerWeldPointType::Normal
            || pointType == RobotCalculation::LowerWeldPointType::Noise)
        {
            continue;
        }
        keyPointPositions.push_back(index);
        keyPointTypes.push_back(pointType);
    }

    if (keyPointPositions.empty())
    {
        return lines;
    }
    if (keyPointTypes.front() != RobotCalculation::LowerWeldPointType::Start)
    {
        keyPointPositions.insert(keyPointPositions.begin(), 0);
        keyPointTypes.insert(keyPointTypes.begin(), RobotCalculation::LowerWeldPointType::Start);
    }
    if (keyPointTypes.back() != RobotCalculation::LowerWeldPointType::End)
    {
        keyPointPositions.push_back(result.points.size() - 1);
        keyPointTypes.push_back(RobotCalculation::LowerWeldPointType::End);
    }
    if (keyPointPositions.size() < 2)
    {
        return lines;
    }

    if (appendLog)
    {
        appendLog("焊道RZ按焊道法向生成：RZ=0表示枪尖指向机器人X-，顺时针为正，180输出为-180；每段先计算焊道方向的两个垂直法向，再用本次测量姿态RZ选择唯一法向；坡面段再按夹紧范围限制相对上一段的偏转。");
    }

    QVector<QString> measurementDepthSegmentKinds;
    const bool hasMeasurementDepthSegmentKinds =
        AssignSegmentKindsByMeasurementGunDepth(
            result.points,
            keyPointPositions,
            preset,
            platformFlatSlopeThreshold,
            keyPointTypes,
            measurementDepthSegmentKinds,
            appendLog);

    QVector<double> distanceFromStart(result.points.size(), 0.0);
    for (int index = 1; index < result.points.size(); ++index)
    {
        distanceFromStart[index] = distanceFromStart[index - 1]
            + (result.points[index].point - result.points[index - 1].point).norm();
    }

    std::vector<SegmentInfo> segments;
    double previousSegmentRz = preset.measureReferenceRz;
    Eigen::Vector3d previousPoseCompWeldNormal = HorizontalUnitOrZero(
        GunDirectionVectorFromRobotRz(preset.measureReferenceRz));
    QString previousSegmentKind;
    for (size_t segmentIndex = 0; segmentIndex + 1 < keyPointPositions.size(); ++segmentIndex)
    {
        SegmentInfo segment;
        segment.begin = keyPointPositions[segmentIndex];
        segment.beginType = keyPointTypes[segmentIndex];
        segment.endMarkerType = keyPointTypes[segmentIndex + 1];
        segment.nextBegin = keyPointPositions[segmentIndex + 1];
        segment.end = (segmentIndex + 2 < keyPointPositions.size())
            ? std::max(segment.begin, segment.nextBegin - 1)
            : std::max(segment.begin, segment.nextBegin);
        segment.kind = hasMeasurementDepthSegmentKinds
            && static_cast<int>(segmentIndex) < measurementDepthSegmentKinds.size()
                ? measurementDepthSegmentKinds[static_cast<int>(segmentIndex)]
                : QString();
        if (segment.kind.isEmpty())
        {
            segment.kind = result.points[segment.begin].segmentKindAfter.trimmed();
        }
        if (segment.kind.isEmpty())
        {
            segment.kind = LowerWeldSegmentKindText(segment.beginType, segment.endMarkerType);
        }

        // 搭接错位台阶段(两端均为 geometry_lap_step)：段方向≈侧向，不当独立焊段——继承上一段(平台)类型，
        // 使其姿态补偿槽/轨迹段属性归属于平台而非被误判为斜边。
        const bool isLapStepSegment =
            result.points[segment.begin].source == QStringLiteral("geometry_lap_step")
            && result.points[segment.nextBegin].source == QStringLiteral("geometry_lap_step");
        if (isLapStepSegment)
        {
            // 台阶段无条件落到平台 kind(首段即台阶时 previousSegmentKind 为空也兜底)，绝不留几何角/坡道类型。
            segment.kind = previousSegmentKind.isEmpty()
                ? QStringLiteral("low_platform") : previousSegmentKind;
        }

        bool segmentValid = false;
        const double segmentDirectionDeg = ComputeDirectionAngleDeg(
            result.points[segment.begin].point,
            result.points[segment.nextBegin].point,
            &segmentValid);
        segment.directionDeg = segmentDirectionDeg;
        segment.directionValid = segmentValid;
        double segmentRz = previousSegmentRz;
        if (segmentValid && !isLapStepSegment)
        {
            double rejectedRz = 0.0;
            double normalReferenceDistance = 0.0;
            const Eigen::Vector3d segmentVector =
                result.points[segment.nextBegin].point - result.points[segment.begin].point;
            const double normalRzDeg = ComputeLineNormalRobotRz(
                segmentVector,
                preset.measureReferenceRy,
                preset.measureReferenceRz,
                nullptr,
                &rejectedRz,
                &normalReferenceDistance);
            segment.poseCompWeldNormal = HorizontalUnitOrZero(
                GunDirectionVectorFromRobotRz(normalRzDeg));
            const double selectedRz = NormalizeAngleNear(normalRzDeg, previousSegmentRz);
            const double rawRzDeviation =
                selectedRz - previousSegmentRz;
            double clampedRzDeviation = rawRzDeviation;
            double baseRz = selectedRz;
            if (IsSlopeSegmentKind(segment.kind))
            {
                clampedRzDeviation = std::clamp(
                    rawRzDeviation,
                    preset.slopeRzMinDeg,
                    preset.slopeRzMaxDeg);
                baseRz = previousSegmentRz + clampedRzDeviation;
                segment.slopeRzClamped = std::abs(clampedRzDeviation - rawRzDeviation) > 1e-6;
                if (segment.slopeRzClamped && appendLog)
                {
                    appendLog(QString("斜面段 %1 RZ夹紧：相对上一段原始变化=%2 deg，夹紧后=%3 deg，范围=[%4, %5] deg")
                        .arg(segment.kind)
                        .arg(rawRzDeviation, 0, 'f', 3)
                        .arg(clampedRzDeviation, 0, 'f', 3)
                        .arg(preset.slopeRzMinDeg, 0, 'f', 3)
                        .arg(preset.slopeRzMaxDeg, 0, 'f', 3));
                }
            }

            segment.rawRzDeviationFromReference = rawRzDeviation;
            segment.clampedRzDeviationFromReference = clampedRzDeviation;
            segment.normalReferenceDistance = normalReferenceDistance;
            segment.rejectedRz = rejectedRz;
            segment.baseRz = NormalizeRobotRzOutputRange(baseRz);
            segmentRz = NormalizeAngleNear(baseRz, previousSegmentRz);
        }
        else if (isLapStepSegment)
        {
            // 台阶段：焊枪保持上一平台段姿态横移跨过错位，RZ 沿用 previousSegmentRz、不按焊道法向重算
            // （台阶段方向≈侧向，按法向算会产生 ~90° 姿态突跳，危及焊枪/工件安全）。
            segment.baseRz = NormalizeRobotRzOutputRange(previousSegmentRz);
            segment.poseCompWeldNormal = previousPoseCompWeldNormal;
            segment.directionValid = false;
            if (appendLog)
            {
                appendLog(QString("搭接错位台阶段：RZ 沿用上一段=%1 deg，不按焊道法向重算(避免姿态突跳)。")
                    .arg(NormalizeRobotRzOutputRange(previousSegmentRz), 0, 'f', 3));
            }
        }

        segment.fixedRz = NormalizeRobotRzOutputRange(segmentRz);
        if (HorizontalUnitOrZero(segment.poseCompWeldNormal).head<2>().norm() <= 1e-9)
        {
            segment.poseCompWeldNormal = previousPoseCompWeldNormal;
        }
        segment.distanceToEnd.resize(segment.end - segment.begin + 1);
        double accumulatedDistance = 0.0;
        segment.distanceToEnd[segment.end - segment.begin] = 0.0;
        for (int index = segment.end - 1; index >= segment.begin; --index)
        {
            accumulatedDistance += (result.points[index + 1].point - result.points[index].point).norm();
            segment.distanceToEnd[index - segment.begin] = accumulatedDistance;
        }
        segment.beginDistance = distanceFromStart[segment.begin];
        segment.endDistance = distanceFromStart[segment.end];

        if (segmentIndex + 1 < segments.capacity())
        {
            // no-op, just silence accidental warnings in some configurations
        }

        if ((segmentIndex + 1) < (keyPointPositions.size() - 1) && preset.cornerTransitionLeadDistance > 1e-6)
        {
            segment.transitionBegin = segment.end;
            for (int index = segment.end; index >= segment.begin; --index)
            {
                const double remainingDistance = segment.distanceToEnd[index - segment.begin];
                if (remainingDistance >= preset.cornerTransitionLeadDistance)
                {
                    segment.transitionBegin = std::min(segment.end, index + 1);
                    segment.transitionBeginDistance = distanceFromStart[segment.transitionBegin];
                    break;
                }
                segment.transitionBegin = index;
            }
        }

        segments.push_back(segment);
        previousSegmentRz = segmentRz;
        previousPoseCompWeldNormal = segment.poseCompWeldNormal;
        previousSegmentKind = segment.kind;
    }

    if (segments.empty())
    {
        return lines;
    }

    const bool useTaughtWeldPose = preset.useTaughtWeldPose;
    const double outputPoseRx = useTaughtWeldPose ? preset.taughtWeldPoseRx : preset.rx;
    const double outputPoseRy = useTaughtWeldPose ? preset.taughtWeldPoseRy : preset.ry;
    double taughtPlatformRz = NormalizeRobotRzOutputRange(preset.taughtWeldPoseRz);
    double taughtComputedPlatformRz = 0.0;
    double taughtRzOffset = 0.0;
    bool hasTaughtRzReference = false;
    QString taughtReferenceKind;
    if (useTaughtWeldPose)
    {
        const SegmentInfo* referenceSegment = nullptr;
        for (const SegmentInfo& segment : segments)
        {
            if (IsPlatformSegmentKind(segment.kind))
            {
                referenceSegment = &segment;
                break;
            }
        }
        if (referenceSegment == nullptr)
        {
            referenceSegment = &segments.front();
        }

        taughtComputedPlatformRz = NormalizeAngleNear(referenceSegment->fixedRz, taughtPlatformRz);
        const double taughtRzNearComputed = NormalizeAngleNear(taughtPlatformRz, taughtComputedPlatformRz);
        taughtRzOffset = taughtComputedPlatformRz - taughtRzNearComputed;
        taughtReferenceKind = referenceSegment->kind;
        hasTaughtRzReference = true;
        if (appendLog)
        {
            appendLog(QString("启用示教焊接姿态：RX=%1, RY=%2；参考段=%3，计算平台RZ=%4 deg，示教RZ=%5 deg，差值=%6 deg；平台使用示教RZ，坡道使用计算RZ减差值。")
                .arg(outputPoseRx, 0, 'f', 3)
                .arg(outputPoseRy, 0, 'f', 3)
                .arg(taughtReferenceKind)
                .arg(taughtComputedPlatformRz, 0, 'f', 3)
                .arg(taughtRzNearComputed, 0, 'f', 3)
                .arg(taughtRzOffset, 0, 'f', 3));
        }
    }

    auto taughtAdjustedRzForKind = [&](double calculatedRz, const QString& segmentKind) -> double
    {
        if (!useTaughtWeldPose || !hasTaughtRzReference)
        {
            return NormalizeRobotRzOutputRange(calculatedRz);
        }
        if (IsPlatformSegmentKind(segmentKind))
        {
            return taughtPlatformRz;
        }

        const double calculatedNearReference = NormalizeAngleNear(calculatedRz, taughtComputedPlatformRz);
        return NormalizeRobotRzOutputRange(calculatedNearReference - taughtRzOffset);
    };

    const auto poseCompReferenceRzForSegment =
        [&](const SegmentInfo& segment) -> double
    {
        // 焊枪输出姿态可在段尾渐变；这里的固定参考姿态只用于补偿槽匹配，
        // 位置补偿方向另由该物理段的固定焊道切向/法向基准决定。
        if (useTaughtWeldPose && hasTaughtRzReference)
        {
            return NormalizeRobotRzOutputRange(
                taughtAdjustedRzForKind(segment.fixedRz, segment.kind));
        }
        return NormalizeRobotRzOutputRange(segment.fixedRz + preset.weldRzGainDeg);
    };

    std::vector<WeldPosePreset::PoseCompSlot> poseCompSlots = preset.poseCompSlots;
    std::vector<PoseCompSlotAccumulator> poseCompAccumulators(poseCompSlots.size());
    for (const SegmentInfo& segment : segments)
    {
        const int slotIndex = DefaultPoseCompSlotIndex(segment.kind);
        if (slotIndex < 0 || slotIndex >= static_cast<int>(poseCompSlots.size()))
        {
            continue;
        }

        WeldPosePreset::PoseCompSlot& slot = poseCompSlots[slotIndex];
        const double segmentLength = !segment.distanceToEnd.isEmpty()
            ? std::max(1.0, segment.distanceToEnd.front())
            : std::max(1.0, static_cast<double>(segment.end - segment.begin + 1));
        poseCompAccumulators[slotIndex].Add(
            outputPoseRx,
            outputPoseRy,
            poseCompReferenceRzForSegment(segment),
            segmentLength);
    }

    for (int slotIndex = 0; slotIndex < static_cast<int>(poseCompSlots.size()); ++slotIndex)
    {
        WeldPosePreset::PoseCompSlot& slot = poseCompSlots[slotIndex];
        slot.generatedReference = false;
        if (poseCompAccumulators[slotIndex].Resolve(slot.poseRx, slot.poseRy, slot.poseRz))
        {
            slot.poseRz = NormalizeAngleToFanucRange(slot.poseRz);
            slot.generatedReference = true;
            slot.validReference = true;
        }
        else
        {
            slot.validReference = slot.hasIniReference;
        }

        if (appendLog)
        {
            appendLog(QString("姿态补偿槽 %1 [%2]：姿态 RX=%3, RY=%4, RZ=%5, 补偿 dX=%6, dY=%7, dZ=%8, 参考来源=%9")
                .arg(slot.name)
                .arg(slot.segmentKind.isEmpty() ? QString("unassigned") : slot.segmentKind)
                .arg(slot.poseRx, 0, 'f', 3)
                .arg(slot.poseRy, 0, 'f', 3)
                .arg(slot.poseRz, 0, 'f', 3)
                .arg(slot.compX, 0, 'f', 3)
                .arg(slot.compY, 0, 'f', 3)
                .arg(slot.compZ, 0, 'f', 3)
                .arg(slot.generatedReference
                    ? QString("分段均值")
                    : (slot.hasIniReference ? QString("ini回退") : QString("未生成"))));
        }
    }

    const int lowPlatformSlotIndex = DefaultPoseCompSlotIndex("low_platform");
    const int highPlatformSlotIndex = DefaultPoseCompSlotIndex("high_platform");
    if (lowPlatformSlotIndex >= 0
        && highPlatformSlotIndex >= 0
        && lowPlatformSlotIndex < static_cast<int>(poseCompSlots.size())
        && highPlatformSlotIndex < static_cast<int>(poseCompSlots.size()))
    {
        WeldPosePreset::PoseCompSlot& lowPlatformSlot = poseCompSlots[lowPlatformSlotIndex];
        WeldPosePreset::PoseCompSlot& highPlatformSlot = poseCompSlots[highPlatformSlotIndex];
        if (lowPlatformSlot.validReference && highPlatformSlot.validReference)
        {
            const double lowHighPoseDistance = PoseDistanceDeg(
                highPlatformSlot.poseRx,
                highPlatformSlot.poseRy,
                highPlatformSlot.poseRz,
                lowPlatformSlot.poseRx,
                lowPlatformSlot.poseRy,
                lowPlatformSlot.poseRz);
            if (lowHighPoseDistance <= preset.poseMatchMaxErrorDeg)
            {
                highPlatformSlot.poseRx = lowPlatformSlot.poseRx;
                highPlatformSlot.poseRy = lowPlatformSlot.poseRy;
                highPlatformSlot.poseRz = lowPlatformSlot.poseRz;
                if (appendLog)
                {
                    appendLog(QString("姿态补偿槽 [%1] 与 [%2] 姿态差=%3 deg，小于复用阈值=%4 deg，统一复用 [%1] 的参考姿态。")
                        .arg(lowPlatformSlot.segmentKind)
                        .arg(highPlatformSlot.segmentKind)
                        .arg(lowHighPoseDistance, 0, 'f', 3)
                        .arg(preset.poseMatchMaxErrorDeg, 0, 'f', 3));
                }
            }
        }
    }

    if (appendLog)
    {
        appendLog(QString("姿态补偿匹配方式=%1。")
            .arg(PoseCompMatchModeDisplayName(preset.poseCompMatchMode)));
        if (NormalizePoseCompMatchMode(preset.poseCompMatchMode) == POSE_COMP_MATCH_BY_POSE)
        {
            appendLog(QString("姿态匹配最大误差阈值=%1 deg，超过该阈值则该点不做姿态补偿。")
                .arg(preset.poseMatchMaxErrorDeg, 0, 'f', 3));
        }
        else
        {
            appendLog("按四类段属性时，低平台/上升边/高平台/下降边分别匹配姿态补偿槽0/1/2/3。");
        }
        appendLog("姿态补偿位置变换：低平台、上升边、高平台、下降边统一使用焊道基准(X=稳定切向、Y=焊道法向、Z=世界Z)；段尾RZ渐变和坡面RZ夹紧只作用于焊枪姿态，不改变段内补偿方向。");
    }

    if (appendLog)
    {
        appendLog(QString("焊道补偿（整条统一）：dZ=%1, dGunDir=%2, dSeamDir=%3, 来源=%4")
            .arg(preset.seamComp.weldZComp, 0, 'f', 3)
            .arg(preset.seamComp.weldGunDirComp, 0, 'f', 3)
            .arg(preset.seamComp.weldSeamDirComp, 0, 'f', 3)
            .arg(preset.seamCompFromIni ? preset.seamCompFilePath : QString("默认值")));
    }

    const int weldBeginCandidate = segments.front().begin;
    const int weldEndCandidate = segments.back().end;

    auto findSegmentIndex = [&segments](int pointIndex) -> int
    {
        for (int segmentIndex = 0; segmentIndex < static_cast<int>(segments.size()); ++segmentIndex)
        {
            if (pointIndex >= segments[segmentIndex].begin && pointIndex <= segments[segmentIndex].end)
            {
                return segmentIndex;
            }
        }
        return -1;
    };

    const int weldStartIndex = weldBeginCandidate;
    const int weldEndIndex = weldEndCandidate;
    if (appendLog
        && (preset.weldStartSkipDistance > 1e-6 || preset.weldEndSkipDistance > 1e-6))
    {
        appendLog(QString("起终点裁剪延后到焊道补偿和姿态补偿之后执行：StartSkip=%1mm, EndSkip=%2mm")
            .arg(preset.weldStartSkipDistance, 0, 'f', 3)
            .arg(preset.weldEndSkipDistance, 0, 'f', 3));
    }

    for (size_t segmentIndex = 0; segmentIndex < segments.size(); ++segmentIndex)
    {
        const SegmentInfo& segment = segments[segmentIndex];
        if (appendLog)
        {
            appendLog(QString("焊道姿态段 %1: 点[%2-%3], 固定RZ=%4 deg, 输出基础RZ=%5 deg, 测量参考RZ=%6 deg, 法向与测量参考夹角=%7 deg, 反向候选RZ=%8 deg, RZ原始偏差=%9 deg, RZ夹紧后=%10 deg, RX=%11 deg, RY=%12 deg, 补偿范围=%13, 过渡起点=%14, 起点跳过=%15 mm, 终点跳过=%16 mm, Z补偿=%17 mm, 枪向补偿=%18 mm, 焊道方向补偿=%19 mm")
                .arg(segment.kind)
                .arg(result.points[segment.begin].index)
                .arg(result.points[segment.end].index)
                .arg(segment.fixedRz, 0, 'f', 3)
                .arg(segment.baseRz, 0, 'f', 3)
                .arg(preset.measureReferenceRz, 0, 'f', 3)
                .arg(segment.normalReferenceDistance, 0, 'f', 3)
                .arg(segment.rejectedRz, 0, 'f', 3)
                .arg(segment.rawRzDeviationFromReference, 0, 'f', 3)
                .arg(segment.clampedRzDeviationFromReference, 0, 'f', 3)
                .arg(outputPoseRx, 0, 'f', 3)
                .arg(outputPoseRy, 0, 'f', 3)
                .arg(QStringLiteral("整条焊道统一"))
                .arg(segment.transitionBegin == std::numeric_limits<int>::max()
                    ? QString("none")
                    : QString::number(result.points[segment.transitionBegin].index))
                .arg(preset.weldStartSkipDistance, 0, 'f', 3)
                .arg(preset.weldEndSkipDistance, 0, 'f', 3)
                .arg(preset.seamComp.weldZComp, 0, 'f', 3)
                .arg(preset.seamComp.weldGunDirComp, 0, 'f', 3)
                .arg(preset.seamComp.weldSeamDirComp, 0, 'f', 3));
        }
    }

    // 姿态补偿槽位匹配 + 四类物理段焊道基准变换已下沉为
    // ApplyPoseCompToPoint（见上方），管线与补偿预览共用同一份数学。

    QVector<double> sampleDistances;
    sampleDistances.reserve(static_cast<int>((weldEndIndex - weldStartIndex + 1) * 2));
    for (double distance = distanceFromStart[weldStartIndex];
         distance <= distanceFromStart[weldEndIndex] + 1e-9;
         distance += kPoseCompOutputStepMm)
    {
        sampleDistances.push_back(distance);
    }

    for (int pointIndex = weldStartIndex; pointIndex <= weldEndIndex; ++pointIndex)
    {
        const RobotCalculation::LowerWeldPointType pointType = result.points[pointIndex].type;
        if (pointType == RobotCalculation::LowerWeldPointType::Normal
            || pointType == RobotCalculation::LowerWeldPointType::Noise)
        {
            continue;
        }

        const double pointDistance = distanceFromStart[pointIndex];
        if (pointDistance >= distanceFromStart[weldStartIndex] - 1e-9
            && pointDistance <= distanceFromStart[weldEndIndex] + 1e-9)
        {
            sampleDistances.push_back(pointDistance);
        }
    }

    sampleDistances.push_back(distanceFromStart[weldStartIndex]);
    sampleDistances.push_back(distanceFromStart[weldEndIndex]);
    std::sort(sampleDistances.begin(), sampleDistances.end());
    sampleDistances.erase(std::unique(sampleDistances.begin(), sampleDistances.end(),
        [](double left, double right)
        {
            return std::abs(left - right) <= 1e-6;
        }), sampleDistances.end());

    auto keyPointTypeForPointIndex = [&](int pointIndex) -> RobotCalculation::LowerWeldPointType
    {
        for (std::size_t keyIndex = 0; keyIndex < keyPointPositions.size(); ++keyIndex)
        {
            if (keyPointPositions[keyIndex] == pointIndex)
            {
                return keyPointTypes[keyIndex];
            }
        }
        if (pointIndex >= 0 && pointIndex < result.points.size())
        {
            return result.points[pointIndex].type;
        }
        return RobotCalculation::LowerWeldPointType::Normal;
    };

    auto samplePointTypeAtDistance = [&](double sampleDistance, int lowerIndex) -> RobotCalculation::LowerWeldPointType
    {
        for (int pointIndex = weldStartIndex; pointIndex <= weldEndIndex; ++pointIndex)
        {
            const RobotCalculation::LowerWeldPointType pointType = result.points[pointIndex].type;
            if (pointType == RobotCalculation::LowerWeldPointType::Normal
                || pointType == RobotCalculation::LowerWeldPointType::Noise)
            {
                continue;
            }

            if (std::abs(distanceFromStart[pointIndex] - sampleDistance) <= 1e-6)
            {
                return keyPointTypeForPointIndex(pointIndex);
            }
        }

        if (lowerIndex >= weldStartIndex && lowerIndex <= weldEndIndex)
        {
            return result.points[lowerIndex].type == RobotCalculation::LowerWeldPointType::Noise
                ? RobotCalculation::LowerWeldPointType::Normal
                : RobotCalculation::LowerWeldPointType::Normal;
        }
        return RobotCalculation::LowerWeldPointType::Normal;
    };

    auto samplePointOnPath = [&](double sampleDistance, int& sourceIndex, int& sourceNextIndex) -> Eigen::Vector3d
    {
        if (sampleDistance <= distanceFromStart[weldStartIndex] + 1e-9)
        {
            sourceIndex = weldStartIndex;
            sourceNextIndex = weldStartIndex;
            return result.points[weldStartIndex].point;
        }
        if (sampleDistance >= distanceFromStart[weldEndIndex] - 1e-9)
        {
            sourceIndex = weldEndIndex;
            sourceNextIndex = weldEndIndex;
            return result.points[weldEndIndex].point;
        }

        auto upperIt = std::upper_bound(
            distanceFromStart.begin() + weldStartIndex,
            distanceFromStart.begin() + weldEndIndex + 1,
            sampleDistance);
        int upperIndex = static_cast<int>(upperIt - distanceFromStart.begin());
        upperIndex = std::clamp(upperIndex, weldStartIndex + 1, weldEndIndex);
        const int lowerIndex = upperIndex - 1;
        const double beginDistance = distanceFromStart[lowerIndex];
        const double endDistance = distanceFromStart[upperIndex];
        sourceIndex = lowerIndex;
        sourceNextIndex = upperIndex;
        if (std::abs(endDistance - beginDistance) <= 1e-9)
        {
            return result.points[lowerIndex].point;
        }

        const double ratio = std::clamp((sampleDistance - beginDistance) / (endDistance - beginDistance), 0.0, 1.0);
        return result.points[lowerIndex].point
            + (result.points[upperIndex].point - result.points[lowerIndex].point) * ratio;
    };

    QVector<WeldPoseFileRecord> records;
    records.reserve(sampleDistances.size());

    int weldIndex = 1;
    QString pendingLapStepKind;  // 台阶第一点(A1)记下平台 kind，第二点(A2,属下一段)沿用，使两点补偿同槽
    double pendingLapStepPoseCompRz = 0.0;
    Eigen::Vector3d pendingLapStepPoseCompWeldNormal = Eigen::Vector3d::Zero();
    for (double sampleDistance : sampleDistances)
    {
        if (sampleDistance < distanceFromStart[weldStartIndex] - 1e-9
            || sampleDistance > distanceFromStart[weldEndIndex] + 1e-9)
        {
            continue;
        }

        int sourceIndex = weldStartIndex;
        int sourceNextIndex = weldStartIndex;
        const Eigen::Vector3d sampledPoint = samplePointOnPath(sampleDistance, sourceIndex, sourceNextIndex);

        int segmentIndex = findSegmentIndex(sourceIndex);
        if (segmentIndex < 0)
        {
            segmentIndex = findSegmentIndex(sourceNextIndex);
        }
        if (segmentIndex < 0)
        {
            continue;
        }

        const SegmentInfo& segment = segments[segmentIndex];
        // 搭接错位台阶两点(A1/A2)补偿同槽：A1 记下台阶段平台 kind，A2(属下一段)沿用，
        // 保证焊缝/姿态补偿对两点施加相同位移，X 台阶刚性保留不被斜切。
        const bool pointIsLapStep = (sourceIndex >= 0 && sourceIndex < result.points.size()
            && result.points[sourceIndex].source == QStringLiteral("geometry_lap_step"));
        QString effectiveSegmentKind = segment.kind;
        double poseCompReferenceRz = poseCompReferenceRzForSegment(segment);
        Eigen::Vector3d poseCompWeldNormal = segment.poseCompWeldNormal;
        if (pointIsLapStep)
        {
            if (pendingLapStepKind.isEmpty())
            {
                pendingLapStepKind = segment.kind;
                pendingLapStepPoseCompRz = poseCompReferenceRz;
                pendingLapStepPoseCompWeldNormal = poseCompWeldNormal;
            }
            effectiveSegmentKind = pendingLapStepKind;
            poseCompReferenceRz = pendingLapStepPoseCompRz;
            poseCompWeldNormal = pendingLapStepPoseCompWeldNormal;
        }
        else
        {
            pendingLapStepKind.clear();
        }
        const bool hasNextSegment = segmentIndex + 1 < static_cast<int>(segments.size());
        const double nextSegmentRz = hasNextSegment
            ? NormalizeAngleNear(segments[segmentIndex + 1].fixedRz, segment.fixedRz)
            : segment.fixedRz;
        const QString nextSegmentKind = hasNextSegment
            ? segments[segmentIndex + 1].kind
            : segment.kind;
        const bool inTransition = hasNextSegment
            && segment.transitionBeginDistance < std::numeric_limits<double>::max()
            && sampleDistance >= segment.transitionBeginDistance;

        double pointRz = segment.fixedRz;
        double transitionRatio = 0.0;
        if (inTransition && preset.cornerTransitionLeadDistance > 1e-6)
        {
            const double remainingDistance = std::max(0.0, segment.endDistance - sampleDistance);
            transitionRatio = 1.0 - (remainingDistance / preset.cornerTransitionLeadDistance);
            pointRz = segment.fixedRz
                + (nextSegmentRz - segment.fixedRz) * std::clamp(transitionRatio, 0.0, 1.0);
        }
        // Transition points still change angle. The target RZ at each side of
        // the transition comes from the segment weld normal, while slope
        // segments are clamped before interpolation.
        if (useTaughtWeldPose && hasTaughtRzReference)
        {
            if (inTransition && preset.cornerTransitionLeadDistance > 1e-6)
            {
                const double beginRz = taughtAdjustedRzForKind(segment.fixedRz, segment.kind);
                const double endRz = NormalizeAngleNear(
                    taughtAdjustedRzForKind(nextSegmentRz, nextSegmentKind),
                    beginRz);
                pointRz = beginRz + (endRz - beginRz) * std::clamp(transitionRatio, 0.0, 1.0);
            }
            else
            {
                pointRz = taughtAdjustedRzForKind(segment.fixedRz, segment.kind);
            }
            pointRz = NormalizeRobotRzOutputRange(pointRz);
        }
        else
        {
            pointRz = NormalizeRobotRzOutputRange(pointRz + preset.weldRzGainDeg);
        }

        const RobotCalculation::LowerWeldPointType pointType =
            samplePointTypeAtDistance(sampleDistance, sourceIndex);

        double pointRx = outputPoseRx;
        double pointRy = outputPoseRy;
        Eigen::Vector3d point = sampledPoint;
        point = ApplyPoseCompToPoint(
            poseCompSlots,
            preset.poseCompMatchMode,
            preset.poseMatchMaxErrorDeg,
            preset.robotType,
            point,
            pointRx,
            pointRy,
            poseCompReferenceRz,
            effectiveSegmentKind,
            poseCompWeldNormal);

        WeldPoseFileRecord record;
        record.weldIndex = weldIndex++;
        record.rawIndex = result.points[sourceIndex].index;
        record.point = point;
        record.rx = pointRx;
        record.ry = pointRy;
        record.rz = pointRz;
        record.bx = param.tStartPos.dBX;
        record.by = param.tStartPos.dBY;
        record.bz = param.tStartPos.dBZ;
        record.pointType = RobotCalculation::LowerWeldPointTypeName(pointType);
        record.segmentKind = inTransition ? (effectiveSegmentKind + "_transition") : effectiveSegmentKind;
        record.isLapStep = pointIsLapStep;
        records.push_back(record);
    }

    const PoseCompJunctionApplyStats poseCompJunctionStats =
        ApplyPoseCompSegmentJunctionIntersections(records);
    const int poseCompStraightenedPointCount =
        StraightenPoseCompPhysicalSegments(records);
    const int poseCompDensifiedPointCount =
        DensifyWeldPoseRecordsByStep(records, kPoseCompOutputStepMm);
    if (appendLog && poseCompJunctionStats.adjustedJunctionCount > 0)
    {
        appendLog(QString("姿态补偿段交点重建：重建平台/坡面交点=%1，裁剪多余采样点=%2。")
            .arg(poseCompJunctionStats.adjustedJunctionCount)
            .arg(poseCompJunctionStats.removedPointCount));
    }
    if (appendLog && poseCompDensifiedPointCount > 0)
    {
        appendLog(QString("姿态补偿段交点重建后补点：新增=%1，步长=%2mm。")
            .arg(poseCompDensifiedPointCount)
            .arg(kPoseCompOutputStepMm, 0, 'f', 3));
    }
    if (appendLog && poseCompStraightenedPointCount > 0)
    {
        appendLog(QString("姿态补偿坡面直线保持：平台仅刚性平移并延长求交，坡面校正中间点=%1。")
            .arg(poseCompStraightenedPointCount));
    }

    for (const WeldPoseFileRecord& record : records)
    {
        lines.push_back(BuildWeldPoseFileRecordLine(record));
    }

    return lines;
}
}

double MeasureThenWeldService::SafeSpeed(double value, double fallback)
{
    return value > 0.0 ? value : fallback;
}

bool MeasureThenWeldService::InvalidateStoredWeldResumeCheckpoint(
    const QString& robotName,
    QString& error)
{
    return InvalidateStoredWeldResumeCheckpointImpl(robotName, error);
}

bool MeasureThenWeldService::CapturePointCloudProductionExpectation(
    const RobotDriverAdaptor* pRobotDriver,
    const QString& expectedRobotName,
    PointCloudProductionExpectation& expectation,
    QString& error)
{
    expectation = PointCloudProductionExpectation{};
    error.clear();
    if (pRobotDriver == nullptr)
    {
        error = QStringLiteral("缺少当前机器人驱动，无法冻结点云生产上下文。");
        return false;
    }
    const QString driverRobotName =
        QString::fromStdString(pRobotDriver->m_sRobotName).trimmed();
    const QString normalizedRobotName = expectedRobotName.trimmed().isEmpty()
        ? driverRobotName
        : expectedRobotName.trimmed();
    if (normalizedRobotName.isEmpty()
        || driverRobotName.compare(normalizedRobotName, Qt::CaseInsensitive) != 0)
    {
        error = QStringLiteral("当前驱动机器人与待冻结的点云生产机器人不一致。");
        return false;
    }
    if (!LoadCurrentPointCloudContextExpectations(
            normalizedRobotName,
            pRobotDriver,
            true,
            expectation.robotEndpoint,
            expectation.cameraSection,
            expectation.handEyeSha256,
            error))
    {
        expectation = PointCloudProductionExpectation{};
        return false;
    }
    expectation.robotName = normalizedRobotName;
    return true;
}

namespace
{
double FanucLinearSpeedMmPerSecFromConfig(double speedMmPerMin, double fallbackMmPerSec = 1.0)
{
    if (speedMmPerMin <= 0.0)
    {
        return fallbackMmPerSec;
    }

    const double converted = speedMmPerMin / 60.0;
    return converted >= 1.0 ? std::floor(converted) : 0.0;
}

double LinearCommandSpeedForRobot(RobotDriverAdaptor* pRobotDriver, double speedMmPerMin, double fallback)
{
    if (pRobotDriver != nullptr && dynamic_cast<FANUCRobotCtrl*>(pRobotDriver) != nullptr)
    {
        return FanucLinearSpeedMmPerSecFromConfig(speedMmPerMin, fallback);
    }
    return speedMmPerMin > 0.0 ? speedMmPerMin : fallback;
}

QString LinearCommandSpeedUnitText(RobotDriverAdaptor* pRobotDriver)
{
    return (pRobotDriver != nullptr && dynamic_cast<FANUCRobotCtrl*>(pRobotDriver) != nullptr)
        ? QStringLiteral("mm/sec")
        : QStringLiteral("mm/min");
}

QString RobotMotionStatusText(RobotDriverAdaptor* pRobotDriver)
{
    if (pRobotDriver == nullptr)
    {
        return QString();
    }

    QStringList details;
    const std::string lastError = pRobotDriver->GetLastRobotError();
    if (!lastError.empty())
    {
        details << QString("最近错误：%1").arg(DecodeRobotMessageText(lastError));
    }
    const std::string statusText = pRobotDriver->GetRobotStatusText();
    if (!statusText.empty() && statusText != lastError)
    {
        details << QString("当前状态：%1").arg(DecodeRobotMessageText(statusText));
    }
    return details.join("；");
}

bool StopUnverifiedMotionAfterFailure(
    RobotDriverAdaptor* pRobotDriver,
    const QString& failure,
    const MeasureThenWeldService::LogCallback& appendLog)
{
    if (pRobotDriver == nullptr)
    {
        return false;
    }

    const QString priorDetail = RobotMotionStatusText(pRobotDriver);
    const bool stopped = RobotOperationLease::StopAndConfirmUnverifiedMotion(pRobotDriver);
    const QString stopDetail = RobotMotionStatusText(pRobotDriver);

    QString combined = failure;
    if (!priorDetail.isEmpty() && !combined.contains(priorDetail))
    {
        combined += QString("；%1").arg(priorDetail);
    }
    if (stopped)
    {
        combined += QStringLiteral("；已自动中止并确认机器人任务终态。");
    }
    else
    {
        combined += stopDetail.isEmpty()
            ? QStringLiteral("；自动中止未得到机器人终态确认。")
            : QString("；自动中止未确认：%1").arg(stopDetail);
    }
    pRobotDriver->SetLastRobotError(combined.toUtf8().toStdString());
    if (appendLog)
    {
        appendLog(combined);
    }
    return stopped;
}

bool WaitRobotMotionDone(
    RobotDriverAdaptor* pRobotDriver,
    const QString& name,
    const MeasureThenWeldService::LogCallback& appendLog,
    int startTimeoutMs = 3000,
    int finishTimeoutMs = 1800000,
    int pollDelayMs = 100)
{
    if (pRobotDriver == nullptr)
    {
        return false;
    }

    if (FANUCRobotCtrl* pFanucDriver = dynamic_cast<FANUCRobotCtrl*>(pRobotDriver))
    {
        int lastState = 0;
        const bool doneOk = pFanucDriver->WaitStateDone(
            FANUC_MOTION_STATE_REG,
            1,
            10,
            20,
            startTimeoutMs,
            finishTimeoutMs,
            pollDelayMs,
            &lastState);
        if (appendLog)
        {
            appendLog(QString("运动结束：%1, R[%2]=%3, WaitStateDone=%4")
                .arg(name)
                .arg(FANUC_MOTION_STATE_REG)
                .arg(lastState)
                .arg(doneOk ? 1 : 0));
        }
        if (!doneOk)
        {
            const QString detail = RobotMotionStatusText(pRobotDriver);
            const QString failure = detail.isEmpty()
                ? QString("运动异常：%1，未取得 FANUC 状态寄存器完成态。").arg(name)
                : QString("运动异常：%1，未取得 FANUC 状态寄存器完成态，%2").arg(name, detail);
            if (!RobotOperationLease::IsCancellationRequested(pRobotDriver))
            {
                StopUnverifiedMotionAfterFailure(pRobotDriver, failure, appendLog);
            }
            else if (appendLog)
            {
                appendLog(failure);
            }
            return false;
        }

        // R[93]=1 只是 TP 程序内部里程碑；还要等 CHECK_DONE 稳定回读，
        // 确认 CALL_JOB 任务本身已终止后才能释放 motionPending。
        const int taskDone = pFanucDriver->CheckRobotDone(pollDelayMs, finishTimeoutMs);
        if (appendLog)
        {
            appendLog(QString("运动任务终态：%1, CheckRobotDone=%2").arg(name).arg(taskDone));
            if (taskDone <= 0)
            {
                const QString detail = RobotMotionStatusText(pRobotDriver);
                if (!detail.isEmpty())
                {
                    appendLog(QString("运动异常：%1，%2").arg(name, detail));
                }
            }
        }
        // CheckRobotDone 已对非取消失败执行 StopAndConfirm，此处不重复中止。
        return taskDone > 0;
    }

    const int done = pRobotDriver->CheckRobotDone(pollDelayMs, finishTimeoutMs);
    if (appendLog)
    {
        appendLog(QString("运动结束：%1, CheckRobotDone=%2").arg(name).arg(done));
        if (done <= 0)
        {
            const QString detail = RobotMotionStatusText(pRobotDriver);
            if (!detail.isEmpty())
            {
                appendLog(QString("运动异常：%1，%2").arg(name, detail));
            }
        }
    }
    return done > 0;
}
}

bool MeasureThenWeldService::LoadPresetParam(RobotDriverAdaptor* pRobotDriver, T_PRECISE_MEASURE_PARAM& param, QString& error) const
{
    if (pRobotDriver == nullptr)
    {
        error = "机器人驱动为空。";
        return false;
    }

    param = T_PRECISE_MEASURE_PARAM();
    param.sRobotName = pRobotDriver->m_sRobotName.empty() ? "RobotA" : pRobotDriver->m_sRobotName;

    const QString robotName = QString::fromStdString(param.sRobotName);
    QString ensureError;
    if (!RobotDataHelper::EnsureMeasureWeldParamFile(robotName, &ensureError))
    {
        error = ensureError.isEmpty()
            ? QString("创建或打开测量焊接参数数据失败：%1").arg(RobotDataHelper::MeasureWeldParamPath(robotName))
            : ensureError;
        return false;
    }
    const QString iniPath = RobotDataHelper::MeasureWeldParamPath(robotName);
    if (!ConfigDatabase::HasIniFile(iniPath))
    {
        error = ensureError.isEmpty() ? QString("未找到测量焊接参数数据：%1").arg(iniPath) : ensureError;
        return false;
    }

    param.sIniFilePath = ToUtf8StdString(iniPath);

    COPini ini;
    if (!ini.SetFileName(param.sIniFilePath))
    {
        error = QString("打开测量焊接参数数据失败：%1").arg(iniPath);
        return false;
    }

    int useNo = 0;
    std::string groupName;
    ini.SetSectionName("MeasureWeldGroups");
    ini.ReadString(false, "UseGroupNo", &useNo);
    ini.ReadString(false, ToUtf8StdString(QString("Group%1Name").arg(useNo)), groupName);
    param.nParamGroupIndex = std::max(0, useNo);
    param.sParamGroupName = groupName.empty()
        ? QString("参数组%1").arg(param.nParamGroupIndex + 1)
        : QString::fromStdString(groupName);
    param.sSectionName = ToUtf8StdString(RobotDataHelper::MeasureWeldScanSectionName(param.nParamGroupIndex));
    param.sWeldSectionName = ToUtf8StdString(RobotDataHelper::MeasureWeldWeldSectionName(param.nParamGroupIndex));
    param.sWeldParamFilePath = ToUtf8StdString(iniPath);

    ini.SetSectionName(param.sSectionName);
    ini.ReadString(false, "ScanSpeed", &param.dScanSpeed);
    ini.ReadString(false, "RunSpeed", &param.dRunSpeed);
    ini.ReadString(false, "CameraTimeOffsetMs", &param.dCameraTimeOffsetMs);
    int useStatTimeAlign = 1;
    ini.ReadString(false, "UseStatTimeAlign", &useStatTimeAlign);
    param.bUseStatTimeAlign = (useStatTimeAlign != 0);
    ini.ReadString(false, "dAcc", &param.dAcc);
    ini.ReadString(false, "dDec", &param.dDec);

    COPini weldIni;
    COPini* pWeldIni = &ini;
    const QString weldParamPath = QString::fromStdString(param.sWeldParamFilePath.empty()
        ? param.sIniFilePath
        : param.sWeldParamFilePath);
    if (weldParamPath != iniPath)
    {
        if (!weldIni.SetFileName(ToUtf8StdString(weldParamPath)))
        {
            error = QString("打开焊接参数数据失败：%1").arg(weldParamPath);
            return false;
        }
        pWeldIni = &weldIni;
    }
    pWeldIni->SetSectionName(param.sWeldSectionName);
    int doActualWeld = 1;
    pWeldIni->ReadString(false, "WeldEnable", &doActualWeld);
    pWeldIni->ReadString(false, "WeldSpeedMmPerMin", &param.dWeldSpeedMmPerMin);
    pWeldIni->ReadString(false, "DryRunSpeedMmPerMin", &param.dDryRunSpeedMmPerMin);
    pWeldIni->ReadString(false, "WeldSafeMoveSpeedMmPerMin", &param.dWeldSafeMoveSpeedMmPerMin);
    pWeldIni->ReadString(false, "StepOverlapRel", &param.dStepOverlapRel);
    pWeldIni->ReadString(false, "FinalWeldTrajectoryStepMm", &param.dFinalWeldTrajectoryStepMm);
    pWeldIni->ReadString(false, "ResumeBacktrackDistanceMm", &param.dResumeBacktrackMm);
    pWeldIni->ReadString(false, "WeldDirection", &param.nWeldDirection);
    pWeldIni->ReadString(false, "GunDownBackSafeDis", &param.dGunDownBackSafeDis);
    pWeldIni->ReadString(false, "WeldSafeRetreatDirection", &param.nWeldSafeRetreatDirection);
    pWeldIni->ReadString(false, "WeldRzGainDeg", &param.dWeldRzGainDeg);
    int useTaughtWeldPose = 0;
    pWeldIni->ReadString(false, "UseTaughtWeldPose", &useTaughtWeldPose);
    pWeldIni->ReadString(false, "TaughtWeldPoseRX", &param.dTaughtWeldPoseRxDeg);
    pWeldIni->ReadString(false, "TaughtWeldPoseRY", &param.dTaughtWeldPoseRyDeg);
    pWeldIni->ReadString(false, "TaughtWeldPoseRZ", &param.dTaughtWeldPoseRzDeg);
    param.bUseTaughtWeldPose = (useTaughtWeldPose != 0);
    pWeldIni->ReadString(false, "SlopeRzMinDeg", &param.dSlopeRzMinDeg);
    pWeldIni->ReadString(false, "SlopeRzMaxDeg", &param.dSlopeRzMaxDeg);
    param.bDoActualWeld = (doActualWeld != 0);

    ini.SetSectionName(param.sSectionName);
    int useComputedScanSafe = 1;
    ini.ReadString(false, "UseComputedScanSafe", &useComputedScanSafe);
    param.bUseComputedScanSafe = (useComputedScanSafe != 0);
    ini.ReadString(false, "ScanSafeOffsetDistanceMm", &param.dScanSafeOffsetDistanceMm);
    ini.ReadString(false, "ScanSafeGunAngleDeg", &param.dScanSafeGunAngleDeg);
    ini.ReadString(false, "ScanSafeXDirection", &param.nScanSafeXDirection);
    ini.ReadString(false, "ScanSafeLiftHeightMm", &param.dScanSafeLiftHeightMm);
    ini.ReadString(false, "ScanSafeFlipWarnThresholdDeg", &param.dScanSafeFlipWarnThresholdDeg);

    if (!std::isfinite(param.dCameraTimeOffsetMs))
    {
        param.dCameraTimeOffsetMs = 0.0;
    }
    if (!std::isfinite(param.dWeldSpeedMmPerMin) || param.dWeldSpeedMmPerMin <= 0.0)
    {
        param.dWeldSpeedMmPerMin = FANUC_WELD_PATH_SPEED_MM_PER_MIN;
    }
    if (!std::isfinite(param.dDryRunSpeedMmPerMin) || param.dDryRunSpeedMmPerMin <= 0.0)
    {
        param.dDryRunSpeedMmPerMin = DEFAULT_DRY_RUN_SPEED_MM_PER_MIN;
    }
    if (!std::isfinite(param.dWeldSafeMoveSpeedMmPerMin) || param.dWeldSafeMoveSpeedMmPerMin <= 0.0)
    {
        param.dWeldSafeMoveSpeedMmPerMin = DEFAULT_WELD_SAFE_MOVE_SPEED_MM_PER_MIN;
    }
    if (!std::isfinite(param.dStepOverlapRel))
    {
        param.dStepOverlapRel = 20.0;
    }
    param.dStepOverlapRel = std::max(0.0, param.dStepOverlapRel);
    param.dFinalWeldTrajectoryStepMm = NormalizeFinalWeldTrajectorySampleStepMm(param.dFinalWeldTrajectoryStepMm);
    if (!std::isfinite(param.dResumeBacktrackMm) || param.dResumeBacktrackMm < 0.0)
    {
        param.dResumeBacktrackMm = DEFAULT_RESUME_BACKTRACK_DISTANCE_MM;
    }
    param.dResumeBacktrackMm = std::clamp(param.dResumeBacktrackMm, 0.0, 1000.0);
    param.nWeldDirection = param.nWeldDirection < 0 ? -1 : 1;
    // 焊接方向已迁入工艺参数：当前工艺设置过（非0）时优先于测量参数页旧值，
    // 此处统一覆盖，下游（执行反转/已焊起点截断/预览箭头）全部跟随。
    {
        T_WELD_PARA activeWeld = {};
        if (TryLoadActiveWeldProcessParam(robotName, activeWeld, nullptr)
            && activeWeld.nWeldDirection != 0)
        {
            param.nWeldDirection = activeWeld.nWeldDirection < 0 ? -1 : 1;
        }
    }
    if (!std::isfinite(param.dGunDownBackSafeDis) || param.dGunDownBackSafeDis <= 0.0)
    {
        param.dGunDownBackSafeDis = WELD_SAFE_OFFSET_DISTANCE_MM;
    }
    param.nWeldSafeRetreatDirection =
        NormalizeWeldSafeRetreatDirectionMode(param.nWeldSafeRetreatDirection);
    if (!std::isfinite(param.dWeldRzGainDeg))
    {
        param.dWeldRzGainDeg = 0.0;
    }
    if (!std::isfinite(param.dTaughtWeldPoseRxDeg))
    {
        param.dTaughtWeldPoseRxDeg = 0.0;
    }
    if (!std::isfinite(param.dTaughtWeldPoseRyDeg))
    {
        param.dTaughtWeldPoseRyDeg = 0.0;
    }
    if (!std::isfinite(param.dTaughtWeldPoseRzDeg))
    {
        param.dTaughtWeldPoseRzDeg = 0.0;
    }
    NormalizeSlopeRzClamp(param.dSlopeRzMinDeg, param.dSlopeRzMaxDeg);
    if (!std::isfinite(param.dScanSafeOffsetDistanceMm) || param.dScanSafeOffsetDistanceMm <= 0.0)
    {
        param.dScanSafeOffsetDistanceMm = 150.0;
    }
    if (!std::isfinite(param.dScanSafeGunAngleDeg))
    {
        param.dScanSafeGunAngleDeg = 30.0;
    }
    if (param.nScanSafeXDirection == 0)
    {
        param.nScanSafeXDirection = -1;
    }
    if (!std::isfinite(param.dScanSafeLiftHeightMm) || param.dScanSafeLiftHeightMm < 0.0)
    {
        param.dScanSafeLiftHeightMm = 150.0;
    }
    if (!std::isfinite(param.dScanSafeFlipWarnThresholdDeg) || param.dScanSafeFlipWarnThresholdDeg <= 0.0)
    {
        param.dScanSafeFlipWarnThresholdDeg = 90.0;
    }

    QString pulseError;
    param.bHasStartPulse = ReadPulse(ini, "StartPulse", param.tStartPulse, pulseError);
    if (!param.bHasStartPulse)
    {
        param.tStartPulse = T_ANGLE_PULSE();
        error.clear();
    }

    if (!ReadCoors(ini, "StartPos", param.tStartPos, error)
        || !ReadCoors(ini, "EndPos", param.tEndPos, error))
    {
        return false;
    }

    if (!param.bUseComputedScanSafe)
    {
        if (!ReadPulseList(ini, "StartSafePulseNum", "StartSafePulse", param.vtStartSafePulse, error)
            || !ReadPulseList(ini, "EndSafePulseNum", "EndSafePulse", param.vtEndSafePulse, error))
        {
            return false;
        }
        if (param.vtStartSafePulse.empty() || param.vtEndSafePulse.empty())
        {
            QStringList missingSafePositions;
            if (param.vtStartSafePulse.empty())
            {
                missingSafePositions << QStringLiteral("扫描下枪安全位置");
            }
            if (param.vtEndSafePulse.empty())
            {
                missingSafePositions << QStringLiteral("扫描收枪安全位置");
            }
            error = QStringLiteral(
                "已选择示教安全位，但%1未配置有效脉冲点。为避免静默改用自动计算安全位，流程已中止；"
                "请重新示教并保存，或在测量焊接参数中明确启用自动计算安全位。")
                .arg(missingSafePositions.join(QStringLiteral("、")));
            return false;
        }
    }

    return true;
}

bool MeasureThenWeldService::ResolveWeldExecutionParameters(
    const T_PRECISE_MEASURE_PARAM& param,
    double overrideFinalStepMm,
    QString& fingerprint,
    double& effectiveFinalStepMm,
    QString& error,
    bool* resumeCheckpointSupported,
    QString* resumeUnsupportedReason) const
{
    fingerprint.clear();
    effectiveFinalStepMm = 0.0;
    error.clear();
    if (resumeCheckpointSupported != nullptr)
    {
        *resumeCheckpointSupported = false;
    }
    if (resumeUnsupportedReason != nullptr)
    {
        resumeUnsupportedReason->clear();
    }
    if (!std::isfinite(overrideFinalStepMm) || overrideFinalStepMm < 0.0)
    {
        error = QStringLiteral("最终轨迹点距覆盖值无效。");
        return false;
    }

    const WeldPosePreset preset = LoadWeldPosePreset(param);
    const bool checkpointSupported = !(param.bDoActualWeld
        && (preset.weaveEnabled || preset.trackEnabled));
    const QString unsupportedReason = checkpointSupported
        ? QString()
        : QStringLiteral("当前实际焊接工艺启用了摆焊或焊缝跟踪；FinalSampled仅是中心线，无法证明机器人真实TCP弧长，禁止生成或使用自动续焊断点。");
    if (param.bDoActualWeld && !preset.weldProcessLoaded)
    {
        error = preset.weldProcessLoadError.isEmpty()
            ? QStringLiteral("当前实际焊接工艺未能加载，无法冻结执行参数。")
            : preset.weldProcessLoadError;
        return false;
    }
    if (param.bDoActualWeld && !preset.weldProcessSafetyError.isEmpty())
    {
        error = preset.weldProcessSafetyError;
        return false;
    }
    if (param.bDoActualWeld && preset.transitionCurrentVoltageEnableMismatch)
    {
        error = QStringLiteral("拐点过渡电流和电压启用状态不一致，无法冻结执行参数。");
        return false;
    }

    effectiveFinalStepMm = ResolveEffectiveFinalStepMm(param, preset, overrideFinalStepMm);
    fingerprint = BuildEffectiveWeldExecutionFingerprint(param, preset, effectiveFinalStepMm);
    if (fingerprint.size() != 64 || !std::isfinite(effectiveFinalStepMm) || effectiveFinalStepMm <= 0.0)
    {
        error = QStringLiteral("生成焊接执行参数指纹失败。");
        fingerprint.clear();
        effectiveFinalStepMm = 0.0;
        return false;
    }
    if (resumeCheckpointSupported != nullptr)
    {
        *resumeCheckpointSupported = checkpointSupported;
    }
    if (resumeUnsupportedReason != nullptr)
    {
        *resumeUnsupportedReason = unsupportedReason;
    }
    return true;
}

bool MeasureThenWeldService::ReadPulse(COPini& ini, const std::string& prefix, T_ANGLE_PULSE& pulse, QString& error) const
{
    int bRtn = 1;
    bRtn = (bRtn && ini.ReadString(prefix + ".nS", &pulse.nSPulse) > 0) ? 1 : 0;
    bRtn = (bRtn && ini.ReadString(prefix + ".nL", &pulse.nLPulse) > 0) ? 1 : 0;
    bRtn = (bRtn && ini.ReadString(prefix + ".nU", &pulse.nUPulse) > 0) ? 1 : 0;
    bRtn = (bRtn && ini.ReadString(prefix + ".nR", &pulse.nRPulse) > 0) ? 1 : 0;
    bRtn = (bRtn && ini.ReadString(prefix + ".nB", &pulse.nBPulse) > 0) ? 1 : 0;
    bRtn = (bRtn && ini.ReadString(prefix + ".nT", &pulse.nTPulse) > 0) ? 1 : 0;
    bRtn = (bRtn && ini.ReadString(prefix + ".lBX", &pulse.lBXPulse) > 0) ? 1 : 0;
    bRtn = (bRtn && ini.ReadString(prefix + ".lBY", &pulse.lBYPulse) > 0) ? 1 : 0;
    bRtn = (bRtn && ini.ReadString(prefix + ".lBZ", &pulse.lBZPulse) > 0) ? 1 : 0;
    if (bRtn == 0)
    {
        error = QString("读取脉冲失败：%1").arg(QString::fromStdString(prefix));
        return false;
    }
    return true;
}

bool MeasureThenWeldService::ReadCoors(COPini& ini, const std::string& prefix, T_ROBOT_COORS& coors, QString& error) const
{
    coors = T_ROBOT_COORS();
    QStringList missingKeys;

    auto readRequired = [&ini, &prefix, &missingKeys](const char* suffix, double& value)
        {
            const std::string key = prefix + "." + suffix;
            if (ini.ReadString(false, key, &value) <= 0)
            {
                missingKeys << QString::fromStdString(key);
            }
        };
    auto readOptional = [&ini, &prefix](const char* suffix, double& value)
        {
            const std::string key = prefix + "." + suffix;
            ini.ReadString(false, key, &value);
        };

    readRequired("X", coors.dX);
    readRequired("Y", coors.dY);
    readRequired("Z", coors.dZ);
    readRequired("RX", coors.dRX);
    readRequired("RY", coors.dRY);
    readRequired("RZ", coors.dRZ);
    readOptional("BX", coors.dBX);
    readOptional("BY", coors.dBY);
    readOptional("BZ", coors.dBZ);

    if (!missingKeys.isEmpty())
    {
        error = QString("读取直角坐标失败：%1，缺少 %2，请在“测量焊接参数”里重新示教并保存扫描起点/终点。")
            .arg(QString::fromStdString(prefix), missingKeys.join(", "));
        return false;
    }
    return true;
}

bool MeasureThenWeldService::ReadPulseList(COPini& ini, const std::string& countKey, const std::string& prefix, std::vector<T_ANGLE_PULSE>& pulses, QString& error) const
{
    int count = 0;
    ini.ReadString(false, countKey, &count);
    if (count <= 0)
    {
        pulses.clear();
        return true;
    }

    pulses.clear();
    for (int index = 0; index < count; ++index)
    {
        T_ANGLE_PULSE pulse;
        if (!ReadPulse(ini, prefix + std::to_string(index), pulse, error))
        {
            return false;
        }
        pulses.push_back(pulse);
    }
    return true;
}

bool MeasureThenWeldService::MovePulseAndWait(RobotDriverAdaptor* pRobotDriver, const T_ANGLE_PULSE& pulse, double speed, const QString& name, const LogCallback& appendLog, const StepCallback& setFlowStep) const
{
    if (pRobotDriver == nullptr)
    {
        return false;
    }
    if (setFlowStep)
    {
        setFlowStep(QString("正在运动：%1").arg(name));
    }
    if (appendLog)
    {
        appendLog(QString("开始运动：%1").arg(name));
    }

    const bool moveOk = pRobotDriver->MoveByJob(pulse, T_ROBOT_MOVE_SPEED(speed, 0.0, 0.0), pRobotDriver->m_nExternalAxleType, "MOVJ");
    if (!moveOk)
    {
        if (appendLog)
        {
            const QString detail = RobotMotionStatusText(pRobotDriver);
            appendLog(detail.isEmpty()
                ? QString("运动失败：%1").arg(name)
                : QString("运动失败：%1，%2").arg(name, detail));
        }
        return false;
    }

    return WaitRobotMotionDone(pRobotDriver, name, appendLog);
}

bool MeasureThenWeldService::MovePulseListAndWait(RobotDriverAdaptor* pRobotDriver, const std::vector<T_ANGLE_PULSE>& pulses, double speed, const QString& name, const LogCallback& appendLog, const StepCallback& setFlowStep) const
{
    if (pulses.empty())
    {
        if (appendLog)
        {
            appendLog(QString("运动失败：%1未配置有效脉冲点，禁止按空列表跳过。").arg(name));
        }
        if (setFlowStep)
        {
            setFlowStep(QString("%1未配置").arg(name));
        }
        return false;
    }

    for (size_t index = 0; index < pulses.size(); ++index)
    {
        if (!MovePulseAndWait(pRobotDriver, pulses[index], speed, QString("%1[%2]").arg(name).arg(index), appendLog, setFlowStep))
        {
            return false;
        }
    }
    return true;
}

bool MeasureThenWeldService::MoveCoorsAndWait(RobotDriverAdaptor* pRobotDriver, const T_ROBOT_COORS& coors, double speed, const QString& name, const LogCallback& appendLog, const StepCallback& setFlowStep) const
{
    if (pRobotDriver == nullptr)
    {
        return false;
    }
    const double commandSpeed = LinearCommandSpeedForRobot(pRobotDriver, speed, 1.0);
    const QString commandSpeedUnit = LinearCommandSpeedUnitText(pRobotDriver);
	T_ROBOT_COORS current;
	if (!pRobotDriver->TryGetCurrentPos(current))
	{
		const QString failure = QString("直线运动失败：%1，读取当前位置失败，%2")
			.arg(name, RobotMotionStatusText(pRobotDriver));
		if (appendLog)
		{
			appendLog(failure);
		}
		return false;
	}
	int motionTimeoutMs = 0;
	std::string admissionError;
	if (!RobotMotionTimeoutPolicy::AdmitCartesianMove(
		current,
		coors,
		dynamic_cast<FANUCRobotCtrl*>(pRobotDriver) != nullptr ? commandSpeed * 60.0 : speed,
		motionTimeoutMs,
		&admissionError))
	{
		const QString failure = QString("直线运动已拒绝：%1，%2")
			.arg(name, QString::fromUtf8(admissionError.c_str()));
		pRobotDriver->SetLastRobotError(failure.toUtf8().toStdString());
		if (appendLog)
		{
			appendLog(failure);
		}
		return false;
	}
    if (setFlowStep)
    {
        setFlowStep(QString("正在直线运动：%1").arg(name));
    }
    if (appendLog)
    {
        appendLog(QString("开始直线运动：%1，配置速度= %2 mm/min，下发速度= %3 %4")
            .arg(name)
            .arg(speed, 0, 'f', 3)
            .arg(commandSpeed, 0, 'f', 3)
            .arg(commandSpeedUnit));
    }

    const bool moveOk = pRobotDriver->MoveByJob(coors, T_ROBOT_MOVE_SPEED(commandSpeed, 0.0, 0.0), pRobotDriver->m_nExternalAxleType, "MOVL");
    if (!moveOk)
    {
        if (appendLog)
        {
            const QString detail = RobotMotionStatusText(pRobotDriver);
            appendLog(detail.isEmpty()
                ? QString("直线运动失败：%1").arg(name)
                : QString("直线运动失败：%1，%2").arg(name, detail));
        }
        return false;
    }

    return WaitRobotMotionDone(
		pRobotDriver,
		name,
		appendLog,
		3000,
		motionTimeoutMs,
		100);
}

bool MeasureThenWeldService::VerifyRobotAtSafePose(
    RobotDriverAdaptor* pRobotDriver,
    const T_ROBOT_COORS& expected,
    T_ROBOT_COORS& observed,
    QString& error) const
{
    error.clear();
    if (pRobotDriver == nullptr || !pRobotDriver->TryGetCurrentPos(observed))
    {
        error = QStringLiteral("安全回撤终态验证失败：无法读取机器人当前位置。");
        return false;
    }

    constexpr double kCartesianToleranceMm = 5.0;
    constexpr double kAngularToleranceDeg = 5.0;
    constexpr double kExternalToleranceMm = 5.0;
    const double cartesianError = std::hypot(
        std::hypot(observed.dX - expected.dX, observed.dY - expected.dY),
        observed.dZ - expected.dZ);
    const double angularError = (std::max)({
        WrappedAngleDistanceDeg(observed.dRX, expected.dRX),
        WrappedAngleDistanceDeg(observed.dRY, expected.dRY),
        WrappedAngleDistanceDeg(observed.dRZ, expected.dRZ) });
    const double externalError = std::hypot(
        std::hypot(observed.dBX - expected.dBX, observed.dBY - expected.dBY),
        observed.dBZ - expected.dBZ);
    if (!std::isfinite(cartesianError) || !std::isfinite(angularError) || !std::isfinite(externalError)
        || cartesianError > kCartesianToleranceMm
        || angularError > kAngularToleranceDeg
        || externalError > kExternalToleranceMm)
    {
        error = QString(
            "安全回撤终态验证失败：位置偏差=%1 mm（上限%2），姿态偏差=%3 deg（上限%4），外部轴偏差=%5（上限%6）。Expected=[%7] Observed=[%8]")
            .arg(cartesianError, 0, 'f', 3)
            .arg(kCartesianToleranceMm, 0, 'f', 1)
            .arg(angularError, 0, 'f', 3)
            .arg(kAngularToleranceDeg, 0, 'f', 1)
            .arg(externalError, 0, 'f', 3)
            .arg(kExternalToleranceMm, 0, 'f', 1)
            .arg(RobotCoorsText(expected), RobotCoorsText(observed));
        return false;
    }
    return true;
}

bool MeasureThenWeldService::MoveScanStartSafeAndWait(
    RobotDriverAdaptor* pRobotDriver,
    const T_PRECISE_MEASURE_PARAM& param,
    double speed,
    const LogCallback& appendLog,
    const StepCallback& setFlowStep,
    const CheckpointCallback& checkpoint) const
{
    if (pRobotDriver == nullptr)
    {
        return false;
    }

    if (!param.bUseComputedScanSafe)
    {
        return MovePulseListAndWait(pRobotDriver, param.vtStartSafePulse, speed, "下枪安全姿态", appendLog, setFlowStep);
    }

    const T_ROBOT_COORS startSafeCoors = BuildScanSafeCoorsFromAnchor(param.tStartPos, param);
    if (appendLog)
    {
        appendLog(QString("扫描下枪安全位置按配置推算：起点=%1，安全位=%2，偏移=%3mm，枪角=%4deg，X方向=%5")
            .arg(RobotCoorsText(param.tStartPos))
            .arg(RobotCoorsText(startSafeCoors))
            .arg(param.dScanSafeOffsetDistanceMm, 0, 'f', 3)
            .arg(param.dScanSafeGunAngleDeg, 0, 'f', 3)
            .arg(param.nScanSafeXDirection >= 0 ? "X+" : "X-"));
    }

    if (param.bHasStartPulse)
    {
        T_ANGLE_PULSE currentPulse;
        if (!pRobotDriver->TryGetCurrentPulse(currentPulse))
        {
            if (appendLog)
            {
                appendLog("扫描安全位规划已拒绝：读取当前关节位置失败，"
                    + RobotMotionStatusText(pRobotDriver));
            }
            return false;
        }
        const double maxWristDeltaDeg = MaxWristDeltaDeg(currentPulse, param.tStartPulse, pRobotDriver->m_tAxisUnit);
        const double warnThresholdDeg = param.dScanSafeFlipWarnThresholdDeg > 0.0
            ? param.dScanSafeFlipWarnThresholdDeg
            : 90.0;
        if (maxWristDeltaDeg >= warnThresholdDeg)
        {
            const QString detail = QString(
                "当前关节姿态和扫描起点示教关节姿态差异较大，最大腕部轴差≈%1°，阈值=%2°。\n"
                "为降低姿态翻转时碰撞风险，流程将先原地抬高 %3mm，再在高位切换到扫描起点姿态，最后移动到扫描下枪安全位置。\n"
                "扫描起点关节：R=%4 B=%5 T=%6\n"
                "当前关节：R=%7 B=%8 T=%9")
                .arg(maxWristDeltaDeg, 0, 'f', 3)
                .arg(warnThresholdDeg, 0, 'f', 3)
                .arg(param.dScanSafeLiftHeightMm, 0, 'f', 3)
                .arg(param.tStartPulse.nRPulse)
                .arg(param.tStartPulse.nBPulse)
                .arg(param.tStartPulse.nTPulse)
                .arg(currentPulse.nRPulse)
                .arg(currentPulse.nBPulse)
                .arg(currentPulse.nTPulse);

            if (!checkpoint)
            {
                if (appendLog)
                {
                    appendLog("检测到扫描姿态翻转风险，但当前入口无法完成人工确认，流程已安全中止。");
                }
                return false;
            }
            if (!checkpoint("扫描姿态翻转风险提醒", detail))
            {
                if (appendLog)
                {
                    appendLog("用户取消扫描姿态翻转保护流程。");
                }
                return false;
            }

            T_ROBOT_COORS currentCoors;
            if (!pRobotDriver->TryGetCurrentPos(currentCoors))
            {
                if (appendLog)
                {
                    appendLog("扫描姿态翻转保护已拒绝：读取当前直角位置失败，"
                        + RobotMotionStatusText(pRobotDriver));
                }
                return false;
            }
            const double liftHeight = std::max(0.0, param.dScanSafeLiftHeightMm);
            if (liftHeight > 1e-6)
            {
                T_ROBOT_COORS liftCoors = currentCoors;
                liftCoors.dZ += liftHeight;
                if (!MoveCoorsAndWait(pRobotDriver, liftCoors, speed, "扫描姿态切换抬高点", appendLog, setFlowStep))
                {
                    return false;
                }
                currentCoors = liftCoors;
            }

            T_ROBOT_COORS highPoseCoors = currentCoors;
            highPoseCoors.dRX = param.tStartPos.dRX;
            highPoseCoors.dRY = param.tStartPos.dRY;
            highPoseCoors.dRZ = param.tStartPos.dRZ;
            if (!MoveCoorsAndWait(pRobotDriver, highPoseCoors, speed, "高位切换扫描起点姿态", appendLog, setFlowStep))
            {
                return false;
            }
        }
    }
    else if (appendLog)
    {
        appendLog("扫描起点关节脉冲未配置，跳过姿态翻转风险判断，仅按直角位姿推算安全位。");
    }

    return MoveCoorsAndWait(pRobotDriver, startSafeCoors, speed, "扫描下枪安全位置", appendLog, setFlowStep);
}

bool MeasureThenWeldService::MoveScanEndSafeAndWait(
    RobotDriverAdaptor* pRobotDriver,
    const T_PRECISE_MEASURE_PARAM& param,
    double speed,
    const LogCallback& appendLog,
    const StepCallback& setFlowStep) const
{
    if (pRobotDriver == nullptr)
    {
        return false;
    }

    if (!param.bUseComputedScanSafe)
    {
        return MovePulseListAndWait(pRobotDriver, param.vtEndSafePulse, speed, "收枪姿态", appendLog, setFlowStep);
    }

    const T_ROBOT_COORS endSafeCoors = BuildScanSafeCoorsFromAnchor(param.tEndPos, param);
    if (appendLog)
    {
        appendLog(QString("扫描收枪安全位置按配置推算：终点=%1，安全位=%2")
            .arg(RobotCoorsText(param.tEndPos))
            .arg(RobotCoorsText(endSafeCoors)));
    }
    return MoveCoorsAndWait(pRobotDriver, endSafeCoors, speed, "扫描收枪安全位置", appendLog, setFlowStep);
}

bool MeasureThenWeldService::RunScanCycle(
    RobotDriverAdaptor* pRobotDriver,
    const T_PRECISE_MEASURE_PARAM& param,
    double runSpeed,
    CameraFrameCache* cameraCache,
    ScanCycleResult& result,
    const LogCallback& appendLog,
    const StepCallback& setFlowStep,
    const CheckpointCallback& safetyCheckpoint,
    const BeforeActionCallback& beforeAction,
    const StopRequestedCallback& stopRequested,
    const ScanProgressCallback& scanProgressCallback,
    const ScanPauseAvailabilityCallback& scanPauseAvailability) const
{
    result = ScanCycleResult{};

    auto fail = [&](const QString& error, bool motionFailure) -> bool
        {
            result.status = ScanCycleStatus::Failed;
            result.error = error;
            result.motionFailure = motionFailure;
            if (appendLog && !error.isEmpty())
            {
                appendLog(error);
            }
            return false;
        };
    auto allowAction = [&](const QString& action) -> bool
        {
            if (RobotOperationLease::IsCancellationRequested(pRobotDriver))
            {
                result.status = ScanCycleStatus::Stopped;
                result.error = QString("安全停止已取消当前硬件流程，禁止在%1前继续下发。").arg(action);
                if (appendLog)
                {
                    appendLog(result.error);
                }
                return false;
            }
            if (stopRequested && stopRequested())
            {
                result.status = ScanCycleStatus::Stopped;
                result.error = QString("已在%1前停止。").arg(action);
                if (appendLog)
                {
                    appendLog(result.error);
                }
                return false;
            }
            if (beforeAction && !beforeAction(action))
            {
                result.status = ScanCycleStatus::Stopped;
                result.error = QString("已取消：%1。").arg(action);
                if (appendLog)
                {
                    appendLog(result.error);
                }
                return false;
            }
            return true;
        };

    if (pRobotDriver == nullptr)
    {
        return fail("扫描循环失败：机器人驱动为空。", false);
    }
    if (cameraCache == nullptr)
    {
        return fail("扫描循环失败：当前机器人没有可用的专属相机缓存。", false);
    }
    if (!param.bUseComputedScanSafe
        && (param.vtStartSafePulse.empty() || param.vtEndSafePulse.empty()))
    {
        result.fatalFailure = true;
        return fail(
            QStringLiteral("扫描前置检查失败：已选择示教安全位，但下枪或收枪安全脉冲列表为空；"
                "禁止静默改用自动计算安全位。"),
            false);
    }

    const std::uint64_t readyFrameMark = cameraCache->Mark();
    QString readyFrameError;
    if (!cameraCache->WaitForReadyFrameAfter(
        readyFrameMark, CAMERA_READY_FRAME_TIMEOUT_MS, &readyFrameError))
    {
        result.fatalFailure = true;
        return fail(
            QString("扫描前置检查失败：当前扫描周期未取得新鲜有效相机帧：%1").arg(readyFrameError),
            false);
    }
    if (appendLog)
    {
        appendLog(QStringLiteral("扫描前置检查通过：当前扫描周期已取得新鲜有效相机帧。"));
    }

    HandEyeMatrixConfig validatedCalibration;
    QString calibrationError;
    QString calibrationPath;
    const QString robotName = QString::fromStdString(param.sRobotName);
    const QString cameraSection = RobotDataHelper::MeasureCameraSection(robotName);
    if (!LoadExistingValidatedHandEyeMatrixConfig(
        robotName,
        cameraSection,
        validatedCalibration,
        &calibrationError,
        &calibrationPath))
    {
        result.fatalFailure = true;
        return fail(
            QString("扫描前置检查失败：手眼矩阵不可用：%1 [%2]")
                .arg(calibrationError, cameraSection),
            false);
    }
    if (appendLog)
    {
        appendLog(QString("扫描前置检查通过：手眼矩阵=%1 [%2]")
            .arg(calibrationPath, cameraSection));
    }

    const double safeRunSpeed = std::isfinite(runSpeed) && runSpeed > 0.0 ? runSpeed : 1.0;
	if (dynamic_cast<FANUCRobotCtrl*>(pRobotDriver) != nullptr)
	{
		const double safeCommandSpeed = LinearCommandSpeedForRobot(
			pRobotDriver, safeRunSpeed, 0.0);
		const double scanCommandSpeed = LinearCommandSpeedForRobot(
			pRobotDriver, param.dScanSpeed, 0.0);
		QStringList unrepresentableSpeeds;
		if (safeCommandSpeed < 1.0)
		{
			unrepresentableSpeeds << QString("安全移动速度=%1mm/min")
				.arg(safeRunSpeed, 0, 'f', 3);
		}
		if (scanCommandSpeed < 1.0)
		{
			unrepresentableSpeeds << QString("扫描速度=%1mm/min")
				.arg(param.dScanSpeed, 0, 'f', 3);
		}
		if (!unrepresentableSpeeds.isEmpty())
		{
			result.fatalFailure = true;
			return fail(
				QString("FANUC扫描流程已在首条运动前拒绝：固定TP最低只能表示1mm/sec（60mm/min），"
					"禁止把低速静默替换为更高速度。不可表示项：%1")
					.arg(unrepresentableSpeeds.join("，")),
				false);
		}
	}
    if (!allowAction("扫描下枪安全位置"))
    {
        return false;
    }
    bool safetyCheckpointRejected = false;
    CheckpointCallback trackedSafetyCheckpoint;
    if (safetyCheckpoint)
    {
        trackedSafetyCheckpoint = [&](const QString& title, const QString& detail) -> bool
            {
                const bool approved = safetyCheckpoint(title, detail);
                safetyCheckpointRejected = !approved;
                return approved;
            };
    }
    if (!MoveScanStartSafeAndWait(
        pRobotDriver, param, safeRunSpeed, appendLog, setFlowStep, trackedSafetyCheckpoint))
    {
        if (safetyCheckpointRejected)
        {
            result.status = ScanCycleStatus::Stopped;
            result.fatalFailure = true;
            result.error = QStringLiteral("扫描姿态安全风险未获人工确认，未发送后续运动。");
            if (appendLog)
            {
                appendLog(result.error);
            }
            return false;
        }
        return fail("扫描循环失败：未能到达扫描下枪安全位置。", true);
    }
    result.lastPhase = ScanCyclePhase::AtStartSafe;

    if (!allowAction("移动到扫描起点"))
    {
        return false;
    }
    if (!MoveCoorsAndWait(
        pRobotDriver, param.tStartPos, safeRunSpeed, "扫描起点", appendLog, setFlowStep))
    {
        return fail("扫描循环失败：未能到达扫描起点。", true);
    }
    result.lastPhase = ScanCyclePhase::AtScanStart;

    if (!allowAction("扫描终点并采集相机点"))
    {
        return false;
    }
    ScanMotionProgress scanProgress;
    result.scanAttempted = true;
    QString scanOutputPath;
    const bool scanOk = ScanMoveAndCollect(
        pRobotDriver,
        param,
        scanOutputPath,
        appendLog,
        setFlowStep,
        cameraCache,
        &scanProgress,
        &validatedCalibration,
        scanProgressCallback,
        scanPauseAvailability);
    if (!scanOutputPath.isEmpty())
    {
        const QFileInfo outputInfo(scanOutputPath);
        if (outputInfo.isFile())
        {
            result.weldPosePath = outputInfo.absoluteFilePath();
            result.poseGenerated = true;
            QDir caseDir = outputInfo.dir();  // LaserPoint
            if (caseDir.cdUp())
            {
                result.caseDir = caseDir.absolutePath();
            }
        }
        else
        {
            const QDir outputDir(scanOutputPath);
            if (outputDir.exists())
            {
                result.caseDir = outputDir.absolutePath();
            }
        }
    }
    result.scanDataSucceeded = scanOk;
    if (stopRequested && stopRequested())
    {
        result.stopRequestedDuringCycle = true;
    }
    if (scanProgress.motionStarted)
    {
        result.lastPhase = ScanCyclePhase::ScanMotionStarted;
    }
    if (scanProgress.motionCompleted)
    {
        result.lastPhase = ScanCyclePhase::AtScanEnd;
    }

    if (RobotOperationLease::IsCancellationRequested(pRobotDriver))
    {
        result.status = ScanCycleStatus::Stopped;
        result.stopRequestedDuringCycle = true;
        result.error = QStringLiteral("扫描期间收到安全停止；已丢弃本轮结果，并禁止自动收枪或后续焊接运动。");
        if (appendLog)
        {
            appendLog(result.error);
        }
        return false;
    }

    if (!scanProgress.motionCompleted)
    {
        return fail(
            scanOk
                ? QStringLiteral("扫描循环失败：扫描接口返回成功，但未确认机器人到达扫描终点。")
                : QStringLiteral("扫描循环失败：扫描运动未确认完成，禁止继续运动或进入下一轮。"),
            true);
    }

    const bool scanDataOk = scanOk;
    // 一旦确认机器人到达扫描终点，安全收枪就是强制收尾步骤：停止请求、后处理失败和
    // GUI 普通确认回调都不得把机器人留在扫描终点。需要立即制止运动时应使用机器人急停，
    // 而不是让软件在已完成扫描后跳过配置好的安全撤离路径。
    if (!MoveScanEndSafeAndWait(pRobotDriver, param, safeRunSpeed, appendLog, setFlowStep))
    {
        return fail("扫描循环失败：未能到达扫描收枪安全位置。", true);
    }
    result.lastPhase = ScanCyclePhase::AtEndSafe;
    result.safelyRetracted = true;
    if (stopRequested && stopRequested())
    {
        result.stopRequestedDuringCycle = true;
    }

    if (!scanDataOk)
    {
        return fail("扫描运动已完成并安全收枪，但采集、处理或文件保存失败。", false);
    }

    if (result.stopRequestedDuringCycle)
    {
        result.status = ScanCycleStatus::Stopped;
        result.error = QStringLiteral("本次扫描和安全收枪已完成；已按停止请求取消焊接及后续循环。");
        if (appendLog)
        {
            appendLog(result.error);
        }
        return false;
    }

    result.status = ScanCycleStatus::Success;
    result.error.clear();
    return true;
}

bool MeasureThenWeldService::ScanMoveAndCollect(
    RobotDriverAdaptor* pRobotDriver,
    const T_PRECISE_MEASURE_PARAM& param,
    QString& savedPath,
    const LogCallback& appendLog,
    const StepCallback& setFlowStep,
    CameraFrameCache* cameraCache,
    ScanMotionProgress* progress,
    const HandEyeMatrixConfig* validatedCalibration,
    const ScanProgressCallback& scanProgressCallback,
    const ScanPauseAvailabilityCallback& scanPauseAvailability) const
{
    if (progress != nullptr)
    {
        *progress = ScanMotionProgress{};
    }
    if (pRobotDriver == nullptr)
    {
        return false;
    }
    if (cameraCache == nullptr)
    {
        appendLog("扫描失败：当前机器人没有可用的专属相机缓存。");
        setFlowStep("扫描失败：相机缓存未初始化");
        return false;
    }
    CameraFrameCache* frameCache = cameraCache;
    FANUCRobotCtrl* pFanucDriver = dynamic_cast<FANUCRobotCtrl*>(pRobotDriver);
    STEPRobotCtrl* pStepDriver = dynamic_cast<STEPRobotCtrl*>(pRobotDriver);
    bool scanPausePublished = false;
    auto publishScanPauseAvailability = [&](bool available, const QString& programName = QString())
        {
            if (!scanPauseAvailability)
            {
                return;
            }
            if (!available && !scanPausePublished)
            {
                return;
            }
            scanPauseAvailability(available, available ? programName : QString());
            scanPausePublished = available;
        };
    struct ScanPauseScopeGuard
    {
        std::function<void()> cleanup;
        ~ScanPauseScopeGuard()
        {
            if (cleanup)
            {
                cleanup();
            }
        }
    } scanPauseScopeGuard{ [&]() { publishScanPauseAvailability(false); } };
    const double scanCommandSpeed = LinearCommandSpeedForRobot(pRobotDriver, param.dScanSpeed, 1.0);
    const QString scanCommandSpeedUnit = LinearCommandSpeedUnitText(pRobotDriver);
    const qint64 cameraTimeOffsetUs = static_cast<qint64>(std::llround(param.dCameraTimeOffsetMs * 1000.0));
    const MeasureThenWeldRuntimeConfig::ScanTimestampSource scanTimestampSource =
        MeasureThenWeldRuntimeConfig::LoadScanTimestampSource();
    const bool useRobotTimestampForScan =
        scanTimestampSource != MeasureThenWeldRuntimeConfig::ScanTimestampSource::Pc;
    const QString scanTimestampSourceName = MeasureThenWeldRuntimeConfig::DisplayName(scanTimestampSource);
    const QString scanTimestampFieldName = MeasureThenWeldRuntimeConfig::FieldName(scanTimestampSource);
    if (setFlowStep)
    {
        setFlowStep("扫描运动中，正在采集相机点、机器人位置和激光点");
    }

    HandEyeMatrixConfig calibration;
    QString calibrationError;
    QString calibrationPath;
    const QString cameraSection = RobotDataHelper::MeasureCameraSection(QString::fromStdString(param.sRobotName));
    if (validatedCalibration != nullptr)
    {
        calibration = *validatedCalibration;
        if (appendLog)
        {
            appendLog(QString("使用扫描前置检查已锁定的手眼矩阵 [%1]").arg(cameraSection));
        }
    }
    else if (LoadExistingValidatedHandEyeMatrixConfig(
        QString::fromStdString(param.sRobotName),
        cameraSection,
        calibration,
        &calibrationError,
        &calibrationPath))
    {
        if (appendLog)
        {
            appendLog(QString("已读取并校验手眼矩阵参数：%1 [%2]").arg(calibrationPath, cameraSection));
        }
    }
    else
    {
        if (appendLog)
        {
            appendLog(QString("读取或校验手眼矩阵失败，禁止扫描：%1 [%2]").arg(calibrationError, cameraSection));
        }
        if (setFlowStep)
        {
            setFlowStep("扫描失败：手眼矩阵不可用");
        }
        return false;
    }

    const PointCloudProductionContext productionContext =
        BuildLivePointCloudProductionContext(pRobotDriver, cameraSection, calibration);
    if (productionContext.robotEndpoint.isEmpty())
    {
        if (appendLog)
        {
            appendLog(QStringLiteral(
                "当前机器人没有有效的持久 TCP 端点，本次扫描不能形成生产质量证明。"));
        }
        if (setFlowStep)
        {
            setFlowStep(QStringLiteral("扫描失败：机器人持久端点无效"));
        }
        return false;
    }
    PointCloudProductionExpectation productionExpectation;
    productionExpectation.robotName = QString::fromStdString(param.sRobotName).trimmed();
    productionExpectation.robotEndpoint = productionContext.robotEndpoint;
    productionExpectation.cameraSection = productionContext.cameraSection;
    productionExpectation.handEyeSha256 = productionContext.handEyeSha256;

    // 相机读取帧率已迁至相机参数(CameraParam.ini 的 CameraReadFps)；这里读出来仅用于相机时间戳
    // 跳变告警阈值与日志显示——真正驱动取帧节奏的是相机 worker 的轮询定时器（按 1000/帧率 的间隔轮询）。
    RobotDataHelper::CameraParamData cameraParamForScan;
    int cameraReadFpsConfig = DEFAULT_CAMERA_READ_FPS;
    if (RobotDataHelper::LoadCameraParam(QString::fromStdString(param.sRobotName), cameraSection, cameraParamForScan, nullptr))
    {
        bool okFps = false;
        const int parsedFps = cameraParamForScan.readFps.trimmed().toInt(&okFps);
        if (okFps && parsedFps > 0)
        {
            cameraReadFpsConfig = parsedFps;
        }
    }
    const qint64 cameraReadIntervalMs = std::max<qint64>(1, static_cast<qint64>(std::llround(1000.0 / cameraReadFpsConfig)));
    const double actualCameraReadFps = 1000.0 / static_cast<double>(cameraReadIntervalMs);

    frameCache->Clear();
    frameCache->ClearPollStatus();  // 扫描开始重置 SDK 取帧调用日志；此后到快照期间的记录不再被运动后清帧连带清掉

    // 相机图像随扫描后台采集（SDK v1.2.0 图像传输）：先存进临时目录（结果目录序号在写盘时才分配），
    // 写盘阶段整体挪到 Result/<案例>/CameraImage/。RAII 保证任何提前 return 都会关闭采集窗口。
    // 开关与抽帧间隔来自相机参数（ImageCaptureEnable / ImageCaptureFrameStride），关闭时完全不开窗。
    const bool imageCaptureEnabled = cameraParamForScan.imageCaptureEnable.trimmed() != QStringLiteral("0");
    bool okImageStride = false;
    int imageCaptureStride = cameraParamForScan.imageCaptureStride.trimmed().toInt(&okImageStride);
    if (!okImageStride || imageCaptureStride <= 0)
    {
        imageCaptureStride = 5;
    }
    QString imageCaptureTmpDir;
    if (imageCaptureEnabled)
    {
        imageCaptureTmpDir = QDir(QStringLiteral("Temp")).filePath(
            QStringLiteral("CameraImages_%1").arg(QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss_zzz")));
        QDir().mkpath(imageCaptureTmpDir);
        frameCache->SetImageCaptureDir(imageCaptureTmpDir, imageCaptureStride);
        if (appendLog)
        {
            appendLog(QString("相机图像采集已开启：抽帧间隔=%1（每 %1 张存 1 张，WebP），关闭/调整见相机参数页。").arg(imageCaptureStride));
        }
    }
    struct ImageCaptureScopeGuard
    {
        CameraFrameCache* cache;
        ~ImageCaptureScopeGuard()
        {
            if (cache != nullptr)
            {
                cache->SetImageCaptureDir(QString());
            }
        }
    } imageCaptureScopeGuard{ frameCache };

    std::vector<RobotCalculation::TimestampedRobotPose> robotSamples;
    robotSamples.reserve(1000);
    // (所选时间轴时间戳, PC接收时刻) 样本对：扫描结束后统计估计「PC钟→机器人轴」偏移用（方案A统计对齐）。
    // 轴=PC接收时间时两者相等（中位差恒0），统一收集不区分。与 robotSamples 共用一把锁。
    std::vector<std::pair<qint64, qint64>> robotAxisPcPairsUs;
    robotAxisPcPairsUs.reserve(1000);
    std::mutex robotSamplesMutex;
    std::condition_variable robotSamplesCv;

    std::deque<QueuedScanCameraFrame> pendingCameraFrames;
    std::mutex pendingCameraFramesMutex;
    std::condition_variable pendingCameraFramesCv;
    std::atomic_bool cameraEnqueueFinished(false);

    std::vector<ProcessedScanCameraSample> processedCameraSamples;
    std::mutex processedCameraSamplesMutex;

    std::vector<TimestampedCameraPoint> cameraSamples;
    std::vector<TimestampedCameraPoint> matchedCameraSamples;
    QVector<RobotCalculation::IndexedPoint3D> laserFitInput;
    QVector<RobotCalculation::IndexedPoint3D> workpieceCloudInput;
    cameraSamples.reserve(10000);
    matchedCameraSamples.reserve(10000);
    laserFitInput.reserve(10000);
    workpieceCloudInput.reserve(100000);
    long long lastRobotMonitorMs = std::numeric_limits<long long>::min();
    bool passiveRobotSamplingActive = false;
    int invalidCameraTimestampCount = 0;
    int cameraBeforeRobotTimeBaseCount = 0;
    int cameraTimestampBackwardsCount = 0;
    int cameraTimestampJumpCount = 0;
    int enqueuedCameraSampleCount = 0;
    qint64 lastCameraRawTimestampUs = 0;
    qint64 maxCameraRawDeltaUs = 0;
    const qint64 cameraTimestampJumpWarnUs = std::max<qint64>(50000, cameraReadIntervalMs * 4000);
    bool hasCameraToRobotTimeOffset = false;
    qint64 cameraToRobotTimeOffsetUs = 0;
    qint64 firstCameraRawTimestampUs = 0;
    qint64 firstRobotTimestampUs = 0;
    bool hasCameraTimeBaseRobotTimestamp = false;
    qint64 cameraTimeBaseRobotTimestampUs = 0;
    qint64 latestEnqueuedCameraTimestampUs = 0;
    std::uint64_t scanStartCameraSequence = 0;
    std::uint64_t scanEndCameraSequence = 0;
    std::uint64_t lastPulledCameraSequence = 0;

    auto appendCameraFrame = [
        &pendingCameraFrames,
        &pendingCameraFramesMutex,
        &pendingCameraFramesCv,
        cameraTimeOffsetUs,
        &invalidCameraTimestampCount,
        &cameraBeforeRobotTimeBaseCount,
        &cameraTimestampBackwardsCount,
        &cameraTimestampJumpCount,
        &lastCameraRawTimestampUs,
        &maxCameraRawDeltaUs,
        cameraTimestampJumpWarnUs,
        &hasCameraToRobotTimeOffset,
        &cameraToRobotTimeOffsetUs,
        &firstCameraRawTimestampUs,
        &firstRobotTimestampUs,
        &hasCameraTimeBaseRobotTimestamp,
        &cameraTimeBaseRobotTimestampUs,
        &latestEnqueuedCameraTimestampUs,
        &enqueuedCameraSampleCount](const udpDataShow& frame)
        {
            const qint64 rawTimestampUs = static_cast<qint64>(frame.timestamp);
            if (rawTimestampUs <= 0)
            {
                ++invalidCameraTimestampCount;
                return;
            }
            qint64 rawDeltaUs = 0;
            if (lastCameraRawTimestampUs > 0)
            {
                rawDeltaUs = rawTimestampUs - lastCameraRawTimestampUs;
                if (rawDeltaUs <= 0)
                {
                    ++cameraTimestampBackwardsCount;
                }
                else if (rawDeltaUs > cameraTimestampJumpWarnUs)
                {
                    ++cameraTimestampJumpCount;
                    maxCameraRawDeltaUs = std::max(maxCameraRawDeltaUs, rawDeltaUs);
                }
            }
            lastCameraRawTimestampUs = rawTimestampUs;
            if (!hasCameraToRobotTimeOffset)
            {
                if (!hasCameraTimeBaseRobotTimestamp)
                {
                    ++cameraBeforeRobotTimeBaseCount;
                    return;
                }

                firstCameraRawTimestampUs = rawTimestampUs;
                firstRobotTimestampUs = cameraTimeBaseRobotTimestampUs;
                cameraToRobotTimeOffsetUs = firstRobotTimestampUs - firstCameraRawTimestampUs;
                hasCameraToRobotTimeOffset = true;
            }

            {
                std::lock_guard<std::mutex> locker(pendingCameraFramesMutex);
                QueuedScanCameraFrame queuedFrame;
                queuedFrame.sampleIndex = ++enqueuedCameraSampleCount;
                queuedFrame.rawTimestampUs = rawTimestampUs;
                queuedFrame.rawDeltaUs = rawDeltaUs;
                queuedFrame.timestampUs = rawTimestampUs + cameraToRobotTimeOffsetUs + cameraTimeOffsetUs;
                queuedFrame.frame = frame;
                latestEnqueuedCameraTimestampUs = queuedFrame.timestampUs;
                pendingCameraFrames.push_back(std::move(queuedFrame));
            }
            pendingCameraFramesCv.notify_one();
        };

    auto appendRobotPose = [
        &robotSamples,
        &robotAxisPcPairsUs,
        &robotSamplesMutex,
        &robotSamplesCv,
        pRobotDriver,
        &lastRobotMonitorMs,
        &passiveRobotSamplingActive,
        useRobotTimestampForScan]()
        {
            RobotCalculation::TimestampedRobotPose sample;
            RobotDriverAdaptor::StateSnapshot snapshot;
            bool hasAxisPcPair = false;
            qint64 pairAxisUs = 0;
            qint64 pairPcUs = 0;
            if (pRobotDriver->LatestStateSnapshot(snapshot) && snapshot.valid)
            {
                const long long selectedTimestampMs = useRobotTimestampForScan
                    ? snapshot.robotMs
                    : snapshot.pcRecvMs;
                if (selectedTimestampMs > 0
                    && passiveRobotSamplingActive
                    && selectedTimestampMs == lastRobotMonitorMs)
                {
					// 状态监控通常慢于扫描主循环；重复看到同一有效被动帧只表示
					// “暂无新样本”，不是读取失败。保持成功但不重复入队，下一轮等待新帧。
                    return true;
                }

                if (selectedTimestampMs > 0)
                {
                    const bool usableSnapshotPose = RobotMotionTimeoutPolicy::IsFinitePose(snapshot.pose)
                        && (std::abs(snapshot.pose.dX) >= 1e-12
                            || std::abs(snapshot.pose.dY) >= 1e-12
                            || std::abs(snapshot.pose.dZ) >= 1e-12
                            || std::abs(snapshot.pose.dRX) >= 1e-12
                            || std::abs(snapshot.pose.dRY) >= 1e-12
                            || std::abs(snapshot.pose.dRZ) >= 1e-12
                            || std::abs(snapshot.pose.dBX) >= 1e-12
                            || std::abs(snapshot.pose.dBY) >= 1e-12
                            || std::abs(snapshot.pose.dBZ) >= 1e-12);
                    if (usableSnapshotPose)
                    {
                        sample.pose = snapshot.pose;
                        sample.timestampUs = static_cast<qint64>(selectedTimestampMs) * 1000;
                        lastRobotMonitorMs = selectedTimestampMs;
                        passiveRobotSamplingActive = true;
                        if (snapshot.pcRecvMs > 0)
                        {
                            hasAxisPcPair = true;
                            pairAxisUs = sample.timestampUs;
                            pairPcUs = static_cast<qint64>(snapshot.pcRecvMs) * 1000;
                        }
                    }
                }
            }
            if (sample.timestampUs <= 0)
            {
				// 本轮若锁定机器人时间轴，主动GET_CUR_POS只有PC接收时刻，不能把
				// steady-clock纪元混入robot_ms序列；拿不到同帧被动时间戳就失败关闭。
				if (useRobotTimestampForScan)
				{
					return false;
				}
                T_ROBOT_COORS strictPose;
                if (!pRobotDriver->TryGetCurrentPos(strictPose)
                    || !RobotMotionTimeoutPolicy::IsFinitePose(strictPose)
                    || (std::abs(strictPose.dX) < 1e-12
                        && std::abs(strictPose.dY) < 1e-12
                        && std::abs(strictPose.dZ) < 1e-12
                        && std::abs(strictPose.dRX) < 1e-12
                        && std::abs(strictPose.dRY) < 1e-12
                        && std::abs(strictPose.dRZ) < 1e-12
                        && std::abs(strictPose.dBX) < 1e-12
                        && std::abs(strictPose.dBY) < 1e-12
                        && std::abs(strictPose.dBZ) < 1e-12))
                {
                    return false;
                }
                sample.pose = strictPose;
                sample.timestampUs = SteadyNowUs();
            }

            {
                std::lock_guard<std::mutex> locker(robotSamplesMutex);
                robotSamples.push_back(sample);
                if (hasAxisPcPair)
                {
                    robotAxisPcPairsUs.emplace_back(pairAxisUs, pairPcUs);
                }
            }
            robotSamplesCv.notify_all();
            return true;
        };

    auto latestRobotTimestampUs = [&robotSamples, &robotSamplesMutex]()
        {
            std::lock_guard<std::mutex> locker(robotSamplesMutex);
            return robotSamples.empty() ? 0 : robotSamples.back().timestampUs;
        };

    int lastReportedScanPercent = -1;
    auto reportScanProgress = [&](bool forceComplete = false)
        {
            if (!scanProgressCallback)
            {
                return;
            }
            double ratio = forceComplete ? 1.0 : 0.0;
            if (!forceComplete)
            {
                T_ROBOT_COORS currentPose{};
                bool hasPose = false;
                {
                    std::lock_guard<std::mutex> locker(robotSamplesMutex);
                    if (!robotSamples.empty())
                    {
                        currentPose = robotSamples.back().pose;
                        hasPose = true;
                    }
                }
                const double vx = param.tEndPos.dX - param.tStartPos.dX;
                const double vy = param.tEndPos.dY - param.tStartPos.dY;
                const double vz = param.tEndPos.dZ - param.tStartPos.dZ;
                const double lengthSquared = vx * vx + vy * vy + vz * vz;
                if (hasPose && lengthSquared > 1e-9)
                {
                    const double px = currentPose.dX - param.tStartPos.dX;
                    const double py = currentPose.dY - param.tStartPos.dY;
                    const double pz = currentPose.dZ - param.tStartPos.dZ;
                    ratio = std::clamp(
                        (px * vx + py * vy + pz * vz) / lengthSquared,
                        0.0,
                        1.0);
                }
            }
            const int percent = std::clamp(
                static_cast<int>(std::floor(ratio * 100.0 + 1e-9)), 0, 100);
            if (forceComplete || percent > lastReportedScanPercent)
            {
                lastReportedScanPercent = (std::max)(lastReportedScanPercent, percent);
                scanProgressCallback(static_cast<double>(lastReportedScanPercent) / 100.0);
            }
        };

    auto pullScanCameraFramesTo = [frameCache, &lastPulledCameraSequence, &appendCameraFrame](std::uint64_t targetSequence)
        {
            if (targetSequence <= lastPulledCameraSequence)
            {
                return;
            }

            const std::vector<udpDataShow> frames = frameCache->FramesBetween(
                lastPulledCameraSequence,
                targetSequence);
            for (const udpDataShow& frame : frames)
            {
                appendCameraFrame(frame);
            }
            lastPulledCameraSequence = targetSequence;
        };

    auto pullScanCameraFrames = [frameCache, &pullScanCameraFramesTo]()
        {
            pullScanCameraFramesTo(frameCache->Mark());
        };

    const unsigned int hardwareThreads = std::thread::hardware_concurrency();
    const int processingWorkerCount = std::max(
        1,
        std::min(4, static_cast<int>(hardwareThreads > 1 ? hardwareThreads - 1 : 1)));
    const qint64 processingWallStartMs = SteadyNowMs();
    std::vector<std::thread> processingWorkers;
    processingWorkers.reserve(static_cast<std::size_t>(processingWorkerCount));
    for (int workerIndex = 0; workerIndex < processingWorkerCount; ++workerIndex)
    {
        processingWorkers.emplace_back([&]()
            {
                while (true)
                {
                    QueuedScanCameraFrame queuedFrame;
                    {
                        std::unique_lock<std::mutex> locker(pendingCameraFramesMutex);
                        pendingCameraFramesCv.wait(locker, [&]()
                            {
                                return !pendingCameraFrames.empty() || cameraEnqueueFinished.load();
                            });
                        if (pendingCameraFrames.empty())
                        {
                            if (cameraEnqueueFinished.load())
                            {
                                break;
                            }
                            continue;
                        }

                        queuedFrame = std::move(pendingCameraFrames.front());
                        pendingCameraFrames.pop_front();
                    }

                    ProcessedScanCameraSample processed;
                    processed.sample.sampleIndex = queuedFrame.sampleIndex;
                    processed.sample.rawTimestampUs = queuedFrame.rawTimestampUs;
                    processed.sample.rawDeltaUs = queuedFrame.rawDeltaUs;
                    processed.sample.timestampUs = queuedFrame.timestampUs;
                    processed.sample.point = Eigen::Vector3d(
                        queuedFrame.frame.targetPoint.x,
                        queuedFrame.frame.targetPoint.y,
                        queuedFrame.frame.targetPoint.z);
                    processed.sample.error = queuedFrame.frame.errorMessage;

                    std::vector<RobotCalculation::TimestampedRobotPose> interpolationSamples;
                    {
                        std::unique_lock<std::mutex> locker(robotSamplesMutex);
                        while (true)
                        {
                            if (robotSamples.empty())
                            {
                                if (cameraEnqueueFinished.load())
                                {
                                    processed.status = "unmatched_no_robot_sample";
                                    break;
                                }
                            }
                            else if (queuedFrame.timestampUs < robotSamples.front().timestampUs)
                            {
                                processed.status = "unmatched_before_robot";
                                break;
                            }
                            else if (queuedFrame.timestampUs <= robotSamples.back().timestampUs)
                            {
                                processed.robotWindow = FindRobotInterpolationWindow(robotSamples, queuedFrame.timestampUs);
                                if (processed.robotWindow.prevIndex > 0)
                                {
                                    const int prevZeroIndex = std::clamp(
                                        processed.robotWindow.prevIndex - 1,
                                        0,
                                        static_cast<int>(robotSamples.size()) - 1);
                                    const int nextZeroIndex = std::clamp(
                                        processed.robotWindow.nextIndex - 1,
                                        0,
                                        static_cast<int>(robotSamples.size()) - 1);
                                    interpolationSamples.push_back(robotSamples[static_cast<std::size_t>(prevZeroIndex)]);
                                    if (nextZeroIndex != prevZeroIndex)
                                    {
                                        interpolationSamples.push_back(robotSamples[static_cast<std::size_t>(nextZeroIndex)]);
                                    }
                                }
                                break;
                            }
                            else if (cameraEnqueueFinished.load())
                            {
                                processed.status = "unmatched_after_robot";
                                break;
                            }

                            robotSamplesCv.wait_for(locker, std::chrono::milliseconds(CAMERA_ROBOT_MATCH_TAIL_POLL_MS));
                        }
                    }

                    if (!interpolationSamples.empty())
                    {
                        processed.robotPose = RobotCalculation::InterpolateRobotPose(interpolationSamples, queuedFrame.timestampUs);
                        processed.hasRobotPose = true;
                        processed.contributedWorkpieceFrame = !queuedFrame.frame.allResultPoint.empty();
                        processed.workpiecePoints.reserve(queuedFrame.frame.allResultPoint.size());
                        for (int linePointIndex = 0; linePointIndex < static_cast<int>(queuedFrame.frame.allResultPoint.size()); ++linePointIndex)
                        {
                            const cv::Point3d& sourcePoint = queuedFrame.frame.allResultPoint[static_cast<std::size_t>(linePointIndex)];
                            // allResultPoint 的 Z 轴符号与 targetPoint 相反；生成工件点云前先统一到 targetPoint 使用的相机坐标约定。
                            const Eigen::Vector3d cameraLinePoint(sourcePoint.x, sourcePoint.y, -sourcePoint.z);
                            constexpr double kZeroPointEps = 1e-9;
                            const bool isZeroPoint =
                                std::abs(cameraLinePoint.x()) <= kZeroPointEps
                                && std::abs(cameraLinePoint.y()) <= kZeroPointEps
                                && std::abs(cameraLinePoint.z()) <= kZeroPointEps;
                            if (!IsFiniteCameraPoint(cameraLinePoint) || isZeroPoint)
                            {
                                ++processed.skippedWorkpieceCloudPointCount;
                                continue;
                            }

                            ProcessedScanWorkpiecePoint cloudPoint;
                            cloudPoint.workpiecePoint =
                                RobotCalculation::CalcLaserPointInRobot(processed.robotPose, cameraLinePoint, calibration);
                            cloudPoint.cameraPoint = cameraLinePoint;
                            processed.workpiecePoints.push_back(cloudPoint);
                        }

                        if (ShouldSkipLaserCalc(processed.sample))
                        {
                            processed.status = "skip_invalid_camera_point";
                        }
                        else
                        {
                            processed.status = "laser_ok";
                            processed.laserPoint =
                                RobotCalculation::CalcLaserPointInRobot(processed.robotPose, processed.sample.point, calibration);
                            processed.hasLaserPoint = true;
                        }
                    }

                    {
                        std::lock_guard<std::mutex> locker(processedCameraSamplesMutex);
                        processedCameraSamples.push_back(std::move(processed));
                    }
                }
            });
    }
    auto finishCameraProcessingWorkers = [&]()
        {
            cameraEnqueueFinished.store(true);
            pendingCameraFramesCv.notify_all();
            robotSamplesCv.notify_all();
            for (std::thread& worker : processingWorkers)
            {
                if (worker.joinable())
                {
                    worker.join();
                }
            }
        };

    if (appendLog)
    {
        appendLog(QString("开始扫描运动：相机帧由当前机器人专属缓存读取，相机读取帧率=%1 Hz（约 %2 ms/帧，来自相机参数 CameraReadFps，用于时间间隔统计），机器人位姿约 %3 ms 采样；扫描匹配时间轴=%4（%5），相机帧timestamp会在首帧处映射到该时间轴，并叠加相机时间补偿 %6 ms。点云转换使用 %7 个后台处理线程。配置扫描速度= %8 mm/min，下发速度= %9 %10")
            .arg(actualCameraReadFps, 0, 'f', 2)
            .arg(cameraReadIntervalMs)
            .arg(ROBOT_SAMPLE_INTERVAL_MS)
            .arg(scanTimestampSourceName)
            .arg(scanTimestampFieldName)
            .arg(param.dCameraTimeOffsetMs, 0, 'f', 3)
            .arg(processingWorkerCount)
            .arg(param.dScanSpeed, 0, 'f', 3)
            .arg(scanCommandSpeed, 0, 'f', 3)
            .arg(scanCommandSpeedUnit));
    }

    // 超长扫描必须在第一条运动前拒绝；30分钟是异常运行边界，不能用来截断
    // 已知会合法运行更久的扫描。
    const double scanDistanceMm = std::hypot(
        param.tEndPos.dX - param.tStartPos.dX,
        param.tEndPos.dY - param.tStartPos.dY,
        param.tEndPos.dZ - param.tStartPos.dZ);
    const double scanSpeedMmPerSec = pFanucDriver != nullptr
        ? scanCommandSpeed
        : (param.dScanSpeed > 1e-6 ? param.dScanSpeed / 60.0 : 0.0);
    const double estimatedScanMs = scanSpeedMmPerSec > 1e-6
        ? (scanDistanceMm / scanSpeedMmPerSec) * 1000.0
        : std::numeric_limits<double>::infinity();
    const double requiredScanTimeoutMs = estimatedScanMs * 2.0 + 30000.0;
    if (!std::isfinite(requiredScanTimeoutMs)
        || requiredScanTimeoutMs > RobotMotionTimeoutPolicy::kMotionTimeoutMs)
    {
        const QString failure = QString(
            "扫描运动已拒绝：距离≈%1 mm、预计≈%2 min，含安全裕量后超过30分钟上限；请提高扫描速度或拆分扫描。")
            .arg(scanDistanceMm, 0, 'f', 1)
            .arg(estimatedScanMs / 60000.0, 0, 'f', 1);
        pRobotDriver->SetLastRobotError(failure.toUtf8().toStdString());
        if (appendLog)
        {
            appendLog(failure);
        }
        finishCameraProcessingWorkers();
        frameCache->Clear();
        return false;
    }
    const int scanFinishTimeoutMs = static_cast<int>(std::clamp(
        requiredScanTimeoutMs,
        120000.0,
        static_cast<double>(RobotMotionTimeoutPolicy::kMotionTimeoutMs)));
    if (appendLog)
    {
        appendLog(QString("扫描距离≈%1 mm，预计扫描≈%2 s，完成超时=%3 s。")
            .arg(scanDistanceMm, 0, 'f', 1)
            .arg(estimatedScanMs / 1000.0, 0, 'f', 1)
            .arg(scanFinishTimeoutMs / 1000.0, 0, 'f', 0));
    }

    const bool moveOk = pRobotDriver->MoveByJob(
        param.tEndPos,
        T_ROBOT_MOVE_SPEED(scanCommandSpeed, 0.0, 0.0),
        pRobotDriver->m_nExternalAxleType,
        "MOVL");
    if (!moveOk)
    {
        const QString failure = QStringLiteral("扫描终点运动启动失败。");
        if (!RobotOperationLease::IsCancellationRequested(pRobotDriver))
        {
            StopUnverifiedMotionAfterFailure(pRobotDriver, failure, appendLog);
        }
        else if (appendLog)
        {
            appendLog(failure);
        }
        finishCameraProcessingWorkers();
        return false;
    }
    if (progress != nullptr)
    {
        progress->commandAccepted = true;
    }

    QString trackedScanProgram;
    if (pStepDriver != nullptr && scanPauseAvailability)
    {
        std::string trackedProject;
        std::string trackedProgram;
        bool trackedMotionAlreadyStopped = false;
        if (!pStepDriver->GetTrackedMotionIdentity(
            trackedProject, trackedProgram, &trackedMotionAlreadyStopped))
        {
            const QString failure = QString("扫描运动已启动，但无法冻结受跟踪的 STEP 程序身份：%1")
                .arg(DecodeRobotMessageText(pStepDriver->GetLastRobotError()));
            StopUnverifiedMotionAfterFailure(pRobotDriver, failure, appendLog);
            finishCameraProcessingWorkers();
            frameCache->Clear();
            return false;
        }
        trackedScanProgram = QString::fromStdString(trackedProgram);
        if (!trackedMotionAlreadyStopped)
        {
            publishScanPauseAvailability(true, trackedScanProgram);
        }
        if (appendLog)
        {
            appendLog(trackedMotionAlreadyStopped
                ? QString("STEP 扫描程序在暂停入口开放前已自然停止；保留身份用于终态见证，不开放暂停按钮：Project=%1 Program=%2")
                    .arg(QString::fromStdString(trackedProject), trackedScanProgram)
                : QString("扫描暂停控制已绑定受跟踪程序：Project=%1 Program=%2")
                    .arg(QString::fromStdString(trackedProject), trackedScanProgram));
        }
    }

    if (!appendRobotPose())
    {
        const QString failure = QStringLiteral(
            "扫描运动已启动，但首个严格机器人位姿读取失败；已中止运动并丢弃本轮扫描。");
        StopUnverifiedMotionAfterFailure(pRobotDriver, failure, appendLog);
        finishCameraProcessingWorkers();
        frameCache->Clear();
        return false;
    }
    reportScanProgress();
    int motionState = 0;
    // STEP ContiMoveAnyWithProgramName 返回成功前已经回读过 eRun；
    // 首次扫描轮询可能已经直接看到 eStop，不能因此误判为“未启动”。
    bool motionStarted = pFanucDriver == nullptr;
    if (motionStarted)
    {
        if (progress != nullptr)
        {
            progress->motionStarted = true;
        }
        scanStartCameraSequence = frameCache->Mark();
        lastPulledCameraSequence = scanStartCameraSequence;
    }
    const qint64 motionStartMs = SteadyNowMs();
    qint64 lastRobotPollMs = motionStartMs - ROBOT_SAMPLE_INTERVAL_MS;
    qint64 lastBudgetTickMs = motionStartMs;
    qint64 activeRunElapsedMs = 0;
    qint64 pauseStartedMs = -1;
    bool scanPaused = false;
    while (true)
    {
        if (RobotOperationLease::IsCancellationRequested(pRobotDriver))
        {
            if (appendLog)
            {
                appendLog(QStringLiteral("扫描运动收到安全停止：立即结束采集并丢弃本轮缓存。"));
            }
            finishCameraProcessingWorkers();
            frameCache->Clear();
            return false;
        }
        const qint64 nowMs = SteadyNowMs();

        if ((nowMs - lastRobotPollMs) >= ROBOT_SAMPLE_INTERVAL_MS)
        {
            lastRobotPollMs = nowMs;
            motionState = pFanucDriver != nullptr
                ? pFanucDriver->GetIntVar(FANUC_MOTION_STATE_REG)
                : pRobotDriver->CheckDone();
            const bool isRunningState = pFanucDriver != nullptr
                ? (motionState == 10 || motionState == 20 || motionState == 1)
                : (motionState == STEPROBOTSDK::eRun);
            const bool isPausedState = pStepDriver != nullptr
                && motionState == STEPROBOTSDK::ePause;
            const bool isDoneState = pFanucDriver != nullptr
                ? (motionState == 1)
                : (motionState == STEPROBOTSDK::eStop);
            const bool isInvalidState = pFanucDriver != nullptr
                ? (motionState != 0 && motionState != 1 && motionState != 10 && motionState != 20)
                : (motionState != STEPROBOTSDK::eRun
                    && motionState != STEPROBOTSDK::ePause
                    && motionState != STEPROBOTSDK::eStop);
            const qint64 budgetDeltaMs = (std::max)(qint64(0), nowMs - lastBudgetTickMs);
            if (motionStarted && !scanPaused)
            {
                activeRunElapsedMs += budgetDeltaMs;
            }
            lastBudgetTickMs = nowMs;
            if (isInvalidState)
            {
                const QString failure = pFanucDriver != nullptr
                    ? QString("扫描运动状态读取失败：R[%1]=%2。")
                        .arg(FANUC_MOTION_STATE_REG)
                        .arg(motionState)
                    : QString("扫描运动状态异常：CheckDone=%1（不是 eRun/ePause/eStop）。")
                        .arg(motionState);
                StopUnverifiedMotionAfterFailure(pRobotDriver, failure, appendLog);
                finishCameraProcessingWorkers();
                return false;
            }
            if (isPausedState)
            {
                // 暂停期间持续推进丢弃水位。恢复后从最后一次暂停水位继续拉取，
                // 既排除静止积压，又保留 UI 发送 START 之后产生的有效运动帧。
                lastPulledCameraSequence = frameCache->Mark();
                if (!scanPaused)
                {
                    scanPaused = true;
                    pauseStartedMs = nowMs;
                    if (imageCaptureEnabled)
                    {
                        frameCache->SetImageCaptureDir(QString());
                    }
                    if (setFlowStep)
                    {
                        setFlowStep(QStringLiteral("扫描已暂停；采样与运动超时计时均已暂停"));
                    }
                    if (appendLog)
                    {
                        appendLog(QStringLiteral(
                            "STEP 扫描已稳定进入暂停态：停止写入机器人位姿和相机帧，保留当前受跟踪程序等待继续。"));
                    }
                }
                if (pauseStartedMs >= 0
                    && (nowMs - pauseStartedMs) > RobotMotionTimeoutPolicy::kMotionTimeoutMs)
                {
                    const QString failure = QStringLiteral(
                        "STEP 扫描暂停超过 30 分钟安全上限；已中止当前程序并丢弃本轮扫描。");
                    publishScanPauseAvailability(false);
                    StopUnverifiedMotionAfterFailure(pRobotDriver, failure, appendLog);
                    finishCameraProcessingWorkers();
                    frameCache->Clear();
                    return false;
                }
            }
            else if (scanPaused)
            {
                scanPaused = false;
                pauseStartedMs = -1;
                if (imageCaptureEnabled && !imageCaptureTmpDir.isEmpty())
                {
                    frameCache->SetImageCaptureDir(imageCaptureTmpDir, imageCaptureStride);
                }
                if (setFlowStep)
                {
                    setFlowStep(QStringLiteral("扫描已继续，恢复采集相机点、机器人位置和激光点"));
                }
                if (appendLog)
                {
                    appendLog(QStringLiteral(
                        "STEP 扫描已继续：暂停期间积压帧已持续丢弃，START 后有效帧和位姿采集已恢复。"));
                }
            }
            if (isRunningState)
            {
                if (!motionStarted)
                {
                    if (progress != nullptr)
                    {
                        progress->motionStarted = true;
                    }
                    scanStartCameraSequence = frameCache->Mark();
					lastPulledCameraSequence = scanStartCameraSequence;
					if (appendLog)
					{
						if (pFanucDriver != nullptr)
						{
							appendLog(QString("扫描运动状态寄存器进入运行态：R[%1]=%2").arg(FANUC_MOTION_STATE_REG).arg(motionState));
						}
						else
						{
							appendLog(QString("扫描运动进入运行态：CheckDone=%1").arg(motionState));
						}
					}
                }
                motionStarted = true;
            }
            if (motionStarted && !isPausedState)
            {
                if (!appendRobotPose())
                {
                    const QString failure = QStringLiteral(
                        "扫描运动期间严格机器人位姿读取失败；已中止运动并丢弃本轮扫描，禁止生成焊道。");
                    StopUnverifiedMotionAfterFailure(pRobotDriver, failure, appendLog);
                    finishCameraProcessingWorkers();
                    frameCache->Clear();
                    return false;
                }
                reportScanProgress();
                if (!hasCameraTimeBaseRobotTimestamp)
                {
                    const qint64 latestRobotUs = latestRobotTimestampUs();
                    if (latestRobotUs > 0)
                    {
                        cameraTimeBaseRobotTimestampUs = latestRobotUs;
                        hasCameraTimeBaseRobotTimestamp = true;
                    }
                }
                pullScanCameraFrames();
            }
            if (motionStarted && isDoneState)
            {
                publishScanPauseAvailability(false);
                // STEP 的 ePause 是可恢复态，绝不是完成；eStop 也要再由
                // CheckRobotDone 校验无控制器错误。FANUC R[93]=1 同样只是里程碑，
                // 还需 CHECK_DONE 稳定确认任务退出。
                const int terminalState = pRobotDriver->CheckRobotDone(100, 30000);
                if (appendLog)
                {
                    appendLog(QString("扫描运动终态确认：CheckRobotDone=%1").arg(terminalState));
                }
                if (terminalState <= 0)
                {
                    // CheckRobotDone 自身已对非取消异常执行可验证中止，不重复 stop。
                    finishCameraProcessingWorkers();
                    return false;
                }
                RobotOperationLease::MarkMotionCompleted(pRobotDriver);
                if (progress != nullptr)
                {
                    progress->motionCompleted = true;
                }
                reportScanProgress(true);
                scanEndCameraSequence = frameCache->Mark();
                pullScanCameraFramesTo(scanEndCameraSequence);
                break;
            }

            const qint64 startupElapsedMs = SteadyNowMs() - motionStartMs;
			if (!motionStarted && startupElapsedMs > 3000)
			{
				const QString failure = pFanucDriver != nullptr
					? QString("扫描运动未在 3s 内进入运行态：R[%1]=%2。")
						.arg(FANUC_MOTION_STATE_REG)
						.arg(motionState)
					: QString("扫描运动未在 3s 内进入运行态：CheckDone=%1。")
						.arg(motionState);
				StopUnverifiedMotionAfterFailure(pRobotDriver, failure, appendLog);
				finishCameraProcessingWorkers();
				return false;
            }
			if (motionStarted && activeRunElapsedMs > scanFinishTimeoutMs)
			{
				const QString failure = pFanucDriver != nullptr
					? QString("扫描运动等待完成超时（%1 s）：R[%2]=%3。")
						.arg(scanFinishTimeoutMs / 1000.0, 0, 'f', 0)
						.arg(FANUC_MOTION_STATE_REG)
						.arg(motionState)
					: QString("扫描运动等待完成超时（%1 s）：CheckDone=%2。")
						.arg(scanFinishTimeoutMs / 1000.0, 0, 'f', 0)
						.arg(motionState);
				StopUnverifiedMotionAfterFailure(pRobotDriver, failure, appendLog);
                finishCameraProcessingWorkers();
                return false;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    if (!appendRobotPose())
    {
        if (appendLog)
        {
            appendLog(QStringLiteral("扫描终态位姿读取失败，已丢弃本轮扫描，禁止生成焊道。"));
        }
        finishCameraProcessingWorkers();
        frameCache->Clear();
        return false;
    }
    const qint64 scanMotionElapsedMs = activeRunElapsedMs;

    if (scanEndCameraSequence == 0)
    {
        scanEndCameraSequence = frameCache->Mark();
        pullScanCameraFramesTo(scanEndCameraSequence);
    }
    frameCache->Clear();

    bool tailWaitTriggered = false;
    const qint64 tailWaitStartMs = SteadyNowMs();
    while (latestEnqueuedCameraTimestampUs > 0
        && latestRobotTimestampUs() > 0
        && latestEnqueuedCameraTimestampUs > latestRobotTimestampUs()
        && (SteadyNowMs() - tailWaitStartMs) < CAMERA_ROBOT_MATCH_TAIL_WAIT_MS)
    {
        if (!tailWaitTriggered && appendLog)
        {
            appendLog(QString("检测到相机时间戳晚于最新机器人位姿，开始等待机器人监控时间追上（最长 %1 ms）。")
                .arg(CAMERA_ROBOT_MATCH_TAIL_WAIT_MS));
        }
        tailWaitTriggered = true;

        if (!appendRobotPose())
        {
            if (appendLog)
            {
                appendLog(QStringLiteral("扫描尾部位姿读取失败，已丢弃本轮扫描，禁止生成焊道。"));
            }
            finishCameraProcessingWorkers();
            frameCache->Clear();
            return false;
        }
        if (latestEnqueuedCameraTimestampUs <= latestRobotTimestampUs())
        {
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(CAMERA_ROBOT_MATCH_TAIL_POLL_MS));
    }

    const qint64 tailWaitElapsedMs = tailWaitTriggered ? (SteadyNowMs() - tailWaitStartMs) : 0;

    // 扫描运动结束即关闭图像采集窗口（点云处理期无需继续拍摄）。
    frameCache->SetImageCaptureDir(QString());

    // 扫描运动+尾部匹配结束这一刻，对相机 SDK 逐帧取帧状态做快照：此后进入点云处理，相机仍在空转，
    // 必须在此处取，避免处理期的空轮询污染“无新帧间断”统计。除 OK(0) 外的状态稍后逐条写入匹配明细文件。
    const std::vector<CameraFrameCache::PollStatus> pollStatusLog = frameCache->PollStatusSnapshot();
    qint64 maxNoFrameGapUs = 0;       // 扫描期间相邻两帧之间最长的“无新帧”间断
    int okPollCount = 0;              // 实际取到新帧的轮询次数
    int sdkNoDataPollCount = 0;       // -106 无新帧
    int sdkDuplicatePollCount = 0;    // -107 重复帧
    int sdkErrorPollCount = 0;        // 其它取帧错误
    {
        qint64 lastFrameReceiveUs = -1;
        for (const CameraFrameCache::PollStatus& poll : pollStatusLog)
        {
            if (poll.ret == 0)
            {
                if (lastFrameReceiveUs >= 0)
                {
                    maxNoFrameGapUs = std::max(maxNoFrameGapUs, poll.receiveTimestampUs - lastFrameReceiveUs);
                }
                lastFrameReceiveUs = poll.receiveTimestampUs;
                ++okPollCount;
            }
            else if (poll.ret == -106)
            {
                ++sdkNoDataPollCount;
            }
            else if (poll.ret == -107)
            {
                ++sdkDuplicatePollCount;
            }
            else
            {
                ++sdkErrorPollCount;
            }
        }
        // 末帧到扫描结束（快照时刻）的尾部无帧时长：捕获“扫描中途相机/SDK 卡死后再未恢复”。
        if (lastFrameReceiveUs >= 0 && !pollStatusLog.empty())
        {
            maxNoFrameGapUs = std::max(maxNoFrameGapUs, pollStatusLog.back().receiveTimestampUs - lastFrameReceiveUs);
        }
    }

    const qint64 processingJoinStartMs = SteadyNowMs();
    finishCameraProcessingWorkers();
    const qint64 postMotionProcessingWaitMs = SteadyNowMs() - processingJoinStartMs;
    const qint64 parallelProcessingElapsedMs = SteadyNowMs() - processingWallStartMs;
    if (RobotOperationLease::IsCancellationRequested(pRobotDriver))
    {
        frameCache->Clear();
        savedPath.clear();
        if (appendLog)
        {
            appendLog(QStringLiteral("扫描后处理开始前检测到安全停止，已丢弃本轮结果。"));
        }
        return false;
    }

    // ===== 统计时间对齐（方案A）=====
    // 首帧单点对齐的抖动实测可达 ±60ms（机器人位姿50ms采样量化 + 运行态检测抖动 + 单帧传输延迟抖动，
    // 实测单帧延迟全幅 38.5ms），50mm/s 扫描速度下即 ±3mm 的扫描方向整体平移——同一工件重复扫不重合的根因。
    // 这里改用整场统计量重估对齐（实测相邻扫描重复性 <2ms）：
    //   ①相机钟→PC钟：全部成功取帧 (SDK帧时间戳, PC接收时刻) 差值的 P10 低分位（低分位≈最小传输延迟，抗抖动）；
    //   ②PC钟→机器人轴：全部位姿采样 (所选轴时间戳, PC接收时刻) 差值的中位数（轴=PC接收时间时恒0）。
    // 重估后对全部已处理帧重新插值匹配+手眼变换（纯内存重算，百ms级）。样本不足时保留首帧对齐并写日志。
    qint64 statCamToPcUs = 0;
    qint64 statPcToAxisUs = 0;
    qint64 statRealignDeltaUs = 0;
    qint64 statFirstFrameOffsetUs = 0;   // 覆盖前的首帧粗对齐偏移，供日志区分统计值/首帧值
    std::size_t statCamSampleCount = 0;
    std::size_t statAxisSampleCount = 0;
    int statFlippedTailCount = 0;        // 重匹配后由已匹配翻为 unmatched_after 的尾帧数
    bool statRealignApplied = false;
    if (hasCameraToRobotTimeOffset && param.bUseStatTimeAlign)
    {
        std::vector<qint64> camPcDeltasUs;
        camPcDeltasUs.reserve(pollStatusLog.size());
        for (const CameraFrameCache::PollStatus& poll : pollStatusLog)
        {
            if (poll.ret == 0 && poll.frameTimestampUs > 0)
            {
                camPcDeltasUs.push_back(poll.receiveTimestampUs - poll.frameTimestampUs);
            }
        }
        std::vector<qint64> axisPcDeltasUs;
        {
            std::lock_guard<std::mutex> locker(robotSamplesMutex);
            axisPcDeltasUs.reserve(robotAxisPcPairsUs.size());
            for (const std::pair<qint64, qint64>& pair : robotAxisPcPairsUs)
            {
                axisPcDeltasUs.push_back(pair.first - pair.second);
            }
        }
        statCamSampleCount = camPcDeltasUs.size();
        statAxisSampleCount = axisPcDeltasUs.size();
        if (statCamSampleCount >= 50 && statAxisSampleCount >= 20)
        {
            std::sort(camPcDeltasUs.begin(), camPcDeltasUs.end());
            std::sort(axisPcDeltasUs.begin(), axisPcDeltasUs.end());
            statCamToPcUs = camPcDeltasUs[camPcDeltasUs.size() / 10];
            statPcToAxisUs = axisPcDeltasUs[axisPcDeltasUs.size() / 2];
            const qint64 statOffsetUs = statCamToPcUs + statPcToAxisUs;
            statFirstFrameOffsetUs = cameraToRobotTimeOffsetUs;
            statRealignDeltaUs = statOffsetUs - cameraToRobotTimeOffsetUs;
            cameraToRobotTimeOffsetUs = statOffsetUs;

            // 正向修正会把末帧时间戳右移越过位姿末样本。此刻机器人仍静止在扫描终点（收枪安全位
            // 在 ScanMoveAndCollect 返回后才执行），补采几个位姿样本延长时间轴覆盖，避免尾帧被改判丢弃。
            if (statRealignDeltaUs > 0)
            {
                qint64 lastFrameNewTimestampUs = 0;
                {
                    std::lock_guard<std::mutex> locker(processedCameraSamplesMutex);
                    for (const ProcessedScanCameraSample& processed : processedCameraSamples)
                    {
                        if (processed.sample.rawTimestampUs > 0)
                        {
                            lastFrameNewTimestampUs = std::max(
                                lastFrameNewTimestampUs,
                                processed.sample.rawTimestampUs + statOffsetUs + cameraTimeOffsetUs);
                        }
                    }
                }
                for (int extraPoll = 0; extraPoll < 20; ++extraPoll)
                {
                    if (latestRobotTimestampUs() >= lastFrameNewTimestampUs)
                    {
                        break;
                    }
                    if (!appendRobotPose())
                    {
                        if (appendLog)
                        {
                            appendLog(QStringLiteral(
                                "扫描统计校时补采位姿失败，已丢弃本轮扫描，禁止生成焊道。"));
                        }
                        return false;
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(CAMERA_ROBOT_MATCH_TAIL_POLL_MS));
                }
            }

            std::lock_guard<std::mutex> locker(processedCameraSamplesMutex);
            for (ProcessedScanCameraSample& processed : processedCameraSamples)
            {
                TimestampedCameraPoint& sample = processed.sample;
                if (sample.rawTimestampUs <= 0)
                {
                    continue;
                }
                const qint64 newTimestampUs = sample.rawTimestampUs + statOffsetUs + cameraTimeOffsetUs;
                sample.timestampUs = newTimestampUs;
                const bool outOfRange = robotSamples.empty()
                    || newTimestampUs < robotSamples.front().timestampUs
                    || newTimestampUs > robotSamples.back().timestampUs;
                if (outOfRange)
                {
                    const bool wasMatched = processed.hasRobotPose;
                    processed.status = robotSamples.empty()
                        ? QStringLiteral("unmatched_no_robot_sample")
                        : (newTimestampUs < robotSamples.front().timestampUs
                            ? QStringLiteral("unmatched_before_robot")
                            : QStringLiteral("unmatched_after_robot"));
                    if (wasMatched && processed.status == QStringLiteral("unmatched_after_robot"))
                    {
                        ++statFlippedTailCount;
                    }
                    processed.hasRobotPose = false;
                    processed.hasLaserPoint = false;
                    processed.contributedWorkpieceFrame = false;
                    processed.robotWindow = RobotInterpolationWindow();
                    processed.workpiecePoints.clear();
                    processed.skippedWorkpieceCloudPointCount = 0;  // 整帧已剔除，跳点计数一并清零，避免污染点云统计口径
                    continue;
                }
                const RobotInterpolationWindow window = FindRobotInterpolationWindow(robotSamples, newTimestampUs);
                if (window.prevIndex <= 0)
                {
                    continue;
                }
                const int prevZeroIndex = std::clamp(window.prevIndex - 1, 0, static_cast<int>(robotSamples.size()) - 1);
                const int nextZeroIndex = std::clamp(window.nextIndex - 1, 0, static_cast<int>(robotSamples.size()) - 1);
                std::vector<RobotCalculation::TimestampedRobotPose> interpolationSamples;
                interpolationSamples.push_back(robotSamples[static_cast<std::size_t>(prevZeroIndex)]);
                if (nextZeroIndex != prevZeroIndex)
                {
                    interpolationSamples.push_back(robotSamples[static_cast<std::size_t>(nextZeroIndex)]);
                }
                processed.robotWindow = window;
                processed.robotPose = RobotCalculation::InterpolateRobotPose(interpolationSamples, newTimestampUs);
                processed.hasRobotPose = true;
                // 线点云重变换：cameraPoint 为相机系原始点，直接按新位姿重算工件系坐标。
                // 原先未匹配的帧线点数据未保留（workpiecePoints 为空），无法复活，仅影响边界1~2帧的点云密度。
                for (ProcessedScanWorkpiecePoint& cloudPoint : processed.workpiecePoints)
                {
                    cloudPoint.workpiecePoint =
                        RobotCalculation::CalcLaserPointInRobot(processed.robotPose, cloudPoint.cameraPoint, calibration);
                }
                if (ShouldSkipLaserCalc(sample))
                {
                    processed.status = QStringLiteral("skip_invalid_camera_point");
                    processed.hasLaserPoint = false;
                }
                else
                {
                    processed.status = QStringLiteral("laser_ok");
                    processed.laserPoint =
                        RobotCalculation::CalcLaserPointInRobot(processed.robotPose, sample.point, calibration);
                    processed.hasLaserPoint = true;
                }
            }
            statRealignApplied = true;
        }
    }
    if (appendLog)
    {
        if (statRealignApplied)
        {
            appendLog(QString("统计时间对齐已生效：相机→PC钟差(P10)=%1 us（帧样本=%2），PC→机器人轴钟差(中位)=%3 us（位姿样本=%4），相对首帧粗对齐修正 %5 ms，已按新偏移全量重匹配%6。")
                .arg(statCamToPcUs)
                .arg(static_cast<int>(statCamSampleCount))
                .arg(statPcToAxisUs)
                .arg(static_cast<int>(statAxisSampleCount))
                .arg(statRealignDeltaUs / 1000.0, 0, 'f', 2)
                .arg(statFlippedTailCount > 0
                    ? QString("（%1 个尾帧因时间戳平移超出机器人采样范围被改判丢弃）").arg(statFlippedTailCount)
                    : QString()));
        }
        else if (hasCameraToRobotTimeOffset && !param.bUseStatTimeAlign)
        {
            appendLog("统计时间对齐已按参数关闭（UseStatTimeAlign=0），本次使用首帧对齐（旧算法，对照测试模式）。");
        }
        else if (hasCameraToRobotTimeOffset)
        {
            appendLog(QString("统计时间对齐未启用（帧样本=%1<50 或 位姿样本=%2<20），保留首帧粗对齐——扫描方向重复性受限（±3mm级）。")
                .arg(static_cast<int>(statCamSampleCount))
                .arg(static_cast<int>(statAxisSampleCount)));
        }
    }

    {
        std::lock_guard<std::mutex> locker(processedCameraSamplesMutex);
        std::sort(processedCameraSamples.begin(), processedCameraSamples.end(),
            [](const ProcessedScanCameraSample& left, const ProcessedScanCameraSample& right)
            {
                return left.sample.sampleIndex < right.sample.sampleIndex;
            });
        cameraSamples.reserve(processedCameraSamples.size());
        matchedCameraSamples.reserve(processedCameraSamples.size());
        for (const ProcessedScanCameraSample& processed : processedCameraSamples)
        {
            cameraSamples.push_back(processed.sample);
            if (processed.hasRobotPose)
            {
                matchedCameraSamples.push_back(processed.sample);
            }
        }
    }

    int droppedHeadCameraCount = 0;
    int droppedTailCameraCount = 0;
    for (const ProcessedScanCameraSample& processed : processedCameraSamples)
    {
        if (processed.status == "unmatched_before_robot")
        {
            ++droppedHeadCameraCount;
        }
        else if (processed.status == "unmatched_after_robot")
        {
            ++droppedTailCameraCount;
        }
    }

    const QString resultDir = BuildResultDir(param.sRobotName);
    const QString cameraDir = QDir(resultDir).filePath("CameraPoint");
    const QString robotDir = QDir(resultDir).filePath("RobotPoint");
    const QString laserDir = QDir(resultDir).filePath("LaserPoint");
    QDir().mkpath(cameraDir);
    QDir().mkpath(robotDir);
    QDir().mkpath(laserDir);
    const QString qualityGatePath = QDir(laserDir).filePath(
        QString::fromLatin1(POINT_CLOUD_QUALITY_GATE_FILE_NAME));
    QString qualityGateError;
    PointCloudProofReplacementSession qualityGateReplacement;
    if (!PointCloudProofIntegrity::BeginProofReplacement(
            qualityGatePath,
            QStringLiteral("liveScan 后处理尚未完整完成，拒绝任何旧/中间点云授权。"),
            qualityGateError))
    {
        if (appendLog)
        {
            appendLog(QStringLiteral("扫描失败：无法建立点云证明持久拒绝闭锁：")
                + qualityGateError);
        }
        if (setFlowStep)
        {
            setFlowStep("扫描失败：无法建立质量证明拒绝闭锁");
        }
        return false;
    }
    qualityGateReplacement.Arm(qualityGatePath);
    if (!InvalidatePointCloudQualityGate(laserDir, qualityGateError))
    {
        if (appendLog)
        {
            appendLog(qualityGateError);
        }
        if (setFlowStep)
        {
            setFlowStep("扫描失败：旧质量证明删除失败，拒绝闭锁保持有效");
        }
        return false;
    }

    const QString cameraPath = QDir(cameraDir).filePath("PreciseCameraPoint.txt");
    const QString robotPath = QDir(robotDir).filePath("PreciseRobotPoint.txt");
    const QString laserPath = QDir(laserDir).filePath(RAW_LASER_FILE_NAME);
    const QString workpieceCloudPath = QDir(laserDir).filePath(WORKPIECE_CLOUD_FILE_NAME);
    const QString matchDebugPath = QDir(laserDir).filePath(MATCH_DEBUG_FILE_NAME);
    const QString sdkPointCloudDir = QDir(laserDir).filePath(SDK_POINT_CLOUD_OUTPUT_DIR_NAME);
    const QString sdkSeamExtractedPath = QDir(sdkPointCloudDir).filePath(SDK_SEAM_EXTRACTED_FILE_NAME);
    const QString sdkSeamExtracted2mmPath = QDir(sdkPointCloudDir).filePath(SDK_SEAM_EXTRACTED_2MM_FILE_NAME);
    const QString sdkBaseWeldPath = QDir(sdkPointCloudDir).filePath(SDK_BASE_WELD_FILE_NAME);
    const QString preservePathFitPath = QDir(laserDir).filePath(PRESERVE_PATH_FILE_NAME);
    const QString keyPointsPath = QDir(laserDir).filePath(KEY_POINTS_FILE_NAME);
    const QString classifiedPath = QDir(laserDir).filePath(CLASSIFIED_FILE_NAME);
    const QString cornerCompKeyPointsPath = QDir(laserDir).filePath(CORNER_COMP_KEY_POINTS_FILE_NAME);
    const QString cornerCompClassifiedPath = QDir(laserDir).filePath(CORNER_COMP_CLASSIFIED_FILE_NAME);
    const QString classifiedNoisePath = QDir(laserDir).filePath(CLASSIFIED_NOISE_FILE_NAME);
    const QString weldPosePath = QDir(laserDir).filePath(WELD_POSE_FILE_NAME);
    const QString weldPoseSeamCompPath = QDir(laserDir).filePath(WELD_POSE_SEAM_COMP_FILE_NAME);
    savedPath = resultDir;

    const qint64 outputBuildStartMs = SteadyNowMs();
    std::vector<QString> cameraLines;
    std::vector<QString> robotLines;
    std::vector<QString> laserLines;
    std::vector<QString> workpieceCloudLines;
    std::vector<QString> matchDebugLines;
    cameraLines.reserve(cameraSamples.size() + 1);
    robotLines.reserve(matchedCameraSamples.size() + 1);
    laserLines.reserve(matchedCameraSamples.size() + 1);
    workpieceCloudLines.reserve(cameraSamples.size() * 16 + 1);
    matchDebugLines.reserve(cameraSamples.size() + 1);
    cameraLines.push_back("index,x,y,z,error");
    robotLines.push_back("index,x,y,z,rx,ry,rz,bx,by,bz");
    laserLines.push_back("index,x,y,z");
    workpieceCloudLines.push_back("index x y z");
    matchDebugLines.push_back("index,status,camera_raw_timestamp_us,camera_raw_delta_us,mapped_robot_timestamp_us,prev_robot_index,prev_robot_timestamp_us,next_robot_index,next_robot_timestamp_us,interp_ratio,camera_x,camera_y,camera_z,robot_x,robot_y,robot_z,robot_rx,robot_ry,robot_rz,robot_bx,robot_by,robot_bz,laser_x,laser_y,laser_z,error,sdk_status,sdk_recv_timestamp_us");

    // 完整点云逐帧调试导出（独立开关，默认关闭——仅排查相机散点时勾选；与目标点/特征点流程无关）：
    // 每点附 frame_index + 相机原始坐标 + 机器人位姿 + 时间戳，CloudCompare 按 frame_index 着色定位散点帧。
    // 文件达 647MB 级，每次扫描生成会明显拖慢流程，故默认不生成。
    const bool exportWorkpieceFrameDebug = PointCloudProcessingConfig::Load().exportWorkpieceFrameDebug;
    std::vector<QString> workpieceFrameDebugLines;
    if (exportWorkpieceFrameDebug)
    {
        workpieceFrameDebugLines.reserve(cameraSamples.size() * 16 + 1);
        workpieceFrameDebugLines.push_back("X Y Z frame_index camera_x camera_y camera_z robot_x robot_y robot_z robot_rx robot_ry robot_rz camera_raw_ts_us mapped_ts_us");
    }

    const qint64 cameraLineBuildStartMs = SteadyNowMs();
    for (const TimestampedCameraPoint& sample : cameraSamples)
    {
        cameraLines.push_back(RobotCalculation::Vector3IndexedCsv(sample.sampleIndex, sample.point, sample.error));
    }
    const qint64 cameraLineBuildElapsedMs = SteadyNowMs() - cameraLineBuildStartMs;

    int skippedLaserCount = 0;
    int unmatchedBeforeRobotCount = 0;
    int unmatchedAfterRobotCount = 0;
    int unmatchedUnknownCount = 0;
    int laserIndexGapCount = 0;
    int maxLaserIndexGap = 0;
    int lastLaserIndex = -1;
    int workpieceCloudFrameCount = 0;
    int workpieceCloudPointCount = 0;
    int skippedWorkpieceCloudPointCount = 0;
    int workpieceCloudPointIndex = 1;
    const qint64 pointComputeStartMs = SteadyNowMs();
    for (const ProcessedScanCameraSample& processed : processedCameraSamples)
    {
        const TimestampedCameraPoint& sample = processed.sample;
        const int index = sample.sampleIndex;
        const QString& status = processed.status;
        if (status == "unmatched_before_robot")
        {
            ++unmatchedBeforeRobotCount;
        }
        else if (status == "unmatched_after_robot")
        {
            ++unmatchedAfterRobotCount;
        }
        else if (status == "skip_invalid_camera_point")
        {
            ++skippedLaserCount;
        }
        else if (status != "laser_ok")
        {
            ++unmatchedUnknownCount;
        }

        if (processed.contributedWorkpieceFrame)
        {
            ++workpieceCloudFrameCount;
        }
        skippedWorkpieceCloudPointCount += processed.skippedWorkpieceCloudPointCount;
        if (processed.hasRobotPose)
        {
            for (const ProcessedScanWorkpiecePoint& cloudPoint : processed.workpiecePoints)
            {
                const int cloudIndex = workpieceCloudPointIndex++;
                QStringList cloudFields;
                cloudFields
                    << QString::number(cloudIndex)
                    << QString::number(cloudPoint.workpiecePoint.x(), 'f', 6)
                    << QString::number(cloudPoint.workpiecePoint.y(), 'f', 6)
                    << QString::number(cloudPoint.workpiecePoint.z(), 'f', 6);
                workpieceCloudLines.push_back(cloudFields.join(' '));

                if (exportWorkpieceFrameDebug)
                {
                    // 逐帧排查列：变换后 XYZ + 帧号 + 相机原始坐标 + 该帧机器人位姿 + 相机/映射时间戳。
                    // 据此可对每个翘起点判定成因：相机原始坐标正常但变换后偏=端部位姿/时间对齐；相机原始就偏=扫到焊缝外结构/反光。
                    // 用 snprintf 一次格式化（避免 15 次 QString::arg 链对每行重复扫描，430 万点下性能差异巨大）。
                    char frameDebugBuf[320];
                    std::snprintf(frameDebugBuf, sizeof(frameDebugBuf),
                        "%.6f %.6f %.6f %d %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %lld %lld",
                        cloudPoint.workpiecePoint.x(), cloudPoint.workpiecePoint.y(), cloudPoint.workpiecePoint.z(),
                        index,
                        cloudPoint.cameraPoint.x(), cloudPoint.cameraPoint.y(), cloudPoint.cameraPoint.z(),
                        processed.robotPose.dX, processed.robotPose.dY, processed.robotPose.dZ,
                        processed.robotPose.dRX, processed.robotPose.dRY, processed.robotPose.dRZ,
                        static_cast<long long>(processed.sample.rawTimestampUs),
                        static_cast<long long>(processed.sample.timestampUs));
                    workpieceFrameDebugLines.push_back(QString::fromLatin1(frameDebugBuf));
                }

                RobotCalculation::IndexedPoint3D cloudInputPoint;
                cloudInputPoint.index = cloudIndex;
                cloudInputPoint.point = cloudPoint.workpiecePoint;
                workpieceCloudInput.push_back(cloudInputPoint);
                ++workpieceCloudPointCount;
            }
        }

        if (processed.hasLaserPoint)
        {
            if (lastLaserIndex > 0 && index - lastLaserIndex > 1)
            {
                ++laserIndexGapCount;
                maxLaserIndexGap = std::max(maxLaserIndexGap, index - lastLaserIndex - 1);
            }
            lastLaserIndex = index;

            robotLines.push_back(RobotCalculation::RobotPoseIndexedCsv(index, processed.robotPose));
            laserLines.push_back(RobotCalculation::Vector3IndexedCsv(index, processed.laserPoint));

            RobotCalculation::IndexedPoint3D laserFitPoint;
            laserFitPoint.index = index;
            laserFitPoint.point = processed.laserPoint;
            laserFitInput.push_back(laserFitPoint);
        }

        QStringList fields;
        fields
            << QString::number(index)
            << status
            << QString::number(sample.rawTimestampUs)
            << QString::number(sample.rawDeltaUs)
            << QString::number(sample.timestampUs)
            << QString::number(processed.robotWindow.prevIndex)
            << QString::number(processed.robotWindow.prevTimestampUs)
            << QString::number(processed.robotWindow.nextIndex)
            << QString::number(processed.robotWindow.nextTimestampUs)
            << QString::number(processed.robotWindow.ratio, 'f', 6)
            << QString::number(sample.point.x(), 'f', 6)
            << QString::number(sample.point.y(), 'f', 6)
            << QString::number(sample.point.z(), 'f', 6);
        if (processed.hasRobotPose)
        {
            fields
                << QString::number(processed.robotPose.dX, 'f', 6)
                << QString::number(processed.robotPose.dY, 'f', 6)
                << QString::number(processed.robotPose.dZ, 'f', 6)
                << QString::number(processed.robotPose.dRX, 'f', 6)
                << QString::number(processed.robotPose.dRY, 'f', 6)
                << QString::number(processed.robotPose.dRZ, 'f', 6)
                << QString::number(processed.robotPose.dBX, 'f', 6)
                << QString::number(processed.robotPose.dBY, 'f', 6)
                << QString::number(processed.robotPose.dBZ, 'f', 6);
        }
        else
        {
            fields << "" << "" << "" << "" << "" << "" << "" << "" << "";
        }
        if (processed.hasLaserPoint)
        {
            fields
                << QString::number(processed.laserPoint.x(), 'f', 6)
                << QString::number(processed.laserPoint.y(), 'f', 6)
                << QString::number(processed.laserPoint.z(), 'f', 6);
        }
        else
        {
            fields << "" << "" << "";
        }
        fields << CsvEscape(sample.error);
        fields << "" << "";  // sdk_status / sdk_recv_timestamp_us：匹配行不涉及 SDK 取帧状态，留空
        matchDebugLines.push_back(fields.join(','));
    }
    const qint64 pointComputeElapsedMs = SteadyNowMs() - pointComputeStartMs;

    // 把扫描期间所有非 OK 的 SDK 取帧状态作为独立明细行追加进同一个匹配明细文件：状态名写入
    // 专属的 sdk_status 新列（不复用原 status 列），接收时刻写入 sdk_recv_timestamp_us 新列，
    // 其余原有数据列（含 status）一律留空——“除 0 以外只保存状态和空数据”。
    int sdkStatusRowIndex = 0;
    for (const CameraFrameCache::PollStatus& poll : pollStatusLog)
    {
        if (poll.ret == 0)
        {
            continue;
        }
        QString sdkStatusName;
        if (poll.ret == -106)
        {
            sdkStatusName = QStringLiteral("skj_no_data");
        }
        else if (poll.ret == -107)
        {
            sdkStatusName = QStringLiteral("skj_duplicate");
        }
        else
        {
            sdkStatusName = QStringLiteral("skj_error(%1)").arg(poll.ret);
        }
        QStringList sdkFields;
        sdkFields.reserve(28);
        sdkFields << QString("sdk_%1").arg(sdkStatusRowIndex++);  // index
        for (int emptyCol = 0; emptyCol < 25; ++emptyCol)         // status..error 共 25 个原有列全部留空（含 status 列）
        {
            sdkFields << QString();
        }
        sdkFields << sdkStatusName;                               // sdk_status（专属新列）
        sdkFields << QString::number(poll.receiveTimestampUs);    // sdk_recv_timestamp_us（专属新列）
        matchDebugLines.push_back(sdkFields.join(','));
    }

    // SDK 逐次取帧调用日志：pollStatusLog 里每次 GetLatestFrame 一行（返回码/帧时间戳/点数/PC轮询间隔）。
    // 用途——帧时间戳(camera_raw_delta)突然翻倍=丢了一帧时，对齐这里判丢帧来源：
    //   · 跳变窗口内一直是 skj_no_data/duplicate → SDK 当时没新帧可给（上游/相机/网络少给）；
    //   · pc_delta_us 也跟着跳大（我方定时器卡了没去轮询）→ 我方漏取；
    //   · 轮询正常(pc_delta≈10ms)却拿到跳变的新帧 → SDK 只给了隔帧。
    std::vector<QString> sdkPollLogLines;
    sdkPollLogLines.reserve(pollStatusLog.size() + 1);
    // recv_frame_seq=我方成功取帧累计编号；frame_channel=SDK帧号(厂商以GetChannel承载,逐帧+1递增)，
    // channel_delta=相邻帧号差(正常1，N=中间丢了N-1帧——比时间戳差更硬的丢帧判据)。
    sdkPollLogLines.push_back("poll_index,pc_recv_us,pc_delta_us,ret,ret_name,frame_ts_us,frame_ts_delta_us,point_count,recv_frame_seq,frame_channel,channel_delta");
    {
        qint64 prevPollPcUs = -1;
        qint64 prevFrameTsUs = -1;
        int sdkPollIndex = 0;
        int recvFrameSeq = 0;
        int prevFrameChannel = -1;
        for (const CameraFrameCache::PollStatus& poll : pollStatusLog)
        {
            QString retName;
            if (poll.ret == 0) retName = QStringLiteral("ok");
            else if (poll.ret == -106) retName = QStringLiteral("skj_no_data");
            else if (poll.ret == -107) retName = QStringLiteral("skj_duplicate");
            else retName = QStringLiteral("skj_error(%1)").arg(poll.ret);

            const qint64 pcDeltaUs = (prevPollPcUs >= 0) ? (poll.receiveTimestampUs - prevPollPcUs) : 0;
            QString frameTsField;
            QString frameDeltaField;
            QString pointField;
            QString frameSeqField;
            QString frameChannelField;
            QString channelDeltaField;
            if (poll.ret == 0)
            {
                frameTsField = QString::number(poll.frameTimestampUs);
                if (prevFrameTsUs >= 0)
                {
                    frameDeltaField = QString::number(poll.frameTimestampUs - prevFrameTsUs);
                }
                pointField = QString::number(poll.pointCount);
                frameSeqField = QString::number(++recvFrameSeq);
                if (poll.frameChannel >= 0)
                {
                    frameChannelField = QString::number(poll.frameChannel);
                    if (prevFrameChannel >= 0)
                    {
                        channelDeltaField = QString::number(poll.frameChannel - prevFrameChannel);
                    }
                    prevFrameChannel = poll.frameChannel;
                }
                prevFrameTsUs = poll.frameTimestampUs;
            }
            QStringList sdkPollFields;
            sdkPollFields << QString::number(sdkPollIndex++)
                << QString::number(poll.receiveTimestampUs)
                << QString::number(pcDeltaUs)
                << QString::number(poll.ret)
                << retName
                << frameTsField
                << frameDeltaField
                << pointField
                << frameSeqField
                << frameChannelField
                << channelDeltaField;
            sdkPollLogLines.push_back(sdkPollFields.join(','));
            prevPollPcUs = poll.receiveTimestampUs;
        }
    }

    const qint64 outputBuildElapsedMs = SteadyNowMs() - outputBuildStartMs;

    if (RobotOperationLease::IsCancellationRequested(pRobotDriver))
    {
        savedPath.clear();
        if (appendLog)
        {
            appendLog(QStringLiteral("扫描输出写盘前检测到安全停止，已取消本轮文件保存。"));
        }
        return false;
    }

    QString error;
    auto saveTextLinesTimed = [this, &error](const QString& filePath, const std::vector<QString>& lines, qint64& elapsedMs)
        {
            const qint64 saveStartMs = SteadyNowMs();
            const bool ok = SaveTextLines(filePath, lines, error);
            elapsedMs = SteadyNowMs() - saveStartMs;
            return ok;
        };
    qint64 saveCameraElapsedMs = 0;
    qint64 saveRobotElapsedMs = 0;
    qint64 saveLaserElapsedMs = 0;
    qint64 saveWorkpieceCloudElapsedMs = 0;
    qint64 saveMatchDebugElapsedMs = 0;
    if (!saveTextLinesTimed(cameraPath, cameraLines, saveCameraElapsedMs)
        || !saveTextLinesTimed(robotPath, robotLines, saveRobotElapsedMs)
        || !saveTextLinesTimed(laserPath, laserLines, saveLaserElapsedMs)
        || !saveTextLinesTimed(workpieceCloudPath, workpieceCloudLines, saveWorkpieceCloudElapsedMs)
        || !saveTextLinesTimed(matchDebugPath, matchDebugLines, saveMatchDebugElapsedMs))
    {
        if (appendLog)
        {
            appendLog(error);
        }
        return false;
    }
    // SDK 逐次取帧调用日志（可选，失败不中断主流程）：诊断帧时间戳跳变=我方漏取还是上游少给。
    if (sdkPollLogLines.size() > 1)
    {
        const QString sdkPollLogPath = QDir(laserDir).filePath("PreciseLaserPoint_SdkPollLog.csv");
        QString sdkPollLogError;
        if (SaveTextLines(sdkPollLogPath, sdkPollLogLines, sdkPollLogError))
        {
            if (appendLog)
            {
                appendLog(QString("SDK 取帧调用日志：%1（共 %2 次调用；帧时间戳跳变处对齐 ret/pc_delta 判丢帧来源）")
                    .arg(sdkPollLogPath).arg(static_cast<int>(pollStatusLog.size())));
            }
        }
        else if (appendLog)
        {
            appendLog(QString("SDK 取帧调用日志导出失败（不影响流程）：%1").arg(sdkPollLogError));
        }
    }
    // 扫描期间后台采集的相机图像：从临时目录整体挪进本次结果目录（同盘 rename，瞬间完成）。
    // 采集窗口早在扫描运动结束时已关闭，此刻距停窗已数秒，保存队列必已排干。空目录直接删除。
    if (!imageCaptureTmpDir.isEmpty())
    {
        QDir imageTmpDir(imageCaptureTmpDir);
        const int imageCount = imageTmpDir.exists()
            ? static_cast<int>(imageTmpDir.entryList(QStringList() << "*.webp" << "*.jpg", QDir::Files).size())
            : 0;
        if (imageCount > 0)
        {
            const QString imageTargetDir = QDir(resultDir).filePath("CameraImage");
            if (QDir().rename(imageCaptureTmpDir, imageTargetDir))
            {
                if (appendLog)
                {
                    appendLog(QString("相机图像已保存：%1 张，目录=%2（文件名=img_<SDK图像时间戳>_<PC接收us>.webp，可与取帧日志时间轴对齐）")
                        .arg(imageCount)
                        .arg(imageTargetDir));
                }
            }
            else if (appendLog)
            {
                appendLog(QString("相机图像归档失败（仍保留在临时目录）：%1（%2 张）").arg(imageCaptureTmpDir).arg(imageCount));
            }
        }
        else if (imageTmpDir.exists())
        {
            imageTmpDir.removeRecursively();  // 图像口未连接/无帧：清掉空临时目录
        }
    }
    // 完整点云逐帧调试文件（可选，失败不中断主流程）：供 CloudCompare 按 frame_index 着色排查散点。
    if (exportWorkpieceFrameDebug && workpieceFrameDebugLines.size() > 1)
    {
        const QString workpieceFrameDebugPath =
            QDir(laserDir).filePath("PreciseLaserPoint_WorkpieceCloud_FrameDebug.txt");
        QString frameDebugError;
        if (SaveTextLines(workpieceFrameDebugPath, workpieceFrameDebugLines, frameDebugError))
        {
            if (appendLog)
            {
                appendLog(QString("完整点云逐帧调试导出：%1（CloudCompare 按 frame_index 标量着色可定位起终点散点来自哪几帧）")
                    .arg(workpieceFrameDebugPath));
            }
        }
        else if (appendLog)
        {
            appendLog(QString("完整点云逐帧调试导出失败（不影响流程）：%1").arg(frameDebugError));
        }
    }
    const qint64 saveAllElapsedMs =
        saveCameraElapsedMs
        + saveRobotElapsedMs
        + saveLaserElapsedMs
        + saveWorkpieceCloudElapsedMs
        + saveMatchDebugElapsedMs;

    if (appendLog)
    {
        const std::uint64_t cacheFrameSpan =
            scanEndCameraSequence >= scanStartCameraSequence
            ? scanEndCameraSequence - scanStartCameraSequence
            : 0;
        const double cacheEffectiveFps =
            scanMotionElapsedMs > 0
            ? static_cast<double>(cacheFrameSpan) * 1000.0 / static_cast<double>(scanMotionElapsedMs)
            : 0.0;
        appendLog(QString("扫描完成，相机点=%1，机器人采样=%2，已匹配相机点=%3，保存目录=%4")
            .arg(static_cast<int>(cameraSamples.size()))
            .arg(static_cast<int>(robotSamples.size()))
            .arg(static_cast<int>(matchedCameraSamples.size()))
            .arg(resultDir));
        appendLog(QString("扫描期间已处理相机帧=%1，缓存序号范围=(%2, %3]，缓存新增=%4，估算缓存FPS=%5")
            .arg(static_cast<int>(cameraSamples.size() + invalidCameraTimestampCount + cameraBeforeRobotTimeBaseCount))
            .arg(scanStartCameraSequence)
            .arg(scanEndCameraSequence)
            .arg(static_cast<qulonglong>(cacheFrameSpan))
            .arg(cacheEffectiveFps, 0, 'f', 2));
        if (hasCameraToRobotTimeOffset)
        {
            if (statRealignApplied)
            {
                // 统计对齐生效后 cameraToRobotTimeOffsetUs 已是统计值，与首帧两时间戳之差不再相等——分开注明，避免现场按旧句式手算对不上。
                appendLog(QString("相机时间轴映射到机器人采样时间轴：映射偏移=%1 us（统计对齐值；首帧粗对齐值=%2 us，首帧相机timestamp=%3 us，对齐机器人时间=%4 us），额外补偿=%5 ms。")
                    .arg(cameraToRobotTimeOffsetUs)
                    .arg(statFirstFrameOffsetUs)
                    .arg(firstCameraRawTimestampUs)
                    .arg(firstRobotTimestampUs)
                    .arg(param.dCameraTimeOffsetMs, 0, 'f', 3));
            }
            else
            {
                appendLog(QString("相机时间轴映射到机器人采样时间轴：首帧相机timestamp=%1 us，对齐机器人时间=%2 us，映射偏移=%3 us，额外补偿=%4 ms。")
                    .arg(firstCameraRawTimestampUs)
                    .arg(firstRobotTimestampUs)
                    .arg(cameraToRobotTimeOffsetUs)
                    .arg(param.dCameraTimeOffsetMs, 0, 'f', 3));
            }
        }
        if (droppedHeadCameraCount > 0)
        {
            appendLog(QString("有 %1 个相机点早于首个机器人时间戳，已跳过未参与插值。").arg(droppedHeadCameraCount));
        }
        if (cameraBeforeRobotTimeBaseCount > 0)
        {
            appendLog(QString("有 %1 个相机点早于机器人时间基准建立，已跳过未参与插值。").arg(cameraBeforeRobotTimeBaseCount));
        }
        if (invalidCameraTimestampCount > 0)
        {
            appendLog(QString("有 %1 个相机点timestamp无效，已跳过未参与插值。").arg(invalidCameraTimestampCount));
        }
        appendLog(QString("相机原始timestamp间隔统计：倒退次数=%1，大跳次数=%2，大跳阈值=%3 us，最大间隔=%4 us。")
            .arg(cameraTimestampBackwardsCount)
            .arg(cameraTimestampJumpCount)
            .arg(cameraTimestampJumpWarnUs)
            .arg(maxCameraRawDeltaUs));
        if (droppedTailCameraCount > 0)
        {
            appendLog(QString("有 %1 个相机点晚于最后一个机器人时间戳，等待 %2 ms 后仍未匹配到机器人位姿，已跳过未参与插值。")
                .arg(droppedTailCameraCount)
                .arg(CAMERA_ROBOT_MATCH_TAIL_WAIT_MS));
        }
        appendLog(QString("激光计算有效点=%1，跳过异常相机点=%2")
            .arg(static_cast<int>(laserLines.size()) - 1)
            .arg(skippedLaserCount));
        appendLog(QString("局部完整点云：参与帧=%1，点数=%2，跳过异常线点=%3")
            .arg(workpieceCloudFrameCount)
            .arg(workpieceCloudPointCount)
            .arg(skippedWorkpieceCloudPointCount));
        appendLog(QString("扫描耗时统计：运动等待=%1 ms，尾部匹配等待=%2 ms，并行点云处理墙钟=%3 ms（运动后等待=%4 ms），输出构建=%5 ms（相机文本=%6 ms，明细组装=%7 ms），文件写入=%8 ms（相机=%9 ms，机器人=%10 ms，激光=%11 ms，完整点云=%12 ms，匹配明细=%13 ms）。")
            .arg(scanMotionElapsedMs)
            .arg(tailWaitElapsedMs)
            .arg(parallelProcessingElapsedMs)
            .arg(postMotionProcessingWaitMs)
            .arg(outputBuildElapsedMs)
            .arg(cameraLineBuildElapsedMs)
            .arg(pointComputeElapsedMs)
            .arg(saveAllElapsedMs)
            .arg(saveCameraElapsedMs)
            .arg(saveRobotElapsedMs)
            .arg(saveLaserElapsedMs)
            .arg(saveWorkpieceCloudElapsedMs)
            .arg(saveMatchDebugElapsedMs));
        // 完整点云读写速率提示：明确这步是不是瓶颈、txt 写盘有多快（对比 SDK 计算 parallelProcessing 耗时）。
        const double workpieceCloudMB = static_cast<double>(QFileInfo(workpieceCloudPath).size()) / (1024.0 * 1024.0);
        const double workpieceWriteMBps = saveWorkpieceCloudElapsedMs > 0
            ? workpieceCloudMB * 1000.0 / static_cast<double>(saveWorkpieceCloudElapsedMs)
            : 0.0;
        appendLog(QString("完整点云写盘：%1 个点 / %2 MB / %3 ms / %4 MB/s；SDK点云算法等处理墙钟=%5 ms（对比：处理>写盘则瓶颈在计算，反之在写txt）。")
            .arg(workpieceCloudPointCount)
            .arg(workpieceCloudMB, 0, 'f', 1)
            .arg(saveWorkpieceCloudElapsedMs)
            .arg(workpieceWriteMBps, 0, 'f', 1)
            .arg(parallelProcessingElapsedMs));
        appendLog(QString("激光点序号断点统计：断点段数=%1，最大连续缺失帧数=%2，匹配前丢弃=%3，匹配后丢弃=%4，未知未匹配=%5")
            .arg(laserIndexGapCount)
            .arg(maxLaserIndexGap)
            .arg(unmatchedBeforeRobotCount)
            .arg(unmatchedAfterRobotCount)
            .arg(unmatchedUnknownCount));
        appendLog(QString("相机点文件：%1").arg(cameraPath));
        appendLog(QString("机器人插值位姿文件：%1").arg(robotPath));
        appendLog(QString("激光点文件：%1").arg(laserPath));
        appendLog(QString("局部完整点云文件：%1").arg(workpieceCloudPath));
        appendLog(QString("相机-机器人-激光匹配明细文件：%1").arg(matchDebugPath));
    }

    // 扫描期间 SDK 取帧状态汇总 + 数据完整性门禁：连续无新帧超过阈值（或全程无帧）则判本次扫描数据不完整，
    // 报错并终止流程（不进入后续特征分析/焊接）。仅在 SKJ 取帧路径有逐帧状态记录时启用，避免误伤其它相机路径。
    if (!pollStatusLog.empty())
    {
        if (appendLog)
        {
            appendLog(QString("SDK 取帧状态统计：取到新帧=%1，无新帧(-106)=%2，重复帧(-107)=%3，取帧错误=%4，最长无新帧间断=%5 ms。")
                .arg(okPollCount)
                .arg(sdkNoDataPollCount)
                .arg(sdkDuplicatePollCount)
                .arg(sdkErrorPollCount)
                .arg(maxNoFrameGapUs / 1000));
        }
        if (okPollCount == 0)
        {
            if (appendLog)
            {
                appendLog("扫描期间相机全程未取到任何有效帧（SDK 无新帧/取帧失败），本次扫描数据无效，已终止流程。");
            }
            return false;
        }
        if (maxNoFrameGapUs > CAMERA_NO_FRAME_FAIL_GAP_US)
        {
            if (appendLog)
            {
                appendLog(QString("扫描期间相机连续 %1 ms 未取到新帧（最大间断，超过 %2 ms 阈值），本次扫描数据不完整，已终止流程。")
                    .arg(maxNoFrameGapUs / 1000)
                    .arg(CAMERA_NO_FRAME_FAIL_GAP_US / 1000));
            }
            return false;
        }
    }

    const PointCloudProcessingConfig::Settings pointCloudSettings = PointCloudProcessingConfig::Load();
    RobotCalculation::LowerWeldFilterParams originalFitParams =
        BuildOriginalTrackFitParams(param, pointCloudSettings);
    if (originalFitParams.exportFitDebugCloud && !laserDir.isEmpty())
    {
        // 真机路径把拟合调试点云导出到本次结果的 LaserPoint 目录下（FitDebug 子目录）。
        originalFitParams.fitDebugDir = laserDir;
    }
    // ①②③ 三种点云链方法都以完整点云为输入（③另需相机轨迹点做投影种子，④只用激光轨迹点）。
    const bool canUseExternalCloud =
        pointCloudSettings.mode != PointCloudProcessingConfig::Mode::LegacyLaserPath
        && workpieceCloudInput.size() >= 2;
    if (laserFitInput.size() < 2 && !canUseExternalCloud)
    {
        error = QString("激光有效点过少（%1），完整点云有效点=%2，无法生成可验证焊道。")
            .arg(laserFitInput.size())
            .arg(workpieceCloudInput.size());
        if (appendLog)
        {
            appendLog(error);
        }
        return false;
    }

    if (setFlowStep)
    {
        setFlowStep("扫描完成，正在进行先测后焊特征分析");
    }
    if (appendLog)
    {
        appendLog(QString("开始先测后焊特征分析：采样主轴=%1，重采样步长=%2 mm，拐点拟合容差=%3 mm，每段最少点数=%4")
            .arg(SampleAxisName(originalFitParams.sampleAxis))
            .arg(originalFitParams.sampleStep, 0, 'f', 3)
            .arg(originalFitParams.piecewiseFitTolerance, 0, 'f', 3)
            .arg(originalFitParams.piecewiseMinSegmentPoints));
    }

    bool usedExternalLibrary = false;
    PointCloudExtractionProcessor::ExtractionResult externalExtraction;
    const RobotCalculation::MeasureThenWeldAnalysisResult originalAnalysis =
        AnalyzeMeasureThenWeldPointCloud(
            laserFitInput,
            workpieceCloudInput,
            param,
            pointCloudSettings,
            originalFitParams,
            sdkBaseWeldPath,
            laserDir,
            appendLog,
            &usedExternalLibrary,
            &externalExtraction);
    EnsureWorkpieceMeshCacheFromCloud(laserDir, workpieceCloudInput, appendLog);
    if (!originalAnalysis.ok)
    {
        error = QString("先测后焊特征分析失败：%1").arg(originalAnalysis.error);
        QString reportError;
        if (!WritePointCloudQualityGate(
                laserDir,
                QString::fromStdString(param.sRobotName),
                pointCloudSettings,
                originalAnalysis.qualityReport,
                QStringList() << laserPath << workpieceCloudPath,
                QString(),
                QString(),
                -1,
                QString(),
                QString(),
                -1,
                productionContext,
                QStringLiteral("liveScan"),
                reportError,
                [pRobotDriver]()
                {
                    return RobotOperationLease::IsCancellationRequested(pRobotDriver);
                })
            && appendLog)
        {
            appendLog(QString("保存点云质量失败报告失败：%1").arg(reportError));
        }
        if (appendLog)
        {
            appendLog(error);
            appendLog("已保留原始激光点文件，可先按原始点云继续分析。");
        }
        return false;
    }
    if (appendLog && (!originalAnalysis.qualityReport.failures.isEmpty()
        || !originalAnalysis.qualityReport.warnings.isEmpty()))
    {
        appendLog(QString("点云质量%1：失败项=%2；告警项=%3")
            .arg(originalFitParams.validationAuditOnly ? "审计" : "门禁")
            .arg(originalAnalysis.qualityReport.failures.join(" | "))
            .arg(originalAnalysis.qualityReport.warnings.join(" | ")));
    }

    if (usedExternalLibrary)
    {
        QDir().mkpath(sdkPointCloudDir);
        if (!SaveTextLines(sdkSeamExtractedPath, BuildSdkTrackOutputLines(externalExtraction.rawPoints, "sdk_extracted"), error))
        {
            if (appendLog)
            {
                appendLog(QString("保存SDK提取焊道结果失败：%1").arg(error));
            }
            return false;
        }
        if (!SaveTextLines(
                sdkSeamExtracted2mmPath,
                BuildSdkTrackOutputLines(externalExtraction.keyPointExpandedPoints, "sdk_keypoint_2mm"),
                error))
        {
            if (appendLog)
            {
                appendLog(QString("保存SDK提取焊道2mm采样结果失败：%1").arg(error));
            }
            return false;
        }
        if (appendLog)
        {
            appendLog(QString("SDK提取点云焊道文件：%1，点数=%2")
                .arg(sdkSeamExtractedPath)
                .arg(externalExtraction.rawPoints.size()));
            appendLog(QString("SDK提取点云焊道2mm采样文件：%1，点数=%2")
                .arg(sdkSeamExtracted2mmPath)
                .arg(externalExtraction.keyPointExpandedPoints.size()));
            appendLog(QString("SDK基础焊道文件：%1，点数=%2")
                .arg(externalExtraction.baseWeldPath.isEmpty() ? sdkBaseWeldPath : externalExtraction.baseWeldPath)
                .arg(externalExtraction.points.size()));
        }
        QString schemeCompareError;
        if (!SaveSdkSchemeCompareOutputs(
                *this,
                sdkPointCloudDir,
                laserFitInput,
                externalExtraction,
                originalFitParams,
                schemeCompareError,
                appendLog)
            && appendLog)
        {
            appendLog(QString("SDK三方案对比输出失败：%1").arg(schemeCompareError));
        }
    }

    if (!SaveTextLines(preservePathFitPath, BuildFilterOutputLines(originalAnalysis.filterResult), error))
    {
        if (appendLog)
        {
            appendLog(QString("保存先测后焊特征提取结果失败：%1").arg(error));
        }
        return false;
    }

    if (appendLog)
    {
        appendLog(QString("先测后焊特征提取完成：输入=%1，输出=%2，文件=%3")
            .arg(originalAnalysis.filterResult.inputPointCount)
            .arg(originalAnalysis.filterResult.points.size())
            .arg(preservePathFitPath));
        if (usedExternalLibrary)
        {
            appendLog("本次 PreservePath 来自新版精测点云库输出。");
        }
        appendLog(FilterResultSummary("先测后焊特征提取", originalFitParams, originalAnalysis.filterResult, preservePathFitPath));
    }

    if (setFlowStep)
    {
        setFlowStep("先测后焊特征提取完成，正在进行焊道分类");
    }
    if (!SaveTextLines(classifiedPath, BuildClassifiedOutputLines(originalAnalysis.classificationResult), error))
    {
        if (appendLog)
        {
            appendLog(QString("保存焊道分类结果失败：%1").arg(error));
        }
        return false;
    }

    // 圆弧调试预览只能由最终 _SeamComp 结果生成。先清掉旧版本在分类阶段
    // 提前生成的同名文件，避免失败/中断后把陈旧圆弧误认为本轮中间结果。
    {
        const QString arcPreviewPath =
            QDir(laserDir).filePath(QStringLiteral("PreciseLaserPoint_ArcTransitionPreview.txt"));
        if (QFileInfo::exists(arcPreviewPath)
            && !QFile::remove(arcPreviewPath)
            && appendLog)
        {
            appendLog(QString("旧圆弧过渡预览未能删除，将在最终后处理完成后覆盖：%1")
                .arg(arcPreviewPath));
        }
    }

    if (!SaveTextLines(keyPointsPath, BuildKeyPointOutputLines(originalAnalysis.keyPoints), error))
    {
        if (appendLog)
        {
            appendLog(QString("保存起终点/拐点结果失败：%1").arg(error));
        }
        return false;
    }

    if (!SaveTextLines(classifiedNoisePath, BuildNoiseOutputLines(laserFitInput, originalAnalysis.filterResult), error))
    {
        if (appendLog)
        {
            appendLog(QString("保存焊道杂点结果失败：%1").arg(error));
        }
        return false;
    }

    const bool useCornerCompensatedClassification =
        originalAnalysis.cornerCompensatedClassificationResult.ok
        && !originalAnalysis.cornerCompensatedClassificationResult.points.isEmpty();
    if (useCornerCompensatedClassification)
    {
        if (!SaveTextLines(
                cornerCompClassifiedPath,
                BuildClassifiedOutputLines(originalAnalysis.cornerCompensatedClassificationResult),
                error))
        {
            if (appendLog)
            {
                appendLog(QString("保存拐点补偿后焊道分类结果失败：%1").arg(error));
            }
            return false;
        }
        if (!SaveTextLines(cornerCompKeyPointsPath, BuildKeyPointOutputLines(originalAnalysis.cornerCompensatedKeyPoints), error))
        {
            if (appendLog)
            {
                appendLog(QString("保存拐点补偿后起终点/拐点结果失败：%1").arg(error));
            }
            return false;
        }
    }
    else if (originalFitParams.enableCornerCompensation && appendLog)
    {
        appendLog(QString("拐点补偿未生成：%1").arg(
            originalAnalysis.cornerCompensatedClassificationResult.error.isEmpty()
                ? QStringLiteral("未找到可补偿的上坡/下坡拐点或补偿值为 0。")
                : originalAnalysis.cornerCompensatedClassificationResult.error));
    }

    if (appendLog)
    {
        int startCount = 0;
        int endCount = 0;
        int innerCount = 0;
        int outerCount = 0;
        int normalCount = 0;
        for (const RobotCalculation::LowerWeldClassifiedPoint& point : originalAnalysis.classificationResult.points)
        {
            switch (point.type)
            {
            case RobotCalculation::LowerWeldPointType::Start:
                ++startCount;
                break;
            case RobotCalculation::LowerWeldPointType::End:
                ++endCount;
                break;
            case RobotCalculation::LowerWeldPointType::InnerCorner:
                ++innerCount;
                break;
            case RobotCalculation::LowerWeldPointType::OuterCorner:
                ++outerCount;
                break;
            case RobotCalculation::LowerWeldPointType::Normal:
                ++normalCount;
                break;
            default:
                break;
            }
        }

        appendLog(QString("焊道分类完成：起点=%1，终点=%2，内拐点=%3，外拐点=%4，普通点=%5，分类文件=%6")
            .arg(startCount)
            .arg(endCount)
            .arg(innerCount)
            .arg(outerCount)
            .arg(normalCount)
            .arg(classifiedPath));
        appendLog(QString("起终点/拐点文件：%1").arg(keyPointsPath));
        appendLog(QString("焊道杂点文件：%1").arg(classifiedNoisePath));
        if (useCornerCompensatedClassification)
        {
            appendLog(QString("拐点补偿后分类文件：%1").arg(cornerCompClassifiedPath));
            appendLog(QString("拐点补偿后起终点/拐点文件：%1").arg(cornerCompKeyPointsPath));
            appendLog("焊接姿态将使用拐点补偿后分类点生成。");
        }
        appendLog(QString("先测后焊特征分析摘要：输入=%1，下层候选=%2，输出=%3，剔除Z突变=%4，剔除Z连续异常=%5，连续段剔除=%6")
            .arg(originalAnalysis.filterResult.inputPointCount)
            .arg(originalAnalysis.filterResult.lowerPointCount)
            .arg(originalAnalysis.filterResult.points.size())
            .arg(originalAnalysis.filterResult.zJumpRejectedCount)
            .arg(originalAnalysis.filterResult.zContinuityRejectedCount)
            .arg(originalAnalysis.filterResult.segmentRejectedCount));
    }

    const MeasurementPoseReference measurementPoseReference =
        FirstMeasurementPoseReferenceFromProcessedSamples(processedCameraSamples, matchDebugPath);
    const WeldPosePreset weldPosePreset = ApplyMeasurementPoseReferenceForCalculation(
        LoadWeldPosePreset(param),
        measurementPoseReference,
        appendLog);
    if (appendLog)
    {
        appendLog(QString("焊接姿态参数：模式=%1, RX=%2, RY=%3, 示教RZ=%4 deg, RZ增益=%5 deg, 爬坡RZ夹紧=[%6, %7] deg, 拐点前过渡=%8 mm, 起点跳过=%9 mm, 终点跳过=%10 mm, 姿态补偿槽=%11, 焊道补偿=%12, 基础参数来源=%13, 姿态补偿来源=%14, 焊道补偿来源=%15")
            .arg(weldPosePreset.useTaughtWeldPose ? QStringLiteral("示教平台姿态") : QStringLiteral("原始固定姿态"))
            .arg(weldPosePreset.useTaughtWeldPose ? weldPosePreset.taughtWeldPoseRx : weldPosePreset.rx, 0, 'f', 3)
            .arg(weldPosePreset.useTaughtWeldPose ? weldPosePreset.taughtWeldPoseRy : weldPosePreset.ry, 0, 'f', 3)
            .arg(weldPosePreset.taughtWeldPoseRz, 0, 'f', 3)
            .arg(weldPosePreset.weldRzGainDeg, 0, 'f', 3)
            .arg(weldPosePreset.slopeRzMinDeg, 0, 'f', 3)
            .arg(weldPosePreset.slopeRzMaxDeg, 0, 'f', 3)
            .arg(weldPosePreset.cornerTransitionLeadDistance, 0, 'f', 3)
            .arg(weldPosePreset.weldStartSkipDistance, 0, 'f', 3)
            .arg(weldPosePreset.weldEndSkipDistance, 0, 'f', 3)
            .arg(static_cast<int>(weldPosePreset.poseCompSlots.size()))
            .arg(QStringLiteral("整条统一"))
            .arg(weldPosePreset.weldLineFromIni
                ? QString("%1 [%2]").arg(weldPosePreset.weldLineFilePath, weldPosePreset.weldLineSectionName)
                : QString("扫描起点姿态回退"))
            .arg(weldPosePreset.poseCompFromIni ? weldPosePreset.poseCompFilePath : QString("默认值"))
            .arg(weldPosePreset.seamCompFromIni ? weldPosePreset.seamCompFilePath : QString("默认值")));
    }

    if (setFlowStep)
    {
        setFlowStep("焊道分类完成，正在生成分段焊接姿态");
    }

    const RobotCalculation::LowerWeldClassificationResult& classificationForWeldPose =
        useCornerCompensatedClassification
            ? originalAnalysis.cornerCompensatedClassificationResult
            : originalAnalysis.classificationResult;
    const std::vector<QString> weldPoseLines =
        BuildSegmentPoseOutputLines(
            classificationForWeldPose,
            param,
            weldPosePreset,
            originalFitParams.platformSnapFlatSlope,
            appendLog);
    if (!weldPoseLines.empty())
    {
        if (!SaveTextLines(weldPosePath, weldPoseLines, error))
        {
            if (appendLog)
            {
                appendLog(QString("保存焊接姿态结果失败：%1").arg(error));
            }
            return false;
        }

        if (appendLog)
        {
            appendLog(QString("焊接姿态文件：%1").arg(weldPosePath));
        }

        QString seamCompSummary;
        QString generatedSeamCompSha256;
        qint64 generatedSeamCompSize = -1;
        if (!ApplyWeldSeamCompToPoseFile(
            QString::fromStdString(param.sRobotName),
            weldPosePath,
            weldPoseSeamCompPath,
            seamCompSummary,
            error,
            &generatedSeamCompSha256,
            &generatedSeamCompSize,
            [pRobotDriver]()
            {
                return RobotOperationLease::IsCancellationRequested(pRobotDriver);
            }))
        {
            if (appendLog)
            {
                appendLog(QString("保存焊道补偿后文件失败：%1").arg(error));
                appendLog(QString("已保留姿态文件，可在修正补偿参数后重新生成：%1").arg(weldPosePath));
            }
            return false;
        }

        QString validatedSourcePoseSha256;
        qint64 validatedSourcePoseSize = -1;
        QString validatedPoseSha256;
        qint64 validatedPoseSize = -1;
        const bool finalArtifactValid = ValidateFinalWeldPoseArtifact(
                weldPoseSeamCompPath,
                weldPosePath,
                generatedSeamCompSha256,
                generatedSeamCompSize,
                weldPosePreset.robotType,
                weldPosePreset.weldStartSkipDistance,
                weldPosePreset.weldEndSkipDistance,
                pointCloudSettings,
                originalAnalysis.qualityReport,
                validatedSourcePoseSha256,
                validatedSourcePoseSize,
                validatedPoseSha256,
                validatedPoseSize,
                error,
                [pRobotDriver]()
                {
                    return RobotOperationLease::IsCancellationRequested(pRobotDriver);
                });
        if (!finalArtifactValid)
        {
            if (pointCloudSettings.validationPolicy
                == PointCloudProcessingConfig::ValidationPolicy::Enforce)
            {
                if (appendLog)
                {
                    appendLog(QString("最终焊接姿态写后回读验证失败：%1").arg(error));
                }
                return false;
            }
            if (appendLog)
            {
                appendLog(QString("Audit：最终焊接姿态结构验证未通过，仅保留未授权审计证据：%1")
                    .arg(error));
            }
            error.clear();
            validatedSourcePoseSha256.clear();
            validatedSourcePoseSize = -1;
            validatedPoseSha256.clear();
            validatedPoseSize = -1;
        }
        if (!WritePointCloudQualityGate(
                laserDir,
                QString::fromStdString(param.sRobotName),
                pointCloudSettings,
                originalAnalysis.qualityReport,
                QStringList() << laserPath << workpieceCloudPath,
                weldPosePath,
                validatedSourcePoseSha256,
                validatedSourcePoseSize,
                weldPoseSeamCompPath,
                validatedPoseSha256,
                validatedPoseSize,
                productionContext,
                QStringLiteral("liveScan"),
                error,
                [pRobotDriver]()
                {
                    return RobotOperationLease::IsCancellationRequested(pRobotDriver);
                }))
        {
            if (appendLog)
            {
                appendLog(QString("生成点云质量证明失败：%1").arg(error));
            }
            return false;
        }

        if (appendLog)
        {
            appendLog(QString("焊道补偿文件：%1").arg(weldPoseSeamCompPath));
            appendLog(QString("焊道补偿摘要：%1").arg(seamCompSummary));
            appendLog(QString("点云质量报告：%1")
                .arg(QDir(laserDir).filePath(QString::fromLatin1(POINT_CLOUD_QUALITY_GATE_FILE_NAME))));
        }
        if (pointCloudSettings.validationPolicy == PointCloudProcessingConfig::ValidationPolicy::Audit)
        {
            if (appendLog)
            {
                appendLog("当前为点云质量审计模式：已生成分析产物，但不会生成可执行证明或进入焊接。");
            }
            savedPath.clear();
            if (!qualityGateReplacement.Complete(qualityGateError))
            {
                if (appendLog)
                {
                    appendLog(QStringLiteral("审计产物已生成，但解除点云证明拒绝闭锁失败：")
                        + qualityGateError);
                }
                return false;
            }
            return true;
        }
        // 焊道补偿生成后立即同步生成 STEP job(srp/srd)到焊道同目录，便于提取查看，不必等下枪执行才保存。
        {
            QString jobName, jobSrp, jobSrd, jobSum, jobErr;
            if (GenerateStepWeldProgramFiles(QString::fromStdString(param.sRobotName), weldPoseSeamCompPath,
                    QFileInfo(weldPoseSeamCompPath).absolutePath(), true, 0.0, jobName, jobSrp, jobSrd, jobSum, jobErr,
                    0.0, true, WeldPoseSource::PointCloudProduction, productionExpectation, true))
            {
                if (appendLog) { appendLog(QString("STEP焊接程序(job)已同步生成：srp=%1，srd=%2").arg(jobSrp, jobSrd)); }
            }
            else if (appendLog) { appendLog(QString("STEP焊接程序(job)同步生成失败(不影响焊道)：%1").arg(jobErr)); }
        }
        savedPath = weldPoseSeamCompPath;
    }
    else
    {
        error = "焊接姿态生成结果为空，请检查起终点跳过距离或焊道分类结果。";
        if (appendLog)
        {
            appendLog(error);
        }
        return false;
    }
    if (RobotOperationLease::IsCancellationRequested(pRobotDriver))
    {
        savedPath.clear();
        if (appendLog)
        {
            appendLog(QStringLiteral("扫描后处理期间收到安全停止：本轮输出不进入后续焊接流程。"));
        }
        return false;
    }
    if (!qualityGateReplacement.Complete(qualityGateError))
    {
        savedPath.clear();
        if (appendLog)
        {
            appendLog(QStringLiteral("扫描产物完成，但解除点云证明拒绝闭锁失败：")
                + qualityGateError);
        }
        return false;
    }
    return true;
}

bool MeasureThenWeldService::RebuildWeldFilesFromLaserDir(
    const T_PRECISE_MEASURE_PARAM& param,
    const QString& laserDir,
    QString& preservePath,
    QString& weldPosePath,
    QString& seamCompPath,
    QString& summary,
    QString& error,
    const LogCallback& appendLog,
    const StepCallback& setFlowStep,
    const PointCloudProductionExpectation& productionExpectation,
    const StopRequestedCallback& stopRequested) const
{
    preservePath.clear();
    weldPosePath.clear();
    seamCompPath.clear();
    summary.clear();
    error.clear();

    if (stopRequested && stopRequested())
    {
        error = QStringLiteral("已取消重建（启动前）。");
        return false;
    }

    const QDir dir(laserDir);
    if (!dir.exists())
    {
        error = QString("LaserPoint目录不存在：%1").arg(laserDir);
        return false;
    }
    const PointCloudProcessingConfig::Settings pointCloudSettings =
        PointCloudProcessingConfig::Load();
    PointCloudProductionContext productionContext;
    if (pointCloudSettings.validationPolicy
        == PointCloudProcessingConfig::ValidationPolicy::Enforce)
    {
        if (productionExpectation.robotName.trimmed().isEmpty()
            || productionExpectation.robotEndpoint.trimmed().isEmpty()
            || productionExpectation.cameraSection.trimmed().isEmpty()
            || !IsSha256Text(productionExpectation.handEyeSha256))
        {
            error = QStringLiteral(
                "Enforce 生产重建必须提供当前机器人上下文；离线预览不能生成可运动授权。");
            return false;
        }
        if (!LoadValidatedRebuildPointCloudContext(
                laserDir,
                QString::fromStdString(param.sRobotName),
                productionExpectation,
                pointCloudSettings,
                productionContext,
                error,
                stopRequested))
        {
            return false;
        }
    }
    const QString qualityGatePath = dir.filePath(
        QString::fromLatin1(POINT_CLOUD_QUALITY_GATE_FILE_NAME));
    PointCloudProofReplacementSession qualityGateReplacement;
    if (!PointCloudProofIntegrity::BeginProofReplacement(
            qualityGatePath,
            QStringLiteral("validatedRebuild 尚未完整完成，拒绝任何旧/中间点云授权。"),
            error))
    {
        error = QStringLiteral("无法建立重建期间的点云证明持久拒绝闭锁：") + error;
        return false;
    }
    qualityGateReplacement.Arm(qualityGatePath);
    if (!InvalidatePointCloudQualityGate(laserDir, error))
    {
        error += QStringLiteral("；持久拒绝标记仍有效，旧证明不会被接受。");
        return false;
    }
    const auto cancellationPoint = [&](const QString& phase) -> bool
    {
        if (!stopRequested || !stopRequested())
        {
            return false;
        }
        // 只有完整成功后才解除持久拒绝标记。取消时再次尝试删除 proof；
        // 即使 Windows 独占/ACL 令删除失败，denied tombstone 仍会让
        // Verify fail-closed，并且删除错误必须向上报告。
        QString invalidateError;
        const bool invalidated = InvalidatePointCloudQualityGate(laserDir, invalidateError);
        preservePath.clear();
        weldPosePath.clear();
        seamCompPath.clear();
        summary.clear();
        error = QString("已取消重建（%1）；持久拒绝标记保持有效，未发布可执行质量证明。")
            .arg(phase);
        if (!invalidated)
        {
            error += QStringLiteral("；旧 proof 删除失败但仍被拒绝：") + invalidateError;
        }
        return true;
    };
    if (cancellationPoint(QStringLiteral("撤销旧质量证明后")))
    {
        return false;
    }

    preservePath = dir.filePath(PRESERVE_PATH_FILE_NAME);
    const QString workpieceCloudPath = dir.filePath(WORKPIECE_CLOUD_FILE_NAME);
    const QString sdkPointCloudDir = dir.filePath(SDK_POINT_CLOUD_OUTPUT_DIR_NAME);
    const QString sdkSeamExtractedPath = QDir(sdkPointCloudDir).filePath(SDK_SEAM_EXTRACTED_FILE_NAME);
    const QString sdkSeamExtracted2mmPath = QDir(sdkPointCloudDir).filePath(SDK_SEAM_EXTRACTED_2MM_FILE_NAME);
    const QString sdkBaseWeldPath = QDir(sdkPointCloudDir).filePath(SDK_BASE_WELD_FILE_NAME);
    const QString keyPointsPath = dir.filePath(KEY_POINTS_FILE_NAME);
    const QString classifiedPath = dir.filePath(CLASSIFIED_FILE_NAME);
    const QString cornerCompKeyPointsPath = dir.filePath(CORNER_COMP_KEY_POINTS_FILE_NAME);
    const QString cornerCompClassifiedPath = dir.filePath(CORNER_COMP_CLASSIFIED_FILE_NAME);
    const QString classifiedNoisePath = dir.filePath(CLASSIFIED_NOISE_FILE_NAME);
    const QString matchDebugPath = dir.filePath(MATCH_DEBUG_FILE_NAME);
    weldPosePath = dir.filePath(WELD_POSE_FILE_NAME);
    seamCompPath = dir.filePath(WELD_POSE_SEAM_COMP_FILE_NAME);

    QString sourceLaserPath = dir.filePath(RAW_LASER_FILE_NAME);
    if (!QFileInfo::exists(sourceLaserPath))
    {
        if (QFileInfo::exists(workpieceCloudPath))
        {
            sourceLaserPath = workpieceCloudPath;
            if (appendLog)
            {
                appendLog(QString("未找到原始激光点文件 %1，临时使用局部完整点云作为回退输入。").arg(RAW_LASER_FILE_NAME));
            }
        }
        else if (QFileInfo::exists(preservePath))
        {
            sourceLaserPath = preservePath;
            if (appendLog)
            {
                appendLog(QString("未找到原始激光点文件 %1，临时使用已有 PreservePath 文件作为重建输入。").arg(RAW_LASER_FILE_NAME));
            }
        }
        else
        {
            error = QString("未找到原始激光点文件：%1").arg(dir.filePath(RAW_LASER_FILE_NAME));
            return false;
        }
    }

    auto logReadRate = [&appendLog](const QString& label, const QString& path, int pointCount, qint64 elapsedMs)
    {
        if (!appendLog) return;
        const double mb = static_cast<double>(QFileInfo(path).size()) / (1024.0 * 1024.0);
        appendLog(QString("%1读取：%2 点 / %3 MB / %4 ms / %5 MB/s")
            .arg(label)
            .arg(pointCount)
            .arg(mb, 0, 'f', 1)
            .arg(elapsedMs)
            .arg(elapsedMs > 0 ? mb * 1000.0 / static_cast<double>(elapsedMs) : 0.0, 0, 'f', 1));
    };

    QVector<RobotCalculation::IndexedPoint3D> laserFitInput;
    {
        const qint64 readStartMs = SteadyNowMs();
        QString laserLoadError;
        if (RobotDataHelper::LoadIndexedPoint3DFile(
                sourceLaserPath, laserFitInput, &laserLoadError, stopRequested))
        {
            logReadRate(QStringLiteral("原始激光点"), sourceLaserPath, laserFitInput.size(), SteadyNowMs() - readStartMs);
        }
        else if (appendLog)
        {
            // 激光特征点为空/缺失不直接失败：点云链方法(①②③)只需完整点云，能否继续由下方 canUseExternalCloud 统一判定（与实时扫描路径一致）。
            appendLog(QString("原始激光点读取为空或失败：%1（点云链方法将改用完整点云）").arg(laserLoadError));
        }
    }

    QVector<RobotCalculation::IndexedPoint3D> workpieceCloudInput;
    QString workpieceLoadError;
    if (QFileInfo::exists(workpieceCloudPath))
    {
        const qint64 readStartMs = SteadyNowMs();
        if (RobotDataHelper::LoadIndexedPoint3DFile(
                workpieceCloudPath, workpieceCloudInput, &workpieceLoadError, stopRequested))
        {
            logReadRate(QStringLiteral("完整点云"), workpieceCloudPath, workpieceCloudInput.size(), SteadyNowMs() - readStartMs);
        }
        else if (appendLog)
        {
            appendLog(QString("读取局部完整点云失败：%1（点云链方法将因输入不足报错）").arg(workpieceLoadError));
        }
    }

    if (cancellationPoint(QStringLiteral("读取点云后")))
    {
        return false;
    }

    RobotCalculation::LowerWeldFilterParams originalFitParams =
        BuildOriginalTrackFitParams(param, pointCloudSettings);
    originalFitParams.stopRequested = stopRequested;

    // ①②③ 三种点云链方法都以完整点云为输入（③另需相机轨迹点做投影种子，④只用激光轨迹点）。
    const bool canUseExternalCloud =
        pointCloudSettings.mode != PointCloudProcessingConfig::Mode::LegacyLaserPath
        && workpieceCloudInput.size() >= 2;
    if (laserFitInput.size() < 2 && !canUseExternalCloud)
    {
        error = QString("激光有效点过少（%1），完整点云有效点=%2，无法重建焊接文件。")
            .arg(laserFitInput.size())
            .arg(workpieceCloudInput.size());
        return false;
    }

    if (setFlowStep)
    {
        setFlowStep("正在重新计算 PreservePath、焊接姿态和焊道补偿文件");
    }
    if (appendLog)
    {
        appendLog(QString("跳过扫描重建输入：%1，点数=%2").arg(sourceLaserPath).arg(laserFitInput.size()));
        appendLog(QString("局部完整点云输入：%1，点数=%2").arg(workpieceCloudPath).arg(workpieceCloudInput.size()));
        appendLog(QString("开始先测后焊特征分析：采样主轴=%1，重采样步长=%2 mm，拐点拟合容差=%3 mm，每段最少点数=%4")
            .arg(SampleAxisName(originalFitParams.sampleAxis))
            .arg(originalFitParams.sampleStep, 0, 'f', 3)
            .arg(originalFitParams.piecewiseFitTolerance, 0, 'f', 3)
            .arg(originalFitParams.piecewiseMinSegmentPoints));
    }

    bool usedExternalLibrary = false;
    PointCloudExtractionProcessor::ExtractionResult externalExtraction;
    const RobotCalculation::MeasureThenWeldAnalysisResult originalAnalysis =
        AnalyzeMeasureThenWeldPointCloud(
            laserFitInput,
            workpieceCloudInput,
            param,
            pointCloudSettings,
            originalFitParams,
            sdkBaseWeldPath,
            laserDir,
            appendLog,
            &usedExternalLibrary,
            &externalExtraction,
            stopRequested);
    if (cancellationPoint(QStringLiteral("特征分析后")))
    {
        return false;
    }
    // 网格缓存只是查看器加速材料，不影响重建/焊接安全。当调用方
    // 提供取消令牌（预览 worker）时不进入尚未支持中断的大网格构建。
    if (!stopRequested)
    {
        EnsureWorkpieceMeshCacheFromCloud(laserDir, workpieceCloudInput, appendLog);
    }
    else if (appendLog)
    {
        appendLog(QStringLiteral("可取消重建跳过非必要的工件网格缓存生成。"));
    }
    if (!originalAnalysis.ok)
    {
        error = QString("先测后焊特征分析失败：%1").arg(originalAnalysis.error);
        QString reportError;
        if (!WritePointCloudQualityGate(
                laserDir,
                QString::fromStdString(param.sRobotName),
                pointCloudSettings,
                originalAnalysis.qualityReport,
                QStringList() << sourceLaserPath << workpieceCloudPath,
                QString(),
                QString(),
                -1,
                QString(),
                QString(),
                -1,
                productionContext,
                QStringLiteral("validatedRebuild"),
                reportError,
                stopRequested)
            && appendLog)
        {
            appendLog(QString("保存点云质量失败报告失败：%1").arg(reportError));
        }
        return false;
    }

    if (usedExternalLibrary)
    {
        QDir().mkpath(sdkPointCloudDir);
        if (!SaveTextLines(sdkSeamExtractedPath, BuildSdkTrackOutputLines(externalExtraction.rawPoints, "sdk_extracted"), error, stopRequested))
        {
            return false;
        }
        if (!SaveTextLines(
                sdkSeamExtracted2mmPath,
                BuildSdkTrackOutputLines(externalExtraction.keyPointExpandedPoints, "sdk_keypoint_2mm"),
                error,
                stopRequested))
        {
            return false;
        }
        QString schemeCompareError;
        if (!SaveSdkSchemeCompareOutputs(
                *this,
                sdkPointCloudDir,
                laserFitInput,
                externalExtraction,
                originalFitParams,
                schemeCompareError,
                appendLog)
            && appendLog)
        {
            appendLog(QString("SDK三方案对比输出失败：%1").arg(schemeCompareError));
        }
    }

    if (cancellationPoint(QStringLiteral("SDK 辅助输出后")))
    {
        return false;
    }

    if (!SaveTextLines(preservePath, BuildFilterOutputLines(originalAnalysis.filterResult), error, stopRequested))
    {
        return false;
    }
    if (!SaveTextLines(classifiedPath, BuildClassifiedOutputLines(originalAnalysis.classificationResult), error, stopRequested))
    {
        return false;
    }

    // 圆弧调试预览只能由最终 _SeamComp 结果生成。先清掉旧版本在分类阶段
    // 提前生成的同名文件，避免失败/中断后把陈旧圆弧误认为本轮中间结果。
    {
        const QString arcPreviewPath = QDir(QFileInfo(classifiedPath).absolutePath())
            .filePath(QStringLiteral("PreciseLaserPoint_ArcTransitionPreview.txt"));
        if (QFileInfo::exists(arcPreviewPath)
            && !QFile::remove(arcPreviewPath)
            && appendLog)
        {
            appendLog(QString("旧圆弧过渡预览未能删除，将在最终后处理完成后覆盖：%1")
                .arg(arcPreviewPath));
        }
    }

    if (!SaveTextLines(keyPointsPath, BuildKeyPointOutputLines(originalAnalysis.keyPoints), error, stopRequested))
    {
        return false;
    }
    if (!SaveTextLines(classifiedNoisePath, BuildNoiseOutputLines(laserFitInput, originalAnalysis.filterResult), error, stopRequested))
    {
        return false;
    }
    const bool useCornerCompensatedClassification =
        originalAnalysis.cornerCompensatedClassificationResult.ok
        && !originalAnalysis.cornerCompensatedClassificationResult.points.isEmpty();
    if (useCornerCompensatedClassification)
    {
        if (!SaveTextLines(
                cornerCompClassifiedPath,
                BuildClassifiedOutputLines(originalAnalysis.cornerCompensatedClassificationResult),
                error,
                stopRequested))
        {
            return false;
        }
        if (!SaveTextLines(cornerCompKeyPointsPath, BuildKeyPointOutputLines(originalAnalysis.cornerCompensatedKeyPoints), error, stopRequested))
        {
            return false;
        }
    }
    else if (originalFitParams.enableCornerCompensation && appendLog)
    {
        appendLog(QString("拐点补偿未生成：%1").arg(
            originalAnalysis.cornerCompensatedClassificationResult.error.isEmpty()
                ? QStringLiteral("未找到可补偿的上坡/下坡拐点或补偿值为 0。")
                : originalAnalysis.cornerCompensatedClassificationResult.error));
    }

    if (cancellationPoint(QStringLiteral("原子写入特征产物后")))
    {
        return false;
    }

    QString measurementPoseError;
    const MeasurementPoseReference measurementPoseReference =
        LoadMeasurementPoseReferenceFromMatchDebug(matchDebugPath, &measurementPoseError);
    if (!measurementPoseReference.valid && appendLog)
    {
        appendLog(QString("读取点云测量姿态失败：%1").arg(measurementPoseError));
    }
    const WeldPosePreset weldPosePreset = ApplyMeasurementPoseReferenceForCalculation(
        LoadWeldPosePreset(param),
        measurementPoseReference,
        appendLog);
    if (appendLog)
    {
        appendLog(QString("先测后焊特征提取完成：输入=%1，输出=%2，文件=%3")
            .arg(originalAnalysis.filterResult.inputPointCount)
            .arg(originalAnalysis.filterResult.points.size())
            .arg(preservePath));
        if (usedExternalLibrary)
        {
            appendLog("本次 PreservePath 来自新版精测点云库输出。");
            appendLog(QString("SDK提取点云焊道文件：%1，点数=%2")
                .arg(sdkSeamExtractedPath)
                .arg(externalExtraction.rawPoints.size()));
            appendLog(QString("SDK提取点云焊道2mm采样文件：%1，点数=%2")
                .arg(sdkSeamExtracted2mmPath)
                .arg(externalExtraction.keyPointExpandedPoints.size()));
            appendLog(QString("SDK基础焊道文件：%1，点数=%2")
                .arg(externalExtraction.baseWeldPath.isEmpty() ? sdkBaseWeldPath : externalExtraction.baseWeldPath)
                .arg(externalExtraction.points.size()));
        }
        appendLog(QString("焊道分类文件：%1").arg(classifiedPath));
        appendLog(QString("起终点/拐点文件：%1").arg(keyPointsPath));
        appendLog(QString("焊道杂点文件：%1").arg(classifiedNoisePath));
        if (useCornerCompensatedClassification)
        {
            appendLog(QString("拐点补偿后分类文件：%1").arg(cornerCompClassifiedPath));
            appendLog(QString("拐点补偿后起终点/拐点文件：%1").arg(cornerCompKeyPointsPath));
            appendLog("焊接姿态将使用拐点补偿后分类点生成。");
        }
        appendLog(QString("焊接姿态参数：模式=%1, RX=%2, RY=%3, 示教RZ=%4 deg, RZ增益=%5 deg, 爬坡RZ夹紧=[%6, %7] deg, 拐点前过渡=%8 mm, 起点跳过=%9 mm, 终点跳过=%10 mm")
            .arg(weldPosePreset.useTaughtWeldPose ? QStringLiteral("示教平台姿态") : QStringLiteral("原始固定姿态"))
            .arg(weldPosePreset.useTaughtWeldPose ? weldPosePreset.taughtWeldPoseRx : weldPosePreset.rx, 0, 'f', 3)
            .arg(weldPosePreset.useTaughtWeldPose ? weldPosePreset.taughtWeldPoseRy : weldPosePreset.ry, 0, 'f', 3)
            .arg(weldPosePreset.taughtWeldPoseRz, 0, 'f', 3)
            .arg(weldPosePreset.weldRzGainDeg, 0, 'f', 3)
            .arg(weldPosePreset.slopeRzMinDeg, 0, 'f', 3)
            .arg(weldPosePreset.slopeRzMaxDeg, 0, 'f', 3)
            .arg(weldPosePreset.cornerTransitionLeadDistance, 0, 'f', 3)
            .arg(weldPosePreset.weldStartSkipDistance, 0, 'f', 3)
            .arg(weldPosePreset.weldEndSkipDistance, 0, 'f', 3));
    }

    if (setFlowStep)
    {
        setFlowStep("特征分析完成，正在生成焊接姿态");
    }
    const RobotCalculation::LowerWeldClassificationResult& classificationForWeldPose =
        useCornerCompensatedClassification
            ? originalAnalysis.cornerCompensatedClassificationResult
            : originalAnalysis.classificationResult;
    const std::vector<QString> weldPoseLines =
        BuildSegmentPoseOutputLines(
            classificationForWeldPose,
            param,
            weldPosePreset,
            originalFitParams.platformSnapFlatSlope,
            appendLog);
    if (weldPoseLines.empty())
    {
        error = "焊接姿态生成结果为空，请检查起终点跳过距离或焊道分类结果。";
        return false;
    }
    if (!SaveTextLines(weldPosePath, weldPoseLines, error, stopRequested))
    {
        return false;
    }

    if (setFlowStep)
    {
        setFlowStep("焊接姿态已生成，正在生成焊道补偿文件");
    }
    QString seamCompSummary;
    QString generatedSeamCompSha256;
    qint64 generatedSeamCompSize = -1;
    if (!ApplyWeldSeamCompToPoseFile(
        QString::fromStdString(param.sRobotName),
        weldPosePath,
        seamCompPath,
        seamCompSummary,
        error,
        &generatedSeamCompSha256,
        &generatedSeamCompSize,
        stopRequested))
    {
        return false;
    }
    if (cancellationPoint(QStringLiteral("焊道补偿原子输出后")))
    {
        return false;
    }
    QString validatedSourcePoseSha256;
    qint64 validatedSourcePoseSize = -1;
    QString validatedPoseSha256;
    qint64 validatedPoseSize = -1;
    const bool finalArtifactValid = ValidateFinalWeldPoseArtifact(
            seamCompPath,
            weldPosePath,
            generatedSeamCompSha256,
            generatedSeamCompSize,
            weldPosePreset.robotType,
            weldPosePreset.weldStartSkipDistance,
            weldPosePreset.weldEndSkipDistance,
            pointCloudSettings,
            originalAnalysis.qualityReport,
            validatedSourcePoseSha256,
            validatedSourcePoseSize,
            validatedPoseSha256,
            validatedPoseSize,
            error,
            stopRequested);
    if (!finalArtifactValid)
    {
        if (pointCloudSettings.validationPolicy
            == PointCloudProcessingConfig::ValidationPolicy::Enforce)
        {
            return false;
        }
        if (appendLog)
        {
            appendLog(QString("Audit：最终焊接姿态结构验证未通过，仅保留未授权审计证据：%1")
                .arg(error));
        }
        error.clear();
        validatedSourcePoseSha256.clear();
        validatedSourcePoseSize = -1;
        validatedPoseSha256.clear();
        validatedPoseSize = -1;
    }
    if (!WritePointCloudQualityGate(
            laserDir,
            QString::fromStdString(param.sRobotName),
            pointCloudSettings,
            originalAnalysis.qualityReport,
            QStringList() << sourceLaserPath << workpieceCloudPath,
            weldPosePath,
            validatedSourcePoseSha256,
            validatedSourcePoseSize,
            seamCompPath,
            validatedPoseSha256,
            validatedPoseSize,
            productionContext,
            QStringLiteral("validatedRebuild"),
            error,
            stopRequested))
    {
        return false;
    }
    if (cancellationPoint(QStringLiteral("质量证明提交后")))
    {
        return false;
    }

    summary = QString("重建完成：PreservePath=%1；WeldPose=%2；SeamComp=%3；%4")
        .arg(preservePath, weldPosePath, seamCompPath, seamCompSummary);
    if (appendLog)
    {
        appendLog(QString("PreservePath文件：%1").arg(preservePath));
        appendLog(QString("焊接姿态文件：%1").arg(weldPosePath));
        appendLog(QString("焊道补偿文件：%1").arg(seamCompPath));
        appendLog(QString("焊道补偿摘要：%1").arg(seamCompSummary));
        appendLog(QString("点云质量报告：%1")
            .arg(dir.filePath(QString::fromLatin1(POINT_CLOUD_QUALITY_GATE_FILE_NAME))));
    }
    if (pointCloudSettings.validationPolicy == PointCloudProcessingConfig::ValidationPolicy::Audit)
    {
        summary = QString("审计完成：已生成点云分析与姿态产物，但没有可执行质量证明；%1")
            .arg(seamCompSummary);
        seamCompPath.clear();
        if (appendLog)
        {
            appendLog("当前为点云质量审计模式：跳过 STEP job 生成并禁止进入焊接。");
        }
        if (!qualityGateReplacement.Complete(error))
        {
            error = QStringLiteral("审计重建完成，但解除点云证明拒绝闭锁失败：") + error;
            return false;
        }
        return true;
    }
    // 焊道补偿生成后立即同步生成 STEP job(srp/srd)到焊道同目录，便于提取查看，不必等下枪执行才保存。
    {
        QString jobName, jobSrp, jobSrd, jobSum, jobErr;
        if (GenerateStepWeldProgramFiles(QString::fromStdString(param.sRobotName), seamCompPath,
                QFileInfo(seamCompPath).absolutePath(), true, 0.0, jobName, jobSrp, jobSrd, jobSum, jobErr,
                0.0, true, WeldPoseSource::PointCloudProduction, productionExpectation, true))
        {
            if (appendLog) { appendLog(QString("STEP焊接程序(job)已同步生成：srp=%1，srd=%2").arg(jobSrp, jobSrd)); }
        }
        else if (appendLog) { appendLog(QString("STEP焊接程序(job)同步生成失败(不影响焊道)：%1").arg(jobErr)); }
    }
    if (cancellationPoint(QStringLiteral("STEP 程序同步后")))
    {
        return false;
    }
    if (!qualityGateReplacement.Complete(error))
    {
        preservePath.clear();
        weldPosePath.clear();
        seamCompPath.clear();
        summary.clear();
        error = QStringLiteral("重建产物完成，但解除点云证明拒绝闭锁失败：") + error;
        return false;
    }
    return true;
}

QString MeasureThenWeldService::BuildResultDir(const std::string& robotName) const
{
    const QString dateText = QDateTime::currentDateTime().toString("yyyyMMdd");
    const QString baseDir = RobotDataHelper::BuildProjectPath(QString("Result/%1").arg(QString::fromStdString(robotName)));
    QDir dir(baseDir);
    if (!dir.exists())
    {
        dir.mkpath(".");
    }

    int flowNo = 1;
    const QStringList entries = dir.entryList(QStringList() << (dateText + "_*"), QDir::Dirs | QDir::NoDotAndDotDot, QDir::Name);
    for (const QString& entry : entries)
    {
        const QString suffix = entry.mid(dateText.length() + 1);
        bool ok = false;
        const int value = suffix.toInt(&ok);
        if (ok && value >= flowNo)
        {
            flowNo = value + 1;
        }
    }

    const QString flowDir = dir.filePath(QString("%1_%2").arg(dateText).arg(flowNo, 3, 10, QChar('0')));
    QDir().mkpath(flowDir);
    return QDir::toNativeSeparators(flowDir);
}

bool MeasureThenWeldService::SaveTextLines(
    const QString& filePath,
    const std::vector<QString>& lines,
    QString& error,
    const StopRequestedCallback& stopRequested) const
{
    const QFileInfo fileInfo(filePath);
    const QDir parentDir = fileInfo.dir();
    if (!parentDir.exists() && !QDir().mkpath(parentDir.absolutePath()))
    {
        error = QString("创建保存目录失败：%1").arg(parentDir.absolutePath());
        return false;
    }

    QSaveFile file(filePath);
    if (!file.open(QIODevice::WriteOnly))
    {
        error = QString("保存数据文件失败：%1").arg(filePath);
        return false;
    }

    // 分块批量写：逐行 QTextStream<< 对百万级行有可观的编码/格式化开销；改为攒到约 4MB 的
    // QByteArray 再一次 write，写 185MB 级完整点云明显更快，内存峰值也受控（不一次性翻倍）。
    constexpr int kFlushThreshold = 4 * 1024 * 1024;
    QByteArray buffer;
    buffer.reserve(kFlushThreshold + 256);
    size_t lineIndex = 0;
    for (const QString& line : lines)
    {
        if ((lineIndex++ & 0xffU) == 0U && stopRequested && stopRequested())
        {
            error = QString("已取消写入：%1").arg(filePath);
            file.cancelWriting();
            return false;
        }
        buffer += line.toUtf8();
        buffer += '\n';
        if (buffer.size() >= kFlushThreshold)
        {
            if (file.write(buffer) != buffer.size())
            {
                error = QString("写入数据文件失败：%1").arg(filePath);
                file.cancelWriting();
                return false;
            }
            buffer.clear();
        }
    }
    if (stopRequested && stopRequested())
    {
        error = QString("已取消写入：%1").arg(filePath);
        file.cancelWriting();
        return false;
    }
    if (!buffer.isEmpty() && file.write(buffer) != buffer.size())
    {
        error = QString("写入数据文件失败：%1").arg(filePath);
        file.cancelWriting();
        return false;
    }
    if (!file.commit())
    {
        error = QString("原子提交数据文件失败：%1").arg(filePath);
        return false;
    }
    return true;
}

bool MeasureThenWeldService::ApplyWeldSeamCompToPoseFile(
    const QString& robotName,
    const QString& inputPath,
    const QString& outputPath,
    QString& summary,
    QString& error,
    QString* generatedSha256,
    qint64* generatedSize,
    const StopRequestedCallback& stopRequested) const
{
    summary.clear();
    error.clear();
    if (generatedSha256 != nullptr)
    {
        generatedSha256->clear();
    }
    if (generatedSize != nullptr)
    {
        *generatedSize = -1;
    }

    QVector<WeldPoseFileRecord> records;
    if (!LoadWeldPoseFileRecords(
            inputPath, records, error, nullptr, nullptr, stopRequested))
    {
        return false;
    }

    T_PRECISE_MEASURE_PARAM param = BuildMeasureWeldParamShell(robotName);
    const WeldPosePreset preset = LoadWeldPosePreset(param);

    WeldSeamCompApplyStats compStats = ApplyWeldSeamCompToWeldPoseRecords(preset, records);
    const QVector<WeldPoseFileRecord> recordsBeforeTrim = records;
    const SeamCompFinalizeStats finalizeStats =
        FinalizeSeamCompedWeldPoseRecords(preset, recordsBeforeTrim, records, compStats);
    if (finalizeStats.emptyAfterEndpointTrim)
    {
        error = "焊道补偿和起终点裁剪后没有有效焊接点。";
        return false;
    }
    if (finalizeStats.emptyAfterSelfIntersection)
    {
        error = "焊道自交裁剪后没有有效焊接点。";
        return false;
    }

    QStringList outputLines;
    outputLines.reserve(records.size() + 1);
    outputLines << "weld_index raw_index x y z rx ry rz bx by bz point_type segment_kind is_lap_step";
    for (const WeldPoseFileRecord& record : records)
    {
        outputLines << BuildWeldPoseFileRecordLine(record);
    }

    QByteArray expectedPayload;
#ifdef Q_OS_WIN
    constexpr auto outputLineEnding = "\r\n";
#else
    constexpr auto outputLineEnding = "\n";
#endif
    for (const QString& line : outputLines)
    {
        expectedPayload.append(line.toUtf8());
        expectedPayload.append(outputLineEnding);
    }

    if (!RobotDataHelper::SaveTextFileLines(outputPath, outputLines, &error))
    {
        return false;
    }
    QFile generatedFile(outputPath);
    if (!generatedFile.open(QIODevice::ReadOnly))
    {
        error = QStringLiteral("焊道补偿文件写后快照回读失败：") + outputPath;
        return false;
    }
    const QByteArray generatedPayload = generatedFile.readAll();
    if (generatedFile.error() != QFileDevice::NoError || generatedPayload != expectedPayload)
    {
        error = QStringLiteral("焊道补偿文件写后字节与内存生成结果不一致，拒绝继续：") + outputPath;
        return false;
    }
    if (generatedSha256 != nullptr)
    {
        *generatedSha256 = QString::fromLatin1(
            QCryptographicHash::hash(generatedPayload, QCryptographicHash::Sha256).toHex()).toLower();
    }
    if (generatedSize != nullptr)
    {
        *generatedSize = generatedPayload.size();
    }

    const QString segmentKindDebugPath = QFileInfo(outputPath)
        .dir()
        .filePath(WELD_SEGMENT_KIND_DEBUG_FILE_NAME);
    if (!SaveTextLines(segmentKindDebugPath, BuildWeldSegmentKindDebugLines(records), error))
    {
        return false;
    }

    QString arcPreviewWarning;
    if (PointCloudProcessingConfig::Load().exportFitDebugCloud)
    {
        const QString arcPreviewPath = QFileInfo(outputPath)
            .dir()
            .filePath(QStringLiteral("PreciseLaserPoint_ArcTransitionPreview.txt"));
        QString arcPreviewError;
        if (!SaveTextLines(
                arcPreviewPath,
                BuildArcTransitionPreviewCloudLines(records),
                arcPreviewError))
        {
            arcPreviewWarning = QStringLiteral("；最终圆弧调试预览写入失败=")
                + arcPreviewError;
        }
    }

    summary = QString("焊道补偿完成：点数=%1，作用范围=%2，Z补偿点数=%3，枪反向补偿点数=%4，焊道方向补偿点数=%5，起点裁剪=%6点，终点裁剪=%7点，自交裁剪=%8次，删除回折点=%9，交点校正拐点=%10，丢失拐点=%11，交点恢复=%12，跳过恢复=%13，未过渡拐点补偿=%14，补齐点=%15，过渡后补点=%16，最终补点=%17，最大步长=%18mm，圆弧过渡=%19处，半径=%20mm，新增点=%21，段属性调试=%22，配置=%23")
        .arg(records.size())
        .arg(QStringLiteral("整条焊道统一"))
        .arg(compStats.zAdjustedCount)
        .arg(compStats.gunDirAdjustedCount)
        .arg(compStats.seamDirAdjustedCount)
        .arg(finalizeStats.endpointTrim.removedStartCount)
        .arg(finalizeStats.endpointTrim.removedEndCount)
        .arg(compStats.selfIntersectionTrimCount)
        .arg(compStats.selfIntersectionRemovedPointCount)
        .arg(finalizeStats.cornerRestore.adjustedCornerCount)
        .arg(finalizeStats.cornerRestore.missingCornerCount)
        .arg(finalizeStats.cornerRestore.restoredCornerCount)
        .arg(finalizeStats.cornerRestore.skippedCornerCount)
        .arg(finalizeStats.smoothedRemainingCornerCount)
        .arg(finalizeStats.densifiedPointCount)
        .arg(finalizeStats.postArcDensifiedPointCount)
        .arg(finalizeStats.finalDensifiedPointCount)
        .arg(finalizeStats.densifyStepMm, 0, 'f', 3)
        .arg(finalizeStats.arc.roundedCornerCount)
        .arg(finalizeStats.arc.radiusMm, 0, 'f', 3)
        .arg(finalizeStats.arc.insertedPointCount())
        .arg(QDir::toNativeSeparators(segmentKindDebugPath))
        .arg(QDir::toNativeSeparators(preset.seamCompFilePath));
    summary += QString("；圆弧候选=%1，未生成=%2，缩径=%3，最小实际半径=%4mm")
        .arg(finalizeStats.arc.candidateCornerCount)
        .arg(finalizeStats.arc.skippedCornerCount())
        .arg(finalizeStats.arc.reducedRadiusCornerCount)
        .arg(finalizeStats.arc.minimumAppliedRadiusMm, 0, 'f', 3);
    summary += arcPreviewWarning;
    if (!preset.seamCompLoadError.isEmpty())
    {
        summary += QStringLiteral("；配置读取告警=") + preset.seamCompLoadError;
    }
    if (!preset.seamCompWarnings.isEmpty())
    {
        summary += QStringLiteral("；配置兼容告警=") + preset.seamCompWarnings.join(QStringLiteral(" | "));
    }
    return true;
}

bool MeasureThenWeldService::GenerateStepWeldProgramFiles(
    const QString& robotName,
    const QString& poseFilePath,
    const QString& outputDir,
    bool actualWeld,
    double weldSpeedMmPerMin,
    QString& programName,
    QString& srpPath,
    QString& srdPath,
    QString& summary,
    QString& error,
    double overrideFinalStepMm,
    bool allowPointwiseWeave,
    WeldPoseSource poseSource,
    const PointCloudProductionExpectation& authorizationExpectation,
    bool allowActiveProofReplacement) const
{
    programName.clear();
    srpPath.clear();
    srdPath.clear();
    summary.clear();
    error.clear();

    QFileInfo poseInfo(QDir::fromNativeSeparators(poseFilePath.trimmed()));
    if (!poseInfo.isAbsolute())
    {
        poseInfo = QFileInfo(AppPaths::CommandLinePath(poseInfo.filePath()));
    }
    if (!poseInfo.exists() || !poseInfo.isFile())
    {
        error = QString("焊接姿态文件不存在：%1")
            .arg(QDir::toNativeSeparators(poseInfo.absoluteFilePath()));
        return false;
    }
    QVector<WeldPoseFileRecord> records;
    QString loadedPoseSha256;
    qint64 loadedPoseSize = -1;
    if (!LoadWeldPoseFileRecords(
            poseInfo.absoluteFilePath(), records, error, &loadedPoseSha256, &loadedPoseSize))
    {
        return false;
    }
    if (!VerifyWeldPoseAuthorization(
            poseSource,
            poseInfo.absoluteFilePath(),
            robotName,
            loadedPoseSha256,
            loadedPoseSize,
            error,
            nullptr,
            &authorizationExpectation,
            allowActiveProofReplacement))
    {
        return false;
    }

    T_PRECISE_MEASURE_PARAM param = BuildMeasureWeldParamShell(robotName);
    WeldPosePreset preset = LoadWeldPosePreset(param);
    if (actualWeld && (!preset.weldProcessLoaded || !preset.weldProcessSafetyError.isEmpty()))
    {
        error = !preset.weldProcessSafetyError.isEmpty()
            ? preset.weldProcessSafetyError
            : (preset.weldProcessLoadError.isEmpty()
                ? QStringLiteral("当前实际焊接工艺未能加载，禁止生成实际焊接程序。")
                : preset.weldProcessLoadError);
        return false;
    }
    if (!allowPointwiseWeave && preset.weaveEnabled && preset.weaveAppPointwise)
    {
        error = QStringLiteral("pointwise 自定义摆动已被禁用(allowPointwiseWeave=false)，无法生成轨迹");
        return false;
    }
    if (std::isfinite(overrideFinalStepMm) && overrideFinalStepMm > 0.0)
    {
        // 虚拟焊道测试：用户指定的点间距覆盖工艺/测量参数，作为机器人最终逐点执行间距。
        const double normalizedStep = NormalizeFinalWeldTrajectorySampleStepMm(overrideFinalStepMm);
        param.dFinalWeldTrajectoryStepMm = normalizedStep;
        preset.finalWeldStepFromProcessMm = normalizedStep;
    }
    ApplyWeldDirectionToExecutionRecords(preset, records);
    const double effectiveWeldSpeedMmPerMin =
        (std::isfinite(weldSpeedMmPerMin) && weldSpeedMmPerMin > 0.0)
        ? weldSpeedMmPerMin
        : ((std::isfinite(preset.weldProcessSpeedMmPerMin) && preset.weldProcessSpeedMmPerMin > 0.0)
            ? preset.weldProcessSpeedMmPerMin
            : FANUC_WELD_PATH_SPEED_MM_PER_MIN);
    const double transitionCommandSpeed =
        (preset.transitionSpeedEnabled && preset.transitionSpeedMmPerMin > 0.0)
        ? preset.transitionSpeedMmPerMin
        : 0.0;

    std::vector<T_ROBOT_MOVE_INFO> moveInfos;
    QVector<WeldPoseFileRecord> sampledRecords;
    if (!BuildWeldPoseMoveInfos(
        records,
        effectiveWeldSpeedMmPerMin,
        moveInfos,
        error,
        &preset,
        param.dFinalWeldTrajectoryStepMm,
        transitionCommandSpeed,
        true,
        &sampledRecords))
    {
        return false;
    }
    const QString sampledPosePath = BuildFinalSampledWeldPosePath(poseInfo.absoluteFilePath());
    QString sampledSaveError;
    if (!SaveWeldPoseFileRecords(sampledPosePath, sampledRecords, sampledSaveError))
    {
        error = QStringLiteral("保存可验证的 STEP 最终抽样轨迹失败，禁止生成可执行程序：")
            + sampledSaveError;
        return false;
    }

    QString resolvedOutputDir = outputDir.trimmed();
    if (resolvedOutputDir.isEmpty())
    {
        resolvedOutputDir = RobotDataHelper::BuildProjectPath("Job/STEP");
    }
    else
    {
        QFileInfo outputInfo(QDir::fromNativeSeparators(resolvedOutputDir));
        resolvedOutputDir = outputInfo.isAbsolute()
            ? outputInfo.absoluteFilePath()
            : QFileInfo(AppPaths::CommandLinePath(outputInfo.filePath())).absoluteFilePath();
    }

    const std::string generatedProgramName = STEPRobotCtrl::MakeTimestampWeldProgramName();
    T_AXISUNIT axisUnit;
    std::string localSrpPath;
    std::string localSrdPath;
    std::string writeError;
    if (!VerifyWeldPoseAuthorization(
            poseSource,
            poseInfo.absoluteFilePath(),
            robotName,
            loadedPoseSha256,
            loadedPoseSize,
            error,
            nullptr,
            &authorizationExpectation,
            allowActiveProofReplacement))
    {
        return false;
    }
    if (!STEPRobotCtrl::WriteContiMoveAnyFiles(
        moveInfos,
        QDir::toNativeSeparators(resolvedOutputDir).toLocal8Bit().toStdString(),
        generatedProgramName,
        axisUnit,
        &localSrpPath,
        &localSrdPath,
        &writeError,
        actualWeld))
    {
        error = QString("生成STEP焊接程序失败：%1").arg(QString::fromStdString(writeError));
        return false;
    }

    programName = QString::fromStdString(generatedProgramName);
    srpPath = QDir::toNativeSeparators(QString::fromLocal8Bit(
        localSrpPath.data(), static_cast<int>(localSrpPath.size())));
    srdPath = QDir::toNativeSeparators(QString::fromLocal8Bit(
        localSrdPath.data(), static_cast<int>(localSrdPath.size())));
    summary = QString("STEP焊接程序生成完成：程序=%1，模式=%2，方向=%3，点数=%4，轨迹速度=%5 mm/min，OVERLAPREL=%6，SRP=%7，SRD=%8")
        .arg(programName)
        .arg(actualWeld ? QStringLiteral("实际焊接") : QStringLiteral("空跑"))
        .arg(WeldDirectionText(preset))
        .arg(static_cast<int>(moveInfos.size()))
        .arg(effectiveWeldSpeedMmPerMin, 0, 'f', 3)
        .arg(preset.stepOverlapRel, 0, 'f', 3)
        .arg(srpPath)
        .arg(srdPath);
    summary += QString("；最终轨迹点间距=%1 mm")
        .arg(param.dFinalWeldTrajectoryStepMm, 0, 'f', 3);
    summary += QString("；抽样轨迹文件=%1").arg(sampledPosePath);
    if (!preset.weldProcessLoaded)
    {
        summary += "；未读取到当前焊接工艺参数，本次文件不包含起弧/停弧工艺语句";
    }
    else
    {
        summary += QString("；工艺=%1A/%2V -> %3A/%4V -> %5A/%6V")
            .arg(preset.startArcCurrent, 0, 'f', 3)
            .arg(preset.startArcVoltage, 0, 'f', 3)
            .arg(preset.weldCurrent, 0, 'f', 3)
            .arg(preset.weldVoltage, 0, 'f', 3)
            .arg(preset.stopArcCurrent, 0, 'f', 3)
            .arg(preset.stopArcVoltage, 0, 'f', 3);
        summary += QString("；摆动=%1，跟踪=%2")
            .arg(preset.weaveEnabled ? QStringLiteral("启用") : QStringLiteral("NULL"))
            .arg(preset.trackEnabled ? QStringLiteral("启用") : QStringLiteral("NULL"));
        summary += QString("；焊接模式=%1").arg(preset.arcMode);
        if (preset.transitionSpeedEnabled)
        {
            summary += QString("；过渡速度=%1 mm/min")
                .arg(preset.transitionSpeedMmPerMin, 0, 'f', 3);
        }
        if (preset.transitionCurrentVoltageEnabled)
        {
            summary += QString("；过渡电流电压=%1A/%2V")
                .arg(preset.transitionCurrent, 0, 'f', 3)
                .arg(preset.transitionVoltage, 0, 'f', 3);
        }
        if (preset.transitionSpeedEnabled || preset.transitionCurrentVoltageEnabled)
        {
            summary += QString("；拐点过渡作用范围=%1")
                .arg(TransitionApplyScopeText(preset.transitionApplyScope));
        }
        if (!actualWeld)
        {
            summary += "；空跑模式不生成ARCON/ARCSET/ARCOFF";
        }
    }
    return true;
}

bool MeasureThenWeldService::GenerateVirtualStraightWeldFiles(
    const QString& robotName,
    const T_ROBOT_COORS& startCoors,
    double lengthMm,
    double pointStepMm,
    int directionSign,
    bool actualWeld,
    QString& outputDir,
    QString& weldPosePath,
    QString& srpPath,
    QString& srdPath,
    QString& programName,
    QString& summary,
    QString& error,
    const LogCallback& appendLog) const
{
    weldPosePath.clear();
    srpPath.clear();
    srdPath.clear();
    programName.clear();
    summary.clear();
    error.clear();

    const QString normalizedRobotName = robotName.trimmed().isEmpty()
        ? QStringLiteral("RobotA")
        : robotName.trimmed();

    const double absLength = std::abs(lengthMm);
    if (!std::isfinite(absLength) || absLength < 1.0)
    {
        error = "虚拟焊道长度无效（需 >= 1mm）。";
        return false;
    }
    const double normalizedStep = NormalizeFinalWeldTrajectorySampleStepMm(pointStepMm);
    const int sign = (directionSign < 0) ? -1 : 1;

    // 干净直线：保持机器人当前焊枪姿态，沿 ±Y 等距造点；不走点云拟合/姿态补偿/焊缝补偿/起终裁剪/拐点处理。
    // 内部按细间距(<=2mm)造稠密点，最终由 GenerateStepWeldProgramFiles 按用户点间距抽样成机器人逐点执行轨迹。
    const double internalStep = std::max(0.5, std::min(2.0, normalizedStep));
    const int segCount = std::max(1, static_cast<int>(std::ceil(absLength / internalStep)));

    QVector<WeldPoseFileRecord> records;
    records.reserve(segCount + 1);
    for (int i = 0; i <= segCount; ++i)
    {
        const double t = static_cast<double>(i) / static_cast<double>(segCount);
        WeldPoseFileRecord r;
        r.weldIndex = i + 1;
        r.rawIndex = i;
        r.point = Eigen::Vector3d(
            startCoors.dX,
            startCoors.dY + sign * absLength * t,
            startCoors.dZ);
        r.rx = startCoors.dRX;
        r.ry = startCoors.dRY;
        r.rz = startCoors.dRZ;
        r.bx = startCoors.dBX;
        r.by = startCoors.dBY;
        r.bz = startCoors.dBZ;
        r.pointType = (i == 0)
            ? QStringLiteral("start")
            : ((i == segCount) ? QStringLiteral("end") : QStringLiteral("normal"));
        r.segmentKind = QStringLiteral("low_platform");  // 单段直线占位段类（不参与补偿，仅满足文件格式）
        r.isLapStep = false;
        records.push_back(r);
    }

    // 解析输出目录（默认 Result/<robot>/VirtualWeld_<时间>）。
    QString resolvedDir = outputDir.trimmed();
    if (resolvedDir.isEmpty())
    {
        const QString stamp = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
        resolvedDir = RobotDataHelper::BuildProjectPath(
            QString("Result/%1/VirtualWeld_%2").arg(normalizedRobotName, stamp));
    }
    {
        QDir dir(resolvedDir);
        if (!dir.exists() && !dir.mkpath("."))
        {
            error = QString("无法创建输出目录：%1").arg(QDir::toNativeSeparators(resolvedDir));
            return false;
        }
    }
    outputDir = QDir::toNativeSeparators(resolvedDir);
    weldPosePath = QDir(resolvedDir).filePath(WELD_POSE_FILE_NAME);

    // 写干净直线 pose 文件（带与正式文件一致的表头），既供查看也作下发执行文件。
    std::vector<QString> lines;
    lines.reserve(records.size() + 1);
    lines.push_back("weld_index raw_index x y z rx ry rz bx by bz point_type segment_kind is_lap_step");
    for (const WeldPoseFileRecord& r : records)
    {
        lines.push_back(BuildWeldPoseFileRecordLine(r));
    }
    if (!SaveTextLines(weldPosePath, lines, error))
    {
        return false;
    }

    QVector<WeldPoseFileRecord> loadedRecords;
    QString loadedPoseSha256;
    qint64 loadedPoseSize = -1;
    if (!LoadWeldPoseFileRecords(
            weldPosePath, loadedRecords, error, &loadedPoseSha256, &loadedPoseSize))
    {
        return false;
    }
    constexpr double kSyntheticGeometryTolerance = 1e-4;
    if (loadedRecords.size() != records.size())
    {
        error = QStringLiteral("虚拟焊道写后回读点数不一致。");
        return false;
    }
    for (int index = 0; index < loadedRecords.size(); ++index)
    {
        const WeldPoseFileRecord& point = loadedRecords[index];
        const double expectedY = startCoors.dY
            + sign * absLength * static_cast<double>(index)
                / static_cast<double>(loadedRecords.size() - 1);
        const bool endpointTypeOk = index == 0
            ? point.pointType == QStringLiteral("start")
            : (index + 1 == loadedRecords.size()
                ? point.pointType == QStringLiteral("end")
                : point.pointType == QStringLiteral("normal"));
        if (point.weldIndex != index + 1
            || point.rawIndex != index
            || !endpointTypeOk
            || point.isLapStep
            || std::abs(point.point.x() - startCoors.dX) > kSyntheticGeometryTolerance
            || std::abs(point.point.y() - expectedY) > kSyntheticGeometryTolerance
            || std::abs(point.point.z() - startCoors.dZ) > kSyntheticGeometryTolerance
            || std::abs(point.rx - startCoors.dRX) > kSyntheticGeometryTolerance
            || std::abs(point.ry - startCoors.dRY) > kSyntheticGeometryTolerance
            || std::abs(point.rz - startCoors.dRZ) > kSyntheticGeometryTolerance
            || std::abs(point.bx - startCoors.dBX) > kSyntheticGeometryTolerance
            || std::abs(point.by - startCoors.dBY) > kSyntheticGeometryTolerance
            || std::abs(point.bz - startCoors.dBZ) > kSyntheticGeometryTolerance)
        {
            error = QString("虚拟焊道写后几何身份验证失败：第 %1 点不是固定姿态的单段 %2Y 直线。")
                .arg(index + 1)
                .arg(sign > 0 ? "+" : "-");
            return false;
        }
    }
    if (!RegisterSyntheticPoseAuthorization(
            weldPosePath,
            normalizedRobotName,
            loadedPoseSha256,
            loadedPoseSize,
            error))
    {
        return false;
    }

    if (appendLog)
    {
        appendLog(QString("虚拟焊道(干净直线)：起点=%1，方向=%2Y，长度=%3 mm，点间距=%4 mm，造点=%5（保持当前焊枪姿态，不叠加补偿）")
            .arg(RobotCoorsText(startCoors))
            .arg(sign > 0 ? "+" : "-")
            .arg(absLength, 0, 'f', 3)
            .arg(normalizedStep, 0, 'f', 3)
            .arg(records.size()));
    }

    // 直接生成 STEP 焊接程序 srp/srd：actualWeld=true 含 ARC/WEAVEDATA 摆动；摆动/速度/姿态/位形读保存工艺；
    // 点间距用用户值覆盖；方向由 GenerateStepWeldProgramFiles 内部默认 weldDirection=1 保证前向，与下发执行一致。
    QString jobSummary;
    if (!GenerateStepWeldProgramFiles(normalizedRobotName, weldPosePath, resolvedDir, actualWeld, 0.0,
            programName, srpPath, srdPath, jobSummary, error, normalizedStep,
            /*allowPointwiseWeave=*/true, WeldPoseSource::SyntheticVirtualTest))
    {
        RevokeSyntheticPoseAuthorization(weldPosePath);
        return false;
    }

    summary = QString("虚拟焊道已生成(干净直线，不含补偿)：长度=%1 mm，点间距=%2 mm，模式=%3；执行/查看文件=%4；%5")
        .arg(absLength, 0, 'f', 3)
        .arg(normalizedStep, 0, 'f', 3)
        .arg(actualWeld ? QStringLiteral("实焊(含摆动)") : QStringLiteral("空跑"))
        .arg(QDir::toNativeSeparators(weldPosePath))
        .arg(jobSummary);
    return true;
}

bool MeasureThenWeldService::DownlinkWeldPoseFile(
    RobotDriverAdaptor* pRobotDriver,
    const QString& poseFilePath,
    double linearSpeedConfigMmPerMin,
    QString& summary,
    QString& error,
    WeldPoseSource poseSource) const
{
    summary.clear();
    error.clear();
    const double selectedSpeedMmPerMin =
        (std::isfinite(linearSpeedConfigMmPerMin) && linearSpeedConfigMmPerMin > 0.0)
        ? linearSpeedConfigMmPerMin
        : FANUC_WELD_PATH_SPEED_MM_PER_MIN;

    if (pRobotDriver == nullptr)
    {
        error = "机器人驱动为空。";
        return false;
    }
    const QString robotName = QString::fromStdString(pRobotDriver->m_sRobotName);
    QVector<WeldPoseFileRecord> records;
    QString loadedPoseSha256;
    qint64 loadedPoseSize = -1;
    if (!LoadWeldPoseFileRecords(
            poseFilePath, records, error, &loadedPoseSha256, &loadedPoseSize))
    {
        return false;
    }
    if (!VerifyWeldPoseAuthorization(
            poseSource,
            poseFilePath,
            robotName,
            loadedPoseSha256,
            loadedPoseSize,
            error,
            pRobotDriver))
    {
        return false;
    }

    const double linearCommandSpeed =
        LinearCommandSpeedForRobot(pRobotDriver, selectedSpeedMmPerMin, 1.0);
    const QString linearCommandSpeedUnit = LinearCommandSpeedUnitText(pRobotDriver);
    const T_PRECISE_MEASURE_PARAM param = BuildMeasureWeldParamShell(QString::fromStdString(pRobotDriver->m_sRobotName));
    const WeldPosePreset preset = LoadWeldPosePreset(param);
    ApplyWeldDirectionToExecutionRecords(preset, records);
    std::vector<T_ROBOT_MOVE_INFO> moveInfos;
    QVector<WeldPoseFileRecord> sampledRecords;
    if (!BuildWeldPoseMoveInfos(records, linearCommandSpeed, moveInfos, error, &preset, param.dFinalWeldTrajectoryStepMm, 0.0, false, &sampledRecords))
    {
        return false;
    }
    const QString sampledPosePath = BuildFinalSampledWeldPosePath(poseFilePath);
    QString sampledSaveError;
    if (!SaveWeldPoseFileRecords(sampledPosePath, sampledRecords, sampledSaveError))
    {
        error = QStringLiteral("保存可验证的下发轨迹失败，禁止改变机器人外部状态：")
            + sampledSaveError;
        return false;
    }

    if (FANUCRobotCtrl* pFanucDriver = dynamic_cast<FANUCRobotCtrl*>(pRobotDriver))
    {
        std::string programName;
        std::string localLsPath;
        std::string remoteTpPath;
        if (!VerifyWeldPoseAuthorization(
                poseSource,
                poseFilePath,
                robotName,
                loadedPoseSha256,
                loadedPoseSize,
                error,
                pRobotDriver))
        {
            return false;
        }
        const int downlinkRet = pFanucDriver->UploadMultiPointTpProgram(
            moveInfos,
            FANUCRobotCtrl::TrajectoryProgramMode::DryRun,
            &programName,
            &localLsPath,
            &remoteTpPath);
        if (downlinkRet != 0)
        {
            error = QString("下发焊接轨迹失败：ret=%1，姿态文件=%2")
                .arg(downlinkRet)
                .arg(QDir::toNativeSeparators(QFileInfo(poseFilePath).absoluteFilePath()));
            return false;
        }

        summary = QString("点数=%1，最终轨迹点间距=%2 mm，轨迹速度=%3 mm/min (下发=%4 %5)，程序=%6，本地LS=%7，远程TP=%8，抽样轨迹文件=%9，当前仅下发未自动执行")
            .arg(static_cast<int>(moveInfos.size()))
            .arg(param.dFinalWeldTrajectoryStepMm, 0, 'f', 3)
            .arg(selectedSpeedMmPerMin, 0, 'f', 3)
            .arg(linearCommandSpeed, 0, 'f', 3)
            .arg(linearCommandSpeedUnit)
            .arg(QString::fromStdString(programName))
            .arg(QDir::toNativeSeparators(QString::fromStdString(localLsPath)))
            .arg(QString::fromStdString(remoteTpPath))
            .arg(sampledPosePath);
        return true;
    }

    STEPRobotCtrl* pStepDriver = dynamic_cast<STEPRobotCtrl*>(pRobotDriver);
    if (pStepDriver == nullptr)
    {
        error = QStringLiteral("下发焊接轨迹失败：当前机器人驱动不是 FANUC 或 STEP。");
        return false;
    }

    // Downlink 的语义与上面 FANUC 分支保持一致：只生成和上传，
    // 不加载、不启动机器人程序。不能调用 ContiMoveAnyWithProgramName，
    // 因为该 API 会立即启动运动，导致本函数在未等待终态时就返回。
    const std::string stepProgramName = STEPRobotCtrl::MakeTimestampWeldProgramName();
    const std::string localStepDir = QDir::toNativeSeparators(
        AppPaths::WritablePath(QStringLiteral("Job/STEP"))).toLocal8Bit().toStdString();
    std::string localProgramFile;
    std::string localDataFile;
    std::string generateError;
    if (!VerifyWeldPoseAuthorization(
            poseSource,
            poseFilePath,
            robotName,
            loadedPoseSha256,
            loadedPoseSize,
            error,
            pRobotDriver))
    {
        return false;
    }
    if (!STEPRobotCtrl::WriteContiMoveAnyFiles(
        moveInfos,
        localStepDir,
        stepProgramName,
        pStepDriver->m_tAxisUnit,
        &localProgramFile,
        &localDataFile,
        &generateError,
        true))
    {
        error = QString("生成 STEP 焊接轨迹文件失败：%1，姿态文件=%2")
            .arg(DecodeRobotMessageText(generateError))
            .arg(QDir::toNativeSeparators(QFileInfo(poseFilePath).absoluteFilePath()));
        return false;
    }

    const std::string remoteBaseDir = "/UserPrograms/PCRobot.sr/";
    const std::string remoteProgramFile = remoteBaseDir + stepProgramName + ".srp";
    const std::string remoteDataFile = remoteBaseDir + stepProgramName + ".srd";
    if (!VerifyWeldPoseAuthorization(
            poseSource,
            poseFilePath,
            robotName,
            loadedPoseSha256,
            loadedPoseSize,
            error,
            pRobotDriver))
    {
        return false;
    }
    if (pStepDriver->UploadFile(localProgramFile, remoteProgramFile) != 0
        || pStepDriver->UploadFile(localDataFile, remoteDataFile) != 0)
    {
        error = QString("STEP焊接轨迹上传失败：本地SRP=%1，本地SRD=%2，远程目录=%3")
            .arg(QDir::toNativeSeparators(QString::fromLocal8Bit(
                localProgramFile.data(), static_cast<int>(localProgramFile.size()))))
            .arg(QDir::toNativeSeparators(QString::fromLocal8Bit(
                localDataFile.data(), static_cast<int>(localDataFile.size()))))
            .arg(QString::fromStdString(remoteBaseDir));
        return false;
    }

    summary = QString("点数=%1，最终轨迹点间距=%2 mm，方向=%3，轨迹速度=%4 mm/min (下发=%5 %6)，抽样轨迹文件=%7，程序=%8，本地SRP=%9，本地SRD=%10，远程目录=%11，当前仅下发未自动执行")
        .arg(static_cast<int>(moveInfos.size()))
        .arg(param.dFinalWeldTrajectoryStepMm, 0, 'f', 3)
        .arg(WeldDirectionText(preset))
        .arg(selectedSpeedMmPerMin, 0, 'f', 3)
        .arg(linearCommandSpeed, 0, 'f', 3)
        .arg(linearCommandSpeedUnit)
        .arg(sampledPosePath)
        .arg(QString::fromStdString(stepProgramName))
        .arg(QDir::toNativeSeparators(QString::fromLocal8Bit(
            localProgramFile.data(), static_cast<int>(localProgramFile.size()))))
        .arg(QDir::toNativeSeparators(QString::fromLocal8Bit(
            localDataFile.data(), static_cast<int>(localDataFile.size()))))
        .arg(QString::fromStdString(remoteBaseDir));
    return true;
}

bool MeasureThenWeldService::ExecuteWeldPoseFileWithSafePos(
    RobotDriverAdaptor* pRobotDriver,
    const QString& poseFilePath,
    const T_PRECISE_MEASURE_PARAM& param,
    QString& summary,
    QString& error,
    T_ROBOT_COORS* pStartSafeCoors,
    T_ROBOT_COORS* pEndSafeCoors,
    const LogCallback& appendLog,
    const StepCallback& setFlowStep,
    const CheckpointCallback& checkpoint,
    double overrideFinalStepMm,
    bool allowPointwiseWeave,
    WeldPoseSource poseSource,
    double resumeStartArcMm,
    bool inputAlreadyInExecutionOrder,
    const StopRequestedCallback& stopRequested,
    const WeldExecutionPreparedCallback& executionPrepared,
    const WeldExecutionFinishedCallback& executionFinished,
    const QString& expectedSourceSha256,
    const WeldExecutionPreMotionCallback& executionPreMotion,
    const QString& qualityProofSourcePosePath) const
{
    summary.clear();
    error.clear();

    if (pRobotDriver == nullptr)
    {
        error = "机器人驱动为空。";
        return false;
    }
    // 本入口始终会下发真实机器人运动；SyntheticVirtualTest 只放宽点云来源，绝不表示离线。
    // 没有持久准备/终态回调的 CLI、虚拟焊道或未来调用点必须在第一条运动前 fail-closed。
    if (!executionPrepared || !executionFinished)
    {
        error = QStringLiteral(
            "生产焊接缺少START前持久门禁或安全终态回调，已在任何机器人运动前拒绝执行。");
        return false;
    }
    const QString qualityProofRobotName = QString::fromStdString(param.sRobotName).trimmed().isEmpty()
        ? QString::fromStdString(pRobotDriver->m_sRobotName)
        : QString::fromStdString(param.sRobotName);
    if (!std::isfinite(overrideFinalStepMm) || overrideFinalStepMm < 0.0)
    {
        error = QStringLiteral("最终轨迹点距覆盖值必须为 0 或有限正数。");
        return false;
    }

    QVector<WeldPoseFileRecord> records;
    QString loadedPoseSha256;
    qint64 loadedPoseSize = -1;
    if (!LoadWeldPoseFileRecords(
            poseFilePath, records, error, &loadedPoseSha256, &loadedPoseSize))
    {
        return false;
    }
    if (!std::isfinite(resumeStartArcMm)
        || (resumeStartArcMm < 0.0 && resumeStartArcMm != -1.0))
    {
        error = QStringLiteral("断点续焊起始弧长必须为 -1（普通执行）或有限非负毫米值。");
        return false;
    }
    const bool resumeMode = resumeStartArcMm >= 0.0;
    if (resumeMode != inputAlreadyInExecutionOrder
        || (resumeMode && !executionPrepared))
    {
        error = QStringLiteral(
            "断点续焊必须同时提供非负弧长、已按执行顺序绑定的FinalSampled输入和START前身份回调。");
        return false;
    }
    if (resumeMode)
    {
        if (!IsSha256Text(expectedSourceSha256)
            || loadedPoseSha256.compare(expectedSourceSha256, Qt::CaseInsensitive) != 0)
        {
            error = QStringLiteral("V2续焊实际解析轨迹与绑定的预期 SHA256 不一致。");
            return false;
        }
        if (!QFileInfo(poseFilePath).fileName().endsWith(
                QStringLiteral("_FinalSampled.txt"), Qt::CaseInsensitive))
        {
            error = QStringLiteral("V2续焊只允许执行绑定案例目录中的 FinalSampled 轨迹。");
            return false;
        }
    }
    else if (!expectedSourceSha256.isEmpty())
    {
        error = QStringLiteral("普通完整焊接不应携带V2续焊源轨迹SHA256。");
        return false;
    }

    const QString absolutePosePath = QFileInfo(QDir::fromNativeSeparators(poseFilePath)).absoluteFilePath();
    QString qualityProofPosePath = absolutePosePath;
    QString qualityProofPoseSha256 = loadedPoseSha256;
    qint64 qualityProofPoseSize = loadedPoseSize;
    if (poseSource == WeldPoseSource::SyntheticVirtualTest)
    {
        if (resumeMode || !qualityProofSourcePosePath.trimmed().isEmpty())
        {
            error = QStringLiteral("虚拟焊道进程内授权不允许替代生产证明或进入断点续焊。");
            return false;
        }
    }
    else if (resumeMode)
    {
        if (qualityProofSourcePosePath.trimmed().isEmpty())
        {
            error = QStringLiteral("V2续焊缺少最初 Enforce 授权的 SeamComp 证明源。");
            return false;
        }
        qualityProofPosePath = QFileInfo(
            QDir::fromNativeSeparators(qualityProofSourcePosePath)).absoluteFilePath();
        const QFileInfo executionInfo(absolutePosePath);
        const QFileInfo proofInfo(qualityProofPosePath);
        if (executionInfo.dir().absolutePath().compare(
                proofInfo.dir().absolutePath(), Qt::CaseInsensitive) != 0)
        {
            error = QStringLiteral("V2续焊轨迹与原始质量证明源不在同一 LaserPoint 案例目录。");
            return false;
        }
        QVector<WeldPoseFileRecord> proofSourceRecords;
        if (!LoadWeldPoseFileRecords(
                qualityProofPosePath,
                proofSourceRecords,
                error,
                &qualityProofPoseSha256,
                &qualityProofPoseSize))
        {
            error = QStringLiteral("V2续焊原始质量证明源回读失败：") + error;
            return false;
        }
    }
    else if (!qualityProofSourcePosePath.trimmed().isEmpty()
        && QFileInfo(QDir::fromNativeSeparators(qualityProofSourcePosePath)).absoluteFilePath()
            .compare(absolutePosePath, Qt::CaseInsensitive) != 0)
    {
        error = QStringLiteral("普通完整焊接禁止用另一条轨迹的质量证明替代当前输入。");
        return false;
    }

    // Keep a non-blocking read-use lease for the entire real motion lifecycle.
    // BeginProofReplacement takes the exclusive writer side and therefore cannot
    // install a denial tombstone between a successful final verification and any
    // Move/START/Call below.  Pure STEP file generation never enters this function
    // and deliberately does not receive a motion lease.
    PointCloudProofIntegrity::ProofUseLease qualityProofUseLease;
    if (poseSource == WeldPoseSource::PointCloudProduction)
    {
        const QString qualityProofPath = QFileInfo(qualityProofPosePath).dir().filePath(
            QString::fromLatin1(POINT_CLOUD_QUALITY_GATE_FILE_NAME));
        if (!PointCloudProofIntegrity::AcquireProofUseLease(
                qualityProofPath, qualityProofUseLease, error))
        {
            error = QStringLiteral(
                "生产焊接无法取得点云证明全生命周期只读租约，已在任何运动前拒绝：") + error;
            return false;
        }
    }

    const auto verifyLoadedPoseAuthorization = [&]() -> bool
    {
        return VerifyWeldPoseAuthorization(
            poseSource,
            qualityProofPosePath,
            qualityProofRobotName,
            qualityProofPoseSha256,
            qualityProofPoseSize,
            error,
            pRobotDriver);
    };
    if (!verifyLoadedPoseAuthorization())
    {
        return false;
    }

    WeldPosePreset weldPosePreset = LoadWeldPosePreset(param);
    if (param.bDoActualWeld
        && (!weldPosePreset.weldProcessLoaded || !weldPosePreset.weldProcessSafetyError.isEmpty()))
    {
        error = !weldPosePreset.weldProcessSafetyError.isEmpty()
            ? weldPosePreset.weldProcessSafetyError
            : (weldPosePreset.weldProcessLoadError.isEmpty()
                ? QStringLiteral("当前实际焊接工艺未能加载，已在机器人运动前中止。")
                : weldPosePreset.weldProcessLoadError);
        return false;
    }
    if (!allowPointwiseWeave && weldPosePreset.weaveEnabled && weldPosePreset.weaveAppPointwise)
    {
        error = QStringLiteral("pointwise 自定义摆动已被禁用(allowPointwiseWeave=false)，无法生成轨迹");
        return false;
    }
    const double effectiveFinalStepMm = ResolveEffectiveFinalStepMm(
        param, weldPosePreset, overrideFinalStepMm);
    const QString executionParameterFingerprint = BuildEffectiveWeldExecutionFingerprint(
        param, weldPosePreset, effectiveFinalStepMm);
    if (std::isfinite(overrideFinalStepMm) && overrideFinalStepMm > 0.0)
    {
        // 虚拟焊道测试：用户点间距覆盖工艺，作为机器人最终逐点执行间距。
        weldPosePreset.finalWeldStepFromProcessMm = effectiveFinalStepMm;
    }
    if (!inputAlreadyInExecutionOrder)
    {
        ApplyWeldDirectionToExecutionRecords(weldPosePreset, records);
    }
    if (resumeStartArcMm >= 0.0)
    {
        double actualStartArcMm = 0.0;
        if (!ClipWeldPoseRecordsAtArcLength(records, resumeStartArcMm, &actualStartArcMm, error))
        {
            return false;
        }
        if (appendLog)
        {
            appendLog(QString("断点续焊：按执行顺序从弧长 %1 mm 精确插值裁剪，剩余 %2 个源点；起弧在续焊首点前自动生成。")
                .arg(actualStartArcMm, 0, 'f', 3)
                .arg(records.size()));
        }
    }
    T_ROBOT_COORS startSafeCoors;
    if (!TryBuildWeldSafeCoors(
        records,
        0,
        param.dGunDownBackSafeDis,
        param.nWeldSafeRetreatDirection,
        weldPosePreset.robotType,
        startSafeCoors,
        error))
    {
        return false;
    }

    T_ROBOT_COORS endSafeCoors;
    if (!TryBuildWeldSafeCoors(
        records,
        records.size() - 1,
        param.dGunDownBackSafeDis,
        param.nWeldSafeRetreatDirection,
        weldPosePreset.robotType,
        endSafeCoors,
        error))
    {
        return false;
    }
    const T_ROBOT_COORS weldStartCoors = BuildWeldPoseCoors(records.front());

    const bool useWeldProcessTrajectorySpeed = param.bDoActualWeld
        && weldPosePreset.weldProcessLoaded
        && std::isfinite(weldPosePreset.weldProcessSpeedMmPerMin)
        && weldPosePreset.weldProcessSpeedMmPerMin > 0.0;
    const double selectedWeldSpeedMmPerMin = param.bDoActualWeld
        ? (useWeldProcessTrajectorySpeed ? weldPosePreset.weldProcessSpeedMmPerMin : param.dWeldSpeedMmPerMin)
        : param.dDryRunSpeedMmPerMin;
    const double weldSpeedMmPerMin =
        (std::isfinite(selectedWeldSpeedMmPerMin) && selectedWeldSpeedMmPerMin > 0.0)
        ? selectedWeldSpeedMmPerMin
        : (param.bDoActualWeld ? FANUC_WELD_PATH_SPEED_MM_PER_MIN : DEFAULT_DRY_RUN_SPEED_MM_PER_MIN);
    const QString weldSpeedSourceText = param.bDoActualWeld
        ? (useWeldProcessTrajectorySpeed ? QStringLiteral("焊接工艺WeldVelocity") : QStringLiteral("测量焊接参数WeldSpeedMmPerMin"))
        : QStringLiteral("空跑速度DryRunSpeedMmPerMin");
    const double safeMoveSpeedMmPerMin =
        (std::isfinite(param.dWeldSafeMoveSpeedMmPerMin) && param.dWeldSafeMoveSpeedMmPerMin > 0.0)
        ? param.dWeldSafeMoveSpeedMmPerMin
        : DEFAULT_WELD_SAFE_MOVE_SPEED_MM_PER_MIN;
    const QString weldModeText = param.bDoActualWeld ? QStringLiteral("实际焊接") : QStringLiteral("空跑");
    const double weldCommandSpeed =
        LinearCommandSpeedForRobot(pRobotDriver, weldSpeedMmPerMin, 1.0);
    const double transitionCommandSpeed =
        (weldPosePreset.transitionSpeedEnabled && weldPosePreset.transitionSpeedMmPerMin > 0.0)
        ? LinearCommandSpeedForRobot(pRobotDriver, weldPosePreset.transitionSpeedMmPerMin, weldCommandSpeed)
        : 0.0;
    const double safeMoveCommandSpeed =
        LinearCommandSpeedForRobot(pRobotDriver, safeMoveSpeedMmPerMin, 1.0);
    const QString weldCommandSpeedUnit = LinearCommandSpeedUnitText(pRobotDriver);
    FANUCRobotCtrl* pFanucDriverForGate = dynamic_cast<FANUCRobotCtrl*>(pRobotDriver);
    const bool isFanucDriver = pFanucDriverForGate != nullptr;
    if (isFanucDriver && param.bDoActualWeld)
    {
        std::string contractReason;
        if (!pFanucDriverForGate->HasVerifiedArcWeldContract(&contractReason))
        {
            error = DecodeRobotMessageText(contractReason);
            pRobotDriver->SetLastRobotError(contractReason);
            if (appendLog)
            {
                appendLog(error);
            }
            return false;
        }
    }
    if (isFanucDriver)
    {
        QStringList unrepresentableSpeeds;
        if (weldCommandSpeed < 1.0)
        {
            unrepresentableSpeeds << QString("焊接/空跑速度=%1mm/min")
                .arg(weldSpeedMmPerMin, 0, 'f', 3);
        }
        if (safeMoveCommandSpeed < 1.0)
        {
            unrepresentableSpeeds << QString("安全移动速度=%1mm/min")
                .arg(safeMoveSpeedMmPerMin, 0, 'f', 3);
        }
        if (weldPosePreset.transitionSpeedEnabled
            && weldPosePreset.transitionSpeedMmPerMin > 0.0
            && transitionCommandSpeed < 1.0)
        {
            unrepresentableSpeeds << QString("拐点过渡速度=%1mm/min")
                .arg(weldPosePreset.transitionSpeedMmPerMin, 0, 'f', 3);
        }
        if (!unrepresentableSpeeds.isEmpty())
        {
            error = "FANUC焊接流程已在首条运动前拒绝：固定TP最低只能表示1mm/sec（60mm/min），"
                "禁止把低速静默替换为更高速度。不可表示项："
                + unrepresentableSpeeds.join("，");
            pRobotDriver->SetLastRobotError(error.toUtf8().toStdString());
            if (appendLog)
            {
                appendLog(error);
            }
            return false;
        }
    }
    std::vector<T_ROBOT_MOVE_INFO> moveInfos;
    QVector<WeldPoseFileRecord> sampledRecords;
    if (!BuildWeldPoseMoveInfos(
        records,
        weldCommandSpeed,
        moveInfos,
        error,
        &weldPosePreset,
        effectiveFinalStepMm,
        transitionCommandSpeed,
        param.bDoActualWeld,
        &sampledRecords,
        inputAlreadyInExecutionOrder))
    {
        return false;
    }
    if (dynamic_cast<FANUCRobotCtrl*>(pRobotDriver) != nullptr)
    {
        for (size_t index = 0; index < moveInfos.size(); ++index)
        {
            const T_ROBOT_MOVE_INFO& info = moveInfos[index];
            if (info.nMoveType == MOVL
                && (!std::isfinite(info.tSpeed.dSpeed) || info.tSpeed.dSpeed < 1.0))
            {
                error = QString(
                    "FANUC焊接轨迹已在首条运动前拒绝：点%1的线速度无法表示（%2 mm/sec），最低1mm/sec且禁止静默提速。")
                    .arg(index)
                    .arg(info.tSpeed.dSpeed, 0, 'f', 6);
                pRobotDriver->SetLastRobotError(error.toUtf8().toStdString());
                if (appendLog)
                {
                    appendLog(error);
                }
                return false;
            }
        }
    }
    const QString sampledPosePath = BuildFinalSampledWeldPosePath(
        poseFilePath, inputAlreadyInExecutionOrder);
    QString sampledSaveError;
    QString sampledPoseSha256;
    qint64 sampledPoseSize = -1;
    const bool sampledSaved = SaveWeldPoseFileRecords(
        sampledPosePath,
        sampledRecords,
        sampledSaveError,
        &sampledPoseSha256,
        &sampledPoseSize);
    if (!sampledSaved)
    {
        error = QString("无法保存可验证的实际执行轨迹，已在机器人运动前中止：%1")
            .arg(sampledSaveError);
        return false;
    }

    WeldExecutionIdentity executionIdentity;
    executionIdentity.sourcePosePath = QDir::toNativeSeparators(QFileInfo(poseFilePath).absoluteFilePath());
    executionIdentity.sourcePoseSha256 = loadedPoseSha256;
    executionIdentity.sourcePoseSize = loadedPoseSize;
    executionIdentity.qualityProofPosePath = QDir::toNativeSeparators(qualityProofPosePath);
    executionIdentity.qualityProofPoseSha256 = qualityProofPoseSha256;
    executionIdentity.qualityProofPoseSize = qualityProofPoseSize;
    executionIdentity.sampledPosePath = QDir::toNativeSeparators(QFileInfo(sampledPosePath).absoluteFilePath());
    executionIdentity.sampledPoseSha256 = sampledPoseSha256;
    executionIdentity.sampledPoseSize = sampledPoseSize;
    executionIdentity.sampledPointCount = sampledRecords.size();
    executionIdentity.effectiveFinalStepMm = effectiveFinalStepMm;
    executionIdentity.parameterFingerprint = executionParameterFingerprint;
    executionIdentity.trajectoryInExecutionOrder = true;
    executionIdentity.requiredEndSafePose = endSafeCoors;
    executionIdentity.safeMoveSpeedMmPerMin = safeMoveSpeedMmPerMin;
    executionIdentity.resumeCheckpointSupported = !(param.bDoActualWeld
        && (weldPosePreset.weaveEnabled || weldPosePreset.trackEnabled));
    if (!executionIdentity.resumeCheckpointSupported)
    {
        executionIdentity.resumeUnsupportedReason = QStringLiteral(
            "当前实际焊接工艺启用了摆焊或焊缝跟踪；FinalSampled仅是中心线，无法证明机器人真实TCP弧长，禁止生成或使用自动续焊断点。");
    }

    // 必须早于旧断点失效和第一条机器人运动。续焊调用方在这里复核机器人、工艺和
    // 轨迹身份；后续 executionPrepared 还会在具体程序 START 前再次复核并冻结暂停上下文。
    if (executionPreMotion)
    {
        QString preMotionError;
        if (!executionPreMotion(executionIdentity, preMotionError))
        {
            error = preMotionError.isEmpty()
                ? QStringLiteral("机器人运动前复核焊接执行身份失败，已中止。")
                : preMotionError;
            return false;
        }
    }
    if (!verifyLoadedPoseAuthorization())
    {
        error = QStringLiteral("机器人运动前点云质量证明复核失败：") + error;
        return false;
    }

    // 所有入口共用：任何新的完整焊接在第一条机器人运动前都必须使旧 paused 记录失效。
    // V2续焊输入除外，它已在对话框中先原子切到 resuming，并需保留到新程序终态。
    if (!inputAlreadyInExecutionOrder)
    {
        QString invalidateError;
        const QString robotName = QString::fromStdString(param.sRobotName).trimmed().isEmpty()
            ? QString::fromStdString(pRobotDriver->m_sRobotName)
            : QString::fromStdString(param.sRobotName);
        if (!InvalidateStoredWeldResumeCheckpoint(robotName, invalidateError))
        {
            error = invalidateError + QStringLiteral("；为避免新焊道完成后误用旧断点，已在机器人运动前中止。");
            return false;
        }
    }

    if (pStartSafeCoors != nullptr)
    {
        *pStartSafeCoors = startSafeCoors;
    }
    if (pEndSafeCoors != nullptr)
    {
        *pEndSafeCoors = endSafeCoors;
    }

    if (appendLog)
    {
        appendLog(QString("焊接安全位：回退距离=%1 mm，水平回撤方向=%2")
            .arg(param.dGunDownBackSafeDis, 0, 'f', 3)
            .arg(WeldSafeRetreatDirectionText(param.nWeldSafeRetreatDirection)));
        appendLog(QString("下枪安全位置：%1").arg(RobotCoorsText(startSafeCoors)));
        appendLog(QString("焊接起点：%1").arg(RobotCoorsText(weldStartCoors)));
        appendLog(QString("收枪安全位置：%1").arg(RobotCoorsText(endSafeCoors)));
        appendLog(QString("焊接轨迹模式：%1，轨迹速度=%2 mm/min，来源=%3，下发速度=%4 %5")
            .arg(weldModeText)
            .arg(weldSpeedMmPerMin, 0, 'f', 3)
            .arg(weldSpeedSourceText)
            .arg(weldCommandSpeed, 0, 'f', 3)
            .arg(weldCommandSpeedUnit));
        appendLog(QString("最终焊接轨迹点间距：%1 mm，仅在生成/下发运动点时抽样")
            .arg(effectiveFinalStepMm, 0, 'f', 3));
        appendLog(sampledSaved
            ? QString("最终抽样轨迹文件：%1").arg(sampledPosePath)
            : QString("最终抽样轨迹文件保存失败：%1").arg(sampledSaveError));
        appendLog(QString("焊接方向：%1").arg(WeldDirectionText(weldPosePreset)));
        if (param.bDoActualWeld)
        {
            if (weldPosePreset.weldProcessLoaded)
            {
                appendLog(QString("焊接工艺：起弧=%1A/%2V，焊接=%3A/%4V，停弧=%5A/%6V")
                    .arg(weldPosePreset.startArcCurrent, 0, 'f', 3)
                    .arg(weldPosePreset.startArcVoltage, 0, 'f', 3)
                    .arg(weldPosePreset.weldCurrent, 0, 'f', 3)
                    .arg(weldPosePreset.weldVoltage, 0, 'f', 3)
                    .arg(weldPosePreset.stopArcCurrent, 0, 'f', 3)
                    .arg(weldPosePreset.stopArcVoltage, 0, 'f', 3));
                appendLog(QString("工艺附加变量：摆动=%1，跟踪=%2")
                    .arg(weldPosePreset.weaveEnabled ? QStringLiteral("启用") : QStringLiteral("NULL"))
                    .arg(weldPosePreset.trackEnabled ? QStringLiteral("启用") : QStringLiteral("NULL")));
                if (weldPosePreset.cornerArcRadiusFromWeldProcess)
                {
                    appendLog(QString("拐点圆弧半径使用当前工艺参数：%1 mm")
                        .arg(weldPosePreset.cornerArcRadiusMm, 0, 'f', 3));
                }
                if (weldPosePreset.transitionSpeedEnabled)
                {
                    appendLog(QString("拐点过渡速度启用：%1 mm/min (下发=%2 %3)")
                        .arg(weldPosePreset.transitionSpeedMmPerMin, 0, 'f', 3)
                        .arg(transitionCommandSpeed, 0, 'f', 3)
                        .arg(weldCommandSpeedUnit));
                }
                if (weldPosePreset.transitionCurrentVoltageEnabled)
                {
                    appendLog(QString("拐点过渡电流电压启用：%1A/%2V")
                        .arg(weldPosePreset.transitionCurrent, 0, 'f', 3)
                        .arg(weldPosePreset.transitionVoltage, 0, 'f', 3));
                }
                if (weldPosePreset.transitionSpeedEnabled || weldPosePreset.transitionCurrentVoltageEnabled)
                {
                    appendLog(QString("拐点过渡作用范围：%1")
                        .arg(TransitionApplyScopeText(weldPosePreset.transitionApplyScope)));
                }
            }
            else
            {
                appendLog("未读取到当前焊接工艺参数，本次只按轨迹运动执行，不生成起弧/停弧工艺语句。");
            }
        }
    }

    QString downlinkSummary;
    QString programNameText;
    int lastState = 0;
    const double pathLengthMm = EstimateMoveInfosPathLengthMm(moveInfos);
    const double estimatedRunMs = EstimateMoveInfosRunMs(
        moveInfos,
        dynamic_cast<FANUCRobotCtrl*>(pRobotDriver) != nullptr);
    const double requiredFinishTimeoutMs = estimatedRunMs * 2.0 + 30000.0;
    if (!std::isfinite(requiredFinishTimeoutMs)
        || requiredFinishTimeoutMs > RobotMotionTimeoutPolicy::kMotionTimeoutMs)
    {
        error = QString(
            "焊接轨迹已拒绝：长度≈%1 mm、预计≈%2 min，含安全裕量后超过30分钟上限；请提高速度或拆分轨迹。")
            .arg(pathLengthMm, 0, 'f', 1)
            .arg(estimatedRunMs / 60000.0, 0, 'f', 1);
        if (appendLog)
        {
            appendLog(error);
        }
        pRobotDriver->SetLastRobotError(error.toUtf8().toStdString());
        return false;
    }
    const int finishTimeoutMs = static_cast<int>(std::clamp(
        requiredFinishTimeoutMs,
        120000.0,
        static_cast<double>(RobotMotionTimeoutPolicy::kMotionTimeoutMs)));

    auto stopBeforeNextWeldAction = [&](const QString& nextAction) -> bool
        {
            if (!stopRequested || !stopRequested())
            {
                return false;
            }

            error = QString("已收到停止请求，当前步骤已结束，未继续%1。").arg(nextAction);
            if (appendLog)
            {
                appendLog(error);
            }
            return true;
        };

    class ExecutionContextGuard
    {
    public:
        ExecutionContextGuard(
            RobotDriverAdaptor* driver,
            const WeldExecutionFinishedCallback& callback)
            : m_driver(driver), m_callback(callback)
        {
        }
        ~ExecutionContextGuard()
        {
            if (!m_armed || m_programCompletionPersisted)
            {
                return;
            }
            if (m_result.programStartAttempted)
            {
                RobotOperationLease::RequestCancellation(m_driver);
            }
            m_result.state = WeldExecutionTerminalState::Incomplete;
            m_result.reason = m_result.programStartAttempted
                ? QStringLiteral("机器人程序START已尝试，但未形成完整焊接+安全回撤终态；保持持久闭锁。")
                : QStringLiteral("已确认未尝试机器人程序START，释放本轮预锁存门禁。");
            m_result.observedAtUtc = QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs);
            QString ignoredError;
            if (m_callback)
            {
                m_callback(m_result, ignoredError);
            }
        }
        void Arm(const WeldExecutionIdentity& identity)
        {
            m_result.requiredSafePose = identity.requiredEndSafePose;
            m_result.safeMoveSpeedMmPerMin = identity.safeMoveSpeedMmPerMin;
            m_armed = true;
        }
        void MarkProgramStartAttempted()
        {
            if (m_armed)
            {
                m_result.programStartAttempted = true;
            }
        }
        bool IsArmed() const { return m_armed; }
        bool PersistProgramCompletedUnretracted(const QString& reason, QString& persistError)
        {
            if (!m_armed)
            {
                persistError = QStringLiteral("焊接安全终态门禁未预先冻结。");
                return false;
            }
            m_result.state = WeldExecutionTerminalState::ProgramCompletedUnretracted;
            m_result.reason = reason;
            m_result.observedAtUtc = QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs);
            T_ROBOT_COORS observed;
            m_result.observedTerminalPoseValid =
                m_driver != nullptr && m_driver->TryGetCurrentPos(observed);
            if (m_result.observedTerminalPoseValid)
            {
                m_result.observedTerminalPose = observed;
            }
            if (!m_callback || !m_callback(m_result, persistError))
            {
                return false;
            }
            m_programCompletionPersisted = true;
            return true;
        }
        void UpdateUnretractedReason(const QString& reason)
        {
            if (!m_armed || !m_programCompletionPersisted || !m_callback)
            {
                return;
            }
            m_result.state = WeldExecutionTerminalState::ProgramCompletedUnretracted;
            m_result.reason = reason;
            m_result.observedAtUtc = QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs);
            QString ignoredError;
            m_callback(m_result, ignoredError);
        }
        bool FinishSafelyRetracted(const T_ROBOT_COORS& observed, QString& persistError)
        {
            if (!m_armed || !m_programCompletionPersisted)
            {
                persistError = QStringLiteral("焊接程序完成/未回撤终态尚未持久化，禁止释放安全门禁。");
                return false;
            }
            m_result.state = WeldExecutionTerminalState::SafelyRetracted;
            m_result.reason = QStringLiteral("焊接程序完成，收枪安全位置运动及实际位置回读均已验证成功。");
            m_result.observedAtUtc = QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs);
            m_result.observedTerminalPose = observed;
            m_result.observedTerminalPoseValid = true;
            if (m_callback)
            {
                if (!m_callback(m_result, persistError))
                {
                    return false;
                }
            }
            m_armed = false;
            return true;
        }

    private:
        RobotDriverAdaptor* m_driver = nullptr;
        const WeldExecutionFinishedCallback& m_callback;
        WeldExecutionTerminalResult m_result;
        bool m_armed = false;
        bool m_programCompletionPersisted = false;
    } executionContextGuard(pRobotDriver, executionFinished);

    // 自动流程的最后进入门禁：必须放在所有解析/生成之后、第一条机器人运动之前，
    // 避免用户在扫描完成到焊接函数真正开始之间按下停止仍触发下枪运动。
    if (stopBeforeNextWeldAction("焊接下枪运动"))
    {
        return false;
    }

    if (FANUCRobotCtrl* pFanucDriver = dynamic_cast<FANUCRobotCtrl*>(pRobotDriver))
    {
        const double safeMoveSpeedMmPerSec = safeMoveCommandSpeed;
        if (setFlowStep)
        {
            setFlowStep("正在移动到下枪安全位置，并行下发焊接轨迹");
        }
        if (appendLog)
        {
            appendLog(QString("开始直线运动：下枪安全位置，配置速度= %1 mm/min，下发速度= %2 mm/sec")
                .arg(safeMoveSpeedMmPerMin, 0, 'f', 3)
                .arg(safeMoveSpeedMmPerSec, 0, 'f', 3));
        }

        T_ROBOT_COORS currentSafeStart;
        if (!pFanucDriver->TryGetCurrentPos(currentSafeStart))
        {
            error = "下枪安全位置运动已拒绝：读取当前机器人位置失败，"
                + RobotMotionStatusText(pRobotDriver);
            if (appendLog)
            {
                appendLog(error);
            }
            return false;
        }
        int startSafeTimeoutMs = 0;
        std::string startSafeAdmissionError;
        if (!RobotMotionTimeoutPolicy::AdmitCartesianMove(
            currentSafeStart,
            startSafeCoors,
            safeMoveSpeedMmPerSec * 60.0,
            startSafeTimeoutMs,
            &startSafeAdmissionError))
        {
            error = "下枪安全位置运动已拒绝："
                + QString::fromUtf8(startSafeAdmissionError.c_str());
            pRobotDriver->SetLastRobotError(error.toUtf8().toStdString());
            if (appendLog)
            {
                appendLog(error);
            }
            return false;
        }

        if (!verifyLoadedPoseAuthorization())
        {
            error = QStringLiteral("下枪安全运动前焊道授权复核失败：") + error;
            return false;
        }
        if (!pFanucDriver->MoveByJob(
            startSafeCoors,
            T_ROBOT_MOVE_SPEED(safeMoveSpeedMmPerSec, 0.0, 0.0),
            pFanucDriver->m_nExternalAxleType,
            "MOVL"))
        {
            error = "启动下枪安全位置直线运动失败。";
            if (RobotOperationLease::MotionCompletionPending(pRobotDriver)
                && !RobotOperationLease::IsCancellationRequested(pRobotDriver))
            {
                StopUnverifiedMotionAfterFailure(pRobotDriver, error, appendLog);
                error = DecodeRobotMessageText(pRobotDriver->GetLastRobotError());
            }
            return false;
        }
        if (appendLog)
        {
            appendLog("下枪安全位置运动已启动，开始并行下发焊接轨迹程序。");
        }

        std::string programName;
        std::string localLsPath;
        std::string remoteTpPath;
        const int downlinkRet = pFanucDriver->UploadMultiPointTpProgram(
            moveInfos,
            param.bDoActualWeld
                ? FANUCRobotCtrl::TrajectoryProgramMode::ActualWeld
                : FANUCRobotCtrl::TrajectoryProgramMode::DryRun,
            &programName,
            &localLsPath,
            &remoteTpPath);
        if (downlinkRet != 0)
        {
            error = QString("下发焊接轨迹失败：ret=%1，姿态文件=%2。机器人可能仍在移动到下枪安全位置。")
                .arg(downlinkRet)
                .arg(QDir::toNativeSeparators(QFileInfo(poseFilePath).absoluteFilePath()));
            StopUnverifiedMotionAfterFailure(pRobotDriver, error, appendLog);
            error = DecodeRobotMessageText(pRobotDriver->GetLastRobotError());
            return false;
        }

        int startSafeLastState = 0;
        const bool startSafeDoneOk = pFanucDriver->WaitStateDone(
            FANUC_MOTION_STATE_REG, 1, 10, 20, 3000, startSafeTimeoutMs, 100, &startSafeLastState);
        if (appendLog)
        {
            appendLog(QString("直线运动结束：下枪安全位置, R[%1]=%2, WaitStateDone=%3")
                .arg(FANUC_MOTION_STATE_REG)
                .arg(startSafeLastState)
                .arg(startSafeDoneOk ? 1 : 0));
        }
        if (!startSafeDoneOk)
        {
            error = QString("下枪安全位置未完成，R[%1]=%2，取消运行焊接轨迹。")
                .arg(FANUC_MOTION_STATE_REG)
                .arg(startSafeLastState);
            StopUnverifiedMotionAfterFailure(pRobotDriver, error, appendLog);
            error = DecodeRobotMessageText(pRobotDriver->GetLastRobotError());
            return false;
        }

        const int startSafeTaskDone = pFanucDriver->CheckRobotDone(100, 30000);
        if (appendLog)
        {
            appendLog(QString("下枪安全位置任务终态：CheckRobotDone=%1").arg(startSafeTaskDone));
        }
        if (startSafeTaskDone <= 0)
        {
            const QString detail = RobotMotionStatusText(pRobotDriver);
            error = detail.isEmpty()
                ? QString("下枪安全位置虽写入 R[%1]=%2，但机器人任务未稳定退出：CheckRobotDone=%3。")
                    .arg(FANUC_MOTION_STATE_REG)
                    .arg(startSafeLastState)
                    .arg(startSafeTaskDone)
                : QString("下枪安全位置任务未稳定退出：CheckRobotDone=%1，%2")
                    .arg(startSafeTaskDone)
                    .arg(detail);
            // CheckRobotDone 已对非取消异常执行可验证中止。
            return false;
        }

        if (stopBeforeNextWeldAction("移动到焊接起点"))
        {
            return false;
        }

        if (!verifyLoadedPoseAuthorization())
        {
            error = QStringLiteral("焊接起点运动前焊道授权复核失败：") + error;
            return false;
        }
        if (!MoveCoorsAndWait(
            pRobotDriver,
            weldStartCoors,
            safeMoveSpeedMmPerMin,
            "焊接起点",
            appendLog,
            setFlowStep))
        {
            error = "移动到焊接起点失败。";
            return false;
        }

        if (stopBeforeNextWeldAction("执行焊接轨迹程序"))
        {
            return false;
        }

        if (checkpoint && !checkpoint(
            "焊前确认",
            QString("下枪安全位置和焊接起点已到位，焊接轨迹程序也已下发完成。\n"
                    "运行模式：%1\n"
                    "下枪安全位置：%2\n"
                    "焊接起点：%3\n"
                    "焊接程序：%4\n"
                    "是否开始执行焊道？")
                .arg(weldModeText)
                .arg(RobotCoorsText(startSafeCoors))
                .arg(RobotCoorsText(weldStartCoors))
                .arg(QString::fromStdString(programName))))
        {
            error = "用户在焊前确认节点取消了流程。";
            return false;
        }

        downlinkSummary =
            QString("点数=%1，模式=%2，轨迹速度=%3 mm/min，来源=%4 (下发=%5 %6)，程序=%7，本地LS=%8，远程TP=%9")
                .arg(static_cast<int>(moveInfos.size()))
                .arg(weldModeText)
                .arg(weldSpeedMmPerMin, 0, 'f', 3)
                .arg(weldSpeedSourceText)
                .arg(weldCommandSpeed, 0, 'f', 3)
                .arg(weldCommandSpeedUnit)
                .arg(QString::fromStdString(programName))
                .arg(QDir::toNativeSeparators(QString::fromStdString(localLsPath)))
                .arg(QString::fromStdString(remoteTpPath));
        programNameText = QString::fromStdString(programName);
        executionIdentity.programName = programNameText;
        executionIdentity.localProgramPath = QString::fromStdString(localLsPath);
        if (executionPrepared)
        {
            QString prepareError;
            if (!executionPrepared(executionIdentity, prepareError))
            {
                error = prepareError.isEmpty()
                    ? QStringLiteral("冻结焊接执行身份失败，未启动机器人程序。")
                    : prepareError;
                return false;
            }
            executionContextGuard.Arm(executionIdentity);
        }

        if (stopBeforeNextWeldAction("启动焊接轨迹程序"))
        {
            return false;
        }

        if (setFlowStep)
        {
            setFlowStep(QString("正在执行焊接轨迹程序：%1").arg(programNameText));
        }
        if (appendLog)
        {
            appendLog(QString("开始执行焊接轨迹程序：%1，轨迹长度≈%2 mm，预计运行≈%3 s，完成超时=%4 s")
                .arg(programNameText)
                .arg(pathLengthMm, 0, 'f', 3)
                .arg(estimatedRunMs / 1000.0, 0, 'f', 1)
                .arg(finishTimeoutMs / 1000.0));
        }

        if (!verifyLoadedPoseAuthorization())
        {
            error = QStringLiteral("焊接程序启动前焊道授权复核失败：") + error;
            return false;
        }
        executionContextGuard.MarkProgramStartAttempted();
        if (!pFanucDriver->CallJobAndWaitStateDone(
            programName,
            FANUC_MOTION_STATE_REG,
            1,
            10,
            20,
            5000,
            finishTimeoutMs,
            100,
            &lastState,
            true))
        {
            error = QString("焊接轨迹程序执行失败：%1，R[%2]=%3")
                .arg(programNameText)
                .arg(FANUC_MOTION_STATE_REG)
                .arg(lastState);
            return false;
        }
        if (appendLog)
        {
            appendLog(QString("焊接轨迹程序执行完成：%1，R[%2]=%3")
                .arg(programNameText)
                .arg(FANUC_MOTION_STATE_REG)
                .arg(lastState));
        }
    }
    else
    {
        if (!verifyLoadedPoseAuthorization())
        {
            error = QStringLiteral("下枪安全运动前焊道授权复核失败：") + error;
            return false;
        }
        if (!MoveCoorsAndWait(
            pRobotDriver,
            startSafeCoors,
            safeMoveSpeedMmPerMin,
            "下枪安全位置",
            appendLog,
            setFlowStep))
        {
            error = "移动到下枪安全位置失败。";
            return false;
        }

        if (stopBeforeNextWeldAction("移动到焊接起点"))
        {
            return false;
        }

        if (!verifyLoadedPoseAuthorization())
        {
            error = QStringLiteral("焊接起点运动前焊道授权复核失败：") + error;
            return false;
        }
        if (!MoveCoorsAndWait(
            pRobotDriver,
            weldStartCoors,
            safeMoveSpeedMmPerMin,
            "焊接起点",
            appendLog,
            setFlowStep))
        {
            error = "移动到焊接起点失败。";
            return false;
        }

        if (stopBeforeNextWeldAction("生成、上传并执行STEP焊接轨迹"))
        {
            return false;
        }

        if (checkpoint && !checkpoint(
            "焊前确认",
            QString("下枪安全位置和焊接起点已到位，STEP 将生成、上传并启动焊接轨迹。\n"
                    "运行模式：%1\n"
                    "下枪安全位置：%2\n"
                    "焊接起点：%3\n"
                    "点数：%4\n"
                    "是否开始执行焊道？")
                .arg(weldModeText)
                .arg(RobotCoorsText(startSafeCoors))
                .arg(RobotCoorsText(weldStartCoors))
                .arg(static_cast<int>(moveInfos.size()))))
        {
            error = "用户在焊前确认节点取消了流程。";
            return false;
        }

        std::string stepProgramName = STEPRobotCtrl::MakeTimestampWeldProgramName();
        STEPRobotCtrl* pStepDriver = dynamic_cast<STEPRobotCtrl*>(pRobotDriver);
        programNameText = pStepDriver != nullptr
            ? QString::fromStdString(stepProgramName)
            : QStringLiteral("STEP ContiMoveAny");
        executionIdentity.programName = programNameText;
        executionIdentity.localProgramPath.clear();
        if (executionPrepared)
        {
            QString prepareError;
            if (!executionPrepared(executionIdentity, prepareError))
            {
                error = prepareError.isEmpty()
                    ? QStringLiteral("冻结焊接执行身份失败，未启动STEP程序。")
                    : prepareError;
                return false;
            }
            executionContextGuard.Arm(executionIdentity);
        }
        if (stopBeforeNextWeldAction("启动STEP焊接轨迹程序"))
        {
            return false;
        }
        downlinkSummary =
            QString("点数=%1，模式=%2，轨迹速度=%3 mm/min，来源=%4 (下发=%5 %6)，STEP使用%7生成、上传并启动程序")
                .arg(static_cast<int>(moveInfos.size()))
                .arg(weldModeText)
                .arg(weldSpeedMmPerMin, 0, 'f', 3)
                .arg(weldSpeedSourceText)
                .arg(weldCommandSpeed, 0, 'f', 3)
                .arg(weldCommandSpeedUnit)
                .arg(programNameText);

        if (setFlowStep)
        {
            setFlowStep(QString("正在执行STEP焊接轨迹程序：%1").arg(programNameText));
        }
        if (appendLog)
        {
            appendLog(QString("开始执行STEP焊接轨迹：程序=%1，轨迹长度≈%2 mm，预计运行≈%3 s，完成超时=%4 s")
                .arg(programNameText)
                .arg(pathLengthMm, 0, 'f', 3)
                .arg(estimatedRunMs / 1000.0, 0, 'f', 1)
                .arg(finishTimeoutMs / 1000.0));
        }

        if (!verifyLoadedPoseAuthorization())
        {
            error = QStringLiteral("STEP 焊接程序启动前焊道授权复核失败：") + error;
            return false;
        }
        executionContextGuard.MarkProgramStartAttempted();
        const int ret = pStepDriver != nullptr
            ? pStepDriver->ContiMoveAnyWithProgramName(moveInfos, stepProgramName)
            : pRobotDriver->ContiMoveAny(moveInfos);
        if (ret != 0)
        {
            const QString detail = RobotMotionStatusText(pRobotDriver);
            error = detail.isEmpty()
                ? QString("STEP焊接轨迹下发/启动失败：ret=%1").arg(ret)
                : QString("STEP焊接轨迹下发/启动失败：ret=%1，%2").arg(ret).arg(detail);
            return false;
        }

        lastState = pRobotDriver->CheckRobotDone(100, finishTimeoutMs);
        if (lastState <= 0)
        {
            const QString detail = RobotMotionStatusText(pRobotDriver);
            error = detail.isEmpty()
                ? QString("STEP焊接轨迹等待完成失败：CheckRobotDone=%1").arg(lastState)
                : QString("STEP焊接轨迹等待完成失败：CheckRobotDone=%1，%2").arg(lastState).arg(detail);
            return false;
        }
        if (appendLog)
        {
            appendLog(QString("STEP焊接轨迹执行完成：CheckRobotDone=%1").arg(lastState));
        }
    }

    QString terminalPersistError;
    if (executionContextGuard.IsArmed()
        && !executionContextGuard.PersistProgramCompletedUnretracted(
            QStringLiteral("焊接程序已确认完成，仅待用户确认并执行收枪安全回撤。"),
            terminalPersistError))
    {
        RobotOperationLease::RequestCancellation(pRobotDriver);
        error = QStringLiteral("焊接程序已完成，但持久化未回撤安全门禁失败；已锁存安全停止：")
            + terminalPersistError;
        return false;
    }

    const bool postWeldConfirmed = !checkpoint || checkpoint(
        "焊后确认",
        QString("焊接轨迹已执行完成。\n程序：%1\n完成状态=%2\n"
                "确定：移动到收枪安全位置并验证。\n"
                "取消：立即终止流程，保留“程序已完成/未回撤”持久锁；之后只能走焊后安全回撤恢复，绝不会重焊焊缝。")
            .arg(programNameText)
            .arg(lastState));
    if (!postWeldConfirmed)
    {
        executionContextGuard.UpdateUnretractedReason(
            QStringLiteral("用户在焊后确认选择取消；未发起任何新运动，保留未回撤恢复门禁。"));
        RobotOperationLease::RequestCancellation(pRobotDriver);
        error = QStringLiteral(
            "用户在焊后确认节点取消了流程；焊接程序已完成但未安全回撤，恢复门禁和端点闭锁已保留。");
        return false;
    }

    const bool stopLatched = RobotOperationLease::IsCancellationRequested(pRobotDriver)
        || (stopRequested && stopRequested());
    if (!WeldExecutionSafety::ShouldAttemptMandatoryRetreat(postWeldConfirmed, stopLatched))
    {
        executionContextGuard.UpdateUnretractedReason(
            QStringLiteral("焊接程序完成后检测到安全 STOP 锁存；禁止绕过 STOP 自动回撤。"));
        RobotOperationLease::RequestCancellation(pRobotDriver);
        error = QStringLiteral(
            "焊接程序已完成，但安全 STOP 已锁存；未发起收枪运动，未回撤恢复门禁保持有效。");
        return false;
    }

    if (!MoveCoorsAndWait(
        pRobotDriver,
        endSafeCoors,
        safeMoveSpeedMmPerMin,
        "收枪安全位置",
        appendLog,
        setFlowStep))
    {
        const QString retreatFailure = QStringLiteral("移动到收枪安全位置失败；程序已完成但回撤未验证。");
        if (RobotOperationLease::MotionCompletionPending(pRobotDriver)
            && !RobotOperationLease::IsCancellationRequested(pRobotDriver))
        {
            RobotOperationLease::StopAndConfirmUnverifiedMotion(pRobotDriver);
        }
        RobotOperationLease::RequestCancellation(pRobotDriver);
        executionContextGuard.UpdateUnretractedReason(retreatFailure);
        error = retreatFailure;
        return false;
    }

    T_ROBOT_COORS observedSafePose;
    QString safePoseError;
    if (!VerifyRobotAtSafePose(pRobotDriver, endSafeCoors, observedSafePose, safePoseError))
    {
        RobotOperationLease::RequestCancellation(pRobotDriver);
        executionContextGuard.UpdateUnretractedReason(safePoseError);
        error = safePoseError;
        return false;
    }
    QString finishPersistError;
    if (executionContextGuard.IsArmed()
        && !executionContextGuard.FinishSafelyRetracted(observedSafePose, finishPersistError))
    {
        RobotOperationLease::RequestCancellation(pRobotDriver);
        error = QStringLiteral("安全回撤已验证，但持久终态写回失败，仍保持闭锁：") + finishPersistError;
        return false;
    }

    summary = QString("%1；安全移动速度=%2 mm/min；起点安全位=%3；焊接起点=%4；终点安全位=%5")
        .arg(downlinkSummary)
        .arg(safeMoveSpeedMmPerMin, 0, 'f', 3)
        .arg(RobotCoorsText(startSafeCoors))
        .arg(RobotCoorsText(weldStartCoors))
        .arg(RobotCoorsText(endSafeCoors));
    return true;
}

// ===== 补偿前后焊道可视化预览实现 =====
// 这些成员函数位于匿名命名空间之后，可直接复用其内的真实补偿数学
// （ApplyWeldSeamCompToWeldPoseRecords / TryParseWeldPoseFileRecord /
//  ResolveOverallHorizontalWeldDirection 等），保证预览与实际下发轨迹一致。

bool MeasureThenWeldService::LoadCompPreviewBaseline(
    CompPreviewKind kind,
    const QString& laserDir,
    QVector<CompPreviewPoint>& baseline,
    QString& error,
    const StopRequestedCallback& stopRequested) const
{
    baseline.clear();
    QDir dir(laserDir);
    if (!dir.exists())
    {
        error = QString("目录不存在：%1").arg(laserDir);
        return false;
    }

    if (kind == CompPreviewKind::Corner)
    {
        // 拐点补偿的补偿前基准 = 关键点（start/end/inner/outer），用于按几何重算段类并整体补偿。
        const QString keyPath = dir.filePath(KEY_POINTS_FILE_NAME);
        QFile keyFile(keyPath);
        if (!keyFile.open(QIODevice::ReadOnly | QIODevice::Text))
        {
            error = QString("打开拐点文件失败：%1").arg(keyPath);
            return false;
        }
        QTextStream keyStream(&keyFile);
        keyStream.setEncoding(QStringConverter::Utf8);
        quint64 lineIndex = 0;
        while (!keyStream.atEnd())
        {
            if ((lineIndex++ & 0xffU) == 0U && stopRequested && stopRequested())
            {
                baseline.clear();
                error = QStringLiteral("已取消读取拐点预览。");
                return false;
            }
            const QString lineText = keyStream.readLine().trimmed();
            if (lineText.isEmpty() || lineText.startsWith('#'))
            {
                continue;
            }
            const QStringList parts = lineText.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
            if (parts.size() < 5)
            {
                continue;
            }
            bool xOk = false, yOk = false, zOk = false, codeOk = false;
            const double x = parts[1].toDouble(&xOk);
            const double y = parts[2].toDouble(&yOk);
            const double z = parts[3].toDouble(&zOk);
            const int code = parts[4].toInt(&codeOk);
            if (!xOk || !yOk || !zOk || !codeOk)
            {
                continue;
            }
            CompPreviewPoint point;
            point.x = x;
            point.y = y;
            point.z = z;
            point.typeCode = code;
            point.pointType = parts.size() > 5 ? parts[5] : QString();
            baseline.push_back(point);
        }
        if (baseline.size() < 2)
        {
            error = QString("未从 %1 解析到足够拐点（至少2个）。").arg(KEY_POINTS_FILE_NAME);
            return false;
        }
        return true;
    }

    // 焊道补偿/姿态补偿的补偿前基准 = 稠密 2mm 焊道姿态文件。
    const QString path = dir.filePath(WELD_POSE_FILE_NAME);
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        error = QString("打开焊道文件失败：%1").arg(path);
        return false;
    }
    QTextStream stream(&file);
    stream.setEncoding(QStringConverter::Utf8);
    quint64 lineIndex = 0;
    while (!stream.atEnd())
    {
        if ((lineIndex++ & 0xffU) == 0U && stopRequested && stopRequested())
        {
            baseline.clear();
            error = QStringLiteral("已取消读取基准焊道预览。");
            return false;
        }
        const QString line = stream.readLine();
        WeldPoseFileRecord record;
        if (!TryParseWeldPoseFileRecord(line, record))
        {
            continue;
        }
        CompPreviewPoint point;
        point.x = record.point.x();
        point.y = record.point.y();
        point.z = record.point.z();
        point.rx = record.rx;
        point.ry = record.ry;
        point.rz = record.rz;
        point.bx = record.bx;
        point.by = record.by;
        point.bz = record.bz;
        point.weldIndex = record.weldIndex;
        point.rawIndex = record.rawIndex;
        point.segmentKind = record.segmentKind;
        point.pointType = record.pointType;
        point.isLapStep = record.isLapStep;
        baseline.push_back(point);
    }
    if (baseline.isEmpty())
    {
        error = QString("未从 %1 解析到有效焊道点。").arg(WELD_POSE_FILE_NAME);
        return false;
    }
    return true;
}

bool MeasureThenWeldService::LoadCompPreviewOriginalTrack(
    const QString& laserDir,
    QVector<CompPreviewPoint>& points,
    QString& error,
    const StopRequestedCallback& stopRequested) const
{
    points.clear();
    QDir dir(laserDir);
    if (!dir.exists())
    {
        error = QString("目录不存在：%1").arg(laserDir);
        return false;
    }
    const QString path = dir.filePath(CLASSIFIED_FILE_NAME);
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        error = QString("打开原始焊道文件失败：%1").arg(path);
        return false;
    }
    QTextStream stream(&file);
    stream.setEncoding(QStringConverter::Utf8);
    quint64 lineIndex = 0;
    while (!stream.atEnd())
    {
        if ((lineIndex++ & 0xffU) == 0U && stopRequested && stopRequested())
        {
            points.clear();
            error = QStringLiteral("已取消读取原始焊道预览。");
            return false;
        }
        const QString lineText = stream.readLine().trimmed();
        if (lineText.isEmpty() || lineText.startsWith('#'))
        {
            continue;
        }
        const QStringList parts = lineText.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
        if (parts.size() < 4)
        {
            continue;
        }
        bool xOk = false, yOk = false, zOk = false;
        const double x = parts[1].toDouble(&xOk);
        const double y = parts[2].toDouble(&yOk);
        const double z = parts[3].toDouble(&zOk);
        if (!xOk || !yOk || !zOk)
        {
            continue;
        }
        CompPreviewPoint point;
        point.x = x;
        point.y = y;
        point.z = z;
        if (parts.size() > 4)
        {
            point.typeCode = parts[4].toInt();
        }
        points.push_back(point);
    }
    if (points.isEmpty())
    {
        error = QString("未从 %1 解析到原始焊道点。").arg(CLASSIFIED_FILE_NAME);
        return false;
    }
    return true;
}

MeasureThenWeldService::CompPreviewResult MeasureThenWeldService::RecomputeCompPreview(
    CompPreviewKind kind,
    const QString& robotName,
    const QVector<CompPreviewPoint>& baseline,
    const CompPreviewEditValues& edits) const
{
    CompPreviewResult result;
    if (baseline.isEmpty())
    {
        result.error = QStringLiteral("没有可用的基准焊道点。");
        return result;
    }
    result.before = baseline;

    struct ArrowBasis { double cx = 0.0, cy = 0.0, cz = 0.0, len = 10.0; };
    const auto computeArrowBasis = [](const QVector<CompPreviewPoint>& points) -> ArrowBasis
    {
        ArrowBasis basis;
        if (points.isEmpty())
        {
            return basis;
        }
        double minv[3] = { 1e300, 1e300, 1e300 };
        double maxv[3] = { -1e300, -1e300, -1e300 };
        double sum[3] = { 0.0, 0.0, 0.0 };
        for (const CompPreviewPoint& point : points)
        {
            const double coord[3] = { point.x, point.y, point.z };
            for (int axis = 0; axis < 3; ++axis)
            {
                sum[axis] += coord[axis];
                minv[axis] = std::min(minv[axis], coord[axis]);
                maxv[axis] = std::max(maxv[axis], coord[axis]);
            }
        }
        const double count = static_cast<double>(points.size());
        basis.cx = sum[0] / count;
        basis.cy = sum[1] / count;
        basis.cz = sum[2] / count;
        const double span = std::max({ maxv[0] - minv[0], maxv[1] - minv[1], maxv[2] - minv[2], 10.0 });
        basis.len = std::clamp(span * 0.15, 8.0, 80.0);
        return basis;
    };
    const auto addArrow = [&result](double ox, double oy, double oz, double vx, double vy, double vz,
        const QString& label, int colorId, bool doubleHeaded)
    {
        CompPreviewArrow arrow;
        arrow.origin[0] = ox; arrow.origin[1] = oy; arrow.origin[2] = oz;
        arrow.vector[0] = vx; arrow.vector[1] = vy; arrow.vector[2] = vz;
        arrow.label = label;
        arrow.colorId = colorId;
        arrow.doubleHeaded = doubleHeaded;
        result.arrows.push_back(arrow);
    };

    if (kind == CompPreviewKind::Seam)
    {
        QVector<WeldPoseFileRecord> records;
        records.reserve(baseline.size());
        for (int index = 0; index < baseline.size(); ++index)
        {
            WeldPoseFileRecord record;
            record.weldIndex = baseline[index].weldIndex;
            record.rawIndex = baseline[index].rawIndex;
            record.point = Eigen::Vector3d(baseline[index].x, baseline[index].y, baseline[index].z);
            record.rx = baseline[index].rx;
            record.ry = baseline[index].ry;
            record.rz = baseline[index].rz;
            record.bx = baseline[index].bx;
            record.by = baseline[index].by;
            record.bz = baseline[index].bz;
            record.segmentKind = baseline[index].segmentKind;
            record.pointType = baseline[index].pointType;
            records.push_back(record);
        }

        QVector<Eigen::Vector3d> basePoints;
        basePoints.reserve(records.size());
        for (const WeldPoseFileRecord& record : records)
        {
            basePoints.push_back(record.point);
        }
        const Eigen::Vector3d seamDir = ResolveOverallHorizontalWeldDirection(basePoints);
        const Eigen::Vector3d gunDir = HorizontalUnitOrZero(Eigen::Vector3d::UnitZ().cross(seamDir));
        result.seamAxis[0] = seamDir.x();
        result.seamAxis[1] = seamDir.y();
        result.seamAxis[2] = seamDir.z();
        result.gunAxis[0] = gunDir.x();
        result.gunAxis[1] = gunDir.y();
        result.gunAxis[2] = gunDir.z();

        // 载入真实焊接预设（含圆弧过渡/裁剪等下发参数），再用对话框当前值覆盖整条焊道统一补偿。
        WeldPosePreset preset = LoadWeldPosePreset(BuildMeasureWeldParamShell(robotName));
        preset.seamComp.weldZComp = edits.weldZComp;
        preset.seamComp.weldGunDirComp = edits.weldGunDirComp;
        preset.seamComp.weldSeamDirComp = edits.weldSeamDirComp;

        // 复用管线真实焊缝补偿平移 + 完整后处理（端点裁剪/自交/拐点恢复/圆弧过渡/加密），贴近 _SeamComp 下发文件。
        WeldSeamCompApplyStats compStats = ApplyWeldSeamCompToWeldPoseRecords(preset, records);
        const QVector<WeldPoseFileRecord> recordsBeforeTrim = records;
        FinalizeSeamCompedWeldPoseRecords(preset, recordsBeforeTrim, records, compStats);
        if (records.isEmpty())
        {
            records = recordsBeforeTrim;  // 后处理裁空则回退显示纯补偿平移结果
        }

        result.after.reserve(records.size());
        for (int index = 0; index < records.size(); ++index)
        {
            CompPreviewPoint point;
            point.x = records[index].point.x();
            point.y = records[index].point.y();
            point.z = records[index].point.z();
            point.segmentKind = records[index].segmentKind;
            point.pointType = records[index].pointType;
            result.after.push_back(point);
        }

        // 方向箭头：质心处 Z向 / 枪反向 / 焊道方向，标正负影响方向。
        const ArrowBasis basis = computeArrowBasis(result.before);
        addArrow(basis.cx, basis.cy, basis.cz, 0.0, 0.0, basis.len, QStringLiteral("Z向+"), 0, true);
        addArrow(basis.cx, basis.cy, basis.cz, gunDir.x() * basis.len, gunDir.y() * basis.len, gunDir.z() * basis.len, QStringLiteral("枪反向+"), 1, true);
        addArrow(basis.cx, basis.cy, basis.cz, seamDir.x() * basis.len, seamDir.y() * basis.len, seamDir.z() * basis.len, QStringLiteral("焊道方向+"), 2, true);
        result.ok = true;
        return result;
    }

    if (kind == CompPreviewKind::Pose)
    {
        // 用对话框当前编辑值构造 4 段姿态补偿槽（段类硬映射 0..3，validReference=true 以参与按姿态匹配）。
        std::vector<WeldPosePreset::PoseCompSlot> poseCompSlots(4);
        static const char* const kSegmentKinds[4] = { "low_platform", "rising_edge", "high_platform", "falling_edge" };
        for (int slotIndex = 0; slotIndex < 4; ++slotIndex)
        {
            poseCompSlots[slotIndex].segmentKind = QString::fromLatin1(kSegmentKinds[slotIndex]);
            poseCompSlots[slotIndex].poseRx = edits.poseRx[slotIndex];
            poseCompSlots[slotIndex].poseRy = edits.poseRy[slotIndex];
            poseCompSlots[slotIndex].poseRz = edits.poseRz[slotIndex];
            poseCompSlots[slotIndex].compX = edits.compX[slotIndex];
            poseCompSlots[slotIndex].compY = edits.compY[slotIndex];
            poseCompSlots[slotIndex].compZ = edits.compZ[slotIndex];
            poseCompSlots[slotIndex].validReference = true;
        }

        QVector<Eigen::Vector3d> basePoints;
        basePoints.reserve(baseline.size());
        QVector<WeldPoseFileRecord> poseRecords;
        poseRecords.reserve(baseline.size());
        for (const CompPreviewPoint& basePoint : baseline)
        {
            WeldPoseFileRecord record;
            record.weldIndex = basePoint.weldIndex;
            record.rawIndex = basePoint.rawIndex;
            record.point = Eigen::Vector3d(basePoint.x, basePoint.y, basePoint.z);
            record.rx = basePoint.rx;
            record.ry = basePoint.ry;
            record.rz = basePoint.rz;
            record.bx = basePoint.bx;
            record.by = basePoint.by;
            record.bz = basePoint.bz;
            record.segmentKind = basePoint.segmentKind;
            record.pointType = basePoint.pointType;
            record.isLapStep = basePoint.isLapStep;
            basePoints.push_back(record.point);
            poseRecords.push_back(record);
        }

        const QVector<PoseCompReferencePose> referencePoses =
            ResolvePoseCompReferencePoses(poseRecords);
        const QVector<Eigen::Vector3d> segmentWeldNormals =
            ResolvePoseCompSegmentWeldNormals(poseRecords, referencePoses);
        const int selectedSlotIndex = std::clamp(edits.posePreviewSegmentIndex, 0, 3);
        QVector<int> preferredSelectedPoints;
        QVector<int> fallbackSelectedPoints;
        for (int index = 0; index < poseRecords.size(); ++index)
        {
            const WeldPoseFileRecord& record = poseRecords[index];
            const PoseCompReferencePose& referencePose = referencePoses[index];
            if (ResolvePoseCompSlotIndex(
                    poseCompSlots,
                    edits.poseMatchMode,
                    edits.poseMatchMaxErrorDeg,
                    referencePose.rx,
                    referencePose.ry,
                    referencePose.rz,
                    record.segmentKind) != selectedSlotIndex)
            {
                continue;
            }
            fallbackSelectedPoints.push_back(index);
            if (DefaultPoseCompSlotIndex(NormalizeWeldSegmentKind(record.segmentKind))
                == selectedSlotIndex)
            {
                preferredSelectedPoints.push_back(index);
            }
        }
        const QVector<int>& selectedPoints = preferredSelectedPoints.isEmpty()
            ? fallbackSelectedPoints
            : preferredSelectedPoints;
        const int selectedRepresentativeIndex = selectedPoints.isEmpty()
            ? -1
            : selectedPoints[selectedPoints.size() / 2];
        Eigen::Vector3d selectedWorldComp = Eigen::Vector3d::Zero();
        if (selectedRepresentativeIndex >= 0)
        {
            const PoseCompReferencePose& referencePose =
                referencePoses[selectedRepresentativeIndex];
            selectedWorldComp = ResolvePoseCompWorldVector(
                poseCompSlots[selectedSlotIndex],
                edits.robotType,
                referencePose.rx,
                referencePose.ry,
                referencePose.rz,
                poseRecords[selectedRepresentativeIndex].segmentKind,
                segmentWeldNormals[selectedRepresentativeIndex]);
        }
        for (int index = 0; index < poseRecords.size(); ++index)
        {
            WeldPoseFileRecord& record = poseRecords[index];
            const PoseCompReferencePose& referencePose = referencePoses[index];
            record.point = ApplyPoseCompToPoint(
                poseCompSlots,
                edits.poseMatchMode,
                edits.poseMatchMaxErrorDeg,
                edits.robotType,
                record.point,
                referencePose.rx,
                referencePose.ry,
                referencePose.rz,
                record.segmentKind,
                segmentWeldNormals[index]);
        }
        ApplyPoseCompSegmentJunctionIntersections(poseRecords);
        StraightenPoseCompPhysicalSegments(poseRecords);
        DensifyWeldPoseRecordsByStep(poseRecords, kPoseCompOutputStepMm);

        result.after.reserve(poseRecords.size());
        for (const WeldPoseFileRecord& record : poseRecords)
        {
            CompPreviewPoint afterPreview;
            afterPreview.x = record.point.x();
            afterPreview.y = record.point.y();
            afterPreview.z = record.point.z();
            afterPreview.rx = record.rx;
            afterPreview.ry = record.ry;
            afterPreview.rz = record.rz;
            afterPreview.segmentKind = record.segmentKind;
            afterPreview.pointType = record.pointType;
            afterPreview.isLapStep = record.isLapStep;
            result.after.push_back(afterPreview);
        }

        const Eigen::Vector3d seamDir = ResolveOverallHorizontalWeldDirection(basePoints);
        const Eigen::Vector3d gunDir = HorizontalUnitOrZero(Eigen::Vector3d::UnitZ().cross(seamDir));
        result.seamAxis[0] = seamDir.x();
        result.seamAxis[1] = seamDir.y();
        result.seamAxis[2] = seamDir.z();
        result.gunAxis[0] = gunDir.x();
        result.gunAxis[1] = gunDir.y();
        result.gunAxis[2] = gunDir.z();

        // 方向箭头：只画当前选中段的实际合成补偿向量；单箭头保留补偿正负。
        const ArrowBasis basis = computeArrowBasis(result.before);
        if (selectedRepresentativeIndex >= 0
            && selectedRepresentativeIndex < baseline.size()
            && selectedWorldComp.norm() > 1e-9)
        {
            const double localAbs[3] = {
                std::abs(edits.compX[selectedSlotIndex]),
                std::abs(edits.compY[selectedSlotIndex]),
                std::abs(edits.compZ[selectedSlotIndex])
            };
            int dominantAxis = 0;
            if (localAbs[1] > localAbs[dominantAxis]) dominantAxis = 1;
            if (localAbs[2] > localAbs[dominantAxis]) dominantAxis = 2;
            const Eigen::Vector3d displayVector = selectedWorldComp.normalized() * basis.len;
            const CompPreviewPoint& origin = baseline[selectedRepresentativeIndex];
            addArrow(
                origin.x,
                origin.y,
                origin.z,
                displayVector.x(),
                displayVector.y(),
                displayVector.z(),
                QStringLiteral("当前段补偿方向"),
                3 + dominantAxis,
                false);
        }
        result.ok = true;
        return result;
    }

    // kind == CompPreviewKind::Corner：拐点补偿（复用公开的 RobotCalculation 拐点补偿，内部按几何重算段类）。
    result.before = baseline;

    const auto typeFromCode = [](int code) -> RobotCalculation::LowerWeldPointType
    {
        switch (code)
        {
        case 1: return RobotCalculation::LowerWeldPointType::Start;
        case 2: return RobotCalculation::LowerWeldPointType::End;
        case 3: return RobotCalculation::LowerWeldPointType::InnerCorner;
        case 4: return RobotCalculation::LowerWeldPointType::OuterCorner;
        case 6: return RobotCalculation::LowerWeldPointType::Noise;
        default: return RobotCalculation::LowerWeldPointType::Normal;
        }
    };

    QVector<RobotCalculation::LowerWeldClassifiedPoint> keyPoints;
    keyPoints.reserve(baseline.size());
    for (int index = 0; index < baseline.size(); ++index)
    {
        RobotCalculation::LowerWeldClassifiedPoint keyPoint;
        keyPoint.index = index;
        keyPoint.point = Eigen::Vector3d(baseline[index].x, baseline[index].y, baseline[index].z);
        keyPoint.type = typeFromCode(baseline[index].typeCode);
        keyPoints.push_back(keyPoint);
    }

    RobotCalculation::LowerWeldFilterParams params;
    params.enableCornerCompensation = edits.cornerEnabled;
    params.risingCornerCompensation.innerToOuterMm = edits.risingInnerToOuter;
    params.risingCornerCompensation.innerToInnerMm = edits.risingInnerToInner;
    params.risingCornerCompensation.outerToOuterMm = edits.risingOuterToOuter;
    params.risingCornerCompensation.outerToInnerMm = edits.risingOuterToInner;
    params.fallingCornerCompensation.innerToOuterMm = edits.fallingInnerToOuter;
    params.fallingCornerCompensation.innerToInnerMm = edits.fallingInnerToInner;
    params.fallingCornerCompensation.outerToOuterMm = edits.fallingOuterToOuter;
    params.fallingCornerCompensation.outerToInnerMm = edits.fallingOuterToInner;

    QVector<RobotCalculation::LowerWeldClassifiedPoint> compensatedKeyPoints;
    RobotCalculation::BuildCornerCompensatedLowerWeldClassification(keyPoints, params, &compensatedKeyPoints);

    if (compensatedKeyPoints.size() == keyPoints.size())
    {
        result.after.reserve(compensatedKeyPoints.size());
        for (int index = 0; index < compensatedKeyPoints.size(); ++index)
        {
            CompPreviewPoint point;
            point.x = compensatedKeyPoints[index].point.x();
            point.y = compensatedKeyPoints[index].point.y();
            point.z = compensatedKeyPoints[index].point.z();
            point.typeCode = baseline[index].typeCode;
            point.pointType = baseline[index].pointType;
            result.after.push_back(point);
        }

        // 方向箭头：每个发生位移的拐点画其移动方向（小位移放大显示）。
        const ArrowBasis basis = computeArrowBasis(result.before);
        for (int index = 0; index < result.before.size() && index < result.after.size(); ++index)
        {
            const double dx = result.after[index].x - result.before[index].x;
            const double dy = result.after[index].y - result.before[index].y;
            const double dz = result.after[index].z - result.before[index].z;
            const double moved = std::sqrt(dx * dx + dy * dy + dz * dz);
            if (moved < 1e-6)
            {
                continue;
            }
            const double shown = std::clamp(moved * 4.0, basis.len * 0.4, basis.len * 1.5);
            const double scale = shown / moved;
            addArrow(result.before[index].x, result.before[index].y, result.before[index].z,
                dx * scale, dy * scale, dz * scale, QStringLiteral("拐点位移"), 6, false);
        }
        result.ok = true;
    }
    else
    {
        // 未启用拐点补偿或补偿值全为零 → 补偿后与补偿前一致。
        result.after = baseline;
        result.ok = true;
        if (!edits.cornerEnabled)
        {
            result.error = QStringLiteral("未启用本组拐点补偿（勾选\"启用本组拐点补偿\"后可见效果）。");
        }
    }
    return result;
}

namespace
{
// 流式抽样加载点文件（index x y z 格式）：按文件大小估算行数定抽样步长，
// 解析点数不超过 maxPoints。用于补偿预览显示完整点云（154MB 级，全量加载会卡死界面）。
bool LoadSampledPointFile(
    const QString& filePath,
    int maxPoints,
    QVector<MeasureThenWeldService::CompPreviewPoint>& points,
    QString& error,
    const MeasureThenWeldService::StopRequestedCallback& stopRequested)
{
    points.clear();
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        error = QString("打开点文件失败：%1").arg(filePath);
        return false;
    }
    // 估算每行约 45 字节，定隔行抽样步长；估偏只影响显示点数，不影响正确性。
    const qint64 estimatedLines = std::max<qint64>(1, file.size() / 45);
    const int stride = static_cast<int>(std::max<qint64>(1, estimatedLines / std::max(1, maxPoints)));

    QTextStream stream(&file);
    stream.setEncoding(QStringConverter::Utf8);
    qint64 lineIndex = 0;
    while (!stream.atEnd())
    {
        if ((lineIndex & 0xff) == 0 && stopRequested && stopRequested())
        {
            points.clear();
            error = QStringLiteral("已取消读取原始点云预览。");
            return false;
        }
        const QString line = stream.readLine();
        if ((lineIndex++ % stride) != 0)
        {
            continue;
        }
        const QString trimmed = line.trimmed();
        if (trimmed.isEmpty())
        {
            continue;
        }
        const QStringList parts = trimmed.contains(',')
            ? trimmed.split(',', Qt::SkipEmptyParts)
            : trimmed.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
        if (parts.size() < 4)
        {
            continue;
        }
        bool xOk = false;
        bool yOk = false;
        bool zOk = false;
        const double x = parts[1].trimmed().toDouble(&xOk);
        const double y = parts[2].trimmed().toDouble(&yOk);
        const double z = parts[3].trimmed().toDouble(&zOk);
        if (!xOk || !yOk || !zOk
            || !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
        {
            continue;
        }
        MeasureThenWeldService::CompPreviewPoint point;
        point.x = x;
        point.y = y;
        point.z = z;
        points.push_back(point);
    }
    return !points.isEmpty();
}
}

RobotCalculation::LowerWeldFilterParams MeasureThenWeldService::BuildTrackFitParamsFromSettings(
    const PointCloudProcessingConfig::Settings& pointCloudSettings,
    RobotCalculation::SampleAxis fallbackSampleAxis)
{
    RobotCalculation::LowerWeldFilterParams params;
    switch (pointCloudSettings.sampleAxisMode)
    {
    case PointCloudProcessingConfig::SampleAxisMode::AxisX:
        params.sampleAxis = RobotCalculation::SampleAxis::AxisX;
        break;
    case PointCloudProcessingConfig::SampleAxisMode::AxisY:
        params.sampleAxis = RobotCalculation::SampleAxis::AxisY;
        break;
    default:
        params.sampleAxis = fallbackSampleAxis;
        break;
    }
    // 方案三（立板投影）已并入方法③做前置提取，不再作为拟合方案映射；旧配置值按旧版几何处理。
    if (pointCloudSettings.featurePointStrategy == PointCloudProcessingConfig::FeaturePointStrategy::RobustSegmentedKeys)
    {
        params.geometryStrategy = RobotCalculation::LowerWeldGeometryStrategy::RobustSegmentedKeys;
    }
    else if (pointCloudSettings.featurePointStrategy == PointCloudProcessingConfig::FeaturePointStrategy::SlopeWaveFiltered)
    {
        params.geometryStrategy = RobotCalculation::LowerWeldGeometryStrategy::SlopeWaveFiltered;
    }
    else
    {
        params.geometryStrategy = RobotCalculation::LowerWeldGeometryStrategy::LegacyGeometry;
    }
    params.zThreshold = pointCloudSettings.cloudZThresholdMm;
    params.zJumpThreshold = pointCloudSettings.cloudZJumpThresholdMm;
    params.zContinuityThreshold = pointCloudSettings.cloudZContinuityThresholdMm;
    params.segmentBreakDistance = pointCloudSettings.cloudSegmentBreakDistanceMm;
    params.keepLongestSegmentOnly = pointCloudSettings.cloudKeepLongestSegmentOnly;
    params.sampleStep = pointCloudSettings.fitSampleStepMm;
    params.searchWindow = pointCloudSettings.fitSearchWindowMm;
    params.piecewiseFitTolerance = pointCloudSettings.fitPiecewiseToleranceMm;
    params.piecewiseMinSegmentPoints = pointCloudSettings.fitPiecewiseMinSegmentPoints;
    params.minPointCount = pointCloudSettings.fitMinPointCount;
    params.smoothRadius = pointCloudSettings.fitSmoothRadius;
    params.azimuthTurnThresholdDeg = pointCloudSettings.fitAzimuthTurnThresholdDeg;
    params.azimuthHeadingWindow = pointCloudSettings.fitAzimuthHeadingWindow;
    params.azimuthNmsSpanMm = pointCloudSettings.fitAzimuthNmsSpanMm;
    params.azimuthStraightenResidualMm = pointCloudSettings.fitAzimuthStraightenResidualMm;
    params.cornerRefineEnable = pointCloudSettings.fitCornerRefineEnable;
    params.azimuthRefineFloorMm = pointCloudSettings.fitAzimuthRefineFloorMm;
    params.cornerRefineOneSidedFrac = pointCloudSettings.fitCornerRefineOneSidedPct / 100.0;
    params.cornerRefineMidMultiple = pointCloudSettings.fitCornerRefineMidMultiple;
    params.cornerRefineEndFrac = pointCloudSettings.fitCornerRefineEndFracPct / 100.0;
    params.cornerPatternRefitEnable = pointCloudSettings.fitCornerPatternRefitEnable;
    params.cornerPlatformMinSegPoints = pointCloudSettings.fitCornerPlatformMinSegPoints;
    params.enableLapMisalignmentSplit = pointCloudSettings.enableLapMisalignmentSplit;
    params.lapStepHeightThresholdMm = pointCloudSettings.lapStepHeightThresholdMm;
    params.lapStepStationWindowMm = pointCloudSettings.lapStepStationWindowMm;
    params.lapStepSideFlatnessMm = pointCloudSettings.lapStepSideFlatnessMm;
    params.lapStepPlatformSlopeMax = pointCloudSettings.lapStepPlatformSlopeMax;
    params.projectionStationWindowMm = pointCloudSettings.projectionStationWindowMm;
    params.projectionTransverseWindowMm = pointCloudSettings.projectionTransverseWindowMm;
    params.projectionZBandBelowMm = pointCloudSettings.projectionZBandBelowMm;
    params.projectionZBandAboveMm = pointCloudSettings.projectionZBandAboveMm;
    params.projectionMaxCandidatePerSeed = pointCloudSettings.projectionMaxCandidatePerSeed;
    params.projectionLayerLowPercent = pointCloudSettings.projectionLayerLowPercent;
    params.projectionLayerHighPercent = pointCloudSettings.projectionLayerHighPercent;
    params.projectionSmoothRadius = pointCloudSettings.projectionSmoothRadius;
    params.useSlopeConsistentCornerFit = pointCloudSettings.slopeConsistentCornerFit;
    params.exportFitDebugCloud = pointCloudSettings.exportFitDebugCloud;
    params.validationAuditOnly =
        pointCloudSettings.validationPolicy == PointCloudProcessingConfig::ValidationPolicy::Audit;
    params.validationCoverageEnabled = pointCloudSettings.validationCoverageEnabled;
    params.validationMinFinitePointCount = pointCloudSettings.validationMinFinitePointCount;
    params.validationMinProjectedSpanMm = pointCloudSettings.validationMinProjectedSpanMm;
    params.validationContinuityEnabled = pointCloudSettings.validationContinuityEnabled;
    params.validationMinStationCoverageRatio = pointCloudSettings.validationMinStationCoverageRatio;
    params.validationMinLongestContinuousRatio = pointCloudSettings.validationMinLongestContinuousRatio;
    params.validationDenoiseRatioEnabled = pointCloudSettings.validationDenoiseRatioEnabled;
    params.validationMaxRejectedRatio = pointCloudSettings.validationMaxRejectedRatio;
    params.validationResidualEnabled = pointCloudSettings.validationResidualEnabled;
    params.validationMaxMedianResidualMm = pointCloudSettings.validationMaxMedianResidualMm;
    params.validationMaxP95ResidualMm = pointCloudSettings.validationMaxP95ResidualMm;
    params.validationResidualInlierThresholdMm = pointCloudSettings.validationResidualInlierThresholdMm;
    params.validationMinResidualInlierRatio = pointCloudSettings.validationMinResidualInlierRatio;
    params.validationKeyPointEnabled = pointCloudSettings.validationKeyPointEnabled;
    params.validationMinKeyPointCount = pointCloudSettings.validationMinKeyPointCount;
    params.validationMinCornerCount = pointCloudSettings.validationMinCornerCount;
    params.validationMinSegmentLengthMm = pointCloudSettings.validationMinSegmentLengthMm;
    params.validationOutputEnabled = pointCloudSettings.validationOutputEnabled;
    params.validationMinOutputPointCount = pointCloudSettings.validationMinOutputPointCount;
    params.validationMinOutputLengthRatio = pointCloudSettings.validationMinOutputLengthRatio;
    params.enableEdgeTruncate = pointCloudSettings.fitEdgeTruncateEnable;
    params.truncateHeadMm = pointCloudSettings.fitTruncateHeadMm;
    params.truncateTailMm = pointCloudSettings.fitTruncateTailMm;
    params.enableEndPeriodCornerRecover = pointCloudSettings.fitEndPeriodRecoverEnable;
    params.endPeriodRatioThreshold = pointCloudSettings.fitEndPeriodRatioThreshold;
    params.endPeriodMinBendDeg = pointCloudSettings.fitEndPeriodMinBendDeg;
    params.enableSameTypeShortCornerMerge = pointCloudSettings.fitSameTypeShortMergeEnable;
    params.endPeriodMergeFrac = pointCloudSettings.fitEndPeriodMergeFrac;
    params.sameTypeShortMinReferenceSegments = pointCloudSettings.fitSameTypeMinReferenceSegments;
    params.sameTypeShortFlatSlope = pointCloudSettings.fitSameTypeFlatSlope;
    params.enablePlatformCornerSnap = pointCloudSettings.fitPlatformSnapEnable;
    params.platformSnapFlatSlope = pointCloudSettings.fitPlatformSnapFlatSlope;
    params.platformSnapMinFrac = pointCloudSettings.fitPlatformSnapMinFrac;
    return params;
}

QString MeasureThenWeldService::MethodBaseTrackFileName(PointCloudProcessingConfig::Mode mode)
{
    switch (mode)
    {
    case PointCloudProcessingConfig::Mode::ExternalCorrugatedSheet:
        return QString::fromLatin1(METHOD_TRACK_SDK_CLASS_FILE_NAME);
    case PointCloudProcessingConfig::Mode::SdkBaseWeldFit:
        return QString::fromLatin1(METHOD_TRACK_SDK_BASE_FILE_NAME);
    case PointCloudProcessingConfig::Mode::CloudFit:
        return QString::fromLatin1(METHOD_TRACK_POINT_BASE_FILE_NAME);
    case PointCloudProcessingConfig::Mode::LegacyLaserPath:
    default:
        return QString::fromLatin1(METHOD_TRACK_POINT_LASER_FILE_NAME);
    }
}

bool MeasureThenWeldService::LoadCompPreviewRawCloud(
    const QString& laserDir,
    QVector<CompPreviewPoint>& points,
    QString& error,
    QString* sourceDescription,
    const StopRequestedCallback& stopRequested) const
{
    points.clear();
    if (sourceDescription != nullptr)
    {
        sourceDescription->clear();
    }
    QDir dir(laserDir);
    if (!dir.exists())
    {
        error = QString("目录不存在：%1").arg(laserDir);
        return false;
    }

    // "原始数据"显示当前方法自己的基础焊道文件（处理成功时落盘）；
    // 未生成（该目录还没按当前方法处理过）时回退相机目标点轨迹。
    const PointCloudProcessingConfig::Settings settings = PointCloudProcessingConfig::Load();
    const QString methodFileName = MethodBaseTrackFileName(settings.mode);
    const QString methodFilePath = dir.filePath(methodFileName);
    QString methodError;
    if (QFileInfo::exists(methodFilePath)
        && LoadSampledPointFile(
            methodFilePath, std::numeric_limits<int>::max(), points, methodError, stopRequested))
    {
        if (sourceDescription != nullptr)
        {
            *sourceDescription = QString("%1（%2）")
                .arg(PointCloudProcessingConfig::ModeDisplayName(settings.mode))
                .arg(methodFileName);
        }
        return true;
    }

    QVector<RobotCalculation::IndexedPoint3D> rawPoints;
    if (!RobotDataHelper::LoadIndexedPoint3DFile(
            dir.filePath(RAW_LASER_FILE_NAME), rawPoints, &error, stopRequested))
    {
        return false;
    }
    points.reserve(rawPoints.size());
    int rawIndex = 0;
    for (const RobotCalculation::IndexedPoint3D& raw : rawPoints)
    {
        if ((rawIndex++ & 0xff) == 0 && stopRequested && stopRequested())
        {
            points.clear();
            error = QStringLiteral("已取消生成原始点云预览。");
            return false;
        }
        CompPreviewPoint point;
        point.x = raw.point.x();
        point.y = raw.point.y();
        point.z = raw.point.z();
        points.push_back(point);
    }
    if (points.isEmpty())
    {
        error = QString("未从 %1 解析到原始点。").arg(RAW_LASER_FILE_NAME);
        return false;
    }
    if (sourceDescription != nullptr)
    {
        *sourceDescription = QStringLiteral("相机目标点轨迹（该方法基础焊道未生成）");
    }
    return true;
}

MeasureThenWeldService::CompPreviewStages MeasureThenWeldService::ComputeCompPreviewStages(
    const QString& robotName,
    const QVector<CompPreviewPoint>& baseline,
    const CompPreviewEditValues& currentEdits,
    const CompPreviewEditValues& savedEdits,
    bool showPoseSelection,
    const StopRequestedCallback& stopRequested) const
{
    CompPreviewStages stages;
    const auto canceled = [&stopRequested]()
    {
        return stopRequested && stopRequested();
    };
    if (canceled())
    {
        stages.error = QStringLiteral("已取消计算补偿预览。");
        return stages;
    }
    if (baseline.isEmpty())
    {
        stages.error = QStringLiteral("没有可用的基准焊道点。");
        return stages;
    }

    static const char* const kSegmentKinds[4] = { "low_platform", "rising_edge", "high_platform", "falling_edge" };
    const auto buildPoseSlots = [&](const CompPreviewEditValues& edits)
    {
        // 注意：局部变量不能叫 slots（Qt 宏）。
        std::vector<WeldPosePreset::PoseCompSlot> poseSlots(4);
        for (int slotIndex = 0; slotIndex < 4; ++slotIndex)
        {
            poseSlots[slotIndex].segmentKind = QString::fromLatin1(kSegmentKinds[slotIndex]);
            poseSlots[slotIndex].poseRx = edits.poseRx[slotIndex];
            poseSlots[slotIndex].poseRy = edits.poseRy[slotIndex];
            poseSlots[slotIndex].poseRz = edits.poseRz[slotIndex];
            poseSlots[slotIndex].compX = edits.compX[slotIndex];
            poseSlots[slotIndex].compY = edits.compY[slotIndex];
            poseSlots[slotIndex].compZ = edits.compZ[slotIndex];
            poseSlots[slotIndex].validReference = true;
        }
        return poseSlots;
    };
    const std::vector<WeldPosePreset::PoseCompSlot> currentPoseSlots = buildPoseSlots(currentEdits);
    const std::vector<WeldPosePreset::PoseCompSlot> savedPoseSlots = buildPoseSlots(savedEdits);
    const int selectedPoseSegmentIndex = std::clamp(currentEdits.posePreviewSegmentIndex, 0, 3);
    stages.selectedPoseSegmentIndex = selectedPoseSegmentIndex;
    stages.selectedPoseSegmentKind = QString::fromLatin1(kSegmentKinds[selectedPoseSegmentIndex]);
    stages.selectedPoseCompLocal[0] = currentEdits.compX[selectedPoseSegmentIndex];
    stages.selectedPoseCompLocal[1] = currentEdits.compY[selectedPoseSegmentIndex];
    stages.selectedPoseCompLocal[2] = currentEdits.compZ[selectedPoseSegmentIndex];
    int selectedPoseRepresentativeIndex = -1;

    // 阶段「姿态补偿」：基准(_WeldPose_2mm)已烘焙扫描时保存的姿态补偿，
    // 按 delta = 当前补偿位移 − 已保存补偿位移 叠加，当前=已保存时与文件一致；
    // 位移纯加性，等价于用当前值重跑姿态生成。
    QVector<WeldPoseFileRecord> records;
    records.reserve(baseline.size());
    int baseIndex = 0;
    for (const CompPreviewPoint& base : baseline)
    {
        if ((baseIndex++ & 0xff) == 0 && canceled())
        {
            stages.error = QStringLiteral("已取消计算姿态补偿预览。");
            return stages;
        }
        WeldPoseFileRecord record;
        record.weldIndex = base.weldIndex;
        record.rawIndex = base.rawIndex;
        record.point = Eigen::Vector3d(base.x, base.y, base.z);
        record.rx = base.rx;
        record.ry = base.ry;
        record.rz = base.rz;
        record.bx = base.bx;
        record.by = base.by;
        record.bz = base.bz;
        record.segmentKind = base.segmentKind;
        record.pointType = base.pointType;
        record.isLapStep = base.isLapStep;
        records.push_back(record);
    }

    const QVector<PoseCompReferencePose> referencePoses =
        ResolvePoseCompReferencePoses(records);
    const QVector<Eigen::Vector3d> segmentWeldNormals =
        ResolvePoseCompSegmentWeldNormals(records, referencePoses);

    // 当前选中段的方向提示必须走与生产补偿相同的槽位匹配和固定焊道局部基准。
    // 优先显示同段类的匹配点；按姿态匹配时若该槽被其他段复用，再回退到任意匹配点。
    QVector<int> preferredSelectedPosePoints;
    QVector<int> fallbackSelectedPosePoints;
    for (int index = 0; index < records.size(); ++index)
    {
        const WeldPoseFileRecord& record = records[index];
        const PoseCompReferencePose& referencePose = referencePoses[index];
        const int matchedSlotIndex = ResolvePoseCompSlotIndex(
            currentPoseSlots,
            currentEdits.poseMatchMode,
            currentEdits.poseMatchMaxErrorDeg,
            referencePose.rx,
            referencePose.ry,
            referencePose.rz,
            record.segmentKind);
        if (matchedSlotIndex != selectedPoseSegmentIndex)
        {
            continue;
        }
        fallbackSelectedPosePoints.push_back(index);
        if (DefaultPoseCompSlotIndex(NormalizeWeldSegmentKind(record.segmentKind))
            == selectedPoseSegmentIndex)
        {
            preferredSelectedPosePoints.push_back(index);
        }
    }
    const QVector<int>& selectedPosePoints = preferredSelectedPosePoints.isEmpty()
        ? fallbackSelectedPosePoints
        : preferredSelectedPosePoints;
    if (!selectedPosePoints.isEmpty())
    {
        selectedPoseRepresentativeIndex = selectedPosePoints[selectedPosePoints.size() / 2];
        stages.selectedPoseSegmentMatched = true;
        const PoseCompReferencePose& referencePose =
            referencePoses[selectedPoseRepresentativeIndex];
        const Eigen::Vector3d worldComp = ResolvePoseCompWorldVector(
            currentPoseSlots[selectedPoseSegmentIndex],
            currentEdits.robotType,
            referencePose.rx,
            referencePose.ry,
            referencePose.rz,
            records[selectedPoseRepresentativeIndex].segmentKind,
            segmentWeldNormals[selectedPoseRepresentativeIndex]);
        stages.selectedPoseCompWorld[0] = worldComp.x();
        stages.selectedPoseCompWorld[1] = worldComp.y();
        stages.selectedPoseCompWorld[2] = worldComp.z();
        stages.selectedPoseDirectionValid = worldComp.norm() > 1e-9;
    }

    for (int index = 0; index < records.size(); ++index)
    {
        if ((index & 0xff) == 0 && canceled())
        {
            stages.error = QStringLiteral("已取消计算姿态补偿预览。");
            return stages;
        }
        WeldPoseFileRecord& record = records[index];
        const PoseCompReferencePose& referencePose = referencePoses[index];
        const Eigen::Vector3d basePoint = record.point;
        const Eigen::Vector3d withCurrent = ApplyPoseCompToPoint(
            currentPoseSlots, currentEdits.poseMatchMode, currentEdits.poseMatchMaxErrorDeg,
            currentEdits.robotType, basePoint,
            referencePose.rx, referencePose.ry, referencePose.rz, record.segmentKind,
            segmentWeldNormals[index]);
        const Eigen::Vector3d withSaved = ApplyPoseCompToPoint(
            savedPoseSlots, savedEdits.poseMatchMode, savedEdits.poseMatchMaxErrorDeg,
            savedEdits.robotType, basePoint,
            referencePose.rx, referencePose.ry, referencePose.rz, record.segmentKind,
            segmentWeldNormals[index]);
        record.point = basePoint + (withCurrent - basePoint) - (withSaved - basePoint);
    }
    ApplyPoseCompSegmentJunctionIntersections(records);
    StraightenPoseCompPhysicalSegments(records);
    DensifyWeldPoseRecordsByStep(records, kPoseCompOutputStepMm);

    const auto snapshotRecords = [](const QVector<WeldPoseFileRecord>& sourceRecords)
    {
        QVector<CompPreviewPoint> out;
        out.reserve(sourceRecords.size());
        for (const WeldPoseFileRecord& record : sourceRecords)
        {
            CompPreviewPoint point;
            point.x = record.point.x();
            point.y = record.point.y();
            point.z = record.point.z();
            point.rx = record.rx;
            point.ry = record.ry;
            point.rz = record.rz;
            point.bx = record.bx;
            point.by = record.by;
            point.bz = record.bz;
            point.weldIndex = record.weldIndex;
            point.rawIndex = record.rawIndex;
            point.segmentKind = record.segmentKind;
            point.pointType = record.pointType;
            point.isLapStep = record.isLapStep;
            out.push_back(point);
        }
        return out;
    };
    stages.poseComp = snapshotRecords(records);

    QVector<Eigen::Vector3d> basePoints;
    basePoints.reserve(records.size());
    for (const WeldPoseFileRecord& record : records)
    {
        basePoints.push_back(record.point);
    }
    const Eigen::Vector3d seamDir = ResolveOverallHorizontalWeldDirection(basePoints);
    const Eigen::Vector3d gunDir = HorizontalUnitOrZero(Eigen::Vector3d::UnitZ().cross(seamDir));
    stages.selectedSeamCompLocal[0] = currentEdits.weldZComp;
    stages.selectedSeamCompLocal[1] = currentEdits.weldGunDirComp;
    stages.selectedSeamCompLocal[2] = currentEdits.weldSeamDirComp;
    const Eigen::Vector3d selectedSeamWorldComp =
        Eigen::Vector3d::UnitZ() * currentEdits.weldZComp
        + gunDir * currentEdits.weldGunDirComp
        + seamDir * currentEdits.weldSeamDirComp;
    stages.selectedSeamCompWorld[0] = selectedSeamWorldComp.x();
    stages.selectedSeamCompWorld[1] = selectedSeamWorldComp.y();
    stages.selectedSeamCompWorld[2] = selectedSeamWorldComp.z();
    stages.selectedSeamDirectionValid = selectedSeamWorldComp.norm() > 1e-9;

    // 阶段「焊道补偿」：真实预设 + 当前整条焊道统一补偿覆盖。
    const T_PRECISE_MEASURE_PARAM measureParam = BuildMeasureWeldParamShell(robotName);
    WeldPosePreset preset = LoadWeldPosePreset(measureParam);
    // 工艺区域试调覆盖（仅预览联动，不落盘）：圆弧过渡启用/半径 + 实际焊道点间距。
    if (currentEdits.processOverrideValid)
    {
        preset.cornerArcRadiusMm = (currentEdits.arcEnabled && currentEdits.arcRadiusMm > 0.0)
            ? std::max(2.0, currentEdits.arcRadiusMm)
            : 0.0;
        preset.finalWeldStepFromProcessMm = currentEdits.processFinalStepMm;
    }
    preset.seamComp.weldZComp = currentEdits.weldZComp;
    preset.seamComp.weldGunDirComp = currentEdits.weldGunDirComp;
    preset.seamComp.weldSeamDirComp = currentEdits.weldSeamDirComp;
    WeldSeamCompApplyStats compStats = ApplyWeldSeamCompToWeldPoseRecords(preset, records);
    if (canceled())
    {
        stages.error = QStringLiteral("已取消计算焊道补偿预览。");
        return stages;
    }
    stages.seamComp = snapshotRecords(records);

    // 阶段「圆弧过渡」：完整后处理（端点/自交裁剪、拐点恢复、加密、显式圆弧过渡、锐角裁剪、重编号）。
    const QVector<WeldPoseFileRecord> recordsBeforeTrim = records;
    FinalizeSeamCompedWeldPoseRecords(preset, recordsBeforeTrim, records, compStats);
    if (canceled())
    {
        stages.error = QStringLiteral("已取消计算圆弧过渡预览。");
        return stages;
    }
    if (records.isEmpty())
    {
        records = recordsBeforeTrim;  // 后处理裁空则回退显示纯补偿平移结果
    }
    stages.arc = snapshotRecords(records);

    // 阶段「实际焊道」：按点间距最终抽样（首尾+拐点必留，沿弧长≥间距取点），
    // 复用下发管线同一个抽样函数 = 机器人逐点执行的轨迹。
    const double actualStepMm = preset.finalWeldStepFromProcessMm > 0.0
        ? preset.finalWeldStepFromProcessMm
        : measureParam.dFinalWeldTrajectoryStepMm;
    const QVector<WeldPoseFileRecord> actualRecords =
        SampleFinalWeldTrajectoryRecords(records, NormalizeFinalWeldTrajectorySampleStepMm(actualStepMm),
            currentEdits.keepAnchorsOnly);
    stages.actual = snapshotRecords(actualRecords);

    // 方向箭头：质心 + 自适应长度。
    double sum[3] = { 0.0, 0.0, 0.0 };
    double minv[3] = { 1e300, 1e300, 1e300 };
    double maxv[3] = { -1e300, -1e300, -1e300 };
    for (const CompPreviewPoint& point : stages.poseComp)
    {
        const double coord[3] = { point.x, point.y, point.z };
        for (int axis = 0; axis < 3; ++axis)
        {
            sum[axis] += coord[axis];
            minv[axis] = std::min(minv[axis], coord[axis]);
            maxv[axis] = std::max(maxv[axis], coord[axis]);
        }
    }
    const double count = static_cast<double>(stages.poseComp.size());
    const double cx = sum[0] / count;
    const double cy = sum[1] / count;
    const double cz = sum[2] / count;
    const double span = std::max({ maxv[0] - minv[0], maxv[1] - minv[1], maxv[2] - minv[2], 10.0 });
    const double arrowLen = std::clamp(span * 0.15, 8.0, 80.0);
    const auto addArrow = [&stages](double ox, double oy, double oz, double vx, double vy, double vz,
        const QString& label, int colorId, bool doubleHeaded)
    {
        CompPreviewArrow arrow;
        arrow.origin[0] = ox; arrow.origin[1] = oy; arrow.origin[2] = oz;
        arrow.vector[0] = vx; arrow.vector[1] = vy; arrow.vector[2] = vz;
        arrow.label = label;
        arrow.colorId = colorId;
        arrow.doubleHeaded = doubleHeaded;
        stages.arrows.push_back(arrow);
    };
    addArrow(cx, cy, cz, 0.0, 0.0, arrowLen, QStringLiteral("Z向+"), 0, true);
    addArrow(cx, cy, cz, gunDir.x() * arrowLen, gunDir.y() * arrowLen, gunDir.z() * arrowLen, QStringLiteral("枪反向+"), 1, true);
    addArrow(cx, cy, cz, seamDir.x() * arrowLen, seamDir.y() * arrowLen, seamDir.z() * arrowLen, QStringLiteral("焊道方向+"), 2, true);

    if (showPoseSelection
        && stages.selectedPoseDirectionValid
        && selectedPoseRepresentativeIndex >= 0
        && selectedPoseRepresentativeIndex < baseline.size())
    {
        const Eigen::Vector3d worldComp(
            stages.selectedPoseCompWorld[0],
            stages.selectedPoseCompWorld[1],
            stages.selectedPoseCompWorld[2]);
        const Eigen::Vector3d displayVector = worldComp.normalized() * arrowLen;
        const double localAbs[3] = {
            std::abs(stages.selectedPoseCompLocal[0]),
            std::abs(stages.selectedPoseCompLocal[1]),
            std::abs(stages.selectedPoseCompLocal[2])
        };
        int dominantAxis = 0;
        if (localAbs[1] > localAbs[dominantAxis]) dominantAxis = 1;
        if (localAbs[2] > localAbs[dominantAxis]) dominantAxis = 2;
        const CompPreviewPoint& origin = baseline[selectedPoseRepresentativeIndex];
        addArrow(
            origin.x,
            origin.y,
            origin.z,
            displayVector.x(),
            displayVector.y(),
            displayVector.z(),
            QStringLiteral("当前段补偿方向"),
            3 + dominantAxis,
            false);
    }
    else if (!showPoseSelection
        && stages.selectedSeamDirectionValid
        && !stages.poseComp.isEmpty())
    {
        const Eigen::Vector3d worldComp(
            stages.selectedSeamCompWorld[0],
            stages.selectedSeamCompWorld[1],
            stages.selectedSeamCompWorld[2]);
        const Eigen::Vector3d displayVector = worldComp.normalized() * arrowLen;
        const double localAbs[3] = {
            std::abs(stages.selectedSeamCompLocal[0]),
            std::abs(stages.selectedSeamCompLocal[1]),
            std::abs(stages.selectedSeamCompLocal[2])
        };
        int dominantAxis = 0;
        if (localAbs[1] > localAbs[dominantAxis]) dominantAxis = 1;
        if (localAbs[2] > localAbs[dominantAxis]) dominantAxis = 2;
        const CompPreviewPoint& origin = stages.poseComp[stages.poseComp.size() / 2];
        addArrow(
            origin.x,
            origin.y,
            origin.z,
            displayVector.x(),
            displayVector.y(),
            displayVector.z(),
            QStringLiteral("当前焊道补偿方向"),
            7 + dominantAxis,
            false);
    }

    stages.ok = true;
    return stages;
}
