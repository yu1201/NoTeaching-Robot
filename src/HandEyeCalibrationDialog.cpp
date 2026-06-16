#include "HandEyeCalibrationDialog.h"

#include "CameraFrameCache.h"
#include "FANUCRobotDriver.h"
#include "HandEyeMatrixDialog.h"
#include "RobotCalculation.h"
#include "RobotDataHelper.h"
#include "RobotDriverAdaptor.h"
#include "RobotLog.h"
#include "RobotMessage.h"
#include "RobotPoseTransform.h"
#include "WindowStyleHelper.h"
#include "groove/framebuffer.h"

#include <QApplication>
#include <QCloseEvent>
#include <QDateTime>
#include <QDir>
#include <QDoubleValidator>
#include <QDoubleSpinBox>
#include <QDebug>
#include <QEventLoop>
#include <QFile>
#include <QFileInfo>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QSizePolicy>
#include <QPointer>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QStringList>
#include <QTabBar>
#include <QTabWidget>
#include <QTextDocument>
#include <QTextStream>
#include <QVBoxLayout>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <thread>
#include <vector>

namespace
{
constexpr int kHandEyeAutoTargetStep = 10;
constexpr int kHandEyeAutoFirstSampleStep = 11;
constexpr int kHandEyeAutoLastSampleStep = 16;
constexpr int kHandEyeAutoDoneStep = 999;
constexpr int kHandEyeAutoAbortValue = -1;
constexpr double kHandEyeAutoMoveSpeedMmPerMin = 500.0;
constexpr int kHandEyeAutoMoveDoneTimeoutMs = 180000;
constexpr int kHandEyeAutoNoMotionTimeoutMs = 8000;
constexpr double kHandEyeAutoMoveDetectMm = 0.05;
constexpr double kHandEyeAutoRotateDetectDeg = 0.05;
constexpr double kHandEyeAutoArrivePositionToleranceMm = 2.0;
constexpr double kHandEyeAutoArriveRotationToleranceDeg = 3.0;
constexpr const char* kHandEyeAutoProgramName = "FANUC_HECALIB";
constexpr int kHandEyeRobotCheckStartPrIndex = 79;
constexpr int kHandEyeRobotCheckPrIndex = 80;
constexpr int kHandEyeRobotCheckStateReg = 92;
constexpr const char* kHandEyeRobotCheckProgramName = "FANUC_HECHECK";
constexpr int kCameraTimestampCheckDurationMs = 5000;

QString FormatDouble(double value, int precision = 6)
{
    return QString::number(value, 'f', precision);
}

void ApplyHandEyeNumericEdit(QLineEdit* edit)
{
    if (edit == nullptr)
    {
        return;
    }
    QDoubleValidator* validator = new QDoubleValidator(-1.0e12, 1.0e12, 6, edit);
    validator->setNotation(QDoubleValidator::StandardNotation);
    edit->setValidator(validator);
    edit->setProperty("touchKeyboardLayout", QStringLiteral("numeric"));
    edit->setInputMethodHints(Qt::ImhFormattedNumbersOnly);
    edit->setAlignment(Qt::AlignRight);
}

QString FindHandEyeAutoProgramPath()
{
    return RobotDataHelper::FindProjectFilePath("SDK/FANUC/FANUC_HECALIB.ls");
}

QString FindHandEyeRobotCheckProgramPath()
{
    return RobotDataHelper::FindProjectFilePath("SDK/FANUC/FANUC_HECHECK.ls");
}

QString BuildHandEyeCalibrationReportPath(const QString& robotName, const QString& cameraSection)
{
    const QFileInfo calibrationInfo(GetHandEyeCalibrationIniPath(robotName, cameraSection));
    return QDir::toNativeSeparators(calibrationInfo.dir().filePath(QString("HandEyeCalibrationReport_%1.txt").arg(cameraSection)));
}

QString BuildCameraTimestampCheckPath(const QString& robotName, const QString& cameraSection)
{
    const QFileInfo calibrationInfo(GetHandEyeCalibrationIniPath(robotName, cameraSection));
    const QString stamp = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
    return QDir::toNativeSeparators(calibrationInfo.dir().filePath(
        QString("CameraTimestampCheck_%1_%2.csv").arg(cameraSection, stamp)));
}

QString BuildHandEyeCalibrationDiagnosticPath(const QString& robotName, const QString& cameraSection, const QString& sceneName)
{
    const QFileInfo calibrationInfo(GetHandEyeCalibrationIniPath(robotName, cameraSection));
    const QString stamp = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
    const QString safeSceneName = sceneName.isEmpty() ? QString("Check") : sceneName;
    return QDir::toNativeSeparators(calibrationInfo.dir().filePath(
        QString("HandEyeCalibrationDiagnostic_%1_%2_%3.txt").arg(cameraSection, safeSceneName, stamp)));
}

QString FormatPoseSummary(const T_ROBOT_COORS& pose)
{
    return QString("X=%1 Y=%2 Z=%3 RX=%4 RY=%5 RZ=%6 BX=%7 BY=%8 BZ=%9")
        .arg(pose.dX, 0, 'f', 6)
        .arg(pose.dY, 0, 'f', 6)
        .arg(pose.dZ, 0, 'f', 6)
        .arg(pose.dRX, 0, 'f', 6)
        .arg(pose.dRY, 0, 'f', 6)
        .arg(pose.dRZ, 0, 'f', 6)
        .arg(pose.dBX, 0, 'f', 6)
        .arg(pose.dBY, 0, 'f', 6)
        .arg(pose.dBZ, 0, 'f', 6);
}

QString FormatVectorSummary(const Eigen::Vector3d& point, int precision = 6)
{
    return QString("X=%1 Y=%2 Z=%3")
        .arg(point.x(), 0, 'f', precision)
        .arg(point.y(), 0, 'f', precision)
        .arg(point.z(), 0, 'f', precision);
}

Eigen::Vector3d PosePositionVector(const T_ROBOT_COORS& pose)
{
    return Eigen::Vector3d(pose.dX, pose.dY, pose.dZ);
}

bool IsFiniteDiagnosticVector(const Eigen::Vector3d& value)
{
    return std::isfinite(value.x()) && std::isfinite(value.y()) && std::isfinite(value.z());
}

struct HandEyeDiagnosticSummary
{
    int validSampleCount = 0;
    int validPairCount = 0;
    double sampleRmseMm = 0.0;
    double sampleMaxErrorMm = 0.0;
    double pairMeanDiffMm = 0.0;
    double pairMaxAbsDiffMm = 0.0;
};

QString FormatHandEyeDiagnosticSummary(const HandEyeDiagnosticSummary& summary)
{
    return QString("有效样本=%1，样本RMSE=%2 mm，样本最大误差=%3 mm，距离对=%4，距离差平均=%5 mm，距离差最大绝对值=%6 mm")
        .arg(summary.validSampleCount)
        .arg(summary.sampleRmseMm, 0, 'f', 3)
        .arg(summary.sampleMaxErrorMm, 0, 'f', 3)
        .arg(summary.validPairCount)
        .arg(summary.pairMeanDiffMm, 0, 'f', 3)
        .arg(summary.pairMaxAbsDiffMm, 0, 'f', 3);
}

QStringList BuildHandEyeDiagnosticLines(
    const HandEyeCalibrationConfig& calibration,
    const HandEyeMatrixConfig& matrix,
    HandEyeDiagnosticSummary* summaryOut = nullptr)
{
    HandEyeDiagnosticSummary summary;
    QStringList lines;
    const Eigen::Vector3d tcpWorld = PosePositionVector(calibration.tcpPoint);

    struct DiagnosticSample
    {
        int index = 0;
        Eigen::Vector3d cameraPoint = Eigen::Vector3d::Zero();
        Eigen::Vector3d robotLocalPoint = Eigen::Vector3d::Zero();
    };
    std::vector<DiagnosticSample> validSamples;

    lines << "[HandEyeDiagnostics]";
    lines << QString("TcpWorld %1").arg(FormatVectorSummary(tcpWorld));
    lines << "";
    lines << "[SampleErrors]";

    double sampleErrorSquareSum = 0.0;
    for (int index = 0; index < calibration.samples.size(); ++index)
    {
        const HandEyeCalibrationSample& sample = calibration.samples[index];
        if (!sample.valid)
        {
            lines << QString("Sample %1 Valid=0 Skip=not_collected").arg(index + 1);
            continue;
        }
        if (!IsFiniteDiagnosticVector(sample.cameraPoint))
        {
            lines << QString("Sample %1 Valid=1 Skip=invalid_camera_point Camera %2")
                .arg(index + 1)
                .arg(FormatVectorSummary(sample.cameraPoint));
            continue;
        }

        const Eigen::Matrix3d robotRotation =
            RobotPoseTransform::RotationFromPose(sample.robotPose, matrix.robotType);
        const Eigen::Vector3d robotWorld = PosePositionVector(sample.robotPose);
        const Eigen::Vector3d robotLocalPoint = robotRotation.transpose() * (tcpWorld - robotWorld);
        const Eigen::Vector3d matrixLocalPoint = matrix.rotation * sample.cameraPoint + matrix.translation;
        const Eigen::Vector3d worldTargetPoint = robotRotation * matrixLocalPoint + robotWorld;
        const Eigen::Vector3d localError = matrixLocalPoint - robotLocalPoint;
        const Eigen::Vector3d worldError = worldTargetPoint - tcpWorld;
        const double errorNorm = worldError.norm();

        if (!IsFiniteDiagnosticVector(robotLocalPoint)
            || !IsFiniteDiagnosticVector(matrixLocalPoint)
            || !IsFiniteDiagnosticVector(worldTargetPoint)
            || !std::isfinite(errorNorm))
        {
            lines << QString("Sample %1 Valid=1 Skip=invalid_calculation Robot %2 Camera %3")
                .arg(index + 1)
                .arg(FormatPoseSummary(sample.robotPose))
                .arg(FormatVectorSummary(sample.cameraPoint));
            continue;
        }

        ++summary.validSampleCount;
        sampleErrorSquareSum += errorNorm * errorNorm;
        summary.sampleMaxErrorMm = std::max(summary.sampleMaxErrorMm, errorNorm);
        validSamples.push_back({ index + 1, sample.cameraPoint, robotLocalPoint });

        lines << QString("Sample %1").arg(index + 1);
        lines << QString("  RobotPose %1").arg(FormatPoseSummary(sample.robotPose));
        lines << QString("  CameraPoint %1").arg(FormatVectorSummary(sample.cameraPoint));
        lines << QString("  RobotLocalExpected %1").arg(FormatVectorSummary(robotLocalPoint));
        lines << QString("  MatrixLocalCalculated %1").arg(FormatVectorSummary(matrixLocalPoint));
        lines << QString("  LocalError %1 Norm=%2")
            .arg(FormatVectorSummary(localError))
            .arg(localError.norm(), 0, 'f', 6);
        lines << QString("  WorldTargetCalculated %1").arg(FormatVectorSummary(worldTargetPoint));
        lines << QString("  TcpWorldError %1 Norm=%2")
            .arg(FormatVectorSummary(worldError))
            .arg(errorNorm, 0, 'f', 6);
    }

    if (summary.validSampleCount > 0)
    {
        summary.sampleRmseMm = std::sqrt(sampleErrorSquareSum / static_cast<double>(summary.validSampleCount));
    }

    lines << "";
    lines << "[PairDistanceCheck]";
    double pairDiffSum = 0.0;
    for (std::size_t left = 0; left < validSamples.size(); ++left)
    {
        for (std::size_t right = left + 1; right < validSamples.size(); ++right)
        {
            const double cameraDistance = (validSamples[right].cameraPoint - validSamples[left].cameraPoint).norm();
            const double robotLocalDistance = (validSamples[right].robotLocalPoint - validSamples[left].robotLocalPoint).norm();
            const double diff = cameraDistance - robotLocalDistance;
            ++summary.validPairCount;
            pairDiffSum += diff;
            summary.pairMaxAbsDiffMm = std::max(summary.pairMaxAbsDiffMm, std::abs(diff));

            lines << QString("Pair %1-%2 CameraDist=%3 RobotLocalDist=%4 Diff(Camera-RobotLocal)=%5")
                .arg(validSamples[left].index)
                .arg(validSamples[right].index)
                .arg(cameraDistance, 0, 'f', 6)
                .arg(robotLocalDistance, 0, 'f', 6)
                .arg(diff, 0, 'f', 6);
        }
    }
    if (summary.validPairCount > 0)
    {
        summary.pairMeanDiffMm = pairDiffSum / static_cast<double>(summary.validPairCount);
    }

    lines << "";
    lines << "[Summary]";
    lines << FormatHandEyeDiagnosticSummary(summary);
    if (summary.sampleRmseMm > 2.0 || summary.pairMaxAbsDiffMm > 2.0)
    {
        lines << "Warning 标定数据刚体一致性偏差较大；优先检查相机三维标定/单位、采样点是否同一物理TCP、机器人姿态/工具号和相机缓存帧。";
    }

    if (summaryOut != nullptr)
    {
        *summaryOut = summary;
    }
    return lines;
}

bool SaveHandEyeDiagnosticFile(
    const QString& robotName,
    const QString& cameraSection,
    const QString& sceneName,
    const HandEyeCalibrationConfig& calibration,
    const HandEyeMatrixConfig& matrix,
    QString* filePathOut = nullptr,
    QString* summaryTextOut = nullptr,
    QString* error = nullptr)
{
    HandEyeDiagnosticSummary summary;
    const QStringList lines = BuildHandEyeDiagnosticLines(calibration, matrix, &summary);
    const QString filePath = BuildHandEyeCalibrationDiagnosticPath(robotName, cameraSection, sceneName);
    if (filePathOut != nullptr)
    {
        *filePathOut = filePath;
    }
    if (summaryTextOut != nullptr)
    {
        *summaryTextOut = FormatHandEyeDiagnosticSummary(summary);
    }
    return RobotDataHelper::SaveTextFileLines(filePath, lines, error);
}

QString FormatAutoCalibrationState(int state)
{
    if (state == 0)
    {
        return "自动标定状态：待启动，已完成 0/6 组";
    }
    if (state == kHandEyeAutoTargetStep)
    {
        return "自动标定状态：当前停在位置变量[10] / 固定标定目标点，已完成 0/6 组";
    }
    if (state >= kHandEyeAutoFirstSampleStep && state <= kHandEyeAutoLastSampleStep)
    {
        const int sampleIndex = state - kHandEyeAutoFirstSampleStep + 1;
        const int completedCount = std::max(0, sampleIndex - 1);
        return QString("自动标定状态：当前停在位置变量[%1] / 第 %2 组采样，已完成 %3/6 组")
            .arg(state)
            .arg(sampleIndex)
            .arg(completedCount);
    }
    if (state == kHandEyeAutoDoneStep)
    {
        return "自动标定状态：采样完成，已完成 6/6 组，正在计算矩阵";
    }
    if (state == kHandEyeAutoAbortValue)
    {
        return "自动标定状态：机器人侧已中止";
    }
    return QString("自动标定状态：当前步骤=%1").arg(state);
}

QString DescribeAutoCalibrationPoint(int step)
{
    if (step == kHandEyeAutoTargetStep)
    {
        return "位置变量[10] 固定标定目标点";
    }
    if (step >= kHandEyeAutoFirstSampleStep && step <= kHandEyeAutoLastSampleStep)
    {
        return QString("位置变量[%1] 第 %2 组采样点")
            .arg(step)
            .arg(step - kHandEyeAutoFirstSampleStep + 1);
    }
    return QString("位置变量[%1]").arg(step);
}

struct HandEyeAutoTarget
{
    int varIndex = 0;
    T_ROBOT_COORS target;
    std::array<int, 7> config = {};
};

T_ROBOT_COORS PoseFromPosVarArray(const double values[6])
{
    T_ROBOT_COORS pose;
    pose.dX = values[0];
    pose.dY = values[1];
    pose.dZ = values[2];
    pose.dRX = values[3];
    pose.dRY = values[4];
    pose.dRZ = values[5];
    return pose;
}

bool IsFinitePose(const T_ROBOT_COORS& pose)
{
    return std::isfinite(pose.dX)
        && std::isfinite(pose.dY)
        && std::isfinite(pose.dZ)
        && std::isfinite(pose.dRX)
        && std::isfinite(pose.dRY)
        && std::isfinite(pose.dRZ);
}

double PoseTranslationDistance(const T_ROBOT_COORS& a, const T_ROBOT_COORS& b)
{
    const double dx = a.dX - b.dX;
    const double dy = a.dY - b.dY;
    const double dz = a.dZ - b.dZ;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double AngleDiffDeg(double a, double b)
{
    double diff = std::fmod(a - b + 180.0, 360.0);
    if (diff < 0.0)
    {
        diff += 360.0;
    }
    return diff - 180.0;
}

double PoseRotationDistance(const T_ROBOT_COORS& a, const T_ROBOT_COORS& b)
{
    const double drx = AngleDiffDeg(a.dRX, b.dRX);
    const double dry = AngleDiffDeg(a.dRY, b.dRY);
    const double drz = AngleDiffDeg(a.dRZ, b.dRZ);
    return std::sqrt(drx * drx + dry * dry + drz * drz);
}

bool ReadAutoCalibrationTarget(
    RobotDriverAdaptor* driver,
    int varIndex,
    HandEyeAutoTarget& target,
    QString* error)
{
    if (driver == nullptr)
    {
        if (error != nullptr)
        {
            *error = "机器人驱动为空。";
        }
        return false;
    }

    double values[6] = {};
    int config[7] = {};
    const int ret = driver->GetPosVar(varIndex, values, config, POSVAR);
    if (ret != 0)
    {
        if (error != nullptr)
        {
            const QString detail = DecodeRobotMessageText(driver->GetLastRobotError());
            *error = detail.isEmpty()
                ? QString("读取位置变量[%1]失败，返回码=%2。").arg(varIndex).arg(ret)
                : QString("读取位置变量[%1]失败，返回码=%2，%3").arg(varIndex).arg(ret).arg(detail);
        }
        return false;
    }

    target.varIndex = varIndex;
    target.target = PoseFromPosVarArray(values);
    for (int i = 0; i < 7; ++i)
    {
        target.config[static_cast<size_t>(i)] = config[i];
    }

    if (!IsFinitePose(target.target))
    {
        if (error != nullptr)
        {
            *error = QString("位置变量[%1]包含无效数值。").arg(varIndex);
        }
        return false;
    }
    return true;
}

bool WaitGenericRobotDone(
    RobotDriverAdaptor* driver,
    int timeoutMs,
    int pollIntervalMs,
    QString* error,
    const T_ROBOT_COORS* targetPose = nullptr,
    const std::function<void(const QString&)>& progressLog = {})
{
    if (driver == nullptr)
    {
        if (error != nullptr)
        {
            *error = "机器人驱动为空。";
        }
        return false;
    }

    if (pollIntervalMs <= 0)
    {
        pollIntervalMs = 100;
    }

    const auto startTime = std::chrono::steady_clock::now();
    bool seenRunning = false;
    bool movementSeen = false;
    const T_ROBOT_COORS startPose = driver->GetCurrentPos();
    T_ROBOT_COORS lastPose = startPose;
    const bool hasStartPose = IsFinitePose(startPose);
    int stableDoneCount = 0;
    int lastState = -1;
    int lastLogSecond = -1;

    while (true)
    {
        lastState = driver->CheckDone();
        if (lastState < 0)
        {
            if (error != nullptr)
            {
                *error = QString("读取机器人运行状态失败，返回码=%1。").arg(lastState);
            }
            return false;
        }

        const auto now = std::chrono::steady_clock::now();
        const int elapsedMs = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(now - startTime).count());
        if (lastState == 0)
        {
            seenRunning = true;
            stableDoneCount = 0;

            const T_ROBOT_COORS currentPose = driver->GetCurrentPos();
            if (IsFinitePose(currentPose))
            {
                lastPose = currentPose;
                if (hasStartPose)
                {
                    const double moveDelta = PoseTranslationDistance(startPose, currentPose);
                    const double rotateDelta = PoseRotationDistance(startPose, currentPose);
                    if (moveDelta >= kHandEyeAutoMoveDetectMm || rotateDelta >= kHandEyeAutoRotateDetectDeg)
                    {
                        movementSeen = true;
                    }
                }
            }

            const int elapsedSecond = elapsedMs / 1000;
            if (progressLog && elapsedSecond != lastLogSecond)
            {
                lastLogSecond = elapsedSecond;
                progressLog(QString("等待机器人到位：已运行 %1 s，状态=%2，当前位置=%3")
                    .arg(elapsedSecond)
                    .arg(lastState)
                    .arg(FormatPoseSummary(lastPose)));
            }

            if (hasStartPose
                && !movementSeen
                && elapsedMs >= kHandEyeAutoNoMotionTimeoutMs)
            {
                if (error != nullptr)
                {
                    *error = QString("机器人程序已进入运行态，但 %1 ms 内当前位置未变化。当前状态：%2")
                        .arg(kHandEyeAutoNoMotionTimeoutMs)
                        .arg(DecodeRobotMessageText(driver->GetRobotStatusText()));
                }
                return false;
            }
        }
        else if (seenRunning || elapsedMs >= 1000)
        {
            ++stableDoneCount;
            if (stableDoneCount >= 2)
            {
                if (targetPose != nullptr)
                {
                    const T_ROBOT_COORS finalPose = driver->GetCurrentPos();
                    if (!IsFinitePose(finalPose))
                    {
                        if (error != nullptr)
                        {
                            *error = "机器人状态已停止，但读取当前位置无效，无法确认是否真正到位。";
                        }
                        return false;
                    }

                    const double positionError = PoseTranslationDistance(finalPose, *targetPose);
                    const double rotationError = PoseRotationDistance(finalPose, *targetPose);
                    if (progressLog)
                    {
                        progressLog(QString("到位位置复核：XYZ误差=%1 mm，姿态误差=%2 deg；目标=%3；当前=%4")
                            .arg(positionError, 0, 'f', 3)
                            .arg(rotationError, 0, 'f', 3)
                            .arg(FormatPoseSummary(*targetPose))
                            .arg(FormatPoseSummary(finalPose)));
                    }

                    if (positionError > kHandEyeAutoArrivePositionToleranceMm
                        || rotationError > kHandEyeAutoArriveRotationToleranceDeg)
                    {
                        if (error != nullptr)
                        {
                            *error = QString("机器人状态已停止，但当前位置未到目标。XYZ误差=%1 mm(阈值%2)，姿态误差=%3 deg(阈值%4)。目标=%5；当前=%6；当前状态：%7")
                                .arg(positionError, 0, 'f', 3)
                                .arg(kHandEyeAutoArrivePositionToleranceMm, 0, 'f', 3)
                                .arg(rotationError, 0, 'f', 3)
                                .arg(kHandEyeAutoArriveRotationToleranceDeg, 0, 'f', 3)
                                .arg(FormatPoseSummary(*targetPose))
                                .arg(FormatPoseSummary(finalPose))
                                .arg(DecodeRobotMessageText(driver->GetRobotStatusText()));
                        }
                        return false;
                    }
                }
                return true;
            }
        }

        if (elapsedMs >= timeoutMs)
        {
            if (error != nullptr)
            {
                *error = QString("等待机器人到位超时，最后状态=%1，当前状态：%2")
                    .arg(lastState)
                    .arg(DecodeRobotMessageText(driver->GetRobotStatusText()));
            }
            return false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(pollIntervalMs));
    }
}

QString SampleStateText(const HandEyeCalibrationSample& sample)
{
    if (!sample.valid)
    {
        return "状态：未采集";
    }
    return QString("状态：已采集  相机点=(%1, %2, %3)")
        .arg(sample.cameraPoint.x(), 0, 'f', 3)
        .arg(sample.cameraPoint.y(), 0, 'f', 3)
        .arg(sample.cameraPoint.z(), 0, 'f', 3);
}

struct IntervalStats
{
    int count = 0;
    double min = 0.0;
    double max = 0.0;
    double mean = 0.0;
    double median = 0.0;
    double stddev = 0.0;
};

IntervalStats CalcIntervalStats(QVector<double> values)
{
    IntervalStats stats;
    stats.count = values.size();
    if (values.isEmpty())
    {
        return stats;
    }

    std::sort(values.begin(), values.end());
    stats.min = values.front();
    stats.max = values.back();
    if ((values.size() % 2) == 0)
    {
        stats.median = (values[values.size() / 2 - 1] + values[values.size() / 2]) * 0.5;
    }
    else
    {
        stats.median = values[values.size() / 2];
    }

    double sum = 0.0;
    for (double value : values)
    {
        sum += value;
    }
    stats.mean = sum / static_cast<double>(values.size());

    double variance = 0.0;
    for (double value : values)
    {
        const double diff = value - stats.mean;
        variance += diff * diff;
    }
    stats.stddev = std::sqrt(variance / static_cast<double>(values.size()));
    return stats;
}

QString FormatStatsLine(const QString& name, const IntervalStats& stats, const QString& unit, int precision = 3)
{
    if (stats.count <= 0)
    {
        return QString("%1：无有效数据").arg(name);
    }
    return QString("%1：N=%2  平均=%3%4  中位=%5%4  最小=%6%4  最大=%7%4  标准差=%8%4")
        .arg(name)
        .arg(stats.count)
        .arg(stats.mean, 0, 'f', precision)
        .arg(unit)
        .arg(stats.median, 0, 'f', precision)
        .arg(stats.min, 0, 'f', precision)
        .arg(stats.max, 0, 'f', precision)
        .arg(stats.stddev, 0, 'f', precision);
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
}

HandEyeCalibrationDialog::HandEyeCalibrationDialog(
    ContralUnit* pContralUnit,
    const QString& robotName,
    const QString& cameraSection,
    StartCameraFunc startCamera,
    StopCameraFunc stopCamera,
    CameraFrameCache* cameraCache,
    QWidget* parent)
    : QDialog(parent)
    , m_pContralUnit(pContralUnit)
    , m_robotName(robotName)
    , m_cameraSection(cameraSection)
    , m_startCamera(startCamera)
    , m_stopCamera(stopCamera)
    , m_pCameraCache(cameraCache)
{
    setWindowTitle(QString("%1 %2 手眼标定").arg(robotName, cameraSection));
    ApplyUnifiedWindowChrome(this);
    ResizeWindowForAvailableGeometry(this, QSize(1680, 720), 0.96, 0.78);
    setStyleSheet(
        "QDialog { background: #111820; color: #ECF3F4; }"
        "QGroupBox { border: 1px solid #2E4656; border-radius: 12px; margin-top: 18px; padding: 14px; font-weight: bold; color: #9ED8DB; }"
        "QGroupBox::title { subcontrol-origin: margin; left: 16px; padding: 0 8px; }"
        "QPushButton { background: #233645; color: #F5FAFA; border: 1px solid #3C6173; border-radius: 10px; padding: 8px 14px; }"
        "QPushButton:hover { background: #2D5465; border-color: #72D4DD; }"
        "QLineEdit { background: #0B1117; color: #F5FAFA; border: 1px solid #385366; border-radius: 8px; padding: 6px 8px; min-width: 88px; }"
        "QDoubleSpinBox { background: #0B1117; color: #F5FAFA; border: 1px solid #385366; border-radius: 8px; padding: 6px 8px; min-width: 150px; }"
        "QPlainTextEdit { background: #081018; color: #BFE8EC; border: 1px solid #2C4653; border-radius: 10px; padding: 8px; }"
        "QScrollArea { border: none; }"
        "#HandEyeCalibrationScrollContent { background: #111820; }"
        "QLabel { color: #BACBD1; }");

    QVBoxLayout* windowLayout = new QVBoxLayout(this);
    windowLayout->setContentsMargins(0, 0, 0, 0);
    windowLayout->setSpacing(0);

    QScrollArea* scrollArea = new QScrollArea(this);
    scrollArea->setObjectName("AdaptiveWindowScrollArea");
    ConfigureResponsiveScrollArea(scrollArea);
    scrollArea->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    QWidget* scrollContent = new QWidget(scrollArea);
    scrollContent->setObjectName("HandEyeCalibrationScrollContent");
    scrollContent->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);

    QVBoxLayout* rootLayout = new QVBoxLayout(scrollContent);
    rootLayout->setContentsMargins(12, 12, 12, 12);
    rootLayout->setSpacing(8);

    scrollArea->setWidget(scrollContent);
    windowLayout->addWidget(scrollArea, 1);

    QLabel* titleLabel = new QLabel(QString("手眼标定 - %1 / %2").arg(robotName, cameraSection));
    titleLabel->setStyleSheet("font-size: 24px; font-weight: bold; color: #F7FCFC;");
    rootLayout->addWidget(titleLabel);

    QLabel* hintLabel = new QLabel("读取目标点后采集 6 组机器人位姿和相机点；自动标定按 PR[10]~PR[16] 到位确认采集。");
    hintLabel->setWordWrap(true);
    rootLayout->addWidget(hintLabel);

    QGroupBox* baseGroup = new QGroupBox("基础信息 / 运行日志");
    baseGroup->setMaximumHeight(188);
    QHBoxLayout* baseLayout = new QHBoxLayout(baseGroup);
    baseLayout->setSpacing(10);

    QWidget* infoPanel = new QWidget(baseGroup);
    QVBoxLayout* infoLayout = new QVBoxLayout(infoPanel);
    infoLayout->setContentsMargins(0, 0, 0, 0);
    infoLayout->setSpacing(2);

    QLabel* summaryLabel = new QLabel(QString("当前机器人：%1    当前相机：%2").arg(robotName, cameraSection));
    summaryLabel->setStyleSheet("color: #E6F1F2;");
    infoLayout->addWidget(summaryLabel);

    m_pAutoStateLabel = new QLabel("自动标定状态：待启动");
    m_pAutoStateLabel->setStyleSheet("font-weight: bold; color: #F3D37A;");
    infoLayout->addWidget(m_pAutoStateLabel);

    QWidget* autoSpeedRow = new QWidget(infoPanel);
    QHBoxLayout* autoSpeedLayout = new QHBoxLayout(autoSpeedRow);
    autoSpeedLayout->setContentsMargins(0, 0, 0, 0);
    autoSpeedLayout->setSpacing(8);
    QLabel* autoSpeedLabel = new QLabel("自动标定速度：");
    m_pAutoMoveSpeedSpin = new QDoubleSpinBox(autoSpeedRow);
    m_pAutoMoveSpeedSpin->setRange(1.0, 10000.0);
    m_pAutoMoveSpeedSpin->setDecimals(1);
    m_pAutoMoveSpeedSpin->setSingleStep(50.0);
    m_pAutoMoveSpeedSpin->setValue(kHandEyeAutoMoveSpeedMmPerMin);
    m_pAutoMoveSpeedSpin->setSuffix(" mm/min");
    m_pAutoMoveSpeedSpin->setAlignment(Qt::AlignRight);
    m_pAutoMoveSpeedSpin->setKeyboardTracking(false);
    m_pAutoMoveSpeedSpin->setButtonSymbols(QAbstractSpinBox::NoButtons);
    autoSpeedLayout->addWidget(autoSpeedLabel);
    autoSpeedLayout->addWidget(m_pAutoMoveSpeedSpin);
    autoSpeedLayout->addStretch(1);
    infoLayout->addWidget(autoSpeedRow);

    m_pCalibrationPathLabel = new QLabel("标定文件：");
    m_pCalibrationPathLabel->setWordWrap(true);
    infoLayout->addWidget(m_pCalibrationPathLabel);

    m_pMatrixPathLabel = new QLabel("矩阵文件：");
    m_pMatrixPathLabel->setWordWrap(true);
    infoLayout->addWidget(m_pMatrixPathLabel);

    m_pReportPathLabel = new QLabel("报告文件：");
    m_pReportPathLabel->setWordWrap(true);
    infoLayout->addWidget(m_pReportPathLabel);
    infoLayout->addStretch(1);

    QWidget* logPanel = new QWidget(baseGroup);
    QVBoxLayout* logLayout = new QVBoxLayout(logPanel);
    logLayout->setContentsMargins(0, 0, 0, 0);
    logLayout->setSpacing(2);

    QLabel* logTitle = new QLabel("运行日志");
    logTitle->setStyleSheet("font-weight: bold; color: #9ED8DB;");
    logLayout->addWidget(logTitle);

    m_pLogText = new QPlainTextEdit();
    m_pLogText->setReadOnly(true);
    m_pLogText->document()->setMaximumBlockCount(1200);
    m_pLogText->setMinimumHeight(64);
    m_pLogText->setMaximumHeight(78);
    logLayout->addWidget(m_pLogText, 1);

    baseLayout->addWidget(infoPanel, 3);
    baseLayout->addWidget(logPanel, 2);
    rootLayout->addWidget(baseGroup);

    QHBoxLayout* calibrationLayout = new QHBoxLayout();
    calibrationLayout->setSpacing(12);

    QGroupBox* tcpGroup = new QGroupBox("固定标定目标点");
    QGridLayout* tcpLayout = new QGridLayout(tcpGroup);
    tcpLayout->setHorizontalSpacing(8);
    tcpLayout->setVerticalSpacing(10);
    const QStringList poseLabels = { "X", "Y", "Z" };
    for (int index = 0; index < poseLabels.size(); ++index)
    {
        QLabel* label = new QLabel(poseLabels[index]);
        QLineEdit* edit = new QLineEdit();
        ApplyHandEyeNumericEdit(edit);
        tcpLayout->addWidget(label, index, 0);
        tcpLayout->addWidget(edit, index, 1);
        m_tcpEdits.push_back(edit);
    }
    QPushButton* tcpCaptureBtn = new QPushButton("读取目标点");
    tcpCaptureBtn->setMinimumHeight(42);
    tcpLayout->addWidget(tcpCaptureBtn, poseLabels.size(), 0, 1, 2);
    tcpLayout->setRowStretch(poseLabels.size() + 1, 1);
    tcpGroup->setMinimumWidth(320);
    tcpGroup->setMaximumWidth(360);
    calibrationLayout->addWidget(tcpGroup, 0);

    QGroupBox* sampleGroup = new QGroupBox("六组标定采样");
    QVBoxLayout* sampleLayout = new QVBoxLayout(sampleGroup);
    QWidget* sampleShell = new QWidget(sampleGroup);
    sampleShell->setObjectName("sampleTabShell");
    sampleShell->setStyleSheet("#sampleTabShell { background: #0B1117; border: 1px solid #2E4656; }");
    QVBoxLayout* sampleShellLayout = new QVBoxLayout(sampleShell);
    sampleShellLayout->setContentsMargins(10, 10, 10, 10);
    sampleShellLayout->setSpacing(8);
    m_pSampleTabWidget = new QTabWidget(sampleGroup);
    m_pSampleTabWidget->setDocumentMode(true);
    m_pSampleTabWidget->setUsesScrollButtons(false);
    m_pSampleTabWidget->setElideMode(Qt::ElideNone);
    m_pSampleTabWidget->tabBar()->setExpanding(true);
    m_pSampleTabWidget->tabBar()->setDrawBase(false);
    m_pSampleTabWidget->setStyleSheet(
        "QTabWidget::pane { border: none; background: transparent; margin-top: 0px; }"
        "QTabBar::tab { background: #1A2834; color: #BFD9DD; border: 1px solid #365162; border-bottom: none; "
        "min-width: 104px; padding: 7px 18px; margin-right: 4px; }"
        "QTabBar::tab:selected { background: #274052; color: #F7FCFC; border-color: #4D7389; }"
        "QTabBar::tab:hover { background: #223747; }");

    m_sampleWidgets.resize(kHandEyeCalibrationSampleCount);
    for (int index = 0; index < kHandEyeCalibrationSampleCount; ++index)
    {
        QWidget* page = new QWidget(m_pSampleTabWidget);
        QVBoxLayout* groupLayout = new QVBoxLayout(page);
        groupLayout->setContentsMargins(8, 10, 8, 8);
        groupLayout->setSpacing(12);

        QPushButton* captureBtn = new QPushButton(QString("采集第 %1 组").arg(index + 1));
        captureBtn->setMinimumHeight(40);
        captureBtn->setFixedWidth(156);
        QLabel* stateLabel = new QLabel("状态：未采集");
        stateLabel->setWordWrap(true);
        stateLabel->setMinimumWidth(280);
        stateLabel->setMaximumWidth(340);
        m_sampleWidgets[index].pStateLabel = stateLabel;

        QWidget* contentWidget = new QWidget(page);
        QVBoxLayout* dataLayout = new QVBoxLayout(contentWidget);
        dataLayout->setSpacing(12);
        dataLayout->setContentsMargins(0, 0, 0, 0);

        QGroupBox* robotGroup = new QGroupBox("机器人数据");
        QGridLayout* robotGrid = new QGridLayout(robotGroup);
        robotGrid->setHorizontalSpacing(12);
        robotGrid->setVerticalSpacing(10);
        const QStringList robotLabels = { "RobotX", "RobotY", "RobotZ", "RobotRX", "RobotRY", "RobotRZ" };
        for (int fieldIndex = 0; fieldIndex < robotLabels.size(); ++fieldIndex)
        {
            QLabel* label = new QLabel(robotLabels[fieldIndex]);
            QLineEdit* edit = new QLineEdit();
            ApplyHandEyeNumericEdit(edit);
            edit->setMinimumWidth(280);
            edit->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
            const int columnPair = fieldIndex < 3 ? 0 : 1;
            const int row = fieldIndex % 3;
            robotGrid->addWidget(label, row, columnPair * 2);
            robotGrid->addWidget(edit, row, columnPair * 2 + 1);
            m_sampleWidgets[index].robotEdits.push_back(edit);
        }
        robotGrid->setColumnStretch(1, 1);
        robotGrid->setColumnStretch(3, 1);
        robotGroup->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
        dataLayout->addWidget(robotGroup);

        QGroupBox* cameraGroup = new QGroupBox("相机数据");
        QGridLayout* cameraGrid = new QGridLayout(cameraGroup);
        cameraGrid->setHorizontalSpacing(12);
        cameraGrid->setVerticalSpacing(10);
        const QStringList cameraLabels = { "CameraX", "CameraY", "CameraZ" };
        for (int row = 0; row < cameraLabels.size(); ++row)
        {
            QLabel* label = new QLabel(cameraLabels[row]);
            QLineEdit* edit = new QLineEdit();
            ApplyHandEyeNumericEdit(edit);
            edit->setMinimumWidth(220);
            edit->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
            cameraGrid->addWidget(label, row, 0);
            cameraGrid->addWidget(edit, row, 1);
            m_sampleWidgets[index].cameraEdits.push_back(edit);
        }
        cameraGrid->setColumnStretch(1, 1);
        cameraGroup->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

        QWidget* sidePanel = new QWidget(page);
        QVBoxLayout* sideLayout = new QVBoxLayout(sidePanel);
        sideLayout->setContentsMargins(0, 0, 0, 0);
        sideLayout->setSpacing(12);

        QLabel* stateTitle = new QLabel("采集状态");
        stateTitle->setStyleSheet("font-weight: bold; color: #9ED8DB;");
        sideLayout->addWidget(stateTitle);
        sideLayout->addWidget(stateLabel);
        sideLayout->addWidget(captureBtn, 0, Qt::AlignLeft);
        sideLayout->addStretch(1);
        sidePanel->setMinimumWidth(320);
        sidePanel->setMaximumWidth(420);
        sidePanel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

        QHBoxLayout* lowerLayout = new QHBoxLayout();
        lowerLayout->setContentsMargins(0, 0, 0, 0);
        lowerLayout->setSpacing(24);
        lowerLayout->addWidget(cameraGroup, 3, Qt::AlignTop);
        lowerLayout->addWidget(sidePanel, 2, Qt::AlignTop);
        dataLayout->addLayout(lowerLayout);

        groupLayout->addWidget(contentWidget);
        groupLayout->addStretch(1);
        m_pSampleTabWidget->addTab(page, QString("第%1组").arg(index + 1));

        connect(captureBtn, &QPushButton::clicked, this, [this, index]() { CaptureSample(index); });
    }
    sampleShellLayout->addWidget(m_pSampleTabWidget, 1);
    sampleLayout->addWidget(sampleShell, 1);
    calibrationLayout->addWidget(sampleGroup, 1);
    rootLayout->addLayout(calibrationLayout, 1);

    QHBoxLayout* buttonLayout = new QHBoxLayout();
    buttonLayout->setSpacing(10);
    m_pUploadAutoProgramBtn = new QPushButton("发送FANUC自动程序");
    m_pAutoCalibrationBtn = new QPushButton("自动标定 变量10~16");
    QPushButton* reloadBtn = new QPushButton("重新读取");
    QPushButton* saveBtn = new QPushButton("保存采样数据");
    QPushButton* computeBtn = new QPushButton("计算矩阵并写入");
    QPushButton* testBtn = new QPushButton("手眼参数检测");
    QPushButton* timestampCheckBtn = new QPushButton("相机时间戳检测");
    QPushButton* matrixBtn = new QPushButton("打开矩阵参数");
    QPushButton* closeBtn = new QPushButton("关闭");
    buttonLayout->addWidget(m_pUploadAutoProgramBtn);
    buttonLayout->addWidget(m_pAutoCalibrationBtn);
    buttonLayout->addStretch(1);
    buttonLayout->addWidget(reloadBtn);
    buttonLayout->addWidget(saveBtn);
    buttonLayout->addWidget(computeBtn);
    buttonLayout->addWidget(testBtn);
    buttonLayout->addWidget(timestampCheckBtn);
    buttonLayout->addWidget(matrixBtn);
    buttonLayout->addWidget(closeBtn);
    rootLayout->addLayout(buttonLayout);

    connect(tcpCaptureBtn, &QPushButton::clicked, this, [this]() { CaptureTcpPoint(); });
    connect(m_pUploadAutoProgramBtn, &QPushButton::clicked, this, [this]() { UploadAutoCalibrationProgram(); });
    connect(m_pAutoCalibrationBtn, &QPushButton::clicked, this, [this]() { StartAutoCalibration(); });
    connect(reloadBtn, &QPushButton::clicked, this, [this]() { LoadConfig(); });
    connect(saveBtn, &QPushButton::clicked, this, [this]() { SaveConfig(); });
    connect(computeBtn, &QPushButton::clicked, this, [this]() { ComputeAndSaveMatrix(); });
    connect(testBtn, &QPushButton::clicked, this, [this]() { TestHandEyeMatrix(); });
    connect(timestampCheckBtn, &QPushButton::clicked, this, [this]() { CheckCameraTimestampIntervals(); });
    connect(matrixBtn, &QPushButton::clicked, this, [this]() { OpenMatrixDialog(); });
    connect(closeBtn, &QPushButton::clicked, this, &QDialog::close);

    LoadConfig();
}

void HandEyeCalibrationDialog::closeEvent(QCloseEvent* event)
{
    if (!HasUnsavedChanges())
    {
        QDialog::closeEvent(event);
        return;
    }

    if (ConfirmCloseWithUnsavedChanges(this, "手眼标定", [this]() { return SaveConfig(); }))
    {
        event->accept();
    }
    else
    {
        event->ignore();
    }
}

bool HandEyeCalibrationDialog::LoadConfig()
{
    QString error;
    QString filePath;
    if (!LoadHandEyeCalibrationConfig(m_robotName, m_cameraSection, m_config, &error, &filePath))
    {
        QMessageBox::warning(this, "手眼标定", error);
        AppendLog("读取标定文件失败：" + error);
        return false;
    }

    UpdatePathLabels();
    SetRobotPoseEditors(m_tcpEdits, m_config.tcpPoint);
    for (int index = 0; index < std::min(m_sampleWidgets.size(), m_config.samples.size()); ++index)
    {
        SetRobotPoseEditors(m_sampleWidgets[index].robotEdits, m_config.samples[index].robotPose);
        SetVectorEditors(m_sampleWidgets[index].cameraEdits, m_config.samples[index].cameraPoint);
    }
    UpdateSampleStates();
    int expandedIndex = 0;
    for (int index = m_config.samples.size() - 1; index >= 0; --index)
    {
        if (m_config.samples[index].valid)
        {
            expandedIndex = index;
            break;
        }
    }
    SelectSampleTab(expandedIndex);
    AppendLog(QString("已读取标定文件：%1").arg(filePath));
    MarkCleanSnapshot();
    return true;
}

bool HandEyeCalibrationDialog::SaveConfig()
{
    QString error;
    if (!SaveConfigSilently(&error))
    {
        QMessageBox::warning(this, "手眼标定", error);
        AppendLog("保存失败：" + error);
        return false;
    }

    UpdateSampleStates();
    AppendLog("手眼标定采样数据已保存。");
    QMessageBox::information(this, "手眼标定", "采样数据已保存。");
    MarkCleanSnapshot();
    return true;
}

bool HandEyeCalibrationDialog::SaveConfigSilently(QString* error)
{
    bool ok = false;
    QString parseError;
    m_config.tcpPoint = ReadRobotPoseEditors(m_tcpEdits, &ok, &parseError);
    if (!ok)
    {
        if (error != nullptr)
        {
            *error = "固定标定目标点输入无效：" + parseError;
        }
        return false;
    }

    m_config.samples.resize(kHandEyeCalibrationSampleCount);
    for (int index = 0; index < m_sampleWidgets.size(); ++index)
    {
        HandEyeCalibrationSample& sample = m_config.samples[index];
        sample.robotPose = ReadRobotPoseEditors(m_sampleWidgets[index].robotEdits, &ok, &parseError);
        if (!ok)
        {
            if (error != nullptr)
            {
                *error = QString("第 %1 组机器人位姿无效：%2").arg(index + 1).arg(parseError);
            }
            return false;
        }

        sample.cameraPoint = ReadVectorEditors(m_sampleWidgets[index].cameraEdits, &ok, &parseError);
        if (!ok)
        {
            if (error != nullptr)
            {
                *error = QString("第 %1 组相机点无效：%2").arg(index + 1).arg(parseError);
            }
            return false;
        }
    }

    QString filePath;
    if (!SaveHandEyeCalibrationConfig(m_robotName, m_cameraSection, m_config, error, &filePath))
    {
        return false;
    }
    UpdatePathLabels();
    MarkCleanSnapshot();
    return true;
}

bool HandEyeCalibrationDialog::ApplyCapturedTargetPoint(const T_ROBOT_COORS& pose, QString* error)
{
    m_config.tcpPoint = pose;
    SetRobotPoseEditors(m_tcpEdits, pose);

    if (!SaveConfigSilently(error))
    {
        return false;
    }

    AppendLog(QString("已读取固定标定目标点：X=%1 Y=%2 Z=%3 RX=%4 RY=%5 RZ=%6")
        .arg(pose.dX, 0, 'f', 3)
        .arg(pose.dY, 0, 'f', 3)
        .arg(pose.dZ, 0, 'f', 3)
        .arg(pose.dRX, 0, 'f', 3)
        .arg(pose.dRY, 0, 'f', 3)
        .arg(pose.dRZ, 0, 'f', 3));
    return true;
}

bool HandEyeCalibrationDialog::ApplyCapturedSample(int index, const T_ROBOT_COORS& pose, const Eigen::Vector3d& cameraPoint, QString* error)
{
    if (index < 0 || index >= m_sampleWidgets.size())
    {
        if (error != nullptr)
        {
            *error = QString("采样组索引越界：%1").arg(index);
        }
        return false;
    }

    HandEyeCalibrationSample& sample = m_config.samples[index];
    sample.valid = true;
    sample.robotPose = pose;
    sample.cameraPoint = cameraPoint;

    SetRobotPoseEditors(m_sampleWidgets[index].robotEdits, sample.robotPose);
    SetVectorEditors(m_sampleWidgets[index].cameraEdits, sample.cameraPoint);

    if (!SaveConfigSilently(error))
    {
        return false;
    }

    UpdateSampleStates();
    SelectSampleTab(index);
    AppendLog(QString("已采集第 %1 组：Robot=(%2,%3,%4) Camera=(%5,%6,%7)")
        .arg(index + 1)
        .arg(sample.robotPose.dX, 0, 'f', 3)
        .arg(sample.robotPose.dY, 0, 'f', 3)
        .arg(sample.robotPose.dZ, 0, 'f', 3)
        .arg(sample.cameraPoint.x(), 0, 'f', 3)
        .arg(sample.cameraPoint.y(), 0, 'f', 3)
        .arg(sample.cameraPoint.z(), 0, 'f', 3));
    return true;
}

bool HandEyeCalibrationDialog::CaptureTcpPoint()
{
    QString error;
    RobotDriverAdaptor* driver = CurrentDriver(&error);
    if (driver == nullptr)
    {
        QMessageBox::warning(this, "手眼标定", error);
        AppendLog("读取目标点失败：" + error);
        return false;
    }

    const T_ROBOT_COORS targetPoint = driver->GetCurrentPos();
    if (!ApplyCapturedTargetPoint(targetPoint, &error))
    {
        QMessageBox::warning(this, "手眼标定", error);
        AppendLog("保存目标点失败：" + error);
        return false;
    }
    return true;
}

bool HandEyeCalibrationDialog::CaptureSample(int index)
{
    if (index < 0 || index >= m_sampleWidgets.size())
    {
        return false;
    }

    QString error;
    RobotDriverAdaptor* driver = CurrentDriver(&error);
    if (driver == nullptr)
    {
        QMessageBox::warning(this, "手眼标定", error);
        AppendLog(QString("采集第 %1 组失败：%2").arg(index + 1).arg(error));
        return false;
    }

    Eigen::Vector3d cameraPoint = Eigen::Vector3d::Zero();
    if (!ReadLatestCameraPoint(cameraPoint, &error))
    {
        QMessageBox::warning(this, "手眼标定", error);
        AppendLog(QString("采集第 %1 组失败：%2").arg(index + 1).arg(error));
        return false;
    }

    const T_ROBOT_COORS robotPose = driver->GetCurrentPos();
    if (!ApplyCapturedSample(index, robotPose, cameraPoint, &error))
    {
        QMessageBox::warning(this, "手眼标定", error);
        AppendLog(QString("保存第 %1 组失败：%2").arg(index + 1).arg(error));
        return false;
    }
    return true;
}

bool HandEyeCalibrationDialog::EnsureCameraReady(const QString& sceneName, Eigen::Vector3d* cameraPointOut, QString* error)
{
    Eigen::Vector3d cameraPoint = Eigen::Vector3d::Zero();
    QString latestError;
    if (ReadLatestCameraPoint(cameraPoint, &latestError))
    {
        AppendLog(QString("%1：相机已在线，相机点=(%2, %3, %4)")
            .arg(sceneName)
            .arg(cameraPoint.x(), 0, 'f', 3)
            .arg(cameraPoint.y(), 0, 'f', 3)
            .arg(cameraPoint.z(), 0, 'f', 3));
        if (cameraPointOut != nullptr)
        {
            *cameraPointOut = cameraPoint;
        }
        return true;
    }

    if (!m_startCamera)
    {
        if (error != nullptr)
        {
            *error = "当前没有可用的相机三维点，且未配置自动开相机回调。";
        }
        return false;
    }

    QString cameraIP;
    AppendLog(QString("%1：未检测到有效相机点，准备自动打开相机。原因：%2").arg(sceneName, latestError));
    if (!m_startCamera(cameraIP))
    {
        if (error != nullptr)
        {
            *error = "自动打开相机失败，请检查配置库中的设备地址。";
        }
        return false;
    }

    AppendLog(QString("%1：已触发相机启动：%2，等待首帧三维点...").arg(sceneName, cameraIP));

    using namespace std::chrono_literals;
    const auto deadline = std::chrono::steady_clock::now() + 3s;
    while (std::chrono::steady_clock::now() < deadline)
    {
        if (ReadLatestCameraPoint(cameraPoint, &latestError))
        {
            AppendLog(QString("%1：相机启动成功，首帧三维点=(%2, %3, %4)")
                .arg(sceneName)
                .arg(cameraPoint.x(), 0, 'f', 3)
                .arg(cameraPoint.y(), 0, 'f', 3)
                .arg(cameraPoint.z(), 0, 'f', 3));
            if (cameraPointOut != nullptr)
            {
                *cameraPointOut = cameraPoint;
            }
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (error != nullptr)
    {
        *error = QString("%1：相机已尝试打开，但 3 秒内未收到有效三维点：%2").arg(sceneName, latestError);
    }
    return false;
}

bool HandEyeCalibrationDialog::EnsureCameraStarted(const QString& sceneName, QString* error)
{
    if (!m_startCamera)
    {
        if (error != nullptr)
        {
            *error = "未配置自动开相机回调，无法启动相机接收。";
        }
        return false;
    }

    QString cameraIP;
    if (!m_startCamera(cameraIP))
    {
        if (error != nullptr)
        {
            *error = "自动打开相机失败，请检查配置库中的设备地址。";
        }
        return false;
    }

    AppendLog(QString("%1：已触发相机启动：%2，本步骤不等待三维点有效。")
        .arg(sceneName, cameraIP));
    return true;
}

bool HandEyeCalibrationDialog::ComputeAndSaveMatrix()
{
    QString error;
    if (!SaveConfigSilently(&error))
    {
        QMessageBox::warning(this, "手眼标定", error);
        AppendLog("计算前保存采样失败：" + error);
        return false;
    }

    HandEyeMatrixConfig matrix;
    if (!ComputeHandEyeMatrixFromCalibration(m_robotName, m_config, matrix, &error))
    {
        QMessageBox::warning(this, "手眼标定", error);
        AppendLog("计算矩阵失败：" + error);
        return false;
    }

    QString matrixFilePath;
    if (!SaveHandEyeMatrixConfig(m_robotName, m_cameraSection, matrix, &error, &matrixFilePath))
    {
        QMessageBox::warning(this, "手眼标定", error);
        AppendLog("写入矩阵失败：" + error);
        return false;
    }
    m_bMatrixComputedThisSession = true;

    UpdatePathLabels();
    AppendLog("手眼矩阵计算完成：");
    for (int row = 0; row < 3; ++row)
    {
        AppendLog(QString("R%1 = [%2, %3, %4]")
            .arg(row)
            .arg(matrix.rotation(row, 0), 0, 'f', 6)
            .arg(matrix.rotation(row, 1), 0, 'f', 6)
            .arg(matrix.rotation(row, 2), 0, 'f', 6));
    }
    AppendLog(QString("T = [%1, %2, %3]")
        .arg(matrix.translation.x(), 0, 'f', 6)
        .arg(matrix.translation.y(), 0, 'f', 6)
        .arg(matrix.translation.z(), 0, 'f', 6));

    QString diagnosticPath;
    QString diagnosticSummary;
    QString diagnosticError;
    if (SaveHandEyeDiagnosticFile(
        m_robotName,
        m_cameraSection,
        "Compute",
        m_config,
        matrix,
        &diagnosticPath,
        &diagnosticSummary,
        &diagnosticError))
    {
        AppendLog(QString("手眼标定诊断：%1").arg(diagnosticSummary));
        AppendLog(QString("手眼标定诊断文件：%1").arg(diagnosticPath));
    }
    else
    {
        AppendLog(QString("手眼标定诊断文件保存失败：%1").arg(diagnosticError));
    }

    QString reportPath;
    if (!ExportCalibrationReport(matrix, &reportPath, &error))
    {
        AppendLog("报告导出失败：" + error);
        QMessageBox::warning(this, "手眼标定", QString("矩阵已写入，但报告导出失败：\n%1").arg(error));
        QMessageBox::information(this, "手眼标定", QString("矩阵已计算并写入：\n%1").arg(matrixFilePath));
        return true;
    }

    AppendLog(QString("标定报告已导出：%1").arg(reportPath));
    QMessageBox::information(this, "手眼标定", QString("矩阵已计算并写入：\n%1\n\n标定报告：\n%2").arg(matrixFilePath, reportPath));
    return true;
}

bool HandEyeCalibrationDialog::TestHandEyeMatrix()
{
    QString error;
    RobotDriverAdaptor* driver = CurrentDriver(&error);
    if (driver == nullptr)
    {
        QMessageBox::warning(this, "手眼标定", error);
        AppendLog("手眼参数检测失败：" + error);
        return false;
    }

    Eigen::Vector3d cameraPoint = Eigen::Vector3d::Zero();
    QString currentCameraError;
    bool hasCurrentCameraPoint = EnsureCameraReady("手眼参数检测前检查", &cameraPoint, &error);
    if (!hasCurrentCameraPoint)
    {
        currentCameraError = error;
        QMessageBox::warning(
            this,
            "手眼标定",
            "当前相机点获取失败，将跳过当前点检测，仅计算每组采样目标点。\n\n原因：" + currentCameraError);
        AppendLog("手眼参数检测：当前相机点获取失败，已跳过当前点检测：" + currentCameraError);
        error.clear();
    }

    HandEyeMatrixConfig matrix;
    QString matrixFilePath;
    if (!LoadHandEyeMatrixConfig(m_robotName, m_cameraSection, matrix, &error, &matrixFilePath))
    {
        QMessageBox::warning(this, "手眼标定", error);
        AppendLog("手眼参数检测失败：" + error);
        return false;
    }

    HandEyeCalibrationConfig checkConfig = m_config;
    checkConfig.samples.resize(kHandEyeCalibrationSampleCount);
    bool parseOk = false;
    QString parseError;
    for (int index = 0; index < m_sampleWidgets.size() && index < checkConfig.samples.size(); ++index)
    {
        HandEyeCalibrationSample& sample = checkConfig.samples[index];
        sample.robotPose = ReadRobotPoseEditors(m_sampleWidgets[index].robotEdits, &parseOk, &parseError);
        if (!parseOk)
        {
            error = QString("第 %1 组机器人位姿无效：%2").arg(index + 1).arg(parseError);
            QMessageBox::warning(this, "手眼标定", error);
            AppendLog("手眼参数检测失败：" + error);
            return false;
        }

        sample.cameraPoint = ReadVectorEditors(m_sampleWidgets[index].cameraEdits, &parseOk, &parseError);
        if (!parseOk)
        {
            error = QString("第 %1 组相机点无效：%2").arg(index + 1).arg(parseError);
            QMessageBox::warning(this, "手眼标定", error);
            AppendLog("手眼参数检测失败：" + error);
            return false;
        }
    }

    QStringList sampleTargetLines;
    QStringList sampleTargetLogLines;
    int validSampleTargetCount = 0;
    for (int index = 0; index < checkConfig.samples.size(); ++index)
    {
        const HandEyeCalibrationSample& sample = checkConfig.samples[index];
        if (!sample.valid)
        {
            sampleTargetLines.push_back(QString("第%1组：未采集，跳过。").arg(index + 1));
            sampleTargetLogLines.push_back(QString("第%1组：未采集，跳过。").arg(index + 1));
            continue;
        }

        const Eigen::Vector3d sampleTarget =
            RobotCalculation::CalcLaserPointInRobot(sample.robotPose, sample.cameraPoint, matrix);
        if (!std::isfinite(sampleTarget.x()) ||
            !std::isfinite(sampleTarget.y()) ||
            !std::isfinite(sampleTarget.z()))
        {
            sampleTargetLines.push_back(QString("第%1组：目标点计算结果无效。").arg(index + 1));
            sampleTargetLogLines.push_back(QString("第%1组：目标点计算结果无效。").arg(index + 1));
            continue;
        }

        ++validSampleTargetCount;
        sampleTargetLines.push_back(QString("第%1组：X=%2  Y=%3  Z=%4")
            .arg(index + 1)
            .arg(sampleTarget.x(), 0, 'f', 3)
            .arg(sampleTarget.y(), 0, 'f', 3)
            .arg(sampleTarget.z(), 0, 'f', 3));
        sampleTargetLogLines.push_back(QString(
            "第%1组：Robot=(%2, %3, %4, %5, %6, %7) Camera=(%8, %9, %10) Target=(%11, %12, %13)")
            .arg(index + 1)
            .arg(sample.robotPose.dX, 0, 'f', 3)
            .arg(sample.robotPose.dY, 0, 'f', 3)
            .arg(sample.robotPose.dZ, 0, 'f', 3)
            .arg(sample.robotPose.dRX, 0, 'f', 3)
            .arg(sample.robotPose.dRY, 0, 'f', 3)
            .arg(sample.robotPose.dRZ, 0, 'f', 3)
            .arg(sample.cameraPoint.x(), 0, 'f', 3)
            .arg(sample.cameraPoint.y(), 0, 'f', 3)
            .arg(sample.cameraPoint.z(), 0, 'f', 3)
            .arg(sampleTarget.x(), 0, 'f', 3)
            .arg(sampleTarget.y(), 0, 'f', 3)
            .arg(sampleTarget.z(), 0, 'f', 3));
    }
    if (validSampleTargetCount == 0)
    {
        sampleTargetLines.push_back("没有可计算的已采集采样组。");
        sampleTargetLogLines.push_back("没有可计算的已采集采样组。");
    }

    const T_ROBOT_COORS robotPose = driver->GetCurrentPos();
    Eigen::Vector3d laserPoint = Eigen::Vector3d::Zero();
    bool hasLocalLaserPoint = false;
    QString localLaserText = "当前相机点未获取，已跳过当前点本地矩阵计算。";
    if (hasCurrentCameraPoint)
    {
        laserPoint = RobotCalculation::CalcLaserPointInRobot(robotPose, cameraPoint, matrix);
        hasLocalLaserPoint =
            std::isfinite(laserPoint.x()) &&
            std::isfinite(laserPoint.y()) &&
            std::isfinite(laserPoint.z());
        if (hasLocalLaserPoint)
        {
            localLaserText = QString("X=%1  Y=%2  Z=%3")
                .arg(laserPoint.x(), 0, 'f', 3)
                .arg(laserPoint.y(), 0, 'f', 3)
                .arg(laserPoint.z(), 0, 'f', 3);
        }
        else
        {
            localLaserText = "根据当前手眼矩阵计算出的激光位置无效，已跳过当前点 TP 对比和运动。";
            AppendLog("手眼参数检测：当前点本地矩阵结果无效，已跳过当前点 TP 对比和运动。");
        }
    }

    Eigen::Vector3d robotLaserPoint = Eigen::Vector3d::Zero();
    bool hasRobotResult = false;
    QString robotDetailText = "机器人程序结果：当前驱动不是 FANUC，未执行 TP 对比。";
    FANUCRobotCtrl* fanucDriver = CurrentFanucDriver(nullptr);
    if (fanucDriver != nullptr && hasLocalLaserPoint)
    {
        AppendLog(QString("机器人手眼检测使用机器人侧现有程序：%1；SENSOR 指令由现场 TP 程序提供，不在检测时自动编译上传。")
            .arg(kHandEyeRobotCheckProgramName));

        int config[7] = { 0 };
        double prStartSeed[8] =
        {
            robotPose.dX, robotPose.dY, robotPose.dZ,
            robotPose.dRX, robotPose.dRY, robotPose.dRZ,
            robotPose.dBX, robotPose.dBY
        };
        if (!fanucDriver->SetPosVar(kHandEyeRobotCheckStartPrIndex, prStartSeed, POSVAR, 1, config, ENGINEEVAR, POSVAR))
        {
            error = QString("写入 PR[%1] 失败，无法启动机器人手眼检测。").arg(kHandEyeRobotCheckStartPrIndex);
            QMessageBox::warning(this, "手眼标定", error);
            AppendLog("手眼参数检测失败：" + error);
            return false;
        }
        AppendLog(QString("已将当前位置写入 PR[%1]，准备调用机器人手眼检测程序。")
            .arg(kHandEyeRobotCheckStartPrIndex));

        AppendLog(QString("准备调用机器人手眼检测程序：%1，按 R[%2]=10/20/1 这套状态流程等待完成。")
            .arg(kHandEyeRobotCheckProgramName)
            .arg(kHandEyeRobotCheckStateReg));

        int robotState = 0;
        if (!fanucDriver->CallJobAndWaitStateDone(
            kHandEyeRobotCheckProgramName,
            kHandEyeRobotCheckStateReg,
            1,
            10,
            20,
            5000,
            10000,
            100,
            &robotState,
            true))
        {
            error = QString("机器人程序 %1 未按状态寄存器约定完成，R[%2] 最终=%3。请确认 TP 程序里有 R[%2]=10/20/1。")
                .arg(kHandEyeRobotCheckProgramName)
                .arg(kHandEyeRobotCheckStateReg)
                .arg(robotState);
            QMessageBox::warning(this, "手眼标定", error);
            AppendLog("手眼参数检测失败：" + error);
            return false;
        }

        AppendLog(QString("机器人程序已完成：R[%1]=%2，准备读取 PR[%3]。")
            .arg(kHandEyeRobotCheckStateReg)
            .arg(robotState)
            .arg(kHandEyeRobotCheckPrIndex));

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        double pr80[6] = { 0.0 };
        if (fanucDriver->GetPosVar(kHandEyeRobotCheckPrIndex, pr80, config, POSVAR) != 0)
        {
            error = QString("读取 PR[%1] 失败。").arg(kHandEyeRobotCheckPrIndex);
            QMessageBox::warning(this, "手眼标定", error);
            AppendLog("手眼参数检测失败：" + error);
            return false;
        }

        robotLaserPoint = Eigen::Vector3d(pr80[0], pr80[1], pr80[2]);
        hasRobotResult = std::isfinite(robotLaserPoint.x()) && std::isfinite(robotLaserPoint.y()) && std::isfinite(robotLaserPoint.z());
        if (!hasRobotResult)
        {
            error = QString("读取到的 PR[%1] 结果无效。").arg(kHandEyeRobotCheckPrIndex);
            QMessageBox::warning(this, "手眼标定", error);
            AppendLog("手眼参数检测失败：" + error);
            return false;
        }

        const Eigen::Vector3d delta = robotLaserPoint - laserPoint;
        robotDetailText = QString(
            "机器人程序结果（PR[%1]）：\n"
            "X=%2  Y=%3  Z=%4\n"
            "与本地矩阵差值：dX=%5  dY=%6  dZ=%7")
            .arg(kHandEyeRobotCheckPrIndex)
            .arg(robotLaserPoint.x(), 0, 'f', 3)
            .arg(robotLaserPoint.y(), 0, 'f', 3)
            .arg(robotLaserPoint.z(), 0, 'f', 3)
            .arg(delta.x(), 0, 'f', 3)
            .arg(delta.y(), 0, 'f', 3)
            .arg(delta.z(), 0, 'f', 3);
    }
    else if (fanucDriver != nullptr)
    {
        robotDetailText = "机器人程序结果：当前点检测已跳过，未执行 TP 对比。";
    }

    if (hasLocalLaserPoint)
    {
        AppendLog(QString("手眼参数检测：Robot=(%1, %2, %3, %4, %5, %6) Camera=(%7, %8, %9) LocalLaser=(%10, %11, %12)")
            .arg(robotPose.dX, 0, 'f', 3)
            .arg(robotPose.dY, 0, 'f', 3)
            .arg(robotPose.dZ, 0, 'f', 3)
            .arg(robotPose.dRX, 0, 'f', 3)
            .arg(robotPose.dRY, 0, 'f', 3)
            .arg(robotPose.dRZ, 0, 'f', 3)
            .arg(cameraPoint.x(), 0, 'f', 3)
            .arg(cameraPoint.y(), 0, 'f', 3)
            .arg(cameraPoint.z(), 0, 'f', 3)
            .arg(laserPoint.x(), 0, 'f', 3)
            .arg(laserPoint.y(), 0, 'f', 3)
            .arg(laserPoint.z(), 0, 'f', 3));
    }
    else
    {
        AppendLog(QString("手眼参数检测：当前点检测已跳过。Robot=(%1, %2, %3, %4, %5, %6)")
            .arg(robotPose.dX, 0, 'f', 3)
            .arg(robotPose.dY, 0, 'f', 3)
            .arg(robotPose.dZ, 0, 'f', 3)
            .arg(robotPose.dRX, 0, 'f', 3)
            .arg(robotPose.dRY, 0, 'f', 3)
            .arg(robotPose.dRZ, 0, 'f', 3));
    }
    if (hasRobotResult)
    {
        AppendLog(QString("机器人程序检测：PR[%1]=(%2, %3, %4)")
            .arg(kHandEyeRobotCheckPrIndex)
            .arg(robotLaserPoint.x(), 0, 'f', 3)
            .arg(robotLaserPoint.y(), 0, 'f', 3)
            .arg(robotLaserPoint.z(), 0, 'f', 3));
    }

    QString diagnosticPath;
    QString diagnosticSummary;
    QString diagnosticError;
    if (SaveHandEyeDiagnosticFile(
        m_robotName,
        m_cameraSection,
        "Test",
        checkConfig,
        matrix,
        &diagnosticPath,
        &diagnosticSummary,
        &diagnosticError))
    {
        AppendLog(QString("手眼参数检测诊断：%1").arg(diagnosticSummary));
        AppendLog(QString("手眼参数检测诊断文件：%1").arg(diagnosticPath));
    }
    else
    {
        AppendLog(QString("手眼参数检测诊断文件保存失败：%1").arg(diagnosticError));
    }

    AppendLog("手眼参数检测：每组采样目标点计算结果");
    for (const QString& line : sampleTargetLogLines)
    {
        AppendLog(line);
    }

    const QString cameraPointText = hasCurrentCameraPoint
        ? QString("X=%1  Y=%2  Z=%3")
            .arg(cameraPoint.x(), 0, 'f', 3)
            .arg(cameraPoint.y(), 0, 'f', 3)
            .arg(cameraPoint.z(), 0, 'f', 3)
        : QString("获取失败，已跳过当前点检测。\n原因：%1").arg(currentCameraError);

    const QString resultText = QString(
        "矩阵文件：%1\n\n"
        "当前位置：\n"
        "X=%2  Y=%3  Z=%4\n"
        "RX=%5  RY=%6  RZ=%7\n\n"
        "相机点：\n"
        "%8\n\n"
        "本地矩阵结果：\n"
        "%9\n\n"
        "每组采样目标点：\n"
        "%10\n\n"
        "%11")
        .arg(matrixFilePath)
        .arg(robotPose.dX, 0, 'f', 3)
        .arg(robotPose.dY, 0, 'f', 3)
        .arg(robotPose.dZ, 0, 'f', 3)
        .arg(robotPose.dRX, 0, 'f', 3)
        .arg(robotPose.dRY, 0, 'f', 3)
        .arg(robotPose.dRZ, 0, 'f', 3)
        .arg(cameraPointText)
        .arg(localLaserText)
        .arg(sampleTargetLines.join("\n"))
        .arg(robotDetailText);
    QMessageBox::information(this, "手眼参数检测", resultText);

    if (fanucDriver != nullptr && hasLocalLaserPoint)
    {
        const QString moveQuestion = QString(
            "是否运动到本地矩阵计算的激光点？\n\n"
            "目标点：\n"
            "X=%1  Y=%2  Z=%3\n\n"
            "运动方式：MOVL\n"
            "速度：500 mm/min\n"
            "姿态：保持当前姿态不变")
            .arg(laserPoint.x(), 0, 'f', 3)
            .arg(laserPoint.y(), 0, 'f', 3)
            .arg(laserPoint.z(), 0, 'f', 3);

        if (QMessageBox::question(
            this,
            "运动到激光点",
            moveQuestion,
            QMessageBox::Yes | QMessageBox::No,
            QMessageBox::No) == QMessageBox::Yes)
        {
            RobotDriverAdaptor::StateSnapshot snapshot;
            if (fanucDriver->LatestStateSnapshot(snapshot) && snapshot.done == 0)
            {
                const QString busyMessage = "机器人当前处于运动中，未执行运动到激光点。";
                QMessageBox::warning(this, "运动到激光点", busyMessage);
                AppendLog(busyMessage);
                return false;
            }

            T_ROBOT_COORS targetPose = robotPose;
            targetPose.dX = laserPoint.x();
            targetPose.dY = laserPoint.y();
            targetPose.dZ = laserPoint.z();

            AppendLog(QString("开始运动到激光点：X=%1 Y=%2 Z=%3，MOVL 500 mm/min。")
                .arg(targetPose.dX, 0, 'f', 3)
                .arg(targetPose.dY, 0, 'f', 3)
                .arg(targetPose.dZ, 0, 'f', 3));

            const bool moveOk = fanucDriver->MoveByJob(
                targetPose,
                T_ROBOT_MOVE_SPEED(500.0, 0.0, 0.0),
                fanucDriver->m_nExternalAxleType,
                "MOVL");
            const int done = moveOk ? fanucDriver->CheckRobotDone(100) : -1;
            if (!moveOk || done <= 0)
            {
                const QString moveError = QString("运动到激光点失败：Move=%1，CheckRobotDone=%2")
                    .arg(moveOk ? "OK" : "FAIL")
                    .arg(done);
                QMessageBox::warning(this, "运动到激光点", moveError);
                AppendLog(moveError);
                return false;
            }

            AppendLog("已运动到激光点。");
            QMessageBox::information(this, "运动到激光点", "已运动到激光点。");

            const QString returnQuestion = QString(
                "是否退回扫描位置？\n\n"
                "扫描位置：\n"
                "X=%1  Y=%2  Z=%3\n\n"
                "运动方式：MOVL\n"
                "速度：500 mm/min\n"
                "姿态：恢复检测开始时的扫描姿态")
                .arg(robotPose.dX, 0, 'f', 3)
                .arg(robotPose.dY, 0, 'f', 3)
                .arg(robotPose.dZ, 0, 'f', 3);

            if (QMessageBox::question(
                this,
                "退回扫描位置",
                returnQuestion,
                QMessageBox::Yes | QMessageBox::No,
                QMessageBox::No) == QMessageBox::Yes)
            {
                AppendLog(QString("开始退回扫描位置：X=%1 Y=%2 Z=%3，MOVL 500 mm/min。")
                    .arg(robotPose.dX, 0, 'f', 3)
                    .arg(robotPose.dY, 0, 'f', 3)
                    .arg(robotPose.dZ, 0, 'f', 3));

                const bool returnOk = fanucDriver->MoveByJob(
                    robotPose,
                    T_ROBOT_MOVE_SPEED(500.0, 0.0, 0.0),
                    fanucDriver->m_nExternalAxleType,
                    "MOVL");
                const int returnDone = returnOk ? fanucDriver->CheckRobotDone(100) : -1;
                if (!returnOk || returnDone <= 0)
                {
                    const QString returnError = QString("退回扫描位置失败：Move=%1，CheckRobotDone=%2")
                        .arg(returnOk ? "OK" : "FAIL")
                        .arg(returnDone);
                    QMessageBox::warning(this, "退回扫描位置", returnError);
                    AppendLog(returnError);
                    return false;
                }

                AppendLog("已退回扫描位置。");
                QMessageBox::information(this, "退回扫描位置", "已退回扫描位置。");
            }
        }
    }
    return true;
}

bool HandEyeCalibrationDialog::CheckCameraTimestampIntervals()
{
    QString error;
    Eigen::Vector3d cameraPoint = Eigen::Vector3d::Zero();
    if (!EnsureCameraReady("相机时间戳检测前检查", &cameraPoint, &error))
    {
        QMessageBox::warning(this, "相机时间戳检测", error);
        AppendLog("相机时间戳检测失败：" + error);
        return false;
    }

    if (m_pCameraCache == nullptr)
    {
        const QString message = "当前机器人没有可用的专属相机缓存，请确认机器人相机线程已初始化。";
        QMessageBox::warning(this, "相机时间戳检测", message);
        AppendLog("相机时间戳检测失败：" + message);
        return false;
    }

    CameraFrameCache* cache = m_pCameraCache;
    const std::uint64_t beginSequence = cache->Mark();
    AppendLog(QString("相机时间戳检测开始：采集 %1 ms，比较相机timestamp间隔和本机接收间隔。")
        .arg(kCameraTimestampCheckDurationMs));

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(kCameraTimestampCheckDurationMs);
    while (std::chrono::steady_clock::now() < deadline)
    {
        QApplication::processEvents(QEventLoop::AllEvents, 20);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    const std::uint64_t endSequence = cache->Mark();
    const std::vector<CameraFrameCache::TimedFrame> frames =
        cache->TimedFramesBetween(beginSequence, endSequence);

    if (frames.size() < 3)
    {
        const QString message = QString("采集到的相机帧不足：%1 帧，请确认相机数据在刷新。").arg(frames.size());
        QMessageBox::warning(this, "相机时间戳检测", message);
        AppendLog("相机时间戳检测失败：" + message);
        return false;
    }

    QVector<double> cameraDeltaMs;
    QVector<double> systemDeltaMs;
    QVector<double> ratioValues;
    int invalidCameraTimestampCount = 0;
    int invalidSystemTimestampCount = 0;
    int cameraBackwardsCount = 0;
    int cameraDuplicateCount = 0;
    int systemNonIncreasingCount = 0;

    QStringList csvLines;
    csvLines.push_back("index,sequence,camera_timestamp_us,system_recv_timestamp_us,camera_delta_us,system_delta_us,camera_to_system_ratio,target_x,target_y,target_z,error");

    for (int index = 0; index < static_cast<int>(frames.size()); ++index)
    {
        const CameraFrameCache::TimedFrame& frame = frames[static_cast<std::size_t>(index)];
        QString cameraDeltaText;
        QString systemDeltaText;
        QString ratioText;

        if (frame.cameraTimestampUs <= 0)
        {
            ++invalidCameraTimestampCount;
        }
        if (frame.receiveTimestampUs <= 0)
        {
            ++invalidSystemTimestampCount;
        }

        if (index > 0)
        {
            const CameraFrameCache::TimedFrame& prev = frames[static_cast<std::size_t>(index - 1)];
            const qint64 cameraDeltaUs = frame.cameraTimestampUs - prev.cameraTimestampUs;
            const qint64 systemDeltaUs = frame.receiveTimestampUs - prev.receiveTimestampUs;

            cameraDeltaText = QString::number(cameraDeltaUs);
            systemDeltaText = QString::number(systemDeltaUs);

            if (cameraDeltaUs < 0)
            {
                ++cameraBackwardsCount;
            }
            else if (cameraDeltaUs == 0)
            {
                ++cameraDuplicateCount;
            }

            if (systemDeltaUs <= 0)
            {
                ++systemNonIncreasingCount;
            }

            if (cameraDeltaUs > 0 && systemDeltaUs > 0)
            {
                const double ratio = static_cast<double>(cameraDeltaUs) / static_cast<double>(systemDeltaUs);
                ratioText = QString::number(ratio, 'f', 6);
                cameraDeltaMs.push_back(static_cast<double>(cameraDeltaUs) / 1000.0);
                systemDeltaMs.push_back(static_cast<double>(systemDeltaUs) / 1000.0);
                ratioValues.push_back(ratio);
            }
        }

        QStringList fields;
        fields
            << QString::number(index + 1)
            << QString::number(frame.sequence)
            << QString::number(frame.cameraTimestampUs)
            << QString::number(frame.receiveTimestampUs)
            << cameraDeltaText
            << systemDeltaText
            << ratioText
            << QString::number(frame.targetPoint.x, 'f', 6)
            << QString::number(frame.targetPoint.y, 'f', 6)
            << QString::number(frame.targetPoint.z, 'f', 6)
            << CsvEscape(frame.errorMessage);
        csvLines.push_back(fields.join(','));
    }

    const IntervalStats cameraStats = CalcIntervalStats(cameraDeltaMs);
    const IntervalStats systemStats = CalcIntervalStats(systemDeltaMs);
    const IntervalStats ratioStats = CalcIntervalStats(ratioValues);

    qint64 totalCameraUs = 0;
    qint64 totalSystemUs = 0;
    if (!frames.empty())
    {
        totalCameraUs = frames.back().cameraTimestampUs - frames.front().cameraTimestampUs;
        totalSystemUs = frames.back().receiveTimestampUs - frames.front().receiveTimestampUs;
    }
    const double totalRatio = totalSystemUs > 0
        ? static_cast<double>(totalCameraUs) / static_cast<double>(totalSystemUs)
        : 0.0;
    const double ratioCvPercent = std::abs(ratioStats.mean) > 1e-12
        ? ratioStats.stddev / std::abs(ratioStats.mean) * 100.0
        : 0.0;

    QString conclusion;
    if (ratioStats.count <= 0)
    {
        conclusion = "结论：没有可用的相邻帧间隔，无法判断。";
    }
    else if (std::abs(ratioStats.mean - 1.0) <= 0.05 && ratioCvPercent <= 5.0)
    {
        conclusion = "结论：相机timestamp增量和本机接收时间增量基本一致，按 us 使用的单位假设暂时成立。";
    }
    else if (ratioCvPercent <= 5.0)
    {
        conclusion = QString("结论：两组时间间隔比例比较稳定，但比例不是 1。相机timestamp可能有固定单位倍率，平均倍率=%1。")
            .arg(ratioStats.mean, 0, 'f', 6);
    }
    else
    {
        conclusion = QString("结论：两组时间间隔比例不稳定，可能存在缓存批量取帧、跳帧、相机timestamp异常或系统接收抖动。比例CV=%1%。")
            .arg(ratioCvPercent, 0, 'f', 2);
    }

    const QString csvPath = BuildCameraTimestampCheckPath(m_robotName, m_cameraSection);
    QString saveError;
    const bool saveOk = RobotDataHelper::SaveTextFileLines(csvPath, csvLines, &saveError);

    const QString resultText = QString(
        "采集帧数：%1\n"
        "序号范围：(%2, %3]\n"
        "相机总时长：%4 ms\n"
        "系统总时长：%5 ms\n"
        "总时长比例 camera/system：%6\n\n"
        "%7\n"
        "%8\n"
        "%9\n"
        "比例CV：%10%\n\n"
        "异常统计：camera无效=%11，camera倒退=%12，camera重复=%13，system无效=%14，system非递增=%15\n\n"
        "%16\n\n"
        "CSV：%17")
        .arg(frames.size())
        .arg(beginSequence)
        .arg(endSequence)
        .arg(static_cast<double>(totalCameraUs) / 1000.0, 0, 'f', 3)
        .arg(static_cast<double>(totalSystemUs) / 1000.0, 0, 'f', 3)
        .arg(totalRatio, 0, 'f', 6)
        .arg(FormatStatsLine("相机timestamp间隔", cameraStats, " ms"))
        .arg(FormatStatsLine("本机接收间隔", systemStats, " ms"))
        .arg(FormatStatsLine("camera/system间隔比例", ratioStats, "", 6))
        .arg(ratioCvPercent, 0, 'f', 2)
        .arg(invalidCameraTimestampCount)
        .arg(cameraBackwardsCount)
        .arg(cameraDuplicateCount)
        .arg(invalidSystemTimestampCount)
        .arg(systemNonIncreasingCount)
        .arg(conclusion)
        .arg(saveOk ? csvPath : QString("保存失败：%1").arg(saveError));

    AppendLog(QString("相机时间戳检测完成：帧数=%1，camera/system平均比例=%2，比例CV=%3%，CSV=%4")
        .arg(frames.size())
        .arg(ratioStats.mean, 0, 'f', 6)
        .arg(ratioCvPercent, 0, 'f', 2)
        .arg(saveOk ? csvPath : QString("保存失败")));
    QMessageBox::information(this, "相机时间戳检测", resultText);
    return saveOk;
}

bool HandEyeCalibrationDialog::UploadRobotHandEyeCheckProgram(QString* error)
{
    FANUCRobotCtrl* fanucDriver = CurrentFanucDriver(error);
    if (fanucDriver == nullptr)
    {
        return false;
    }

    const QString lsPath = FindHandEyeRobotCheckProgramPath();
    if (lsPath.isEmpty() || !QFileInfo::exists(lsPath))
    {
        if (error != nullptr)
        {
            *error = "未找到机器人手眼检测 LS 文件。";
        }
        return false;
    }

    const int uploadRet = fanucDriver->UploadLsFile(lsPath.toStdString(), "/md/");
    if (uploadRet != 0)
    {
        if (error != nullptr)
        {
            *error = QString("上传机器人手眼检测程序失败，返回=%1，文件=%2").arg(uploadRet).arg(lsPath);
        }
        return false;
    }

    AppendLog(QString("机器人手眼检测程序已上传：%1 -> %2").arg(lsPath, kHandEyeRobotCheckProgramName));
    return true;
}

void HandEyeCalibrationDialog::SetAutoCalibrationUiRunning(bool running)
{
    if (m_pUploadAutoProgramBtn != nullptr)
    {
        m_pUploadAutoProgramBtn->setEnabled(!running);
    }
    if (m_pAutoCalibrationBtn != nullptr)
    {
        m_pAutoCalibrationBtn->setEnabled(!running);
        m_pAutoCalibrationBtn->setText(running ? "自动标定进行中..." : "自动标定 变量10~16");
    }
    if (m_pAutoMoveSpeedSpin != nullptr)
    {
        m_pAutoMoveSpeedSpin->setEnabled(!running);
    }
    if (running)
    {
        SetAutoCalibrationStateText("自动标定状态：已启动，等待机器人到达位置变量[10]，已完成 0/6 组");
    }
    else if (m_pAutoStateLabel != nullptr
        && (m_pAutoStateLabel->text().contains("进行中")
            || m_pAutoStateLabel->text().contains("已启动")))
    {
        SetAutoCalibrationStateText("自动标定状态：待启动，已完成 0/6 组");
    }
}

void HandEyeCalibrationDialog::SetAutoCalibrationStateText(const QString& text)
{
    if (m_pAutoStateLabel != nullptr)
    {
        m_pAutoStateLabel->setText(text);
    }
}

bool HandEyeCalibrationDialog::ExportCalibrationReport(const HandEyeMatrixConfig& matrix, QString* reportPathOut, QString* error) const
{
    const QString reportPath = BuildHandEyeCalibrationReportPath(m_robotName, m_cameraSection);
    if (reportPathOut != nullptr)
    {
        *reportPathOut = reportPath;
    }

    QStringList lines;
    lines << QString("HandEyeCalibrationReport %1").arg(QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss"));
    lines << QString("RobotName %1").arg(m_robotName);
    lines << QString("CameraSection %1").arg(m_cameraSection);
    lines << QString("CalibrationIni %1").arg(GetHandEyeCalibrationIniPath(m_robotName, m_cameraSection));
    lines << QString("MatrixIni %1").arg(GetHandEyeMatrixIniPath(m_robotName, m_cameraSection));
    lines << QString("ReportFile %1").arg(reportPath);
    lines << "";
    lines << "[TargetPoint]";
    lines << FormatPoseSummary(m_config.tcpPoint);
    lines << "";
    lines << "[Samples]";
    for (int index = 0; index < m_config.samples.size(); ++index)
    {
        const HandEyeCalibrationSample& sample = m_config.samples[index];
        lines << QString("Sample %1 Valid=%2").arg(index + 1).arg(sample.valid ? 1 : 0);
        lines << QString("Robot %1").arg(FormatPoseSummary(sample.robotPose));
        lines << QString("Camera X=%1 Y=%2 Z=%3")
            .arg(sample.cameraPoint.x(), 0, 'f', 6)
            .arg(sample.cameraPoint.y(), 0, 'f', 6)
            .arg(sample.cameraPoint.z(), 0, 'f', 6);
        lines << "";
    }
    lines << "[Matrix]";
    for (int row = 0; row < 3; ++row)
    {
        lines << QString("R%1 %2 %3 %4")
            .arg(row)
            .arg(matrix.rotation(row, 0), 0, 'f', 12)
            .arg(matrix.rotation(row, 1), 0, 'f', 12)
            .arg(matrix.rotation(row, 2), 0, 'f', 12);
    }
    lines << QString("T %1 %2 %3")
        .arg(matrix.translation.x(), 0, 'f', 12)
        .arg(matrix.translation.y(), 0, 'f', 12)
        .arg(matrix.translation.z(), 0, 'f', 12);

    lines << "";
    lines.append(BuildHandEyeDiagnosticLines(m_config, matrix));

    return RobotDataHelper::SaveTextFileLines(reportPath, lines, error);
}

FANUCRobotCtrl* HandEyeCalibrationDialog::CurrentFanucDriver(QString* error) const
{
    RobotDriverAdaptor* driver = CurrentDriver(error);
    FANUCRobotCtrl* fanucDriver = dynamic_cast<FANUCRobotCtrl*>(driver);
    if (fanucDriver != nullptr)
    {
        return fanucDriver;
    }

    if (error != nullptr)
    {
        *error = QString("机器人 %1 当前不是 FANUC 驱动，自动标定只支持 FANUC。").arg(m_robotName);
    }
    return nullptr;
}

bool HandEyeCalibrationDialog::UploadAutoCalibrationProgram()
{
    if (m_bAutoCalibrationRunning.load())
    {
        QMessageBox::information(this, "手眼标定", "自动标定正在执行，请等待本次流程结束。");
        return false;
    }

    QString error;
    FANUCRobotCtrl* fanucDriver = CurrentFanucDriver(&error);
    if (fanucDriver == nullptr)
    {
        QMessageBox::warning(this, "手眼标定", error);
        AppendLog("发送自动标定程序失败：" + error);
        return false;
    }

    const QString lsPath = FindHandEyeAutoProgramPath();
    if (lsPath.isEmpty())
    {
        const QString message = "未找到自动标定程序文件：SDK/FANUC/FANUC_HECALIB.ls";
        QMessageBox::warning(this, "手眼标定", message);
        AppendLog(message);
        return false;
    }

    const QByteArray lsPathBytes = lsPath.toLocal8Bit();
    const int ret = fanucDriver->UploadLsFile(lsPathBytes.constData());
    const QString message = ret == 0
        ? QString("自动标定程序发送成功：%1 -> %2").arg(lsPath, kHandEyeAutoProgramName)
        : QString("自动标定程序发送失败，返回码=%1，文件=%2").arg(ret).arg(lsPath);
    AppendLog(message);
    if (ret == 0)
    {
        QMessageBox::information(this, "手眼标定", message);
        return true;
    }

    QMessageBox::warning(this, "手眼标定", message);
    return false;
}

bool HandEyeCalibrationDialog::StartAutoCalibration()
{
    if (m_bAutoCalibrationRunning.exchange(true))
    {
        QMessageBox::information(this, "手眼标定", "自动标定正在执行，请等待本次流程结束。");
        return false;
    }

    QString error;
    RobotDriverAdaptor* driver = CurrentDriver(&error);
    if (driver == nullptr)
    {
        m_bAutoCalibrationRunning.store(false);
        QMessageBox::warning(this, "手眼标定", error);
        AppendLog("自动标定启动失败：" + error);
        return false;
    }

    const auto answer = QMessageBox::question(
        this,
        "自动标定",
        "请确认已经完成以下准备：\n"
        "1. 位置变量[10] 示教为固定标定目标点。\n"
        "2. 位置变量[11]~[16] 示教为六组采样点。\n"
        "3. 相机实时三维点通信正常。\n"
        "4. 机器人服务已启动。\n"
        "5. 后续每次到位后，都会弹窗确认是否采集当前点。\n\n"
        "确认后上位机会逐点读取位置变量，调用通用 MOVL 运动，到位后读取当前位置并采集相机点。",
        QMessageBox::Yes | QMessageBox::No,
        QMessageBox::Yes);
    if (answer != QMessageBox::Yes)
    {
        m_bAutoCalibrationRunning.store(false);
        return false;
    }

    if (!EnsureCameraStarted("自动标定前检查", &error))
    {
        m_bAutoCalibrationRunning.store(false);
        QMessageBox::warning(this, "手眼标定", error);
        AppendLog("自动标定启动失败：" + error);
        return false;
    }

    QVector<HandEyeAutoTarget> targets;
    targets.reserve(kHandEyeAutoLastSampleStep - kHandEyeAutoTargetStep + 1);
    for (int varIndex = kHandEyeAutoTargetStep; varIndex <= kHandEyeAutoLastSampleStep; ++varIndex)
    {
        HandEyeAutoTarget target;
        if (!ReadAutoCalibrationTarget(driver, varIndex, target, &error))
        {
            m_bAutoCalibrationRunning.store(false);
            QMessageBox::warning(this, "手眼标定", error);
            AppendLog("自动标定启动失败：" + error);
            return false;
        }
        targets.push_back(target);
    }

    const double autoMoveSpeedMmPerMin = (m_pAutoMoveSpeedSpin != nullptr)
        ? m_pAutoMoveSpeedSpin->value()
        : kHandEyeAutoMoveSpeedMmPerMin;

    SetAutoCalibrationUiRunning(true);
    AppendLog(QString("自动标定已启动：机器人=%1，已读取位置变量[10]~[16]，MOVL速度=%2 mm/min。")
        .arg(m_robotName)
        .arg(autoMoveSpeedMmPerMin, 0, 'f', 1));

    QPointer<HandEyeCalibrationDialog> self(this);
    std::thread([self, driver, targets, autoMoveSpeedMmPerMin]()
        {
            using namespace std::chrono_literals;

            auto postLog = [self](const QString& message)
                {
                    QMetaObject::invokeMethod(qApp, [self, message]()
                        {
                            if (self == nullptr)
                            {
                                return;
                            }
                            self->AppendLog(message);
                        }, Qt::QueuedConnection);
                };

            auto postState = [self](const QString& stateText)
                {
                    QMetaObject::invokeMethod(qApp, [self, stateText]()
                        {
                            if (self == nullptr)
                            {
                                return;
                            }
                            self->SetAutoCalibrationStateText(stateText);
                        }, Qt::QueuedConnection);
                };

            auto confirmCapture = [self](int currentStep) -> bool
                {
                    bool confirmed = false;
                    QMetaObject::invokeMethod(qApp, [self, currentStep, &confirmed]()
                        {
                            if (self == nullptr)
                            {
                                return;
                            }

                            const QString message = QString(
                                "机器人已到达 %1。\n\n是否采集当前点数据？")
                                .arg(DescribeAutoCalibrationPoint(currentStep));

                            confirmed = (QMessageBox::question(
                                self,
                                "自动标定采集确认",
                                message,
                                QMessageBox::Yes | QMessageBox::No,
                                QMessageBox::Yes) == QMessageBox::Yes);
                        }, Qt::BlockingQueuedConnection);
                    return confirmed;
                };

            auto finish = [self](const QString& message, bool showWarning, bool showInfo)
                {
                    QMetaObject::invokeMethod(qApp, [self, message, showWarning, showInfo]()
                        {
                            if (self == nullptr)
                            {
                                return;
                            }
                            self->SetAutoCalibrationUiRunning(false);
                            self->m_bAutoCalibrationRunning.store(false);
                            self->SetAutoCalibrationStateText("自动标定状态：待启动");
                            self->AppendLog(message);
                            if (showWarning)
                            {
                                QMessageBox::warning(self, "手眼标定", message);
                            }
                            else if (showInfo)
                            {
                                QMessageBox::information(self, "手眼标定", message);
                            }
                        }, Qt::QueuedConnection);
                };

            for (const HandEyeAutoTarget& target : targets)
            {
                if (self == nullptr)
                {
                    return;
                }

                postState(QString("自动标定状态：正在移动到%1。").arg(DescribeAutoCalibrationPoint(target.varIndex)));
                postLog(QString("开始移动到%1：%2")
                    .arg(DescribeAutoCalibrationPoint(target.varIndex))
                    .arg(FormatPoseSummary(target.target)));

                int config[7] = {};
                for (int i = 0; i < 7; ++i)
                {
                    config[i] = target.config[static_cast<size_t>(i)];
                }

                const bool moveOk = driver->MoveByJob(
                    target.target,
                    T_ROBOT_MOVE_SPEED(autoMoveSpeedMmPerMin, 0.0, 0.0),
                    driver->m_nExternalAxleType,
                    "MOVL",
                    1,
                    config);
                if (!moveOk)
                {
                    const QString detail = DecodeRobotMessageText(driver->GetLastRobotError());
                    finish(QString("自动标定移动失败：%1。%2")
                        .arg(DescribeAutoCalibrationPoint(target.varIndex))
                        .arg(detail.isEmpty() ? QString() : QString("原因：%1").arg(detail)), true, false);
                    return;
                }

                QString waitError;
                if (!WaitGenericRobotDone(driver, kHandEyeAutoMoveDoneTimeoutMs, 100, &waitError, &target.target, postLog))
                {
                    finish(QString("自动标定等待到位失败：%1，%2").arg(DescribeAutoCalibrationPoint(target.varIndex), waitError), true, false);
                    return;
                }

                std::this_thread::sleep_for(200ms);
                postState(FormatAutoCalibrationState(target.varIndex));
                postLog(QString("机器人已到达%1，等待确认是否采集...").arg(DescribeAutoCalibrationPoint(target.varIndex)));

                if (!confirmCapture(target.varIndex))
                {
                    finish(QString("自动标定已中止：用户取消采集 %1。").arg(DescribeAutoCalibrationPoint(target.varIndex)), true, false);
                    return;
                }

                const T_ROBOT_COORS robotPose = driver->GetCurrentPos();
                if (!IsFinitePose(robotPose))
                {
                    finish(QString("自动标定采样失败：%1 到位后读取当前位置无效。").arg(DescribeAutoCalibrationPoint(target.varIndex)), true, false);
                    return;
                }

                QString applyError;
                bool applyOk = false;
                if (target.varIndex == kHandEyeAutoTargetStep)
                {
                    QMetaObject::invokeMethod(self.data(), [&applyOk, &applyError, self, robotPose]()
                        {
                            if (self == nullptr)
                            {
                                applyError = "界面已关闭，无法写入固定标定目标点。";
                                return;
                            }
                            applyOk = self->ApplyCapturedTargetPoint(robotPose, &applyError);
                        }, Qt::BlockingQueuedConnection);
                }
                else
                {
                    Eigen::Vector3d cameraPoint = Eigen::Vector3d::Zero();
                    if (!self->ReadLatestCameraPoint(cameraPoint, &applyError))
                    {
                        finish(QString("自动标定采样失败：%1 的相机点无效：%2")
                            .arg(DescribeAutoCalibrationPoint(target.varIndex), applyError), true, false);
                        return;
                    }

                    const int sampleIndex = target.varIndex - kHandEyeAutoFirstSampleStep;
                    QMetaObject::invokeMethod(self.data(), [&applyOk, &applyError, self, sampleIndex, robotPose, cameraPoint]()
                        {
                            if (self == nullptr)
                            {
                                applyError = "界面已关闭，无法写入自动采样数据。";
                                return;
                            }
                            applyOk = self->ApplyCapturedSample(sampleIndex, robotPose, cameraPoint, &applyError);
                        }, Qt::BlockingQueuedConnection);
                }

                if (!applyOk)
                {
                    finish(QString("自动标定写入失败：%1 -> %2").arg(DescribeAutoCalibrationPoint(target.varIndex), applyError), true, false);
                    return;
                }

                postLog(QString("%1 采样完成。").arg(DescribeAutoCalibrationPoint(target.varIndex)));
            }

            postState(FormatAutoCalibrationState(kHandEyeAutoDoneStep));
            bool computeOk = false;
            QMetaObject::invokeMethod(self.data(), [&computeOk, self]()
                {
                    if (self == nullptr)
                    {
                        return;
                    }
                    computeOk = self->ComputeAndSaveMatrix();
                }, Qt::BlockingQueuedConnection);

            if (computeOk)
            {
                finish("自动标定完成，位置变量[10]~[16] 采样和矩阵计算均已写入。", false, false);
            }
            else
            {
                finish("自动标定采样完成，但矩阵计算失败，请检查采样数据。", false, false);
            }
        }).detach();

    return true;
}

void HandEyeCalibrationDialog::OpenMatrixDialog()
{
    DelayedLoadingGuard loading(this, "正在打开手眼矩阵参数", 1000);
    HandEyeMatrixDialog dialog(m_pContralUnit, m_robotName, m_cameraSection, this);
    loading.Pulse();
    loading.Finish();
    dialog.exec();
    UpdatePathLabels();
}

bool HandEyeCalibrationDialog::ReadLatestCameraPoint(Eigen::Vector3d& cameraPoint, QString* error) const
{
    if (m_pCameraCache == nullptr)
    {
        if (error != nullptr)
        {
            *error = "当前机器人没有可用的专属相机缓存，请确认机器人相机线程已初始化。";
        }
        return false;
    }

    udpDataShow frame;
    if (!m_pCameraCache->Latest(frame))
    {
        if (error != nullptr)
        {
            *error = "当前没有可用的相机三维点，请先确认相机通信正常。";
        }
        return false;
    }

    cameraPoint = Eigen::Vector3d(frame.targetPoint.x, frame.targetPoint.y, frame.targetPoint.z);
    if (!std::isfinite(cameraPoint.x()) || !std::isfinite(cameraPoint.y()) || !std::isfinite(cameraPoint.z()))
    {
        if (error != nullptr)
        {
            *error = "相机三维点无效，存在 NaN/Inf。";
        }
        return false;
    }

    if (std::abs(cameraPoint.x()) < 1e-6
        && std::abs(cameraPoint.y()) < 1e-6
        && std::abs(cameraPoint.z()) < 1e-6)
    {
        if (error != nullptr)
        {
            *error = "相机三维点为 0,0,0，当前帧不可用于标定。";
        }
        return false;
    }

    return true;
}

RobotDriverAdaptor* HandEyeCalibrationDialog::CurrentDriver(QString* error) const
{
    const QVector<RobotDataHelper::RobotInfo> robots = RobotDataHelper::LoadRobotList(m_pContralUnit);
    for (const RobotDataHelper::RobotInfo& info : robots)
    {
        if (info.robotName == m_robotName)
        {
            RobotDriverAdaptor* driver = RobotDataHelper::GetRobotDriver(m_pContralUnit, info.unitIndex);
            if (driver != nullptr)
            {
                return driver;
            }
            break;
        }
    }

    if (error != nullptr)
    {
        *error = QString("未找到机器人 %1 的驱动，请先确认控制单元已经创建。").arg(m_robotName);
    }
    return nullptr;
}

void HandEyeCalibrationDialog::SetRobotPoseEditors(QVector<QLineEdit*>& edits, const T_ROBOT_COORS& pose)
{
    if (edits.isEmpty())
    {
        return;
    }

    const QVector<double> values = {
        pose.dX, pose.dY, pose.dZ,
        pose.dRX, pose.dRY, pose.dRZ,
        pose.dBX, pose.dBY, pose.dBZ
    };
    for (int index = 0; index < edits.size() && index < values.size(); ++index)
    {
        edits[index]->setText(FormatDouble(values[index]));
    }
}

T_ROBOT_COORS HandEyeCalibrationDialog::ReadRobotPoseEditors(const QVector<QLineEdit*>& edits, bool* ok, QString* error) const
{
    T_ROBOT_COORS pose;
    if (ok != nullptr)
    {
        *ok = false;
    }
    if (edits.size() != 3 && edits.size() != 6 && edits.size() != 9)
    {
        if (error != nullptr)
        {
            *error = QString("位姿编辑框数量异常：%1").arg(edits.size());
        }
        return pose;
    }

    QVector<double> values;
    values.reserve(edits.size());
    for (int index = 0; index < edits.size(); ++index)
    {
        bool parseOk = false;
        const double value = edits[index]->text().trimmed().toDouble(&parseOk);
        if (!parseOk)
        {
            if (error != nullptr)
            {
                *error = QString("第 %1 个输入不是有效数字。").arg(index + 1);
            }
            return pose;
        }
        values.push_back(value);
    }

    pose.dX = values[0];
    pose.dY = values[1];
    pose.dZ = values[2];
    if (values.size() >= 6)
    {
        pose.dRX = values[3];
        pose.dRY = values[4];
        pose.dRZ = values[5];
    }
    else
    {
        pose.dRX = 0.0;
        pose.dRY = 0.0;
        pose.dRZ = 0.0;
    }

    if (values.size() >= 9)
    {
        pose.dBX = values[6];
        pose.dBY = values[7];
        pose.dBZ = values[8];
    }
    else
    {
        pose.dBX = 0.0;
        pose.dBY = 0.0;
        pose.dBZ = 0.0;
    }

    if (ok != nullptr)
    {
        *ok = true;
    }
    return pose;
}

void HandEyeCalibrationDialog::SetVectorEditors(QVector<QLineEdit*>& edits, const Eigen::Vector3d& point)
{
    if (edits.size() < 3)
    {
        return;
    }

    edits[0]->setText(FormatDouble(point.x()));
    edits[1]->setText(FormatDouble(point.y()));
    edits[2]->setText(FormatDouble(point.z()));
}

Eigen::Vector3d HandEyeCalibrationDialog::ReadVectorEditors(const QVector<QLineEdit*>& edits, bool* ok, QString* error) const
{
    if (ok != nullptr)
    {
        *ok = false;
    }
    if (edits.size() < 3)
    {
        if (error != nullptr)
        {
            *error = "三维点编辑框数量不足。";
        }
        return Eigen::Vector3d::Zero();
    }

    bool xOk = false;
    bool yOk = false;
    bool zOk = false;
    const double x = edits[0]->text().trimmed().toDouble(&xOk);
    const double y = edits[1]->text().trimmed().toDouble(&yOk);
    const double z = edits[2]->text().trimmed().toDouble(&zOk);
    if (!(xOk && yOk && zOk))
    {
        if (error != nullptr)
        {
            *error = "三维点输入包含无效数字。";
        }
        return Eigen::Vector3d::Zero();
    }

    if (ok != nullptr)
    {
        *ok = true;
    }
    return Eigen::Vector3d(x, y, z);
}

void HandEyeCalibrationDialog::UpdatePathLabels()
{
    QString calibrationError;
    QString calibrationPath;
    if (EnsureHandEyeCalibrationIni(m_robotName, m_cameraSection, &calibrationError, &calibrationPath))
    {
        m_pCalibrationPathLabel->setText(QString("标定文件：%1").arg(calibrationPath));
    }
    else
    {
        m_pCalibrationPathLabel->setText("标定文件：创建失败");
        AppendLog("标定文件准备失败：" + calibrationError);
    }

    QString matrixError;
    QString matrixPath;
    if (EnsureHandEyeMatrixIni(m_robotName, m_cameraSection, &matrixError, &matrixPath))
    {
        m_pMatrixPathLabel->setText(QString("矩阵文件：%1").arg(matrixPath));
    }
    else
    {
        m_pMatrixPathLabel->setText("矩阵文件：创建失败");
        AppendLog("矩阵文件准备失败：" + matrixError);
    }

    if (m_pReportPathLabel != nullptr)
    {
        m_pReportPathLabel->setText(QString("报告文件：%1").arg(BuildHandEyeCalibrationReportPath(m_robotName, m_cameraSection)));
    }
}

void HandEyeCalibrationDialog::UpdateSampleStates()
{
    for (int index = 0; index < m_sampleWidgets.size() && index < m_config.samples.size(); ++index)
    {
        if (m_sampleWidgets[index].pStateLabel != nullptr)
        {
            m_sampleWidgets[index].pStateLabel->setText(SampleStateText(m_config.samples[index]));
        }
    }
}

void HandEyeCalibrationDialog::SelectSampleTab(int index)
{
    if (m_pSampleTabWidget == nullptr)
    {
        return;
    }
    if (index < 0 || index >= m_pSampleTabWidget->count())
    {
        return;
    }
    m_pSampleTabWidget->setCurrentIndex(index);
}

bool HandEyeCalibrationDialog::HasUnsavedChanges() const
{
    return BuildSnapshot() != m_cleanSnapshot;
}

QString HandEyeCalibrationDialog::BuildSnapshot() const
{
    QStringList fields;
    fields << m_robotName << m_cameraSection;
    for (QLineEdit* edit : m_tcpEdits)
    {
        fields << (edit != nullptr ? edit->text().trimmed() : QString());
    }
    for (const SampleWidgets& sample : m_sampleWidgets)
    {
        for (QLineEdit* edit : sample.robotEdits)
        {
            fields << (edit != nullptr ? edit->text().trimmed() : QString());
        }
        for (QLineEdit* edit : sample.cameraEdits)
        {
            fields << (edit != nullptr ? edit->text().trimmed() : QString());
        }
    }
    return fields.join('\n');
}

void HandEyeCalibrationDialog::MarkCleanSnapshot()
{
    m_cleanSnapshot = BuildSnapshot();
}

void HandEyeCalibrationDialog::AppendLog(const QString& text)
{
    qDebug().noquote() << "[HandEyeCalibration]" << text;

    if (m_pLogText != nullptr)
    {
        m_pLogText->appendPlainText(text);
    }

    // 统一走 RobotLog：按天归档 Log/<日期>/、线程安全、自动毫秒时间戳。
    static RobotLog fileLogger("Log/HandEyeCalibrationLog.txt", false);
    fileLogger.writeLine(text.toStdString());
}
