#pragma once

#include "ContralUnit.h"
#include "HandEyeMatrixConfig.h"
#include "RobotDriverAdaptor.h"

#include <QDialog>
#include <QPointer>
#include <QVector>

#include <Eigen/Dense>
#include <atomic>
#include <functional>
#include <initializer_list>

class QLabel;
class QCloseEvent;
class QDoubleSpinBox;
class QLineEdit;
class QPlainTextEdit;
class QPushButton;
class QTabWidget;
class CameraFrameCache;

class HandEyeCalibrationDialog : public QDialog
{
public:
    using StartCameraFunc = std::function<bool(QString&)>;
    using StopCameraFunc = std::function<void()>;

    explicit HandEyeCalibrationDialog(
        ContralUnit* pContralUnit,
        const QString& robotName,
        const QString& cameraSection,
        StartCameraFunc startCamera,
        StopCameraFunc stopCamera,
        CameraFrameCache* cameraCache,
        QWidget* parent = nullptr);
    bool MatrixComputedThisSession() const { return m_bMatrixComputedThisSession; }

private:
    void closeEvent(QCloseEvent* event) override;
    struct SampleWidgets
    {
        QLabel* pStateLabel = nullptr;
        QVector<QLineEdit*> robotEdits;
        QVector<QLineEdit*> cameraEdits;
    };

    bool LoadConfig();
    bool SaveConfig();
    bool SaveConfigSilently(QString* error = nullptr);
    bool CaptureTcpPoint();
    bool CaptureSample(int index);
    bool ComputeAndSaveMatrix(bool showDialogs = true);
    bool TestHandEyeMatrix();
    bool StartRobotPoseMove(
        const T_ROBOT_COORS& targetPose,
        const QString& actionName,
        bool offerReturn,
        const T_ROBOT_COORS& returnPose);
    void RequestSafetyStop();
    bool CheckCameraTimestampIntervals();
    bool UploadRobotHandEyeCheckProgram(QString* error = nullptr);
    bool UploadAutoCalibrationProgram();
    bool StartAutoCalibration();
    bool EnsureCameraReady(const QString& sceneName, Eigen::Vector3d* cameraPointOut = nullptr, QString* error = nullptr);
    bool EnsureCameraStarted(const QString& sceneName, QString* error = nullptr);
    void OpenMatrixDialog();

    bool ReadLatestCameraPoint(Eigen::Vector3d& cameraPoint, QString* error = nullptr) const;
    RobotDriverAdaptor* CurrentDriver(QString* error = nullptr) const;
    RobotDriverAdaptor* CurrentDriverWithCapability(
        RobotDriverCapability capability,
        const QString& actionName,
        QString* error = nullptr) const;
    RobotDriverAdaptor* CurrentDriverWithCapabilities(
        std::initializer_list<RobotDriverCapability> capabilities,
        const QString& actionName,
        QString* error = nullptr) const;

    bool ApplyCapturedTargetPoint(const T_ROBOT_COORS& pose, QString* error = nullptr);
    bool ApplyCapturedSample(int index, const T_ROBOT_COORS& pose, const Eigen::Vector3d& cameraPoint, QString* error = nullptr);
    void SetAutoCalibrationUiRunning(bool running);
    void SetRobotTestUiRunning(bool running);
    void RefreshBusyInteractionState();
    void ApplyRobotTaskTerminalState(
        RobotDriverAdaptor* driver,
        bool verifiedTerminal,
        const QString& statusText);
    void SetAutoCalibrationStateText(const QString& text);
    void InvalidateLastTestActions();
    bool ExportCalibrationReport(const HandEyeMatrixConfig& matrix, QString* reportPathOut = nullptr, QString* error = nullptr) const;

    void SetRobotPoseEditors(QVector<QLineEdit*>& edits, const T_ROBOT_COORS& pose);
    T_ROBOT_COORS ReadRobotPoseEditors(const QVector<QLineEdit*>& edits, bool* ok = nullptr, QString* error = nullptr) const;
    void SetVectorEditors(QVector<QLineEdit*>& edits, const Eigen::Vector3d& point);
    Eigen::Vector3d ReadVectorEditors(const QVector<QLineEdit*>& edits, bool* ok = nullptr, QString* error = nullptr) const;

    void UpdatePathLabels();
    void UpdateSampleStates();
    void SelectSampleTab(int index);
    bool HasUnsavedChanges() const;
    QString BuildSnapshot() const;
    void MarkCleanSnapshot();
    void AppendLog(const QString& text);

    ContralUnit* m_pContralUnit = nullptr;
    QString m_robotName;
    QString m_cameraSection;
    HandEyeCalibrationConfig m_config;
    StartCameraFunc m_startCamera;
    StopCameraFunc m_stopCamera;
    CameraFrameCache* m_pCameraCache = nullptr;

    QLabel* m_pAutoStateLabel = nullptr;
    QLabel* m_pCalibrationPathLabel = nullptr;
    QLabel* m_pMatrixPathLabel = nullptr;
    QLabel* m_pReportPathLabel = nullptr;
    QDoubleSpinBox* m_pAutoMoveSpeedSpin = nullptr;
    QPlainTextEdit* m_pLogText = nullptr;
    QWidget* m_pScrollContent = nullptr;
    QPushButton* m_pUploadAutoProgramBtn = nullptr;
    QPushButton* m_pAutoCalibrationBtn = nullptr;
    QPushButton* m_pTestHandEyeBtn = nullptr;
    QPushButton* m_pMoveToLastTestPointBtn = nullptr;
    QPushButton* m_pReturnToLastTestPoseBtn = nullptr;
    QPushButton* m_pSafetyStopBtn = nullptr;
    RobotDriverAdaptor* m_pSafetyStopDriver = nullptr;
    QString m_activeRobotTaskOwner;
    QPointer<QDialog> m_pCaptureConfirmation;
    QTabWidget* m_pSampleTabWidget = nullptr;
    QVector<QLineEdit*> m_tcpEdits;
    QVector<SampleWidgets> m_sampleWidgets;
    std::atomic_bool m_bAutoCalibrationRunning = false;
    std::atomic_bool m_bRobotTestRunning = false;
    std::atomic_bool m_bSafetyStopRunning = false;
    std::atomic_bool m_bSafetyStopRequested = false;
    T_ROBOT_COORS m_lastTestMoveTarget;
    T_ROBOT_COORS m_lastTestReturnPose;
    bool m_hasLastTestMoveTarget = false;
    bool m_hasLastTestReturnPose = false;
    bool m_bMatrixComputedThisSession = false;
    QString m_cleanSnapshot;
};
