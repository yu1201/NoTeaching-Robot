#pragma once

#include "ContralUnit.h"
#include "RobotDriverAdaptor.h"

#include <QDialog>
#include <initializer_list>
#include <QString>
#include <QStringList>
#include <QVector>
#include <functional>
#include <memory>

class QPushButton;
class QPlainTextEdit;
class QTimer;
class QDoubleSpinBox;
class QSpinBox;
class QTableWidget;
class QTabWidget;
class QLabel;
class QListWidget;
class QStackedWidget;
class QComboBox;
class CameraFrameCache;
class QWidget;
class RobotOperationLease;

// 功能测试子界面：把主界面上的机器人调试/运动测试按钮集中管理。
class FunctionTestDialog : public QDialog
{
public:
    explicit FunctionTestDialog(
        ContralUnit* pContralUnit,
        int unitIndex = 0,
        CameraFrameCache* cameraCache = nullptr,
        QWidget* parent = nullptr,
        std::function<bool(const QString&, int)> workflowLauncher = {},
        std::function<CameraFrameCache*(int)> cameraCacheResolver = {});
    bool RunDashboardTool(const QString& actionId);
    void ShowAdaptorAcceptancePage();
    void ShowSingleTestPage();

protected:
    void closeEvent(QCloseEvent* event) override;

private:
    RobotDriverAdaptor* GetFirstDriverWithCapability(
        RobotDriverCapability capability,
        const QString& actionName);
    RobotDriverAdaptor* GetFirstDriverWithCapabilities(
        std::initializer_list<RobotDriverCapability> capabilities,
        const QString& actionName);
    RobotDriverAdaptor* GetFirstRobotDriverAdaptor();
    bool IsMotionBusy() const;
    void RefreshMotionButtonState();
    void AppendLog(const QString& text);
    QString EnsureKinematicsSampleFilePath();

    void FanucGetCurrentPosTest();
    void FanucGetCurrentPulseTest();
    void FanucCheckDoneTest();
    void FanucSetGetIntTest();
    void FanucSetTpSpeedTest();
    void FanucCallJobTest();
    void FanucUploadLsTest();
    void FanucCurposDiagnosticTest();
    void RobotCameraTimestampDiagnosticTest();
    void FanucMovlTest();
    void FanucMovjTest();
    void FanucMoveZeroTest();
    void EditKinematicsParameters();
    void FanucCaptureKinematicsSample();
    void FitDhParametersFromSamples();
    void OpenLaserWeldFilterTest();
    void ExportCurrentCameraFramePointFilterTest();
    QWidget* CreateAdaptorAcceptancePage();
    void StartNewAdaptorAcceptanceRun();
    void LoadAdaptorAcceptanceRun(const QString& requestedRunId = QString());
    bool SaveAdaptorAcceptanceRun();
    bool BeginAdaptorAcceptanceStage(int stage);
    void RefreshAdaptorAcceptanceUi();
    void ExecuteAdaptorAcceptanceStage();
    void MarkAdaptorAcceptanceStage(const QString& state, const QString& evidence = QString());
    void FinishAdaptorAcceptanceStage(int stage, bool success, const QString& evidence, bool manualConfirmation = false);
    void RunAdaptorConnectionTest();
    void RunAdaptorFtpRoundTrip();
    void RunAdaptorProgramDownlink();
    void RunAdaptorPositionStatusCheck();
    void RunAdaptorSafeLinearMotion(bool jointMotion = false);
    void RunAdaptorModeCombinationTests(bool allCases);
    void RunAdaptorInterfaceMatrix();
    bool CheckAdaptorRegisterRecovery();
    bool SaveAdaptorRegisterProgress(const QString& evidence, const QString& recovery);
    bool ConfirmAdaptorRegisterValue(RobotDriverAdaptor* driver, const QString& description);
    void RunAdaptorAssetAudit();
    void RunAdaptorTwoToThreeCheck();
    void OpenAdaptorWorkflow(const QString& workflowId);
    void FinalizeAdaptorAcceptance();
    void ChangeAdaptorAcceptanceRobot(int comboIndex);
    std::uint64_t AdaptorAcceptanceRequiredMask(int stage) const;
    bool AdaptorAcceptancePrerequisitesReady(int stage, QString* reason = nullptr) const;
    QString AdaptorAcceptanceStorageRobotName() const;

private:
    ContralUnit* m_pContralUnit = nullptr;
    int m_unitIndex = 0;
    CameraFrameCache* m_pCameraCache = nullptr;
    QPushButton* m_pMovlTestBtn = nullptr;
    QPushButton* m_pMovjTestBtn = nullptr;
    QPushButton* m_pMoveZeroBtn = nullptr;
    QPlainTextEdit* m_pLogText = nullptr;
    QWidget* m_pCommandContent = nullptr;
    QTimer* m_pMotionStateTimer = nullptr;
    QVector<QPushButton*> m_motionButtons;
    QString m_kinematicsSampleFilePath;
    int m_kinematicsSampleCount = 0;

    bool m_bFanucMovlForward = true;
    bool m_bFanucMovlRunning = false;
    bool m_bFanucMovjRunning = false;
    bool m_bFanucMoveZeroRunning = false;
    bool m_bRobotCommandRunning = false;
    std::function<bool(const QString&, int)> m_workflowLauncher;
    std::function<CameraFrameCache*(int)> m_cameraCacheResolver;
    QLabel* m_pPageTitleLabel = nullptr;
    QLabel* m_pPageHintLabel = nullptr;
    QTabWidget* m_pTestTabs = nullptr;
    QWidget* m_pAdaptorAcceptancePage = nullptr;
    QLabel* m_pAdaptorAcceptanceTitleLabel = nullptr;
    QComboBox* m_pAdaptorAcceptanceRobotCombo = nullptr;
    QComboBox* m_pAdaptorAcceptanceRunCombo = nullptr;
    QLabel* m_pAdaptorAcceptanceSaveStatus = nullptr;
    QTimer* m_pAdaptorAcceptanceSaveTimer = nullptr;
    QLabel* m_pAdaptorAcceptanceRobotSummary = nullptr;
    QListWidget* m_pAdaptorAcceptanceStageList = nullptr;
    QStackedWidget* m_pAdaptorAcceptanceStageStack = nullptr;
    QLabel* m_pAdaptorAcceptanceStageTitle = nullptr;
    QLabel* m_pAdaptorAcceptanceStageDescription = nullptr;
    QLabel* m_pAdaptorAcceptanceStageGate = nullptr;
    QLabel* m_pAdaptorAcceptanceStageStatus = nullptr;
    QPlainTextEdit* m_pAdaptorAcceptanceEvidence = nullptr;
    QPushButton* m_pAdaptorAcceptanceExecuteBtn = nullptr;
    QPushButton* m_pAdaptorAcceptancePassBtn = nullptr;
    QPushButton* m_pAdaptorAcceptanceFailBtn = nullptr;
    QPushButton* m_pAdaptorAcceptanceSkipBtn = nullptr;
    QPushButton* m_pAdaptorAcceptanceNewRunBtn = nullptr;
    QPushButton* m_pAdaptorAcceptanceReportBtn = nullptr;
    QDoubleSpinBox* m_pAdaptorAcceptanceDistanceSpin = nullptr;
    QDoubleSpinBox* m_pAdaptorAcceptanceSpeedSpin = nullptr;
    QPushButton* m_pAdaptorJointMotionBtn = nullptr;
    QPushButton* m_pAdaptorJointCancelBtn = nullptr;
    QLabel* m_pAdaptorJointMotionStatus = nullptr;
    QString m_adaptorJointMotionState = QStringLiteral("pending");
    QString m_adaptorJointMotionEvidence;
    T_ANGLE_PULSE m_adaptorJointOriginalPulse;
    bool m_adaptorJointOriginalPulseValid = false;
    bool m_adaptorJointMovedOut = false;
    std::shared_ptr<RobotOperationLease> m_adaptorJointLease;
    QComboBox* m_pAdaptorModeCombinationCombo = nullptr;
    QPushButton* m_pAdaptorModeSingleBtn = nullptr;
    QPushButton* m_pAdaptorModeBatchBtn = nullptr;
    QPushButton* m_pAdaptorModeApplyBtn = nullptr;
    QPushButton* m_pAdaptorModeStopBtn = nullptr;
    QString m_adaptorModeCombinationEvidence;
    bool m_adaptorModeBatchRunning = false;
    QDoubleSpinBox* m_pAdaptorAcceptanceProgramSpeedSpin = nullptr;
    QSpinBox* m_pAdaptorAcceptanceIntIndexSpin = nullptr;
    QSpinBox* m_pAdaptorAcceptanceRealIndexSpin = nullptr;
    QSpinBox* m_pAdaptorAcceptanceToolIndexSpin = nullptr;
    QDoubleSpinBox* m_pAdaptorAcceptanceTwoToThreeToleranceSpin = nullptr;
    int m_adaptorAcceptanceSelectedStage = -1;
    QString m_adaptorAcceptanceRunId;
    QString m_adaptorEvidencePlanRevision;
    QString m_adaptorRegisterRecovery;
    QStringList m_adaptorAcceptanceStates;
    QStringList m_adaptorAcceptanceEvidence;
    T_ROBOT_COORS m_adaptorAcceptanceOriginalPose;
    bool m_adaptorAcceptanceOriginalPoseValid = false;
    bool m_adaptorAcceptanceMovedOut = false;
    bool m_adaptorAcceptanceMotionRoundTripCompleted = false;
    bool m_adaptorAcceptanceBusy = false;
    bool m_adaptorAcceptanceLoading = false;
    bool m_adaptorAcceptanceRecordLoaded = false;
    bool m_adaptorAcceptanceStandaloneMode = false;
};
