#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_QtWidgetsApplication4.h"
#include "ContralUnit.h"
#include "ProcessLoopTestDialog.h"   // 流程循环测试：设置/默认结构体 + 回调类型

#include <atomic>
#include <QDateTime>
#include <QHash>
#include <QList>
#include <QPointer>
#include <QSet>
#include <QString>
#include <QStringList>

#include <limits>

class ClientUDPFormSensorWorker;
class ScanCameraTcpClientWorker;
class CameraFrameCache;
class CameraParamDialog;
class SKJCameraControlClient;
class FANUCRobotCtrl;
class FunctionTestDialog;
class LaserWeldFilterDialog;
class WorkpieceMeshViewerDialog;
class ModelAlignmentDialog;
class VirtualWeldTestDialog;
class ScanDataUploader;
class QAction;
class QCheckBox;
class QCloseEvent;
class MeasureThenWeldDialog;
class PreciseMeasureEditDialog;
class QComboBox;
class QEvent;
class QLabel;
class QLineEdit;
class QPlainTextEdit;
class QPushButton;
class QResizeEvent;
class QStackedWidget;
class QThread;
class QTimer;
class RobotDriverAdaptor;
class RobotJogDialog;
class TouchKeyboardManager;
class WeldProcessDialog;
class WeldSeamCompDialog;

class QtWidgetsApplication4 : public QMainWindow
{
    Q_OBJECT

public:
    QtWidgetsApplication4(QWidget *parent = nullptr);
    ~QtWidgetsApplication4();
    void ApplyStartupArguments(const QStringList& arguments);
    void CheckPendingUpdateResult();   // GUI 启动自检：升级后实际版本未达目标则告警

protected:
    bool eventFilter(QObject* watched, QEvent* event) override;
    void closeEvent(QCloseEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;

private slots:
    void RobotRunTest();
    void OpenWeldProcessDialog();
    void OpenFunctionTestDialog();
    void OpenMeasureThenWeldDialog();
    void OpenPreciseMeasureEditDialog();
    void OpenPositionTeachDialog();
    void OpenWeldSeamCompDialog();
    void OpenCameraParamDialog();
    void OpenResultArchiveDialog();
    void OpenOnlineServicesDialog();
    void FanucConnectTest();
    void FanucDisconnectTest();
    void RobotClearAlarmTest();
    void RobotEmergencyStop();
    void RobotSwitchStepMode();
    void ReadTool1ToGunTool();
    void FanucGetCurrentPosTest();
    void FanucGetCurrentPulseTest();
    void FanucCheckDoneTest();
    void FanucSetGetIntTest();
    void FanucSetTpSpeedTest();
    void FanucCallJobTest();
    void FanucUploadLsTest();
    void FanucMovlTest();
    void FanucMovjTest();
    void FanucMoveZeroTest();
    void OpenRobotJogDialog();
    void OpenAboutDialog();
    void GrooveCameraTest(bool checked);
    void UpdateGrooveCameraData();
    void OpenGroovePointCloudDialog();
    void CloseGrooveCameraPreviewWindow();
    void StartGrooveCameraPreview();
    void OpenPointCloudViewerDialog();

private:
    void ShowDashboardPage();
    void ShowCurrentUserMenu();
    void ShowManagementPage();
    void ShowManagementHomePage();
    void ShowEmbeddedPage(QWidget* page);
    void ShowManagementEmbeddedPage(QWidget* page);
    void PrepareEmbeddedPage(QWidget* page);
    void PrepareEmbeddedPage(QWidget* page, QStackedWidget* targetStack);
    QStackedWidget* CurrentEmbeddedTargetStack() const;
    void ShowCurrentEmbeddedPage(QWidget* page);
    void RefreshRobotSelectorUi();
    void RefreshRobotOperationAvailability();
    void RefreshDashboardConnectionState();
    bool EnsureRobotUiActionIdle(const QString& actionName);
    void RunFunctionTestDashboardTool(const QString& actionId);
    bool IsCurrentRobotConnected();
    void ToggleCurrentRobotConnection();
    int CurrentRobotUnitIndex() const;
    QString CurrentRobotName() const;
    bool IsCurrentRobotSetupReady(bool requireCameraParam, bool requireHandEye, QString* issueText = nullptr) const;
    const T_CONTRAL_UNIT* CurrentContralUnit() const;
    bool IsRobotUnitDriverReady(int unitIndex, QString* issueText = nullptr) const;
    int FindFirstReadyRobotUnitIndex() const;
    RobotDriverAdaptor* GetCurrentRobotDriver(QWidget* parent);
    QString RoleDisplayName(const QString& role) const;
    int RoleLevel(const QString& role) const;
    bool RequirePermission(const QString& minimumRole, const QString& actionName);
    bool ValidateCurrentAccountSession(const QString& actionName = QString());
    void RevokePrivilegedUiAccess();
    void EnsureDefaultAdminAccount();
    void RefreshAccountUi();
    void LoadLoginState();
    void SaveLoginState() const;
    void RefreshLoginNameHistory();
    void FillSavedPasswordForUser(const QString& userName);
    bool TryAutoLogin();
    void ShowAuthPage(const QString& promptMessage = QString());
    bool VerifyAccount(
        const QString& userName,
        const QString& password,
        QString& role,
        bool& mustChangePassword,
        QString& passwordRecord,
        QString& error) const;
    bool SaveAccount(const QString& userName, const QString& password, const QString& role, QString& error) const;
    bool PromptForcedPasswordChange(
        const QString& userName,
        const QString& temporaryPassword,
        const QString& expectedPasswordRecord,
        QString& replacementPassword,
        QString& currentRole,
        QString& securityFingerprint);
    void SetAuthRegisterMode(bool registerMode);
    void RefreshAuthModeUi();
    void LoginCurrentAccount();
    void LoginAsGuest();
    void LogoutCurrentAccount();
    void RegisterAccount();
    void OpenAccountManagementDialog();
    void OpenControlUnitManagementDialog();
    void OpenFtpJobManagementDialog();
    void OpenPrecisePointCloudProcessingPage();
    void OpenWorkpieceMeshPage();
    void OpenModelAlignmentPage();
    void OpenVirtualWeldTestPage();
    void OpenProcessLoopTestPage();
    void OpenConfigDatabaseViewerDialog();
    void SetDebugLogMode(bool enabled);
    void RefreshDebugLogButtonUi();
    void ApplyDebugLogVisibility(QWidget* page);
    void LoadCameraReceiveMode();
    void SaveCameraReceiveMode() const;
    void RefreshCameraReceiveModeButtonUi();
    void SetSharedScanCameraReceiverMode(bool enabled);
    void RefreshDesktopIconBgButtonUi();
    void SetDesktopIconWithBackground(bool withBackground);
    void RefreshAllWindowIcons();
    void RefreshScanTimestampSourceUi();
    void RefreshStepSdkInterfaceModeUi();
    void RefreshTouchKeyboardModeUi();
    bool LoadGrooveCameraIP(QString& cameraIP) const;
    bool LoadGrooveCameraIPForUnit(int unitIndex, QString& cameraIP) const;
    bool LoadGrooveCameraEndpointForUnit(int unitIndex, QString& cameraIP, int& cameraPort, int* pollIntervalMs = nullptr) const;
    void InitializeScanCameraRuntimes();
    void StopScanCameraRuntimes();
    // blockingConnect=false：相机连接异步发起(Qt::QueuedConnection)，不阻塞调用线程。
    // 启动期(InitializeScanCameraRuntimes)用 false，避免相机连不上时 3s 同步超时拖延主窗口显示。
    bool EnsureScanCameraRunningForUnit(int unitIndex, QString& cameraIP, bool clearCache, bool blockingConnect = true);
    CameraFrameCache* ScanCameraCacheForUnit(int unitIndex) const;
    void LoadRobotLogFile(const QString& relativePath, bool forceRefresh = false);
    void RunCommandLineActions(const QStringList& arguments);
    void LogCommandLineMessage(const QString& message) const;
    void EnsureCommandLineConsole() const;
    void WaitForCommandLineEnter(const QString& message) const;
    FANUCRobotCtrl* GetFirstFanucDriverForCli() const;
    RobotDriverAdaptor* GetRobotDriverForCli(
        const QStringList& arguments,
        QString* robotLabelOut = nullptr,
        int* unitIndexOut = nullptr) const;
    void RunRobotMotionForCli(const QStringList& arguments);
    bool UploadFanucServiceBundleForCli(FANUCRobotCtrl* pFanucDriver);
    void RunFanucCurposDiagnosticForCli(FANUCRobotCtrl* pFanucDriver);
    bool RunLaserClassifyForCli(const QString& inputPath, const QString& outputPath) const;
    bool RunLaserClassifyDirForCli(const QString& dirPath) const;
    bool RunRebuildMeasureWeldFilesForCli(const QStringList& arguments, const QString& laserDirPath) const;
    bool RunWeldSeamCompForCli(const QStringList& arguments, const QString& inputPath, const QString& outputPath) const;
    bool RunGenerateStepWeldProgramForCli(
        const QStringList& arguments,
        const QString& inputPath,
        const QString& outputDir,
        bool actualWeld,
        double weldSpeedMmPerMin) const;
    void RunUpdateWeldPoseAverageForCli(const QString& inputPath) const;
    bool HasRunningMeasureThenWeldFlow() const;
    ScanDataUploader* EnsureScanDataUploader();
    bool RunMeasureThenWeldScanOnlyRepeatForCli(
        RobotDriverAdaptor* pRobotDriver,
        int unitIndex,
        int repeatCount,
        double scanSpeedOverrideMmPerMin = 0.0,
        double cameraTimeOffsetOverrideMs = std::numeric_limits<double>::quiet_NaN());

    // 流程测试页后端：读某单元先测后焊预设默认（界面预填用）+ 在后台线程循环跑仅扫描流程。
    ProcessLoopTestDefaults LoadProcessLoopTestDefaults(int unitIndex);
    void RunProcessLoopTest(
        const ProcessLoopTestSettings& settings,
        std::atomic<bool>* stopFlag,
        const ProcessLoopTestDialog::ProgressCallback& progress,
        const ProcessLoopTestDialog::LogCallback& log);

    Ui::QtWidgetsApplication4Class ui;
    struct CameraRuntime;
    ContralUnit* m_pContralUnit;
    QTimer* m_grooveCameraDisplayTimer;
    QTimer* m_robotLogDisplayTimer;
    QStackedWidget* m_pMainStack;
    QWidget* m_pAuthPage;
    QWidget* m_pDashboardPage;
    QWidget* m_pManagementPage;
    QStackedWidget* m_pManagementStack;
    QWidget* m_pManagementHomePage;
    QWidget* m_pAccountManagementPage;
    QWidget* m_pControlUnitManagementPage;
    QWidget* m_pFtpJobManagementPage;
    QWidget* m_pConfigDatabaseViewerPage;
    QWidget* m_pOnlineServicesPage = nullptr;              // 「在线服务」嵌入管理栈的页（非独立弹窗）
    bool m_bOnlineServicesPageRemoteAllowed = false;      // 缓存页构建时的 admin 权限，权限变化则重建
    QWidget* m_pProcessLoopTestPage = nullptr;            // 「流程测试」嵌入管理栈的页
    LaserWeldFilterDialog* m_pPrecisePointCloudProcessingPage;
    WorkpieceMeshViewerDialog* m_pWorkpieceMeshPage = nullptr;
    ModelAlignmentDialog* m_pModelAlignmentPage = nullptr;
    VirtualWeldTestDialog* m_pVirtualWeldTestPage = nullptr;
    int m_nVirtualWeldTestPageUnitIndex = -1;
    QComboBox* m_pRobotSelectorCombo;
    QLabel* m_pRobotSelectorLabel;
    int m_nCurrentRobotUnitIndex;
    int m_nWeldProcessPageUnitIndex;
    int m_nFunctionTestPageUnitIndex;
    int m_nMeasureThenWeldPageUnitIndex;
    int m_nRobotJogPageUnitIndex;
    int m_nFtpJobManagementPageUnitIndex;
    QPlainTextEdit* m_pRobotLogText;
    QWidget* m_pGroovePointCloudDialog;
    QWidget* m_pPointCloudViewerDialog;
    QPushButton* m_pCurrentUserButton;
    QLabel* m_pManagementUserLabel;
    QLabel* m_pPermissionHintLabel;
    QAction* m_pAccountManagementAction;
    QPushButton* m_pManagementCameraReceiveModeBtn;
    QPushButton* m_pManagementIconBgBtn = nullptr;
    QComboBox* m_pScanTimestampSourceCombo;
    QComboBox* m_pStepSdkInterfaceModeCombo;
    QComboBox* m_pTouchKeyboardModeCombo;
    QLabel* m_pAuthTitleLabel;
    QLabel* m_pAuthHintLabel;
    QComboBox* m_pLoginNameCombo;
    QLineEdit* m_pLoginNameEdit;
    QLineEdit* m_pLoginPasswordEdit;
    QWidget* m_pAuthConfirmPasswordRow;
    QLineEdit* m_pAuthConfirmPasswordEdit;
    QPlainTextEdit* m_pAccountLogText;
    QCheckBox* m_pAutoLoginCheck;
    QCheckBox* m_pRememberPasswordCheck;
    QPushButton* m_pAuthLoginModeBtn;
    QPushButton* m_pAuthRegisterModeBtn;
    QPushButton* m_pAuthSubmitBtn;
    QPushButton* m_pGuestLoginBtn;
    QPushButton* m_pDashboardConnectBtn;
    QPushButton* m_pDashboardClearAlarmBtn;
    QPushButton* m_pDashboardEmergencyStopBtn;
    QPushButton* m_pDashboardModeBtn;
    QPushButton* m_pDashboardDebugLogBtn;
    QWidget* m_pDashboardToolPanel;
    QList<QPointer<QWidget>> m_robotOperationWidgets;
    QList<QPointer<QWidget>> m_fanucOnlyWidgets;
    QList<QPointer<QWidget>> m_cameraParamDependentWidgets;
    QList<QPointer<QWidget>> m_handEyeDependentWidgets;
    QPushButton* m_pCameraParamBtn;
    QPushButton* m_pWeldSeamCompBtn;
    WeldProcessDialog* m_pWeldProcessPage;
    FunctionTestDialog* m_pFunctionTestPage;
    MeasureThenWeldDialog* m_pMeasureThenWeldPage;
    PreciseMeasureEditDialog* m_pPreciseMeasureEditPage;
    WeldSeamCompDialog* m_pWeldSeamCompPage;
    CameraParamDialog* m_pCameraParamPage;
    QString m_sCameraParamPageRobotName;
    QString m_sGrooveCameraStatusText;
    RobotJogDialog* m_pRobotJogPage;
    TouchKeyboardManager* m_pTouchKeyboardManager;
    QHash<int, CameraRuntime*> m_scanCameraRuntimes;
    QHash<int, CameraRuntime*> m_scanCameraReceiversByPort;
    QHash<QString, int> m_scanCameraUnitByIP;
    QSet<CameraRuntime*> m_liveScanCameraRuntimes;
    SKJCameraControlClient* m_skjCameraControlClient;
    QHash<int, QPointer<MeasureThenWeldDialog>> m_measureThenWeldPages;
    // 在线服务：扫描数据上传常驻服务（懒创建，自动上传与管理页共用一个实例）。
    ScanDataUploader* m_pScanDataUploader = nullptr;
    QHash<int, QPointer<RobotJogDialog>> m_robotJogPages;
    bool m_bFanucMovlForward;
    bool m_bFanucMovlRunning;
    bool m_bFanucMovjRunning;
    bool m_bFanucMoveZeroRunning;
    QString m_sCurrentUserName;
    QString m_sCurrentUserRole;
    QString m_sCurrentUserSecurityFingerprint;
    QString m_sMeasureThenWeldStatus;
    QString m_sAuthHintOverride;
    bool m_bAuthRegisterMode;
    bool m_bAccountRecoveryRequired = false;
    bool m_bInitialAdministratorSetupRequired = false;
    bool m_bAccountSessionEventValidation = false;
    bool m_bSessionRevocationPendingSafetyStop = false;
    bool m_bEnforceInteractiveSessionLeaseGate = false;
    bool m_bOpenEmbeddedInManagement;
    bool m_bPendingOpenManagementAfterLogin = false;
    bool m_bDebugLogMode;
    bool m_bUseSharedScanCameraReceiver;
    QString m_sCurrentRobotLogPath;
    QString m_sLastRobotLogFilePath;
    QDateTime m_lastRobotLogModified;
    qint64 m_nLastRobotLogSize = -1;
    std::atomic_bool m_bFanucMonitorReading;
};
