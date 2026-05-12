#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_QtWidgetsApplication4.h"
#include "ContralUnit.h"

#include <atomic>
#include <QDateTime>
#include <QHash>
#include <QList>
#include <QPointer>
#include <QString>
#include <QStringList>

#include <limits>

class ClientUDPFormSensorWorker;
class CameraFrameCache;
class CameraParamDialog;
class FANUCRobotCtrl;
class FunctionTestDialog;
class QCheckBox;
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
class WeldProcessDialog;
class WeldSeamCompDialog;
class QAction;

class QtWidgetsApplication4 : public QMainWindow
{
    Q_OBJECT

public:
    QtWidgetsApplication4(QWidget *parent = nullptr);
    ~QtWidgetsApplication4();
    void ApplyStartupArguments(const QStringList& arguments);

protected:
    bool eventFilter(QObject* watched, QEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;

private slots:
    void RobotRunTest();
    void OpenWeldProcessDialog();
    void OpenFunctionTestDialog();
    void OpenMeasureThenWeldDialog();
    void OpenPreciseMeasureEditDialog();
    void OpenWeldSeamCompDialog();
    void OpenCameraParamDialog();
    void FanucConnectTest();
    void FanucDisconnectTest();
    void RobotClearAlarmTest();
    void RobotSwitchStepMode();
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

private:
    void ShowDashboardPage();
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
    bool IsCurrentRobotConnected();
    void ToggleCurrentRobotConnection();
    int CurrentRobotUnitIndex() const;
    const T_CONTRAL_UNIT* CurrentContralUnit() const;
    bool IsRobotUnitDriverReady(int unitIndex, QString* issueText = nullptr) const;
    int FindFirstReadyRobotUnitIndex() const;
    RobotDriverAdaptor* GetCurrentRobotDriver(QWidget* parent);
    FANUCRobotCtrl* GetCurrentFanucDriver(QWidget* parent);
    QString AccountConfigPath() const;
    QString RoleDisplayName(const QString& role) const;
    int RoleLevel(const QString& role) const;
    bool RequirePermission(const QString& minimumRole, const QString& actionName);
    void EnsureDefaultAdminAccount();
    void RefreshAccountUi();
    QString LoginStateConfigPath() const;
    void LoadLoginState();
    void SaveLoginState() const;
    void RefreshLoginNameHistory();
    void FillSavedPasswordForUser(const QString& userName);
    bool TryAutoLogin();
    void ShowAuthPage(const QString& promptMessage = QString());
    bool VerifyAccount(const QString& userName, const QString& password, QString& role, QString& error) const;
    bool SaveAccount(const QString& userName, const QString& password, const QString& role, QString& error) const;
    void SetAuthRegisterMode(bool registerMode);
    void RefreshAuthModeUi();
    void LoginCurrentAccount();
    void LoginAsGuest();
    void LogoutCurrentAccount();
    void RegisterAccount();
    void OpenAccountManagementDialog();
    void SetDebugLogMode(bool enabled);
    void RefreshDebugLogButtonUi();
    void ApplyDebugLogVisibility(QWidget* page);
    QString CameraReceiveModeConfigPath() const;
    void LoadCameraReceiveMode();
    void SaveCameraReceiveMode() const;
    void RefreshCameraReceiveModeButtonUi();
    void SetSharedScanCameraReceiverMode(bool enabled);
    bool LoadGrooveCameraIP(QString& cameraIP) const;
    bool LoadGrooveCameraIPForUnit(int unitIndex, QString& cameraIP) const;
    bool LoadGrooveCameraEndpointForUnit(int unitIndex, QString& cameraIP, int& cameraPort) const;
    void InitializeScanCameraRuntimes();
    void StopScanCameraRuntimes();
    bool EnsureScanCameraRunningForUnit(int unitIndex, QString& cameraIP, bool clearCache);
    CameraFrameCache* ScanCameraCacheForUnit(int unitIndex) const;
    void LoadRobotLogFile(const QString& relativePath, bool forceRefresh = false);
    void RunCommandLineActions(const QStringList& arguments);
    void LogCommandLineMessage(const QString& message) const;
    void EnsureCommandLineConsole() const;
    void WaitForCommandLineEnter(const QString& message) const;
    FANUCRobotCtrl* GetFirstFanucDriverForCli() const;
    bool UploadFanucServiceBundleForCli(FANUCRobotCtrl* pFanucDriver);
    void RunFanucCurposDiagnosticForCli(FANUCRobotCtrl* pFanucDriver);
    void RunLaserClassifyForCli(const QString& inputPath, const QString& outputPath) const;
    void RunWeldSeamCompForCli(const QString& inputPath, const QString& outputPath) const;
    void RunUpdateWeldPoseAverageForCli(const QString& inputPath) const;
    bool RunMeasureThenWeldScanOnlyRepeatForCli(
        FANUCRobotCtrl* pFanucDriver,
        int repeatCount,
        double scanSpeedOverrideMmPerMin = 0.0,
        double cameraTimeOffsetOverrideMs = std::numeric_limits<double>::quiet_NaN());

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
    QComboBox* m_pRobotSelectorCombo;
    QLabel* m_pRobotSelectorLabel;
    int m_nCurrentRobotUnitIndex;
    int m_nWeldProcessPageUnitIndex;
    int m_nFunctionTestPageUnitIndex;
    int m_nMeasureThenWeldPageUnitIndex;
    int m_nRobotJogPageUnitIndex;
    QPlainTextEdit* m_pRobotLogText;
    QLabel* m_pCurrentUserLabel;
    QLabel* m_pManagementUserLabel;
    QLabel* m_pPermissionHintLabel;
    QPushButton* m_pManagementCameraReceiveModeBtn;
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
    QPushButton* m_pDashboardModeBtn;
    QPushButton* m_pDashboardDebugLogBtn;
    QAction* m_pAccountManagementAction = nullptr;
    QList<QPointer<QWidget>> m_robotOperationWidgets;
    QPushButton* m_pCameraParamBtn;
    QPushButton* m_pWeldSeamCompBtn;
    WeldProcessDialog* m_pWeldProcessPage;
    FunctionTestDialog* m_pFunctionTestPage;
    MeasureThenWeldDialog* m_pMeasureThenWeldPage;
    PreciseMeasureEditDialog* m_pPreciseMeasureEditPage;
    WeldSeamCompDialog* m_pWeldSeamCompPage;
    CameraParamDialog* m_pCameraParamPage;
    QString m_sCameraParamPageRobotName;
    RobotJogDialog* m_pRobotJogPage;
    QHash<int, CameraRuntime*> m_scanCameraRuntimes;
    QHash<int, CameraRuntime*> m_scanCameraReceiversByPort;
    QHash<QString, int> m_scanCameraUnitByIP;
    QHash<int, QPointer<MeasureThenWeldDialog>> m_measureThenWeldPages;
    QHash<int, QPointer<RobotJogDialog>> m_robotJogPages;
    bool m_bFanucMovlForward;
    bool m_bFanucMovlRunning;
    bool m_bFanucMovjRunning;
    bool m_bFanucMoveZeroRunning;
    QString m_sCurrentUserName;
    QString m_sCurrentUserRole;
    QString m_sMeasureThenWeldStatus;
    QString m_sAuthHintOverride;
    bool m_bAuthRegisterMode;
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

