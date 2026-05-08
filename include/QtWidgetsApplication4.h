#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_QtWidgetsApplication4.h"
#include "ContralUnit.h"

#include <atomic>
#include <QDateTime>
#include <QString>
#include <QStringList>

#include <limits>

class ClientUDPFormSensorWorker;
class CameraParamDialog;
class FANUCRobotCtrl;
class FunctionTestDialog;
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
class RobotJogDialog;
class WeldProcessDialog;
class WeldSeamCompDialog;

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

signals:
    void startAllCommThreads(const QString& serverIP);
    void stopAllCommThreads();

private:
    void ShowDashboardPage();
    void ShowManagementPage();
    void ShowEmbeddedPage(QWidget* page);
    void PrepareEmbeddedPage(QWidget* page);
    void RefreshRobotSelectorUi();
    int CurrentRobotUnitIndex() const;
    const T_CONTRAL_UNIT* CurrentContralUnit() const;
    FANUCRobotCtrl* GetCurrentFanucDriver(QWidget* parent) const;
    QString AccountConfigPath() const;
    QString RoleDisplayName(const QString& role) const;
    int RoleLevel(const QString& role) const;
    bool RequirePermission(const QString& minimumRole, const QString& actionName);
    void EnsureDefaultAdminAccount();
    void RefreshAccountUi();
    bool VerifyAccount(const QString& userName, const QString& password, QString& role, QString& error) const;
    bool SaveAccount(const QString& userName, const QString& password, const QString& role, QString& error) const;
    void LoginCurrentAccount();
    void LogoutCurrentAccount();
    void RegisterAccount();
    bool LoadGrooveCameraIP(QString& cameraIP) const;
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
    ContralUnit* m_pContralUnit;
    ClientUDPFormSensorWorker* m_clientUDPFormSensorWorker;
    QThread* m_clientUDPFormSensorThread;
    QTimer* m_grooveCameraDisplayTimer;
    QTimer* m_robotLogDisplayTimer;
    QStackedWidget* m_pMainStack;
    QWidget* m_pDashboardPage;
    QWidget* m_pManagementPage;
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
    QLineEdit* m_pLoginNameEdit;
    QLineEdit* m_pLoginPasswordEdit;
    QLineEdit* m_pRegisterNameEdit;
    QLineEdit* m_pRegisterPasswordEdit;
    QComboBox* m_pRegisterRoleCombo;
    QPlainTextEdit* m_pAccountLogText;
    QPushButton* m_pCameraParamBtn;
    QPushButton* m_pWeldSeamCompBtn;
    WeldProcessDialog* m_pWeldProcessPage;
    FunctionTestDialog* m_pFunctionTestPage;
    MeasureThenWeldDialog* m_pMeasureThenWeldPage;
    PreciseMeasureEditDialog* m_pPreciseMeasureEditPage;
    WeldSeamCompDialog* m_pWeldSeamCompPage;
    CameraParamDialog* m_pCameraParamPage;
    RobotJogDialog* m_pRobotJogPage;
    bool m_bFanucMovlForward;
    bool m_bFanucMovlRunning;
    bool m_bFanucMovjRunning;
    bool m_bFanucMoveZeroRunning;
    QString m_sCurrentUserName;
    QString m_sCurrentUserRole;
    QString m_sMeasureThenWeldStatus;
    QString m_sCurrentRobotLogPath;
    QString m_sLastRobotLogFilePath;
    QDateTime m_lastRobotLogModified;
    qint64 m_nLastRobotLogSize = -1;
    std::atomic_bool m_bFanucMonitorReading;
};

