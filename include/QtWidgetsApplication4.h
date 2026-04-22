#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_QtWidgetsApplication4.h"
#include "ContralUnit.h"

#include <atomic>
#include <QDateTime>
#include <QString>
#include <QStringList>

class ClientUDPFormSensorWorker;
class FANUCRobotCtrl;
class QPlainTextEdit;
class QPushButton;
class QThread;
class QTimer;

class QtWidgetsApplication4 : public QMainWindow
{
    Q_OBJECT

public:
    QtWidgetsApplication4(QWidget *parent = nullptr);
    ~QtWidgetsApplication4();
    void ApplyStartupArguments(const QStringList& arguments);

private slots:
    void RobotRunTest();
    void OpenWeldProcessDialog();
    void OpenFunctionTestDialog();
    void OpenMeasureThenWeldDialog();
    void OpenPreciseMeasureEditDialog();
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
    bool LoadGrooveCameraIP(QString& cameraIP) const;
    void LoadRobotLogFile(const QString& relativePath, bool forceRefresh = false);
    void RunCommandLineActions(const QStringList& arguments);
    void LogCommandLineMessage(const QString& message) const;
    void EnsureCommandLineConsole() const;
    void WaitForCommandLineEnter(const QString& message) const;
    FANUCRobotCtrl* GetFirstFanucDriverForCli() const;
    bool UploadFanucServiceBundleForCli(FANUCRobotCtrl* pFanucDriver);
    void RunFanucCurposDiagnosticForCli(FANUCRobotCtrl* pFanucDriver);

    Ui::QtWidgetsApplication4Class ui;
    ContralUnit* m_pContralUnit;
    ClientUDPFormSensorWorker* m_clientUDPFormSensorWorker;
    QThread* m_clientUDPFormSensorThread;
    QTimer* m_grooveCameraDisplayTimer;
    QTimer* m_robotLogDisplayTimer;
    QPlainTextEdit* m_pRobotLogText;
    QPushButton* m_pCameraParamBtn;
    bool m_bFanucMovlForward;
    bool m_bFanucMovlRunning;
    bool m_bFanucMovjRunning;
    bool m_bFanucMoveZeroRunning;
    QString m_sMeasureThenWeldStatus;
    QString m_sCurrentRobotLogPath;
    QString m_sLastRobotLogFilePath;
    QDateTime m_lastRobotLogModified;
    qint64 m_nLastRobotLogSize = -1;
    std::atomic_bool m_bFanucMonitorReading;
};

