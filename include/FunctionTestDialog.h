#pragma once

#include "ContralUnit.h"

#include <QDialog>
#include <QVector>

class QPushButton;
class QPlainTextEdit;
class FANUCRobotCtrl;
class RobotDriverAdaptor;
class QTimer;

// 功能测试子界面：把主界面上的 FANUC 调试/运动测试按钮集中管理。
class FunctionTestDialog : public QDialog
{
public:
    explicit FunctionTestDialog(ContralUnit* pContralUnit, QWidget* parent = nullptr);

protected:
    void closeEvent(QCloseEvent* event) override;

private:
    FANUCRobotCtrl* GetFirstFanucDriver();
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
    void FanucMovlTest();
    void FanucMovjTest();
    void FanucMoveZeroTest();
    void FanucCaptureKinematicsSample();
    void FitDhParametersFromSamples();
    void OpenLaserWeldFilterTest();

private:
    ContralUnit* m_pContralUnit = nullptr;
    QPushButton* m_pMovlTestBtn = nullptr;
    QPushButton* m_pMovjTestBtn = nullptr;
    QPushButton* m_pMoveZeroBtn = nullptr;
    QPlainTextEdit* m_pLogText = nullptr;
    QTimer* m_pMotionStateTimer = nullptr;
    QVector<QPushButton*> m_motionButtons;
    QString m_kinematicsSampleFilePath;
    int m_kinematicsSampleCount = 0;

    bool m_bFanucMovlForward = true;
    bool m_bFanucMovlRunning = false;
    bool m_bFanucMovjRunning = false;
    bool m_bFanucMoveZeroRunning = false;
};
