#include "FunctionTestDialog.h"

#include "FANUCRobotDriver.h"
#include "WindowStyleHelper.h"

#include <QCloseEvent>
#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <QGridLayout>
#include <QGroupBox>
#include <QInputDialog>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPointer>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QStringList>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>
#include <thread>

namespace
{
QString FindProjectFilePathForFunctionTest(const QString& relativePath)
{
    QDir dir(QCoreApplication::applicationDirPath());
    for (int depth = 0; depth < 6; ++depth)
    {
        const QString candidate = dir.filePath(relativePath);
        if (QFileInfo::exists(candidate))
        {
            return QDir::toNativeSeparators(QFileInfo(candidate).absoluteFilePath());
        }
        if (!dir.cdUp())
        {
            break;
        }
    }
    return QString();
}

QPushButton* CreateTestButton(const QString& text)
{
    QPushButton* button = new QPushButton(text);
    button->setMinimumSize(150, 44);
    return button;
}
}

FunctionTestDialog::FunctionTestDialog(ContralUnit* pContralUnit, QWidget* parent)
    : QDialog(parent)
    , m_pContralUnit(pContralUnit)
{
    setWindowTitle("功能测试");
    ApplyUnifiedWindowChrome(this);
    resize(760, 560);

    setStyleSheet(
        "QDialog { background: #101820; color: #E8F1F2; }"
        "QGroupBox { border: 1px solid #2E4656; border-radius: 12px; margin-top: 18px; padding: 14px; font-weight: bold; }"
        "QGroupBox::title { subcontrol-origin: margin; left: 16px; padding: 0 6px; color: #9ED8DB; }"
        "QPushButton { background: #1F3542; color: #F4FAFA; border: 1px solid #3C6475; border-radius: 10px; padding: 8px 14px; }"
        "QPushButton:hover { background: #2C5364; border-color: #63C7D1; }"
        "QPushButton:pressed { background: #16303A; }"
        "QPushButton:disabled { background: #26313A; color: #778893; border-color: #34434B; }"
        "QPlainTextEdit { background: #0B1117; color: #BFE7EA; border: 1px solid #2E4656; border-radius: 10px; padding: 8px; }"
        "QLabel { color: #B8C7CC; }");

    QVBoxLayout* rootLayout = new QVBoxLayout(this);

    QLabel* titleLabel = new QLabel("FANUC 功能测试区");
    titleLabel->setStyleSheet("font-size: 20px; font-weight: bold; color: #F4FAFA;");
    rootLayout->addWidget(titleLabel);

    QLabel* hintLabel = new QLabel("这里集中放置设置速度、读取位置、检查运行、往返运动、零位运动等测试功能，避免主界面继续堆按钮。");
    rootLayout->addWidget(hintLabel);

    QGridLayout* groupLayout = new QGridLayout();
    rootLayout->addLayout(groupLayout);

    QGroupBox* basicGroup = new QGroupBox("基础通讯/状态");
    QGridLayout* basicLayout = new QGridLayout(basicGroup);
    QPushButton* setSpeedBtn = CreateTestButton("设置速度");
    QPushButton* getPosBtn = CreateTestButton("读取当前位置");
    QPushButton* getPulseBtn = CreateTestButton("读取关节脉冲");
    QPushButton* checkDoneBtn = CreateTestButton("检查运行完成");
    QPushButton* setGetIntBtn = CreateTestButton("写读INT寄存器");
    QPushButton* callJobBtn = CreateTestButton("调用任务");
    QPushButton* uploadLsBtn = CreateTestButton("发送LS程序");
    QPushButton* curposDiagBtn = CreateTestButton("CURPOS诊断");
    basicLayout->addWidget(setSpeedBtn, 0, 0);
    basicLayout->addWidget(getPosBtn, 0, 1);
    basicLayout->addWidget(getPulseBtn, 1, 0);
    basicLayout->addWidget(checkDoneBtn, 1, 1);
    basicLayout->addWidget(setGetIntBtn, 2, 0);
    basicLayout->addWidget(callJobBtn, 2, 1);
    basicLayout->addWidget(uploadLsBtn, 3, 0);
    basicLayout->addWidget(curposDiagBtn, 3, 1);
    groupLayout->addWidget(basicGroup, 0, 0);

    QGroupBox* motionGroup = new QGroupBox("运动测试");
    QGridLayout* motionLayout = new QGridLayout(motionGroup);
    m_pMovlTestBtn = CreateTestButton("MOVL往返测试");
    m_pMovjTestBtn = CreateTestButton("MOVJ J2/J3 +5deg");
    m_pMoveZeroBtn = CreateTestButton("运动到零位");
    motionLayout->addWidget(m_pMovlTestBtn, 0, 0);
    motionLayout->addWidget(m_pMovjTestBtn, 1, 0);
    motionLayout->addWidget(m_pMoveZeroBtn, 2, 0);
    groupLayout->addWidget(motionGroup, 0, 1);

    m_pLogText = new QPlainTextEdit();
    m_pLogText->setReadOnly(true);
    m_pLogText->setPlainText("功能测试日志：等待操作...");
    rootLayout->addWidget(m_pLogText, 1);

    connect(setSpeedBtn, &QPushButton::clicked, this, &FunctionTestDialog::FanucSetTpSpeedTest);
    connect(getPosBtn, &QPushButton::clicked, this, &FunctionTestDialog::FanucGetCurrentPosTest);
    connect(getPulseBtn, &QPushButton::clicked, this, &FunctionTestDialog::FanucGetCurrentPulseTest);
    connect(checkDoneBtn, &QPushButton::clicked, this, &FunctionTestDialog::FanucCheckDoneTest);
    connect(setGetIntBtn, &QPushButton::clicked, this, &FunctionTestDialog::FanucSetGetIntTest);
    connect(callJobBtn, &QPushButton::clicked, this, &FunctionTestDialog::FanucCallJobTest);
    connect(uploadLsBtn, &QPushButton::clicked, this, &FunctionTestDialog::FanucUploadLsTest);
    connect(curposDiagBtn, &QPushButton::clicked, this, &FunctionTestDialog::FanucCurposDiagnosticTest);
    connect(m_pMovlTestBtn, &QPushButton::clicked, this, &FunctionTestDialog::FanucMovlTest);
    connect(m_pMovjTestBtn, &QPushButton::clicked, this, &FunctionTestDialog::FanucMovjTest);
    connect(m_pMoveZeroBtn, &QPushButton::clicked, this, &FunctionTestDialog::FanucMoveZeroTest);
}

void FunctionTestDialog::closeEvent(QCloseEvent* event)
{
    if (IsMotionBusy())
    {
        QMessageBox::information(this, "功能测试", "运动测试正在执行，请等本次运动结束后再关闭窗口。");
        event->ignore();
        return;
    }
    QDialog::closeEvent(event);
}

FANUCRobotCtrl* FunctionTestDialog::GetFirstFanucDriver()
{
    if (m_pContralUnit == nullptr || m_pContralUnit->m_vtContralUnitInfo.empty())
    {
        QMessageBox::warning(this, "FANUC测试", "未找到可用的控制单元。");
        return nullptr;
    }

    RobotDriverAdaptor* pRobotDriverAdaptor = static_cast<RobotDriverAdaptor*>(m_pContralUnit->m_vtContralUnitInfo[0].pUnitDriver);
    if (pRobotDriverAdaptor == nullptr)
    {
        QMessageBox::warning(this, "FANUC测试", "当前控制单元未创建驱动。");
        return nullptr;
    }

    FANUCRobotCtrl* pFanucDriver = dynamic_cast<FANUCRobotCtrl*>(pRobotDriverAdaptor);
    if (pFanucDriver == nullptr)
    {
        QMessageBox::warning(this, "FANUC测试", "当前控制单元不是 FANUC 驱动。");
        return nullptr;
    }
    return pFanucDriver;
}

bool FunctionTestDialog::IsMotionBusy() const
{
    return m_bFanucMovlRunning || m_bFanucMovjRunning || m_bFanucMoveZeroRunning;
}

void FunctionTestDialog::AppendLog(const QString& text)
{
    if (m_pLogText == nullptr)
    {
        return;
    }
    m_pLogText->appendPlainText(text);
}

void FunctionTestDialog::FanucGetCurrentPosTest()
{
    FANUCRobotCtrl* pFanucDriver = GetFirstFanucDriver();
    if (pFanucDriver == nullptr)
    {
        return;
    }

    const T_ROBOT_COORS pos = pFanucDriver->GetCurrentPos();
    const QString message = QString("当前位置: X=%1, Y=%2, Z=%3, RX=%4, RY=%5, RZ=%6")
        .arg(pos.dX, 0, 'f', 3)
        .arg(pos.dY, 0, 'f', 3)
        .arg(pos.dZ, 0, 'f', 3)
        .arg(pos.dRX, 0, 'f', 3)
        .arg(pos.dRY, 0, 'f', 3)
        .arg(pos.dRZ, 0, 'f', 3);
    AppendLog(message);
    QMessageBox::information(this, "读取当前位置", message);
}

void FunctionTestDialog::FanucGetCurrentPulseTest()
{
    FANUCRobotCtrl* pFanucDriver = GetFirstFanucDriver();
    if (pFanucDriver == nullptr)
    {
        return;
    }

    const T_ANGLE_PULSE pulse = pFanucDriver->GetCurrentPulse();
    const QString message = QString("关节脉冲: S=%1, L=%2, U=%3, R=%4, B=%5, T=%6, EX1=%7, EX2=%8, EX3=%9")
        .arg(pulse.nSPulse)
        .arg(pulse.nLPulse)
        .arg(pulse.nUPulse)
        .arg(pulse.nRPulse)
        .arg(pulse.nBPulse)
        .arg(pulse.nTPulse)
        .arg(pulse.lBXPulse)
        .arg(pulse.lBYPulse)
        .arg(pulse.lBZPulse);
    AppendLog(message);
    QMessageBox::information(this, "读取关节脉冲", message);
}

void FunctionTestDialog::FanucCurposDiagnosticTest()
{
    FANUCRobotCtrl* pFanucDriver = GetFirstFanucDriver();
    if (pFanucDriver == nullptr)
    {
        return;
    }

    const QStringList commands =
    {
        "GET_USER_PROGRAM",
        "GET_CUR_POS",
        "GET_CUR_POS_00",
        "GET_CUR_POS_001",
        "GET_CUR_POS_01",
        "GET_CUR_POS_10",
        "GET_CUR_POS_11",
        "GET_CUR_POS_011",
        "GET_CUR_POS_111",
        "GET_POS_VAR:20,0"
    };

    QStringList lines;
    for (const QString& command : commands)
    {
        const std::string response = pFanucDriver->SendRawCommandForTest(command.toStdString());
        lines << QString("%1 -> %2").arg(command, QString::fromStdString(response));
    }

    const QString message = lines.join("\n");
    AppendLog("CURPOS诊断:\n" + message);
    QMessageBox::information(this, "CURPOS诊断", message);
}

void FunctionTestDialog::FanucCheckDoneTest()
{
    FANUCRobotCtrl* pFanucDriver = GetFirstFanucDriver();
    if (pFanucDriver == nullptr)
    {
        return;
    }

    const int done = pFanucDriver->CheckDone();
    const QString message = QString("CheckDone 返回值：%1").arg(done);
    AppendLog(message);
    QMessageBox::information(this, "检查运行完成", message);
}

void FunctionTestDialog::FanucSetGetIntTest()
{
    FANUCRobotCtrl* pFanucDriver = GetFirstFanucDriver();
    if (pFanucDriver == nullptr)
    {
        return;
    }

    bool ok = false;
    const int index = QInputDialog::getInt(this, "写读INT寄存器", "寄存器编号：", 10, 1, 9999, 1, &ok);
    if (!ok)
    {
        return;
    }

    const int value = QInputDialog::getInt(this, "写读INT寄存器", "写入值：", 123, -999999, 999999, 1, &ok);
    if (!ok)
    {
        return;
    }

    if (!pFanucDriver->SetIntVar(index, value))
    {
        QMessageBox::warning(this, "写读INT寄存器", GetStr("写入 INT%d 失败。", index).c_str());
        return;
    }

    const int readValue = pFanucDriver->GetIntVar(index);
    const QString message = QString("写入 INT%1=%2, 读取值=%3").arg(index).arg(value).arg(readValue);
    AppendLog(message);
    QMessageBox::information(this, "写读INT寄存器", message);
}

void FunctionTestDialog::FanucSetTpSpeedTest()
{
    FANUCRobotCtrl* pFanucDriver = GetFirstFanucDriver();
    if (pFanucDriver == nullptr)
    {
        return;
    }

    bool ok = false;
    const int speed = QInputDialog::getInt(this, "设置速度", "速度百分比：", 50, 1, 100, 1, &ok);
    if (!ok)
    {
        return;
    }

    const bool setOk = pFanucDriver->SetTpSpeed(speed);
    const QString message = setOk ? QString("设置速度成功：%1").arg(speed) : QString("设置速度失败：%1").arg(speed);
    AppendLog(message);
    QMessageBox::information(this, "设置速度", message);
}

void FunctionTestDialog::FanucCallJobTest()
{
    FANUCRobotCtrl* pFanucDriver = GetFirstFanucDriver();
    if (pFanucDriver == nullptr)
    {
        return;
    }

    bool ok = false;
    const QString jobName = QInputDialog::getText(this, "调用任务", "任务/程序名：", QLineEdit::Normal, "FANUC_PORT_OPEN_TEST", &ok);
    if (!ok || jobName.trimmed().isEmpty())
    {
        return;
    }

    const QByteArray jobNameBytes = jobName.trimmed().toLocal8Bit();
    const bool callOk = pFanucDriver->CallJob(jobNameBytes.constData());
    const QString message = callOk ? QString("调用任务成功：%1").arg(jobName.trimmed()) : QString("调用任务失败：%1").arg(jobName.trimmed());
    AppendLog(message);
    QMessageBox::information(this, "调用任务", message);
}

void FunctionTestDialog::FanucUploadLsTest()
{
    FANUCRobotCtrl* pFanucDriver = GetFirstFanucDriver();
    if (pFanucDriver == nullptr)
    {
        return;
    }

    const QString lsPath = FindProjectFilePathForFunctionTest("SDK/FANUC/STARTALL.ls");
    if (lsPath.isEmpty())
    {
        QMessageBox::warning(this, "发送LS程序", "未找到测试程序文件：SDK/FANUC/STARTALL.ls");
        return;
    }

    const QByteArray lsPathBytes = lsPath.toLocal8Bit();
    const int ret = pFanucDriver->UploadLsFile(lsPathBytes.constData());
    const QString message = ret == 0
        ? QString("LS程序发送成功：%1").arg(lsPath)
        : QString("LS程序发送失败，返回码=%1，文件=%2").arg(ret).arg(lsPath);
    AppendLog(message);
    if (ret == 0)
    {
        QMessageBox::information(this, "发送LS程序", message);
    }
    else
    {
        QMessageBox::warning(this, "发送LS程序", message);
    }
}

void FunctionTestDialog::FanucMovlTest()
{
    FANUCRobotCtrl* pFanucDriver = GetFirstFanucDriver();
    if (pFanucDriver == nullptr)
    {
        return;
    }
    if (m_bFanucMovlRunning)
    {
        QMessageBox::information(this, "MOVL往返测试", "MOVL测试正在执行，请等本次运动结束。");
        return;
    }

    const bool moveForward = m_bFanucMovlForward;
    m_bFanucMovlForward = !m_bFanucMovlForward;
    m_bFanucMovlRunning = true;
    m_pMovlTestBtn->setEnabled(false);
    AppendLog(QString("开始 MOVL %1 100mm 测试...").arg(moveForward ? "Y+" : "Y-"));

    QPointer<FunctionTestDialog> self(this);
    std::thread([self, pFanucDriver, moveForward]()
        {
            T_ROBOT_COORS target = pFanucDriver->GetCurrentPos();
            target.dY += moveForward ? 100.0 : -100.0;

            const bool moveOk = pFanucDriver->MoveByJob(target, T_ROBOT_MOVE_SPEED(5.0, 0.0, 0.0), pFanucDriver->m_nExternalAxleType, "MOVL");
            const int done = moveOk ? pFanucDriver->CheckRobotDone(200) : -1;
            const QString message = QString("MOVL %1 100mm, Move=%2, CheckRobotDone=%3")
                .arg(moveForward ? "Y+" : "Y-")
                .arg(moveOk ? "OK" : "FAIL")
                .arg(done);

            QMetaObject::invokeMethod(qApp, [self, message]()
                {
                    if (self == nullptr)
                    {
                        return;
                    }
                    self->m_bFanucMovlRunning = false;
                    self->m_pMovlTestBtn->setEnabled(true);
                    self->AppendLog(message);
                    QMessageBox::information(self, "MOVL往返测试", message);
                }, Qt::QueuedConnection);
        }).detach();
}

void FunctionTestDialog::FanucMovjTest()
{
    FANUCRobotCtrl* pFanucDriver = GetFirstFanucDriver();
    if (pFanucDriver == nullptr)
    {
        return;
    }
    if (m_bFanucMovjRunning)
    {
        QMessageBox::information(this, "MOVJ测试", "MOVJ测试正在执行，请等本次运动结束。");
        return;
    }

    m_bFanucMovjRunning = true;
    m_pMovjTestBtn->setEnabled(false);
    AppendLog("开始 MOVJ J2/J3 +5deg 测试...");

    QPointer<FunctionTestDialog> self(this);
    std::thread([self, pFanucDriver]()
        {
            T_ANGLE_PULSE target = pFanucDriver->GetCurrentPulse();
            const double j2PulseUnit = pFanucDriver->m_tAxisUnit.dLPulseUnit;
            const double j3PulseUnit = pFanucDriver->m_tAxisUnit.dUPulseUnit;
            const long j2DeltaPulse = j2PulseUnit == 0.0 ? 0 : static_cast<long>(std::lround(5.0 / j2PulseUnit));
            const long j3DeltaPulse = j3PulseUnit == 0.0 ? 0 : static_cast<long>(std::lround(5.0 / j3PulseUnit));
            target.nLPulse += j2DeltaPulse;
            target.nUPulse += j3DeltaPulse;

            const bool moveOk = pFanucDriver->MoveByJob(target, T_ROBOT_MOVE_SPEED(1.0, 0.0, 0.0), pFanucDriver->m_nExternalAxleType, "MOVJ");
            const int done = moveOk ? pFanucDriver->CheckRobotDone(200) : -1;
            const QString message = QString("MOVJ J2/J3 +5deg, J2DeltaPulse=%1, J3DeltaPulse=%2, Move=%3, CheckRobotDone=%4")
                .arg(j2DeltaPulse)
                .arg(j3DeltaPulse)
                .arg(moveOk ? "OK" : "FAIL")
                .arg(done);

            QMetaObject::invokeMethod(qApp, [self, message]()
                {
                    if (self == nullptr)
                    {
                        return;
                    }
                    self->m_bFanucMovjRunning = false;
                    self->m_pMovjTestBtn->setEnabled(true);
                    self->AppendLog(message);
                    QMessageBox::information(self, "MOVJ测试", message);
                }, Qt::QueuedConnection);
        }).detach();
}

void FunctionTestDialog::FanucMoveZeroTest()
{
    FANUCRobotCtrl* pFanucDriver = GetFirstFanucDriver();
    if (pFanucDriver == nullptr)
    {
        return;
    }
    if (m_bFanucMoveZeroRunning)
    {
        QMessageBox::information(this, "运动到零位", "零位运动正在执行，请等本次运动结束。");
        return;
    }

    const QMessageBox::StandardButton confirm = QMessageBox::question(
        this,
        "运动到零位",
        "将通过 MOVJ 低速运动到 J1-J6=0 的零位。\n请确认机器人周围安全，是否继续？",
        QMessageBox::Yes | QMessageBox::No,
        QMessageBox::No);
    if (confirm != QMessageBox::Yes)
    {
        return;
    }

    m_bFanucMoveZeroRunning = true;
    m_pMoveZeroBtn->setEnabled(false);
    AppendLog("开始 MOVJ 到零位...");

    QPointer<FunctionTestDialog> self(this);
    std::thread([self, pFanucDriver]()
        {
            const T_ANGLE_PULSE zeroPulse = T_ANGLE_PULSE();
            const T_ROBOT_MOVE_SPEED speed(1.0, 0.0, 0.0);
            const bool moveOk = pFanucDriver->MoveByJob(zeroPulse, speed, pFanucDriver->m_nExternalAxleType, "MOVJ");
            const int done = moveOk ? pFanucDriver->CheckRobotDone(200) : -1;
            const T_ROBOT_COORS pos = pFanucDriver->GetCurrentPos();
            const T_ANGLE_PULSE pulse = pFanucDriver->GetCurrentPulse();

            const QString message = QString(
                "MOVJ 到零位, Move=%1, CheckRobotDone=%2\n"
                "当前位置: X=%3, Y=%4, Z=%5, RX=%6, RY=%7, RZ=%8\n"
                "当前脉冲: S=%9, L=%10, U=%11, R=%12, B=%13, T=%14, EX1=%15, EX2=%16, EX3=%17")
                .arg(moveOk ? "OK" : "FAIL")
                .arg(done)
                .arg(pos.dX, 0, 'f', 3)
                .arg(pos.dY, 0, 'f', 3)
                .arg(pos.dZ, 0, 'f', 3)
                .arg(pos.dRX, 0, 'f', 3)
                .arg(pos.dRY, 0, 'f', 3)
                .arg(pos.dRZ, 0, 'f', 3)
                .arg(pulse.nSPulse)
                .arg(pulse.nLPulse)
                .arg(pulse.nUPulse)
                .arg(pulse.nRPulse)
                .arg(pulse.nBPulse)
                .arg(pulse.nTPulse)
                .arg(pulse.lBXPulse)
                .arg(pulse.lBYPulse)
                .arg(pulse.lBZPulse);

            QMetaObject::invokeMethod(qApp, [self, message]()
                {
                    if (self == nullptr)
                    {
                        return;
                    }
                    self->m_bFanucMoveZeroRunning = false;
                    self->m_pMoveZeroBtn->setEnabled(true);
                    self->AppendLog(message);
                    QMessageBox::information(self, "运动到零位", message);
                }, Qt::QueuedConnection);
        }).detach();
}
