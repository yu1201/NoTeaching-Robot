#include "MeasureThenWeldDialog.h"

#include "CameraFrameAccessGuard.h"
#include "FANUCRobotDriver.h"
#include "HandEyeMatrixConfig.h"
#include "MeasureThenWeldService.h"
#include "OPini.h"
#include "RobotDataHelper.h"
#include "WindowStyleHelper.h"
#include "groove/framebuffer.h"

#include <QCloseEvent>
#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QGridLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPointer>
#include <QPushButton>
#include <QTextStream>
#include <QThread>
#include <QVBoxLayout>

#include <algorithm>
#include <thread>

namespace
{
constexpr auto WELD_POSE_FILE_NAME = "PreciseLaserPoint_WeldPose_2mm.txt";
constexpr auto WELD_POSE_SEAM_COMP_FILE_NAME = "PreciseLaserPoint_WeldPose_2mm_SeamComp.txt";

double SafeSpeed(double value, double fallback)
{
    return value > 0.0 ? value : fallback;
}

QString ResolveLaserPointDirFromSelection(const QString& selectedDir)
{
    const QFileInfo selectedInfo(selectedDir);
    if (!selectedInfo.exists() || !selectedInfo.isDir())
    {
        return QString();
    }

    const QDir dir(selectedInfo.absoluteFilePath());
    if (QFileInfo::exists(dir.filePath(WELD_POSE_FILE_NAME)))
    {
        return dir.absolutePath();
    }

    const QString laserDir = dir.filePath("LaserPoint");
    if (QFileInfo::exists(QDir(laserDir).filePath(WELD_POSE_FILE_NAME)))
    {
        return QDir(laserDir).absolutePath();
    }

    return QString();
}
}

MeasureThenWeldDialog::MeasureThenWeldDialog(ContralUnit* pContralUnit, StartCameraFunc startCamera, StopCameraFunc stopCamera, QWidget* parent)
    : QDialog(parent)
    , m_pContralUnit(pContralUnit)
    , m_pService(new MeasureThenWeldService())
    , m_startCamera(startCamera)
    , m_stopCamera(stopCamera)
{
    setWindowTitle("先测后焊");
    ApplyUnifiedWindowChrome(this);
    ResizeWindowForAvailableGeometry(this, QSize(620, 430), 0.68, 0.62);
    setStyleSheet(
        "QDialog { background: #111820; color: #ECF3F4; }"
        "QPushButton { background: #233645; color: #F5FAFA; border: 1px solid #3C6173; border-radius: 12px; padding: 12px 18px; font-size: 16px; }"
        "QPushButton:hover { background: #2D5465; border-color: #72D4DD; }"
        "QPushButton:pressed { background: #18303B; }"
        "QPushButton:disabled { background: #27323A; color: #7D8B91; border-color: #364650; }"
        "QPlainTextEdit { background: #081018; color: #BFE8EC; border: 1px solid #2C4653; border-radius: 10px; padding: 8px; }"
        "QLabel { color: #BACBD1; }");

    QVBoxLayout* rootLayout = new QVBoxLayout(this);

    QLabel* titleLabel = new QLabel("先测后焊功能");
    titleLabel->setStyleSheet("font-size: 22px; font-weight: bold; color: #F7FCFC;");
    rootLayout->addWidget(titleLabel);

    QLabel* hintLabel = new QLabel("预设参数：读取 PreciseMeasureParam.ini 并执行安全姿态、扫描起点、扫描终点、收枪姿态；扫描段采集相机三维点，并在扫描后自动执行 PreservePath 拟合、焊道分类、焊接姿态生成和焊道补偿。也可以跳过扫描，直接选历史结果文件夹焊接。");
    rootLayout->addWidget(hintLabel);

    QGridLayout* buttonLayout = new QGridLayout();
    m_pPresetParamBtn = new QPushButton("预设参数");
    m_pSkipScanWeldBtn = new QPushButton("跳过扫描焊接");
    m_pLineScanProcessBtn = new QPushButton("线扫处理");
    m_pPresetParamBtn->setMinimumHeight(64);
    m_pSkipScanWeldBtn->setMinimumHeight(64);
    m_pLineScanProcessBtn->setMinimumHeight(64);
    buttonLayout->addWidget(m_pPresetParamBtn, 0, 0);
    buttonLayout->addWidget(m_pLineScanProcessBtn, 0, 1);
    buttonLayout->addWidget(m_pSkipScanWeldBtn, 1, 0, 1, 2);
    rootLayout->addLayout(buttonLayout);

    m_pLogText = new QPlainTextEdit();
    m_pLogText->setReadOnly(true);
    m_pLogText->setPlainText("流程日志：等待操作...");
    rootLayout->addWidget(m_pLogText, 1);

    connect(m_pPresetParamBtn, &QPushButton::clicked, this, &MeasureThenWeldDialog::RunPresetParamFlow);
    connect(m_pSkipScanWeldBtn, &QPushButton::clicked, this, &MeasureThenWeldDialog::RunSkipScanWeldFlow);
    connect(m_pLineScanProcessBtn, &QPushButton::clicked, this, &MeasureThenWeldDialog::RunLineScanProcess);
}

void MeasureThenWeldDialog::closeEvent(QCloseEvent* event)
{
    if (m_bRunning)
    {
        QMessageBox::information(this, "先测后焊", "流程正在运行，请等待当前流程结束后再关闭。");
        event->ignore();
        return;
    }
    QDialog::closeEvent(event);
}

FANUCRobotCtrl* MeasureThenWeldDialog::GetFirstFanucDriver()
{
    if (m_pContralUnit == nullptr || m_pContralUnit->m_vtContralUnitInfo.empty())
    {
        QMessageBox::warning(this, "先测后焊", "未找到可用的控制单元。");
        return nullptr;
    }

    RobotDriverAdaptor* pRobotDriverAdaptor = static_cast<RobotDriverAdaptor*>(m_pContralUnit->m_vtContralUnitInfo[0].pUnitDriver);
    if (pRobotDriverAdaptor == nullptr)
    {
        QMessageBox::warning(this, "先测后焊", "当前控制单元未创建驱动。");
        return nullptr;
    }

    FANUCRobotCtrl* pFanucDriver = dynamic_cast<FANUCRobotCtrl*>(pRobotDriverAdaptor);
    if (pFanucDriver == nullptr)
    {
        QMessageBox::warning(this, "先测后焊", "当前控制单元不是 FANUC 驱动。");
        return nullptr;
    }
    return pFanucDriver;
}

bool MeasureThenWeldDialog::LoadPresetParam(FANUCRobotCtrl* pFanucDriver, T_PRECISE_MEASURE_PARAM& param, QString& error)
{
    return m_pService != nullptr && m_pService->LoadPresetParam(pFanucDriver, param, error);
}

bool MeasureThenWeldDialog::ReadPulse(COPini& ini, const std::string& prefix, T_ANGLE_PULSE& pulse, QString& error) const
{
    return m_pService != nullptr && m_pService->ReadPulse(ini, prefix, pulse, error);
}

bool MeasureThenWeldDialog::ReadCoors(COPini& ini, const std::string& prefix, T_ROBOT_COORS& coors, QString& error) const
{
    return m_pService != nullptr && m_pService->ReadCoors(ini, prefix, coors, error);
}

bool MeasureThenWeldDialog::ReadPulseList(COPini& ini, const std::string& countKey, const std::string& prefix, std::vector<T_ANGLE_PULSE>& pulses, QString& error) const
{
    return m_pService != nullptr && m_pService->ReadPulseList(ini, countKey, prefix, pulses, error);
}

bool MeasureThenWeldDialog::MovePulseAndWait(FANUCRobotCtrl* pFanucDriver, const T_ANGLE_PULSE& pulse, double speed, const QString& name)
{
    return m_pService != nullptr && m_pService->MovePulseAndWait(
        pFanucDriver,
        pulse,
        speed,
        name,
        [this](const QString& text) { AppendLog(text); },
        [this](const QString& text) { SetFlowStep(text); });
}

bool MeasureThenWeldDialog::MovePulseListAndWait(FANUCRobotCtrl* pFanucDriver, const std::vector<T_ANGLE_PULSE>& pulses, double speed, const QString& name)
{
    return m_pService != nullptr && m_pService->MovePulseListAndWait(
        pFanucDriver,
        pulses,
        speed,
        name,
        [this](const QString& text) { AppendLog(text); },
        [this](const QString& text) { SetFlowStep(text); });
}

bool MeasureThenWeldDialog::MoveCoorsAndWait(FANUCRobotCtrl* pFanucDriver, const T_ROBOT_COORS& coors, double speed, const QString& name)
{
    return m_pService != nullptr && m_pService->MoveCoorsAndWait(
        pFanucDriver,
        coors,
        speed,
        name,
        [this](const QString& text) { AppendLog(text); },
        [this](const QString& text) { SetFlowStep(text); });
}

bool MeasureThenWeldDialog::ScanMoveAndCollect(FANUCRobotCtrl* pFanucDriver, const T_PRECISE_MEASURE_PARAM& param, QString& savedPath)
{
    return m_pService != nullptr && m_pService->ScanMoveAndCollect(
        pFanucDriver,
        param,
        savedPath,
        [this](const QString& text) { AppendLog(text); },
        [this](const QString& text) { SetFlowStep(text); });
}

QString MeasureThenWeldDialog::BuildResultDir(const std::string& robotName) const
{
    return m_pService != nullptr ? m_pService->BuildResultDir(robotName) : QString();
}

bool MeasureThenWeldDialog::SaveTextLines(const QString& filePath, const std::vector<QString>& lines, QString& error) const
{
    return m_pService != nullptr && m_pService->SaveTextLines(filePath, lines, error);
}

bool MeasureThenWeldDialog::ConfirmContinue(const QString& actionName)
{
    SetFlowStep(QString("等待确认：%1").arg(actionName));

    if (QThread::currentThread() != thread())
    {
        // 流程在线程里跑，确认框必须切回 UI 线程并阻塞等待用户选择。
        bool confirmed = false;
        QPointer<MeasureThenWeldDialog> self(this);
        QMetaObject::invokeMethod(qApp, [self, actionName, &confirmed]()
            {
                if (self == nullptr)
                {
                    confirmed = false;
                    return;
                }
                confirmed = self->ConfirmContinue(actionName);
            }, Qt::BlockingQueuedConnection);
        return confirmed;
    }

    const QMessageBox::StandardButton ret = QMessageBox::question(
        this,
        "先测后焊确认",
        QString("即将执行：%1\n\n请确认机器人周围安全，是否继续移动？\n选择“取消”将退出当前流程。").arg(actionName),
        QMessageBox::Ok | QMessageBox::Cancel,
        QMessageBox::Cancel);
    const bool confirmed = (ret == QMessageBox::Ok);
    AppendLog(QString("%1：%2").arg(actionName).arg(confirmed ? "已确认" : "已取消"));
    return confirmed;
}

bool MeasureThenWeldDialog::ShowCheckpointDialog(const QString& title, const QString& detail)
{
    SetFlowStep(QString("关键节点确认：%1").arg(title));

    if (QThread::currentThread() != thread())
    {
        bool confirmed = false;
        QPointer<MeasureThenWeldDialog> self(this);
        QMetaObject::invokeMethod(qApp, [self, title, detail, &confirmed]()
            {
                if (self == nullptr)
                {
                    confirmed = false;
                    return;
                }
                confirmed = self->ShowCheckpointDialog(title, detail);
            }, Qt::BlockingQueuedConnection);
        return confirmed;
    }

    const QMessageBox::StandardButton ret = QMessageBox::question(
        this,
        title,
        detail + "\n\n选择“确定”继续，选择“取消”终止当前流程。",
        QMessageBox::Ok | QMessageBox::Cancel,
        QMessageBox::Ok);
    const bool confirmed = (ret == QMessageBox::Ok);
    AppendLog(QString("关键节点[%1]：%2").arg(title).arg(confirmed ? "已确认继续" : "已取消流程"));
    return confirmed;
}

void MeasureThenWeldDialog::RunPresetParamFlow()
{
    if (m_bRunning)
    {
        QMessageBox::information(this, "先测后焊", "流程正在运行。");
        return;
    }

    FANUCRobotCtrl* pFanucDriver = GetFirstFanucDriver();
    if (pFanucDriver == nullptr)
    {
        return;
    }

    T_PRECISE_MEASURE_PARAM param;
    QString error;
    if (!LoadPresetParam(pFanucDriver, param, error))
    {
        QMessageBox::warning(this, "预设参数", error);
        return;
    }
    if (!CameraFrameAccess::TryBeginMeasureThenWeldExclusive())
    {
        QMessageBox::information(this, "预设参数", "已有先测后焊流程正在独占相机帧，请等待当前流程结束。");
        return;
    }

    SetRunning(true);
    SetFlowStep("读取预设参数完成，准备启动相机");
    AppendLog(QString("已读取参数：%1 [%2]").arg(QString::fromStdString(param.sIniFilePath)).arg(QString::fromStdString(param.sSectionName)));

    // 机器人运动和扫描采集放到后台线程，避免 UI 被 CheckRobotDone 和文件保存卡住。
    QPointer<MeasureThenWeldDialog> self(this);
    std::thread([self, pFanucDriver, param]()
        {
            bool ok = true;
            QString message;
            QString cameraIP;
            QString savedPath;
            QString executeSummary;

            QMetaObject::invokeMethod(qApp, [self, &cameraIP, &ok]()
                {
                    // 相机 UDP 线程由主界面统一管理，这里通过回调启动。
                    if (self == nullptr)
                    {
                        ok = false;
                        return;
                    }
                    ok = self->m_startCamera ? self->m_startCamera(cameraIP) : false;
                }, Qt::BlockingQueuedConnection);

            if (!ok)
            {
                message = "相机启动失败，流程中止。";
            }
            else
            {
                QMetaObject::invokeMethod(qApp, [self, cameraIP]()
                    {
                        if (self != nullptr)
                        {
                            self->SetFlowStep(QString("相机接收已启动：%1，准备下枪安全姿态").arg(cameraIP));
                            self->AppendLog(QString("相机接收已启动：%1").arg(cameraIP));
                        }
                    }, Qt::QueuedConnection);

                ok = self != nullptr && self->ConfirmContinue("下枪安全姿态");
                if (ok)
                {
                    self->SetFlowStep("准备移动到下枪安全姿态");
                    // 1. 下枪前先到安全姿态，避免直接切入扫描起点。
                    ok = self != nullptr && self->MovePulseListAndWait(pFanucDriver, param.vtStartSafePulse, SafeSpeed(param.dRunSpeed, 1.0), "下枪安全姿态");
                }
                if (ok)
                {
                    ok = self != nullptr && self->ConfirmContinue("移动到扫描起点");
                }
                if (ok)
                {
                    self->SetFlowStep("准备移动到扫描起点");
                    // 2. 到扫描起点使用直线运动，保持扫描段的空间姿态连续。
                    ok = self != nullptr && self->MoveCoorsAndWait(pFanucDriver, param.tStartPos, SafeSpeed(param.dRunSpeed, 1.0), "扫描起点");
                }
                if (ok)
                {
                    ok = self != nullptr && self->ConfirmContinue("扫描终点并采集相机点");
                }
                if (ok)
                {
                    self->SetFlowStep("准备扫描终点并采集相机点");
                    // 3. 从扫描起点运动到扫描终点，同时按 10ms 周期读取相机点。
                    ok = self != nullptr && self->ScanMoveAndCollect(pFanucDriver, param, savedPath);
                }
                if (ok)
                {
                    ok = self != nullptr && self->ConfirmContinue("收枪姿态");
                }
                if (ok)
                {
                    self->SetFlowStep("准备移动到收枪姿态");
                    // 4. 扫描结束后收枪到安全姿态。
                    ok = self != nullptr && self->MovePulseListAndWait(pFanucDriver, param.vtEndSafePulse, SafeSpeed(param.dRunSpeed, 1.0), "收枪姿态");
                }
                if (ok)
                {
                    const QFileInfo weldPoseFileInfo(savedPath);
                    if (!weldPoseFileInfo.isFile())
                    {
                        if (self != nullptr)
                        {
                            self->AppendLog(QString("未生成可下发的焊接姿态文件，当前结果=%1").arg(savedPath));
                        }
                        ok = false;
                        message = "预设参数流程已完成测量，但未生成可下发的焊接姿态文件。";
                    }
                }
                if (ok)
                {
                    ok = self != nullptr && self->ShowCheckpointDialog(
                        "扫描完成",
                        QString("扫描、拟合、焊道分类和焊接姿态生成已完成。\n焊接姿态文件：%1").arg(savedPath));
                }
                if (ok)
                {
                    ok = self != nullptr && self->ConfirmContinue("移动到焊接下枪安全位置并执行焊接轨迹");
                }
                if (ok)
                {
                    QString executeError;
                    T_ROBOT_COORS startSafeCoors;
                    T_ROBOT_COORS endSafeCoors;
                    if (self != nullptr)
                    {
                        self->SetFlowStep("准备执行焊接轨迹");
                        self->AppendLog(QString("开始执行焊接轨迹：%1").arg(savedPath));
                    }

                    ok = self != nullptr
                        && self->m_pService != nullptr
                        && self->m_pService->ExecuteWeldPoseFileWithSafePos(
                            pFanucDriver,
                            savedPath,
                            executeSummary,
                            executeError,
                            &startSafeCoors,
                            &endSafeCoors,
                            [self](const QString& text) { if (self != nullptr) self->AppendLog(text); },
                            [self](const QString& text) { if (self != nullptr) self->SetFlowStep(text); },
                            [self](const QString& title, const QString& detail) -> bool
                            {
                                return self != nullptr && self->ShowCheckpointDialog(title, detail);
                            });
                    if (ok)
                    {
                        if (self != nullptr)
                        {
                            self->AppendLog(QString("焊接轨迹执行完成：%1").arg(executeSummary));
                        }
                        message = QString("预设参数流程完成。\n结果位置：%1\n执行结果：%2\n下枪安全位置：%3\n收枪安全位置：%4")
                            .arg(savedPath)
                            .arg(executeSummary)
                            .arg(QString("%1, %2, %3")
                                .arg(startSafeCoors.dX, 0, 'f', 3)
                                .arg(startSafeCoors.dY, 0, 'f', 3)
                                .arg(startSafeCoors.dZ, 0, 'f', 3))
                            .arg(QString("%1, %2, %3")
                                .arg(endSafeCoors.dX, 0, 'f', 3)
                                .arg(endSafeCoors.dY, 0, 'f', 3)
                                .arg(endSafeCoors.dZ, 0, 'f', 3));
                    }
                    else
                    {
                        if (self != nullptr)
                        {
                            self->AppendLog(QString("焊接轨迹执行失败：%1").arg(executeError));
                        }
                        message = QString("预设参数流程已完成测量，但焊接轨迹执行失败。\n%1").arg(executeError);
                    }
                }
                else if (message.isEmpty())
                {
                    message = "预设参数流程失败，请查看流程日志。";
                }
            }

            QMetaObject::invokeMethod(qApp, [self, message, ok]()
                {
                    if (self == nullptr)
                    {
                        CameraFrameAccess::EndMeasureThenWeldExclusive();
                        return;
                    }
                    if (self->m_stopCamera)
                    {
                        // 流程正常完成、失败或用户取消，都会关闭相机接收。
                        self->m_stopCamera();
                    }
                    CameraFrameAccess::EndMeasureThenWeldExclusive();
                    self->SetFlowStep(ok ? "流程完成" : "流程失败，请查看流程日志");
                    self->AppendLog(ok ? "相机接收已停止，流程完成。" : "相机接收已停止，流程失败。");
                    self->SetRunning(false);
                    if (ok)
                    {
                        QMessageBox::information(self, "预设参数", message);
                    }
                    else
                    {
                        QMessageBox::warning(self, "预设参数", message);
                    }
                }, Qt::QueuedConnection);
        }).detach();
}

void MeasureThenWeldDialog::RunSkipScanWeldFlow()
{
    if (m_bRunning)
    {
        QMessageBox::information(this, "先测后焊", "流程正在运行。");
        return;
    }

    FANUCRobotCtrl* pFanucDriver = GetFirstFanucDriver();
    if (pFanucDriver == nullptr)
    {
        return;
    }

    T_PRECISE_MEASURE_PARAM param;
    QString error;
    if (!LoadPresetParam(pFanucDriver, param, error))
    {
        QMessageBox::warning(this, "跳过扫描焊接", error);
        return;
    }

    const QString defaultDir = RobotDataHelper::BuildProjectPath(
        QString("Result/%1").arg(QString::fromStdString(param.sRobotName)));
    const QString selectedDir = QFileDialog::getExistingDirectory(
        this,
        "选择先测后焊结果文件夹",
        defaultDir,
        QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    if (selectedDir.isEmpty())
    {
        return;
    }

    const QString laserDir = ResolveLaserPointDirFromSelection(selectedDir);
    if (laserDir.isEmpty())
    {
        QMessageBox::warning(
            this,
            "跳过扫描焊接",
            QString("在所选目录中未找到姿态文件 %1。\n请选择结果目录本身，或其下的 LaserPoint 目录。")
                .arg(WELD_POSE_FILE_NAME));
        return;
    }

    const QString poseFilePath = QDir(laserDir).filePath(WELD_POSE_FILE_NAME);
    const QString seamCompPath = QDir(laserDir).filePath(WELD_POSE_SEAM_COMP_FILE_NAME);
    if (!QFileInfo::exists(poseFilePath))
    {
        QMessageBox::warning(
            this,
            "跳过扫描焊接",
            QString("未找到姿态文件：%1").arg(poseFilePath));
        return;
    }

    SetRunning(true);
    SetFlowStep("已选择历史结果，准备按当前补偿参数重建焊接文件");
    AppendLog(QString("跳过扫描模式：结果目录=%1").arg(selectedDir));
    AppendLog(QString("姿态文件=%1").arg(poseFilePath));
    AppendLog(QString("补偿后文件将输出到=%1").arg(seamCompPath));

    QPointer<MeasureThenWeldDialog> self(this);
    std::thread([self, pFanucDriver, param, selectedDir, poseFilePath, seamCompPath]()
        {
            bool ok = true;
            QString message;
            QString processError;
            QString seamCompSummary;
            QString executeSummary;
            T_ROBOT_COORS startSafeCoors;
            T_ROBOT_COORS endSafeCoors;

            if (self != nullptr)
            {
                ok = self->ShowCheckpointDialog(
                    "跳过扫描确认",
                    QString("已选择结果目录：%1\n将读取姿态文件：%2\n并按当前焊道补偿参数重新生成补偿后文件。")
                        .arg(selectedDir)
                        .arg(poseFilePath));
            }

            if (ok)
            {
                if (self != nullptr)
                {
                    self->SetFlowStep("正在根据姿态文件重建焊道补偿结果");
                }
                ok = self != nullptr
                    && self->m_pService != nullptr
                    && self->m_pService->ApplyWeldSeamCompToPoseFile(
                        QString::fromStdString(param.sRobotName),
                        poseFilePath,
                        seamCompPath,
                        seamCompSummary,
                        processError);
                if (self != nullptr)
                {
                    if (ok)
                    {
                        self->AppendLog(QString("焊道补偿文件已更新：%1").arg(seamCompPath));
                        self->AppendLog(QString("焊道补偿摘要：%1").arg(seamCompSummary));
                    }
                    else
                    {
                        self->AppendLog(QString("重新生成焊道补偿文件失败：%1").arg(processError));
                    }
                }
            }

            if (ok)
            {
                ok = self != nullptr && self->ShowCheckpointDialog(
                    "补偿完成",
                    QString("姿态文件：%1\n补偿文件：%2\n%3")
                        .arg(poseFilePath)
                        .arg(seamCompPath)
                        .arg(seamCompSummary));
            }

            if (ok)
            {
                ok = self != nullptr && self->ConfirmContinue("移动到焊接下枪安全位置并执行焊接轨迹");
            }

            if (ok)
            {
                if (self != nullptr)
                {
                    self->SetFlowStep("准备执行跳过扫描后的焊接轨迹");
                    self->AppendLog(QString("开始执行焊接轨迹：%1").arg(seamCompPath));
                }

                ok = self != nullptr
                    && self->m_pService != nullptr
                    && self->m_pService->ExecuteWeldPoseFileWithSafePos(
                        pFanucDriver,
                        seamCompPath,
                        executeSummary,
                        processError,
                        &startSafeCoors,
                        &endSafeCoors,
                        [self](const QString& text) { if (self != nullptr) self->AppendLog(text); },
                        [self](const QString& text) { if (self != nullptr) self->SetFlowStep(text); },
                        [self](const QString& title, const QString& detail) -> bool
                        {
                            return self != nullptr && self->ShowCheckpointDialog(title, detail);
                        });
                if (ok)
                {
                    if (self != nullptr)
                    {
                        self->AppendLog(QString("焊接轨迹执行完成：%1").arg(executeSummary));
                    }
                    message = QString("跳过扫描焊接完成。\n结果目录：%1\n姿态文件：%2\n补偿文件：%3\n执行结果：%4\n下枪安全位置：%5\n收枪安全位置：%6")
                        .arg(selectedDir)
                        .arg(poseFilePath)
                        .arg(seamCompPath)
                        .arg(executeSummary)
                        .arg(QString("%1, %2, %3")
                            .arg(startSafeCoors.dX, 0, 'f', 3)
                            .arg(startSafeCoors.dY, 0, 'f', 3)
                            .arg(startSafeCoors.dZ, 0, 'f', 3))
                        .arg(QString("%1, %2, %3")
                            .arg(endSafeCoors.dX, 0, 'f', 3)
                            .arg(endSafeCoors.dY, 0, 'f', 3)
                            .arg(endSafeCoors.dZ, 0, 'f', 3));
                }
                else
                {
                    if (self != nullptr)
                    {
                        self->AppendLog(QString("焊接轨迹执行失败：%1").arg(processError));
                    }
                    message = QString("跳过扫描后焊接失败。\n%1").arg(processError);
                }
            }
            else if (message.isEmpty())
            {
                message = processError.isEmpty()
                    ? QString("跳过扫描焊接流程失败，请查看流程日志。")
                    : QString("跳过扫描焊接流程失败。\n%1").arg(processError);
            }

            QMetaObject::invokeMethod(qApp, [self, message, ok]()
                {
                    if (self == nullptr)
                    {
                        return;
                    }
                    self->SetFlowStep(ok ? "流程完成" : "流程失败，请查看流程日志");
                    self->AppendLog(ok ? "跳过扫描焊接流程完成。" : "跳过扫描焊接流程失败。");
                    self->SetRunning(false);
                    if (ok)
                    {
                        QMessageBox::information(self, "跳过扫描焊接", message);
                    }
                    else
                    {
                        QMessageBox::warning(self, "跳过扫描焊接", message);
                    }
                }, Qt::QueuedConnection);
        }).detach();
}

void MeasureThenWeldDialog::RunLineScanProcess()
{
    SetFlowStep("线扫处理功能暂未实现");
    AppendLog("线扫处理功能暂未实现。");
    QMessageBox::information(this, "线扫处理", "线扫处理暂时为空函数，后续接入点云处理算法。");
}

void MeasureThenWeldDialog::AppendLog(const QString& text)
{
    if (QThread::currentThread() != thread())
    {
        QPointer<MeasureThenWeldDialog> self(this);
        QMetaObject::invokeMethod(qApp, [self, text]()
            {
                if (self != nullptr)
                {
                    self->AppendLog(text);
                }
            }, Qt::QueuedConnection);
        return;
    }

    if (m_pLogText != nullptr)
    {
        m_pLogText->appendPlainText(QString("[%1] %2").arg(QDateTime::currentDateTime().toString("HH:mm:ss.zzz")).arg(text));
    }
}

void MeasureThenWeldDialog::SetFlowStep(const QString& text)
{
    if (QThread::currentThread() != thread())
    {
        QPointer<MeasureThenWeldDialog> self(this);
        QMetaObject::invokeMethod(qApp, [self, text]()
            {
                if (self != nullptr)
                {
                    self->SetFlowStep(text);
                }
            }, Qt::QueuedConnection);
        return;
    }

    emit FlowStepChanged(QString("先测后焊流程进行中，目前：%1").arg(text));
}

void MeasureThenWeldDialog::SetRunning(bool running)
{
    m_bRunning = running;
    m_pPresetParamBtn->setEnabled(!running);
    m_pSkipScanWeldBtn->setEnabled(!running);
    m_pLineScanProcessBtn->setEnabled(!running);
}
