#include "QtWidgetsApplication4.h"
#include <QMessageBox>  // 弹窗头文件，测试用
#include "FTPClient.h"
#include "FANUCRobotDriver.h"
#include "CameraParamDialog.h"
#include "FunctionTestDialog.h"
#include "MeasureThenWeldDialog.h"
#include "OPini.h"
#include "PreciseMeasureEditDialog.h"
#include "RobotDataHelper.h"
#include "RobotJogDialog.h"
#include "WindowStyleHelper.h"
#include "WeldProcessDialog.h"
#include "groove/clientudpformsensorworker.h"
#include "groove/framebuffer.h"
#include "groove/threadsafebuffer.h"
#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QInputDialog>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMetaObject>
#include <QMenuBar>
#include <QPushButton>
#include <QScrollBar>
#include <QSplitter>
#include <QStatusBar>
#include <QThread>
#include <QTimer>
#include <QToolBar>
#include <QTextStream>
#include <QVBoxLayout>
#include <QStringList>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <thread>
#ifdef Q_OS_WIN
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#endif

namespace
{
	QString FindProjectFilePath(const QString& relativePath)
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

	FANUCRobotCtrl* GetFirstFanucDriver(ContralUnit* contralUnit, QWidget* parent)
	{
		if (contralUnit == nullptr || contralUnit->m_vtContralUnitInfo.empty())
		{
			QMessageBox::warning(parent, "FANUC测试", "未找到可用的控制单元。");
			return nullptr;
		}

		RobotDriverAdaptor* pRobotDriverAdaptor = static_cast<RobotDriverAdaptor*>(contralUnit->m_vtContralUnitInfo[0].pUnitDriver);
		if (pRobotDriverAdaptor == nullptr)
		{
			QMessageBox::warning(parent, "FANUC测试", "当前控制单元未创建驱动。");
			return nullptr;
		}

		FANUCRobotCtrl* pFanucDriver = dynamic_cast<FANUCRobotCtrl*>(pRobotDriverAdaptor);
		if (pFanucDriver == nullptr)
		{
			QMessageBox::warning(parent, "FANUC测试", "当前控制单元不是 FANUC 驱动。");
			return nullptr;
		}
		return pFanucDriver;
	}

	QString FormatVectorPreview(const QVector<double>& values, int maxCount = 5)
	{
		QStringList parts;
		const int count = std::min(static_cast<int>(values.size()), maxCount);
		for (int i = 0; i < count; ++i)
		{
			parts << QString::number(values[i], 'f', 3);
		}
		if (values.size() > maxCount)
		{
			parts << "...";
		}
		return parts.join(", ");
	}

	QString BuildAppVersionText()
	{
		const QString version = QCoreApplication::applicationVersion().trimmed();
		return version.isEmpty() ? QStringLiteral("未标注版本") : version;
	}

	QString BuildAboutText()
	{
		const QString versionText = BuildAppVersionText();
		return QStringLiteral(
			"软件名称：%1\n"
			"版本号：v%2\n"
			"项目仓库：https://github.com/yu1201/NoTeaching-Robot\n"
			"安装目录：%3\n"
			"\n"
			"说明：\n"
			"1. 当前安装包已包含 Qt / OpenCV 运行库。\n"
			"2. 安装程序会自动补装 VC++ 运行库。\n"
			"3. 如需现场编译 FANUC 程序，安装包已附带 WinOLPC 编译工具。")
			.arg(QCoreApplication::applicationName(),
				versionText,
				QDir::toNativeSeparators(QCoreApplication::applicationDirPath()));
	}
}

QtWidgetsApplication4::QtWidgetsApplication4(QWidget* parent)
	: QMainWindow(parent)
	, m_pContralUnit(nullptr)
	, m_clientUDPFormSensorWorker(nullptr)
	, m_clientUDPFormSensorThread(nullptr)
	, m_grooveCameraDisplayTimer(nullptr)
	, m_robotLogDisplayTimer(nullptr)
	, m_pRobotLogText(nullptr)
	, m_pCameraParamBtn(nullptr)
	, m_bFanucMovlForward(true)
	, m_bFanucMovlRunning(false)
	, m_bFanucMovjRunning(false)
	, m_bFanucMoveZeroRunning(false)
	, m_bFanucMonitorReading(false)
{
	ui.setupUi(this);
	ApplyUnifiedWindowChrome(this);
	if (ui.menuBar != nullptr)
	{
		ui.menuBar->hide();
	}
	if (ui.mainToolBar != nullptr)
	{
		ui.mainToolBar->hide();
	}
	if (ui.statusBar != nullptr)
	{
		ui.statusBar->hide();
	}
	setWindowTitle(QString("%1 v%2").arg(QCoreApplication::applicationName(), BuildAppVersionText()));
	setMinimumSize(920, 700);
	setStyleSheet(
		"QMainWindow, QWidget { background: #111820; color: #ECF3F4; }"
		"QGroupBox { border: 1px solid #2E4656; border-radius: 14px; margin-top: 18px; padding: 16px; font-weight: bold; color: #9ED8DB; }"
		"QGroupBox::title { subcontrol-origin: margin; left: 18px; padding: 0 8px; }"
		"QPushButton { background: #233645; color: #F5FAFA; border: 1px solid #3C6173; border-radius: 12px; padding: 10px 16px; font-size: 15px; }"
		"QPushButton:hover { background: #2D5465; border-color: #72D4DD; }"
		"QPushButton:pressed { background: #18303B; }"
		"QPushButton:checked { background: #305F55; border-color: #7BD8B3; }"
		"QPlainTextEdit { background: #081018; color: #BFE8EC; border: 1px solid #2C4653; border-radius: 12px; padding: 10px; font-family: Consolas, 'Microsoft YaHei UI'; }"
		"QMenuBar, QStatusBar, QToolBar { background: #0B1117; color: #ECF3F4; }"
		"QLabel { color: #BACBD1; }");

	QWidget* mainPanel = new QWidget(this);
	QVBoxLayout* rootLayout = new QVBoxLayout(mainPanel);
	rootLayout->setContentsMargins(22, 18, 22, 18);
	rootLayout->setSpacing(14);
	setCentralWidget(mainPanel);

	QHBoxLayout* titleLayout = new QHBoxLayout();
	titleLayout->setSpacing(10);
	QLabel* titleLabel = new QLabel("机器人控制与调试中心");
	titleLabel->setStyleSheet("font-size: 26px; font-weight: bold; color: #F7FCFC; letter-spacing: 1px;");
	QLabel* versionLabel = new QLabel(QString("v%1").arg(BuildAppVersionText()));
	versionLabel->setStyleSheet(
		"QLabel { background: #173041; color: #9ED8DB; border: 1px solid #3C6173; "
		"border-radius: 10px; padding: 4px 10px; font-size: 13px; font-weight: bold; }");
	QPushButton* aboutButton = new QPushButton("关于");
	aboutButton->setMinimumHeight(32);
	aboutButton->setMaximumWidth(88);
	aboutButton->setStyleSheet("QPushButton { padding: 6px 12px; font-size: 13px; border-radius: 10px; }");
	titleLayout->addWidget(titleLabel);
	titleLayout->addWidget(versionLabel, 0, Qt::AlignVCenter);
	titleLayout->addStretch(1);
	titleLayout->addWidget(aboutButton, 0, Qt::AlignVCenter);
	rootLayout->addLayout(titleLayout);

	QGroupBox* entryGroup = new QGroupBox("常用功能");
	QGridLayout* entryLayout = new QGridLayout(entryGroup);
	entryLayout->setSpacing(12);
	m_pCameraParamBtn = new QPushButton("相机参数", entryGroup);
	const QList<QPushButton*> entryButtons = {
		ui.RunTest,
		ui.WeldProcessBtn,
		ui.FunctionTestBtn,
		ui.MeasureThenWeldBtn,
		ui.PreciseMeasureEditBtn,
		m_pCameraParamBtn,
		ui.RobotJogBtn,
		ui.FanucConnectBtn,
		ui.FanucDisconnectBtn,
		ui.GrooveCameraTestBtn
	};
	for (int i = 0; i < entryButtons.size(); ++i)
	{
		QPushButton* button = entryButtons[i];
		button->setMinimumHeight(52);
		entryLayout->addWidget(button, i / 3, i % 3);
	}

	QSplitter* infoSplitter = new QSplitter(Qt::Horizontal);
	infoSplitter->setChildrenCollapsible(false);

	QSplitter* robotInfoSplitter = new QSplitter(Qt::Vertical);
	robotInfoSplitter->setChildrenCollapsible(false);

	QGroupBox* monitorGroup = new QGroupBox("机器人监控");
	QVBoxLayout* monitorLayout = new QVBoxLayout(monitorGroup);
	monitorLayout->addWidget(ui.FanucMonitorText);

	QGroupBox* logGroup = new QGroupBox("运行日志");
	QVBoxLayout* logLayout = new QVBoxLayout(logGroup);
	QHBoxLayout* logButtonLayout = new QHBoxLayout();
	logButtonLayout->setSpacing(6);
	const QList<QPair<QString, QString>> logButtons = {
		{ "系统", "Log/Log.txt" },
		{ "运行", "Log/RobotRunLog.txt" },
		{ "机器人A", "Log/RobotALog.txt" },
		{ "控制单元", "Log/ContralUnit.txt" },
		{ "焊接", "Log/WeldProcessFile.txt" }
	};
	for (int i = 0; i < logButtons.size(); ++i)
	{
		QPushButton* button = new QPushButton(logButtons[i].first);
		button->setCheckable(true);
		button->setMinimumHeight(28);
		button->setMaximumHeight(32);
		button->setMinimumWidth(64);
		button->setStyleSheet("QPushButton { padding: 4px 8px; font-size: 13px; border-radius: 8px; }");
		logButtonLayout->addWidget(button);
		connect(button, &QPushButton::clicked, this, [this, logButtons, i, logButtonLayout]()
			{
				for (int j = 0; j < logButtonLayout->count(); ++j)
				{
					QPushButton* other = qobject_cast<QPushButton*>(logButtonLayout->itemAt(j)->widget());
					if (other != nullptr)
					{
						other->setChecked(j == i);
					}
				}
				LoadRobotLogFile(logButtons[i].second, true);
			});
		if (i == 0)
		{
			button->setChecked(true);
		}
	}
	logButtonLayout->addStretch(1);
	logLayout->addLayout(logButtonLayout);
	m_pRobotLogText = new QPlainTextEdit();
	m_pRobotLogText->setReadOnly(true);
	m_pRobotLogText->setPlainText("日志：等待读取...");
	logLayout->addWidget(m_pRobotLogText, 1);

	robotInfoSplitter->addWidget(monitorGroup);
	robotInfoSplitter->addWidget(logGroup);
	robotInfoSplitter->setStretchFactor(0, 1);
	robotInfoSplitter->setStretchFactor(1, 1);

	QGroupBox* cameraGroup = new QGroupBox("坡口相机数据");
	QVBoxLayout* cameraLayout = new QVBoxLayout(cameraGroup);
	cameraLayout->addWidget(ui.GrooveCameraText);
	infoSplitter->addWidget(robotInfoSplitter);
	infoSplitter->addWidget(cameraGroup);
	infoSplitter->setStretchFactor(0, 1);
	infoSplitter->setStretchFactor(1, 1);
	rootLayout->addWidget(infoSplitter, 1);
	rootLayout->addWidget(entryGroup, 0);

	connect(ui.WeldProcessBtn, &QPushButton::clicked, this, &QtWidgetsApplication4::OpenWeldProcessDialog);
	connect(ui.FunctionTestBtn, &QPushButton::clicked, this, &QtWidgetsApplication4::OpenFunctionTestDialog);
	connect(ui.MeasureThenWeldBtn, &QPushButton::clicked, this, &QtWidgetsApplication4::OpenMeasureThenWeldDialog);
	connect(ui.PreciseMeasureEditBtn, &QPushButton::clicked, this, &QtWidgetsApplication4::OpenPreciseMeasureEditDialog);
	connect(m_pCameraParamBtn, &QPushButton::clicked, this, &QtWidgetsApplication4::OpenCameraParamDialog);
	connect(aboutButton, &QPushButton::clicked, this, &QtWidgetsApplication4::OpenAboutDialog);
	connect(ui.FanucConnectBtn, &QPushButton::clicked, this, &QtWidgetsApplication4::FanucConnectTest);
	connect(ui.FanucDisconnectBtn, &QPushButton::clicked, this, &QtWidgetsApplication4::FanucDisconnectTest);
	connect(ui.FanucGetPosBtn, &QPushButton::clicked, this, &QtWidgetsApplication4::FanucGetCurrentPosTest);
	connect(ui.FanucGetPulseBtn, &QPushButton::clicked, this, &QtWidgetsApplication4::FanucGetCurrentPulseTest);
	connect(ui.FanucCheckDoneBtn, &QPushButton::clicked, this, &QtWidgetsApplication4::FanucCheckDoneTest);
	connect(ui.FanucSetGetIntBtn, &QPushButton::clicked, this, &QtWidgetsApplication4::FanucSetGetIntTest);
	connect(ui.FanucSetSpeedBtn, &QPushButton::clicked, this, &QtWidgetsApplication4::FanucSetTpSpeedTest);
	connect(ui.FanucCallJobBtn, &QPushButton::clicked, this, &QtWidgetsApplication4::FanucCallJobTest);
	connect(ui.FanucUploadLsBtn, &QPushButton::clicked, this, &QtWidgetsApplication4::FanucUploadLsTest);
	connect(ui.FanucMovlTestBtn, &QPushButton::clicked, this, &QtWidgetsApplication4::FanucMovlTest);
	connect(ui.FanucMovjTestBtn, &QPushButton::clicked, this, &QtWidgetsApplication4::FanucMovjTest);
	connect(ui.FanucMoveZeroBtn, &QPushButton::clicked, this, &QtWidgetsApplication4::FanucMoveZeroTest);
	connect(ui.RobotJogBtn, &QPushButton::clicked, this, &QtWidgetsApplication4::OpenRobotJogDialog);
	connect(ui.GrooveCameraTestBtn, &QPushButton::toggled, this, &QtWidgetsApplication4::GrooveCameraTest);

	ui.FanucGetPosBtn->hide();
	ui.FanucGetPulseBtn->hide();
	ui.FanucCheckDoneBtn->hide();
	ui.FanucSetGetIntBtn->hide();
	ui.FanucSetSpeedBtn->hide();
	ui.FanucCallJobBtn->hide();
	ui.FanucUploadLsBtn->hide();
	ui.FanucMovlTestBtn->hide();
	ui.FanucMovjTestBtn->hide();
	ui.FanucMoveZeroBtn->hide();

	m_clientUDPFormSensorWorker = new ClientUDPFormSensorWorker();
	m_clientUDPFormSensorThread = new QThread(this);
	connect(this, &QtWidgetsApplication4::startAllCommThreads, m_clientUDPFormSensorWorker, &ClientUDPFormSensorWorker::startReceive);
	connect(this, &QtWidgetsApplication4::stopAllCommThreads, m_clientUDPFormSensorWorker, &ClientUDPFormSensorWorker::stopReceive);
	connect(m_clientUDPFormSensorThread, &QThread::finished, m_clientUDPFormSensorWorker, &QObject::deleteLater);
	m_clientUDPFormSensorWorker->moveToThread(m_clientUDPFormSensorThread);
	m_clientUDPFormSensorThread->start();

	m_grooveCameraDisplayTimer = new QTimer(this);
	connect(m_grooveCameraDisplayTimer, &QTimer::timeout, this, &QtWidgetsApplication4::UpdateGrooveCameraData);

	m_robotLogDisplayTimer = new QTimer(this);
	connect(m_robotLogDisplayTimer, &QTimer::timeout, this, [this]()
		{
			if (!m_sCurrentRobotLogPath.isEmpty())
			{
				LoadRobotLogFile(m_sCurrentRobotLogPath);
			}
		});
	LoadRobotLogFile("Log/Log.txt", true);
	m_robotLogDisplayTimer->start(1000);

	QTimer* fanucMonitorTimer = new QTimer(this);
	connect(fanucMonitorTimer, &QTimer::timeout, this, [this]()
		{
			if (m_pContralUnit == nullptr || m_pContralUnit->m_vtContralUnitInfo.empty())
			{
				return;
			}

			RobotDriverAdaptor* pRobotDriverAdaptor = static_cast<RobotDriverAdaptor*>(m_pContralUnit->m_vtContralUnitInfo[0].pUnitDriver);
			FANUCRobotCtrl* pFanucDriver = dynamic_cast<FANUCRobotCtrl*>(pRobotDriverAdaptor);
			if (pFanucDriver == nullptr)
			{
				return;
			}

			pFanucDriver->StartMonitor();
			long long robotMs = 0;
			long long pcRecvMs = 0;
			const T_ROBOT_COORS pos = pFanucDriver->GetCurrentPosPassive(&robotMs, &pcRecvMs);
			const T_ANGLE_PULSE pulse = pFanucDriver->GetCurrentPulsePassive();
			const int done = pFanucDriver->CheckDonePassive();
			const QString stateText = done == 0 ? "运行中" : (done == 1 ? "停止/完成" : QString("未知/异常(%1)").arg(done));
			QString monitorText = QString(
				"状态: %1\n"
				"robot_ms=%2  pc_recv_ms=%3\n"
				"位置: X=%4  Y=%5  Z=%6  W=%7  P=%8  R=%9\n"
				"脉冲: S=%10  L=%11  U=%12  R=%13  B=%14  T=%15  EX1=%16  EX2=%17  EX3=%18")
				.arg(stateText)
				.arg(robotMs)
				.arg(pcRecvMs)
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
			if (!m_sMeasureThenWeldStatus.isEmpty())
			{
				monitorText += "\n\n" + m_sMeasureThenWeldStatus;
			}
			ui.FanucMonitorText->setPlainText(monitorText);
		});
	fanucMonitorTimer->start(50);
	//RobotLog* ContralUnitLog = new RobotLog(".//Log//ContralUnit.txt");
	//FtpClient* pFTP = new FtpClient(ContralUnitLog, "192.168.39.222");
	//pFTP->downloadFile("/UserPrograms/testcyh.sr/test1.srp", ".//Job//STEP//test1.srp");
	m_pContralUnit = new ContralUnit();

}

QtWidgetsApplication4::~QtWidgetsApplication4()
{
	if (m_grooveCameraDisplayTimer != nullptr)
	{
		m_grooveCameraDisplayTimer->stop();
	}
	if (m_robotLogDisplayTimer != nullptr)
	{
		m_robotLogDisplayTimer->stop();
	}
	if (m_clientUDPFormSensorWorker != nullptr && m_clientUDPFormSensorThread != nullptr && m_clientUDPFormSensorThread->isRunning())
	{
		QMetaObject::invokeMethod(m_clientUDPFormSensorWorker, "stopReceive", Qt::BlockingQueuedConnection);
		m_clientUDPFormSensorThread->quit();
		m_clientUDPFormSensorThread->wait();
		m_clientUDPFormSensorWorker = nullptr;
	}
	delete m_pContralUnit;
	m_pContralUnit = nullptr;
}

void QtWidgetsApplication4::OpenAboutDialog()
{
	QMessageBox aboutBox(this);
	aboutBox.setWindowTitle("关于");
	aboutBox.setIcon(QMessageBox::Information);
	aboutBox.setText(BuildAboutText());
	aboutBox.exec();
}

void QtWidgetsApplication4::ApplyStartupArguments(const QStringList& arguments)
{
	if (arguments.size() <= 1)
	{
		return;
	}

	EnsureCommandLineConsole();
	QTimer::singleShot(0, this, [this, arguments]()
		{
			RunCommandLineActions(arguments);
		});
}

void QtWidgetsApplication4::RunCommandLineActions(const QStringList& arguments)
{
	if (arguments.contains("--help-cli"))
	{
		QTextStream out(stdout);
		out << "QtWidgetsApplication4 command line options:\n";
		out << "  --no-show                         不显示主窗口，适合自动测试\n";
		out << "  --open-function-test              打开 FANUC 功能测试窗口\n";
		out << "  --open-jog                        打开机器人点动控制窗口\n";
		out << "  --open-precise-measure            打开精测量数据修改窗口\n";
		out << "  --open-camera-param               打开相机参数窗口\n";
		out << "  --fanuc-connect                   连接 FANUC 常驻服务端口\n";
		out << "  --fanuc-upload-services           上传/编译 FANUC 服务库和固定 TP\n";
		out << "  --skip-upload-wait                上传服务后不等待回车，自动化测试用\n";
		out << "  --fanuc-curpos-diag               运行当前位置/PR20 诊断命令\n";
		out << "  --fanuc-pr20-diag                 仅读取 FANUC PR[20] 诊断点\n";
		out << "  --fanuc-raw <CMD>                 发送一条原始 FANUC 服务命令\n";
		out << "  --fanuc-call <PROGRAM>            调用机器人程序\n";
		out << "  --quit-after <ms>                 指定毫秒后退出程序\n";
		out.flush();
		QTimer::singleShot(0, QCoreApplication::instance(), &QCoreApplication::quit);
		return;
	}

	if (arguments.contains("--open-function-test"))
	{
		LogCommandLineMessage("CLI 打开功能测试窗口");
		OpenFunctionTestDialog();
	}
	if (arguments.contains("--open-jog"))
	{
		LogCommandLineMessage("CLI 打开机器人点动控制窗口");
		OpenRobotJogDialog();
	}
	if (arguments.contains("--open-precise-measure"))
	{
		LogCommandLineMessage("CLI 打开精测量数据修改窗口");
		OpenPreciseMeasureEditDialog();
	}
	if (arguments.contains("--open-camera-param"))
	{
		LogCommandLineMessage("CLI 打开相机参数窗口");
		OpenCameraParamDialog();
	}

	FANUCRobotCtrl* pFanucDriver = GetFirstFanucDriverForCli();
	const bool needsFanuc = arguments.contains("--fanuc-connect")
		|| arguments.contains("--fanuc-upload-services")
		|| arguments.contains("--fanuc-curpos-diag")
		|| arguments.contains("--fanuc-pr20-diag")
		|| arguments.contains("--fanuc-raw")
		|| arguments.contains("--fanuc-call");
	if (needsFanuc && pFanucDriver == nullptr)
	{
		LogCommandLineMessage("CLI 未找到 FANUC 驱动，跳过 FANUC 命令。");
	}
	if (pFanucDriver != nullptr)
	{
		const bool needsSocket = arguments.contains("--fanuc-connect")
			|| arguments.contains("--fanuc-curpos-diag")
			|| arguments.contains("--fanuc-pr20-diag")
			|| arguments.contains("--fanuc-raw")
			|| arguments.contains("--fanuc-call");

		bool uploadOk = true;
		if (arguments.contains("--fanuc-upload-services"))
		{
			uploadOk = UploadFanucServiceBundleForCli(pFanucDriver);
			if (uploadOk)
			{
				if (arguments.contains("--skip-upload-wait"))
				{
					LogCommandLineMessage("CLI 已跳过上传后的回车等待。");
				}
				else
				{
					WaitForCommandLineEnter("FANUC 服务文件已上传完成。请在示教器重新运行 STARTALL，确认服务启动后按回车继续。");
				}
			}
		}

		bool socketReady = !needsSocket;
		if (needsSocket && uploadOk)
		{
			socketReady = pFanucDriver->InitSocket(pFanucDriver->m_sSocketIP.c_str(), static_cast<u_short>(pFanucDriver->m_nSocketPort));
			LogCommandLineMessage(QString("CLI FANUC连接%1：%2:%3")
				.arg(socketReady ? "成功" : "失败")
				.arg(QString::fromStdString(pFanucDriver->m_sSocketIP))
				.arg(pFanucDriver->m_nSocketPort));
		}

		if (!uploadOk)
		{
			LogCommandLineMessage("CLI FANUC 服务文件上传失败，跳过后续 FANUC socket 命令。");
		}
		else if (needsSocket && !socketReady)
		{
			LogCommandLineMessage("CLI FANUC socket 未连接，跳过 RAW/CALL/诊断命令。");
		}

		for (int i = 1; socketReady && i < arguments.size(); ++i)
		{
			if (arguments[i] == "--fanuc-raw" && i + 1 < arguments.size())
			{
				const QString command = arguments[++i];
				const std::string response = pFanucDriver->SendRawCommandForTest(command.toStdString());
				LogCommandLineMessage(QString("CLI FANUC RAW %1 -> %2")
					.arg(command, QString::fromStdString(response)));
			}
			else if (arguments[i] == "--fanuc-call" && i + 1 < arguments.size())
			{
				const QString program = arguments[++i];
				const bool ok = pFanucDriver->CallJob(program.toStdString());
				LogCommandLineMessage(QString("CLI FANUC CALL %1 -> %2").arg(program, ok ? "OK" : "FAIL"));
			}
		}

		if (socketReady && arguments.contains("--fanuc-curpos-diag"))
		{
			RunFanucCurposDiagnosticForCli(pFanucDriver);
		}

		if (socketReady && arguments.contains("--fanuc-pr20-diag"))
		{
			const std::string response = pFanucDriver->SendRawCommandForTest("GET_POS_VAR:20,0");
			LogCommandLineMessage(QString("CLI FANUC PR20 -> %1").arg(QString::fromStdString(response)));
		}
	}

	const int quitAfterIndex = arguments.indexOf("--quit-after");
	if (quitAfterIndex >= 0 && quitAfterIndex + 1 < arguments.size())
	{
		bool ok = false;
		const int quitAfterMs = arguments[quitAfterIndex + 1].toInt(&ok);
		if (ok && quitAfterMs >= 0)
		{
			LogCommandLineMessage(QString("CLI 动作完成，将在 %1 ms 后退出").arg(quitAfterMs));
			QTimer::singleShot(quitAfterMs, QCoreApplication::instance(), &QCoreApplication::quit);
		}
	}
}

void QtWidgetsApplication4::LogCommandLineMessage(const QString& message) const
{
	const QString line = QString("[%1] %2")
		.arg(QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss.zzz"), message);
	QTextStream(stdout) << line << Qt::endl;

	QString logPath = FindProjectFilePath("Log/RobotALog.txt");
	if (logPath.isEmpty())
	{
		QDir().mkpath("Log");
		logPath = QDir(QDir::currentPath()).filePath("Log/RobotALog.txt");
	}

	QFile file(logPath);
	if (file.open(QIODevice::Append | QIODevice::Text))
	{
		QTextStream stream(&file);
		stream << line << "\n";
	}
}

void QtWidgetsApplication4::EnsureCommandLineConsole() const
{
#ifdef Q_OS_WIN
	if (GetConsoleWindow() == nullptr)
	{
		if (!AttachConsole(ATTACH_PARENT_PROCESS))
		{
			AllocConsole();
		}
	}

	FILE* stream = nullptr;
	freopen_s(&stream, "CONOUT$", "w", stdout);
	freopen_s(&stream, "CONOUT$", "w", stderr);
	freopen_s(&stream, "CONIN$", "r", stdin);
	std::ios::sync_with_stdio(true);
#endif
}

void QtWidgetsApplication4::WaitForCommandLineEnter(const QString& message) const
{
	EnsureCommandLineConsole();
	LogCommandLineMessage(message);
	QTextStream(stdout) << "\n" << message << "\n按回车继续..." << Qt::flush;
	std::string unusedLine;
	std::getline(std::cin, unusedLine);
	LogCommandLineMessage("CLI 已收到回车，继续执行后续命令。");
}

FANUCRobotCtrl* QtWidgetsApplication4::GetFirstFanucDriverForCli() const
{
	if (m_pContralUnit == nullptr || m_pContralUnit->m_vtContralUnitInfo.empty())
	{
		return nullptr;
	}

	RobotDriverAdaptor* pRobotDriverAdaptor = static_cast<RobotDriverAdaptor*>(m_pContralUnit->m_vtContralUnitInfo[0].pUnitDriver);
	return dynamic_cast<FANUCRobotCtrl*>(pRobotDriverAdaptor);
}

bool QtWidgetsApplication4::UploadFanucServiceBundleForCli(FANUCRobotCtrl* pFanucDriver)
{
	if (pFanucDriver == nullptr)
	{
		return false;
	}

	struct UploadItem
	{
		QString label;
		QString localRelativePath;
		QString remotePath;
		int uploadMode;
	};
	const QList<UploadItem> items = {
		{ "服务库", "SDK/FANUC/FanucServiceLib.kl", QString(), 0 },
		{ "常驻服务", "SDK/FANUC/FanucResidentService.kl", QString(), 0 },
		{ "监控服务", "SDK/FANUC/FanucMonitorService.kl", QString(), 0 },
		{ "点动缓冲加载程序", "SDK/FANUC/LOADJOGBUF.kl", QString(), 0 },
		{ "合并启动TP", "SDK/FANUC/STARTALL.tp", "/md/STARTALL.tp", 1 },
		{ "直角点动TP", "SDK/FANUC/FANUC_JOGL.ls", QString(), 2 },
		{ "关节点动TP", "SDK/FANUC/FANUC_JOGJ.ls", QString(), 2 }
	};

	LogCommandLineMessage("CLI 准备上传 FANUC 服务文件，先请求机器人停止常驻服务。");
	const bool stopOk = pFanucDriver->StopRobotServices();
	LogCommandLineMessage(QString("CLI 停止 FANUC 常驻服务 -> %1，随后继续上传文件。")
		.arg(stopOk ? "OK" : "FAIL/可能已停止"));

	LogCommandLineMessage("CLI 开始上传 FANUC 服务文件");
	for (const UploadItem& item : items)
	{
		const QString localPath = FindProjectFilePath(item.localRelativePath);
		if (localPath.isEmpty())
		{
			LogCommandLineMessage(QString("CLI 上传失败：未找到%1，文件=%2").arg(item.label, item.localRelativePath));
			return false;
		}

		const QByteArray localPathBytes = localPath.toLocal8Bit();
		const QByteArray remotePathBytes = item.remotePath.toLocal8Bit();
		int ret = -1;
		if (item.uploadMode == 0)
		{
			ret = pFanucDriver->UploadKlFile(localPathBytes.constData());
		}
		else if (item.uploadMode == 1)
		{
			ret = pFanucDriver->UploadFile(localPathBytes.constData(), remotePathBytes.constData());
		}
		else
		{
			ret = pFanucDriver->UploadLsFile(localPathBytes.constData());
		}
		LogCommandLineMessage(QString("CLI 上传%1 -> 返回码=%2，文件=%3")
			.arg(item.label)
			.arg(ret)
			.arg(localPath));
		if (ret != 0)
		{
			return false;
		}
	}

	LogCommandLineMessage("CLI FANUC 服务文件上传完成。请在示教器重新运行 STARTALL，确认服务已启动后再继续测试。");
	return true;
}

void QtWidgetsApplication4::RunFanucCurposDiagnosticForCli(FANUCRobotCtrl* pFanucDriver)
{
	if (pFanucDriver == nullptr)
	{
		return;
	}

	const QStringList commands = {
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

	LogCommandLineMessage("CLI 开始 FANUC 当前位置诊断");
	for (const QString& command : commands)
	{
		const std::string response = pFanucDriver->SendRawCommandForTest(command.toStdString());
		LogCommandLineMessage(QString("CLI FANUC DIAG %1 -> %2")
			.arg(command, QString::fromStdString(response)));
	}
}

bool QtWidgetsApplication4::LoadGrooveCameraIP(QString& cameraIP) const
{
	std::string robotName = "RobotA";
	if (m_pContralUnit != nullptr && !m_pContralUnit->m_vtContralUnitInfo.empty() && !m_pContralUnit->m_vtContralUnitInfo[0].sUnitName.empty())
	{
		robotName = m_pContralUnit->m_vtContralUnitInfo[0].sUnitName;
	}

    RobotDataHelper::CameraParamData cameraParam;
	const QString robotNameText = QString::fromStdString(robotName);
    const QString cameraSection = RobotDataHelper::MeasureCameraSection(robotNameText);
    if (!RobotDataHelper::LoadCameraParam(robotNameText, cameraSection, cameraParam, nullptr)
		|| cameraParam.deviceAddress.trimmed().isEmpty())
	{
		return false;
	}

	cameraIP = cameraParam.deviceAddress.trimmed();
	return true;
}

void QtWidgetsApplication4::LoadRobotLogFile(const QString& relativePath, bool forceRefresh)
{
	m_sCurrentRobotLogPath = relativePath;
	if (m_pRobotLogText == nullptr)
	{
		return;
	}

	const QString filePath = FindProjectFilePath(relativePath);
	if (filePath.isEmpty())
	{
		if (forceRefresh || m_sLastRobotLogFilePath != relativePath)
		{
			m_pRobotLogText->setPlainText(QString("未找到日志文件：%1").arg(relativePath));
			m_sLastRobotLogFilePath = relativePath;
			m_lastRobotLogModified = QDateTime();
			m_nLastRobotLogSize = -1;
		}
		return;
	}

	const QFileInfo fileInfo(filePath);
	const QDateTime lastModified = fileInfo.lastModified();
	const qint64 fileSize = fileInfo.size();
	if (!forceRefresh
		&& m_sLastRobotLogFilePath == filePath
		&& m_lastRobotLogModified == lastModified
		&& m_nLastRobotLogSize == fileSize)
	{
		return;
	}

	QFile file(filePath);
	if (!file.open(QIODevice::ReadOnly))
	{
		if (forceRefresh || m_sLastRobotLogFilePath != filePath)
		{
			m_pRobotLogText->setPlainText(QString("日志文件打开失败：%1").arg(filePath));
		}
		m_sLastRobotLogFilePath = filePath;
		m_lastRobotLogModified = lastModified;
		m_nLastRobotLogSize = fileSize;
		return;
	}

	const qint64 maxBytes = 48 * 1024;
	if (file.size() > maxBytes)
	{
		file.seek(file.size() - maxBytes);
	}
	const QString text = QString::fromLocal8Bit(file.readAll());
	m_pRobotLogText->setPlainText(QString("文件：%1\n\n%2").arg(filePath, text));
	m_pRobotLogText->verticalScrollBar()->setValue(m_pRobotLogText->verticalScrollBar()->maximum());
	m_sLastRobotLogFilePath = filePath;
	m_lastRobotLogModified = lastModified;
	m_nLastRobotLogSize = fileSize;
}

void QtWidgetsApplication4::GrooveCameraTest(bool checked)
{
	if (checked)
	{
		QString cameraIP;
		if (!LoadGrooveCameraIP(cameraIP))
		{
			QMessageBox::warning(this, "坡口相机测试", "未读取到 CAMERA0 的 DeviceAddress。");
			ui.GrooveCameraTestBtn->setChecked(false);
			return;
		}

		ThreadSafeBuffer<udpDataShow>::Instance().clear();
		ui.GrooveCameraText->setPlainText(QString("正在接收 CAMERA0：%1\n本地UDP端口：50005").arg(cameraIP));
		emit startAllCommThreads(cameraIP);
		m_grooveCameraDisplayTimer->start(100);
	}
	else
	{
		if (m_grooveCameraDisplayTimer != nullptr)
		{
			m_grooveCameraDisplayTimer->stop();
		}
		emit stopAllCommThreads();
		ui.GrooveCameraText->appendPlainText("已停止坡口相机接收。");
	}
}

void QtWidgetsApplication4::UpdateGrooveCameraData()
{
	udpDataShow frame;
	udpDataShow latestFrame;
	bool hasFrame = false;
	while (ThreadSafeBuffer<udpDataShow>::Instance().dequeue(frame, 0))
	{
		latestFrame = frame;
		hasFrame = true;
	}
	if (!hasFrame)
	{
		return;
	}

	const QString text = QString(
		"点云数量: %1\n"
		"拟合点数量: %2\n"
		"目标点: X=%3  Y=%4  Z=%5\n"
		"点云X预览: %6\n"
		"点云Y预览: %7\n"
		"拟合X预览: %8\n"
		"拟合Y预览: %9\n"
		"错误信息: %10")
		.arg(latestFrame.XData.size())
		.arg(latestFrame.fitLineX.size())
		.arg(latestFrame.targetPoint.x, 0, 'f', 3)
		.arg(latestFrame.targetPoint.y, 0, 'f', 3)
		.arg(latestFrame.targetPoint.z, 0, 'f', 3)
		.arg(FormatVectorPreview(latestFrame.XData))
		.arg(FormatVectorPreview(latestFrame.YData))
		.arg(FormatVectorPreview(latestFrame.fitLineX))
		.arg(FormatVectorPreview(latestFrame.fitLineY))
		.arg(latestFrame.errorMessage.isEmpty() ? "无" : latestFrame.errorMessage);
	ui.GrooveCameraText->setPlainText(text);
}


void QtWidgetsApplication4::RobotRunTest()
{
	if (m_pContralUnit == nullptr || m_pContralUnit->m_vtContralUnitInfo.empty())
	{
		QMessageBox::warning(this, "测试程序", "未找到可用的控制单元。");
		return;
	}

	T_CONTRAL_UNIT& contralUnitInfo = m_pContralUnit->m_vtContralUnitInfo[0];
	RobotDriverAdaptor* pRobotDriverAdaptor = static_cast<RobotDriverAdaptor*>(contralUnitInfo.pUnitDriver);
	if (pRobotDriverAdaptor == nullptr)
	{
		QMessageBox::warning(this, "测试程序", "当前控制单元未创建驱动。");
		return;
	}

	FANUCRobotCtrl* pFanucDriver = dynamic_cast<FANUCRobotCtrl*>(pRobotDriverAdaptor);
	if (pFanucDriver != nullptr)
	{
		const bool stopOk = pFanucDriver->StopRobotServices();
		if (!stopOk)
		{
			QMessageBox::warning(this, "FANUC测试程序", "停止常驻服务失败，可能服务已经停止或网络未连接。\n将继续上传文件，上传完成后请在示教器重新运行 STARTALL。");
		}

		const QString serviceLibPath = FindProjectFilePath("SDK/FANUC/FanucServiceLib.kl");
		const QString residentServicePath = FindProjectFilePath("SDK/FANUC/FanucResidentService.kl");
		const QString monitorServicePath = FindProjectFilePath("SDK/FANUC/FanucMonitorService.kl");
		const QString loadJogBufferPath = FindProjectFilePath("SDK/FANUC/LOADJOGBUF.kl");
		const QString startAllPath = FindProjectFilePath("SDK/FANUC/STARTALL.tp");
		const QString joglPath = FindProjectFilePath("SDK/FANUC/FANUC_JOGL.ls");
		const QString jogjPath = FindProjectFilePath("SDK/FANUC/FANUC_JOGJ.ls");
		if (serviceLibPath.isEmpty() || residentServicePath.isEmpty() || monitorServicePath.isEmpty() || loadJogBufferPath.isEmpty() || startAllPath.isEmpty() || joglPath.isEmpty() || jogjPath.isEmpty())
		{
			QMessageBox::warning(this, "FANUC测试程序", "未找到测试程序文件：FanucServiceLib.kl / FanucResidentService.kl / FanucMonitorService.kl / LOADJOGBUF.kl / STARTALL.tp / FANUC_JOGL.ls / FANUC_JOGJ.ls");
			return;
		}

		const QByteArray serviceLibPathBytes = serviceLibPath.toLocal8Bit();
		const int libRet = pFanucDriver->UploadKlFile(serviceLibPathBytes.constData());
		if (libRet != 0)
		{
			QMessageBox::warning(this, "FANUC测试程序", GetStr("服务库发送失败，返回码=%d\n文件=%s", libRet, serviceLibPathBytes.constData()).c_str());
			return;
		}

		const QByteArray residentServicePathBytes = residentServicePath.toLocal8Bit();
		const int residentRet = pFanucDriver->UploadKlFile(residentServicePathBytes.constData());
		if (residentRet != 0)
		{
			QMessageBox::warning(this, "FANUC测试程序", GetStr("常驻服务发送失败，返回码=%d\n文件=%s", residentRet, residentServicePathBytes.constData()).c_str());
			return;
		}

		const QByteArray monitorServicePathBytes = monitorServicePath.toLocal8Bit();
		const int monitorRet = pFanucDriver->UploadKlFile(monitorServicePathBytes.constData());
		if (monitorRet != 0)
		{
			QMessageBox::warning(this, "FANUC测试程序", GetStr("监控服务发送失败，返回码=%d\n文件=%s", monitorRet, monitorServicePathBytes.constData()).c_str());
			return;
		}

		const QByteArray loadJogBufferPathBytes = loadJogBufferPath.toLocal8Bit();
		const int loadJogBufferRet = pFanucDriver->UploadKlFile(loadJogBufferPathBytes.constData());
		if (loadJogBufferRet != 0)
		{
			QMessageBox::warning(this, "FANUC测试程序", GetStr("点动缓冲加载程序发送失败，返回码=%d\n文件=%s", loadJogBufferRet, loadJogBufferPathBytes.constData()).c_str());
			return;
		}

		const QByteArray startAllPathBytes = startAllPath.toLocal8Bit();
		const int startAllRet = pFanucDriver->UploadFile(startAllPathBytes.constData(), "/md/STARTALL.tp");
		if (startAllRet != 0)
		{
			QMessageBox::warning(this, "FANUC测试程序", GetStr("合并启动TP发送失败，返回码=%d\n文件=%s", startAllRet, startAllPathBytes.constData()).c_str());
			return;
		}

		const QByteArray joglPathBytes = joglPath.toLocal8Bit();
		const int joglRet = pFanucDriver->UploadLsFile(joglPathBytes.constData());
		if (joglRet != 0)
		{
			QMessageBox::warning(this, "FANUC测试程序", GetStr("直角点动TP发送失败，返回码=%d\n文件=%s", joglRet, joglPathBytes.constData()).c_str());
			return;
		}

		const QByteArray jogjPathBytes = jogjPath.toLocal8Bit();
		const int jogjRet = pFanucDriver->UploadLsFile(jogjPathBytes.constData());
		if (jogjRet != 0)
		{
			QMessageBox::warning(this, "FANUC测试程序", GetStr("关节点动TP发送失败，返回码=%d\n文件=%s", jogjRet, jogjPathBytes.constData()).c_str());
			return;
		}

		QMessageBox::information(this, "FANUC测试程序",
			GetStr("常驻服务、监控服务、点动缓冲程序和点动TP发送成功。\n\n现在请在示教器重新运行 STARTALL，确认服务已启动后再点击确定。\n\n文件：\n%s\n%s\n%s\n%s\n%s\n%s\n%s",
				serviceLibPathBytes.constData(),
				residentServicePathBytes.constData(),
				monitorServicePathBytes.constData(),
				loadJogBufferPathBytes.constData(),
				startAllPathBytes.constData(),
				joglPathBytes.constData(),
				jogjPathBytes.constData()).c_str());
		return;
	}

	T_ANGLE_PULSE tNowPulse = pRobotDriverAdaptor->GetCurrentPulse();
	T_ANGLE_PULSE testPulse = tNowPulse;
	if (testPulse.nSPulse == 0 && testPulse.nLPulse == 0 && testPulse.nUPulse == 0 &&
		testPulse.nRPulse == 0 && testPulse.nBPulse == 0 && testPulse.nTPulse == 0) {
		testPulse = T_ANGLE_PULSE(90000, 180000, 0, 0, 0, 0, 0, 0, 0);
	}

	T_ANGLE_PULSE bestResult;
	const bool ok = pRobotDriverAdaptor->RunKinematicsSelfTest(testPulse, T_ROBOT_COORS(), &bestResult);

	if (ok) {
		QMessageBox::information(
			this,
			"运动学自检",
			GetStr("FK -> IK -> FK 自检完成。\n回代脉冲: S=%ld L=%ld U=%ld R=%ld B=%ld T=%ld",
				bestResult.nSPulse, bestResult.nLPulse, bestResult.nUPulse,
				bestResult.nRPulse, bestResult.nBPulse, bestResult.nTPulse).c_str());
	}
	else {
		QMessageBox::warning(this, "运动学自检", "FK -> IK -> FK 自检失败，请查看日志。");
	}
}

void QtWidgetsApplication4::OpenWeldProcessDialog()
{
	if (m_pContralUnit == nullptr || m_pContralUnit->m_vtContralUnitInfo.empty())
	{
		QMessageBox::warning(this, "工艺参数", "未找到可用的控制单元。");
		return;
	}

	WeldProcessDialog dlg(m_pContralUnit->m_vtContralUnitInfo[0], this);
	dlg.exec();
}

void QtWidgetsApplication4::OpenFunctionTestDialog()
{
	FunctionTestDialog* dialog = new FunctionTestDialog(m_pContralUnit, this);
	dialog->setAttribute(Qt::WA_DeleteOnClose);
	dialog->show();
	dialog->raise();
	dialog->activateWindow();
}

void QtWidgetsApplication4::OpenMeasureThenWeldDialog()
{
	auto startCamera = [this](QString& cameraIP) -> bool
		{
			if (!LoadGrooveCameraIP(cameraIP))
			{
				return false;
			}
			ThreadSafeBuffer<udpDataShow>::Instance().clear();
			emit startAllCommThreads(cameraIP);
			return true;
		};

	auto stopCamera = [this]()
		{
			emit stopAllCommThreads();
		};

	MeasureThenWeldDialog* dialog = new MeasureThenWeldDialog(m_pContralUnit, startCamera, stopCamera, this);
	dialog->setAttribute(Qt::WA_DeleteOnClose);
	connect(dialog, &MeasureThenWeldDialog::FlowStepChanged, this, [this](const QString& text)
		{
			m_sMeasureThenWeldStatus = text;
			if (ui.FanucMonitorText != nullptr && ui.FanucMonitorText->toPlainText().isEmpty())
			{
				ui.FanucMonitorText->setPlainText(text);
			}
		});
	dialog->show();
	dialog->raise();
	dialog->activateWindow();
}

void QtWidgetsApplication4::OpenPreciseMeasureEditDialog()
{
	PreciseMeasureEditDialog* dialog = new PreciseMeasureEditDialog(m_pContralUnit, this);
	dialog->setAttribute(Qt::WA_DeleteOnClose);
	dialog->show();
	dialog->raise();
	dialog->activateWindow();
}

void QtWidgetsApplication4::OpenCameraParamDialog()
{
	CameraParamDialog* dialog = new CameraParamDialog(m_pContralUnit, this);
	dialog->setAttribute(Qt::WA_DeleteOnClose);
	dialog->show();
	dialog->raise();
	dialog->activateWindow();
}

void QtWidgetsApplication4::FanucConnectTest()
{
	FANUCRobotCtrl* pFanucDriver = GetFirstFanucDriver(m_pContralUnit, this);
	if (pFanucDriver == nullptr)
	{
		return;
	}

	const bool ok = pFanucDriver->InitSocket(pFanucDriver->m_sSocketIP.c_str(), static_cast<u_short>(pFanucDriver->m_nSocketPort));
	if (ok)
	{
		QMessageBox::information(this, "FANUC连接", GetStr("连接成功：%s:%d", pFanucDriver->m_sSocketIP.c_str(), pFanucDriver->m_nSocketPort).c_str());
	}
	else
	{
		QMessageBox::warning(this, "FANUC连接", GetStr("连接失败：%s:%d", pFanucDriver->m_sSocketIP.c_str(), pFanucDriver->m_nSocketPort).c_str());
	}
}

void QtWidgetsApplication4::FanucDisconnectTest()
{
	FANUCRobotCtrl* pFanucDriver = GetFirstFanucDriver(m_pContralUnit, this);
	if (pFanucDriver == nullptr)
	{
		return;
	}

	const bool ok = pFanucDriver->StopRobotServices();
	QMessageBox::information(this, "FANUC断开", ok ? "已请求机器人服务退出，并断开本地连接。" : "请求机器人服务退出失败，已尝试断开本地连接。");
}

void QtWidgetsApplication4::FanucGetCurrentPosTest()
{
	FANUCRobotCtrl* pFanucDriver = GetFirstFanucDriver(m_pContralUnit, this);
	if (pFanucDriver == nullptr)
	{
		return;
	}

	const T_ROBOT_COORS pos = pFanucDriver->GetCurrentPos();
	QMessageBox::information(
		this,
		"读取当前位置",
		GetStr("X=%.3f\nY=%.3f\nZ=%.3f\nRX=%.3f\nRY=%.3f\nRZ=%.3f",
			pos.dX, pos.dY, pos.dZ, pos.dRX, pos.dRY, pos.dRZ).c_str());
}

void QtWidgetsApplication4::FanucGetCurrentPulseTest()
{
	FANUCRobotCtrl* pFanucDriver = GetFirstFanucDriver(m_pContralUnit, this);
	if (pFanucDriver == nullptr)
	{
		return;
	}

	const T_ANGLE_PULSE pulse = pFanucDriver->GetCurrentPulse();
	QMessageBox::information(
		this,
		"读取关节脉冲",
		GetStr("S=%ld\nL=%ld\nU=%ld\nR=%ld\nB=%ld\nT=%ld\nEX1=%ld\nEX2=%ld\nEX3=%ld",
			pulse.nSPulse, pulse.nLPulse, pulse.nUPulse, pulse.nRPulse, pulse.nBPulse, pulse.nTPulse,
			pulse.lBXPulse, pulse.lBYPulse, pulse.lBZPulse).c_str());
}

void QtWidgetsApplication4::FanucCheckDoneTest()
{
	FANUCRobotCtrl* pFanucDriver = GetFirstFanucDriver(m_pContralUnit, this);
	if (pFanucDriver == nullptr)
	{
		return;
	}

	const int done = pFanucDriver->CheckDone();
	QMessageBox::information(this, "检查运行完成", GetStr("CheckDone 返回值：%d", done).c_str());
}

void QtWidgetsApplication4::FanucSetGetIntTest()
{
	FANUCRobotCtrl* pFanucDriver = GetFirstFanucDriver(m_pContralUnit, this);
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
	QMessageBox::information(this, "写读INT寄存器", GetStr("写入 INT%d=%d\n读取值=%d", index, value, readValue).c_str());
}

void QtWidgetsApplication4::FanucSetTpSpeedTest()
{
	FANUCRobotCtrl* pFanucDriver = GetFirstFanucDriver(m_pContralUnit, this);
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
	const std::string message = setOk ? GetStr("设置速度成功：%d", speed) : GetStr("设置速度失败：%d", speed);
	QMessageBox::information(this, "设置速度", message.c_str());
}

void QtWidgetsApplication4::FanucCallJobTest()
{
	FANUCRobotCtrl* pFanucDriver = GetFirstFanucDriver(m_pContralUnit, this);
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
	const std::string message = callOk ? GetStr("调用任务成功：%s", jobNameBytes.constData()) : GetStr("调用任务失败：%s", jobNameBytes.constData());
	QMessageBox::information(this, "调用任务", message.c_str());
}

void QtWidgetsApplication4::FanucUploadLsTest()
{
	FANUCRobotCtrl* pFanucDriver = GetFirstFanucDriver(m_pContralUnit, this);
	if (pFanucDriver == nullptr)
	{
		return;
	}

	const QString lsPath = FindProjectFilePath("SDK/FANUC/STARTALL.ls");
	if (lsPath.isEmpty())
	{
		QMessageBox::warning(this, "发送LS程序", "未找到测试程序文件：SDK/FANUC/STARTALL.ls");
		return;
	}

	const QByteArray lsPathBytes = lsPath.toLocal8Bit();
	const int ret = pFanucDriver->UploadLsFile(lsPathBytes.constData());
	if (ret == 0)
	{
		QMessageBox::information(this, "发送LS程序", GetStr("LS程序发送成功：%s", lsPathBytes.constData()).c_str());
	}
	else
	{
		QMessageBox::warning(this, "发送LS程序", GetStr("LS程序发送失败，返回码=%d\n文件=%s", ret, lsPathBytes.constData()).c_str());
	}
}

void QtWidgetsApplication4::FanucMovlTest()
{
	FANUCRobotCtrl* pFanucDriver = GetFirstFanucDriver(m_pContralUnit, this);
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
	ui.FanucMovlTestBtn->setEnabled(false);

	std::thread([this, pFanucDriver, moveForward]()
		{
			T_ROBOT_COORS target = pFanucDriver->GetCurrentPos();
			target.dY += moveForward ? 100.0 : -100.0;

			const bool moveOk = pFanucDriver->MoveByJob(target, T_ROBOT_MOVE_SPEED(5.0, 0.0, 0.0), pFanucDriver->m_nExternalAxleType, "MOVL");
			const int done = moveOk ? pFanucDriver->CheckRobotDone(200) : -1;
			const QString message = QString("MOVL %1 100mm\nMove=%2\nCheckRobotDone=%3")
				.arg(moveForward ? "Y+" : "Y-")
				.arg(moveOk ? "OK" : "FAIL")
				.arg(done);

			QMetaObject::invokeMethod(this, [this, message]()
				{
					m_bFanucMovlRunning = false;
					ui.FanucMovlTestBtn->setEnabled(true);
					QMessageBox::information(this, "MOVL往返测试", message);
				}, Qt::QueuedConnection);
		}).detach();
}

void QtWidgetsApplication4::FanucMovjTest()
{
	FANUCRobotCtrl* pFanucDriver = GetFirstFanucDriver(m_pContralUnit, this);
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
	ui.FanucMovjTestBtn->setEnabled(false);

	std::thread([this, pFanucDriver]()
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
			const QString message = QString("MOVJ J2/J3 +5deg\nJ2DeltaPulse=%1\nJ3DeltaPulse=%2\nMove=%3\nCheckRobotDone=%4\n提示：固定TP当前用R[17]%，测试速度取1%%。")
				.arg(j2DeltaPulse)
				.arg(j3DeltaPulse)
				.arg(moveOk ? "OK" : "FAIL")
				.arg(done);

			QMetaObject::invokeMethod(this, [this, message]()
				{
					m_bFanucMovjRunning = false;
					ui.FanucMovjTestBtn->setEnabled(true);
					QMessageBox::information(this, "MOVJ测试", message);
				}, Qt::QueuedConnection);
		}).detach();
}

void QtWidgetsApplication4::FanucMoveZeroTest()
{
	FANUCRobotCtrl* pFanucDriver = GetFirstFanucDriver(m_pContralUnit, this);
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
	ui.FanucMoveZeroBtn->setEnabled(false);

	std::thread([this, pFanucDriver]()
		{
			const T_ANGLE_PULSE zeroPulse = T_ANGLE_PULSE();
			const T_ROBOT_MOVE_SPEED speed(1.0, 0.0, 0.0);
			const bool moveOk = pFanucDriver->MoveByJob(zeroPulse, speed, pFanucDriver->m_nExternalAxleType, "MOVJ");
			const int done = moveOk ? pFanucDriver->CheckRobotDone(200) : -1;
			const T_ROBOT_COORS pos = pFanucDriver->GetCurrentPos();
			const T_ANGLE_PULSE pulse = pFanucDriver->GetCurrentPulse();

			const QString message = QString(
				"MOVJ 到零位\n"
				"Move=%1\n"
				"CheckRobotDone=%2\n\n"
				"当前位置:\n"
				"X=%3\nY=%4\nZ=%5\nRX=%6\nRY=%7\nRZ=%8\n\n"
				"当前脉冲/角度反馈:\n"
				"S=%9\nL=%10\nU=%11\nR=%12\nB=%13\nT=%14\nEX1=%15\nEX2=%16\nEX3=%17")
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

			QMetaObject::invokeMethod(this, [this, message]()
				{
					m_bFanucMoveZeroRunning = false;
					ui.FanucMoveZeroBtn->setEnabled(true);
					QMessageBox::information(this, "运动到零位", message);
				}, Qt::QueuedConnection);
		}).detach();
}

void QtWidgetsApplication4::OpenRobotJogDialog()
{
	FANUCRobotCtrl* pFanucDriver = GetFirstFanucDriver(m_pContralUnit, this);
	if (pFanucDriver == nullptr)
	{
		return;
	}

	RobotJogDialog* dialog = new RobotJogDialog(pFanucDriver, this);
	dialog->setAttribute(Qt::WA_DeleteOnClose);
	dialog->show();
	dialog->raise();
	dialog->activateWindow();
}
