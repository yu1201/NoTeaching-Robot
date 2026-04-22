#include "RobotJogDialog.h"

#include "FANUCRobotDriver.h"
#include "WindowStyleHelper.h"

#include <QApplication>
#include <QCoreApplication>
#include <QDoubleValidator>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QMetaObject>
#include <QPointer>
#include <QSettings>
#include <QValidator>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>
#include <thread>

namespace
{
	constexpr int STREAM_BUFFER_COUNT = 20;
	constexpr int STREAM_START_POINT_COUNT = 20;
	constexpr int JOG_HOLD_START_DELAY_MS = 180;
	constexpr int STREAM_FEED_INTERVAL_MS = 500;
	constexpr double STREAM_POINT_TIME_SEC = 0.5;

	QPushButton* CreateJogButton(const QString& text)
	{
		QPushButton* button = new QPushButton(text);
		button->setMinimumSize(42, 34);
		button->setCursor(Qt::PointingHandCursor);
		return button;
	}

	double AxisPulseUnit(const FANUCRobotCtrl* driver, int axisIndex)
	{
		if (driver == nullptr)
		{
			return 0.0;
		}

		switch (axisIndex)
		{
		case 0: return driver->m_tAxisUnit.dSPulseUnit;
		case 1: return driver->m_tAxisUnit.dLPulseUnit;
		case 2: return driver->m_tAxisUnit.dUPulseUnit;
		case 3: return driver->m_tAxisUnit.dRPulseUnit;
		case 4: return driver->m_tAxisUnit.dBPulseUnit;
		case 5: return driver->m_tAxisUnit.dTPulseUnit;
		case 6: return driver->m_tAxisUnit.dBXPulseUnit;
		case 7: return driver->m_tAxisUnit.dBYPulseUnit;
		case 8: return driver->m_tAxisUnit.dBZPulseUnit;
		default: return 0.0;
		}
	}

	void AddJointDelta(T_ANGLE_PULSE& pulse, int axisIndex, long deltaPulse)
	{
		switch (axisIndex)
		{
		case 0: pulse.nSPulse += deltaPulse; break;
		case 1: pulse.nLPulse += deltaPulse; break;
		case 2: pulse.nUPulse += deltaPulse; break;
		case 3: pulse.nRPulse += deltaPulse; break;
		case 4: pulse.nBPulse += deltaPulse; break;
		case 5: pulse.nTPulse += deltaPulse; break;
		case 6: pulse.lBXPulse += deltaPulse; break;
		case 7: pulse.lBYPulse += deltaPulse; break;
		case 8: pulse.lBZPulse += deltaPulse; break;
		default: break;
		}
	}

	void AddCartesianDelta(T_ROBOT_COORS& pos, int axisIndex, double delta)
	{
		switch (axisIndex)
		{
		case 0: pos.dX += delta; break;
		case 1: pos.dY += delta; break;
		case 2: pos.dZ += delta; break;
		case 3: pos.dRX += delta; break;
		case 4: pos.dRY += delta; break;
		case 5: pos.dRZ += delta; break;
		default: break;
		}
	}

	QString JogSettingsPath()
	{
		return QCoreApplication::applicationDirPath() + "/RobotJogDialog.ini";
	}

	void LogCartesianPoint(FANUCRobotCtrl* driver, const char* prefix, const T_ROBOT_COORS& pos)
	{
		if (driver == nullptr || driver->m_pRobotLog == nullptr || prefix == nullptr)
		{
			return;
		}

		driver->m_pRobotLog->write(LogColor::DEFAULT,
			"%s: X=%.3f Y=%.3f Z=%.3f RX=%.3f RY=%.3f RZ=%.3f BX=%.3f BY=%.3f BZ=%.3f",
			prefix,
			pos.dX, pos.dY, pos.dZ,
			pos.dRX, pos.dRY, pos.dRZ,
			pos.dBX, pos.dBY, pos.dBZ);
	}

	void LogJointPoint(FANUCRobotCtrl* driver, const char* prefix, const T_ANGLE_PULSE& pulse)
	{
		if (driver == nullptr || driver->m_pRobotLog == nullptr || prefix == nullptr)
		{
			return;
		}

		driver->m_pRobotLog->write(LogColor::DEFAULT,
			"%s: J1=%ld J2=%ld J3=%ld J4=%ld J5=%ld J6=%ld EX1=%ld EX2=%ld EX3=%ld",
			prefix,
			pulse.nSPulse, pulse.nLPulse, pulse.nUPulse,
			pulse.nRPulse, pulse.nBPulse, pulse.nTPulse,
			pulse.lBXPulse, pulse.lBYPulse, pulse.lBZPulse);
	}
}

RobotJogDialog::RobotJogDialog(FANUCRobotCtrl* fanucDriver, QWidget* parent)
	: QDialog(parent)
	, m_fanucDriver(fanucDriver)
	, m_jogStartTimer(new QTimer(this))
	, m_jogTimer(new QTimer(this))
	, m_stateTimer(new QTimer(this))
	, m_stateLabel(nullptr)
	, m_cartesianSpeedEdit(nullptr)
	, m_jointSpeedEdit(nullptr)
	, m_stopButton(nullptr)
	, m_jogActive(false)
	, m_currentMode(JogMode::Cartesian)
	, m_currentAxis(0)
	, m_currentDirection(0)
	, m_nextStreamStep(0)
	, m_streamCartesianSpeed(60.0)
	, m_streamJointSpeed(1.0)
{
	setWindowTitle("机器人点动控制");
	ApplyUnifiedWindowChrome(this);
	setMinimumSize(860, 600);
	ResizeWindowForAvailableGeometry(this, QSize(1120, 680), 0.86, 0.78);
	BuildUi();
	ApplyStyle();
	LoadSpeedSettings();

	m_jogStartTimer->setSingleShot(true);
	m_jogStartTimer->setInterval(JOG_HOLD_START_DELAY_MS);
	connect(m_jogStartTimer, &QTimer::timeout, this, [this]() { BeginJog(); });
	connect(m_jogTimer, &QTimer::timeout, this, [this]() { FeedNextPoint(); });
	connect(m_stateTimer, &QTimer::timeout, this, [this]() { RefreshStateText(); });
	m_jogTimer->setInterval(STREAM_FEED_INTERVAL_MS);
	m_stateTimer->start(200);
	RefreshStateText();
	ReadCurrentCartesianTarget();
	ReadCurrentJointTarget();
}

RobotJogDialog::~RobotJogDialog()
{
	StopJog();
	SaveSpeedSettings();
}

void RobotJogDialog::BuildUi()
{
	QVBoxLayout* root = new QVBoxLayout(this);
	root->setContentsMargins(18, 18, 18, 18);
	root->setSpacing(14);

	QLabel* title = new QLabel("机器人点动控制");
	title->setObjectName("TitleLabel");
	QLabel* subtitle = new QLabel("当前版本已临时禁用长按连续点动，仅保留单击步进和编辑目标值后运动到指定位置。");
	subtitle->setObjectName("SubTitleLabel");

	m_stateLabel = new QLabel("状态: 等待数据...");
	m_stateLabel->setObjectName("StateCard");
	m_stateLabel->setMinimumHeight(92);

	QHBoxLayout* speedLayout = new QHBoxLayout();
	speedLayout->setSpacing(12);

	QGroupBox* cartSpeedBox = new QGroupBox("直角速度");
	QHBoxLayout* cartSpeedLayout = new QHBoxLayout(cartSpeedBox);
	m_cartesianSpeedEdit = new QLineEdit("60");
	m_cartesianSpeedEdit->setValidator(new QDoubleValidator(0.01, 9999.0, 3, this));
	cartSpeedLayout->addWidget(m_cartesianSpeedEdit);
	cartSpeedLayout->addWidget(new QLabel("mm/min 或 deg/min"));

	QGroupBox* jointSpeedBox = new QGroupBox("关节速度");
	QHBoxLayout* jointSpeedLayout = new QHBoxLayout(jointSpeedBox);
	m_jointSpeedEdit = new QLineEdit("1");
	m_jointSpeedEdit->setValidator(new QDoubleValidator(1.0, 100.0, 3, this));
	jointSpeedLayout->addWidget(m_jointSpeedEdit);
	jointSpeedLayout->addWidget(new QLabel("%"));

	m_stopButton = new QPushButton("长按已禁用");
	m_stopButton->setObjectName("DangerButton");
	m_stopButton->setMinimumHeight(48);
	m_stopButton->setEnabled(false);
	m_stopButton->setToolTip("当前版本仅保留单击步进和运动到指定位置。");
	connect(m_cartesianSpeedEdit, &QLineEdit::editingFinished, this, [this]() { SaveSpeedSettings(); });
	connect(m_jointSpeedEdit, &QLineEdit::editingFinished, this, [this]() { SaveSpeedSettings(); });

	speedLayout->addWidget(cartSpeedBox, 1);
	speedLayout->addWidget(jointSpeedBox, 1);
	speedLayout->addWidget(m_stopButton);

	QHBoxLayout* axisCards = new QHBoxLayout();
	axisCards->setSpacing(14);

	QGroupBox* cartBox = new QGroupBox("直角坐标");
	QVBoxLayout* cartBoxLayout = new QVBoxLayout(cartBox);
	QGridLayout* cartGrid = new QGridLayout();
	cartGrid->setHorizontalSpacing(10);
	cartGrid->setVerticalSpacing(10);
	AddAxisRow(cartGrid, 0, "X", JogMode::Cartesian, 0, "mm");
	AddAxisRow(cartGrid, 1, "Y", JogMode::Cartesian, 1, "mm");
	AddAxisRow(cartGrid, 2, "Z", JogMode::Cartesian, 2, "mm");
	AddAxisRow(cartGrid, 3, "RX", JogMode::Cartesian, 3, "deg");
	AddAxisRow(cartGrid, 4, "RY", JogMode::Cartesian, 4, "deg");
	AddAxisRow(cartGrid, 5, "RZ", JogMode::Cartesian, 5, "deg");
	QHBoxLayout* cartActionLayout = new QHBoxLayout();
	QPushButton* readCartButton = new QPushButton("读取当前位置");
	QPushButton* moveCartButton = new QPushButton("运动到指定位置");
	cartActionLayout->addWidget(readCartButton);
	cartActionLayout->addWidget(moveCartButton);
	cartBoxLayout->addLayout(cartGrid);
	cartBoxLayout->addLayout(cartActionLayout);
	connect(readCartButton, &QPushButton::clicked, this, &RobotJogDialog::ReadCurrentCartesianTarget);
	connect(moveCartButton, &QPushButton::clicked, this, &RobotJogDialog::MoveToCartesianTarget);

	QGroupBox* jointBox = new QGroupBox("关节坐标");
	QVBoxLayout* jointBoxLayout = new QVBoxLayout(jointBox);
	QGridLayout* jointGrid = new QGridLayout();
	jointGrid->setHorizontalSpacing(10);
	jointGrid->setVerticalSpacing(10);
	AddAxisRow(jointGrid, 0, "J1 / S", JogMode::Joint, 0, "pulse");
	AddAxisRow(jointGrid, 1, "J2 / L", JogMode::Joint, 1, "pulse");
	AddAxisRow(jointGrid, 2, "J3 / U", JogMode::Joint, 2, "pulse");
	AddAxisRow(jointGrid, 3, "J4 / R", JogMode::Joint, 3, "pulse");
	AddAxisRow(jointGrid, 4, "J5 / B", JogMode::Joint, 4, "pulse");
	AddAxisRow(jointGrid, 5, "J6 / T", JogMode::Joint, 5, "pulse");
	QHBoxLayout* jointActionLayout = new QHBoxLayout();
	QPushButton* readJointButton = new QPushButton("读取当前位置");
	QPushButton* moveJointButton = new QPushButton("运动到指定位置");
	jointActionLayout->addWidget(readJointButton);
	jointActionLayout->addWidget(moveJointButton);
	jointBoxLayout->addLayout(jointGrid);
	jointBoxLayout->addLayout(jointActionLayout);
	connect(readJointButton, &QPushButton::clicked, this, &RobotJogDialog::ReadCurrentJointTarget);
	connect(moveJointButton, &QPushButton::clicked, this, &RobotJogDialog::MoveToJointTarget);

	axisCards->addWidget(cartBox, 1);
	axisCards->addWidget(jointBox, 1);

	root->addWidget(title);
	root->addWidget(subtitle);
	root->addWidget(m_stateLabel);
	root->addLayout(speedLayout);
	root->addLayout(axisCards, 1);
}

void RobotJogDialog::ApplyStyle()
{
	setStyleSheet(
		"RobotJogDialog { background: #0e141b; }"
		"QLabel { color: #dce7f3; font-size: 13px; }"
		"QLabel#TitleLabel { color: #f6fbff; font-size: 24px; font-weight: 700; }"
		"QLabel#SubTitleLabel { color: #8fa4b8; font-size: 13px; }"
		"QLabel#StateCard { background: #151f2a; border: 1px solid #263546; border-radius: 14px; padding: 12px; color: #dce7f3; }"
		"QGroupBox { background: #121b25; border: 1px solid #263546; border-radius: 16px; margin-top: 12px; padding: 14px; color: #dce7f3; font-weight: 600; }"
		"QGroupBox::title { subcontrol-origin: margin; left: 14px; padding: 0 8px; color: #7cc7ff; }"
		"QLineEdit { background: #0b1118; border: 1px solid #2f4054; border-radius: 10px; padding: 8px 10px; color: #f5fbff; selection-background-color: #1e88e5; }"
		"QPushButton { background: #1d2a38; border: 1px solid #33475d; border-radius: 12px; color: #eef7ff; font-size: 15px; font-weight: 650; padding: 8px 14px; }"
		"QPushButton:hover { background: #26384a; border-color: #4e6c8d; }"
		"QPushButton:pressed { background: #2f9e6f; border-color: #5ee1a7; color: #07100c; }"
		"QPushButton#DangerButton { background: #3a1d24; border-color: #7a3442; color: #ffdce3; }"
		"QPushButton#DangerButton:hover { background: #512633; }"
	);
}

void RobotJogDialog::AddAxisRow(QGridLayout* layout, int row, const QString& axisName, JogMode mode, int axisIndex, const QString& unitText)
{
	QLabel* axisLabel = new QLabel(axisName);
	axisLabel->setMinimumWidth(72);
	QLabel* unitLabel = new QLabel(unitText);
	unitLabel->setMinimumWidth(42);
	QLineEdit* targetEdit = new QLineEdit();
	targetEdit->setMinimumWidth(90);
	targetEdit->setValidator(mode == JogMode::Cartesian
		? static_cast<QValidator*>(new QDoubleValidator(-999999.0, 999999.0, 3, targetEdit))
		: static_cast<QValidator*>(new QDoubleValidator(-999999999.0, 999999999.0, 0, targetEdit)));
	QPushButton* minusButton = CreateJogButton("-");
	QPushButton* plusButton = CreateJogButton("+");

	connect(minusButton, &QPushButton::clicked, this, [this, mode, axisIndex]() { StepJog(mode, axisIndex, -1); });
	connect(plusButton, &QPushButton::clicked, this, [this, mode, axisIndex]() { StepJog(mode, axisIndex, 1); });

	layout->addWidget(axisLabel, row, 0);
	layout->addWidget(targetEdit, row, 1);
	layout->addWidget(unitLabel, row, 2);
	layout->addWidget(minusButton, row, 3);
	layout->addWidget(plusButton, row, 4);
	layout->setColumnStretch(1, 1);

	if (mode == JogMode::Cartesian)
	{
		if (m_cartesianTargetEdits.size() <= axisIndex)
		{
			m_cartesianTargetEdits.resize(axisIndex + 1);
		}
		m_cartesianTargetEdits[axisIndex] = targetEdit;
	}
	else
	{
		if (m_jointTargetEdits.size() <= axisIndex)
		{
			m_jointTargetEdits.resize(axisIndex + 1);
		}
		m_jointTargetEdits[axisIndex] = targetEdit;
	}
}

void RobotJogDialog::LoadSpeedSettings()
{
	QSettings settings(JogSettingsPath(), QSettings::IniFormat);
	const double cartSpeed = settings.value("Speed/Cartesian", 60.0).toDouble();
	const double jointSpeed = settings.value("Speed/Joint", 1.0).toDouble();
	if (m_cartesianSpeedEdit != nullptr)
	{
		m_cartesianSpeedEdit->setText(QString::number(std::clamp(cartSpeed, 0.01, 9999.0), 'f', 3));
	}
	if (m_jointSpeedEdit != nullptr)
	{
		m_jointSpeedEdit->setText(QString::number(std::clamp(jointSpeed, 1.0, 100.0), 'f', 3));
	}
}

void RobotJogDialog::SaveSpeedSettings() const
{
	QSettings settings(JogSettingsPath(), QSettings::IniFormat);
	settings.setValue("Speed/Cartesian", CartesianSpeed());
	settings.setValue("Speed/Joint", JointSpeed());
}

void RobotJogDialog::ReadCurrentCartesianTarget()
{
	if (m_fanucDriver == nullptr)
	{
		return;
	}
	const T_ROBOT_COORS current = m_fanucDriver->GetCurrentPos();
	LogCartesianPoint(m_fanucDriver, "点动界面读取当前位置并保存到直角编辑框", current);
	SetCartesianTargetEditors(current);
}

void RobotJogDialog::ReadCurrentJointTarget()
{
	if (m_fanucDriver == nullptr)
	{
		return;
	}
	const T_ANGLE_PULSE current = m_fanucDriver->GetCurrentPulse();
	LogJointPoint(m_fanucDriver, "点动界面读取当前位置并保存到关节编辑框", current);
	SetJointTargetEditors(current);
}

void RobotJogDialog::MoveToCartesianTarget()
{
	if (m_fanucDriver == nullptr)
	{
		QMessageBox::warning(this, "点动控制", "FANUC驱动无效。");
		return;
	}
	StopJog();

	T_ROBOT_COORS target;
	QString error;
	if (!ReadCartesianTargetFromEditors(target, error))
	{
		QMessageBox::warning(this, "运动到指定位置", error);
		return;
	}

	const double robotSpeed = std::max(1.0, CartesianSpeed() / 60.0);
	LogCartesianPoint(m_fanucDriver, "点动界面从直角编辑框发送目标点", target);
	FANUCRobotCtrl* driver = m_fanucDriver;
	QPointer<RobotJogDialog> self(this);
	std::thread([self, driver, target, robotSpeed]()
		{
			const bool moveOk = driver->MoveByJob(target, T_ROBOT_MOVE_SPEED(robotSpeed, 0.0, 0.0), driver->m_nExternalAxleType, "MOVL");
			const int done = moveOk ? driver->CheckRobotDone(100) : -1;
			QMetaObject::invokeMethod(qApp, [self, moveOk, done]()
				{
					if (self == nullptr)
					{
						return;
					}
					if (!moveOk || done <= 0)
					{
						QMessageBox::warning(self, "运动到指定位置", QString("直角坐标运动失败，CheckRobotDone=%1").arg(done));
					}
				}, Qt::QueuedConnection);
		}).detach();
}

void RobotJogDialog::MoveToJointTarget()
{
	if (m_fanucDriver == nullptr)
	{
		QMessageBox::warning(this, "点动控制", "FANUC驱动无效。");
		return;
	}
	StopJog();

	T_ANGLE_PULSE target;
	QString error;
	if (!ReadJointTargetFromEditors(target, error))
	{
		QMessageBox::warning(this, "运动到指定位置", error);
		return;
	}

	const double robotSpeed = std::clamp(JointSpeed(), 1.0, 100.0);
	LogJointPoint(m_fanucDriver, "点动界面从关节编辑框发送目标点", target);
	FANUCRobotCtrl* driver = m_fanucDriver;
	QPointer<RobotJogDialog> self(this);
	std::thread([self, driver, target, robotSpeed]()
		{
			const bool moveOk = driver->MoveByJob(target, T_ROBOT_MOVE_SPEED(robotSpeed, 0.0, 0.0), driver->m_nExternalAxleType, "MOVJ");
			const int done = moveOk ? driver->CheckRobotDone(100) : -1;
			QMetaObject::invokeMethod(qApp, [self, moveOk, done]()
				{
					if (self == nullptr)
					{
						return;
					}
					if (!moveOk || done <= 0)
					{
						QMessageBox::warning(self, "运动到指定位置", QString("关节脉冲运动失败，CheckRobotDone=%1").arg(done));
					}
				}, Qt::QueuedConnection);
		}).detach();
}

void RobotJogDialog::StartJog(JogMode mode, int axisIndex, int direction)
{
	if (m_fanucDriver == nullptr)
	{
		QMessageBox::warning(this, "点动控制", "FANUC驱动无效。");
		return;
	}

	m_currentMode = mode;
	m_currentAxis = axisIndex;
	m_currentDirection = direction;
	m_jogActive = false;
	if (m_jogStartTimer != nullptr)
	{
		m_jogStartTimer->start();
	}
}

void RobotJogDialog::BeginJog()
{
	if (m_fanucDriver == nullptr || m_currentDirection == 0 || m_jogActive)
	{
		return;
	}

	const JogMode mode = m_currentMode;
	m_streamCartesianSpeed = CartesianSpeed();
	m_streamJointSpeed = JointSpeed();
	m_jogActive = true;
	m_nextStreamStep = 0;

	if (mode == JogMode::Cartesian)
	{
		m_streamBasePos = m_fanucDriver->GetCurrentPos();
		m_lastStreamPos = m_streamBasePos;
	}
	else
	{
		m_streamBasePulse = m_fanucDriver->GetCurrentPulse();
		m_lastStreamPulse = m_streamBasePulse;
	}

	const double robotSpeed = mode == JogMode::Cartesian
		? std::max(1.0, m_streamCartesianSpeed / 60.0)
		: std::clamp(m_streamJointSpeed, 1.0, 100.0);

	if (m_fanucDriver->m_pRobotLog != nullptr)
	{
		m_fanucDriver->m_pRobotLog->write(LogColor::DEFAULT,
			"点动界面长按开始: mode=%s axis=%d direction=%d speed=%.3f baseCart=(X=%.3f Y=%.3f Z=%.3f RX=%.3f RY=%.3f RZ=%.3f) baseJoint=(J1=%ld J2=%ld J3=%ld J4=%ld J5=%ld J6=%ld)",
			mode == JogMode::Cartesian ? "MOVL" : "MOVJ",
			m_currentAxis,
			m_currentDirection,
			robotSpeed,
			m_streamBasePos.dX, m_streamBasePos.dY, m_streamBasePos.dZ,
			m_streamBasePos.dRX, m_streamBasePos.dRY, m_streamBasePos.dRZ,
			m_streamBasePulse.nSPulse, m_streamBasePulse.nLPulse, m_streamBasePulse.nUPulse,
			m_streamBasePulse.nRPulse, m_streamBasePulse.nBPulse, m_streamBasePulse.nTPulse);
	}

	if (!m_fanucDriver->StartContinuousMoveQueue(mode == JogMode::Cartesian ? MOVL : MOVJ, robotSpeed))
	{
		m_jogActive = false;
		QMessageBox::warning(this, "点动控制", "启动连续运动队列失败。");
		return;
	}

	for (int i = 0; i < STREAM_START_POINT_COUNT; ++i)
	{
		if (mode == JogMode::Cartesian)
		{
			m_lastStreamPos = BuildCartesianStreamPoint(i);
			LogCartesianPoint(m_fanucDriver, GetStr("点动界面长按预装MOVL点[%d]", i).c_str(), m_lastStreamPos);
			m_fanucDriver->PushContinuousMovePoint(m_lastStreamPos, robotSpeed);
		}
		else
		{
			m_lastStreamPulse = BuildJointStreamPoint(i);
			LogJointPoint(m_fanucDriver, GetStr("点动界面长按预装MOVJ点[%d]", i).c_str(), m_lastStreamPulse);
			m_fanucDriver->PushContinuousMovePoint(m_lastStreamPulse, robotSpeed);
		}
	}

	m_nextStreamStep = STREAM_START_POINT_COUNT;
	m_jogTimer->start();
}

void RobotJogDialog::StepJog(JogMode mode, int axisIndex, int direction)
{
	if (m_fanucDriver == nullptr)
	{
		return;
	}

	if (mode == JogMode::Cartesian)
	{
		T_ROBOT_COORS target = m_fanucDriver->GetCurrentPos();
		const double stepDistance = CartesianSpeed() * STREAM_POINT_TIME_SEC / 60.0;
		AddCartesianDelta(target, axisIndex, static_cast<double>(direction) * stepDistance);
		const double robotSpeed = std::max(1.0, CartesianSpeed() / 60.0);
		LogCartesianPoint(m_fanucDriver, "点动界面单击生成直角目标点", target);
		m_fanucDriver->MoveByJob(target, T_ROBOT_MOVE_SPEED(robotSpeed, 0.0, 0.0), m_fanucDriver->m_nExternalAxleType, "MOVL");
		SetCartesianTargetEditors(target);
		return;
	}

	T_ANGLE_PULSE target = m_fanucDriver->GetCurrentPulse();
	const double stepDeg = std::max(0.25, JointSpeed()) * STREAM_POINT_TIME_SEC;
	const double pulseUnit = AxisPulseUnit(m_fanucDriver, axisIndex);
	const long deltaPulse = pulseUnit == 0.0 ? 0 : static_cast<long>(std::lround(static_cast<double>(direction) * stepDeg / pulseUnit));
	AddJointDelta(target, axisIndex, deltaPulse);
	const double robotSpeed = std::clamp(JointSpeed(), 1.0, 100.0);
	LogJointPoint(m_fanucDriver, "点动界面单击生成关节目标点", target);
	m_fanucDriver->MoveByJob(target, T_ROBOT_MOVE_SPEED(robotSpeed, 0.0, 0.0), m_fanucDriver->m_nExternalAxleType, "MOVJ");
	SetJointTargetEditors(target);
}

void RobotJogDialog::StopJog()
{
	const bool wasActive = m_jogActive;
	m_jogActive = false;
	m_currentDirection = 0;
	if (m_jogStartTimer != nullptr)
	{
		m_jogStartTimer->stop();
	}
	m_jogTimer->stop();
	if (wasActive && m_fanucDriver != nullptr)
	{
		m_fanucDriver->RequestEndContinuousMoveQueue();
	}
}

void RobotJogDialog::FeedNextPoint()
{
	if (!m_jogActive || m_fanucDriver == nullptr)
	{
		return;
	}
	const JogMode mode = m_currentMode;
	const int stepIndex = m_nextStreamStep;
	const T_ROBOT_COORS cartTarget = BuildCartesianStreamPoint(stepIndex);
	const T_ANGLE_PULSE jointTarget = BuildJointStreamPoint(stepIndex);
	const double robotSpeed = mode == JogMode::Cartesian
		? std::max(1.0, m_streamCartesianSpeed / 60.0)
		: std::clamp(m_streamJointSpeed, 1.0, 100.0);
	if (mode == JogMode::Cartesian)
	{
		m_lastStreamPos = cartTarget;
		LogCartesianPoint(m_fanucDriver, GetStr("点动界面长按追加MOVL点[%d]", stepIndex).c_str(), cartTarget);
		m_fanucDriver->PushContinuousMovePoint(cartTarget, robotSpeed);
	}
	else
	{
		m_lastStreamPulse = jointTarget;
		LogJointPoint(m_fanucDriver, GetStr("点动界面长按追加MOVJ点[%d]", stepIndex).c_str(), jointTarget);
		m_fanucDriver->PushContinuousMovePoint(jointTarget, robotSpeed);
	}
	++m_nextStreamStep;
}

void RobotJogDialog::RefreshStateText()
{
	if (m_stateLabel == nullptr || m_fanucDriver == nullptr)
	{
		return;
	}

	long long robotMs = 0;
	long long pcRecvMs = 0;
	const T_ROBOT_COORS pos = m_fanucDriver->GetCurrentPosPassive(&robotMs, &pcRecvMs);
	const T_ANGLE_PULSE pulse = m_fanucDriver->GetCurrentPulsePassive();
	const int done = m_fanucDriver->CheckDonePassive();
	const QString doneText = done == 0 ? "运行中" : (done == 1 ? "停止/完成" : QString("未知(%1)").arg(done));

	m_stateLabel->setText(QString(
		"状态: %1    robot_ms=%2    pc_recv_ms=%3\n"
		"直角: X=%4  Y=%5  Z=%6  RX=%7  RY=%8  RZ=%9\n"
		"关节: J1=%10  J2=%11  J3=%12  J4=%13  J5=%14  J6=%15")
		.arg(doneText)
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
		.arg(pulse.nTPulse));
}

double RobotJogDialog::CartesianSpeed() const
{
	const double value = m_cartesianSpeedEdit == nullptr ? 60.0 : m_cartesianSpeedEdit->text().toDouble();
	return std::clamp(value, 0.01, 9999.0);
}

double RobotJogDialog::JointSpeed() const
{
	const double value = m_jointSpeedEdit == nullptr ? 5.0 : m_jointSpeedEdit->text().toDouble();
	return std::clamp(value, 1.0, 100.0);
}

bool RobotJogDialog::ReadCartesianTargetFromEditors(T_ROBOT_COORS& target, QString& error) const
{
	if (m_cartesianTargetEdits.size() < 6)
	{
		error = "直角坐标目标输入框不完整。";
		return false;
	}

	double values[6] = { 0 };
	for (int i = 0; i < 6; ++i)
	{
		QLineEdit* edit = m_cartesianTargetEdits[i];
		if (edit == nullptr)
		{
			error = QString("直角坐标第%1轴输入框无效。").arg(i + 1);
			return false;
		}
		bool ok = false;
		values[i] = edit->text().trimmed().toDouble(&ok);
		if (!ok)
		{
			error = QString("直角坐标第%1轴目标值不是数字。").arg(i + 1);
			return false;
		}
	}

	target = m_fanucDriver == nullptr ? T_ROBOT_COORS() : m_fanucDriver->GetCurrentPosPassive();
	target.dX = values[0];
	target.dY = values[1];
	target.dZ = values[2];
	target.dRX = values[3];
	target.dRY = values[4];
	target.dRZ = values[5];
	return true;
}

bool RobotJogDialog::ReadJointTargetFromEditors(T_ANGLE_PULSE& target, QString& error) const
{
	if (m_jointTargetEdits.size() < 6)
	{
		error = "关节脉冲目标输入框不完整。";
		return false;
	}

	long values[6] = { 0 };
	for (int i = 0; i < 6; ++i)
	{
		QLineEdit* edit = m_jointTargetEdits[i];
		if (edit == nullptr)
		{
			error = QString("关节第%1轴输入框无效。").arg(i + 1);
			return false;
		}
		bool ok = false;
		values[i] = edit->text().trimmed().toLong(&ok);
		if (!ok)
		{
			error = QString("关节第%1轴目标脉冲不是整数。").arg(i + 1);
			return false;
		}
	}

	target = m_fanucDriver == nullptr ? T_ANGLE_PULSE() : m_fanucDriver->GetCurrentPulsePassive();
	target.nSPulse = values[0];
	target.nLPulse = values[1];
	target.nUPulse = values[2];
	target.nRPulse = values[3];
	target.nBPulse = values[4];
	target.nTPulse = values[5];
	return true;
}

void RobotJogDialog::SetCartesianTargetEditors(const T_ROBOT_COORS& target)
{
	const double values[6] = { target.dX, target.dY, target.dZ, target.dRX, target.dRY, target.dRZ };
	for (int i = 0; i < m_cartesianTargetEdits.size() && i < 6; ++i)
	{
		if (m_cartesianTargetEdits[i] != nullptr)
		{
			m_cartesianTargetEdits[i]->setText(QString::number(values[i], 'f', 3));
		}
	}
}

void RobotJogDialog::SetJointTargetEditors(const T_ANGLE_PULSE& target)
{
	const long values[6] = { target.nSPulse, target.nLPulse, target.nUPulse, target.nRPulse, target.nBPulse, target.nTPulse };
	for (int i = 0; i < m_jointTargetEdits.size() && i < 6; ++i)
	{
		if (m_jointTargetEdits[i] != nullptr)
		{
			m_jointTargetEdits[i]->setText(QString::number(values[i]));
		}
	}
}

T_ROBOT_COORS RobotJogDialog::BuildCartesianStreamPoint(int stepIndex) const
{
	T_ROBOT_COORS target = m_streamBasePos;
	const double speedPerMinute = m_streamCartesianSpeed;
	const double delta = static_cast<double>(m_currentDirection) * speedPerMinute * STREAM_POINT_TIME_SEC * static_cast<double>(stepIndex + 1) / 60.0;
	AddCartesianDelta(target, m_currentAxis, delta);
	return target;
}

T_ANGLE_PULSE RobotJogDialog::BuildJointStreamPoint(int stepIndex) const
{
	T_ANGLE_PULSE target = m_streamBasePulse;
	const double speedPercent = m_streamJointSpeed;
	const double degPerSecond = std::max(0.25, speedPercent);
	const double deltaDeg = static_cast<double>(m_currentDirection) * degPerSecond * STREAM_POINT_TIME_SEC * static_cast<double>(stepIndex + 1);
	const double pulseUnit = AxisPulseUnit(m_fanucDriver, m_currentAxis);
	const long deltaPulse = pulseUnit == 0.0 ? 0 : static_cast<long>(std::lround(deltaDeg / pulseUnit));
	AddJointDelta(target, m_currentAxis, deltaPulse);
	return target;
}
