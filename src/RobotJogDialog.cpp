#include "RobotJogDialog.h"

#include "ConfigDatabase.h"
#include "RobotDriverAdaptor.h"
#include "RobotMessage.h"
#include "RobotMotionTimeoutPolicy.h"
#include "RobotOperationLease.h"
#include "WindowStyleHelper.h"

#include <QApplication>
#include <QCloseEvent>
#include <QDoubleValidator>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLayout>
#include <QMessageBox>
#include <QMetaObject>
#include <QPointer>
#include <QScrollArea>
#include <QSizePolicy>
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

	double AxisPulseUnit(const RobotDriverAdaptor* driver, int axisIndex)
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

	constexpr auto JOG_SETTINGS_MODULE = "RobotJog";
	constexpr int JOG_CAPABILITY_PASSIVE = 0;
	constexpr int JOG_CAPABILITY_CARTESIAN = 1;
	constexpr int JOG_CAPABILITY_JOINT = 2;

	bool RequireJogCapabilities(
		RobotDriverAdaptor* driver,
		std::initializer_list<RobotDriverCapability> capabilities,
		const QString& action,
		QString& error)
	{
		if (driver == nullptr)
		{
			error = QStringLiteral("机器人驱动无效。");
			return false;
		}
		if (driver->SupportsAll(capabilities))
		{
			return true;
		}
		error = QStringLiteral("当前机器人品牌底层无法执行“%1”，缺少适配能力：%2；功能已限制。")
			.arg(action, QString::fromUtf8(driver->MissingCapabilitiesText(capabilities).c_str()));
		return false;
	}

	QString MotionFailureText(RobotDriverAdaptor* driver, const QString& action, int done)
	{
		QString text = QString("%1失败，CheckRobotDone=%2").arg(action).arg(done);
		if (driver == nullptr)
		{
			return text;
		}

		const std::string lastError = driver->GetLastRobotError();
		if (!lastError.empty())
		{
			text += "\n最近错误：" + DecodeRobotMessageText(lastError);
		}

		const std::string statusText = driver->GetRobotStatusText();
		if (!statusText.empty() && statusText != lastError)
		{
			text += "\n当前状态：" + DecodeRobotMessageText(statusText);
		}
		return text;
	}

	void LogCartesianPoint(RobotDriverAdaptor* driver, const char* prefix, const T_ROBOT_COORS& pos)
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

	void LogJointPoint(RobotDriverAdaptor* driver, const char* prefix, const T_ANGLE_PULSE& pulse)
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

RobotJogDialog::RobotJogDialog(RobotDriverAdaptor* robotDriver, QWidget* parent)
	: QDialog(parent)
	, m_robotDriver(robotDriver)
	, m_jogStartTimer(new QTimer(this))
	, m_jogTimer(new QTimer(this))
	, m_stateTimer(new QTimer(this))
	, m_stateLabel(nullptr)
	, m_cartesianSpeedEdit(nullptr)
	, m_jointSpeedEdit(nullptr)
	, m_stopButton(nullptr)
	, m_motionTaskRunning(false)
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
	setMinimumSize(720, 500);
	ResizeWindowForAvailableGeometry(this, QSize(1120, 680), 0.86, 0.78);
	BuildUi();
	ApplyStyle();
	LoadSpeedSettings();
	UpdateMotionButtonState();

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
	if (m_robotDriver != nullptr && m_robotDriver->Supports(RobotDriverCapability::ContinuousJog))
	{
		m_robotDriver->EndContinuousJog();
		if (m_robotDriver->IsContinuousJogRunning())
		{
			// worker 已退出但机器人任务终态未确认时，析构前主动走真实 ABORT；
			// 失败则 pending 随 lease 析构转为 sticky，绝不静默释放。
			RobotOperationLease::StopAndConfirmUnverifiedMotion(m_robotDriver);
		}
	}
	m_jogOperationLease.reset();
	SaveSpeedSettings();
}

void RobotJogDialog::closeEvent(QCloseEvent* event)
{
	// 关闭/返回主页时必须立即停止继续喂点。窗口默认仅隐藏，后台目标运动仍可由
	// 主页的 STOP/MSTOP 处理；连续点动租约会保留到机器人侧真实停机确认。
	StopJog();
	QDialog::closeEvent(event);
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
	readCartButton->setProperty("robotCapabilityMode", JOG_CAPABILITY_PASSIVE);
	moveCartButton->setProperty("robotCapabilityMode", JOG_CAPABILITY_CARTESIAN);
	cartActionLayout->addWidget(readCartButton);
	cartActionLayout->addWidget(moveCartButton);
	m_motionButtons.push_back(readCartButton);
	m_motionButtons.push_back(moveCartButton);
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
	readJointButton->setProperty("robotCapabilityMode", JOG_CAPABILITY_PASSIVE);
	moveJointButton->setProperty("robotCapabilityMode", JOG_CAPABILITY_JOINT);
	jointActionLayout->addWidget(readJointButton);
	jointActionLayout->addWidget(moveJointButton);
	m_motionButtons.push_back(readJointButton);
	m_motionButtons.push_back(moveJointButton);
	jointBoxLayout->addLayout(jointGrid);
	jointBoxLayout->addLayout(jointActionLayout);
	connect(readJointButton, &QPushButton::clicked, this, &RobotJogDialog::ReadCurrentJointTarget);
	connect(moveJointButton, &QPushButton::clicked, this, &RobotJogDialog::MoveToJointTarget);

	axisCards->addWidget(cartBox, 1);
	axisCards->addWidget(jointBox, 1);

	root->addWidget(title);
	root->addWidget(subtitle);

	QScrollArea* jogScrollArea = new QScrollArea(this);
	jogScrollArea->setObjectName("AdaptiveWindowScrollArea");
	ConfigureResponsiveScrollArea(jogScrollArea);

	QWidget* jogContent = new QWidget(jogScrollArea);
	jogContent->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
	QVBoxLayout* jogContentLayout = new QVBoxLayout(jogContent);
	jogContentLayout->setContentsMargins(0, 0, 8, 0);
	jogContentLayout->setSpacing(14);
	jogContentLayout->setSizeConstraint(QLayout::SetMinAndMaxSize);
	jogContentLayout->addWidget(m_stateLabel);
	jogContentLayout->addLayout(speedLayout);
	jogContentLayout->addLayout(axisCards, 1);
	jogContentLayout->addStretch(1);
	jogScrollArea->setWidget(jogContent);
	root->addWidget(jogScrollArea, 1);
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
		"QPushButton:disabled { background: #18212b; border-color: #253340; color: #6f7f8d; }"
		"QPushButton#DangerButton { background: #3a1d24; border-color: #7a3442; color: #ffdce3; }"
		"QPushButton#DangerButton:hover { background: #512633; }"
		"QPushButton#DangerButton:disabled { background: #2a2125; border-color: #4a373d; color: #8f757d; }"
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
	const int capabilityMode = mode == JogMode::Cartesian
		? JOG_CAPABILITY_CARTESIAN : JOG_CAPABILITY_JOINT;
	minusButton->setProperty("robotCapabilityMode", capabilityMode);
	plusButton->setProperty("robotCapabilityMode", capabilityMode);

	connect(minusButton, &QPushButton::clicked, this, [this, mode, axisIndex]() { StepJog(mode, axisIndex, -1); });
	connect(plusButton, &QPushButton::clicked, this, [this, mode, axisIndex]() { StepJog(mode, axisIndex, 1); });
	m_motionButtons.push_back(minusButton);
	m_motionButtons.push_back(plusButton);

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
	QString cartSpeedText;
	QString jointSpeedText;
	const double cartSpeed = ConfigDatabase::ReadScopedSetting(
		QStringLiteral("global"), QString(), JOG_SETTINGS_MODULE, "Speed/Cartesian", &cartSpeedText)
		? cartSpeedText.toDouble()
		: 60.0;
	const double jointSpeed = ConfigDatabase::ReadScopedSetting(
		QStringLiteral("global"), QString(), JOG_SETTINGS_MODULE, "Speed/Joint", &jointSpeedText)
		? jointSpeedText.toDouble()
		: 1.0;
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
	ConfigDatabase::WriteScopedSetting(
		QStringLiteral("global"),
		QString(),
		JOG_SETTINGS_MODULE,
		"Speed/Cartesian",
		QString::number(CartesianSpeed(), 'f', 6),
		QStringLiteral("number"));
	ConfigDatabase::WriteScopedSetting(
		QStringLiteral("global"),
		QString(),
		JOG_SETTINGS_MODULE,
		"Speed/Joint",
		QString::number(JointSpeed(), 'f', 6),
		QStringLiteral("number"));
}

void RobotJogDialog::ShowMessageOnUiThread(QMessageBox::Icon icon, const QString& title, const QString& text)
{
	QPointer<RobotJogDialog> self(this);
	QMetaObject::invokeMethod(this, [self, icon, title, text]()
		{
			if (self == nullptr)
			{
				return;
			}
			QMessageBox* msgBox = new QMessageBox(
				icon, title, text, QMessageBox::Ok, self);
			msgBox->setAttribute(Qt::WA_DeleteOnClose);
			msgBox->setWindowModality(Qt::NonModal);
			msgBox->show();
		}, Qt::QueuedConnection);
}

void RobotJogDialog::ReadCurrentCartesianTarget()
{
	QString capabilityError;
	if (!RequireJogCapabilities(
		m_robotDriver,
		{ RobotDriverCapability::PassiveState },
		QStringLiteral("读取当前位置"),
		capabilityError))
	{
		ShowMessageOnUiThread(QMessageBox::Warning, "点动控制", capabilityError);
		return;
	}
	T_ROBOT_COORS current;
	if (!m_robotDriver->TryGetCurrentPos(current))
	{
		ShowMessageOnUiThread(QMessageBox::Warning, "点动控制",
			"读取当前位置失败，未改写直角目标框："
			+ DecodeRobotMessageText(m_robotDriver->GetLastRobotError()));
		return;
	}
	LogCartesianPoint(m_robotDriver, "点动界面读取当前位置并保存到直角编辑框", current);
	SetCartesianTargetEditors(current);
}

void RobotJogDialog::ReadCurrentJointTarget()
{
	QString capabilityError;
	if (!RequireJogCapabilities(
		m_robotDriver,
		{ RobotDriverCapability::PassiveState },
		QStringLiteral("读取当前关节"),
		capabilityError))
	{
		ShowMessageOnUiThread(QMessageBox::Warning, "点动控制", capabilityError);
		return;
	}
	T_ANGLE_PULSE current;
	if (!m_robotDriver->TryGetCurrentPulse(current))
	{
		ShowMessageOnUiThread(QMessageBox::Warning, "点动控制",
			"读取当前关节失败，未改写关节目标框："
			+ DecodeRobotMessageText(m_robotDriver->GetLastRobotError()));
		return;
	}
	LogJointPoint(m_robotDriver, "点动界面读取当前位置并保存到关节编辑框", current);
	SetJointTargetEditors(current);
}

void RobotJogDialog::MoveToCartesianTarget()
{
	QString capabilityError;
	if (!RequireJogCapabilities(
		m_robotDriver,
		{ RobotDriverCapability::LinearMotion,
		  RobotDriverCapability::PassiveState,
		  RobotDriverCapability::VerifiedProgramCompletion,
		  RobotDriverCapability::VerifiedSafeAbort },
		QStringLiteral("直角目标运动"),
		capabilityError))
	{
		ShowMessageOnUiThread(QMessageBox::Warning, "点动控制", capabilityError);
		return;
	}
	if (IsMotionBusy())
	{
		return;
	}
	StopJog();

	T_ROBOT_COORS target;
	QString error;
	if (!ReadCartesianTargetFromEditors(target, error))
	{
		ShowMessageOnUiThread(QMessageBox::Warning, "运动到指定位置", error);
		return;
	}

	T_ROBOT_COORS current;
	if (!m_robotDriver->TryGetCurrentPos(current))
	{
		ShowMessageOnUiThread(
			QMessageBox::Warning,
			"运动到指定位置",
			"读取当前机器人位置失败，已拒绝启动运动："
			+ DecodeRobotMessageText(m_robotDriver->GetLastRobotError()));
		return;
	}
	const double configuredSpeedMmPerMin = CartesianSpeed();
	std::string speedError;
	if (!m_robotDriver->ValidateLinearSpeedMmPerMin(configuredSpeedMmPerMin, &speedError))
	{
		ShowMessageOnUiThread(
			QMessageBox::Warning,
			"运动到指定位置",
			DecodeRobotMessageText(speedError));
		return;
	}
	int motionTimeoutMs = 0;
	std::string admissionError;
	if (!RobotMotionTimeoutPolicy::AdmitCartesianMove(
		current, target, configuredSpeedMmPerMin, motionTimeoutMs, &admissionError))
	{
		ShowMessageOnUiThread(
			QMessageBox::Warning,
			"运动到指定位置",
			QString::fromUtf8(admissionError.c_str()));
		return;
	}
	QString leaseError;
	const auto operationLease = RobotOperationLease::TryAcquire(
		m_robotDriver, QStringLiteral("点动界面直角目标运动"), &leaseError);
	if (!operationLease)
	{
		ShowMessageOnUiThread(QMessageBox::Warning, "运动到指定位置", leaseError);
		return;
	}
	LogCartesianPoint(m_robotDriver, "点动界面从直角编辑框发送目标点", target);
	RobotDriverAdaptor* driver = m_robotDriver;
	QPointer<RobotJogDialog> self(this);
	SetMotionTaskRunning(true);
		std::thread([self, driver, target, configuredSpeedMmPerMin, motionTimeoutMs, operationLease]()
		{
			if (self == nullptr)
			{
				return;
			}
			const bool moveOk = driver->MoveLinearMmPerMin(
				target, configuredSpeedMmPerMin, driver->m_nExternalAxleType);
			const int done = moveOk ? driver->CheckRobotDone(100, motionTimeoutMs) : -1;
			const QString failureText = (!moveOk || done <= 0) ? MotionFailureText(driver, "直角坐标运动", done) : QString();
			QMetaObject::invokeMethod(self.data(), [self, moveOk, done, failureText]()
				{
					if (self == nullptr)
					{
						return;
					}
					self->SetMotionTaskRunning(false);
					if (!moveOk || done <= 0)
					{
						self->ShowMessageOnUiThread(QMessageBox::Warning, "运动到指定位置", failureText);
					}
				}, Qt::QueuedConnection);
		}).detach();
}

void RobotJogDialog::MoveToJointTarget()
{
	QString capabilityError;
	if (!RequireJogCapabilities(
		m_robotDriver,
		{ RobotDriverCapability::JointMotion,
		  RobotDriverCapability::PassiveState,
		  RobotDriverCapability::VerifiedProgramCompletion,
		  RobotDriverCapability::VerifiedSafeAbort },
		QStringLiteral("关节目标运动"),
		capabilityError))
	{
		ShowMessageOnUiThread(QMessageBox::Warning, "点动控制", capabilityError);
		return;
	}
	if (IsMotionBusy())
	{
		return;
	}
	StopJog();

	T_ANGLE_PULSE target;
	QString error;
	if (!ReadJointTargetFromEditors(target, error))
	{
		ShowMessageOnUiThread(QMessageBox::Warning, "运动到指定位置", error);
		return;
	}

	const double robotSpeed = std::clamp(JointSpeed(), 1.0, 100.0);
	QString leaseError;
	const auto operationLease = RobotOperationLease::TryAcquire(
		m_robotDriver, QStringLiteral("点动界面关节目标运动"), &leaseError);
	if (!operationLease)
	{
		ShowMessageOnUiThread(QMessageBox::Warning, "运动到指定位置", leaseError);
		return;
	}
	LogJointPoint(m_robotDriver, "点动界面从关节编辑框发送目标点", target);
	RobotDriverAdaptor* driver = m_robotDriver;
	QPointer<RobotJogDialog> self(this);
	SetMotionTaskRunning(true);
		std::thread([self, driver, target, robotSpeed, operationLease]()
		{
			if (self == nullptr)
			{
				return;
			}
			const bool moveOk = driver->MoveJointPercent(target, robotSpeed, driver->m_nExternalAxleType);
			const int done = moveOk ? driver->CheckRobotDone(100, 1800000) : -1;
			const QString failureText = (!moveOk || done <= 0) ? MotionFailureText(driver, "关节脉冲运动", done) : QString();
			QMetaObject::invokeMethod(self.data(), [self, moveOk, done, failureText]()
				{
					if (self == nullptr)
					{
						return;
					}
					self->SetMotionTaskRunning(false);
					if (!moveOk || done <= 0)
					{
						self->ShowMessageOnUiThread(QMessageBox::Warning, "运动到指定位置", failureText);
					}
				}, Qt::QueuedConnection);
		}).detach();
}

void RobotJogDialog::StartJog(JogMode mode, int axisIndex, int direction)
{
	if (m_robotDriver == nullptr || IsMotionBusy())
	{
		if (m_robotDriver == nullptr)
		{
			ShowMessageOnUiThread(QMessageBox::Warning, "点动控制", "机器人驱动无效。");
		}
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
	if (m_robotDriver == nullptr || m_currentDirection == 0 || m_jogActive)
	{
		return;
	}

	const JogMode mode = m_currentMode;
	QString capabilityError;
	if (!RequireJogCapabilities(
		m_robotDriver,
		{ RobotDriverCapability::ContinuousJog,
		  RobotDriverCapability::PassiveState,
		  RobotDriverCapability::VerifiedSafeAbort },
		QStringLiteral("长按连续点动"),
		capabilityError))
	{
		ShowMessageOnUiThread(QMessageBox::Information, "点动控制", capabilityError);
		return;
	}
	QString leaseError;
	m_jogOperationLease = RobotOperationLease::TryAcquire(
		m_robotDriver, QStringLiteral("长按连续点动"), &leaseError);
	if (!m_jogOperationLease)
	{
		ShowMessageOnUiThread(QMessageBox::Warning, "点动控制", leaseError);
		return;
	}
	m_streamCartesianSpeed = CartesianSpeed();
	m_streamJointSpeed = JointSpeed();

	if (mode == JogMode::Cartesian)
	{
		if (!m_robotDriver->TryGetCurrentPos(m_streamBasePos))
		{
			m_jogOperationLease.reset();
			ShowMessageOnUiThread(QMessageBox::Warning, "点动控制",
				"长按点动已拒绝：读取当前直角位置失败，"
				+ DecodeRobotMessageText(m_robotDriver->GetLastRobotError()));
			return;
		}
		m_lastStreamPos = m_streamBasePos;
	}
	else
	{
		if (!m_robotDriver->TryGetCurrentPulse(m_streamBasePulse))
		{
			m_jogOperationLease.reset();
			ShowMessageOnUiThread(QMessageBox::Warning, "点动控制",
				"长按点动已拒绝：读取当前关节位置失败，"
				+ DecodeRobotMessageText(m_robotDriver->GetLastRobotError()));
			return;
		}
		m_lastStreamPulse = m_streamBasePulse;
	}
	m_jogActive = true;
	m_nextStreamStep = 0;
	UpdateMotionButtonState();

	const double robotSpeed = mode == JogMode::Cartesian
		? m_streamCartesianSpeed
		: std::clamp(m_streamJointSpeed, 1.0, 100.0);

	if (m_robotDriver->m_pRobotLog != nullptr)
	{
		m_robotDriver->m_pRobotLog->write(LogColor::DEFAULT,
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

	if (!m_robotDriver->StartContinuousJog(mode == JogMode::Cartesian ? MOVL : MOVJ, robotSpeed))
	{
		m_jogActive = false;
		m_jogOperationLease.reset();
		UpdateMotionButtonState();
		if (m_stateLabel != nullptr)
		{
			m_stateLabel->setText(QStringLiteral(
				"启动连续运动队列失败；若安全停机未确认，请使用主页固定红色停止按钮重试。"));
		}
		return;
	}

	for (int i = 0; i < STREAM_START_POINT_COUNT; ++i)
	{
		if (mode == JogMode::Cartesian)
		{
			m_lastStreamPos = BuildCartesianStreamPoint(i);
			LogCartesianPoint(m_robotDriver, GetStr("点动界面长按预装MOVL点[%d]", i).c_str(), m_lastStreamPos);
			m_robotDriver->PushContinuousJogPoint(m_lastStreamPos, robotSpeed);
		}
		else
		{
			m_lastStreamPulse = BuildJointStreamPoint(i);
			LogJointPoint(m_robotDriver, GetStr("点动界面长按预装MOVJ点[%d]", i).c_str(), m_lastStreamPulse);
			m_robotDriver->PushContinuousJogPoint(m_lastStreamPulse, robotSpeed);
		}
	}

	m_nextStreamStep = STREAM_START_POINT_COUNT;
	m_jogTimer->start();
}

void RobotJogDialog::StepJog(JogMode mode, int axisIndex, int direction)
{
	QString capabilityError;
	const bool capabilityReady = mode == JogMode::Cartesian
		? RequireJogCapabilities(
			m_robotDriver,
			{ RobotDriverCapability::LinearMotion,
			  RobotDriverCapability::PassiveState,
			  RobotDriverCapability::VerifiedProgramCompletion,
			  RobotDriverCapability::VerifiedSafeAbort },
			QStringLiteral("直角步进"), capabilityError)
		: RequireJogCapabilities(
			m_robotDriver,
			{ RobotDriverCapability::JointMotion,
			  RobotDriverCapability::PassiveState,
			  RobotDriverCapability::VerifiedProgramCompletion,
			  RobotDriverCapability::VerifiedSafeAbort },
			QStringLiteral("关节步进"), capabilityError);
	if (!capabilityReady)
	{
		ShowMessageOnUiThread(QMessageBox::Warning, "点动控制", capabilityError);
		return;
	}
	if (IsMotionBusy())
	{
		return;
	}

	if (mode == JogMode::Cartesian)
	{
		QString leaseError;
		const auto operationLease = RobotOperationLease::TryAcquire(
			m_robotDriver, QStringLiteral("点动界面直角步进"), &leaseError);
		if (!operationLease)
		{
			ShowMessageOnUiThread(QMessageBox::Warning, "点动控制", leaseError);
			return;
		}
		T_ROBOT_COORS target;
		if (!m_robotDriver->TryGetCurrentPos(target))
		{
			ShowMessageOnUiThread(QMessageBox::Warning, "点动控制",
				"直角步进已拒绝：读取当前位置失败，"
				+ DecodeRobotMessageText(m_robotDriver->GetLastRobotError()));
			return;
		}
		const double stepDistance = CartesianSpeed() * STREAM_POINT_TIME_SEC / 60.0;
		AddCartesianDelta(target, axisIndex, static_cast<double>(direction) * stepDistance);
		const double robotSpeed = CartesianSpeed();
		LogCartesianPoint(m_robotDriver, "点动界面单击生成直角目标点", target);
		SetMotionTaskRunning(true);
		RobotDriverAdaptor* driver = m_robotDriver;
		QPointer<RobotJogDialog> self(this);
		std::thread([self, driver, target, robotSpeed, operationLease]()
			{
				const bool moveOk = driver->MoveLinearMmPerMin(target, robotSpeed, driver->m_nExternalAxleType);
				const int done = moveOk ? driver->CheckRobotDone(100, 1800000) : -1;
				const QString failureText = (!moveOk || done <= 0) ? MotionFailureText(driver, "直角步进", done) : QString();
				QMetaObject::invokeMethod(qApp, [self, moveOk, done, failureText]()
					{
						if (self == nullptr)
						{
							return;
						}
						self->SetMotionTaskRunning(false);
						if (!moveOk || done <= 0)
						{
							self->ShowMessageOnUiThread(
								QMessageBox::Warning, "点动控制", failureText);
						}
					}, Qt::QueuedConnection);
			}).detach();
		SetCartesianTargetEditors(target);
		return;
	}

	QString leaseError;
	const auto operationLease = RobotOperationLease::TryAcquire(
		m_robotDriver, QStringLiteral("点动界面关节步进"), &leaseError);
	if (!operationLease)
	{
		ShowMessageOnUiThread(QMessageBox::Warning, "点动控制", leaseError);
		return;
	}
	T_ANGLE_PULSE target;
	if (!m_robotDriver->TryGetCurrentPulse(target))
	{
		ShowMessageOnUiThread(QMessageBox::Warning, "点动控制",
			"关节步进已拒绝：读取当前位置失败，"
			+ DecodeRobotMessageText(m_robotDriver->GetLastRobotError()));
		return;
	}
	const double stepDeg = std::max(0.25, JointSpeed()) * STREAM_POINT_TIME_SEC;
	const double pulseUnit = AxisPulseUnit(m_robotDriver, axisIndex);
	const long deltaPulse = pulseUnit == 0.0 ? 0 : static_cast<long>(std::lround(static_cast<double>(direction) * stepDeg / pulseUnit));
	AddJointDelta(target, axisIndex, deltaPulse);
	const double robotSpeed = std::clamp(JointSpeed(), 1.0, 100.0);
	LogJointPoint(m_robotDriver, "点动界面单击生成关节目标点", target);
	SetMotionTaskRunning(true);
	RobotDriverAdaptor* driver = m_robotDriver;
	QPointer<RobotJogDialog> self(this);
	std::thread([self, driver, target, robotSpeed, operationLease]()
		{
			const bool moveOk = driver->MoveJointPercent(target, robotSpeed, driver->m_nExternalAxleType);
			const int done = moveOk ? driver->CheckRobotDone(100, 1800000) : -1;
			const QString failureText = (!moveOk || done <= 0) ? MotionFailureText(driver, "关节步进", done) : QString();
			QMetaObject::invokeMethod(qApp, [self, moveOk, done, failureText]()
				{
					if (self == nullptr)
					{
						return;
					}
					self->SetMotionTaskRunning(false);
					if (!moveOk || done <= 0)
					{
						self->ShowMessageOnUiThread(
							QMessageBox::Warning, "点动控制", failureText);
					}
				}, Qt::QueuedConnection);
		}).detach();
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
	if (wasActive && m_robotDriver != nullptr)
	{
		m_robotDriver->RequestEndContinuousJog();
	}
	UpdateMotionButtonState();
}

void RobotJogDialog::FeedNextPoint()
{
	if (!m_jogActive || m_robotDriver == nullptr)
	{
		return;
	}
	const JogMode mode = m_currentMode;
	const int stepIndex = m_nextStreamStep;
	const T_ROBOT_COORS cartTarget = BuildCartesianStreamPoint(stepIndex);
	const T_ANGLE_PULSE jointTarget = BuildJointStreamPoint(stepIndex);
	const double robotSpeed = mode == JogMode::Cartesian
		? m_streamCartesianSpeed
		: std::clamp(m_streamJointSpeed, 1.0, 100.0);
	if (mode == JogMode::Cartesian)
	{
		m_lastStreamPos = cartTarget;
		LogCartesianPoint(m_robotDriver, GetStr("点动界面长按追加MOVL点[%d]", stepIndex).c_str(), cartTarget);
		m_robotDriver->PushContinuousJogPoint(cartTarget, robotSpeed);
	}
	else
	{
		m_lastStreamPulse = jointTarget;
		LogJointPoint(m_robotDriver, GetStr("点动界面长按追加MOVJ点[%d]", stepIndex).c_str(), jointTarget);
		m_robotDriver->PushContinuousJogPoint(jointTarget, robotSpeed);
	}
	++m_nextStreamStep;
}

void RobotJogDialog::RefreshStateText()
{
	if (m_stateLabel == nullptr || m_robotDriver == nullptr)
	{
		return;
	}

	long long robotMs = 0;
	long long pcRecvMs = 0;
	T_ROBOT_COORS pos;
	T_ANGLE_PULSE pulse;
	int done = -1;
	RobotDriverAdaptor::StateSnapshot snapshot;
	if (m_robotDriver->LatestStateSnapshot(snapshot))
	{
		robotMs = snapshot.robotMs;
		pcRecvMs = snapshot.pcRecvMs;
		pos = snapshot.pose;
		pulse = snapshot.pulse;
		done = snapshot.done;
	}
	const RobotMotionStatus motionStatus = m_robotDriver->ReadMotionStatusPassive();
	QString doneText;
	switch (motionStatus.state)
	{
	case RobotMotionState::Idle: doneText = "空闲"; break;
	case RobotMotionState::Starting: doneText = "启动中"; break;
	case RobotMotionState::Running: doneText = "运行中"; break;
	case RobotMotionState::Paused: doneText = "已暂停"; break;
	case RobotMotionState::Completed: doneText = "已完成"; break;
	case RobotMotionState::Interrupted: doneText = "已中断"; break;
	case RobotMotionState::Faulted: doneText = "故障"; break;
	default: doneText = QString("未知(%1)").arg(motionStatus.rawCode); break;
	}
	const QString sourceText = QString::fromStdString(m_robotDriver->GetStateMonitorSourceText());
	if (m_jogOperationLease)
	{
		if (!m_robotDriver->IsContinuousJogRunning())
		{
			m_jogOperationLease.reset();
		}
	}
	UpdateMotionButtonState();

	m_stateLabel->setText(QString(
		"状态: %1    接口: %2    robot_ms=%3    pc_recv_ms=%4\n"
		"直角: X=%5  Y=%6  Z=%7  RX=%8  RY=%9  RZ=%10\n"
		"关节: J1=%11  J2=%12  J3=%13  J4=%14  J5=%15  J6=%16")
		.arg(doneText)
		.arg(sourceText)
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

void RobotJogDialog::UpdateMotionButtonState()
{
	const bool busy = IsMotionBusy();
	for (QPushButton* button : m_motionButtons)
	{
		if (button != nullptr)
		{
			const int capabilityMode = button->property("robotCapabilityMode").toInt();
			bool capabilityReady = m_robotDriver != nullptr;
			if (capabilityReady && capabilityMode == JOG_CAPABILITY_PASSIVE)
			{
				capabilityReady = m_robotDriver->Supports(RobotDriverCapability::PassiveState);
			}
			else if (capabilityReady && capabilityMode == JOG_CAPABILITY_CARTESIAN)
			{
				capabilityReady = m_robotDriver->SupportsAll(
					{ RobotDriverCapability::LinearMotion,
					  RobotDriverCapability::PassiveState,
					  RobotDriverCapability::VerifiedProgramCompletion,
					  RobotDriverCapability::VerifiedSafeAbort });
			}
			else if (capabilityReady && capabilityMode == JOG_CAPABILITY_JOINT)
			{
				capabilityReady = m_robotDriver->SupportsAll(
					{ RobotDriverCapability::JointMotion,
					  RobotDriverCapability::PassiveState,
					  RobotDriverCapability::VerifiedProgramCompletion,
					  RobotDriverCapability::VerifiedSafeAbort });
			}
			button->setEnabled(!busy && capabilityReady);
			button->setToolTip(capabilityReady
				? QString()
				: QStringLiteral("当前机器人品牌底层缺少本操作所需适配能力，功能已限制。"));
		}
	}
}

void RobotJogDialog::SetMotionTaskRunning(bool running)
{
	m_motionTaskRunning = running;
	UpdateMotionButtonState();
}

bool RobotJogDialog::IsMotionBusy() const
{
	if (m_jogActive || m_motionTaskRunning)
	{
		return true;
	}
	if (m_robotDriver == nullptr)
	{
		return false;
	}
	RobotDriverAdaptor::StateSnapshot snapshot;
	if (m_robotDriver->LatestStateSnapshot(snapshot))
	{
		return snapshot.done == 0;
	}
	return false;
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

	if (m_robotDriver == nullptr)
	{
		target = T_ROBOT_COORS();
	}
	else
	{
		RobotDriverAdaptor::StateSnapshot snapshot;
		target = m_robotDriver->LatestStateSnapshot(snapshot) ? snapshot.pose : T_ROBOT_COORS();
	}
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

	if (m_robotDriver == nullptr)
	{
		target = T_ANGLE_PULSE();
	}
	else
	{
		RobotDriverAdaptor::StateSnapshot snapshot;
		target = m_robotDriver->LatestStateSnapshot(snapshot) ? snapshot.pulse : T_ANGLE_PULSE();
	}
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
	const double pulseUnit = AxisPulseUnit(m_robotDriver, m_currentAxis);
	const long deltaPulse = pulseUnit == 0.0 ? 0 : static_cast<long>(std::lround(deltaDeg / pulseUnit));
	AddJointDelta(target, m_currentAxis, deltaPulse);
	return target;
}
