#include "QtWidgetsApplication4.h"
#include <QMessageBox>  // 弹窗头文件，测试用
#include "CameraFrameCache.h"
#include "FTPClient.h"
#include "FANUCRobotDriver.h"
#include "CameraBasicParamDialog.h"
#include "CameraParamDialog.h"
#include "FunctionTestDialog.h"
#include "HandEyeCalibrationDialog.h"
#include "MeasureThenWeldDialog.h"
#include "MeasureThenWeldService.h"
#include "OPini.h"
#include "PreciseMeasureEditDialog.h"
#include "RobotCalculation.h"
#include "RobotDataHelper.h"
#include "RobotJogDialog.h"
#include "RobotMessage.h"
#include "STEPRobotDriver.h"
#include "TouchKeyboardManager.h"
#include "WindowStyleHelper.h"
#include "WeldPoseAverageUpdater.h"
#include "WeldProcessDialog.h"
#include "WeldSeamCompDialog.h"
#include "../portable/LaserFramePoint3DFilter/LaserFramePoint3DFilter.h"
#include "groove/tcpsensorclientworker.h"
#include "groove/framebuffer.h"
#include <QApplication>
#include <QByteArray>
#include <QCoreApplication>
#include <QComboBox>
#include <QCryptographicHash>
#include <QDialog>
#include <QDir>
#include <QDirIterator>
#include <QDoubleValidator>
#include <QFile>
#include <QFileInfo>
#include <QFormLayout>
#include <QGuiApplication>
#include <QInputDialog>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHostAddress>
#include <QKeyEvent>
#include <QLabel>
#include <QLayout>
#include <QLineEdit>
#include <QLineF>
#include <QIntValidator>
#include <QAction>
#include <QCheckBox>
#include <QFrame>
#include <QFontMetrics>
#include <QMetaObject>
#include <QMenu>
#include <QMenuBar>
#include <QMouseEvent>
#include <QPainter>
#include <QPushButton>
#include <QResizeEvent>
#include <QScrollArea>
#include <QScrollBar>
#include <QSignalBlocker>
#include <QSizePolicy>
#include <QGraphicsDropShadowEffect>
#include <QPointer>
#include <QPixmap>
#include <QRegularExpression>
#include <QTableWidget>
#include <QHeaderView>
#include <QSplitter>
#include <QStackedWidget>
#include <QStatusBar>
#include <QStandardItem>
#include <QStandardItemModel>
#include <QScreen>
#include <QSet>
#include <QSettings>
#include <QThread>
#include <QTimer>
#include <QToolBar>
#include <QTextDocument>
#include <QTextStream>
#include <QVBoxLayout>
#include <QWheelEvent>
#include <QStringList>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <iostream>
#include <limits>
#include <thread>
#include <utility>
#ifdef Q_OS_WIN
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#endif

namespace
{
	const QString kRoleOperator = QStringLiteral("operator");
	const QString kRoleEngineer = QStringLiteral("engineer");
	const QString kRoleAdmin = QStringLiteral("admin");
	const QString kWorkpieceCorrugatedPlate = QStringLiteral("CorrugatedPlate");
	const QString kWorkpieceCorrugatedPlateName = QStringLiteral("波纹板");

	QString DecodeConfigText(const std::string& text);

	struct RobotSetupStatus
	{
		bool enabled = true;
		QString workpieceType = kWorkpieceCorrugatedPlate;
		bool cameraParamReady = true;
		bool handEyeReady = true;
	};

	std::string ToIniBytesGlobal(const QString& text)
	{
		return text.toLocal8Bit().toStdString();
	}

	void MarkNumericEditGlobal(QLineEdit* edit)
	{
		if (edit == nullptr)
		{
			return;
		}
		edit->setProperty("touchKeyboardLayout", QStringLiteral("numeric"));
		edit->setInputMethodHints(Qt::ImhFormattedNumbersOnly);
	}

	class IpAddressEdit final : public QWidget
	{
	public:
		explicit IpAddressEdit(QWidget* parent = nullptr)
			: QWidget(parent)
		{
			QHBoxLayout* layout = new QHBoxLayout(this);
			layout->setContentsMargins(0, 0, 0, 0);
			layout->setSpacing(3);
			for (int index = 0; index < 4; ++index)
			{
				QLineEdit* partEdit = new QLineEdit(this);
				partEdit->setFixedSize(48, 36);
				partEdit->setMaxLength(3);
				partEdit->setAlignment(Qt::AlignCenter);
				partEdit->setStyleSheet(
					"QLineEdit { background: #0B1117; color: #F5FAFA; border: 1px solid #385366; "
					"border-radius: 0px; padding: 4px 3px; }"
					"QLineEdit:focus { border-color: #72D4DD; }");
				partEdit->setValidator(new QIntValidator(0, 255, partEdit));
				partEdit->installEventFilter(this);
				MarkNumericEditGlobal(partEdit);
				connect(partEdit, &QLineEdit::textEdited, this, [this, index](const QString& text)
					{
						if (text.size() >= 3 && m_partEdits.value(index)->hasAcceptableInput())
						{
							FocusPart(index + 1);
						}
					});
				m_partEdits.push_back(partEdit);
				layout->addWidget(partEdit);
				if (index < 3)
				{
					QLabel* dotLabel = new QLabel(".", this);
					dotLabel->setFixedWidth(6);
					dotLabel->setAlignment(Qt::AlignCenter);
					dotLabel->setStyleSheet("QLabel { color: #CFEFF5; }");
					layout->addWidget(dotLabel);
				}
			}
			setFixedWidth(228);
			setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
		}

		QString text() const
		{
			bool hasAnyPart = false;
			QStringList parts;
			for (const QLineEdit* partEdit : m_partEdits)
			{
				const QString part = partEdit != nullptr ? partEdit->text().trimmed() : QString();
				hasAnyPart = hasAnyPart || !part.isEmpty();
				parts.push_back(part);
			}
			return hasAnyPart ? parts.join(".") : QString();
		}

		void setText(const QString& ip)
		{
			const QStringList parts = ip.trimmed().split('.', Qt::KeepEmptyParts);
			for (int index = 0; index < m_partEdits.size(); ++index)
			{
				m_partEdits[index]->setText(index < parts.size() ? parts.at(index).trimmed().left(3) : QString());
			}
		}

		bool isEmpty() const
		{
			for (const QLineEdit* partEdit : m_partEdits)
			{
				if (partEdit != nullptr && !partEdit->text().trimmed().isEmpty())
				{
					return false;
				}
			}
			return true;
		}

		bool isComplete() const
		{
			for (const QLineEdit* partEdit : m_partEdits)
			{
				if (partEdit == nullptr || partEdit->text().trimmed().isEmpty() || !partEdit->hasAcceptableInput())
				{
					return false;
				}
			}
			return true;
		}

		QList<QLineEdit*> partEdits() const
		{
			return m_partEdits;
		}

	protected:
		bool eventFilter(QObject* watched, QEvent* event) override
		{
			if (event->type() != QEvent::KeyPress)
			{
				return QWidget::eventFilter(watched, event);
			}
			QKeyEvent* keyEvent = static_cast<QKeyEvent*>(event);
			const int index = m_partEdits.indexOf(qobject_cast<QLineEdit*>(watched));
			if (index < 0)
			{
				return QWidget::eventFilter(watched, event);
			}
			if (keyEvent->key() == Qt::Key_Period || keyEvent->key() == Qt::Key_Comma)
			{
				FocusPart(index + 1);
				return true;
			}
			if (keyEvent->key() == Qt::Key_Backspace && m_partEdits[index]->text().isEmpty())
			{
				FocusPart(index - 1);
				return true;
			}
			return QWidget::eventFilter(watched, event);
		}

	private:
		void FocusPart(int index)
		{
			if (index < 0 || index >= m_partEdits.size() || m_partEdits[index] == nullptr)
			{
				return;
			}
			m_partEdits[index]->setFocus();
			m_partEdits[index]->selectAll();
		}

		QList<QLineEdit*> m_partEdits;
	};

	void ConnectIpAddressEdited(IpAddressEdit* ipEdit, QObject* context, const std::function<void()>& callback)
	{
		if (ipEdit == nullptr || context == nullptr)
		{
			return;
		}
		for (QLineEdit* partEdit : ipEdit->partEdits())
		{
			QObject::connect(partEdit, &QLineEdit::textEdited, context, [callback](const QString&)
				{
					callback();
				});
		}
	}

	void SetFormRowVisible(QFormLayout* form, QWidget* fieldWidget, bool visible)
	{
		if (fieldWidget == nullptr)
		{
			return;
		}
		if (form != nullptr)
		{
			if (QWidget* label = form->labelForField(fieldWidget))
			{
				label->setVisible(visible);
			}
		}
		fieldWidget->setVisible(visible);
	}

	QString RobotParaPathForSetup(const QString& robotName)
	{
		return RobotDataHelper::BuildProjectPath(QString("Data/%1/RobotPara.ini").arg(robotName));
	}

	bool WriteRobotSetupReadyFlagGlobal(const QString& robotName, const QString& key, QString* error = nullptr)
	{
		COPini ini;
		const QString path = RobotParaPathForSetup(robotName);
		if (!ini.SetFileName(false, ToIniBytesGlobal(path)))
		{
			if (error != nullptr)
			{
				*error = QString("打开机器人参数文件失败：%1").arg(path);
			}
			return false;
		}
		ini.SetSectionName("SetupStatus");
		if (!ini.WriteString(ToIniBytesGlobal(key), 1))
		{
			if (error != nullptr)
			{
				*error = QString("写入设置完成状态失败：%1 [%2]").arg(path, key);
			}
			return false;
		}
		return true;
	}

	QString ReadIniStringGlobal(COPini& ini, const QString& key, const QString& fallback = QString())
	{
		std::string rawValue;
		if (ini.ReadString(false, ToIniBytesGlobal(key), rawValue) > 0)
		{
			return DecodeConfigText(rawValue);
		}
		return fallback;
	}

	int ReadIniIntGlobal(COPini& ini, const QString& key, int fallback = 0)
	{
		int value = fallback;
		if (ini.ReadString(false, ToIniBytesGlobal(key), &value) > 0)
		{
			return value;
		}
		return fallback;
	}

	RobotSetupStatus LoadRobotSetupStatus(const QString& robotName, bool defaultReady = true)
	{
		RobotSetupStatus status;
		status.cameraParamReady = defaultReady;
		status.handEyeReady = defaultReady;

		if (robotName.trimmed().isEmpty())
		{
			return status;
		}

		COPini ini;
		if (!ini.SetFileName(ToIniBytesGlobal(RobotParaPathForSetup(robotName))))
		{
			return status;
		}

		ini.SetSectionName("SetupStatus");
		status.enabled = ReadIniIntGlobal(ini, "Enabled", status.enabled ? 1 : 0) != 0;
		status.workpieceType = ReadIniStringGlobal(ini, "WorkpieceType", status.workpieceType).trimmed();
		if (status.workpieceType.isEmpty())
		{
			status.workpieceType = kWorkpieceCorrugatedPlate;
		}
		status.cameraParamReady = ReadIniIntGlobal(ini, "CameraParamReady", status.cameraParamReady ? 1 : 0) != 0;
		status.handEyeReady = ReadIniIntGlobal(ini, "HandEyeReady", status.handEyeReady ? 1 : 0) != 0;
		return status;
	}

	QString WorkpieceDisplayName(const QString& workpieceType)
	{
		return workpieceType.compare(kWorkpieceCorrugatedPlate, Qt::CaseInsensitive) == 0
			? kWorkpieceCorrugatedPlateName
			: workpieceType;
	}

	QString DecodeConfigText(const std::string& text)
	{
		if (text.empty())
		{
			return QString();
		}

		const QByteArray bytes(text.data(), static_cast<int>(text.size()));
#ifdef Q_OS_WIN
		const auto decodeWindowsCodePage = [&bytes](UINT codePage, DWORD flags) -> QString
			{
				const int wideLength = MultiByteToWideChar(
					codePage,
					flags,
					bytes.constData(),
					bytes.size(),
					nullptr,
					0);
				if (wideLength <= 0)
				{
					return QString();
				}
				std::wstring wideText(static_cast<size_t>(wideLength), L'\0');
				MultiByteToWideChar(
					codePage,
					flags,
					bytes.constData(),
					bytes.size(),
					wideText.data(),
					wideLength);
				return QString::fromWCharArray(wideText.data(), wideLength);
			};

		QString decodedUtf8Text = decodeWindowsCodePage(CP_UTF8, MB_ERR_INVALID_CHARS);
		if (!decodedUtf8Text.isNull() && !decodedUtf8Text.contains(QChar(0xfffd)))
		{
			return decodedUtf8Text;
		}
		QString decodedGbkText = decodeWindowsCodePage(936, 0);
		if (!decodedGbkText.isNull())
		{
			return decodedGbkText;
		}
#endif
		const QString utf8Text = QString::fromUtf8(bytes.constData(), bytes.size());
		if (!utf8Text.contains(QChar(0xfffd)))
		{
			return utf8Text;
		}
		return QString::fromLocal8Bit(bytes.constData(), bytes.size());
	}

	QString RobotDriverTypeText(const RobotDriverAdaptor* driver)
	{
		if (driver == nullptr)
		{
			return "未知类型";
		}
		switch (driver->m_nRobotType)
		{
		case ROBOT_TYPE_STEP:
			return "STEP";
		case ROBOT_TYPE_FANUC:
			return "FANUC";
		default:
			return QString("未知类型%1").arg(driver->m_nRobotType);
		}
	}

	void CenterWindowOnScreen(QWidget* window)
	{
		if (window == nullptr)
		{
			return;
		}

		QScreen* screen = QGuiApplication::screenAt(window->frameGeometry().center());
		if (screen == nullptr)
		{
			screen = QGuiApplication::primaryScreen();
		}
		if (screen == nullptr)
		{
			return;
		}

		QRect frame = window->frameGeometry();
		frame.moveCenter(screen->availableGeometry().center());
		window->move(frame.topLeft());
	}

	void ShowMaximizedWithUnifiedChrome(QWidget* window)
	{
		if (window == nullptr)
		{
			return;
		}

		window->showMaximized();
		RefreshUnifiedWindowTitleBar(window);
		QTimer::singleShot(0, window, [window]() { RefreshUnifiedWindowTitleBar(window); });
		QTimer::singleShot(120, window, [window]() { RefreshUnifiedWindowTitleBar(window); });
	}

	QPushButton* EmbeddedBackButton(QWidget* page)
	{
		if (page == nullptr)
		{
			return nullptr;
		}
		return page->findChild<QPushButton*>("EmbeddedBackToDashboardButton", Qt::FindDirectChildrenOnly);
	}

	void PositionEmbeddedBackButton(QWidget* page)
	{
		QPushButton* backButton = EmbeddedBackButton(page);
		if (page == nullptr || backButton == nullptr)
		{
			return;
		}
		const QSize buttonSize(118, 38);
		backButton->setFixedSize(buttonSize);
		backButton->move(std::max(16, page->width() - buttonSize.width() - 22), 16);
		backButton->raise();
	}

	bool IsLikelyDebugLogWidget(const QPlainTextEdit* edit)
	{
		if (edit == nullptr || !edit->isReadOnly())
		{
			return false;
		}
		if (edit->property("_debug_log_exempt").toBool())
		{
			return false;
		}
		const QString objectName = edit->objectName();
		return objectName.contains("log", Qt::CaseInsensitive)
			|| edit->maximumHeight() <= 220
			|| edit->document()->maximumBlockCount() >= 300;
	}

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

	QString BuildWeldSeamCompOutputPath(const QString& inputFilePath)
	{
		const QFileInfo inputInfo(inputFilePath);
		const QString suffix = inputInfo.completeSuffix();
		const QString fileName = suffix.isEmpty()
			? QString("%1_SeamComp").arg(inputInfo.completeBaseName())
			: QString("%1_SeamComp.%2").arg(inputInfo.completeBaseName(), suffix);
		return inputInfo.dir().filePath(fileName);
	}

	QString NormalizeCameraAddressKey(const QString& ipText)
	{
		const QString trimmed = ipText.trimmed();
		const QHostAddress address(trimmed);
		if (!address.isNull())
		{
			return QHostAddress(address.toIPv4Address()).toString();
		}
		return trimmed;
	}

	QString InferRobotNameFromResultPath(const QString& inputFilePath)
	{
		const QString normalizedPath = QDir::fromNativeSeparators(
			QFileInfo(inputFilePath).absoluteFilePath());
		const QStringList pathParts = normalizedPath.split('/', Qt::SkipEmptyParts);
		for (int index = 0; index + 1 < pathParts.size(); ++index)
		{
			if (pathParts[index].compare("Result", Qt::CaseInsensitive) == 0)
			{
				const QString robotName = pathParts[index + 1].trimmed();
				if (!robotName.isEmpty())
				{
					return robotName;
				}
				break;
			}
		}
		return "RobotA";
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

	template <typename DialogType>
	DialogType* PrepareTaskbarDialog(DialogType* dialog, QWidget* owner)
	{
		if (dialog == nullptr)
		{
			return nullptr;
		}

		dialog->setAttribute(Qt::WA_DeleteOnClose);
		dialog->setParent(nullptr);
		dialog->setWindowModality(Qt::NonModal);
		Qt::WindowFlags flags = dialog->windowFlags();
		flags &= ~Qt::WindowType_Mask;
		flags |= Qt::Window
			| Qt::WindowTitleHint
			| Qt::WindowSystemMenuHint
			| Qt::WindowMinimizeButtonHint
			| Qt::WindowMaximizeButtonHint
			| Qt::WindowCloseButtonHint;
		dialog->setWindowFlags(flags);
		if (owner != nullptr)
		{
			dialog->setWindowIcon(owner->windowIcon());
			QObject::connect(owner, &QObject::destroyed, dialog, &QWidget::close);
		}
#ifdef Q_OS_WIN
		const HWND hwnd = reinterpret_cast<HWND>(dialog->winId());
		LONG_PTR exStyle = GetWindowLongPtr(hwnd, GWL_EXSTYLE);
		exStyle &= ~WS_EX_TOOLWINDOW;
		exStyle |= WS_EX_APPWINDOW;
		SetWindowLongPtr(hwnd, GWL_EXSTYLE, exStyle);
		SetWindowPos(
			hwnd,
			nullptr,
			0,
			0,
			0,
			0,
			SWP_NOMOVE | SWP_NOSIZE | SWP_NOZORDER | SWP_FRAMECHANGED);
#endif
		return dialog;
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

	QString CliOptionValue(const QStringList& arguments, const QString& option)
	{
		const int optionIndex = arguments.indexOf(option);
		if (optionIndex < 0 || optionIndex + 1 >= arguments.size())
		{
			return QString();
		}
		return arguments[optionIndex + 1].trimmed();
	}

	bool CliOptionContains(const QStringList& arguments, const QString& option)
	{
		return arguments.contains(option);
	}

	bool ParseCliDoubleValues(
		const QString& text,
		int minCount,
		int maxCount,
		QVector<double>& values,
		QString* errorText)
	{
		values.clear();

		QString normalized = text.trimmed();
		normalized.replace(QChar(0xff0c), ' ');
		normalized.replace(QChar(0x3001), ' ');
		normalized.replace(',', ' ');
		normalized.replace(';', ' ');
		normalized.replace('|', ' ');
		const QStringList parts = normalized.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
		if (parts.size() < minCount || parts.size() > maxCount)
		{
			if (errorText != nullptr)
			{
				*errorText = QString("参数数量错误：需要 %1~%2 个数值，实际 %3 个。")
					.arg(minCount)
					.arg(maxCount)
					.arg(parts.size());
			}
			return false;
		}

		values.reserve(parts.size());
		for (const QString& part : parts)
		{
			bool ok = false;
			const double value = part.toDouble(&ok);
			if (!ok || !std::isfinite(value))
			{
				if (errorText != nullptr)
				{
					*errorText = QString("数值解析失败：%1").arg(part);
				}
				values.clear();
				return false;
			}
			values.push_back(value);
		}
		return true;
	}

	bool TryParseCliDoubleOption(
		const QStringList& arguments,
		const QString& option,
		double& value,
		bool& specified,
		QString* errorText)
	{
		specified = false;
		const int optionIndex = arguments.indexOf(option);
		if (optionIndex < 0)
		{
			return true;
		}
		specified = true;
		if (optionIndex + 1 >= arguments.size())
		{
			if (errorText != nullptr)
			{
				*errorText = QString("%1 缺少数值。").arg(option);
			}
			return false;
		}

		bool ok = false;
		value = arguments[optionIndex + 1].toDouble(&ok);
		if (!ok || !std::isfinite(value) || value <= 0.0)
		{
			if (errorText != nullptr)
			{
				*errorText = QString("%1 数值无效：%2").arg(option, arguments[optionIndex + 1]);
			}
			return false;
		}
		return true;
	}

	bool TryParseCliIntOption(
		const QStringList& arguments,
		const QString& option,
		int& value,
		bool& specified,
		QString* errorText)
	{
		specified = false;
		const int optionIndex = arguments.indexOf(option);
		if (optionIndex < 0)
		{
			return true;
		}
		specified = true;
		if (optionIndex + 1 >= arguments.size())
		{
			if (errorText != nullptr)
			{
				*errorText = QString("%1 缺少数值。").arg(option);
			}
			return false;
		}

		bool ok = false;
		value = arguments[optionIndex + 1].toInt(&ok);
		if (!ok || value < 0)
		{
			if (errorText != nullptr)
			{
				*errorText = QString("%1 数值无效：%2").arg(option, arguments[optionIndex + 1]);
			}
			return false;
		}
		return true;
	}

	QString BuildRobotCliLabel(const T_CONTRAL_UNIT& unitInfo)
	{
		RobotDriverAdaptor* driver = static_cast<RobotDriverAdaptor*>(unitInfo.pUnitDriver);
		const QString unitName = DecodeConfigText(unitInfo.sUnitName).trimmed();
		const QString chineseName = DecodeConfigText(unitInfo.sChineseName).trimmed();
		const QString displayName = chineseName.isEmpty() ? unitName : chineseName;
		const QString typeText = driver == nullptr
			? DecodeConfigText(unitInfo.sContralUnitType).trimmed().toUpper()
			: RobotDriverTypeText(driver);
		return QString("%1 / %2 (%3)")
			.arg(displayName.isEmpty() ? QString("Unit%1").arg(unitInfo.nUnitNo) : displayName)
			.arg(typeText.isEmpty() ? QStringLiteral("未知类型") : typeText)
			.arg(unitName.isEmpty() ? QString("Unit%1").arg(unitInfo.nUnitNo) : unitName);
	}

	bool CliTextEquals(const QString& lhs, const QString& rhs)
	{
		return !lhs.trimmed().isEmpty()
			&& lhs.trimmed().compare(rhs.trimmed(), Qt::CaseInsensitive) == 0;
	}

	QString FormatCliCoors(const T_ROBOT_COORS& coors)
	{
		return QString("X=%1 Y=%2 Z=%3 RX=%4 RY=%5 RZ=%6 BX=%7 BY=%8 BZ=%9")
			.arg(coors.dX, 0, 'f', 6)
			.arg(coors.dY, 0, 'f', 6)
			.arg(coors.dZ, 0, 'f', 6)
			.arg(coors.dRX, 0, 'f', 6)
			.arg(coors.dRY, 0, 'f', 6)
			.arg(coors.dRZ, 0, 'f', 6)
			.arg(coors.dBX, 0, 'f', 6)
			.arg(coors.dBY, 0, 'f', 6)
			.arg(coors.dBZ, 0, 'f', 6);
	}

	QString FormatCliPulse(const T_ANGLE_PULSE& pulse)
	{
		return QString("S=%1 L=%2 U=%3 R=%4 B=%5 T=%6 EX1=%7 EX2=%8 EX3=%9")
			.arg(pulse.nSPulse)
			.arg(pulse.nLPulse)
			.arg(pulse.nUPulse)
			.arg(pulse.nRPulse)
			.arg(pulse.nBPulse)
			.arg(pulse.nTPulse)
			.arg(pulse.lBXPulse)
			.arg(pulse.lBYPulse)
			.arg(pulse.lBZPulse);
	}

	RobotCalculation::SampleAxis InferLaserSampleAxis(
		const QVector<RobotCalculation::IndexedPoint3D>& points)
	{
		if (points.size() < 2)
		{
			return RobotCalculation::SampleAxis::AxisY;
		}

		QVector<double> xValues;
		QVector<double> yValues;
		xValues.reserve(points.size());
		yValues.reserve(points.size());
		for (const RobotCalculation::IndexedPoint3D& point : points)
		{
			xValues.push_back(point.point.x());
			yValues.push_back(point.point.y());
		}

		auto percentileValue = [](QVector<double> values, double percentile) -> double
		{
			if (values.isEmpty())
			{
				return 0.0;
			}
			std::sort(values.begin(), values.end());
			const double clampedPercentile = std::clamp(percentile, 0.0, 1.0);
			const int index = static_cast<int>(std::round(clampedPercentile * (values.size() - 1)));
			return values[index];
		};

		const double robustMinX = percentileValue(xValues, 0.05);
		const double robustMaxX = percentileValue(xValues, 0.95);
		const double robustMinY = percentileValue(yValues, 0.05);
		const double robustMaxY = percentileValue(yValues, 0.95);
		return (robustMaxX - robustMinX) > (robustMaxY - robustMinY)
			? RobotCalculation::SampleAxis::AxisX
			: RobotCalculation::SampleAxis::AxisY;
	}

	RobotCalculation::LowerWeldFilterParams BuildCliOriginalTrackFitParams(
		RobotCalculation::SampleAxis sampleAxis)
	{
		RobotCalculation::LowerWeldFilterParams params;
		params.sampleAxis = sampleAxis;
		params.fitMode = RobotCalculation::LowerWeldFitMode::PreservePath;
		params.zThreshold = -230.0;
		params.zJumpThreshold = 3.0;
		params.zContinuityThreshold = 2.0;
		params.segmentBreakDistance = 6.0;
		params.keepLongestSegmentOnly = true;
		params.sampleStep = 2.0;
		params.searchWindow = 8.0;
		params.lineFitTrimCount = 0;
		params.piecewiseFitTolerance = 4.0;
		params.piecewiseMinSegmentPoints = 10;
		params.minPointCount = 4;
		params.smoothRadius = 3;
		return params;
	}

	RobotCalculation::LowerWeldFilterParams BuildCliTrapezoidFitParams(
		const RobotCalculation::LowerWeldFilterParams& originalFitParams)
	{
		RobotCalculation::LowerWeldFilterParams params = originalFitParams;
		params.fitMode = RobotCalculation::LowerWeldFitMode::TrapezoidFit;
		params.zThreshold = std::numeric_limits<double>::max();
		params.zJumpThreshold = 0.0;
		params.zContinuityThreshold = 0.0;
		params.keepLongestSegmentOnly = false;
		params.searchWindow = std::max(2.0, originalFitParams.sampleStep);
		params.lineFitTrimCount = 0;
		params.piecewiseFitTolerance = std::max(5.0, originalFitParams.piecewiseFitTolerance);
		params.piecewiseMinSegmentPoints = std::max(12, originalFitParams.piecewiseMinSegmentPoints);
		params.minPointCount = 2;
		params.smoothRadius = 0;
		return params;
	}

	QVector<RobotCalculation::IndexedPoint3D> ToIndexedInput(
		const QVector<RobotCalculation::LowerWeldFilterPoint>& points)
	{
		QVector<RobotCalculation::IndexedPoint3D> indexedPoints;
		indexedPoints.reserve(points.size());
		for (const RobotCalculation::LowerWeldFilterPoint& point : points)
		{
			RobotCalculation::IndexedPoint3D indexedPoint;
			indexedPoint.index = point.index;
			indexedPoint.point = point.point;
			indexedPoints.push_back(indexedPoint);
		}
		return indexedPoints;
	}

	QString BuildClassifiedOutputPath(const QString& inputPath)
	{
		const QFileInfo info(inputPath);
		return info.dir().filePath(info.completeBaseName() + "_Classified.txt");
	}

	QString BuildGeometryClassifiedOutputPath(const QString& inputPath)
	{
		const QFileInfo info(inputPath);
		return info.dir().filePath(info.completeBaseName() + "_Classified_Geometry.txt");
	}

	QString BuildNoiseOutputPath(const QString& outputPath)
	{
		const QFileInfo info(outputPath);
		return info.dir().filePath(info.completeBaseName() + "_Noise.txt");
	}

	QString BuildKeyPointsOutputPath(const QString& classifiedOutputPath)
	{
		const QFileInfo info(classifiedOutputPath);
		QString baseName = info.completeBaseName();
		if (baseName.contains("_Classified_Geometry"))
		{
			baseName.replace("_Classified_Geometry", "_KeyPoints_Geometry");
		}
		else if (baseName.contains("_Classified"))
		{
			baseName.replace("_Classified", "_KeyPoints");
		}
		else
		{
			baseName += "_KeyPoints";
		}
		return info.dir().filePath(baseName + ".txt");
	}

	QString AccountManagementConfigPath()
	{
		return RobotDataHelper::BuildProjectPath("Data/Accounts.ini");
	}

	QString HashAccountPassword(const QString& userName, const QString& password)
	{
		return QString::fromLatin1(
			QCryptographicHash::hash(QString("%1\n%2").arg(userName.trimmed(), password).toUtf8(), QCryptographicHash::Sha256).toHex());
	}

	QString DisplayRoleNameForAccount(const QString& role)
	{
		if (role == kRoleAdmin)
		{
			return "管理员";
		}
		if (role == kRoleEngineer)
		{
			return "工程师";
		}
		return "操作员";
	}

	class AccountManagementDialog final : public QDialog
	{
	public:
		explicit AccountManagementDialog(QWidget* parent = nullptr)
			: QDialog(parent)
		{
			setWindowTitle("账号管理");
			ApplyUnifiedWindowChrome(this);
			ResizeWindowForAvailableGeometry(this, QSize(980, 700), 0.82, 0.78);
			setStyleSheet(
				"QDialog { background: #111820; color: #ECF3F4; }"
				"QGroupBox { border: 1px solid #2E4656; border-radius: 12px; margin-top: 16px; padding-top: 12px; }"
				"QGroupBox::title { subcontrol-origin: margin; left: 14px; padding: 0 6px; color: #9ED8DB; }"
				"QPushButton { background: #233645; color: #F5FAFA; border: 1px solid #3C6173; border-radius: 10px; padding: 10px 14px; }"
				"QPushButton:hover { background: #2D5465; border-color: #72D4DD; }"
				"QPushButton:pressed { background: #18303B; }"
				"QLineEdit, QPlainTextEdit, QTableWidget { background: #081018; color: #ECF3F4; border: 1px solid #2C4653; border-radius: 8px; padding: 6px 8px; }"
				"QTableWidget::item:selected { background: #8BE8F2; color: #071018; }"
				"QTableWidget::item:selected:!active { background: #6FCFDC; color: #071018; }"
				"QComboBox { background: #000000; color: #ECF3F4; border: 1px solid #2C4653; border-radius: 0px; padding: 6px 34px 6px 8px; }"
				"QComboBox::drop-down { border-left: 1px solid #2C4653; border-radius: 0px; width: 28px; background: #000000; }"
				"QComboBox::down-arrow { image: url(:/QtWidgetsApplication4/icons/chevron-down.svg); width: 12px; height: 8px; }"
				"QComboBox QAbstractItemView { background: #000000; color: #ECF3F4; selection-background-color: #2D5465; border: 1px solid #2C4653; border-radius: 0px; outline: 0px; }"
				"QHeaderView::section { background: #13202A; color: #BFE8EC; border: 0px; padding: 6px; }");

			QVBoxLayout* outerLayout = new QVBoxLayout(this);
			outerLayout->setContentsMargins(0, 0, 0, 0);
			outerLayout->setSpacing(0);
			QScrollArea* pageScrollArea = new QScrollArea(this);
			pageScrollArea->setObjectName("AdaptiveWindowScrollArea");
			pageScrollArea->setWidgetResizable(true);
			pageScrollArea->setAlignment(Qt::AlignLeft | Qt::AlignTop);
			pageScrollArea->setFrameShape(QFrame::NoFrame);
			pageScrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
			pageScrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
			outerLayout->addWidget(pageScrollArea);

			QWidget* pageWidget = new QWidget(pageScrollArea);
			pageWidget->setMinimumWidth(960);
			QVBoxLayout* rootLayout = new QVBoxLayout(pageWidget);
			rootLayout->setContentsMargins(12, 10, 12, 12);
			rootLayout->setSpacing(8);
			pageScrollArea->setWidget(pageWidget);

			QLabel* titleLabel = new QLabel("账号管理");
			titleLabel->setStyleSheet("font-size: 24px; font-weight: bold; color: #F7FCFC;");
			titleLabel->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
			titleLabel->setFixedHeight(34);
			rootLayout->addWidget(titleLabel);
			QLabel* hintLabel = new QLabel("这里可以新增账号、修改权限、重置密码或删除账号。仅管理员可使用。");
			hintLabel->setWordWrap(true);
			hintLabel->setStyleSheet("QLabel { color: #AFC8CE; font-size: 14px; }");
			hintLabel->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
			hintLabel->setMaximumHeight(48);
			rootLayout->addWidget(hintLabel);

			QGroupBox* createGroup = new QGroupBox("新增账号", this);
			QFormLayout* createForm = new QFormLayout(createGroup);
			createForm->setFieldGrowthPolicy(QFormLayout::FieldsStayAtSizeHint);
			m_createUserEdit = new QLineEdit(createGroup);
			m_createPassEdit = new QLineEdit(createGroup);
			m_createPassEdit->setEchoMode(QLineEdit::Password);
			m_createRoleCombo = new QComboBox(createGroup);
			m_createRoleCombo->addItem("操作员", kRoleOperator);
			m_createRoleCombo->addItem("工程师", kRoleEngineer);
			m_createRoleCombo->addItem("管理员", kRoleAdmin);
			m_createUserEdit->setFixedWidth(260);
			m_createPassEdit->setFixedWidth(260);
			m_createRoleCombo->setFixedWidth(180);
			createForm->addRow("账号", m_createUserEdit);
			createForm->addRow("密码", m_createPassEdit);
			createForm->addRow("权限", m_createRoleCombo);
			QHBoxLayout* createButtonLayout = new QHBoxLayout();
			QPushButton* createBtn = new QPushButton("创建账号", createGroup);
			QPushButton* refreshBtn = new QPushButton("刷新列表", createGroup);
			createBtn->setFixedWidth(160);
			refreshBtn->setFixedWidth(160);
			createButtonLayout->addWidget(createBtn);
			createButtonLayout->addWidget(refreshBtn);
			createButtonLayout->addStretch(1);
			createForm->addRow(createButtonLayout);
			rootLayout->addWidget(createGroup);

			QSplitter* splitter = new QSplitter(Qt::Horizontal, this);
			splitter->setChildrenCollapsible(false);
			QGroupBox* listGroup = new QGroupBox("账号列表", splitter);
			QVBoxLayout* listLayout = new QVBoxLayout(listGroup);
			m_accountTable = new QTableWidget(listGroup);
			m_accountTable->setColumnCount(3);
			m_accountTable->setHorizontalHeaderLabels(QStringList() << "账号" << "权限" << "创建时间");
			m_accountTable->horizontalHeader()->setStretchLastSection(true);
			m_accountTable->setSelectionBehavior(QAbstractItemView::SelectRows);
			m_accountTable->setSelectionMode(QAbstractItemView::SingleSelection);
			m_accountTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
			listLayout->addWidget(m_accountTable);

			QGroupBox* editGroup = new QGroupBox("编辑选中账号", splitter);
			QVBoxLayout* editLayout = new QVBoxLayout(editGroup);
			QFormLayout* editForm = new QFormLayout();
			editForm->setFieldGrowthPolicy(QFormLayout::FieldsStayAtSizeHint);
			m_editRoleCombo = new QComboBox(editGroup);
			m_editRoleCombo->addItem("操作员", kRoleOperator);
			m_editRoleCombo->addItem("工程师", kRoleEngineer);
			m_editRoleCombo->addItem("管理员", kRoleAdmin);
			m_editPassEdit = new QLineEdit(editGroup);
			m_editPassEdit->setEchoMode(QLineEdit::Password);
			m_editPassEdit->setPlaceholderText("留空表示不修改密码");
			m_editRoleCombo->setFixedWidth(180);
			m_editPassEdit->setFixedWidth(260);
			editForm->addRow("权限", m_editRoleCombo);
			editForm->addRow("新密码", m_editPassEdit);
			editLayout->addLayout(editForm);
			QHBoxLayout* editButtonLayout = new QHBoxLayout();
			QPushButton* saveBtn = new QPushButton("保存修改", editGroup);
			QPushButton* deleteBtn = new QPushButton("删除账号", editGroup);
			QPushButton* closeBtn = new QPushButton("关闭", editGroup);
			saveBtn->setFixedWidth(140);
			deleteBtn->setFixedWidth(140);
			closeBtn->setFixedWidth(140);
			editButtonLayout->addWidget(saveBtn);
			editButtonLayout->addWidget(deleteBtn);
			editButtonLayout->addWidget(closeBtn);
			editButtonLayout->addStretch(1);
			editLayout->addLayout(editButtonLayout);
			m_logText = new QPlainTextEdit(editGroup);
			m_logText->setReadOnly(true);
			m_logText->document()->setMaximumBlockCount(300);
			m_logText->setMaximumHeight(160);
			editLayout->addWidget(m_logText, 1);

			splitter->addWidget(listGroup);
			splitter->addWidget(editGroup);
			splitter->setStretchFactor(0, 2);
			splitter->setStretchFactor(1, 1);
			rootLayout->addWidget(splitter, 1);

			connect(createBtn, &QPushButton::clicked, this, [this]()
				{
					if (CreateAccount())
					{
						LoadAccounts();
					}
				});
			connect(refreshBtn, &QPushButton::clicked, this, [this]() { LoadAccounts(); });
			connect(saveBtn, &QPushButton::clicked, this, [this]()
				{
					if (UpdateAccount())
					{
						LoadAccounts();
					}
				});
			connect(deleteBtn, &QPushButton::clicked, this, [this]()
				{
					if (DeleteAccount())
					{
						LoadAccounts();
					}
				});
			connect(closeBtn, &QPushButton::clicked, this, &QDialog::accept);
			connect(m_accountTable, &QTableWidget::itemSelectionChanged, this, [this]() { SyncEditorFromSelection(); });
			LoadAccounts();
		}

	private:
		void AppendLog(const QString& text)
		{
			if (m_logText != nullptr)
			{
				m_logText->appendPlainText(text);
			}
		}

		QString SelectedAccount() const
		{
			if (m_accountTable == nullptr)
			{
				return QString();
			}
			const int row = m_accountTable->currentRow();
			if (row < 0)
			{
				return QString();
			}
			QTableWidgetItem* item = m_accountTable->item(row, 0);
			return item != nullptr ? item->text().trimmed() : QString();
		}

		void LoadAccounts()
		{
			if (m_accountTable == nullptr)
			{
				return;
			}

			QSettings settings(AccountManagementConfigPath(), QSettings::IniFormat);
			settings.beginGroup("Users");
			const QStringList users = settings.childGroups();
			m_accountTable->setRowCount(0);
			int row = 0;
			for (const QString& user : users)
			{
				settings.beginGroup(user);
				const QString role = settings.value("Role", kRoleOperator).toString();
				const QString createdAt = settings.value("CreatedAt").toString();
				settings.endGroup();
				m_accountTable->insertRow(row);
				m_accountTable->setItem(row, 0, new QTableWidgetItem(user));
				m_accountTable->setItem(row, 1, new QTableWidgetItem(DisplayRoleNameForAccount(role)));
				m_accountTable->setItem(row, 2, new QTableWidgetItem(createdAt));
				++row;
			}
			settings.endGroup();
			if (row > 0)
			{
				m_accountTable->selectRow(0);
			}
			AppendLog(QString("已刷新账号列表，共 %1 个账号。").arg(row));
		}

		void SyncEditorFromSelection()
		{
			const QString user = SelectedAccount();
			if (user.isEmpty())
			{
				return;
			}
			QSettings settings(AccountManagementConfigPath(), QSettings::IniFormat);
			settings.beginGroup("Users");
			settings.beginGroup(user);
			const QString role = settings.value("Role", kRoleOperator).toString();
			settings.endGroup();
			settings.endGroup();
			const int roleIndex = m_editRoleCombo->findData(role);
			if (roleIndex >= 0)
			{
				m_editRoleCombo->setCurrentIndex(roleIndex);
			}
		}

		bool CreateAccount()
		{
			const QString userName = m_createUserEdit->text().trimmed();
			const QString password = m_createPassEdit->text();
			const QString role = m_createRoleCombo->currentData().toString();
			if (userName.size() < 4)
			{
				QMessageBox::warning(this, "新增账号", "账号至少需要 4 个字符。");
				return false;
			}
			if (password.size() < 8)
			{
				QMessageBox::warning(this, "新增账号", "密码至少需要 8 个字符。");
				return false;
			}
			if (role != kRoleOperator && role != kRoleEngineer && role != kRoleAdmin)
			{
				QMessageBox::warning(this, "新增账号", "权限类型无效。");
				return false;
			}

			QSettings settings(AccountManagementConfigPath(), QSettings::IniFormat);
			settings.beginGroup("Users");
			if (settings.childGroups().contains(userName))
			{
				settings.endGroup();
				QMessageBox::warning(this, "新增账号", "账号已存在。");
				return false;
			}
			settings.beginGroup(userName);
			settings.setValue("PasswordHash", HashAccountPassword(userName, password));
			settings.setValue("Role", role);
			settings.setValue("CreatedAt", QDateTime::currentDateTime().toString(Qt::ISODate));
			settings.endGroup();
			settings.endGroup();
			settings.sync();
			if (settings.status() != QSettings::NoError)
			{
				QMessageBox::warning(this, "新增账号", "写入账号文件失败。");
				return false;
			}

			m_createUserEdit->clear();
			m_createPassEdit->clear();
			AppendLog(QString("已创建账号 %1，权限：%2。").arg(userName, DisplayRoleNameForAccount(role)));
			return true;
		}

		bool UpdateAccount()
		{
			const QString userName = SelectedAccount();
			if (userName.isEmpty())
			{
				QMessageBox::warning(this, "保存修改", "请先选择一个账号。");
				return false;
			}

			QSettings settings(AccountManagementConfigPath(), QSettings::IniFormat);
			settings.beginGroup("Users");
			if (!settings.childGroups().contains(userName))
			{
				settings.endGroup();
				QMessageBox::warning(this, "保存修改", "账号不存在。");
				return false;
			}
			settings.beginGroup(userName);
			settings.setValue("Role", m_editRoleCombo->currentData().toString());
			const QString newPassword = m_editPassEdit->text();
			if (!newPassword.isEmpty())
			{
				if (newPassword.size() < 8)
				{
					settings.endGroup();
					settings.endGroup();
					QMessageBox::warning(this, "保存修改", "新密码至少需要 8 个字符。");
					return false;
				}
				settings.setValue("PasswordHash", HashAccountPassword(userName, newPassword));
			}
			settings.setValue("UpdatedAt", QDateTime::currentDateTime().toString(Qt::ISODate));
			settings.endGroup();
			settings.endGroup();
			settings.sync();
			if (settings.status() != QSettings::NoError)
			{
				QMessageBox::warning(this, "保存修改", "保存账号失败。");
				return false;
			}
			m_editPassEdit->clear();
			AppendLog(QString("已更新账号 %1 的权限与密码。").arg(userName));
			return true;
		}

		bool DeleteAccount()
		{
			const QString userName = SelectedAccount();
			if (userName.isEmpty())
			{
				QMessageBox::warning(this, "删除账号", "请先选择一个账号。");
				return false;
			}
			if (QMessageBox::question(this, "删除账号", QString("确定删除账号 %1 吗？").arg(userName),
				QMessageBox::Yes | QMessageBox::No, QMessageBox::No) != QMessageBox::Yes)
			{
				return false;
			}

			QSettings settings(AccountManagementConfigPath(), QSettings::IniFormat);
			settings.beginGroup("Users");
			settings.remove(userName);
			settings.endGroup();
			settings.sync();
			if (settings.status() != QSettings::NoError)
			{
				QMessageBox::warning(this, "删除账号", "删除账号失败。");
				return false;
			}
			AppendLog(QString("已删除账号 %1。").arg(userName));
			return true;
		}

	private:
		QTableWidget* m_accountTable = nullptr;
		QLineEdit* m_createUserEdit = nullptr;
		QLineEdit* m_createPassEdit = nullptr;
		QComboBox* m_createRoleCombo = nullptr;
		QComboBox* m_editRoleCombo = nullptr;
		QLineEdit* m_editPassEdit = nullptr;
		QPlainTextEdit* m_logText = nullptr;
	};

	class ControlUnitManagementDialog final : public QDialog
	{
	public:
		using OpenCameraBasicParamFunc = std::function<bool(const QString&, const QString&)>;
		using OpenHandEyeCalibrationFunc = std::function<bool(const QString&, const QString&)>;

		explicit ControlUnitManagementDialog(
			std::function<void()> reloadCallback,
			OpenCameraBasicParamFunc openCameraBasicParam,
			OpenHandEyeCalibrationFunc openHandEyeCalibration,
			QWidget* parent = nullptr)
			: QDialog(parent)
			, m_reloadCallback(std::move(reloadCallback))
			, m_openCameraBasicParam(std::move(openCameraBasicParam))
			, m_openHandEyeCalibration(std::move(openHandEyeCalibration))
		{
			setWindowTitle("机器人控制单元管理");
			ApplyUnifiedWindowChrome(this);
			ResizeWindowForAvailableGeometry(this, QSize(1080, 720), 0.84, 0.78);
			setStyleSheet(
				"QDialog { background: #111820; color: #ECF3F4; }"
				"QGroupBox { border: 1px solid #2E4656; border-radius: 12px; margin-top: 16px; padding-top: 12px; }"
				"QGroupBox::title { subcontrol-origin: margin; left: 14px; padding: 0 6px; color: #9ED8DB; }"
				"QPushButton { background: #233645; color: #F5FAFA; border: 1px solid #3C6173; border-radius: 10px; padding: 10px 14px; }"
				"QPushButton:hover { background: #2D5465; border-color: #72D4DD; }"
				"QPushButton:pressed { background: #18303B; }"
				"QLineEdit, QPlainTextEdit, QTableWidget { background: #081018; color: #ECF3F4; border: 1px solid #2C4653; border-radius: 8px; padding: 6px 8px; }"
				"QTableWidget::item:selected { background: #8BE8F2; color: #071018; }"
				"QTableWidget::item:selected:!active { background: #6FCFDC; color: #071018; }"
				"QLineEdit[readOnly=\"true\"] { color: #91A7AE; background: #0A121A; }"
				"QComboBox { background: #000000; color: #ECF3F4; border: 1px solid #2C4653; border-radius: 0px; padding: 6px 34px 6px 8px; }"
				"QComboBox::drop-down { border-left: 1px solid #2C4653; border-radius: 0px; width: 28px; background: #000000; }"
				"QComboBox::down-arrow { image: url(:/QtWidgetsApplication4/icons/chevron-down.svg); width: 12px; height: 8px; }"
				"QComboBox QAbstractItemView { background: #000000; color: #ECF3F4; selection-background-color: #2D5465; border: 1px solid #2C4653; border-radius: 0px; outline: 0px; }"
				"QHeaderView::section { background: #13202A; color: #BFE8EC; border: 0px; padding: 6px; }");

			QVBoxLayout* outerLayout = new QVBoxLayout(this);
			outerLayout->setContentsMargins(0, 0, 0, 0);
			outerLayout->setSpacing(0);
			QScrollArea* pageScrollArea = new QScrollArea(this);
			pageScrollArea->setObjectName("AdaptiveWindowScrollArea");
			pageScrollArea->setWidgetResizable(true);
			pageScrollArea->setAlignment(Qt::AlignLeft | Qt::AlignTop);
			pageScrollArea->setFrameShape(QFrame::NoFrame);
			pageScrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
			pageScrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
			outerLayout->addWidget(pageScrollArea);

			QWidget* pageWidget = new QWidget(pageScrollArea);
			pageWidget->setMinimumWidth(1080);
			QVBoxLayout* rootLayout = new QVBoxLayout(pageWidget);
			rootLayout->setContentsMargins(18, 16, 18, 18);
			rootLayout->setSpacing(12);
			pageScrollArea->setWidget(pageWidget);

			QLabel* titleLabel = new QLabel("机器人控制单元管理", pageWidget);
			titleLabel->setStyleSheet("font-size: 24px; font-weight: bold; color: #F7FCFC;");
			titleLabel->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
			titleLabel->setMaximumHeight(44);
			rootLayout->addWidget(titleLabel);

			QLabel* hintLabel = new QLabel(
				"这里维护 ContralUnitInfo.ini 的控制单元列表，以及每个机器人 RobotPara.ini 里的 IP、端口和 FTP 参数。保存后建议重新加载控制单元，正在运行流程时不要重载。",
				pageWidget);
			hintLabel->setWordWrap(true);
			hintLabel->setAlignment(Qt::AlignLeft | Qt::AlignTop);
			hintLabel->setStyleSheet("QLabel { color: #AFC8CE; font-size: 14px; }");
			hintLabel->setFixedHeight(34);
			rootLayout->addWidget(hintLabel);

			QSplitter* splitter = new QSplitter(Qt::Horizontal, pageWidget);
			splitter->setChildrenCollapsible(false);

			QGroupBox* listGroup = new QGroupBox("控制单元列表", splitter);
			QVBoxLayout* listLayout = new QVBoxLayout(listGroup);
			listLayout->setSpacing(6);
			m_unitTable = new QTableWidget(listGroup);
			m_unitTable->setColumnCount(11);
			m_unitTable->setHorizontalHeaderLabels(QStringList()
				<< "序号" << "启用" << "内部名" << "中文名" << "工件"
				<< "类型" << "相机" << "手眼" << "IP" << "端口" << "FTP");
			m_unitTable->horizontalHeader()->setStretchLastSection(true);
			m_unitTable->setSelectionBehavior(QAbstractItemView::SelectRows);
			m_unitTable->setSelectionMode(QAbstractItemView::SingleSelection);
			m_unitTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
			listLayout->addWidget(m_unitTable, 1);

			QHBoxLayout* listButtons = new QHBoxLayout();
			QPushButton* newBtn = new QPushButton("新建", listGroup);
			QPushButton* copyBtn = new QPushButton("复制选中", listGroup);
			QPushButton* refreshBtn = new QPushButton("刷新", listGroup);
			for (QPushButton* button : { newBtn, copyBtn, refreshBtn })
			{
				button->setFixedWidth(120);
				listButtons->addWidget(button);
			}
			listButtons->addStretch(1);
			listLayout->addLayout(listButtons);

			QGroupBox* editGroup = new QGroupBox("编辑控制单元", splitter);
			QVBoxLayout* editLayout = new QVBoxLayout(editGroup);
			editLayout->setSpacing(6);
			QFormLayout* form = new QFormLayout();
			form->setFieldGrowthPolicy(QFormLayout::FieldsStayAtSizeHint);
			form->setLabelAlignment(Qt::AlignRight | Qt::AlignVCenter);
			form->setHorizontalSpacing(8);
			form->setVerticalSpacing(4);
			m_editorForm = form;

			m_unitNameEdit = new QLineEdit(editGroup);
			m_chineseNameEdit = new QLineEdit(editGroup);
			m_customNameEdit = new QLineEdit(editGroup);
			m_enabledCheck = new QCheckBox("启用该控制单元", editGroup);
			m_workpieceTypeCombo = new QComboBox(editGroup);
			m_workpieceTypeCombo->addItem("波纹板", kWorkpieceCorrugatedPlate);
			m_cameraReadyCheck = new QCheckBox("相机参数已完成", editGroup);
			m_handEyeReadyCheck = new QCheckBox("手眼标定已完成", editGroup);
			m_robotTypeCombo = new QComboBox(editGroup);
			m_robotTypeCombo->addItem("FANUC", ROBOT_TYPE_FANUC);
			m_robotTypeCombo->addItem("STEP", ROBOT_TYPE_STEP);
			m_unitTypeEdit = new QLineEdit(editGroup);
			m_socketIpEdit = new IpAddressEdit(editGroup);
			m_socketPortEdit = new QLineEdit(editGroup);
			m_monitorPortEdit = new QLineEdit(editGroup);
			m_ftpIpEdit = new IpAddressEdit(editGroup);
			m_ftpPortEdit = new QLineEdit(editGroup);
			m_ftpUserEdit = new QLineEdit(editGroup);
			m_ftpPasswordEdit = new QLineEdit(editGroup);
			m_stepProjectEdit = new QLineEdit(editGroup);

			m_socketIpEdit->setFixedWidth(228);
			m_ftpIpEdit->setFixedWidth(228);
			for (QLineEdit* edit : { m_unitNameEdit, m_chineseNameEdit, m_customNameEdit, m_ftpUserEdit, m_ftpPasswordEdit, m_stepProjectEdit })
			{
				edit->setFixedWidth(260);
			}
			for (QLineEdit* edit : { m_unitTypeEdit, m_socketPortEdit, m_monitorPortEdit, m_ftpPortEdit })
			{
				edit->setFixedWidth(120);
				edit->setAlignment(Qt::AlignRight);
				MarkNumericEditGlobal(edit);
			}
			m_unitTypeEdit->setValidator(new QIntValidator(0, 9999, m_unitTypeEdit));
			m_socketPortEdit->setValidator(new QIntValidator(0, 65535, m_socketPortEdit));
			m_monitorPortEdit->setValidator(new QIntValidator(0, 65535, m_monitorPortEdit));
			m_ftpPortEdit->setValidator(new QIntValidator(0, 65535, m_ftpPortEdit));
			m_robotTypeCombo->setFixedWidth(160);
			m_workpieceTypeCombo->setFixedWidth(160);

			form->addRow("内部名", m_unitNameEdit);
			form->addRow("中文名", m_chineseNameEdit);
			form->addRow("显示名", m_customNameEdit);
			form->addRow("启用", m_enabledCheck);
			form->addRow("工件类型", m_workpieceTypeCombo);
			form->addRow("设置状态", m_cameraReadyCheck);
			form->addRow(QString(), m_handEyeReadyCheck);
			form->addRow("机器人类型", m_robotTypeCombo);
			form->addRow("UnitType", m_unitTypeEdit);
			form->addRow("Socket IP", m_socketIpEdit);
			form->addRow("Socket端口", m_socketPortEdit);
			form->addRow("监控端口", m_monitorPortEdit);
			form->addRow("FTP IP", m_ftpIpEdit);
			form->addRow("FTP端口", m_ftpPortEdit);
			form->addRow("FTP用户", m_ftpUserEdit);
			form->addRow("FTP密码", m_ftpPasswordEdit);
			form->addRow("STEP工程名", m_stepProjectEdit);
			editLayout->addLayout(form);

			QHBoxLayout* editButtons = new QHBoxLayout();
			QPushButton* saveBtn = new QPushButton("保存配置", editGroup);
			QPushButton* saveReloadBtn = new QPushButton("保存并重载", editGroup);
			QPushButton* reloadBtn = new QPushButton("只重载", editGroup);
			QPushButton* closeBtn = new QPushButton("关闭", editGroup);
			for (QPushButton* button : { saveBtn, saveReloadBtn, reloadBtn, closeBtn })
			{
				button->setFixedWidth(130);
				editButtons->addWidget(button);
			}
			editButtons->addStretch(1);
			editLayout->addLayout(editButtons);

			m_logText = new QPlainTextEdit(editGroup);
			m_logText->setReadOnly(true);
			m_logText->document()->setMaximumBlockCount(300);
			m_logText->setMaximumHeight(150);
			editLayout->addWidget(m_logText, 1);

			splitter->addWidget(listGroup);
			splitter->addWidget(editGroup);
			splitter->setStretchFactor(0, 3);
			splitter->setStretchFactor(1, 2);
			rootLayout->addWidget(splitter, 1);

			connect(m_unitTable, &QTableWidget::itemSelectionChanged, this, [this]() { SyncEditorFromSelection(); });
			connect(newBtn, &QPushButton::clicked, this, [this]() { PrepareNewUnit(false); });
			connect(copyBtn, &QPushButton::clicked, this, [this]() { PrepareNewUnit(true); });
			connect(refreshBtn, &QPushButton::clicked, this, [this]() { LoadUnits(true); });
			connect(saveBtn, &QPushButton::clicked, this, [this]() { SaveCurrent(false); });
			connect(saveReloadBtn, &QPushButton::clicked, this, [this]() { SaveCurrent(true); });
			connect(reloadBtn, &QPushButton::clicked, this, [this]() { ReloadControlUnits(); });
			connect(closeBtn, &QPushButton::clicked, this, &QDialog::accept);
			ConnectIpAddressEdited(m_socketIpEdit, this, [this]()
				{
					SyncEditorFtpIpWithSocketIp();
				});
			connect(m_robotTypeCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, [this]()
				{
					ApplyFtpCredentialForRobotType(
						m_robotTypeCombo->currentData().toInt(),
						m_ftpUserEdit,
						m_ftpPasswordEdit,
						true);
					ApplyEditorRobotTypeUi();
					SyncEditorFtpIpWithSocketIp();
				});

			LoadUnits(false);
		}

	private:
		struct UnitConfig
		{
			int unitNo = -1;
			QString unitName;
			QString chineseName;
			QString controlType = "R";
			int unitType = 0;
			int robotType = ROBOT_TYPE_FANUC;
			QString customName;
			QString socketIP;
			int socketPort = 0;
			int monitorPort = 0;
			QString ftpIP;
			int ftpPort = 21;
			QString ftpUser;
			QString ftpPassword;
			QString stepProjectName;
			bool enabled = true;
			QString workpieceType = kWorkpieceCorrugatedPlate;
			bool cameraParamReady = true;
			bool handEyeReady = true;
		};

		struct FtpCredential
		{
			QString user;
			QString password;
		};

		static QString ControlInfoPath()
		{
			return RobotDataHelper::BuildProjectPath("Data/ContralUnitInfo.ini");
		}

		static QString RobotParaPath(const QString& unitName)
		{
			return RobotDataHelper::BuildProjectPath(QString("Data/%1/RobotPara.ini").arg(unitName));
		}

		static std::string ToIniBytes(const QString& text)
		{
			return text.toLocal8Bit().toStdString();
		}

		static QString ReadIniString(COPini& ini, const QString& key, const QString& fallback = QString())
		{
			std::string rawValue;
			if (ini.ReadString(false, ToIniBytes(key), rawValue) > 0)
			{
				return DecodeConfigText(rawValue);
			}
			return fallback;
		}

		static int ReadIniInt(COPini& ini, const QString& key, int fallback = 0)
		{
			int value = fallback;
			if (ini.ReadString(false, ToIniBytes(key), &value) > 0)
			{
				return value;
			}
			return fallback;
		}

		static FtpCredential DefaultFtpCredentialForRobotType(int robotType)
		{
			const QString templateRobot = robotType == ROBOT_TYPE_STEP ? "RobotB" : "RobotA";
			FtpCredential credential;
			credential.user = robotType == ROBOT_TYPE_STEP ? "root" : "anonymous";
			credential.password = robotType == ROBOT_TYPE_STEP ? "STEP_ROBOT_SRH" : QString();

			COPini robotIni;
			if (robotIni.SetFileName(ToIniBytes(RobotParaPath(templateRobot))))
			{
				robotIni.SetSectionName("BaseParam");
				credential.user = ReadIniString(robotIni, "FTPUser", credential.user);
				credential.password = ReadIniString(robotIni, "FTPPassWord", credential.password);
			}
			return credential;
		}

		static void ApplyFtpCredentialForRobotType(
			int robotType,
			QLineEdit* userEdit,
			QLineEdit* passwordEdit,
			bool forceOverwrite)
		{
			const FtpCredential credential = DefaultFtpCredentialForRobotType(robotType);
			if (userEdit != nullptr && (forceOverwrite || userEdit->text().trimmed().isEmpty()))
			{
				userEdit->setText(credential.user);
			}
			if (passwordEdit != nullptr && (forceOverwrite || passwordEdit->text().isEmpty()))
			{
				passwordEdit->setText(credential.password);
			}
		}

		static bool WriteIniString(COPini& ini, const QString& key, const QString& value)
		{
			return ini.WriteString(ToIniBytes(key), ToIniBytes(value));
		}

		static bool WriteIniInt(COPini& ini, const QString& key, int value)
		{
			return ini.WriteString(ToIniBytes(key), value);
		}

		void AppendLog(const QString& text)
		{
			if (m_logText != nullptr)
			{
				m_logText->appendPlainText(QString("[%1] %2")
					.arg(QDateTime::currentDateTime().toString("HH:mm:ss.zzz"), text));
			}
		}

		QList<UnitConfig> ReadUnits(QString* error = nullptr) const
		{
			QList<UnitConfig> units;
			QHash<QString, int> unitRowByName;
			COPini ini;
			if (!ini.SetFileName(ToIniBytes(ControlInfoPath())))
			{
				if (error != nullptr)
				{
					*error = "打开控制单元配置失败：" + ControlInfoPath();
				}
				return units;
			}

			ini.SetSectionName("UnitNum");
			const int unitCount = ReadIniInt(ini, "UnitNum", 0);
			for (int index = 0; index < unitCount; ++index)
			{
				UnitConfig unit;
				unit.unitNo = index;
				const QString key = QString("Unit%1").arg(index);
				ini.SetSectionName("UnitName");
				unit.unitName = ReadIniString(ini, key);
				ini.SetSectionName("ChineseName");
				unit.chineseName = ReadIniString(ini, key);
				ini.SetSectionName("ContralType");
				unit.controlType = ReadIniString(ini, key, "R");
				ini.SetSectionName("UnitType");
				unit.unitType = ReadIniInt(ini, key, 0);
				LoadRobotPara(unit);
				unit.enabled = true;
				unitRowByName.insert(unit.unitName.toLower(), units.size());
				units.push_back(unit);
			}

			QDir dataDir(RobotDataHelper::BuildProjectPath("Data"));
			const QFileInfoList robotDirs = dataDir.entryInfoList(QDir::Dirs | QDir::NoDotAndDotDot, QDir::Name);
			for (const QFileInfo& dirInfo : robotDirs)
			{
				const QString unitName = dirInfo.fileName();
				if (!QFileInfo::exists(dirInfo.filePath() + "/RobotPara.ini"))
				{
					continue;
				}
				const QString key = unitName.toLower();
				if (unitRowByName.contains(key))
				{
					UnitConfig& unit = units[unitRowByName.value(key)];
					LoadRobotPara(unit);
					continue;
				}

				UnitConfig unit;
				unit.unitNo = -1;
				unit.unitName = unitName;
				unit.chineseName = unitName;
				LoadRobotPara(unit);
				if (!unit.customName.isEmpty() && unit.chineseName == unitName)
				{
					unit.chineseName = unit.customName;
				}
				unitRowByName.insert(key, units.size());
				units.push_back(unit);
			}
			return units;
		}

		void LoadRobotPara(UnitConfig& unit) const
		{
			if (unit.unitName.isEmpty())
			{
				return;
			}
			COPini robotIni;
			if (!robotIni.SetFileName(ToIniBytes(RobotParaPath(unit.unitName))))
			{
				return;
			}
			robotIni.SetSectionName("BaseParam");
			unit.customName = ReadIniString(robotIni, "CustomName");
			unit.robotType = ReadIniInt(robotIni, "RobotType", unit.robotType);
			unit.socketIP = ReadIniString(robotIni, "SocketIP");
			unit.socketPort = ReadIniInt(robotIni, "SocketPort", 0);
			unit.monitorPort = ReadIniInt(robotIni, "MonitorPort", 0);
			unit.ftpIP = ReadIniString(robotIni, "FTPIP");
			unit.ftpPort = ReadIniInt(robotIni, "FTPPort", 21);
			unit.ftpUser = ReadIniString(robotIni, "FTPUser");
			unit.ftpPassword = ReadIniString(robotIni, "FTPPassWord");
			unit.stepProjectName = ReadIniString(robotIni, "StepProjectName");
			if (unit.stepProjectName.isEmpty())
			{
				unit.stepProjectName = ReadIniString(robotIni, "ProjectName");
			}
			robotIni.SetSectionName("SetupStatus");
			unit.enabled = ReadIniInt(robotIni, "Enabled", unit.enabled ? 1 : 0) != 0;
			unit.workpieceType = ReadIniString(robotIni, "WorkpieceType", unit.workpieceType).trimmed();
			if (unit.workpieceType.isEmpty())
			{
				unit.workpieceType = kWorkpieceCorrugatedPlate;
			}
			unit.cameraParamReady = ReadIniInt(robotIni, "CameraParamReady", unit.cameraParamReady ? 1 : 0) != 0;
			unit.handEyeReady = ReadIniInt(robotIni, "HandEyeReady", unit.handEyeReady ? 1 : 0) != 0;
		}

		void LoadUnits(bool keepSelection)
		{
			const QString previousUnitName = SelectedUnitName();
			QString error;
			m_units = ReadUnits(&error);
			if (!error.isEmpty())
			{
				QMessageBox::warning(this, "控制单元管理", error);
			}
			RefreshTable();
			if (keepSelection)
			{
				SelectUnit(previousUnitName);
			}
			else if (!m_units.isEmpty())
			{
				m_unitTable->selectRow(0);
			}
			AppendLog(QString("已读取控制单元：%1 个。").arg(m_units.size()));
		}

		void RefreshTable()
		{
			if (m_unitTable == nullptr)
			{
				return;
			}
			QSignalBlocker blocker(m_unitTable);
			m_unitTable->setRowCount(0);
			for (int row = 0; row < m_units.size(); ++row)
			{
				const UnitConfig& unit = m_units.at(row);
				m_unitTable->insertRow(row);
				auto setItem = [this, row](int column, const QString& text)
					{
						QTableWidgetItem* item = new QTableWidgetItem(text);
						item->setData(Qt::UserRole, row);
						m_unitTable->setItem(row, column, item);
					};
				setItem(0, unit.unitNo >= 0 ? QString::number(unit.unitNo) : "-");
				setItem(1, unit.enabled ? "启用" : "停用");
				setItem(2, unit.unitName);
				setItem(3, unit.chineseName);
				setItem(4, WorkpieceDisplayName(unit.workpieceType));
				setItem(5, unit.robotType == ROBOT_TYPE_STEP ? "STEP" : "FANUC");
				setItem(6, unit.cameraParamReady ? "已完成" : "未完成");
				setItem(7, unit.handEyeReady ? "已完成" : "未完成");
				setItem(8, unit.socketIP);
				setItem(9, QString::number(unit.socketPort));
				setItem(10, QString("%1:%2").arg(unit.ftpIP).arg(unit.ftpPort));
			}
			m_unitTable->resizeColumnsToContents();
		}

		QString SelectedUnitName() const
		{
			if (m_unitTable == nullptr || m_unitTable->currentRow() < 0)
			{
				return QString();
			}
			QTableWidgetItem* item = m_unitTable->item(m_unitTable->currentRow(), 2);
			return item != nullptr ? item->text().trimmed() : QString();
		}

		void SelectUnit(const QString& unitName)
		{
			if (unitName.isEmpty() || m_unitTable == nullptr)
			{
				return;
			}
			for (int row = 0; row < m_unitTable->rowCount(); ++row)
			{
				QTableWidgetItem* item = m_unitTable->item(row, 2);
				if (item != nullptr && item->text() == unitName)
				{
					m_unitTable->selectRow(row);
					return;
				}
			}
		}

		void SyncEditorFromSelection()
		{
			if (m_unitTable == nullptr || m_unitTable->currentRow() < 0)
			{
				return;
			}
			QTableWidgetItem* indexItem = m_unitTable->item(m_unitTable->currentRow(), 0);
			const int unitRow = indexItem != nullptr ? indexItem->data(Qt::UserRole).toInt() : -1;
			if (unitRow < 0 || unitRow >= m_units.size())
			{
				return;
			}
			m_editingRow = unitRow;
			FillEditor(m_units.at(unitRow), true);
		}

		void FillEditor(const UnitConfig& unit, bool existing)
		{
			m_unitNameEdit->setText(unit.unitName);
			m_unitNameEdit->setReadOnly(existing);
			m_chineseNameEdit->setText(unit.chineseName);
			m_customNameEdit->setText(unit.customName);
			m_enabledCheck->setChecked(unit.enabled);
			const int workpieceIndex = m_workpieceTypeCombo->findData(unit.workpieceType);
			m_workpieceTypeCombo->setCurrentIndex(workpieceIndex >= 0 ? workpieceIndex : 0);
			m_cameraReadyCheck->setChecked(unit.cameraParamReady);
			m_handEyeReadyCheck->setChecked(unit.handEyeReady);
			const int robotTypeIndex = m_robotTypeCombo->findData(unit.robotType);
			m_robotTypeCombo->setCurrentIndex(robotTypeIndex >= 0 ? robotTypeIndex : 0);
			m_unitTypeEdit->setText(QString::number(unit.unitType));
			m_socketIpEdit->setText(unit.socketIP);
			m_socketPortEdit->setText(QString::number(unit.socketPort));
			m_monitorPortEdit->setText(unit.monitorPort > 0 ? QString::number(unit.monitorPort) : QString());
			m_ftpIpEdit->setText(unit.ftpIP.isEmpty() ? unit.socketIP : unit.ftpIP);
			m_ftpPortEdit->setText(QString::number(unit.ftpPort));
			m_ftpUserEdit->setText(unit.ftpUser);
			m_ftpPasswordEdit->setText(unit.ftpPassword);
			m_stepProjectEdit->setText(unit.stepProjectName);
			m_lastEditorSocketIpForFtp = unit.socketIP;
			ApplyEditorRobotTypeUi();
		}

		void ApplyEditorRobotTypeUi()
		{
			const bool isStep = m_robotTypeCombo != nullptr
				&& m_robotTypeCombo->currentData().toInt() == ROBOT_TYPE_STEP;
			SetFormRowVisible(m_editorForm, m_monitorPortEdit, !isStep);
			SetFormRowVisible(m_editorForm, m_stepProjectEdit, isStep);
		}

		void SyncEditorFtpIpWithSocketIp()
		{
			if (m_socketIpEdit == nullptr || m_ftpIpEdit == nullptr)
			{
				return;
			}
			const QString socketIp = m_socketIpEdit->text().trimmed();
			const QString ftpIp = m_ftpIpEdit->text().trimmed();
			if (!socketIp.isEmpty() && (ftpIp.isEmpty() || ftpIp == m_lastEditorSocketIpForFtp))
			{
				m_ftpIpEdit->setText(socketIp);
			}
			m_lastEditorSocketIpForFtp = socketIp;
		}

		void PrepareNewUnit(bool copySelected)
		{
			UnitConfig unit;
			if (copySelected && m_editingRow >= 0 && m_editingRow < m_units.size())
			{
				unit = m_units.at(m_editingRow);
				unit.chineseName = unit.chineseName.isEmpty() ? "新机器人" : unit.chineseName + "副本";
			}
			unit.unitNo = -1;
			unit.unitName = GenerateNextUnitName();
			if (unit.chineseName.isEmpty())
			{
				unit.chineseName = QString("焊接机器人%1").arg(m_units.size() + 1);
			}
			if (unit.customName.isEmpty())
			{
				unit.customName = unit.chineseName;
			}
			if (!copySelected)
			{
				unit.robotType = ROBOT_TYPE_FANUC;
				unit.unitType = 0;
				unit.socketPort = 9000;
				unit.monitorPort = 9001;
				unit.ftpPort = 21;
				const FtpCredential ftpCredential = DefaultFtpCredentialForRobotType(unit.robotType);
				unit.ftpUser = ftpCredential.user;
				unit.ftpPassword = ftpCredential.password;
				unit.cameraParamReady = false;
				unit.handEyeReady = false;
			}
			if (!RunNewUnitWizard(unit, copySelected))
			{
				AppendLog("已取消新建控制单元向导。");
				return;
			}

			for (const UnitConfig& existingUnit : m_units)
			{
				if (existingUnit.unitName.compare(unit.unitName, Qt::CaseInsensitive) == 0)
				{
					QMessageBox::warning(this, "控制单元管理", "内部名已存在，新增控制单元请换一个内部名。");
					return;
				}
			}

			QList<UnitConfig> nextUnits = m_units;
			unit.unitNo = unit.enabled ? nextUnits.size() : -1;
			nextUnits.push_back(unit);

			QString error;
			if (!WriteControlInfo(nextUnits, error) || !WriteRobotPara(unit, true, error))
			{
				QMessageBox::warning(this, "控制单元管理", error);
				return;
			}

			NormalizeRuntimeUnitNumbers(nextUnits);
			m_units = nextUnits;
			RefreshTable();
			SelectUnit(unit.unitName);
			m_editingRow = m_units.size() - 1;
			if (m_editingRow >= 0 && m_editingRow < m_units.size())
			{
				FillEditor(m_units.at(m_editingRow), true);
			}
			AppendLog(copySelected ? "已创建复制控制单元，并刷新到列表。" : "已创建新控制单元，并刷新到列表。");
		}

		bool RunNewUnitWizard(UnitConfig& unit, bool copySelected)
		{
			QDialog wizard(this);
			wizard.setWindowTitle(copySelected ? "复制机器人控制单元向导" : "新建机器人控制单元向导");
			ApplyUnifiedWindowChrome(&wizard);
			wizard.resize(620, 460);
			wizard.setStyleSheet(styleSheet());

			QVBoxLayout* rootLayout = new QVBoxLayout(&wizard);
			rootLayout->setContentsMargins(18, 16, 18, 18);
			rootLayout->setSpacing(12);

			QLabel* titleLabel = new QLabel(copySelected ? "复制机器人控制单元" : "新建机器人控制单元", &wizard);
			titleLabel->setStyleSheet("font-size: 22px; font-weight: bold; color: #F7FCFC;");
			rootLayout->addWidget(titleLabel);

			QLabel* stepHintLabel = new QLabel(&wizard);
			stepHintLabel->setWordWrap(true);
			stepHintLabel->setStyleSheet("QLabel { color: #AFC8CE; font-size: 14px; }");
			rootLayout->addWidget(stepHintLabel);

			QStackedWidget* stack = new QStackedWidget(&wizard);
			stack->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
			rootLayout->addWidget(stack, 1);

			auto makePage = [&wizard]() -> QWidget*
				{
					QWidget* page = new QWidget(&wizard);
					QFormLayout* form = new QFormLayout(page);
					form->setFieldGrowthPolicy(QFormLayout::FieldsStayAtSizeHint);
					form->setLabelAlignment(Qt::AlignRight | Qt::AlignVCenter);
					form->setVerticalSpacing(14);
					page->setLayout(form);
					return page;
				};

			QWidget* basicPage = makePage();
			QFormLayout* basicForm = qobject_cast<QFormLayout*>(basicPage->layout());
			QLineEdit* unitNameEdit = new QLineEdit(basicPage);
			QLineEdit* chineseNameEdit = new QLineEdit(basicPage);
			QLineEdit* customNameEdit = new QLineEdit(basicPage);
			QComboBox* robotTypeCombo = new QComboBox(basicPage);
			robotTypeCombo->addItem("FANUC", ROBOT_TYPE_FANUC);
			robotTypeCombo->addItem("STEP", ROBOT_TYPE_STEP);
			QComboBox* workpieceCombo = new QComboBox(basicPage);
			workpieceCombo->addItem("波纹板", kWorkpieceCorrugatedPlate);
			for (QLineEdit* edit : { unitNameEdit, chineseNameEdit, customNameEdit })
			{
				edit->setFixedWidth(300);
			}
			robotTypeCombo->setFixedWidth(180);
			workpieceCombo->setFixedWidth(180);
			unitNameEdit->setText(unit.unitName);
			chineseNameEdit->setText(unit.chineseName);
			customNameEdit->setText(unit.customName);
			const int defaultTypeIndex = robotTypeCombo->findData(unit.robotType);
			robotTypeCombo->setCurrentIndex(defaultTypeIndex >= 0 ? defaultTypeIndex : 0);
			const int defaultWorkpieceIndex = workpieceCombo->findData(unit.workpieceType);
			workpieceCombo->setCurrentIndex(defaultWorkpieceIndex >= 0 ? defaultWorkpieceIndex : 0);
			basicForm->addRow("内部名", unitNameEdit);
			basicForm->addRow("中文名", chineseNameEdit);
			basicForm->addRow("显示名", customNameEdit);
			basicForm->addRow("机器人类型", robotTypeCombo);
			basicForm->addRow("工件类型", workpieceCombo);
			QLabel* basicTip = new QLabel("内部名会作为 Data 下的机器人目录名，例如 RobotC；保存后不建议再改。", basicPage);
			basicTip->setWordWrap(true);
			basicTip->setStyleSheet("QLabel { color: #8EB7BF; }");
			basicForm->addRow(QString(), basicTip);
			stack->addWidget(basicPage);

			QWidget* socketPage = makePage();
			QFormLayout* socketForm = qobject_cast<QFormLayout*>(socketPage->layout());
			IpAddressEdit* socketIpEdit = new IpAddressEdit(socketPage);
			QLineEdit* socketPortEdit = new QLineEdit(socketPage);
			QLineEdit* monitorPortEdit = new QLineEdit(socketPage);
			QLineEdit* unitTypeEdit = new QLineEdit(socketPage);
			socketIpEdit->setText(unit.socketIP);
			socketPortEdit->setText(unit.socketPort > 0 ? QString::number(unit.socketPort) : QString());
			monitorPortEdit->setText(unit.monitorPort > 0 ? QString::number(unit.monitorPort) : QString());
			unitTypeEdit->setText(QString::number(unit.unitType));
    socketIpEdit->setFixedWidth(228);
			for (QLineEdit* edit : { socketPortEdit, monitorPortEdit, unitTypeEdit })
			{
				edit->setFixedWidth(140);
				edit->setAlignment(Qt::AlignRight);
				MarkNumericEditGlobal(edit);
			}
			socketPortEdit->setValidator(new QIntValidator(0, 65535, socketPortEdit));
			monitorPortEdit->setValidator(new QIntValidator(0, 65535, monitorPortEdit));
			unitTypeEdit->setValidator(new QIntValidator(0, 9999, unitTypeEdit));
			socketForm->addRow("Socket IP", socketIpEdit);
			socketForm->addRow("Socket端口", socketPortEdit);
			socketForm->addRow("监控端口", monitorPortEdit);
			socketForm->addRow("UnitType", unitTypeEdit);
			QLabel* socketTip = new QLabel("FANUC 默认 Socket=9000、监控=9001；STEP 默认 Socket=30312，监控端口可留空。", socketPage);
			socketTip->setWordWrap(true);
			socketTip->setStyleSheet("QLabel { color: #8EB7BF; }");
			socketForm->addRow(QString(), socketTip);
			stack->addWidget(socketPage);

			QWidget* ftpPage = makePage();
			QFormLayout* ftpForm = qobject_cast<QFormLayout*>(ftpPage->layout());
			IpAddressEdit* ftpIpEdit = new IpAddressEdit(ftpPage);
			QLineEdit* ftpPortEdit = new QLineEdit(ftpPage);
			QLineEdit* ftpUserEdit = new QLineEdit(ftpPage);
			QLineEdit* ftpPasswordEdit = new QLineEdit(ftpPage);
			QLineEdit* stepProjectEdit = new QLineEdit(ftpPage);
			ftpIpEdit->setText(unit.ftpIP.isEmpty() ? unit.socketIP : unit.ftpIP);
			ftpPortEdit->setText(unit.ftpPort > 0 ? QString::number(unit.ftpPort) : "21");
			ftpUserEdit->setText(unit.ftpUser);
			ftpPasswordEdit->setText(unit.ftpPassword);
			stepProjectEdit->setText(unit.stepProjectName);
    ftpIpEdit->setFixedWidth(228);
			for (QLineEdit* edit : { ftpUserEdit, ftpPasswordEdit, stepProjectEdit })
			{
				edit->setFixedWidth(300);
			}
			ftpPortEdit->setFixedWidth(140);
			ftpPortEdit->setAlignment(Qt::AlignRight);
			ftpPortEdit->setValidator(new QIntValidator(0, 65535, ftpPortEdit));
			MarkNumericEditGlobal(ftpPortEdit);
			ftpForm->addRow("FTP IP", ftpIpEdit);
			ftpForm->addRow("FTP端口", ftpPortEdit);
			ftpForm->addRow("FTP用户", ftpUserEdit);
			ftpForm->addRow("FTP密码", ftpPasswordEdit);
			ftpForm->addRow("STEP工程名", stepProjectEdit);
			QLabel* ftpTip = new QLabel("STEP 工程名用于程序上传路径；FANUC 可以留空。向导完成后还需要点击“保存配置”写入文件。", ftpPage);
			ftpTip->setWordWrap(true);
			ftpTip->setStyleSheet("QLabel { color: #8EB7BF; }");
			ftpForm->addRow(QString(), ftpTip);
			stack->addWidget(ftpPage);

			QWidget* cameraSetupPage = makePage();
			QFormLayout* cameraSetupForm = qobject_cast<QFormLayout*>(cameraSetupPage->layout());
			QCheckBox* cameraReadyCheck = new QCheckBox("本次已完成相机基础参数设置", cameraSetupPage);
			cameraReadyCheck->setChecked(copySelected ? unit.cameraParamReady : false);
			QPushButton* openCameraBasicBtn = new QPushButton("打开相机基础参数界面", cameraSetupPage);
			openCameraBasicBtn->setFixedWidth(220);
			QLabel* cameraSetupTip = new QLabel(
				"可以在这里直接打开相机 IP、端口、曝光、增益等基础参数弹窗。若暂时跳过，相机预览和先测后焊等依赖相机的入口会保持禁用。",
				cameraSetupPage);
			cameraSetupTip->setWordWrap(true);
			cameraSetupTip->setStyleSheet("QLabel { color: #8EB7BF; }");
			cameraSetupForm->addRow("相机基础参数", openCameraBasicBtn);
			cameraSetupForm->addRow("完成状态", cameraReadyCheck);
			cameraSetupForm->addRow(QString(), cameraSetupTip);
			stack->addWidget(cameraSetupPage);

			QWidget* handEyeSetupPage = makePage();
			QFormLayout* handEyeSetupForm = qobject_cast<QFormLayout*>(handEyeSetupPage->layout());
			QCheckBox* handEyeReadyCheck = new QCheckBox("本次已完成手眼标定", handEyeSetupPage);
			handEyeReadyCheck->setChecked(copySelected ? unit.handEyeReady : false);
			QPushButton* openHandEyeCalibrationBtn = new QPushButton("打开手眼标定界面", handEyeSetupPage);
			openHandEyeCalibrationBtn->setFixedWidth(220);
			QLabel* handEyeSetupTip = new QLabel(
				"手眼标定可以先跳过。跳过后，先测后焊等依赖手眼矩阵的流程会禁用；完成标定并生成矩阵后会自动解除限制。",
				handEyeSetupPage);
			handEyeSetupTip->setWordWrap(true);
			handEyeSetupTip->setStyleSheet("QLabel { color: #8EB7BF; }");
			handEyeSetupForm->addRow("手眼标定", openHandEyeCalibrationBtn);
			handEyeSetupForm->addRow("完成状态", handEyeReadyCheck);
			handEyeSetupForm->addRow(QString(), handEyeSetupTip);
			stack->addWidget(handEyeSetupPage);

			bool typeDefaultsApplied = copySelected;
			int lastDefaultRobotType = robotTypeCombo->currentData().toInt();
			QString lastSocketIpForFtp = unit.socketIP;
			auto syncFtpIpWithSocketIp = [&]()
				{
					const QString socketIp = socketIpEdit->text().trimmed();
					const QString ftpIp = ftpIpEdit->text().trimmed();
					if (!socketIp.isEmpty() && (ftpIp.isEmpty() || ftpIp == lastSocketIpForFtp))
					{
						ftpIpEdit->setText(socketIp);
					}
					lastSocketIpForFtp = socketIp;
				};
			auto updateTypeSpecificFields = [&]()
				{
					const bool isStep = robotTypeCombo->currentData().toInt() == ROBOT_TYPE_STEP;
					SetFormRowVisible(socketForm, monitorPortEdit, !isStep);
					SetFormRowVisible(ftpForm, stepProjectEdit, isStep);
					socketTip->setText(isStep
						? "STEP 默认 Socket=30312；该型号没有监控端口，界面会自动隐藏。"
						: "FANUC 默认 Socket=9000、监控=9001。");
					ftpTip->setText(isStep
						? "STEP 工程名用于程序上传路径，默认 PCRobot。FTP IP 默认跟随机器人 Socket IP。"
						: "FANUC 不需要 STEP 工程名，FTP IP 默认跟随机器人 Socket IP。向导完成后还需要点击“保存配置”写入文件。");
				};
			ConnectIpAddressEdited(socketIpEdit, &wizard, syncFtpIpWithSocketIp);
			auto applyTypeDefaults = [&]()
				{
					const int robotType = robotTypeCombo->currentData().toInt();
					const bool forceFtpDefaults = !typeDefaultsApplied || robotType != lastDefaultRobotType;
					if (robotType == ROBOT_TYPE_STEP)
					{
						if (socketPortEdit->text().trimmed().isEmpty() || !typeDefaultsApplied)
						{
							socketPortEdit->setText("30312");
						}
						if (monitorPortEdit->text().trimmed() == "9001" || !typeDefaultsApplied)
						{
							monitorPortEdit->clear();
						}
					}
					else
					{
						if (socketPortEdit->text().trimmed().isEmpty() || !typeDefaultsApplied)
						{
							socketPortEdit->setText("9000");
						}
						if (monitorPortEdit->text().trimmed().isEmpty() || !typeDefaultsApplied)
						{
							monitorPortEdit->setText("9001");
						}
					}
					ApplyFtpCredentialForRobotType(robotType, ftpUserEdit, ftpPasswordEdit, forceFtpDefaults);
					if (ftpPortEdit->text().trimmed().isEmpty())
					{
						ftpPortEdit->setText("21");
					}
					updateTypeSpecificFields();
					syncFtpIpWithSocketIp();
					typeDefaultsApplied = true;
					lastDefaultRobotType = robotType;
				};
			applyTypeDefaults();
			connect(robotTypeCombo, &QComboBox::currentIndexChanged, &wizard, [applyTypeDefaults]() mutable { applyTypeDefaults(); });

			QHBoxLayout* buttonLayout = new QHBoxLayout();
			QPushButton* prevBtn = new QPushButton("上一步", &wizard);
			QPushButton* nextBtn = new QPushButton("下一步", &wizard);
			QPushButton* cancelBtn = new QPushButton("取消", &wizard);
			for (QPushButton* button : { prevBtn, nextBtn, cancelBtn })
			{
				button->setFixedWidth(120);
			}
			buttonLayout->addStretch(1);
			buttonLayout->addWidget(prevBtn);
			buttonLayout->addWidget(nextBtn);
			buttonLayout->addWidget(cancelBtn);
			rootLayout->addLayout(buttonLayout);

			auto setStep = [&]()
				{
					const int index = stack->currentIndex();
					prevBtn->setEnabled(index > 0);
					nextBtn->setText(index == stack->count() - 1 ? "完成" : "下一步");
					const QStringList hints = {
						"第 1 步 / 5：设置控制单元名称、机器人类型和工件类型。",
						"第 2 步 / 5：设置机器人 Socket 通讯参数。",
						"第 3 步 / 5：设置 FTP 和 STEP 工程参数。",
						"第 4 步 / 5：是否打开相机基础参数界面；跳过后相机相关功能会暂时禁用。",
						"第 5 步 / 5：是否打开手眼标定界面；跳过后先测后焊等流程会暂时禁用。"
					};
					stepHintLabel->setText(hints.value(index));
				};

			auto validatePort = [this](const QString& text, const QString& title, bool allowEmpty, int& value) -> bool
				{
					const QString trimmed = text.trimmed();
					if (allowEmpty && trimmed.isEmpty())
					{
						value = 0;
						return true;
					}
					bool ok = false;
					value = trimmed.toInt(&ok);
					if (!ok || value <= 0 || value > 65535)
					{
						QMessageBox::warning(this, "新建控制单元向导", QString("%1 必须在 1 到 65535 之间。").arg(title));
						return false;
					}
					return true;
				};

			auto validateCurrentPage = [&]() -> bool
				{
					if (stack->currentIndex() == 0)
					{
						const QString unitName = unitNameEdit->text().trimmed();
						const QRegularExpression unitNamePattern("^[A-Za-z0-9_\\-]+$");
						if (unitName.isEmpty() || !unitNamePattern.match(unitName).hasMatch())
						{
							QMessageBox::warning(&wizard, "新建控制单元向导", "内部名不能为空，只能使用字母、数字、下划线或中划线，例如 RobotC。");
							return false;
						}
						for (const UnitConfig& existingUnit : m_units)
						{
							if (existingUnit.unitName.compare(unitName, Qt::CaseInsensitive) == 0)
							{
								QMessageBox::warning(&wizard, "新建控制单元向导", "内部名已存在，请换一个名称。");
								return false;
							}
						}
						if (chineseNameEdit->text().trimmed().isEmpty())
						{
							QMessageBox::warning(&wizard, "新建控制单元向导", "中文名不能为空。");
							return false;
						}
						return true;
					}
					if (stack->currentIndex() == 1)
					{
						if (socketIpEdit->isEmpty())
						{
							QMessageBox::warning(&wizard, "新建控制单元向导", "Socket IP 不能为空。");
							return false;
						}
						if (!socketIpEdit->isComplete())
						{
							QMessageBox::warning(&wizard, "新建控制单元向导", "Socket IP 必须是 4 段 0-255 的数字。");
							return false;
						}
						int ignored = 0;
						if (!validatePort(socketPortEdit->text(), "Socket端口", false, ignored))
						{
							return false;
						}
						if (robotTypeCombo->currentData().toInt() != ROBOT_TYPE_STEP
							&& !validatePort(monitorPortEdit->text(), "监控端口", true, ignored))
						{
							return false;
						}
						bool ok = false;
						unitTypeEdit->text().trimmed().toInt(&ok);
						if (!ok)
						{
							QMessageBox::warning(&wizard, "新建控制单元向导", "UnitType 必须是整数。");
							return false;
						}
						return true;
					}
					if (stack->currentIndex() == 2 && ftpIpEdit->isEmpty())
					{
						QMessageBox::warning(&wizard, "新建控制单元向导", "FTP IP 不能为空。");
						return false;
					}
					if (stack->currentIndex() == 2 && !ftpIpEdit->isComplete())
					{
						QMessageBox::warning(&wizard, "新建控制单元向导", "FTP IP 必须是 4 段 0-255 的数字。");
						return false;
					}
					if (stack->currentIndex() != 2)
					{
						return true;
					}
					int ignored = 0;
					return validatePort(ftpPortEdit->text(), "FTP端口", false, ignored);
				};

			auto buildWizardDraft = [&]() -> UnitConfig
				{
					UnitConfig draft = unit;
					draft.unitName = unitNameEdit->text().trimmed();
					draft.chineseName = chineseNameEdit->text().trimmed();
					draft.customName = customNameEdit->text().trimmed();
					if (draft.customName.isEmpty())
					{
						draft.customName = draft.chineseName;
					}
					draft.robotType = robotTypeCombo->currentData().toInt();
					draft.workpieceType = workpieceCombo->currentData().toString();
					draft.enabled = true;
					draft.cameraParamReady = cameraReadyCheck->isChecked();
					draft.handEyeReady = handEyeReadyCheck->isChecked();
					draft.unitType = unitTypeEdit->text().trimmed().toInt();
					draft.socketIP = socketIpEdit->text().trimmed();
					draft.socketPort = socketPortEdit->text().trimmed().toInt();
					draft.monitorPort = draft.robotType == ROBOT_TYPE_STEP ? 0 : monitorPortEdit->text().trimmed().toInt();
					draft.ftpIP = ftpIpEdit->text().trimmed();
					draft.ftpPort = ftpPortEdit->text().trimmed().toInt();
					draft.ftpUser = ftpUserEdit->text().trimmed();
					draft.ftpPassword = ftpPasswordEdit->text();
					draft.stepProjectName = draft.robotType == ROBOT_TYPE_STEP ? stepProjectEdit->text().trimmed() : QString();
					return draft;
				};

			auto prepareWizardRobotFiles = [&](const UnitConfig& draft) -> bool
				{
					QString error;
					if (!WriteRobotPara(draft, true, error))
					{
						QMessageBox::warning(&wizard, "新建控制单元向导", error);
						return false;
					}
					return true;
				};

			connect(openCameraBasicBtn, &QPushButton::clicked, &wizard, [&]()
				{
					UnitConfig draft = buildWizardDraft();
					if (!prepareWizardRobotFiles(draft))
					{
						return;
					}
					const QString cameraSection = RobotDataHelper::MeasureCameraSection(draft.unitName);
					if (m_openCameraBasicParam && m_openCameraBasicParam(draft.unitName, cameraSection))
					{
						cameraReadyCheck->setChecked(true);
						AppendLog(QString("向导中已完成 %1 的相机基础参数。").arg(draft.unitName));
					}
				});

			connect(openHandEyeCalibrationBtn, &QPushButton::clicked, &wizard, [&]()
				{
					UnitConfig draft = buildWizardDraft();
					if (!prepareWizardRobotFiles(draft))
					{
						return;
					}
					const bool alreadyLoaded = std::any_of(m_units.cbegin(), m_units.cend(), [&draft](const UnitConfig& existingUnit)
						{
							return existingUnit.unitName.compare(draft.unitName, Qt::CaseInsensitive) == 0;
						});
					if (!alreadyLoaded)
					{
						QMessageBox::information(
							&wizard,
							"新建控制单元向导",
							"新控制单元尚未保存并重载，手眼标定界面可以先打开查看参数；实际采集前请完成向导、保存并重载控制单元。");
					}
					const QString cameraSection = RobotDataHelper::MeasureCameraSection(draft.unitName);
					if (m_openHandEyeCalibration && m_openHandEyeCalibration(draft.unitName, cameraSection))
					{
						handEyeReadyCheck->setChecked(true);
						AppendLog(QString("向导中已完成 %1 的手眼标定。").arg(draft.unitName));
					}
				});

			connect(prevBtn, &QPushButton::clicked, &wizard, [&]()
				{
					stack->setCurrentIndex(std::max(0, stack->currentIndex() - 1));
					setStep();
				});
			connect(nextBtn, &QPushButton::clicked, &wizard, [&]()
				{
					if (!validateCurrentPage())
					{
						return;
					}
					if (stack->currentIndex() < stack->count() - 1)
					{
						stack->setCurrentIndex(stack->currentIndex() + 1);
						setStep();
						return;
					}
					wizard.accept();
				});
			connect(cancelBtn, &QPushButton::clicked, &wizard, &QDialog::reject);
			setStep();

			if (wizard.exec() != QDialog::Accepted)
			{
				return false;
			}

			int parsedValue = 0;
			unit.unitName = unitNameEdit->text().trimmed();
			unit.chineseName = chineseNameEdit->text().trimmed();
			unit.customName = customNameEdit->text().trimmed();
			if (unit.customName.isEmpty())
			{
				unit.customName = unit.chineseName;
			}
			unit.robotType = robotTypeCombo->currentData().toInt();
			unit.workpieceType = workpieceCombo->currentData().toString();
			unit.enabled = true;
			unit.cameraParamReady = cameraReadyCheck->isChecked();
			unit.handEyeReady = handEyeReadyCheck->isChecked();
			unit.unitType = unitTypeEdit->text().trimmed().toInt();
			unit.socketIP = socketIpEdit->text().trimmed();
			unit.socketPort = socketPortEdit->text().trimmed().toInt();
			unit.monitorPort = 0;
			if (unit.robotType != ROBOT_TYPE_STEP
				&& validatePort(monitorPortEdit->text(), "监控端口", true, parsedValue))
			{
				unit.monitorPort = parsedValue;
			}
			unit.ftpIP = ftpIpEdit->text().trimmed();
			unit.ftpPort = ftpPortEdit->text().trimmed().toInt();
			unit.ftpUser = ftpUserEdit->text().trimmed();
			unit.ftpPassword = ftpPasswordEdit->text();
			unit.stepProjectName = unit.robotType == ROBOT_TYPE_STEP ? stepProjectEdit->text().trimmed() : QString();
			return true;
		}

		QString GenerateNextUnitName() const
		{
			QSet<QString> existingNames;
			for (const UnitConfig& unit : m_units)
			{
				existingNames.insert(unit.unitName.toLower());
			}
			for (ushort code = 'A'; code <= 'Z'; ++code)
			{
				const QString name = QString("Robot%1").arg(QChar(code));
				if (!existingNames.contains(name.toLower()))
				{
					return name;
				}
			}
			int index = m_units.size() + 1;
			while (existingNames.contains(QString("Robot%1").arg(index).toLower()))
			{
				++index;
			}
			return QString("Robot%1").arg(index);
		}

		bool CollectEditor(UnitConfig& unit, QString& error) const
		{
			unit.unitName = m_unitNameEdit->text().trimmed();
			unit.chineseName = m_chineseNameEdit->text().trimmed();
			unit.customName = m_customNameEdit->text().trimmed();
			unit.controlType = "R";
			unit.enabled = m_enabledCheck != nullptr && m_enabledCheck->isChecked();
			unit.workpieceType = m_workpieceTypeCombo != nullptr
				? m_workpieceTypeCombo->currentData().toString()
				: kWorkpieceCorrugatedPlate;
			unit.cameraParamReady = m_cameraReadyCheck != nullptr && m_cameraReadyCheck->isChecked();
			unit.handEyeReady = m_handEyeReadyCheck != nullptr && m_handEyeReadyCheck->isChecked();
			unit.robotType = m_robotTypeCombo->currentData().toInt();
			unit.socketIP = m_socketIpEdit->text().trimmed();
			unit.ftpIP = m_ftpIpEdit->text().trimmed();
			unit.ftpUser = m_ftpUserEdit->text().trimmed();
			unit.ftpPassword = m_ftpPasswordEdit->text();
			unit.stepProjectName = unit.robotType == ROBOT_TYPE_STEP ? m_stepProjectEdit->text().trimmed() : QString();
			if (unit.customName.isEmpty())
			{
				unit.customName = unit.chineseName;
			}

			const QRegularExpression unitNamePattern("^[A-Za-z0-9_\\-]+$");
			if (unit.unitName.isEmpty() || !unitNamePattern.match(unit.unitName).hasMatch())
			{
				error = "内部名不能为空，只能使用字母、数字、下划线或中划线，例如 RobotC。";
				return false;
			}
			if (unit.chineseName.isEmpty())
			{
				error = "中文名不能为空。";
				return false;
			}
			if (unit.socketIP.isEmpty())
			{
				error = "Socket IP 不能为空。";
				return false;
			}
			if (!m_socketIpEdit->isComplete())
			{
				error = "Socket IP 必须是 4 段 0-255 的数字。";
				return false;
			}
			if (unit.ftpIP.isEmpty())
			{
				error = "FTP IP 不能为空。";
				return false;
			}
			if (!m_ftpIpEdit->isComplete())
			{
				error = "FTP IP 必须是 4 段 0-255 的数字。";
				return false;
			}
			if (unit.workpieceType.isEmpty())
			{
				error = "工件类型不能为空。";
				return false;
			}

			bool ok = false;
			unit.unitType = m_unitTypeEdit->text().trimmed().toInt(&ok);
			if (!ok)
			{
				error = "UnitType 必须是整数。";
				return false;
			}
			unit.socketPort = m_socketPortEdit->text().trimmed().toInt(&ok);
			if (!ok || unit.socketPort <= 0 || unit.socketPort > 65535)
			{
				error = "Socket端口必须在 1 到 65535 之间。";
				return false;
			}
			const QString monitorPortText = m_monitorPortEdit->text().trimmed();
			unit.monitorPort = 0;
			if (unit.robotType != ROBOT_TYPE_STEP && !monitorPortText.isEmpty())
			{
				unit.monitorPort = monitorPortText.toInt(&ok);
				if (!ok || unit.monitorPort <= 0 || unit.monitorPort > 65535)
				{
					error = "监控端口必须为空或在 1 到 65535 之间。";
					return false;
				}
			}
			unit.ftpPort = m_ftpPortEdit->text().trimmed().toInt(&ok);
			if (!ok || unit.ftpPort <= 0 || unit.ftpPort > 65535)
			{
				error = "FTP端口必须在 1 到 65535 之间。";
				return false;
			}
			return true;
		}

		bool SaveCurrent(bool reloadAfterSave)
		{
			UnitConfig edited;
			QString error;
			if (!CollectEditor(edited, error))
			{
				QMessageBox::warning(this, "控制单元管理", error);
				return false;
			}

			QList<UnitConfig> nextUnits = m_units;
			const bool isNew = m_editingRow < 0 || m_editingRow >= nextUnits.size();
			if (isNew)
			{
				for (const UnitConfig& unit : nextUnits)
				{
					if (unit.unitName.compare(edited.unitName, Qt::CaseInsensitive) == 0)
					{
						QMessageBox::warning(this, "控制单元管理", "内部名已存在，新增控制单元请换一个内部名。");
						return false;
					}
				}
				edited.unitNo = edited.enabled ? nextUnits.size() : -1;
				nextUnits.push_back(edited);
			}
			else
			{
				edited.unitNo = nextUnits[m_editingRow].unitNo;
				edited.unitName = nextUnits[m_editingRow].unitName;
				nextUnits[m_editingRow] = edited;
			}

			if (!WriteControlInfo(nextUnits, error) || !WriteRobotPara(edited, isNew, error))
			{
				QMessageBox::warning(this, "控制单元管理", error);
				return false;
			}

			NormalizeRuntimeUnitNumbers(nextUnits);
			m_units = nextUnits;
			RefreshTable();
			SelectUnit(edited.unitName);
			AppendLog(QString("已保存 %1 的控制单元和机器人参数。").arg(edited.unitName));
			if (reloadAfterSave)
			{
				ReloadControlUnits();
			}
			return true;
		}

		bool WriteControlInfo(const QList<UnitConfig>& units, QString& error) const
		{
			COPini ini;
			if (!ini.SetFileName(false, ToIniBytes(ControlInfoPath())))
			{
				error = "打开控制单元配置失败：" + ControlInfoPath();
				return false;
			}
			ini.SetSectionName("UnitNum");
			const int enabledCount = std::count_if(units.cbegin(), units.cend(), [](const UnitConfig& unit) { return unit.enabled; });
			bool ok = WriteIniInt(ini, "UnitNum", enabledCount);
			int runtimeIndex = 0;
			for (int index = 0; index < units.size(); ++index)
			{
				const UnitConfig& unit = units.at(index);
				if (!unit.enabled)
				{
					continue;
				}
				const QString key = QString("Unit%1").arg(runtimeIndex++);
				ini.SetSectionName("UnitName");
				ok = ok && WriteIniString(ini, key, unit.unitName);
				ini.SetSectionName("ChineseName");
				ok = ok && WriteIniString(ini, key, unit.chineseName);
				ini.SetSectionName("ContralType");
				ok = ok && WriteIniString(ini, key, unit.controlType.isEmpty() ? "R" : unit.controlType);
				ini.SetSectionName("UnitType");
				ok = ok && WriteIniInt(ini, key, unit.unitType);
			}
			if (!ok)
			{
				error = "写入控制单元配置失败：" + ControlInfoPath();
			}
			return ok;
		}

		void NormalizeRuntimeUnitNumbers(QList<UnitConfig>& units) const
		{
			int runtimeIndex = 0;
			for (UnitConfig& unit : units)
			{
				unit.unitNo = unit.enabled ? runtimeIndex++ : -1;
			}
		}

		QString TemplateRobotParaPath(int robotType, const QString& targetUnitName) const
		{
			const QString preferredName = robotType == ROBOT_TYPE_STEP ? "RobotB" : "RobotA";
			const QString preferredPath = RobotParaPath(preferredName);
			if (preferredName.compare(targetUnitName, Qt::CaseInsensitive) != 0 && QFileInfo::exists(preferredPath))
			{
				return preferredPath;
			}
			for (const UnitConfig& unit : m_units)
			{
				if (unit.unitName.compare(targetUnitName, Qt::CaseInsensitive) != 0
					&& unit.robotType == robotType
					&& QFileInfo::exists(RobotParaPath(unit.unitName)))
				{
					return RobotParaPath(unit.unitName);
				}
			}
			return QString();
		}

		bool EnsureRobotParaFile(const UnitConfig& unit, bool isNew, QString& error) const
		{
			const QString targetPath = RobotParaPath(unit.unitName);
			QDir().mkpath(QFileInfo(targetPath).absolutePath());
			if (QFileInfo::exists(targetPath))
			{
				return true;
			}
			const QString templatePath = TemplateRobotParaPath(unit.robotType, unit.unitName);
			if (!templatePath.isEmpty() && QFile::copy(templatePath, targetPath))
			{
				return true;
			}
			QFile file(targetPath);
			if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
			{
				error = QString("创建机器人参数文件失败：%1").arg(QDir::toNativeSeparators(targetPath));
				return false;
			}
			QTextStream out(&file);
			out << "[BaseParam]\n\n[Tool]\n\n[Kinematics]\n\n[ExternalAxle]\n\n[ExternalAxleFuncation]\n";
			return true;
		}

		static QString WorkpieceTemplatePath(const QString& workpieceType)
		{
			return RobotDataHelper::BuildProjectPath(QString("Data/WorkpieceTemplates/%1").arg(workpieceType));
		}

		bool EnsureWorkpieceTemplateFiles(const UnitConfig& unit, bool isNew, QString& error) const
		{
			if (!isNew)
			{
				return true;
			}

			const QString templateDirPath = WorkpieceTemplatePath(unit.workpieceType);
			QDir templateDir(templateDirPath);
			if (!templateDir.exists())
			{
				error = QString("未找到工件默认参数模板：%1").arg(QDir::toNativeSeparators(templateDirPath));
				return false;
			}

			const QString targetDirPath = RobotDataHelper::BuildProjectPath(QString("Data/%1").arg(unit.unitName));
			QDir().mkpath(targetDirPath);
			const QStringList templateFiles = {
				"CameraParam.ini",
				"HandEyeCalibration_CAMERA1.ini",
				"HandEyeMatrix_CAMERA1.ini",
				"LineCoarseScanParam.ini",
				"MeasureWeldParam.ini",
				"WeldPoseCompParam.ini",
				"WeldSeamCompParam.ini",
				"WeaveDate.txt",
				"WeldPara.txt"
			};
			for (const QString& fileName : templateFiles)
			{
				const QString sourcePath = templateDir.filePath(fileName);
				if (!QFileInfo::exists(sourcePath))
				{
					error = QString("工件模板缺少参数文件：%1").arg(QDir::toNativeSeparators(sourcePath));
					return false;
				}
				const QString targetPath = QDir(targetDirPath).filePath(fileName);
				if (QFileInfo::exists(targetPath))
				{
					continue;
				}
				if (!QFile::copy(sourcePath, targetPath))
				{
					error = QString("复制工件模板失败：%1 -> %2")
						.arg(QDir::toNativeSeparators(sourcePath), QDir::toNativeSeparators(targetPath));
					return false;
				}
			}
			return true;
		}

		bool WriteRobotPara(const UnitConfig& unit, bool isNew, QString& error) const
		{
			if (!EnsureRobotParaFile(unit, isNew, error))
			{
				return false;
			}
			if (!EnsureWorkpieceTemplateFiles(unit, isNew, error))
			{
				return false;
			}
			COPini ini;
			const QString path = RobotParaPath(unit.unitName);
			if (!ini.SetFileName(false, ToIniBytes(path)))
			{
				error = "打开机器人参数文件失败：" + path;
				return false;
			}
			ini.SetSectionName("BaseParam");
			bool ok = true;
			ok = ok && WriteIniString(ini, "RobotName", unit.unitName);
			ok = ok && WriteIniString(ini, "CustomName", unit.customName);
			ok = ok && WriteIniInt(ini, "RobotType", unit.robotType);
			ok = ok && WriteIniString(ini, "SocketIP", unit.socketIP);
			ok = ok && WriteIniInt(ini, "SocketPort", unit.socketPort);
			if (unit.monitorPort > 0)
			{
				ok = ok && WriteIniInt(ini, "MonitorPort", unit.monitorPort);
			}
			ok = ok && WriteIniString(ini, "FTPIP", unit.ftpIP);
			ok = ok && WriteIniInt(ini, "FTPPort", unit.ftpPort);
			ok = ok && WriteIniString(ini, "FTPUser", unit.ftpUser);
			ok = ok && WriteIniString(ini, "FTPPassWord", unit.ftpPassword);
			if (unit.robotType == ROBOT_TYPE_STEP || !unit.stepProjectName.isEmpty())
			{
				ok = ok && WriteIniString(ini, "StepProjectName", unit.stepProjectName);
			}
			ini.SetSectionName("SetupStatus");
			ok = ok && WriteIniInt(ini, "Enabled", unit.enabled ? 1 : 0);
			ok = ok && WriteIniString(ini, "WorkpieceType", unit.workpieceType.isEmpty() ? kWorkpieceCorrugatedPlate : unit.workpieceType);
			ok = ok && WriteIniInt(ini, "CameraParamReady", unit.cameraParamReady ? 1 : 0);
			ok = ok && WriteIniInt(ini, "HandEyeReady", unit.handEyeReady ? 1 : 0);
			if (!ok)
			{
				error = "写入机器人参数文件失败：" + path;
			}
			return ok;
		}

		void ReloadControlUnits()
		{
			if (m_reloadCallback)
			{
				m_reloadCallback();
			}
			AppendLog("已请求重新加载控制单元，主界面机器人列表会同步刷新。");
			LoadUnits(true);
		}

	private:
		std::function<void()> m_reloadCallback;
		OpenCameraBasicParamFunc m_openCameraBasicParam;
		OpenHandEyeCalibrationFunc m_openHandEyeCalibration;
		QList<UnitConfig> m_units;
		int m_editingRow = -1;
		QTableWidget* m_unitTable = nullptr;
		QLineEdit* m_unitNameEdit = nullptr;
		QLineEdit* m_chineseNameEdit = nullptr;
		QLineEdit* m_customNameEdit = nullptr;
		QCheckBox* m_enabledCheck = nullptr;
		QComboBox* m_workpieceTypeCombo = nullptr;
		QCheckBox* m_cameraReadyCheck = nullptr;
		QCheckBox* m_handEyeReadyCheck = nullptr;
		QComboBox* m_robotTypeCombo = nullptr;
		QLineEdit* m_unitTypeEdit = nullptr;
		IpAddressEdit* m_socketIpEdit = nullptr;
		QLineEdit* m_socketPortEdit = nullptr;
		QLineEdit* m_monitorPortEdit = nullptr;
		IpAddressEdit* m_ftpIpEdit = nullptr;
		QLineEdit* m_ftpPortEdit = nullptr;
		QLineEdit* m_ftpUserEdit = nullptr;
		QLineEdit* m_ftpPasswordEdit = nullptr;
		QLineEdit* m_stepProjectEdit = nullptr;
		QFormLayout* m_editorForm = nullptr;
		QString m_lastEditorSocketIpForFtp;
		QPlainTextEdit* m_logText = nullptr;
	};

	class GroovePointCloudView final : public QWidget
	{
	public:
		struct ViewState
		{
			double baseMinX = -120.0;
			double baseMaxX = 120.0;
			double baseMinY = -600.0;
			double baseMaxY = -160.0;
			double zoomFactor = 1.0;
			QPointF panOffset;
			bool hasBaseBounds = false;
		};

		explicit GroovePointCloudView(QWidget* parent = nullptr)
			: QWidget(parent)
		{
			setFixedSize(960, 960);
			setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
		}

		QSize sizeHint() const override
		{
			return QSize(width(), height());
		}

		void SetFrame(const udpDataShow& frame, const QString& statusText, const ViewState& viewState)
		{
			m_profilePoints.clear();
			m_trendLines.clear();
			m_hasTargetPoint = false;
			m_fps = frame.mFps;
			m_timestamp = frame.timestamp;
			m_statusText = statusText;
			m_viewState = viewState;

			const int pointCount = std::min(frame.XData.size(), frame.YData.size());
			m_profilePoints.reserve(pointCount > 0 ? pointCount : static_cast<int>(frame.allResultPoint.size()));
			for (int index = 0; index < pointCount; ++index)
			{
				const QPointF point(frame.XData.at(index), frame.YData.at(index));
				if (IsFinitePoint(point))
				{
					m_profilePoints.push_back(point);
				}
			}

			if (m_profilePoints.isEmpty())
			{
				for (const cv::Point3d& point : frame.allResultPoint)
				{
					const QPointF projectedPoint(point.y, point.z);
					if (IsFinitePoint(projectedPoint))
					{
						m_profilePoints.push_back(projectedPoint);
					}
				}
			}

			const int trendPointCount = std::min(frame.fitLineX.size(), frame.fitLineY.size());
			m_trendLines.reserve(trendPointCount / 2);
			for (int index = 0; index + 1 < trendPointCount; index += 2)
			{
				const QPointF start(frame.fitLineX.at(index), frame.fitLineY.at(index));
				const QPointF end(frame.fitLineX.at(index + 1), frame.fitLineY.at(index + 1));
				if (IsFinitePoint(start) && IsFinitePoint(end) && QLineF(start, end).length() > 1.0e-6)
				{
					m_trendLines.push_back(QLineF(start, end));
				}
			}

			for (int index = std::min(frame.targetX.size(), frame.targetY.size()) - 1; index >= 0; --index)
			{
				const QPointF point(frame.targetX.at(index), frame.targetY.at(index));
				if (IsFinitePoint(point))
				{
					m_targetPoint = point;
					m_hasTargetPoint = true;
					break;
				}
			}
			if (!m_hasTargetPoint)
			{
				const QPointF point(frame.targetPoint.y, frame.targetPoint.z);
				if (IsFinitePoint(point))
				{
					m_targetPoint = point;
					m_hasTargetPoint = true;
				}
			}

			if (!m_viewState.hasBaseBounds)
			{
				FitViewToCurrentFrame();
			}
			update();
		}

		void ClearPreview(const QString& statusText)
		{
			m_profilePoints.clear();
			m_hasTargetPoint = false;
			m_fps = 0.0;
			m_timestamp = 0;
			m_statusText = statusText;
			m_viewState = ViewState();
			m_trendLines.clear();
			update();
		}

		void SetShowTrendLines(bool showTrendLines)
		{
			if (m_showTrendLines == showTrendLines)
			{
				return;
			}
			m_showTrendLines = showTrendLines;
			update();
		}

		ViewState CurrentViewState() const
		{
			return m_viewState;
		}

	protected:
		void paintEvent(QPaintEvent* event) override
		{
			Q_UNUSED(event);

			QPainter painter(this);
			painter.setRenderHint(QPainter::Antialiasing, true);
			painter.fillRect(rect(), QColor(18, 18, 18));

			const QRectF contentRect = rect().adjusted(12, 12, -12, -12);
			const double plotSide = std::max(1.0, std::min(contentRect.width(), contentRect.height()));
			const QRectF plotRect(
				contentRect.left() + (contentRect.width() - plotSide) * 0.5,
				contentRect.top() + (contentRect.height() - plotSide) * 0.5,
				plotSide,
				plotSide);
			painter.fillRect(plotRect, QColor(14, 15, 15));
			painter.setClipRect(plotRect);

			double minX = 0.0;
			double maxX = 0.0;
			double minY = 0.0;
			double maxY = 0.0;
			ComputeViewBounds(minX, maxX, minY, maxY);

			const double xScale = plotRect.width() / (maxX - minX);
			const double yScale = plotRect.height() / (maxY - minY);
			auto mapPoint = [plotRect, minX, minY, xScale, yScale](const QPointF& point) -> QPointF
			{
				return QPointF(
					plotRect.left() + (point.x() - minX) * xScale,
					plotRect.bottom() - (point.y() - minY) * yScale);
			};

			DrawGrid(painter, plotRect, minX, maxX, minY, maxY);
			DrawPointSeries(painter, m_profilePoints, mapPoint, QColor(0, 255, 48), 2.0);
			if (m_showTrendLines)
			{
				DrawTrendLines(painter, m_trendLines, mapPoint, plotRect);
			}
			if (m_hasTargetPoint)
			{
				QPen targetPen(QColor(255, 48, 48));
				targetPen.setWidthF(5.0);
				targetPen.setCapStyle(Qt::RoundCap);
				painter.setPen(targetPen);
				painter.drawPoint(mapPoint(m_targetPoint));
			}

			painter.setClipping(false);
			DrawOverlay(painter, plotRect);
		}

		void wheelEvent(QWheelEvent* event) override
		{
			const QPoint angleDelta = event->angleDelta();
			if (angleDelta.y() == 0)
			{
				QWidget::wheelEvent(event);
				return;
			}

			const double zoomStep = angleDelta.y() > 0 ? 1.18 : (1.0 / 1.18);
			m_viewState.zoomFactor = std::clamp(m_viewState.zoomFactor * zoomStep, 0.25, 20.0);
			event->accept();
			update();
		}

		void mousePressEvent(QMouseEvent* event) override
		{
			if (event->button() == Qt::MiddleButton)
			{
				m_isPanning = true;
				m_lastPanMousePos = event->position();
				setCursor(Qt::ClosedHandCursor);
				event->accept();
				return;
			}
			QWidget::mousePressEvent(event);
		}

		void mouseMoveEvent(QMouseEvent* event) override
		{
			if (m_isPanning)
			{
				double minX = 0.0;
				double maxX = 0.0;
				double minY = 0.0;
				double maxY = 0.0;
				ComputeViewBounds(minX, maxX, minY, maxY);

				const QRectF plotRect = ComputePlotRect();
				const double xScale = plotRect.width() / std::max(1.0e-9, maxX - minX);
				const double yScale = plotRect.height() / std::max(1.0e-9, maxY - minY);
				const QPointF delta = event->position() - m_lastPanMousePos;
				m_viewState.panOffset += QPointF(-delta.x() / xScale, delta.y() / yScale);
				m_lastPanMousePos = event->position();
				event->accept();
				update();
				return;
			}
			QWidget::mouseMoveEvent(event);
		}

		void mouseReleaseEvent(QMouseEvent* event) override
		{
			if (event->button() == Qt::MiddleButton && m_isPanning)
			{
				m_isPanning = false;
				unsetCursor();
				event->accept();
				return;
			}
			QWidget::mouseReleaseEvent(event);
		}

	private:
		QRectF ComputePlotRect() const
		{
			const QRectF contentRect = rect().adjusted(12, 12, -12, -12);
			const double plotSide = std::max(1.0, std::min(contentRect.width(), contentRect.height()));
			return QRectF(
				contentRect.left() + (contentRect.width() - plotSide) * 0.5,
				contentRect.top() + (contentRect.height() - plotSide) * 0.5,
				plotSide,
				plotSide);
		}

		void FitViewToCurrentFrame()
		{
			bool hasBounds = false;
			double minX = 0.0;
			double maxX = 0.0;
			double minY = 0.0;
			double maxY = 0.0;
			IncludeBounds(m_profilePoints, hasBounds, minX, maxX, minY, maxY);
			if (!hasBounds)
			{
				minX = -120.0;
				maxX = 120.0;
				minY = -600.0;
				maxY = -160.0;
			}
			NormalizePreviewBounds(minX, maxX, minY, maxY);
			m_viewState.baseMinX = minX;
			m_viewState.baseMaxX = maxX;
			m_viewState.baseMinY = minY;
			m_viewState.baseMaxY = maxY;
			m_viewState.zoomFactor = 1.0;
			m_viewState.panOffset = QPointF();
			m_viewState.hasBaseBounds = true;
		}

		void ComputeViewBounds(double& minX, double& maxX, double& minY, double& maxY) const
		{
			if (m_viewState.hasBaseBounds)
			{
				minX = m_viewState.baseMinX;
				maxX = m_viewState.baseMaxX;
				minY = m_viewState.baseMinY;
				maxY = m_viewState.baseMaxY;
			}
			else
			{
				minX = -120.0;
				maxX = 120.0;
				minY = -600.0;
				maxY = -160.0;
			}
			ApplyZoomToBounds(minX, maxX, minY, maxY, m_viewState.zoomFactor);
			minX += m_viewState.panOffset.x();
			maxX += m_viewState.panOffset.x();
			minY += m_viewState.panOffset.y();
			maxY += m_viewState.panOffset.y();
		}

		static bool IsFiniteValue(double value)
		{
			return std::isfinite(value);
		}

		static bool IsFinitePoint(const QPointF& point)
		{
			return IsFiniteValue(point.x()) && IsFiniteValue(point.y());
		}

		static void IncludeBounds(
			const QPointF& point,
			bool& hasBounds,
			double& minX,
			double& maxX,
			double& minY,
			double& maxY)
		{
			if (!IsFinitePoint(point))
			{
				return;
			}
			if (!hasBounds)
			{
				minX = maxX = point.x();
				minY = maxY = point.y();
				hasBounds = true;
				return;
			}
			minX = std::min(minX, point.x());
			maxX = std::max(maxX, point.x());
			minY = std::min(minY, point.y());
			maxY = std::max(maxY, point.y());
		}

		static void IncludeBounds(
			const QVector<QPointF>& points,
			bool& hasBounds,
			double& minX,
			double& maxX,
			double& minY,
			double& maxY)
		{
			for (const QPointF& point : points)
			{
				IncludeBounds(point, hasBounds, minX, maxX, minY, maxY);
			}
		}

		static void NormalizePreviewBounds(double& minX, double& maxX, double& minY, double& maxY)
		{
			const double centerX = (minX + maxX) * 0.5;
			const double centerY = (minY + maxY) * 0.5;
			const double dataSpanX = std::max(1.0, maxX - minX);
			const double dataSpanY = std::max(1.0, maxY - minY);
			const double spanX = std::max(220.0, dataSpanX * 1.16);
			const double spanY = std::max(450.0, dataSpanY * 1.24);
			minX = centerX - spanX * 0.5;
			maxX = centerX + spanX * 0.5;
			minY = centerY - spanY * 0.5;
			maxY = centerY + spanY * 0.5;
		}

		static void ApplyZoomToBounds(double& minX, double& maxX, double& minY, double& maxY, double zoomFactor)
		{
			zoomFactor = std::clamp(zoomFactor, 0.25, 20.0);
			const double centerX = (minX + maxX) * 0.5;
			const double centerY = (minY + maxY) * 0.5;
			const double spanX = (maxX - minX) / zoomFactor;
			const double spanY = (maxY - minY) / zoomFactor;
			minX = centerX - spanX * 0.5;
			maxX = centerX + spanX * 0.5;
			minY = centerY - spanY * 0.5;
			maxY = centerY + spanY * 0.5;
		}

		static void DrawGrid(
			QPainter& painter,
			const QRectF& plotRect,
			double minX,
			double maxX,
			double minY,
			double maxY)
		{
			QPen gridPen(QColor(190, 190, 190, 150));
			gridPen.setStyle(Qt::DotLine);
			gridPen.setWidthF(1.0);
			painter.setPen(gridPen);
			const int gridCount = 4;
			for (int index = 0; index <= gridCount; ++index)
			{
				const double x = plotRect.left() + plotRect.width() * index / gridCount;
				const double y = plotRect.top() + plotRect.height() * index / gridCount;
				painter.drawLine(QPointF(x, plotRect.top()), QPointF(x, plotRect.bottom()));
				painter.drawLine(QPointF(plotRect.left(), y), QPointF(plotRect.right(), y));
			}

			QPen axisPen(QColor(230, 230, 230, 190));
			axisPen.setStyle(Qt::SolidLine);
			axisPen.setWidthF(1.0);
			painter.setPen(axisPen);
			painter.drawLine(QPointF(plotRect.center().x(), plotRect.top()), QPointF(plotRect.center().x(), plotRect.bottom()));
			painter.drawLine(QPointF(plotRect.left(), plotRect.center().y()), QPointF(plotRect.right(), plotRect.center().y()));

			QFont labelFont = painter.font();
			labelFont.setPointSize(8);
			painter.setFont(labelFont);
			painter.setPen(QColor(220, 220, 220));
			QFontMetrics labelMetrics(labelFont);
			for (int index = 0; index <= gridCount; ++index)
			{
				const double x = plotRect.left() + plotRect.width() * index / gridCount;
				const double y = plotRect.top() + plotRect.height() * index / gridCount;
				const double labelX = minX + (maxX - minX) * index / gridCount;
				const double labelY = maxY - (maxY - minY) * index / gridCount;
				if (index > 0)
				{
					const QString xText = QString::number(labelX, 'f', 0);
					const int textWidth = labelMetrics.horizontalAdvance(xText);
					double labelLeft = x - textWidth * 0.5;
					labelLeft = std::clamp(labelLeft, plotRect.left() + 6.0, plotRect.right() - textWidth - 6.0);
					painter.drawText(QPointF(labelLeft, plotRect.bottom() - 6.0), xText);
				}
				if (index < gridCount)
				{
					const QString yText = QString::number(labelY, 'f', 0);
					painter.drawText(QPointF(plotRect.left() + 6.0, y - 6.0), yText);
				}
			}
		}

		template<typename Mapper>
		static void DrawPointSeries(
			QPainter& painter,
			const QVector<QPointF>& points,
			const Mapper& mapper,
			const QColor& color,
			double pointWidth)
		{
			if (points.isEmpty())
			{
				return;
			}
			QPen pointPen(color.lighter(110));
			pointPen.setWidthF(pointWidth);
			painter.setPen(pointPen);
			const int drawStep = points.size() > 4000 ? 3 : (points.size() > 2000 ? 2 : 1);
			for (int index = 0; index < points.size(); index += drawStep)
			{
				painter.drawPoint(mapper(points.at(index)));
			}
		}

		template<typename Mapper>
		static void DrawTrendLines(
			QPainter& painter,
			const QVector<QLineF>& lines,
			const Mapper& mapper,
			const QRectF& plotRect)
		{
			if (lines.isEmpty())
			{
				return;
			}

			const QColor colors[] =
			{
				QColor(255, 205, 54),
				QColor(68, 210, 255),
				QColor(255, 92, 190)
			};
			const double extendLength = std::max(plotRect.width(), plotRect.height()) * 2.0;
			for (int index = 0; index < lines.size(); ++index)
			{
				const QPointF start = mapper(lines.at(index).p1());
				const QPointF end = mapper(lines.at(index).p2());
				QLineF screenLine(start, end);
				if (screenLine.length() <= 1.0)
				{
					continue;
				}

				const double dx = (end.x() - start.x()) / screenLine.length();
				const double dy = (end.y() - start.y()) / screenLine.length();
				const QPointF center((start.x() + end.x()) * 0.5, (start.y() + end.y()) * 0.5);
				QPen linePen(colors[index % 3]);
				linePen.setWidthF(2.5);
				linePen.setCapStyle(Qt::RoundCap);
				painter.setPen(linePen);
				painter.drawLine(
					center - QPointF(dx * extendLength, dy * extendLength),
					center + QPointF(dx * extendLength, dy * extendLength));
			}
		}

		void DrawOverlay(QPainter& painter, const QRectF& plotRect) const
		{
			QFont fpsFont = painter.font();
			fpsFont.setPointSize(18);
			fpsFont.setBold(false);
			painter.setFont(fpsFont);
			const QString fpsText = QString("FPS:%1").arg(m_fps, 0, 'f', 2);
			QFontMetrics fpsMetrics(fpsFont);
			QRectF fpsRect = fpsMetrics.boundingRect(fpsText).adjusted(-10, -6, 10, 6);
			fpsRect.moveTopLeft(plotRect.topLeft() + QPointF(10, 10));
			painter.setPen(QPen(QColor(230, 230, 230), 1.0));
			painter.setBrush(QColor(0, 0, 0, 210));
			painter.drawRoundedRect(fpsRect, 4, 4);
			painter.setPen(QColor(255, 255, 255));
			painter.drawText(fpsRect, Qt::AlignCenter, fpsText);

			QFont statusFont = painter.font();
			statusFont.setPointSize(9);
			painter.setFont(statusFont);
			painter.setPen(QColor(190, 230, 240));
			const QString status = m_profilePoints.isEmpty()
				? (m_statusText.isEmpty() ? QStringLiteral("等待相机点云...") : m_statusText)
				: QStringLiteral("%1  点数:%2  时间戳:%3").arg(m_statusText).arg(m_profilePoints.size()).arg(m_timestamp);
			painter.drawText(plotRect.adjusted(12, 12, -12, -12), Qt::AlignRight | Qt::AlignTop | Qt::TextWordWrap, status);
		}

		QVector<QPointF> m_profilePoints;
		QVector<QLineF> m_trendLines;
		QPointF m_targetPoint;
		bool m_hasTargetPoint = false;
		bool m_showTrendLines = false;
		double m_fps = 0.0;
		qulonglong m_timestamp = 0;
		QString m_statusText;
		ViewState m_viewState;
		QPointF m_lastPanMousePos;
		bool m_isPanning = false;
	};

	static udpDataShow BuildFilteredGroovePreviewFrame(const udpDataShow& frame)
	{
		LaserFramePoint3DFilterOptions filterOptions;
		filterOptions.enableDominantLineSegmentFilter = true;
		filterOptions.dominantLineMinSegmentCount = 2;
		filterOptions.dominantLineMaxSegmentCount = 3;
		filterOptions.dominantLineTrendRecoverDistanceMinMm = 2.0;
		filterOptions.dominantLineTrendRecoverDistanceStepScale = 5.0;
		filterOptions.dominantLineTrendRecoverEndpointToleranceMm = 3.0;
		filterOptions.dominantLineFastSampleCount = 160;
		filterOptions.dominantLineFastCandidateCount = 128;
		filterOptions.profileComponentKeepStandalone = true;
		filterOptions.profileRunKeepStandalone = true;

		udpDataShow filteredFrame = frame;
		std::vector<LaserFramePoint3DTrendLine> trendLines;
		filteredFrame.allResultPoint = FilterSingleFrameLaserPoint3D(frame.allResultPoint, filterOptions, &trendLines);
		filteredFrame.XData.clear();
		filteredFrame.YData.clear();
		filteredFrame.XData.reserve(static_cast<int>(filteredFrame.allResultPoint.size()));
		filteredFrame.YData.reserve(static_cast<int>(filteredFrame.allResultPoint.size()));
		for (const cv::Point3d& point : filteredFrame.allResultPoint)
		{
			filteredFrame.XData.append(point.y);
			filteredFrame.YData.append(point.z);
		}
		filteredFrame.fitLineX.clear();
		filteredFrame.fitLineY.clear();
		filteredFrame.fitLineX.reserve(static_cast<int>(trendLines.size() * 2));
		filteredFrame.fitLineY.reserve(static_cast<int>(trendLines.size() * 2));
		for (const LaserFramePoint3DTrendLine& line : trendLines)
		{
			filteredFrame.fitLineX.append(line.start.y);
			filteredFrame.fitLineY.append(line.start.z);
			filteredFrame.fitLineX.append(line.end.y);
			filteredFrame.fitLineY.append(line.end.z);
		}
		return filteredFrame;
	}

	class GroovePointCloudDialog final : public QDialog
	{
	public:
		explicit GroovePointCloudDialog(QWidget* parent = nullptr)
			: QDialog(parent)
		{
			setWindowTitle("坡口相机点云预览");
			setModal(false);
			setAttribute(Qt::WA_DeleteOnClose, true);

			QVBoxLayout* mainLayout = new QVBoxLayout(this);
			mainLayout->setContentsMargins(14, 14, 14, 14);
			mainLayout->setSpacing(10);

			QHBoxLayout* toolbarLayout = new QHBoxLayout();
			toolbarLayout->addStretch(1);
			m_rawButton = new QPushButton("滤波前", this);
			m_filteredButton = new QPushButton("滤波后", this);
			for (QPushButton* button : { m_rawButton, m_filteredButton })
			{
				button->setCheckable(true);
				button->setMinimumSize(150, 44);
				toolbarLayout->addWidget(button);
			}
			m_trendLineButton = new QPushButton("三段线", this);
			m_trendLineButton->setCheckable(true);
			m_trendLineButton->setMinimumSize(120, 44);
			toolbarLayout->addSpacing(12);
			toolbarLayout->addWidget(m_trendLineButton);
			toolbarLayout->addStretch(1);
			mainLayout->addLayout(toolbarLayout);

			m_view = new GroovePointCloudView(this);
			const int plotSide = ComputePlotSide(parent);
			m_view->setFixedSize(plotSide, plotSide);
			m_view->ClearPreview("等待相机点云...");
			mainLayout->addWidget(m_view, 1, Qt::AlignCenter);

			connect(m_rawButton, &QPushButton::clicked, this, [this]()
				{
					SetFilteredMode(false);
				});
			connect(m_filteredButton, &QPushButton::clicked, this, [this]()
				{
					SetFilteredMode(true);
				});
			connect(m_trendLineButton, &QPushButton::clicked, this, [this]()
				{
					m_showTrendLines = (m_trendLineButton != nullptr && m_trendLineButton->isChecked());
					RefreshView(false, true);
				});
			SetFilteredMode(false);
			adjustSize();
			setMinimumSize(sizeHint());
			resize(sizeHint());
		}

		void SetFrame(const udpDataShow& rawFrame, const QString& statusText)
		{
			const bool sourceChanged = m_hasFrame && (m_statusText != statusText);
			if (sourceChanged)
			{
				ResetViewStates();
				m_filteredFrameValid = false;
				m_filterBuildRunning = false;
				++m_filterBuildGeneration;
			}
			m_rawFrame = rawFrame;
			m_statusText = statusText;
			m_hasFrame = true;
			RefreshView();
		}

		void ClearPreview(const QString& statusText)
		{
			m_hasFrame = false;
			m_statusText = statusText;
			m_filteredFrameValid = false;
			m_filterBuildRunning = false;
			++m_filterBuildGeneration;
			ResetViewStates();
			if (m_view != nullptr)
			{
				m_view->ClearPreview(statusText);
			}
		}

	private:
		static int ComputePlotSide(QWidget* parent)
		{
			const QScreen* screen = nullptr;
			if (parent != nullptr)
			{
				screen = parent->screen();
			}
			if (screen == nullptr)
			{
				screen = QGuiApplication::primaryScreen();
			}

			int maxSide = 1080;
			if (screen != nullptr)
			{
				const QRect available = screen->availableGeometry();
				maxSide = std::min(available.width() - 120, available.height() - 170);
			}
			return std::max(520, std::min(maxSide, 1080));
		}

		void SetFilteredMode(bool showFiltered)
		{
			if (m_showFiltered != showFiltered)
			{
				SaveCurrentViewState();
			}
			m_showFiltered = showFiltered;
			if (m_rawButton != nullptr)
			{
				m_rawButton->setChecked(!m_showFiltered);
			}
			if (m_filteredButton != nullptr)
			{
				m_filteredButton->setChecked(m_showFiltered);
			}
			RefreshView(true, false);
		}

		void ResetViewStates()
		{
			m_rawViewState = GroovePointCloudView::ViewState();
			m_filteredViewState = GroovePointCloudView::ViewState();
		}

		GroovePointCloudView::ViewState& CurrentModeViewState()
		{
			return m_showFiltered ? m_filteredViewState : m_rawViewState;
		}

		void SaveCurrentViewState()
		{
			if (m_view == nullptr)
			{
				return;
			}
			CurrentModeViewState() = m_view->CurrentViewState();
		}

		void RequestFilteredFrame(bool forceBuild)
		{
			if (!m_hasFrame)
			{
				return;
			}
			if (m_filteredFrameValid && m_filteredFrame.timestamp == m_rawFrame.timestamp)
			{
				return;
			}
			if (m_filterBuildRunning)
			{
				return;
			}

			const qint64 nowMs = QDateTime::currentMSecsSinceEpoch();
			constexpr qint64 kFilteredPreviewMinIntervalMs = 100;
			if (!forceBuild && m_filteredFrameValid && (nowMs - m_lastFilteredBuildMs) < kFilteredPreviewMinIntervalMs)
			{
				return;
			}

			m_filterBuildRunning = true;
			m_lastFilteredBuildMs = nowMs;
			const udpDataShow rawFrame = m_rawFrame;
			const quint64 buildGeneration = m_filterBuildGeneration;
			QPointer<GroovePointCloudDialog> dialog(this);
			QThread* filterThread = QThread::create([dialog, rawFrame, buildGeneration]() mutable
				{
					udpDataShow filteredFrame = BuildFilteredGroovePreviewFrame(rawFrame);
					if (dialog == nullptr)
					{
						return;
					}
					QMetaObject::invokeMethod(dialog.data(), [dialog, filteredFrame = std::move(filteredFrame), buildGeneration]() mutable
						{
							if (dialog != nullptr)
							{
								dialog->FinishFilteredFrame(std::move(filteredFrame), buildGeneration);
							}
						}, Qt::QueuedConnection);
				});
			connect(filterThread, &QThread::finished, filterThread, &QObject::deleteLater);
			filterThread->start();
		}

		void FinishFilteredFrame(udpDataShow filteredFrame, quint64 buildGeneration)
		{
			if (buildGeneration != m_filterBuildGeneration)
			{
				return;
			}
			m_filterBuildRunning = false;
			m_filteredFrame = std::move(filteredFrame);
			m_filteredFrameValid = true;
			if (m_showFiltered)
			{
				if (m_hasFrame && m_filteredFrame.timestamp != m_rawFrame.timestamp)
				{
					RequestFilteredFrame(false);
				}
				RefreshView(false, false);
			}
		}

		void RefreshView(bool forceFilteredBuild = false, bool saveCurrentViewState = true)
		{
			if (m_view == nullptr)
			{
				return;
			}
			if (saveCurrentViewState)
			{
				SaveCurrentViewState();
			}
			if (!m_hasFrame)
			{
				m_view->ClearPreview(m_statusText);
				return;
			}
			if (m_showFiltered)
			{
				RequestFilteredFrame(forceFilteredBuild);
			}
			const bool showCachedFiltered = m_showFiltered && m_filteredFrameValid;
			if (m_showFiltered && !showCachedFiltered)
			{
				m_view->ClearPreview(QString("%1  滤波后计算中").arg(m_statusText));
				return;
			}
			const udpDataShow& frame = showCachedFiltered ? m_filteredFrame : m_rawFrame;
			const QString modeText = m_showFiltered
				? (showCachedFiltered ? "滤波后" : "滤波后计算中")
				: "滤波前";
			m_view->SetShowTrendLines(m_showTrendLines && showCachedFiltered);
			m_view->SetFrame(frame, QString("%1  %2").arg(m_statusText, modeText), CurrentModeViewState());
			SaveCurrentViewState();
		}

		GroovePointCloudView* m_view = nullptr;
		QPushButton* m_rawButton = nullptr;
		QPushButton* m_filteredButton = nullptr;
		QPushButton* m_trendLineButton = nullptr;
		udpDataShow m_rawFrame;
		udpDataShow m_filteredFrame;
		QString m_statusText;
		bool m_hasFrame = false;
		bool m_showFiltered = false;
		bool m_showTrendLines = false;
		bool m_filteredFrameValid = false;
		bool m_filterBuildRunning = false;
		qint64 m_lastFilteredBuildMs = 0;
		quint64 m_filterBuildGeneration = 0;
		GroovePointCloudView::ViewState m_rawViewState;
		GroovePointCloudView::ViewState m_filteredViewState;
	};
}

struct QtWidgetsApplication4::CameraRuntime
{
	TcpSensorClientWorker* worker = nullptr;
	QThread* thread = nullptr;
	CameraFrameCache* cache = nullptr;
	QString cameraIP;
	int cameraPort = 0;
	bool running = false;
	qint64 datagramCount = 0;
	qint64 filteredDatagramCount = 0;
	qint64 decodedFrameCount = 0;
	qint64 decodeFailedCount = 0;
	qint64 appendedFrameCount = 0;
	QString cameraStatus;
};

QtWidgetsApplication4::QtWidgetsApplication4(QWidget* parent)
	: QMainWindow(parent)
	, m_pContralUnit(nullptr)
	, m_grooveCameraDisplayTimer(nullptr)
	, m_robotLogDisplayTimer(nullptr)
	, m_pMainStack(nullptr)
	, m_pAuthPage(nullptr)
	, m_pDashboardPage(nullptr)
	, m_pManagementPage(nullptr)
	, m_pManagementStack(nullptr)
	, m_pManagementHomePage(nullptr)
	, m_pAccountManagementPage(nullptr)
	, m_pControlUnitManagementPage(nullptr)
	, m_pRobotSelectorCombo(nullptr)
	, m_pRobotSelectorLabel(nullptr)
	, m_nCurrentRobotUnitIndex(-1)
	, m_nWeldProcessPageUnitIndex(-1)
	, m_nFunctionTestPageUnitIndex(-1)
	, m_nMeasureThenWeldPageUnitIndex(-1)
	, m_nRobotJogPageUnitIndex(-1)
	, m_pRobotLogText(nullptr)
	, m_pGroovePointCloudDialog(nullptr)
	, m_pCurrentUserButton(nullptr)
	, m_pManagementUserLabel(nullptr)
	, m_pPermissionHintLabel(nullptr)
	, m_pManagementCameraReceiveModeBtn(nullptr)
	, m_pTouchKeyboardModeCombo(nullptr)
	, m_pAuthTitleLabel(nullptr)
	, m_pAuthHintLabel(nullptr)
	, m_pLoginNameCombo(nullptr)
	, m_pLoginNameEdit(nullptr)
	, m_pLoginPasswordEdit(nullptr)
	, m_pAuthConfirmPasswordRow(nullptr)
	, m_pAuthConfirmPasswordEdit(nullptr)
	, m_pAccountLogText(nullptr)
	, m_pAutoLoginCheck(nullptr)
	, m_pRememberPasswordCheck(nullptr)
	, m_pAuthLoginModeBtn(nullptr)
	, m_pAuthRegisterModeBtn(nullptr)
	, m_pAuthSubmitBtn(nullptr)
	, m_pGuestLoginBtn(nullptr)
	, m_pDashboardConnectBtn(nullptr)
	, m_pDashboardClearAlarmBtn(nullptr)
	, m_pDashboardModeBtn(nullptr)
	, m_pDashboardDebugLogBtn(nullptr)
	, m_pAccountManagementAction(nullptr)
	, m_pCameraParamBtn(nullptr)
	, m_pWeldSeamCompBtn(nullptr)
	, m_pWeldProcessPage(nullptr)
	, m_pFunctionTestPage(nullptr)
	, m_pMeasureThenWeldPage(nullptr)
	, m_pPreciseMeasureEditPage(nullptr)
	, m_pWeldSeamCompPage(nullptr)
	, m_pCameraParamPage(nullptr)
	, m_pRobotJogPage(nullptr)
	, m_pTouchKeyboardManager(nullptr)
	, m_bFanucMovlForward(true)
	, m_bFanucMovlRunning(false)
	, m_bFanucMovjRunning(false)
	, m_bFanucMoveZeroRunning(false)
	, m_sCurrentUserName(QStringLiteral("访客"))
	, m_sCurrentUserRole(kRoleOperator)
	, m_sAuthHintOverride()
	, m_bAuthRegisterMode(false)
	, m_bOpenEmbeddedInManagement(false)
	, m_bPendingOpenManagementAfterLogin(false)
	, m_bDebugLogMode(false)
	, m_bUseSharedScanCameraReceiver(false)
	, m_bFanucMonitorReading(false)
{
	ui.setupUi(this);
	ApplyUnifiedWindowChrome(this);
	m_pTouchKeyboardManager = new TouchKeyboardManager(this);
	m_pTouchKeyboardManager->LoadSettings();
	m_pTouchKeyboardManager->Install(qApp);
	if (ui.FanucMonitorText != nullptr)
	{
		ui.FanucMonitorText->document()->setMaximumBlockCount(200);
	}
	if (ui.GrooveCameraText != nullptr)
	{
		ui.GrooveCameraText->document()->setMaximumBlockCount(200);
	}
	if (ui.menuBar != nullptr)
	{
		ui.menuBar->clear();
		ui.menuBar->hide();
	}
	if (ui.mainToolBar != nullptr)
	{
		ui.mainToolBar->clear();
		ui.mainToolBar->setMovable(false);
		ui.mainToolBar->setFloatable(false);
		ui.mainToolBar->setToolButtonStyle(Qt::ToolButtonTextOnly);
		ui.mainToolBar->setIconSize(QSize(18, 18));
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
		"QMenuBar, QStatusBar, QToolBar { background: #0B1117; color: #ECF3F4; spacing: 4px; }"
		"QMenuBar::item { padding: 5px 10px; background: transparent; }"
		"QMenuBar::item:selected { background: #223240; }"
		"QMenu { background: #101820; color: #ECF3F4; border: 1px solid #2E4656; padding: 4px; }"
		"QMenu::item { padding: 6px 28px 6px 20px; }"
		"QMenu::item:selected { background: #2D5465; }"
		"QToolButton { background: transparent; color: #ECF3F4; border: 1px solid transparent; border-radius: 4px; padding: 5px 8px; }"
		"QToolButton:hover { background: #223240; border-color: #3C6173; }"
		"QToolButton:checked { background: #305F55; border-color: #7BD8B3; }"
		"QLabel { color: #BACBD1; }");

	QWidget* mainPanel = new QWidget(this);
	QVBoxLayout* rootLayout = new QVBoxLayout(mainPanel);
	rootLayout->setContentsMargins(0, 0, 0, 0);
	rootLayout->setSpacing(0);
	setCentralWidget(mainPanel);

	m_pMainStack = new QStackedWidget(mainPanel);
	m_pMainStack->setContentsMargins(0, 0, 0, 0);
	m_pMainStack->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

	m_pAuthPage = new QWidget(m_pMainStack);
	m_pAuthPage->setObjectName("AuthPage");
	m_pAuthPage->setStyleSheet(
		"QWidget#AuthPage {"
		"  background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #0C121A, stop:0.55 #132033, stop:1 #0F1824);"
		"}");
	QVBoxLayout* authLayout = new QVBoxLayout(m_pAuthPage);
	authLayout->setContentsMargins(24, 24, 24, 24);
	authLayout->setSpacing(0);
	m_pMainStack->addWidget(m_pAuthPage);

	QWidget* authCard = new QWidget(m_pAuthPage);
	authCard->setObjectName("AuthCard");
	authCard->setMaximumWidth(360);
	authCard->setStyleSheet(
		"QWidget#AuthCard { background: transparent; border: none; }"
		"QWidget#AuthCard QLabel { color: #BACBD1; }"
		"QWidget#AuthCard QCheckBox { color: #AFC8CE; font-size: 13px; spacing: 6px; }"
		"QWidget#AuthCard QCheckBox::indicator { width: 18px; height: 18px; }"
		"QWidget#AuthCard QCheckBox::indicator:unchecked { border: 1px solid #4B6275; border-radius: 9px; background: #101820; }"
		"QWidget#AuthCard QCheckBox::indicator:checked { border: 1px solid #67B5FF; border-radius: 9px; background: #67B5FF; }");
	QGraphicsDropShadowEffect* authShadow = new QGraphicsDropShadowEffect(authCard);
	authShadow->setBlurRadius(0);
	authShadow->setOffset(0, 0);
	authShadow->setColor(QColor(0, 0, 0, 0));
	authCard->setGraphicsEffect(authShadow);

	QVBoxLayout* authCardLayout = new QVBoxLayout(authCard);
	authCardLayout->setContentsMargins(0, 0, 0, 0);
	authCardLayout->setSpacing(8);

	QHBoxLayout* avatarRowLayout = new QHBoxLayout();
	avatarRowLayout->setContentsMargins(0, 0, 0, 0);
	avatarRowLayout->addStretch(1);
	QLabel* avatarLabel = new QLabel(authCard);
	avatarLabel->setFixedSize(126, 126);
	avatarLabel->setAlignment(Qt::AlignCenter);
	avatarLabel->setStyleSheet(
		"QLabel { background: #101923; border-radius: 63px; border: 1px solid #2E4256; }");
	QPixmap avatarPixmap(":/QtWidgetsApplication4/icons/minimal_robot_icon_blue_black.svg");
	if (!avatarPixmap.isNull())
	{
		avatarLabel->setPixmap(avatarPixmap.scaled(88, 88, Qt::KeepAspectRatio, Qt::SmoothTransformation));
	}
	avatarRowLayout->addWidget(avatarLabel, 0, Qt::AlignHCenter);
	avatarRowLayout->addStretch(1);
	authCardLayout->addLayout(avatarRowLayout);

	m_pAuthHintLabel = new QLabel("软件启动后先登录；没有账号时可切换到注册，只创建操作员权限。", authCard);
	m_pAuthHintLabel->setAlignment(Qt::AlignCenter);
	m_pAuthHintLabel->setWordWrap(true);
	m_pAuthHintLabel->setStyleSheet("QLabel { color: #AFC8CE; font-size: 11px; }");
	m_pAuthHintLabel->hide();
	authCardLayout->addWidget(m_pAuthHintLabel);

	QHBoxLayout* authModeLayout = new QHBoxLayout();
	authModeLayout->setContentsMargins(0, 0, 0, 0);
	authModeLayout->setSpacing(26);
	m_pAuthLoginModeBtn = new QPushButton("登录", authCard);
	m_pAuthRegisterModeBtn = new QPushButton("注册", authCard);
	m_pAuthLoginModeBtn->setCheckable(true);
	m_pAuthRegisterModeBtn->setCheckable(true);
	m_pAuthLoginModeBtn->setCursor(Qt::PointingHandCursor);
	m_pAuthRegisterModeBtn->setCursor(Qt::PointingHandCursor);
	m_pAuthLoginModeBtn->setMinimumHeight(28);
	m_pAuthRegisterModeBtn->setMinimumHeight(28);
	m_pAuthLoginModeBtn->setFixedWidth(86);
	m_pAuthRegisterModeBtn->setFixedWidth(86);
	m_pAuthLoginModeBtn->setFlat(true);
	m_pAuthRegisterModeBtn->setFlat(true);
	m_pAuthLoginModeBtn->setStyleSheet(
		"QPushButton { color: #AFC8CE; border: none; background: transparent; font-size: 15px; }"
		"QPushButton:checked { color: #72D4DD; font-weight: 700; }");
	m_pAuthRegisterModeBtn->setStyleSheet(
		"QPushButton { color: #AFC8CE; border: none; background: transparent; font-size: 15px; }"
		"QPushButton:checked { color: #72D4DD; font-weight: 700; }");
	authModeLayout->addStretch(1);
	authModeLayout->addWidget(m_pAuthLoginModeBtn);
	authModeLayout->addWidget(m_pAuthRegisterModeBtn);
	authModeLayout->addStretch(1);
	authCardLayout->addLayout(authModeLayout);

	QFormLayout* authForm = new QFormLayout();
	authForm->setContentsMargins(0, 0, 0, 0);
	authForm->setSpacing(9);
	authForm->setFormAlignment(Qt::AlignHCenter | Qt::AlignTop);
	m_pLoginNameCombo = new QComboBox(authCard);
	m_pLoginNameCombo->setEditable(true);
	m_pLoginNameCombo->setInsertPolicy(QComboBox::NoInsert);
	m_pLoginNameCombo->setMaxVisibleItems(8);
	m_pLoginNameEdit = m_pLoginNameCombo->lineEdit();
	m_pLoginPasswordEdit = new QLineEdit(authCard);
	m_pAuthConfirmPasswordRow = new QWidget(authCard);
	m_pAuthConfirmPasswordRow->setFixedSize(300, 44);
	QHBoxLayout* confirmRowLayout = new QHBoxLayout(m_pAuthConfirmPasswordRow);
	confirmRowLayout->setContentsMargins(0, 0, 0, 0);
	confirmRowLayout->setSpacing(0);
	QLabel* confirmPasswordLabel = new QLabel("确认密码", m_pAuthConfirmPasswordRow);
	m_pAuthConfirmPasswordEdit = new QLineEdit(m_pAuthConfirmPasswordRow);
	m_pLoginPasswordEdit->setEchoMode(QLineEdit::Password);
	m_pAuthConfirmPasswordEdit->setEchoMode(QLineEdit::Password);
	m_pLoginNameEdit->setPlaceholderText("请输入账号");
	m_pLoginPasswordEdit->setPlaceholderText("请输入密码");
	m_pAuthConfirmPasswordEdit->setPlaceholderText("再次输入密码");
	m_pLoginNameCombo->setFixedSize(300, 44);
	m_pLoginPasswordEdit->setFixedSize(300, 44);
	m_pAuthConfirmPasswordEdit->setFixedSize(300, 44);
	auto styleAuthEdit = [](QLineEdit* edit)
		{
			edit->setStyleSheet(
				"QLineEdit {"
				"  background: #0F1720;"
				"  color: #F7FCFC;"
				"  border: 1px solid #2E4256;"
				"  border-radius: 11px;"
				"  padding: 0 14px;"
				"  font-size: 14px;"
				"}"
				"QLineEdit:focus { border-color: #72D4DD; }");
		};
	m_pLoginNameEdit->setStyleSheet(
		"QLineEdit {"
		"  background: transparent;"
		"  color: #F7FCFC;"
		"  border: none;"
		"  padding: 0 10px;"
		"  font-size: 14px;"
		"}");
	styleAuthEdit(m_pLoginPasswordEdit);
	styleAuthEdit(m_pAuthConfirmPasswordEdit);
	m_pLoginNameCombo->setStyleSheet(
		"QComboBox {"
		"  background: #000000;"
		"  color: #F7FCFC;"
		"  border: 1px solid #2E4256;"
		"  border-radius: 0px;"
		"  padding: 0 38px 0 0;"
		"  font-size: 14px;"
		"}"
		"QComboBox:focus { border-color: #72D4DD; }"
		"QComboBox::drop-down { border-left: 1px solid #2E4256; border-radius: 0px; width: 32px; background: #000000; }"
		"QComboBox::down-arrow { image: url(:/QtWidgetsApplication4/icons/chevron-down.svg); width: 12px; height: 8px; }"
		"QComboBox QAbstractItemView { background: #000000; color: #ECF3F4; selection-background-color: #2D5465; border: 1px solid #2E4256; border-radius: 0px; outline: 0px; }");
	confirmPasswordLabel->setStyleSheet("QLabel { color: #AFC8CE; font-size: 12px; }");
	confirmPasswordLabel->hide();
	authForm->addRow(m_pLoginNameCombo);
	authForm->addRow(m_pLoginPasswordEdit);
	confirmRowLayout->addWidget(m_pAuthConfirmPasswordEdit, 1);
	authForm->addRow(m_pAuthConfirmPasswordRow);
	authCardLayout->addLayout(authForm);

	QHBoxLayout* optionLayout = new QHBoxLayout();
	optionLayout->setContentsMargins(0, 0, 0, 0);
	optionLayout->setSpacing(14);
	m_pRememberPasswordCheck = new QCheckBox("记住密码", authCard);
	m_pAutoLoginCheck = new QCheckBox("自动登录", authCard);
	optionLayout->addStretch(1);
	optionLayout->addWidget(m_pRememberPasswordCheck);
	optionLayout->addWidget(m_pAutoLoginCheck);
	optionLayout->addStretch(1);
	authCardLayout->addLayout(optionLayout);

	QHBoxLayout* authButtonLayout = new QHBoxLayout();
	authButtonLayout->setContentsMargins(0, 0, 0, 0);
	authButtonLayout->setSpacing(12);
	m_pAuthSubmitBtn = new QPushButton("登录", authCard);
	QPushButton* authCancelBtn = new QPushButton("退出登录", authCard);
	m_pGuestLoginBtn = new QPushButton("游客登录", authCard);
	m_pAuthSubmitBtn->setCursor(Qt::PointingHandCursor);
	authCancelBtn->setCursor(Qt::PointingHandCursor);
	m_pGuestLoginBtn->setCursor(Qt::PointingHandCursor);
	m_pAuthSubmitBtn->setFixedSize(144, 42);
	authCancelBtn->setFixedSize(144, 42);
	m_pGuestLoginBtn->setFixedSize(300, 34);
	m_pAuthSubmitBtn->setStyleSheet(
		"QPushButton {"
		"  background: #67B5FF;"
		"  color: #FFFFFF;"
		"  border: none;"
		"  border-radius: 11px;"
		"  font-size: 15px;"
		"  font-weight: 700;"
		"}"
		"QPushButton:hover { background: #5AA8F3; }"
		"QPushButton:pressed { background: #4B97EA; }");
	authCancelBtn->setStyleSheet(
		"QPushButton {"
		"  background: #16212D;"
		"  color: #AFC8CE;"
		"  border: 1px solid #2E4256;"
		"  border-radius: 11px;"
		"  font-size: 13px;"
		"}"
		"QPushButton:hover { background: #1B2835; }");
	m_pGuestLoginBtn->setStyleSheet(
		"QPushButton {"
		"  background: transparent;"
		"  color: #AFC8CE;"
		"  border: none;"
		"  font-size: 13px;"
		"}"
		"QPushButton:hover { color: #72D4DD; }");
	authButtonLayout->addStretch(1);
	authButtonLayout->addWidget(m_pAuthSubmitBtn);
	authButtonLayout->addWidget(authCancelBtn);
	authButtonLayout->addStretch(1);
	authCardLayout->addLayout(authButtonLayout);
	authCardLayout->addWidget(m_pGuestLoginBtn, 0, Qt::AlignHCenter);

	m_pAccountLogText = new QPlainTextEdit(authCard);
	m_pAccountLogText->setReadOnly(true);
	m_pAccountLogText->document()->setMaximumBlockCount(300);
	m_pAccountLogText->setMaximumHeight(54);
	m_pAccountLogText->hide();
	m_pAccountLogText->setStyleSheet(
		"QPlainTextEdit {"
		"  background: rgba(13, 20, 29, 0.88);"
		"  color: #AFC8CE;"
		"  border: 1px solid #2E4256;"
		"  border-radius: 12px;"
		"  padding: 8px;"
		"  font-size: 12px;"
		"}");
	m_pAccountLogText->setPlainText("账号日志：等待登录或注册...");
	authCardLayout->addWidget(m_pAccountLogText, 1);

	authLayout->addStretch(1);
	authLayout->addWidget(authCard, 0, Qt::AlignHCenter);
	authLayout->addStretch(1);

	connect(m_pAuthLoginModeBtn, &QPushButton::clicked, this, [this]()
		{
			SetAuthRegisterMode(false);
		});
	connect(m_pAuthRegisterModeBtn, &QPushButton::clicked, this, [this]()
		{
			SetAuthRegisterMode(true);
		});
	connect(m_pAuthSubmitBtn, &QPushButton::clicked, this, [this]()
		{
			if (m_bAuthRegisterMode)
			{
				RegisterAccount();
			}
			else
			{
				LoginCurrentAccount();
			}
		});
	connect(authCancelBtn, &QPushButton::clicked, this, &QtWidgetsApplication4::LogoutCurrentAccount);
	connect(m_pGuestLoginBtn, &QPushButton::clicked, this, &QtWidgetsApplication4::LoginAsGuest);
	connect(m_pLoginNameCombo, &QComboBox::currentTextChanged, this, [this](const QString& userName)
		{
			if (!m_bAuthRegisterMode)
			{
				FillSavedPasswordForUser(userName);
			}
		});

	m_pDashboardPage = new QWidget(m_pMainStack);
	QVBoxLayout* dashboardLayout = new QVBoxLayout(m_pDashboardPage);
	dashboardLayout->setContentsMargins(24, 18, 24, 24);
	dashboardLayout->setSpacing(14);
	m_pMainStack->addWidget(m_pDashboardPage);
	rootLayout->addWidget(m_pMainStack, 1);

	QVBoxLayout* dashboardHeaderLayout = new QVBoxLayout();
	dashboardHeaderLayout->setContentsMargins(0, 0, 0, 0);
	dashboardHeaderLayout->setSpacing(8);
	QHBoxLayout* titleLayout = new QHBoxLayout();
	titleLayout->setSpacing(10);
	QLabel* titleLabel = new QLabel("机器人控制与调试中心", m_pDashboardPage);
	titleLabel->setStyleSheet("font-size: 26px; font-weight: bold; color: #F7FCFC; letter-spacing: 1px;");
	QLabel* versionLabel = new QLabel(QString("v%1").arg(BuildAppVersionText()), m_pDashboardPage);
	versionLabel->setStyleSheet(
		"QLabel { background: #173041; color: #9ED8DB; border: 1px solid #3C6173; "
		"border-radius: 10px; padding: 4px 10px; font-size: 13px; font-weight: bold; }");
	QPushButton* aboutButton = new QPushButton("关于", m_pDashboardPage);
	aboutButton->setMinimumHeight(32);
	aboutButton->setMaximumWidth(88);
	aboutButton->setStyleSheet("QPushButton { padding: 6px 12px; font-size: 13px; border-radius: 10px; }");
	aboutButton->hide();
	m_pCurrentUserButton = new QPushButton(m_pDashboardPage);
	m_pCurrentUserButton->setCursor(Qt::PointingHandCursor);
	m_pCurrentUserButton->setMinimumHeight(34);
	m_pCurrentUserButton->setToolTip("点击打开账号菜单");
	m_pCurrentUserButton->setStyleSheet(
		"QPushButton { color: #9ED8DB; padding: 5px 10px; border: 1px solid #2E4656; "
		"border-radius: 10px; background: #101923; font-size: 14px; text-align: left; }"
		"QPushButton:hover { border-color: #72D4DD; background: #152230; }"
		"QPushButton:pressed { background: #0D171F; }");
	m_pRobotSelectorLabel = new QLabel("机器人：", m_pDashboardPage);
	m_pRobotSelectorLabel->setStyleSheet("QLabel { color: #BACBD1; font-weight: 600; }");
	m_pRobotSelectorCombo = new QComboBox(m_pDashboardPage);
	m_pRobotSelectorCombo->setMinimumWidth(270);
	m_pRobotSelectorCombo->setMaximumWidth(290);
	m_pRobotSelectorCombo->setStyleSheet(
		"QComboBox { background: #000000; color: #ECF3F4; border: 1px solid #3C6173; border-radius: 0px; padding: 6px 36px 6px 12px; }"
		"QComboBox:hover { border-color: #72D4DD; }"
		"QComboBox::drop-down { border-left: 1px solid #3C6173; border-radius: 0px; width: 30px; background: #000000; }"
		"QComboBox::down-arrow { image: url(:/QtWidgetsApplication4/icons/chevron-down.svg); width: 12px; height: 8px; }"
		"QComboBox QAbstractItemView { background: #000000; color: #ECF3F4; selection-background-color: #2D5465; border: 1px solid #3C6173; border-radius: 0px; outline: 0px; }");
	titleLayout->addWidget(titleLabel);
	titleLayout->addWidget(versionLabel, 0, Qt::AlignVCenter);
	titleLayout->addStretch(1);
	titleLayout->addWidget(m_pCurrentUserButton, 0, Qt::AlignVCenter);
	titleLayout->addWidget(m_pRobotSelectorLabel, 0, Qt::AlignVCenter);
	titleLayout->addWidget(m_pRobotSelectorCombo, 0, Qt::AlignVCenter);
	dashboardHeaderLayout->addLayout(titleLayout);
	dashboardLayout->addLayout(dashboardHeaderLayout);

	QLabel* homeHintLabel = new QLabel("主页面向现场快速操作：流程和子界面入口使用大按钮；连接、清报警、模式切换和调试日志放在右侧小工具区。", m_pDashboardPage);
	homeHintLabel->setWordWrap(true);
	homeHintLabel->setStyleSheet("QLabel { color: #AFC8CE; font-size: 14px; }");
	dashboardLayout->addWidget(homeHintLabel);

	auto makeLargeButton = [](const QString& text, QWidget* parent) -> QPushButton*
		{
			QPushButton* button = new QPushButton(text, parent);
			button->setMinimumHeight(72);
			button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
			button->setStyleSheet(
				"QPushButton { font-size: 17px; font-weight: 600; text-align: left; padding: 12px 18px; border-radius: 12px; }"
				"QPushButton:hover { background: #2D5465; border-color: #72D4DD; }");
			return button;
		};

	auto makeToolButton = [](const QString& text, QWidget* parent) -> QPushButton*
		{
			QPushButton* button = new QPushButton(text, parent);
			button->setMinimumHeight(38);
			button->setMinimumWidth(118);
			button->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
			button->setStyleSheet(
				"QPushButton { font-size: 14px; font-weight: 600; padding: 7px 12px; border-radius: 8px; }"
				"QPushButton:hover { background: #2D5465; border-color: #72D4DD; }");
			return button;
		};

	QHBoxLayout* dashboardActionLayout = new QHBoxLayout();
	dashboardActionLayout->setSpacing(14);

	QGroupBox* quickGroup = new QGroupBox("流程与子界面", m_pDashboardPage);
	QGridLayout* quickLayout = new QGridLayout(quickGroup);
	quickLayout->setSpacing(12);
	QPushButton* quickMeasureBtn = makeLargeButton("先测后焊\n按预设流程扫描并焊接", quickGroup);
	QPushButton* quickJogBtn = makeLargeButton("点动控制\n单步移动/目标点运动", quickGroup);
	QPushButton* quickCalibrationBtn = makeLargeButton("标定与相机参数\n手眼标定、矩阵和相机配置", quickGroup);
	quickLayout->addWidget(quickMeasureBtn, 0, 0);
	quickLayout->addWidget(quickJogBtn, 0, 1);
	quickLayout->addWidget(quickCalibrationBtn, 1, 0, 1, 2);
	dashboardActionLayout->addWidget(quickGroup, 1);

	QGroupBox* toolGroup = new QGroupBox("现场小工具", m_pDashboardPage);
	toolGroup->setMinimumWidth(330);
	toolGroup->setMaximumWidth(390);
	QGridLayout* toolLayout = new QGridLayout(toolGroup);
	toolLayout->setSpacing(10);
	m_pDashboardConnectBtn = makeToolButton("连接服务", toolGroup);
	QPushButton* quickPositionBtn = makeToolButton("读取当前位置", toolGroup);
	QPushButton* quickPreviewBtn = makeToolButton("坡口相机预览", toolGroup);
	m_pDashboardClearAlarmBtn = makeToolButton("清除报警", toolGroup);
	m_pDashboardModeBtn = makeToolButton("STEP 模式", toolGroup);
	QPushButton* quickReadTool1Btn = makeToolButton("读取Tool1", toolGroup);
	m_pDashboardDebugLogBtn = makeToolButton("显示调试日志", toolGroup);
	quickPreviewBtn->setCheckable(true);
	m_pDashboardDebugLogBtn->setCheckable(true);
	m_pDashboardDebugLogBtn->hide();
	toolLayout->addWidget(m_pDashboardConnectBtn, 0, 0);
	toolLayout->addWidget(quickPositionBtn, 0, 1);
	toolLayout->addWidget(quickPreviewBtn, 1, 0);
	toolLayout->addWidget(m_pDashboardClearAlarmBtn, 1, 1);
	toolLayout->addWidget(m_pDashboardModeBtn, 2, 0);
	toolLayout->addWidget(quickReadTool1Btn, 2, 1);
	toolLayout->addWidget(m_pDashboardDebugLogBtn, 3, 0, 1, 2);
	toolLayout->setColumnStretch(0, 1);
	toolLayout->setColumnStretch(1, 1);
	toolLayout->setRowStretch(4, 1);
	dashboardActionLayout->addWidget(toolGroup, 0);
	dashboardLayout->addLayout(dashboardActionLayout, 0);

	m_pManagementPage = new QWidget(this, Qt::Window);
	m_pManagementPage->setWindowTitle("管理页面");
	ApplyUnifiedWindowChrome(m_pManagementPage);
	ResizeWindowForAvailableGeometry(m_pManagementPage, QSize(1120, 760), 0.88, 0.82);
	m_pManagementPage->setStyleSheet(
		"QWidget { background: #111820; color: #ECF3F4; }"
		"QGroupBox { border: 1px solid #2E4656; border-radius: 12px; margin-top: 16px; padding-top: 12px; }"
		"QGroupBox::title { subcontrol-origin: margin; left: 14px; padding: 0 6px; color: #9ED8DB; }"
		"QPushButton { background: #233645; color: #F5FAFA; border: 1px solid #3C6173; border-radius: 10px; padding: 10px 14px; }"
		"QPushButton:hover { background: #2D5465; border-color: #72D4DD; }"
		"QPushButton:pressed { background: #18303B; }"
		"QLineEdit, QPlainTextEdit { background: #081018; color: #ECF3F4; border: 1px solid #2C4653; border-radius: 8px; padding: 6px 8px; }"
		"QComboBox { background: #000000; color: #ECF3F4; border: 1px solid #2C4653; border-radius: 0px; padding: 6px 34px 6px 8px; }"
		"QComboBox::drop-down { border-left: 1px solid #2C4653; border-radius: 0px; width: 28px; background: #000000; }"
		"QComboBox::down-arrow { image: url(:/QtWidgetsApplication4/icons/chevron-down.svg); width: 12px; height: 8px; }"
		"QComboBox QAbstractItemView { background: #000000; color: #ECF3F4; selection-background-color: #2D5465; border: 1px solid #2C4653; border-radius: 0px; outline: 0px; }"
		"QMenuBar { background: #0B1117; color: #ECF3F4; border-bottom: 1px solid #223743; }"
		"QMenuBar::item:selected { background: #244554; }"
		"QMenu { background: #101923; color: #ECF3F4; border: 1px solid #2C4653; }"
		"QMenu::item:selected { background: #2B5363; }");
	QVBoxLayout* managementLayout = new QVBoxLayout(m_pManagementPage);
	managementLayout->setContentsMargins(18, 14, 18, 18);
	managementLayout->setSpacing(12);

	QMenuBar* managementMenuBar = new QMenuBar(m_pManagementPage);
	managementLayout->setMenuBar(managementMenuBar);

	auto addManagementAction = [this](QMenu* menu, const QString& text, const std::function<void()>& handler) -> QAction*
		{
			QAction* action = new QAction(text, m_pManagementPage);
			if (menu != nullptr)
			{
				menu->addAction(action);
			}
			connect(action, &QAction::triggered, this, [handler]()
				{
					if (handler)
					{
						handler();
					}
				});
			return action;
		};
	auto openInManagement = [this](const std::function<void()>& handler)
		{
			const bool previous = m_bOpenEmbeddedInManagement;
			m_bOpenEmbeddedInManagement = true;
			if (handler)
			{
				handler();
			}
			m_bOpenEmbeddedInManagement = previous;
		};

	QMenu* managementHomeMenu = managementMenuBar->addMenu("主页");
	QMenu* managementRobotMenu = managementMenuBar->addMenu("机器人");
	QMenu* managementAccountMenu = managementMenuBar->addMenu("账号");
	QMenu* managementProcessMenu = managementMenuBar->addMenu("工艺");
	QMenu* managementCameraMenu = managementMenuBar->addMenu("相机");
	QMenu* managementDebugMenu = managementMenuBar->addMenu("调试");

	addManagementAction(managementHomeMenu, "返回主页", [this]() { ShowDashboardPage(); });
	addManagementAction(managementHomeMenu, "关闭管理界面", [this]() { if (m_pManagementPage != nullptr) { m_pManagementPage->hide(); } });
	addManagementAction(managementRobotMenu, "控制单元管理", [this]() { OpenControlUnitManagementDialog(); });
	m_pAccountManagementAction = addManagementAction(managementAccountMenu, "账号管理", [this]() { OpenAccountManagementDialog(); });
	m_pAccountManagementAction->setEnabled(false);
	addManagementAction(managementProcessMenu, "工艺参数", [this, openInManagement]() { openInManagement([this]() { OpenWeldProcessDialog(); }); });
	addManagementAction(managementProcessMenu, "测量焊接参数", [this, openInManagement]() { openInManagement([this]() { OpenPreciseMeasureEditDialog(); }); });
	addManagementAction(managementCameraMenu, "相机参数", [this, openInManagement]() { openInManagement([this]() { OpenCameraParamDialog(); }); });
	addManagementAction(managementCameraMenu, "焊道补偿", [this, openInManagement]() { openInManagement([this]() { OpenWeldSeamCompDialog(); }); });
	addManagementAction(managementDebugMenu, "点动控制", [this, openInManagement]() { openInManagement([this]() { OpenRobotJogDialog(); }); });
	addManagementAction(managementDebugMenu, "功能测试", [this, openInManagement]() { openInManagement([this]() { OpenFunctionTestDialog(); }); });

	m_pManagementStack = new QStackedWidget(m_pManagementPage);
	m_pManagementStack->setContentsMargins(0, 0, 0, 0);
	m_pManagementStack->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	m_pManagementHomePage = new QWidget(m_pManagementStack);
	QVBoxLayout* managementHomeLayout = new QVBoxLayout(m_pManagementHomePage);
	managementHomeLayout->setContentsMargins(0, 0, 0, 0);
	managementHomeLayout->setSpacing(12);
	m_pManagementStack->addWidget(m_pManagementHomePage);
	managementLayout->addWidget(m_pManagementStack, 1);

	QHBoxLayout* managementTitleLayout = new QHBoxLayout();
	QLabel* managementTitleLabel = new QLabel("管理页面", m_pManagementHomePage);
	managementTitleLabel->setStyleSheet("font-size: 24px; font-weight: bold; color: #F7FCFC;");
	m_pManagementCameraReceiveModeBtn = new QPushButton("相机接收：共享", m_pManagementHomePage);
	m_pManagementCameraReceiveModeBtn->setCheckable(true);
	m_pManagementCameraReceiveModeBtn->setMinimumHeight(34);
	m_pManagementCameraReceiveModeBtn->setMinimumWidth(150);
	m_pManagementCameraReceiveModeBtn->setStyleSheet("QPushButton { padding: 6px 14px; font-size: 14px; border-radius: 10px; }");
	QLabel* touchKeyboardLabel = new QLabel("虚拟键盘：", m_pManagementHomePage);
	touchKeyboardLabel->setStyleSheet("QLabel { color: #9ED8DB; padding-left: 8px; }");
	m_pTouchKeyboardModeCombo = new QComboBox(m_pManagementHomePage);
	m_pTouchKeyboardModeCombo->addItem(TouchKeyboardManager::ModeDisplayName(TouchKeyboardManager::Mode::Auto), TouchKeyboardManager::ModeToStorageString(TouchKeyboardManager::Mode::Auto));
	m_pTouchKeyboardModeCombo->addItem(TouchKeyboardManager::ModeDisplayName(TouchKeyboardManager::Mode::Always), TouchKeyboardManager::ModeToStorageString(TouchKeyboardManager::Mode::Always));
	m_pTouchKeyboardModeCombo->addItem(TouchKeyboardManager::ModeDisplayName(TouchKeyboardManager::Mode::Never), TouchKeyboardManager::ModeToStorageString(TouchKeyboardManager::Mode::Never));
	m_pTouchKeyboardModeCombo->setFixedSize(110, 34);
	m_pTouchKeyboardModeCombo->setToolTip("自动：触摸输入时弹出；总是：输入框聚焦即弹出；从不：禁用虚拟键盘。");
	m_pManagementUserLabel = new QLabel(m_pManagementHomePage);
	m_pManagementUserLabel->setStyleSheet("QLabel { color: #9ED8DB; padding: 5px 10px; border: 1px solid #2E4656; border-radius: 10px; background: #101923; }");
	managementTitleLayout->addWidget(managementTitleLabel);
	managementTitleLayout->addStretch(1);
	managementTitleLayout->addWidget(m_pManagementCameraReceiveModeBtn);
	managementTitleLayout->addWidget(touchKeyboardLabel);
	managementTitleLayout->addWidget(m_pTouchKeyboardModeCombo);
	managementTitleLayout->addWidget(m_pManagementUserLabel);
	managementHomeLayout->addLayout(managementTitleLayout);

	m_pPermissionHintLabel = new QLabel("管理功能请从上方菜单栏进入，下面只保留账号和状态信息。", m_pManagementHomePage);
	m_pPermissionHintLabel->setWordWrap(true);
	m_pPermissionHintLabel->setStyleSheet("QLabel { color: #AFC8CE; font-size: 14px; }");
	managementHomeLayout->addWidget(m_pPermissionHintLabel);

	QGroupBox* managementInfoGroup = new QGroupBox("管理说明", m_pManagementHomePage);
	QVBoxLayout* managementInfoLayout = new QVBoxLayout(managementInfoGroup);
	QLabel* managementInfoLabel = new QLabel(
		"管理页面仅保留菜单栏入口，不再放大按钮。只有工程师或管理员可以进入管理页面；其中“账号管理”仅管理员可用。",
		managementInfoGroup);
	managementInfoLabel->setWordWrap(true);
	managementInfoLayout->addWidget(managementInfoLabel);
	QPlainTextEdit* managementLogText = new QPlainTextEdit(managementInfoGroup);
	managementLogText->setReadOnly(true);
	managementLogText->document()->setMaximumBlockCount(120);
	managementLogText->setPlainText("管理页面提示：请从菜单栏打开工艺、标定、补偿、点动和功能测试。");
	managementInfoLayout->addWidget(managementLogText, 1);
	managementHomeLayout->addWidget(managementInfoGroup, 1);

	connect(m_pManagementCameraReceiveModeBtn, &QPushButton::toggled, this, [this](bool checked)
		{
			SetSharedScanCameraReceiverMode(checked);
		});
	connect(m_pTouchKeyboardModeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int index)
		{
			if (m_pTouchKeyboardModeCombo == nullptr || m_pTouchKeyboardManager == nullptr || index < 0)
			{
				return;
			}

			m_pTouchKeyboardManager->SetMode(TouchKeyboardManager::ModeFromStorageString(m_pTouchKeyboardModeCombo->itemData(index).toString()));
		});

	QGroupBox* entryGroup = new QGroupBox("常用功能", mainPanel);
	QGridLayout* entryLayout = new QGridLayout(entryGroup);
	entryLayout->setSpacing(12);
	m_pCameraParamBtn = new QPushButton("相机参数", entryGroup);
	m_pWeldSeamCompBtn = new QPushButton("焊道补偿", entryGroup);
	const QList<QPushButton*> entryButtons = {
		ui.RunTest,
		ui.WeldProcessBtn,
		ui.FunctionTestBtn,
		ui.MeasureThenWeldBtn,
		ui.PreciseMeasureEditBtn,
		m_pWeldSeamCompBtn,
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
	entryGroup->hide();

	m_robotOperationWidgets = {
		m_pDashboardConnectBtn,
		m_pDashboardClearAlarmBtn,
		m_pDashboardModeBtn,
		quickReadTool1Btn,
		quickMeasureBtn,
		quickPreviewBtn,
		quickJogBtn,
		quickPositionBtn,
		quickCalibrationBtn,
		m_pWeldSeamCompBtn,
		m_pCameraParamBtn
	};
	m_cameraParamDependentWidgets = {
		quickMeasureBtn,
		quickPreviewBtn,
		ui.MeasureThenWeldBtn,
		ui.GrooveCameraTestBtn
	};
	m_handEyeDependentWidgets = {
		quickMeasureBtn,
		ui.MeasureThenWeldBtn
	};

	auto addCommandAction = [this](QMenu* menu, const QString& text, const std::function<void()>& handler, bool addToToolbar = true) -> QAction*
		{
			QAction* action = new QAction(text, this);
			if (menu != nullptr)
			{
				menu->addAction(action);
			}
			if (addToToolbar && ui.mainToolBar != nullptr)
			{
				ui.mainToolBar->addAction(action);
			}
			connect(action, &QAction::triggered, this, [handler]()
				{
					if (handler)
					{
						handler();
					}
				});
			return action;
		};

	auto addToolbarSeparator = [this]()
		{
			if (ui.mainToolBar != nullptr)
			{
				ui.mainToolBar->addSeparator();
			}
		};

	QMenu* fileMenu = ui.menuBar != nullptr ? ui.menuBar->addMenu("文件(F)") : nullptr;
	QMenu* robotMenu = ui.menuBar != nullptr ? ui.menuBar->addMenu("机器人(R)") : nullptr;
	QMenu* measureMenu = ui.menuBar != nullptr ? ui.menuBar->addMenu("测量(M)") : nullptr;
	QMenu* cameraMenu = ui.menuBar != nullptr ? ui.menuBar->addMenu("相机(C)") : nullptr;
	QMenu* weldMenu = ui.menuBar != nullptr ? ui.menuBar->addMenu("焊接(W)") : nullptr;
	QMenu* debugMenu = ui.menuBar != nullptr ? ui.menuBar->addMenu("调试(D)") : nullptr;
	QMenu* logMenu = ui.menuBar != nullptr ? ui.menuBar->addMenu("日志(L)") : nullptr;
	QMenu* helpMenu = ui.menuBar != nullptr ? ui.menuBar->addMenu("帮助(H)") : nullptr;

	addCommandAction(fileMenu, "首页监控", [this]() { ShowDashboardPage(); }, false);
	addCommandAction(fileMenu, "管理页面", [this]() { ShowManagementPage(); }, false);
	if (fileMenu != nullptr)
	{
		fileMenu->addSeparator();
	}
	addCommandAction(robotMenu, "运行测试", [this]() { RobotRunTest(); });
	addCommandAction(robotMenu, "连接服务", [this]() { FanucConnectTest(); });
	addCommandAction(robotMenu, "断开服务", [this]() { FanucDisconnectTest(); });
	addCommandAction(robotMenu, "点动控制", [this]() { OpenRobotJogDialog(); });
	addToolbarSeparator();
	addCommandAction(measureMenu, "先测后焊", [this]() { OpenMeasureThenWeldDialog(); });
	addCommandAction(measureMenu, "测量焊接参数", [this]() { OpenPreciseMeasureEditDialog(); });
	addToolbarSeparator();
	addCommandAction(cameraMenu, "相机参数", [this]() { OpenCameraParamDialog(); });
	QAction* cameraPreviewAction = new QAction("坡口相机预览", this);
	cameraPreviewAction->setCheckable(true);
	cameraPreviewAction->setChecked(ui.GrooveCameraTestBtn != nullptr && ui.GrooveCameraTestBtn->isChecked());
	if (cameraMenu != nullptr)
	{
		cameraMenu->addAction(cameraPreviewAction);
	}
	if (ui.mainToolBar != nullptr)
	{
		ui.mainToolBar->addAction(cameraPreviewAction);
	}
	connect(cameraPreviewAction, &QAction::toggled, ui.GrooveCameraTestBtn, &QPushButton::setChecked);
	connect(ui.GrooveCameraTestBtn, &QPushButton::toggled, cameraPreviewAction, &QAction::setChecked);
	addToolbarSeparator();
	addCommandAction(weldMenu, "工艺参数", [this]() { OpenWeldProcessDialog(); });
	addCommandAction(weldMenu, "焊道补偿", [this]() { OpenWeldSeamCompDialog(); });
	addToolbarSeparator();
	addCommandAction(debugMenu, "功能测试", [this]() { OpenFunctionTestDialog(); });
	if (debugMenu != nullptr)
	{
		debugMenu->addSeparator();
	}
	addCommandAction(debugMenu, "读取当前位置", [this]() { FanucGetCurrentPosTest(); }, false);
	addCommandAction(debugMenu, "读取当前脉冲", [this]() { FanucGetCurrentPulseTest(); }, false);
	addCommandAction(debugMenu, "检查完成状态", [this]() { FanucCheckDoneTest(); }, false);
	addCommandAction(debugMenu, "读写整型寄存器", [this]() { if (RequirePermission(kRoleEngineer, "读写整型寄存器")) FanucSetGetIntTest(); }, false);
	addCommandAction(debugMenu, "设置TP速度", [this]() { if (RequirePermission(kRoleEngineer, "设置TP速度")) FanucSetTpSpeedTest(); }, false);
	addCommandAction(debugMenu, "调用程序", [this]() { if (RequirePermission(kRoleEngineer, "调用程序")) FanucCallJobTest(); }, false);
	addCommandAction(debugMenu, "上传LS", [this]() { if (RequirePermission(kRoleEngineer, "上传LS")) FanucUploadLsTest(); }, false);
	addCommandAction(debugMenu, "MOVL测试", [this]() { if (RequirePermission(kRoleEngineer, "MOVL测试")) FanucMovlTest(); }, false);
	addCommandAction(debugMenu, "MOVJ测试", [this]() { if (RequirePermission(kRoleEngineer, "MOVJ测试")) FanucMovjTest(); }, false);
	addCommandAction(debugMenu, "运动到零位", [this]() { if (RequirePermission(kRoleEngineer, "运动到零位")) FanucMoveZeroTest(); }, false);

	const QList<QPair<QString, QString>> menuLogActions = {
		{ "系统日志", "Log/Log.txt" },
		{ "运行日志", "Log/RobotRunLog.txt" },
		{ "机器人A日志", "Log/RobotALog.txt" },
		{ "控制单元日志", "Log/ContralUnit.txt" },
		{ "焊接日志", "Log/WeldProcessFile.txt" }
	};
	for (const QPair<QString, QString>& item : menuLogActions)
	{
		addCommandAction(logMenu, item.first, [this, item]() { LoadRobotLogFile(item.second, true); }, false);
	}
	QAction* aboutAction = addCommandAction(helpMenu, "关于", [this]() { OpenAboutDialog(); }, false);
	if (fileMenu != nullptr)
	{
		fileMenu->addAction(aboutAction);
		fileMenu->addSeparator();
	}
	addCommandAction(fileMenu, "退出", [this]() { close(); }, false);

	QSplitter* infoSplitter = new QSplitter(Qt::Horizontal);
	infoSplitter->setChildrenCollapsible(false);

	QSplitter* robotInfoSplitter = new QSplitter(Qt::Vertical);
	robotInfoSplitter->setChildrenCollapsible(false);

	QGroupBox* monitorGroup = new QGroupBox("机器人监控");
	QVBoxLayout* monitorLayout = new QVBoxLayout(monitorGroup);
	ui.FanucMonitorText->setLineWrapMode(QPlainTextEdit::WidgetWidth);
	ui.FanucMonitorText->setMinimumWidth(420);
	monitorGroup->setMaximumWidth(1050);
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
	m_pRobotLogText->document()->setMaximumBlockCount(1200);
	m_pRobotLogText->setPlainText("日志：等待读取...");
	logLayout->addWidget(m_pRobotLogText, 1);

	robotInfoSplitter->addWidget(monitorGroup);
	robotInfoSplitter->addWidget(logGroup);
	robotInfoSplitter->setStretchFactor(0, 1);
	robotInfoSplitter->setStretchFactor(1, 1);

	QGroupBox* cameraGroup = new QGroupBox("坡口相机数据");
	cameraGroup->setMinimumWidth(420);
	QVBoxLayout* cameraLayout = new QVBoxLayout(cameraGroup);
	cameraLayout->setSpacing(8);
	ui.GrooveCameraText->setMinimumHeight(220);
	ui.GrooveCameraText->setMaximumHeight(QWIDGETSIZE_MAX);
	cameraLayout->addWidget(ui.GrooveCameraText, 1);
	infoSplitter->addWidget(robotInfoSplitter);
	infoSplitter->addWidget(cameraGroup);
	infoSplitter->setStretchFactor(0, 3);
	infoSplitter->setStretchFactor(1, 2);
	infoSplitter->setSizes(QList<int>() << 1100 << 520);
	dashboardLayout->addWidget(infoSplitter, 1);
	dashboardLayout->addWidget(entryGroup, 0);

	connect(m_pCurrentUserButton, &QPushButton::clicked, this, &QtWidgetsApplication4::ShowCurrentUserMenu);
	connect(m_pDashboardDebugLogBtn, &QPushButton::toggled, this, &QtWidgetsApplication4::SetDebugLogMode);
	connect(m_pDashboardConnectBtn, &QPushButton::clicked, this, &QtWidgetsApplication4::ToggleCurrentRobotConnection);
	connect(m_pDashboardClearAlarmBtn, &QPushButton::clicked, this, &QtWidgetsApplication4::RobotClearAlarmTest);
	connect(m_pDashboardModeBtn, &QPushButton::clicked, this, &QtWidgetsApplication4::RobotSwitchStepMode);
	connect(quickReadTool1Btn, &QPushButton::clicked, this, &QtWidgetsApplication4::ReadTool1ToGunTool);
	connect(quickMeasureBtn, &QPushButton::clicked, this, &QtWidgetsApplication4::OpenMeasureThenWeldDialog);
	connect(quickPositionBtn, &QPushButton::clicked, this, &QtWidgetsApplication4::FanucGetCurrentPosTest);
	connect(quickJogBtn, &QPushButton::clicked, this, &QtWidgetsApplication4::OpenRobotJogDialog);
	connect(quickCalibrationBtn, &QPushButton::clicked, this, &QtWidgetsApplication4::OpenCameraParamDialog);
	connect(quickPreviewBtn, &QPushButton::toggled, ui.GrooveCameraTestBtn, &QPushButton::setChecked);
	connect(ui.GrooveCameraTestBtn, &QPushButton::toggled, quickPreviewBtn, &QPushButton::setChecked);
	connect(m_pRememberPasswordCheck, &QCheckBox::toggled, this, [this](bool checked)
		{
			if (m_pAutoLoginCheck != nullptr)
			{
				m_pAutoLoginCheck->setEnabled(checked);
				if (!checked)
				{
					m_pAutoLoginCheck->setChecked(false);
				}
			}
		});

	connect(ui.RunTest, &QPushButton::clicked, this, &QtWidgetsApplication4::RobotRunTest);
	connect(ui.WeldProcessBtn, &QPushButton::clicked, this, &QtWidgetsApplication4::OpenWeldProcessDialog);
	connect(ui.FunctionTestBtn, &QPushButton::clicked, this, &QtWidgetsApplication4::OpenFunctionTestDialog);
	connect(ui.MeasureThenWeldBtn, &QPushButton::clicked, this, &QtWidgetsApplication4::OpenMeasureThenWeldDialog);
	connect(ui.PreciseMeasureEditBtn, &QPushButton::clicked, this, &QtWidgetsApplication4::OpenPreciseMeasureEditDialog);
	connect(m_pWeldSeamCompBtn, &QPushButton::clicked, this, &QtWidgetsApplication4::OpenWeldSeamCompDialog);
	connect(m_pCameraParamBtn, &QPushButton::clicked, this, &QtWidgetsApplication4::OpenCameraParamDialog);
	connect(aboutButton, &QPushButton::clicked, this, &QtWidgetsApplication4::OpenAboutDialog);
	connect(ui.FanucConnectBtn, &QPushButton::clicked, this, &QtWidgetsApplication4::ToggleCurrentRobotConnection);
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

	EnsureDefaultAdminAccount();
	RefreshAccountUi();
	LoadCameraReceiveMode();
	RefreshTouchKeyboardModeUi();
	LoadLoginState();
	ShowAuthPage();
	TryAutoLogin();

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
			RobotDriverAdaptor* pRobotDriver = GetCurrentRobotDriver(nullptr);
			if (pRobotDriver == nullptr)
			{
				if (ui.FanucMonitorText != nullptr && ui.FanucMonitorText->toPlainText().isEmpty())
				{
					ui.FanucMonitorText->setPlainText("状态: 未选择可用机器人驱动。\n现场操作已禁用，管理页面仍可使用。");
				}
				return;
			}

			long long robotMs = 0;
			long long pcRecvMs = 0;
			T_ROBOT_COORS pos;
			T_ANGLE_PULSE pulse;
			int done = -1;
			RobotDriverAdaptor::StateSnapshot snapshot;
			const bool hasSnapshot = pRobotDriver->LatestStateSnapshot(snapshot);
			if (hasSnapshot)
			{
				robotMs = snapshot.robotMs;
				pcRecvMs = snapshot.pcRecvMs;
				pos = snapshot.pose;
				pulse = snapshot.pulse;
				done = snapshot.done;
			}
			const QString stateText = done == 0 ? "运行中" : (done == 1 ? "停止/完成" : QString("未知/异常(%1)").arg(done));
			QString monitorText = QString(
				"状态: %1\n"
				"robot_ms=%2  pc_recv_ms=%3  cache=%4/200\n"
				"位置: X=%5  Y=%6  Z=%7  W=%8  P=%9  R=%10\n"
				"脉冲: S=%11  L=%12  U=%13  R=%14  B=%15  T=%16  EX1=%17  EX2=%18  EX3=%19")
				.arg(stateText)
				.arg(robotMs)
				.arg(pcRecvMs)
				.arg(hasSnapshot ? pRobotDriver->StateMonitorCachedCount() : 0)
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
	InitializeScanCameraRuntimes();
	RefreshRobotSelectorUi();
	RefreshRobotOperationAvailability();
	QTimer* robotConnectionUiTimer = new QTimer(this);
	connect(robotConnectionUiTimer, &QTimer::timeout, this, &QtWidgetsApplication4::RefreshDashboardConnectionState);
	robotConnectionUiTimer->start(1000);
	if (m_pRobotSelectorCombo != nullptr)
	{
		connect(m_pRobotSelectorCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int)
			{
				if (m_pRobotSelectorCombo != nullptr)
				{
					const int unitIndex = m_pRobotSelectorCombo->currentData().toInt();
					if (unitIndex >= 0)
					{
						QString issueText;
						if (!IsRobotUnitDriverReady(unitIndex, &issueText))
						{
							QMessageBox::warning(this, "机器人类型错误",
								QString("%1\n已切回可用机器人；如果没有可用机器人，主页现场操作将保持禁用。")
									.arg(issueText));
							RefreshRobotSelectorUi();
							RefreshRobotOperationAvailability();
							return;
						}
						m_nCurrentRobotUnitIndex = unitIndex;
					}
					RefreshRobotOperationAvailability();
				}
			});
	}

}

QtWidgetsApplication4::~QtWidgetsApplication4()
{
	const QList<QPointer<MeasureThenWeldDialog>> measureThenWeldPages = m_measureThenWeldPages.values();
	for (const QPointer<MeasureThenWeldDialog>& guardedPage : measureThenWeldPages)
	{
		if (MeasureThenWeldDialog* page = guardedPage.data())
		{
			QObject::disconnect(page, nullptr, this, nullptr);
			delete page;
		}
	}
	m_measureThenWeldPages.clear();
	m_pMeasureThenWeldPage = nullptr;
	m_nMeasureThenWeldPageUnitIndex = -1;

	const QList<QPointer<RobotJogDialog>> robotJogPages = m_robotJogPages.values();
	for (const QPointer<RobotJogDialog>& guardedPage : robotJogPages)
	{
		if (RobotJogDialog* page = guardedPage.data())
		{
			QObject::disconnect(page, nullptr, this, nullptr);
			delete page;
		}
	}
	m_robotJogPages.clear();
	m_pRobotJogPage = nullptr;
	m_nRobotJogPageUnitIndex = -1;

	if (m_grooveCameraDisplayTimer != nullptr)
	{
		m_grooveCameraDisplayTimer->stop();
	}
	if (m_robotLogDisplayTimer != nullptr)
	{
		m_robotLogDisplayTimer->stop();
	}
	StopScanCameraRuntimes();
	delete m_pContralUnit;
	m_pContralUnit = nullptr;
}

bool QtWidgetsApplication4::eventFilter(QObject* watched, QEvent* event)
{
	if (event != nullptr && event->type() == QEvent::Resize)
	{
		if (QWidget* page = qobject_cast<QWidget*>(watched))
		{
			PositionEmbeddedBackButton(page);
		}
	}
	if (event != nullptr && event->type() == QEvent::Close)
	{
		if (watched == m_pWeldProcessPage
			|| watched == m_pFunctionTestPage
			|| watched == m_pMeasureThenWeldPage
			|| watched == m_pPreciseMeasureEditPage
			|| watched == m_pWeldSeamCompPage
			|| watched == m_pCameraParamPage
			|| watched == m_pRobotJogPage
			|| watched == m_pAccountManagementPage
			|| watched == m_pControlUnitManagementPage)
		{
			QWidget* page = qobject_cast<QWidget*>(watched);
			const bool inManagementStack = page != nullptr
				&& m_pManagementStack != nullptr
				&& m_pManagementStack->indexOf(page) >= 0;
			QTimer::singleShot(0, this, [this, inManagementStack]()
				{
					if (inManagementStack)
					{
						ShowManagementHomePage();
					}
					else
					{
						ShowDashboardPage();
					}
				});
		}
	}

	return QMainWindow::eventFilter(watched, event);
}

void QtWidgetsApplication4::resizeEvent(QResizeEvent* event)
{
	QMainWindow::resizeEvent(event);
	if (m_pMainStack == nullptr)
	{
		return;
	}

	QWidget* currentPage = m_pMainStack->currentWidget();
	if (currentPage != nullptr && currentPage != m_pDashboardPage)
	{
		currentPage->setGeometry(m_pMainStack->contentsRect());
		PositionEmbeddedBackButton(currentPage);
		if (QLayout* pageLayout = currentPage->layout())
		{
			pageLayout->activate();
		}
	}
}

void QtWidgetsApplication4::ShowDashboardPage()
{
	setMaximumSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX);
	setMinimumSize(1280, 700);
	if (isMaximized() || isFullScreen())
	{
		showNormal();
	}
	resize(minimumSize());
	CenterWindowOnScreen(this);

	if (m_pMainStack != nullptr && m_pDashboardPage != nullptr)
	{
		m_pMainStack->setCurrentWidget(m_pDashboardPage);
	}
	if (m_pManagementPage != nullptr)
	{
		m_pManagementPage->hide();
	}
	if (m_pMainStack != nullptr)
	{
		m_pMainStack->show();
	}
}

void QtWidgetsApplication4::ShowCurrentUserMenu()
{
	if (m_pCurrentUserButton == nullptr)
	{
		return;
	}

	QMenu menu(this);
	menu.setStyleSheet(
		"QMenu { background: #000000; color: #ECF3F4; border: 1px solid #3C6173; padding: 4px; }"
		"QMenu::item { padding: 8px 28px 8px 14px; min-width: 120px; }"
		"QMenu::item:selected { background: #2D5465; color: #FFFFFF; }");

	QAction* logoutAction = menu.addAction("退出登录");
	QAction* managementAction = nullptr;
	if (RoleLevel(m_sCurrentUserRole) >= RoleLevel(kRoleAdmin))
	{
		managementAction = menu.addAction("管理界面");
	}

	const QPoint menuPos = m_pCurrentUserButton->mapToGlobal(QPoint(0, m_pCurrentUserButton->height() + 4));
	QAction* selectedAction = menu.exec(menuPos);
	if (selectedAction == logoutAction)
	{
		LogoutCurrentAccount();
	}
	else if (selectedAction != nullptr && selectedAction == managementAction)
	{
		ShowManagementPage();
	}
}

void QtWidgetsApplication4::ShowManagementPage()
{
	if (RoleLevel(m_sCurrentUserRole) < RoleLevel(kRoleEngineer))
	{
		const QMessageBox::StandardButton button = QMessageBox::question(
			this,
			"权限不足",
			QString("管理页面需要工程师或管理员权限。\n当前用户：%1（%2）\n是否重新登录？")
				.arg(m_sCurrentUserName, RoleDisplayName(m_sCurrentUserRole)),
			QMessageBox::Yes | QMessageBox::No,
			QMessageBox::Yes);
		if (button == QMessageBox::Yes)
		{
			m_bPendingOpenManagementAfterLogin = true;
			ShowAuthPage("管理页面需要工程师或管理员权限，请重新登录。");
		}
		return;
	}

	if (m_pManagementPage != nullptr)
	{
		m_pManagementPage->setMaximumSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX);
		m_pManagementPage->setMinimumSize(920, 700);
		RefreshAccountUi();
		ShowManagementHomePage();
		ShowMaximizedWithUnifiedChrome(m_pManagementPage);
		m_pManagementPage->raise();
		m_pManagementPage->activateWindow();
	}
}

void QtWidgetsApplication4::ShowManagementHomePage()
{
	if (m_pManagementStack != nullptr && m_pManagementHomePage != nullptr)
	{
		m_pManagementStack->setCurrentWidget(m_pManagementHomePage);
	}
	if (m_pManagementPage != nullptr)
	{
		ShowMaximizedWithUnifiedChrome(m_pManagementPage);
		m_pManagementPage->raise();
		m_pManagementPage->activateWindow();
	}
}

int QtWidgetsApplication4::CurrentRobotUnitIndex() const
{
    if (m_pRobotSelectorCombo != nullptr)
    {
        const int unitIndex = m_pRobotSelectorCombo->currentData().toInt();
        if (unitIndex >= 0)
        {
            return unitIndex;
        }
    }
    if (m_nCurrentRobotUnitIndex >= 0
        && m_pContralUnit != nullptr
        && m_nCurrentRobotUnitIndex < static_cast<int>(m_pContralUnit->m_vtContralUnitInfo.size()))
    {
        return m_nCurrentRobotUnitIndex;
    }
    if (m_pContralUnit == nullptr || m_pContralUnit->m_vtContralUnitInfo.empty())
    {
        return -1;
    }
    return 0;
}

QString QtWidgetsApplication4::CurrentRobotName() const
{
	const T_CONTRAL_UNIT* unitInfo = CurrentContralUnit();
	if (unitInfo == nullptr)
	{
		return QString();
	}
	RobotDriverAdaptor* driver = static_cast<RobotDriverAdaptor*>(unitInfo->pUnitDriver);
	const QString driverName = DecodeConfigText(driver != nullptr ? driver->m_sRobotName : std::string()).trimmed();
	if (!driverName.isEmpty())
	{
		return driverName;
	}
	return DecodeConfigText(unitInfo->sUnitName).trimmed();
}

bool QtWidgetsApplication4::IsCurrentRobotSetupReady(bool requireCameraParam, bool requireHandEye, QString* issueText) const
{
	if (issueText != nullptr)
	{
		issueText->clear();
	}
	const QString robotName = CurrentRobotName();
	if (robotName.isEmpty())
	{
		if (issueText != nullptr)
		{
			*issueText = "未选择机器人。";
		}
		return false;
	}

	const RobotSetupStatus status = LoadRobotSetupStatus(robotName, true);
	QStringList issues;
	if (!status.enabled)
	{
		issues << "当前控制单元已停用。";
	}
	if (requireCameraParam && !status.cameraParamReady)
	{
		issues << "相机参数尚未完成，请先进入“相机参数”保存当前相机设置。";
	}
	if (requireHandEye && !status.handEyeReady)
	{
		issues << "手眼标定尚未完成，请先进入“相机参数 -> 手眼标定”计算并保存矩阵。";
	}
	if (!issues.isEmpty())
	{
		if (issueText != nullptr)
		{
			*issueText = issues.join('\n');
		}
		return false;
	}
	return true;
}

const T_CONTRAL_UNIT* QtWidgetsApplication4::CurrentContralUnit() const
{
    const int unitIndex = CurrentRobotUnitIndex();
    if (m_pContralUnit == nullptr || unitIndex < 0)
    {
        return nullptr;
    }
    for (const T_CONTRAL_UNIT& unitInfo : m_pContralUnit->m_vtContralUnitInfo)
    {
        if (unitInfo.nUnitNo == unitIndex)
        {
            return &unitInfo;
        }
    }
    if (unitIndex < static_cast<int>(m_pContralUnit->m_vtContralUnitInfo.size()))
    {
        return &m_pContralUnit->m_vtContralUnitInfo[unitIndex];
    }
    return nullptr;
}

bool QtWidgetsApplication4::IsRobotUnitDriverReady(int unitIndex, QString* issueText) const
{
    if (issueText != nullptr)
    {
        issueText->clear();
    }
    if (m_pContralUnit == nullptr || unitIndex < 0)
    {
        if (issueText != nullptr)
        {
            *issueText = "未选择可用机器人。";
        }
        return false;
    }

    const T_CONTRAL_UNIT* unitInfo = nullptr;
    for (const T_CONTRAL_UNIT& candidate : m_pContralUnit->m_vtContralUnitInfo)
    {
        if (candidate.nUnitNo == unitIndex)
        {
            unitInfo = &candidate;
            break;
        }
    }
    if (unitInfo == nullptr && unitIndex < static_cast<int>(m_pContralUnit->m_vtContralUnitInfo.size()))
    {
        unitInfo = &m_pContralUnit->m_vtContralUnitInfo[unitIndex];
    }
    if (unitInfo == nullptr)
    {
        if (issueText != nullptr)
        {
            *issueText = QString("未找到编号为 %1 的机器人配置。").arg(unitIndex);
        }
        return false;
    }

    const QString robotName = DecodeConfigText(unitInfo->sUnitName);
    const QString displayName = DecodeConfigText(unitInfo->sChineseName);
    const QString robotLabel = displayName.isEmpty()
        ? robotName
        : QString("%1 (%2)").arg(displayName, robotName);
    RobotDriverAdaptor* pRobotDriverAdaptor = static_cast<RobotDriverAdaptor*>(unitInfo->pUnitDriver);
    if (pRobotDriverAdaptor == nullptr)
    {
        if (issueText != nullptr)
        {
            *issueText = QString("机器人类型错误：%1 的 RobotType 不支持或驱动未创建。").arg(robotLabel);
        }
        return false;
    }
    return true;
}

int QtWidgetsApplication4::FindFirstReadyRobotUnitIndex() const
{
    if (m_pContralUnit == nullptr)
    {
        return -1;
    }
    for (const T_CONTRAL_UNIT& unitInfo : m_pContralUnit->m_vtContralUnitInfo)
    {
        if (IsRobotUnitDriverReady(unitInfo.nUnitNo))
        {
            return unitInfo.nUnitNo;
        }
    }
    return -1;
}

void QtWidgetsApplication4::RefreshRobotOperationAvailability()
{
    QString issueText;
    const bool hasReadyDriver = IsRobotUnitDriverReady(CurrentRobotUnitIndex(), &issueText);
	const QString robotName = CurrentRobotName();
	const RobotSetupStatus setupStatus = robotName.isEmpty()
		? RobotSetupStatus()
		: LoadRobotSetupStatus(robotName, true);
	auto containsWidget = [](const QList<QPointer<QWidget>>& widgets, QWidget* target) -> bool
		{
			for (const QPointer<QWidget>& widget : widgets)
			{
				if (widget.data() == target)
				{
					return true;
				}
			}
			return false;
		};
    for (const QPointer<QWidget>& widget : m_robotOperationWidgets)
    {
        if (!widget.isNull())
        {
			QStringList disableReasons;
			bool enabled = hasReadyDriver;
			if (!hasReadyDriver)
			{
				disableReasons << "未选择可用机器人驱动，现场操作已禁用。";
			}
			if (enabled && !setupStatus.enabled)
			{
				enabled = false;
				disableReasons << "当前控制单元已停用。";
			}
			if (enabled && containsWidget(m_cameraParamDependentWidgets, widget.data()) && !setupStatus.cameraParamReady)
			{
				enabled = false;
				disableReasons << "相机参数尚未完成，请先进入“相机参数”保存当前相机设置。";
			}
			if (enabled && containsWidget(m_handEyeDependentWidgets, widget.data()) && !setupStatus.handEyeReady)
			{
				enabled = false;
				disableReasons << "手眼标定尚未完成，请先进入“相机参数 -> 手眼标定”计算并保存矩阵。";
			}
            widget->setEnabled(enabled);
            widget->setToolTip(enabled ? QString() : disableReasons.join('\n'));
        }
    }
    RefreshDashboardConnectionState();
    if (!hasReadyDriver)
    {
        if (ui.FanucMonitorText != nullptr)
        {
            ui.FanucMonitorText->setPlainText(QString("状态: 机器人类型错误/不可用\n%1\n现场操作已禁用，管理页面仍可使用。").arg(issueText));
        }
    }
}

bool QtWidgetsApplication4::IsCurrentRobotConnected()
{
	QString issueText;
	if (!IsRobotUnitDriverReady(CurrentRobotUnitIndex(), &issueText))
	{
		return false;
	}
	const T_CONTRAL_UNIT* unitInfo = CurrentContralUnit();
	if (unitInfo == nullptr)
	{
		return false;
	}
	RobotDriverAdaptor* driver = static_cast<RobotDriverAdaptor*>(unitInfo->pUnitDriver);
	return driver != nullptr && driver->IsConnected();
}

void QtWidgetsApplication4::RefreshDashboardConnectionState()
{
	const bool connected = IsCurrentRobotConnected();
	RobotDriverAdaptor* currentDriver = nullptr;
	if (const T_CONTRAL_UNIT* unitInfo = CurrentContralUnit())
	{
		currentDriver = static_cast<RobotDriverAdaptor*>(unitInfo->pUnitDriver);
	}
	const bool isStepDriver = dynamic_cast<STEPRobotCtrl*>(currentDriver) != nullptr;
	if (m_pDashboardConnectBtn != nullptr)
	{
		m_pDashboardConnectBtn->setText(connected
			? "断开连接"
			: "连接服务");
		m_pDashboardConnectBtn->setToolTip(connected
			? "当前机器人已连接，点击后断开连接。"
			: "当前机器人未连接，点击后连接并初始化通讯。");
	}
	if (ui.FanucConnectBtn != nullptr)
	{
		ui.FanucConnectBtn->setText(connected ? "断开连接" : "连接服务");
	}
	if (m_pDashboardModeBtn != nullptr)
	{
		m_pDashboardModeBtn->setVisible(isStepDriver);
		m_pDashboardModeBtn->setToolTip(isStepDriver
			? "切换新时达机器人模式。"
			: "FANUC 不显示新时达模式切换。");
	}
}

void QtWidgetsApplication4::ToggleCurrentRobotConnection()
{
	if (IsCurrentRobotConnected())
	{
		FanucDisconnectTest();
	}
	else
	{
		FanucConnectTest();
	}
}

RobotDriverAdaptor* QtWidgetsApplication4::GetCurrentRobotDriver(QWidget* parent)
{
    QString issueText;
    if (!IsRobotUnitDriverReady(CurrentRobotUnitIndex(), &issueText))
    {
        const int fallbackUnitIndex = FindFirstReadyRobotUnitIndex();
        if (fallbackUnitIndex >= 0 && fallbackUnitIndex != CurrentRobotUnitIndex())
        {
            if (m_pRobotSelectorCombo != nullptr)
            {
                const int comboIndex = m_pRobotSelectorCombo->findData(fallbackUnitIndex);
                if (comboIndex >= 0)
                {
                    QSignalBlocker blocker(m_pRobotSelectorCombo);
                    m_pRobotSelectorCombo->setCurrentIndex(comboIndex);
                }
            }
            m_nCurrentRobotUnitIndex = fallbackUnitIndex;
            RefreshRobotOperationAvailability();
            if (parent != nullptr)
            {
                QMessageBox::warning(parent, "机器人类型错误",
                    QString("%1\n已切回可用机器人，请确认后重新执行操作。").arg(issueText));
            }
            return nullptr;
        }

        m_nCurrentRobotUnitIndex = -1;
        RefreshRobotOperationAvailability();
        if (parent != nullptr)
        {
            QMessageBox::warning(parent, "机器人类型错误",
                QString("%1\n没有可用机器人驱动，主页现场操作已禁用，管理页面仍可使用。").arg(issueText));
        }
        return nullptr;
    }

    const T_CONTRAL_UNIT* unitInfo = CurrentContralUnit();
    if (unitInfo == nullptr)
    {
        return nullptr;
    }
    return static_cast<RobotDriverAdaptor*>(unitInfo->pUnitDriver);
}

FANUCRobotCtrl* QtWidgetsApplication4::GetCurrentFanucDriver(QWidget* parent)
{
    RobotDriverAdaptor* pRobotDriverAdaptor = GetCurrentRobotDriver(parent);
    if (pRobotDriverAdaptor == nullptr)
    {
        return nullptr;
    }

    FANUCRobotCtrl* fanucDriver = dynamic_cast<FANUCRobotCtrl*>(pRobotDriverAdaptor);
    if (fanucDriver == nullptr && parent != nullptr)
    {
        const QString robotName = DecodeConfigText(pRobotDriverAdaptor->m_sRobotName);
        const QString displayName = DecodeConfigText(pRobotDriverAdaptor->m_sCustomName);
        const QString robotLabel = displayName.isEmpty()
            ? robotName
            : QString("%1 (%2)").arg(displayName, robotName);
        QMessageBox::warning(parent, "驱动类型不匹配",
            QString("当前功能需要 FANUC 驱动。\n当前选择：%1，驱动已创建但不是 FANUC。").arg(robotLabel));
    }
    return fanucDriver;
}

void QtWidgetsApplication4::RefreshRobotSelectorUi()
{
    if (m_pRobotSelectorCombo == nullptr)
    {
        return;
    }

    QSignalBlocker blocker(m_pRobotSelectorCombo);
    const int previousIndex = CurrentRobotUnitIndex();
    m_pRobotSelectorCombo->clear();
    if (m_pContralUnit == nullptr || m_pContralUnit->m_vtContralUnitInfo.empty())
    {
        m_pRobotSelectorCombo->addItem("无可用机器人", -1);
        m_pRobotSelectorCombo->setEnabled(false);
        m_nCurrentRobotUnitIndex = -1;
        if (m_pRobotSelectorLabel != nullptr)
        {
            m_pRobotSelectorLabel->setVisible(false);
        }
        return;
    }

    int firstReadyComboIndex = -1;
    int readyCount = 0;
    for (const T_CONTRAL_UNIT& unitInfo : m_pContralUnit->m_vtContralUnitInfo)
    {
        RobotDriverAdaptor* driver = static_cast<RobotDriverAdaptor*>(unitInfo.pUnitDriver);
        const QString robotName = DecodeConfigText(
            driver != nullptr && !driver->m_sRobotName.empty() ? driver->m_sRobotName : unitInfo.sUnitName);
        QString displayName = DecodeConfigText(unitInfo.sChineseName);
        if (displayName.isEmpty())
        {
            displayName = DecodeConfigText(
                driver != nullptr && !driver->m_sCustomName.empty() ? driver->m_sCustomName : unitInfo.sUnitName);
        }
        const QString typeText = RobotDriverTypeText(driver);
        QString label = robotName.isEmpty()
            ? QString("%1 / %2").arg(displayName, typeText)
            : QString("%1 / %2 (%3)").arg(displayName, typeText, robotName);
        const bool driverReady = IsRobotUnitDriverReady(unitInfo.nUnitNo);
        if (!driverReady)
        {
            label += "（类型错误）";
        }
        m_pRobotSelectorCombo->addItem(label, unitInfo.nUnitNo);
        const int itemIndex = m_pRobotSelectorCombo->count() - 1;
        if (driverReady)
        {
            if (firstReadyComboIndex < 0)
            {
                firstReadyComboIndex = itemIndex;
            }
            ++readyCount;
        }
        else if (QStandardItemModel* model = qobject_cast<QStandardItemModel*>(m_pRobotSelectorCombo->model()))
        {
            if (QStandardItem* item = model->item(itemIndex))
            {
                item->setEnabled(false);
                item->setSelectable(false);
                item->setToolTip("机器人类型错误或驱动未创建，主页现场操作不可用。");
            }
        }
    }

    if (m_pRobotSelectorLabel != nullptr)
    {
        m_pRobotSelectorLabel->setVisible(m_pContralUnit->m_vtContralUnitInfo.size() > 0);
    }
    int comboIndex = -1;
    if (previousIndex >= 0 && IsRobotUnitDriverReady(previousIndex))
    {
        comboIndex = m_pRobotSelectorCombo->findData(previousIndex);
    }
    if (comboIndex < 0)
    {
        comboIndex = firstReadyComboIndex;
    }
    if (comboIndex < 0)
    {
        m_pRobotSelectorCombo->clear();
        m_pRobotSelectorCombo->addItem("无可用机器人驱动", -1);
        m_pRobotSelectorCombo->setEnabled(false);
        m_nCurrentRobotUnitIndex = -1;
        return;
    }
    m_pRobotSelectorCombo->setCurrentIndex(comboIndex);
    m_nCurrentRobotUnitIndex = m_pRobotSelectorCombo->currentData().toInt();
    m_pRobotSelectorCombo->setEnabled(readyCount > 1 || m_pRobotSelectorCombo->count() > readyCount);
}

QString QtWidgetsApplication4::AccountConfigPath() const
{
	return RobotDataHelper::BuildProjectPath("Data/Accounts.ini");
}

QString QtWidgetsApplication4::RoleDisplayName(const QString& role) const
{
	if (role == kRoleAdmin)
	{
		return "管理员";
	}
	if (role == kRoleEngineer)
	{
		return "工程师";
	}
	return "操作员";
}

int QtWidgetsApplication4::RoleLevel(const QString& role) const
{
	if (role == kRoleAdmin)
	{
		return 3;
	}
	if (role == kRoleEngineer)
	{
		return 2;
	}
	return 1;
}

bool QtWidgetsApplication4::RequirePermission(const QString& minimumRole, const QString& actionName)
{
	if (RoleLevel(m_sCurrentUserRole) >= RoleLevel(minimumRole))
	{
		return true;
	}

	const QMessageBox::StandardButton button = QMessageBox::question(
		this,
		"权限不足",
		QString("%1 需要 %2 或更高权限。\n当前用户：%3（%4）\n是否重新登录？")
			.arg(actionName, RoleDisplayName(minimumRole), m_sCurrentUserName, RoleDisplayName(m_sCurrentUserRole)),
		QMessageBox::Yes | QMessageBox::No,
		QMessageBox::Yes);
	if (button == QMessageBox::Yes)
	{
		ShowAuthPage(QString("%1 需要 %2 或更高权限，请重新登录。")
			.arg(actionName, RoleDisplayName(minimumRole)));
	}
	return false;
}

QString QtWidgetsApplication4::LoginStateConfigPath() const
{
	return RobotDataHelper::BuildProjectPath("Data/LoginState.ini");
}

void QtWidgetsApplication4::EnsureDefaultAdminAccount()
{
	const QString accountPath = AccountConfigPath();
	QSettings settings(accountPath, QSettings::IniFormat);
	settings.beginGroup("Users");
	const bool hasAccount = !settings.childGroups().isEmpty();
	settings.endGroup();
	if (hasAccount)
	{
		return;
	}

	QString error;
	SaveAccount("admin", "admin", kRoleAdmin, error);
}

void QtWidgetsApplication4::RefreshAccountUi()
{
	const QString userText = QString("当前用户：%1（%2）").arg(m_sCurrentUserName, RoleDisplayName(m_sCurrentUserRole));
	if (m_pCurrentUserButton != nullptr)
	{
		m_pCurrentUserButton->setText(userText);
	}
	if (m_pManagementUserLabel != nullptr)
	{
		m_pManagementUserLabel->setText(userText);
	}
	if (m_pPermissionHintLabel != nullptr)
	{
		m_pPermissionHintLabel->setText(
			"权限说明：登录后可进入主页；工程师或管理员可打开管理页面；管理员还可进入账号管理。首次启动默认管理员为 admin / admin。");
	}

	if (m_pAccountManagementAction != nullptr)
	{
		m_pAccountManagementAction->setEnabled(RoleLevel(m_sCurrentUserRole) >= RoleLevel(kRoleAdmin));
	}
	RefreshDebugLogButtonUi();
}

void QtWidgetsApplication4::RefreshDebugLogButtonUi()
{
	const bool isAdmin = RoleLevel(m_sCurrentUserRole) >= RoleLevel(kRoleAdmin);
	if (!isAdmin && m_bDebugLogMode)
	{
		m_bDebugLogMode = false;
	}
	if (m_pDashboardDebugLogBtn != nullptr)
	{
		QSignalBlocker blocker(m_pDashboardDebugLogBtn);
		m_pDashboardDebugLogBtn->setVisible(isAdmin);
		m_pDashboardDebugLogBtn->setEnabled(isAdmin);
		m_pDashboardDebugLogBtn->setChecked(isAdmin && m_bDebugLogMode);
		m_pDashboardDebugLogBtn->setText(m_bDebugLogMode ? "隐藏调试日志" : "显示调试日志");
	}

	if (m_pMainStack != nullptr)
	{
		for (int i = 0; i < m_pMainStack->count(); ++i)
		{
			ApplyDebugLogVisibility(m_pMainStack->widget(i));
		}
	}
	if (m_pManagementStack != nullptr)
	{
		for (int i = 0; i < m_pManagementStack->count(); ++i)
		{
			ApplyDebugLogVisibility(m_pManagementStack->widget(i));
		}
	}
	if (m_pMeasureThenWeldPage != nullptr)
	{
		ApplyDebugLogVisibility(m_pMeasureThenWeldPage);
	}
	for (const QPointer<MeasureThenWeldDialog>& page : m_measureThenWeldPages)
	{
		ApplyDebugLogVisibility(page);
	}
	for (const QPointer<RobotJogDialog>& page : m_robotJogPages)
	{
		ApplyDebugLogVisibility(page);
	}
}

void QtWidgetsApplication4::SetDebugLogMode(bool enabled)
{
	if (RoleLevel(m_sCurrentUserRole) < RoleLevel(kRoleAdmin))
	{
		enabled = false;
	}
	m_bDebugLogMode = enabled;
	RefreshDebugLogButtonUi();
}

void QtWidgetsApplication4::ApplyDebugLogVisibility(QWidget* page)
{
	if (page == nullptr)
	{
		return;
	}
	const bool showLogs = m_bDebugLogMode && RoleLevel(m_sCurrentUserRole) >= RoleLevel(kRoleAdmin);
	const QList<QPlainTextEdit*> logWidgets = page->findChildren<QPlainTextEdit*>();
	for (QPlainTextEdit* logWidget : logWidgets)
	{
		if (IsLikelyDebugLogWidget(logWidget))
		{
			logWidget->setVisible(showLogs);
		}
	}
}

QString QtWidgetsApplication4::CameraReceiveModeConfigPath() const
{
	return RobotDataHelper::BuildProjectPath("Data/CameraReceiveMode.ini");
}

void QtWidgetsApplication4::LoadCameraReceiveMode()
{
	QSettings settings(CameraReceiveModeConfigPath(), QSettings::IniFormat);
	const bool savedSharedUdpMode = settings.value("Camera/UseSharedReceiver", false).toBool();
	m_bUseSharedScanCameraReceiver = false;
	if (savedSharedUdpMode)
	{
		settings.setValue("Camera/UseSharedReceiver", false);
		settings.sync();
	}
	RefreshCameraReceiveModeButtonUi();
}

void QtWidgetsApplication4::SaveCameraReceiveMode() const
{
	QSettings settings(CameraReceiveModeConfigPath(), QSettings::IniFormat);
	settings.setValue("Camera/UseSharedReceiver", m_bUseSharedScanCameraReceiver);
	settings.sync();
}

void QtWidgetsApplication4::RefreshCameraReceiveModeButtonUi()
{
	if (m_pManagementCameraReceiveModeBtn == nullptr)
	{
		return;
	}

	QSignalBlocker blocker(m_pManagementCameraReceiveModeBtn);
	m_pManagementCameraReceiveModeBtn->setChecked(false);
	m_pManagementCameraReceiveModeBtn->setEnabled(false);
	m_pManagementCameraReceiveModeBtn->setText("相机接收：TCP独立");
	m_pManagementCameraReceiveModeBtn->setToolTip("新版TCP相机客户端按相机IP独立连接，旧UDP共享端口模式不再使用。");
}

void QtWidgetsApplication4::RefreshTouchKeyboardModeUi()
{
	if (m_pTouchKeyboardModeCombo == nullptr || m_pTouchKeyboardManager == nullptr)
	{
		return;
	}

	const QString storageValue = TouchKeyboardManager::ModeToStorageString(m_pTouchKeyboardManager->CurrentMode());
	const int index = m_pTouchKeyboardModeCombo->findData(storageValue);
	QSignalBlocker blocker(m_pTouchKeyboardModeCombo);
	m_pTouchKeyboardModeCombo->setCurrentIndex(index >= 0 ? index : 0);
}

void QtWidgetsApplication4::SetSharedScanCameraReceiverMode(bool enabled)
{
	// TCP客户端无法复用旧UDP共享端口/IP分发模式，界面开关统一落到独立连接。
	enabled = false;
	if (m_bUseSharedScanCameraReceiver == enabled)
	{
		RefreshCameraReceiveModeButtonUi();
		return;
	}

	const bool wasPreviewing = ui.GrooveCameraTestBtn != nullptr && ui.GrooveCameraTestBtn->isChecked();
	if (m_grooveCameraDisplayTimer != nullptr)
	{
		m_grooveCameraDisplayTimer->stop();
	}
	m_bUseSharedScanCameraReceiver = enabled;
	SaveCameraReceiveMode();
	RefreshCameraReceiveModeButtonUi();

	StopScanCameraRuntimes();
	InitializeScanCameraRuntimes();

	if (wasPreviewing && ui.GrooveCameraTestBtn != nullptr)
	{
		QString cameraIP;
		const int unitIndex = CurrentRobotUnitIndex();
		EnsureScanCameraRunningForUnit(unitIndex, cameraIP, false);
		m_grooveCameraDisplayTimer->start(33);
	}
	if (ui.GrooveCameraText != nullptr)
	{
		ui.GrooveCameraText->appendPlainText("相机接收模式保持为：TCP独立连接。");
	}
}

void QtWidgetsApplication4::SetAuthRegisterMode(bool registerMode)
{
	m_bAuthRegisterMode = registerMode;
	RefreshAuthModeUi();
}

void QtWidgetsApplication4::RefreshAuthModeUi()
{
	if (m_pAuthLoginModeBtn != nullptr)
	{
		QSignalBlocker blockerLogin(m_pAuthLoginModeBtn);
		m_pAuthLoginModeBtn->setChecked(!m_bAuthRegisterMode);
	}
	if (m_pAuthRegisterModeBtn != nullptr)
	{
		QSignalBlocker blockerRegister(m_pAuthRegisterModeBtn);
		m_pAuthRegisterModeBtn->setChecked(m_bAuthRegisterMode);
	}
	if (m_pAuthSubmitBtn != nullptr)
	{
		m_pAuthSubmitBtn->setText(m_bAuthRegisterMode ? "注册账号" : "登录");
	}
	if (m_pAuthHintLabel != nullptr)
	{
		if (!m_sAuthHintOverride.trimmed().isEmpty())
		{
			m_pAuthHintLabel->setText(m_sAuthHintOverride);
			m_pAuthHintLabel->show();
		}
		else
		{
			m_pAuthHintLabel->setText(m_bAuthRegisterMode
				? "注册账号只能创建操作员权限，账号至少 4 位，密码至少 8 位，并且需要确认密码一致。"
				: "软件启动后先进行账号登录；如没有账号，可切到注册模式创建操作员账号。");
			m_pAuthHintLabel->hide();
		}
	}
	if (m_pAuthConfirmPasswordRow != nullptr)
	{
		m_pAuthConfirmPasswordRow->setVisible(m_bAuthRegisterMode);
	}
	if (m_pRememberPasswordCheck != nullptr)
	{
		m_pRememberPasswordCheck->setVisible(!m_bAuthRegisterMode);
	}
	if (m_pAutoLoginCheck != nullptr)
	{
		m_pAutoLoginCheck->setVisible(!m_bAuthRegisterMode);
		if (!m_bAuthRegisterMode)
		{
			m_pAutoLoginCheck->setEnabled(m_pRememberPasswordCheck == nullptr || m_pRememberPasswordCheck->isChecked());
		}
	}
	if (m_pGuestLoginBtn != nullptr)
	{
		m_pGuestLoginBtn->setVisible(!m_bAuthRegisterMode);
	}
}

void QtWidgetsApplication4::LoadLoginState()
{
	if (m_pLoginNameEdit == nullptr || m_pLoginPasswordEdit == nullptr || m_pRememberPasswordCheck == nullptr || m_pAutoLoginCheck == nullptr)
	{
		return;
	}

	QSettings settings(LoginStateConfigPath(), QSettings::IniFormat);
	const QString userName = settings.value("UserName").toString();
	const bool rememberPassword = settings.value("RememberPassword", false).toBool();
	const bool autoLogin = settings.value("AutoLogin", false).toBool();
	const QByteArray passwordBytes = QByteArray::fromBase64(settings.value("PasswordBase64").toByteArray());
	const QString password = QString::fromUtf8(passwordBytes);

	RefreshLoginNameHistory();
	if (!userName.trimmed().isEmpty())
	{
		if (m_pLoginNameCombo != nullptr)
		{
			m_pLoginNameCombo->setCurrentText(userName);
		}
		else
		{
			m_pLoginNameEdit->setText(userName);
		}
	}
	if (rememberPassword)
	{
		const QString savedPassword = settings.value(QString("SavedPasswords/%1").arg(userName)).toByteArray().isEmpty()
			? password
			: QString::fromUtf8(QByteArray::fromBase64(settings.value(QString("SavedPasswords/%1").arg(userName)).toByteArray()));
		m_pLoginPasswordEdit->setText(savedPassword);
	}
	m_pRememberPasswordCheck->setChecked(rememberPassword);
	m_pAutoLoginCheck->setChecked(rememberPassword && autoLogin);
	m_pAutoLoginCheck->setEnabled(rememberPassword);
}

void QtWidgetsApplication4::SaveLoginState() const
{
	if (m_pLoginNameEdit == nullptr || m_pLoginPasswordEdit == nullptr || m_pRememberPasswordCheck == nullptr || m_pAutoLoginCheck == nullptr)
	{
		return;
	}

	QSettings settings(LoginStateConfigPath(), QSettings::IniFormat);
	const QString userName = m_pLoginNameEdit->text().trimmed();
	QStringList history = settings.value("AccountHistory").toStringList();
	history.removeAll(userName);
	if (!userName.isEmpty())
	{
		history.prepend(userName);
	}
	while (history.size() > 10)
	{
		history.removeLast();
	}
	settings.setValue("AccountHistory", history);
	settings.setValue("UserName", userName);
	settings.setValue("RememberPassword", m_pRememberPasswordCheck->isChecked());
	settings.setValue("AutoLogin", m_pRememberPasswordCheck->isChecked() && m_pAutoLoginCheck->isChecked());
	if (m_pRememberPasswordCheck->isChecked())
	{
		settings.setValue("PasswordBase64", m_pLoginPasswordEdit->text().toUtf8().toBase64());
		settings.setValue(QString("SavedPasswords/%1").arg(userName), m_pLoginPasswordEdit->text().toUtf8().toBase64());
	}
	else
	{
		settings.remove("PasswordBase64");
		settings.remove("AutoLogin");
		settings.remove(QString("SavedPasswords/%1").arg(userName));
	}
	settings.sync();
}

void QtWidgetsApplication4::RefreshLoginNameHistory()
{
	if (m_pLoginNameCombo == nullptr)
	{
		return;
	}

	const QString currentName = m_pLoginNameCombo->currentText();
	QSettings settings(LoginStateConfigPath(), QSettings::IniFormat);
	QStringList history = settings.value("AccountHistory").toStringList();
	const QString lastUser = settings.value("UserName").toString().trimmed();
	if (!lastUser.isEmpty() && !history.contains(lastUser))
	{
		history.prepend(lastUser);
	}
	history.removeDuplicates();

	QSignalBlocker blocker(m_pLoginNameCombo);
	m_pLoginNameCombo->clear();
	m_pLoginNameCombo->addItems(history);
	m_pLoginNameCombo->setCurrentText(currentName.isEmpty() ? lastUser : currentName);
}

void QtWidgetsApplication4::FillSavedPasswordForUser(const QString& userName)
{
	if (m_pLoginPasswordEdit == nullptr || m_pRememberPasswordCheck == nullptr || m_pAutoLoginCheck == nullptr)
	{
		return;
	}

	const QString normalizedUser = userName.trimmed();
	if (normalizedUser.isEmpty())
	{
		return;
	}

	QSettings settings(LoginStateConfigPath(), QSettings::IniFormat);
	const QByteArray savedBytes = settings.value(QString("SavedPasswords/%1").arg(normalizedUser)).toByteArray();
	if (!savedBytes.isEmpty())
	{
		m_pLoginPasswordEdit->setText(QString::fromUtf8(QByteArray::fromBase64(savedBytes)));
		m_pRememberPasswordCheck->setChecked(true);
		m_pAutoLoginCheck->setEnabled(true);
		return;
	}

	if (settings.value("UserName").toString().trimmed() == normalizedUser
		&& settings.value("RememberPassword", false).toBool())
	{
		const QByteArray legacyBytes = settings.value("PasswordBase64").toByteArray();
		if (!legacyBytes.isEmpty())
		{
			m_pLoginPasswordEdit->setText(QString::fromUtf8(QByteArray::fromBase64(legacyBytes)));
			m_pRememberPasswordCheck->setChecked(true);
			m_pAutoLoginCheck->setEnabled(true);
			return;
		}
	}

	m_pLoginPasswordEdit->clear();
	m_pRememberPasswordCheck->setChecked(false);
	m_pAutoLoginCheck->setChecked(false);
	m_pAutoLoginCheck->setEnabled(false);
}

bool QtWidgetsApplication4::TryAutoLogin()
{
	if (m_pAutoLoginCheck == nullptr || !m_pAutoLoginCheck->isChecked())
	{
		return false;
	}
	if (m_pLoginNameEdit == nullptr || m_pLoginPasswordEdit == nullptr)
	{
		return false;
	}
	if (m_pLoginNameEdit->text().trimmed().isEmpty() || m_pLoginPasswordEdit->text().isEmpty())
	{
		return false;
	}

	LoginCurrentAccount();
	return RoleLevel(m_sCurrentUserRole) >= RoleLevel(kRoleOperator) && m_pMainStack != nullptr && m_pMainStack->currentWidget() == m_pDashboardPage;
}

void QtWidgetsApplication4::ShowAuthPage(const QString& promptMessage)
{
	if (m_pMainStack == nullptr || m_pAuthPage == nullptr)
	{
		return;
	}

	setMaximumSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX);
	setMinimumSize(920, 700);
	if (isMaximized() || isFullScreen())
	{
		showNormal();
	}
	resize(minimumSize());
	CenterWindowOnScreen(this);

	m_sAuthHintOverride = promptMessage.trimmed();
	SetAuthRegisterMode(false);
	m_pMainStack->setCurrentWidget(m_pAuthPage);
	if (m_pLoginNameEdit != nullptr)
	{
		m_pLoginNameEdit->setFocus(Qt::OtherFocusReason);
	}
}

bool QtWidgetsApplication4::VerifyAccount(const QString& userName, const QString& password, QString& role, QString& error) const
{
	const QString normalizedUser = userName.trimmed();
	if (normalizedUser.isEmpty() || password.isEmpty())
	{
		error = "账号和密码不能为空。";
		return false;
	}

	QSettings settings(AccountConfigPath(), QSettings::IniFormat);
	settings.beginGroup("Users");
	if (!settings.childGroups().contains(normalizedUser))
	{
		settings.endGroup();
		error = "账号不存在。";
		return false;
	}

	settings.beginGroup(normalizedUser);
	const QString expectedHash = settings.value("PasswordHash").toString();
	role = settings.value("Role", kRoleOperator).toString();
	settings.endGroup();
	settings.endGroup();

	const QString actualHash = QString::fromLatin1(
		QCryptographicHash::hash(QString("%1\n%2").arg(normalizedUser, password).toUtf8(), QCryptographicHash::Sha256).toHex());
	if (expectedHash.compare(actualHash, Qt::CaseInsensitive) != 0)
	{
		error = "密码不正确。";
		return false;
	}
	return true;
}

bool QtWidgetsApplication4::SaveAccount(const QString& userName, const QString& password, const QString& role, QString& error) const
{
	const QString normalizedUser = userName.trimmed();
	if (normalizedUser.size() < 3 || normalizedUser.size() > 32)
	{
		error = "账号长度需要在 3 到 32 个字符之间。";
		return false;
	}
	for (const QChar& ch : normalizedUser)
	{
		if (!(ch.isLetterOrNumber() || ch == '_' || ch == '-' || ch == '.'))
		{
			error = "账号只能包含字母、数字、中文、下划线、中划线或点。";
			return false;
		}
	}
	if (password.size() < 3)
	{
		error = "密码至少需要 3 个字符。";
		return false;
	}
	if (role != kRoleOperator && role != kRoleEngineer && role != kRoleAdmin)
	{
		error = "权限类型无效。";
		return false;
	}

	const QString accountPath = AccountConfigPath();
	const QFileInfo accountInfo(accountPath);
	QDir dir;
	if (!dir.mkpath(accountInfo.absolutePath()))
	{
		error = "创建账号目录失败：" + QDir::toNativeSeparators(accountInfo.absolutePath());
		return false;
	}

	QSettings settings(accountPath, QSettings::IniFormat);
	settings.beginGroup("Users");
	if (settings.childGroups().contains(normalizedUser))
	{
		settings.endGroup();
		error = "账号已存在。";
		return false;
	}

	settings.beginGroup(normalizedUser);
	const QString hash = QString::fromLatin1(
		QCryptographicHash::hash(QString("%1\n%2").arg(normalizedUser, password).toUtf8(), QCryptographicHash::Sha256).toHex());
	settings.setValue("PasswordHash", hash);
	settings.setValue("Role", role);
	settings.setValue("CreatedAt", QDateTime::currentDateTime().toString(Qt::ISODate));
	settings.endGroup();
	settings.endGroup();
	settings.sync();

	if (settings.status() != QSettings::NoError)
	{
		error = "写入账号文件失败：" + accountPath;
		return false;
	}
	return true;
}

void QtWidgetsApplication4::LoginCurrentAccount()
{
	if (m_pLoginNameEdit == nullptr || m_pLoginPasswordEdit == nullptr)
	{
		return;
	}

	QString role;
	QString error;
	const QString userName = m_pLoginNameEdit->text().trimmed();
	if (!VerifyAccount(userName, m_pLoginPasswordEdit->text(), role, error))
	{
		QMessageBox::warning(this, "账号登录", error);
		return;
	}

	m_sCurrentUserName = userName;
	m_sCurrentUserRole = role;
	SaveLoginState();
	RefreshLoginNameHistory();
	if (m_pRememberPasswordCheck == nullptr || !m_pRememberPasswordCheck->isChecked())
	{
		m_pLoginPasswordEdit->clear();
	}
	if (m_pAccountLogText != nullptr)
	{
		m_pAccountLogText->appendPlainText(QString("[%1] %2 登录成功，权限：%3")
			.arg(QDateTime::currentDateTime().toString("HH:mm:ss"), m_sCurrentUserName, RoleDisplayName(m_sCurrentUserRole)));
	}
	RefreshAccountUi();
	SetAuthRegisterMode(false);
	if (m_bPendingOpenManagementAfterLogin && RoleLevel(m_sCurrentUserRole) >= RoleLevel(kRoleEngineer))
	{
		m_bPendingOpenManagementAfterLogin = false;
		ShowManagementPage();
		return;
	}
	m_bPendingOpenManagementAfterLogin = false;
	ShowDashboardPage();
}

void QtWidgetsApplication4::LoginAsGuest()
{
	m_sCurrentUserName = "游客";
	m_sCurrentUserRole = kRoleOperator;
	m_bPendingOpenManagementAfterLogin = false;
	if (m_pAccountLogText != nullptr)
	{
		m_pAccountLogText->appendPlainText(QString("[%1] 游客登录，权限：%2")
			.arg(QDateTime::currentDateTime().toString("HH:mm:ss"), RoleDisplayName(m_sCurrentUserRole)));
	}
	RefreshAccountUi();
	SetAuthRegisterMode(false);
	ShowDashboardPage();
}

void QtWidgetsApplication4::LogoutCurrentAccount()
{
	m_sCurrentUserName = "访客";
	m_sCurrentUserRole = kRoleOperator;
	m_bPendingOpenManagementAfterLogin = false;
	if (m_pAccountLogText != nullptr)
	{
		m_pAccountLogText->appendPlainText(QString("[%1] 已退出登录，当前为访客操作员权限。")
			.arg(QDateTime::currentDateTime().toString("HH:mm:ss")));
	}
	RefreshAccountUi();
	ShowAuthPage();
}

void QtWidgetsApplication4::RegisterAccount()
{
	if (m_pLoginNameEdit == nullptr || m_pLoginPasswordEdit == nullptr || m_pAuthConfirmPasswordEdit == nullptr)
	{
		return;
	}

	const QString role = kRoleOperator;
	const QString normalizedUser = m_pLoginNameEdit->text().trimmed();
	if (normalizedUser.size() < 4)
	{
		QMessageBox::warning(this, "注册账号", "账号至少需要 4 个字符。");
		return;
	}
	const QString password = m_pLoginPasswordEdit->text();
	if (password.size() < 8)
	{
		QMessageBox::warning(this, "注册账号", "密码至少需要 8 个字符。");
		return;
	}
	if (m_pAuthConfirmPasswordEdit->text() != password)
	{
		QMessageBox::warning(this, "注册账号", "两次输入的密码不一致。");
		return;
	}
	QString error;
	if (!SaveAccount(m_pLoginNameEdit->text(), password, role, error))
	{
		QMessageBox::warning(this, "注册账号", error);
		return;
	}

	if (m_pAccountLogText != nullptr)
	{
		m_pAccountLogText->appendPlainText(QString("[%1] 已注册账号 %2，权限：%3")
			.arg(QDateTime::currentDateTime().toString("HH:mm:ss"),
				normalizedUser,
				RoleDisplayName(role)));
	}
	m_pLoginPasswordEdit->clear();
	m_pAuthConfirmPasswordEdit->clear();
	SetAuthRegisterMode(false);
	if (m_pLoginNameEdit != nullptr)
	{
		m_pLoginNameEdit->setText(normalizedUser);
		m_pLoginNameEdit->setFocus(Qt::OtherFocusReason);
	}
	if (m_pRememberPasswordCheck != nullptr)
	{
		m_pRememberPasswordCheck->setChecked(false);
	}
	if (m_pAutoLoginCheck != nullptr)
	{
		m_pAutoLoginCheck->setChecked(false);
		m_pAutoLoginCheck->setEnabled(false);
	}
	QMessageBox::information(this, "注册账号", "注册成功。请使用新账号登录。");
	RefreshAccountUi();
}

void QtWidgetsApplication4::OpenAccountManagementDialog()
{
	if (RoleLevel(m_sCurrentUserRole) < RoleLevel(kRoleAdmin))
	{
		QMessageBox::information(this, "账号管理", "账号管理仅管理员可用。");
		return;
	}

	if (m_pManagementStack == nullptr)
	{
		AccountManagementDialog dialog(this);
		dialog.exec();
		return;
	}

	if (m_pAccountManagementPage != nullptr)
	{
		m_pManagementStack->removeWidget(m_pAccountManagementPage);
		m_pAccountManagementPage->deleteLater();
		m_pAccountManagementPage = nullptr;
	}

	m_pAccountManagementPage = new AccountManagementDialog(m_pManagementStack);
	PrepareEmbeddedPage(m_pAccountManagementPage, m_pManagementStack);
	ShowManagementEmbeddedPage(m_pAccountManagementPage);
}

void QtWidgetsApplication4::OpenControlUnitManagementDialog()
{
	if (RoleLevel(m_sCurrentUserRole) < RoleLevel(kRoleEngineer))
	{
		QMessageBox::information(this, "控制单元管理", "控制单元管理需要工程师或管理员权限。");
		return;
	}

	auto reloadControlUnits = [this]()
		{
			StopScanCameraRuntimes();
			if (m_pContralUnit != nullptr)
			{
				m_pContralUnit->InitContralUnit();
			}
			InitializeScanCameraRuntimes();
			RefreshRobotSelectorUi();
			RefreshRobotOperationAvailability();
			RefreshDashboardConnectionState();
		};

	auto unitIndexForRobotName = [this](const QString& robotName) -> int
		{
			if (m_pContralUnit == nullptr)
			{
				return -1;
			}
			for (int index = 0; index < static_cast<int>(m_pContralUnit->m_vtContralUnitInfo.size()); ++index)
			{
				const T_CONTRAL_UNIT& unitInfo = m_pContralUnit->m_vtContralUnitInfo.at(index);
				if (DecodeConfigText(unitInfo.sUnitName).trimmed().compare(robotName, Qt::CaseInsensitive) == 0)
				{
					return index;
				}
			}
			return -1;
		};

	auto openCameraBasicParam = [this](const QString& robotName, const QString& cameraSection) -> bool
		{
			CameraBasicParamDialog dialog(
				robotName,
				cameraSection,
				[this]() { RefreshRobotOperationAvailability(); },
				this);
			dialog.exec();
			return dialog.SavedThisSession();
		};

	auto openHandEyeCalibration = [this, unitIndexForRobotName](const QString& robotName, const QString& cameraSection) -> bool
		{
			const int unitIndex = unitIndexForRobotName(robotName);
			auto startCamera = [this, unitIndex](QString& cameraIP) -> bool
				{
					if (unitIndex < 0)
					{
						return false;
					}
					return EnsureScanCameraRunningForUnit(unitIndex, cameraIP, true);
				};
			auto stopCamera = []()
				{
				};

			HandEyeCalibrationDialog dialog(
				m_pContralUnit,
				robotName,
				cameraSection,
				startCamera,
				stopCamera,
				unitIndex >= 0 ? ScanCameraCacheForUnit(unitIndex) : nullptr,
				this);
			dialog.exec();
			if (!dialog.MatrixComputedThisSession())
			{
				return false;
			}

			QString setupError;
			if (!WriteRobotSetupReadyFlagGlobal(robotName, "HandEyeReady", &setupError))
			{
				QMessageBox::warning(this, "手眼标定", setupError);
				return false;
			}
			RefreshRobotOperationAvailability();
			return true;
		};

	if (m_pManagementStack == nullptr)
	{
		ControlUnitManagementDialog dialog(reloadControlUnits, openCameraBasicParam, openHandEyeCalibration, this);
		dialog.exec();
		return;
	}

	if (m_pControlUnitManagementPage != nullptr)
	{
		m_pManagementStack->removeWidget(m_pControlUnitManagementPage);
		m_pControlUnitManagementPage->deleteLater();
		m_pControlUnitManagementPage = nullptr;
	}

	m_pControlUnitManagementPage = new ControlUnitManagementDialog(reloadControlUnits, openCameraBasicParam, openHandEyeCalibration, m_pManagementStack);
	PrepareEmbeddedPage(m_pControlUnitManagementPage, m_pManagementStack);
	ShowManagementEmbeddedPage(m_pControlUnitManagementPage);
}

void QtWidgetsApplication4::PrepareEmbeddedPage(QWidget* page)
{
	PrepareEmbeddedPage(page, m_pMainStack);
}

void QtWidgetsApplication4::PrepareEmbeddedPage(QWidget* page, QStackedWidget* targetStack)
{
	if (page == nullptr || targetStack == nullptr)
	{
		return;
	}

	page->setAttribute(Qt::WA_DeleteOnClose, false);
	if (m_pMainStack != nullptr && m_pMainStack != targetStack && m_pMainStack->indexOf(page) >= 0)
	{
		m_pMainStack->removeWidget(page);
	}
	if (m_pManagementStack != nullptr && m_pManagementStack != targetStack && m_pManagementStack->indexOf(page) >= 0)
	{
		m_pManagementStack->removeWidget(page);
	}
	page->setParent(targetStack);
	page->setWindowFlags(Qt::Widget);
	page->setWindowModality(Qt::NonModal);
	page->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	page->setMinimumSize(0, 0);
	page->setMaximumSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX);
	page->setGeometry(targetStack->contentsRect());

	const bool targetIsManagement = targetStack == m_pManagementStack;
	page->setProperty("_management_embedded_page", targetIsManagement);
	QPushButton* backButton = EmbeddedBackButton(page);
	if (targetIsManagement)
	{
		if (backButton != nullptr)
		{
			backButton->hide();
		}
	}
	else if (backButton == nullptr)
	{
		backButton = new QPushButton(page);
		backButton->setObjectName("EmbeddedBackToDashboardButton");
		backButton->setCursor(Qt::PointingHandCursor);
		backButton->setFocusPolicy(Qt::NoFocus);
		backButton->setStyleSheet(
			"QPushButton#EmbeddedBackToDashboardButton {"
			"background: #1F3542; color: #F4FAFA; border: 1px solid #4B7184;"
			"border-radius: 10px; padding: 7px 14px; font-size: 14px; font-weight: 600;"
			"}"
			"QPushButton#EmbeddedBackToDashboardButton:hover { background: #2D5465; border-color: #72D4DD; }"
			"QPushButton#EmbeddedBackToDashboardButton:pressed { background: #18303B; }");
		connect(backButton, &QPushButton::clicked, this, [this, page]()
			{
				if (page == nullptr)
				{
					ShowDashboardPage();
					return;
				}
				if (page->close())
				{
					const bool inManagementStack = m_pManagementStack != nullptr
						&& m_pManagementStack->indexOf(page) >= 0;
					if (inManagementStack)
					{
						ShowManagementHomePage();
					}
					else
					{
						ShowDashboardPage();
					}
				}
			});
	}
	if (backButton != nullptr && !targetIsManagement)
	{
		backButton->setText("返回主页");
		backButton->setToolTip("返回大按钮主页");
		backButton->show();
		PositionEmbeddedBackButton(page);
	}

	const QList<QScrollArea*> scrollAreas = page->findChildren<QScrollArea*>();
	for (QScrollArea* scrollArea : scrollAreas)
	{
		if (scrollArea == nullptr)
		{
			continue;
		}
		scrollArea->setWidgetResizable(true);
		scrollArea->setAlignment(Qt::AlignLeft | Qt::AlignTop);
		scrollArea->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
		if (QWidget* scrollWidget = scrollArea->widget())
		{
			scrollWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
		}
	}

	page->installEventFilter(this);
	if (targetStack->indexOf(page) < 0)
	{
		targetStack->addWidget(page);
	}
	ApplyCompactControlWidths(page);
	ApplyAdaptiveScrollSupport(page);
	ApplyDebugLogVisibility(page);
	QTimer::singleShot(80, page, [page]() { ApplyCompactControlWidths(page); });
	QPointer<QWidget> pagePtr(page);
	QTimer::singleShot(80, this, [this, pagePtr]() { ApplyDebugLogVisibility(pagePtr); });
}

void QtWidgetsApplication4::ShowEmbeddedPage(QWidget* page)
{
	if (page == nullptr || m_pMainStack == nullptr)
	{
		return;
	}

	auto refreshPageGeometry = [this, page]()
		{
			if (m_pMainStack == nullptr || m_pMainStack->currentWidget() != page)
			{
				return;
			}
			page->setGeometry(m_pMainStack->contentsRect());
			page->updateGeometry();
			PositionEmbeddedBackButton(page);
			if (QLayout* pageLayout = page->layout())
			{
				pageLayout->invalidate();
				pageLayout->activate();
			}
			page->update();
		};

	PrepareEmbeddedPage(page);
	m_pMainStack->setCurrentWidget(page);
	refreshPageGeometry();
	page->show();
	if (QLayout* stackLayout = m_pMainStack->layout())
	{
		stackLayout->invalidate();
		stackLayout->activate();
	}
	refreshPageGeometry();
	page->setFocus(Qt::OtherFocusReason);
	QTimer::singleShot(0, this, refreshPageGeometry);
	QTimer::singleShot(40, this, refreshPageGeometry);
	QTimer::singleShot(160, this, refreshPageGeometry);
}

void QtWidgetsApplication4::ShowManagementEmbeddedPage(QWidget* page)
{
	if (page == nullptr || m_pManagementStack == nullptr)
	{
		return;
	}

	auto refreshPageGeometry = [this, page]()
		{
			if (m_pManagementStack == nullptr || m_pManagementStack->currentWidget() != page)
			{
				return;
			}
			page->setGeometry(m_pManagementStack->contentsRect());
			page->updateGeometry();
			PositionEmbeddedBackButton(page);
			if (QLayout* pageLayout = page->layout())
			{
				pageLayout->invalidate();
				pageLayout->activate();
			}
			page->update();
		};

	PrepareEmbeddedPage(page, m_pManagementStack);
	m_pManagementStack->setCurrentWidget(page);
	refreshPageGeometry();
	page->show();
	if (QLayout* stackLayout = m_pManagementStack->layout())
	{
		stackLayout->invalidate();
		stackLayout->activate();
	}
	refreshPageGeometry();
	page->setFocus(Qt::OtherFocusReason);
	QTimer::singleShot(0, this, refreshPageGeometry);
	QTimer::singleShot(40, this, refreshPageGeometry);
	QTimer::singleShot(160, this, refreshPageGeometry);
	if (m_pManagementPage != nullptr)
	{
		ShowMaximizedWithUnifiedChrome(m_pManagementPage);
		m_pManagementPage->raise();
		m_pManagementPage->activateWindow();
	}
}

QStackedWidget* QtWidgetsApplication4::CurrentEmbeddedTargetStack() const
{
	if (m_bOpenEmbeddedInManagement && m_pManagementStack != nullptr)
	{
		return m_pManagementStack;
	}
	return m_pMainStack;
}

void QtWidgetsApplication4::ShowCurrentEmbeddedPage(QWidget* page)
{
	if (CurrentEmbeddedTargetStack() == m_pManagementStack)
	{
		ShowManagementEmbeddedPage(page);
		return;
	}
	ShowEmbeddedPage(page);
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
	auto containsAnyArgument = [&arguments](std::initializer_list<const char*> names) -> bool
		{
			for (const char* name : names)
			{
				if (arguments.contains(QString::fromLatin1(name)))
				{
					return true;
				}
			}
			return false;
		};

	const bool opensWindowForCli = containsAnyArgument({
		"--open-function-test",
		"--open-jog",
		"--open-precise-measure",
		"--open-camera-param"
		});
	const bool hasAutoExitCliAction = containsAnyArgument({
		"--no-show",
		"--robot-connect",
		"--robot-movel",
		"--robot-movel-relative",
		"--robot-movj",
		"--fanuc-connect",
		"--fanuc-upload-services",
		"--fanuc-curpos-diag",
		"--fanuc-pr20-diag",
		"--fanuc-raw",
		"--fanuc-call",
		"--measure-then-weld-scan-only-repeat",
		"--laser-classify",
		"--laser-classify-dir",
		"--apply-weld-seam-comp",
		"--update-weld-pose-average"
		});

	if (arguments.contains("--help-cli"))
	{
		QTextStream out(stdout);
		out << "QtWidgetsApplication4 command line options:\n";
		out << "  --no-show                         不显示主窗口，适合自动测试\n";
		out << "  --open-function-test              打开机器人功能测试窗口\n";
		out << "  --open-jog                        打开机器人点动控制窗口\n";
		out << "  --open-precise-measure            打开测量焊接参数窗口\n";
		out << "  --open-camera-param               打开相机参数窗口\n";
		out << "  --robot <UnitNo|RobotA|RobotB|中文名> 选择通用机器人CLI目标，默认当前/第一个可用机器人\n";
		out << "  --robot-connect                   连接选中的机器人驱动\n";
		out << "  --robot-movel <X,Y,Z,RX,RY,RZ[,BX,BY,BZ]> 发送直角 MOVL，默认速度500mm/min\n";
		out << "  --robot-movel-relative <DX,DY,DZ[,DRX,DRY,DRZ,BX,BY,BZ]> 基于当前位置做直角相对MOVL\n";
		out << "  --robot-movj <S,L,U,R,B,T[,EX1,EX2,EX3]> 发送关节脉冲 MOVJ，默认速度1%\n";
		out << "  --robot-speed <VALUE>             覆盖本次运动速度，MOVL按mm/min，MOVJ按驱动百分比/约定\n";
		out << "  --robot-done-delay <ms>           运动完成轮询间隔，默认200ms\n";
		out << "  --robot-no-wait                   运动下发后不等待完成\n";
		out << "  --fanuc-connect                   连接 FANUC 常驻服务端口\n";
		out << "  --fanuc-upload-services           上传/编译 FANUC 服务库和固定 TP\n";
		out << "  --skip-upload-wait                上传服务后不等待回车，自动化测试用\n";
		out << "  --fanuc-curpos-diag               运行当前位置/PR20 诊断命令\n";
		out << "  --fanuc-pr20-diag                 仅读取 FANUC PR[20] 诊断点\n";
		out << "  --fanuc-raw <CMD>                 发送一条原始 FANUC 服务命令\n";
		out << "  --fanuc-call <PROGRAM>            调用机器人程序\n";
		out << "  --measure-then-weld-scan-only-repeat <N> 自动执行先测后焊扫描流程N次，仅到收枪安全位置，不执行焊接\n";
		out << "  --measure-then-weld-scan-speed <mm/min> 覆盖本次CLI先测后焊扫描速度，不修改ini\n";
		out << "  --measure-then-weld-camera-offset-ms <ms> 覆盖本次CLI相机时间补偿，不修改ini\n";
		out << "  --laser-classify <FILE>           对激光点云做去噪/拟合/起终点拐点分类\n";
		out << "  --laser-classify-dir <DIR>        批量处理目录下所有 PreciseLaserPoint.txt\n";
		out << "  --laser-classify-output <FILE>    指定分类结果输出文件\n";
		out << "  --apply-weld-seam-comp <FILE>     对焊道姿态文件应用 WeldSeamCompParam.ini 补偿\n";
		out << "  --apply-weld-seam-comp-output <FILE> 指定补偿结果输出文件，默认另存 _SeamComp\n";
		out << "  --update-weld-pose-average <FILE_OR_DIR> 离线统计四类焊道平均姿态并更新补偿姿态库\n";
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
		LogCommandLineMessage("CLI 打开测量焊接参数窗口");
		OpenPreciseMeasureEditDialog();
	}
	if (arguments.contains("--open-camera-param"))
	{
		LogCommandLineMessage("CLI 打开相机参数窗口");
		OpenCameraParamDialog();
	}

	const int scanOnlyRepeatIndex = arguments.indexOf("--measure-then-weld-scan-only-repeat");
	int scanOnlyRepeatCount = 0;
	if (scanOnlyRepeatIndex >= 0 && scanOnlyRepeatIndex + 1 < arguments.size())
	{
		bool ok = false;
		scanOnlyRepeatCount = arguments[scanOnlyRepeatIndex + 1].toInt(&ok);
		if (!ok || scanOnlyRepeatCount <= 0)
		{
			LogCommandLineMessage("CLI 先测后焊扫描重复次数无效，请使用 --measure-then-weld-scan-only-repeat <N>。");
			scanOnlyRepeatCount = 0;
		}
	}
	const int scanSpeedOverrideIndex = arguments.indexOf("--measure-then-weld-scan-speed");
	double scanSpeedOverrideMmPerMin = 0.0;
	if (scanSpeedOverrideIndex >= 0 && scanSpeedOverrideIndex + 1 < arguments.size())
	{
		bool ok = false;
		scanSpeedOverrideMmPerMin = arguments[scanSpeedOverrideIndex + 1].toDouble(&ok);
		if (!ok || !std::isfinite(scanSpeedOverrideMmPerMin) || scanSpeedOverrideMmPerMin <= 0.0)
		{
			LogCommandLineMessage("CLI 先测后焊扫描速度无效，请使用 --measure-then-weld-scan-speed <mm/min>。");
			scanSpeedOverrideMmPerMin = 0.0;
		}
	}
	const int cameraOffsetOverrideIndex = arguments.indexOf("--measure-then-weld-camera-offset-ms");
	double cameraTimeOffsetOverrideMs = std::numeric_limits<double>::quiet_NaN();
	if (cameraOffsetOverrideIndex >= 0 && cameraOffsetOverrideIndex + 1 < arguments.size())
	{
		bool ok = false;
		cameraTimeOffsetOverrideMs = arguments[cameraOffsetOverrideIndex + 1].toDouble(&ok);
		if (!ok || !std::isfinite(cameraTimeOffsetOverrideMs))
		{
			LogCommandLineMessage("CLI 相机时间补偿无效，请使用 --measure-then-weld-camera-offset-ms <ms>。");
			cameraTimeOffsetOverrideMs = std::numeric_limits<double>::quiet_NaN();
		}
	}

	const int laserClassifyIndex = arguments.indexOf("--laser-classify");
	if (laserClassifyIndex >= 0 && laserClassifyIndex + 1 < arguments.size())
	{
		const QString inputPath = arguments[laserClassifyIndex + 1];
		QString outputPath;
		const int laserOutputIndex = arguments.indexOf("--laser-classify-output");
		if (laserOutputIndex >= 0 && laserOutputIndex + 1 < arguments.size())
		{
			outputPath = arguments[laserOutputIndex + 1];
		}
		RunLaserClassifyForCli(inputPath, outputPath);
	}

	const int laserClassifyDirIndex = arguments.indexOf("--laser-classify-dir");
	if (laserClassifyDirIndex >= 0 && laserClassifyDirIndex + 1 < arguments.size())
	{
		RunLaserClassifyDirForCli(arguments[laserClassifyDirIndex + 1]);
	}

	const int weldSeamCompIndex = arguments.indexOf("--apply-weld-seam-comp");
	if (weldSeamCompIndex >= 0 && weldSeamCompIndex + 1 < arguments.size())
	{
		const QString inputPath = arguments[weldSeamCompIndex + 1];
		QString outputPath;
		const int weldSeamCompOutputIndex = arguments.indexOf("--apply-weld-seam-comp-output");
		if (weldSeamCompOutputIndex >= 0 && weldSeamCompOutputIndex + 1 < arguments.size())
		{
			outputPath = arguments[weldSeamCompOutputIndex + 1];
		}
		RunWeldSeamCompForCli(inputPath, outputPath);
	}

	const int updateWeldPoseAverageIndex = arguments.indexOf("--update-weld-pose-average");
	if (updateWeldPoseAverageIndex >= 0 && updateWeldPoseAverageIndex + 1 < arguments.size())
	{
		RunUpdateWeldPoseAverageForCli(arguments[updateWeldPoseAverageIndex + 1]);
	}

	RunRobotMotionForCli(arguments);

	FANUCRobotCtrl* pFanucDriver = GetFirstFanucDriverForCli();
	const bool needsFanuc = arguments.contains("--fanuc-connect")
		|| arguments.contains("--fanuc-upload-services")
		|| arguments.contains("--fanuc-curpos-diag")
		|| arguments.contains("--fanuc-pr20-diag")
		|| arguments.contains("--fanuc-raw")
		|| arguments.contains("--fanuc-call")
		|| scanOnlyRepeatCount > 0;
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
			|| arguments.contains("--fanuc-call")
			|| scanOnlyRepeatCount > 0;

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

		if (socketReady && scanOnlyRepeatCount > 0)
		{
			const bool scanOk = RunMeasureThenWeldScanOnlyRepeatForCli(
				pFanucDriver,
				scanOnlyRepeatCount,
				scanSpeedOverrideMmPerMin,
				cameraTimeOffsetOverrideMs);
			LogCommandLineMessage(QString("CLI 先测后焊扫描重复流程%1。").arg(scanOk ? "完成" : "失败"));
		}
	}

	const int quitAfterIndex = arguments.indexOf("--quit-after");
	bool hasExplicitQuitAfter = false;
	if (quitAfterIndex >= 0 && quitAfterIndex + 1 < arguments.size())
	{
		bool ok = false;
		const int quitAfterMs = arguments[quitAfterIndex + 1].toInt(&ok);
		if (ok && quitAfterMs >= 0)
		{
			hasExplicitQuitAfter = true;
			LogCommandLineMessage(QString("CLI 动作完成，将在 %1 ms 后退出").arg(quitAfterMs));
			QTimer::singleShot(quitAfterMs, QCoreApplication::instance(), &QCoreApplication::quit);
		}
	}
	if (!opensWindowForCli && !hasExplicitQuitAfter && hasAutoExitCliAction)
	{
		LogCommandLineMessage("CLI 动作完成，自动退出。");
		QTimer::singleShot(0, QCoreApplication::instance(), &QCoreApplication::quit);
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

RobotDriverAdaptor* QtWidgetsApplication4::GetRobotDriverForCli(const QStringList& arguments, QString* robotLabelOut) const
{
	if (robotLabelOut != nullptr)
	{
		robotLabelOut->clear();
	}
	if (m_pContralUnit == nullptr || m_pContralUnit->m_vtContralUnitInfo.empty())
	{
		LogCommandLineMessage("CLI 未找到机器人控制单元配置。");
		return nullptr;
	}

	QString selector = CliOptionValue(arguments, "--robot");
	if (selector.isEmpty())
	{
		selector = CliOptionValue(arguments, "--robot-unit");
	}

	const T_CONTRAL_UNIT* selectedUnit = nullptr;
	if (!selector.isEmpty())
	{
		bool numericOk = false;
		const int requestedIndex = selector.toInt(&numericOk);
		if (numericOk)
		{
			for (const T_CONTRAL_UNIT& unitInfo : m_pContralUnit->m_vtContralUnitInfo)
			{
				if (unitInfo.nUnitNo == requestedIndex)
				{
					selectedUnit = &unitInfo;
					break;
				}
			}
			if (selectedUnit == nullptr
				&& requestedIndex >= 0
				&& requestedIndex < static_cast<int>(m_pContralUnit->m_vtContralUnitInfo.size()))
			{
				selectedUnit = &m_pContralUnit->m_vtContralUnitInfo[requestedIndex];
			}
		}
		else
		{
			for (const T_CONTRAL_UNIT& unitInfo : m_pContralUnit->m_vtContralUnitInfo)
			{
				RobotDriverAdaptor* driver = static_cast<RobotDriverAdaptor*>(unitInfo.pUnitDriver);
				const QString unitName = DecodeConfigText(unitInfo.sUnitName).trimmed();
				const QString chineseName = DecodeConfigText(unitInfo.sChineseName).trimmed();
				const QString driverName = driver == nullptr ? QString() : DecodeConfigText(driver->m_sRobotName).trimmed();
				const QString driverCustomName = driver == nullptr ? QString() : DecodeConfigText(driver->m_sCustomName).trimmed();
				const QString label = BuildRobotCliLabel(unitInfo);

				if (CliTextEquals(selector, unitName)
					|| CliTextEquals(selector, chineseName)
					|| CliTextEquals(selector, driverName)
					|| CliTextEquals(selector, driverCustomName)
					|| CliTextEquals(selector, label))
				{
					selectedUnit = &unitInfo;
					break;
				}
			}
		}

		if (selectedUnit == nullptr)
		{
			LogCommandLineMessage(QString("CLI 未找到 --robot 指定的机器人：%1").arg(selector));
			return nullptr;
		}
	}
	else
	{
		const int currentIndex = CurrentRobotUnitIndex();
		for (const T_CONTRAL_UNIT& unitInfo : m_pContralUnit->m_vtContralUnitInfo)
		{
			if (unitInfo.nUnitNo == currentIndex)
			{
				selectedUnit = &unitInfo;
				break;
			}
		}
		if (selectedUnit == nullptr
			&& currentIndex >= 0
			&& currentIndex < static_cast<int>(m_pContralUnit->m_vtContralUnitInfo.size()))
		{
			selectedUnit = &m_pContralUnit->m_vtContralUnitInfo[currentIndex];
		}
		if (selectedUnit == nullptr)
		{
			const int fallbackIndex = FindFirstReadyRobotUnitIndex();
			for (const T_CONTRAL_UNIT& unitInfo : m_pContralUnit->m_vtContralUnitInfo)
			{
				if (unitInfo.nUnitNo == fallbackIndex)
				{
					selectedUnit = &unitInfo;
					break;
				}
			}
		}
	}

	if (selectedUnit == nullptr)
	{
		LogCommandLineMessage("CLI 未找到可用机器人。");
		return nullptr;
	}

	RobotDriverAdaptor* driver = static_cast<RobotDriverAdaptor*>(selectedUnit->pUnitDriver);
	const QString robotLabel = BuildRobotCliLabel(*selectedUnit);
	if (robotLabelOut != nullptr)
	{
		*robotLabelOut = robotLabel;
	}
	if (driver == nullptr)
	{
		LogCommandLineMessage(QString("CLI 机器人驱动不可用：%1").arg(robotLabel));
		return nullptr;
	}
	return driver;
}

void QtWidgetsApplication4::RunRobotMotionForCli(const QStringList& arguments)
{
	const bool needsRobotCli = CliOptionContains(arguments, "--robot-connect")
		|| CliOptionContains(arguments, "--robot-movel")
		|| CliOptionContains(arguments, "--robot-movel-relative")
		|| CliOptionContains(arguments, "--robot-movj");
	if (!needsRobotCli)
	{
		return;
	}

	QString parseError;
	double speedOverride = 0.0;
	bool speedSpecified = false;
	if (!TryParseCliDoubleOption(arguments, "--robot-speed", speedOverride, speedSpecified, &parseError))
	{
		LogCommandLineMessage(QString("CLI 机器人运动参数错误：%1").arg(parseError));
		return;
	}

	int doneDelayMs = 200;
	bool doneDelaySpecified = false;
	if (!TryParseCliIntOption(arguments, "--robot-done-delay", doneDelayMs, doneDelaySpecified, &parseError))
	{
		LogCommandLineMessage(QString("CLI 机器人运动参数错误：%1").arg(parseError));
		return;
	}
	(void)doneDelaySpecified;

	QString robotLabel;
	RobotDriverAdaptor* driver = GetRobotDriverForCli(arguments, &robotLabel);
	if (driver == nullptr)
	{
		return;
	}

	bool connected = driver->IsConnected();
	if (!connected)
	{
		connected = driver->InitSocket(driver->m_sSocketIP.c_str(), static_cast<u_short>(driver->m_nSocketPort));
	}
	LogCommandLineMessage(QString("CLI 机器人连接%1：%2，地址=%3:%4")
		.arg(connected ? "成功" : "失败")
		.arg(robotLabel)
		.arg(QString::fromStdString(driver->m_sSocketIP))
		.arg(driver->m_nSocketPort));
	if (!connected)
	{
		const QString lastError = DecodeRobotMessageText(driver->GetLastRobotError());
		if (!lastError.isEmpty())
		{
			LogCommandLineMessage(QString("CLI 机器人连接错误：%1").arg(lastError));
		}
		return;
	}

	const bool noWait = CliOptionContains(arguments, "--robot-no-wait");
	bool executedMotion = false;
	for (int i = 1; i < arguments.size(); ++i)
	{
		if (arguments[i] == "--robot-movel")
		{
			executedMotion = true;
			if (i + 1 >= arguments.size())
			{
				LogCommandLineMessage("CLI MOVL失败：--robot-movel 缺少目标值。");
				continue;
			}

			QVector<double> values;
			QString errorText;
			if (!ParseCliDoubleValues(arguments[++i], 6, 9, values, &errorText))
			{
				LogCommandLineMessage(QString("CLI MOVL目标解析失败：%1").arg(errorText));
				continue;
			}

			T_ROBOT_COORS target;
			target.dX = values[0];
			target.dY = values[1];
			target.dZ = values[2];
			target.dRX = values[3];
			target.dRY = values[4];
			target.dRZ = values[5];
			target.dBX = values.size() > 6 ? values[6] : 0.0;
			target.dBY = values.size() > 7 ? values[7] : 0.0;
			target.dBZ = values.size() > 8 ? values[8] : 0.0;

			const double moveSpeed = speedSpecified ? speedOverride : 500.0;
			LogCommandLineMessage(QString("CLI MOVL下发：机器人=%1，速度=%2mm/min，目标=%3")
				.arg(robotLabel)
				.arg(moveSpeed, 0, 'f', 3)
				.arg(FormatCliCoors(target)));
			const bool moveOk = driver->MoveByJob(target, T_ROBOT_MOVE_SPEED(moveSpeed, 0.0, 0.0), driver->m_nExternalAxleType, "MOVL");
			const int done = (moveOk && !noWait) ? driver->CheckRobotDone(doneDelayMs) : -1;
			LogCommandLineMessage(QString("CLI MOVL结果：Move=%1%2，CheckRobotDone=%3，状态=%4，最近错误=%5")
				.arg(moveOk ? "OK" : "FAIL")
				.arg(noWait ? "，已跳过完成等待" : QString())
				.arg(done)
				.arg(DecodeRobotMessageText(driver->GetRobotStatusText()))
				.arg(DecodeRobotMessageText(driver->GetLastRobotError())));
		}
		else if (arguments[i] == "--robot-movel-relative")
		{
			executedMotion = true;
			if (i + 1 >= arguments.size())
			{
				LogCommandLineMessage("CLI 相对MOVL失败：--robot-movel-relative 缺少增量值。");
				continue;
			}

			QVector<double> values;
			QString errorText;
			if (!ParseCliDoubleValues(arguments[++i], 3, 9, values, &errorText))
			{
				LogCommandLineMessage(QString("CLI 相对MOVL增量解析失败：%1").arg(errorText));
				continue;
			}

			const T_ROBOT_COORS current = driver->GetCurrentPos();
			T_ROBOT_COORS target = current;
			target.dX += values[0];
			target.dY += values[1];
			target.dZ += values[2];
			target.dRX += values.size() > 3 ? values[3] : 0.0;
			target.dRY += values.size() > 4 ? values[4] : 0.0;
			target.dRZ += values.size() > 5 ? values[5] : 0.0;
			target.dBX += values.size() > 6 ? values[6] : 0.0;
			target.dBY += values.size() > 7 ? values[7] : 0.0;
			target.dBZ += values.size() > 8 ? values[8] : 0.0;

			const double moveSpeed = speedSpecified ? speedOverride : 500.0;
			LogCommandLineMessage(QString("CLI 相对MOVL下发：机器人=%1，速度=%2mm/min，当前位置=%3，目标=%4")
				.arg(robotLabel)
				.arg(moveSpeed, 0, 'f', 3)
				.arg(FormatCliCoors(current))
				.arg(FormatCliCoors(target)));
			const bool moveOk = driver->MoveByJob(target, T_ROBOT_MOVE_SPEED(moveSpeed, 0.0, 0.0), driver->m_nExternalAxleType, "MOVL");
			const int done = (moveOk && !noWait) ? driver->CheckRobotDone(doneDelayMs) : -1;
			LogCommandLineMessage(QString("CLI 相对MOVL结果：Move=%1%2，CheckRobotDone=%3，状态=%4，最近错误=%5")
				.arg(moveOk ? "OK" : "FAIL")
				.arg(noWait ? "，已跳过完成等待" : QString())
				.arg(done)
				.arg(DecodeRobotMessageText(driver->GetRobotStatusText()))
				.arg(DecodeRobotMessageText(driver->GetLastRobotError())));
		}
		else if (arguments[i] == "--robot-movj")
		{
			executedMotion = true;
			if (i + 1 >= arguments.size())
			{
				LogCommandLineMessage("CLI MOVJ失败：--robot-movj 缺少目标值。");
				continue;
			}

			QVector<double> values;
			QString errorText;
			if (!ParseCliDoubleValues(arguments[++i], 6, 9, values, &errorText))
			{
				LogCommandLineMessage(QString("CLI MOVJ目标解析失败：%1").arg(errorText));
				continue;
			}

			const auto toPulse = [](double value) -> long
				{
					return static_cast<long>(std::llround(value));
				};
			T_ANGLE_PULSE target;
			target.nSPulse = toPulse(values[0]);
			target.nLPulse = toPulse(values[1]);
			target.nUPulse = toPulse(values[2]);
			target.nRPulse = toPulse(values[3]);
			target.nBPulse = toPulse(values[4]);
			target.nTPulse = toPulse(values[5]);
			target.lBXPulse = values.size() > 6 ? toPulse(values[6]) : 0;
			target.lBYPulse = values.size() > 7 ? toPulse(values[7]) : 0;
			target.lBZPulse = values.size() > 8 ? toPulse(values[8]) : 0;

			const double moveSpeed = speedSpecified ? speedOverride : 1.0;
			LogCommandLineMessage(QString("CLI MOVJ下发：机器人=%1，速度=%2，目标脉冲=%3")
				.arg(robotLabel)
				.arg(moveSpeed, 0, 'f', 3)
				.arg(FormatCliPulse(target)));
			const bool moveOk = driver->MoveByJob(target, T_ROBOT_MOVE_SPEED(moveSpeed, 0.0, 0.0), driver->m_nExternalAxleType, "MOVJ");
			const int done = (moveOk && !noWait) ? driver->CheckRobotDone(doneDelayMs) : -1;
			LogCommandLineMessage(QString("CLI MOVJ结果：Move=%1%2，CheckRobotDone=%3，状态=%4，最近错误=%5")
				.arg(moveOk ? "OK" : "FAIL")
				.arg(noWait ? "，已跳过完成等待" : QString())
				.arg(done)
				.arg(DecodeRobotMessageText(driver->GetRobotStatusText()))
				.arg(DecodeRobotMessageText(driver->GetLastRobotError())));
		}
	}

	if (!executedMotion && CliOptionContains(arguments, "--robot-connect"))
	{
		LogCommandLineMessage(QString("CLI 机器人连接完成：%1，状态=%2")
			.arg(robotLabel, DecodeRobotMessageText(driver->GetRobotStatusText())));
	}
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
		{ "通用任务运行器", "SDK/FANUC/FanucJobRunner.kl", QString(), 0 },
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

bool QtWidgetsApplication4::RunLaserClassifyForCli(const QString& inputPath, const QString& outputPath) const
{
	QString normalizedInputPath = QDir::fromNativeSeparators(inputPath.trimmed());
	if (normalizedInputPath.isEmpty())
	{
		LogCommandLineMessage("CLI 激光点云分类失败：输入文件为空。");
		return false;
	}

	QFileInfo inputInfo(normalizedInputPath);
	if (!inputInfo.isAbsolute())
	{
		inputInfo = QFileInfo(QDir::current().filePath(normalizedInputPath));
	}
	if (!inputInfo.exists())
	{
		LogCommandLineMessage(QString("CLI 激光点云分类失败：未找到输入文件 %1")
			.arg(QDir::toNativeSeparators(inputInfo.absoluteFilePath())));
		return false;
	}

	QVector<RobotCalculation::IndexedPoint3D> inputPoints;
	QString error;
	if (!RobotDataHelper::LoadIndexedPoint3DFile(inputInfo.absoluteFilePath(), inputPoints, &error))
	{
		LogCommandLineMessage("CLI 激光点云分类失败：" + error);
		return false;
	}

	const RobotCalculation::SampleAxis sampleAxis = InferLaserSampleAxis(inputPoints);
	const RobotCalculation::LowerWeldFilterParams originalParams = BuildCliOriginalTrackFitParams(sampleAxis);
	const RobotCalculation::MeasureThenWeldAnalysisResult analysisResult =
		RobotCalculation::AnalyzeMeasureThenWeldLowerWeldPathDirect(inputPoints, originalParams);
	if (!analysisResult.ok)
	{
		LogCommandLineMessage("CLI 激光点云分类失败，起终点/拐点特征提取失败：" + analysisResult.error);
		return false;
	}
	const RobotCalculation::LowerWeldFilterResult& originalFitResult = analysisResult.filterResult;
	const RobotCalculation::LowerWeldClassificationResult& classifiedResult = analysisResult.classificationResult;

	const QString normalizedOutputPath = outputPath.trimmed().isEmpty()
		? BuildClassifiedOutputPath(inputInfo.absoluteFilePath())
		: QDir::fromNativeSeparators(outputPath.trimmed());
	const QString classifiedOutputPath = QFileInfo(normalizedOutputPath).isAbsolute()
		? QFileInfo(normalizedOutputPath).absoluteFilePath()
		: QFileInfo(QDir::current().filePath(normalizedOutputPath)).absoluteFilePath();
	const QString noiseOutputPath = BuildNoiseOutputPath(classifiedOutputPath);
	const QString keyPointsOutputPath = BuildKeyPointsOutputPath(classifiedOutputPath);

	QSet<int> validIndexes;
	validIndexes.reserve(originalFitResult.points.size());
	for (const RobotCalculation::LowerWeldFilterPoint& point : originalFitResult.points)
	{
		validIndexes.insert(point.index);
	}

	QStringList classifiedLines;
	classifiedLines << "# index x y z type_code type_name source";
	classifiedLines << "# 1=start 2=end 3=inner_corner 4=outer_corner 5=normal 6=noise";
	for (const RobotCalculation::LowerWeldClassifiedPoint& point : classifiedResult.points)
	{
		classifiedLines << QString("%1 %2 %3 %4 %5 %6 %7")
			.arg(point.index)
			.arg(point.point.x(), 0, 'f', 6)
			.arg(point.point.y(), 0, 'f', 6)
			.arg(point.point.z(), 0, 'f', 6)
			.arg(RobotCalculation::LowerWeldPointTypeCode(point.type))
			.arg(RobotCalculation::LowerWeldPointTypeName(point.type))
			.arg(point.source.isEmpty() ? "-" : point.source);
	}

	QStringList keyPointLines;
	keyPointLines << "# source_index x y z type_code type_name source";
	keyPointLines << "# 1=start 2=end 3=inner_corner 4=outer_corner";
	for (const RobotCalculation::LowerWeldClassifiedPoint& point : analysisResult.keyPoints)
	{
		if (point.type == RobotCalculation::LowerWeldPointType::Normal
			|| point.type == RobotCalculation::LowerWeldPointType::Noise)
		{
			continue;
		}

		keyPointLines << QString("%1 %2 %3 %4 %5 %6 %7")
			.arg(point.index)
			.arg(point.point.x(), 0, 'f', 6)
			.arg(point.point.y(), 0, 'f', 6)
			.arg(point.point.z(), 0, 'f', 6)
			.arg(RobotCalculation::LowerWeldPointTypeCode(point.type))
			.arg(RobotCalculation::LowerWeldPointTypeName(point.type))
			.arg(point.source.isEmpty() ? "-" : point.source);
	}

	QStringList noiseLines;
	noiseLines << "# index x y z type_code type_name source";
	noiseLines << "# noise points filtered out before final weld fit";
	int noiseCount = 0;
	for (const RobotCalculation::IndexedPoint3D& point : inputPoints)
	{
		if (validIndexes.contains(point.index))
		{
			continue;
		}

		++noiseCount;
		noiseLines << QString("%1 %2 %3 %4 %5 %6 %7")
			.arg(point.index)
			.arg(point.point.x(), 0, 'f', 6)
			.arg(point.point.y(), 0, 'f', 6)
			.arg(point.point.z(), 0, 'f', 6)
			.arg(RobotCalculation::LowerWeldPointTypeCode(RobotCalculation::LowerWeldPointType::Noise))
			.arg(RobotCalculation::LowerWeldPointTypeName(RobotCalculation::LowerWeldPointType::Noise))
			.arg("raw_noise");
	}

	if (!RobotDataHelper::SaveTextFileLines(classifiedOutputPath, classifiedLines, &error))
	{
		LogCommandLineMessage("CLI 激光点云分类失败，保存分类文件失败：" + error);
		return false;
	}
	if (!RobotDataHelper::SaveTextFileLines(keyPointsOutputPath, keyPointLines, &error))
	{
		LogCommandLineMessage("CLI 激光点云分类失败，保存起终点/拐点文件失败：" + error);
		return false;
	}
	if (!RobotDataHelper::SaveTextFileLines(noiseOutputPath, noiseLines, &error))
	{
		LogCommandLineMessage("CLI 激光点云分类失败，保存杂点文件失败：" + error);
		return false;
	}

	LogCommandLineMessage(QString("CLI 激光点云分类完成（起终点/拐点特征）：输入=%1，主轴=%2，分类点=%3，杂点=%4")
		.arg(QDir::toNativeSeparators(inputInfo.absoluteFilePath()))
		.arg(sampleAxis == RobotCalculation::SampleAxis::AxisX ? "X" : "Y")
		.arg(classifiedResult.points.size())
		.arg(noiseCount));
	LogCommandLineMessage(QString("CLI 分类统计：起点=%1 终点=%2 内拐点=%3 外拐点=%4 普通点=%5")
		.arg(classifiedResult.startCount)
		.arg(classifiedResult.endCount)
		.arg(classifiedResult.innerCornerCount)
		.arg(classifiedResult.outerCornerCount)
		.arg(classifiedResult.normalCount));
	LogCommandLineMessage(QString("CLI 分类结果文件：%1")
		.arg(QDir::toNativeSeparators(classifiedOutputPath)));
	LogCommandLineMessage(QString("CLI 起终点/拐点文件：%1")
		.arg(QDir::toNativeSeparators(keyPointsOutputPath)));
	LogCommandLineMessage(QString("CLI 杂点文件：%1")
		.arg(QDir::toNativeSeparators(noiseOutputPath)));
	return true;
}

void QtWidgetsApplication4::RunLaserClassifyDirForCli(const QString& dirPath) const
{
	QString normalizedDirPath = QDir::fromNativeSeparators(dirPath.trimmed());
	if (normalizedDirPath.isEmpty())
	{
		LogCommandLineMessage("CLI 批量激光点云分类失败：目录为空。");
		return;
	}

	QDir rootDir(normalizedDirPath);
	if (!rootDir.isAbsolute())
	{
		rootDir = QDir(QDir::current().filePath(normalizedDirPath));
	}
	if (!rootDir.exists())
	{
		LogCommandLineMessage(QString("CLI 批量激光点云分类失败：未找到目录 %1")
			.arg(QDir::toNativeSeparators(rootDir.absolutePath())));
		return;
	}

	int totalCount = 0;
	int successCount = 0;
	QStringList failedFiles;
	QDirIterator iterator(
		rootDir.absolutePath(),
		QStringList() << "PreciseLaserPoint.txt",
		QDir::Files,
		QDirIterator::Subdirectories);
	while (iterator.hasNext())
	{
		const QString inputFilePath = iterator.next();
		const QString outputFilePath = BuildGeometryClassifiedOutputPath(inputFilePath);
		++totalCount;
		if (RunLaserClassifyForCli(inputFilePath, outputFilePath))
		{
			++successCount;
			continue;
		}
		failedFiles << QDir::toNativeSeparators(inputFilePath);
	}

	LogCommandLineMessage(QString("CLI 批量激光点云分类完成：目录=%1，总数=%2，成功=%3，失败=%4")
		.arg(QDir::toNativeSeparators(rootDir.absolutePath()))
		.arg(totalCount)
		.arg(successCount)
		.arg(failedFiles.size()));
	if (!failedFiles.isEmpty())
	{
		LogCommandLineMessage("CLI 批量失败文件：");
		for (const QString& failedFile : failedFiles)
		{
			LogCommandLineMessage("  " + failedFile);
		}
	}
}

void QtWidgetsApplication4::RunWeldSeamCompForCli(const QString& inputPath, const QString& outputPath) const
{
	QString normalizedInputPath = QDir::fromNativeSeparators(inputPath.trimmed());
	if (normalizedInputPath.isEmpty())
	{
		LogCommandLineMessage("CLI 焊道补偿失败：输入文件为空。");
		return;
	}

	QFileInfo inputInfo(normalizedInputPath);
	if (!inputInfo.isAbsolute())
	{
		inputInfo = QFileInfo(QDir::current().filePath(normalizedInputPath));
	}
	if (!inputInfo.exists())
	{
		LogCommandLineMessage(QString("CLI 焊道补偿失败：未找到输入文件 %1")
			.arg(QDir::toNativeSeparators(inputInfo.absoluteFilePath())));
		return;
	}

	const QString normalizedOutputPath = outputPath.trimmed().isEmpty()
		? BuildWeldSeamCompOutputPath(inputInfo.absoluteFilePath())
		: QDir::fromNativeSeparators(outputPath.trimmed());
	const QString resolvedOutputPath = QFileInfo(normalizedOutputPath).isAbsolute()
		? QFileInfo(normalizedOutputPath).absoluteFilePath()
		: QFileInfo(QDir::current().filePath(normalizedOutputPath)).absoluteFilePath();

	const Qt::CaseSensitivity pathCaseSensitivity =
#ifdef Q_OS_WIN
		Qt::CaseInsensitive;
#else
		Qt::CaseSensitive;
#endif
	if (QString::compare(inputInfo.absoluteFilePath(), resolvedOutputPath, pathCaseSensitivity) == 0)
	{
		LogCommandLineMessage(QString("CLI 焊道补偿失败：输出文件不能覆盖输入文件 %1")
			.arg(QDir::toNativeSeparators(inputInfo.absoluteFilePath())));
		return;
	}

	const QString robotName = InferRobotNameFromResultPath(inputInfo.absoluteFilePath());
	MeasureThenWeldService service;
	QString summary;
	QString error;
	if (!service.ApplyWeldSeamCompToPoseFile(
		robotName,
		inputInfo.absoluteFilePath(),
		resolvedOutputPath,
		summary,
		error))
	{
		LogCommandLineMessage("CLI 焊道补偿失败：" + error);
		return;
	}

	LogCommandLineMessage(QString("CLI 焊道补偿完成：输入=%1，输出=%2，机器人=%3")
		.arg(QDir::toNativeSeparators(inputInfo.absoluteFilePath()))
		.arg(QDir::toNativeSeparators(resolvedOutputPath))
		.arg(robotName));
	LogCommandLineMessage("CLI 焊道补偿摘要：" + summary);
}

void QtWidgetsApplication4::RunUpdateWeldPoseAverageForCli(const QString& inputPath) const
{
	WeldPoseAverageUpdater::UpdateResult result;
	QString error;
	if (!WeldPoseAverageUpdater::UpdateFromInput(inputPath, QString(), result, error))
	{
		LogCommandLineMessage("CLI 姿态均值更新失败：" + error);
		return;
	}

	for (const QString& line : result.reportLines)
	{
		LogCommandLineMessage(line);
	}
	LogCommandLineMessage(QString("CLI 姿态均值更新完成：机器人=%1，新增姿态组=%2，复用姿态组=%3，报告=%4")
		.arg(result.robotName)
		.arg(result.addedSlotCount)
		.arg(result.reusedSlotCount)
		.arg(QDir::toNativeSeparators(result.reportPath)));
}

bool QtWidgetsApplication4::RunMeasureThenWeldScanOnlyRepeatForCli(
	FANUCRobotCtrl* pFanucDriver,
	int repeatCount,
	double scanSpeedOverrideMmPerMin,
	double cameraTimeOffsetOverrideMs)
{
	if (pFanucDriver == nullptr)
	{
		LogCommandLineMessage("CLI 先测后焊扫描失败：机器人驱动为空。");
		return false;
	}
	if (repeatCount <= 0)
	{
		LogCommandLineMessage("CLI 先测后焊扫描失败：重复次数必须大于0。");
		return false;
	}

	const int unitIndex = CurrentRobotUnitIndex();
	QString cameraIP;
	if (!EnsureScanCameraRunningForUnit(unitIndex, cameraIP, true))
	{
		LogCommandLineMessage("CLI 先测后焊扫描失败：未读取到测量相机IP。");
		return false;
	}
	CameraFrameCache* cameraCache = ScanCameraCacheForUnit(unitIndex);

	MeasureThenWeldService service;
	auto appendLog = [this](const QString& text)
		{
			LogCommandLineMessage("CLI 先测后焊扫描：" + text);
		};
	auto setFlowStep = [this](const QString& text)
		{
			LogCommandLineMessage("CLI 流程节点：" + text);
		};

	LogCommandLineMessage(QString("CLI 先测后焊扫描：相机接收已启动 %1，准备重复 %2 次。")
		.arg(cameraIP)
		.arg(repeatCount));
	QThread::msleep(500);

	bool allOk = true;
	for (int repeatIndex = 1; repeatIndex <= repeatCount; ++repeatIndex)
	{
		T_PRECISE_MEASURE_PARAM param;
		QString error;
		if (!service.LoadPresetParam(pFanucDriver, param, error))
		{
			LogCommandLineMessage(QString("CLI 第%1次扫描失败：读取预设参数失败：%2")
				.arg(repeatIndex)
				.arg(error));
			allOk = false;
			break;
		}
		if (std::isfinite(scanSpeedOverrideMmPerMin) && scanSpeedOverrideMmPerMin > 0.0)
		{
			param.dScanSpeed = scanSpeedOverrideMmPerMin;
		}
		if (std::isfinite(cameraTimeOffsetOverrideMs))
		{
			param.dCameraTimeOffsetMs = cameraTimeOffsetOverrideMs;
		}

		QString savedPath;
		LogCommandLineMessage(QString("CLI 第%1/%2次扫描开始：参数=%3 [%4]，ScanSpeed=%5 mm/min%6，CameraReadFps=%7，CameraTimeOffsetMs=%8%9")
			.arg(repeatIndex)
			.arg(repeatCount)
			.arg(QString::fromStdString(param.sIniFilePath))
			.arg(QString::fromStdString(param.sSectionName))
			.arg(param.dScanSpeed, 0, 'f', 3)
			.arg(std::isfinite(scanSpeedOverrideMmPerMin) && scanSpeedOverrideMmPerMin > 0.0 ? "（CLI覆盖）" : "")
			.arg(param.dCameraReadFps, 0, 'f', 3)
			.arg(param.dCameraTimeOffsetMs, 0, 'f', 3)
			.arg(std::isfinite(cameraTimeOffsetOverrideMs) ? "（CLI覆盖）" : ""));

		const double runSpeed = std::isfinite(param.dRunSpeed) && param.dRunSpeed > 0.0
			? param.dRunSpeed
			: 1.0;
		bool ok = service.MovePulseListAndWait(
			pFanucDriver,
			param.vtStartSafePulse,
			runSpeed,
			QString("CLI第%1次下枪安全姿态").arg(repeatIndex),
			appendLog,
			setFlowStep);
		if (ok)
		{
			ok = service.MoveCoorsAndWait(
				pFanucDriver,
				param.tStartPos,
				runSpeed,
				QString("CLI第%1次扫描起点").arg(repeatIndex),
				appendLog,
				setFlowStep);
		}
		if (ok)
		{
			ok = service.ScanMoveAndCollect(
				pFanucDriver,
				param,
				savedPath,
				appendLog,
				setFlowStep,
				cameraCache);
		}
		if (ok)
		{
			ok = service.MovePulseListAndWait(
				pFanucDriver,
				param.vtEndSafePulse,
				runSpeed,
				QString("CLI第%1次收枪安全姿态").arg(repeatIndex),
				appendLog,
				setFlowStep);
		}

		if (!ok)
		{
			LogCommandLineMessage(QString("CLI 第%1次扫描流程失败，已停止后续重复。").arg(repeatIndex));
			allOk = false;
			break;
		}

		const QString savedText = savedPath.isEmpty()
			? QString("扫描文件已保存，未生成焊接姿态文件")
			: QString("最终姿态/补偿文件=%1").arg(QDir::toNativeSeparators(savedPath));
		LogCommandLineMessage(QString("CLI 第%1/%2次扫描完成，已到收枪安全位置，%3")
			.arg(repeatIndex)
			.arg(repeatCount)
			.arg(savedText));
	}

	QThread::msleep(300);
	LogCommandLineMessage("CLI 先测后焊扫描：扫描结束。");
	return allOk;
}

bool QtWidgetsApplication4::LoadGrooveCameraEndpointForUnit(int unitIndex, QString& cameraIP, int& cameraPort) const
{
	constexpr int kDefaultTcpSensorPort = 50006;
	std::string robotName = "RobotA";
	const T_CONTRAL_UNIT* selectedUnit = nullptr;
	if (m_pContralUnit != nullptr)
	{
		for (const T_CONTRAL_UNIT& unitInfo : m_pContralUnit->m_vtContralUnitInfo)
		{
			if (unitInfo.nUnitNo == unitIndex)
			{
				selectedUnit = &unitInfo;
				break;
			}
		}
		if (selectedUnit == nullptr
			&& unitIndex >= 0
			&& unitIndex < static_cast<int>(m_pContralUnit->m_vtContralUnitInfo.size()))
		{
			selectedUnit = &m_pContralUnit->m_vtContralUnitInfo[unitIndex];
		}
	}
	if (selectedUnit != nullptr)
	{
		RobotDriverAdaptor* driver = static_cast<RobotDriverAdaptor*>(selectedUnit->pUnitDriver);
		if (driver != nullptr && !driver->m_sRobotName.empty())
		{
			robotName = driver->m_sRobotName;
		}
		else if (!selectedUnit->sUnitName.empty())
		{
			robotName = selectedUnit->sUnitName;
		}
	}

    RobotDataHelper::CameraParamData cameraParam;
	const QString robotNameText = DecodeConfigText(robotName);
    const QString cameraSection = RobotDataHelper::MeasureCameraSection(robotNameText);
    if (!RobotDataHelper::LoadCameraParam(robotNameText, cameraSection, cameraParam, nullptr)
		|| cameraParam.deviceAddress.trimmed().isEmpty())
	{
		return false;
	}

	cameraIP = cameraParam.deviceAddress.trimmed();
	// DevicePort 仍保留旧UDP端口配置；新版坡口相机TCP预览固定连接 PointCloundTcpServer。
	cameraPort = kDefaultTcpSensorPort;
	return true;
}

bool QtWidgetsApplication4::LoadGrooveCameraIPForUnit(int unitIndex, QString& cameraIP) const
{
	int cameraPort = 0;
	return LoadGrooveCameraEndpointForUnit(unitIndex, cameraIP, cameraPort);
}

bool QtWidgetsApplication4::LoadGrooveCameraIP(QString& cameraIP) const
{
	return LoadGrooveCameraIPForUnit(CurrentRobotUnitIndex(), cameraIP);
}

void QtWidgetsApplication4::InitializeScanCameraRuntimes()
{
	if (m_pContralUnit == nullptr)
	{
		return;
	}

	m_scanCameraUnitByIP.clear();

	for (const T_CONTRAL_UNIT& unitInfo : m_pContralUnit->m_vtContralUnitInfo)
	{
		if (m_scanCameraRuntimes.contains(unitInfo.nUnitNo))
		{
			continue;
		}

		CameraRuntime* runtime = new CameraRuntime();
		runtime->cache = new CameraFrameCache();
		runtime->worker = new TcpSensorClientWorker(runtime->cache);
		runtime->thread = new QThread(this);
		m_liveScanCameraRuntimes.insert(runtime);
		connect(runtime->worker, &TcpSensorClientWorker::diagnosticChanged, this,
			[this, runtime](
				qint64 datagramCount,
				qint64 filteredDatagramCount,
				qint64 decodedFrameCount,
				qint64 decodeFailedCount,
				qint64 appendedFrameCount,
				const QString& statusText)
			{
				if (runtime == nullptr || !m_liveScanCameraRuntimes.contains(runtime))
				{
					return;
				}
				runtime->datagramCount = datagramCount;
				runtime->filteredDatagramCount = filteredDatagramCount;
				runtime->decodedFrameCount = decodedFrameCount;
				runtime->decodeFailedCount = decodeFailedCount;
				runtime->appendedFrameCount = appendedFrameCount;
				runtime->cameraStatus = statusText;
			});
		connect(runtime->thread, &QThread::finished, runtime->worker, &QObject::deleteLater);
		runtime->worker->moveToThread(runtime->thread);
		runtime->thread->start();
		m_scanCameraRuntimes.insert(unitInfo.nUnitNo, runtime);

		QString cameraIP;
		EnsureScanCameraRunningForUnit(unitInfo.nUnitNo, cameraIP, false);
	}
}

void QtWidgetsApplication4::StopScanCameraRuntimes()
{
	const QList<CameraRuntime*> runtimes = m_scanCameraRuntimes.values();
	const QList<CameraRuntime*> receiverRuntimes = m_scanCameraReceiversByPort.values();
	m_scanCameraRuntimes.clear();
	m_scanCameraReceiversByPort.clear();
	m_scanCameraUnitByIP.clear();
	auto stopRuntime = [this](CameraRuntime* runtime, bool deleteCache)
	{
		if (runtime == nullptr)
		{
			return;
		}
		m_liveScanCameraRuntimes.remove(runtime);
		if (runtime->worker != nullptr)
		{
			QObject::disconnect(runtime->worker, nullptr, this, nullptr);
		}
		if (runtime->worker != nullptr
			&& runtime->thread != nullptr
			&& runtime->thread->isRunning())
		{
			QMetaObject::invokeMethod(runtime->worker, "stopClient", Qt::BlockingQueuedConnection);
		}
		if (runtime->thread != nullptr)
		{
			runtime->thread->quit();
			runtime->thread->wait();
			delete runtime->thread;
			runtime->thread = nullptr;
		}
		if (deleteCache)
		{
			delete runtime->cache;
			runtime->cache = nullptr;
		}
		delete runtime;
	};
	for (CameraRuntime* receiverRuntime : receiverRuntimes)
	{
		stopRuntime(receiverRuntime, false);
	}
	for (CameraRuntime* runtime : runtimes)
	{
		stopRuntime(runtime, true);
	}
}

bool QtWidgetsApplication4::EnsureScanCameraRunningForUnit(int unitIndex, QString& cameraIP, bool clearCache)
{
	int cameraPort = 0;
	if (!LoadGrooveCameraEndpointForUnit(unitIndex, cameraIP, cameraPort))
	{
		return false;
	}

	CameraRuntime* runtime = m_scanCameraRuntimes.value(unitIndex, nullptr);

	if (runtime == nullptr)
	{
		runtime = new CameraRuntime();
		runtime->cache = new CameraFrameCache();
		runtime->worker = new TcpSensorClientWorker(runtime->cache);
		runtime->thread = new QThread(this);
		m_liveScanCameraRuntimes.insert(runtime);
		connect(runtime->worker, &TcpSensorClientWorker::diagnosticChanged, this,
			[this, runtime](
				qint64 datagramCount,
				qint64 filteredDatagramCount,
				qint64 decodedFrameCount,
				qint64 decodeFailedCount,
				qint64 appendedFrameCount,
				const QString& statusText)
			{
				if (runtime == nullptr || !m_liveScanCameraRuntimes.contains(runtime))
				{
					return;
				}
				runtime->datagramCount = datagramCount;
				runtime->filteredDatagramCount = filteredDatagramCount;
				runtime->decodedFrameCount = decodedFrameCount;
				runtime->decodeFailedCount = decodeFailedCount;
				runtime->appendedFrameCount = appendedFrameCount;
				runtime->cameraStatus = statusText;
			});
		connect(runtime->thread, &QThread::finished, runtime->worker, &QObject::deleteLater);
		runtime->worker->moveToThread(runtime->thread);
		runtime->thread->start();
		m_scanCameraRuntimes.insert(unitIndex, runtime);
	}
	if (runtime->cache != nullptr && clearCache)
	{
		runtime->cache->Clear();
	}

	const bool needRestart = !runtime->running
		|| runtime->cameraIP != cameraIP
		|| runtime->cameraPort != cameraPort;
	if (needRestart)
	{
		QMetaObject::invokeMethod(
			runtime->worker,
			"startClient",
			Qt::BlockingQueuedConnection,
			Q_ARG(QString, cameraIP),
			Q_ARG(int, cameraPort));
		runtime->cameraIP = cameraIP;
		runtime->cameraPort = cameraPort;
		runtime->running = true;
	}
	return true;
}

CameraFrameCache* QtWidgetsApplication4::ScanCameraCacheForUnit(int unitIndex) const
{
	CameraRuntime* runtime = m_scanCameraRuntimes.value(unitIndex, nullptr);
	return runtime != nullptr ? runtime->cache : nullptr;
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

void QtWidgetsApplication4::OpenGroovePointCloudDialog()
{
	if (m_pGroovePointCloudDialog == nullptr)
	{
		GroovePointCloudDialog* dialog = new GroovePointCloudDialog(this);
		m_pGroovePointCloudDialog = dialog;
		connect(dialog, &QObject::destroyed, this, [this]()
			{
				m_pGroovePointCloudDialog = nullptr;
				if (ui.GrooveCameraTestBtn != nullptr && ui.GrooveCameraTestBtn->isChecked())
				{
					ui.GrooveCameraTestBtn->setChecked(false);
				}
			});
	}

	m_pGroovePointCloudDialog->show();
	m_pGroovePointCloudDialog->raise();
	m_pGroovePointCloudDialog->activateWindow();
}

void QtWidgetsApplication4::GrooveCameraTest(bool checked)
{
	if (checked)
	{
		QString setupIssue;
		if (!IsCurrentRobotSetupReady(true, false, &setupIssue))
		{
			QMessageBox::warning(this, "坡口相机测试", setupIssue);
			ui.GrooveCameraTestBtn->setChecked(false);
			RefreshRobotOperationAvailability();
			return;
		}
		QString cameraIP;
		const int unitIndex = CurrentRobotUnitIndex();
		if (!EnsureScanCameraRunningForUnit(unitIndex, cameraIP, true))
		{
			QMessageBox::warning(this, "坡口相机测试", "未读取到当前机器人扫描相机的 DeviceAddress。");
			ui.GrooveCameraTestBtn->setChecked(false);
			return;
		}

		const int cameraPort = m_scanCameraRuntimes.value(unitIndex, nullptr) != nullptr
			? m_scanCameraRuntimes.value(unitIndex)->cameraPort
			: 0;
		ui.GrooveCameraText->setPlainText(QString("正在预览 Robot%1 扫描相机：%2\nTCP端口：%3\n接收模式：%4")
			.arg(unitIndex)
			.arg(cameraIP)
			.arg(cameraPort > 0 ? cameraPort : 50006)
			.arg("TCP独立连接"));
		OpenGroovePointCloudDialog();
		if (m_pGroovePointCloudDialog != nullptr)
		{
			static_cast<GroovePointCloudDialog*>(m_pGroovePointCloudDialog)->ClearPreview("正在等待相机帧...");
		}
		m_grooveCameraDisplayTimer->start(33);
	}
	else
	{
		if (m_grooveCameraDisplayTimer != nullptr)
		{
			m_grooveCameraDisplayTimer->stop();
		}
		ui.GrooveCameraText->appendPlainText("已停止坡口相机预览。");
		if (m_pGroovePointCloudDialog != nullptr)
		{
			static_cast<GroovePointCloudDialog*>(m_pGroovePointCloudDialog)->ClearPreview("已停止坡口相机预览。");
		}
	}
}

void QtWidgetsApplication4::UpdateGrooveCameraData()
{
	const int unitIndex = CurrentRobotUnitIndex();
	CameraFrameCache* cache = ScanCameraCacheForUnit(unitIndex);
	if (cache == nullptr)
	{
		if (m_pGroovePointCloudDialog != nullptr)
		{
			static_cast<GroovePointCloudDialog*>(m_pGroovePointCloudDialog)->ClearPreview("当前机器人没有相机缓存。");
		}
		return;
	}

	QString cameraIP;
	int cameraPort = 0;
	const CameraRuntime* runtime = m_scanCameraRuntimes.value(unitIndex, nullptr);
	if (runtime != nullptr)
	{
		cameraIP = runtime->cameraIP;
		cameraPort = runtime->cameraPort;
	}
	if (cameraIP.trimmed().isEmpty())
	{
		LoadGrooveCameraEndpointForUnit(unitIndex, cameraIP, cameraPort);
	}
	if (cameraIP.trimmed().isEmpty())
	{
		cameraIP = "未配置";
	}
	if (cameraPort <= 0)
	{
		cameraPort = 50006;
	}

	QStringList diagnosticLines;
	diagnosticLines
		<< QString("当前机器人: Robot%1").arg(unitIndex)
		<< QString("接收模式: TCP独立连接")
		<< QString("当前相机IP: %1").arg(cameraIP)
		<< QString("TCP端口: %1").arg(cameraPort)
		<< QString("相机线程状态: %1").arg(runtime != nullptr && !runtime->cameraStatus.isEmpty() ? runtime->cameraStatus : "未收到线程状态")
		<< QString("TCP接收次数: %1").arg(runtime != nullptr ? runtime->datagramCount : 0)
		<< QString("丢弃/跳过次数: %1").arg(runtime != nullptr ? runtime->filteredDatagramCount : 0)
		<< QString("解码成功帧: %1").arg(runtime != nullptr ? runtime->decodedFrameCount : 0)
		<< QString("解码失败帧: %1").arg(runtime != nullptr ? runtime->decodeFailedCount : 0)
		<< QString("写入缓存帧: %1").arg(runtime != nullptr ? runtime->appendedFrameCount : 0)
		<< QString("当前缓存帧: %1").arg(cache->CachedCount());

	udpDataShow frame;
	udpDataShow latestFrame;
	bool hasFrame = false;
	if (cache->Latest(frame))
	{
		latestFrame = frame;
		hasFrame = true;
	}
	if (!hasFrame)
	{
		diagnosticLines << "状态: 暂未从当前机器人专属缓存取到相机帧。";
		ui.GrooveCameraText->setPlainText(diagnosticLines.join('\n'));
		if (m_pGroovePointCloudDialog != nullptr)
		{
			static_cast<GroovePointCloudDialog*>(m_pGroovePointCloudDialog)->ClearPreview(diagnosticLines.join("  "));
		}
		return;
	}

	const QString text = QString(
		"%1\n"
		"点云数量: %2\n"
		"拟合点数量: %3\n"
		"目标点: X=%4  Y=%5  Z=%6\n"
		"相机帧时间戳: %7\n"
		"相机计算帧率: %8 fps\n"
		"点云X预览: %9\n"
		"点云Y预览: %10\n"
		"拟合X预览: %11\n"
		"拟合Y预览: %12\n"
		"错误信息: %13")
		.arg(diagnosticLines.join('\n'))
		.arg(latestFrame.XData.size())
		.arg(latestFrame.fitLineX.size())
		.arg(latestFrame.targetPoint.x, 0, 'f', 3)
		.arg(latestFrame.targetPoint.y, 0, 'f', 3)
		.arg(latestFrame.targetPoint.z, 0, 'f', 3)
		.arg(latestFrame.timestamp)
		.arg(latestFrame.mFps, 0, 'f', 2)
		.arg(FormatVectorPreview(latestFrame.XData))
		.arg(FormatVectorPreview(latestFrame.YData))
		.arg(FormatVectorPreview(latestFrame.fitLineX))
		.arg(FormatVectorPreview(latestFrame.fitLineY))
		.arg(latestFrame.errorMessage.isEmpty() ? "无" : latestFrame.errorMessage);
	ui.GrooveCameraText->setPlainText(text);
	if (m_pGroovePointCloudDialog != nullptr)
	{
		const QString statusText = QString("Robot%1  %2:%3")
			.arg(unitIndex)
			.arg(cameraIP)
			.arg(cameraPort);
		static_cast<GroovePointCloudDialog*>(m_pGroovePointCloudDialog)->SetFrame(
			latestFrame,
			statusText);
	}
}


void QtWidgetsApplication4::RobotRunTest()
{
	if (m_pContralUnit == nullptr || m_pContralUnit->m_vtContralUnitInfo.empty())
	{
		QMessageBox::warning(this, "测试程序", "未找到可用的控制单元。");
		return;
	}

	const T_CONTRAL_UNIT* currentUnit = CurrentContralUnit();
	if (currentUnit == nullptr)
	{
		QMessageBox::warning(this, "测试程序", "未找到当前选择的控制单元。");
		return;
	}

	RobotDriverAdaptor* pRobotDriverAdaptor = static_cast<RobotDriverAdaptor*>(currentUnit->pUnitDriver);
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
		const QString jobRunnerPath = FindProjectFilePath("SDK/FANUC/FanucJobRunner.kl");
		const QString loadJogBufferPath = FindProjectFilePath("SDK/FANUC/LOADJOGBUF.kl");
		const QString startAllPath = FindProjectFilePath("SDK/FANUC/STARTALL.tp");
		const QString joglPath = FindProjectFilePath("SDK/FANUC/FANUC_JOGL.ls");
		const QString jogjPath = FindProjectFilePath("SDK/FANUC/FANUC_JOGJ.ls");
		if (serviceLibPath.isEmpty() || residentServicePath.isEmpty() || monitorServicePath.isEmpty() || jobRunnerPath.isEmpty() || loadJogBufferPath.isEmpty() || startAllPath.isEmpty() || joglPath.isEmpty() || jogjPath.isEmpty())
		{
			QMessageBox::warning(this, "FANUC测试程序", "未找到测试程序文件：FanucServiceLib.kl / FanucResidentService.kl / FanucMonitorService.kl / FanucJobRunner.kl / LOADJOGBUF.kl / STARTALL.tp / FANUC_JOGL.ls / FANUC_JOGJ.ls");
			return;
		}

		const QByteArray serviceLibPathBytes = serviceLibPath.toLocal8Bit();
		const int libRet = pFanucDriver->UploadKlFile(serviceLibPathBytes.constData());
		if (libRet != 0)
		{
			QMessageBox::warning(this, "FANUC测试程序",
				DecodeRobotMessageText(GetStr("服务库发送失败，返回码=%d\n文件=%s", libRet, serviceLibPathBytes.constData())));
			return;
		}

		const QByteArray residentServicePathBytes = residentServicePath.toLocal8Bit();
		const int residentRet = pFanucDriver->UploadKlFile(residentServicePathBytes.constData());
		if (residentRet != 0)
		{
			QMessageBox::warning(this, "FANUC测试程序",
				DecodeRobotMessageText(GetStr("常驻服务发送失败，返回码=%d\n文件=%s", residentRet, residentServicePathBytes.constData())));
			return;
		}

		const QByteArray monitorServicePathBytes = monitorServicePath.toLocal8Bit();
		const int monitorRet = pFanucDriver->UploadKlFile(monitorServicePathBytes.constData());
		if (monitorRet != 0)
		{
			QMessageBox::warning(this, "FANUC测试程序",
				DecodeRobotMessageText(GetStr("监控服务发送失败，返回码=%d\n文件=%s", monitorRet, monitorServicePathBytes.constData())));
			return;
		}

		const QByteArray jobRunnerPathBytes = jobRunnerPath.toLocal8Bit();
		const int jobRunnerRet = pFanucDriver->UploadKlFile(jobRunnerPathBytes.constData());
		if (jobRunnerRet != 0)
		{
			QMessageBox::warning(this, "FANUC测试程序",
				DecodeRobotMessageText(GetStr("通用任务运行器发送失败，返回码=%d\n文件=%s", jobRunnerRet, jobRunnerPathBytes.constData())));
			return;
		}

		const QByteArray loadJogBufferPathBytes = loadJogBufferPath.toLocal8Bit();
		const int loadJogBufferRet = pFanucDriver->UploadKlFile(loadJogBufferPathBytes.constData());
		if (loadJogBufferRet != 0)
		{
			QMessageBox::warning(this, "FANUC测试程序",
				DecodeRobotMessageText(GetStr("点动缓冲加载程序发送失败，返回码=%d\n文件=%s", loadJogBufferRet, loadJogBufferPathBytes.constData())));
			return;
		}

		const QByteArray startAllPathBytes = startAllPath.toLocal8Bit();
		const int startAllRet = pFanucDriver->UploadFile(startAllPathBytes.constData(), "/md/STARTALL.tp");
		if (startAllRet != 0)
		{
			QMessageBox::warning(this, "FANUC测试程序",
				DecodeRobotMessageText(GetStr("合并启动TP发送失败，返回码=%d\n文件=%s", startAllRet, startAllPathBytes.constData())));
			return;
		}

		const QByteArray joglPathBytes = joglPath.toLocal8Bit();
		const int joglRet = pFanucDriver->UploadLsFile(joglPathBytes.constData());
		if (joglRet != 0)
		{
			QMessageBox::warning(this, "FANUC测试程序",
				DecodeRobotMessageText(GetStr("直角点动TP发送失败，返回码=%d\n文件=%s", joglRet, joglPathBytes.constData())));
			return;
		}

		const QByteArray jogjPathBytes = jogjPath.toLocal8Bit();
		const int jogjRet = pFanucDriver->UploadLsFile(jogjPathBytes.constData());
		if (jogjRet != 0)
		{
			QMessageBox::warning(this, "FANUC测试程序",
				DecodeRobotMessageText(GetStr("关节点动TP发送失败，返回码=%d\n文件=%s", jogjRet, jogjPathBytes.constData())));
			return;
		}

		QMessageBox::information(this, "FANUC测试程序",
			DecodeRobotMessageText(GetStr("常驻服务、监控服务、通用任务运行器、点动缓冲程序和点动TP发送成功。\n\n现在请在示教器重新运行 STARTALL，确认服务已启动后再点击确定。\n\n文件：\n%s\n%s\n%s\n%s\n%s\n%s\n%s\n%s",
				serviceLibPathBytes.constData(),
				residentServicePathBytes.constData(),
				monitorServicePathBytes.constData(),
				jobRunnerPathBytes.constData(),
				loadJogBufferPathBytes.constData(),
				startAllPathBytes.constData(),
				joglPathBytes.constData(),
				jogjPathBytes.constData())));
		return;
	}

	T_ANGLE_PULSE tNowPulse = pRobotDriverAdaptor->GetCurrentPulse();
	T_ANGLE_PULSE testPulse = tNowPulse;
	if (testPulse.nSPulse == 0 && testPulse.nLPulse == 0 && testPulse.nUPulse == 0 &&
		testPulse.nRPulse == 0 && testPulse.nBPulse == 0 && testPulse.nTPulse == 0)
	{
		testPulse = T_ANGLE_PULSE(90000, 180000, 0, 0, 0, 0, 0, 0, 0);
	}

	T_ANGLE_PULSE bestResult;
	const bool ok = pRobotDriverAdaptor->RunKinematicsSelfTest(testPulse, T_ROBOT_COORS(), &bestResult);

	if (ok)
	{
		QMessageBox::information(
			this,
			"运动学自检",
			GetStr("FK -> IK -> FK 自检完成。\n回代脉冲: S=%ld L=%ld U=%ld R=%ld B=%ld T=%ld",
				bestResult.nSPulse, bestResult.nLPulse, bestResult.nUPulse,
				bestResult.nRPulse, bestResult.nBPulse, bestResult.nTPulse).c_str());
	}
	else
	{
		QMessageBox::warning(this, "运动学自检", "FK -> IK -> FK 自检失败，请查看日志。");
	}
}

void QtWidgetsApplication4::OpenWeldProcessDialog()
{
	if (!RequirePermission(kRoleEngineer, "工艺参数"))
	{
		return;
	}
	const T_CONTRAL_UNIT* currentUnit = CurrentContralUnit();
	if (currentUnit == nullptr)
	{
		QMessageBox::warning(this, "工艺参数", "未找到可用的控制单元。");
		return;
	}

	const int currentUnitIndex = CurrentRobotUnitIndex();
	QStackedWidget* targetStack = CurrentEmbeddedTargetStack();
	if (m_pWeldProcessPage != nullptr && m_nWeldProcessPageUnitIndex != currentUnitIndex)
	{
		delete m_pWeldProcessPage;
		m_pWeldProcessPage = nullptr;
	}
	if (m_pWeldProcessPage == nullptr)
	{
		m_pWeldProcessPage = new WeldProcessDialog(*currentUnit, targetStack);
		m_nWeldProcessPageUnitIndex = currentUnitIndex;
		PrepareEmbeddedPage(m_pWeldProcessPage, targetStack);
	}
	ShowCurrentEmbeddedPage(m_pWeldProcessPage);
}

void QtWidgetsApplication4::OpenFunctionTestDialog()
{
	if (!RequirePermission(kRoleEngineer, "功能测试"))
	{
		return;
	}
	const int currentUnitIndex = CurrentRobotUnitIndex();
	QStackedWidget* targetStack = CurrentEmbeddedTargetStack();
	if (m_pFunctionTestPage != nullptr && m_nFunctionTestPageUnitIndex != currentUnitIndex)
	{
		delete m_pFunctionTestPage;
		m_pFunctionTestPage = nullptr;
	}
	if (m_pFunctionTestPage == nullptr)
	{
		m_pFunctionTestPage = new FunctionTestDialog(m_pContralUnit, currentUnitIndex, ScanCameraCacheForUnit(currentUnitIndex), targetStack);
		m_nFunctionTestPageUnitIndex = currentUnitIndex;
		PrepareEmbeddedPage(m_pFunctionTestPage, targetStack);
	}
	ShowCurrentEmbeddedPage(m_pFunctionTestPage);
}

void QtWidgetsApplication4::OpenMeasureThenWeldDialog()
{
	const int currentUnitIndex = CurrentRobotUnitIndex();
	if (!IsRobotUnitDriverReady(currentUnitIndex))
	{
		QMessageBox::warning(this, "先测后焊", "当前机器人驱动不可用。");
		return;
	}
	QString setupIssue;
	if (!IsCurrentRobotSetupReady(true, true, &setupIssue))
	{
		QMessageBox::warning(this, "先测后焊", setupIssue);
		RefreshRobotOperationAvailability();
		return;
	}

	auto startCamera = [this, currentUnitIndex](QString& cameraIP) -> bool
		{
			if (!EnsureScanCameraRunningForUnit(currentUnitIndex, cameraIP, true))
			{
				return false;
			}
			if (m_grooveCameraDisplayTimer != nullptr && m_grooveCameraDisplayTimer->isActive())
			{
				m_grooveCameraDisplayTimer->stop();
			}
			if (ui.GrooveCameraTestBtn != nullptr && ui.GrooveCameraTestBtn->isChecked())
			{
				QSignalBlocker blocker(ui.GrooveCameraTestBtn);
				ui.GrooveCameraTestBtn->setChecked(false);
			}
			if (ui.GrooveCameraText != nullptr)
			{
				ui.GrooveCameraText->appendPlainText("先测后焊已接管相机帧，主界面预览已暂停。");
			}
			return true;
		};

	auto stopCamera = []()
		{
		};

	QPointer<MeasureThenWeldDialog> existingPage = m_measureThenWeldPages.value(currentUnitIndex);
	if (existingPage != nullptr)
	{
		existingPage->show();
		existingPage->raise();
		existingPage->activateWindow();
		m_pMeasureThenWeldPage = existingPage;
		m_nMeasureThenWeldPageUnitIndex = currentUnitIndex;
		return;
	}

	CameraFrameCache* cameraCache = ScanCameraCacheForUnit(currentUnitIndex);
	if (cameraCache == nullptr)
	{
		QString ignoredIP;
		EnsureScanCameraRunningForUnit(currentUnitIndex, ignoredIP, false);
		cameraCache = ScanCameraCacheForUnit(currentUnitIndex);
	}

	MeasureThenWeldDialog* page = new MeasureThenWeldDialog(
		m_pContralUnit,
		currentUnitIndex,
		startCamera,
		stopCamera,
		cameraCache,
		this);
	page->setWindowModality(Qt::NonModal);
	m_measureThenWeldPages.insert(currentUnitIndex, page);
	m_pMeasureThenWeldPage = page;
	m_nMeasureThenWeldPageUnitIndex = currentUnitIndex;
	connect(page, &QObject::destroyed, this, [this, currentUnitIndex, page]()
		{
			const auto it = m_measureThenWeldPages.find(currentUnitIndex);
			if (it != m_measureThenWeldPages.end() && it.value() == page)
			{
				m_measureThenWeldPages.erase(it);
			}
			if (m_pMeasureThenWeldPage == page)
			{
				m_pMeasureThenWeldPage = nullptr;
				m_nMeasureThenWeldPageUnitIndex = -1;
			}
		});
	connect(page, &MeasureThenWeldDialog::FlowStepChanged, this, [this, currentUnitIndex](const QString& text)
		{
			m_sMeasureThenWeldStatus = QString("Robot%1：%2").arg(currentUnitIndex).arg(text);
			if (ui.FanucMonitorText != nullptr && ui.FanucMonitorText->toPlainText().isEmpty())
			{
				ui.FanucMonitorText->setPlainText(m_sMeasureThenWeldStatus);
			}
		});
	ApplyDebugLogVisibility(page);
	page->show();
	page->raise();
	page->activateWindow();
}

void QtWidgetsApplication4::OpenPreciseMeasureEditDialog()
{
	if (!RequirePermission(kRoleEngineer, "测量焊接参数"))
	{
		return;
	}
	QStackedWidget* targetStack = CurrentEmbeddedTargetStack();
	if (m_pPreciseMeasureEditPage == nullptr)
	{
		m_pPreciseMeasureEditPage = new PreciseMeasureEditDialog(m_pContralUnit, targetStack);
		PrepareEmbeddedPage(m_pPreciseMeasureEditPage, targetStack);
	}
	ShowCurrentEmbeddedPage(m_pPreciseMeasureEditPage);
}

void QtWidgetsApplication4::OpenWeldSeamCompDialog()
{
	if (!RequirePermission(kRoleEngineer, "焊道补偿"))
	{
		return;
	}
	QStackedWidget* targetStack = CurrentEmbeddedTargetStack();
	if (m_pWeldSeamCompPage == nullptr)
	{
		m_pWeldSeamCompPage = new WeldSeamCompDialog(m_pContralUnit, targetStack);
		PrepareEmbeddedPage(m_pWeldSeamCompPage, targetStack);
	}
	ShowCurrentEmbeddedPage(m_pWeldSeamCompPage);
}

void QtWidgetsApplication4::OpenCameraParamDialog()
{
	if (!RequirePermission(kRoleEngineer, "相机参数/手眼标定"))
	{
		return;
	}

	const int currentUnitIndex = CurrentRobotUnitIndex();
	auto startCamera = [this, currentUnitIndex](QString& cameraIP) -> bool
		{
			return EnsureScanCameraRunningForUnit(currentUnitIndex, cameraIP, true);
		};

	auto stopCamera = []()
		{
		};

	QString robotName = "RobotA";
	if (const T_CONTRAL_UNIT* currentUnit = CurrentContralUnit())
	{
		const QString selectedRobotName = DecodeConfigText(currentUnit->sUnitName).trimmed();
		if (!selectedRobotName.isEmpty())
		{
			robotName = selectedRobotName;
		}
	}

	if (m_pCameraParamPage != nullptr && m_sCameraParamPageRobotName != robotName)
	{
		if (m_pMainStack != nullptr)
		{
			m_pMainStack->removeWidget(m_pCameraParamPage);
		}
		if (m_pManagementStack != nullptr)
		{
			m_pManagementStack->removeWidget(m_pCameraParamPage);
		}
		m_pCameraParamPage->deleteLater();
		m_pCameraParamPage = nullptr;
		m_sCameraParamPageRobotName.clear();
	}

	QStackedWidget* targetStack = CurrentEmbeddedTargetStack();
	if (m_pCameraParamPage == nullptr)
	{
		m_pCameraParamPage = new CameraParamDialog(
			m_pContralUnit,
			robotName,
			startCamera,
			stopCamera,
			ScanCameraCacheForUnit(currentUnitIndex),
			[this]() { RefreshRobotOperationAvailability(); },
			targetStack);
		m_sCameraParamPageRobotName = robotName;
		PrepareEmbeddedPage(m_pCameraParamPage, targetStack);
	}
	ShowCurrentEmbeddedPage(m_pCameraParamPage);
}

void QtWidgetsApplication4::FanucConnectTest()
{
	RobotDriverAdaptor* pRobotDriver = GetCurrentRobotDriver(this);
	if (pRobotDriver == nullptr)
	{
		return;
	}
	if (pRobotDriver->IsConnected())
	{
		RefreshDashboardConnectionState();
		QMessageBox::information(this, "机器人连接", "当前机器人已经连接。");
		return;
	}

	const bool ok = pRobotDriver->InitSocket(pRobotDriver->m_sSocketIP.c_str(), static_cast<u_short>(pRobotDriver->m_nSocketPort));
	int ftpRet = -1;
	if (ok)
	{
		QStringList connectSteps;
		pRobotDriver->StartStateMonitor(50);
		if (STEPRobotCtrl* pStepDriver = dynamic_cast<STEPRobotCtrl*>(pRobotDriver))
		{
			const bool alarmOk = pStepDriver->cleanAlarm();
			const bool modeOk = pStepDriver->SetSysMode(2);
			const bool servoOk = pStepDriver->ServoOn();
			connectSteps
				<< QString("STEP清除报警：%1").arg(alarmOk ? "成功" : "失败")
				<< QString("STEP切换自动模式：%1").arg(modeOk ? "成功" : "失败")
				<< QString("STEP上使能：%1").arg(servoOk ? "成功" : "失败");
			if (pRobotDriver->m_pRobotLog != nullptr)
			{
				pRobotDriver->m_pRobotLog->write(
					alarmOk && modeOk && servoOk ? LogColor::SUCCESS : LogColor::WARNING,
					"STEP连接初始化 | 清报警=%d 自动模式=%d 上使能=%d",
					alarmOk ? 1 : 0,
					modeOk ? 1 : 0,
					servoOk ? 1 : 0);
			}
		}
		ftpRet = pRobotDriver->InitFtp();
		const QString extraText = connectSteps.isEmpty()
			? QString()
			: QString("\n%1").arg(connectSteps.join('\n'));
		QMessageBox::information(
			this,
			"机器人连接",
			DecodeRobotMessageText(GetStr("机器人连接成功：%s:%d\nFTP初始化返回：%d",
				pRobotDriver->m_sSocketIP.c_str(),
				pRobotDriver->m_nSocketPort,
				ftpRet)) + extraText);
	}
	else
	{
		QMessageBox::warning(this, "机器人连接",
			DecodeRobotMessageText(GetStr("连接失败：%s:%d", pRobotDriver->m_sSocketIP.c_str(), pRobotDriver->m_nSocketPort)));
	}
	RefreshDashboardConnectionState();
}

void QtWidgetsApplication4::FanucDisconnectTest()
{
	RobotDriverAdaptor* pRobotDriver = GetCurrentRobotDriver(this);
	if (pRobotDriver == nullptr)
	{
		return;
	}

	bool ok = true;
	if (FANUCRobotCtrl* pFanucDriver = dynamic_cast<FANUCRobotCtrl*>(pRobotDriver))
	{
		ok = pFanucDriver->StopRobotServices();
		pFanucDriver->StopMonitor();
	}
	ok = pRobotDriver->CloseSocket() && ok;
	QMessageBox::information(this, "机器人断开", ok ? "已断开当前机器人连接。" : "机器人断开时返回失败，请检查日志。");
	RefreshDashboardConnectionState();
}

void QtWidgetsApplication4::RobotClearAlarmTest()
{
	RobotDriverAdaptor* pRobotDriver = GetCurrentRobotDriver(this);
	if (pRobotDriver == nullptr)
	{
		return;
	}
	if (!pRobotDriver->IsConnected())
	{
		QMessageBox::warning(this, "清除报警", "当前机器人未连接，请先连接机器人。");
		RefreshDashboardConnectionState();
		return;
	}
	const bool ok = pRobotDriver->cleanAlarm();
	const bool servoOk = ok ? pRobotDriver->ServoOn() : false;
	if (pRobotDriver->m_pRobotLog != nullptr)
	{
		pRobotDriver->m_pRobotLog->write(
			ok && servoOk ? LogColor::SUCCESS : LogColor::WARNING,
			"主页清除报警并上使能 | 清报警=%d 上使能=%d",
			ok ? 1 : 0,
			servoOk ? 1 : 0);
	}
	QMessageBox::information(
		this,
		"清除报警",
		ok
			? QString("机器人报警已清除。\n上使能：%1").arg(servoOk ? "成功" : "失败，请查看日志或示教器报警。")
			: "清除报警失败，请查看日志或示教器报警。");
}

void QtWidgetsApplication4::RobotSwitchStepMode()
{
	RobotDriverAdaptor* pRobotDriver = GetCurrentRobotDriver(this);
	if (pRobotDriver == nullptr)
	{
		return;
	}
	STEPRobotCtrl* pStepDriver = dynamic_cast<STEPRobotCtrl*>(pRobotDriver);
	if (pStepDriver == nullptr)
	{
		QMessageBox::information(this, "模式切换", "当前机器人不是新时达机器人，已隐藏/禁用模式切换。");
		RefreshDashboardConnectionState();
		return;
	}
	if (!pStepDriver->IsConnected())
	{
		QMessageBox::warning(this, "模式切换", "当前新时达机器人未连接，请先连接机器人。");
		RefreshDashboardConnectionState();
		return;
	}

	struct StepModeOption
	{
		const char* text;
		int mode;
	};
	const StepModeOption options[] = {
		{ "手动模式 (MANUAL=1)", MODEKEY::MANUAL },
		{ "自动模式 (AUTO=2)", MODEKEY::AUTO },
		{ "外部自动模式 (AUTO_EXT=3)", MODEKEY::AUTO_EXT },
		{ "开始/运行 (START=4)", MODEKEY::START },
		{ "停止 (STOP=23)", MODEKEY::STOP },
		{ "停止点动 (MSTOP=100)", MODEKEY::MSTOP }
	};
	QStringList modeItems;
	for (const StepModeOption& option : options)
	{
		modeItems << QString::fromUtf8(option.text);
	}

	bool ok = false;
	const QString selectedText = QInputDialog::getItem(
		this,
		"新时达模式切换",
		"选择要切换的模式：",
		modeItems,
		1,
		false,
		&ok);
	if (!ok || selectedText.isEmpty())
	{
		return;
	}

	int selectedMode = -1;
	for (int i = 0; i < modeItems.size(); ++i)
	{
		if (modeItems[i] == selectedText)
		{
			selectedMode = options[i].mode;
			break;
		}
	}
	if (selectedMode < 0)
	{
		QMessageBox::warning(this, "模式切换", "未识别选择的模式。");
		return;
	}

	if (selectedMode == MODEKEY::START)
	{
		const QMessageBox::StandardButton confirm = QMessageBox::question(
			this,
			"确认开始",
			"START 会启动当前已加载程序，确认要切换到开始/运行吗？",
			QMessageBox::Yes | QMessageBox::No,
			QMessageBox::No);
		if (confirm != QMessageBox::Yes)
		{
			return;
		}
	}

	const bool modeOk = pStepDriver->SetSysMode(selectedMode);
	if (pRobotDriver->m_pRobotLog != nullptr)
	{
		pRobotDriver->m_pRobotLog->write(
			modeOk ? LogColor::SUCCESS : LogColor::WARNING,
			"主页STEP模式切换 | mode=%d(%s) result=%d",
			selectedMode,
			GetModeText(selectedMode),
			modeOk ? 1 : 0);
	}
	if (modeOk)
	{
		QMessageBox::information(
			this,
			"模式切换",
			QString("新时达机器人已切换到：%1").arg(QString::fromUtf8(GetModeText(selectedMode))));
	}
	else
	{
		const QString errorText = DecodeRobotMessageText(pStepDriver->GetLastRobotError());
		QMessageBox::warning(
			this,
			"模式切换",
			errorText.isEmpty()
				? "新时达模式切换失败，请查看日志或示教器报警。"
				: QString("新时达模式切换失败：\n%1").arg(errorText));
	}
	RefreshDashboardConnectionState();
}

void QtWidgetsApplication4::ReadTool1ToGunTool()
{
	RobotDriverAdaptor* pRobotDriver = GetCurrentRobotDriver(this);
	if (pRobotDriver == nullptr)
	{
		return;
	}

	const T_CONTRAL_UNIT* unitInfo = CurrentContralUnit();
	if (unitInfo == nullptr)
	{
		QMessageBox::warning(this, "读取Tool1", "未找到当前机器人配置，无法写入枪工具参数。");
		return;
	}

	T_ROBOT_COORS tool1;
	if (!pRobotDriver->GetToolData(1, tool1))
	{
		const QString errorText = DecodeRobotMessageText(pRobotDriver->GetLastRobotError()).trimmed();
		QMessageBox::warning(
			this,
			"读取Tool1",
			errorText.isEmpty()
				? "读取 Tool1 工具失败，请确认机器人已连接并支持读取工具坐标。"
				: QString("读取 Tool1 工具失败：\n%1").arg(errorText));
		return;
	}

	const std::string iniPath = DATA_PATH + unitInfo->sUnitName + ROBOT_PARA_INI;
	const QString iniPathText = QDir::toNativeSeparators(DecodeRobotMessageText(iniPath));
	if (!QFileInfo::exists(iniPathText))
	{
		QMessageBox::warning(
			this,
			"读取Tool1",
			QString("机器人参数文件不存在，无法写入 GunTool：\n%1").arg(iniPathText));
		return;
	}

	COPini ini;
	ini.SetFileName(iniPath);
	ini.SetSectionName("Tool");
	const bool saveOk = ini.WriteString(
		"GunTool_d",
		"",
		tool1,
		T_ROBOT_COORS(1, 1, 1, 1, 1, 1, 0, 0, 0));
	if (!saveOk)
	{
		QMessageBox::warning(
			this,
			"读取Tool1",
			QString("Tool1 已读取，但写入 GunTool 失败：\n%1").arg(iniPathText));
		return;
	}

	pRobotDriver->m_tTools.tGunTool = tool1;
	if (pRobotDriver->m_pRobotLog != nullptr)
	{
		pRobotDriver->m_pRobotLog->write(
			LogColor::SUCCESS,
			"读取Tool1并写入GunTool | unit=%s X=%.6f Y=%.6f Z=%.6f RX=%.6f RY=%.6f RZ=%.6f file=%s",
			unitInfo->sUnitName.c_str(),
			tool1.dX,
			tool1.dY,
			tool1.dZ,
			tool1.dRX,
			tool1.dRY,
			tool1.dRZ,
			iniPath.c_str());
	}

	QMessageBox::information(
		this,
		"读取Tool1",
		QString("已读取 Tool1 并写入 GunTool。\n\n"
			"X=%1\nY=%2\nZ=%3\nRX=%4\nRY=%5\nRZ=%6\n\n%7")
		.arg(tool1.dX, 0, 'f', 6)
		.arg(tool1.dY, 0, 'f', 6)
		.arg(tool1.dZ, 0, 'f', 6)
		.arg(tool1.dRX, 0, 'f', 6)
		.arg(tool1.dRY, 0, 'f', 6)
		.arg(tool1.dRZ, 0, 'f', 6)
		.arg(iniPathText));
}

void QtWidgetsApplication4::FanucGetCurrentPosTest()
{
	RobotDriverAdaptor* pRobotDriver = GetCurrentRobotDriver(this);
	if (pRobotDriver == nullptr)
	{
		return;
	}

	const T_ROBOT_COORS pos = pRobotDriver->GetCurrentPos();
	QMessageBox::information(
		this,
		"读取当前位置",
		DecodeRobotMessageText(GetStr("X=%.3f\nY=%.3f\nZ=%.3f\nRX=%.3f\nRY=%.3f\nRZ=%.3f",
			pos.dX, pos.dY, pos.dZ, pos.dRX, pos.dRY, pos.dRZ)));
}

void QtWidgetsApplication4::FanucGetCurrentPulseTest()
{
	RobotDriverAdaptor* pRobotDriver = GetCurrentRobotDriver(this);
	if (pRobotDriver == nullptr)
	{
		return;
	}

	const T_ANGLE_PULSE pulse = pRobotDriver->GetCurrentPulse();
	QMessageBox::information(
		this,
		"读取关节脉冲",
		DecodeRobotMessageText(GetStr("S=%ld\nL=%ld\nU=%ld\nR=%ld\nB=%ld\nT=%ld\nEX1=%ld\nEX2=%ld\nEX3=%ld",
			pulse.nSPulse, pulse.nLPulse, pulse.nUPulse, pulse.nRPulse, pulse.nBPulse, pulse.nTPulse,
			pulse.lBXPulse, pulse.lBYPulse, pulse.lBZPulse)));
}

void QtWidgetsApplication4::FanucCheckDoneTest()
{
	RobotDriverAdaptor* pRobotDriver = GetCurrentRobotDriver(this);
	if (pRobotDriver == nullptr)
	{
		return;
	}

	const int done = pRobotDriver->CheckDone();
	QMessageBox::information(this, "检查运行完成", DecodeRobotMessageText(GetStr("CheckDone 返回值：%d", done)));
}

void QtWidgetsApplication4::FanucSetGetIntTest()
{
	RobotDriverAdaptor* pRobotDriver = GetCurrentRobotDriver(this);
	if (pRobotDriver == nullptr)
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

	if (!pRobotDriver->SetIntVar(index, value))
	{
		QMessageBox::warning(this, "写读INT寄存器", DecodeRobotMessageText(GetStr("写入 INT%d 失败。", index)));
		return;
	}

	const int readValue = pRobotDriver->GetIntVar(index);
	QMessageBox::information(this, "写读INT寄存器", DecodeRobotMessageText(GetStr("写入 INT%d=%d\n读取值=%d", index, value, readValue)));
}

void QtWidgetsApplication4::FanucSetTpSpeedTest()
{
	RobotDriverAdaptor* pRobotDriver = GetCurrentRobotDriver(this);
	if (pRobotDriver == nullptr)
	{
		return;
	}

	bool ok = false;
	const int speed = QInputDialog::getInt(this, "设置速度", "速度百分比：", 50, 1, 100, 1, &ok);
	if (!ok)
	{
		return;
	}

	const bool setOk = pRobotDriver->SetTpSpeed(speed);
	const std::string message = setOk ? GetStr("设置速度成功：%d", speed) : GetStr("设置速度失败：%d", speed);
	QMessageBox::information(this, "设置速度", DecodeRobotMessageText(message));
}

void QtWidgetsApplication4::FanucCallJobTest()
{
	RobotDriverAdaptor* pRobotDriver = GetCurrentRobotDriver(this);
	if (pRobotDriver == nullptr)
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
	const bool callOk = pRobotDriver->CallJob(jobNameBytes.constData());
	const std::string message = callOk ? GetStr("调用任务成功：%s", jobNameBytes.constData()) : GetStr("调用任务失败：%s", jobNameBytes.constData());
	QMessageBox::information(this, "调用任务", DecodeRobotMessageText(message));
}

void QtWidgetsApplication4::FanucUploadLsTest()
{
	RobotDriverAdaptor* pRobotDriver = GetCurrentRobotDriver(this);
	if (pRobotDriver == nullptr)
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
	FANUCRobotCtrl* pFanucDriver = dynamic_cast<FANUCRobotCtrl*>(pRobotDriver);
	if (pFanucDriver == nullptr)
	{
		QMessageBox::information(this, "发送LS程序", "LS 编译/上传是 FANUC 专用功能；STEP 请使用通用 FTP 上传或焊道下发流程。");
		return;
	}
	const int ret = pFanucDriver->UploadLsFile(lsPathBytes.constData());
	if (ret == 0)
	{
		QMessageBox::information(this, "发送LS程序", DecodeRobotMessageText(GetStr("LS程序发送成功：%s", lsPathBytes.constData())));
	}
	else
	{
		QMessageBox::warning(this, "发送LS程序", DecodeRobotMessageText(GetStr("LS程序发送失败，返回码=%d\n文件=%s", ret, lsPathBytes.constData())));
	}
}

void QtWidgetsApplication4::FanucMovlTest()
{
	RobotDriverAdaptor* pRobotDriver = GetCurrentRobotDriver(this);
	if (pRobotDriver == nullptr)
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

	std::thread([this, pRobotDriver, moveForward]()
		{
			T_ROBOT_COORS target = pRobotDriver->GetCurrentPos();
			target.dY += moveForward ? 100.0 : -100.0;

			const bool moveOk = pRobotDriver->MoveByJob(target, T_ROBOT_MOVE_SPEED(5.0, 0.0, 0.0), pRobotDriver->m_nExternalAxleType, "MOVL");
			const int done = moveOk ? pRobotDriver->CheckRobotDone(200) : -1;
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
	RobotDriverAdaptor* pRobotDriver = GetCurrentRobotDriver(this);
	if (pRobotDriver == nullptr)
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

	std::thread([this, pRobotDriver]()
		{
			T_ANGLE_PULSE target = pRobotDriver->GetCurrentPulse();
			const double j2PulseUnit = pRobotDriver->m_tAxisUnit.dLPulseUnit;
			const double j3PulseUnit = pRobotDriver->m_tAxisUnit.dUPulseUnit;
			const long j2DeltaPulse = j2PulseUnit == 0.0 ? 0 : static_cast<long>(std::lround(5.0 / j2PulseUnit));
			const long j3DeltaPulse = j3PulseUnit == 0.0 ? 0 : static_cast<long>(std::lround(5.0 / j3PulseUnit));
			target.nLPulse += j2DeltaPulse;
			target.nUPulse += j3DeltaPulse;

			const bool moveOk = pRobotDriver->MoveByJob(target, T_ROBOT_MOVE_SPEED(1.0, 0.0, 0.0), pRobotDriver->m_nExternalAxleType, "MOVJ");
			const int done = moveOk ? pRobotDriver->CheckRobotDone(200) : -1;
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
	RobotDriverAdaptor* pRobotDriver = GetCurrentRobotDriver(this);
	if (pRobotDriver == nullptr)
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

	std::thread([this, pRobotDriver]()
		{
			const T_ANGLE_PULSE zeroPulse = T_ANGLE_PULSE();
			const T_ROBOT_MOVE_SPEED speed(1.0, 0.0, 0.0);
			const bool moveOk = pRobotDriver->MoveByJob(zeroPulse, speed, pRobotDriver->m_nExternalAxleType, "MOVJ");
			const int done = moveOk ? pRobotDriver->CheckRobotDone(200) : -1;
			const T_ROBOT_COORS pos = pRobotDriver->GetCurrentPos();
			const T_ANGLE_PULSE pulse = pRobotDriver->GetCurrentPulse();

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
	if (!RequirePermission(kRoleEngineer, "点动控制"))
	{
		return;
	}
	RobotDriverAdaptor* pRobotDriver = GetCurrentRobotDriver(this);
	if (pRobotDriver == nullptr)
	{
		return;
	}

	const int currentUnitIndex = CurrentRobotUnitIndex();
	QPointer<RobotJogDialog> existingPage = m_robotJogPages.value(currentUnitIndex);
	if (existingPage != nullptr)
	{
		existingPage->show();
		existingPage->raise();
		existingPage->activateWindow();
		m_pRobotJogPage = existingPage;
		m_nRobotJogPageUnitIndex = currentUnitIndex;
		return;
	}

	RobotJogDialog* page = new RobotJogDialog(pRobotDriver, this);
	page->setWindowModality(Qt::NonModal);
	m_robotJogPages.insert(currentUnitIndex, page);
	m_pRobotJogPage = page;
	m_nRobotJogPageUnitIndex = currentUnitIndex;
	connect(page, &QObject::destroyed, this, [this, currentUnitIndex, page]()
		{
			const auto it = m_robotJogPages.find(currentUnitIndex);
			if (it != m_robotJogPages.end() && it.value() == page)
			{
				m_robotJogPages.erase(it);
			}
			if (m_pRobotJogPage == page)
			{
				m_pRobotJogPage = nullptr;
				m_nRobotJogPageUnitIndex = -1;
			}
		});
	ApplyDebugLogVisibility(page);
	page->show();
	page->raise();
	page->activateWindow();
}
