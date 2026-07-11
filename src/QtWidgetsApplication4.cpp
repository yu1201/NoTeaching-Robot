#include "QtWidgetsApplication4.h"
#include "AppPaths.h"
#include "CliHelp.h"
#include <QMessageBox>  // 弹窗头文件，测试用
#include "CameraFrameCache.h"
#include "BrandingConfig.h"
#include "ConfigDatabase.h"
#include "CredentialSecurity.h"
#include "FTPClient.h"
#include "FANUCRobotDriver.h"
#include "CameraBasicParamDialog.h"
#include "CameraParamDialog.h"
#include "FunctionTestDialog.h"
#include "HandEyeCalibrationDialog.h"
#include "LaserWeldFilterDialog.h"
#include "WorkpieceMeshViewerDialog.h"
#include "ModelAlignmentDialog.h"
#include "VirtualWeldTestDialog.h"
#include "MeasureThenWeldDialog.h"
#include "MeasureThenWeldRuntimeConfig.h"
#include "MeasureThenWeldService.h"
#include "OnlineServicesConfig.h"
#include "OnlineServicesDialog.h"
#include "OPini.h"
#include "PointCloudProcessingConfig.h"
#include "ScanDataUploader.h"
#include "PreciseMeasureEditDialog.h"
#include "RobotCalculation.h"
#include "RobotDataHelper.h"
#include "RobotJogDialog.h"
#include "RobotMessage.h"
#include "RobotMotionTimeoutPolicy.h"
#include "RobotOperationLease.h"
#include "SKJCameraControlClient.h"
#include "STEPRobotDriver.h"
#include "TouchKeyboardManager.h"
#include "WindowStyleHelper.h"
#include "WeldPoseAverageUpdater.h"
#include "WeldProcessDialog.h"
#include "WeldSeamCompDialog.h"
#include "ResultArchiveDialog.h"
#include "PointCloud3DView.h"
#include "../portable/LaserFramePoint3DFilter/LaserFramePoint3DFilter.h"
#include "groove/clientudpformsensorworker.h"
#include "groove/scancameraskjworker.h"
#include "groove/framebuffer.h"
#include <QApplication>
#include <QByteArray>
#include <QCloseEvent>
#include <QContextMenuEvent>
#include <QCoreApplication>
#include <QEventLoop>
#include <QComboBox>
#include <QDebug>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDir>
#include <QDirIterator>
#include <QDoubleValidator>
#include <QDropEvent>
#include <QElapsedTimer>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QSaveFile>
#include <QFormLayout>
#include <QGuiApplication>
#include <QAbstractItemView>
#include <QAbstractButton>
#include <QInputDialog>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHostAddress>
#include <QKeyEvent>
#include <QLabel>
#include <QLayout>
#include <QLinearGradient>
#include <QLineEdit>
#include <QLineF>
#include <QEasingCurve>
#include <QIntValidator>
#include <QAction>
#include <QCheckBox>
#include <QClipboard>
#include <QFrame>
#include <QFontMetrics>
#include <QMetaObject>
#include <QProgressBar>
#include <QProgressDialog>
#include <QMenu>
#include <QMenuBar>
#include <QMouseEvent>
#include <QTabletEvent>
#include <QPainter>
#include <QPropertyAnimation>
#include <QPushButton>
#include <QResizeEvent>
#include <QScrollArea>
#include <QScrollBar>
#include <QSignalBlocker>
#include <QSizePolicy>
#include <QSlider>
#include <QSpinBox>
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
#include <QSqlDatabase>
#include <QSqlError>
#include <QSqlQuery>
#include <QTabBar>
#include <QThread>
#include <QTimer>
#include <QTextCursor>
#include <QTextDocument>
#include <QTextStream>
#include <QTime>
#include <QTouchEvent>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QVBoxLayout>
#include <QVariant>
#include <QWheelEvent>
#include <QStringConverter>
#include <QStringList>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <thread>
#include <atomic>
#include <chrono>
#include <utility>
#ifdef Q_OS_WIN
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#endif

namespace
{
	// 复用已抽取到 include/PointCloud3DView.h 的 3D 点云视图（PointCloud3DView/PointCloudVec3/Layer/LoadPointCloudFile3D）。
	using namespace pcview;

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
		return text.toUtf8().toStdString();
	}

	void ConfigureUtf8TextStream(QTextStream& stream)
	{
		stream.setEncoding(QStringConverter::Utf8);
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
			setMinimumWidth(228);
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

	class DashboardToolButton final : public QPushButton
	{
	public:
		DashboardToolButton(const QString& id, const QString& text, QWidget* parent = nullptr)
			: QPushButton(text, parent)
			, m_id(id)
		{
		}

		const QString& toolId() const
		{
			return m_id;
		}

	private:
		QString m_id;
	};

	class DashboardToolPanel final : public QWidget
	{
	public:
		explicit DashboardToolPanel(QWidget* parent = nullptr)
			: QWidget(parent)
		{
			setObjectName("DashboardToolPanel");
			setContextMenuPolicy(Qt::DefaultContextMenu);
			setMinimumHeight(220);
			setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

			m_dragDelayTimer = new QTimer(this);
			m_dragDelayTimer->setSingleShot(true);
			connect(m_dragDelayTimer, &QTimer::timeout, this, [this]()
				{
					if (m_pressedButton != nullptr && !m_dragging)
					{
						BeginDrag(m_pressedButton, m_lastGlobalPos);
					}
				});
		}

		void RegisterTool(DashboardToolButton* button)
		{
			if (button == nullptr || button->toolId().trimmed().isEmpty())
			{
				return;
			}
			button->setParent(this);
			button->installEventFilter(this);
			button->setCursor(Qt::PointingHandCursor);
			button->setProperty("dashboardToolListed", false);
			button->setProperty("dashboardToolRuntimeVisible", true);
			button->hide();

			ToolEntry entry;
			entry.id = button->toolId();
			entry.text = button->text();
			entry.button = button;
			m_tools.insert(entry.id, entry);
			button->hide();
		}

		void SetDefaultOrder(const QStringList& ids)
		{
			m_defaultOrder.clear();
			for (const QString& id : ids)
			{
				if (m_tools.contains(id) && !m_defaultOrder.contains(id))
				{
					m_defaultOrder.push_back(id);
				}
			}
			m_candidateOrder = m_defaultOrder;
			LoadOrder();
			RebuildLayout();
		}

		void SetCandidateOrder(const QStringList& ids)
		{
			m_candidateOrder = m_defaultOrder;
			for (const QString& id : ids)
			{
				if (m_tools.contains(id) && !m_candidateOrder.contains(id))
				{
					m_candidateOrder.push_back(id);
				}
			}
		}

		void SetEditingEnabled(bool enabled, const QString& reason = QString())
		{
			m_editingEnabled = enabled;
			m_editingDisabledReason = reason;
			if (!m_editingEnabled)
			{
				CancelDrag();
			}
		}

		void SetCurrentRobotScope(const QString& scope)
		{
			m_currentRobotScope = scope.trimmed().toLower();
		}

		void RefreshToolStates()
		{
			if (m_dragActive)
			{
				m_refreshPending = true;
				return;
			}
			LayoutTools(false);
		}

	protected:
		bool eventFilter(QObject* watched, QEvent* event) override
		{
			DashboardToolButton* button = dynamic_cast<DashboardToolButton*>(watched);
			if (button == nullptr)
			{
				return QWidget::eventFilter(watched, event);
			}

			if (event->type() == QEvent::MouseButtonPress)
			{
				QMouseEvent* mouseEvent = static_cast<QMouseEvent*>(event);
				if (RobotOperationLease::AnyActive())
				{
					return true;
				}
				if (mouseEvent->button() == Qt::RightButton)
				{
					ShowToolContextMenu(button->toolId(), mouseEvent->globalPosition().toPoint());
					return true;
				}
				if (mouseEvent->button() == Qt::LeftButton && button->isEnabled())
				{
					if (m_editingEnabled)
					{
						m_pressedButton = button;
						m_pressGlobalPos = mouseEvent->globalPosition().toPoint();
						m_lastGlobalPos = m_pressGlobalPos;
						m_dragOffset = button->mapFromGlobal(m_pressGlobalPos);
						m_dragDelayTimer->start(kDragDelayMs);
					}
				}
			}
			else if (event->type() == QEvent::MouseMove)
			{
				QMouseEvent* mouseEvent = static_cast<QMouseEvent*>(event);
				m_lastGlobalPos = mouseEvent->globalPosition().toPoint();
				if (m_dragging)
				{
					if (!m_editingEnabled)
					{
						CancelDrag();
						return true;
					}
					MoveDraggedButton(m_lastGlobalPos);
					ReorderDraggedTool(TargetVisibleIndex(mapFromGlobal(m_lastGlobalPos)));
					return true;
				}
				if (m_editingEnabled
					&& m_pressedButton == button
					&& (m_lastGlobalPos - m_pressGlobalPos).manhattanLength() >= QApplication::startDragDistance())
				{
					m_dragDelayTimer->stop();
					BeginDrag(button, m_lastGlobalPos);
					return true;
				}
			}
			else if (event->type() == QEvent::MouseButtonRelease)
			{
				QMouseEvent* mouseEvent = static_cast<QMouseEvent*>(event);
				m_dragDelayTimer->stop();
				if (m_dragging)
				{
					FinishDrag(mouseEvent->globalPosition().toPoint());
					return true;
				}
				m_pressedButton = nullptr;
			}

			return QWidget::eventFilter(watched, event);
		}

		void contextMenuEvent(QContextMenuEvent* event) override
		{
			if (event == nullptr)
			{
				QWidget::contextMenuEvent(event);
				return;
			}
			if (RobotOperationLease::AnyActive())
			{
				event->accept();
				return;
			}
			if (QWidget* child = childAt(event->pos()))
			{
				if (DashboardToolButton* button = dynamic_cast<DashboardToolButton*>(child))
				{
					ShowToolContextMenu(button->toolId(), event->globalPos());
					event->accept();
					return;
				}
			}
			ShowBlankContextMenu(event->globalPos());
			event->accept();
		}

		void resizeEvent(QResizeEvent* event) override
		{
			QWidget::resizeEvent(event);
			LayoutTools(false);
		}

	private:
		static constexpr int kColumnCount = 2;
		static constexpr int kToolHeight = 46;
		static constexpr int kSpacing = 10;
		static constexpr int kAnimationMs = 130;
		static constexpr int kDragDelayMs = 220;

		struct ToolEntry
		{
			QString id;
			QString text;
			DashboardToolButton* button = nullptr;
		};

		void LoadOrder()
		{
			QString savedOrderText;
			const bool hasSavedOrder = ConfigDatabase::ReadScopedSetting(
				QStringLiteral("global"),
				QString(),
				QStringLiteral("DashboardTools"),
				QStringLiteral("Order"),
				&savedOrderText);
			const QStringList savedOrder = savedOrderText.split('\n', Qt::SkipEmptyParts);
			m_order = hasSavedOrder ? FilterKnownIds(savedOrder) : m_defaultOrder;
		}

		void SaveOrder() const
		{
			ConfigDatabase::WriteScopedSetting(
				QStringLiteral("global"),
				QString(),
				QStringLiteral("DashboardTools"),
				QStringLiteral("Order"),
				m_order.join('\n'),
				QStringLiteral("list"));
		}

		QStringList FilterKnownIds(const QStringList& ids) const
		{
			QStringList filtered;
			for (const QString& id : ids)
			{
				if (m_tools.contains(id) && !filtered.contains(id))
				{
					filtered.push_back(id);
				}
			}
			return filtered;
		}

		bool IsRuntimeVisible(const QString& id) const
		{
			const ToolEntry entry = m_tools.value(id);
			if (entry.button == nullptr)
			{
				return false;
			}
			const QVariant runtimeVisible = entry.button->property("dashboardToolRuntimeVisible");
			return !runtimeVisible.isValid() || runtimeVisible.toBool();
		}

		bool IsCompatibleWithCurrentRobot(const QString& id) const
		{
			const ToolEntry entry = m_tools.value(id);
			if (entry.button == nullptr)
			{
				return false;
			}
			const QString requiredScope = entry.button->property("dashboardToolRequiredRobotScope").toString().trimmed().toLower();
			if (requiredScope.isEmpty() || requiredScope == "all")
			{
				return true;
			}
			return !m_currentRobotScope.isEmpty() && requiredScope == m_currentRobotScope;
		}

		QStringList MissingToolIds() const
		{
			QStringList missing;
			for (const QString& id : m_candidateOrder)
			{
				if (!m_order.contains(id) && IsRuntimeVisible(id) && IsCompatibleWithCurrentRobot(id))
				{
					missing.push_back(id);
				}
			}
			return missing;
		}

		QStringList VisibleToolOrder() const
		{
			QStringList visibleOrder;
			const QStringList filteredOrder = FilterKnownIds(m_order);
			for (const QString& id : filteredOrder)
			{
				if (IsRuntimeVisible(id))
				{
					visibleOrder.push_back(id);
				}
			}
			return visibleOrder;
		}

		void RebuildLayout()
		{
			LayoutTools(false);
		}

		void ShowToolContextMenu(const QString& id, const QPoint& globalPos)
		{
			QMenu menu(this);
			if (!m_editingEnabled)
			{
				AddEditingDisabledAction(menu);
				menu.exec(globalPos);
				return;
			}
			QAction* deleteAction = menu.addAction("从常用工具删除");
			menu.addSeparator();
			QMenu* addMenu = menu.addMenu("添加模块");
			AddMissingToolsToMenu(*addMenu);
			menu.addSeparator();
			QAction* restoreAction = menu.addAction("恢复默认工具");

			QAction* selectedAction = menu.exec(globalPos);
			if (selectedAction == deleteAction)
			{
				RemoveTool(id);
			}
			else if (selectedAction == restoreAction)
			{
				RestoreDefaults();
			}
		}

		void ShowBlankContextMenu(const QPoint& globalPos)
		{
			QMenu menu(this);
			if (!m_editingEnabled)
			{
				AddEditingDisabledAction(menu);
				menu.exec(globalPos);
				return;
			}
			QMenu* addMenu = menu.addMenu("添加模块");
			AddMissingToolsToMenu(*addMenu);
			menu.addSeparator();
			QAction* restoreAction = menu.addAction("恢复默认工具");

			QAction* selectedAction = menu.exec(globalPos);
			if (selectedAction == restoreAction)
			{
				RestoreDefaults();
			}
		}

		void AddEditingDisabledAction(QMenu& menu)
		{
			const QString text = m_editingDisabledReason.isEmpty()
				? "当前没有可用控制单元，不能修改模块。"
				: m_editingDisabledReason;
			QAction* disabledAction = menu.addAction(text);
			disabledAction->setEnabled(false);
		}

		void AddMissingToolsToMenu(QMenu& menu)
		{
			const QStringList missingIds = MissingToolIds();
			if (missingIds.isEmpty())
			{
				QAction* emptyAction = menu.addAction("没有可添加工具");
				emptyAction->setEnabled(false);
				return;
			}
			for (const QString& id : missingIds)
			{
				const ToolEntry entry = m_tools.value(id);
				QAction* action = menu.addAction(QString("添加%1").arg(entry.text));
				connect(action, &QAction::triggered, this, [this, id]()
					{
						AddTool(id);
					});
			}
		}

		void AddTool(const QString& id)
		{
			if (!m_editingEnabled || !m_tools.contains(id) || m_order.contains(id) || !IsCompatibleWithCurrentRobot(id))
			{
				return;
			}
			m_order.push_back(id);
			SaveOrder();
			LayoutTools(true);
		}

		void RemoveTool(const QString& id)
		{
			if (!m_editingEnabled)
			{
				return;
			}
			m_order.removeAll(id);
			SaveOrder();
			LayoutTools(true);
		}

		void RestoreDefaults()
		{
			if (!m_editingEnabled)
			{
				return;
			}
			m_order = m_defaultOrder;
			SaveOrder();
			LayoutTools(true);
		}

		void BeginDrag(DashboardToolButton* button, const QPoint& globalPos)
		{
			if (!m_editingEnabled || button == nullptr || !button->isEnabled() || !IsRuntimeVisible(button->toolId()))
			{
				return;
			}
			m_dragActive = true;
			m_dragging = true;
			m_dragButton = button;
			m_dragId = button->toolId();
			m_dragOffset = button->mapFromGlobal(globalPos);
			m_dragStartOrder = m_order;
			button->setDown(false);
			button->setCursor(Qt::ClosedHandCursor);
			button->raise();
			StopButtonAnimation(button);
			MoveDraggedButton(globalPos);
		}

		void CancelDrag()
		{
			if (m_dragDelayTimer != nullptr)
			{
				m_dragDelayTimer->stop();
			}
			DashboardToolButton* button = m_dragButton;
			m_dragging = false;
			m_dragActive = false;
			m_pressedButton = nullptr;
			m_dragButton = nullptr;
			m_dragId.clear();
			if (button != nullptr)
			{
				button->setCursor(Qt::PointingHandCursor);
			}
			LayoutTools(false);
		}

		void FinishDrag(const QPoint& globalPos)
		{
			MoveDraggedButton(globalPos);
			DashboardToolButton* button = m_dragButton;
			m_dragging = false;
			m_dragActive = false;
			m_dragButton = nullptr;
			m_pressedButton = nullptr;
			m_dragId.clear();
			if (button != nullptr)
			{
				button->setCursor(Qt::PointingHandCursor);
			}
			SaveOrder();
			LayoutTools(true);
			if (m_refreshPending)
			{
				m_refreshPending = false;
				LayoutTools(false);
			}
		}

		void MoveDraggedButton(const QPoint& globalPos)
		{
			if (m_dragButton == nullptr)
			{
				return;
			}
			m_dragButton->move(mapFromGlobal(globalPos) - m_dragOffset);
			m_dragButton->raise();
		}

		int ToolWidth() const
		{
			const QRect area = contentsRect();
			return qMax(118, (area.width() - kSpacing) / kColumnCount);
		}

		QRect ToolRectForVisibleIndex(int index) const
		{
			const QRect area = contentsRect();
			const int width = ToolWidth();
			const int row = index / kColumnCount;
			const int column = index % kColumnCount;
			return QRect(
				area.left() + column * (width + kSpacing),
				area.top() + row * (kToolHeight + kSpacing),
				width,
				kToolHeight);
		}

		int TargetVisibleIndex(const QPoint& panelPos) const
		{
			const int visibleCount = VisibleToolOrder().size();
			if (visibleCount <= 0)
			{
				return -1;
			}
			const QRect area = contentsRect();
			const int width = ToolWidth();
			const int x = qMax(0, panelPos.x() - area.left());
			const int y = qMax(0, panelPos.y() - area.top());
			const int column = qBound(0, x / qMax(1, width + kSpacing), kColumnCount - 1);
			const int row = y / qMax(1, kToolHeight + kSpacing);
			return qBound(0, row * kColumnCount + column, visibleCount - 1);
		}

		void ReorderDraggedTool(int targetIndex)
		{
			if (targetIndex < 0 || m_dragId.isEmpty())
			{
				return;
			}
			QStringList visibleOrder = VisibleToolOrder();
			const int currentIndex = visibleOrder.indexOf(m_dragId);
			if (currentIndex < 0 || currentIndex == targetIndex)
			{
				return;
			}
			visibleOrder.move(currentIndex, targetIndex);
			ApplyVisibleOrder(visibleOrder);
			LayoutTools(true);
		}

		void ApplyVisibleOrder(const QStringList& visibleOrder)
		{
			QStringList merged;
			int visibleIndex = 0;
			const QStringList previousOrder = FilterKnownIds(m_order);
			for (const QString& id : previousOrder)
			{
				if (IsRuntimeVisible(id))
				{
					if (visibleIndex < visibleOrder.size() && !merged.contains(visibleOrder[visibleIndex]))
					{
						merged.push_back(visibleOrder[visibleIndex]);
					}
					++visibleIndex;
				}
				else if (m_tools.contains(id) && !merged.contains(id))
				{
					merged.push_back(id);
				}
			}
			while (visibleIndex < visibleOrder.size())
			{
				if (!merged.contains(visibleOrder[visibleIndex]))
				{
					merged.push_back(visibleOrder[visibleIndex]);
				}
				++visibleIndex;
			}
			m_order = merged;
		}

		void LayoutTools(bool animate)
		{
			const QStringList filteredOrder = FilterKnownIds(m_order);
			const QStringList visibleOrder = VisibleToolOrder();
			int visibleIndex = 0;
			for (const QString& id : filteredOrder)
			{
				ToolEntry& entry = m_tools[id];
				if (entry.button == nullptr)
				{
					continue;
				}
				const bool visible = IsRuntimeVisible(id);
				entry.button->setProperty("dashboardToolListed", visible);
				entry.button->setVisible(visible);
				if (!visible)
				{
					StopButtonAnimation(entry.button);
					continue;
				}
				const QRect targetRect = ToolRectForVisibleIndex(visibleIndex);
				++visibleIndex;
				if (m_dragging && entry.button == m_dragButton)
				{
					continue;
				}
				MoveButtonTo(entry.button, targetRect, animate);
			}
			m_order = filteredOrder;
			const int rows = qMax(1, (visibleOrder.size() + kColumnCount - 1) / kColumnCount);
			setMinimumHeight(qMax(220, rows * kToolHeight + qMax(0, rows - 1) * kSpacing));
		}

		void MoveButtonTo(DashboardToolButton* button, const QRect& targetRect, bool animate)
		{
			if (button == nullptr)
			{
				return;
			}
			if (!animate || !button->isVisible() || button->geometry().isNull())
			{
				StopButtonAnimation(button);
				button->setGeometry(targetRect);
				return;
			}
			if (button->geometry() == targetRect)
			{
				return;
			}
			StopButtonAnimation(button);
			QPropertyAnimation* animation = new QPropertyAnimation(button, "geometry", button);
			animation->setObjectName("dashboardToolGeometryAnimation");
			animation->setDuration(kAnimationMs);
			animation->setEasingCurve(QEasingCurve::OutCubic);
			animation->setStartValue(button->geometry());
			animation->setEndValue(targetRect);
			animation->start(QAbstractAnimation::DeleteWhenStopped);
		}

		void StopButtonAnimation(DashboardToolButton* button)
		{
			if (button == nullptr)
			{
				return;
			}
			const QList<QPropertyAnimation*> animations = button->findChildren<QPropertyAnimation*>("dashboardToolGeometryAnimation");
			for (QPropertyAnimation* animation : animations)
			{
				animation->stop();
				animation->deleteLater();
			}
		}

		QHash<QString, ToolEntry> m_tools;
		QStringList m_defaultOrder;
		QStringList m_candidateOrder;
		QStringList m_order;
		QString m_currentRobotScope;
		QString m_editingDisabledReason;
		bool m_editingEnabled = false;
		bool m_dragActive = false;
		bool m_dragging = false;
		bool m_refreshPending = false;
		QTimer* m_dragDelayTimer = nullptr;
		DashboardToolButton* m_pressedButton = nullptr;
		DashboardToolButton* m_dragButton = nullptr;
		QString m_dragId;
		QStringList m_dragStartOrder;
		QPoint m_pressGlobalPos;
		QPoint m_lastGlobalPos;
		QPoint m_dragOffset;
	};

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
				*error = QString("打开机器人参数数据失败：%1").arg(path);
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

	class PageOpenTrace final
	{
	public:
		explicit PageOpenTrace(QString pageName)
			: name(std::move(pageName))
		{
			timer.start();
		}

		~PageOpenTrace()
		{
			qInfo().noquote() << QString("[PageOpen] %1 %2 ms").arg(name).arg(timer.elapsed());
		}

	private:
		QString name;
		QElapsedTimer timer;
	};

	void ShowMaximizedWithUnifiedChrome(QWidget* window)
	{
		if (window == nullptr)
		{
			return;
		}

		ApplyResponsivePageDefaults(window);
		window->showMaximized();
		RefreshUnifiedWindowTitleBar(window);
		QTimer::singleShot(0, window, [window]() {
			ApplyResponsivePageDefaults(window);
			RefreshUnifiedWindowTitleBar(window);
		});
		QTimer::singleShot(120, window, [window]() {
			ApplyResponsivePageDefaults(window);
			RefreshUnifiedWindowTitleBar(window);
		});
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

	QString FindInstalledResourcePath(const QString& relativePath)
	{
		return QDir::toNativeSeparators(AppPaths::FindResourcePath(relativePath));
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

	QString InferRobotNameFromResultPath(const QString& inputFilePath, const QString& fallbackRobotName = QStringLiteral("RobotA"))
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
		return fallbackRobotName;
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
		// 与真机流程共用同一参数名册（MeasureThenWeldService::BuildTrackFitParamsFromSettings），
		// 此前 CLI 手抄副本曾漂移漏掉 geometryStrategy 映射。CLI 不启用拐点补偿（保持默认 false）。
		return MeasureThenWeldService::BuildTrackFitParamsFromSettings(
			PointCloudProcessingConfig::Load(), sampleAxis);
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

	QString BuildCornerCompClassifiedOutputPath(const QString& classifiedOutputPath)
	{
		const QFileInfo info(classifiedOutputPath);
		QString baseName = info.completeBaseName();
		if (baseName.contains("_Classified_Geometry"))
		{
			baseName.replace("_Classified_Geometry", "_CornerComp_Classified_Geometry");
		}
		else if (baseName.contains("_Classified"))
		{
			baseName.replace("_Classified", "_CornerComp_Classified");
		}
		else
		{
			baseName += "_CornerComp_Classified";
		}
		const QString suffix = info.suffix().isEmpty() ? QStringLiteral("txt") : info.suffix();
		return info.dir().filePath(baseName + "." + suffix);
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

	QString AppConfigValue(const QString& group, const QString& key, const QString& defaultValue = QString())
	{
		QString value;
		return ConfigDatabase::ReadScopedSetting(QStringLiteral("global"), QString(), group, key, &value) ? value : defaultValue;
	}

	bool AppConfigBoolValue(const QString& group, const QString& key, bool defaultValue = false)
	{
		const QString value = AppConfigValue(group, key);
		if (value.isEmpty())
		{
			return defaultValue;
		}
		const QString normalized = value.trimmed().toLower();
		return normalized == "1" || normalized == "true" || normalized == "yes";
	}

	void WriteAppConfigValue(const QString& group, const QString& key, const QString& value)
	{
		ConfigDatabase::WriteScopedSetting(QStringLiteral("global"), QString(), group, key, value);
	}

	void WriteAppConfigValue(const QString& group, const QString& key, bool value)
	{
		ConfigDatabase::WriteScopedSetting(QStringLiteral("global"), QString(), group, key, value ? "1" : "0", QStringLiteral("bool"));
	}

	QStringList AppConfigListValue(const QString& group, const QString& key)
	{
		return AppConfigValue(group, key).split('\n', Qt::SkipEmptyParts);
	}

	void WriteAppConfigListValue(const QString& group, const QString& key, const QStringList& values)
	{
		ConfigDatabase::WriteScopedSetting(QStringLiteral("global"), QString(), group, key, values.join('\n'), QStringLiteral("list"));
	}

	QString AccountProfileModule()
	{
		return QStringLiteral("Profile");
	}

	QString AccountUserId(const QString& userName)
	{
		return userName.trimmed();
	}

	QStringList AccountUserNames()
	{
		return ConfigDatabase::ListScopedSettingIds(QStringLiteral("account"), AccountProfileModule());
	}

	QString LoginStateGroup()
	{
		return QStringLiteral("LoginState");
	}

	QString SavedPasswordsGroup()
	{
		return QStringLiteral("LoginState/SavedPasswords");
	}

	QString RememberedCredentialsGroup()
	{
		return QStringLiteral("LoginState/RememberedCredentials");
	}

	QString CameraReceiveModeGroup()
	{
		return QStringLiteral("Camera/ReceiveMode");
	}

	bool StoredBool(const QString& value, bool defaultValue = false)
	{
		if (value.trimmed().isEmpty())
		{
			return defaultValue;
		}
		const QString normalized = value.trimmed().toLower();
		return normalized == QStringLiteral("1")
			|| normalized == QStringLiteral("true")
			|| normalized == QStringLiteral("yes");
	}

	bool WriteAccountPasswordRecord(
		const QString& userName,
		const QString& password,
		const QString& expectedPasswordRecord,
		bool requireMustChangePassword,
		bool newMustChangePassword,
		QString& newPasswordRecord,
		QString& currentRole,
		QString& securityFingerprint,
		QString& error)
	{
		newPasswordRecord = CredentialSecurity::CreatePasswordRecord(password, &error);
		if (newPasswordRecord.isEmpty())
		{
			return false;
		}
		if (!ConfigDatabase::TryCompareAndSetAccountPassword(
				AccountUserId(userName),
				expectedPasswordRecord,
				newPasswordRecord,
				requireMustChangePassword,
				newMustChangePassword,
				&currentRole,
				&securityFingerprint))
		{
			error = QStringLiteral("账号已被删除或安全状态已变化，密码修改已拒绝，请重新登录。");
			return false;
		}
		error.clear();
		return true;
	}

	bool WriteNewAccountRecord(
		const QString& userName,
		const QString& password,
		const QString& role,
		bool mustChangePassword,
		QString& error,
		bool initializeAuthentication = false,
		const QString& administratorId = QString())
	{
		const QString record = CredentialSecurity::CreatePasswordRecord(password, &error);
		if (record.isEmpty())
		{
			return false;
		}
		const QString now = QDateTime::currentDateTime().toString(Qt::ISODate);
		QMap<QString, QString> values;
		values.insert(QStringLiteral("PasswordHash"), record);
		values.insert(QStringLiteral("Role"), role);
		values.insert(QStringLiteral("MustChangePassword"), mustChangePassword ? QStringLiteral("1") : QStringLiteral("0"));
		values.insert(QStringLiteral("CreatedAt"), now);
		values.insert(QStringLiteral("PasswordChangedAt"), now);
		const bool written = initializeAuthentication
			? ConfigDatabase::TryInitializeAuthenticationAccount(
				AccountUserId(userName), values)
			: ConfigDatabase::TryCreateAccount(
				AccountUserId(userName), values, administratorId);
		if (!written)
		{
			error = QStringLiteral("写入账号配置库失败。");
			return false;
		}
		error.clear();
		return true;
	}

	int AdminAccountCount()
	{
		int count = 0;
		for (const QString& userName : AccountUserNames())
		{
			QString role;
			if (ConfigDatabase::ReadScopedSetting(
					QStringLiteral("account"), AccountUserId(userName), AccountProfileModule(),
					QStringLiteral("Role"), &role)
				&& role == kRoleAdmin)
			{
				++count;
			}
		}
		return count;
	}

	void ClearRememberedCredential(const QString& userName)
	{
		const QString normalized = userName.trimmed();
		if (normalized.isEmpty())
		{
			return;
		}
		ConfigDatabase::RemoveScopedSetting(
			QStringLiteral("global"), QString(), RememberedCredentialsGroup(), normalized);
		ConfigDatabase::RemoveScopedSetting(
			QStringLiteral("global"), QString(), SavedPasswordsGroup(), normalized);
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
		explicit AccountManagementDialog(const QString& currentUserName, QWidget* parent = nullptr)
			: QDialog(parent)
			, m_currentUserName(currentUserName.trimmed())
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
			ConfigureResponsiveScrollArea(pageScrollArea);
			outerLayout->addWidget(pageScrollArea);

			QWidget* pageWidget = new QWidget(pageScrollArea);
			pageWidget->setMinimumWidth(760);
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
			m_createUserEdit->setMinimumWidth(260);
			m_createPassEdit->setMinimumWidth(260);
			m_createRoleCombo->setMinimumWidth(180);
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
			m_accountTable->setColumnCount(4);
			m_accountTable->setHorizontalHeaderLabels(QStringList() << "账号" << "权限" << "创建时间" << "改密状态");
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
			m_editRoleCombo->setMinimumWidth(180);
			m_editPassEdit->setMinimumWidth(260);
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

			const QStringList users = AccountUserNames();
			m_accountTable->setRowCount(0);
			int row = 0;
			for (const QString& user : users)
			{
				QString role;
				QString createdAt;
				QString mustChangePassword;
				if (!ConfigDatabase::ReadScopedSetting(QStringLiteral("account"), AccountUserId(user), AccountProfileModule(), "Role", &role))
				{
					role = kRoleOperator;
				}
				ConfigDatabase::ReadScopedSetting(QStringLiteral("account"), AccountUserId(user), AccountProfileModule(), "CreatedAt", &createdAt);
				ConfigDatabase::ReadScopedSetting(QStringLiteral("account"), AccountUserId(user), AccountProfileModule(), "MustChangePassword", &mustChangePassword);
				m_accountTable->insertRow(row);
				m_accountTable->setItem(row, 0, new QTableWidgetItem(user));
				m_accountTable->setItem(row, 1, new QTableWidgetItem(DisplayRoleNameForAccount(role)));
				m_accountTable->setItem(row, 2, new QTableWidgetItem(createdAt));
				m_accountTable->setItem(row, 3, new QTableWidgetItem(
					StoredBool(mustChangePassword) ? QStringLiteral("首次登录须改密") : QStringLiteral("已设置")));
				++row;
			}
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
			QString role;
			if (!ConfigDatabase::ReadScopedSetting(QStringLiteral("account"), AccountUserId(user), AccountProfileModule(), "Role", &role))
			{
				role = kRoleOperator;
			}
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

			if (AccountUserNames().contains(userName))
			{
				QMessageBox::warning(this, "新增账号", "账号已存在。");
				return false;
			}
			QString error;
			if (!WriteNewAccountRecord(
					userName, password, role, true, error, false, m_currentUserName))
			{
				QMessageBox::warning(this, "新增账号", error);
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

			if (!AccountUserNames().contains(userName))
			{
				QMessageBox::warning(this, "保存修改", "账号不存在。");
				return false;
			}
			QString currentRole;
			if (!ConfigDatabase::ReadScopedSetting(
					QStringLiteral("account"), AccountUserId(userName), AccountProfileModule(), QStringLiteral("Role"), &currentRole))
			{
				QMessageBox::warning(this, "保存修改", "无法读取账号当前权限，已拒绝修改。");
				return false;
			}
			const QString newRole = m_editRoleCombo->currentData().toString();
			if (currentRole == kRoleAdmin && newRole != kRoleAdmin && AdminAccountCount() <= 1)
			{
				QMessageBox::warning(this, "保存修改", "不能降级最后一个管理员账号。");
				return false;
			}
			const QString newPassword = m_editPassEdit->text();
			if (userName.compare(m_currentUserName, Qt::CaseInsensitive) == 0
				&& (currentRole != newRole || !newPassword.isEmpty()))
			{
				QMessageBox::warning(this, "保存修改", "不能在当前会话中修改自己的权限或重置自己的密码，请由另一个管理员操作。");
				return false;
			}

			QString newPasswordRecord;
			if (!newPassword.isEmpty())
			{
				if (newPassword.size() < 8)
				{
					QMessageBox::warning(this, "保存修改", "新密码至少需要 8 个字符。");
					return false;
				}
				QString error;
				newPasswordRecord = CredentialSecurity::CreatePasswordRecord(newPassword, &error);
				if (newPasswordRecord.isEmpty())
				{
					QMessageBox::warning(this, "保存修改", error);
					return false;
				}
			}
			if (!ConfigDatabase::TryUpdateAccountByAdministrator(
					m_currentUserName, AccountUserId(userName), newRole, newPasswordRecord))
			{
				QMessageBox::warning(this, "保存修改", "账号状态或管理员权限已变化，安全事务已拒绝本次修改。请刷新后重试。");
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
			if (userName.compare(m_currentUserName, Qt::CaseInsensitive) == 0)
			{
				QMessageBox::warning(this, "删除账号", "不能删除当前登录账号，请由另一个管理员操作。");
				return false;
			}
			QString role;
			if (!ConfigDatabase::ReadScopedSetting(
					QStringLiteral("account"), AccountUserId(userName), AccountProfileModule(), QStringLiteral("Role"), &role))
			{
				QMessageBox::warning(this, "删除账号", "无法读取账号当前权限，已拒绝删除。");
				return false;
			}
			if (role == kRoleAdmin && AdminAccountCount() <= 1)
			{
				QMessageBox::warning(this, "删除账号", "不能删除最后一个管理员账号。");
				return false;
			}

			if (!ConfigDatabase::TryDeleteAccountByAdministrator(
					m_currentUserName, AccountUserId(userName)))
			{
				QMessageBox::warning(this, "删除账号", "账号状态或管理员权限已变化，安全事务已拒绝删除。请刷新后重试。");
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
		QString m_currentUserName;
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
			ConfigureResponsiveScrollArea(pageScrollArea);
			outerLayout->addWidget(pageScrollArea);

			QWidget* pageWidget = new QWidget(pageScrollArea);
			pageWidget->setMinimumWidth(860);
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
				"这里维护配置库中的控制单元列表，以及每个机器人单元的 IP、端口和 FTP 参数。保存后建议重新加载控制单元，正在运行流程时不要重载。",
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
			QPushButton* deleteBtn = new QPushButton("删除单元", listGroup);
			QPushButton* refreshBtn = new QPushButton("刷新", listGroup);
			for (QPushButton* button : { newBtn, copyBtn, deleteBtn, refreshBtn })
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

			m_socketIpEdit->setMinimumWidth(228);
			m_ftpIpEdit->setMinimumWidth(228);
			for (QLineEdit* edit : { m_unitNameEdit, m_chineseNameEdit, m_customNameEdit, m_ftpUserEdit, m_ftpPasswordEdit, m_stepProjectEdit })
			{
				edit->setMinimumWidth(260);
			}
			for (QLineEdit* edit : { m_unitTypeEdit, m_socketPortEdit, m_monitorPortEdit, m_ftpPortEdit })
			{
				edit->setMinimumWidth(120);
				edit->setAlignment(Qt::AlignRight);
				MarkNumericEditGlobal(edit);
			}
			m_unitTypeEdit->setValidator(new QIntValidator(0, 9999, m_unitTypeEdit));
			m_socketPortEdit->setValidator(new QIntValidator(0, 65535, m_socketPortEdit));
			m_monitorPortEdit->setValidator(new QIntValidator(0, 65535, m_monitorPortEdit));
			m_ftpPortEdit->setValidator(new QIntValidator(0, 65535, m_ftpPortEdit));
			m_robotTypeCombo->setMinimumWidth(160);
			m_workpieceTypeCombo->setMinimumWidth(160);

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
			connect(deleteBtn, &QPushButton::clicked, this, [this]() { DeleteSelectedUnit(); });
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
			return text.toUtf8().toStdString();
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
			credential.password.clear();

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

		bool DeleteSelectedUnit()
		{
			if (m_unitTable == nullptr || m_unitTable->currentRow() < 0)
			{
				QMessageBox::warning(this, "删除控制单元", "请先选择一个控制单元。");
				return false;
			}

			QTableWidgetItem* indexItem = m_unitTable->item(m_unitTable->currentRow(), 0);
			const int unitRow = indexItem != nullptr ? indexItem->data(Qt::UserRole).toInt() : -1;
			if (unitRow < 0 || unitRow >= m_units.size())
			{
				QMessageBox::warning(this, "删除控制单元", "当前选择的控制单元无效，请刷新后重试。");
				return false;
			}

			const UnitConfig unit = m_units.at(unitRow);
			const QString displayName = unit.chineseName.isEmpty() ? unit.unitName : unit.chineseName;
			const QString confirmText = QString(
				"确定删除控制单元 %1（%2）吗？\n\n"
				"这会从控制单元列表移除，并删除配置库里 Data/%2 下的机器人、相机、手眼和工艺配置记录。\n"
				"删除后如需让主界面立即生效，请点击“只重载”。")
				.arg(displayName, unit.unitName);
			if (QMessageBox::question(this, "删除控制单元", confirmText,
				QMessageBox::Yes | QMessageBox::No, QMessageBox::No) != QMessageBox::Yes)
			{
				return false;
			}

			QList<UnitConfig> nextUnits = m_units;
			nextUnits.removeAt(unitRow);
			NormalizeRuntimeUnitNumbers(nextUnits);

			const QString unitConfigPrefix = QString("Data/%1").arg(unit.unitName.trimmed());
			if (!ConfigDatabase::RemoveConfigPathPrefix(unitConfigPrefix))
			{
				QMessageBox::warning(this, "删除控制单元",
					QString("删除 %1 的配置记录失败，请检查配置库：%2")
					.arg(unit.unitName, ConfigDatabase::DatabasePath()));
				return false;
			}

			QString error;
			if (!WriteControlInfo(nextUnits, error))
			{
				QMessageBox::warning(this, "删除控制单元", error);
				return false;
			}

			m_units = nextUnits;
			RefreshTable();
			if (!m_units.isEmpty())
			{
				m_unitTable->selectRow(std::min(unitRow, static_cast<int>(m_units.size()) - 1));
			}
			else
			{
				m_editingRow = -1;
				FillEditor(UnitConfig(), false);
			}
			AppendLog(QString("已删除控制单元 %1，并清理 Data/%1 下的配置记录。").arg(unit.unitName));
			return true;
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
				edit->setMinimumWidth(300);
			}
			robotTypeCombo->setMinimumWidth(180);
			workpieceCombo->setMinimumWidth(180);
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
    socketIpEdit->setMinimumWidth(228);
			for (QLineEdit* edit : { socketPortEdit, monitorPortEdit, unitTypeEdit })
			{
				edit->setMinimumWidth(140);
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
    ftpIpEdit->setMinimumWidth(228);
			for (QLineEdit* edit : { ftpUserEdit, ftpPasswordEdit, stepProjectEdit })
			{
				edit->setMinimumWidth(300);
			}
			ftpPortEdit->setMinimumWidth(140);
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
			const QString controlInfoPath = ControlInfoPath();
			const QStringList unitSections = {
				"UnitName",
				"ChineseName",
				"ContralType",
				"UnitType"
			};
			for (const QString& sectionName : unitSections)
			{
				if (!ConfigDatabase::RemoveIniSection(controlInfoPath, sectionName))
				{
					error = QString("清理控制单元旧列表失败：%1 [%2]")
						.arg(controlInfoPath, sectionName);
					return false;
				}
			}

			COPini ini;
			if (!ini.SetFileName(false, ToIniBytes(controlInfoPath)))
			{
				error = "打开控制单元配置失败：" + controlInfoPath;
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
				error = "写入控制单元配置失败：" + controlInfoPath;
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
			if (preferredName.compare(targetUnitName, Qt::CaseInsensitive) != 0
				&& ConfigDatabase::HasIniFile(ToIniBytes(preferredPath)))
			{
				return preferredPath;
			}
			for (const UnitConfig& unit : m_units)
			{
				if (unit.unitName.compare(targetUnitName, Qt::CaseInsensitive) != 0
					&& unit.robotType == robotType
					&& ConfigDatabase::HasIniFile(ToIniBytes(RobotParaPath(unit.unitName))))
				{
					return RobotParaPath(unit.unitName);
				}
			}
			return QString();
		}

		bool EnsureRobotParaFile(const UnitConfig& unit, bool isNew, QString& error) const
		{
			const QString targetPath = RobotParaPath(unit.unitName);
			if (!ConfigDatabase::IsAvailable())
			{
				error = QString("配置库不存在或结构无效，请先运行迁移工具：%1").arg(ConfigDatabase::DatabasePath());
				return false;
			}
			if (ConfigDatabase::HasIniFile(ToIniBytes(targetPath)))
			{
				return true;
			}
			const QString templatePath = TemplateRobotParaPath(unit.robotType, unit.unitName);
			if (!templatePath.isEmpty() && ConfigDatabase::CopyIniFile(templatePath, targetPath, false))
			{
				return true;
			}
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
			if (!ConfigDatabase::IsAvailable())
			{
				error = QString("配置库不存在或结构无效，请先运行迁移工具：%1").arg(ConfigDatabase::DatabasePath());
				return false;
			}

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
				const QString sourcePath = QDir(templateDirPath).filePath(fileName);
				const QString targetPath = RobotDataHelper::BuildProjectPath(QString("Data/%1/%2").arg(unit.unitName, fileName));
				const bool isTextFile = fileName.endsWith(".txt", Qt::CaseInsensitive);
				const bool sourceExists = isTextFile
					? ConfigDatabase::HasTextFile(ToIniBytes(sourcePath))
					: ConfigDatabase::HasIniFile(ToIniBytes(sourcePath));
				if (!sourceExists)
				{
					continue;
				}
				const bool targetExists = isTextFile
					? ConfigDatabase::HasTextFile(ToIniBytes(targetPath))
					: ConfigDatabase::HasIniFile(ToIniBytes(targetPath));
				if (targetExists)
				{
					continue;
				}
				const bool copied = isTextFile
					? ConfigDatabase::CopyTextFile(sourcePath, targetPath, false)
					: ConfigDatabase::CopyIniFile(sourcePath, targetPath, false);
				if (!copied)
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
				error = "打开机器人参数数据失败：" + path;
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
				error = "写入机器人参数数据失败：" + path;
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

	class FtpJobManagementDialog final : public QDialog
	{
	public:
		explicit FtpJobManagementDialog(
			ContralUnit* pContralUnit,
			int currentUnitIndex,
			std::function<bool()> liveSessionGuard,
			QWidget* parent = nullptr)
			: QDialog(parent)
			, m_pContralUnit(pContralUnit)
			, m_initialUnitIndex(currentUnitIndex)
			, m_liveSessionGuard(std::move(liveSessionGuard))
		{
			setWindowTitle("FTP Job 文件管理");
			ApplyUnifiedWindowChrome(this);
			ResizeWindowForAvailableGeometry(this, QSize(1180, 760), 0.90, 0.82);
			setStyleSheet(
				"QDialog { background: #111820; color: #ECF3F4; }"
				"QGroupBox { border: 1px solid #2E4656; border-radius: 12px; margin-top: 16px; padding-top: 12px; }"
				"QGroupBox::title { subcontrol-origin: margin; left: 14px; padding: 0 6px; color: #9ED8DB; }"
				"QPushButton { background: #233645; color: #F5FAFA; border: 1px solid #3C6173; border-radius: 10px; padding: 9px 12px; }"
				"QPushButton:hover { background: #2D5465; border-color: #72D4DD; }"
				"QPushButton:pressed { background: #18303B; }"
				"QPushButton:disabled { background: #18242D; color: #607580; border-color: #263844; }"
				"QLineEdit, QPlainTextEdit, QTableWidget { background: #081018; color: #ECF3F4; border: 1px solid #2C4653; border-radius: 8px; padding: 6px 8px; }"
				"QTableWidget::item:selected { background: #8BE8F2; color: #071018; }"
				"QTableWidget::item:selected:!active { background: #6FCFDC; color: #071018; }"
				"QLineEdit[readOnly=\"true\"] { color: #91A7AE; background: #0A121A; }"
				"QComboBox { background: #000000; color: #ECF3F4; border: 1px solid #2C4653; border-radius: 0px; padding: 6px 34px 6px 8px; }"
				"QComboBox::drop-down { border-left: 1px solid #2C4653; border-radius: 0px; width: 28px; background: #000000; }"
				"QComboBox::down-arrow { image: url(:/QtWidgetsApplication4/icons/chevron-down.svg); width: 12px; height: 8px; }"
				"QComboBox QAbstractItemView { background: #000000; color: #ECF3F4; selection-background-color: #2D5465; border: 1px solid #2C4653; border-radius: 0px; outline: 0px; }"
				"QHeaderView::section { background: #13202A; color: #BFE8EC; border: 0px; padding: 6px; }");

			QVBoxLayout* rootLayout = new QVBoxLayout(this);
			rootLayout->setContentsMargins(18, 16, 18, 18);
			rootLayout->setSpacing(12);

			QLabel* titleLabel = new QLabel("FTP Job 文件管理", this);
			titleLabel->setStyleSheet("font-size: 24px; font-weight: bold; color: #F7FCFC;");
			titleLabel->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
			titleLabel->setMaximumHeight(44);
			rootLayout->addWidget(titleLabel);

			QGroupBox* connectionGroup = new QGroupBox("控制单元与 FTP 参数", this);
			QGridLayout* connectionLayout = new QGridLayout(connectionGroup);
			connectionLayout->setHorizontalSpacing(10);
			connectionLayout->setVerticalSpacing(8);

			m_unitCombo = new QComboBox(connectionGroup);
			m_hostEdit = new QLineEdit(connectionGroup);
			m_portEdit = new QLineEdit(connectionGroup);
			m_userEdit = new QLineEdit(connectionGroup);
			m_passwordEdit = new QLineEdit(connectionGroup);
			m_remoteDirEdit = new QLineEdit(connectionGroup);
			m_localDirEdit = new QLineEdit(connectionGroup);
			m_passwordEdit->setEchoMode(QLineEdit::Password);
			m_portEdit->setValidator(new QIntValidator(1, 65535, m_portEdit));
			m_hostEdit->setMinimumWidth(160);
			m_remoteDirEdit->setMinimumWidth(260);
			m_localDirEdit->setMinimumWidth(300);

			m_chooseLocalDirBtn = new QPushButton("选择本地目录", connectionGroup);
			m_reloadUnitsBtn = new QPushButton("重读控制单元", connectionGroup);

			connectionLayout->addWidget(new QLabel("控制单元", connectionGroup), 0, 0);
			connectionLayout->addWidget(m_unitCombo, 0, 1);
			connectionLayout->addWidget(new QLabel("FTP IP", connectionGroup), 0, 2);
			connectionLayout->addWidget(m_hostEdit, 0, 3);
			connectionLayout->addWidget(new QLabel("端口", connectionGroup), 0, 4);
			connectionLayout->addWidget(m_portEdit, 0, 5);
			connectionLayout->addWidget(m_reloadUnitsBtn, 0, 6);
			connectionLayout->addWidget(new QLabel("用户名", connectionGroup), 1, 0);
			connectionLayout->addWidget(m_userEdit, 1, 1);
			connectionLayout->addWidget(new QLabel("密码", connectionGroup), 1, 2);
			connectionLayout->addWidget(m_passwordEdit, 1, 3);
			connectionLayout->addWidget(new QLabel("服务器目录", connectionGroup), 1, 4);
			connectionLayout->addWidget(m_remoteDirEdit, 1, 5, 1, 2);
			connectionLayout->addWidget(new QLabel("本地目录", connectionGroup), 2, 0);
			connectionLayout->addWidget(m_localDirEdit, 2, 1, 1, 5);
			connectionLayout->addWidget(m_chooseLocalDirBtn, 2, 6);
			connectionLayout->setColumnStretch(5, 1);
			rootLayout->addWidget(connectionGroup);

			QSplitter* splitter = new QSplitter(Qt::Horizontal, this);
			splitter->setChildrenCollapsible(false);
			rootLayout->addWidget(splitter, 1);

			QGroupBox* localGroup = new QGroupBox("本地 Job 文件", splitter);
			QVBoxLayout* localLayout = new QVBoxLayout(localGroup);
			QHBoxLayout* localButtons = new QHBoxLayout();
			m_refreshLocalBtn = new QPushButton("刷新本地", localGroup);
			m_pickUploadFilesBtn = new QPushButton("选择文件上传", localGroup);
			m_uploadSelectedBtn = new QPushButton("上传选中", localGroup);
			m_deleteLocalBtn = new QPushButton("删除本地选中", localGroup);
			for (QPushButton* button : { m_refreshLocalBtn, m_pickUploadFilesBtn, m_uploadSelectedBtn, m_deleteLocalBtn })
			{
				button->setFixedWidth(118);
				localButtons->addWidget(button);
			}
			localButtons->addStretch(1);
			localLayout->addLayout(localButtons);

			m_localTable = new QTableWidget(localGroup);
			m_localTable->setColumnCount(4);
			m_localTable->setHorizontalHeaderLabels(QStringList() << "文件名" << "大小" << "修改时间" << "路径");
			m_localTable->horizontalHeader()->setStretchLastSection(true);
			m_localTable->setSelectionBehavior(QAbstractItemView::SelectRows);
			m_localTable->setSelectionMode(QAbstractItemView::ExtendedSelection);
			m_localTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
			m_localTable->verticalHeader()->setVisible(false);
			localLayout->addWidget(m_localTable, 1);

			QGroupBox* remoteGroup = new QGroupBox("服务器 Job 文件", splitter);
			QVBoxLayout* remoteLayout = new QVBoxLayout(remoteGroup);
			QHBoxLayout* remoteButtons = new QHBoxLayout();
			m_refreshRemoteBtn = new QPushButton("刷新服务器", remoteGroup);
			m_remoteParentBtn = new QPushButton("服务器上一级", remoteGroup);
			m_downloadSelectedBtn = new QPushButton("下载选中", remoteGroup);
			m_deleteRemoteBtn = new QPushButton("删除服务器选中", remoteGroup);
			for (QPushButton* button : { m_refreshRemoteBtn, m_remoteParentBtn, m_downloadSelectedBtn, m_deleteRemoteBtn })
			{
				button->setFixedWidth(128);
				remoteButtons->addWidget(button);
			}
			remoteButtons->addStretch(1);
			remoteLayout->addLayout(remoteButtons);

			m_remoteTable = new QTableWidget(remoteGroup);
			m_remoteTable->setColumnCount(5);
			m_remoteTable->setHorizontalHeaderLabels(QStringList() << "名称" << "类型" << "大小" << "修改时间" << "路径");
			m_remoteTable->horizontalHeader()->setStretchLastSection(true);
			m_remoteTable->setSelectionBehavior(QAbstractItemView::SelectRows);
			m_remoteTable->setSelectionMode(QAbstractItemView::ExtendedSelection);
			m_remoteTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
			m_remoteTable->verticalHeader()->setVisible(false);
			remoteLayout->addWidget(m_remoteTable, 1);

			splitter->setStretchFactor(0, 1);
			splitter->setStretchFactor(1, 1);

			QHBoxLayout* bottomLayout = new QHBoxLayout();
			m_statusLabel = new QLabel("就绪", this);
			m_statusLabel->setStyleSheet("QLabel { color: #9ED8DB; }");
			m_closeBtn = new QPushButton("关闭", this);
			m_closeBtn->setFixedWidth(96);
			bottomLayout->addWidget(m_statusLabel, 1);
			bottomLayout->addWidget(m_closeBtn);
			rootLayout->addLayout(bottomLayout);

			m_logText = new QPlainTextEdit(this);
			m_logText->setReadOnly(true);
			m_logText->setMaximumHeight(110);
			rootLayout->addWidget(m_logText);

			connect(m_unitCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int index) { ApplyUnitSelection(index); });
			connect(m_reloadUnitsBtn, &QPushButton::clicked, this, [this]() { LoadUnits(true); });
			connect(m_chooseLocalDirBtn, &QPushButton::clicked, this, [this]() { ChooseLocalDir(); });
			connect(m_refreshLocalBtn, &QPushButton::clicked, this, [this]() { RefreshLocalFiles(); });
			connect(m_refreshRemoteBtn, &QPushButton::clicked, this, [this]() { RefreshRemoteFiles(); });
			connect(m_remoteParentBtn, &QPushButton::clicked, this, [this]() { GoRemoteParent(); });
			connect(m_pickUploadFilesBtn, &QPushButton::clicked, this, [this]() { PickFilesAndUpload(); });
			connect(m_uploadSelectedBtn, &QPushButton::clicked, this, [this]() { UploadSelectedLocalFiles(); });
			connect(m_downloadSelectedBtn, &QPushButton::clicked, this, [this]() { DownloadSelectedRemoteFiles(); });
			connect(m_deleteLocalBtn, &QPushButton::clicked, this, [this]() { DeleteSelectedLocalFiles(); });
			connect(m_deleteRemoteBtn, &QPushButton::clicked, this, [this]() { DeleteSelectedRemoteFiles(); });
			connect(m_closeBtn, &QPushButton::clicked, this, &QDialog::close);
			connect(m_remoteTable, &QTableWidget::cellDoubleClicked, this, [this](int row, int)
				{
					const bool isDirectory = m_remoteTable->item(row, 0) != nullptr
						&& m_remoteTable->item(row, 0)->data(Qt::UserRole + 1).toBool();
					if (!isDirectory)
					{
						return;
					}
					const QString remotePath = m_remoteTable->item(row, 0)->data(Qt::UserRole).toString();
					m_remoteDirEdit->setText(NormalizeRemotePath(remotePath));
					RefreshRemoteFiles();
				});

			LoadUnits(false);
		}

		bool IsBusy() const
		{
			return m_busy;
		}

	protected:
		void closeEvent(QCloseEvent* event) override
		{
			if (m_busy)
			{
				QMessageBox::information(this, "FTP Job 文件管理",
					"FTP 操作仍在执行，完成前不能关闭或销毁此页面。");
				event->ignore();
				return;
			}
			QDialog::closeEvent(event);
		}

	private:
		struct UnitConfig
		{
			int unitIndex = -1;
			QString unitName;
			QString chineseName;
			QString customName;
			int robotType = ROBOT_TYPE_FANUC;
			QString ftpIP;
			int ftpPort = 21;
			QString ftpUser;
			QString ftpPassword;
			QString stepProjectName;
		};

		struct FtpConnection
		{
			QString host;
			int port = 21;
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
			return text.toUtf8().toStdString();
		}

		static std::string ToLocalStd(const QString& text)
		{
			const QByteArray bytes = QDir::toNativeSeparators(text).toLocal8Bit();
			return std::string(bytes.constData(), static_cast<size_t>(bytes.size()));
		}

		static std::string ToFtpStd(const QString& text)
		{
			const QByteArray bytes = NormalizeRemotePath(text).toLocal8Bit();
			return std::string(bytes.constData(), static_cast<size_t>(bytes.size()));
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

		static QString NormalizeRemotePath(QString path)
		{
			path = path.trimmed();
			path.replace('\\', '/');
			if (path.isEmpty())
			{
				path = "/";
			}
			if (!path.startsWith('/'))
			{
				path.prepend('/');
			}
			while (path.contains("//"))
			{
				path.replace("//", "/");
			}
			while (path.size() > 1 && path.endsWith('/'))
			{
				path.chop(1);
			}
			return path;
		}

		static QString JoinRemotePath(const QString& remoteDir, const QString& name)
		{
			const QString dir = NormalizeRemotePath(remoteDir);
			const QString cleanName = QString(name).replace('\\', '/').section('/', -1, -1);
			if (dir == "/")
			{
				return "/" + cleanName;
			}
			return dir + "/" + cleanName;
		}

		static QString RemoteParentDir(const QString& remoteDir)
		{
			const QString path = NormalizeRemotePath(remoteDir);
			if (path == "/")
			{
				return "/";
			}
			const int slash = path.lastIndexOf('/');
			if (slash <= 0)
			{
				return "/";
			}
			return path.left(slash);
		}

		static QStringList JobNameFilters()
		{
			return QStringList()
				<< "*.srp" << "*.srd" << "*.sr"
				<< "*.ls" << "*.tp" << "*.kl" << "*.pc" << "*.var" << "*.vr" << "*.dt" << "*.job";
		}

		static bool IsJobFileName(const QString& fileName)
		{
			const QString suffix = QFileInfo(fileName).suffix().toLower();
			return QStringList({ "srp", "srd", "sr", "ls", "tp", "kl", "pc", "var", "vr", "dt", "job" }).contains(suffix);
		}

		static QString FormatBytes(quint64 bytes)
		{
			const double value = static_cast<double>(bytes);
			if (bytes >= 1024ULL * 1024ULL)
			{
				return QString::number(value / 1024.0 / 1024.0, 'f', 2) + " MB";
			}
			if (bytes >= 1024ULL)
			{
				return QString::number(value / 1024.0, 'f', 1) + " KB";
			}
			return QString::number(bytes) + " B";
		}

		void LoadUnits(bool keepSelection)
		{
			const QString previousUnitName = CurrentUnit().unitName;
			m_units.clear();
			m_unitCombo->clear();

			QString error;
			COPini ini;
			if (!ini.SetFileName(ToIniBytes(ControlInfoPath())))
			{
				error = "打开控制单元配置失败：" + ControlInfoPath();
			}
			else
			{
				ini.SetSectionName("UnitNum");
				const int unitCount = ReadIniInt(ini, "UnitNum", 0);
				for (int index = 0; index < unitCount; ++index)
				{
					UnitConfig unit;
					unit.unitIndex = index;
					const QString key = QString("Unit%1").arg(index);
					ini.SetSectionName("UnitName");
					unit.unitName = ReadIniString(ini, key);
					ini.SetSectionName("ChineseName");
					unit.chineseName = ReadIniString(ini, key);
					LoadRobotPara(unit);
					m_units.push_back(unit);
				}
			}

			for (int index = 0; index < m_units.size(); ++index)
			{
				const UnitConfig& unit = m_units.at(index);
				QString displayName = unit.chineseName.trimmed();
				if (displayName.isEmpty())
				{
					displayName = unit.customName.trimmed();
				}
				if (displayName.isEmpty())
				{
					displayName = unit.unitName.trimmed();
				}
				if (!unit.unitName.trimmed().isEmpty() && displayName.compare(unit.unitName, Qt::CaseInsensitive) != 0)
				{
					displayName += QString(" (%1)").arg(unit.unitName);
				}
				m_unitCombo->addItem(displayName, index);
			}

			if (!error.isEmpty())
			{
				QMessageBox::warning(this, "FTP Job 文件管理", error);
				AppendLog(error);
			}

			int selectedComboIndex = -1;
			if (keepSelection && !previousUnitName.isEmpty())
			{
				for (int comboIndex = 0; comboIndex < m_unitCombo->count(); ++comboIndex)
				{
					const int unitListIndex = m_unitCombo->itemData(comboIndex).toInt();
					if (unitListIndex >= 0
						&& unitListIndex < m_units.size()
						&& m_units.at(unitListIndex).unitName.compare(previousUnitName, Qt::CaseInsensitive) == 0)
					{
						selectedComboIndex = comboIndex;
						break;
					}
				}
			}
			if (selectedComboIndex < 0 && m_initialUnitIndex >= 0)
			{
				for (int comboIndex = 0; comboIndex < m_unitCombo->count(); ++comboIndex)
				{
					const int unitListIndex = m_unitCombo->itemData(comboIndex).toInt();
					if (unitListIndex >= 0 && unitListIndex < m_units.size() && m_units.at(unitListIndex).unitIndex == m_initialUnitIndex)
					{
						selectedComboIndex = comboIndex;
						break;
					}
				}
			}
			if (selectedComboIndex < 0 && m_unitCombo->count() > 0)
			{
				selectedComboIndex = 0;
			}

			if (selectedComboIndex >= 0)
			{
				m_unitCombo->setCurrentIndex(selectedComboIndex);
				ApplyUnitSelection(selectedComboIndex);
			}
			else
			{
				ApplyUnitSelection(-1);
			}
			AppendLog(QString("已读取控制单元：%1 个。").arg(m_units.size()));
		}

		void LoadRobotPara(UnitConfig& unit) const
		{
			if (unit.unitName.trimmed().isEmpty())
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
			unit.ftpIP = ReadIniString(robotIni, "FTPIP");
			unit.ftpPort = ReadIniInt(robotIni, "FTPPort", 21);
			unit.ftpUser = ReadIniString(robotIni, "FTPUser");
			unit.ftpPassword = ReadIniString(robotIni, "FTPPassWord");
			unit.stepProjectName = ReadIniString(robotIni, "StepProjectName");
			if (unit.stepProjectName.trimmed().isEmpty())
			{
				unit.stepProjectName = ReadIniString(robotIni, "ProjectName");
			}
		}

		UnitConfig CurrentUnit() const
		{
			if (m_unitCombo == nullptr)
			{
				return UnitConfig();
			}
			const int unitListIndex = m_unitCombo->currentData().toInt();
			if (unitListIndex < 0 || unitListIndex >= m_units.size())
			{
				return UnitConfig();
			}
			return m_units.at(unitListIndex);
		}

		QString DefaultRemoteDir(const UnitConfig& unit) const
		{
			if (unit.robotType == ROBOT_TYPE_STEP)
			{
				QString projectName = unit.stepProjectName.trimmed();
				if (projectName.isEmpty())
				{
					projectName = "PCRobot";
				}
				if (!projectName.endsWith(".sr", Qt::CaseInsensitive))
				{
					projectName += ".sr";
				}
				return NormalizeRemotePath("/UserPrograms/" + projectName);
			}
			return "/md";
		}

		QString DefaultLocalDir(const UnitConfig& unit) const
		{
			return RobotDataHelper::BuildProjectPath(unit.robotType == ROBOT_TYPE_STEP ? "Job/STEP" : "Job/FANUC");
		}

		void ApplyUnitSelection(int)
		{
			const UnitConfig unit = CurrentUnit();
			m_hostEdit->setText(unit.ftpIP);
			m_portEdit->setText(QString::number(unit.ftpPort > 0 ? unit.ftpPort : 21));
			m_userEdit->setText(unit.ftpUser);
			m_passwordEdit->setText(unit.ftpPassword);
			m_remoteDirEdit->setText(DefaultRemoteDir(unit));
			m_localDirEdit->setText(DefaultLocalDir(unit));
			RefreshLocalFiles();
			FillRemoteTable(std::vector<FtpRemoteFileInfo>());
			if (!unit.unitName.isEmpty())
			{
				AppendLog(QString("当前控制单元：%1，服务器目录：%2。").arg(unit.unitName, m_remoteDirEdit->text()));
			}
		}

		FtpConnection CurrentConnection(QString* error = nullptr) const
		{
			FtpConnection connection;
			connection.host = m_hostEdit->text().trimmed();
			connection.port = m_portEdit->text().toInt();
			connection.user = m_userEdit->text().trimmed();
			connection.password = m_passwordEdit->text();
			if (connection.host.isEmpty())
			{
				if (error != nullptr)
				{
					*error = "FTP IP 不能为空。";
				}
			}
			if (connection.port <= 0)
			{
				connection.port = 21;
			}
			if (connection.user.isEmpty())
			{
				connection.user = "anonymous";
			}
			return connection;
		}

		void RefreshLocalFiles()
		{
			const QString localDir = QDir::fromNativeSeparators(m_localDirEdit->text().trimmed());
			if (localDir.isEmpty())
			{
				m_localTable->setRowCount(0);
				return;
			}
			QDir dir(localDir);
			if (!dir.exists())
			{
				dir.mkpath(".");
			}

			const QFileInfoList files = dir.entryInfoList(JobNameFilters(), QDir::Files, QDir::Name | QDir::IgnoreCase);
			m_localTable->setSortingEnabled(false);
			m_localTable->setRowCount(files.size());
			for (int row = 0; row < files.size(); ++row)
			{
				const QFileInfo& info = files.at(row);
				QTableWidgetItem* nameItem = new QTableWidgetItem(info.fileName());
				nameItem->setData(Qt::UserRole, info.absoluteFilePath());
				m_localTable->setItem(row, 0, nameItem);
				m_localTable->setItem(row, 1, new QTableWidgetItem(FormatBytes(static_cast<quint64>(info.size()))));
				m_localTable->setItem(row, 2, new QTableWidgetItem(info.lastModified().toString("yyyy-MM-dd HH:mm:ss")));
				m_localTable->setItem(row, 3, new QTableWidgetItem(QDir::toNativeSeparators(info.absoluteFilePath())));
			}
			m_localTable->setSortingEnabled(true);
			m_localTable->resizeColumnsToContents();
			m_localTable->horizontalHeader()->setStretchLastSection(true);
			m_statusLabel->setText(QString("本地文件：%1 个").arg(files.size()));
		}

		void FillRemoteTable(const std::vector<FtpRemoteFileInfo>& entries)
		{
			m_remoteTable->setSortingEnabled(false);
			m_remoteTable->setRowCount(0);
			int row = 0;
			for (const FtpRemoteFileInfo& entry : entries)
			{
				const QString name = DecodeRobotMessageText(entry.name);
				if (!entry.isDirectory && !IsJobFileName(name))
				{
					continue;
				}
				m_remoteTable->insertRow(row);
				const QString path = DecodeRobotMessageText(entry.path);
				QTableWidgetItem* nameItem = new QTableWidgetItem(entry.isDirectory ? QString("[%1]").arg(name) : name);
				nameItem->setData(Qt::UserRole, path);
				nameItem->setData(Qt::UserRole + 1, entry.isDirectory);
				m_remoteTable->setItem(row, 0, nameItem);
				m_remoteTable->setItem(row, 1, new QTableWidgetItem(entry.isDirectory ? "目录" : "文件"));
				m_remoteTable->setItem(row, 2, new QTableWidgetItem(entry.isDirectory ? QString() : FormatBytes(static_cast<quint64>(entry.size))));
				m_remoteTable->setItem(row, 3, new QTableWidgetItem(DecodeRobotMessageText(entry.modifiedTime)));
				m_remoteTable->setItem(row, 4, new QTableWidgetItem(path));
				++row;
			}
			m_remoteTable->setSortingEnabled(true);
			m_remoteTable->resizeColumnsToContents();
			m_remoteTable->horizontalHeader()->setStretchLastSection(true);
			m_statusLabel->setText(QString("服务器文件：%1 个").arg(row));
		}

		QList<int> SelectedRows(QTableWidget* table) const
		{
			QList<int> rows;
			if (table == nullptr || table->selectionModel() == nullptr)
			{
				return rows;
			}
			const QModelIndexList selectedRows = table->selectionModel()->selectedRows();
			for (const QModelIndex& index : selectedRows)
			{
				if (!rows.contains(index.row()))
				{
					rows.push_back(index.row());
				}
			}
			std::sort(rows.begin(), rows.end());
			return rows;
		}

		QStringList SelectedLocalFilePaths() const
		{
			QStringList paths;
			const QList<int> rows = SelectedRows(m_localTable);
			for (int row : rows)
			{
				QTableWidgetItem* item = m_localTable->item(row, 0);
				if (item != nullptr)
				{
					paths << item->data(Qt::UserRole).toString();
				}
			}
			return paths;
		}

		QList<QPair<QString, QString>> SelectedRemoteFiles() const
		{
			QList<QPair<QString, QString>> files;
			const QList<int> rows = SelectedRows(m_remoteTable);
			for (int row : rows)
			{
				QTableWidgetItem* item = m_remoteTable->item(row, 0);
				if (item == nullptr || item->data(Qt::UserRole + 1).toBool())
				{
					continue;
				}
				files.push_back(qMakePair(item->data(Qt::UserRole).toString(), item->text()));
			}
			return files;
		}

		void SetBusy(bool busy, const QString& text = QString())
		{
			m_busy = busy;
			for (QWidget* widget : {
				static_cast<QWidget*>(m_unitCombo),
				static_cast<QWidget*>(m_hostEdit),
				static_cast<QWidget*>(m_portEdit),
				static_cast<QWidget*>(m_userEdit),
				static_cast<QWidget*>(m_passwordEdit),
				static_cast<QWidget*>(m_remoteDirEdit),
				static_cast<QWidget*>(m_localDirEdit),
				static_cast<QWidget*>(m_chooseLocalDirBtn),
				static_cast<QWidget*>(m_reloadUnitsBtn),
				static_cast<QWidget*>(m_refreshLocalBtn),
				static_cast<QWidget*>(m_refreshRemoteBtn),
				static_cast<QWidget*>(m_remoteParentBtn),
				static_cast<QWidget*>(m_pickUploadFilesBtn),
				static_cast<QWidget*>(m_uploadSelectedBtn),
				static_cast<QWidget*>(m_downloadSelectedBtn),
				static_cast<QWidget*>(m_deleteLocalBtn),
				static_cast<QWidget*>(m_deleteRemoteBtn),
				static_cast<QWidget*>(m_localTable),
				static_cast<QWidget*>(m_remoteTable) })
			{
				if (widget != nullptr)
				{
					widget->setEnabled(!busy);
				}
			}
			if (m_closeBtn != nullptr)
			{
				m_closeBtn->setEnabled(!busy);
			}
			if (m_statusLabel != nullptr)
			{
				m_statusLabel->setText(text.isEmpty() ? (busy ? "正在执行 FTP 操作..." : "就绪") : text);
			}
		}

		void AppendLog(const QString& text)
		{
			if (m_logText != nullptr)
			{
				m_logText->appendPlainText(QString("[%1] %2")
					.arg(QDateTime::currentDateTime().toString("HH:mm:ss.zzz"), text));
			}
		}

		void RunFtpTask(
			const QString& title,
			const std::function<bool(FtpClient&, QString*)>& work,
			const std::function<void()>& onSuccess = std::function<void()>())
		{
			if (m_busy)
			{
				return;
			}
			// Windows 原生文件选择框会阻塞 Qt 定时器；危险提交点必须同步重读
			// 账号指纹和工程师权限，不能只依赖 250ms 会话轮询或窗口事件。
			if (!m_liveSessionGuard || !m_liveSessionGuard())
			{
				QMessageBox::warning(
					this,
					title,
					QStringLiteral("账号会话或工程师权限已失效，本次 FTP 操作已拒绝。请重新登录。"));
				return;
			}
			QString connectionError;
			const FtpConnection connection = CurrentConnection(&connectionError);
			if (!connectionError.isEmpty())
			{
				QMessageBox::warning(this, title, connectionError);
				return;
			}
			const UnitConfig unit = CurrentUnit();
			RobotDriverAdaptor* driver = RobotDataHelper::GetRobotDriver(m_pContralUnit, unit.unitIndex);
			QString leaseError;
			const auto operationLease = RobotOperationLease::TryAcquire(
				driver, QString("FTP Job：%1").arg(title), &leaseError);
			if (!operationLease)
			{
				QMessageBox::warning(this, title, leaseError);
				return;
			}

			const QString logPath = RobotDataHelper::BuildProjectPath("Log/FtpJobManagement.log");
			QDir().mkpath(QFileInfo(logPath).absolutePath());
			SetBusy(true, title + "...");
			AppendLog(title + "开始。");

			QPointer<FtpJobManagementDialog> dialog(this);
			QThread* taskThread = QThread::create([dialog, connection, title, work, onSuccess, logPath, operationLease]() mutable
				{
					bool ok = false;
					QString error;
					try
					{
						RobotLog log(ToLocalStd(logPath), false);
						FtpClient ftp(
							&log,
							ToLocalStd(connection.host),
							connection.port,
							ToLocalStd(connection.user),
							ToLocalStd(connection.password));
						ftp.setMessageBoxesEnabled(false);
						ok = work(ftp, &error);
					}
					catch (const std::exception& e)
					{
						error = DecodeRobotMessageText(e.what());
					}
					catch (...)
					{
						error = "FTP 操作发生未知异常。";
					}

					QMetaObject::invokeMethod(qApp, [dialog, title, ok, error, onSuccess]() mutable
						{
							if (dialog == nullptr)
							{
								return;
							}
							dialog->SetBusy(false);
							if (ok)
							{
								dialog->AppendLog(title + "完成。");
								if (onSuccess)
								{
									onSuccess();
								}
							}
							else
							{
								const QString message = error.isEmpty() ? title + "失败。" : error;
								dialog->AppendLog(title + "失败：" + message);
								QMessageBox::warning(dialog.data(), title, message);
							}
						}, Qt::QueuedConnection);
				});
			connect(taskThread, &QThread::finished, taskThread, &QObject::deleteLater);
			taskThread->start();
		}

		void RefreshRemoteFiles()
		{
			const QString remoteDir = NormalizeRemotePath(m_remoteDirEdit->text());
			m_remoteDirEdit->setText(remoteDir);
			auto entries = std::make_shared<std::vector<FtpRemoteFileInfo>>();
			RunFtpTask(
				"刷新服务器目录",
				[remoteDir, entries](FtpClient& ftp, QString* error) -> bool
				{
					if (error != nullptr)
					{
						error->clear();
					}
					return ftp.listFiles(ToFtpStd(remoteDir), *entries);
				},
				[this, entries]()
				{
					FillRemoteTable(*entries);
				});
		}

		void ChooseLocalDir()
		{
			const QString selectedDir = QFileDialog::getExistingDirectory(
				this,
				"选择本地 Job 目录",
				m_localDirEdit->text().trimmed().isEmpty() ? RobotDataHelper::BuildProjectPath("Job") : m_localDirEdit->text());
			if (selectedDir.isEmpty())
			{
				return;
			}
			m_localDirEdit->setText(QDir::toNativeSeparators(selectedDir));
			RefreshLocalFiles();
		}

		void GoRemoteParent()
		{
			m_remoteDirEdit->setText(RemoteParentDir(m_remoteDirEdit->text()));
			RefreshRemoteFiles();
		}

		void PickFilesAndUpload()
		{
			const QStringList files = QFileDialog::getOpenFileNames(
				this,
				"选择要上传的 Job 文件",
				m_localDirEdit->text(),
				"Job files (*.srp *.srd *.sr *.ls *.tp *.kl *.pc *.var *.vr *.dt *.job);;All files (*.*)");
			if (files.isEmpty())
			{
				return;
			}
			UploadFiles(files);
		}

		void UploadSelectedLocalFiles()
		{
			const QStringList files = SelectedLocalFilePaths();
			if (files.isEmpty())
			{
				QMessageBox::information(this, "上传选中", "请先选择要上传的本地文件。");
				return;
			}
			UploadFiles(files);
		}

		void UploadFiles(const QStringList& localFiles)
		{
			const QString remoteDir = NormalizeRemotePath(m_remoteDirEdit->text());
			QList<QPair<QString, QString>> transfers;
			for (const QString& localFile : localFiles)
			{
				const QFileInfo info(localFile);
				if (!info.exists() || !info.isFile())
				{
					continue;
				}
				transfers.push_back(qMakePair(info.absoluteFilePath(), JoinRemotePath(remoteDir, info.fileName())));
			}
			if (transfers.isEmpty())
			{
				QMessageBox::warning(this, "上传文件", "没有可上传的文件。");
				return;
			}

			RunFtpTask(
				QString("上传 %1 个文件").arg(transfers.size()),
				[transfers](FtpClient& ftp, QString* error) -> bool
				{
					for (const QPair<QString, QString>& transfer : transfers)
					{
						if (!ftp.uploadFile(ToLocalStd(transfer.first), ToFtpStd(transfer.second), true))
						{
							if (error != nullptr)
							{
								*error = QString("上传失败：%1").arg(transfer.first);
							}
							return false;
						}
					}
					return true;
				},
				[this]()
				{
					RefreshLocalFiles();
					RefreshRemoteFiles();
				});
		}

		void DownloadSelectedRemoteFiles()
		{
			const QList<QPair<QString, QString>> files = SelectedRemoteFiles();
			if (files.isEmpty())
			{
				QMessageBox::information(this, "下载选中", "请先选择服务器文件，目录不会被直接下载。");
				return;
			}

			const QString localDir = QDir::fromNativeSeparators(m_localDirEdit->text().trimmed());
			if (localDir.isEmpty())
			{
				QMessageBox::warning(this, "下载选中", "本地目录不能为空。");
				return;
			}
			QDir().mkpath(localDir);

			QList<QPair<QString, QString>> transfers;
			for (const QPair<QString, QString>& file : files)
			{
				transfers.push_back(qMakePair(file.first, QDir(localDir).absoluteFilePath(file.second)));
			}

			RunFtpTask(
				QString("下载 %1 个文件").arg(transfers.size()),
				[transfers](FtpClient& ftp, QString* error) -> bool
				{
					for (const QPair<QString, QString>& transfer : transfers)
					{
						if (!ftp.downloadFile(ToFtpStd(transfer.first), ToLocalStd(transfer.second)))
						{
							if (error != nullptr)
							{
								*error = QString("下载失败：%1").arg(transfer.first);
							}
							return false;
						}
					}
					return true;
				},
				[this]()
				{
					RefreshLocalFiles();
				});
		}

		void DeleteSelectedLocalFiles()
		{
			const QStringList files = SelectedLocalFilePaths();
			if (files.isEmpty())
			{
				QMessageBox::information(this, "删除本地文件", "请先选择要删除的本地文件。");
				return;
			}
			if (QMessageBox::question(
				this,
				"删除本地文件",
				QString("确定删除选中的 %1 个本地文件吗？").arg(files.size()),
				QMessageBox::Yes | QMessageBox::No,
				QMessageBox::No) != QMessageBox::Yes)
			{
				return;
			}

			int removed = 0;
			for (const QString& file : files)
			{
				if (QFile::remove(file))
				{
					++removed;
					AppendLog("已删除本地文件：" + QDir::toNativeSeparators(file));
				}
				else
				{
					AppendLog("删除本地文件失败：" + QDir::toNativeSeparators(file));
				}
			}
			RefreshLocalFiles();
			m_statusLabel->setText(QString("已删除本地文件：%1/%2").arg(removed).arg(files.size()));
		}

		void DeleteSelectedRemoteFiles()
		{
			const QList<QPair<QString, QString>> files = SelectedRemoteFiles();
			if (files.isEmpty())
			{
				QMessageBox::information(this, "删除服务器文件", "请先选择服务器文件，目录不会被直接删除。");
				return;
			}
			if (QMessageBox::question(
				this,
				"删除服务器文件",
				QString("确定删除服务器上的 %1 个文件吗？").arg(files.size()),
				QMessageBox::Yes | QMessageBox::No,
				QMessageBox::No) != QMessageBox::Yes)
			{
				return;
			}

			auto deletedCount = std::make_shared<int>(0);
			auto failedFiles = std::make_shared<QStringList>();
			RunFtpTask(
				QString("删除服务器 %1 个文件").arg(files.size()),
				[files, deletedCount, failedFiles](FtpClient& ftp, QString* error) -> bool
				{
					if (error != nullptr)
					{
						error->clear();
					}
					for (const QPair<QString, QString>& file : files)
					{
						if (ftp.deleteFile(ToFtpStd(file.first), false))
						{
							++(*deletedCount);
						}
						else
						{
							failedFiles->append(file.first);
						}
					}
					return true;
				},
				[this, files, deletedCount, failedFiles]()
				{
					RefreshRemoteFiles();
					const int failedCount = failedFiles->size();
					const QString summary = QString("服务器文件删除完成：成功 %1 个，失败 %2 个。")
						.arg(*deletedCount)
						.arg(failedCount);
					m_statusLabel->setText(summary);
					AppendLog(summary);
					if (failedCount > 0)
					{
						const QString detail = failedFiles->mid(0, 10).join(QStringLiteral("\n"));
						const QString more = failedCount > 10 ? QString("\n... 另有 %1 个失败文件").arg(failedCount - 10) : QString();
						QMessageBox::warning(this, "删除服务器文件", summary + "\n\n" + detail + more);
					}
					else
					{
						QMessageBox::information(this, "删除服务器文件", QString("已删除服务器上的 %1 个文件。").arg(files.size()));
					}
				});
		}

		ContralUnit* m_pContralUnit = nullptr;
		int m_initialUnitIndex = -1;
		bool m_busy = false;
		std::function<bool()> m_liveSessionGuard;
		QList<UnitConfig> m_units;
		QComboBox* m_unitCombo = nullptr;
		QLineEdit* m_hostEdit = nullptr;
		QLineEdit* m_portEdit = nullptr;
		QLineEdit* m_userEdit = nullptr;
		QLineEdit* m_passwordEdit = nullptr;
		QLineEdit* m_remoteDirEdit = nullptr;
		QLineEdit* m_localDirEdit = nullptr;
		QPushButton* m_chooseLocalDirBtn = nullptr;
		QPushButton* m_reloadUnitsBtn = nullptr;
		QPushButton* m_refreshLocalBtn = nullptr;
		QPushButton* m_refreshRemoteBtn = nullptr;
		QPushButton* m_remoteParentBtn = nullptr;
		QPushButton* m_pickUploadFilesBtn = nullptr;
		QPushButton* m_uploadSelectedBtn = nullptr;
		QPushButton* m_downloadSelectedBtn = nullptr;
		QPushButton* m_deleteLocalBtn = nullptr;
		QPushButton* m_deleteRemoteBtn = nullptr;
		QPushButton* m_closeBtn = nullptr;
		QTableWidget* m_localTable = nullptr;
		QTableWidget* m_remoteTable = nullptr;
		QLabel* m_statusLabel = nullptr;
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
		using RefreshCameraParamsFunc = std::function<bool(SKJCameraParameterValues&, QString*)>;
		using SetCameraParamFunc = std::function<bool(SKJCameraControlClient::Parameter, int, QString*)>;
		using SetCameraSwitchFunc = std::function<bool(bool, QString*)>;

		explicit GroovePointCloudDialog(
			RefreshCameraParamsFunc refreshCameraParams,
			SetCameraParamFunc setCameraParam,
			SetCameraSwitchFunc setLaserEnabled,
			QWidget* parent = nullptr)
			: QDialog(parent)
			, m_refreshCameraParams(std::move(refreshCameraParams))
			, m_setCameraParam(std::move(setCameraParam))
			, m_setLaserEnabled(std::move(setLaserEnabled))
		{
			setWindowTitle("坡口相机点云预览");
			setModal(false);
			setWindowFlag(Qt::WindowStaysOnTopHint, true);
			setAttribute(Qt::WA_DeleteOnClose, true);

			QVBoxLayout* mainLayout = new QVBoxLayout(this);
			mainLayout->setContentsMargins(ScalePixels(14), ScalePixels(14), ScalePixels(14), ScalePixels(14));
			mainLayout->setSpacing(ScalePixels(10));

			QHBoxLayout* toolbarLayout = new QHBoxLayout();
			toolbarLayout->setContentsMargins(0, 0, 0, 0);
			m_previewModeTabs = new QTabBar(this);
			m_previewModeTabs->setObjectName("GroovePreviewModeTabs");
			m_previewModeTabs->setDrawBase(false);
			m_previewModeTabs->setExpanding(false);
			m_previewModeTabs->addTab("滤波前");
			m_previewModeTabs->addTab("滤波后");
			m_previewModeTabs->addTab("相机图像");
			m_previewModeTabs->setStyleSheet(
				"QTabBar::tab {"
				"background:#182832;"
				"color:#DDFBFF;"
				"border:1px solid #35596D;"
				"border-bottom-color:#2A4352;"
				"padding:7px 22px;"
				"min-width:72px;"
				"font-size:15px;"
				"}"
				"QTabBar::tab:first {"
				"border-top-left-radius:6px;"
				"}"
				"QTabBar::tab:last {"
				"border-top-right-radius:6px;"
				"}"
				"QTabBar::tab:selected {"
				"background:#246A58;"
				"border-color:#7DE8C0;"
				"color:#FFFFFF;"
				"}"
				"QTabBar::tab:hover:!selected {"
				"background:#213949;"
				"border-color:#5F9BB2;"
				"}");
			toolbarLayout->addWidget(m_previewModeTabs, 0, Qt::AlignLeft);
			m_trendLineToggleButton = new QPushButton("三段线", this);
			m_trendLineToggleButton->setCheckable(true);
			m_trendLineToggleButton->setCursor(Qt::PointingHandCursor);
			m_trendLineToggleButton->setMinimumHeight(ScalePixels(34));
			m_trendLineToggleButton->setStyleSheet(
				"QPushButton {"
				"background:#182832;"
				"color:#DDFBFF;"
				"border:1px solid #35596D;"
				"border-radius:6px;"
				"padding:6px 18px;"
				"font-size:15px;"
				"}"
				"QPushButton:hover {"
				"background:#213949;"
				"border-color:#5F9BB2;"
				"}"
				"QPushButton:checked {"
				"background:#246A58;"
				"border-color:#7DE8C0;"
				"color:#FFFFFF;"
				"}");
			toolbarLayout->addSpacing(ScalePixels(8));
			toolbarLayout->addWidget(m_trendLineToggleButton, 0, Qt::AlignLeft);
			toolbarLayout->addStretch(1);
			mainLayout->addLayout(toolbarLayout);

			QGroupBox* cameraControlGroup = new QGroupBox("相机参数", this);
			QGridLayout* cameraControlLayout = new QGridLayout(cameraControlGroup);
			cameraControlLayout->setContentsMargins(ScalePixels(12), ScalePixels(10), ScalePixels(12), ScalePixels(10));
			cameraControlLayout->setHorizontalSpacing(ScalePixels(10));
			cameraControlLayout->setVerticalSpacing(ScalePixels(8));
			AddCameraParameterRow(
				cameraControlLayout,
				0,
				m_exposureControl,
				"曝光",
				SKJCameraControlClient::Parameter::Exposure,
				25,
				5000,
				25);
			AddCameraParameterRow(
				cameraControlLayout,
				1,
				m_gainControl,
				"增益",
				SKJCameraControlClient::Parameter::Gain,
				0,
				100,
				1);
			AddCameraParameterRow(
				cameraControlLayout,
				2,
				m_binarizeControl,
				"二值化",
				SKJCameraControlClient::Parameter::Binarize,
				0,
				255,
				1);
			QLabel* laserLabel = new QLabel("激光", this);
			m_laserToggleButton = new QPushButton("打开激光", this);
			m_laserToggleButton->setObjectName("CameraLaserToggleButton");
			m_laserToggleButton->setCheckable(true);
			m_laserToggleButton->setCursor(Qt::PointingHandCursor);
			m_laserToggleButton->setMinimumSize(ScalePixels(138), ScalePixels(46));
			m_laserToggleButton->setStyleSheet(
				"QPushButton#CameraLaserToggleButton {"
				"background:#18313F;"
				"color:#DDFBFF;"
				"border:1px solid #4C7890;"
				"border-radius:18px;"
				"font-size:16px;"
				"font-weight:600;"
				"padding:6px 18px;"
				"}"
				"QPushButton#CameraLaserToggleButton:hover {"
				"background:#21475A;"
				"border-color:#7FD7E7;"
				"}"
				"QPushButton#CameraLaserToggleButton:checked {"
				"background:#236A54;"
				"border-color:#8EF2C4;"
				"color:#FFFFFF;"
				"}"
				"QPushButton#CameraLaserToggleButton:pressed {"
				"background:#123242;"
				"}");
			cameraControlLayout->addWidget(laserLabel, 3, 0);
			cameraControlLayout->addWidget(m_laserToggleButton, 3, 1);
			// 图像传输总开关：断开图像口(50001)给相机端减负，排查/规避图像流挤占导致的点云跳帧。
			QLabel* imageTransportLabel = new QLabel("图像传输", this);
			m_imageTransportToggleButton = new QPushButton("图像传输：开", this);
			m_imageTransportToggleButton->setCheckable(true);
			m_imageTransportToggleButton->setChecked(true);
			m_imageTransportToggleButton->setCursor(Qt::PointingHandCursor);
			m_imageTransportToggleButton->setMinimumSize(ScalePixels(138), ScalePixels(46));
			m_imageTransportToggleButton->setToolTip("关闭后断开相机图像口（预览图像/扫描存图停用），点云不受影响；用于验证或规避图像流挤占相机资源造成的点云丢帧。");
			connect(m_imageTransportToggleButton, &QPushButton::toggled, this, [this](bool enabled)
				{
					m_imageTransportToggleButton->setText(enabled ? "图像传输：开" : "图像传输：关");
					if (m_imageTransportToggle)
					{
						m_imageTransportToggle(enabled);
					}
				});
			cameraControlLayout->addWidget(imageTransportLabel, 4, 0);
			cameraControlLayout->addWidget(m_imageTransportToggleButton, 4, 1);
			// 接收缓冲帧数：SDK v1.2.0 接收 FIFO 深度（本地取帧缓冲，非下发相机的参数），
			// 松开滑条即下发所有相机客户端并保存。样式与曝光/增益行一致。
			QLabel* frameBufferLabel = new QLabel("接收缓冲帧数", this);
			frameBufferLabel->setToolTip(
				"相机客户端接收 FIFO 深度（2~16，默认 8，SDK 出厂默认 2）。\n"
				"取帧线程短暂卡顿时的缓冲垫，越大越抗积压；对所有机器人单元生效，\n"
				"改动立即应用并保存，重启保持。扫描流程运行中改动只保存不下发\n"
				"（SDK 修改缓冲会清空未取帧，避免打断采集），相机下次连接时生效。");
			m_frameBufferSlider = new QSlider(Qt::Horizontal, this);
			m_frameBufferSlider->setRange(2, 16);
			m_frameBufferSlider->setSingleStep(1);
			m_frameBufferSlider->setPageStep(2);
			m_frameBufferSlider->setMinimumWidth(ScalePixels(190));
			m_frameBufferSlider->setValue(8);
			m_frameBufferSpin = new QSpinBox(this);
			m_frameBufferSpin->setRange(2, 16);
			m_frameBufferSpin->setSingleStep(1);
			m_frameBufferSpin->setButtonSymbols(QAbstractSpinBox::NoButtons);
			m_frameBufferSpin->setKeyboardTracking(false);
			m_frameBufferSpin->setAlignment(Qt::AlignCenter);
			m_frameBufferSpin->setFixedWidth(ScalePixels(86));
			m_frameBufferSpin->setValue(8);
			m_frameBufferSpin->setStyleSheet(
				"QSpinBox {"
				"background:#071119;"
				"color:#DDFBFF;"
				"border:1px solid #315168;"
				"border-radius:4px;"
				"padding:3px 6px;"
				"selection-background-color:#256A7A;"
				"}"
				"QSpinBox:focus {"
				"border-color:#7FD7E7;"
				"}");
			// 提交统一走 CommitFrameBufferCount（带同值去重）：点击滑轨/滚轮/方向键只发
			// valueChanged 不发 sliderReleased，靠去抖定时器兜底提交，保证显示值必然落库生效。
			m_frameBufferCommitTimer = new QTimer(this);
			m_frameBufferCommitTimer->setSingleShot(true);
			m_frameBufferCommitTimer->setInterval(400);
			connect(m_frameBufferCommitTimer, &QTimer::timeout, this, [this]() { CommitFrameBufferCount(); });
			connect(m_frameBufferSlider, &QSlider::valueChanged, this, [this](int value)
				{
					if (m_frameBufferSpin != nullptr)
					{
						const QSignalBlocker blocker(m_frameBufferSpin);
						m_frameBufferSpin->setValue(value);
					}
					if (m_frameBufferSlider != nullptr && !m_frameBufferSlider->isSliderDown()
						&& m_frameBufferCommitTimer != nullptr)
					{
						m_frameBufferCommitTimer->start();  // 非拖拽改值（点槽/滚轮/键盘）去抖提交
					}
				});
			connect(m_frameBufferSlider, &QSlider::sliderReleased, this, [this]()
				{
					if (m_frameBufferCommitTimer != nullptr)
					{
						m_frameBufferCommitTimer->stop();
					}
					CommitFrameBufferCount();
				});
			connect(m_frameBufferSpin, &QSpinBox::editingFinished, this, [this]()
				{
					if (m_frameBufferSpin == nullptr)
					{
						return;
					}
					if (m_frameBufferSlider != nullptr)
					{
						const QSignalBlocker blocker(m_frameBufferSlider);
						m_frameBufferSlider->setValue(m_frameBufferSpin->value());
					}
					if (m_frameBufferCommitTimer != nullptr)
					{
						m_frameBufferCommitTimer->stop();
					}
					CommitFrameBufferCount();  // 同值去重：单纯失焦不会重发
				});
			cameraControlLayout->addWidget(frameBufferLabel, 5, 0);
			cameraControlLayout->addWidget(m_frameBufferSlider, 5, 1);
			cameraControlLayout->addWidget(m_frameBufferSpin, 5, 2);
			m_refreshParamsButton = new QPushButton("刷新参数", this);
			m_refreshParamsButton->setMinimumHeight(ScalePixels(32));
			m_cameraControlStatusLabel = new QLabel("参数控制未连接", this);
			m_cameraControlStatusLabel->setWordWrap(true);
			cameraControlLayout->addWidget(m_refreshParamsButton, 6, 0);
			cameraControlLayout->addWidget(m_cameraControlStatusLabel, 6, 1, 1, 2);
			cameraControlGroup->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);

			m_view = new GroovePointCloudView(this);
			const int plotSide = ScalePixels(ComputePlotSide(parent));
			m_view->setFixedSize(plotSide, plotSide);
			m_view->ClearPreview("等待相机点云...");

			m_infoText = new QPlainTextEdit(this);
			m_infoText->setReadOnly(true);
			m_infoText->setLineWrapMode(QPlainTextEdit::WidgetWidth);
			m_infoText->setMinimumWidth(ScalePixels(480));
			m_infoText->setMaximumWidth(ScalePixels(560));
			m_infoText->setMinimumHeight(ScalePixels(360));
			m_infoText->setMaximumHeight(ScalePixels(430));
			m_infoText->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
			m_infoText->setPlainText("坡口相机数据：等待测试...");

			QWidget* rightPanel = new QWidget(this);
			rightPanel->setMinimumWidth(ScalePixels(480));
			rightPanel->setMaximumWidth(ScalePixels(560));
			QVBoxLayout* rightPanelLayout = new QVBoxLayout(rightPanel);
			rightPanelLayout->setContentsMargins(0, 0, 0, 0);
			rightPanelLayout->setSpacing(ScalePixels(10));
			rightPanelLayout->addWidget(m_infoText, 0);
			rightPanelLayout->addWidget(cameraControlGroup, 1);

			// 相机实时图像（SDK v1.2.0 图像传输）：占用主显示区，经顶部「相机图像」页签切换，与点云视图互斥显示。
			m_pCameraImageLabel = new QLabel("相机图像：等待图像帧...\n（需相机图像口 50001 支持）", this);
			m_pCameraImageLabel->setAlignment(Qt::AlignCenter);
			m_pCameraImageLabel->setStyleSheet("QLabel { background: #101C24; color: #6E8894; border: 1px solid #2B4552; border-radius: 8px; }");
			// Ignored：布局忽略 pixmap 撑起的 sizeHint，防 setPixmap→sizeHint→布局放大 的反馈膨胀。
			m_pCameraImageLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
			m_pCameraImageLabel->hide();

			QHBoxLayout* contentLayout = new QHBoxLayout();
			contentLayout->setSpacing(ScalePixels(12));
			contentLayout->addWidget(m_view, 1, Qt::AlignCenter);
			contentLayout->addWidget(m_pCameraImageLabel, 1);
			contentLayout->addWidget(rightPanel, 0);
			mainLayout->addLayout(contentLayout, 1);

			connect(m_previewModeTabs, &QTabBar::currentChanged, this, [this](int index)
				{
					SetPreviewModeTab(index);
				});
			connect(m_trendLineToggleButton, &QPushButton::clicked, this, [this](bool checked)
				{
					SetTrendLineOverlay(checked);
				});
			connect(m_refreshParamsButton, &QPushButton::clicked, this, [this]()
				{
					RefreshCameraControlParams();
				});
			connect(m_laserToggleButton, &QPushButton::clicked, this, [this](bool checked)
				{
					ApplyLaserEnabled(checked);
				});
			SetPreviewModeTab(0);
			adjustSize();
			setMinimumSize(sizeHint());
			resize(sizeHint());
		}

		void SetFrame(const udpDataShow& rawFrame, const QString& statusText, const QString& detailText)
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
			UpdateInfoText(detailText);
			m_hasFrame = true;
			RefreshView();
		}

		void SetImageTransportToggleHandler(std::function<void(bool)> handler)
		{
			m_imageTransportToggle = std::move(handler);
		}

		void SetFrameBufferCountHandler(std::function<void(int)> handler)
		{
			m_frameBufferCountChanged = std::move(handler);
		}

		void ShowCameraControlMessage(const QString& text, bool ok)
		{
			UpdateCameraControlStatus(text, ok);
		}

		void SetFrameBufferCountValue(int count)
		{
			if (m_frameBufferSlider != nullptr)
			{
				const QSignalBlocker sliderBlocker(m_frameBufferSlider);
				m_frameBufferSlider->setValue(count);
			}
			if (m_frameBufferSpin != nullptr)
			{
				const QSignalBlocker spinBlocker(m_frameBufferSpin);
				m_frameBufferSpin->setValue(count);
			}
			m_lastSentFrameBufferCount = count;  // 外部装载值即已生效基线，防止随后一次去抖误提交
			if (m_frameBufferCommitTimer != nullptr)
			{
				m_frameBufferCommitTimer->stop();
			}
		}

		void CommitFrameBufferCount()
		{
			if (m_frameBufferSlider == nullptr)
			{
				return;
			}
			const int value = m_frameBufferSlider->value();
			if (value == m_lastSentFrameBufferCount)
			{
				return;  // 同值去重：失焦重复 editingFinished / 原地按放滑块不重发
			}
			m_lastSentFrameBufferCount = value;
			if (m_frameBufferCountChanged)
			{
				m_frameBufferCountChanged(value);
			}
		}

		void SetCameraImage(const QImage& image)
		{
			if (m_pCameraImageLabel == nullptr)
			{
				return;
			}
			if (image.isNull())
			{
				m_pCameraImageLabel->setText("相机图像：等待图像帧...\n（需相机图像口 50001 支持）");
				return;
			}
			m_pCameraImageLabel->setPixmap(QPixmap::fromImage(image).scaled(
				m_pCameraImageLabel->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
		}

		void ClearPreview(const QString& statusText, const QString& detailText = QString())
		{
			m_hasFrame = false;
			m_statusText = statusText;
			UpdateInfoText(detailText.isEmpty() ? statusText : detailText);
			m_filteredFrameValid = false;
			m_filterBuildRunning = false;
			++m_filterBuildGeneration;
			ResetViewStates();
			if (m_view != nullptr)
			{
				m_view->ClearPreview(statusText);
			}
		}

		void RefreshCameraControlParams()
		{
			if (!m_refreshCameraParams)
			{
				UpdateCameraControlStatus("未配置相机参数控制。", false);
				return;
			}

			SKJCameraParameterValues values;
			QString error;
			if (!m_refreshCameraParams(values, &error))
			{
				UpdateCameraControlStatus(error, false);
				return;
			}

			m_loadingCameraParams = true;
			SetCameraControlValue(m_exposureControl, values.exposure);
			SetCameraControlValue(m_gainControl, values.gain);
			SetCameraControlValue(m_binarizeControl, values.binarize);
			m_loadingCameraParams = false;
			UpdateCameraControlStatus(
				QString("已同步：曝光=%1，增益=%2，二值化=%3")
					.arg(values.exposure)
					.arg(values.gain)
					.arg(values.binarize),
				true);
		}

	private:
		struct CameraParameterControl
		{
			SKJCameraControlClient::Parameter parameter = SKJCameraControlClient::Parameter::Exposure;
			QString name;
			QSlider* slider = nullptr;
			QSpinBox* spin = nullptr;
		};

		static int ScalePixels(int value)
		{
			constexpr double kPreviewUiScale = 0.8;
			return std::max(1, static_cast<int>(std::round(value * kPreviewUiScale)));
		}

		void AddCameraParameterRow(
			QGridLayout* layout,
			int row,
			CameraParameterControl& control,
			const QString& name,
			SKJCameraControlClient::Parameter parameter,
			int minimum,
			int maximum,
			int singleStep)
		{
			control.parameter = parameter;
			control.name = name;

			QLabel* label = new QLabel(name, this);
			control.slider = new QSlider(Qt::Horizontal, this);
			control.slider->setRange(minimum, maximum);
			control.slider->setSingleStep(std::max(1, singleStep));
			control.slider->setPageStep(std::max(1, singleStep * 10));
			control.slider->setMinimumWidth(ScalePixels(190));

			control.spin = new QSpinBox(this);
			control.spin->setRange(minimum, maximum);
			control.spin->setSingleStep(std::max(1, singleStep));
			control.spin->setButtonSymbols(QAbstractSpinBox::NoButtons);
			control.spin->setKeyboardTracking(false);
			control.spin->setAlignment(Qt::AlignCenter);
			control.spin->setFixedWidth(ScalePixels(86));
			control.spin->setStyleSheet(
				"QSpinBox {"
				"background:#071119;"
				"color:#DDFBFF;"
				"border:1px solid #315168;"
				"border-radius:4px;"
				"padding:3px 6px;"
				"selection-background-color:#256A7A;"
				"}"
				"QSpinBox:focus {"
				"border-color:#7FD7E7;"
				"}");

			layout->addWidget(label, row, 0);
			layout->addWidget(control.slider, row, 1);
			layout->addWidget(control.spin, row, 2);

			CameraParameterControl* controlPtr = &control;
			connect(control.slider, &QSlider::valueChanged, this, [controlPtr](int value)
				{
					if (controlPtr != nullptr && controlPtr->spin != nullptr)
					{
						const QSignalBlocker blocker(controlPtr->spin);
						controlPtr->spin->setValue(value);
					}
				});
			connect(control.slider, &QSlider::sliderReleased, this, [this, controlPtr]()
				{
					if (controlPtr != nullptr && controlPtr->slider != nullptr)
					{
						ApplyCameraParameter(*controlPtr, controlPtr->slider->value());
					}
				});
			connect(control.spin, QOverload<int>::of(&QSpinBox::valueChanged), this, [this, controlPtr](int value)
				{
					if (controlPtr != nullptr && controlPtr->slider != nullptr)
					{
						const QSignalBlocker blocker(controlPtr->slider);
						controlPtr->slider->setValue(value);
					}
					if (controlPtr != nullptr)
					{
						ApplyCameraParameter(*controlPtr, value);
					}
				});
		}

		void SetCameraControlValue(CameraParameterControl& control, int value)
		{
			if (control.slider == nullptr || control.spin == nullptr)
			{
				return;
			}
			value = std::clamp(value, control.spin->minimum(), control.spin->maximum());
			const QSignalBlocker spinBlocker(control.spin);
			const QSignalBlocker sliderBlocker(control.slider);
			control.spin->setValue(value);
			control.slider->setValue(value);
		}

		void ApplyCameraParameter(const CameraParameterControl& control, int value)
		{
			if (m_loadingCameraParams)
			{
				return;
			}
			if (!m_setCameraParam)
			{
				UpdateCameraControlStatus("未配置相机参数控制。", false);
				return;
			}

			QString error;
			if (!m_setCameraParam(control.parameter, value, &error))
			{
				UpdateCameraControlStatus(error, false);
				return;
			}
			UpdateCameraControlStatus(QString("已设置%1=%2").arg(control.name).arg(value), true);
		}

		void ApplyLaserEnabled(bool enabled)
		{
			if (!m_setLaserEnabled)
			{
				RollbackLaserButton(!enabled);
				UpdateCameraControlStatus("未配置相机激光控制。", false);
				return;
			}

			QString error;
			if (!m_setLaserEnabled(enabled, &error))
			{
				RollbackLaserButton(!enabled);
				UpdateCameraControlStatus(error, false);
				return;
			}
			UpdateLaserButtonText(enabled);
			UpdateCameraControlStatus(enabled ? "已打开激光。" : "已关闭激光。", true);
		}

		void RollbackLaserButton(bool enabled)
		{
			if (m_laserToggleButton == nullptr)
			{
				return;
			}
			const QSignalBlocker blocker(m_laserToggleButton);
			m_laserToggleButton->setChecked(enabled);
			UpdateLaserButtonText(enabled);
		}

		void UpdateLaserButtonText(bool enabled)
		{
			if (m_laserToggleButton != nullptr)
			{
				m_laserToggleButton->setText(enabled ? "关闭激光" : "打开激光");
			}
		}

		void UpdateCameraControlStatus(const QString& text, bool ok)
		{
			if (m_cameraControlStatusLabel == nullptr)
			{
				return;
			}
			m_cameraControlStatusLabel->setText(text.trimmed().isEmpty() ? QStringLiteral("参数控制状态未知") : text);
			m_cameraControlStatusLabel->setStyleSheet(ok ? "color:#4ADE80;" : "color:#F87171;");
		}

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
				maxSide = std::min(available.width() - 560, available.height() - 170);
			}
			return std::max(520, std::min(maxSide, 1080));
		}

		void UpdateInfoText(const QString& text)
		{
			if (m_infoText == nullptr)
			{
				return;
			}
			m_infoText->setPlainText(text.trimmed().isEmpty() ? QStringLiteral("坡口相机数据：等待测试...") : text);
			m_infoText->moveCursor(QTextCursor::Start);
		}

		void SetPreviewModeTab(int index)
		{
			if (m_previewModeTabs != nullptr && m_previewModeTabs->currentIndex() != index)
			{
				const QSignalBlocker blocker(m_previewModeTabs);
				m_previewModeTabs->setCurrentIndex(index);
			}

			// 页签2=相机图像：主显示区切到实时图像，点云视图隐藏；其余页签恢复点云视图。
			const bool showCameraImage = (index == 2);
			if (m_pCameraImageLabel != nullptr)
			{
				m_pCameraImageLabel->setVisible(showCameraImage);
			}
			if (m_view != nullptr)
			{
				m_view->setVisible(!showCameraImage);
			}
			if (showCameraImage)
			{
				SetTrendLineButtonChecked(false);
				m_showTrendLines = false;
				return;
			}

			const bool showFiltered = (index == 1);
			if (!showFiltered)
			{
				SetTrendLineButtonChecked(false);
				m_showTrendLines = false;
			}
			SetFilteredMode(showFiltered);
		}

		void SetTrendLineOverlay(bool enabled)
		{
			if (enabled && m_previewModeTabs != nullptr && m_previewModeTabs->currentIndex() != 1)
			{
				m_previewModeTabs->setCurrentIndex(1);  // 三段线叠加只在「滤波后」视图有意义（含从相机图像页切回）
			}
			SetTrendLineButtonChecked(enabled);
			m_showTrendLines = enabled;
			RefreshView(false, true);
		}

		void SetTrendLineButtonChecked(bool enabled)
		{
			if (m_trendLineToggleButton == nullptr)
			{
				return;
			}
			const QSignalBlocker blocker(m_trendLineToggleButton);
			m_trendLineToggleButton->setChecked(enabled);
		}

		void SetFilteredMode(bool showFiltered)
		{
			if (m_showFiltered != showFiltered)
			{
				SaveCurrentViewState();
			}
			m_showFiltered = showFiltered;
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
			const QString modeText = m_showTrendLines
				? (showCachedFiltered ? "三段线" : "三段线计算中")
				: (m_showFiltered
					? (showCachedFiltered ? "滤波后" : "滤波后计算中")
					: "滤波前");
			m_view->SetShowTrendLines(m_showTrendLines && showCachedFiltered);
			m_view->SetFrame(frame, QString("%1  %2").arg(m_statusText, modeText), CurrentModeViewState());
			SaveCurrentViewState();
		}

		GroovePointCloudView* m_view = nullptr;
		QPlainTextEdit* m_infoText = nullptr;
		QLabel* m_pCameraImageLabel = nullptr;
		QPushButton* m_imageTransportToggleButton = nullptr;
		std::function<void(bool)> m_imageTransportToggle;
		QSlider* m_frameBufferSlider = nullptr;
		QSpinBox* m_frameBufferSpin = nullptr;
		QTimer* m_frameBufferCommitTimer = nullptr;
		int m_lastSentFrameBufferCount = 8;
		std::function<void(int)> m_frameBufferCountChanged;
		QTabBar* m_previewModeTabs = nullptr;
		QPushButton* m_trendLineToggleButton = nullptr;
		QPushButton* m_refreshParamsButton = nullptr;
		QPushButton* m_laserToggleButton = nullptr;
		QLabel* m_cameraControlStatusLabel = nullptr;
		CameraParameterControl m_exposureControl;
		CameraParameterControl m_gainControl;
		CameraParameterControl m_binarizeControl;
		RefreshCameraParamsFunc m_refreshCameraParams;
		SetCameraParamFunc m_setCameraParam;
		SetCameraSwitchFunc m_setLaserEnabled;
		udpDataShow m_rawFrame;
		udpDataShow m_filteredFrame;
		QString m_statusText;
		bool m_loadingCameraParams = false;
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

	// PointCloudVec3 / PointCloud3DView 已抽取到 include/PointCloud3DView.h（pcview 命名空间）。

	static QString FindLatestLaserPointDirectory(const QString& robotName)
	{
		const QString resultRoot = RobotDataHelper::BuildProjectPath(QString("Result/%1").arg(robotName.trimmed().isEmpty() ? "RobotC" : robotName));
		QDir rootDir(resultRoot);
		if (!rootDir.exists())
		{
			return QString();
		}

		const QStringList expectedFiles = {
			"PreciseLaserPoint.txt",
			"PreciseLaserPoint_Classified.txt",
			"PreciseLaserPoint_CornerComp_Classified.txt",
			"PreciseLaserPoint_WeldPose_2mm_SeamComp.txt"
		};
		const QFileInfoList resultDirs = rootDir.entryInfoList(QDir::Dirs | QDir::NoDotAndDotDot, QDir::Time);
		for (const QFileInfo& resultDirInfo : resultDirs)
		{
			const QString laserDirPath = QDir(resultDirInfo.absoluteFilePath()).filePath("LaserPoint");
			QDir laserDir(laserDirPath);
			if (!laserDir.exists())
			{
				continue;
			}
			for (const QString& fileName : expectedFiles)
			{
				if (QFileInfo::exists(laserDir.filePath(fileName)))
				{
					return QDir::toNativeSeparators(laserDirPath);
				}
			}
		}
		return QString();
	}

	class PointCloudViewerDialog final : public QDialog
	{
	public:
		explicit PointCloudViewerDialog(const QString& robotName, QWidget* parent = nullptr)
			: QDialog(parent)
			, m_robotName(robotName.trimmed().isEmpty() ? "RobotC" : robotName.trimmed())
		{
			setWindowTitle("点云查看");
			setAttribute(Qt::WA_DeleteOnClose, true);
			setModal(false);
			ApplyUnifiedWindowChrome(this);
			ResizeWindowForAvailableGeometry(this, QSize(1480, 900), 0.92, 0.86);
			BuildUi();

			const QString latestDir = FindLatestLaserPointDirectory(m_robotName);
			if (!latestDir.isEmpty())
			{
				m_dirEdit->setText(latestDir);
			}
			else
			{
				m_dirEdit->setText(RobotDataHelper::BuildProjectPath(QString("Result/%1").arg(m_robotName)));
			}
			// 不在构造里同步加载：show 之后由事件循环触发后台加载（先显示界面、再读点云、进度条反馈）。
			QMetaObject::invokeMethod(this, [this]() { LoadCurrentDirectoryAsync(); }, Qt::QueuedConnection);
		}

		~PointCloudViewerDialog() override
		{
			// 关闭/退出时若后台读取线程仍在跑：置销毁标记让其尽快收尾（进度回调返回 false 取消），
			// 等计数归零后再析构——期间排队的 invokeMethod 由 ~QObject 自动清除，安全。
			m_destroyed->store(true);
			while (m_workerCount->load() > 0)
			{
				std::this_thread::sleep_for(std::chrono::milliseconds(5));
			}
		}

	private:
		void BuildUi()
		{
			setStyleSheet(
				"QDialog, QWidget { background: #101820; color: #ECF3F4; }"
				"QPushButton { background: #233645; color: #F5FAFA; border: 1px solid #3C6173; border-radius: 8px; padding: 8px 14px; font-size: 14px; }"
				"QPushButton:hover { background: #2D5465; border-color: #72D4DD; }"
				"QLineEdit, QPlainTextEdit, QTreeWidget { background: #071017; color: #DDF5F7; border: 1px solid #2C4653; border-radius: 4px; padding: 6px; }"
				"QHeaderView::section { background: #142633; color: #BEE8EA; border: none; padding: 5px; }"
				"QTreeWidget::item:selected { background: #2D6071; color: #FFFFFF; }"
				"QSplitter::handle { background: #1D3340; }");

			QVBoxLayout* mainLayout = new QVBoxLayout(this);
			mainLayout->setContentsMargins(10, 10, 10, 10);
			mainLayout->setSpacing(8);

			QHBoxLayout* toolbarLayout = new QHBoxLayout();
			toolbarLayout->setSpacing(8);
			QLabel* pathLabel = new QLabel("目录：", this);
			m_dirEdit = new QLineEdit(this);
			m_dirEdit->setReadOnly(true);
			QPushButton* chooseButton = new QPushButton("选择目录", this);
			QPushButton* reloadButton = new QPushButton("重新读取", this);
			QPushButton* cameraViewButton = new QPushButton("相机视角", this);
			QPushButton* topViewButton = new QPushButton("俯视", this);
			QPushButton* frontViewButton = new QPushButton("正视", this);
			QPushButton* resetViewButton = new QPushButton("重置视图", this);
			toolbarLayout->addWidget(pathLabel);
			toolbarLayout->addWidget(m_dirEdit, 1);
			toolbarLayout->addWidget(chooseButton);
			toolbarLayout->addWidget(reloadButton);
			toolbarLayout->addSpacing(10);
			toolbarLayout->addWidget(cameraViewButton);
			toolbarLayout->addWidget(topViewButton);
			toolbarLayout->addWidget(frontViewButton);
			toolbarLayout->addWidget(resetViewButton);
			mainLayout->addLayout(toolbarLayout);

			QSplitter* centerSplitter = new QSplitter(Qt::Horizontal, this);
			centerSplitter->setChildrenCollapsible(false);
			QWidget* leftPanel = new QWidget(centerSplitter);
			QVBoxLayout* leftLayout = new QVBoxLayout(leftPanel);
			leftLayout->setContentsMargins(0, 0, 0, 0);
			leftLayout->setSpacing(8);
			QLabel* treeLabel = new QLabel("DB Tree", leftPanel);
			treeLabel->setStyleSheet("QLabel { color: #BFE8EC; font-weight: bold; }");
			m_tree = new QTreeWidget(leftPanel);
			m_tree->setHeaderLabels({ "图层" });
			m_tree->setRootIsDecorated(true);
			m_tree->setMinimumWidth(320);
			m_tree->setAlternatingRowColors(false);
			QLabel* propLabel = new QLabel("Properties", leftPanel);
			propLabel->setStyleSheet("QLabel { color: #BFE8EC; font-weight: bold; }");
			m_propertiesText = new QPlainTextEdit(leftPanel);
			m_propertiesText->setReadOnly(true);
			m_propertiesText->setMaximumBlockCount(200);
			leftLayout->addWidget(treeLabel);
			leftLayout->addWidget(m_tree, 2);
			leftLayout->addWidget(propLabel);
			leftLayout->addWidget(m_propertiesText, 1);

			// 显示设置(对选中图层实时生效)：颜色类型 / 点大小 / 连线
			QLabel* styleLabel = new QLabel("显示设置(选中图层)", leftPanel);
			styleLabel->setStyleSheet("QLabel { color: #BFE8EC; font-weight: bold; }");
			QWidget* styleBox = new QWidget(leftPanel);
			QGridLayout* styleGrid = new QGridLayout(styleBox);
			styleGrid->setContentsMargins(0, 0, 0, 0);
			styleGrid->setHorizontalSpacing(8);
			styleGrid->setVerticalSpacing(6);
			m_colorModeCombo = new QComboBox(styleBox);
			m_colorModeCombo->addItem("固定颜色");
			m_colorModeCombo->addItem("按顺序渐变");
			m_pointSizeSpin = new QDoubleSpinBox(styleBox);
			m_pointSizeSpin->setRange(0.0, 8.0);
			m_pointSizeSpin->setDecimals(1);
			m_pointSizeSpin->setSingleStep(0.5);
			m_pointSizeSpin->setSpecialValueText("自动");  // 0 → 自动(连线2.4/散点2.0)
			m_pointSizeSpin->setToolTip("点大小(px)，0=自动");
			m_connectLinesCheck = new QCheckBox("连线显示", styleBox);
			styleGrid->addWidget(new QLabel("颜色类型", styleBox), 0, 0);
			styleGrid->addWidget(m_colorModeCombo, 0, 1);
			styleGrid->addWidget(new QLabel("点大小", styleBox), 1, 0);
			styleGrid->addWidget(m_pointSizeSpin, 1, 1);
			styleGrid->addWidget(m_connectLinesCheck, 2, 0, 1, 2);
			leftLayout->addWidget(styleLabel);
			leftLayout->addWidget(styleBox);
			connect(m_colorModeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
				[this](int) { ApplyStyleToSelectedLayer(); });
			connect(m_connectLinesCheck, &QCheckBox::toggled, this,
				[this](bool) { ApplyStyleToSelectedLayer(); });
			connect(m_pointSizeSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
				[this](double) { ApplyStyleToSelectedLayer(); });
			centerSplitter->addWidget(leftPanel);

			m_view = new PointCloud3DView(centerSplitter);
			centerSplitter->addWidget(m_view);
			centerSplitter->setStretchFactor(0, 0);
			centerSplitter->setStretchFactor(1, 1);
			centerSplitter->setSizes({ 360, 1100 });
			mainLayout->addWidget(centerSplitter, 1);

			QLabel* consoleLabel = new QLabel("Console", this);
			consoleLabel->setStyleSheet("QLabel { color: #BFE8EC; font-weight: bold; }");
			m_consoleText = new QPlainTextEdit(this);
			m_consoleText->setReadOnly(true);
			m_consoleText->setMaximumHeight(110);
			m_consoleText->setMaximumBlockCount(300);
			mainLayout->addWidget(consoleLabel);
			mainLayout->addWidget(m_consoleText);

			connect(chooseButton, &QPushButton::clicked, this, [this]()
				{
					const QString selectedDir = QFileDialog::getExistingDirectory(this, "选择点云目录", m_dirEdit->text());
					if (!selectedDir.isEmpty())
					{
						m_dirEdit->setText(QDir::toNativeSeparators(selectedDir));
						LoadCurrentDirectoryAsync();
					}
				});
			connect(reloadButton, &QPushButton::clicked, this, [this]() { LoadCurrentDirectoryAsync(); });
			connect(cameraViewButton, &QPushButton::clicked, this, [this]()
				{
					if (m_view != nullptr)
					{
						m_view->SetCameraLikeView();
					}
				});
			connect(topViewButton, &QPushButton::clicked, this, [this]()
				{
					if (m_view != nullptr)
					{
						m_view->SetTopView();
					}
				});
			connect(frontViewButton, &QPushButton::clicked, this, [this]()
				{
					if (m_view != nullptr)
					{
						m_view->SetFrontView();
					}
				});
			connect(resetViewButton, &QPushButton::clicked, this, [this]()
				{
					if (m_view != nullptr)
					{
						m_view->ResetView();
					}
				});
			connect(m_tree, &QTreeWidget::itemChanged, this, [this](QTreeWidgetItem* item, int column)
				{
					Q_UNUSED(column);
					if (m_updatingTree || item == nullptr || m_view == nullptr)
					{
						return;
					}
					const int layerIndex = item->data(0, Qt::UserRole).toInt();
					m_view->SetLayerVisible(layerIndex, item->checkState(0) == Qt::Checked);
					RefreshProperties(layerIndex);
				});
			connect(m_tree, &QTreeWidget::currentItemChanged, this, [this](QTreeWidgetItem* current, QTreeWidgetItem*)
				{
					if (current == nullptr)
					{
						return;
					}
					RefreshProperties(current->data(0, Qt::UserRole).toInt());
				});
		}

		void LoadCurrentDirectoryAsync()
		{
			if (m_loading)
			{
				return;  // 上一次加载未完成（WindowModal 进度框通常已挡住重入）。
			}
			const QString dirPath = m_dirEdit != nullptr ? m_dirEdit->text().trimmed() : QString();
			QDir dir(dirPath);
			AppendLog(QString("读取点云目录：%1").arg(dirPath));
			if (!dir.exists())
			{
				AppendLog("目录不存在。");
				SetLayers({});
				return;
			}

			struct FileSpec
			{
				QString fileName;
				QString displayName;
				QColor color;
				bool rainbow = false;
				bool connectLines = false;
				double pointSize = 0.0;  // 0=自动
			};

			// 各阶段点云默认外观；完整工件大点云默认白色、小点不连线。
			const FileSpec kWorkpiece{ "PreciseLaserPoint_WorkpieceCloud.txt", "完整工件点云", QColor(255, 255, 255), false, false, 1.2 };
			const FileSpec kRaw{ "PreciseLaserPoint.txt", "原始精确点云", QColor(200, 225, 235), false, false, 2.0 };
			const FileSpec kSdkBase{ "PreciseLaserPoint_SdkBase.txt", "SDK基础焊道", QColor(255, 170, 0), false, true, 0.0 };
			const FileSpec kPreserve{ "PreciseLaserPoint_PreservePath_2mm.txt", "保留路径(2mm)", QColor(0, 210, 210), false, true, 0.0 };
			const FileSpec kClassified{ "PreciseLaserPoint_Classified.txt", "分类点云", QColor(0, 255, 80), true, true, 0.0 };
			const FileSpec kWeldPose{ "PreciseLaserPoint_WeldPose_2mm_SeamComp.txt", "焊接姿态点云", QColor(255, 230, 90), false, true, 0.0 };

			// 按当前处理方法显示该方法用到的全部数据(直到焊接姿态)；目录里缺失的文件自动跳过。
			const PointCloudProcessingConfig::Mode mode = PointCloudProcessingConfig::Load().mode;
			QVector<FileSpec> specs;
			switch (mode)
			{
			case PointCloudProcessingConfig::Mode::ExternalCorrugatedSheet:  // SDK全处理：全点云→SDK直接出拐点
				specs = { kWorkpiece, kClassified, kWeldPose };
				break;
			case PointCloudProcessingConfig::Mode::SdkBaseWeldFit:           // SDK+拟合：全点云→SDK基础焊道→拟合
				specs = { kWorkpiece, kSdkBase, kClassified, kWeldPose };
				break;
			case PointCloudProcessingConfig::Mode::CloudFit:                 // 点云+拟合：全点云投影→保留路径→拟合
				specs = { kWorkpiece, kRaw, kPreserve, kClassified, kWeldPose };
				break;
			case PointCloudProcessingConfig::Mode::LegacyLaserPath:          // 特征点+拟合：相机轨迹点→拟合
			default:
				specs = { kRaw, kClassified, kWeldPose };
				break;
			}
			AppendLog(QString("处理方法：%1").arg(PointCloudProcessingConfig::ModeDisplayName(mode)));

			// 重活搬到后台线程：逐文件读盘+解析（带进度回报/取消），完成后回 UI 线程 SetLayers。
			m_loading = true;
			EnsurePointCloudProgress();
			m_pProgress->setLabelText("正在读取点云…");
			m_pProgress->setValue(0);
			auto cancelFlag = std::make_shared<std::atomic_bool>(false);
			if (m_progressCancelConn)
			{
				disconnect(m_progressCancelConn);
			}
			m_progressCancelConn = connect(m_pProgress, &QProgressDialog::canceled, this,
				[cancelFlag]() { cancelFlag->store(true); });
			m_pProgress->show();

			auto destroyed = m_destroyed;
			auto workerCount = m_workerCount;
			workerCount->fetch_add(1);
			std::thread([this, dirPath, specs, cancelFlag, destroyed, workerCount]()
				{
					QDir dir(dirPath);
					// 先统计存在文件的总字节，用于整体进度百分比（大点云文件占绝大部分）。
					qint64 totalBytes = 0;
					for (const FileSpec& spec : specs)
					{
						const QFileInfo info(dir.filePath(spec.fileName));
						if (info.exists())
						{
							totalBytes += info.size();
						}
					}
					const qint64 totalBytesSafe = std::max<qint64>(1, totalBytes);

					QVector<PointCloud3DView::Layer> layers;
					QStringList logs;
					qint64 bytesBefore = 0;
					bool canceled = false;
					for (const FileSpec& spec : specs)
					{
						const QString filePath = QDir::toNativeSeparators(dir.filePath(spec.fileName));
						const QFileInfo info(filePath);
						if (!info.exists())
						{
							logs.append(QString("未找到：%1").arg(spec.fileName));
							continue;
						}
						const qint64 fileBytes = std::max<qint64>(1, info.size());
						const qint64 base = bytesBefore;
						const pcview::PointCloudLoadProgress progressCb =
							[this, cancelFlag, destroyed, base, fileBytes, totalBytesSafe, name = spec.displayName](int filePercent) -> bool
							{
								if (cancelFlag->load() || destroyed->load())
								{
									return false;
								}
								const qint64 done = std::min<qint64>(totalBytesSafe, base + qint64(filePercent) * fileBytes / 100);
								const int overall = static_cast<int>(done * 100 / totalBytesSafe);
								QMetaObject::invokeMethod(this, [this, overall, name]()
									{
										if (m_pProgress != nullptr)
										{
											m_pProgress->setValue(overall);
											m_pProgress->setLabelText(QString("正在读取点云：%1…").arg(name));
										}
									}, Qt::QueuedConnection);
								return true;
							};
						const LoadedPointCloudFile loaded = LoadPointCloudFile3D(filePath, progressCb);
						if (loaded.canceled)
						{
							canceled = true;
							break;
						}
						bytesBefore += info.size();
						if (!loaded.error.isEmpty())
						{
							logs.append(QString("读取失败：%1，%2").arg(spec.fileName, loaded.error));
							continue;
						}
						PointCloud3DView::Layer layer;
						layer.name = spec.displayName;
						layer.path = filePath;
						layer.points = loaded.points;
						layer.color = spec.color;
						layer.rainbow = spec.rainbow;
						layer.connectLines = spec.connectLines;
						layer.pointSize = spec.pointSize;
						layers.push_back(layer);
						logs.append(QString("已加载：%1，点数：%2，跳过行：%3")
							.arg(spec.fileName)
							.arg(loaded.points.size())
							.arg(loaded.skippedLineCount));
					}
					if (!destroyed->load())
					{
						QMetaObject::invokeMethod(this, [this, layers, logs, canceled]()
							{
								OnLayersLoaded(layers, logs, canceled);
							}, Qt::QueuedConnection);
					}
					workerCount->fetch_sub(1);
				}).detach();
		}

		void OnLayersLoaded(const QVector<PointCloud3DView::Layer>& layers, const QStringList& logs, bool canceled)
		{
			m_loading = false;
			if (m_pProgress != nullptr)
			{
				m_pProgress->hide();
			}
			for (const QString& message : logs)
			{
				AppendLog(message);
			}
			if (canceled)
			{
				AppendLog("已取消读取。");
				return;
			}
			SetLayers(layers);
		}

		void EnsurePointCloudProgress()
		{
			if (m_pProgress == nullptr)
			{
				m_pProgress = new QProgressDialog(this);
				m_pProgress->setWindowTitle("读取点云");
				m_pProgress->setWindowModality(Qt::WindowModal);
				m_pProgress->setCancelButtonText("取消");
				m_pProgress->setRange(0, 100);
				m_pProgress->setMinimumDuration(0);
				m_pProgress->setAutoClose(false);
				m_pProgress->setAutoReset(false);
			}
		}

		void SetLayers(const QVector<PointCloud3DView::Layer>& layers)
		{
			m_layers = layers;
			if (m_view != nullptr)
			{
				m_view->SetLayers(m_layers);
			}
			RefreshTree();
			RefreshProperties(m_layers.isEmpty() ? -1 : 0);
		}

		void RefreshTree()
		{
			if (m_tree == nullptr)
			{
				return;
			}
			m_updatingTree = true;
			m_tree->clear();
			for (int index = 0; index < m_layers.size(); ++index)
			{
				const PointCloud3DView::Layer& layer = m_layers.at(index);
				QTreeWidgetItem* fileItem = new QTreeWidgetItem(m_tree);
				fileItem->setText(0, QString("%1 (%2)").arg(layer.name).arg(QFileInfo(layer.path).fileName()));
				fileItem->setData(0, Qt::UserRole, index);
				fileItem->setFlags(fileItem->flags() | Qt::ItemIsUserCheckable);
				fileItem->setCheckState(0, layer.visible ? Qt::Checked : Qt::Unchecked);
				QTreeWidgetItem* childItem = new QTreeWidgetItem(fileItem);
				childItem->setText(0, QString("%1 - Cloud").arg(layer.name));
				childItem->setData(0, Qt::UserRole, index);
				childItem->setFlags(childItem->flags() | Qt::ItemIsUserCheckable);
				childItem->setCheckState(0, layer.visible ? Qt::Checked : Qt::Unchecked);
				fileItem->setExpanded(true);
			}
			m_tree->expandAll();
			if (m_tree->topLevelItemCount() > 0)
			{
				m_tree->setCurrentItem(m_tree->topLevelItem(0));
			}
			m_updatingTree = false;
		}

		void UpdatePropertiesText(int layerIndex)
		{
			if (m_propertiesText == nullptr)
			{
				return;
			}
			if (layerIndex < 0 || layerIndex >= m_layers.size())
			{
				m_propertiesText->setPlainText("未选择点云图层。");
				return;
			}
			const PointCloud3DView::Layer& layer = m_layers.at(layerIndex);
			m_propertiesText->setPlainText(QString(
				"名称：%1\n"
				"文件：%2\n"
				"点数：%3\n"
				"显示：%4\n"
				"连线：%5\n"
				"颜色：%6")
				.arg(layer.name)
				.arg(layer.path)
				.arg(layer.points.size())
				.arg(layer.visible ? "是" : "否")
				.arg(layer.connectLines ? "是" : "否")
				.arg(layer.rainbow ? "按顺序渐变" : "固定颜色"));
		}

		void RefreshProperties(int layerIndex)
		{
			m_selectedLayerIndex = layerIndex;
			UpdatePropertiesText(layerIndex);
			// 同步"显示设置"控件到选中图层(屏蔽信号避免回环)
			const bool valid = (layerIndex >= 0 && layerIndex < m_layers.size());
			m_updatingStyle = true;
			if (m_colorModeCombo != nullptr)
			{
				m_colorModeCombo->setEnabled(valid);
				if (valid) m_colorModeCombo->setCurrentIndex(m_layers.at(layerIndex).rainbow ? 1 : 0);
			}
			if (m_connectLinesCheck != nullptr)
			{
				m_connectLinesCheck->setEnabled(valid);
				if (valid) m_connectLinesCheck->setChecked(m_layers.at(layerIndex).connectLines);
			}
			if (m_pointSizeSpin != nullptr)
			{
				m_pointSizeSpin->setEnabled(valid);
				if (valid) m_pointSizeSpin->setValue(m_layers.at(layerIndex).pointSize);
			}
			m_updatingStyle = false;
		}

		// 把"显示设置"控件的值应用到当前选中图层并实时刷新(不重置相机)。
		void ApplyStyleToSelectedLayer()
		{
			if (m_updatingStyle)
			{
				return;
			}
			const int idx = m_selectedLayerIndex;
			if (idx < 0 || idx >= m_layers.size())
			{
				return;
			}
			m_layers[idx].rainbow = (m_colorModeCombo != nullptr && m_colorModeCombo->currentIndex() == 1);
			m_layers[idx].connectLines = (m_connectLinesCheck != nullptr && m_connectLinesCheck->isChecked());
			m_layers[idx].pointSize = (m_pointSizeSpin != nullptr) ? m_pointSizeSpin->value() : 0.0;
			if (m_view != nullptr)
			{
				m_view->SetLayersPreserveView(m_layers);
			}
			UpdatePropertiesText(idx);
		}

		void AppendLog(const QString& text)
		{
			if (m_consoleText == nullptr)
			{
				return;
			}
			m_consoleText->appendPlainText(QString("[%1] %2")
				.arg(QTime::currentTime().toString("HH:mm:ss"))
				.arg(text));
			m_consoleText->verticalScrollBar()->setValue(m_consoleText->verticalScrollBar()->maximum());
		}

		QString m_robotName;
		QLineEdit* m_dirEdit = nullptr;
		QTreeWidget* m_tree = nullptr;
		QPlainTextEdit* m_propertiesText = nullptr;
		QPlainTextEdit* m_consoleText = nullptr;
		PointCloud3DView* m_view = nullptr;
		QVector<PointCloud3DView::Layer> m_layers;
		bool m_updatingTree = false;
		// 左下"显示设置"控件(对选中图层实时改样式)
		QComboBox* m_colorModeCombo = nullptr;
		QCheckBox* m_connectLinesCheck = nullptr;
		QDoubleSpinBox* m_pointSizeSpin = nullptr;
		int m_selectedLayerIndex = -1;
		bool m_updatingStyle = false;
		// 后台异步读取点云：进度框 + 生命周期守卫（析构置 destroyed 并等待 worker 归零），
		// 参照 WorkpieceMeshViewerDialog 样板，避免大点云在 UI 线程同步加载卡死。
		QProgressDialog* m_pProgress = nullptr;
		bool m_loading = false;
		QMetaObject::Connection m_progressCancelConn;
		std::shared_ptr<std::atomic_bool> m_destroyed = std::make_shared<std::atomic_bool>(false);
		std::shared_ptr<std::atomic_int> m_workerCount = std::make_shared<std::atomic_int>(0);
	};

	class ConfigDatabaseViewerDialog final : public QDialog
	{
	public:
		explicit ConfigDatabaseViewerDialog(QWidget* parent = nullptr)
			: QDialog(parent)
			, m_connectionName(QString("ConfigDatabaseViewer_%1").arg(reinterpret_cast<quintptr>(this)))
		{
			setWindowTitle("配置数据库查看");
			ApplyUnifiedWindowChrome(this);
			ResizeWindowForAvailableGeometry(this, QSize(1200, 760), 0.88, 0.82);
			setStyleSheet(
				"QDialog { background: #111820; color: #ECF3F4; }"
				"QLabel { color: #C7DEE2; }"
				"QPushButton { background: #233645; color: #F5FAFA; border: 1px solid #3C6173; border-radius: 10px; padding: 8px 14px; }"
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

			QVBoxLayout* rootLayout = new QVBoxLayout(this);
			rootLayout->setContentsMargins(18, 16, 18, 18);
			rootLayout->setSpacing(10);

			QLabel* titleLabel = new QLabel("配置数据库查看", this);
			titleLabel->setStyleSheet("font-size: 24px; font-weight: bold; color: #F7FCFC;");
			rootLayout->addWidget(titleLabel);

			QHBoxLayout* pathLayout = new QHBoxLayout();
			QLabel* pathTitleLabel = new QLabel("数据库：", this);
			m_pathLabel = new QLabel(QDir::toNativeSeparators(ConfigDatabase::DatabasePath()), this);
			m_pathLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
			m_pathLabel->setStyleSheet("QLabel { color: #9ED8DB; }");
			pathLayout->addWidget(pathTitleLabel);
			pathLayout->addWidget(m_pathLabel, 1);
			rootLayout->addLayout(pathLayout);

			QHBoxLayout* toolLayout = new QHBoxLayout();
			m_viewCombo = new QComboBox(this);
			m_viewCombo->addItem("参数键值", "settings");
			m_viewCombo->addItem("数据库信息", "meta");
			m_viewCombo->setMinimumWidth(170);
			m_categoryCombo = new QComboBox(this);
			m_categoryCombo->addItem("全部分类", "");
			m_categoryCombo->addItem("全局", "全局");
			m_categoryCombo->addItem("控制单元", "控制单元");
			m_categoryCombo->addItem("工件模板", "工件模板");
			m_categoryCombo->addItem("账号", "账号");
			m_categoryCombo->addItem("结果", "结果");
			m_categoryCombo->setMinimumWidth(130);
			m_filterEdit = new QLineEdit(this);
			m_filterEdit->setPlaceholderText("按控制单元、功能参数、分组、键名或内容过滤");
			QPushButton* refreshBtn = new QPushButton("刷新", this);
			QPushButton* copyBtn = new QPushButton("复制选中行", this);
			QPushButton* closeBtn = new QPushButton("关闭", this);
			refreshBtn->setFixedWidth(110);
			copyBtn->setFixedWidth(140);
			closeBtn->setFixedWidth(110);
			toolLayout->addWidget(new QLabel("查看：", this));
			toolLayout->addWidget(m_viewCombo);
			toolLayout->addWidget(new QLabel("分类：", this));
			toolLayout->addWidget(m_categoryCombo);
			toolLayout->addWidget(m_filterEdit, 1);
			toolLayout->addWidget(refreshBtn);
			toolLayout->addWidget(copyBtn);
			toolLayout->addWidget(closeBtn);
			rootLayout->addLayout(toolLayout);

			QSplitter* splitter = new QSplitter(Qt::Vertical, this);
			splitter->setChildrenCollapsible(false);
			m_table = new QTableWidget(splitter);
			m_table->setSelectionBehavior(QAbstractItemView::SelectRows);
			m_table->setSelectionMode(QAbstractItemView::ExtendedSelection);
			m_table->setEditTriggers(QAbstractItemView::NoEditTriggers);
			m_table->setAlternatingRowColors(false);
			m_table->verticalHeader()->setVisible(false);
			m_table->horizontalHeader()->setStretchLastSection(true);
			m_detailText = new QPlainTextEdit(splitter);
			m_detailText->setReadOnly(true);
			m_detailText->document()->setMaximumBlockCount(2000);
			m_detailText->setPlaceholderText("选择一行后显示完整内容。");
			splitter->addWidget(m_table);
			splitter->addWidget(m_detailText);
			splitter->setStretchFactor(0, 4);
			splitter->setStretchFactor(1, 1);
			rootLayout->addWidget(splitter, 1);

			m_statusLabel = new QLabel(this);
			m_statusLabel->setStyleSheet("QLabel { color: #9ED8DB; }");
			rootLayout->addWidget(m_statusLabel);

			connect(m_viewCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, [this]() { Reload(); });
			connect(m_categoryCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, [this]() { ApplyFilter(); });
			connect(m_filterEdit, &QLineEdit::textChanged, this, [this]() { ApplyFilter(); });
			connect(refreshBtn, &QPushButton::clicked, this, [this]() { Reload(); });
			connect(copyBtn, &QPushButton::clicked, this, [this]() { CopySelectedRows(); });
			connect(closeBtn, &QPushButton::clicked, this, &QDialog::accept);
			connect(m_table, &QTableWidget::itemSelectionChanged, this, [this]() { UpdateDetailText(); });

			// 先显示界面，配置加载延后到 show 之后：settings(可达9000+行)走后台查询+UI分块填充+进度条，不卡死。
			QMetaObject::invokeMethod(this, [this]() { Reload(); }, Qt::QueuedConnection);
		}

		~ConfigDatabaseViewerDialog() override
		{
			// 后台查询线程仍在跑时：置销毁标记让其尽快收尾、等计数归零，再拆数据库连接。
			m_destroyed->store(true);
			while (m_workerCount->load() > 0)
			{
				std::this_thread::sleep_for(std::chrono::milliseconds(5));
			}
			if (QSqlDatabase::contains(m_connectionName))
			{
				{
					QSqlDatabase db = QSqlDatabase::database(m_connectionName, false);
					if (db.isValid())
					{
						db.close();
					}
				}
				QSqlDatabase::removeDatabase(m_connectionName);
			}
		}

	private:
		static std::string ToUtf8Std(const QString& text)
		{
			const QByteArray bytes = text.toUtf8();
			return std::string(bytes.constData(), static_cast<size_t>(bytes.size()));
		}

		static QString FromUtf8Std(const std::string& text)
		{
			return QString::fromUtf8(text.data(), static_cast<int>(text.size()));
		}

		static QString PreviewText(QString text, int maxChars = 500)
		{
			text.replace("\r\n", "\n");
			text.replace('\r', '\n');
			const bool truncated = text.size() > maxChars;
			text = text.left(maxChars);
			text.replace('\n', "  ");
			if (truncated)
			{
				text += " ...";
			}
			return text;
		}

		struct DisplayPathParts
		{
			QString category;
			QString robot;
			QString file;
		};

		struct DisplayKeyParts
		{
			QString group;
			QString name;
			QString key;
		};

		static QString CleanDisplayToken(QString text)
		{
			text = text.trimmed();
			text.replace('\\', '/');
			text.replace('.', " / ");
			text.replace('_', ' ');
			text.replace(QRegularExpression("\\s+"), " ");
			return text;
		}

		static QString ModuleFirstPart(const QString& module)
		{
			const QStringList parts = module.split('/', Qt::SkipEmptyParts);
			return parts.isEmpty() ? QString() : parts.first();
		}

		static QString ModuleRemainder(const QString& module)
		{
			const QStringList parts = module.split('/', Qt::SkipEmptyParts);
			return parts.size() <= 1 ? QString() : parts.mid(1).join(" / ");
		}

		static bool IsControlUnitRegistry(const QString& scopeType, const QString& module)
		{
			return scopeType.compare("global", Qt::CaseInsensitive) == 0
				&& ModuleFirstPart(module).compare("ControlUnits", Qt::CaseInsensitive) == 0;
		}

		static QString DisplaySettingCategory(const QString& scopeType, const QString& module)
		{
			if (IsControlUnitRegistry(scopeType, module))
			{
				return "控制单元";
			}
			if (scopeType.compare("global", Qt::CaseInsensitive) == 0)
			{
				return "全局";
			}
			if (scopeType.compare("robot", Qt::CaseInsensitive) == 0)
			{
				return "控制单元";
			}
			if (scopeType.compare("workpiece_template", Qt::CaseInsensitive) == 0)
			{
				return "工件模板";
			}
			if (scopeType.compare("account", Qt::CaseInsensitive) == 0)
			{
				return "账号";
			}
			if (scopeType.compare("result", Qt::CaseInsensitive) == 0)
			{
				return "结果";
			}
			return CleanDisplayToken(scopeType);
		}

		static QString DisplaySettingObject(const QString& scopeType, const QString& scopeId, const QString& module)
		{
			if (IsControlUnitRegistry(scopeType, module))
			{
				const QString remainder = ModuleRemainder(module);
				return remainder.isEmpty() ? "控制单元列表" : CleanDisplayToken(remainder);
			}
			if (scopeType.compare("global", Qt::CaseInsensitive) == 0)
			{
				return "全局";
			}
			return scopeId.trimmed().isEmpty() ? "-" : CleanDisplayToken(scopeId);
		}

		static QString DisplayFunctionalModule(const QString& scopeType, const QString& module)
		{
			if (IsControlUnitRegistry(scopeType, module))
			{
				return "控制单元列表";
			}

			const QString first = ModuleFirstPart(module);
			if (first.compare("RobotPara", Qt::CaseInsensitive) == 0) return "机器人参数";
			if (first.compare("CameraParam", Qt::CaseInsensitive) == 0) return "相机参数";
			if (first.compare("HandEyeMatrix", Qt::CaseInsensitive) == 0) return "手眼矩阵";
			if (first.compare("HandEyeCalibration", Qt::CaseInsensitive) == 0) return "手眼标定";
			if (first.compare("LineScanParam", Qt::CaseInsensitive) == 0) return "线扫参数";
			if (first.compare("LineCoarseScanParam", Qt::CaseInsensitive) == 0) return "线扫粗定位参数";
			if (first.compare("MeasureWeldParam", Qt::CaseInsensitive) == 0) return "测量焊接参数";
			if (first.compare("WeldPoseCompParam", Qt::CaseInsensitive) == 0) return "姿态补偿参数";
			if (first.compare("WeldSeamCompParam", Qt::CaseInsensitive) == 0) return "焊道补偿参数";
			if (first.compare("WeldProcess", Qt::CaseInsensitive) == 0) return "焊接工艺参数";
			if (first.compare("DashboardTools", Qt::CaseInsensitive) == 0) return "管理页工具";
			if (first.compare("PointCloudProcessing", Qt::CaseInsensitive) == 0) return "精测点云处理";
			if (first.compare("MeasureThenWeld", Qt::CaseInsensitive) == 0) return "先测后焊";
			if (first.compare("TouchKeyboard", Qt::CaseInsensitive) == 0) return "触摸键盘";
			if (first.compare("RobotJog", Qt::CaseInsensitive) == 0) return "机器人点动";
			if (first.compare("Auth", Qt::CaseInsensitive) == 0) return "账号权限";
			if (first.compare("Camera", Qt::CaseInsensitive) == 0) return "相机状态";
			return first.isEmpty() ? "未分类参数" : CleanDisplayToken(first);
		}

		static QString DisplayParameterGroup(const QString& scopeType, const QString& module)
		{
			if (IsControlUnitRegistry(scopeType, module))
			{
				return "基础";
			}
			const QString remainder = ModuleRemainder(module);
			return remainder.isEmpty() ? "基础" : CleanDisplayToken(remainder);
		}

		static QString DisplayCategory(QString rootName, bool hasRobot)
		{
			if (rootName.compare("Data", Qt::CaseInsensitive) == 0)
			{
				return hasRobot ? "机器人数据" : "全局数据";
			}
			if (rootName.compare("Result", Qt::CaseInsensitive) == 0)
			{
				return "结果数据";
			}
			return CleanDisplayToken(rootName);
		}

		static DisplayPathParts BuildDisplayPathParts(const QString& filePath)
		{
			QString normalized = filePath;
			normalized.replace('\\', '/');
			const QStringList parts = normalized.split('/', Qt::SkipEmptyParts);

			DisplayPathParts display;
			const auto robotIt = std::find_if(parts.cbegin(), parts.cend(), [](const QString& part)
				{
					return part.startsWith("Robot", Qt::CaseInsensitive);
				});
			const bool hasRobot = robotIt != parts.cend();
			display.category = !parts.isEmpty() ? DisplayCategory(parts.first(), hasRobot) : "配置数据";
			display.robot = hasRobot ? CleanDisplayToken(*robotIt) : "";

			QFileInfo fileInfo(normalized);
			display.file = CleanDisplayToken(fileInfo.completeBaseName());
			if (display.file.isEmpty() && !parts.isEmpty())
			{
				display.file = CleanDisplayToken(parts.last());
			}
			return display;
		}

		static DisplayKeyParts BuildDisplayKeyParts(const QString& section, const QString& key)
		{
			QString normalizedSection = section;
			normalizedSection.replace('\\', '/');
			const QStringList sectionParts = normalizedSection.split('/', Qt::SkipEmptyParts);

			DisplayKeyParts display;
			display.group = !sectionParts.isEmpty() ? CleanDisplayToken(sectionParts.first()) : "默认";
			if (sectionParts.size() > 1)
			{
				display.name = CleanDisplayToken(sectionParts.mid(1).join(" / "));
			}

			QString normalizedKey = key;
			normalizedKey.replace('\\', '/');
			const QStringList keyParts = normalizedKey.split('/', Qt::SkipEmptyParts);
			if (keyParts.size() > 1)
			{
				if (display.name.isEmpty())
				{
					display.name = CleanDisplayToken(keyParts.mid(0, keyParts.size() - 1).join(" / "));
				}
				else
				{
					display.name += " / " + CleanDisplayToken(keyParts.mid(0, keyParts.size() - 1).join(" / "));
				}
				display.key = CleanDisplayToken(keyParts.last());
			}
			else
			{
				display.key = CleanDisplayToken(key);
				const QRegularExpression numericSuffixPattern("^(.+)_([0-9]+)$");
				const QRegularExpressionMatch match = numericSuffixPattern.match(key);
				if (match.hasMatch())
				{
					if (display.name.isEmpty())
					{
						display.name = CleanDisplayToken(match.captured(1));
					}
					display.key = CleanDisplayToken(match.captured(2));
				}
			}

			return display;
		}

		bool OpenDatabase(QString* error)
		{
			if (!QFileInfo::exists(ConfigDatabase::DatabasePath()))
			{
				if (error != nullptr)
				{
					*error = "数据库文件不存在：" + QDir::toNativeSeparators(ConfigDatabase::DatabasePath());
				}
				return false;
			}

			if (!ConfigDatabase::IsAvailable())
			{
				if (error != nullptr)
				{
					*error = "数据库结构无效，请先用迁移工具重新生成："
						+ QDir::toNativeSeparators(ConfigDatabase::DatabasePath());
				}
				return false;
			}

			if (!QSqlDatabase::contains(m_connectionName))
			{
				QSqlDatabase db = QSqlDatabase::addDatabase("QSQLITE", m_connectionName);
				db.setDatabaseName(ConfigDatabase::DatabasePath());
				db.setConnectOptions("QSQLITE_OPEN_READONLY");
			}

			QSqlDatabase db = QSqlDatabase::database(m_connectionName);
			if (!db.isOpen() && !db.open())
			{
				if (error != nullptr)
				{
					*error = "打开数据库失败：" + db.lastError().text();
				}
				return false;
			}

			QSqlQuery pragma(db);
			pragma.exec("PRAGMA busy_timeout=2000");
			return true;
		}

		void SetHeaders(const QStringList& headers)
		{
			m_table->clear();
			m_table->setColumnCount(headers.size());
			m_table->setHorizontalHeaderLabels(headers);
			m_table->setRowCount(0);
		}

		void AddRow(const QStringList& cells, const QString& detail)
		{
			const int row = m_table->rowCount();
			m_table->insertRow(row);
			for (int column = 0; column < cells.size(); ++column)
			{
				QTableWidgetItem* item = new QTableWidgetItem(cells.at(column));
				item->setFlags(item->flags() & ~Qt::ItemIsEditable);
				if (column == 0)
				{
					item->setData(Qt::UserRole, detail);
				}
				m_table->setItem(row, column, item);
			}
		}

		void Reload()
		{
			if (m_loading)
			{
				return;  // 后台加载进行中，避免重入
			}
			QString error;
			if (!OpenDatabase(&error))
			{
				SetHeaders({ "错误" });
				AddRow({ error }, error);
				m_statusLabel->setText(error);
				return;
			}

			const QString mode = m_viewCombo != nullptr ? m_viewCombo->currentData().toString() : "settings";
			if (mode == "meta")
			{
				if (m_categoryCombo != nullptr)
				{
					m_categoryCombo->setEnabled(false);
				}
				LoadMeta();
				ApplyFilter();
				UpdateDetailText();
				return;
			}
			if (m_categoryCombo != nullptr)
			{
				m_categoryCombo->setEnabled(true);
			}
			// settings 表可达 9000+ 行：后台线程查询/提取，UI 分块填充 + 进度条；完成后在回调里 ApplyFilter/UpdateDetailText。
			LoadSettingsAsync();
		}

		struct ConfigRowData
		{
			QStringList cells;
			QString detail;
		};

		void EnsureProgress()
		{
			if (m_pProgress == nullptr)
			{
				m_pProgress = new QProgressDialog(this);
				m_pProgress->setWindowTitle("加载配置");
				m_pProgress->setWindowModality(Qt::WindowModal);
				m_pProgress->setCancelButton(nullptr);  // 加载有界、不提供取消
				m_pProgress->setRange(0, 100);
				m_pProgress->setMinimumDuration(0);
				m_pProgress->setAutoClose(false);
				m_pProgress->setAutoReset(false);
			}
		}

		// settings 表可达 9000+ 行：后台线程(独立只读连接)查询并构造行数据，回 UI 线程分块填充表格 + 进度条。
		void LoadSettingsAsync()
		{
			SetHeaders({ "分类", "控制单元/对象", "功能参数", "参数分组", "参数名", "值", "类型", "敏感", "加密", "更新时间" });
			m_loading = true;
			EnsureProgress();
			m_pProgress->setLabelText("正在后台读取配置参数…");
			m_pProgress->setValue(0);
			m_pProgress->show();

			// QSqlDatabase 有线程亲和性：后台线程用独立连接名，不复用 UI 线程的 m_connectionName。
			const QString dbPath = ConfigDatabase::DatabasePath();
			const QString workerConn = m_connectionName + "_worker";
			auto destroyed = m_destroyed;
			auto workerCount = m_workerCount;
			workerCount->fetch_add(1);
			std::thread([this, dbPath, workerConn, destroyed, workerCount]()
				{
					QVector<ConfigRowData> rows;
					QString err;
					{
						QSqlDatabase db = QSqlDatabase::addDatabase("QSQLITE", workerConn);
						db.setDatabaseName(dbPath);
						db.setConnectOptions("QSQLITE_OPEN_READONLY");
						if (db.open())
						{
							QSqlQuery query(db);
							query.prepare(
								"SELECT scope_type, scope_id, module, key_name, value_text, value_type, sensitive, encrypted, updated_at "
								"FROM settings "
								"ORDER BY CASE scope_type "
								"WHEN 'global' THEN 0 WHEN 'robot' THEN 1 WHEN 'workpiece_template' THEN 2 "
								"WHEN 'account' THEN 3 WHEN 'result' THEN 4 ELSE 9 END, "
								"scope_id COLLATE NOCASE, module COLLATE NOCASE, key_name COLLATE NOCASE");
							if (query.exec())
							{
								rows.reserve(4096);
								while (query.next())
								{
									if (destroyed->load())
									{
										break;
									}
									const QString scopeType = query.value(0).toString();
									const QString scopeId = query.value(1).toString();
									const QString module = query.value(2).toString();
									const QString keyName = query.value(3).toString();
									const QString category = DisplaySettingCategory(scopeType, module);
									const QString objectName = DisplaySettingObject(scopeType, scopeId, module);
									const QString functionName = DisplayFunctionalModule(scopeType, module);
									const QString groupName = DisplayParameterGroup(scopeType, module);
									const QString displayKey = CleanDisplayToken(keyName);
									const bool sensitive = query.value(6).toInt() != 0;
									const bool encrypted = query.value(7).toInt() != 0;
									const QString updatedAt = query.value(8).toString();
									const QString value = (sensitive || encrypted)
										? QStringLiteral("<敏感值已隐藏>")
										: query.value(4).toString();
									const QString valueType = query.value(5).toString();
									const QString detail = QString(
										"分类：%1\n控制单元/对象：%2\n功能参数：%3\n参数分组：%4\n参数名：%5\n\n"
										"原始作用域：%6\n原始对象：%7\n原始模块：%8\n原始键名：%9\n类型：%10\n敏感：%11\n加密：%12\n更新时间：%13\n\n值：\n%14")
										.arg(category, objectName, functionName, groupName, displayKey,
											scopeType, scopeId, module, keyName, valueType,
											QString(sensitive ? "是" : "否"), QString(encrypted ? "是" : "否"), updatedAt, value);
									ConfigRowData rd;
									rd.cells = QStringList{ category, objectName, functionName, groupName, displayKey,
										PreviewText(value), valueType, sensitive ? "是" : "否", encrypted ? "是" : "否", updatedAt };
									rd.detail = detail;
									rows.push_back(rd);
								}
							}
							else
							{
								err = query.lastError().text();
							}
							db.close();
						}
						else
						{
							err = db.lastError().text();
						}
					}
					QSqlDatabase::removeDatabase(workerConn);
					if (!destroyed->load())
					{
						QMetaObject::invokeMethod(this, [this, rows, err]()
							{
								OnSettingsRowsLoaded(rows, err);
							}, Qt::QueuedConnection);
					}
					workerCount->fetch_sub(1);
				}).detach();
		}

		void OnSettingsRowsLoaded(const QVector<ConfigRowData>& rows, const QString& error)
		{
			if (!error.isEmpty())
			{
				m_loading = false;
				if (m_pProgress != nullptr)
				{
					m_pProgress->hide();
				}
				AddRow({ "读取参数键值失败：" + error, "", "", "", "", "", "", "", "", "" }, error);
				m_statusLabel->setText("读取参数键值失败：" + error);
				return;
			}
			// O(N) 填充：关排序/更新 + 预分配行数 + 按行索引 setItem（不用 insertRow，避免逐行重排的 O(N²)），分块刷进度。
			const int total = rows.size();
			m_table->setSortingEnabled(false);
			m_table->setUpdatesEnabled(false);
			m_table->setRowCount(total);
			for (int row = 0; row < total; ++row)
			{
				const ConfigRowData& rd = rows.at(row);
				for (int column = 0; column < rd.cells.size(); ++column)
				{
					QTableWidgetItem* item = new QTableWidgetItem(rd.cells.at(column));
					item->setFlags(item->flags() & ~Qt::ItemIsEditable);
					if (column == 0)
					{
						item->setData(Qt::UserRole, rd.detail);
					}
					m_table->setItem(row, column, item);
				}
				if ((row & 0x3FF) == 0)
				{
					if (m_pProgress != nullptr)
					{
						m_pProgress->setValue(static_cast<int>(qint64(row) * 100 / qMax(1, total)));
					}
					QApplication::processEvents(QEventLoop::ExcludeUserInputEvents);
				}
			}
			m_table->setUpdatesEnabled(true);
			if (m_pProgress != nullptr)
			{
				m_pProgress->setValue(100);
				m_pProgress->hide();
			}
			m_loading = false;
			FinishTable(QString("参数键值：%1 条。").arg(total));
			ApplyFilter();
			UpdateDetailText();
		}

		void LoadMeta()
		{
			SetHeaders({ "键", "值" });
			QSqlDatabase db = QSqlDatabase::database(m_connectionName);
			QSqlQuery query(db);
			query.prepare("SELECT key, value FROM meta ORDER BY key COLLATE NOCASE");
			if (!query.exec())
			{
				const QString error = "读取数据库信息失败：" + query.lastError().text();
				AddRow({ error, "" }, error);
				m_statusLabel->setText(error);
				return;
			}

			int rowCount = 0;
			while (query.next())
			{
				const QString key = query.value(0).toString();
				const QString value = query.value(1).toString();
				AddRow({ key, value }, QString("键：%1\n值：%2").arg(key, value));
				++rowCount;
			}
			FinishTable(QString("数据库信息：%1 条。").arg(rowCount));
		}

		void FinishTable(const QString& status)
		{
			m_table->resizeColumnsToContents();
			m_table->horizontalHeader()->setStretchLastSection(true);
			if (m_table->rowCount() > 0)
			{
				m_table->selectRow(0);
			}
			m_statusLabel->setText(status);
		}

		void ApplyFilter()
		{
			if (m_table == nullptr || m_filterEdit == nullptr)
			{
				return;
			}
			const QString filter = m_filterEdit->text().trimmed();
			const bool hasCategoryColumn = m_table->columnCount() > 0
				&& m_table->horizontalHeaderItem(0) != nullptr
				&& m_table->horizontalHeaderItem(0)->text() == "分类";
			const QString categoryFilter = (hasCategoryColumn && m_categoryCombo != nullptr)
				? m_categoryCombo->currentData().toString()
				: QString();
			int visibleCount = 0;
			for (int row = 0; row < m_table->rowCount(); ++row)
			{
				if (!categoryFilter.isEmpty() && m_table->columnCount() > 0)
				{
					QTableWidgetItem* categoryItem = m_table->item(row, 0);
					if (categoryItem == nullptr || categoryItem->text() != categoryFilter)
					{
						m_table->setRowHidden(row, true);
						continue;
					}
				}
				bool match = filter.isEmpty();
				for (int column = 0; !match && column < m_table->columnCount(); ++column)
				{
					QTableWidgetItem* item = m_table->item(row, column);
					if (item == nullptr)
					{
						continue;
					}
					if (item->text().contains(filter, Qt::CaseInsensitive)
						|| item->data(Qt::UserRole).toString().contains(filter, Qt::CaseInsensitive))
					{
						match = true;
					}
				}
				m_table->setRowHidden(row, !match);
				if (match)
				{
					++visibleCount;
				}
			}
			if (!filter.isEmpty() || !categoryFilter.isEmpty())
			{
				m_statusLabel->setText(QString("过滤后显示：%1 / %2。").arg(visibleCount).arg(m_table->rowCount()));
			}
		}

		void UpdateDetailText()
		{
			if (m_table == nullptr || m_detailText == nullptr)
			{
				return;
			}
			const int row = m_table->currentRow();
			if (row < 0)
			{
				m_detailText->clear();
				return;
			}
			QTableWidgetItem* item = m_table->item(row, 0);
			m_detailText->setPlainText(item != nullptr ? item->data(Qt::UserRole).toString() : QString());
		}

		void CopySelectedRows()
		{
			if (m_table == nullptr)
			{
				return;
			}
			QSet<int> selectedRows;
			for (const QTableWidgetSelectionRange& range : m_table->selectedRanges())
			{
				for (int row = range.topRow(); row <= range.bottomRow(); ++row)
				{
					if (!m_table->isRowHidden(row))
					{
						selectedRows.insert(row);
					}
				}
			}
			if (selectedRows.isEmpty() && m_table->currentRow() >= 0)
			{
				selectedRows.insert(m_table->currentRow());
			}

			QList<int> rows = selectedRows.values();
			std::sort(rows.begin(), rows.end());
			QStringList lines;
			QStringList headers;
			for (int column = 0; column < m_table->columnCount(); ++column)
			{
				headers << m_table->horizontalHeaderItem(column)->text();
			}
			lines << headers.join('\t');
			for (int row : rows)
			{
				QStringList cells;
				for (int column = 0; column < m_table->columnCount(); ++column)
				{
					QTableWidgetItem* item = m_table->item(row, column);
					cells << (item != nullptr ? item->text() : QString());
				}
				lines << cells.join('\t');
			}
			QApplication::clipboard()->setText(lines.join('\n'));
			m_statusLabel->setText(QString("已复制 %1 行。").arg(rows.size()));
		}

		QString m_connectionName;
		QComboBox* m_viewCombo = nullptr;
		QComboBox* m_categoryCombo = nullptr;
		QLineEdit* m_filterEdit = nullptr;
		QTableWidget* m_table = nullptr;
		QPlainTextEdit* m_detailText = nullptr;
		QLabel* m_statusLabel = nullptr;
		QLabel* m_pathLabel = nullptr;
		// 后台异步加载 settings(9000+行) + 真实进度条；析构等 worker 归零守卫，杜绝悬垂 this/连接。
		QProgressDialog* m_pProgress = nullptr;
		bool m_loading = false;
		std::shared_ptr<std::atomic_bool> m_destroyed = std::make_shared<std::atomic_bool>(false);
		std::shared_ptr<std::atomic_int> m_workerCount = std::make_shared<std::atomic_int>(0);
	};
}

struct QtWidgetsApplication4::CameraRuntime
{
	// TCP 独立模式的数据接收 worker。相机升级到 "SKJF" 协议后改用厂商 SKJCamera SDK 接收
	// （ScanCameraSkjWorker，历史变量名沿用 tcpWorker）；旧的自解析 ScanCameraTcpClientWorker 已不再使用。
	ScanCameraSkjWorker* tcpWorker = nullptr;
	ClientUDPFormSensorWorker* udpWorker = nullptr;
	QThread* thread = nullptr;
	CameraFrameCache* cache = nullptr;
	QString cameraIP;
	int cameraPort = 0;
	int cameraPollIntervalMs = 10;  // 已下发给 worker 的取帧轮询间隔；变化时需重启 worker 才生效
	QString receiveMode;
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
	, m_pFtpJobManagementPage(nullptr)
	, m_pConfigDatabaseViewerPage(nullptr)
	, m_pPrecisePointCloudProcessingPage(nullptr)
	, m_pRobotSelectorCombo(nullptr)
	, m_pRobotSelectorLabel(nullptr)
	, m_nCurrentRobotUnitIndex(-1)
	, m_nWeldProcessPageUnitIndex(-1)
	, m_nFunctionTestPageUnitIndex(-1)
	, m_nMeasureThenWeldPageUnitIndex(-1)
	, m_nRobotJogPageUnitIndex(-1)
	, m_nFtpJobManagementPageUnitIndex(-1)
	, m_pRobotLogText(nullptr)
	, m_pGroovePointCloudDialog(nullptr)
	, m_pPointCloudViewerDialog(nullptr)
	, m_pCurrentUserButton(nullptr)
	, m_pManagementUserLabel(nullptr)
	, m_pPermissionHintLabel(nullptr)
	, m_pAccountManagementAction(nullptr)
	, m_pManagementCameraReceiveModeBtn(nullptr)
	, m_pScanTimestampSourceCombo(nullptr)
	, m_pStepSdkInterfaceModeCombo(nullptr)
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
	, m_pDashboardEmergencyStopBtn(nullptr)
	, m_pDashboardModeBtn(nullptr)
	, m_pDashboardDebugLogBtn(nullptr)
	, m_pDashboardToolPanel(nullptr)
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
	, m_skjCameraControlClient(nullptr)
	, m_bFanucMovlForward(true)
	, m_bFanucMovlRunning(false)
	, m_bFanucMovjRunning(false)
	, m_bFanucMoveZeroRunning(false)
	, m_sCurrentUserName(QStringLiteral("访客"))
	, m_sCurrentUserRole(kRoleOperator)
	, m_sAuthHintOverride()
	, m_bAuthRegisterMode(false)
	, m_bEnforceInteractiveSessionLeaseGate(
		!QCoreApplication::arguments().contains(QStringLiteral("--no-show")))
	, m_bOpenEmbeddedInManagement(false)
	, m_bPendingOpenManagementAfterLogin(false)
	, m_bDebugLogMode(false)
	, m_bUseSharedScanCameraReceiver(false)
	, m_bFanucMonitorReading(false)
{
	if (m_bEnforceInteractiveSessionLeaseGate)
	{
		RobotOperationLease::SetNewOperationsAllowed(
			false,
			QStringLiteral("请先完成有效的本地账号登录，再启动机器人硬件操作。"));
	}
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
		ui.GrooveCameraText->hide();
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
	// 在线服务：启动后恢复上传队列（自动上传开启且有历史待传项时续传——含 CLI 上轮没传完的）。
	QTimer::singleShot(3000, this, [this]()
		{
			if (OnlineServicesConfig::AutoUploadEnabled())
			{
				ScanDataUploader* uploader = EnsureScanDataUploader();
				if (!uploader->PendingList().isEmpty())
				{
					uploader->TriggerUploadNow();
				}
			}
		});
	setMinimumSize(640, 480);
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
	// 主页不再重复显示大标题（与顶部窗口标题栏重复）；版本徽章可点击 → 「关于」（当前/最新版本 + 在线升级）。
	QPushButton* versionLabel = new QPushButton(QString("v%1").arg(BuildAppVersionText()), m_pDashboardPage);
	versionLabel->setCursor(Qt::PointingHandCursor);
	versionLabel->setToolTip("点击查看关于与在线更新");
	versionLabel->setStyleSheet(
		"QPushButton { background: #173041; color: #9ED8DB; border: 1px solid #3C6173; "
		"border-radius: 10px; padding: 4px 10px; font-size: 13px; font-weight: bold; }"
		"QPushButton:hover { border-color: #72D4DD; background: #1D3C51; }"
		"QPushButton:pressed { background: #122636; }");
	connect(versionLabel, &QPushButton::clicked, this, [this]() { OpenAboutDialog(); });
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

	auto makeToolButton = [](const QString& id, const QString& text, QWidget* parent) -> DashboardToolButton*
		{
			DashboardToolButton* button = new DashboardToolButton(id, text, parent);
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
	QPushButton* quickTeachPositionBtn = makeLargeButton("扫描位置示教\n下枪/起点/终点/收枪", quickGroup);
	QPushButton* quickCalibrationBtn = makeLargeButton("标定与相机参数\n手眼标定、矩阵和相机配置", quickGroup);
	m_pDashboardEmergencyStopBtn = new QPushButton("安全停止所有本软件活动机器人任务（不可从主页删除）", quickGroup);
	m_pDashboardEmergencyStopBtn->setMinimumHeight(58);
	m_pDashboardEmergencyStopBtn->setStyleSheet(
		"QPushButton { background: #8F1D24; color: white; border: 2px solid #FF7B82; border-radius: 12px; "
		"font-size: 18px; font-weight: 800; padding: 10px 18px; }"
		"QPushButton:hover { background: #B3262E; border-color: #FFB2B6; }"
		"QPushButton:pressed { background: #641318; }");
	m_pDashboardEmergencyStopBtn->setToolTip(
		"锁存取消本软件持有租约的全部机器人流程，并由机器人侧真实终止/卸载其程序；"
		"停机未确认时继续闭锁。它不是控制柜急停，不能替代示教器或外部安全回路。");
	quickLayout->addWidget(quickMeasureBtn, 0, 0);
	quickLayout->addWidget(quickJogBtn, 0, 1);
	quickLayout->addWidget(quickTeachPositionBtn, 1, 0);
	quickLayout->addWidget(quickCalibrationBtn, 1, 1);
	quickLayout->addWidget(m_pDashboardEmergencyStopBtn, 2, 0, 1, 2);
	dashboardActionLayout->addWidget(quickGroup, 1);

	QGroupBox* toolGroup = new QGroupBox("现场小工具", m_pDashboardPage);
	toolGroup->setMinimumWidth(330);
	toolGroup->setMaximumWidth(390);
	QVBoxLayout* toolLayout = new QVBoxLayout(toolGroup);
	toolLayout->setSpacing(10);
	DashboardToolPanel* dashboardToolPanel = new DashboardToolPanel(toolGroup);
	m_pDashboardToolPanel = dashboardToolPanel;
	toolLayout->addWidget(dashboardToolPanel, 1);
	m_pDashboardConnectBtn = makeToolButton("connect", "连接服务", dashboardToolPanel);
	QPushButton* quickPositionBtn = makeToolButton("currentPosition", "读取当前位置", dashboardToolPanel);
	QPushButton* quickPreviewBtn = makeToolButton("groovePreview", "坡口相机预览", dashboardToolPanel);
	m_pDashboardClearAlarmBtn = makeToolButton("clearAlarm", "清除报警", dashboardToolPanel);
	m_pDashboardModeBtn = makeToolButton("stepMode", "STEP 模式", dashboardToolPanel);
	QPushButton* quickReadTool1Btn = makeToolButton("readTool1", "读取Tool1", dashboardToolPanel);
	QPushButton* quickPointCloudBtn = makeToolButton("pointCloudViewer", "点云查看", dashboardToolPanel);
	m_pDashboardDebugLogBtn = makeToolButton("debugLog", "显示调试日志", dashboardToolPanel);
	const QList<QPair<QString, QString>> functionToolSpecs = {
		{ "setSpeed", "设置速度" },
		{ "getPulse", "读取关节脉冲" },
		{ "checkDone", "检查运行完成" },
		{ "setGetInt", "写读INT寄存器" },
		{ "callJob", "调用任务" },
		{ "uploadLs", "发送FANUC LS" },
		{ "curposDiagnostic", "FANUC CURPOS诊断" },
		{ "timestampDiagnostic", "机器人+相机时间戳" },
		{ "movlTest", "MOVL往返测试" },
		{ "movjTest", "MOVJ J2/J3 +5deg" },
		{ "moveZero", "运动到零位" },
		{ "captureKinematics", "保存关节+直角" },
		{ "fitDh", "拟合DH参数" },
		{ "currentFrameFilter", "当前帧点云滤波" }
	};
	QHash<QString, QPushButton*> functionToolButtons;
	for (const QPair<QString, QString>& spec : functionToolSpecs)
	{
		const QString toolId = QString("function.%1").arg(spec.first);
		functionToolButtons.insert(spec.first, makeToolButton(toolId, spec.second, dashboardToolPanel));
	}
	for (const QString& fanucOnlyId : { QString("uploadLs"), QString("curposDiagnostic") })
	{
		if (QPushButton* button = functionToolButtons.value(fanucOnlyId, nullptr))
		{
			button->setProperty("dashboardToolRequiredRobotScope", "fanuc");
		}
	}
	quickPreviewBtn->setCheckable(true);
	m_pDashboardDebugLogBtn->setCheckable(true);
	dashboardToolPanel->RegisterTool(static_cast<DashboardToolButton*>(m_pDashboardConnectBtn));
	dashboardToolPanel->RegisterTool(static_cast<DashboardToolButton*>(quickPositionBtn));
	dashboardToolPanel->RegisterTool(static_cast<DashboardToolButton*>(quickPreviewBtn));
	dashboardToolPanel->RegisterTool(static_cast<DashboardToolButton*>(m_pDashboardClearAlarmBtn));
	dashboardToolPanel->RegisterTool(static_cast<DashboardToolButton*>(m_pDashboardModeBtn));
	dashboardToolPanel->RegisterTool(static_cast<DashboardToolButton*>(quickReadTool1Btn));
	dashboardToolPanel->RegisterTool(static_cast<DashboardToolButton*>(quickPointCloudBtn));
	dashboardToolPanel->RegisterTool(static_cast<DashboardToolButton*>(m_pDashboardDebugLogBtn));
	for (QPushButton* button : functionToolButtons)
	{
		dashboardToolPanel->RegisterTool(static_cast<DashboardToolButton*>(button));
	}
	dashboardToolPanel->SetDefaultOrder({
		"connect",
		"currentPosition",
		"groovePreview",
		"clearAlarm",
		"stepMode",
		"readTool1",
		"pointCloudViewer",
		"debugLog"
		});
	QStringList dashboardToolCandidates = {
		"connect",
		"currentPosition",
		"groovePreview",
		"clearAlarm",
		"stepMode",
		"readTool1",
		"pointCloudViewer",
		"debugLog"
	};
	for (const QPair<QString, QString>& spec : functionToolSpecs)
	{
		dashboardToolCandidates.push_back(QString("function.%1").arg(spec.first));
	}
	dashboardToolPanel->SetCandidateOrder(dashboardToolCandidates);
	m_pDashboardDebugLogBtn->hide();
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
		"QPushButton:checked { background: #2F6F7A; border-color: #8EE7EC; }"
		"QLineEdit, QPlainTextEdit { background: #081018; color: #ECF3F4; border: 1px solid #2C4653; border-radius: 8px; padding: 6px 8px; }"
		"QComboBox { background: #000000; color: #ECF3F4; border: 1px solid #2C4653; border-radius: 0px; padding: 6px 34px 6px 8px; }"
		"QComboBox::drop-down { border-left: 1px solid #2C4653; border-radius: 0px; width: 28px; background: #000000; }"
		"QComboBox::down-arrow { image: url(:/QtWidgetsApplication4/icons/chevron-down.svg); width: 12px; height: 8px; }"
		"QComboBox QAbstractItemView { background: #000000; color: #ECF3F4; selection-background-color: #2D5465; border: 1px solid #2C4653; border-radius: 0px; outline: 0px; }"
		"QMenuBar#ManagementMenuBar { background: #0B1117; color: #ECF3F4; border: 1px solid #223743; border-radius: 0px; padding: 3px 6px; }"
		"QMenuBar#ManagementMenuBar::item { spacing: 4px; padding: 7px 14px; border-radius: 0px; background: transparent; }"
		"QMenuBar#ManagementMenuBar::item:selected { background: #1C3543; color: #FFFFFF; }"
		"QMenuBar#ManagementMenuBar::item:pressed { background: #244A58; }"
		"QMenu { background: #0D161E; color: #ECF3F4; border: 1px solid #2C4653; padding: 5px; }"
		"QMenu::item { padding: 8px 32px 8px 18px; min-width: 132px; }"
		"QMenu::item:selected { background: #244A58; color: #FFFFFF; }"
		"QMenu::item:disabled { color: #617884; }"
		"QMenu::separator { height: 1px; background: #263E4A; margin: 5px 8px; }");
	QVBoxLayout* managementLayout = new QVBoxLayout(m_pManagementPage);
	managementLayout->setContentsMargins(18, 12, 18, 18);
	managementLayout->setSpacing(10);

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

	QMenuBar* managementMenuBar = new QMenuBar(m_pManagementPage);
	managementMenuBar->setObjectName("ManagementMenuBar");
	managementLayout->setMenuBar(managementMenuBar);

	auto createManagementAction = [this](const QString& text, const std::function<void()>& handler) -> QAction*
		{
			QAction* action = new QAction(text, m_pManagementPage);
			connect(action, &QAction::triggered, this, [handler]()
				{
					if (handler)
					{
						handler();
					}
				});
			return action;
		};
	auto addMenuAction = [](QMenu* menu, QAction* action) -> QAction*
		{
			if (menu != nullptr && action != nullptr)
			{
				menu->addAction(action);
			}
			return action;
		};

	QMenu* managementHomeMenu = managementMenuBar->addMenu("主页");
	QMenu* managementRobotMenu = managementMenuBar->addMenu("机器人");
	QMenu* managementProcessMenu = managementMenuBar->addMenu("工艺");
	QMenu* managementCameraMenu = managementMenuBar->addMenu("相机");
	QMenu* managementDebugMenu = managementMenuBar->addMenu("调试");
	QMenu* managementAccountMenu = managementMenuBar->addMenu("账号");

	addMenuAction(managementHomeMenu, createManagementAction("管理首页", [this]() { ShowManagementHomePage(); }));
	addMenuAction(managementHomeMenu, createManagementAction("返回主界面", [this]() { ShowDashboardPage(); }));
	managementHomeMenu->addSeparator();
	addMenuAction(managementHomeMenu, createManagementAction("关闭管理界面", [this]() { if (m_pManagementPage != nullptr) { m_pManagementPage->hide(); } }));

	addMenuAction(managementRobotMenu, createManagementAction("控制单元管理", [this]() { OpenControlUnitManagementDialog(); }));
	addMenuAction(managementRobotMenu, createManagementAction("FTP Job 文件", [this]() { OpenFtpJobManagementDialog(); }));

	addMenuAction(managementProcessMenu, createManagementAction("工艺参数", [this, openInManagement]() { openInManagement([this]() { OpenWeldProcessDialog(); }); }));
	addMenuAction(managementProcessMenu, createManagementAction("焊道补偿", [this, openInManagement]() { openInManagement([this]() { OpenWeldSeamCompDialog(); }); }));
	addMenuAction(managementProcessMenu, createManagementAction("精测点云处理", [this]() { OpenPrecisePointCloudProcessingPage(); }));
	addMenuAction(managementProcessMenu, createManagementAction("测量焊接参数", [this, openInManagement]() { openInManagement([this]() { OpenPreciseMeasureEditDialog(); }); }));

	addMenuAction(managementCameraMenu, createManagementAction("相机参数", [this, openInManagement]() { openInManagement([this]() { OpenCameraParamDialog(); }); }));

	addMenuAction(managementDebugMenu, createManagementAction("点动控制", [this, openInManagement]() { openInManagement([this]() { OpenRobotJogDialog(); }); }));
	addMenuAction(managementDebugMenu, createManagementAction("功能测试", [this, openInManagement]() { openInManagement([this]() { OpenFunctionTestDialog(); }); }));
	addMenuAction(managementDebugMenu, createManagementAction("结果打包压缩", [this]() { OpenResultArchiveDialog(); }));
	addMenuAction(managementDebugMenu, createManagementAction("在线服务", [this]() { OpenOnlineServicesDialog(); }));
	addMenuAction(managementDebugMenu, createManagementAction("工件模型", [this]() { OpenWorkpieceMeshPage(); }));
	addMenuAction(managementDebugMenu, createManagementAction("模型配准", [this]() { OpenModelAlignmentPage(); }));
	addMenuAction(managementDebugMenu, createManagementAction("虚拟焊道测试", [this]() { OpenVirtualWeldTestPage(); }));
	addMenuAction(managementDebugMenu, createManagementAction("流程测试", [this]() { OpenProcessLoopTestPage(); }));
	addMenuAction(managementDebugMenu, createManagementAction("配置数据库查看", [this]() { OpenConfigDatabaseViewerDialog(); }));

	m_pAccountManagementAction = addMenuAction(managementAccountMenu, createManagementAction("账号管理", [this]() { OpenAccountManagementDialog(); }));
	m_pAccountManagementAction->setEnabled(false);

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
	m_pManagementIconBgBtn = new QPushButton("桌面图标：无底色", m_pManagementHomePage);
	m_pManagementIconBgBtn->setCheckable(true);
	m_pManagementIconBgBtn->setMinimumHeight(34);
	m_pManagementIconBgBtn->setMinimumWidth(150);
	m_pManagementIconBgBtn->setStyleSheet("QPushButton { padding: 6px 14px; font-size: 14px; border-radius: 10px; }");
	QLabel* scanTimestampLabel = new QLabel("扫描时间轴：", m_pManagementHomePage);
	scanTimestampLabel->setStyleSheet("QLabel { color: #9ED8DB; padding-left: 8px; }");
	m_pScanTimestampSourceCombo = new QComboBox(m_pManagementHomePage);
	m_pScanTimestampSourceCombo->addItem(
		MeasureThenWeldRuntimeConfig::DisplayName(MeasureThenWeldRuntimeConfig::ScanTimestampSource::Robot),
		MeasureThenWeldRuntimeConfig::ToStorageString(MeasureThenWeldRuntimeConfig::ScanTimestampSource::Robot));
	m_pScanTimestampSourceCombo->addItem(
		MeasureThenWeldRuntimeConfig::DisplayName(MeasureThenWeldRuntimeConfig::ScanTimestampSource::Pc),
		MeasureThenWeldRuntimeConfig::ToStorageString(MeasureThenWeldRuntimeConfig::ScanTimestampSource::Pc));
	m_pScanTimestampSourceCombo->setFixedSize(150, 34);
	m_pScanTimestampSourceCombo->setToolTip("先测后焊扫描匹配用的机器人位姿时间轴：机器人时间戳使用robot_ms；PC接收时间使用pc_recv_ms。");
	QLabel* stepSdkInterfaceLabel = new QLabel("STEP接口：", m_pManagementHomePage);
	stepSdkInterfaceLabel->setStyleSheet("QLabel { color: #9ED8DB; padding-left: 8px; }");
	m_pStepSdkInterfaceModeCombo = new QComboBox(m_pManagementHomePage);
	m_pStepSdkInterfaceModeCombo->addItem(
		MeasureThenWeldRuntimeConfig::DisplayName(MeasureThenWeldRuntimeConfig::StepSdkInterfaceMode::Timestamp),
		MeasureThenWeldRuntimeConfig::ToStorageString(MeasureThenWeldRuntimeConfig::StepSdkInterfaceMode::Timestamp));
	m_pStepSdkInterfaceModeCombo->addItem(
		MeasureThenWeldRuntimeConfig::DisplayName(MeasureThenWeldRuntimeConfig::StepSdkInterfaceMode::Legacy),
		MeasureThenWeldRuntimeConfig::ToStorageString(MeasureThenWeldRuntimeConfig::StepSdkInterfaceMode::Legacy));
	m_pStepSdkInterfaceModeCombo->setFixedSize(150, 34);
	m_pStepSdkInterfaceModeCombo->setToolTip("新版使用STEP SDK getTimestamp()读取robot_ms；旧版绕开时间戳接口，使用getCartPosWorld()/getAxisPos()/getProgramState()并以PC接收时间兜底。");
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
	managementTitleLayout->addWidget(m_pManagementIconBgBtn);
	managementTitleLayout->addWidget(scanTimestampLabel);
	managementTitleLayout->addWidget(m_pScanTimestampSourceCombo);
	managementTitleLayout->addWidget(stepSdkInterfaceLabel);
	managementTitleLayout->addWidget(m_pStepSdkInterfaceModeCombo);
	managementTitleLayout->addWidget(touchKeyboardLabel);
	managementTitleLayout->addWidget(m_pTouchKeyboardModeCombo);
	managementTitleLayout->addWidget(m_pManagementUserLabel);
	managementHomeLayout->addLayout(managementTitleLayout);

	m_pPermissionHintLabel = new QLabel("管理功能请从顶部菜单栏进入，常用状态和全局开关保留在本页。", m_pManagementHomePage);
	m_pPermissionHintLabel->setWordWrap(true);
	m_pPermissionHintLabel->setStyleSheet("QLabel { color: #AFC8CE; font-size: 14px; }");
	managementHomeLayout->addWidget(m_pPermissionHintLabel);

	QGroupBox* managementInfoGroup = new QGroupBox("管理说明", m_pManagementHomePage);
	QVBoxLayout* managementInfoLayout = new QVBoxLayout(managementInfoGroup);
	QLabel* managementInfoLabel = new QLabel(
		"管理页面使用顶部菜单栏组织功能，只有工程师或管理员可以进入管理页面；其中“账号管理”仅管理员可用。",
		managementInfoGroup);
	managementInfoLabel->setWordWrap(true);
	managementInfoLayout->addWidget(managementInfoLabel);
	QPlainTextEdit* managementLogText = new QPlainTextEdit(managementInfoGroup);
	managementLogText->setReadOnly(true);
	managementLogText->document()->setMaximumBlockCount(120);
	managementLogText->setPlainText("管理页面提示：请从顶部菜单栏打开机器人、工艺、相机、调试和账号功能。");
	managementInfoLayout->addWidget(managementLogText, 1);
	managementHomeLayout->addWidget(managementInfoGroup, 1);

	connect(m_pManagementCameraReceiveModeBtn, &QPushButton::toggled, this, [this](bool checked)
		{
			SetSharedScanCameraReceiverMode(checked);
		});
	connect(m_pManagementIconBgBtn, &QPushButton::toggled, this, [this](bool checked)
		{
			SetDesktopIconWithBackground(checked);
		});
	connect(m_pScanTimestampSourceCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int index)
		{
			if (m_pScanTimestampSourceCombo == nullptr || index < 0)
			{
				return;
			}

			MeasureThenWeldRuntimeConfig::SaveScanTimestampSource(
				MeasureThenWeldRuntimeConfig::FromStorageString(m_pScanTimestampSourceCombo->itemData(index).toString()));
		});
	connect(m_pStepSdkInterfaceModeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int index)
		{
			if (m_pStepSdkInterfaceModeCombo == nullptr || index < 0)
			{
				return;
			}

			MeasureThenWeldRuntimeConfig::SaveStepSdkInterfaceMode(
				MeasureThenWeldRuntimeConfig::StepSdkInterfaceModeFromStorageString(m_pStepSdkInterfaceModeCombo->itemData(index).toString()));
			STEPRobotCtrl::InvalidateStepSdkInterfaceModeCache();  // 驱动侧进程级缓存立即失效
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
		quickTeachPositionBtn,
		quickPositionBtn,
		quickCalibrationBtn,
		m_pWeldSeamCompBtn,
		m_pCameraParamBtn
	};
	for (QPushButton* button : functionToolButtons)
	{
		m_robotOperationWidgets.push_back(button);
	}
	m_fanucOnlyWidgets = {
		functionToolButtons.value("uploadLs"),
		functionToolButtons.value("curposDiagnostic")
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
	addCommandAction(measureMenu, "扫描位置示教", [this]() { OpenPositionTeachDialog(); });
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
	addCommandAction(cameraMenu, "点云查看", [this]() { OpenPointCloudViewerDialog(); });
	addToolbarSeparator();
	addCommandAction(weldMenu, "工艺参数", [this]() { OpenWeldProcessDialog(); });
	addCommandAction(weldMenu, "焊道补偿", [this]() { OpenWeldSeamCompDialog(); });
	addCommandAction(weldMenu, "精测点云处理", [this]() {
		if (RoleLevel(m_sCurrentUserRole) < RoleLevel(kRoleEngineer))
		{
			ShowManagementPage();
			return;
		}
		ShowManagementPage();
		OpenPrecisePointCloudProcessingPage();
	});
	addToolbarSeparator();
	addCommandAction(debugMenu, "功能测试", [this]() { OpenFunctionTestDialog(); });
	addCommandAction(debugMenu, "结果打包压缩", [this]() { OpenResultArchiveDialog(); });
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
		{ "命令行日志", "Log/CliLog.txt" },
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
		{ "命令行", "Log/CliLog.txt" },
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

	dashboardLayout->addWidget(robotInfoSplitter, 1);
	dashboardLayout->addWidget(entryGroup, 0);

	connect(m_pCurrentUserButton, &QPushButton::clicked, this, &QtWidgetsApplication4::ShowCurrentUserMenu);
	connect(m_pDashboardDebugLogBtn, &QPushButton::toggled, this, &QtWidgetsApplication4::SetDebugLogMode);
	connect(m_pDashboardConnectBtn, &QPushButton::clicked, this, &QtWidgetsApplication4::ToggleCurrentRobotConnection);
	connect(m_pDashboardClearAlarmBtn, &QPushButton::clicked, this, &QtWidgetsApplication4::RobotClearAlarmTest);
	connect(m_pDashboardEmergencyStopBtn, &QPushButton::clicked, this, &QtWidgetsApplication4::RobotEmergencyStop);
	connect(m_pDashboardModeBtn, &QPushButton::clicked, this, &QtWidgetsApplication4::RobotSwitchStepMode);
	for (const QPair<QString, QString>& spec : functionToolSpecs)
	{
		if (QPushButton* button = functionToolButtons.value(spec.first, nullptr))
		{
			connect(button, &QPushButton::clicked, this, [this, actionId = spec.first]()
				{
					RunFunctionTestDashboardTool(actionId);
				});
		}
	}
	connect(quickReadTool1Btn, &QPushButton::clicked, this, &QtWidgetsApplication4::ReadTool1ToGunTool);
	connect(quickMeasureBtn, &QPushButton::clicked, this, &QtWidgetsApplication4::OpenMeasureThenWeldDialog);
	connect(quickPointCloudBtn, &QPushButton::clicked, this, &QtWidgetsApplication4::OpenPointCloudViewerDialog);
	connect(quickTeachPositionBtn, &QPushButton::clicked, this, &QtWidgetsApplication4::OpenPositionTeachDialog);
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

	if (qApp != nullptr)
	{
		qApp->installEventFilter(this);
	}
	QTimer* accountSessionTimer = new QTimer(this);
	accountSessionTimer->setInterval(250);
	connect(accountSessionTimer, &QTimer::timeout, this, [this]()
		{
			const bool managementVisible = m_pManagementPage != nullptr
				&& m_pManagementPage->isVisible();
			const bool mainSurfaceActive = m_pMainStack != nullptr
				&& m_pMainStack->currentWidget() != m_pAuthPage;
			bool taggedSessionWindowVisible = false;
			if (qApp != nullptr
				&& !RobotOperationLease::AnyActive()
				&& !HasRunningMeasureThenWeldFlow())
			{
				for (QWidget* window : qApp->topLevelWidgets())
				{
					if (window != nullptr
						&& window->isVisible()
						&& window->property("_requires_live_account_session").toBool())
					{
						taggedSessionWindowVisible = true;
						break;
					}
				}
			}
			if ((!managementVisible && !mainSurfaceActive && !taggedSessionWindowVisible)
				|| m_bAccountSessionEventValidation)
			{
				return;
			}
			m_bAccountSessionEventValidation = true;
			const bool validSession = ValidateCurrentAccountSession(
				managementVisible
					? QStringLiteral("保持管理页面会话")
					: QStringLiteral("保持当前页面会话"));
			const bool authorized = validSession
				&& (!managementVisible
					|| RoleLevel(m_sCurrentUserRole) >= RoleLevel(kRoleEngineer));
			m_bAccountSessionEventValidation = false;
			if (!authorized)
			{
				RevokePrivilegedUiAccess();
			}
		});
	accountSessionTimer->start();

	EnsureDefaultAdminAccount();
	RefreshAccountUi();
	LoadCameraReceiveMode();
	RefreshScanTimestampSourceUi();
	RefreshStepSdkInterfaceModeUi();
	RefreshTouchKeyboardModeUi();
	RefreshDesktopIconBgButtonUi();
	if (BrandingConfig::IsActive())
	{
		BrandingConfig::ApplyDesktopShortcutIcons();  // 安装后首次启动即把桌面/开始菜单快捷方式图标刷成当前品牌图标
	}
	LoadLoginState();
	ShowAuthPage(m_bAccountRecoveryRequired
		? QStringLiteral("账号认证库需要安全恢复，已禁用自动登录。请从受控备份恢复账号库或联系维护人员，程序不会重新生成已知默认管理员。")
		: QString());
	if (!m_bAccountRecoveryRequired)
	{
		TryAutoLogin();
	}

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
	LoadRobotLogFile("Log/RobotRunLog.txt", true);  // 默认展示运行日志（Log/Log.txt 无写入方，避免首屏空白）
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
			const QString sourceText = QString::fromStdString(pRobotDriver->GetStateMonitorSourceText());
			// 扫描匹配时间轴每次刷新现读配置：管理页下拉切换后，下一个刷新周期即更新，与扫描实际取值一致。
			const MeasureThenWeldRuntimeConfig::ScanTimestampSource monitorScanTimestampSource =
				MeasureThenWeldRuntimeConfig::LoadScanTimestampSource();
			const QString scanAxisText = QString("%1(%2)")
				.arg(MeasureThenWeldRuntimeConfig::DisplayName(monitorScanTimestampSource))
				.arg(MeasureThenWeldRuntimeConfig::FieldName(monitorScanTimestampSource));
			QString monitorText = QString(
				"状态: %1\n"
				"接口: %2\n"
				"扫描匹配时间轴: %21\n"
				"robot_ms=%3  pc_recv_ms=%4  cache=%5/200\n"
				"位置: X=%6  Y=%7  Z=%8  W=%9  P=%10  R=%11\n"
				"脉冲: S=%12  L=%13  U=%14  R=%15  B=%16  T=%17  EX1=%18  EX2=%19  EX3=%20")
				.arg(stateText)
				.arg(sourceText)
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
				.arg(pulse.lBZPulse)
				.arg(scanAxisText);
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
					// 虚拟焊道测试运行中禁止切换机器人：运行线程持有旧单元驱动指针，切换会造成并发/状态混乱。
					if (m_pVirtualWeldTestPage != nullptr && m_pVirtualWeldTestPage->IsRunning())
					{
						QMessageBox::information(this, "虚拟焊道测试",
							"虚拟焊道测试流程正在运行，请等待流程结束后再切换机器人。");
						RefreshRobotSelectorUi();
						return;
					}
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
	if (qApp != nullptr)
	{
		qApp->removeEventFilter(this);
	}
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
	delete m_skjCameraControlClient;
	m_skjCameraControlClient = nullptr;
	StopScanCameraRuntimes();
	delete m_pContralUnit;
	m_pContralUnit = nullptr;
	if (m_bEnforceInteractiveSessionLeaseGate)
	{
		// 测试进程或同进程重建主窗口时不遗留上一窗口的交互式会话状态。
		RobotOperationLease::SetNewOperationsAllowed(true);
	}
}

bool QtWidgetsApplication4::HasRunningMeasureThenWeldFlow() const
{
	const QList<QPointer<MeasureThenWeldDialog>> pages = m_measureThenWeldPages.values();
	for (const QPointer<MeasureThenWeldDialog>& guardedPage : pages)
	{
		if (const MeasureThenWeldDialog* page = guardedPage.data())
		{
			if (page->IsRunning())
			{
				return true;
			}
		}
	}
	// 流程测试是循环跑的先测后焊，同样在驱动机器人：一并纳入本守卫，
	// 使关程序/升级安装/先测后焊页避让在流程测试运行时也生效。
	if (const ProcessLoopTestDialog* loopPage =
		qobject_cast<const ProcessLoopTestDialog*>(m_pProcessLoopTestPage))
	{
		if (loopPage->IsRunning())
		{
			return true;
		}
	}
	return false;
}

void QtWidgetsApplication4::closeEvent(QCloseEvent* event)
{
	if (RobotOperationLease::AnyActive())
	{
		if (ui.statusBar != nullptr)
		{
			ui.statusBar->showMessage(
				QString("机器人硬件操作正在运行：%1。关闭已拦截；如需终止请使用主页固定红色安全停止按钮。")
					.arg(RobotOperationLease::ActiveSummary()),
				12000);
		}
		if (event != nullptr)
		{
			event->ignore();
		}
		return;
	}
	if (HasRunningMeasureThenWeldFlow())
	{
		QMessageBox::information(
			this,
			"先测后焊",
			"先测后焊流程正在运行，请等待流程结束后再关闭程序。");
		if (event != nullptr)
		{
			event->ignore();
		}
		return;
	}

	if (m_pVirtualWeldTestPage != nullptr && m_pVirtualWeldTestPage->IsRunning())
	{
		QMessageBox::information(
			this,
			"虚拟焊道测试",
			"虚拟焊道测试流程正在运行，请等待流程结束后再关闭程序。");
		if (event != nullptr)
		{
			event->ignore();
		}
		return;
	}

	// 扫描数据上传中：拦截退出，弹进度框（已完成/剩余/速度/ETA），传完自动放行；
	// 用户可「强制退出」——先删掉传一半的文件再退出；关掉进度框(X/Esc)则不退出、上传继续后台跑。
	if (m_pScanDataUploader != nullptr && m_pScanDataUploader->IsBusy())
	{
		QDialog dlg(this);
		dlg.setWindowTitle(QStringLiteral("扫描数据上传中"));
		dlg.setModal(true);
		dlg.setMinimumWidth(460);
		QVBoxLayout* lay = new QVBoxLayout(&dlg);
		QLabel* info = new QLabel(&dlg);
		info->setWordWrap(true);
		QProgressBar* bar = new QProgressBar(&dlg);
		bar->setRange(0, 100);
		QPushButton* forceBtn = new QPushButton(QStringLiteral("强制退出（删除未传完的文件）"), &dlg);
		lay->addWidget(info);
		lay->addWidget(bar);
		lay->addWidget(forceBtn);

		auto fmtMB = [](qint64 b) { return QString::number(b / 1048576.0, 'f', 1); };
		auto render = [&](int done, int total, const QString& name, qint64 sent, qint64 tot, double bps, int eta)
		{
			const int pct = tot > 0 ? static_cast<int>(sent * 100 / tot) : 0;
			bar->setValue(pct);
			const QString etaText = eta > 0 ? QStringLiteral("约 %1 秒").arg(eta) : QStringLiteral("计算中…");
			info->setText(QStringLiteral(
				"正在上传第 %1/%2 个案例：%3\n"
				"当前文件 %4/%5 MB（%6%），速度 %7 MB/s，预计剩余 %8\n"
				"还有 %9 个案例未完成，请勿断电。")
				.arg(done + 1).arg(total).arg(name)
				.arg(fmtMB(sent)).arg(fmtMB(tot)).arg(pct)
				.arg(QString::number(bps / 1048576.0, 'f', 2)).arg(etaText)
				.arg(total - done));
		};
		const ScanDataUploader::ProgressSnapshot snap = m_pScanDataUploader->CurrentProgress();
		render(snap.doneItems, snap.totalItems, snap.currentName, snap.sentBytes, snap.totalBytes,
			snap.bytesPerSec, snap.etaSeconds);

		const QMetaObject::Connection progConn = connect(m_pScanDataUploader, &ScanDataUploader::uploadProgress,
			&dlg, [&](int done, int total, const QString& name, qint64 sent, qint64 tot, double bps, int eta)
			{ render(done, total, name, sent, tot, bps, eta); });
		connect(forceBtn, &QPushButton::clicked, &dlg, [&dlg]() { dlg.done(2); });  // 2=强制退出
		QTimer poll;
		connect(&poll, &QTimer::timeout, &dlg, [this, &dlg]()
			{ if (!m_pScanDataUploader->IsBusy()) { dlg.done(1); } });  // 1=传完自动放行
		poll.start(200);

		const int r = dlg.exec();
		disconnect(progConn);

		if (r == 2)
		{
			forceBtn->setText(QStringLiteral("正在停止并清理…"));
			m_pScanDataUploader->CancelAndWait();  // 删服务器半截文件 + 等后台收尾后放行
		}
		else if (r != 1)
		{
			// X/Esc 关掉了进度框：不退出程序，上传转后台继续。
			if (event != nullptr)
			{
				event->ignore();
			}
			return;
		}
	}

	QMainWindow::closeEvent(event);
}

bool QtWidgetsApplication4::eventFilter(QObject* watched, QEvent* event)
{
	if (event != nullptr && event->type() == QEvent::Show)
	{
		QWidget* shownWindow = qobject_cast<QWidget*>(watched);
		if (shownWindow != nullptr
			&& shownWindow->isWindow()
			&& shownWindow != this
			&& shownWindow != m_pManagementPage)
		{
			QWidget* owner = shownWindow->parentWidget();
			const bool ownerTagged = owner != nullptr
				&& owner->window() != nullptr
				&& owner->window()->property("_requires_live_account_session").toBool();
			const bool ownerInManagement = owner != nullptr
				&& m_pManagementPage != nullptr
				&& (owner == m_pManagementPage || m_pManagementPage->isAncestorOf(owner));
			const bool ownerInActiveMain = owner != nullptr
				&& m_pMainStack != nullptr
				&& m_pMainStack->currentWidget() != m_pAuthPage
				&& (owner == this || isAncestorOf(owner));
			if (ownerTagged || ownerInManagement || ownerInActiveMain)
			{
				shownWindow->setProperty("_requires_live_account_session", true);
			}
		}
	}
	const bool mouseDrag = event != nullptr
		&& event->type() == QEvent::MouseMove
		&& static_cast<QMouseEvent*>(event)->buttons() != Qt::NoButton;
	const bool tabletDrag = event != nullptr
		&& event->type() == QEvent::TabletMove
		&& static_cast<QTabletEvent*>(event)->buttons() != Qt::NoButton;
	const bool privilegedInput = event != nullptr
		&& (event->type() == QEvent::MouseButtonPress
			|| event->type() == QEvent::MouseButtonRelease
			|| event->type() == QEvent::MouseButtonDblClick
			|| mouseDrag
			|| event->type() == QEvent::Wheel
			|| event->type() == QEvent::KeyPress
			|| event->type() == QEvent::KeyRelease
			|| event->type() == QEvent::Shortcut
			|| event->type() == QEvent::ContextMenu
			|| event->type() == QEvent::TouchBegin
			|| event->type() == QEvent::TouchEnd
			|| event->type() == QEvent::TabletPress
			|| tabletDrag
			|| event->type() == QEvent::TabletRelease
			|| event->type() == QEvent::Gesture
			|| event->type() == QEvent::NativeGesture);
	QWidget* watchedWidget = qobject_cast<QWidget*>(watched);
	const bool insideVisibleManagement = watchedWidget != nullptr
		&& m_pManagementPage != nullptr
		&& m_pManagementPage->isVisible()
		&& (watchedWidget == m_pManagementPage
			|| m_pManagementPage->isAncestorOf(watchedWidget));
	const bool insideActiveMainSurface = watchedWidget != nullptr
		&& m_pMainStack != nullptr
		&& m_pMainStack->currentWidget() != m_pAuthPage
		&& (watchedWidget == this || isAncestorOf(watchedWidget));
	QWidget* watchedWindow = watchedWidget != nullptr ? watchedWidget->window() : nullptr;
	const bool insideTaggedSessionWindow = watchedWindow != nullptr
		&& watchedWindow->isVisible()
		&& watchedWindow->property("_requires_live_account_session").toBool();
	if (privilegedInput
		&& (insideVisibleManagement || insideActiveMainSurface || insideTaggedSessionWindow)
		&& !m_bAccountSessionEventValidation)
	{
		m_bAccountSessionEventValidation = true;
		const bool validSession = ValidateCurrentAccountSession(
			insideVisibleManagement
				? QStringLiteral("执行管理页面操作")
				: QStringLiteral("执行当前页面操作"));
		const bool authorized = validSession
			&& (!insideVisibleManagement
				|| RoleLevel(m_sCurrentUserRole) >= RoleLevel(kRoleEngineer));
		m_bAccountSessionEventValidation = false;
		if (!authorized)
		{
			RevokePrivilegedUiAccess();
			const bool safetyEscape = event->type() == QEvent::KeyPress
				&& static_cast<QKeyEvent*>(event)->key() == Qt::Key_Escape;
			const QAbstractButton* button = qobject_cast<QAbstractButton*>(watchedWidget);
			const QString safetyControlText = button == nullptr
				? QString()
				: (button->objectName() + QLatin1Char(' ') + button->text()).toLower();
			const bool safetyControl = safetyControlText.contains(QStringLiteral("stop"))
				|| safetyControlText.contains(QStringLiteral("abort"))
				|| safetyControlText.contains(QStringLiteral("cancel"))
				|| safetyControlText.contains(QStringLiteral("release"))
				|| safetyControlText.contains(QStringLiteral("停止"))
				|| safetyControlText.contains(QStringLiteral("急停"))
				|| safetyControlText.contains(QStringLiteral("取消"))
				|| safetyControlText.contains(QStringLiteral("中止"))
				|| safetyControlText.contains(QStringLiteral("松开"));
			if ((RobotOperationLease::AnyActive() || HasRunningMeasureThenWeldFlow())
				&& (safetyEscape || safetyControl))
			{
				return QMainWindow::eventFilter(watched, event);
			}
			event->accept();
			return true;
		}
	}
	if (event != nullptr && event->type() == QEvent::Resize)
	{
		if (QWidget* page = qobject_cast<QWidget*>(watched))
		{
			if (page->property("_management_embedded_page").isValid())
			{
				PositionEmbeddedBackButton(page);
			}
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
			|| watched == m_pControlUnitManagementPage
			|| watched == m_pFtpJobManagementPage
			|| watched == m_pConfigDatabaseViewerPage
			|| watched == m_pOnlineServicesPage
			|| watched == m_pProcessLoopTestPage
			|| watched == m_pPrecisePointCloudProcessingPage)
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
	if (!ValidateCurrentAccountSession(QStringLiteral("进入主页")))
	{
		return;
	}
	CloseGrooveCameraPreviewWindow();
	setMaximumSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX);
	setMinimumSize(720, 500);
	if (isMaximized() || isFullScreen())
	{
		showNormal();
	}
	ResizeWindowForAvailableGeometry(this, QSize(1280, 700), 0.96, 0.90);

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
	if (!ValidateCurrentAccountSession(QStringLiteral("打开当前用户菜单")))
	{
		return;
	}
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
	if (!ValidateCurrentAccountSession(QStringLiteral("进入管理页面")))
	{
		return;
	}
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
		m_pManagementPage->setMinimumSize(720, 500);
		RefreshAccountUi();
		ShowManagementHomePage();
		ShowMaximizedWithUnifiedChrome(m_pManagementPage);
		m_pManagementPage->raise();
		m_pManagementPage->activateWindow();
	}
}

void QtWidgetsApplication4::ShowManagementHomePage()
{
	if (!ValidateCurrentAccountSession(QStringLiteral("打开管理首页"))
		|| RoleLevel(m_sCurrentUserRole) < RoleLevel(kRoleEngineer))
	{
		RevokePrivilegedUiAccess();
		return;
	}
	CloseGrooveCameraPreviewWindow();
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
	RobotDriverAdaptor* currentDriver = nullptr;
	if (const T_CONTRAL_UNIT* unitInfo = CurrentContralUnit())
	{
		currentDriver = static_cast<RobotDriverAdaptor*>(unitInfo->pUnitDriver);
	}
	const bool isStepDriver = dynamic_cast<STEPRobotCtrl*>(currentDriver) != nullptr;
	const bool isFanucDriver = dynamic_cast<FANUCRobotCtrl*>(currentDriver) != nullptr;
	if (DashboardToolPanel* panel = dynamic_cast<DashboardToolPanel*>(m_pDashboardToolPanel))
	{
		panel->SetCurrentRobotScope(isFanucDriver ? "fanuc" : (isStepDriver ? "step" : QString()));
		panel->SetEditingEnabled(hasReadyDriver, hasReadyDriver
			? QString()
			: "当前没有可用控制单元，不能修改模块。");
	}
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
	const bool robotOperationBusy = RobotOperationLease::AnyActive();
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
			if (enabled && containsWidget(m_fanucOnlyWidgets, widget.data()) && !isFanucDriver)
			{
				enabled = false;
				disableReasons << "该测试依赖 FANUC 驱动，当前机器人不是 FANUC。";
			}
			const QString baseToolTip = enabled ? QString() : disableReasons.join('\n');
			widget->setProperty("robotSetupEnabled", enabled);
			widget->setProperty("robotSetupToolTip", baseToolTip);
			widget->setEnabled(enabled && !robotOperationBusy);
			widget->setToolTip(robotOperationBusy
				? QString("机器人硬件操作正在运行（%1）；仅保留主页固定红色安全停止按钮。")
					.arg(RobotOperationLease::ActiveSummary())
				: baseToolTip);
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
	const bool isFanucDriver = dynamic_cast<FANUCRobotCtrl*>(currentDriver) != nullptr;
	const bool robotOperationBusy = RobotOperationLease::AnyActive();
	if (m_pDashboardEmergencyStopBtn != nullptr)
	{
		m_pDashboardEmergencyStopBtn->setEnabled(robotOperationBusy);
	}
	for (const QPointer<QWidget>& widget : m_robotOperationWidgets)
	{
		if (widget.isNull())
		{
			continue;
		}
		const QVariant baseEnabledProperty = widget->property("robotSetupEnabled");
		const bool baseEnabled = baseEnabledProperty.isValid()
			? baseEnabledProperty.toBool()
			: widget->isEnabled();
		if (!baseEnabledProperty.isValid())
		{
			widget->setProperty("robotSetupEnabled", baseEnabled);
			widget->setProperty("robotSetupToolTip", widget->toolTip());
		}
		const QString baseToolTip = widget->property("robotSetupToolTip").toString();
		widget->setEnabled(baseEnabled && !robotOperationBusy);
		widget->setToolTip(robotOperationBusy
			? QString("机器人硬件操作正在运行（%1）；仅保留主页固定红色安全停止按钮。")
				.arg(RobotOperationLease::ActiveSummary())
			: baseToolTip);
	}
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
		m_pDashboardModeBtn->setProperty("dashboardToolRuntimeVisible", isStepDriver);
		m_pDashboardModeBtn->setText("STEP 模式");
		m_pDashboardModeBtn->hide();
		m_pDashboardModeBtn->setToolTip(isStepDriver
			? "切换新时达机器人模式；安全中止请使用主页固定红色按钮。"
			: "仅 STEP 显示模式切换。");
	}
	if (DashboardToolPanel* panel = dynamic_cast<DashboardToolPanel*>(m_pDashboardToolPanel))
	{
		panel->RefreshToolStates();
	}
}

bool QtWidgetsApplication4::EnsureRobotUiActionIdle(const QString& actionName)
{
	if (!RobotOperationLease::AnyActive())
	{
		return true;
	}
	if (ui.statusBar != nullptr)
	{
		ui.statusBar->showMessage(
			QString("机器人硬件操作正在运行（%1）；%2已冻结，请使用主页固定红色安全停止按钮。")
				.arg(RobotOperationLease::ActiveSummary(), actionName),
			10000);
	}
	return false;
}

void QtWidgetsApplication4::RunFunctionTestDashboardTool(const QString& actionId)
{
	if (!EnsureRobotUiActionIdle(QStringLiteral("普通工具")))
	{
		return;
	}
	if (!RequirePermission(kRoleEngineer, "机器人功能测试"))
	{
		return;
	}
	const int currentUnitIndex = CurrentRobotUnitIndex();
	QStackedWidget* targetStack = CurrentEmbeddedTargetStack();
	if (targetStack == nullptr)
	{
		targetStack = m_pMainStack;
	}
	if (m_pFunctionTestPage != nullptr && m_nFunctionTestPageUnitIndex != currentUnitIndex)
	{
		if (RobotOperationLease::AnyActive())
		{
			QMessageBox::warning(this, "机器人功能测试",
				QString("机器人硬件操作正在运行（%1），不能销毁原机器人功能测试页。")
					.arg(RobotOperationLease::ActiveSummary()));
			return;
		}
		delete m_pFunctionTestPage;
		m_pFunctionTestPage = nullptr;
	}
	if (m_pFunctionTestPage == nullptr)
	{
		m_pFunctionTestPage = new FunctionTestDialog(m_pContralUnit, currentUnitIndex, ScanCameraCacheForUnit(currentUnitIndex), targetStack);
		m_nFunctionTestPageUnitIndex = currentUnitIndex;
		PrepareEmbeddedPage(m_pFunctionTestPage, targetStack);
	}
	if (m_pFunctionTestPage != nullptr && !m_pFunctionTestPage->RunDashboardTool(actionId))
	{
		QMessageBox::warning(this, "机器人功能测试", "未找到对应的功能测试项。");
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
	if (!ValidateCurrentAccountSession(actionName))
	{
		return false;
	}
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

bool QtWidgetsApplication4::ValidateCurrentAccountSession(const QString& actionName)
{
	if (m_bSessionRevocationPendingSafetyStop)
	{
		if (m_bEnforceInteractiveSessionLeaseGate)
		{
			RobotOperationLease::SetNewOperationsAllowed(
				false,
				QStringLiteral("账号会话已失效；禁止启动新的机器人硬件操作。"));
		}
		if (RobotOperationLease::AnyActive() || HasRunningMeasureThenWeldFlow())
		{
			return false;
		}
		m_bSessionRevocationPendingSafetyStop = false;
		RevokePrivilegedUiAccess();
		RefreshAccountUi();
		ShowAuthPage(QStringLiteral("账号会话已失效；机器人操作已安全结束，请重新登录。"));
		return false;
	}
	if (m_bAccountRecoveryRequired)
	{
		ShowAuthPage(QStringLiteral("账号认证库处于安全恢复状态，所有登录、游客、注册和主页操作均已锁定。"));
		return false;
	}
	if (m_sCurrentUserName == QStringLiteral("游客"))
	{
		return m_sCurrentUserRole == kRoleOperator;
	}
	if (m_sCurrentUserName == QStringLiteral("访客")
		|| m_sCurrentUserName.trimmed().isEmpty())
	{
		ShowAuthPage(QStringLiteral("请先登录或明确选择游客登录。"));
		return false;
	}
	QString passwordRecord;
	QString currentRole;
	QString fingerprint;
	bool mustChangePassword = true;
	if (m_sCurrentUserSecurityFingerprint.isEmpty()
		|| !ConfigDatabase::TryReadAccountSecurityState(
			m_sCurrentUserName,
			&passwordRecord,
			&currentRole,
			&mustChangePassword,
			&fingerprint)
		|| mustChangePassword
		|| currentRole != m_sCurrentUserRole
		|| fingerprint != m_sCurrentUserSecurityFingerprint)
	{
		m_sCurrentUserName = QStringLiteral("访客");
		m_sCurrentUserRole = kRoleOperator;
		m_sCurrentUserSecurityFingerprint.clear();
		m_bPendingOpenManagementAfterLogin = false;
		if (m_bEnforceInteractiveSessionLeaseGate)
		{
			RobotOperationLease::SetNewOperationsAllowed(
				false,
				QStringLiteral("账号会话已失效；禁止启动新的机器人硬件操作。"));
		}
		if (RobotOperationLease::AnyActive() || HasRunningMeasureThenWeldFlow())
		{
			m_bSessionRevocationPendingSafetyStop = true;
			if (m_pAccountLogText != nullptr)
			{
				m_pAccountLogText->appendPlainText(QStringLiteral(
					"账号会话已失效；当前仅保留正在运行任务的安全停止/释放操作，任务结束后必须重新登录。"));
			}
			RefreshAccountUi();
			return false;
		}
		RevokePrivilegedUiAccess();
		RefreshAccountUi();
		ShowAuthPage(actionName.trimmed().isEmpty()
			? QStringLiteral("账号权限、密码或安全状态已变化，请重新登录。")
			: QStringLiteral("%1 前检测到账号权限、密码或安全状态已变化，请重新登录。")
				.arg(actionName));
		return false;
	}
	return true;
}

void QtWidgetsApplication4::RevokePrivilegedUiAccess()
{
	if (RobotOperationLease::AnyActive() || HasRunningMeasureThenWeldFlow())
	{
		return;
	}
	if (m_pManagementPage != nullptr)
	{
		m_pManagementPage->hide();
	}
	CloseGrooveCameraPreviewWindow();
	if (qApp != nullptr)
	{
		for (QWidget* window : qApp->topLevelWidgets())
		{
			if (window != nullptr
				&& window->property("_requires_live_account_session").toBool())
			{
				if (QDialog* dialog = qobject_cast<QDialog*>(window))
				{
					dialog->reject();
				}
				else
				{
					window->hide();
				}
			}
		}
	}
}

void QtWidgetsApplication4::EnsureDefaultAdminAccount()
{
	m_bInitialAdministratorSetupRequired = false;
	bool authenticationInitialized = false;
	if (!ConfigDatabase::TryReadAuthenticationInitialized(&authenticationInitialized))
	{
		m_bAccountRecoveryRequired = true;
		qCritical() << "Authentication initialization metadata is missing or invalid; automatic bootstrap is refused.";
		return;
	}
	QStringList accountNames;
	if (!ConfigDatabase::TryListScopedSettingIds(
			QStringLiteral("account"), AccountProfileModule(), &accountNames))
	{
		m_bAccountRecoveryRequired = true;
		qCritical() << "Account store enumeration failed; automatic bootstrap is refused.";
		return;
	}
	if (authenticationInitialized && accountNames.isEmpty())
	{
		m_bAccountRecoveryRequired = true;
		qCritical() << "Authentication was initialized but the account store is empty; automatic bootstrap is refused.";
		return;
	}
	if (!authenticationInitialized && !accountNames.isEmpty())
	{
		m_bAccountRecoveryRequired = true;
		qCritical() << "The account store is populated before authentication initialization; automatic bootstrap is refused.";
		return;
	}
	if (authenticationInitialized)
	{
		m_bAccountRecoveryRequired = AdminAccountCount() == 0;
		if (m_bAccountRecoveryRequired)
		{
			qCritical() << "Account store has users but no administrator; automatic bootstrap is refused.";
		}
		return;
	}

	// A fresh database has no public bootstrap credential.  The first local
	// user must choose the administrator password through the setup form; the
	// account and auth_initialized marker still commit atomically below.
	m_bAccountRecoveryRequired = false;
	m_bInitialAdministratorSetupRequired = true;
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
		m_pPermissionHintLabel->setText(m_bAccountRecoveryRequired
			? QStringLiteral("安全恢复状态：账号认证库缺失、损坏、账号被清空或没有管理员时，程序不会自动重新生成默认管理员；请恢复受控备份或联系维护人员。")
			: (m_bInitialAdministratorSetupRequired
				? QStringLiteral("首次启动：请在本机设置初始管理员密码。程序不提供公开的默认管理员密码。")
				: QStringLiteral("权限说明：登录后可进入主页；工程师或管理员可打开管理页面；管理员还可进入账号管理。")));
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
		m_pDashboardDebugLogBtn->setProperty("dashboardToolRuntimeVisible", isAdmin);
		m_pDashboardDebugLogBtn->hide();
		m_pDashboardDebugLogBtn->setEnabled(isAdmin);
		m_pDashboardDebugLogBtn->setChecked(isAdmin && m_bDebugLogMode);
		m_pDashboardDebugLogBtn->setText(m_bDebugLogMode ? "隐藏调试日志" : "显示调试日志");
	}
	if (DashboardToolPanel* panel = dynamic_cast<DashboardToolPanel*>(m_pDashboardToolPanel))
	{
		panel->RefreshToolStates();
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
	if (enabled && !ValidateCurrentAccountSession(QStringLiteral("开启调试日志")))
	{
		return;
	}
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

void QtWidgetsApplication4::LoadCameraReceiveMode()
{
	m_bUseSharedScanCameraReceiver = AppConfigBoolValue(CameraReceiveModeGroup(), "UseSharedReceiver", false);
	RefreshCameraReceiveModeButtonUi();
}

void QtWidgetsApplication4::SaveCameraReceiveMode() const
{
	WriteAppConfigValue(CameraReceiveModeGroup(), "UseSharedReceiver", m_bUseSharedScanCameraReceiver);
}

void QtWidgetsApplication4::RefreshCameraReceiveModeButtonUi()
{
	if (m_pManagementCameraReceiveModeBtn == nullptr)
	{
		return;
	}

	QSignalBlocker blocker(m_pManagementCameraReceiveModeBtn);
	m_pManagementCameraReceiveModeBtn->setEnabled(true);
	m_pManagementCameraReceiveModeBtn->setChecked(m_bUseSharedScanCameraReceiver);
	m_pManagementCameraReceiveModeBtn->setText(m_bUseSharedScanCameraReceiver
		? "相机接收：UDP共享"
		: "相机接收：TCP独立");
	m_pManagementCameraReceiveModeBtn->setToolTip("关闭为TCP独立连接；打开为旧UDP共享端口接收，并按相机IP分发到对应机器人缓存。");
}

void QtWidgetsApplication4::RefreshDesktopIconBgButtonUi()
{
	if (m_pManagementIconBgBtn == nullptr)
	{
		return;
	}
	const bool active = BrandingConfig::IsActive();
	const bool withBg = BrandingConfig::IconWithBackground();
	QSignalBlocker blocker(m_pManagementIconBgBtn);
	m_pManagementIconBgBtn->setVisible(active);  // 仅放了本地品牌包时才显示该开关
	m_pManagementIconBgBtn->setChecked(withBg);
	m_pManagementIconBgBtn->setText(withBg ? "桌面图标：有底色" : "桌面图标：无底色");
	m_pManagementIconBgBtn->setToolTip("切换窗口/任务栏/桌面快捷方式图标为有底色或无底色（仅本地品牌包 branding/ 生效）。");
}

void QtWidgetsApplication4::RefreshAllWindowIcons()
{
	const QIcon icon = BrandingConfig::WindowIcon();
	qApp->setWindowIcon(icon);
	const QList<QWidget*> tops = QApplication::topLevelWidgets();
	for (QWidget* w : tops)
	{
		if (w == nullptr || !w->isWindow())
		{
			continue;
		}
		const Qt::WindowType type = w->windowType();
		if (type == Qt::Window || type == Qt::Dialog)  // 跳过菜单/工具提示/下拉等瞬态顶层对象
		{
			w->setWindowIcon(icon);
		}
	}
}

void QtWidgetsApplication4::SetDesktopIconWithBackground(bool withBackground)
{
	if (BrandingConfig::IconWithBackground() == withBackground)
	{
		RefreshDesktopIconBgButtonUi();
		return;
	}
	BrandingConfig::SetIconWithBackground(withBackground);
	RefreshAllWindowIcons();                       // 窗口标题栏 + 任务栏即时刷新
	BrandingConfig::ApplyDesktopShortcutIcons();   // 桌面/开始菜单快捷方式图标重写（找不到则跳过）
	RefreshDesktopIconBgButtonUi();
}

void QtWidgetsApplication4::RefreshScanTimestampSourceUi()
{
	if (m_pScanTimestampSourceCombo == nullptr)
	{
		return;
	}

	const QString storageValue = MeasureThenWeldRuntimeConfig::ToStorageString(
		MeasureThenWeldRuntimeConfig::LoadScanTimestampSource());
	const int index = m_pScanTimestampSourceCombo->findData(storageValue);
	QSignalBlocker blocker(m_pScanTimestampSourceCombo);
	m_pScanTimestampSourceCombo->setCurrentIndex(index >= 0 ? index : 0);
}

void QtWidgetsApplication4::RefreshStepSdkInterfaceModeUi()
{
	if (m_pStepSdkInterfaceModeCombo == nullptr)
	{
		return;
	}

	const QString storageValue = MeasureThenWeldRuntimeConfig::ToStorageString(
		MeasureThenWeldRuntimeConfig::LoadStepSdkInterfaceMode());
	const int index = m_pStepSdkInterfaceModeCombo->findData(storageValue);
	QSignalBlocker blocker(m_pStepSdkInterfaceModeCombo);
	m_pStepSdkInterfaceModeCombo->setCurrentIndex(index >= 0 ? index : 0);
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
	if (m_bUseSharedScanCameraReceiver == enabled)
	{
		RefreshCameraReceiveModeButtonUi();
		return;
	}
	if (RobotOperationLease::AnyActive() || HasRunningMeasureThenWeldFlow())
	{
		QMessageBox::warning(
			this,
			QStringLiteral("相机接收模式"),
			QString("机器人硬件操作正在运行（%1），不能重建相机接收线程。请先安全停止操作。")
				.arg(RobotOperationLease::ActiveSummary()));
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

	delete m_pFunctionTestPage;
	m_pFunctionTestPage = nullptr;
	m_nFunctionTestPageUnitIndex = -1;
	delete m_pCameraParamPage;
	m_pCameraParamPage = nullptr;
	m_sCameraParamPageRobotName.clear();
	StopScanCameraRuntimes();
	InitializeScanCameraRuntimes();

	if (wasPreviewing && ui.GrooveCameraTestBtn != nullptr)
	{
		QString cameraIP;
		const int unitIndex = CurrentRobotUnitIndex();
		EnsureScanCameraRunningForUnit(unitIndex, cameraIP, false);
		m_grooveCameraDisplayTimer->start(33);
	}
	m_sGrooveCameraStatusText = QString("相机接收模式已切换为：%1。")
		.arg(m_bUseSharedScanCameraReceiver ? "UDP共享接收" : "TCP独立连接");
}

void QtWidgetsApplication4::SetAuthRegisterMode(bool registerMode)
{
	m_bAuthRegisterMode = !m_bAccountRecoveryRequired
		&& (m_bInitialAdministratorSetupRequired || registerMode);
	RefreshAuthModeUi();
}

void QtWidgetsApplication4::RefreshAuthModeUi()
{
	if (m_pAuthLoginModeBtn != nullptr)
	{
		QSignalBlocker blockerLogin(m_pAuthLoginModeBtn);
		m_pAuthLoginModeBtn->setChecked(!m_bAuthRegisterMode);
		m_pAuthLoginModeBtn->setEnabled(
			!m_bAccountRecoveryRequired && !m_bInitialAdministratorSetupRequired);
	}
	if (m_pAuthRegisterModeBtn != nullptr)
	{
		QSignalBlocker blockerRegister(m_pAuthRegisterModeBtn);
		m_pAuthRegisterModeBtn->setChecked(m_bAuthRegisterMode);
		m_pAuthRegisterModeBtn->setEnabled(
			!m_bAccountRecoveryRequired && !m_bInitialAdministratorSetupRequired);
	}
	if (m_pAuthSubmitBtn != nullptr)
	{
		m_pAuthSubmitBtn->setText(m_bInitialAdministratorSetupRequired
			? QStringLiteral("创建初始管理员")
			: (m_bAuthRegisterMode ? QStringLiteral("注册账号") : QStringLiteral("登录")));
	}
	if (m_pAuthHintLabel != nullptr)
	{
		if (m_bInitialAdministratorSetupRequired)
		{
			m_pAuthHintLabel->setText(QStringLiteral(
				"首次启动没有默认密码。请在本机为 admin 设置至少 8 位的新密码；账号与初始化标志会原子写入。"));
			m_pAuthHintLabel->show();
		}
		else if (!m_sAuthHintOverride.trimmed().isEmpty())
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
		m_pGuestLoginBtn->setVisible(
			!m_bAuthRegisterMode
			&& !m_bAccountRecoveryRequired
			&& !m_bInitialAdministratorSetupRequired);
		m_pGuestLoginBtn->setEnabled(
			!m_bAccountRecoveryRequired && !m_bInitialAdministratorSetupRequired);
	}
	if (m_pLoginNameCombo != nullptr)
	{
		if (m_bInitialAdministratorSetupRequired)
		{
			m_pLoginNameCombo->setCurrentText(QStringLiteral("admin"));
		}
		m_pLoginNameCombo->setEnabled(!m_bInitialAdministratorSetupRequired);
	}
}

void QtWidgetsApplication4::LoadLoginState()
{
	if (m_pLoginNameEdit == nullptr || m_pLoginPasswordEdit == nullptr || m_pRememberPasswordCheck == nullptr || m_pAutoLoginCheck == nullptr)
	{
		return;
	}

	const QString userName = AppConfigValue(LoginStateGroup(), "UserName");
	const bool rememberPassword = AppConfigBoolValue(LoginStateGroup(), "RememberPassword", false);
	const bool autoLogin = AppConfigBoolValue(LoginStateGroup(), "AutoLogin", false);

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
		const QString savedPassword = AppConfigValue(RememberedCredentialsGroup(), userName);
		if (!savedPassword.isEmpty())
		{
			m_pLoginPasswordEdit->setText(savedPassword);
		}
	}
	const bool credentialAvailable = rememberPassword && !m_pLoginPasswordEdit->text().isEmpty();
	m_pRememberPasswordCheck->setChecked(credentialAvailable);
	m_pAutoLoginCheck->setChecked(credentialAvailable && autoLogin);
	m_pAutoLoginCheck->setEnabled(credentialAvailable);
}

void QtWidgetsApplication4::SaveLoginState() const
{
	if (m_pLoginNameEdit == nullptr || m_pLoginPasswordEdit == nullptr || m_pRememberPasswordCheck == nullptr || m_pAutoLoginCheck == nullptr)
	{
		return;
	}

	const QString userName = m_pLoginNameEdit->text().trimmed();
	QStringList history = AppConfigListValue(LoginStateGroup(), "AccountHistory");
	history.removeAll(userName);
	if (!userName.isEmpty())
	{
		history.prepend(userName);
	}
	while (history.size() > 10)
	{
		history.removeLast();
	}
	const bool rememberRequested = m_pRememberPasswordCheck->isChecked();
	bool credentialStored = false;
	if (rememberRequested)
	{
		credentialStored = ConfigDatabase::WriteScopedSetting(
			QStringLiteral("global"),
			QString(),
			RememberedCredentialsGroup(),
			userName,
			m_pLoginPasswordEdit->text(),
			QStringLiteral("string"),
			true);
	}
	if (!credentialStored)
	{
		ClearRememberedCredential(userName);
		m_pRememberPasswordCheck->setChecked(false);
		m_pAutoLoginCheck->setChecked(false);
		m_pAutoLoginCheck->setEnabled(false);
	}
	QMap<QString, QString> loginState;
	loginState.insert(QStringLiteral("AccountHistory"), history.join(QLatin1Char('\n')));
	loginState.insert(QStringLiteral("UserName"), userName);
	loginState.insert(QStringLiteral("RememberPassword"), credentialStored ? QStringLiteral("1") : QStringLiteral("0"));
	loginState.insert(
		QStringLiteral("AutoLogin"),
		credentialStored && m_pAutoLoginCheck->isChecked()
			? QStringLiteral("1")
			: QStringLiteral("0"));
	if (!ConfigDatabase::WriteScopedSettings(
			QStringLiteral("global"), QString(), LoginStateGroup(), loginState))
	{
		ClearRememberedCredential(userName);
		m_pRememberPasswordCheck->setChecked(false);
		m_pAutoLoginCheck->setChecked(false);
		m_pAutoLoginCheck->setEnabled(false);
	}
	ConfigDatabase::RemoveScopedSetting(QStringLiteral("global"), QString(), LoginStateGroup(), "PasswordBase64");
	ConfigDatabase::RemoveScopedSetting(QStringLiteral("global"), QString(), SavedPasswordsGroup(), userName);
}

void QtWidgetsApplication4::RefreshLoginNameHistory()
{
	if (m_pLoginNameCombo == nullptr)
	{
		return;
	}

	const QString currentName = m_pLoginNameCombo->currentText();
	QStringList history = AppConfigListValue(LoginStateGroup(), "AccountHistory");
	const QString lastUser = AppConfigValue(LoginStateGroup(), "UserName").trimmed();
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

	const QString savedPassword = AppConfigValue(RememberedCredentialsGroup(), normalizedUser);
	if (!savedPassword.isEmpty())
	{
		m_pLoginPasswordEdit->setText(savedPassword);
		m_pRememberPasswordCheck->setChecked(true);
		m_pAutoLoginCheck->setEnabled(true);
		return;
	}

	m_pLoginPasswordEdit->clear();
	m_pRememberPasswordCheck->setChecked(false);
	m_pAutoLoginCheck->setChecked(false);
	m_pAutoLoginCheck->setEnabled(false);
}

bool QtWidgetsApplication4::TryAutoLogin()
{
	if (m_bAccountRecoveryRequired || m_bInitialAdministratorSetupRequired)
	{
		return false;
	}
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
	// 登录页会隐藏主页固定红色 STOP。只要仍有机器人流程在运行，就必须保留
	// 当前安全停止入口；主动切换账号应先结束流程。真实会话撤销走 pending
	// safety-stop 分支，不会依赖登录页完成停机。
	if (RobotOperationLease::AnyActive() || HasRunningMeasureThenWeldFlow())
	{
		m_bPendingOpenManagementAfterLogin = false;
		if (m_pAccountLogText != nullptr)
		{
			m_pAccountLogText->appendPlainText(QStringLiteral(
				"机器人操作运行期间已拒绝切换到登录页；主页安全停止入口保持可用。"));
		}
		QMessageBox* warning = new QMessageBox(
			QMessageBox::Warning,
			QStringLiteral("机器人操作正在运行"),
			QStringLiteral("当前机器人硬件操作尚未结束，不能切换账号。请先使用主页固定红色安全停止按钮或当前流程的停止/取消按钮安全结束操作。"),
			QMessageBox::Ok,
			this);
		warning->setAttribute(Qt::WA_DeleteOnClose);
		warning->setWindowModality(Qt::NonModal);
		warning->show();
		if (ui.statusBar != nullptr)
		{
			ui.statusBar->showMessage(
				QStringLiteral("机器人操作运行期间不能切换账号；主页安全停止入口保持可用。"),
				12000);
		}
		return;
	}
	if (m_bEnforceInteractiveSessionLeaseGate)
	{
		RobotOperationLease::SetNewOperationsAllowed(
			false,
			QStringLiteral("请先完成有效的本地账号登录，再启动机器人硬件操作。"));
	}
	if (m_pMainStack == nullptr || m_pAuthPage == nullptr)
	{
		return;
	}
	RevokePrivilegedUiAccess();

	setMaximumSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX);
	setMinimumSize(620, 480);
	if (isMaximized() || isFullScreen())
	{
		showNormal();
	}
	ResizeWindowForAvailableGeometry(this, QSize(920, 700), 0.88, 0.88);

	m_sAuthHintOverride = promptMessage.trimmed();
	SetAuthRegisterMode(m_bInitialAdministratorSetupRequired);
	m_pMainStack->setCurrentWidget(m_pAuthPage);
	if (m_pLoginNameEdit != nullptr)
	{
		m_pLoginNameEdit->setFocus(Qt::OtherFocusReason);
	}
}

bool QtWidgetsApplication4::VerifyAccount(
	const QString& userName,
	const QString& password,
	QString& role,
	bool& mustChangePassword,
	QString& passwordRecord,
	QString& error) const
{
	mustChangePassword = false;
	passwordRecord.clear();
	const QString normalizedUser = userName.trimmed();
	if (normalizedUser.isEmpty() || password.isEmpty())
	{
		error = "账号和密码不能为空。";
		return false;
	}

	QString expectedHash;
	QString securityFingerprint;
	if (!ConfigDatabase::TryReadAccountSecurityState(
			normalizedUser,
			&expectedHash,
			&role,
			&mustChangePassword,
			&securityFingerprint))
	{
		error = "账号安全记录无法读取。";
		return false;
	}

	const CredentialSecurity::PasswordVerification verification =
		CredentialSecurity::VerifyPasswordRecord(normalizedUser, password, expectedHash);
	if (verification == CredentialSecurity::PasswordVerification::Invalid)
	{
		error = "账号或密码不正确。";
		return false;
	}

	if (verification == CredentialSecurity::PasswordVerification::ValidNeedsUpgrade
		&& password.size() < 8)
	{
		mustChangePassword = true;
	}

	if (verification == CredentialSecurity::PasswordVerification::ValidNeedsUpgrade)
	{
		QString upgradedRecord;
		QString currentRole;
		QString upgradedFingerprint;
		if (!WriteAccountPasswordRecord(
				normalizedUser,
				password,
				expectedHash,
				false,
				mustChangePassword,
				upgradedRecord,
				currentRole,
				upgradedFingerprint,
				error))
		{
			error = QStringLiteral("旧账号密码验证成功，但安全升级失败：%1").arg(error);
			return false;
		}
		expectedHash = upgradedRecord;
		role = currentRole;
	}
	passwordRecord = expectedHash;
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
	if (password.size() < 8)
	{
		error = "密码至少需要 8 个字符。";
		return false;
	}
	if (role != kRoleOperator && role != kRoleEngineer && role != kRoleAdmin)
	{
		error = "权限类型无效。";
		return false;
	}

	if (AccountUserNames().contains(normalizedUser))
	{
		error = "账号已存在。";
		return false;
	}

	return WriteNewAccountRecord(normalizedUser, password, role, false, error);
}

bool QtWidgetsApplication4::PromptForcedPasswordChange(
	const QString& userName,
	const QString& temporaryPassword,
	const QString& expectedPasswordRecord,
	QString& replacementPassword,
	QString& currentRole,
	QString& securityFingerprint)
{
	replacementPassword.clear();
	currentRole.clear();
	securityFingerprint.clear();
	QDialog dialog(this);
	dialog.setWindowTitle(QStringLiteral("首次登录必须修改密码"));
	dialog.setWindowModality(Qt::ApplicationModal);
	dialog.setWindowFlag(Qt::WindowContextHelpButtonHint, false);
	ApplyUnifiedWindowChrome(&dialog);
	QVBoxLayout* layout = new QVBoxLayout(&dialog);
	QLabel* hint = new QLabel(
		QStringLiteral("账号 %1 当前使用临时密码。修改成功前不会建立登录会话，也不能进入主页或管理功能。")
			.arg(userName),
		&dialog);
	hint->setWordWrap(true);
	layout->addWidget(hint);
	QFormLayout* form = new QFormLayout();
	QLineEdit* passwordEdit = new QLineEdit(&dialog);
	QLineEdit* confirmEdit = new QLineEdit(&dialog);
	passwordEdit->setEchoMode(QLineEdit::Password);
	confirmEdit->setEchoMode(QLineEdit::Password);
	passwordEdit->setPlaceholderText(QStringLiteral("至少 8 个字符"));
	form->addRow(QStringLiteral("新密码"), passwordEdit);
	form->addRow(QStringLiteral("确认新密码"), confirmEdit);
	layout->addLayout(form);
	QDialogButtonBox* buttons = new QDialogButtonBox(
		QDialogButtonBox::Ok | QDialogButtonBox::Cancel, Qt::Horizontal, &dialog);
	buttons->button(QDialogButtonBox::Ok)->setText(QStringLiteral("修改并登录"));
	buttons->button(QDialogButtonBox::Cancel)->setText(QStringLiteral("取消登录"));
	connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
	connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
	layout->addWidget(buttons);

	while (dialog.exec() == QDialog::Accepted)
	{
		const QString newPassword = passwordEdit->text();
		if (newPassword.size() < 8)
		{
			QMessageBox::warning(&dialog, QStringLiteral("修改密码"), QStringLiteral("新密码至少需要 8 个字符。"));
			continue;
		}
		if (newPassword == temporaryPassword)
		{
			QMessageBox::warning(&dialog, QStringLiteral("修改密码"), QStringLiteral("新密码不能与临时密码相同。"));
			continue;
		}
		if (confirmEdit->text() != newPassword)
		{
			QMessageBox::warning(&dialog, QStringLiteral("修改密码"), QStringLiteral("两次输入的密码不一致。"));
			continue;
		}
		QString error;
		QString newPasswordRecord;
		if (!WriteAccountPasswordRecord(
				userName,
				newPassword,
				expectedPasswordRecord,
				true,
				false,
				newPasswordRecord,
				currentRole,
				securityFingerprint,
				error))
		{
			QMessageBox::critical(&dialog, QStringLiteral("修改密码"), error);
			continue;
		}
		ClearRememberedCredential(userName);
		replacementPassword = newPassword;
		return true;
	}
	return false;
}

void QtWidgetsApplication4::LoginCurrentAccount()
{
	if (m_bAccountRecoveryRequired)
	{
		QMessageBox::critical(this, "账号登录", "账号认证库处于安全恢复状态，已拒绝手工登录。");
		return;
	}
	if (m_bInitialAdministratorSetupRequired)
	{
		QMessageBox::information(
			this, "初始化管理员", "请先设置初始管理员密码，当前没有可用的默认登录凭据。");
		SetAuthRegisterMode(true);
		return;
	}
	if (m_pLoginNameEdit == nullptr || m_pLoginPasswordEdit == nullptr)
	{
		return;
	}

	QString role;
	bool mustChangePassword = false;
	QString error;
	QString verifiedPasswordRecord;
	const QString userName = m_pLoginNameEdit->text().trimmed();
	const QString submittedPassword = m_pLoginPasswordEdit->text();
	if (!VerifyAccount(
			userName,
			submittedPassword,
			role,
			mustChangePassword,
			verifiedPasswordRecord,
			error))
	{
		QMessageBox::warning(this, "账号登录", error);
		return;
	}
	if (mustChangePassword)
	{
		QString replacementPassword;
		QString changedRole;
		QString changedFingerprint;
		if (!PromptForcedPasswordChange(
				userName,
				submittedPassword,
				verifiedPasswordRecord,
				replacementPassword,
				changedRole,
				changedFingerprint))
		{
			m_sCurrentUserName = QStringLiteral("访客");
			m_sCurrentUserRole = kRoleOperator;
			m_sCurrentUserSecurityFingerprint.clear();
			m_pLoginPasswordEdit->clear();
			if (m_pRememberPasswordCheck != nullptr)
			{
				m_pRememberPasswordCheck->setChecked(false);
			}
			if (m_pAutoLoginCheck != nullptr)
			{
				m_pAutoLoginCheck->setChecked(false);
				m_pAutoLoginCheck->setEnabled(false);
			}
			WriteAppConfigValue(LoginStateGroup(), QStringLiteral("RememberPassword"), false);
			WriteAppConfigValue(LoginStateGroup(), QStringLiteral("AutoLogin"), false);
			RefreshAccountUi();
			ShowAuthPage(QStringLiteral("临时密码尚未修改，登录已取消。"));
			return;
		}
		m_pLoginPasswordEdit->setText(replacementPassword);
		role = changedRole;
		m_sCurrentUserSecurityFingerprint = changedFingerprint;
	}

	QString currentPasswordRecord;
	QString currentRole;
	QString currentFingerprint;
	bool currentMustChangePassword = true;
	if (!ConfigDatabase::TryReadAccountSecurityState(
			userName,
			&currentPasswordRecord,
			&currentRole,
			&currentMustChangePassword,
			&currentFingerprint)
		|| currentMustChangePassword
		|| (!mustChangePassword && currentPasswordRecord != verifiedPasswordRecord)
		|| (mustChangePassword && currentFingerprint != m_sCurrentUserSecurityFingerprint))
	{
		m_sCurrentUserSecurityFingerprint.clear();
		QMessageBox::warning(
			this,
			"账号登录",
			"账号在登录过程中已被删除、重置或修改权限，本次登录已取消，请重新输入密码。");
		ShowAuthPage(QStringLiteral("账号安全状态已变化，请重新登录。"));
		return;
	}
	role = currentRole;
	m_sCurrentUserSecurityFingerprint = currentFingerprint;

	m_sCurrentUserName = userName;
	m_sCurrentUserRole = role;
	m_bSessionRevocationPendingSafetyStop = false;
	if (m_bEnforceInteractiveSessionLeaseGate)
	{
		RobotOperationLease::SetNewOperationsAllowed(true);
	}
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
	if (m_bAccountRecoveryRequired)
	{
		QMessageBox::critical(this, "游客登录", "账号认证库处于安全恢复状态，已拒绝游客登录。");
		return;
	}
	if (m_bInitialAdministratorSetupRequired)
	{
		QMessageBox::critical(this, "游客登录", "必须先在本机完成初始管理员设置。游客登录已拒绝。");
		return;
	}
	m_sCurrentUserName = "游客";
	m_sCurrentUserRole = kRoleOperator;
	m_bSessionRevocationPendingSafetyStop = false;
	m_sCurrentUserSecurityFingerprint.clear();
	m_bPendingOpenManagementAfterLogin = false;
	if (m_bEnforceInteractiveSessionLeaseGate)
	{
		RobotOperationLease::SetNewOperationsAllowed(true);
	}
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
	if (RobotOperationLease::AnyActive() || HasRunningMeasureThenWeldFlow())
	{
		QMessageBox::warning(
			this,
			QStringLiteral("退出登录"),
			QStringLiteral("机器人硬件操作仍在运行。请先通过当前流程的安全停止/取消按钮结束操作，再退出登录。"));
		return;
	}
	m_sCurrentUserName = "访客";
	m_sCurrentUserRole = kRoleOperator;
	m_bSessionRevocationPendingSafetyStop = false;
	m_sCurrentUserSecurityFingerprint.clear();
	m_bPendingOpenManagementAfterLogin = false;
	if (m_bEnforceInteractiveSessionLeaseGate)
	{
		RobotOperationLease::SetNewOperationsAllowed(
			false,
			QStringLiteral("已退出登录；禁止启动新的机器人硬件操作。"));
	}
	RevokePrivilegedUiAccess();
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
	if (m_bAccountRecoveryRequired)
	{
		QMessageBox::critical(this, "注册账号", "账号认证库处于安全恢复状态，已拒绝注册新账号。");
		return;
	}
	if (m_pLoginNameEdit == nullptr || m_pLoginPasswordEdit == nullptr || m_pAuthConfirmPasswordEdit == nullptr)
	{
		return;
	}

	const bool initialAdministratorSetup = m_bInitialAdministratorSetupRequired;
	const QString role = initialAdministratorSetup ? kRoleAdmin : kRoleOperator;
	const QString normalizedUser = initialAdministratorSetup
		? QStringLiteral("admin")
		: m_pLoginNameEdit->text().trimmed();
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
	const bool saved = initialAdministratorSetup
		? WriteNewAccountRecord(
			normalizedUser, password, role, false, error, true)
		: SaveAccount(normalizedUser, password, role, error);
	if (!saved)
	{
		if (initialAdministratorSetup)
		{
			EnsureDefaultAdminAccount();
			RefreshAccountUi();
			ShowAuthPage(m_bInitialAdministratorSetupRequired
				? QStringLiteral("初始管理员写入失败，请检查配置库后重试。")
				: QStringLiteral("初始管理员已由另一进程完成设置，请使用现有账号登录。"));
		}
		QMessageBox::warning(
			this,
			initialAdministratorSetup ? QStringLiteral("初始化管理员") : QStringLiteral("注册账号"),
			error);
		return;
	}

	if (m_pAccountLogText != nullptr)
	{
		m_pAccountLogText->appendPlainText(QString("[%1] %2账号 %3，权限：%4")
			.arg(QDateTime::currentDateTime().toString("HH:mm:ss"),
				initialAdministratorSetup ? QStringLiteral("已初始化管理员") : QStringLiteral("已注册"),
				normalizedUser,
				RoleDisplayName(role)));
	}
	m_pAuthConfirmPasswordEdit->clear();
	if (initialAdministratorSetup)
	{
		m_bInitialAdministratorSetupRequired = false;
		m_pLoginNameEdit->setText(normalizedUser);
		RefreshAccountUi();
		SetAuthRegisterMode(false);
		LoginCurrentAccount();
		return;
	}
	m_pLoginPasswordEdit->clear();
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
	PageOpenTrace trace("账号管理");
	if (!ValidateCurrentAccountSession(QStringLiteral("打开账号管理")))
	{
		return;
	}
	if (RoleLevel(m_sCurrentUserRole) < RoleLevel(kRoleAdmin))
	{
		QMessageBox::information(this, "账号管理", "账号管理仅管理员可用。");
		return;
	}

	if (m_pManagementStack == nullptr)
	{
		CloseGrooveCameraPreviewWindow();
		AccountManagementDialog dialog(m_sCurrentUserName, this);
		dialog.exec();
		return;
	}

	if (m_pAccountManagementPage != nullptr)
	{
		m_pManagementStack->removeWidget(m_pAccountManagementPage);
		delete m_pAccountManagementPage;
		m_pAccountManagementPage = nullptr;
	}

	m_pAccountManagementPage = new AccountManagementDialog(m_sCurrentUserName, m_pManagementStack);
	PrepareEmbeddedPage(m_pAccountManagementPage, m_pManagementStack);
	ShowManagementEmbeddedPage(m_pAccountManagementPage);
}

void QtWidgetsApplication4::OpenControlUnitManagementDialog()
{
	PageOpenTrace trace("控制单元管理");
	if (!ValidateCurrentAccountSession(QStringLiteral("打开控制单元管理")))
	{
		return;
	}
	if (RoleLevel(m_sCurrentUserRole) < RoleLevel(kRoleEngineer))
	{
		QMessageBox::information(this, "控制单元管理", "控制单元管理需要工程师或管理员权限。");
		return;
	}

	auto reloadControlUnits = [this]()
		{
			// 重载会销毁并重建机器人驱动；若有焊接/虚拟焊道流程正在后台线程驱动机器人，重载会造成悬垂指针/并发，必须先拦截。
			if (RobotOperationLease::AnyActive()
				|| HasRunningMeasureThenWeldFlow()
				|| (m_pVirtualWeldTestPage != nullptr && m_pVirtualWeldTestPage->IsRunning()))
			{
				QMessageBox::information(this, "控制单元管理",
					QString("机器人硬件操作正在运行：%1。\n请等待安全结束后再重新加载控制单元。")
						.arg(RobotOperationLease::ActiveSummary()));
				return;
			}

			// 所有缓存 driver/camera 指针的页面必须在重载前销毁；即使页面当前空闲，
			// InitContralUnit/StopScanCameraRuntimes 后继续复用也会指向已释放对象。
			for (const QPointer<MeasureThenWeldDialog>& guardedPage : m_measureThenWeldPages.values())
			{
				delete guardedPage.data();
			}
			m_measureThenWeldPages.clear();
			m_pMeasureThenWeldPage = nullptr;
			m_nMeasureThenWeldPageUnitIndex = -1;
			for (const QPointer<RobotJogDialog>& guardedPage : m_robotJogPages.values())
			{
				delete guardedPage.data();
			}
			m_robotJogPages.clear();
			m_pRobotJogPage = nullptr;
			m_nRobotJogPageUnitIndex = -1;
			delete m_pFunctionTestPage;
			m_pFunctionTestPage = nullptr;
			m_nFunctionTestPageUnitIndex = -1;
			delete m_pVirtualWeldTestPage;
			m_pVirtualWeldTestPage = nullptr;
			m_nVirtualWeldTestPageUnitIndex = -1;
			delete m_pProcessLoopTestPage;
			m_pProcessLoopTestPage = nullptr;
			delete m_pFtpJobManagementPage;
			m_pFtpJobManagementPage = nullptr;
			m_nFtpJobManagementPageUnitIndex = -1;
			delete m_pCameraParamPage;
			m_pCameraParamPage = nullptr;
			m_sCameraParamPageRobotName.clear();
			delete m_pPreciseMeasureEditPage;
			m_pPreciseMeasureEditPage = nullptr;
			CloseGrooveCameraPreviewWindow();
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
			CloseGrooveCameraPreviewWindow();
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
			auto stopCamera = [this]()
				{
					CloseGrooveCameraPreviewWindow();
				};

			CloseGrooveCameraPreviewWindow();
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
		CloseGrooveCameraPreviewWindow();
		ControlUnitManagementDialog dialog(reloadControlUnits, openCameraBasicParam, openHandEyeCalibration, this);
		dialog.exec();
		return;
	}

	if (m_pControlUnitManagementPage != nullptr)
	{
		ShowManagementEmbeddedPage(m_pControlUnitManagementPage);
		return;
	}

	m_pControlUnitManagementPage = new ControlUnitManagementDialog(reloadControlUnits, openCameraBasicParam, openHandEyeCalibration, m_pManagementStack);
	PrepareEmbeddedPage(m_pControlUnitManagementPage, m_pManagementStack);
	ShowManagementEmbeddedPage(m_pControlUnitManagementPage);
}

void QtWidgetsApplication4::OpenFtpJobManagementDialog()
{
	PageOpenTrace trace("FTP Job 文件");
	if (!ValidateCurrentAccountSession(QStringLiteral("打开 FTP Job 文件管理")))
	{
		return;
	}
	if (RoleLevel(m_sCurrentUserRole) < RoleLevel(kRoleEngineer))
	{
		QMessageBox::information(this, "FTP Job 文件", "FTP Job 文件管理需要工程师或管理员权限。");
		return;
	}
	const auto liveSessionGuard = [this]()
	{
		return ValidateCurrentAccountSession(QStringLiteral("提交 FTP Job 文件操作"))
			&& RoleLevel(m_sCurrentUserRole) >= RoleLevel(kRoleEngineer);
	};

	const int currentUnitIndex = CurrentRobotUnitIndex();
	if (m_pManagementStack == nullptr)
	{
		CloseGrooveCameraPreviewWindow();
		FtpJobManagementDialog dialog(
			m_pContralUnit,
			currentUnitIndex,
			liveSessionGuard,
			this);
		dialog.exec();
		return;
	}

	if (m_pFtpJobManagementPage != nullptr && m_nFtpJobManagementPageUnitIndex != currentUnitIndex)
	{
		FtpJobManagementDialog* ftpPage = static_cast<FtpJobManagementDialog*>(m_pFtpJobManagementPage);
		if (ftpPage->IsBusy())
		{
			QMessageBox::information(this, "FTP Job 文件",
				"FTP 操作仍在执行，完成前不能切换控制单元或销毁页面。");
			ShowManagementEmbeddedPage(m_pFtpJobManagementPage);
			return;
		}
		m_pManagementStack->removeWidget(m_pFtpJobManagementPage);
		m_pFtpJobManagementPage->deleteLater();
		m_pFtpJobManagementPage = nullptr;
		m_nFtpJobManagementPageUnitIndex = -1;
	}
	if (m_pFtpJobManagementPage != nullptr)
	{
		ShowManagementEmbeddedPage(m_pFtpJobManagementPage);
		return;
	}

	m_pFtpJobManagementPage = new FtpJobManagementDialog(
		m_pContralUnit,
		currentUnitIndex,
		liveSessionGuard,
		m_pManagementStack);
	m_nFtpJobManagementPageUnitIndex = currentUnitIndex;
	PrepareEmbeddedPage(m_pFtpJobManagementPage, m_pManagementStack);
	ShowManagementEmbeddedPage(m_pFtpJobManagementPage);
}

void QtWidgetsApplication4::OpenPrecisePointCloudProcessingPage()
{
	PageOpenTrace trace("精测点云处理");
	if (!ValidateCurrentAccountSession(QStringLiteral("打开精测点云处理")))
	{
		return;
	}
	if (RoleLevel(m_sCurrentUserRole) < RoleLevel(kRoleEngineer))
	{
		QMessageBox::information(this, "精测点云处理", "精测点云处理需要工程师或管理员权限。");
		return;
	}

	if (m_pManagementStack == nullptr)
	{
		QMessageBox::warning(this, "精测点云处理", "管理页面尚未初始化，无法嵌入精测点云处理页面。");
		return;
	}

	if (m_pPrecisePointCloudProcessingPage != nullptr)
	{
		ShowManagementEmbeddedPage(m_pPrecisePointCloudProcessingPage);
		return;
	}

	const auto liveSessionGuard = [this]()
	{
		// 角色先行短路，避免撤权关闭页面触发 finished->SaveSettings 时
		// 再次进入会话撤销 UI；原生选择框返回时角色仍旧则继续同步查库。
		return RoleLevel(m_sCurrentUserRole) >= RoleLevel(kRoleEngineer)
			&& ValidateCurrentAccountSession(QStringLiteral("保存精测点云与外部库设置"))
			&& RoleLevel(m_sCurrentUserRole) >= RoleLevel(kRoleEngineer);
	};
	m_pPrecisePointCloudProcessingPage = new LaserWeldFilterDialog(
		liveSessionGuard,
		m_pManagementStack);
	PrepareEmbeddedPage(m_pPrecisePointCloudProcessingPage, m_pManagementStack);
	ShowManagementEmbeddedPage(m_pPrecisePointCloudProcessingPage);
}

void QtWidgetsApplication4::OpenModelAlignmentPage()
{
	PageOpenTrace trace("模型配准");
	if (!ValidateCurrentAccountSession(QStringLiteral("打开模型配准")))
	{
		return;
	}
	if (RoleLevel(m_sCurrentUserRole) < RoleLevel(kRoleEngineer))
	{
		QMessageBox::information(this, "模型配准", "模型配准 / 点云去噪需要工程师或管理员权限。");
		return;
	}

	if (m_pManagementStack == nullptr)
	{
		QMessageBox::warning(this, "模型配准", "管理页面尚未初始化，无法嵌入模型配准页面。");
		return;
	}

	if (m_pModelAlignmentPage != nullptr)
	{
		ShowManagementEmbeddedPage(m_pModelAlignmentPage);
		return;
	}

	m_pModelAlignmentPage = new ModelAlignmentDialog(m_pManagementStack);
	PrepareEmbeddedPage(m_pModelAlignmentPage, m_pManagementStack);
	ShowManagementEmbeddedPage(m_pModelAlignmentPage);
}

void QtWidgetsApplication4::OpenWorkpieceMeshPage()
{
	PageOpenTrace trace("工件模型");
	if (!ValidateCurrentAccountSession(QStringLiteral("打开工件模型")))
	{
		return;
	}
	if (RoleLevel(m_sCurrentUserRole) < RoleLevel(kRoleEngineer))
	{
		QMessageBox::information(this, "工件模型", "工件模型查看需要工程师或管理员权限。");
		return;
	}

	if (m_pManagementStack == nullptr)
	{
		QMessageBox::warning(this, "工件模型", "管理页面尚未初始化，无法嵌入工件模型页面。");
		return;
	}

	if (m_pWorkpieceMeshPage != nullptr)
	{
		ShowManagementEmbeddedPage(m_pWorkpieceMeshPage);
		return;
	}

	m_pWorkpieceMeshPage = new WorkpieceMeshViewerDialog(m_pManagementStack);
	PrepareEmbeddedPage(m_pWorkpieceMeshPage, m_pManagementStack);
	ShowManagementEmbeddedPage(m_pWorkpieceMeshPage);
}

void QtWidgetsApplication4::OpenVirtualWeldTestPage()
{
	PageOpenTrace trace("虚拟焊道测试");
	if (!ValidateCurrentAccountSession(QStringLiteral("打开虚拟焊道测试")))
	{
		return;
	}
	if (RoleLevel(m_sCurrentUserRole) < RoleLevel(kRoleEngineer))
	{
		QMessageBox::information(this, "虚拟焊道测试", "虚拟焊道测试需要工程师或管理员权限。");
		return;
	}

	if (m_pManagementStack == nullptr)
	{
		QMessageBox::warning(this, "虚拟焊道测试", "管理页面尚未初始化，无法嵌入虚拟焊道测试页面。");
		return;
	}

	const int currentUnitIndex = CurrentRobotUnitIndex();
	// 运行中禁止因切换机器人而删除正在驱动机器人的页面（避免线程脱离页面继续驱动/并发下发）。
	if (m_pVirtualWeldTestPage != nullptr
		&& m_nVirtualWeldTestPageUnitIndex != currentUnitIndex
		&& m_pVirtualWeldTestPage->IsRunning())
	{
		QMessageBox::information(this, "虚拟焊道测试",
			"虚拟焊道测试流程正在运行，请等待流程结束后再切换机器人。");
		ShowManagementEmbeddedPage(m_pVirtualWeldTestPage);
		return;
	}
	if (m_pVirtualWeldTestPage != nullptr && m_nVirtualWeldTestPageUnitIndex != currentUnitIndex)
	{
		if (m_pManagementStack->indexOf(m_pVirtualWeldTestPage) >= 0)
		{
			m_pManagementStack->removeWidget(m_pVirtualWeldTestPage);
		}
		delete m_pVirtualWeldTestPage;
		m_pVirtualWeldTestPage = nullptr;
	}

	if (m_pVirtualWeldTestPage != nullptr)
	{
		ShowManagementEmbeddedPage(m_pVirtualWeldTestPage);
		return;
	}

	m_pVirtualWeldTestPage = new VirtualWeldTestDialog(m_pContralUnit, currentUnitIndex, m_pManagementStack);
	m_nVirtualWeldTestPageUnitIndex = currentUnitIndex;
	PrepareEmbeddedPage(m_pVirtualWeldTestPage, m_pManagementStack);
	ShowManagementEmbeddedPage(m_pVirtualWeldTestPage);
}

void QtWidgetsApplication4::OpenConfigDatabaseViewerDialog()
{
	PageOpenTrace trace("配置数据库查看");
	if (!ValidateCurrentAccountSession(QStringLiteral("打开配置数据库查看")))
	{
		return;
	}
	if (RoleLevel(m_sCurrentUserRole) < RoleLevel(kRoleAdmin))
	{
		QMessageBox::information(this, "配置数据库查看", "配置数据库查看仅管理员可用。");
		return;
	}

	if (m_pManagementStack == nullptr)
	{
		CloseGrooveCameraPreviewWindow();
		ConfigDatabaseViewerDialog dialog(this);
		dialog.exec();
		return;
	}

	if (m_pConfigDatabaseViewerPage != nullptr)
	{
		ShowManagementEmbeddedPage(m_pConfigDatabaseViewerPage);
		return;
	}

	m_pConfigDatabaseViewerPage = new ConfigDatabaseViewerDialog(m_pManagementStack);
	PrepareEmbeddedPage(m_pConfigDatabaseViewerPage, m_pManagementStack);
	ShowManagementEmbeddedPage(m_pConfigDatabaseViewerPage);
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
		ConfigureResponsiveScrollArea(scrollArea);
		if (QWidget* scrollWidget = scrollArea->widget())
		{
			if (QLayout* scrollLayout = scrollWidget->layout())
			{
				scrollLayout->setSizeConstraint(QLayout::SetMinAndMaxSize);
			}
		}
	}

	page->installEventFilter(this);
	if (targetStack->indexOf(page) < 0)
	{
		targetStack->addWidget(page);
	}
	ApplyResponsivePageDefaults(page);
	ApplyDebugLogVisibility(page);
	QTimer::singleShot(80, page, [page]() { ApplyResponsivePageDefaults(page); });
	QPointer<QWidget> pagePtr(page);
	QTimer::singleShot(80, this, [this, pagePtr]() { ApplyDebugLogVisibility(pagePtr); });
}

void QtWidgetsApplication4::ShowEmbeddedPage(QWidget* page)
{
	if (page == nullptr || m_pMainStack == nullptr)
	{
		return;
	}
	if (m_pMainStack->currentWidget() != page)
	{
		CloseGrooveCameraPreviewWindow();
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
	if (!ValidateCurrentAccountSession(QStringLiteral("打开管理功能页"))
		|| RoleLevel(m_sCurrentUserRole) < RoleLevel(kRoleEngineer))
	{
		RevokePrivilegedUiAccess();
		return;
	}
	if (m_pManagementStack->currentWidget() != page)
	{
		CloseGrooveCameraPreviewWindow();
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
	// 主页版本号点开的「关于」：在线服务对话框的精简形态（软件名/当前版本/最新版本/
	// 更新内容 + 一键升级），打开即自动检查更新。无权限门槛——升级安装自带确认与流程守卫。
	if (OnlineServicesDialog* existing = findChild<OnlineServicesDialog*>(QStringLiteral("OnlineServicesDialogAbout")))
	{
		existing->show();
		existing->raise();
		existing->activateWindow();
		return;
	}
	auto* dlg = new OnlineServicesDialog(
		EnsureScanDataUploader(),
		[this]() { return RobotOperationLease::AnyActive() || HasRunningMeasureThenWeldFlow(); },
		true,
		false,
		{},
		this);
	dlg->setObjectName(QStringLiteral("OnlineServicesDialogAbout"));
	dlg->setAttribute(Qt::WA_DeleteOnClose);
	dlg->show();
}

void QtWidgetsApplication4::ApplyStartupArguments(const QStringList& arguments)
{
	// GUI 启动自检：上次若有「待验证升级」，比对实际版本是否达到目标；不符则告警（补丁/安装没生效）。
	if (!arguments.contains(QStringLiteral("--no-show")))
	{
		QTimer::singleShot(1500, this, [this]() { CheckPendingUpdateResult(); });
	}

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

void QtWidgetsApplication4::CheckPendingUpdateResult()
{
	const QString target = OnlineServicesConfig::PendingUpdateTargetVersion();
	if (target.isEmpty())
	{
		return;  // 无待验证升级
	}
	const QString current = QApplication::applicationVersion();
	OnlineServicesConfig::SetPendingUpdateTargetVersion(QString());  // 无论成败都清除，避免反复弹
	if (current == target)
	{
		// updater 正等待新进程的健康握手。只有目标版本稳定运行 15 秒才原子写 health；
		// 批处理在最长 30 秒内同时监控进程与该标记，提前退出会回滚旧 exe，成功后才由
		// 批处理清 failure marker/backup。应用本身不能提前删除恢复件。
		QTimer::singleShot(15000, this, [target]()
			{
				if (QApplication::applicationVersion() != target)
				{
					return;
				}
				const QString healthMarker = AppPaths::WritableChildPath(
					QStringLiteral("Temp/OnlineUpdate"),
					QStringLiteral("patch_healthy_%1.flag").arg(target));
				if (healthMarker.isEmpty())
				{
					return;
				}
				QSaveFile marker(healthMarker);
				const QByteArray content = target.toUtf8() + '\n';
				if (!marker.open(QIODevice::WriteOnly)
					|| marker.write(content) != content.size()
					|| !marker.commit())
				{
					marker.cancelWriting();
				}
			});
		return;  // 升级成功（版本已达目标），静默
	}
	// 版本没达到目标：补丁/安装没真正生效（如误挂了含旧 exe 的补丁——1004 那类问题）。
	QMessageBox::warning(this, QStringLiteral("升级未生效"),
		QStringLiteral("上次升级似乎未成功：期望升级到 %1，当前仍是 %2。\n\n"
			"建议在「在线服务 / 关于」里改用「完整安装包」重新升级；若多次仍失败，请联系维护人员。")
			.arg(target, current));
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
		"--rebuild-measure-weld-files",
		"--pointcloud-processing-mode",
		"--pointcloud-scan-direction",
		"--apply-weld-seam-comp",
		"--generate-step-weld-program",
		"--update-weld-pose-average",
		"--test-pointwise-weave"
		});

	if (arguments.contains("--help-cli"))
	{
		QTextStream out(stdout);
		ConfigureUtf8TextStream(out);
		WriteCliHelp(out);
		out.flush();
		QTimer::singleShot(0, QCoreApplication::instance(), &QCoreApplication::quit);
		return;
	}

	const int pointCloudModeIndex = arguments.indexOf("--pointcloud-processing-mode");
	if (pointCloudModeIndex >= 0 && pointCloudModeIndex + 1 >= arguments.size())
	{
		LogCommandLineMessage("CLI 点云处理方式覆盖缺少参数，请使用 sdk/sdkfit/cloudfit/legacy。");
		QCoreApplication::exit(1);
		return;
	}
	const QString pointCloudModeOverride = CliOptionValue(arguments, "--pointcloud-processing-mode").trimmed();
	if (!pointCloudModeOverride.isEmpty())
	{
		const QString normalizedMode = pointCloudModeOverride.toLower();
		if (normalizedMode == "sdk"
			|| normalizedMode == "sdkfull"
			|| normalizedMode == "external"
			|| normalizedMode == "externalcorrugatedsheet"
			|| normalizedMode == "pointcloud")
		{
			PointCloudProcessingConfig::SetRuntimeModeOverride(
				PointCloudProcessingConfig::Mode::ExternalCorrugatedSheet);
			LogCommandLineMessage("CLI 点云处理方式已临时覆盖为：SDK点云算法全处理。");
		}
		else if (normalizedMode == "sdkfit"
			|| normalizedMode == "sdkbaseweldfit"
			|| normalizedMode == "sdkbaseweld")
		{
			PointCloudProcessingConfig::SetRuntimeModeOverride(
				PointCloudProcessingConfig::Mode::SdkBaseWeldFit);
			LogCommandLineMessage("CLI 点云处理方式已临时覆盖为：SDK点云算法+拟合。");
		}
		else if (normalizedMode == "cloudfit"
			|| normalizedMode == "cloud")
		{
			PointCloudProcessingConfig::SetRuntimeModeOverride(
				PointCloudProcessingConfig::Mode::CloudFit);
			LogCommandLineMessage("CLI 点云处理方式已临时覆盖为：点云算法+拟合。");
		}
		else if (normalizedMode == "legacy"
			|| normalizedMode == "old"
			|| normalizedMode == "feature"
			|| normalizedMode == "featurepoint"
			|| normalizedMode == "legacylaserpath")
		{
			PointCloudProcessingConfig::SetRuntimeModeOverride(
				PointCloudProcessingConfig::Mode::LegacyLaserPath);
			LogCommandLineMessage("CLI 点云处理方式已临时覆盖为：特征点+拟合。");
		}
		else
		{
			LogCommandLineMessage(QString("CLI 点云处理方式覆盖无效：%1，请使用 sdk/sdkfit/cloudfit/legacy。")
				.arg(pointCloudModeOverride));
			QCoreApplication::exit(1);
			return;
		}
	}

	const int pointCloudScanDirectionIndex = arguments.indexOf("--pointcloud-scan-direction");
	if (pointCloudScanDirectionIndex >= 0 && pointCloudScanDirectionIndex + 1 >= arguments.size())
	{
		LogCommandLineMessage("CLI 点云SDK扫描方向覆盖缺少参数，请使用 X,Y,Z。");
		QCoreApplication::exit(1);
		return;
	}
	const QString pointCloudScanDirectionOverride = CliOptionValue(arguments, "--pointcloud-scan-direction").trimmed();
	if (!pointCloudScanDirectionOverride.isEmpty())
	{
		const QStringList directionParts = pointCloudScanDirectionOverride.split(',', Qt::SkipEmptyParts);
		if (directionParts.size() == 3)
		{
			bool okX = false;
			bool okY = false;
			bool okZ = false;
			const double x = directionParts[0].trimmed().toDouble(&okX);
			const double y = directionParts[1].trimmed().toDouble(&okY);
			const double z = directionParts[2].trimmed().toDouble(&okZ);
			if (okX && okY && okZ)
			{
				PointCloudProcessingConfig::SetRuntimeScanDirectionOverride(x, y, z);
				LogCommandLineMessage(QString("CLI 点云SDK扫描方向已临时覆盖为：(%1, %2, %3)。")
					.arg(x, 0, 'f', 6)
					.arg(y, 0, 'f', 6)
					.arg(z, 0, 'f', 6));
			}
			else
			{
				LogCommandLineMessage(QString("CLI 点云SDK扫描方向覆盖无效：%1，请使用 X,Y,Z 数值格式。")
					.arg(pointCloudScanDirectionOverride));
				QCoreApplication::exit(1);
				return;
			}
		}
		else
		{
			LogCommandLineMessage(QString("CLI 点云SDK扫描方向覆盖无效：%1，请使用 X,Y,Z 格式。")
				.arg(pointCloudScanDirectionOverride));
			QCoreApplication::exit(1);
			return;
		}
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
	if (laserClassifyIndex >= 0)
	{
		if (laserClassifyIndex + 1 >= arguments.size())
		{
			LogCommandLineMessage("CLI 激光点云分类失败：--laser-classify 缺少文件参数。");
			QCoreApplication::exit(1);
			return;
		}
		const QString inputPath = arguments[laserClassifyIndex + 1];
		QString outputPath;
		const int laserOutputIndex = arguments.indexOf("--laser-classify-output");
		if (laserOutputIndex >= 0)
		{
			if (laserOutputIndex + 1 >= arguments.size())
			{
				LogCommandLineMessage("CLI 激光点云分类失败：--laser-classify-output 缺少文件参数。");
				QCoreApplication::exit(1);
				return;
			}
			outputPath = arguments[laserOutputIndex + 1];
		}
		if (!RunLaserClassifyForCli(inputPath, outputPath))
		{
			QCoreApplication::exit(1);
			return;
		}
	}

	const int laserClassifyDirIndex = arguments.indexOf("--laser-classify-dir");
	if (laserClassifyDirIndex >= 0)
	{
		if (laserClassifyDirIndex + 1 >= arguments.size()
			|| !RunLaserClassifyDirForCli(arguments[laserClassifyDirIndex + 1]))
		{
			if (laserClassifyDirIndex + 1 >= arguments.size())
			{
				LogCommandLineMessage("CLI 批量激光点云分类失败：--laser-classify-dir 缺少目录参数。");
			}
			QCoreApplication::exit(1);
			return;
		}
	}

	const int rebuildMeasureWeldIndex = arguments.indexOf("--rebuild-measure-weld-files");
	if (rebuildMeasureWeldIndex >= 0)
	{
		if (rebuildMeasureWeldIndex + 1 >= arguments.size()
			|| !RunRebuildMeasureWeldFilesForCli(arguments, arguments[rebuildMeasureWeldIndex + 1]))
		{
			if (rebuildMeasureWeldIndex + 1 >= arguments.size())
			{
				LogCommandLineMessage("CLI 先测后焊重建失败：--rebuild-measure-weld-files 缺少目录参数。");
			}
			QCoreApplication::exit(1);
			return;
		}
	}

	const int weldSeamCompIndex = arguments.indexOf("--apply-weld-seam-comp");
	if (weldSeamCompIndex >= 0)
	{
		if (weldSeamCompIndex + 1 >= arguments.size())
		{
			LogCommandLineMessage("CLI 焊道补偿失败：--apply-weld-seam-comp 缺少文件参数。");
			QCoreApplication::exit(1);
			return;
		}
		const QString inputPath = arguments[weldSeamCompIndex + 1];
		QString outputPath;
		const int weldSeamCompOutputIndex = arguments.indexOf("--apply-weld-seam-comp-output");
		if (weldSeamCompOutputIndex >= 0)
		{
			if (weldSeamCompOutputIndex + 1 >= arguments.size())
			{
				LogCommandLineMessage("CLI 焊道补偿失败：--apply-weld-seam-comp-output 缺少文件参数。");
				QCoreApplication::exit(1);
				return;
			}
			outputPath = arguments[weldSeamCompOutputIndex + 1];
		}
		if (!RunWeldSeamCompForCli(arguments, inputPath, outputPath))
		{
			QCoreApplication::exit(1);
			return;
		}
	}

	const int generateStepWeldIndex = arguments.indexOf("--generate-step-weld-program");
	if (generateStepWeldIndex >= 0)
	{
		if (generateStepWeldIndex + 1 >= arguments.size())
		{
			LogCommandLineMessage("CLI STEP 焊接程序生成失败：--generate-step-weld-program 缺少文件参数。");
			QCoreApplication::exit(1);
			return;
		}
		const QString inputPath = arguments[generateStepWeldIndex + 1];
		QString outputDir;
		const int stepOutputDirIndex = arguments.indexOf("--generate-step-weld-program-output-dir");
		if (stepOutputDirIndex >= 0)
		{
			if (stepOutputDirIndex + 1 >= arguments.size())
			{
				LogCommandLineMessage("CLI STEP 焊接程序生成失败：--generate-step-weld-program-output-dir 缺少目录参数。");
				QCoreApplication::exit(1);
				return;
			}
			outputDir = arguments[stepOutputDirIndex + 1];
		}

		double weldSpeedMmPerMin = 0.0;
		const int stepSpeedIndex = arguments.indexOf("--generate-step-weld-speed");
		if (stepSpeedIndex >= 0)
		{
			if (stepSpeedIndex + 1 >= arguments.size())
			{
				LogCommandLineMessage("CLI STEP 焊接程序生成失败：--generate-step-weld-speed 缺少速度参数。");
				QCoreApplication::exit(1);
				return;
			}
			bool ok = false;
			weldSpeedMmPerMin = arguments[stepSpeedIndex + 1].toDouble(&ok);
			if (!ok || !std::isfinite(weldSpeedMmPerMin) || weldSpeedMmPerMin <= 0.0)
			{
				LogCommandLineMessage("CLI STEP焊接程序速度无效，将使用当前工艺速度或默认速度。");
				weldSpeedMmPerMin = 0.0;
			}
		}

		const bool actualWeld = !arguments.contains("--generate-step-weld-program-dry-run");
		if (!RunGenerateStepWeldProgramForCli(arguments, inputPath, outputDir, actualWeld, weldSpeedMmPerMin))
		{
			QCoreApplication::exit(1);
			return;
		}
	}

	const int testPointwiseWeaveIndex = arguments.indexOf("--test-pointwise-weave");
	if (testPointwiseWeaveIndex >= 0)
	{
		// 离线测试 pointwise 摆动：造一条直线中心线，跑真实摆动算法，输出点位 txt（不连机器人）
		auto argDoubleAt = [&](int offset, double def) -> double {
			const int idx = testPointwiseWeaveIndex + offset;
			if (idx < arguments.size())
			{
				bool ok = false;
				const double v = arguments[idx].toDouble(&ok);
				if (ok) { return v; }
			}
			return def;
		};
		const int shape = static_cast<int>(argDoubleAt(1, 5.0));   // 默认 eSin=5
		const double amplitude = argDoubleAt(2, 3.0);
		const double freqHz = argDoubleAt(3, 2.0);
		const double speedMmPerMin = 300.0;
		const double lineLengthMm = 200.0;
		const int nCenterPts = 6;

		std::vector<T_ROBOT_MOVE_INFO> centerline;
		for (int i = 0; i < nCenterPts; ++i)
		{
			T_ROBOT_MOVE_INFO p;
			p.tCoord.dX = 1000.0;
			p.tCoord.dY = lineLengthMm * i / (nCenterPts - 1);
			p.tCoord.dZ = 200.0;
			p.tCoord.dRX = -180.0; p.tCoord.dRY = 30.0; p.tCoord.dRZ = -90.0;
			p.nMoveType = MOVL;
			p.bWeldProcessEnabled = true;
			p.bHasWeaveParam = true;
			p.bAppPointwiseWeave = true;
			p.dWeldSpeedMmPerMin = speedMmPerMin;
			p.tWeaveParam.nWeaveShape = shape;
			p.tWeaveParam.dWeaveAmplitudeMm = amplitude;
			p.tWeaveParam.dWeaveFrequencyHz = freqHz;
			centerline.push_back(p);
		}

		std::string weaveErr;
		std::vector<T_ROBOT_MOVE_INFO> weave =
			RobotDriverAdaptor::ExpandMoveInfosByPointwiseWeave(centerline, &weaveErr);
		std::string speedInfo;
		weave = RobotDriverAdaptor::ApplyWeaveSpeedCompensation(centerline, weave, 0.0, &speedInfo);

		const auto writePts = [](const QString& path, const std::vector<T_ROBOT_MOVE_INFO>& pts) {
			QFile f(path);
			if (f.open(QIODevice::WriteOnly | QIODevice::Text))
			{
				QTextStream ts(&f);
				for (const auto& p : pts)
				{
					ts << p.tCoord.dX << " " << p.tCoord.dY << " " << p.tCoord.dZ << " "
						<< p.tCoord.dRX << " " << p.tCoord.dRY << " " << p.tCoord.dRZ << "\n";
				}
			}
		};
		writePts("WeaveTest_centerline.txt", centerline);
		writePts("WeaveTest_weave.txt", weave);

		QString msg = QString("CLI pointwise 摆动测试：shape=%1 摆幅=%2mm 频率=%3Hz → 中心线 %4 点 / 摆动 %5 点。")
			.arg(shape).arg(amplitude).arg(freqHz)
			.arg(static_cast<int>(centerline.size())).arg(static_cast<int>(weave.size()));
		if (!weaveErr.empty()) { msg += QString::fromStdString(" 错误:" + weaveErr); }
		if (!speedInfo.empty()) { msg += " " + QString::fromStdString(speedInfo); }
		msg += " 输出 WeaveTest_centerline.txt / WeaveTest_weave.txt";
		LogCommandLineMessage(msg);
	}

	const int updateWeldPoseAverageIndex = arguments.indexOf("--update-weld-pose-average");
	if (updateWeldPoseAverageIndex >= 0 && updateWeldPoseAverageIndex + 1 < arguments.size())
	{
		RunUpdateWeldPoseAverageForCli(arguments[updateWeldPoseAverageIndex + 1]);
	}

	RunRobotMotionForCli(arguments);

	if (scanOnlyRepeatCount > 0)
	{
		QString robotLabel;
		int scanUnitIndex = -1;
		RobotDriverAdaptor* scanDriver = GetRobotDriverForCli(arguments, &robotLabel, &scanUnitIndex);
		if (scanDriver == nullptr)
		{
			LogCommandLineMessage("CLI 先测后焊扫描失败：未找到可用机器人。");
		}
		else
		{
			bool connected = scanDriver->IsConnected();
			if (!connected)
			{
				connected = scanDriver->InitSocket(scanDriver->m_sSocketIP.c_str(), static_cast<unsigned short>(scanDriver->m_nSocketPort));
			}
			LogCommandLineMessage(QString("CLI 先测后焊扫描机器人连接%1：%2，地址=%3:%4")
				.arg(connected ? "成功" : "失败")
				.arg(robotLabel)
				.arg(QString::fromStdString(scanDriver->m_sSocketIP))
				.arg(scanDriver->m_nSocketPort));
			if (!connected)
			{
				const QString lastError = DecodeRobotMessageText(scanDriver->GetLastRobotError());
				if (!lastError.isEmpty())
				{
					LogCommandLineMessage(QString("CLI 先测后焊扫描机器人连接错误：%1").arg(lastError));
				}
			}
			else
			{
				const bool scanOk = RunMeasureThenWeldScanOnlyRepeatForCli(
					scanDriver,
					scanUnitIndex,
					scanOnlyRepeatCount,
					scanSpeedOverrideMmPerMin,
					cameraTimeOffsetOverrideMs);
				LogCommandLineMessage(QString("CLI 先测后焊扫描重复流程%1。").arg(scanOk ? "完成" : "失败"));
				// CLI 即将随动作结束退出：限时等待本轮上传收尾（成功清空或失败落回队列均结束等待；
				// 未传完的留在持久化队列，下次启动自动续传）。
				if (m_pScanDataUploader != nullptr && m_pScanDataUploader->IsBusy())
				{
					LogCommandLineMessage("正在等待扫描数据上传收尾（最多 3 分钟）…");
					QElapsedTimer uploadWait;
					uploadWait.start();
					while (m_pScanDataUploader->IsBusy() && uploadWait.elapsed() < 180000)
					{
						QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
						QThread::msleep(100);
					}
					LogCommandLineMessage(QString("上传收尾结束，剩余待传 %1 项。")
						.arg(m_pScanDataUploader->PendingList().size()));
				}
			}
		}
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
	RobotOperationLease::Ptr fanucCliLease;
	if (pFanucDriver != nullptr && needsFanuc)
	{
		QString leaseError;
		fanucCliLease = RobotOperationLease::TryAcquire(
			pFanucDriver, QStringLiteral("CLI FANUC 硬件操作"), &leaseError);
		if (!fanucCliLease)
		{
			LogCommandLineMessage("CLI FANUC 操作已被互锁拦截：" + leaseError);
			pFanucDriver = nullptr;
		}
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
			socketReady = pFanucDriver->InitSocket(pFanucDriver->m_sSocketIP.c_str(), static_cast<unsigned short>(pFanucDriver->m_nSocketPort));
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
				const QString command = arguments[++i].trimmed();
				const QString normalizedCommand = command.toUpper();
				static const QRegularExpression readOnlyFanucCommand(
					QStringLiteral("^(?:GET_[A-Z0-9_]+|CHECK_DONE)(?::[A-Z0-9_+.,-]+)?$"));
				if (!readOnlyFanucCommand.match(normalizedCommand).hasMatch())
				{
					LogCommandLineMessage(QString(
						"CLI FANUC RAW 已拒绝可能改变状态或触发运动的命令：%1。"
						"RAW 仅允许 GET_* / CHECK_DONE 只读诊断；调用程序请使用 --fanuc-call（会等待真实完成）。")
						.arg(command));
					continue;
				}
				const std::string response = pFanucDriver->SendRawCommandForTest(command.toStdString());
				LogCommandLineMessage(QString("CLI FANUC RAW %1 -> %2")
					.arg(command, QString::fromStdString(response)));
			}
			else if (arguments[i] == "--fanuc-call" && i + 1 < arguments.size())
			{
				const QString program = arguments[++i];
				const bool ok = pFanucDriver->CallJob(program.toStdString());
				const int done = ok ? pFanucDriver->CheckRobotDone(200, 1800000) : -1;
				const bool flowOk = ok && done > 0;
				LogCommandLineMessage(QString("CLI FANUC CALL %1 -> %2，CheckRobotDone=%3，详情=%4")
					.arg(program, flowOk ? "OK" : "FAIL")
					.arg(done)
					.arg(DecodeRobotMessageText(pFanucDriver->GetLastRobotError())));
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
	QTextStream out(stdout);
	ConfigureUtf8TextStream(out);
	out << line << Qt::endl;

	// CLI 命令行日志独立成 CliLog.txt（历史上被误写进机器人A单元日志 RobotALog.txt），统一走 RobotLog 按天归档。
	static RobotLog cliLogger("Log/CliLog.txt", false);
	cliLogger.writeLine(message.toStdString());
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
	SetConsoleOutputCP(CP_UTF8);
	SetConsoleCP(CP_UTF8);
	std::ios::sync_with_stdio(true);
#endif
}

void QtWidgetsApplication4::WaitForCommandLineEnter(const QString& message) const
{
	EnsureCommandLineConsole();
	LogCommandLineMessage(message);
	QTextStream out(stdout);
	ConfigureUtf8TextStream(out);
	out << "\n" << message << "\n按回车继续..." << Qt::flush;
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

	for (const T_CONTRAL_UNIT& unitInfo : m_pContralUnit->m_vtContralUnitInfo)
	{
		RobotDriverAdaptor* pRobotDriverAdaptor = static_cast<RobotDriverAdaptor*>(unitInfo.pUnitDriver);
		if (FANUCRobotCtrl* pFanucDriver = dynamic_cast<FANUCRobotCtrl*>(pRobotDriverAdaptor))
		{
			return pFanucDriver;
		}
	}
	return nullptr;
}

RobotDriverAdaptor* QtWidgetsApplication4::GetRobotDriverForCli(
	const QStringList& arguments,
	QString* robotLabelOut,
	int* unitIndexOut) const
{
	if (robotLabelOut != nullptr)
	{
		robotLabelOut->clear();
	}
	if (unitIndexOut != nullptr)
	{
		*unitIndexOut = -1;
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
	if (unitIndexOut != nullptr)
	{
		*unitIndexOut = selectedUnit->nUnitNo;
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
	if (speedSpecified && (!std::isfinite(speedOverride) || speedOverride <= 0.0))
	{
		LogCommandLineMessage("CLI 机器人运动参数错误：--robot-speed 必须是有限正数。");
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
	const bool isFanucDriver = dynamic_cast<FANUCRobotCtrl*>(driver) != nullptr;
	const auto linearCommandSpeed = [isFanucDriver](double speedMmPerMin)
		{
			if (!isFanucDriver)
			{
				return speedMmPerMin;
			}
			const double mmPerSec = speedMmPerMin / 60.0;
			return mmPerSec >= 1.0 ? std::floor(mmPerSec) : 0.0;
		};
	const QString linearCommandSpeedUnit = isFanucDriver
		? QStringLiteral("mm/sec")
		: QStringLiteral("mm/min");

	bool connected = driver->IsConnected();
	if (!connected)
	{
		connected = driver->InitSocket(driver->m_sSocketIP.c_str(), static_cast<unsigned short>(driver->m_nSocketPort));
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

	const bool noWaitRequested = CliOptionContains(arguments, "--robot-no-wait");
	if (noWaitRequested)
	{
		LogCommandLineMessage("CLI --robot-no-wait 已因全局硬件互锁禁用：本次仍会等待真实运动完成。");
	}
	const bool noWait = false;
	QString leaseError;
	const auto operationLease = RobotOperationLease::TryAcquire(
		driver, QStringLiteral("CLI 机器人运动"), &leaseError);
	if (!operationLease)
	{
		LogCommandLineMessage("CLI 机器人运动已被互锁拦截：" + leaseError);
		return;
	}
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

			const double configuredSpeedMmPerMin = speedSpecified ? speedOverride : 500.0;
			const double moveSpeed = linearCommandSpeed(configuredSpeedMmPerMin);
			if (moveSpeed <= 0.0)
			{
				LogCommandLineMessage(
					"CLI MOVL已拒绝：FANUC固定TP最低只能表示1mm/sec（60mm/min），禁止静默提速。");
				continue;
			}
			T_ROBOT_COORS current;
			if (!driver->TryGetCurrentPos(current))
			{
				LogCommandLineMessage("CLI MOVL已拒绝：读取当前机器人位置失败，"
					+ DecodeRobotMessageText(driver->GetLastRobotError()));
				continue;
			}
			int motionTimeoutMs = 0;
			std::string admissionError;
			if (!RobotMotionTimeoutPolicy::AdmitCartesianMove(
				current,
				target,
				isFanucDriver ? moveSpeed * 60.0 : configuredSpeedMmPerMin,
				motionTimeoutMs,
				&admissionError))
			{
				LogCommandLineMessage("CLI MOVL已拒绝：" + QString::fromUtf8(admissionError.c_str()));
				continue;
			}
			LogCommandLineMessage(QString("CLI MOVL下发：机器人=%1，配置速度=%2mm/min，下发速度=%3%4，目标=%5")
				.arg(robotLabel)
				.arg(configuredSpeedMmPerMin, 0, 'f', 3)
				.arg(moveSpeed, 0, 'f', 3)
				.arg(linearCommandSpeedUnit)
				.arg(FormatCliCoors(target)));
			const bool moveOk = driver->MoveByJob(target, T_ROBOT_MOVE_SPEED(moveSpeed, 0.0, 0.0), driver->m_nExternalAxleType, "MOVL");
			const int done = (moveOk && !noWait) ? driver->CheckRobotDone(doneDelayMs, motionTimeoutMs) : -1;
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

			T_ROBOT_COORS current;
			if (!driver->TryGetCurrentPos(current))
			{
				LogCommandLineMessage("CLI 相对MOVL已拒绝：读取当前机器人位置失败，"
					+ DecodeRobotMessageText(driver->GetLastRobotError()));
				continue;
			}
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

			const double configuredSpeedMmPerMin = speedSpecified ? speedOverride : 500.0;
			const double moveSpeed = linearCommandSpeed(configuredSpeedMmPerMin);
			if (moveSpeed <= 0.0)
			{
				LogCommandLineMessage(
					"CLI 相对MOVL已拒绝：FANUC固定TP最低只能表示1mm/sec（60mm/min），禁止静默提速。");
				continue;
			}
			int motionTimeoutMs = 0;
			std::string admissionError;
			if (!RobotMotionTimeoutPolicy::AdmitCartesianMove(
				current,
				target,
				isFanucDriver ? moveSpeed * 60.0 : configuredSpeedMmPerMin,
				motionTimeoutMs,
				&admissionError))
			{
				LogCommandLineMessage("CLI 相对MOVL已拒绝：" + QString::fromUtf8(admissionError.c_str()));
				continue;
			}
			LogCommandLineMessage(QString("CLI 相对MOVL下发：机器人=%1，配置速度=%2mm/min，下发速度=%3%4，当前位置=%5，目标=%6")
				.arg(robotLabel)
				.arg(configuredSpeedMmPerMin, 0, 'f', 3)
				.arg(moveSpeed, 0, 'f', 3)
				.arg(linearCommandSpeedUnit)
				.arg(FormatCliCoors(current))
				.arg(FormatCliCoors(target)));
			const bool moveOk = driver->MoveByJob(target, T_ROBOT_MOVE_SPEED(moveSpeed, 0.0, 0.0), driver->m_nExternalAxleType, "MOVL");
			const int done = (moveOk && !noWait) ? driver->CheckRobotDone(doneDelayMs, motionTimeoutMs) : -1;
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
			const int done = (moveOk && !noWait) ? driver->CheckRobotDone(doneDelayMs, 1800000) : -1;
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
		const QString localPath = FindInstalledResourcePath(item.localRelativePath);
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
		inputInfo = QFileInfo(AppPaths::CommandLinePath(normalizedInputPath));
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
	RobotCalculation::LowerWeldFilterParams originalParams = BuildCliOriginalTrackFitParams(sampleAxis);
	if (originalParams.exportFitDebugCloud)
	{
		// CLI 把每段拟合点集与拟合直线导出到输入文件同目录的 FitDebug 子目录，便于拖入 CloudCompare 核对。
		originalParams.fitDebugDir = inputInfo.absolutePath();
	}
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
		: QFileInfo(AppPaths::CommandLinePath(normalizedOutputPath)).absoluteFilePath();
	const QString noiseOutputPath = BuildNoiseOutputPath(classifiedOutputPath);
	const QString keyPointsOutputPath = BuildKeyPointsOutputPath(classifiedOutputPath);
	const QString cornerCompClassifiedOutputPath = BuildCornerCompClassifiedOutputPath(classifiedOutputPath);
	const QString cornerCompKeyPointsOutputPath = BuildKeyPointsOutputPath(cornerCompClassifiedOutputPath);

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
	const bool hasCornerCompensatedResult =
		analysisResult.cornerCompensatedClassificationResult.ok
		&& !analysisResult.cornerCompensatedClassificationResult.points.isEmpty();
	if (hasCornerCompensatedResult)
	{
		QStringList cornerCompLines;
		cornerCompLines << "# index x y z type_code type_name source";
		cornerCompLines << "# 1=start 2=end 3=inner_corner 4=outer_corner 5=normal 6=noise";
		for (const RobotCalculation::LowerWeldClassifiedPoint& point
			: analysisResult.cornerCompensatedClassificationResult.points)
		{
			cornerCompLines << QString("%1 %2 %3 %4 %5 %6 %7")
				.arg(point.index)
				.arg(point.point.x(), 0, 'f', 6)
				.arg(point.point.y(), 0, 'f', 6)
				.arg(point.point.z(), 0, 'f', 6)
				.arg(RobotCalculation::LowerWeldPointTypeCode(point.type))
				.arg(RobotCalculation::LowerWeldPointTypeName(point.type))
				.arg(point.source.isEmpty() ? "-" : point.source);
		}

		QStringList cornerCompKeyPointLines;
		cornerCompKeyPointLines << "# source_index x y z type_code type_name source";
		cornerCompKeyPointLines << "# 1=start 2=end 3=inner_corner 4=outer_corner";
		for (const RobotCalculation::LowerWeldClassifiedPoint& point : analysisResult.cornerCompensatedKeyPoints)
		{
			if (point.type == RobotCalculation::LowerWeldPointType::Normal
				|| point.type == RobotCalculation::LowerWeldPointType::Noise)
			{
				continue;
			}

			cornerCompKeyPointLines << QString("%1 %2 %3 %4 %5 %6 %7")
				.arg(point.index)
				.arg(point.point.x(), 0, 'f', 6)
				.arg(point.point.y(), 0, 'f', 6)
				.arg(point.point.z(), 0, 'f', 6)
				.arg(RobotCalculation::LowerWeldPointTypeCode(point.type))
				.arg(RobotCalculation::LowerWeldPointTypeName(point.type))
				.arg(point.source.isEmpty() ? "-" : point.source);
		}

		if (!RobotDataHelper::SaveTextFileLines(cornerCompClassifiedOutputPath, cornerCompLines, &error))
		{
			LogCommandLineMessage("CLI 激光点云分类失败，保存拐点补偿分类文件失败：" + error);
			return false;
		}
		if (!RobotDataHelper::SaveTextFileLines(cornerCompKeyPointsOutputPath, cornerCompKeyPointLines, &error))
		{
			LogCommandLineMessage("CLI 激光点云分类失败，保存拐点补偿拐点文件失败：" + error);
			return false;
		}
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
	if (hasCornerCompensatedResult)
	{
		LogCommandLineMessage(QString("CLI 拐点补偿分类文件：%1")
			.arg(QDir::toNativeSeparators(cornerCompClassifiedOutputPath)));
		LogCommandLineMessage(QString("CLI 拐点补偿起终点/拐点文件：%1")
			.arg(QDir::toNativeSeparators(cornerCompKeyPointsOutputPath)));
	}
	return true;
}

bool QtWidgetsApplication4::RunLaserClassifyDirForCli(const QString& dirPath) const
{
	QString normalizedDirPath = QDir::fromNativeSeparators(dirPath.trimmed());
	if (normalizedDirPath.isEmpty())
	{
		LogCommandLineMessage("CLI 批量激光点云分类失败：目录为空。");
		return false;
	}

	QDir rootDir(normalizedDirPath);
	if (!rootDir.isAbsolute())
	{
		rootDir = QDir(AppPaths::CommandLinePath(normalizedDirPath));
	}
	if (!rootDir.exists())
	{
		LogCommandLineMessage(QString("CLI 批量激光点云分类失败：未找到目录 %1")
			.arg(QDir::toNativeSeparators(rootDir.absolutePath())));
		return false;
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
	if (totalCount <= 0)
	{
		LogCommandLineMessage("CLI 批量激光点云分类失败：目录内没有 PreciseLaserPoint.txt。");
		return false;
	}
	return failedFiles.isEmpty();
}

bool QtWidgetsApplication4::RunRebuildMeasureWeldFilesForCli(
	const QStringList& arguments,
	const QString& laserDirPath) const
{
	QString normalizedDirPath = QDir::fromNativeSeparators(laserDirPath.trimmed());
	if (normalizedDirPath.isEmpty())
	{
		LogCommandLineMessage("CLI 先测后焊重建失败：LaserPoint目录为空。");
		return false;
	}

	QDir laserDir(normalizedDirPath);
	if (!laserDir.isAbsolute())
	{
		laserDir = QDir(AppPaths::CommandLinePath(normalizedDirPath));
	}
	if (!laserDir.exists())
	{
		LogCommandLineMessage(QString("CLI 先测后焊重建失败：未找到目录 %1")
			.arg(QDir::toNativeSeparators(laserDir.absolutePath())));
		return false;
	}

	QString robotLabel;
	RobotDriverAdaptor* pRobotDriver = GetRobotDriverForCli(arguments, &robotLabel, nullptr);
	if (pRobotDriver == nullptr)
	{
		LogCommandLineMessage("CLI 先测后焊重建失败：未找到可用机器人，请通过 --robot 指定。");
		return false;
	}

	MeasureThenWeldService service;
	T_PRECISE_MEASURE_PARAM param;
	QString error;
	if (!service.LoadPresetParam(pRobotDriver, param, error))
	{
		LogCommandLineMessage("CLI 先测后焊重建失败：读取预设参数失败：" + error);
		return false;
	}

	auto appendLog = [this](const QString& text)
		{
			LogCommandLineMessage("CLI 先测后焊重建：" + text);
		};
	auto setFlowStep = [this](const QString& text)
		{
			LogCommandLineMessage("CLI 流程节点：" + text);
		};

	QString preservePath;
	QString weldPosePath;
	QString seamCompPath;
	QString summary;
	if (!service.RebuildWeldFilesFromLaserDir(
		param,
		laserDir.absolutePath(),
		preservePath,
		weldPosePath,
		seamCompPath,
		summary,
		error,
		appendLog,
		setFlowStep))
	{
		LogCommandLineMessage("CLI 先测后焊重建失败：" + error);
		return false;
	}

	LogCommandLineMessage(QString("CLI 先测后焊重建完成：机器人=%1，目录=%2")
		.arg(robotLabel)
		.arg(QDir::toNativeSeparators(laserDir.absolutePath())));
	LogCommandLineMessage("CLI 先测后焊重建摘要：" + summary);
	return true;
}

bool QtWidgetsApplication4::RunWeldSeamCompForCli(
	const QStringList& arguments,
	const QString& inputPath,
	const QString& outputPath) const
{
	QString normalizedInputPath = QDir::fromNativeSeparators(inputPath.trimmed());
	if (normalizedInputPath.isEmpty())
	{
		LogCommandLineMessage("CLI 焊道补偿失败：输入文件为空。");
		return false;
	}

	QFileInfo inputInfo(normalizedInputPath);
	if (!inputInfo.isAbsolute())
	{
		inputInfo = QFileInfo(AppPaths::CommandLinePath(normalizedInputPath));
	}
	if (!inputInfo.exists())
	{
		LogCommandLineMessage(QString("CLI 焊道补偿失败：未找到输入文件 %1")
			.arg(QDir::toNativeSeparators(inputInfo.absoluteFilePath())));
		return false;
	}

	const QString normalizedOutputPath = outputPath.trimmed().isEmpty()
		? BuildWeldSeamCompOutputPath(inputInfo.absoluteFilePath())
		: QDir::fromNativeSeparators(outputPath.trimmed());
	const QString resolvedOutputPath = QFileInfo(normalizedOutputPath).isAbsolute()
		? QFileInfo(normalizedOutputPath).absoluteFilePath()
		: QFileInfo(AppPaths::CommandLinePath(normalizedOutputPath)).absoluteFilePath();

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
		return false;
	}

	QString robotName = InferRobotNameFromResultPath(inputInfo.absoluteFilePath(), QString());
	if (robotName.isEmpty())
	{
		QString robotLabel;
		if (RobotDriverAdaptor* driver = GetRobotDriverForCli(arguments, &robotLabel))
		{
			robotName = DecodeConfigText(driver->m_sRobotName).trimmed();
			LogCommandLineMessage(QString("CLI 焊道补偿：输入路径未包含 Result/机器人目录，使用 --robot 解析目标：%1")
				.arg(robotLabel));
		}
	}
	if (robotName.isEmpty())
	{
		robotName = QStringLiteral("RobotA");
	}

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
		return false;
	}

	LogCommandLineMessage(QString("CLI 焊道补偿完成：输入=%1，输出=%2，机器人=%3")
		.arg(QDir::toNativeSeparators(inputInfo.absoluteFilePath()))
		.arg(QDir::toNativeSeparators(resolvedOutputPath))
		.arg(robotName));
	LogCommandLineMessage("CLI 焊道补偿摘要：" + summary);
	return true;
}

bool QtWidgetsApplication4::RunGenerateStepWeldProgramForCli(
	const QStringList& arguments,
	const QString& inputPath,
	const QString& outputDir,
	bool actualWeld,
	double weldSpeedMmPerMin) const
{
	QString normalizedInputPath = QDir::fromNativeSeparators(inputPath.trimmed());
	if (normalizedInputPath.isEmpty())
	{
		LogCommandLineMessage("CLI STEP焊接程序生成失败：输入文件为空。");
		return false;
	}

	QFileInfo inputInfo(normalizedInputPath);
	if (!inputInfo.isAbsolute())
	{
		inputInfo = QFileInfo(AppPaths::CommandLinePath(normalizedInputPath));
	}
	if (!inputInfo.exists())
	{
		LogCommandLineMessage(QString("CLI STEP焊接程序生成失败：未找到输入文件 %1")
			.arg(QDir::toNativeSeparators(inputInfo.absoluteFilePath())));
		return false;
	}

	QString robotName = InferRobotNameFromResultPath(inputInfo.absoluteFilePath(), QString());
	if (robotName.isEmpty())
	{
		QString robotLabel;
		if (RobotDriverAdaptor* driver = GetRobotDriverForCli(arguments, &robotLabel))
		{
			robotName = DecodeConfigText(driver->m_sRobotName).trimmed();
			LogCommandLineMessage(QString("CLI STEP焊接程序：输入路径未包含 Result/机器人目录，使用 --robot 解析目标：%1")
				.arg(robotLabel));
		}
	}
	if (robotName.isEmpty())
	{
		robotName = QStringLiteral("RobotA");
	}

	MeasureThenWeldService service;
	QString programName;
	QString srpPath;
	QString srdPath;
	QString summary;
	QString error;
	if (!service.GenerateStepWeldProgramFiles(
		robotName,
		inputInfo.absoluteFilePath(),
		outputDir,
		actualWeld,
		weldSpeedMmPerMin,
		programName,
		srpPath,
		srdPath,
		summary,
		error))
	{
		LogCommandLineMessage("CLI STEP焊接程序生成失败：" + error);
		return false;
	}

	LogCommandLineMessage(QString("CLI STEP焊接程序生成完成：输入=%1，机器人=%2，程序=%3")
		.arg(QDir::toNativeSeparators(inputInfo.absoluteFilePath()))
		.arg(robotName)
		.arg(programName));
	LogCommandLineMessage("CLI STEP焊接程序摘要：" + summary);
	LogCommandLineMessage("CLI STEP SRP：" + srpPath);
	LogCommandLineMessage("CLI STEP SRD：" + srdPath);
	return true;
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
	RobotDriverAdaptor* pRobotDriver,
	int unitIndex,
	int repeatCount,
	double scanSpeedOverrideMmPerMin,
	double cameraTimeOffsetOverrideMs)
{
	if (pRobotDriver == nullptr)
	{
		LogCommandLineMessage("CLI 先测后焊扫描失败：机器人驱动为空。");
		return false;
	}
	if (repeatCount <= 0)
	{
		LogCommandLineMessage("CLI 先测后焊扫描失败：重复次数必须大于0。");
		return false;
	}
	QString leaseError;
	const auto operationLease = RobotOperationLease::TryAcquire(
		pRobotDriver, QStringLiteral("CLI 先测后焊扫描"), &leaseError);
	if (!operationLease)
	{
		LogCommandLineMessage("CLI 先测后焊扫描已被互锁拦截：" + leaseError);
		return false;
	}

	if (unitIndex < 0 && m_pContralUnit != nullptr)
	{
		for (const T_CONTRAL_UNIT& unitInfo : m_pContralUnit->m_vtContralUnitInfo)
		{
			if (unitInfo.pUnitDriver == pRobotDriver)
			{
				unitIndex = unitInfo.nUnitNo;
				break;
			}
		}
	}
	if (unitIndex < 0)
	{
		unitIndex = CurrentRobotUnitIndex();
	}

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
		if (!service.LoadPresetParam(pRobotDriver, param, error))
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
		LogCommandLineMessage(QString("CLI 第%1/%2次扫描开始：参数=%3 [%4]，ScanSpeed=%5 mm/min%6，CameraTimeOffsetMs=%7%8")
			.arg(repeatIndex)
			.arg(repeatCount)
			.arg(QString::fromStdString(param.sIniFilePath))
			.arg(QString::fromStdString(param.sSectionName))
			.arg(param.dScanSpeed, 0, 'f', 3)
			.arg(std::isfinite(scanSpeedOverrideMmPerMin) && scanSpeedOverrideMmPerMin > 0.0 ? "（CLI覆盖）" : "")
			.arg(param.dCameraTimeOffsetMs, 0, 'f', 3)
			.arg(std::isfinite(cameraTimeOffsetOverrideMs) ? "（CLI覆盖）" : ""));

		const double runSpeed = std::isfinite(param.dRunSpeed) && param.dRunSpeed > 0.0
			? param.dRunSpeed
			: 1.0;
		auto unattendedSafetyCheckpoint = [&](const QString& title, const QString& detail) -> bool
			{
				appendLog(QString("第%1次扫描检测到需要人工确认的安全风险，CLI 无人值守模式不自动放行：%2\n%3")
					.arg(repeatIndex)
					.arg(title, detail));
				return false;
			};
		MeasureThenWeldService::ScanCycleResult scanCycle;
		const bool ok = service.RunScanCycle(
			pRobotDriver,
			param,
			runSpeed,
			cameraCache,
			scanCycle,
			appendLog,
			setFlowStep,
			unattendedSafetyCheckpoint);
		savedPath = scanCycle.weldPosePath;

		if (!ok)
		{
			LogCommandLineMessage(QString("CLI 第%1次扫描流程失败，已停止后续重复：%2")
				.arg(repeatIndex)
				.arg(scanCycle.error));
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

		// 只扫描数据在线上传：优先使用 Runner 明确返回的案例目录；仅为兼容旧结果再回退最新目录。
		if (OnlineServicesConfig::AutoUploadEnabled())
		{
			QString caseDir = scanCycle.caseDir;
			if (caseDir.isEmpty())
			{
				const QDir robotResultDir(AppPaths::WritablePath(
					QStringLiteral("Result/") + QString::fromStdString(param.sRobotName)));
				const QFileInfoList cases = robotResultDir.entryInfoList(QDir::Dirs | QDir::NoDotAndDotDot, QDir::Time);
				if (!cases.isEmpty())
				{
					caseDir = cases.first().absoluteFilePath();
				}
			}
			if (!caseDir.isEmpty())
			{
				EnsureScanDataUploader()->QueueUpload(caseDir);
				LogCommandLineMessage(QString("扫描数据已加入上传队列：%1").arg(QDir::toNativeSeparators(caseDir)));
			}
		}
	}

	QThread::msleep(300);
	LogCommandLineMessage("CLI 先测后焊扫描：扫描结束。");
	return allOk;
}

// ============ 流程测试页后端（自动循环先测后焊·仅扫描） ============
// 循环骨架与 RunMeasureThenWeldScanOnlyRepeatForCli 一致（LoadPresetParam→移动→ScanMoveAndCollect），
// 但改为可停止 + 进度回调、跑在流程测试对话框的后台线程。相机启动编列到主线程（非线程安全）。

ProcessLoopTestDefaults QtWidgetsApplication4::LoadProcessLoopTestDefaults(int unitIndex)
{
	ProcessLoopTestDefaults d;
	if (unitIndex < 0)
	{
		unitIndex = CurrentRobotUnitIndex();
	}
	RobotDriverAdaptor* driver = nullptr;
	if (m_pContralUnit != nullptr)
	{
		for (const T_CONTRAL_UNIT& unitInfo : m_pContralUnit->m_vtContralUnitInfo)
		{
			if (unitInfo.nUnitNo == unitIndex)
			{
				driver = static_cast<RobotDriverAdaptor*>(unitInfo.pUnitDriver);
				break;
			}
		}
	}
	if (driver == nullptr)
	{
		d.issue = QStringLiteral("目标单元无机器人驱动");
		return d;
	}

	MeasureThenWeldService service;
	T_PRECISE_MEASURE_PARAM param;
	QString error;
	if (!service.LoadPresetParam(driver, param, error))
	{
		d.issue = error.isEmpty() ? QStringLiteral("读取预设参数失败") : error;
		return d;
	}
	d.ok = true;
	d.scanSpeedMmPerMin = param.dScanSpeed;
	d.runSpeedMmPerMin = param.dRunSpeed;
	d.cameraTimeOffsetMs = param.dCameraTimeOffsetMs;
	d.paramGroupName = param.sParamGroupName;
	return d;
}

void QtWidgetsApplication4::RunProcessLoopTest(
	const ProcessLoopTestSettings& settings,
	std::atomic<bool>* stopFlag,
	const ProcessLoopTestDialog::ProgressCallback& progress,
	const ProcessLoopTestDialog::LogCallback& log)
{
	const int total = settings.infinite ? 0 : settings.repeatCount;
	int okCount = 0;
	int failCount = 0;
	// ProcessLoop 持有一个跨多轮的长租约。账号撤销后全局 gate 会关闭；
	// 将它并入停止信号，允许当前动作执行安全收枪，但禁止焊接段和下一轮。
	auto stopped = [stopFlag]()
		{
			return (stopFlag != nullptr && stopFlag->load())
				|| !RobotOperationLease::NewOperationsAllowed();
		};
	auto emitLog = [&log](const QString& t) { if (log) { log(t); } };
	auto emitProgress = [&](int iter, const QString& step, bool finished)
		{ if (progress) { progress(iter, total, okCount, failCount, step, finished); } };

	int unitIndex = settings.unitIndex >= 0 ? settings.unitIndex : CurrentRobotUnitIndex();
	RobotDriverAdaptor* driver = nullptr;
	if (m_pContralUnit != nullptr)
	{
		for (const T_CONTRAL_UNIT& unitInfo : m_pContralUnit->m_vtContralUnitInfo)
		{
			if (unitInfo.nUnitNo == unitIndex)
			{
				driver = static_cast<RobotDriverAdaptor*>(unitInfo.pUnitDriver);
				break;
			}
		}
	}
	if (driver == nullptr)
	{
		emitLog(QStringLiteral("目标单元 %1 无机器人驱动，无法测试。").arg(unitIndex));
		emitProgress(0, QStringLiteral("无驱动"), true);
		return;
	}

	// 相机启动/取缓存在主线程完成（相机管理非线程安全）；随后循环在本后台线程执行（同先测后焊 GUI 流程）。
	QString cameraIP;
	bool cameraOk = false;
	CameraFrameCache* cameraCache = nullptr;
	QMetaObject::invokeMethod(this, [&]()
		{
			cameraOk = EnsureScanCameraRunningForUnit(unitIndex, cameraIP, true);
			if (cameraOk) { cameraCache = ScanCameraCacheForUnit(unitIndex); }
		}, Qt::BlockingQueuedConnection);
	if (!cameraOk)
	{
		emitLog(QStringLiteral("未能启动测量相机，无法测试。"));
		emitProgress(0, QStringLiteral("相机未就绪"), true);
		return;
	}

	emitLog(QStringLiteral("相机已启动 %1。%2")
		.arg(cameraIP,
			settings.infinite ? QStringLiteral("持续循环，点「停止」结束。")
			: QStringLiteral("计划循环 %1 次。").arg(settings.repeatCount)));

	MeasureThenWeldService service;
	for (int i = 1; settings.infinite || i <= settings.repeatCount; ++i)
	{
		if (stopped())
		{
			emitLog(RobotOperationLease::NewOperationsAllowed()
				? QStringLiteral("已手动停止。")
				: QStringLiteral("账号会话已失效：当前动作安全结束后，不再开始下一轮机器人操作。"));
			break;
		}

		T_PRECISE_MEASURE_PARAM param;
		QString error;
		if (!service.LoadPresetParam(driver, param, error))
		{
			++failCount;
			emitLog(QStringLiteral("第%1次：读取预设参数失败：%2").arg(i).arg(error));
			emitProgress(i, QStringLiteral("参数读取失败"), false);
			break;
		}
		if (settings.overrideScanSpeed && settings.scanSpeedMmPerMin > 0.0) { param.dScanSpeed = settings.scanSpeedMmPerMin; }
		if (settings.overrideRunSpeed && settings.runSpeedMmPerMin > 0.0) { param.dRunSpeed = settings.runSpeedMmPerMin; }
		if (settings.overrideCameraOffset) { param.dCameraTimeOffsetMs = settings.cameraTimeOffsetMs; }
		if (settings.doWeld)
		{
			QString invalidateError;
			if (!MeasureThenWeldService::InvalidateStoredWeldResumeCheckpoint(
				QString::fromStdString(param.sRobotName), invalidateError))
			{
				++failCount;
				emitLog(QStringLiteral("第%1次：使旧焊接断点失效失败，未开始机器人动作：%2")
					.arg(i).arg(invalidateError));
				emitProgress(i, QStringLiteral("断点失效失败"), false);
				break;
			}
		}

		// 记录本次开始前 Result/<机器人>/ 的最新案例目录：本次结束后若出现新目录，即产生了扫描数据。
		// 点云分析失败的次数也要传原始扫描数据（非波纹板工件分析必失败，但原始激光点数据有效）。
		auto newestCaseDir = [](const QString& robot) -> QString
			{
				const QDir dir(AppPaths::WritablePath(QStringLiteral("Result/") + robot));
				const QFileInfoList cases = dir.entryInfoList(QDir::Dirs | QDir::NoDotAndDotDot, QDir::Time);
				return cases.isEmpty() ? QString() : cases.first().absoluteFilePath();
			};
		const QString robotName = QString::fromStdString(param.sRobotName);
		const QString caseDirBefore = newestCaseDir(robotName);

		const double runSpeed = (std::isfinite(param.dRunSpeed) && param.dRunSpeed > 0.0) ? param.dRunSpeed : 1.0;
		auto stepCb = [&](const QString& t) { emitProgress(i, t, false); };

		emitLog(QStringLiteral("第%1%2次开始：扫描速度 %3 mm/min，相机偏移 %4 ms。")
			.arg(i)
			.arg(total > 0 ? QStringLiteral("/%1").arg(total) : QString())
			.arg(param.dScanSpeed, 0, 'f', 1)
			.arg(param.dCameraTimeOffsetMs, 0, 'f', 1));

		auto unattendedSafetyCheckpoint = [&](const QString& title, const QString& detail) -> bool
			{
				emitLog(QStringLiteral("第%1次检测到需要人工确认的安全风险，自动循环不自动放行：%2\n%3")
					.arg(i)
					.arg(title, detail));
				return false;
			};
		MeasureThenWeldService::ScanCycleResult scanCycle;
		service.RunScanCycle(
			driver,
			param,
			runSpeed,
			cameraCache,
			scanCycle,
			emitLog,
			stepCb,
			unattendedSafetyCheckpoint,
			MeasureThenWeldService::BeforeActionCallback(),
			stopped);
		const bool motionStepFailed = scanCycle.motionFailure;
		const bool fatalCycleFailure = scanCycle.fatalFailure;
		bool stopAfterCycle = scanCycle.stopRequestedDuringCycle || stopped();
		bool ok = scanCycle.scanDataSucceeded && scanCycle.safelyRetracted;
		QString savedPath = scanCycle.weldPosePath;

		// 扫描前停止：不再发送新的运动，也没有本轮数据需要上传或统计。
		if (scanCycle.status == MeasureThenWeldService::ScanCycleStatus::Stopped
			&& !scanCycle.scanAttempted
			&& stopped())
		{
			emitLog(QStringLiteral("第%1次已按停止请求结束，不再进入焊接或下一轮。").arg(i));
			emitProgress(i, QStringLiteral("已停止"), false);
			break;
		}

		// 焊接段（勾选「包含焊接段」时）：执行扫描产出的焊接姿态文件。实际焊接/空跑按预设 param.bDoActualWeld。
		bool weldAttempted = false;
		if (ok && settings.doWeld && !stopAfterCycle && !stopped())
		{
			if (savedPath.isEmpty())
			{
				emitLog(QStringLiteral("第%1次：要求包含焊接段，但扫描未生成焊接姿态文件，本轮判定失败。").arg(i));
				ok = false;
			}
			else
			{
				stopAfterCycle = stopped();
				if (stopAfterCycle)
				{
					emitLog(QStringLiteral("焊接下枪前收到停止请求，已取消本轮焊接。"));
				}
				else
				{
				weldAttempted = true;
				QString weldSummary;
				QString weldError;
				// 自动化循环：确认点自动放行；用户按「停止」则在下个确认点中止焊接。
				auto weldCheckpoint = [&](const QString& t, const QString&) -> bool
					{ if (!t.isEmpty()) { emitProgress(i, t, false); } return !stopped(); };
				ok = service.ExecuteWeldPoseFileWithSafePos(
					driver, savedPath, param, weldSummary, weldError,
					nullptr, nullptr, emitLog, stepCb, weldCheckpoint,
					param.dFinalWeldTrajectoryStepMm, /*allowPointwiseWeave=*/true,
					MeasureThenWeldService::WeldPoseSource::PointCloudProduction,
					/*resumeStartArcMm=*/-1.0, /*inputAlreadyInExecutionOrder=*/false, stopped);
				emitLog(ok
					? QStringLiteral("第%1次焊接完成：%2").arg(i).arg(weldSummary)
					: QStringLiteral("第%1次焊接失败/中止：%2").arg(i).arg(weldError));
				}
			}
		}

		// 扫描数据在线上传：受「扫描完成后自动上传」总开关控制，成功/失败都传本次新产生的数据
		//（案例目录优先从 savedPath 反推；仅扫描/分析失败时按「循环前后最新案例目录变化」判定，
		// 避免把上一轮的旧案例重复入队）。ConfigDatabase 与上传器须在 UI 线程访问，编列过去。
		{
			QString caseDir = scanCycle.caseDir;
			if (caseDir.isEmpty())
			{
				const QString after = newestCaseDir(robotName);
				if (!after.isEmpty() && after != caseDirBefore)
				{
					caseDir = after;
				}
			}
			if (!caseDir.isEmpty())
			{
				QMetaObject::invokeMethod(this, [this, caseDir, log]()
					{
						if (!OnlineServicesConfig::AutoUploadEnabled())
						{
							return;
						}
						EnsureScanDataUploader()->QueueUpload(caseDir);
						if (log)
						{
							log(QStringLiteral("扫描数据已加入上传队列：%1").arg(QDir::toNativeSeparators(caseDir)));
						}
					}, Qt::QueuedConnection);
			}
		}

		if (ok)
		{
			++okCount;
			emitLog(QStringLiteral("第%1次完成%2。").arg(i)
				.arg(weldAttempted
					? QStringLiteral("（含焊接）")
					: (stopAfterCycle && settings.doWeld
						? QStringLiteral("（仅扫描，已按停止请求跳过焊接）")
						: QStringLiteral("，已回收枪安全位"))));
			emitProgress(i, QStringLiteral("本次完成"), false);
		}
		else
		{
			++failCount;
			const bool byStop = stopped();
			emitLog(QStringLiteral("第%1次%2。").arg(i).arg(byStop ? QStringLiteral("被停止") : QStringLiteral("失败")));
			emitProgress(i, byStop ? QStringLiteral("已停止") : QStringLiteral("本次失败"), false);
			// 任一运动阶段失败后机器人位姿不再可信，禁止因「失败后继续」设置进入下一轮。
			if (byStop || motionStepFailed || fatalCycleFailure || settings.stopOnFailure) { break; }
		}

		if (stopAfterCycle)
		{
			emitLog(QStringLiteral("停止请求已生效：本轮数据已处理，不再进入下一轮。"));
			break;
		}

		// 定时扫描：本次完成后、若还有下一次，等待设定间隔（可被「停止」逐秒打断，UI 显示倒计时）。
		if (settings.scanIntervalSeconds > 0
			&& (settings.infinite || i < settings.repeatCount)
			&& !stopped())
		{
			const int totalMs = settings.scanIntervalSeconds * 1000;
			int elapsedMs = 0;
			int lastShownSec = -1;
			while (elapsedMs < totalMs && !stopped())
			{
				const int remainingSec = (totalMs - elapsedMs + 999) / 1000;
				if (remainingSec != lastShownSec)
				{
					emitProgress(i, QStringLiteral("等待下次扫描（剩 %1 秒）").arg(remainingSec), false);
					lastShownSec = remainingSec;
				}
				QThread::msleep(200);
				elapsedMs += 200;
			}
		}
	}

	emitLog(QStringLiteral("测试结束：成功 %1，失败 %2。").arg(okCount).arg(failCount));
	emitProgress(okCount + failCount, QStringLiteral("成功 %1 / 失败 %2").arg(okCount).arg(failCount), true);
}

void QtWidgetsApplication4::OpenProcessLoopTestPage()
{
	PageOpenTrace trace("流程测试");
	if (!RequirePermission(kRoleEngineer, "流程测试"))
	{
		return;
	}
	if (m_pManagementStack == nullptr)
	{
		QMessageBox::information(this, QStringLiteral("流程测试"), QStringLiteral("流程测试请在管理界面中打开。"));
		return;
	}
	if (m_pProcessLoopTestPage != nullptr)
	{
		ShowManagementEmbeddedPage(m_pProcessLoopTestPage);
		return;
	}

	auto loadDefaults = [this](int unitIndex) { return LoadProcessLoopTestDefaults(unitIndex); };
	auto runner = [this](const ProcessLoopTestSettings& s, std::atomic<bool>* stop,
		const ProcessLoopTestDialog::ProgressCallback& progress, const ProcessLoopTestDialog::LogCallback& log)
		{
			RunProcessLoopTest(s, stop, progress, log);
		};

	ProcessLoopTestDialog* loopPage = new ProcessLoopTestDialog(m_pContralUnit, CurrentRobotUnitIndex(),
		loadDefaults, runner, m_pManagementStack);
	// 同机互锁由 ProcessLoopTestDialog::OnStart 的原子租约完成；不同机器人允许并行。
	m_pProcessLoopTestPage = loopPage;
	PrepareEmbeddedPage(m_pProcessLoopTestPage, m_pManagementStack);
	ShowManagementEmbeddedPage(m_pProcessLoopTestPage);
}

bool QtWidgetsApplication4::LoadGrooveCameraEndpointForUnit(int unitIndex, QString& cameraIP, int& cameraPort, int* pollIntervalMs) const
{
	constexpr int kDefaultTcpSensorPort = 50006;
	constexpr int kDefaultUdpSensorPort = 50004;
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
	if (pollIntervalMs != nullptr)
	{
		bool okFps = false;
		const int parsedFps = cameraParam.readFps.trimmed().toInt(&okFps);
		const int effectiveFps = (okFps && parsedFps > 0) ? parsedFps : 100;
		// 配置存的是帧率(fps)，worker 轮询用的是间隔(ms)，这里换算：间隔 = round(1000/帧率)。
		int intervalMs = (1000 + effectiveFps / 2) / effectiveFps;
		if (intervalMs < 1)
		{
			intervalMs = 1;
		}
		*pollIntervalMs = intervalMs;
	}
	if (m_bUseSharedScanCameraReceiver)
	{
		bool okPort = false;
		const int configuredPort = cameraParam.devicePort.trimmed().toInt(&okPort);
		cameraPort = okPort && configuredPort > 0 ? configuredPort : kDefaultUdpSensorPort;
	}
	else
	{
		// TCP 模式连接 PointCloundTcpServer，端口固定为新版相机服务端口。
		cameraPort = kDefaultTcpSensorPort;
	}
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

	if (m_bUseSharedScanCameraReceiver)
	{
		QHash<int, QHash<QString, CameraFrameCache*>> dispatchTargetsByPort;
		for (const T_CONTRAL_UNIT& unitInfo : m_pContralUnit->m_vtContralUnitInfo)
		{
			CameraRuntime* runtime = m_scanCameraRuntimes.value(unitInfo.nUnitNo, nullptr);
			if (runtime == nullptr)
			{
				runtime = new CameraRuntime();
				runtime->cache = new CameraFrameCache();
				m_liveScanCameraRuntimes.insert(runtime);
				m_scanCameraRuntimes.insert(unitInfo.nUnitNo, runtime);
			}

			QString cameraIP;
			int cameraPort = 0;
			if (!LoadGrooveCameraEndpointForUnit(unitInfo.nUnitNo, cameraIP, cameraPort))
			{
				runtime->cameraStatus = "未配置扫描相机地址。";
				runtime->running = false;
				continue;
			}

			runtime->cameraIP = cameraIP;
			runtime->cameraPort = cameraPort;
			runtime->receiveMode = "UDP共享接收";
			runtime->running = true;
			runtime->cache->SetConnectionState(
				CameraFrameCache::ConnectionState::Connected,
				QStringLiteral("UDP 共享接收已启动，等待目标相机首帧。"));
			m_scanCameraUnitByIP.insert(cameraIP, unitInfo.nUnitNo);
			dispatchTargetsByPort[cameraPort].insert(cameraIP, runtime->cache);
		}

		for (auto it = dispatchTargetsByPort.constBegin(); it != dispatchTargetsByPort.constEnd(); ++it)
		{
			const int port = it.key();
			if (m_scanCameraReceiversByPort.contains(port))
			{
				continue;
			}

			CameraRuntime* receiverRuntime = new CameraRuntime();
			receiverRuntime->udpWorker = new ClientUDPFormSensorWorker(nullptr);
			receiverRuntime->thread = new QThread(this);
			receiverRuntime->cameraPort = port;
			receiverRuntime->receiveMode = "UDP共享接收";
			receiverRuntime->running = true;
			m_liveScanCameraRuntimes.insert(receiverRuntime);
			receiverRuntime->udpWorker->setDispatchTargets(it.value());
			connect(receiverRuntime->udpWorker, &ClientUDPFormSensorWorker::diagnosticChanged, this,
				[this, receiverRuntime](
					qint64 datagramCount,
					qint64 filteredDatagramCount,
					qint64 decodedFrameCount,
					qint64 decodeFailedCount,
					qint64 appendedFrameCount,
					const QString& statusText)
				{
					if (receiverRuntime == nullptr || !m_liveScanCameraRuntimes.contains(receiverRuntime))
					{
						return;
					}
					receiverRuntime->datagramCount = datagramCount;
					receiverRuntime->filteredDatagramCount = filteredDatagramCount;
					receiverRuntime->decodedFrameCount = decodedFrameCount;
					receiverRuntime->decodeFailedCount = decodeFailedCount;
					receiverRuntime->appendedFrameCount = appendedFrameCount;
					receiverRuntime->cameraStatus = statusText;
				});
			connect(receiverRuntime->udpWorker, &ClientUDPFormSensorWorker::targetDiagnosticChanged, this,
				[this](
					const QString& targetIP,
					qint64 datagramCount,
					qint64 filteredDatagramCount,
					qint64 decodedFrameCount,
					qint64 decodeFailedCount,
					qint64 appendedFrameCount,
					const QString& statusText)
				{
					const int unitIndex = m_scanCameraUnitByIP.value(targetIP, -1);
					CameraRuntime* runtime = m_scanCameraRuntimes.value(unitIndex, nullptr);
					if (runtime == nullptr)
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
			connect(receiverRuntime->thread, &QThread::finished, receiverRuntime->udpWorker, &QObject::deleteLater);
			receiverRuntime->udpWorker->moveToThread(receiverRuntime->thread);
			receiverRuntime->thread->start();
			m_scanCameraReceiversByPort.insert(port, receiverRuntime);
			QMetaObject::invokeMethod(
				receiverRuntime->udpWorker,
				"startReceiveDemux",
				Qt::BlockingQueuedConnection,
				Q_ARG(int, port));
		}
		return;
	}

	for (const T_CONTRAL_UNIT& unitInfo : m_pContralUnit->m_vtContralUnitInfo)
	{
		if (m_scanCameraRuntimes.contains(unitInfo.nUnitNo))
		{
			continue;
		}

		CameraRuntime* runtime = new CameraRuntime();
		runtime->cache = new CameraFrameCache();
		runtime->cache->SetSkjFrameBufferCount(MeasureThenWeldRuntimeConfig::LoadSkjFrameBufferCount());
		runtime->tcpWorker = new ScanCameraSkjWorker(runtime->cache);
		runtime->thread = new QThread(this);
		runtime->receiveMode = "TCP独立连接";
		m_liveScanCameraRuntimes.insert(runtime);
		connect(runtime->tcpWorker, &ScanCameraSkjWorker::diagnosticChanged, this,
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
		connect(runtime->thread, &QThread::finished, runtime->tcpWorker, &QObject::deleteLater);
		runtime->tcpWorker->moveToThread(runtime->thread);
		runtime->thread->start();
		m_scanCameraRuntimes.insert(unitInfo.nUnitNo, runtime);
		QString cameraIP;
		EnsureScanCameraRunningForUnit(unitInfo.nUnitNo, cameraIP, false, false);  // 启动期异步连接，不阻塞主窗口
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
		if (runtime->tcpWorker != nullptr)
		{
			QObject::disconnect(runtime->tcpWorker, nullptr, this, nullptr);
		}
		if (runtime->udpWorker != nullptr)
		{
			QObject::disconnect(runtime->udpWorker, nullptr, this, nullptr);
		}
		if (runtime->tcpWorker != nullptr
			&& runtime->thread != nullptr
			&& runtime->thread->isRunning())
		{
			QMetaObject::invokeMethod(runtime->tcpWorker, "stopClient", Qt::BlockingQueuedConnection);
		}
		if (runtime->udpWorker != nullptr
			&& runtime->thread != nullptr
			&& runtime->thread->isRunning())
		{
			QMetaObject::invokeMethod(runtime->udpWorker, "stopReceive", Qt::BlockingQueuedConnection);
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

bool QtWidgetsApplication4::EnsureScanCameraRunningForUnit(int unitIndex, QString& cameraIP, bool clearCache, bool blockingConnect)
{
	int cameraPort = 0;
	int cameraPollIntervalMs = 10;
	if (!LoadGrooveCameraEndpointForUnit(unitIndex, cameraIP, cameraPort, &cameraPollIntervalMs))
	{
		return false;
	}

	CameraRuntime* runtime = m_scanCameraRuntimes.value(unitIndex, nullptr);

	if (m_bUseSharedScanCameraReceiver)
	{
		if (runtime == nullptr || !runtime->running || !m_scanCameraReceiversByPort.contains(cameraPort))
		{
			// 共享接收拓扑重建会删除所有单元的 CameraFrameCache。任一机器人流程持租约时
			// 都必须失败关闭，不能为了修当前单元而让另一单元出现悬垂缓存指针。
			if (RobotOperationLease::AnyActive())
			{
				m_sGrooveCameraStatusText = QString("相机接收拓扑缺失，但机器人硬件操作正在运行（%1），已禁止全局重建。")
					.arg(RobotOperationLease::ActiveSummary());
				return false;
			}
			StopScanCameraRuntimes();
			InitializeScanCameraRuntimes();
			runtime = m_scanCameraRuntimes.value(unitIndex, nullptr);
		}
		if (runtime == nullptr || runtime->cache == nullptr)
		{
			return false;
		}
		if (clearCache)
		{
			runtime->cache->Clear();
		}
		const std::uint64_t readyMark = runtime->cache->Mark();
		runtime->cameraIP = cameraIP;
		runtime->cameraPort = cameraPort;
		runtime->receiveMode = "UDP共享接收";
		runtime->running = true;
		if (blockingConnect)
		{
			QString readyError;
			const int readyTimeoutMs = std::max(5000, cameraPollIntervalMs * 20);
			if (!runtime->cache->WaitForReadyFrameAfter(readyMark, readyTimeoutMs, &readyError))
			{
				runtime->cameraStatus = readyError;
				runtime->running = false;
				return false;
			}
			runtime->cameraStatus = QStringLiteral("UDP 共享接收已取得扫描前新帧。");
		}
		return true;
	}

	if (runtime == nullptr)
	{
		runtime = new CameraRuntime();
		runtime->cache = new CameraFrameCache();
		runtime->cache->SetSkjFrameBufferCount(MeasureThenWeldRuntimeConfig::LoadSkjFrameBufferCount());
		runtime->tcpWorker = new ScanCameraSkjWorker(runtime->cache);
		runtime->thread = new QThread(this);
		runtime->receiveMode = "TCP独立连接";
		m_liveScanCameraRuntimes.insert(runtime);
		connect(runtime->tcpWorker, &ScanCameraSkjWorker::diagnosticChanged, this,
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
		connect(runtime->thread, &QThread::finished, runtime->tcpWorker, &QObject::deleteLater);
		runtime->tcpWorker->moveToThread(runtime->thread);
		runtime->thread->start();
		m_scanCameraRuntimes.insert(unitIndex, runtime);
	}
	if (runtime->tcpWorker == nullptr)
	{
		return false;
	}
	if (runtime->cache != nullptr && clearCache)
	{
		runtime->cache->Clear();
	}
	if (runtime->cache == nullptr)
	{
		return false;
	}
	const std::uint64_t readyMark = runtime->cache->Mark();
	const CameraFrameCache::ConnectionState connectionState = runtime->cache->GetConnectionState();

	const bool needRestart = !runtime->running
		|| runtime->cameraIP != cameraIP
		|| runtime->cameraPort != cameraPort
		|| runtime->cameraPollIntervalMs != cameraPollIntervalMs
		|| connectionState == CameraFrameCache::ConnectionState::Stopped
		|| connectionState == CameraFrameCache::ConnectionState::Failed;
	if (needRestart)
	{
		// 启动期 blockingConnect=false：异步发起，不让相机 3s 同步连接超时阻塞主窗口显示；
		// 扫描前等场景用 BlockingQueuedConnection 完成连接尝试，随后再等待一张新点云帧。
		runtime->cache->SetConnectionState(
			CameraFrameCache::ConnectionState::Connecting,
			QString("正在连接相机 %1:%2。").arg(cameraIP).arg(cameraPort));
		const bool invoked = QMetaObject::invokeMethod(
			runtime->tcpWorker,
			"startClient",
			blockingConnect ? Qt::BlockingQueuedConnection : Qt::QueuedConnection,
			Q_ARG(QString, cameraIP),
			Q_ARG(int, cameraPort),
			Q_ARG(int, cameraPollIntervalMs));
		runtime->cameraIP = cameraIP;
		runtime->cameraPort = cameraPort;
		runtime->cameraPollIntervalMs = cameraPollIntervalMs;
		runtime->receiveMode = "TCP独立连接";
		runtime->running = true;
		if (!invoked)
		{
			runtime->cache->SetConnectionState(
				CameraFrameCache::ConnectionState::Failed,
				QStringLiteral("无法调用相机接收线程。"));
			runtime->cameraStatus = QStringLiteral("无法调用相机接收线程。");
			runtime->running = false;
			return false;
		}
	}
	if (blockingConnect)
	{
		QString readyError;
		const int readyTimeoutMs = std::max(5000, cameraPollIntervalMs * 20);
		if (!runtime->cache->WaitForReadyFrameAfter(readyMark, readyTimeoutMs, &readyError))
		{
			runtime->cameraStatus = readyError;
			runtime->running = false;
			return false;
		}
		runtime->cameraStatus = QStringLiteral("相机已连接并取得扫描前新帧。");
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

	// 日志已按天归档到 Log/<yyyy-MM-dd>/，把平铺的 "Log/xxx.txt" 重定向到当天目录后再查找（每次调用重算日期，跨天自动跟随）。
	QString resolvedRelative = relativePath;
	if (relativePath.startsWith("Log/") && !relativePath.mid(4).contains('/'))
	{
		const QString logDay = QDateTime::currentDateTime().toString("yyyy-MM-dd");
		resolvedRelative = QString("Log/%1/%2").arg(logDay, relativePath.mid(4));
	}

	const QString filePath = AppPaths::WritablePath(resolvedRelative);
	if (filePath.isEmpty())
	{
		if (forceRefresh || m_sLastRobotLogFilePath != resolvedRelative)
		{
			m_pRobotLogText->setPlainText(QString("未找到日志文件：%1").arg(resolvedRelative));
			m_sLastRobotLogFilePath = resolvedRelative;
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
		file.readLine();  // 丢弃可能被截断的半行，避免从 UTF-8 多字节字符中间开始解码
	}
	const QString text = QString::fromUtf8(file.readAll());  // 日志统一 UTF-8 写入，按 UTF-8 解码
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
		constexpr int kSkjCameraControlPort = 50006;
			auto ensureControlTarget = [this, kSkjCameraControlPort](QString* error) -> bool
		{
			const int unitIndex = CurrentRobotUnitIndex();
			QString cameraIP;
			int ignoredReceivePort = 0;
			if (!LoadGrooveCameraEndpointForUnit(unitIndex, cameraIP, ignoredReceivePort)
				|| cameraIP.trimmed().isEmpty())
			{
				if (error != nullptr)
				{
					*error = "未读取到当前机器人扫描相机的 DeviceAddress。";
				}
				return false;
			}
			if (m_skjCameraControlClient == nullptr)
			{
				m_skjCameraControlClient = new SKJCameraControlClient();
			}
				return m_skjCameraControlClient->EnsureConnected(cameraIP, kSkjCameraControlPort, error);
			};
		auto acquireCameraOperation = [this](const QString& owner, QString* error) -> RobotOperationLease::Ptr
			{
				const T_CONTRAL_UNIT* unit = CurrentContralUnit();
				RobotDriverAdaptor* driver = unit != nullptr
					? static_cast<RobotDriverAdaptor*>(unit->pUnitDriver)
					: nullptr;
				return RobotOperationLease::TryAcquire(driver, owner, error);
			};
		auto refreshCameraParams = [this, ensureControlTarget, acquireCameraOperation](SKJCameraParameterValues& values, QString* error) -> bool
		{
			const auto operationLease = acquireCameraOperation(QStringLiteral("读取相机参数"), error);
			if (!operationLease)
			{
				return false;
			}
			if (!ensureControlTarget(error) || m_skjCameraControlClient == nullptr)
			{
				return false;
			}
			return m_skjCameraControlClient->ReadParameters(values, error);
		};
		auto setCameraParam = [this, ensureControlTarget, acquireCameraOperation](SKJCameraControlClient::Parameter parameter, int value, QString* error) -> bool
		{
			const auto operationLease = acquireCameraOperation(QStringLiteral("修改相机参数"), error);
			if (!operationLease)
			{
				return false;
			}
			if (!ensureControlTarget(error) || m_skjCameraControlClient == nullptr)
			{
				return false;
			}
			return m_skjCameraControlClient->SetParameter(parameter, value, error);
		};
		auto setLaserEnabled = [this, ensureControlTarget, acquireCameraOperation](bool enabled, QString* error) -> bool
		{
			const auto operationLease = acquireCameraOperation(
				enabled ? QStringLiteral("开启相机激光") : QStringLiteral("关闭相机激光"), error);
			if (!operationLease)
			{
				return false;
			}
			if (m_skjCameraControlClient != nullptr)
			{
				m_skjCameraControlClient->Disconnect();
			}

			bool ok = false;
			if (!ensureControlTarget(error) || m_skjCameraControlClient == nullptr)
			{
				ok = false;
			}
			else
			{
				ok = m_skjCameraControlClient->SetLaserEnabled(enabled, error);
			}

			if (m_skjCameraControlClient != nullptr)
			{
				m_skjCameraControlClient->Disconnect();
			}
			return ok;
		};
		GroovePointCloudDialog* dialog = new GroovePointCloudDialog(
			refreshCameraParams,
			setCameraParam,
			setLaserEnabled,
			this);
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

	m_pGroovePointCloudDialog->setWindowFlag(Qt::WindowStaysOnTopHint, true);
	m_pGroovePointCloudDialog->show();
	m_pGroovePointCloudDialog->raise();
	m_pGroovePointCloudDialog->activateWindow();
}

void QtWidgetsApplication4::CloseGrooveCameraPreviewWindow()
{
	if (ui.GrooveCameraTestBtn != nullptr && ui.GrooveCameraTestBtn->isChecked())
	{
		ui.GrooveCameraTestBtn->setChecked(false);
	}
	else if (m_grooveCameraDisplayTimer != nullptr)
	{
		m_grooveCameraDisplayTimer->stop();
	}

	m_sGrooveCameraStatusText = "已暂停坡口相机预览。";
	if (m_pGroovePointCloudDialog != nullptr)
	{
		QPointer<QWidget> previewDialog(m_pGroovePointCloudDialog);
		previewDialog->close();
	}
}

void QtWidgetsApplication4::OpenPointCloudViewerDialog()
{
	if (m_pPointCloudViewerDialog == nullptr)
	{
		PointCloudViewerDialog* dialog = new PointCloudViewerDialog(CurrentRobotName(), this);
		m_pPointCloudViewerDialog = dialog;
		connect(dialog, &QObject::destroyed, this, [this]()
			{
				m_pPointCloudViewerDialog = nullptr;
			});
	}

	m_pPointCloudViewerDialog->show();
	m_pPointCloudViewerDialog->raise();
	m_pPointCloudViewerDialog->activateWindow();
}

void QtWidgetsApplication4::StartGrooveCameraPreview()
{
	if (ui.GrooveCameraTestBtn != nullptr)
	{
		if (!ui.GrooveCameraTestBtn->isChecked())
		{
			ui.GrooveCameraTestBtn->setChecked(true);
		}
		else
		{
			OpenGroovePointCloudDialog();
		}
		return;
	}

	GrooveCameraTest(true);
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
		RobotDriverAdaptor* cameraDriver = RobotDataHelper::GetRobotDriver(m_pContralUnit, unitIndex);
		const QString cameraOwner = RobotOperationLease::CurrentOwner(cameraDriver);
		if (!cameraOwner.isEmpty())
		{
			QMessageBox::warning(this, "坡口相机测试",
				QString("当前机器人正在执行“%1”，不能打开预览并清空本轮相机缓存。").arg(cameraOwner));
			ui.GrooveCameraTestBtn->setChecked(false);
			RefreshRobotOperationAvailability();
			return;
		}
		if (!EnsureScanCameraRunningForUnit(unitIndex, cameraIP, true))
		{
			QMessageBox::warning(this, "坡口相机测试", "未读取到当前机器人扫描相机的 DeviceAddress。");
			ui.GrooveCameraTestBtn->setChecked(false);
			return;
		}

		const CameraRuntime* runtime = m_scanCameraRuntimes.value(unitIndex, nullptr);
		const int cameraPort = runtime != nullptr
			? runtime->cameraPort
			: 0;
		const QString receiveMode = runtime != nullptr && !runtime->receiveMode.isEmpty()
			? runtime->receiveMode
			: (m_bUseSharedScanCameraReceiver ? "UDP共享接收" : "TCP独立连接");
		const QString portName = m_bUseSharedScanCameraReceiver ? "UDP端口" : "TCP端口";
		m_sGrooveCameraStatusText = QString("正在预览 Robot%1 扫描相机：%2\n%3：%4\n接收模式：%5")
			.arg(unitIndex)
			.arg(cameraIP)
			.arg(portName)
			.arg(cameraPort > 0 ? cameraPort : (m_bUseSharedScanCameraReceiver ? 50004 : 50006))
			.arg(receiveMode);
		OpenGroovePointCloudDialog();
		if (m_pGroovePointCloudDialog != nullptr)
		{
			static_cast<GroovePointCloudDialog*>(m_pGroovePointCloudDialog)->ClearPreview(
				"正在等待相机帧...",
				m_sGrooveCameraStatusText);
			static_cast<GroovePointCloudDialog*>(m_pGroovePointCloudDialog)->RefreshCameraControlParams();
			static_cast<GroovePointCloudDialog*>(m_pGroovePointCloudDialog)->SetImageTransportToggleHandler(
				[this, unitIndex](bool enabled)
				{
					RobotDriverAdaptor* toggleDriver = RobotDataHelper::GetRobotDriver(m_pContralUnit, unitIndex);
					QString leaseError;
					const auto cameraSettingLease = RobotOperationLease::TryAcquire(
						toggleDriver, QStringLiteral("相机图像传输设置"), &leaseError);
					if (!cameraSettingLease)
					{
						if (m_pGroovePointCloudDialog != nullptr)
						{
							static_cast<GroovePointCloudDialog*>(m_pGroovePointCloudDialog)->ShowCameraControlMessage(
								leaseError + " 图像传输设置未下发。", false);
						}
						return;
					}
					const int toggleUnitIndex = unitIndex;
					CameraRuntime* toggleRuntime = m_scanCameraRuntimes.value(toggleUnitIndex, nullptr);
					if (toggleRuntime != nullptr && toggleRuntime->tcpWorker != nullptr)
					{
						QPointer<ScanCameraSkjWorker> worker(toggleRuntime->tcpWorker);
						if (worker->thread() == QThread::currentThread())
						{
							worker->setImageTransportEnabled(enabled);
						}
						else
						{
							// 图像口连接可能触发 SDK 网络超时，不能用 BlockingQueuedConnection
							// 卡住 GUI 和固定红色 STOP；租约随 queued functor 保持到实际执行结束。
							QMetaObject::invokeMethod(worker.data(), [worker, enabled, cameraSettingLease]()
								{
									if (worker != nullptr)
									{
										worker->setImageTransportEnabled(enabled);
									}
								}, Qt::QueuedConnection);
						}
					}
					else if (CameraFrameCache* toggleCache = ScanCameraCacheForUnit(toggleUnitIndex))
					{
						toggleCache->SetImageTransportEnabled(enabled);  // worker 未建时先记状态，连接时生效
					}
				});
			static_cast<GroovePointCloudDialog*>(m_pGroovePointCloudDialog)->SetFrameBufferCountValue(
				MeasureThenWeldRuntimeConfig::LoadSkjFrameBufferCount());
			static_cast<GroovePointCloudDialog*>(m_pGroovePointCloudDialog)->SetFrameBufferCountHandler(
				[this](int count)
				{
					// 接收缓冲是全局参数：持久化后广播给全部单元。SDK 明示修改缓冲帧数会清空
					// 已缓冲未取走的帧（skjcamera.h SetFrameBufferCount 注释），扫描流程运行中
					// 只存不发（写 cache 待相机重连生效），避免采集中途清 FIFO 静默丢帧。
					MeasureThenWeldRuntimeConfig::SaveSkjFrameBufferCount(count);
					std::vector<std::pair<int, RobotOperationLease::Ptr>> cameraSettingLeases;
					bool canApplyNow = !HasRunningMeasureThenWeldFlow();
					if (canApplyNow)
					{
						for (auto it = m_scanCameraRuntimes.constBegin(); it != m_scanCameraRuntimes.constEnd(); ++it)
						{
							RobotDriverAdaptor* driver = RobotDataHelper::GetRobotDriver(m_pContralUnit, it.key());
							QString leaseError;
							auto lease = RobotOperationLease::TryAcquire(
								driver, QStringLiteral("相机接收缓冲设置"), &leaseError);
							if (!lease)
							{
								canApplyNow = false;
								break;
							}
							cameraSettingLeases.emplace_back(it.key(), std::move(lease));
						}
					}
					for (auto it = m_scanCameraRuntimes.constBegin(); it != m_scanCameraRuntimes.constEnd(); ++it)
					{
						CameraRuntime* bufferRuntime = it.value();
						if (bufferRuntime == nullptr)
						{
							continue;
						}
						if (canApplyNow && bufferRuntime->tcpWorker != nullptr)
						{
							const auto leaseIt = std::find_if(
								cameraSettingLeases.cbegin(), cameraSettingLeases.cend(),
								[unitIndex = it.key()](const auto& entry) { return entry.first == unitIndex; });
							const RobotOperationLease::Ptr settingLease = leaseIt != cameraSettingLeases.cend()
								? leaseIt->second
								: RobotOperationLease::Ptr();
							QPointer<ScanCameraSkjWorker> worker(bufferRuntime->tcpWorker);
							if (worker->thread() == QThread::currentThread())
							{
								worker->setFrameBufferCount(count);
							}
							else
							{
								QMetaObject::invokeMethod(worker.data(), [worker, count, settingLease]()
									{
										if (worker != nullptr)
										{
											worker->setFrameBufferCount(count);
										}
									}, Qt::QueuedConnection);
							}
						}
						else if (bufferRuntime->cache != nullptr)
						{
							bufferRuntime->cache->SetSkjFrameBufferCount(count);
						}
					}
					if (m_pGroovePointCloudDialog != nullptr)
					{
						static_cast<GroovePointCloudDialog*>(m_pGroovePointCloudDialog)->ShowCameraControlMessage(
							!canApplyNow
								? QString("接收缓冲帧数已保存为 %1：扫描流程运行中不打断采集，相机下次连接时生效。").arg(count)
								: QString("接收缓冲帧数已排队应用：%1").arg(count),
							true);
					}
				});
		}
		if (CameraFrameCache* liveCache = ScanCameraCacheForUnit(unitIndex))
		{
			liveCache->SetLiveImageEnabled(true);  // 预览期间让 SKJ worker 同步取相机图像供显示
		}
		if (m_grooveCameraDisplayTimer != nullptr)
		{
			m_grooveCameraDisplayTimer->start(33);
		}
	}
	else
	{
		if (m_grooveCameraDisplayTimer != nullptr)
		{
			m_grooveCameraDisplayTimer->stop();
		}
		if (CameraFrameCache* liveCache = ScanCameraCacheForUnit(CurrentRobotUnitIndex()))
		{
			liveCache->SetLiveImageEnabled(false);
		}
		m_sGrooveCameraStatusText = "已停止坡口相机预览。";
		if (m_pGroovePointCloudDialog != nullptr)
		{
			static_cast<GroovePointCloudDialog*>(m_pGroovePointCloudDialog)->ClearPreview(
				m_sGrooveCameraStatusText,
				m_sGrooveCameraStatusText);
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
		cameraPort = m_bUseSharedScanCameraReceiver ? 50004 : 50006;
	}
	const QString receiveMode = runtime != nullptr && !runtime->receiveMode.isEmpty()
		? runtime->receiveMode
		: (m_bUseSharedScanCameraReceiver ? "UDP共享接收" : "TCP独立连接");
	const QString portName = m_bUseSharedScanCameraReceiver ? "UDP端口" : "TCP端口";
	const QString receiveCountName = m_bUseSharedScanCameraReceiver ? "UDP接收次数" : "TCP接收次数";

	QStringList diagnosticLines;
	diagnosticLines
		<< QString("当前机器人: Robot%1").arg(unitIndex)
		<< QString("接收模式: %1").arg(receiveMode)
		<< QString("当前相机IP: %1").arg(cameraIP)
		<< QString("%1: %2").arg(portName).arg(cameraPort)
		<< QString("相机线程状态: %1").arg(runtime != nullptr && !runtime->cameraStatus.isEmpty() ? runtime->cameraStatus : "未收到线程状态")
		<< QString("%1: %2").arg(receiveCountName).arg(runtime != nullptr ? runtime->datagramCount : 0)
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
		m_sGrooveCameraStatusText = diagnosticLines.join('\n');
		if (m_pGroovePointCloudDialog != nullptr)
		{
			static_cast<GroovePointCloudDialog*>(m_pGroovePointCloudDialog)->ClearPreview(
				diagnosticLines.join("  "),
				m_sGrooveCameraStatusText);
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
	m_sGrooveCameraStatusText = text;
	if (m_pGroovePointCloudDialog != nullptr)
	{
		const QString statusText = QString("Robot%1  %2:%3")
			.arg(unitIndex)
			.arg(cameraIP)
			.arg(cameraPort);
		static_cast<GroovePointCloudDialog*>(m_pGroovePointCloudDialog)->SetFrame(
			latestFrame,
			statusText,
			m_sGrooveCameraStatusText);
	}
	if (m_pGroovePointCloudDialog != nullptr)
	{
		static_cast<GroovePointCloudDialog*>(m_pGroovePointCloudDialog)->SetCameraImage(cache->LatestImage());
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
	QString leaseError;
	const auto operationLease = RobotOperationLease::TryAcquire(
		pRobotDriverAdaptor, QStringLiteral("上传机器人测试服务"), &leaseError);
	if (!operationLease)
	{
		QMessageBox::warning(this, "测试程序", leaseError);
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

		const QString serviceLibPath = FindInstalledResourcePath("SDK/FANUC/FanucServiceLib.kl");
		const QString residentServicePath = FindInstalledResourcePath("SDK/FANUC/FanucResidentService.kl");
		const QString monitorServicePath = FindInstalledResourcePath("SDK/FANUC/FanucMonitorService.kl");
		const QString jobRunnerPath = FindInstalledResourcePath("SDK/FANUC/FanucJobRunner.kl");
		const QString loadJogBufferPath = FindInstalledResourcePath("SDK/FANUC/LOADJOGBUF.kl");
		const QString startAllPath = FindInstalledResourcePath("SDK/FANUC/STARTALL.tp");
		const QString joglPath = FindInstalledResourcePath("SDK/FANUC/FANUC_JOGL.ls");
		const QString jogjPath = FindInstalledResourcePath("SDK/FANUC/FANUC_JOGJ.ls");
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
	PageOpenTrace trace("工艺参数");
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
		m_pWeldProcessPage = new WeldProcessDialog(*currentUnit, m_pContralUnit, targetStack);
		m_nWeldProcessPageUnitIndex = currentUnitIndex;
		PrepareEmbeddedPage(m_pWeldProcessPage, targetStack);
	}
	ShowCurrentEmbeddedPage(m_pWeldProcessPage);
}

void QtWidgetsApplication4::OpenFunctionTestDialog()
{
	PageOpenTrace trace("功能测试");
	if (!RequirePermission(kRoleEngineer, "功能测试"))
	{
		return;
	}
	const int currentUnitIndex = CurrentRobotUnitIndex();
	QStackedWidget* targetStack = CurrentEmbeddedTargetStack();
	if (m_pFunctionTestPage != nullptr && m_nFunctionTestPageUnitIndex != currentUnitIndex)
	{
		if (RobotOperationLease::AnyActive())
		{
			QMessageBox::warning(this, "功能测试",
				QString("机器人硬件操作正在运行（%1），不能切换功能测试目标。")
					.arg(RobotOperationLease::ActiveSummary()));
			return;
		}
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
	PageOpenTrace trace("先测后焊");
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

	auto startCamera = [this](int unitIndex, QString& cameraIP) -> bool
		{
			if (!EnsureScanCameraRunningForUnit(unitIndex, cameraIP, true))
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
			m_sGrooveCameraStatusText = "先测后焊已接管相机帧，主界面预览已暂停。";
			return true;
		};

	auto stopCamera = [this]()
		{
			CloseGrooveCameraPreviewWindow();
		};

	auto cameraCacheForUnit = [this](int unitIndex) -> CameraFrameCache*
		{
			CameraFrameCache* cameraCache = ScanCameraCacheForUnit(unitIndex);
			if (cameraCache == nullptr)
			{
				QString ignoredIP;
				EnsureScanCameraRunningForUnit(unitIndex, ignoredIP, false);
				cameraCache = ScanCameraCacheForUnit(unitIndex);
			}
			return cameraCache;
		};

	QPointer<MeasureThenWeldDialog> existingPage = m_measureThenWeldPages.value(currentUnitIndex);
	if (existingPage != nullptr)
	{
		existingPage->setProperty("_requires_live_account_session", true);
		existingPage->SetLiveSessionGuard([this]()
			{
				return ValidateCurrentAccountSession(QStringLiteral("继续先测后焊流程"));
			});
		existingPage->ReloadSelectors();
		existingPage->show();
		existingPage->raise();
		existingPage->activateWindow();
		m_pMeasureThenWeldPage = existingPage;
		m_nMeasureThenWeldPageUnitIndex = currentUnitIndex;
		return;
	}

	MeasureThenWeldDialog* page = new MeasureThenWeldDialog(
		m_pContralUnit,
		currentUnitIndex,
		startCamera,
		stopCamera,
		cameraCacheForUnit,
		this);
	page->setWindowModality(Qt::NonModal);
	page->setProperty("_requires_live_account_session", true);
	page->SetLiveSessionGuard([this]()
		{
			return ValidateCurrentAccountSession(QStringLiteral("继续先测后焊流程"));
		});
	// 提前展示同机占用者；真正的无竞态互锁仍由流程入口 TryAcquire 完成。
	page->SetPreStartGuard([this, currentUnitIndex](QString& reason) -> bool
		{
			RobotDriverAdaptor* driver = RobotDataHelper::GetRobotDriver(m_pContralUnit, currentUnitIndex);
			const QString owner = RobotOperationLease::CurrentOwner(driver);
			if (!owner.isEmpty())
			{
				reason = QString("当前机器人正在执行“%1”，请等待其安全结束。").arg(owner);
				return false;
			}
			return true;
		});
	// 扫描数据在线上传钩子：流程成功后按「自动上传」开关入队（钩子回调已在 UI 线程）。
	page->SetScanDataUploadHook([this](const QString& caseDir)
		{
			if (OnlineServicesConfig::AutoUploadEnabled())
			{
				EnsureScanDataUploader()->QueueUpload(caseDir);
			}
		});
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
	PageOpenTrace trace("测量焊接参数");
	if (!RequirePermission(kRoleEngineer, "测量焊接参数"))
	{
		return;
	}
	QStackedWidget* targetStack = CurrentEmbeddedTargetStack();
	if (m_pPreciseMeasureEditPage == nullptr)
	{
		m_pPreciseMeasureEditPage = new PreciseMeasureEditDialog(
			m_pContralUnit,
			targetStack,
			false,
			[this]() { StartGrooveCameraPreview(); });
		PrepareEmbeddedPage(m_pPreciseMeasureEditPage, targetStack);
	}
	ShowCurrentEmbeddedPage(m_pPreciseMeasureEditPage);
}

void QtWidgetsApplication4::OpenPositionTeachDialog()
{
	PageOpenTrace trace("扫描位置示教");
	PreciseMeasureEditDialog* dialog = new PreciseMeasureEditDialog(
		m_pContralUnit,
		this,
		true,
		[this]() { StartGrooveCameraPreview(); });
	dialog->setAttribute(Qt::WA_DeleteOnClose);
	dialog->setProperty("_requires_live_account_session", true);
	ApplyDebugLogVisibility(dialog);
	dialog->show();
	dialog->raise();
	dialog->activateWindow();
}

void QtWidgetsApplication4::OpenWeldSeamCompDialog()
{
	PageOpenTrace trace("焊道补偿");
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

void QtWidgetsApplication4::OpenResultArchiveDialog()
{
	PageOpenTrace trace("结果打包压缩");
	if (!RequirePermission(kRoleEngineer, "结果打包压缩"))
	{
		return;
	}
	// 已打开则前置，避免多个实例（弹窗带后台压缩线程）。
	if (ResultArchiveDialog* existing = findChild<ResultArchiveDialog*>())
	{
		existing->setProperty("_requires_live_account_session", true);
		existing->show();
		existing->raise();
		existing->activateWindow();
		return;
	}
	const QString resultRoot = AppPaths::WritablePath(QStringLiteral("Result"));
	QDir().mkpath(resultRoot);
	auto* dlg = new ResultArchiveDialog(resultRoot, this);
	dlg->setAttribute(Qt::WA_DeleteOnClose);
	dlg->setProperty("_requires_live_account_session", true);
	dlg->show();
}

ScanDataUploader* QtWidgetsApplication4::EnsureScanDataUploader()
{
	if (m_pScanDataUploader == nullptr)
	{
		m_pScanDataUploader = new ScanDataUploader(this);
	}
	return m_pScanDataUploader;
}

void QtWidgetsApplication4::OpenOnlineServicesDialog()
{
	PageOpenTrace trace("在线服务");
	if (!RequirePermission(kRoleEngineer, "在线服务"))
	{
		return;
	}

	const bool remoteAllowed = RoleLevel(m_sCurrentUserRole) >= RoleLevel(kRoleAdmin);  // admin 才见「远程数据」区

	// 无管理栈（异常兜底）时退回独立模态窗口；正常路径嵌入管理栈，作为管理页显示（不再单独弹窗）。
	if (m_pManagementStack == nullptr)
	{
		OnlineServicesDialog dialog(
			EnsureScanDataUploader(),
			[this]() { return RobotOperationLease::AnyActive() || HasRunningMeasureThenWeldFlow(); },
			false,
			remoteAllowed,
			[this]()
			{
				return ValidateCurrentAccountSession(QStringLiteral("执行在线服务管理员操作"))
					&& RoleLevel(m_sCurrentUserRole) >= RoleLevel(kRoleAdmin);
			},
			this);
		dialog.exec();
		return;
	}

	// 账号权限变化（admin 登出后换低权限账号再进）需重建，避免「远程数据」区串给非 admin。
	if (m_pOnlineServicesPage != nullptr && m_bOnlineServicesPageRemoteAllowed != remoteAllowed)
	{
		m_pManagementStack->removeWidget(m_pOnlineServicesPage);
		m_pOnlineServicesPage->deleteLater();
		m_pOnlineServicesPage = nullptr;
	}

	if (m_pOnlineServicesPage != nullptr)
	{
		ShowManagementEmbeddedPage(m_pOnlineServicesPage);
		return;
	}

	m_pOnlineServicesPage = new OnlineServicesDialog(
		EnsureScanDataUploader(),
		[this]() { return RobotOperationLease::AnyActive() || HasRunningMeasureThenWeldFlow(); },
		false,
		remoteAllowed,
		[this]()
		{
			return ValidateCurrentAccountSession(QStringLiteral("执行在线服务管理员操作"))
				&& RoleLevel(m_sCurrentUserRole) >= RoleLevel(kRoleAdmin);
		},
		m_pManagementStack);
	m_bOnlineServicesPageRemoteAllowed = remoteAllowed;
	PrepareEmbeddedPage(m_pOnlineServicesPage, m_pManagementStack);
	ShowManagementEmbeddedPage(m_pOnlineServicesPage);
}

void QtWidgetsApplication4::OpenCameraParamDialog()
{
	PageOpenTrace trace("相机参数");
	if (!RequirePermission(kRoleEngineer, "相机参数/手眼标定"))
	{
		return;
	}

	const int currentUnitIndex = CurrentRobotUnitIndex();
	auto startCamera = [this, currentUnitIndex](QString& cameraIP) -> bool
		{
			return EnsureScanCameraRunningForUnit(currentUnitIndex, cameraIP, true);
		};

	auto stopCamera = [this]()
		{
			CloseGrooveCameraPreviewWindow();
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
	QString leaseError;
	const auto operationLease = RobotOperationLease::TryAcquire(
		pRobotDriver, QStringLiteral("连接机器人"), &leaseError);
	if (!operationLease)
	{
		QMessageBox::warning(this, "机器人连接", leaseError);
		return;
	}

	const bool ok = pRobotDriver->InitSocket(pRobotDriver->m_sSocketIP.c_str(), static_cast<unsigned short>(pRobotDriver->m_nSocketPort));
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
	QString leaseError;
	const auto operationLease = RobotOperationLease::TryAcquire(
		pRobotDriver, QStringLiteral("断开机器人连接"), &leaseError);
	if (!operationLease)
	{
		QMessageBox::warning(this, "机器人断开", leaseError);
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
	QString leaseError;
	const auto operationLease = RobotOperationLease::TryAcquire(
		pRobotDriver, QStringLiteral("清除报警并上使能"), &leaseError);
	if (!operationLease)
	{
		QMessageBox::warning(this, "清除报警", leaseError);
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

void QtWidgetsApplication4::RobotEmergencyStop()
{
	struct StopTarget
	{
		RobotDriverAdaptor* driver = nullptr;
		QString owner;
	};
	struct StopResult
	{
		QString label;
		bool ok = false;
		QString detail;
	};

	const auto latchedTargets = RobotOperationLease::LatchGlobalCancellation();
	std::vector<StopTarget> targets;
	targets.reserve(latchedTargets.size());
	for (const RobotOperationLease::CancellationTarget& target : latchedTargets)
	{
		RobotDriverAdaptor* driver = const_cast<RobotDriverAdaptor*>(target.driver);
		if (driver == nullptr)
		{
			continue;
		}
		targets.push_back(StopTarget{ driver, target.owner });
	}

	if (targets.empty())
	{
		QMessageBox::information(this, "安全停止", "当前没有本软件跟踪的活动机器人硬件流程。");
		return;
	}

	if (m_pDashboardEmergencyStopBtn != nullptr)
	{
		m_pDashboardEmergencyStopBtn->setEnabled(false);
		m_pDashboardEmergencyStopBtn->setText("正在安全停止所有本软件活动机器人任务...");
	}

	QPointer<QtWidgetsApplication4> self(this);
	std::thread([self, targets = std::move(targets)]() mutable
		{
			std::vector<StopResult> results(targets.size());
			std::vector<std::thread> workers;
			workers.reserve(targets.size());
			for (size_t index = 0; index < targets.size(); ++index)
			{
				workers.emplace_back([index, &targets, &results]()
					{
						RobotDriverAdaptor* driver = targets[index].driver;
						QString label = QString::fromStdString(driver->m_sCustomName).trimmed();
						if (label.isEmpty())
						{
							label = QString::fromStdString(driver->m_sRobotName).trimmed();
						}
						if (label.isEmpty())
						{
							label = QString::fromStdString(driver->m_sSocketIP);
						}
						bool ok = false;
						if (FANUCRobotCtrl* fanucDriver = dynamic_cast<FANUCRobotCtrl*>(driver))
						{
							ok = fanucDriver->Prog_stop_Py();
						}
						else if (STEPRobotCtrl* stepDriver = dynamic_cast<STEPRobotCtrl*>(driver))
						{
							ok = stepDriver->AbortCurrentProgram();
						}
						else
						{
							driver->SetLastRobotError("当前机器人驱动没有可验证的安全终止实现。");
						}
						results[index] = StopResult{
							label,
							ok,
							DecodeRobotMessageText(driver->GetLastRobotError())
						};
						if (driver->m_pRobotLog != nullptr)
						{
							driver->m_pRobotLog->write(
								ok ? LogColor::SUCCESS : LogColor::ERR,
								"全局安全停止 result=%d owner=%s",
								ok ? 1 : 0,
								targets[index].owner.toLocal8Bit().constData());
						}
						// 清除闭锁后 UI 可能立即允许重载并销毁 driver；必须把它放在
						// 本 worker 对 driver 的最后一次访问。
						if (ok)
						{
							RobotOperationLease::MarkMotionCompleted(driver);
							RobotOperationLease::ConfirmCancellationHandled(driver);
						}
					});
			}
			for (std::thread& worker : workers)
			{
				worker.join();
			}

			QMetaObject::invokeMethod(qApp, [self, results = std::move(results)]()
				{
					if (self == nullptr)
					{
						return;
					}
					QStringList lines;
					bool allOk = true;
					for (const StopResult& result : results)
					{
						allOk = allOk && result.ok;
						lines << (result.ok
							? QString("%1：本软件任务已取消，机器人侧已确认其程序终止或不存在").arg(result.label)
							: QString("%1：停机未确认，已保持全局闭锁。%2").arg(result.label, result.detail));
					}
					if (self->m_pDashboardEmergencyStopBtn != nullptr)
					{
						self->m_pDashboardEmergencyStopBtn->setText("安全停止所有本软件活动机器人任务（不可从主页删除）");
						self->m_pDashboardEmergencyStopBtn->setEnabled(true);
					}
					if (allOk)
					{
						QMessageBox::information(self, "安全停止", lines.join('\n'));
					}
					else
					{
						// 停机未确认时红色按钮必须立即可重试；非模态提示不能截断 STOP。
						QMessageBox* warning = new QMessageBox(
							QMessageBox::Critical,
							"安全停止未完全确认",
							lines.join('\n'),
							QMessageBox::Ok,
							self);
						warning->setAttribute(Qt::WA_DeleteOnClose);
						warning->setWindowModality(Qt::NonModal);
						warning->show();
						if (self->ui.statusBar != nullptr)
						{
							self->ui.statusBar->showMessage(
								"安全停止尚未全部确认；全局闭锁保留，可立即再次点击红色按钮重试。",
								15000);
						}
					}
					self->RefreshDashboardConnectionState();
				}, Qt::QueuedConnection);
		}).detach();
}

void QtWidgetsApplication4::RobotSwitchStepMode()
{
	if (RobotOperationLease::AnyActive())
	{
		if (ui.statusBar != nullptr)
		{
			ui.statusBar->showMessage(
				QString("机器人硬件操作正在运行（%1）；STEP 模式输入已冻结。")
					.arg(RobotOperationLease::ActiveSummary()),
				8000);
		}
		return;
	}
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
		{ "开始/运行 (START=4)", MODEKEY::START }
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

	QString leaseError;
	const auto operationLease = RobotOperationLease::TryAcquire(
		pRobotDriver, QStringLiteral("切换 STEP 机器人模式"), &leaseError);
	if (!operationLease)
	{
		QMessageBox::warning(this, "模式切换", leaseError);
		return;
	}

	if (selectedMode == MODEKEY::START)
	{
		QPointer<QtWidgetsApplication4> self(this);
		std::thread([self, pRobotDriver, pStepDriver, selectedMode, operationLease]()
			{
				const bool modeOk = pStepDriver->SetSysMode(selectedMode);
				const int startDone = modeOk ? pStepDriver->CheckRobotDone(200, 1800000) : -1;
				const bool flowOk = modeOk && startDone > 0;
				if (pRobotDriver->m_pRobotLog != nullptr)
				{
					pRobotDriver->m_pRobotLog->write(
						flowOk ? LogColor::SUCCESS : LogColor::WARNING,
						"主页STEP模式切换 | mode=%d(%s) result=%d done=%d",
						selectedMode, GetModeText(selectedMode), flowOk ? 1 : 0, startDone);
				}
				const QString errorText = flowOk
					? QString()
					: DecodeRobotMessageText(pStepDriver->GetLastRobotError());
				QMetaObject::invokeMethod(qApp, [self, flowOk, startDone, errorText]()
					{
						if (self == nullptr)
						{
							return;
						}
						const QString resultText = flowOk
							? QString("新时达机器人 START 流程已结束，CheckRobotDone=%1").arg(startDone)
							: (errorText.isEmpty()
								? QStringLiteral("新时达 START 失败，请查看日志或示教器报警。")
								: QString("新时达 START 失败：%1").arg(errorText));
						if (self->ui.statusBar != nullptr)
						{
							self->ui.statusBar->showMessage(resultText, 15000);
						}
						self->RefreshDashboardConnectionState();
					}, Qt::QueuedConnection);
			}).detach();
		if (ui.statusBar != nullptr)
		{
			ui.statusBar->show();
			ui.statusBar->showMessage(
				"STEP START 已在后台执行；如需终止，请使用主页固定红色安全停止按钮。", 8000);
		}
		return;
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
			QString("新时达机器人已切换到：%1")
				.arg(QString::fromUtf8(GetModeText(selectedMode))));
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
	QString leaseError;
	const auto operationLease = RobotOperationLease::TryAcquire(
		pRobotDriver, QStringLiteral("读取 Tool1 并写入枪工具"), &leaseError);
	if (!operationLease)
	{
		QMessageBox::warning(this, "读取Tool1", leaseError);
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
	if (!ConfigDatabase::HasIniFile(iniPath))
	{
		QMessageBox::warning(
			this,
			"读取Tool1",
			QString("配置库中未找到机器人参数，无法写入 GunTool：\n%1").arg(iniPathText));
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
	if (!EnsureRobotUiActionIdle(QStringLiteral("读取当前位置")))
	{
		return;
	}
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
	if (!EnsureRobotUiActionIdle(QStringLiteral("读取关节脉冲")))
	{
		return;
	}
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
	if (!EnsureRobotUiActionIdle(QStringLiteral("检查运行完成")))
	{
		return;
	}
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
	if (!EnsureRobotUiActionIdle(QStringLiteral("写读 INT 寄存器")))
	{
		return;
	}
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
	QString leaseError;
	const auto operationLease = RobotOperationLease::TryAcquire(
		pRobotDriver, QStringLiteral("主页写读寄存器"), &leaseError);
	if (!operationLease)
	{
		QMessageBox::warning(this, "写读INT寄存器", leaseError);
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
	if (!EnsureRobotUiActionIdle(QStringLiteral("设置速度")))
	{
		return;
	}
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
	QString leaseError;
	const auto operationLease = RobotOperationLease::TryAcquire(
		pRobotDriver, QStringLiteral("主页设置 TP 速度"), &leaseError);
	if (!operationLease)
	{
		QMessageBox::warning(this, "设置速度", leaseError);
		return;
	}

	const bool setOk = pRobotDriver->SetTpSpeed(speed);
	const std::string message = setOk ? GetStr("设置速度成功：%d", speed) : GetStr("设置速度失败：%d", speed);
	QMessageBox::information(this, "设置速度", DecodeRobotMessageText(message));
}

void QtWidgetsApplication4::FanucCallJobTest()
{
	if (!EnsureRobotUiActionIdle(QStringLiteral("调用任务")))
	{
		return;
	}
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
	QString leaseError;
	const auto operationLease = RobotOperationLease::TryAcquire(
		pRobotDriver, QStringLiteral("主页调用机器人任务"), &leaseError);
	if (!operationLease)
	{
		QMessageBox::warning(this, "调用任务", leaseError);
		return;
	}

	const QByteArray jobNameBytes = jobName.trimmed().toLocal8Bit();
	QPointer<QtWidgetsApplication4> self(this);
	std::thread([self, pRobotDriver, jobNameBytes, operationLease]()
		{
			const bool callOk = pRobotDriver->CallJob(jobNameBytes.constData());
			const int done = callOk ? pRobotDriver->CheckRobotDone(200, 1800000) : -1;
			const bool flowOk = callOk && done > 0;
			const QString detail = DecodeRobotMessageText(pRobotDriver->GetLastRobotError());
			const QString message = flowOk
				? QString("调用任务完成：%1，CheckRobotDone=%2")
					.arg(QString::fromLocal8Bit(jobNameBytes)).arg(done)
				: QString("调用任务失败：%1，CallJob=%2，CheckRobotDone=%3，详情=%4")
					.arg(QString::fromLocal8Bit(jobNameBytes))
					.arg(callOk ? QStringLiteral("OK") : QStringLiteral("FAIL"))
					.arg(done)
					.arg(detail);
			QMetaObject::invokeMethod(qApp, [self, message, flowOk]()
				{
					if (self == nullptr)
					{
						return;
					}
					if (self->ui.statusBar != nullptr)
					{
						self->ui.statusBar->showMessage(
							(flowOk ? QStringLiteral("成功：") : QStringLiteral("失败：")) + message,
							12000);
					}
					self->RefreshDashboardConnectionState();
				}, Qt::QueuedConnection);
		}).detach();
	if (ui.statusBar != nullptr)
	{
		ui.statusBar->show();
		ui.statusBar->showMessage(
			QString("任务 %1 已在后台执行；主页停止按钮仍可立即中止。").arg(jobName.trimmed()), 8000);
	}
}

void QtWidgetsApplication4::FanucUploadLsTest()
{
	if (!EnsureRobotUiActionIdle(QStringLiteral("发送 LS 程序")))
	{
		return;
	}
	RobotDriverAdaptor* pRobotDriver = GetCurrentRobotDriver(this);
	if (pRobotDriver == nullptr)
	{
		return;
	}

	const QString lsPath = FindInstalledResourcePath("SDK/FANUC/STARTALL.ls");
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
	QString leaseError;
	const auto operationLease = RobotOperationLease::TryAcquire(
		pFanucDriver, QStringLiteral("主页上传 LS 程序"), &leaseError);
	if (!operationLease)
	{
		QMessageBox::warning(this, "发送LS程序", leaseError);
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
	if (!EnsureRobotUiActionIdle(QStringLiteral("MOVL 测试")))
	{
		return;
	}
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
	QString leaseError;
	const auto operationLease = RobotOperationLease::TryAcquire(
		pRobotDriver, QStringLiteral("主页 MOVL 测试"), &leaseError);
	if (!operationLease)
	{
		QMessageBox::warning(this, "MOVL往返测试", leaseError);
		return;
	}

	const bool moveForward = m_bFanucMovlForward;
	m_bFanucMovlForward = !m_bFanucMovlForward;
	m_bFanucMovlRunning = true;
	ui.FanucMovlTestBtn->setEnabled(false);

	std::thread([this, pRobotDriver, moveForward, operationLease]()
		{
			T_ROBOT_COORS target;
			const bool currentOk = pRobotDriver->TryGetCurrentPos(target);
			if (currentOk)
			{
				target.dY += moveForward ? 100.0 : -100.0;
			}

			const bool moveOk = currentOk
				&& pRobotDriver->MoveByJob(target, T_ROBOT_MOVE_SPEED(5.0, 0.0, 0.0), pRobotDriver->m_nExternalAxleType, "MOVL");
			const int done = moveOk ? pRobotDriver->CheckRobotDone(200, 1800000) : -1;
			const QString message = QString("MOVL %1 100mm\nMove=%2\nCheckRobotDone=%3")
				.arg(moveForward ? "Y+" : "Y-")
				.arg(moveOk ? "OK" : "FAIL")
				.arg(done);

			QMetaObject::invokeMethod(this, [this, message]()
				{
					m_bFanucMovlRunning = false;
					ui.FanucMovlTestBtn->setEnabled(true);
					if (ui.statusBar != nullptr)
					{
						ui.statusBar->showMessage(message, 12000);
					}
					RefreshDashboardConnectionState();
				}, Qt::QueuedConnection);
		}).detach();
}

void QtWidgetsApplication4::FanucMovjTest()
{
	if (!EnsureRobotUiActionIdle(QStringLiteral("MOVJ 测试")))
	{
		return;
	}
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
	QString leaseError;
	const auto operationLease = RobotOperationLease::TryAcquire(
		pRobotDriver, QStringLiteral("主页 MOVJ 测试"), &leaseError);
	if (!operationLease)
	{
		QMessageBox::warning(this, "MOVJ测试", leaseError);
		return;
	}

	m_bFanucMovjRunning = true;
	ui.FanucMovjTestBtn->setEnabled(false);

	std::thread([this, pRobotDriver, operationLease]()
		{
			T_ANGLE_PULSE target;
			const bool currentOk = pRobotDriver->TryGetCurrentPulse(target);
			const double j2PulseUnit = pRobotDriver->m_tAxisUnit.dLPulseUnit;
			const double j3PulseUnit = pRobotDriver->m_tAxisUnit.dUPulseUnit;
			const long j2DeltaPulse = j2PulseUnit == 0.0 ? 0 : static_cast<long>(std::lround(5.0 / j2PulseUnit));
			const long j3DeltaPulse = j3PulseUnit == 0.0 ? 0 : static_cast<long>(std::lround(5.0 / j3PulseUnit));
			if (currentOk)
			{
				target.nLPulse += j2DeltaPulse;
				target.nUPulse += j3DeltaPulse;
			}

			const bool moveOk = currentOk
				&& pRobotDriver->MoveByJob(target, T_ROBOT_MOVE_SPEED(1.0, 0.0, 0.0), pRobotDriver->m_nExternalAxleType, "MOVJ");
			const int done = moveOk ? pRobotDriver->CheckRobotDone(200, 1800000) : -1;
			const QString message = QString("MOVJ J2/J3 +5deg\nJ2DeltaPulse=%1\nJ3DeltaPulse=%2\nMove=%3\nCheckRobotDone=%4\n提示：固定TP当前用R[17]%，测试速度取1%%。")
				.arg(j2DeltaPulse)
				.arg(j3DeltaPulse)
				.arg(moveOk ? "OK" : "FAIL")
				.arg(done);

			QMetaObject::invokeMethod(this, [this, message]()
				{
					m_bFanucMovjRunning = false;
					ui.FanucMovjTestBtn->setEnabled(true);
					if (ui.statusBar != nullptr)
					{
						ui.statusBar->showMessage(message, 12000);
					}
					RefreshDashboardConnectionState();
				}, Qt::QueuedConnection);
		}).detach();
}

void QtWidgetsApplication4::FanucMoveZeroTest()
{
	if (!EnsureRobotUiActionIdle(QStringLiteral("运动到零位")))
	{
		return;
	}
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
	QString leaseError;
	const auto operationLease = RobotOperationLease::TryAcquire(
		pRobotDriver, QStringLiteral("主页运动到零位"), &leaseError);
	if (!operationLease)
	{
		QMessageBox::warning(this, "运动到零位", leaseError);
		return;
	}

	m_bFanucMoveZeroRunning = true;
	ui.FanucMoveZeroBtn->setEnabled(false);

	std::thread([this, pRobotDriver, operationLease]()
		{
			const T_ANGLE_PULSE zeroPulse = T_ANGLE_PULSE();
			const T_ROBOT_MOVE_SPEED speed(1.0, 0.0, 0.0);
			const bool moveOk = pRobotDriver->MoveByJob(zeroPulse, speed, pRobotDriver->m_nExternalAxleType, "MOVJ");
			const int done = moveOk ? pRobotDriver->CheckRobotDone(200, 1800000) : -1;
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
					if (ui.statusBar != nullptr)
					{
						ui.statusBar->showMessage(message, 15000);
					}
					RefreshDashboardConnectionState();
				}, Qt::QueuedConnection);
		}).detach();
}

void QtWidgetsApplication4::OpenRobotJogDialog()
{
	PageOpenTrace trace("点动控制");
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
		existingPage->setProperty("_requires_live_account_session", true);
		existingPage->show();
		existingPage->raise();
		existingPage->activateWindow();
		m_pRobotJogPage = existingPage;
		m_nRobotJogPageUnitIndex = currentUnitIndex;
		return;
	}

	RobotJogDialog* page = new RobotJogDialog(pRobotDriver, this);
	page->setWindowModality(Qt::NonModal);
	page->setProperty("_requires_live_account_session", true);
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
