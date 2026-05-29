#include "TouchKeyboardManager.h"

#include "ConfigDatabase.h"
#include "RobotDataHelper.h"

#include <QAbstractSpinBox>
#include <QApplication>
#include <QComboBox>
#include <QDateTime>
#include <QEvent>
#include <QFileInfo>
#include <QGridLayout>
#include <QGuiApplication>
#include <QHBoxLayout>
#include <QInputDevice>
#include <QKeyEvent>
#include <QLineEdit>
#include <QMouseEvent>
#include <QPlainTextEdit>
#include <QPointingDevice>
#include <QPointer>
#include <QProcess>
#include <QScreen>
#include <QStringList>
#include <QToolButton>
#include <QTextEdit>
#include <QTimer>
#include <QValidator>
#include <QVector>
#include <QVBoxLayout>
#include <QWidget>
#include <QWindow>

#include <functional>

#ifdef Q_OS_WIN
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#endif

namespace
{
constexpr qint64 kRecentTouchWindowMs = 1800;
constexpr qint64 kRecentPhysicalKeyWindowMs = 1200;
constexpr qint64 kKeyboardShowThrottleMs = 800;
constexpr int kKeyboardDragHoldMs = 350;

qint64 NowMs()
{
    return QDateTime::currentMSecsSinceEpoch();
}

bool IsTouchMouseEvent(const QMouseEvent* event)
{
    if (event == nullptr)
    {
        return false;
    }

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    const QPointingDevice* device = event->pointingDevice();
    return device != nullptr && device->type() == QInputDevice::DeviceType::TouchScreen;
#else
    return event->source() == Qt::MouseEventSynthesizedBySystem
        || event->source() == Qt::MouseEventSynthesizedByQt
        || event->source() == Qt::MouseEventSynthesizedByApplication;
#endif
}

QString TabTipPath()
{
#ifdef Q_OS_WIN
    const QString programFiles = qEnvironmentVariable("ProgramFiles", QStringLiteral("C:\\Program Files"));
    const QString tabTip = programFiles + QStringLiteral("\\Common Files\\Microsoft Shared\\ink\\TabTip.exe");
    if (QFileInfo::exists(tabTip))
    {
        return tabTip;
    }
#endif
    return QStringLiteral("TabTip.exe");
}

class TouchKeyboardPanel final : public QWidget
{
public:
    TouchKeyboardPanel(
        std::function<void(const QString&)> textCallback,
        std::function<void()> backspaceCallback,
        std::function<void()> clearCallback,
        std::function<void()> closeCallback)
        : QWidget(nullptr)
        , m_textCallback(std::move(textCallback))
        , m_backspaceCallback(std::move(backspaceCallback))
        , m_clearCallback(std::move(clearCallback))
        , m_closeCallback(std::move(closeCallback))
        , m_shiftButton(nullptr)
        , m_spaceButton(nullptr)
        , m_fullSwitchButton(nullptr)
        , m_numericSwitchButton(nullptr)
        , m_numericPanel(nullptr)
        , m_dragStartTimer(new QTimer(this))
        , m_numericMode(false)
        , m_shift(false)
        , m_dragCandidate(false)
        , m_dragging(false)
    {
        setWindowFlags(Qt::Tool | Qt::FramelessWindowHint | Qt::WindowStaysOnTopHint | Qt::WindowDoesNotAcceptFocus);
        setAttribute(Qt::WA_ShowWithoutActivating, true);
        setFocusPolicy(Qt::NoFocus);
        setObjectName(QStringLiteral("TouchKeyboardPanel"));
        setStyleSheet(QStringLiteral(
            "#TouchKeyboardPanel { background:#050b10; border:1px solid #3e6b82; }"
            "QToolButton { min-width:48px; min-height:42px; padding:4px 8px; color:#eaf9ff; "
            "background:#142839; border:1px solid #416a80; font-size:18px; font-weight:600; }"
            "QToolButton#NumericKey { min-width:72px; min-height:50px; font-size:22px; }"
            "QToolButton:pressed { background:#226274; border-color:#79e9f6; }"
            "QToolButton#WideKey { min-width:108px; }"
            "QToolButton#SpaceKey { min-width:260px; }"));

        QVBoxLayout* rootLayout = new QVBoxLayout(this);
        rootLayout->setContentsMargins(10, 10, 10, 10);
        rootLayout->setSpacing(8);

        m_dragStartTimer->setSingleShot(true);
        connect(m_dragStartTimer, &QTimer::timeout, this, [this]()
            {
                if (!m_dragCandidate)
                {
                    return;
                }

                m_dragging = true;
                setCursor(Qt::SizeAllCursor);
            });

        AddTextRow(rootLayout, QStringList{ "1", "2", "3", "4", "5", "6", "7", "8", "9", "0", ".", "-" });
        AddTextRow(rootLayout, QStringList{ "q", "w", "e", "r", "t", "y", "u", "i", "o", "p", "/", "_" });
        AddTextRow(rootLayout, QStringList{ "a", "s", "d", "f", "g", "h", "j", "k", "l", "@", "#", ":" });
        AddTextRow(rootLayout, QStringList{ "z", "x", "c", "v", "b", "n", "m", ",", "+", "*" });

        QWidget* actionRow = new QWidget(this);
        QHBoxLayout* actionLayout = new QHBoxLayout(actionRow);
        actionLayout->setContentsMargins(0, 0, 0, 0);
        actionLayout->setSpacing(8);
        m_fullKeyboardRows.append(actionRow);
        RegisterDragArea(actionRow);
        m_shiftButton = CreateButton(QStringLiteral("Shift"));
        m_shiftButton->setObjectName(QStringLiteral("WideKey"));
        connect(m_shiftButton, &QToolButton::clicked, this, [this]()
            {
                m_shift = !m_shift;
                RefreshLetterKeys();
            });
        actionLayout->addWidget(m_shiftButton);

        m_fullSwitchButton = CreateButton(QStringLiteral("123"));
        m_fullSwitchButton->setObjectName(QStringLiteral("WideKey"));
        connect(m_fullSwitchButton, &QToolButton::clicked, this, [this]()
            {
                SetNumericMode(true);
            });
        actionLayout->addWidget(m_fullSwitchButton);

        m_spaceButton = CreateButton(QStringLiteral("空格"));
        m_spaceButton->setObjectName(QStringLiteral("SpaceKey"));
        connect(m_spaceButton, &QToolButton::clicked, this, [this]()
            {
                if (m_textCallback)
                {
                    m_textCallback(QStringLiteral(" "));
                }
            });
        actionLayout->addWidget(m_spaceButton, 1);

        QToolButton* backspaceButton = CreateButton(QStringLiteral("退格"));
        backspaceButton->setObjectName(QStringLiteral("WideKey"));
        connect(backspaceButton, &QToolButton::clicked, this, [this]()
            {
                if (m_backspaceCallback)
                {
                    m_backspaceCallback();
                }
            });
        actionLayout->addWidget(backspaceButton);

        QToolButton* clearButton = CreateButton(QStringLiteral("清空"));
        clearButton->setObjectName(QStringLiteral("WideKey"));
        connect(clearButton, &QToolButton::clicked, this, [this]()
            {
                if (m_clearCallback)
                {
                    m_clearCallback();
                }
            });
        actionLayout->addWidget(clearButton);

        QToolButton* closeButton = CreateButton(QStringLiteral("关闭"));
        closeButton->setObjectName(QStringLiteral("WideKey"));
        connect(closeButton, &QToolButton::clicked, this, [this]()
            {
                if (m_closeCallback)
                {
                    m_closeCallback();
                }
            });
        actionLayout->addWidget(closeButton);

        rootLayout->addWidget(actionRow);

        m_numericPanel = CreateNumericPanel();
        rootLayout->addWidget(m_numericPanel);
        m_numericPanel->hide();
        installEventFilter(this);
    }

    void SetNumericMode(bool numericMode)
    {
        if (m_numericMode == numericMode)
        {
            return;
        }

        m_numericMode = numericMode;
        CancelDrag();
        for (QWidget* row : m_fullKeyboardRows)
        {
            if (row != nullptr)
            {
                row->setVisible(!m_numericMode);
            }
        }
        if (m_numericPanel != nullptr)
        {
            m_numericPanel->setVisible(m_numericMode);
        }
        adjustSize();
        ClampToVisibleScreen();
    }

    int PreferredHeight(int availableHeight) const
    {
        const int safeHeight = qMax(220, availableHeight - 24);
        if (m_numericMode)
        {
            return qMin(safeHeight, 420);
        }
        return qMin(safeHeight, 360);
    }

    int PreferredWidth(int availableWidth) const
    {
        const int safeWidth = qMax(240, availableWidth - 24);
        if (m_numericMode)
        {
            return qMin(safeWidth, 420);
        }
        return qMin(safeWidth, 920);
    }

    void ClampToVisibleScreen()
    {
        QScreen* screen = nullptr;
        if (windowHandle() != nullptr)
        {
            screen = windowHandle()->screen();
        }
        if (screen == nullptr)
        {
            screen = QGuiApplication::screenAt(frameGeometry().center());
        }
        if (screen == nullptr)
        {
            screen = QGuiApplication::primaryScreen();
        }
        if (screen == nullptr)
        {
            return;
        }

        const QRect available = screen->availableGeometry().adjusted(8, 8, -8, -8);
        QRect rect = geometry();
        if (rect.width() > available.width())
        {
            rect.setWidth(available.width());
        }
        if (rect.height() > available.height())
        {
            rect.setHeight(available.height());
        }
        if (rect.left() < available.left())
        {
            rect.moveLeft(available.left());
        }
        if (rect.top() < available.top())
        {
            rect.moveTop(available.top());
        }
        if (rect.right() > available.right())
        {
            rect.moveRight(available.right());
        }
        if (rect.bottom() > available.bottom())
        {
            rect.moveBottom(available.bottom());
        }
        if (geometry() != rect)
        {
            setGeometry(rect);
        }
    }

private:
    bool eventFilter(QObject* watched, QEvent* event) override
    {
        if (event == nullptr)
        {
            return QWidget::eventFilter(watched, event);
        }

        if (event->type() == QEvent::MouseButtonPress
            || event->type() == QEvent::MouseMove
            || event->type() == QEvent::MouseButtonRelease)
        {
            if (QMouseEvent* mouseEvent = static_cast<QMouseEvent*>(event))
            {
                return HandleDragEvent(watched, mouseEvent);
            }
        }

        return QWidget::eventFilter(watched, event);
    }

    void mousePressEvent(QMouseEvent* event) override
    {
        if (HandleDragEvent(this, event))
        {
            return;
        }

        QWidget::mousePressEvent(event);
    }

    void mouseMoveEvent(QMouseEvent* event) override
    {
        if (HandleDragEvent(this, event))
        {
            return;
        }

        QWidget::mouseMoveEvent(event);
    }

    void mouseReleaseEvent(QMouseEvent* event) override
    {
        if (HandleDragEvent(this, event))
        {
            return;
        }

        QWidget::mouseReleaseEvent(event);
    }

    bool HandleDragEvent(QObject* watched, QMouseEvent* event)
    {
        if (event == nullptr)
        {
            return false;
        }

        if (event->type() == QEvent::MouseButtonPress)
        {
            if (event->button() != Qt::LeftButton || IsKeyButtonAt(watched, event->position().toPoint()))
            {
                return false;
            }

            m_dragCandidate = true;
            m_dragging = false;
            m_pressGlobalPos = event->globalPosition().toPoint();
            m_dragOffset = m_pressGlobalPos - frameGeometry().topLeft();
            m_dragStartTimer->start(kKeyboardDragHoldMs);
            event->accept();
            return true;
        }

        if (event->type() == QEvent::MouseMove)
        {
            if (!m_dragCandidate)
            {
                return false;
            }

            const QPoint currentGlobalPos = event->globalPosition().toPoint();
            if (!m_dragging)
            {
                const int distance = (currentGlobalPos - m_pressGlobalPos).manhattanLength();
                if (distance > QApplication::startDragDistance())
                {
                    CancelDrag();
                }
                event->accept();
                return true;
            }

            move(currentGlobalPos - m_dragOffset);
            ClampToVisibleScreen();
            event->accept();
            return true;
        }

        if (event->type() == QEvent::MouseButtonRelease)
        {
            if (!m_dragCandidate && !m_dragging)
            {
                return false;
            }

            CancelDrag();
            ClampToVisibleScreen();
            event->accept();
            return true;
        }

        return false;
    }

    void CancelDrag()
    {
        if (m_dragStartTimer != nullptr)
        {
            m_dragStartTimer->stop();
        }
        m_dragCandidate = false;
        m_dragging = false;
        unsetCursor();
    }

    bool IsKeyButtonAt(QObject* watched, const QPoint& localPos) const
    {
        QWidget* watchedWidget = qobject_cast<QWidget*>(watched);
        if (watchedWidget == nullptr)
        {
            return false;
        }
        if (qobject_cast<QToolButton*>(watchedWidget) != nullptr)
        {
            return true;
        }

        QWidget* child = watchedWidget->childAt(localPos);
        while (child != nullptr)
        {
            if (qobject_cast<QToolButton*>(child) != nullptr)
            {
                return true;
            }
            child = child->parentWidget();
            if (child == watchedWidget)
            {
                break;
            }
        }
        return false;
    }

    QToolButton* CreateButton(const QString& text)
    {
        QToolButton* button = new QToolButton(this);
        button->setText(text);
        button->setFocusPolicy(Qt::NoFocus);
        return button;
    }

    void AddTextRow(QVBoxLayout* rootLayout, const QStringList& keys)
    {
        QWidget* rowWidget = new QWidget(this);
        QHBoxLayout* rowLayout = new QHBoxLayout(rowWidget);
        rowLayout->setContentsMargins(0, 0, 0, 0);
        rowLayout->setSpacing(8);
        m_fullKeyboardRows.append(rowWidget);
        RegisterDragArea(rowWidget);
        for (const QString& key : keys)
        {
            QToolButton* button = CreateButton(key);
            const bool isLetter = key.size() == 1 && key.at(0).isLetter();
            if (isLetter)
            {
                m_letterButtons.append(button);
            }
            connect(button, &QToolButton::clicked, this, [this, button, isLetter, key]()
                {
                    if (!m_textCallback)
                    {
                        return;
                    }

                    QString text = isLetter ? button->text() : key;
                    m_textCallback(text);
                    if (m_shift && isLetter)
                    {
                        m_shift = false;
                        RefreshLetterKeys();
                    }
                });
            rowLayout->addWidget(button);
        }
        rootLayout->addWidget(rowWidget);
    }

    QWidget* CreateNumericPanel()
    {
        QWidget* panel = new QWidget(this);
        RegisterDragArea(panel);
        QGridLayout* layout = new QGridLayout(panel);
        layout->setContentsMargins(0, 0, 0, 0);
        layout->setHorizontalSpacing(8);
        layout->setVerticalSpacing(8);

        const QStringList keys = {
            "7", "8", "9",
            "4", "5", "6",
            "1", "2", "3",
            "-", "0", "."
        };

        for (int index = 0; index < keys.size(); ++index)
        {
            const QString key = keys[index];
            QToolButton* button = CreateButton(key);
            button->setObjectName(QStringLiteral("NumericKey"));
            connect(button, &QToolButton::clicked, this, [this, key]()
                {
                    if (m_textCallback)
                    {
                        m_textCallback(key);
                    }
                });
            layout->addWidget(button, index / 3, index % 3);
        }

        m_numericSwitchButton = CreateButton(QStringLiteral("ABC"));
        m_numericSwitchButton->setObjectName(QStringLiteral("NumericKey"));
        connect(m_numericSwitchButton, &QToolButton::clicked, this, [this]()
            {
                SetNumericMode(false);
            });
        layout->addWidget(m_numericSwitchButton, 4, 0);

        QToolButton* backspaceButton = CreateButton(QStringLiteral("退格"));
        backspaceButton->setObjectName(QStringLiteral("NumericKey"));
        connect(backspaceButton, &QToolButton::clicked, this, [this]()
            {
                if (m_backspaceCallback)
                {
                    m_backspaceCallback();
                }
            });
        layout->addWidget(backspaceButton, 4, 1);

        QToolButton* clearButton = CreateButton(QStringLiteral("清空"));
        clearButton->setObjectName(QStringLiteral("NumericKey"));
        connect(clearButton, &QToolButton::clicked, this, [this]()
            {
                if (m_clearCallback)
                {
                    m_clearCallback();
                }
            });
        layout->addWidget(clearButton, 4, 2);

        QToolButton* closeButton = CreateButton(QStringLiteral("关闭"));
        closeButton->setObjectName(QStringLiteral("NumericKey"));
        connect(closeButton, &QToolButton::clicked, this, [this]()
            {
                if (m_closeCallback)
                {
                    m_closeCallback();
                }
            });
        layout->addWidget(closeButton, 5, 0, 1, 3);

        return panel;
    }

    void RegisterDragArea(QWidget* widget)
    {
        if (widget == nullptr)
        {
            return;
        }

        widget->installEventFilter(this);
    }

    void RefreshLetterKeys()
    {
        for (QToolButton* button : m_letterButtons)
        {
            if (button == nullptr)
            {
                continue;
            }
            button->setText(m_shift ? button->text().toUpper() : button->text().toLower());
        }
        if (m_shiftButton != nullptr)
        {
            m_shiftButton->setStyleSheet(m_shift ? QStringLiteral("background:#226274; border-color:#79e9f6;") : QString());
        }
    }

    std::function<void(const QString&)> m_textCallback;
    std::function<void()> m_backspaceCallback;
    std::function<void()> m_clearCallback;
    std::function<void()> m_closeCallback;
    QVector<QToolButton*> m_letterButtons;
    QVector<QWidget*> m_fullKeyboardRows;
    QToolButton* m_shiftButton;
    QToolButton* m_spaceButton;
    QToolButton* m_fullSwitchButton;
    QToolButton* m_numericSwitchButton;
    QWidget* m_numericPanel;
    QTimer* m_dragStartTimer;
    bool m_numericMode;
    bool m_shift;
    bool m_dragCandidate;
    bool m_dragging;
    QPoint m_pressGlobalPos;
    QPoint m_dragOffset;
};
}

TouchKeyboardManager::TouchKeyboardManager(QObject* parent)
    : QObject(parent)
    , m_pInstalledTarget(nullptr)
    , m_pKeyboardPanel(nullptr)
    , m_pKeyboardTarget(nullptr)
    , m_mode(Mode::Auto)
    , m_nStartMs(NowMs())
    , m_nLastTouchMs(-1000000)
    , m_nLastPhysicalKeyMs(-1000000)
    , m_nLastShowMs(-1000000)
{
}

void TouchKeyboardManager::Install(QObject* target)
{
    if (m_pInstalledTarget == target)
    {
        return;
    }

    if (m_pInstalledTarget != nullptr)
    {
        m_pInstalledTarget->removeEventFilter(this);
    }

    m_pInstalledTarget = target;
    if (m_pInstalledTarget != nullptr)
    {
        m_pInstalledTarget->installEventFilter(this);
    }
}

void TouchKeyboardManager::LoadSettings()
{
    QString storedMode;
    if (!ConfigDatabase::ReadSetting(ConfigPath(), QStringLiteral("General/Mode"), &storedMode))
    {
        storedMode = QStringLiteral("Auto");
    }
    m_mode = ModeFromStorageString(storedMode);
}

void TouchKeyboardManager::SaveSettings() const
{
    ConfigDatabase::WriteSetting(ConfigPath(), QStringLiteral("General/Mode"), ModeToStorageString(m_mode));
}

void TouchKeyboardManager::SetMode(Mode mode)
{
    if (m_mode == mode)
    {
        return;
    }

    m_mode = mode;
    SaveSettings();
}

TouchKeyboardManager::Mode TouchKeyboardManager::CurrentMode() const
{
    return m_mode;
}

QString TouchKeyboardManager::ConfigPath()
{
    return RobotDataHelper::BuildProjectPath(QStringLiteral("Data/TouchKeyboard.ini"));
}

QString TouchKeyboardManager::ModeToStorageString(Mode mode)
{
    switch (mode)
    {
    case Mode::Always:
        return QStringLiteral("Always");
    case Mode::Never:
        return QStringLiteral("Never");
    case Mode::Auto:
    default:
        return QStringLiteral("Auto");
    }
}

QString TouchKeyboardManager::ModeDisplayName(Mode mode)
{
    switch (mode)
    {
    case Mode::Always:
        return QStringLiteral("总是");
    case Mode::Never:
        return QStringLiteral("从不");
    case Mode::Auto:
    default:
        return QStringLiteral("自动");
    }
}

TouchKeyboardManager::Mode TouchKeyboardManager::ModeFromStorageString(const QString& value)
{
    const QString normalized = value.trimmed().toLower();
    if (normalized == QStringLiteral("always"))
    {
        return Mode::Always;
    }
    if (normalized == QStringLiteral("never"))
    {
        return Mode::Never;
    }
    return Mode::Auto;
}

bool TouchKeyboardManager::eventFilter(QObject* watched, QEvent* event)
{
    if (event == nullptr)
    {
        return false;
    }

    switch (event->type())
    {
    case QEvent::KeyPress:
        MarkPhysicalKeyInput();
        break;
    case QEvent::TouchBegin:
    case QEvent::TouchUpdate:
    case QEvent::TouchEnd:
        MarkTouchInput();
        break;
    case QEvent::MouseButtonPress:
    {
        QWidget* widget = qobject_cast<QWidget*>(watched);
        if (IsTouchMouseEvent(static_cast<QMouseEvent*>(event)))
        {
            MarkTouchInput();
        }

        // Global filters may also receive mouse events from non-QWidget objects
        // such as backing windows. Treat only a known widget outside both the
        // editor and keyboard as an intentional outside tap.
        if (widget != nullptr && !ShouldHandleWidget(widget) && !IsKeyboardWidget(widget))
        {
            HideKeyboard();
        }
        break;
    }
    case QEvent::FocusIn:
    case QEvent::MouseButtonRelease:
    {
        QWidget* widget = qobject_cast<QWidget*>(watched);
        if (ShouldHandleWidget(widget))
        {
            ScheduleKeyboardFor(widget);
        }
        break;
    }
    default:
        break;
    }

    return false;
}

bool TouchKeyboardManager::ShouldHandleWidget(QWidget* widget) const
{
    if (widget == nullptr || !widget->isEnabled() || !widget->isVisible())
    {
        return false;
    }

    if (const QLineEdit* edit = qobject_cast<QLineEdit*>(widget))
    {
        return !edit->isReadOnly();
    }

    if (const QTextEdit* edit = qobject_cast<QTextEdit*>(widget))
    {
        return !edit->isReadOnly();
    }

    if (const QPlainTextEdit* edit = qobject_cast<QPlainTextEdit*>(widget))
    {
        return !edit->isReadOnly();
    }

    if (qobject_cast<QAbstractSpinBox*>(widget) != nullptr)
    {
        return true;
    }

    if (const QComboBox* combo = qobject_cast<QComboBox*>(widget))
    {
        return combo->isEditable();
    }

    return false;
}

bool TouchKeyboardManager::ShouldShowKeyboard() const
{
    if (m_mode == Mode::Never)
    {
        return false;
    }
    if (m_mode == Mode::Always)
    {
        return true;
    }

    const qint64 now = NowMs();
    const bool recentTouch = (now - m_nLastTouchMs) <= kRecentTouchWindowMs;
    const bool recentPhysicalKey = (now - m_nLastPhysicalKeyMs) <= kRecentPhysicalKeyWindowMs;
    if (recentTouch)
    {
        return true;
    }
    if (recentPhysicalKey)
    {
        return false;
    }

    // Windows 对“是否外接键盘”的判断不是 100% 可靠，所以自动模式优先响应触摸，
    // 只有在明显是触摸设备且未发现硬键盘时，才主动弹出键盘。
    return HasTouchCapability() && !HasHardwareKeyboard();
}

void TouchKeyboardManager::MarkTouchInput()
{
    m_nLastTouchMs = NowMs();
}

void TouchKeyboardManager::MarkPhysicalKeyInput()
{
    m_nLastPhysicalKeyMs = NowMs();
}

void TouchKeyboardManager::ScheduleKeyboardFor(QWidget* widget)
{
    if (widget == nullptr)
    {
        return;
    }

    QPointer<QWidget> guard(widget);
    QTimer::singleShot(120, this, [this, guard]()
        {
            QWidget* target = guard.data();
            if (target == nullptr || !target->hasFocus() || !ShouldHandleWidget(target))
            {
                return;
            }

            if (ShouldShowKeyboard())
            {
                m_pKeyboardTarget = target;
                ShowKeyboard();
            }
        });
}

void TouchKeyboardManager::ShowKeyboard()
{
    const qint64 now = NowMs();
    const bool alreadyVisible = (m_pKeyboardPanel != nullptr && m_pKeyboardPanel->isVisible());
    if (!alreadyVisible && (now - m_nLastShowMs) < kKeyboardShowThrottleMs)
    {
        return;
    }
    m_nLastShowMs = now;

    EnsureKeyboardPanel();
    if (m_pKeyboardPanel == nullptr)
    {
        return;
    }

    if (TouchKeyboardPanel* panel = static_cast<TouchKeyboardPanel*>(m_pKeyboardPanel.data()))
    {
        panel->SetNumericMode(IsNumericInputWidget(m_pKeyboardTarget.data()));
    }
    PositionKeyboardPanel(m_pKeyboardTarget.data());
    m_pKeyboardPanel->show();
    m_pKeyboardPanel->raise();
}

void TouchKeyboardManager::HideKeyboard()
{
    if (m_pKeyboardPanel != nullptr)
    {
        m_pKeyboardPanel->hide();
    }
}

void TouchKeyboardManager::EnsureKeyboardPanel()
{
    if (m_pKeyboardPanel != nullptr)
    {
        return;
    }

    m_pKeyboardPanel = new TouchKeyboardPanel(
        [this](const QString& text) { SendKeyboardText(text); },
        [this]() { SendKeyboardBackspace(); },
        [this]() { SendKeyboardClear(); },
        [this]() { HideKeyboard(); });
}

void TouchKeyboardManager::PositionKeyboardPanel(QWidget* target)
{
    if (m_pKeyboardPanel == nullptr)
    {
        return;
    }

    QScreen* screen = nullptr;
    if (target != nullptr && target->window() != nullptr && target->window()->windowHandle() != nullptr)
    {
        screen = target->window()->windowHandle()->screen();
    }
    if (screen == nullptr)
    {
        screen = QGuiApplication::primaryScreen();
    }
    if (screen == nullptr)
    {
        m_pKeyboardPanel->resize(840, 300);
        return;
    }

    const QRect available = screen->availableGeometry();
    int width = qMin(920, qMax(360, available.width() - 24));
    int height = qMin(310, qMax(240, available.height() / 3));
    if (TouchKeyboardPanel* panel = static_cast<TouchKeyboardPanel*>(m_pKeyboardPanel.data()))
    {
        width = panel->PreferredWidth(available.width());
        height = panel->PreferredHeight(available.height());
    }
    int x = available.left() + (available.width() - width) / 2;
    int y = available.bottom() - height - 12;

    if (target != nullptr)
    {
        const QRect targetRect(target->mapToGlobal(QPoint(0, 0)), target->size());
        const QRect bottomRect(x, y, width, height);
        if (bottomRect.intersects(targetRect))
        {
            const int aboveY = targetRect.top() - height - 12;
            if (aboveY >= available.top() + 12)
            {
                y = aboveY;
            }
        }
    }

    m_pKeyboardPanel->setGeometry(x, y, width, height);
    if (TouchKeyboardPanel* panel = static_cast<TouchKeyboardPanel*>(m_pKeyboardPanel.data()))
    {
        panel->ClampToVisibleScreen();
    }
}

void TouchKeyboardManager::SendKeyboardText(const QString& text)
{
    QWidget* target = m_pKeyboardTarget.data();
    if (target == nullptr)
    {
        target = QApplication::focusWidget();
        m_pKeyboardTarget = target;
    }
    if (target == nullptr || !ShouldHandleWidget(target))
    {
        return;
    }

    target->setFocus(Qt::OtherFocusReason);
    if (QLineEdit* edit = qobject_cast<QLineEdit*>(target))
    {
        edit->insert(text);
        return;
    }
    if (QComboBox* combo = qobject_cast<QComboBox*>(target))
    {
        if (combo->isEditable() && combo->lineEdit() != nullptr)
        {
            combo->lineEdit()->insert(text);
        }
        return;
    }
    if (QTextEdit* edit = qobject_cast<QTextEdit*>(target))
    {
        edit->insertPlainText(text);
        return;
    }
    if (QPlainTextEdit* edit = qobject_cast<QPlainTextEdit*>(target))
    {
        edit->insertPlainText(text);
        return;
    }

    for (const QChar ch : text)
    {
        QKeyEvent pressEvent(QEvent::KeyPress, 0, Qt::NoModifier, QString(ch));
        QCoreApplication::sendEvent(target, &pressEvent);
        QKeyEvent releaseEvent(QEvent::KeyRelease, 0, Qt::NoModifier, QString(ch));
        QCoreApplication::sendEvent(target, &releaseEvent);
    }
}

void TouchKeyboardManager::SendKeyboardBackspace()
{
    QWidget* target = m_pKeyboardTarget.data();
    if (target == nullptr)
    {
        return;
    }
    target->setFocus(Qt::OtherFocusReason);

    if (QLineEdit* edit = qobject_cast<QLineEdit*>(target))
    {
        edit->backspace();
        return;
    }

    QKeyEvent pressEvent(QEvent::KeyPress, Qt::Key_Backspace, Qt::NoModifier);
    QCoreApplication::sendEvent(target, &pressEvent);
    QKeyEvent releaseEvent(QEvent::KeyRelease, Qt::Key_Backspace, Qt::NoModifier);
    QCoreApplication::sendEvent(target, &releaseEvent);
}

void TouchKeyboardManager::SendKeyboardClear()
{
    QWidget* target = m_pKeyboardTarget.data();
    if (target == nullptr)
    {
        return;
    }
    target->setFocus(Qt::OtherFocusReason);

    if (QLineEdit* edit = qobject_cast<QLineEdit*>(target))
    {
        edit->clear();
        return;
    }
    if (QTextEdit* edit = qobject_cast<QTextEdit*>(target))
    {
        edit->clear();
        return;
    }
    if (QPlainTextEdit* edit = qobject_cast<QPlainTextEdit*>(target))
    {
        edit->clear();
        return;
    }
    if (QComboBox* combo = qobject_cast<QComboBox*>(target))
    {
        if (combo->isEditable() && combo->lineEdit() != nullptr)
        {
            combo->lineEdit()->clear();
        }
        return;
    }

    QKeyEvent selectEvent(QEvent::KeyPress, Qt::Key_A, Qt::ControlModifier);
    QCoreApplication::sendEvent(target, &selectEvent);
    QKeyEvent deleteEvent(QEvent::KeyPress, Qt::Key_Delete, Qt::NoModifier);
    QCoreApplication::sendEvent(target, &deleteEvent);
}

bool TouchKeyboardManager::IsKeyboardWidget(QWidget* widget) const
{
    if (widget == nullptr || m_pKeyboardPanel == nullptr)
    {
        return false;
    }
    return widget == m_pKeyboardPanel.data() || m_pKeyboardPanel->isAncestorOf(widget);
}

bool TouchKeyboardManager::IsNumericInputWidget(QWidget* widget) const
{
    if (widget == nullptr)
    {
        return false;
    }

    const QString layoutHint = widget->property("touchKeyboardLayout").toString().trimmed().toLower();
    if (layoutHint == QStringLiteral("numeric") || layoutHint == QStringLiteral("number"))
    {
        return true;
    }

    if (qobject_cast<QAbstractSpinBox*>(widget) != nullptr)
    {
        return true;
    }

    if (const QLineEdit* edit = qobject_cast<QLineEdit*>(widget))
    {
        const Qt::InputMethodHints hints = edit->inputMethodHints();
        return edit->validator() != nullptr
            || hints.testFlag(Qt::ImhDigitsOnly)
            || hints.testFlag(Qt::ImhFormattedNumbersOnly);
    }

    if (const QComboBox* combo = qobject_cast<QComboBox*>(widget))
    {
        if (!combo->isEditable() || combo->lineEdit() == nullptr)
        {
            return false;
        }
        return IsNumericInputWidget(combo->lineEdit());
    }

    return false;
}

bool TouchKeyboardManager::HasTouchCapability() const
{
#ifdef Q_OS_WIN
#ifndef NID_INTEGRATED_TOUCH
#define NID_INTEGRATED_TOUCH 0x01
#endif
#ifndef NID_EXTERNAL_TOUCH
#define NID_EXTERNAL_TOUCH 0x02
#endif
    const int digitizer = GetSystemMetrics(SM_DIGITIZER);
    return (digitizer & (NID_INTEGRATED_TOUCH | NID_EXTERNAL_TOUCH)) != 0;
#else
    return false;
#endif
}

bool TouchKeyboardManager::HasHardwareKeyboard() const
{
#ifdef Q_OS_WIN
    UINT deviceCount = 0;
    if (GetRawInputDeviceList(nullptr, &deviceCount, sizeof(RAWINPUTDEVICELIST)) == 0 && deviceCount > 0)
    {
        QVector<RAWINPUTDEVICELIST> devices(static_cast<int>(deviceCount));
        if (GetRawInputDeviceList(devices.data(), &deviceCount, sizeof(RAWINPUTDEVICELIST)) != static_cast<UINT>(-1))
        {
            for (UINT index = 0; index < deviceCount; ++index)
            {
                if (devices[static_cast<int>(index)].dwType == RIM_TYPEKEYBOARD)
                {
                    return true;
                }
            }
        }
    }

    return GetKeyboardType(0) != 0;
#else
    return true;
#endif
}
