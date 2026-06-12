#include "WindowStyleHelper.h"

#include <algorithm>
#include <memory>

#include <QApplication>
#include <QAbstractButton>
#include <QAbstractItemView>
#include <QAbstractSpinBox>
#include <QDialog>
#include <QEvent>
#include <QEventLoop>
#include <QFormLayout>
#include <QFont>
#include <QFontDatabase>
#include <QFrame>
#include <QGuiApplication>
#include <QIcon>
#include <QComboBox>
#include <QElapsedTimer>
#include <QLineEdit>
#include <QLabel>
#include <QLayout>
#include <QMessageBox>
#include <QPointer>
#include <QProgressBar>
#include <QPushButton>
#include <QScrollArea>
#include <QScreen>
#include <QSizePolicy>
#include <QString>
#include <QTimer>
#include <QVariant>
#include <QVBoxLayout>
#include <QWidget>
#include <QWindow>

#ifdef Q_OS_WIN
#include <windows.h>
#endif

namespace
{
const char kAdaptiveScrollAreaProperty[] = "_adaptive_window_scroll_area";
const char kAdaptiveScrollContentProperty[] = "_adaptive_window_scroll_content";
const char kAdaptiveScrollConfiguredProperty[] = "_adaptive_scroll_configured";
const char kResponsiveDefaultsAppliedProperty[] = "_responsive_defaults_applied";

QRect ResolveAvailableGeometry(QWidget* widget);

void CenterWindowOnOwner(QWidget* window, QWidget* owner)
{
    if (window == nullptr)
    {
        return;
    }

    QRect baseRect;
    if (owner != nullptr)
    {
        baseRect = owner->frameGeometry();
    }
    if (!baseRect.isValid())
    {
        baseRect = ResolveAvailableGeometry(owner != nullptr ? owner : window);
    }
    if (!baseRect.isValid())
    {
        return;
    }

    const QSize size = window->sizeHint().expandedTo(window->minimumSize());
    const int x = baseRect.left() + (baseRect.width() - size.width()) / 2;
    const int y = baseRect.top() + (baseRect.height() - size.height()) / 2;
    window->resize(size);
    window->move(x, y);
}

}

class DelayedLoadingGuardPrivate final : public QObject
{
public:
    DelayedLoadingGuardPrivate(QWidget* ownerWidget, const QString& initialText, int delay)
        : QObject(ownerWidget != nullptr ? static_cast<QObject*>(ownerWidget) : static_cast<QObject*>(qApp))
        , owner(ownerWidget)
        , text(initialText)
        , delayMs((std::max)(0, delay))
    {
        if (qApp != nullptr)
        {
            qApp->installEventFilter(this);
        }
        elapsed.start();
        delayTimer.setSingleShot(true);
        delayTimer.setInterval(delayMs);
        connect(&delayTimer, &QTimer::timeout, this, [this]() { ShowIfNeeded(true); });
        delayTimer.start();
    }

    ~DelayedLoadingGuardPrivate() override
    {
        if (qApp != nullptr)
        {
            qApp->removeEventFilter(this);
        }
        Finish();
    }

    void SetText(const QString& newText)
    {
        text = newText;
        if (label != nullptr)
        {
            label->setText(BuildAnimatedText());
        }
    }

    void Pulse()
    {
        ShowIfNeeded(false);
        if (dialog != nullptr && dialog->isVisible() && qApp != nullptr)
        {
            qApp->processEvents(QEventLoop::ExcludeUserInputEvents, 30);
        }
    }

    void Finish()
    {
        finished = true;
        delayTimer.stop();
        animationTimer.stop();
        if (dialog != nullptr)
        {
            dialog->hide();
            dialog->deleteLater();
            dialog = nullptr;
            label = nullptr;
        }
    }

protected:
    bool eventFilter(QObject* watched, QEvent* event) override
    {
        if (!finished
            && event != nullptr
            && (event->type() == QEvent::Show
                || event->type() == QEvent::ShowToParent
                || event->type() == QEvent::WindowActivate))
        {
            QWidget* widget = qobject_cast<QWidget*>(watched);
            if (IsForeignBlockingDialog(widget))
            {
                Finish();
            }
        }
        return QObject::eventFilter(watched, event);
    }

private:
    QString BuildAnimatedText() const
    {
        const QString base = text.trimmed().isEmpty() ? QStringLiteral("正在加载界面") : text.trimmed();
        return QString("%1%2").arg(base, QString((animationTick % 4), QLatin1Char('.')));
    }

    bool IsForeignBlockingDialog(QWidget* widget) const
    {
        if (widget == nullptr || widget == dialog.data())
        {
            return false;
        }
        if (qobject_cast<QMessageBox*>(widget) != nullptr)
        {
            return true;
        }
        const QDialog* modalDialog = qobject_cast<QDialog*>(widget);
        return modalDialog != nullptr && modalDialog->isModal();
    }

    bool HasForeignBlockingDialog() const
    {
        if (qApp == nullptr)
        {
            return false;
        }
        const QWidgetList widgets = QApplication::topLevelWidgets();
        for (QWidget* widget : widgets)
        {
            if (widget != nullptr && widget->isVisible() && IsForeignBlockingDialog(widget))
            {
                return true;
            }
        }
        return false;
    }

    void ShowIfNeeded(bool fromTimer)
    {
        if (finished || dialog != nullptr)
        {
            return;
        }
        if (!fromTimer && elapsed.elapsed() < delayMs)
        {
            return;
        }
        if (HasForeignBlockingDialog())
        {
            Finish();
            return;
        }
        CreateLoadingDialog();
    }

    void CreateLoadingDialog()
    {
        QWidget* ownerWidget = owner.data();
        QDialog* loading = new QDialog(ownerWidget);
        loading->setWindowTitle(QStringLiteral("加载中"));
        loading->setWindowModality(Qt::ApplicationModal);
        loading->setWindowFlags(Qt::Dialog | Qt::FramelessWindowHint | Qt::WindowStaysOnTopHint);
        loading->setAttribute(Qt::WA_DeleteOnClose, false);
        loading->setStyleSheet(
            "QDialog { background: #111820; color: #ECF3F4; border: 1px solid #3C6173; border-radius: 10px; }"
            "QLabel { color: #F4FAFA; font-size: 16px; font-weight: 600; }"
            "QProgressBar { background: #081018; border: 1px solid #2C4653; border-radius: 8px; min-height: 16px; }"
            "QProgressBar::chunk { background: #2D8DA0; border-radius: 7px; }");

        QVBoxLayout* layout = new QVBoxLayout(loading);
        layout->setContentsMargins(26, 22, 26, 22);
        layout->setSpacing(14);
        label = new QLabel(BuildAnimatedText(), loading);
        label->setAlignment(Qt::AlignCenter);
        QProgressBar* progress = new QProgressBar(loading);
        progress->setRange(0, 0);
        progress->setTextVisible(false);
        progress->setMinimumWidth(260);
        layout->addWidget(label);
        layout->addWidget(progress);

        dialog = loading;
        CenterWindowOnOwner(loading, ownerWidget);
        loading->show();
        loading->raise();
        loading->activateWindow();

        animationTimer.setInterval(300);
        connect(&animationTimer, &QTimer::timeout, this, [this]()
            {
                ++animationTick;
                if (label != nullptr)
                {
                    label->setText(BuildAnimatedText());
                }
            });
        animationTimer.start();
        if (qApp != nullptr)
        {
            qApp->processEvents(QEventLoop::ExcludeUserInputEvents, 30);
        }
    }

    QPointer<QWidget> owner;
    QPointer<QDialog> dialog;
    QPointer<QLabel> label;
    QTimer delayTimer;
    QTimer animationTimer;
    QElapsedTimer elapsed;
    QString text;
    int delayMs = 1000;
    int animationTick = 0;
    bool finished = false;
};

namespace
{
QRect ResolveAvailableGeometry(QWidget* widget)
{
    if (widget == nullptr)
    {
        return QRect();
    }

    if (QScreen* screen = widget->screen())
    {
        return screen->availableGeometry();
    }
    if (widget->windowHandle() != nullptr && widget->windowHandle()->screen() != nullptr)
    {
        return widget->windowHandle()->screen()->availableGeometry();
    }
    if (QScreen* primary = QGuiApplication::primaryScreen())
    {
        return primary->availableGeometry();
    }
    return QRect();
}

QString ResolveChineseCapableFontFamily()
{
    const QStringList families = QFontDatabase::families();
    const QStringList candidates = {
        QStringLiteral("Microsoft YaHei UI"),
        QStringLiteral("Microsoft YaHei"),
        QStringLiteral("微软雅黑"),
        QStringLiteral("SimHei"),
        QStringLiteral("黑体"),
        QStringLiteral("SimSun"),
        QStringLiteral("宋体"),
        QStringLiteral("Arial Unicode MS")
    };

    for (const QString& candidate : candidates)
    {
        if (families.contains(candidate, Qt::CaseInsensitive))
        {
            return candidate;
        }
    }
    return QApplication::font().family();
}

QFont ChineseCapableFont(const QFont& baseFont = QApplication::font())
{
    QFont font(baseFont);
    const QString family = ResolveChineseCapableFontFamily();
    if (!family.trimmed().isEmpty())
    {
        font.setFamily(family);
    }
    return font;
}

void ApplyChineseCapableFont(QWidget* widget)
{
    if (widget == nullptr)
    {
        return;
    }
    widget->setFont(ChineseCapableFont(widget->font()));
}

QRect ResolveUsableGeometry(QWidget* widget)
{
    return ResolveAvailableGeometry(widget);
}

QPoint ResolveCenteredPosition(const QRect& available, const QRect& frame)
{
    if (!available.isValid() || !frame.isValid())
    {
        return QPoint();
    }

    const int centeredX = available.left() + (available.width() - frame.width()) / 2;
    const int centeredY = available.top() + (available.height() - frame.height()) / 2;
    return QPoint((std::max)(available.left(), centeredX), (std::max)(available.top(), centeredY));
}

void ApplyWindowsTitleBarTheme(QWidget* widget)
{
    if (widget == nullptr)
    {
        return;
    }

#ifdef Q_OS_WIN
    using DwmSetWindowAttributeProc = HRESULT(WINAPI*)(HWND, DWORD, LPCVOID, DWORD);
    HMODULE dwmapi = LoadLibraryW(L"dwmapi.dll");
    if (dwmapi == nullptr)
    {
        return;
    }

    auto setAttribute = reinterpret_cast<DwmSetWindowAttributeProc>(GetProcAddress(dwmapi, "DwmSetWindowAttribute"));
    if (setAttribute != nullptr)
    {
        HWND hwnd = reinterpret_cast<HWND>(widget->winId());
        const BOOL darkMode = TRUE;
        setAttribute(hwnd, 20, &darkMode, sizeof(darkMode));
        setAttribute(hwnd, 19, &darkMode, sizeof(darkMode));

        const COLORREF captionColor = RGB(6, 14, 22);
        const COLORREF textColor = RGB(236, 243, 244);
        const COLORREF borderColor = RGB(36, 62, 76);
        setAttribute(hwnd, 35, &captionColor, sizeof(captionColor));
        setAttribute(hwnd, 36, &textColor, sizeof(textColor));
        setAttribute(hwnd, 34, &borderColor, sizeof(borderColor));
    }

    FreeLibrary(dwmapi);
#endif
}

int ClampedControlWidth(int value, int minimum, int maximum)
{
    return (std::min)((std::max)(value, minimum), maximum);
}

int CompactTextWidth(QWidget* widget, const QString& text, int horizontalPadding)
{
    if (widget == nullptr)
    {
        return horizontalPadding;
    }
    const QString normalized = text.trimmed().replace('\n', ' ');
    if (normalized.isEmpty())
    {
        return horizontalPadding;
    }
    return widget->fontMetrics().horizontalAdvance(normalized) + horizontalPadding;
}

bool HasKeepWideProperty(QWidget* widget)
{
    return widget != nullptr && widget->property("_keep_wide_control").toBool();
}

bool IsInsideAdaptiveScrollContent(QWidget* widget)
{
    for (QWidget* current = widget; current != nullptr; current = current->parentWidget())
    {
        if (current->property(kAdaptiveScrollContentProperty).toBool())
        {
            return true;
        }
    }
    return false;
}

void LockAdaptiveScrollContentSize(QScrollArea* scrollArea)
{
    if (scrollArea == nullptr)
    {
        return;
    }

    QWidget* content = scrollArea->widget();
    if (content == nullptr)
    {
        return;
    }

    if (QLayout* layout = content->layout())
    {
        layout->setSizeConstraint(QLayout::SetMinimumSize);
    }

    const QSize contentHint = content->sizeHint().expandedTo(content->minimumSizeHint());
    if (contentHint.isValid())
    {
        content->setMinimumSize(contentHint);
    }
}

bool HasCompactMaximum(QWidget* widget)
{
    return widget != nullptr && widget->maximumWidth() > 0 && widget->maximumWidth() < 1000;
}

void SetCompactMaximumWidth(QWidget* widget, int targetWidth)
{
    if (widget == nullptr || HasKeepWideProperty(widget) || IsInsideAdaptiveScrollContent(widget))
    {
        return;
    }

    const int finalWidth = (std::max)(targetWidth, widget->minimumWidth());
    if (!HasCompactMaximum(widget) || widget->maximumWidth() > finalWidth)
    {
        widget->setMaximumWidth(finalWidth);
    }

    QSizePolicy policy = widget->sizePolicy();
    if (policy.horizontalPolicy() == QSizePolicy::Expanding
        || policy.horizontalPolicy() == QSizePolicy::MinimumExpanding)
    {
        policy.setHorizontalPolicy(QSizePolicy::Preferred);
        widget->setSizePolicy(policy);
    }
}

bool TextLooksLikeLongContent(const QString& text)
{
    const QString normalized = text.trimmed();
    return normalized.contains('\\')
        || normalized.contains('/')
        || normalized.contains(':')
        || normalized.contains("路径")
        || normalized.contains("目录")
        || normalized.contains("文件")
        || normalized.contains("地址")
        || normalized.contains("URL", Qt::CaseInsensitive)
        || normalized.contains("IP", Qt::CaseInsensitive);
}

void CompactLineEdit(QLineEdit* edit)
{
    if (edit == nullptr || HasKeepWideProperty(edit))
    {
        return;
    }

    const QString sampleText = !edit->text().trimmed().isEmpty()
        ? edit->text().trimmed()
        : edit->placeholderText().trimmed();
    const bool longContent = TextLooksLikeLongContent(sampleText) || sampleText.size() > 24;
    const int minWidth = edit->echoMode() == QLineEdit::Password ? 220 : 150;
    const int maxWidth = longContent ? 420 : 280;
    const int fallback = edit->echoMode() == QLineEdit::Password ? 260 : 220;
    const int measured = CompactTextWidth(edit, sampleText, 48);
    SetCompactMaximumWidth(edit, ClampedControlWidth((std::max)(fallback, measured), minWidth, maxWidth));
}

void CompactComboBox(QComboBox* combo)
{
    if (combo == nullptr || HasKeepWideProperty(combo))
    {
        return;
    }
    ApplyChineseCapableFont(combo);
    if (QAbstractItemView* view = combo->view())
    {
        ApplyChineseCapableFont(view);
    }

    int maxTextWidth = CompactTextWidth(combo, combo->currentText(), 58);
    for (int i = 0; i < combo->count(); ++i)
    {
        maxTextWidth = (std::max)(maxTextWidth, CompactTextWidth(combo, combo->itemText(i), 58));
    }
    const bool longContent = maxTextWidth > 280;
    SetCompactMaximumWidth(combo, ClampedControlWidth(maxTextWidth, 150, longContent ? 380 : 300));
}

void CompactSpinBox(QAbstractSpinBox* spinBox)
{
    if (spinBox == nullptr || HasKeepWideProperty(spinBox))
    {
        return;
    }
    SetCompactMaximumWidth(spinBox, 170);
}

void CompactButton(QPushButton* button)
{
    if (button == nullptr || HasKeepWideProperty(button))
    {
        return;
    }
    if (button->text().contains('\n') || button->minimumHeight() >= 46)
    {
        return;
    }

    const int measured = CompactTextWidth(button, button->text(), 54);
    SetCompactMaximumWidth(button, ClampedControlWidth(measured, 96, 240));
}

void CompactFormLayouts(QWidget* root)
{
    if (root == nullptr)
    {
        return;
    }
    const QList<QFormLayout*> formLayouts = root->findChildren<QFormLayout*>();
    for (QFormLayout* formLayout : formLayouts)
    {
        if (formLayout != nullptr)
        {
            formLayout->setFieldGrowthPolicy(QFormLayout::FieldsStayAtSizeHint);
        }
    }
}

void ClampWindowToAvailableGeometry(QWidget* widget)
{
    if (widget == nullptr)
    {
        return;
    }
    if (!widget->isWindow())
    {
        return;
    }
    if (widget->isMaximized() || widget->isFullScreen())
    {
        return;
    }
    if (widget->property("_clamping_window_geometry").toBool())
    {
        return;
    }

    const QRect available = ResolveUsableGeometry(widget);
    if (!available.isValid())
    {
        return;
    }

    const QSize currentSize = widget->size();
    const QSize availableSize = available.size();
    const bool isDialog = qobject_cast<QDialog*>(widget) != nullptr;
    const double widthRatio = isDialog ? 0.88 : 0.94;
    const double heightRatio = isDialog ? 0.86 : 0.92;
    const int widthLimit = (std::min)(
        availableSize.width(),
        (std::max)(widget->minimumWidth(), static_cast<int>(availableSize.width() * widthRatio)));
    const int heightLimit = (std::min)(
        availableSize.height(),
        (std::max)(widget->minimumHeight(), static_cast<int>(availableSize.height() * heightRatio)));

    const QSize preferredSize = widget->property("_preferred_window_size").toSize();
    const bool hasPreferredSize = preferredSize.isValid()
        && preferredSize.width() > 0
        && preferredSize.height() > 0;
    const int sourceWidth = hasPreferredSize ? preferredSize.width() : currentSize.width();
    const int sourceHeight = hasPreferredSize ? preferredSize.height() : currentSize.height();
    const int targetWidth = (std::min)((std::max)(sourceWidth, widget->minimumWidth()), widthLimit);
    const int targetHeight = (std::min)((std::max)(sourceHeight, widget->minimumHeight()), heightLimit);
    QRect currentFrame = widget->frameGeometry();
    QPoint targetPos = ResolveCenteredPosition(available, currentFrame);

    widget->setProperty("_clamping_window_geometry", true);
    if (targetWidth != currentSize.width() || targetHeight != currentSize.height())
    {
        widget->resize(targetWidth, targetHeight);
        currentFrame = widget->frameGeometry();
        targetPos = ResolveCenteredPosition(available, currentFrame);
    }

    if (currentFrame.left() < available.left())
    {
        targetPos.setX(available.left());
    }
    else if (currentFrame.right() > available.right())
    {
        targetPos.setX((std::max)(available.left(), available.right() - currentFrame.width() + 1));
    }

    if (currentFrame.top() < available.top())
    {
        targetPos.setY(available.top());
    }
    else if (currentFrame.bottom() > available.bottom())
    {
        targetPos.setY((std::max)(available.top(), available.bottom() - currentFrame.height() + 1));
    }

    if (targetPos != widget->pos())
    {
        widget->move(targetPos);
    }
    widget->setProperty("_clamping_window_geometry", false);
}

QString AdaptiveScrollAreaStyleSheet()
{
    return QStringLiteral(
        "QScrollArea { background: transparent; border: none; }"
        "QScrollArea > QWidget > QWidget { background: transparent; }"
        "QScrollBar:vertical { background: #0B1117; width: 12px; margin: 0px; }"
        "QScrollBar:vertical:disabled { background: transparent; }"
        "QScrollBar::handle:vertical { background: #385366; border-radius: 6px; min-height: 28px; }"
        "QScrollBar::handle:vertical:disabled { background: transparent; }"
        "QScrollBar::handle:vertical:hover { background: #4D7488; }"
        "QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0px; }"
        "QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical { background: transparent; }"
        "QScrollBar::add-page:vertical:disabled, QScrollBar::sub-page:vertical:disabled { background: transparent; }"
        "QScrollBar:horizontal { background: #0B1117; height: 12px; margin: 0px; }"
        "QScrollBar::handle:horizontal { background: #385366; border-radius: 6px; min-width: 28px; }"
        "QScrollBar::handle:horizontal:hover { background: #4D7488; }"
        "QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal { width: 0px; }"
        "QScrollBar::add-page:horizontal, QScrollBar::sub-page:horizontal { background: transparent; }");
}

void ConfigureAdaptiveScrollArea(QScrollArea* scrollArea)
{
    if (scrollArea == nullptr)
    {
        return;
    }

    const bool configured = scrollArea->property(kAdaptiveScrollConfiguredProperty).toBool();
    if (!configured)
    {
        scrollArea->setProperty(kAdaptiveScrollAreaProperty, true);
        scrollArea->setWidgetResizable(true);
        scrollArea->setAlignment(Qt::AlignLeft | Qt::AlignTop);
        scrollArea->setFrameShape(QFrame::NoFrame);
        scrollArea->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        scrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
        scrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
        scrollArea->viewport()->setAutoFillBackground(false);
        scrollArea->setStyleSheet(AdaptiveScrollAreaStyleSheet());
        scrollArea->setProperty(kAdaptiveScrollConfiguredProperty, true);
    }

    if (QWidget* content = scrollArea->widget())
    {
        content->setProperty(kAdaptiveScrollContentProperty, true);
        content->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
        LockAdaptiveScrollContentSize(scrollArea);
        QTimer::singleShot(0, scrollArea, [scrollArea]() { LockAdaptiveScrollContentSize(scrollArea); });
        QTimer::singleShot(120, scrollArea, [scrollArea]() { LockAdaptiveScrollContentSize(scrollArea); });
    }
}

void ConfigureExistingScrollAreas(QWidget* widget)
{
    if (widget == nullptr)
    {
        return;
    }

    const QList<QScrollArea*> scrollAreas = widget->findChildren<QScrollArea*>();
    for (QScrollArea* scrollArea : scrollAreas)
    {
        ConfigureAdaptiveScrollArea(scrollArea);
    }
}

class WindowGeometryGuard final : public QObject
{
public:
    static WindowGeometryGuard* Instance()
    {
        static WindowGeometryGuard* instance = new WindowGeometryGuard(qApp);
        return instance;
    }

protected:
    bool eventFilter(QObject* watched, QEvent* event) override
    {
        QWidget* widget = qobject_cast<QWidget*>(watched);
        if (widget == nullptr)
        {
            return QObject::eventFilter(watched, event);
        }
        if (!widget->isWindow())
        {
            return QObject::eventFilter(watched, event);
        }

        switch (event->type())
        {
        case QEvent::Show:
            ClampWindowToAvailableGeometry(widget);
            ConfigureExistingScrollAreas(widget);
            QTimer::singleShot(60, widget, [widget]() {
                ClampWindowToAvailableGeometry(widget);
                ConfigureExistingScrollAreas(widget);
            });
            QTimer::singleShot(160, widget, [widget]() {
                ClampWindowToAvailableGeometry(widget);
                ConfigureExistingScrollAreas(widget);
            });
            break;
        case QEvent::Resize:
            break;
        case QEvent::WindowStateChange:
            ClampWindowToAvailableGeometry(widget);
            ConfigureExistingScrollAreas(widget);
            break;
        default:
            break;
        }

        return QObject::eventFilter(watched, event);
    }

private:
    explicit WindowGeometryGuard(QObject* parent)
        : QObject(parent)
    {
    }
};
}

DelayedLoadingGuard::DelayedLoadingGuard(QWidget* owner, const QString& text, int delayMs)
    : d(std::make_unique<DelayedLoadingGuardPrivate>(owner, text, delayMs))
{
}

DelayedLoadingGuard::~DelayedLoadingGuard() = default;

void DelayedLoadingGuard::SetText(const QString& text)
{
    if (d)
    {
        d->SetText(text);
    }
}

void DelayedLoadingGuard::Pulse()
{
    if (d)
    {
        d->Pulse();
    }
}

void DelayedLoadingGuard::Finish()
{
    if (d)
    {
        d->Finish();
    }
}

void ApplyUnifiedWindowChrome(QWidget* widget)
{
    if (widget == nullptr)
    {
        return;
    }

    widget->setWindowFlag(Qt::WindowMinimizeButtonHint, true);
    widget->setWindowFlag(Qt::WindowMaximizeButtonHint, true);
    widget->setWindowFlag(Qt::WindowCloseButtonHint, true);
    widget->setWindowFlag(Qt::WindowSystemMenuHint, true);
    widget->setWindowFlag(Qt::MSWindowsFixedSizeDialogHint, false);
    if (widget->isWindow())
    {
        widget->setMinimumSize(0, 0);
        widget->setMaximumSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX);
    }

    if (QDialog* dialog = qobject_cast<QDialog*>(widget))
    {
        dialog->setSizeGripEnabled(true);
        QTimer::singleShot(0, dialog, [dialog]() { ApplyResponsivePageDefaults(dialog); });
        QTimer::singleShot(120, dialog, [dialog]() { ApplyResponsivePageDefaults(dialog); });
    }

    widget->installEventFilter(WindowGeometryGuard::Instance());
    QTimer::singleShot(0, widget, [widget]() {
        ApplyResponsivePageDefaults(widget);
        ClampWindowToAvailableGeometry(widget);
    });

    widget->setWindowIcon(QIcon(":/QtWidgetsApplication4/icons/minimal_robot_icon_blue_black.svg"));
    RefreshUnifiedWindowTitleBar(widget);
}

void ConfigureApplicationFontFallback()
{
    if (qApp == nullptr)
    {
        return;
    }

    QFont font = ChineseCapableFont(qApp->font());
    if (font.pointSize() <= 0)
    {
        font.setPointSize(10);
    }
    qApp->setFont(font);
}

void ConfigureResponsiveScrollArea(QScrollArea* scrollArea)
{
    ConfigureAdaptiveScrollArea(scrollArea);
}

void ApplyEditorOnlySpinBoxes(QWidget* widget)
{
    if (widget == nullptr)
    {
        return;
    }

    const QList<QAbstractSpinBox*> spinBoxes = widget->findChildren<QAbstractSpinBox*>();
    for (QAbstractSpinBox* spinBox : spinBoxes)
    {
        if (spinBox != nullptr)
        {
            spinBox->setButtonSymbols(QAbstractSpinBox::NoButtons);
        }
    }
}

void ApplyResponsivePageDefaults(QWidget* widget)
{
    if (widget == nullptr)
    {
        return;
    }
    if (widget->property(kResponsiveDefaultsAppliedProperty).toBool())
    {
        ConfigureExistingScrollAreas(widget);
        return;
    }

    ConfigureExistingScrollAreas(widget);
    ApplyEditorOnlySpinBoxes(widget);
    ApplyCompactControlWidths(widget);
    widget->setProperty(kResponsiveDefaultsAppliedProperty, true);
}

void RefreshUnifiedWindowTitleBar(QWidget* widget)
{
    if (widget == nullptr)
    {
        return;
    }

    widget->setWindowIcon(QIcon(":/QtWidgetsApplication4/icons/minimal_robot_icon_blue_black.svg"));
    ApplyWindowsTitleBarTheme(widget);
}

QString UnifiedComboBoxStyleSheet()
{
    return QStringLiteral(
        "QComboBox { background: #000000; color: #F5FAFA; border: 1px solid #385366; border-radius: 0px; padding: 4px 34px 4px 8px; min-height: 24px; font-family: 'Microsoft YaHei UI', 'Microsoft YaHei', 'SimSun', 'Arial Unicode MS'; }"
        "QComboBox::drop-down { subcontrol-origin: padding; subcontrol-position: top right; width: 28px; border-left: 1px solid #385366; border-radius: 0px; background: #000000; }"
        "QComboBox::down-arrow { image: url(:/QtWidgetsApplication4/icons/chevron-down.svg); width: 12px; height: 8px; }"
        "QComboBox QAbstractItemView { background: #000000; color: #F4FAFA; selection-background-color: #1F4F5C; border: 1px solid #385366; border-radius: 0px; outline: 0px; font-family: 'Microsoft YaHei UI', 'Microsoft YaHei', 'SimSun', 'Arial Unicode MS'; }");
}

void ApplyCompactControlWidths(QWidget* widget)
{
    if (widget == nullptr)
    {
        return;
    }

    CompactFormLayouts(widget);

    const QList<QLineEdit*> lineEdits = widget->findChildren<QLineEdit*>();
    for (QLineEdit* edit : lineEdits)
    {
        CompactLineEdit(edit);
    }

    const QList<QComboBox*> comboBoxes = widget->findChildren<QComboBox*>();
    for (QComboBox* combo : comboBoxes)
    {
        CompactComboBox(combo);
    }

    const QList<QAbstractSpinBox*> spinBoxes = widget->findChildren<QAbstractSpinBox*>();
    for (QAbstractSpinBox* spinBox : spinBoxes)
    {
        CompactSpinBox(spinBox);
    }

    const QList<QPushButton*> buttons = widget->findChildren<QPushButton*>();
    for (QPushButton* button : buttons)
    {
        CompactButton(button);
    }
}

bool ConfirmCloseWithUnsavedChanges(
    QWidget* widget,
    const QString& title,
    const std::function<bool()>& saveCallback)
{
    QMessageBox messageBox(widget);
    messageBox.setWindowTitle(title);
    messageBox.setIcon(QMessageBox::Question);
    messageBox.setText("当前界面有未保存的修改，是否保存后关闭？");
    QPushButton* saveButton = messageBox.addButton("保存", QMessageBox::AcceptRole);
    QPushButton* discardButton = messageBox.addButton("不保存", QMessageBox::DestructiveRole);
    messageBox.addButton("取消", QMessageBox::RejectRole);
    messageBox.setDefaultButton(saveButton);
    messageBox.exec();

    QAbstractButton* clickedButton = messageBox.clickedButton();
    if (clickedButton == saveButton)
    {
        return saveCallback ? saveCallback() : true;
    }
    if (clickedButton == discardButton)
    {
        return true;
    }
    return false;
}

void ResizeWindowForAvailableGeometry(
    QWidget* widget,
    const QSize& preferredSize,
    double maxWidthRatio,
    double maxHeightRatio)
{
    if (widget == nullptr)
    {
        return;
    }

    const QRect available = ResolveUsableGeometry(widget);
    if (!available.isValid())
    {
        widget->resize(preferredSize);
        return;
    }

    const double safeWidthRatio = maxWidthRatio > 0.0 && maxWidthRatio <= 1.0 ? maxWidthRatio : 0.88;
    const double safeHeightRatio = maxHeightRatio > 0.0 && maxHeightRatio <= 1.0 ? maxHeightRatio : 0.88;

    const int maxWidth = (std::max)(widget->minimumWidth(), static_cast<int>(available.width() * safeWidthRatio));
    const int maxHeight = (std::max)(widget->minimumHeight(), static_cast<int>(available.height() * safeHeightRatio));

    const int finalWidth = (std::min)(preferredSize.width(), maxWidth);
    const int finalHeight = (std::min)(preferredSize.height(), maxHeight);
    widget->setProperty("_preferred_window_size", QSize(finalWidth, finalHeight));
    widget->resize(finalWidth, finalHeight);

    const QRect frame = widget->frameGeometry();
    widget->move(ResolveCenteredPosition(available, frame));
}

namespace
{
// 应用级滚轮拦截：数字框/下拉框收到的滚轮事件直接吞掉，避免滚动页面时悬停误改参数。
// 下拉框弹出的列表是独立控件，不受影响。
class WheelEventGuard final : public QObject
{
public:
    using QObject::QObject;

protected:
    bool eventFilter(QObject* watched, QEvent* event) override
    {
        if (event->type() == QEvent::Wheel)
        {
            QWidget* widget = qobject_cast<QWidget*>(watched);
            if (widget != nullptr
                && (qobject_cast<QAbstractSpinBox*>(widget) != nullptr
                    || qobject_cast<QComboBox*>(widget) != nullptr))
            {
                event->ignore();
                return true;
            }
        }
        return QObject::eventFilter(watched, event);
    }
};
}

void InstallGlobalWheelGuard(QApplication& app)
{
    app.installEventFilter(new WheelEventGuard(&app));
}
