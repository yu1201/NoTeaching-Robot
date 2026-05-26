#include "WindowStyleHelper.h"

#include <algorithm>

#include <QApplication>
#include <QAbstractButton>
#include <QAbstractItemView>
#include <QAbstractSpinBox>
#include <QBoxLayout>
#include <QDialog>
#include <QEvent>
#include <QFormLayout>
#include <QFont>
#include <QFontDatabase>
#include <QFrame>
#include <QGuiApplication>
#include <QGridLayout>
#include <QIcon>
#include <QComboBox>
#include <QLineEdit>
#include <QLayout>
#include <QMainWindow>
#include <QMessageBox>
#include <QPushButton>
#include <QScrollArea>
#include <QScrollBar>
#include <QScreen>
#include <QSizePolicy>
#include <QString>
#include <QTimer>
#include <QVariant>
#include <QWidget>
#include <QWindow>

#ifdef Q_OS_WIN
#include <windows.h>
#endif

namespace
{
const char kAdaptiveScrollInstalledProperty[] = "_adaptive_scroll_installed";
const char kAdaptiveScrollAreaProperty[] = "_adaptive_window_scroll_area";
const char kAdaptiveScrollContentProperty[] = "_adaptive_window_scroll_content";
const char kAdaptiveScrollAutoWrappedProperty[] = "_adaptive_window_auto_wrapped";
const char kAdaptiveScrollCheckPendingProperty[] = "_adaptive_scroll_check_pending";
const char kDirectMouseInputWindowProperty[] = "_direct_mouse_input_window";

void EnsureAdaptiveScrollSupport(QWidget* widget);

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

bool HasCompactMaximum(QWidget* widget)
{
    return widget != nullptr && widget->maximumWidth() > 0 && widget->maximumWidth() < 1000;
}

void SetCompactMaximumWidth(QWidget* widget, int targetWidth)
{
    if (widget == nullptr || HasKeepWideProperty(widget))
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
        "QScrollArea#AdaptiveWindowScrollArea { background: transparent; border: none; }"
        "QScrollArea#AdaptiveWindowScrollArea > QWidget > QWidget { background: transparent; }"
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

    scrollArea->setProperty(kAdaptiveScrollAreaProperty, true);
    scrollArea->setWidgetResizable(true);
    scrollArea->setAlignment(Qt::AlignLeft | Qt::AlignTop);
    scrollArea->setFrameShape(QFrame::NoFrame);
    scrollArea->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    if (scrollArea->horizontalScrollBarPolicy() != Qt::ScrollBarAlwaysOff)
    {
        scrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    }
    // Reserve vertical scrollbar space up front. Pages with collapsible/dropdown
    // sections can otherwise shrink horizontally when overflow first appears.
    scrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
    scrollArea->viewport()->setAutoFillBackground(false);
    scrollArea->setStyleSheet(AdaptiveScrollAreaStyleSheet());
    if (QWidget* content = scrollArea->widget())
    {
        content->setProperty(kAdaptiveScrollContentProperty, true);
        content->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
    }
}

void RelaxWindowMinimumSizeForAdaptiveScroll(QWidget* widget)
{
    if (widget == nullptr || !widget->isWindow())
    {
        return;
    }

    const int relaxedWidth = widget->minimumWidth() > 0
        ? (std::min)(widget->minimumWidth(), 480)
        : 0;
    const int relaxedHeight = widget->minimumHeight() > 0
        ? (std::min)(widget->minimumHeight(), 320)
        : 0;
    widget->setMinimumSize(relaxedWidth, relaxedHeight);
}

QBoxLayout* CreateMirroredBoxLayout(QLayout* sourceLayout, QWidget* parent)
{
    QBoxLayout::Direction direction = QBoxLayout::TopToBottom;
    if (QBoxLayout* sourceBox = qobject_cast<QBoxLayout*>(sourceLayout))
    {
        direction = sourceBox->direction();
    }

    QBoxLayout* contentLayout = new QBoxLayout(direction, parent);
    QMargins margins = sourceLayout != nullptr ? sourceLayout->contentsMargins() : QMargins();
    contentLayout->setContentsMargins(margins);
    contentLayout->setSpacing(sourceLayout != nullptr ? sourceLayout->spacing() : -1);
    return contentLayout;
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

bool LayoutHasSingleScrollArea(QLayout* layout, QScrollArea** outScrollArea)
{
    if (outScrollArea != nullptr)
    {
        *outScrollArea = nullptr;
    }
    if (layout == nullptr || layout->count() != 1)
    {
        return false;
    }

    QLayoutItem* item = layout->itemAt(0);
    QScrollArea* scrollArea = item != nullptr ? qobject_cast<QScrollArea*>(item->widget()) : nullptr;
    if (scrollArea == nullptr)
    {
        return false;
    }
    if (outScrollArea != nullptr)
    {
        *outScrollArea = scrollArea;
    }
    return true;
}

bool LayoutHasSingleAutoWrappedScrollArea(QLayout* layout, QScrollArea** outScrollArea)
{
    QScrollArea* scrollArea = nullptr;
    if (!LayoutHasSingleScrollArea(layout, &scrollArea))
    {
        if (outScrollArea != nullptr)
        {
            *outScrollArea = nullptr;
        }
        return false;
    }
    if (scrollArea == nullptr || !scrollArea->property(kAdaptiveScrollAutoWrappedProperty).toBool())
    {
        if (outScrollArea != nullptr)
        {
            *outScrollArea = nullptr;
        }
        return false;
    }
    if (outScrollArea != nullptr)
    {
        *outScrollArea = scrollArea;
    }
    return true;
}

QSize RequiredLayoutSize(QLayout* layout)
{
    if (layout == nullptr)
    {
        return QSize();
    }

    const QSize minimum = layout->minimumSize();
    if (minimum.isValid() && !minimum.isEmpty())
    {
        return minimum;
    }
    return layout->sizeHint();
}

bool LayoutNeedsAdaptiveScroll(QWidget* widget, QLayout* layout)
{
    if (widget == nullptr || layout == nullptr)
    {
        return false;
    }

    const QSize required = RequiredLayoutSize(layout).expandedTo(widget->minimumSize());
    if (!required.isValid() || required.isEmpty())
    {
        return false;
    }

    constexpr int kTolerance = 8;
    const QSize current = widget->contentsRect().size();
    if (current.isValid()
        && (required.width() > current.width() + kTolerance
            || required.height() > current.height() + kTolerance))
    {
        return true;
    }

    const QRect available = ResolveUsableGeometry(widget);
    if (!available.isValid())
    {
        return false;
    }

    const bool isDialog = qobject_cast<QDialog*>(widget) != nullptr;
    const double widthRatio = isDialog ? 0.88 : 0.94;
    const double heightRatio = isDialog ? 0.86 : 0.92;
    const QSize availableSize = available.size();
    const int widthLimit = (std::min)(availableSize.width(), static_cast<int>(availableSize.width() * widthRatio));
    const int heightLimit = (std::min)(availableSize.height(), static_cast<int>(availableSize.height() * heightRatio));
    return required.width() > widthLimit + kTolerance
        || required.height() > heightLimit + kTolerance;
}

bool UnwrapAdaptiveScrollAreaIfPossible(QWidget* widget, bool force = false)
{
    if (widget == nullptr)
    {
        return false;
    }

    QLayout* rootLayout = widget->layout();
    QBoxLayout* rootBox = qobject_cast<QBoxLayout*>(rootLayout);
    if (rootBox == nullptr)
    {
        return false;
    }

    QScrollArea* scrollArea = nullptr;
    if (!LayoutHasSingleAutoWrappedScrollArea(rootBox, &scrollArea) || scrollArea == nullptr)
    {
        return false;
    }

    QWidget* contentWidget = scrollArea->widget();
    QLayout* contentLayout = contentWidget != nullptr ? contentWidget->layout() : nullptr;
    if (contentLayout == nullptr)
    {
        return false;
    }
    if (!force && LayoutNeedsAdaptiveScroll(widget, contentLayout))
    {
        return false;
    }

    QLayoutItem* scrollItem = rootBox->takeAt(0);
    delete scrollItem;

    const QMargins margins = contentLayout->contentsMargins();
    const int spacing = contentLayout->spacing();
    while (contentLayout->count() > 0)
    {
        QLayoutItem* item = contentLayout->takeAt(0);
        if (item != nullptr)
        {
            rootBox->addItem(item);
        }
    }

    rootBox->setContentsMargins(margins);
    rootBox->setSpacing(spacing);

    scrollArea->takeWidget();
    scrollArea->deleteLater();
    contentWidget->deleteLater();
    widget->setProperty(kAdaptiveScrollInstalledProperty, false);
    return true;
}

bool WrapMainWindowCentralWidget(QMainWindow* mainWindow)
{
    if (mainWindow == nullptr)
    {
        return false;
    }

    // Main dashboards usually have responsive splitters/layouts. Wrapping the whole
    // central widget makes QScrollArea use sizeHint as an overflow signal, so a page
    // that could have compressed cleanly may show an unnecessary scrollbar.
    ConfigureExistingScrollAreas(mainWindow);
    return false;
}

bool WrapLayoutInAdaptiveScrollArea(QWidget* widget)
{
    if (widget == nullptr)
    {
        return false;
    }
    if (widget->property(kAdaptiveScrollContentProperty).toBool())
    {
        return false;
    }
    if (widget->property(kDirectMouseInputWindowProperty).toBool())
    {
        ConfigureExistingScrollAreas(widget);
        return false;
    }

    QLayout* rootLayout = widget->layout();
    if (rootLayout == nullptr || rootLayout->count() <= 0)
    {
        return false;
    }
    if (widget->property("_management_embedded_page").toBool())
    {
        // Management subpages are interactive dialogs embedded as widgets. Keep
        // their root layout intact; only configure scroll areas they own.
        ConfigureExistingScrollAreas(widget);
        return false;
    }
    if (rootLayout->menuBar() != nullptr)
    {
        // Keep top-level menu bars in their original root layout; moving only
        // layout items into a scroll area can compress the menu/title region.
        ConfigureExistingScrollAreas(widget);
        return false;
    }

    QScrollArea* existingScrollArea = nullptr;
    if (LayoutHasSingleScrollArea(rootLayout, &existingScrollArea))
    {
        ConfigureAdaptiveScrollArea(existingScrollArea);
        return false;
    }

    QBoxLayout* rootBox = qobject_cast<QBoxLayout*>(rootLayout);
    QGridLayout* rootGrid = qobject_cast<QGridLayout*>(rootLayout);
    if (rootBox == nullptr && rootGrid == nullptr)
    {
        return false;
    }
    if (!LayoutNeedsAdaptiveScroll(widget, rootLayout))
    {
        ConfigureExistingScrollAreas(widget);
        return false;
    }

    QWidget* contentWidget = new QWidget(widget);
    contentWidget->setObjectName("AdaptiveWindowScrollContent");
    contentWidget->setProperty(kAdaptiveScrollContentProperty, true);
    contentWidget->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);

    QBoxLayout* contentLayout = CreateMirroredBoxLayout(rootLayout, contentWidget);
    contentWidget->setLayout(contentLayout);
    while (rootLayout->count() > 0)
    {
        QLayoutItem* item = rootLayout->takeAt(0);
        if (item != nullptr)
        {
            contentLayout->addItem(item);
        }
    }

    rootLayout->setContentsMargins(0, 0, 0, 0);
    rootLayout->setSpacing(0);

    QScrollArea* scrollArea = new QScrollArea(widget);
    scrollArea->setObjectName("AdaptiveWindowScrollArea");
    scrollArea->setProperty(kAdaptiveScrollAutoWrappedProperty, true);
    ConfigureAdaptiveScrollArea(scrollArea);
    scrollArea->setWidget(contentWidget);

    if (rootBox != nullptr)
    {
        rootBox->addWidget(scrollArea);
        return true;
    }

    rootGrid->addWidget(scrollArea, 0, 0, 1, 1);
    return true;
}

void EnsureAdaptiveScrollSupport(QWidget* widget)
{
    if (widget == nullptr)
    {
        return;
    }
    if (widget->property(kDirectMouseInputWindowProperty).toBool())
    {
        UnwrapAdaptiveScrollAreaIfPossible(widget, true);
        ConfigureExistingScrollAreas(widget);
        return;
    }
    if (widget->property(kAdaptiveScrollInstalledProperty).toBool())
    {
        UnwrapAdaptiveScrollAreaIfPossible(widget);
        ConfigureExistingScrollAreas(widget);
        return;
    }

    bool installed = false;
    if (QMainWindow* mainWindow = qobject_cast<QMainWindow*>(widget))
    {
        installed = WrapMainWindowCentralWidget(mainWindow);
    }
    else
    {
        installed = WrapLayoutInAdaptiveScrollArea(widget);
    }

    if (installed)
    {
        widget->setProperty(kAdaptiveScrollInstalledProperty, true);
        RelaxWindowMinimumSizeForAdaptiveScroll(widget);
    }
    ConfigureExistingScrollAreas(widget);
}

void ScheduleAdaptiveScrollSupport(QWidget* widget)
{
    if (widget == nullptr)
    {
        return;
    }
    if (widget->property(kAdaptiveScrollCheckPendingProperty).toBool())
    {
        return;
    }

    widget->setProperty(kAdaptiveScrollCheckPendingProperty, true);
    QTimer::singleShot(0, widget, [widget]() {
        widget->setProperty(kAdaptiveScrollCheckPendingProperty, false);
        EnsureAdaptiveScrollSupport(widget);
    });
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
            EnsureAdaptiveScrollSupport(widget);
            ScheduleAdaptiveScrollSupport(widget);
            QTimer::singleShot(60, widget, [widget]() {
                ClampWindowToAvailableGeometry(widget);
                EnsureAdaptiveScrollSupport(widget);
            });
            QTimer::singleShot(160, widget, [widget]() {
                ClampWindowToAvailableGeometry(widget);
                EnsureAdaptiveScrollSupport(widget);
            });
            break;
        case QEvent::Resize:
            ScheduleAdaptiveScrollSupport(widget);
            break;
        case QEvent::WindowStateChange:
            ClampWindowToAvailableGeometry(widget);
            ScheduleAdaptiveScrollSupport(widget);
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
        QTimer::singleShot(0, dialog, [dialog]() { ApplyCompactControlWidths(dialog); });
        QTimer::singleShot(120, dialog, [dialog]() { ApplyCompactControlWidths(dialog); });
    }

    widget->installEventFilter(WindowGeometryGuard::Instance());
    QTimer::singleShot(0, widget, [widget]() { ClampWindowToAvailableGeometry(widget); });

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

void ApplyAdaptiveScrollSupport(QWidget* widget)
{
    EnsureAdaptiveScrollSupport(widget);
}

void MarkDirectMouseInputWindow(QWidget* widget)
{
    if (widget == nullptr)
    {
        return;
    }
    widget->setProperty(kDirectMouseInputWindowProperty, true);
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
