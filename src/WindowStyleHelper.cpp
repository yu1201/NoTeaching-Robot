#include "WindowStyleHelper.h"

#include <algorithm>

#include <QApplication>
#include <QAbstractButton>
#include <QAbstractSpinBox>
#include <QDialog>
#include <QEvent>
#include <QFormLayout>
#include <QGuiApplication>
#include <QIcon>
#include <QComboBox>
#include <QLineEdit>
#include <QLayout>
#include <QMessageBox>
#include <QPushButton>
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
            QTimer::singleShot(60, widget, [widget]() { ClampWindowToAvailableGeometry(widget); });
            QTimer::singleShot(160, widget, [widget]() { ClampWindowToAvailableGeometry(widget); });
            break;
        case QEvent::WindowStateChange:
            ClampWindowToAvailableGeometry(widget);
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
        widget->setWindowState(widget->windowState() | Qt::WindowMaximized);
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

QString UnifiedComboBoxStyleSheet()
{
    return QStringLiteral(
        "QComboBox { background: #000000; color: #F5FAFA; border: 1px solid #385366; border-radius: 0px; padding: 4px 34px 4px 8px; min-height: 24px; }"
        "QComboBox::drop-down { subcontrol-origin: padding; subcontrol-position: top right; width: 28px; border-left: 1px solid #385366; border-radius: 0px; background: #000000; }"
        "QComboBox::down-arrow { image: url(:/QtWidgetsApplication4/icons/chevron-down.svg); width: 12px; height: 8px; }"
        "QComboBox QAbstractItemView { background: #000000; color: #F4FAFA; selection-background-color: #1F4F5C; border: 1px solid #385366; border-radius: 0px; outline: 0px; }");
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
