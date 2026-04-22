#include "WindowStyleHelper.h"

#include <algorithm>

#include <QApplication>
#include <QDialog>
#include <QEvent>
#include <QGuiApplication>
#include <QIcon>
#include <QScreen>
#include <QTimer>
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

void ClampWindowToAvailableGeometry(QWidget* widget)
{
    if (widget == nullptr)
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

    const int targetWidth = (std::min)(currentSize.width(), widthLimit);
    const int targetHeight = (std::min)(currentSize.height(), heightLimit);
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

    if (QDialog* dialog = qobject_cast<QDialog*>(widget))
    {
        dialog->setSizeGripEnabled(true);
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

        const COLORREF captionColor = RGB(17, 24, 32);
        const COLORREF textColor = RGB(236, 243, 244);
        const COLORREF borderColor = RGB(46, 70, 86);
        setAttribute(hwnd, 35, &captionColor, sizeof(captionColor));
        setAttribute(hwnd, 36, &textColor, sizeof(textColor));
        setAttribute(hwnd, 34, &borderColor, sizeof(borderColor));
    }

    FreeLibrary(dwmapi);
#endif
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
    widget->resize(finalWidth, finalHeight);

    const QRect frame = widget->frameGeometry();
    widget->move(ResolveCenteredPosition(available, frame));
}
