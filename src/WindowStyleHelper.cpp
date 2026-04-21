#include "WindowStyleHelper.h"

#include <QIcon>
#include <QWidget>

#ifdef Q_OS_WIN
#include <windows.h>
#endif

void ApplyUnifiedWindowChrome(QWidget* widget)
{
    if (widget == nullptr)
    {
        return;
    }

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
