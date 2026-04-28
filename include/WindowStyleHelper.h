#pragma once

#include <QSize>
#include <QString>

#include <functional>

class QWidget;

// 统一所有顶层窗口的标题栏：应用图标、Windows 深色标题栏、标题栏背景色，
// 并默认打开最小化/最大化/关闭按钮，支持各页面独立放大缩小。
// 同时会按当前屏幕可用区域（已扣除任务栏）自动限制窗口尺寸。
void ApplyUnifiedWindowChrome(QWidget* widget);

// 统一深色主题下拉框样式，后续所有 QComboBox 优先复用这个模板。
QString UnifiedComboBoxStyleSheet();

// 关闭有手动保存按钮的窗口时，统一询问是否保存未保存修改。
bool ConfirmCloseWithUnsavedChanges(
    QWidget* widget,
    const QString& title,
    const std::function<bool()>& saveCallback);

// 按当前屏幕可用区域，给窗口设置一个更合适的初始尺寸。
// preferredSize 为理想尺寸，maxWidthRatio / maxHeightRatio 为可用区域占比上限。
void ResizeWindowForAvailableGeometry(
    QWidget* widget,
    const QSize& preferredSize,
    double maxWidthRatio = 0.88,
    double maxHeightRatio = 0.88);
