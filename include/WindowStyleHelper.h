#pragma once

#include <QSize>
#include <QString>

#include <functional>
#include <memory>

class QApplication;
class QScrollArea;
class QWidget;

class DelayedLoadingGuardPrivate;

// 打开子界面或读取配置时使用的延迟加载提示：超过 delayMs 还没结束才显示，
// Finish 或析构后自动关闭，避免短操作闪一下加载框。
class DelayedLoadingGuard
{
public:
    explicit DelayedLoadingGuard(QWidget* owner, const QString& text, int delayMs = 1000);
    ~DelayedLoadingGuard();

    void SetText(const QString& text);
    void Pulse();
    void Finish();

private:
    std::unique_ptr<DelayedLoadingGuardPrivate> d;
};

// 统一所有顶层窗口的标题栏：应用图标、Windows 深色标题栏、标题栏背景色，
// 并默认打开最小化/最大化/关闭按钮，支持各页面独立放大缩小。
// 同时会按当前屏幕可用区域（已扣除任务栏）自动限制窗口尺寸。
void ApplyUnifiedWindowChrome(QWidget* widget);

// 设置应用默认中文字体 fallback，避免下拉框、菜单和标题在缺少默认中文字库的设备上乱码。
void ConfigureApplicationFontFallback();

// 统一配置显式滚动区：透明背景、按需滚动条、内容随视口伸缩。
void ConfigureResponsiveScrollArea(QScrollArea* scrollArea);

// 统一页面适配默认项：显式滚动区、紧凑控件宽度、数字输入框编辑框化。
void ApplyResponsivePageDefaults(QWidget* widget);

// 数字输入框默认只保留编辑框，去掉上下调节按钮，减少小屏显示挤压。
void ApplyEditorOnlySpinBoxes(QWidget* widget);

// 全局禁用数字框/下拉框的鼠标滚轮改值：滚动页面时悬停在编辑框上会误改参数。
// main 里安装一次全程序生效；下拉框弹出列表内的滚动不受影响。
void InstallGlobalWheelGuard(QApplication& app);

// 窗口显示或最大化后，补刷新 Windows 原生标题栏颜色。
void RefreshUnifiedWindowTitleBar(QWidget* widget);

// 统一深色主题下拉框样式，后续所有 QComboBox 优先复用这个模板。
QString UnifiedComboBoxStyleSheet();

// 统一压缩短内容控件宽度，避免输入框、下拉框和普通按钮跟随整行无限拉长。
// 如确实需要保持宽控件，可给控件设置属性 _keep_wide_control=true。
void ApplyCompactControlWidths(QWidget* widget);

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
