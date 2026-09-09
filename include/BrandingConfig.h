#pragma once
#ifndef BRANDINGCONFIG_H
#define BRANDINGCONFIG_H

#include <QString>
#include <QIcon>

// 品牌文字和启用状态存于 ConfigStore 数据库的 global/Branding 模块；
// branding/ 目录只承载图标资源，不再承载配置文件。
class BrandingConfig
{
public:
    // global/Branding/Active（首次发现完整品牌图标资源时会写入品牌默认值）。
    static bool IsActive();

    // 应用名：启用时返回数据库 Branding/ApplicationName，否则 "NoTeaching-Robot"
    static QString ApplicationName();
    static QString DisplayName();
    static QString SloganZh();
    static QString SloganEn();
    static QString DashboardTitle();   // 主页大标题（默认 机器人控制与调试中心）

    // 窗口/任务栏图标：启用时按“底色开关”返回品牌有底色/无底色图标，否则返回默认资源图标。
    static QIcon WindowIcon();

    // 桌面图标底色开关（持久化到 ConfigStore：scope=global, module=Desktop/Icon, key=ShowBackground，默认 false=无底色）
    static bool IconWithBackground();
    static void SetIconWithBackground(bool withBackground);

    // 当前应使用的品牌图标文件绝对路径（按底色开关选 color/nobg）；未启用或文件缺失返回空。
    static QString CurrentIconFilePath();

    // 把当前品牌图标写入已安装的桌面/开始菜单快捷方式(.lnk)的图标位置；
    // 未启用、找不到快捷方式或写入失败均静默跳过，返回成功改写的快捷方式数量。
    static int ApplyDesktopShortcutIcons();

private:
    // 默认资源图标路径（与历史一致）
    static QString DefaultWindowIconResource();
};

#endif // BRANDINGCONFIG_H
