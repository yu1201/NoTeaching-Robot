#pragma once
#ifndef BRANDINGCONFIG_H
#define BRANDINGCONFIG_H

#include <QString>
#include <QIcon>

// 本地品牌覆盖：工程根下若存在 branding/branding.ini 则启用品牌名与品牌图标，否则保持默认。
// 该目录不纳入 git（.gitignore 排除），因此 git 仓库里呈现的名称/图标始终是默认的
// NoTeaching-Robot + 原资源图标——品牌化只在放了 branding/ 的本地机器上生效。
class BrandingConfig
{
public:
    // branding/branding.ini 是否存在（决定是否启用品牌覆盖）
    static bool IsActive();

    // 应用名：启用时返回 branding.ini 的 ApplicationName，否则 "NoTeaching-Robot"
    static QString ApplicationName();
    static QString DisplayName();
    static QString SloganZh();
    static QString SloganEn();

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
