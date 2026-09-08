#include "BrandingConfig.h"

#include "AppPaths.h"
#include "ConfigDatabase.h"

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QStandardPaths>
#include <QStringList>

#ifdef Q_OS_WIN
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <shlobj.h>
#include <objbase.h>
#pragma comment(lib, "ole32.lib")
#pragma comment(lib, "uuid.lib")
#pragma comment(lib, "shell32.lib")
#endif

namespace
{
const QString kDefaultAppName = QStringLiteral("NoTeaching-Robot");
const QString kDefaultIconResource = QStringLiteral(":/QtWidgetsApplication4/icons/minimal_robot_icon_blue_black.svg");
const QString kCfgScope = QStringLiteral("global");
const QString kCfgModule = QStringLiteral("Desktop/Icon");
const QString kCfgKey = QStringLiteral("ShowBackground");
const QString kBrandingModule = QStringLiteral("Branding");
const QString kBrandIconColor = QStringLiteral("app_color.ico");
const QString kBrandIconNoBg = QStringLiteral("app_nobg.ico");
// 安装快捷方式文件名（installer/*.iss 的 [Icons] 用 MyAppName="NoTeaching-Robot"，运行时按此固定名查找）
const QString kShortcutBaseName = QStringLiteral("NoTeaching-Robot");

// 品牌资源只能来自可信安装根，禁止可写 data-root 或启动 cwd 覆盖发布通道身份。
QString ResolveBrandingDir()
{
    const QStringList bases = {
        AppPaths::InstallRootPath(),
        QCoreApplication::applicationDirPath()
    };
    for (const QString& base : bases)
    {
        if (base.isEmpty())
        {
            continue;
        }
        const QString candidate = QDir(base).filePath(QStringLiteral("branding"));
        if (QFileInfo::exists(QDir(candidate).filePath(kBrandIconColor))
            && QFileInfo::exists(QDir(candidate).filePath(kBrandIconNoBg)))
        {
            return QDir(candidate).absolutePath();
        }
    }
    return QString();
}

QString BrandingDir()
{
    static const QString dir = ResolveBrandingDir();  // 进程内解析一次
    return dir;
}

bool ParseBool(const QString& value)
{
    const QString normalized = value.trimmed().toLower();
    return normalized == QStringLiteral("1")
        || normalized == QStringLiteral("true")
        || normalized == QStringLiteral("yes");
}

void EnsureBrandingDatabaseDefaults()
{
    QString active;
    if (ConfigDatabase::ReadScopedSetting(kCfgScope, QString(), kBrandingModule,
            QStringLiteral("Active"), &active))
    {
        return;
    }
    const bool hasBrandAssets = !BrandingDir().isEmpty();
    if (!hasBrandAssets)
    {
        return;
    }
    QMap<QString, QString> defaults;
    defaults.insert(QStringLiteral("Active"), QStringLiteral("true"));
    defaults.insert(QStringLiteral("ApplicationName"), QStringLiteral("海瞰智焊 HK-Pathlynx-CORPLA"));
    defaults.insert(QStringLiteral("DisplayName"), QStringLiteral("海瞰智焊"));
    defaults.insert(QStringLiteral("DashboardTitle"), QStringLiteral("海瞰智焊 HK-Pathlynx-CORPLA"));
    defaults.insert(QStringLiteral("SloganZh"), QStringLiteral("海瞰智焊｜波纹板专属免示教焊接系统"));
    defaults.insert(QStringLiteral("SloganEn"), QStringLiteral("Auto-Weld for Corrugated Plates, No Teaching Required"));
    defaults.insert(QStringLiteral("IconColor"), kBrandIconColor);
    defaults.insert(QStringLiteral("IconNoBg"), kBrandIconNoBg);
    ConfigDatabase::WriteScopedSettings(kCfgScope, QString(), kBrandingModule, defaults);
}

QString DatabaseValue(const QString& key, const QString& fallback)
{
    EnsureBrandingDatabaseDefaults();
    QString value;
    if (!ConfigDatabase::ReadScopedSetting(
            kCfgScope, QString(), kBrandingModule, key, &value))
    {
        return fallback;
    }
    const QString trimmed = value.trimmed();
    return trimmed.isEmpty() ? fallback : trimmed;
}
}  // namespace

bool BrandingConfig::IsActive()
{
    EnsureBrandingDatabaseDefaults();
    QString value;
    return ConfigDatabase::ReadScopedSetting(
            kCfgScope, QString(), kBrandingModule, QStringLiteral("Active"), &value)
        && ParseBool(value);
}

QString BrandingConfig::ApplicationName()
{
    return IsActive()
        ? DatabaseValue(QStringLiteral("ApplicationName"), kDefaultAppName)
        : kDefaultAppName;
}

QString BrandingConfig::DisplayName()
{
    return IsActive()
        ? DatabaseValue(QStringLiteral("DisplayName"), ApplicationName())
        : ApplicationName();
}

QString BrandingConfig::SloganZh()
{
    return IsActive() ? DatabaseValue(QStringLiteral("SloganZh"), QString()) : QString();
}

QString BrandingConfig::SloganEn()
{
    return IsActive() ? DatabaseValue(QStringLiteral("SloganEn"), QString()) : QString();
}

QString BrandingConfig::DashboardTitle()
{
    return IsActive()
        ? DatabaseValue(QStringLiteral("DashboardTitle"), QStringLiteral("机器人控制与调试中心"))
        : QStringLiteral("机器人控制与调试中心");
}

QString BrandingConfig::DefaultWindowIconResource()
{
    return kDefaultIconResource;
}

bool BrandingConfig::IconWithBackground()
{
    QString value;
    if (ConfigDatabase::ReadScopedSetting(kCfgScope, QString(), kCfgModule, kCfgKey, &value))
    {
        const QString v = value.trimmed().toLower();
        return v == QStringLiteral("1") || v == QStringLiteral("true") || v == QStringLiteral("yes");
    }
    return false;  // 默认无底色
}

void BrandingConfig::SetIconWithBackground(bool withBackground)
{
    ConfigDatabase::WriteScopedSetting(
        kCfgScope, QString(), kCfgModule, kCfgKey,
        withBackground ? QStringLiteral("1") : QStringLiteral("0"), QStringLiteral("bool"));
}

QString BrandingConfig::CurrentIconFilePath()
{
    if (!IsActive())
    {
        return QString();
    }
    const QString dir = BrandingDir();
    if (dir.isEmpty())
    {
        return QString();
    }
    const QString fileName = IconWithBackground()
        ? DatabaseValue(QStringLiteral("IconColor"), kBrandIconColor)
        : DatabaseValue(QStringLiteral("IconNoBg"), kBrandIconNoBg);
    const QString path = QDir(dir).filePath(fileName);
    return QFileInfo::exists(path) ? QDir::toNativeSeparators(path) : QString();
}

QIcon BrandingConfig::WindowIcon()
{
    const QString path = CurrentIconFilePath();
    if (!path.isEmpty())
    {
        return QIcon(path);
    }
    return QIcon(kDefaultIconResource);
}

int BrandingConfig::ApplyDesktopShortcutIcons()
{
    const QString iconPath = CurrentIconFilePath();
    if (iconPath.isEmpty())
    {
        return 0;
    }

#ifdef Q_OS_WIN
    const QString brand = ApplicationName();  // 品牌名（默认 NoTeaching-Robot 则不重命名，只改图标）

    // 安装器固定建 NoTeaching-Robot.lnk；逐个目录查（用户/公共桌面、用户/公共开始菜单）
    QStringList dirs;
    const QString userDesktop = QStandardPaths::writableLocation(QStandardPaths::DesktopLocation);
    if (!userDesktop.isEmpty())
    {
        dirs << userDesktop;
    }
    wchar_t pathBuf[MAX_PATH] = {0};
    for (int csidl : {CSIDL_COMMON_DESKTOPDIRECTORY, CSIDL_COMMON_PROGRAMS, CSIDL_PROGRAMS})
    {
        if (SUCCEEDED(SHGetFolderPathW(nullptr, csidl, nullptr, 0, pathBuf)))
        {
            dirs << QString::fromWCharArray(pathBuf);
        }
    }

    const HRESULT initHr = CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED);
    // 仅在本次初始化成功(含 S_FALSE：本线程已按同套间初始化过)时才配对 CoUninitialize；
    // RPC_E_CHANGED_MODE 是 FAILED 码，已被 SUCCEEDED 排除，此时不应 CoUninitialize。
    const bool needUninit = SUCCEEDED(initHr);

    int changed = 0;
    const std::wstring iconW = QDir::toNativeSeparators(iconPath).toStdWString();

    // 一处快捷方式：把 NoTeaching-Robot.lnk 重命名为 <brand>.lnk（品牌不同且原名存在时），再改图标。幂等。
    auto processLnk = [&](const QString& oldLnk, const QString& brandLnk)
    {
        QString target = oldLnk;
        if (brand != kShortcutBaseName && QFileInfo::exists(oldLnk))
        {
            if (QFileInfo::exists(brandLnk))
            {
                QFile::remove(brandLnk);  // 重装场景：用新建的覆盖旧品牌快捷方式
            }
            if (QFile::rename(oldLnk, brandLnk))
            {
                target = brandLnk;
            }
        }
        else if (QFileInfo::exists(brandLnk))
        {
            target = brandLnk;  // 之前已重命名过
        }
        if (target.isEmpty() || !QFileInfo::exists(target))
        {
            return;
        }
        IShellLinkW* link = nullptr;
        if (FAILED(CoCreateInstance(CLSID_ShellLink, nullptr, CLSCTX_INPROC_SERVER, IID_IShellLinkW, reinterpret_cast<void**>(&link))) || link == nullptr)
        {
            return;
        }
        IPersistFile* persist = nullptr;
        if (SUCCEEDED(link->QueryInterface(IID_IPersistFile, reinterpret_cast<void**>(&persist))) && persist != nullptr)
        {
            const std::wstring lnkW = QDir::toNativeSeparators(target).toStdWString();
            if (SUCCEEDED(persist->Load(lnkW.c_str(), STGM_READWRITE)))
            {
                if (SUCCEEDED(link->SetIconLocation(iconW.c_str(), 0)))
                {
                    if (SUCCEEDED(persist->Save(lnkW.c_str(), TRUE)))
                    {
                        ++changed;
                    }
                }
            }
            persist->Release();
        }
        link->Release();
    };

    const QString lnkOld = kShortcutBaseName + QStringLiteral(".lnk");
    const QString lnkBrand = brand + QStringLiteral(".lnk");
    for (const QString& d : dirs)
    {
        QDir dir(d);
        processLnk(dir.filePath(lnkOld), dir.filePath(lnkBrand));
        // 开始菜单若建了同名子文件夹（DisableProgramGroupPage=yes 时通常没有，保险处理）
        const QDir sub(dir.filePath(kShortcutBaseName));
        if (sub.exists())
        {
            processLnk(sub.filePath(lnkOld), sub.filePath(lnkBrand));
        }
    }

    if (needUninit)
    {
        CoUninitialize();
    }
    return changed;
#else
    return 0;
#endif
}
