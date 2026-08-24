#include "AppPaths.h"

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QSet>

namespace
{
constexpr auto DATA_ROOT_ENV = "QTWIDGETSAPP4_DATA_ROOT";

bool g_initialized = false;
bool g_explicitDataRoot = false;
QString g_originalWorkingDirectory;
QString g_installRoot;
QString g_dataRoot;

QString CleanAbsolutePath(const QString& path, const QString& relativeBase)
{
    const QString nativeNeutral = QDir::fromNativeSeparators(path.trimmed());
    if (nativeNeutral.isEmpty())
    {
        return QString();
    }
    const QFileInfo info(nativeNeutral);
#ifdef Q_OS_WIN
    const int colonIndex = nativeNeutral.indexOf(QLatin1Char(':'));
    const bool absoluteDrivePath = colonIndex == 1
        && nativeNeutral.at(0).isLetter()
        && nativeNeutral.size() >= 3
        && nativeNeutral.at(2) == QLatin1Char('/')
        && info.isAbsolute();
    // Windows 的 C:foo 是“当前盘工作目录相对路径”，而 path:stream 会写入 ADS；
    // 两者都不允许成为运行目录或 CLI 文件路径。
    if ((colonIndex >= 0 && !absoluteDrivePath)
        || (absoluteDrivePath && nativeNeutral.indexOf(QLatin1Char(':'), 2) >= 0))
    {
        return QString();
    }
#endif
    const QString absolute = info.isAbsolute()
        ? info.absoluteFilePath()
        : QDir(relativeBase).absoluteFilePath(nativeNeutral);
    return QDir::cleanPath(QFileInfo(absolute).absoluteFilePath());
}

QString SafeRelativePath(const QString& path)
{
    const QString nativeNeutral = QDir::fromNativeSeparators(path.trimmed());
    const QString normalized = QDir::cleanPath(nativeNeutral);
    if (path.trimmed().isEmpty()
        || normalized == QStringLiteral(".")
        || QFileInfo(normalized).isAbsolute()
        || QDir::isAbsolutePath(normalized)
        || normalized.startsWith(QStringLiteral("//"))
        || normalized == QStringLiteral("..")
        || normalized.startsWith(QStringLiteral("../")))
    {
        return QString();
    }
#ifdef Q_OS_WIN
    // 同时拒绝 drive-relative 路径与 NTFS alternate data stream。
    if (nativeNeutral.contains(QLatin1Char(':')))
    {
        return QString();
    }
#endif
    return normalized;
}

QString JoinContainedPath(const QString& rootPath, const QString& relativePath)
{
    const QString safeRelative = SafeRelativePath(relativePath);
    const QString cleanRoot = QDir::cleanPath(rootPath);
    if (cleanRoot.isEmpty() || safeRelative.isEmpty())
    {
        return QString();
    }

    const QString candidate = QDir::cleanPath(
        QDir(cleanRoot).absoluteFilePath(safeRelative));
    const QString relativeCheck = QDir(cleanRoot)
        .relativeFilePath(candidate)
        .replace(QLatin1Char('\\'), QLatin1Char('/'));
    if (relativeCheck == QStringLiteral("..")
        || relativeCheck.startsWith(QStringLiteral("../"))
        || QFileInfo(relativeCheck).isAbsolute()
        || QDir::isAbsolutePath(relativeCheck))
    {
        return QString();
    }
    return candidate;
}

bool IsWindowsReservedComponent(const QString& component)
{
#ifdef Q_OS_WIN
    if (component.endsWith(QLatin1Char('.')) || component.endsWith(QLatin1Char(' ')))
    {
        return true;
    }
    const QString stem = component.section(QLatin1Char('.'), 0, 0).toUpper();
    static const QSet<QString> reserved = {
        QStringLiteral("CON"), QStringLiteral("PRN"), QStringLiteral("AUX"), QStringLiteral("NUL"),
        QStringLiteral("COM1"), QStringLiteral("COM2"), QStringLiteral("COM3"), QStringLiteral("COM4"),
        QStringLiteral("COM5"), QStringLiteral("COM6"), QStringLiteral("COM7"), QStringLiteral("COM8"),
        QStringLiteral("COM9"), QStringLiteral("LPT1"), QStringLiteral("LPT2"), QStringLiteral("LPT3"),
        QStringLiteral("LPT4"), QStringLiteral("LPT5"), QStringLiteral("LPT6"), QStringLiteral("LPT7"),
        QStringLiteral("LPT8"), QStringLiteral("LPT9")
    };
    return reserved.contains(stem);
#else
    Q_UNUSED(component);
    return false;
#endif
}

QString FindDevelopmentRoot(const QString& startPath)
{
    QDir dir(startPath);
    for (int depth = 0; depth < 8; ++depth)
    {
        if (QFileInfo::exists(dir.filePath(QStringLiteral("QtWidgetsApplication4.sln"))))
        {
            return QDir::cleanPath(dir.absolutePath());
        }
        if (!dir.cdUp())
        {
            break;
        }
    }
    return QString();
}

QString DefaultInstallRoot()
{
    const QString applicationDir = QCoreApplication::applicationDirPath();
    const QDir executableDir(applicationDir);
    const bool selfContainedDeployment =
        QFileInfo::exists(executableDir.filePath(QStringLiteral("DEPLOY_NOTES.txt")))
        || QFileInfo::exists(executableDir.filePath(QStringLiteral("BUILD_VERSION.txt")))
        || (QFileInfo(executableDir.filePath(QStringLiteral("Data"))).isDir()
            && QFileInfo(executableDir.filePath(QStringLiteral("SDK"))).isDir());
    if (selfContainedDeployment)
    {
        return QDir::cleanPath(applicationDir);
    }
    const QString developmentRoot = FindDevelopmentRoot(applicationDir);
    if (!developmentRoot.isEmpty())
    {
        const QString relative = QDir(developmentRoot)
            .relativeFilePath(applicationDir)
            .replace('\\', '/')
            .toLower();
        if (relative == QStringLiteral("x64")
            || relative == QStringLiteral("x64/debug")
            || relative == QStringLiteral("x64/release"))
        {
            return developmentRoot;
        }
    }
    // 非标准位置即使位于仓库树内也视为自包含部署目录。这样缺少
    // DEPLOY_NOTES/SDK/Tools 的坏包不会向上偷用源码树资源而假通过。
    return QDir::cleanPath(applicationDir);
}

bool ParseDataRootArgument(
    const QStringList& arguments,
    const QString& originalWorkingDirectory,
    QString& dataRoot,
    bool& explicitlySelected,
    QString* error)
{
    QString selected;
    for (int index = 1; index < arguments.size(); ++index)
    {
        const QString argument = arguments.at(index);
        QString value;
        if (argument == QStringLiteral("--data-root"))
        {
            if (index + 1 >= arguments.size()
                || arguments.at(index + 1).startsWith(QStringLiteral("--")))
            {
                if (error != nullptr)
                {
                    *error = QStringLiteral("--data-root 缺少目录参数。");
                }
                return false;
            }
            value = arguments.at(++index);
        }
        else if (argument.startsWith(QStringLiteral("--data-root=")))
        {
            value = argument.mid(QStringLiteral("--data-root=").size());
        }
        else
        {
            continue;
        }

        const QString normalized = CleanAbsolutePath(value, originalWorkingDirectory);
        if (normalized.isEmpty())
        {
            if (error != nullptr)
            {
                *error = QStringLiteral("--data-root 目录参数为空。");
            }
            return false;
        }
        if (!selected.isEmpty()
            && QDir::cleanPath(selected).compare(normalized, Qt::CaseInsensitive) != 0)
        {
            if (error != nullptr)
            {
                *error = QStringLiteral("命令行提供了多个不同的 --data-root，无法确定数据真源。");
            }
            return false;
        }
        selected = normalized;
    }

    if (!selected.isEmpty())
    {
        dataRoot = selected;
        explicitlySelected = true;
        return true;
    }

    const QString inherited = QString::fromUtf8(qgetenv(DATA_ROOT_ENV)).trimmed();
    if (!inherited.isEmpty())
    {
        dataRoot = CleanAbsolutePath(inherited, originalWorkingDirectory);
        if (dataRoot.isEmpty())
        {
            if (error != nullptr)
            {
                *error = QStringLiteral("环境变量 %1 不是有效目录路径。")
                    .arg(QString::fromLatin1(DATA_ROOT_ENV));
            }
            return false;
        }
        explicitlySelected = true;
    }
    return true;
}

bool EnsureWritableLayout(const QString& rootPath, QString* error)
{
    const QFileInfo rootInfo(rootPath);
    if (rootInfo.exists() && !rootInfo.isDir())
    {
        if (error != nullptr)
        {
            *error = QStringLiteral("数据根路径不是目录：%1")
                .arg(QDir::toNativeSeparators(rootPath));
        }
        return false;
    }
    if (!QDir().mkpath(rootPath))
    {
        if (error != nullptr)
        {
            *error = QStringLiteral("无法创建数据根目录：%1")
                .arg(QDir::toNativeSeparators(rootPath));
        }
        return false;
    }

    QDir root(rootPath);
    const QStringList writableDirectories = {
        QStringLiteral("Data"),
        QStringLiteral("Result"),
        QStringLiteral("Log"),
        QStringLiteral("Temp"),
        QStringLiteral("Job")
    };
    for (const QString& relative : writableDirectories)
    {
        if (!root.mkpath(relative))
        {
            if (error != nullptr)
            {
                *error = QStringLiteral("无法创建运行目录：%1")
                    .arg(QDir::toNativeSeparators(root.filePath(relative)));
            }
            return false;
        }
        const QString probePath = root.filePath(
            QStringLiteral("%1/.app-paths-write-probe-%2")
                .arg(relative)
                .arg(QCoreApplication::applicationPid()));
        QFile probe(probePath);
        if (!probe.open(QIODevice::WriteOnly | QIODevice::Truncate)
            || probe.write("ok", 2) != 2)
        {
            if (error != nullptr)
            {
                *error = QStringLiteral(
                    "运行目录不可写：%1。请调整目录权限，或使用 --data-root 指定可写目录。")
                    .arg(QDir::toNativeSeparators(root.filePath(relative)));
            }
            probe.close();
            QFile::remove(probePath);
            return false;
        }
        probe.close();
        if (!QFile::remove(probePath))
        {
            if (error != nullptr)
            {
                *error = QStringLiteral("运行目录写入探针无法清理：%1")
                    .arg(QDir::toNativeSeparators(probePath));
            }
            return false;
        }
    }
    return true;
}
}

bool AppPaths::Initialize(const QStringList& arguments, QString* error)
{
    if (error != nullptr)
    {
        error->clear();
    }

    const QString originalWorkingDirectory = QDir::cleanPath(QDir::currentPath());
    const QString installRoot = DefaultInstallRoot();
    QString dataRoot = installRoot;
    bool explicitDataRoot = false;
    if (!ParseDataRootArgument(
            arguments,
            originalWorkingDirectory,
            dataRoot,
            explicitDataRoot,
            error))
    {
        return false;
    }
    dataRoot = CleanAbsolutePath(dataRoot, originalWorkingDirectory);
    if (installRoot.isEmpty() || dataRoot.isEmpty())
    {
        if (error != nullptr)
        {
            *error = QStringLiteral("无法确定安装根或数据根目录。");
        }
        return false;
    }

    if (g_initialized)
    {
        const bool sameRoot = g_dataRoot.compare(dataRoot, Qt::CaseInsensitive) == 0;
        if (!sameRoot && error != nullptr)
        {
            *error = QStringLiteral("AppPaths 已初始化，禁止在进程内切换数据根。");
        }
        return sameRoot;
    }

    if (!EnsureWritableLayout(dataRoot, error))
    {
        return false;
    }
    if (!QDir::setCurrent(dataRoot))
    {
        if (error != nullptr)
        {
            *error = QStringLiteral("无法把运行目录固定到数据根：%1")
                .arg(QDir::toNativeSeparators(dataRoot));
        }
        return false;
    }

    g_originalWorkingDirectory = originalWorkingDirectory;
    g_installRoot = QDir::cleanPath(installRoot);
    g_dataRoot = QDir::cleanPath(dataRoot);
    g_explicitDataRoot = explicitDataRoot;
    g_initialized = true;
    qputenv(DATA_ROOT_ENV, g_dataRoot.toUtf8());
    return true;
}

bool AppPaths::IsInitialized()
{
    return g_initialized;
}

QString AppPaths::OriginalWorkingDirectory()
{
    return g_initialized ? g_originalWorkingDirectory : QDir::cleanPath(QDir::currentPath());
}

QString AppPaths::InstallRootPath()
{
    return g_initialized ? g_installRoot : DefaultInstallRoot();
}

QString AppPaths::DataRootPath()
{
    return g_initialized ? g_dataRoot : InstallRootPath();
}

bool AppPaths::HasExplicitDataRoot()
{
    return g_initialized && g_explicitDataRoot;
}

QString AppPaths::WritablePath(const QString& relativePath)
{
    return JoinContainedPath(DataRootPath(), relativePath);
}

bool AppPaths::IsSafePathComponent(const QString& component)
{
    if (component.isEmpty()
        || component.size() > 240
        || component == QStringLiteral(".")
        || component == QStringLiteral("..")
        || component.contains(QLatin1Char('/'))
        || component.contains(QLatin1Char('\\'))
        || component.contains(QLatin1Char(':'))
        || QDir::isAbsolutePath(component)
        || QFileInfo(component).fileName() != component
        || IsWindowsReservedComponent(component))
    {
        return false;
    }
    for (const QChar ch : component)
    {
        if (ch.unicode() < 0x20 || ch.unicode() == 0x7f)
        {
            return false;
        }
    }
    return true;
}

QString AppPaths::WritableChildPath(
    const QString& relativeDirectory,
    const QString& component)
{
    const QString safeDirectory = SafeRelativePath(relativeDirectory);
    if (safeDirectory.isEmpty() || !IsSafePathComponent(component))
    {
        return QString();
    }
    return JoinContainedPath(DataRootPath(), safeDirectory + QLatin1Char('/') + component);
}

QString AppPaths::ResourcePath(const QString& relativePath)
{
    return JoinContainedPath(InstallRootPath(), relativePath);
}

QString AppPaths::FindResourcePath(const QString& relativePath)
{
    const QString normalized = SafeRelativePath(relativePath);
    if (normalized.isEmpty())
    {
        return QString();
    }

    const QStringList bases = {
        InstallRootPath(),
        QCoreApplication::applicationDirPath()
    };
    QSet<QString> visited;
    for (const QString& base : bases)
    {
        const QString cleanBase = QDir::cleanPath(base);
        const QString key = cleanBase.toLower();
        if (cleanBase.isEmpty() || visited.contains(key))
        {
            continue;
        }
        visited.insert(key);
        const QString candidatePath = JoinContainedPath(cleanBase, normalized);
        const QFileInfo candidate(candidatePath);
        if (candidate.exists())
        {
            return QDir::cleanPath(candidate.absoluteFilePath());
        }
    }
    return QString();
}

QString AppPaths::CommandLinePath(const QString& path)
{
    return CleanAbsolutePath(path, OriginalWorkingDirectory());
}
