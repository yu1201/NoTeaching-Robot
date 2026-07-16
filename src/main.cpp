#include "QtWidgetsApplication4.h"
#include "ApplicationInstanceGuard.h"
#include "AppPaths.h"
#include "CliHelp.h"
#include "ConfigDatabase.h"
#include "WindowStyleHelper.h"
#include "BrandingConfig.h"
#include "PointCloudExtractionProcessor.h"
#include "RobotDriverAdaptor.h"

#include <QDir>
#include <QIcon>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLibraryInfo>
#include <QLocale>
#include <QStringList>
#include <QStringConverter>
#include <QTextStream>
#include <QTranslator>
#include <QtWidgets/QApplication>
#include <QtWidgets/QMessageBox>

#include <cstdio>

namespace
{
bool IsHeadlessPathInvocation(const QStringList& arguments)
{
    return arguments.contains(QStringLiteral("--no-show"))
        || arguments.contains(QStringLiteral("--print-app-paths-json"))
        || arguments.contains(QStringLiteral("--pointcloud-extract-worker"));
}

int PrintCliHelp()
{
    QTextStream stream(stdout);
    stream.setEncoding(QStringConverter::Utf8);
    WriteCliHelp(stream);
    stream.flush();
    return 0;
}

int PrintAppPathsJson()
{
    const QJsonObject output = {
        {QStringLiteral("schemaVersion"), 1},
        {QStringLiteral("installRoot"), AppPaths::InstallRootPath()},
        {QStringLiteral("dataRoot"), AppPaths::DataRootPath()},
        {QStringLiteral("originalWorkingDirectory"), AppPaths::OriginalWorkingDirectory()},
        {QStringLiteral("currentWorkingDirectory"), QDir::currentPath()},
        {QStringLiteral("databasePath"), ConfigDatabase::DatabasePath()},
        {QStringLiteral("hasExplicitDataRoot"), AppPaths::HasExplicitDataRoot()},
        {QStringLiteral("writableProbe"),
            AppPaths::WritablePath(QStringLiteral("Result/path-probe"))},
        {QStringLiteral("safeChildProbe"),
            AppPaths::WritableChildPath(QStringLiteral("Temp/OnlineUpdate"), QStringLiteral("payload.zip"))},
        {QStringLiteral("cliRelativeProbe"),
            AppPaths::CommandLinePath(QStringLiteral("relative-path-probe"))},
        {QStringLiteral("sdkPath"), AppPaths::FindResourcePath(QStringLiteral("SDK"))},
        {QStringLiteral("toolsPath"), AppPaths::FindResourcePath(QStringLiteral("Tools"))},
        {QStringLiteral("rejectTraversal"),
            AppPaths::WritablePath(QStringLiteral("../escape")).isEmpty()},
        {QStringLiteral("rejectDriveRelative"),
            AppPaths::WritablePath(QStringLiteral("C:escape")).isEmpty()},
        {QStringLiteral("rejectAbsolute"),
            AppPaths::WritablePath(QStringLiteral("C:/escape")).isEmpty()},
        {QStringLiteral("rejectUnc"),
            AppPaths::WritablePath(QStringLiteral("//server/share")).isEmpty()},
        {QStringLiteral("rejectAds"),
            AppPaths::WritablePath(QStringLiteral("safe/file:stream")).isEmpty()},
        {QStringLiteral("rejectMixedTraversal"),
            AppPaths::WritablePath(QStringLiteral("safe\\..\\..\\escape")).isEmpty()},
        {QStringLiteral("rejectUnsafeComponent"),
            AppPaths::WritableChildPath(
                QStringLiteral("Temp/OnlineUpdate"), QStringLiteral("../../Data/ConfigStore.db")).isEmpty()},
        {QStringLiteral("rejectReservedComponent"),
            AppPaths::WritableChildPath(QStringLiteral("Temp/OnlineUpdate"), QStringLiteral("CON.txt")).isEmpty()},
        {QStringLiteral("rejectComponentSlash"),
            AppPaths::WritableChildPath(QStringLiteral("Temp/OnlineUpdate"), QStringLiteral("nested/file.zip")).isEmpty()},
        {QStringLiteral("rejectComponentAds"),
            AppPaths::WritableChildPath(QStringLiteral("Temp/OnlineUpdate"), QStringLiteral("file.zip:stream")).isEmpty()},
        {QStringLiteral("acceptUnicodeComponent"),
            !AppPaths::WritableChildPath(QStringLiteral("Temp/OnlineUpdate"), QStringLiteral("设备 1.zip")).isEmpty()}
    };
    QTextStream stream(stdout);
    stream << QJsonDocument(output).toJson(QJsonDocument::Compact) << Qt::endl;
    return 0;
}

void InstallChineseQtTranslations(QApplication& app)
{
    QLocale::setDefault(QLocale(QLocale::Chinese, QLocale::China));

    static QTranslator qtTranslator;
    static QTranslator qtBaseTranslator;

    const QStringList translationDirs = {
        AppPaths::ResourcePath(QStringLiteral("translations")),
        QDir(QCoreApplication::applicationDirPath()).filePath("translations"),
        QLibraryInfo::path(QLibraryInfo::TranslationsPath)
    };

    auto installTranslator = [&app, &translationDirs](QTranslator& translator, const QString& baseName)
    {
        for (const QString& dir : translationDirs)
        {
            if (translator.load(baseName, dir))
            {
                app.installTranslator(&translator);
                return;
            }
        }
    };

    // Load Qt's own Chinese translations so standard dialog buttons such as
    // OK/Cancel/Yes/No stay Chinese on machines that do not have Qt installed.
    installTranslator(qtTranslator, QStringLiteral("qt_zh_CN"));
    installTranslator(qtBaseTranslator, QStringLiteral("qtbase_zh_CN"));
}
}

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    const QStringList arguments = app.arguments();
    if (arguments.contains(QStringLiteral("--help-cli")))
    {
        return PrintCliHelp();
    }
    QString pathError;
    if (!AppPaths::Initialize(arguments, &pathError))
    {
        QTextStream(stderr) << "运行目录初始化失败：" << pathError << Qt::endl;
        if (!IsHeadlessPathInvocation(arguments))
        {
            QMessageBox::critical(nullptr, QStringLiteral("运行目录初始化失败"), pathError);
        }
        return 2;
    }
    if (arguments.contains(QStringLiteral("--print-app-paths-json")))
    {
        return PrintAppPathsJson();
    }
    app.setApplicationName(BrandingConfig::ApplicationName());
    app.setApplicationVersion(QStringLiteral("2026.07.16.1546"));
    app.setOrganizationName("yu1201");
    InstallChineseQtTranslations(app);
    ConfigureApplicationFontFallback();
    InstallGlobalWheelGuard(app);
    app.setWindowIcon(BrandingConfig::WindowIcon());

    // SDK 点云提取子进程模式：隔离 SDK DLL(pcl_kdtree 多线程)崩溃。在构造主窗口/连接机器人之前拦截，
    // 只调 SDK 提取后即退；子进程崩溃不会拖垮主程序(由 ExtractCorrugatedSheetIsolated 检测处理)。
    {
        const QStringList earlyArgs = app.arguments();
        const int workerIdx = earlyArgs.indexOf(QStringLiteral("--pointcloud-extract-worker"));
        if (workerIdx >= 0)
        {
            return PointCloudExtractionProcessor::RunExtractWorker(earlyArgs.mid(workerIdx + 1));
        }
    }

    // RobotOperationLease 的活动表和 STOP 锁存均为进程内状态。除上面的只读/隔离 worker
    // 入口外，中性 GUI、品牌 GUI 与 --no-show CLI 必须共享同一个跨进程单实例锁，避免
    // 第二个进程绕过租约并同时控制实体机器人。
    QString instanceGuardError;
    auto instanceGuard = ApplicationInstanceGuard::TryAcquire(
        ApplicationInstanceGuard::RobotControlScope(), &instanceGuardError);
    if (!instanceGuard)
    {
        QTextStream(stderr) << instanceGuardError << Qt::endl;
        if (!arguments.contains(QStringLiteral("--no-show")))
        {
            QMessageBox::critical(nullptr, QStringLiteral("机器人控制进程互锁"), instanceGuardError);
        }
        return 3;
    }

    // GUI 模式(非 --no-show)：机器人驱动构造不再同步连接，连接改由后台状态监控线程发起，
    // 避免机器人/相机连不上时拖慢主窗口显示(每台不可达 STEP 约 5s OS connect 超时)。
    // CLI(--no-show)保持构造内同步连接，确保 CLI 命令执行时机器人已连上。须在构造窗口前设置。
    if (!app.arguments().contains(QStringLiteral("--no-show")))
    {
        RobotDriverAdaptor::s_connectDriversAtConstruct = false;
    }

    QtWidgetsApplication4 window;
    window.setWindowIcon(BrandingConfig::WindowIcon());
    if (!arguments.contains("--no-show"))
    {
        window.show();
    }
    window.ApplyStartupArguments(arguments);
    return app.exec();
}
