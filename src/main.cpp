#include "QtWidgetsApplication4.h"
#include "ApplicationInstanceGuard.h"
#include "AppPaths.h"
#include "CliHelp.h"
#include "ConfigDatabase.h"
#include "WindowStyleHelper.h"
#include "BrandingConfig.h"
#include "PointCloudExtractionProcessor.h"
#include "PointCloudProcessingConfig.h"
#include "RobotDriverAdaptor.h"
#include "LicenseManager.h"
#include "LicenseDialog.h"
#include "LicenseBuildConfig.h"
#include "RobotOperationLease.h"

#include <QDir>
#include <QCryptographicHash>
#include <QEventLoop>
#include <QProgressDialog>
#include <QTimer>
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

bool IsOfflineWeldFileInvocation(const QStringList& arguments)
{
    if (!arguments.contains(QStringLiteral("--no-show")))
    {
        return false;
    }

    return arguments.contains(QStringLiteral("--rebuild-measure-weld-files"))
        || arguments.contains(QStringLiteral("--generate-step-weld-program"));
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
    if (arguments.contains(QStringLiteral("--print-license-build-json")))
    {
        const QJsonObject build = {
            {QStringLiteral("schemaVersion"), 1},
            {QStringLiteral("licenseMode"), HK_LICENSE_MODE},
            {QStringLiteral("licenseChannel"), QString::fromUtf8(HK_LICENSE_CHANNEL)},
            {QStringLiteral("keyId"), QString::fromUtf8(HK_LICENSE_KEY_ID)},
            {QStringLiteral("publicKeySha256"), QString::fromLatin1(QCryptographicHash::hash(
                QByteArray::fromBase64(HK_LICENSE_PUBLIC_KEY_B64), QCryptographicHash::Sha256).toHex())}
        };
        QTextStream(stdout) << QJsonDocument(build).toJson(QJsonDocument::Compact) << Qt::endl;
        return 0;
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
    app.setApplicationVersion(QStringLiteral("2026.09.08.2143"));
    app.setOrganizationName("yu1201");
    InstallChineseQtTranslations(app);
    ConfigureApplicationFontFallback();
    InstallGlobalWheelGuard(app);
    app.setWindowIcon(BrandingConfig::WindowIcon());

    auto& license = LicenseManager::Instance();
    const bool extractionWorker = arguments.contains(QStringLiteral("--pointcloud-extract-worker"));
    license.Initialize(extractionWorker);
    if (!extractionWorker) license.Start();
    if (!extractionWorker && license.IsEnforced() && license.IsNetworkBusy())
    {
        app.setQuitOnLastWindowClosed(false);
        QEventLoop initialSync;
        QTimer poll;
        poll.setInterval(50);
        QObject::connect(&poll, &QTimer::timeout, &initialSync, [&]()
            { if (!license.IsNetworkBusy()) initialSync.quit(); });
        QProgressDialog checking(QStringLiteral("正在同步授权状态…"), QStringLiteral("退出"), 0, 0);
        checking.setStyleSheet(LicenseDialog::ThemeStyleSheet());
        ApplyUnifiedWindowChrome(&checking);
        checking.setWindowTitle(QStringLiteral("软件授权"));
        checking.setWindowModality(Qt::ApplicationModal);
        checking.setMinimumDuration(0);
        bool cancelled = false;
        QObject::connect(&checking, &QProgressDialog::canceled, &initialSync, [&]()
            { cancelled = true; initialSync.quit(); });
        if (!arguments.contains(QStringLiteral("--no-show"))) checking.show();
        QTimer::singleShot(16000, &initialSync, &QEventLoop::quit);
        poll.start();
        initialSync.exec();
        QObject::disconnect(&checking, nullptr, &initialSync, nullptr);
        checking.close();
        if (cancelled) return 4;
    }
    if (!license.CanStartProtectedOperation())
    {
        if (arguments.contains(QStringLiteral("--no-show")) || extractionWorker)
        {
            QTextStream(stderr) << license.StatusText() << Qt::endl;
            return 4;
        }
        // 先显示过期/激活界面；此时尚未构造主窗口，也未构造机器人驱动。
        app.setQuitOnLastWindowClosed(false);
        license.ShowDialog(nullptr, {}, {}, {}, true);
        if (!license.CanStartProtectedOperation()) return 4;
    }
    RobotOperationLease::SetLicenseOperationsAllowed(license.CanStartProtectedOperation(), license.StatusText());

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

    // 独立互锁在启动时冻结；进程单实例只受其对应开关控制。
    const bool singleProcessInterlockEnabled =
        PointCloudProcessingConfig::RuntimeSystemInterlocks().IsEnabled(SystemInterlock::SingleProcess);
    ApplicationInstanceGuard::Ptr instanceGuard;
    if (singleProcessInterlockEnabled)
    {
        QString instanceGuardError;
        instanceGuard = ApplicationInstanceGuard::TryAcquire(
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
    }
    else
    {
        const QString warning = QStringLiteral(
            "机器人控制进程单实例互锁已关闭：本进程未取得单实例锁。其他系统互锁按各自开关执行。");
        QTextStream(stderr) << warning << Qt::endl;
        if (!arguments.contains(QStringLiteral("--no-show")))
        {
            QMessageBox::warning(nullptr, QStringLiteral("单实例互锁已关闭"), warning);
        }
    }

    // GUI 模式和纯文件离线 CLI：机器人驱动构造不做同步连接。重建先测后焊文件、生成 STEP
    // 程序只读取机器人配置与本地点云/姿态文件，不应因现场控制器不可达而阻塞。
    // 其余 --no-show 机器人动作 CLI 仍保持构造内同步连接。须在构造窗口前设置。
    if (!app.arguments().contains(QStringLiteral("--no-show"))
        || IsOfflineWeldFileInvocation(app.arguments()))
    {
        RobotDriverAdaptor::s_connectDriversAtConstruct = false;
    }
    if (IsOfflineWeldFileInvocation(app.arguments()))
    {
        RobotDriverAdaptor::s_startStateMonitorsAtConstruct = false;
    }

    QtWidgetsApplication4 window;
    window.setWindowIcon(BrandingConfig::WindowIcon());
    if (!arguments.contains("--no-show"))
    {
        window.show();
    }
    app.setQuitOnLastWindowClosed(true);
    window.ApplyStartupArguments(arguments);
    return app.exec();
}
