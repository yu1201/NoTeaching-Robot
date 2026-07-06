#include "QtWidgetsApplication4.h"
#include "WindowStyleHelper.h"
#include "BrandingConfig.h"
#include "PointCloudExtractionProcessor.h"
#include "RobotDriverAdaptor.h"

#include <QDir>
#include <QFileInfo>
#include <QIcon>
#include <QLibraryInfo>
#include <QLocale>
#include <QStringList>
#include <QTranslator>
#include <QtWidgets/QApplication>

namespace
{
void SetWorkingDirectoryToProjectRoot()
{
    QDir dir(QCoreApplication::applicationDirPath());
    for (int depth = 0; depth < 6; ++depth)
    {
        if (QFileInfo::exists(dir.filePath("QtWidgetsApplication4.sln")))
        {
            QDir::setCurrent(dir.absolutePath());
            return;
        }

        if (!dir.cdUp())
        {
            break;
        }
    }
}

void InstallChineseQtTranslations(QApplication& app)
{
    QLocale::setDefault(QLocale(QLocale::Chinese, QLocale::China));

    static QTranslator qtTranslator;
    static QTranslator qtBaseTranslator;

    const QStringList translationDirs = {
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
    SetWorkingDirectoryToProjectRoot();  // 提前：使本地 branding/ 品牌覆盖可被定位（不依赖后续步骤）
    app.setApplicationName(BrandingConfig::ApplicationName());
    app.setApplicationVersion("2026.07.06.1806");
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

    // GUI 模式(非 --no-show)：机器人驱动构造不再同步连接，连接改由后台状态监控线程发起，
    // 避免机器人/相机连不上时拖慢主窗口显示(每台不可达 STEP 约 5s OS connect 超时)。
    // CLI(--no-show)保持构造内同步连接，确保 CLI 命令执行时机器人已连上。须在构造窗口前设置。
    if (!app.arguments().contains(QStringLiteral("--no-show")))
    {
        RobotDriverAdaptor::s_connectDriversAtConstruct = false;
    }

    QtWidgetsApplication4 window;
    window.setWindowIcon(BrandingConfig::WindowIcon());
    const QStringList arguments = app.arguments();
    if (!arguments.contains("--no-show"))
    {
        window.show();
    }
    window.ApplyStartupArguments(arguments);
    return app.exec();
}
