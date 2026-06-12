#include "QtWidgetsApplication4.h"
#include "WindowStyleHelper.h"

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
    app.setApplicationName("NoTeaching-Robot");
    app.setApplicationVersion("2026.06.11.1711");
    app.setOrganizationName("yu1201");
    InstallChineseQtTranslations(app);
    ConfigureApplicationFontFallback();
    InstallGlobalWheelGuard(app);
    SetWorkingDirectoryToProjectRoot();
    app.setWindowIcon(QIcon(":/QtWidgetsApplication4/icons/minimal_robot_icon_blue_black.svg"));

    QtWidgetsApplication4 window;
    window.setWindowIcon(QIcon(":/QtWidgetsApplication4/icons/minimal_robot_icon_blue_black.svg"));
    const QStringList arguments = app.arguments();
    if (!arguments.contains("--no-show"))
    {
        window.show();
    }
    window.ApplyStartupArguments(arguments);
    return app.exec();
}
