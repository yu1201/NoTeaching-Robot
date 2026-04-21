#include "QtWidgetsApplication4.h"

#include <QDir>
#include <QFileInfo>
#include <QIcon>
#include <QStringList>
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
}

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
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
