#include "RobotModelOriginalPreview.h"
#include "RobotModelRemoteCatalog.h"

#include <QElapsedTimer>
#include <QFileInfo>
#include <QGuiApplication>
#include <QImage>
#include <QTextStream>

int main(int argc, char* argv[])
{
    QGuiApplication application(argc, argv);
    if (application.arguments().size() != 3)
    {
        QTextStream(stderr)
            << "usage: RobotModelOriginalPreviewTests <robot.step> <output.png>"
            << Qt::endl;
        return 2;
    }

    const QString sourcePath = application.arguments().at(1);
    const QString outputPath = application.arguments().at(2);
    if (!QFileInfo::exists(sourcePath))
    {
        QTextStream(stderr)
            << "FAIL: STEP fixture missing: " << sourcePath << Qt::endl;
        return 3;
    }

    QElapsedTimer timer;
    timer.start();
    QImage image;
    QString error;
    if (!RobotModelOriginalPreview::RenderStepFile(
            sourcePath,
            QStringLiteral("新时达 SA10-2000H"),
            image,
            error))
    {
        QTextStream(stderr)
            << "FAIL: " << error << Qt::endl;
        return 4;
    }
    if (image.width() != RobotModelRemoteCatalog::PreviewWidth
        || image.height() != RobotModelRemoteCatalog::PreviewHeight)
    {
        QTextStream(stderr)
            << "FAIL: unexpected preview dimensions" << Qt::endl;
        return 5;
    }

    qint64 visibleCadPixels = 0;
    for (int y = 42; y < image.height() - 36; ++y)
    {
        for (int x = 24; x < image.width() - 24; ++x)
        {
            const QColor color = image.pixelColor(x, y);
            if (color.red() >= 35
                && color.green() >= 90
                && color.blue() >= 120)
            {
                ++visibleCadPixels;
            }
        }
    }
    if (visibleCadPixels < 2000)
    {
        QTextStream(stderr)
            << "FAIL: original CAD silhouette is unexpectedly sparse: "
            << visibleCadPixels << Qt::endl;
        return 6;
    }
    if (!image.save(outputPath, "PNG", 9))
    {
        QTextStream(stderr)
            << "FAIL: cannot save rendered preview: "
            << outputPath << Qt::endl;
        return 7;
    }

    QTextStream(stdout)
        << "RobotModelOriginalPreview tests passed: cadPixels="
        << visibleCadPixels
        << " elapsedMs=" << timer.elapsed()
        << " output=" << outputPath
        << Qt::endl;
    return 0;
}
