#include "PointCloud3DView.h"

#include <QFile>
#include <QIODevice>
#include <QRegularExpression>
#include <QStringConverter>
#include <QStringList>
#include <QTextStream>

#include <cmath>

namespace pcview
{
    namespace
    {
        QStringList SplitPointCloudLine(const QString& text)
        {
            QString normalized = text.trimmed();
            normalized.replace(',', ' ');
            return normalized.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
        }

        bool ParseFiniteDouble(const QString& text, double& value)
        {
            bool ok = false;
            value = text.toDouble(&ok);
            return ok && std::isfinite(value);
        }
    }

    LoadedPointCloudFile LoadPointCloudFile3D(const QString& filePath)
    {
        LoadedPointCloudFile result;
        QFile file(filePath);
        if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
        {
            result.error = QString("打开失败：%1").arg(file.errorString());
            return result;
        }

        QTextStream stream(&file);
        stream.setEncoding(QStringConverter::Utf8);
        int xIndex = -1;
        int yIndex = -1;
        int zIndex = -1;
        bool hasHeader = false;
        int lineNumber = 0;
        while (!stream.atEnd())
        {
            QString line = stream.readLine().trimmed();
            ++lineNumber;
            if (line.isEmpty())
            {
                continue;
            }

            if (line.startsWith('#'))
            {
                const QStringList commentTokens = SplitPointCloudLine(line.mid(1));
                for (int index = 0; index < commentTokens.size(); ++index)
                {
                    const QString token = commentTokens.at(index).trimmed().toLower();
                    if (token == "x")
                    {
                        xIndex = index;
                    }
                    else if (token == "y")
                    {
                        yIndex = index;
                    }
                    else if (token == "z")
                    {
                        zIndex = index;
                    }
                }
                hasHeader = xIndex >= 0 && yIndex >= 0 && zIndex >= 0;
                continue;
            }

            const QStringList tokens = SplitPointCloudLine(line);
            if (tokens.isEmpty())
            {
                continue;
            }

            if (!hasHeader)
            {
                bool firstValueOk = false;
                tokens.first().toDouble(&firstValueOk);
                if (!firstValueOk)
                {
                    for (int index = 0; index < tokens.size(); ++index)
                    {
                        const QString token = tokens.at(index).trimmed().toLower();
                        if (token == "x")
                        {
                            xIndex = index;
                        }
                        else if (token == "y")
                        {
                            yIndex = index;
                        }
                        else if (token == "z")
                        {
                            zIndex = index;
                        }
                    }
                    hasHeader = xIndex >= 0 && yIndex >= 0 && zIndex >= 0;
                    continue;
                }
            }

            int xi = xIndex;
            int yi = yIndex;
            int zi = zIndex;
            if (!hasHeader)
            {
                if (tokens.size() >= 4)
                {
                    xi = 1;
                    yi = 2;
                    zi = 3;
                }
                else if (tokens.size() >= 3)
                {
                    xi = 0;
                    yi = 1;
                    zi = 2;
                }
            }
            if (xi < 0 || yi < 0 || zi < 0 || xi >= tokens.size() || yi >= tokens.size() || zi >= tokens.size())
            {
                ++result.skippedLineCount;
                continue;
            }

            PointCloudVec3 point;
            if (!ParseFiniteDouble(tokens.at(xi), point.x)
                || !ParseFiniteDouble(tokens.at(yi), point.y)
                || !ParseFiniteDouble(tokens.at(zi), point.z))
            {
                ++result.skippedLineCount;
                continue;
            }
            result.points.push_back(point);
        }

        if (result.points.isEmpty())
        {
            result.error = QString("未读取到有效 x/y/z 点，跳过行数：%1").arg(result.skippedLineCount);
        }
        return result;
    }
} // namespace pcview
