#include "PointCloud3DView.h"

#include <QByteArray>
#include <QFile>
#include <QIODevice>

#include <array>
#include <charconv>
#include <cmath>
#include <system_error>

namespace pcview
{
    namespace
    {
        inline bool IsSeparator(char c)
        {
            return c == ' ' || c == '\t' || c == '\r' || c == '\n' || c == ',';
        }

        // 一行分词结果：只记录每个 token 的起始指针与长度，零分配。点云行通常 ≤6 列，
        // 多出的 token 直接忽略（足够覆盖 idx x y z source 这类格式与表头）。
        struct LineTokens
        {
            static constexpr int kMax = 16;
            std::array<const char*, kMax> begin{};
            std::array<int, kMax> length{};
            int count = 0;
        };

        void TokenizeLine(const char* data, qsizetype length, LineTokens& out)
        {
            out.count = 0;
            qsizetype i = 0;
            while (i < length && out.count < LineTokens::kMax)
            {
                if (IsSeparator(data[i]))
                {
                    ++i;
                    continue;
                }
                qsizetype j = i + 1;
                while (j < length && !IsSeparator(data[j]))
                {
                    ++j;
                }
                out.begin[out.count] = data + i;
                out.length[out.count] = static_cast<int>(j - i);
                ++out.count;
                i = j;
            }
        }

        bool TokenToDouble(const char* text, int length, double& value)
        {
            if (length <= 0)
            {
                return false;
            }
            // locale 无关 + 无拷贝；要求整个 token 都被解析（拒绝 "1.2x" 这类，等价 QString::toDouble 的严格性）。
            const std::from_chars_result parsed = std::from_chars(text, text + length, value);
            return parsed.ec == std::errc() && parsed.ptr == text + length && std::isfinite(value);
        }

        // token 是否为表头名 x/y/z（不区分大小写）。
        bool TokenEqualsChar(const char* text, int length, char lower)
        {
            if (length != 1)
            {
                return false;
            }
            const char c = text[0];
            return (c == lower) || (c == (lower - 'a' + 'A'));
        }
    } // namespace

    LoadedPointCloudFile LoadPointCloudFile3D(const QString& filePath, const PointCloudLoadProgress& progress)
    {
        LoadedPointCloudFile result;
        QFile file(filePath);
        if (!file.open(QIODevice::ReadOnly))
        {
            result.error = QString("打开失败：%1").arg(file.errorString());
            return result;
        }

        // 整文件一次读入内存（数十 MB 量级一次性 OS 读，远快于逐行 QTextStream）。
        const QByteArray content = file.readAll();
        file.close();
        const char* const data = content.constData();
        const qsizetype total = content.size();
        // 行均约 24~40 字节，预留容量减少 QVector 扩容。
        result.points.reserve(static_cast<int>(std::min<qsizetype>(total / 24 + 16, 8'000'000)));

        int xIndex = -1;
        int yIndex = -1;
        int zIndex = -1;
        bool hasHeader = false;
        qsizetype pos = 0;
        qsizetype lineNumber = 0;
        int lastPercent = -1;
        LineTokens tokens;
        while (pos < total)
        {
            const qsizetype lineStart = pos;
            while (pos < total && data[pos] != '\n')
            {
                ++pos;
            }
            const qsizetype lineLength = pos - lineStart;
            if (pos < total)
            {
                ++pos;  // 跳过 '\n'
            }
            ++lineNumber;

            // 周期性回报进度并检查取消（每 16384 行一次，避免回调风暴）。
            if (progress && (lineNumber & 0x3FFF) == 0)
            {
                const int percent = total > 0 ? static_cast<int>(lineStart * 100 / total) : 0;
                if (percent != lastPercent)
                {
                    lastPercent = percent;
                    if (!progress(percent))
                    {
                        result.canceled = true;
                        result.error = QStringLiteral("已取消");
                        return result;
                    }
                }
            }

            const char* line = data + lineStart;
            // 跳过行首空白找首个有效字符。
            qsizetype head = 0;
            while (head < lineLength && IsSeparator(line[head]))
            {
                ++head;
            }
            if (head >= lineLength)
            {
                continue;  // 空行
            }

            if (line[head] == '#')
            {
                // 注释式表头：# x y z 之类，识别列序。
                TokenizeLine(line + head + 1, lineLength - head - 1, tokens);
                for (int index = 0; index < tokens.count; ++index)
                {
                    if (TokenEqualsChar(tokens.begin[index], tokens.length[index], 'x')) xIndex = index;
                    else if (TokenEqualsChar(tokens.begin[index], tokens.length[index], 'y')) yIndex = index;
                    else if (TokenEqualsChar(tokens.begin[index], tokens.length[index], 'z')) zIndex = index;
                }
                hasHeader = xIndex >= 0 && yIndex >= 0 && zIndex >= 0;
                continue;
            }

            TokenizeLine(line, lineLength, tokens);
            if (tokens.count == 0)
            {
                continue;
            }

            if (!hasHeader)
            {
                double probe = 0.0;
                if (!TokenToDouble(tokens.begin[0], tokens.length[0], probe))
                {
                    // 首列不是数字 = 文本表头行：按列名识别 x/y/z。
                    for (int index = 0; index < tokens.count; ++index)
                    {
                        if (TokenEqualsChar(tokens.begin[index], tokens.length[index], 'x')) xIndex = index;
                        else if (TokenEqualsChar(tokens.begin[index], tokens.length[index], 'y')) yIndex = index;
                        else if (TokenEqualsChar(tokens.begin[index], tokens.length[index], 'z')) zIndex = index;
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
                if (tokens.count >= 4)
                {
                    xi = 1;  // idx x y z [source]
                    yi = 2;
                    zi = 3;
                }
                else if (tokens.count >= 3)
                {
                    xi = 0;  // x y z
                    yi = 1;
                    zi = 2;
                }
            }
            if (xi < 0 || yi < 0 || zi < 0 || xi >= tokens.count || yi >= tokens.count || zi >= tokens.count)
            {
                ++result.skippedLineCount;
                continue;
            }

            PointCloudVec3 point;
            if (!TokenToDouble(tokens.begin[xi], tokens.length[xi], point.x)
                || !TokenToDouble(tokens.begin[yi], tokens.length[yi], point.y)
                || !TokenToDouble(tokens.begin[zi], tokens.length[zi], point.z))
            {
                ++result.skippedLineCount;
                continue;
            }
            result.points.push_back(point);
        }

        if (progress)
        {
            progress(100);
        }
        if (result.points.isEmpty())
        {
            result.error = QString("未读取到有效 x/y/z 点，跳过行数：%1").arg(result.skippedLineCount);
        }
        return result;
    }
} // namespace pcview
