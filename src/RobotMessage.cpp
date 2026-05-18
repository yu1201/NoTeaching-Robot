#include "RobotMessage.h"

#include <QByteArray>
#include <QCoreApplication>
#include <QMetaObject>
#include <QPushButton>
#include <QString>
#include <QThread>
#include <vector>
#ifdef Q_OS_WIN
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#endif

namespace
{
std::string safeTitle(const char* title)
{
    return title != nullptr ? std::string(title) : std::string();
}

std::string formatStringV(const char* format, va_list args)
{
    if (format == nullptr) {
        return "";
    }

    va_list argsCopy;
    va_copy(argsCopy, args);
    int bufferSize = vsnprintf(nullptr, 0, format, argsCopy) + 1;
    va_end(argsCopy);

    if (bufferSize <= 0)
    {
        return "";
    }

    std::vector<char> buffer(bufferSize);
    vsnprintf(buffer.data(), bufferSize, format, args);
    return std::string(buffer.data());
}

QString decodeMessageText(const std::string& text)
{
    if (text.empty())
    {
        return QString();
    }

    const QByteArray bytes(text.data(), static_cast<int>(text.size()));
#ifdef Q_OS_WIN
    const auto decodeWindowsCodePage = [&bytes](UINT codePage, DWORD flags) -> QString
        {
            const int wideLength = MultiByteToWideChar(
                codePage,
                flags,
                bytes.constData(),
                bytes.size(),
                nullptr,
                0);
            if (wideLength <= 0)
            {
                return QString();
            }

            std::wstring wideText(static_cast<size_t>(wideLength), L'\0');
            MultiByteToWideChar(
                codePage,
                flags,
                bytes.constData(),
                bytes.size(),
                wideText.data(),
                wideLength);
            return QString::fromWCharArray(wideText.data(), wideLength);
        };

    const QString utf8Text = decodeWindowsCodePage(CP_UTF8, MB_ERR_INVALID_CHARS);
    if (!utf8Text.isNull() && !utf8Text.contains(QChar(0xfffd)))
    {
        return utf8Text;
    }

    const QString gbkText = decodeWindowsCodePage(936, 0);
    if (!gbkText.isNull() && !gbkText.contains(QChar(0xfffd)))
    {
        return gbkText;
    }
#endif

    const QString fallbackUtf8Text = QString::fromUtf8(bytes.constData(), bytes.size());
    if (!fallbackUtf8Text.contains(QChar(0xfffd)))
    {
        return fallbackUtf8Text;
    }

    return QString::fromLocal8Bit(bytes.constData(), bytes.size());
}

void showMessageOnGuiThread(QMessageBox::Icon icon, const QString& title, const QString& text)
{
    auto show = [icon, title, text]()
    {
        QMessageBox msgBox(nullptr);
        msgBox.setIcon(icon);
        msgBox.setWindowTitle(title);
        msgBox.setText(text);
        msgBox.addButton(QStringLiteral("确定"), QMessageBox::AcceptRole);
        msgBox.exec();
    };

    QCoreApplication* app = QCoreApplication::instance();
    if (app == nullptr || QThread::currentThread() == app->thread())
    {
        show();
        return;
    }

    QMetaObject::invokeMethod(app, show, Qt::QueuedConnection);
}

bool showConfirmOnGuiThread(const QString& title, const QString& text)
{
    auto ask = [title, text]() -> bool
    {
        QMessageBox msgBox(nullptr);
        msgBox.setIcon(QMessageBox::Question);
        msgBox.setWindowTitle(title);
        msgBox.setText(text);
        QAbstractButton* yesButton = msgBox.addButton(QStringLiteral("是"), QMessageBox::YesRole);
        msgBox.addButton(QStringLiteral("否"), QMessageBox::NoRole);
        msgBox.exec();
        return msgBox.clickedButton() == yesButton;
    };

    QCoreApplication* app = QCoreApplication::instance();
    if (app == nullptr || QThread::currentThread() == app->thread())
    {
        return ask();
    }

    bool result = false;
    QMetaObject::invokeMethod(app, [&result, ask]()
        {
            result = ask();
        }, Qt::BlockingQueuedConnection);
    return result;
}
}

QString DecodeRobotMessageText(const std::string& text)
{
    return decodeMessageText(text);
}

QString DecodeRobotMessageText(const char* text)
{
    return decodeMessageText(text != nullptr ? std::string(text) : std::string());
}

std::string formatString(const char* format, ...) 
{
    if (format == nullptr) {
        return "";
    }

    va_list args;
    va_start(args, format);
    std::string result = formatStringV(format, args);
    va_end(args);
    return result;
}

// 格式化信息弹窗实现
void showInfoMessage(const std::string& title, const char* format, ...) 
{
    if (!m_bMessageEnable[MESSAGE_INFO])
    {
        return;
    }

    // 解析可变参数并格式化字符串
    va_list args;
    va_start(args, format);
    std::string msgStr = formatStringV(format, args);
    va_end(args);

    showMessageOnGuiThread(
        QMessageBox::Information,
        DecodeRobotMessageText(title),
        DecodeRobotMessageText(msgStr));
}

void showInfoMessage(const char* title, const char* format, ...)
{
    if (!m_bMessageEnable[MESSAGE_INFO])
    {
        return;
    }

    va_list args;
    va_start(args, format);
    std::string msgStr = formatStringV(format, args);
    va_end(args);

    showMessageOnGuiThread(
        QMessageBox::Information,
        DecodeRobotMessageText(safeTitle(title)),
        DecodeRobotMessageText(msgStr));
}

// 格式化警告弹窗实现
void showWarnMessage(const std::string& title, const char* format, ...) {
    if (!m_bMessageEnable[MESSAGE_WARN])
    {
        return;
    }

    va_list args;
    va_start(args, format);
    std::string msgStr = formatStringV(format, args);
    va_end(args);

    showMessageOnGuiThread(
        QMessageBox::Warning,
        DecodeRobotMessageText(title),
        DecodeRobotMessageText(msgStr));
}

void showWarnMessage(const char* title, const char* format, ...)
{
    if (!m_bMessageEnable[MESSAGE_WARN])
    {
        return;
    }

    va_list args;
    va_start(args, format);
    std::string msgStr = formatStringV(format, args);
    va_end(args);

    showMessageOnGuiThread(
        QMessageBox::Warning,
        DecodeRobotMessageText(safeTitle(title)),
        DecodeRobotMessageText(msgStr));
}

// 格式化错误弹窗（基础版）
void showErrorMessage(const std::string& title, const char* format, ...) {
    if (!m_bMessageEnable[MESSAGE_ERROR])
    {
        return;
    }

    va_list args;
    va_start(args, format);
    std::string msgStr = formatStringV(format, args);
    va_end(args);

    showMessageOnGuiThread(
        QMessageBox::Critical,
        DecodeRobotMessageText(title),
        DecodeRobotMessageText(msgStr));
}

void showErrorMessage(const char* title, const char* format, ...)
{
    if (!m_bMessageEnable[MESSAGE_ERROR])
    {
        return;
    }

    va_list args;
    va_start(args, format);
    std::string msgStr = formatStringV(format, args);
    va_end(args);

    showMessageOnGuiThread(
        QMessageBox::Critical,
        DecodeRobotMessageText(safeTitle(title)),
        DecodeRobotMessageText(msgStr));
}

// 格式化确认弹窗实现
bool showConfirmMessage(const std::string& title, const char* format, ...) {
    if (!m_bMessageEnable[MESSAGE_CONFIRM])
    {
        return false;
    }

    va_list args;
    va_start(args, format);
    std::string msgStr = formatStringV(format, args);
    va_end(args);

    return showConfirmOnGuiThread(
        DecodeRobotMessageText(title),
        DecodeRobotMessageText(msgStr));
}

bool showConfirmMessage(const char* title, const char* format, ...)
{
    if (!m_bMessageEnable[MESSAGE_CONFIRM])
    {
        return false;
    }

    va_list args;
    va_start(args, format);
    std::string msgStr = formatStringV(format, args);
    va_end(args);

    return showConfirmOnGuiThread(
        DecodeRobotMessageText(safeTitle(title)),
        DecodeRobotMessageText(msgStr));
}
