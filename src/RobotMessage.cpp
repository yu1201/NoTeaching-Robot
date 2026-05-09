#include "RobotMessage.h"

#include <QCoreApplication>
#include <QMetaObject>
#include <QPushButton>
#include <QString>
#include <QThread>
#include <vector>

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

void showMessageOnGuiThread(QMessageBox::Icon icon, const QString& title, const QString& text)
{
    auto show = [icon, title, text]()
    {
        QMessageBox msgBox(nullptr);
        msgBox.setIcon(icon);
        msgBox.setWindowTitle(title);
        msgBox.setText(text);
        msgBox.addButton("确定", QMessageBox::AcceptRole);
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
        QAbstractButton* yesButton = msgBox.addButton("是", QMessageBox::YesRole);
        msgBox.addButton("否", QMessageBox::NoRole);
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
        QString::fromLocal8Bit(title.c_str()),
        QString::fromUtf8(msgStr.c_str()));
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
        QString::fromLocal8Bit(safeTitle(title).c_str()),
        QString::fromUtf8(msgStr.c_str()));
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
        QString::fromLocal8Bit(title.c_str()),
        QString::fromUtf8(msgStr.c_str()));
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
        QString::fromLocal8Bit(safeTitle(title).c_str()),
        QString::fromUtf8(msgStr.c_str()));
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
        QString::fromLocal8Bit(title.c_str()),
        QString::fromUtf8(msgStr.c_str()));
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
        QString::fromLocal8Bit(safeTitle(title).c_str()),
        QString::fromUtf8(msgStr.c_str()));
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
        QString::fromLocal8Bit(title.c_str()),
        QString::fromUtf8(msgStr.c_str()));
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
        QString::fromLocal8Bit(safeTitle(title).c_str()),
        QString::fromUtf8(msgStr.c_str()));
}
