#include "RobotMessage.h"

#include <QCoreApplication>
#include <QMetaObject>
#include <QPushButton>
#include <QString>
#include <QThread>

#include <functional>
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

void runOnGuiThreadBlocking(const std::function<void()>& action)
{
    QCoreApplication* app = QCoreApplication::instance();
    if (app != nullptr && QThread::currentThread() != app->thread())
    {
        QMetaObject::invokeMethod(app, [action]()
            {
                action();
            }, Qt::BlockingQueuedConnection);
        return;
    }

    action();
}

bool runConfirmOnGuiThreadBlocking(const std::function<bool()>& action)
{
    bool result = false;
    runOnGuiThreadBlocking([&result, action]()
        {
            result = action();
        });
    return result;
}

void showMessageBoxOnGuiThread(QMessageBox::Icon icon, const std::string& title, const std::string& text)
{
    runOnGuiThreadBlocking([icon, title, text]()
        {
            QMessageBox msgBox(nullptr);
            msgBox.setIcon(icon);
            msgBox.setWindowTitle(QString::fromLocal8Bit(title.c_str()));
            msgBox.setText(QString::fromUtf8(text.c_str()));
            msgBox.addButton("确定", QMessageBox::AcceptRole);
            msgBox.exec();
        });
}

bool showConfirmBoxOnGuiThread(const std::string& title, const std::string& text)
{
    return runConfirmOnGuiThreadBlocking([title, text]() -> bool
        {
            QMessageBox msgBox(nullptr);
            msgBox.setIcon(QMessageBox::Question);
            msgBox.setWindowTitle(QString::fromLocal8Bit(title.c_str()));
            msgBox.setText(QString::fromUtf8(text.c_str()));
            QPushButton* yesButton = msgBox.addButton("是", QMessageBox::YesRole);
            msgBox.addButton("否", QMessageBox::NoRole);
            msgBox.exec();
            return msgBox.clickedButton() == yesButton;
        });
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

    showMessageBoxOnGuiThread(QMessageBox::Information, title, msgStr);
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

    showMessageBoxOnGuiThread(QMessageBox::Information, safeTitle(title), msgStr);
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

    showMessageBoxOnGuiThread(QMessageBox::Warning, title, msgStr);
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

    showMessageBoxOnGuiThread(QMessageBox::Warning, safeTitle(title), msgStr);
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

    showMessageBoxOnGuiThread(QMessageBox::Critical, title, msgStr);
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

    showMessageBoxOnGuiThread(QMessageBox::Critical, safeTitle(title), msgStr);
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

    return showConfirmBoxOnGuiThread(title, msgStr);
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

    return showConfirmBoxOnGuiThread(safeTitle(title), msgStr);
}
