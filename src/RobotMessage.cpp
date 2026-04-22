#include "RobotMessage.h"

#include <QString>
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

    QMessageBox msgBox(nullptr);
    msgBox.setIcon(QMessageBox::Information);
    msgBox.setWindowTitle(QString::fromLocal8Bit(title.c_str()));
    msgBox.setText(QString::fromUtf8(msgStr.c_str()));
    msgBox.addButton("确定", QMessageBox::AcceptRole);
    msgBox.exec();
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

    QMessageBox msgBox(nullptr);
    msgBox.setIcon(QMessageBox::Information);
    msgBox.setWindowTitle(QString::fromLocal8Bit(safeTitle(title).c_str()));
    msgBox.setText(QString::fromUtf8(msgStr.c_str()));
    msgBox.addButton("确定", QMessageBox::AcceptRole);
    msgBox.exec();
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

    QMessageBox msgBox(nullptr);
    msgBox.setIcon(QMessageBox::Warning);
    msgBox.setWindowTitle(QString::fromLocal8Bit(title.c_str()));
    msgBox.setText(QString::fromUtf8(msgStr.c_str()));
    msgBox.addButton("确定", QMessageBox::AcceptRole);
    msgBox.exec();
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

    QMessageBox msgBox(nullptr);
    msgBox.setIcon(QMessageBox::Warning);
    msgBox.setWindowTitle(QString::fromLocal8Bit(safeTitle(title).c_str()));
    msgBox.setText(QString::fromUtf8(msgStr.c_str()));
    msgBox.addButton("确定", QMessageBox::AcceptRole);
    msgBox.exec();
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

    QMessageBox msgBox(nullptr);
    msgBox.setIcon(QMessageBox::Critical);
    msgBox.setWindowTitle(QString::fromLocal8Bit(title.c_str()));
    msgBox.setText(QString::fromUtf8(msgStr.c_str()));
    msgBox.addButton("确定", QMessageBox::AcceptRole);
    msgBox.exec();
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

    QMessageBox msgBox(nullptr);
    msgBox.setIcon(QMessageBox::Critical);
    msgBox.setWindowTitle(QString::fromLocal8Bit(safeTitle(title).c_str()));
    msgBox.setText(QString::fromUtf8(msgStr.c_str()));
    msgBox.addButton("确定", QMessageBox::AcceptRole);
    msgBox.exec();
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

    QMessageBox msgBox(nullptr);
    msgBox.setIcon(QMessageBox::Question);
    msgBox.setWindowTitle(QString::fromLocal8Bit(title.c_str()));
    msgBox.setText(QString::fromUtf8(msgStr.c_str()));
    msgBox.addButton("是", QMessageBox::YesRole);
    msgBox.addButton("否", QMessageBox::NoRole);
    return (msgBox.exec() == QMessageBox::Yes);
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

    QMessageBox msgBox(nullptr);
    msgBox.setIcon(QMessageBox::Question);
    msgBox.setWindowTitle(QString::fromLocal8Bit(safeTitle(title).c_str()));
    msgBox.setText(QString::fromUtf8(msgStr.c_str()));
    msgBox.addButton("是", QMessageBox::YesRole);
    msgBox.addButton("否", QMessageBox::NoRole);
    return (msgBox.exec() == QMessageBox::Yes);
}
