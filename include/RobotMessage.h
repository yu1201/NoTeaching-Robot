#pragma once
#ifndef ROBOT_MESSAGE_UTILS_H
#define ROBOT_MESSAGE_UTILS_H

#include <QMessageBox>
#include <QInputDialog>
#include <QWidget>
#include <cstdarg>
#include <string>

//class Message
//{
//public:
//	Message();
//	~Message();

    // 弹窗类型枚举
    enum E_MESSAGE_TYPE {
        MESSAGE_INFO = 0,        // 普通信息弹窗
        MESSAGE_WARN = 1,        // 警告弹窗
        MESSAGE_ERROR = 2,       // 错误弹窗
        MESSAGE_CONFIRM = 3,     // 确认弹窗
        MESSAGE_INPUT = 4,       // 输入弹窗
        MESSAGE_TYPE_MAX = 5     // 弹窗类型总数
    };


	// 全局控制变量（来自Const.h）
    inline bool m_bMessageEnable[MESSAGE_TYPE_MAX] = {
        true,  // MESSAGE_INFO
        true,  // MESSAGE_WARN
        true,  // MESSAGE_ERROR
        true,  // MESSAGE_CONFIRM
        true,  // MESSAGE_INPUT
    };
	// 格式化字符串函数（来自Const.cpp）
	std::string formatString(const char* format, ...);
    /**
     * @brief 格式化信息弹窗
     * @param parent 父窗口
     * @param title 标题
     * @param format 格式化字符串（如："坐标X超出范围：%.2lf"）
     * @param ... 可变参数（匹配format的占位符）
     * @param bForceShow 强制显示（忽略全局控制）
     */
    void showInfoMessage(const std::string& title, const char* format, ...);
    void showInfoMessage(const char* title, const char* format, ...);

    /**
     * @brief 格式化警告弹窗
     * @param parent 父窗口
     * @param title 标题
     * @param format 格式化字符串
     * @param ... 可变参数
     * @param bForceShow 强制显示（忽略全局控制）
     */
    void showWarnMessage(const std::string& title, const char* format, ...);
    void showWarnMessage(const char* title, const char* format, ...);

    /**
     * @brief 格式化错误弹窗
     * @param parent 父窗口
     * @param title 标题
     * @param format 格式化字符串
     * @param ... 可变参数
     * @param detailedFormat 详细信息格式化字符串（可为空）
     * @param ... 详细信息可变参数
     * @param bForceShow 强制显示（忽略全局控制）
     */
    void showErrorMessage(const std::string& title, const char* format, ...);
    void showErrorMessage(const char* title, const char* format, ...);

    /**
     * @brief 格式化确认弹窗
     * @param parent 父窗口
     * @param title 标题
     * @param format 格式化字符串
     * @param ... 可变参数
     * @param bForceShow 强制显示（忽略全局控制）
     * @return true-点击是，false-点击否/关闭/弹窗被禁用
     */
    bool showConfirmMessage(const std::string& title, const char* format, ...);
    bool showConfirmMessage(const char* title, const char* format, ...);

//private:

//};




#endif
