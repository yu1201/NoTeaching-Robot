#pragma once
#ifndef ROBOTLOG_H
#define ROBOTLOG_H

#include <string>
#include <iostream>
#include <fstream>
#include <ctime>
#include <iomanip>
#include <cstdio>   // vsnprintf
#include <cstdarg>  // va_list
#include <sstream>  // 用于 std::stringstream
#include <windows.h>

// 颜色枚举（常用4种）
enum class LogColor 
{
    DEFAULT,   // 默认（白色）
    SUCCESS,   // 成功/信息（绿色）
    WARNING,   // 警告（黄色）
    ERR      // 错误（红色）
};

// 机器人Log类声明
class RobotLog 
{
public:
    // 构造函数：指定日志文件路径，默认开启控制台输出
    RobotLog(const std::string& logPath, bool showConsole = true);

    // 核心接口：格式化输出 + 颜色默认参数（放在最后）
    void write(const char* format, ...);                          // 默认颜色
    void write(LogColor color, const char* format, ...);          // 指定颜色

    // 析构函数
    ~RobotLog();

private:
    // 生成时间戳（内部工具函数）
    std::string getTimestamp();

    // 控制台彩色输出（内部工具函数）
    void printWithColor(const std::string& msg, LogColor color);

    // 内部辅助函数：处理可变参数（避免代码冗余）
    void writeImpl(LogColor color, const char* format, va_list args);

    // 成员变量（私有化）
    std::ofstream m_logFile;    // 日志文件流
    std::string m_logPath;      // 日志文件路径
    bool m_showConsole;         // 是否输出到控制台
};

#endif // SIMPLELOG_H