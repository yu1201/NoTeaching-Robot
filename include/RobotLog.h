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
#include <filesystem> // 日志按天分目录归档
#include <mutex>    // 写入线程安全
#include <chrono>   // 毫秒时间戳
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

    // 线程安全地写入一整行：自动加 [yyyy-MM-dd HH:mm:ss.zzz] 时间戳与换行。
    // 不走 printf，故不受 % 转义/1024 字节截断影响——供 Qt 侧（QString 文本、长行、多线程）统一接入。
    void writeLine(const std::string& text);

    // 析构函数
    ~RobotLog();

private:
    // 生成时间戳（内部工具函数）
    std::string getTimestamp();

    // 生成带毫秒的时间戳 yyyy-MM-dd HH:mm:ss.zzz（供 writeLine 使用）
    std::string getTimestampMs();

    // 控制台彩色输出（内部工具函数）
    void printWithColor(const std::string& msg, LogColor color);

    // 内部辅助函数：处理可变参数（避免代码冗余）
    void writeImpl(LogColor color, const char* format, va_list args);

    // 按天归档：在 m_logDir 下创建 day(yyyy-MM-dd) 子目录并打开日志文件（跨天时重开）
    void openForDay(const std::string& day);

    // 跨天则关闭旧文件并切换到新日期目录（调用方须已持有 m_mutex）
    void switchDayIfNeeded(const std::string& day);

    // 成员变量（私有化）
    std::ofstream m_logFile;    // 日志文件流
    std::string m_logPath;      // 实际日志文件路径（含日期子目录）
    bool m_showConsole;         // 是否输出到控制台
    std::string m_logDir;       // 原始路径的目录部分（不含日期）
    std::string m_fileName;     // 日志文件名
    std::string m_currentDay;   // 当前归档日期 yyyy-MM-dd
    std::mutex m_mutex;         // 保护文件写入与跨天切换
};

#endif // SIMPLELOG_H