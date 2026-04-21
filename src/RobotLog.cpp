#include "RobotLog.h"
// Windows平台颜色所需头文件

// 构造函数实现
RobotLog::RobotLog(const std::string& logPath, bool showConsole)
    : m_logPath(logPath), m_showConsole(showConsole) 
{
    // 打开日志文件（追加模式，不存在则创建）
    m_logFile.open(logPath, std::ios::out | std::ios::app);
    if (!m_logFile.is_open()) {
        std::cerr << "日志文件打开失败：" << logPath << std::endl;
    }
}

// 内部辅助函数：处理可变参数核心逻辑
void RobotLog::writeImpl(LogColor color, const char* format, va_list args) 
{
    if (!m_logFile.is_open()) return;

    // 1. 格式化字符串
    char buffer[1024]; // 日志缓冲区（可按需调整大小）
    vsnprintf(buffer, sizeof(buffer), format, args);

    // 2. 生成时间戳
    std::string timestamp = getTimestamp();
    std::string logLine = "[" + timestamp + "] " + std::string(buffer) + "\n";

    // 3. 控制台彩色输出
    if (m_showConsole) {
        printWithColor(logLine, color);
    }

    // 4. 写入日志文件并刷新
    m_logFile << logLine;
    m_logFile.flush();
}

// 重载1：默认颜色（调用内部辅助函数）
void RobotLog::write(const char* format, ...) 
{
    va_list args;
    va_start(args, format);
    writeImpl(LogColor::DEFAULT, format, args);
    va_end(args);
}

// 重载2：指定颜色（调用内部辅助函数）
void RobotLog::write(LogColor color, const char* format, ...) 
{
    va_list args;
    va_start(args, format);
    writeImpl(color, format, args);
    va_end(args);
}

// 析构函数实现
RobotLog::~RobotLog() 
{
    if (m_logFile.is_open()) 
    {
        m_logFile.close();
    }
}

std::string RobotLog::getTimestamp() {
    std::time_t now = std::time(nullptr);
    std::tm tm{}; // 初始化空结构体
    localtime_s(&tm, &now); // 替换localtime为localtime_s
    std::stringstream ss;
    ss << std::setfill('0') << (tm.tm_year + 1900) << "-"
        << std::setw(2) << (tm.tm_mon + 1) << "-"
        << std::setw(2) << tm.tm_mday << " "
        << std::setw(2) << tm.tm_hour << ":"
        << std::setw(2) << tm.tm_min << ":"
        << std::setw(2) << tm.tm_sec;
    return ss.str();
}

// 控制台彩色输出实现（映射语义化枚举到颜色）
void RobotLog::printWithColor(const std::string& msg, LogColor color) 
{
#ifdef _WIN32
    // Windows控制台颜色设置
    HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
    switch (color) {
    case LogColor::DEFAULT:  SetConsoleTextAttribute(hConsole, 7); break;  // 白色
    case LogColor::SUCCESS:  SetConsoleTextAttribute(hConsole, 2); break;  // 绿色
    case LogColor::WARNING:  SetConsoleTextAttribute(hConsole, 6); break;  // 黄色
    case LogColor::ERR:    SetConsoleTextAttribute(hConsole, 4); break;  // 红色
    }
    std::cout << msg;
    SetConsoleTextAttribute(hConsole, 7); // 恢复默认颜色
#else
    // Linux/Mac ANSI颜色码
    switch (color) {
    case LogColor::DEFAULT:  std::cout << "\033[0m"; break;    // 默认（白色）
    case LogColor::SUCCESS:  std::cout << "\033[32m"; break;   // 绿色
    case LogColor::WARNING:  std::cout << "\033[33m"; break;   // 黄色
    case LogColor::ERROR:    std::cout << "\033[31m"; break;   // 红色
    }
    std::cout << msg << "\033[0m"; // 恢复默认
#endif
}