#include "RobotLog.h"
// Windows平台颜色所需头文件

// 构造函数实现
RobotLog::RobotLog(const std::string& logPath, bool showConsole)
    : m_logPath(logPath), m_showConsole(showConsole)
{
    // 日志按天归档：在原路径的目录下插入 yyyy-MM-dd 子目录，运行中跨天自动切换。
    try {
        std::filesystem::path p(logPath);
        m_fileName = p.filename().string();
        m_logDir = p.parent_path().string();
    } catch (...) {
        m_fileName = logPath;
        m_logDir.clear();
    }
    if (m_fileName.empty()) {
        m_fileName = "RobotRunLog.txt";
    }
    m_currentDay = getTimestamp().substr(0, 10);  // yyyy-MM-dd
    openForDay(m_currentDay);
}

// 在 m_logDir/<day>/ 下打开日志文件（目录不存在则创建）
void RobotLog::openForDay(const std::string& day)
{
    std::filesystem::path datedDir = m_logDir.empty()
        ? (std::filesystem::path("Log") / day)
        : (std::filesystem::path(m_logDir) / day);
    std::error_code ec;
    std::filesystem::create_directories(datedDir, ec);
    m_logPath = (datedDir / m_fileName).string();
    m_logFile.open(m_logPath, std::ios::out | std::ios::app);
    if (!m_logFile.is_open()) {
        std::cerr << "日志文件打开失败：" << m_logPath << std::endl;
    }
}

// 跨天则切换到新日期目录（调用方须已持有 m_mutex）。
// 仅在新目录打开成功后才提交 m_currentDay；若打开失败则保持原值，下次写入会再次进入并重试，
// 避免一次瞬时失败（磁盘满/权限）导致 m_currentDay 提前推进、当天剩余日志被静默丢弃。
void RobotLog::switchDayIfNeeded(const std::string& day)
{
    if (day == m_currentDay && m_logFile.is_open()) return;
    if (m_logFile.is_open()) m_logFile.close();
    openForDay(day);
    if (m_logFile.is_open()) {
        m_currentDay = day;
    }
}

// 内部辅助函数：处理可变参数核心逻辑
void RobotLog::writeImpl(LogColor color, const char* format, va_list args)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    // 生成时间戳并按天切换归档目录（跨天自动重开到新日期文件夹）
    std::string timestamp = getTimestamp();
    switchDayIfNeeded(timestamp.substr(0, 10));  // yyyy-MM-dd

    if (!m_logFile.is_open()) return;

    // 1. 格式化字符串
    char buffer[1024]; // 日志缓冲区（可按需调整大小）
    vsnprintf(buffer, sizeof(buffer), format, args);

    // 2. 组装日志行
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

// 线程安全写入整行：自动加毫秒时间戳前缀，不走 printf（避免 % 转义与 1024 截断）
void RobotLog::writeLine(const std::string& text)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::string ts = getTimestampMs();
    switchDayIfNeeded(ts.substr(0, 10));  // yyyy-MM-dd
    if (!m_logFile.is_open()) return;
    m_logFile << '[' << ts << "] " << text << '\n';
    m_logFile.flush();
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

// 带毫秒的时间戳 yyyy-MM-dd HH:mm:ss.zzz
std::string RobotLog::getTimestampMs() {
    using namespace std::chrono;
    auto now = system_clock::now();
    auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;
    std::time_t t = system_clock::to_time_t(now);
    std::tm tm{};
    localtime_s(&tm, &t);
    std::stringstream ss;
    ss << std::setfill('0')
        << (tm.tm_year + 1900) << "-"
        << std::setw(2) << (tm.tm_mon + 1) << "-"
        << std::setw(2) << tm.tm_mday << " "
        << std::setw(2) << tm.tm_hour << ":"
        << std::setw(2) << tm.tm_min << ":"
        << std::setw(2) << tm.tm_sec << "."
        << std::setw(3) << static_cast<int>(ms.count());
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