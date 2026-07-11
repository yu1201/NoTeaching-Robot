#pragma once

#include <QString>
#include <QStringList>

// 进程级路径真源：安装/资源根与现场可写数据根分离。
// main() 在构造任何窗口、配置库或设备对象前完成初始化。
class AppPaths
{
public:
    static bool Initialize(const QStringList& arguments, QString* error = nullptr);
    static bool IsInitialized();

    static QString OriginalWorkingDirectory();
    static QString InstallRootPath();
    static QString DataRootPath();
    static bool HasExplicitDataRoot();

    // Data/Result/Log/Temp/Job 等现场可写内容。
    static QString WritablePath(const QString& relativePath);
    // 将不可信文件名作为单一子项拼入可写目录；拒绝分隔符、盘符、ADS、设备名和越界。
    static bool IsSafePathComponent(const QString& component);
    static QString WritableChildPath(const QString& relativeDirectory, const QString& component);
    // SDK/Tools/branding/translations 等随程序交付的只读资源。
    static QString ResourcePath(const QString& relativePath);
    // 只在可信安装根和 exe 目录查找可执行资源，禁止 data-root/cwd 覆盖 SDK、DLL、TP 等。
    static QString FindResourcePath(const QString& relativePath);
    // CLI 显式相对输入/输出保持相对启动目录；不受随后固定运行 cwd 影响。
    static QString CommandLinePath(const QString& path);

private:
    AppPaths() = delete;
};
