#pragma once

#include <QString>

#include <memory>

class QLockFile;

// 跨进程单实例锁。机器人高层互锁主要是进程内状态，因此所有会构造机器人驱动的
// GUI/CLI 入口必须先持有同一个进程锁；只读路径探针和点云提取 worker 在 main 中豁免。
class ApplicationInstanceGuard final
{
public:
    using Ptr = std::unique_ptr<ApplicationInstanceGuard>;

    static QString RobotControlScope();
    static QString MachineMutexName(const QString& scope);
    static Ptr TryAcquire(const QString& scope, QString* reason = nullptr);

    ~ApplicationInstanceGuard();

    ApplicationInstanceGuard(const ApplicationInstanceGuard&) = delete;
    ApplicationInstanceGuard& operator=(const ApplicationInstanceGuard&) = delete;

private:
    explicit ApplicationInstanceGuard(
        std::unique_ptr<QLockFile> lockFile,
        void* machineMutexHandle);

    std::unique_ptr<QLockFile> m_lockFile;
    void* m_machineMutexHandle = nullptr;
};
