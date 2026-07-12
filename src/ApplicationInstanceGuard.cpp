#include "ApplicationInstanceGuard.h"

#include <QCryptographicHash>
#include <QDir>
#include <QLockFile>
#include <QStandardPaths>

#include <utility>

#ifdef Q_OS_WIN
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#endif

namespace
{
QString LockRootPath()
{
    QString tempRoot = QStandardPaths::writableLocation(QStandardPaths::TempLocation);
    if (tempRoot.trimmed().isEmpty())
    {
        tempRoot = QDir::tempPath();
    }
    if (tempRoot.trimmed().isEmpty())
    {
        return {};
    }
    return QDir(tempRoot).filePath(QStringLiteral("NoTeaching-Robot-ProcessLocks-v1"));
}

QString LockFilePath(const QString& scope)
{
    const QByteArray digest = QCryptographicHash::hash(
        scope.trimmed().toUtf8(), QCryptographicHash::Sha256).toHex();
    return QDir(LockRootPath()).filePath(QString::fromLatin1(digest) + QStringLiteral(".lock"));
}
}

QString ApplicationInstanceGuard::RobotControlScope()
{
    // 中性版、品牌版和 --no-show CLI 必须共享同一个固定 scope，不能跟随产品名变化。
    return QStringLiteral("QtWidgetsApplication4/robot-hardware-control/v1");
}

QString ApplicationInstanceGuard::MachineMutexName(const QString& scope)
{
    const QByteArray digest = QCryptographicHash::hash(
        scope.trimmed().toUtf8(), QCryptographicHash::Sha256).toHex();
    return QStringLiteral("Global\\NoTeaching-Robot-Hardware-Control-v1-")
        + QString::fromLatin1(digest);
}

ApplicationInstanceGuard::ApplicationInstanceGuard(
    std::unique_ptr<QLockFile> lockFile,
    void* machineMutexHandle)
    : m_lockFile(std::move(lockFile))
    , m_machineMutexHandle(machineMutexHandle)
{
}

ApplicationInstanceGuard::~ApplicationInstanceGuard()
{
    // 先释放辅助文件锁，再关闭机器级对象；整个析构窗口始终至少有一层锁存在。
    m_lockFile.reset();
#ifdef Q_OS_WIN
    if (m_machineMutexHandle != nullptr)
    {
        CloseHandle(static_cast<HANDLE>(m_machineMutexHandle));
        m_machineMutexHandle = nullptr;
    }
#endif
}

ApplicationInstanceGuard::Ptr ApplicationInstanceGuard::TryAcquire(
    const QString& scope,
    QString* reason)
{
    if (reason != nullptr)
    {
        reason->clear();
    }
    if (scope.trimmed().isEmpty())
    {
        if (reason != nullptr)
        {
            *reason = QStringLiteral("进程互锁 scope 为空，拒绝启动机器人控制入口。");
        }
        return {};
    }

    void* machineMutexHandle = nullptr;
#ifdef Q_OS_WIN
    const QString machineMutexName = MachineMutexName(scope);
    SetLastError(ERROR_SUCCESS);
    // 不取得 thread-affine ownership；唯一的新建句柄本身就是租约，进程异常退出时由内核回收。
    // 这样 guard 即使在另一线程析构也不会遗留一个仍被原线程拥有的 mutex。
    HANDLE machineMutex = CreateMutexW(
        nullptr,
        FALSE,
        reinterpret_cast<LPCWSTR>(machineMutexName.utf16()));
    const DWORD machineMutexError = GetLastError();
    if (machineMutex == nullptr)
    {
        if (reason != nullptr)
        {
            if (machineMutexError == ERROR_ACCESS_DENIED)
            {
                *reason = QStringLiteral(
                    "无法访问机器级机器人控制锁（可能由另一 Windows 用户/会话持有），已安全中止：%1")
                    .arg(machineMutexName);
            }
            else
            {
                *reason = QStringLiteral("无法创建机器级机器人控制锁，已安全中止：%1（Windows错误=%2）")
                    .arg(machineMutexName)
                    .arg(machineMutexError);
            }
        }
        return {};
    }
    if (machineMutexError == ERROR_ALREADY_EXISTS)
    {
        CloseHandle(machineMutex);
        if (reason != nullptr)
        {
            *reason = QStringLiteral(
                "已有另一个 Windows 用户、会话或程序进程持有机器级机器人控制权。"
                "请先在原进程停止机器人流程并退出，再重试。");
        }
        return {};
    }
    machineMutexHandle = machineMutex;
#endif

    const QString lockRoot = LockRootPath();
    if (lockRoot.isEmpty() || !QDir().mkpath(lockRoot))
    {
#ifdef Q_OS_WIN
        CloseHandle(static_cast<HANDLE>(machineMutexHandle));
#endif
        if (reason != nullptr)
        {
            *reason = QStringLiteral("无法创建机器人控制进程锁目录，已安全中止：%1")
                .arg(QDir::toNativeSeparators(lockRoot));
        }
        return {};
    }

    auto lockFile = std::make_unique<QLockFile>(LockFilePath(scope));
    // 不按文件年龄强删；QLockFile 仍会用 PID/主机信息识别已退出进程的真正 stale 锁。
    lockFile->setStaleLockTime(0);
    if (!lockFile->tryLock(0))
    {
#ifdef Q_OS_WIN
        CloseHandle(static_cast<HANDLE>(machineMutexHandle));
#endif
        if (reason != nullptr)
        {
            qint64 pid = 0;
            QString hostName;
            QString appName;
            const bool hasOwner = lockFile->getLockInfo(&pid, &hostName, &appName);
            if (lockFile->error() == QLockFile::PermissionError)
            {
                *reason = QStringLiteral("无法访问机器人控制进程锁，已安全中止：%1")
                    .arg(QDir::toNativeSeparators(lockFile->fileName()));
            }
            else if (hasOwner)
            {
                *reason = QStringLiteral(
                    "已有另一个程序进程持有机器人控制权（PID=%1，主机=%2，程序=%3）。"
                    "请先在原进程停止机器人流程并退出，再重试。")
                    .arg(pid)
                    .arg(hostName, appName);
            }
            else
            {
                *reason = QStringLiteral(
                    "已有另一个程序进程持有机器人控制权。请先在原进程停止机器人流程并退出，再重试。");
            }
        }
        return {};
    }

    return Ptr(new ApplicationInstanceGuard(
        std::move(lockFile), machineMutexHandle));
}
