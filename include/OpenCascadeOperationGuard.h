#pragma once

#include <QMutex>

// OCCT STEP 读取/传输会访问进程级注册表和全局控制器状态。
// CAD 外显与后台网格转换共用同一把锁，避免两个 reader 并发初始化或传输。
namespace occtsync
{
inline QMutex& OperationMutex()
{
    static QMutex mutex;
    return mutex;
}
}
