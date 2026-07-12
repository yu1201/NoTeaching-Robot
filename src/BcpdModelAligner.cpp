#include "BcpdModelAligner.h"

#include "AppPaths.h"
#include "RobotDataHelper.h"

#include <QByteArray>
#include <QCoreApplication>
#include <QDir>
#include <QElapsedTimer>
#include <QFile>
#include <QFileInfo>
#include <QProcess>
#include <QStringList>
#include <QTemporaryDir>

#include <algorithm>
#include <cstdlib>

namespace
{
// QTemporaryDir 的存续期必须严格长于子进程。第一次有界等待保持
// 取消 UI 反馈；若句柄退出延迟，再次 kill 并无限等待 NotRunning。
// 宁可在异常操作系统状态下停在这里，也不能让仍在写的 bcpd.exe
// 与 QTemporaryDir 析构/删除临时文件竞争。本函数不分配内存。
bool TerminateProcessAndConfirmStopped(QProcess& process) noexcept
{
    if (process.state() == QProcess::NotRunning) return true;

    process.kill();
    if (process.waitForFinished(2000)
        && process.state() == QProcess::NotRunning)
        return true;

    process.kill();
    while (process.state() != QProcess::NotRunning)
    {
        // Windows 的 TerminateProcess 返回后句柄可能尚未 signal；-1 确保
        // QProcess 已观察到真正退出。若平台层短暂返回 false，
        // 重新 kill 后继续等，绝不携活子进程返回。
        if (!process.waitForFinished(-1)
            && process.state() != QProcess::NotRunning)
            process.kill();
    }
    return true;
}

// 点云（可降采样）→ BCPD 输入矩阵：每行 "x y z"，空格分隔、无表头。
bool WriteCloudMatrix(const QString& path,
                      const QVector<RobotCalculation::IndexedPoint3D>& pts,
                      int stride, int& written,
                      const std::function<bool()>& isCancelled)
{
    QFile f(path);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate)) return false;
    if (stride < 1) stride = 1;
    QByteArray buf;
    buf.reserve(1 << 20);
    written = 0;
    for (int i = 0; i < pts.size(); i += stride)
    {
        if ((written & 0x3ff) == 0 && isCancelled && isCancelled()) return false;
        const Eigen::Vector3d& p = pts[i].point;
        buf += QByteArray::number(p.x(), 'f', 4);
        buf += ' ';
        buf += QByteArray::number(p.y(), 'f', 4);
        buf += ' ';
        buf += QByteArray::number(p.z(), 'f', 4);
        buf += '\n';
        ++written;
        if (buf.size() >= (4 << 20))
        {
            if (f.write(buf) != buf.size()) return false;
            buf.clear();
        }
    }
    if (!buf.isEmpty() && f.write(buf) != buf.size()) return false;
    f.close();
    return written > 0;
}

// 网格顶点 → BCPD 输入矩阵（source Y）。
bool WriteMeshVertices(const QString& path, const WorkpieceMeshBuilder::Mesh& mesh,
                       const std::function<bool()>& isCancelled)
{
    QFile f(path);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate)) return false;
    QByteArray buf;
    buf.reserve(1 << 20);
    for (qsizetype index = 0; index < mesh.vertices.size(); ++index)
    {
        if ((index & 0x3ff) == 0 && isCancelled && isCancelled()) return false;
        const Eigen::Vector3f& v = mesh.vertices.at(index);
        buf += QByteArray::number(v.x(), 'f', 4);
        buf += ' ';
        buf += QByteArray::number(v.y(), 'f', 4);
        buf += ' ';
        buf += QByteArray::number(v.z(), 'f', 4);
        buf += '\n';
        if (buf.size() >= (4 << 20))
        {
            if (f.write(buf) != buf.size()) return false;
            buf.clear();
        }
    }
    if (!buf.isEmpty() && f.write(buf) != buf.size()) return false;
    f.close();
    return mesh.vertices.size() > 0;
}

// 读 BCPD 形变后顶点（M×3 文本）。手写 strtod 解析，比正则快。
bool ReadDeformedVertices(const QString& path, QVector<Eigen::Vector3f>& out, int reserveHint,
                          const std::function<bool()>& isCancelled)
{
    constexpr qint64 kMaximumBytesPerVertex = 192;
    if (reserveHint <= 0) return false;
    const QFileInfo info(path);
    const qint64 maximumBytes = std::min<qint64>(
        1024LL * 1024LL * 1024LL,
        static_cast<qint64>(reserveHint) * kMaximumBytesPerVertex + 1024);
    if (!info.isFile() || info.size() <= 0 || info.size() > maximumBytes) return false;
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly)) return false;
    out.clear();
    if (reserveHint > 0) out.reserve(reserveHint);
    while (!f.atEnd())
    {
        if ((out.size() & 0x3ff) == 0 && isCancelled && isCancelled()) return false;
        if (out.size() >= reserveHint) return false;
        const QByteArray line = f.readLine(512);
        if (line.isEmpty() && f.error() != QFileDevice::NoError) return false;
        if (line.size() >= 511 && !line.endsWith('\n')) return false;
        const QList<QByteArray> fields = line.simplified().split(' ');
        if (fields.size() != 3) return false;
        bool okX = false, okY = false, okZ = false;
        const double x = fields.at(0).toDouble(&okX);
        const double y = fields.at(1).toDouble(&okY);
        const double z = fields.at(2).toDouble(&okZ);
        if (!okX || !okY || !okZ || !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
            return false;
        const Eigen::Vector3f vertex(
            static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
        if (!vertex.allFinite()) return false;  // finite double 仍可在窄化成 float 时溢出
        out.push_back(vertex);
    }
    return f.error() == QFileDevice::NoError && out.size() == reserveHint;
}
}  // namespace

QString BcpdModelAligner::DefaultExePath()
{
    // 优先工程根/SDK/BCPD（运行时工作目录已切到工程根，与其他 SDK 定位一致）；
    // 回退 exe 同目录（部署目录无 .sln 时工作目录即 exe 目录，SDK 随包在此）。
    const QString rootPath = RobotDataHelper::FindProjectFilePath(QStringLiteral("SDK/BCPD/bcpd.exe"));
    if (QFileInfo::exists(rootPath)) return rootPath;
    return QDir(QCoreApplication::applicationDirPath()).filePath(QStringLiteral("SDK/BCPD/bcpd.exe"));
}

bool BcpdModelAligner::IsAvailable(const QString& bcpdExePath)
{
    const QString exe = bcpdExePath.isEmpty() ? DefaultExePath() : bcpdExePath;
    return QFileInfo::exists(exe);
}

bool BcpdModelAligner::DeformModelToCloud(
    const QVector<RobotCalculation::IndexedPoint3D>& cloud,
    const WorkpieceMeshBuilder::Mesh& model,
    WorkpieceMeshBuilder::Mesh& outDeformedMesh,
    const Options& options, const LogCallback& log)
{
    auto logmsg = [&](const QString& m) { if (log) log(m); };
    const auto isCancelled = [&options]()
        {
            return options.cancelRequested && options.cancelRequested();
        };

    if (isCancelled())
    {
        logmsg(QStringLiteral("BCPD：任务已取消，未启动配准"));
        return false;
    }

    const QString exe = options.bcpdExePath.isEmpty() ? DefaultExePath() : options.bcpdExePath;
    if (!QFileInfo::exists(exe))
    {
        logmsg(QStringLiteral("BCPD：未找到 bcpd.exe（%1），跳过配准").arg(exe));
        return false;
    }
    if (!model.IsValid() || model.vertices.size() < 4)
    {
        logmsg(QStringLiteral("BCPD：模型无效，跳过配准"));
        return false;
    }
    if (cloud.size() < 10)
    {
        logmsg(QStringLiteral("BCPD：点云过少（%1），跳过配准").arg(cloud.size()));
        return false;
    }

    const QString tempRoot = AppPaths::WritablePath(QStringLiteral("Temp/BCPD"));
    if (tempRoot.isEmpty() || !QDir().mkpath(tempRoot))
    {
        logmsg(QStringLiteral("BCPD：临时根目录创建失败（%1），跳过配准")
            .arg(QDir::toNativeSeparators(tempRoot)));
        return false;
    }
    QTemporaryDir tmp(QDir(tempRoot).filePath(QStringLiteral("bcpd_XXXXXX")));
    if (!tmp.isValid())
    {
        logmsg(QStringLiteral("BCPD：临时目录创建失败，跳过配准"));
        return false;
    }
    const QString xFile = tmp.filePath(QStringLiteral("x_cloud.txt"));  // target（点云，固定）
    const QString yFile = tmp.filePath(QStringLiteral("y_model.txt"));  // source（模型，被形变）
    const QString prefix = tmp.filePath(QStringLiteral("bcpd_"));
    const QString outY = prefix + QStringLiteral("y.txt");

    // 点云降采样（仅用于配准定形；去噪仍用全分辨率点云，不受此影响）。
    int stride = 1;
    if (options.targetSampleCap > 0 && cloud.size() > options.targetSampleCap)
        stride = (cloud.size() + options.targetSampleCap - 1) / options.targetSampleCap;
    int xWritten = 0;
    if (!WriteCloudMatrix(xFile, cloud, stride, xWritten, isCancelled))
    {
        logmsg(QStringLiteral("BCPD：写点云临时文件失败，跳过配准"));
        return false;
    }
    if (!WriteMeshVertices(yFile, model, isCancelled))
    {
        logmsg(QStringLiteral("BCPD：写模型临时文件失败，跳过配准"));
        return false;
    }

    QStringList args;
    args << QStringLiteral("-x") << QDir::toNativeSeparators(xFile)
         << QStringLiteral("-y") << QDir::toNativeSeparators(yFile)
         << QStringLiteral("-Tsrn")  // 相似 + 非刚性 sR(y+v)+t
         << QStringLiteral("-o") << QDir::toNativeSeparators(prefix)
         << QStringLiteral("-sy")    // 输出形变后 source
         << QStringLiteral("-w") << QString::number(options.omega, 'g', 4);
    if (options.lambda > 0.0)
        args << QStringLiteral("-l") << QString::number(options.lambda, 'g', 4);
    if (options.beta > 0.0)
        args << QStringLiteral("-b") << QString::number(options.beta, 'g', 4);
    // -A 内部 Nyström 采样数默认 70，要求 source 顶点数 > 70，否则 bcpd 报错中止。
    if (options.useAcceleration && model.vertices.size() > 100)
        args << QStringLiteral("-A");

    QElapsedTimer timer;
    timer.start();
    QProcess proc;
    proc.setProcessChannelMode(QProcess::MergedChannels);
    proc.setWorkingDirectory(tmp.path());
    proc.start(exe, args);
    if (!proc.waitForStarted(10000))
    {
        TerminateProcessAndConfirmStopped(proc);
        logmsg(QStringLiteral("BCPD：进程启动失败，跳过配准"));
        return false;
    }
    const int timeoutMs = std::max(1, options.timeoutMs);
    bool timedOut = false;
    QByteArray outputTail;
    const auto drainOutput = [&]()
    {
        outputTail.append(proc.readAll());
        constexpr qsizetype kMaximumDiagnosticBytes = 8192;
        if (outputTail.size() > kMaximumDiagnosticBytes)
        {
            outputTail.remove(0, outputTail.size() - kMaximumDiagnosticBytes);
        }
    };
    while (proc.state() != QProcess::NotRunning)
    {
        if (isCancelled())
        {
            TerminateProcessAndConfirmStopped(proc);
            drainOutput();
            logmsg(QStringLiteral("BCPD：任务已取消，子进程已终止"));
            return false;
        }
        const int remainingMs = timeoutMs - static_cast<int>(timer.elapsed());
        if (remainingMs <= 0)
        {
            timedOut = true;
            break;
        }
        proc.waitForFinished(std::min(100, remainingMs));
        drainOutput();
    }
    drainOutput();
    if (timedOut)
    {
        TerminateProcessAndConfirmStopped(proc);
        drainOutput();
        logmsg(QStringLiteral("BCPD：配准超时（>%1ms），跳过").arg(timeoutMs));
        return false;
    }
    if (proc.exitStatus() != QProcess::NormalExit || proc.exitCode() != 0)
    {
        const QString tail = QString::fromLocal8Bit(outputTail).trimmed().right(300);
        logmsg(QStringLiteral("BCPD：进程异常退出（code=%1）：%2").arg(proc.exitCode()).arg(tail));
        return false;
    }

    QVector<Eigen::Vector3f> deformed;
    if (!ReadDeformedVertices(outY, deformed, model.vertices.size(), isCancelled))
    {
        logmsg(QStringLiteral("BCPD：读形变结果失败（%1）").arg(outY));
        return false;
    }
    if (deformed.size() != model.vertices.size())
    {
        logmsg(QStringLiteral("BCPD：形变顶点数(%1)≠模型顶点数(%2)，弃用")
                   .arg(deformed.size()).arg(model.vertices.size()));
        return false;
    }

    // 形变后网格 = 模型拓扑 + 形变后顶点；法线沿用原模型（点到面距离去噪不依赖精确法线）。
    outDeformedMesh.vertices = deformed;
    outDeformedMesh.normals = model.normals;
    outDeformedMesh.indices = model.indices;

    logmsg(QStringLiteral("BCPD 配准完成：点云 %1（采样 %2）↦ 模型 %3 顶点形变贴合，耗时 %4ms")
               .arg(cloud.size()).arg(xWritten).arg(deformed.size()).arg(timer.elapsed()));
    return true;
}
