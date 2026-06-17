#include "BcpdModelAligner.h"

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

#include <cstdlib>

namespace
{
// 点云（可降采样）→ BCPD 输入矩阵：每行 "x y z"，空格分隔、无表头。
bool WriteCloudMatrix(const QString& path,
                      const QVector<RobotCalculation::IndexedPoint3D>& pts,
                      int stride, int& written)
{
    QFile f(path);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate)) return false;
    if (stride < 1) stride = 1;
    QByteArray buf;
    buf.reserve(1 << 20);
    written = 0;
    for (int i = 0; i < pts.size(); i += stride)
    {
        const Eigen::Vector3d& p = pts[i].point;
        buf += QByteArray::number(p.x(), 'f', 4);
        buf += ' ';
        buf += QByteArray::number(p.y(), 'f', 4);
        buf += ' ';
        buf += QByteArray::number(p.z(), 'f', 4);
        buf += '\n';
        ++written;
        if (buf.size() >= (4 << 20)) { f.write(buf); buf.clear(); }
    }
    if (!buf.isEmpty()) f.write(buf);
    f.close();
    return written > 0;
}

// 网格顶点 → BCPD 输入矩阵（source Y）。
bool WriteMeshVertices(const QString& path, const WorkpieceMeshBuilder::Mesh& mesh)
{
    QFile f(path);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate)) return false;
    QByteArray buf;
    buf.reserve(1 << 20);
    for (const Eigen::Vector3f& v : mesh.vertices)
    {
        buf += QByteArray::number(v.x(), 'f', 4);
        buf += ' ';
        buf += QByteArray::number(v.y(), 'f', 4);
        buf += ' ';
        buf += QByteArray::number(v.z(), 'f', 4);
        buf += '\n';
        if (buf.size() >= (4 << 20)) { f.write(buf); buf.clear(); }
    }
    if (!buf.isEmpty()) f.write(buf);
    f.close();
    return mesh.vertices.size() > 0;
}

// 读 BCPD 形变后顶点（M×3 文本）。手写 strtod 解析，比正则快。
bool ReadDeformedVertices(const QString& path, QVector<Eigen::Vector3f>& out, int reserveHint)
{
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly)) return false;
    const QByteArray data = f.readAll();
    f.close();
    out.clear();
    if (reserveHint > 0) out.reserve(reserveHint);
    const char* p = data.constData();
    const char* const end = p + data.size();
    while (p < end)
    {
        while (p < end && (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n')) ++p;
        if (p >= end) break;
        char* q = nullptr;
        const double x = std::strtod(p, &q);
        if (q == p) { ++p; continue; }
        p = q;
        const double y = std::strtod(p, &q);
        if (q == p) { ++p; continue; }
        p = q;
        const double z = std::strtod(p, &q);
        if (q == p) { ++p; continue; }
        p = q;
        out.push_back(Eigen::Vector3f(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)));
    }
    return !out.isEmpty();
}
}  // namespace

QString BcpdModelAligner::DefaultExePath()
{
    // 优先工程根/SDK/BCPD（运行时工作目录已切到工程根，与其他 SDK 定位一致）；
    // 回退 exe 同目录（部署目录无 .sln 时工作目录即 exe 目录，SDK 随包在此）。
    const QString rootPath = RobotDataHelper::BuildProjectPath(QStringLiteral("SDK/BCPD/bcpd.exe"));
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

    QTemporaryDir tmp;
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
    if (!WriteCloudMatrix(xFile, cloud, stride, xWritten))
    {
        logmsg(QStringLiteral("BCPD：写点云临时文件失败，跳过配准"));
        return false;
    }
    if (!WriteMeshVertices(yFile, model))
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
        logmsg(QStringLiteral("BCPD：进程启动失败，跳过配准"));
        return false;
    }
    if (!proc.waitForFinished(options.timeoutMs))
    {
        proc.kill();
        proc.waitForFinished(2000);
        logmsg(QStringLiteral("BCPD：配准超时（>%1ms），跳过").arg(options.timeoutMs));
        return false;
    }
    if (proc.exitStatus() != QProcess::NormalExit || proc.exitCode() != 0)
    {
        const QString tail = QString::fromLocal8Bit(proc.readAll()).trimmed().right(300);
        logmsg(QStringLiteral("BCPD：进程异常退出（code=%1）：%2").arg(proc.exitCode()).arg(tail));
        return false;
    }

    QVector<Eigen::Vector3f> deformed;
    if (!ReadDeformedVertices(outY, deformed, model.vertices.size()))
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
