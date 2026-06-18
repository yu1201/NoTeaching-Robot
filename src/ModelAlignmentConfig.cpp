#include "ModelAlignmentConfig.h"

#include "ConfigDatabase.h"

#include <QtGlobal>

namespace
{
const QString kScope = QStringLiteral("global");
const QString kMeta = QStringLiteral("ModelAlignment");
const QString kCountKey = QStringLiteral("GroupCount");
const QString kCurrentKey = QStringLiteral("CurrentGroup");

QString GroupModule(int i) { return QStringLiteral("ModelAlignment/Group%1").arg(qMax(0, i)); }
QString GroupNameKey(int i) { return QStringLiteral("Group%1Name").arg(qMax(0, i)); }

QString Rd(const QString& module, const QString& key, const QString& def)
{
    QString v;
    if (ConfigDatabase::ReadScopedSetting(kScope, QString(), module, key, &v) && !v.isEmpty())
        return v;
    return def;
}
void Wr(const QString& module, const QString& key, const QString& val, const QString& type)
{
    ConfigDatabase::WriteScopedSetting(kScope, QString(), module, key, val, type);
}
double RdD(const QString& m, const QString& k, double def)
{
    bool ok = false;
    const double v = Rd(m, k, QString()).toDouble(&ok);
    return ok ? v : def;
}
int RdI(const QString& m, const QString& k, int def)
{
    bool ok = false;
    const int v = Rd(m, k, QString()).toInt(&ok);
    return ok ? v : def;
}
bool RdB(const QString& m, const QString& k, bool def)
{
    const QString v = Rd(m, k, QString()).trimmed().toLower();
    if (v.isEmpty()) return def;
    return v == QStringLiteral("1") || v == QStringLiteral("true") || v == QStringLiteral("yes");
}
}  // namespace

int ModelAlignmentConfig::GroupCount()
{
    const int n = RdI(kMeta, kCountKey, 0);
    if (n < 1)
    {
        // 首次使用：建一个默认组并落盘。
        SaveGroup(0, QStringLiteral("默认参数组"), ModelAlignmentParams());
        Wr(kMeta, kCountKey, QStringLiteral("1"), QStringLiteral("int"));
        return 1;
    }
    return n;
}

int ModelAlignmentConfig::CurrentGroupIndex()
{
    const int n = GroupCount();
    return qBound(0, RdI(kMeta, kCurrentKey, 0), n - 1);
}

void ModelAlignmentConfig::SetCurrentGroupIndex(int index)
{
    Wr(kMeta, kCurrentKey, QString::number(qMax(0, index)), QStringLiteral("int"));
}

QString ModelAlignmentConfig::GroupName(int index)
{
    return Rd(kMeta, GroupNameKey(index), QStringLiteral("参数组%1").arg(index + 1));
}

ModelAlignmentParams ModelAlignmentConfig::LoadGroup(int index)
{
    const QString m = GroupModule(index);
    ModelAlignmentParams p;
    p.referenceModel = Rd(m, QStringLiteral("RefModel"), QString());
    p.omega = RdD(m, QStringLiteral("Omega"), p.omega);
    p.lambda = RdD(m, QStringLiteral("Lambda"), p.lambda);
    p.beta = RdD(m, QStringLiteral("Beta"), p.beta);
    p.targetSampleCap = RdI(m, QStringLiteral("SampleCap"), p.targetSampleCap);
    p.useAcceleration = RdB(m, QStringLiteral("Accel"), p.useAcceleration);
    p.distanceThresholdMm = RdD(m, QStringLiteral("DistThresh"), p.distanceThresholdMm);
    p.keepPointsWithoutSurface = RdB(m, QStringLiteral("KeepNoSurface"), p.keepPointsWithoutSurface);
    return p;
}

void ModelAlignmentConfig::SaveGroup(int index, const QString& name, const ModelAlignmentParams& p)
{
    Wr(kMeta, GroupNameKey(index), name, QStringLiteral("string"));
    const QString m = GroupModule(index);
    Wr(m, QStringLiteral("RefModel"), p.referenceModel, QStringLiteral("string"));
    Wr(m, QStringLiteral("Omega"), QString::number(p.omega, 'g', 6), QStringLiteral("double"));
    Wr(m, QStringLiteral("Lambda"), QString::number(p.lambda, 'g', 6), QStringLiteral("double"));
    Wr(m, QStringLiteral("Beta"), QString::number(p.beta, 'g', 6), QStringLiteral("double"));
    Wr(m, QStringLiteral("SampleCap"), QString::number(p.targetSampleCap), QStringLiteral("int"));
    Wr(m, QStringLiteral("Accel"), p.useAcceleration ? QStringLiteral("1") : QStringLiteral("0"), QStringLiteral("bool"));
    Wr(m, QStringLiteral("DistThresh"), QString::number(p.distanceThresholdMm, 'g', 6), QStringLiteral("double"));
    Wr(m, QStringLiteral("KeepNoSurface"), p.keepPointsWithoutSurface ? QStringLiteral("1") : QStringLiteral("0"), QStringLiteral("bool"));
}

int ModelAlignmentConfig::AddGroup(const QString& name, const ModelAlignmentParams& p)
{
    const int n = GroupCount();
    SaveGroup(n, name, p);
    Wr(kMeta, kCountKey, QString::number(n + 1), QStringLiteral("int"));
    return n;
}

bool ModelAlignmentConfig::DeleteGroup(int index)
{
    const int n = GroupCount();
    if (n <= 1 || index < 0 || index >= n) return false;  // 至少保留 1 组
    // 把 index 之后的组整体前移一位（覆盖被删组）。
    for (int i = index; i < n - 1; ++i)
    {
        SaveGroup(i, GroupName(i + 1), LoadGroup(i + 1));
    }
    Wr(kMeta, kCountKey, QString::number(n - 1), QStringLiteral("int"));
    const int cur = CurrentGroupIndex();
    if (cur >= n - 1) SetCurrentGroupIndex(n - 2);
    return true;
}
