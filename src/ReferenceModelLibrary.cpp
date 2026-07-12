#include "ReferenceModelLibrary.h"

#include "RobotDataHelper.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QRegularExpression>

namespace
{
const QString kLibSubdir = QStringLiteral("Data/ReferenceModels");
}

QString ReferenceModelLibrary::LibraryDir()
{
    const QString dir = RobotDataHelper::BuildProjectPath(kLibSubdir);
    QDir().mkpath(dir);  // 不存在则递归创建
    return dir;
}

QString ReferenceModelLibrary::ModelPath(const QString& name)
{
    return QDir(LibraryDir()).filePath(name + QStringLiteral(".ply"));
}

QStringList ReferenceModelLibrary::ListModels()
{
    QStringList names;
    const QDir d(LibraryDir());
    const QFileInfoList items = d.entryInfoList(QStringList() << QStringLiteral("*.ply"),
                                                QDir::Files, QDir::Name);
    for (const QFileInfo& fi : items)
    {
        names << fi.completeBaseName();
    }
    return names;
}

bool ReferenceModelLibrary::Exists(const QString& name)
{
    return IsValidName(name) && QFileInfo::exists(ModelPath(name));
}

bool ReferenceModelLibrary::IsValidName(const QString& name)
{
    if (name.isEmpty() || name.length() > 80) return false;
    if (name == QStringLiteral(".") || name == QStringLiteral("..")) return false;
    static const QRegularExpression bad(QStringLiteral("[\\\\/:*?\"<>|\\x00-\\x1f]"));
    return !name.contains(bad);
}

bool ReferenceModelLibrary::ImportFromMesh(const QString& name, const WorkpieceMeshBuilder::Mesh& mesh, QString& error)
{
    if (!IsValidName(name)) { error = QStringLiteral("模型名非法（含路径分隔符或非法字符）"); return false; }
    if (!mesh.IsValid()) { error = QStringLiteral("网格无效（顶点/三角形不足）"); return false; }
    return WorkpieceMeshBuilder::SaveMeshPly(ModelPath(name), mesh, error);
}

bool ReferenceModelLibrary::ImportFromFile(const QString& name, const QString& srcPlyPath, QString& error)
{
    if (!IsValidName(name)) { error = QStringLiteral("模型名非法（含路径分隔符或非法字符）"); return false; }
    WorkpieceMeshBuilder::Mesh mesh;
    if (!WorkpieceMeshBuilder::LoadMeshPly(srcPlyPath, mesh, error)) return false;  // 先校验是合法网格
    if (!mesh.IsValid()) { error = QStringLiteral("源文件不是有效网格"); return false; }
    return WorkpieceMeshBuilder::SaveMeshPly(ModelPath(name), mesh, error);
}

bool ReferenceModelLibrary::LoadModel(
    const QString& name,
    WorkpieceMeshBuilder::Mesh& mesh,
    QString& error,
    const WorkpieceMeshBuilder::CancelCallback& cancelRequested)
{
    if (!Exists(name)) { error = QStringLiteral("基准模型不存在：%1").arg(name); return false; }
    return WorkpieceMeshBuilder::LoadMeshPly(ModelPath(name), mesh, error, cancelRequested);
}

bool ReferenceModelLibrary::DeleteModel(const QString& name, QString& error)
{
    if (!Exists(name)) { error = QStringLiteral("基准模型不存在：%1").arg(name); return false; }
    if (!QFile::remove(ModelPath(name))) { error = QStringLiteral("删除失败：%1").arg(name); return false; }
    return true;
}

bool ReferenceModelLibrary::RenameModel(const QString& fromName, const QString& toName, QString& error)
{
    if (!Exists(fromName)) { error = QStringLiteral("源模型不存在：%1").arg(fromName); return false; }
    if (!IsValidName(toName)) { error = QStringLiteral("新名非法"); return false; }
    if (Exists(toName)) { error = QStringLiteral("目标名已存在：%1").arg(toName); return false; }
    if (!QFile::rename(ModelPath(fromName), ModelPath(toName))) { error = QStringLiteral("重命名失败"); return false; }
    return true;
}
