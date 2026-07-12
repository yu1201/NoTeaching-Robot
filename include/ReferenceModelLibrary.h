#pragma once

#include "WorkpieceMeshBuilder.h"

#include <QString>
#include <QStringList>

// 基准模型库：在工程根 Data/ReferenceModels/ 下管理多个 .ply 基准模型，供「模型配准」
// 功能选用作去噪参照。每个模型 = <名字>.ply（沿用 WorkpieceMeshBuilder 的二进制网格格式，
// 可直接拖入 CloudCompare/MeshLab 查看）。模型名即文件主名，供 UI 列表展示与选择。
class ReferenceModelLibrary
{
public:
    // 模型库目录全路径（相对工程根 Data/ReferenceModels/，不存在则自动创建）。
    static QString LibraryDir();
    // 某模型的 .ply 全路径（不保证存在）。
    static QString ModelPath(const QString& name);
    // 列举所有基准模型名（不含 .ply 扩展名，按名称排序）。
    static QStringList ListModels();
    static bool Exists(const QString& name);

    // 把网格导入为基准模型（覆盖同名）。
    static bool ImportFromMesh(const QString& name, const WorkpieceMeshBuilder::Mesh& mesh, QString& error);
    // 从已有 .ply 文件导入为基准（先校验是合法网格，再存入库）。
    static bool ImportFromFile(const QString& name, const QString& srcPlyPath, QString& error);
    // 加载某基准模型为网格。
    static bool LoadModel(
        const QString& name,
        WorkpieceMeshBuilder::Mesh& mesh,
        QString& error,
        const WorkpieceMeshBuilder::CancelCallback& cancelRequested = WorkpieceMeshBuilder::CancelCallback());
    static bool DeleteModel(const QString& name, QString& error);
    static bool RenameModel(const QString& fromName, const QString& toName, QString& error);

    // 合法文件名校验（禁路径分隔符/通配/控制字符，防注入与跨目录写）。
    static bool IsValidName(const QString& name);
};
