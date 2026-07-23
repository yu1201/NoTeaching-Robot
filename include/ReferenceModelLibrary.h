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
    // 从 STEP/STP 导入：保留原始 CAD，使用 OCCT 转换为毫米制 PLY，并写入可追溯元数据。
    // PLY 仍是模板 modelSha256 的运行身份；源 STEP 供后续精确边线/焊缝提取使用。
    static bool ImportFromStepFile(
        const QString& name,
        const QString& srcStepPath,
        QString& error,
        QString* summary = nullptr,
        QString* confirmationToken = nullptr);
    // 查询上一次中断在“人工确认”之前的 STEP 导入。查询本身不会删除或公开任何模型。
    static bool InspectPendingStepImport(
        const QString& name,
        bool& pending,
        bool& canConfirm,
        QString& summary,
        QString& confirmationToken,
        QString& error);
    // STEP 转换完成后仍处于隐藏待确认态；token 将确认动作绑定到界面刚展示的那次导入事务。
    static bool ConfirmStepImport(
        const QString& name,
        const QString& confirmationToken,
        QString& error);
    // 仅显式撤销隐藏的待确认/中断 STEP 记录；不会删除已确认模型或普通 PLY 模型。
    static bool DiscardPendingStepImport(
        const QString& name,
        const QString& expectedPendingToken,
        QString& error);
    static QString SourceStepPath(const QString& name);
    static QString ModelMetadataPath(const QString& name);
    // 返回经过已确认元数据约束的 STEP 外显源。expectedPlySha256 把源 CAD 与
    // 模板使用的计算 PLY 身份绑定；返回的 sourceSha256 由显示控件在读取前复核。
    static bool ResolveConfirmedStepDisplaySource(
        const QString& name,
        const QString& expectedPlySha256,
        QString& sourceStepPath,
        QString& sourceSha256,
        QString& error);
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
