#pragma once

#include "RobotCollisionEnvelopeStore.h"
#include "TheoreticalRobotModelStore.h"

#include <QList>
#include <QString>

// 机器人型号目录。
//
// STEP 和碰撞简模仍由各自的内容寻址存储层管理；本目录只在两项资产均已
// 完整验证后，原子发布一个可供“模型焊接流程”选择的型号记录。catalog 的
// payload SHA-256 只用于完整性检查；流程资格还必须匹配程序代码内的受信
// 型号、适配器、驱动类型，以及源 STEP / 碰撞简模 revision 元组注册规则。
// modelId 是
// 控制单元保存的稳定可读型号键（例如 step.sa10-2000h），源 STEP SHA-256
// 则是该型号当前不可变的 revision 身份。机器人到型号的选择属于控制单元
// RobotModelId 配置，不在本目录重复保存。
class RobotModelCatalogStore final
{
public:
    struct ModelRecord
    {
        QString modelId;
        QString displayName;
        QString adapterId;
        int sourceRobotType = -1;
        TheoreticalRobotModelStore::Asset sourceStep;
        RobotCollisionEnvelopeStore::StoredAsset collision;
        QString collisionPayloadSha256;
        QString registeredUtc;
    };

    struct Eligibility
    {
        bool eligible = false;
        // 返回 true 但 eligible=false 时，本字段可直接用于界面门禁提示。
        QString reason;
        ModelRecord model;
        RobotCollisionEnvelopeStore::EnvelopeSet collisionEnvelope;
    };

    static constexpr qint64 MaximumCatalogFileBytes = 1024 * 1024;
    static constexpr qsizetype MaximumModels = 1024;

    static QString StoreDirectory();
    static QString CatalogFilePath();

    // 只读取并严格解析小型 catalog；不会打开或哈希大型 STEP。
    static bool ListModels(QList<ModelRecord>& models, QString& error);

    // 注册前完整复核源 STEP 的普通文件、STEP 边界、大小及 SHA-256，并通过
    // RobotCollisionEnvelopeStore 回读校验余量、payload 和 J0...J6。只有最后
    // 一次 catalog 原子提交会使该型号对流程可见；不会自动绑定任何机器人。
    static bool RegisterValidatedModel(
        const QString& modelId,
        const QString& displayName,
        const QString& adapterId,
        int sourceRobotType,
        const TheoreticalRobotModelStore::Asset& sourceStep,
        const RobotCollisionEnvelopeStore::StoredAsset& collision,
        ModelRecord& registeredModel,
        QString& error);

    // 返回 false 表示 catalog 自身无法可信读取；返回 true 且 eligible=false
    // 表示正常的门禁拒绝（型号不存在、代码不信任、类型不符或资产当前不可用）。
    // 资格查询先复核代码内受信身份，再只对大型 STEP 做普通文件/大小快速门禁；
    // 碰撞 JSON 仍做完整回读校验。
    static bool ResolveModelEligibility(
        const QString& modelId,
        int actualRobotType,
        Eligibility& eligibility,
        QString& error);

    // 一次性兼容当前 assets.json 的 active SA10。仅当 active STEP 的精确 SHA、
    // 完整 STEP 校验和默认 30 mm 简模全部通过时登记型号；绝不根据 RobotA/B/C
    // 或当前选择写控制单元配置。没有 active 或 active 不是已核验 SA10 时安静跳过。
    static bool BootstrapVerifiedSa10FromLegacy(
        ModelRecord& model,
        bool& registered,
        QString& error);

private:
    RobotModelCatalogStore() = delete;
};
