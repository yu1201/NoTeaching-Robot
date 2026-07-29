#pragma once

#include "RobotCadAssemblyLoader.h"

#include <Eigen/Core>

#include <QString>
#include <QVector>
#include <QtGlobal>

// 机器人简化碰撞包络库。
//
// 当前有意只生成源 STEP 坐标系下的轴对齐包围盒（AABB），不把它描述成
// OBB，也不修改工件的原始 STEP/B-Rep；详细机器人 B-Rep 仅保留为诊断后备。
// 每个 J0...J6 独立膨胀安全余量，适合静态、粗粒度碰撞预检。动态轨迹检查
// 仍需关节值/DH、机器人基座变换和工具模型，调用方必须先把各关节包络更新
// 到同一个场景坐标系。
class RobotCollisionEnvelopeStore final
{
public:
    struct GenerationParameters
    {
        // 持久化精度为 0.001 mm；Generate/Load 均拒绝无法精确量化的值。
        double safetyMarginMm = 30.0;
    };

    struct AxisAlignedBounds
    {
        Eigen::Vector3d minimumMm = Eigen::Vector3d::Zero();
        Eigen::Vector3d maximumMm = Eigen::Vector3d::Zero();
    };

    struct JointEnvelope
    {
        int jointIndex = -1;
        QString jointName;
        QString assemblyPath;
        AxisAlignedBounds sourceBoundsMm;
        AxisAlignedBounds collisionBoundsMm;
    };

    struct EnvelopeSet
    {
        QString profileKeySha256;
        QString sourceStepSha256;
        QString sourceLengthUnit;
        QString coordinateFrame;
        bool staticOnly = true;
        QString algorithm;
        qint64 safetyMarginMicrometres = 0;
        Eigen::Vector3d sourceUp = Eigen::Vector3d::UnitY();
        double baseMinimumYmm = 0.0;
        Eigen::Vector3d baseCenterMm = Eigen::Vector3d::Zero();
        AxisAlignedBounds overallSourceBoundsMm;
        AxisAlignedBounds overallExpandedBoundsMm;
        QVector<JointEnvelope> joints;
        QString payloadSha256;
    };

    struct StoredAsset
    {
        QString profileKeySha256;
        QString sourceStepSha256;
        QString storedFileName;
        qint64 sizeBytes = 0;
        qint64 safetyMarginMicrometres = 0;
    };

    static constexpr double DefaultSafetyMarginMm = 30.0;
    static constexpr double MaximumSafetyMarginMm = 500.0;
    static constexpr qint64 MaximumEnvelopeFileBytes = 64 * 1024;

    static QString StoreDirectory();

    // 对内存中的完整包络执行与持久化/回读完全相同的身份、payload、参数、
    // J0-J6 和边界一致性校验。显示层应先调用本接口，再追加型号专属场景约束。
    static bool ValidateForUse(const EnvelopeSet& envelope, QString& error);

    // 从 loader 统计信息中精确选择唯一的 J0...J6。成功时完整替换 output；
    // 失败时清空 output，绝不返回部分关节集合。
    static bool Generate(
        const RobotCadAssemblyLoader::Result& loadedRobot,
        const GenerationParameters& parameters,
        EnvelopeSet& output,
        QString& error);

    // 使用 profileKey 的 ASCII 哈希文件名原子发布到 Data/RobotModels。
    static bool Persist(
        const EnvelopeSet& envelope,
        StoredAsset& asset,
        QString& error);

    // 导入服务器下载的碰撞简模：先对任意来源小文件执行完整 schema、payload、
    // J0-J6 与边界校验，再通过 Persist 的内容寻址和原子回读发布到本地库。
    static bool ImportFile(
        const QString& sourcePath,
        StoredAsset& asset,
        EnvelopeSet& envelope,
        QString& error);

    // 根据 STEP SHA 和生成参数解析确定性文件名，并完整复核 JSON、payload
    // 哈希、J0...J6 唯一性、边界覆盖及安全余量。
    static bool Load(
        const QString& sourceStepSha256,
        const GenerationParameters& parameters,
        EnvelopeSet& envelope,
        StoredAsset& asset,
        QString& error);

    // 查询确定性 profile 文件是否尚未生成。返回 true 表示查询本身通过，
    // missing 才是文件状态；路径为链接、非普通文件或存储目录无效时失败关闭。
    // 上层只可在 missing=true 时回退解析大型 STEP，已有但损坏的 JSON 不应
    // 被当成“缺失”而自动覆盖。
    static bool IsMissing(
        const QString& sourceStepSha256,
        const GenerationParameters& parameters,
        bool& missing,
        QString& error);

    // 供上层清单保存 StoredAsset 后回读。storedFileName 必须严格等于
    // profileKeySha256 + ".robot-aabb.json"，不能包含用户路径。
    static bool LoadAsset(
        const StoredAsset& expectedAsset,
        EnvelopeSet& envelope,
        QString& error);

private:
    RobotCollisionEnvelopeStore() = delete;
};
