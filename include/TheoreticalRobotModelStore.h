#pragma once

#include <QList>
#include <QString>
#include <QtGlobal>

// 理论机器人 STEP 资产库。
//
// 所有持久文件都位于 AppPaths::WritablePath("Data/RobotModels")。导入后的
// 文件名只由内容 SHA-256 构成，用户提供的文件名仅作为界面显示元数据保存，
// 不参与路径拼接。
class TheoreticalRobotModelStore final
{
public:
    struct Asset
    {
        QString sha256;
        QString storedFileName;
        QString originalDisplayName;
        qint64 sizeBytes = 0;
        QString importedUtc;
    };

    static constexpr qint64 MaximumAssetBytes = 256LL * 1024LL * 1024LL;

    static QString StoreDirectory();
    static QString ManifestFilePath();

    // 稳定读取用户选择的 STEP/STP 并按 SHA-256 去重复制。activate=false 用于
    // “先做 J0-J6 语义验证、再 SetActive”的事务式导入；失败不会替换旧当前资产。
    // 不会覆盖库中已存在但内容不一致的文件。
    static bool ImportStepFile(
        const QString& sourcePath,
        Asset& importedAsset,
        QString& error,
        bool activate = true);

    // 返回清单中的全部资产。这里严格校验清单结构和受控文件名，但不会为了
    // 列表刷新而逐个读取大型 STEP；ResolveActive/SetActive 会完整复核目标文件。
    static bool ListAssets(QList<Asset>& assets, QString& error);

    // 只读取并校验小型清单，返回其中的当前资产记录；不会打开、读取或哈希
    // 对应的大型 STEP。没有当前资产不是错误，此时 hasActive=false 且输出清空。
    static bool ReadActiveRecord(
        Asset& activeAsset,
        bool& hasActive,
        QString& error);

    // 只有目标文件的大小和 SHA-256 均与清单一致时才会切换当前资产。
    static bool SetActive(const QString& sha256, QString& error);

    // 在进程内互斥锁和跨进程文件锁共同保护下执行条件切换。expectedSha256 和
    // desiredSha256 均可为空，分别表示“预期当前为空”和“切换为空”。预期不符
    // 时关闭失败；非空目标仍会完整复核 STEP 后才写入清单。
    static bool CompareExchangeActive(
        const QString& expectedSha256,
        const QString& desiredSha256,
        QString& error);

    // 每次调用都重新校验普通文件、大小和 SHA-256。失败时清空所有输出，禁止
    // 调用方继续使用上一次解析出的路径。
    static bool ResolveActive(
        QString& stepPath,
        Asset& activeAsset,
        QString& error);

private:
    TheoreticalRobotModelStore() = delete;
};
