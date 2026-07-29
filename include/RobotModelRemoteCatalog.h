#pragma once

#include "RobotModelCatalogStore.h"

#include <QByteArray>
#include <QImage>
#include <QList>
#include <QSize>
#include <QString>
#include <QtGlobal>

// 服务器机器人模型库的只读清单协议与预览图生成器。
//
// 服务器清单只负责传输和展示，不授予任何机器人型号资格。下载完成后仍必须
// 通过 TheoreticalRobotModelStore、RobotCollisionEnvelopeStore 和
// RobotModelCatalogStore 的代码内可信 revision 门禁，才能进入生产流程。
class RobotModelRemoteCatalog final
{
public:
    struct PreviewAsset
    {
        QString sourceKind;
        // 旧版 catalog-v1 没有 sourceKind。解析后仍会把它识别成碰撞简模，
        // 但重算旧清单 payload 时必须保持原字段形态，才能通过既有 SHA-256。
        bool sourceKindDeclared = false;
        QString sha256;
        QString storedFileName;
        qint64 sizeBytes = 0;
        int width = 0;
        int height = 0;
    };

    struct ModelRecord
    {
        RobotModelCatalogStore::ModelRecord model;
        PreviewAsset preview;
    };

    struct Catalog
    {
        qint64 revisionUtcMs = 0;
        QString publishedUtc;
        QString previousCatalogSha256;
        QList<ModelRecord> models;
        QString payloadSha256;
    };

    static constexpr qint64 MaximumCatalogBytes = 2LL * 1024LL * 1024LL;
    static constexpr qint64 MaximumPreviewBytes = 4LL * 1024LL * 1024LL;
    static constexpr int PreviewWidth = 480;
    static constexpr int PreviewHeight = 320;

    static QString RemoteRoot();
    static QString RemoteAssetsDirectory();
    static QString RemoteCollisionDirectory();
    static QString RemotePreviewDirectory();
    static QString RemoteCatalogDirectory();
    static QString OriginalStepPreviewKind();
    static QString CollisionEnvelopePreviewKind();

    static QString CatalogFileName(const Catalog& catalog);
    static bool ParseCatalogFileName(
        const QString& fileName,
        qint64& revisionUtcMs,
        QString& payloadSha256);

    static bool Serialize(
        Catalog catalog,
        QByteArray& bytes,
        Catalog& normalized,
        QString& error);
    static bool Parse(
        const QByteArray& bytes,
        Catalog& catalog,
        QString& error);

    static bool RenderPreview(
        const RobotCollisionEnvelopeStore::EnvelopeSet& envelope,
        const QString& displayName,
        QImage& image,
        QString& error);
    static bool EncodePreview(
        const QImage& image,
        const QString& sourceKind,
        QByteArray& pngBytes,
        PreviewAsset& asset,
        QString& error);
    static bool ValidatePreview(
        const QByteArray& pngBytes,
        const PreviewAsset& expected,
        QImage& image,
        QString& error);

    static QString Sha256(const QByteArray& bytes);
    static bool HashFileStable(
        const QString& path,
        qint64 maximumBytes,
        QString& sha256,
        qint64& sizeBytes,
        QString& error);

private:
    RobotModelRemoteCatalog() = delete;
};
