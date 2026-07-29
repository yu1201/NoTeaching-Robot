#include "RobotModelRemoteCatalog.h"

#include <QCoreApplication>
#include <QFile>
#include <QGuiApplication>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTemporaryDir>
#include <QTextStream>

namespace
{
bool Require(bool condition, const QString& message)
{
    if (!condition)
        QTextStream(stderr) << "FAIL: " << message << Qt::endl;
    return condition;
}

RobotCadAssemblyLoader::Result RobotFixture(const QString& sourceSha256)
{
    RobotCadAssemblyLoader::Result result;
    result.statistics.sourceSha256 = sourceSha256;
    result.statistics.sourceLengthUnits = { QStringLiteral("mm") };
    for (int index = 0; index <= 6; ++index)
    {
        RobotCadAssemblyLoader::ComponentStatistics component;
        component.assemblyPath =
            QStringLiteral("SA10-2000H/J%1").arg(index);
        component.instanceName = QStringLiteral("J%1").arg(index);
        component.productName = QStringLiteral("J%1").arg(index);
        component.jointIndex = index;
        component.included = true;
        component.boundsMm.valid = true;
        component.boundsMm.minimumMm = Eigen::Vector3d(
            -150.0 - index * 10.0,
            -40.0 + index * 120.0,
            -90.0 - index * 5.0);
        component.boundsMm.maximumMm = Eigen::Vector3d(
            150.0 + index * 10.0,
            180.0 + index * 180.0,
            90.0 + index * 5.0);
        result.statistics.components.append(component);
    }
    result.base.valid = true;
    result.base.sourceUp = Eigen::Vector3d::UnitY();
    result.base.j0BoundsMm =
        result.statistics.components.first().boundsMm;
    result.base.minimumYmm =
        result.base.j0BoundsMm.minimumMm.y();
    result.base.conservativeBaseCenterMm =
        Eigen::Vector3d(0.0, result.base.minimumYmm, 0.0);
    return result;
}

RobotModelRemoteCatalog::ModelRecord ModelFixture(
    const RobotCollisionEnvelopeStore::EnvelopeSet& envelope,
    const RobotModelRemoteCatalog::PreviewAsset& preview)
{
    RobotModelRemoteCatalog::ModelRecord record;
    auto& model = record.model;
    model.modelId = QStringLiteral("step.sa10-2000h");
    model.displayName = QStringLiteral("新时达 SA10-2000H");
    model.adapterId = QStringLiteral("step.sa10-2000h.occt.v1");
    model.sourceRobotType = 1;
    model.sourceStep.sha256 = envelope.sourceStepSha256;
    model.sourceStep.storedFileName =
        envelope.sourceStepSha256 + QStringLiteral(".step");
    model.sourceStep.originalDisplayName =
        QStringLiteral("SA10-2000H.step");
    model.sourceStep.sizeBytes = 103894430;
    model.sourceStep.importedUtc =
        QStringLiteral("2026-07-27T03:15:12.000Z");
    model.collision.profileKeySha256 = envelope.profileKeySha256;
    model.collision.sourceStepSha256 = envelope.sourceStepSha256;
    model.collision.storedFileName =
        envelope.profileKeySha256
        + QStringLiteral(".robot-aabb.json");
    model.collision.sizeBytes = 8192;
    model.collision.safetyMarginMicrometres =
        envelope.safetyMarginMicrometres;
    model.collisionPayloadSha256 = envelope.payloadSha256;
    model.registeredUtc =
        QStringLiteral("2026-07-27T03:16:00.000Z");
    record.preview = preview;
    return record;
}
}

int main(int argc, char* argv[])
{
    QGuiApplication application(argc, argv);
    if (application.arguments().size() == 3
        && application.arguments().at(1)
            == QStringLiteral("--parse-catalog"))
    {
        QFile file(application.arguments().at(2));
        if (!file.open(QIODevice::ReadOnly)
            || file.size()
                > RobotModelRemoteCatalog::MaximumCatalogBytes)
            return 2;
        RobotModelRemoteCatalog::Catalog catalog;
        QString parseError;
        if (!RobotModelRemoteCatalog::Parse(
                file.readAll(), catalog, parseError))
        {
            QTextStream(stderr)
                << "FAIL: " << parseError << Qt::endl;
            return 3;
        }
        QTextStream(stdout)
            << "Remote catalog parsed: models="
            << catalog.models.size()
            << " payloadSha256=" << catalog.payloadSha256
            << Qt::endl;
        return 0;
    }
    const QString sourceSha(64, QLatin1Char('a'));
    RobotCollisionEnvelopeStore::EnvelopeSet envelope;
    QString error;
    if (!Require(
            RobotCollisionEnvelopeStore::Generate(
                RobotFixture(sourceSha),
                RobotCollisionEnvelopeStore::GenerationParameters(),
                envelope,
                error),
            QStringLiteral("生成 J0-J6 测试简模失败：%1").arg(error)))
        return 1;

    QImage previewImage;
    if (!Require(
            RobotModelRemoteCatalog::RenderPreview(
                envelope,
                QStringLiteral("新时达 SA10-2000H"),
                previewImage,
                error),
            QStringLiteral("生成预览图失败：%1").arg(error)))
        return 1;

    QByteArray previewBytes;
    RobotModelRemoteCatalog::PreviewAsset previewAsset;
    if (!Require(
            RobotModelRemoteCatalog::EncodePreview(
                previewImage,
                RobotModelRemoteCatalog::CollisionEnvelopePreviewKind(),
                previewBytes, previewAsset, error),
            QStringLiteral("编码预览图失败：%1").arg(error))
        || !Require(
            previewAsset.width == RobotModelRemoteCatalog::PreviewWidth
                && previewAsset.height
                    == RobotModelRemoteCatalog::PreviewHeight
                && previewAsset.sourceKind
                    == RobotModelRemoteCatalog::
                        CollisionEnvelopePreviewKind()
                && previewAsset.sourceKindDeclared
                && previewAsset.sizeBytes == previewBytes.size(),
            QStringLiteral("预览图元数据不一致")))
        return 1;

    QImage decoded;
    if (!Require(
            RobotModelRemoteCatalog::ValidatePreview(
                previewBytes, previewAsset, decoded, error),
            QStringLiteral("预览图回读失败：%1").arg(error)))
        return 1;
    QByteArray tamperedPreview = previewBytes;
    tamperedPreview[tamperedPreview.size() / 2] =
        static_cast<char>(tamperedPreview.at(tamperedPreview.size() / 2) ^ 1);
    if (!Require(
            !RobotModelRemoteCatalog::ValidatePreview(
                tamperedPreview, previewAsset, decoded, error),
            QStringLiteral("被篡改的预览图未被拒绝")))
        return 1;

    RobotModelRemoteCatalog::Catalog input;
    input.revisionUtcMs = 1785122160000LL;
    input.publishedUtc = QStringLiteral("2026-07-27T03:16:00.000Z");
    input.models.append(ModelFixture(envelope, previewAsset));
    QByteArray catalogBytes;
    RobotModelRemoteCatalog::Catalog normalized;
    if (!Require(
            RobotModelRemoteCatalog::Serialize(
                input, catalogBytes, normalized, error),
            QStringLiteral("序列化服务器清单失败：%1").arg(error)))
        return 1;

    const QString catalogFileName =
        RobotModelRemoteCatalog::CatalogFileName(normalized);
    qint64 parsedRevision = 0;
    QString parsedFileSha;
    if (!Require(
            RobotModelRemoteCatalog::ParseCatalogFileName(
                catalogFileName, parsedRevision, parsedFileSha)
                && parsedRevision == normalized.revisionUtcMs
                && parsedFileSha == normalized.payloadSha256,
            QStringLiteral("版本化清单文件名解析失败")))
        return 1;

    RobotModelRemoteCatalog::Catalog parsed;
    if (!Require(
            RobotModelRemoteCatalog::Parse(
                catalogBytes, parsed, error)
                && parsed.models.size() == 1
                && parsed.models.first().model.modelId
                    == QStringLiteral("step.sa10-2000h"),
            QStringLiteral("服务器清单回读失败：%1").arg(error)))
        return 1;

    RobotModelRemoteCatalog::Catalog legacyInput = input;
    legacyInput.models.first().preview.sourceKind =
        RobotModelRemoteCatalog::CollisionEnvelopePreviewKind();
    legacyInput.models.first().preview.sourceKindDeclared = false;
    RobotModelRemoteCatalog::Catalog legacyNormalized;
    QByteArray legacyBytes;
    if (!Require(
            RobotModelRemoteCatalog::Serialize(
                legacyInput, legacyBytes, legacyNormalized, error),
            QStringLiteral("序列化旧版简模预览清单失败：%1").arg(error))
        || !Require(
            RobotModelRemoteCatalog::Parse(
                legacyBytes, parsed, error)
                && parsed.models.size() == 1
                && parsed.models.first().preview.sourceKind
                    == RobotModelRemoteCatalog::
                        CollisionEnvelopePreviewKind()
                && !parsed.models.first().preview.sourceKindDeclared,
            QStringLiteral("旧版简模预览清单兼容读取失败：%1").arg(error)))
        return 1;

    QJsonObject tamperedRoot =
        QJsonDocument::fromJson(catalogBytes).object();
    tamperedRoot.insert(
        QStringLiteral("publishedUtc"),
        QStringLiteral("2026-07-27T03:17:00.000Z"));
    const QByteArray tamperedCatalog =
        QJsonDocument(tamperedRoot).toJson(QJsonDocument::Compact);
    if (!Require(
            !RobotModelRemoteCatalog::Parse(
                tamperedCatalog, parsed, error),
            QStringLiteral("payload 被篡改的服务器清单未被拒绝")))
        return 1;

    QJsonObject extendedRoot =
        QJsonDocument::fromJson(catalogBytes).object();
    extendedRoot.insert(QStringLiteral("unexpected"), true);
    if (!Require(
            !RobotModelRemoteCatalog::Parse(
                QJsonDocument(extendedRoot).toJson(QJsonDocument::Compact),
                parsed,
                error),
            QStringLiteral("带未知字段的服务器清单未被拒绝")))
        return 1;

    RobotModelRemoteCatalog::Catalog duplicate = input;
    duplicate.models.append(input.models.first());
    if (!Require(
            !RobotModelRemoteCatalog::Serialize(
                duplicate, catalogBytes, normalized, error),
            QStringLiteral("重复型号未被拒绝")))
        return 1;

    QTemporaryDir temporary;
    const QString fixturePath =
        temporary.filePath(QStringLiteral("asset.step"));
    QFile fixture(fixturePath);
    const QByteArray fixtureBytes("ISO-10303-21;\nEND-ISO-10303-21;\n");
    if (!fixture.open(QIODevice::WriteOnly)
        || fixture.write(fixtureBytes) != fixtureBytes.size())
        return 1;
    fixture.close();
    QString fileSha;
    qint64 fileSize = 0;
    if (!Require(
            RobotModelRemoteCatalog::HashFileStable(
                fixturePath, 1024, fileSha, fileSize, error)
                && fileSha
                    == RobotModelRemoteCatalog::Sha256(fixtureBytes)
                && fileSize == fixtureBytes.size(),
            QStringLiteral("资产稳定哈希失败：%1").arg(error)))
        return 1;
    if (!Require(
            !RobotModelRemoteCatalog::HashFileStable(
                fixturePath, 8, fileSha, fileSize, error),
            QStringLiteral("超过上限的资产未被拒绝")))
        return 1;

    QTextStream(stdout)
        << "RobotModelRemoteCatalog tests passed." << Qt::endl;
    return 0;
}
