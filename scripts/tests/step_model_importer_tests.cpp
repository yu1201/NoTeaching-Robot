#include "StepModelImporter.h"
#include "ReferenceModelLibrary.h"
#include "RobotDataHelper.h"

#include <BRepPrimAPI_MakeBox.hxx>
#include <DESTEP_Parameters.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <STEPControl_StepModelType.hxx>
#include <STEPControl_Writer.hxx>
#include <TopoDS_Shape.hxx>
#include <UnitsMethods_LengthUnit.hxx>
#include <gp_Pnt.hxx>

#ifdef _WIN32
#include <Windows.h>
#endif

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTemporaryDir>
#include <QTextStream>

#include <cmath>
#include <filesystem>
#include <fstream>

namespace
{
QString gProjectRoot;

int Fail(const QString& message)
{
    QTextStream(stderr) << "FAIL: " << message << Qt::endl;
    return 1;
}

bool WriteBoxStep(
    const QString& path,
    const gp_Pnt& origin,
    const Eigen::Vector3d& dimensionsMm,
    UnitsMethods_LengthUnit outputUnit,
    QString& error)
{
    const TopoDS_Shape shape = BRepPrimAPI_MakeBox(
        origin, dimensionsMm.x(), dimensionsMm.y(), dimensionsMm.z()).Shape();
    STEPControl_Writer writer;
    DESTEP_Parameters parameters;
    parameters.WriteUnit = outputUnit;
    if (writer.Transfer(shape, STEPControl_AsIs, parameters) != IFSelect_RetDone)
    {
        error = QStringLiteral("cannot transfer test box to STEP");
        return false;
    }
#ifdef _WIN32
    std::ofstream output(std::filesystem::path(path.toStdWString()), std::ios::binary);
#else
    std::ofstream output(std::filesystem::path(path.toUtf8().constData()), std::ios::binary);
#endif
    if (!output.is_open() || writer.WriteStream(output) != IFSelect_RetDone)
    {
        error = QStringLiteral("cannot write STEP box fixture");
        return false;
    }
    output.close();
    return true;
}
}

QString RobotDataHelper::BuildProjectPath(const QString& relativePath)
{
    return QDir::toNativeSeparators(QDir(gProjectRoot).filePath(relativePath));
}

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);
    if (app.arguments().size() != 2)
        return Fail(QStringLiteral("usage: StepModelImporterTests <model.step>"));

    const QString sourcePath = app.arguments().at(1);
    if (!QFileInfo::exists(sourcePath))
        return Fail(QStringLiteral("sample STEP does not exist: %1").arg(sourcePath));

    WorkpieceMeshBuilder::Mesh mesh;
    StepModelImporter::Statistics statistics;
    QString error;
    if (!StepModelImporter::ImportFile(sourcePath, mesh, error, &statistics))
        return Fail(QStringLiteral("real STEP import failed: %1").arg(error));

    if (!mesh.IsValid() || mesh.normals.size() != mesh.vertices.size()
        || mesh.indices.size() % 3 != 0)
        return Fail(QStringLiteral("imported mesh structure is invalid"));
    if (statistics.vertexCount != mesh.vertices.size()
        || statistics.triangleCount != mesh.indices.size() / 3
        || statistics.sourceFaceCount <= 0
        || statistics.meshedFaceCount != statistics.sourceFaceCount)
        return Fail(QStringLiteral("import statistics do not match the mesh"));
    if (statistics.sourceLengthUnits.isEmpty())
        return Fail(QStringLiteral("declared STEP length unit evidence is missing"));
    if (!statistics.boundsMinMm.allFinite() || !statistics.boundsMaxMm.allFinite()
        || (statistics.boundsMaxMm - statistics.boundsMinMm).minCoeff() <= 0.0)
        return Fail(QStringLiteral("millimetre bounding box is invalid"));

    for (qsizetype index = 0; index < mesh.vertices.size(); ++index)
    {
        if (!mesh.vertices.at(index).allFinite() || !mesh.normals.at(index).allFinite())
            return Fail(QStringLiteral("mesh contains non-finite vertex/normal"));
        const float normalLength = mesh.normals.at(index).norm();
        if (!std::isfinite(normalLength) || std::abs(normalLength - 1.0f) > 1.0e-3f)
            return Fail(QStringLiteral("mesh contains a non-unit normal"));
    }
    for (quint32 index : mesh.indices)
    {
        if (static_cast<qsizetype>(index) >= mesh.vertices.size())
            return Fail(QStringLiteral("mesh contains an out-of-range index"));
    }

    QTemporaryDir temporaryDir;
    if (!temporaryDir.isValid()) return Fail(QStringLiteral("cannot create temporary directory"));
    const QString invalidPath = temporaryDir.filePath(QStringLiteral("损坏模型.step"));
    QFile invalidFile(invalidPath);
    if (!invalidFile.open(QIODevice::WriteOnly)
        || invalidFile.write("not a STEP file\n") <= 0)
        return Fail(QStringLiteral("cannot create invalid STEP fixture"));
    invalidFile.close();

    WorkpieceMeshBuilder::Mesh invalidMesh = mesh;
    if (StepModelImporter::ImportFile(invalidPath, invalidMesh, error))
        return Fail(QStringLiteral("invalid STEP was unexpectedly accepted"));
    if (!invalidMesh.vertices.isEmpty() || !invalidMesh.normals.isEmpty()
        || !invalidMesh.indices.isEmpty() || error.isEmpty())
        return Fail(QStringLiteral("failed import did not clear output or return an error"));

    StepModelImporter::Options sizeLimit;
    sizeLimit.maximumFileBytes = (std::max<qint64>)(1, QFileInfo(sourcePath).size() - 1);
    if (StepModelImporter::ImportFile(sourcePath, invalidMesh, error, nullptr, &sizeLimit))
        return Fail(QStringLiteral("STEP larger than the configured limit was accepted"));

    const QString farOriginPath = temporaryDir.filePath(QStringLiteral("far-origin.step"));
    if (!WriteBoxStep(
            farOriginPath,
            gp_Pnt(1.0e9, 1.0e9, 1.0e9),
            Eigen::Vector3d(10.0, 20.0, 30.0),
            UnitsMethods_LengthUnit_Millimeter,
            error))
        return Fail(error);
    WorkpieceMeshBuilder::Mesh farOriginMesh = mesh;
    if (StepModelImporter::ImportFile(farOriginPath, farOriginMesh, error))
        return Fail(QStringLiteral("far-origin STEP with unsafe float quantization was accepted"));
    if (!farOriginMesh.vertices.isEmpty() || !farOriginMesh.normals.isEmpty()
        || !farOriginMesh.indices.isEmpty() || !error.contains(QStringLiteral("float")))
        return Fail(QStringLiteral("far-origin rejection did not clear the mesh or explain precision loss"));

    struct UnitFixture
    {
        QString fileName;
        Eigen::Vector3d dimensionsMm;
        UnitsMethods_LengthUnit unit;
    };
    const UnitFixture unitFixtures[] = {
        { QStringLiteral("inch-box.step"), Eigen::Vector3d(25.4, 50.8, 76.2),
          UnitsMethods_LengthUnit_Inch },
        { QStringLiteral("metre-box.step"), Eigen::Vector3d(1000.0, 2000.0, 3000.0),
          UnitsMethods_LengthUnit_Meter }
    };
    for (const UnitFixture& fixture : unitFixtures)
    {
        const QString fixturePath = temporaryDir.filePath(fixture.fileName);
        if (!WriteBoxStep(
                fixturePath, gp_Pnt(0.0, 0.0, 0.0), fixture.dimensionsMm,
                fixture.unit, error))
            return Fail(error);
        WorkpieceMeshBuilder::Mesh convertedMesh;
        StepModelImporter::Statistics convertedStatistics;
        if (!StepModelImporter::ImportFile(
                fixturePath, convertedMesh, error, &convertedStatistics))
            return Fail(QStringLiteral("non-mm STEP import failed: %1").arg(error));
        const Eigen::Vector3d convertedDimensions =
            convertedStatistics.boundsMaxMm - convertedStatistics.boundsMinMm;
        if ((convertedDimensions - fixture.dimensionsMm).cwiseAbs().maxCoeff() > 0.05
            || convertedStatistics.sourceLengthUnits.isEmpty())
            return Fail(QStringLiteral("non-mm STEP was not converted back to expected millimetres"));
    }

    WorkpieceMeshBuilder::Mesh missingNormals = mesh;
    missingNormals.normals.clear();
    if (WorkpieceMeshBuilder::SaveMeshPly(
            temporaryDir.filePath(QStringLiteral("missing-normals.ply")),
            missingNormals, error))
        return Fail(QStringLiteral("PLY writer accepted a mesh without per-vertex normals"));

    QTemporaryDir libraryRoot;
    if (!libraryRoot.isValid()) return Fail(QStringLiteral("cannot create model library root"));
    gProjectRoot = libraryRoot.path();
    QString importSummary;
    QString confirmationToken;
    const QString modelName = QStringLiteral("中文STEP样本");
    for (const QString& orphanPath : {
             ReferenceModelLibrary::SourceStepPath(modelName),
             ReferenceModelLibrary::ModelMetadataPath(modelName) })
    {
        QFile orphan(orphanPath);
        if (!orphan.open(QIODevice::WriteOnly) || orphan.write("interrupted import") <= 0)
            return Fail(QStringLiteral("cannot create interrupted-import fixture"));
    }
    bool pending = false;
    bool canConfirm = true;
    QString pendingSummary;
    QString pendingToken;
    if (!ReferenceModelLibrary::InspectPendingStepImport(
            modelName, pending, canConfirm, pendingSummary, pendingToken, error)
        || !pending || canConfirm || pendingToken.isEmpty())
        return Fail(QStringLiteral("interrupted STEP import was not detected safely: %1").arg(error));
    if (ReferenceModelLibrary::ImportFromStepFile(
            modelName, sourcePath, error, &importSummary, &confirmationToken))
        return Fail(QStringLiteral("STEP import silently replaced an interrupted transaction"));
    if (!QFileInfo::exists(ReferenceModelLibrary::SourceStepPath(modelName))
        || !QFileInfo::exists(ReferenceModelLibrary::ModelMetadataPath(modelName)))
        return Fail(QStringLiteral("rejected replacement deleted the interrupted transaction"));
    if (ReferenceModelLibrary::DiscardPendingStepImport(
            modelName, QStringLiteral("wrong-pending-token"), error)
        || !QFileInfo::exists(ReferenceModelLibrary::SourceStepPath(modelName))
        || !QFileInfo::exists(ReferenceModelLibrary::ModelMetadataPath(modelName)))
        return Fail(QStringLiteral("wrong pending token deleted interrupted STEP artifacts"));
    if (!ReferenceModelLibrary::DiscardPendingStepImport(modelName, pendingToken, error))
        return Fail(QStringLiteral("cannot explicitly discard interrupted STEP import: %1").arg(error));
    if (!ReferenceModelLibrary::ImportFromStepFile(
            modelName, sourcePath, error, &importSummary, &confirmationToken))
        return Fail(QStringLiteral("atomic STEP library import failed: %1").arg(error));
    if (importSummary.isEmpty() || confirmationToken.isEmpty()
        || !QFileInfo::exists(ReferenceModelLibrary::ModelPath(modelName))
        || !QFileInfo::exists(ReferenceModelLibrary::SourceStepPath(modelName))
        || !QFileInfo::exists(ReferenceModelLibrary::ModelMetadataPath(modelName)))
        return Fail(QStringLiteral("STEP library artifacts or summary are missing"));
    if (ReferenceModelLibrary::Exists(modelName)
        || ReferenceModelLibrary::ListModels().contains(modelName))
        return Fail(QStringLiteral("unconfirmed STEP model became visible to the model library"));

    QFile metadataFile(ReferenceModelLibrary::ModelMetadataPath(modelName));
    if (!metadataFile.open(QIODevice::ReadOnly))
        return Fail(QStringLiteral("cannot read STEP model metadata"));
    QJsonParseError jsonError;
    const QJsonDocument metadataDocument = QJsonDocument::fromJson(metadataFile.readAll(), &jsonError);
    metadataFile.close();
    if (jsonError.error != QJsonParseError::NoError || !metadataDocument.isObject())
        return Fail(QStringLiteral("STEP model metadata is not valid JSON"));
    const QJsonObject metadata = metadataDocument.object();
    if (metadata.value(QStringLiteral("schemaVersion")).toInt() != 1
        || metadata.value(QStringLiteral("outputLengthUnit")).toString() != QStringLiteral("mm")
        || metadata.value(QStringLiteral("sourceSha256")).toString().size() != 64
        || metadata.value(QStringLiteral("plySha256")).toString().size() != 64
        || metadata.value(QStringLiteral("transactionId")).toString() != confirmationToken
        || metadata.value(QStringLiteral("dimensionsConfirmed")).toBool(true)
        || metadata.value(QStringLiteral("vertexCount")).toInteger() != statistics.vertexCount
        || metadata.value(QStringLiteral("triangleCount")).toInteger() != statistics.triangleCount)
        return Fail(QStringLiteral("STEP model metadata fields are inconsistent"));

    const QString pendingPlyPath = ReferenceModelLibrary::ModelPath(modelName);
    QFile pendingPlyFile(pendingPlyPath);
    if (!pendingPlyFile.open(QIODevice::ReadOnly))
        return Fail(QStringLiteral("cannot retain pending PLY for snapshot-token test"));
    const QByteArray pendingPlyBytes = pendingPlyFile.readAll();
    pendingPlyFile.close();
    if (!pendingPlyFile.open(QIODevice::Append)
        || pendingPlyFile.write("pending-tamper") != 14)
        return Fail(QStringLiteral("cannot tamper pending PLY for snapshot-token test"));
    pendingPlyFile.close();
    pending = false;
    canConfirm = true;
    pendingSummary.clear();
    pendingToken.clear();
    if (!ReferenceModelLibrary::InspectPendingStepImport(
            modelName, pending, canConfirm, pendingSummary, pendingToken, error)
        || !pending || canConfirm || pendingToken.isEmpty()
        || pendingToken == confirmationToken)
        return Fail(QStringLiteral("tampered pending STEP did not receive a snapshot token"));
    if (ReferenceModelLibrary::DiscardPendingStepImport(modelName, confirmationToken, error)
        || !QFileInfo::exists(ReferenceModelLibrary::SourceStepPath(modelName))
        || !QFileInfo::exists(ReferenceModelLibrary::ModelMetadataPath(modelName)))
        return Fail(QStringLiteral("stale transaction token discarded a tampered pending STEP"));
    if (!pendingPlyFile.open(QIODevice::WriteOnly | QIODevice::Truncate)
        || pendingPlyFile.write(pendingPlyBytes) != pendingPlyBytes.size())
        return Fail(QStringLiteral("cannot restore pending PLY after snapshot-token test"));
    pendingPlyFile.close();
    if (!ReferenceModelLibrary::InspectPendingStepImport(
            modelName, pending, canConfirm, pendingSummary, pendingToken, error)
        || !pending || !canConfirm || pendingToken != confirmationToken)
        return Fail(QStringLiteral("restored pending STEP did not recover its transaction token"));

#ifdef _WIN32
    HANDLE pendingPlyHandle = CreateFileW(
        reinterpret_cast<LPCWSTR>(pendingPlyPath.utf16()),
        GENERIC_READ,
        FILE_SHARE_READ | FILE_SHARE_WRITE,
        nullptr,
        OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL,
        nullptr);
    if (pendingPlyHandle == INVALID_HANDLE_VALUE)
        return Fail(QStringLiteral("cannot lock pending PLY for rollback safety test"));
    const bool unsafeDiscardAccepted = ReferenceModelLibrary::DiscardPendingStepImport(
        modelName, confirmationToken, error);
    CloseHandle(pendingPlyHandle);
    if (unsafeDiscardAccepted
        || !QFileInfo::exists(ReferenceModelLibrary::ModelPath(modelName))
        || !QFileInfo::exists(ReferenceModelLibrary::SourceStepPath(modelName))
        || !QFileInfo::exists(ReferenceModelLibrary::ModelMetadataPath(modelName))
        || ReferenceModelLibrary::Exists(modelName))
        return Fail(QStringLiteral("failed PLY removal exposed or stripped a pending STEP model"));
#endif

    if (ReferenceModelLibrary::ConfirmStepImport(
            modelName, QStringLiteral("wrong-transaction-token"), error)
        || ReferenceModelLibrary::Exists(modelName))
        return Fail(QStringLiteral("wrong STEP transaction token was accepted"));
    if (!ReferenceModelLibrary::ConfirmStepImport(modelName, confirmationToken, error)
        || !ReferenceModelLibrary::Exists(modelName)
        || !ReferenceModelLibrary::ListModels().contains(modelName))
        return Fail(QStringLiteral("STEP dimension confirmation did not publish the model: %1").arg(error));

    QString displaySourcePath;
    QString displaySourceSha256;
    if (ReferenceModelLibrary::ResolveConfirmedStepDisplaySource(
            modelName,
            QString(64, QLatin1Char('0')),
            displaySourcePath,
            displaySourceSha256,
            error))
        return Fail(QStringLiteral("STEP display source accepted the wrong computation PLY identity"));
    const QString expectedPlySha256 = metadata.value(QStringLiteral("plySha256"))
        .toString().toLower();
    const QString expectedSourceSha256 = metadata.value(QStringLiteral("sourceSha256"))
        .toString().toLower();
    if (!ReferenceModelLibrary::ResolveConfirmedStepDisplaySource(
            modelName,
            expectedPlySha256,
            displaySourcePath,
            displaySourceSha256,
            error)
        || QFileInfo(displaySourcePath).canonicalFilePath()
            != QFileInfo(ReferenceModelLibrary::SourceStepPath(modelName)).canonicalFilePath()
        || displaySourceSha256 != expectedSourceSha256)
        return Fail(QStringLiteral("confirmed STEP display source did not resolve safely: %1").arg(error));

    const QString librarySourcePath = ReferenceModelLibrary::SourceStepPath(modelName);
    QFile originalSourceFile(librarySourcePath);
    if (!originalSourceFile.open(QIODevice::ReadOnly))
        return Fail(QStringLiteral("cannot retain original STEP source for display integrity test"));
    const QByteArray originalSourceBytes = originalSourceFile.readAll();
    originalSourceFile.close();
    QFile tamperedSourceFile(librarySourcePath);
    if (!tamperedSourceFile.open(QIODevice::Append) || tamperedSourceFile.write("tamper") != 6)
        return Fail(QStringLiteral("cannot create tampered STEP display fixture"));
    tamperedSourceFile.close();
    if (ReferenceModelLibrary::ResolveConfirmedStepDisplaySource(
            modelName,
            expectedPlySha256,
            displaySourcePath,
            displaySourceSha256,
            error))
        return Fail(QStringLiteral("tampered STEP source passed display provenance verification"));
    if (!tamperedSourceFile.open(QIODevice::WriteOnly | QIODevice::Truncate)
        || tamperedSourceFile.write(originalSourceBytes) != originalSourceBytes.size())
        return Fail(QStringLiteral("cannot restore STEP source after display integrity test"));
    tamperedSourceFile.close();

    const QString libraryPlyPath = ReferenceModelLibrary::ModelPath(modelName);
    QFile originalPlyFile(libraryPlyPath);
    if (!originalPlyFile.open(QIODevice::ReadOnly))
        return Fail(QStringLiteral("cannot retain original library PLY for integrity test"));
    const QByteArray originalPlyBytes = originalPlyFile.readAll();
    originalPlyFile.close();
    QFile tamperedPlyFile(libraryPlyPath);
    if (!tamperedPlyFile.open(QIODevice::Append) || tamperedPlyFile.write("tamper") != 6)
        return Fail(QStringLiteral("cannot create tampered library PLY fixture"));
    tamperedPlyFile.close();
    WorkpieceMeshBuilder::Mesh libraryMesh;
    if (ReferenceModelLibrary::LoadModel(modelName, libraryMesh, error))
        return Fail(QStringLiteral("tampered STEP-derived PLY passed provenance verification"));
    if (ReferenceModelLibrary::ResolveConfirmedStepDisplaySource(
            modelName,
            expectedPlySha256,
            displaySourcePath,
            displaySourceSha256,
            error))
        return Fail(QStringLiteral("tampered computation PLY remained eligible for STEP display"));
    QFile restorePlyFile(libraryPlyPath);
    if (!restorePlyFile.open(QIODevice::WriteOnly | QIODevice::Truncate)
        || restorePlyFile.write(originalPlyBytes) != originalPlyBytes.size())
        return Fail(QStringLiteral("cannot restore library PLY after integrity test"));
    restorePlyFile.close();

    if (!ReferenceModelLibrary::LoadModel(modelName, libraryMesh, error)
        || libraryMesh.vertices.size() != mesh.vertices.size()
        || libraryMesh.indices.size() != mesh.indices.size())
        return Fail(QStringLiteral("library PLY write-back validation failed: %1").arg(error));
    if (ReferenceModelLibrary::ImportFromStepFile(modelName, sourcePath, error))
        return Fail(QStringLiteral("same-name STEP import was unexpectedly accepted"));

    const QString renamedModel = QStringLiteral("中文STEP样本_重命名");
    if (!ReferenceModelLibrary::RenameModel(modelName, renamedModel, error)
        || !QFileInfo::exists(ReferenceModelLibrary::ModelPath(renamedModel))
        || !QFileInfo::exists(ReferenceModelLibrary::SourceStepPath(renamedModel))
        || !QFileInfo::exists(ReferenceModelLibrary::ModelMetadataPath(renamedModel)))
        return Fail(QStringLiteral("STEP model artifact rename failed: %1").arg(error));
    if (!ReferenceModelLibrary::DeleteModel(renamedModel, error)
        || QFileInfo::exists(ReferenceModelLibrary::ModelPath(renamedModel))
        || QFileInfo::exists(ReferenceModelLibrary::SourceStepPath(renamedModel))
        || QFileInfo::exists(ReferenceModelLibrary::ModelMetadataPath(renamedModel)))
        return Fail(QStringLiteral("STEP model artifact deletion failed: %1").arg(error));

    const Eigen::Vector3d dimensions = statistics.boundsMaxMm - statistics.boundsMinMm;
    QTextStream(stdout)
        << "STEP_IMPORT_OK"
        << " occt=" << statistics.occtVersion
        << " units=" << statistics.sourceLengthUnits.join('|')
        << " faces=" << statistics.sourceFaceCount
        << " vertices=" << statistics.vertexCount
        << " triangles=" << statistics.triangleCount
        << " dimensions_mm=" << dimensions.x() << ',' << dimensions.y() << ',' << dimensions.z()
        << " deflection_mm=" << statistics.linearDeflectionMm
        << " skipped_degenerate=" << statistics.skippedDegenerateTriangles
        << Qt::endl;
    return 0;
}
