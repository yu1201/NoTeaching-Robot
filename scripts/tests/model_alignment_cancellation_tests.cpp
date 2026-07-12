#include "PointCloudModelDenoiser.h"
#include "WorkpieceMeshBuilder.h"

#include <QFile>
#include <QTemporaryDir>

#include <atomic>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits>

namespace
{
void Check(bool condition, const char* message)
{
    if (!condition)
    {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(1);
    }
}

QByteArray PlyHeader(qint64 vertexCount, qint64 faceCount, const QByteArray& extra = {})
{
    return QByteArray("ply\nformat binary_little_endian 1.0\n")
        + extra
        + "element vertex " + QByteArray::number(vertexCount) + "\n"
        + "property float x\nproperty float y\nproperty float z\n"
          "property float nx\nproperty float ny\nproperty float nz\n"
          "element face " + QByteArray::number(faceCount) + "\n"
          "property list uchar uint vertex_indices\nend_header\n";
}

bool WritePly(const QString& path, qint64 vertexCount, qint64 faceCount,
              const float* firstVertex = nullptr, const quint32* firstFace = nullptr,
              const QByteArray& suffix = {})
{
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) return false;
    if (file.write(PlyHeader(vertexCount, faceCount)) < 0) return false;
    const float normalVertex[6] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f };
    for (qint64 i = 0; i < vertexCount; ++i)
    {
        float record[6];
        std::memcpy(record, normalVertex, sizeof(record));
        record[0] = static_cast<float>(i % 128);
        record[1] = static_cast<float>((i / 128) % 128);
        if (i == 0 && firstVertex != nullptr) std::memcpy(record, firstVertex, sizeof(record));
        if (file.write(reinterpret_cast<const char*>(record), sizeof(record)) != sizeof(record)) return false;
    }
    for (qint64 i = 0; i < faceCount; ++i)
    {
        const char count = 3;
        quint32 indices[3] = { 0, 1, 2 };
        if (i == 0 && firstFace != nullptr) std::memcpy(indices, firstFace, sizeof(indices));
        if (file.write(&count, 1) != 1
            || file.write(reinterpret_cast<const char*>(indices), sizeof(indices)) != sizeof(indices))
            return false;
    }
    return file.write(suffix) == suffix.size() && file.error() == QFileDevice::NoError;
}

bool WriteCloud(const QString& path, int pointCount, const QByteArray& suffix = {})
{
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) return false;
    QByteArray block("index x y z\n");
    for (int i = 0; i < pointCount; ++i)
    {
        block += QByteArray::number(i) + ' '
            + QByteArray::number(i * 0.25, 'f', 4) + " 1.0000 2.0000\n";
    }
    block += suffix;
    return file.write(block) == block.size() && file.error() == QFileDevice::NoError;
}

bool WriteLegacyCloud(const QString& path, int pointCount)
{
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) return false;
    QByteArray block("index frame_index line_point_index x y z camera_x camera_y camera_z "
                     "robot_x robot_y robot_z robot_rx robot_ry robot_rz\n");
    for (int i = 0; i < pointCount; ++i)
    {
        block += QByteArray::number(i) + " 1 2 "
            + QByteArray::number(100.0 + i, 'f', 3)
            + " 200.000 300.000 4 5 6 7 8 9 10 11 12\n";
    }
    return file.write(block) == block.size() && file.error() == QFileDevice::NoError;
}
}

int main()
{
    WorkpieceMeshBuilder::Mesh mesh;
    constexpr int kVertexCount = 8192;
    mesh.vertices.reserve(kVertexCount);
    for (int i = 0; i < kVertexCount; ++i)
    {
        mesh.vertices.push_back(Eigen::Vector3f(
            static_cast<float>(i % 128),
            static_cast<float>((i / 128) % 64),
            static_cast<float>(i % 7) * 0.01f));
    }
    for (int i = 0; i + 2 < kVertexCount; i += 3)
    {
        mesh.indices.push_back(static_cast<quint32>(i));
        mesh.indices.push_back(static_cast<quint32>(i + 1));
        mesh.indices.push_back(static_cast<quint32>(i + 2));
    }

    QVector<RobotCalculation::IndexedPoint3D> cloud;
    for (int i = 0; i < 4096; ++i)
    {
        RobotCalculation::IndexedPoint3D point;
        point.index = i;
        point.point = Eigen::Vector3d(i % 128, (i / 128) % 64, 0.0);
        cloud.push_back(point);
    }

    std::atomic_int checks{ 0 };
    PointCloudModelDenoiser::Options options;
    options.cancelRequested = [&checks]()
    {
        return checks.fetch_add(1) >= 2;
    };
    PointCloudModelDenoiser::Stats stats;
    const auto output = PointCloudModelDenoiser::DenoiseByModelDistance(
        cloud, mesh, options, &stats);
    Check(stats.cancelled, "denoiser did not report cooperative cancellation");
    Check(output.isEmpty(), "cancelled denoiser exposed a partial point cloud");
    Check(checks.load() >= 3, "denoiser cancellation callback was not polled in a hot loop");

    // 病态的大 AABB 三角形不得向数万 bucket 无界复制索引。
    WorkpieceMeshBuilder::Mesh pathological;
    pathological.vertices = {
        Eigen::Vector3f(0.0f, 0.0f, 0.0f),
        Eigen::Vector3f(300.0f, 0.0f, 0.0f),
        Eigen::Vector3f(0.0f, 300.0f, 0.0f)
    };
    pathological.indices = { 0, 1, 2 };
    PointCloudModelDenoiser::Options boundedOptions;
    boundedOptions.cellSizeMm = 1.0;
    PointCloudModelDenoiser::Stats boundedStats;
    const auto boundedOutput = PointCloudModelDenoiser::DenoiseByModelDistance(
        cloud.mid(0, 1), pathological, boundedOptions, &boundedStats);
    Check(boundedStats.resourceLimitExceeded,
          "pathological triangle AABB did not trip the per-triangle coverage limit");
    Check(boundedOutput.isEmpty(), "resource-limited denoiser exposed a partial output");

    // 取消在单三角形的 bucket 三重循环内也必须可达，不能只在外层按面轮询。
    pathological.vertices[1] = Eigen::Vector3f(200.0f, 0.0f, 0.0f);
    pathological.vertices[2] = Eigen::Vector3f(0.0f, 200.0f, 0.0f);
    std::atomic_int innerChecks{ 0 };
    PointCloudModelDenoiser::Options innerCancelOptions;
    innerCancelOptions.cellSizeMm = 1.0;
    innerCancelOptions.cancelRequested = [&innerChecks]()
    {
        return innerChecks.fetch_add(1) >= 7;
    };
    PointCloudModelDenoiser::Stats innerCancelStats;
    const auto innerCancelOutput = PointCloudModelDenoiser::DenoiseByModelDistance(
        cloud.mid(0, 1), pathological, innerCancelOptions, &innerCancelStats);
    Check(innerCancelStats.cancelled,
          "denoiser did not cancel from inside a triangle bucket-coverage loop");
    Check(innerCancelOutput.isEmpty(), "inner-loop cancellation exposed a partial output");

    WorkpieceMeshBuilder::Mesh denseBucket;
    denseBucket.vertices = {
        Eigen::Vector3f(0.0f, 0.0f, 0.0f),
        Eigen::Vector3f(1.0f, 0.0f, 0.0f),
        Eigen::Vector3f(0.0f, 1.0f, 0.0f)
    };
    constexpr int kBucketLimitPlusOne = 256 * 1024 + 1;
    denseBucket.indices.reserve(kBucketLimitPlusOne * 3);
    for (int i = 0; i < kBucketLimitPlusOne; ++i)
    {
        denseBucket.indices << 0U << 1U << 2U;
    }
    PointCloudModelDenoiser::Options denseOptions;
    denseOptions.cellSizeMm = 10.0;
    PointCloudModelDenoiser::Stats denseStats;
    const auto denseOutput = PointCloudModelDenoiser::DenoiseByModelDistance(
        cloud.mid(0, 1), denseBucket, denseOptions, &denseStats);
    Check(denseStats.resourceLimitExceeded,
          "pathological single bucket did not trip its candidate-count limit");
    Check(denseOutput.isEmpty(), "single-bucket resource failure exposed a partial output");

    // 真实 PLY 加载器：有效文件可读，NaN/越界索引/多余载荷/巨大计数均拒绝。
    QTemporaryDir temp;
    Check(temp.isValid(), "could not create temporary PLY directory");
    QString error;
    WorkpieceMeshBuilder::Mesh loaded;
    const QString validPath = temp.filePath(QStringLiteral("valid.ply"));
    Check(WritePly(validPath, 3, 1), "could not write valid PLY fixture");
    Check(WorkpieceMeshBuilder::LoadMeshPly(validPath, loaded, error),
          "strict PLY loader rejected a valid fixed-schema file");
    Check(loaded.vertices.size() == 3 && loaded.indices.size() == 3,
          "valid PLY returned unexpected mesh sizes");

    float nonFiniteVertex[6] = {
        std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f, 0.0f, 0.0f, 1.0f
    };
    const QString nanPath = temp.filePath(QStringLiteral("nan.ply"));
    Check(WritePly(nanPath, 3, 1, nonFiniteVertex), "could not write NaN PLY fixture");
    Check(!WorkpieceMeshBuilder::LoadMeshPly(nanPath, loaded, error),
          "strict PLY loader accepted a NaN vertex");

    const quint32 outOfRangeFace[3] = { 0, 1, 3 };
    const QString indexPath = temp.filePath(QStringLiteral("bad-index.ply"));
    Check(WritePly(indexPath, 3, 1, nullptr, outOfRangeFace),
          "could not write out-of-range PLY fixture");
    Check(!WorkpieceMeshBuilder::LoadMeshPly(indexPath, loaded, error),
          "strict PLY loader accepted an out-of-range face index");

    const QString trailingPath = temp.filePath(QStringLiteral("trailing.ply"));
    Check(WritePly(trailingPath, 3, 1, nullptr, nullptr, QByteArray("x")),
          "could not write trailing-byte PLY fixture");
    Check(!WorkpieceMeshBuilder::LoadMeshPly(trailingPath, loaded, error),
          "strict PLY loader accepted a trailing payload byte");

    const QString hugePath = temp.filePath(QStringLiteral("huge-count.ply"));
    {
        QFile huge(hugePath);
        Check(huge.open(QIODevice::WriteOnly | QIODevice::Truncate),
              "could not write huge-count PLY fixture");
        Check(huge.write(PlyHeader(2'000'001, 1)) > 0,
              "could not write huge-count PLY header");
    }
    Check(!WorkpieceMeshBuilder::LoadMeshPly(hugePath, loaded, error),
          "strict PLY loader accepted a vertex count above its allocation cap");

    const QString longHeaderPath = temp.filePath(QStringLiteral("long-header.ply"));
    {
        QFile longHeader(longHeaderPath);
        Check(longHeader.open(QIODevice::WriteOnly | QIODevice::Truncate),
              "could not write long-line PLY fixture");
        const QByteArray extra = QByteArray("comment ") + QByteArray(1100, 'x') + '\n';
        Check(longHeader.write(PlyHeader(3, 1, extra)) > 0,
              "could not write long-line PLY header");
    }
    Check(!WorkpieceMeshBuilder::LoadMeshPly(longHeaderPath, loaded, error),
          "strict PLY loader accepted an overlong header line");

    const QString cancelPath = temp.filePath(QStringLiteral("cancel.ply"));
    Check(WritePly(cancelPath, 8193, 1), "could not write cancellable PLY fixture");
    std::atomic_int plyChecks{ 0 };
    const bool loadedAfterCancel = WorkpieceMeshBuilder::LoadMeshPly(
        cancelPath, loaded, error, [&plyChecks]()
        {
            return plyChecks.fetch_add(1) >= 16;
        });
    Check(!loadedAfterCancel && error.contains(QStringLiteral("取消")),
          "PLY loader did not stop cooperatively between bounded vertex chunks");
    Check(!loaded.IsValid(), "cancelled PLY loader exposed a partial mesh");

    // 文本点云加载也是信任边界：输出只在全文件验证后一次发布。
    const QString validCloudPath = temp.filePath(QStringLiteral("valid-cloud.txt"));
    Check(WriteCloud(validCloudPath, 32), "could not write valid cloud fixture");
    QVector<RobotCalculation::IndexedPoint3D> loadedCloud;
    Check(WorkpieceMeshBuilder::LoadCloudPointsWithProgress(
              validCloudPath, loadedCloud, error),
          "bounded cloud loader rejected a valid point cloud");
    Check(loadedCloud.size() == 32 && loadedCloud.first().index == 0,
          "valid cloud returned unexpected points");

    const QString legacyCloudPath = temp.filePath(QStringLiteral("legacy-cloud.txt"));
    Check(WriteLegacyCloud(legacyCloudPath, 16), "could not write legacy cloud fixture");
    Check(WorkpieceMeshBuilder::LoadCloudPointsWithProgress(
              legacyCloudPath, loadedCloud, error),
          "bounded cloud loader rejected the supported 15-column legacy schema");
    Check(loadedCloud.size() == 16
          && std::abs(loadedCloud.first().point.x() - 100.0) < 1e-9
          && std::abs(loadedCloud.first().point.y() - 200.0) < 1e-9,
          "legacy cloud schema mapped frame/line fields into XYZ");

    RobotCalculation::IndexedPoint3D sentinel;
    sentinel.index = 424242;
    QVector<RobotCalculation::IndexedPoint3D> atomicCloud{ sentinel };
    const auto outputUnchanged = [&atomicCloud]()
    {
        return atomicCloud.size() == 1 && atomicCloud.first().index == 424242;
    };

    const QString corruptTextPath = temp.filePath(QStringLiteral("corrupt-text-cloud.txt"));
    Check(WriteCloud(corruptTextPath, 16, QByteArray("corrupted 1 2 3\n")),
          "could not write corrupt nonnumeric cloud fixture");
    Check(!WorkpieceMeshBuilder::LoadCloudPointsWithProgress(
              corruptTextPath, atomicCloud, error),
          "cloud loader silently treated a corrupt nonnumeric data row as a header");
    Check(outputUnchanged(), "corrupt nonnumeric row published partial points");

    WorkpieceMeshBuilder::CloudLoadLimits lineLengthLimits;
    lineLengthLimits.maximumLineBytes = 64;
    const QString longCloudPath = temp.filePath(QStringLiteral("long-cloud-line.txt"));
    Check(WriteCloud(longCloudPath, 16, QByteArray(100, '7')),
          "could not write long-line cloud fixture");
    Check(!WorkpieceMeshBuilder::LoadCloudPointsWithProgress(
              longCloudPath, atomicCloud, error, {}, &lineLengthLimits),
          "cloud loader accepted an overlong physical line");
    Check(outputUnchanged(), "failed long-line load published a partial point cloud");

    const QString manyLinesPath = temp.filePath(QStringLiteral("many-lines-cloud.txt"));
    Check(WriteCloud(manyLinesPath, 17), "could not write physical-line-limit fixture");
    WorkpieceMeshBuilder::CloudLoadLimits physicalLineLimits;
    physicalLineLimits.maximumPhysicalLines = 16;
    Check(!WorkpieceMeshBuilder::LoadCloudPointsWithProgress(
              manyLinesPath, atomicCloud, error, {}, &physicalLineLimits),
          "cloud loader accepted more physical lines than its injected cap");
    Check(outputUnchanged(), "physical-line limit published a partial point cloud");

    WorkpieceMeshBuilder::CloudLoadLimits pointCountLimits;
    pointCountLimits.maximumValidPoints = 16;
    Check(!WorkpieceMeshBuilder::LoadCloudPointsWithProgress(
              manyLinesPath, atomicCloud, error, {}, &pointCountLimits),
          "cloud loader accepted more valid points than its injected cap");
    Check(outputUnchanged(), "valid-point limit published a partial point cloud");

    const QString badIndexCloudPath = temp.filePath(QStringLiteral("bad-index-cloud.txt"));
    Check(WriteCloud(badIndexCloudPath, 16, QByteArray("2147483648 1 2 3\n")),
          "could not write bad-index cloud fixture");
    Check(!WorkpieceMeshBuilder::LoadCloudPointsWithProgress(
              badIndexCloudPath, atomicCloud, error),
          "cloud loader accepted an index outside int range");
    Check(outputUnchanged(), "bad-index load published a partial point cloud");

    const QString nanCloudPath = temp.filePath(QStringLiteral("nan-cloud.txt"));
    Check(WriteCloud(nanCloudPath, 16, QByteArray("17 nan 2 3\n")),
          "could not write NaN cloud fixture");
    Check(!WorkpieceMeshBuilder::LoadCloudPointsWithProgress(
              nanCloudPath, atomicCloud, error),
          "cloud loader accepted a non-finite coordinate");
    Check(outputUnchanged(), "NaN load published a partial point cloud");

    WorkpieceMeshBuilder::CloudLoadLimits cancelCloudLimits;
    cancelCloudLimits.progressPollIntervalLines = 1;
    std::atomic_int cloudProgressChecks{ 0 };
    Check(!WorkpieceMeshBuilder::LoadCloudPointsWithProgress(
              validCloudPath, atomicCloud, error,
              [&cloudProgressChecks](int, const QString&)
              {
                  return cloudProgressChecks.fetch_add(1) < 5;
              }, &cancelCloudLimits),
          "cloud loader ignored cancellation during line parsing");
    Check(error.contains(QStringLiteral("取消")) && outputUnchanged(),
          "cancelled cloud load published partial points or lost cancellation state");

    const QString sparsePath = temp.filePath(QStringLiteral("oversized-sparse-cloud.txt"));
    {
        QFile sparse(sparsePath);
        Check(sparse.open(QIODevice::WriteOnly | QIODevice::Truncate),
              "could not create oversized sparse cloud fixture");
        Check(sparse.resize(768LL * 1024LL * 1024LL + 1),
              "could not resize oversized sparse cloud fixture");
    }
    Check(!WorkpieceMeshBuilder::LoadCloudPointsWithProgress(
              sparsePath, atomicCloud, error),
          "cloud loader did not reject an oversized sparse file before parsing");
    Check(outputUnchanged(), "oversized sparse precheck published a partial point cloud");

    const QString linkPath = temp.filePath(QStringLiteral("cloud-link.lnk"));
    Check(QFile::link(validCloudPath, linkPath), "could not create cloud link fixture");
    Check(!WorkpieceMeshBuilder::LoadCloudPointsWithProgress(
              linkPath, atomicCloud, error),
          "cloud loader accepted a symlink/Windows shortcut path");
    Check(outputUnchanged(), "linked-file rejection published a partial point cloud");

    std::cout << "PASS: model-alignment PLY/cloud bounds, bucket caps, and hot-loop cancellation\n";
    return 0;
}
