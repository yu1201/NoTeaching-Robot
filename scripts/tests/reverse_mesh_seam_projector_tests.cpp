#include "ReverseMeshSeamProjector.h"

#include <QCoreApplication>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

namespace
{
int failures = 0;

void Check(bool condition, const char* message)
{
    if (!condition)
    {
        std::cerr << "FAIL: " << message << '\n';
        ++failures;
    }
}

WorkpieceMeshBuilder::Mesh SyntheticMesh()
{
    WorkpieceMeshBuilder::Mesh mesh;
    for (int station = 0; station <= 40; ++station)
    {
        const double x = static_cast<double>(station);
        const double surfaceZ = -4.0 + 0.2 * std::sin(x * 0.2);
        for (int transverse = -4; transverse <= 4; ++transverse)
        {
            mesh.vertices.push_back(Eigen::Vector3f(
                static_cast<float>(x),
                static_cast<float>(transverse),
                static_cast<float>(surfaceZ - 0.3)));
            mesh.vertices.push_back(Eigen::Vector3f(
                static_cast<float>(x),
                static_cast<float>(transverse),
                static_cast<float>(surfaceZ)));
            mesh.vertices.push_back(Eigen::Vector3f(
                static_cast<float>(x),
                static_cast<float>(transverse),
                static_cast<float>(surfaceZ + 0.3)));
        }
    }
    return mesh;
}

WorkpieceMeshBuilder::Mesh LoadBinaryMesh(const QString& path)
{
#ifdef _WIN32
    std::ifstream input(std::filesystem::path(path.toStdWString()), std::ios::binary);
#else
    std::ifstream input(std::filesystem::path(path.toUtf8().constData()), std::ios::binary);
#endif
    std::string line;
    std::size_t vertexCount = 0;
    while (std::getline(input, line))
    {
        if (line.rfind("element vertex ", 0) == 0)
            vertexCount = static_cast<std::size_t>(std::stoull(line.substr(15)));
        if (line == "end_header") break;
    }
    WorkpieceMeshBuilder::Mesh mesh;
    mesh.vertices.reserve(static_cast<qsizetype>(vertexCount));
    for (std::size_t index = 0; index < vertexCount; ++index)
    {
        float record[6]{};
        input.read(reinterpret_cast<char*>(record), sizeof(record));
        if (!input) break;
        mesh.vertices.push_back(Eigen::Vector3f(record[0], record[1], record[2]));
    }
    return mesh;
}

QVector<Eigen::Vector3d> LoadSeed(const QString& path)
{
#ifdef _WIN32
    std::ifstream input(std::filesystem::path(path.toStdWString()));
#else
    std::ifstream input(std::filesystem::path(path.toUtf8().constData()));
#endif
    QVector<Eigen::Vector3d> seed;
    std::string line;
    while (std::getline(input, line))
    {
        std::replace(line.begin(), line.end(), ',', ' ');
        std::istringstream stream(line);
        int index = 0;
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        if (stream >> index >> x >> y >> z)
            seed.push_back(Eigen::Vector3d(x, y, z));
    }
    return seed;
}
}

int main(int argc, char** argv)
{
    QCoreApplication app(argc, argv);
    const WorkpieceMeshBuilder::Mesh mesh = SyntheticMesh();
    QVector<Eigen::Vector3d> seed;
    for (int index = 0; index <= 20; ++index)
        seed.push_back(Eigen::Vector3d(index * 2.0, 0.0, 0.0));

    ReverseMeshSeamProjector::Result result;
    QString error;
    Check(ReverseMeshSeamProjector::Project(mesh, seed, result, error),
        "synthetic reverse-mesh seed projection should pass");
    Check(result.pathModelMm.size() == seed.size(),
        "projection should preserve seed point count");
    Check(result.fallbackPointCount == 0,
        "dense synthetic surface should not use seed fallback");
    for (int index = 0; index < result.pathModelMm.size(); ++index)
    {
        Check(std::abs(result.pathModelMm.at(index).x() - seed.at(index).x()) < 1.0e-9
                && std::abs(result.pathModelMm.at(index).y() - seed.at(index).y()) < 1.0e-9,
            "projection must preserve seed XY");
        Check(result.pathModelMm.at(index).z() < -3.0
                && result.pathModelMm.at(index).z() > -5.0,
            "projection should move seed Z onto the synthetic surface band");
    }
    ReverseMeshSeamProjector::Result missingSeed;
    Check(!ReverseMeshSeamProjector::Project(
            mesh, QVector<Eigen::Vector3d>(), missingSeed, error)
            && error.contains(QStringLiteral("无种子")),
        "reverse mesh must reject blind extraction without a seed");

    if (app.arguments().size() >= 3)
    {
        const WorkpieceMeshBuilder::Mesh realMesh = LoadBinaryMesh(app.arguments().at(1));
        const QVector<Eigen::Vector3d> realSeed = LoadSeed(app.arguments().at(2));
        ReverseMeshSeamProjector::Result realResult;
        Check(realMesh.vertices.size() == 718233,
            "real reverse PLY should contain the expected vertex count");
        Check(realSeed.size() == 914,
            "real reference seed should contain the expected point count");
        Check(ReverseMeshSeamProjector::Project(
                realMesh, realSeed, realResult, error),
            "real reverse-mesh seed projection should pass");
        Check(realResult.pathModelMm.size() == realSeed.size()
                && realResult.fallbackPointCount == 0,
            "real reverse-mesh projection should cover every seed without fallback");
        std::cout << "real_reverse_mesh vertices=" << realResult.meshVertexCount
                  << " seed=" << realResult.seedPointCount
                  << " output=" << realResult.pathModelMm.size()
                  << " fallback=" << realResult.fallbackPointCount << '\n';
    }

    if (failures == 0)
        std::cout << "PASS: reverse mesh seed projection tests\n";
    return failures == 0 ? 0 : 1;
}
