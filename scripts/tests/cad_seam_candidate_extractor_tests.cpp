#include "CadSeamCandidateExtractor.h"

#include <BRep_Builder.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <DESTEP_Parameters.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <STEPControl_StepModelType.hxx>
#include <STEPControl_Writer.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Shape.hxx>
#include <UnitsMethods_LengthUnit.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QTemporaryDir>
#include <QTextStream>

#include <cmath>
#include <filesystem>
#include <fstream>

namespace
{
int failures = 0;

void Check(bool condition, const QString& message)
{
    if (!condition)
    {
        QTextStream(stderr) << "FAIL: " << message << Qt::endl;
        ++failures;
    }
}

bool WriteStep(const QString& path, const TopoDS_Shape& shape, QString& error)
{
    STEPControl_Writer writer;
    DESTEP_Parameters parameters;
    parameters.WriteUnit = UnitsMethods_LengthUnit_Millimeter;
    if (writer.Transfer(shape, STEPControl_AsIs, parameters) != IFSelect_RetDone)
    {
        error = QStringLiteral("cannot transfer test shape to STEP");
        return false;
    }
#ifdef _WIN32
    std::ofstream output(std::filesystem::path(path.toStdWString()), std::ios::binary);
#else
    std::ofstream output(std::filesystem::path(path.toUtf8().constData()), std::ios::binary);
#endif
    if (!output.is_open() || writer.WriteStream(output) != IFSelect_RetDone)
    {
        error = QStringLiteral("cannot write STEP fixture");
        return false;
    }
    output.close();
    return true;
}

double LongestCandidate(const QVector<CadSeamCandidateExtractor::Candidate>& candidates)
{
    double longest = 0.0;
    for (const auto& candidate : candidates) longest = std::max(longest, candidate.lengthMm);
    return longest;
}

void TestSharedEdges(const QString& path)
{
    QString error;
    Check(WriteStep(path, BRepPrimAPI_MakeBox(30.0, 20.0, 10.0).Shape(), error), error);
    QVector<CadSeamCandidateExtractor::Candidate> candidates;
    CadSeamCandidateExtractor::Statistics statistics;
    CadSeamCandidateExtractor::Options options;
    options.includeSharedEdges = true;
    options.includeRootIntersections = false;
    options.minimumLengthMm = 9.0;
    options.samplingStepMm = 2.0;
    Check(CadSeamCandidateExtractor::ExtractFromStepFile(
        path, candidates, error, &statistics, &options), error);
    Check(statistics.faceCount == 6, QStringLiteral("box should contain six faces"));
    Check(statistics.sharedEdgeCount == 12, QStringLiteral("box should expose twelve shared sharp edges"));
    Check(candidates.size() == 12, QStringLiteral("each disconnected box edge should be one candidate"));
    Check(std::abs(LongestCandidate(candidates) - 30.0) < 0.05,
        QStringLiteral("longest box candidate should be 30 mm"));
    for (const auto& candidate : candidates)
    {
        Check(candidate.sourceKind == CadSeamCandidateExtractor::SourceKind::SharedEdge,
            QStringLiteral("box candidate source should be shared edge"));
        Check(candidate.candidateId.startsWith(QStringLiteral("CS-"))
            && candidate.candidateId.size() == 19,
            QStringLiteral("candidate identity should be deterministic SHA prefix"));
        Check(candidate.pointsModelMm.size() >= 2,
            QStringLiteral("candidate should include sampled points"));
        Check(std::abs(candidate.dihedralDegrees - 90.0) < 0.1,
            QStringLiteral("box edge dihedral should be 90 degrees"));
    }
}

void TestRootIntersection(const QString& path)
{
    const TopoDS_Shape plate = BRepPrimAPI_MakeBox(40.0, 30.0, 2.0).Shape();
    const TopoDS_Shape wall = BRepPrimAPI_MakeBox(
        gp_Pnt(0.0, 14.0, -4.0), gp_Pnt(40.0, 16.0, 12.0)).Shape();
    TopoDS_Compound assembly;
    BRep_Builder builder;
    builder.MakeCompound(assembly);
    builder.Add(assembly, plate);
    builder.Add(assembly, wall);
    QString error;
    Check(WriteStep(path, assembly, error), error);

    QVector<CadSeamCandidateExtractor::Candidate> candidates;
    CadSeamCandidateExtractor::Statistics statistics;
    CadSeamCandidateExtractor::Options options;
    options.includeSharedEdges = false;
    options.minimumLengthMm = 5.0;
    options.samplingStepMm = 2.0;
    Check(CadSeamCandidateExtractor::ExtractFromStepFile(
        path, candidates, error, &statistics, &options), error);
    Check(statistics.rootCount == 2, QStringLiteral("assembly should contain two solids"));
    Check(statistics.rootPairCount == 1, QStringLiteral("two solids should produce one root pair"));
    Check(statistics.intersectedRootPairCount == 1,
        QStringLiteral("overlapping solids should produce an intersection"));
    Check(!candidates.isEmpty(), QStringLiteral("solid intersection should generate candidates"));
    bool hasLongIntersection = false;
    for (const auto& candidate : candidates)
    {
        if (candidate.sourceKind == CadSeamCandidateExtractor::SourceKind::ShapeIntersection
            && candidate.lengthMm >= 39.9)
        {
            hasLongIntersection = true;
        }
    }
    Check(hasLongIntersection, QStringLiteral("assembly should contain a 40 mm section candidate"));
}

void TestInvalidAndTightLimits(const QString& invalidPath, const QString& validPath)
{
    QFile invalid(invalidPath);
    Check(invalid.open(QIODevice::WriteOnly), QStringLiteral("cannot create invalid fixture"));
    invalid.write("not a STEP file\n");
    invalid.close();
    QVector<CadSeamCandidateExtractor::Candidate> candidates;
    QString error;
    Check(!CadSeamCandidateExtractor::ExtractFromStepFile(invalidPath, candidates, error),
        QStringLiteral("invalid STEP must fail"));
    Check(candidates.isEmpty() && !error.isEmpty(),
        QStringLiteral("invalid STEP must clear candidates and explain failure"));

    CadSeamCandidateExtractor::Options tight;
    tight.maximumFaces = 5;
    Check(!CadSeamCandidateExtractor::ExtractFromStepFile(
        validPath, candidates, error, nullptr, &tight),
        QStringLiteral("face limit should reject a six-face box"));
    Check(candidates.isEmpty() && error.contains(QStringLiteral("面数量")),
        QStringLiteral("face-limit rejection should fail closed with evidence"));
}
}

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);
    QTemporaryDir temporaryDir;
    if (!temporaryDir.isValid())
    {
        QTextStream(stderr) << "FAIL: cannot create temporary directory" << Qt::endl;
        return 1;
    }
    const QString boxPath = temporaryDir.filePath(QStringLiteral("box.step"));
    TestSharedEdges(boxPath);
    TestRootIntersection(temporaryDir.filePath(QStringLiteral("assembly.step")));
    TestInvalidAndTightLimits(
        temporaryDir.filePath(QStringLiteral("invalid.step")), boxPath);
    if (app.arguments().size() >= 2)
    {
        QVector<CadSeamCandidateExtractor::Candidate> realCandidates;
        CadSeamCandidateExtractor::Statistics realStatistics;
        CadSeamCandidateExtractor::Options realOptions;
        realOptions.maximumRootPairs = 4096;
        realOptions.maximumPointsPerCandidate = 20000;
        if (app.arguments().size() >= 3)
        {
            bool ok = false;
            const double minimumLength = app.arguments().at(2).toDouble(&ok);
            if (ok) realOptions.minimumLengthMm = minimumLength;
        }
        QString realError;
        const bool extractedRealStep = CadSeamCandidateExtractor::ExtractFromStepFile(
            app.arguments().at(1), realCandidates, realError, &realStatistics, &realOptions);
        Check(extractedRealStep,
            QStringLiteral("real STEP extraction failed: %1").arg(realError));
        QTextStream(stdout)
            << "real_step roots=" << realStatistics.rootCount
            << " faces=" << realStatistics.faceCount
            << " shared_edges=" << realStatistics.sharedEdgeCount
            << " root_pairs=" << realStatistics.rootPairCount
            << " intersected_pairs=" << realStatistics.intersectedRootPairCount
            << " candidates=" << realCandidates.size()
            << " rejected_short=" << realStatistics.rejectedShortCount
            << " rejected_tangent=" << realStatistics.rejectedTangentCount
            << " duplicates=" << realStatistics.duplicateCount << Qt::endl;
        int approximatelyClosedCount = 0;
        for (const auto& candidate : realCandidates)
        {
            if (candidate.pointsModelMm.size() >= 2
                && (candidate.pointsModelMm.front() - candidate.pointsModelMm.back()).norm()
                    <= realOptions.linearToleranceMm * 2.0)
            {
                ++approximatelyClosedCount;
            }
        }
        QTextStream(stdout) << "real_step approximately_closed="
            << approximatelyClosedCount << Qt::endl;
        if (QFileInfo(app.arguments().at(1)).fileName().contains(QStringLiteral("海星箱体模拟")))
        {
            int buttCount = 0;
            int baseCount = 0;
            int sharedCount = 0;
            for (const auto& candidate : realCandidates)
            {
                if (candidate.sourceKind
                    == CadSeamCandidateExtractor::SourceKind::CorrugatedButtJoint)
                    ++buttCount;
                else if (candidate.sourceKind
                    == CadSeamCandidateExtractor::SourceKind::CorrugatedBaseJoint)
                    ++baseCount;
                else if (candidate.sourceKind
                    == CadSeamCandidateExtractor::SourceKind::SharedEdge)
                    ++sharedCount;
            }
            Check(realStatistics.assemblySemanticsUsed,
                QStringLiteral("real corrugated fixture must use assembly seam semantics"));
            Check(realCandidates.size() == 8 && buttCount == 6 && baseCount == 2,
                QStringLiteral("real corrugated fixture must contain six butt and two base seams"));
            Check(sharedCount == 0 && realStatistics.sharedEdgeCount == 0,
                QStringLiteral("corrugated fold edges must never appear as weld candidates"));
        }
        const int previewCount = std::min(10, static_cast<int>(realCandidates.size()));
        for (int index = 0; index < previewCount; ++index)
        {
            const auto& candidate = realCandidates.at(index);
            Eigen::Vector3d minimum = candidate.pointsModelMm.front();
            Eigen::Vector3d maximum = candidate.pointsModelMm.front();
            for (const Eigen::Vector3d& point : candidate.pointsModelMm)
            {
                minimum = minimum.cwiseMin(point);
                maximum = maximum.cwiseMax(point);
            }
            QString kind = QStringLiteral("section");
            if (candidate.sourceKind == CadSeamCandidateExtractor::SourceKind::SharedEdge)
                kind = QStringLiteral("shared");
            else if (candidate.sourceKind == CadSeamCandidateExtractor::SourceKind::CorrugatedButtJoint)
                kind = QStringLiteral("butt");
            else if (candidate.sourceKind == CadSeamCandidateExtractor::SourceKind::CorrugatedBaseJoint)
                kind = QStringLiteral("base");
            QTextStream(stdout)
                << "  " << candidate.candidateId
                << " kind=" << kind
                << " length_mm=" << candidate.lengthMm
                << " points=" << candidate.pointsModelMm.size()
                << " roots=" << candidate.firstRootIndex << "/" << candidate.secondRootIndex
                << " min=(" << minimum.x() << "," << minimum.y() << "," << minimum.z() << ")"
                << " max=(" << maximum.x() << "," << maximum.y() << "," << maximum.z() << ")"
                << " dihedral_deg=" << candidate.dihedralDegrees << Qt::endl;
        }
    }
    if (failures == 0)
        QTextStream(stdout) << "CadSeamCandidateExtractor tests passed." << Qt::endl;
    return failures == 0 ? 0 : 1;
}
