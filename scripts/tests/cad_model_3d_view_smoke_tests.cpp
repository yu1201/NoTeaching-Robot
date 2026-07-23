#include "CadModel3DView.h"

#include <QApplication>
#include <QCryptographicHash>
#include <QElapsedTimer>
#include <QEventLoop>
#include <QFile>
#include <QFileInfo>
#include <QImage>
#include <QMouseEvent>
#include <QPixmap>
#include <QScreen>
#include <QTemporaryDir>
#include <QThread>
#include <QTimer>
#include <QToolButton>
#include <QVBoxLayout>

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <utility>

namespace
{
bool Require(bool condition, const char* message)
{
    if (condition) return true;
    std::cerr << "FAIL: " << message << '\n';
    return false;
}

QString FileSha256(const QString& path)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) return QString();
    QCryptographicHash hash(QCryptographicHash::Sha256);
    while (!file.atEnd())
    {
        const QByteArray chunk = file.read(1024 * 1024);
        if (chunk.isEmpty() && file.error() != QFileDevice::NoError)
            return QString();
        hash.addData(chunk);
    }
    return QString::fromLatin1(hash.result().toHex());
}

cadview::Vec3d Add(const cadview::Vec3d& left, const cadview::Vec3d& right)
{
    return { left.x + right.x, left.y + right.y, left.z + right.z };
}

cadview::Vec3d Subtract(const cadview::Vec3d& left, const cadview::Vec3d& right)
{
    return { left.x - right.x, left.y - right.y, left.z - right.z };
}

cadview::Vec3d Multiply(const cadview::Vec3d& value, double scale)
{
    return { value.x * scale, value.y * scale, value.z * scale };
}

double Dot(const cadview::Vec3d& left, const cadview::Vec3d& right)
{
    return left.x * right.x + left.y * right.y + left.z * right.z;
}

double Length(const cadview::Vec3d& value)
{
    return std::sqrt(Dot(value, value));
}

cadview::Vec3d Normalize(const cadview::Vec3d& value)
{
    const double length = Length(value);
    return Multiply(value, 1.0 / length);
}

cadview::Vec3d RotateInPlane(
    const cadview::Vec3d& axisX,
    const cadview::Vec3d& axisY,
    double degrees)
{
    const double radians = degrees * std::acos(-1.0) / 180.0;
    return Normalize(Add(
        Multiply(axisX, std::cos(radians)),
        Multiply(axisY, std::sin(radians))));
}

cadview::Vec3d Cross(const cadview::Vec3d& left, const cadview::Vec3d& right)
{
    return {
        left.y * right.z - left.z * right.y,
        left.z * right.x - left.x * right.z,
        left.x * right.y - left.y * right.x
    };
}

void ProcessFor(int milliseconds)
{
    QEventLoop loop;
    QTimer::singleShot(milliseconds, &loop, &QEventLoop::quit);
    loop.exec();
}

RobotCollisionEnvelopeStore::EnvelopeSet SyntheticRobotCollisionEnvelope(
    const Eigen::Vector3d& sourceUp = Eigen::Vector3d::UnitY())
{
    RobotCadAssemblyLoader::Result loaded;
    loaded.statistics.sourceSha256 = QString(64, QLatin1Char('a'));
    loaded.statistics.sourceLengthUnits = { QStringLiteral("mm") };
    loaded.base.valid = true;
    loaded.base.sourceUp = sourceUp;
    loaded.base.minimumYmm = 0.0;
    loaded.base.conservativeBaseCenterMm = Eigen::Vector3d::Zero();
    for (int index = 0; index < 7; ++index)
    {
        RobotCadAssemblyLoader::ComponentStatistics component;
        component.jointIndex = index;
        component.included = true;
        component.assemblyPath = QStringLiteral("synthetic/J%1").arg(index);
        component.boundsMm.valid = true;
        const double halfWidth = 245.0 - index * 18.0;
        const double minimumY = index == 0 ? 0.0 : 260.0 + index * 245.0;
        const double maximumY = minimumY + (index == 0 ? 430.0 : 360.0);
        const double minimumZ = -230.0 + index * 55.0;
        const double maximumZ = 230.0 + index * 95.0;
        component.boundsMm.minimumMm = Eigen::Vector3d(
            -halfWidth, minimumY, minimumZ);
        component.boundsMm.maximumMm = Eigen::Vector3d(
            halfWidth, maximumY, maximumZ);
        loaded.statistics.components.append(component);
        if (index == 0) loaded.base.j0BoundsMm = component.boundsMm;
    }
    RobotCollisionEnvelopeStore::GenerationParameters parameters;
    RobotCollisionEnvelopeStore::EnvelopeSet envelope;
    QString error;
    RobotCollisionEnvelopeStore::Generate(loaded, parameters, envelope, error);
    return envelope;
}

bool ViewportCoversNativeWindow(const QImage& image)
{
    if (image.isNull() || image.width() < 20 || image.height() < 20) return false;
    const int xs[] = { image.width() / 20, image.width() * 19 / 20 };
    const int ys[] = { image.height() / 20, image.height() * 19 / 20 };
    for (const int x : xs)
    {
        for (const int y : ys)
        {
            const QColor pixel = image.pixelColor(x, y);
            if (pixel.red() + pixel.green() + pixel.blue() <= 12) return false;
        }
    }
    return true;
}

bool ScreenshotContainsCadAndLabels(const QImage& image)
{
    qsizetype modelPixels = 0;
    qsizetype redLabelPixels = 0;
    qsizetype greenLabelPixels = 0;
    qsizetype fixturePixels = 0;
    qsizetype scanRegionPixels = 0;
    for (int y = image.height() / 10; y < image.height(); ++y)
    {
        for (int x = 0; x < image.width(); ++x)
        {
            const QColor pixel = image.pixelColor(x, y);
            if (pixel.blue() > pixel.red() + 8 && pixel.red() < 230)
                ++modelPixels;
            if (pixel.red() > 180 && pixel.green() < 150 && pixel.blue() < 150)
                ++redLabelPixels;
            if (pixel.green() > 115 && pixel.red() < 170 && pixel.blue() < 170)
                ++greenLabelPixels;
            if (pixel.red() > 180 && pixel.green() > 75 && pixel.green() < 205
                && pixel.blue() < 105)
                ++fixturePixels;
            if (pixel.red() > 185 && pixel.green() > 155 && pixel.blue() < 145)
                ++scanRegionPixels;
        }
    }
    const qsizetype pixels = static_cast<qsizetype>(image.width()) * image.height();
    return modelPixels > pixels / 50
        && redLabelPixels > 20
        && greenLabelPixels > 20
        && fixturePixels > 30
        && scanRegionPixels > 30;
}

void SendMouseEvent(
    QWidget& target,
    QEvent::Type type,
    const QPointF& localPosition,
    Qt::MouseButton button,
    Qt::MouseButtons buttons)
{
    const QPoint globalPoint = target.mapToGlobal(localPosition.toPoint());
    QMouseEvent event(
        type,
        localPosition,
        QPointF(globalPoint),
        button,
        buttons,
        Qt::NoModifier);
    QApplication::sendEvent(&target, &event);
}

bool CorruptSamePathAndRequireClear(
    cadview::CadModel3DView& view,
    const QString& sourcePath,
    const QString& workingPath,
    bool preserveView,
    bool retainSizeAndModifiedTime)
{
    QFile::remove(workingPath);
    if (!Require(QFile::copy(sourcePath, workingPath), "failed to prepare same-path STEP fixture"))
        return false;
    QFile::setPermissions(workingPath,
        QFile::permissions(workingPath) | QFileDevice::WriteOwner | QFileDevice::WriteUser);
    QFile sourceHashFile(workingPath);
    if (!Require(sourceHashFile.open(QIODevice::ReadOnly),
            "same-path fixture could not be hashed")) return false;
    const QString expectedSha256 = QString::fromLatin1(
        QCryptographicHash::hash(sourceHashFile.readAll(), QCryptographicHash::Sha256).toHex());
    sourceHashFile.close();
    QString error;
    if (!Require(view.SetStepFile(workingPath, expectedSha256, error, false),
            "same-path fixture could not be loaded")) return false;
    const QFileInfo originalInfo(workingPath);
    QFile damaged(workingPath);
    if (!Require(damaged.open(QIODevice::WriteOnly | QIODevice::Truncate),
            "same-path fixture could not be replaced")) return false;
    if (retainSizeAndModifiedTime)
    {
        const QByteArray replacement(originalInfo.size(), 'X');
        if (!Require(damaged.write(replacement) == replacement.size(),
                "same-size replacement could not be written")) return false;
        if (!Require(damaged.flush(), "same-size replacement could not be flushed")) return false;
        if (!Require(damaged.setFileTime(
                originalInfo.lastModified(), QFileDevice::FileModificationTime),
                "same-size replacement timestamp could not be restored")) return false;
    }
    else
    {
        damaged.write("not a STEP file\n");
    }
    damaged.close();
    if (retainSizeAndModifiedTime)
    {
        const QFileInfo replacedInfo(workingPath);
        if (!Require(replacedInfo.size() == originalInfo.size()
                && replacedInfo.lastModified().toMSecsSinceEpoch()
                    == originalInfo.lastModified().toMSecsSinceEpoch(),
                "test fixture did not retain size/mtime")) return false;
    }
    if (!Require(!view.SetStepFile(
            workingPath, expectedSha256, error, preserveView),
            "replaced same-path STEP unexpectedly reused stale B-Rep")) return false;
    if (!Require(!view.HasCadShape(), "same-path failure retained stale B-Rep shape")) return false;
    if (!Require(!view.HasDisplayedShape(), "same-path failure retained stale AIS display")) return false;
    return Require(view.LoadedSourcePath().isEmpty(),
        "same-path failure retained stale source path");
}
}

int main(int argc, char* argv[])
{
    QApplication application(argc, argv);
    if (argc < 3)
    {
        std::cerr << "usage: CadModel3DViewSmokeTests <source.step> <screenshot.png>\n";
        return 2;
    }

    const QStringList arguments = application.arguments();
    const QString sourcePath = arguments.at(1);
    const QString screenshotPath = arguments.at(2);
    const QString robotStepPath = arguments.size() >= 4 ? arguments.at(3) : QString();
    if (!Require(QFileInfo::exists(sourcePath), "real STEP fixture is missing")) return 1;

    // 非循环验证：直接提供已知矩形外角，以及模拟孔轮廓、加强筋和边上重复点。
    // 期望值由几何定义给出，不从被测结果反推；吸附点只能留下四个外轮廓角。
    const QVector<cadview::Vec3d> syntheticSupportVertices = {
        { 0.0, 0.0, 25.0 }, { 1000.0, 0.0, 25.0 },
        { 1000.0, 600.0, 25.0 }, { 0.0, 600.0, 25.0 },
        { 500.0, 0.0, 25.0 }, { 1000.0, 300.0, 25.0 },
        { 0.0, 0.0, 25.0 + 1.0e-8 },
        // 内部加强筋顶点。
        { 180.0, 110.0, 25.0 }, { 820.0, 110.0, 25.0 },
        { 820.0, 490.0, 25.0 }, { 180.0, 490.0, 25.0 },
        // 孔轮廓顶点。
        { 430.0, 260.0, 25.0 }, { 570.0, 260.0, 25.0 },
        { 570.0, 340.0, 25.0 }, { 430.0, 340.0, 25.0 }
    };
    const QVector<cadview::Vec3d> syntheticSnapPoints =
        cadview::CadModel3DView::BuildGroundSnapHullForTesting(
            syntheticSupportVertices, { 0.0, 0.0, 25.0 }, { 0.0, 0.0, 1.0 });
    if (!Require(syntheticSnapPoints.size() == 4,
            "ground snap hull retained a hole/interior/collinear vertex")) return 1;
    const QVector<cadview::Vec3d> expectedOuterCorners = {
        { 0.0, 0.0, 25.0 }, { 1000.0, 0.0, 25.0 },
        { 1000.0, 600.0, 25.0 }, { 0.0, 600.0, 25.0 }
    };
    for (const cadview::Vec3d& actual : syntheticSnapPoints)
    {
        bool isOuterCorner = false;
        for (const cadview::Vec3d& expected : expectedOuterCorners)
        {
            if (Length(Subtract(actual, expected)) <= 1.0e-5)
            {
                isOuterCorner = true;
                break;
            }
        }
        if (!Require(isOuterCorner,
                "ground snap hull returned an internal or hole point")) return 1;
    }

    QWidget window;
    window.setWindowTitle(QStringLiteral("原始 STEP/B-Rep 外显冒烟测试"));
    window.setWindowFlag(Qt::WindowStaysOnTopHint, true);
    window.resize(1100, 700);
    QVBoxLayout layout(&window);
    cadview::CadModel3DView view(&window);
    layout.addWidget(&view);

    QString error;
    if (!Require(view.SetStepFile(sourcePath, error), error.toLocal8Bit().constData())) return 1;
    if (!Require(view.HasCadShape(), "STEP did not produce a B-Rep shape")) return 1;
    if (!Require(view.LoadedFaceCount() > 0, "STEP B-Rep contains no faces")) return 1;
    const QVector<cadview::GroundFaceCandidate> groundFaces = view.GroundFaceCandidates();
    if (!Require(!groundFaces.isEmpty(), "STEP produced no large supporting planar face")) return 1;
    // 该回归模型有六个主要外包络支撑方向（±X/±Y/±Z）。内部隔层面积更大，
    // 但不在全模型投影极值，绝不能混入候选。
    if (!Require(groundFaces.size() == 6,
            "support-face filter did not isolate the six outer-envelope directions")) return 1;
    if (!Require(groundFaces.first().areaMm2 > 1000.0,
            "largest supporting planar face area is implausibly small")) return 1;
    for (const cadview::GroundFaceCandidate& candidate : groundFaces)
    {
        if (!Require(candidate.largestFaceAreaMm2 > 0.0
                && candidate.largestFaceAreaMm2 <= candidate.areaMm2 + 1.0e-6,
                "support candidate does not contain one real large B-Rep face")) return 1;
        if (!Require(candidate.supportToleranceMm >= 0.05
                && candidate.supportToleranceMm <= 1.0,
                "support candidate tolerance is outside the safety bound")) return 1;
    }
    const int supportCandidateIndex = 0;
    const QVector<cadview::Vec3d> snapPoints =
        view.GroundFaceSnapPoints(supportCandidateIndex);
    if (!Require(snapPoints.size() >= 3,
            "support face exposes no projected outer-hull points")) return 1;
    view.SetGroundFaceCandidateHighlight(supportCandidateIndex);

    cadview::VSlotFixture fixture;
    fixture.visible = true;
    fixture.anchor = groundFaces.at(supportCandidateIndex).centerModel;
    fixture.axisZ = Normalize(groundFaces.at(supportCandidateIndex).groundUpModel);
    // 用外轮廓最长边建立粗放置轴。+Y 指向该边左侧（凸包内部），
    // 后续吸附使用 +X/+Y 最小支撑值，因此倒角工件也会卡到两条内壁。
    int longestEdgeStart = -1;
    double longestEdgeLength = 0.0;
    for (qsizetype index = 0; index < snapPoints.size(); ++index)
    {
        const double edgeLength = Length(Subtract(
            snapPoints.at((index + 1) % snapPoints.size()), snapPoints.at(index)));
        if (edgeLength > longestEdgeLength)
        {
            longestEdgeLength = edgeLength;
            longestEdgeStart = static_cast<int>(index);
        }
    }
    if (!Require(longestEdgeStart >= 0 && longestEdgeLength > 1.0,
            "support hull has no usable placement edge")) return 1;
    const cadview::Vec3d workpieceEdgeAxisX = Normalize(Subtract(
        snapPoints.at((longestEdgeStart + 1) % snapPoints.size()),
        snapPoints.at(longestEdgeStart)));
    const cadview::Vec3d workpieceEdgeAxisY = Normalize(Cross(
        fixture.axisZ, workpieceEdgeAxisX));
    // 模拟现场先粗转 V 槽：比真实工件长边偏 7°。拖近外角后视图必须
    // 同时修正位置与方向，而不是只把一个错误朝向的 anchor 拉到角点。
    fixture.axisX = RotateInPlane(
        workpieceEdgeAxisX, workpieceEdgeAxisY, 7.0);
    fixture.axisY = Normalize(Cross(fixture.axisZ, fixture.axisX));
    fixture.longLengthMm = 420.0;
    fixture.shortLengthMm = 260.0;
    fixture.wallThicknessMm = 26.0;
    fixture.wallHeightMm = 70.0;
    cadview::ScanRegion scanRegion;
    scanRegion.center = Add(fixture.anchor,
        Add(Multiply(fixture.axisX, 600.0), Multiply(fixture.axisY, 250.0)));
    scanRegion.outwardNormal = fixture.axisZ;
    scanRegion.label = QStringLiteral("S01");
    scanRegion.color = QColor(255, 214, 64);
    scanRegion.radiusMm = 95.0;
    scanRegion.thicknessMm = 4.0;
    scanRegion.selected = true;
    cadview::StationMarker stationMarker;
    stationMarker.position = scanRegion.center;
    stationMarker.label = QStringLiteral("S01 人工扫描彩色区域");
    stationMarker.color = scanRegion.color;
    stationMarker.size = 10.0;
    view.SetSceneOverlays(fixture, { scanRegion }, {}, { stationMarker });

    // 模拟真实启动顺序：小型碰撞 JSON 可先于工件大面/地面就绪。此时先
    // 接管但不显示；地面首次成立后必须自动摆放并重新 Fit，而不是留在视口外。
    const RobotCollisionEnvelopeStore::EnvelopeSet syntheticEnvelope =
        SyntheticRobotCollisionEnvelope();
    QString collisionError;
    if (!Require(view.SetRobotCollisionEnvelope(
            syntheticEnvelope, collisionError, false),
            qPrintable(QStringLiteral("pre-ground collision envelope failed: %1")
                .arg(collisionError)))) return 1;
    if (!Require(!view.HasDisplayedTheoreticalRobot(),
            "collision envelope was displayed before verified ground")) return 1;

    bool dragCommitted = false;
    bool snapStateCommitted = false;
    bool poseCommitted = false;
    bool committedPoseSnapped = false;
    cadview::Vec3d draggedAnchor;
    cadview::VSlotFixture committedPose;
    view.SetVSlotMovedCallback([&](const cadview::Vec3d& anchor)
        {
            dragCommitted = true;
            draggedAnchor = anchor;
        });
    view.SetVSlotPoseChangedCallback(
        [&](const cadview::VSlotFixture& pose, bool snapped)
        {
            poseCommitted = true;
            committedPoseSnapped = snapped;
            committedPose = pose;
            dragCommitted = true;
            draggedAnchor = pose.anchor;
        });
    view.SetVSlotSnapStateCallback([&](bool snapped)
        {
            snapStateCommitted = snapped;
        });
    QVector<double> workpieceRotationRequests;
    QVector<double> vSlotRotationRequests;
    view.SetWorkpieceRotationRequestedCallback([&](double degrees)
        {
            workpieceRotationRequests.push_back(degrees);
        });
    view.SetVSlotRotationRequestedCallback([&](double degrees)
        {
            vSlotRotationRequests.push_back(degrees);
        });
    view.SetRotationStepDegrees(7.5);
    QToolButton* const workpieceRotateMinus =
        view.findChild<QToolButton*>(QStringLiteral("workpieceRotateMinusButton"));
    QToolButton* const workpieceRotatePlus =
        view.findChild<QToolButton*>(QStringLiteral("workpieceRotatePlusButton"));
    QToolButton* const vSlotRotateMinus =
        view.findChild<QToolButton*>(QStringLiteral("vSlotRotateMinusButton"));
    QToolButton* const vSlotRotatePlus =
        view.findChild<QToolButton*>(QStringLiteral("vSlotRotatePlusButton"));
    if (!Require(workpieceRotateMinus != nullptr && workpieceRotatePlus != nullptr
            && vSlotRotateMinus != nullptr && vSlotRotatePlus != nullptr,
            "in-view rotation controls are missing")) return 1;

    window.show();
    window.raise();
    window.activateWindow();
    ProcessFor(1200);
    if (!Require(view.HasDisplayedShape(), "AIS_Shape is not displayed by the OCCT context")) return 1;
    if (!Require(view.HasDisplayedVSlot(), "solid V-slot AIS shape is not displayed")) return 1;
    if (!Require(!view.HasDisplayedGround(),
            "ground was displayed before a supporting face was applied")) return 1;
    if (!Require(!workpieceRotateMinus->isVisible()
            && !workpieceRotatePlus->isVisible()
            && !vSlotRotateMinus->isVisible()
            && !vSlotRotatePlus->isVisible(),
            "rotation controls were visible before verified ground")) return 1;
    if (!Require(view.DisplayedScanRegionCount() == 1,
            "colored scan region is not displayed")) return 1;
    if (!Require(view.AnnotationLabelCount() == 3,
            "Qt overlay did not receive V-slot and scan-region labels")) return 1;

    const cadview::Vec3d clickOffset = Add(
        Multiply(fixture.axisX, 70.0), Multiply(fixture.axisY, -13.0));
    const cadview::Vec3d clickPoint = Add(
        Add(fixture.anchor, clickOffset), Multiply(fixture.axisZ, 35.0));
    QPointF lockedStart;
    if (!Require(view.ProjectModelPoint(clickPoint, lockedStart),
            "could not project locked V-slot")) return 1;
    SendMouseEvent(view, QEvent::MouseButtonPress, lockedStart,
        Qt::LeftButton, Qt::LeftButton);
    SendMouseEvent(view, QEvent::MouseMove, lockedStart + QPointF(30.0, 20.0),
        Qt::NoButton, Qt::LeftButton);
    SendMouseEvent(view, QEvent::MouseButtonRelease, lockedStart + QPointF(30.0, 20.0),
        Qt::LeftButton, Qt::NoButton);
    ProcessFor(100);
    if (!Require(!dragCommitted,
            "V-slot moved before a supporting ground face was applied")) return 1;

    cadview::GroundSurface ground;
    ground.visible = true;
    ground.workpieceRotatable = true;
    ground.planePoint = groundFaces.at(supportCandidateIndex).centerModel;
    ground.axisX = workpieceEdgeAxisX;
    ground.axisY = workpieceEdgeAxisY;
    ground.axisZ = fixture.axisZ;
    ground.supportCandidateIndex = supportCandidateIndex;
    ground.snapDistanceMm = 120.0;
    view.SetSceneOverlays(ground, fixture, { scanRegion }, {}, { stationMarker });
    ProcessFor(250);
    if (!Require(view.HasDisplayedGround(),
            "verified ground slab is not displayed")) return 1;
    if (!Require(view.GroundFaceCandidates().size() == groundFaces.size(),
            "ground overlay polluted supporting-face analysis")) return 1;
    if (!Require(view.HasDisplayedTheoreticalRobot(),
            "preloaded collision envelope did not appear when ground became valid")) return 1;
    if (!Require(workpieceRotateMinus->isVisible()
            && workpieceRotatePlus->isVisible()
            && vSlotRotateMinus->isVisible()
            && vSlotRotatePlus->isVisible()
            && workpieceRotateMinus->isEnabled()
            && workpieceRotatePlus->isEnabled()
            && vSlotRotateMinus->isEnabled()
            && vSlotRotatePlus->isEnabled(),
            "verified ground did not enable both in-view rotation controls")) return 1;
    workpieceRotateMinus->click();
    workpieceRotatePlus->click();
    vSlotRotateMinus->click();
    vSlotRotatePlus->click();
    if (!Require(workpieceRotationRequests.size() == 2
            && std::abs(workpieceRotationRequests.at(0) + 7.5) < 1.0e-9
            && std::abs(workpieceRotationRequests.at(1) - 7.5) < 1.0e-9
            && vSlotRotationRequests.size() == 2
            && std::abs(vSlotRotationRequests.at(0) + 7.5) < 1.0e-9
            && std::abs(vSlotRotationRequests.at(1) - 7.5) < 1.0e-9,
            "in-view rotation controls emitted the wrong direction or step")) return 1;

    // 已审核继承工装必须是只读显示；工件旋转仍可用，但 V 槽旋转按钮
    // 必须禁用或隐藏，不能从视图绕过 draggable 门禁。
    fixture.draggable = false;
    view.SetSceneOverlays(ground, fixture, { scanRegion }, {}, { stationMarker });
    ProcessFor(100);
    if (!Require((!vSlotRotateMinus->isVisible() || !vSlotRotateMinus->isEnabled())
            && (!vSlotRotatePlus->isVisible() || !vSlotRotatePlus->isEnabled()),
            "locked V-slot retained active in-view rotation controls")) return 1;
    fixture.draggable = true;
    view.SetSceneOverlays(ground, fixture, { scanRegion }, {}, { stationMarker });
    ProcessFor(100);
    if (!Require(vSlotRotateMinus->isEnabled() && vSlotRotatePlus->isEnabled(),
            "unlocked V-slot rotation controls did not recover")) return 1;

    double minimumFixtureX = (std::numeric_limits<double>::max)();
    double minimumFixtureY = (std::numeric_limits<double>::max)();
    for (const cadview::Vec3d& point : snapPoints)
    {
        minimumFixtureX = std::min(minimumFixtureX, Dot(point, workpieceEdgeAxisX));
        minimumFixtureY = std::min(minimumFixtureY, Dot(point, workpieceEdgeAxisY));
    }
    const cadview::Vec3d targetSnapPoint = Add(ground.planePoint,
        Add(Multiply(workpieceEdgeAxisX,
                minimumFixtureX - Dot(ground.planePoint, workpieceEdgeAxisX)),
            Multiply(workpieceEdgeAxisY,
                minimumFixtureY - Dot(ground.planePoint, workpieceEdgeAxisY))));
    QPointF dragStart;
    QPointF dragEnd;
    if (!Require(view.ProjectModelPoint(clickPoint, dragStart),
            "could not project enabled V-slot drag start")) return 1;
    const cadview::Vec3d dragDestination = Add(
        Add(targetSnapPoint, clickOffset), Multiply(workpieceEdgeAxisX, 10.0));
    if (!Require(view.ProjectModelPoint(dragDestination, dragEnd),
            "could not project V-slot corner-snap destination")) return 1;
    poseCommitted = false;
    committedPoseSnapped = false;
    SendMouseEvent(view, QEvent::MouseButtonPress, dragStart,
        Qt::LeftButton, Qt::LeftButton);
    SendMouseEvent(view, QEvent::MouseMove, dragEnd,
        Qt::NoButton, Qt::LeftButton);
    SendMouseEvent(view, QEvent::MouseButtonRelease, dragEnd,
        Qt::LeftButton, Qt::NoButton);
    ProcessFor(250);
    if (!Require(dragCommitted, "solid V-slot drag did not commit through callback")) return 1;
    if (!Require(std::abs(Dot(
                Subtract(draggedAnchor, ground.planePoint), fixture.axisZ)) < 1.0e-3,
            "V-slot drag did not stay explicitly projected onto ground")) return 1;
    if (!Require(Length(Subtract(targetSnapPoint, draggedAnchor)) < 2.0,
            "V-slot release did not snap to the +X/+Y support corner")) return 1;
    if (!Require(snapStateCommitted && view.IsVSlotSnappedToWorkpiece(),
            "V-slot corner contact did not persist the workpiece-snap state")) return 1;
    if (!Require(poseCommitted && committedPoseSnapped,
            "V-slot snap did not commit its complete pose")) return 1;
    if (!Require(Dot(committedPose.axisX, workpieceEdgeAxisX) > 1.0 - 1.0e-8,
            "V-slot within 12 degrees was not aligned to the workpiece edge")) return 1;
    if (!Require(std::abs(Length(committedPose.axisX) - 1.0) < 1.0e-8
            && std::abs(Length(committedPose.axisY) - 1.0) < 1.0e-8
            && std::abs(Length(committedPose.axisZ) - 1.0) < 1.0e-8
            && std::abs(Dot(committedPose.axisX, committedPose.axisY)) < 1.0e-8
            && std::abs(Dot(committedPose.axisX, committedPose.axisZ)) < 1.0e-8
            && std::abs(Dot(committedPose.axisY, committedPose.axisZ)) < 1.0e-8
            && Dot(Cross(committedPose.axisX, committedPose.axisY),
                committedPose.axisZ) > 1.0 - 1.0e-8,
            "committed V-slot pose is not an orthonormal right-handed frame")) return 1;
    const cadview::Vec3d displayedAnchor = view.VSlotAnchor();
    if (!Require(std::abs(displayedAnchor.x - draggedAnchor.x) < 1.0e-6
            && std::abs(displayedAnchor.y - draggedAnchor.y) < 1.0e-6
            && std::abs(displayedAnchor.z - draggedAnchor.z) < 1.0e-6,
            "displayed V-slot anchor does not match committed callback")) return 1;
    if (!Require(view.HasDisplayedShape(), "V-slot drag disturbed the STEP AIS shape")) return 1;

    // 上层收到完整姿态后会立即刷新覆盖层。刷新不能把已经验证为绿色的
    // 工件吸附状态退回橙色，也不能丢掉磁吸自动修正后的方向。
    fixture = committedPose;
    view.SetSceneOverlays(ground, fixture, { scanRegion }, {}, { stationMarker });
    ProcessFor(100);
    if (!Require(view.IsVSlotSnappedToWorkpiece(),
            "reapplying the committed V-slot pose lost the green snap state")) return 1;

    // 模拟上层收到“工件 +7.5°”请求后更新工件显示框架。fixture 的模型坐标
    // 方向随新框架重新表达，但自身屏幕姿态和 anchor 必须保持不动；与此同时，
    // 原始 B-Rep 上远离 anchor 的点应发生可见位移，证明不是整个场景一起转。
    const cadview::GroundSurface groundBeforeWorkpieceRotation = ground;
    const cadview::VSlotFixture fixtureBeforeWorkpieceRotation = fixture;
    QPointF pivotScreenBefore;
    QPointF fixtureAxisScreenBefore;
    if (!Require(view.ProjectModelPoint(
                fixtureBeforeWorkpieceRotation.anchor, pivotScreenBefore)
            && view.ProjectModelPoint(
                Add(fixtureBeforeWorkpieceRotation.anchor,
                    Multiply(fixtureBeforeWorkpieceRotation.axisX, 100.0)),
                fixtureAxisScreenBefore),
            "could not project workpiece-rotation pivot or fixture axis")) return 1;
    QVector<std::pair<cadview::Vec3d, QPointF>> projectedWorkpiecePoints;
    for (const cadview::Vec3d& point : snapPoints)
    {
        QPointF projected;
        if (view.ProjectModelPoint(point, projected))
            projectedWorkpiecePoints.push_back({ point, projected });
    }
    if (!Require(!projectedWorkpiecePoints.isEmpty(),
            "no workpiece hull point was visible before pivot rotation")) return 1;

    bool workpieceFrameUpdateApplied = false;
    view.SetWorkpieceRotationRequestedCallback([&](double degrees)
        {
            workpieceRotationRequests.push_back(degrees);
            const cadview::GroundSurface previousGround = ground;
            const cadview::VSlotFixture previousFixture = fixture;
            const cadview::Vec3d nextGroundX = RotateInPlane(
                previousGround.axisX, previousGround.axisY, -degrees);
            const cadview::Vec3d nextGroundY = Normalize(Cross(
                previousGround.axisZ, nextGroundX));
            const auto preserveFixtureLocalDirection = [&](const cadview::Vec3d& direction)
                {
                    return Add(
                        Multiply(nextGroundX, Dot(direction, previousGround.axisX)),
                        Add(Multiply(nextGroundY, Dot(direction, previousGround.axisY)),
                            Multiply(previousGround.axisZ,
                                Dot(direction, previousGround.axisZ))));
                };
            ground.axisX = nextGroundX;
            ground.axisY = nextGroundY;
            fixture.axisX = Normalize(preserveFixtureLocalDirection(previousFixture.axisX));
            fixture.axisY = Normalize(preserveFixtureLocalDirection(previousFixture.axisY));
            fixture.axisZ = Normalize(preserveFixtureLocalDirection(previousFixture.axisZ));
            view.SetSceneOverlays(
                ground, fixture, { scanRegion }, {}, { stationMarker });
            workpieceFrameUpdateApplied = true;
        });
    workpieceRotatePlus->click();
    ProcessFor(150);
    if (!Require(workpieceFrameUpdateApplied,
            "workpiece in-view rotation request did not reach the simulated upper layer")) return 1;
    QPointF pivotScreenAfter;
    QPointF fixtureAxisScreenAfter;
    if (!Require(view.ProjectModelPoint(fixture.anchor, pivotScreenAfter)
            && view.ProjectModelPoint(
                Add(fixture.anchor, Multiply(fixture.axisX, 100.0)),
                fixtureAxisScreenAfter),
            "could not project fixture after workpiece pivot rotation")) return 1;
    const auto screenDistance = [](const QPointF& left, const QPointF& right)
        {
            return std::hypot(left.x() - right.x(), left.y() - right.y());
        };
    if (!Require(screenDistance(pivotScreenBefore, pivotScreenAfter) <= 2.0,
            "workpiece rotation moved the V-slot anchor screen pivot")) return 1;
    if (!Require(screenDistance(
                fixtureAxisScreenBefore, fixtureAxisScreenAfter) <= 2.0,
            "workpiece rotation carried the independent V-slot around the workpiece")) return 1;
    if (!Require(Length(Subtract(
                fixture.anchor, fixtureBeforeWorkpieceRotation.anchor)) <= 1.0e-8,
            "workpiece rotation mutated the fixture model anchor")) return 1;
    const double localFixtureXBefore = Dot(
        fixtureBeforeWorkpieceRotation.axisX,
        groundBeforeWorkpieceRotation.axisX);
    const double localFixtureYBefore = Dot(
        fixtureBeforeWorkpieceRotation.axisX,
        groundBeforeWorkpieceRotation.axisY);
    if (!Require(std::abs(Dot(fixture.axisX, ground.axisX) - localFixtureXBefore) < 1.0e-8
            && std::abs(Dot(fixture.axisX, ground.axisY) - localFixtureYBefore) < 1.0e-8,
            "workpiece frame update changed the independent fixture yaw")) return 1;
    double maximumWorkpieceScreenMotion = 0.0;
    for (const auto& projectedPoint : projectedWorkpiecePoints)
    {
        QPointF after;
        if (view.ProjectModelPoint(projectedPoint.first, after))
        {
            maximumWorkpieceScreenMotion = std::max(
                maximumWorkpieceScreenMotion,
                screenDistance(projectedPoint.second, after));
        }
    }
    if (!Require(maximumWorkpieceScreenMotion > 3.0,
            "workpiece B-Rep did not visibly rotate around the fixed V-slot pivot")) return 1;

    // 恢复已吸附姿态，避免本段工件旋转验证影响后续自由拖动回归。
    ground = groundBeforeWorkpieceRotation;
    fixture = fixtureBeforeWorkpieceRotation;
    view.SetSceneOverlays(ground, fixture, { scanRegion }, {}, { stationMarker });
    view.SetWorkpieceRotationRequestedCallback([&](double degrees)
        {
            workpieceRotationRequests.push_back(degrees);
        });
    ProcessFor(100);
    if (!Require(view.IsVSlotSnappedToWorkpiece(),
            "restoring the pre-rotation pose did not restore V-slot snap")) return 1;

    // 距离 +X/+Y 支撑角超过阈值时应保持用户的贴地自由位置。
    fixture.anchor = draggedAnchor;
    ground.snapDistanceMm = 5.0;
    view.SetSceneOverlays(ground, fixture, { scanRegion }, {}, { stationMarker });
    ProcessFor(100);
    const cadview::Vec3d freeClickPoint = Add(
        Add(fixture.anchor, clickOffset), Multiply(fixture.axisZ, 35.0));
    QPointF freeDragStart;
    if (!Require(view.ProjectModelPoint(freeClickPoint, freeDragStart),
            "could not project free-position V-slot start")) return 1;
    QPointF freeDragEnd;
    bool foundFreeTarget = false;
    for (int attempt = 0; attempt < 12 && !foundFreeTarget; ++attempt)
    {
        const cadview::Vec3d candidate = Add(fixture.anchor,
            Add(Multiply(fixture.axisX, 137.0 + attempt * 41.0),
                Multiply(fixture.axisY, 83.0 + attempt * 29.0)));
        const double distanceToCorner = Length(Subtract(targetSnapPoint, candidate));
        const cadview::Vec3d destination = Add(candidate, clickOffset);
        QPointF projectedDestination;
        if (distanceToCorner > 20.0
            && view.ProjectModelPoint(destination, projectedDestination))
        {
            freeDragEnd = projectedDestination;
            foundFreeTarget = true;
        }
    }
    if (!Require(foundFreeTarget,
            "could not find a visible free ground position away from snap corners")) return 1;
    dragCommitted = false;
    snapStateCommitted = true;
    SendMouseEvent(view, QEvent::MouseButtonPress, freeDragStart,
        Qt::LeftButton, Qt::LeftButton);
    SendMouseEvent(view, QEvent::MouseMove, freeDragEnd,
        Qt::NoButton, Qt::LeftButton);
    SendMouseEvent(view, QEvent::MouseButtonRelease, freeDragEnd,
        Qt::LeftButton, Qt::NoButton);
    ProcessFor(200);
    if (!Require(dragCommitted,
            "free ground drag did not commit through callback")) return 1;
    const double freeNearestSnapDistance = Length(
        Subtract(targetSnapPoint, draggedAnchor));
    if (!Require(freeNearestSnapDistance > ground.snapDistanceMm + 1.0,
            "V-slot outside snap threshold was forced onto a support corner")) return 1;
    if (!Require(!snapStateCommitted && !view.IsVSlotSnappedToWorkpiece(),
            "free ground drag incorrectly remained workpiece-snapped")) return 1;

    const int workpieceFaceCountBeforeRobot = view.LoadedFaceCount();
    const int workpieceGroundCandidatesBeforeRobot = view.GroundFaceCandidates().size();
    QElapsedTimer collisionAdoptTimer;
    collisionAdoptTimer.start();
    if (!Require(view.SetRobotCollisionEnvelope(
            syntheticEnvelope, collisionError, false),
            collisionError.toLocal8Bit().constData())) return 1;
    const qint64 collisionAdoptMs = collisionAdoptTimer.elapsed();
    ProcessFor(150);
    if (!Require(collisionAdoptMs < 1000,
            "synthetic collision envelope takeover exceeded one second")) return 1;
    if (!Require(view.IsRobotCollisionEnvelope()
            && view.HasTheoreticalRobotShape()
            && view.HasDisplayedTheoreticalRobot()
            && view.TheoreticalRobotJointComponentCount() == 7
            && view.TheoreticalRobotDisplayBlockCount() == 7
            && view.TheoreticalRobotDisplayedBlockCount() == 7,
            "synthetic J0-J6 collision envelope is not the active displayed robot"))
        return 1;
    if (!Require(view.LoadedFaceCount() == workpieceFaceCountBeforeRobot
            && view.GroundFaceCandidates().size()
                == workpieceGroundCandidatesBeforeRobot,
            "collision envelope polluted workpiece face/support analysis")) return 1;
    const cadview::Vec3d collisionBase = view.TheoreticalRobotBaseDisplayPosition();
    const double displayedGroundZ = Dot(ground.planePoint, fixture.axisZ);
    if (!Require(std::abs(collisionBase.z - displayedGroundZ) < 1.0e-3,
            "collision envelope J0 installation plane is not on verified ground")) return 1;
    const int collisionPresentationBuilds =
        view.TheoreticalRobotPresentationBuildCount();
    view.SetGroundFaceCandidateHighlight(
        (supportCandidateIndex + 1) % groundFaces.size());
    view.SetGroundFaceCandidateHighlight(supportCandidateIndex);
    view.SetTheoreticalRobotVisible(false);
    if (!Require(!view.HasDisplayedTheoreticalRobot(),
            "collision envelope did not hide on request")) return 1;
    view.SetTheoreticalRobotVisible(true);
    ProcessFor(100);
    if (!Require(view.HasDisplayedTheoreticalRobot()
            && view.TheoreticalRobotPresentationBuildCount()
                == collisionPresentationBuilds,
            "collision envelope hide/overlay recovery rebuilt its OCCT boxes")) return 1;
    const RobotCollisionEnvelopeStore::EnvelopeSet invalidEnvelope =
        SyntheticRobotCollisionEnvelope(Eigen::Vector3d::UnitZ());
    if (!Require(!view.SetRobotCollisionEnvelope(
                invalidEnvelope, collisionError, true)
            && view.IsRobotCollisionEnvelope()
            && view.HasDisplayedTheoreticalRobot(),
            "invalid source-up envelope was accepted or displaced the valid scene")) return 1;

    qint64 robotLoadMs = -1;
    qint64 robotAdoptMs = -1;
    qint64 robotProgressiveMs = -1;
    bool robotScenePassed = false;
    if (!robotStepPath.isEmpty())
    {
        if (!Require(QFileInfo::exists(robotStepPath),
                "real theoretical robot STEP fixture is missing")) return 1;
        const int workpieceFaceCount = view.LoadedFaceCount();
        const int workpieceGroundCandidateCount = view.GroundFaceCandidates().size();
        const int detailedPresentationBuildBaseline =
            view.TheoreticalRobotPresentationBuildCount();
        const QString robotSha256 = FileSha256(robotStepPath);
        if (!Require(robotSha256.size() == 64,
                "real theoretical robot STEP could not be hashed")) return 1;
        RobotCadAssemblyLoader::Result loadedRobot;
        QString robotError;
        bool robotLoaded = false;
        int heartbeatTicks = 0;
        QEventLoop robotLoadLoop;
        QTimer heartbeat;
        heartbeat.setInterval(50);
        QObject::connect(&heartbeat, &QTimer::timeout, [&]() { ++heartbeatTicks; });
        QElapsedTimer robotLoadTimer;
        QThread* robotLoader = QThread::create([&]()
            {
                robotLoaded = RobotCadAssemblyLoader::LoadFile(
                    robotStepPath, loadedRobot, robotError);
            });
        QObject::connect(robotLoader, &QThread::finished,
            &robotLoadLoop, &QEventLoop::quit);
        robotLoadTimer.start();
        heartbeat.start();
        robotLoader->start();
        robotLoadLoop.exec();
        heartbeat.stop();
        robotLoader->wait();
        delete robotLoader;
        robotLoadMs = robotLoadTimer.elapsed();
        if (!Require(robotLoadTimer.elapsed() <= 250 || heartbeatTicks > 0,
                "GUI event loop did not remain responsive during robot parsing")) return 1;
        if (!Require(robotLoaded, robotError.toLocal8Bit().constData())) return 1;
        if (!Require(loadedRobot.statistics.sourceSha256 == robotSha256
                && loadedRobot.statistics.displayTriangulationPrepared,
                "robot parse did not preserve identity or prepare the display cache")) return 1;
        QElapsedTimer robotAdoptTimer;
        robotAdoptTimer.start();
        if (!Require(view.AdoptTheoreticalRobotAssembly(
                robotStepPath, robotSha256, std::move(loadedRobot), robotError, false),
                robotError.toLocal8Bit().constData())) return 1;
        robotAdoptMs = robotAdoptTimer.elapsed();
        std::cout << "INFO robot_load_ms=" << robotLoadMs
                  << " robot_adopt_ms=" << robotAdoptMs << '\n';
        if (!Require(robotAdoptMs < 5000,
                "GUI-thread robot presentation adoption exceeded 5 seconds")) return 1;
        if (!Require(view.HasTheoreticalRobotShape()
                && view.TheoreticalRobotDisplayBlockCount() > 1
                && !view.HasDisplayedTheoreticalRobot()
                && view.IsTheoreticalRobotPresentationInProgress(),
                "robot adoption did not start a non-blocking progressive presentation"))
            return 1;

        int progressiveHeartbeatTicks = 0;
        int progressiveObservedAdvances = 0;
        int lastDisplayedBlocks = view.TheoreticalRobotDisplayedBlockCount();
        QEventLoop progressiveLoop;
        QTimer progressiveHeartbeat;
        progressiveHeartbeat.setInterval(50);
        QObject::connect(&progressiveHeartbeat, &QTimer::timeout, [&]()
        {
            ++progressiveHeartbeatTicks;
            const int displayedBlocks = view.TheoreticalRobotDisplayedBlockCount();
            if (displayedBlocks > lastDisplayedBlocks)
            {
                ++progressiveObservedAdvances;
                lastDisplayedBlocks = displayedBlocks;
            }
            if (view.IsTheoreticalRobotDisplayComplete()) progressiveLoop.quit();
        });
        QTimer progressiveTimeout;
        progressiveTimeout.setSingleShot(true);
        QObject::connect(&progressiveTimeout, &QTimer::timeout,
            &progressiveLoop, &QEventLoop::quit);
        QElapsedTimer progressiveTimer;
        progressiveTimer.start();
        progressiveHeartbeat.start();
        progressiveTimeout.start(300000);
        progressiveLoop.exec();
        progressiveHeartbeat.stop();
        robotProgressiveMs = progressiveTimer.elapsed();
        std::cout << "INFO robot_progressive_ms=" << robotProgressiveMs
                  << " heartbeat_ticks=" << progressiveHeartbeatTicks
                  << " observed_advances=" << progressiveObservedAdvances
                  << " display_blocks=" << view.TheoreticalRobotDisplayBlockCount() << '\n';
        if (!Require(view.IsTheoreticalRobotDisplayComplete()
                && view.TheoreticalRobotDisplayedBlockCount()
                    == view.TheoreticalRobotDisplayBlockCount(),
                "progressive robot presentation did not complete within five minutes"))
            return 1;
        if (!Require(progressiveHeartbeatTicks >= 2
                && progressiveObservedAdvances >= 2,
                "GUI event loop heartbeat did not continue between robot display blocks"))
            return 1;
        if (!Require(view.HasTheoreticalRobotShape()
                && view.HasDisplayedTheoreticalRobot(),
                "J0-J6 theoretical robot was not displayed beside the grounded workpiece"))
            return 1;
        if (!Require(view.TheoreticalRobotJointComponentCount() == 7,
                "theoretical robot display is not exactly J0-J6")) return 1;
        if (!Require(view.LoadedFaceCount() == workpieceFaceCount
                && view.GroundFaceCandidates().size() == workpieceGroundCandidateCount,
                "theoretical robot polluted workpiece face/support analysis")) return 1;
        const cadview::Vec3d robotBase = view.TheoreticalRobotBaseDisplayPosition();
        const double displayedGroundZ = Dot(ground.planePoint, fixture.axisZ);
        if (!Require(std::abs(robotBase.z - displayedGroundZ) < 1.0e-3,
                "theoretical robot base is not on the verified ground plane")) return 1;
        const int presentationBuilds = view.TheoreticalRobotPresentationBuildCount();
        if (!Require(presentationBuilds == detailedPresentationBuildBaseline + 1,
                "theoretical robot presentation was not built exactly once")) return 1;
        QElapsedTimer refreshTimer;
        refreshTimer.start();
        view.SetGroundFaceCandidateHighlight(
            (supportCandidateIndex + 1) % groundFaces.size());
        view.SetGroundFaceCandidateHighlight(supportCandidateIndex);
        view.SetTheoreticalRobotVisible(false);
        if (!Require(!view.HasDisplayedTheoreticalRobot(),
                "theoretical robot did not hide on request")) return 1;
        view.SetTheoreticalRobotVisible(true);
        ProcessFor(100);
        if (!Require(view.HasDisplayedTheoreticalRobot()
                && view.TheoreticalRobotPresentationBuildCount() == presentationBuilds,
                "ordinary refresh rebuilt the robot presentation instead of reusing it")) return 1;
        if (!Require(refreshTimer.elapsed() < 5000,
                "candidate/visibility refresh took more than 5 seconds")) return 1;
        robotScenePassed = true;
    }
    if (!Require(Length(Subtract(draggedAnchor, fixture.anchor)) > 20.0,
            "free ground drag did not retain a moved position")) return 1;
    if (!Require(std::abs(Dot(
                Subtract(draggedAnchor, ground.planePoint), fixture.axisZ)) < 1.0e-3,
            "free V-slot position left the ground plane")) return 1;
    const QSize nativeViewport = view.NativeViewportSizePixels();
    const QSize expectedViewport(
        qRound(view.width() * view.devicePixelRatioF()),
        qRound(view.height() * view.devicePixelRatioF()));
    if (!Require(std::abs(nativeViewport.width() - expectedViewport.width()) <= 2
            && std::abs(nativeViewport.height() - expectedViewport.height()) <= 2,
            "OCCT native client size does not match the high-DPI widget")) return 1;

    QScreen* screen = window.screen();
    if (screen == nullptr) screen = QGuiApplication::primaryScreen();
    if (!Require(screen != nullptr, "no Windows screen is available")) return 1;
    const QPixmap desktop = screen->grabWindow(0);
    const QPoint globalTopLeft = view.mapToGlobal(QPoint(0, 0));
    const qreal captureDpr = screen->devicePixelRatio();
    const QRect physicalViewRect(
        qRound(globalTopLeft.x() * captureDpr),
        qRound(globalTopLeft.y() * captureDpr),
        qRound(view.width() * captureDpr),
        qRound(view.height() * captureDpr));
    const QPixmap composedScreenshot = desktop.copy(physicalViewRect);
    if (!Require(!composedScreenshot.isNull(), "composited CAD screenshot is empty")) return 1;
    if (!Require(composedScreenshot.save(screenshotPath), "failed to save CAD screenshot")) return 1;
    if (!Require(ViewportCoversNativeWindow(composedScreenshot.toImage()),
            "OCCT viewport does not cover the full high-DPI widget area")) return 1;
    if (!Require(ScreenshotContainsCadAndLabels(composedScreenshot.toImage()),
            "screenshot does not contain shaded CAD plus red/green Qt labels")) return 1;

    const int faceCount = view.LoadedFaceCount();
    const QString normalizedSource = view.LoadedSourcePath();
    QTemporaryDir temporary;
    if (!Require(temporary.isValid(), "could not create same-path test directory")) return 1;
    const QString workingPath = temporary.filePath(QStringLiteral("同路径替换.step"));
    if (!CorruptSamePathAndRequireClear(view, sourcePath, workingPath, true, true)) return 1;
    if (!CorruptSamePathAndRequireClear(view, sourcePath, workingPath, false, false)) return 1;

    std::cout << "PASS source=" << normalizedSource.toLocal8Bit().constData()
              << " faces=" << faceCount
              << " ground_faces=" << groundFaces.size()
              << " capture=" << composedScreenshot.width() << 'x' << composedScreenshot.height()
              << " widget=" << view.width() << 'x' << view.height()
              << " native=" << nativeViewport.width() << 'x' << nativeViewport.height()
              << " dpr=" << view.devicePixelRatioF()
              << " robot=" << (robotScenePassed ? "detailed-J0-J6" : "collision-J0-J6")
              << " collision_adopt_ms=" << collisionAdoptMs
              << " robot_load_ms=" << robotLoadMs
              << " robot_adopt_ms=" << robotAdoptMs
              << " robot_progressive_ms=" << robotProgressiveMs
              << " screenshot=" << screenshotPath.toLocal8Bit().constData() << '\n';
    return 0;
}
