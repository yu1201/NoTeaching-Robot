#pragma once

#include "RobotCadAssemblyLoader.h"
#include "RobotCollisionEnvelopeStore.h"

#include <QColor>
#include <QSize>
#include <QString>
#include <QVector>
#include <QWidget>

#include <functional>
#include <memory>

class QMouseEvent;
class QEvent;
class QPaintEngine;
class QPaintEvent;
class QPointF;
class QResizeEvent;
class QShowEvent;
class QWheelEvent;

namespace cadview
{
struct Vec3d
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

// 始终位于 CAD 模型之上的三维引导线。arrowHead=true 时在 end 端绘制箭头。
struct GuideSegment
{
    Vec3d start;
    Vec3d end;
    QString label;
    QColor color = QColor(255, 215, 64);
    double width = 2.5;
    bool arrowHead = false;
};

// 屏幕尺寸不随缩放变化的三维站点标记和标签。
struct StationMarker
{
    Vec3d position;
    QString label;
    QColor color = QColor(255, 215, 64);
    double size = 7.0;
};

// 模型粗放置用的可拖动实体直角槽。axisX/axisY/axisZ 必须构成右手正交坐标系；
// 它们描述 V 槽自身姿态，与工件显示/落地坐标系相互独立。完成工件落地大面
// 验证后，anchor 与地面内方向可共同磁吸到该支撑面的工件外角。
struct VSlotFixture
{
    bool visible = false;
    // 相似模板已审核继承工装位置时可只显示而禁止拖动，避免无意
    // 修改后破坏继承证明链。
    bool draggable = true;
    Vec3d anchor;
    Vec3d axisX = { 1.0, 0.0, 0.0 };
    Vec3d axisY = { 0.0, 1.0, 0.0 };
    Vec3d axisZ = { 0.0, 0.0, 1.0 };
    double longLengthMm = 300.0;
    double shortLengthMm = 180.0;
    double wallThicknessMm = 14.0;
    double wallHeightMm = 36.0;
    QColor color = QColor(255, 167, 38);
};

// 已通过工件外包络大面验证的现场地面。它仅是场景覆盖，不参与 STEP
// 候选分析，也不写入模型模板；supportCandidateIndex 用于 V 槽拖动时
// 求取该支撑面外轮廓的方向与支撑角，实现位置和方向联合吸附。
struct GroundSurface
{
    bool visible = false;
    // 即使地面尚未显示，这三个轴也定义工件在三维场景中的落地显示框架。
    // workpieceRotatable 只控制视图内旋转按钮，不改变落地面验证结果。
    bool workpieceRotatable = false;
    Vec3d planePoint;
    Vec3d axisX = { 1.0, 0.0, 0.0 };
    Vec3d axisY = { 0.0, 1.0, 0.0 };
    Vec3d axisZ = { 0.0, 0.0, 1.0 };
    int supportCandidateIndex = -1;
    double thicknessMm = 12.0;
    double snapDistanceMm = 60.0;
    QColor color = QColor(205, 210, 214);
};

// 覆盖在 STEP 表面的人工点云采集建议区域。它只用于显示，不修改 STEP/PLY。
struct ScanRegion
{
    Vec3d center;
    Vec3d outwardNormal = { 0.0, 0.0, 1.0 };
    QString label;
    QColor color = QColor(239, 83, 80);
    double radiusMm = 25.0;
    double thicknessMm = 2.0;
    bool selected = false;
};

// 原始 STEP 中位于模型外包络、可作为工件落地面的平面候选。
struct GroundFaceCandidate
{
    Vec3d centerModel;
    Vec3d groundUpModel = { 0.0, 0.0, 1.0 };
    // 同一支撑平面上的总接触面积，以及其中最大的单个连续 B-Rep 面。
    // largestFaceAreaMm2 防止很多离散小脚被简单累加后冒充一个“大面”。
    double areaMm2 = 0.0;
    double largestFaceAreaMm2 = 0.0;
    double supportToleranceMm = 0.1;
};

// 直接显示原始 STEP/B-Rep 的 OCCT 原生视图。计算用 PLY 不进入此控件。
class CadModel3DView final : public QWidget
{
public:
    explicit CadModel3DView(QWidget* parent = nullptr);
    ~CadModel3DView() override;

    // 读取并显示原始 STEP；OCCT 传输目标固定为毫米，保留文件坐标轴和原点。
    // 同一路径且 preserveView=true 时不会重复读取或复位相机。
    bool SetStepFile(
        const QString& sourceStepPath,
        QString& error,
        bool preserveView = false);
    // expectedSourceSha256 非空时，读取前必须与追溯元数据中的源 SHA-256 完全一致。
    bool SetStepFile(
        const QString& sourceStepPath,
        const QString& expectedSourceSha256,
        QString& error,
        bool preserveView = false);
    // 后台完成 XCAF 解析后，在 GUI 线程接管已加载的 J0-J6 B-Rep；接管前会
    // 再次核验受控资产 SHA-256，避免解析期间文件被替换。
    bool AdoptTheoreticalRobotAssembly(
        const QString& sourceStepPath,
        const QString& expectedSourceSha256,
        RobotCadAssemblyLoader::Result&& loaded,
        QString& error,
        bool preserveView = false);
    // 接管已经过 RobotCollisionEnvelopeStore 校验/持久化的 J0...J6 静态
    // 碰撞包络。每个 collisionBoundsMm 直接构造成独立 OCCT box B-Rep，
    // 不并入工件 shape，也不参与工件曲面和落地候选分析。
    bool SetRobotCollisionEnvelope(
        const RobotCollisionEnvelopeStore::EnvelopeSet& envelope,
        QString& error,
        bool preserveView = false);
    bool IsRobotCollisionEnvelope() const;
    void ClearTheoreticalRobot();
    // 机器人只在已验证地面可见时显示，并自动以静态理论姿态放在工件 -X 侧。
    void SetTheoreticalRobotVisible(bool visible);
    void ClearModel(const QString& message = QString());
    void SetAnnotations(
        const QVector<GuideSegment>& guides,
        const QVector<StationMarker>& markers);
    void SetSceneOverlays(
        const VSlotFixture& fixture,
        const QVector<ScanRegion>& scanRegions,
        const QVector<GuideSegment>& guides,
        const QVector<StationMarker>& markers);
    void SetSceneOverlays(
        const GroundSurface& ground,
        const VSlotFixture& fixture,
        const QVector<ScanRegion>& scanRegions,
        const QVector<GuideSegment>& guides,
        const QVector<StationMarker>& markers);
    void ClearSceneOverlays();
    void SetVSlotMovedCallback(std::function<void(const Vec3d&)> callback);
    // V 槽平移或方向磁吸提交时，以模型坐标一次性回传完整姿态。
    void SetVSlotPoseChangedCallback(
        std::function<void(const VSlotFixture&, bool)> callback);
    // 鼠标释放 V 槽时回传“是否真正磁吸到工件外角”。位置回调仍
    // 只负责 anchor，便于旧调用方保持单一职责。
    void SetVSlotSnapStateCallback(std::function<void(bool)> callback);
    // 三维视图内悬浮按钮只发出旋转请求；模板更新、失效门禁由调用方处理。
    void SetWorkpieceRotationRequestedCallback(
        std::function<void(double)> callback);
    void SetVSlotRotationRequestedCallback(
        std::function<void(double)> callback);
    void SetRotationStepDegrees(double degrees);
    void SetGroundFaceCandidateHighlight(int candidateIndex);
    void FitAll();

    QString LoadedSourcePath() const;
    QString LastError() const;
    bool HasCadShape() const;
    bool HasDisplayedShape() const;
    int LoadedFaceCount() const;
    int AnnotationLabelCount() const;
    bool HasDisplayedGround() const;
    bool HasDisplayedVSlot() const;
    bool HasTheoreticalRobotShape() const;
    bool HasDisplayedTheoreticalRobot() const;
    QString LoadedTheoreticalRobotPath() const;
    int TheoreticalRobotJointComponentCount() const;
    int TheoreticalRobotPresentationBuildCount() const;
    int TheoreticalRobotDisplayBlockCount() const;
    int TheoreticalRobotDisplayedBlockCount() const;
    bool IsTheoreticalRobotDisplayComplete() const;
    bool IsTheoreticalRobotPresentationInProgress() const;
    Vec3d TheoreticalRobotBaseDisplayPosition() const;
    int DisplayedScanRegionCount() const;
    Vec3d VSlotAnchor() const;
    bool IsVSlotSnappedToWorkpiece() const;
    QVector<GroundFaceCandidate> GroundFaceCandidates() const;
    QVector<Vec3d> GroundFaceSnapPoints(int candidateIndex) const;
#ifdef CAD_MODEL_3D_VIEW_TESTING
    // 仅供 CAD 冒烟测试用：用已知外角、内部点和孔轮廓点验证生产吸附点凸包。
    static QVector<Vec3d> BuildGroundSnapHullForTesting(
        const QVector<Vec3d>& points,
        const Vec3d& planePoint,
        const Vec3d& planeNormal);
#endif
    bool ProjectModelPoint(const Vec3d& point, QPointF& widgetPoint) const;
    QSize NativeViewportSizePixels() const;

    QPaintEngine* paintEngine() const override;

protected:
    bool event(QEvent* event) override;
    void showEvent(QShowEvent* event) override;
    void paintEvent(QPaintEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void mouseDoubleClickEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;

private:
    class Impl;
    std::unique_ptr<Impl> m_impl;
};
}
