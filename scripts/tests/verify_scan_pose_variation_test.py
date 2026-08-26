"""Static contract checks for the embedded scan pose-variation workflow.

This test does not connect to a robot.  It protects the UI entry, taught-pose
persistence, weld-compatible posture math, fail-closed motion admission and
reuse of the existing scan acquisition/output chain.
"""

from __future__ import annotations

from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[2]


def read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def section(text: str, start: str, end: str) -> str:
    begin = text.find(start)
    if begin < 0:
        raise AssertionError(f"missing section start: {start}")
    finish = text.find(end, begin + len(start))
    if finish < 0:
        raise AssertionError(f"missing section end: {end}")
    return text[begin:finish]


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def main() -> int:
    project = read("QtWidgetsApplication4.vcxproj")
    function_test = read("src/FunctionTestDialog.cpp")
    app = read("src/QtWidgetsApplication4.cpp")
    dialog = read("src/ScanPoseVariationTestDialog.cpp")
    dialog_header = read("include/ScanPoseVariationTestDialog.h")
    service = read("src/MeasureThenWeldService.cpp")
    service_header = read("include/MeasureThenWeldService.h")

    require("扫描变姿态精度测试" not in function_test,
            "scan pose-variation entry still lives inside the ordinary function-test page")
    menu_entry = 'createManagementAction("扫描变姿态精度测试", [this]() { OpenScanPoseVariationTestPage(); })'
    require(menu_entry in app,
            "management Debug menu is missing the scan pose-variation entry")
    page_open = section(
        app,
        "void QtWidgetsApplication4::OpenScanPoseVariationTestPage()",
        "void QtWidgetsApplication4::OpenConfigDatabaseViewerDialog()",
    )
    for token in (
        "m_pManagementStack",
        "new ScanPoseVariationTestDialog(",
        "PrepareEmbeddedPage(m_pScanPoseVariationTestPage, m_pManagementStack)",
        "ShowManagementEmbeddedPage(m_pScanPoseVariationTestPage)",
    ):
        require(token in page_open, f"management embedded-page wiring missing: {token}")
    require("class ScanPoseVariationTestDialog : public QWidget" in dialog_header,
            "scan pose-variation UI remains a standalone dialog instead of an embedded page")
    require("ScanPoseVariationTestDialog.cpp" in project
            and "ScanPoseVariationTestDialog.h" in project,
            "new dialog is not part of the Visual Studio build")

    for token in (
        "使用机器人",
        "扫描相机",
        "扫描速度",
        "后处理方式",
        "无=点云生成后结束",
        "直线处理=采集特征点并生成三维平滑曲线",
        "波纹板处理=进入现有特征点、拐点拟合及焊接姿态生成流程",
        'setObjectName(QStringLiteral("scanInputSourceCard"))',
        "参数来源（自动继承）",
        "机器人运动与时序",
        "相机与空间坐标",
        "contentLayout->setSizeConstraint(QLayout::SetMinAndMaxSize)",
        "扫描实时相机图像",
        "示教基础姿态并保存",
        "示教扫描起点并保存",
        "示教扫描终点并保存",
        "生成并保存扫描轨迹",
        "运行扫描并保存数据",
        'ConfigLocation::Robot(RobotName(), QStringLiteral("FunctionTestScanPoseVariation"))',
        "RobotDataHelper::WriteCoors",
        "RobotDataHelper::WritePulse",
        'QStringLiteral("RobotName")',
        'QStringLiteral("CameraSection")',
        'QStringLiteral("ScanSpeedMmPerMin")',
        'QStringLiteral("PostProcessMode")',
    ):
        require(token in dialog, f"teach/config workflow missing: {token}")
    require('targetLayout->addRow(QStringLiteral("继承参数"), inheritedHint)' not in dialog,
            "long inherited-parameter copy remains constrained inside the narrow form field")

    generator = section(
        service,
        "bool MeasureThenWeldService::GenerateScanPoseVariationTrajectory(",
        "bool MeasureThenWeldService::SaveScanPoseVariationTrajectory(",
    )
    for token in (
        "RobotPoseTransform::RotZDeg(currentAngleDeg) * taughtRotation",
        "RobotPoseTransform::AnglesFromRotationDegNear(",
        "beginOrientation.slerp(",
        "taughtStartPose.dX",
        "taughtEndPose.dX",
        "baseStartRotationDeltaDeg > 5.0",
        "baseEndRotationDeltaDeg > 5.0",
        "externalAxisDelta > 1e-6",
        "SCAN_POSE_VARIATION_MAX_POINTS",
        "禁止在同一空间点产生突变姿态",
    ):
        require(token in generator, f"trajectory contract missing: {token}")
    require("ScanPoseVariationParams" in service_header
            and "ScanPoseVariationPoint" in service_header,
            "trajectory types are not exposed by the scan service")

    run_scan = section(
        dialog,
        "void ScanPoseVariationTestDialog::RunScan()",
        "void ScanPoseVariationTestDialog::AppendLog(",
    )
    for token in (
        "m_startCamera(m_unitIndex, cameraSection, cameraIp)",
        "param.dScanSpeed = scanSpeedMmPerMin",
        "selectedCameraSection",
        "RobotOperationLease::TryAcquire(",
        "QMessageBox::question(",
        "param.tStartPos = trajectory.front()",
        "param.tEndPos = trajectory.back()",
        "param.tStartPulse = startPulse",
        "service.RunScanCycle(",
        "&trajectory",
        "ScanPoseVariation_CommandedTrajectory.csv",
        "ScanPoseVariation_TestSummary.txt",
        "scan_speed_mm_per_min=",
        "camera_section=",
        "safe_move_speed_mm_per_min=",
        "camera_time_offset_ms=",
        "post_process_mode=",
        "post_process_name=",
        "CurrentPostProcessMode()",
        "postProcessMode",
    ):
        require(token in run_scan, f"execution/output workflow missing: {token}")
    for token in ("SetLiveImageEnabled(true)", "LatestImage(&imageTimestamp)"):
        require(token in dialog, f"live image workflow missing: {token}")

    run_cycle = section(
        service,
        "bool MeasureThenWeldService::RunScanCycle(",
        "bool MeasureThenWeldService::ScanMoveAndCollect(",
    )
    custom_validation = run_cycle.find("if (scanTrajectory != nullptr)")
    first_motion = run_cycle.find('allowAction("扫描下枪安全位置")')
    require(0 <= custom_validation < first_motion,
            "custom trajectory is not fully validated before the first safe-position motion")
    for token in (
        "RobotDriverCapability::ContinuousTrajectory",
        "scanTrajectory->size() < 2",
        "IsFinitePose((*scanTrajectory)[index])",
        "endpointMatches(scanTrajectory->front(), param.tStartPos)",
        "endpointMatches(scanTrajectory->back(), param.tEndPos)",
        "MoveScanStartSafeAndWait(",
        "MoveScanEndSafeAndWait(",
        "cameraSectionOverride.trimmed().isEmpty()",
        "postProcessMode",
    ):
        require(token in run_cycle, f"pre-motion/safe-retraction contract missing: {token}")

    collector = service[service.find("bool MeasureThenWeldService::ScanMoveAndCollect("):]
    for token in (
        "pRobotDriver->StartTrajectory(",
        "RobotTrajectoryPurpose::ScanDryRun",
        "RobotTrajectoryHandle scanTrajectoryHandle",
        "pRobotDriver->ReadMotionStatus()",
        "ScanPoseVariation_MAX_POINTS",  # checked case-insensitively below
        "appendRobotPose()",
        "PreciseLaserPoint",
    ):
        if token == "ScanPoseVariation_MAX_POINTS":
            require("SCAN_POSE_VARIATION_MAX_POINTS" in collector,
                    "collector lacks the controller point-count gate")
        else:
            require(token in collector, f"existing scan/driver integration missing: {token}")
    require("if (scanTrajectory != nullptr)" in collector,
            "custom trajectory can silently fall back to an ordinary endpoint MOVL")
    require("const std::vector<T_ROBOT_COORS>* scanTrajectory = nullptr" in service_header,
            "ordinary scan callers did not retain backward-compatible default behavior")
    require("enum class ScanPostProcessMode" in service_header
            and "ScanPostProcessMode::CorrugatedBoard" in service_header,
            "scan service does not expose a backward-compatible per-run post-process contract")
    for token in (
        "runCorrugatedBoardPostProcess",
        "ScanPostProcessMode::FeaturePointSmoothCurve",
        "BuildSmoothFeatureCurve",
    ):
        require(token in collector, f"post-process branch missing from scan collector: {token}")
    require("PreciseLaserPoint_FeatureSmoothCurve_2mm.txt" in service,
            "feature-point smooth-curve output has no dedicated result artifact")
    raw_only_branch = collector.find("if (!runCorrugatedBoardPostProcess)")
    feature_analysis = collector.find("AnalyzeMeasureThenWeldPointCloud(")
    require(0 <= raw_only_branch < feature_analysis,
            "non-corrugated scan modes do not finish before corrugated feature/corner processing")
    require("RobotOperationLease::IsCancellationRequested(driver)" in run_scan,
            "background test cannot participate in the shared safety-stop contract")

    print("PASS: scan pose-variation teach, math, motion gates, acquisition and output contracts are wired")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except AssertionError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        raise SystemExit(1)
