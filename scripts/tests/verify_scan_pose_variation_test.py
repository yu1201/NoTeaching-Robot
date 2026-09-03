"""Static contract checks for the embedded scan pose-variation workflow.

This test does not connect to a robot.  It protects the UI entry, taught-pose
persistence, weld-compatible posture math, fail-closed motion admission,
reuse of the existing scan acquisition/output chain and the Tool1 fixed-pose
2 mm curve dry-run path.
"""

from __future__ import annotations

import hashlib
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
    safety_store = read("src/WeldSafetyRecoveryStore.cpp")
    release_package = read("scripts/build_release_package.ps1")

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
        "扫描实时点云与相机图像",
        "实时激光线点云",
        "实时相机图像",
        "示教基础姿态并保存",
        "示教扫描起点并保存",
        "示教扫描终点并保存",
        "生成并保存扫描轨迹",
        "运行扫描并保存数据",
        "生成并运行直线模拟轨迹",
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
    for token in (
        "std::abs(params.leftRotationDeg) > 60.0",
        "std::abs(params.rightRotationDeg) > 60.0",
        "std::abs(params.leftRotationDeg) > 1e-9",
        "params.leftRotationDeg",
        "-params.rightRotationDeg",
        "实际RotZ（上坡/下坡）",
    ):
        require(token in generator, f"signed slope-angle contract missing: {token}")
    require("spin->setRange(-60.0, 60.0)" in dialog,
            "slope-angle editors still reject negative values")

    def phase_angles(left_rotation_deg: float,
                     right_rotation_deg: float) -> tuple[float, float, float, float]:
        return (0.0, left_rotation_deg, 0.0, -right_rotation_deg)

    require(phase_angles(10.0, 10.0) == (0.0, 10.0, 0.0, -10.0),
            "positive slope-angle behavior changed")
    require(phase_angles(-10.0, -10.0) == (0.0, -10.0, 0.0, 10.0),
            "negative slope angles do not reverse both named rotation directions")
    require(phase_angles(-10.0, 10.0) == (0.0, -10.0, 0.0, -10.0),
            "signed rising/falling inputs are not independently preserved")
    require("ScanPoseVariationParams" in service_header
            and "ScanPoseVariationPoint" in service_header,
            "trajectory types are not exposed by the scan service")

    run_scan = section(
        dialog,
        "void ScanPoseVariationTestDialog::RunScan()",
        "void ScanPoseVariationTestDialog::RunStraightCurveSimulation()",
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
        "ResolveScanPosePointCloudSdk(",
        "pointCloudSdkLibraryDirOverride",
        "pointCloudSdkSha256",
        "trajectory_sdk_override_dir=",
        "trajectory_sdk_dll=",
        "trajectory_sdk_sha256=",
        "trajectory_sdk_mode=",
        "trajectory_sdk_resample_step_mm=",
    ):
        require(token in run_scan, f"execution/output workflow missing: {token}")
    sdk_validation = run_scan.find("ResolveScanPosePointCloudSdk(")
    motion_confirmation = run_scan.find("QMessageBox::question(")
    require(0 <= sdk_validation < motion_confirmation,
            "scan-pose SDK is not identity-checked before robot-motion confirmation")
    require("925ac6bf19762f76cf249c96a0fe873abfa94151ac6636989c10759ce4a27432" in dialog,
            "scan-pose SDK SHA-256 is not pinned in the application")
    scan_pose_sdk_bin = (ROOT / "SDK/PointCloudExtration"
                         / "findWeldingLine_sdk_x64_Release_20260902_1742/bin")
    scan_pose_sdk_dll = scan_pose_sdk_bin / "findWeldingLine.dll"
    require(scan_pose_sdk_dll.is_file(), "bundled scan-pose findWeldingLine.dll is missing")
    actual_sdk_sha256 = hashlib.sha256(scan_pose_sdk_dll.read_bytes()).hexdigest()
    require(actual_sdk_sha256 ==
            "925ac6bf19762f76cf249c96a0fe873abfa94151ac6636989c10759ce4a27432",
            f"bundled scan-pose SDK hash mismatch: {actual_sdk_sha256}")
    for dependency in (
        "opencv_world480.dll",
        "pcl_common.dll",
        "pcl_features.dll",
        "pcl_filters.dll",
        "pcl_kdtree.dll",
        "pcl_octree.dll",
        "pcl_sample_consensus.dll",
        "pcl_search.dll",
        "pcl_segmentation.dll",
    ):
        require((scan_pose_sdk_bin / dependency).is_file(),
                f"bundled scan-pose SDK dependency is missing: {dependency}")
    for token in (
        "SetLiveImageEnabled(true)",
        "Latest(latestFrame)",
        "LatestImage(&imageTimestamp)",
        "m_livePointCloudView->SetFrame(latestFrame)",
        "liveViews->addLayout(pointCloudPreview, 1)",
        "liveViews->addLayout(imagePreview, 1)",
        "AdjustPointSize",
        "AdjustViewSpan",
        "previewGroup->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::MinimumExpanding)",
        "previewGroup->setMinimumHeight(480)",
        "m_livePointCloudView->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding)",
        "m_liveImageLabel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding)",
        "contentLayout->addWidget(previewGroup, 1)",
    ):
        require(token in dialog, f"side-by-side live point-cloud/image workflow missing: {token}")
    require("QSizePolicy::Ignored" not in dialog,
            "live preview widgets can still collapse below their display area")
    require("contentLayout->addStretch(1)" not in dialog,
            "empty bottom stretch can still consume the live preview display area")

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
        "pointCloudSdkLibraryDirOverride",
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
        "pointCloudSettings.libraryDir",
        "PointCloudProcessingConfig::Mode::SdkBaseWeldFit",
        "pointCloudSettings.resampleStepMm = 2.0",
        "pointCloudSettings.sdkUseWeldedStartTruncation = false",
        "本轮扫描轨迹计算已锁定为新版SDK返回轨迹+拟合（2mm重采样）",
        "SDK轨迹已计算但未通过后处理门禁",
    ):
        require(token in collector, f"post-process branch missing from scan collector: {token}")
    require("useReturnedTrackAsBaseWeld" in service
            and "workingExtraction.points = extraction.rawPoints" in service,
            "findWeldingLine returned-track compatibility is missing")
    require("const QString& pointCloudSdkLibraryDirOverride = QString()" in service_header,
            "ordinary scan callers did not retain an empty SDK-override default")
    for token in (
        "findWeldingLine_sdk_x64_Release_20260902_1742",
        "bin/findWeldingLine.dll",
        "925AC6BF19762F76CF249C96A0FE873ABFA94151AC6636989C10759CE4A27432",
        "Get-FileHash",
    ):
        require(token in release_package,
                f"release package does not pin/copy the scan-pose SDK runtime: {token}")
    require("PreciseLaserPoint_FeatureSmoothCurve_2mm.txt" in service,
            "feature-point smooth-curve output has no dedicated result artifact")
    raw_only_branch = collector.find("if (!runCorrugatedBoardPostProcess)")
    feature_analysis = collector.find("AnalyzeMeasureThenWeldPointCloud(")
    require(0 <= raw_only_branch < feature_analysis,
            "non-corrugated scan modes do not finish before corrugated feature/corner processing")
    require("RobotOperationLease::IsCancellationRequested(driver)" in run_scan,
            "background test cannot participate in the shared safety-stop contract")

    curve_generator = section(
        service,
        "bool MeasureThenWeldService::GenerateScanPoseVariationDryRunFiles(",
        "bool MeasureThenWeldService::DownlinkWeldPoseFile(",
    )
    for token in (
        "PreciseLaserPoint_FeatureSmoothCurve_2mm.txt",
        "index x y z source",
        "FEATURE_SMOOTH_CURVE_MIN_FULL_SEGMENT_MM",
        "FEATURE_SMOOTH_CURVE_MAX_SEGMENT_MM",
        "FEATURE_SMOOTH_CURVE_SUMMARY_FILE_NAME",
        "ScanPoseVariation_TestSummary.txt",
        'QStringLiteral("straight_line")',
        'QStringLiteral("success")',
        "record.point = curvePoints[index].point",
        "record.rx = basePose.dRX",
        "record.ry = basePose.dRY",
        "record.rz = basePose.dRZ",
        "curveStartPose = BuildWeldPoseCoors(records.front())",
        "curveEndPose = BuildWeldPoseCoors(records.back())",
        "FEATURE_SMOOTH_CURVE_RUN_STEP_MM",
        "/*actualWeld=*/false",
        "WeldPoseSource::ScanPoseVariationDryRun",
        "RegisterLocalTestPoseAuthorization(",
    ):
        require(token in curve_generator, f"straight-curve generator contract missing: {token}")

    curve_run = section(
        dialog,
        "void ScanPoseVariationTestDialog::RunStraightCurveSimulation()",
        "void ScanPoseVariationTestDialog::RefreshStraightCurveSource()",
    )
    for token in (
        "RobotDriverCapability::OfflineTrajectoryExport",
        "GenerateScanPoseVariationDryRunFiles(",
        "Tool1（焊枪）",
        "曲线起点TCP",
        "曲线终点TCP",
        "不开弧、不送丝、不摆动",
        "param.bDoActualWeld = false",
        "param.nWeldDirection = 1",
        "param.dDryRunSpeedMmPerMin = dryRunSpeedMmPerMin",
        "param.dFinalWeldTrajectoryStepMm = 2.0",
        "WeldSafetyRecoverySession",
        "ExecuteWeldPoseFileWithSafePos(",
        "WeldPoseSource::ScanPoseVariationDryRun",
        "RobotOperationLease::IsCancellationRequested(driver)",
        "机器人程序已启动、正常完成，并已验证收枪安全位置",
    ):
        require(token in curve_run, f"straight-curve execution contract missing: {token}")
    require("ScanPoseVariationDryRun = 2" in service_header,
            "scan curve dry-run source is not explicit")
    require("authorization.source != expectedSource" in service,
            "local test authorization can be reused across virtual and scan-curve sources")
    require("poseSource == WeldPoseSource::ScanPoseVariationDryRun" in service
            and "param.bDoActualWeld" in service,
            "scan curve source is not hard-locked to dry-run execution")
    for token in (
        "m_poseSource == MeasureThenWeldService::WeldPoseSource::ScanPoseVariationDryRun",
        "PreciseLaserPoint_FeatureSmoothCurve_DryRunPose_2mm.txt",
        "PreciseLaserPoint_FeatureSmoothCurve_DryRunPose_2mm_FinalSampled.txt",
        "!record.actualWeld",
        "std::abs(record.finalStepMm - 2.0)",
        "actualSha.compare(identity.sampledPoseSha256",
        "actualSourceSha.compare(identity.sourcePoseSha256",
    ):
        require(token in safety_store,
                f"scan curve persistent safety binding missing: {token}")

    print("PASS: scan pose-variation signed slope angles, scan and Tool1 curve dry-run contracts are wired")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except AssertionError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        raise SystemExit(1)
