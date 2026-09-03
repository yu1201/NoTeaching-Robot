#!/usr/bin/env python3
"""Static safety contract for cancelable weld-file rebuild and preview lifetime."""

from __future__ import annotations

from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def require(value: bool, message: str) -> None:
    if not value:
        raise AssertionError(message)


def text(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8", errors="strict")


def main() -> int:
    service_h = text("include/MeasureThenWeldService.h")
    service = text("src/MeasureThenWeldService.cpp")
    dialog_h = text("include/WeldSeamCompDialog.h")
    dialog = text("src/WeldSeamCompDialog.cpp")
    pointcloud = text("src/PointCloudExtractionProcessor.cpp")
    helper = text("src/RobotDataHelper.cpp")
    calculation = text("src/RobotCalculation.cpp")

    rebuild = service[service.index("bool MeasureThenWeldService::RebuildWeldFilesFromLaserDir(") :]
    rebuild = rebuild[: rebuild.index("QString MeasureThenWeldService::BuildResultDir(")]
    for fragment in (
        "const StopRequestedCallback& stopRequested",
        "cancellationPoint",
        "InvalidatePointCloudQualityGate(laserDir, invalidateError)",
        "preservePath.clear()",
        "weldPosePath.clear()",
        "seamCompPath.clear()",
        "未发布可执行质量证明",
        "RobotDataHelper::LoadIndexedPoint3DFile(\n                sourceLaserPath",
        "&externalExtraction,\n            stopRequested",
        "SaveTextLines(weldPosePath, weldPoseLines, error, stopRequested)",
        'cancellationPoint(QStringLiteral("质量证明提交后"))',
    ):
        require(fragment in rebuild, f"rebuild cancellation contract missing: {fragment}")

    save = service[service.index("bool MeasureThenWeldService::SaveTextLines(") :]
    save = save[: save.index("bool MeasureThenWeldService::ApplyWeldSeamCompToPoseFile(")]
    require("QSaveFile file(filePath)" in save, "rebuild output is not atomically staged")
    require("file.cancelWriting()" in save, "canceled write can commit a partial file")
    require(save.index("stopRequested && stopRequested()") < save.index("file.commit()"),
            "cancel is not checked before atomic commit")

    for forbidden in ("m_workerCount", ").detach()", "sleep_for(std::chrono::milliseconds(5))"):
        require(forbidden not in dialog + dialog_h, f"detached/busy-wait lifetime remains: {forbidden}")
    for fragment in (
        "std::thread m_previewWorker",
        "StopAndJoinPreviewWorker()",
        "m_previewCancel->store(true",
        "m_previewWorker.join()",
        "isStopRequested",
        "rebuildExpectation,\n                    isStopRequested",
        "result.baselineError,\n                    isStopRequested",
    ):
        require(fragment in dialog + dialog_h, f"owned preview-worker contract missing: {fragment}")

    for fragment in (
        "const qint64 workerTimeoutMs = SdkWorkerTimeoutMs(result.finiteInputPointCount)",
        "workerTimer.elapsed() < workerTimeoutMs",
        "proc.waitForFinished(50)",
        "proc.kill()",
        "已取消 SDK 点云提取（子进程已终止）",
        "WriteWorkerCloudFile(cloudFile, inputPoints, stopRequested)",
    ):
        require(fragment in pointcloud, f"SDK worker cancellation missing: {fragment}")
    for fragment in (
        "SDK_WORKER_BASE_TIMEOUT_MS = 300000",
        "SDK_WORKER_BASE_POINT_COUNT = 2000000",
        "SDK_WORKER_EXTRA_BLOCK_TIMEOUT_MS = 120000",
        "SDK_WORKER_MAX_TIMEOUT_MS = 900000",
        "等待上限=%1秒，有效输入点=%2",
    ):
        require(fragment in pointcloud, f"SDK worker adaptive timeout missing: {fragment}")
    require("lineCount++ & 0x3ffU" in helper and "stopRequested && stopRequested()" in helper,
            "large point-file parsing is not cancelable")

    geometry_branch_start = calculation.index("QVector<int> keyIndexes;\n    if (skipDenoise)")
    geometry_branch = calculation[
        geometry_branch_start :
        calculation.index("// \u8d77\u7ec8\u70b9\u5148\u9a8c\u81ea\u9002\u5e94\u7ec6\u5316", geometry_branch_start)
    ]
    require("\n    else\n    {" in geometry_branch,
            "skipDenoise geometry branch lost its matching else")
    require(
        geometry_branch.index("\n    else\n    {")
        < geometry_branch.rindex("\n    if (canceled())\n    {"),
        "cancel check captured the skipDenoise else branch",
    )
    require("const GeometryStopScope stopScope(params.stopRequested);" in calculation,
            "geometry stop callback is not installed in the calculation thread")
    hot_functions = (
        ("QVector<RobotCalculation::IndexedPoint3D> RemoveGeometryLocalOutliers(", "struct RawFilterPoint"),
        ("QVector<GeometryProjectedPoint> ProjectGeometryPoints(", "double GeometryDistanceToSegment2D("),
        ("void DouglasPeuckerGeometry(", "QVector<int> BuildGeometryKeyIndexes("),
        ("QVector<int> BuildAzimuthCornerKeyIndexes(", "QVector<int> SplitNonStraightAzimuthSegments("),
        ("QVector<int> SplitNonStraightAzimuthSegments(", "QVector<int> RefineCornersByCoherentBow("),
        ("QVector<int> RefineCornersByCoherentBow(", "QPair<int, int> FitPlatformTwoCorners("),
        ("QVector<int> RefineGeometryKeysBySegmentDeviation(", "QVector<int> PruneNonMonotonicFittedGeometryKeys("),
        ("QVector<int> PruneGeometrySpikeDrivenKeys(", "QVector<GeometryProjectedPoint> RemoveGeometryProjectedBranchOutliers("),
        ("QVector<GeometryProjectedPoint> RemoveGeometryProjectedBranchOutliers(", "double GeometryClampUnit("),
        ("QVector<int> PruneShortSameTypeGeometryRuns(", "QVector<RobotCalculation::IndexedPoint3D> RemoveGeometryLocalOutliers("),
    )
    for start, end in hot_functions:
        body = calculation[calculation.index(start) : calculation.index(end, calculation.index(start))]
        require("GeometryStopRequested()" in body, f"hot geometry stage lacks an internal stop check: {start}")

    require("const StopRequestedCallback& stopRequested" in service_h,
            "public rebuild API does not carry a stop callback")
    print("PASS: weld rebuild cancellation reaches I/O/SDK stages and preview worker has joinable lifetime")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
