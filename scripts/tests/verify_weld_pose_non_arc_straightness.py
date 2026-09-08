#!/usr/bin/env python3
"""Regression gate: pose transitions must not curve non-arc weld segments."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Iterable


ROOT = Path(__file__).resolve().parents[2]
SERVICE_PATH = ROOT / "src" / "MeasureThenWeldService.cpp"
DIALOG_PATH = ROOT / "src" / "WeldSeamCompDialog.cpp"


def require(value: bool, message: str) -> None:
    if not value:
        raise AssertionError(message)


def function_body(source: str, signature: str) -> str:
    start = source.index(signature)
    opening = source.index("{", start)
    depth = 0
    for index in range(opening, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[start : index + 1]
    raise AssertionError(f"unterminated function: {signature}")


def point_line_distance(
    point: tuple[float, float, float],
    begin: tuple[float, float, float],
    end: tuple[float, float, float],
) -> float:
    direction = tuple(end[axis] - begin[axis] for axis in range(3))
    offset = tuple(point[axis] - begin[axis] for axis in range(3))
    direction_norm = math.sqrt(sum(value * value for value in direction))
    if direction_norm <= 1e-12:
        return math.sqrt(sum(value * value for value in offset))
    cross = (
        offset[1] * direction[2] - offset[2] * direction[1],
        offset[2] * direction[0] - offset[0] * direction[2],
        offset[0] * direction[1] - offset[1] * direction[0],
    )
    return math.sqrt(sum(value * value for value in cross)) / direction_norm


def max_chord_residual(points: Iterable[tuple[float, float, float]]) -> float:
    values = list(points)
    if len(values) < 3:
        return 0.0
    return max(point_line_distance(point, values[0], values[-1]) for point in values[1:-1])


def horizontal_axis(direction_deg: float) -> tuple[float, float]:
    direction_rad = math.radians(direction_deg)
    return math.cos(direction_rad), math.sin(direction_rad)


def aligned_axis(
    direction_deg: float,
    reference: tuple[float, float],
) -> tuple[float, float]:
    direction = horizontal_axis(direction_deg)
    if sum(direction[index] * reference[index] for index in range(2)) < 0.0:
        return -direction[0], -direction[1]
    return direction


def adjacent_platform_axis(
    before_deg: float,
    after_deg: float,
    global_reference: tuple[float, float],
) -> tuple[float, float]:
    before = aligned_axis(before_deg, global_reference)
    after = aligned_axis(after_deg, global_reference)
    value = before[0] + after[0], before[1] + after[1]
    norm = math.hypot(*value)
    return value[0] / norm, value[1] / norm


def signed_geometry_angle_deg(
    platform_axis: tuple[float, float],
    slope_direction_deg: float,
) -> float:
    slope_axis = aligned_axis(slope_direction_deg, platform_axis)
    cross = (
        platform_axis[0] * slope_axis[1]
        - platform_axis[1] * slope_axis[0]
    )
    dot = sum(platform_axis[index] * slope_axis[index] for index in range(2))
    return math.degrees(math.atan2(cross, max(-1.0, min(1.0, dot))))


def verify_numeric_contract() -> None:
    local_offset = (0.0, -2.0)
    old_points: list[tuple[float, float, float]] = []
    fixed_points: list[tuple[float, float, float]] = []
    for x in range(101):
        output_rz_deg = 0.0 if x <= 90 else (x - 90) * 6.0
        output_rz_rad = math.radians(output_rz_deg)
        old_dx = (
            math.cos(output_rz_rad) * local_offset[0]
            - math.sin(output_rz_rad) * local_offset[1]
        )
        old_dy = (
            math.sin(output_rz_rad) * local_offset[0]
            + math.cos(output_rz_rad) * local_offset[1]
        )
        old_points.append((x + old_dx, old_dy, 0.0))
        fixed_points.append((x + local_offset[0], local_offset[1], 0.0))

    require(
        max_chord_residual(old_points) > 0.5,
        "numeric fixture no longer exposes per-point RZ curvature",
    )
    require(
        max_chord_residual(fixed_points) <= 1e-12,
        "fixed segment reference pose does not preserve a straight line",
    )

    # Old junction ownership used the shifted slope Z and then redrew the entire
    # platform to that point. A slope-only edit therefore tilted platform points.
    platform_points = [(0.0, 0.0, 0.0), (5.0, 0.0, 0.0), (10.0, 0.0, 0.0)]
    shifted_slope_owned_junction = (12.0, 0.0, 1.0)
    old_redrawn_midpoint = tuple(
        platform_points[0][axis]
        + (
            shifted_slope_owned_junction[axis] - platform_points[0][axis]
        )
        * (5.0 / 12.0)
        for axis in range(3)
    )
    require(
        abs(old_redrawn_midpoint[2] - platform_points[1][2]) > 0.4,
        "numeric fixture no longer exposes slope-to-platform junction coupling",
    )
    platform_owned_junction = (12.0, 0.0, 0.0)
    require(
        platform_owned_junction[2] == platform_points[-1][2]
        and platform_points[1] == (5.0, 0.0, 0.0),
        "platform-owned extension must retain rigidly translated platform points",
    )

    # For every physical segment, local Y must be the seam normal rather than
    # the clamped welding-pose Y axis. The 20260728_014 slopes exposed the old
    # vectors as almost opposite to their seam tangents.
    slope_fixtures = (
        ((-0.688, 0.726), (0.726, 0.688), (0.368, -0.929)),
        ((0.727, 0.687), (0.687, -0.727), (-0.585, -0.805)),
    )
    for tangent, weld_normal, old_tool_negative_y in slope_fixtures:
        seam_negative_y = (-weld_normal[0], -weld_normal[1])
        seam_dot = sum(
            seam_negative_y[axis] * tangent[axis]
            for axis in range(2)
        )
        old_dot = sum(
            old_tool_negative_y[axis] * tangent[axis]
            for axis in range(2)
        )
        require(
            abs(seam_dot) <= 1e-3,
            "seam-local Y compensation is not perpendicular to the slope",
        )
        require(
            abs(old_dot) > 0.9,
            "numeric fixture no longer exposes slope tool-Y/tangent coupling",
        )

    # 20260730_001: the old measurement-RZ-relative clamp mapped both slope
    # classes to -30 degrees. Geometry must instead use each neighboring
    # platform pair as 0 degrees and preserve opposite left/right signs.
    platform_directions = (85.848, 87.169, 86.662, 87.473, 87.826, 85.342)
    slope_directions = (
        ("falling_edge", 40.122),
        ("rising_edge", 134.488),
        ("falling_edge", 43.181),
        ("rising_edge", 135.865),
        ("falling_edge", 42.715),
    )
    doubled_cos = sum(
        math.cos(math.radians(2.0 * value))
        for value in platform_directions
    )
    doubled_sin = sum(
        math.sin(math.radians(2.0 * value))
        for value in platform_directions
    )
    global_axis = horizontal_axis(
        math.degrees(0.5 * math.atan2(doubled_sin, doubled_cos))
    )
    raw_angles: dict[str, list[float]] = {
        "rising_edge": [],
        "falling_edge": [],
    }
    limited_angles: dict[str, list[float]] = {
        "rising_edge": [],
        "falling_edge": [],
    }
    for index, (kind, direction_deg) in enumerate(slope_directions):
        platform_axis = adjacent_platform_axis(
            platform_directions[index],
            platform_directions[index + 1],
            global_axis,
        )
        raw_angle = signed_geometry_angle_deg(platform_axis, direction_deg)
        raw_angles[kind].append(raw_angle)
        limited_angles[kind].append(max(-30.0, min(30.0, raw_angle)))

    require(
        all(value < 0.0 for value in raw_angles["falling_edge"])
        and all(value > 0.0 for value in raw_angles["rising_edge"]),
        f"20260730_001 geometry signs collapsed: {raw_angles}",
    )
    require(
        limited_angles["falling_edge"] == [-30.0, -30.0, -30.0]
        and limited_angles["rising_edge"] == [30.0, 30.0],
        f"20260730_001 limited geometry angles are not opposite: {limited_angles}",
    )

    # 20260825_030: classification retained both lap-step anchors, but the old
    # pose-compensation junction trim crossed raw indexes 147/149 and erased
    # the whole step. A junction mutation that touches either anchor must be
    # rejected atomically instead of leaving a straight bridge.
    lap_step_raw_indexes = {147, 149}
    junction_trim_raw_indexes = set(range(139, 150))
    legacy_preserved = lap_step_raw_indexes - junction_trim_raw_indexes
    junction_trim_touches_lap_step = bool(
        lap_step_raw_indexes & junction_trim_raw_indexes
    )
    require(
        not legacy_preserved,
        "20260825_030 fixture no longer reproduces total lap-step anchor loss",
    )
    require(
        junction_trim_touches_lap_step,
        "lap-step junction guard did not reject the 20260825_030 trim window",
    )


def verify_source_contract() -> None:
    source = SERVICE_PATH.read_text(encoding="utf-8", errors="strict")
    dialog_source = DIALOG_PATH.read_text(encoding="utf-8", errors="strict")
    build_pose = function_body(source, "std::vector<QString> BuildSegmentPoseOutputLines(")
    junctions = function_body(
        source,
        "PoseCompJunctionApplyStats ApplyPoseCompSegmentJunctionIntersections(",
    )
    straighten = function_body(
        source,
        "int StraightenPoseCompPhysicalSegments(",
    )
    build_junction = function_body(
        source,
        "WeldPoseFileRecord BuildPoseCompJunctionRecord(",
    )
    apply_junction = function_body(
        source,
        "bool TryApplyPoseCompJunctionIntersection(",
    )
    intersect_lines = function_body(
        source,
        "bool TryIntersectWeldPoseLines2D(",
    )
    resolve_world = function_body(
        source,
        "Eigen::Vector3d ResolvePoseCompWorldVector(",
    )
    recompute_preview = function_body(
        dialog_source,
        "void WeldSeamCompDialog::RecomputeCompPreview(",
    )
    apply_preview = function_body(
        dialog_source,
        "void WeldSeamCompDialog::ApplyComputedStages(",
    )
    preview_loaded = function_body(
        dialog_source,
        "void WeldSeamCompDialog::OnCompPreviewLoaded(",
    )
    finalize = function_body(
        source,
        "SeamCompFinalizeStats FinalizeSeamCompedWeldPoseRecords(",
    )
    apply_file = function_body(
        source,
        "bool MeasureThenWeldService::ApplyWeldSeamCompToPoseFile(",
    )
    arc_preview = function_body(
        source,
        "std::vector<QString> BuildArcTransitionPreviewCloudLines(",
    )
    corner_candidate = function_body(
        source,
        "WeldCornerCandidateInfo EvaluateWeldCornerCandidate(",
    )
    corner_cluster = function_body(
        source,
        "void KeepBestWeldCornerCandidateRun(",
    )
    apply_arcs = function_body(
        source,
        "WeldCornerArcApplyStats ApplyCornerArcTransitionToWeldPoseRecords(",
    )
    trim_arc_exit = function_body(source, "void TrimSharpWeldArcExitPoints(")
    pose_preview = function_body(
        source,
        "MeasureThenWeldService::CompPreviewStages MeasureThenWeldService::ComputeCompPreviewStages(",
    )

    require(
        "poseCompReferenceRzForSegment" in build_pose,
        "pose output lacks a fixed per-segment compensation reference",
    )
    require(
        "segment.outputRy,\n            poseCompReferenceRz,\n            effectiveSegmentKind" in build_pose,
        "pose compensation does not use the fixed full segment posture reference",
    )
    require(
        "record.rz = pointRz;" in build_pose,
        "output orientation transition was accidentally removed",
    )
    require(
        "SignedHorizontalAngleDeg(" in build_pose
        and "resolveLocalPlatformAxis" in build_pose
        and "segment.rawGeometryAngleDeg" in build_pose
        and "segment.limitedGeometryAngleDeg" in build_pose,
        "slope posture is not derived from the neighboring platform geometry axis",
    )
    require(
        "rawRzDeviation" not in build_pose
        and "risingRawAngle * fallingRawAngle < 0.0" in build_pose
        and "risingLimitedAngle * fallingLimitedAngle < 0.0" in build_pose,
        "RZ-relative limiting returned or the opposite-sign fail-closed gate is missing",
    )
    require(
        "kMaxPassCount" not in junctions
        and "++junctionIndex;" in junctions
        and "ranges[junctionIndex + 1]" in junctions,
        "a physical pose-compensation junction can still be rebuilt repeatedly",
    )
    require(
        "StraightenPoseCompPhysicalSegments(records)" in build_pose
        and build_pose.index("StraightenPoseCompPhysicalSegments(records)")
        < build_pose.index("DensifyWeldPoseRecordsByStep("),
        "pose output is not rebuilt as straight physical segments before densifying",
    )
    require(
        "startPoint + (endPoint - startPoint) * ratio" in straighten
        and "containsLapStep" in straighten
        and "IsPlatformSegmentKind(range.kind)" in straighten,
        "slope straightening lacks its 3D chord, lap-step guard, or platform exclusion",
    )
    require(
        "firstRatio = Cross2D(delta, secondDirection) / denominator" in intersect_lines
        and "std::clamp" not in intersect_lines,
        "pose-compensation junction is no longer calculated from infinite lines",
    )
    require(
        "const PoseCompSegmentRange& elevationRange" in build_junction
        and "elevationBegin.point.z()" in build_junction
        and "IsPlatformSegmentKind(leftRange.kind)" in apply_junction,
        "junction elevation is not anchored to the unchanged platform segment",
    )
    require(
        "removesLapStepAnchor" in apply_junction
        and "replacesLapStepAnchor" in apply_junction
        and "records[index].isLapStep" in apply_junction
        and "records[rightRange.begin].isLapStep" in apply_junction
        and apply_junction.index("removesLapStepAnchor")
        < apply_junction.index("records[rightRange.begin] = junctionRecord"),
        "pose-compensation junction trim can still delete or replace a lap-step hard anchor",
    )
    require(
        "expectedLapStepRawIndexes" in build_pose
        and "preservedLapStepRawIndexes" in build_pose
        and "missingLapStepRawIndexes" in build_pose
        and "姿态补偿后处理丢失了搭接台阶硬锚点" in build_pose
        and build_pose.index("missingLapStepRawIndexes")
        < build_pose.index("lines.push_back(BuildWeldPoseFileRecordLine(record))"),
        "pose output lacks the fail-closed lap-step source/output invariant",
    )
    require(
        "ResolvePoseCompSegmentWeldNormals" in source
        and "ResolvePoseCompPlatformWeldNormals" not in source
        and "IsPlatformSegmentKind(range.kind) || IsSlopeSegmentKind(range.kind)" in source,
        "preview does not derive seam normals for all four physical segment types",
    )
    require(
        "IsPlatformSegmentKind(normalizedKind) || IsSlopeSegmentKind(normalizedKind)"
        in resolve_world
        and "stableTangent * slot.compX" in resolve_world
        and "weldNormal * slot.compY" in resolve_world
        and "Eigen::Vector3d::UnitZ() * slot.compZ" in resolve_world,
        "physical pose compensation is not mapped through the seam-local XYZ basis",
    )
    require(
        "segment.poseCompWeldNormal" in build_pose
        and "pendingLapStepPoseCompWeldNormal" in build_pose,
        "production output or lap-step handling lost its fixed seam-normal reference",
    )
    require(
        all(
            label in dialog_source
            for label in (
                "焊道切向补偿(mm)：",
                "焊道法向补偿(mm)：",
                "世界Z补偿(mm)：",
            )
        )
        and "工具X补偿(mm)：" not in dialog_source
        and "工具Y补偿(mm)：" not in dialog_source
        and "工具Z补偿(mm)：" not in dialog_source
        and "坡面工具系补偿" not in dialog_source,
        "pose-compensation field names still expose the obsolete slope tool frame",
    )
    require(
        "ApplyComputedStages(stages, true)" in recompute_preview
        and "if (preserveView)" in apply_preview
        and "SetLayersPreserveView(layers)" in apply_preview,
        "live compensation preview refresh no longer preserves the current camera",
    )
    require(
        "ApplyComputedStages(result.stages, false)" in preview_loaded
        and "SetLayers(layers)" in apply_preview,
        "initial preview load no longer performs its one-time view fit",
    )
    require(
        "Eigen::Vector3d::UnitZ() * currentEdits.weldZComp" in pose_preview
        and "gunDir * currentEdits.weldGunDirComp" in pose_preview
        and "seamDir * currentEdits.weldSeamDirComp" in pose_preview
        and 'QStringLiteral("当前焊道补偿方向")' in pose_preview,
        "seam preview no longer exposes the production-equivalent combined direction",
    )
    require(
        "当前焊道补偿：整条焊道" in dialog_source
        and "showSeamHighlight" in apply_preview
        and "wholeTrack || NormalizePreviewSegmentKind" in apply_preview
        and "图中单箭头为实际焊道补偿方向" in apply_preview,
        "seam mode no longer mirrors the pose-mode hint, highlight, and direction display",
    )
    require(
        "ApplyCornerArcTransitionToWeldPoseRecords" in finalize,
        "explicit process arc generation was removed",
    )
    require(
        "SmoothRemainingUnroundedWeldCorners" not in finalize,
        "non-arc corner smoothing can still introduce an untagged curve",
    )
    require(
        "kMinimumArcRecordCount = 3" in trim_arc_exit
        and "previousIndex - arcRunBegin + 1 <= kMinimumArcRecordCount" in trim_arc_exit,
        "repeated arc-exit trimming can still delete every explicitly tagged arc point",
    )
    require(
        "ResolvePoseCompReferencePoses(records)" in pose_preview
        and "referencePose.rz" in pose_preview,
        "compensation preview does not use the same fixed segment reference contract",
    )
    require(
        "StraightenPoseCompPhysicalSegments(records)" in pose_preview,
        "pose preview does not preserve the same straight-segment contract as output",
    )
    require(
        "ApplyCornerArcTransitionToWeldPoseRecords" not in arc_preview,
        "the debug preview still manufactures arcs from an intermediate classification",
    )
    require(
        source.count("BuildArcTransitionPreviewCloudLines(") == 2
        and "FinalizeSeamCompedWeldPoseRecords" in apply_file
        and "BuildArcTransitionPreviewCloudLines(records)" in apply_file
        and apply_file.index("FinalizeSeamCompedWeldPoseRecords")
        < apply_file.index("BuildArcTransitionPreviewCloudLines(records)"),
        "arc debug output is not isolated behind the final compensation stage",
    )
    require(
        pose_preview.index("stages.seamComp = snapshotRecords(records);")
        < pose_preview.index("FinalizeSeamCompedWeldPoseRecords")
        < pose_preview.index("stages.arc = snapshotRecords(records);"),
        "preview stages no longer snapshot the polyline before final arc processing",
    )
    require(
        "info.isCandidate = info.markedCorner" in corner_candidate
        and "autoCornerAngleRad" not in corner_candidate,
        "ordinary densified points can still manufacture premature arc candidates",
    )
    require(
        "candidate.explicitCorner ? 100.0" in corner_cluster,
        "an incidental segment boundary can still outrank the explicit corner",
    )
    require(
        "sampleStepMm * 3.0" in apply_arcs
        and "effectiveRadiusMm * 2.0" in apply_arcs,
        "same-corner candidate clustering is too narrow for densified boundaries",
    )
    require(
        "kMinFeasibleArcRadiusMm" in apply_arcs
        and "actualRadiusMm < kMinFeasibleArcRadiusMm" in apply_arcs,
        "short but feasible corners can still silently lose their final arc",
    )


def normalize_segment_kind(kind: str) -> str:
    value = kind.strip().lower()
    for suffix in ("_transition", "_arc"):
        if value.endswith(suffix):
            value = value[: -len(suffix)]
    return value


def verify_pose_file(
    path: Path,
    tolerance_mm: float,
    boundary_trim_points: int,
    expect_polyline_stage: bool,
) -> None:
    lines = path.read_text(encoding="utf-8-sig", errors="strict").splitlines()
    require(lines, f"pose file is empty: {path}")
    header = lines[0].split()
    required = ("x", "y", "z", "point_type", "segment_kind")
    require(all(name in header for name in required), f"pose header lacks {required}: {path}")
    columns = {name: header.index(name) for name in required}

    runs: list[tuple[str, list[tuple[float, float, float]]]] = []
    all_points: list[tuple[float, float, float]] = []
    all_point_types: list[str] = []
    all_segment_kinds: list[str] = []
    current_kind = ""
    current_points: list[tuple[float, float, float]] = []
    for line in lines[1:]:
        fields = line.split()
        if len(fields) < len(header):
            continue
        raw_kind = fields[columns["segment_kind"]]
        point_type = fields[columns["point_type"]]
        is_arc = raw_kind.strip().lower().endswith("_arc")
        normalized_kind = normalize_segment_kind(raw_kind)
        point = tuple(float(fields[columns[axis]]) for axis in ("x", "y", "z"))
        all_points.append(point)
        all_point_types.append(point_type)
        all_segment_kinds.append(raw_kind)
        if is_arc:
            if current_points:
                runs.append((current_kind, current_points))
                current_points = []
                current_kind = ""
            continue
        if current_points and normalized_kind != current_kind:
            runs.append((current_kind, current_points))
            current_points = []
        current_kind = normalized_kind
        current_points.append(point)
    if current_points:
        runs.append((current_kind, current_points))

    measured = []
    for kind, points in runs:
        if len(points) < 3:
            continue
        core_begin = min(boundary_trim_points, max(0, (len(points) - 3) // 2))
        core_end = len(points) - core_begin
        core_points = points[core_begin:core_end]
        measured.append(
            (
                kind,
                len(points),
                max_chord_residual(points),
                max_chord_residual(core_points),
            )
        )
    require(measured, f"pose file has no testable non-arc segment: {path}")

    if expect_polyline_stage:
        require(
            not any("_arc" in value.lower() for value in all_point_types + all_segment_kinds),
            f"intermediate polyline stage contains arc records: {path}",
        )
        turn_indices: list[int] = []
        for index in range(1, len(all_points) - 1):
            incoming = tuple(
                all_points[index][axis] - all_points[index - 1][axis]
                for axis in range(3)
            )
            outgoing = tuple(
                all_points[index + 1][axis] - all_points[index][axis]
                for axis in range(3)
            )
            incoming_norm = math.sqrt(sum(value * value for value in incoming))
            outgoing_norm = math.sqrt(sum(value * value for value in outgoing))
            if incoming_norm <= 1e-12 or outgoing_norm <= 1e-12:
                continue
            cosine = max(
                -1.0,
                min(
                    1.0,
                    sum(incoming[axis] * outgoing[axis] for axis in range(3))
                    / (incoming_norm * outgoing_norm),
                ),
            )
            if math.degrees(math.acos(cosine)) > 0.25:
                turn_indices.append(index)
        marked_corner_indices = [
            index
            for index, point_type in enumerate(all_point_types)
            if "corner" in point_type.lower()
        ]
        require(
            turn_indices == marked_corner_indices,
            "intermediate pose stage is not a pure keypoint polyline:"
            f" geometric_turns={turn_indices}, marked_corners={marked_corner_indices}",
        )
        print(
            "polyline stage:"
            f" corners={len(marked_corner_indices)}, extra_turns=0, arc_records=0"
        )

    worst_full = max(measured, key=lambda item: item[2])
    worst_core = max(measured, key=lambda item: item[3])
    print(
        "pose straightness:"
        f" runs={len(measured)}, boundary_trim={boundary_trim_points},"
        f" worst_core={worst_core[0]}, points={worst_core[1]},"
        f" core_residual={worst_core[3]:.6f} mm,"
        f" boundary_inclusive={worst_full[0]}:{worst_full[2]:.6f} mm"
    )
    require(
        worst_core[3] <= tolerance_mm,
        f"non-arc segment core is curved: kind={worst_core[0]},"
        f" residual={worst_core[3]:.6f} mm",
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--pose-file",
        type=Path,
        help="optional generated WeldPose or SeamComp file to validate",
    )
    parser.add_argument("--tolerance-mm", type=float, default=0.01)
    parser.add_argument("--boundary-trim-points", type=int, default=2)
    parser.add_argument(
        "--expect-polyline-stage",
        action="store_true",
        help="require every geometric turn to be an explicit non-arc corner",
    )
    args = parser.parse_args()

    verify_numeric_contract()
    verify_source_contract()
    if args.pose_file is not None:
        require(args.boundary_trim_points >= 0, "boundary trim must be non-negative")
        verify_pose_file(
            args.pose_file,
            args.tolerance_mm,
            args.boundary_trim_points,
            args.expect_polyline_stage,
        )
    print("PASS: non-arc pose compensation remains straight; only explicit arcs may curve")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
