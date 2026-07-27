#!/usr/bin/env python3
"""Regression gate: pose transitions must not curve non-arc weld segments."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Iterable


ROOT = Path(__file__).resolve().parents[2]
SERVICE_PATH = ROOT / "src" / "MeasureThenWeldService.cpp"


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


def verify_source_contract() -> None:
    source = SERVICE_PATH.read_text(encoding="utf-8", errors="strict")
    build_pose = function_body(source, "std::vector<QString> BuildSegmentPoseOutputLines(")
    junctions = function_body(
        source,
        "PoseCompJunctionApplyStats ApplyPoseCompSegmentJunctionIntersections(",
    )
    straighten = function_body(
        source,
        "int StraightenPoseCompPhysicalSegments(",
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
        "pointRy,\n            poseCompReferenceRz,\n            effectiveSegmentKind" in build_pose,
        "pose compensation still uses the per-point output RZ",
    )
    require(
        "record.rz = pointRz;" in build_pose,
        "output orientation transition was accidentally removed",
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
        < build_pose.index("DensifyWeldPoseRecordsByStep(records, kPoseCompOutputStepMm)"),
        "pose output is not rebuilt as straight physical segments before densifying",
    )
    require(
        "startPoint + (endPoint - startPoint) * ratio" in straighten
        and "containsLapStep" in straighten,
        "physical-segment straightening is missing its 3D chord or lap-step guard",
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
