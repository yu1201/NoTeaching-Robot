#!/usr/bin/env python3
"""Replay SchemeCompare BaseWeld artifacts against quality profile v1.

This is an offline audit tool. It never edits Result cases; it writes a JSONL
record per case and a compact CSV summary into the requested output directory.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any

import numpy as np


PROFILE_VERSION = 1
THRESHOLDS = {
    "min_finite_points": 300,
    "min_span_mm": 180.0,
    "min_station_coverage": 0.55,
    "min_longest_continuous": 0.60,
    "max_rejected_ratio": 0.40,
    "max_median_residual_mm": 3.0,
    "max_p95_residual_mm": 8.0,
    "residual_inlier_mm": 6.0,
    "min_residual_inlier_ratio": 0.75,
    "min_key_points": 6,
    "min_corners": 4,
    "min_non_lap_hard_mm": 3.0,
    "min_lap_hard_mm": 0.25,
    "min_endpoint_hard_mm": 0.25,
    "short_segment_warning_mm": 15.0,
    "min_output_points": 80,
    "min_output_length_ratio": 0.70,
    "max_output_step_mm": 50.0,
    "sample_step_mm": 2.0,
}


def read_points(path: Path) -> list[dict[str, Any]]:
    points: list[dict[str, Any]] = []
    if not path.is_file():
        return points
    for raw in path.read_text(encoding="utf-8-sig", errors="replace").splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        fields = line.replace(",", " ").split()
        if len(fields) < 4:
            continue
        try:
            xyz = tuple(float(value) for value in fields[1:4])
        except ValueError:
            continue
        if not all(math.isfinite(value) for value in xyz):
            continue
        points.append({
            "xyz": xyz,
            "type": fields[5].lower() if len(fields) > 5 else "",
            "source": fields[6].lower() if len(fields) > 6 else "",
        })
    return points


def principal_projection(points: list[dict[str, Any]]) -> tuple[np.ndarray, np.ndarray, float]:
    xyz = np.asarray([point["xyz"] for point in points], dtype=float)
    center = xyz.mean(axis=0)
    covariance = np.cov((xyz - center).T, bias=True)
    values, vectors = np.linalg.eigh(covariance)
    main = vectors[:, int(np.argmax(values))]
    if float(np.dot(main, xyz[-1] - xyz[0])) < 0.0:
        main = -main
    remaining = [index for index in range(3) if index != int(np.argmax(values))]
    side = vectors[:, remaining[int(np.argmax(values[remaining]))]]
    side = side - main * float(np.dot(main, side))
    side /= max(float(np.linalg.norm(side)), 1e-12)
    projected = np.column_stack(((xyz - center) @ main, (xyz - center) @ side))
    span = float(np.ptp(projected[:, 0]))
    return projected, center, span


def point_segment_distance(point: np.ndarray, begin: np.ndarray, end: np.ndarray) -> float:
    delta = end - begin
    denominator = float(np.dot(delta, delta))
    if denominator <= 1e-18:
        return float(np.linalg.norm(point - begin))
    ratio = max(0.0, min(1.0, float(np.dot(point - begin, delta)) / denominator))
    return float(np.linalg.norm(point - (begin + ratio * delta)))


def polyline_length(points: np.ndarray) -> float:
    if len(points) < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())


def audit_case(input_path: Path, result_root: Path) -> dict[str, Any]:
    compare_dir = input_path.parent
    prefix = "BaseWeldPointCloudFit"
    raw = read_points(input_path)
    preserve = read_points(compare_dir / f"{prefix}_PreservePath.txt")
    keys = read_points(compare_dir / f"{prefix}_KeyPoints.txt")
    output = read_points(compare_dir / f"{prefix}_Classified_2mm.txt")
    failures: list[str] = []
    warnings: list[str] = []

    if len(preserve) < 2 or len(keys) < 2 or len(output) < 2:
        failures.append("missing_or_too_short_artifact")
        return {
            "profile_version": PROFILE_VERSION,
            "case": str(compare_dir.relative_to(result_root)),
            "passed": False,
            "failures": failures,
            "warnings": warnings,
            "input_points": len(raw),
            "preserve_points": len(preserve),
            "key_points": len(keys),
            "output_points": len(output),
        }

    projected, center, span = principal_projection(preserve)
    xyz = np.asarray([point["xyz"] for point in preserve], dtype=float)
    key_xyz = np.asarray([point["xyz"] for point in keys], dtype=float)
    covariance = np.cov((xyz - center).T, bias=True)
    values, vectors = np.linalg.eigh(covariance)
    main = vectors[:, int(np.argmax(values))]
    if float(np.dot(main, xyz[-1] - xyz[0])) < 0.0:
        main = -main
    side_index = int(np.argsort(values)[-2])
    side = vectors[:, side_index]
    side = side - main * float(np.dot(main, side))
    side /= max(float(np.linalg.norm(side)), 1e-12)
    key_projected = np.column_stack(((key_xyz - center) @ main, (key_xyz - center) @ side))

    bin_count = max(1, int(math.floor(span / THRESHOLDS["sample_step_mm"])) + 1)
    occupied = np.zeros(bin_count, dtype=bool)
    min_station = float(projected[:, 0].min())
    indexes = np.floor((projected[:, 0] - min_station) / THRESHOLDS["sample_step_mm"]).astype(int)
    occupied[np.clip(indexes, 0, bin_count - 1)] = True
    station_coverage = float(occupied.mean())
    longest = current = 0
    for value in occupied:
        current = current + 1 if value else 0
        longest = max(longest, current)
    longest_ratio = longest / bin_count

    residuals = np.asarray([
        min(point_segment_distance(point, key_projected[i], key_projected[i + 1])
            for i in range(len(key_projected) - 1))
        for point in projected
    ], dtype=float)
    median_residual = float(np.median(residuals))
    p95_residual = float(np.quantile(residuals, 0.95))
    inlier_ratio = float(np.mean(residuals <= THRESHOLDS["residual_inlier_mm"]))
    rejected_ratio = max(0, len(raw) - len(preserve)) / max(1, len(raw))

    key_segments = np.linalg.norm(np.diff(key_xyz, axis=0), axis=1)
    min_non_lap = math.inf
    min_lap = math.inf
    for index, length in enumerate(key_segments, start=1):
        lap_pair = (keys[index - 1]["source"] == "geometry_lap_step"
                    and keys[index]["source"] == "geometry_lap_step")
        endpoint_adjacent = index == 1 or index == len(key_segments)
        if lap_pair:
            min_lap = min(min_lap, float(length))
            if length < THRESHOLDS["min_lap_hard_mm"]:
                failures.append("lap_segment_below_hard_min")
        elif endpoint_adjacent:
            min_non_lap = min(min_non_lap, float(length))
            if length < THRESHOLDS["min_endpoint_hard_mm"]:
                failures.append("endpoint_segment_below_hard_min")
            elif length < THRESHOLDS["short_segment_warning_mm"]:
                warnings.append(f"short_endpoint_segment_{index}:{length:.3f}mm")
        else:
            min_non_lap = min(min_non_lap, float(length))
            if length < THRESHOLDS["min_non_lap_hard_mm"]:
                failures.append("non_lap_segment_below_hard_min")
            elif length < THRESHOLDS["short_segment_warning_mm"]:
                warnings.append(f"short_non_lap_segment_{index}:{length:.3f}mm")

    start_count = sum(point["type"] == "start" for point in keys)
    end_count = sum(point["type"] == "end" for point in keys)
    corner_count = sum("corner" in point["type"] for point in keys)
    output_xyz = np.asarray([point["xyz"] for point in output], dtype=float)
    max_output_step = float(np.linalg.norm(np.diff(output_xyz, axis=0), axis=1).max())
    output_length = polyline_length(key_xyz)
    output_length_ratio = output_length / max(span, 1e-12)

    checks = {
        "finite_point_count": len(raw) >= THRESHOLDS["min_finite_points"],
        "projected_span": span >= THRESHOLDS["min_span_mm"],
        "station_coverage": station_coverage >= THRESHOLDS["min_station_coverage"],
        "longest_continuous": longest_ratio >= THRESHOLDS["min_longest_continuous"],
        "rejected_ratio": rejected_ratio <= THRESHOLDS["max_rejected_ratio"],
        "median_residual": median_residual <= THRESHOLDS["max_median_residual_mm"],
        "p95_residual": p95_residual <= THRESHOLDS["max_p95_residual_mm"],
        "residual_inlier_ratio": inlier_ratio >= THRESHOLDS["min_residual_inlier_ratio"],
        "unique_start_end": start_count == 1 and end_count == 1,
        "key_point_count": len(keys) >= THRESHOLDS["min_key_points"],
        "corner_count": corner_count >= THRESHOLDS["min_corners"],
        "key_monotonic": bool(np.all(np.diff(key_projected[:, 0]) > 1e-6)),
        "output_point_count": len(output) >= THRESHOLDS["min_output_points"],
        "output_length_ratio": output_length_ratio >= THRESHOLDS["min_output_length_ratio"],
        "max_output_step": max_output_step <= THRESHOLDS["max_output_step_mm"],
    }
    failures.extend(name for name, passed in checks.items() if not passed)
    return {
        "profile_version": PROFILE_VERSION,
        "case": str(compare_dir.relative_to(result_root)),
        "passed": not failures,
        "failures": sorted(set(failures)),
        "warnings": warnings,
        "checks": checks,
        "input_points": len(raw),
        "preserve_points": len(preserve),
        "key_points": len(keys),
        "corner_count": corner_count,
        "output_points": len(output),
        "projected_span_mm": span,
        "station_coverage_ratio": station_coverage,
        "longest_continuous_ratio": longest_ratio,
        "rejected_ratio": rejected_ratio,
        "median_residual_mm": median_residual,
        "p95_residual_mm": p95_residual,
        "residual_inlier_ratio": inlier_ratio,
        "min_non_lap_segment_mm": 0.0 if math.isinf(min_non_lap) else min_non_lap,
        "min_lap_segment_mm": 0.0 if math.isinf(min_lap) else min_lap,
        "output_length_mm": output_length,
        "output_length_ratio": output_length_ratio,
        "max_output_step_mm": max_output_step,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--result-root", type=Path, default=Path("Result"))
    parser.add_argument("--output-dir", type=Path, default=Path("Temp/PointCloudQualityAudit"))
    parser.add_argument("--fail-on-rejected", action="store_true")
    args = parser.parse_args()
    result_root = args.result_root.resolve()
    output_dir = args.output_dir.resolve()
    cases = sorted(result_root.rglob("BaseWeldPointCloudFit_InputPointCloud.txt"))
    records = [audit_case(path, result_root) for path in cases]
    output_dir.mkdir(parents=True, exist_ok=True)

    jsonl_path = output_dir / "quality_audit.jsonl"
    with jsonl_path.open("w", encoding="utf-8", newline="\n") as stream:
        for record in records:
            stream.write(json.dumps(record, ensure_ascii=False, sort_keys=True) + "\n")

    csv_path = output_dir / "quality_audit.csv"
    fields = [
        "case", "passed", "input_points", "preserve_points", "key_points", "corner_count",
        "output_points", "projected_span_mm", "station_coverage_ratio",
        "longest_continuous_ratio", "rejected_ratio", "median_residual_mm",
        "p95_residual_mm", "residual_inlier_ratio", "min_non_lap_segment_mm",
        "min_lap_segment_mm", "output_length_mm", "output_length_ratio",
        "max_output_step_mm", "failures", "warnings",
    ]
    with csv_path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for record in records:
            row = {field: record.get(field, "") for field in fields}
            row["failures"] = "|".join(record.get("failures", []))
            row["warnings"] = "|".join(record.get("warnings", []))
            writer.writerow(row)

    passed = sum(bool(record.get("passed")) for record in records)
    print(f"quality profile v{PROFILE_VERSION}: {passed}/{len(records)} passed")
    print(jsonl_path)
    print(csv_path)
    return 1 if args.fail_on_rejected and passed != len(records) else 0


if __name__ == "__main__":
    raise SystemExit(main())
