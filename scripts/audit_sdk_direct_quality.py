#!/usr/bin/env python3
"""Audit historical SDK FeaturePoint direct-output artifacts for profile v1.

The SDK worker's complete-cloud finite/invalid counters only exist at runtime;
this replay covers the remaining direct-output gates using the exact historical
key-point and resampled-track artifacts. Result data is read-only.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import numpy as np


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
        points.append({"xyz": xyz, "type": fields[5].lower() if len(fields) > 5 else ""})
    return points


def point_segment_distance(point: np.ndarray, begin: np.ndarray, end: np.ndarray) -> float:
    delta = end - begin
    denominator = float(np.dot(delta, delta))
    if denominator <= 1e-18:
        return float(np.linalg.norm(point - begin))
    ratio = float(np.clip(np.dot(point - begin, delta) / denominator, 0.0, 1.0))
    return float(np.linalg.norm(point - (begin + ratio * delta)))


def audit_case(key_path: Path, result_root: Path, sample_step_mm: float) -> dict[str, Any]:
    output_path = key_path.with_name("FeaturePoint_KeyPointExpanded_2mm.txt")
    keys = read_points(key_path)
    output = read_points(output_path)
    failures: list[str] = []
    warnings: list[str] = []
    case = str(key_path.parent.relative_to(result_root))
    if len(keys) < 2 or len(output) < 2:
        return {"case": case, "passed": False, "failures": ["missing_or_too_short_artifact"]}

    xyz = np.asarray([point["xyz"] for point in output], dtype=float)
    key_xyz = np.asarray([point["xyz"] for point in keys], dtype=float)
    center = xyz.mean(axis=0)
    values, vectors = np.linalg.eigh(np.cov((xyz - center).T, bias=True))
    main = vectors[:, int(np.argmax(values))]
    if float(np.dot(main, xyz[-1] - xyz[0])) < 0.0:
        main = -main
    side = vectors[:, int(np.argsort(values)[-2])]
    side -= main * float(np.dot(main, side))
    side /= max(float(np.linalg.norm(side)), 1e-12)
    projected = np.column_stack(((xyz - center) @ main, (xyz - center) @ side))
    key_projected = np.column_stack(((key_xyz - center) @ main, (key_xyz - center) @ side))
    span = float(np.ptp(projected[:, 0]))

    bin_width = max(0.5, sample_step_mm)
    bin_count = max(1, int(math.floor(span / bin_width)) + 1)
    occupied = np.zeros(bin_count, dtype=bool)
    indexes = np.floor((projected[:, 0] - float(projected[:, 0].min())) / bin_width).astype(int)
    occupied[np.clip(indexes, 0, bin_count - 1)] = True
    station_coverage = float(occupied.mean())
    longest = current = 0
    for value in occupied:
        current = current + 1 if value else 0
        longest = max(longest, current)
    longest_ratio = longest / bin_count

    residuals = np.asarray([
        min(point_segment_distance(point, key_projected[index], key_projected[index + 1])
            for index in range(len(key_projected) - 1))
        for point in projected
    ])
    median_residual = float(np.median(residuals))
    p95_residual = float(np.quantile(residuals, 0.95))
    inlier_ratio = float(np.mean(residuals <= 6.0))

    segment_lengths = np.linalg.norm(np.diff(key_xyz, axis=0), axis=1)
    for index, length in enumerate(segment_lengths, start=1):
        endpoint_adjacent = index == 1 or index == len(segment_lengths)
        hard_minimum = 0.25 if endpoint_adjacent else 3.0
        if length < hard_minimum:
            failures.append("endpoint_segment_below_hard_min" if endpoint_adjacent
                            else "interior_segment_below_hard_min")
        elif length < 15.0:
            warnings.append(f"short_segment_{index}:{length:.3f}mm")

    output_steps = np.linalg.norm(np.diff(xyz, axis=0), axis=1)
    key_length = float(segment_lengths.sum())
    checks = {
        "projected_span": span >= 180.0,
        "station_coverage": station_coverage >= 0.55,
        "longest_continuous": longest_ratio >= 0.60,
        "median_residual": median_residual <= 3.0,
        "p95_residual": p95_residual <= 8.0,
        "residual_inlier_ratio": inlier_ratio >= 0.75,
        "unique_start_end": sum(p["type"] == "start" for p in keys) == 1
                            and sum(p["type"] == "end" for p in keys) == 1,
        "key_point_count": len(keys) >= 6,
        "corner_count": sum("corner" in p["type"] for p in keys) >= 4,
        "key_monotonic": bool(np.all(np.diff(key_projected[:, 0]) > 1e-6)),
        "output_point_count": len(output) >= 80,
        "output_length_ratio": key_length / max(span, 1e-12) >= 0.70,
        "max_output_step": float(output_steps.max()) <= 50.0,
    }
    failures.extend(name for name, passed in checks.items() if not passed)
    return {
        "case": case,
        "passed": not failures,
        "failures": sorted(set(failures)),
        "warnings": warnings,
        "runtime_full_cloud_gate": "finite/invalid counters validated by SDK worker at runtime",
        "key_points": len(keys),
        "output_points": len(output),
        "projected_span_mm": span,
        "station_coverage_ratio": station_coverage,
        "longest_continuous_ratio": longest_ratio,
        "median_residual_mm": median_residual,
        "p95_residual_mm": p95_residual,
        "residual_inlier_ratio": inlier_ratio,
        "min_segment_mm": float(segment_lengths.min()),
        "output_length_ratio": key_length / max(span, 1e-12),
        "max_output_step_mm": float(output_steps.max()),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--result-root", type=Path, default=Path("Result"))
    parser.add_argument("--output", type=Path, default=Path("Temp/SdkDirectQualityAudit.jsonl"))
    parser.add_argument("--sample-step-mm", type=float, default=2.0)
    parser.add_argument("--fail-on-rejected", action="store_true")
    args = parser.parse_args()
    root = args.result_root.resolve()
    records = [audit_case(path, root, args.sample_step_mm)
               for path in sorted(root.rglob("FeaturePoint_KeyPoints.txt"))]
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="utf-8", newline="\n") as stream:
        for record in records:
            stream.write(json.dumps(record, ensure_ascii=False, sort_keys=True) + "\n")
    passed = sum(bool(record["passed"]) for record in records)
    print(f"SDK direct quality profile v1: {passed}/{len(records)} passed")
    print(output)
    return 1 if args.fail_on_rejected and passed != len(records) else 0


if __name__ == "__main__":
    raise SystemExit(main())
