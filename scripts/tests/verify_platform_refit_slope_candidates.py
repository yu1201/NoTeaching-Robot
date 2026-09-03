#!/usr/bin/env python3
"""Regression gate for platform-refit candidate selection and individual slopes."""

from __future__ import annotations

import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SERVICE_PATH = ROOT / "src" / "MeasureThenWeldService.cpp"
VALIDATOR_PATH = ROOT / "src" / "PlatformSemanticValidator.cpp"


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


def verify_numeric_contract() -> None:
    # 20260826 _004: the aggregate rising/falling sign gate passed because the
    # other five rising slopes hid one false -2.358 degree slope. Every slope
    # segment must independently clear the configured flat/slope boundary.
    rising_geometry_angles_deg = (
        -25.023,
        -23.641,
        -23.811,
        -2.358,
        -14.750,
        -23.981,
    )
    falling_geometry_angles_deg = (22.397, 25.504, 24.034)
    require(
        sum(rising_geometry_angles_deg) / len(rising_geometry_angles_deg) < 0.0
        and sum(falling_geometry_angles_deg) / len(falling_geometry_angles_deg) > 0.0,
        "20260826 _004 fixture no longer passes the old aggregate sign gate",
    )
    min_slope_angle_deg = math.degrees(math.atan(0.15))
    require(
        abs(rising_geometry_angles_deg[3]) < min_slope_angle_deg,
        "20260826 _004 false slope no longer trips the individual slope gate",
    )


def verify_source_contract() -> None:
    source = SERVICE_PATH.read_text(encoding="utf-8", errors="strict")
    validator = VALIDATOR_PATH.read_text(encoding="utf-8", errors="strict")
    dual_candidate = function_body(
        source,
        "AnalyzeDirectWithPlatformRefitCandidateSelection(",
    )
    build_pose = function_body(
        source,
        "std::vector<QString> BuildSegmentPoseOutputLines(",
    )
    require(
        "refitOffParams.cornerPatternRefitEnable = false" in dual_candidate
        and "EvaluatePlatformRefitSlopeCandidate(refitOn" in dual_candidate
        and "EvaluatePlatformRefitSlopeCandidate(refitOff" in dual_candidate
        and "PlatformSemanticValidator::SelectCandidate" in dual_candidate
        and "AppendPlatformRefitCandidateDiagnostics" in dual_candidate
        and "自动回退关闭候选" in dual_candidate
        and "两个结果均存在异常斜率或平台语义错误" in dual_candidate
        and "双候选语义冲突" in dual_candidate,
        "platform-refit on/off candidates are not independently checked and selected fail-closed",
    )
    require(
        source.count("AnalyzeDirectWithPlatformRefitCandidateSelection(") >= 4,
        "dual candidate selection is not shared by all three fitted production paths",
    )
    require(
        "minSlopeGeometryAngleDeg" in build_pose
        and "检测到单一异常斜率" in build_pose
        and "std::abs(segment.rawGeometryAngleDeg)" in build_pose
        and build_pose.index("检测到单一异常斜率")
        < build_pose.index("risingRawAngle * fallingRawAngle < 0.0"),
        "pose generation lacks the per-slope hard gate before aggregate sign validation",
    )
    require(
        "RequiresTrustedSegmentAttributeCompensation(preset)" in build_pose
        and "!preset.measurementPoseTrusted" in build_pose
        and "PlatformSemanticValidator::EvaluateAssignedSegments" in build_pose
        and "平台属性补偿前复核失败" in build_pose
        and build_pose.index("PlatformSemanticValidator::EvaluateAssignedSegments")
        < build_pose.index("DefaultPoseCompSlotIndex(segment.kind)"),
        "platform attributes are not proven before four-slot pose compensation",
    )
    require(
        "preset.measurementPoseTrusted = true" in source
        and "fields[statusCol].trimmed() != \"laser_ok\"" in source,
        "platform depth reference is not bound to laser_ok measurement evidence",
    )
    require(
        "FitTwoBands" in validator
        and "slope direction conflicts with platform bands" in validator
        and "invalid segment transition=" in validator
        and "low platform is not farther along measurement gun depth" in validator,
        "semantic validator lacks band, direction, sequence, or low/high checks",
    )


if __name__ == "__main__":
    verify_numeric_contract()
    verify_source_contract()
    print("PASS: platform-refit candidates, platform attributes, and compensation gates fail closed")
