"""Static guard for the camera point-coordinate boundary.

The three camera transports may expose different native Z conventions, but
CameraFrameCache and all business flows must receive allResultPoint in exactly
the same device XYZ convention as targetPoint.
"""

from __future__ import annotations

from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[2]


def read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def main() -> int:
    frame = read("include/groove/framebuffer.h")
    tcp = read("src/groove/scancameratcpclientworker.cpp")
    udp = read("src/groove/clientudpformsensorworker.cpp")
    skj = read("src/groove/scancameraskjworker.cpp")
    service = read("src/MeasureThenWeldService.cpp")

    for token in (
        "CameraNativePointCloudConvention",
        "SameAsTarget",
        "LegacyCloudZOppositeTarget",
        "CanonicalizeCameraPointCloudPoint(",
        "CanonicalizeCameraPointCloud(",
        "bool allResultPointCanonical = false",
    ):
        require(token in frame, f"camera coordinate contract missing: {token}")

    for name, source in (("legacy TCP", tcp), ("legacy UDP", udp)):
        require(
            "CanonicalizeCameraPointCloud(" in source
            and "CameraNativePointCloudConvention::LegacyCloudZOppositeTarget" in source,
            f"{name} must normalize its native opposite-Z cloud at the transport boundary",
        )
        require(
            "allResultPointCanonical = true" in source,
            f"{name} must attest that its cached cloud is canonical",
        )
        require(
            "allResultPoint = frame.dataPoints3D" not in source,
            f"{name} bypasses point-cloud canonicalization",
        )

    require(
        "CameraNativePointCloudConvention::SameAsTarget" in skj,
        "SKJ must preserve the common points3d/result_point device XYZ convention",
    )
    require(
        "allResultPointCanonical = true" in skj,
        "SKJ must attest that its cached cloud is canonical",
    )
    require(
        "LegacyCloudZOppositeTarget" not in skj,
        "SKJ must not inherit the legacy TCP/UDP Z inversion",
    )

    require(
        "Eigen::Vector3d cameraLinePoint(sourcePoint.x, sourcePoint.y, sourcePoint.z)" in service,
        "measure-then-weld must consume the canonical camera point without a brand-specific sign change",
    )
    require(
        "sourcePoint.y, -sourcePoint.z" not in service,
        "legacy Z inversion leaked back into the business layer",
    )
    for token in (
        "if (!readyCameraFrame.allResultPoint.empty()",
        "&& !readyCameraFrame.allResultPointCanonical)",
        "已在首条机器人运动前拒绝本轮扫描",
        "if (!frame.allResultPointCanonical)",
        "nonCanonicalCameraPointCloudFrameCount",
        "相机完整点云坐标契约失败",
        "已丢弃本轮扫描，禁止生成焊道",
        "相机完整点云坐标契约检查通过",
        "坐标约定=TargetDeviceXYZ",
    ):
        require(token in service, f"fail-closed scan contract missing: {token}")

    print(
        "PASS: legacy TCP/UDP normalize opposite Z in their transports, "
        "SKJ preserves device XYZ, and scan business consumes only attested canonical clouds"
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except AssertionError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        raise SystemExit(1)
