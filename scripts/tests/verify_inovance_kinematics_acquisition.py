#!/usr/bin/env python3
"""Offline regression for the read-only Inovance kinematics acquisition contract."""

from __future__ import annotations

import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
DRIVER = (ROOT / "src/InovanceRobotDriver.cpp").read_text(encoding="utf-8")
ADAPTOR = (ROOT / "include/RobotDriverAdaptor.h").read_text(encoding="utf-8")
ACCEPTANCE = (ROOT / "src/FunctionTestDialog.cpp").read_text(encoding="utf-8")


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


for token in (
    "ControllerKinematicsRead",
    "RobotKinematicsValidationResult",
    "RefreshKinematicsFromController",
):
    require(token in ADAPTOR, f"adaptor kinematics contract missing: {token}")

for token in (
    '"Get_RobotType"',
    '"Get_StrPara"',
    '"Get_StrParaComp"',
    '"Get_RdctRatio"',
    '"Get_CpParaM"',
    '"Get_CpParaS"',
    '"Get_ZeroPos"',
    '"Get_AxisNLim "',
    '"Get_AxisPLim "',
    '"Get_RobJPHere"',
    '"Get_PosHerePulse"',
    '"Get_ToolCNum"',
    '"Get_WobjNum"',
    '"Get_WobjData "',
    '"/RobotParams/MachineParams.json"',
    '"i32EncBit"',
    '"dCoupParam"',
    "InstallValidatedKinematicsModel",
):
    require(token in DRIVER, f"Inovance acquisition source missing: {token}")

for token in (
    "positionErrorMm > 2.0",
    "orientationErrorDeg > 0.1",
    "maxPulseJointError > 0.01",
    "2222接口值与FTP MachineParams.json不一致",
):
    require(token in DRIVER, f"Inovance fail-closed validation missing: {token}")

for token in (
    "控制器运动学资产=",
    "关节/直角闭环误差",
    "控制器机械数据已由品牌底层按固定来源获取并交叉校验",
):
    require(token in ACCEPTANCE, f"acceptance evidence missing: {token}")


# Read-only snapshot captured from the field controller on 2026-09-05.
# This fixture protects the documented pulse-zero-ratio-coupling convention and
# the standard-DH mapping. It never connects to or writes the controller.
raw_pulse = [-2768698, 22531858, -17987179, 10461640, -1420926, 14634072]
absolute_zero = [-649714, 277637.219, -672718, -636858.375, -313825.375, -752388]
ratios = [111.242, 160.977, 126.023, 75.714, 75.601, 48.189]
controller_joints = [-6.540, 47.462, -47.169, 50.326, -5.028, 104.499]
j6_from_j5 = -1.01886792453
units = [360.0 / ((2**20) * ratio) for ratio in ratios]
reconstructed_joints = [
    (pulse - zero) * unit
    for pulse, zero, unit in zip(raw_pulse, absolute_zero, units)
]
reconstructed_joints[5] -= j6_from_j5 * controller_joints[4]
max_joint_error = max(
    abs(calculated - reported)
    for calculated, reported in zip(reconstructed_joints, controller_joints)
)
require(max_joint_error < 0.01, f"pulse/joint closure regressed: {max_joint_error} deg")


def multiply(left: list[list[float]], right: list[list[float]]) -> list[list[float]]:
    return [
        [sum(left[row][k] * right[k][column] for k in range(4)) for column in range(4)]
        for row in range(4)
    ]


def dh(a_mm: float, alpha_deg: float, d_mm: float, theta_deg: float) -> list[list[float]]:
    alpha = math.radians(alpha_deg)
    theta = math.radians(theta_deg)
    ca, sa = math.cos(alpha), math.sin(alpha)
    ct, st = math.cos(theta), math.sin(theta)
    return [
        [ct, -st * ca, st * sa, a_mm * ct],
        [st, ct * ca, -ct * sa, a_mm * st],
        [0.0, sa, ca, d_mm],
        [0.0, 0.0, 0.0, 1.0],
    ]


identity = [[1.0 if row == column else 0.0 for column in range(4)] for row in range(4)]
structure = [169.864, 840.083, 205.217, 1037.391, 75.000, 500.000]
alpha = [89.98899078, 0.001699788, 90.02835083, -89.99526978, 90.05046844, -0.0204225]
a = [structure[0], structure[1], structure[2], -0.2084527, -0.02581433, 0.0]
d = [structure[5], 0.0, -10.888879776, structure[3], 0.110285699, structure[4]]
theta_offset = [0.0, 90.0, 0.0, 0.0, 0.0, 0.0]
calculated_flange = identity
for row in range(6):
    calculated_flange = multiply(
        calculated_flange,
        dh(a[row], alpha[row], d[row], theta_offset[row] + controller_joints[row]),
    )

# Independent base-flange result obtained from active Wobj * current TCP * Tool^-1.
controller_flange_position = [659.4308913219, -58.4037385385, 1274.0901490258]
controller_flange_rotation = [
    [-0.0646882363, 0.0194139464, 0.9977166586],
    [-0.4214474250, 0.9057381688, -0.0449492774],
    [-0.9045427022, -0.4233928061, -0.0504086470],
]
position_error = math.sqrt(sum(
    (calculated_flange[index][3] - controller_flange_position[index]) ** 2
    for index in range(3)
))
rotation_trace = sum(
    controller_flange_rotation[row][column] * calculated_flange[row][column]
    for row in range(3)
    for column in range(3)
)
orientation_error = math.degrees(math.acos(max(-1.0, min(1.0, (rotation_trace - 1.0) / 2.0))))
require(position_error < 2.0, f"joint/Cartesian position closure regressed: {position_error} mm")
require(orientation_error < 0.1, f"joint/Cartesian orientation closure regressed: {orientation_error} deg")

print(
    "PASS: Inovance read-only kinematics acquisition is fixed below the adaptor; "
    f"field snapshot closes at joint={max_joint_error:.6f} deg, "
    f"position={position_error:.6f} mm, orientation={orientation_error:.6f} deg"
)
