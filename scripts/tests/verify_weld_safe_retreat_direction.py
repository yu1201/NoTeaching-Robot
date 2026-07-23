from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def main() -> int:
    const_h = read("include/Const.h")
    param_h = read("include/MeasureThenWeldDialog.h")
    service = read("src/MeasureThenWeldService.cpp")
    editor = read("src/PreciseMeasureEditDialog.cpp")
    defaults = read("src/RobotDataHelper.cpp")
    migration = read("tools/migrate_config_to_sqlite.py")

    expected_modes = {
        "WELD_SAFE_RETREAT_AUTO_LEGACY_X_NEGATIVE": 0,
        "WELD_SAFE_RETREAT_WORLD_X_NEGATIVE": 1,
        "WELD_SAFE_RETREAT_WORLD_X_POSITIVE": 2,
        "WELD_SAFE_RETREAT_WORLD_Y_NEGATIVE": 3,
        "WELD_SAFE_RETREAT_WORLD_Y_POSITIVE": 4,
    }
    for name, value in expected_modes.items():
        require(f"{name} = {value}" in const_h, f"missing stable mode {name}={value}")
    require(
        "nWeldSafeRetreatDirection = WELD_SAFE_RETREAT_AUTO_LEGACY_X_NEGATIVE" in param_h,
        "old parameter databases do not default to the legacy X-negative behavior",
    )

    for token in (
        'ReadString(false, "WeldSafeRetreatDirection", &param.nWeldSafeRetreatDirection)',
        "NormalizeWeldSafeRetreatDirectionMode(param.nWeldSafeRetreatDirection)",
        'addInt("safeRetreatDirection", param.nWeldSafeRetreatDirection)',
        "-Eigen::Vector3d::UnitX()",
        "Eigen::Vector3d::UnitX()",
        "-Eigen::Vector3d::UnitY()",
        "Eigen::Vector3d::UnitY()",
        "param.nWeldSafeRetreatDirection,\n        weldPosePreset.robotType",
        "水平回撤方向=%2",
    ):
        require(token in service, f"runtime retreat-direction wiring missing: {token}")
    require(service.count("param.nWeldSafeRetreatDirection,\n        weldPosePreset.robotType") == 2,
            "start and end safe positions are not both using the configured direction")
    require("兼容旧现场：自动模式继续从世界 X- 侧优先回撤" in service,
            "legacy automatic behavior was not retained explicitly")

    for label in ("自动（兼容旧算法，X-优先）", "世界 X-", "世界 X+", "世界 Y-", "世界 Y+"):
        require(label in editor, f"UI direction choice missing: {label}")
    require('WELD_SAFE_RETREAT_DIRECTION_KEY = "WeldSafeRetreatDirection"' in editor,
            "UI config key is missing")
    require("收下枪安全距必须为大于 0 mm 的有限数值" in editor,
            "non-positive safe distance is not rejected at save time")
    require("new QDoubleValidator(0.000001, 1.0e12, 6, edit)" in editor,
            "safe-distance editor does not constrain input to positive values")

    for text in (defaults, migration, editor):
        require("GunDownBackSafeDis" in text, "safe-distance default is missing")
        require("WeldSafeRetreatDirection" in text, "retreat-direction default is missing")

    print("PASS: weld safe retreat direction is configurable and backward compatible")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
