#!/usr/bin/env python3
from __future__ import annotations

import importlib.util
import sqlite3
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
MIGRATE_PATH = ROOT / "tools" / "migrate_config_to_sqlite.py"


def load_migrate_module():
    spec = importlib.util.spec_from_file_location("config_migrate_under_test", MIGRATE_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def value(module, connection: sqlite3.Connection, section: str, key: str) -> str | None:
    return module.read_ini_value(
        connection,
        "Data/RobotA/WeldSeamCompParam.ini",
        section,
        key,
    )


def test_fresh_defaults(module) -> None:
    connection = sqlite3.connect(":memory:")
    module.init_schema(connection, False)
    inserted = module.ensure_seam_comp_defaults(connection, ["RobotA"], False)
    assert inserted > 0
    assert value(module, connection, "ALLWeldSeamComp", "SeamCompSchemaVersion") == "2"
    assert value(module, connection, "ALLWeldSeamComp", "SeamCompCount") == "1"
    assert value(module, connection, "ALLWeldSeamComp", "SeamCompGroupCount") == "1"
    assert value(module, connection, "WeldSeamCompGroup0", "WeldZComp") == "0.000000"
    assert value(module, connection, "WeldSeamCompGroup0", "WeldGunDirComp") == "0.000000"
    assert value(module, connection, "WeldSeamCompGroup0", "WeldSeamDirComp") == "0.000000"
    assert value(module, connection, "WeldSeamCompGroup0", "SegmentKind") is None
    assert value(module, connection, "WeldSeamComp0", "SegmentKind") is None


def test_legacy_defaults_are_not_shadowed(module) -> None:
    connection = sqlite3.connect(":memory:")
    module.init_schema(connection, False)
    path = "Data/RobotA/WeldSeamCompParam.ini"
    module.insert_ini_value(connection, path, "ALLWeldSeamComp", "SeamCompCount", "4", False)
    module.insert_ini_value(connection, path, "ALLWeldSeamComp", "SeamCompGroupCount", "1", False)
    module.insert_ini_value(connection, path, "WeldSeamComp0", "SegmentKind", "CorrugatedPlate", False)
    module.insert_ini_value(connection, path, "WeldSeamComp0", "WeldZComp", "2.000000", False)
    module.insert_ini_value(connection, path, "WeldSeamComp0", "WeldGunDirComp", "3.000000", False)
    module.insert_ini_value(connection, path, "WeldSeamComp0", "WeldSeamDirComp", "1.000000", False)

    assert module.ensure_seam_comp_defaults(connection, ["RobotA"], False) == 0
    assert value(module, connection, "ALLWeldSeamComp", "SeamCompSchemaVersion") is None
    assert value(module, connection, "WeldSeamComp0", "SegmentKind") == "CorrugatedPlate"
    assert value(module, connection, "WeldSeamComp0", "WeldZComp") == "2.000000"
    assert value(module, connection, "WeldSeamCompGroup0", "WeldZComp") is None


def test_countless_legacy_slot_is_not_shadowed(module) -> None:
    connection = sqlite3.connect(":memory:")
    module.init_schema(connection, False)
    path = "Data/RobotA/WeldSeamCompParam.ini"
    module.insert_ini_value(connection, path, "WeldSeamComp0", "SegmentKind", "CorrugatedPlate", False)
    module.insert_ini_value(connection, path, "WeldSeamComp0", "WeldZComp", "9.000000", False)

    assert module.ensure_seam_comp_defaults(connection, ["RobotA"], False) == 0
    assert value(module, connection, "ALLWeldSeamComp", "SeamCompSchemaVersion") is None
    assert value(module, connection, "WeldSeamComp0", "WeldZComp") == "9.000000"
    assert value(module, connection, "WeldSeamCompGroup0", "WeldZComp") is None


def test_runtime_source_contract() -> None:
    service = (ROOT / "src" / "MeasureThenWeldService.cpp").read_text(encoding="utf-8")
    header = (ROOT / "include" / "MeasureThenWeldService.h").read_text(encoding="utf-8")
    dialog = (ROOT / "src" / "WeldSeamCompDialog.cpp").read_text(encoding="utf-8")
    config = (ROOT / "src" / "WeldSeamCompConfig.cpp").read_text(encoding="utf-8")
    average_updater = (ROOT / "src" / "WeldPoseAverageUpdater.cpp").read_text(encoding="utf-8")

    for removed in (
        "FindSeamCompSlotByKind",
        "FindSeamCompSlotForRecord",
        "seamCompSlots",
        "seamSegmentKind",
    ):
        assert removed not in service
        assert removed not in header
    assert "const WeldPosePreset::SeamCompValues& seamComp = preset.seamComp;" in service
    assert "record.point.z() += seamComp.weldZComp;" in service
    assert "m_pTypeCombo->setVisible(poseMode);" in dialog
    assert "m_seamRows.push_back(MakeDefaultSeamRow());" in dialog
    assert "ConfigDatabase::ReadIniFileSnapshot(path, snapshot, &error)" in config
    assert "COPini" not in config
    assert "WeldSeamPoseAverage" not in average_updater
    assert "WeldSeamCompParam" not in average_updater


def main() -> None:
    module = load_migrate_module()
    test_fresh_defaults(module)
    test_legacy_defaults_are_not_shadowed(module)
    test_countless_legacy_slot_is_not_shadowed(module)
    test_runtime_source_contract()
    print("uniform weld seam compensation tests: PASS")


if __name__ == "__main__":
    main()
