#!/usr/bin/env python3
from __future__ import annotations

import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def main() -> None:
    tracked_ini = subprocess.check_output(
        ["git", "ls-files", "*.ini"], cwd=ROOT, text=True, encoding="utf-8"
    ).splitlines()
    require(
        tracked_ini == ["SDK/PointCloudExtration/config/CorrugatedSheetPointCloudEctration.ini"],
        f"unexpected tracked INI configuration: {tracked_ini}",
    )

    database_header = read("include/ConfigDatabase.h")
    database_source = read("src/ConfigDatabase.cpp")
    combined_database = database_header + database_source
    for removed in (
        "NormalizeFilePath",
        "HasIniFile",
        "ReadIniValue",
        "WriteIniValue",
        "CopyIniFile",
        "ReadIniSection",
        "ReadIniFileSnapshot",
        "ReplaceIniSectionsAtomically",
        "HasTextFile",
        "ReadTextFile",
        "WriteTextFile",
        "RemoveConfigPathPrefix",
        "BuildScopedFileIdentity",
        "BuildScopedIniIdentity",
    ):
        require(removed not in combined_database, f"path compatibility API remains: {removed}")

    require(not (ROOT / "include" / "OPini.h").exists(), "legacy OPini header remains")
    require(not (ROOT / "src" / "OPini.cpp").exists(), "legacy OPini source remains")
    project = read("QtWidgetsApplication4.vcxproj") + read("QtWidgetsApplication4.vcxproj.filters")
    require("OPini" not in project, "project still compiles the legacy path adapter")
    require("ConfigSection" in project, "database-native ConfigSection is not compiled")
    for token in (
        "IsDatabaseNativeIdentityPart",
        "HasLegacyConfigFileSuffix",
        "value.contains(QLatin1Char('\\\\'))",
        "value.contains(QLatin1Char(':'))",
    ):
        require(token in database_source, f"database-native identity gate missing: {token}")

    runtime_contracts = {
        "src/BrandingConfig.cpp": ("QSettings", "branding.ini"),
        "src/FANUCRobotDriver.cpp": ("robot.ini", "FanucReadWinOlpcOutputDir"),
        "src/FunctionTestDialog.cpp": ("QSettings", "KinematicsModelCandidate_"),
        "src/WeldProcessFile.cpp": (
            "WeldPara.txt",
            "WeaveDate.txt",
            "SaveWeldTxt",
            "SaveWeaveTxt",
            "ReadTextFile",
            "WriteTextFile",
        ),
        "tools/camera_fps_probe.ps1": (
            "Get-IniValue",
            "ContralUnitInfo.ini",
            "CameraParam.ini",
        ),
    }
    for relative, forbidden in runtime_contracts.items():
        content = read(relative)
        for token in forbidden:
            require(token not in content, f"{relative} still contains path configuration token: {token}")

    pointcloud_source = "\n".join(
        read(relative)
        for relative in (
            "src/LaserWeldFilterDialog.cpp",
            "src/PointCloudExtractionProcessor.cpp",
            "src/PointCloudProcessingConfig.cpp",
        )
    )
    require(
        "CorrugatedSheetPointCloudEctration.ini" in pointcloud_source
        and "CorrugatedSheetPointCloudEctration.runtime.ini" in pointcloud_source,
        "point-cloud algorithm INI read/runtime-copy contract was removed",
    )

    print("PASS: all non-point-cloud runtime configuration is database-native")


if __name__ == "__main__":
    main()
