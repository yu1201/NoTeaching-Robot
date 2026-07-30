from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def main() -> int:
    dialog_h = (ROOT / "include/LaserWeldFilterDialog.h").read_text(encoding="utf-8")
    dialog_cpp = (ROOT / "src/LaserWeldFilterDialog.cpp").read_text(encoding="utf-8")
    extraction_cpp = (ROOT / "src/PointCloudExtractionProcessor.cpp").read_text(encoding="utf-8")

    require("m_pCloudPixelNumSpin" in dialog_h, "pixel_num editor is missing from dialog state")
    for token in (
        'new QLabel("骨架毛刺长度")',
        'ReadPlainIniIntValue(configPath, "pixel_num", 10)',
        '{ "pixel_num", QString::number(m_pCloudPixelNumSpin->value()) }',
        "SDK 20260729 版新增",
    ):
        require(token in dialog_cpp, f"pixel_num UI persistence is incomplete: {token}")

    require('ConfigLineValue(content, "pixel_num").isEmpty()' in extraction_cpp,
            "historical SDK configs do not receive a runtime pixel_num fallback")
    require('ReplaceConfigValue(&content, "pixel_num", "10")' in extraction_cpp,
            "runtime pixel_num fallback is not the UI default")

    print("PASS: SDK pixel_num is exposed, persisted, and injected for historical configs")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
