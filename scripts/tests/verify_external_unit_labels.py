#!/usr/bin/env python3
"""Static guard: numeric editor units must be rendered outside edit controls."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def main() -> int:
    sources = "\n".join(
        path.read_text(encoding="utf-8")
        for directory in (ROOT / "src", ROOT / "include")
        for path in directory.rglob("*")
        if path.suffix in {".cpp", ".h"}
    )
    assert "setSuffix(" not in sources
    assert "setPrefix(" not in sources
    assert 'setPlaceholderText(QStringLiteral("输入mm"))' not in sources

    helper = (ROOT / "src" / "WindowStyleHelper.cpp").read_text(encoding="utf-8")
    assert "QWidget* CreateExternalUnitEditor(" in helper
    for relative in (
        "src/ModelAlignmentDialog.cpp",
        "src/ProcessLoopTestDialog.cpp",
        "src/WorkpieceMeshViewerDialog.cpp",
        "src/FunctionTestDialog.cpp",
        "src/ModelWeldingFlowDialog.cpp",
        "src/WeldSeamCompDialog.cpp",
        "src/VirtualWeldTestDialog.cpp",
    ):
        assert "CreateExternalUnitEditor(" in (ROOT / relative).read_text(encoding="utf-8")

    print("EXTERNAL_UNIT_LABELS_OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
