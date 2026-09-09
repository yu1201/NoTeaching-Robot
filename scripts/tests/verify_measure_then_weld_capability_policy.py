"""Offline integration guard: route capabilities must be checked before motion."""

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[2]


class MeasureThenWeldCapabilityIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = (ROOT / "src/QtWidgetsApplication4.cpp").read_text(encoding="utf-8-sig")
        cls.service = (ROOT / "src/MeasureThenWeldService.cpp").read_text(encoding="utf-8-sig")

    def test_all_page_open_paths_use_shared_entry_gate(self):
        quick = self.app.split('quickMeasureBtn->setProperty("requiredRobotCapabilities"', 1)[1].split(
            "setRequiredCapabilities(quickJogBtn", 1
        )[0]
        self.assertIn("MeasureThenWeldCapabilityPolicy::EntryMask<RobotDriverCapability>()", quick)
        self.assertNotIn("JointMotion", quick)
        opening = self.app.split("void QtWidgetsApplication4::OpenMeasureThenWeldDialog()", 1)[1].split(
            "void QtWidgetsApplication4::OpenPreciseMeasureEditDialog()", 1
        )[0]
        self.assertLess(opening.index("SupportsMask(entryMask)"), opening.index("new MeasureThenWeldDialog"))

    def test_scan_preflight_checks_configured_route_before_any_motion(self):
        scan = self.service.split("bool MeasureThenWeldService::RunScanCycle(", 1)[1].split(
            "bool MeasureThenWeldService::ScanMoveAndCollect(", 1
        )[0]
        self.assertIn("ScanMask<RobotDriverCapability>(\n        param.bUseComputedScanSafe, scanTrajectory != nullptr)", scan)
        self.assertLess(scan.index("RequireRobotCapabilityMask("), scan.index("MoveScanStartSafeAndWait("))
        self.assertLess(scan.index("ValidateWristAxisUnits("), scan.index("MoveScanStartSafeAndWait("))
        self.assertIn("禁止静默改用自动计算安全位", scan)

    def test_wrist_conversion_failure_cannot_reach_computed_motion(self):
        start = self.service.split("bool MeasureThenWeldService::MoveScanStartSafeAndWait(", 1)[1].split(
            "bool MeasureThenWeldService::MoveScanEndSafeAndWait(", 1
        )[0]
        check = start.split("if (!TryMaxWristDeltaDeg(", 1)[1].split("const double warnThresholdDeg", 1)[0]
        self.assertIn("return false;", check)
        self.assertLess(start.index("if (!TryMaxWristDeltaDeg("), start.index("MoveCoorsAndWait("))
        self.assertLess(start.index("TryGetCurrentPulse(currentPulse)"), start.index("if (param.bHasStartPulse)"))
        self.assertLess(start.index("if (!TryMaxWristDeltaDeg("), start.index("if (param.bHasStartPulse)"))
        self.assertIn("if (!param.bUseComputedScanSafe)", start)
        self.assertIn('param.vtStartSafePulse, speed, "下枪安全姿态"', start)
        self.assertNotIn("double PulseDeltaDeg(", self.service)


if __name__ == "__main__":
    unittest.main(verbosity=2)
