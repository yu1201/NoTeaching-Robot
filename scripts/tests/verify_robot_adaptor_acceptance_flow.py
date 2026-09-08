#!/usr/bin/env python3
"""Static safety contract for the brand-neutral robot adaptor acceptance page."""

from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[2]
SOURCE = (ROOT / "src/FunctionTestDialog.cpp").read_text(encoding="utf-8")
STORE = (ROOT / "include/RobotAdaptorAcceptanceStore.h").read_text(encoding="utf-8")
LAYOUT = (ROOT / "include/RobotAdaptorAcceptanceLayout.h").read_text(encoding="utf-8")
PLAN = (ROOT / "include/RobotAdaptorAcceptancePlan.h").read_text(encoding="utf-8")
MAIN = (ROOT / "src/QtWidgetsApplication4.cpp").read_text(encoding="utf-8")


def fail(message: str) -> None:
    print(f"FAIL: {message}", file=sys.stderr)
    raise SystemExit(1)


for stage in (
    "机器人控制连接测试",
    "FTP下载/同名上传/回读闭环",
    "生成并下发只加载不运行的测试程序",
    "当前位置、关节与完成状态读取",
    "低速直线单步移动并返回原位",
    "常用接口与寄存器写读恢复",
    "工具、运动学与程序资产接口检查",
    "二转三/手眼坐标转换验证",
    "先测后焊扫描流程",
    "实际焊接流程",
    "恢复检查与验收汇总",
):
    if stage not in PLAN:
        fail(f"acceptance stage is missing: {stage}")

for token in (
    'RobotAdaptorAcceptancePlan-20260907-v2',
    'inline constexpr int StageCount = 11',
    'RobotAdaptorAcceptancePlan::StageCount',
    'RobotAdaptorAcceptancePlan::Revision',
):
    if token not in PLAN + SOURCE + STORE:
        fail(f"shared acceptance plan/version binding is missing: {token}")
if '#include "RobotAdaptorAcceptancePlan.h"' not in SOURCE:
    fail("acceptance UI does not consume the shared brand-neutral plan")
if 'reportJson["planRevision"]' not in SOURCE or 'record["PlanRevision"]' not in STORE:
    fail("acceptance persistence/report does not identify the shared plan revision")
if "RobotDriverCapabilityMaxBitIndex" not in SOURCE:
    fail("acceptance UI/report can omit newly added capability bits")

for forbidden in ("FANUCRobotDriver.h", "STEPRobotDriver.h", "InovanceRobotDriver.h"):
    if forbidden in SOURCE:
        fail(f"brand-neutral acceptance page includes a concrete driver: {forbidden}")

layout_body = SOURCE.split("QWidget* FunctionTestDialog::CreateAdaptorAcceptancePage()", 1)[1].split(
    "void FunctionTestDialog::ShowAdaptorAcceptancePage()", 1
)[0]
for token in (
    'new QComboBox(robotTargetGroup)',
    'new QListWidget(navigationPanel)',
    'new QStackedWidget(detailPanel)',
    'QSplitter(Qt::Horizontal, page)',
    '&QListWidget::currentRowChanged',
    'm_pAdaptorAcceptanceStageStack->setCurrentIndex(row)',
    '"开始连接测试"',
    '"生成并下发测试程序"',
    '"打开实际焊接流程"',
):
    if token not in layout_body and token not in SOURCE:
        fail(f"master-detail acceptance UI token is missing: {token}")

change_robot_body = SOURCE.split(
    "void FunctionTestDialog::ChangeAdaptorAcceptanceRobot(int comboIndex)", 1
)[1].split("QString FunctionTestDialog::AdaptorAcceptanceStorageRobotName()", 1)[0]
for token in (
    "m_unitIndex = nextUnitIndex",
    "m_cameraCacheResolver(selectedUnitNo)",
    "LoadAdaptorAcceptanceRun()",
    "SaveAdaptorAcceptanceRun()",
):
    if token not in change_robot_body:
        fail(f"robot selector does not switch the complete acceptance context: {token}")

connection_body = SOURCE.split("void FunctionTestDialog::RunAdaptorConnectionTest()", 1)[1].split(
    "void FunctionTestDialog::RunAdaptorFtpRoundTrip()", 1
)[0]
for token in ("ControlEndpoint()", "Connect()", "IsConnected()"):
    if token not in connection_body:
        fail(f"robot connection test token is missing: {token}")
for forbidden in ("Disconnect()", "TryGetCurrentPos(", "ReadMotionStatus()"):
    if forbidden in connection_body:
        fail(f"robot connection test contains a later-stage or disruptive action: {forbidden}")

program_body = SOURCE.split("void FunctionTestDialog::RunAdaptorProgramDownlink()", 1)[1].split(
    "void FunctionTestDialog::RunAdaptorPositionStatusCheck()", 1
)[0]
if "DownlinkTrajectory" not in program_body:
    fail("program-load stage does not use the adaptor downlink contract")
if "RobotTrajectoryPurpose::WeldDryRun" not in program_body:
    fail("program-load stage does not force brand-native dry-run program generation/upload")
if "driver->StartTrajectory(" in program_body:
    fail("program-load stage must never start the generated program")

ftp_body = SOURCE.split("void FunctionTestDialog::RunAdaptorFtpRoundTrip()", 1)[1].split(
    "void FunctionTestDialog::RunAdaptorProgramDownlink()", 1
)[0]
for token in (
    "选择控制器专用测试JOB",
    "DownloadProgramFile(\n                                sourceRemoteBytes, sourceLocalBytes)",
    "UploadProgramFile(\n                                    sourceLocalBytes, sourceRemoteBytes, false)",
    "DownloadProgramFile(\n                                    sourceRemoteBytes, roundTripLocalBytes)",
    "FileSha256(sourceLocalPath",
    "按原路径同名回传",
    "未执行上传前删除",
    "远端不新增文件、不删除文件",
    "profile.acceptanceProgramExtensions",
    "请先在示教器当前工程内创建并保存一个专用测试JOB",
):
    if token not in ftp_body:
        fail(f"FTP round-trip safety token is missing: {token}")
if "QFileDialog::getOpenFileName" in ftp_body:
    fail("FTP acceptance must use a controller-existing JOB, not require a local JOB")
download_index = ftp_body.find("sourceRemoteBytes, sourceLocalBytes")
upload_index = ftp_body.find("sourceLocalBytes, sourceRemoteBytes, false")
if download_index < 0 or upload_index < 0 or download_index >= upload_index:
    fail("FTP acceptance must download a controller JOB before uploading the test copy")
if "DeleteProgramFile(" in ftp_body:
    fail("FTP acceptance must not delete a controller engineering file")

motion_body = SOURCE.split("void FunctionTestDialog::RunAdaptorSafeLinearMotion(bool jointMotion)", 1)[1].split(
    "void FunctionTestDialog::RunAdaptorInterfaceMatrix()", 1
)[0]
for token in (
    "已握住示教器并准备急停",
    "QMessageBox::Ok | QMessageBox::Cancel",
    "safetyConfirmation != QMessageBox::Ok",
    "RobotDriverCapability::OperationModeControl",
    "RobotDriverCapability::ServoPowerControl",
    "SetOperationMode(RobotOperationMode::Automatic)",
    "driver->ServoOn()",
    "恢复执行前状态",
    "MarkMotionStarted",
    "MoveLinearMmPerMin",
    "CheckRobotDone",
    "MarkMotionCompleted",
    "StopAndConfirmUnverifiedMotion",
    "m_adaptorAcceptanceMotionRoundTripCompleted = ok",
    "RobotAcceptanceAlarmPreparation::Prepare(",
    "driver->Supports(RobotDriverCapability::AlarmReset)",
    "const bool modeAttempted = alarmPreparation.ready && !brandPreparesStream",
    "const bool servoAttempted = modeOk && !brandPreparesStream",
    "const bool setupOk = alarmPreparation.ready && modeOk && servoOk && preparedStatusOk",
    "报警复位后状态：",
    "报警复位接口错误：",
    "安全中止并确认=未执行（尚未发送运动命令）",
):
    if token not in motion_body:
        fail(f"safe motion contract token is missing: {token}")
if "QInputDialog::getText" in motion_body or "确认语句不匹配" in motion_body:
    fail("safe motion confirmation must use buttons and must not require typed text")

combination_body = SOURCE.split("void FunctionTestDialog::RunAdaptorModeCombinationTests(bool allCases)", 1)[1].split(
    "void FunctionTestDialog::RunAdaptorSafeLinearMotion(bool jointMotion)", 1
)[0]
for token in ("RunModePreparationTestCase(item.id, result)", "canContinue = result.restoreVerified",
              "Qt::BlockingQueuedConnection", "if (!startSaved)", "if (!saved)",
              "RobotOperationLease::TryAcquire", "RobotOperationLease::IsCancellationRequested",
              "m_adaptorAcceptanceMotionRoundTripCompleted = false", '"pending" : "fail"',
              'm_adaptorModeCombinationEvidence', 'QMessageBox::Ok | QMessageBox::Cancel'):
    if token not in combination_body:
        fail(f"non-motion batch test safety/persistence contract missing: {token}")
for forbidden in ("MoveLinearMmPerMin", "StartTrajectory(", "UseVerifiedModePreparation(",
                  "MarkMotionStarted(", 'm_adaptorAcceptanceStates[4] = "pass"'):
    if forbidden in combination_body:
        fail(f"batch must not move, auto-select a recipe, or approve displacement: {forbidden}")
for token in ('record["ModeCombinationEvidence"]', 'record.value("ModeCombinationEvidence")',
              'reportJson["modeCombinationEvidence"]', '一次测试全部组合', '选用已通过组合', '停止组合测试'):
    if token not in SOURCE + LAYOUT:
        fail(f"batch UI/history/report binding missing: {token}")
for token in ('RobotAdaptorAcceptanceLayout::CreateModeControls(motionPage)',
              'motionPageLayout->insertWidget', 'RobotAdaptorAcceptanceLayout::WrapDetailPanel(stageSplitter, detailPanel)',
              'RobotAdaptorAcceptanceLayout::PopulateModeOptions', 'stageSplitter->addWidget(detailScroll)'):
    if token not in SOURCE:
        fail(f"scrollable/full-width mode controls are not wired to the actual acceptance page: {token}")
for token in ('"_keep_wide_control", true', 'index / 2, index % 2', 'setMinimumHeight(40)',
              'ConfigureResponsiveScrollArea(scroll)', 'combo->currentIndex() < 0'):
    if token not in LAYOUT:
        fail(f"mode-control no-clipping/initial-selection rule missing: {token}")

mark_body = SOURCE.split("void FunctionTestDialog::MarkAdaptorAcceptanceStage(", 1)[1].split(
    "void FunctionTestDialog::FinishAdaptorAcceptanceStage(", 1
)[0]
if "!m_adaptorAcceptanceMotionRoundTripCompleted" not in mark_body:
    fail("stage 4 can be marked passed without a completed return-to-origin round trip")
if "不能人工标记通过" not in mark_body:
    fail("missing adaptor capabilities can be manually overridden")

interface_body = SOURCE.split("void FunctionTestDialog::RunAdaptorInterfaceMatrix()", 1)[1].split(
    "void FunctionTestDialog::RunAdaptorAssetAudit()", 1
)[0]
for token in (
    "RobotRegisterRoundTrip::Run<int>",
    "RobotRegisterRoundTrip::Run<double>",
    "driver->TryGetIntVar", "driver->TryGetRealVar", "driver->SetIntVar", "driver->SetRealVar",
    "ok = result.Passed()", "actual == expected", "std::abs(actual - expected) <= 1e-6",
    "REAL寄存器：前序检查或INT测试失败，未执行。",
    "INT寄存器：前序状态检查失败，未执行。",
):
    if token not in interface_body:
        fail(f"register backup/readback/restore token is missing: {token}")
for token in ("为避免覆盖未知原值，未执行任何写入", "RobotRegisterRoundTrip::StatusText",
              "result.read.ok ? format(result.readback)", "result.restoreRead.ok ? format(result.restored)",
              "临时写入", "恢复回读", "step.second->error"):
    if token not in SOURCE:
        fail(f"register evidence must distinguish unattempted operations and preserve step errors: {token}")
for forbidden in ("AcqPermit", "EnsureControlPermit", "SendDiagnosticCommand"):
    if forbidden in interface_body:
        fail(f"brand-specific permission leaked into acceptance business code: {forbidden}")

for token in ('CheckAdaptorRegisterRecovery()', 'AcceptanceRegisterName(false, intIndex)',
              'AcceptanceRegisterName(true, realIndex)', 'ConfirmAdaptorRegisterValue(driver,',
              'Qt::BlockingQueuedConnection', '原值数据库保存失败',
              'result.restore.ok && result.restoreRead.ok && result.originalMatches',
              'FinishAdaptorAcceptanceStage(5, ok, result, false)',
              'RobotMotionState::Idle', 'motion.terminalVerified'):
    if token not in interface_body:
        fail(f"manual register handshake/restore gate missing: {token}")
if 'state == "pass" && stage == 5' not in mark_body:
    fail('manual pass can bypass stage 5 acknowledgement and readback')

asset_body = SOURCE.split("void FunctionTestDialog::RunAdaptorAssetAudit()", 1)[1].split(
    "void FunctionTestDialog::RunAdaptorTwoToThreeCheck()", 1
)[0]
for token in (
    "RobotDriverCapability::ControllerKinematicsRead",
    "RefreshKinematicsFromController(validation)",
    "关节/直角闭环误差",
    "控制器机械数据已由品牌底层按固定来源获取并交叉校验",
    "控制器程序资产清单（有限递归）",
):
    if token not in asset_body:
        fail(f"controller asset acquisition evidence missing: {token}")
inovance_header = (ROOT / 'include/InovanceRobotDriver.h').read_text(encoding='utf-8')
fanuc_header = (ROOT / 'include/FANUCRobotDriver.h').read_text(encoding='utf-8')
if 'real ? "D[" : "R["' not in inovance_header or 'AcceptanceRegisterName' not in fanuc_header:
    fail('register aliases must be translated by the brand driver, not the acceptance UI')
for token in ('record["RegisterRecovery"]', 'record.value("RegisterRecovery")',
              '现场数值一致，回读校验', '不一致/取消并恢复', 'timeout.start(5 * 60 * 1000)',
              'for (auto it = history.cbegin()', '先处理未恢复变量'):
    if token not in SOURCE:
        fail(f"register on-site confirmation/restart recovery missing: {token}")

for token in (
    'ConfigLocation::Robot(robotName, "RobotAdaptorAcceptance")',
    'OpenAdaptorWorkflow("measureThenWeldScan")',
    'OpenAdaptorWorkflow("measureThenWeldActual")',
    "MeasureThenWeldCapabilityPolicy::WeldMask<RobotDriverCapability>(true)",
):
    if token not in SOURCE:
        fail(f"acceptance persistence/workflow gate is missing: {token}")

if MAIN.count("OpenRobotAdaptorAcceptanceWorkflow(workflowId, unitIndex)") < 2:
    fail("all FunctionTestDialog construction paths must wire the workflow launcher")
if MAIN.count("ScanCameraCacheForUnit(unitIndex)") < 2:
    fail("all FunctionTestDialog construction paths must wire robot-specific camera caches")
workflow_launcher_body = MAIN.split(
    "bool QtWidgetsApplication4::OpenRobotAdaptorAcceptanceWorkflow", 1
)[1].split("void QtWidgetsApplication4::OpenMeasureThenWeldDialog", 1)[0]
for token in (
    "findData(unitIndex)",
    "setCurrentIndex(comboIndex)",
    "CurrentRobotUnitIndex() != unitIndex",
    "OpenMeasureThenWeldDialog();",
    "m_nMeasureThenWeldPageUnitIndex == unitIndex",
):
    if token not in workflow_launcher_body:
        fail(f"acceptance workflow is not bound to the selected robot: {token}")
if "OpenMeasureThenWeldDialog();" not in workflow_launcher_body:
    fail("acceptance workflow launcher is not connected to the existing measure-then-weld page")

for token in (
    'createManagementAction("机器人适配测试"',
    'addCommandAction(debugMenu, "机器人适配测试"',
    "OpenRobotAdaptorAcceptanceDialog()",
    "ShowAdaptorAcceptancePage()",
    "OpenFunctionTestPage(true)",
):
    if token not in MAIN:
        fail(f"debug robot-adaptor-test entry is missing: {token}")

show_page_body = SOURCE.split("void FunctionTestDialog::ShowAdaptorAcceptancePage()", 1)[1].split(
    "void FunctionTestDialog::ShowSingleTestPage()", 1
)[0]
for token in ("tabBar()->hide()", "机器人适配测试 — %1", "任意阶段均可导出报告"):
    if token not in show_page_body:
        fail(f"standalone adaptor-test presentation token is missing: {token}")

report_body = SOURCE.split("void FunctionTestDialog::FinalizeAdaptorAcceptance()", 1)[1].split(
    "bool FunctionTestDialog::RunDashboardTool", 1
)[0]
for token in (
    "RobotAdaptorAcceptanceReportV1",
    "Result/RobotAdaptorAcceptance/%1",
    "QSaveFile markdownFile",
    "QSaveFile jsonFile",
    'reportJson["stages"]',
    'reportJson["recommendations"]',
    'QStringLiteral("restricted")',
    'QStringLiteral("incomplete")',
):
    if token not in report_body:
        fail(f"actionable acceptance report token is missing: {token}")
if "AdaptorAcceptancePrerequisitesReady(10" in report_body:
    fail("report export must remain available before all stages pass")
if "m_adaptorAcceptanceEvidence[selectedStage]" not in report_body:
    fail("report export can overwrite the evidence editor with another stage's evidence")

for token in ('ReplaceScopedModuleSectionsAtomically', 'QStringLiteral("RobotAdaptorAcceptanceV1")',
              'legacyBooleanSchema', '未自动恢复任何机器人运动'):
    if token not in STORE:
        fail(f"acceptance database recovery contract is missing: {token}")
for token in ('m_pAdaptorAcceptanceRunCombo', '&QCoreApplication::aboutToQuit',
              'm_pAdaptorAcceptanceSaveTimer->start()', 'if (!SaveAdaptorAcceptanceRun())',
              'record.value("SelectedStage")', 'BeginAdaptorAcceptanceStage(4)'):
    if token not in SOURCE:
        fail(f"acceptance autosave/restart contract is missing: {token}")
if 'setCurrentRow(0)' in change_robot_body:
    fail("switching robots must restore the recorded stage, not reset it to stage zero")

for token in ('RobotAcceptanceJointMotion::OutwardTarget', 'RobotAcceptanceJointMotion::Matches',
              'driver->MoveJointPercent(jointTarget, speed, externalAxisType)',
              'jointStartOk', 'jointReadbackOk', 'restoredReadbackOk',
              'm_adaptorJointLease = ok && !returning ? lease : nullptr',
              'm_adaptorJointMotionState = !ok ? "fail" : (returning ? "pass" : "awaiting_return")',
              'jointMotion && driver->ExternalAxleType() != 0'):
    if token not in motion_body:
        fail(f"joint round-trip must prove start/target/restoration and hold its own lease: {token}")
if motion_body.index('RobotOperationLease::TryAcquire') > motion_body.index('driver->TryGetCurrentPulse(jointStart)'):
    fail('joint origin capture must be protected by the operation lease')
joint_finish = motion_body.split('if (jointMotion)\n                    {', 1)[1].split('return;', 1)[0]
if 'FinishAdaptorAcceptanceStage(4' in joint_finish or 'm_adaptorAcceptanceStates[4]' in joint_finish:
    fail('joint result must never overwrite the existing linear result')
for token in ('record["JointMotionState"]', 'record["JointMotionEvidence"]',
              'reportJson["jointMotionState"]', 'reportJson["jointMotionEvidence"]',
              'm_pAdaptorJointCancelBtn', '结束关节专项（不自动返回）',
              'm_adaptorEvidencePlanRevision'):
    if token not in SOURCE:
        fail(f'joint history/report/cancel contract missing: {token}')
for token in ('!record.contains("JointMotionState")', 'record["JointMotionState"] = "pending"',
              'record.value("JointMotionState") == "running"',
              'record.value("JointMotionState") == "awaiting_return"',
              'record["JointMotionState"] = "fail"'):
    if token not in STORE:
        fail(f'legacy/interrupted joint proof must fail closed: {token}')

print("PASS: robot adaptor acceptance flow is staged, brand-neutral, recoverable, and fail-closed")
