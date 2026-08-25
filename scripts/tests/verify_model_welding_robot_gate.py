"""Static guard for the explicit robot-model eligibility gate."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def section(text: str, start: str, end: str | None = None) -> str:
    start_index = text.index(start)
    end_index = len(text) if end is None else text.index(end, start_index + len(start))
    return text[start_index:end_index]


dialog = read("src/MeasureThenWeldDialog.cpp")
header = read("include/MeasureThenWeldDialog.h")
robot_data = read("include/RobotDataHelper.h")
catalog = read("include/RobotModelCatalogStore.h")
flow_dialog = read("src/ModelWeldingFlowDialog.cpp")
flow_header = read("include/ModelWeldingFlowDialog.h")
workflow_header = read("include/ModelWeldingWorkflow.h")

load = section(
    dialog,
    "void MeasureThenWeldDialog::LoadRobotList()",
    "void MeasureThenWeldDialog::LoadParamGroups()",
)
changed = section(
    dialog,
    "void MeasureThenWeldDialog::OnRobotChanged(int index)",
    "void MeasureThenWeldDialog::OnParamGroupChanged(int index)",
)
resolve = section(
    dialog,
    "bool MeasureThenWeldDialog::ResolveModelWeldingAvailabilityForRow(",
    "void MeasureThenWeldDialog::RefreshModelWeldingAvailability()",
)
refresh = section(
    dialog,
    "void MeasureThenWeldDialog::RefreshModelWeldingAvailability()",
    "int MeasureThenWeldDialog::CurrentParamGroupIndex()",
)
open_flow = section(
    dialog,
    "void MeasureThenWeldDialog::OpenModelWeldingFlowDialog()",
    "void MeasureThenWeldDialog::RunPresetParamFlow()",
)
set_running = section(dialog, "void MeasureThenWeldDialog::SetRunning(bool running)")

assert "QString robotModelId;" in robot_data and "int robotType = -1;" in robot_data
assert "ResolveModelEligibility(" in catalog
assert "info.robotModelId" in load and "info.robotType" in load
assert "RefreshModelWeldingAvailability();" in load
assert "RefreshModelWeldingAvailability();" in changed
assert "RefreshModelWeldingAvailability();" in set_running

for token in (
    "unitIndex < 0",
    "RobotDataHelper::GetRobotDriver",
    "driver == nullptr",
    "configuredRobotType != driver->RobotType()",
    "modelId.isEmpty()",
    "ResolveModelEligibility(",
    "modelId, driver->RobotType()",
):
    assert token in resolve, f"missing fail-closed robot-model gate token: {token}"

assert "ResolveRobotEligibility" not in dialog
assert "ROBOT_MODEL_ID_ROLE" in resolve
assert 'QStringLiteral("%1 — 型号: %2 / %3")' in refresh
assert 'QStringLiteral("已适配")' in refresh and 'QStringLiteral("未适配")' in refresh
assert "RobotDataHelper::LoadRobotList(m_pContralUnit)" in refresh
assert "fresh->robotModelId" in refresh and "fresh->robotType" in refresh
assert "setEnabled(!m_bRunning && eligible)" in refresh
assert "m_pModelWeldingFlowBtn->setEnabled(!running)" not in set_running

assert "ResolveModelWeldingAvailabilityForRow(" in open_flow
assert open_flow.index("RefreshModelWeldingAvailability();") < open_flow.index(
    "ResolveModelWeldingAvailabilityForRow("
)
assert open_flow.index("ResolveModelWeldingAvailabilityForRow(") < open_flow.index(
    "ModelWeldingFlowDialog dialog("
)
for token in (
    "ActiveModelWeldingFlowDialog()",
    "if (!activeDialog.isNull())",
    "activeDialog->raise();",
    "activeDialog->activateWindow();",
    "activeDialog = &dialog;",
    "activeDialog.clear();",
    "不能重复打开",
):
    assert token in open_flow, f"model welding dialog single-window guard missing: {token}"
assert open_flow.index("if (!activeDialog.isNull())") < open_flow.index(
    "ModelWeldingFlowDialog dialog("
)

for forbidden in (
    "RobotType == ROBOT_TYPE_STEP",
    "robotName.contains",
    "RobotB",
    "SA10",
):
    assert forbidden not in resolve, f"robot model must not be guessed from identity: {forbidden}"

assert "ResolveModelWeldingAvailabilityForRow" in header
assert "RefreshModelWeldingAvailability" in header

flow_load_robots = section(
    flow_dialog,
    "void ModelWeldingFlowDialog::LoadRobots()",
    "void ModelWeldingFlowDialog::LoadModels()",
)
flow_load_model = section(
    flow_dialog,
    "void ModelWeldingFlowDialog::LoadCurrentRobotCatalogModel(",
    "void ModelWeldingFlowDialog::RefreshTheoreticalRobotStatus(",
)
flow_ready = section(
    flow_dialog,
    "void ModelWeldingFlowDialog::CheckProductionReadiness()",
    "QString ModelWeldingFlowDialog::CurrentRobotName() const",
)
flow_load_teaching = section(
    flow_dialog,
    "void ModelWeldingFlowDialog::LoadTeachingForCurrentRobot()",
    "void ModelWeldingFlowDialog::CreateDraftTemplate()",
)
flow_save_teaching = section(
    flow_dialog,
    "void ModelWeldingFlowDialog::SaveTeaching()",
    "void ModelWeldingFlowDialog::BindRuntimeIdentity()",
)

for token in (
    "RobotDataHelper::GetRobotDriver",
    "robot.unitIndex < 0",
    "robot.robotType != driver->RobotType()",
    "robot.robotModelId",
    "ResolveModelEligibility(",
    "eligibility.eligible",
    'QStringLiteral("%1 — 型号: %2 (%3)")',
):
    assert token in flow_load_robots, f"flow robot list is not eligibility-only: {token}"

assert "m_robotCombo->setCurrentIndex(-1);" in flow_load_robots
assert "initialIndex < 0" in flow_load_robots
assert "不会自动切换到列表中的其他机器人" in flow_load_robots

assert flow_load_model.index("ClearTheoreticalRobotModel();") < flow_load_model.index(
    "ResolveModelEligibility("
)
for token in (
    "CurrentRobotModelId()",
    "CurrentConfiguredRobotType()",
    "driver->RobotType()",
    "eligibility.collisionEnvelope",
    "eligibility.model.sourceStep.sha256",
    "eligibility.model.collision.profileKeySha256",
    "m_theoreticalRobotModelId",
    "m_theoreticalRobotSha256",
    "m_theoreticalRobotProfileKeySha256",
):
    assert token in flow_load_model, f"flow model identity gate missing: {token}"

for token in (
    "RobotDataHelper::LoadRobotList(m_contralUnit)",
    "currentRobot->robotModelId",
    "currentRobot->robotType",
    "ResolveModelEligibility(",
    "m_theoreticalRobotModelId != currentRobotModelId",
    "m_theoreticalRobotSha256 != resolvedSourceSha256",
    "m_theoreticalRobotProfileKeySha256 != resolvedProfileKeySha256",
):
    assert token in flow_ready, f"production model revalidation missing: {token}"

for field in (
    "QString robotModelId;",
    "QString sourceStepSha256;",
    "QString collisionProfileSha256;",
):
    assert field in workflow_header, f"teaching identity field missing: {field}"

for token in (
    "value.robotModelId == currentRobotModelId",
    "value.sourceStepSha256 == currentSourceStepSha256",
    "value.collisionProfileSha256 == currentCollisionProfileSha256",
    "scan.startTaught = false",
    "scan.endTaught = false",
    "scan.startPulseTaught = false",
    "旧记录不会自动通过生产检查",
):
    assert token in flow_load_teaching, f"teaching load is not fail-closed: {token}"

for token in (
    "m_teaching.robotModelId = currentRobotModelId",
    "m_teaching.sourceStepSha256 = currentSourceStepSha256",
    "m_teaching.collisionProfileSha256 = currentCollisionProfileSha256",
):
    assert token in flow_save_teaching, f"teaching save identity missing: {token}"

for token in (
    "m_teaching.robotModelId != robotEligibility.model.modelId",
    "m_teaching.sourceStepSha256",
    "m_teaching.collisionProfileSha256",
):
    assert token in flow_ready, f"production teaching identity comparison missing: {token}"

for forbidden in (
    "ImportTheoreticalRobotModel",
    "LoadActiveTheoreticalRobotModel",
    "TheoreticalRobotModelStore",
    "RobotCadAssemblyLoader::LoadFile",
    "RobotCollisionEnvelopeStore::Generate",
    "导入总装并生成简模",
    "缺失时会自动解析",
):
    assert forbidden not in flow_dialog, f"flow must never import or rebuild robot STEP: {forbidden}"

assert "ClearTheoreticalRobotModel" in flow_header
assert "LoadCurrentRobotCatalogModel" in flow_header

print("MODEL_WELDING_ROBOT_GATE_OK")
