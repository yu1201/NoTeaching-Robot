#!/usr/bin/env python3
"""Static regression guard for the production point-cloud quality gate."""

from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[2]


def read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def main() -> int:
    config_h = read("include/PointCloudProcessingConfig.h")
    config_cpp = read("src/PointCloudProcessingConfig.cpp")
    config_db_h = read("include/ConfigDatabase.h")
    config_db = read("src/ConfigDatabase.cpp")
    calc_h = read("include/RobotCalculation.h")
    calc_cpp = read("src/RobotCalculation.cpp")
    extraction = read("src/PointCloudExtractionProcessor.cpp")
    extraction_h = read("include/PointCloudExtractionProcessor.h")
    service_h = read("include/MeasureThenWeldService.h")
    service = read("src/MeasureThenWeldService.cpp")
    proof_integrity_h = read("include/PointCloudProofIntegrity.h")
    proof_integrity = read("src/PointCloudProofIntegrity.cpp")
    proof_integrity_test = read("scripts/tests/pointcloud_proof_integrity_tests.cpp")
    dialog = read("src/MeasureThenWeldDialog.cpp")
    virtual = read("src/VirtualWeldTestDialog.cpp")
    robot_data = read("src/RobotDataHelper.cpp")
    seam_dialog = read("src/WeldSeamCompDialog.cpp")
    app_h = read("include/QtWidgetsApplication4.h")
    app = read("src/QtWidgetsApplication4.cpp")

    require("enum class ValidationPolicy" in config_h, "missing Audit/Enforce policy")
    require("CURRENT_VALIDATION_PROFILE_VERSION = 1" in config_h, "missing versioned quality profile")
    require('ReadIntSetting("Validation/ProfileVersion", 0)' in config_cpp,
            "legacy databases are not detected")
    require("storedValidationProfileVersion < CURRENT_VALIDATION_PROFILE_VERSION" in config_cpp,
            "legacy all-off profile is not migrated")
    quality_switches = (
        "Coverage", "Continuity", "DenoiseRatio", "Residual", "KeyPoint", "Output",
    )
    for name in quality_switches:
        field = f"validation{name}Enabled"
        require(f"bool {field} = true;" in config_h, f"quality gate default is not safe-on: {field}")
        require(f'ReadBoolSetting("Validation/{name}Enabled"' in config_cpp,
                f"quality gate is not loaded: {field}")
        require(f'write("Validation/{name}Enabled", normalizedSettings.{field} ? "1" : "0")'
                in config_cpp, f"quality gate is not persisted as a real switch: {field}")
        require(f"normalizedSettings.{field} = true;" not in config_cpp,
                f"Save still silently forces quality gate on: {field}")
    require("if (storedValidationProfileVersion < CURRENT_VALIDATION_PROFILE_VERSION)" in config_cpp
            and all(f"settings.validation{name}Enabled = true;" in config_cpp
                    for name in quality_switches),
            "legacy profile migration no longer restores all six quality gates")

    safety_gate_names = (
        "ProofIntegrity", "ProductionPurpose", "RobotNameBinding", "CaseBinding",
        "EndpointBinding", "CameraHandEyeBinding", "Freshness", "PolicySnapshot",
        "InputEvidence", "AuthorizedPoseIdentity", "TrajectoryStructure", "MotionPrecheck",
    )
    for name in safety_gate_names:
        field = f"safetyGate{name}Enabled"
        require(f"bool {field} = true;" in config_h, f"system gate default is not safe-on: {field}")
        require(f'ReadBoolSetting("SafetyGates/{name}Enabled"' in config_cpp,
                f"system gate is not loaded: {field}")
        require(f'write("SafetyGates/{name}Enabled", settings.{field} ? "1" : "0")'
                in config_cpp, f"system gate is not persisted: {field}")
    core_helper_start = config_cpp.index(
        "bool PointCloudProcessingConfig::CoreSafetyGatesEnabled(")
    core_helper_end = config_cpp.index(
        "bool PointCloudProcessingConfig::HasDisabledCoreSafetyGate(", core_helper_start)
    core_helper = config_cpp[core_helper_start:core_helper_end]
    require("safetyGateRobotNameBindingEnabled" not in core_helper
            and all(f"safetyGate{name}Enabled" in core_helper
                    for name in safety_gate_names if name != "RobotNameBinding"),
            "core-gate helper must cover every system gate except logical robot-name binding")
    require("if (HasDisabledCoreSafetyGate(settings))" in config_cpp
            and "settings.validationPolicy = ValidationPolicy::Audit;" in config_cpp,
            "disabled core gate does not force the effective loaded policy to Audit")
    save_start = config_cpp.index("bool PointCloudProcessingConfig::Save(")
    save_body = config_cpp[save_start:]
    require("if (HasDisabledCoreSafetyGate(normalizedSettings))" in save_body
            and "normalizedSettings.validationPolicy = ValidationPolicy::Audit;" in save_body,
            "Save can persist Enforce while a core system gate is disabled")

    require("struct PointCloudQualityReport" in calc_h, "missing structured quality report")
    for metric in (
        "projectedSpanMm", "stationCoverageRatio", "longestContinuousRatio",
        "medianResidualMm", "p95ResidualMm", "residualInlierRatio",
        "keyPointCount", "cornerCount", "outputLengthMm", "maxOutputStepMm",
    ):
        require(metric in calc_h and metric in calc_cpp, f"quality metric not wired: {metric}")

    require("(isLapStepPair || isEndpointAdjacent) ? 0.25 : 3.0" in calc_cpp,
            "lap/endpoint short segments and interior hard minimums are not separated")
    require("validationMinSegmentLengthMm" in calc_cpp and "report.warnings" in calc_cpp,
            "15mm segment threshold must be warning-only after hard structural checks")
    require("startCount != 1 || endCount != 1" in calc_cpp,
            "unique start/end structural gate missing")
    require("kHardMaxOutputStepMm = 50.0" in calc_cpp,
            "output discontinuity hard gate missing")
    require("if (!params.validationAuditOnly)" in calc_cpp,
            "Audit mode does not preserve report without enforcing thresholds")

    require("EvaluateMeasureThenWeldOutputQuality" in extraction,
            "SDK direct output still bypasses shared quality evaluation")
    require("finiteInputPointCount" in extraction_h and "invalidInputPointCount" in extraction_h,
            "SDK worker does not report complete-cloud finite/invalid counters")
    require("extraction.finiteInputPointCount" in extraction
            and "extraction.invalidInputPointCount" in extraction,
            "SDK direct gate still hard-codes its input quality counters")
    require("result.qualityReport.failures.join" in extraction,
            "SDK direct quality failure is not propagated")

    require("PreciseLaserPoint_QualityGate.json" in service,
            "production quality proof filename missing")
    require("WritePointCloudQualityGate" in service and "VerifyPointCloudQualityGate" in service,
            "quality proof write/verify pair missing")
    require(service.count("VerifyWeldPoseAuthorization(") >= 8
            and service.count("verifyLoadedPoseAuthorization()") >= 8,
            "pose authorization is not checked at generation/downlink/execution/TOCTOU points")
    require("policyRevisionSha256" in service and "authorizedPose" in service,
            "proof does not bind policy revision and authorized pose")
    for key, field in (
        ("coverageEnabled", "validationCoverageEnabled"),
        ("continuityEnabled", "validationContinuityEnabled"),
        ("denoiseRatioEnabled", "validationDenoiseRatioEnabled"),
        ("residualEnabled", "validationResidualEnabled"),
        ("keyPointEnabled", "validationKeyPointEnabled"),
        ("outputEnabled", "validationOutputEnabled"),
    ):
        require(f'if (!settings.{field})' in service
                and f'thresholds.insert("{key}", false);' in service,
                f"proof threshold snapshot does not expose disabled quality gate: {field}")
    require('root.value("caseId").toString().compare(expectedCaseId' in service,
            "proof can be copied across case directories without rejection")
    require("POINT_CLOUD_QUALITY_ALGORITHM_REVISION" in service,
            "proof policy does not bind an explicit algorithm revision")
    require('POINT_CLOUD_QUALITY_ALGORITHM_REVISION = "pcq-v3-20260712-a"' in service,
            "proof algorithm revision was not advanced for topology/orientation changes")
    require("POINT_CLOUD_QUALITY_SCHEMA_VERSION = 3" in service
            and 'root.insert("schemaVersion", POINT_CLOUD_QUALITY_SCHEMA_VERSION)' in service,
            "MAC/receipt-bound schema 3 is not the only newly written proof schema")
    for token in (
        "SignPointCloudQualityProof",
        "VerifyPointCloudQualityProofMac",
        "ConstantTimeBytesEqual",
        "POINT_CLOUD_PROOF_RECEIPT_SCOPE",
        "RegisterPointCloudProofReceipt",
        "VerifyPointCloudProofReceipt",
        "VerifyPointCloudProofInputs",
        "QStringLiteral(\"secret\")",
    ):
        require(token in service, f"schema-3 proof integrity/receipt wiring missing: {token}")
    for token in (
        "QMessageAuthenticationCode::hash",
        "POINT_CLOUD_PROOF_HMAC_ALGORITHM",
        "ConstantTimeEqual",
        "SignProof",
        "VerifyProofMac",
        "BuildReceiptRecord",
        "VerifyReceiptRecord",
        "BeginProofReplacement",
        "RequireProofReplacementActive",
        "VerifyProofNotDenied",
        "CompleteProofReplacement",
        "AbandonProofReplacement",
        "AcquireProofUseLease",
        "g_activeProofReaders",
        "QSaveFile marker",
        "setDirectWriteFallback(false)",
    ):
        require(token in proof_integrity,
                f"shared C++ proof integrity implementation missing: {token}")
    require("PointCloudProofIntegrity::SignProof" in service
            and "PointCloudProofIntegrity::VerifyProofMac" in service
            and "PointCloudProofIntegrity::VerifyReceiptRecord" in service,
            "production verifier is not wired to the shared dynamically tested C++ implementation")
    for attack in (
        "state mutation must invalidate",
        "input-evidence mutation must invalidate",
        "different machine/key",
        "missing DPAPI receipt",
        "another case path",
        "pose-evidence mutation",
        "production-context mutation",
        "proof-time mutation",
        "exclusive old proof fixture must force deletion failure",
        "locked old authorized proof must remain rejected",
        "failed marker removal must keep proof denied",
        "normal cancellation must retain a persistent denial tombstone",
        "non-owner thread must not inherit active producer privilege",
        "corrupt persisted denial marker must reject proof without memory state",
        "replacement writer must fail immediately while motion lease is active",
        "motion reader must fail immediately while replacement writer is active",
        "oversized proof must reject before readAll allocation",
        "sparse giant evidence must reject before streaming",
        "oversized tombstone must fail closed without readAll",
    ):
        require(attack in proof_integrity_test,
                f"real C++ point-cloud attack regression missing: {attack}")
    require(service.index("VerifyPointCloudQualityProofMac(root, error)")
            < service.index("ValidatePointCloudProductionContext(",
                            service.index("bool VerifyPointCloudQualityGate(")),
            "production verifier trusts context before checking proof MAC")
    for token in (
        'root.insert("productionContext", productionContextJson)',
        'result.insert("scanRunId", context.scanRunId)',
        'result.insert("scanStartedUtc", context.scanStartedUtc)',
        'result.insert("robotEndpoint", context.robotEndpoint)',
        'result.insert("cameraSection", context.cameraSection)',
        'result.insert("handEyeSha256", context.handEyeSha256)',
        'context.origin = QStringLiteral("liveRobotCameraScan")',
        "RobotOperationLease::PersistentEndpointIdentity(driver)",
        "HandEyeMatrixContentSha256(calibration)",
    ):
        require(token in service, f"production proof context binding missing: {token}")
    require("POINT_CLOUD_PROOF_MAX_AGE_SECONDS = 24 * 60 * 60" in service
            and "POINT_CLOUD_PROOF_MAX_FUTURE_SKEW_SECONDS = 5 * 60" in service
            and "ParseStrictUtcTimestamp" in service
            and "proofAgeSeconds > POINT_CLOUD_PROOF_MAX_AGE_SECONDS" in service
            and "scanAgeSeconds > POINT_CLOUD_PROOF_MAX_AGE_SECONDS" in service,
            "proof/scan UTC freshness and future-skew gates are incomplete")
    require("expectedDriver != nullptr" in service
            and service.count("pRobotDriver))") >= 4
            and "pRobotDriver);" in service,
            "movement paths do not compare the proof endpoint with the current driver")
    require("frozenExpectation != nullptr" in service
            and "PointCloudProduction 验证缺少当前机器人或 UI 线程冻结的完整生产上下文" in service,
            "offline production verification can run without current/frozen robot context")
    require("ComputeFileSha256ForResumeGate" in service,
            "proof does not bind files by SHA256")
    require("ValidateFinalWeldPoseArtifact" in service,
            "final SeamComp artifact is not read back and structurally checked")
    require("WeldPoseSource" in service_h and "PointCloudProduction" in service_h,
            "pose provenance is not explicit")
    require("SyntheticVirtualTest" in virtual,
            "virtual weld exemption is not narrow and explicit")
    require("qualityProofSourcePosePath" in dialog,
            "resume flow does not bind proof to original SeamComp source")
    require("identity.qualityProofPosePath" in dialog,
            "a second resume would lose the original authorized SeamComp proof source")
    require("QCryptographicHash payloadHash" in service
            and "file.readLine(" in service
            and "payloadHash.addData(rawLine)" in service,
            "pose parsing and SHA256 are not derived from the same bounded streamed bytes")
    require("RegisterSyntheticPoseAuthorization" in service
            and "VerifySyntheticPoseAuthorization" in service,
            "SyntheticVirtualTest remains an unbound public-enum bypass")
    for token in (
        "FINAL_MIN_PRECOMP_LENGTH_RATIO",
        "FINAL_MAX_PRECOMP_LENGTH_RATIO",
        "FINAL_MIN_MATCHED_ARC_RATIO",
        "FINAL_MIN_SOURCE_UNIQUE_COVERAGE_RATIO",
        "FINAL_MIN_SOURCE_ARC_SPAN_RATIO",
        "matchedFinalArcMm",
        "uniqueMatchedSourceIndexes",
        "sourceArcSpanRatio",
        "sourceIndex < lastMatchedSourceIndex",
    ):
        require(token in service, f"final SeamComp topology gate missing: {token}")
    require("qualityReport.projectedSpanMm" in service,
            "final SeamComp length is not bound to this analysis")
    require("ControllerEulerDistanceDeg" in service
            and "PhysicalOrientationDistanceDeg" in service
            and "RotationFromAnglesDeg" in service,
            "final pose orientation is not checked in controller and physical rotation semantics")
    require("std::remainder" in service
            and "MAX_REASONABLE_ROBOT_ANGLE_DEG" in service,
            "large finite robot angles can still hang iterative normalization")
    require("generatedPayload != expectedPayload" in service
            and "expectedGeneratedSha256" in service
            and "generatedSeamCompSha256" in service,
            "final artifact validation is not bound to the exact bytes produced by compensation")
    require("hasNearbySourcePose" in service,
            "25mm/60deg per-point hard limits can be diluted as unmatched arc")
    require("IsKnownWeldPointType" in service
            and "IsKnownWeldSegmentKind" in service
            and "sourceLapIndexes" in service,
            "execution-affecting final pose labels are not semantically constrained")
    require("validatedSourceSha256" in service
            and "validatedWeldPoseSha256" in service
            and "补偿前焊道在结构验证与质量证明提交之间发生变化" in service,
            "pre-compensation source validation and proof do not bind the same snapshot")
    require('root.value("thresholds").toObject() != BuildPointCloudQualityThresholds' in service,
            "proof threshold evidence is not compared with the active policy")
    require('QStringLiteral("candidatePose")' in service
            and "Audit：最终焊接姿态结构验证未通过" in service,
            "Audit failures cannot persist an explicitly unauthorized evidence report")
    require("hasValidatedAuthorizedPose && hasProductionContext" in service
            and "Enforce 质量通过但缺少 live-scan" in service,
            "an offline/missing-context result can still become authorized")
    require("authorize && inputPaths.isEmpty()" in service
            and "if (authorize)" in service
            and "inputs.size() != inputPaths.size()" in service
            and "预期点云输入为空或不存在" in service,
            "authorized proof can omit a missing/empty expected point-cloud input")
    require("expectedRetainedSourceLengthMm" in service
            and "declaredStartSkipMm" in service
            and "declaredEndSkipMm" in service,
            "final topology/length thresholds ignore declared endpoint trimming")
    build_evidence_start = service.index("bool BuildQualityFileEvidence(")
    build_evidence_end = service.index("bool WritePointCloudQualityGate(")
    build_evidence = service[build_evidence_start:build_evidence_end]
    require("PointCloudProofIntegrity::HashFileBounded" in build_evidence
            and "MaximumEvidenceFileBytes" in build_evidence
            and "stopRequested" in build_evidence
            and "readAll()" not in build_evidence,
            "quality evidence is not bounded/cancelable streaming hash evidence")
    for token in (
        "MaximumProofBytes",
        "MaximumDenialTombstoneBytes",
        "MaximumEvidenceFileBytes",
        "MaximumEvidenceTotalBytes",
        "MaximumWeldPoseBytes",
        "MaximumWeldPoseLines",
    ):
        require(token in proof_integrity_h, f"proof/evidence hard limit missing: {token}")
    require("保存可验证的下发轨迹失败" in service
            and "保存可验证的 STEP 最终抽样轨迹失败" in service,
            "FinalSampled save failure is not fail-closed before generation/downlink")
    require("savedSha256" in service and "sampledPoseSha256" in service_h
            and "identity.sampledPoseSha256" in dialog,
            "FinalSampled save and checkpoint freeze do not bind the serialized byte snapshot")

    rebuild_start = service.index("bool MeasureThenWeldService::RebuildWeldFilesFromLaserDir(")
    rebuild_end = service.index("QString MeasureThenWeldService::BuildResultDir(", rebuild_start)
    rebuild = service[rebuild_start:rebuild_end]
    require(rebuild.index("LoadValidatedRebuildPointCloudContext(")
            < rebuild.index("InvalidatePointCloudQualityGate("),
            "rebuild destroys the old proof before freezing its live-scan context")
    require(rebuild.index("BeginProofReplacement(")
            < rebuild.index("InvalidatePointCloudQualityGate("),
            "rebuild can touch the old proof before persisting its denial tombstone")
    require("const bool invalidated = InvalidatePointCloudQualityGate" in rebuild
            and "旧 proof 删除失败但仍被拒绝" in rebuild,
            "rebuild cancellation still swallows proof invalidation failure")
    require(rebuild.index('cancellationPoint(QStringLiteral("STEP 程序同步后"))')
            < rebuild.rindex("qualityGateReplacement.Complete(error)"),
            "rebuild releases its denial tombstone before STEP/cancellation completes")
    require("PointCloudProofReplacementSession qualityGateReplacement" in rebuild,
            "rebuild early returns do not abandon producer privilege through RAII")
    require("旧 schema 1/2 点云质量证明" in service
            and "VerifyPointCloudProofInputs(root, dir, error, stopRequested)" in service
            and "return PointCloudProofIntegrity::VerifyProofNotDenied(reportPath, error);"
                in service,
            "legacy/self-declared proof or modified scan inputs can still be upgraded")
    require("productionExpectation.robotName.trimmed().isEmpty()" in rebuild
            and "离线预览不能生成可运动授权" in rebuild,
            "offline rebuild can synthesize a production proof without robot context")
    require("const PointCloudProcessingConfig::Settings& settings" in service
            and "productionExpectation,\n                pointCloudSettings,\n                productionContext"
                in rebuild,
            "validated rebuild does not use the same loaded safety-gate snapshot")
    require(service.count("safetyGateRobotNameBindingEnabled") >= 5,
            "robot-name gate is not applied consistently to rebuild and execution checks")
    require('thresholds.insert("robotNameBindingEnabled", false);' in service,
            "robot-name exception is not bound into the proof policy revision")
    require("settings.safetyGateRobotNameBindingEnabled\n        && proofRobotName.compare" in service,
            "old-proof robot-name inheritance ignores its configured switch")
    require("currentSettings.safetyGateRobotNameBindingEnabled\n        && !expectedRobotName" in service,
            "production execution robot-name comparison ignores its configured switch")
    require("expectation.robotEndpoint.trimmed().isEmpty()" in service
            and "expectation.cameraSection.trimmed().isEmpty()" in service
            and "IsSha256Text(expectation.handEyeSha256)" in service,
            "robot-name switch accidentally weakens endpoint/camera/hand-eye binding")
    require("CapturePointCloudProductionExpectation" in service_h
            and "productionExpectation" in dialog
            and "rebuildExpectation" in seam_dialog
            and "productionExpectation" in app,
            "production rebuild entry points do not propagate a frozen robot context")
    require("rebuildDriver" not in seam_dialog
            and "rebuildExpectation, cancelFlag" in seam_dialog,
            "preview background worker still captures a raw robot driver")

    require("ApplyEnforceValidationSafetyBounds" in config_cpp,
            "Enforce thresholds can still be weakened to an effective off state")
    bounds_start = config_cpp.index("void ApplyEnforceValidationSafetyBounds(")
    bounds_end = config_cpp.index("QString ReadSetting(", bounds_start)
    bounds = config_cpp[bounds_start:bounds_end]
    for name in quality_switches:
        require(f"if (settings.validation{name}Enabled)" in bounds,
                f"Enforce safety floor is not scoped to enabled quality gate: {name}")
    require("NormalizeFiniteLoadValues(settings);" in config_cpp
            and config_cpp.index("NormalizeFiniteLoadValues(settings);")
            < config_cpp.index("settings.validationMinStationCoverageRatio = std::clamp"),
            "non-finite point-cloud values are not normalized before threshold clamps")
    for field in (
        "zTruncationValue",
        "resampleStepMm",
        "fitSampleStepMm",
        "validationMinProjectedSpanMm",
        "validationMinStationCoverageRatio",
        "validationMinLongestContinuousRatio",
        "validationMaxRejectedRatio",
        "validationMaxMedianResidualMm",
        "validationMaxP95ResidualMm",
        "validationResidualInlierThresholdMm",
        "validationMinResidualInlierRatio",
        "validationMinSegmentLengthMm",
        "validationMinOutputLengthRatio",
    ):
        require(f"useDefaultIfNonFinite(settings.{field}, defaults.{field})" in config_cpp,
                f"non-finite configuration can penetrate load: {field}")
    require('ReadDoubleSetting("External/ZTruncationValue", settings.zTruncationValue)' in config_cpp
            and 'ReadDoubleSetting("External/ResampleStepMm", settings.resampleStepMm)' in config_cpp,
            "SDK Z/step settings still use unchecked numeric conversion")
    require("return ok && std::isfinite(value) ? value : defaultValue;" in config_cpp,
            "generic point-cloud double settings still accept persisted NaN/Inf")
    require("std::isfinite(stepMm) && stepMm > 0.0" in extraction,
            "SDK resampling still accepts NaN/Inf steps")
    require("ReadScopedSettings" in config_db_h and "SettingsSnapshotScope" in config_cpp,
            "point-cloud profile load is not one SQLite query snapshot")
    require("WriteScopedSettings" in config_db_h and "db.transaction()" in config_db,
            "point-cloud profile persistence is not transaction-capable")
    require("ConfigDatabase::WriteScopedSettings" in config_cpp,
            "point-cloud profile save still exposes a mixed-version partial-write window")
    require("bool RunGenerateStepWeldProgramForCli" in app_h,
            "STEP generation CLI cannot propagate a quality-gate failure")
    require("if (!RunGenerateStepWeldProgramForCli(" in app
            and "if (!RunWeldSeamCompForCli(" in app,
            "CLI dispatcher does not return a non-zero exit status on proof/output failure")
    require("bool RunLaserClassifyDirForCli" in app_h
            and "!RunLaserClassifyDirForCli(" in app,
            "batch quality replay can still report exit 0 after a rejected case")
    require("--generate-step-weld-program 缺少文件参数" in app,
            "missing CLI action parameters still silently exit 0")

    require("QSaveFile file(filePath);" in service,
            "service text outputs are not atomically committed")
    require("QSaveFile file(filePath);" in robot_data,
            "RobotDataHelper text outputs are not atomically committed")
    require("file.error() != QFileDevice::NoError" in service
            and "stream.status() != QTextStream::Ok" in robot_data,
            "streamed text/file errors are not checked")

    scan_start = service.index("bool MeasureThenWeldService::ScanMoveAndCollect(")
    scan_end = service.index("bool MeasureThenWeldService::RebuildWeldFilesFromLaserDir(")
    scan = service[scan_start:scan_end]
    require(scan.index("BeginProofReplacement(")
            < scan.index("InvalidatePointCloudQualityGate("),
            "live scan can touch a proof before persisting its denial tombstone")
    require(scan.index("RobotOperationLease::IsCancellationRequested(pRobotDriver)")
            < scan.rindex("qualityGateReplacement.Complete(qualityGateError)"),
            "live scan releases its denial tombstone before its final STOP check")
    require("PointCloudProofReplacementSession qualityGateReplacement" in scan,
            "live-scan early returns do not abandon producer privilege through RAII")

    write_gate_start = service.index("bool WritePointCloudQualityGate(")
    write_gate_end = service.index("bool InvalidatePointCloudQualityGate(", write_gate_start)
    write_gate = service[write_gate_start:write_gate_end]
    require("RequireProofReplacementActive(reportPath, error)" in write_gate,
            "proof writer can publish outside an active denied replacement session")

    verify_gate_start = service.index("bool VerifyPointCloudQualityGate(")
    verify_gate_end = service.index("bool VerifyWeldPoseAuthorization(", verify_gate_start)
    verify_gate = service[verify_gate_start:verify_gate_end]
    require(verify_gate.index("if (!verifyNotDenied())")
            < verify_gate.index("PointCloudProofIntegrity::ReadFileBounded("),
            "production verifier reads proof bytes before checking the denial tombstone")
    require("return verifyNotDenied();" in verify_gate,
            "production verifier does not recheck denial after proof/context/input reads")

    context_start = service.index("bool LoadValidatedRebuildPointCloudContext(")
    context_end = service.index("bool WritePointCloudQualityGate(", context_start)
    context_loader = service[context_start:context_end]
    require(context_loader.index("VerifyProofNotDenied(reportPath, error)")
            < context_loader.index("PointCloudProofIntegrity::ReadFileBounded("),
            "rebuild context loader can inherit a persistently denied proof")

    execute_start = service.index("bool MeasureThenWeldService::ExecuteWeldPoseFileWithSafePos(")
    execute_end = service.index("bool MeasureThenWeldService::LoadCompPreviewBaseline(", execute_start)
    execute = service[execute_start:execute_end]
    require("PointCloudProofIntegrity::ProofUseLease qualityProofUseLease" in execute
            and execute.index("AcquireProofUseLease(") < execute.index("verifyLoadedPoseAuthorization()")
            and execute.index("AcquireProofUseLease(") < execute.index("MoveByJob(")
            and execute.index("AcquireProofUseLease(") < execute.index("CallJobAndWaitStateDone("),
            "actual motion does not retain a proof use lease across verify and START")
    generate_start = service.index("bool MeasureThenWeldService::GenerateStepWeldProgramFiles(")
    generate_end = service.index("bool MeasureThenWeldService::GenerateVirtualStraightWeldFiles(", generate_start)
    require("AcquireProofUseLease(" not in service[generate_start:generate_end],
            "pure STEP file generation incorrectly acquires a motion proof lease")
    for message in (
        "保存SDK提取焊道结果失败",
        "保存先测后焊特征提取结果失败",
        "保存焊道分类结果失败",
        "保存起终点/拐点结果失败",
        "保存焊接姿态结果失败",
    ):
        pos = scan.index(message)
        tail = scan[pos:pos + 260]
        require("return false;" in tail, f"save failure remains fail-open: {message}")

    print("point-cloud quality gate static verification: PASS")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except AssertionError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        raise SystemExit(1)
