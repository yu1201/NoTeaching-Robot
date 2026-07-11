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
    dialog = read("src/MeasureThenWeldDialog.cpp")
    virtual = read("src/VirtualWeldTestDialog.cpp")
    robot_data = read("src/RobotDataHelper.cpp")
    app_h = read("include/QtWidgetsApplication4.h")
    app = read("src/QtWidgetsApplication4.cpp")

    require("enum class ValidationPolicy" in config_h, "missing Audit/Enforce policy")
    require("CURRENT_VALIDATION_PROFILE_VERSION = 1" in config_h, "missing versioned quality profile")
    require('ReadIntSetting("Validation/ProfileVersion", 0)' in config_cpp,
            "legacy databases are not detected")
    require("storedValidationProfileVersion < CURRENT_VALIDATION_PROFILE_VERSION" in config_cpp,
            "legacy all-off profile is not migrated")
    for token in (
        "settings.validationCoverageEnabled = true;",
        "settings.validationContinuityEnabled = true;",
        "settings.validationDenoiseRatioEnabled = true;",
        "settings.validationResidualEnabled = true;",
        "settings.validationKeyPointEnabled = true;",
        "settings.validationOutputEnabled = true;",
    ):
        require(token in config_cpp, f"mandatory quality check not forced: {token}")

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
    require('root.value("caseId").toString().compare(expectedCaseId' in service,
            "proof can be copied across case directories without rejection")
    require("POINT_CLOUD_QUALITY_ALGORITHM_REVISION" in service,
            "proof policy does not bind an explicit algorithm revision")
    require('POINT_CLOUD_QUALITY_ALGORITHM_REVISION = "pcq-v1-20260711-d"' in service,
            "proof algorithm revision was not advanced for topology/orientation changes")
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
    require("QBuffer payloadBuffer" in service
            and "QCryptographicHash::hash(payload" in service,
            "pose parsing and SHA256 are not derived from the same immutable read snapshot")
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
    require("expectedRetainedSourceLengthMm" in service
            and "declaredStartSkipMm" in service
            and "declaredEndSkipMm" in service,
            "final topology/length thresholds ignore declared endpoint trimming")
    build_evidence_start = service.index("bool BuildQualityFileEvidence(")
    build_evidence_end = service.index("bool WritePointCloudQualityGate(")
    build_evidence = service[build_evidence_start:build_evidence_end]
    require("const QByteArray payload = file.readAll();" in build_evidence
            and "payload.size()" in build_evidence,
            "quality evidence combines file size and hash from different snapshots")
    require("保存可验证的下发轨迹失败" in service
            and "保存可验证的 STEP 最终抽样轨迹失败" in service,
            "FinalSampled save failure is not fail-closed before generation/downlink")
    require("savedSha256" in service and "sampledPoseSha256" in service_h
            and "identity.sampledPoseSha256" in dialog,
            "FinalSampled save and checkpoint freeze do not bind the serialized byte snapshot")

    require("ApplyEnforceValidationSafetyBounds" in config_cpp,
            "Enforce thresholds can still be weakened to an effective off state")
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
    require("stream.status() != QTextStream::Ok" in service
            and "stream.status() != QTextStream::Ok" in robot_data,
            "text stream errors are not checked")

    scan_start = service.index("bool MeasureThenWeldService::ScanMoveAndCollect(")
    scan_end = service.index("bool MeasureThenWeldService::RebuildWeldFilesFromLaserDir(")
    scan = service[scan_start:scan_end]
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
