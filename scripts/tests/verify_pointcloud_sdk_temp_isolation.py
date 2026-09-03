from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def section(text: str, start: str, end: str) -> str:
    begin = text.find(start)
    require(begin >= 0, f"missing section start: {start}")
    finish = text.find(end, begin + len(start))
    require(finish >= 0, f"missing section end after {start}: {end}")
    return text[begin:finish]


def main() -> int:
    source = (ROOT / "src/PointCloudExtractionProcessor.cpp").read_text(encoding="utf-8")

    require("#include <QTemporaryDir>" in source,
            "point-cloud extraction does not use RAII temporary directories")
    require("QtWidgetsApplication4_PointCloudExtration/" not in source,
            "runtime SDK config still uses the process-global fixed temporary directory")

    extract = section(
        source,
        "PointCloudExtractionProcessor::ExtractionResult PointCloudExtractionProcessor::ExtractCorrugatedSheet(",
        "RobotCalculation::MeasureThenWeldAnalysisResult PointCloudExtractionProcessor::BuildAnalysisResult(",
    )
    for token in (
        "const QString& runtimeConfigDir",
        "const QFileInfo runtimeConfigDirInfo(runtimeConfigDir)",
        "runtimeConfigDirInfo.exists()",
        "runtimeConfigDirInfo.isDir()",
        "runtimeConfigDirInfo.absoluteFilePath()",
        "EncodeSdkPathLosslessly(",
        "runtimeConfigPath, QStringLiteral(\"运行配置路径\"), &configPathBytes, &result.error",
    ):
        require(token in extract, f"parent-owned SDK config isolation missing: {token}")
    require("QString::fromLocal8Bit(localBytes) != path" in source,
            "SDK path conversion does not reject lossy Windows local-code-page paths")
    require("NoTeaching-Robot-low-level" in source,
            "SDK path conversion error does not show an ASCII-safe directory example")
    for purpose in ("日志目录", "基础焊道输出路径", "运行配置路径"):
        require(f'QStringLiteral("{purpose}")' in source,
                f"SDK narrow path is not guarded before conversion: {purpose}")
    require("QTemporaryDir runtimeConfigDir" not in extract,
            "SDK child still owns a sibling temp directory that survives a hard crash")
    for token in (
        'QStringLiteral("findWeldingLine.dll")',
        'QStringLiteral("bin/findWeldingLine.dll")',
        'QStringLiteral("PointCloudExtration.dll")',
        "LOAD_WITH_ALTERED_SEARCH_PATH",
        "findWeldingLineCompatibility",
    ):
        require(token in extract, f"SDK DLL layout compatibility missing: {token}")
    require('ReplaceConfigValue(&content, "is_remove_noise", "false")' in source,
            "findWeldingLine runtime config does not disable the incompatible noise pass")
    require('ReplaceConfigValue(&content, "Move_distance", "0")' in source,
            "findWeldingLine runtime config can still extrapolate beyond measured endpoints")

    isolated = section(
        source,
        "PointCloudExtractionProcessor::ExtractionResult PointCloudExtractionProcessor::ExtractCorrugatedSheetIsolated(",
        "int PointCloudExtractionProcessor::RunExtractWorker(",
    )
    for token in (
        "QTemporaryDir workDir",
        "QtWidgetsApplication4_sdkworker_%1_XXXXXX",
        "QCoreApplication::applicationPid()",
        "workDir.setAutoRemove(true)",
        "workDir.isValid()",
        "workDir.filePath(QStringLiteral(\"input_cloud.txt\"))",
        "workDir.filePath(QStringLiteral(\"extract_result.txt\"))",
        "workDir.filePath(QStringLiteral(\"extract_settings.bin\"))",
        "WriteWorkerSettingsFile(settingsFile, settings)",
        "<< workDir.path()",
        "<< settingsFile",
    ):
        require(token in isolated, f"SDK worker temporary-file isolation missing: {token}")

    require("QDir().mkpath(workDir)" not in isolated,
            "SDK worker still creates a shared fixed work directory")
    require("QDir(workDir).filePath" not in isolated,
            "SDK worker is not using the QTemporaryDir-owned path")

    worker_start = source.find("int PointCloudExtractionProcessor::RunExtractWorker(")
    require(worker_start >= 0, "missing point-cloud SDK worker entry")
    worker = source[worker_start:]
    for token in (
        "workerArgs.size() < 8",
        "const QString runtimeConfigDir = workerArgs[6]",
        "const QString settingsFile = workerArgs[7]",
        "ReadWorkerSettingsFile(settingsFile, &settings, &settingsError)",
        "baseWeldOut, runtimeConfigDir",
    ):
        require(token in worker, f"worker does not consume its parent-owned runtime directory: {token}")
    require("PointCloudProcessingConfig::Load()" not in worker,
            "worker reloads mutable global settings instead of using the parent snapshot")

    for token in (
        "WORKER_SETTINGS_MAGIC",
        "settings.libraryDir",
        "settings.configPath",
        "settings.zTruncationValue",
        "settings.resampleStepMm",
    ):
        require(token in source, f"SDK settings snapshot is incomplete: {token}")

    require("out << p.index << ' '" in source,
            "worker protocol does not preserve TrackPoint.index")
    require("p.index = t[0].toInt()" in source,
            "worker result reader does not restore TrackPoint.index")

    print("PASS: SDK worker IPC files and runtime config are isolated per invocation and RAII-cleaned")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
