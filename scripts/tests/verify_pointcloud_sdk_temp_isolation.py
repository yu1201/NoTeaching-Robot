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
        "const QByteArray configPathBytes = runtimeConfigPath.toLocal8Bit()",
    ):
        require(token in extract, f"parent-owned SDK config isolation missing: {token}")
    require("QTemporaryDir runtimeConfigDir" not in extract,
            "SDK child still owns a sibling temp directory that survives a hard crash")

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
