from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[2]


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def main() -> int:
    header = (ROOT / "include" / "BcpdModelAligner.h").read_text(encoding="utf-8")
    aligner = (ROOT / "src" / "BcpdModelAligner.cpp").read_text(encoding="utf-8")
    dialog = (ROOT / "src" / "ModelAlignmentDialog.cpp").read_text(encoding="utf-8")
    dialog_h = (ROOT / "include" / "ModelAlignmentDialog.h").read_text(encoding="utf-8")
    denoiser = (ROOT / "src" / "PointCloudModelDenoiser.cpp").read_text(encoding="utf-8")
    denoiser_h = (ROOT / "include" / "PointCloudModelDenoiser.h").read_text(encoding="utf-8")
    mesh_h = (ROOT / "include" / "WorkpieceMeshBuilder.h").read_text(encoding="utf-8")
    mesh_cpp = (ROOT / "src" / "WorkpieceMeshBuilder.cpp").read_text(encoding="utf-8")
    model_library_h = (ROOT / "include" / "ReferenceModelLibrary.h").read_text(encoding="utf-8")
    model_library_cpp = (ROOT / "src" / "ReferenceModelLibrary.cpp").read_text(encoding="utf-8")

    require("std::function<bool()> cancelRequested" in header,
            "BCPD options do not expose a cancellation callback")
    for token in (
        "options.cancelRequested && options.cancelRequested()",
        "while (proc.state() != QProcess::NotRunning)",
        "proc.waitForFinished(std::min(100, remainingMs))",
        "BCPD：任务已取消，子进程已终止",
    ):
        require(token in aligner, f"cancellable BCPD wait mechanism missing: {token}")
    terminate_start = aligner.index("bool TerminateProcessAndConfirmStopped")
    terminate_end = aligner.index("// 点云", terminate_start)
    terminate_helper = aligner[terminate_start:terminate_end]
    for token in (
        "process.kill()",
        "process.waitForFinished(2000)",
        "process.waitForFinished(-1)",
        "while (process.state() != QProcess::NotRunning)",
    ):
        require(token in terminate_helper,
                f"fail-safe BCPD termination helper missing: {token}")
    require(aligner.count("TerminateProcessAndConfirmStopped(proc)") == 3,
            "BCPD start-failure/cancel/timeout paths do not all confirm child termination")
    cancel_branch = aligner[
        aligner.index("if (isCancelled())", aligner.index("while (proc.state()")):
        aligner.index("const int remainingMs", aligner.index("while (proc.state()"))
    ]
    timeout_branch = aligner[
        aligner.index("if (timedOut)"):
        aligner.index("if (proc.exitStatus()", aligner.index("if (timedOut)"))
    ]
    require("TerminateProcessAndConfirmStopped(proc)" in cancel_branch
            and "TerminateProcessAndConfirmStopped(proc)" in timeout_branch,
            "BCPD cancel/timeout can release the temporary directory while the child is alive")
    require("proc.kill()" not in aligner,
            "BCPD still has an unchecked direct kill outside the confirmed-stop helper")
    require("bo.cancelRequested = stopRequested" in dialog,
            "model-alignment dialog destruction is not wired to BCPD cancellation")
    require("std::thread m_worker" in dialog_h and "m_worker.join()" in dialog
            and ").detach()" not in dialog,
            "model-alignment worker is not owned and joinable")
    require(dialog.find("m_stopRequested.store(true)") < dialog.find("m_worker.join()"),
            "dialog does not request cancellation before joining its worker")
    require("m_destroying" in dialog_h and "std::memory_order_release" in dialog,
            "normal cancellation and object destruction are not distinguished")
    require("std::thread([this, cloud = std::move(cloud), p]() noexcept" in dialog,
            "worker entry is not a no-throw boundary")
    for token in ("catch (const std::bad_alloc&)", "catch (const std::exception& ex)",
                  "catch (...)"):
        require(dialog.count(token) >= 2,
                f"worker/start boundary is missing exception containment: {token}")
    require("setCancelButtonText(QStringLiteral(\"取消\"))" in dialog
            and "&QProgressDialog::canceled" in dialog,
            "normal UI does not expose a reachable model-alignment cancel action")
    browse_start = dialog.index("void ModelAlignmentDialog::OnBrowseCloud()")
    browse_end = dialog.index("void ModelAlignmentDialog::OnRun()", browse_start)
    browse = dialog[browse_start:browse_end]
    require("m_worker = std::thread" in browse and ") noexcept" in browse
            and "LoadCloudPointsWithProgress" in browse,
            "cloud browsing still parses a large file on the UI thread")
    require("OnCloudLoadFinished" in browse and "m_inputCloud = std::move(points)" in browse
            and "if (m_worker.joinable()) m_worker.join()" in browse,
            "cloud load is not atomically published after the owned worker joins")
    require("QApplication::setOverrideCursor" not in browse,
            "cloud browsing still uses a blocking UI-thread wait cursor path")
    require("dopt.cancelRequested" in dialog and "bool cancelled = false" in denoiser_h,
            "denoiser does not receive or expose cancellation state")
    require(denoiser.count("isCancelled()") >= 8,
            "denoiser hot loops do not check the shared cancellation token")
    for token in (
        "kMaximumCellsPerTriangle",
        "kMaximumTriangleReferences",
        "kMaximumTrianglesPerBucket",
        "(totalReferences & 0xff) == 0 && isCancelled()",
        "(visitedCandidates++ & 0xff) == 0 && isCancelled()",
        "resourceLimitExceeded",
    ):
        require(token in denoiser or token in denoiser_h,
                f"bounded/cancellable triangle grid contract missing: {token}")
    require("CancelCallback" in mesh_h and "cancelRequested" in model_library_h
            and "cancelRequested" in model_library_cpp,
            "reference-model PLY loading is not cancellation-aware end-to-end")
    for token in (
        "kMaximumFileBytes",
        "kMaximumHeaderBytes",
        "kMaximumHeaderLineBytes",
        "kMaximumVertices",
        "kMaximumFaces",
        "headerBytes + vertexBytes + faceBytes != openedFileSize",
        "!IsFiniteVec(vertex) || !IsFiniteVec(normal)",
        "indices[0] >= static_cast<quint32>(vertexCount)",
        "catch (const std::bad_alloc&)",
    ):
        require(token in mesh_cpp, f"strict bounded PLY loader contract missing: {token}")
    require("file.read(vertexCount" not in mesh_cpp and "file.read(faceCount" not in mesh_cpp,
            "PLY loader still allocates an attacker-sized whole payload block")
    for token in (
        "struct CloudLoadLimits",
        "768LL * 1024LL * 1024LL",
        "maximumPhysicalLines = 10'000'000",
        "maximumLineBytes = 64 * 1024",
        "maximumValidPoints = 8'000'000",
    ):
        require(token in mesh_h, f"bounded cloud-loader API missing: {token}")
    for token in (
        "FILE_ATTRIBUTE_REPARSE_POINT",
        "info.isSymLink()",
        "file.readLine(limits.maximumLineBytes + 1)",
        "lineCounter > limits.maximumPhysicalLines",
        "loaded.size() >= limits.maximumValidPoints",
        "std::numeric_limits<int>::min()",
        "if (!std::isfinite(value))",
        "file.pos() != openedFileSize || file.size() != openedFileSize",
        "points = std::move(loaded)",
        "normalized == kSimpleHeader || normalized == kLegacyHeader",
        "未知非数字内容",
    ):
        require(token in mesh_cpp, f"strict bounded cloud loader contract missing: {token}")
    require("const QByteArray line = file.readLine();" not in mesh_cpp,
            "cloud loader still performs an unbounded physical-line read")
    require((ROOT / "scripts" / "tests" / "model_alignment_cancellation_tests.cpp").is_file(),
            "real C++ denoiser cancellation regression is missing")
    require("const QByteArray data = f.readAll()" not in aligner and "readLine(512)" in aligner,
            "BCPD output is still read as one unbounded allocation")
    require("kMaximumDiagnosticBytes" in aligner and "drainOutput()" in aligner,
            "BCPD child output is not drained into a bounded diagnostic tail")
    require("f.write(buf) != buf.size()" in aligner,
            "BCPD temporary matrix writes are not checked")

    print("PASS: model-alignment owned worker cancels BCPD I/O, parsing, and denoising before join")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except AssertionError as exc:
        print(f"FAIL: {exc}")
        sys.exit(1)
