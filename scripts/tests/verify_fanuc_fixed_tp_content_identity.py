from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


driver = (ROOT / "src" / "FANUCRobotDriver.cpp").read_text(encoding="utf-8")
ensure_start = driver.index("bool FANUCRobotCtrl::EnsureFixedMoveTpUploaded")
ensure_end = driver.index("bool FANUCRobotCtrl::CreateUploadRunTpMove", ensure_start)
ensure = driver[ensure_start:ensure_end]
run = driver[ensure_end:driver.index("bool FANUCRobotCtrl::MoveByJob", ensure_end)]

for token in (
    "QTemporaryDir verificationDir",
    "fixedMoveFtp.downloadFile(remoteTpPath",
    "FanucTpContentIdentity::Read",
    "FanucTpContentIdentity::Matches(localIdentity, remoteIdentity)",
    "localIdentityAfterUpload",
):
    require(token in ensure, f"fixed TP full-content verification missing: {token}")
require(ensure.index("fixedMoveFtp.downloadFile(remoteTpPath") < ensure.index("return true;"),
        "remote TP is trusted before its bytes are downloaded")
require('const std::string remoteFileName = programName + ".tp";' in ensure,
        "fixed TP unexpectedly changed the controller program filename")
require("CallJobWithCompletionState(programName" in run,
        "fixed TP call no longer uses the verified program identity")
require("endpointVerifiedCallLock" in run
        and run.index("endpointVerifiedCallLock") < run.index("EnsureFixedMoveTpUploaded")
        < run.index("CallJobWithCompletionState"),
        "same-endpoint replacement can race between SHA-256 verification and CALL_JOB")
require("file.size == localIdentity.size" in ensure
        and "Matches(localIdentity, remoteIdentity)" in ensure,
        "directory size is still being used as the content proof")

print("PASS: fixed TP keeps CALL name and requires downloaded SHA-256 content identity")
