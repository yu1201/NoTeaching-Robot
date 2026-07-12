#!/usr/bin/env python3
"""Static safety contract for FANUC line reception.

The production implementation lives in the driver translation unit and cannot
be linked cheaply as a standalone unit test.  This guard verifies the important
fail-closed structure; the full Release build supplies the compiler check.
"""

from pathlib import Path
import re
import sys


ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "src" / "FANUCRobotDriver.cpp"


def fail(message: str) -> None:
    print(f"FAIL: {message}", file=sys.stderr)
    raise SystemExit(1)


text = SOURCE.read_text(encoding="utf-8")
match = re.search(
    r"bool\s+FanucReceiveLine\s*\([^)]*\)\s*\{(?P<body>.*?)\n\s*\}\n\s*\n\s*std::vector<std::string>\s+FanucSplit",
    text,
    re.DOTALL,
)
if not match:
    fail("FanucReceiveLine body was not found")

body = match.group("body")
required = {
    "non-positive timeout rejection": "timeoutMs <= 0",
    "monotonic absolute deadline": "std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs)",
    "deadline exhaustion check": "now >= deadline",
    "remaining deadline passed to select": "FanucWaitReadable(sock, remainingTimeoutMs)",
    "raw-byte accounting": "++rawBytesReceived",
    "raw-byte hard cap": "rawBytesReceived > FANUC_SOCKET_MAX_LINE_SIZE",
}
for label, needle in required.items():
    if needle not in body:
        fail(f"missing {label}")

if "FanucWaitReadable(sock, timeoutMs)" in body:
    fail("per-byte timeout reset is still present")

raw_count = body.find("++rawBytesReceived")
newline_accept = body.find("if (ch == '\\n')")
carriage_ignore = body.find("if (ch != '\\r')")
if min(raw_count, newline_accept, carriage_ignore) < 0:
    fail("could not prove raw-byte ordering")
if not (raw_count < newline_accept < carriage_ignore):
    fail("ignored CR/LF bytes are not counted before parsing")

print("PASS: FANUC line reception uses one bounded deadline and counts every raw byte")
