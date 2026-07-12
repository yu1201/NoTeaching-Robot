#!/usr/bin/env python3
"""Keep every WinINet FTP connection and data operation explicitly bounded."""

from pathlib import Path
import re
import sys


ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "src" / "FTPClient.cpp"


def fail(message: str) -> None:
    print(f"FAIL: {message}", file=sys.stderr)
    raise SystemExit(1)


text = SOURCE.read_text(encoding="utf-8")
required = {
    "bounded connect timeout": "INTERNET_OPTION_CONNECT_TIMEOUT",
    "bounded send timeout": "INTERNET_OPTION_SEND_TIMEOUT",
    "bounded receive timeout": "INTERNET_OPTION_RECEIVE_TIMEOUT",
    "bounded FTP data send timeout": "INTERNET_OPTION_DATA_SEND_TIMEOUT",
    "bounded FTP data receive timeout": "INTERNET_OPTION_DATA_RECEIVE_TIMEOUT",
    "root handle configured before login": "ConfigureFtpInternetTimeouts(m_hInternet)",
}
for label, token in required.items():
    if token not in text:
        fail(f"missing {label}")

connect_match = re.search(
    r'm_hInternet\s*=\s*InternetOpenA\(.*?m_hFtpSession\s*=\s*InternetConnectA\(',
    text,
    re.DOTALL,
)
if not connect_match:
    fail("FTP login sequence not found")
if "ConfigureFtpInternetTimeouts(m_hInternet)" not in connect_match.group(0):
    fail("connect timeout is not installed before InternetConnectA")

if "已拒绝建立无界连接" not in text:
    fail("timeout setup failures do not fail closed")

print("PASS: WinINet FTP connect/send/receive calls have explicit fail-closed timeouts")
