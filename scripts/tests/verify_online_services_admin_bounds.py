#!/usr/bin/env python3
"""Static regression guard for bounded Online Services admin replies."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SOURCE = (ROOT / "src" / "OnlineServicesDialog.cpp").read_text(
    encoding="utf-8", errors="strict"
)

start = SOURCE.index("void OnlineServicesDialog::AdminRequest")
end = SOURCE.index("void OnlineServicesDialog::RefreshServerStats", start)
body = SOURCE[start:end]

required = (
    "kMaximumAdminResponseBytes",
    "setReadBufferSize(kMaximumAdminResponseBytes + 1)",
    "QNetworkReply::readyRead",
    "adminResponseTooLarge",
    "response.size() > kMaximumAdminResponseBytes",
    "reply->bytesAvailable() > 0",
    "reply->abort()",
)
for token in required:
    assert token in body, f"missing bounded admin-response guard: {token}"

assert "reply->readAll()" not in body, "admin response must not be accumulated with readAll()"

print("Online Services admin response bounds static regression: PASS")
