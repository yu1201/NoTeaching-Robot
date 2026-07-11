$ErrorActionPreference = "Stop"

throw @"
scripts/upload_release.ps1 已永久停用：它是旧的单通道发布入口，无法满足双通道、
payload_history、主机密钥、原子 latest 和远端回读门禁。

请使用：
  python scripts/upload_ota.py prepare-dual --help
  python scripts/upload_ota.py publish-dual --help

先离线生成并审核双通道候选报告，之后才能执行 publish-dual。
"@
