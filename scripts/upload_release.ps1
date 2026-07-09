# 把安装包发布到自建 OTA 服务器（156.239.225.105，端口 8090；域名 xiaomomoyun.cn 解析切换后可换回域名）。
# 用法（在 QtWidgetsApplication4/ 下，先跑 build_installer.ps1 出包）：
#   powershell -ExecutionPolicy Bypass -File scripts\upload_release.ps1 `
#       -Version 2026.07.09.1000 -Channel neutral -Notes "修复xxx；新增yyy"
#   品牌包： -Channel brand （自动找 HK-Pathlynx-CORPLA-Setup-v<版本>.exe / HK-Pathlynx-CORPLA.exe）
# 默认同时生成并上架【增量补丁】（仅主程序 exe 的 zip，约 2MB，客户端优先走增量）；
#   Qt/OpenCV/SDK DLL 有变化的版本请加 -NoPatch 强制全量（或 -PatchFiles 附带变更 DLL）。
# 依赖：Windows 自带 OpenSSH（scp），按提示输入服务器 root 密码（两次上传各一次）。
param(
    [Parameter(Mandatory = $true)][string]$Version,
    [ValidateSet("neutral", "brand")][string]$Channel = "neutral",
    [string]$Notes = "",
    [switch]$NoPatch,
    [string[]]$PatchFiles = @(),   # 额外并入补丁 zip 的文件（dist 包内相对路径）
    [string]$ServerHost = "156.239.225.105",
    [string]$SshUser = "root"
)

$ErrorActionPreference = "Stop"
$repoRoot = Split-Path -Parent $PSScriptRoot

$baseName = if ($Channel -eq "brand") { "HK-Pathlynx-CORPLA-Setup-v$Version" } else { "NoTeaching-Robot-Setup-v$Version" }
$exeName = if ($Channel -eq "brand") { "HK-Pathlynx-CORPLA.exe" } else { "QtWidgetsApplication4.exe" }
$installer = Join-Path $repoRoot "dist\installer\$baseName.exe"
if (-not (Test-Path $installer)) {
    throw "找不到安装包：$installer（先运行 build_installer.ps1）"
}

Write-Host "计算安装包 SHA256..."
$sha256 = (Get-FileHash -Algorithm SHA256 $installer).Hash.ToLower()
$size = (Get-Item $installer).Length

$manifest = [ordered]@{
    version = $Version
    file    = "$baseName.exe"
    sha256  = $sha256
    size    = $size
    notes   = $Notes
}

# —— 增量补丁：dist 包里的主程序（+可选附加文件）打 zip ——
$patchPath = $null
if (-not $NoPatch) {
    $distDir = Join-Path $repoRoot "dist\QtWidgetsApplication4"
    $mainExe = Join-Path $distDir $exeName
    if (-not (Test-Path $mainExe)) {
        throw "找不到 $mainExe（补丁需要 dist 包，先运行 build_release_package.ps1；只发全量请加 -NoPatch）"
    }
    $patchName = ($baseName -replace "-Setup-", "-Patch-") + ".zip"
    $patchPath = Join-Path $env:TEMP $patchName
    $patchItems = @($mainExe) + ($PatchFiles | ForEach-Object { Join-Path $distDir $_ })
    Compress-Archive -Path $patchItems -DestinationPath $patchPath -CompressionLevel Optimal -Force
    $patchSha = (Get-FileHash -Algorithm SHA256 $patchPath).Hash.ToLower()
    $patchSize = (Get-Item $patchPath).Length
    $manifest["patch"] = [ordered]@{
        file   = $patchName
        sha256 = $patchSha
        size   = $patchSize
    }
    Write-Host "增量补丁：$patchName（$([math]::Round($patchSize/1MB,1)) MB，含 $($patchItems.Count) 个文件）"
}

$manifestJson = $manifest | ConvertTo-Json -Depth 3
$manifestPath = Join-Path $env:TEMP "latest.json"
# 客户端按 UTF-8 解析（QJsonDocument），必须无 BOM
[System.IO.File]::WriteAllText($manifestPath, $manifestJson, (New-Object System.Text.UTF8Encoding($false)))

$remoteDir = "/var/www/ota/$Channel"
$files = @($installer)
if ($patchPath) { $files += $patchPath }
$files += $manifestPath
Write-Host "上传 $($files.Count) 个文件到 ${ServerHost}:${remoteDir} ..."
scp @files "${SshUser}@${ServerHost}:${remoteDir}/"
if ($LASTEXITCODE -ne 0) { throw "上传失败" }

Write-Host ""
Write-Host "发布完成 ✓  通道=$Channel 版本=$Version $(if ($patchPath) { '（含增量补丁）' })"
Write-Host "验证: curl http://${ServerHost}:8090/ota/$Channel/latest.json"
