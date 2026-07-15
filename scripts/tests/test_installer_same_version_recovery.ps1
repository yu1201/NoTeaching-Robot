[CmdletBinding()]
param(
    [string]$IsccPath = ''
)

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$installerPath = Join-Path $repoRoot 'installer\QtWidgetsApplication4.iss'
$installerText = Get-Content -LiteralPath $installerPath -Raw
$tempRoot = Join-Path ([System.IO.Path]::GetTempPath()) (
    'NoTeaching-Robot same-version recovery tests ' + [Guid]::NewGuid().ToString('N')
)

function Assert-True {
    param($Condition, [Parameter(Mandatory = $true)][string]$Message)
    if (-not [bool]$Condition) {
        throw $Message
    }
}

function Get-CodeBlock {
    param(
        [Parameter(Mandatory = $true)][string]$Start,
        [Parameter(Mandatory = $true)][string]$Next
    )
    $startIndex = $installerText.IndexOf($Start, [StringComparison]::Ordinal)
    Assert-True ($startIndex -ge 0) "missing installer block: $Start"
    $nextIndex = $installerText.IndexOf($Next, $startIndex + $Start.Length, [StringComparison]::Ordinal)
    Assert-True ($nextIndex -gt $startIndex) "missing installer block boundary after: $Start"
    return $installerText.Substring($startIndex, $nextIndex - $startIndex)
}

function Get-Sha256 {
    param([Parameter(Mandatory = $true)][string]$Path)
    return (Get-FileHash -LiteralPath $Path -Algorithm SHA256).Hash.ToLowerInvariant()
}

function Get-TextSha256 {
    param([Parameter(Mandatory = $true)][string]$Value)
    $sha256 = [Security.Cryptography.SHA256]::Create()
    try {
        return [BitConverter]::ToString(
            $sha256.ComputeHash([Text.Encoding]::UTF8.GetBytes($Value))
        ).Replace('-', '').ToLowerInvariant()
    }
    finally {
        $sha256.Dispose()
    }
}

function Normalize-TestPath {
    param([Parameter(Mandatory = $true)][string]$Path)
    return [IO.Path]::GetFullPath($Path).TrimEnd('\')
}

function Test-RecoveryCredentialModel {
    param(
        [Parameter(Mandatory = $true)][string]$RegistrationRoot,
        [Parameter(Mandatory = $true)][hashtable]$Registration,
        [AllowNull()][hashtable]$Marker,
        [Parameter(Mandatory = $true)][string]$PackageVersion,
        [Parameter(Mandatory = $true)][string]$PackageDisplayName,
        [Parameter(Mandatory = $true)][string]$PackageExecutable,
        [Parameter(Mandatory = $true)][string]$SelectedInstallDirectory
    )
    if ($RegistrationRoot -cne 'HKLM64' -or $null -eq $Marker) {
        return $false
    }
    $requiredRegistration = @('Version', 'InstallDirectory', 'DisplayName', 'DisplayIcon')
    $requiredMarker = @(
        'Ready', 'Format', 'Version', 'DisplayName', 'Executable',
        'InstallDirectory', 'RecordPath', 'RecordSha256'
    )
    foreach ($field in $requiredRegistration) {
        if (-not $Registration.ContainsKey($field)) { return $false }
    }
    foreach ($field in $requiredMarker) {
        if (-not $Marker.ContainsKey($field)) { return $false }
    }
    if ($Marker.Ready -cne '1' -or
        $Marker.Format -cne 'NoTeaching-Robot-Same-Version-Recovery-v1' -or
        $Registration.Version -cne $PackageVersion -or
        $Registration.DisplayName -cne $PackageDisplayName -or
        $Marker.Version -cne $Registration.Version -or
        $Marker.DisplayName -cne $Registration.DisplayName -or
        $Marker.Executable -cne $PackageExecutable) {
        return $false
    }
    $installed = Normalize-TestPath $Registration.InstallDirectory
    $selected = Normalize-TestPath $SelectedInstallDirectory
    $markerInstall = Normalize-TestPath $Marker.InstallDirectory
    $expectedIcon = Join-Path $installed $PackageExecutable
    $expectedRecord = Join-Path (Join-Path $installed 'Data') 'ConfigStore.db.install-transaction-v1'
    if (-not [StringComparer]::OrdinalIgnoreCase.Equals($installed, $selected) -or
        -not [StringComparer]::OrdinalIgnoreCase.Equals($installed, $markerInstall) -or
        -not [StringComparer]::OrdinalIgnoreCase.Equals(
            (Normalize-TestPath $Registration.DisplayIcon),
            (Normalize-TestPath $expectedIcon)) -or
        -not [StringComparer]::OrdinalIgnoreCase.Equals(
            (Normalize-TestPath $Marker.RecordPath),
            (Normalize-TestPath $expectedRecord)) -or
        -not (Test-Path -LiteralPath $expectedIcon -PathType Leaf) -or
        -not (Test-Path -LiteralPath $expectedRecord -PathType Leaf) -or
        [string]$Marker.RecordSha256 -cnotmatch '^[0-9a-f]{64}$') {
        return $false
    }
    return (Get-Sha256 $expectedRecord) -ceq [string]$Marker.RecordSha256
}

function Resolve-IsccPath {
    if (-not [string]::IsNullOrWhiteSpace($IsccPath)) {
        return (Resolve-Path -LiteralPath $IsccPath).Path
    }
    $command = Get-Command iscc.exe -ErrorAction SilentlyContinue
    if ($null -ne $command) {
        return $command.Source
    }
    foreach ($candidate in @(
            'D:\SoftWare\Inno Setup 6\ISCC.exe',
            'C:\Program Files (x86)\Inno Setup 6\ISCC.exe',
            'C:\Program Files\Inno Setup 6\ISCC.exe')) {
        if (Test-Path -LiteralPath $candidate -PathType Leaf) {
            return $candidate
        }
    }
    throw 'Inno Setup 6 ISCC.exe was not found.'
}

function Invoke-ChannelCompile {
    param(
        [Parameter(Mandatory = $true)][ValidateSet('neutral', 'brand')][string]$Channel,
        [Parameter(Mandatory = $true)][string]$Compiler
    )
    $channelRoot = Join-Path $tempRoot $Channel
    $payload = Join-Path $channelRoot 'payload'
    $tools = Join-Path $channelRoot 'tools'
    $output = Join-Path $channelRoot 'output'
    [IO.Directory]::CreateDirectory($payload) | Out-Null
    [IO.Directory]::CreateDirectory($tools) | Out-Null
    [IO.Directory]::CreateDirectory($output) | Out-Null

    if ($Channel -ceq 'brand') {
        $appName = 'HK-Pathlynx-CORPLA'
        $exeName = 'HK-Pathlynx-CORPLA.exe'
    }
    else {
        $appName = 'NoTeaching-Robot'
        $exeName = 'QtWidgetsApplication4.exe'
    }
    [IO.File]::WriteAllText((Join-Path $payload $exeName), 'compile-only payload')
    [IO.File]::WriteAllText((Join-Path $tools 'ConfigMigrate.exe'), 'compile-only tool')
    [IO.File]::WriteAllText((Join-Path $tools 'ConfigMigrate_Run.cmd'), '@exit /b 0')
    [IO.File]::WriteAllText((Join-Path $tools 'ConfigMigrate_Install.ps1'), 'exit 0')
    Copy-Item -LiteralPath (Join-Path $repoRoot 'icons\app.ico') `
        -Destination (Join-Path $channelRoot 'app.ico')

    $source = $installerText
    $source = $source.Replace(
        '#define MyAppName "NoTeaching-Robot"',
        '#define MyAppName "' + $appName + '"')
    $source = $source.Replace(
        '#define MyAppExeName "QtWidgetsApplication4.exe"',
        '#define MyAppExeName "' + $exeName + '"')
    $source = $source.Replace(
        '#define MySourceDir "..\dist\QtWidgetsApplication4"',
        '#define MySourceDir "' + $payload + '"')
    $source = $source.Replace('SetupIconFile=..\icons\app.ico', 'SetupIconFile=' + (Join-Path $channelRoot 'app.ico'))
    $source = $source.Replace('Source: "..\dist\tools\', 'Source: "' + $tools + '\')
    $scriptPath = Join-Path $channelRoot 'QtWidgetsApplication4.iss'
    [IO.File]::WriteAllText($scriptPath, $source, [Text.UTF8Encoding]::new($false))

    & $Compiler /Q "/O$output" "/Fsame-version-$Channel" '/DMyAppVersion=2099.01.02.0304' $scriptPath
    Assert-True ($LASTEXITCODE -eq 0) "$Channel Inno compile failed"
    $compiled = Join-Path $output "same-version-$Channel.exe"
    Assert-True ((Test-Path -LiteralPath $compiled -PathType Leaf) -and
        (Get-Item -LiteralPath $compiled).Length -gt 0) "$Channel Inno output is missing"
}

try {
    [IO.Directory]::CreateDirectory($tempRoot) | Out-Null

    # Structural contract: HKLM64 marker is committed last, invalidated first,
    # bound to every identity field, and independently revalidated in both gates.
    $clearBlock = Get-CodeBlock 'function ClearSameVersionRecoveryMarker' 'function TryReadSameVersionRecoveryMarker'
    $validateBlock = Get-CodeBlock 'function ValidateSameVersionRecoveryMarker' 'function PersistSameVersionRecoveryMarker'
    $persistBlock = Get-CodeBlock 'function PersistSameVersionRecoveryMarker' 'function IsCanonicalUpgradeBackupName'
    $prepareBlock = Get-CodeBlock 'function PrepareToInstall' 'function FinalizeDeferredCredentialScrub'
    $postInstallBlock = Get-CodeBlock 'procedure CurStepChanged' 'function GetCustomSetupExitCode'
    $initializeBlock = Get-CodeBlock 'function InitializeSetup' 'function VcRedistInstallerExists'

    Assert-True ($validateBlock.Contains('(RootKey <> HKLM64)') -and
        $validateBlock.Contains('(UninstallKey <> GetApplicationUninstallKey)')) `
        'recovery validation is not fixed to the same HKLM64 uninstall key'
    Assert-True ($validateBlock.Contains("MarkerExecutableName <> '{#MyAppExeName}'") -and
        $validateBlock.Contains('MarkerDisplayName <> InstalledDisplayName') -and
        $validateBlock.Contains('MarkerVersion <> InstalledVersion') -and
        $validateBlock.Contains('GetSHA256OfFile(ExpectedRecordPath)')) `
        'recovery marker does not bind channel, version, path, and transaction bytes'
    Assert-True ($persistBlock.Contains(
            'RecordSha256 <> DatabaseClosingExpectedRecordSha256') -and
        $prepareBlock.Contains('TryGetPendingTransactionRecordSha256(') -and
        $postInstallBlock.Contains('TryGetPendingTransactionRecordSha256(')) `
        'ssPostInstall can bless a transaction record not hash-bound by the verified prepare/finalize state'
    Assert-True ($initializeBlock.Contains('ValidateSameVersionRecoveryMarker(') -and
        $prepareBlock.Contains('ValidateSameVersionRecoveryMarker(') -and
        $prepareBlock.Contains('SameVersionRecoveryRecordSha256')) `
        'InitializeSetup and PrepareToInstall do not independently validate the same marker'
    Assert-True ($prepareBlock.Contains('SameVersionRecoveryRegistryRoot') -and
        $prepareBlock.Contains('SameVersionRecoveryUninstallKey') -and
        $prepareBlock.Contains('RemoveBackslashUnlessRoot(ExpandFileName(MigrationDataPath))')) `
        'PrepareToInstall does not pin registry root/key and /DIR'
    Assert-True (-not $initializeBlock.Contains('FileExists(PendingRecordPath)')) `
        'same-version eligibility still accepts a Data-file existence check as a credential'
    $readyWrite = $persistBlock.IndexOf('RecoveryReadyValueName, ''1''')
    $shaWrite = $persistBlock.IndexOf('RecoveryRecordSha256ValueName, RecordSha256')
    $readBack = $persistBlock.IndexOf('ValidateSameVersionRecoveryMarker(')
    Assert-True ($shaWrite -ge 0 -and $readyWrite -gt $shaWrite -and $readBack -gt $readyWrite) `
        'protected marker Ready is not the last write followed by full read-back'
    $readyDelete = $clearBlock.IndexOf('RecoveryReadyValueName')
    $formatDelete = $clearBlock.IndexOf('RecoveryFormatValueName')
    Assert-True ($readyDelete -ge 0 -and $formatDelete -gt $readyDelete) `
        'protected marker Ready is not invalidated before deleting binding fields'
    Assert-True ($postInstallBlock.Contains('if DatabaseClosingFailed then') -and
        $postInstallBlock.Contains('PersistSameVersionRecoveryMarker') -and
        ([regex]::Matches(
            $postInstallBlock,
            [regex]::Escape('DatabaseClosingFailed := True;')).Count -ge 2)) `
        'marker persistence is not limited to ssPostInstall database commit/finalize failures'
    Assert-True ([regex]::Matches(
        $installerText,
        [regex]::Escape('PersistSameVersionRecoveryMarker')).Count -eq 2) `
        'protected marker is persisted from a path other than its definition and ssPostInstall failure branch'
    Assert-True ([regex]::Matches(
        $installerText,
        [regex]::Escape("RollbackStatus <> 'OK:PENDING_PUBLISHED_PRESERVED'")).Count -eq 2) `
        'full rollback does not clear the marker or published-preserved recovery incorrectly clears it'

    # State model: a real protected marker succeeds; Data-only records, including
    # a syntactically self-consistent transaction, never become credentials.
    $installDirectory = Join-Path $tempRoot 'installed neutral'
    $dataDirectory = Join-Path $installDirectory 'Data'
    [IO.Directory]::CreateDirectory($dataDirectory) | Out-Null
    $exePath = Join-Path $installDirectory 'QtWidgetsApplication4.exe'
    $recordPath = Join-Path $dataDirectory 'ConfigStore.db.install-transaction-v1'
    [IO.File]::WriteAllText($exePath, 'installed executable')
    [IO.File]::WriteAllText($recordPath, "real failed-install transaction`n")
    $registration = @{
        Version = '2099.01.02.0304'
        InstallDirectory = $installDirectory
        DisplayName = 'NoTeaching-Robot version 2099.01.02.0304'
        DisplayIcon = $exePath
    }
    $realMarker = @{
        Ready = '1'
        Format = 'NoTeaching-Robot-Same-Version-Recovery-v1'
        Version = $registration.Version
        DisplayName = $registration.DisplayName
        Executable = 'QtWidgetsApplication4.exe'
        InstallDirectory = $installDirectory
        RecordPath = $recordPath
        RecordSha256 = Get-Sha256 $recordPath
    }
    Assert-True (Test-RecoveryCredentialModel 'HKLM64' $registration $realMarker `
        $registration.Version $registration.DisplayName 'QtWidgetsApplication4.exe' $installDirectory) `
        'a complete HKLM64-bound failure marker was rejected'
    Assert-True (-not (Test-RecoveryCredentialModel 'HKLM64' $registration $null `
        $registration.Version $registration.DisplayName 'QtWidgetsApplication4.exe' $installDirectory)) `
        'missing marker was accepted'

    foreach ($fakeContent in @('', 'random', 'FORMAT=invalid')) {
        [IO.File]::WriteAllText($recordPath, $fakeContent)
        Assert-True (-not (Test-RecoveryCredentialModel 'HKLM64' $registration $null `
            $registration.Version $registration.DisplayName 'QtWidgetsApplication4.exe' $installDirectory)) `
            'a Data-only empty/random record was accepted'
    }
    $noopPayload = "FORMAT=NoTeaching-Robot-Install-Transaction-v1`nMODE=NOOP_CURRENT`nCURRENT_SHA256=" + ('a' * 64)
    $selfConsistentNoop = $noopPayload + "`nPAYLOAD_SHA256=" + (Get-TextSha256 $noopPayload) + "`n"
    [IO.File]::WriteAllText($recordPath, $selfConsistentNoop, [Text.UTF8Encoding]::new($false))
    Assert-True (-not (Test-RecoveryCredentialModel 'HKLM64' $registration $null `
        $registration.Version $registration.DisplayName 'QtWidgetsApplication4.exe' $installDirectory)) `
        'a self-consistent forged NOOP_CURRENT record was treated as a recovery credential'

    [IO.File]::WriteAllText($recordPath, "real failed-install transaction`n")
    $realMarker.RecordSha256 = Get-Sha256 $recordPath
    Assert-True (-not (Test-RecoveryCredentialModel 'HKCU' $registration $realMarker `
        $registration.Version $registration.DisplayName 'QtWidgetsApplication4.exe' $installDirectory)) `
        'an HKCU marker was accepted'
    Assert-True (-not (Test-RecoveryCredentialModel 'HKLM64' $registration $realMarker `
        $registration.Version 'HK-Pathlynx-CORPLA version 2099.01.02.0304' 'HK-Pathlynx-CORPLA.exe' $installDirectory)) `
        'the opposite channel reused the neutral marker'
    Assert-True (-not (Test-RecoveryCredentialModel 'HKLM64' $registration $realMarker `
        $registration.Version $registration.DisplayName 'QtWidgetsApplication4.exe' (Join-Path $tempRoot 'wrong dir'))) `
        '/DIR override escaped the marker-bound install directory'
    [IO.File]::AppendAllText($recordPath, 'tamper')
    Assert-True (-not (Test-RecoveryCredentialModel 'HKLM64' $registration $realMarker `
        $registration.Version $registration.DisplayName 'QtWidgetsApplication4.exe' $installDirectory)) `
        'transaction digest drift was accepted'

    $compiler = Resolve-IsccPath
    Invoke-ChannelCompile neutral $compiler
    Invoke-ChannelCompile brand $compiler
    Write-Host 'PASS: HKLM same-version recovery gate, fake-record matrix, /DIR/channel binding, and neutral/brand compile'
}
finally {
    $resolvedTempBase = [IO.Path]::GetFullPath([IO.Path]::GetTempPath()).TrimEnd('\') + '\'
    $resolvedTempRoot = [IO.Path]::GetFullPath($tempRoot)
    if ($resolvedTempRoot.StartsWith($resolvedTempBase, [StringComparison]::OrdinalIgnoreCase) -and
        (Test-Path -LiteralPath $resolvedTempRoot)) {
        Remove-Item -LiteralPath $resolvedTempRoot -Recurse -Force -ErrorAction SilentlyContinue
    }
}
