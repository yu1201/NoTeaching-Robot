$script:ReleaseGateExpectedAppId = "A5A7E2A0-8226-40BB-B126-94C5D298B3CF"
$script:ReleaseGateSchemaVersion = 1
$script:ReleaseGateCommonPath = $PSCommandPath
$releaseGitCandidate = if ([string]::IsNullOrWhiteSpace($env:NO_TEACHING_RELEASE_GIT_EXE)) {
    "C:\Program Files\Git\cmd\git.exe"
}
else {
    $env:NO_TEACHING_RELEASE_GIT_EXE
}
if (-not [System.IO.Path]::IsPathRooted($releaseGitCandidate)) {
    throw "Release Git executable must use an absolute path."
}
$script:ReleaseGateGitExe = [System.IO.Path]::GetFullPath($releaseGitCandidate)
if (-not (Test-Path -LiteralPath $script:ReleaseGateGitExe -PathType Leaf) `
    -or [System.IO.Path]::GetFileName($script:ReleaseGateGitExe) -cne "git.exe") {
    throw "Release Git executable is missing or has an unexpected name."
}
$releaseGitItem = Get-Item -LiteralPath $script:ReleaseGateGitExe -Force
if (($releaseGitItem.Attributes -band [System.IO.FileAttributes]::ReparsePoint) -ne 0) {
    throw "Release Git executable must not be a reparse point."
}
$releaseGitSignature = Get-AuthenticodeSignature -LiteralPath $script:ReleaseGateGitExe
$releaseGitSubject = if ($null -ne $releaseGitSignature.SignerCertificate) {
    [string]$releaseGitSignature.SignerCertificate.Subject
}
else { "" }
if ([string]$releaseGitSignature.Status -cne "Valid" `
    -or ($releaseGitSubject -notmatch '(?i)Johannes Schindelin|Git for Windows')) {
    throw "Release Git executable does not have the expected valid Authenticode publisher."
}
$script:ReleaseGateGitSha256 = (Get-FileHash -LiteralPath $script:ReleaseGateGitExe -Algorithm SHA256).Hash.ToLowerInvariant()

function Get-ReleaseChannelSpec {
    param([Parameter(Mandatory = $true)][ValidateSet("neutral", "brand")][string]$Channel)

    if ($Channel -eq "brand") {
        return [pscustomobject]@{
            Channel            = "brand"
            Branch             = "hk-pathlynx-corpla"
            AppName            = "HK-Pathlynx-CORPLA"
            ExeName            = "HK-Pathlynx-CORPLA.exe"
            OutputPrefix       = "HK-Pathlynx-CORPLA-Setup-v"
            RequiresBranding   = $true
        }
    }

    return [pscustomobject]@{
        Channel            = "neutral"
        Branch             = "main"
        AppName            = "NoTeaching-Robot"
        ExeName            = "QtWidgetsApplication4.exe"
        OutputPrefix       = "NoTeaching-Robot-Setup-v"
        RequiresBranding   = $false
    }
}

function Assert-ReleaseVersion {
    param([Parameter(Mandatory = $true)][string]$AppVersion)

    if ($AppVersion -notmatch '^\d{4}\.\d{2}\.\d{2}\.\d{4}$') {
        throw "AppVersion must be explicit and match yyyy.MM.dd.HHmm: $AppVersion"
    }
    $parsed = [DateTime]::MinValue
    if (-not [DateTime]::TryParseExact(
        $AppVersion,
        "yyyy.MM.dd.HHmm",
        [Globalization.CultureInfo]::InvariantCulture,
        [Globalization.DateTimeStyles]::None,
        [ref]$parsed)) {
        throw "AppVersion is not a valid calendar date/time: $AppVersion"
    }
}

function Resolve-ReleaseGatePath {
    param([Parameter(Mandatory = $true)][string]$Path)

    return [System.IO.Path]::GetFullPath($Path).TrimEnd('\', '/')
}

function Test-ReleaseGateSamePath {
    param(
        [Parameter(Mandatory = $true)][string]$Left,
        [Parameter(Mandatory = $true)][string]$Right
    )
    return [string]::Equals(
        (Resolve-ReleaseGatePath $Left),
        (Resolve-ReleaseGatePath $Right),
        [System.StringComparison]::OrdinalIgnoreCase)
}

function Get-ReleaseFileSha256 {
    param([Parameter(Mandatory = $true)][string]$Path)

    if (-not (Test-Path -LiteralPath $Path -PathType Leaf)) {
        throw "Required file does not exist: $Path"
    }
    return (Get-FileHash -LiteralPath $Path -Algorithm SHA256).Hash.ToLowerInvariant()
}

function Assert-ReleaseExternalTool {
    param(
        [Parameter(Mandatory = $true)][string]$Path,
        [Parameter(Mandatory = $true)][string]$ExpectedSha256,
        [Parameter(Mandatory = $true)][string]$ExpectedFileName,
        [Parameter(Mandatory = $true)][string]$PublisherPattern
    )

    if (-not [System.IO.Path]::IsPathRooted($Path) `
        -or $ExpectedSha256 -notmatch '^[0-9a-f]{64}$') {
        throw "Release external tool path/hash is not canonical."
    }
    $resolved = [System.IO.Path]::GetFullPath($Path)
    if (-not (Test-Path -LiteralPath $resolved -PathType Leaf) `
        -or [System.IO.Path]::GetFileName($resolved) -cne $ExpectedFileName) {
        throw "Release external tool is missing or has an unexpected name: $resolved"
    }
    $cursorPath = $resolved
    while (-not [string]::IsNullOrWhiteSpace($cursorPath)) {
        $cursor = Get-Item -LiteralPath $cursorPath -Force
        if (($cursor.Attributes -band [System.IO.FileAttributes]::ReparsePoint) -ne 0) {
            throw "Release external tool path contains a reparse point: $($cursor.FullName)"
        }
        $parentPath = Split-Path -Parent $cursorPath
        if ([string]::IsNullOrWhiteSpace($parentPath) -or $parentPath -ceq $cursorPath) {
            break
        }
        $cursorPath = $parentPath
    }
    $signature = Get-AuthenticodeSignature -LiteralPath $resolved
    $subject = if ($null -ne $signature.SignerCertificate) {
        [string]$signature.SignerCertificate.Subject
    }
    else { "" }
    if ([string]$signature.Status -cne "Valid" -or $subject -notmatch $PublisherPattern) {
        throw "Release external tool publisher is not trusted: $resolved"
    }
    if ((Get-ReleaseFileSha256 $resolved) -cne $ExpectedSha256) {
        throw "Release external tool hash changed: $resolved"
    }
    return $resolved
}

function Set-ReleasePythonTool {
    param(
        [Parameter(Mandatory = $true)][string]$PythonExecutable,
        [Parameter(Mandatory = $true)][string]$PythonSha256
    )
    $script:ReleasePythonExecutable = Assert-ReleaseExternalTool `
        -Path $PythonExecutable `
        -ExpectedSha256 $PythonSha256 `
        -ExpectedFileName "python.exe" `
        -PublisherPattern '(?i)Python Software Foundation'
    $script:ReleasePythonSha256 = $PythonSha256
}

function Get-ReleasePythonTool {
    if ([string]::IsNullOrWhiteSpace($script:ReleasePythonExecutable) `
        -or [string]::IsNullOrWhiteSpace($script:ReleasePythonSha256)) {
        throw "Release Python tool was not explicitly initialized."
    }
    $path = Assert-ReleaseExternalTool `
        -Path $script:ReleasePythonExecutable `
        -ExpectedSha256 $script:ReleasePythonSha256 `
        -ExpectedFileName "python.exe" `
        -PublisherPattern '(?i)Python Software Foundation'
    return [pscustomobject]@{ path = $path; sha256 = $script:ReleasePythonSha256 }
}

function Get-ReleaseRelativePath {
    param(
        [Parameter(Mandatory = $true)][string]$Root,
        [Parameter(Mandatory = $true)][string]$Path
    )

    $rootFull = Resolve-ReleaseGatePath $Root
    $pathFull = [System.IO.Path]::GetFullPath($Path)
    $prefix = $rootFull + [System.IO.Path]::DirectorySeparatorChar
    if (-not $pathFull.StartsWith($prefix, [System.StringComparison]::OrdinalIgnoreCase)) {
        throw "Path escapes inventory root: $pathFull (root: $rootFull)"
    }
    return $pathFull.Substring($prefix.Length).Replace('\', '/')
}

function Get-ReleaseFileInventory {
    param([Parameter(Mandatory = $true)][string]$Root)

    $rootFull = Resolve-ReleaseGatePath $Root
    if (-not (Test-Path -LiteralPath $rootFull -PathType Container)) {
        throw "Inventory root does not exist: $rootFull"
    }

    $items = @()
    $seenPaths = @{}
    Get-ChildItem -LiteralPath $rootFull -Recurse -Force -File | ForEach-Object {
        $relative = Get-ReleaseRelativePath -Root $rootFull -Path $_.FullName
        $pathKey = $relative.ToLowerInvariant()
        if ($seenPaths.ContainsKey($pathKey)) {
            throw "Case-colliding release inventory paths are forbidden: $relative"
        }
        $seenPaths[$pathKey] = $true
        $items += [pscustomobject]@{
            path   = $relative
            size   = [int64]$_.Length
            sha256 = Get-ReleaseFileSha256 -Path $_.FullName
        }
    }
    return @($items | Sort-Object path)
}

function Assert-ReleaseInventoryMatches {
    param(
        [Parameter(Mandatory = $true)]$Expected,
        [Parameter(Mandatory = $true)]$Actual,
        [string]$Context = "release inventory"
    )

    $expectedItems = @($Expected)
    $actualItems = @($Actual)
    if ($expectedItems.Count -ne $actualItems.Count) {
        throw "$Context file count changed: expected $($expectedItems.Count), actual $($actualItems.Count)."
    }
    for ($i = 0; $i -lt $expectedItems.Count; ++$i) {
        $left = $expectedItems[$i]
        $right = $actualItems[$i]
        if ([string]$left.path -cne [string]$right.path `
            -or [int64]$left.size -ne [int64]$right.size `
            -or [string]$left.sha256 -cne [string]$right.sha256) {
            throw ("{0} mismatch at item {1}: expected {2} ({3}, {4}), actual {5} ({6}, {7})." -f `
                $Context, $i, $left.path, $left.size, $left.sha256, $right.path, $right.size, $right.sha256)
        }
    }
}

function Write-ReleaseGateJson {
    param(
        [Parameter(Mandatory = $true)]$Value,
        [Parameter(Mandatory = $true)][string]$Path
    )

    $parent = Split-Path -Parent $Path
    if (-not [string]::IsNullOrWhiteSpace($parent)) {
        New-Item -ItemType Directory -Path $parent -Force | Out-Null
    }
    $fullPath = [System.IO.Path]::GetFullPath($Path)
    $tempPath = $fullPath + ".tmp." + [Guid]::NewGuid().ToString("N")
    $backupPath = $fullPath + ".bak." + [Guid]::NewGuid().ToString("N")
    $json = $Value | ConvertTo-Json -Depth 12
    try {
        [System.IO.File]::WriteAllText(
            $tempPath,
            $json + [Environment]::NewLine,
            (New-Object System.Text.UTF8Encoding($false)))
        if (Test-Path -LiteralPath $fullPath -PathType Leaf) {
            [System.IO.File]::Replace($tempPath, $fullPath, $backupPath, $true)
        }
        else {
            [System.IO.File]::Move($tempPath, $fullPath)
        }
    }
    finally {
        if (Test-Path -LiteralPath $tempPath) {
            Remove-Item -LiteralPath $tempPath -Force
        }
        if (Test-Path -LiteralPath $backupPath) {
            Remove-Item -LiteralPath $backupPath -Force
        }
    }
}

function New-ReleaseGatePendingReport {
    param(
        [Parameter(Mandatory = $true)][string]$Path,
        [Parameter(Mandatory = $true)][string]$Kind,
        [Parameter(Mandatory = $true)][string]$AppVersion,
        [Parameter(Mandatory = $true)][ValidateSet("neutral", "brand")][string]$Channel
    )
    $commonPath = $script:ReleaseGateCommonPath
    $pending = [ordered]@{
        schemaVersion   = $script:ReleaseGateSchemaVersion
        kind            = $Kind
        status          = "pending"
        runId           = [Guid]::NewGuid().ToString("D")
        startedAtUtc    = [DateTime]::UtcNow.ToString("o")
        version         = $AppVersion
        channel         = $Channel
        gateScriptSha256 = Get-ReleaseFileSha256 $commonPath
    }
    Write-ReleaseGateJson -Value $pending -Path $Path
    return $pending.runId
}

function Read-ReleaseGateJson {
    param([Parameter(Mandatory = $true)][string]$Path)

    if (-not (Test-Path -LiteralPath $Path -PathType Leaf)) {
        throw "Release gate report does not exist: $Path"
    }
    try {
        return (Get-Content -LiteralPath $Path -Raw -Encoding UTF8 | ConvertFrom-Json)
    }
    catch {
        throw "Release gate report is not valid JSON: $Path. $($_.Exception.Message)"
    }
}

function Invoke-ReleaseGit {
    param(
        [Parameter(Mandatory = $true)][string]$RepoRoot,
        [Parameter(Mandatory = $true)][string[]]$Arguments
    )

    if ((Get-FileHash -LiteralPath $script:ReleaseGateGitExe -Algorithm SHA256).Hash.ToLowerInvariant() `
        -cne $script:ReleaseGateGitSha256) {
        throw "Release Git executable changed during gate execution."
    }
    $output = @(& $script:ReleaseGateGitExe -C $RepoRoot @Arguments 2>&1)
    if ($LASTEXITCODE -ne 0) {
        throw "git $($Arguments -join ' ') failed in ${RepoRoot}: $($output -join [Environment]::NewLine)"
    }
    return @($output | ForEach-Object { [string]$_ })
}

function Get-ReleaseGitPath {
    param(
        [Parameter(Mandatory = $true)][string]$RepoRoot,
        [Parameter(Mandatory = $true)][ValidateSet("common", "worktree")][string]$Kind
    )

    $argument = if ($Kind -eq "common") { "--git-common-dir" } else { "--git-dir" }
    $value = (Invoke-ReleaseGit -RepoRoot $RepoRoot -Arguments @("rev-parse", "--path-format=absolute", $argument) |
        Select-Object -Last 1).Trim()
    if ([string]::IsNullOrWhiteSpace($value)) {
        throw "Git returned an empty $Kind directory for release root: $RepoRoot"
    }
    if (-not [System.IO.Path]::IsPathRooted($value)) {
        $value = Join-Path $RepoRoot $value
    }
    return (Resolve-ReleaseGatePath $value)
}

function Assert-GitTrackedReleaseFile {
    param(
        [Parameter(Mandatory = $true)][string]$RepoRoot,
        [Parameter(Mandatory = $true)][string]$RelativePath
    )

    $normalized = $RelativePath.Replace('\', '/')
    if ($normalized.StartsWith("/", [System.StringComparison]::Ordinal) `
        -or $normalized -match '(^|/)\.\.(/|$)') {
        throw "Release asset path escapes the repository: $RelativePath"
    }
    $tracked = @(Invoke-ReleaseGit -RepoRoot $RepoRoot -Arguments @("ls-files", "--error-unmatch", "--", $normalized))
    if ($tracked.Count -ne 1 -or ([string]$tracked[0]).Replace('\', '/') -cne $normalized) {
        throw "Release asset is not tracked exactly by Git: $RelativePath"
    }
    $fullPath = Join-Path $RepoRoot $normalized.Replace('/', '\')
    if (-not (Test-Path -LiteralPath $fullPath -PathType Leaf)) {
        throw "Tracked release asset is missing from the worktree: $RelativePath"
    }
    return (Resolve-ReleaseGatePath $fullPath)
}

function Split-InnoRecordFields {
    param([Parameter(Mandatory = $true)][string]$Line)

    $parts = New-Object System.Collections.Generic.List[string]
    $builder = New-Object System.Text.StringBuilder
    $insideQuotes = $false
    foreach ($character in $Line.ToCharArray()) {
        if ($character -eq '"') {
            $insideQuotes = -not $insideQuotes
        }
        if ($character -eq ';' -and -not $insideQuotes) {
            $parts.Add($builder.ToString())
            $builder.Clear() | Out-Null
            continue
        }
        $builder.Append($character) | Out-Null
    }
    if ($insideQuotes) {
        throw "Unterminated quoted value in installer [Files] record: $Line"
    }
    $parts.Add($builder.ToString())
    return @($parts)
}

function Get-InnoQuotedValue {
    param(
        [Parameter(Mandatory = $true)][string]$Value,
        [Parameter(Mandatory = $true)][string]$FieldName
    )

    $trimmed = $Value.Trim()
    if ($trimmed -notmatch '^"([^"\r\n]*)"$') {
        throw "Installer $FieldName must be one non-empty quoted value: $Value"
    }
    if ([string]::IsNullOrWhiteSpace($Matches[1])) {
        throw "Installer $FieldName must not be empty."
    }
    return $Matches[1]
}

function Get-InnoFilesRecords {
    param([Parameter(Mandatory = $true)][string]$Text)

    $records = @()
    $section = ""
    $filesSectionCount = 0
    foreach ($rawLine in ($Text -split '\r?\n')) {
        $line = $rawLine.Trim()
        if ($line -match '^\[([^\]]+)\]$') {
            $section = $Matches[1].Trim()
            if ($section -ieq "Files") {
                ++$filesSectionCount
            }
            continue
        }
        if ($section -ine "Files" -or [string]::IsNullOrWhiteSpace($line) -or $line.StartsWith(";")) {
            continue
        }
        if ($line.StartsWith("#")) {
            throw "Preprocessor directives are forbidden inside installer [Files]; the release file set must be explicit: $line"
        }
        $fields = @{}
        foreach ($part in @(Split-InnoRecordFields $rawLine)) {
            if ($part -notmatch '^\s*([A-Za-z][A-Za-z0-9]*)\s*:\s*(.*?)\s*$') {
                throw "Malformed installer [Files] field: $part"
            }
            $name = $Matches[1].ToLowerInvariant()
            if ($fields.ContainsKey($name)) {
                throw "Duplicate installer [Files] field '$name': $rawLine"
            }
            $fields[$name] = $Matches[2]
        }
        if (-not $fields.ContainsKey("source")) {
            throw "Every installer [Files] record must contain Source: $rawLine"
        }
        $source = Get-InnoQuotedValue -Value ([string]$fields["source"]) -FieldName "Source"
        if (-not $fields.ContainsKey("destdir")) {
            throw "Every installer [Files] record must contain DestDir: $rawLine"
        }
        $destDir = Get-InnoQuotedValue -Value ([string]$fields["destdir"]) -FieldName "DestDir"
        $flags = @()
        if ($fields.ContainsKey("flags")) {
            $flags = @(([string]$fields["flags"]).Trim() -split '\s+' | Where-Object { -not [string]::IsNullOrWhiteSpace($_) } |
                ForEach-Object { $_.ToLowerInvariant() })
        }
        $records += [pscustomobject]@{
            source   = $source.Replace('/', '\')
            destDir  = $destDir.Replace('/', '\')
            flags    = @($flags)
            excludes = if ($fields.ContainsKey("excludes")) { [string]$fields["excludes"] } else { "" }
            fieldNames = @($fields.Keys | Sort-Object)
            raw      = $rawLine
        }
    }
    if ($filesSectionCount -ne 1 -or $records.Count -eq 0) {
        throw "Installer must contain exactly one non-empty [Files] section."
    }
    return @($records)
}

function Assert-InstallerDefinition {
    param(
        [Parameter(Mandatory = $true)][string]$RepoRoot,
        [Parameter(Mandatory = $true)][string]$AppVersion,
        [Parameter(Mandatory = $true)][ValidateSet("neutral", "brand")][string]$Channel
    )

    Assert-ReleaseVersion $AppVersion
    $spec = Get-ReleaseChannelSpec $Channel
    $issPath = Join-Path $RepoRoot "installer\QtWidgetsApplication4.iss"
    if (-not (Test-Path -LiteralPath $issPath -PathType Leaf)) {
        throw "Installer definition does not exist: $issPath"
    }
    $text = Get-Content -LiteralPath $issPath -Raw -Encoding UTF8
    $escapedVersion = [regex]::Escape($AppVersion)
    $escapedAppName = [regex]::Escape($spec.AppName)
    $escapedExeName = [regex]::Escape($spec.ExeName)
    $escapedOutput = [regex]::Escape($spec.OutputPrefix + $AppVersion)
    $escapedAppId = [regex]::Escape($script:ReleaseGateExpectedAppId)
    # The installer preprocessor is intentionally non-programmable at release
    # time. Exact top-level shape prevents #include/#emit/#undef and #if 0 from
    # changing the file set after this text-level gate has inspected it.
    $preprocessorLines = @([regex]::Matches($text, '(?m)^[\t ]*#[^\r\n]*\r?$') | ForEach-Object { $_.Value.Trim() })
    $preprocessorPatterns = @(
        ('^#define\s+MyAppName\s+"' + $escapedAppName + '"$'),
        ('^#define\s+MyAppGuid\s+"' + $escapedAppId + '"$'),
        '^#ifndef\s+MyAppVersion$',
        ('^#define\s+MyAppVersion\s+"' + $escapedVersion + '"$'),
        '^#endif$',
        '^#ifndef\s+MyOutputBaseFilename$',
        ('^#define\s+MyOutputBaseFilename\s+"' + $escapedOutput + '"$'),
        '^#endif$',
        '^#define\s+MyAppPublisher\s+"[^"\r\n]+"$',
        ('^#define\s+MyAppExeName\s+"' + $escapedExeName + '"$'),
        '^#define\s+MySourceDir\s+"\.\.\\dist\\QtWidgetsApplication4"$'
    )
    if ($preprocessorLines.Count -ne $preprocessorPatterns.Count) {
        throw "Installer preprocessor directive count is not the fixed release shape; include/undef/conditional/extra define directives are forbidden: $issPath"
    }
    for ($i = 0; $i -lt $preprocessorPatterns.Count; ++$i) {
        if ($preprocessorLines[$i] -cnotmatch $preprocessorPatterns[$i]) {
            throw "Installer preprocessor directive $($i + 1) is not allowed by the fixed release shape: $($preprocessorLines[$i])"
        }
    }
    $requirements = @(
        @{ Pattern = '(?m)^#define\s+MyAppName\s+"' + $escapedAppName + '"\s*$'; Name = "MyAppName" },
        @{ Pattern = '(?m)^#define\s+MyAppGuid\s+"' + $escapedAppId + '"\s*$'; Name = "MyAppGuid" },
        @{ Pattern = '(?m)^\s*#define\s+MyAppVersion\s+"' + $escapedVersion + '"\s*$'; Name = "MyAppVersion" },
        @{ Pattern = '(?m)^\s*#define\s+MyOutputBaseFilename\s+"' + $escapedOutput + '"\s*$'; Name = "MyOutputBaseFilename" },
        @{ Pattern = '(?m)^#define\s+MyAppExeName\s+"' + $escapedExeName + '"\s*$'; Name = "MyAppExeName" },
        @{ Pattern = '(?m)^#define\s+MySourceDir\s+"\.\.\\dist\\QtWidgetsApplication4"\s*$'; Name = "MySourceDir" },
        @{ Pattern = '(?m)^AppId=\{\{' + $escapedAppId + '\}\s*$'; Name = "AppId" }
    )
    foreach ($requirement in $requirements) {
        if ($text -notmatch $requirement.Pattern) {
            throw "Installer $($requirement.Name) does not match channel=$Channel version=$AppVersion AppId=$($script:ReleaseGateExpectedAppId): $issPath"
        }
    }
    $fileRecords = @(Get-InnoFilesRecords -Text $text)
    $required = @(
        [pscustomobject]@{
            Name = "recursive application payload"; Source = "{#MySourceDir}\*"; DestDir = "{app}"
            AllowedFields = @("destdir", "excludes", "flags", "source")
            RequiredFlags = @("ignoreversion", "recursesubdirs", "createallsubdirs"); RequiredExcludes = @("data\*", "sdk\step\versions\*")
        },
        [pscustomobject]@{
            Name = "ConfigMigrate.exe"; Source = "..\dist\tools\ConfigMigrate.exe"; DestDir = "{app}\tools"
            AllowedFields = @("destdir", "flags", "source"); RequiredFlags = @("ignoreversion"); RequiredExcludes = @()
        },
        [pscustomobject]@{
            Name = "ConfigMigrate_Run.cmd"; Source = "..\dist\tools\ConfigMigrate_Run.cmd"; DestDir = "{app}\tools"
            AllowedFields = @("destdir", "flags", "source"); RequiredFlags = @("ignoreversion"); RequiredExcludes = @()
        }
    )
    if ($fileRecords.Count -ne $required.Count) {
        throw "Installer [Files] must contain exactly the three approved release Source records; found $($fileRecords.Count): $issPath"
    }
    $actualSources = @($fileRecords | ForEach-Object { [string]$_.source } | Sort-Object)
    $allowedSources = @($required | ForEach-Object { [string]$_.Source } | Sort-Object)
    if (($actualSources -join "`n") -cne ($allowedSources -join "`n")) {
        throw "Installer [Files] Source set is not exactly the approved three paths: $($actualSources -join ', ')"
    }
    foreach ($expectation in $required) {
        $matches = @($fileRecords | Where-Object {
            [string]$_.source -ceq [string]$expectation.Source -and
            [string]$_.destDir -ceq [string]$expectation.DestDir
        })
        if ($matches.Count -ne 1) {
            throw "Installer [Files] must contain exactly one required $($expectation.Name) Source/DestDir record: $issPath"
        }
        $record = $matches[0]
        if ((@($record.fieldNames) -join "`n") -cne (@($expectation.AllowedFields | Sort-Object) -join "`n")) {
            throw "Installer required $($expectation.Name) record has conditional/unapproved fields: $($record.fieldNames -join ', ')"
        }
        $actualFlags = @($record.flags | Sort-Object)
        $requiredFlags = @($expectation.RequiredFlags | Sort-Object)
        if (($actualFlags -join "`n") -cne ($requiredFlags -join "`n")) {
            throw "Installer required $($expectation.Name) flags must be exact; expected $($requiredFlags -join ', '), actual $($actualFlags -join ', '): $issPath"
        }
        if (@($expectation.RequiredExcludes).Count -gt 0) {
            $rawExcludes = Get-InnoQuotedValue -Value ([string]$record.excludes) -FieldName "Excludes"
            $actualExcludes = @($rawExcludes -split ',' | ForEach-Object { $_.Trim().Replace('/', '\').ToLowerInvariant() } | Sort-Object)
            $requiredExcludes = @($expectation.RequiredExcludes | Sort-Object)
            if (($actualExcludes -join "`n") -cne ($requiredExcludes -join "`n")) {
                throw "Installer recursive payload exclusions must be exact; expected $($requiredExcludes -join ', '), actual $($actualExcludes -join ', '): $issPath"
            }
        }
    }
    return [pscustomobject]@{
        path    = Resolve-ReleaseGatePath $issPath
        appId   = $script:ReleaseGateExpectedAppId
        sha256  = Get-ReleaseFileSha256 $issPath
        filesSources = @($fileRecords | ForEach-Object { [string]$_.source })
    }
}

function Assert-GitReleaseState {
    param(
        [Parameter(Mandatory = $true)][string]$RepoRoot,
        [Parameter(Mandatory = $true)][string]$AppVersion,
        [Parameter(Mandatory = $true)][ValidateSet("neutral", "brand")][string]$Channel
    )

    $repoRoot = Resolve-ReleaseGatePath $RepoRoot
    $inside = (Invoke-ReleaseGit -RepoRoot $repoRoot -Arguments @("rev-parse", "--is-inside-work-tree") | Select-Object -Last 1).Trim()
    if ($inside -ne "true") {
        throw "Release root is not a git worktree: $repoRoot"
    }
    $dirty = @(Invoke-ReleaseGit -RepoRoot $repoRoot -Arguments @("status", "--porcelain=v1", "--untracked-files=all"))
    if ($dirty.Count -ne 0) {
        throw "Tracked or unmerged changes make the release worktree ambiguous:`n$($dirty -join [Environment]::NewLine)"
    }
    $head = (Invoke-ReleaseGit -RepoRoot $repoRoot -Arguments @("rev-parse", "HEAD") | Select-Object -Last 1).Trim()
    $spec = Get-ReleaseChannelSpec $Channel
    $branchHead = (Invoke-ReleaseGit -RepoRoot $repoRoot -Arguments @("rev-parse", "refs/heads/$($spec.Branch)") | Select-Object -Last 1).Trim()
    if ($head -ne $branchHead) {
        throw "Release HEAD $head does not equal refs/heads/$($spec.Branch) $branchHead. Stale or detached historical commits are forbidden."
    }
    if ($Channel -eq "brand") {
        Invoke-ReleaseGit -RepoRoot $repoRoot -Arguments @("merge-base", "--is-ancestor", "refs/heads/main", "HEAD") | Out-Null
    }

    $trackedBranding = @(Invoke-ReleaseGit -RepoRoot $repoRoot -Arguments @("ls-files", "branding")) | Where-Object { -not [string]::IsNullOrWhiteSpace($_) }
    if ($spec.RequiresBranding) {
        if ($trackedBranding.Count -eq 0) {
            throw "Brand release has no git-tracked branding files."
        }
        foreach ($relative in $trackedBranding) {
            if (-not (Test-Path -LiteralPath (Join-Path $repoRoot $relative) -PathType Leaf)) {
                throw "Tracked brand asset is missing from the worktree: $relative"
            }
        }
    }
    elseif ($trackedBranding.Count -ne 0) {
        throw "Neutral release unexpectedly tracks branding files: $($trackedBranding -join ', ')"
    }

    $installer = Assert-InstallerDefinition -RepoRoot $repoRoot -AppVersion $AppVersion -Channel $Channel
    return [pscustomobject]@{
        repoRoot          = $repoRoot
        head              = $head
        branch            = $spec.Branch
        gitCommonDir      = Get-ReleaseGitPath -RepoRoot $repoRoot -Kind common
        gitWorktreeDir    = Get-ReleaseGitPath -RepoRoot $repoRoot -Kind worktree
        trackedBranding   = @($trackedBranding)
        installer         = $installer
    }
}

function Assert-LinkedReleaseWorktrees {
    param(
        [Parameter(Mandatory = $true)][string]$NeutralRoot,
        [Parameter(Mandatory = $true)][string]$BrandRoot,
        [Parameter(Mandatory = $true)][string]$NeutralHead,
        [Parameter(Mandatory = $true)][string]$BrandHead
    )

    $neutralRoot = Resolve-ReleaseGatePath $NeutralRoot
    $brandRoot = Resolve-ReleaseGatePath $BrandRoot
    if (Test-ReleaseGateSamePath $neutralRoot $brandRoot) {
        throw "Neutral and brand releases must use two distinct linked worktrees."
    }
    $neutralCommon = Get-ReleaseGitPath -RepoRoot $neutralRoot -Kind common
    $brandCommon = Get-ReleaseGitPath -RepoRoot $brandRoot -Kind common
    if (-not (Test-ReleaseGateSamePath $neutralCommon $brandCommon)) {
        throw "Neutral and brand release roots are independent clones; linked worktrees with one Git common directory are required."
    }
    $neutralGitDir = Get-ReleaseGitPath -RepoRoot $neutralRoot -Kind worktree
    $brandGitDir = Get-ReleaseGitPath -RepoRoot $brandRoot -Kind worktree
    if (Test-ReleaseGateSamePath $neutralGitDir $brandGitDir) {
        throw "Neutral and brand paths resolve to the same Git worktree metadata directory."
    }
    $liveNeutralHead = (Invoke-ReleaseGit -RepoRoot $neutralRoot -Arguments @("rev-parse", "HEAD") | Select-Object -Last 1).Trim()
    $liveBrandHead = (Invoke-ReleaseGit -RepoRoot $brandRoot -Arguments @("rev-parse", "HEAD") | Select-Object -Last 1).Trim()
    if ($liveNeutralHead -cne $NeutralHead -or $liveBrandHead -cne $BrandHead) {
        throw "Release worktree HEAD changed while validating neutral/brand ancestry."
    }
    Invoke-ReleaseGit -RepoRoot $brandRoot -Arguments @("merge-base", "--is-ancestor", $NeutralHead, $BrandHead) | Out-Null
    return [pscustomobject]@{
        commonDir       = $neutralCommon
        neutralGitDir   = $neutralGitDir
        brandGitDir     = $brandGitDir
        neutralHead     = $NeutralHead
        brandHead       = $BrandHead
    }
}

function Assert-ExpectedReleaseExecutable {
    param(
        [Parameter(Mandatory = $true)][string]$Directory,
        [Parameter(Mandatory = $true)][string]$AppVersion,
        [Parameter(Mandatory = $true)][ValidateSet("neutral", "brand")][string]$Channel
    )

    $spec = Get-ReleaseChannelSpec $Channel
    $allowedNonApplication = @("vc_redist.x64.exe", "vc_redist.x86.exe")
    $executables = @(Get-ChildItem -LiteralPath $Directory -File -Filter "*.exe" -ErrorAction Stop | Where-Object {
        $allowedNonApplication -notcontains $_.Name.ToLowerInvariant()
    })
    if ($executables.Count -ne 1 -or $executables[0].Name -cne $spec.ExeName) {
        throw "Channel $Channel must contain exactly one top-level application exe named $($spec.ExeName); found: $($executables.Name -join ', ')"
    }
    Assert-WindowsPeReleaseVersion `
        -Path $executables[0].FullName `
        -AppVersion $AppVersion `
        -ExpectedProductName $spec.AppName `
        -ExpectedOriginalFilename $spec.ExeName | Out-Null
    return $executables[0].FullName
}

function Assert-ReleaseChannelAssets {
    param(
        [Parameter(Mandatory = $true)][string]$RepoRoot,
        [Parameter(Mandatory = $true)][string]$PackageDir,
        [Parameter(Mandatory = $true)][ValidateSet("neutral", "brand")][string]$Channel
    )

    $spec = Get-ReleaseChannelSpec $Channel
    $trackedBranding = @(Invoke-ReleaseGit -RepoRoot $RepoRoot -Arguments @("ls-files", "branding")) | Where-Object {
        -not [string]::IsNullOrWhiteSpace($_)
    } | Sort-Object
    $packageBrandingDir = Join-Path $PackageDir "branding"
    if (-not $spec.RequiresBranding) {
        if (Test-Path -LiteralPath $packageBrandingDir) {
            throw "Neutral package must not contain a branding directory."
        }
    }
    else {
        if ($trackedBranding.Count -eq 0 -or -not (Test-Path -LiteralPath $packageBrandingDir -PathType Container)) {
            throw "Brand package is missing its tracked branding assets."
        }
        $actualBranding = @(Get-ChildItem -LiteralPath $packageBrandingDir -Recurse -File | ForEach-Object {
            Get-ReleaseRelativePath -Root $PackageDir -Path $_.FullName
        } | Sort-Object)
        if (($actualBranding -join "`n") -cne ($trackedBranding -join "`n")) {
            throw "Brand package branding file set differs from git-tracked branding assets."
        }
        foreach ($relative in $trackedBranding) {
            if ((Get-ReleaseFileSha256 (Join-Path $RepoRoot $relative)) -cne (Get-ReleaseFileSha256 (Join-Path $PackageDir $relative))) {
                throw "Branding asset hash differs between source and package: $relative"
            }
        }
    }

    $trackedIcons = @(Invoke-ReleaseGit -RepoRoot $RepoRoot -Arguments @("ls-files", "icons")) | Where-Object {
        -not [string]::IsNullOrWhiteSpace($_)
    } | Sort-Object
    if ($trackedIcons.Count -eq 0) {
        throw "Release source has no git-tracked icons."
    }
    $packageIconsDir = Join-Path $PackageDir "icons"
    $actualIcons = @()
    if (Test-Path -LiteralPath $packageIconsDir -PathType Container) {
        $actualIcons = @(Get-ChildItem -LiteralPath $packageIconsDir -Recurse -File | ForEach-Object {
            Get-ReleaseRelativePath -Root $PackageDir -Path $_.FullName
        } | Sort-Object)
    }
    if (($actualIcons -join "`n") -cne ($trackedIcons -join "`n")) {
        throw "Package icon file set differs from git-tracked icons."
    }
    foreach ($relative in $trackedIcons) {
        if ((Get-ReleaseFileSha256 (Join-Path $RepoRoot $relative)) -cne (Get-ReleaseFileSha256 (Join-Path $PackageDir $relative))) {
            throw "Icon hash differs between source and package: $relative"
        }
    }
}

function Get-WindowsPeMetadata {
    param([Parameter(Mandatory = $true)][string]$Path)

    if (-not (Test-Path -LiteralPath $Path -PathType Leaf)) {
        throw "Windows executable does not exist: $Path"
    }
    $stream = $null
    $reader = $null
    try {
        $stream = [System.IO.File]::Open($Path, [System.IO.FileMode]::Open, [System.IO.FileAccess]::Read, [System.IO.FileShare]::Read)
        if ($stream.Length -lt 256) {
            throw "Windows executable is too small to be a PE image: $Path"
        }
        $reader = New-Object System.IO.BinaryReader($stream)
        if ($reader.ReadUInt16() -ne 0x5A4D) {
            throw "Windows executable has no MZ header: $Path"
        }
        $stream.Position = 0x3c
        $peOffset = $reader.ReadInt32()
        if ($peOffset -lt 0x40 -or ([int64]$peOffset + 26) -gt $stream.Length) {
            throw "Windows executable has an invalid PE header offset: $Path"
        }
        $stream.Position = $peOffset
        if ($reader.ReadUInt32() -ne 0x00004550) {
            throw "Windows executable has no PE signature: $Path"
        }
        $machine = $reader.ReadUInt16()
        $stream.Position = [int64]$peOffset + 22
        $characteristics = $reader.ReadUInt16()
        $stream.Position = [int64]$peOffset + 24
        $optionalMagic = $reader.ReadUInt16()
        if (($characteristics -band 0x0002) -eq 0 -or ($characteristics -band 0x2000) -ne 0) {
            throw "Release application must be an executable PE image, not a DLL/object: $Path"
        }
        return [pscustomobject]@{
            machine         = [int]$machine
            characteristics = [int]$characteristics
            optionalMagic   = [int]$optionalMagic
        }
    }
    finally {
        if ($null -ne $reader) { $reader.Dispose() }
        elseif ($null -ne $stream) { $stream.Dispose() }
    }
}

function Assert-WindowsPeReleaseVersion {
    param(
        [Parameter(Mandatory = $true)][string]$Path,
        [Parameter(Mandatory = $true)][string]$AppVersion,
        [Parameter(Mandatory = $true)][string]$ExpectedProductName,
        [Parameter(Mandatory = $true)][string]$ExpectedOriginalFilename
    )

    Assert-ReleaseVersion $AppVersion
    $pe = Get-WindowsPeMetadata -Path $Path
    if ($pe.machine -ne 0x8664 -or $pe.optionalMagic -ne 0x020b) {
        throw ("Release application must be a native x64 PE32+ image: machine=0x{0:x4}, optionalMagic=0x{1:x4}, path={2}" -f `
            $pe.machine, $pe.optionalMagic, $Path)
    }
    $versionInfo = [System.Diagnostics.FileVersionInfo]::GetVersionInfo((Resolve-ReleaseGatePath $Path))
    $fileVersion = [string]$versionInfo.FileVersion
    $productVersion = [string]$versionInfo.ProductVersion
    $productName = [string]$versionInfo.ProductName
    $originalFilename = [string]$versionInfo.OriginalFilename
    if ($fileVersion -cne $AppVersion -or $productVersion -cne $AppVersion) {
        throw "Release PE FileVersion/ProductVersion mismatch: expected $AppVersion, actual file=$fileVersion product=$productVersion, path=$Path"
    }
    if ($productName -cne $ExpectedProductName -or $originalFilename -cne $ExpectedOriginalFilename) {
        throw "Release PE channel identity mismatch: expected product=$ExpectedProductName file=$ExpectedOriginalFilename, actual product=$productName file=$originalFilename, path=$Path"
    }
    return [pscustomobject]@{
        machine          = "0x8664"
        fileVersion      = $fileVersion
        productVersion   = $productVersion
        productName      = $productName
        originalFilename = $originalFilename
    }
}

function Assert-SourceApplicationVersion {
    param(
        [Parameter(Mandatory = $true)][string]$RepoRoot,
        [Parameter(Mandatory = $true)][string]$AppVersion
    )

    $mainPath = Join-Path $RepoRoot "src\main.cpp"
    $text = Get-Content -LiteralPath $mainPath -Raw -Encoding UTF8
    $matches = [regex]::Matches($text, 'app\.setApplicationVersion\s*\(\s*QStringLiteral\s*\(\s*"([^"]+)"\s*\)\s*\)')
    if ($matches.Count -ne 1 -or $matches[0].Groups[1].Value -cne $AppVersion) {
        $actual = if ($matches.Count -eq 1) { $matches[0].Groups[1].Value } else { "ambiguous-or-missing" }
        throw "src/main.cpp application version must exactly match release version: expected $AppVersion, actual $actual"
    }
    return (Get-ReleaseFileSha256 $mainPath)
}

function Read-FanucRuntimeManifest {
    param([Parameter(Mandatory = $true)][string]$RepoRoot)

    $manifestPath = Join-Path $RepoRoot "installer\fanuc-runtime-manifest.json"
    Invoke-ReleaseGit -RepoRoot $RepoRoot -Arguments @("ls-files", "--error-unmatch", "installer/fanuc-runtime-manifest.json") | Out-Null
    $manifest = Read-ReleaseGateJson $manifestPath
    if ([int]$manifest.schemaVersion -ne 1 -or [string]::IsNullOrWhiteSpace([string]$manifest.manifestVersion)) {
        throw "Unsupported FANUC runtime manifest schema: $manifestPath"
    }
    $files = @($manifest.files)
    if ([int]$manifest.expectedFileCount -ne 21 `
        -or [int]$manifest.expectedTpCount -ne 12 `
        -or [int]$manifest.expectedPcCount -ne 9 `
        -or $files.Count -ne [int]$manifest.expectedFileCount `
        -or @($files | Where-Object { ([System.IO.Path]::GetExtension([string]$_.path)).ToLowerInvariant() -eq '.tp' }).Count -ne [int]$manifest.expectedTpCount `
        -or @($files | Where-Object { ([System.IO.Path]::GetExtension([string]$_.path)).ToLowerInvariant() -eq '.pc' }).Count -ne [int]$manifest.expectedPcCount) {
        throw "FANUC runtime manifest counts are inconsistent: $manifestPath"
    }
    $seen = @{}
    foreach ($item in $files) {
        $relative = ([string]$item.path).Replace('\', '/')
        if ($relative -notmatch '^SDK/FANUC/[A-Za-z0-9_.-]+\.(?i:tp|pc)$' `
            -or [string]$item.sha256 -notmatch '^[0-9a-f]{64}$' `
            -or [int64]$item.size -le 0) {
            throw "Invalid FANUC runtime manifest item: $relative"
        }
        $key = $relative.ToLowerInvariant()
        if ($seen.ContainsKey($key)) {
            throw "Duplicate FANUC runtime manifest path: $relative"
        }
        $seen[$key] = $true
    }
    return [pscustomobject]@{
        path     = Resolve-ReleaseGatePath $manifestPath
        sha256   = Get-ReleaseFileSha256 $manifestPath
        manifest = $manifest
    }
}

function Assert-FanucRuntimeManifest {
    param(
        [Parameter(Mandatory = $true)][string]$RepoRoot,
        [Parameter(Mandatory = $true)][string]$RuntimeRoot
    )

    $manifestInfo = Read-FanucRuntimeManifest $RepoRoot
    $expected = @($manifestInfo.manifest.files | Sort-Object path)
    $fanucDir = Join-Path $RuntimeRoot "SDK\FANUC"
    if (-not (Test-Path -LiteralPath $fanucDir -PathType Container)) {
        throw "FANUC runtime directory does not exist: $fanucDir"
    }
    $actualPaths = @(Get-ChildItem -LiteralPath $fanucDir -File | Where-Object {
        $_.Extension.ToLowerInvariant() -in @('.tp', '.pc')
    } | ForEach-Object {
        Get-ReleaseRelativePath -Root $RuntimeRoot -Path $_.FullName
    } | Sort-Object)
    $expectedPaths = @($expected | ForEach-Object { ([string]$_.path).Replace('\', '/') })
    if (($actualPaths -join "`n") -cne ($expectedPaths -join "`n")) {
        throw "FANUC runtime file set differs from the versioned manifest. Expected: $($expectedPaths -join ', '); actual: $($actualPaths -join ', ')"
    }
    foreach ($item in $expected) {
        $path = Join-Path $RuntimeRoot (([string]$item.path).Replace('/', '\'))
        $file = Get-Item -LiteralPath $path -ErrorAction Stop
        $sha = Get-ReleaseFileSha256 $path
        if ([int64]$file.Length -ne [int64]$item.size -or $sha -cne [string]$item.sha256) {
            throw "FANUC runtime file does not match manifest size/SHA256: $($item.path)"
        }
    }
    return $manifestInfo
}

function Assert-ConfigMigrateProvenance {
    param(
        [Parameter(Mandatory = $true)][string]$RepoRoot,
        [Parameter(Mandatory = $true)][string]$ExecutablePath
    )

    $pythonTool = Get-ReleasePythonTool
    $sourcePath = Join-Path $RepoRoot "tools\migrate_config_to_sqlite.py"
    $buildScript = Join-Path $RepoRoot "scripts\build_config_migrate.ps1"
    $expectedSourceHash = Get-ReleaseFileSha256 $sourcePath
    $buildScriptHash = Get-ReleaseFileSha256 $buildScript
    if (-not (Test-Path -LiteralPath $ExecutablePath -PathType Leaf)) {
        throw "ConfigMigrate.exe is missing: $ExecutablePath"
    }
    $exeHashBefore = Get-ReleaseFileSha256 $ExecutablePath
    $verificationRoot = Join-Path ([System.IO.Path]::GetTempPath()) ("config-migrate-provenance-" + [Guid]::NewGuid().ToString("N"))
    $rebuiltPath = Join-Path $verificationRoot "ConfigMigrate.exe"
    try {
        New-Item -ItemType Directory -Path $verificationRoot -Force | Out-Null
        $buildOutput = @(& $buildScript `
            -PythonExecutable $pythonTool.path `
            -PythonSha256 $pythonTool.sha256 `
            -OutputPath $rebuiltPath | ForEach-Object { [string]$_ })
        if (-not (Test-Path -LiteralPath $rebuiltPath -PathType Leaf)) {
            throw "Current ConfigMigrate builder did not create the isolated verification executable: $rebuiltPath"
        }
        $sourceHashAfter = Get-ReleaseFileSha256 $sourcePath
        $buildScriptHashAfter = Get-ReleaseFileSha256 $buildScript
        $exeHashAfter = Get-ReleaseFileSha256 $ExecutablePath
        $rebuiltHash = Get-ReleaseFileSha256 $rebuiltPath
        if ($sourceHashAfter -cne $expectedSourceHash `
            -or $buildScriptHashAfter -cne $buildScriptHash `
            -or $exeHashAfter -cne $exeHashBefore) {
            throw "ConfigMigrate source, builder, or candidate changed during isolated rebuild verification."
        }
        if ($rebuiltHash -cne $exeHashAfter) {
            throw "ConfigMigrate.exe bytes differ from an isolated rebuild by the current source and build_config_migrate.ps1."
        }
        return [pscustomobject]@{
            path                = Resolve-ReleaseGatePath $ExecutablePath
            sha256              = $exeHashAfter
            sourceSha256        = $expectedSourceHash
            builderSha256       = $buildScriptHash
            isolatedRebuildSha256 = $rebuiltHash
            pythonSha256        = $pythonTool.sha256
        }
    }
    finally {
        if (Test-Path -LiteralPath $verificationRoot) {
            Remove-Item -LiteralPath $verificationRoot -Recurse -Force -ErrorAction SilentlyContinue
        }
    }
}

function Assert-EmptyReleaseRuntimeDirectories {
    param([Parameter(Mandatory = $true)][string]$PackageDir)

    foreach ($name in @("Data", "Log", "Result", "Temp")) {
        $path = Join-Path $PackageDir $name
        if (-not (Test-Path -LiteralPath $path -PathType Container)) {
            throw "Release package runtime directory is missing: $path"
        }
        $children = @(Get-ChildItem -LiteralPath $path -Force)
        if ($children.Count -ne 0) {
            throw "Release package runtime directory must be completely empty: $path"
        }
    }
}

function Get-PackageGateReportPath {
    param(
        [Parameter(Mandatory = $true)][string]$RepoRoot,
        [Parameter(Mandatory = $true)][string]$AppVersion,
        [Parameter(Mandatory = $true)][ValidateSet("neutral", "brand")][string]$Channel
    )
    return (Join-Path $RepoRoot "dist\release-gates\package-$Channel-$AppVersion.json")
}

function Get-InstallerGateReportPath {
    param(
        [Parameter(Mandatory = $true)][string]$RepoRoot,
        [Parameter(Mandatory = $true)][string]$AppVersion,
        [Parameter(Mandatory = $true)][ValidateSet("neutral", "brand")][string]$Channel
    )
    return (Join-Path $RepoRoot "dist\release-gates\installer-$Channel-$AppVersion.json")
}

function Get-ReleasePairGateReportPath {
    param(
        [Parameter(Mandatory = $true)][string]$RepoRoot,
        [Parameter(Mandatory = $true)][string]$AppVersion
    )
    Assert-ReleaseVersion $AppVersion
    return (Join-Path $RepoRoot "dist\release-gates\release-pair-$AppVersion.json")
}

function Assert-ReleasePairOutputPath {
    param(
        [Parameter(Mandatory = $true)][string]$CandidatePath,
        [Parameter(Mandatory = $true)][string]$CanonicalPath,
        [Parameter(Mandatory = $true)][string[]]$ProtectedPaths
    )

    if (-not (Test-ReleaseGateSamePath $CandidatePath $CanonicalPath)) {
        throw "Release pair report path must be canonical: $CanonicalPath"
    }
    $candidate = Resolve-ReleaseGatePath $CandidatePath
    foreach ($protectedPath in @($ProtectedPaths)) {
        if ([string]::IsNullOrWhiteSpace($protectedPath)) {
            continue
        }
        if (Test-ReleaseGateSamePath $candidate $protectedPath) {
            throw "Release pair output aliases a validated input/report/artifact: $protectedPath"
        }
    }
    if (Test-Path -LiteralPath $candidate -PathType Container) {
        throw "Release pair output path is an existing directory: $candidate"
    }
    if (Test-Path -LiteralPath $candidate -PathType Leaf) {
        $attributes = (Get-Item -LiteralPath $candidate -Force).Attributes
        if (($attributes -band [System.IO.FileAttributes]::ReparsePoint) -ne 0) {
            throw "Release pair output must not be a reparse-point alias: $candidate"
        }
    }
    return $candidate
}

function New-PackageGateReport {
    param(
        [Parameter(Mandatory = $true)][string]$RepoRoot,
        [Parameter(Mandatory = $true)][string]$PackageDir,
        [Parameter(Mandatory = $true)][string]$AppVersion,
        [Parameter(Mandatory = $true)][ValidateSet("neutral", "brand")][string]$Channel,
        [Parameter(Mandatory = $true)][string]$ConfigMigratePath,
        [string]$RunId = "",
        [string]$OutputPath = ""
    )

    $canonicalPackageDir = Join-Path $RepoRoot "dist\QtWidgetsApplication4"
    $canonicalConfigMigrate = Join-Path $RepoRoot "dist\tools\ConfigMigrate.exe"
    if (-not (Test-ReleaseGateSamePath $PackageDir $canonicalPackageDir) `
        -or -not (Test-ReleaseGateSamePath $ConfigMigratePath $canonicalConfigMigrate)) {
        throw "Package and ConfigMigrate paths must be the canonical repo dist paths."
    }
    $state = Assert-GitReleaseState -RepoRoot $RepoRoot -AppVersion $AppVersion -Channel $Channel
    $sourceVersionSha256 = Assert-SourceApplicationVersion -RepoRoot $RepoRoot -AppVersion $AppVersion
    $exe = Assert-ExpectedReleaseExecutable -Directory $PackageDir -AppVersion $AppVersion -Channel $Channel
    $channelSpec = Get-ReleaseChannelSpec $Channel
    $exeVersion = Assert-WindowsPeReleaseVersion `
        -Path $exe `
        -AppVersion $AppVersion `
        -ExpectedProductName $channelSpec.AppName `
        -ExpectedOriginalFilename $channelSpec.ExeName
    Assert-ReleaseChannelAssets -RepoRoot $RepoRoot -PackageDir $PackageDir -Channel $Channel
    $fanuc = Assert-FanucRuntimeManifest -RepoRoot $RepoRoot -RuntimeRoot $PackageDir
    $config = Assert-ConfigMigrateProvenance -RepoRoot $RepoRoot -ExecutablePath $ConfigMigratePath
    Assert-EmptyReleaseRuntimeDirectories $PackageDir
    if ([string]::IsNullOrWhiteSpace($OutputPath)) {
        $OutputPath = Get-PackageGateReportPath -RepoRoot $RepoRoot -AppVersion $AppVersion -Channel $Channel
    }
    $canonicalOutputPath = Get-PackageGateReportPath -RepoRoot $RepoRoot -AppVersion $AppVersion -Channel $Channel
    if (-not (Test-ReleaseGateSamePath $OutputPath $canonicalOutputPath)) {
        throw "Package gate report must be written to its canonical dist/release-gates path."
    }
    $configRunSource = Join-Path $RepoRoot "tools\ConfigMigrate_Run.cmd"
    $configRunDist = Join-Path $RepoRoot "dist\tools\ConfigMigrate_Run.cmd"
    $configRunSourceHash = Get-ReleaseFileSha256 $configRunSource
    $configRunDistHash = Get-ReleaseFileSha256 $configRunDist
    if ($configRunSourceHash -cne $configRunDistHash) {
        throw "dist/tools/ConfigMigrate_Run.cmd differs from the tracked source command."
    }
    $gateScriptPath = $script:ReleaseGateCommonPath
    if ([string]::IsNullOrWhiteSpace($RunId)) {
        $RunId = [Guid]::NewGuid().ToString("D")
    }
    $producerScriptPath = Join-Path $RepoRoot "scripts\build_release_package.ps1"
    $report = [ordered]@{
        schemaVersion      = $script:ReleaseGateSchemaVersion
        kind               = "package"
        status             = "pass"
        runId              = $RunId
        generatedAtUtc     = [DateTime]::UtcNow.ToString("o")
        gateScriptSha256   = Get-ReleaseFileSha256 $gateScriptPath
        producerScriptSha256 = Get-ReleaseFileSha256 $producerScriptPath
        appId              = $script:ReleaseGateExpectedAppId
        version            = $AppVersion
        channel            = $Channel
        head               = $state.head
        gitCommonDir       = $state.gitCommonDir
        gitWorktreeDir     = $state.gitWorktreeDir
        repoRoot           = Resolve-ReleaseGatePath $RepoRoot
        packageDir         = Resolve-ReleaseGatePath $PackageDir
        expectedExe        = $channelSpec.ExeName
        executableSha256   = Get-ReleaseFileSha256 $exe
        executableVersion  = $exeVersion
        sourceVersionSha256 = $sourceVersionSha256
        installerSource    = $state.installer
        fanucManifest      = [ordered]@{ sha256 = $fanuc.sha256; fileCount = [int]$fanuc.manifest.expectedFileCount }
        configMigrate      = $config
        configMigrateRun   = [ordered]@{ path = Resolve-ReleaseGatePath $configRunDist; sha256 = $configRunDistHash }
        packageInventory   = @(Get-ReleaseFileInventory $PackageDir)
    }
    Write-ReleaseGateJson -Value $report -Path $OutputPath
    return (Resolve-ReleaseGatePath $OutputPath)
}

function Assert-PackageGateReport {
    param(
        [Parameter(Mandatory = $true)][string]$ReportPath,
        [Parameter(Mandatory = $true)][string]$ExpectedRepoRoot,
        [Parameter(Mandatory = $true)][string]$ExpectedAppVersion,
        [Parameter(Mandatory = $true)][ValidateSet("neutral", "brand")][string]$ExpectedChannel
    )

    $report = Read-ReleaseGateJson $ReportPath
    if ([int]$report.schemaVersion -ne $script:ReleaseGateSchemaVersion `
        -or [string]$report.kind -cne "package" `
        -or [string]$report.status -cne "pass" `
        -or [string]::IsNullOrWhiteSpace([string]$report.runId)) {
        throw "Unsupported package gate report: $ReportPath"
    }
    $gateScriptPath = $script:ReleaseGateCommonPath
    if ([string]$report.gateScriptSha256 -cne (Get-ReleaseFileSha256 $gateScriptPath)) {
        throw "Package gate report was produced by a different release gate script."
    }
    $producerScriptPath = Join-Path $ExpectedRepoRoot "scripts\build_release_package.ps1"
    if ([string]$report.producerScriptSha256 -cne (Get-ReleaseFileSha256 $producerScriptPath)) {
        throw "Package gate report was produced by a different build_release_package.ps1."
    }
    $canonicalReportPath = Get-PackageGateReportPath -RepoRoot $ExpectedRepoRoot -AppVersion $ExpectedAppVersion -Channel $ExpectedChannel
    if (-not (Test-ReleaseGateSamePath $ReportPath $canonicalReportPath)) {
        throw "Package gate report path is not canonical."
    }
    if ([string]$report.version -cne $ExpectedAppVersion `
        -or [string]$report.channel -cne $ExpectedChannel `
        -or [string]$report.appId -cne $script:ReleaseGateExpectedAppId `
        -or -not (Test-ReleaseGateSamePath ([string]$report.repoRoot) $ExpectedRepoRoot)) {
        throw "Package gate report is not bound to the requested repo/version/channel/AppId: $ReportPath"
    }
    $state = Assert-GitReleaseState -RepoRoot $ExpectedRepoRoot -AppVersion $ExpectedAppVersion -Channel $ExpectedChannel
    if ([string]$report.head -cne [string]$state.head `
        -or -not (Test-ReleaseGateSamePath ([string]$report.gitCommonDir) ([string]$state.gitCommonDir)) `
        -or -not (Test-ReleaseGateSamePath ([string]$report.gitWorktreeDir) ([string]$state.gitWorktreeDir))) {
        throw "Package gate report Git HEAD/common-dir/worktree identity is stale."
    }
    $packageDir = Resolve-ReleaseGatePath ([string]$report.packageDir)
    if (-not (Test-ReleaseGateSamePath $packageDir (Join-Path $ExpectedRepoRoot "dist\QtWidgetsApplication4")) `
        -or -not (Test-ReleaseGateSamePath ([string]$report.configMigrate.path) (Join-Path $ExpectedRepoRoot "dist\tools\ConfigMigrate.exe")) `
        -or -not (Test-ReleaseGateSamePath ([string]$report.configMigrateRun.path) (Join-Path $ExpectedRepoRoot "dist\tools\ConfigMigrate_Run.cmd"))) {
        throw "Package gate report contains a non-canonical dist path."
    }
    $actualInventory = @(Get-ReleaseFileInventory $packageDir)
    Assert-ReleaseInventoryMatches -Expected @($report.packageInventory) -Actual $actualInventory -Context "package gate report"
    $sourceVersionSha256 = Assert-SourceApplicationVersion -RepoRoot $ExpectedRepoRoot -AppVersion $ExpectedAppVersion
    if ($sourceVersionSha256 -cne [string]$report.sourceVersionSha256) {
        throw "Application version source changed after package gate creation."
    }
    $exe = Assert-ExpectedReleaseExecutable -Directory $packageDir -AppVersion $ExpectedAppVersion -Channel $ExpectedChannel
    Assert-ReleaseChannelAssets -RepoRoot $ExpectedRepoRoot -PackageDir $packageDir -Channel $ExpectedChannel
    if ((Get-ReleaseFileSha256 $exe) -cne [string]$report.executableSha256) {
        throw "Package executable hash changed after gate report creation."
    }
    $channelSpec = Get-ReleaseChannelSpec $ExpectedChannel
    $exeVersion = Assert-WindowsPeReleaseVersion `
        -Path $exe `
        -AppVersion $ExpectedAppVersion `
        -ExpectedProductName $channelSpec.AppName `
        -ExpectedOriginalFilename $channelSpec.ExeName
    if ([string]$report.executableVersion.machine -cne [string]$exeVersion.machine `
        -or [string]$report.executableVersion.fileVersion -cne [string]$exeVersion.fileVersion `
        -or [string]$report.executableVersion.productVersion -cne [string]$exeVersion.productVersion `
        -or [string]$report.executableVersion.productName -cne [string]$exeVersion.productName `
        -or [string]$report.executableVersion.originalFilename -cne [string]$exeVersion.originalFilename) {
        throw "Package executable PE version identity changed after gate report creation."
    }
    $fanuc = Assert-FanucRuntimeManifest -RepoRoot $ExpectedRepoRoot -RuntimeRoot $packageDir
    if ($fanuc.sha256 -cne [string]$report.fanucManifest.sha256) {
        throw "FANUC runtime manifest changed after package gate creation."
    }
    $config = Assert-ConfigMigrateProvenance -RepoRoot $ExpectedRepoRoot -ExecutablePath ([string]$report.configMigrate.path)
    if ($config.sha256 -cne [string]$report.configMigrate.sha256 `
        -or $config.sourceSha256 -cne [string]$report.configMigrate.sourceSha256 `
        -or $config.builderSha256 -cne [string]$report.configMigrate.builderSha256 `
        -or $config.isolatedRebuildSha256 -cne [string]$report.configMigrate.isolatedRebuildSha256 `
        -or $config.pythonSha256 -cne [string]$report.configMigrate.pythonSha256) {
        throw "ConfigMigrate provenance changed after package gate creation."
    }
    $configRunSourceHash = Get-ReleaseFileSha256 (Join-Path $ExpectedRepoRoot "tools\ConfigMigrate_Run.cmd")
    $configRunDistHash = Get-ReleaseFileSha256 ([string]$report.configMigrateRun.path)
    if ($configRunSourceHash -cne $configRunDistHash -or $configRunDistHash -cne [string]$report.configMigrateRun.sha256) {
        throw "ConfigMigrate_Run.cmd changed after package gate creation."
    }
    Assert-EmptyReleaseRuntimeDirectories $packageDir
    return $report
}

function Assert-InstallerProductVersion {
    param(
        [Parameter(Mandatory = $true)][string]$InstallerPath,
        [Parameter(Mandatory = $true)][string]$AppVersion,
        [Parameter(Mandatory = $true)][ValidateSet("neutral", "brand")][string]$Channel
    )

    if (-not (Test-Path -LiteralPath $InstallerPath -PathType Leaf)) {
        throw "Installer was not created: $InstallerPath"
    }
    $versionInfo = [System.Diagnostics.FileVersionInfo]::GetVersionInfo((Resolve-ReleaseGatePath $InstallerPath))
    $productVersion = $versionInfo.ProductVersion.Trim()
    if ($productVersion -cne $AppVersion) {
        throw "Installer ProductVersion mismatch: expected $AppVersion, actual $productVersion."
    }
    $expectedProductName = (Get-ReleaseChannelSpec $Channel).AppName
    if ($versionInfo.ProductName.Trim() -cne $expectedProductName) {
        throw "Installer ProductName/channel mismatch: expected $expectedProductName, actual $($versionInfo.ProductName.Trim())."
    }
}

function New-InstallerGateReport {
    param(
        [Parameter(Mandatory = $true)][string]$RepoRoot,
        [Parameter(Mandatory = $true)][string]$PackageGateReportPath,
        [Parameter(Mandatory = $true)][string]$InstallerPath,
        [Parameter(Mandatory = $true)][string]$AppVersion,
        [Parameter(Mandatory = $true)][ValidateSet("neutral", "brand")][string]$Channel,
        [string]$RunId = "",
        [string]$OutputPath = ""
    )

    $package = Assert-PackageGateReport -ReportPath $PackageGateReportPath -ExpectedRepoRoot $RepoRoot -ExpectedAppVersion $AppVersion -ExpectedChannel $Channel
    $spec = Get-ReleaseChannelSpec $Channel
    $expectedName = $spec.OutputPrefix + $AppVersion + ".exe"
    $canonicalInstallerPath = Join-Path $RepoRoot "dist\installer\$expectedName"
    if ([System.IO.Path]::GetFileName($InstallerPath) -cne $expectedName `
        -or -not (Test-ReleaseGateSamePath $InstallerPath $canonicalInstallerPath)) {
        throw "Installer filename/channel/version mismatch: expected $expectedName, got $([System.IO.Path]::GetFileName($InstallerPath))."
    }
    Assert-InstallerProductVersion -InstallerPath $InstallerPath -AppVersion $AppVersion -Channel $Channel
    $file = Get-Item -LiteralPath $InstallerPath
    if ($file.Length -le 0) {
        throw "Installer is empty: $InstallerPath"
    }
    if ([string]::IsNullOrWhiteSpace($OutputPath)) {
        $OutputPath = Get-InstallerGateReportPath -RepoRoot $RepoRoot -AppVersion $AppVersion -Channel $Channel
    }
    $canonicalOutputPath = Get-InstallerGateReportPath -RepoRoot $RepoRoot -AppVersion $AppVersion -Channel $Channel
    if (-not (Test-ReleaseGateSamePath $OutputPath $canonicalOutputPath)) {
        throw "Installer gate report must be written to its canonical dist/release-gates path."
    }
    $gateScriptPath = $script:ReleaseGateCommonPath
    if ([string]::IsNullOrWhiteSpace($RunId)) {
        $RunId = [Guid]::NewGuid().ToString("D")
    }
    $producerScriptPath = Join-Path $RepoRoot "scripts\build_installer.ps1"
    $report = [ordered]@{
        schemaVersion            = $script:ReleaseGateSchemaVersion
        kind                     = "installer"
        status                   = "pass"
        runId                    = $RunId
        generatedAtUtc           = [DateTime]::UtcNow.ToString("o")
        gateScriptSha256         = Get-ReleaseFileSha256 $gateScriptPath
        producerScriptSha256     = Get-ReleaseFileSha256 $producerScriptPath
        appId                    = $script:ReleaseGateExpectedAppId
        version                  = $AppVersion
        channel                  = $Channel
        head                     = [string]$package.head
        gitCommonDir             = [string]$package.gitCommonDir
        gitWorktreeDir           = [string]$package.gitWorktreeDir
        repoRoot                 = Resolve-ReleaseGatePath $RepoRoot
        packageGateReport        = Resolve-ReleaseGatePath $PackageGateReportPath
        packageGateReportSha256  = Get-ReleaseFileSha256 $PackageGateReportPath
        installer                = [ordered]@{
            path          = Resolve-ReleaseGatePath $InstallerPath
            name          = $expectedName
            size          = [int64]$file.Length
            sha256        = Get-ReleaseFileSha256 $InstallerPath
            productVersion = $AppVersion
        }
    }
    Write-ReleaseGateJson -Value $report -Path $OutputPath
    return (Resolve-ReleaseGatePath $OutputPath)
}

function Assert-InstallerGateReport {
    param([Parameter(Mandatory = $true)][string]$ReportPath)

    $report = Read-ReleaseGateJson $ReportPath
    if ([int]$report.schemaVersion -ne $script:ReleaseGateSchemaVersion `
        -or [string]$report.kind -cne "installer" `
        -or [string]$report.status -cne "pass" `
        -or [string]::IsNullOrWhiteSpace([string]$report.runId)) {
        throw "Unsupported installer gate report: $ReportPath"
    }
    Assert-ReleaseVersion ([string]$report.version)
    Get-ReleaseChannelSpec ([string]$report.channel) | Out-Null
    $gateScriptPath = $script:ReleaseGateCommonPath
    if ([string]$report.gateScriptSha256 -cne (Get-ReleaseFileSha256 $gateScriptPath)) {
        throw "Installer gate report was produced by a different release gate script."
    }
    $producerScriptPath = Join-Path ([string]$report.repoRoot) "scripts\build_installer.ps1"
    if ([string]$report.producerScriptSha256 -cne (Get-ReleaseFileSha256 $producerScriptPath)) {
        throw "Installer gate report was produced by a different build_installer.ps1."
    }
    $canonicalReportPath = Get-InstallerGateReportPath -RepoRoot ([string]$report.repoRoot) -AppVersion ([string]$report.version) -Channel ([string]$report.channel)
    if (-not (Test-ReleaseGateSamePath $ReportPath $canonicalReportPath)) {
        throw "Installer gate report path is not canonical."
    }
    if ([string]$report.appId -cne $script:ReleaseGateExpectedAppId) {
        throw "Installer gate AppId is not the immutable published AppId."
    }
    $repoRoot = [string]$report.repoRoot
    $packagePath = [string]$report.packageGateReport
    if ((Get-ReleaseFileSha256 $packagePath) -cne [string]$report.packageGateReportSha256) {
        throw "Package gate report was modified after installer gate creation."
    }
    $verifiedPackage = Assert-PackageGateReport -ReportPath $packagePath -ExpectedRepoRoot $repoRoot -ExpectedAppVersion ([string]$report.version) -ExpectedChannel ([string]$report.channel)
    if ([string]$report.head -cne [string]$verifiedPackage.head `
        -or -not (Test-ReleaseGateSamePath ([string]$report.gitCommonDir) ([string]$verifiedPackage.gitCommonDir)) `
        -or -not (Test-ReleaseGateSamePath ([string]$report.gitWorktreeDir) ([string]$verifiedPackage.gitWorktreeDir))) {
        throw "Installer gate Git HEAD/common-dir/worktree identity differs from its package gate."
    }
    $installerPath = [string]$report.installer.path
    $expectedInstallerPath = Join-Path ([string]$report.repoRoot) "dist\installer\$([string]$report.installer.name)"
    if (-not (Test-ReleaseGateSamePath $installerPath $expectedInstallerPath)) {
        throw "Installer gate report points outside the canonical dist/installer path."
    }
    $file = Get-Item -LiteralPath $installerPath -ErrorAction Stop
    if ($file.Name -cne [string]$report.installer.name `
        -or [int64]$file.Length -ne [int64]$report.installer.size `
        -or (Get-ReleaseFileSha256 $installerPath) -cne [string]$report.installer.sha256) {
        throw "Installer bytes no longer match installer gate report: $installerPath"
    }
    Assert-InstallerProductVersion -InstallerPath $installerPath -AppVersion ([string]$report.version) -Channel ([string]$report.channel)
    return $report
}
