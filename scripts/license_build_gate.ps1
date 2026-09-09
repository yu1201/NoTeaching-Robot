function Get-LicenseBuildSpec {
    param(
        [Parameter(Mandatory)][ValidateSet('neutral', 'brand')][string]$Channel,
        [ValidateSet('', 'Off', 'Audit', 'Enforce')][string]$Mode = '',
        [string]$PublicKeyHeader = ''
    )
    if ([string]::IsNullOrWhiteSpace($Mode)) {
        $Mode = if ($Channel -eq 'brand') { 'Enforce' } else { 'Off' }
    }
    $modeValue = @{ Off = 0; Audit = 1; Enforce = 2 }[$Mode]
    $keyId = ''
    $keyBytes = [byte[]]@()
    $headerPath = ''
    if (-not [string]::IsNullOrWhiteSpace($PublicKeyHeader)) {
        $headerPath = (Resolve-Path -LiteralPath $PublicKeyHeader -ErrorAction Stop).Path
        $header = Get-Content -LiteralPath $headerPath -Raw -Encoding UTF8
        $keyMatch = [regex]::Matches($header, '(?m)^\s*#define\s+HK_LICENSE_PUBLIC_KEY_B64\s+"([A-Za-z0-9+/=]+)"\s*$')
        $idMatch = [regex]::Matches($header, '(?m)^\s*#define\s+HK_LICENSE_KEY_ID\s+"([A-Za-z0-9_-]{1,128})"\s*$')
        if ($keyMatch.Count -ne 1 -or $idMatch.Count -ne 1 -or $header -match 'PRIVATE KEY') {
            throw 'License header must contain exactly one public CNG blob and one key ID.'
        }
        $keyBytes = [Convert]::FromBase64String($keyMatch[0].Groups[1].Value)
        if ($keyBytes.Length -ne 411 -or [BitConverter]::ToUInt32($keyBytes, 0) -ne 0x31415352 `
            -or [BitConverter]::ToUInt32($keyBytes, 4) -ne 3072 `
            -or [BitConverter]::ToUInt32($keyBytes, 8) -ne 3 `
            -or [BitConverter]::ToUInt32($keyBytes, 12) -ne 384 `
            -or [BitConverter]::ToUInt32($keyBytes, 16) -ne 0 `
            -or [BitConverter]::ToUInt32($keyBytes, 20) -ne 0) {
            throw 'License verifier must use a public RSA-3072 CNG blob.'
        }
        $keyId = $idMatch[0].Groups[1].Value
    }
    if ($modeValue -ne 0 -and [string]::IsNullOrWhiteSpace($headerPath)) {
        throw 'Enabled licensing requires -LicensePublicKeyHeader from the independent license authority.'
    }
    $sha = [Security.Cryptography.SHA256]::Create()
    try { $keyHash = ([BitConverter]::ToString($sha.ComputeHash($keyBytes))).Replace('-', '').ToLowerInvariant() }
    finally { $sha.Dispose() }
    [pscustomobject]@{ Channel = $Channel; Mode = $Mode; Value = $modeValue;
        Header = $headerPath; KeyId = $keyId; PublicKeySha256 = $keyHash }
}

function Assert-ExecutableLicenseBuild {
    param([Parameter(Mandatory)][string]$Executable, [Parameter(Mandatory)]$Expected)
    # A Windows GUI executable is otherwise launched asynchronously by PowerShell.
    $start = [Diagnostics.ProcessStartInfo]::new($Executable, '--print-license-build-json')
    $start.UseShellExecute = $false
    $start.CreateNoWindow = $true
    $start.RedirectStandardOutput = $true
    $start.RedirectStandardError = $true
    $process = [Diagnostics.Process]::new()
    $process.StartInfo = $start
    try {
        [void]$process.Start()
        $outputTask = $process.StandardOutput.ReadToEndAsync()
        $errorTask = $process.StandardError.ReadToEndAsync()
        if (-not $process.WaitForExit(20000)) {
            $process.Kill()
            throw 'Executable license metadata probe timed out.'
        }
        $raw = $outputTask.GetAwaiter().GetResult()
        $probeError = $errorTask.GetAwaiter().GetResult()
        if ($process.ExitCode -ne 0) { throw "Executable license metadata probe failed ($($process.ExitCode)): $probeError" }
    }
    finally { $process.Dispose() }
    try { $actual = ($raw -join "`n") | ConvertFrom-Json -ErrorAction Stop }
    catch { throw 'Executable did not return valid license build metadata.' }
    if ($actual.schemaVersion -ne 1 -or $actual.licenseMode -ne $Expected.Value `
        -or $actual.licenseChannel -cne $Expected.Channel -or $actual.keyId -cne $Expected.KeyId `
        -or $actual.publicKeySha256 -cne $Expected.PublicKeySha256) {
        throw 'Executable license mode/channel/public key does not match the requested build.'
    }
    return $actual
}
