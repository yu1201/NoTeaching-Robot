param(
    [string]$HostAddress = "",
    [int]$Port = 50006,
    [ValidateSet("Tcp", "Udp")]
    [string]$Protocol = "Tcp",
    [double]$Seconds = 10.0,
    [string]$Robot = "",
    [string]$CameraSection = "",
    [string]$ProjectRoot = "",
    [int]$ConnectTimeoutMs = 3000,
    [int]$ReadTimeoutMs = 2000,
    [switch]$UseConfiguredDevicePort,
    [switch]$Udp,
    [switch]$ShowWindow,
    [switch]$NoAutoStart
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$MagicNumber = [Convert]::ToUInt32("ABCDEF12", 16)
$QtNullLength = [Convert]::ToUInt32("FFFFFFFF", 16)
$HeaderSize = 24
$MaxPacketSize = 32 * 1024 * 1024

if ($Udp) {
    $Protocol = "Udp"
}
if ($Protocol -eq "Udp" -and $Port -eq 50006 -and -not $PSBoundParameters.ContainsKey("Port")) {
    $Port = 50004
}

function Get-ProjectRoot {
    param([string]$Root)

    if (-not [string]::IsNullOrWhiteSpace($Root)) {
        return (Resolve-Path -LiteralPath $Root).Path
    }

    $dir = (Get-Location).Path
    for ($i = 0; $i -lt 8; $i++) {
        if (Test-Path -LiteralPath (Join-Path $dir "QtWidgetsApplication4.sln")) {
            return $dir
        }
        $parent = Split-Path -Parent $dir
        if ([string]::IsNullOrWhiteSpace($parent) -or $parent -eq $dir) {
            break
        }
        $dir = $parent
    }
    return (Get-Location).Path
}

function Get-IniValue {
    param(
        [string]$Path,
        [string]$Section,
        [string]$Key
    )

    if (-not (Test-Path -LiteralPath $Path)) {
        return $null
    }

    $inSection = $false
    foreach ($line in Get-Content -LiteralPath $Path) {
        $trimmed = $line.Trim()
        if ($trimmed.Length -eq 0 -or $trimmed.StartsWith("#") -or $trimmed.StartsWith(";")) {
            continue
        }
        if ($trimmed -match '^\[(.+)\]$') {
            $inSection = ($Matches[1] -eq $Section)
            continue
        }
        if ($inSection -and $trimmed -match ('^{0}\s*=\s*(.*)$' -f [regex]::Escape($Key))) {
            return $Matches[1].Trim()
        }
    }
    return $null
}

function Get-DefaultRobot {
    param([string]$Root)

    $path = Join-Path $Root "Data\ContralUnitInfo.ini"
    $value = Get-IniValue -Path $path -Section "UnitName" -Key "Unit0"
    if ([string]::IsNullOrWhiteSpace($value)) {
        return "RobotA"
    }
    return $value
}

function Resolve-CameraEndpoint {
    param(
        [string]$Root,
        [string]$RobotName,
        [string]$Section,
        [int]$DefaultPort,
        [bool]$UseIniPort
    )

    if ([string]::IsNullOrWhiteSpace($RobotName)) {
        $RobotName = Get-DefaultRobot -Root $Root
    }

    $cameraIni = Join-Path $Root ("Data\{0}\CameraParam.ini" -f $RobotName)
    if ([string]::IsNullOrWhiteSpace($Section)) {
        $measureNo = Get-IniValue -Path $cameraIni -Section "Base" -Key "MeasureCameraNo"
        if ([string]::IsNullOrWhiteSpace($measureNo)) {
            $measureNo = "0"
        }
        $Section = "CAMERA$measureNo"
    }

    $address = Get-IniValue -Path $cameraIni -Section $Section -Key "DeviceAddress"
    $configuredPortText = Get-IniValue -Path $cameraIni -Section $Section -Key "DevicePort"
    $configuredPort = 0
    [void][int]::TryParse($configuredPortText, [ref]$configuredPort)

    $targetPort = $DefaultPort
    if ($UseIniPort -and $configuredPort -gt 0) {
        $targetPort = $configuredPort
    }

    [pscustomobject]@{
        Robot = $RobotName
        CameraSection = $Section
        CameraIni = $cameraIni
        DeviceAddress = $address
        ConfiguredDevicePort = $configuredPort
        TargetPort = $targetPort
    }
}

function Read-UInt32BE {
    param($Buffer, [int]$Offset)

    return ([uint32]$Buffer[$Offset] -shl 24) -bor
        ([uint32]$Buffer[$Offset + 1] -shl 16) -bor
        ([uint32]$Buffer[$Offset + 2] -shl 8) -bor
        ([uint32]$Buffer[$Offset + 3])
}

function Read-Int32BE {
    param($Buffer, [int]$Offset)

    return [int32](Read-UInt32BE -Buffer $Buffer -Offset $Offset)
}

function Read-Int64BE {
    param($Buffer, [int]$Offset)

    $value = [uint64]0
    for ($i = 0; $i -lt 8; $i++) {
        $value = ($value -shl 8) -bor [uint64]$Buffer[$Offset + $i]
    }
    return [int64]$value
}

function Read-DoubleBE {
    param([byte[]]$Buffer, [int]$Offset)

    $bytes = New-Object byte[] 8
    [Array]::Copy($Buffer, $Offset, $bytes, 0, 8)
    if ([BitConverter]::IsLittleEndian) {
        [Array]::Reverse($bytes)
    }
    return [BitConverter]::ToDouble($bytes, 0)
}

function Read-FloatBE {
    param([byte[]]$Buffer, [int]$Offset)

    $bytes = New-Object byte[] 4
    [Array]::Copy($Buffer, $Offset, $bytes, 0, 4)
    if ([BitConverter]::IsLittleEndian) {
        [Array]::Reverse($bytes)
    }
    return [BitConverter]::ToSingle($bytes, 0)
}

function Find-FrameHeader {
    param([System.Collections.Generic.List[byte]]$Buffer)

    if ($Buffer.Count -lt $HeaderSize) {
        return -1
    }

    $last = $Buffer.Count - $HeaderSize
    for ($i = 0; $i -le $last; $i++) {
        $magic = Read-UInt32BE -Buffer $Buffer -Offset $i
        if ($magic -ne $MagicNumber) {
            continue
        }
        $header = Read-UInt32BE -Buffer $Buffer -Offset ($i + 4)
        if ($header -ne $HeaderSize) {
            continue
        }
        $total = Read-UInt32BE -Buffer $Buffer -Offset ($i + 8)
        if ($total -lt ($HeaderSize + 4) -or $total -gt $MaxPacketSize) {
            continue
        }
        return $i
    }
    return -1
}

function Resolve-FrameSize {
    param(
        [System.Collections.Generic.List[byte]]$Buffer,
        [uint32]$HeaderTotalSize
    )

    $frameSize = [int]$HeaderTotalSize
    if ($Buffer.Count -lt ($HeaderSize + 4)) {
        return $frameSize
    }

    $bodySize = Read-UInt32BE -Buffer $Buffer -Offset $HeaderSize
    if ($bodySize -gt $MaxPacketSize) {
        return $frameSize
    }

    $qDataStreamPacketSize = [uint64]$HeaderSize + 4 + [uint64]$bodySize + 4
    $legacyHeaderTotalSize = [uint64]$HeaderSize + [uint64]$bodySize + 4
    if ([uint64]$HeaderTotalSize -eq $legacyHeaderTotalSize -or [uint64]$HeaderTotalSize -eq $qDataStreamPacketSize) {
        return [int]$qDataStreamPacketSize
    }
    return $frameSize
}

function Add-Bytes {
    param(
        [System.Collections.Generic.List[byte]]$Buffer,
        [byte[]]$Bytes,
        [int]$Count
    )

    $chunk = New-Object byte[] $Count
    [Array]::Copy($Bytes, 0, $chunk, 0, $Count)
    $Buffer.AddRange($chunk)
}

function Copy-FrameBytes {
    param(
        [System.Collections.Generic.List[byte]]$Buffer,
        [int]$Count
    )

    $frameBytes = New-Object byte[] $Count
    $Buffer.CopyTo(0, $frameBytes, 0, $Count)
    return $frameBytes
}

function Read-QtString {
    param(
        [byte[]]$Buffer,
        [int]$Offset,
        [int]$Limit
    )

    if ($Offset + 4 -gt $Limit) {
        return [pscustomobject]@{ Text = ""; NextOffset = $Offset; Ok = $false }
    }
    $length = Read-UInt32BE -Buffer $Buffer -Offset $Offset
    $Offset += 4
    if ($length -eq $QtNullLength) {
        return [pscustomobject]@{ Text = ""; NextOffset = $Offset; Ok = $true }
    }

    $byteLength = [int]$length
    if ($Offset + $byteLength -gt $Limit -and $Offset + ($byteLength * 2) -le $Limit) {
        $byteLength = $byteLength * 2
    }
    if ($Offset + $byteLength -gt $Limit) {
        return [pscustomobject]@{ Text = ""; NextOffset = $Offset; Ok = $false }
    }

    if ($byteLength -eq 0) {
        return [pscustomobject]@{ Text = ""; NextOffset = $Offset; Ok = $true }
    }

    $bytes = New-Object byte[] $byteLength
    [Array]::Copy($Buffer, $Offset, $bytes, 0, $byteLength)
    $text = [System.Text.Encoding]::BigEndianUnicode.GetString($bytes).TrimEnd([char]0)
    return [pscustomobject]@{ Text = $text; NextOffset = ($Offset + $byteLength); Ok = $true }
}

function Decode-PointCloudFrameSummary {
    param(
        [byte[]]$FrameBytes,
        [int]$FrameIndex
    )

    $summary = [ordered]@{
        FrameIndex = $FrameIndex
        FrameBytes = $FrameBytes.Length
        Timestamp = $null
        Channel = $null
        BodyBytes = $null
        Checksum = $null
        ImageBytes = $null
        ImageCols = $null
        ImageRows = $null
        ImageType = $null
        ImageStep = $null
        PointCount = $null
        FitPointCount = $null
        TargetX = $null
        TargetY = $null
        TargetZ = $null
        CalcFrameRate = $null
        ErrorMessage = ""
        FirstPoints = @()
        FitPoints = @()
        DecodeNote = "ok"
    }

    try {
        if ($FrameBytes.Length -lt ($HeaderSize + 8)) {
            throw "frame too short"
        }

        $summary.Timestamp = Read-Int64BE -Buffer $FrameBytes -Offset 12
        $summary.Channel = Read-Int32BE -Buffer $FrameBytes -Offset 20
        $bodyLength = Read-UInt32BE -Buffer $FrameBytes -Offset $HeaderSize
        $bodyOffset = $HeaderSize + 4
        $bodyLimit = $bodyOffset + [int]$bodyLength
        if ($bodyLimit + 4 -gt $FrameBytes.Length) {
            throw "body length exceeds frame bytes"
        }
        $summary.BodyBytes = [int]$bodyLength
        $summary.Checksum = Read-UInt32BE -Buffer $FrameBytes -Offset $bodyLimit

        $offset = $bodyOffset
        $imageBytes = Read-UInt32BE -Buffer $FrameBytes -Offset $offset
        $offset += 4
        if ($imageBytes -eq $QtNullLength) {
            $imageBytes = 0
        }
        $summary.ImageBytes = [int]$imageBytes
        if ($imageBytes -gt 0 -and $offset + [int]$imageBytes -le $bodyLimit) {
            if ($imageBytes -ge 16) {
                $summary.ImageCols = Read-Int32BE -Buffer $FrameBytes -Offset $offset
                $summary.ImageRows = Read-Int32BE -Buffer $FrameBytes -Offset ($offset + 4)
                $summary.ImageType = Read-Int32BE -Buffer $FrameBytes -Offset ($offset + 8)
                $summary.ImageStep = Read-Int32BE -Buffer $FrameBytes -Offset ($offset + 12)
            }
            $offset += [int]$imageBytes
        }
        elseif ($imageBytes -gt 0) {
            throw "image bytes exceed body"
        }

        $pointCount = Read-UInt32BE -Buffer $FrameBytes -Offset $offset
        $offset += 4
        $summary.PointCount = [int]$pointCount
        $firstPoints = New-Object System.Collections.Generic.List[string]
        for ($i = 0; $i -lt [int]$pointCount; $i++) {
            if ($offset + 24 -gt $bodyLimit) {
                throw "point data exceeds body"
            }
            $x = Read-DoubleBE -Buffer $FrameBytes -Offset $offset
            $y = Read-DoubleBE -Buffer $FrameBytes -Offset ($offset + 8)
            $z = Read-DoubleBE -Buffer $FrameBytes -Offset ($offset + 16)
            if ($i -lt 20) {
                $firstPoints.Add(("{0,4}: x={1,10:F3} y={2,10:F3} z={3,10:F3}" -f $i, $x, $y, $z))
            }
            $offset += 24
        }
        $summary.FirstPoints = $firstPoints.ToArray()

        $fitCount = Read-UInt32BE -Buffer $FrameBytes -Offset $offset
        $offset += 4
        $summary.FitPointCount = [int]$fitCount
        $fitPoints = New-Object System.Collections.Generic.List[string]
        for ($i = 0; $i -lt [int]$fitCount; $i++) {
            if ($offset + 16 -gt $bodyLimit) {
                throw "fit point data exceeds body"
            }
            $x = Read-DoubleBE -Buffer $FrameBytes -Offset $offset
            $y = Read-DoubleBE -Buffer $FrameBytes -Offset ($offset + 8)
            if ($i -lt 20) {
                $fitPoints.Add(("{0,4}: x={1,10:F3} y={2,10:F3}" -f $i, $x, $y))
            }
            $offset += 16
        }
        $summary.FitPoints = $fitPoints.ToArray()

        if ($offset + 24 -gt $bodyLimit) {
            throw "target point exceeds body"
        }
        $summary.TargetX = Read-DoubleBE -Buffer $FrameBytes -Offset $offset
        $summary.TargetY = Read-DoubleBE -Buffer $FrameBytes -Offset ($offset + 8)
        $summary.TargetZ = Read-DoubleBE -Buffer $FrameBytes -Offset ($offset + 16)
        $offset += 24

        $qtString = Read-QtString -Buffer $FrameBytes -Offset $offset -Limit $bodyLimit
        if ($qtString.Ok) {
            $summary.ErrorMessage = $qtString.Text
            $offset = $qtString.NextOffset
        }

        if ($offset + 4 -le $bodyLimit) {
            $summary.CalcFrameRate = Read-FloatBE -Buffer $FrameBytes -Offset $offset
        }
    }
    catch {
        $summary.DecodeNote = $_.Exception.Message
    }

    return [pscustomobject]$summary
}

function Format-ProbeReport {
    param($Result)

    $sample = $Result.SampleFrame
    $lines = New-Object System.Collections.Generic.List[string]
    $lines.Add(("Target: {0} {1}:{2}" -f $Result.Protocol, $Result.HostAddress, $Result.Port))
    $lines.Add(("Config: Robot={0}, Camera={1}, iniPort={2}" -f $Result.Robot, $Result.CameraSection, $Result.ConfiguredDevicePort))
    $lines.Add(("Sample: {0:N3}s, frames={1}, average={2:N2} fps, data={3:N2} Mbps" -f $Result.ElapsedSeconds, $Result.FrameCount, $Result.Fps, $Result.DataMbps))
    $lines.Add(("Frame timestamps: first={0}, last={1}" -f $Result.FirstTimestamp, $Result.LastTimestamp))
    $lines.Add(("Parser: datagrams={0}, filtered_datagrams={1}, filtered_bytes={2}, decode_failed={3}" -f $Result.DatagramCount, $Result.FilteredDatagrams, $Result.FilteredBytes, $Result.DecodeFailed))
    $lines.Add("")
    $lines.Add("Sample frame:")
    if ($null -eq $sample) {
        $lines.Add("  No sample frame saved.")
    }
    else {
        $lines.Add(("  index={0}, timestamp={1}, channel={2}, frameBytes={3}, bodyBytes={4}, checksum=0x{5:X8}" -f $sample.FrameIndex, $sample.Timestamp, $sample.Channel, $sample.FrameBytes, $sample.BodyBytes, $sample.Checksum))
        $lines.Add(("  imageBytes={0}, image={1}x{2}, type={3}, step={4}" -f $sample.ImageBytes, $sample.ImageCols, $sample.ImageRows, $sample.ImageType, $sample.ImageStep))
        $lines.Add(("  pointCount={0}, fitPointCount={1}, calcFrameRate={2:N2}" -f $sample.PointCount, $sample.FitPointCount, $sample.CalcFrameRate))
        $lines.Add(("  targetPoint=({0:N3}, {1:N3}, {2:N3})" -f $sample.TargetX, $sample.TargetY, $sample.TargetZ))
        if (-not [string]::IsNullOrWhiteSpace($sample.ErrorMessage)) {
            $lines.Add(("  errorMessage={0}" -f $sample.ErrorMessage))
        }
        if ($sample.DecodeNote -ne "ok") {
            $lines.Add(("  decodeNote={0}" -f $sample.DecodeNote))
        }
        $lines.Add("")
        $lines.Add("First 20 3D points:")
        foreach ($line in $sample.FirstPoints) {
            $lines.Add("  " + $line)
        }
        $lines.Add("")
        $lines.Add("First 20 fit points:")
        foreach ($line in $sample.FitPoints) {
            $lines.Add("  " + $line)
        }
    }
    return ($lines -join [Environment]::NewLine)
}

function Resolve-IPv4Address {
    param([string]$HostText)

    $address = $null
    if ([System.Net.IPAddress]::TryParse($HostText, [ref]$address)) {
        if ($address.AddressFamily -eq [System.Net.Sockets.AddressFamily]::InterNetwork) {
            return $address
        }
    }

    $addresses = [System.Net.Dns]::GetHostAddresses($HostText)
    foreach ($item in $addresses) {
        if ($item.AddressFamily -eq [System.Net.Sockets.AddressFamily]::InterNetwork) {
            return $item
        }
    }
    throw ("Cannot resolve IPv4 address: {0}" -f $HostText)
}

function Test-IPv4AddressEqual {
    param(
        [System.Net.IPAddress]$Left,
        [System.Net.IPAddress]$Right
    )

    if ($null -eq $Left -or $null -eq $Right) {
        return $false
    }
    $leftBytes = $Left.GetAddressBytes()
    $rightBytes = $Right.GetAddressBytes()
    if ($leftBytes.Length -ne $rightBytes.Length) {
        return $false
    }
    for ($i = 0; $i -lt $leftBytes.Length; $i++) {
        if ($leftBytes[$i] -ne $rightBytes[$i]) {
            return $false
        }
    }
    return $true
}

function Invoke-CameraFpsProbe {
    param(
        [string]$TargetHost,
        [int]$TargetPort,
        [string]$ProtocolName,
        [double]$DurationSeconds,
        [string]$RobotName,
        [string]$SectionName,
        [string]$RootPath,
        [int]$ConnectTimeout,
        [int]$ReadTimeout,
        [bool]$UseIniPort
    )

    $root = Get-ProjectRoot -Root $RootPath
    $endpoint = Resolve-CameraEndpoint -Root $root -RobotName $RobotName -Section $SectionName -DefaultPort $TargetPort -UseIniPort $UseIniPort

    if ([string]::IsNullOrWhiteSpace($TargetHost)) {
        $TargetHost = $endpoint.DeviceAddress
    }
    if ([string]::IsNullOrWhiteSpace($TargetHost)) {
        throw "Camera host is empty. Pass -HostAddress or check CameraParam.ini."
    }
    if ($DurationSeconds -le 0) {
        throw "-Seconds must be positive."
    }

    $protocolValue = $ProtocolName
    if ([string]::IsNullOrWhiteSpace($protocolValue)) {
        $protocolValue = "Tcp"
    }
    $protocolValue = (Get-Culture).TextInfo.ToTitleCase($protocolValue.ToLowerInvariant())

    if ($protocolValue -eq "Udp") {
        $remoteAddress = Resolve-IPv4Address -HostText $TargetHost
        $remoteEndpoint = [System.Net.IPEndPoint]::new($remoteAddress, $endpoint.TargetPort)
        $udp = [System.Net.Sockets.UdpClient]::new()
        $udp.Client.ExclusiveAddressUse = $false
        $udp.Client.SetSocketOption([System.Net.Sockets.SocketOptionLevel]::Socket, [System.Net.Sockets.SocketOptionName]::ReuseAddress, $true)
        $udp.Client.ReceiveBufferSize = 32 * 1024 * 1024
        $udp.Client.ReceiveTimeout = $ReadTimeout
        $udp.Client.Bind([System.Net.IPEndPoint]::new([System.Net.IPAddress]::Any, $endpoint.TargetPort))

        $buffer = [System.Collections.Generic.List[byte]]::new()
        $heartbeat = [System.Text.Encoding]::ASCII.GetBytes("HEARTBEAT")
        $sw = [System.Diagnostics.Stopwatch]::StartNew()
        $sampleStart = $null
        $deadline = $null
        $nextHeartbeatMs = 0
        $frameCount = 0
        $byteCount = 0
        $datagramCount = 0
        $filteredDatagrams = 0
        $decodeFailed = 0
        $filteredBytes = 0
        $firstTimestamp = $null
        $lastTimestamp = $null
        $sampleFrame = $null

        try {
            while ($true) {
                if ($null -ne $deadline -and $sw.Elapsed.TotalSeconds -ge $deadline) {
                    break
                }
                if ($null -eq $sampleStart -and $sw.Elapsed.TotalMilliseconds -ge $ConnectTimeout) {
                    break
                }
                if ($sw.Elapsed.TotalMilliseconds -ge $nextHeartbeatMs) {
                    [void]$udp.Send($heartbeat, $heartbeat.Length, $remoteEndpoint)
                    $nextHeartbeatMs = $sw.Elapsed.TotalMilliseconds + 300
                }

                $sender = [System.Net.IPEndPoint]::new([System.Net.IPAddress]::Any, 0)
                try {
                    $datagram = $udp.Receive([ref]$sender)
                }
                catch [System.Net.Sockets.SocketException] {
                    if ($null -ne $deadline -and $sw.Elapsed.TotalSeconds -ge $deadline) {
                        break
                    }
                    continue
                }
                $datagramCount++
                if (-not (Test-IPv4AddressEqual -Left $sender.Address -Right $remoteAddress)) {
                    $filteredDatagrams++
                    continue
                }

                Add-Bytes -Buffer $buffer -Bytes $datagram -Count $datagram.Length

                while ($true) {
                    $framePos = Find-FrameHeader -Buffer $buffer
                    if ($framePos -lt 0) {
                        break
                    }
                    if ($framePos -gt 0) {
                        $buffer.RemoveRange(0, $framePos)
                        $filteredBytes += $framePos
                    }
                    if ($buffer.Count -lt $HeaderSize) {
                        break
                    }

                    $totalSize = Read-UInt32BE -Buffer $buffer -Offset 8
                    if ($totalSize -lt $HeaderSize -or $totalSize -gt $MaxPacketSize) {
                        $buffer.RemoveAt(0)
                        $filteredBytes++
                        continue
                    }

                    $frameSize = Resolve-FrameSize -Buffer $buffer -HeaderTotalSize $totalSize
                    if ($frameSize -lt $HeaderSize -or $frameSize -gt ($MaxPacketSize + 4)) {
                        $buffer.RemoveAt(0)
                        $filteredBytes++
                        continue
                    }
                    if ($buffer.Count -lt $frameSize) {
                        break
                    }

                    $timestamp = Read-Int64BE -Buffer $buffer -Offset 12
                    $now = $sw.Elapsed.TotalSeconds
                    if ($null -eq $sampleStart) {
                        $sampleStart = $now
                        $deadline = $sampleStart + $DurationSeconds
                        $firstTimestamp = $timestamp
                    }

                    if ($now -le $deadline) {
                        $frameCount++
                        $byteCount += $frameSize
                        $lastTimestamp = $timestamp
                        if ($null -eq $sampleFrame) {
                            $frameBytes = Copy-FrameBytes -Buffer $buffer -Count $frameSize
                            $sampleFrame = Decode-PointCloudFrameSummary -FrameBytes $frameBytes -FrameIndex $frameCount
                            if ($sampleFrame.DecodeNote -ne "ok") {
                                $decodeFailed++
                            }
                        }
                    }

                    $buffer.RemoveRange(0, $frameSize)

                    if ($null -ne $deadline -and $now -ge $deadline) {
                        break
                    }
                }
            }
        }
        finally {
            $udp.Close()
        }

        if ($null -eq $sampleStart) {
            throw ("UDP bound to local port {0} but no complete frame was decoded from {1}." -f $endpoint.TargetPort, $TargetHost)
        }

        $elapsed = [Math]::Max(0.001, [Math]::Min($DurationSeconds, ($sw.Elapsed.TotalSeconds - $sampleStart)))
        $fps = $frameCount / $elapsed
        $mbps = ($byteCount * 8.0 / 1000000.0) / $elapsed

        return [pscustomobject]@{
            Protocol = $protocolValue
            HostAddress = $TargetHost
            Port = $endpoint.TargetPort
            Robot = $endpoint.Robot
            CameraSection = $endpoint.CameraSection
            CameraIni = $endpoint.CameraIni
            ConfiguredDevicePort = $endpoint.ConfiguredDevicePort
            ElapsedSeconds = $elapsed
            FrameCount = $frameCount
            Fps = $fps
            DataMbps = $mbps
            Bytes = $byteCount
            FirstTimestamp = $firstTimestamp
            LastTimestamp = $lastTimestamp
            FilteredBytes = $filteredBytes
            DecodeFailed = $decodeFailed
            DatagramCount = $datagramCount
            FilteredDatagrams = $filteredDatagrams
            SampleFrame = $sampleFrame
        }
    }

    $client = [System.Net.Sockets.TcpClient]::new()
    $async = $client.BeginConnect($TargetHost, $endpoint.TargetPort, $null, $null)
    if (-not $async.AsyncWaitHandle.WaitOne($ConnectTimeout)) {
        $client.Close()
        throw ("Connect timeout: {0}:{1} after {2} ms" -f $TargetHost, $endpoint.TargetPort, $ConnectTimeout)
    }
    $client.EndConnect($async)
    $client.NoDelay = $true
    $client.ReceiveTimeout = $ReadTimeout
    $stream = $client.GetStream()

    $buffer = [System.Collections.Generic.List[byte]]::new()
    $recv = New-Object byte[] 65536
    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    $sampleStart = $null
    $deadline = $null
    $frameCount = 0
    $byteCount = 0
    $decodeFailed = 0
    $filteredBytes = 0
    $firstTimestamp = $null
    $lastTimestamp = $null
    $sampleFrame = $null

    try {
        while ($true) {
            if ($null -ne $deadline -and $sw.Elapsed.TotalSeconds -ge $deadline) {
                break
            }

            try {
                $read = $stream.Read($recv, 0, $recv.Length)
            }
            catch [System.IO.IOException] {
                if ($null -ne $deadline -and $sw.Elapsed.TotalSeconds -ge $deadline) {
                    break
                }
                throw
            }
            if ($read -le 0) {
                break
            }
            Add-Bytes -Buffer $buffer -Bytes $recv -Count $read

            while ($true) {
                $framePos = Find-FrameHeader -Buffer $buffer
                if ($framePos -lt 0) {
                    break
                }
                if ($framePos -gt 0) {
                    $buffer.RemoveRange(0, $framePos)
                    $filteredBytes += $framePos
                }
                if ($buffer.Count -lt $HeaderSize) {
                    break
                }

                $totalSize = Read-UInt32BE -Buffer $buffer -Offset 8
                if ($totalSize -lt $HeaderSize -or $totalSize -gt $MaxPacketSize) {
                    $buffer.RemoveAt(0)
                    $filteredBytes++
                    continue
                }

                $frameSize = Resolve-FrameSize -Buffer $buffer -HeaderTotalSize $totalSize
                if ($frameSize -lt $HeaderSize -or $frameSize -gt ($MaxPacketSize + 4)) {
                    $buffer.RemoveAt(0)
                    $filteredBytes++
                    continue
                }
                if ($buffer.Count -lt $frameSize) {
                    break
                }

                $timestamp = Read-Int64BE -Buffer $buffer -Offset 12
                $now = $sw.Elapsed.TotalSeconds
                if ($null -eq $sampleStart) {
                    $sampleStart = $now
                    $deadline = $sampleStart + $DurationSeconds
                    $firstTimestamp = $timestamp
                }

                if ($now -le $deadline) {
                    $frameCount++
                    $byteCount += $frameSize
                    $lastTimestamp = $timestamp
                    if ($null -eq $sampleFrame) {
                        $frameBytes = Copy-FrameBytes -Buffer $buffer -Count $frameSize
                        $sampleFrame = Decode-PointCloudFrameSummary -FrameBytes $frameBytes -FrameIndex $frameCount
                    }
                }

                $buffer.RemoveRange(0, $frameSize)

                if ($null -ne $deadline -and $now -ge $deadline) {
                    break
                }
            }
        }
    }
    finally {
        $stream.Close()
        $client.Close()
    }

    if ($null -eq $sampleStart) {
        throw "Connected but no complete frame was decoded."
    }

    $elapsed = [Math]::Max(0.001, [Math]::Min($DurationSeconds, ($sw.Elapsed.TotalSeconds - $sampleStart)))
    $fps = $frameCount / $elapsed
    $mbps = ($byteCount * 8.0 / 1000000.0) / $elapsed

    return [pscustomobject]@{
        Protocol = $protocolValue
        HostAddress = $TargetHost
        Port = $endpoint.TargetPort
        Robot = $endpoint.Robot
        CameraSection = $endpoint.CameraSection
        CameraIni = $endpoint.CameraIni
        ConfiguredDevicePort = $endpoint.ConfiguredDevicePort
        ElapsedSeconds = $elapsed
        FrameCount = $frameCount
        Fps = $fps
        DataMbps = $mbps
        Bytes = $byteCount
        FirstTimestamp = $firstTimestamp
        LastTimestamp = $lastTimestamp
        FilteredBytes = $filteredBytes
        DecodeFailed = $decodeFailed
        DatagramCount = $null
        FilteredDatagrams = $null
        SampleFrame = $sampleFrame
    }
}

function Show-CameraFpsProbeWindow {
    Add-Type -AssemblyName System.Windows.Forms
    Add-Type -AssemblyName System.Drawing

    $form = New-Object System.Windows.Forms.Form
    $form.Text = "Camera FPS Probe"
    $form.StartPosition = "CenterScreen"
    $form.Size = New-Object System.Drawing.Size(980, 700)

    $font = New-Object System.Drawing.Font("Microsoft YaHei UI", 9)
    $monoFont = New-Object System.Drawing.Font("Consolas", 9)
    $form.Font = $font

    $topPanel = New-Object System.Windows.Forms.Panel
    $topPanel.Dock = "Top"
    $topPanel.Height = 112
    $form.Controls.Add($topPanel)

    $labelHost = New-Object System.Windows.Forms.Label
    $labelHost.Text = "IP"
    $labelHost.Location = New-Object System.Drawing.Point(14, 17)
    $labelHost.Size = New-Object System.Drawing.Size(32, 24)
    $topPanel.Controls.Add($labelHost)

    $hostBox = New-Object System.Windows.Forms.TextBox
    $hostBox.Location = New-Object System.Drawing.Point(52, 14)
    $hostBox.Size = New-Object System.Drawing.Size(150, 24)
    $hostBox.Text = $HostAddress
    $topPanel.Controls.Add($hostBox)

    $labelPort = New-Object System.Windows.Forms.Label
    $labelPort.Text = "Port"
    $labelPort.Location = New-Object System.Drawing.Point(218, 17)
    $labelPort.Size = New-Object System.Drawing.Size(40, 24)
    $topPanel.Controls.Add($labelPort)

    $portBox = New-Object System.Windows.Forms.TextBox
    $portBox.Location = New-Object System.Drawing.Point(264, 14)
    $portBox.Size = New-Object System.Drawing.Size(70, 24)
    $portBox.Text = $Port.ToString()
    $topPanel.Controls.Add($portBox)

    $labelSeconds = New-Object System.Windows.Forms.Label
    $labelSeconds.Text = "Sec"
    $labelSeconds.Location = New-Object System.Drawing.Point(350, 17)
    $labelSeconds.Size = New-Object System.Drawing.Size(28, 24)
    $topPanel.Controls.Add($labelSeconds)

    $secondsBox = New-Object System.Windows.Forms.TextBox
    $secondsBox.Location = New-Object System.Drawing.Point(380, 14)
    $secondsBox.Size = New-Object System.Drawing.Size(58, 24)
    $secondsBox.Text = $Seconds.ToString("0.###")
    $topPanel.Controls.Add($secondsBox)

    $labelProtocol = New-Object System.Windows.Forms.Label
    $labelProtocol.Text = "Mode"
    $labelProtocol.Location = New-Object System.Drawing.Point(456, 17)
    $labelProtocol.Size = New-Object System.Drawing.Size(42, 24)
    $topPanel.Controls.Add($labelProtocol)

    $protocolBox = New-Object System.Windows.Forms.ComboBox
    $protocolBox.DropDownStyle = "DropDownList"
    [void]$protocolBox.Items.Add("Tcp")
    [void]$protocolBox.Items.Add("Udp")
    $protocolBox.Location = New-Object System.Drawing.Point(502, 14)
    $protocolBox.Size = New-Object System.Drawing.Size(76, 24)
    $protocolBox.SelectedItem = $Protocol
    $topPanel.Controls.Add($protocolBox)

    $startButton = New-Object System.Windows.Forms.Button
    $startButton.Text = "Start"
    $startButton.Location = New-Object System.Drawing.Point(596, 12)
    $startButton.Size = New-Object System.Drawing.Size(92, 30)
    $topPanel.Controls.Add($startButton)

    $protocolBox.Add_SelectedIndexChanged({
        if ($protocolBox.SelectedItem -eq "Udp" -and $portBox.Text.Trim() -eq "50006") {
            $portBox.Text = "50004"
        }
        elseif ($protocolBox.SelectedItem -eq "Tcp" -and $portBox.Text.Trim() -eq "50004") {
            $portBox.Text = "50006"
        }
    })

    $statusLabel = New-Object System.Windows.Forms.Label
    $statusLabel.Text = "Ready"
    $statusLabel.Location = New-Object System.Drawing.Point(14, 54)
    $statusLabel.Size = New-Object System.Drawing.Size(860, 22)
    $topPanel.Controls.Add($statusLabel)

    $fpsLabel = New-Object System.Windows.Forms.Label
    $fpsLabel.Text = "Average FPS: --"
    $fpsLabel.Font = New-Object System.Drawing.Font("Microsoft YaHei UI", 16, [System.Drawing.FontStyle]::Bold)
    $fpsLabel.Location = New-Object System.Drawing.Point(14, 76)
    $fpsLabel.Size = New-Object System.Drawing.Size(860, 32)
    $topPanel.Controls.Add($fpsLabel)

    $outputBox = New-Object System.Windows.Forms.RichTextBox
    $outputBox.Dock = "Fill"
    $outputBox.Font = $monoFont
    $outputBox.ReadOnly = $true
    $outputBox.WordWrap = $false
    $outputBox.Text = "Click Start. The window will stay open after the test and show average FPS plus one sample frame."
    $form.Controls.Add($outputBox)

    $runProbe = {
        $startButton.Enabled = $false
        $statusLabel.Text = "Testing..."
        $fpsLabel.Text = "Average FPS: testing"
        $outputBox.Text = ""
        [System.Windows.Forms.Application]::DoEvents()

        try {
            $targetPort = 0
            if (-not [int]::TryParse($portBox.Text.Trim(), [ref]$targetPort)) {
                throw "Port must be a valid integer."
            }
            $duration = 0.0
            if (-not [double]::TryParse($secondsBox.Text.Trim(), [ref]$duration) -or $duration -le 0) {
                throw "Seconds must be greater than 0."
            }

            $result = Invoke-CameraFpsProbe `
                -TargetHost $hostBox.Text.Trim() `
                -TargetPort $targetPort `
                -ProtocolName $protocolBox.SelectedItem.ToString() `
                -DurationSeconds $duration `
                -RobotName $Robot `
                -SectionName $CameraSection `
                -RootPath $ProjectRoot `
                -ConnectTimeout $ConnectTimeoutMs `
                -ReadTimeout $ReadTimeoutMs `
                -UseIniPort ([bool]$UseConfiguredDevicePort)

            if ([string]::IsNullOrWhiteSpace($hostBox.Text.Trim())) {
                $hostBox.Text = $result.HostAddress
            }
            $fpsLabel.Text = ("Average FPS: {0:N2}    Frames: {1}    Sample: {2:N1}s" -f $result.Fps, $result.FrameCount, $result.ElapsedSeconds)
            $statusLabel.Text = ("Done: {0}:{1}" -f $result.HostAddress, $result.Port)
            $outputBox.Text = Format-ProbeReport -Result $result
        }
        catch {
            $fpsLabel.Text = "Average FPS: failed"
            $statusLabel.Text = "Failed"
            $outputBox.Text = $_.Exception.Message
        }
        finally {
            $startButton.Enabled = $true
        }
    }

    $startButton.Add_Click($runProbe)
    $form.Add_Shown({
        if ([string]::IsNullOrWhiteSpace($hostBox.Text)) {
            try {
                $root = Get-ProjectRoot -Root $ProjectRoot
                $endpoint = Resolve-CameraEndpoint -Root $root -RobotName $Robot -Section $CameraSection -DefaultPort $Port -UseIniPort ([bool]$UseConfiguredDevicePort)
                $hostBox.Text = $endpoint.DeviceAddress
            }
            catch {
            }
        }
        if (-not $NoAutoStart) {
            & $runProbe
        }
    })

    [void]$form.ShowDialog()
}

if ($ShowWindow) {
    Show-CameraFpsProbeWindow
    return
}

$result = Invoke-CameraFpsProbe `
    -TargetHost $HostAddress `
    -TargetPort $Port `
    -ProtocolName $Protocol `
    -DurationSeconds $Seconds `
    -RobotName $Robot `
    -SectionName $CameraSection `
    -RootPath $ProjectRoot `
    -ConnectTimeout $ConnectTimeoutMs `
    -ReadTimeout $ReadTimeoutMs `
    -UseIniPort ([bool]$UseConfiguredDevicePort)

Write-Host ("camera_fps_probe protocol={0} target={1}:{2} seconds={3:N3}" -f $result.Protocol, $result.HostAddress, $result.Port, $Seconds)
Write-Host ("config robot={0} section={1} iniPort={2} ini={3}" -f $result.Robot, $result.CameraSection, $result.ConfiguredDevicePort, $result.CameraIni)
if (-not $UseConfiguredDevicePort -and $result.ConfiguredDevicePort -gt 0 -and $result.ConfiguredDevicePort -ne $result.Port) {
    Write-Host ("note using fixed {0} point-cloud port {1}; CameraParam DevicePort is {2}" -f $result.Protocol, $result.Port, $result.ConfiguredDevicePort)
}
Write-Host ("result frames={0} elapsed_s={1:N3} fps={2:N2} data_mbps={3:N2} bytes={4}" -f $result.FrameCount, $result.ElapsedSeconds, $result.Fps, $result.DataMbps, $result.Bytes)
Write-Host ("timestamps first={0} last={1} datagrams={2} filtered_datagrams={3} filtered_bytes={4} decode_failed={5}" -f $result.FirstTimestamp, $result.LastTimestamp, $result.DatagramCount, $result.FilteredDatagrams, $result.FilteredBytes, $result.DecodeFailed)
Write-Host ""
Write-Host (Format-ProbeReport -Result $result)
