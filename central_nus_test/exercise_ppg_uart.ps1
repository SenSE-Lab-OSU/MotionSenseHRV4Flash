[CmdletBinding()]
param(
    [string[]]$Ports = @('COM22', 'COM23'),
    [string]$CapturePath = 'C:\nathan\MotionSenseHRV4Flash\central_nus_test\captures\ppg_nrf54l15_live.mrly'
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

function New-TestPort {
    param(
        [Parameter(Mandatory)][string]$Name,
        [System.IO.Ports.Handshake]$Handshake = [System.IO.Ports.Handshake]::None
    )

    $port = [System.IO.Ports.SerialPort]::new($Name, 115200,
        [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
    $port.Handshake = $Handshake
    $port.DtrEnable = $true
    $port.ReadTimeout = 100
    $port.WriteTimeout = 1000
    $port.NewLine = "`r`n"
    return $port
}

function Read-AvailableText {
    param([Parameter(Mandatory)][System.IO.Ports.SerialPort]$Port)

    if ($Port.BytesToRead -eq 0) {
        return ''
    }
    return $Port.ReadExisting()
}

function Send-Command {
    param(
        [Parameter(Mandatory)][System.IO.Ports.SerialPort]$Port,
        [Parameter(Mandatory)][string]$Command
    )

    Write-Output "COMMAND_SEND port=$($Port.PortName) command=$Command"
    $Port.Write("$Command`r`n")
}

if ($Ports.Count -ne 2) {
    throw 'Exactly two candidate ports are required.'
}

$commandPortName = $null
foreach ($portName in $Ports) {
    $probe = New-TestPort -Name $portName
    try {
        $probe.Open()
        $probe.DiscardInBuffer()
        Send-Command -Port $probe -Command 'help'
        Start-Sleep -Milliseconds 800
        $response = Read-AvailableText -Port $probe
        $printable = $response -replace '[^\x09\x0A\x0D\x20-\x7E]', '.'
        Write-Output "PROBE port=$portName response=$($printable.Trim())"
        if ($response -match 'COMMANDS:') {
            $commandPortName = $portName
        }
    }
    finally {
        if ($probe.IsOpen) {
            $probe.Close()
        }
        $probe.Dispose()
    }
}

if ($null -eq $commandPortName) {
    throw 'Neither candidate port responded to help as the command channel.'
}
$relayPortName = @($Ports | Where-Object { $_ -ne $commandPortName })[0]
Write-Output "PORT_MAPPING command=$commandPortName relay=$relayPortName"

$commandPort = New-TestPort -Name $commandPortName
$relayPort = New-TestPort -Name $relayPortName -Handshake RequestToSend
$capture = [System.Collections.Generic.List[byte]]::new()
$commandText = ''
$terminalResult = $null

try {
    $commandPort.Open()
    $relayPort.Open()
    $commandPort.DiscardInBuffer()
    $relayPort.DiscardInBuffer()

    Send-Command -Port $commandPort -Command 'status'
    Start-Sleep -Milliseconds 500
    $statusText = Read-AvailableText -Port $commandPort
    Write-Output "STATUS_RESPONSE=$($statusText.Trim())"

    if ($statusText -notmatch 'state=(READY|COMPLETE)') {
        Send-Command -Port $commandPort -Command 'connect ppg'
        $connectDeadline = [DateTime]::UtcNow.AddSeconds(30)
        while ([DateTime]::UtcNow -lt $connectDeadline) {
            $text = Read-AvailableText -Port $commandPort
            if ($text.Length -gt 0) {
                $commandText += $text
                Write-Output ($text.Trim())
            }
            if ($commandText -match 'NUS_READY') {
                break
            }
            if ($commandText -match '(ERROR|ERR connect|DISCONNECTED)') {
                throw "PPG connection failed: $commandText"
            }
            Start-Sleep -Milliseconds 100
        }
        if ($commandText -notmatch 'NUS_READY') {
            throw "Timed out waiting for NUS_READY. Output: $commandText"
        }
    }

    Send-Command -Port $commandPort -Command 'start'
    $streamDeadline = [DateTime]::UtcNow.AddSeconds(75)
    $terminalAt = $null
    while ([DateTime]::UtcNow -lt $streamDeadline) {
        $text = Read-AvailableText -Port $commandPort
        if ($text.Length -gt 0) {
            $commandText += $text
            Write-Output ($text.Trim())
            if ($text -match '(STREAM_OK[^\r\n]*|STREAM_END[^\r\n]*|START_RESULT[^\r\n]*|PROTOCOL_ERROR[^\r\n]*)') {
                $terminalResult = $Matches[1]
                $terminalAt = [DateTime]::UtcNow
            }
        }

        $available = $relayPort.BytesToRead
        if ($available -gt 0) {
            $buffer = [byte[]]::new($available)
            $read = $relayPort.Read($buffer, 0, $buffer.Length)
            for ($index = 0; $index -lt $read; $index++) {
                $capture.Add($buffer[$index])
            }
        }

        if ($null -ne $terminalAt -and
            ([DateTime]::UtcNow - $terminalAt).TotalSeconds -ge 3 -and
            $relayPort.BytesToRead -eq 0) {
            break
        }
        Start-Sleep -Milliseconds 50
    }

    [IO.File]::WriteAllBytes($CapturePath, $capture.ToArray())

    $frameCount = 0
    $payloadBytes = 0
    $offset = 0
    while ($offset + 12 -le $capture.Count) {
        if ($capture[$offset] -ne 0x4d -or $capture[$offset + 1] -ne 0x52 -or
            $capture[$offset + 2] -ne 0x4c -or $capture[$offset + 3] -ne 0x59) {
            $offset++
            continue
        }
        $length = [int]$capture[$offset + 6] -bor ([int]$capture[$offset + 7] -shl 8)
        if ($length -gt 512 -or $offset + 12 + $length -gt $capture.Count) {
            break
        }
        $frameCount++
        $payloadBytes += $length
        $offset += 12 + $length
    }

    Write-Output "CAPTURE path=$CapturePath bytes=$($capture.Count) frames=$frameCount payload_bytes=$payloadBytes"
    Write-Output "TERMINAL_RESULT=$terminalResult"
    if ($null -eq $terminalResult -or $terminalResult -notmatch '^STREAM_OK') {
        exit 2
    }
}
finally {
    foreach ($port in @($commandPort, $relayPort)) {
        if ($port.IsOpen) {
            $port.Close()
        }
        $port.Dispose()
    }
}
