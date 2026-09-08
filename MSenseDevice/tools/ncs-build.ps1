[CmdletBinding()]
param(
    [switch]$Pristine,
    [string]$BuildDirectory,
    [string]$NcsRoot,
    [string]$ToolchainRoot
)

$ErrorActionPreference = 'Stop'

$environmentArguments = @{}

if ($PSBoundParameters.ContainsKey('NcsRoot')) {
    $environmentArguments.NcsRoot = $NcsRoot
}

if ($PSBoundParameters.ContainsKey('ToolchainRoot')) {
    $environmentArguments.ToolchainRoot = $ToolchainRoot
}

. (Join-Path $PSScriptRoot 'ncs-env.ps1') @environmentArguments

$app = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path

if (-not $BuildDirectory) {
    $BuildDirectory = Join-Path $app 'build'
}

$west = Join-Path $env:NCS_TOOLCHAIN_ROOT 'opt\bin\Scripts\west.exe'

$westArgs = @(
    'build'
    '--build-dir', $BuildDirectory
    $app
)

if ($Pristine) {
    $westArgs += '--pristine'
}

$westArgs += @(
    '--board', 'nrf5340dk/nrf5340/cpuapp'
    '--'
    '-DNCS_TOOLCHAIN_VERSION=NONE'
    '-DCONF_FILE=prj.conf'
    '-DDTC_OVERLAY_FILE=nrf5340dk_nrf5340_cpuapp.overlay'
)

& $west @westArgs
exit $LASTEXITCODE
