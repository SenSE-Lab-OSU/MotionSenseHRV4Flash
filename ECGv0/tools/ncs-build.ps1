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
    '--board', 'ecgv0/nrf5340/cpuapp'
    '--'
    '-DNCS_TOOLCHAIN_VERSION=NONE'
    '-DCONF_FILE=prj.conf'
)

& $west @westArgs
exit $LASTEXITCODE
