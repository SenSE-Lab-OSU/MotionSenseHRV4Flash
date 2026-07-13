[CmdletBinding()]
param(
    [string]$NcsRoot = 'C:\ncs\v2.9.3',
    [string]$ToolchainRoot = 'C:\ncs\toolchains\b620d30767'
)

$ErrorActionPreference = 'Stop'

$bin = Join-Path $ToolchainRoot 'opt\bin'
$scripts = Join-Path $bin 'Scripts'

foreach ($requiredPath in @(
    $NcsRoot,
    (Join-Path $NcsRoot 'zephyr'),
    (Join-Path $scripts 'west.exe'),
    (Join-Path $bin 'cmake.exe'),
    (Join-Path $bin 'ninja.exe'),
    (Join-Path $bin 'python.exe')
)) {
    if (-not (Test-Path -LiteralPath $requiredPath)) {
        throw "Required nRF Connect SDK path is missing: $requiredPath"
    }
}

$env:NCS_ROOT = $NcsRoot
$env:ZEPHYR_BASE = Join-Path $NcsRoot 'zephyr'
$env:WEST_PYTHON = Join-Path $bin 'python.exe'
$env:NCS_TOOLCHAIN_ROOT = $ToolchainRoot

# Codex inherits both Path and PATH. PowerShell resolves the former, while the
# Python process that runs west resolves the latter. Replace both with one
# canonical PATH so west can locate CMake and Ninja.
$existingPath = [System.Environment]::GetEnvironmentVariable('Path', 'Process')
[System.Environment]::SetEnvironmentVariable('Path', $null, 'Process')
[System.Environment]::SetEnvironmentVariable('PATH', "$scripts;$bin;$existingPath", 'Process')

# Codex runs under a different Windows identity than the owner of the NCS
# checkout. Limit the Git trust exception to this SDK and this process tree.
$gitSafeDirectory = ($NcsRoot -replace '\\', '/').TrimEnd([char]'/') + '/*'
$env:GIT_CONFIG_COUNT = '1'
$env:GIT_CONFIG_KEY_0 = 'safe.directory'
$env:GIT_CONFIG_VALUE_0 = $gitSafeDirectory
