# nRF Connect SDK build scripts

`ncs-env.ps1` configures a PowerShell process to use the nRF Connect SDK and
toolchain required by this application. `ncs-build.ps1` loads that environment
and runs the same sysbuild-oriented `west build` configuration used by the
existing VS Code build directory.

The scripts do not change machine-wide environment variables, VS Code settings,
or the nRF Connect VS Code extension. They affect only the PowerShell process
that runs them and any child processes it starts.

## Build this project

From the `MSenseDevice` directory, run:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\tools\ncs-build.ps1 -Pristine
```

Omit `-Pristine` for an incremental build. The default output directory is
`MSenseDevice\build`; do not run this script and a VS Code build against that
same directory at the same time.

## Required installation

This project is currently configured and verified with:

| Component | Required location |
| --- | --- |
| nRF Connect SDK | `C:\ncs\v2.9.3` |
| Nordic toolchain | `C:\ncs\toolchains\b620d30767` |
| Application | `MSenseDevice` |
| Target board | `nrf5340dk/nrf5340/cpuapp` |

Install the exact SDK version through Nordic's nRF Connect tooling and ensure
the SDK workspace has been initialized. From the SDK root, the following should
succeed in a normal NCS terminal:

```powershell
west topdir
west list zephyr nrf
```

The project uses sysbuild to include MCUboot and the nRF5340 network-core HCI
image. Do not replace the supplied `west build` command with a direct CMake
build or a single-image build.

## Using another machine or installation path

Supply the installed locations directly to the build script:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\tools\ncs-build.ps1 `
  -NcsRoot 'D:\ncs\v2.9.3' `
  -ToolchainRoot 'D:\ncs\toolchains\b620d30767' `
  -Pristine
```

The same `-NcsRoot` and `-ToolchainRoot` parameters are available on
`ncs-env.ps1` when an interactive shell needs the environment. The toolchain
and SDK versions must remain compatible with the project; do not silently
substitute a newer NCS release.

The scripts require these files within the supplied paths:

```text
<NcsRoot>\zephyr
<ToolchainRoot>\opt\bin\python.exe
<ToolchainRoot>\opt\bin\cmake.exe
<ToolchainRoot>\opt\bin\ninja.exe
<ToolchainRoot>\opt\bin\Scripts\west.exe
```

## Git ownership when invoked by Codex

Codex can run under a different Windows identity from the person who installed
the SDK. Git then rejects the SDK repositories as "dubious ownership," which
prevents west from resolving imported manifests. `ncs-env.ps1` derives a
process-local `safe.directory` exception from `-NcsRoot`; it does not modify
global Git configuration.

## SDK modifications

The active SDK checkout contains project-specific modifications that are not
made by these scripts. Reproducing a build on another computer requires the
same SDK revision and all documented project patches. Verify and reconcile the
repository's patch instructions before relying on a fresh NCS installation.
