# Docker build environment

This repo can build `MSenseDevice` firmware in a container instead of a local
nRF Connect for VS Code install. The image (`msense-sdk`) bakes in the
expensive, rarely-changing parts -- the nRF Connect SDK toolchain and a full
west workspace -- and your `MSenseDevice/` checkout is bind-mounted in at
`docker run` time, so `west build` runs directly against your working tree.

## Quick start

```bash
docker build -t msense-sdk .      # once, or when the SDK/toolchain pins below change
docker/build.sh                   # incremental west build -> MSenseDevice/build
```

`docker/build.sh` is a thin wrapper around `docker run` that mounts
`./MSenseDevice` into the container. Other useful invocations:

```bash
docker/build.sh --pristine        # clean rebuild (new board/config, weird build state)
docker/build.sh bash              # interactive shell with west/cmake/ninja/gcc on PATH
docker/build.sh west flash        # any other command, run inside the toolchain env
```

Or call `docker run` directly if you don't want the wrapper:

```bash
docker run --rm -v "$PWD/MSenseDevice:/opt/nordic/ncs/MSenseDevice" msense-sdk
```

Build output lands in `MSenseDevice/build/` exactly like a local build
(`merged.hex`, `merged_CPUNET.hex`, `dfu_application.zip`, etc.) -- it's
gitignored already.

## What's in the image

- **Toolchain**: installed via `nrfutil toolchain-manager`, pinned to NCS
  `v2.9.3` -- the same toolchain the nRF Connect for VS Code extension uses
  (CMake, Ninja, Python + a preconfigured venv with `west`, the
  `arm-zephyr-eabi` GCC toolchain, device-tree-compiler, gperf, etc., all
  self-contained).
- **West workspace**: the standard `sdk-nrf` `v2.9.3` manifest, but with the
  `zephyr` project swapped for SenSE Lab's custom fork/branch:
  `https://github.com/SenSE-Lab-OSU/SenSE-sdk-zephyr.git` @ `SenSE-ncs2.9.3`
  (this bakes in the local SDK changes documented in
  `MSenseDevice/changelist.txt`, e.g. the LFXO bypass Kconfig, so nothing
  needs to be patched at build time).
- **Not included**: the application source. `MSenseDevice/` is bind-mounted
  at `docker run` time -- the image has an empty `/opt/nordic/ncs/MSenseDevice`
  and expects something to be mounted there.

The image runs on `linux/amd64` (Nordic's toolchain bundles are x86_64-only);
the `FROM` line pins that platform explicitly so it also works via emulation
on Apple Silicon / other arm64 Docker hosts.

## Files

| File | Purpose |
|---|---|
| `Dockerfile` | Builds the `msense-sdk` image (toolchain + west workspace). |
| `docker/west.yml` | Local west manifest: imports `sdk-nrf` v2.9.3, overrides the `zephyr` project. |
| `docker/ncs-run` | Wraps a command with `nrfutil toolchain-manager launch` so it runs inside the toolchain env. |
| `docker/entrypoint.sh` | Container entrypoint -- picks the fast incremental-build path vs. `--pristine` vs. passthrough. |
| `docker/build.sh` | Convenience wrapper around `docker run` for local iteration. |
| `.dockerignore` | Keeps the build context small (`MSenseDevice/` isn't sent -- it's mounted, not copied). |

## How the zephyr swap works

`docker/west.yml` imports the real `sdk-nrf` manifest and then redefines the
`zephyr` project. West gives projects defined directly in the top-level
manifest precedence over same-named projects pulled in transitively via
`import:`, so this cleanly replaces zephyr without touching `sdk-nrf` itself.

Two fields on that override matter and are easy to get wrong (both are
copied verbatim from `sdk-nrf`'s own `zephyr` entry, since replacing the
project entry also discards everything sdk-nrf normally attaches to it):

- **`west-commands: scripts/west-commands.yml`** -- without this, `west`
  doesn't know zephyr provides extension commands, and things like
  `west build` and `west flash` fail with `unknown command`.
- **`import: name-allowlist: [...]`** -- without this, a curated list of
  projects that NCS pulls out of zephyr's *own* nested manifest (`hal_nordic`,
  `cmsis`, `fatfs`, `nrf_hw_models`, `mbedtls`'s Nordic bits via hal, etc.)
  never gets fetched. The build fails at CMake configure with `Unmet or
  cyclic dependencies in modules: nrfxlib depends on: ['hal_nordic']`.

Also note: `west init -l <path>` sets the workspace topdir to the *parent*
of `<path>`, not `<path>` itself -- that's why `docker/west.yml`'s
`self: path: manifest-repo` combined with the Dockerfile's
`mkdir -p ${NCS_TOPDIR}/manifest-repo` (manifest repo *inside* the topdir)
is what makes the topdir end up at `NCS_TOPDIR` (`/opt/nordic/ncs`) as
intended.

## Why incremental builds are fast (and how not to break it)

`west build` treats passing `--board` and the app source directory as a
request to (re)configure -- that reruns CMake's full generate step
(devicetree, Kconfig, `list_hardware.py`, sysbuild setup, ...), which costs
tens of seconds on its own even when nothing needs recompiling. Once the
build directory is already configured, `docker/entrypoint.sh` detects that
(`build/CMakeCache.txt` exists) and passes only `--build-dir`, so `west
build` goes straight to `ninja` instead.

Measured on Apple Silicon (via Rosetta):

| Scenario | Time |
|---|---|
| `docker build` (image + full west workspace), cold | ~10-15 min, one-time |
| `docker/build.sh --pristine` (clean firmware rebuild) | ~3.5 min |
| `docker/build.sh` with nothing changed | ~3 s |
| `docker/build.sh` after touching one source file | ~9 s |

This only holds for the entrypoint's own commands. If you pass extra
`west build` arguments yourself (e.g. `docker/build.sh west build --board
...`), you bypass the fast path in `docker/entrypoint.sh` and force a full
reconfigure every time, same as the `--board`/source-dir case above.

## Updating the SDK or toolchain version

Bump `ARG NCS_VERSION` in the `Dockerfile` and the `revision:` on both the
`nrf` and `zephyr` projects in `docker/west.yml` (the `zephyr` revision is
whatever branch/tag on the SenSE Lab fork matches that NCS version), then
`docker build -t msense-sdk .` again. The `import: name-allowlist` may also
need updating if it changed upstream -- diff against the `zephyr` project
entry in the real `sdk-nrf` manifest's `west.yml` for that version
(`docker/build.sh bash -c "cat nrf/west.yml"` after a build, or browse
`https://github.com/nrfconnect/sdk-nrf/blob/<tag>/west.yml`).

## Troubleshooting

- **`nrfutil: command not found` / toolchain install fails**: nRF Util's
  download URL and CLI have changed across releases. Check
  <https://docs.nordicsemi.com/bundle/nrfutil/page/README.html> for the
  current install method and update `NRFUTIL_URL` / the `nrfutil
  toolchain-manager` invocations in the `Dockerfile`.
- **`west: unknown command "build"` (or `flash`, `zephyr-export`, ...)**:
  the `west-commands:` field is missing from the `zephyr` project override
  in `docker/west.yml` -- see above.
- **`Unmet or cyclic dependencies in modules: ... depends on: ['hal_nordic']`**
  (or any other module): the `import: name-allowlist` on the `zephyr`
  project override is missing that project -- see above.
- **`west update` fails for several projects at once, or is very slow**:
  avoid `west update --narrow` and `-o=--depth=1` together -- some pinned
  projects reference a commit SHA that isn't at the tip of any branch, and
  not every git host allows a shallow fetch-by-SHA. A full (non-shallow)
  `west update` is what's currently in the `Dockerfile`; if you want shallow
  clones back for speed, keep `--narrow` off (it can silently skip nested
  manifest imports, causing the `hal_nordic`-style errors above) and test
  `-o=--depth=1` on its own first.
- **A `docker/build.sh` run takes ~50s+ even with nothing changed**: you're
  hitting the full-reconfigure path -- confirm you're not passing extra
  `west build` arguments (see the incremental-builds section above), and
  that `MSenseDevice/build/CMakeCache.txt` actually exists (a deleted or
  never-created build dir also takes this path, correctly, for its first run).
