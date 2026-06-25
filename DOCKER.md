# Building with Docker

## Prerequisites

1. Clone the repository:

```
git clone https://github.com/SenSE-Lab-OSU/MotionSenseHRV4Flash.git
```

2. If Docker is not installed, follow [the installation guide](https://docs.docker.com/engine/install/).

## Setup

**Option A — pre-built image (NCS v2.5.3):**

```
cd MSenseDevice
docker run -it -v ${PWD}/:/workdir/project agilemelon/msense_nrf bash
```

**Option B — build the image locally (NCS v2.9.3):**

```
cd MSenseDevice
docker build -t msense_nrf_v293 .
docker run -it -v ${PWD}/:/workdir/project msense_nrf_v293 bash
```

> The `Dockerfile` in `MSenseDevice/` applies the same SDK patches as
> `msense_ncs_v2.5.3.patch` (FatFS f_expand, NORTC, and USB MSC block size)
> against NCS v2.9.3. The first build downloads the full NCS workspace
> (~10 GB) and will take 20–40 minutes depending on your connection.
> Works on macOS (Intel and Apple Silicon), Linux, and Windows via Docker Desktop.
>
> **Windows note:** use `%cd%` instead of `${PWD}` in Command Prompt;
> `${PWD}` works as-is in PowerShell and Git Bash.

## Compile the firmware

Inside the container (same for both options):

```
cd project
west build -p always -b nrf5340dk_nrf5340_cpuapp
```

## Output artifacts

After a successful compile (NCS v2.9.3 / sysbuild):

| File | Description |
|------|-------------|
| `MSenseDevice/build/merged.hex` | Full flash image (app + MCUboot + net core) |
| `MSenseDevice/build/dfu_application.zip` | DFU package for OTA/USB update |

> NCS v2.5.3 (Option A) placed these under `build/zephyr/merged_domains.hex`
> and `build/zephyr/dfu_application.zip` instead.
