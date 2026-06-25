# Build environment for MotionSenseHRV4 firmware using nRF Connect SDK v2.9.3
#
# Usage:
#   cd MSenseDevice
#   docker build -t msense_nrf_v293 .
#   docker run -it -v ${PWD}/:/workdir/project msense_nrf_v293 bash
#   # Inside container:
#   cd project && west build -p always -b nrf5340dk_nrf5340_cpuapp
#
# Applies the same SDK patches as msense_ncs_v2.5.3.patch:
#   - FatFS: FF_USE_EXPAND=1, FF_FS_NORTC=1
#   - USB MSC: BLOCK_SIZE 512 -> 4096 (NAND flash compatibility)

FROM ubuntu:22.04

# Automatically set by Docker BuildKit to the host architecture.
# Drives which Zephyr SDK binary is downloaded:
#   arm64  (Apple Silicon, Windows-on-ARM) -> linux-aarch64
#   amd64  (Intel/AMD, most Windows/Linux) -> linux-x86_64
ARG TARGETARCH
ARG NCS_VERSION=v2.9.3
# Check NCS v2.9.3 release notes for the required Zephyr SDK version.
# For NCS v2.5.x the SDK was 0.16.3; adjust this ARG if the build fails.
ARG ZEPHYR_SDK_VERSION=0.17.0
ARG DEBIAN_FRONTEND=noninteractive

# System dependencies required by Zephyr/NCS build system
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    cmake \
    ninja-build \
    gperf \
    ccache \
    dfu-util \
    device-tree-compiler \
    wget \
    python3-dev \
    python3-pip \
    python3-setuptools \
    python3-tk \
    python3-wheel \
    xz-utils \
    file \
    make \
    gcc \
    libsdl2-dev \
    libmagic1 \
    && rm -rf /var/lib/apt/lists/*

# Install west (Zephyr meta-tool / workspace manager)
RUN pip3 install --no-cache-dir west

# Install Zephyr SDK (provides arm-zephyr-eabi cross-compiler and host tools)
# Map Docker's TARGETARCH values to Zephyr SDK archive naming.
RUN case "${TARGETARCH}" in \
        arm64) SDK_ARCH="aarch64" ;; \
        *)     SDK_ARCH="x86_64"  ;; \
    esac \
    && wget -q \
        "https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v${ZEPHYR_SDK_VERSION}/zephyr-sdk-${ZEPHYR_SDK_VERSION}_linux-${SDK_ARCH}_minimal.tar.xz" \
        -O /tmp/zephyr-sdk.tar.xz \
    && tar xf /tmp/zephyr-sdk.tar.xz -C /opt \
    && rm /tmp/zephyr-sdk.tar.xz \
    && /opt/zephyr-sdk-${ZEPHYR_SDK_VERSION}/setup.sh -t arm-zephyr-eabi -h -c

# Initialize the NCS west workspace at /workdir so that west can find .west
# when building from /workdir/project (west searches upward for .west/).
WORKDIR /workdir
RUN west init -m https://github.com/nrfconnect/sdk-nrf --mr ${NCS_VERSION} . \
    && west update \
    && west zephyr-export

# Install Python packages required by NCS and its sub-projects
RUN pip3 install --no-cache-dir \
    -r /workdir/zephyr/scripts/requirements.txt \
    -r /workdir/nrf/scripts/requirements.txt \
    -r /workdir/bootloader/mcuboot/scripts/requirements.txt

# Apply SDK patches equivalent to msense_ncs_v2.5.3.patch using Python
# (string replacement is robust across SDK versions; patch context lines
# may not match exactly between v2.5.3 and v2.9.3).

# Patch 1: FatFS ffconf.h
#   FF_USE_EXPAND 0 -> 1  (enable f_expand function)
#   FF_FS_NORTC   0 -> 1  (disable RTC dependency, uses compile-time constants)
RUN python3 - <<'EOF'
import pathlib, sys

matches = list(pathlib.Path('/workdir').rglob('ffconf.h'))
if not matches:
    sys.exit('ERROR: ffconf.h not found in /workdir')

for p in matches:
    text = p.read_text()
    original = text
    text = text.replace('#define FF_USE_EXPAND\t0', '#define FF_USE_EXPAND\t1')
    text = text.replace('#define FF_FS_NORTC\t\t0', '#define FF_FS_NORTC\t\t1')
    if text == original:
        sys.exit(f'ERROR: expected substitutions not found in {p}')
    p.write_text(text)
    print(f'Patched {p}')
EOF

# Patch 2: USB MSC msc.c
#   BLOCK_SIZE 512 -> 4096  (match NAND flash erase block size)
RUN python3 - <<'EOF'
import pathlib, sys

matches = [p for p in pathlib.Path('/workdir').rglob('msc.c')
           if 'usb' in str(p) and 'class' in str(p)]
if not matches:
    sys.exit('ERROR: USB msc.c not found in /workdir')

for p in matches:
    text = p.read_text()
    original = text
    text = text.replace('#define BLOCK_SIZE\t512', '#define BLOCK_SIZE\t4096')
    if text == original:
        sys.exit(f'ERROR: expected substitution not found in {p}')
    p.write_text(text)
    print(f'Patched {p}')
EOF

ENV ZEPHYR_BASE=/workdir/zephyr

# Start at /workdir so `cd project` in the README goes to /workdir/project
# (the volume mount target). The west workspace .west/ is also here, so
# west can locate it from any subdirectory.
WORKDIR /workdir
CMD ["bash"]
