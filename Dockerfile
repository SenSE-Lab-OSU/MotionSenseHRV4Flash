# syntax=docker/dockerfile:1
#
# A reusable build ENVIRONMENT for the MSenseDevice nRF5340 firmware --
# not a one-shot firmware build. The image bakes in the expensive,
# rarely-changing parts:
#   - the default/official nRF Connect SDK toolchain, installed via
#     nRF Util's toolchain-manager and pinned to NCS v2.9.3 (the same
#     toolchain the nRF Connect for VS Code extension uses)
#   - a full west workspace (sdk-nrf v2.9.3) with a custom SenSE Lab fork
#     of Zephyr in place of Nordic's stock zephyr:
#     https://github.com/SenSE-Lab-OSU/SenSE-sdk-zephyr.git @ SenSE-ncs2.9.3
#
# The zephyr swap is done via a small local west manifest (docker/west.yml)
# that imports the standard sdk-nrf v2.9.3 manifest and then redefines the
# "zephyr" project to point at the SenSE Lab fork/branch. West gives
# projects defined in the top-level manifest precedence over same-named
# projects pulled in transitively via `import:`, so nothing in sdk-nrf
# itself needs to be patched.
#
# The application source is NOT baked in -- it's bind-mounted from your
# checkout at `docker run` time, so builds happen directly against your
# working tree, `build/` lands in MSenseDevice/build like a normal local
# build, and ninja's incremental cache persists across runs on the host.
#
# Usage:
#   docker build -t msense-sdk .        # once, or whenever the SDK/toolchain
#                                        # pins in this Dockerfile change
#   docker run --rm -v "$PWD/MSenseDevice:/opt/nordic/ncs/MSenseDevice" msense-sdk
#     -> incremental `west build`, output in ./MSenseDevice/build
#   docker run --rm -v "$PWD/MSenseDevice:/opt/nordic/ncs/MSenseDevice" msense-sdk --pristine
#     -> clean rebuild (~4 min on Apple Silicon via Rosetta; longer under QEMU)
#   docker run --rm -it -v "$PWD/MSenseDevice:/opt/nordic/ncs/MSenseDevice" msense-sdk bash
#     -> interactive shell in the build environment (west, cmake, ninja,
#        the arm-zephyr-eabi toolchain, etc. all on PATH via `ncs-run`)
#
# See also docker/build.sh, a thin wrapper around the docker run command
# (also handles TTY allocation for the `bash` case above).
#
# Once the build dir is configured, a fresh `docker run` (no args) with
# nothing changed finishes in a few seconds, and a single touched source
# file adds only a few more -- ninja does a real incremental build, not
# a fresh container's worth of CMake reconfigure. That only holds if you
# don't pass extra `west build` args yourself: doing so bypasses the
# entrypoint's fast path (see docker/entrypoint.sh) and forces a full
# CMake reconfigure (tens of seconds) even with nothing to rebuild.
#
# nRF Util's CLI and download URL have changed across releases; if the
# `nrfutil` / `toolchain-manager` steps below fail, check
# https://docs.nordicsemi.com/bundle/nrfutil/page/README.html for the
# current install method and adjust accordingly.

# Nordic's toolchain-manager bundles are published for x86_64 Linux; pin
# the build platform so this also works (via QEMU emulation) from an
# Apple Silicon or other arm64 Docker host.
FROM --platform=linux/amd64 ubuntu:22.04 AS builder

ARG NCS_VERSION=v2.9.3
ARG BOARD=nrf5340dk/nrf5340/cpuapp
ARG NRFUTIL_URL=https://developer.nordicsemi.com/.pc-tools/nrfutil/x64-linux/nrfutil

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    NCS_VERSION=${NCS_VERSION} \
    BOARD=${BOARD} \
    NCS_TOPDIR=/opt/nordic/ncs \
    APP_DIR=/opt/nordic/ncs/MSenseDevice

# --- Minimal host dependencies -----------------------------------------
# The nRF Connect SDK toolchain bundle (installed below) is self
# contained: it ships its own CMake, Ninja, Python (+ venv with west and
# friends), device-tree-compiler, gperf, and the GNU Arm Embedded
# toolchain. We only need enough on the host to fetch and run it.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates curl git unzip xz-utils \
    && rm -rf /var/lib/apt/lists/*

# --- nRF Util + the default nRF Connect SDK toolchain -------------------
RUN curl -fL "${NRFUTIL_URL}" -o /usr/local/bin/nrfutil \
    && chmod +x /usr/local/bin/nrfutil \
    && nrfutil install toolchain-manager \
    && nrfutil toolchain-manager install --ncs-version "${NCS_VERSION}"

COPY docker/ncs-run /usr/local/bin/ncs-run
RUN chmod +x /usr/local/bin/ncs-run

# --- west workspace: sdk-nrf v2.9.3 with zephyr swapped for the custom fork
#
# `west init -l <path>` makes the *parent* of <path> the workspace
# topdir, so the manifest repo (self: path: manifest-repo in west.yml)
# must live directly inside NCS_TOPDIR for topdir to end up there.
RUN mkdir -p ${NCS_TOPDIR}/manifest-repo
COPY docker/west.yml ${NCS_TOPDIR}/manifest-repo/west.yml
RUN git -C ${NCS_TOPDIR}/manifest-repo init -q \
    && git -C ${NCS_TOPDIR}/manifest-repo add west.yml \
    && git -C ${NCS_TOPDIR}/manifest-repo -c user.email=docker@local -c user.name=docker \
        commit -q -m "manifest"

WORKDIR ${NCS_TOPDIR}
RUN ncs-run west init -l ${NCS_TOPDIR}/manifest-repo \
    && ncs-run west update \
    && ncs-run python3 -m pip install --no-cache-dir \
        -r zephyr/scripts/requirements.txt \
        -r nrf/scripts/requirements.txt \
        -r bootloader/mcuboot/scripts/requirements.txt \
    && ncs-run west zephyr-export

# --- entrypoint -------------------------------------------------------
# APP_DIR is intentionally empty here: mount MSenseDevice from the host
# at `docker run` time (see the Usage block above / docker/build.sh).
COPY docker/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh \
    && mkdir -p ${APP_DIR}

VOLUME ${APP_DIR}
WORKDIR ${APP_DIR}
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
