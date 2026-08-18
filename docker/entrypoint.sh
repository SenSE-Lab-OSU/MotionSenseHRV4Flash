#!/usr/bin/env bash
# Default entrypoint for the msense-sdk dev environment image.
#
# APP_DIR (/opt/nordic/ncs/MSenseDevice) is expected to be bind-mounted
# from the host at `docker run` time, so build/ lands directly in your
# host checkout and ninja's incremental cache persists across runs.
#
# With no arguments: runs an incremental `west build` (no --pristine),
# so repeat runs after small source edits are fast. Pass `--pristine`
# (or any other west/ncs-run/shell command) to override.
set -euo pipefail

if [ ! -d "${APP_DIR}" ] || [ -z "$(ls -A "${APP_DIR}" 2>/dev/null)" ]; then
	echo "error: ${APP_DIR} is empty -- mount your MSenseDevice checkout there, e.g.:" >&2
	echo "  docker run --rm -v \"\$PWD/MSenseDevice:${APP_DIR}\" msense-sdk" >&2
	exit 1
fi

cd "${APP_DIR}"

if [ "$#" -eq 0 ]; then
	# `west build` treats passing --board/the source dir as a request to
	# (re)configure, which reruns CMake's full generate step (devicetree,
	# Kconfig, list_hardware.py, ...) even with nothing to do -- ~50s on
	# its own here, dwarfing the actual incremental compile. Once the
	# build dir is already configured, drop those and pass only
	# --build-dir so west goes straight to `ninja` (a no-op or
	# single-file rebuild finishes in a few seconds instead).
	if [ -f "${APP_DIR}/build/CMakeCache.txt" ]; then
		exec ncs-run west build --build-dir "${APP_DIR}/build"
	fi
	exec ncs-run west build \
		--board "${BOARD}" \
		--build-dir "${APP_DIR}/build" \
		"${APP_DIR}" \
		-- -DNCS_TOOLCHAIN_VERSION=NONE
fi

if [ "$1" = "--pristine" ]; then
	shift
	exec ncs-run west build \
		--pristine \
		--board "${BOARD}" \
		--build-dir "${APP_DIR}/build" \
		"${APP_DIR}" \
		"$@" \
		-- -DNCS_TOOLCHAIN_VERSION=NONE
fi

# Anything else (bash, west flash, west build with custom args, ...) runs
# inside the toolchain environment automatically.
exec ncs-run "$@"
