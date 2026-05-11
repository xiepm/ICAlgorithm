#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${1:-build}"
BUILD_TYPE="${BUILD_TYPE:-Release}"
if [[ -z "${JOBS:-}" ]]; then
	if command -v nproc >/dev/null 2>&1; then
		JOBS="$(nproc)"
	else
		JOBS="4"
	fi
else
	JOBS="${JOBS}"
fi

BUILD_PATH="${ROOT_DIR}/${BUILD_DIR}"

echo "Root: ${ROOT_DIR}"
echo "Build dir: ${BUILD_DIR}"
echo "Build type: ${BUILD_TYPE}"
echo "Jobs: ${JOBS}"

cd "${ROOT_DIR}"
mkdir -p "${BUILD_PATH}"

if cmake --help 2>/dev/null | grep -q -- "^-S <path-to-source>"; then
	if cmake -S "${ROOT_DIR}" -B "${BUILD_PATH}" -DCMAKE_BUILD_TYPE="${BUILD_TYPE}"; then
		echo "Configured with cmake -S/-B."
	else
		echo "Falling back to classic CMake configure mode."
		(
			cd "${BUILD_PATH}"
			cmake -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" "${ROOT_DIR}"
		)
	fi
else
	echo "Detected older CMake. Using classic configure mode."
	(
		cd "${BUILD_PATH}"
		cmake -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" "${ROOT_DIR}"
	)
fi

cmake --build "${BUILD_PATH}" -- -j"${JOBS}"

echo "Build finished."
