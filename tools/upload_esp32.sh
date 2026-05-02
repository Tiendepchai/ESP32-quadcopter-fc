#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  tools/upload_esp32.sh [PORT]

Environment overrides:
  PORT=/dev/cu.usbserial-2130
  FQBN=esp32:esp32:esp32
  UPLOAD_SPEED=115200
  TMP_SKETCH_DIR=/private/tmp/fc_esp32_quad
  BUILD_PATH=/private/tmp/fc_esp32_build

Notes:
  - This script copies the repo to a temporary sketch directory whose name
    matches the .ino filename, because arduino-cli expects that layout.
  - Upload defaults to 115200 because 921600 was unstable on this board/link.
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

if ! command -v arduino-cli >/dev/null 2>&1; then
  echo "arduino-cli not found in PATH" >&2
  exit 1
fi

if ! command -v rsync >/dev/null 2>&1; then
  echo "rsync not found in PATH" >&2
  exit 1
fi

readonly REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
readonly SKETCH_NAME="fc_esp32_quad"
readonly FQBN="${FQBN:-esp32:esp32:esp32}"
readonly UPLOAD_SPEED="${UPLOAD_SPEED:-115200}"
readonly TMP_SKETCH_DIR="${TMP_SKETCH_DIR:-/private/tmp/${SKETCH_NAME}}"
readonly BUILD_PATH="${BUILD_PATH:-/private/tmp/${SKETCH_NAME}_build}"

PORT="${PORT:-${1:-}}"

detect_port() {
  local detected=""

  detected="$(arduino-cli board list | awk '$1 ~ /\/dev\/cu\.(usbserial|usbmodem)/ { print $1; exit }')"
  if [[ -n "${detected}" ]]; then
    printf '%s\n' "${detected}"
    return 0
  fi

  detected="$(ls /dev/cu.usbserial* /dev/cu.usbmodem* 2>/dev/null | head -n 1 || true)"
  if [[ -n "${detected}" ]]; then
    printf '%s\n' "${detected}"
    return 0
  fi

  return 1
}

if [[ -z "${PORT}" ]]; then
  if ! PORT="$(detect_port)"; then
    echo "No ESP32 serial port found. Pass PORT or run: tools/upload_esp32.sh /dev/cu.usbserial-XXXX" >&2
    exit 1
  fi
fi

echo "repo       : ${REPO_ROOT}"
echo "port       : ${PORT}"
echo "fqbn       : ${FQBN}"
echo "baud       : ${UPLOAD_SPEED}"
echo "tmp sketch : ${TMP_SKETCH_DIR}"
echo "build path : ${BUILD_PATH}"

mkdir -p "${TMP_SKETCH_DIR}" "${BUILD_PATH}"
rsync -a --delete --exclude .git "${REPO_ROOT}/" "${TMP_SKETCH_DIR}/"

if [[ ! -f "${TMP_SKETCH_DIR}/${SKETCH_NAME}.ino" ]]; then
  echo "Missing sketch file: ${TMP_SKETCH_DIR}/${SKETCH_NAME}.ino" >&2
  exit 1
fi

arduino-cli compile \
  --fqbn "${FQBN}" \
  --build-path "${BUILD_PATH}" \
  "${TMP_SKETCH_DIR}"

arduino-cli upload \
  -p "${PORT}" \
  --fqbn "${FQBN}" \
  --build-path "${BUILD_PATH}" \
  "${TMP_SKETCH_DIR}" \
  --upload-property "upload.speed=${UPLOAD_SPEED}" \
  -t \
  -v
