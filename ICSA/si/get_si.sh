#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARCHIVE_URL="https://liris.cnrs.fr/csolnon/newSIPbenchmarks.tgz"
ARCHIVE_PATH="${SCRIPT_DIR}/newSIPbenchmarks.tgz"
WORK_DIR="$(mktemp -d "${SCRIPT_DIR}/si_download.XXXXXX")"

cleanup() {
  rm -f "${ARCHIVE_PATH}"
  rm -rf "${WORK_DIR}"
}
trap cleanup EXIT

wget -O "${ARCHIVE_PATH}" "${ARCHIVE_URL}"
tar -xzf "${ARCHIVE_PATH}" -C "${WORK_DIR}"

SI_DIR="$(find "${WORK_DIR}" -type d -name "si" -print -quit)"
if [[ -z "${SI_DIR}" ]]; then
  echo "Unable to locate si directory in extracted archive" >&2
  exit 1
fi

find "${SI_DIR}" -mindepth 1 -maxdepth 1 -type d -exec cp -R {} "${SCRIPT_DIR}/" \;

echo "SI benchmarks copied to ${SCRIPT_DIR}"
