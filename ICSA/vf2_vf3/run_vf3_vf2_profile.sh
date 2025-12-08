#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EXECUTABLE="${SCRIPT_DIR}/vf3_vs_vf2"
DATA_ROOT="${SCRIPT_DIR}/../si"
START_INDEX=${1:-0}
STRIDE=${2:-1}

if [[ ${STRIDE} -le 0 ]]; then
    echo "Stride must be a positive integer."
    exit 1
fi

if [[ ! -x "${EXECUTABLE}" ]]; then
    echo "Executable ${EXECUTABLE} not found or not executable."
    exit 1
fi

mapfile -t DIRS < <(find "${DATA_ROOT}" -mindepth 1 -maxdepth 1 -type d | sort)

for ((idx=START_INDEX; idx<${#DIRS[@]}; idx+=STRIDE)); do
    dir="${DIRS[idx]}"
    for subdir in "${dir}"/*; do
        files=("${subdir}"/*)
        pattern="${files[0]}"
        target="${files[1]}"
        perf_csv="${SCRIPT_DIR}/results/perf_results_${START_INDEX}.csv"
        mismatch_csv="${SCRIPT_DIR}/results/mismatches_${START_INDEX}.csv"
        aborted_csv="${SCRIPT_DIR}/results/aborted_${START_INDEX}.csv"
        echo "Running ${EXECUTABLE} with pattern=${pattern}, target=${target}"
        ./vf3_vs_vf2 "${target}" "${pattern}" "${perf_csv}" "${mismatch_csv}" "${aborted_csv}"
    done
done
