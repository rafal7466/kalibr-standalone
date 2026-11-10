#!/usr/bin/bash
# Configure + build kalibr_cpp with an MSYS2 UCRT64 toolchain.
#
# From an "MSYS2 UCRT64" shell, in the repository root:
#   ./scripts/build.sh                            # build everything
#   ./scripts/build.sh kalibr_calibrate_cameras   # build selected targets
#
# Environment: JOBS=<n> overrides the number of parallel jobs.
set -o pipefail
export MSYSTEM=UCRT64
export PATH=/ucrt64/bin:/usr/bin:$PATH

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
JOBS="${JOBS:-$(nproc)}"
LOG="$ROOT/kalibr_cpp/build.log"
cd "$ROOT/kalibr_cpp" || exit 1

{
  echo "=== configure $(date)"
  cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DKALIBR_SRC_DIR="$ROOT/kalibr-master" || exit 1
  echo "=== build $(date) targets: ${*:-all}"
  if [ $# -eq 0 ]; then
    cmake --build build -j "$JOBS"
  else
    cmake --build build -j "$JOBS" --target "$@"
  fi
  rc=$?
  echo "BUILD_EXIT=$rc $(date)"
  exit $rc
} 2>&1 | tee "$LOG"
