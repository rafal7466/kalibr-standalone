#!/usr/bin/bash
# Installs the build dependencies of kalibr_cpp into an MSYS2 installation
# (a private one in <repo>/tools/msys64 or a system-wide C:\msys64).
# Run inside that MSYS2's bash, e.g.:  C:\msys64\usr\bin\bash.exe -lc "/e/path/to/repo/scripts/setup_msys2.sh"
set -x
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG="$ROOT/setup_msys2.log"
{
pacman-key --init
pacman-key --populate msys2
pacman -Syuu --noconfirm
pacman -Syuu --noconfirm
pacman -S --noconfirm --needed \
  mingw-w64-ucrt-x86_64-gcc mingw-w64-ucrt-x86_64-cmake mingw-w64-ucrt-x86_64-ninja \
  mingw-w64-ucrt-x86_64-boost mingw-w64-ucrt-x86_64-eigen3 mingw-w64-ucrt-x86_64-suitesparse \
  mingw-w64-ucrt-x86_64-opencv mingw-w64-ucrt-x86_64-yaml-cpp mingw-w64-ucrt-x86_64-openblas \
  mingw-w64-ucrt-x86_64-qt6-base
echo "SETUP_EXIT=$?"
} 2>&1 | tee "$LOG"
