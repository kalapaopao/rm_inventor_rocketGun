#!/usr/bin/env bash
# Linux/WSL-first build and run entry point.
# Source files stay on the Windows drive and are accessed directly from WSL.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD="${BUILD_DIR:-$HERE/build}"
WATCHDOG_PID=""

cleanup() {
  if [[ -n "$WATCHDOG_PID" ]]; then
    kill "$WATCHDOG_PID" 2>/dev/null || true
    wait "$WATCHDOG_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

if [[ "${1:-}" != "--help" && "${1:-}" != "-h" ]]; then
  running="$(pgrep -x show 2>/dev/null || true)"
  if [[ -n "$running" && -z "${HIKCAM_ALLOW_MULTI:-}" ]]; then
    echo "[run] show is already running (PID: $running); refusing to share the camera." >&2
    echo "[run] Set HIKCAM_ALLOW_MULTI=1 only when parallel capture is intentional." >&2
    exit 1
  fi
fi

# WSL-only USB/IP support lives in a helper. The Windows .bat launcher is kept.
if grep -qi microsoft /proc/version 2>/dev/null && command -v usbipd.exe >/dev/null 2>&1; then
  if ! lsusb 2>/dev/null | grep -q '2bdf:'; then
    bash "$HERE/tools/wsl_usbipd.sh" --attach || \
      echo "[run] camera attach failed; continuing with the build." >&2
  fi
  if [[ -z "${HIKCAM_NO_WATCHDOG:-}" ]]; then
    bash "$HERE/tools/wsl_usbipd.sh" --watch &
    WATCHDOG_PID=$!
  fi
fi

mkdir -p "$BUILD"
# Keep the existing build directory to avoid a second copy of build artifacts.
cmake -S "$HERE/Camera" -B "$BUILD" -DCMAKE_BUILD_TYPE=Release
cmake --build "$BUILD" -j"$(nproc)"

PRELOAD=""
if grep -qi microsoft /proc/version 2>/dev/null; then
  if [[ ! -f "$BUILD/noreset.so" ]]; then
    printf 'struct libusb_device_handle;\nint libusb_reset_device(struct libusb_device_handle *d){(void)d;return 0;}\n' \
      > "$BUILD/noreset.c"
    gcc -shared -fPIC -o "$BUILD/noreset.so" "$BUILD/noreset.c" || true
  fi
  [[ -f "$BUILD/noreset.so" ]] && PRELOAD="$BUILD/noreset.so"
fi

cd "$BUILD"
if [[ -n "$PRELOAD" ]]; then
  LD_PRELOAD="$PRELOAD" ./show "$@"
else
  ./show "$@"
fi
