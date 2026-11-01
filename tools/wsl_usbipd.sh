#!/usr/bin/env bash
# Optional WSL-only USB/IP helper. Native Linux does not use this file.
set -euo pipefail

have_camera() { lsusb 2>/dev/null | grep -q '2bdf:'; }
USBIPD="$(command -v usbipd.exe 2>/dev/null || true)"
[[ -n "$USBIPD" ]] || { echo "[usbipd] usbipd.exe not found" >&2; exit 1; }

find_busid() {
  timeout 15 "$USBIPD" list 2>/dev/null | tr -d '\r' | awk '/2bdf:0001/ {print $1; exit}'
}
attach_once() { timeout 20 "$USBIPD" attach --wsl --busid "$1" 2>&1 | tr -d '\r'; }
detach_once() { timeout 15 "$USBIPD" detach --busid "$1" >/dev/null 2>&1 || true; }

attach_camera() {
  local busid output attempt
  busid="$(find_busid)"
  [[ -n "$busid" ]] || { echo "[usbipd] camera not found" >&2; return 1; }
  output="$(attach_once "$busid")"
  case "$output" in
    *"Device busy"*)
      timeout 10 taskkill.exe /IM MVS.exe /F >/dev/null 2>&1 || true
      sleep 2
      output="$(attach_once "$busid")"
      ;;
  esac
  case "$output" in
    *"not shared"*|*"not bound"*|*"not been shared"*)
      timeout 15 "$USBIPD" bind --busid "$busid" >/dev/null 2>&1 || return 1
      output="$(attach_once "$busid")"
      ;;
  esac
  for attempt in {1..10}; do have_camera && return 0; sleep 1; done
  detach_once "$busid"
  sleep 1
  attach_once "$busid" >/dev/null || true
  for attempt in {1..10}; do have_camera && return 0; sleep 1; done
  return 1
}

watch_camera() {
  local failures=0
  while sleep 3; do
    if have_camera; then failures=0; continue; fi
    if attach_camera; then
      echo "[usbipd] camera re-attached to WSL"
      failures=0
    else
      failures=$((failures + 1))
      [[ "$failures" -ge 3 ]] && echo "[usbipd] re-attach failed; retrying in background" >&2
    fi
  done
}

case "${1:---attach}" in
  --attach) attach_camera ;;
  --watch) watch_camera ;;
  *) echo "usage: $0 [--attach|--watch]" >&2; exit 2 ;;
esac
