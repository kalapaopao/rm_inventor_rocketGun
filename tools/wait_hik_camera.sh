#!/bin/sh
# 等待海康 USB 相机枚举（VID 2bdf），供 systemd ExecStartPre 使用。
# 用法: wait_hik_camera.sh [超时秒数，默认 30]
# 找到返回 0；超时返回 1（服务会按 Restart 策略重试）。
TIMEOUT="${1:-30}"
i=0
while [ "$i" -lt "$TIMEOUT" ]; do
    if lsusb 2>/dev/null | grep -q '2bdf:'; then
        exit 0
    fi
    sleep 1
    i=$((i + 1))
done
echo "wait_hik_camera: 未在 ${TIMEOUT}s 内检测到海康相机（lsusb 2bdf:）" >&2
exit 1
