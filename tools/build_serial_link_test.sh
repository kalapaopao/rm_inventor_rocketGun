#!/usr/bin/env bash
# 编译板端独立串口链路测试工具（不依赖相机、不依赖内核 ch341 驱动）。
#
#   bash tools/build_serial_link_test.sh            # 产物: build/serial_link_test
#   bash tools/build_serial_link_test.sh /tmp/out   # 自定义产物路径
#
# 依赖：gcc + libusb-1.0 开发头文件。板端缺头文件时：
#   sudo apt-get install -y libusb-1.0-0-dev
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC="$HERE/tools/serial_link_test.c"
OUT="${1:-$HERE/build/serial_link_test}"

if [ ! -f "$SRC" ]; then
    echo "缺少源码: $SRC" >&2
    exit 1
fi
if ! command -v gcc >/dev/null 2>&1; then
    echo "缺少 gcc，请先安装: sudo apt-get install -y build-essential" >&2
    exit 1
fi
if [ ! -f /usr/include/libusb-1.0/libusb.h ]; then
    echo "缺少 libusb 开发头文件，请先安装:" >&2
    echo "  sudo apt-get install -y libusb-1.0-0-dev" >&2
    exit 1
fi

mkdir -p "$(dirname "$OUT")"
# shellcheck disable=SC2086
gcc -O2 -Wall -Wextra -Wpedantic -o "$OUT" "$SRC" -lusb-1.0
echo "已生成: $OUT"
echo "运行: sudo $OUT probe    # 先确认芯片，再 tx 秒数 发送"
