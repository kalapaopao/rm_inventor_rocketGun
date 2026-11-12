#!/bin/sh
# 串口就绪检查（板端运行）：./check_serial_port.sh [/dev/ttyS10]
# 只做只读检查；带 -l 时做一次自回环收发测试（需先短接 TX-RX）。
PORT="${1:-/dev/ttyS10}"
LOOP=0
[ "$2" = "-l" ] && LOOP=1

echo "=== 检查 $PORT ==="
if [ ! -e "$PORT" ]; then
  echo "[FAIL] 节点不存在：$PORT"
  echo "       现有串口：$(ls /dev/ttyS* 2>/dev/null | tr '\n' ' ')"
  echo "       若 UART10 未启用，需在设备树把 serial@2afc0000 的 status 改为 okay 并重编/替换 dtb。"
  exit 1
fi

ls -l "$PORT"
if [ -r "$PORT" ] && [ -w "$PORT" ]; then
  echo "[ OK ] 当前用户可读写（在 dialout 组即可）"
else
  echo "[FAIL] 当前用户无读写权限，检查是否在 dialout 组"
fi

if systemctl list-units --all --no-pager 2>/dev/null | grep -q "getty@$(basename "$PORT")"; then
  echo "[WARN] 该口有 getty 占用，二进制帧会被终端交互污染，先停用"
else
  echo "[ OK ] 无 getty 占用"
fi

echo "--- 当前波特率 ---"
stty -F "$PORT" -a 2>/dev/null | head -1

if [ "$LOOP" = "1" ]; then
  echo "--- 自回环测试（请确认 TX 已短接 RX）---"
  if command -v python3 >/dev/null 2>&1; then
    python3 - "$PORT" <<'PY'
import sys, serial
port = sys.argv[1]
p = serial.Serial(port, 115200, timeout=2)
frame = bytes.fromhex('A5017B0051D8')
p.reset_input_buffer()
p.write(frame)
got = p.read(6)
print('sent:', frame.hex(' ').upper())
print('recv:', got.hex(' ').upper() if got else '(nothing)')
print('[ OK ] 回环通过' if got == frame else '[FAIL] 回环不一致')
sys.exit(0 if got == frame else 2)
PY
  else
    echo "python3/pyserial 不可用，跳过回环测试"
  fi
fi
