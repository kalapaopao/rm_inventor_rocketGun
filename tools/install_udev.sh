#!/usr/bin/env bash
# 一次装好海康相机 udev 规则，让普通用户（不需要 sudo）也能直接跑 ./run.sh
#
#   sudo bash tools/install_udev.sh
#
# 背景：相机设备节点默认是 crw-rw-r-- root root，普通用户只有读权限，
# MVS SDK 枚举不到设备，程序会报"未发现海康相机"。装上规则后节点变成
# crw-rw-rw- root plugdev，plugdev 组成员（含默认用户）即可读写。
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RULE_SRC="$HERE/99-hikrobot.rules"
RULE_DST="/etc/udev/rules.d/99-hikrobot.rules"

if [ "$(id -u)" -ne 0 ]; then
  echo "需要 root 权限，请执行： sudo bash $0"
  exit 1
fi

if [ ! -f "$RULE_SRC" ]; then
  echo "找不到规则文件：$RULE_SRC"
  exit 1
fi

install -m 644 "$RULE_SRC" "$RULE_DST"
echo "[udev] 已安装 $RULE_DST"

if ! pgrep -x systemd-udevd >/dev/null 2>&1; then
  echo "[udev] 警告：systemd-udevd 没在运行，规则不会自动生效。"
  echo "       确认 /etc/wsl.conf 里有 [boot] systemd=true 后执行 wsl --shutdown 重启。"
fi

udevadm control --reload-rules 2>/dev/null || true
udevadm trigger --subsystem-match=usb --action=change 2>/dev/null || true
sleep 1

# 顺带把当前用户加进 plugdev（多数发行版默认已在，保险起见）
TARGET_USER="${SUDO_USER:-}"
if [ -n "$TARGET_USER" ] && [ "$TARGET_USER" != "root" ]; then
  if id -nG "$TARGET_USER" 2>/dev/null | grep -qw plugdev; then
    echo "[udev] 用户 $TARGET_USER 已在 plugdev 组"
  else
    usermod -aG plugdev "$TARGET_USER"
    echo "[udev] 已把 $TARGET_USER 加入 plugdev 组（需重新登录 shell 生效）"
  fi
fi

echo
echo "当前相机设备节点："
for node in /dev/bus/usb/*/*; do
  [ -e "$node" ] || continue
  ls -l "$node"
done 2>/dev/null | grep -E 'crw' || echo "  （暂无 /dev/bus/usb 节点）"
echo
echo "完成。验证方式："
echo "  lsusb | grep 2bdf        # 能看到相机"
echo "  ./run.sh --no-window     # 普通用户直接跑，应该不再报「未发现海康相机」"
