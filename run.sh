#!/usr/bin/env bash
# 海康相机一键构建 + 运行（WSL / 原生 Linux 通用）
#
#   ./run.sh                直接运行已有 build/Camera/show（不编译）
#   ./run.sh --build        重新配置并编译后运行
#   ./run.sh --recording    按配置 record_seconds 录一段就退出
#   ./run.sh --calib        相机标定（s 保存 / d 删除 / Esc 结束并标定）
#   ./run.sh -h             查看程序自己的参数
#
# 脚本负责三件环境上的事，程序本身不用管：
#   1. WSL 下自动 usbipd attach，省得手动敲；
#   2. 注入 noreset.so，屏蔽 SDK 打开设备时的 libusb_reset_device
#      （这一下会把 usbipd 的虚拟链路打死，之后只能物理拔插）；
#   3. 把 /opt/MVS/lib/64 加进 LD_LIBRARY_PATH —— SDK 的 Bayer 转换走
#      libMediaProcess.so，它依赖 MVS 自带的 LIBSWSCALE_6，
#      系统里的 libswscale 是 5，不指过去就会静默降级成 "转换失败"。
set -uo pipefail

BUILD_REQUESTED=0
ARGS=()
for arg in "$@"; do
  case "$arg" in
    --build|--rebuild) BUILD_REQUESTED=1 ;;
    *) ARGS+=("$arg") ;;
  esac
done

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD="$HERE/build"
IS_WSL=0
grep -qi microsoft /proc/version 2>/dev/null && IS_WSL=1

have_camera() { lsusb 2>/dev/null | grep -q '2bdf:'; }

# 已经有一个实例在跑时，第二个只会拿到“设备被占用”，直接拦住。
RUNNING="$(pgrep -x 'show|hik_calibration' 2>/dev/null | tr '\n' ' ')"
if [ -n "${RUNNING// /}" ] && [ -z "${HIKCAM_ALLOW_MULTI:-}" ]; then
  case " $* " in
    *" -h "*|*" --help "*) ;;
    *)
      echo "[run] 已经有 show / hik_calibration 在运行（PID: $RUNNING），相机会被独占。" >&2
      echo "[run] 先停掉：pkill -x 'show|hik_calibration'" >&2
      exit 1
      ;;
  esac
fi

# ---------- WSL：把相机挂进来 ----------
USBIPD="$(command -v usbipd.exe 2>/dev/null || true)"
if [ "$IS_WSL" -eq 1 ] && ! have_camera && [ -n "$USBIPD" ]; then
  BUSID="$(timeout 15 "$USBIPD" list 2>/dev/null | tr -d '\r' | awk '/2bdf:0001/ {print $1; exit}')"
  if [ -z "$BUSID" ]; then
    echo "[run] Windows 侧看不到海康相机：检查线缆 / 换个 USB3 口。" >&2
  else
    echo "[run] 正在把相机挂载到 WSL（busid $BUSID）..."
    OUT="$(timeout 20 "$USBIPD" attach --wsl --busid "$BUSID" 2>&1 | tr -d '\r')"
    case "$OUT" in
      *busy*|*占用*)
        echo "[run] 相机被 Windows 程序占用，尝试关掉 MVS 客户端后重试 ..."
        timeout 10 taskkill.exe /IM MVS.exe /F >/dev/null 2>&1 || true
        sleep 2
        timeout 20 "$USBIPD" attach --wsl --busid "$BUSID" >/dev/null 2>&1
        ;;
    esac
    for _ in 1 2 3 4 5; do have_camera && break; sleep 1; done
    if have_camera; then
      echo "[run] 相机已进入 WSL：$(lsusb | grep 2bdf)"
    else
      echo "[run] 挂载失败：关掉 MVS → 换个 USB 口 → 物理拔插相机后重跑。" >&2
    fi
  fi
fi

# ---------- 构建 ----------
if [ "$BUILD_REQUESTED" -eq 1 ] || [ ! -x "$BUILD/Camera/show" ]; then
  cmake -S "$HERE" -B "$BUILD" -DCMAKE_BUILD_TYPE=Release >/dev/null || exit 1
  cmake --build "$BUILD" -j"$(nproc)" || exit 1
fi

BIN="$BUILD/Camera/show"
[ -x "$BIN" ] || { echo "[run] 没找到 $BIN" >&2; exit 1; }
CALIB_BIN="$BUILD/hik_calibration/hik_calibration"

# ---------- 运行环境 ----------
ENV_PREFIX=(env)
if [ "$IS_WSL" -eq 1 ]; then
  if [ ! -f "$BUILD/noreset.so" ]; then
    printf 'struct libusb_device_handle;\nint libusb_reset_device(struct libusb_device_handle *d){ (void)d; return 0; }\n' \
      > "$BUILD/noreset.c"
    gcc -shared -fPIC -o "$BUILD/noreset.so" "$BUILD/noreset.c" || true
  fi
  [ -f "$BUILD/noreset.so" ] && ENV_PREFIX+=(LD_PRELOAD="$BUILD/noreset.so")
fi
ENV_PREFIX+=(LD_LIBRARY_PATH="/opt/MVS/lib/64${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}")

# 标定模式：同一个运行环境（usbipd / noreset / MVS 库），跑标定程序而不是预览程序。
if [ "${1:-}" = "--calib" ]; then
  [ -x "$CALIB_BIN" ] || { echo "[run] 没找到 $CALIB_BIN" >&2; exit 1; }
  exec "${ENV_PREFIX[@]}" "$CALIB_BIN"
fi

exec "${ENV_PREFIX[@]}" "$BIN" "${ARGS[@]}"
