#!/usr/bin/env bash
# 飞镖视觉自启动服务控制脚本（与项目路径无关：项目整体挪到别处也能用）。
#
#   bash tools/dart_vision.sh status    查看运行状态 + 最近日志（默认动作）
#   bash tools/dart_vision.sh stop      停止，并取消开机自启（彻底停掉，重启也不会自己起来）
#   bash tools/dart_vision.sh pause     只停本次，保留开机自启（下次开机仍自动运行）
#   bash tools/dart_vision.sh start     启动，并恢复开机自启
#   bash tools/dart_vision.sh restart   重启（改配置/换串口后用它）
#
# 为什么需要它：程序由 systemd 托管（Restart=always），直接 kill 会被 5 秒后自动拉起；
# 必须用 systemctl stop 才是"人为停止"。本脚本封装这些细节，并清理残留的手动进程。
#
# 可选环境变量（一般不用设）：
#   DART_VISION_SERVICE  服务单元名，默认 dart-vision.service
#   DART_VISION_BIN      可执行文件路径，默认 <项目根>/build/Camera/show
#
# 项目根目录由本脚本位置推导（tools/ 的上一级），所以换目录、换机器都无需改脚本。
set -euo pipefail

SELF="$(readlink -f "${BASH_SOURCE[0]}" 2>/dev/null || printf '%s' "${BASH_SOURCE[0]}")"
HERE="$(cd "$(dirname "$SELF")/.." && pwd)"
SERVICE="${DART_VISION_SERVICE:-dart-vision.service}"
BIN="${DART_VISION_BIN:-$HERE/build/Camera/show}"

ACTION="${1:-status}"
KEEP_AUTOSTART=0
if [ "$ACTION" = "pause" ]; then
    ACTION="stop"
    KEEP_AUTOSTART=1
fi

if [ "$(id -u)" -ne 0 ]; then
    echo "[dart] 需要 root 权限，通过 sudo 重新执行…"
    passthrough=(DART_VISION_SERVICE="$SERVICE")
    [ -n "${DART_VISION_BIN:-}" ] && passthrough+=(DART_VISION_BIN="$DART_VISION_BIN")
    exec sudo env "${passthrough[@]}" bash "$SELF" "$@"
fi

# 把路径转成可读形式，仅用于日志展示（不再用于 pgrep 匹配）。
service_installed() {
    systemctl cat "$SERVICE" >/dev/null 2>&1
}

# 服务单元里记录的部署目录；用于提示"单元指向的并不是当前这份代码"。
unit_workdir() {
    systemctl show -p WorkingDirectory --value "$SERVICE" 2>/dev/null || true
}

warn_if_unit_elsewhere() {
    local dir
    dir="$(unit_workdir)"
    if [ -n "$dir" ] && [ "$dir" != "$HERE" ]; then
        echo "[dart] 注意：服务单元的工作目录是 $dir" >&2
        echo "[dart]       当前项目在 $HERE（两者不一致，可能需要重新安装单元文件）" >&2
    fi
}

# 精确匹配"正在运行这个二进制"的进程：读 /proc/<pid>/exe 与 $BIN 比对。
# 不用 pgrep -f，因为那会把命令行里恰好包含该路径的无关进程（编辑器、grep、ssh 命令）也算进去。
stray_pids() {
    local proc pid exe
    for proc in /proc/[0-9]*; do
        [ -r "$proc/exe" ] || continue
        exe="$(readlink -f "$proc/exe" 2>/dev/null || true)"
        exe="${exe% (deleted)}"   # 二进制被重编替换后仍能识别旧进程
        [ "$exe" = "$BIN" ] || continue
        pid="${proc#/proc/}"
        [ "$pid" = "$$" ] && continue
        printf '%s\n' "$pid"
    done
}

show_strays() {
    local pid found=0
    while IFS= read -r pid; do
        [ -z "$pid" ] && continue
        found=1
        printf '%s %s\n' "$pid" "$(tr '\0' ' ' <"/proc/$pid/cmdline" 2>/dev/null || true)"
    done < <(stray_pids)
    [ "$found" -eq 0 ] && echo "（无）"
    return 0
}

kill_strays() {
    local pids
    pids="$(stray_pids | tr '\n' ' ')"
    [ -z "${pids// /}" ] && return 0
    echo "[dart] 清理残留进程: $pids"
    # shellcheck disable=SC2086
    kill -INT $pids 2>/dev/null || true
    sleep 1
    pids="$(stray_pids | tr '\n' ' ')"
    if [ -n "${pids// /}" ]; then
        # shellcheck disable=SC2086
        kill -9 $pids 2>/dev/null || true
    fi
}

do_status() {
    if ! service_installed; then
        echo "[dart] 服务 $SERVICE 未安装；当前项目: $HERE"
        echo "[dart] 手动运行: bash $HERE/tools/run_rk3576.sh"
        return 0
    fi
    warn_if_unit_elsewhere
    systemctl status "$SERVICE" --no-pager -l 2>/dev/null | head -14 || true
    echo
    echo "[dart] 项目目录: $HERE"
    echo "[dart] 开机自启: $(systemctl is-enabled "$SERVICE" 2>/dev/null || echo unknown)"
    echo "[dart] 进程:"
    show_strays
}

do_stop() {
    if service_installed; then
        systemctl stop "$SERVICE"
        if [ "$KEEP_AUTOSTART" -eq 0 ]; then
            systemctl disable "$SERVICE" >/dev/null 2>&1 || true
        fi
        sleep 1
        if systemctl is-active --quiet "$SERVICE"; then
            echo "[dart] 停止失败：服务仍在运行。用 'systemctl status $SERVICE' 排查。" >&2
            return 1
        fi
    else
        echo "[dart] 服务 $SERVICE 未安装，跳过 systemctl。"
    fi
    kill_strays
    echo "[dart] 已停止。"
    if [ "$KEEP_AUTOSTART" -eq 0 ]; then
        echo "[dart] 开机自启已取消（重启板子也不会自动运行）。恢复: bash $SELF start"
    else
        echo "[dart] 保留了开机自启（下次开机仍会自动运行）。彻底停止: bash $SELF stop"
    fi
}

do_start() {
    if ! service_installed; then
        echo "[dart] 服务 $SERVICE 未安装。安装步骤：" >&2
        echo "  sudo cp $HERE/deploy/dart-vision.service /etc/systemd/system/" >&2
        echo "  sudo systemctl daemon-reload && sudo systemctl enable --now $SERVICE" >&2
        return 1
    fi
    warn_if_unit_elsewhere
    systemctl enable "$SERVICE" >/dev/null 2>&1 || true
    systemctl start "$SERVICE"
    sleep 1
    if systemctl is-active --quiet "$SERVICE"; then
        echo "[dart] 已启动。"
        echo "[dart] 日志: sudo journalctl -u $SERVICE -b -f"
    else
        echo "[dart] 启动失败，最近日志：" >&2
        journalctl -u "$SERVICE" -b --no-pager -n 15 >&2 || true
        return 1
    fi
}

do_restart() {
    if ! service_installed; then
        echo "[dart] 服务 $SERVICE 未安装，无法重启。" >&2
        return 1
    fi
    warn_if_unit_elsewhere
    systemctl restart "$SERVICE"
    sleep 1
    if systemctl is-active --quiet "$SERVICE"; then
        echo "[dart] 已重启。"
    else
        echo "[dart] 重启后未运行，最近日志：" >&2
        journalctl -u "$SERVICE" -b --no-pager -n 15 >&2 || true
        return 1
    fi
}

usage() {
    sed -n '2,20p' "$SELF" | sed 's/^# \{0,1\}//'
}

case "$ACTION" in
    status) do_status ;;
    stop) do_stop ;;
    start) do_start ;;
    restart) do_restart ;;
    -h | --help | help) usage ;;
    *)
        echo "[dart] 未知动作: $ACTION" >&2
        usage >&2
        exit 2
        ;;
esac
