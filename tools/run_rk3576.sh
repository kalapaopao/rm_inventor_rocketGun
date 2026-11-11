#!/usr/bin/env bash
# Project-local ARM64 launch: no system installation, no usbipd/noreset preload.
#
# 目录可以被整体挪动：构建缓存里的旧源码路径会被检测并自动重建，
# 配置路径则在运行时通过 --config 显式传入，不依赖编译期写死的
# DEFAULT_CAMERA_CONFIG_PATH / DEFAULT_RECORDING_DIR 宏。
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD="$HERE/build"
CONFIG="$HERE/Camera/camera_config.yaml"
MVS_ROOT="${MVS_ROOT:-$HERE/sdk/MVS}"
LIB="$MVS_ROOT/lib/aarch64"

if [ "$(uname -m)" != aarch64 ]; then
    echo "This launcher targets aarch64 Linux." >&2
    exit 1
fi
if [ ! -f "$LIB/libMvCameraControl.so" ]; then
    echo "Missing ARM64 MVS SDK: $LIB (set MVS_ROOT)." >&2
    exit 1
fi
if [ ! -f "$CONFIG" ]; then
    echo "Missing camera config: $CONFIG" >&2
    exit 1
fi

export LD_LIBRARY_PATH="$LIB${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export MVCAM_COMMON_RUNENV="$MVS_ROOT/lib"

REBUILD=0
if [ "${1:-}" = --rebuild ]; then REBUILD=1; shift; fi

NEED_REBUILD=0
if [ "$REBUILD" -eq 1 ]; then
    NEED_REBUILD=1
elif [ ! -x "$BUILD/Camera/show" ]; then
    NEED_REBUILD=1
elif [ -f "$BUILD/CMakeCache.txt" ]; then
    # 项目挪动后 CMAKE_HOME_DIRECTORY / MVS_ROOT 仍是旧绝对路径，且二进制内嵌的
    # 默认配置、录像目录宏同样失效。只清缓存不重编会让这些宏继续指向不存在的目录。
    CACHED_SRC="$(sed -n 's/^CMAKE_HOME_DIRECTORY:INTERNAL=//p' "$BUILD/CMakeCache.txt" | head -1)"
    if [ -n "$CACHED_SRC" ] && [ "$CACHED_SRC" != "$HERE" ]; then
        echo "[run] project moved: $CACHED_SRC -> $HERE" >&2
        echo "[run] stale build cache detected; rebuilding to refresh embedded paths." >&2
        # 只清空 build 目录内容，不删除目录本身：项目根目录属 root:root，
        # 普通用户无权删除父目录下的目录项，但可以在自己的 build 目录内增删。
        if [ "$BUILD" = "$HERE/build" ] && [ -d "$BUILD" ]; then
            find "$BUILD" -mindepth 1 -maxdepth 1 -exec rm -rf -- {} +
        else
            echo "[run] unexpected build dir, refusing to clean: $BUILD" >&2
            exit 1
        fi
        NEED_REBUILD=1
    fi
fi

if [ "$NEED_REBUILD" -eq 1 ]; then
    cmake -S "$HERE" -B "$BUILD" -DCMAKE_BUILD_TYPE=Release -DMVS_ROOT="$MVS_ROOT"
    cmake --build "$BUILD" -j2
fi

# DEFAULT_RECORDING_DIR 编译期宏指向 <Camera>/../recordings，提前建好目录，
# 避免 --recording 时写盘失败。项目根目录可能属 root（部署包解压所致），
# 此时只告警不中止：默认不带 --recording 的启动不需要该目录。
if ! mkdir -p "$HERE/recordings" 2>/dev/null; then
    echo "[run] note: cannot create $HERE/recordings; --recording needs write access there." >&2
fi

# 显式传 --config：绕过编译期写死的默认配置路径；配置里 calibration.file 的
# "../camera_intrinsics.yaml" 会由程序相对配置文件所在目录解析，而不是 cwd。
# 用户自带 --config / 位置参数时不再追加，保持自定义配置优先。
HAS_CONFIG=0
for arg in "$@"; do
    case "$arg" in
        --config|--config=*) HAS_CONFIG=1 ;;
    esac
done

cd "$HERE"
if [ "$HAS_CONFIG" -eq 0 ]; then
    exec "$BUILD/Camera/show" --config "$CONFIG" "$@"
else
    exec "$BUILD/Camera/show" "$@"
fi
