#!/bin/bash
# STM32H723 固件刷写脚本
# Always rebuild; bench options are passed explicitly to a separate directory.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="$PROJECT_DIR/build/Debug"
USE_OPENOCD=false
BUILD_ONLY=false
BENCH=false
OVERSPEED_STOP=ON
SERIAL=""
POSITION_MIN=""
POSITION_MAX=""
VELOCITY_MAX=""

while (($#)); do
    case "$1" in
        --openocd) USE_OPENOCD=true ;;
        --build) : ;; # Historical alias: every invocation already rebuilds.
        --build-only) BUILD_ONLY=true ;;
        --bench-joint7) BENCH=true ;;
        --disable-overspeed-stop) OVERSPEED_STOP=OFF ;;
        --serial|--position-min|--position-max|--velocity-max)
            if (($# < 2)); then echo "Missing value for $1" >&2; exit 2; fi
            case "$1" in
                --serial) SERIAL="$2" ;;
                --position-min) POSITION_MIN="$2" ;;
                --position-max) POSITION_MAX="$2" ;;
                --velocity-max) VELOCITY_MAX="$2" ;;
            esac
            shift ;;
        --help|-h)
            echo "Usage: $0 [--openocd] [--serial SERIAL] [--build-only]"
            echo "Bench: --bench-joint7 --position-min RAD --position-max RAD --velocity-max RAD_S"
            echo "Supervised bench only: --disable-overspeed-stop (speed logging remains; other guards remain)"
            echo "Bench flash requires --serial; bounds are mandatory finite decimals."
            exit 0
            ;;
        *) echo "Unknown argument: $1" >&2; exit 2 ;;
    esac
    shift
done

if [[ "$OVERSPEED_STOP" == OFF ]] && ! "$BENCH"; then
    echo "--disable-overspeed-stop requires --bench-joint7" >&2; exit 2
fi
CONFIGURE_ARGS=(--preset Debug -DMOTOR_BENCH_JOINT7_POSVEL=OFF -DMOTOR_BENCH_OVERSPEED_STOP=ON)
if "$BENCH"; then
    if [[ -z "$POSITION_MIN" || -z "$POSITION_MAX" || -z "$VELOCITY_MAX" ]]; then
        echo "Bench requires explicit --position-min, --position-max and --velocity-max" >&2; exit 2
    fi
    if ! "$BUILD_ONLY" && [[ -z "$SERIAL" ]]; then
        echo "Bench flashing requires an explicitly verified --serial" >&2; exit 2
    fi
    BUILD_DIR="$PROJECT_DIR/build/BenchJoint7Debug"
    CONFIGURE_ARGS=(--preset Debug -B "$BUILD_DIR" -DMOTOR_BENCH_JOINT7_POSVEL=ON
        "-DMOTOR_BENCH_POSITION_MIN=$POSITION_MIN" "-DMOTOR_BENCH_POSITION_MAX=$POSITION_MAX"
        "-DMOTOR_BENCH_VELOCITY_MAX=$VELOCITY_MAX"
        "-DMOTOR_BENCH_OVERSPEED_STOP=$OVERSPEED_STOP")
elif [[ -n "$POSITION_MIN$POSITION_MAX$VELOCITY_MAX" ]]; then
    echo "Bounds require --bench-joint7" >&2; exit 2
fi
if [[ -n "$SERIAL" && ! "$SERIAL" =~ ^[a-zA-Z0-9]+$ ]]; then
    echo "Invalid probe serial" >&2; exit 2
fi
BIN_FILE="$BUILD_DIR/rebotarm_executor.bin"
ELF_FILE="$BUILD_DIR/rebotarm_executor.elf"
cd "$PROJECT_DIR"

# 清理并重新编译
echo ">>> 清理历史构建产物..."
rm -rf "$BUILD_DIR"
echo ">>> 配置 CMake..."
cmake "${CONFIGURE_ARGS[@]}"
echo ">>> 编译项目..."
cmake --build "$BUILD_DIR"
echo ""

# 检查固件文件，如果 .bin 不存在则从 .elf 生成
if [ ! -f "$BIN_FILE" ]; then
    if [ -f "$ELF_FILE" ]; then
        echo ">>> 从 ELF 生成 BIN..."
        arm-none-eabi-objcopy -O binary "$ELF_FILE" "$BIN_FILE"
    else
        echo "错误: 找不到固件文件 $ELF_FILE"
        echo "请先编译: cmake --build build/Debug"
        exit 1
    fi
fi

if "$BUILD_ONLY"; then
    echo "Build only: $BIN_FILE"
    exit 0
fi

echo ">>> 刷写固件: $BIN_FILE"
echo ">>> 目标地址: 0x08000000"
echo ""

if [ "$USE_OPENOCD" = true ]; then
    # OpenOCD 方式
    echo ">>> 使用 OpenOCD 刷写..."
    FLASH_COMMAND=(openocd -f interface/stlink.cfg -f target/stm32h7x.cfg)
    if [[ -n "$SERIAL" ]]; then FLASH_COMMAND+=(-c "adapter serial $SERIAL"); fi
    "${FLASH_COMMAND[@]}" -c "program $BIN_FILE 0x08000000 verify reset exit"
else
    # st-flash 方式（默认）
    echo ">>> 使用 st-flash 刷写..."
    FLASH_COMMAND=(st-flash)
    if [[ -n "$SERIAL" ]]; then FLASH_COMMAND+=(--serial "$SERIAL"); fi
    "${FLASH_COMMAND[@]}" --reset write "$BIN_FILE" 0x08000000
fi

echo ""
echo ">>> 刷写完成!"
