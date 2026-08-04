#!/bin/bash
# STM32H723 固件刷写脚本
# 用法: ./scripts/flash.sh [--build] [--openocd]
#   --build    刷写前先编译
#   --openocd  使用 OpenOCD 代替 st-flash

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="$PROJECT_DIR/build/Debug"
BIN_FILE="$BUILD_DIR/rebotarm_executor.bin"
ELF_FILE="$BUILD_DIR/rebotarm_executor.elf"

USE_OPENOCD=false

for arg in "$@"; do
    case $arg in
        --openocd) USE_OPENOCD=true ;;
        --help|-h)
            echo "用法: $0 [--openocd]"
            echo "  --openocd  使用 OpenOCD 代替 st-flash"
            exit 0
            ;;
    esac
done

# 清理并重新编译
echo ">>> 清理历史构建产物..."
rm -rf "$BUILD_DIR"
echo ">>> 配置 CMake..."
cmake --preset Debug
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

echo ">>> 刷写固件: $BIN_FILE"
echo ">>> 目标地址: 0x08000000"
echo ""

if [ "$USE_OPENOCD" = true ]; then
    # OpenOCD 方式
    echo ">>> 使用 OpenOCD 刷写..."
    openocd \
        -f interface/stlink.cfg \
        -f target/stm32h7x.cfg \
        -c "program $BIN_FILE 0x08000000 verify reset exit"
else
    # st-flash 方式（默认）
    echo ">>> 使用 st-flash 刷写..."
    st-flash --reset write "$BIN_FILE" 0x08000000
fi

echo ""
echo ">>> 刷写完成!"
