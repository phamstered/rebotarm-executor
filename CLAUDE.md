# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

基于 STM32H723VGTx（Cortex-M7, FPU）的机械臂执行器固件，使用 STM32CubeMX 生成外设初始化代码 + FreeRTOS (CMSIS-RTOS V2) 进行任务调度。项目名 `rebotarm_executor`。

## 构建命令

```bash
# 配置（使用 CMake Presets，Ninja 生成器）
cmake --preset Debug          # 或 Release

# 编译
cmake --build build/Debug     # 或 build/Release

# 清理
cmake --build build/Debug --target clean
```

工具链文件在 `cmake/` 下：
- `gcc-arm-none-eabi.cmake` — 默认工具链，需要 `arm-none-eabi-gcc` 在 PATH 中
- `starm-clang.cmake` — ST Clang 工具链（默认 PICOLIBC 配置），需要在 `CMakePresets.json` 中切换 `toolchainFile`

## 架构

### 构建系统分层

- `CMakeLists.txt`（根）— 用户层，可在此添加自定义源文件、头文件、宏定义
- `cmake/stm32cubemx/CMakeLists.txt` — CubeMX 生成的源文件、HAL 驱动、FreeRTOS 源文件的汇集点，编译为 `STM32_Drivers` 和 `FreeRTOS` 两个 OBJECT 库
- `CMakePresets.json` — 构建配置，Debug/Release 两种预设

### 代码组织

```
Core/Src/         — 应用代码（main.c, freertos.c, 中断处理, MSP 初始化）
Core/Inc/         — 头文件（main.h, FreeRTOSConfig.h）
Drivers/          — STM32 HAL/LL 驱动 + CMSIS（CubeMX 管理，勿手动修改）
Middlewares/      — FreeRTOS 内核源码（CubeMX 管理）
```

### CubeMX USER CODE 约定

所有 CubeMX 生成的 `.c`/`.h` 文件中，用户代码**必须**写在 `/* USER CODE BEGIN X */` 和 `/* USER CODE END X */` 标记之间。这些标记之外的代码在重新生成时会被覆盖。

修改外设配置应通过 `rebotarm_executor.ioc` 文件在 STM32CubeMX 中操作，而非直接改 `Core/Src/main.c` 中的初始化函数。

### 外设引脚映射

| 外设 | 引脚 | 功能 |
|------|------|------|
| FDCAN1 | PD0 (RX), PD1 (TX) | CAN 通信 |
| FDCAN2 | PB5 (RX), PB6 (TX) | CAN 通信 |
| USB OTG HS | PA9 (VBUS), PA10 (ID) | USB |

### FreeRTOS 配置要点

- API 风格：CMSIS-RTOS V2（`osThreadNew`, `osDelay` 等），在 `freertos.c` 中添加任务
- 堆管理：heap_4，总堆 15360 字节
- Tick 频率：1000 Hz
- HAL 时间基准：TIM6（非 SysTick），通过 `stm32h7xx_hal_timebase_tim.c` 实现
- FPU 配置：FreeRTOS 端口使用 ARM_CM4F（兼容 CM7）

### 链接脚本

`STM32H723xG_flash.ld` — 1024K FLASH + 560K RAM。修改堆栈大小在此文件中调整。

## 开发注意事项

- 添加新的 FreeRTOS 任务时，在 `main.c` 的 `/* USER CODE BEGIN RTOS_THREADS */` 区域创建线程，任务函数体放在 `freertos.c`
- 添加新外设：在 CubeMX (.ioc) 中配置，然后在 MSP 文件中补充 GPIO/时钟初始化
- FreeRTOS 中断优先级规则：调用 FreeRTOS API 的中断优先级数值必须 ≥ 5（`configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY`）
- 编译输出为 `.elf` 文件，需通过 `objcopy` 转换为 `.bin`/`.hex` 用于烧录
