# 构建与烧录

命令摘自仓库 [`AGENTS.md`](https://github.com/phamstered/rebotarm-executor/blob/main/AGENTS.md) / [`CLAUDE.md`](https://github.com/phamstered/rebotarm-executor/blob/main/CLAUDE.md)。需本机已安装 `arm-none-eabi-gcc`、CMake、Ninja，以及 ST-Link 或 OpenOCD（烧录时）。

## 配置与编译

```bash
# Debug（默认开发构建）
cmake --preset Debug
cmake --build build/Debug

# Release（优化 / 时序 / 链接相关变更时建议再编一次）
cmake --preset Release
cmake --build build/Release

# 清理 Debug 产物
cmake --build build/Debug --target clean
```

产物为 `rebotarm_executor.elf`（在对应 `build/` 目录下）。工具链文件在 `cmake/`：

- `gcc-arm-none-eabi.cmake` — 默认，要求 `arm-none-eabi-gcc` 在 `PATH` 中
- `starm-clang.cmake` — ST Clang（需在 `CMakePresets.json` 中切换 `toolchainFile`）

## 烧录

```bash
./scripts/flash.sh            # 重建 Debug 并通过 ST-Link 烧录
./scripts/flash.sh --openocd  # 改用 OpenOCD
```

`flash.sh` 写入地址 **`0x08000000`**。烧录前确认目标 MCU、探针与地址；错误目标可能导致变砖或写错芯片。

## 遥测与会话检查

```bash
# USB CDC 电机遥测（需 Python pyserial）；参数为秒数
./scripts/monitor_motor.sh 30
```

无运动的 100 Hz XRCE 负载与 Agent 重连检查见 [micro-ROS Session 稳定性](./microros-session-stability)。接口与故障码见 [ROS 2 接口](./ros2-interface)。

## 测试约定

仓库当前无自动化单元测试目标。每次变更至少保证 **Debug 能编过**；涉及优化、时序或链接行为时再编 Release。硬件相关改动请在 PR 中写明板级配置，并附 FDCAN / 电机 / USB CDC 的观察结果或串口摘录。
