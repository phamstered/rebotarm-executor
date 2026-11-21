# reBot Arm Executor

基于 **STM32H723** 的机械臂执行器固件：STM32CubeMX 外设初始化 + **FreeRTOS**（CMSIS-RTOS V2）任务调度，经 **micro-ROS / USB CDC** 与上位机 ROS 2 对接，通过 **FDCAN** 驱动达妙（Damiao）MIT 电机。目标节点 `/stm32h723_motor_bridge`，覆盖 7 关节（六轴 + 夹爪）。

| | |
|---|---|
| **产品 / 文档名** | reBot Arm Executor |
| **仓库** | [`phamstered/rebotarm-executor`](https://github.com/phamstered/rebotarm-executor) |
| **MCU** | STM32H723VGTx（Cortex-M7, FPU） |
| **栈** | FreeRTOS · micro-ROS · CMake / Ninja · `arm-none-eabi-gcc` |
| **ROS 节点** | `/stm32h723_motor_bridge` |
| **用户文档** | [https://phamstered.github.io/rebotarm-executor/](https://phamstered.github.io/rebotarm-executor/)（GitHub Pages） |

## 相关仓库 / 职责

| 仓库 | 职责 | 文档 |
|---|---|---|
| [`phamstered/rebotarm-debug-desk`](https://github.com/phamstered/rebotarm-debug-desk) | 上位机软件（Tauri / React 调试台） | [Pages](https://phamstered.github.io/rebotarm-debug-desk/) |
| [`phamstered/rebotarm-ros2-stack`](https://github.com/phamstered/rebotarm-ros2-stack) | ROS 2 控制栈（controller / MoveIt / 仿真） | [Pages](https://phamstered.github.io/rebotarm-ros2-stack/) |
| [`phamstered/rebotarm-executor`](https://github.com/phamstered/rebotarm-executor) | FreeRTOS + micro-ROS 固件（本仓） | [Pages](https://phamstered.github.io/rebotarm-executor/) |

## 架构一览

| 层 | 组件 |
|---|---|
| **上位机** | ROS 2 · micro-ROS Agent（USB CDC）· 可选 Debug Desk / Jetson Bridge |
| **固件** | `stm32h723_motor_bridge` · FreeRTOS 任务（控制 / 安全 / micro-ROS / 诊断） |
| **总线** | FDCAN → 达妙电机（MIT · 经典 CAN 1 Mbit/s · 目标 250 Hz） |
| **外设** | USB CDC · FDCAN1/2 · CubeMX 生成 HAL |

固件负责实时 MIT 控制、硬件保护、失联降级与状态反馈；**ROS / USB 不得阻塞控制与安全任务**。目录分层、任务边界与引脚见用户文档 **[架构](https://phamstered.github.io/rebotarm-executor/architecture)**。源文件：[docs/architecture.md](docs/architecture.md)。

权威接口（topic / 消息 / 模式 / 故障码）：[docs/ros2-interface.md](docs/ros2-interface.md) · [Pages](https://phamstered.github.io/rebotarm-executor/ros2-interface)。

## 构建 / 烧录 / 监控

需本机已安装 `arm-none-eabi-gcc`、CMake、Ninja；烧录另需 ST-Link 或 OpenOCD。
首次构建、`interfaces/` 消息或实体池需求更新后，先准备子模块并生成对应的 micro-ROS 静态库。
准备脚本保留其他构建设置，并确保至少 11 个 publisher（生产 6 个、台架另加 5 个）；旧库不能仅修改头文件代替重建：

```bash
git submodule update --init --recursive
python3 scripts/prepare_microros_interfaces.py
printf 'y\n' | docker run -i --rm -v "$PWD":/project \
  --env MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library \
  microros/micro_ros_static_library_builder:humble
```

V2 控制、异步切换和真实停机证明见[接口文档](docs/ros2-interface.md#pos-vel-v2-control-and-runtime-configuration)。
POS_VEL 支持掩码默认 0，须完成逐型号/固件台架验证后才可开放。当前本地 Debug 链接占用约 93% DTCMRAM；运行时栈/堆余量仍需按接口文档完成台架测量。
首次 joint7 验证可使用默认关闭的[受限台架构建](docs/ros2-interface.md#joint7-bench-image)，它不改变生产资格表，必须显式指定位置窗口和速度上限。
台架镜像可经 ROS 导出只读启动参数和有界 joint7 CAN 记录，详见[诊断接口](docs/ros2-interface.md#bench-diagnostics-over-ros)。诊断不改写参数、不自动使能，发送入队记录不等于线侧完成或运动验收通过。
台架还提供默认不执行的[临时加速度事务](docs/ros2-interface.md#bench-volatile-acceleration-profile)：仅允许 joint7 固定 ACC 参数及原值恢复，独立读回成功后才开放 POS_VEL 租约；不保存驱动 Flash。MCU 重启不等于电机参数恢复。

```bash
# 配置与编译（Debug）
cmake --preset Debug
cmake --build build/Debug

# Release（优化 / 时序 / 链接相关变更时建议再编）
cmake --preset Release
cmake --build build/Release

# 清理
cmake --build build/Debug --target clean

# 烧录（重建 Debug；默认 ST-Link）
./scripts/flash.sh
./scripts/flash.sh --openocd

# USB CDC 电机遥测（需 Python pyserial；参数为秒数）
./scripts/monitor_motor.sh 30
```

产物为 `rebotarm_executor.elf`。更完整的工具链与约定见 [docs/build.md](docs/build.md) · [Pages](https://phamstered.github.io/rebotarm-executor/build)，以及仓库根目录 [`AGENTS.md`](AGENTS.md)。

本机预览用户文档站点：

```bash
npm install
npm run docs:dev
```

## 文档索引

| 文档 | 说明 |
|---|---|
| [架构概览](docs/architecture.md) · [Pages](https://phamstered.github.io/rebotarm-executor/architecture) | 目录分层、任务与外设边界 |
| [构建与烧录](docs/build.md) · [Pages](https://phamstered.github.io/rebotarm-executor/build) | CMake Presets、`flash.sh`、监控 |
| [ROS 2 接口](docs/ros2-interface.md) · [Pages](https://phamstered.github.io/rebotarm-executor/ros2-interface) | **权威** topic / 消息 / 模式 / 故障码 |
| [FDCAN 集成](docs/fdcan-integration.md) · [Pages](https://phamstered.github.io/rebotarm-executor/fdcan-integration) | Damiao / FDCAN 集成说明 |
| [micro-ROS Session 稳定性](docs/microros-session-stability.md) · [Pages](https://phamstered.github.io/rebotarm-executor/microros-session-stability) | 非运动 100 Hz XRCE 负载与重连 |
| [Vofa+ USB CDC](docs/vofa-usb-cdc-integration.md) · [Pages](https://phamstered.github.io/rebotarm-executor/vofa-usb-cdc-integration) | 遥测 / USB CDC |
| [实施计划](PLAN.md) | 分阶段 checklist（开发用） |
| [贡献 / Agent 规范](AGENTS.md) | 构建、风格、安全与 PR 约定 |

用户文档站点首页：[https://phamstered.github.io/rebotarm-executor/](https://phamstered.github.io/rebotarm-executor/)。

## CI

- **STM32 Debug firmware build**（[`.github/workflows/ci.yml`](.github/workflows/ci.yml)）：`push` / `pull_request` → `main`；递归 submodule、时钟映射及 V2 核心/CAN 模拟主机测试、按需生成 `libmicroros.a`、`cmake --preset Debug` 并确认 `rebotarm_executor.elf`。不覆盖烧录、实机联调、Agent 会话或 Release。
- **Deploy docs to GitHub Pages**（[`.github/workflows/deploy-docs.yml`](.github/workflows/deploy-docs.yml)）：`main` 推送或 `workflow_dispatch` 构建 VitePress 并部署；仓库 Settings → Pages → Source 需为 **GitHub Actions**。

## 安全

- 上电保持电机 **`DISABLE`**；任何初始化失败均不得使能。
- TRACK 命令约 **100 ms** 未更新进入 HOLD；约 **500 ms** 锁存通信丢失，需安全回零并重启后恢复（不能仅靠 Agent 重连解除）。
- `./scripts/flash.sh` 写入地址 **`0x08000000`**；烧录前确认目标 MCU、探针与地址。
- 切勿提交机器相关串口、工具链路径、凭据或 `build/` 产物。
- 接口（topic / 字段 / 模式 / 故障码等）变更须与 [`docs/ros2-interface.md`](docs/ros2-interface.md) **同补丁**更新。

细节以接口文档与仓库内实现为准；本 README 不替代 `docs/` 与 `PLAN.md`。

### Explicit seven-axis POS_VEL configuration

Use `./scripts/flash.sh --posvel-seven --build-only` to prepare the opt-in
seven-axis URDF profile (0.1 rad/s command caps). It does not automatically
switch modes or enable outputs. Default builds remain closed. See
[the interface contract](docs/ros2-interface.md#explicit-seven-axis-urdf-pos_vel-deployment-profile)
for exact units, gripper calibration, limits and physical validation boundaries.
