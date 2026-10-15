# 架构概览

仓库：[`phamstered/rebotarm-executor`](https://github.com/phamstered/rebotarm-executor)。本文整理自 `AGENTS.md`、`CLAUDE.md`、`PLAN.md`，不替代仓库内源文件。

## 固件角色

固件将上位机 ROS 2 命令桥接到 7 路达妙电机的实时 MIT 控制，并回传关节状态与硬件状态。目标节点名 `stm32h723_motor_bridge`。**ROS / USB 不得阻塞控制与安全任务**；失联与异常输入走 HOLD / DISABLE 等保护路径。

关节顺序与 CAN 映射见 [ROS 2 接口](./ros2-interface)。首轮配置使用 `active_joint_mask=0x7F`（`ESC_ID=0x01..0x07`，`MST_ID=0x11..0x17` → joint1..joint6 与夹爪）；完整控制前须逐轴核对实机地址、方向、零点与传动换算。

## 目录分层

| 路径 | 职责 |
|------|------|
| `Core/Src/`、`Core/Inc/` | 应用启动与 FreeRTOS 任务、中断、MSP |
| `User/` | Damiao / FDCAN 等项目模块（`bsp_fdcan`、`damiao_protocol` 等） |
| `USB_DEVICE/` | USB CDC 设备栈 |
| `vofa+/` | 遥测 / Vofa+ 波形相关 |
| `docs/` | 技术说明与本 VitePress 站点源 |
| `scripts/` | `flash.sh`、监控与 micro-ROS 负载脚本 |
| `Drivers/`、`Middlewares/`、`.ioc` 生成文件 | 厂商 / CubeMX 管理，勿随意重排 |

CubeMX 生成文件中的手工修改必须落在 `/* USER CODE BEGIN ... */` 区域内。外设配置优先改 `rebotarm_executor.ioc`，再生成。

## 构建系统

- 根 `CMakeLists.txt`：用户层，可添加自定义源、头文件与宏。
- `cmake/stm32cubemx/CMakeLists.txt`：CubeMX 汇集点，产出 `STM32_Drivers`、`FreeRTOS` 等 OBJECT 库。
- `CMakePresets.json`：`Debug` / `Release`（Ninja + `arm-none-eabi-gcc` 默认工具链）。
- 链接脚本 `STM32H723xG_flash.ld`：1024K FLASH + 560K RAM。

## FreeRTOS 要点

- API：CMSIS-RTOS V2（`osThreadNew`、`osDelay` 等）。
- 堆：heap_4（配置见工程；`CLAUDE.md` 记载总堆约 15360 字节）。
- Tick：1000 Hz；HAL 时间基准用 TIM6（非 SysTick）。
- 调用 FreeRTOS API 的中断优先级数值须 ≥ `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY`（工程中为 5）。

计划中的任务划分（见 `PLAN.md`）：`motor_control_task`、`safety_task`、`micro_ros_task`、`diagnostic_task` 等，以明确优先级与栈预算隔离控制与通信。

## 外设引脚（文档记载）

| 外设 | 引脚 | 功能 |
|------|------|------|
| FDCAN1 | PD0 (RX), PD1 (TX) | CAN 通信 |
| FDCAN2 | PB5 (RX), PB6 (TX) | CAN 通信 |
| USB OTG HS | PA9 (VBUS), PA10 (ID) 等 | USB CDC / micro-ROS 传输 |

FDCAN 与 USB 细节见 [FDCAN 集成](./fdcan-integration)、[Vofa+ USB CDC](./vofa-usb-cdc-integration)。控制侧实际使用的总线模式与周期以 [ROS 2 接口](./ros2-interface) 为准（例如 7 电机统一 250 Hz、经典 CAN 1 Mbit/s）。

## 与上位机的边界

- 权威接口：[`docs/ros2-interface.md`](./ros2-interface)（本站同页）。
- Session / USB 稳定性验收：[micro-ROS Session 稳定性](./microros-session-stability)。
- 桌面调试控制台见姊妹项目 [`rebotarm-debug-desk`](https://github.com/phamstered/rebotarm-debug-desk)（经 Jetson Bridge / Agent，不在本固件仓库内）。
