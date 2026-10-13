---
layout: home

hero:
  name: reBot Arm Executor
  text: STM32H723 micro-ROS 电机桥接固件
  tagline: FreeRTOS · Damiao MIT · FDCAN · USB CDC · 节点 stm32h723_motor_bridge
  actions:
    - theme: brand
      text: 架构概览
      link: /architecture
    - theme: alt
      text: 构建与烧录
      link: /build
    - theme: alt
      text: ROS 2 接口
      link: /ros2-interface
    - theme: alt
      text: GitHub
      link: https://github.com/phamstered/rebotarm-executor

features:
  - title: 实时电机桥
    details: 固件负责 7 关节 MIT 控制、硬件保护、失联降级与状态反馈；ROS 通信不得阻塞控制与安全任务。
  - title: micro-ROS over USB CDC
    details: 节点 /stm32h723_motor_bridge 经 USB CDC 连接 Agent；JointState / HardwareStatus 以 250 Hz 为目标发布。
  - title: 安全优先
    details: 上电默认 DISABLE；TRACK 超时进入 HOLD；命令丢失锁存后需安全回零并重启恢复。烧录前确认 MCU、探针与地址。
---

## 这是什么

**reBot Arm Executor**（仓库 [`phamstered/rebotarm-executor`](https://github.com/phamstered/rebotarm-executor)）是运行在 **STM32H723** 上的机械臂执行器固件：STM32CubeMX 外设初始化 + FreeRTOS（CMSIS-RTOS V2）任务调度，经 micro-ROS / USB CDC 与上位机 ROS 2 对接，通过 FDCAN 驱动达妙（Damiao）电机。

| | |
|---|---|
| **产品 / 文档名** | reBot Arm Executor |
| **仓库** | `phamstered/rebotarm-executor` |
| **MCU** | STM32H723VGTx（Cortex-M7, FPU） |
| **ROS 节点** | `/stm32h723_motor_bridge` |

仓库根目录的 `AGENTS.md`、`PLAN.md`、`CLAUDE.md` 仍是开发与实施说明源；本站点将其中面向用户的部分整理为 VitePress 文档，**不会移动或删除**这些根文件。

## 快速入口

1. [架构概览](./architecture) — 目录分层、任务与外设边界
2. [构建与烧录](./build) — CMake Presets、`flash.sh`、监控脚本
3. [ROS 2 接口](./ros2-interface) — **权威** topic / 消息 / 模式 / 故障码说明
4. [FDCAN 集成](./fdcan-integration) · [micro-ROS Session](./microros-session-stability) · [Vofa+ USB CDC](./vofa-usb-cdc-integration)

## 安全摘要

- 上电保持电机 `DISABLE`；初始化失败不得使能。
- TRACK 命令超过约 100 ms 未更新进入 HOLD；超过约 500 ms 锁存通信丢失（fault 11），需安全回零并重启后恢复，不能仅靠 Agent 重连解除。
- `./scripts/flash.sh` 写入地址 `0x08000000`；烧录前确认目标 MCU、探针与地址。
- 切勿提交机器相关串口、工具链路径、凭据或 `build/` 产物。

接口变更必须与 [`ros2-interface`](./ros2-interface) 同补丁更新。细节以该文档与仓库内实现为准。

## 本机文档预览

```bash
npm install
npm run docs:dev
```
