# VLA 接入（固件侧使用说明）

本页说明 VLA（Vision-Language-Action）策略驱动机械臂时，**下位机固件看到什么、保证什么、需要上层注意什么**。
VLA 的控制器、消息和启动方式以 ROS 2 栈文档为准：
[rebotarm-ros2-stack · docs/vla-integration.md](https://github.com/phamstered/rebotarm-ros2-stack/blob/main/docs/vla-integration.md)。

::: tip 结论
固件**没有**为 VLA 做任何改动，也不认识 VLA。VLA 输出的关节角经 ros2_control 合成为普通的
`JointCommand`，固件按现有 [ROS 2 接口](./ros2-interface.md) 的规则执行。
:::

## 数据通路

```
VLA 推理（Jetson 或外部主机）
   │  VlaJointChunk  →  /vla/joint_chunk
   ▼
vla_position_controller（position, j1–6） + gravity_compensation_controller（effort, 可选）
   ▼
MicroRosSystem.write()  ── 合成 MIT 指令 ──►  /stm32/joint_commands
   ▼
micro-ROS Agent（USB CDC 0483:5740, /dev/ttyACM*）
   ▼
STM32H723 固件 → FDCAN 1 Mbit/s → 达妙电机（250 Hz MIT）
```

- VLA 只给 **关节角 q**（j1–j6），不含末端位姿/IK，也不控制夹爪（第 7 路保持上层原有逻辑）。
- 重力补偿以 `effort` 前馈叠加在同一帧 `JointCommand` 里，固件只做透传与限幅检查。
- **不要**让 VLA 节点直接发布 `/stm32/joint_commands`：那会绕过 ros2_control、重力补偿和控制器互斥。直写仅用于调试旁路。

## 固件看到的指令

每个控制周期，`MicroRosSystem` 发布一帧 `JointCommand`：

| 字段 | VLA 运行时的典型值 | 固件行为 |
|------|------------------|---------|
| `mode` | `1`（TRACK） | 按 MIT 公式 \( \tau = k_p(q_{ref}-q) + k_d(\dot q_{ref}-\dot q) + \tau_{ff} \) 下发 |
| `position[i]` | 控制器限速后的 VLA 目标角 | 直接作为 \( q_{ref} \) |
| `velocity[i]` | 通常 0 或合成的前馈速度 | 直接作为 \( \dot q_{ref} \) |
| `kp[i]` / `kd[i]` | 硬件接口参数 | 应用层只要求有限且 ≥ 0；协议编码时钳位到 kp 0–500、kd 0–5 |
| `effort[i]` | 重力补偿力矩（未开启时为 0） | 要求 \|effort\| ≤ 10 Nm，否则整帧拒绝（fault 3） |
| `sequence` | 单调递增 | 不比上一帧新则拒绝（fault 4） |

> 应用层已移除 kp/kd 上限（commit `8e96ef5`），实际增益由上层参数决定，调高 kp 前务必先在低增益下验证。

## 超时与安全语义

VLA 推理慢或断流时，有**两层**保护，互不替代：

| 层级 | 条件 | 行为 |
|------|------|------|
| `vla_position_controller` | 超过 `command_timeout_sec`（默认 0.2 s）没有新 chunk，或 chunk 含 NaN | 保持最后一次实际下发的位置（限速后的值，不是最后的目标）；ros2_control 仍持续下发 TRACK 帧 |
| 固件 `COMMAND_TIMEOUT_MS` | **仅 TRACK 模式下**，100 ms 没收到合法 `JointCommand` | 自动切 **HOLD**：锁存当前位置，v=0、τ=0。RETURN（mode 2）下不做这一步，会一直执行最后一帧直到 500 ms 锁存 |
| 固件 `COMMAND_LOSS_LATCH_MS` | 非 DISABLE 状态（含 TRACK / HOLD / RETURN）下 500 ms 没有合法命令 | 锁存 `fault_code = 11`（CMD_LOSS），已使能则 HOLD，否则 DISABLE；**之后拒绝所有新命令，需安全回零并重启固件** |

要点：

1. **VLA 推理延迟不会触发固件超时**，只要 ros2_control 的控制循环和 Agent 链路在跑（控制器会持续发保持帧）。
2. 控制器有限速（默认 `max_delta_per_sec: 1.0`），chunk 需要持续发，单发一次只会走一小段。
3. 触发固件超时的是 **ros2_control / Agent / USB 链路**中断，不是策略本身慢。
4. 固件 HOLD 下 `effort = 0`，**没有重力补偿**；长时间 HOLD 时重关节会靠 kp/kd 硬扛重力，可能有下垂。
5. fault 11 锁存后不能靠 Agent 重连解除，切换 VLA 前请确保链路稳定。

## 上线检查清单

按顺序执行，前一步不通过不要进入下一步：

1. **链路**：`lsusb` 能看到 `0483:5740 STM32 Virtual ComPort`；`micro-ros-agent-stm32h723.service` 为 active。
2. **状态**：`ros2 topic hz /stm32/status` ≈ 250 Hz，`fault_code == 0`，`mode` 为 HOLD 或 DISABLE。
3. **反馈**：`ros2 topic echo /stm32/joint_states --once`，6 个臂关节位置与实物一致。
4. **控制器互斥**：`ros2 control list_controllers` 中 teleop / trajectory 为 inactive，再激活 `vla_position_controller`（见栈文档 `switch_controllers` 示例）。
5. **Mock 先行**：先用 `ros2 run microros_stm32h723_bridge mock_vla_chunk_publisher.py`（默认 ±0.15 rad 正弦）验证，再接真实策略。注意正弦以 **0 rad 绝对角**为中心，激活前先把臂放到零位附近，否则会以最高 1 rad/s 被拉向零位；只想测 HOLD 时激活控制器但不发 chunk。
6. **观察**：运行中监看 `/stm32/status` 的 `last_command_sequence` 持续递增、`fault_code` 保持 0；`/stm32/joint_states` 的 `effort` 为实测力矩，可用来对比重力补偿效果。

## 常见故障对照

| `fault_code` | 含义 | VLA 场景下最可能的原因 |
|------|------|------|
| 2 | 非有限值 | 上层把 NaN/Inf 合成进命令（控制器本应拦截，检查自定义节点） |
| 3 | 命令超限 | 重力补偿 effort > 10 Nm，或 kp/kd 为负 |
| 4 | 序列号过期 | 有两个发布者同时写 `/stm32/joint_commands`（例如调试旁路没关） |
| 6 / 7 | 反馈超时 / 驱动故障 | 硬件问题，与 VLA 无关；固件会 DISABLE |
| 11 | 命令丢失锁存 | ros2_control 或 Agent 停了超过 500 ms；需回零并重启 |

完整故障码表见 [ROS 2 接口](./ros2-interface.md)。

## 相关文档

- [ROS 2 接口](./ros2-interface.md)：`JointCommand` / `HardwareStatus` 权威定义
- [rebotarm-ros2-stack · VLA integration](https://github.com/phamstered/rebotarm-ros2-stack/blob/main/docs/vla-integration.md)：控制器、参数、激活方式
- [rebotarm-debug-desk](https://phamstered.github.io/rebotarm-debug-desk/)：上位机 Status / 监控面板
