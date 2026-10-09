# STM32H723 micro-ROS 接口文档

本文档是固件 ROS 2 接口的规范性说明，描述当前实现而非规划功能。节点名为
`/stm32h723_motor_bridge`，通过 USB CDC 连接 micro-ROS Agent。全部实体使用
best-effort QoS。关节顺序固定如下：

| 索引/掩码 | 名称 | ESC ID | 反馈 ID |
|---|---|---|---|
| 0 / `0x01` | `joint1` | `0x01` | `0x11` |
| 1 / `0x02` | `joint2` | `0x02` | `0x12` |
| 2 / `0x04` | `joint3` | `0x03` | `0x13` |
| 3 / `0x08` | `joint4` | `0x04` | `0x14` |
| 4 / `0x10` | `joint5` | `0x05` | `0x15` |
| 5 / `0x20` | `joint6` | `0x06` | `0x16` |
| 6 / `0x40` | `gripper_finger_left_joint` | `0x07` | `0x17` |

## Topic 总览

方向均以 STM32 节点为参照。

| Topic | 方向 | 消息类型 | 频率/触发 | 功能 |
|---|---|---|---|---|
| `/stm32/joint_commands` | 订阅 | `microros_stm32h723_bridge/msg/JointCommand` | 上位机周期发送 | 7 关节 MIT 控制命令 |
| `/stm32/joint_states` | 发布 | `sensor_msgs/msg/JointState` | 250 Hz（4 ms） | 位置、速度、估算力矩反馈 |
| `/stm32/status` | 发布 | `microros_stm32h723_bridge/msg/HardwareStatus` | 250 Hz（4 ms） | 控制状态与故障码 |
| `/stm32/zero_position/get` | 订阅 | `std_msgs/msg/UInt32` | 单次请求 | 按掩码读取输出轴零点偏置 |
| `/stm32/zero_position/set` | 订阅 | `std_msgs/msg/UInt32` | 单次请求 | 按掩码将当前位置设为零点 |
| `/stm32/zero_position/state` | 发布 | `sensor_msgs/msg/JointState` | 结果变化及每 500 ms | 返回 RID54 零点偏置 |
| `/stm32/zero_position/status` | 发布 | `std_msgs/msg/UInt32MultiArray` | 结果变化及每 500 ms | 零点操作状态和逐轴结果 |
| `/stm32/microros_diagnostics` | 发布 | `std_msgs/msg/UInt32MultiArray` | 1 Hz | XRCE session、ping、executor 和 USB 诊断计数 |

## JointCommand

消息定义位于
`micro_ros_stm32cubemx_utils/microros_static_library/library_generation/extra_packages/microros_stm32h723_bridge/msg/JointCommand.msg`。

```text
std_msgs/Header header
uint32 sequence
uint8 mode
float32[7] position
float32[7] velocity
float32[7] kp
float32[7] kd
float32[7] effort
```

- `header` 当前不参与固件校验；安全超时使用 STM32 单调时钟。
- `sequence` 必须按 uint32 回绕规则严格递增，重复或旧序号被拒绝。
- `position` 单位 rad，`velocity` 单位 rad/s，`effort` 单位 Nm；数组按本文关节顺序排列。
- 所有数值必须有限；`0 <= kp <= 40`、`0 <= kd <= 5`、`|effort| <= 6 Nm`。
  这里的 effort 限制是独立的应用层安全策略，不是 MIT 报文的归一化量程。
- JointCommand 层不限制 position/velocity；超出对应电机 MIT 编码范围的值会在
  序列化时钳制。命令编码和反馈解码均使用以下型号量程：

| 关节 | 电机型号 | position | velocity | effort/torque |
|---|---|---:|---:|---:|
| joint1–joint3 | DM4340P | ±12.5 rad | ±10 rad/s | ±28 Nm |
| joint4–joint6、夹爪 | DM4310 | ±12.5 rad | ±30 rad/s | ±10 Nm |

MIT 报文只传输归一化后的 16/12-bit 整数，不携带上述量程。固件量程必须与电机
型号一致；例如把 DM4340P 错按 ±5 Nm 编码，会让电机把 1 Nm 命令解释为约
5.6 Nm。当前 `|effort| <= 6 Nm` 的安全限制低于两种电机的协议力矩范围，因此
正常命令不会因协议力矩量程而被钳制。

控制模式：

| 值 | 模式 | 当前行为 |
|---|---|---|
| 0 | `HOLD` | 首次进入时锁存实测位置；发送速度 0、力矩 0，并沿用命令的 kp/kd。 |
| 1 | `TRACK` | 直接执行 position、velocity、kp、kd、effort。 |
| 2 | `RETURN` | 当前与 TRACK 使用相同控制路径；尚未实现自动回零或限速轨迹。 |
| 3 | `DISABLE` | 向所有电机发送 DISABLE；其余数组字段不产生驱动输出。 |

TRACK 命令超过 100 ms 未更新时进入 HOLD；超过 500 ms 锁存通信丢失故障并
拒绝后续命令。锁存后需要安全回零并重启才能恢复，不能通过 Agent 重连解除。
零点维护执行期间普通 JointCommand 也会被拒绝。

## CAN 控制周期与总线负载

电机控制任务周期为 4 ms（250 Hz）。在 TRACK、RETURN 或 HOLD 且 7 个电机均已
使能时，每周期依次发送 7 帧 MIT 命令，即 1750 条控制帧/s；接收队列也以 250 Hz
控制循环排空。FDCAN1 使用经典 CAN、11 位标准 ID、8-byte 数据和 1 Mbit/s 仲裁
速率，不使用 CAN FD/BRS。

标准 8-byte 数据帧连同帧间隔约 111 bit，考虑最坏位填充约 135 bit。因此仅命令
方向负载约为：

```text
7 × 250 × 111 / 1,000,000 = 19.4%
7 × 250 × 135 / 1,000,000 = 23.6%（最坏位填充估算）
```

若每条 MIT 命令触发一条同长度电机反馈，双向合计约 38.9%～47.3%。原 500 Hz
配置在逐命令反馈时约为 77.7%～94.5%，会显著增加仲裁延迟、TX/RX FIFO 积压和
错误重传后的周期抖动，因此 7 电机统一采用 250 Hz。DISABLE 状态不发送周期 MIT
命令，而是每个关节每 50 ms 请求一次反馈；零点维护期间常规控制循环暂停。

## JointState 与 HardwareStatus

`/stm32/joint_states` 的 `name`、`position`、`velocity`、`effort` 均为 7 项，
单位分别为名称、rad、rad/s、Nm；`header.frame_id="base_link"`。时间同步成功后
使用 Agent epoch，否则时间戳保持无效/零值。velocity 和 effort 反馈使用上表中
对应电机型号的 MIT 量程解码。

`HardwareStatus` 字段为：

```text
std_msgs/Header header
uint32 last_command_sequence
uint8 mode
uint32 active_joint_mask
uint16 fault_code
```

当前 `active_joint_mask=0x7F`。故障码如下：

| 值 | 含义 | 值 | 含义 |
|---|---|---|---|
| 0 | 无故障 | 1 | 无效模式 |
| 2 | NaN/Inf 命令 | 3 | kp/kd/effort 超限 |
| 4 | sequence 过期 | 5 | 非活动关节命令（保留） |
| 6 | 反馈超时/无效 | 7 | 电机驱动故障 |
| 8 | CAN 发送失败 | 9 | 初始化失败 |
| 10 | 电机使能超时 | 11 | 命令丢失已锁存 |
| 12 | 零点维护忙 |  |  |

## 零点维护接口

GET/SET 的 `UInt32.data` 是关节位掩码；`0` 无效，`127` 表示全部关节。GET
读取 Damiao RID54 `x_off`（输出轴零点偏置，rad）。SET 发送 `0xFE`，把当前
输出轴位置设为零点；它不会执行独立的参数持久化命令。

SET 仅在全局 DISABLE、所选关节反馈有效、`|velocity| <= 0.10 rad/s` 且
`|effort| <= 0.50 Nm` 时执行。固件会再次发送 DISABLE，等待 20 ms，设置零点，
验证反馈位置在 ±0.05 rad 内，再读取 RID54。维护工作由电机任务异步执行，ROS
回调不直接进行阻塞式 CAN 操作。

`/stm32/zero_position/state` 的 `name` 与标准关节顺序一致，`position` 为 7 个
RID54 偏置，`header.frame_id="motor_zero_offset"`；velocity/effort 为空。
`/stm32/zero_position/status.data` 固定为：

```text
[sequence, operation, requested_mask, success_mask, failure_mask, result]
```

operation：`0=NONE`、`1=GET`、`2=SET`。result：`0=处理中/尚无结果`、`1=成功`、
`2=无效掩码`、`3=忙（保留）`、`4=未处于安全 DISABLE`、`5=反馈/运动状态不安全`、
`6=CAN/寄存器 I/O 失败`、`7=设零验证失败`、`8=电机桥尚未初始化`。并发维护
请求不会排队；当前请求完成前的新请求会被忽略。

joint1 操作示例：

```bash
# 读取零点
ros2 topic pub --once /stm32/zero_position/get std_msgs/msg/UInt32 '{data: 1}'
ros2 topic echo /stm32/zero_position/status --once
ros2 topic echo /stm32/zero_position/state --once

# 将当前位置设为零点（先支撑机械结构并确认 DISABLE）
ros2 topic pub --once /stm32/zero_position/set std_msgs/msg/UInt32 '{data: 1}'
```

## 修改与兼容性要求

修改 topic、消息字段、关节顺序、单位、QoS、频率、模式语义、限幅、超时、故障码
或维护命令时，必须在同一提交中更新本文档。自定义消息变化还必须重新构建
`libmicroros.a`，并确保 Jetson 工作空间安装完全相同的消息定义。

## micro-ROS Session 与诊断

活动 session 每 1 秒执行一次最长 20 ms 的 Agent ping。该上限低于上位机默认
100 ms JointState timeout，避免同步探测本身制造超时空档。单次失败不会销毁实体；
只有连续 3 次 ping 失败才重建 session。`rclc_executor_spin_some()` 的
`RCL_RET_TIMEOUT` 表示等待期内没有就绪数据，按正常结果处理；其他 executor 错误
也必须连续出现 3 次才触发重建。executor 最长等待 1 ms，以保证 100 Hz
JointCommand 输入下及时排空 USB RX 环形缓冲。该通信判定不改变 TRACK 100 ms 转 HOLD 和命令
丢失 500 ms 锁存 fault 11 的电机安全策略。

USB CDC 使用异步传输。micro-ROS `write()` 将完整字节流复制到 RAM_D2 中的
16 KiB TX 环形队列后立即返回；独立 `usbTx` 任务以最多 512 字节一批提交 CDC，
发送完成中断通过 FreeRTOS semaphore 唤醒该任务。OTG HS IRQ 使用可调用
`FromISR` API 的优先级 5。RX 环形队列为 8 KiB。发布周期为 4 ms（250 Hz 目标），
验收要求在 100 Hz JointCommand 输入下 Jetson 实收 JointState 持续高于 200 Hz。
旧阻塞式写入同时发布 JointState/HardwareStatus 时仅约 75 Hz，且会造成 RX overflow。

`/stm32/microros_diagnostics.data` 固定为以下 16 项，计数器从 STM32 启动后累计：

```text
[schema_version, session_generation, last_exit_reason,
 last_spin_return_code, last_ping_return_code,
 consecutive_ping_failures, consecutive_spin_failures,
 spin_failure_count, ping_failure_count, publish_failure_count,
 usb_rx_overflow_count, usb_write_timeout_count,
 joint_state_publish_count, max_joint_state_publish_gap_ms,
 usb_tx_overflow_count, usb_tx_high_water_mark]
```

- `schema_version` 当前为 1。
- `session_generation` 每次成功创建完整 ROS 实体后加一；正常运行应保持不变。
- `last_exit_reason`：`0=NONE`、`1=连续 spin 错误`、`2=连续 ping 失败`、
  `3=实体初始化失败`。
- return code 保存最近一次底层 `rcl_ret_t`/`rmw_ret_t` 数值；
  `RCL_RET_TIMEOUT` 本身不增加错误计数。
- USB overflow/write timeout 或 session generation 持续增长，说明仍存在传输层或
  Agent 稳定性问题。可用相邻两次 `joint_state_publish_count` 之差核对实际发布率。
- `usb_tx_overflow_count` 表示 16 KiB TX 队列无法容纳完整 XRCE 写入的次数；正常为
  0。`usb_tx_high_water_mark` 用于核对最坏积压量，必须始终小于 16383。
- `max_joint_state_publish_gap_ms` 是启动以来两次成功 JointState 发布之间的最大
  本地间隔；在上位机使用 100 ms timeout 时应始终显著小于 100。
