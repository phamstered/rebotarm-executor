# STM32H723 FreeRTOS micro-ROS 电机桥接实施计划

## 目标与范围

将当前 `std_msgs/Int32` 连通性示例升级为可安全控制 7 关节的 `stm32h723_motor_bridge` 节点。固件负责实时 MIT 控制、硬件保护、失联降级和状态反馈；ROS 通信不得阻塞控制与安全任务。

本计划以文末 checklist 为验收规范。实现过程中若接口、单位或安全阈值需要变更，应先同步修改上位机设计和 checklist。

## 首轮测试范围

功能初次实现后，实机只连接机械臂底座的一个电机，用于验证上位机 → micro-ROS → STM32 → FDCAN → 电机以及反馈反向链路。消息、状态机、关节顺序和内部数据结构仍按 7 关节设计；未连接的 6 个关节必须通过显式的启用掩码配置为停用，不得因反馈缺失触发整机故障，也不得向其发送 CAN 命令。该单电机配置仅用于分阶段验证，不改变最终 7 关节目标。

## 阶段 0：基线与安全边界

- [ ] 固化 Debug/Release 构建基线，消除未检查的 micro-ROS 返回值等警告。
- [ ] 恢复 FDCAN1/2、`bsp_fdcan.c` 和 `damiao_protocol.c` 的构建与初始化。
- [ ] 上电保持所有电机 `DISABLE`；任何初始化失败均不得使能电机。
- [ ] 将 7 关节 CAN ID、方向、零点、传动比和位置/速度/力矩限制集中到只读配置。

**完成标准：** 两种构建通过；断开 agent 上电时无电机输出；首轮配置能稳定识别底座电机反馈，最终配置能无丢帧地识别 7 路反馈。

## 阶段 1：实时任务与数据通道

- [ ] 建立静态 `motor_control_task`、`safety_task`、`micro_ros_task` 和 `diagnostic_task`，设置明确的优先级与栈预算。
- [ ] 用 ISR-safe 静态队列替换 FDCAN 单帧缓存，记录溢出、反馈时间戳和节点在线状态。
- [ ] 使用 `vTaskDelayUntil()` 或硬件定时器驱动控制周期，统计执行时间、最大抖动及 deadline miss。
- [ ] 建立命令双缓冲和状态快照；ROS 回调只校验、复制并原子切换。
- [ ] 周期记录各任务 stack high-water mark、heap 水位和队列峰值。

**完成标准：** ROS/USB 阻塞或断开不影响控制周期与 watchdog；并发访问通过压力测试。

## 阶段 2：控制、安全与状态机

- [ ] 实现 `HOLD`、`TRACK`、`RETURN`、`DISABLE`，上电默认 `DISABLE`。
- [ ] 校验模式、wrap-safe sequence、NaN/Inf、位置、速度、增益及广义力；拒绝值写入 fault code。
- [ ] 实现关节侧/电机侧换算、MIT 公式以及位置、速度、力矩和力矩变化率二次限幅。
- [ ] HOLD 首次进入锁存实测位置；RETURN 跟随限速轨迹；DISABLE 清零输出并等待驱动确认。
- [ ] 实现 `command_timeout` 的 TRACK→HOLD 和 `disable_timeout` 的 HOLD→DISABLE。
- [ ] 编码器/CAN 反馈过期、非有限反馈、驱动器故障或急停立即覆盖 ROS 命令并锁存故障。

**完成标准：** 离线向量验证 MIT 输出；所有异常输入和失联路径产生预期模式与 fault code。

## 阶段 3：ROS 实体与生命周期

- [ ] 引入 `JointCommand`、`JointState` 和 `HardwareStatus` 类型支持，固定 7 关节顺序及单位。
- [ ] 创建节点 `stm32h723_motor_bridge`、命令订阅者、两个发布者及 ROS 定时器。
- [ ] 使用 `rclc_executor`，核对 handle 数量；初始 QoS 使用 best-effort、keep-last(1)。
- [ ] 在初始化期一次性分配消息字符串和数组，运行循环不得重新分配。
- [ ] 实现 agent ping、epoch 同步、有限退避重连和实体逆序销毁；重连前先进入 DISABLE。
- [ ] 修复 USB CDC 环形缓冲区满检测、并发同步和错误/溢出统计。

**完成标准：** agent 启停 100 次无死锁、任务泄漏或持续内存增长；时间无效时正确上报且安全逻辑仍使用本地单调时钟。

## 阶段 4：系统验收

- [ ] 自动注入重复、乱序、未知模式、NaN、Inf 和越界命令。
- [ ] 验证串口断开后按配置时限进入 HOLD，再进入 DISABLE。
- [ ] 核对 7 关节顺序、符号、单位、时间戳和串口带宽。
- [ ] 保存控制周期抖动、deadline miss、栈余量、内存水位及 CAN/USB 丢包记录。
- [ ] 首轮只连接底座电机，验证命令 sequence/mode/参数下发、CAN 执行、反馈发布、故障上报及上下位机双向通信。
- [ ] 对首轮单电机测试执行“上电 DISABLE → 悬空低增益 HOLD → 小幅 TRACK → RETURN → 主动/超时 DISABLE”，全程保留物理急停。
- [ ] 单电机验收通过后，再按“其余单关节 → 六轴 → 夹爪 → 全系统”逐级扩展，禁止直接跳到多电机联调。

## 交付约束

每个阶段应独立提交并附构建结果、硬件配置和测试证据。CubeMX 生成文件的手工修改必须位于 `USER CODE` 区域。安全阈值、故障清除规则和超时时间必须在实机使能前完成评审。

## 引用

- 规范性来源：[STM32H723_FREERTOS_ROS2_NODE_CHECKLIST.md](../leader_follower/src/plugin/STM32H723_FREERTOS_ROS2_NODE_CHECKLIST.md)
- 仓库集成说明：[`docs/fdcan-integration.md`](docs/fdcan-integration.md)、[`docs/vofa-usb-cdc-integration.md`](docs/vofa-usb-cdc-integration.md)
