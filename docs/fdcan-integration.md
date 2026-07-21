# FDCAN 通信集成文档

## 概述

将 dm-mc02 项目的 FDCAN 实现迁移到 rebotarm_executor，配置 FDCAN1 为 CAN FD + BRS 模式，实现周期性 CAN 数据发送。

## 架构

```
┌─────────────────────────────────────────────────┐
│              FreeRTOS 任务层                      │
│  StartCanDemoTask (100ms 周期)                   │
│    └─ fdcanx_send_data(&hfdcan1, 0x100, ...)    │
├─────────────────────────────────────────────────┤
│              BSP 驱动层 (User/)                   │
│  bsp_fdcan.c                                     │
│    ├─ bsp_can_filter_start()  滤波器+启动        │
│    ├─ fdcanx_send_data()      发送 API           │
│    ├─ fdcanx_receive()        接收 API           │
│    └─ HAL_FDCAN_RxFifo0Callback()  RX 回调       │
├─────────────────────────────────────────────────┤
│              HAL 驱动层                           │
│  main.c :: MX_FDCAN1_Init()  CubeMX 风格初始化   │
│  stm32h7xx_hal_msp.c         GPIO/时钟/NVIC      │
│  stm32h7xx_it.c              中断分发             │
├─────────────────────────────────────────────────┤
│              硬件层                               │
│  FDCAN1: PD0(RX) PD1(TX)  AF9                   │
└─────────────────────────────────────────────────┘
```

## FDCAN1 配置参数

| 参数 | 值 | 说明 |
|------|-----|------|
| 模式 | Normal, FD + BRS | CAN FD 带速率切换 |
| 自动重传 | 启用 | 发送失败自动重试 |
| Nominal 波特率 | **1Mbps** | PSC=1, Seg1=59, Seg2=20, SJW=20 |
| Data 波特率 | **5Mbps** | PSC=1, Seg1=13, Seg2=2, SJW=2 |
| 滤波器 | 1 个标准 ID 滤波器 | Mask 模式，接受所有 ID → RxFIFO0 |
| RxFIFO0 深度 | 8 元素 | 8 字节/元素 |
| TX FIFO 队列 | 8 元素 | FIFO 模式 |
| 中断优先级 | 6 | ≥ 5 兼容 FreeRTOS API |

## 修改的文件

### 新增文件

| 文件 | 说明 |
|------|------|
| `User/bsp_fdcan.h` | BSP 驱动头文件：API 声明 |
| `User/bsp_fdcan.c` | BSP 驱动实现：滤波器、启动、发送、接收、RX 回调 |

### 修改的文件

| 文件 | 修改内容 |
|------|----------|
| `Core/Src/main.c` | FDCAN1 配置改为 CAN FD (1M/5M)，添加 `#include "bsp_fdcan.h"`，调用 `bsp_can_filter_start()`，创建 CAN demo 任务 |
| `Core/Src/freertos.c` | 添加 `#include "bsp_fdcan.h"`，实现 `StartCanDemoTask()` |
| `Core/Src/stm32h7xx_it.c` | 添加 `#include "bsp_fdcan.h"`，`FDCAN1_IT0_IRQHandler` 和 `FDCAN1_IT1_IRQHandler` |
| `Core/Src/stm32h7xx_hal_msp.c` | FDCAN1_MspInit USER CODE 区添加 NVIC 配置（优先级 6） |
| `cmake/stm32cubemx/CMakeLists.txt` | `User/` 加入 include path，`bsp_fdcan.c` 加入源文件列表 |

## 关键实现细节

### 初始化流程

```c
// main.c 中
MX_FDCAN1_Init();           // CubeMX 风格：配置 FDCAN1 寄存器参数
bsp_can_filter_start();     // BSP：配置滤波器 → HAL_FDCAN_Start → 使能 RX 中断

// 然后 osKernelInitialize() 启动 FreeRTOS
// RTOS_THREADS 区创建 StartCanDemoTask
```

### 发送 API

```c
uint8_t fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id,
                          uint8_t *data, uint32_t len);
// 返回 0 成功，1 失败
// 支持 CAN FD DLC: 8/12/16/20/24/32/48/64 字节
// 自动设置 FD + BRS 标志
```

### 接收回调

```c
// bsp_fdcan.c 中注册的回调
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);
// FDCAN1 收到数据时自动调用 fdcan1_rx_callback()
```

### CAN Demo 任务

```c
void StartCanDemoTask(void *argument)
{
  uint8_t tx_data[8] = {0};
  uint32_t counter = 0;
  for(;;)
  {
    // 前 4 字节：递增计数器（大端）
    // 后 4 字节：固定 0xAA 0xBB 0xCC 0xDD
    tx_data[0..3] = counter (big-endian);
    tx_data[4..7] = {0xAA, 0xBB, 0xCC, 0xDD};
    fdcanx_send_data(&hfdcan1, 0x100, tx_data, 8);
    counter++;
    osDelay(100);  // 100ms 周期
  }
}
```

## 构建和烧录

```bash
# 配置
cmake --preset Debug

# 编译
cmake --build build/Debug

# 烧录（OpenOCD + ST-Link）
openocd -f interface/stlink.cfg -f target/stm32h7x.cfg \
  -c "program build/Debug/rebotarm_executor.elf verify reset exit"
```

## 验证结果

- ✅ 编译通过（54/54 目标，仅 MX_FDCAN2_Init 未使用警告）
- ✅ OpenOCD 烧录 + verify OK
- ✅ USB CDC vofa 数据正常（确认固件运行）
- ✅ FDCAN1 每 100ms 发送 ID=0x100 的 CAN FD 帧

## USBCAN 监听配置

| 设置项 | 值 |
|--------|-----|
| CAN 通道 | CAN1 (PD0/PD1) |
| 协议 | CAN FD |
| Nominal 波特率 | 1 Mbps |
| Data 波特率 | 5 Mbps |
| 期望 CAN ID | 0x100 |
| 数据长度 | 8 字节 |
| 发送周期 | 100ms |

## 注意事项

1. **中断优先级**：FDCAN1 中断优先级设为 6（≥ `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY=5`），确保可以在中断回调中安全调用 FreeRTOS API
2. **FDCAN2 未使用**：main.c 中保留了 `MX_FDCAN2_Init()` 函数但未调用，编译会有 unused 警告
3. **滤波器配置**：当前接受所有标准 ID（FilterID1=0x00, FilterID2=0x00, Mask 模式），生产环境需按实际需求修改
4. **CAN FD DLC 映射**：`fdcanx_send_data()` 和 `fdcanx_receive()` 支持 8~64 字节的 CAN FD DLC 映射
5. **HAL FDCAN 源文件**：`stm32h7xx_hal_fdcan.c` 已在 STM32_Drivers 库中编译，无需额外添加
