# Vofa+ USB CDC 集成

## 背景

将 dm-mc02 例程中的 vofa+ 波形上位机通信模块移植到 rebotarm_executor 项目，通过 USB OTG HS 以 CDC（虚拟串口）方式向 PC 发送波形数据。

## 源文件来源

从 `/Users/jialiang/Project/dm-mc02/例程/CtrBoard-H7_USB/` 拷贝：

```
vofa+/                          → 项目 vofa+/          (vofa.c, vofa.h)
USB_DEVICE/                     → 项目 USB_DEVICE/     (App + Target)
Middlewares/ST/                 → 项目 Middlewares/ST/  (USB Device Library)
```

## 架构

```
┌─────────────┐     ┌──────────────┐     ┌───────────────┐
│  vofa_demo   │────▶│ CDC_Transmit │────▶│  USB OTG HS   │──▶ PC (vofa+)
│  (FreeRTOS)  │     │    _HS()     │     │  PCD Device   │
└─────────────┘     └──────────────┘     └───────────────┘
```

- **协议**: FireWater（每帧 4×float + 帧尾 `0x00 0x00 0x80 0x7f`）
- **接口**: USB OTG HS，Embedded PHY，Full Speed
- **类**: USB CDC（虚拟串口），VID=1155, PID=22336

## 适配改动

### 移除 CubeMX 中的 PCD 初始化

`main.c` 中原有的 `MX_USB_OTG_HS_PCD_Init()` 和 `PCD_HandleTypeDef hpcd_USB_OTG_HS` 被移除，改由 USB Device 库的 `usbd_conf.c` 管理（通过 `MX_USB_DEVICE_Init()` → `USBD_Init()` → `USBD_LL_Init()` 链路）。

### MSP 冲突处理

| 函数 | 原位置 | 新位置 |
|------|--------|--------|
| `HAL_PCD_MspInit` | `stm32h7xx_hal_msp.c` | `usbd_conf.c` |
| `HAL_PCD_MspDeInit` | `stm32h7xx_hal_msp.c` | `usbd_conf.c` |

`stm32h7xx_hal_msp.c` 中仅保留 FDCAN 和 HCD 的 MSP 函数。

### USB 中断

在 `stm32h7xx_it.c` 中添加：

```c
extern PCD_HandleTypeDef hpcd_USB_OTG_HS;

void OTG_HS_IRQHandler(void)
{
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_HS);
}
```

### FreeRTOS 任务

`freertos.c` 中创建 `StartVofaTask`，10ms 周期（100Hz）调用 `vofa_demo()`：

```c
void StartVofaTask(void *argument)
{
  for(;;)
  {
    vofa_demo();
    osDelay(10);
  }
}
```

### CMake 构建

新增三个 OBJECT 库（`cmake/stm32cubemx/CMakeLists.txt`）：

| 库名 | 内容 |
|------|------|
| `USB_Device_Lib` | USB Device Core + CDC Class |
| `USB_Device_App` | usb_device.c, usbd_cdc_if.c, usbd_desc.c, usbd_conf.c |
| `VOFA` | vofa.c |

头文件搜索路径新增 `USB_DEVICE/App`、`USB_DEVICE/Target`、`Middlewares/ST/.../Inc`、`vofa+/`。

## 构建与烧录

```bash
# 构建
cmake --preset Debug
cmake --build build/Debug

# 烧录（OpenOCD + ST-Link）
openocd -f interface/stlink.cfg -f target/stm32h7x.cfg \
  -c "program build/Debug/rebotarm_executor.elf verify reset exit"
```

## 串口验证

```bash
# macOS 下监听 USB CDC 串口
/usr/bin/python3 -c "
import serial, time
ser = serial.Serial('/dev/tty.usbmodem3262388232331', 115200, timeout=2)
data = b''
start = time.time()
while time.time() - start < 3:
    chunk = ser.read(256)
    if chunk:
        data += chunk
        print(f'{len(chunk)} bytes: {chunk.hex()}')
ser.close()
"
```

验证结果：4 通道数据正确（计数器 + 3 路相位差 120° 正弦波），帧尾 `0000807f` 正确分隔。

## 注意事项

- 链接脚本 `STM32H723xG_flash.ld` 中的 `(READONLY)` 关键字需要 GCC 11+，GCC 9 需移除
- `usbd_conf.c` 中 `USBD_LL_Init` 内 PCD 配置 `vbus_sensing_enable = DISABLE`（与原 main.c 的 ENABLE 不同）
- vofa 发送函数 `CDC_Transmit_HS` 会在 USB 忙时返回 `USBD_BUSY`，当前任务未做重试处理
