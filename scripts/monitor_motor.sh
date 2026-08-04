#!/bin/bash
# Damiao 电机监控脚本
# 用法: ./scripts/monitor_motor.sh [秒数]
# 默认监听 30 秒

DURATION=${1:-30}
PORT=$(ls /dev/tty.usbmodem* 2>/dev/null | head -1)

if [ -z "$PORT" ]; then
    echo "错误: 未找到 USB 串口设备"
    exit 1
fi

echo "监听端口: $PORT"
echo "监听时长: ${DURATION}秒"
echo "按 Ctrl+C 提前退出"
echo ""

/usr/bin/python3 -c "
import serial
import time
import struct

ser = serial.Serial('$PORT', 115200, timeout=1)
print('监听电机数据...')
print()

start = time.time()
while time.time() - start < $DURATION:
    if ser.in_waiting:
        data = ser.read(ser.in_waiting)
        if len(data) >= 24:
            for i in range(0, len(data) - 23, 24):
                frame = data[i:i+24]
                if frame[-4:] == b'\\x00\\x00\\x80\\x7f':
                    ch = []
                    for j in range(5):
                        val = struct.unpack('<f', frame[j*4:(j+1)*4])[0]
                        ch.append(val)

                    loop_count = int(ch[0])
                    target_pos = ch[1]
                    actual_pos = ch[2]
                    vel = ch[3]
                    torq = ch[4]

                    # 错误码检测
                    if loop_count < 0:
                        error_names = {-1: 'enable失败', -2: 'set_zero失败'}
                        print(f'错误: {error_names.get(loop_count, \"未知\")}, code={int(target_pos)}')
                    else:
                        print(f'#{loop_count:3d}: 目标={target_pos*57.3:6.1f}度, 实际={actual_pos*57.3:6.1f}度, 速度={vel:+.4f}, 力矩={torq:+.4f}')
    time.sleep(0.01)

ser.close()
print()
print('监听结束')
"
