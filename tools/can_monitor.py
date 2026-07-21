#!/usr/bin/env python3
"""
USBCAN / 串口 CAN 数据监听脚本
支持多种 USBCAN 适配器协议自动检测，也支持解析 vofa FireWater 数据。

用法:
    python3 tools/can_monitor.py                       # 自动探测并监听
    python3 tools/can_monitor.py /dev/cu.usbmodemXXX   # 指定设备
    python3 tools/can_monitor.py --list                 # 列出所有串口

选项:
    /dev/cu.usbmodemXXX  指定串口设备
    --list               列出所有可用串口
    --raw                原始十六进制模式 (不尝试解析协议)
    --vofa               解析 vofa FireWater 协议 (4×float + 帧尾)
    --baud N             设置波特率 (默认 2000000)
    --duration N         监听时长秒数 (默认 60，0=无限)
    --filter-id 0xNNN    只显示指定 CAN ID
    --help               显示帮助
"""

import sys
import os
import time
import struct
import signal
import serial
import serial.tools.list_ports


# ── 配置 ─────────────────────────────────────────────────────────────────────

KNOWN_USBCAN_VIDS = {
    0x2E88: "HDSC",
    0x1A86: "CH340/CH9344",
    0x1D50: "OpenMoko (GS_USB)",
    0x04D8: "Microchip",
    0x0403: "FTDI",
    0x067B: "Prolific",
    0x10C4: "Silicon Labs",
    0x0C41: "ZLG",
}

# vofa FireWater 帧尾
VOFA_TAIL = b"\x00\x00\x80\x7f"


# ── 工具函数 ──────────────────────────────────────────────────────────────────

def hex_dump(data, prefix=""):
    """格式化十六进制 + ASCII 输出。"""
    lines = []
    for i in range(0, len(data), 16):
        chunk = data[i : i + 16]
        hex_str = " ".join(f"{b:02X}" for b in chunk)
        ascii_str = "".join(chr(b) if 32 <= b < 127 else "." for b in chunk)
        lines.append(f"{prefix}{hex_str:<48s} |{ascii_str}|")
    return "\n".join(lines)


def colorize(text, color_code):
    """终端颜色输出。"""
    if not sys.stdout.isatty():
        return text
    return f"\033[{color_code}m{text}\033[0m"


def green(text):
    return colorize(text, "32")


def cyan(text):
    return colorize(text, "36")


def yellow(text):
    return colorize(text, "33")


def red(text):
    return colorize(text, "31")


def dim(text):
    return colorize(text, "2")


# ── 设备发现 ──────────────────────────────────────────────────────────────────

def find_serial_ports():
    """扫描并分类串口设备。"""
    devices = []
    for p in serial.tools.list_ports.comports():
        info = {
            "device": p.device,
            "description": p.description or "",
            "manufacturer": p.manufacturer or "",
            "vid": p.vid,
            "pid": p.pid,
            "serial": p.serial_number or "",
            "category": "unknown",
        }

        desc = info["description"].upper()
        mfr = info["manufacturer"].upper()

        # 分类
        if "STLINK" in desc or "ST-LINK" in desc:
            info["category"] = "stlink"
        elif "VIRTUAL COMPORT" in desc or "STM32" in desc:
            info["category"] = "stm32_cdc"
        elif "CAN" in desc or "CAN" in mfr:
            info["category"] = "usbcn"
        elif info["vid"] in KNOWN_USBCAN_VIDS:
            info["category"] = "usbcn_candidate"

        devices.append(info)
    return devices


def list_ports():
    """列出所有串口设备。"""
    devices = find_serial_ports()
    print(f"\n{'设备':<40s} {'描述':<25s} {'厂商':<15s} {'类型':<15s} {'VID:PID'}")
    print("─" * 110)
    for d in devices:
        vid_pid = f"{d['vid']:04X}:{d['pid']:04X}" if d["vid"] else "N/A"
        cat = {
            "stlink": dim("[ST-Link]"),
            "stm32_cdc": cyan("[STM32 CDC]"),
            "usbcn": green("[USBCAN]"),
            "usbcan_candidate": yellow("[候选]"),
            "unknown": "[未知]",
        }.get(d["category"], "[未知]")
        print(f"{d['device']:<40s} {d['description']:<25s} {d['manufacturer']:<15s} {cat:<15s} {vid_pid}")
    print()


# ── vofa FireWater 解析 ───────────────────────────────────────────────────────

class VofaFireWaterParser:
    """解析 vofa FireWater 协议: 4×float32 + 0x0000807F 帧尾。"""

    def __init__(self, channels=4):
        self.channels = channels
        self.frame_size = channels * 4 + 4  # 4 floats + 4-byte tail
        self.buf = b""

    def feed(self, data):
        """喂入数据，返回解析出的帧列表。"""
        self.buf += data
        frames = []
        while True:
            idx = self.buf.find(VOFA_TAIL)
            if idx < 0:
                # 保留最后 frame_size-1 字节（可能跨包）
                if len(self.buf) > self.frame_size:
                    self.buf = self.buf[-(self.frame_size - 1) :]
                break
            # 帧尾前面应有 channels*4 字节数据
            data_start = idx - self.channels * 4
            if data_start >= 0:
                raw = self.buf[data_start : idx]
                try:
                    values = struct.unpack(f"<{self.channels}f", raw)
                    frames.append(values)
                except struct.error:
                    pass
            self.buf = self.buf[idx + 4 :]  # 跳过帧尾
        return frames


# ── CAN 协议探测器 ───────────────────────────────────────────────────────────

class CANProtocolDetector:
    """自动探测 USBCAN 协议并解析 CAN 帧。"""

    def __init__(self):
        self.protocol = None  # 检测到的协议名
        self.buf = b""
        self.stats = {"frames": 0, "bytes": 0, "errors": 0}

    def feed(self, data):
        """喂入数据，返回 (protocol_name, frames) 列表。"""
        self.buf += data
        self.stats["bytes"] += len(data)
        frames = []

        while len(self.buf) > 0:
            consumed = False

            # ── SLCAN (ASCII 文本协议) ────────────────────────────────
            # tIIIDL + data + \r  (标准 CAN)
            # TIIIIIIIIDL + data + \r  (扩展 CAN)
            # r/R (远程帧), e/E (CAN FD)
            if self.buf[0:1] in (b"t", b"T", b"r", b"R", b"e", b"E"):
                cr = self.buf.find(b"\r")
                if cr > 0:
                    line = self.buf[:cr].decode("ascii", errors="replace")
                    frame = self._parse_slcan(line)
                    if frame:
                        frames.append(frame)
                        self.protocol = "SLCAN"
                    self.buf = self.buf[cr + 1 :]
                    consumed = True
                    continue

            # ── GS_USB / gs_usb (Linux SocketCAN over USB) ──────────
            # 固定 16 字节帧 (classic) 或 72 字节 (CAN FD)
            # 格式: echo_id(4) + can_id(4) + dlc(1) + flags(1) + pad(1) + len(1) + data(8)
            if len(self.buf) >= 16:
                frame = self._try_gs_usb()
                if frame:
                    frames.append(frame)
                    self.protocol = "GS_USB"
                    self.buf = self.buf[16:]
                    consumed = True
                    continue

            # ── ZLG/创芯 (二进制协议, 帧头 AA 55) ───────────────────
            # AA 55 [cmd(1)] [ID_H(1) ID_L(1)] [DLC(1)] [Data(8)] [sum(1)]
            if len(self.buf) >= 4 and self.buf[0:2] in (b"\xaa\x55", b"\x55\xaa"):
                frame = self._try_zlg_protocol()
                if frame:
                    frames.append(frame)
                    self.protocol = "ZLG"
                    consumed = True
                    continue

            # ── 创芯科技 (CANFD-200U 等, 帧头 88) ───────────────────
            # 88 [len(1)] [type(1)] [cmd(1)] [ID(4)] [DLC(1)] [flags(1)] [data(N)] [sum(1)]
            if len(self.buf) >= 4 and self.buf[0] == 0x88:
                frame = self._try_cx_protocol()
                if frame:
                    frames.append(frame)
                    self.protocol = "CX/CANFD"
                    consumed = True
                    continue

            # ── 原始数据匹配失败 ─────────────────────────────────────
            # 尝试寻找下一个可能的帧头
            if not consumed:
                next_start = self._find_next_frame_start()
                if next_start > 0:
                    # 输出未识别的数据
                    unknown = self.buf[:next_start]
                    frames.append(("raw", unknown, None))
                    self.buf = self.buf[next_start:]
                    consumed = True
                elif next_start < 0:
                    # 没有找到任何帧头，丢弃前面的数据
                    if len(self.buf) > 256:
                        frames.append(("raw", self.buf[:64], None))
                        self.buf = self.buf[64:]
                        consumed = True

            if not consumed:
                break

        return self.protocol, frames

    def _find_next_frame_start(self):
        """在缓冲区中查找下一个可能的帧头位置。"""
        candidates = []
        for header in [b"\xaa\x55", b"\x55\xaa", b"\x88", b"t", b"T", b"r", b"R"]:
            idx = self.buf.find(header, 1)
            if idx >= 0:
                candidates.append(idx)
        return min(candidates) if candidates else -1

    def _try_gs_usb(self):
        """尝试解析 GS_USB 帧。"""
        # echo_id should be 0 for received frames
        echo_id = struct.unpack_from("<I", self.buf, 0)[0]
        can_id_raw = struct.unpack_from("<I", self.buf, 4)[0]
        dlc = self.buf[8]
        flags = self.buf[9]
        data_len = self.buf[11]

        # 基本合理性检查
        can_id = can_id_raw & 0x1FFFFFFF
        is_extended = bool(can_id_raw & 0x80000000)
        is_rtr = bool(can_id_raw & 0x40000000)
        is_fd = bool(can_id_raw & 0x20000000)

        if echo_id != 0:
            return None
        if not is_extended and can_id > 0x7FF:
            return None
        if is_extended and can_id > 0x1FFFFFFF:
            return None
        if dlc > 15:
            return None

        actual_len = min(dlc, 8) if not is_fd else min(dlc_to_len(dlc), 64)
        payload = self.buf[12 : 12 + min(actual_len, 8)]

        return ("can", can_id, payload, dlc, is_extended, is_rtr, is_fd)

    def _try_zlg_protocol(self):
        """尝试解析 ZLG 帧头 AA55 格式。"""
        if len(self.buf) < 6:
            return None
        # AA 55 [ID_H ID_L] [DLC] [Data...] [sum]
        can_id = (self.buf[2] << 8) | self.buf[3]
        dlc = self.buf[4]
        if dlc > 8 or can_id > 0x7FF:
            return None
        if len(self.buf) < 5 + dlc + 1:
            return None
        payload = self.buf[5 : 5 + dlc]
        # 验证校验和 (可选)
        self.buf_consumed = 5 + dlc + 1
        return ("can", can_id, payload, dlc, False, False, False)

    def _try_cx_protocol(self):
        """尝试解析创芯 0x88 帧头格式。"""
        if len(self.buf) < 4:
            return None
        frame_len = self.buf[1]
        if frame_len < 8 or frame_len > 76:
            return None
        if len(self.buf) < frame_len + 3:
            return None
        # 简单提取: 尝试从固定偏移读取 ID 和数据
        cmd = self.buf[3]
        if cmd not in (0x01, 0x02, 0x03):  # 常见接收命令
            return None
        # 保守处理: 跳过整个帧
        total = frame_len + 3
        return ("raw_frame", self.buf[:total], None)

    def _parse_slcan(self, line):
        """解析 SLCAN ASCII 帧。"""
        try:
            if line[0] == "t":
                # Standard CAN: tIIIL[data]
                can_id = int(line[1:4], 16)
                dlc = int(line[4])
                data_hex = line[5:]
                data = bytes.fromhex(data_hex) if data_hex else b""
                return ("can", can_id, data, dlc, False, False, False)
            elif line[0] == "T":
                # Extended CAN: TIIIIIIIIL[data]
                can_id = int(line[1:9], 16)
                dlc = int(line[9])
                data_hex = line[10:]
                data = bytes.fromhex(data_hex) if data_hex else b""
                return ("can", can_id, data, dlc, True, False, False)
            elif line[0] in ("r", "R"):
                # Remote frame
                can_id = int(line[1:4], 16) if line[0] == "r" else int(line[1:9], 16)
                return ("can", can_id, b"", 0, line[0] == "R", True, False)
        except (ValueError, IndexError):
            pass
        return None


def dlc_to_len(dlc):
    """CAN FD DLC 到实际数据长度的映射。"""
    mapping = {0:0,1:1,2:2,3:3,4:4,5:5,6:6,7:7,8:8,9:12,10:16,11:20,12:24,13:32,14:48,15:64}
    return mapping.get(dlc, dlc)


# ── 主监听逻辑 ───────────────────────────────────────────────────────────────

def monitor(port, baud=2000000, duration=60.0, mode="auto", filter_id=None):
    """
    主监听函数。
    mode: "auto" 自动探测, "raw" 原始数据, "vofa" vofa解析
    """
    print(f"\n{'═'*60}")
    print(f"  CAN / 串口 数据监听")
    print(f"  设备: {port}")
    print(f"  波特率: {baud}")
    print(f"  模式: {mode}")
    if filter_id is not None:
        print(f"  ID 过滤: 0x{filter_id:03X}")
    if duration > 0:
        print(f"  监听时长: {duration}s (Ctrl+C 提前退出)")
    else:
        print(f"  监听时长: 无限 (Ctrl+C 退出)")
    print(f"{'═'*60}\n")

    try:
        ser = serial.Serial(port, baud, timeout=0.1)
    except serial.SerialException as e:
        print(f"{red('✗')} 无法打开串口: {e}")
        print(f"\n提示: 如果端口被占用，请先关闭 USBCAN 上位机软件")
        return False

    # 发送空字节唤醒设备
    ser.write(b"\x00" * 8)
    time.sleep(0.05)

    vofa_parser = VofaFireWaterParser(channels=4)
    can_detector = CANProtocolDetector()

    start = time.time()
    total_bytes = 0
    frame_count = 0
    vofa_count = 0
    auto_detected = False

    def sigint_handler(sig, frame):
        print(f"\n\n{yellow('!')} 用户中断")
        print_stats()
        sys.exit(0)

    signal.signal(signal.SIGINT, sigint_handler)

    def print_stats():
        elapsed = time.time() - start
        print(f"\n{'─'*60}")
        print(f"  统计: {elapsed:.1f}s | {total_bytes} 字节 | {frame_count} CAN帧 | {vofa_count} vofa帧")
        if can_detector.protocol:
            print(f"  检测到协议: {green(can_detector.protocol)}")
        print(f"{'─'*60}")

    def format_can_frame(frame):
        """格式化 CAN 帧输出。"""
        _, can_id, data, dlc, ext, rtr, fd = frame
        id_str = f"0x{can_id:08X}" if ext else f"0x{can_id:03X}"
        data_hex = " ".join(f"{b:02X}" for b in data)
        flags = []
        if ext:
            flags.append("EXT")
        if rtr:
            flags.append("RTR")
        if fd:
            flags.append("FD")
        flag_str = f" [{' '.join(flags)}]" if flags else ""

        if filter_id is not None and can_id != filter_id:
            return None

        return f"ID={id_str:>10s}  DLC={dlc}  [{data_hex:<24s}]{flag_str}"

    try:
        while True:
            if duration > 0 and (time.time() - start) >= duration:
                break

            data = ser.read(4096)
            if not data:
                continue

            total_bytes += len(data)

            # ── vofa 模式 ────────────────────────────────────────────
            if mode == "vofa":
                frames = vofa_parser.feed(data)
                for values in frames:
                    vofa_count += 1
                    elapsed = time.time() - start
                    vals = "  ".join(f"CH{i}={v:+10.4f}" for i, v in enumerate(values))
                    print(f"[{elapsed:8.3f}s] vofa #{vofa_count:>6d}: {vals}")
                continue

            # ── 原始模式 ─────────────────────────────────────────────
            if mode == "raw":
                elapsed = time.time() - start
                print(f"[{elapsed:8.3f}s] {len(data)} 字节:")
                print(hex_dump(data, "  "))
                print()
                continue

            # ── 自动探测模式 ─────────────────────────────────────────
            protocol, frames = can_detector.feed(data)

            for frame in frames:
                elapsed = time.time() - start

                if frame[0] == "can":
                    frame_count += 1
                    line = format_can_frame(frame)
                    if line is not None:
                        print(f"[{elapsed:8.3f}s] CAN #{frame_count:>6d}: {line}")

                elif frame[0] == "raw" or frame[0] == "raw_frame":
                    raw_data = frame[1]
                    if not auto_detected and len(raw_data) > 0:
                        # 首次看到未识别数据，可能是协议握手/响应
                        print(f"[{elapsed:8.3f}s] {dim('待识别')} ({len(raw_data)}B): "
                              + " ".join(f"{b:02X}" for b in raw_data[:32]))

                # 自动切换到 vofa 检测
                if not auto_detected and vofa_parser.buf:
                    # 检查是否包含 vofa 帧尾
                    if VOFA_TAIL in data:
                        print(f"\n{cyan('ℹ')} 检测到 vofa FireWater 帧尾，切换到 vofa 解析模式")
                        print(f"  (重新运行并加 --vofa 参数可强制 vofa 模式)\n")
                        auto_detected = True

            if can_detector.protocol and auto_detected == False:
                print(f"\n{green('✓')} 检测到 CAN 协议: {green(can_detector.protocol)}")
                auto_detected = True

    except serial.SerialException as e:
        print(f"\n{red('✗')} 串口错误: {e}")
    finally:
        ser.close()

    print_stats()
    return True


# ── 入口 ──────────────────────────────────────────────────────────────────────

def main():
    port = None
    baud = 2000000
    mode = "auto"
    duration = 60.0
    filter_id = None

    args = sys.argv[1:]
    i = 0
    while i < len(args):
        arg = args[i]
        if arg == "--list":
            list_ports()
            return
        elif arg == "--raw":
            mode = "raw"
        elif arg == "--vofa":
            mode = "vofa"
        elif arg == "--baud" and i + 1 < len(args):
            i += 1
            baud = int(args[i])
        elif arg == "--duration" and i + 1 < len(args):
            i += 1
            duration = float(args[i])
        elif arg == "--filter-id" and i + 1 < len(args):
            i += 1
            filter_id = int(args[i], 0)
        elif arg == "--help" or arg == "-h":
            print(__doc__)
            return
        elif arg.startswith("/dev/"):
            port = arg
        i += 1

    # 自动探测设备
    if port is None:
        print("正在扫描串口设备...")
        devices = find_serial_ports()
        can_devices = [d for d in devices if d["category"] in ("usbcn", "usbcn_candidate")]
        other_devices = [d for d in devices if d["category"] not in ("stlink", "stm32_cdc")]

        if can_devices:
            if len(can_devices) == 1:
                port = can_devices[0]["device"]
                print(f"自动选择 USBCAN: {port}")
            else:
                print("发现多个 USBCAN 候选:")
                for i, d in enumerate(can_devices):
                    print(f"  [{i}] {d['device']}  {d['description']}")
                choice = input("选择 [0]: ").strip()
                idx = int(choice) if choice else 0
                port = can_devices[idx]["device"]
        elif other_devices:
            print("未识别 USBCAN，使用第一个非调试串口:")
            for d in other_devices:
                print(f"  {d['device']}  {d['description']}")
            port = other_devices[0]["device"]
        else:
            print(f"{red('✗')} 未发现可用串口")
            print("\n所有串口:")
            list_ports()
            print("请手动指定: python3 tools/can_monitor.py /dev/cu.usbmodemXXX")
            return

    # 如果是 STM32 CDC 端口但未指定模式，自动切换到 vofa
    if mode == "auto":
        devices = find_serial_ports()
        for d in devices:
            if d["device"] == port and d["category"] == "stm32_cdc":
                print(f"{cyan('ℹ')} 检测到 STM32 CDC 端口，自动切换到 vofa 模式")
                mode = "vofa"
                break

    monitor(port, baud, duration, mode, filter_id)


if __name__ == "__main__":
    main()
