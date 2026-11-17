#!/usr/bin/env python3
"""Decode a captured J7B3 SWD dump. Never connects to hardware or sends commands."""
import argparse
import json
import math
from pathlib import Path
import struct

LAYOUT = struct.Struct('<4I4f12I12i2I3f')
MAGIC = 0x4A374233
REGISTERS = [('hw_ver', 13), ('sw_ver', 14), ('SN', 15), ('sub_ver', 36),
             ('Damp', 11), ('ACC', 4), ('DEC', 5), ('MAX_SPD', 6),
             ('PMAX', 21), ('VMAX', 22), ('TMAX', 23), ('Deta', 31)]


def json_float(value):
    return value if math.isfinite(value) else str(value)


def decode_dump(data):
    if len(data) != LAYOUT.size:
        raise ValueError(f'expected exactly {LAYOUT.size} bytes (J7B3), found {len(data)}')
    values = LAYOUT.unpack(data)
    if values[0] != MAGIC:
        raise ValueError(f'expected J7B3 magic 0x{MAGIC:08x}, found 0x{values[0]:08x}')
    raw, errors = values[8:20], values[20:32]
    registers = {}
    for i, (name, rid) in enumerate(REGISTERS):
        value = raw[i] if i < 4 else struct.unpack('<f', struct.pack('<I', raw[i]))[0]
        registers[name] = {'rid': rid, 'raw_hex': f'0x{raw[i]:08x}',
                           'value': value if i < 4 else json_float(value),
                           'error': errors[i], 'read_ok': bool(values[2] & (1 << i))}
    return dict(magic=f'0x{values[0]:08x}', complete=values[1],
                read_mask=f'0x{values[2]:x}', admitted_mask=f'0x{values[3]:x}',
                position_min=json_float(values[4]), position_max=json_float(values[5]),
                velocity_max=json_float(values[6]), startup_position=json_float(values[7]),
                registers=registers, trip_reason=values[32], trip_ms=values[33],
                trip_position=json_float(values[34]), trip_velocity=json_float(values[35]),
                trip_effort=json_float(values[36]),
                startup_diagnostic_ready=(values[1] == 1 and values[2] == 0xfff and
                    values[3] == 0x40 and values[32] == 0 and not any(errors)),
                note='Diagnostic admission only; fresh ROS boot/epoch/fault/DISABLE/readback proof is still required.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('dump', type=Path)
    args = parser.parse_args()
    try:
        result = decode_dump(args.dump.read_bytes())
    except (OSError, ValueError) as error:
        parser.error(str(error))
    print(json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False))


if __name__ == '__main__':
    main()
