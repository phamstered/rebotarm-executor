#!/usr/bin/env python3
"""Decode local UInt32MultiArray JSON/JSONL captures; no ROS or hardware access.

Input: {"layout":{"dim":[],"data_offset":0},"data":[uint32,...]} or
{"topic":"/stm32/joint7_bench_trace","msg":{...}}. A JSON array of messages
is also accepted. Snapshot/calibration-only files prove no captured trace.
Trace captures require a frozen, complete retained window; overwritten history
is reported as lost. HAL TX enqueue is never evidence of wire completion.
"""
import argparse
import json
import math
from pathlib import Path
import struct

from decode_joint7_bench import decode_dump

SNAPSHOT_MAGIC = 0x4A375331
TRACE_MAGIC = 0x4A375431
CALIBRATION_MAGIC = 0x4A374331
SNAPSHOT_WORDS = 87
TRACE_HEADER_WORDS = 24
CALIBRATION_WORDS = 29
MASK32 = 0xFFFFFFFF
TOPICS = {SNAPSHOT_MAGIC: '/stm32/joint7_bench_snapshot',
          TRACE_MAGIC: '/stm32/joint7_bench_trace',
          CALIBRATION_MAGIC: '/stm32/joint7_bench_calibration'}
# DM-J4310-2EC V1.1 manual v1.3 p18: RID20 Gr is a read-only float gear ratio.
# This observation alone does not identify or qualify the attached motor.
LOOP_REGISTERS = [('Gr', 20), ('I_BW', 24), ('KP_ASR', 25), ('KI_ASR', 26),
                  ('KP_APR', 27), ('KI_APR', 28), ('V_BW', 32),
                  ('IQ_c1', 33), ('VL_c1', 34)]
# Fixed startup read whitelist; these observations neither run calibration nor
# establish motor identity, valid tuning, or permission to move. TIMEOUT is u32
# in 50 us units and NPP is u32; neither may be decoded as float bits.
CALIBRATION_REGISTERS = [('TIMEOUT', 9, 'uint32'), ('Inertia', 12, 'float32'),
                         ('NPP', 16, 'uint32'), ('Rs', 17, 'float32'),
                         ('Ls', 18, 'float32'), ('Flux', 19, 'float32'),
                         ('KT_Value', 1, 'float32'), ('GREF', 30, 'float32')]
META_NAMES = ('version', 'first_sequence', 'next_sequence', 'count', 'overwritten',
              'armed', 'triggered', 'frozen', 'trigger_kind', 'trigger_ms', 'tail_count')
KINDS = {1: 'tx_enqueue_ok', 2: 'tx_enqueue_error', 3: 'rx',
         4: 'accepted_v2', 5: 'stop', 6: 'trip'}


def require(condition, message):
    if not condition:
        raise ValueError(message)


def signed(value):
    return value if value < 0x80000000 else value - 0x100000000


def float_bits(value):
    number = struct.unpack('<f', struct.pack('<I', value))[0]
    return number if math.isfinite(number) else str(number)


def decode_meta(words):
    require(len(words) == len(META_NAMES), 'trace metadata length mismatch')
    meta = dict(zip(META_NAMES, words))
    require(meta['version'] == 1, 'unsupported trace metadata version')
    require(meta['count'] <= 512 and
            (meta['next_sequence'] - meta['first_sequence']) & MASK32 == meta['count'],
            'invalid trace retained sequence interval')
    for flag in ('armed', 'triggered', 'frozen'):
        require(meta[flag] in (0, 1), f'invalid trace {flag} flag')
    require(meta['tail_count'] <= 32, 'invalid trace tail count')
    if meta['triggered']:
        require(meta['armed'] == 1 and meta['trigger_kind'] in (5, 6),
                'invalid trace trigger state')
    else:
        require(not any(meta[key] for key in ('frozen', 'trigger_kind', 'trigger_ms', 'tail_count')),
                'trace trigger metadata without trigger')
    require(meta['count'] == 512 or meta['overwritten'] == 0,
            'overwritten trace without full retained ring')
    return meta


def decode_record(words, sequence):
    tick, tag, data0, data1 = words
    require(tag >> 22 == 0, 'trace tag has reserved bits')
    kind, axis = (tag >> 18) & 15, (tag >> 15) & 7
    can_id, dlc = tag & 0x7FF, (tag >> 11) & 15
    require(kind in KINDS and axis == 7, 'unsupported trace kind or axis')
    record = dict(sequence=sequence, mcu_tick_ms=tick, kind=KINDS[kind], axis=axis,
                  raw_words=list(words))
    if kind <= 3:
        require(dlc == 8, 'bench CAN trace must have DLC=8')
        require(can_id == 0x17 if kind == 3 else can_id in (7, 0x107, 0x7FF),
                'unexpected joint7 CAN ID')
        data = struct.pack('<II', data0, data1)
        if can_id == 0x7FF:
            require(data[:2] == b'\x07\x00', 'register trace targets a different motor')
        record.update(can_id=f'0x{can_id:03x}', dlc=dlc, data_hex=data.hex())
    else:
        require(can_id == 0 and dlc == 0, 'marker contains CAN ID/DLC')
        record['command_sequence'] = data0
        if kind == 4:
            require(data1 >> 23 == 0, 'accepted marker has reserved bits')
            mode, control_mode, mask = data1 & 255, (data1 >> 8) & 255, (data1 >> 16) & 127
            require(mode in (0, 1, 2, 3) and control_mode in (1, 2),
                    'accepted marker has unsupported mode')
            record.update(mode=mode, control_mode=control_mode, active_joint_mask=mask)
        else:
            record['fault_code' if kind == 5 else 'trip_reason'] = data1
    return record


def extract_words(message):
    topic = None
    if isinstance(message, dict) and ('msg' in message or 'message' in message):
        require(not ('msg' in message and 'message' in message), 'ambiguous message wrapper')
        topic, message = message.get('topic'), message.get('msg', message.get('message'))
    elif isinstance(message, dict):
        topic = message.get('topic')
    require(isinstance(message, dict) and isinstance(message.get('data'), list),
            'expected UInt32MultiArray JSON object with data array')
    layout = message.get('layout', {'dim': [], 'data_offset': 0})
    require(isinstance(layout, dict) and layout.get('dim') == [] and
            type(layout.get('data_offset')) is int and layout['data_offset'] == 0,
            'bench layout must have empty dim and data_offset=0')
    words = message['data']
    require(all(type(value) is int and 0 <= value <= MASK32 for value in words),
            'data must contain only uint32 integers')
    require(7 <= len(words) <= SNAPSHOT_WORDS, 'invalid bench message length')
    require(words[0] in TOPICS, 'unknown bench magic')
    require(topic is None or topic == TOPICS[words[0]], 'topic and message magic disagree')
    require(words[1] == 1, 'unsupported bench wire version')
    require(words[2] == len(words), 'declared word length mismatch')
    require(words[6] == 0, 'common reserved word must be zero')
    return words


def decode_message(message):
    words = extract_words(message)
    result = dict(topic=TOPICS[words[0]], version=words[1],
                  boot_id=f'0x{words[4]:08x}{words[3]:08x}', mcu_tick_ms=words[5])
    if words[0] == SNAPSHOT_MAGIC:
        require(len(words) == SNAPSHOT_WORDS, 'snapshot must contain exactly 87 words')
        require(words[10:13] == [37, 23, 11], 'snapshot section lengths mismatch')
        require(words[14:16] == [0, 0], 'snapshot reserved words must be zero')
        require(words[8] <= words[7], 'minimum heap exceeds current heap')
        admission = words[16:53]
        require(admission[1] in (0, 1) and admission[2] & ~0xFFF == 0 and
                admission[3] in (0, 0x40), 'invalid J7B3 flags/masks')
        for index in range(12):
            require(not (admission[2] & (1 << index)) or admission[20 + index] == 0,
                    'J7B3 successful read bit has nonzero error')
        loop = words[53:76]
        require(loop[0] == 1, 'unsupported loop diagnostic version')
        require(loop[1] in (0, 1) and loop[2] & ~0x1FF == 0, 'invalid loop flags/mask')
        registers = {}
        for index, (name, rid) in enumerate(LOOP_REGISTERS):
            raw, error = loop[5 + index], signed(loop[14 + index])
            read_ok = bool(loop[2] & (1 << index))
            require(not read_ok or error == 0, 'loop successful read bit has nonzero error')
            registers[name] = dict(rid=rid, raw_hex=f'0x{raw:08x}',
                                   error=error, read_ok=read_ok)
            registers[name]['value'] = float_bits(raw)
            registers[name]['value_valid'] = isinstance(registers[name]['value'], float)
        admission_decoded = decode_dump(struct.pack('<37I', *admission))
        for register in admission_decoded['registers'].values():
            register['value_valid'] = isinstance(register['value'], (float, int))
        result.update(kind='snapshot',
                      runtime=dict(free_heap_bytes=words[7], min_ever_free_heap_bytes=words[8],
                                   microros_stack_high_water_words=words[9],
                                   can_rx_overflow_count=words[13]),
                      admission=admission_decoded,
                      admission_raw_words=admission,
                      loop=dict(version=loop[0], complete=loop[1], read_mask=loop[2],
                                started_ms=loop[3], completed_ms=loop[4], registers=registers),
                      trace_meta=decode_meta(words[76:87]))
        return result
    if words[0] == CALIBRATION_MAGIC:
        require(len(words) == CALIBRATION_WORDS, 'calibration must contain exactly 29 words')
        require(words[7] == 0, 'calibration reserved word must be zero')
        calibration = words[8:29]
        require(calibration[0] == 1, 'unsupported calibration diagnostic version')
        require(calibration[1] in (0, 1) and calibration[2] & ~0xFF == 0,
                'invalid calibration flags/mask')
        registers = {}
        for index, (name, rid, data_type) in enumerate(CALIBRATION_REGISTERS):
            raw, error = calibration[5 + index], signed(calibration[13 + index])
            read_ok = bool(calibration[2] & (1 << index))
            require(not read_ok or error == 0,
                    'calibration successful read bit has nonzero error')
            value = raw if data_type == 'uint32' else float_bits(raw)
            registers[name] = dict(rid=rid, data_type=data_type, raw_hex=f'0x{raw:08x}',
                                   error=error, read_ok=read_ok, value=value,
                                   value_valid=read_ok and isinstance(value, (float, int)))
        result.update(kind='calibration', calibration_raw_words=calibration,
                      calibration=dict(version=calibration[0], complete=calibration[1],
                                       read_mask=calibration[2], started_ms=calibration[3],
                                       completed_ms=calibration[4], registers=registers))
        return result
    require(len(words) >= TRACE_HEADER_WORDS, 'truncated trace header')
    require(words[21:24] == [512, 4, 0], 'invalid trace capacity, record size, or reserved word')
    count = words[19]
    require(count <= 8 and len(words) == TRACE_HEADER_WORDS + count * 4,
            'trace page record count/length mismatch')
    require(words[20] in (0, 1), 'invalid trace gap flag')
    meta = decode_meta(words[7:18])
    offset = (words[18] - meta['first_sequence']) & MASK32
    require(offset <= meta['count'] and offset + count <= meta['count'],
            'trace page is outside retained interval')
    result.update(kind='trace', trace_meta=meta, page_start=words[18], page_count=count,
                  gap=bool(words[20]), records=[
                      decode_record(words[24 + i * 4:28 + i * 4], (words[18] + i) & MASK32)
                      for i in range(count)])
    return result


def decode_capture(messages):
    decoded = []
    for message in messages:
        if isinstance(message, dict):
            if 'topic' in message and message['topic'] not in TOPICS.values():
                continue
            if 'topic' not in message and not any(key in message for key in ('data', 'msg', 'message')):
                continue  # Collector metadata, not a bench topic message.
        decoded.append(decode_message(message))
    require(bool(decoded), 'empty capture')
    require(len({message['boot_id'] for message in decoded}) == 1, 'mixed boot IDs in capture')
    snapshots = [message for message in decoded if message['kind'] == 'snapshot']
    calibrations = [message for message in decoded if message['kind'] == 'calibration']
    pages = [message for message in decoded if message['kind'] == 'trace']
    result = dict(boot_id=decoded[0]['boot_id'], snapshots=snapshots, calibrations=calibrations,
                  transmission_completion_proven=False, motion_success_proven=False,
                  note='TX enqueue is not wire completion; this capture is not motion qualification. '
                       'Ring overwrite and CAN RX queue overflow are separate losses; compare '
                       'the cumulative overflow counter before and after a test. '
                       'Calibration diagnostics are startup read observations; complete means '
                       'collection ended, not calibration success or motion admission.')
    if not pages:
        result['trace'] = dict(status='not_captured', retained_window_complete=False)
        return result
    frozen = [message['trace_meta'] for message in snapshots + pages
              if message['trace_meta']['frozen']]
    require(bool(frozen), 'trace is not frozen; acquisition completeness cannot be established')
    final = frozen[-1]
    require(all(meta == final for meta in frozen), 'conflicting frozen trace metadata')
    records = {}
    for page in pages:
        for record in page['records']:
            sequence = record['sequence']
            require(sequence not in records or records[sequence] == record,
                    f'conflicting duplicate trace record sequence {sequence}')
            records[sequence] = record
    expected = [(final['first_sequence'] + offset) & MASK32 for offset in range(final['count'])]
    missing = [sequence for sequence in expected if sequence not in records]
    require(not missing, f'missing trace records/pages: {len(missing)}; first={missing[:8]}')
    require(any(records[sequence]['kind'] == KINDS[final['trigger_kind']] and
                records[sequence]['mcu_tick_ms'] == final['trigger_ms'] for sequence in expected),
            'frozen trace is missing its trigger marker')
    result['trace'] = dict(status='frozen_retained_window', retained_window_complete=True,
                          prehistory_lost=bool(final['overwritten']),
                          gap_seen=any(page['gap'] for page in pages), metadata=final,
                          records=[records[sequence] for sequence in expected])
    return result


def unique_object(pairs):
    result = {}
    for key, value in pairs:
        require(key not in result, f'duplicate JSON key: {key}')
        result[key] = value
    return result


def read_messages(path):
    text = path.read_text(encoding='utf-8')
    try:
        value = json.loads(text, object_pairs_hook=unique_object)
        return value if isinstance(value, list) else [value]
    except json.JSONDecodeError:
        return [json.loads(line, object_pairs_hook=unique_object)
                for line in text.splitlines() if line.strip()]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('captures', type=Path, nargs='+')
    args = parser.parse_args()
    try:
        messages = [message for path in args.captures for message in read_messages(path)]
        result = decode_capture(messages)
    except (OSError, ValueError) as error:
        parser.error(str(error))
    print(json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False))


if __name__ == '__main__':
    main()
