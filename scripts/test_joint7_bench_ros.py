#!/usr/bin/env python3
"""Host-only C/Python wire interoperability and corrupt/incomplete capture tests."""
import copy
import json
from pathlib import Path
import subprocess
import tempfile
import unittest

from decode_joint7_bench_ros import decode_capture, decode_message, read_messages

ROOT = Path(__file__).resolve().parents[1]


class BenchRosTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.temporary.cleanup)
        cls.directory = Path(cls.temporary.name)
        binary = cls.directory / 'wire'
        command = ['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-I', str(ROOT / 'User'),
                   str(ROOT / 'scripts/test_motor_bench_ros.c'), '-o', str(binary)]
        subprocess.run(command + ['-DMOTOR_BENCH_JOINT7_POSVEL=1',
                       '-DMOTOR_BENCH_POSITION_MIN=0.48f', '-DMOTOR_BENCH_POSITION_MAX=0.51f',
                       '-DMOTOR_BENCH_VELOCITY_MAX=0.05f'], check=True, capture_output=True)
        subprocess.run([str(binary)], check=True, capture_output=True)
        cls.fixture = [json.loads(line) for line in subprocess.check_output(
            [str(binary), '--emit'], text=True).splitlines()]
        production = cls.directory / 'production'
        command[command.index('-o') + 1] = str(production)
        subprocess.run(command + ['-DMOTOR_BENCH_JOINT7_POSVEL=0'], check=True, capture_output=True)
        subprocess.run([str(production)], check=True, capture_output=True)
        startup_binary = cls.directory / 'startup'
        subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                        '-I', str(ROOT / 'User'), '-DMOTOR_BENCH_JOINT7_POSVEL=1',
                        '-DMOTOR_BENCH_POSITION_MIN=0.48f', '-DMOTOR_BENCH_POSITION_MAX=0.51f',
                        '-DMOTOR_BENCH_VELOCITY_MAX=0.05f',
                        str(ROOT / 'scripts/test_motor_bench_startup_trace.c'),
                        '-o', str(startup_binary)], check=True, capture_output=True)
        subprocess.run([str(startup_binary)], check=True, capture_output=True)
        cls.startup_fixture = [json.loads(line) for line in subprocess.check_output(
            [str(startup_binary), '--emit'], text=True).splitlines()]

    def setUp(self):
        self.snapshot, self.trace, self.calibration = copy.deepcopy(self.fixture)
        self.startup = copy.deepcopy(self.startup_fixture)

    def startup_with_records(self, records, **metadata):
        header = self.startup[0]['data'][:22]
        header[11] = len(records)
        indices = dict(version=7, armed=8, frozen=9, reason=10, count=11,
                       start_ms=12, enable_enqueued=13, enable_ms=14, command_sequence=15)
        for name, value in metadata.items():
            header[indices[name]] = value
        pages = []
        for offset in range(0, len(records), 8):
            chunk = records[offset:offset + 8]
            words = header[:] + [word for record in chunk for word in record]
            words[2], words[16], words[17] = len(words), offset, len(chunk)
            pages.append(dict(data=words))
        return pages

    def startup_raw_records(self):
        return [message['data'][index:index + 4] for message in self.startup
                for index in range(22, len(message['data']), 4)]

    def test_startup_c_prefix_survives_long_run_and_decodes_independently_of_tail(self):
        result = decode_capture([self.startup[1], self.startup[0], self.startup[1]])
        startup = result['startup_trace']
        self.assertEqual(result['trace']['status'], 'not_captured')
        self.assertTrue(startup['buffer_complete'])
        self.assertTrue(startup['startup_sequence_complete'])
        self.assertTrue(all(startup['evidence'].values()))
        self.assertEqual(startup['metadata']['command_sequence'], 123)
        self.assertEqual(startup['records'][1]['mode'], 0)  # actual first-enable HOLD
        self.assertEqual(startup['freeze_reason'], 'first_enable_window')
        self.assertEqual([record['kind'] for record in startup['records'][:6]],
                         ['startup_start', 'accepted_v2', 'tx_enqueue_ok', 'rx',
                          'tx_enqueue_ok', 'tx_enqueue_ok'])
        self.assertFalse(result['transmission_completion_proven'])
        self.assertFalse(result['motion_success_proven'])
        combined = decode_capture(self.fixture + self.startup)
        self.assertTrue(combined['trace']['retained_window_complete'])
        self.assertEqual(combined['startup_trace'], startup)
        self.assertEqual(decode_capture(self.fixture)['startup_trace']['status'], 'not_captured')

    def test_startup_hold_and_track_acceptance_must_precede_precise_seed_read(self):
        for mode in (0, 1):
            records = self.startup_raw_records()
            records[1][3] = mode | (2 << 8) | (0x40 << 16)
            startup = decode_capture(self.startup_with_records(records))['startup_trace']
            self.assertTrue(startup['startup_sequence_complete'])
        for mode in (2, 3):
            records = self.startup_raw_records()
            records[1][3] = mode | (2 << 8) | (0x40 << 16)
            self.assertFalse(decode_capture(self.startup_with_records(records))[
                'startup_trace']['startup_sequence_complete'])
        records = self.startup_raw_records()
        records[1], records[4] = records[4], records[1]
        records[1][0], records[4][0] = 100, 103
        startup = decode_capture(self.startup_with_records(records))['startup_trace']
        self.assertFalse(startup['startup_sequence_complete'])
        self.assertFalse(startup['evidence']['precise_read_request'])
        records = self.startup_raw_records()
        for record in records[3:]:
            record[0] += 100
        # A matching response received >100 ms after its read is stale.
        startup = decode_capture(self.startup_with_records(records, enable_ms=204))['startup_trace']
        self.assertFalse(startup['evidence']['precise_read_response'])

    def test_startup_missing_page_missing_head_and_conflicting_duplicates_are_detected(self):
        for pages in (self.startup[:1], self.startup[1:]):
            with self.assertRaisesRegex(ValueError, 'missing startup'):
                decode_capture(pages)
        conflict = copy.deepcopy(self.startup[0])
        conflict['data'][24] ^= 1
        with self.assertRaisesRegex(ValueError, 'conflicting duplicate startup'):
            decode_capture(self.startup + [conflict])
        conflict = copy.deepcopy(self.startup[1])
        conflict['data'][15] ^= 1
        with self.assertRaisesRegex(ValueError, 'conflicting frozen startup'):
            decode_capture(self.startup + [conflict])
        records = self.startup_raw_records()
        # All exported words can be present while the acquisition lacks START.
        records[0] = records[1][:]
        startup = decode_capture(self.startup_with_records(records))['startup_trace']
        self.assertTrue(startup['buffer_complete'])
        self.assertFalse(startup['startup_sequence_complete'])
        self.assertIn('start_marker', startup['incomplete_reasons'])

    def test_startup_each_required_stage_is_proven_by_records_not_frozen_metadata(self):
        cases = [(1, 2, 999, 'accepted_v2'),  # wrong command sequence
                 (2, 2, 0x51330007, 'precise_read_request'),
                 (3, 2, 0x51330007, 'precise_read_response'),
                 (4, 2, 0x3f000001, 'matching_position_preload'),
                 (4, 3, 0x7f800000, 'matching_position_preload'),
                 (4, 3, 0, 'matching_position_preload'),
                 (5, 3, 0xfdffffff, 'first_enable_enqueue')]
        for index, word, replacement, evidence in cases:
            records = self.startup_raw_records()
            records[index][word] = replacement
            with self.subTest(evidence=evidence, word=word):
                startup = decode_capture(self.startup_with_records(records))['startup_trace']
                self.assertFalse(startup['startup_sequence_complete'])
                self.assertFalse(startup['evidence'][evidence])
        records = self.startup_raw_records()
        for record in records[6:]:
            record[2] = 0x07  # output disabled, still a valid frame
        startup = decode_capture(self.startup_with_records(records))['startup_trace']
        self.assertFalse(startup['evidence']['enabled_feedback'])
        self.assertFalse(startup['startup_sequence_complete'])
        # FC TX_ERROR does not constitute a successful enable enqueue.
        records = self.startup_raw_records()
        records[5][1] = (records[5][1] & ~(15 << 18)) | (2 << 18)
        startup = decode_capture(self.startup_with_records(records))['startup_trace']
        self.assertFalse(startup['evidence']['first_enable_enqueue'])

    def test_startup_capacity_timeout_stop_and_trip_have_explicit_outcomes(self):
        records = self.startup_raw_records()
        while len(records) < 256:
            records.append(records[-1][:])
        startup = decode_capture(self.startup_with_records(records, reason=2))['startup_trace']
        self.assertTrue(startup['buffer_complete'])
        self.assertTrue(startup['truncated'])
        self.assertFalse(startup['startup_sequence_complete'])
        self.assertIn('capacity_truncated', startup['incomplete_reasons'])
        records = self.startup_raw_records()[:3]
        startup = decode_capture(self.startup_with_records(
            records, reason=3, enable_enqueued=0, enable_ms=0))['startup_trace']
        self.assertEqual(startup['freeze_reason'], 'no_enable_timeout')
        self.assertFalse(startup['startup_sequence_complete'])
        for reason, kind in ((4, 5), (5, 6)):
            records = self.startup_raw_records()
            records.append([108, (7 << 15) | (kind << 18), 124, 0])
            startup = decode_capture(self.startup_with_records(records, reason=reason))['startup_trace']
            self.assertTrue(startup['startup_sequence_complete'])
            self.assertEqual(startup['freeze_reason'], 'stop' if reason == 4 else 'trip')
            with self.assertRaisesRegex(ValueError, 'missing its stop/trip marker'):
                decode_capture(self.startup_with_records(records[:-1], reason=reason))

    def test_startup_order_wrong_seed_later_overwrite_and_wrap_are_checked(self):
        records = self.startup_raw_records()
        # A later mismatched preload invalidates an earlier matching preload.
        bad_preload = records[4][:]
        bad_preload[2] ^= 1
        records.insert(5, bad_preload)
        self.assertFalse(decode_capture(self.startup_with_records(records))[
            'startup_trace']['startup_sequence_complete'])
        records = self.startup_raw_records()
        records[4], records[5] = records[5], records[4]
        records[4][0], records[5][0] = 103, 104
        startup = decode_capture(self.startup_with_records(records, enable_ms=103))['startup_trace']
        self.assertFalse(startup['evidence']['matching_position_preload'])
        records = self.startup_raw_records()
        records[3][0] = 99
        startup = decode_capture(self.startup_with_records(records))['startup_trace']
        self.assertFalse(startup['record_ticks_monotonic'])
        self.assertFalse(startup['evidence']['precise_read_response'])
        self.assertFalse(startup['startup_sequence_complete'])
        # A queued old disabled feedback is preserved but is not enable proof.
        records = self.startup_raw_records()
        records[6][0] = 102
        startup = decode_capture(self.startup_with_records(records))['startup_trace']
        self.assertFalse(startup['record_ticks_monotonic'])
        self.assertTrue(startup['startup_sequence_complete'])  # newer RXs still prove it
        for record in records[7:]:
            record[0] = 102
        startup = decode_capture(self.startup_with_records(records))['startup_trace']
        self.assertFalse(startup['evidence']['enabled_feedback'])
        records = self.startup_raw_records()
        for record in records:
            record[0] = (record[0] - 102) & 0xffffffff
        startup = decode_capture(self.startup_with_records(
            records, start_ms=0xfffffffe, enable_ms=2))['startup_trace']
        self.assertTrue(startup['startup_sequence_complete'])

    def test_startup_corrupt_headers_bounds_live_capture_and_boot_are_rejected(self):
        for index, value in ((7, 2), (8, 2), (9, 2), (10, 6), (11, 257),
                             (13, 2), (14, 99), (16, 9), (17, 9), (18, 2),
                             (19, 512), (20, 5), (21, 1), (25, 1)):
            item = copy.deepcopy(self.startup[0])
            item['data'][index] = value
            with self.subTest(index=index), self.assertRaises(ValueError):
                decode_message(item)
        item = copy.deepcopy(self.startup[0])
        item['data'][9] = item['data'][10] = 0
        with self.assertRaisesRegex(ValueError, 'not frozen'):
            decode_capture([item])
        item = copy.deepcopy(self.startup[0])
        item['data'][3] ^= 1
        with self.assertRaisesRegex(ValueError, 'mixed boot'):
            decode_capture([item, self.startup[1]])
        # J7T1 remains closed to the new marker kind.
        item = copy.deepcopy(self.trace)
        item['data'][25] = (7 << 15) | (7 << 18)
        with self.assertRaisesRegex(ValueError, 'startup marker'):
            decode_message(item)

    def split_pages(self):
        first, second = copy.deepcopy(self.trace), copy.deepcopy(self.trace)
        first['data'] = first['data'][:40]
        first['data'][2] = 40
        first['data'][19] = 4
        second['data'] = second['data'][:24] + second['data'][40:]
        second['data'][2] = 40
        second['data'][18] = 0
        second['data'][19] = 4
        return first, second

    def test_c_encoder_roundtrip_preserves_original_admission_and_all_loop_rids(self):
        snapshot = decode_message(self.snapshot)
        self.assertEqual(snapshot['boot_id'], '0x1234567887654321')
        self.assertEqual(snapshot['runtime'], dict(free_heap_bytes=10000,
                         min_ever_free_heap_bytes=9000, microros_stack_high_water_words=777,
                         can_rx_overflow_count=5))
        self.assertEqual(len(snapshot['admission_raw_words']), 37)
        self.assertEqual(snapshot['admission_raw_words'], self.snapshot['data'][16:53])
        self.assertAlmostEqual(snapshot['admission']['trip_velocity'], -0.1)
        self.assertEqual([value['rid'] for value in snapshot['loop']['registers'].values()],
                         [20, 24, 25, 26, 27, 28, 32, 33, 34])
        self.assertEqual(snapshot['loop']['registers']['VL_c1']['raw_hex'], '0x40000008')
        self.assertEqual(snapshot['loop']['registers']['Gr'], dict(
            rid=20, raw_hex='0x40000000', error=0, read_ok=True, value=2.0, value_valid=True))

    def test_wraparound_pages_out_of_order_duplicates_are_not_motion_success(self):
        first, second = self.split_pages()
        result = decode_capture([self.snapshot, second, first, second])
        trace = result['trace']
        self.assertTrue(trace['retained_window_complete'])
        self.assertEqual([record['sequence'] for record in trace['records']],
                         [0xfffffffc, 0xfffffffd, 0xfffffffe, 0xffffffff, 0, 1, 2, 3])
        self.assertEqual(trace['records'][0]['kind'], 'tx_enqueue_ok')
        self.assertEqual(trace['records'][0]['data_hex'], '0000003fcdcc4c3d')
        self.assertFalse(result['transmission_completion_proven'])
        self.assertFalse(result['motion_success_proven'])

    def test_calibration_roundtrip_typed_rids_and_unchanged_historical_formats(self):
        result = decode_message(self.calibration)
        self.assertEqual(result['kind'], 'calibration')
        self.assertEqual(result['topic'], '/stm32/joint7_bench_calibration')
        self.assertEqual(result['boot_id'], '0x1234567887654321')
        self.assertEqual(result['mcu_tick_ms'], 125)
        self.assertEqual(result['calibration_raw_words'], self.calibration['data'][8:])
        diagnostics = result['calibration']
        self.assertEqual([diagnostics[key] for key in
                          ('version', 'complete', 'read_mask', 'started_ms', 'completed_ms')],
                         [1, 1, 255, 21, 29])
        registers = diagnostics['registers']
        self.assertEqual([value['rid'] for value in registers.values()],
                         [9, 12, 16, 17, 18, 19, 1, 30])
        self.assertEqual(registers['TIMEOUT'], dict(rid=9, data_type='uint32',
                         raw_hex='0x00002710', error=0, read_ok=True,
                         value=10000, value_valid=True))
        self.assertEqual(registers['NPP']['value'], 21)
        self.assertEqual(registers['Inertia']['data_type'], 'float32')
        self.assertEqual(registers['Inertia']['value'], 1.0 + 2**-23)
        self.assertEqual(registers['KT_Value']['rid'], 1)
        self.assertEqual(registers['GREF']['raw_hex'], '0x3f800007')
        self.assertEqual(self.snapshot['data'][:3], [0x4a375331, 1, 87])
        self.assertEqual(self.trace['data'][:3], [0x4a375431, 1, 56])
        old_capture = decode_capture([self.snapshot, self.trace])
        new_capture = decode_capture([self.snapshot, self.calibration, self.trace])
        self.assertEqual(new_capture['snapshots'], old_capture['snapshots'])
        self.assertEqual(new_capture['trace'], old_capture['trace'])
        self.assertEqual(old_capture['calibrations'], [])
        self.assertEqual(new_capture['calibrations'], [result])

    def test_calibration_only_and_failed_reads_do_not_imply_success(self):
        self.calibration['data'][10] = 0
        self.calibration['data'][21:29] = [0xfffffffa] * 8
        result = decode_capture([self.calibration])
        self.assertEqual(result['snapshots'], [])
        self.assertEqual(result['trace'], dict(status='not_captured', retained_window_complete=False))
        self.assertFalse(result['motion_success_proven'])
        self.assertFalse(result['transmission_completion_proven'])
        self.assertEqual(result['calibrations'][0]['calibration']['complete'], 1)
        for register in result['calibrations'][0]['calibration']['registers'].values():
            self.assertFalse(register['read_ok'])
            self.assertFalse(register['value_valid'])
            self.assertEqual(register['error'], -6)
        self.calibration['data'][9] = 0  # Initialized, collection has not ended.
        self.assertEqual(decode_message(self.calibration)['calibration']['complete'], 0)

    def test_calibration_nonfinite_float_and_uint32_max_preserve_raw(self):
        for bits, label in ((0x7fc00000, 'nan'), (0x7f800000, 'inf'), (0xff800000, '-inf')):
            with self.subTest(bits=bits):
                item = copy.deepcopy(self.calibration)
                item['data'][13:21] = [bits] * 8
                item['data'][13] = item['data'][15] = 0xffffffff
                result = decode_message(item)
                self.assertEqual(result['calibration_raw_words'], item['data'][8:])
                for name, register in result['calibration']['registers'].items():
                    if name in ('TIMEOUT', 'NPP'):
                        self.assertEqual(register['value'], 0xffffffff)
                        self.assertTrue(register['value_valid'])
                    else:
                        self.assertEqual(register['raw_hex'], f'0x{bits:08x}')
                        self.assertEqual(register['value'], label)
                        self.assertFalse(register['value_valid'])
                    self.assertTrue(register['read_ok'])
                json.dumps(result, allow_nan=False)

    def test_calibration_partial_reads_boot_consistency_and_malformed_fields(self):
        self.calibration['data'][10] &= ~(1 << 7)
        self.calibration['data'][28] = 0xfffffffd
        register = decode_message(self.calibration)['calibration']['registers']['GREF']
        self.assertEqual(register['error'], -3)
        self.assertFalse(register['read_ok'])
        self.assertFalse(register['value_valid'])
        for index, value in ((7, 1), (8, 2), (9, 2), (10, 0x100), (21, 1)):
            item = copy.deepcopy(self.calibration)
            item['data'][index] = value
            with self.subTest(index=index), self.assertRaises(ValueError):
                decode_message(item)
        self.calibration['data'][3] ^= 1
        for companion in (self.snapshot, self.trace, self.fixture[2]):
            with self.subTest(companion=companion), self.assertRaisesRegex(ValueError, 'mixed boot'):
                decode_capture([companion, self.calibration])

    def test_snapshot_only_and_failed_reads_never_mean_trace_success(self):
        self.snapshot['data'][55] &= ~(1 << 8)
        self.snapshot['data'][75] = 0xfffffffa  # DAMIAO_ERR_NOT_DISABLED
        result = decode_capture([self.snapshot])
        self.assertEqual(result['trace']['status'], 'not_captured')
        register = result['snapshots'][0]['loop']['registers']['VL_c1']
        self.assertFalse(register['read_ok'])
        self.assertEqual(register['error'], -6)
        self.assertFalse(result['motion_success_proven'])

    def test_nonfinite_raw_registers_remain_explicitly_invalid(self):
        self.snapshot['data'][58] = 0xff800000
        self.snapshot['data'][59] = 0x7fc00000
        self.snapshot['data'][28] = 0x7f800000
        result = decode_message(self.snapshot)
        loop = result['loop']['registers']['I_BW']
        self.assertEqual(loop['raw_hex'], '0x7fc00000')
        self.assertEqual(loop['value'], 'nan')
        self.assertFalse(loop['value_valid'])
        gr = result['loop']['registers']['Gr']
        self.assertEqual(gr['raw_hex'], '0xff800000')
        self.assertEqual(gr['value'], '-inf')
        self.assertFalse(gr['value_valid'])
        self.assertFalse(result['admission']['registers']['Damp']['value_valid'])
        json.dumps(result, allow_nan=False)

    def test_all_nonfinite_admission_float_fields_preserve_raw_and_emit_valid_json(self):
        float_indices = list(range(4, 8)) + list(range(12, 20)) + list(range(34, 37))
        for bits in (0x7fc00000, 0x7f800000, 0xff800000):
            with self.subTest(bits=bits):
                item = copy.deepcopy(self.snapshot)
                for index in float_indices:
                    item['data'][16 + index] = bits
                result = decode_message(item)
                self.assertEqual(result['admission_raw_words'], item['data'][16:53])
                encoded = json.dumps(result, allow_nan=False)
                admission = json.loads(encoded)['admission']
                for name in ('position_min', 'position_max', 'velocity_max', 'startup_position',
                             'trip_position', 'trip_velocity', 'trip_effort'):
                    self.assertIn(admission[name], ('nan', 'inf', '-inf'))
                for name in ('Damp', 'ACC', 'DEC', 'MAX_SPD', 'PMAX', 'VMAX', 'TMAX', 'Deta'):
                    self.assertFalse(admission['registers'][name]['value_valid'])

    def test_truncated_extra_words_bad_version_lengths_and_reserved_rejected(self):
        cases = []
        for original in (self.snapshot, self.trace, self.calibration):
            for index, value in ((1, 2), (2, 999), (6, 1)):
                item = copy.deepcopy(original)
                item['data'][index] = value
                cases.append(item)
            cases += [{'data': original['data'][:-1]}, {'data': original['data'] + [0]}]
        for index, value in ((10, 36), (11, 19), (14, 1), (15, 1), (53, 2), (55, 0x200)):
            item = copy.deepcopy(self.snapshot)
            item['data'][index] = value
            cases.append(item)
        for index, value in ((19, 9), (20, 2), (21, 511), (22, 5), (23, 1), (25, 1 << 22)):
            item = copy.deepcopy(self.trace)
            item['data'][index] = value
            cases.append(item)
        for item in cases:
            with self.subTest(item=item), self.assertRaises(ValueError):
                decode_message(item)

    def test_missing_pages_and_conflicting_duplicate_are_rejected(self):
        first, second = self.split_pages()
        with self.assertRaisesRegex(ValueError, 'missing trace'):
            decode_capture([self.snapshot, first])
        conflicting = copy.deepcopy(first)
        conflicting['data'][26] ^= 1
        with self.assertRaisesRegex(ValueError, 'conflicting duplicate'):
            decode_capture([first, second, conflicting])

    def test_mixed_boot_and_inconsistent_frozen_metadata_are_rejected(self):
        self.trace['data'][3] += 1
        with self.assertRaisesRegex(ValueError, 'mixed boot'):
            decode_capture([self.snapshot, self.trace])
        self.trace['data'][3] -= 1
        self.trace['data'][16] += 1
        with self.assertRaisesRegex(ValueError, 'conflicting frozen'):
            decode_capture([self.snapshot, self.trace])

    def test_live_trace_does_not_claim_complete_capture(self):
        self.trace['data'][14] = 0
        with self.assertRaisesRegex(ValueError, 'not frozen'):
            decode_capture([self.trace])

    def test_trace_window_and_trigger_consistency(self):
        for index, value in ((8, 12), (10, 513), (12, 2), (13, 0), (15, 1), (17, 33), (18, 4)):
            item = copy.deepcopy(self.trace)
            item['data'][index] = value
            with self.subTest(index=index), self.assertRaises(ValueError):
                decode_message(item)

    def test_overwritten_history_and_pagination_gap_are_explicit(self):
        pages = []
        for offset in range(0, 512, 8):
            page = copy.deepcopy(self.trace)
            page['data'][8:12] = [100, 612, 512, 100]
            page['data'][18] = 100 + offset
            page['data'][20] = int(offset == 0)
            pages.append(page)
        result = decode_capture(pages)
        self.assertTrue(result['trace']['prehistory_lost'])
        self.assertTrue(result['trace']['gap_seen'])
        self.assertEqual(len(result['trace']['records']), 512)
        self.assertFalse(result['motion_success_proven'])
        with self.assertRaisesRegex(ValueError, 'missing trace'):
            decode_capture(pages[:7] + pages[8:])

    def test_frozen_metadata_cannot_substitute_for_a_trigger_record(self):
        self.trace['data'][48:52] = self.trace['data'][24:28]
        with self.assertRaisesRegex(ValueError, 'trigger marker'):
            decode_capture([self.trace])

    def test_empty_capture_and_metadata_only_are_rejected(self):
        for items in ([], [{'metadata': True}], [{'topic': '/somewhere', 'message': {}}]):
            with self.assertRaisesRegex(ValueError, 'empty capture'):
                decode_capture(items)

    def test_uint32_types_layout_and_topic_validated(self):
        for value in (-1, 2**32, True, 1.0, '1', None):
            item = copy.deepcopy(self.snapshot)
            item['data'][5] = value
            with self.subTest(value=value), self.assertRaises(ValueError):
                decode_message(item)
        for layout in ({'dim': [], 'data_offset': 1}, {'dim': [1], 'data_offset': 0}):
            with self.assertRaises(ValueError):
                decode_message(dict(self.snapshot, layout=layout))
        with self.assertRaisesRegex(ValueError, 'topic and message'):
            decode_message(dict(topic='/stm32/joint7_bench_trace', msg=self.snapshot))

    def test_markers_and_rx_keep_their_distinct_semantics(self):
        item = copy.deepcopy(self.trace)
        item['data'][25:28] = [(7 << 15) | (4 << 18), 55, 1 | (2 << 8) | (0x40 << 16)]
        item['data'][29:32] = [(7 << 15) | (5 << 18), 56, 11]
        item['data'][33:36] = [(7 << 15) | (6 << 18), 57, 3]
        item['data'][37] = 0x17 | (8 << 11) | (7 << 15) | (3 << 18)
        records = decode_message(item)['records']
        self.assertEqual(records[0]['active_joint_mask'], 0x40)
        self.assertEqual(records[1]['fault_code'], 11)
        self.assertEqual(records[2]['trip_reason'], 3)
        self.assertEqual(records[3]['kind'], 'rx')

    def test_native_collector_metadata_other_topics_and_broken_bench_topic(self):
        native = dict(topic='/stm32/joint7_bench_snapshot', elapsed_ms=100, message=self.snapshot)
        result = decode_capture([{'metadata': {'host': 'test'}},
                                 {'topic': '/stm32/status', 'message': {'mode': 3}}, native])
        self.assertEqual(len(result['snapshots']), 1)
        with self.assertRaises(ValueError):
            decode_capture([dict(topic='/stm32/joint7_bench_snapshot', message={})])
        calibration = dict(topic='/stm32/joint7_bench_calibration', elapsed_ms=101,
                           message=dict(self.calibration, layout={'dim': [], 'data_offset': 0}))
        result = decode_capture([{'metadata': {'host': 'test'}}, native, calibration, self.trace])
        self.assertEqual(len(result['calibrations']), 1)
        self.assertTrue(result['trace']['retained_window_complete'])
        with self.assertRaises(ValueError):
            decode_capture([dict(calibration, message={})])
        with self.assertRaisesRegex(ValueError, 'topic and message'):
            decode_capture([dict(calibration, message=self.snapshot)])

    def test_json_jsonl_duplicate_keys_and_cli_are_strict(self):
        path = self.directory / 'capture.jsonl'
        path.write_text('\n'.join(json.dumps(message) for message in [self.snapshot, self.trace]))
        self.assertEqual(len(read_messages(path)), 2)
        result = subprocess.run(['python3', str(ROOT / 'scripts/decode_joint7_bench_ros.py'), str(path)],
                                text=True, capture_output=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertFalse(json.loads(result.stdout)['motion_success_proven'])
        path.write_text(json.dumps(dict(topic='/stm32/joint7_bench_calibration',
                                       elapsed_ms=123, message=self.calibration)))
        result = subprocess.run(['python3', str(ROOT / 'scripts/decode_joint7_bench_ros.py'), str(path)],
                                text=True, capture_output=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(len(json.loads(result.stdout)['calibrations']), 1)
        path.write_text(json.dumps([self.snapshot, self.trace]))
        self.assertEqual(len(read_messages(path)), 2)
        path.write_text('{"data":[],"data":[]}')
        with self.assertRaisesRegex(ValueError, 'duplicate JSON key'):
            read_messages(path)


if __name__ == '__main__':
    unittest.main()
