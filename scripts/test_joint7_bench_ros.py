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

    def setUp(self):
        self.snapshot, self.trace = copy.deepcopy(self.fixture)

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
        for original in (self.snapshot, self.trace):
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

    def test_json_jsonl_duplicate_keys_and_cli_are_strict(self):
        path = self.directory / 'capture.jsonl'
        path.write_text('\n'.join(json.dumps(message) for message in [self.snapshot, self.trace]))
        self.assertEqual(len(read_messages(path)), 2)
        result = subprocess.run(['python3', str(ROOT / 'scripts/decode_joint7_bench_ros.py'), str(path)],
                                text=True, capture_output=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertFalse(json.loads(result.stdout)['motion_success_proven'])
        path.write_text(json.dumps([self.snapshot, self.trace]))
        self.assertEqual(len(read_messages(path)), 2)
        path.write_text('{"data":[],"data":[]}')
        with self.assertRaisesRegex(ValueError, 'duplicate JSON key'):
            read_messages(path)


if __name__ == '__main__':
    unittest.main()
