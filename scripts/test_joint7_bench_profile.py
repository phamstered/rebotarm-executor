#!/usr/bin/env python3
"""Host protocol tests; imports no ROS and cannot publish or access hardware."""
import copy
import unittest
from joint7_bench_profile import (SOURCE, ACC_ORIGINAL, ACC_TARGET, DEC_ORIGINAL,
                                 decode_profile, request_words, evidence, validate_baseline)


class ProfileClientTest(unittest.TestCase):
    def setUp(self):
        self.words = [0x4a375031, 1, 32, 0x87654321, 0x12345678, 1000] + [0]*26
        self.boot = 0x1234567887654321
        self.profile = decode_profile(self.words)
        self.control = dict(boot_id=self.boot, disabled_verified=True, mode=3,
                            enabled_joint_mask=0, lease_id=0, switching=False,
                            owner_source_id=0, fault_code=0, control_epoch=3,
                            last_command_sequence=7, pos_vel_supported_mask=64)

    def test_request_binds_identity_and_fixed_profile_without_register_values(self):
        words = request_words('apply', self.boot, self.control, self.profile)
        self.assertEqual(words, [0x4a375131, 1, 14, 0x87654321, 0x12345678,
                                SOURCE & 0xffffffff, SOURCE >> 32, 1, 1, 1, 3, 7, 0, 1750])
        self.assertNotIn(ACC_TARGET, words)
        self.assertNotIn(ACC_ORIGINAL, words)

    def test_decode_preserves_bits_and_complete_status(self):
        words = self.words.copy()
        words[7] = 2
        words[9:25] = [1, SOURCE & 0xffffffff, SOURCE >> 32, 1, 1, 1, 0, 1, 1,
                      ACC_ORIGINAL, DEC_ORIGINAL, ACC_TARGET, DEC_ORIGINAL, 3, 1, 0]
        result = decode_profile(words)
        self.assertEqual(result['boot_id'], str(self.boot))
        self.assertEqual(result['source_id'], str(SOURCE))
        self.assertEqual(result['current_acc'], ACC_TARGET)
        self.assertTrue(result['applied_verified'])
        proof = evidence(result, 1234)
        self.assertEqual(proof['schema'], 'joint7_bench_profile_evidence_v1')
        self.assertEqual(proof['captured_at_ms'], 1234)

    def test_malformed_wire_rejected(self):
        cases = [self.words[:-1], self.words+[0]]
        for index, value in [(0, 0), (1, 2), (2, 33), (6, 12), (7, 7), (15, 2),
                             (22, 4), (0, True), (0, 1.0), (0, -1), (0, 2**32)]:
            bad = self.words.copy(); bad[index] = value; cases.append(bad)
        for words in cases:
            with self.subTest(words=words), self.assertRaises(ValueError):
                decode_profile(words)

    def test_apply_refuses_control_and_baseline_races(self):
        for key, value in [('boot_id', 1), ('disabled_verified', False), ('mode', 1),
                           ('enabled_joint_mask', 64), ('lease_id', 1), ('switching', True),
                           ('owner_source_id', 1), ('fault_code', 3), ('pos_vel_supported_mask', 0)]:
            control = dict(self.control, **{key: value})
            with self.subTest(key=key), self.assertRaises(ValueError):
                request_words('apply', self.boot, control, self.profile)
        for key, value in [('boot_id', '1'), ('busy', True), ('source_id', '42'),
                           ('operation_id', 0xffffffff), ('generation', 0xffffffff),
                           ('result', 5), ('original_valid', True), ('dirty', True)]:
            profile = dict(self.profile, **{key: value})
            with self.subTest(key=key), self.assertRaises(ValueError):
                request_words('apply', self.boot, self.control, profile)

    def test_restore_allows_fault_owner_only_with_recorded_original(self):
        profile = dict(self.profile, original_valid=True, dirty=True, generation=1,
                       operation_id=1, source_id=str(SOURCE), original_acc=ACC_ORIGINAL,
                       original_dec=DEC_ORIGINAL)
        control = dict(self.control, fault_code=3, owner_source_id=321)
        words = request_words('restore', self.boot, control, profile)
        self.assertEqual(words[7:13], [2, 2, 1, 3, 7, 1])
        for key, value in [('original_valid', False), ('dirty', False),
                           ('original_acc', ACC_TARGET), ('original_dec', 0)]:
            with self.subTest(key=key), self.assertRaises(ValueError):
                request_words('restore', self.boot, control, dict(profile, **{key: value}))
        with self.assertRaises(ValueError):
            request_words('restore', self.boot, dict(control, lease_id=1), profile)

    def test_deadline_wrap_does_not_wrap_operation_identity(self):
        profile = dict(self.profile, mcu_ms=0xffffff00)
        self.assertEqual(request_words('apply', self.boot, self.control, profile)[13], 494)

    def test_unknown_action_and_boot_rejected(self):
        for action, boot in [('save', self.boot), ('apply', 0), ('apply', 2**64)]:
            with self.assertRaises(ValueError):
                request_words(action, boot, self.control, self.profile)

    def test_baseline_requires_full_reads_and_original_values_and_gr(self):
        words = [0]*87
        words[:5] = [0x4a375331, 1, 87, self.boot & 0xffffffff, self.boot >> 32]
        words[10:13] = [37, 23, 11]
        words[16:20] = [0x4a374233, 1, 0xfff, 64]
        words[29:31] = [ACC_ORIGINAL, DEC_ORIGINAL]
        words[53:56] = [1, 1, 0x1ff]
        words[58] = 0x41200000
        validate_baseline(words, self.boot)
        for index, value in [(3, 1), (6, 1), (10, 36), (14, 1), (15, 1),
                             (17, 0), (18, 0x7ff), (19, 0), (29, ACC_TARGET),
                             (36, 1), (48, 4), (54, 0), (55, 255), (58, 0), (67, 1)]:
            bad = words.copy(); bad[index] = value
            with self.subTest(index=index), self.assertRaises(ValueError):
                validate_baseline(bad, self.boot)


if __name__ == '__main__':
    unittest.main()
