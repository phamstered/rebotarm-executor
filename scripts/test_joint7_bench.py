#!/usr/bin/env python3
"""No hardware: verify fail-closed CMake inputs and flash script command routing."""
import os
from pathlib import Path
import shutil
import struct
import subprocess
import tempfile
import unittest

from decode_joint7_bench import decode_dump

ROOT = Path(__file__).resolve().parents[1]


class BenchDumpTest(unittest.TestCase):
    def dump(self, *, magic=0x4a374233, read_mask=0xfff, error=0, trip=0):
        raw = [0, 0x38313037, 0, 54]
        raw += [struct.unpack('<I', struct.pack('<f', x))[0]
                for x in [0.000212438, 2, -2, 600, 12.5, 30, 10, 4]]
        return struct.pack('<4I4f12I12i2I3f', magic, 1, read_mask, 0x40,
                           0.488, 0.509, 0.05, 0.508316, *raw,
                           *([0] * 11 + [error]), trip, 123, 0.49, -0.02, 0.1)

    def test_dump_layout_and_signed_values(self):
        result = decode_dump(self.dump())
        self.assertTrue(result['startup_diagnostic_ready'])
        self.assertEqual(result['registers']['DEC']['value'], -2)
        self.assertEqual(result['registers']['Deta']['rid'], 31)
        self.assertEqual(result['registers']['Deta']['value'], 4)
        self.assertEqual(result['trip_ms'], 123)
        self.assertAlmostEqual(result['trip_position'], 0.49)
        self.assertAlmostEqual(result['trip_velocity'], -0.02)
        self.assertAlmostEqual(result['trip_effort'], 0.1)

    def test_incomplete_deta_read_or_trip_is_not_ready(self):
        for changes in ({'read_mask': 0x7ff}, {'error': -3}, {'trip': 5}):
            self.assertFalse(decode_dump(self.dump(**changes))['startup_diagnostic_ready'])
        result = decode_dump(self.dump(read_mask=0x7ff, error=-3))
        self.assertFalse(result['registers']['Deta']['read_ok'])
        self.assertEqual(result['registers']['Deta']['error'], -3)

    def test_old_or_invalid_layout_is_rejected(self):
        for data in (bytes(140), self.dump(magic=0x4a374232), self.dump()[:-1], self.dump()+b'\0'):
            with self.assertRaises(ValueError):
                decode_dump(data)


class BenchBuildTest(unittest.TestCase):
    def test_cmake_validation(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / 'CMakeLists.txt').write_text(
                'cmake_minimum_required(VERSION 3.22)\nproject(test NONE)\n'
                f'include("{ROOT}/cmake/joint7-bench.cmake")\n')
            cases = [
                ({}, 'normal', True),
                ({'MOTOR_BENCH_JOINT7_POSVEL': 'OFF', 'MOTOR_BENCH_OVERSPEED_STOP': 'OFF'}, 'production_optout', False),
                ({'MOTOR_BENCH_OVERSPEED_STOP': 'OFF'}, 'supervised', True),
                ({'MOTOR_BENCH_EFFORT_STOP': 'OFF'}, 'effort_supervised', True),
                ({'MOTOR_BENCH_OVERSPEED_STOP': 'OFF', 'MOTOR_BENCH_EFFORT_STOP': 'OFF'}, 'both_supervised', True),
                ({'MOTOR_BENCH_JOINT7_POSVEL': 'OFF', 'MOTOR_BENCH_EFFORT_STOP': 'OFF'}, 'effort_production', False),
                ({'MOTOR_BENCH_JOINT7_POSVEL': 'ON'}, 'missing', False),
                ({'MOTOR_BENCH_POSITION_MIN': 'nan'}, 'nan', False),
                ({'MOTOR_BENCH_POSITION_MAX': '0.4'}, 'inverted', False),
                ({'MOTOR_BENCH_VELOCITY_MAX': '0.0'}, 'zero', False),
                ({'MOTOR_BENCH_VELOCITY_MAX': '0.1'}, 'speed_ceiling', True),
                ({'MOTOR_BENCH_VELOCITY_MAX': '0.100000001'}, 'fast', False),
                ({'MOTOR_BENCH_VELOCITY_MAX': '-0.1'}, 'negative_speed', False),
                ({'MOTOR_BENCH_VELOCITY_MAX': 'inf'}, 'nonfinite_speed', False),
                ({'MOTOR_BENCH_POSITION_MIN': '-12.6'}, 'range', False),
                ({}, 'build/Debug', False),
                ({}, 'bench', True),
            ]
            for changes, directory, expected in cases:
                values = {} if directory in ('normal', 'missing') else {
                    'MOTOR_BENCH_JOINT7_POSVEL': 'ON',
                    'MOTOR_BENCH_POSITION_MIN': '0.48',
                    'MOTOR_BENCH_POSITION_MAX': '0.51',
                    'MOTOR_BENCH_VELOCITY_MAX': '0.05',
                }
                values.update(changes)
                command = ['cmake', '-S', str(root), '-B', str(root / directory)]
                command.extend(f'-D{k}={v}' for k, v in values.items())
                result = subprocess.run(command, capture_output=True, text=True)
                self.assertEqual(result.returncode == 0, expected,
                                 (directory, result.stdout, result.stderr))

    def test_feedback_guard_default_and_supervised(self):
        with tempfile.TemporaryDirectory() as temporary:
            for mode, effort in ((None, None), ('0', None), (None, '0'), ('0', '0')):
                output = str(Path(temporary) / ('guard-' + str(mode) + '-' + str(effort)))
                command = ['cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                    '-I' + str(ROOT / 'User'), '-DMOTOR_BENCH_JOINT7_POSVEL=1',
                    '-DMOTOR_BENCH_POSITION_MIN=0.48f', '-DMOTOR_BENCH_POSITION_MAX=0.51f',
                    '-DMOTOR_BENCH_VELOCITY_MAX=0.1f']
                if mode is not None:
                    command.append('-DMOTOR_BENCH_OVERSPEED_STOP=' + mode)
                if effort is not None:
                    command.append('-DMOTOR_BENCH_EFFORT_STOP=' + effort)
                command += [str(ROOT / 'scripts/test_motor_bench.c'), '-o', output]
                subprocess.run(command, check=True, capture_output=True)
                subprocess.run([output], check=True, capture_output=True)

    def test_flash_script_routing(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / 'scripts').mkdir()
            shutil.copy2(ROOT / 'scripts/flash.sh', root / 'scripts/flash.sh')
            bin_dir = root / 'bin'
            bin_dir.mkdir()
            log = root / 'commands'
            for tool in ['cmake', 'st-flash', 'openocd', 'arm-none-eabi-objcopy']:
                script = bin_dir / tool
                script.write_text('#!/bin/bash\nset -e\n'
                                  'printf "%s\\n" "$(basename "$0") $*" >> "$COMMAND_LOG"\n'
                                  'if [[ "$(basename "$0")" == cmake && "$1" == --build ]]; then\n'
                                  ' mkdir -p "$2"; touch "$2/rebotarm_executor.elf"\nfi\n'
                                  'if [[ "$(basename "$0")" == arm-none-eabi-objcopy ]]; then touch "$4"; fi\n')
                script.chmod(0o755)
            env = dict(os.environ, PATH=f'{bin_dir}:{os.environ["PATH"]}', COMMAND_LOG=str(log))
            def run(*args):
                return subprocess.run(['/bin/bash', str(root / 'scripts/flash.sh'), *args],
                                      env=env, capture_output=True, text=True)
            bounds = ['--bench-joint7', '--position-min', '0.48',
                      '--position-max', '0.51', '--velocity-max', '0.05']
            self.assertNotEqual(run(*bounds).returncode, 0)  # explicit probe required
            self.assertFalse(log.exists())
            self.assertEqual(run(*bounds, '--build-only').returncode, 0)
            text = log.read_text()
            self.assertIn('BenchJoint7Debug', text)
            self.assertIn('-DMOTOR_BENCH_JOINT7_POSVEL=ON', text)
            for parameter in ['POSITION_MIN=0.48', 'POSITION_MAX=0.51', 'VELOCITY_MAX=0.05']:
                self.assertIn(parameter, text)
            self.assertNotIn('st-flash', text)
            self.assertIn('-DMOTOR_BENCH_OVERSPEED_STOP=ON', text)
            self.assertIn('-DMOTOR_BENCH_EFFORT_STOP=ON', text)
            log.write_text('')
            self.assertNotEqual(run('--disable-overspeed-stop', '--build-only').returncode, 0)
            self.assertEqual(log.read_text(), '')
            self.assertEqual(run(*bounds, '--disable-overspeed-stop', '--build-only').returncode, 0)
            self.assertIn('-DMOTOR_BENCH_OVERSPEED_STOP=OFF', log.read_text())
            self.assertIn('-DMOTOR_BENCH_EFFORT_STOP=ON', log.read_text())
            log.write_text('')
            self.assertNotEqual(run('--disable-effort-stop', '--build-only').returncode, 0)
            self.assertEqual(log.read_text(), '')
            self.assertEqual(run(*bounds, '--disable-overspeed-stop', '--disable-effort-stop', '--build-only').returncode, 0)
            self.assertIn('-DMOTOR_BENCH_OVERSPEED_STOP=OFF', log.read_text())
            self.assertIn('-DMOTOR_BENCH_EFFORT_STOP=OFF', log.read_text())
            log.write_text('')
            self.assertEqual(run(*bounds, '--build-only').returncode, 0)
            self.assertIn('-DMOTOR_BENCH_EFFORT_STOP=ON', log.read_text())
            self.assertIn('-DMOTOR_BENCH_OVERSPEED_STOP=ON', log.read_text())
            log.write_text('')
            self.assertEqual(run(*bounds, '--serial', 'ABC123').returncode, 0)
            self.assertIn('st-flash --serial ABC123 --reset write', log.read_text())
            log.write_text('')
            self.assertEqual(run(*bounds, '--serial', 'ABC123', '--openocd').returncode, 0)
            self.assertIn('adapter serial ABC123', log.read_text())
            log.write_text('')
            self.assertEqual(run('--build-only').returncode, 0)
            self.assertIn('-DMOTOR_BENCH_JOINT7_POSVEL=OFF', log.read_text())
            self.assertIn('-DMOTOR_BENCH_OVERSPEED_STOP=ON', log.read_text())
            self.assertNotIn('BenchJoint7Debug', log.read_text())
            self.assertEqual(run().returncode, 0)
            self.assertIn('st-flash --reset write', log.read_text())
            self.assertEqual(run('--openocd').returncode, 0)
            self.assertEqual(run('--build', '--build-only').returncode, 0)
            self.assertNotEqual(run('--position-min', '0.48').returncode, 0)
            self.assertNotEqual(run('--surprise').returncode, 0)


if __name__ == '__main__':
    unittest.main()
