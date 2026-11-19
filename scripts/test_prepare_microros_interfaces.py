#!/usr/bin/env python3
"""Exercise the reproducible entity-pool overlay without touching the submodule."""
import json
from pathlib import Path
import tempfile
import unittest

from prepare_microros_interfaces import prepare


class PrepareTest(unittest.TestCase):
    def test_pool_minimum_preserves_other_options_and_larger_capacity(self):
        for original, expected in ((None, 11), (10, 11), (16, 16)):
            with self.subTest(original=original), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                generation = root / 'micro_ros_stm32cubemx_utils/microros_static_library/library_generation'
                (generation / 'extra_packages').mkdir(parents=True)
                source = root / 'interfaces/microros_stm32h723_bridge'
                source.mkdir(parents=True)
                (source / 'test.msg').write_text('uint32 sample\n')
                args = ['-DRMW_UXRCE_MAX_SERVICES=1']
                if original is not None:
                    args.append(f'-DRMW_UXRCE_MAX_PUBLISHERS={original}')
                meta = {'names': {'rmw_microxrcedds': {'cmake-args': args}}, 'local': True}
                path = generation / 'colcon.meta'
                path.write_text(json.dumps(meta))
                prepare(root)
                first = path.read_bytes()
                prepare(root)
                self.assertEqual(first, path.read_bytes())
                result = json.loads(first)
                self.assertTrue(result['local'])
                self.assertEqual(result['names']['rmw_microxrcedds']['cmake-args'], [
                    '-DRMW_UXRCE_MAX_SERVICES=1', f'-DRMW_UXRCE_MAX_PUBLISHERS={expected}'])
                self.assertEqual((generation / 'extra_packages/microros_stm32h723_bridge/test.msg').read_text(),
                                 'uint32 sample\n')


if __name__ == '__main__':
    unittest.main()
