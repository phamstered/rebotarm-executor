#!/usr/bin/env python3
"""Install the MCU IDL and entity-pool requirements before generating libmicroros."""
import json
from pathlib import Path
import shutil


def prepare(root):
    generation = root / 'micro_ros_stm32cubemx_utils/microros_static_library/library_generation'
    target = generation / 'extra_packages/microros_stm32h723_bridge'
    if not target.parent.is_dir():
        raise SystemExit('Initialize micro_ros_stm32cubemx_utils submodule first')
    # Six normal publishers plus five opt-in bench diagnostics publishers.
    # Preserve larger local pools and all unrelated generation settings.
    meta_path = generation / 'colcon.meta'
    meta = json.loads(meta_path.read_text())
    args = meta.setdefault('names', {}).setdefault('rmw_microxrcedds', {}).setdefault('cmake-args', [])
    prefix = '-DRMW_UXRCE_MAX_PUBLISHERS='
    existing = [int(arg[len(prefix):]) for arg in args if arg.startswith(prefix)]
    capacity = max([11, *existing])
    args[:] = [arg for arg in args if not arg.startswith(prefix)]
    args.append(f'{prefix}{capacity}')
    meta_path.write_text(json.dumps(meta, indent=4) + '\n')
    shutil.copytree(root / 'interfaces/microros_stm32h723_bridge', target, dirs_exist_ok=True)
    print(f'Installed firmware interface overlay; publisher pool >= {capacity}')


if __name__ == '__main__':
    prepare(Path(__file__).resolve().parents[1])
