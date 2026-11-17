#!/usr/bin/env python3
"""Install the versioned MCU IDL overlay before generating libmicroros."""
from pathlib import Path
import shutil
root = Path(__file__).resolve().parents[1]
target = root / 'micro_ros_stm32cubemx_utils/microros_static_library/library_generation/extra_packages/microros_stm32h723_bridge'
if not target.parent.is_dir():
    raise SystemExit('Initialize micro_ros_stm32cubemx_utils submodule first')
shutil.copytree(root / 'interfaces/microros_stm32h723_bridge', target, dirs_exist_ok=True)
print('Installed firmware interface overlay')
