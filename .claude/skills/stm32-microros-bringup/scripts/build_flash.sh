#!/usr/bin/env bash
set -euo pipefail

flash=false
if [[ "${1:-}" == "--flash" ]]; then
  flash=true
elif [[ $# -ne 0 ]]; then
  echo "usage: $0 [--flash]" >&2
  exit 2
fi

repo_dir="$(cd "$(dirname "$0")/../../../.." && pwd)"
cd "$repo_dir"

cmake --preset Debug
cmake --build build/Debug
arm-none-eabi-objcopy -O binary \
  build/Debug/rebotarm_executor.elf \
  build/Debug/rebotarm_executor.bin

if [[ "$flash" != true ]]; then
  echo "Build complete; pass --flash to program the target."
  exit 0
fi

probe_output="$(st-info --probe)"
printf '%s\n' "$probe_output"
probe_count="$(printf '%s\n' "$probe_output" | grep -c '^  serial:' || true)"
if [[ "$probe_count" -ne 1 ]]; then
  echo "Refusing to flash: expected exactly one ST-Link, found $probe_count." >&2
  exit 1
fi
if ! printf '%s\n' "$probe_output" | grep -q 'dev-type:.*STM32H72x_H73x'; then
  echo "Refusing to flash: probe target is not STM32H72x/H73x." >&2
  exit 1
fi

st-flash --reset write build/Debug/rebotarm_executor.bin 0x08000000
