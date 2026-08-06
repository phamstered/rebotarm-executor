---
name: stm32-microros-bringup
description: Build, flash, and validate this repository's STM32H723 FreeRTOS micro-ROS firmware using a macOS ST-Link and the Jetson host reached as `ssh damiao`. Use for firmware compilation, ST-Link programming, USB CDC/XRCE-DDS bring-up, ROS node/topic discovery, or non-actuating STM32-to-Jetson communication smoke tests.
---

# STM32 micro-ROS Bring-up

Run the repository-specific workflow without publishing motor commands.

## Safety rules

- Work from the repository root and preserve unrelated changes.
- Resolve exactly one STM32H72x/H73x ST-Link before flashing. Stop if zero or multiple probes appear.
- Flash only `build/Debug/rebotarm_executor.bin` at `0x08000000`.
- Never publish `/stm32/joint_commands` during a basic communication test.
- Treat agent process cleanup as external state. Inspect first; use `--replace-agent` only when authorized to replace an existing agent on `/dev/ttyACM0`.
- Report whether flashing verified, whether the node appeared, and whether `JointState` data arrived.

## 1. Build

Run:

```bash
.claude/skills/stm32-microros-bringup/scripts/build_flash.sh
```

This configures and builds Debug, then creates the BIN without flashing. Resolve build warnings before proceeding.

## 2. Flash

Run the same script with explicit authorization:

```bash
.claude/skills/stm32-microros-bringup/scripts/build_flash.sh --flash
```

USB access may require running outside the sandbox. Confirm `st-flash` reports both `Flash written and verified` and reset completion. Do not claim success from exit code alone when the verification text is absent.

## 3. Verify Jetson communication

Confirm Jetson exposes the board:

```bash
ssh damiao 'ls -l /dev/ttyACM*'
```

Then run:

```bash
.claude/skills/stm32-microros-bringup/scripts/verify_jetson.sh
```

If an existing agent owns the port and replacing it is authorized:

```bash
.claude/skills/stm32-microros-bringup/scripts/verify_jetson.sh --replace-agent
```

Success requires all of the following:

- Node `/stm32h723_motor_bridge` appears.
- Topics `/stm32/joint_commands`, `/stm32/joint_states`, and `/stm32/status` appear with expected types.
- One `JointState` sample contains only `joint1` during single-motor bring-up.
- `/stm32/joint_states` publishes near 50 Hz.

The Jetson must source `/opt/ros/humble/setup.bash` and `/home/jialiang/Project/microros_ws/install/setup.bash`. Source the leader/follower install only when custom-message deserialization is needed; topic discovery itself does not require it.

## Troubleshooting

- **Agent opens the port but no session:** confirm `/dev/ttyACM0` re-enumerated after reset and no second agent owns it.
- **Session exists but participant does not:** inspect FreeRTOS heap. Keep the micro-ROS task stack static so it does not consume the 32 KB heap.
- **Node appears but topics do not:** check agent logs for entity creation and remove competing agents before changing firmware.
- **Rate is much higher than 50 Hz:** inspect `ros2 topic info -v /stm32/joint_states`; stop mock or duplicate publishers before measuring.
- **CLI reports unknown custom package:** the Jetson plugin interfaces are not installed in the sourced environment. This affects `ros2 topic echo` for custom messages, not XRCE transport or graph discovery.
