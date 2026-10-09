# Repository Guidelines

## Project Structure & Module Organization

Application startup and FreeRTOS tasks live in `Core/Src/`, with headers in `Core/Inc/`. Damiao and FDCAN modules belong in `User/`; USB CDC is under `USB_DEVICE/`, telemetry under `vofa+/`, notes under `docs/`, and utilities under `scripts/`. Treat `Drivers/`, `Middlewares/`, and `.ioc`-generated files as vendor/CubeMX-managed code.

## Build, Test, and Development Commands

- `cmake --preset Debug` configures a Ninja debug build with `arm-none-eabi-gcc`.
- `cmake --build build/Debug` builds `rebotarm_executor.elf`.
- `cmake --preset Release && cmake --build build/Release` creates an optimized firmware build.
- `cmake --build build/Debug --target clean` removes debug build outputs.
- `./scripts/flash.sh` rebuilds Debug and flashes through ST-Link; add `--openocd` to use OpenOCD.
- `./scripts/monitor_motor.sh 30` monitors USB CDC motor telemetry for 30 seconds (requires Python `pyserial`).

## Coding Style & Naming Conventions

Use C11 and STM32 HAL style: four-space indentation, next-line function braces, and `snake_case` project symbols/files. Keep public declarations in paired `.h` files with uppercase guards. Put manual generated-file changes inside `/* USER CODE BEGIN ... */` blocks. Prefer CMSIS-RTOS v2 APIs and do not reformat vendor code.

## Testing Guidelines

There is no automated unit-test target or coverage threshold. Every change must compile in Debug; build Release when changing optimization-sensitive, timing, or linker behavior. For hardware-facing changes, document the board setup and verify relevant FDCAN, motor, or USB CDC behavior. Include concise serial output or observed results in the pull request.

## ROS 2 Interface Contract

[`docs/ros2-interface.md`](docs/ros2-interface.md) is the authoritative firmware interface reference. Any change to a topic name, message type or field meaning, joint ordering, QoS, publish rate, control mode, limit, timeout, fault code, or maintenance command must update that document in the same patch. Keep documented behavior aligned with the implementation, including incomplete modes.

Use [`docs/microros-session-stability.md`](docs/microros-session-stability.md)
for the non-moving 100 Hz XRCE load test and Agent reconnect checks.

## Commit & Pull Request Guidelines

Use focused Conventional Commit-style subjects such as `fix: handle FDCAN timeout`. Pull requests should explain intent, affected peripherals/tasks, build and hardware checks, and related issues. Attach relevant telemetry evidence and call out `.ioc`, linker, pin, clock, or interrupt-priority changes.

## Safety & Configuration

Confirm the target MCU, probe, and flash address before programming; `flash.sh` writes at `0x08000000`. Never commit machine-specific serial ports, toolchain paths, credentials, or generated `build/` artifacts.
