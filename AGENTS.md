# Repository Guidelines

## Project Structure & Module Organization

This repository contains STM32H723 robotic-arm firmware. Application startup, interrupts, and FreeRTOS tasks live in `Core/Src/`, with matching headers in `Core/Inc/`. Device-specific modules such as the Damiao motor protocol and FDCAN wrapper belong in `User/`. USB CDC code is under `USB_DEVICE/`, while `vofa+/` provides telemetry. Treat `Drivers/`, `Middlewares/`, and files generated from `rebotarm_executor.ioc` as vendor or CubeMX-managed code. Build definitions are split between the root `CMakeLists.txt` and `cmake/stm32cubemx/`; integration notes live in `docs/`, and utilities in `scripts/`.

## Build, Test, and Development Commands

- `cmake --preset Debug` configures a Ninja debug build with `arm-none-eabi-gcc`.
- `cmake --build build/Debug` builds `rebotarm_executor.elf`.
- `cmake --preset Release && cmake --build build/Release` creates an optimized firmware build.
- `cmake --build build/Debug --target clean` removes debug build outputs.
- `./scripts/flash.sh` rebuilds Debug and flashes through ST-Link; add `--openocd` to use OpenOCD.
- `./scripts/monitor_motor.sh 30` monitors USB CDC motor telemetry for 30 seconds (requires Python `pyserial`).

## Coding Style & Naming Conventions

Use C11 and follow the existing STM32 HAL style: four-space indentation, braces on the next line for functions, and `snake_case` for project functions and files (for example, `damiao_protocol.c`). Keep public declarations in paired `.h` files and use uppercase include guards. In CubeMX-generated files, place every manual change inside `/* USER CODE BEGIN ... */` blocks so regeneration preserves it. Prefer CMSIS-RTOS v2 APIs (`osThreadNew`, `osDelay`) for tasks. Do not reformat vendor code or mix unrelated generated changes into a patch.

## Testing Guidelines

There is no automated unit-test target or coverage threshold. Every change must compile in Debug; build Release when changing optimization-sensitive, timing, or linker behavior. For hardware-facing changes, document the board setup and verify relevant FDCAN, motor, or USB CDC behavior. Include concise serial output or observed results in the pull request.

## Commit & Pull Request Guidelines

History follows Conventional Commit-style subjects such as `feat: integrate micro-ROS...` and `chore: ...`. Use an imperative, scoped summary (`fix: handle FDCAN timeout`) and keep commits focused. Pull requests should explain intent, identify affected peripherals/tasks, list build and hardware checks, and link related issues. Attach logs or screenshots for telemetry changes, and call out modifications to `.ioc`, linker scripts, pin mappings, clocks, or interrupt priorities.

## Safety & Configuration

Confirm the target MCU, probe, and flash address before programming; `flash.sh` writes at `0x08000000`. Never commit machine-specific serial ports, toolchain paths, credentials, or generated `build/` artifacts.
