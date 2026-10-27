# STM32 FreeRTOS + micro-ROS firmware CI base image for rebotarm-executor.
#
# Bakes: ARM GCC toolchain + CMake + Ninja (+ minimal host tools).
# Does NOT bake: libmicroros.a / microros builder (cold miss stays on a bare
#                ubuntu-22.04 job that can run Docker; see ci.yml split jobs),
#                firmware source, flash / OpenOCD / ST-Link tooling.
#
# Image: ghcr.io/phamstered/rebotarm-executor-ci
# Prefer consuming by digest in .github/workflows/ci.yml after first publish.

FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8

# Ubuntu's /bin/sh is dash; pipefail needs bash.
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN set -eux \
 && apt-get update \
 && apt-get install -y --no-install-recommends \
      ca-certificates \
      git \
      gcc-arm-none-eabi \
      binutils-arm-none-eabi \
      cmake \
      ninja-build \
      make \
      python3 \
 && rm -rf /var/lib/apt/lists/* \
 && arm-none-eabi-gcc --version \
 && cmake --version \
 && ninja --version

WORKDIR /workspace
