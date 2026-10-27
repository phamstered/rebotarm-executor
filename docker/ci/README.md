# rebotarm-executor CI base image

Published as **`ghcr.io/phamstered/rebotarm-executor-ci`**.

Pre-warmed Ubuntu 22.04 toolchain image for GitHub Actions firmware builds.
Not a flash / hardware / micro-ROS agent image.

## What is baked

- Base: `ubuntu:22.04`
- `gcc-arm-none-eabi` / `binutils-arm-none-eabi`
- `cmake`, `ninja-build`, `make`
- `git`, `ca-certificates`, `python3` (minimal host helpers)

## What is **not** baked (v1)

- `libmicroros.a` / the `microros/micro_ros_static_library_builder` image
- Firmware source or a `build/Debug` tree
- OpenOCD / ST-Link / flash tooling
- Desk (`robot-debug-desk`) or Stack (`rebotarm-ros2-stack`) concerns

`libmicroros` stays on Actions cache + a bare-metal job that can `docker run`
the official builder (nested Docker inside `container:` jobs is painful). A
follow-up PR can split jobs or keep generation on the host when switching
`ci.yml` onto this image.

## Local build

From the repository root:

```bash
docker build -f docker/ci/Dockerfile -t ghcr.io/phamstered/rebotarm-executor-ci:ubuntu22 docker/ci
```

Smoke-check (optional):

```bash
docker run --rm ghcr.io/phamstered/rebotarm-executor-ci:ubuntu22 arm-none-eabi-gcc --version
```

Optional date tag:

```bash
docker tag ghcr.io/phamstered/rebotarm-executor-ci:ubuntu22 \
  ghcr.io/phamstered/rebotarm-executor-ci:ubuntu22-$(date +%Y%m%d)
```

## How CI publishes it

`.github/workflows/ci-image.yml` builds with Buildx and pushes to GHCR on:

- `workflow_dispatch`
- `push` to `main` when `docker/ci/**` or the publish workflow itself changes

Tags pushed:

- `ubuntu22` (floating)
- `ubuntu22-<short-sha>`
- `ubuntu22-YYYYMMDD`

The workflow prints the image **digest**. Prefer consuming
`ghcr.io/phamstered/rebotarm-executor-ci@sha256:<digest>` rather than a floating tag.

## Two-step rollout

1. **This change** — Dockerfile + publish workflow only. `.github/workflows/ci.yml`
   still runs on bare `ubuntu-22.04` with `apt-get install` for the toolchain
   (same green path as `main`).
2. **Follow-up PR** — after the image exists on GHCR, switch `ci.yml` to the
   container and **digest-pin**
   (`container: ghcr.io/phamstered/rebotarm-executor-ci@sha256:…`). Do not
   switch until the image exists and the digest is recorded.
