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
the official builder (nested Docker inside `container:` jobs is painful).

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

## Rollout / pin

1. Merge this PR (Dockerfile + `ci-image.yml` + split-job `ci.yml`).
2. Run **CI base image** (`workflow_dispatch`) once on `main` (or wait for the
   path-filter push that landed `docker/ci/**`).
3. Record the digest from the workflow summary.
4. Follow-up (optional): pin `ci.yml` `container:` to `@sha256:<digest>` instead
   of the floating `ubuntu22` tag.

Until the image exists on GHCR, the `build` job that uses `container:` will fail
to pull — publish first, then re-run CI.
