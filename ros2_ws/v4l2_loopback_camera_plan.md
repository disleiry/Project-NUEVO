# Pi Camera -> Docker via V4L2 Loopback - Implementation Plan

Status: approved for implementation. This file documents the intended host
camera architecture and the acceptance checks for the implementation.

## Goals

The camera setup should be boring for students:

- one setup command on the Raspberry Pi host
- stable host service that starts at boot and restarts after crashes
- one sanity-check command that proves the camera works on native Ubuntu and
  from inside the ROS2 container
- no libcamera, PiSP, or Raspberry Pi camera debugging inside the ROS2 Docker
  container
- ROS camera nodes managed by launch files in the future

## Architecture

```text
Pi Camera Module 2 (imx219, CSI)
    |
    | native Ubuntu 25.10 libcamera/rpicam stack
    v
host camera feed service
    |
    | rpicam-vid -> ffmpeg/GStreamer producer
    v
/dev/video10
    |
    | v4l2loopback virtual camera
    v
ROS2 Docker container
    |
    | /dev/video10 mounted as a standard V4L2 camera
    v
ros-jazzy-v4l2-camera
    |
    v
ROS image topics
```

Key point: the host owns the real Pi camera. Docker only sees a regular V4L2
device.

## Why V4L2 Loopback

`camera_ros` is a libcamera wrapper, so it pulls the problematic libcamera layer
back into the ROS container. `v4l2_camera` is a better fit for this architecture
because it reads a standard V4L2 device and publishes ROS image topics.

This approach should avoid:

- building libcamera in the Jazzy container
- mounting `/dev/media*`, `/run/udev`, or raw Pi camera device nodes into Docker
- exposing students to Pi-specific camera internals

## Proposed File Layout

Use a host-camera directory, not `ros2_ws/docker/camera`, because the service
runs on native Ubuntu, not inside Docker.

```text
ros2_ws/host_camera/
├── install.sh                       # one-time/idempotent host setup
├── check.sh                         # one-command sanity check
├── pi-camera-feed             # installed to /usr/local/bin/
├── pi-camera.env              # installed to /etc/default/
└── pi-camera-feed.service     # installed to /etc/systemd/system/
```

The service file should be a real file in the repo, not only embedded in a
script. That makes review and maintenance easier. The installer can still copy
it into `/etc/systemd/system`.

## Host Installer Design

`install.sh` should be safe to re-run and should do the host setup end to end.

Responsibilities:

1. Verify this is Ubuntu, warning if it is not Ubuntu 25.10.
2. Enable `universe` if needed.
3. Run `apt-get update`.
4. Install required packages:
   - `rpicam-apps-lite` when available, otherwise `rpicam-apps`
   - `libcamera-tools`
   - `libcamera-ipa`
   - `v4l2loopback-dkms`
   - `v4l2loopback-utils`
   - `v4l-utils`
   - `ffmpeg`
   - `linux-headers-$(uname -r)` when available
5. Ensure `/boot/firmware/config.txt` has `camera_auto_detect=1`, with a backup
   before editing.
6. Configure v4l2loopback module autoload:
   - `/etc/modules-load.d/pi-camera.conf`
   - `/etc/modprobe.d/pi-camera.conf`
7. Load the module immediately for first setup.
8. Install the feed wrapper, env file, and systemd service.
9. Enable and start the service.
10. Print the sanity-check command at the end.

Proposed loopback defaults:

```text
PI_CAMERA_DEVICE=/dev/video10
PI_CAMERA_VIDEO_NR=10
PI_CAMERA_CARD_LABEL=Pi Camera
PI_CAMERA_WIDTH=1280
PI_CAMERA_HEIGHT=720
PI_CAMERA_FPS=15
PI_CAMERA_PIXEL_FORMAT=YUYV
```

Use `exclusive_caps=1` for compatibility with clients that expect a capture-only
camera device.

## Systemd Service Design

Prefer one durable feed service plus module autoload config.

```ini
[Unit]
Description=Pi camera feed to V4L2 loopback
After=systemd-modules-load.service
Wants=systemd-modules-load.service
StartLimitIntervalSec=0

[Service]
Type=simple
EnvironmentFile=/etc/default/pi-camera
ExecStart=/usr/local/bin/pi-camera-feed
Restart=on-failure
RestartSec=10

[Install]
WantedBy=multi-user.target
```

Notes:

- Use `Restart=on-failure` with a moderate delay so transient camera failures
  do not hammer the Pi camera driver.
- Put the camera pipeline in `/usr/local/bin/pi-camera-feed`, not directly
  in a long quoted `ExecStart`.
- The wrapper should use `set -Eeuo pipefail` so pipeline failures are visible
  to systemd.
- Do not use `ExecStop=/sbin/modprobe -r v4l2loopback`; unloading the module on
  service stop can surprise other readers and makes debugging harder.

## Feed Pipeline

Initial candidate:

```bash
rpicam-vid \
  --nopreview \
  --width "$PI_CAMERA_WIDTH" \
  --height "$PI_CAMERA_HEIGHT" \
  --framerate "$PI_CAMERA_FPS" \
  --codec mjpeg \
  --timeout 0 \
  --output - |
ffmpeg \
  -nostdin \
  -loglevel warning \
  -re \
  -i pipe:0 \
  -vf "scale=${PI_CAMERA_WIDTH}:${PI_CAMERA_HEIGHT},format=yuyv422" \
  -f v4l2 \
  "$PI_CAMERA_DEVICE"
```

YUYV/YUYV422 is the first target because `v4l2_camera` defaults to YUYV-style
input. If YUYV is too CPU-heavy or unreliable, the fallback candidate is MJPEG
through the loopback device and matching `v4l2_camera` parameters.

Open question to validate on hardware: whether `rpicam-vid -> ffmpeg` is more
stable than a GStreamer `libcamerasrc -> v4l2sink` pipeline on Ubuntu 25.10.
The plan should keep the wrapper script isolated so this can change without
changing Docker or ROS launch files.

## Sanity Check Design

Use one command:

```bash
./ros2_ws/host_camera/check.sh
```

The check should prove:

1. Host camera stack sees the real camera.
2. v4l2loopback module is loaded.
3. `/dev/video10` exists and has the expected card label.
4. The feed service is active.
5. The loopback device is producing frames on the host.
6. The same loopback device is visible inside the ROS2 container.
7. A frame can be captured from inside the ROS2 container.

For the normal student-facing check, do not use `rpicam-still` while the service
is running. The feed service owns the real camera. Instead:

- host test picture: capture one frame from `/dev/video10`
- container test picture: capture one frame from `/dev/video10` inside Docker

Example outputs:

```text
/tmp/pi_camera_check/host_loopback.jpg
/tmp/pi_camera_check/docker_loopback.jpg
```

Direct native `rpicam-still` can remain an advanced hardware-debug mode, but it
should stop and restart the feed service with a trap if used:

```bash
sudo systemctl stop pi-camera-feed
trap 'sudo systemctl start pi-camera-feed' EXIT
rpicam-still ...
```

The default check should avoid that disruption.

## Docker Changes

`Dockerfile.robot` should add only generic V4L2/ROS client dependencies:

```dockerfile
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-v4l2-camera \
        ffmpeg \
        v4l-utils \
    && rm -rf /var/lib/apt/lists/*
```

`docker-compose.rpi.yml` should mount the loopback device:

```yaml
devices:
  - /dev/ttyAMA0:/dev/ttyAMA0
  - /dev/rplidar:/dev/rplidar
  - /dev/video10:/dev/video10

environment:
  - PI_CAMERA_DEVICE=/dev/video10
```

Remove Pi camera internals from the ROS container:

- no `/dev/media*`
- no raw Pi camera `/dev/video0` through `/dev/video7`
- no `/run/udev` mount for libcamera
- no `LIBCAMERA_IPA_MODULE_PATH`
- no `python3-libcamera`
- no `picamera2`

## ROS Launch Plan

Do not require students to memorize `v4l2_camera` parameters.

Future ROS work should add a launch file, likely under the `vision` package:

```text
ros2_ws/src/vision/launch/pi_camera.launch.py
```

Student-facing command:

```bash
ros2 launch vision pi_camera.launch.py
```

The launch file should read `PI_CAMERA_DEVICE`, defaulting to `/dev/video10`,
and start `v4l2_camera_node` with the agreed image size and pixel format.

Use the documented `v4l2_camera` parameter style:

```bash
ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -p video_device:=/dev/video10 \
  -p image_size:="[1280,720]" \
  -p pixel_format:=YUYV \
  -p output_encoding:=rgb8
```

Topic naming should be decided in the launch file. Prefer one stable namespace,
for example:

```text
/camera/image_raw
/camera/camera_info
```

## Student Workflow

One-time setup on the Raspberry Pi host:

```bash
./ros2_ws/host_camera/install.sh
```

Anytime sanity check on the Raspberry Pi host:

```bash
./ros2_ws/host_camera/check.sh
```

Later, once launch files exist, start the ROS camera node inside Docker:

```bash
ros2 launch vision pi_camera.launch.py
```

Students should not need to know about `libcamera`, `rpicam-vid`, ffmpeg,
v4l2loopback module options, or Docker device internals for normal operation.

## What Can Go Wrong + Mitigations

| Issue | Mitigation |
|---|---|
| `v4l2loopback-dkms` fails to build | Installer should install/check `linux-headers-$(uname -r)` and print a clear failure if headers are unavailable. |
| `rpicam-apps` package unavailable | Installer should enable `universe`, run `apt-get update`, and fail with a clear Ubuntu-version message. |
| Real CSI camera not detected | Check should report `rpicam-hello --list-cameras` output and point to cable/config/reboot. |
| Feed service exits | `Restart=always`, `RestartSec=2`, and wrapper `pipefail` should make systemd restart the pipeline. |
| Direct `rpicam-still` check conflicts with service | Default check captures from `/dev/video10`; direct native capture is advanced mode only. |
| `/dev/video10` conflicts with another device | Keep video number configurable in `/etc/default/pi-camera`; check should print the active card label. |
| Docker cannot see `/dev/video10` | Check should use the known compose service, not a guessed container name, and print the compose device mapping to inspect. |
| `v4l2_camera` format mismatch | Validate YUYV first; keep MJPEG as fallback by changing host feed format and launch parameters together. |
| Students forget setup | `check.sh` should detect missing service/module/device and print the exact one-time setup command. |

## Implementation Steps After Approval

1. Reconcile or revert any earlier draft camera-service changes in the worktree.
2. Create `ros2_ws/host_camera/` files.
3. Add Docker package/device changes.
4. Add a future-facing `vision` launch file if approved for the same change.
5. Test manually on the Pi host:
   - install script
   - reboot behavior
   - service restart behavior
   - host sanity capture
   - Docker sanity capture
   - ROS `v4l2_camera` launch
6. Document the final student commands in `ros2_ws/README.md` and
   `ros2_ws/RPI_SETUP.md`.
