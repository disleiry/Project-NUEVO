# Docker Setup

This folder contains the Docker files for the ROS 2 runtime containers.

Most students should use the Raspberry Pi setup:

```bash
docker compose -f ros2_ws/docker/docker-compose.rpi.yml up -d
```

The RPi container builds the ROS workspace on startup, starts the bridge launch
file automatically, and keeps the bridge running through Docker's restart
policy.

## Raspberry Pi Runtime

Start or update the RPi container from the repository root:

```bash
COMPOSE=ros2_ws/docker/docker-compose.rpi.yml

docker compose -f $COMPOSE build
docker compose -f $COMPOSE up -d
docker compose -f $COMPOSE logs -f ros2_runtime
```

The first startup can take about a minute while `colcon build` creates the ROS
workspace install. Later restarts reuse the cached Docker volumes.

The RPi container starts:

```bash
ros2 launch bridge bridge.launch.py
```

That launch file starts the `bridge` executable, which runs the shared bridge
web runtime and creates the ROS node named `/bridge`.

## Sanity Check

Run one command from the repository root:

```bash
./ros2_ws/docker/check_rpi_runtime.sh
```

Expected result:

```text
Results: 9 passed, 0 failed
```

The check verifies:

- Docker can see the RPi container.
- The Docker healthcheck is healthy.
- ROS discovery is using `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`.
- The ROS workspace has been built.
- `ros2 node list` can see `/bridge`.
- `http://127.0.0.1:8000/health` responds.

Manual node check:

```bash
docker compose -f ros2_ws/docker/docker-compose.rpi.yml exec ros2_runtime bash -lc \
  'source /ros2_ws/install/setup.bash && ros2 node list'
```

Expected bridge node:

```text
/bridge
```

Manual bridge health check:

```bash
python3 -c "import urllib.request; print(urllib.request.urlopen('http://127.0.0.1:8000/health', timeout=3).read().decode())"
```

## Restarting Docker

Restart only the ROS 2 runtime container:

```bash
COMPOSE=ros2_ws/docker/docker-compose.rpi.yml
docker compose -f $COMPOSE restart ros2_runtime
```

Stop and start it again:

```bash
docker compose -f $COMPOSE down
docker compose -f $COMPOSE up -d
```

Watch logs:

```bash
docker compose -f $COMPOSE logs -f ros2_runtime
```

Rebuild after Dockerfile or Python dependency changes:

```bash
docker compose -f $COMPOSE build
docker compose -f $COMPOSE up -d
```

Clear cached ROS build/install volumes and rebuild from scratch:

```bash
docker compose -f $COMPOSE down -v
docker compose -f $COMPOSE up -d
```

## Docker Healthcheck

The RPi compose file has a Docker healthcheck. It checks that:

1. `/ros2_ws/install/setup.bash` exists.
2. The bridge HTTP health endpoint responds inside the container.

View health status:

```bash
docker compose -f ros2_ws/docker/docker-compose.rpi.yml ps
```

Detailed health status:

```bash
CONTAINER=$(docker compose -f ros2_ws/docker/docker-compose.rpi.yml ps -q ros2_runtime)
docker inspect --format '{{json .State.Health}}' "$CONTAINER"
```

Important: Docker Compose does not restart a container only because it becomes
`unhealthy`. The bridge launch process is the container's main process, so if
the bridge crashes and exits, Docker restarts the container because the compose
file uses:

```yaml
restart: unless-stopped
```

If the container is still running but marked unhealthy, inspect logs and restart
it manually:

```bash
docker compose -f ros2_ws/docker/docker-compose.rpi.yml logs --tail=120 ros2_runtime
docker compose -f ros2_ws/docker/docker-compose.rpi.yml restart ros2_runtime
```

## ROS Discovery Isolation

RPi containers use:

```yaml
ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
```

This is the Jazzy replacement for the deprecated `ROS_LOCALHOST_ONLY=1` setting.
It keeps each robot's ROS graph local to that robot and prevents ROS nodes from
bleeding across teams on the same network.

Do not set `ROS_LOCALHOST_ONLY` in the RPi compose file. Setting the deprecated
variable can trigger Jazzy warnings even when the intended behavior still works.

The Jetson global GPS path does not require ROS DDS discovery between machines.
The Jetson runs a TCP server, and each robot's `robot_gps` node connects to the
hardcoded Jetson address and republishes detections locally on `/tag_detections`.

## Optional Hardware Devices

The RPi compose file bind-mounts the host `/dev` tree:

```yaml
volumes:
  - /dev:/dev
  - /run/udev:/run/udev:ro
```

It also grants access to the device classes used by:

- `/dev/video10` for the Pi camera loopback device
- `/dev/rplidar` for the RPLidar symlink
- `/dev/ttyAMA0` for the Arduino UART

Because the compose file does not hard-mount individual device paths with
`devices:`, the container can start even when optional hardware is unplugged.
ROS nodes should still handle missing devices in code and report clear errors.

## Development VM

The VM compose file is for development and mock mode:

```bash
docker compose -f ros2_ws/docker/docker-compose.vm.yml up -d
```

It does not manage real RPi hardware.

## Jetson Global GPS

The Jetson compose file is for the RealSense global GPS stack:

```bash
docker compose -f ros2_ws/docker/docker-compose.jetson.yml up -d
```

The Jetson publishes local ROS topics inside its own container and pushes robot
detections to RPis over TCP port `7777`.
