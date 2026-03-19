# NUEVO UI

Web-based monitoring and control interface for the MAE 162 robotics platform.
A React/TypeScript frontend communicates over WebSocket with a FastAPI backend (`nuevo_bridge`) that bridges to the Arduino Mega 2560 via UART.

```
Browser  ←── WebSocket ──→  nuevo_bridge (FastAPI)  ←── UART/TLV ──→  Arduino Mega
(React)                       runs on RPi 5                              (firmware)
                                    │
                              ROS2 topics (optional)
                           see ros2_ws/ for details
```

---

## Prerequisites

| Tool | Version | Notes |
|------|---------|-------|
| Node.js | ≥ 18 | For the frontend dev server |
| npm | ≥ 9 | Bundled with Node.js |
| Python | ≥ 3.11 | For the backend |
| pip | latest | `pip install --upgrade pip` |

---

## Production build & deploy (RPi, no ROS2)

The frontend must be compiled to static files so `nuevo_bridge` can serve everything on a single port.

```bash
# From nuevo_ui/
cd frontend && npm run build && cd .. && cp -r frontend/dist/. backend/static/
```

Then start the backend — it serves both the API and the compiled UI:

```bash
# From nuevo_ui/
cd backend && NUEVO_SERIAL_PORT=/dev/ttyAMA0 python3 -m nuevo_bridge
```

Access the UI from any browser on the local network at `http://<rpi-hostname>:8000`.

Default login:
```
username: user
password: 162
```

---

## Development (frontend and backend separated)

The default dev setup runs with mock Arduino data — no hardware needed.

### Using the dev script

The easiest way to run both servers at once:

```bash
cd nuevo_ui
./scripts/dev.sh
```

This starts:
- **Backend** at `http://localhost:8000` — mock mode (no Arduino needed)
- **Frontend** at `http://localhost:5173` — Vite dev server with **hot reload**

Open `http://localhost:5173` in your browser. The frontend proxies `/ws` and `/auth` to the backend automatically.

### Manual startup (two separate terminals)

**Terminal 1 — backend (mock mode, no Arduino needed)**
```bash
cd nuevo_ui/backend
pip install -e . --break-system-packages   # first time only
NUEVO_MOCK=1 python3 -m nuevo_bridge
```

**Terminal 2 — frontend**
```bash
cd nuevo_ui/frontend
npm install       # first time only
npm run dev
```

### Verify it works

1. Visit `http://localhost:8000/health` — should return `{"status": "ok"}`
2. Visit `http://localhost:5173` — dashboard loads, connection badge shows green

---

## Backend options

| Variable | Default | Description |
|----------|---------|-------------|
| `NUEVO_MOCK` | `0` | `1` = simulate Arduino data (no hardware needed) |
| `NUEVO_SERIAL_PORT` | `/dev/ttyAMA0` | Serial device path to the Arduino |
| `NUEVO_SERIAL_BAUD` | `500000` | Baud rate — must match firmware `RPI_BAUD_RATE` |
| `NUEVO_ROS2` | `0` | `1` = enable ROS2 topic bridge (requires ROS2 installed) |

### Mock mode (no Arduino, works on any machine)
```bash
cd nuevo_ui/backend
NUEVO_MOCK=1 python3 -m nuevo_bridge
```

### Real Arduino
```bash
cd nuevo_ui/backend
NUEVO_SERIAL_PORT=/dev/ttyAMA0 python3 -m nuevo_bridge
```

### With ROS2
Set `NUEVO_ROS2=1` to enable ROS2 topic publishing/subscribing alongside the WebSocket bridge. The bridge is started automatically by the Docker entrypoint when running in the ROS2 container.
See [`../ros2_ws/README.md`](../ros2_ws/README.md) for Docker setup and ROS2 integration details.

---

## WebSocket API — summary

All messages are JSON. The frontend connects to `ws://<host>/ws?token=<jwt>`.

**Bridge → browser (telemetry topics):**

| Topic | Description |
|-------|-------------|
| `system_status` | Arduino uptime, loop timing, error flags, enable masks |
| `voltage` | Battery mV, 5V rail mV, servo rail mV |
| `dc_status_all` | Position, velocity, PWM, PID state for all 4 DC motors |
| `step_status_all` | State, position, speed for all 4 steppers |
| `servo_status_all` | Enable mask and pulse width for all 16 servo channels |
| `io_status` | Button states, LED brightness, NeoPixel colors |
| `imu` | Quaternion, accel/gyro/mag, calibration state |
| `kinematics` | x, y, θ, vx, vy, ωz from wheel odometry |
| `connection` | Serial port stats (rx/tx bytes, CRC errors) |

**Browser → bridge (commands):**
`dc_enable`, `dc_set_velocity`, `dc_set_pwm`, `dc_set_position`, `set_pid`,
`step_enable`, `step_move`, `step_home`, `step_set_params`,
`servo_enable`, `servo_set`,
`set_led`, `set_neopixel`,
`sys_cmd`, `mag_cal_cmd`

Full message formats and field definitions: [`docs/architecture.md`](docs/architecture.md)

---

## Further reading

- [`docs/architecture.md`](docs/architecture.md) — backend internals, message router, TLV codec, WebSocket protocol reference
- [`../ros2_ws/README.md`](../ros2_ws/README.md) — ROS2 integration and Docker setup
- [`../firmware/README.md`](../firmware/README.md) — Arduino firmware and UART protocol
