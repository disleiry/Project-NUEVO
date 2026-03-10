# Oscilloscope Test Procedure — UART / ISR Timing

Firmware version: v0.8.0
Requires: `DEBUG_PINS_ENABLED 1` in `config.h`

---

## Debug Pin Map

| Pin | Label          | Signal description                              |
|-----|----------------|-------------------------------------------------|
| A7  | ENCODER_ISR    | HIGH inside any encoder interrupt               |
| A8  | STEPPER_ISR    | HIGH inside Timer3 (stepper) ISR — 10 kHz       |
| A9  | OVERRUN        | Short pulse each time `taskUART()` exceeds 10 ms budget |
| A10 | PID_LOOP       | HIGH inside Timer1 (PID) ISR — 200 Hz           |
| A11 | UART_TASK      | HIGH for entire `taskUART()` execution          |
| A12 | UART_RX        | HIGH during `processIncoming()` only            |
| A13 | UART_TX        | HIGH during `sendTelemetry()` only              |

All pins are on the Arduino Mega 2560 analog header (they operate as digital outputs).
Connect scope ground to Arduino GND.

---

## Test 1 — Baseline ISR overhead (IDLE, no host connected)

**Goal**: Confirm Timer1 and Timer3 ISRs are running at correct rates with no motors active.

**Setup**
- System in IDLE (no CMD_START sent)
- No motors enabled
- Ch1 = A10 (PID_LOOP), Ch2 = A8 (STEPPER_ISR)
- Time base: 2 ms/div, trigger Ch1 rising edge

**Expected**
- PID_LOOP pulses at exactly 200 Hz (5 ms period); pulse width ≈ 20–50 µs
- STEPPER_ISR pulses at 10 kHz (100 µs period); pulse width ≈ 2–5 µs
- Both run independently — STEPPER pulses will appear inside PID gaps

**Pass criteria**: Consistent period; no missed beats; no pulse widths growing over time.

---

## Test 2 — UART task timing (IDLE)

**Goal**: Measure how long `taskUART()` actually takes and whether ISRs inflate the measurement.

**Setup**
- System in IDLE
- Host connected (bridge running, sending heartbeats)
- Ch1 = A11 (UART_TASK), Ch2 = A12 (UART_RX), Ch3 = A13 (UART_TX)
- Time base: 2 ms/div, trigger Ch1 rising edge

**Expected**
- UART_TASK goes HIGH at 100 Hz (10 ms period)
- UART_TASK width = total `taskUART()` wall-clock time
  - In IDLE: typically 0.5–2 ms (only RX + system status at 1 Hz)
  - UART_RX width (processIncoming) ≈ 0.2–0.5 ms
  - UART_TX width (sendTelemetry) ≈ 0.2–0.5 ms in IDLE
- PID_LOOP (A10) pulses will appear *inside* UART_TASK HIGH windows — this is normal ISR preemption and does NOT represent a real overrun

**Pass criteria**: UART_TASK width < 10 ms in IDLE. No OVERRUN (A9) pulses visible.

---

## Test 3 — UART task timing (RUNNING, all telemetry active)

**Goal**: Measure worst-case `taskUART()` duration with full telemetry flood.

**Setup**
- System in RUNNING (CMD_START sent from UI)
- At least one DC motor enabled
- Ch1 = A11 (UART_TASK), Ch2 = A13 (UART_TX), Ch3 = A9 (OVERRUN)
- Time base: 5 ms/div, trigger Ch1 rising edge

**Expected**
- UART_TX (sendTelemetry) will be wider than in IDLE — DC status (50 Hz, 184 B), kinematics, IMU, IO status all active
- UART_TASK total width: 2–6 ms typical; up to ~10 ms at peak
- OVERRUN (A9) pulses: occasional short spikes are acceptable; they indicate wall-clock inflation from ISR preemption, NOT real PID degradation
- The PID_LOOP ISR (A10) continues at 200 Hz regardless of UART_TASK width — verify on a second run with Ch1=A10

**Pass criteria**: PID_LOOP (A10) uninterrupted at 200 Hz. Motors run smoothly. OVERRUN pulses should be infrequent (< 10 per 200-tick window).

---

## Test 4 — ISR preemption anatomy

**Goal**: Confirm that ISR pulses *inside* UART_TASK HIGH are preemptions, not stalls.

**Setup**
- Ch1 = A11 (UART_TASK), Ch2 = A10 (PID_LOOP)
- Time base: 1 ms/div

**Procedure**
1. Capture a UART_TASK window that appears "wide"
2. Look for PID_LOOP pulses inside it
3. Sum the PID_LOOP pulse widths visible inside the UART_TASK window
4. Subtract that sum from UART_TASK width → actual UART code time

**Expected**
- PID_LOOP ≈ 200 Hz = one pulse every 5 ms inside any 5 ms+ UART_TASK window
- If UART_TASK = 8 ms and PID_LOOP adds 3× ~50 µs pulses → real UART time ≈ 7.85 ms (still acceptable)
- OVERRUN pulse (A9) fired because wall-clock > 10 ms, but real UART work < 10 ms → false alarm

**Pass criteria**: Identify at least one ISR preemption event. Confirm actual UART code time < 10 ms even when wall-clock overrun is reported.

---

## Test 5 — Encoder ISR rate (motor spinning)

**Goal**: Verify encoder pulses arrive at expected rate for measured motor RPM.

**Setup**
- One DC motor enabled and spinning at a known velocity setpoint
- Ch1 = A7 (ENCODER_ISR)
- Time base: 1 ms/div or zoom as needed

**Expected**
- At 1440 CPR encoder (4x = 5760 counts/rev) and e.g. 50 RPM:
  - 50 RPM × 5760 counts/rev ÷ 60 s = 4800 pulses/s → 208 µs between ISR pulses
- Pulse width should be very short (< 5 µs)

**Calculation**: `period_us = 60,000,000 / (RPM × CPR × encoder_multiplier)`

**Pass criteria**: Pulse period matches calculated value within ±10%. No missing pulses or irregular bursts.

---

## Test 6 — Safety: Heartbeat timeout → ERROR transition

**Goal**: Confirm that LIVENESS_LOST in RUNNING state correctly forces ERROR and disables motors.

**Setup**
- System in RUNNING with motor enabled
- Ch1 = A11 (UART_TASK) — to observe when RX stops
- Monitor DEBUG_SERIAL output on serial terminal at 115200 baud

**Procedure**
1. Enter RUNNING state, enable a motor, observe it running
2. Disconnect or kill the host bridge (stop heartbeats)
3. Watch DEBUG_SERIAL for `[SYS]` → ERROR transition message
4. Verify motor stops and state shows ERROR in UI

**Expected timing**: ERROR state forced within `HEARTBEAT_TIMEOUT_MS` (default 2000 ms) of last received TLV.

**Pass criteria**: Motor stops ≤ 2.1 s after bridge disconnect. System enters ERROR cleanly. CMD_RESET → IDLE recovers the system.

---

## Interpreting OVERRUN Pulses (A9)

OVERRUN = `taskUART()` wall-clock time exceeded 10 ms budget. This is **not** a PID loop overrun.

| What you see | Likely cause | Action needed |
|---|---|---|
| Rare OVERRUN pulses in IDLE | Normal ISR preemption inflation | None — ignore |
| Frequent OVERRUN in RUNNING | TX buffer full (telemetry flood) or heavy I2C in Timer4 | Reduce telemetry rate or check IMU |
| OVERRUN coincides with PID_LOOP bursts | ISR preemption confirmed | None — motors are unaffected |
| UART_TX (A13) stays HIGH for many ms | `Serial.write()` blocking on full TX buffer | Check baud rate / reduce telemetry |

As of v0.8.0, OVERRUN pulses are **diagnostic only** — they no longer stop DC motors.

---

## Quick Reference — Scope Recipes

```
Q: Is the UART task too slow overall?
   Ch1=A11 (UART_TASK), timebase 5ms/div, trigger rising
   → width should be < 10 ms

Q: Which half is the bottleneck — RX or TX?
   Ch1=A12 (UART_RX), Ch2=A13 (UART_TX), timebase 2ms/div
   → wider channel = bottleneck

Q: Are ISRs preempting and inflating the UART task?
   Ch1=A11 (UART_TASK), Ch2=A10 (PID_LOOP), timebase 1ms/div
   → PID pulses inside UART_TASK window confirm preemption

Q: How often do overruns happen?
   Ch1=A9 (OVERRUN), timebase 200ms/div, trigger rising
   → count pulses; each = one 10ms-budget overrun

Q: Is the PID loop actually affected by UART slowness?
   Ch1=A10 (PID_LOOP), timebase 2ms/div
   → must be rock-solid 200 Hz regardless of UART load
```
