# Hardware Plan & Validation Checklist (Pi 4 / ESP32 / ESP32-CAM)

> **Status: HARDWARE IS NOT BUILT.** Nothing in this repository has run on a real
> vehicle. This document is the *plan* for the future physical build and the
> validation checklist that the hardware must pass *before* it is ever allowed to
> command a propeller. The simulation runs completely independently of every item
> below, and the current code has **no motor/ESC path** — the “hardware” backend
> stays at zero by design.

---

## 1. Why this split

The design keeps **two domains** strictly apart:

| Domain | Responsibility | Compute |
|---|---|---|
| **Mission / planning** | Waypoint survey, oil-decision, CSV logging, operator UI, camera processing | Raspberry Pi 4 (ROS 2) |
| **Actuation / interfacing** | Reading thruster commands, safe-stop policing, spinning motors, reading depth/prop RPM | ESP32 (real-time, close to the motors) |
| **Vision capture** | Forward camera stream for oil detection | ESP32-CAM |

The reason for the split is safety + determinism: ROS 2 and Linux on the Pi 4 are
convenient but not hard-real-time. The ESP32 is the *last line of defence* that
applies the same zero-command rule as the actuator backend **in firmware**, next
to the ESCs, independent of whether the Pi 4 is alive, frozen, or rebooted.

Nothing about this split is invented hardware: it is the existing node
architecture (see `architecture.md`) mapped onto processors. The seam is exactly
the `actuator_backend` node — the only node allowed to publish thruster commands.

```
  quest /cmd_vel → allocator → /actuator/thruster_cmd
                                         │  (Float64MultiArray, 4 × N)
                                         ▼
                              actuator_backend  (sim OR hardware mode)
                                         │
            ┌────────────────────────────┴────────────────────────────┐
            │ sim (now)                        │ hardware (future)      │
            │ ros_gz_bridge → gz Thruster ✕4  │ UART ─► ESP32 ─► 4 ESCs │
```

## 2. Role split (functional; no pinouts invented)

This section deliberately assigns **roles, not GPIO pins**. Pins must come from
the real schematic / datasheet of the boards actually bought, and must be
recorded in this document before any wiring happens. **No pinout is specified
here** to avoid fabricating hardware detail that does not exist yet.

### 2.1 Raspberry Pi 4 — mission brain

* Runs Ubuntu + ROS 2 (Jazzy, matching the sim) with the existing nodes:
  `mission_manager`, `thruster_allocator`, `actuator_backend (backend:=hardware)`,
  `oil_detector`, `depth`, `gps`, `color_sensor`.
* Consumes the camera stream; runs the same HSV oil filter as the sim.
* Logs detections to CSV, same schema as `~/ros2_ws/oil_detections.csv`.
* Talks to the ESP32 over a **serial link** (UART; future PoE/USB alternative
  recorded once chosen). Message format: the same 4 × Float64 thruster commands
  re-encoded per the ESP32 protocol document (TBD in firmware bring-up §4).

### 2.2 ESP32 — thruster controller / safety watch

* Receives the 4 thruster set-points from the Pi 4.
* **Own stale-guard in firmware**: if no fresh command within the configured
  timeout (default 0.5 s, same as the node), all PWM outputs go to the
  **“safe” (motor-off / zero-thrust) level** — independent of the Pi 4.
* Own ±max_thrust clamp and, if implemented, its own slew limiter, mirroring
  `allocation.py`.
* Drives 4 ESCs (one per thruster) with standard servo-style control signals at
  the ESC's documented pulse range (datasheet value; applied in the future motor
  driver stage, not in this repo yet).
* Optional: reads an I²C pressure/depth sensor and a current sensor on the
  battery rail, feeding `/depth` and `/battery` back to the Pi 4.

### 2.3 ESP32-CAM — vision front-end

* Frames the forward camera and streams to the Pi 4 over the same network
  (Wi-Fi or ESP-NOW/SPI — transport chosen in the hardware bring-up phase).
* Pi 4 runs `oil_detector` on those frames exactly as in the sim (downsampled,
  centre-25 % HSV fraction).

## 3. What is already verifiable NOW (no hardware)

The following parts of the chain are fully validated in simulation and their
validation carries over to hardware *as unit tests*, but **not** as physical
proof:

* Thruster allocation math, slew limiting, clamping (unit tests).
* Zero-command safety at the allocator and backend (unit + integration tests).
* Mission state machine, stop-turn-go waypoint policy, depth hold (mission run).
* Oil detection, CSV logging, HOLD behaviour (mission run).
* The `backend:=hardware` gate: see the motor-disconnected test in §4.

## 4. Motor-disconnected test mode (the current behaviour)

The repo **already implements** a motor-disconnected validation of the hardware
gate — there is no hardware and no motor code, and the code refuses on purpose:

```bash
# Attempt to arm the hardware backend WITHOUT the flag  → refuses, stays at zero.
ros2 run underwater_robot actuator_backend --ros-args \
  -p backend:=hardware
# Logs: "backend='hardware' requested but allow_hardware is false:
#        REFUSING to command actuators (staying at zero)."

# With the flag  → still stays at zero (no transport implemented).
ros2 run underwater_robot actuator_backend --ros-args \
  -p backend:=hardware -p allow_hardware:=true
# Logs: "no serial/ESC transport is implemented in this build: staying at zero."
```

Verify with (thrusters remain all `0.0`):

```bash
ros2 topic echo /underwater_robot/thruster/left_horizontal/cmd_thrust
```

**Checklist for this mode (all must be green before shipping the software half):**

- [ ] `backend:=sim` starts `armed=True` (default launch; confirmed in integration test).
- [ ] `backend:=hardware` without `allow_hardware` logs REFUSING and stays at zero.
- [ ] `backend:=hardware allow_hardware:=true` logs “no serial/ESC transport” and stays at zero.
- [ ] Unknown backend name logs an error and stays at zero.
- [ ] In every `failed-to-arm` case all four `/underwater_robot/thruster/*/cmd_thrust`
      topics are exactly `0.0` under a continued stream of non-zero commands.
- [ ] Sim runs never start the hardware backend (launch defaults to `backend:=sim`).

## 5. Validation checklist for the FUTURE physical build

To be executed in this exact order, each stage gated on the previous one. Every
step requires human eyes on the components; none of it is automated yet.

### 5.1 Dry bench (no propellers fitted)

- [ ] ESCs + motors connected per the bought boards' datasheet wiring; **no pinout
      assumed from this document** — pins recorded in §2 from the actual schematics.
- [ ] Propellers **removed**. Motor direction test: each motor spins the correct
      sense for its thruster (see `architecture.md` table: front/rear vertical
      push −z = down; left/right horizontal push ±x).
- [ ] ESP32 firmware applies zero-PWM at start-up before any command is received.
- [ ] With the Pi 4 powered off, ESP32 keeps all ESCs at zero (watchdog / no-frame
      behaviour verified with a logic analyser or the ESC status LED).
- [ ] Cross-check: Pi 4 `backend:=hardware allow_hardware:=true` publishes non-zero
      commands over UART; ESP32 responds; motors spin **slowly** per protocol.
- [ ] Kill the Pi 4 mid-command (`kill -9` or power cut): all motors stop within
      the stale timeout (≤ 0.5 s + one control period), verified by encoder/logic
      analyser, not by eye.

### 5.2 Pool bench (vehicle in a small test tank, tethered)

- [ ] Depth sensor vs. ruler: `/depth` matches the measured submergence within the
      sensor tolerance recorded in its datasheet.
- [ ] Nominal surge: normalised `0.20` should give a slow cruise; measure with a
      stopwatch over a marked distance and compare with §5.3 below.
- [ ] Depth hold at 0.3–0.5 m below surface oscillated depth ≤ ±0.15 m.
- [ ] Emergency stop: physical kill switch (add to the ESP32 firmware design)
      immediately forces zero thrust regardless of ROS state.
- [ ] HOLD behaviour on a bilge-water test patch: robot stops forward motion for
      the configured `hold_duration`.

### 5.3 Re-tuning note

Every coefficient marked “provisional” in `architecture.md` (drag 2.0/0.8,
max_thrust 2.0 N, depth gains 0.8/2.0, heave_limit 0.35, oil thresholds) is a
simulation value. On real hardware the controller gains and the oil thresholds
must be re-tuned from measured responses (e.g. fit the surge drag model from a
coast-down test: `4·s = c₁·v + c₂·v²` gives the real `c₁, c₂`). **Do not carry
sim coefficients to the physical vehicle.**

## 6. Explicit non-goals / honest limits

- No motor driver, ESC configuration, wiring diagram, or pinout exists in this
  repo — none has been fabricated.
- No waterproofing, battery or thermal analysis has been done.
- The ESP32 firmware, the serial protocol document, and the ESP32-CAM streaming
  link are **design targets**, not deliverables of this project.
- Simulation results (this repo) demonstrate the *control + perception
  algorithms*; they are not evidence of physical system performance.