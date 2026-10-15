# Architecture — Bio-Inspired Turtle Robot Simulation

This document describes *what is actually built* in the `underwater_robot`
package: the ROS 2 node graph, the Gazebo Harmonic simulation model, the
control chain, the mission logic and every tuning value with honest provenance.
Nothing here describes hardware that does not exist yet — the hardware plan is
in [`hardware_validation.md`](hardware_validation.md).

---

## 1. System overview

```
                    simulation.launch.py
   ┌──────────────────────┬───────────────────────────┐
   │ gz sim server        │ ros_gz_bridge             │
   │ (underwater_world)   │ (config/bridge.yaml)      │
   └──────┬───────────────┴───────────┬───────────────┘
          │ gz topics                 │ ROS topics
          ▼                           ▼
   sensors────────┐             depth_pressure ─ /depth, /depth/pressure
   odometry───────┼───────────► /odometry, /imu, /navsat, /camera...
   thrusters◄─────┼───────────► /underwater_robot/thruster/<name>/cmd_thrust
                  │
   ┌──────────────┴───────────────────────────────┐
   │  ROS 2 nodes (each own process)              │
   │  actuator_backend  ← /actuator/thruster_cmd  │
   │  thruster_allocator ← /cmd_vel               │
   │  mission_manager   ← /odometry /depth        │
   │                     ← /oil/detected          │
   │  oil_detector      ← /camera/image_raw /depth│
   │  color_sensor      ← /camera/image_raw       │
   │  gps_adapter       ← /navsat                 │
   │  depth_pressure    ← /odometry               │
   └──────────────────────────────────────────────┘
```

Command authority is deliberately **one-directional and single-source**:

```
/ cmd_vel (mission_manager or teleop/test)
      │ Twist (normalised body command)
      ▼
thruster_allocator  ──►  /actuator/thruster_cmd  (Float64MultiArray, 4×N)
      │
      ▼
actuator_backend    ──►  /underwater_robot/thruster/<name>/cmd_thrust (Float64)
      │                       (only publisher allowed on these topics)
      ▼                     ros_gz_bridge ROS→GZ
gz::sim::systems::Thruster × 4  →  AddWorldWrench on the robot body
```

**`actuator_backend` is the single authority.** Mission manager, teleop, tests —
everything — goes through `/actuator/thruster_cmd`. This is what guarantees
“only one backend can command the actuators” and is the seam where a future
ESP32/ESC transport would be inserted.

---

## 2. Simulation model (Gazebo Harmonic)

### 2.1 The turtle (`urdf/final_robot.urdf`)

| Part | Purpose |
|---|---|
| `base_link` (hull, 0.30 kg) | body; buoyancy volume is the hull |
| 4 thruster housings (0.16 kg each) | mounts: front/rear **vertical**, left/right **horizontal** |
| 4 propeller links (0.002 kg each) | continuous joints, spun by the Thruster plugin |
| camera / IMU / GPS links | sensor mounts (forward camera) |
| trim ballast | mass trim so the vehicle is **slightly positive** (~0.58 kg ballast) → it surfaces slowly when unpowered |

Total modelled dry mass ≈ 1.55 kg. Hull displacement at ρ = 1000 kg/m³ ≈
1.549 kg, so the unpowered vehicle rises only a few cm/s — it never rockets to
the surface, and floating-up after mission end is expected behaviour.

### 2.2 Thrusters

Each propeller joint is driven by the native
`gz::sim::systems::Thruster` plugin:

| Parameter | Value | Meaning |
|---|---|---|
| `fluid_density` | 1000 kg/m³ | water |
| `propeller_diameter` | 0.053 m | small ROV-class prop |
| `thrust_coefficient` | 0.005 | cancels in ForceCmd mode; only sets the spin rate |
| `max_thrust_cmd` / `min_thrust_cmd` | ±2.0 N | clamp on the command |
| `velocity_control` | true | propeller joint velocity is written directly (no PID on the prop) |

In **ForceCmd mode the commanded value is a force in Newtons** — the `cmd_thrust`
from the allocator is applied directly as a wrench on the link through
`AddWorldWrench(unitVector * thrust, ...)`. `velocity_control=true` only makes
the propeller spin at the matching rate (`ω = √(|T| / (ρ cT D⁴))`); it does
**not** amplify the force. The thrust sign convention was verified empirically
with a probe sphere: positive `cmd_thrust` pushes along the thruster axis
(+x for the horizontal pair, −z for the vertical pair, i.e. **down**).

Thruster geometry in the body frame (+x forward, +y left, +z up):

```
front_vertical @ ( x=+0.062, z-axis )      rear_vertical @ ( x=-0.205, z-axis )
left_horizontal  @ ( x=-0.046, y=+0.133 )  right_horizontal @ ( x=-0.046, y=-0.133 )
```

### 2.3 Hydrodynamic drag — and why it matters

Water drag is applied by the native Fossen-style
`gz::sim::systems::Hydrodynamics` system, not by the URDF `<velocity_decay>`
element (which proved to be ~10× weaker than its documented value: during
tuning the vehicle reached ~1.2 m/s under a modest surge command and repeatedly
crashed the DART/ODE contact solver with an AABB assertion when it hit the pool
corner at speed).

Coefficients (SNAME convention, negative = damping, **provisional**):

| Coefficient | Value | Effect |
|---|---|---|
| `xU, yV, zW` | −2.0 N/(m/s) | linear drag — near rest, cruise ≈ 2 × normalised surge |
| `xUabsU, yVabsV, zWabsW` | −0.8 N/(m/s)² | quadratic drag — soft cap; full surge (4 N) cruises at ~1.2 m/s |
| `kP, mQ, nR` | −0.5 N·m·s/rad | mild turn damping |

Drag balance for surge with the horizontal pair (total force `4·s` for a
normalised surge `s`): `4·s = 2·v + 0.8·v²`. Therefore **`s = 0.20 → ≈ 0.33 m/s`
cruise**, which is the value used as `forward_speed` in the mission. At no point
in the nominal envelope does the vehicle exceed ~0.5 m/s, comfortably below the
regime where the contact solver previously asserted.

### 2.4 World (`worlds/underwater_world.sdf`)

* Pool **22 × 22 m**, floor at z = −6.5, walls, optional transparent lid at z =
  6.1 m. Interior free space ≈ 21 × 21 × 12 m.
* Buoyancy: native `gz-sim-buoyancy-system` with `fluid_level = 0` (water fills
  the tank) and density 1000 kg/m³.
* Two **oil-like patches** (dark crude-oil colour, thin discs with collision):
  `oil_patch_1 @ (6, 2)` r=0.7, `oil_patch_2 @ (−4, −5)` r=0.5.
* Two rubble obstacles at (3, −1) and (−1, 3) and a home marker at the origin.
* Spawn: (0, 0, −4 m), yaw −135°, facing the first waypoint (−6, −6) so the
  mission starts without a violent turn.
* Spherical coordinates (18 N, 72 E) so the NavSat pipe produces plausible
  coordinates; the adapter marks the fix **INVALID while submerged**.

---

## 3. Control chain

### 3.1 Thruster allocation (`allocation.py` + `thruster_allocator.py`)

A normalised body Twist `(surge x, heave z, pitch ang.y, yaw ang.z)` maps to
four Newtons (order: front_vertical, rear_vertical, left_horizontal,
right_horizontal):

```
front_vertical  = (−heave + pitch)  · max_thrust
rear_vertical   = (−heave − pitch)  · max_thrust
left_horizontal = (surge − yaw)     · max_thrust
right_horizontal= (surge + yaw)     · max_thrust
```

* `max_thrust = 2.0 N` default.
* Commands clamped to ±max_thrust.
* **Sway (linear.y) and roll (angular.x) are not actuated** and are ignored
  (with a throttled warning).
* **Slew limiter** (`max_step = 0.12 N/cycle`): thrust ramps smoothly between
  set-points, reducing propeller transients.
* **Stale guard (0.5 s):** no fresh `/cmd_vel` ⇒ all zeros.

### 3.2 Actuator backend (`actuator_backend.py`)

* Subscribes `/actuator/thruster_cmd`, re-publishes the four Float64 commands on
  the bridged topics as **the only publisher**.
* Re-clamps to ±max_thrust, own 0.5 s stale guard, zero on shutdown.
* `backend:=sim` by default. `backend:=hardware` requires `allow_hardware:=true`
  and **still stays at zero** — no motor/ESC code exists in this build.

### 3.3 Sensors

| Node | Input | Output | Notes |
|---|---|---|---|
| `depth_pressure` | `/odometry` | `/depth` (m), `/depth/pressure` (Pa) | `depth = surface_z − z_body`, hydrostatic pressure. Sim adapter, no noise model. |
| `gps_adapter` | `/navsat` | `/gps/fix` | marks `status = -1` (INVALID) deeper than 0.30 m |
| `color_sensor` | `/camera/image_raw` | `/color_sensor/rgb` | dominant colour of the centre 25 % of the image |
| `oil_detector` | `/camera/image_raw`, `/depth` | `/oil/detected`, `/oil/confidence`, CSV | see §5 |

---

## 4. Mission logic (`mission_manager.py`)

### 4.1 State machine

```
IDLE ──(/mission/start true or autostart)──► SEARCH ──(/oil/detected true)──► HOLD
                                               ▲  │                          │
                                               │  │ waypoints exhausted      │
                                               │  ▼                          ▼
                                               └─ SEARCH ◄──(hold 5 s)──────┘
                                               │
                                               ▼
                                             RETURN ──(home within radius)──► DONE
```

* **IDLE / DONE publish nothing** on `/cmd_vel` — the vehicle is left free for
  teleop/tests, and the allocator/backend time out to zero independently.
* **SEARCH** follows the lawnmower waypoint list.
* **HOLD** (5 s on oil detection) keeps depth and stops horizontal motion.
* **RETURN** navigates home, then DONE.

### 4.2 Waypoint policy (stop-turn-*then*-go)

1. While farther than `arrival_radius` the vehicle steers with a wrapped-yaw P
   controller and **bleeds surge to zero over `3 × arrival_radius`** so it
   arrives slow.
2. Inside `arrival_radius` **and** within 0.35 rad heading error the waypoint is
   claimed.
3. The vehicle then **rotates in place (surge = 0)** until it is within 0.15 rad
   of the *next* leg heading, and only then advances.

This policy is what keeps the robot from skidding through waypoints into the
pool walls at speed (the previous failure mode).

### 4.3 Depth hold

```
heave_command = clamp( depth_gain·(depth − target_depth) − depth_damp·body_vz,
                       −heave_limit, +heave_limit )
```

Non-finite depth/velocity readings are treated as unchanged/zero. The heave
command is capped to `heave_limit = 0.35` so the vehicle can never rocket
vertically.

### 4.4 Parameters (defaults, `simulation.launch.py`)

| Parameter | Default | Meaning |
|---|---|---|
| `forward_speed` | 0.20 | normalised surge during search ⇒ **≈ 0.33 m/s** cruise with current drag |
| `yaw_gain` | 0.9 | heading P gain |
| `depth_gain` / `depth_damp` | 0.8 / 2.0 | depth hold P / velocity damping |
| `heave_limit` | 0.35 | max normalised heave authority |
| `pitch_gain` | 0.5 | pitch levelling gain |
| `arrival_radius` | 1.2 m | waypoint claim + RETURN radius |
| `hold_duration` | 5.0 s | oil-hold time |
| `odom_timeout` | 1.0 s | stale-odometry ⇒ safe stop |
| `mission_timeout` | 600 s | overall mission stop |
| `rate` | 10 Hz | control rate |
| survey area | x,y ∈ [−6, 6] m, spacing 3 m | 10 lawnmower waypoints |
| `target_depth` | 4.0 m | search depth |

---

## 5. Oil detection (`oil_detector.py` + `detection.py`)

* HSV colour filter: low `(0, 100, 0)` … high `(28, 255, 150)` targets dark
  crude-oil brown/black in the forward camera.
* `oil_fraction(image)` returns the fraction of pixels inside the HSV blob in
  the **centre 25 %** of the frame (the camera peers forward-down at the water).
* Detected when `fraction ≥ min_oil_fraction = 0.05`.
* Rising edges and then every `min_log_interval = 2 s` while above threshold are
  appended to `~/ros2_ws/oil_detections.csv`
  (`wall_time_iso, wall_time_unix, stamp_sec, stamp_nanosec, confidence, depth_m`).
* The thresholds are **uncalibrated sim values**: the true patch reads
  confidence ≈ 0.5–0.9 while stray dark pixels (propeller blades, housing edges)
  stay below ≈ 0.05, hence the working margin.

---

## 6. Safety

* Single-command-authority (backend) + hard clamps + slew limiter + 0.5 s stale
  guards at two independent layers (allocator **and** backend).
* Mission safe-stops on stale odometry and on mission timeout.
* Simulation/hardware separation: no motor path exists; the “hardware” backend
  cannot arm without an explicit flag.
* Backups of the whole source tree are kept in `~/ros2_ws/.backups/` whenever a
  significant change is made.

---

## 7. Provenance & limitations (honest)

* Every numeric coefficient above is **provisional simulation tuning** chosen so
  the demo is stable and visually plausible: drag (2.0 / 0.8), thruster
  `max_thrust` (2.0 N), `thrust_coefficient` (0.005), `forward_speed` (0.20),
  depth gains (0.8/2.0), heave cap (0.35), oil thresholds. None of these come
  from a physical robot; they are re-verifiable constants in the URDF/world/launch.
* `forward_speed`, `heave` etc. are **normalised commands**, not m/s velocities.
* Sway/roll are unactuated; the mission and allocator are 4-DOF by design.
* The depth and oil sensors are simulation adapters derived from ground truth;
  no noise model is applied (`/depth` is `surface_z − z_body`, exactly).
* The ODE “AABB bounds” crash seen during development was resolved by replacing
  the weak `velocity_decay` damping with proper Hydrodynamics drag, reducing the
  commanded cruise speed, and the stop-turn-go waypoint policy. This is
  simulation-tuned; the same fix would *not* be needed on real hardware and no
  real-hardware claims are made.
* The Gazebo thruster plugin applies force on the propeller link via
  `AddWorldWrench`; the commanded unit is Newtons (ForceCmd mode). If the plugin
  mode ever changes, the allocator/backend scale factor must be re-verified.