# Bio-Inspired Turtle Robot — Marine Oil Spill Detection and Monitoring

A ROS 2 (Jazzy) + Gazebo Harmonic simulation of a small bio-inspired **turtle-shaped
underwater vehicle** that patrols an oil-contaminated test tank, detects oil-like
patches from its camera, and returns to the launch point.

> **Project status:** working simulation demo. The full loop runs end-to-end:
> spawn → depth hold (≈ 4 m) → lawnmower search → oil detection → 5 s hold → return
> home → mission complete. All unit, lint and simulation tests pass. See
> [`docs/test_report.md`](docs/test_report.md) for evidence and
> [`docs/hardware_validation.md`](docs/hardware_validation.md) for the (not yet
> built) Pi 4 / ESP32 hardware plan.

---

## What the robot does

1. **Stays at a commanded depth** (default 4 m) with a damped proportional
   depth-hold controller.
2. **Sweeps a survey area** ([−6, 6] × [−6, 6] m, lawnmower pattern, 10
   waypoints) inside a 22 × 22 × 12 m pool, using a stop-turn-*then*-go
   waypoint policy so it never skids through a waypoint into the pool walls.
3. **Detects oil-like patches** from the forward camera (HSV colour filter,
   dark crude-oil colour), logs detections to `~/ros2_ws/oil_detections.csv`,
   and **holds position for 5 s** when oil is seen.
4. **Returns home** after the sweep and reports `mission complete (N detections)`.

## Quick start

```bash
# Terminal 1: build (non-symlink; symlink install breaks entry points)
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --packages-select underwater_robot

# Terminal 2: run the autonomous mission demo
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch underwater_robot simulation.launch.py mission_autostart:=true
```

Watch in the log, or alongside with:

```bash
ros2 topic echo /odometry --field pose.pose.position
ros2 topic echo /depth
ros2 topic echo /oil/detected
tail -f ~/ros2_ws/oil_detections.csv
```

A mission takes about **5 minutes** (≈ 300 s measured: 10 waypoints + 5 s oil
hold + return; see `docs/test_report.md`). If you want to see the GUI:

```bash
ros2 launch underwater_robot simulation.launch.py mission_autostart:=true gui:=true
```

#### Seeing the robot in the Gazebo GUI

The launch starts the GUI camera at a fixed overview pose from
`config/gui.config` (passed with `gz sim -g --gui-config …`), so the pool and
robot are in view when the window opens.

If the viewport looks blue (e.g. the camera was left zoomed far out by an
earlier session) or the robot is not in frame:

* Click `underwater_robot` in the **Entity tree** (left-hand panel), then
  right-click it and choose **Follow** (or **Move to**). In Gazebo Harmonic the
  old Gazebo-Classic `F` key is **not** a follow shortcut; following is done via
  the right-click menu.
* …or drive the camera straight to the robot (no mouse needed):

```bash
# centre the view on the robot (returns true when done)
gz service -s /gui/move_to --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean \
  --timeout 3000 --req 'data: "underwater_robot"'

# keep the camera following the robot through the whole mission
gz topic -t /gui/track -m gz.msgs.CameraTrack -p \
  'track_mode: 2, follow_target: {name: "underwater_robot"}, follow_offset: {x: 2, y: -2, z: 1}'
```

(`/gui/track` and `/gui/camera/pose` are standard Gazebo transport topics, so
the follow state is visible while the GUI runs.)

### Surface demo (robot cruising just under the water surface)

For a presentation you may want the turtle to survey the pool **at the water
surface** instead of the 4 m acceptance depth. A thin wrapper launch reuses
the exact same stack with two arguments changed:

```bash
ros2 launch underwater_robot surface_demo.launch.py
```

What it does differently from `simulation.launch.py` (whose defaults are
unchanged):

| Setting | Surface demo | Acceptance (default) |
|---|---|---|
| `spawn_z` | `-0.5` (just below the surface) | `-4.0` |
| `target_depth` | `0.15` m below the surface | `4.0` m |
| `mission_autostart` | `true` | `false` |
| `gui` | `true` | `false` |

The GUI camera is put into *follow* mode automatically (via `/gui/track`, the
same command shown above). Depth convention everywhere in this package:
**depth is positive below the surface** (`depth = surface_z − z`, world
`z = −depth`), so `target_depth=0.15` holds the robot at z ≈ −0.15 m.

> **Oil detection at the surface.** The oil patches sit on the seafloor at
> z ≈ −6 m; from 0.15 m depth the camera is ~5.8 m above them, so detections
> are far weaker than in the 4 m run. Caveat: on this workstation's virtualised
> GL, the *sensor* camera renders only uniform clear frames when the viewed
> geometry is far away (≈ beyond 4–6 m), and the seafloor at 5.8 m is beyond
> that — so a surface mission may record **0 detections** simply because the
> sensor renders nothing, not because the detector range changed. This is a
> rendering-environment issue, **not** caused by the water-surface visual
> (details and A/B evidence: `docs/test_report.md` §10.4). The 4 m acceptance
> run remains the reference for oil detection (`docs/test_report.md` §4).
>
> **Water-surface visual cue.** `surface_demo_world.sdf` adds a visual-only
> marker at the water line z = 0 (sparse 3×3 grid of thin bars; no collision, no
> buoyancy) so the GUI reads "cruising just under the surface". The robot camera
> cannot be used to verify it (see §10.4); use the GUI (follow camera is enabled
> automatically).

### Manual / teleop style run (mission off)

By default the mission does **not** start (`mission_autostart:=false`): the
vehicle idles (and slowly surfaces, because its trim is slightly positive).
Drive it with plain Twist commands:

```bash
ros2 launch underwater_robot simulation.launch.py
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.8}, angular: {}}"    # forward; release to stop (safe stop)
```

Commands are **normalised** (−1…1): `x=0.8` is “80 % surge”. The allocator
scales them by `max_thrust = 2.0 N`. The relationship between a normalised surge
command and the actual cruise speed is set by the (provisional) drag model, see
`docs/architecture.md`.

## Running the tests

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon test --packages-select underwater_robot
colcon test-result --verbose
```

Result: **32 passed, 1 skipped** (the skipped one is the upstream copyright
checker). `ament_flake8` and `ament_pep257` are clean.

There are also two shell harnesses used during development (the integration one
takes ~3 minutes, the full-mission one ~9 minutes, and both drive the real
simulation):

```bash
bash /tmp/opencode/mission_test.sh        # autonomous mission under test
bash /tmp/opencode/integration_test.sh    # full-stack chain + safe-stop check
bash /tmp/opencode/full_mission_test.sh   # full mission: SEARCH -> … -> DONE
```

## Package layout

```
underwater_robot/
├── launch/simulation.launch.py   # everything: gz server, bridge, all nodes
├── launch/surface_demo.launch.py # surface cruise wrapper (world:= + gui follow)
├── urdf/final_robot.urdf         # the turtle model + thrusters + hydrodynamics
├── worlds/underwater_world.sdf   # pool, obstacles, oil patches, buoyancy
├── worlds/surface_demo_world.sdf # = acceptance world + water-surface marker
├── config/bridge.yaml            # ros_gz_bridge topic map
├── config/gui.config             # GUI camera start pose (deterministic view)
├── meshes/                        # thruster/propeller STL meshes
├── underwater_robot/
│   ├── mission_manager.py        # state machine + guidance (SEARCH/HOLD/RETURN/DONE)
│   ├── thruster_allocator.py     # /cmd_vel → 4× thrust (with slew limiter)
│   ├── actuator_backend.py       # the ONLY node that commands thrusters
│   ├── depth_pressure.py         # /depth + /depth/pressure (sim adapter)
│   ├── gps_adapter.py            # marks GPS INVALID while submerged
│   ├── color_sensor.py           # dominant colour of the camera
│   ├── oil_detector.py           # HSV oil detection + CSV logger
│   ├── scripted_test.py          # bounded scripted command test
│   ├── mission.py, allocation.py, safety.py, detection.py  # pure logic
├── test/                          # unit tests (32 pass)
├── docs/                          # architecture, hardware plan, test report
└── README.md
```

## Safety model (simulation)

* **Only `actuator_backend` commands thrusters.** Everything else goes through
  `/actuator/thruster_cmd`. A real (future) ESP32 link plugs in there and
  nowhere else.
* **Zero-command safety:** both `thruster_allocator` and `actuator_backend`
  zero their output if no fresh command arrives within **0.5 s**.
* **Hard clamps:** every thruster command is clamped to ±`max_thrust`.
* **Slew limiter:** per-thruster commands change by at most 0.12 N per cycle.
* **Mission fails safe:** on stale odometry it stops; on timeout (600 s) it
  stops and returns to DONE.
* **Simulation vs hardware is strictly separated:** the backend defaults to
  `backend:=sim`; the `hardware` backend refuses to arm unless
  `allow_hardware:=true` and even then stays at zero (no motor code exists).

## Known limitations (honest)

* All drag, buoyancy and thruster coefficients are **provisional simulation
  tuning**, not measured physical constants.
* `cmd_vel` commands are normalised thrust fractions, not m/s velocities.
* Oil detection thresholds are uncalibrated sim values.
* A **positive sighting in a single mission run is not guaranteed**: the
  camera must actually point at a patch, and the lawnmower pass geometry varies
  a little run-to-run — the 10:31 verification run, for example, completed the
  mission with 10/10 waypoints but 0 detections (see `docs/test_report.md` §9).
* **Sensor camera rendering is intermittent on this workstation's virtualised
  GL**: frames alternate between the real scene and a uniform clear colour, and
  geometry farther than ~4–6 m from the camera is not drawn at all (the sensor
  shows a plain fill). This is independent of the water-surface visual (the
  acceptance world shows the same behaviour) and limits what can be verified
  from `/camera/image_raw` — especially at the 0.15 m surface-demo depth, where
  the seafloor (~5.8 m below) never renders. Verify visuals in the GUI instead.
  Full A/B evidence: `docs/test_report.md` §10.4.
* `~/ros2_ws/oil_detections.csv` accumulated ~324 artefact rows (uniform
  clear-frame colours inside the HSV window) during the 2026-10-13 surface
  investigation; do not treat today's CSV rows as detection evidence (the
  detector's own log line `/oil/detected` remains authoritative per run).
* The vehicle has 4 thrusters: surge, heave, pitch, yaw. **Sway and roll are
  not actuated** (they are ignored by the allocator).
* After the mission completes the vehicle idles; its slightly positive trim
  makes it surface slowly. The survey area and pool keep it clear of walls.
* See `docs/architecture.md` for the full list and rationale.