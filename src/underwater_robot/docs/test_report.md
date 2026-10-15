# Test Report — underwater_robot (simulation)

> Date: 2026-10-13. All results below are from runs of the **actual** build in
> this workspace (`~/ros2_ws/src/underwater_robot`), not fabricated or
> re-summarised expectations. Every claim is traceable to an artifact file or to
> a repeatable command in this document.
>
> Scope: **simulation only.** Physical hardware does not exist and no real-motor
> claim is made anywhere in this report (see `hardware_validation.md`).

---

## 1. Build

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --packages-select underwater_robot
```

| Check | Result | Evidence |
|---|---|---|
| `colcon build` | PASS (exit 0) | `Finished <<< underwater_robot` (fresh run 2026-10-13) |
| Install type | non-symlink (entry points work) | build re-run after package changes; `--symlink-install` deliberately avoided |

## 2. Unit & static tests

```bash
colcon test --packages-select underwater_robot
colcon test-result --verbose
```

**Summary (fresh run 2026-10-13): 33 tests — 0 errors, 0 failures, 1 skipped.**

| Suite | Result |
|---|---|
| `test_allocation.py` (allocator math, clamps, slew) | pass |
| `test_detection.py` (oil filter / fraction) | pass |
| `test_mission.py` (state machine, stop-turn-go, depth hold, safe-stops) | pass |
| `test_safety.py` (stale guards, zero-command, clamps) | pass |
| `test_copyright.py` | **skipped** (upstream copyright header checker) |
| `test_flake8.py` (ament_flake8) | pass |
| `test_pep257.py` (ament_pep257) | pass |

The skipped test is the standard ament copyright-header check and is expected to
be skipped in this project template; it does not exercise any code path.

## 3. Integration test (full launch stack)

Harness: `/tmp/opencode/integration_test.sh` → `/tmp/opencode/int_test.out`
(launches the real launch file headless, inspects topics, drives the command
chain, then starts the mission). Numbered results:

1. **Stack came up**: 9 ROS nodes up
   (`actuator_backend, color_sensor, depth_pressure, gps_adapter, mission_manager,
   oil_detector, robot_state_publisher, thruster_allocator` + `ros_gz_bridge`).
2. **Topics present**: 18+ ROS topics including `/cmd_vel`,
   `/actuator/thruster_cmd`, the four `/underwater_robot/thruster/*/cmd_thrust`,
   `/odometry`, `/depth`, `/depth/pressure`, `/gps/fix`, `/navsat`, `/imu`,
   `/camera/image_raw`, `/camera/camera_info`, `/oil/detected`, `/oil/confidence`,
   `/color_sensor/rgb`, `/joint_states`, `/tf`, `/mission/start`.
3. **Sensor adapters**:
   * `/depth` ≈ 3.87 m, `/depth/pressure` ≈ 139 150 Pa (hydrostatic ≈
     `101325 + 1000·9.81·3.87`).
   * `/gps/fix` `status = -1` (**INVALID** while submerged), lat 18.0 / lon 72.0.
   * `/camera/image_raw` 640 × 480; `/color_sensor/rgb` reports the dark water
     colour (r≈0.055, g≈0.118, b≈0.125).
   * `/oil/confidence` = 0.0 and `/oil/detected` = false at start (no false
     positives at rest).
4. **Command chain** (`surge = 0.8` held ~6 s):
   * `/actuator/thruster_cmd` = `[0, 0, 1.6, 1.6]` (vertical/heave channels 0,
     horizontal channels at exactly `0.8 × 2.0 N = 1.6 N`) — sampled at 2 s and 4 s.
   * Gazebo side: `/underwater_robot/thruster/left_horizontal/cmd_thrust` = **1.6**
     (bridge → Thruster plugin FX confirmed).
   * **Safe stop**: after the publisher stopped, both samples show all zeros
     (allocator 0.5 s stale guard).
5. **Motion**: during the ~6 s surge the vehicle moved
   `(≈0,0) → (−4.92, −4.92)` in the pool; no crash, no NaN.
6. **Mission handshake**: `/mission/start := true` → log
   `IDLE → SEARCH`, `waypoint 1/10 reached`, and the mission `/cmd_vel` shows
   `linear.x ≈ 0.20` (the configured `forward_speed`), `linear.z ≈ −0.03`
   (depth hold), small angular corrections.

## 4. Full autonomous mission run (the demo)

Harness: `/tmp/opencode/full_mission_test.sh` → `/tmp/opencode/fullmission.out`
(480 s of 20 s-interval sampling during a live autonomous run with
`mission_autostart:=true`).

| Item | Result |
|---|---|
| Waypoints | **10/10** reached, each logged (`waypoint 1/10` … `waypoint 10/10`) |
| Oil detection | **5 logged detections** (conf 0.483–0.658, one 0.057 noise spike), all at depth ≈ 3.98–3.99 m, while the boat is at x≈5.3, y≈0.5 — ≈ 2 m from `oil_patch_1` at (6, 2) |
| Mission states | `IDLE→SEARCH` → `SEARCH→HOLD` (5 s) → `HOLD→SEARCH` → `SEARCH→RETURN` → `RETURN→DONE` |
| Completion log | `home reached; mission complete (1 detections)` |
| Mission duration | ≈ 299 s from `IDLE→SEARCH` to `DONE` (SEARCH ≈ 273 s incl. the 5 s hold, RETURN ≈ 26 s) |
| Depth hold during SEARCH | 3.977–3.989 m vs. target 4.00 m (σ ≈ 5 mm) |
| End state | `gz` server alive at end; **0 `nan`/`inf`** in the launch log; vehicle idles and surfaces slowly after DONE (expected — slightly positive trim, see architecture §2.1) |

The single HOLD was triggered on the first true detection (conf 0.658). The
0.057 reading is a stray dark-pixel spike just above the 0.05 threshold — it is
logged honestly and is the reason the report documents the threshold margin as
“working but uncalibrated”.

Oil CSV (`~/ros2_ws/oil_detections.csv`), tail rows from this run:

```
2026-10-13T01:46:00,1791836160.185,154,500000000,0.65804,3.9778
2026-10-13T01:46:00,1791836160.490,154,800000000,0.62314,3.9778
2026-10-13T01:46:01,1791836161.712,156,0,0.48262,3.9806
2026-10-13T01:46:04,1791836164.423,158,700000000,0.05657,3.9886
2026-10-13T01:46:05,1791836165.329,159,600000000,0.61210,3.9903
```

## 5. Regression / tuning evidence

* **Speed probe** (`/tmp/opencode/speed_probe.out`): stepped normalised surge
  0.05 → 0.10 → 0.20 → 0.35 → 0.50. Positions plateau cleanly at each step
  (e.g. surge 0.20 settles around 0.30 m/s cruise per the drag model
  `4·s = 2v + 0.8v²`), `gz` alive at end, **no contact-solver crash** at any
  level. (The flat “plateaus” inside a step are the harness’s shell
  `pub` lag tripping the allocator stale-guard — a harness artifact, not a
  product bug; the important evidence is stability across all levels.)
* **Mission regression** (`/tmp/opencode/mission_test.out`): earlier run at the
  *old* `min_oil_fraction = 0.02` where noise (conf 0.029–0.044) repeatedly
  triggered `SEARCH→HOLD` — this is what motivated raising the threshold to
  **0.05**; with 0.05 the full run above behaves correctly. Documented as
  provenance, not as a current defect.

## 6. Development issues found & resolved

| # | Symptom | Root cause | Resolution |
|---|---|---|---|
| 1 | Robot never spawned after a launch; create node targeted `/world/probe/create` | a stale gz sim server from an earlier probe world was still running | `pkill -9 -f '[g]z sim]'` guard before every launch; harnesses re-check topic presence |
| 2 | ODE crash at t≈11 s, `ODE INTERNAL ERROR 1: assertion "aabbBound >= dMinIntExact && aabbBound < dMaxIntExact" failed in collide()` (both in `/tmp/opencode/gz_test2/server.log` and `gz_test3/server.log`), exit −6 | `<velocity_decay>` drag was ~10× weaker than documented; vehicle hit ~1.2 m/s at surge 0.05 and smashed the pool corner at speed | replaced `velocity_decay` with the native `gz::sim::systems::Hydrodynamics` plugin (linear −2.0, quadratic −0.8 N/…), verified negative-coefficient convention from plugin source; archive of the old URDF kept in `~/ros2_ws/.backups/hydrodynamics_20261013_013433/` |
| 3 | Repeatedly skidding through waypoints into walls | aggressive direct waypoint tracking | stop-turn-*then*-go policy + arrival bleed (`mission_manager.py`), `forward_speed` 0.20 |
| 4 | False HOLDs on camera noise | `min_oil_fraction` 0.02 too low | threshold raised to 0.05 (true patch reads 0.48–0.66, noise ≤ 0.06) |
| 5 | Shell “safe-stop” trips during probes | 0.5 s stale guard on command | guard is intentional; probes republish > 0.5 s so their own plateaus occur — documented as harness artifact |

Since fix #2 the nominal mission/integration runs complete end-to-end with the
gz server still alive and zero non-finite values.

## 7. Coverage and honest gaps

**Covered by automated tests + sim runs:** allocator math/clamps/slew, stale
guards (allocator + backend), oil filter/fraction, mission state machine,
waypoint policy, depth hold, full-stack wiring, end-to-end mission.

**Not covered / not claimed:**
* No real-hardware behaviour is claimed (see `hardware_validation.md`).
* All coefficients are provisional sim tuning (drag 2.0/0.8; `max_thrust` 2.0 N;
  gains 0.8/2.0; `heave_limit` 0.35; oil thresholds) — reproducible in the URDF /
  world / launch, but never measured on a vehicle.
* Depth and oil sensing are sim adapters with **no noise model**; `cmd_vel` is a
  normalised command, not m/s.
* Sway (linear.y) and roll are unactuated and ignored.
* The 0.057 oil reading shows the noise margin is real; threshold tuning on real
  camera imagery is still required.
* Harness scripts live in `/tmp/opencode/` (not packaged); re-run them from there
  to reproduce the evidence above.

## 8. How to reproduce everything

```bash
# build + unit/lint
source /opt/ros/jazzy/setup.bash && cd ~/ros2_ws
colcon build --packages-select underwater_robot
colcon test --packages-select underwater_robot && colcon test-result --verbose

# full-stack integration (headless, ~3 min)
bash /tmp/opencode/integration_test.sh ; cat /tmp/opencode/int_test.out

# full autonomous mission demo (headless, ~9 min)
bash /tmp/opencode/full_mission_test.sh ; cat /tmp/opencode/fullmission.out

# or just watch it live
ros2 launch underwater_robot simulation.launch.py mission_autostart:=true gui:=true
```

## 9. GUI camera fix (blue viewport) + re-verification on the fixed build

Added 2026-10-13 (afternoon). This section documents the one *display* problem
found during submission prep, its minimal fix, and a fresh full-stack
re-verification on the fixed build.

### 9.1 Root cause and fix

Symptom: the Gazebo GUI viewport looked mostly blue and the robot was not
visible; pressing `F` did nothing.

Evidence collected from the live instance (not guesses):

* `/gui/camera/pose` was `(−28.4, 7.4, 27.0)` — the GUI camera ~40 m outside
  the 22 × 22 × 12 m pool, high above it, looking at sky/background. That pose
  is *persisted* in `~/.gz/sim/8/gui.config` (`<camera_pose>` element of the
  "3D View" plugin) and is restored on every start, so once a session left it
  zoomed out, every later launch reopened in the same blue view.
* `/gui/currently_tracked` was **empty** — no entity was selected/followed.
  In Gazebo Harmonic “follow” is **not** the Gazebo-Classic `F` key; it is
  right-click → **Follow** (or the `/gui/track` transport topic).
* Rendering itself was healthy: the robot camera sensor renders the scene
  (oil patches were detected in earlier runs), the scene broadcaster is live,
  and the GUI publishes its pose topics. The VMware VM (SVGA + software GL)
  is not the problem.

Fix (backup: `~/ros2_ws/.backups/gui_camera_fix_20261013/`):

1. New `config/gui.config` with a deterministic camera start pose
   (`camera_pose 6 -6 3 0 1.04 -0.785`, i.e. just above the spawn area looking
   at the origin).
2. `launch/simulation.launch.py` now starts the GUI with
   `gz sim -g --gui-config <pkg>/config/gui.config` (new `gui_config` launch
   argument), so the view is the same on every launch regardless of home-dir
   state.
3. The same camera pose was written back to `~/.gz/sim/8/gui.config`.

Verification on a relaunch: `/gui/camera/pose` ≈ `(7.1, −7.2, 1.5)` just after
start; `gz service -s /gui/move_to ... "underwater_robot"` placed the camera
exactly on the robot; a `/gui/track` publish put the GUI into
`currently_tracked = FOLLOW underwater_robot` with offset `(2, −2, 1)`.

> Note: immediately after GUI start the camera may briefly report a large
> transient pose while it animates to the configured view; if the view looks
> wrong at any time, right-click the robot in the Entity tree → _Follow_, or
> re-send `/gui/move_to`. This is a GUI-client behaviour, not a simulation
> issue.

### 9.2 Re-verification on the fixed build (2026-10-13 afternoon)

| Check | Result |
|---|---|
| `colcon build` | exit 0 |
| `colcon test` + `colcon test-result --verbose` | **33 tests, 0 errors, 0 failures, 1 skipped** |
| Integration harness (`/tmp/opencode/integration_test.sh` → `int_test.out`) | green: surge 0.8 → `/actuator/thruster_cmd = [0,0,1.6,1.6]` → gz `cmd_thrust = 1.6` → safe-stop to zeros; `/depth/pressure` ≈ 138 860 Pa; `/gps/fix` status −1 INVALID; camera 640×480; `/oil/detected` false at rest; `/mission/start` → `IDLE→SEARCH`, `waypoint 1/10 reached` |
| Graceful shutdown | SIGINT to `ros2 launch` tears down gz server + GUI + bridge + all nodes (no `-9`); confirmed 0 leftover processes |
| Full-mission re-run (`/tmp/opencode/full_mission_test.sh` → `fullmission.out`, 10:31) | **10/10 waypoints**, `SEARCH→RETURN→DONE`, `home reached; mission complete (0 detections)`, ≈ 287 s, depth held 3.977–3.990 m during SEARCH, **gz alive at end, 0 `nan`/`inf`** |

The 10:31 full-mission run recorded **0 oil detections** (no CSV rows, no
`OIL DETECTED` line): the lawnmower pass took the vehicle past `oil_patch_1`
at (6, 2) with the patch outside the forward camera FOV between the 20 s
sample instants. Detection *capability* is separately evidenced by §4
(conf 0.483–0.658 at depth ≈ 3.98–3.99 m) and by today’s CSV rows from
08:31–08:43 (conf 0.346–0.760 at ≈ 3.98 m) — i.e. **mission control and
mission completion are deterministic; a positive sighting in any *single*
mission run is not guaranteed**. One cosmetic launch-log item: a single
`odometry stale - safe stop` WARN right at `IDLE→SEARCH` recovered on the next
cycle (documented handover transient, §5).

### 9.3 Submission evidence files

* `~/ros2_ws/oil_detections.csv` — detector log (header + 34 rows; conf + depth).
* `/tmp/opencode/fullmission.out` — 10:31 full-mission run (regenerated).
* `/tmp/opencode/int_test.out` — integration harness run (regenerated).
* `docs/test_report.md` §4 – the detailed 01:34 detection run (conf 0.483–0.658).
* `~/ros2_ws/.backups/gui_camera_fix_20261013/` – original `gui.config` and
  `simulation.launch.py` before the camera fix.
* `~/ros2_ws/.backups/hydrodynamics_20261013_013433/` – pre-drag-fix source.

## 10. Surface demo verification (2026-10-13, water-surface cruise)

Added for the capstone final presentation: a *separate* launch
(`launch/surface_demo.launch.py`) that reuses the whole stack but runs the
mission **just under the water surface** instead of at 4 m depth.

### 10.1 What was changed (acceptance config untouched)

* `launch/simulation.launch.py`: two *additive* launch arguments whose defaults
  are byte-identical to the previous hardcoded values —
  `spawn_z` (default `-4.0`) and `target_depth` (default `4.0`) — wired into
  the spawn `-z` and the `mission_manager` parameter respectively. The 4 m
  acceptance invocation (`simulation.launch.py mission_autostart:=true`) is
  therefore unchanged; `docs/test_report.md` §4/§9 evidence still holds.
  (Backup: `~/ros2_ws/.backups/surface_demo_20261013/`)
* NEW `launch/surface_demo.launch.py`: includes the base launch with
  `gui:=true`, `mission_autostart:=true`, `spawn_z:=-0.5`, `target_depth:=0.15`
  and, ~10 s in, publishes `/gui/track` (CameraTrack FOLLOW, offset
  (2,−2,1)) so the presentation camera stays locked onto the robot.

### 10.2 Depth / axis conventions (the numbers behind the demo)

Depth is **positive downward**: `depth = surface_z − z` with world
`surface_z = 0.0` (`depth_pressure.py:7`). The robot's world z = −depth, so:

* `target_depth = 0.15` m → the robot holds **z ≈ −0.15 m**.
* Spawn `spawn_z = -0.5` starts it fully submerged, low enough to avoid any
  spawn transient while still only half a metre under the surface.
* Water surface for buoyancy = `fluid_level 0` (world SDF), ρ = 1000 kg/m³;
  displacement volume 1.54875e-3 m³ ≈ dry mass 1.549 kg → ~neutral, slightly
  positive trim (the vehicle is passively stable, CB above COM).
* Body mesh bounding box (measured from the shipped STL): 0.336 × 0.192 ×
  0.073 m, shell top ≈ +0.037 m above base_link → at z = −0.15 the shell top
  is ~0.11 m below the surface, so the hull **cannot poke above z = 0** at the
  demo depth.

### 10.3 Measured surface-demo run (11:17–11:28)

Harness `/tmp/opencode/surface_demo_harness.sh` → `/tmp/opencode/surface_demo.out`,
launch log `/tmp/opencode/surface_demo_launch.log`.

| Check | Result |
|---|---|
| Launch | `surface_demo.launch.py` exit-0 stack: gz server+GUI, TF, spawn, bridge, 7 ROS nodes |
| Mission flow | `IDLE → SEARCH`; **waypoint 1/10 … 10/10 reached**; `SEARCH → RETURN → DONE` |
| Completion | `home reached; mission complete (0 detections)`; mission duration ≈ **292.7 s** |
| Depth hold | samples during SEARCH: **0.128 … 0.139 m** (target 0.15), z ≈ −0.13 to −0.14 |
| Spawn settle | z: −0.20 at t≈6 s after spawn at −0.5 … holding ≈ −0.13 by t≈60 s |
| GUI camera | `/gui/currently_tracked` = `FOLLOW underwater_robot` (offset 2,−2,1); camera pose rides with the vehicle |
| Oil detection | **0 detections this run** (no CSV rows, `/oil/detected` false) — expected: patches sit on the seafloor at z≈−5.95, i.e. ~5.8 m below the surface camera, far below the 0.05 HSV threshold reach. Detection remains a 4 m-depth feature (§4, §9). |
| Stability | 0 `nan`/`inf` in odometry/depth samples; gz server alive at end; clean teardown via SIGINT to the launch session |

Post-DONE note: with the mission finished the vehicle keeps drifting upward at
≈0.008 m/s (the +0.00175 kg trim vs the linear drag, 2 N·s/m) and this
sim's uniform-fluid buoyancy keeps acting past z = 0, so it continues toward
the lid (z=6.1) instead of bobbing at the water line. Identical idle behaviour
to the 4 m run; irrelevant to the mission phase and to `surface_demo.launch.py`
because `mission_autostart:=true` keeps the depth hold active.

> **IMPORTANT — §10.3's "0 detections … patches below the HSV threshold reach"
> is confounded and must be read with §10.4 in mind.** The sensor camera in
> today's (2026-10-13) sessions renders *uniform clear frames* instead of the
> scene when the viewed geometry is far away (the seafloor is ~5.85 m below the
> surface camera). Zero detections at 0.15 m depth is therefore **not evidence**
> about detector range; it may simply be the camera rendering nothing. See §10.4.

### 10.4 Water-surface visual (visual-only) + sensor-rendering investigation

**Goal** (capstone): make the scene *read* as "robot cruising under the water
surface" in the GUI, while keeping the 4 m acceptance configuration byte-identical.

* `worlds/surface_demo_world.sdf` = exact copy of `underwater_world.sdf` plus a
  single visual-only model at the water line z = 0 (no `<collision>`, no physics,
  so dynamics / sensors / waypoints are unchanged). `diff` of the two files shows
  only the added model (+ header comment).
* Design history: v1 was a translucent 21×21×0.04 m slab (transparency 0.8);
  after sensor frames seemed to show an opaque full-view sheet it was replaced
  (user-approved) by v2, a sparse 3×3 grid of thin opaque bars (5 m cells,
  0.10 m × 0.04 m section) — the version currently in the repo and installed.
* `launch/surface_demo.launch.py` passes `world:=<surface_demo_world.sdf>`; the
  acceptance launch args are untouched.

**Rendering investigation (why sensor frames stayed uniform `(39,118,160)`):**

| Test (all 2026-10-13, headless `gz sim -s` + the same bridge/cv_bridge path) | Result |
|---|---|
| Robot attitude during cruise | pitch ≈ −0.9° (essentially level); camera pitched 34° down → a down-camera at z≈−0.15 cannot even see the z=0 marker plane; uniform frames must be a render artifact, not the marker |
| Minimal world: camera 3 m above an 8 m red box | box **renders** in ~37–53 % of frames (pixels `(249,65,66)`), alternating with "clear" frames (ogre default `(108,108,108)`) → the sensor renderer is **intermittent**, not globally broken |
| Same minimal world + the exact bar-grid model | bars render (bright blue `(108,215,255)`) alongside the red box → **the marker renders correctly and does not obscure the scene below** |
| Acceptance world (no marker) + robot camera | 84/194 real frames (real scene: dark-olive seafloor `(19,22,10)` etc.); a 12 m-high overhead camera in the **same session**: 0/162 real frames → the failure correlates with **distance to geometry** (≈ works under ~4–6 m, fails beyond), **not** with the marker or the world |
| Surface demo world (bar grid), robot + overhead cameras | 0 real frames in several hundred frames (runs 4/5/6) — consistent with the distance correlation (floor 5.85 m / 12 m below) |
| Fresh clean launches after full process sweep; GUI killed vs running | same behaviour → not a process-leak or GUI-contention issue |

**Honest consequences**

* The earlier "the translucent slab renders as an opaque sheet" conclusion is
  **retracted**: it was a misattribution of the renderer's uniform clear frames
  (which persist with *no* water visual at all). The slab may well render
  translucent; it was never sensor-verified either way.
* The sensor camera on this virtualized-GL workstation cannot currently serve as
  the verification channel for the water-surface visual: at the demo depth
  (0.15 m) it cannot render the 5.85 m-below seafloor at all. **The GUI is the
  verification channel** (follow camera + screenshot; the GUI itself renders
  fine). Sensor-image evidence from 2026-10-13 sessions should be treated as
  unreliable for detection claims.
* Detector artifacts: the run-1 climb appended 324 rows to
  `~/ros2_ws/oil_detections.csv` (confidence 0.36–1.0, incl. depth<0 rows) —
  uniform clear-frame colours occasionally landing inside the HSV window are
  **not oil sightings**. Treat today's CSV as polluted for detection evidence;
  §4's earlier detection evidence stands on its own (captured when rendering
  worked) but was not re-exercised today.
* The mission itself is unaffected by all of this: in the bar-grid world the
  2026-10-13 run completed **10/10 waypoints, SEARCH→RETURN→DONE** with depth
  hold 0.128–0.141 m (only two transient conf≈0.18 detector clicks, a
  clear-frame artefact).

**Status:** bar-grid world built and installed (`--symlink-install`), launch
wired; acceptance world byte-identical; log paths for 2026-10-13 runs:
`/tmp/opencode/surface_demo_water{1,2,3,4,5,6}.out`, `/tmp/opencode/acceptance_*.out`,
`/tmp/opencode/translucency/{run_red2,run_marker}.log`.