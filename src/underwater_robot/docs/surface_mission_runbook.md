# Surface mission runbook (capstone demo)

Bio-inspired turtle robot, using the authoritative workspace model
`urdf/final_robot.urdf` (body mesh `final_body.stl` scaled 0.001, plus the
current thruster-housing and propeller meshes) simulated **on the water
surface** at `z = 0.0` in a dedicated world
(`worlds/surface_final_world.sdf`), running an autonomous oil-spill search and
return mission.

## Launch

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# 1) Normal machine (working GPU): Gazebo GUI + mission
ros2 launch underwater_robot surface_mission.launch.py

# 2) This VM (Gazebo GUI renders flat/empty due to the VMware SVGA3D / OGRE2
#    interaction): headless sim + RViz 3D view of the mission
ros2 launch underwater_robot surface_mission.launch.py gui:=false rviz:=true
```

Mission parameters (already the launch defaults): lawnmower survey over
`[-4,4] x [-4,4]`, spacing 2.0 -> 10 waypoints, cruise 0.45, depth hold at
`target_depth = 0.0` (waterline).

## What to see

* Gazebo GUI (`gui:=true`): robot on the water surface, translucent blue water
  plane, 3 dark oil slicks on the route, home marker + mast, camera follows the
  robot.
* RViz view (`rviz:=true`): the full turtle URDF model (RobotModel display),
  white survey route + 10 waypoints, green home disc, 3 dark oil discs. The
  disc under the robot flashes **bright yellow -> red** the moment a detection
  fires.
* Console: per-waypoint progress, `detection #N - holding position`, and a
  final `home reached; mission complete (3 detections)` / `RETURN -> DONE`.
* Evidence outputs:
  * `~/ros2_ws/surface_oil_detections.csv`  (3 rows, one per patch)
  * `~/ros2_ws/surface_demo_frames/`        (overview-camera PNGs)

## Detection method (disclosed)

`SIMULATED_GROUND_TRUTH` (`surface_oil_trigger.py`): the robot's forward camera
is pitched down toward the seafloor and cannot image surface slicks in this
setup, so detection is a geofence test against the known patch positions using
simulated odometry. It is not a camera/ML detection; the log and CSV both tag
every event `method=SIMULATED_GROUND_TRUTH`.

## Physics simplification (disclosed)

The world runs with **zero gravity** and no buoyancy plugin. The robot holds
the surface at `z = 0.0` using its own vertical thrusters through the existing
depth-hold controller. This is an assisted/simplified surface hold for
demonstration, not a buoyancy model.

## Known limitation on this machine

OGRE2 (Gazebo render engine) renders any scene as a flat clear colour on this
VM's virtual GPU (SVGA3D), so the Gazebo GUI and camera images come out flat
here even though physics, autonomy and mission logic run correctly. The
RViz view (`rviz:=true`) uses a different (compatibility-profile) GL path and
renders correctly. On a machine with a working GPU, `gui:=true` shows the full
scene.

## Notes

* The underwater acceptance world (`underwater_world.sdf`) and the original
  `final_robot.urdf` are untouched by the mission launch. A deprecated derived
  `urdf/surface_robot.urdf` (which swapped the body visual for the older
  `Assembly-Of-Robo-(1).stl` turtle assembly) is no longer referenced by any
  launch file and is kept on disk only for history.
* `surface_rviz_markers.py` visualises the route/patch layout; the flash is
  driven by the same `/oil/detected` signal that writes the CSV.
* RViz evidence screenshots (robot model + mission) are saved under
  `~/ros2_ws/surface_evidence/` (`rviz_best.png`, `rviz_official.png`,
  `rviz_zoom.png`, `final_verify3_run.log`).