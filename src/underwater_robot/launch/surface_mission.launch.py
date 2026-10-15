"""
CAPSTONE SURFACE DEMONSTRATION launch file.

Brings up the bio-inspired turtle robot (urdf/final_robot.urdf - the
authoritative robot model in this workspace, body mesh final_body.stl plus the
current thruster-housing/propeller meshes) ON the water surface of
worlds/surface_final_world.sdf and runs an autonomous oil-spill search mission
with a simulated ground-truth detection trigger.

Nothing here touches the underwater acceptance configuration: it is a separate
world, a separate set of launch parameters, and a separate launch file.

Quick use::

    source /opt/ros/jazzy/setup.bash
    source ~/ros2_ws/install/setup.bash
    ros2 launch underwater_robot surface_mission.launch.py

Arguments::

    gui:=true          start the Gazebo GUI (single server+GUI process)
    autostart:=true    start the mission immediately (no teleop)

Surface-demo physics (see the world header for detail): gravity is disabled and
the hull is held exactly at the waterline z = 0.0 by its own vertical thrusters
through the existing depth-hold controller (target_depth = 0.0).  This is a
documented simplified/assisted surface hold, NOT a buoyancy model.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('underwater_robot')

    world = os.path.join(pkg_share, 'worlds', 'surface_final_world.sdf')
    urdf = os.path.join(pkg_share, 'urdf', 'final_robot.urdf')
    bridge_cfg = os.path.join(pkg_share, 'config', 'bridge.yaml')
    gui_cfg = os.path.join(pkg_share, 'config', 'gui_surface.config')

    share_parent = os.path.dirname(pkg_share.rstrip('/'))
    resource_path = os.pathsep.join(
        [share_parent, os.path.expanduser('~/ros2_ws/src'), '/opt/ros/jazzy/share']
    )

    args = [
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('spawn_delay', default_value='5.0'),
        # Start RViz (headless-friendly 3D view of the mission).  On machines
        # where the Gazebo GUI renders flat/empty (e.g. some VM virtual GPUs),
        # use:  gui:=false rviz:=true
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(pkg_share, 'config', 'rviz_surface.rviz')),
    ]
    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    spawn_delay = LaunchConfiguration('spawn_delay')

    # Single process runs BOTH server and GUI so the GUI always has the scene
    # (avoids the "empty/blue viewport" of a detached `gz sim -g` client).
    gz_both = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '--gui-config', gui_cfg, world],
        output='screen',
        condition=IfCondition(LaunchConfiguration('gui')),
    )
    gz_headless = ExecuteProcess(
        cmd=['gz', 'sim', '-s', '-r', world],
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('gui')),
    )

    robot_description = ParameterValue(
        Command(['xacro ', urdf]), value_type=str
    )
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': use_sim_time}],
    )

    # Spawn at the waterline (z = 0.0), facing the first waypoint (-4, -4).
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-file', urdf,
            '-name', 'underwater_robot',
            '-x', '0', '-y', '0', '-z', '0.0',
            '-Y', '-2.356194',
        ],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{'config_file': bridge_cfg}],
    )

    # Second bridge mapping for the world overview (evidence) camera.
    # Same-name mapping (gz /overview_camera -> ros /overview_camera); the
    # surface_frame_saver node subscribes to /overview_camera.
    overview_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=['/overview_camera@sensor_msgs/msg/Image@gz.msgs.Image'],
    )

    def node(exe, **params):
        p = {'use_sim_time': use_sim_time}
        p.update(params)
        return Node(package='underwater_robot', executable=exe,
                    output='screen', parameters=[p])

    autonomy = [
        node('actuator_backend', backend='sim', allow_hardware=False, max_thrust=2.0),
        node('thruster_allocator', max_thrust=2.0),
        node('depth_pressure'),
        node('gps_adapter'),
        node('color_sensor'),
        node('surface_oil_trigger'),
        node('surface_frame_saver'),
        node('surface_rviz_markers'),
        node('mission_manager',
             autostart=ParameterValue(
                 LaunchConfiguration('autostart'), value_type=bool),
             target_depth=0.0,
             forward_speed=0.45,
             yaw_gain=0.9,
             depth_gain=0.6,
             depth_damp=1.5,
             heave_limit=0.30,
             xmin=-4.0, xmax=4.0, ymin=-4.0, ymax=4.0,
             spacing=2.0,
             arrival_radius=1.0,
             hold_duration=4.0,
             mission_timeout=300.0),
    ]

    rviz_view = ExecuteProcess(
        cmd=['rviz2', '-d', LaunchConfiguration('rviz_config')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    # Put the GUI camera in follow mode on the robot a few seconds after start
    # (BEST-EFFORT; the fixed camera_pose in gui_surface.config already gives a
    # good overview if this does not land).
    follow_msg = (
        'track_mode: 2, follow_target: {name: "underwater_robot"}, '
        'follow_offset: {x: 1.6, y: -1.6, z: 0.9}'
    )
    gui_follow = [
        'bash', '-c',
        'for i in $(seq 1 20); do '
        'gz topic -t /gui/track -m gz.msgs.CameraTrack -p "$1" '
        '&& sleep 1; done',
        'surface_follow', follow_msg,
    ]

    return LaunchDescription(
        args + [
            SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
            gz_both,
            gz_headless,
            rsp,
            bridge,
            overview_bridge,
            TimerAction(period=spawn_delay, actions=[spawn]),
            TimerAction(period=spawn_delay, actions=autonomy),
            TimerAction(period=8.0, actions=[ExecuteProcess(cmd=gui_follow, output='screen')]),
            # Start RViz ~12 s in so the robot, TF and odometry already exist.
            TimerAction(period=12.0, actions=[rviz_view]),
        ]
    )
