"""
End-to-end simulation launch for the bio-inspired turtle robot.

Starts, in order:
  1. Gazebo Harmonic server (``gz sim -s -r``), plus the GUI on demand
  2. robot_state_publisher (TF from the URDF + /joint_states)
  3. the robot model, spawned via ros_gz_sim create
  4. ros_gz_bridge with the package bridge.yaml
  5. the ROS 2 control / sensor / autonomy nodes

Typical use::

    ros2 launch underwater_robot simulation.launch.py
    ros2 launch underwater_robot simulation.launch.py gui:=true
    ros2 launch underwater_robot simulation.launch.py mission_autostart:=true

The mission is OFF by default (``mission_autostart:=false``) so the vehicle
does not move until it is explicitly told to; the actuator backend also starts
in the zero-command state.
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
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('underwater_robot')

    world_default = os.path.join(pkg_share, 'worlds', 'underwater_world.sdf')
    urdf_default = os.path.join(pkg_share, 'urdf', 'final_robot.urdf')
    bridge_default = os.path.join(pkg_share, 'config', 'bridge.yaml')
    gui_config_default = os.path.join(pkg_share, 'config', 'gui.config')

    # Gazebo resolves model://underwater_robot/... by looking inside the parent
    # of the installed package share directory (and the source tree).
    share_parent = os.path.dirname(pkg_share.rstrip('/'))
    resource_path = os.pathsep.join(
        [share_parent, os.path.expanduser('~/ros2_ws/src'), '/opt/ros/jazzy/share']
    )

    args = [
        DeclareLaunchArgument('gui', default_value='false',
                              description='Start the Gazebo GUI as well as the server.'),
        DeclareLaunchArgument('world', default_value=world_default),
        DeclareLaunchArgument('robot', default_value=urdf_default),
        DeclareLaunchArgument('bridge_config', default_value=bridge_default),
        DeclareLaunchArgument('gui_config', default_value=gui_config_default,
                              description='GUI client config (deterministic camera pose).'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('mission_autostart', default_value='false'),
        DeclareLaunchArgument('spawn_delay', default_value='5.0'),
        DeclareLaunchArgument('scripted_test', default_value='false',
                              description='Also start the bounded scripted command test.'),
        DeclareLaunchArgument(
            'spawn_z', default_value='-4.0',
            description=('Spawn height of the robot (m).  Depth-positive '
                         'convention: z = 0 is the water surface, negative z '
                         'is below it.  The 4 m acceptance run keeps the '
                         'default -4.0.')),
        DeclareLaunchArgument(
            'target_depth', default_value='4.0',
            description=('Mission hold depth (m below the surface, positive '
                         'downward).  Default 4.0 matches the acceptance run; '
                         'the surface demo overrides this to ~0.15.')),
    ]

    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    spawn_delay = LaunchConfiguration('spawn_delay')

    # --- Gazebo: always run the server; optionally attach the GUI -----------
    gz_server = ExecuteProcess(
        cmd=['gz', 'sim', '-s', '-r', LaunchConfiguration('world')],
        output='screen',
    )
    gz_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-g', '--gui-config', LaunchConfiguration('gui_config')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('gui')),
    )

    # --- TF from the robot description --------------------------------------
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('robot')]), value_type=str
    )
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}],
    )

    # --- Spawn the model -----------------------------------------------------
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-file', LaunchConfiguration('robot'),
            '-name', 'underwater_robot',
            '-x', '0', '-y', '0', '-z', LaunchConfiguration('spawn_z'),
            # Face the first lawnmower waypoint (xmin, ymin) = (-6, -6):
            # heading atan2(-6, -6) = -135 deg, so the mission does not need
            # to make a violent initial yaw turn at spawn.
            '-Y', '-2.356194',
        ],
    )

    # --- Bridge --------------------------------------------------------------
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{'config_file': LaunchConfiguration('bridge_config')}],
    )

    # --- ROS 2 nodes ---------------------------------------------------------
    def node(exe, **params):
        p = {'use_sim_time': use_sim_time}
        p.update(params)
        return Node(package='underwater_robot', executable=exe,
                    output='screen', parameters=[p])

    autonomy_nodes = [
        node('actuator_backend', backend='sim', allow_hardware=False, max_thrust=2.0),
        node('thruster_allocator', max_thrust=2.0),
        node('depth_pressure'),
        node('gps_adapter'),
        node('color_sensor'),
        node('oil_detector'),
        node('mission_manager',
             autostart=ParameterValue(
                 LaunchConfiguration('mission_autostart'), value_type=bool),
             target_depth=ParameterValue(
                 LaunchConfiguration('target_depth'), value_type=float)),
    ]

    return LaunchDescription(
        args
        + [
            SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
            gz_server,
            gz_gui,
            rsp,
            bridge,
            TimerAction(period=spawn_delay, actions=[spawn]),
            TimerAction(period=spawn_delay, actions=autonomy_nodes),
            TimerAction(
                period=spawn_delay,
                actions=[node('scripted_test')],
                condition=IfCondition(LaunchConfiguration('scripted_test')),
            ),
        ]
    )
