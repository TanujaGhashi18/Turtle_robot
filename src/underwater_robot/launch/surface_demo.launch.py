"""
Surface-demo launch for the capstone presentation.

Runs the *same* end-to-end stack as ``simulation.launch.py`` (Gazebo +
TF + spawn + bridge + autonomy nodes) but targets a shallow hold depth so the
turtle robot cruises autonomously just below the water surface:

    * ``spawn_z = -0.5``   spawn fully submerged, close to the surface
    * ``target_depth = 0.15``  hold ~0.15 m below the surface (z ~= -0.15)
    * ``mission_autostart = true``  SEARCH -> RETURN -> DONE immediately
    * ``gui = true``  visual demo for the presentation
    * ``world = surface_demo_world.sdf``  the acceptance world plus a thin,
      translucent, visual-only water-surface slab at z = 0 (no collision,
      no buoyancy, no sensor or logic changes; ``underwater_world.sdf`` is
      untouched)

The GUI camera is also switched into *follow* mode shortly after launch so it
stays pointed at the robot while it surveys the pool (Gazebo Harmonic follow
is set over the ``/gui/track`` topic; the same commands are documented in the
README).  Because the whole stack lives in ``simulation.launch.py``, this
file is a thin argument wrapper: the default 4 m acceptance configuration is
unchanged (see ``simulation.launch.py``'s ``spawn_z`` / ``target_depth``
defaults and ``docs/test_report.md``).

Typical use::

    ros2 launch underwater_robot surface_demo.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_share = get_package_share_directory('underwater_robot')
    base_launch = os.path.join(pkg_share, 'launch', 'simulation.launch.py')

    # Gazebo Harmonic: "follow" is set over the /gui/track topic (there is no
    # F-key shortcut as in Gazebo Classic).  Retry for a while so the command
    # lands even if the GUI has not finished subscribing when we publish.
    # The protobuf text message is passed as "$1" so no shell quoting of the
    # embedded double quotes is needed.
    follow_msg = (
        'track_mode: 2, follow_target: {name: "underwater_robot"}, '
        'follow_offset: {x: 2, y: -2, z: 1}'
    )
    gui_follow = [
        'bash', '-c',
        'for i in $(seq 1 15); do '
        'gz topic -t /gui/track -m gz.msgs.CameraTrack -p "$1" '
        '&& sleep 1; done',
        'surface_demo_follow', follow_msg,
    ]

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(base_launch),
                launch_arguments={
                    'gui': 'true',
                    'mission_autostart': 'true',
                    'spawn_z': '-0.5',
                    'target_depth': '0.15',
                    'world': os.path.join(pkg_share, 'worlds', 'surface_demo_world.sdf'),
                }.items(),
            ),
            # Give the GUI time to attach after the base launch starts it.
            TimerAction(period=10.0, actions=[ExecuteProcess(cmd=gui_follow, output='screen')]),
        ]
    )
