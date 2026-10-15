from glob import glob
import os

from setuptools import setup

package_name = 'underwater_robot'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.py'),
        ),
        (
            os.path.join('share', package_name, 'urdf'),
            glob('urdf/*'),
        ),
        (
            os.path.join('share', package_name, 'worlds'),
            glob('worlds/*'),
        ),
        (
            os.path.join('share', package_name, 'meshes'),
            glob('meshes/*'),
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*'),
        ),
        (
            os.path.join('share', package_name, 'docs'),
            glob('docs/*'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tanuja-ghashi',
    maintainer_email='tanuja-ghashi@todo.todo',
    description='Bio-inspired turtle robot for marine oil-spill detection '
                '(Gazebo Harmonic + ROS 2 Jazzy simulation).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # control / actuation
            'thruster_allocator = underwater_robot.thruster_allocator:main',
            'actuator_backend = underwater_robot.actuator_backend:main',
            # sensor adapters
            'depth_pressure = underwater_robot.depth_pressure:main',
            'gps_adapter = underwater_robot.gps_adapter:main',
            'color_sensor = underwater_robot.color_sensor:main',
            # perception / autonomy
            'oil_detector = underwater_robot.oil_detector:main',
            'surface_oil_trigger = underwater_robot.surface_oil_trigger:main',
            'surface_frame_saver = underwater_robot.surface_frame_saver:main',
            'surface_rviz_markers = underwater_robot.surface_rviz_markers:main',
            'mission_manager = underwater_robot.mission_manager:main',
            # test / demo helpers
            'scripted_test = underwater_robot.scripted_test:main',
            # deprecated stub kept for backwards compatibility
            'thruster_controller = underwater_robot.thruster_controller:main',
        ],
    },
)
