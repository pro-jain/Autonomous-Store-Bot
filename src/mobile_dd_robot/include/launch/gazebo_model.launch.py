# BodhiniJain_U23CS060
# gazebo_model.launch.py
# ========================
# STEP 1 — Run this first in Terminal 1
#
# Starts ONLY:
#   • Gazebo simulator with the Digital Showroom world
#   • robot_state_publisher (TF tree for the robot)
#   • Spawns the robot at its starting position
#
# Does NOT start: SLAM, RViz, Nav2, AMCL — those are separate.
#
# Usage:
#   ros2 launch mobile_dd_robot gazebo_model.launch.py

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    package_name         = 'mobile_dd_robot'
    model_relative_path  = 'model/robot.xacro'
    world_relative_path  = 'model/Digital_showroom/showroom.world'

    pkg_dir       = get_package_share_directory(package_name)
    path_model    = os.path.join(pkg_dir, model_relative_path)
    path_world    = os.path.join(pkg_dir, world_relative_path)
    robot_desc    = xacro.process_file(path_model).toxml()

    # ── Gazebo ───────────────────────────────────────────────────────────────
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch', 'gazebo.launch.py'
            )
        ),
        launch_arguments={
            'world':   path_world,
            'verbose': 'false',
        }.items()
    )

    # ── Robot State Publisher (TF tree) ──────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time':      True,
        }]
    )

    # ── Spawn robot in Gazebo ────────────────────────────────────────────────
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'differential_drive_robot',
            '-x', '-6.06',
            '-y', '-6.5',
            '-z', '0.2',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0',
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher,
        spawn_robot,
    ])
