# slam.launch.py
# ===============
# STEP 2 — Run this in Terminal 2 (while Gazebo is running)
#
# Starts ONLY:
#   • SLAM Toolbox (async online mode) — builds the map live
#   • RViz with the SLAM config — so you can watch the map being built
#     and save it when done
#
# Workflow:
#   1. Drive the robot around (teleop or manually in Gazebo)
#   2. Watch the map build up in RViz
#   3. When the map looks complete, save it:
#        ros2 run nav2_map_server map_saver_cli -f ~/ws_ddmobile/src/mobile_dd_robot/maps/my_map
#   4. Ctrl+C this terminal to stop SLAM
#
# Usage:
#   ros2 launch mobile_dd_robot slam.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'mobile_dd_robot'
    pkg_dir      = get_package_share_directory(package_name)

    slam_params_file = os.path.join(pkg_dir, 'config', 'mapper_params_online_async.yaml')
    rviz_config_file = os.path.join(pkg_dir, 'config', 'rviz_slam_config.rviz')

    # ── SLAM Toolbox ─────────────────────────────────────────────────────────
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file],
        remappings=[('scan', '/scan')]
    )

    # ── RViz (SLAM view — shows map being built) ──────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        slam_toolbox_node,
        rviz_node,
    ])
