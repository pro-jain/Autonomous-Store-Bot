# navigation.launch.py
# =====================
# STEP 4 — Run this in Terminal 2 (after Gazebo is running, map is saved,
#           SLAM is stopped, and YOLO detections CSV is ready)
#
# Starts ONLY:
#   • map_server        — serves the saved my_map.yaml / my_map.pgm
#   • amcl              — Monte-Carlo localisation (locates robot on the map)
#   • Nav2 stack        — controller, planner (A*), behaviour tree, recoveries
#   • RViz              — Nav2 view (set 2D Pose Estimate here before step 5)
#
# Does NOT start: Gazebo, SLAM toolbox, YOLO node, goto_object node.
#
# After this is running and all nodes are ACTIVE:
#   • In RViz → click "2D Pose Estimate" → click where the robot is on the map
#   • Then run Terminal 3: ros2 run semantic_navigation goto_object
#
# Usage:
#   ros2 launch mobile_dd_robot navigation.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'mobile_dd_robot'
    pkg_dir      = get_package_share_directory(package_name)

    # ── File paths ────────────────────────────────────────────────────────────
    nav2_params_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    map_file         = os.path.join(pkg_dir, 'maps',   'showroom_map.yaml')
    # Use the nav2 default RViz config — shows costmaps, path, goal arrow
    rviz_config_file = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'rviz', 'nav2_default_view.rviz'
    )

    # ── Launch arguments ──────────────────────────────────────────────────────
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart    = LaunchConfiguration('autostart',    default='true')

    # ── map_server ────────────────────────────────────────────────────────────
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time':  use_sim_time,
            'yaml_filename': map_file,
        }]
    )

    # ── AMCL (particle-filter localisation) ──────────────────────────────────
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params_file]
    )

    # ── Lifecycle manager: map_server + amcl ─────────────────────────────────
    lifecycle_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart':    autostart,
            'node_names':   ['map_server', 'amcl'],
        }]
    )

    # ── Nav2 navigation stack ─────────────────────────────────────────────────
    # Includes: controller_server, planner_server, behavior_server,
    #           bt_navigator, velocity_smoother, waypoint_follower
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch', 'navigation_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file':  nav2_params_file,
            'autostart':    autostart,
        }.items()
    )

    # ── RViz (Nav2 view — costmaps, path, pose estimate) ─────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart',    default_value='true'),

        map_server_node,
        amcl_node,
        lifecycle_localization,
        nav2_bringup,
        rviz_node,
    ])
