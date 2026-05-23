#!/usr/bin/env python3
# Nav2 Launch File - Based on your working Gazebo setup

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    namePackage = 'mobile_dd_robot'
    
    # Get paths
    nav2_params_file = os.path.join(
        get_package_share_directory(namePackage),
        'config',
        'nav2_params.yaml'
    )
    
    map_file = os.path.join(
        get_package_share_directory(namePackage),
        'maps',
        'your_map.yaml'  # CHANGE THIS TO YOUR ACTUAL MAP NAME
    )
    
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # RViz config for Nav2
    rviz_config_file = os.path.join(
        nav2_bringup_dir,
        'rviz',
        'nav2_default_view.rviz'
    )
    
    # Include Nav2 bringup launch
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'slam': 'False',
            'map': map_file,
            'use_sim_time': 'True',
            'params_file': nav2_params_file,
            'autostart': 'True',
        }.items()
    )
    
    # RViz2 Node with use_sim_time
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    # Map Server (explicit, in case bringup doesn't handle it well)
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'yaml_filename': map_file},
            {'use_sim_time': True}
        ]
    )
    
    # Lifecycle Manager for localization (AMCL)
    localization_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'autostart': True},
            {'node_names': ['map_server', 'amcl']}
        ]
    )
    
    # Lifecycle Manager for navigation
    navigation_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'autostart': True},
            {'node_names': [
                'controller_server',
                'planner_server',
                'recoveries_server',
                'bt_navigator',
                'velocity_smoother'
            ]}
        ]
    )
    
    launch_description = LaunchDescription()
    
    # Add all actions
    launch_description.add_action(nav2_launch)
    launch_description.add_action(map_server_node)
    launch_description.add_action(localization_lifecycle_manager)
    launch_description.add_action(navigation_lifecycle_manager)
    launch_description.add_action(rviz_node)
    
    return launch_description
