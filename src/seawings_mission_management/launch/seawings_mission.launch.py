#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('seawings_mission_management')
    
    # Path to parameters file
    params_file = os.path.join(pkg_dir, 'config', 'mission_params.yaml')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    start_all = LaunchConfiguration('start_all', default='true')
    start_power = LaunchConfiguration('start_power', default='true')
    start_fault = LaunchConfiguration('start_fault', default='true')
    start_supervisor = LaunchConfiguration('start_supervisor', default='true')
    log_level = LaunchConfiguration('log_level', default='info')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    declare_start_all = DeclareLaunchArgument(
        'start_all',
        default_value='true',
        description='Start all nodes'
    )
    
    declare_start_power = DeclareLaunchArgument(
        'start_power',
        default_value='true',
        description='Start power monitor node'
    )
    
    declare_start_fault = DeclareLaunchArgument(
        'start_fault',
        default_value='true',
        description='Start fault detector node'
    )
    
    declare_start_supervisor = DeclareLaunchArgument(
        'start_supervisor',
        default_value='true',
        description='Start mission supervisor node'
    )
    
    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error)'
    )
    
    # Power Monitor Node
    power_monitor_node = Node(
        package='seawings_mission_management',
        executable='power_monitor',
        name='power_monitor',
        output='screen',
        parameters=[params_file],
        arguments=['--ros-args', '--log-level', log_level],
        condition=IfCondition(
            PythonExpression([start_all, ' or ', start_power])
        ),
        prefix=['gnome-terminal --tab --title="Power Monitor" -- '] if os.environ.get('DISPLAY') else [],
        respawn=True,
        respawn_delay=2,
    )
    
    # Fault Detector Node
    fault_detector_node = Node(
        package='seawings_saftety',
        executable='fault_detector',
        name='fault_detector',
        output='screen',
        parameters=[params_file],
        arguments=['--ros-args', '--log-level', log_level],
        condition=IfCondition(
            PythonExpression([start_all, ' or ', start_fault])
        ),
        prefix=['gnome-terminal --tab --title="Fault Detector" -- '] if os.environ.get('DISPLAY') else [],
        respawn=True,
        respawn_delay=2,
    )
    
    # Mission Supervisor Node
    mission_supervisor_node = Node(
        package='sseawings_mission_management',
        executable='mission_supervisor',
        name='mission_supervisor',
        output='screen',
        parameters=[params_file],
        arguments=['--ros-args', '--log-level', log_level],
        condition=IfCondition(
            PythonExpression([start_all, ' or ', start_supervisor])
        ),
        prefix=['gnome-terminal --tab --title="Mission Supervisor" -- '] if os.environ.get('DISPLAY') else [],
        respawn=True,
        respawn_delay=2,
    )
    
    # Optional: Status monitor (prints combined status)
    status_monitor_cmd = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--tab', '--title="Status Monitor"', '--',
            'bash', '-c',
            'while true; do '
            'echo "=== SYSTEM STATUS ==="; '
            'ros2 topic echo /power_monitor/status --once 2>/dev/null | head -20; '
            'echo ""; '
            'ros2 topic echo /fault_detector/status --once 2>/dev/null | head -20; '
            'echo ""; '
            'ros2 topic echo /mission_supervisor/status --once 2>/dev/null | head -20; '
            'echo ""; '
            'sleep 2; '
            'clear; '
            'done'
        ],
        condition=IfCondition(
            PythonExpression(['True if "', os.environ.get('DISPLAY', ''), '" else False'])
        ),
        output='screen'
    )
    
    return LaunchDescription([
        # Launch arguments
        declare_use_sim_time,
        declare_start_all,
        declare_start_power,
        declare_start_fault,
        declare_start_supervisor,
        declare_log_level,
        
        # Nodes
        power_monitor_node,
        fault_detector_node,
        mission_supervisor_node,
        
        # Optional status monitor
        # status_monitor_cmd,  # Uncomment if you want the status terminal
    ])