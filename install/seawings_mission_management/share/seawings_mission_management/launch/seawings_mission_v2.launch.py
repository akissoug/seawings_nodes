#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Package directory
    pkg_dir = get_package_share_directory('seawings_mission_management')
    
    # Get home directory for PX4 path
    home_dir = os.path.expanduser('~')
    
    # Launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'mission_params.yaml'),
        description='Path to mission configuration file'
    )
    
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='seawings',
        description='ROS namespace for the mission nodes'
    )
    
    px4_path_arg = DeclareLaunchArgument(
        'px4_path',
        default_value=os.path.join(home_dir, 'PX4-Autopilot'),
        description='Path to PX4-Autopilot directory'
    )
    
    # Configuration
    config_file = LaunchConfiguration('config_file')
    namespace = LaunchConfiguration('namespace')
    px4_path = LaunchConfiguration('px4_path')

    # Start micro XRCE-DDS Agent in new terminator tab
    xrce_dds_agent = ExecuteProcess(
        cmd=[
            'terminator', '-e',
            f'bash -c \'echo "🛰️ Starting micro XRCE-DDS Agent on port 8888..." && MicroXRCEAgent udp4 -p 8888; exec bash\''
        ],
        output='screen',
        emulate_tty=True
    )

    # Start PX4 SITL in new terminator tab
    px4_sitl = ExecuteProcess(
        cmd=[
            'terminator', '-e',
            f'bash -c \'cd {px4_path} && echo "🛩️ Starting PX4 SITL with Gazebo..." && make px4_sitl gazebo; exec bash\''
        ],
        output='screen',
        emulate_tty=True
    )

    # Power Monitor Node
    power_monitor_terminal = ExecuteProcess(
        cmd=[
            'terminator', '-e',
            f'bash -c \'source {os.path.join(home_dir, "sw_ws/install/setup.bash")} && '
            f'echo "🔋 Power Monitor launching..." && '
            f'ros2 run seawings_mission_management power_monitor --ros-args -p config_file:={config_file}; exec bash\''
        ],
        output='screen',
        emulate_tty=True
    )

    # Fault Detector Node
    fault_detector_terminal = ExecuteProcess(
        cmd=[
            'terminator', '-e',
            f'bash -c \'source {os.path.join(home_dir, "sw_ws/install/setup.bash")} && '
            f'echo "🛰️ Fault Detector launching..." && '
            f'ros2 run seawings_mission_management fault_detector --ros-args -p config_file:={config_file}; exec bash\''
        ],
        output='screen',
        emulate_tty=True
    )

    # Mission Supervisor Node
    mission_supervisor_terminal = ExecuteProcess(
        cmd=[
            'terminator', '-e',
            f'bash -c \'source {os.path.join(home_dir, "sw_ws/install/setup.bash")} && '
            f'echo "🎯 Mission Supervisor launching..." && '
            f'ros2 run seawings_mission_management mission_supervisor --ros-args -p config_file:={config_file}; exec bash\''
        ],
        output='screen',
        emulate_tty=True
    )

    # Final launch description
    return LaunchDescription([
        config_file_arg,
        namespace_arg,
        px4_path_arg,
        LogInfo(msg=['🚀 Launching SEAWINGS Mission Management System with PX4 SITL...']),
        LogInfo(msg=['🔌 Starting XRCE-DDS Agent...']),
        xrce_dds_agent,
        LogInfo(msg=['⏳ Waiting 2 seconds...']),
        ExecuteProcess(cmd=['sleep', '2']),
        LogInfo(msg=['🛫 Starting PX4 SITL...']),
        px4_sitl,
        LogInfo(msg=['⏳ Waiting 30 seconds for PX4 to initialize...']),
        ExecuteProcess(cmd=['sleep', '30']),
        LogInfo(msg=['⚡ Starting Power Monitor node...']),
        power_monitor_terminal,
        LogInfo(msg=['🛑 Starting Fault Detector node...']),
        fault_detector_terminal,
        LogInfo(msg=['🧠 Starting Mission Supervisor node...']),
        mission_supervisor_terminal,
        LogInfo(msg=['✅ All systems launched! Start QGroundControl manually if not running.'])
    ])
