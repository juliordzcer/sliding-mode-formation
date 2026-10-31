# Copyright 2026 Julio Cesar Rodriguez <contact@juliordz.com>
# SPDX-License-Identifier: Apache-2.0
"""Gazebo, the ROS bridge, the formation node and the control panel.

Usage:
    ros2 launch comrob2023_formation formation.launch.py controller:=ntsmc
    ros2 launch comrob2023_formation formation.launch.py controller:=sta disturbance:=true
"""
import json
import os

from ament_index_python.packages import get_package_share_directory
from comrob2023_formation import CONTROLLERS, make_world, N, WORLD
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def bridge_arguments():
    args = ['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            f'/world/{WORLD}/wrench@ros_gz_interfaces/msg/EntityWrench]gz.msgs.EntityWrench']
    for i in range(N):
        args.append(f'/espdrone{i + 1}/cmd_position@geometry_msgs/msg/Pose]gz.msgs.Pose')
        args.append(f'/espdrone{i + 1}/motor_enable@std_msgs/msg/Bool]gz.msgs.Boolean')
        args.append(f'/model/espdrone{i + 1}/pose@geometry_msgs/msg/Pose[gz.msgs.Pose')
    return args


def launch_setup(context):
    get = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    controller = get('controller')
    if controller not in CONTROLLERS:
        raise RuntimeError(f'controller must be one of {CONTROLLERS}')
    share = get_package_share_directory('comrob2023_formation')
    world = make_world(os.path.join(share, 'worlds', 'espdrone_multi.sdf'), controller,
                       json.loads(get('gains')))
    gz_cmd = ['gz', 'sim', '-r'] + (['-s'] if get('headless') == 'true' else []) + [world]
    return [
        ExecuteProcess(cmd=gz_cmd, output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             arguments=bridge_arguments()),
        Node(package='comrob2023_formation', executable='formation_node.py', output='screen',
             parameters=[{
                 'controller': controller,
                 'time_scale': float(get('time_scale')),
                 'disturbance': get('disturbance') == 'true',
                 'auto_sequence': get('auto_sequence') == 'true',
                 'duration': float(get('duration')),
                 'output': get('output'),
             }]),
        Node(package='comrob2023_formation', executable='control_panel.py', output='screen',
             arguments=[controller], condition=IfCondition(get('run_gui'))),
    ]


def generate_launch_description():
    defaults = {'controller': 'ntsmc', 'headless': 'false', 'run_gui': 'true',
                'disturbance': 'false', 'auto_sequence': 'false', 'time_scale': '0.5',
                'duration': '50.0', 'output': '', 'gains': '{}'}
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()]
        + [OpaqueFunction(function=launch_setup)])
