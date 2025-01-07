#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2024 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2024-12-23
################################################################

import xacro
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # arg
    visual_flag = DeclareLaunchArgument(
        'visual_flag',
        default_value='true',
    )
    sim_flag = DeclareLaunchArgument(
        'sim_flag',
        default_value='true',
    )

    # visual
    urdf_file_path = FindPackageShare('hex_toolkit_echo_plus').find(
        'hex_toolkit_echo_plus') + '/urdf/echo_plus_ros2.xacro'
    rviz_file_path = FindPackageShare('hex_toolkit_echo_plus').find(
        'hex_toolkit_echo_plus') + '/config/ros2/display.rviz'
    visual_group = GroupAction(
        [
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{
                    'use_sim_time':
                    LaunchConfiguration('sim_flag'),
                    'robot_description':
                    xacro.process_file(urdf_file_path).toxml(),
                }],
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz',
                output='screen',
                parameters=[{
                    'use_sim_time': LaunchConfiguration('sim_flag'),
                }],
                arguments=['-d', rviz_file_path],
            ),
        ],
        condition=IfCondition(LaunchConfiguration('visual_flag')),
    )

    # sim
    gazebo_launch_path = FindPackageShare('gazebo_ros').find(
        'gazebo_ros') + '/launch/gazebo.launch.py'
    gazebo_param = FindPackageShare('hex_toolkit_echo_plus').find(
        'hex_toolkit_echo_plus') + '/config/ros2/gazebo.yaml'
    sim_group = GroupAction(
        [
            ExecuteProcess(
                cmd=['killall', '-q', 'gzserver'],
                output='screen',
            ),
            ExecuteProcess(
                cmd=['killall', '-q', 'gzclient'],
                output='screen',
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([gazebo_launch_path]),
                launch_arguments={
                    'params_file': gazebo_param,
                }.items(),
            ),
            ExecuteProcess(
                cmd=[
                    'ros2',
                    'param',
                    'set',
                    '/gazebo',
                    'use_sim_time',
                    LaunchConfiguration('sim_flag'),
                ],
                output='screen',
            ),
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                output='screen',
                arguments=[
                    '-topic',
                    'robot_description',
                    '-entity',
                    'echo_plus',
                    '-x',
                    '0.0',
                    '-y',
                    '0.0',
                    '-z',
                    '0.0',
                    '-Y',
                    '0.0',
                ],
            ),
        ],
        condition=IfCondition(LaunchConfiguration('sim_flag')),
    )

    # real
    real_group = GroupAction(
        [
            Node(
                package='xpkg_vehicle',
                executable='xnode_vehicle',
                name='xnode_vehicle',
                output='screen',
                emulate_tty=True,
                parameters=[{
                    'can_device': 'hexcan0',
                    'calc_odom_from_speed': False,
                }],
                remappings=[
                    # subscribe
                    ('/cmd_vel', '/cmd_vel'),
                    # publish
                    ('/odom', '/odom'),
                ],
            ),
        ],
        condition=UnlessCondition(LaunchConfiguration('sim_flag')),
    )

    return LaunchDescription([
        # arg
        visual_flag,
        sim_flag,
        # visual
        visual_group,
        # sim
        sim_group,
        # real
        real_group,
    ])
