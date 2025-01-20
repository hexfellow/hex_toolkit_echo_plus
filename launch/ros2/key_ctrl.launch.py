#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2024 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2024-12-23
################################################################

from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
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

    # key ctrl
    key_ctrl_param = FindPackageShare('hex_toolkit_echo_plus').find(
        'hex_toolkit_echo_plus') + '/config/ros2/key_ctrl.yaml'
    key_ctrl_node = Node(
        package='hex_toolkit_general_chasssis',
        executable='key_ctrl',
        name='key_ctrl',
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('sim_flag'),
            },
            key_ctrl_param,
        ],
        remappings=[
            # publish
            ('/unsafe_ctrl', '/cmd_vel'),
            ('/vel_ctrl', '/unused'),
        ],
    )

    # bringup
    bringup_launch_path = FindPackageShare('hex_toolkit_echo_plus').find(
        'hex_toolkit_echo_plus') + '/launch/ros2/bringup.launch.py'
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([bringup_launch_path]),
        launch_arguments={
            'visual_flag': LaunchConfiguration('visual_flag'),
            'sim_flag': LaunchConfiguration('sim_flag'),
        }.items(),
    )

    return LaunchDescription([
        # arg
        visual_flag,
        sim_flag,
        # key ctrl
        key_ctrl_node,
        # bringup
        bringup_launch,
    ])
