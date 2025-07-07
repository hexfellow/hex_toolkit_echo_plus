#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-07-02
################################################################

from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import GroupAction
from launch.conditions import IfCondition
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
    test_flag = DeclareLaunchArgument(
        'test_flag',
        default_value='true',
    )

    # pid track
    urdf_file_path = FindPackageShare('hex_toolkit_echo_plus').find(
        'hex_toolkit_echo_plus') + '/urdf/echo_plus.urdf'
    pid_root_path = FindPackageShare('hex_toolkit_echo_plus').find(
        'hex_toolkit_echo_plus')
    pid_trace_yaml_path = FindPackageShare('hex_toolkit_echo_plus').find(
        'hex_toolkit_echo_plus') + '/config/ros2/pid_trace.yaml'
    pid_trace_node = Node(
        package='hex_toolkit_echo_plus',
        executable='pid_trace',
        name='pid_trace',
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('sim_flag'),
                'model_path': urdf_file_path,
                'pid_root': pid_root_path,
            },
            pid_trace_yaml_path,
        ],
        remappings=[
            # subscribe
            ('/chassis_odom', '/odom'),
            ('/target_pose', '/target_pose'),
            # publish
            ('/unsafe_ctrl', '/cmd_vel'),
            ('/vel_ctrl', '/unused'),
        ],
    )

    # test
    list_gen_path = FindPackageShare('hex_toolkit_echo_plus').find(
        'hex_toolkit_echo_plus') + '/config/ros2/list_gen.yaml'
    test_group = GroupAction(
        [
            Node(
                package='hex_toolkit_general_chasssis',
                executable='list_gen',
                name='list_gen',
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        'use_sim_time': LaunchConfiguration('sim_flag'),
                    },
                    list_gen_path,
                ],
                remappings=[
                    # publish
                    ('/target_pose', '/target_pose'),
                ],
            )
        ],
        condition=IfCondition(LaunchConfiguration('test_flag')),
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
        test_flag,
        # pid track
        pid_trace_node,
        # test
        test_group,
        # bringup
        bringup_launch,
    ])
