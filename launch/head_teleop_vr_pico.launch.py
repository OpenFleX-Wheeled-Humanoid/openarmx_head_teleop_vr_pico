# Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International
#
# Copyright (c) 2026 Chengdu Changshu Robot Co., Ltd.
# https://www.openarmx.com
#
# This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike
# 4.0 International License (CC BY-NC-SA 4.0).
#
# To view a copy of this license, visit:
# http://creativecommons.org/licenses/by-nc-sa/4.0/
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('control_rate', default_value='50.0'),
        DeclareLaunchArgument('button_b_topic',
                              default_value='/pico_right_controller/button_b'),
        DeclareLaunchArgument('head_pose_topic',
                              default_value='/pico_head/pose'),
        DeclareLaunchArgument('rate_topic',
                              default_value='/pico_left_controller/rate'),
        DeclareLaunchArgument('command_topic',
                              default_value='/head_forward_position_controller/commands'),
        DeclareLaunchArgument('slow_max_step_deg', default_value='2.0'),
        DeclareLaunchArgument('fast_max_step_deg', default_value='5.0'),
        DeclareLaunchArgument('yaw_scale', default_value='1.0'),
        DeclareLaunchArgument('pitch_scale', default_value='1.0'),
        DeclareLaunchArgument('invert_yaw', default_value='false'),
        DeclareLaunchArgument('invert_pitch', default_value='false'),
        DeclareLaunchArgument('enable_soft_limits', default_value='true'),
        DeclareLaunchArgument('yaw_soft_margin_deg', default_value='10.0'),
        DeclareLaunchArgument('pitch_soft_margin_deg', default_value='5.0'),
        DeclareLaunchArgument('soft_limit_blend_deg', default_value='10.0'),
        DeclareLaunchArgument('startup_home_enabled', default_value='true'),
        DeclareLaunchArgument('startup_home_step_deg', default_value='1.0'),
        DeclareLaunchArgument('startup_home_tolerance_deg', default_value='1.0'),
        DeclareLaunchArgument('head_pose_timeout_sec', default_value='0.3'),
        DeclareLaunchArgument('publish_visualization_tf', default_value='false'),

        Node(
            package='openarmx_head_teleop_vr_pico',
            executable='head_teleop_node',
            name='head_teleop_vr_pico',
            output='screen',
            parameters=[{
                'control_rate': LaunchConfiguration('control_rate'),
                'button_b_topic': LaunchConfiguration('button_b_topic'),
                'head_pose_topic': LaunchConfiguration('head_pose_topic'),
                'rate_topic': LaunchConfiguration('rate_topic'),
                'command_topic': LaunchConfiguration('command_topic'),
                'slow_max_step_deg': LaunchConfiguration('slow_max_step_deg'),
                'fast_max_step_deg': LaunchConfiguration('fast_max_step_deg'),
                'yaw_scale': LaunchConfiguration('yaw_scale'),
                'pitch_scale': LaunchConfiguration('pitch_scale'),
                'invert_yaw': LaunchConfiguration('invert_yaw'),
                'invert_pitch': LaunchConfiguration('invert_pitch'),
                'enable_soft_limits': LaunchConfiguration('enable_soft_limits'),
                'yaw_soft_margin_deg': LaunchConfiguration('yaw_soft_margin_deg'),
                'pitch_soft_margin_deg': LaunchConfiguration('pitch_soft_margin_deg'),
                'soft_limit_blend_deg': LaunchConfiguration('soft_limit_blend_deg'),
                'startup_home_enabled': LaunchConfiguration('startup_home_enabled'),
                'startup_home_step_deg': LaunchConfiguration('startup_home_step_deg'),
                'startup_home_tolerance_deg': LaunchConfiguration('startup_home_tolerance_deg'),
                'head_pose_timeout_sec': LaunchConfiguration('head_pose_timeout_sec'),
                'publish_visualization_tf': LaunchConfiguration('publish_visualization_tf'),
            }],
        ),
    ])
