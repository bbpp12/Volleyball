#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robocon排球赛ROS2启动文件：一键启动上位机发布+下位机订阅节点
适配ROS2 Humble
"""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 上位机球点发布节点
    upper_pub_node = Node(
        package="volleyball_communication",
        executable="ball_point_publisher",
        name="upper_ball_point_pub",
        output="screen"  
    )

    # 下位机球点订阅节点
    lower_sub_node = Node(
        package="volleyball_communication",
        executable="ball_point_subscriber",
        name="lower_ball_point_sub",
        output="screen"  
    )

    # 组装并返回启动描述
    return LaunchDescription([upper_pub_node, lower_sub_node])
