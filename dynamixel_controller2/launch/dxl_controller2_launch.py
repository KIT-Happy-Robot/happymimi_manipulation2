#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
    Node(
      package='dynamixel_controller2',
        executable='motor_controller2.py',
        name='motor_controller2',
        output='screen',
        #emulate_tty=True,
        parameters=[
          {}
        ]
    )
  ])
