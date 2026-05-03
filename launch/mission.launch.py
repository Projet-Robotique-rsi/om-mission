#!/usr/bin/env python3
"""
Launch the mission planner.
Run AFTER:
  1. ros2 launch om_description gazebo.launch.py
  2. ros2 launch om_perception perception.launch.py
  3. ros2 launch om_motion motion.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    mission_planner = Node(
        package='om_mission',
        executable='mission_planner',
        name='mission_planner',
        output='screen',
    )

    return LaunchDescription([mission_planner])
