#!/usr/bin/python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    return LaunchDescription([
        Node(
            package='limo_control',
            executable='limo_control_node',

            # Setting desired target position and angle from launch
            parameters=[
                {"target_x": 5.0},
                {"target_y": 5.0},
                {"target_theta": 3.1415}
            ],
            name='limo_control'
        )
    ])