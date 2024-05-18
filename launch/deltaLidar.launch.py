import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='delta_lidar',
            executable='delta_lidar_node',
            name='delta_lidar_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.4:1.0-port0',
                'frame_id': 'laser_frame',
            }]
        )
    ])
