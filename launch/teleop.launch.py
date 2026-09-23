"""Launch the Muto teleop node.

muto_driver must NOT be running -- both want the serial port, and the kernel
does not stop them sharing it. The symptom is corrupt frames, not an error.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port = LaunchConfiguration('port')

    return LaunchDescription([
        DeclareLaunchArgument(
            'port', default_value='auto',
            description='Serial device, or "auto" to find the CH340 by USB id'),
        Node(
            package='muto_teleop',
            executable='teleop_node',
            name='muto_teleop',
            output='screen',
            parameters=[{
                'port': port,
                'home_pan': 90,
                'home_tilt': 4,
            }],
        ),
    ])
