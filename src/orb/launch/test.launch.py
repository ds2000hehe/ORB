import rclpy
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription([
        
        Node(
            package='orb',
            executable='img_send.py',
            name="ImageSender"
        ),
        # ExecuteProcess(
        #     cmd=['python3', '/home/david/go2/src/orb/scripts/keypoint_viewport.py'],
        #     shell=False,
        #     output='screen'
        # ),
        ExecuteProcess(
            cmd=['python3', '/home/david/go2/src/orb/scripts/mono_odom_copy.py'],
            shell=False,
            output='screen'
        )
    ])

