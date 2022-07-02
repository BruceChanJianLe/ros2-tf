from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2-tf',
            executable='listener_node',
            name='tf_broadcaster_node',
            output='screen',
            parameters=[
                {'fixed_frame' : 'map'},
                {'target_frame' : 'odom'}
            ]
        )
    ])