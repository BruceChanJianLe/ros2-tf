from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    static_broadcaster_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=['1', '1', '0', '0', '0', '0', 'map', 'odom']
    )

    # Define launch description
    ld = LaunchDescription()
    ld.add_action(static_broadcaster_node)

    # Return launch description
    return ld