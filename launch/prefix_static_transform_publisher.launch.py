from launch import LaunchDescription
from launch_ros.actions import Node
# Load Prefix
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

ARGUMENTS = [
    DeclareLaunchArgument(
        'prefix',
        default_value='',
        description='The prefix for tf',
    ),
]

def generate_launch_description():

    static_broadcaster_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=['1', '0', '0', '0', '0', '0', PathJoinSubstitution([LaunchConfiguration('prefix'), 'map']), PathJoinSubstitution([LaunchConfiguration('prefix'), 'odom'])]
    )

    # Define launch description
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(static_broadcaster_node)

    # Return launch description
    return ld