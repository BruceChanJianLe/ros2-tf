# ROS2 tf

This repository is a simple demonstration of tf(2) usage in ROS2.

## Content

- [Publish Statis Transform](#Publish-Statis-Transform)
- [TF2 Broadcaster](#TF2-Broadcaster)
- [TF2 Listener](#TF2-Listener)
- [TF Prefix](#TF-Prefix)
- [Errors](#Error)

## Publish Statis Transform

**TF2 (Recommended)**  
```py
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    static_broadcaster_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=['1', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    # Define launch description
    ld = LaunchDescription()
    ld.add_action(static_broadcaster_node)

    # Return launch description
    return ld
```
Explanation: `static_transform_publisher x y z yaw pitch roll parent_frame child_frame`


## TF2 Broadcaster

Details of writing a tf2 broadcaster display in the ROS node broadcaster_node. Look at `broadcaster.launch.py` for details. Use `ros2 topic pub /reset_pose std_msgs/msg/Bool "data: true/false" -1` command to trigger the broadcaster to broadcast. Note that to view the changes, you may want to have the static tf there.  

![before](images/before_broadcast.png) ![after](images/broadcast.png)

```bash
# Terminal 1
ros2 topic pub /reset_pose std_msgs/msg/Bool "data: false" -1
# Terminal 2
[INFO] [1656750099.359073644] [tf2_broadcaster_node]: tf2_broadcaster_node: setting to new pose.
```

Setting back to original pose.
```bash
# Terminal 1
ros2 topic pub /reset_pose std_msgs/msg/Bool "data: true" -1
# Terminal 2
[INFO] [1656750008.527549084] [tf2_broadcaster_node]: tf2_broadcaster_node: reseting to original pose.
```

## TF2 Listener

tf2 listener demonstrates how the node is able to listen to the changes between fixed_frame and odom_frame. You may use the broadcaster node to change the odom_frame position and request the listener node to display the position information through a rostopic as shown below.  

```bash
# Terminal 1
ros2 topic pub /reset_pose std_msgs/msg/Bool "data: false" -1
# Terminal 2
## Before broadcater node change the tf
[INFO] [1656754760.136636691] [tf2_listener_node]: tf2_listener_nodelistener_node: position x, y, z (1, 1, 0) x, y, z, w (0, 0, 0, 1)
## After broadcater node change the tf
[INFO] [1656754759.136687154] [tf2_listener_node]: tf2_listener_nodelistener_node: position x, y, z (2, 3, 1) x, y, z, w (0, 0, 0, 1)
```

## TF Prefix

Unfortunately, you will have to write code to enable prefix for tf frames. An good example will be the `robot_state_publisher` node. However, it is only supported in ROS1 tf package (only supported until `melodic`, if you run on `noetic` it will break.). You may write your own like so.

```python
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
        arguments=['1', '1', '0', '0', '0', '0', PathJoinSubstitution([LaunchConfiguration('prefix'), 'map']), PathJoinSubstitution([LaunchConfiguration('prefix'), 'odom'])]
    )

    # Define launch description
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(static_broadcaster_node)

    # Return launch description
    return ld
```

For tf2 you will need to write a node to do so. Please refer to the link: https://bitbucket.org/osrf/ariac/src/master/osrf_gear/src/tf2_relay.cpp  

## Error

Below are the recommended methods if you would like to listen to a TF (works for simulation and real world scenario).  
```cpp
geometry_msgs::msg::TransformStamped read_transformation;

try
{
    if (tf_buffer_->canTransform(fixed_frame_, target_frame_, rclcpp::Time(0), rclcpp::Duration(3.0)))
        read_transformation = tf_buffer_->lookupTransform(fixed_frame_, target_frame_, this->get_clock()->now());
    else
        RCLCPP_WARN_STREAM(
            this->get_logger(),
            this->get_name()
            << ": unable to transform from "
            << fixed_frame_
            << " to "
            << target_frame_
        );
}
catch(tf2::TransformException & e)
{
    RCLCPP_ERROR_STREAM(
        this->get_logger(),
        this->get_name()
        << " ("
        << __func__
        << ") caught an transform error: "
        << e.what()
    );
    return;
}
```

For more information please visit this [link](https://answers.ros.org/question/312648/could-not-find-waitfortransform-function-in-tf2-package-of-ros2/)
Difference between static broadcaster and broadcaster [link]()
