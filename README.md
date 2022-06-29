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

Details of writing a tf2 broadcaster display in the ROS node broadcaster_node. Look at `broadcaster.launch` for details. Use `rostopic pub /broadcaster_node/reset std_msgs/Bool "data: true"` command to trigger the broadcaster to broadcast. Note that to view the changes, you may want to have the static tf there.  

![before](images/before_broadcast.png) ![after](images/broadcast.png)

```bash
# Terminal 1
rostopic pub /broadcaster_node/reset std_msgs/Bool "data: true" -1
# Terminal 2
[ INFO] [1604912263.076903512]: callback!
[ INFO] [1604912263.276888023]: Processing!
```

## TF2 Listener

tf2 listener demonstrates how the node is able to listen to the changes between fixed_frame and odom_frame. You may use the broadcaster node to change the odom_frame position and request the listener node to display the position information through a rostopic as shown below.  

```bash
# Terminal 1
rostopic pub /listener_node/display_transform std_msgs/Bool "data: true" -1
# Terminal 2
## Before broadcater node change the tf
[ INFO] [1604912229.161304400]: listener_node: position x, y, z (1, 0, 0) x, y, z, w (0, 0, 0, 1)
## After broadcater node change the tf
[ INFO] [1604912270.161046625]: listener_node: position x, y, z (1, 1, 0) x, y, z, w (0, 0, 0, 1)
```

## TF Prefix

Unfortunately, you will have to write code to enable prefix for tf frames. An good example will be the `robot_state_publisher` node. Only supported until `melodic`, if you run on `noetic` it will break.

```xml
<?xml version="1.0"?>
<launch>
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher">
        <param name="tf_prefix" type="string" value="$(arg robot_namespace)"/>
  </node>
</launch>
```

For tf2 you will need to write a node to do so. Please refer to the link: https://bitbucket.org/osrf/ariac/src/master/osrf_gear/src/tf2_relay.cpp  

## Error

There are some funny errors that has been encountered before while using TF1. Below are the recommended methods if you would like to listen to a TF (works for simulation and real world scenario).  
```cpp
tf::StampedTransform transform;
try{
    now = ros::Time(0); // This is very important
    listener.waitForTransform(base_frame, target_frame, ros::Time(0), ros::Duration(3.0));
    listener.lookupTransform(base_frame, target_frame, ros::Time(0), transform);
}
```

There are some differences between tf2 and tf. Here I will list down one function equivalent function in tf2 to tf.
TF | TF2 
--- | ---
waitForTransform() | canTransform()

For more information please visit this [link](https://answers.ros.org/question/312648/could-not-find-waitfortransform-function-in-tf2-package-of-ros2/)
