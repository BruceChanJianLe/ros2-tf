#pragma once

// ROS2
#include "rclcpp/rclcpp.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

// STL
#include <memory>

namespace tf
{
    class broadcaster : public rclcpp::Node
    {
    public:
        broadcaster();
        ~broadcaster();
    private:
        void loadROS2Param();
        void tfBroadcasterCB(const std_msgs::msg::Bool::SharedPtr msg);

        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;

        std::string fixed_frame_;
        std::string odom_frame_;
        geometry_msgs::msg::TransformStamped pose;
    };

} // namespace tf
