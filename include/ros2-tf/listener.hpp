#pragma once

// ROS2
#include "rclcpp/rclcpp.hpp"

// TF2
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"

#include "geometry_msgs/msg/transform_stamped.hpp"

// STL
#include <memory>
#include <string>
#include <chrono>

namespace tf
{
    class listener : public rclcpp::Node
    {
    public:
        listener();
        ~listener();
    private:
        std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::string fixed_frame_;
        std::string target_frame_;
        rclcpp::TimerBase::SharedPtr timer_;

        void loadROS2Param();
        void PerformListening();
    };

} // namespace tf
