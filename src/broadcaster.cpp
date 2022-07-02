#include "ros2-tf/broadcaster.hpp"

namespace tf
{
    broadcaster::broadcaster()
    :   rclcpp::Node("tf2_broadcaster_node")
    {
        // Load ROS2 Param
        loadROS2Param();

        tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster> (this);
        sub_ = this->create_subscription<std_msgs::msg::Bool>("reset_pose", 1, [this](const std_msgs::msg::Bool::SharedPtr msg){this->tfBroadcasterCB(msg);});

        RCLCPP_INFO_STREAM(
            this->get_logger(),
            this->get_name()
            << ": Started tf broadcaster node."
        );
    }

    broadcaster::~broadcaster()
    {
        // Undeclare params
        this->undeclare_parameter("fixed_frame");
        this->undeclare_parameter("odom_frame");
    }

    void broadcaster::loadROS2Param()
    {
        // Declare params
        this->declare_parameter<std::string>("fixed_frame", "map");
        this->declare_parameter<std::string>("odom_frame", "odom");
        // Obtain params
        this->get_parameter("fixed_frame", fixed_frame_);
        this->get_parameter("odom_frame", odom_frame_);
    }

    void broadcaster::tfBroadcasterCB(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data)
        {
            // Reset pose
            pose.child_frame_id = odom_frame_;
            pose.header.frame_id = fixed_frame_;
            pose.transform.translation.set__x(1.0);
            pose.transform.translation.set__y(1.0);
            pose.transform.translation.set__z(0.0);
            pose.transform.rotation.set__w(1.0);
            pose.transform.rotation.set__x(0.0);
            pose.transform.rotation.set__y(0.0);
            pose.transform.rotation.set__z(0.0);
            pose.header.set__stamp(this->get_clock()->now());
            tf_broadcaster_->sendTransform(pose);

            RCLCPP_INFO_STREAM(
                this->get_logger(),
                this->get_name()
                << ": reseting to original pose."
            );
        }
        else
        {
            // Change pose
            pose.child_frame_id = odom_frame_;
            pose.header.frame_id = fixed_frame_;
            pose.transform.translation.set__x(2.0);
            pose.transform.translation.set__y(3.0);
            pose.transform.translation.set__z(1.0);
            pose.transform.rotation.set__w(1.0);
            pose.transform.rotation.set__x(0.0);
            pose.transform.rotation.set__y(0.0);
            pose.transform.rotation.set__z(0.0);
            pose.header.set__stamp(this->get_clock()->now());
            tf_broadcaster_->sendTransform(pose);

            RCLCPP_INFO_STREAM(
                this->get_logger(),
                this->get_name()
                << ": setting to new pose."
            );
        }
    }
} // namespace tf