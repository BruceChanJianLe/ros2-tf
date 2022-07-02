#include "ros2-tf/listener.hpp"

namespace tf
{
    listener::listener()
    : rclcpp::Node("tf2_listener_node")
    {
        // Load ROS2 Param
        loadROS2Param();

        tf_buffer_ = std::make_unique<tf2_ros::Buffer> (this->get_clock());
        transform_listener_ = std::make_shared<tf2_ros::TransformListener> (*tf_buffer_);

        timer_ = this->create_wall_timer (std::chrono::seconds (1), [this](){this->PerformListening();});
    }

    listener::~listener()
    {
        // Undeclare params
        this->undeclare_parameter("fixed_frame");
        this->undeclare_parameter("target_frame");
    }

    void listener::loadROS2Param()
    {
        // Declare params
        this->declare_parameter<std::string>("fixed_frame", "map");
        this->declare_parameter<std::string>("target_frame", "odom");
        // Obtain params
        this->get_parameter("fixed_frame", fixed_frame_);
        this->get_parameter("target_frame", target_frame_);
    }

    void listener::PerformListening()
    {
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

        RCLCPP_INFO_STREAM(
            this->get_logger(),
            this->get_name()
            << "listener_node: position x, y, z ("
            << read_transformation.transform.translation.x
            << ", "
            << read_transformation.transform.translation.y
            << ", "
            << read_transformation.transform.translation.z
            << ") x, y, z, w ("
            << read_transformation.transform.rotation.x 
            << ", "
            << read_transformation.transform.rotation.y
            << ", "
            << read_transformation.transform.rotation.z
            << ", "
            << read_transformation.transform.rotation.w
            << ")"
        );
    }
} // namespace tf