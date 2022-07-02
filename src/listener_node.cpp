#include "ros2-tf/listener.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<tf::listener>());
    rclcpp::shutdown();
    return 0;
}