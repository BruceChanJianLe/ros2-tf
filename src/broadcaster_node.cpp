#include "ros2-tf/broadcaster.hpp"


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<tf::broadcaster>(argv));
    rclcpp::shutdown();
    return 0;
}