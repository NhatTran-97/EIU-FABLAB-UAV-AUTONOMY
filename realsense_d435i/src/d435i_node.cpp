#include "realsense_d435i/d435i_driver.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<D435iDriver>());
    rclcpp::shutdown();
    return 0;
}
