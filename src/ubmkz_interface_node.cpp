#include "ubmkz_interface.hpp"
#include <rclcpp/rclcpp.hpp>

#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ds_dbw_msg::UbmkzInterface>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}