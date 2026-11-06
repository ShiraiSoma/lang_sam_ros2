#include <rclcpp/rclcpp.hpp>
#include "lang_sam_ground_nav/lang_sam_ground_nav_node.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GroundNavNode>());
  rclcpp::shutdown();
  return 0;
}
