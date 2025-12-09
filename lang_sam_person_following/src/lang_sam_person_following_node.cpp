#include "lang_sam_person_following/lang_sam_person_following_node.hpp"
#include <cmath>

FollowPersonNode::FollowPersonNode()
: rclcpp::Node("lang_sam_person_following_node")
{
  using std::placeholders::_1;
  tracks_sub_ = create_subscription<lang_sam_msgs::msg::TrackArray>(
    "/lang_sam/tracks", 1, std::bind(&FollowPersonNode::tracksCallback, this, _1));
  cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);

  declare_parameter<int>("image_width", 1280);
  declare_parameter<double>("kp_angular", 0.002);
}

void FollowPersonNode::tracksCallback(const lang_sam_msgs::msg::TrackArray::SharedPtr msg)
{
  int img_w = get_parameter("image_width").as_int();
  double kp_ang = get_parameter("kp_angular").as_double();

  // BBoxは一つと仮定（先頭のみ使用）
  if (msg->tracks.empty()) return;
  const auto &t = msg->tracks.front();

  // バウンディングボックス中心（横方向のみ使用）
  double cx = (t.x_min + t.x_max) * 0.5;

  // 画面中心との偏差
  double dx = (cx - img_w * 0.5);

  geometry_msgs::msg::Twist cmd;
  // 並進速度を常に一定(0.2 m/s)
  cmd.linear.x = 0.2;
  // 角速度は横方向偏差に比例
  cmd.angular.z = -kp_ang * dx;

  // 角速度クリップのみ維持
  if (std::abs(cmd.angular.z) > 1.0) cmd.angular.z = 1.0 * (cmd.angular.z > 0 ? 1 : -1);

  cmd_pub_->publish(cmd);
}
