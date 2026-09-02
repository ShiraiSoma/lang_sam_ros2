#include "lang_sam_ground_nav/lang_sam_ground_nav_node.hpp"
#include <algorithm>
#include <cmath>

GroundNavNode::GroundNavNode()
: rclcpp::Node("lang_sam_ground_nav_node")
{
  using std::placeholders::_1;
  tracks_sub_ = create_subscription<lang_sam_msgs::msg::TrackArray>(
    "/lang_sam/tracks", 1, std::bind(&GroundNavNode::tracksCallback, this, _1));
  cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);

  declare_parameter<int>("image_width", 640);
  declare_parameter<int>("image_height", 480);
  declare_parameter<std::string>("ground_label", "brown ground");
  declare_parameter<std::string>("robot_label", "robot");
  declare_parameter<double>("linear_speed", 0.2);
  declare_parameter<double>("kp_angular_ground", 0.002);
  declare_parameter<double>("kp_angular_avoid", 0.002);
  declare_parameter<double>("avoid_area_ratio_slow", 0.05);
  declare_parameter<double>("avoid_area_ratio_stop", 0.15);
  declare_parameter<double>("max_angular_z", 0.6);  // 角速度の上限(rad/s)。地面追従・回避の両方に適用
}

bool GroundNavNode::labelContains(const std::string &label, const std::string &keyword)
{
  auto to_lower = [](std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::tolower(c); });
    return s;
  };
  const std::string l = to_lower(label);
  const std::string k = to_lower(keyword);
  return !k.empty() && l.find(k) != std::string::npos;
}

void GroundNavNode::tracksCallback(const lang_sam_msgs::msg::TrackArray::SharedPtr msg)
{
  const int img_w = get_parameter("image_width").as_int();
  const int img_h = get_parameter("image_height").as_int();
  const std::string ground_label = get_parameter("ground_label").as_string();
  const std::string robot_label = get_parameter("robot_label").as_string();
  const double linear_speed = get_parameter("linear_speed").as_double();
  const double kp_ground = get_parameter("kp_angular_ground").as_double();
  const double kp_avoid = get_parameter("kp_angular_avoid").as_double();
  const double area_slow = get_parameter("avoid_area_ratio_slow").as_double();
  const double area_stop = get_parameter("avoid_area_ratio_stop").as_double();
  const double max_ang = get_parameter("max_angular_z").as_double();

  // 地面/ロボットのトラックをラベルで検索（先頭一致のみ使用）
  const lang_sam_msgs::msg::Track *ground_track = nullptr;
  const lang_sam_msgs::msg::Track *robot_track = nullptr;
  for (const auto &t : msg->tracks) {
    if (!ground_track && labelContains(t.label, ground_label)) ground_track = &t;
    if (!robot_track && labelContains(t.label, robot_label)) robot_track = &t;
  }

  geometry_msgs::msg::Twist cmd;

  // ロボットのbbox面積比を先に評価し、avoid_area_ratio_slow未満(=遠くて小さい)は
  // 回避対象とせず無視する(地面追従を優先)
  double robot_area_ratio = 0.0;
  if (robot_track) {
    robot_area_ratio =
      static_cast<double>((robot_track->x_max - robot_track->x_min) *
                           (robot_track->y_max - robot_track->y_min)) /
      static_cast<double>(img_w * img_h);
  }
  const bool avoid_active = robot_track && robot_area_ratio >= area_slow;

  if (avoid_active) {
    // ロボットが十分大きく映っている: 逆方向へ操舵しつつ、近いほど減速・至近距離は停止して回頭のみ
    const double cx = (robot_track->x_min + robot_track->x_max) * 0.5;
    const double dx = cx - img_w * 0.5;

    // ロボットが画面右にいれば左へ、左にいれば右へ（追従則と逆符号）
    cmd.angular.z = kp_avoid * dx;
    if (std::abs(cmd.angular.z) > max_ang) cmd.angular.z = (cmd.angular.z > 0 ? max_ang : -max_ang);

    if (robot_area_ratio >= area_stop) {
      cmd.linear.x = 0.0;
    } else {
      // area_slow <= robot_area_ratio < area_stop の範囲で線形に減速
      const double ratio = (robot_area_ratio - area_slow) / (area_stop - area_slow);
      cmd.linear.x = linear_speed * (1.0 - ratio);
    }
  } else if (ground_track) {
    // 地面のみ検出: bbox中心を画面中心に合わせるよう操舵しつつ直進
    const double cx = (ground_track->x_min + ground_track->x_max) * 0.5;
    const double dx = cx - img_w * 0.5;
    cmd.angular.z = -kp_ground * dx;
    if (std::abs(cmd.angular.z) > max_ang) cmd.angular.z = (cmd.angular.z > 0 ? max_ang : -max_ang);
    cmd.linear.x = linear_speed;
  } else {
    // 地面もロボットも見えない場合は安全のため停止
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
  }

  cmd_pub_->publish(cmd);
}
