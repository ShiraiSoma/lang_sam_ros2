#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <lang_sam_msgs/msg/track_array.hpp>
#include <lang_sam_msgs/msg/track.hpp>

class GroundNavNode : public rclcpp::Node
{
public:
  GroundNavNode();

private:
  void tracksCallback(const lang_sam_msgs::msg::TrackArray::SharedPtr msg);

  // labelにkeyword(小文字)が含まれるか大小文字を無視して判定
  static bool labelContains(const std::string &label, const std::string &keyword);

  // 回避状態: IDLE=地面追従 / AVOID=ロボットを画面端へ寄せて通過 / HOLD=消失後に向きを保持して前進
  enum class AvoidState { IDLE, AVOID, HOLD };
  AvoidState avoid_state_{AvoidState::IDLE};
  int avoid_dir_{0};            // 開始時にラッチした旋回方向 (-1:右旋回, +1:左旋回)
  rclcpp::Time hold_start_{0, 0, RCL_ROS_TIME};
  bool avoid_done_{false};      // 寄せ完了(ヒステリシス付き)

  // ロボットbboxの平滑化値
  bool sm_valid_{false};
  double sm_x_min_{0.0}, sm_x_max_{0.0}, sm_area_{0.0};

  // スルーレート制限用の前回指令
  bool last_cmd_valid_{false};
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  double prev_lin_{0.0}, prev_ang_{0.0};

  // 地面を最後に見た時刻(見失い猶予用)
  bool ground_seen_{false};
  rclcpp::Time last_ground_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<lang_sam_msgs::msg::TrackArray>::SharedPtr tracks_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
};
