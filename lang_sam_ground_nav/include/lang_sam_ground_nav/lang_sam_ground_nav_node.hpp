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

  rclcpp::Subscription<lang_sam_msgs::msg::TrackArray>::SharedPtr tracks_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
};
