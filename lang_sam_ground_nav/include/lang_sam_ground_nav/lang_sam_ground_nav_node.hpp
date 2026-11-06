#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <lang_sam_msgs/msg/track_array.hpp>
#include <lang_sam_msgs/msg/track.hpp>
#include <opencv2/core.hpp>

class GroundNavNode : public rclcpp::Node
{
public:
  GroundNavNode();

private:
  void tracksCallback(const lang_sam_msgs::msg::TrackArray::SharedPtr msg);
  void trackMaskCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
  void watchdogCallback();

  // 同じstampのtracksとtrack_maskが揃ったら(mask_bottomモード時)処理する
  void tryProcessPair();
  // 1フレーム分の指令を計算してpublishする。idx_maskはmask_bottomモード時のみ使用(空ならbbox中心にフォールバック)
  void process(const lang_sam_msgs::msg::TrackArray &msg, const cv::Mat &idx_mask);

  // labelにkeyword(小文字)が含まれるか大小文字を無視して判定
  static bool labelContains(const std::string &label, const std::string &keyword);
  // keywordを含むトラックのうちbbox面積が最大のものを返す(無ければnullptr)
  static const lang_sam_msgs::msg::Track * findLargest(
    const lang_sam_msgs::msg::TrackArray &msg, const std::string &keyword);
  // 地面の追従目標x座標を求める(ground_target_modeに応じてbbox中心 or マスク下部の重心)
  double groundTargetX(const lang_sam_msgs::msg::Track &ground, const cv::Mat &idx_mask) const;

  // 回避状態: IDLE=地面追従 / AVOID=障害物を画面端へ寄せて通過 / HOLD=消失後に向きを保持して前進
  enum class AvoidState { IDLE, AVOID, HOLD };
  AvoidState avoid_state_{AvoidState::IDLE};
  int avoid_dir_{0};            // 開始時にラッチした旋回方向 (-1:右旋回, +1:左旋回)
  rclcpp::Time hold_start_{0, 0, RCL_ROS_TIME};
  bool avoid_done_{false};      // 寄せ完了(ヒステリシス付き)

  // 障害物bboxの平滑化値
  bool sm_valid_{false};
  double sm_x_min_{0.0}, sm_x_max_{0.0}, sm_area_{0.0};

  // スルーレート制限用の前回指令
  bool last_cmd_valid_{false};
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  double prev_lin_{0.0}, prev_ang_{0.0};

  // 地面を最後に見た時刻(見失い猶予用)
  bool ground_seen_{false};
  rclcpp::Time last_ground_time_{0, 0, RCL_ROS_TIME};

  // mask_bottomモード用: tracksとtrack_maskの対応待ち(どちらも最新1件のみ保持)
  lang_sam_msgs::msg::TrackArray::SharedPtr pending_tracks_;
  sensor_msgs::msg::CompressedImage::SharedPtr pending_mask_;

  // tracksが途絶えたときの停止用(通信断・tracker停止対策)
  rclcpp::Time last_tracks_time_{0, 0, RCL_ROS_TIME};
  bool tracks_seen_{false};

  rclcpp::Subscription<lang_sam_msgs::msg::TrackArray>::SharedPtr tracks_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr track_mask_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
};
