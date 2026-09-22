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
  declare_parameter<double>("avoid_edge_target_ratio", 0.33);  // 画面中央側のbbox端をこの割合まで端へ寄せる
  declare_parameter<double>("avoid_hold_sec", 1.5);            // ロボット消失後に回避方向で前進を続ける秒数
  declare_parameter<double>("avoid_pass_speed_ratio", 0.5);    // 通過中(寄せ完了後)の速度比(linear_speedに対する)
  declare_parameter<double>("avoid_smooth_alpha", 0.4);        // ロボットbbox端・面積の指数移動平均係数(0-1)。小さいほど滑らか
  declare_parameter<double>("avoid_done_margin_ratio", 0.05);  // 寄せ完了のヒステリシス幅(画面幅比)。完了後この分ずれるまで旋回に戻らない
  declare_parameter<double>("max_linear_accel", 0.5);          // 並進速度の変化率上限(m/s^2)
  declare_parameter<double>("max_angular_accel", 2.0);         // 角速度の変化率上限(rad/s^2)
  declare_parameter<double>("ground_lost_grace_sec", 0.7);     // 地面が見えなくなっても直前の指令を維持する猶予(秒)。超えたら停止
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
  const double edge_ratio = get_parameter("avoid_edge_target_ratio").as_double();
  const double hold_sec = get_parameter("avoid_hold_sec").as_double();
  const double pass_ratio = get_parameter("avoid_pass_speed_ratio").as_double();
  const double smooth_alpha = get_parameter("avoid_smooth_alpha").as_double();
  const double done_margin_px = get_parameter("avoid_done_margin_ratio").as_double() * img_w;
  const double max_lin_acc = get_parameter("max_linear_accel").as_double();
  const double max_ang_acc = get_parameter("max_angular_accel").as_double();
  const double ground_grace = get_parameter("ground_lost_grace_sec").as_double();

  // 地面/ロボットのトラックをラベルで検索（先頭一致のみ使用）
  const lang_sam_msgs::msg::Track *ground_track = nullptr;
  const lang_sam_msgs::msg::Track *robot_track = nullptr;
  for (const auto &t : msg->tracks) {
    if (!ground_track && labelContains(t.label, ground_label)) ground_track = &t;
    if (!robot_track && labelContains(t.label, robot_label)) robot_track = &t;
  }

  const auto clamp_ang = [max_ang](double v) { return std::max(-max_ang, std::min(max_ang, v)); };
  const rclcpp::Time now = this->now();

  double robot_area_ratio = 0.0;
  if (robot_track) {
    robot_area_ratio =
      static_cast<double>((robot_track->x_max - robot_track->x_min) *
                           (robot_track->y_max - robot_track->y_min)) /
      static_cast<double>(img_w * img_h);
  }

  // ロボットbbox端・面積を指数移動平均で平滑化(ゆらぎが旋回/直進の切替や速度に出るのを防ぐ)
  if (robot_track) {
    if (!sm_valid_) {
      sm_x_min_ = robot_track->x_min;
      sm_x_max_ = robot_track->x_max;
      sm_area_ = robot_area_ratio;
      sm_valid_ = true;
    } else {
      sm_x_min_ += smooth_alpha * (robot_track->x_min - sm_x_min_);
      sm_x_max_ += smooth_alpha * (robot_track->x_max - sm_x_max_);
      sm_area_ += smooth_alpha * (robot_area_ratio - sm_area_);
    }
  }

  // 状態遷移
  // - IDLE中: ロボットが十分大きく映ったら回避開始(方向は開始時にラッチ)
  // - AVOID中: ロボットが見えている間は継続。見えなくなったらHOLDへ
  // - HOLD中: avoid_hold_sec経過でIDLEへ。再度見えたらAVOIDへ戻る(方向は保持)
  if (avoid_state_ == AvoidState::IDLE) {
    if (robot_track && robot_area_ratio >= area_slow) {
      // 空きが大きい側へ避ける（右に空きがあれば右へ旋回=ロボットを左へ寄せる）
      const int free_left = robot_track->x_min;
      const int free_right = img_w - robot_track->x_max;
      avoid_dir_ = (free_right >= free_left) ? -1 : +1;  // -1:右旋回, +1:左旋回
      avoid_state_ = AvoidState::AVOID;
      avoid_done_ = false;
    } else if (!robot_track) {
      sm_valid_ = false;
    }
  } else if (avoid_state_ == AvoidState::AVOID) {
    if (!robot_track) {
      avoid_state_ = AvoidState::HOLD;
      hold_start_ = now;
    }
  } else {  // HOLD
    if (robot_track && robot_area_ratio >= area_slow) {
      avoid_state_ = AvoidState::AVOID;
    } else if ((now - hold_start_).seconds() >= hold_sec) {
      avoid_state_ = AvoidState::IDLE;
      sm_valid_ = false;
    }
  }

  geometry_msgs::msg::Twist cmd;
  bool safety_stop = false;

  if (avoid_state_ == AvoidState::AVOID) {
    // 中央側のbbox端が目標位置になるまで旋回し、寄せ終えたら減速して前進(通過)
    // 右旋回(-1): ロボットが左へ寄り、右端x_maxが img_w*edge_ratio 以下になれば完了
    // 左旋回(+1): ロボットが右へ寄り、左端x_minが img_w*(1-edge_ratio) 以上になれば完了
    double err_px;  // 未達成なら正
    if (avoid_dir_ < 0) {
      err_px = sm_x_max_ - img_w * edge_ratio;
    } else {
      err_px = img_w * (1.0 - edge_ratio) - sm_x_min_;
    }

    // ヒステリシス: 完了後は margin 分ずれるまで旋回に戻らない(境界付近での切替振動を防ぐ)
    if (!avoid_done_ && err_px <= 0.0) {
      avoid_done_ = true;
    } else if (avoid_done_ && err_px > done_margin_px) {
      avoid_done_ = false;
    }

    if (!avoid_done_) {
      // 寄せ動作中: 旋回しつつ、面積比に応じて連続的に減速(area_stopで0)
      cmd.angular.z = clamp_ang(avoid_dir_ * kp_avoid * std::max(err_px, 0.0));
      const double ratio =
        std::min(1.0, std::max(0.0, (sm_area_ - area_slow) / (area_stop - area_slow)));
      cmd.linear.x = linear_speed * (1.0 - ratio);
    } else {
      // 寄せ完了: 向きを保持して減速前進
      cmd.angular.z = 0.0;
      cmd.linear.x = linear_speed * pass_ratio;
    }
  } else if (avoid_state_ == AvoidState::HOLD) {
    // ロボットが画角から消えた直後: 回避方向の向きを保持して前進し、追い越し切る
    cmd.angular.z = 0.0;
    cmd.linear.x = linear_speed * pass_ratio;
  } else if (ground_track) {
    // 地面のみ検出: bbox中心を画面中心に合わせるよう操舵しつつ直進
    const double cx = (ground_track->x_min + ground_track->x_max) * 0.5;
    const double dx = cx - img_w * 0.5;
    cmd.angular.z = clamp_ang(-kp_ground * dx);
    cmd.linear.x = linear_speed;
    last_ground_time_ = now;
    ground_seen_ = true;
  } else if (ground_seen_ && (now - last_ground_time_).seconds() < ground_grace) {
    // 地面が一瞬見えなくなっただけ(再検出の取りこぼし等): 直前の指令を維持して止まらない
    cmd.linear.x = prev_lin_;
    cmd.angular.z = prev_ang_;
  } else {
    // 地面もロボットも一定時間見えない場合は安全のため停止
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    safety_stop = true;
  }

  // 指令のスルーレート制限(急激な速度変化によるガタつきを抑える)。安全停止は即時反映
  const double dt = std::min(0.5, std::max(0.0, (now - last_cmd_time_).seconds()));
  if (!safety_stop && last_cmd_valid_) {
    const double dl = max_lin_acc * dt;
    const double da = max_ang_acc * dt;
    cmd.linear.x = std::max(prev_lin_ - dl, std::min(prev_lin_ + dl, cmd.linear.x));
    cmd.angular.z = std::max(prev_ang_ - da, std::min(prev_ang_ + da, cmd.angular.z));
  }
  prev_lin_ = cmd.linear.x;
  prev_ang_ = cmd.angular.z;
  last_cmd_time_ = now;
  last_cmd_valid_ = true;

  cmd_pub_->publish(cmd);
}
