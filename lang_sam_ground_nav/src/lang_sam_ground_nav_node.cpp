#include "lang_sam_ground_nav/lang_sam_ground_nav_node.hpp"
#include <algorithm>
#include <cmath>
#include <opencv2/imgcodecs.hpp>

GroundNavNode::GroundNavNode()
: rclcpp::Node("lang_sam_ground_nav_node")
{
  using std::placeholders::_1;

  declare_parameter<int>("image_width", 640);
  declare_parameter<int>("image_height", 480);
  declare_parameter<std::string>("ground_label", "red ground");
  declare_parameter<std::string>("obstacle_label", "green pylon");
  // 地面の追従目標: "bbox_center"=bbox中心 / "mask_bottom"=マスク下部(ロボットに近い側)の重心
  declare_parameter<std::string>("ground_target_mode", "bbox_center");
  declare_parameter<double>("ground_bottom_ratio", 0.3);       // mask_bottom: 地面bboxの下からこの割合の高さの帯を使う
  declare_parameter<int>("ground_bottom_min_pixels", 200);     // mask_bottom: 帯内の画素がこれ未満ならbbox中心にフォールバック
  declare_parameter<double>("linear_speed", 0.2);
  declare_parameter<double>("kp_angular_ground", 0.002);
  declare_parameter<double>("kp_angular_avoid", 0.002);
  declare_parameter<double>("avoid_area_ratio_slow", 0.05);
  declare_parameter<double>("avoid_area_ratio_stop", 0.15);
  declare_parameter<double>("max_angular_z", 0.6);  // 角速度の上限(rad/s)。地面追従・回避の両方に適用
  declare_parameter<double>("avoid_edge_target_ratio", 0.33);  // 画面中央側のbbox端をこの割合まで端へ寄せる
  declare_parameter<double>("avoid_hold_sec", 1.5);            // 障害物消失後に回避方向で前進を続ける秒数
  declare_parameter<double>("avoid_pass_speed_ratio", 0.5);    // 通過中(寄せ完了後)の速度比(linear_speedに対する)
  declare_parameter<double>("avoid_smooth_alpha", 0.4);        // 障害物bbox端・面積の指数移動平均係数(0-1)。小さいほど滑らか
  declare_parameter<double>("avoid_done_margin_ratio", 0.05);  // 寄せ完了のヒステリシス幅(画面幅比)。完了後この分ずれるまで旋回に戻らない
  declare_parameter<double>("max_linear_accel", 0.5);          // 並進速度の変化率上限(m/s^2)
  declare_parameter<double>("max_angular_accel", 2.0);         // 角速度の変化率上限(rad/s^2)
  declare_parameter<double>("ground_lost_grace_sec", 0.7);     // 地面が見えなくなっても直前の指令を維持する猶予(秒)。超えたら停止
  declare_parameter<double>("tracks_timeout_sec", 0.5);        // tracksがこの秒数届かなければ停止指令を出す(0以下で無効)

  tracks_sub_ = create_subscription<lang_sam_msgs::msg::TrackArray>(
    "/lang_sam/tracks", 1, std::bind(&GroundNavNode::tracksCallback, this, _1));
  track_mask_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
    "/lang_sam/track_mask", 1, std::bind(&GroundNavNode::trackMaskCallback, this, _1));
  cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
  watchdog_timer_ = create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&GroundNavNode::watchdogCallback, this));

  RCLCPP_INFO(
    get_logger(), "ground_label='%s' obstacle_label='%s' ground_target_mode=%s",
    get_parameter("ground_label").as_string().c_str(),
    get_parameter("obstacle_label").as_string().c_str(),
    get_parameter("ground_target_mode").as_string().c_str());
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

const lang_sam_msgs::msg::Track * GroundNavNode::findLargest(
  const lang_sam_msgs::msg::TrackArray &msg, const std::string &keyword)
{
  const lang_sam_msgs::msg::Track *best = nullptr;
  long best_area = -1;
  for (const auto &t : msg.tracks) {
    if (!labelContains(t.label, keyword)) continue;
    const long area = static_cast<long>(t.x_max - t.x_min) * static_cast<long>(t.y_max - t.y_min);
    if (area > best_area) {
      best = &t;
      best_area = area;
    }
  }
  return best;
}

double GroundNavNode::groundTargetX(
  const lang_sam_msgs::msg::Track &ground, const cv::Mat &idx_mask) const
{
  const double bbox_cx = (ground.x_min + ground.x_max) * 0.5;
  if (get_parameter("ground_target_mode").as_string() != "mask_bottom" || idx_mask.empty()) {
    return bbox_cx;
  }

  // 地面bboxの下側 ground_bottom_ratio の帯に含まれる地面画素のx重心
  const double ratio = std::min(1.0, std::max(0.0, get_parameter("ground_bottom_ratio").as_double()));
  const int min_px = get_parameter("ground_bottom_min_pixels").as_int();
  const int y_max = std::min(ground.y_max, idx_mask.rows - 1);
  const int y_min = std::max(0, static_cast<int>(std::lround(y_max - ratio * (y_max - ground.y_min))));
  const int x_min = std::max(0, ground.x_min);
  const int x_max = std::min(ground.x_max, idx_mask.cols - 1);
  const uint16_t gid = static_cast<uint16_t>(ground.id);

  double sum_x = 0.0;
  long count = 0;
  for (int y = y_min; y <= y_max; ++y) {
    const uint16_t *row = idx_mask.ptr<uint16_t>(y);
    for (int x = x_min; x <= x_max; ++x) {
      if (row[x] == gid) {
        sum_x += x;
        ++count;
      }
    }
  }
  return (count >= min_px) ? sum_x / count : bbox_cx;
}

void GroundNavNode::tracksCallback(const lang_sam_msgs::msg::TrackArray::SharedPtr msg)
{
  last_tracks_time_ = now();
  tracks_seen_ = true;
  if (get_parameter("ground_target_mode").as_string() == "mask_bottom") {
    pending_tracks_ = msg;
    tryProcessPair();
  } else {
    process(*msg, cv::Mat());
  }
}

void GroundNavNode::trackMaskCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
  if (get_parameter("ground_target_mode").as_string() != "mask_bottom") return;
  pending_mask_ = msg;
  tryProcessPair();
}

void GroundNavNode::tryProcessPair()
{
  if (!pending_tracks_ || !pending_mask_) return;
  const auto &ts = pending_tracks_->header.stamp;
  const auto &ms = pending_mask_->header.stamp;
  if (ts.sec != ms.sec || ts.nanosec != ms.nanosec) {
    // 片方が先行している。古い方は対応する相手が来ないので、新しい方を残して待つ
    const bool tracks_newer = (ts.sec > ms.sec) || (ts.sec == ms.sec && ts.nanosec > ms.nanosec);
    if (tracks_newer) pending_mask_.reset(); else pending_tracks_.reset();
    return;
  }
  cv::Mat idx_mask = cv::imdecode(
    cv::Mat(1, static_cast<int>(pending_mask_->data.size()), CV_8UC1,
            const_cast<uint8_t *>(pending_mask_->data.data())),
    cv::IMREAD_UNCHANGED);
  if (idx_mask.empty() || idx_mask.type() != CV_16UC1) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "track_maskのデコードに失敗(16bit PNGではない)。bbox中心で代用します");
    idx_mask.release();
  }
  process(*pending_tracks_, idx_mask);
  pending_tracks_.reset();
  pending_mask_.reset();
}

void GroundNavNode::watchdogCallback()
{
  const double timeout = get_parameter("tracks_timeout_sec").as_double();
  if (timeout <= 0.0 || !tracks_seen_) return;
  if ((now() - last_tracks_time_).seconds() < timeout) return;
  if (prev_lin_ == 0.0 && prev_ang_ == 0.0 && last_cmd_valid_) return;  // 停止済み

  // tracksが途絶えた(通信断・tracker停止): 直前の指令が残り続けないよう即時停止
  RCLCPP_WARN(get_logger(), "tracksが%.1f秒以上届かないため停止指令を出します", timeout);
  geometry_msgs::msg::Twist stop;
  cmd_pub_->publish(stop);
  prev_lin_ = 0.0;
  prev_ang_ = 0.0;
  last_cmd_time_ = now();
  last_cmd_valid_ = true;
  avoid_state_ = AvoidState::IDLE;
  sm_valid_ = false;
}

void GroundNavNode::process(const lang_sam_msgs::msg::TrackArray &msg, const cv::Mat &idx_mask)
{
  const int img_w = get_parameter("image_width").as_int();
  const int img_h = get_parameter("image_height").as_int();
  const std::string ground_label = get_parameter("ground_label").as_string();
  const std::string obstacle_label = get_parameter("obstacle_label").as_string();
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

  // 地面/障害物のトラックをラベルで検索(複数あればbbox面積が最大のもの)
  const lang_sam_msgs::msg::Track *ground_track = findLargest(msg, ground_label);
  const lang_sam_msgs::msg::Track *obstacle_track = findLargest(msg, obstacle_label);

  const auto clamp_ang = [max_ang](double v) { return std::max(-max_ang, std::min(max_ang, v)); };
  const rclcpp::Time now = this->now();

  double obstacle_area_ratio = 0.0;
  if (obstacle_track) {
    obstacle_area_ratio =
      static_cast<double>((obstacle_track->x_max - obstacle_track->x_min) *
                           (obstacle_track->y_max - obstacle_track->y_min)) /
      static_cast<double>(img_w * img_h);
  }

  // 障害物bbox端・面積を指数移動平均で平滑化(ゆらぎが旋回/直進の切替や速度に出るのを防ぐ)
  if (obstacle_track) {
    if (!sm_valid_) {
      sm_x_min_ = obstacle_track->x_min;
      sm_x_max_ = obstacle_track->x_max;
      sm_area_ = obstacle_area_ratio;
      sm_valid_ = true;
    } else {
      sm_x_min_ += smooth_alpha * (obstacle_track->x_min - sm_x_min_);
      sm_x_max_ += smooth_alpha * (obstacle_track->x_max - sm_x_max_);
      sm_area_ += smooth_alpha * (obstacle_area_ratio - sm_area_);
    }
  }

  // 状態遷移
  // - IDLE中: 障害物が十分大きく映ったら回避開始(方向は開始時にラッチ)
  // - AVOID中: 障害物が見えている間は継続。見えなくなったらHOLDへ
  // - HOLD中: avoid_hold_sec経過でIDLEへ。再度見えたらAVOIDへ戻る(方向は保持)
  if (avoid_state_ == AvoidState::IDLE) {
    if (obstacle_track && obstacle_area_ratio >= area_slow) {
      // 空きが大きい側へ避ける(右に空きがあれば右へ旋回=障害物を左へ寄せる)
      const int free_left = obstacle_track->x_min;
      const int free_right = img_w - obstacle_track->x_max;
      avoid_dir_ = (free_right >= free_left) ? -1 : +1;  // -1:右旋回, +1:左旋回
      avoid_state_ = AvoidState::AVOID;
      avoid_done_ = false;
    } else if (!obstacle_track) {
      sm_valid_ = false;
    }
  } else if (avoid_state_ == AvoidState::AVOID) {
    if (!obstacle_track) {
      avoid_state_ = AvoidState::HOLD;
      hold_start_ = now;
    }
  } else {  // HOLD
    if (obstacle_track && obstacle_area_ratio >= area_slow) {
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
    // 右旋回(-1): 障害物が左へ寄り、右端x_maxが img_w*edge_ratio 以下になれば完了
    // 左旋回(+1): 障害物が右へ寄り、左端x_minが img_w*(1-edge_ratio) 以上になれば完了
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
    // 障害物が画角から消えた直後: 回避方向の向きを保持して前進し、追い越し切る
    cmd.angular.z = 0.0;
    cmd.linear.x = linear_speed * pass_ratio;
  } else if (ground_track) {
    // 地面のみ検出: 追従目標(bbox中心 or マスク下部の重心)を画面中心に合わせるよう操舵しつつ直進
    const double dx = groundTargetX(*ground_track, idx_mask) - img_w * 0.5;
    cmd.angular.z = clamp_ang(-kp_ground * dx);
    cmd.linear.x = linear_speed;
    last_ground_time_ = now;
    ground_seen_ = true;
  } else if (ground_seen_ && (now - last_ground_time_).seconds() < ground_grace) {
    // 地面が一瞬見えなくなっただけ(再検出の取りこぼし等): 直前の指令を維持して止まらない
    cmd.linear.x = prev_lin_;
    cmd.angular.z = prev_ang_;
  } else {
    // 地面も障害物も一定時間見えない場合は安全のため停止
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
