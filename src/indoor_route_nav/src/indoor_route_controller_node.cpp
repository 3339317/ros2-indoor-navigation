#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <nlohmann/json.hpp>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace indoor_route_nav
{

namespace
{

struct Point
{
  double x{};
  double y{};
  double z{};
};

double normalizeAngleDeg(double deg)
{
  while (deg > 180.0) {
    deg -= 360.0;
  }
  while (deg < -180.0) {
    deg += 360.0;
  }
  return deg;
}

double clampAbs(double value, double max_abs, double min_abs)
{
  const double sign = value >= 0.0 ? 1.0 : -1.0;
  const double mag = std::abs(value);
  if (mag < 1e-9) {
    return 0.0;
  }
  return sign * std::min(max_abs, std::max(min_abs, mag));
}

double dist3d(const Point & a, const Point & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double dist2d(const Point & a, const Point & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

double yawDegFromQuat(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp) * 180.0 / M_PI;
}

}  // namespace

class ControllerNode : public rclcpp::Node
{
public:
  ControllerNode()
  : Node("indoor_route_controller_node")
  {
    pose_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/localization", 10,
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        pose_.x = msg->pose.pose.position.x;
        pose_.y = msg->pose.pose.position.y;
        pose_.z = msg->pose.pose.position.z;
        yaw_deg_ = yawDegFromQuat(msg->pose.pose.orientation);
        localized_ = true;
      });

    lidar_topic_ = declare_parameter<std::string>("lidar_topic", "/livox/lidar");
    lidar_height_ = declare_parameter("lidar_height", 1.10);
    ground_z_tolerance_ = declare_parameter("ground_z_tolerance", 0.12);
    max_slope_ = declare_parameter("max_slope", 0.65);
    obstacle_avoidance_enabled_ = declare_parameter("obstacle_avoidance_enabled", false);

    obstacle_sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
      lidar_topic_, 10,
      [this](livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
        cloud_callback(msg);
      });

    route_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/indoor_route_nav/controller/route", 10,
      [this](nav_msgs::msg::Path::SharedPtr msg) {
        setRoute(*msg);
      });

    config_sub_ = create_subscription<std_msgs::msg::String>(
      "/indoor_route_nav/controller/config", 10,
      [this](std_msgs::msg::String::SharedPtr msg) {
        applyConfig(msg->data);
      });

    start_sub_ = create_subscription<std_msgs::msg::Empty>(
      "/indoor_route_nav/controller/start", 10,
      [this](std_msgs::msg::Empty::SharedPtr) {
        startNavigation();
      });

    stop_sub_ = create_subscription<std_msgs::msg::Empty>(
      "/indoor_route_nav/controller/stop", 10,
      [this](std_msgs::msg::Empty::SharedPtr) {
        stopNavigation("导航已停止");
      });

    clear_sub_ = create_subscription<std_msgs::msg::Empty>(
      "/indoor_route_nav/controller/clear", 10,
      [this](std_msgs::msg::Empty::SharedPtr) {
        route_.clear();
        route_cumlen_.clear();
        stopNavigation("已清空控制器路线");
      });

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    done_pub_ = create_publisher<std_msgs::msg::String>("/nav/done", 10);
    state_pub_ = create_publisher<std_msgs::msg::String>("/indoor_route_nav/controller/state", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("/indoor_route_nav/controller/status", 10);

    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {controlLoop();});
    publishStatus("C++ 路径跟踪控制器已启动");
  }

private:
  void setRoute(const nav_msgs::msg::Path & path)
  {
    route_.clear();
    route_.reserve(path.poses.size());
    for (const auto & pose : path.poses) {
      route_.push_back(Point{
        pose.pose.position.x,
        pose.pose.position.y,
        pose.pose.position.z});
    }

    route_cumlen_.clear();
    if (!route_.empty()) {
      route_cumlen_.push_back(0.0);
      for (std::size_t i = 1; i < route_.size(); ++i) {
        route_cumlen_.push_back(route_cumlen_.back() + dist3d(route_[i], route_[i - 1]));
      }
    }

    buildRouteSegmentIndex();

    if (route_.size() >= 2 && !final_yaw_configured_) {
      final_target_yaw_ = routeHeadingDeg(route_.size() - 1);
    }
    nav_progress_idx_ = 0;
    lookahead_target_ = Point{};
    has_lookahead_ = false;
    is_obstacle_on_path_ = false;
    obstacle_boxes_.clear();
    obstacle_clusters_.clear();
    obstacle_min_distance_ = 999.0;
    publishStatus("控制器已接收路线，点数 " + std::to_string(route_.size()));
  }

  static std::int64_t gridKey(int ix, int iy)
  {
    return (static_cast<std::int64_t>(ix) << 32) | static_cast<std::uint32_t>(iy);
  }

  void buildRouteSegmentIndex()
  {
    route_segment_index_.clear();
    if (route_.size() < 2) return;

    const double cell_size = 0.8;
    const double inv_cell = 1.0 / cell_size;

    for (std::size_t seg_idx = 0; seg_idx + 1 < route_.size(); ++seg_idx) {
      const auto & a = route_[seg_idx];
      const auto & b = route_[seg_idx + 1];

      int cx0 = static_cast<int>(std::floor(a.x * inv_cell));
      int cy0 = static_cast<int>(std::floor(a.y * inv_cell));
      int cx1 = static_cast<int>(std::floor(b.x * inv_cell));
      int cy1 = static_cast<int>(std::floor(b.y * inv_cell));

      int steps = std::max(std::abs(cx1 - cx0), std::abs(cy1 - cy0));
      if (steps <= 0) steps = 1;

      for (int s = 0; s <= steps; ++s) {
        double t = static_cast<double>(s) / static_cast<double>(steps);
        int cx = static_cast<int>(std::floor((a.x + t * (b.x - a.x)) * inv_cell));
        int cy = static_cast<int>(std::floor((a.y + t * (b.y - a.y)) * inv_cell));
        route_segment_index_[gridKey(cx, cy)].push_back(seg_idx);
      }
    }
  }

  void applyConfig(const std::string & text)
  {
    try {
      const auto cfg = nlohmann::json::parse(text);
      max_vx_ = std::max(0.01, cfg.value("max_linear_x", max_vx_));
      max_wz_ = std::max(0.01, cfg.value("max_angular_z", max_wz_));
      min_vx_ = std::max(0.0, cfg.value("min_linear_x", min_vx_));
      min_wz_ = std::max(0.0, cfg.value("min_angular_z", min_wz_));
      arrival_dist_ = std::max(0.01, cfg.value("arrival_distance", arrival_dist_));
      arrival_angle_ = std::max(0.1, cfg.value("arrival_angle_deg", arrival_angle_));
      alpha_ = std::min(1.0, std::max(0.001, cfg.value("alpha", alpha_)));
      obstacle_threshold_ = std::max(0.01, cfg.value("obstacle_threshold", obstacle_threshold_));
      resume_delay_ = std::max(0.0, cfg.value("resume_delay", resume_delay_));
      deceleration_alpha_ = std::min(1.0, std::max(0.001, cfg.value("deceleration_alpha", deceleration_alpha_)));
      lookahead_distance_ = std::max(0.05, cfg.value("lookahead_distance", lookahead_distance_));
      path_yaw_kp_ = std::max(0.0001, cfg.value("path_yaw_kp", path_yaw_kp_));
      final_yaw_kp_ = std::max(0.0001, cfg.value("final_yaw_kp", final_yaw_kp_));
      end_slowdown_distance_ = std::max(0.05, cfg.value("end_slowdown_distance", end_slowdown_distance_));
      heading_slow_angle_deg_ = std::max(1.0, cfg.value("heading_slow_angle_deg", heading_slow_angle_deg_));
      if (cfg.contains("obstacle_avoidance_enabled")) {
        obstacle_avoidance_enabled_ = cfg.at("obstacle_avoidance_enabled").get<bool>();
      }

      if (cfg.contains("final_target_yaw")) {
        final_target_yaw_ = cfg.at("final_target_yaw").get<double>();
        final_yaw_configured_ = true;
      }
      if (min_vx_ > max_vx_) {
        min_vx_ = max_vx_;
      }
      if (min_wz_ > max_wz_) {
        min_wz_ = max_wz_;
      }
    } catch (const std::exception & e) {
      publishStatus(std::string("控制参数解析失败: ") + e.what());
    }
  }

  void startNavigation()
  {
    if (!localized_) {
      publishStatus("开启导航失败: 尚未定位");
      return;
    }
    if (route_.size() < 2 || route_cumlen_.size() != route_.size()) {
      publishStatus("开启导航失败: 尚未收到路线");
      return;
    }

    is_auto_moving_ = true;
    is_obstacle_paused_ = false;
    stage_ = "path_tracking";
    current_vx_ = 0.0;
    current_wz_ = 0.0;
    nav_progress_idx_ = findClosestRouteIndex(pose_);
    if (!final_yaw_configured_) {
      final_target_yaw_ = routeHeadingDeg(route_.size() - 1);
    }
    publishStatus("C++ 控制器开始导航");
  }

  void stopNavigation(const std::string & status)
  {
    is_auto_moving_ = false;
    is_obstacle_paused_ = false;
    stage_ = "idle";
    has_lookahead_ = false;
    current_vx_ = 0.0;
    current_wz_ = 0.0;
    publishZero();
    publishStatus(status);
  }

  double routeHeadingDeg(std::size_t idx) const
  {
    if (route_.size() < 2) {
      return 0.0;
    }
    idx = std::min(idx, route_.size() - 1);
    const Point * a = nullptr;
    const Point * b = nullptr;
    if (idx + 1 < route_.size()) {
      a = &route_[idx];
      b = &route_[idx + 1];
    } else {
      a = &route_[idx - 1];
      b = &route_[idx];
    }
    return std::atan2(b->y - a->y, b->x - a->x) * 180.0 / M_PI;
  }

  Point interpolateByS(double s) const
  {
    if (route_.empty()) {
      return Point{};
    }
    if (route_.size() == 1 || s <= 0.0) {
      return route_.front();
    }
    if (s >= route_cumlen_.back()) {
      return route_.back();
    }

    const auto it = std::lower_bound(route_cumlen_.begin(), route_cumlen_.end(), s);
    std::size_t idx = static_cast<std::size_t>(std::distance(route_cumlen_.begin(), it));
    if (idx == 0) {
      return route_.front();
    }
    if (idx >= route_cumlen_.size()) {
      return route_.back();
    }

    const double s0 = route_cumlen_[idx - 1];
    const double s1 = route_cumlen_[idx];
    const double ds = s1 - s0;
    if (ds < 1e-9) {
      return route_[idx - 1];
    }
    const double r = (s - s0) / ds;
    const auto & p0 = route_[idx - 1];
    const auto & p1 = route_[idx];
    return Point{
      p0.x + r * (p1.x - p0.x),
      p0.y + r * (p1.y - p0.y),
      p0.z + r * (p1.z - p0.z)};
  }

  std::size_t findClosestRouteIndex(const Point & p) const
  {
    if (route_.empty()) {
      return 0;
    }
    std::size_t start = 0;
    std::size_t end = route_.size();
    if (is_auto_moving_ && route_.size() > 80) {
      start = nav_progress_idx_ > 20 ? nav_progress_idx_ - 20 : 0;
      end = std::min(route_.size(), nav_progress_idx_ + 220);
    }

    std::size_t best_idx = start;
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t i = start; i < end; ++i) {
      const double d = dist3d(p, route_[i]);
      if (d < best) {
        best = d;
        best_idx = i;
      }
    }
    return best_idx;
  }

  void controlLoop()
  {
    publish_state_tick_++;

    if (!obstacle_avoidance_enabled_) {
      is_obstacle_on_path_ = false;
      is_obstacle_paused_ = false;
    }
    if (is_obstacle_on_path_ && is_auto_moving_ && !is_obstacle_paused_) {
      is_obstacle_paused_ = true;
      last_obstacle_time_ = now();
      publishStatus("检测到前方障碍，C++ 控制器暂停");
    }
    if (is_obstacle_paused_ && !is_obstacle_on_path_ &&
        (now() - last_obstacle_time_).seconds() > resume_delay_) {
      is_obstacle_paused_ = false;
      obstacle_min_distance_ = 999.0;
      publishStatus("障碍已移除，C++ 控制器恢复导航");
    }

    if (!is_auto_moving_ || !localized_ || route_.size() < 2) {
      current_vx_ = 0.0;
      current_wz_ = 0.0;
      publishZero();
      throttledPublishState();
      return;
    }

    if (is_obstacle_paused_) {
      current_vx_ *= (1.0 - deceleration_alpha_);
      current_wz_ *= (1.0 - deceleration_alpha_);
      if (std::abs(current_vx_) < 0.005) {
        current_vx_ = 0.0;
      }
      if (std::abs(current_wz_) < 0.005) {
        current_wz_ = 0.0;
      }
      publishCmd(current_vx_, current_wz_);
      stage_ = "obstacle_pause";
      throttledPublishState();
      return;
    }

    const std::size_t nearest_idx = findClosestRouteIndex(pose_);
    nav_progress_idx_ = std::max(nav_progress_idx_, nearest_idx);

    const auto & end_pt = route_.back();
    const double end_dist = dist2d(end_pt, pose_);
    const double remain_s = route_cumlen_.back() - route_cumlen_[nav_progress_idx_];

    if (stage_ == "final_align") {
      const double yaw_err = normalizeAngleDeg(final_target_yaw_ - yaw_deg_);
      if (std::abs(yaw_err) < arrival_angle_ && std::abs(current_vx_) < 0.005) {
        finishNavigation();
        throttledPublishState();
        return;
      }
      const double target_wz = clampAbs(yaw_err * final_yaw_kp_, max_wz_, min_wz_);
      current_vx_ *= (1.0 - deceleration_alpha_);
      if (std::abs(current_vx_) < 0.005) current_vx_ = 0.0;
      current_wz_ = alpha_ * target_wz + (1.0 - alpha_) * current_wz_;
      if (std::abs(current_wz_) < 0.005) {
        current_wz_ = 0.0;
      }
      publishCmd(current_vx_, current_wz_);
      throttledPublishState();
      return;
    }

    stage_ = "path_tracking";
    if (remain_s <= end_slowdown_distance_ && end_dist <= arrival_dist_) {
      stage_ = "final_align";
      current_vx_ *= (1.0 - deceleration_alpha_);
      if (std::abs(current_vx_) < 0.005) current_vx_ = 0.0;
      current_wz_ *= (1.0 - deceleration_alpha_);
      if (std::abs(current_wz_) < 0.005) current_wz_ = 0.0;
      publishCmd(current_vx_, current_wz_);
      throttledPublishState();
      return;
    }

    double desired_v = max_vx_;
    const double target_s = std::min(route_cumlen_.back(), route_cumlen_[nav_progress_idx_] + lookahead_distance_);
    Point target = interpolateByS(target_s);
    if (remain_s <= end_slowdown_distance_) {
      desired_v = std::min(max_vx_, std::max(0.05, end_dist * 0.9));
    }

    lookahead_target_ = target;
    has_lookahead_ = true;

    const double dx = target.x - pose_.x;
    const double dy = target.y - pose_.y;
    const double dz = target.z - pose_.z;
    const double dist_xy = std::hypot(dx, dy);
    const double dist_3d = std::sqrt(dx * dx + dy * dy + dz * dz);

    double bearing = routeHeadingDeg(nav_progress_idx_);
    if (dist_xy >= 1e-6) {
      bearing = std::atan2(dy, dx) * 180.0 / M_PI;
    }
    const double yaw_error = normalizeAngleDeg(bearing - yaw_deg_);

    double turn_scale = 1.0 - std::min(std::abs(yaw_error), heading_slow_angle_deg_) / heading_slow_angle_deg_;
    turn_scale = std::max(0.18, turn_scale);
    double target_vx = desired_v * turn_scale;
    if (remain_s > end_slowdown_distance_) {
      target_vx = std::max(min_vx_, target_vx);
    }

    const double speed_ref_dist = std::max(dist_xy, std::min(dist_3d, 0.30));
    target_vx = std::min(target_vx, std::max(0.05, speed_ref_dist * 1.2));

    const double target_wz = clampAbs(
      yaw_error * path_yaw_kp_,
      max_wz_,
      std::abs(yaw_error) > 1.0 ? min_wz_ : 0.0);

    current_vx_ = alpha_ * target_vx + (1.0 - alpha_) * current_vx_;
    current_wz_ = alpha_ * target_wz + (1.0 - alpha_) * current_wz_;
    if (std::abs(current_vx_) < 0.005) {
      current_vx_ = 0.0;
    }
    if (std::abs(current_wz_) < 0.005) {
      current_wz_ = 0.0;
    }

    publishCmd(current_vx_, current_wz_);
    throttledPublishState();
  }

  void throttledPublishState()
  {
    if (publish_state_tick_ % 4 == 0) {
      publishState();
    }
  }

  void finishNavigation()
  {
    is_auto_moving_ = false;
    stage_ = "idle";
    current_vx_ = 0.0;
    current_wz_ = 0.0;
    has_lookahead_ = false;
    route_.clear();
    route_cumlen_.clear();
    publishZero();
    publishStatus("路线导航完成");

    std_msgs::msg::String msg;
    nlohmann::json data;
    data["event"] = "nav_done";
    data["success"] = true;
    data["message"] = "路线导航完成";
    data["time"] = now().seconds();
    msg.data = data.dump();
    done_pub_->publish(msg);
  }

  void publishCmd(double vx, double wz)
  {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = vx;
    cmd.angular.z = wz;
    cmd_pub_->publish(cmd);
  }

  void publishZero()
  {
    publishCmd(0.0, 0.0);
  }

  void publishStatus(const std::string & text)
  {
    status_text_ = text;
    std_msgs::msg::String msg;
    msg.data = text;
    status_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "%s", text.c_str());
  }

  void publishState()
  {
    nlohmann::json data;
    data["current_vx"] = current_vx_;
    data["current_wz"] = current_wz_;
    data["status_text"] = status_text_;
    data["is_auto_moving"] = is_auto_moving_;
    data["stage"] = stage_;
    data["is_obstacle_paused"] = is_obstacle_paused_;
    data["nav_progress_idx"] = nav_progress_idx_;
    data["route_total_points"] = route_.size();
    data["final_target_yaw"] = final_target_yaw_;
    data["obstacle_distance"] = obstacle_min_distance_;
    if (has_lookahead_) {
      data["lookahead_target"] = {
        {"x", lookahead_target_.x},
        {"y", lookahead_target_.y},
        {"z", lookahead_target_.z}};
    } else {
      data["lookahead_target"] = nullptr;
    }

    nlohmann::json boxes = nlohmann::json::array();
    for (const auto & b : obstacle_boxes_) {
      boxes.push_back({
        {"x_min", b.x_min}, {"y_min", b.y_min},
        {"x_max", b.x_max}, {"y_max", b.y_max},
      });
    }
    data["obstacle_boxes"] = boxes;

    nlohmann::json clusters = nlohmann::json::array();
    for (const auto & c : obstacle_clusters_) {
      clusters.push_back({
        {"center_x", c.center_x},
        {"center_y", c.center_y},
        {"radius", c.radius},
        {"z_min", c.z_min},
        {"z_max", c.z_max},
      });
    }
    data["obstacle_clusters"] = clusters;

    static int publish_tick = 0;
    publish_tick++;
    if (publish_tick % 40 == 1) {
      RCLCPP_DEBUG(get_logger(),
        "[publish #%d] clusters=%zu boxes=%zu dist=%.1f moving=%d paused=%d",
        publish_tick, obstacle_clusters_.size(), obstacle_boxes_.size(),
        obstacle_min_distance_, is_auto_moving_ ? 1 : 0,
        is_obstacle_paused_ ? 1 : 0);
    }

    std_msgs::msg::String msg;
    msg.data = data.dump();
    state_pub_->publish(msg);
  }

  static double point_to_segment_dist(double px, double py,
      double ax, double ay, double bx, double by)
  {
    double abx = bx - ax, aby = by - ay;
    double len_sq = abx * abx + aby * aby;
    if (len_sq < 1e-12) return std::hypot(px - ax, py - ay);
    double t = ((px - ax) * abx + (py - ay) * aby) / len_sq;
    if (t < 0.0) return std::hypot(px - ax, py - ay);
    if (t > 1.0) return std::hypot(px - bx, py - by);
    double proj_x = ax + t * abx, proj_y = ay + t * aby;
    return std::hypot(px - proj_x, py - proj_y);
  }

  void cloud_callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr & msg)
  {
    if (!localized_) return;
    if (route_.size() < 2) {
      is_obstacle_on_path_ = false;
      obstacle_boxes_.clear();
      obstacle_clusters_.clear();
      return;
    }

    obstacle_boxes_.clear();
    obstacle_clusters_.clear();
    is_obstacle_on_path_ = false;
    obstacle_min_distance_ = 999.0;

    double cos_yaw = std::cos(yaw_deg_ * M_PI / 180.0);
    double sin_yaw = std::sin(yaw_deg_ * M_PI / 180.0);
    double corridor_hw = 0.50;
    double floor_z = pose_.z - lidar_height_;
    double close_range = lidar_height_ * 2.5;
    double max_range = obstacle_threshold_ * 4.0;

    struct ObstPt { double x, y, z; double horiz_dist; };
    std::vector<ObstPt> obstacle_points;

    const double cell_size = 0.8;
    const double inv_cell = 1.0 / cell_size;

    double min_x = 1e9, max_x = -1e9, min_y = 1e9, max_y = -1e9;
    bool found = false;

    for (const auto & pt : msg->points) {
      double dx = pt.x - pose_.x;
      double dy = pt.y - pose_.y;
      double horiz_dist = std::hypot(dx, dy);
      if (horiz_dist < 0.08 || horiz_dist > max_range) continue;

      double front_check = (dx * cos_yaw + dy * sin_yaw) / horiz_dist;
      if (front_check < 0.5) continue;

      double dz = pt.z - floor_z;
      bool is_ground = false;

      if (horiz_dist < close_range) {
        if (dz < ground_z_tolerance_) is_ground = true;
      } else {
        double slope = dz / horiz_dist;
        if (slope < max_slope_) is_ground = true;
      }

      if (is_ground) continue;

      int cx = static_cast<int>(std::floor(static_cast<double>(pt.x) * inv_cell));
      int cy = static_cast<int>(std::floor(static_cast<double>(pt.y) * inv_cell));

      bool hit = false;
      for (int dc = -1; dc <= 1 && !hit; ++dc) {
        for (int dr = -1; dr <= 1 && !hit; ++dr) {
          auto it = route_segment_index_.find(gridKey(cx + dc, cy + dr));
          if (it == route_segment_index_.end()) continue;
          for (std::size_t seg_idx : it->second) {
            double d = point_to_segment_dist(
              static_cast<double>(pt.x), static_cast<double>(pt.y),
              route_[seg_idx].x, route_[seg_idx].y,
              route_[seg_idx + 1].x, route_[seg_idx + 1].y);
            if (d < corridor_hw) {
              min_x = std::min(min_x, static_cast<double>(pt.x));
              max_x = std::max(max_x, static_cast<double>(pt.x));
              min_y = std::min(min_y, static_cast<double>(pt.y));
              max_y = std::max(max_y, static_cast<double>(pt.y));
              obstacle_points.push_back({static_cast<double>(pt.x), static_cast<double>(pt.y),
                                         static_cast<double>(pt.z), horiz_dist});
              if (horiz_dist < obstacle_min_distance_) obstacle_min_distance_ = horiz_dist;
              found = true;
              hit = true;
              break;
            }
          }
        }
      }
    }

    if (found) {
      obstacle_boxes_.push_back({min_x, min_y, max_x, max_y});
    }

    is_obstacle_on_path_ = found && obstacle_avoidance_enabled_;

    if (!obstacle_points.empty()) {
      double cluster_dist = 0.50;
      std::vector<bool> assigned(obstacle_points.size(), false);

      for (size_t i = 0; i < obstacle_points.size(); ++i) {
        if (assigned[i]) continue;

        ObstacleCluster cl;
        cl.center_x = obstacle_points[i].x;
        cl.center_y = obstacle_points[i].y;
        cl.z_min = obstacle_points[i].z;
        cl.z_max = obstacle_points[i].z;
        cl.radius = 0.15;
        assigned[i] = true;
        int count = 1;

        bool changed = true;
        while (changed) {
          changed = false;
          for (size_t j = 0; j < obstacle_points.size(); ++j) {
            if (assigned[j]) continue;
            double dx2 = obstacle_points[j].x - cl.center_x;
            double dy2 = obstacle_points[j].y - cl.center_y;
            double d2 = std::sqrt(dx2 * dx2 + dy2 * dy2);
            if (d2 < cluster_dist) {
              cl.center_x = (cl.center_x * count + obstacle_points[j].x) / (count + 1);
              cl.center_y = (cl.center_y * count + obstacle_points[j].y) / (count + 1);
              cl.z_min = std::min(cl.z_min, obstacle_points[j].z);
              cl.z_max = std::max(cl.z_max, obstacle_points[j].z);
              assigned[j] = true;
              count++;
              changed = true;
            }
          }
        }

        if (count > 1) {
          cl.radius = 0.0;
          std::vector<size_t> cluster_indices = {i};
          for (size_t j = i + 1; j < obstacle_points.size(); ++j) {
            if (!assigned[j]) continue;
            double dx2 = obstacle_points[j].x - cl.center_x;
            double dy2 = obstacle_points[j].y - cl.center_y;
            double d2 = std::sqrt(dx2 * dx2 + dy2 * dy2);
            if (d2 < cluster_dist * 2.0) {
              cluster_indices.push_back(j);
            }
          }
          if (cluster_indices.empty()) cluster_indices.push_back(i);
          for (auto idx : cluster_indices) {
            double dx2 = obstacle_points[idx].x - cl.center_x;
            double dy2 = obstacle_points[idx].y - cl.center_y;
            double d2 = std::sqrt(dx2 * dx2 + dy2 * dy2);
            if (d2 > cl.radius) cl.radius = d2;
          }
        }
        cl.radius = std::max(0.10, cl.radius + 0.08);

        obstacle_clusters_.push_back(cl);
      }
    }

    static int cloud_tick = 0;
    cloud_tick++;
    if (cloud_tick % 5 == 1) {
      RCLCPP_DEBUG(get_logger(),
        "[cloud #%d] raw_pts=%zu obstacle_pts=%zu clusters=%zu dist=%.1f avoid=%d",
        cloud_tick, msg->points.size(), obstacle_points.size(),
        obstacle_clusters_.size(), obstacle_min_distance_,
        obstacle_avoidance_enabled_ ? 1 : 0);
    }
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr obstacle_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr route_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr config_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr start_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stop_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr clear_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr done_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<Point> route_;
  std::vector<double> route_cumlen_;
  std::unordered_map<std::int64_t, std::vector<std::size_t>> route_segment_index_;
  Point pose_;
  Point lookahead_target_;
  bool has_lookahead_{false};
  bool localized_{false};
  double yaw_deg_{0.0};

  bool is_auto_moving_{false};
  bool is_obstacle_paused_{false};
  std::string stage_{"idle"};
  std::string status_text_{"C++ 控制器待机"};
  std::size_t nav_progress_idx_{0};
  rclcpp::Time last_obstacle_time_{0, 0, RCL_ROS_TIME};

  double current_vx_{0.0};
  double current_wz_{0.0};
  double final_target_yaw_{0.0};
  bool final_yaw_configured_{false};

  double max_vx_{0.70};
  double max_wz_{0.50};
  double min_vx_{0.12};
  double min_wz_{0.10};
  double arrival_dist_{0.30};
  double arrival_angle_{5.0};
  double alpha_{0.18};
  double obstacle_threshold_{1.5};
  double resume_delay_{0.5};
  double lidar_height_{0.40};
  double ground_z_tolerance_{0.12};
  double max_slope_{0.65};
  std::string lidar_topic_{"/livox/lidar"};
  double obstacle_min_distance_{999.0};
  bool is_obstacle_on_path_{false};
  bool obstacle_avoidance_enabled_{false};

  struct ObstacleBox { double x_min, y_min, x_max, y_max; };
  std::vector<ObstacleBox> obstacle_boxes_;
  struct ObstacleCluster { double center_x, center_y, radius, z_min, z_max; };
  std::vector<ObstacleCluster> obstacle_clusters_;
  double deceleration_alpha_{0.10};
  double lookahead_distance_{1.00};
  int publish_state_tick_{0};
  double path_yaw_kp_{0.020};
  double final_yaw_kp_{0.020};
  double end_slowdown_distance_{1.50};
  double heading_slow_angle_deg_{35.0};
};

}  // namespace indoor_route_nav

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<indoor_route_nav::ControllerNode>());
  rclcpp::shutdown();
  return 0;
}