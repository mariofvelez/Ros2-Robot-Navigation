// src/find_human.cpp

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "navigation/navigation.hpp"   // Professor's Navigator library

#include <vector>
#include <memory>
#include <cmath>

using std::placeholders::_1;
using namespace std::chrono_literals;

class HumanTrackerNode : public rclcpp::Node
{
public:
  enum class State {
    WAIT_FOR_MAP_AND_POSE,
    GOING_TO_ORIGIN,
    GOING_TO_HUMAN0,
    GOING_TO_HUMAN1,
    DONE
  };

  explicit HumanTrackerNode(const std::shared_ptr<Navigator>& navigator)
    : Node("human_tracker_node"),
      navigator_(navigator),
      state_(State::WAIT_FOR_MAP_AND_POSE),
      started_navigation_(false)
  {
    // --- Subscriptions ---

    // /map: latched occupancy grid from map_server
    auto qos_map = rclcpp::QoS(1).reliable().transient_local();
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map",
        qos_map,
        std::bind(&HumanTrackerNode::mapCallback, this, _1));

    // /amcl_pose
    amcl_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose",
        10,
        std::bind(&HumanTrackerNode::amclCallback, this, _1));

    // /scan: laser scanner
    auto qos_scan = rclcpp::SensorDataQoS();
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan",
        qos_scan,
        std::bind(&HumanTrackerNode::scanCallback, this, _1));

    // Control loop timer
    timer_ = create_wall_timer(200ms, std::bind(&HumanTrackerNode::controlLoop, this));

    // Hard-coded original human positions in MAP frame (from Gazebo)
    humans_.clear();
    humans_.push_back({  1.0,  -1.0, false });   // Person 1 - Standing
    humans_.push_back({ -12.0,  15.0, false });  // Person 2 - Walking

    RCLCPP_INFO(get_logger(), "HumanTrackerNode started.");
  }

private:
  // ---------- Callbacks ----------

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    map_ = msg;
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Received /map (size: %u x %u)",
      msg->info.width, msg->info.height);
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    last_scan_ = msg;
  }

  void amclCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    last_amcl_pose_ = msg;

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Received /amcl_pose (x=%.2f, y=%.2f)",
      msg->pose.pose.position.x,
      msg->pose.pose.position.y);

    if (!started_navigation_ && map_ && state_ == State::WAIT_FOR_MAP_AND_POSE) {
      RCLCPP_INFO(get_logger(), "Map and pose ready. Going to map origin.");
      goToOrigin();
      started_navigation_ = true;
      state_ = State::GOING_TO_ORIGIN;
    }
  }

  // ---------- Navigation helpers ----------

  void goToOrigin()
  {
    auto goal = std::make_shared<geometry_msgs::msg::Pose>();

    goal->position.x = 0.0;
    goal->position.y = 0.0;
    goal->position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);  // face +x
    goal->orientation = tf2::toMsg(q);

    if (!navigator_->GoToPose(goal)) {
      RCLCPP_ERROR(get_logger(), "GoToPose request to origin was NOT accepted.");
      state_ = State::DONE;
    } else {
      RCLCPP_INFO(get_logger(), "GoToPose to origin sent!");
    }
  }

  // Place robot 1 m behind the human in -x direction, facing +x so the human is in front.
  void goToHumanVantage(std::size_t idx)
  {
    if (idx >= humans_.size()) {
      RCLCPP_ERROR(get_logger(), "goToHumanVantage: invalid human index %zu", idx);
      state_ = State::DONE;
      return;
    }
    const auto &h = humans_[idx];

    auto goal = std::make_shared<geometry_msgs::msg::Pose>();

    double offset = 1.0;  // 1m behind human along -x
    goal->position.x = h.x - offset;
    goal->position.y = h.y;
    goal->position.z = 0.0;

    // Face +x so the human is directly in front
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);
    goal->orientation = tf2::toMsg(q);

    if (!navigator_->GoToPose(goal)) {
      RCLCPP_ERROR(
        get_logger(),
        "GoToPose to Human %zu vantage (%.2f, %.2f) was NOT accepted.",
        idx, goal->position.x, goal->position.y);
      state_ = State::DONE;
    } else {
      RCLCPP_INFO(
        get_logger(),
        "GoToPose to Human %zu vantage sent (%.2f, %.2f).",
        idx, goal->position.x, goal->position.y);
    }
  }

  // Check a single human using current /amcl_pose + /scan
  void checkHuman(std::size_t idx)
	{
	  if (!last_scan_ || !last_amcl_pose_) {
	    RCLCPP_WARN(get_logger(), "checkHuman(%zu): missing scan or pose.", idx);
	    return;
	  }
	  if (idx >= humans_.size()) {
	    RCLCPP_ERROR(get_logger(), "checkHuman: invalid human index %zu", idx);
	    return;
	  }
  
	  const auto & h    = humans_[idx];
	  const auto & pose = last_amcl_pose_->pose.pose;
  
	  double rx = pose.position.x;
	  double ry = pose.position.y;
  
	  // Extract yaw of robot in map frame
	  tf2::Quaternion q;
	  tf2::fromMsg(pose.orientation, q);
	  double roll, pitch, yaw;
	  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  
	  const auto & scan = *last_scan_;
	  double angle_min  = scan.angle_min;
	  double angle_inc  = scan.angle_increment;
	  int    n          = static_cast<int>(scan.ranges.size());
  
	  const double PI               = 3.14159265358979323846;
	  const double TWO_PI           = 2.0 * PI;
	  const double LASER_YAW_OFFSET = -PI / 2.0;   // base_link → rplidar_link: -90 deg
	  const double RANGE_TOL        = 0.75;        // meters
	  const double MAX_CHECK_DIST   = 25.0;        // max distance we care about
  
	  // --- 1. Geometry: where should the human be relative to robot? ---
  
	  // Vector robot -> human in map frame
	  double dx = h.x - rx;
	  double dy = h.y - ry;
	  double expected_range = std::hypot(dx, dy);
  
	  if (expected_range > MAX_CHECK_DIST) {
	    RCLCPP_WARN(
	      get_logger(),
	      "Human %zu: expected_range=%.2f too far to check reliably.",
	      idx, expected_range);
	    return;
	  }
  
	  // Bearing in map frame (global)
	  double angle_map  = std::atan2(dy, dx);
	  // Bearing in robot base_link frame
	  double angle_base = angle_map - yaw;
	  // Bearing in LiDAR (rplidar_link) frame
	  double angle_sensor = angle_base + LASER_YAW_OFFSET;
  
	  // Normalize into [angle_min, angle_min + n*angle_inc]
	  double angle_max = angle_min + angle_inc * (n - 1);
	  while (angle_sensor < angle_min)  angle_sensor += TWO_PI;
	  while (angle_sensor > angle_max)  angle_sensor -= TWO_PI;
  
	  int center_idx = static_cast<int>(std::round((angle_sensor - angle_min) / angle_inc));
  
	  if (center_idx < 0 || center_idx >= n) {
	    RCLCPP_WARN(
	      get_logger(),
	      "Human %zu: center scan idx %d out of bounds [0, %d). "
	      "angle_map=%.1f deg, angle_base=%.1f deg, angle_sensor=%.1f deg",
	      idx, center_idx, n,
	      angle_map   * 180.0 / PI,
	      angle_base  * 180.0 / PI,
	      angle_sensor* 180.0 / PI);
	    return;
	  }
  
	  // --- 2. Use an angular window around the center index ---
  
	  // e.g. ±15 degrees window
	  double window_deg = 15.0;
	  double window_rad = window_deg * PI / 180.0;
	  int half_window_idx = static_cast<int>(std::ceil(window_rad / angle_inc));
  
	  int idx_min = std::max(0, center_idx - half_window_idx);
	  int idx_max = std::min(n - 1, center_idx + half_window_idx);
  
	  bool found_close = false;
	  double best_measured = std::numeric_limits<double>::infinity();
	  int best_idx = center_idx;
  
	  for (int i = idx_min; i <= idx_max; ++i) {
	    double r = scan.ranges[i];
	    if (!std::isfinite(r)) {
	      continue;  // skip inf/NaN
	    }
	
	    double diff = std::fabs(r - expected_range);
	    if (diff < best_measured || !std::isfinite(best_measured)) {
	      best_measured = r;
	      best_idx = i;
	    }
	
	    if (diff < RANGE_TOL) {
	      found_close = true;
	    }
	  }
  
	  // Debug print
	  RCLCPP_INFO(
	    get_logger(),
	    "Human %zu: expected=%.2f m, best_measured=%.2f m (center_idx=%d, window=[%d,%d], best_idx=%d). "
	    "angles: map=%.1f deg, base=%.1f deg, sensor=%.1f deg",
	    idx, expected_range, best_measured,
	    center_idx, idx_min, idx_max, best_idx,
	    angle_map   * 180.0 / PI,
	    angle_base  * 180.0 / PI,
	    angle_sensor* 180.0 / PI);
	
	  // --- 3. Classification based on window ---
	
	  if (found_close) {
	    humans_[idx].moved = false;
	    RCLCPP_INFO(
	      get_logger(),
	      "Human %zu: STILL at original position (window check).",
	      idx);
	  } else {
	    humans_[idx].moved = true;
	    RCLCPP_INFO(
	      get_logger(),
	      "Human %zu: MOVED (no ranges within tolerance in window).",
	      idx);
	  }
	}

  // ---------- Main control loop ----------

  void controlLoop()
  {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "ControlLoop state=%d, map_=%s, amcl=%s",
      static_cast<int>(state_),
      map_ ? "yes" : "no",
      last_amcl_pose_ ? "yes" : "no");

    switch (state_) {
      case State::WAIT_FOR_MAP_AND_POSE:
        // do nothing; amclCallback will trigger goToOrigin when ready
        break;

      case State::GOING_TO_ORIGIN:
        if (navigator_->IsTaskComplete()) {
          RCLCPP_INFO(get_logger(), "Reached map origin.");
          // Now go to vantage near Human 0
          goToHumanVantage(0);
          state_ = State::GOING_TO_HUMAN0;
        }
        break;

      case State::GOING_TO_HUMAN0:
        if (navigator_->IsTaskComplete()) {
          RCLCPP_INFO(get_logger(), "Reached vantage near Human 0.");
          // Check Human 0 using LiDAR
          checkHuman(0);
          // Then go to Human 1
          goToHumanVantage(1);
          state_ = State::GOING_TO_HUMAN1;
        }
        break;

      case State::GOING_TO_HUMAN1:
        if (navigator_->IsTaskComplete()) {
          RCLCPP_INFO(get_logger(), "Reached vantage near Human 1.");
          checkHuman(1);

          // Print summary
          for (std::size_t i = 0; i < humans_.size(); ++i) {
            auto &h = humans_[i];
            if (h.moved) {
              RCLCPP_INFO(
                get_logger(),
                "SUMMARY: Human %zu MOVED from original position (%.2f, %.2f).",
                i, h.x, h.y);
            } else {
              RCLCPP_INFO(
                get_logger(),
                "SUMMARY: Human %zu STILL at original position (%.2f, %.2f).",
                i, h.x, h.y);
            }
          }

          state_ = State::DONE;
        }
        break;

      case State::DONE:
      default:
        // Nothing else to do; node stays alive.
        break;
    }
  }

  // ---------- Members ----------

  std::shared_ptr<Navigator> navigator_;

  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr last_amcl_pose_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;

  rclcpp::TimerBase::SharedPtr timer_;

  State state_;
  bool started_navigation_;

  struct HumanOriginal {
    double x;
    double y;
    bool moved;
  };
  std::vector<HumanOriginal> humans_;
};

// ---------- main() ----------

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Create Navigator (do NOT add it to any executor!)
  auto navigator = std::make_shared<Navigator>(true, true);

  // We do NOT call WaitUntilNav2Active() to avoid /initialpose side effects.
  // GoToPose() will wait for the action server when needed.

  auto human_tracker = std::make_shared<HumanTrackerNode>(navigator);

  rclcpp::spin(human_tracker);

  rclcpp::shutdown();
  return 0;
}