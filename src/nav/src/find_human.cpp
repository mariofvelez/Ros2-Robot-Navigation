// src/find_human.cpp

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/time.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include "navigation/navigation.hpp"   // Professor's Navigator library

#include <vector>
#include <memory>
#include <cmath>
#include <limits>
#include <fstream>
#include <string>
#include <sstream>
#include <algorithm>

using std::placeholders::_1;
using namespace std::chrono_literals;

class HumanTrackerNode : public rclcpp::Node
{
public:
  enum class State {
    WAIT_FOR_MAP_AND_POSE = 0,
    GOING_TO_FIRST_HUMAN,
    CHECKING_FIRST_HUMAN,
    GOING_TO_SECOND_HUMAN,
    CHECKING_SECOND_HUMAN,
    WAIT_FOR_COSTMAP,           // new
    SEARCH_GOING_TO_WAYPOINT,
    SEARCH_SPINNING_AT_WAYPOINT,
    DONE
  };

  static constexpr const char* STATE_NAMES[] = {
      "Waiting for Map and Pose",
      "Going to 1st Human",
      "Checking 1st Human",
      "Going to 2nd Human",
      "Checking 2nd Human",
      "Waiting for Costmap",
      "Search Going to Waypoint",
      "Search Spinning at Waypoint",
      "Done"
  };

  // current state of tracking a human
  enum HumanTrackingState {
      UNKNOWN,
      STILL,
      MOVED
  };

  struct HumanOriginal {
    double x;
    double y;
    HumanTrackingState state;
  };

  struct ExtraPoint {
    double x;
    double y;
  };

  // Helper: are we in a search state where scans should be recorded?
  bool isSearchScanActive() const
  {
    // Collect scan points any time we're in the Phase 2 search corridor:
    // - moving between waypoints
    // - spinning at waypoints
    return (state_ == State::SEARCH_GOING_TO_WAYPOINT ||
            state_ == State::SEARCH_SPINNING_AT_WAYPOINT);
  }

  explicit HumanTrackerNode(const std::shared_ptr<Navigator>& navigator)
    : Node("human_tracker_node"),
      navigator_(navigator),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_),
      state_(State::WAIT_FOR_MAP_AND_POSE),
      started_navigation_(false),
      current_human_idx_(-1),
      samples_total_(0),
      samples_hit_(0),
      first_human_idx_(0),
      second_human_idx_(1),
      search_wp_index_(0),
      search_spin_started_(false),
      map_frame_("map"),
      have_map_(false),
      have_costmap_(false)
  {
    // Creating subscriptions
    auto qos_map = rclcpp::QoS(1).reliable().transient_local();
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map",
        qos_map,
        std::bind(&HumanTrackerNode::mapCallback, this, _1));

    auto qos_costmap = rclcpp::QoS(1).reliable().transient_local();
    costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/global_costmap/costmap",
        qos_costmap,
        std::bind(&HumanTrackerNode::costmapCallback, this, _1));

    amcl_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose",
        10,
        std::bind(&HumanTrackerNode::amclCallback, this, _1));

    auto qos_scan = rclcpp::SensorDataQoS();
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan",
        qos_scan,
        std::bind(&HumanTrackerNode::scanCallback, this, _1));

    timer_ = create_wall_timer(200ms, std::bind(&HumanTrackerNode::controlLoop, this));

    // Original human positions in map frame
    humans_.clear();
    humans_.push_back({  1.0,  -1.0, HumanTrackingState::UNKNOWN });   // Human 0
    humans_.push_back({ -12.0,  15.0, HumanTrackingState::UNKNOWN });  // Human 1

    initSearchWaypoints();

    // CSV in current working directory
    const char* debug_filename = "human_tracker_debug.csv";
    debug_log_.open(debug_filename);
    if (debug_log_.is_open()) {
      debug_log_ << "time,label,state,wp_or_human_idx,goal_x,goal_y,"
                    "amcl_x,amcl_y,amcl_yaw\n";
      RCLCPP_INFO(get_logger(),
                  "Debug log file opened at '%s'", debug_filename);
    } else {
      RCLCPP_WARN(get_logger(),
                  "Failed to open debug log file at '%s'", debug_filename);
    }

    RCLCPP_INFO(get_logger(), "HumanTrackerNode started.");
  }

private:
  // ---------- Utils & logging ----------
  std::string stateToString(State s) const
  {
    switch (s) {
      case State::WAIT_FOR_MAP_AND_POSE:      return "WAIT_FOR_MAP_AND_POSE";
      case State::GOING_TO_FIRST_HUMAN:       return "GOING_TO_FIRST_HUMAN";
      case State::CHECKING_FIRST_HUMAN:       return "CHECKING_FIRST_HUMAN";
      case State::GOING_TO_SECOND_HUMAN:      return "GOING_TO_SECOND_HUMAN";
      case State::CHECKING_SECOND_HUMAN:      return "CHECKING_SECOND_HUMAN";
      case State::WAIT_FOR_COSTMAP:           return "WAIT_FOR_COSTMAP";
      case State::SEARCH_GOING_TO_WAYPOINT:   return "SEARCH_GOING_TO_WAYPOINT";
      case State::SEARCH_SPINNING_AT_WAYPOINT:return "SEARCH_SPINNING_AT_WAYPOINT";
      case State::DONE:                       return "DONE";
      default:                                return "UNKNOWN";
    }
  }

  void logEvent(const std::string &label,
                int wp_or_human_idx,
                const geometry_msgs::msg::Pose *goal_pose = nullptr)
  {
    if (!debug_log_.is_open()) return;

    double t  = now().seconds();
    double gx = (goal_pose ? goal_pose->position.x : 0.0);
    double gy = (goal_pose ? goal_pose->position.y : 0.0);

    double ax = 0.0, ay = 0.0, ayaw = 0.0;
    if (last_amcl_pose_) {
      const auto &p = last_amcl_pose_->pose.pose;
      ax = p.position.x;
      ay = p.position.y;
      tf2::Quaternion q;
      tf2::fromMsg(p.orientation, q);
      double roll, pitch;
      tf2::Matrix3x3(q).getRPY(roll, pitch, ayaw);
    }

    debug_log_
      << t << ","
      << label << ","
      << stateToString(state_) << ","
      << wp_or_human_idx << ","
      << gx << ","
      << gy << ","
      << ax << ","
      << ay << ","
      << ayaw << "\n";
  }

  // Callbacks
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    map_ = msg;
    have_map_ = true;
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Received /map (size: %u x %u, res=%.3f)",
      msg->info.width, msg->info.height, msg->info.resolution);
  }

  void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    costmap_ = msg;
    have_costmap_ = true;
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Received /global_costmap/costmap (size: %u x %u, res=%.3f)",
      msg->info.width, msg->info.height, msg->info.resolution);
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // Keep last scan for Phase 1 human checks
    last_scan_ = msg;

    // We only care about extra points once we have a map AND a costmap,
    // and we're in Phase 2 search (either moving to waypoints or spinning).
    if (!have_map_ || !have_costmap_) {
      return;
    }

    if (!isSearchScanActive()) {
      // Ignore scans in Phase 1 or after we're done.
      return;
    }

    // Transform from laser frame -> map frame for all valid beams
    geometry_msgs::msg::TransformStamped tf_laser_to_map;
    try {
      tf_laser_to_map = tf_buffer_.lookupTransform(
          map_frame_,               // e.g. "map"
          msg->header.frame_id,     // e.g. "base_scan"
          msg->header.stamp,
          tf2::durationFromSec(0.1));
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          2000,
          "scanCallback: TF lookup failed: %s", ex.what());
      return;
    }

    tf2::Transform tf;
    tf2::fromMsg(tf_laser_to_map.transform, tf);

    // Optional: cap how many extra points we keep overall
    constexpr std::size_t MAX_EXTRA_POINTS = 200000;
    if (extra_points_.size() > MAX_EXTRA_POINTS) {
      // Drop oldest half to prevent unbounded growth
      extra_points_.erase(extra_points_.begin(),
                          extra_points_.begin() + extra_points_.size() / 2);
      RCLCPP_WARN(this->get_logger(),
                  "scanCallback: extra_points_ truncated to %zu",
                  extra_points_.size());
    }

    const auto angle_min   = msg->angle_min;
    const auto angle_inc   = msg->angle_increment;
    const auto range_min   = msg->range_min;
    const auto range_max   = msg->range_max;

    // Tuneable range window: ignore points too close or too far
    const double min_useful_range = std::max(0.7, static_cast<double>(range_min));
    const double max_useful_range = std::min(10.0, static_cast<double>(range_max));

    for (std::size_t i = 0; i < msg->ranges.size(); ++i) {
      const float r = msg->ranges[i];
      if (!std::isfinite(r)) {
        continue;
      }
      if (r < min_useful_range || r > max_useful_range) {
        continue;
      }

      const double angle = angle_min + static_cast<double>(i) * angle_inc;
      const double lx = r * std::cos(angle);
      const double ly = r * std::sin(angle);

      tf2::Vector3 p_laser(lx, ly, 0.0);
      tf2::Vector3 p_map = tf * p_laser;

      const double mx = p_map.x();
      const double my = p_map.y();

      // Extra-point filter: use BOTH map and costmap to decide whether
      // this is a "candidate moved-human point".
      if (!isCandidateExtraPoint(mx, my)) {
        continue;
      }

      extra_points_.push_back(ExtraPoint{mx, my});
    }
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
      started_navigation_ = true;

      // sort humans by distance
      if (last_amcl_pose_) {
          const auto& p = last_amcl_pose_->pose.pose;
          double rx = p.position.x;
          double ry = p.position.y;
          double d0 = std::hypot(humans_[0].x - rx, humans_[0].y - ry);
          double d1 = std::hypot(humans_[1].x - rx, humans_[1].y - ry);
          if (d0 <= d1) {
              first_human_idx_ = 0;
              second_human_idx_ = 1;
          }
          else {
              first_human_idx_ = 1;
              second_human_idx_ = 0;
          }
      }
      else {
          first_human_idx_ = 0;
          second_human_idx_ = 1;
      }

      RCLCPP_INFO(
          get_logger(),
          "Phase 1 ordering: first human=%d at (%.2f, %.2f), second human=%d at (%.2f, %.2f)",
          first_human_idx_, humans_[first_human_idx_].x, humans_[first_human_idx_].y,
          second_human_idx_, humans_[second_human_idx_].x, humans_[second_human_idx_].y);

      goToHumanVantage(first_human_idx_);

      state_ = State::GOING_TO_FIRST_HUMAN;
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
      RCLCPP_ERROR(get_logger(), "Go to pose command was NOT accepted by navigator.");
      state_ = State::DONE;
    } else {
      RCLCPP_INFO(get_logger(), "Go to origin command sent!");
      logEvent("PHASE1_GOAL_SENT", 0, goal.get());
    }
  }

  void goToHumanVantage(int idx)
  {
    if (idx < 0 || static_cast<std::size_t>(idx) >= humans_.size()) {
      RCLCPP_ERROR(get_logger(), "goToHumanVantage: invalid human index %d", idx);
      state_ = State::DONE;
      return;
    }
    const auto& h = humans_[idx];

    auto goal = std::make_shared<geometry_msgs::msg::Pose>();

    double offset = 1.0;  // 1m behind human along -x
    goal->position.x = h.x - offset;
    goal->position.y = h.y;
    goal->position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);  // face +x
    goal->orientation = tf2::toMsg(q);

    if (!navigator_->GoToPose(goal)) {
      RCLCPP_ERROR(
        get_logger(),
        "GoToPose to Human %d vantage (%.2f, %.2f) was NOT accepted.",
        idx, goal->position.x, goal->position.y);
      state_ = State::DONE;
    } else {
      RCLCPP_INFO(
        get_logger(),
        "GoToPose to Human %d vantage sent (%.2f, %.2f).",
        idx, goal->position.x, goal->position.y);
      logEvent("PHASE1_GOAL_SENT", idx, goal.get());
    }
  }

  void initSearchWaypoints()
  {
    search_waypoints_.clear();
    search_waypoints_.reserve(17);

    auto makePose = [](double x, double y) {
      geometry_msgs::msg::Pose p;
      p.position.x = x;
      p.position.y = y;
      p.position.z = 0.0;
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, 0.0);
      p.orientation = tf2::toMsg(q);
      return p;
    };

    // read waypoints
    std::string text;
    std::ifstream file("src/nav/src/waypoints.txt");
    while (std::getline(file, text))
    {
        unsigned int axes = 0;
        std::string coord_str[2];

        std::stringstream ss(text);
        while (ss >> coord_str[axes] && axes < 2)
        {
            axes++;
        }
        if (axes > 0)
        {
            search_waypoints_.push_back(makePose(std::stof(coord_str[0]), std::stof(coord_str[1])));
        }
    }
    RCLCPP_INFO(get_logger(), "here");
  }

  // sends the robot to a waypoint index
  void sendSearchWaypointGoal(int idx)
  {
    if (idx < 0 || static_cast<std::size_t>(idx) >= search_waypoints_.size()) {
      RCLCPP_ERROR(get_logger(), "sendSearchWaypointGoal: invalid index %d", idx);
      state_ = State::DONE;
      return;
    }

    auto goal = std::make_shared<geometry_msgs::msg::Pose>(search_waypoints_[idx]);

    if (!navigator_->GoToPose(goal)) {
      RCLCPP_ERROR(
        get_logger(),
        "GoToPose to search waypoint %d (%.2f, %.2f) was NOT accepted.",
        idx, goal->position.x, goal->position.y);
      state_ = State::DONE;
    } else {
      RCLCPP_INFO(
        get_logger(),
        "GoToPose to search waypoint %d sent (%.2f, %.2f).",
        idx, goal->position.x, goal->position.y);
      logEvent("SEARCH_WP_GOAL_SENT", idx, goal.get());
    }
  }

  // Checks if a human is in its original position with the laser scan.
  // The robot must be within range for an accurate reading
  bool sampleHumanWindow(std::size_t idx)
  {
    if (!last_scan_ || !last_amcl_pose_) {
      RCLCPP_WARN(get_logger(), "sampleHumanWindow(%zu): missing scan or pose.", idx);
      return false;
    }
    if (idx >= humans_.size()) {
      RCLCPP_ERROR(get_logger(), "sampleHumanWindow: invalid human index %zu", idx);
      return false;
    }

    const auto &h    = humans_[idx];
    const auto &pose = last_amcl_pose_->pose.pose;

    double rx = pose.position.x;
    double ry = pose.position.y;

    tf2::Quaternion q;
    tf2::fromMsg(pose.orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    const auto &scan = *last_scan_;
    double angle_min = scan.angle_min;
    double angle_inc = scan.angle_increment;
    int    n         = static_cast<int>(scan.ranges.size());

    const double PI                = 3.14159265358979323846;
    const double TWO_PI            = 2.0 * PI;
    const double LASER_YAW_OFFSET  = -PI / 2.0;
    const double RANGE_TOL         = 0.75;
    const double MAX_CHECK_DIST    = 25.0;
    const double WINDOW_DEG        = 15.0;
    const double window_rad        = WINDOW_DEG * PI / 180.0;

    // expected distance from human
    double dx = h.x - rx;
    double dy = h.y - ry;
    double expected_range = std::hypot(dx, dy);

    if (expected_range > MAX_CHECK_DIST) {
      RCLCPP_WARN(
        get_logger(),
        "Human %zu: expected_range=%.2f too far to check reliably.",
        idx, expected_range);
      return false;
    }

    // get local human angle
    double angle_map    = std::atan2(dy, dx);
    double angle_base   = angle_map - yaw;
    double angle_sensor = angle_base + LASER_YAW_OFFSET;

    double angle_max = angle_min + angle_inc * (n - 1);
    while (angle_sensor < angle_min)  angle_sensor += TWO_PI;
    while (angle_sensor > angle_max)  angle_sensor -= TWO_PI;

    // compute indices of laser scan window
    int center_idx = static_cast<int>(std::round((angle_sensor - angle_min) / angle_inc));
    if (center_idx < 0 || center_idx >= n) {
      RCLCPP_WARN(
        get_logger(),
        "Human %zu: center scan idx %d out of bounds [0, %d).",
        idx, center_idx, n);
      return false;
    }

    int half_window_idx = static_cast<int>(std::ceil(window_rad / angle_inc));
    int idx_min = std::max(0, center_idx - half_window_idx);
    int idx_max = std::min(n - 1, center_idx + half_window_idx);

    bool found_close = false;
    double best_measured = std::numeric_limits<double>::infinity();
    int best_idx = center_idx;

    for (int i = idx_min; i <= idx_max; ++i) {
      double r = scan.ranges[i];
      if (!std::isfinite(r)) continue;

      double diff = std::fabs(r - expected_range);
      if (diff < best_measured) {
        best_measured = r;
        best_idx = i;
      }
      if (diff < RANGE_TOL) {
        found_close = true;
      }
    }

    RCLCPP_INFO(
      get_logger(),
      "Human %zu sample: expected=%.2f, best_measured=%.2f "
      "(center=%d, window=[%d,%d], best_idx=%d). "
      "angles: map=%.1f deg, base=%.1f deg, sensor=%.1f deg",
      idx, expected_range, best_measured,
      center_idx, idx_min, idx_max, best_idx,
      angle_map   * 180.0 / PI,
      angle_base  * 180.0 / PI,
      angle_sensor* 180.0 / PI);

    return found_close;
  }

  // ---------- Costmap / extra-point helpers ----------

  bool isNearStaticMapObstacle(int mx, int my) const
  {
    if (!map_) return true;  // conservative

    const auto &info = map_->info;
    int w = static_cast<int>(info.width);
    int h = static_cast<int>(info.height);
    double res = info.resolution;

    const double R_STATIC = 0.8;    // ~80 cm band around shelves/walls
    int max_cells = static_cast<int>(std::ceil(R_STATIC / res));

    for (int dy = -max_cells; dy <= max_cells; ++dy) {
      int yy = my + dy;
      if (yy < 0 || yy >= h) continue;
      for (int dx = -max_cells; dx <= max_cells; ++dx) {
        int xx = mx + dx;
        if (xx < 0 || xx >= w) continue;
        int idx = yy * w + xx;
        int8_t occ = map_->data[idx];
        if (occ > 0) {
          return true;
        }
      }
    }
    return false;
  }

  // Use map + costmap + static band to decide if (ex,ey) in world coords
  // is a "candidate moved-human" point.
  bool isCandidateExtraPoint(double ex, double ey) const
  {
    if (!map_ || !costmap_) {
      return false;
    }

    // --- Map indexing ---
    const auto &minfo = map_->info;
    double map_res = minfo.resolution;
    double map_origin_x = minfo.origin.position.x;
    double map_origin_y = minfo.origin.position.y;
    int map_w = static_cast<int>(minfo.width);
    int map_h = static_cast<int>(minfo.height);

    int mx = static_cast<int>((ex - map_origin_x) / map_res);
    int my = static_cast<int>((ey - map_origin_y) / map_res);
    if (mx < 0 || mx >= map_w || my < 0 || my >= map_h) {
      // Outside known map: costmap here is usually just inflation.
      return false;
    }

    int map_idx = my * map_w + mx;
    int8_t occ = map_->data[map_idx];

    // Only accept map *free* cells; reject unknown or occupied.
    if (occ != 0) {
      return false;
    }

    // --- Costmap indexing ---
    const auto &cinfo = costmap_->info;
    double c_res      = cinfo.resolution;
    double c_origin_x = cinfo.origin.position.x;
    double c_origin_y = cinfo.origin.position.y;
    int c_w           = static_cast<int>(cinfo.width);
    int c_h           = static_cast<int>(cinfo.height);

    int cmx = static_cast<int>((ex - c_origin_x) / c_res);
    int cmy = static_cast<int>((ey - c_origin_y) / c_res);
    if (cmx < 0 || cmx >= c_w || cmy < 0 || cmy >= c_h) {
      return false;
    }

    int cidx = cmy * c_w + cmx;
    int cost = static_cast<int>(costmap_->data[cidx]);

    // Only keep cost > 0 (inside inflation/obstacle); ignore free costmap cells.
    if (cost <= 0) {
      return false;
    }

    // If the point is within ~0.8 m of any static map obstacle, it's probably
    // just the normal inflation band around shelves/walls, not a new object.
    if (isNearStaticMapObstacle(mx, my)) {
      return false;
    }

    // Otherwise: free in map, cost>0 in costmap, and not near static walls.
    // This is exactly the "free blob of costmap in free space" scenario
    // we care about → candidate moved-human point.
    return true;
  }

  // (Unused now, but kept in case you want the old behavior)
  void collectExtraPointsFromScan()
  {
    if (!map_ || !costmap_ || !last_scan_ || !last_amcl_pose_) {
      return;
    }

    const auto &scan = *last_scan_;
    const auto &pose = last_amcl_pose_->pose.pose;

    double rx = pose.position.x;
    double ry = pose.position.y;

    tf2::Quaternion q;
    tf2::fromMsg(pose.orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    const auto &minfo = map_->info;
    double map_res = minfo.resolution;
    double map_origin_x = minfo.origin.position.x;
    double map_origin_y = minfo.origin.position.y;
    int map_w = static_cast<int>(minfo.width);
    int map_h = static_cast<int>(minfo.height);

    const auto &cinfo = costmap_->info;
    double c_res      = cinfo.resolution;
    double c_origin_x = cinfo.origin.position.x;
    double c_origin_y = cinfo.origin.position.y;
    int c_w           = static_cast<int>(cinfo.width);
    int c_h           = static_cast<int>(cinfo.height);

    double angle_min = scan.angle_min;
    double angle_inc = scan.angle_increment;
    int n = static_cast<int>(scan.ranges.size());

    const double PI               = 3.14159265358979323846;
    const double LASER_YAW_OFFSET = -PI / 2.0;
    const double MIN_RANGE        = 0.3;
    const double MAX_RANGE        = 20.0;

    for (int i = 0; i < n; ++i) {
      double r = scan.ranges[i];
      if (!std::isfinite(r)) continue;
      if (r < MIN_RANGE || r > MAX_RANGE) continue;

      double angle_sensor = angle_min + i * angle_inc;
      double angle_base   = angle_sensor - LASER_YAW_OFFSET;
      double angle_map    = angle_base + yaw;

      double ex = rx + r * std::cos(angle_map);
      double ey = ry + r * std::sin(angle_map);

      int mx = static_cast<int>((ex - map_origin_x) / map_res);
      int my = static_cast<int>((ey - map_origin_y) / map_res);
      if (mx < 0 || mx >= map_w || my < 0 || my >= map_h) {
        continue;
      }

      int map_idx = my * map_w + mx;
      int8_t occ = map_->data[map_idx];
      if (occ < 0) continue;   // unknown
      if (occ > 0) continue;   // known static obstacle

      int cmx = static_cast<int>((ex - c_origin_x) / c_res);
      int cmy = static_cast<int>((ey - c_origin_y) / c_res);
      if (cmx < 0 || cmx >= c_w || cmy < 0 || cmy >= c_h) {
        continue;
      }
      int cidx = cmy * c_w + cmx;
      int cost = static_cast<int>(costmap_->data[cidx]);

      if (cost <= 0) {
        continue;
      }

      if (isNearStaticMapObstacle(mx, my)) {
        continue;
      }

      ExtraPoint p;
      p.x = ex;
      p.y = ey;
      extra_points_.push_back(p);
    }
  }

  // Checks if moved humans were found
  void clusterExtraPointsAndLog()
  {
    if (extra_points_.empty()) {
      RCLCPP_WARN(get_logger(),
                  "No extra obstacle points were collected during search; "
                  "cannot estimate new human locations.");
      return;
    }

    // radius-based clustering
    struct Cluster {
      double sum_x{0.0};
      double sum_y{0.0};
      int    count{0};
    };

    std::vector<Cluster> clusters;
    clusters.reserve(64);

    constexpr double CLUSTER_RADIUS       = 0.8;   // meters
    constexpr double CLUSTER_RADIUS_SQ    = CLUSTER_RADIUS * CLUSTER_RADIUS;
    constexpr int    MIN_CLUSTER_POINTS   = 1;     // set to 1 for debugging
    constexpr int    MAX_CLUSTERS_TO_LOG  = 80;    // avoid spamming console

    for (const auto &pt : extra_points_) {
      bool assigned = false;

      for (auto &c : clusters) {
        const double cx = c.sum_x / std::max(1, c.count);
        const double cy = c.sum_y / std::max(1, c.count);

        const double dx = pt.x - cx;
        const double dy = pt.y - cy;
        const double dist_sq = dx * dx + dy * dy;

        if (dist_sq <= CLUSTER_RADIUS_SQ) {
          c.sum_x += pt.x;
          c.sum_y += pt.y;
          c.count += 1;
          assigned = true;
          break;
        }
      }

      if (!assigned) {
        Cluster c;
        c.sum_x = pt.x;
        c.sum_y = pt.y;
        c.count = 1;
        clusters.push_back(c);
      }
    }

    if (clusters.empty()) {
      RCLCPP_WARN(get_logger(),
                  "Clustering produced 0 clusters from %zu points.",
                  extra_points_.size());
      return;
    }

    // Compute centroids & filter small clusters
    struct ClusterInfo {
      double cx;
      double cy;
      int    count;
    };

    std::vector<ClusterInfo> cluster_infos;
    cluster_infos.reserve(clusters.size());

    int printed = 0;
    for (std::size_t i = 0; i < clusters.size(); ++i) {
      const auto &c = clusters[i];
      if (c.count < MIN_CLUSTER_POINTS) {
        continue;  // ignore small blobs
      }

      ClusterInfo info;
      info.cx    = c.sum_x / c.count;
      info.cy    = c.sum_y / c.count;
      info.count = c.count;
      cluster_infos.push_back(info);

      if (printed < MAX_CLUSTERS_TO_LOG) {
        RCLCPP_INFO(
          get_logger(),
          "FOUND moved-human cluster %zu at approx (x=%.2f, y=%.2f) in map frame (points=%d).",
          i, info.cx, info.cy, info.count);
        ++printed;
      }
    }

    if (clusters.size() > MAX_CLUSTERS_TO_LOG) {
      RCLCPP_WARN(get_logger(),
                  "Cluster list truncated at %d entries; there are more clusters not printed.",
                  MAX_CLUSTERS_TO_LOG);
    }

    if (cluster_infos.empty()) {
      RCLCPP_WARN(get_logger(),
                  "All clusters had < %d points; no reliable moved-human candidates.",
                  MIN_CLUSTER_POINTS);
      return;
    }

    // --- 3. Assign clusters to moved humans WITHOUT reusing clusters ---

    constexpr double MIN_MOVE_DIST = 0.5;   // ignore things too close to original
    constexpr double MAX_MOVE_DIST = 30.0;  // ignore absurdly far matches

    struct Candidate {
      int human_idx;
      int cluster_idx;
      double dist;
    };

    std::vector<Candidate> candidates;

    for (std::size_t ci = 0; ci < cluster_infos.size(); ++ci) {
      const auto &cl = cluster_infos[ci];

      for (std::size_t hi = 0; hi < humans_.size(); ++hi) {
        const auto &h = humans_[hi];
        if (h.state != HumanTrackingState::MOVED) {
          continue;
        }

        const double dx = cl.cx - h.x;
        const double dy = cl.cy - h.y;
        const double d  = std::hypot(dx, dy);

        if (d < MIN_MOVE_DIST || d > MAX_MOVE_DIST) {
          continue;
        }

        candidates.push_back(Candidate{
          static_cast<int>(hi),
          static_cast<int>(ci),
          d
        });
      }
    }

    if (candidates.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "No cluster was within [%.2f, %.2f] m of any moved human's original position.",
        MIN_MOVE_DIST, MAX_MOVE_DIST);
      return;
    }

    std::sort(
      candidates.begin(),
      candidates.end(),
      [](const Candidate &a, const Candidate &b) {
        return a.dist < b.dist;
      });

    std::vector<int>    best_cluster_for_human(humans_.size(), -1);
    std::vector<double> best_dist_for_human(humans_.size(),
                                            std::numeric_limits<double>::infinity());
    std::vector<bool>   cluster_used(cluster_infos.size(), false);

    for (const auto &cand : candidates) {
      const int hi = cand.human_idx;
      const int ci = cand.cluster_idx;

      if (humans_[hi].state != HumanTrackingState::MOVED) {
        continue;
      }
      if (cluster_used[ci]) {
        continue;
      }
      if (best_cluster_for_human[hi] != -1) {
        continue;
      }

      best_cluster_for_human[hi] = ci;
      best_dist_for_human[hi]    = cand.dist;
      cluster_used[ci]           = true;
    }

    for (std::size_t hi = 0; hi < humans_.size(); ++hi) {
      const auto &h = humans_[hi];
      if (h.state != HumanTrackingState::MOVED) {
        continue;
      }

      const int ci = best_cluster_for_human[hi];
      if (ci == -1) {
        RCLCPP_WARN(
          get_logger(),
          "Could not confidently match any cluster to Human %zu (original (%.2f, %.2f)).",
          hi, h.x, h.y);
        continue;
      }

      const auto &cl = cluster_infos[ci];
      const double dx = cl.cx - h.x;
      const double dy = cl.cy - h.y;
      const double d  = std::hypot(dx, dy);

      RCLCPP_INFO(
        get_logger(),
        "ESTIMATE: Human %zu new location ≈ (x=%.2f, y=%.2f) "
        "(moved %.2f m from original (%.2f, %.2f), using cluster %d).",
        hi, cl.cx, cl.cy, d, h.x, h.y, ci);
    }
  }

  // Main control loop
  void controlLoop()
  {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "ControlLoop state=%s, map_=%s, amcl=%s",
      STATE_NAMES[static_cast<size_t>(state_)],
      map_ ? "yes" : "no",
      last_amcl_pose_ ? "yes" : "no");

    switch (state_) {
      case State::WAIT_FOR_MAP_AND_POSE:
        break;

      case State::GOING_TO_FIRST_HUMAN:
        if (navigator_->IsTaskComplete()) {
          RCLCPP_INFO(get_logger(), "Reached vantage near first human (%d).", first_human_idx_);
          logEvent("PHASE1_GOAL_REACHED", first_human_idx_);

          current_human_idx_ = first_human_idx_;
          samples_total_ = 0;
          samples_hit_   = 0;
          check_start_time_ = now();
          state_ = State::CHECKING_FIRST_HUMAN;
        }
        break;

      case State::CHECKING_FIRST_HUMAN: {
        rclcpp::Duration elapsed = now() - check_start_time_;
        double elapsed_sec = elapsed.seconds();

        if (elapsed_sec < 3.0) {
          if (last_scan_) {
            samples_total_++;
            if (sampleHumanWindow(current_human_idx_)) {
              samples_hit_++;
            }
          }
        } else {
            // update human tracking state
          humans_[current_human_idx_].state = samples_hit_ > 0 ? HumanTrackingState::STILL : HumanTrackingState::MOVED;

          RCLCPP_INFO(
            get_logger(),
            "Human %d check over %.2f s: total=%d, hits=%d -> %s",
            current_human_idx_, elapsed_sec, samples_total_, samples_hit_,
              humans_[current_human_idx_].state == HumanTrackingState::STILL ? "STILL" : "MOVED");

          goToHumanVantage(second_human_idx_);
          state_ = State::GOING_TO_SECOND_HUMAN;
        }
        break;
      }

      case State::GOING_TO_SECOND_HUMAN:
        if (navigator_->IsTaskComplete()) {
          RCLCPP_INFO(get_logger(), "Reached vantage near second human (%d).", second_human_idx_);
          logEvent("PHASE1_GOAL_REACHED", second_human_idx_);

          current_human_idx_ = second_human_idx_;
          samples_total_ = 0;
          samples_hit_   = 0;
          check_start_time_ = now();
          state_ = State::CHECKING_SECOND_HUMAN;
        }
        break;

      case State::CHECKING_SECOND_HUMAN: {
        rclcpp::Duration elapsed = now() - check_start_time_;
        double elapsed_sec = elapsed.seconds();

        if (elapsed_sec < 3.0) {
          if (last_scan_) {
            samples_total_++;
            if (sampleHumanWindow(current_human_idx_)) {
              samples_hit_++;
            }
          }
        } else {
            // update human tracking state
          bool still = (samples_hit_ > 0);
          humans_[current_human_idx_].state = samples_hit_ > 0? HumanTrackingState::STILL : HumanTrackingState::MOVED;

          RCLCPP_INFO(
            get_logger(),
            "Human %d check over %.2f s: total=%d, hits=%d -> %s",
            current_human_idx_, elapsed_sec, samples_total_, samples_hit_,
            still ? "STILL" : "MOVED");

          for (std::size_t i = 0; i < humans_.size(); ++i) {
            auto &h = humans_[i];
            if (h.state == HumanTrackingState::MOVED) {
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

          bool any_moved = false;
          for (const auto &h : humans_) {
            if (h.state == HumanTrackingState::MOVED) { any_moved = true; break; }
          }

          if (!any_moved) {
            RCLCPP_INFO(get_logger(),
                        "No humans moved; Phase 2 search not required.");
            state_ = State::DONE;
          } else {
            RCLCPP_INFO(get_logger(),
                        "At least one human moved; waiting for global costmap before search.");
            extra_points_.clear();
            search_wp_index_ = 0;
            search_spin_started_ = false;
            state_ = State::WAIT_FOR_COSTMAP;
          }
        }
        break;
      }

      case State::WAIT_FOR_COSTMAP:
        if (costmap_) {
          RCLCPP_INFO(get_logger(),
                      "Global costmap is available; starting Phase 2 search.");
          sendSearchWaypointGoal(search_wp_index_);
          state_ = State::SEARCH_GOING_TO_WAYPOINT;
        } else {
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 3000,
            "Waiting for /global_costmap/costmap...");
        }
        break;

      case State::SEARCH_GOING_TO_WAYPOINT:
        if (navigator_->IsTaskComplete()) {
          RCLCPP_INFO(
            get_logger(),
            "SEARCH: Reached waypoint %d at (%.2f, %.2f).",
            search_wp_index_,
            search_waypoints_[search_wp_index_].position.x,
            search_waypoints_[search_wp_index_].position.y);
          logEvent("SEARCH_WP_GOAL_REACHED", search_wp_index_);

          search_spin_started_ = false;
          state_ = State::SEARCH_SPINNING_AT_WAYPOINT;
        }
        break;

      case State::SEARCH_SPINNING_AT_WAYPOINT:
        if (!search_spin_started_) {
          if (!navigator_->Spin(2.0 * M_PI)) {
            RCLCPP_ERROR(get_logger(),
                         "Spin request at search waypoint %d was NOT accepted.",
                         search_wp_index_);
            state_ = State::DONE;
            break;
          }
          search_spin_started_ = true;
          RCLCPP_INFO(
            get_logger(),
            "Started 360° spin to search at waypoint %d.",
            search_wp_index_);
          logEvent("SEARCH_SPIN_START", search_wp_index_);
        } else {
          // No need to call collectExtraPointsFromScan() here anymore;
          // scanCallback is already collecting points while moving and spinning.

          if (navigator_->IsTaskComplete()) {
            RCLCPP_INFO(
              get_logger(),
              "Completed spin at search waypoint %d.",
              search_wp_index_);
            logEvent("SEARCH_SPIN_DONE", search_wp_index_);

            search_wp_index_++;
            search_spin_started_ = false;

            if (static_cast<std::size_t>(search_wp_index_) < search_waypoints_.size()) {
              sendSearchWaypointGoal(search_wp_index_);
              state_ = State::SEARCH_GOING_TO_WAYPOINT;
            } else {
              RCLCPP_INFO(get_logger(),
                          "Finished all search waypoints; clustering %zu extra points.",
                          extra_points_.size());
              clusterExtraPointsAndLog();
              state_ = State::DONE;
            }
          }
        }
        break;

      case State::DONE:
      default:
        break;
    }
  }

  // ---------- Members ----------

  std::shared_ptr<Navigator> navigator_;

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  nav_msgs::msg::OccupancyGrid::SharedPtr costmap_;
  sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr last_amcl_pose_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;

  rclcpp::TimerBase::SharedPtr timer_;

  State state_;
  bool started_navigation_;

  rclcpp::Time check_start_time_;
  int current_human_idx_;
  int samples_total_;
  int samples_hit_;

  int first_human_idx_;
  int second_human_idx_;

  std::vector<HumanOriginal> humans_;

  std::vector<geometry_msgs::msg::Pose> search_waypoints_;
  int  search_wp_index_;
  bool search_spin_started_;
  std::vector<ExtraPoint> extra_points_;

  std::ofstream debug_log_;

  // Frame + availability flags
  std::string map_frame_;
  bool have_map_;
  bool have_costmap_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto navigator = std::make_shared<Navigator>(true, true);
  auto human_tracker = std::make_shared<HumanTrackerNode>(navigator);

  rclcpp::spin(human_tracker);

  rclcpp::shutdown();
  return 0;
}