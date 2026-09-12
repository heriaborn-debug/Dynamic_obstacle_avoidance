#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace
{

using SteadyClock = std::chrono::steady_clock;
constexpr double kPi = 3.14159265358979323846;
constexpr float kSnapshotMagic = 270728.0F;
constexpr std::size_t kSnapshotHeaderSize = 11U;

double normalizeAngle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & quaternion)
{
  const double sin_yaw =
    2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y);
  const double cos_yaw =
    1.0 - 2.0 *
    (quaternion.y * quaternion.y + quaternion.z * quaternion.z);
  return std::atan2(sin_yaw, cos_yaw);
}

std::unordered_map<std::string, double> parseScalarConfig(
  const std::string & path)
{
  std::unordered_map<std::string, double> values;
  std::ifstream stream(path);
  std::string line;
  while (std::getline(stream, line)) {
    const auto comment = line.find('#');
    if (comment != std::string::npos) {
      line.erase(comment);
    }
    const auto separator = line.find(':');
    if (separator == std::string::npos) {
      continue;
    }
    std::string key = line.substr(0, separator);
    std::string value = line.substr(separator + 1U);
    const auto trim = [](std::string & text) {
        const auto first = text.find_first_not_of(" \t\r\n");
        if (first == std::string::npos) {
          text.clear();
          return;
        }
        const auto last = text.find_last_not_of(" \t\r\n");
        text = text.substr(first, last - first + 1U);
      };
    trim(key);
    trim(value);
    if (key.empty() || value.empty()) {
      continue;
    }
    try {
      values[key] = std::stod(value);
    } catch (const std::exception &) {
      continue;
    }
  }
  return values;
}

double configValue(
  const std::unordered_map<std::string, double> & config,
  const std::string & key, double fallback)
{
  const auto found = config.find(key);
  return found == config.end() ? fallback : found->second;
}

struct Point
{
  float x;
  float y;
  float z;
};

struct Candidate
{
  std::vector<Point> points;
  double end_x{0.0};
  double end_y{0.0};
  double end_yaw{0.0};
  double velocity_x{0.0};
  double velocity_y{0.0};
  double velocity_yaw{0.0};
};

struct Stage
{
  double scale{1.0};
  double range{4.0};
  std::vector<std::size_t> retained_count;
  std::vector<double> end_x;
  std::vector<double> end_y;
  std::vector<double> end_yaw;
};

struct CloudSnapshot
{
  std::vector<Point> points;
  std::uint64_t generation{0U};
  SteadyClock::time_point received;
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
};

struct DynamicSnapshot
{
  bool valid{false};
  std::uint64_t generation{0U};
  std::size_t stage_count{0U};
  std::size_t candidate_count{0U};
  std::vector<float> costs;
  std::vector<float> escape_costs;
  std::vector<float> obstacles;
  std::vector<float> speed_limits;
  std::vector<float> reachability;
  SteadyClock::time_point received;
};

struct GoalInBase
{
  double x;
  double y;
  double yaw;
  double full_distance;
};

struct Selection
{
  int path_id{-1};
  int stage_index{-1};
  double score{std::numeric_limits<double>::infinity()};
  std::size_t blocked{0U};
  std::string mode{"blocked"};
};

struct StageCosts
{
  std::vector<int> blocked;
  std::vector<int> penalty1;
  std::vector<int> penalty2;
  std::vector<int> penalty3;
};

class FirstPlannerV2Cpp : public rclcpp::Node
{
public:
  FirstPlannerV2Cpp()
  : Node("m20_first_planner_v2")
  {
    path_folder_ = declare_parameter<std::string>("path_folder", "");
    config_path_ = declare_parameter<std::string>("localplanner_config", "");
    odom_topic_ = declare_parameter<std::string>(
      "odom_topic", "/m20/ground_truth");
    goal_topic_ = declare_parameter<std::string>("goal_topic", "/local_goal");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/NAV_POINTS");
    semantic_obstacle_cloud_ = declare_parameter<bool>(
      "semantic_obstacle_cloud", false);
    cmd_topic_ = declare_parameter<std::string>(
      "cmd_topic", "/m20/v2/cmd_vel_candidate");
    local_plan_topic_ = declare_parameter<std::string>(
      "local_plan_topic", "/m20/v2/local_plan");
    track_path_topic_ = declare_parameter<std::string>(
      "track_path_topic", "/m20/v2/track_path");
    local_grid_topic_ = declare_parameter<std::string>(
      "local_obstacle_grid_topic", "/m20/v2/local_obstacle_grid");
    stage_topic_ = declare_parameter<std::string>(
      "planner_stage_topic", "/m20/v2/planner_stage");
    selected_prediction_topic_ = declare_parameter<std::string>(
      "selected_prediction_markers_topic",
      "/m20/v2/first_selected_prediction_markers");
    local_scans_topic_ = declare_parameter<std::string>(
      "local_scans_topic", "/local_scans");
    dynamic_snapshot_topic_ = declare_parameter<std::string>(
      "dynamic_snapshot_topic", "/m20/v2/dynamic_scoring_snapshot");
    reset_topic_ = declare_parameter<std::string>(
      "reset_navigation_topic", "/m20/reset_navigation");

    control_frequency_ = declare_parameter<double>("control_frequency", 10.0);
    compute_deadline_ = declare_parameter<double>("compute_deadline", 0.080);
    dynamic_timeout_ = declare_parameter<double>("dynamic_cost_timeout", 0.35);
    snapshot_max_skew_ = declare_parameter<double>("snapshot_max_skew", 0.20);
    require_dynamic_input_ = declare_parameter<bool>(
      "require_dynamic_input", false);
    publish_zero_without_goal_ = declare_parameter<bool>(
      "publish_zero_without_goal", true);
    dynamic_block_cost_ = declare_parameter<double>(
      "dynamic_block_cost", 1000000.0);
    switch_score_margin_ = declare_parameter<double>(
      "switch_score_margin", 0.50);
    minimum_path_hold_time_ = declare_parameter<double>(
      "minimum_path_hold_time", 0.35);
    path_switch_penalty_ = declare_parameter<double>(
      "path_switch_penalty", 1.25);
    escape_min_speed_ = declare_parameter<double>(
      "escape_min_translation_speed", 0.15);
    escape_max_speed_ = declare_parameter<double>(
      "escape_max_translation_speed", 1.0);
    escape_goal_blend_ = declare_parameter<double>(
      "escape_goal_cost_blend", 0.05);
    minimum_escape_hold_time_ = declare_parameter<double>(
      "minimum_escape_hold_time", 0.60);
    dynamic_mask_padding_ = declare_parameter<double>(
      "dynamic_point_mask_padding", 0.15);
    fallen_height_threshold_ = declare_parameter<double>(
      "fallen_height_threshold", 0.25);
    enable_fallen_height_check_ = declare_parameter<bool>(
      "enable_fallen_height_check", true);
    prediction_horizon_ = declare_parameter<double>(
      "selected_prediction_horizon", 3.0);
    prediction_step_ = declare_parameter<double>(
      "selected_prediction_step", 0.1);

    path_scale_ = declare_parameter<double>("path_scale", 1.0);
    min_path_scale_ = declare_parameter<double>("min_path_scale", 0.75);
    path_scale_step_ = declare_parameter<double>("path_scale_step", 0.25);
    path_range_ = declare_parameter<double>("path_range", 4.0);
    min_path_range_ = declare_parameter<double>("min_path_range", 1.0);
    path_range_step_ = declare_parameter<double>("path_range_step", 0.5);
    obstacle_range_ = declare_parameter<double>("obstacle_range", 4.0);
    self_clear_x_ = declare_parameter<double>("self_clear_x", 0.45);
    self_clear_y_ = declare_parameter<double>("self_clear_y", 0.25);

    const auto config = parseScalarConfig(config_path_);
    goal_tolerance_ = configValue(
      config, "xy_tolerance",
      declare_parameter<double>("goal_tolerance", 0.35));
    yaw_tolerance_ = configValue(
      config, "yaw_tolerance",
      declare_parameter<double>("yaw_tolerance", 0.35));
    require_goal_yaw_ = declare_parameter<bool>("require_goal_yaw", false);
    max_goal_range_ = configValue(
      config, "local_point_dis",
      declare_parameter<double>("max_goal_range", 3.0));
    goal_weight_ = configValue(config, "weight_goal", 1.0);
    yaw_weight_ = configValue(config, "weight_yaw", 0.5);
    speed_weight_ = configValue(config, "weight_spdy", 0.2);
    obstacle_weights_ = {
      configValue(config, "weight_ob1", 0.6),
      configValue(config, "weight_ob2", 0.8),
      configValue(config, "weight_ob3", 1.0)};
    obstacle_height_threshold_ =
      configValue(config, "obstacleHeightThre", 0.10);
    point_per_path_threshold_ = static_cast<int>(
      configValue(config, "pointPerPathThre", 1.0));
    grid_occupancy_threshold_ = static_cast<int>(
      configValue(config, "grid_occu_num_", 2.0));
    grid_size_ = configValue(config, "grid_voxel_size", 0.05);
    grid_offset_x_ = configValue(config, "grid_voxel_offset_x", -1.475);
    grid_offset_y_ = configValue(config, "grid_voxel_offset_y", -1.975);
    grid_count_x_ = static_cast<int>(
      configValue(config, "grid_voxel_num_x", 90.0));
    grid_count_y_ = static_cast<int>(
      configValue(config, "grid_voxel_num_y", 80.0));
    max_speed_x_ = configValue(config, "maxSpeedX", 1.5);
    max_speed_y_ = configValue(config, "maxSpeedY", 0.6);
    max_yaw_ = configValue(config, "maxTheta", 1.0);
    reverse_penalty_ = declare_parameter<double>("reverse_penalty", 0.5);
    lateral_penalty_ = declare_parameter<double>("lateral_penalty", 0.15);

    validateParameters();
    buildStages();
    loadPaths();
    buildStageGeometry();
    loadCorrespondences();

    command_publisher_ = create_publisher<geometry_msgs::msg::Twist>(
      cmd_topic_, 10);
    path_publisher_ = create_publisher<nav_msgs::msg::Path>(
      local_plan_topic_, 10);
    track_path_publisher_ = create_publisher<nav_msgs::msg::Path>(
      track_path_topic_, 10);
    grid_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      local_grid_topic_, 10);
    stage_publisher_ = create_publisher<std_msgs::msg::String>(stage_topic_, 10);
    scans_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      local_scans_topic_, rclcpp::SensorDataQoS());
    prediction_publisher_ =
      create_publisher<visualization_msgs::msg::MarkerArray>(
      selected_prediction_topic_, 10);

    data_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    planning_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions data_options;
    data_options.callback_group = data_group_;

    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(20),
      std::bind(&FirstPlannerV2Cpp::onOdometry, this, std::placeholders::_1),
      data_options);
    goal_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic_, rclcpp::QoS(10),
      std::bind(&FirstPlannerV2Cpp::onGoal, this, std::placeholders::_1),
      data_options);
    cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&FirstPlannerV2Cpp::onCloud, this, std::placeholders::_1),
      data_options);
    dynamic_subscription_ =
      create_subscription<std_msgs::msg::Float32MultiArray>(
      dynamic_snapshot_topic_, rclcpp::QoS(10),
      std::bind(
        &FirstPlannerV2Cpp::onDynamicSnapshot, this, std::placeholders::_1),
      data_options);
    reset_subscription_ = create_subscription<std_msgs::msg::Bool>(
      reset_topic_, rclcpp::QoS(10),
      std::bind(&FirstPlannerV2Cpp::onReset, this, std::placeholders::_1),
      data_options);

    planning_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / control_frequency_),
      std::bind(&FirstPlannerV2Cpp::planOnce, this), planning_group_);
    selected_since_ = SteadyClock::now();
    RCLCPP_INFO(
      get_logger(),
      "FIRST V2 C++ loaded %zu paths, %zu stages; deadline=%.1f ms, "
      "dynamic_snapshot=%s",
      candidates_.size(), stages_.size(), compute_deadline_ * 1000.0,
      dynamic_snapshot_topic_.c_str());
  }

private:
  void validateParameters() const
  {
    if (
      path_folder_.empty() || control_frequency_ <= 0.0 ||
      compute_deadline_ <= 0.0 || grid_size_ <= 0.0 ||
      grid_count_x_ <= 0 || grid_count_y_ <= 0 ||
      path_scale_ <= 0.0 || min_path_scale_ <= 0.0 ||
      path_scale_step_ <= 0.0 || path_range_ <= 0.0 ||
      min_path_range_ <= 0.0 || path_range_step_ <= 0.0 ||
      minimum_escape_hold_time_ < 0.0 ||
      fallen_height_threshold_ <= 0.0 ||
      prediction_horizon_ <= 0.0 || prediction_step_ <= 0.0)
    {
      throw std::runtime_error("Invalid FIRST V2 C++ configuration");
    }
  }

  void buildStages()
  {
    stages_.push_back({path_scale_, path_range_});
    double scale = path_scale_;
    double range = path_range_;
    while (
      scale >= min_path_scale_ - 1.0e-9 &&
      range >= min_path_range_ - 1.0e-9)
    {
      if (scale >= min_path_scale_ + path_scale_step_ - 1.0e-9) {
        scale = std::max(min_path_scale_, scale - path_scale_step_);
        range = path_range_ * scale / path_scale_;
      } else {
        range -= path_range_step_;
      }
      if (range < min_path_range_ - 1.0e-9) {
        break;
      }
      if (
        std::abs(stages_.back().scale - scale) > 1.0e-9 ||
        std::abs(stages_.back().range - range) > 1.0e-9)
      {
        stages_.push_back({scale, range});
      }
    }
  }

  void loadPaths()
  {
    std::ifstream path_stream(path_folder_ + "/pathList.ply");
    if (!path_stream) {
      throw std::runtime_error("Cannot open FIRST pathList.ply");
    }
    std::unordered_map<int, std::vector<Point>> path_points;
    double x;
    double y;
    double z;
    int id;
    while (path_stream >> x >> y >> z >> id) {
      path_points[id].push_back(
        {static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)});
    }

    std::ifstream end_stream(path_folder_ + "/path_end.ply");
    if (!end_stream) {
      throw std::runtime_error("Cannot open FIRST path_end.ply");
    }
    struct EndRecord
    {
      double x;
      double y;
      double z;
      double yaw;
      double vx;
      double vy;
      double wz;
      int id;
    };
    std::vector<EndRecord> records;
    EndRecord record{};
    while (
      end_stream >> record.x >> record.y >> record.z >> record.yaw >>
      record.vx >> record.vy >> record.wz >> record.id)
    {
      records.push_back(record);
    }
    int maximum_id = -1;
    for (const auto & item : records) {
      maximum_id = std::max(maximum_id, item.id);
    }
    candidates_.resize(static_cast<std::size_t>(maximum_id + 1));
    valid_candidate_.assign(candidates_.size(), false);
    for (const auto & item : records) {
      const auto found = path_points.find(item.id);
      if (found == path_points.end()) {
        continue;
      }
      auto & candidate = candidates_[static_cast<std::size_t>(item.id)];
      candidate.points = found->second;
      candidate.end_x = item.x;
      candidate.end_y = item.y;
      candidate.end_yaw = item.yaw;
      candidate.velocity_x = item.vx;
      candidate.velocity_y = item.vy;
      candidate.velocity_yaw = item.wz;
      valid_candidate_[static_cast<std::size_t>(item.id)] = true;
    }
  }

  void buildStageGeometry()
  {
    for (auto & stage : stages_) {
      stage.retained_count.resize(candidates_.size(), 0U);
      stage.end_x.resize(candidates_.size(), 0.0);
      stage.end_y.resize(candidates_.size(), 0.0);
      stage.end_yaw.resize(candidates_.size(), 0.0);
      for (std::size_t id = 0; id < candidates_.size(); ++id) {
        const auto & candidate = candidates_[id];
        if (!valid_candidate_[id] || candidate.points.empty()) {
          continue;
        }
        for (const auto & point : candidate.points) {
          const double scaled_x = stage.scale * point.x;
          const double scaled_y = stage.scale * point.y;
          if (std::hypot(scaled_x, scaled_y) > stage.range + 1.0e-9) {
            break;
          }
          stage.end_x[id] = scaled_x;
          stage.end_y[id] = scaled_y;
          ++stage.retained_count[id];
        }
        if (stage.retained_count[id] == candidate.points.size()) {
          stage.end_yaw[id] = candidate.end_yaw;
        } else {
          stage.end_yaw[id] =
            candidate.velocity_yaw *
            static_cast<double>(stage.retained_count[id]) * 0.1;
        }
      }
    }
  }

  void loadCorrespondences()
  {
    const std::array<std::string, 4> names{
      "correspondences.ply", "correspondences_4.ply",
      "correspondences_5.ply", "correspondences_6.ply"};
    correspondence_.resize(names.size());
    const std::size_t voxel_count =
      static_cast<std::size_t>(grid_count_x_ * grid_count_y_);
    for (std::size_t layer = 0; layer < names.size(); ++layer) {
      correspondence_[layer].resize(voxel_count);
      std::ifstream stream(path_folder_ + "/" + names[layer]);
      if (!stream) {
        throw std::runtime_error("Cannot open FIRST " + names[layer]);
      }
      std::string line;
      while (std::getline(stream, line)) {
        std::istringstream parser(line);
        int voxel = -1;
        parser >> voxel;
        if (voxel < 0 || static_cast<std::size_t>(voxel) >= voxel_count) {
          continue;
        }
        int path_id = -1;
        while (parser >> path_id && path_id >= 0) {
          if (static_cast<std::size_t>(path_id) < candidates_.size()) {
            correspondence_[layer][static_cast<std::size_t>(voxel)]
              .push_back(path_id);
          }
        }
      }
    }
  }

  void onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr message)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_odometry_ = message;
    ++odometry_generation_;
  }

  void onGoal(const geometry_msgs::msg::PoseStamped::ConstSharedPtr message)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (latest_goal_) {
      const auto & previous = *latest_goal_;
      const double position_delta = std::hypot(
        message->pose.position.x - previous.pose.position.x,
        message->pose.position.y - previous.pose.position.y);
      const double yaw_delta = std::abs(normalizeAngle(
        yawFromQuaternion(message->pose.orientation) -
        yawFromQuaternion(previous.pose.orientation)));
      if (
        message->header.frame_id == previous.header.frame_id &&
        position_delta < 1.0e-4 && yaw_delta < 1.0e-4)
      {
        return;
      }
    }
    latest_goal_ = message;
    ++goal_generation_;
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "FIRST V2 C++ goal frame=%s x=%.2f y=%.2f",
      message->header.frame_id.c_str(), message->pose.position.x,
      message->pose.position.y);
  }

  void onCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
  {
    auto snapshot = std::make_shared<CloudSnapshot>();
    snapshot->received = SteadyClock::now();
    snapshot->stamp = rclcpp::Time(message->header.stamp);
    snapshot->points.reserve(
      static_cast<std::size_t>(message->width) * message->height);
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(*message, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(*message, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(*message, "z");
      for (; x != x.end(); ++x, ++y, ++z) {
        if (std::isfinite(*x) && std::isfinite(*y) && std::isfinite(*z)) {
          snapshot->points.push_back({*x, *y, *z});
        }
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Cannot decode /NAV_POINTS: %s", error.what());
      return;
    }
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      snapshot->generation = ++cloud_generation_;
      latest_cloud_ = snapshot;
    }
    scans_publisher_->publish(*message);
  }

  void onDynamicSnapshot(
    const std_msgs::msg::Float32MultiArray::ConstSharedPtr message)
  {
    if (
      message->data.size() < kSnapshotHeaderSize ||
      std::abs(message->data[0] - kSnapshotMagic) > 0.5F ||
      std::abs(message->data[1] - 1.0F) > 0.1F)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejected malformed dynamic scoring snapshot");
      return;
    }
    auto snapshot = std::make_shared<DynamicSnapshot>();
    snapshot->generation =
      static_cast<std::uint64_t>(std::llround(message->data[2]));
    snapshot->valid = message->data[3] > 0.5F;
    snapshot->stage_count =
      static_cast<std::size_t>(std::llround(message->data[4]));
    snapshot->candidate_count =
      static_cast<std::size_t>(std::llround(message->data[5]));
    std::array<std::size_t, 5> sizes{};
    std::size_t expected = kSnapshotHeaderSize;
    for (std::size_t index = 0; index < sizes.size(); ++index) {
      sizes[index] = static_cast<std::size_t>(
        std::llround(message->data[6U + index]));
      expected += sizes[index];
    }
    if (message->data.size() != expected) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejected torn dynamic snapshot: received=%zu expected=%zu",
        message->data.size(), expected);
      return;
    }
    std::size_t cursor = kSnapshotHeaderSize;
    auto take = [&](std::size_t size, std::vector<float> & output) {
        output.assign(
          message->data.begin() + static_cast<std::ptrdiff_t>(cursor),
          message->data.begin() +
          static_cast<std::ptrdiff_t>(cursor + size));
        cursor += size;
      };
    take(sizes[0], snapshot->costs);
    take(sizes[1], snapshot->escape_costs);
    take(sizes[2], snapshot->obstacles);
    take(sizes[3], snapshot->speed_limits);
    take(sizes[4], snapshot->reachability);
    snapshot->received = SteadyClock::now();
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      latest_dynamic_ = snapshot;
      dynamic_generation_ = snapshot->generation;
    }
  }

  void onReset(const std_msgs::msg::Bool::ConstSharedPtr message)
  {
    if (!message->data) {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      latest_goal_.reset();
      ++goal_generation_;
    }
    reset_pending_.store(true);
    publishCommand(0.0, 0.0, 0.0, nullptr);
    publishEmptyPath();
  }

  std::optional<int> voxelId(double x, double y) const
  {
    const int ix = static_cast<int>(
      std::floor((x - grid_offset_x_) / grid_size_));
    const int iy = static_cast<int>(
      std::floor((y - grid_offset_y_) / grid_size_));
    if (ix < 0 || iy < 0 || ix >= grid_count_x_ || iy >= grid_count_y_) {
      return std::nullopt;
    }
    return ix * grid_count_y_ + iy;
  }

  std::vector<int> occupiedVoxels(
    const CloudSnapshot & cloud, const DynamicSnapshot * dynamic,
    std::size_t & filtered_points) const
  {
    std::unordered_map<int, int> counts;
    counts.reserve(cloud.points.size() / 4U + 1U);
    filtered_points = 0U;
    for (const auto & point : cloud.points) {
      const double distance = std::hypot(point.x, point.y);
      if (
        (std::abs(point.x) <= self_clear_x_ &&
        std::abs(point.y) <= self_clear_y_) ||
        distance < 0.18 || distance > obstacle_range_ ||
        (!semantic_obstacle_cloud_ &&
        (point.z < obstacle_height_threshold_ || point.z > 1.2)))
      {
        continue;
      }
      bool dynamic_hit = false;
      if (dynamic != nullptr) {
        for (std::size_t index = 0; index + 2U < dynamic->obstacles.size();
          index += 3U)
        {
          const double radius =
            std::max(0.0, static_cast<double>(dynamic->obstacles[index + 2U]) +
            dynamic_mask_padding_);
          const double dx = point.x - dynamic->obstacles[index];
          const double dy = point.y - dynamic->obstacles[index + 1U];
          if (dx * dx + dy * dy <= radius * radius) {
            dynamic_hit = true;
            break;
          }
        }
      }
      if (dynamic_hit) {
        continue;
      }
      const auto voxel = voxelId(point.x, point.y);
      if (voxel) {
        ++counts[*voxel];
        ++filtered_points;
      }
    }
    std::vector<int> occupied;
    occupied.reserve(counts.size());
    const int occupancy_threshold = semantic_obstacle_cloud_ ? 1 : grid_occupancy_threshold_;
    for (const auto & item : counts) {
      if (item.second >= occupancy_threshold) {
        occupied.push_back(item.first);
      }
    }
    return occupied;
  }

  std::vector<StageCosts> buildStaticCosts(
    const std::vector<int> & occupied) const
  {
    std::vector<StageCosts> output(stages_.size());
    for (std::size_t stage_index = 0; stage_index < stages_.size();
      ++stage_index)
    {
      auto & costs = output[stage_index];
      costs.blocked.assign(candidates_.size(), 0);
      costs.penalty1.assign(candidates_.size(), 0);
      costs.penalty2.assign(candidates_.size(), 0);
      costs.penalty3.assign(candidates_.size(), 0);
      std::unordered_set<int> stage_voxels;
      stage_voxels.reserve(occupied.size());
      const auto & stage = stages_[stage_index];
      for (const int voxel : occupied) {
        const int ix = voxel / grid_count_y_;
        const int iy = voxel % grid_count_y_;
        const double center_x =
          grid_offset_x_ + (static_cast<double>(ix) + 0.5) * grid_size_;
        const double center_y =
          grid_offset_y_ + (static_cast<double>(iy) + 0.5) * grid_size_;
        if (std::hypot(center_x, center_y) > stage.range + 1.0e-9) {
          continue;
        }
        const auto template_voxel = voxelId(
          center_x / stage.scale, center_y / stage.scale);
        if (template_voxel) {
          stage_voxels.insert(*template_voxel);
        }
      }
      std::array<std::vector<int> *, 4> targets{
        &costs.blocked, &costs.penalty1, &costs.penalty2,
        &costs.penalty3};
      for (const int voxel : stage_voxels) {
        for (std::size_t layer = 0; layer < correspondence_.size(); ++layer) {
          for (const int path_id :
            correspondence_[layer][static_cast<std::size_t>(voxel)])
          {
            ++(*targets[layer])[static_cast<std::size_t>(path_id)];
          }
        }
      }
    }
    return output;
  }

  std::optional<GoalInBase> goalInBase(
    const geometry_msgs::msg::PoseStamped & goal,
    const nav_msgs::msg::Odometry & odometry) const
  {
    std::string frame = goal.header.frame_id;
    if (!frame.empty() && frame.front() == '/') {
      frame.erase(frame.begin());
    }
    if (
      frame == base_frame_ || frame == "base_link" || frame == "base_footprint" ||
      frame == "vehicle" || frame == "body" || frame == "m20/base_link")
    {
      double local_x = goal.pose.position.x;
      double local_y = goal.pose.position.y;
      const double distance = std::hypot(local_x, local_y);
      if (distance > max_goal_range_) {
        local_x *= max_goal_range_ / distance;
        local_y *= max_goal_range_ / distance;
      }
      return GoalInBase{
        local_x, local_y, yawFromQuaternion(goal.pose.orientation), distance};
    }
    const auto & pose = odometry.pose.pose;
    const double robot_yaw = yawFromQuaternion(pose.orientation);
    const double dx = goal.pose.position.x - pose.position.x;
    const double dy = goal.pose.position.y - pose.position.y;
    double local_x = std::cos(robot_yaw) * dx + std::sin(robot_yaw) * dy;
    double local_y = -std::sin(robot_yaw) * dx + std::cos(robot_yaw) * dy;
    const double distance = std::hypot(local_x, local_y);
    if (distance > max_goal_range_) {
      local_x *= max_goal_range_ / distance;
      local_y *= max_goal_range_ / distance;
    }
    return GoalInBase{
      local_x, local_y,
      normalizeAngle(yawFromQuaternion(goal.pose.orientation) - robot_yaw),
      distance};
  }

  double candidateScore(
    std::size_t path_id, std::size_t stage_index,
    const GoalInBase & goal, const StageCosts & static_cost,
    const DynamicSnapshot * dynamic, bool & blocked) const
  {
    const auto & candidate = candidates_[path_id];
    const auto & stage = stages_[stage_index];
    blocked =
      static_cost.blocked[path_id] >= point_per_path_threshold_;
    double dynamic_cost = 0.0;
    if (dynamic != nullptr) {
      const std::size_t index =
        stage_index * candidates_.size() + path_id;
      if (index >= dynamic->costs.size()) {
        blocked = true;
      } else {
        dynamic_cost = dynamic->costs[index];
        blocked = blocked || dynamic_cost >= dynamic_block_cost_;
      }
    }
    if (blocked || !valid_candidate_[path_id]) {
      return std::numeric_limits<double>::infinity();
    }
    const double vx = std::clamp(
      stage.scale * candidate.velocity_x, -max_speed_x_, max_speed_x_);
    const double vy = std::clamp(
      stage.scale * candidate.velocity_y, -max_speed_y_, max_speed_y_);
    const double distance_cost = std::hypot(
      goal.x - stage.end_x[path_id], goal.y - stage.end_y[path_id]);
    const double desired_heading = std::atan2(goal.y, goal.x);
    const double heading_cost = std::abs(normalizeAngle(
      desired_heading - stage.end_yaw[path_id]));
    const double goal_yaw_cost = goal.full_distance < 0.9 ?
      std::abs(normalizeAngle(goal.yaw - stage.end_yaw[path_id])) : 0.0;
    const double samples = std::max<std::size_t>(
      1U, stage.retained_count[path_id]);
    const double obstacle_cost =
      (obstacle_weights_[0] * static_cost.penalty1[path_id] +
      obstacle_weights_[1] * static_cost.penalty2[path_id] +
      obstacle_weights_[2] * static_cost.penalty3[path_id]) /
      static_cast<double>(samples);
    double score =
      goal_weight_ * distance_cost +
      yaw_weight_ * heading_cost +
      0.7 * goal_yaw_cost -
      speed_weight_ * std::hypot(vx, vy) +
      reverse_penalty_ * std::max(0.0, -vx) +
      lateral_penalty_ * std::abs(vy) +
      obstacle_cost +
      std::min(dynamic_cost, dynamic_block_cost_);
    if (selected_path_id_ >= 0 &&
      static_cast<std::size_t>(selected_path_id_) != path_id)
    {
      score += path_switch_penalty_;
    }
    return score;
  }

  Selection selectPath(
    const GoalInBase & goal, const std::vector<StageCosts> & static_costs,
    const DynamicSnapshot * dynamic)
  {
    Selection best;
    for (std::size_t stage_index = 0; stage_index < stages_.size();
      ++stage_index)
    {
      Selection stage_best;
      stage_best.stage_index = static_cast<int>(stage_index);
      for (std::size_t path_id = 0; path_id < candidates_.size(); ++path_id) {
        bool blocked = false;
        const double score = candidateScore(
          path_id, stage_index, goal, static_costs[stage_index],
          dynamic, blocked);
        stage_best.blocked += blocked ? 1U : 0U;
        if (score < stage_best.score) {
          stage_best.path_id = static_cast<int>(path_id);
          stage_best.score = score;
        }
      }
      if (stage_best.path_id >= 0) {
        stage_best.mode = stage_index == 0U ? "normal" :
          (stages_[stage_index].scale <
          stages_[stage_index - 1U].scale ? "path_scale" : "path_range");
        best = stage_best;
        break;
      }
      best.blocked = stage_best.blocked;
    }

    const bool intrusion =
      dynamic != nullptr && dynamic->reachability.size() >= 5U &&
      (dynamic->reachability[3] > 0.5F ||
      dynamic->reachability[4] > 0.5F);

    Selection escape;
    escape.mode = intrusion ? "intrusion_escape" : "escape";
    escape.stage_index = 0;
    if (dynamic != nullptr &&
      dynamic->escape_costs.size() == candidates_.size())
    {
      for (std::size_t path_id = 0; path_id < candidates_.size(); ++path_id) {
        if (
          static_costs[0].blocked[path_id] >= point_per_path_threshold_ ||
          !std::isfinite(dynamic->escape_costs[path_id]))
        {
          continue;
        }
        const auto & candidate = candidates_[path_id];
        const double speed = std::hypot(
          candidate.velocity_x, candidate.velocity_y);
        if (speed < escape_min_speed_ || speed > escape_max_speed_) {
          continue;
        }
        const double goal_cost = std::hypot(
          goal.x - stages_[0].end_x[path_id],
          goal.y - stages_[0].end_y[path_id]);
        const double score =
          dynamic->escape_costs[path_id] +
          escape_goal_blend_ * goal_weight_ * goal_cost;
        if (score < escape.score) {
          escape.path_id = static_cast<int>(path_id);
          escape.score = score;
        }
      }
    }

    // A collision with the stationary/braking trajectory means remaining at
    // the current pose is itself unsafe. In that state, a verified escape
    // candidate must compete before ordinary goal following, even when some
    // ordinary FIRST paths remain geometrically open.
    if (intrusion && escape.path_id >= 0) {
      const double held_for = std::chrono::duration<double>(
        SteadyClock::now() - selected_since_).count();
      if (
        selected_mode_.find("intrusion_escape") == 0U &&
        selected_path_id_ >= 0 &&
        static_cast<std::size_t>(selected_path_id_) <
        dynamic->escape_costs.size() &&
        std::isfinite(
          dynamic->escape_costs[
            static_cast<std::size_t>(selected_path_id_)]) &&
        static_costs[0].blocked[
          static_cast<std::size_t>(selected_path_id_)] <
        point_per_path_threshold_ &&
        held_for < minimum_escape_hold_time_)
      {
        escape.path_id = selected_path_id_;
        escape.score = dynamic->escape_costs[
          static_cast<std::size_t>(selected_path_id_)];
        escape.mode = "intrusion_escape_held";
      }
      return escape;
    }
    if (best.path_id < 0 && escape.path_id >= 0) {
      best = escape;
    }

    // Commit hysteresis is bypassed for an unsafe previous path or an active
    // intrusion. Otherwise a small score fluctuation cannot reverse command
    // direction on every sensor frame.
    if (
      best.path_id >= 0 && selected_path_id_ >= 0 &&
      selected_stage_index_ >= 0 && !intrusion &&
      static_cast<std::size_t>(selected_stage_index_) < static_costs.size())
    {
      bool previous_blocked = false;
      const double previous_score = candidateScore(
        static_cast<std::size_t>(selected_path_id_),
        static_cast<std::size_t>(selected_stage_index_), goal,
        static_costs[static_cast<std::size_t>(selected_stage_index_)],
        dynamic, previous_blocked);
      const double held_for = std::chrono::duration<double>(
        SteadyClock::now() - selected_since_).count();
      if (
        !previous_blocked && std::isfinite(previous_score) &&
        (held_for < minimum_path_hold_time_ ||
        previous_score <= best.score + switch_score_margin_))
      {
        best.path_id = selected_path_id_;
        best.stage_index = selected_stage_index_;
        best.score = previous_score;
        best.mode = "held";
      }
    }
    return best;
  }

  bool snapshotStillCurrent(
    std::uint64_t cloud_generation, std::uint64_t dynamic_generation,
    std::uint64_t goal_generation) const
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return cloud_generation_ == cloud_generation &&
           dynamic_generation_ == dynamic_generation &&
           goal_generation_ == goal_generation;
  }

  void planOnce()
  {
    const auto started = SteadyClock::now();
    std::shared_ptr<const CloudSnapshot> cloud;
    std::shared_ptr<const DynamicSnapshot> dynamic;
    nav_msgs::msg::Odometry::ConstSharedPtr odometry;
    geometry_msgs::msg::PoseStamped::ConstSharedPtr goal;
    std::uint64_t cloud_generation;
    std::uint64_t dynamic_generation;
    std::uint64_t goal_generation;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      cloud = latest_cloud_;
      dynamic = latest_dynamic_;
      odometry = latest_odometry_;
      goal = latest_goal_;
      cloud_generation = cloud_generation_;
      dynamic_generation = dynamic_generation_;
      goal_generation = goal_generation_;
    }
    if (reset_pending_.exchange(false)) {
      selected_path_id_ = -1;
      selected_stage_index_ = -1;
      selected_mode_ = "idle";
      selected_since_ = SteadyClock::now();
    }
    if (!goal || !odometry) {
      if (publish_zero_without_goal_) {
        publishCommand(0.0, 0.0, 0.0, nullptr);
      }
      publishStage("idle");
      return;
    }
    if (
      enable_fallen_height_check_ &&
      odometry->pose.pose.position.z < fallen_height_threshold_)
    {
      publishCommand(0.0, 0.0, 0.0, nullptr);
      publishStopPath();
      publishStage("fallen");
      return;
    }
    if (!cloud) {
      publishCommand(0.0, 0.0, 0.0, nullptr);
      publishStage("waiting_for_cloud");
      return;
    }
    const double dynamic_age = dynamic ?
      std::chrono::duration<double>(started - dynamic->received).count() :
      std::numeric_limits<double>::infinity();
    const double cloud_dynamic_skew = dynamic ?
      std::abs(std::chrono::duration<double>(
        cloud->received - dynamic->received).count()) :
      std::numeric_limits<double>::infinity();
    const bool dynamic_usable =
      dynamic && dynamic->valid &&
      dynamic->stage_count == stages_.size() &&
      dynamic->candidate_count == candidates_.size() &&
      dynamic->costs.size() == stages_.size() * candidates_.size() &&
      dynamic_age <= dynamic_timeout_ &&
      cloud_dynamic_skew <= snapshot_max_skew_;
    if (require_dynamic_input_ && !dynamic_usable) {
      publishCommand(0.0, 0.0, 0.0, nullptr);
      publishStage("dynamic_snapshot_unavailable");
      return;
    }
    const DynamicSnapshot * dynamic_data =
      dynamic_usable ? dynamic.get() : nullptr;
    const auto local_goal = goalInBase(*goal, *odometry);
    if (!local_goal) {
      return;
    }
    const bool yaw_ok =
      !require_goal_yaw_ || std::abs(local_goal->yaw) < yaw_tolerance_;
    if (local_goal->full_distance < goal_tolerance_ && yaw_ok) {
      publishCommand(0.0, 0.0, 0.0, dynamic_data);
      publishEmptyPath();
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (goal_generation_ == goal_generation) {
          latest_goal_.reset();
          ++goal_generation_;
      selected_path_id_ = -1;
      selected_stage_index_ = -1;
      selected_mode_ = "goal_reached";
        }
      }
      publishStage("goal_reached");
      return;
    }

    std::size_t filtered_points = 0U;
    const auto occupied = occupiedVoxels(
      *cloud, dynamic_data, filtered_points);
    const auto static_costs = buildStaticCosts(occupied);
    Selection selection = selectPath(
      *local_goal, static_costs, dynamic_data);
    const double elapsed =
      std::chrono::duration<double>(SteadyClock::now() - started).count();
    const bool current = snapshotStillCurrent(
      cloud_generation, dynamic_generation, goal_generation);
    if (!current || elapsed > compute_deadline_) {
      ++discarded_snapshots_;
      publishStage(
        !current ? "snapshot_superseded" : "snapshot_deadline_missed");
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "FIRST C++ discarded result current=%s compute=%.2f ms "
        "discarded=%lu",
        current ? "yes" : "no", elapsed * 1000.0,
        static_cast<unsigned long>(discarded_snapshots_.load()));
      return;
    }

    publishGrid(occupied);
    if (selection.path_id < 0) {
      publishCommand(0.0, 0.0, 0.0, dynamic_data);
      publishStopPath();
      publishStage("all_paths_blocked");
    } else {
      const auto & candidate =
        candidates_[static_cast<std::size_t>(selection.path_id)];
      const auto & stage =
        stages_[static_cast<std::size_t>(selection.stage_index)];
      double vx = candidate.velocity_x;
      double vy = candidate.velocity_y;
      if (
        selection.mode != "escape" &&
        selection.mode != "intrusion_escape")
      {
        vx *= stage.scale;
        vy *= stage.scale;
      }
      vx = std::clamp(vx, -max_speed_x_, max_speed_x_);
      vy = std::clamp(vy, -max_speed_y_, max_speed_y_);
      const double wz = std::clamp(
        candidate.velocity_yaw, -max_yaw_, max_yaw_);
      publishCommand(vx, vy, wz, dynamic_data);
      const bool selection_changed =
        selected_path_id_ != selection.path_id ||
        selected_stage_index_ != selection.stage_index;
      selected_mode_ = selection.mode;
      publishPath(selection.path_id, selection.stage_index);
      if (selection_changed) {
        selected_since_ = SteadyClock::now();
      }
      selected_path_id_ = selection.path_id;
      selected_stage_index_ = selection.stage_index;
      publishStage(selection.mode);
    }
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "FIRST C++ mode=%s path=%d stage=%d score=%.3f blocked=%zu/%zu "
      "cloud_gen=%lu dynamic_gen=%lu voxels=%zu points=%zu compute=%.2f ms",
      selection.mode.c_str(), selection.path_id, selection.stage_index,
      selection.score, selection.blocked, candidates_.size(),
      static_cast<unsigned long>(cloud_generation),
      static_cast<unsigned long>(dynamic_generation), occupied.size(),
      filtered_points, elapsed * 1000.0);
  }

  void publishCommand(
    double vx, double vy, double wz, const DynamicSnapshot * dynamic)
  {
    if (dynamic != nullptr && dynamic->speed_limits.size() >= 3U) {
      vx = std::clamp(
        vx, -static_cast<double>(dynamic->speed_limits[0]),
        static_cast<double>(dynamic->speed_limits[0]));
      vy = std::clamp(
        vy, -static_cast<double>(dynamic->speed_limits[1]),
        static_cast<double>(dynamic->speed_limits[1]));
      wz = std::clamp(
        wz, -static_cast<double>(dynamic->speed_limits[2]),
        static_cast<double>(dynamic->speed_limits[2]));
    }
    geometry_msgs::msg::Twist command;
    command.linear.x = vx;
    command.linear.y = vy;
    command.angular.z = wz;
    command_publisher_->publish(command);
  }

  void publishPath(int path_id, int stage_index)
  {
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = base_frame_;
    const auto & candidate = candidates_[static_cast<std::size_t>(path_id)];
    const auto & stage = stages_[static_cast<std::size_t>(stage_index)];
    const std::size_t count =
      stage.retained_count[static_cast<std::size_t>(path_id)];
    for (std::size_t index = 0; index < count; ++index) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = stage.scale * candidate.points[index].x;
      pose.pose.position.y = stage.scale * candidate.points[index].y;
      pose.pose.position.z = stage.scale * candidate.points[index].z;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
    path_publisher_->publish(path);
    track_path_publisher_->publish(path);
    publishPrediction(path, candidate, stage);
  }

  void publishPrediction(
    const nav_msgs::msg::Path & path, const Candidate & candidate,
    const Stage & stage)
  {
    visualization_msgs::msg::MarkerArray markers;

    visualization_msgs::msg::Marker clear;
    clear.header = path.header;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);

    visualization_msgs::msg::Marker status;
    status.header = path.header;
    status.ns = "first_v2_stage";
    status.id = 100;
    status.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    status.action = visualization_msgs::msg::Marker::ADD;
    status.pose.position.z = 0.85;
    status.pose.orientation.w = 1.0;
    status.scale.z = 0.24;
    status.color.r = 0.10F;
    status.color.g =
      selected_mode_.find("escape") == std::string::npos ? 0.75F : 0.20F;
    status.color.b =
      selected_mode_.find("escape") == std::string::npos ? 1.0F : 0.15F;
    status.color.a = 1.0F;
    std::ostringstream status_text;
    status_text << "V2 C++ " << selected_mode_ << " scale "
                << stage.scale << " range " << stage.range;
    status.text = status_text.str();
    status.lifetime = rclcpp::Duration::from_seconds(0.3);
    markers.markers.push_back(status);

    visualization_msgs::msg::Marker exact;
    exact.header = path.header;
    exact.ns = "first_v2_official";
    exact.id = 0;
    exact.type = visualization_msgs::msg::Marker::LINE_STRIP;
    exact.action = visualization_msgs::msg::Marker::ADD;
    exact.pose.orientation.w = 1.0;
    exact.scale.x = 0.09;
    exact.color.r = 0.05F;
    exact.color.g = 0.45F;
    exact.color.b = 1.0F;
    exact.color.a = 1.0F;
    exact.lifetime = rclcpp::Duration::from_seconds(0.3);
    for (const auto & pose : path.poses) {
      exact.points.push_back(pose.pose.position);
    }
    markers.markers.push_back(exact);

    const auto stateAt = [&](double requested_time) {
        std::array<double, 3> state{0.0, 0.0, 0.0};
        const std::size_t count = path.poses.size();
        if (count == 0U) {
          return state;
        }
        const double exact_horizon =
          static_cast<double>(count) * prediction_step_;
        if (requested_time <= exact_horizon + 1.0e-9) {
          const std::size_t index = std::min(
            count - 1U,
            static_cast<std::size_t>(
              std::max(0.0, std::ceil(
                requested_time / prediction_step_) - 1.0)));
          state[0] = path.poses[index].pose.position.x;
          state[1] = path.poses[index].pose.position.y;
          state[2] = candidate.velocity_yaw * requested_time;
          return state;
        }
        const double extension_time = requested_time - exact_horizon;
        const double yaw_start =
          count >= candidate.points.size() ?
          candidate.end_yaw :
          candidate.velocity_yaw * exact_horizon;
        const double yaw_end =
          yaw_start + candidate.velocity_yaw * extension_time;
        const double vx = stage.scale * candidate.velocity_x;
        const double vy = stage.scale * candidate.velocity_y;
        double delta_x;
        double delta_y;
        if (std::abs(candidate.velocity_yaw) > 1.0e-6) {
          delta_x =
            (vx * (std::sin(yaw_end) - std::sin(yaw_start)) +
            vy * (std::cos(yaw_end) - std::cos(yaw_start))) /
            candidate.velocity_yaw;
          delta_y =
            (vx * (std::cos(yaw_start) - std::cos(yaw_end)) +
            vy * (std::sin(yaw_end) - std::sin(yaw_start))) /
            candidate.velocity_yaw;
        } else {
          delta_x =
            (std::cos(yaw_start) * vx - std::sin(yaw_start) * vy) *
            extension_time;
          delta_y =
            (std::sin(yaw_start) * vx + std::cos(yaw_start) * vy) *
            extension_time;
        }
        state[0] = path.poses.back().pose.position.x + delta_x;
        state[1] = path.poses.back().pose.position.y + delta_y;
        state[2] = yaw_end;
        return state;
      };

    visualization_msgs::msg::Marker extension;
    extension.header = path.header;
    extension.ns = "first_v2_integrated";
    extension.id = 1;
    extension.type = visualization_msgs::msg::Marker::LINE_STRIP;
    extension.action = visualization_msgs::msg::Marker::ADD;
    extension.pose.orientation.w = 1.0;
    extension.scale.x = 0.07;
    extension.color.r = 0.80F;
    extension.color.g = 0.10F;
    extension.color.b = 0.95F;
    extension.color.a = 0.95F;
    extension.lifetime = rclcpp::Duration::from_seconds(0.3);
    const double exact_horizon =
      static_cast<double>(path.poses.size()) * prediction_step_;
    for (
      double time = exact_horizon;
      time <= prediction_horizon_ + 1.0e-9;
      time += prediction_step_)
    {
      const auto state = stateAt(time);
      geometry_msgs::msg::Point point;
      point.x = state[0];
      point.y = state[1];
      point.z = 0.10;
      extension.points.push_back(point);
    }
    markers.markers.push_back(extension);

    const std::array<double, 3> label_times{1.0, 2.0, 3.0};
    for (std::size_t index = 0; index < label_times.size(); ++index) {
      const auto state = stateAt(label_times[index]);
      visualization_msgs::msg::Marker label;
      label.header = path.header;
      label.ns = "first_prediction_time";
      label.id = static_cast<int>(10U + index);
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.action = visualization_msgs::msg::Marker::ADD;
      label.pose.position.x = state[0];
      label.pose.position.y = state[1];
      label.pose.position.z = 0.35;
      label.pose.orientation.w = 1.0;
      label.scale.z = 0.22;
      label.color.r = index == 0U ? 0.15F : 0.75F;
      label.color.g = 0.35F;
      label.color.b = 1.0F;
      label.color.a = 1.0F;
      label.text = "V2 t=" + std::to_string(index + 1U) + "s";
      label.lifetime = rclcpp::Duration::from_seconds(0.3);
      markers.markers.push_back(label);
    }
    prediction_publisher_->publish(markers);
  }

  void publishEmptyPath()
  {
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = base_frame_;
    path_publisher_->publish(path);
    track_path_publisher_->publish(path);
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.header = path.header;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);
    prediction_publisher_->publish(markers);
  }

  void publishStopPath()
  {
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = base_frame_;
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
    path_publisher_->publish(path);
    track_path_publisher_->publish(path);
  }

  void publishGrid(const std::vector<int> & occupied)
  {
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.stamp = now();
    grid.header.frame_id = base_frame_;
    grid.info.resolution = static_cast<float>(grid_size_);
    grid.info.width = static_cast<std::uint32_t>(grid_count_x_);
    grid.info.height = static_cast<std::uint32_t>(grid_count_y_);
    grid.info.origin.position.x = grid_offset_x_;
    grid.info.origin.position.y = grid_offset_y_;
    grid.info.origin.orientation.w = 1.0;
    grid.data.assign(
      static_cast<std::size_t>(grid_count_x_ * grid_count_y_), 0);
    for (const int voxel : occupied) {
      const int ix = voxel / grid_count_y_;
      const int iy = voxel % grid_count_y_;
      const std::size_t ros_index =
        static_cast<std::size_t>(iy * grid_count_x_ + ix);
      if (ros_index < grid.data.size()) {
        grid.data[ros_index] = 100;
      }
    }
    grid_publisher_->publish(grid);
  }

  void publishStage(const std::string & value)
  {
    std_msgs::msg::String message;
    const int stage_index =
      value == "all_paths_blocked" || value == "fallen" ? -1 :
      selected_stage_index_;
    const double scale =
      stage_index >= 0 ? stages_[static_cast<std::size_t>(stage_index)].scale :
      0.0;
    const double range =
      stage_index >= 0 ? stages_[static_cast<std::size_t>(stage_index)].range :
      0.0;
    std::ostringstream stream;
    stream << "mode=" << value << " stage=" << stage_index
           << " scale=" << scale << " range=" << range
           << " reason=cpp_snapshot";
    message.data = stream.str();
    stage_publisher_->publish(message);
  }

  std::string path_folder_;
  std::string config_path_;
  std::string odom_topic_;
  std::string goal_topic_;
  std::string base_frame_;
  std::string cloud_topic_;
  bool semantic_obstacle_cloud_;
  std::string cmd_topic_;
  std::string local_plan_topic_;
  std::string track_path_topic_;
  std::string local_grid_topic_;
  std::string stage_topic_;
  std::string selected_prediction_topic_;
  std::string local_scans_topic_;
  std::string dynamic_snapshot_topic_;
  std::string reset_topic_;

  double control_frequency_;
  double compute_deadline_;
  double dynamic_timeout_;
  double snapshot_max_skew_;
  bool require_dynamic_input_;
  bool publish_zero_without_goal_;
  double dynamic_block_cost_;
  double switch_score_margin_;
  double minimum_path_hold_time_;
  double path_switch_penalty_;
  double escape_min_speed_;
  double escape_max_speed_;
  double escape_goal_blend_;
  double minimum_escape_hold_time_;
  double dynamic_mask_padding_;
  double fallen_height_threshold_;
  bool enable_fallen_height_check_;
  double prediction_horizon_;
  double prediction_step_;
  double path_scale_;
  double min_path_scale_;
  double path_scale_step_;
  double path_range_;
  double min_path_range_;
  double path_range_step_;
  double obstacle_range_;
  double self_clear_x_;
  double self_clear_y_;
  double goal_tolerance_;
  double yaw_tolerance_;
  bool require_goal_yaw_;
  double max_goal_range_;
  double goal_weight_;
  double yaw_weight_;
  double speed_weight_;
  std::array<double, 3> obstacle_weights_;
  double obstacle_height_threshold_;
  int point_per_path_threshold_;
  int grid_occupancy_threshold_;
  double grid_size_;
  double grid_offset_x_;
  double grid_offset_y_;
  int grid_count_x_;
  int grid_count_y_;
  double max_speed_x_;
  double max_speed_y_;
  double max_yaw_;
  double reverse_penalty_;
  double lateral_penalty_;

  std::vector<Candidate> candidates_;
  std::vector<bool> valid_candidate_;
  std::vector<Stage> stages_;
  std::vector<std::vector<std::vector<int>>> correspondence_;

  mutable std::mutex state_mutex_;
  std::shared_ptr<const CloudSnapshot> latest_cloud_;
  std::shared_ptr<const DynamicSnapshot> latest_dynamic_;
  nav_msgs::msg::Odometry::ConstSharedPtr latest_odometry_;
  geometry_msgs::msg::PoseStamped::ConstSharedPtr latest_goal_;
  std::uint64_t cloud_generation_{0U};
  std::uint64_t dynamic_generation_{0U};
  std::uint64_t odometry_generation_{0U};
  std::uint64_t goal_generation_{0U};
  int selected_path_id_{-1};
  int selected_stage_index_{-1};
  std::string selected_mode_{"idle"};
  SteadyClock::time_point selected_since_;
  std::atomic<bool> reset_pending_{false};
  std::atomic<std::uint64_t> discarded_snapshots_{0U};

  rclcpp::CallbackGroup::SharedPtr data_group_;
  rclcpp::CallbackGroup::SharedPtr planning_group_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr track_path_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stage_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scans_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    prediction_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
    odom_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
    goal_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
    cloud_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr
    dynamic_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_subscription_;
  rclcpp::TimerBase::SharedPtr planning_timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FirstPlannerV2Cpp>();
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 3U);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
