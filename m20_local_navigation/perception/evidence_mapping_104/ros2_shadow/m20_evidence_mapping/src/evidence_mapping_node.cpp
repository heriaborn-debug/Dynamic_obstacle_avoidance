#include "m20_evidence_mapping/evidence_map.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace m20_evidence_mapping
{

namespace
{
constexpr double kPi = 3.14159265358979323846;

double degToRad(const double degrees)
{
  return degrees * kPi / 180.0;
}

double yawFromQuaternion(const Eigen::Quaterniond & quaternion)
{
  const Eigen::Matrix3d rotation = quaternion.normalized().toRotationMatrix();
  return std::atan2(rotation(1, 0), rotation(0, 0));
}

Eigen::Matrix3d rpyDegreesToMatrix(const std::vector<double> & rpy_degrees)
{
  const Eigen::AngleAxisd roll(degToRad(rpy_degrees.at(0)), Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd pitch(degToRad(rpy_degrees.at(1)), Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd yaw(degToRad(rpy_degrees.at(2)), Eigen::Vector3d::UnitZ());
  return (yaw * pitch * roll).toRotationMatrix();
}

std::string toString(const double value, const int precision = 3)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << value;
  return stream.str();
}

diagnostic_msgs::msg::KeyValue keyValue(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue result;
  result.key = key;
  result.value = value;
  return result;
}
}  // namespace

struct TimedPose
{
  rclcpp::Time stamp;
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
};

struct TimedOrientation
{
  rclcpp::Time stamp;
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
};

class EvidenceMappingNode final : public rclcpp::Node
{
public:
  EvidenceMappingNode()
  : Node("m20_evidence_mapping"), map_(loadMapConfig())
  {
    cloud_topic_ = declare_parameter<std::string>("topics.cloud", "/LIDAR/POINTS");
    imu_topic_ = declare_parameter<std::string>("topics.imu", "/IMU");
    odom_topic_ = declare_parameter<std::string>("topics.odom", "/ODOM");
    output_topic_ = declare_parameter<std::string>(
      "topics.evidence_output", "/m20/local_mapping/evidence_cloud");
    diagnostics_topic_ = declare_parameter<std::string>(
      "topics.diagnostics", "/m20/local_mapping/diagnostics");
    output_frame_ = declare_parameter<std::string>("frames.output", "m20_local_gravity");
    world_frame_ = declare_parameter<std::string>("frames.world", "world");

    sync_tolerance_sec_ = declare_parameter<double>("synchronization.tolerance_sec", 0.05);
    watchdog_timeout_sec_ = declare_parameter<double>("synchronization.watchdog_timeout_sec", 0.5);
    buffer_duration_sec_ = declare_parameter<double>("synchronization.buffer_duration_sec", 2.0);
    jump_translation_m_ = declare_parameter<double>("odometry_jump.translation_m", 0.30);
    jump_rotation_rad_ = declare_parameter<double>("odometry_jump.rotation_rad", 0.30);
    jump_z_m_ = declare_parameter<double>("odometry_jump.z_m", 0.20);
    require_imu_ = declare_parameter<bool>("synchronization.require_imu", true);
    require_odom_ = declare_parameter<bool>("synchronization.require_odom", true);

    input_translation_ = vector3Parameter("input_transform.translation", {0.0, 0.0, 0.0});
    input_rotation_body_cloud_ = rpyDegreesToMatrix(
      declare_parameter<std::vector<double>>(
        "input_transform.rpy_deg", std::vector<double>{0.0, 0.0, 0.0}));

    filter_radius_ = declare_parameter<double>("preprocessing.outlier_radius", 0.10);
    filter_min_neighbors_ = declare_parameter<int>("preprocessing.outlier_min_neighbors", 4);
    filter_enabled_ = declare_parameter<bool>("preprocessing.enable_outlier_filter", true);
    voxel_size_xy_ = map_config_.voxel_size_xy;
    voxel_size_z_ = map_config_.voxel_size_z;

    loadSensors();

    auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(100)).reliable().durability_volatile();
    auto cloud_qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable().durability_volatile();
    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, sensor_qos,
      std::bind(&EvidenceMappingNode::onImu, this, std::placeholders::_1));
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, sensor_qos,
      std::bind(&EvidenceMappingNode::onOdom, this, std::placeholders::_1));
    cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, cloud_qos,
      std::bind(&EvidenceMappingNode::onCloud, this, std::placeholders::_1));

    cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_, rclcpp::QoS(rclcpp::KeepLast(2)).reliable().durability_volatile());
    transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    watchdog_timer_ = create_wall_timer(
      std::chrono::milliseconds(200), std::bind(&EvidenceMappingNode::publishWatchdog, this));

    const auto clock_type = get_clock()->get_clock_type();
    last_cloud_stamp_ = rclcpp::Time(0, 0, clock_type);
    last_cloud_receive_time_ = rclcpp::Time(0, 0, clock_type);
    last_imu_receive_time_ = rclcpp::Time(0, 0, clock_type);
    last_odom_receive_time_ = rclcpp::Time(0, 0, clock_type);
    last_output_time_ = rclcpp::Time(0, 0, clock_type);

    RCLCPP_INFO(
      get_logger(),
      "Evidence mapper ready: cloud=%s imu=%s odom=%s output=%s (no motion commands)",
      cloud_topic_.c_str(), imu_topic_.c_str(), odom_topic_.c_str(), output_topic_.c_str());
  }

private:
  EvidenceMapConfig loadMapConfig()
  {
    EvidenceMapConfig config;
    config.voxel_size_xy = static_cast<float>(
      declare_parameter<double>("map.voxel_size_xy", config.voxel_size_xy));
    config.voxel_size_z = static_cast<float>(
      declare_parameter<double>("map.voxel_size_z", config.voxel_size_z));
    config.roi_min_x = static_cast<float>(declare_parameter<double>("map.roi_min_x", -4.0));
    config.roi_max_x = static_cast<float>(declare_parameter<double>("map.roi_max_x", 4.0));
    config.roi_min_y = static_cast<float>(declare_parameter<double>("map.roi_min_y", -4.0));
    config.roi_max_y = static_cast<float>(declare_parameter<double>("map.roi_max_y", 4.0));
    config.roi_min_z = static_cast<float>(declare_parameter<double>("map.roi_min_z", -1.0));
    config.roi_max_z = static_cast<float>(declare_parameter<double>("map.roi_max_z", 0.8));
    config.body_min_x = static_cast<float>(declare_parameter<double>("body.min_x", -0.40));
    config.body_max_x = static_cast<float>(declare_parameter<double>("body.max_x", 0.40));
    config.body_min_y = static_cast<float>(declare_parameter<double>("body.min_y", -0.20));
    config.body_max_y = static_cast<float>(declare_parameter<double>("body.max_y", 0.20));
    config.body_min_z = static_cast<float>(declare_parameter<double>("body.min_z", -0.50));
    config.body_max_z = static_cast<float>(declare_parameter<double>("body.max_z", 0.40));
    config.azimuth_bins = declare_parameter<int>("visibility.azimuth_bins", 640);
    config.elevation_bins = declare_parameter<int>("visibility.elevation_bins", 40);
    config.visibility_neighbor_radius = declare_parameter<int>("visibility.neighbor_radius", 1);
    config.weak_free_confirm_frames = static_cast<uint32_t>(
      declare_parameter<int>("evidence.weak_free_confirm_frames", 5));
    config.near_field_protect_range = static_cast<float>(
      declare_parameter<double>("evidence.near_field_protect_range", 0.10));
    config.match_margin_base = static_cast<float>(
      declare_parameter<double>("evidence.match_margin_base", 0.06));
    config.match_margin_scale = static_cast<float>(
      declare_parameter<double>("evidence.match_margin_scale", 0.015));
    config.free_margin_base = static_cast<float>(
      declare_parameter<double>("evidence.free_margin_base", 0.10));
    config.free_margin_scale = static_cast<float>(
      declare_parameter<double>("evidence.free_margin_scale", 0.020));
    config.occupied_gain = static_cast<float>(
      declare_parameter<double>("evidence.occupied_gain", 1.0));
    config.visibility_match_gain_scale = static_cast<float>(
      declare_parameter<double>("evidence.visibility_match_gain_scale", 0.25));
    config.strong_free_gain = static_cast<float>(
      declare_parameter<double>("evidence.strong_free_gain", 1.4));
    config.weak_free_gain = static_cast<float>(
      declare_parameter<double>("evidence.weak_free_gain", 0.5));
    config.relief_gain = static_cast<float>(
      declare_parameter<double>("evidence.relief_gain", 0.5));
    config.publish_score = static_cast<float>(
      declare_parameter<double>("evidence.publish_score", 0.6));
    config.remove_score = static_cast<float>(
      declare_parameter<double>("evidence.remove_score", 0.15));
    config.score_cap = static_cast<float>(
      declare_parameter<double>("evidence.score_cap", 6.0));
    map_config_ = config;
    return config;
  }

  Eigen::Vector3d vector3Parameter(
    const std::string & name, const std::vector<double> & defaults)
  {
    const auto values = declare_parameter<std::vector<double>>(name, defaults);
    if (values.size() != 3U) {
      throw std::runtime_error(name + " must contain exactly three numbers");
    }
    return Eigen::Vector3d(values[0], values[1], values[2]);
  }

  void loadSensors()
  {
    const double azimuth_half = degToRad(
      declare_parameter<double>("sensors.azimuth_half_fov_deg", 45.0));
    const double elevation_down = degToRad(
      declare_parameter<double>("sensors.elevation_down_deg", -45.0));
    const double elevation_up = degToRad(
      declare_parameter<double>("sensors.elevation_up_deg", 45.0));
    const double max_range = declare_parameter<double>("sensors.max_range", 4.0);

    SensorModel front;
    front.name = "front";
    front.origin = Eigen::Vector3f(0.370F, 0.0F, -0.010F);
    front.rotation_body_sensor = rpyDegreesToMatrix({0.0, 90.0, 0.0}).cast<float>();
    front.azimuth_min_rad = static_cast<float>(-azimuth_half);
    front.azimuth_max_rad = static_cast<float>(azimuth_half);
    front.elevation_min_rad = static_cast<float>(elevation_down);
    front.elevation_max_rad = static_cast<float>(elevation_up);
    front.max_range = static_cast<float>(max_range);

    SensorModel rear = front;
    rear.name = "rear";
    rear.origin = Eigen::Vector3f(-0.370F, 0.0F, -0.010F);
    rear.rotation_body_sensor = rpyDegreesToMatrix({0.0, -90.0, 0.0}).cast<float>();
    sensors_body_ = {front, rear};
  }

  void onImu(const sensor_msgs::msg::Imu::SharedPtr message)
  {
    Eigen::Quaterniond orientation(
      message->orientation.w, message->orientation.x,
      message->orientation.y, message->orientation.z);
    if (!std::isfinite(orientation.norm()) || orientation.norm() < 0.5) {
      invalid_imu_count_++;
      return;
    }
    orientation.normalize();
    const rclcpp::Time stamp(message->header.stamp, get_clock()->get_clock_type());
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    imu_buffer_.push_back(TimedOrientation{stamp, orientation});
    trimBuffers(stamp);
    last_imu_receive_time_ = now();
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    Eigen::Quaterniond orientation(
      message->pose.pose.orientation.w, message->pose.pose.orientation.x,
      message->pose.pose.orientation.y, message->pose.pose.orientation.z);
    const Eigen::Vector3d position(
      message->pose.pose.position.x,
      message->pose.pose.position.y,
      message->pose.pose.position.z);
    if (!position.allFinite() || !std::isfinite(orientation.norm()) || orientation.norm() < 0.5) {
      invalid_odom_count_++;
      return;
    }
    orientation.normalize();
    const rclcpp::Time stamp(message->header.stamp, get_clock()->get_clock_type());
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    odom_buffer_.push_back(TimedPose{stamp, position, orientation});
    trimBuffers(stamp);
    last_odom_receive_time_ = now();
  }

  void trimBuffers(const rclcpp::Time & newest_stamp)
  {
    const rclcpp::Duration keep = rclcpp::Duration::from_seconds(buffer_duration_sec_);
    while (!imu_buffer_.empty() && newest_stamp - imu_buffer_.front().stamp > keep) {
      imu_buffer_.pop_front();
    }
    while (!odom_buffer_.empty() && newest_stamp - odom_buffer_.front().stamp > keep) {
      odom_buffer_.pop_front();
    }
  }

  bool interpolateImu(const rclcpp::Time & stamp, Eigen::Quaterniond & orientation)
  {
    if (imu_buffer_.empty()) {
      return !require_imu_;
    }
    if (imu_buffer_.size() == 1U) {
      const double delta = std::abs((stamp - imu_buffer_.front().stamp).seconds());
      if (delta > sync_tolerance_sec_) {
        return false;
      }
      orientation = imu_buffer_.front().orientation;
      return true;
    }
    for (std::size_t index = 1; index < imu_buffer_.size(); ++index) {
      const TimedOrientation & right = imu_buffer_[index];
      const TimedOrientation & left = imu_buffer_[index - 1U];
      if (left.stamp <= stamp && stamp <= right.stamp) {
        const double span = (right.stamp - left.stamp).seconds();
        if (span <= 0.0 || span > 2.0 * sync_tolerance_sec_) {
          return false;
        }
        const double ratio = std::clamp((stamp - left.stamp).seconds() / span, 0.0, 1.0);
        orientation = left.orientation.slerp(ratio, right.orientation).normalized();
        return true;
      }
    }
    const TimedOrientation & nearest =
      std::abs((stamp - imu_buffer_.front().stamp).seconds()) <
      std::abs((stamp - imu_buffer_.back().stamp).seconds()) ?
      imu_buffer_.front() : imu_buffer_.back();
    if (std::abs((stamp - nearest.stamp).seconds()) <= sync_tolerance_sec_) {
      orientation = nearest.orientation;
      return true;
    }
    return false;
  }

  bool interpolateOdom(const rclcpp::Time & stamp, TimedPose & pose)
  {
    if (odom_buffer_.empty()) {
      return !require_odom_;
    }
    if (odom_buffer_.size() == 1U) {
      if (std::abs((stamp - odom_buffer_.front().stamp).seconds()) > sync_tolerance_sec_) {
        return false;
      }
      pose = odom_buffer_.front();
      pose.stamp = stamp;
      return true;
    }
    for (std::size_t index = 1; index < odom_buffer_.size(); ++index) {
      const TimedPose & right = odom_buffer_[index];
      const TimedPose & left = odom_buffer_[index - 1U];
      if (left.stamp <= stamp && stamp <= right.stamp) {
        const double span = (right.stamp - left.stamp).seconds();
        if (span <= 0.0 || span > 2.0 * sync_tolerance_sec_) {
          return false;
        }
        const double ratio = std::clamp((stamp - left.stamp).seconds() / span, 0.0, 1.0);
        pose.stamp = stamp;
        pose.position = left.position + ratio * (right.position - left.position);
        pose.orientation = left.orientation.slerp(ratio, right.orientation).normalized();
        return true;
      }
    }
    const TimedPose & nearest =
      std::abs((stamp - odom_buffer_.front().stamp).seconds()) <
      std::abs((stamp - odom_buffer_.back().stamp).seconds()) ?
      odom_buffer_.front() : odom_buffer_.back();
    if (std::abs((stamp - nearest.stamp).seconds()) <= sync_tolerance_sec_) {
      pose = nearest;
      pose.stamp = stamp;
      return true;
    }
    return false;
  }

  std::vector<Eigen::Vector3f> preprocessIntegration(
    const std::vector<Eigen::Vector3f> & points) const
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr input(new pcl::PointCloud<pcl::PointXYZ>());
    input->reserve(points.size());
    for (const auto & point : points) {
      input->push_back(pcl::PointXYZ(point.x(), point.y(), point.z()));
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(input);
    voxel_grid.setLeafSize(
      static_cast<float>(voxel_size_xy_), static_cast<float>(voxel_size_xy_),
      static_cast<float>(voxel_size_z_));
    voxel_grid.filter(*downsampled);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered = downsampled;
    if (filter_enabled_ && downsampled->size() >= static_cast<std::size_t>(filter_min_neighbors_)) {
      filtered.reset(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::RadiusOutlierRemoval<pcl::PointXYZ> outlier;
      outlier.setInputCloud(downsampled);
      outlier.setRadiusSearch(filter_radius_);
      outlier.setMinNeighborsInRadius(filter_min_neighbors_);
      outlier.filter(*filtered);
    }

    std::vector<Eigen::Vector3f> result;
    result.reserve(filtered->size());
    for (const auto & point : filtered->points) {
      result.emplace_back(point.x, point.y, point.z);
    }
    return result;
  }

  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    const auto start = std::chrono::steady_clock::now();
    const rclcpp::Time stamp(message->header.stamp, get_clock()->get_clock_type());
    last_cloud_receive_time_ = now();

    TimedPose odom_pose;
    Eigen::Quaterniond imu_orientation = Eigen::Quaterniond::Identity();
    {
      std::lock_guard<std::mutex> lock(buffer_mutex_);
      if (!interpolateOdom(stamp, odom_pose) || !interpolateImu(stamp, imu_orientation)) {
        ++sync_drop_count_;
        publishFrameDiagnostic(
          diagnostic_msgs::msg::DiagnosticStatus::WARN,
          "SYNC_REJECTED", nullptr, 0.0, "No valid IMU/ODOM interpolation at cloud stamp");
        return;
      }
    }

    if (last_cloud_stamp_.nanoseconds() != 0 && stamp <= last_cloud_stamp_) {
      map_.reset();
      have_previous_pose_ = false;
      ++time_reset_count_;
    }
    last_cloud_stamp_ = stamp;

    const double imu_yaw = yawFromQuaternion(imu_orientation);
    const Eigen::Matrix3d world_from_imu = imu_orientation.toRotationMatrix();
    const Eigen::Matrix3d world_from_imu_yaw =
      Eigen::AngleAxisd(imu_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const Eigen::Matrix3d gravity_from_body = world_from_imu_yaw.transpose() * world_from_imu;

    const double odom_yaw = yawFromQuaternion(odom_pose.orientation);
    Eigen::Isometry3d world_from_gravity = Eigen::Isometry3d::Identity();
    world_from_gravity.linear() =
      Eigen::AngleAxisd(odom_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    world_from_gravity.translation() = odom_pose.position;

    Eigen::Isometry3f current_from_previous = Eigen::Isometry3f::Identity();
    bool jump_reset = false;
    double relative_translation = 0.0;
    double relative_rotation = 0.0;
    double relative_z = 0.0;
    if (have_previous_pose_) {
      const Eigen::Isometry3d relative = world_from_gravity.inverse() * previous_world_from_gravity_;
      relative_translation = relative.translation().head<2>().norm();
      relative_z = std::abs(relative.translation().z());
      relative_rotation = Eigen::AngleAxisd(relative.linear()).angle();
      if (relative_translation > jump_translation_m_ || relative_z > jump_z_m_ ||
        relative_rotation > jump_rotation_rad_)
      {
        map_.reset();
        current_from_previous.setIdentity();
        jump_reset = true;
        ++jump_reset_count_;
      } else {
        current_from_previous = relative.cast<float>();
      }
    }
    previous_world_from_gravity_ = world_from_gravity;
    have_previous_pose_ = true;
    publishGravityTransform(stamp, world_from_gravity);

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*message, cloud);
    std::vector<Eigen::Vector3f> visibility_points;
    visibility_points.reserve(cloud.size());
    const Eigen::Matrix3d gravity_from_cloud =
      gravity_from_body * input_rotation_body_cloud_;
    const Eigen::Vector3d gravity_translation = gravity_from_body * input_translation_;
    for (const auto & point : cloud.points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }
      const Eigen::Vector3d raw(point.x, point.y, point.z);
      visibility_points.push_back(
        (gravity_from_cloud * raw + gravity_translation).cast<float>());
    }
    const auto integration_points = preprocessIntegration(visibility_points);

    std::vector<SensorModel> sensors_gravity = sensors_body_;
    for (auto & sensor : sensors_gravity) {
      sensor.origin = (gravity_from_body * sensor.origin.cast<double>()).cast<float>();
      sensor.rotation_body_sensor =
        (gravity_from_body * sensor.rotation_body_sensor.cast<double>()).cast<float>();
    }

    const ProcessStatistics statistics = map_.processFrame(
      visibility_points, integration_points, current_from_previous, sensors_gravity);
    publishCloud(stamp);

    const double processing_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();
    ++processed_frame_count_;
    last_output_time_ = now();
    const uint8_t level = processing_ms > 80.0 ?
      diagnostic_msgs::msg::DiagnosticStatus::WARN :
      diagnostic_msgs::msg::DiagnosticStatus::OK;
    publishFrameDiagnostic(
      level, jump_reset ? "ODOM_JUMP_RESET" : "RUNNING", &statistics, processing_ms,
      "relative_xy=" + toString(relative_translation) +
      " relative_z=" + toString(relative_z) +
      " relative_angle=" + toString(relative_rotation));
  }

  void publishCloud(const rclcpp::Time & stamp)
  {
    const auto voxels = map_.publishedVoxels();
    pcl::PointCloud<pcl::PointXYZI> output;
    output.reserve(voxels.size());
    for (const auto & voxel : voxels) {
      pcl::PointXYZI point;
      point.x = voxel.position.x();
      point.y = voxel.position.y();
      point.z = voxel.position.z();
      point.intensity = voxel.occupancy_score;
      output.push_back(point);
    }
    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(output, message);
    message.header.stamp = stamp;
    message.header.frame_id = output_frame_;
    cloud_publisher_->publish(message);
  }

  void publishGravityTransform(
    const rclcpp::Time & stamp, const Eigen::Isometry3d & world_from_gravity)
  {
    const double yaw = std::atan2(
      world_from_gravity.linear()(1, 0), world_from_gravity.linear()(0, 0));
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = world_frame_;
    transform.child_frame_id = output_frame_;
    transform.transform.translation.x = world_from_gravity.translation().x();
    transform.transform.translation.y = world_from_gravity.translation().y();
    transform.transform.translation.z = world_from_gravity.translation().z();
    transform.transform.rotation.z = std::sin(0.5 * yaw);
    transform.transform.rotation.w = std::cos(0.5 * yaw);
    transform_broadcaster_->sendTransform(transform);
  }

  void publishFrameDiagnostic(
    const uint8_t level, const std::string & state,
    const ProcessStatistics * statistics, const double processing_ms,
    const std::string & detail)
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = level;
    status.name = "m20_local_mapping/evidence_mapper";
    status.hardware_id = "m20_nx";
    status.message = state;
    status.values.push_back(keyValue("detail", detail));
    status.values.push_back(keyValue("processing_ms", toString(processing_ms)));
    status.values.push_back(keyValue("processed_frames", std::to_string(processed_frame_count_)));
    status.values.push_back(keyValue("sync_drops", std::to_string(sync_drop_count_)));
    status.values.push_back(keyValue("jump_resets", std::to_string(jump_reset_count_)));
    if (statistics != nullptr) {
      status.values.push_back(keyValue("input_points", std::to_string(statistics->input_points)));
      status.values.push_back(keyValue("accepted_points", std::to_string(statistics->accepted_points)));
      status.values.push_back(keyValue("map_voxels", std::to_string(statistics->map_voxels_after)));
      status.values.push_back(keyValue("matched", std::to_string(statistics->matched_voxels)));
      status.values.push_back(keyValue("strong_free", std::to_string(statistics->strong_cleared_voxels)));
      status.values.push_back(keyValue("weak_free", std::to_string(statistics->weak_cleared_voxels)));
      status.values.push_back(keyValue("occluded", std::to_string(statistics->occluded_voxels)));
    }
    array.status.push_back(std::move(status));
    diagnostics_publisher_->publish(array);
  }

  void publishWatchdog()
  {
    const rclcpp::Time current = now();
    std::vector<std::string> failures;
    if (last_cloud_receive_time_.nanoseconds() == 0 ||
      (current - last_cloud_receive_time_).seconds() > watchdog_timeout_sec_)
    {
      failures.emplace_back("cloud timeout");
    }
    if (require_imu_ && (last_imu_receive_time_.nanoseconds() == 0 ||
      (current - last_imu_receive_time_).seconds() > watchdog_timeout_sec_))
    {
      failures.emplace_back("imu timeout");
    }
    if (require_odom_ && (last_odom_receive_time_.nanoseconds() == 0 ||
      (current - last_odom_receive_time_).seconds() > watchdog_timeout_sec_))
    {
      failures.emplace_back("odom timeout");
    }
    if (!failures.empty()) {
      std::string detail;
      for (std::size_t index = 0; index < failures.size(); ++index) {
        detail += (index == 0U ? "" : ", ") + failures[index];
      }
      publishFrameDiagnostic(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "INPUT_TIMEOUT", nullptr, 0.0, detail);
    }
  }

  EvidenceMapConfig map_config_;
  EvidenceMap map_;
  std::vector<SensorModel> sensors_body_;

  std::string cloud_topic_;
  std::string imu_topic_;
  std::string odom_topic_;
  std::string output_topic_;
  std::string diagnostics_topic_;
  std::string output_frame_;
  std::string world_frame_;

  double sync_tolerance_sec_{0.05};
  double watchdog_timeout_sec_{0.5};
  double buffer_duration_sec_{2.0};
  double jump_translation_m_{0.30};
  double jump_rotation_rad_{0.30};
  double jump_z_m_{0.20};
  double filter_radius_{0.10};
  double voxel_size_xy_{0.045};
  double voxel_size_z_{0.060};
  int filter_min_neighbors_{4};
  bool filter_enabled_{true};
  bool require_imu_{true};
  bool require_odom_{true};

  Eigen::Vector3d input_translation_{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d input_rotation_body_cloud_{Eigen::Matrix3d::Identity()};

  std::mutex buffer_mutex_;
  std::deque<TimedOrientation> imu_buffer_;
  std::deque<TimedPose> odom_buffer_;
  Eigen::Isometry3d previous_world_from_gravity_{Eigen::Isometry3d::Identity()};
  bool have_previous_pose_{false};

  rclcpp::Time last_cloud_stamp_{0, 0, RCL_SYSTEM_TIME};
  rclcpp::Time last_cloud_receive_time_{0, 0, RCL_SYSTEM_TIME};
  rclcpp::Time last_imu_receive_time_{0, 0, RCL_SYSTEM_TIME};
  rclcpp::Time last_odom_receive_time_{0, 0, RCL_SYSTEM_TIME};
  rclcpp::Time last_output_time_{0, 0, RCL_SYSTEM_TIME};

  uint64_t processed_frame_count_{0U};
  uint64_t sync_drop_count_{0U};
  uint64_t jump_reset_count_{0U};
  uint64_t time_reset_count_{0U};
  uint64_t invalid_imu_count_{0U};
  uint64_t invalid_odom_count_{0U};

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

}  // namespace m20_evidence_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<m20_evidence_mapping::EvidenceMappingNode>());
  rclcpp::shutdown();
  return 0;
}
