#include "m20_evidence_mapping/evidence_map.hpp"

#include <drdds/core/drdds_core.h>
#include <drdds/msg/ImuPubSubTypes.h>
#include <drdds/msg/OdometryPubSubTypes.h>
#include <drdds/msg/PointCloud2PubSubTypes.h>
#include <drdds/msg/TFMessagePubSubTypes.h>

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <deque>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace m20_evidence_mapping
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kSyncToleranceSec = 0.05;
constexpr double kBufferDurationSec = 2.0;
constexpr double kJumpTranslationM = 0.30;
constexpr double kJumpRotationRad = 0.30;
constexpr double kJumpZM = 0.20;
constexpr double kFilterRadius = 0.10;
constexpr int kFilterMinNeighbors = 4;

double degToRad(const double value) { return value * kPi / 180.0; }

double yawFromQuaternion(const Eigen::Quaterniond & quaternion)
{
  const Eigen::Matrix3d rotation = quaternion.normalized().toRotationMatrix();
  return std::atan2(rotation(1, 0), rotation(0, 0));
}

Eigen::Matrix3d rpyDegreesToMatrix(const double roll, const double pitch, const double yaw)
{
  return (
    Eigen::AngleAxisd(degToRad(yaw), Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(degToRad(pitch), Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(degToRad(roll), Eigen::Vector3d::UnitX())).toRotationMatrix();
}

double stampSeconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec()) + 1.0e-9 * static_cast<double>(stamp.nanosec());
}

double steadySeconds()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

struct TimedPose
{
  double stamp{0.0};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
};

struct TimedOrientation
{
  double stamp{0.0};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
};
}  // namespace

class DrddsEvidenceNode
{
public:
  DrddsEvidenceNode()
  : map_(makeMapConfig())
  {
    loadSensors();

    // The vendor SDK creates separate local-SHM and network participants from this flag.
    output_channel_ = std::make_unique<DrDDSChannel<sensor_msgs::msg::PointCloud2PubSubType>>(
      "/m20/local_mapping/evidence_cloud", 0, false);
    tf_channel_ = std::make_unique<DrDDSChannel<tf2_msgs::msg::TFMessagePubSubType>>(
      "/tf", 0, false);
    imu_channel_ = std::make_unique<DrDDSChannel<sensor_msgs::msg::ImuPubSubType>>(
      std::bind(&DrddsEvidenceNode::onImu, this, std::placeholders::_1), "/IMU", 0, false);
    odom_channel_ = std::make_unique<DrDDSChannel<nav_msgs::msg::OdometryPubSubType>>(
      std::bind(&DrddsEvidenceNode::onOdom, this, std::placeholders::_1),
      "/lio/robo/odom", 0, false);
    cloud_channel_ = std::make_unique<DrDDSChannel<sensor_msgs::msg::PointCloud2PubSubType>>(
      std::bind(&DrddsEvidenceNode::onCloud, this, std::placeholders::_1),
      "/LIDAR/POINTS", 0, true);

    cloud_worker_thread_ = std::thread([this]() { cloudWorkerLoop(); });
    watchdog_thread_ = std::thread([this]() { watchdogLoop(); });
    std::cout << "drdds evidence mapper ready: SHM=/LIDAR/POINTS output=/m20/local_mapping/evidence_cloud"
              << std::endl;
  }

  ~DrddsEvidenceNode()
  {
    stop_.store(true);
    cloud_condition_.notify_all();
    if (cloud_worker_thread_.joinable()) {
      cloud_worker_thread_.join();
    }
    if (watchdog_thread_.joinable()) {
      watchdog_thread_.join();
    }
  }

private:
  static EvidenceMapConfig makeMapConfig()
  {
    // Values are identical to config/m20_evidence_mapping.yaml.
    EvidenceMapConfig config;
    config.voxel_size_xy = 0.045F;
    config.voxel_size_z = 0.060F;
    config.roi_min_x = -4.0F;
    config.roi_max_x = 4.0F;
    config.roi_min_y = -4.0F;
    config.roi_max_y = 4.0F;
    config.roi_min_z = -1.0F;
    config.roi_max_z = 0.8F;
    config.body_min_x = -0.40F;
    config.body_max_x = 0.40F;
    config.body_min_y = -0.20F;
    config.body_max_y = 0.20F;
    config.body_min_z = -0.50F;
    config.body_max_z = 0.40F;
    config.azimuth_bins = 640;
    config.elevation_bins = 40;
    config.visibility_neighbor_radius = 1;
    config.weak_free_confirm_frames = 5U;
    config.near_field_protect_range = 0.10F;
    config.match_margin_base = 0.06F;
    config.match_margin_scale = 0.015F;
    config.free_margin_base = 0.10F;
    config.free_margin_scale = 0.020F;
    config.occupied_gain = 1.0F;
    config.visibility_match_gain_scale = 0.25F;
    config.strong_free_gain = 1.4F;
    config.weak_free_gain = 0.50F;
    config.relief_gain = 0.50F;
    config.publish_score = 0.60F;
    config.remove_score = 0.15F;
    config.score_cap = 6.0F;
    return config;
  }

  void loadSensors()
  {
    SensorModel front;
    front.name = "front";
    front.origin = Eigen::Vector3f(0.370F, 0.0F, -0.010F);
    front.rotation_body_sensor = rpyDegreesToMatrix(0.0, 90.0, 0.0).cast<float>();
    front.frustum_apex = Eigen::Vector3f(-0.150F, 0.0F, -0.010F);
    SensorModel rear = front;
    rear.name = "rear";
    rear.origin = Eigen::Vector3f(-0.370F, 0.0F, -0.010F);
    rear.rotation_body_sensor = rpyDegreesToMatrix(0.0, -90.0, 0.0).cast<float>();
    rear.frustum_apex = Eigen::Vector3f(0.150F, 0.0F, -0.010F);
    sensors_body_ = {front, rear};
  }

  void onImu(const sensor_msgs::msg::Imu * message)
  {
    const auto & q = message->orientation();
    Eigen::Quaterniond orientation(q.w(), q.x(), q.y(), q.z());
    if (!std::isfinite(orientation.norm()) || orientation.norm() < 0.5) {
      ++invalid_imu_count_;
      return;
    }
    orientation.normalize();
    const double stamp = stampSeconds(message->header().stamp());
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    imu_buffer_.push_back(TimedOrientation{stamp, orientation});
    trimBuffers(stamp);
    last_imu_receive_.store(steadySeconds());
  }

  void onOdom(const nav_msgs::msg::Odometry * message)
  {
    const auto & pose = message->pose().pose();
    const auto & p = pose.position();
    const auto & q = pose.orientation();
    Eigen::Vector3d position(p.x(), p.y(), p.z());
    Eigen::Quaterniond orientation(q.w(), q.x(), q.y(), q.z());
    if (!position.allFinite() || !std::isfinite(orientation.norm()) || orientation.norm() < 0.5) {
      ++invalid_odom_count_;
      return;
    }
    orientation.normalize();
    const double stamp = stampSeconds(message->header().stamp());
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    odom_buffer_.push_back(TimedPose{stamp, position, orientation});
    trimBuffers(stamp);
    last_odom_receive_.store(steadySeconds());
  }

  void trimBuffers(const double newest_stamp)
  {
    while (!imu_buffer_.empty() && newest_stamp - imu_buffer_.front().stamp > kBufferDurationSec) {
      imu_buffer_.pop_front();
    }
    while (!odom_buffer_.empty() && newest_stamp - odom_buffer_.front().stamp > kBufferDurationSec) {
      odom_buffer_.pop_front();
    }
  }

  bool interpolateImu(const double stamp, Eigen::Quaterniond & orientation) const
  {
    if (imu_buffer_.empty()) {
      return false;
    }
    for (std::size_t i = 1; i < imu_buffer_.size(); ++i) {
      const auto & left = imu_buffer_[i - 1U];
      const auto & right = imu_buffer_[i];
      if (left.stamp <= stamp && stamp <= right.stamp) {
        const double span = right.stamp - left.stamp;
        if (span <= 0.0 || span > 2.0 * kSyncToleranceSec) {
          return false;
        }
        const double ratio = std::clamp((stamp - left.stamp) / span, 0.0, 1.0);
        orientation = left.orientation.slerp(ratio, right.orientation).normalized();
        return true;
      }
    }
    const auto & nearest =
      std::abs(stamp - imu_buffer_.front().stamp) < std::abs(stamp - imu_buffer_.back().stamp) ?
      imu_buffer_.front() : imu_buffer_.back();
    if (std::abs(stamp - nearest.stamp) <= kSyncToleranceSec) {
      orientation = nearest.orientation;
      return true;
    }
    return false;
  }

  bool interpolateOdom(const double stamp, TimedPose & result) const
  {
    if (odom_buffer_.empty()) {
      return false;
    }
    for (std::size_t i = 1; i < odom_buffer_.size(); ++i) {
      const auto & left = odom_buffer_[i - 1U];
      const auto & right = odom_buffer_[i];
      if (left.stamp <= stamp && stamp <= right.stamp) {
        const double span = right.stamp - left.stamp;
        if (span <= 0.0 || span > 2.0 * kSyncToleranceSec) {
          return false;
        }
        const double ratio = std::clamp((stamp - left.stamp) / span, 0.0, 1.0);
        result.stamp = stamp;
        result.position = left.position + ratio * (right.position - left.position);
        result.orientation = left.orientation.slerp(ratio, right.orientation).normalized();
        return true;
      }
    }
    const auto & nearest =
      std::abs(stamp - odom_buffer_.front().stamp) < std::abs(stamp - odom_buffer_.back().stamp) ?
      odom_buffer_.front() : odom_buffer_.back();
    if (std::abs(stamp - nearest.stamp) <= kSyncToleranceSec) {
      result = nearest;
      result.stamp = stamp;
      return true;
    }
    return false;
  }

  std::vector<Eigen::Vector3f> decodeCloud(const sensor_msgs::msg::PointCloud2 & message) const
  {
    if (message.is_bigendian()) {
      throw std::runtime_error("big-endian PointCloud2 is unsupported");
    }
    std::uint32_t x_offset = std::numeric_limits<std::uint32_t>::max();
    std::uint32_t y_offset = x_offset;
    std::uint32_t z_offset = x_offset;
    for (const auto & field : message.fields()) {
      if (field.name() == "x") {x_offset = field.offset();}
      if (field.name() == "y") {y_offset = field.offset();}
      if (field.name() == "z") {z_offset = field.offset();}
    }
    if (x_offset == std::numeric_limits<std::uint32_t>::max() ||
      y_offset == x_offset || z_offset == x_offset || message.point_step() < 12U)
    {
      throw std::runtime_error("PointCloud2 has no usable x/y/z fields");
    }

    const std::size_t count = std::min<std::size_t>(
      static_cast<std::size_t>(message.width()) * message.height(),
      message.data().size() / message.point_step());
    std::vector<Eigen::Vector3f> points;
    points.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
      const std::uint8_t * base = message.data().data() + i * message.point_step();
      float x, y, z;
      std::memcpy(&x, base + x_offset, sizeof(float));
      std::memcpy(&y, base + y_offset, sizeof(float));
      std::memcpy(&z, base + z_offset, sizeof(float));
      if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
        points.emplace_back(x, y, z);
      }
    }
    return points;
  }

  std::vector<Eigen::Vector3f> preprocessIntegration(
    const std::vector<Eigen::Vector3f> & points) const
  {
    auto input = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    input->reserve(points.size());
    for (const auto & point : points) {
      input->push_back(pcl::PointXYZ(point.x(), point.y(), point.z()));
    }
    auto downsampled = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(input);
    voxel.setLeafSize(0.045F, 0.045F, 0.060F);
    voxel.filter(*downsampled);

    auto filtered = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    if (downsampled->size() >= static_cast<std::size_t>(kFilterMinNeighbors)) {
      pcl::RadiusOutlierRemoval<pcl::PointXYZ> outlier;
      outlier.setInputCloud(downsampled);
      outlier.setRadiusSearch(kFilterRadius);
      outlier.setMinNeighborsInRadius(kFilterMinNeighbors);
      outlier.filter(*filtered);
    } else {
      *filtered = *downsampled;
    }
    std::vector<Eigen::Vector3f> result;
    result.reserve(filtered->size());
    for (const auto & point : filtered->points) {
      result.emplace_back(point.x, point.y, point.z);
    }
    return result;
  }

  void onCloud(const sensor_msgs::msg::PointCloud2 * message)
  {
    last_cloud_receive_.store(steadySeconds());
    {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      if (cloud_pending_) {
        ++overwritten_cloud_count_;
      }
      latest_cloud_ = *message;
      cloud_pending_ = true;
    }
    cloud_condition_.notify_one();
  }

  void cloudWorkerLoop()
  {
    while (!stop_.load()) {
      sensor_msgs::msg::PointCloud2 message;
      {
        std::unique_lock<std::mutex> lock(cloud_mutex_);
        cloud_condition_.wait(lock, [this]() {return stop_.load() || cloud_pending_;});
        if (stop_.load()) {
          return;
        }
        message = std::move(latest_cloud_);
        cloud_pending_ = false;
      }
      processCloud(message);
    }
  }

  void processCloud(const sensor_msgs::msg::PointCloud2 & message)
  {
    const auto start = std::chrono::steady_clock::now();
    const double stamp = stampSeconds(message.header().stamp());

    TimedPose odom;
    Eigen::Quaterniond imu = Eigen::Quaterniond::Identity();
    {
      std::lock_guard<std::mutex> lock(buffer_mutex_);
      if (!interpolateOdom(stamp, odom) || !interpolateImu(stamp, imu)) {
        ++sync_drop_count_;
        return;
      }
    }

    if (last_cloud_stamp_ != 0.0 && stamp <= last_cloud_stamp_) {
      map_.reset();
      have_previous_pose_ = false;
      ++time_reset_count_;
    }
    last_cloud_stamp_ = stamp;

    const double imu_yaw = yawFromQuaternion(imu);
    const Eigen::Matrix3d gravity_from_body =
      Eigen::AngleAxisd(-imu_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
      imu.toRotationMatrix();

    Eigen::Isometry3d world_from_gravity = Eigen::Isometry3d::Identity();
    world_from_gravity.linear() =
      Eigen::AngleAxisd(yawFromQuaternion(odom.orientation), Eigen::Vector3d::UnitZ()).toRotationMatrix();
    world_from_gravity.translation() = odom.position;

    Eigen::Isometry3f current_from_previous = Eigen::Isometry3f::Identity();
    bool jump_reset = false;
    if (have_previous_pose_) {
      const Eigen::Isometry3d relative = world_from_gravity.inverse() * previous_world_from_gravity_;
      if (relative.translation().head<2>().norm() > kJumpTranslationM ||
        std::abs(relative.translation().z()) > kJumpZM ||
        Eigen::AngleAxisd(relative.linear()).angle() > kJumpRotationRad)
      {
        map_.reset();
        ++jump_reset_count_;
        jump_reset = true;
      } else {
        current_from_previous = relative.cast<float>();
      }
    }
    previous_world_from_gravity_ = world_from_gravity;
    have_previous_pose_ = true;

    auto raw_points = decodeCloud(message);
    std::vector<Eigen::Vector3f> visibility_points;
    visibility_points.reserve(raw_points.size());
    for (const auto & point : raw_points) {
      visibility_points.push_back((gravity_from_body * point.cast<double>()).cast<float>());
    }
    const auto integration_points = preprocessIntegration(visibility_points);

    std::vector<SensorModel> sensors_gravity = sensors_body_;
    for (auto & sensor : sensors_gravity) {
      sensor.origin = (gravity_from_body * sensor.origin.cast<double>()).cast<float>();
      sensor.frustum_apex =
        (gravity_from_body * sensor.frustum_apex.cast<double>()).cast<float>();
      sensor.rotation_body_sensor =
        (gravity_from_body * sensor.rotation_body_sensor.cast<double>()).cast<float>();
    }
    const ProcessStatistics statistics = map_.processFrame(
      visibility_points, integration_points, current_from_previous, sensors_gravity);

    publishCloud(message.header().stamp());
    publishTransform(message.header().stamp(), world_from_gravity);

    const double processing_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();
    const std::uint64_t count = ++processed_frame_count_;
    if (count == 1U || count % 10U == 0U || jump_reset) {
      std::cout << "processed=" << count
                << " ms=" << processing_ms
                << " input=" << statistics.input_points
                << " accepted=" << statistics.accepted_points
                << " voxels=" << statistics.map_voxels_after
                << " matched=" << statistics.matched_voxels
                << " strong_free=" << statistics.strong_cleared_voxels
                << " weak_free=" << statistics.weak_cleared_voxels
                << " occluded=" << statistics.occluded_voxels
                << " unobserved=" << statistics.unobserved_voxels
                << " sync_drop=" << sync_drop_count_.load()
                << " overwritten=" << overwritten_cloud_count_.load()
                << (jump_reset ? " ODOM_JUMP_RESET" : "") << std::endl;
    }
  }

  void publishCloud(const builtin_interfaces::msg::Time & stamp)
  {
    const auto voxels = map_.publishedVoxels();
    sensor_msgs::msg::PointCloud2 output;
    output.header().stamp(stamp);
    output.header().frame_id("m20_local_gravity");
    output.height(1U);
    output.width(static_cast<std::uint32_t>(voxels.size()));
    output.is_bigendian(false);
    output.point_step(16U);
    output.row_step(16U * output.width());
    output.is_dense(true);

    output.fields().resize(4U);
    const char * names[4] = {"x", "y", "z", "intensity"};
    for (std::size_t i = 0; i < 4U; ++i) {
      output.fields()[i].name(names[i]);
      output.fields()[i].offset(static_cast<std::uint32_t>(4U * i));
      output.fields()[i].datatype(7U);  // sensor_msgs/PointField::FLOAT32
      output.fields()[i].count(1U);
    }
    output.data().resize(voxels.size() * 16U);
    for (std::size_t i = 0; i < voxels.size(); ++i) {
      const float values[4] = {
        voxels[i].position.x(), voxels[i].position.y(), voxels[i].position.z(),
        voxels[i].occupancy_score};
      std::memcpy(output.data().data() + i * 16U, values, sizeof(values));
    }
    output_channel_->Write(&output);
  }

  void publishTransform(
    const builtin_interfaces::msg::Time & stamp,
    const Eigen::Isometry3d & world_from_gravity)
  {
    tf2_msgs::msg::TFMessage message;
    message.transforms().resize(1U);
    auto & transform = message.transforms().front();
    transform.header().stamp(stamp);
    transform.header().frame_id("world");
    transform.child_frame_id("m20_local_gravity");
    transform.transform().translation().x(world_from_gravity.translation().x());
    transform.transform().translation().y(world_from_gravity.translation().y());
    transform.transform().translation().z(world_from_gravity.translation().z());
    const double yaw = std::atan2(
      world_from_gravity.linear()(1, 0), world_from_gravity.linear()(0, 0));
    transform.transform().rotation().x(0.0);
    transform.transform().rotation().y(0.0);
    transform.transform().rotation().z(std::sin(0.5 * yaw));
    transform.transform().rotation().w(std::cos(0.5 * yaw));
    tf_channel_->Write(&message);
  }

  void watchdogLoop()
  {
    while (!stop_.load()) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
      const double now = steadySeconds();
      const double cloud_age = now - last_cloud_receive_.load();
      const double imu_age = now - last_imu_receive_.load();
      const double odom_age = now - last_odom_receive_.load();
      if (cloud_age > 0.5 || imu_age > 0.5 || odom_age > 0.5) {
        std::cerr << "INPUT_TIMEOUT cloud=" << cloud_age
                  << " imu=" << imu_age << " odom=" << odom_age
                  << " matches(cloud,imu,odom,out)="
                  << cloud_channel_->GetMatchedCount() << ","
                  << imu_channel_->GetMatchedCount() << ","
                  << odom_channel_->GetMatchedCount() << ","
                  << output_channel_->GetMatchedCount() << std::endl;
      }
    }
  }

  EvidenceMap map_;
  std::vector<SensorModel> sensors_body_;
  std::mutex buffer_mutex_;
  std::deque<TimedOrientation> imu_buffer_;
  std::deque<TimedPose> odom_buffer_;
  std::mutex cloud_mutex_;
  std::condition_variable cloud_condition_;
  sensor_msgs::msg::PointCloud2 latest_cloud_;
  bool cloud_pending_{false};
  Eigen::Isometry3d previous_world_from_gravity_{Eigen::Isometry3d::Identity()};
  bool have_previous_pose_{false};
  double last_cloud_stamp_{0.0};

  std::atomic<double> last_cloud_receive_{0.0};
  std::atomic<double> last_imu_receive_{0.0};
  std::atomic<double> last_odom_receive_{0.0};
  std::atomic<std::uint64_t> processed_frame_count_{0U};
  std::atomic<std::uint64_t> sync_drop_count_{0U};
  std::atomic<std::uint64_t> jump_reset_count_{0U};
  std::atomic<std::uint64_t> time_reset_count_{0U};
  std::atomic<std::uint64_t> invalid_imu_count_{0U};
  std::atomic<std::uint64_t> invalid_odom_count_{0U};
  std::atomic<std::uint64_t> overwritten_cloud_count_{0U};
  std::atomic<bool> stop_{false};
  std::thread cloud_worker_thread_;
  std::thread watchdog_thread_;

  std::unique_ptr<DrDDSChannel<sensor_msgs::msg::PointCloud2PubSubType>> output_channel_;
  std::unique_ptr<DrDDSChannel<tf2_msgs::msg::TFMessagePubSubType>> tf_channel_;
  std::unique_ptr<DrDDSChannel<sensor_msgs::msg::ImuPubSubType>> imu_channel_;
  std::unique_ptr<DrDDSChannel<nav_msgs::msg::OdometryPubSubType>> odom_channel_;
  std::unique_ptr<DrDDSChannel<sensor_msgs::msg::PointCloud2PubSubType>> cloud_channel_;
};
}  // namespace m20_evidence_mapping

int main()
{
  DrDDSManager::Init(0, "eth0");
  {
    m20_evidence_mapping::DrddsEvidenceNode node;
    while (DrDDSManager::Ok()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
  }
  DrDDSManager::Delete();
  return 0;
}
