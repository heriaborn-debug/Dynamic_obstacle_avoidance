#include "m20_traversability_mapping/traversability_grid.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace m20_traversability_mapping
{

namespace
{
diagnostic_msgs::msg::KeyValue keyValue(const std::string & key, const uint64_t value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = std::to_string(value);
  return item;
}

diagnostic_msgs::msg::KeyValue keyValue(const std::string & key, const float value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = std::to_string(value);
  return item;
}
}

class TraversabilityMappingNode final : public rclcpp::Node
{
public:
  TraversabilityMappingNode()
  : Node("m20_traversability_mapping"), grid_(loadConfig())
  {
    input_topic_ = declare_parameter<std::string>(
      "topics.evidence_input", "/m20/local_mapping/evidence_cloud");
    grid_topic_ = declare_parameter<std::string>(
      "topics.traversability_grid", "/m20/local_mapping/traversability/grid");
    obstacle_topic_ = declare_parameter<std::string>(
      "topics.obstacle_cloud", "/m20/local_mapping/traversability/obstacle_cloud");
    terrain_topic_ = declare_parameter<std::string>(
      "topics.terrain_cloud", "/m20/local_mapping/traversability/terrain_cloud");
    diagnostics_topic_ = declare_parameter<std::string>(
      "topics.diagnostics", "/m20/local_mapping/traversability/diagnostics");
    stale_timeout_sec_ = declare_parameter<double>("watchdog.stale_timeout_sec", 0.5);

    const auto input_qos = rclcpp::QoS(rclcpp::KeepLast(2)).reliable().durability_volatile();
    const auto output_qos = rclcpp::QoS(rclcpp::KeepLast(2)).reliable().durability_volatile();
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, input_qos,
      std::bind(&TraversabilityMappingNode::onCloud, this, std::placeholders::_1));
    grid_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(grid_topic_, output_qos);
    obstacle_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(obstacle_topic_, output_qos);
    terrain_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(terrain_topic_, output_qos);
    diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic_, rclcpp::QoS(10).reliable());
    watchdog_ = create_wall_timer(
      std::chrono::milliseconds(200), std::bind(&TraversabilityMappingNode::publishWatchdog, this));
    last_input_time_ = now();
    RCLCPP_INFO(
      get_logger(), "Traversability mapper ready: %s -> %s (no motion publishers)",
      input_topic_.c_str(), grid_topic_.c_str());
  }

private:
  TraversabilityConfig loadConfig()
  {
    TraversabilityConfig config;
    config.length_x = static_cast<float>(declare_parameter<double>("grid.length_x", 8.0));
    config.length_y = static_cast<float>(declare_parameter<double>("grid.length_y", 8.0));
    config.resolution = static_cast<float>(declare_parameter<double>("grid.resolution", 0.05));
    config.min_z = static_cast<float>(declare_parameter<double>("grid.min_z", -1.0));
    config.max_z = static_cast<float>(declare_parameter<double>("grid.max_z", 0.8));
    config.expected_ground_z = static_cast<float>(
      declare_parameter<double>("surface.expected_ground_z", -0.48));
    config.vertical_cluster_gap = static_cast<float>(
      declare_parameter<double>("surface.vertical_cluster_gap", 0.10));
    config.ground_reference_region_enabled = declare_parameter<bool>(
      "surface.ground_reference.region_enabled", true);
    config.ground_reference_region_min_x = static_cast<float>(declare_parameter<double>(
      "surface.ground_reference.region_min_x", -3.0));
    config.ground_reference_region_max_x = static_cast<float>(declare_parameter<double>(
      "surface.ground_reference.region_max_x", 3.0));
    config.ground_reference_region_min_y = static_cast<float>(declare_parameter<double>(
      "surface.ground_reference.region_min_y", -1.0));
    config.ground_reference_region_max_y = static_cast<float>(declare_parameter<double>(
      "surface.ground_reference.region_max_y", 1.0));
    config.ground_reference_window = static_cast<float>(declare_parameter<double>(
      "surface.ground_reference.window", 0.12));
    config.ground_reference_min_support_cells = declare_parameter<int>(
      "surface.ground_reference.min_support_cells", 20);
    config.max_drop = static_cast<float>(declare_parameter<double>("surface.max_drop", 0.25));
    config.min_vertical_obstacle_height = static_cast<float>(
      declare_parameter<double>("surface.min_vertical_obstacle_height", 0.10));
    config.minimum_clearance = static_cast<float>(
      declare_parameter<double>("surface.minimum_clearance", 0.50));
    config.center_padding_enabled = declare_parameter<bool>(
      "coverage.center_padding_enabled", true);
    config.center_padding_distance = static_cast<float>(declare_parameter<double>(
      "coverage.center_padding_distance", 0.80));
    config.raycast_enabled = declare_parameter<bool>("coverage.raycast_enabled", true);
    config.raycast_max_distance = static_cast<float>(declare_parameter<double>(
      "coverage.raycast_max_distance", 4.0));
    config.raycast_max_nan_gap_cells = declare_parameter<int>(
      "coverage.raycast_max_nan_gap_cells", 3);
    config.front_lidar_x = static_cast<float>(declare_parameter<double>(
      "coverage.front_lidar_x", 0.32028));
    config.front_lidar_y = static_cast<float>(declare_parameter<double>(
      "coverage.front_lidar_y", 0.0));
    config.rear_lidar_x = static_cast<float>(declare_parameter<double>(
      "coverage.rear_lidar_x", -0.32028));
    config.rear_lidar_y = static_cast<float>(declare_parameter<double>(
      "coverage.rear_lidar_y", 0.0));
    config.treat_uncoverable_as_missing = declare_parameter<bool>(
      "coverage.treat_uncoverable_as_missing", true);
    config.plane_radius_cells = declare_parameter<int>("terrain.plane_radius_cells", 1);
    config.min_plane_samples = declare_parameter<int>("terrain.min_plane_samples", 5);
    config.slope_free_deg = static_cast<float>(
      declare_parameter<double>("terrain.slope_free_deg", 10.0));
    config.slope_block_deg = static_cast<float>(
      declare_parameter<double>("terrain.slope_block_deg", 80.0));
    config.max_slope_deg = static_cast<float>(
      declare_parameter<double>("terrain.max_slope_deg", 89.0));
    config.roughness_free = static_cast<float>(
      declare_parameter<double>("terrain.roughness_free", 0.01));
    config.roughness_block = static_cast<float>(
      declare_parameter<double>("terrain.roughness_block", 0.04));
    config.max_roughness = static_cast<float>(
      declare_parameter<double>("terrain.max_roughness", 0.10));
    config.step_free = static_cast<float>(declare_parameter<double>("terrain.step_free", 0.08));
    config.step_block = static_cast<float>(declare_parameter<double>("terrain.step_block", 0.35));
    config.max_step_height = static_cast<float>(
      declare_parameter<double>("terrain.max_step_height", 0.35));
    config.slope_weight = static_cast<float>(declare_parameter<double>("cost.slope_weight", 0.30));
    config.roughness_weight = static_cast<float>(
      declare_parameter<double>("cost.roughness_weight", 0.30));
    config.step_weight = static_cast<float>(declare_parameter<double>("cost.step_weight", 0.40));
    config.missing_cost = declare_parameter<int>("cost.missing_cost", 20);
    config.hard_cost = declare_parameter<int>("cost.hard_cost", 90);
    config.max_inpaint_cells = declare_parameter<int>("inpainting.max_cells", 200);
    config.min_inpaint_boundary_cells = declare_parameter<int>("inpainting.min_boundary_cells", 6);
    config.max_inpaint_boundary_range = static_cast<float>(
      declare_parameter<double>("inpainting.max_boundary_height_range", 0.10));
    config.inflation_radius = static_cast<float>(
      declare_parameter<double>("safety.inflation_radius", 0.0));
    config.evidence_full_confidence = static_cast<float>(
      declare_parameter<double>("confidence.full_evidence_score", 3.0));
    return config;
  }

  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    pcl::PointCloud<pcl::PointXYZI> cloud;
    try {
      pcl::fromROSMsg(*message, cloud);
    } catch (const std::exception & error) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Invalid evidence cloud: %s", error.what());
      return;
    }
    std::vector<EvidencePoint> points;
    points.reserve(cloud.size());
    for (const auto & point : cloud) {
      if (std::isfinite(point.x) && std::isfinite(point.y) &&
        std::isfinite(point.z) && std::isfinite(point.intensity))
      {
        points.push_back(EvidencePoint{point.x, point.y, point.z, point.intensity});
      }
    }
    const auto started = std::chrono::steady_clock::now();
    const auto statistics = grid_.build(points);
    const auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now() - started).count();
    publishGrid(message->header);
    publishClouds(message->header);
    publishDiagnostics(message->header, statistics, static_cast<uint64_t>(elapsed_us));
    last_input_time_ = now();
    have_input_ = true;
  }

  void publishGrid(const std_msgs::msg::Header & header)
  {
    nav_msgs::msg::OccupancyGrid output;
    output.header = header;
    output.info.map_load_time = header.stamp;
    output.info.resolution = grid_.resolution();
    output.info.width = static_cast<uint32_t>(grid_.width());
    output.info.height = static_cast<uint32_t>(grid_.height());
    output.info.origin.position.x = grid_.originX();
    output.info.origin.position.y = grid_.originY();
    output.info.origin.orientation.w = 1.0;
    output.data.reserve(grid_.cells().size());
    for (const auto & cell : grid_.cells()) {output.data.push_back(cell.cost);}
    grid_publisher_->publish(output);
  }

  void publishClouds(const std_msgs::msg::Header & header)
  {
    pcl::PointCloud<pcl::PointXYZI> obstacles;
    pcl::PointCloud<pcl::PointXYZI> terrain;
    for (int y = 0; y < grid_.height(); ++y) {
      for (int x = 0; x < grid_.width(); ++x) {
        const auto & cell = grid_.cells()[static_cast<std::size_t>(grid_.index(x, y))];
        if (!cell.observed) {continue;}
        pcl::PointXYZI point;
        point.x = grid_.cellCenterX(x);
        point.y = grid_.cellCenterY(y);
        point.z = cell.ground_z;
        point.intensity = static_cast<float>(cell.cost);
        terrain.push_back(point);
        if (cell.blocked) {obstacles.push_back(point);}
      }
    }
    sensor_msgs::msg::PointCloud2 obstacle_message;
    sensor_msgs::msg::PointCloud2 terrain_message;
    pcl::toROSMsg(obstacles, obstacle_message);
    pcl::toROSMsg(terrain, terrain_message);
    obstacle_message.header = header;
    terrain_message.header = header;
    obstacle_publisher_->publish(obstacle_message);
    terrain_publisher_->publish(terrain_message);
  }

  void publishDiagnostics(
    const std_msgs::msg::Header & header, const TraversabilityStatistics & statistics,
    const uint64_t elapsed_us)
  {
    diagnostic_msgs::msg::DiagnosticArray message;
    message.header = header;
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "m20_traversability_mapping";
    status.hardware_id = "m20_local_mapping";
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "ACTIVE_NO_MOTION_OUTPUT";
    status.values = {
      keyValue("input_points", statistics.input_points),
      keyValue("accepted_points", statistics.accepted_points),
      keyValue("ground_reference_z", statistics.ground_reference_z),
      keyValue("observed_cells", statistics.observed_cells),
      keyValue("interpolated_cells", statistics.interpolated_cells),
      keyValue("padded_cells", statistics.padded_cells),
      keyValue("floating_cells", statistics.floating_cells),
      keyValue("front_covered_cells", statistics.front_covered_cells),
      keyValue("rear_covered_cells", statistics.rear_covered_cells),
      keyValue("uncoverable_observed_cells", statistics.uncoverable_observed_cells),
      keyValue("unknown_cells", statistics.unknown_cells),
      keyValue("low_clearance_cells", statistics.low_clearance_cells),
      keyValue("unsupported_high_cells", statistics.unsupported_high_cells),
      keyValue("excessive_slope_cells", statistics.excessive_slope_cells),
      keyValue("excessive_roughness_cells", statistics.excessive_roughness_cells),
      keyValue("excessive_step_cells", statistics.excessive_step_cells),
      keyValue("blocked_before_inflation", statistics.blocked_cells_before_inflation),
      keyValue("blocked_after_inflation", statistics.blocked_cells_after_inflation),
      keyValue("processing_time_us", elapsed_us)};
    message.status.push_back(status);
    diagnostics_publisher_->publish(message);
  }

  void publishWatchdog()
  {
    if (have_input_ && (now() - last_input_time_).seconds() <= stale_timeout_sec_) {return;}
    diagnostic_msgs::msg::DiagnosticArray message;
    message.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "m20_traversability_mapping";
    status.hardware_id = "m20_local_mapping";
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = have_input_ ? "STALE_EVIDENCE_INPUT" : "WAITING_FOR_EVIDENCE_INPUT";
    message.status.push_back(status);
    diagnostics_publisher_->publish(message);
  }

  TraversabilityGrid grid_;
  std::string input_topic_;
  std::string grid_topic_;
  std::string obstacle_topic_;
  std::string terrain_topic_;
  std::string diagnostics_topic_;
  double stale_timeout_sec_{0.5};
  bool have_input_{false};
  rclcpp::Time last_input_time_{0, 0, RCL_SYSTEM_TIME};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr terrain_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::TimerBase::SharedPtr watchdog_;
};

}  // namespace m20_traversability_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<m20_traversability_mapping::TraversabilityMappingNode>());
  rclcpp::shutdown();
  return 0;
}
