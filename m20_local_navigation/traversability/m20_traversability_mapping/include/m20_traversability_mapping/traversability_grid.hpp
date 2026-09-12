#pragma once

#include <cstdint>
#include <vector>

namespace m20_traversability_mapping
{

struct EvidencePoint
{
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  float evidence{1.0F};
};

struct TraversabilityConfig
{
  float length_x{8.0F};
  float length_y{8.0F};
  float resolution{0.05F};
  float min_z{-1.0F};
  float max_z{0.8F};
  float expected_ground_z{-0.48F};
  float vertical_cluster_gap{0.10F};
  bool ground_reference_region_enabled{true};
  float ground_reference_region_min_x{-3.0F};
  float ground_reference_region_max_x{3.0F};
  float ground_reference_region_min_y{-1.0F};
  float ground_reference_region_max_y{1.0F};
  float ground_reference_window{0.12F};
  int ground_reference_min_support_cells{20};
  float max_drop{0.25F};
  float min_vertical_obstacle_height{0.10F};
  float minimum_clearance{0.50F};
  bool center_padding_enabled{true};
  float center_padding_distance{0.80F};
  bool raycast_enabled{true};
  float raycast_max_distance{4.0F};
  int raycast_max_nan_gap_cells{3};
  float front_lidar_x{0.32028F};
  float front_lidar_y{0.0F};
  float rear_lidar_x{-0.32028F};
  float rear_lidar_y{0.0F};
  bool treat_uncoverable_as_missing{true};
  int plane_radius_cells{1};
  int min_plane_samples{5};
  float slope_free_deg{10.0F};
  float slope_block_deg{80.0F};
  float max_slope_deg{89.0F};
  float roughness_free{0.01F};
  float roughness_block{0.04F};
  float max_roughness{0.10F};
  float step_free{0.08F};
  float step_block{0.35F};
  float max_step_height{0.35F};
  float slope_weight{0.30F};
  float roughness_weight{0.30F};
  float step_weight{0.40F};
  int missing_cost{20};
  int hard_cost{90};
  int max_inpaint_cells{200};
  int min_inpaint_boundary_cells{6};
  float max_inpaint_boundary_range{0.10F};
  float inflation_radius{0.0F};
  float evidence_full_confidence{3.0F};
};

struct TerrainCell
{
  bool observed{false};
  bool ground_valid{false};
  bool interpolated{false};
  bool padded{false};
  bool floating_mask{false};
  bool front_covered{false};
  bool rear_covered{false};
  bool coverable{false};
  bool metrics_valid{false};
  bool blocked{false};
  float ground_z{0.0F};
  float ceiling_z{0.0F};
  float clearance{0.0F};
  float slope_deg{0.0F};
  float roughness{0.0F};
  float step_height{0.0F};
  float confidence{0.0F};
  int8_t cost{-1};
};

struct TraversabilityStatistics
{
  uint64_t input_points{0U};
  uint64_t accepted_points{0U};
  uint64_t observed_cells{0U};
  uint64_t interpolated_cells{0U};
  uint64_t padded_cells{0U};
  uint64_t floating_cells{0U};
  uint64_t front_covered_cells{0U};
  uint64_t rear_covered_cells{0U};
  uint64_t uncoverable_observed_cells{0U};
  uint64_t unknown_cells{0U};
  uint64_t low_clearance_cells{0U};
  uint64_t unsupported_high_cells{0U};
  uint64_t excessive_slope_cells{0U};
  uint64_t excessive_roughness_cells{0U};
  uint64_t excessive_step_cells{0U};
  uint64_t blocked_cells_before_inflation{0U};
  uint64_t blocked_cells_after_inflation{0U};
  float ground_reference_z{0.0F};
};

class TraversabilityGrid
{
public:
  explicit TraversabilityGrid(TraversabilityConfig config);

  TraversabilityStatistics build(const std::vector<EvidencePoint> & points);
  int width() const noexcept;
  int height() const noexcept;
  float originX() const noexcept;
  float originY() const noexcept;
  float resolution() const noexcept;
  int index(int x, int y) const noexcept;
  bool coordinates(float x, float y, int & grid_x, int & grid_y) const noexcept;
  float cellCenterX(int grid_x) const noexcept;
  float cellCenterY(int grid_y) const noexcept;
  const std::vector<TerrainCell> & cells() const noexcept;

private:
  struct Column
  {
    std::vector<float> heights;
    float evidence_sum{0.0F};
  };

  struct SurfaceCluster
  {
    float mean{0.0F};
    std::size_t count{0U};
  };

  std::vector<SurfaceCluster> clusterHeights(std::vector<float> heights) const;
  float estimateGroundReference(const std::vector<Column> & columns) const;
  void extractSurfaces(const std::vector<Column> & columns, float ground_reference_z);
  void applyCenterPadding(float ground_reference_z, TraversabilityStatistics & statistics);
  void inpaintSmallHoles(TraversabilityStatistics & statistics);
  void calculateCoverage(TraversabilityStatistics & statistics);
  bool rayHasCoverage(float sensor_x, float sensor_y, int target_x, int target_y) const;
  void calculateTerrainMetrics();
  void classify(TraversabilityStatistics & statistics, float ground_reference_z);
  void inflate(TraversabilityStatistics & statistics);
  bool inside(int x, int y) const noexcept;
  float normalized(float value, float free_value, float block_value) const noexcept;

  TraversabilityConfig config_;
  int width_{0};
  int height_{0};
  float origin_x_{0.0F};
  float origin_y_{0.0F};
  std::vector<TerrainCell> cells_;
};

}  // namespace m20_traversability_mapping
