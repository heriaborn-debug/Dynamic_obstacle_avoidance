#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstdint>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

namespace m20_evidence_mapping
{

struct VoxelKey
{
  int32_t x{};
  int32_t y{};
  int32_t z{};

  bool operator==(const VoxelKey & other) const noexcept
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey & key) const noexcept;
};

struct EvidenceVoxel
{
  Eigen::Vector3f position{Eigen::Vector3f::Zero()};
  float occupancy_score{0.0F};
  float free_score{0.0F};
  uint32_t last_hit_frame{0U};
  uint16_t hit_count{0U};
  uint32_t consecutive_free_frames{0U};
};

struct SensorModel
{
  std::string name;
  Eigen::Vector3f origin{Eigen::Vector3f::Zero()};
  Eigen::Matrix3f rotation_body_sensor{Eigen::Matrix3f::Identity()};
  Eigen::Vector3f frustum_apex{Eigen::Vector3f::Zero()};
  bool use_frustum{true};
  float fov_min_deg{0.0F};
  float fov_max_deg{45.0F};
  float effective_elevation_min_deg{0.0F};
  float effective_elevation_max_deg{90.0F};
  bool extra_coverage_enabled{true};
  float extra_yaw_min_deg{-90.0F};
  float extra_yaw_max_deg{90.0F};
  float extra_pitch_min_deg{35.0F};
  float extra_pitch_max_deg{90.0F};
  float extra_max_range{3.13F};
  float min_range{0.10F};
};

struct EvidenceMapConfig
{
  float voxel_size_xy{0.045F};
  float voxel_size_z{0.060F};

  float roi_min_x{-4.0F};
  float roi_max_x{4.0F};
  float roi_min_y{-4.0F};
  float roi_max_y{4.0F};
  float roi_min_z{-1.0F};
  float roi_max_z{0.8F};

  float body_min_x{-0.40F};
  float body_max_x{0.40F};
  float body_min_y{-0.20F};
  float body_max_y{0.20F};
  float body_min_z{-0.50F};
  float body_max_z{0.40F};

  int azimuth_bins{640};
  int elevation_bins{40};
  int visibility_neighbor_radius{1};
  int visibility_protect_radius{0};

  uint32_t weak_free_confirm_frames{5U};
  float near_field_protect_range{0.10F};
  float match_margin_base{0.06F};
  float match_margin_scale{0.015F};
  float free_margin_base{0.10F};
  float free_margin_scale{0.020F};

  float occupied_gain{1.0F};
  float visibility_match_gain_scale{0.25F};
  float strong_free_gain{1.4F};
  float weak_free_gain{0.50F};
  float relief_gain{0.50F};
  float publish_score{0.60F};
  float remove_score{0.15F};
  float score_cap{6.0F};
};

struct ProcessStatistics
{
  uint64_t input_points{0U};
  uint64_t accepted_points{0U};
  uint64_t map_voxels_before{0U};
  uint64_t map_voxels_after{0U};
  uint64_t matched_voxels{0U};
  uint64_t strong_cleared_voxels{0U};
  uint64_t weak_cleared_voxels{0U};
  uint64_t occluded_voxels{0U};
  uint64_t unobserved_voxels{0U};
};

class EvidenceMap
{
public:
  explicit EvidenceMap(EvidenceMapConfig config);

  void reset();
  std::size_t size() const noexcept;

  ProcessStatistics processFrame(
    const std::vector<Eigen::Vector3f> & visibility_points,
    const std::vector<Eigen::Vector3f> & integration_points,
    const Eigen::Isometry3f & current_from_previous,
    const std::vector<SensorModel> & sensors);

  std::vector<EvidenceVoxel> publishedVoxels() const;
  const std::unordered_map<VoxelKey, EvidenceVoxel, VoxelKeyHash> & data() const noexcept;

private:
  struct VisibilityBin
  {
    float nearest_range{std::numeric_limits<float>::infinity()};
    uint16_t hit_count{0U};
    bool protected_bin{false};
  };

  struct SensorVisibilityMap
  {
    std::vector<VisibilityBin> bins;
  };

  struct Projection
  {
    int sensor_index{-1};
    int azimuth_index{-1};
    int elevation_index{-1};
    float range{0.0F};
    float forward_score{-1.0F};
  };

  struct Accumulator
  {
    Eigen::Vector3f sum{Eigen::Vector3f::Zero()};
    uint32_t count{0U};
  };

  VoxelKey keyFor(const Eigen::Vector3f & point) const;
  bool insideRoi(const Eigen::Vector3f & point) const;
  bool insideBody(const Eigen::Vector3f & point) const;
  bool projectToSensor(
    const Eigen::Vector3f & point, const SensorModel & sensor,
    int sensor_index, Projection & projection) const;
  bool projectToBestSensor(
    const Eigen::Vector3f & point, const std::vector<SensorModel> & sensors,
    Projection & projection) const;
  std::size_t visibilityIndex(int azimuth_index, int elevation_index) const;
  int wrapAzimuth(int index) const;
  float matchMargin(float range) const;
  float freeMargin(float range) const;

  void transformAndReindex(const Eigen::Isometry3f & current_from_previous);
  std::vector<SensorVisibilityMap> buildVisibilityMaps(
    const std::vector<Eigen::Vector3f> & points,
    const std::vector<SensorModel> & sensors) const;
  void applyVisibilityEvidence(
    const std::vector<SensorVisibilityMap> & visibility_maps,
    const std::vector<SensorModel> & sensors,
    ProcessStatistics & statistics);
  void integrate(
    const std::vector<Eigen::Vector3f> & points,
    ProcessStatistics & statistics);
  void applyFreeEvidence(
    EvidenceVoxel & voxel, float gain, bool strong,
    ProcessStatistics & statistics);

  EvidenceMapConfig config_;
  std::unordered_map<VoxelKey, EvidenceVoxel, VoxelKeyHash> voxels_;
  uint32_t frame_index_{0U};
};

}  // namespace m20_evidence_mapping
