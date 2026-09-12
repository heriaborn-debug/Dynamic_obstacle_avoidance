#include "m20_evidence_mapping/evidence_map.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <vector>

namespace m20_evidence_mapping
{
namespace
{

SensorModel frontSensor()
{
  SensorModel sensor;
  sensor.name = "front";
  sensor.origin = Eigen::Vector3f::Zero();
  sensor.rotation_body_sensor =
    Eigen::AngleAxisf(1.57079632679F, Eigen::Vector3f::UnitY()).toRotationMatrix();
  sensor.azimuth_min_rad = -0.8F;
  sensor.azimuth_max_rad = 0.8F;
  sensor.elevation_min_rad = -0.8F;
  sensor.elevation_max_rad = 0.8F;
  return sensor;
}

TEST(EvidenceMap, IntegratesAndPublishesCurrentHit)
{
  EvidenceMapConfig config;
  EvidenceMap map(config);
  const std::vector<Eigen::Vector3f> points{
    Eigen::Vector3f(1.0F, 0.0F, 0.0F),
    Eigen::Vector3f(1.01F, 0.0F, 0.0F)};
  map.processFrame(points, points, Eigen::Isometry3f::Identity(), {frontSensor()});
  EXPECT_FALSE(map.publishedVoxels().empty());
}

TEST(EvidenceMap, OccludedVoxelIsNotCleared)
{
  EvidenceMapConfig config;
  EvidenceMap map(config);
  const auto sensor = frontSensor();
  const std::vector<Eigen::Vector3f> far_points{
    Eigen::Vector3f(2.0F, 0.0F, 0.0F),
    Eigen::Vector3f(2.01F, 0.0F, 0.0F)};
  map.processFrame(far_points, far_points, Eigen::Isometry3f::Identity(), {sensor});
  const std::size_t initial_size = map.size();
  const std::vector<Eigen::Vector3f> near_points{
    Eigen::Vector3f(1.0F, 0.0F, 0.0F),
    Eigen::Vector3f(1.01F, 0.0F, 0.0F)};
  map.processFrame(near_points, near_points, Eigen::Isometry3f::Identity(), {sensor});
  EXPECT_GE(map.size(), initial_size);
}

TEST(EvidenceMap, StrongFreeEvidenceRemovesOldVoxel)
{
  EvidenceMapConfig config;
  config.visibility_neighbor_radius = 0;
  EvidenceMap map(config);
  const auto sensor = frontSensor();
  const std::vector<Eigen::Vector3f> old_points{
    Eigen::Vector3f(1.0F, 0.0F, 0.0F),
    Eigen::Vector3f(1.01F, 0.0F, 0.0F)};
  map.processFrame(old_points, old_points, Eigen::Isometry3f::Identity(), {sensor});
  const std::vector<Eigen::Vector3f> farther_return{
    Eigen::Vector3f(2.0F, 0.0F, 0.0F),
    Eigen::Vector3f(2.01F, 0.0F, 0.0F)};
  for (int index = 0; index < 3; ++index) {
    map.processFrame(
      farther_return, farther_return, Eigen::Isometry3f::Identity(), {sensor});
  }
  for (const auto & voxel : map.publishedVoxels()) {
    EXPECT_GT(voxel.position.x(), 1.5F);
  }
}

}  // namespace
}  // namespace m20_evidence_mapping
