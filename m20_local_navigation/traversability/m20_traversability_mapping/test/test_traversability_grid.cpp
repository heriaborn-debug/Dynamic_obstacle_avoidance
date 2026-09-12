#include "m20_traversability_mapping/traversability_grid.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

using m20_traversability_mapping::EvidencePoint;
using m20_traversability_mapping::TraversabilityConfig;
using m20_traversability_mapping::TraversabilityGrid;

namespace
{
std::vector<EvidencePoint> flatPatch(const float z, const int radius = 12)
{
  std::vector<EvidencePoint> points;
  for (int y = -radius; y <= radius; ++y) {
    for (int x = -radius; x <= radius; ++x) {
      points.push_back(EvidencePoint{0.05F * x, 0.05F * y, z, 3.0F});
    }
  }
  return points;
}

const m20_traversability_mapping::TerrainCell & cellAt(
  const TraversabilityGrid & grid, const float x, const float y)
{
  int gx = 0;
  int gy = 0;
  EXPECT_TRUE(grid.coordinates(x, y, gx, gy));
  return grid.cells()[static_cast<std::size_t>(grid.index(gx, gy))];
}
}

TEST(TraversabilityGrid, FlatTerrainIsKnownAndFree)
{
  TraversabilityConfig config;
  config.inflation_radius = 0.0F;
  config.center_padding_enabled = false;
  config.treat_uncoverable_as_missing = false;
  TraversabilityGrid grid(config);
  const auto statistics = grid.build(flatPatch(-0.48F));
  const auto & center = cellAt(grid, 0.0F, 0.0F);
  EXPECT_TRUE(center.observed);
  EXPECT_FALSE(center.blocked);
  EXPECT_LE(center.cost, 2);
  EXPECT_NEAR(center.slope_deg, 0.0F, 0.1F);
  EXPECT_GT(statistics.unknown_cells, 0U);
  EXPECT_EQ(cellAt(grid, 2.0F, 2.0F).cost, config.missing_cost);
}

TEST(TraversabilityGrid, StepAboveLimitIsBlocked)
{
  TraversabilityConfig config;
  config.inflation_radius = 0.0F;
  config.center_padding_enabled = false;
  TraversabilityGrid grid(config);
  auto points = flatPatch(-0.48F);
  for (auto & point : points) {
    if (point.x >= 0.0F) {point.z += 0.40F;}
  }
  grid.build(points);
  EXPECT_FALSE(cellAt(grid, -0.025F, 0.0F).blocked);
  EXPECT_TRUE(cellAt(grid, 0.025F, 0.0F).blocked);
}

TEST(TraversabilityGrid, LowOverheadReturnBlocksCell)
{
  TraversabilityConfig config;
  config.inflation_radius = 0.0F;
  config.center_padding_enabled = false;
  TraversabilityGrid grid(config);
  auto points = flatPatch(-0.48F);
  points.push_back(EvidencePoint{0.0F, 0.0F, -0.18F, 3.0F});
  grid.build(points);
  const auto & center = cellAt(grid, 0.0F, 0.0F);
  EXPECT_TRUE(center.blocked);
  EXPECT_NEAR(center.clearance, 0.30F, 0.01F);
}

TEST(TraversabilityGrid, HighOverheadReturnPreservesGroundClearance)
{
  TraversabilityConfig config;
  config.inflation_radius = 0.0F;
  config.center_padding_enabled = false;
  TraversabilityGrid grid(config);
  auto points = flatPatch(-0.48F);
  points.push_back(EvidencePoint{0.0F, 0.0F, 0.12F, 3.0F});
  grid.build(points);
  const auto & center = cellAt(grid, 0.0F, 0.0F);
  EXPECT_TRUE(center.floating_mask);
  EXPECT_NEAR(center.clearance, 0.60F, 0.01F);
  EXPECT_FALSE(center.blocked);
}

TEST(TraversabilityGrid, SparseExpectedGroundIsUncertainNotHardBlocked)
{
  TraversabilityConfig config;
  config.inflation_radius = 0.0F;
  config.center_padding_enabled = false;
  TraversabilityGrid grid(config);
  grid.build({EvidencePoint{0.0F, 0.0F, -0.48F, 3.0F}});
  const auto & center = cellAt(grid, 0.0F, 0.0F);
  EXPECT_FALSE(center.metrics_valid);
  EXPECT_FALSE(center.blocked);
  EXPECT_EQ(center.cost, config.missing_cost);
}

TEST(TraversabilityGrid, SparseHighReturnIsBlockedAsUnsupportedObject)
{
  TraversabilityConfig config;
  config.inflation_radius = 0.0F;
  config.center_padding_enabled = false;
  TraversabilityGrid grid(config);
  grid.build({EvidencePoint{0.0F, 0.0F, 0.0F, 3.0F}});
  EXPECT_TRUE(cellAt(grid, 0.0F, 0.0F).blocked);
}

TEST(TraversabilityGrid, SparseHighReturnCannotBecomeGroundReference)
{
  TraversabilityConfig config;
  config.inflation_radius = 0.0F;
  config.center_padding_enabled = false;
  TraversabilityGrid grid(config);
  const auto statistics = grid.build({EvidencePoint{0.0F, 0.0F, 0.0F, 3.0F}});
  EXPECT_NEAR(statistics.ground_reference_z, config.expected_ground_z, 0.001F);
}

TEST(TraversabilityGrid, SparseBelowGroundReturnDoesNotReplaceDominantFloor)
{
  TraversabilityConfig config;
  config.inflation_radius = 0.0F;
  config.center_padding_enabled = false;
  TraversabilityGrid grid(config);
  auto points = flatPatch(-0.48F);
  points.push_back(EvidencePoint{0.0F, 0.0F, -0.85F, 1.0F});
  grid.build(points);
  const auto & center = cellAt(grid, 0.0F, 0.0F);
  EXPECT_NEAR(center.ground_z, -0.48F, 0.01F);
  EXPECT_FALSE(center.blocked);
}

TEST(TraversabilityGrid, GroundReferenceAdaptsToCurrentBodyHeight)
{
  TraversabilityConfig config;
  config.inflation_radius = 0.0F;
  config.center_padding_enabled = false;
  TraversabilityGrid grid(config);
  const auto statistics = grid.build(flatPatch(-0.10F));
  EXPECT_NEAR(statistics.ground_reference_z, -0.10F, 0.01F);
  EXPECT_NEAR(cellAt(grid, 0.0F, 0.0F).ground_z, -0.10F, 0.01F);
  EXPECT_FALSE(cellAt(grid, 0.0F, 0.0F).blocked);
}

TEST(TraversabilityGrid, BoundedConsistentHoleIsFilledButOutsideRemainsUnknown)
{
  TraversabilityConfig config;
  config.inflation_radius = 0.0F;
  config.center_padding_enabled = false;
  TraversabilityGrid grid(config);
  auto points = flatPatch(-0.48F, 6);
  points.erase(
    std::remove_if(points.begin(), points.end(), [](const EvidencePoint & point) {
      return std::abs(point.x) < 0.02F && std::abs(point.y) < 0.02F;
    }), points.end());
  const auto statistics = grid.build(points);
  const auto & center = cellAt(grid, 0.0F, 0.0F);
  EXPECT_TRUE(center.observed);
  EXPECT_TRUE(center.interpolated);
  EXPECT_FALSE(cellAt(grid, 2.0F, 2.0F).observed);
  EXPECT_GT(statistics.interpolated_cells, 0U);
}

TEST(TraversabilityGrid, CenterBlindAreaUsesExplicitPaddingLayer)
{
  TraversabilityConfig config;
  config.inflation_radius = 0.0F;
  TraversabilityGrid grid(config);
  const auto statistics = grid.build({});
  const auto & center = cellAt(grid, 0.0F, 0.0F);
  EXPECT_TRUE(center.observed);
  EXPECT_TRUE(center.padded);
  EXPECT_TRUE(center.coverable);
  EXPECT_FALSE(center.blocked);
  EXPECT_GT(statistics.padded_cells, 0U);
  EXPECT_FALSE(cellAt(grid, 1.0F, 0.0F).observed);
}
