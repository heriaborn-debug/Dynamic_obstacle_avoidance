#include "m20_traversability_mapping/traversability_grid.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <utility>

namespace m20_traversability_mapping
{

namespace
{
constexpr float kPi = 3.14159265358979323846F;
constexpr float kInfinity = std::numeric_limits<float>::infinity();
}

TraversabilityGrid::TraversabilityGrid(TraversabilityConfig config)
: config_(std::move(config))
{
  if (config_.length_x <= 0.0F || config_.length_y <= 0.0F || config_.resolution <= 0.0F) {
    throw std::invalid_argument("grid dimensions and resolution must be positive");
  }
  if (config_.slope_block_deg <= config_.slope_free_deg ||
    config_.roughness_block <= config_.roughness_free ||
    config_.step_block <= config_.step_free)
  {
    throw std::invalid_argument("block thresholds must be greater than free thresholds");
  }
  if (config_.max_slope_deg < config_.slope_block_deg ||
    config_.max_roughness < config_.roughness_block ||
    config_.max_step_height < config_.step_block)
  {
    throw std::invalid_argument("physical limits must not be below traversal-cost thresholds");
  }
  if (config_.minimum_clearance <= config_.min_vertical_obstacle_height) {
    throw std::invalid_argument("minimum_clearance must exceed vertical obstacle threshold");
  }
  if (config_.max_drop <= 0.0F || config_.center_padding_distance < 0.0F ||
    config_.raycast_max_distance <= 0.0F || config_.raycast_max_nan_gap_cells < 0)
  {
    throw std::invalid_argument("invalid elevation, padding, or raycast parameter");
  }
  if (config_.ground_reference_window <= 0.0F ||
    config_.ground_reference_min_support_cells <= 0 ||
    config_.ground_reference_region_min_x > config_.ground_reference_region_max_x ||
    config_.ground_reference_region_min_y > config_.ground_reference_region_max_y)
  {
    throw std::invalid_argument("invalid ground-reference region, window, or support threshold");
  }
  if (config_.missing_cost < 0 || config_.missing_cost >= config_.hard_cost ||
    config_.hard_cost > 100)
  {
    throw std::invalid_argument("costs must satisfy 0 <= missing_cost < hard_cost <= 100");
  }
  width_ = static_cast<int>(std::lround(config_.length_x / config_.resolution));
  height_ = static_cast<int>(std::lround(config_.length_y / config_.resolution));
  origin_x_ = -0.5F * static_cast<float>(width_) * config_.resolution;
  origin_y_ = -0.5F * static_cast<float>(height_) * config_.resolution;
  cells_.resize(static_cast<std::size_t>(width_ * height_));
}

int TraversabilityGrid::width() const noexcept {return width_;}
int TraversabilityGrid::height() const noexcept {return height_;}
float TraversabilityGrid::originX() const noexcept {return origin_x_;}
float TraversabilityGrid::originY() const noexcept {return origin_y_;}
float TraversabilityGrid::resolution() const noexcept {return config_.resolution;}
int TraversabilityGrid::index(const int x, const int y) const noexcept {return y * width_ + x;}
bool TraversabilityGrid::inside(const int x, const int y) const noexcept
{
  return x >= 0 && x < width_ && y >= 0 && y < height_;
}
bool TraversabilityGrid::coordinates(
  const float x, const float y, int & grid_x, int & grid_y) const noexcept
{
  grid_x = static_cast<int>(std::floor((x - origin_x_) / config_.resolution));
  grid_y = static_cast<int>(std::floor((y - origin_y_) / config_.resolution));
  return inside(grid_x, grid_y);
}
float TraversabilityGrid::cellCenterX(const int x) const noexcept
{
  return origin_x_ + (static_cast<float>(x) + 0.5F) * config_.resolution;
}
float TraversabilityGrid::cellCenterY(const int y) const noexcept
{
  return origin_y_ + (static_cast<float>(y) + 0.5F) * config_.resolution;
}
const std::vector<TerrainCell> & TraversabilityGrid::cells() const noexcept {return cells_;}

TraversabilityStatistics TraversabilityGrid::build(const std::vector<EvidencePoint> & points)
{
  cells_.assign(cells_.size(), TerrainCell{});
  std::vector<Column> columns(cells_.size());
  TraversabilityStatistics statistics;
  statistics.input_points = points.size();
  for (const auto & point : points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) ||
      point.z < config_.min_z || point.z > config_.max_z)
    {
      continue;
    }
    int x = 0;
    int y = 0;
    if (!coordinates(point.x, point.y, x, y)) {
      continue;
    }
    auto & column = columns[static_cast<std::size_t>(index(x, y))];
    column.heights.push_back(point.z);
    column.evidence_sum += std::max(0.0F, point.evidence);
    ++statistics.accepted_points;
  }
  statistics.ground_reference_z = estimateGroundReference(columns);
  extractSurfaces(columns, statistics.ground_reference_z);
  applyCenterPadding(statistics.ground_reference_z, statistics);
  inpaintSmallHoles(statistics);
  calculateCoverage(statistics);
  calculateTerrainMetrics();
  classify(statistics, statistics.ground_reference_z);
  inflate(statistics);
  for (const auto & cell : cells_) {
    if (cell.observed) {
      ++statistics.observed_cells;
      if (cell.floating_mask) {++statistics.floating_cells;}
    } else {
      ++statistics.unknown_cells;
    }
  }
  return statistics;
}

std::vector<TraversabilityGrid::SurfaceCluster> TraversabilityGrid::clusterHeights(
  std::vector<float> heights) const
{
  std::vector<SurfaceCluster> clusters;
  if (heights.empty()) {return clusters;}
  std::sort(heights.begin(), heights.end());
  std::size_t begin = 0U;
  for (std::size_t i = 1U; i <= heights.size(); ++i) {
    if (i == heights.size() || heights[i] - heights[i - 1U] > config_.vertical_cluster_gap) {
      float sum = 0.0F;
      for (std::size_t j = begin; j < i; ++j) {sum += heights[j];}
      clusters.push_back(SurfaceCluster{sum / static_cast<float>(i - begin), i - begin});
      begin = i;
    }
  }
  return clusters;
}

float TraversabilityGrid::estimateGroundReference(const std::vector<Column> & columns) const
{
  std::vector<float> candidates;
  candidates.reserve(columns.size());
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      const float center_x = cellCenterX(x);
      const float center_y = cellCenterY(y);
      if (config_.ground_reference_region_enabled &&
        (center_x < config_.ground_reference_region_min_x ||
        center_x > config_.ground_reference_region_max_x ||
        center_y < config_.ground_reference_region_min_y ||
        center_y > config_.ground_reference_region_max_y))
      {
        continue;
      }
      const auto clusters = clusterHeights(
        columns[static_cast<std::size_t>(index(x, y))].heights);
      if (!clusters.empty()) {
        // One vote per observed XY column prevents vertical walls and dense
        // ceilings from overpowering the terrain surface.
        candidates.push_back(clusters.front().mean);
      }
    }
  }
  if (candidates.size() < static_cast<std::size_t>(config_.ground_reference_min_support_cells)) {
    return config_.expected_ground_z;
  }
  std::sort(candidates.begin(), candidates.end());

  // Robust densest-window mode. Sparse below-ground returns can be lower than
  // the floor, but they cannot become the reference unless spatially dominant.
  std::size_t best_begin = 0U;
  std::size_t best_end = 1U;
  std::size_t begin = 0U;
  for (std::size_t end = 0U; end < candidates.size(); ++end) {
    while (candidates[end] - candidates[begin] > config_.ground_reference_window) {++begin;}
    const std::size_t count = end - begin + 1U;
    const std::size_t best_count = best_end - best_begin;
    const float midpoint = 0.5F * (candidates[begin] + candidates[end]);
    const float best_midpoint =
      0.5F * (candidates[best_begin] + candidates[best_end - 1U]);
    if (count > best_count ||
      (count == best_count &&
      std::abs(midpoint - config_.expected_ground_z) <
      std::abs(best_midpoint - config_.expected_ground_z)))
    {
      best_begin = begin;
      best_end = end + 1U;
    }
  }
  const std::size_t middle = best_begin + (best_end - best_begin) / 2U;
  if (best_end - best_begin <
    static_cast<std::size_t>(config_.ground_reference_min_support_cells))
  {
    return config_.expected_ground_z;
  }
  if ((best_end - best_begin) % 2U == 0U) {
    return 0.5F * (candidates[middle - 1U] + candidates[middle]);
  }
  return candidates[middle];
}

void TraversabilityGrid::extractSurfaces(
  const std::vector<Column> & columns, const float ground_reference_z)
{
  for (std::size_t cell_index = 0; cell_index < columns.size(); ++cell_index) {
    const auto & column = columns[cell_index];
    if (column.heights.empty()) {
      continue;
    }
    const auto clusters = clusterHeights(column.heights);
    const auto closest_iterator = std::min_element(
      clusters.begin(), clusters.end(), [ground_reference_z](const auto & left, const auto & right) {
        const float left_distance = std::abs(left.mean - ground_reference_z);
        const float right_distance = std::abs(right.mean - ground_reference_z);
        if (std::abs(left_distance - right_distance) > 1.0e-6F) {
          return left_distance < right_distance;
        }
        if (left.count != right.count) {return left.count > right.count;}
        return left.mean < right.mean;
      });
    const std::size_t closest_index = static_cast<std::size_t>(
      std::distance(clusters.begin(), closest_iterator));
    const SurfaceCluster & closest_cluster = *closest_iterator;
    auto & cell = cells_[cell_index];
    cell.observed = true;
    const bool unsupported_above_ground =
      closest_cluster.mean - ground_reference_z > config_.max_drop;
    const bool ground_return_available = !unsupported_above_ground;
    const std::size_t ground_index = closest_index;
    cell.ground_valid = ground_return_available;
    cell.floating_mask = unsupported_above_ground;
    cell.ground_z = ground_return_available ? closest_cluster.mean : ground_reference_z;
    cell.ceiling_z = kInfinity;
    if (unsupported_above_ground) {
      cell.ceiling_z = clusters.front().mean;
    } else {
      for (std::size_t i = ground_index + 1U; i < clusters.size(); ++i) {
        if (clusters[i].mean - cell.ground_z >= config_.min_vertical_obstacle_height) {
          cell.ceiling_z = clusters[i].mean;
          cell.floating_mask = true;
          break;
        }
      }
    }
    cell.clearance = std::isfinite(cell.ceiling_z) ?
      cell.ceiling_z - cell.ground_z : kInfinity;
    const float sample_factor = std::min(
      1.0F, static_cast<float>(closest_cluster.count) / 3.0F);
    const float evidence_factor = std::min(
      1.0F, column.evidence_sum / std::max(0.01F, config_.evidence_full_confidence));
    cell.confidence = 0.5F * sample_factor + 0.5F * evidence_factor;
  }
}

void TraversabilityGrid::applyCenterPadding(
  const float ground_reference_z, TraversabilityStatistics & statistics)
{
  if (!config_.center_padding_enabled) {return;}
  const float squared_limit =
    config_.center_padding_distance * config_.center_padding_distance;
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      auto & cell = cells_[static_cast<std::size_t>(index(x, y))];
      if (cell.observed) {continue;}
      const float center_x = cellCenterX(x);
      const float center_y = cellCenterY(y);
      if (center_x * center_x + center_y * center_y > squared_limit) {continue;}
      cell.observed = true;
      cell.ground_valid = true;
      cell.interpolated = true;
      cell.padded = true;
      cell.ground_z = ground_reference_z;
      cell.ceiling_z = kInfinity;
      cell.clearance = kInfinity;
      cell.confidence = 0.25F;
      ++statistics.padded_cells;
    }
  }
}

void TraversabilityGrid::inpaintSmallHoles(TraversabilityStatistics & statistics)
{
  std::vector<uint8_t> visited(cells_.size(), 0U);
  constexpr int dx[4] = {1, -1, 0, 0};
  constexpr int dy[4] = {0, 0, 1, -1};
  for (int seed_y = 0; seed_y < height_; ++seed_y) {
    for (int seed_x = 0; seed_x < width_; ++seed_x) {
      const int seed = index(seed_x, seed_y);
      if (visited[static_cast<std::size_t>(seed)] || cells_[static_cast<std::size_t>(seed)].observed) {
        continue;
      }
      std::queue<std::pair<int, int>> queue;
      std::vector<int> component;
      std::vector<int> boundary;
      bool touches_map_edge = false;
      visited[static_cast<std::size_t>(seed)] = 1U;
      queue.emplace(seed_x, seed_y);
      while (!queue.empty()) {
        const auto [x, y] = queue.front();
        queue.pop();
        component.push_back(index(x, y));
        touches_map_edge = touches_map_edge || x == 0 || y == 0 || x == width_ - 1 || y == height_ - 1;
        for (int direction = 0; direction < 4; ++direction) {
          const int nx = x + dx[direction];
          const int ny = y + dy[direction];
          if (!inside(nx, ny)) {continue;}
          const int neighbor = index(nx, ny);
          if (!cells_[static_cast<std::size_t>(neighbor)].observed &&
            !visited[static_cast<std::size_t>(neighbor)])
          {
            visited[static_cast<std::size_t>(neighbor)] = 1U;
            queue.emplace(nx, ny);
          }
        }
      }
      // Four-connectivity defines the unknown component. Eight-connectivity evaluates
      // its supporting rim so a one-cell hole still has a geometrically meaningful ring.
      for (const int component_index : component) {
        const int x = component_index % width_;
        const int y = component_index / width_;
        for (int by = y - 1; by <= y + 1; ++by) {
          for (int bx = x - 1; bx <= x + 1; ++bx) {
            if (!inside(bx, by) || (bx == x && by == y)) {continue;}
            const int neighbor = index(bx, by);
            if (cells_[static_cast<std::size_t>(neighbor)].observed) {
              boundary.push_back(neighbor);
            }
          }
        }
      }
      std::sort(boundary.begin(), boundary.end());
      boundary.erase(std::unique(boundary.begin(), boundary.end()), boundary.end());
      if (touches_map_edge || component.size() > static_cast<std::size_t>(config_.max_inpaint_cells) ||
        boundary.size() < static_cast<std::size_t>(config_.min_inpaint_boundary_cells))
      {
        continue;
      }
      float minimum = kInfinity;
      float maximum = -kInfinity;
      float sum = 0.0F;
      float confidence_sum = 0.0F;
      for (const int boundary_index : boundary) {
        const auto & cell = cells_[static_cast<std::size_t>(boundary_index)];
        minimum = std::min(minimum, cell.ground_z);
        maximum = std::max(maximum, cell.ground_z);
        sum += cell.ground_z;
        confidence_sum += cell.confidence;
      }
      if (maximum - minimum > config_.max_inpaint_boundary_range) {continue;}
      const float ground = sum / static_cast<float>(boundary.size());
      const float confidence = 0.35F * confidence_sum / static_cast<float>(boundary.size());
      for (const int component_index : component) {
        auto & cell = cells_[static_cast<std::size_t>(component_index)];
        cell.observed = true;
        cell.ground_valid = true;
        cell.interpolated = true;
        cell.ground_z = ground;
        cell.ceiling_z = kInfinity;
        cell.clearance = kInfinity;
        cell.confidence = confidence;
        ++statistics.interpolated_cells;
      }
    }
  }
}

bool TraversabilityGrid::rayHasCoverage(
  const float sensor_x, const float sensor_y, const int target_x, const int target_y) const
{
  if (!config_.raycast_enabled) {return true;}
  int sensor_grid_x = 0;
  int sensor_grid_y = 0;
  if (!coordinates(sensor_x, sensor_y, sensor_grid_x, sensor_grid_y)) {return false;}

  int x = sensor_grid_x;
  int y = sensor_grid_y;
  const int delta_x = std::abs(target_x - sensor_grid_x);
  const int delta_y = std::abs(target_y - sensor_grid_y);
  const int step_x = sensor_grid_x < target_x ? 1 : -1;
  const int step_y = sensor_grid_y < target_y ? 1 : -1;
  int error = delta_x - delta_y;
  int consecutive_unknown = 0;
  while (x != target_x || y != target_y) {
    const int doubled_error = 2 * error;
    if (doubled_error > -delta_y) {
      error -= delta_y;
      x += step_x;
    }
    if (doubled_error < delta_x) {
      error += delta_x;
      y += step_y;
    }
    if (!inside(x, y) || (x == target_x && y == target_y)) {break;}
    const auto & cell = cells_[static_cast<std::size_t>(index(x, y))];
    const float center_x = cellCenterX(x);
    const float center_y = cellCenterY(y);
    const bool center_padding = config_.center_padding_enabled &&
      center_x * center_x + center_y * center_y <=
      config_.center_padding_distance * config_.center_padding_distance;
    if (cell.ground_valid || cell.floating_mask || center_padding) {
      consecutive_unknown = 0;
    } else {
      ++consecutive_unknown;
      if (consecutive_unknown > config_.raycast_max_nan_gap_cells) {return false;}
    }
  }
  return true;
}

void TraversabilityGrid::calculateCoverage(TraversabilityStatistics & statistics)
{
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      auto & cell = cells_[static_cast<std::size_t>(index(x, y))];
      if (!cell.observed) {continue;}
      const float center_x = cellCenterX(x);
      const float center_y = cellCenterY(y);
      const float front_dx = center_x - config_.front_lidar_x;
      const float front_dy = center_y - config_.front_lidar_y;
      const float rear_dx = center_x - config_.rear_lidar_x;
      const float rear_dy = center_y - config_.rear_lidar_y;
      const bool center_padding = cell.padded ||
        (config_.center_padding_enabled &&
        center_x * center_x + center_y * center_y <=
        config_.center_padding_distance * config_.center_padding_distance);
      const bool front_geometry = front_dx >= 0.0F &&
        std::hypot(front_dx, front_dy) <= config_.raycast_max_distance;
      const bool rear_geometry = rear_dx <= 0.0F &&
        std::hypot(rear_dx, rear_dy) <= config_.raycast_max_distance;
      cell.front_covered = center_padding ||
        (front_geometry && rayHasCoverage(config_.front_lidar_x, config_.front_lidar_y, x, y));
      cell.rear_covered = center_padding ||
        (rear_geometry && rayHasCoverage(config_.rear_lidar_x, config_.rear_lidar_y, x, y));
      cell.coverable = cell.front_covered || cell.rear_covered;
      statistics.front_covered_cells += cell.front_covered ? 1U : 0U;
      statistics.rear_covered_cells += cell.rear_covered ? 1U : 0U;
      statistics.uncoverable_observed_cells += cell.coverable ? 0U : 1U;
    }
  }
}

void TraversabilityGrid::calculateTerrainMetrics()
{
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      auto & cell = cells_[static_cast<std::size_t>(index(x, y))];
      if (!cell.observed) {continue;}
      // Fit in cell-local coordinates.  The rolling grid may be centred at a large
      // world coordinate; subtracting the query-cell origin avoids conditioning the
      // normal equations on that arbitrary translation.
      Eigen::Matrix3d normal = Eigen::Matrix3d::Zero();
      Eigen::Vector3d right_hand_side = Eigen::Vector3d::Zero();
      int sample_count = 0;
      for (int ny = y - config_.plane_radius_cells; ny <= y + config_.plane_radius_cells; ++ny) {
        for (int nx = x - config_.plane_radius_cells; nx <= x + config_.plane_radius_cells; ++nx) {
          if (!inside(nx, ny)) {continue;}
          const auto & neighbor = cells_[static_cast<std::size_t>(index(nx, ny))];
          if (neighbor.observed) {
            const Eigen::Vector3d row(
              static_cast<double>(nx - x) * config_.resolution,
              static_cast<double>(ny - y) * config_.resolution, 1.0);
            normal.noalias() += row * row.transpose();
            right_hand_side.noalias() += row * static_cast<double>(neighbor.ground_z);
            ++sample_count;
          }
        }
      }
      if (sample_count >= config_.min_plane_samples) {
        const Eigen::Vector3d plane = normal.ldlt().solve(right_hand_side);
        double squared_error = 0.0;
        for (int ny = y - config_.plane_radius_cells; ny <= y + config_.plane_radius_cells; ++ny) {
          for (int nx = x - config_.plane_radius_cells; nx <= x + config_.plane_radius_cells; ++nx) {
            if (!inside(nx, ny)) {continue;}
            const auto & neighbor = cells_[static_cast<std::size_t>(index(nx, ny))];
            if (!neighbor.observed) {continue;}
            const double local_x = static_cast<double>(nx - x) * config_.resolution;
            const double local_y = static_cast<double>(ny - y) * config_.resolution;
            const double predicted = plane.x() * local_x + plane.y() * local_y + plane.z();
            const double residual = predicted - static_cast<double>(neighbor.ground_z);
            squared_error += residual * residual;
          }
        }
        cell.metrics_valid = plane.allFinite();
        cell.slope_deg = static_cast<float>(
          std::atan(std::hypot(plane.x(), plane.y())) * 180.0 / static_cast<double>(kPi));
        cell.roughness = static_cast<float>(
          std::sqrt(squared_error / static_cast<double>(sample_count)));
      } else {
        // Insufficient support is uncertainty, not physical proof of an obstacle.
        cell.metrics_valid = false;
        cell.slope_deg = 0.0F;
        cell.roughness = 0.0F;
        cell.confidence *= 0.5F;
      }
      float largest_step = 0.0F;
      for (int ny = y - 1; ny <= y + 1; ++ny) {
        for (int nx = x - 1; nx <= x + 1; ++nx) {
          if (!inside(nx, ny) || (nx == x && ny == y)) {continue;}
          const auto & neighbor = cells_[static_cast<std::size_t>(index(nx, ny))];
          if (neighbor.observed) {
            largest_step = std::max(largest_step, std::abs(neighbor.ground_z - cell.ground_z));
          }
        }
      }
      cell.step_height = largest_step;
    }
  }
}

float TraversabilityGrid::normalized(
  const float value, const float free_value, const float block_value) const noexcept
{
  return std::clamp((value - free_value) / (block_value - free_value), 0.0F, 1.0F);
}

void TraversabilityGrid::classify(
  TraversabilityStatistics & statistics, const float ground_reference_z)
{
  for (auto & cell : cells_) {
    // Vendor passable_area assigns missing terrain a finite penalty instead of
    // publishing it as an OccupancyGrid unknown.  Keep observed=false for diagnostics,
    // but preserve the vendor cost semantics for downstream planners.
    if (!cell.observed) {
      cell.cost = static_cast<int8_t>(config_.missing_cost);
      continue;
    }
    const float slope = normalized(cell.slope_deg, config_.slope_free_deg, config_.slope_block_deg);
    const float roughness = normalized(
      cell.roughness, config_.roughness_free, config_.roughness_block);
    const float step = normalized(cell.step_height, config_.step_free, config_.step_block);
    const bool low_clearance = std::isfinite(cell.clearance) &&
      cell.clearance < config_.minimum_clearance;
    const bool unsupported_high_surface = !cell.metrics_valid &&
      std::abs(cell.ground_z - ground_reference_z) >= config_.step_block;
    const bool excessive_slope = cell.metrics_valid && cell.slope_deg >= config_.max_slope_deg;
    const bool excessive_roughness =
      cell.metrics_valid && cell.roughness >= config_.max_roughness;
    const bool excessive_step = cell.step_height >=
      std::min(config_.max_step_height, config_.max_drop);
    statistics.low_clearance_cells += low_clearance ? 1U : 0U;
    statistics.unsupported_high_cells += unsupported_high_surface ? 1U : 0U;
    statistics.excessive_slope_cells += excessive_slope ? 1U : 0U;
    statistics.excessive_roughness_cells += excessive_roughness ? 1U : 0U;
    statistics.excessive_step_cells += excessive_step ? 1U : 0U;
    cell.blocked = low_clearance || unsupported_high_surface ||
      excessive_slope || excessive_roughness || excessive_step;
    if (cell.blocked) {
      cell.cost = static_cast<int8_t>(config_.hard_cost);
      ++statistics.blocked_cells_before_inflation;
    } else {
      const float weighted = config_.slope_weight * slope +
        config_.roughness_weight * roughness + config_.step_weight * step;
      int cost = std::clamp(static_cast<int>(std::lround(weighted * 90.0F)), 0, 90);
      if (!cell.metrics_valid) {cost = std::max(cost, config_.missing_cost);}
      if (config_.treat_uncoverable_as_missing && !cell.coverable) {
        cost = std::max(cost, config_.missing_cost);
      }
      cell.cost = static_cast<int8_t>(cost);
    }
  }
}

void TraversabilityGrid::inflate(TraversabilityStatistics & statistics)
{
  const auto source = cells_;
  const int radius = static_cast<int>(std::ceil(config_.inflation_radius / config_.resolution));
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      if (!source[static_cast<std::size_t>(index(x, y))].blocked) {continue;}
      for (int dy = -radius; dy <= radius; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
          if (dx * dx + dy * dy > radius * radius || !inside(x + dx, y + dy)) {continue;}
          auto & target = cells_[static_cast<std::size_t>(index(x + dx, y + dy))];
          if (!target.observed) {continue;}
          target.blocked = true;
          target.cost = static_cast<int8_t>(config_.hard_cost);
        }
      }
    }
  }
  for (const auto & cell : cells_) {
    if (cell.blocked) {++statistics.blocked_cells_after_inflation;}
  }
}

}  // namespace m20_traversability_mapping
