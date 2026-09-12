#include "m20_evidence_mapping/evidence_map.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace m20_evidence_mapping
{

namespace
{
constexpr float kEpsilon = 1.0e-6F;

inline void hashCombine(std::size_t & seed, const int32_t value) noexcept
{
  seed ^= std::hash<int32_t>{}(value) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
}
}  // namespace

std::size_t VoxelKeyHash::operator()(const VoxelKey & key) const noexcept
{
  std::size_t seed = 0U;
  hashCombine(seed, key.x);
  hashCombine(seed, key.y);
  hashCombine(seed, key.z);
  return seed;
}

EvidenceMap::EvidenceMap(EvidenceMapConfig config)
: config_(std::move(config))
{
  if (config_.voxel_size_xy <= 0.0F || config_.voxel_size_z <= 0.0F) {
    throw std::invalid_argument("voxel sizes must be positive");
  }
  if (config_.azimuth_bins < 8 || config_.elevation_bins < 4) {
    throw std::invalid_argument("visibility image dimensions are too small");
  }
  if (config_.remove_score >= config_.publish_score || config_.publish_score > config_.score_cap) {
    throw std::invalid_argument("invalid evidence score thresholds");
  }
}

void EvidenceMap::reset()
{
  voxels_.clear();
  frame_index_ = 0U;
}

std::size_t EvidenceMap::size() const noexcept
{
  return voxels_.size();
}

const std::unordered_map<VoxelKey, EvidenceVoxel, VoxelKeyHash> &
EvidenceMap::data() const noexcept
{
  return voxels_;
}

VoxelKey EvidenceMap::keyFor(const Eigen::Vector3f & point) const
{
  return VoxelKey{
    static_cast<int32_t>(std::floor(point.x() / config_.voxel_size_xy)),
    static_cast<int32_t>(std::floor(point.y() / config_.voxel_size_xy)),
    static_cast<int32_t>(std::floor(point.z() / config_.voxel_size_z))};
}

bool EvidenceMap::insideRoi(const Eigen::Vector3f & point) const
{
  return point.x() >= config_.roi_min_x && point.x() <= config_.roi_max_x &&
         point.y() >= config_.roi_min_y && point.y() <= config_.roi_max_y &&
         point.z() >= config_.roi_min_z && point.z() <= config_.roi_max_z;
}

bool EvidenceMap::insideBody(const Eigen::Vector3f & point) const
{
  return point.x() >= config_.body_min_x && point.x() <= config_.body_max_x &&
         point.y() >= config_.body_min_y && point.y() <= config_.body_max_y &&
         point.z() >= config_.body_min_z && point.z() <= config_.body_max_z;
}

int EvidenceMap::wrapAzimuth(const int index) const
{
  int wrapped = index % config_.azimuth_bins;
  if (wrapped < 0) {
    wrapped += config_.azimuth_bins;
  }
  return wrapped;
}

std::size_t EvidenceMap::visibilityIndex(
  const int azimuth_index, const int elevation_index) const
{
  return static_cast<std::size_t>(elevation_index * config_.azimuth_bins + azimuth_index);
}

float EvidenceMap::matchMargin(const float range) const
{
  return config_.match_margin_base + config_.match_margin_scale * range;
}

float EvidenceMap::freeMargin(const float range) const
{
  return config_.free_margin_base + config_.free_margin_scale * range;
}

bool EvidenceMap::projectToSensor(
  const Eigen::Vector3f & point, const SensorModel & sensor,
  const int sensor_index, Projection & projection) const
{
  const Eigen::Vector3f delta = point - sensor.origin;
  const float range = delta.norm();
  if (!std::isfinite(range) || range < sensor.min_range || range > sensor.max_range) {
    return false;
  }

  // Manufacturer geometry uses rotation column 2 as the optical/forward axis.
  const Eigen::Vector3f forward = sensor.rotation_body_sensor.col(2);
  const Eigen::Vector3f lateral = sensor.rotation_body_sensor.col(1);
  const Eigen::Vector3f upward = -sensor.rotation_body_sensor.col(0);
  const float f = delta.dot(forward);
  if (f <= kEpsilon) {
    return false;
  }
  const float l = delta.dot(lateral);
  const float u = delta.dot(upward);
  const float azimuth = std::atan2(l, f);
  const float elevation = std::atan2(u, std::hypot(f, l));
  if (azimuth < sensor.azimuth_min_rad || azimuth > sensor.azimuth_max_rad ||
    elevation < sensor.elevation_min_rad || elevation > sensor.elevation_max_rad)
  {
    return false;
  }

  const float azimuth_normalized =
    (azimuth - sensor.azimuth_min_rad) /
    std::max(kEpsilon, sensor.azimuth_max_rad - sensor.azimuth_min_rad);
  const float elevation_normalized =
    (elevation - sensor.elevation_min_rad) /
    std::max(kEpsilon, sensor.elevation_max_rad - sensor.elevation_min_rad);

  projection.sensor_index = sensor_index;
  projection.azimuth_index = std::clamp(
    static_cast<int>(azimuth_normalized * static_cast<float>(config_.azimuth_bins)),
    0, config_.azimuth_bins - 1);
  projection.elevation_index = std::clamp(
    static_cast<int>(elevation_normalized * static_cast<float>(config_.elevation_bins)),
    0, config_.elevation_bins - 1);
  projection.range = range;
  projection.center_score = std::abs(azimuth) + std::abs(elevation);
  return true;
}

bool EvidenceMap::projectToBestSensor(
  const Eigen::Vector3f & point, const std::vector<SensorModel> & sensors,
  Projection & projection) const
{
  bool found = false;
  Projection best;
  for (std::size_t index = 0; index < sensors.size(); ++index) {
    Projection candidate;
    if (!projectToSensor(point, sensors[index], static_cast<int>(index), candidate)) {
      continue;
    }
    if (!found || candidate.center_score < best.center_score) {
      best = candidate;
      found = true;
    }
  }
  if (found) {
    projection = best;
  }
  return found;
}

void EvidenceMap::transformAndReindex(const Eigen::Isometry3f & current_from_previous)
{
  std::unordered_map<VoxelKey, EvidenceVoxel, VoxelKeyHash> transformed;
  transformed.reserve(voxels_.size());

  for (const auto & item : voxels_) {
    EvidenceVoxel voxel = item.second;
    voxel.position = current_from_previous * voxel.position;
    if (!insideRoi(voxel.position) || insideBody(voxel.position)) {
      continue;
    }
    const VoxelKey key = keyFor(voxel.position);
    auto [iterator, inserted] = transformed.emplace(key, voxel);
    if (!inserted) {
      EvidenceVoxel & target = iterator->second;
      const uint32_t combined_hits = std::max(1U, target.hit_count) + std::max(1U, voxel.hit_count);
      target.position =
        (target.position * static_cast<float>(std::max(1U, target.hit_count)) +
        voxel.position * static_cast<float>(std::max(1U, voxel.hit_count))) /
        static_cast<float>(combined_hits);
      target.hit_count = combined_hits;
      target.occupancy_score = std::max(target.occupancy_score, voxel.occupancy_score);
      target.free_score = std::max(target.free_score, voxel.free_score);
      target.last_hit_frame = std::max(target.last_hit_frame, voxel.last_hit_frame);
      target.consecutive_free_frames =
        std::max(target.consecutive_free_frames, voxel.consecutive_free_frames);
    }
  }
  voxels_.swap(transformed);
}

std::vector<EvidenceMap::SensorVisibilityMap> EvidenceMap::buildVisibilityMaps(
  const std::vector<Eigen::Vector3f> & points,
  const std::vector<SensorModel> & sensors) const
{
  const std::size_t bin_count =
    static_cast<std::size_t>(config_.azimuth_bins * config_.elevation_bins);
  std::vector<SensorVisibilityMap> maps(sensors.size());
  for (auto & map : maps) {
    map.bins.resize(bin_count);
  }

  for (const Eigen::Vector3f & point : points) {
    if (!insideRoi(point) || insideBody(point)) {
      continue;
    }
    Projection projection;
    if (!projectToBestSensor(point, sensors, projection)) {
      continue;
    }
    VisibilityBin & bin = maps[static_cast<std::size_t>(projection.sensor_index)].bins[
      visibilityIndex(projection.azimuth_index, projection.elevation_index)];
    bin.nearest_range = std::min(bin.nearest_range, projection.range);
    bin.hit_count = static_cast<uint16_t>(std::min<uint32_t>(65535U, bin.hit_count + 1U));
  }

  // FOV edge rays are protected. Clearing on calibration/FOV boundaries is unsafe.
  for (auto & map : maps) {
    for (int elevation = 0; elevation < config_.elevation_bins; ++elevation) {
      for (int azimuth = 0; azimuth < config_.azimuth_bins; ++azimuth) {
        if (elevation <= config_.visibility_neighbor_radius ||
          elevation >= config_.elevation_bins - 1 - config_.visibility_neighbor_radius ||
          azimuth <= config_.visibility_neighbor_radius ||
          azimuth >= config_.azimuth_bins - 1 - config_.visibility_neighbor_radius)
        {
          map.bins[visibilityIndex(azimuth, elevation)].protected_bin = true;
        }
      }
    }
  }
  return maps;
}

void EvidenceMap::applyFreeEvidence(
  EvidenceVoxel & voxel, const float gain, const bool strong,
  ProcessStatistics & statistics)
{
  voxel.free_score = std::min(config_.score_cap, voxel.free_score + gain);
  voxel.occupancy_score -= voxel.free_score;
  if (strong) {
    ++statistics.strong_cleared_voxels;
  } else {
    ++statistics.weak_cleared_voxels;
  }
}

void EvidenceMap::applyVisibilityEvidence(
  const std::vector<SensorVisibilityMap> & visibility_maps,
  const std::vector<SensorModel> & sensors,
  ProcessStatistics & statistics)
{
  for (auto iterator = voxels_.begin(); iterator != voxels_.end();) {
    EvidenceVoxel & voxel = iterator->second;
    Projection projection;
    if (!projectToBestSensor(voxel.position, sensors, projection)) {
      voxel.consecutive_free_frames = 0U;
      ++statistics.unobserved_voxels;
      ++iterator;
      continue;
    }

    const SensorVisibilityMap & map =
      visibility_maps[static_cast<std::size_t>(projection.sensor_index)];
    bool has_return = false;
    bool protected_ray = false;
    float nearest_range = std::numeric_limits<float>::infinity();
    for (int de = -config_.visibility_neighbor_radius;
      de <= config_.visibility_neighbor_radius; ++de)
    {
      const int elevation = projection.elevation_index + de;
      if (elevation < 0 || elevation >= config_.elevation_bins) {
        protected_ray = true;
        continue;
      }
      for (int da = -config_.visibility_neighbor_radius;
        da <= config_.visibility_neighbor_radius; ++da)
      {
        const int azimuth = wrapAzimuth(projection.azimuth_index + da);
        const VisibilityBin & bin = map.bins[visibilityIndex(azimuth, elevation)];
        protected_ray = protected_ray || bin.protected_bin;
        if (bin.hit_count > 0U && std::isfinite(bin.nearest_range)) {
          has_return = true;
          nearest_range = std::min(nearest_range, bin.nearest_range);
        }
      }
    }

    bool erase = false;
    if (has_return && std::abs(projection.range - nearest_range) <= matchMargin(projection.range)) {
      voxel.occupancy_score = std::min(
        config_.score_cap,
        voxel.occupancy_score + config_.occupied_gain * config_.visibility_match_gain_scale);
      voxel.free_score = std::max(0.0F, voxel.free_score - config_.relief_gain);
      voxel.consecutive_free_frames = 0U;
      ++statistics.matched_voxels;
    } else if (has_return &&
      projection.range + freeMargin(projection.range) < nearest_range)
    {
      voxel.consecutive_free_frames = 0U;
      if (!protected_ray && projection.range > config_.near_field_protect_range) {
        applyFreeEvidence(voxel, config_.strong_free_gain, true, statistics);
        erase = voxel.occupancy_score <= config_.remove_score;
      }
    } else if (has_return &&
      nearest_range + matchMargin(projection.range) < projection.range)
    {
      voxel.free_score = std::max(
        0.0F, voxel.free_score - config_.relief_gain * config_.visibility_match_gain_scale);
      voxel.consecutive_free_frames = 0U;
      ++statistics.occluded_voxels;
    } else if (!has_return && !protected_ray &&
      projection.range > config_.near_field_protect_range)
    {
      voxel.consecutive_free_frames = std::min<uint32_t>(
        std::numeric_limits<uint32_t>::max() - 1U,
        voxel.consecutive_free_frames + 1U);
      if (voxel.consecutive_free_frames >= config_.weak_free_confirm_frames) {
        applyFreeEvidence(voxel, config_.weak_free_gain, false, statistics);
        erase = voxel.occupancy_score <= config_.remove_score;
      }
    } else {
      voxel.consecutive_free_frames = 0U;
      ++statistics.unobserved_voxels;
    }

    if (erase) {
      iterator = voxels_.erase(iterator);
    } else {
      ++iterator;
    }
  }
}

void EvidenceMap::integrate(
  const std::vector<Eigen::Vector3f> & points,
  ProcessStatistics & statistics)
{
  std::unordered_map<VoxelKey, Accumulator, VoxelKeyHash> accumulators;
  accumulators.reserve(points.size() / 2U + 1U);
  for (const Eigen::Vector3f & point : points) {
    ++statistics.input_points;
    if (!point.allFinite() || !insideRoi(point) || insideBody(point)) {
      continue;
    }
    ++statistics.accepted_points;
    Accumulator & accumulator = accumulators[keyFor(point)];
    accumulator.sum += point;
    ++accumulator.count;
  }

  for (const auto & item : accumulators) {
    const Accumulator & accumulator = item.second;
    const Eigen::Vector3f mean = accumulator.sum / static_cast<float>(accumulator.count);
    auto [iterator, inserted] = voxels_.try_emplace(item.first);
    EvidenceVoxel & voxel = iterator->second;
    if (inserted || voxel.hit_count == 0U) {
      voxel.position = mean;
    } else {
      const uint32_t old_weight = std::max(1U, voxel.hit_count);
      const uint32_t combined = old_weight + accumulator.count;
      voxel.position =
        (voxel.position * static_cast<float>(old_weight) + accumulator.sum) /
        static_cast<float>(combined);
    }
    voxel.hit_count = std::min<uint32_t>(
      std::numeric_limits<uint32_t>::max() - accumulator.count,
      voxel.hit_count) + accumulator.count;
    voxel.last_hit_frame = frame_index_;
    voxel.consecutive_free_frames = 0U;
    voxel.occupancy_score = std::min(
      config_.score_cap,
      voxel.occupancy_score + config_.occupied_gain *
      static_cast<float>(std::min<uint32_t>(3U, accumulator.count)));
    voxel.free_score = std::max(0.0F, voxel.free_score - config_.relief_gain);
  }
}

ProcessStatistics EvidenceMap::processFrame(
  const std::vector<Eigen::Vector3f> & visibility_points,
  const std::vector<Eigen::Vector3f> & integration_points,
  const Eigen::Isometry3f & current_from_previous,
  const std::vector<SensorModel> & sensors)
{
  if (sensors.empty()) {
    throw std::invalid_argument("at least one sensor model is required");
  }
  ++frame_index_;
  ProcessStatistics statistics;
  statistics.input_points = integration_points.size();
  statistics.map_voxels_before = voxels_.size();

  if (!voxels_.empty()) {
    transformAndReindex(current_from_previous);
    const auto visibility_maps = buildVisibilityMaps(visibility_points, sensors);
    applyVisibilityEvidence(visibility_maps, sensors, statistics);
  }
  // integrate() owns the accepted point count. Avoid double-counting input points.
  statistics.input_points = 0U;
  integrate(integration_points, statistics);
  statistics.map_voxels_after = voxels_.size();
  return statistics;
}

std::vector<EvidenceVoxel> EvidenceMap::publishedVoxels() const
{
  std::vector<EvidenceVoxel> result;
  result.reserve(voxels_.size());
  for (const auto & item : voxels_) {
    if (item.second.occupancy_score >= config_.publish_score &&
      insideRoi(item.second.position) && !insideBody(item.second.position))
    {
      result.push_back(item.second);
    }
  }
  return result;
}

}  // namespace m20_evidence_mapping
