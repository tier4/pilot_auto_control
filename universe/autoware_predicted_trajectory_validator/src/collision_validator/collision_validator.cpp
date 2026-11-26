// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "autoware/predicted_trajectory_validator/collision_validator/collision_validator.hpp"

#include "autoware/predicted_trajectory_validator/utils.hpp"

#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>

#include <autoware_internal_control_msgs/msg/hazard_sector.hpp>
#include <autoware_perception_msgs/msg/object_classification.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::predicted_trajectory_validator
{
namespace
{
using autoware_internal_control_msgs::msg::HazardSector;
using collision_validator::CollisionCandidate;
using collision_validator::CollisionEvaluatorParam;
using collision_validator::PointCloudClusters;

std::optional<uint8_t> label_from_name(const std::string & name)
{
  using autoware_perception_msgs::msg::ObjectClassification;
  std::string upper(name);
  std::transform(upper.begin(), upper.end(), upper.begin(), ::toupper);
  if (upper == "UNKNOWN") return ObjectClassification::UNKNOWN;
  if (upper == "CAR") return ObjectClassification::CAR;
  if (upper == "TRUCK") return ObjectClassification::TRUCK;
  if (upper == "BUS") return ObjectClassification::BUS;
  if (upper == "TRAILER") return ObjectClassification::TRAILER;
  if (upper == "MOTORCYCLE") return ObjectClassification::MOTORCYCLE;
  if (upper == "BICYCLE") return ObjectClassification::BICYCLE;
  if (upper == "PEDESTRIAN") return ObjectClassification::PEDESTRIAN;
  if (upper == "ANIMAL") return ObjectClassification::ANIMAL;
  if (upper == "HAZARD") return ObjectClassification::HAZARD;
  return std::nullopt;
}

CollisionEvaluatorParam to_collision_evaluator_param(
  const ::predicted_trajectory_validator::Params::CollisionValidator & params)
{
  CollisionEvaluatorParam out;
  out.max_time = params.prediction_horizon;
  out.time_resolution = params.time_resolution;
  out.n_eval_object = static_cast<size_t>(std::max<int64_t>(params.max_evaluations.object, 0));
  out.n_eval_pointcloud =
    static_cast<size_t>(std::max<int64_t>(params.max_evaluations.pointcloud, 0));
  out.max_objects_stamp_gap_s = params.max_objects_stamp_gap_s;
  out.stop_velocity = params.stop_velocity;
  out.proximity_distance = params.proximity_distance;
  std::copy_n(params.braking.reaction_times.begin(), 3, out.reaction_times.begin());
  out.braking_deceleration = params.braking.deceleration;
  out.cos_threshold = params.collision_cos_threshold;
  out.rear_axle_ratio = params.vehicle_yaw.rear_axle_ratio;
  for (const auto & name : params.enabled_object_classes) {
    if (const auto label = label_from_name(name)) {
      out.enabled_labels.push_back(*label);
    }
  }
  return out;
}

uint8_t sources_of(const CollisionCandidate & candidate)
{
  uint8_t sources = 0;
  if (candidate.near) {
    sources |= HazardSector::SOURCE_PROXIMITY;
  }
  if (candidate.brake_level >= 1) {
    sources |= HazardSector::SOURCE_COLLISION;
  }
  return sources;
}
}  // namespace

CollisionValidator::CollisionValidator(
  const rclcpp::Logger & logger, const ::predicted_trajectory_validator::Params & params,
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper)
: logger_(logger),
  time_keeper_(time_keeper),
  params_(params.collision_validator),
  collision_evaluator_(to_collision_evaluator_param(params.collision_validator))
{
}

void CollisionValidator::update_parameters(const ::predicted_trajectory_validator::Params & params)
{
  params_ = params.collision_validator;
  collision_evaluator_.update_parameters(to_collision_evaluator_param(params_));
}

std::vector<HazardSource> CollisionValidator::to_hazard_sources(
  const std::vector<CollisionCandidate> & candidates, const nav_msgs::msg::Odometry & odom)
{
  std::vector<HazardSource> sources;
  for (const auto & candidate : candidates) {
    HazardSource source;
    source.shape = transform_polygon_to_base_link(candidate.polygon_now, odom.pose.pose);
    source.risk_level = candidate.brake_level;
    source.sources = sources_of(candidate);
    source.stamp = odom.header.stamp;
    sources.push_back(std::move(source));
  }
  return sources;
}

void CollisionValidator::validate(
  const autoware_planning_msgs::msg::Trajectory & predicted_trajectory,
  const nav_msgs::msg::Odometry & odom,
  const autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr & objects,
  const vehicle_info_utils::VehicleInfo & vehicle_info,
  const std::shared_ptr<const PointCloudClusters> & clusters, const rclcpp::Time & now,
  HazardWarningSources & hazard_sources)
{
  if (!params_.enable_warning) {
    // Disabled raises nothing; unset is only for a cycle that cannot be evaluated.
    hazard_sources.collision.emplace();
    return;
  }
  if (predicted_trajectory.points.size() < 2) {
    hazard_sources.collision.emplace();
    return;
  }
  const auto & points = predicted_trajectory.points;
  const bool is_time_sorted =
    std::is_sorted(points.begin(), points.end(), [](const auto & lhs, const auto & rhs) {
      return rclcpp::Duration(lhs.time_from_start) < rclcpp::Duration(rhs.time_from_start);
    });
  const bool is_position_finite = std::all_of(points.begin(), points.end(), [](const auto & point) {
    return std::isfinite(point.pose.position.x) && std::isfinite(point.pose.position.y);
  });
  if (!is_time_sorted || !is_position_finite) {
    RCLCPP_WARN_THROTTLE(
      logger_, steady_clock_, 5000, "invalid predicted trajectory. Collision evaluation skipped.");
    return;
  }

  std::shared_ptr<const PointCloudClusters> evaluated_clusters;
  if (params_.enable_pointcloud) {
    if (!clusters || (now - clusters->stamp).seconds() > params_.pointcloud.max_age_s) {
      RCLCPP_WARN_THROTTLE(
        logger_, steady_clock_, 5000,
        "no recent preprocessed pointcloud. Collision evaluation skipped.");
      return;
    }
    evaluated_clusters = clusters;
  } else if (!objects) {
    return;
  }

  std::vector<CollisionCandidate> candidates;
  try {
    autoware_utils_debug::ScopedTimeTrack st("collision_evaluator_evaluate", *time_keeper_);
    candidates = collision_evaluator_.evaluate(
      predicted_trajectory, odom, vehicle_info, objects, evaluated_clusters);
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(
      logger_, steady_clock_, 5000, "collision evaluation skipped: %s", e.what());
    return;
  }

  hazard_sources.collision = to_hazard_sources(candidates, odom);
}
}  // namespace autoware::predicted_trajectory_validator
