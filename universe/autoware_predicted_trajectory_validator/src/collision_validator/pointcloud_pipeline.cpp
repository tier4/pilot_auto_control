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

#include "autoware/predicted_trajectory_validator/collision_validator/pointcloud_pipeline.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

namespace autoware::predicted_trajectory_validator
{
namespace
{
using collision_validator::PointCloudClusters;
using collision_validator::PointCloudParam;

PointCloudParam to_pointcloud_param(
  const ::predicted_trajectory_validator::Params::CollisionValidator & params)
{
  const auto & pointcloud = params.pointcloud;
  PointCloudParam out;
  out.roi_rear = pointcloud.roi.rear;
  out.roi_front_margin = pointcloud.roi.front_margin;
  out.roi_lateral = pointcloud.roi.lateral;
  out.z_margin_low = pointcloud.roi.z_margin_low;
  out.z_margin_high = pointcloud.roi.z_margin_high;
  out.horizon = params.prediction_horizon;
  out.max_voxels = static_cast<size_t>(std::max<int64_t>(pointcloud.max_voxels, 0));
  out.voxel_size = pointcloud.voxel_size;
  out.cluster_tolerance = pointcloud.cluster_tolerance;
  out.min_cluster_size = static_cast<size_t>(std::max<int64_t>(pointcloud.min_cluster_size, 0));
  return out;
}
}  // namespace

PointCloudPipeline::PointCloudPipeline(
  const rclcpp::Logger & logger, const ::predicted_trajectory_validator::Params & params,
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper)
: logger_(logger),
  max_stamp_gap_s_(params.collision_validator.pointcloud.max_stamp_gap_s),
  preprocessor_(logger, to_pointcloud_param(params.collision_validator), std::move(time_keeper))
{
}

void PointCloudPipeline::update_parameters(const ::predicted_trajectory_validator::Params & params)
{
  max_stamp_gap_s_ = params.collision_validator.pointcloud.max_stamp_gap_s;
  preprocessor_.update_parameters(to_pointcloud_param(params.collision_validator));
}

void PointCloudPipeline::update_history(
  const std::vector<std::shared_ptr<const nav_msgs::msg::Odometry>> & odometries)
{
  for (const auto & odom : odometries) {
    odometry_history_.push_back(*odom);
    while (odometry_history_.size() > kOdometryHistorySize) {
      odometry_history_.pop_front();
    }
  }
}

std::optional<nav_msgs::msg::Odometry> PointCloudPipeline::select_odometry_at(
  const rclcpp::Time & stamp) const
{
  std::optional<nav_msgs::msg::Odometry> best;
  double best_gap = std::numeric_limits<double>::infinity();
  for (const auto & odom : odometry_history_) {
    const double gap = std::abs((rclcpp::Time(odom.header.stamp) - stamp).seconds());
    if (gap < best_gap && gap <= max_stamp_gap_s_) {
      best_gap = gap;
      best = odom;
    }
  }
  return best;
}

void PointCloudPipeline::process(
  const sensor_msgs::msg::PointCloud2 & pointcloud,
  const std::vector<std::shared_ptr<const nav_msgs::msg::Odometry>> & odometries,
  const vehicle_info_utils::VehicleInfo & vehicle_info)
{
  update_history(odometries);
  const auto odom = select_odometry_at(rclcpp::Time(pointcloud.header.stamp));
  if (!odom) {
    RCLCPP_WARN_THROTTLE(
      logger_, steady_clock_, 5000, "no odometry close to the pointcloud stamp. Frame dropped.");
    return;
  }
  auto clusters = preprocessor_.process(pointcloud, *odom, vehicle_info);
  if (!clusters) {
    return;
  }
  auto processed = std::make_shared<const PointCloudClusters>(std::move(*clusters));
  std::lock_guard<std::mutex> lock(mutex_);
  latest_ = std::move(processed);
}

std::shared_ptr<const PointCloudClusters> PointCloudPipeline::latest() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return latest_;
}
}  // namespace autoware::predicted_trajectory_validator
