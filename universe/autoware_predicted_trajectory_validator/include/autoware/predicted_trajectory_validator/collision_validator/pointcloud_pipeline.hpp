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

#ifndef AUTOWARE__PREDICTED_TRAJECTORY_VALIDATOR__COLLISION_VALIDATOR__POINTCLOUD_PIPELINE_HPP_
#define AUTOWARE__PREDICTED_TRAJECTORY_VALIDATOR__COLLISION_VALIDATOR__POINTCLOUD_PIPELINE_HPP_

#include "autoware/predicted_trajectory_validator/collision_validator/pointcloud_preprocessor.hpp"

#include <autoware_predicted_trajectory_validator/predicted_trajectory_validator_parameters.hpp>
#include <autoware_utils_debug/time_keeper.hpp>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/time.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cstddef>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <vector>

namespace autoware::predicted_trajectory_validator
{
/**
 * @brief Turns pointclouds into clusters in the pointcloud callback group: odometry history,
 * preprocessing, and the hand-over of the latest clusters to the cycle.
 */
class PointCloudPipeline
{
public:
  PointCloudPipeline(
    const rclcpp::Logger & logger, const ::predicted_trajectory_validator::Params & params,
    std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper =
      std::make_shared<autoware_utils_debug::TimeKeeper>());

  void update_parameters(const ::predicted_trajectory_validator::Params & params);

  /**
   * @brief Add the odometries to the history and process the pointcloud with the odometry closest
   * to its stamp. The latest clusters are replaced only when the frame is processed.
   * @param pointcloud pointcloud in base_link
   * @param odometries odometries received since the last call
   * @param vehicle_info height range and footprint of the ego vehicle
   */
  void process(
    const sensor_msgs::msg::PointCloud2 & pointcloud,
    const std::vector<std::shared_ptr<const nav_msgs::msg::Odometry>> & odometries,
    const vehicle_info_utils::VehicleInfo & vehicle_info);

  /**
   * @brief Clusters of the last processed frame; null when no frame has been processed.
   */
  std::shared_ptr<const collision_validator::PointCloudClusters> latest() const;

private:
  void update_history(
    const std::vector<std::shared_ptr<const nav_msgs::msg::Odometry>> & odometries);
  std::optional<nav_msgs::msg::Odometry> select_odometry_at(const rclcpp::Time & stamp) const;

  static constexpr size_t kOdometryHistorySize = 10;

  rclcpp::Logger logger_;
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  double max_stamp_gap_s_;
  std::deque<nav_msgs::msg::Odometry> odometry_history_;
  collision_validator::PointCloudPreprocessor preprocessor_;
  mutable std::mutex mutex_;
  std::shared_ptr<const collision_validator::PointCloudClusters> latest_;
};
}  // namespace autoware::predicted_trajectory_validator

#endif  // AUTOWARE__PREDICTED_TRAJECTORY_VALIDATOR__COLLISION_VALIDATOR__POINTCLOUD_PIPELINE_HPP_
