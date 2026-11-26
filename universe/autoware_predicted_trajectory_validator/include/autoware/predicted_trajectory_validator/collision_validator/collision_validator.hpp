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

#ifndef AUTOWARE__PREDICTED_TRAJECTORY_VALIDATOR__COLLISION_VALIDATOR__COLLISION_VALIDATOR_HPP_
#define AUTOWARE__PREDICTED_TRAJECTORY_VALIDATOR__COLLISION_VALIDATOR__COLLISION_VALIDATOR_HPP_

#include "autoware/predicted_trajectory_validator/collision_validator/collision_evaluator.hpp"
#include "autoware/predicted_trajectory_validator/collision_validator/pointcloud_preprocessor.hpp"
#include "autoware/predicted_trajectory_validator/hazard_warning/hazard_warning_sources.hpp"

#include <autoware_predicted_trajectory_validator/predicted_trajectory_validator_parameters.hpp>
#include <autoware_utils_debug/time_keeper.hpp>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/time.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <memory>
#include <vector>

namespace autoware::predicted_trajectory_validator
{
/**
 * @brief Entry of the collision warning: freshness of the pointcloud clusters, collision
 * evaluation, and the hand-over of the candidates to raise to HazardWarning.
 */
class CollisionValidator
{
public:
  CollisionValidator(
    const rclcpp::Logger & logger, const ::predicted_trajectory_validator::Params & params,
    std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper =
      std::make_shared<autoware_utils_debug::TimeKeeper>());

  void update_parameters(const ::predicted_trajectory_validator::Params & params);

  /**
   * @brief Evaluate one cycle and write hazard_sources.collision: an empty vector when nothing is
   * raised, the warning is disabled or the predicted trajectory has less than 2 points.
   * hazard_sources.collision is left unset when the cycle cannot be evaluated.
   * @param objects latest objects; may be null
   * @param clusters latest pointcloud clusters; may be null
   * @param now current time, compared with the cluster stamp
   */
  void validate(
    const autoware_planning_msgs::msg::Trajectory & predicted_trajectory,
    const nav_msgs::msg::Odometry & odom,
    const autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr & objects,
    const vehicle_info_utils::VehicleInfo & vehicle_info,
    const std::shared_ptr<const collision_validator::PointCloudClusters> & clusters,
    const rclcpp::Time & now, HazardWarningSources & hazard_sources);

private:
  static std::vector<HazardSource> to_hazard_sources(
    const std::vector<collision_validator::CollisionCandidate> & candidates,
    const nav_msgs::msg::Odometry & odom);

  rclcpp::Logger logger_;
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_;
  ::predicted_trajectory_validator::Params::CollisionValidator params_;
  collision_validator::CollisionEvaluator collision_evaluator_;
};
}  // namespace autoware::predicted_trajectory_validator

#endif  // AUTOWARE__PREDICTED_TRAJECTORY_VALIDATOR__COLLISION_VALIDATOR__COLLISION_VALIDATOR_HPP_
