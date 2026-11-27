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

#ifndef AUTOWARE__PREDICTED_TRAJECTORY_VALIDATOR__COLLISION_VALIDATOR__POINTCLOUD_PREPROCESSOR_HPP_
#define AUTOWARE__PREDICTED_TRAJECTORY_VALIDATOR__COLLISION_VALIDATOR__POINTCLOUD_PREPROCESSOR_HPP_

#include <autoware/point_types/types.hpp>
#include <autoware_utils_debug/time_keeper.hpp>
#include <autoware_utils_geometry/boost_geometry.hpp>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/time.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <bitset>
#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::predicted_trajectory_validator::collision_validator
{
using autoware::point_types::PointCloudClassification;
using autoware_utils_geometry::Point2d;
using autoware_utils_geometry::Polygon2d;

/// Number of class_id values (uint8), the size of the tables indexed by class_id.
constexpr size_t kNumPointCloudClassIds = 256;

struct PointCloudParam
{
  double roi_rear{2.0};
  double roi_front_margin{2.0};
  double roi_lateral{5.0};
  double z_margin_low{0.0};
  double z_margin_high{0.0};
  double horizon{5.0};
  size_t max_voxels{20000};
  double voxel_size{0.10};
  double cluster_tolerance{0.30};
  size_t min_cluster_size{5};
  std::bitset<kNumPointCloudClassIds> target_class_mask;
};

/// Points of one euclidean cluster of one class and their centroid, in the map frame.
struct PointCluster
{
  PointCloudClassification classification{PointCloudClassification::INVALID};
  std::vector<Point2d> points;
  Point2d centroid;
  size_t raw_point_count{0};
};

struct PointCloudClusters
{
  std::vector<PointCluster> clusters;
  rclcpp::Time stamp;
};

/**
 * @brief Class name (lowercase PointCloudClassification name) to the class.
 * @return nullopt for any other name, including names with uppercase letters
 */
std::optional<PointCloudClassification> pointcloud_class_from_name(const std::string & name);

/**
 * @brief Ego-footprint exclusion -> ROI crop and class selection -> voxel grid per class (raw
 * point counts) -> subsample -> euclidean clustering per class, keeping only the points that
 * belong to a cluster. Points are expressed in the map frame through the ego pose at the
 * pointcloud stamp.
 */
class PointCloudPreprocessor
{
public:
  PointCloudPreprocessor(
    const rclcpp::Logger & logger, const PointCloudParam & param,
    std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper =
      std::make_shared<autoware_utils_debug::TimeKeeper>());

  void update_parameters(const PointCloudParam & param);

  /**
   * @brief Process one frame into clusters.
   * @param pointcloud pointcloud in base_link
   * @param odom ego odometry (map) at the pointcloud stamp
   * @param vehicle_info height range and footprint of the ego vehicle
   * @return clusters of the frame. An empty cloud or a cloud without a class_id field gives no
   *   clusters. A frame other than base_link, a class_id other than UINT8, an unsupported layout
   *   or a frame that fails to be processed gives nullopt.
   */
  std::optional<PointCloudClusters> process(
    const sensor_msgs::msg::PointCloud2 & pointcloud, const nav_msgs::msg::Odometry & odom,
    const vehicle_info_utils::VehicleInfo & vehicle_info);

  static constexpr const char * kExpectedFrame = "base_link";

private:
  std::optional<PointCloudClusters> process_frame(
    const sensor_msgs::msg::PointCloud2 & pointcloud, const nav_msgs::msg::Odometry & odom,
    const vehicle_info_utils::VehicleInfo & vehicle_info);

  double z_min(const vehicle_info_utils::VehicleInfo & vehicle_info) const;
  double z_max(const vehicle_info_utils::VehicleInfo & vehicle_info) const;

  rclcpp::Logger logger_;
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_;
  PointCloudParam param_;
};
}  // namespace autoware::predicted_trajectory_validator::collision_validator

// clang-format off
#endif  // AUTOWARE__PREDICTED_TRAJECTORY_VALIDATOR__COLLISION_VALIDATOR__POINTCLOUD_PREPROCESSOR_HPP_  // NOLINT
// clang-format on
