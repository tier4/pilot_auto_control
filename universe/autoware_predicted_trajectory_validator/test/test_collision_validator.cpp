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

#include "test_utils.hpp"

#include <gtest/gtest.h>

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::predicted_trajectory_validator
{
using namespace test_utils;  // NOLINT(build/namespaces)
using autoware_internal_control_msgs::msg::HazardSector;

struct ValidateCase
{
  std::string name;
  std::function<void(::predicted_trajectory_validator::Params &)> modify_params;
  std::function<void(Trajectory &)> modify_trajectory;
  bool with_objects;
  std::optional<double> cluster_age_s;  // nullopt gives no clusters
};

HazardWarningSources validate(const ValidateCase & c)
{
  const auto pose = make_pose(0.0, 0.0, 0.0);
  const auto now = at(10.0);
  auto params = default_params();
  c.modify_params(params);
  auto trajectory = make_trajectory(now, pose);
  c.modify_trajectory(trajectory);

  auto validator = make_validator(params);
  HazardWarningSources sources;
  validator.validate(
    trajectory, make_odom(now, pose),
    c.with_objects ? make_objects(now, {make_car(15.0, 0.0)}) : nullptr, test_vehicle_info(),
    c.cluster_age_s ? make_clusters(now - rclcpp::Duration::from_seconds(*c.cluster_age_s))
                    : nullptr,
    now, sources);
  return sources;
}

const auto kKeepParams = [](auto &) {};
const auto kKeepTrajectory = [](auto &) {};
const auto kShortTrajectory = [](auto & t) { t.points.resize(1); };

class CollisionSourcesEmptyTest : public ::testing::TestWithParam<ValidateCase>
{
};

TEST_P(CollisionSourcesEmptyTest, NothingRaisedGivesAnEmptyArray)
{
  const auto sources = validate(GetParam());
  ASSERT_TRUE(sources.collision.has_value());
  EXPECT_TRUE(sources.collision->empty());
}

INSTANTIATE_TEST_SUITE_P(
  CollisionValidatorTest, CollisionSourcesEmptyTest,
  ::testing::Values(
    ValidateCase{"NothingAhead", kKeepParams, kKeepTrajectory, false, 0.0},
    ValidateCase{
      "ShortTrajectoryWithoutInputs", kKeepParams, kShortTrajectory, false, std::nullopt},
    ValidateCase{
      "WarningDisabled", [](auto & p) { p.collision_validator.enable_warning = false; },
      kKeepTrajectory, false, std::nullopt}),
  [](const ::testing::TestParamInfo<ValidateCase> & info) { return info.param.name; });

class CollisionSourcesUnsetTest : public ::testing::TestWithParam<ValidateCase>
{
};

TEST_P(CollisionSourcesUnsetTest, CollisionSourcesAreLeftUnset)
{
  EXPECT_FALSE(validate(GetParam()).collision.has_value());
}

INSTANTIATE_TEST_SUITE_P(
  CollisionValidatorTest, CollisionSourcesUnsetTest,
  ::testing::Values(
    ValidateCase{
      "UnsortedTrajectory", kKeepParams,
      [](auto & t) { t.points[1].time_from_start = rclcpp::Duration::from_seconds(100.0); }, true,
      0.0},
    ValidateCase{"NoClusters", kKeepParams, kKeepTrajectory, true, std::nullopt},
    ValidateCase{"StaleClusters", kKeepParams, kKeepTrajectory, true, 0.301},
    ValidateCase{
      "PointcloudDisabledWithoutObjects",
      [](auto & p) { p.collision_validator.pointcloud.enable = false; }, kKeepTrajectory, false,
      0.0}),
  [](const ::testing::TestParamInfo<ValidateCase> & info) { return info.param.name; });

TEST(CollisionValidatorTest, ClustersUpToMaxAgeAreEvaluated)
{
  const auto sources = validate(ValidateCase{"", kKeepParams, kKeepTrajectory, false, 0.3});
  ASSERT_TRUE(sources.collision.has_value());
  EXPECT_TRUE(sources.collision->empty());
}

TEST(CollisionValidatorTest, EitherInputAloneIsEvaluated)
{
  const auto pose = make_pose(0.0, 0.0, 0.0);
  const auto now = at(10.0);
  const auto vehicle_info = test_vehicle_info();

  {
    auto params = default_params();
    params.collision_validator.pointcloud.enable = false;
    auto validator = make_validator(params);
    HazardWarningSources sources;
    validator.validate(
      make_trajectory(now, pose), make_odom(now, pose), make_objects(now, {make_car(15.0, 0.0)}),
      vehicle_info, make_clusters(now, {15.0}), now, sources);
    ASSERT_TRUE(sources.collision.has_value());
    EXPECT_EQ(sources.collision->size(), 1U);
    EXPECT_EQ(count_points(*sources.collision), 0U);
  }
  {
    auto validator = make_validator(default_params());
    HazardWarningSources sources;
    validator.validate(
      make_trajectory(now, pose), make_odom(now, pose), nullptr, vehicle_info,
      make_clusters(now, {15.0}), now, sources);
    ASSERT_TRUE(sources.collision.has_value());
    EXPECT_FALSE(sources.collision->empty());
    EXPECT_EQ(count_points(*sources.collision), sources.collision->size());
  }
}

TEST(CollisionValidatorTest, NearCandidatesAreHandedOverAsProximityWithoutALevel)
{
  auto validator = make_validator();
  const auto pose = make_pose(0.0, 0.0, 0.0);
  const auto now = at(10.0);

  HazardWarningSources sources;
  validator.validate(
    make_trajectory(now, pose), make_odom(now, pose),
    make_objects(now, {make_car(1.0, 1.648), make_car(1.0, -1.948)}), test_vehicle_info(),
    make_clusters(now), now, sources);
  ASSERT_TRUE(sources.collision.has_value());
  ASSERT_EQ(sources.collision->size(), 1U);
  EXPECT_EQ(sources.collision->front().risk_level, HazardSector::NONE);
  EXPECT_EQ(sources.collision->front().sources, HazardSector::SOURCE_PROXIMITY);
  for (const auto & vertex : sources.collision->front().shape.outer()) {
    EXPECT_GT(vertex.y(), 1.0);
  }
}
}  // namespace autoware::predicted_trajectory_validator
