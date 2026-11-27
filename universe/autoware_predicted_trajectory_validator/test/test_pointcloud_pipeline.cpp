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

#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::predicted_trajectory_validator
{
using namespace test_utils;  // NOLINT(build/namespaces)

TEST(PointCloudPipelineTest, ProcessedFrameReplacesTheLatestAndRejectedFrameKeepsIt)
{
  auto pipeline = make_pipeline();
  const auto vehicle_info = test_vehicle_info();
  EXPECT_EQ(pipeline.latest(), nullptr);

  pipeline.process(make_cloud(at(10.0), 15.0), odometries_at({10.0}), vehicle_info);
  const auto processed = pipeline.latest();
  ASSERT_NE(processed, nullptr);
  EXPECT_EQ(processed->stamp, at(10.0));
  EXPECT_EQ(processed->clusters.size(), 1U);

  auto other_frame = make_cloud(at(10.05), 15.0);
  other_frame.header.frame_id = "lidar_top";
  pipeline.process(other_frame, {}, vehicle_info);
  pipeline.process(make_cloud(at(10.5), 15.0), {}, vehicle_info);
  EXPECT_EQ(pipeline.latest(), processed);
}

TEST(PointCloudPipelineTest, UnknownClassNameThrows)
{
  auto params = default_params();
  params.collision_validator.pointcloud.enabled_classes = {"structure", "tree"};
  EXPECT_THROW(make_pipeline(params), std::invalid_argument);
}

struct OdometryCase
{
  std::string name;
  std::vector<double> odometry_stamps_sec;
  double cloud_stamp_sec;
  bool processed;
};

class OdometrySelectionTest : public ::testing::TestWithParam<OdometryCase>
{
};

TEST_P(OdometrySelectionTest, FrameIsProcessedOnlyWithAnOdometryWithinMaxStampGap)
{
  auto pipeline = make_pipeline();
  pipeline.process(
    make_cloud(at(GetParam().cloud_stamp_sec), 15.0), odometries_at(GetParam().odometry_stamps_sec),
    test_vehicle_info());
  EXPECT_EQ(pipeline.latest() != nullptr, GetParam().processed);
}

INSTANTIATE_TEST_SUITE_P(
  PointCloudPipelineTest, OdometrySelectionTest,
  ::testing::Values(
    // max_stamp_gap_s = 0.1
    OdometryCase{"Close", {10.0}, 9.95, true}, OdometryCase{"Far", {10.0}, 9.8, false},
    // The history keeps the last 10 odometries: 10.0 is dropped by the 11th.
    OdometryCase{
      "OldestDropped",
      {10.0, 10.2, 10.4, 10.6, 10.8, 11.0, 11.2, 11.4, 11.6, 11.8, 12.0},
      9.95,
      false}),
  [](const ::testing::TestParamInfo<OdometryCase> & info) { return info.param.name; });
}  // namespace autoware::predicted_trajectory_validator
