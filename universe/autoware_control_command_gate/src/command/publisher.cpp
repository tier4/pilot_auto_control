// Copyright 2025 The Autoware Contributors
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

#include "publisher.hpp"

#include <autoware/component_interface_specs/control.hpp>
#include <autoware/component_interface_specs/utils.hpp>

#include <memory>

namespace autoware::control_command_gate
{

CommandPublisher::CommandPublisher(autoware::agnocast_wrapper::Node & node)
{
  namespace control_spec = autoware::component_interface_specs::control;
  using autoware::component_interface_specs::get_qos;

  pub_control_ =
    node.create_publisher<Control>("~/output/control", get_qos<control_spec::ControlCommand>());
  pub_gear_ =
    node.create_publisher<GearCommand>("~/output/gear", get_qos<control_spec::GearCommand>());
  pub_turn_indicators_ = node.create_publisher<TurnIndicatorsCommand>(
    "~/output/turn_indicators", get_qos<control_spec::TurnIndicatorsCommand>());
  pub_hazard_lights_ = node.create_publisher<HazardLightsCommand>(
    "~/output/hazard_lights", get_qos<control_spec::HazardLightsCommand>());
}

void CommandPublisher::on_control(uint16_t, const Control & msg)
{
  *prev_control_ = msg;

  timeout_->update();
  pub_control_->publish(msg);
}

void CommandPublisher::on_gear(const GearCommand & msg)
{
  pub_gear_->publish(msg);
}

void CommandPublisher::on_turn_indicators(const TurnIndicatorsCommand & msg)
{
  pub_turn_indicators_->publish(msg);
}

void CommandPublisher::on_hazard_lights(const HazardLightsCommand & msg)
{
  pub_hazard_lights_->publish(msg);
}

TimeoutDiag * CommandPublisher::create_diag_task(
  const TimeoutDiag::Params & params, const rclcpp::Clock & clock)
{
  if (timeout_) {
    throw std::logic_error("timeout diag has already been created");
  }
  timeout_ = std::make_unique<TimeoutDiag>(params, clock, "timeout_output");
  return timeout_.get();
}

}  // namespace autoware::control_command_gate
