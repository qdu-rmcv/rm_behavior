// Copyright 2025 Jquark
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

//status_check.cpp

#include "rm_behavior/plugins/condition/status_check.hpp"

namespace rm_behavior
{

statusCheckCondition::statusCheckCondition(const std::string &name, const BT::NodeConfig &config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList statusCheckCondition::providedPorts()
{
  return { BT::InputPort<uint32_t>("sentry_hp"), BT::InputPort<uint32_t>("projectile_allowance_17mm") };
}

BT::NodeStatus statusCheckCondition::statusCheck()
{
  uint32_t sentry_hp;
  uint32_t projectile_allowance;

  if (!getInput("sentry_hp", sentry_hp) || !getInput("projectile_allowance_17mm", projectile_allowance))
  {
    RCLCPP_ERROR(logger_, "Missing required input ports");
    return BT::NodeStatus::FAILURE;
  }

  if (sentry_hp < 150 || projectile_allowance <= 10)
  {
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior