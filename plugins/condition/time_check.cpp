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

//time_check.cpp

#include "rm_behavior/plugins/condition/time_check.hpp"

namespace rm_behavior
{
  
timeCheckCondition::timeCheckCondition(const std::string &name, const BT::NodeConfig &config)
   : BT::ConditionNode(name,std::bind(&timeCheckCondition::timeCheck,config) {}

BT::NodeStatus timeCheckCondition::tick()
{
  auto referee = getInput<auto_aim_interfaces::msg::Referee>("referee");
  if (!referee)
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to get referee input");
    return BT::NodeStatus::FAILURE;
  }

  // Specifically check the stage_time_left field from the referee message
  if (referee->stage_time_left.sec < 60)
  {
    RCLCPP_INFO(node_->get_logger(), "Time check passed: %d seconds left", referee->stage_time_left.sec);
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    RCLCPP_DEBUG(node_->get_logger(), "Time check failed: %d seconds left", referee->stage_time_left.sec);
    return BT::NodeStatus::FAILURE;
  }
}
}