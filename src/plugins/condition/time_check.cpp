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

#include "rm_behavior/plugins/condition/time_check.hpp"

namespace rm_behavior
{

TimeCheckCondition::TimeCheckCondition(const std::string & name, const BT::NodeConfig & config)
: BT::ConditionNode(name, config)
{
}

BT::NodeStatus TimeCheckCondition::tick()
{
  int time = 0;

  if (!getInput("time", time)) {
    RCLCPP_ERROR(logger_, "Failed to get referee_time from blackboard");
    return BT::NodeStatus::FAILURE;
  }
  
  RCLCPP_INFO(logger_, "[TimeCheck] 时间检查: 裁判系统时间=%d", time);
  
  // 当时间为0时返回FAILURE
  if (time <= 0) {
    RCLCPP_WARN(logger_, "[TimeCheck] 时间已到: 裁判系统时间=%d -> 返回FAILURE", time);
    return BT::NodeStatus::FAILURE;
  }
  
  RCLCPP_INFO(logger_, "[TimeCheck] 时间充足: 裁判系统时间=%d -> 返回SUCCESS", time);
  return BT::NodeStatus::SUCCESS;
}

BT::PortsList TimeCheckCondition::providedPorts()
{
  return {
    BT::InputPort<int>("time", 
                    "{@referee_time}",
                      "裁判系统剩余时间")
  };
}

}  // namespace rm_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior::TimeCheckCondition>("TimeCheck");
}
