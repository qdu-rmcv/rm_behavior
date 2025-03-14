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

#include "rm_behavior/plugins/condition/status_check.hpp"

namespace rm_behavior
{

StatusCheckCondition::StatusCheckCondition(const std::string & name, const BT::NodeConfig & config)
: BT::ConditionNode(name, config)
{
}

BT::NodeStatus StatusCheckCondition::tick()
{
  int sentry_hp = 0;
  int projectile_allowance = 0;

  if (!getInput("sentry_hp", sentry_hp)) {
    RCLCPP_ERROR(logger_, "Failed to get sentry_hp from blackboard");
    return BT::NodeStatus::FAILURE;
  }
  
  if (!getInput("projectile_allowance", projectile_allowance)) {
    RCLCPP_ERROR(logger_, "Failed to get projectile_allowance from blackboard");
    return BT::NodeStatus::FAILURE;
  }
  
  RCLCPP_INFO(logger_, "[StatusCheck] 状态检查: 哨兵血量=%d, 弹药量=%d", 
              sentry_hp, projectile_allowance);
  
  // 当哨兵HP和弹药量均小于等于100时返回FAILURE
  if (sentry_hp <= 100 || projectile_allowance <= 0) {
    RCLCPP_WARN(logger_, "[StatusCheck] 状态异常: 血量=%d, 弹药=%d -> 返回FAILURE", 
                sentry_hp, projectile_allowance);
    return BT::NodeStatus::FAILURE;
  }
  
  RCLCPP_INFO(logger_, "[StatusCheck] 状态良好: 血量=%d, 弹药=%d -> 返回SUCCESS", 
              sentry_hp, projectile_allowance);
  return BT::NodeStatus::SUCCESS;
}

BT::PortsList StatusCheckCondition::providedPorts()
{
  return {
    BT::InputPort<int>("sentry_hp", "{@referee_sentry_hp}", 
                      "哨兵生命值"),
    BT::InputPort<int>("projectile_allowance", "{@referee_projectile_allowance}", 
                      "裁判系统允许发单量")
  };
}

}  // namespace rm_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior::StatusCheckCondition>("StatusCheck");
}
