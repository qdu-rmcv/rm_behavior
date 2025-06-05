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
#include "referee_interfaces/msg/basic_hp.hpp"

namespace rm_behavior
{

StatusCheckCondition::StatusCheckCondition(const std::string & name, const BT::NodeConfig & config)
: BT::ConditionNode(name, config)
{
}

BT::NodeStatus StatusCheckCondition::tick()
{
  referee_interfaces::msg::BasicHp basic_hp;
  int sentry_hp_limit = 0;
  int projectile_allowance_limit = 0;

  if (!getInput("basic_hp", basic_hp)) {
    RCLCPP_ERROR(logger_, "Failed to get basic_hp from blackboard");
    return BT::NodeStatus::FAILURE;
  }

  if (!getInput("sentry_hp_limit", sentry_hp_limit)) {
    RCLCPP_ERROR(logger_, "Failed to get sentry_hp_limit from blackboard");
    return BT::NodeStatus::FAILURE;
  }

  if (!getInput("projectile_allowance_limit", projectile_allowance_limit)) {
    RCLCPP_ERROR(logger_, "Failed to get projectile_allowance_limit from blackboard");
    return BT::NodeStatus::FAILURE;
  }
  
  RCLCPP_INFO(logger_, "[StatusCheck] 状态检查: 哨兵血量=%d, 弹药量=%d", 
              basic_hp.sentry_hp, basic_hp.projectile_allowance_17mm);

  if (basic_hp.sentry_hp <= sentry_hp_limit || 
      basic_hp.projectile_allowance_17mm <= projectile_allowance_limit) {
    RCLCPP_WARN(logger_,"[StatusCheck] 状态差:血量=%d, 弹药=%d -> 返回FAILURE",
    basic_hp.sentry_hp, basic_hp.projectile_allowance_17mm);
    return BT::NodeStatus::FAILURE;
  }
  
  RCLCPP_INFO(logger_, "[StatusCheck] 状态良好: 血量=%d, 弹药=%d -> 返回SUCCESS", 
              basic_hp.sentry_hp, basic_hp.projectile_allowance_17mm);
  return BT::NodeStatus::SUCCESS;
}

BT::PortsList StatusCheckCondition::providedPorts()
{
  return {
    BT::InputPort<referee_interfaces::msg::BasicHp>("basic_hp", "{@basic_hp}", 
                      "哨兵基本状态信息"),
    BT::InputPort<int>("sentry_hp_limit","{@sentry_hp_limit}","哨兵血量阈值"),
    BT::InputPort<int>("projectile_allowance_limit","{@projectile_allowance_limit}",
                      "允许发弹量阈值")
  };
}

}  // namespace rm_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior::StatusCheckCondition>("StatusCheck");
}
