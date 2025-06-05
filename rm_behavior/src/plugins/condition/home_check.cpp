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

#include "referee_interfaces/msg/basic_hp.hpp"
#include "rm_behavior/plugins/condition/home_check.hpp"

namespace rm_behavior {

HomeCheckCondition::HomeCheckCondition(const std::string &name,
                                           const BT::NodeConfig &config)
    : BT::ConditionNode(name, config), logger_(rclcpp::get_logger("HomeCheck")) {}

BT::NodeStatus HomeCheckCondition::tick() {
  int outpost_hp_limit = 0;
  int base_hp_limit = 0;
  auto msg = getInput<referee_interfaces::msg::BasicHp>("basic_hp");

  if (!msg) {
    RCLCPP_ERROR(logger_, "Basic HP message is not available");
    return BT::NodeStatus::FAILURE;
  }

  getInput("base_hp_limit", base_hp_limit);
  getInput("outpost_hp_limit", outpost_hp_limit);

  RCLCPP_DEBUG(logger_, "Checking base HP: %d vs limit: %d, outpost HP: %d vs limit: %d",
               msg->base_hp, base_hp_limit, msg->outpost_hp, outpost_hp_limit);

  // 检查基地和前哨站血量是否满足条件
  const bool is_base_hp_sufficient = (msg->base_hp > base_hp_limit);
  const bool is_outpost_hp_sufficient = (msg->outpost_hp > outpost_hp_limit);
  const bool is_home_safe = is_base_hp_sufficient && is_outpost_hp_sufficient;

  if (is_home_safe) {
    RCLCPP_DEBUG(logger_, "[HomeCheck] 状态良好: 基地血量=%d, 前哨站血量=%d -> SUCCESS",
                 msg->base_hp, msg->outpost_hp);
    return BT::NodeStatus::SUCCESS;
  } else {
    RCLCPP_DEBUG(logger_, "[HomeCheck] 状态不及预期: 基地血量=%d, 前哨站血量=%d -> FAILURE",
                 msg->base_hp, msg->outpost_hp);
    return BT::NodeStatus::FAILURE;
  }
}

BT::PortsList HomeCheckCondition::providedPorts() {
  return {
      BT::InputPort<referee_interfaces::msg::BasicHp>("basic_hp", "{@basic_hp}",
                                                      "哨兵基本状态信息"),
      BT::InputPort<int>("base_hp_limit", 1000, "基地血量阈值"),
      BT::InputPort<int>("outpost_hp_limit", 500, "前哨战血量阈值")};
}

} // namespace rm_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<rm_behavior::HomeCheckCondition>("HomeCheck");
}
