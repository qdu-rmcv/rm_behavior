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

// hp_check.cpp
#include "rm_behavior/plugins/condition/hp_check.hpp"

namespace rm_behavior {

HPCheck::HPCheck(const std::string &name, const BT::NodeConfiguration &config)
    : BT::ConditionNode(name, config) {
  node_ = rclcpp::Node::make_shared("hp_check_node");
  subscription_ = node_->create_subscription<auto_aim_interfaces::msg::Referee>(
      "referee", 10,
      std::bind(&HPCheck::refereeCallback, this, std::placeholders::_1));
}

BT::PortsList HPCheck::providedPorts() { return {}; }

BT::NodeStatus HPCheck::tick() {
  rclcpp::spin_some(node_);
  // 这里可以根据 last_base_hp_ 和 last_outpost_hp_ 进行血量检查，示例中简单返回
  // SUCCESS
  return BT::NodeStatus::SUCCESS;
}

void HPCheck::refereeCallback(
    const auto_aim_interfaces::msg::Referee::SharedPtr msg) {
  last_base_hp_ = msg->base_hp;
  last_outpost_hp_ = msg->outpost_hp;
}

} // namespace rm_behavior