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

// time_check.cpp
#include "rm_behavior/plugins/condition/time_check.hpp"

namespace rm_behavior {

TimeCheck::TimeCheck(const std::string &name,
                     const BT::NodeConfiguration &config)
    : BT::ConditionNode(name, config) {
  node_ = rclcpp::Node::make_shared("time_check_node");
  subscription_ = node_->create_subscription<auto_aim_interfaces::msg::Referee>(
      "referee", 10,
      std::bind(&TimeCheck::refereeCallback, this, std::placeholders::_1));
}

BT::PortsList TimeCheck::providedPorts() { return {}; }

BT::NodeStatus TimeCheck::tick() {
  rclcpp::spin_some(node_);
  // 这里可以根据 last_time_ 进行时间检查，示例中简单返回 SUCCESS
  return BT::NodeStatus::SUCCESS;
}

void TimeCheck::refereeCallback(
    const auto_aim_interfaces::msg::Referee::SharedPtr msg) {
  last_time_ = msg->time;
}

} // namespace rm_behavior