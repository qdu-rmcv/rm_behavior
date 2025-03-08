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

// event_data_check.cpp
#include "rm_behavior/plugins/condition/event_data.hpp"

namespace rm_behavior {

EventDataCheck::EventDataCheck(const std::string &name,
                               const BT::NodeConfiguration &config)
    : BT::ConditionNode(name, config) {
  node_ = rclcpp::Node::make_shared("event_data_node");
  subscription_ = node_->create_subscription<auto_aim_interfaces::msg::Referee>(
      "referee", 10,
      std::bind(&EventDataCheck::refereeCallback, this, std::placeholders::_1));
}

BT::PortsList EventDataCheck::providedPorts() { return {}; }

BT::NodeStatus EventDataCheck::tick() {
  rclcpp::spin_some(node_);
  // 这里可以根据 last_event_data_ 进行事件数据检查，示例中简单返回 SUCCESS
  return BT::NodeStatus::SUCCESS;
}

void EventDataCheck::refereeCallback(
    const auto_aim_interfaces::msg::Referee::SharedPtr msg) {
  last_event_data_ = msg->event_data;
}

} // namespace rm_behavior