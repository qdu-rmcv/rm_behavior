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

#include "rm_behavior/plugins/condition/rfid.hpp"

namespace rm_behavior {

rfid::rfid(const std::string &name, const BT::NodeConfiguration &config)
    : BT::ConditionNode(name, config) {}

BT::PortsList rfid::providedPorts() {
  return {BT::InputPort<int>("rfid"),
          BT::OutputPort<bool>("center_buff_detected"),
          BT::OutputPort<bool>("supply_zone_detected")};
}

BT::NodeStatus rfid::tick() {
  int rfid_value;
  bool center_buff_detected = false;
  bool supply_zone_detected = false;

  if (!getInput("rfid", rfid_value)) {
    RCLCPP_ERROR(rclcpp::get_logger("rfid"), "Failed to get input port");
    return BT::NodeStatus::FAILURE;
  }

  // 简单示例：根据 rfid 值判断是否检测到中央 buff 或补给区
  if (rfid_value == 1) {
    center_buff_detected = true;
  } else if (rfid_value == 2) {
    supply_zone_detected = true;
  }

  setOutput("center_buff_detected", center_buff_detected);
  setOutput("supply_zone_detected", supply_zone_detected);

  return BT::NodeStatus::SUCCESS;
}

} // namespace rm_behavior