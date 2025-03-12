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

#include "rm_behavior/plugins/condition/rfid_check.hpp"

namespace rm_behavior {

RFIDCheckCondition::RFIDCheckCondition(const std::string &name,
                                           const BT::NodeConfig &config)
    : BT::ConditionNode(name, config) {}

BT::NodeStatus RFIDCheckCondition::tick() {
    int rfid = 0;

    if (!getInput("rfid", rfid)) {
        RCLCPP_ERROR(logger_, "Failed to get rfid from blackboard");
        return BT::NodeStatus::FAILURE;
    }

    if ((rfid & (1 << 17)) || // 堡垒增益点
        (rfid & (1 << 23)) || // 中心增益点（仅 RMUL 适用）
        (rfid & (1 << 19)) || // 补给区（与克换区不重叠）
        (rfid & (1 << 20)))   // 补给区（与克换区重叠）
    {
      RCLCPP_INFO(logger_, "[RFIDCheck] 重要增益点已检测: rfid=0x%X",
                  rfid);
      return BT::NodeStatus::SUCCESS;
    } 
    else {
      RCLCPP_WARN(logger_, "[RFIDCheck] 重要增益点未检测到: rfid=0x%X",
                  rfid);
      return BT::NodeStatus::FAILURE;
    }
}

BT::PortsList RFIDCheckCondition::providedPorts() {
  return {BT::InputPort<int>("rfid", "{@referee_rfid}", "场地交互模块")};
}

} // namespace rm_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<rm_behavior::RFIDCheckCondition>("RFIDCheck");
}
