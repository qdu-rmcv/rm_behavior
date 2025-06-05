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
  // 获取RFID消息
  auto rfid_msg = getInput<referee_interfaces::msg::Rfid>("rfid_msg");
  if (!rfid_msg) {
    RCLCPP_ERROR(rclcpp::get_logger("RFIDCheckCondition"), "RFID message is not available");
    return BT::NodeStatus::FAILURE;
  }

  // 获取要检查的RFID类型
  std::string rfid_chosen;
  if (!getInput("rfid_chosen", rfid_chosen)) {
    RCLCPP_ERROR(rclcpp::get_logger("RFIDCheckCondition"), "rfid_chosen parameter is required");
    return BT::NodeStatus::FAILURE;
  }

  // 根据指定的RFID类型进行检查
  if (rfid_chosen == "base_gain_point" && rfid_msg->base_gain_point) {
    return BT::NodeStatus::SUCCESS; //己方基地增益
  } 
  else if (rfid_chosen == "central_highland_gain_point" && rfid_msg->central_highland_gain_point) {
    return BT::NodeStatus::SUCCESS; //己方中央高地
  } 
  else if (rfid_chosen == "enemy_central_highland_gain_point" && rfid_msg->enemy_central_highland_gain_point) {
    return BT::NodeStatus::SUCCESS; //对方中央高地增益
  } 
  else if (rfid_chosen == "friendly_trapezoidal_highland_gain_point" && rfid_msg->friendly_trapezoidal_highland_gain_point) {
    return BT::NodeStatus::SUCCESS; //己方梯形高地
  } 
  else if (rfid_chosen == "enemy_trapezoidal_highland_gain_point" && rfid_msg->enemy_trapezoidal_highland_gain_point) {
    return BT::NodeStatus::SUCCESS; //对方梯形高地
  } 
  else if (rfid_chosen == "friendly_fly_ramp_front_gain_point" && rfid_msg->friendly_fly_ramp_front_gain_point) {
    return BT::NodeStatus::SUCCESS; //己方飞坡前
  } 
  else if (rfid_chosen == "friendly_fly_ramp_back_gain_point" && rfid_msg->friendly_fly_ramp_back_gain_point) {
    return BT::NodeStatus::SUCCESS; //己方飞坡后
  } 
  else if (rfid_chosen == "enemy_fly_ramp_front_gain_point" && rfid_msg->enemy_fly_ramp_front_gain_point) {
    return BT::NodeStatus::SUCCESS; //对方飞坡前
  } 
  else if (rfid_chosen == "enemy_fly_ramp_back_gain_point" && rfid_msg->enemy_fly_ramp_back_gain_point) {
    return BT::NodeStatus::SUCCESS; //对方飞坡后
  } 
  else if (rfid_chosen == "friendly_central_highland_lower_gain_point" && rfid_msg->friendly_central_highland_lower_gain_point) {
    return BT::NodeStatus::SUCCESS; //己方中央高地下
  } 
  else if (rfid_chosen == "friendly_central_highland_upper_gain_point" && rfid_msg->friendly_central_highland_upper_gain_point) {
    return BT::NodeStatus::SUCCESS; //己方中央高地上
  } 
  else if (rfid_chosen == "enemy_central_highland_lower_gain_point" && rfid_msg->enemy_central_highland_lower_gain_point) {
    return BT::NodeStatus::SUCCESS; //对方中央高地下
  }
  else if (rfid_chosen == "enemy_central_highland_upper_gain_point" && rfid_msg->enemy_central_highland_upper_gain_point) {
    return BT::NodeStatus::SUCCESS; //对方中央高地上
  } 
  else if (rfid_chosen == "friendly_highway_lower_gain_point" && rfid_msg->friendly_highway_lower_gain_point) {
    return BT::NodeStatus::SUCCESS; //己方公路区下
  } 
  else if (rfid_chosen == "friendly_highway_upper_gain_point" && rfid_msg->friendly_highway_upper_gain_point) {
    return BT::NodeStatus::SUCCESS; //己方公路区上
  } 
  else if (rfid_chosen == "enemy_highway_lower_gain_point" && rfid_msg->enemy_highway_lower_gain_point) {
    return BT::NodeStatus::SUCCESS; //对方公路区下
  } 
  else if (rfid_chosen == "enemy_highway_upper_gain_point" && rfid_msg->enemy_highway_upper_gain_point) {
    return BT::NodeStatus::SUCCESS; //对方公路区上
  } 
  else if (rfid_chosen == "friendly_fortress_gain_point" && rfid_msg->friendly_fortress_gain_point) {
    return BT::NodeStatus::SUCCESS; //己方堡垒增益
  } 
  else if (rfid_chosen == "friendly_outpost_gain_point" && rfid_msg->friendly_outpost_gain_point) {
    return BT::NodeStatus::SUCCESS; //己方前哨站增益
  } 
  else if (rfid_chosen == "friendly_supply_zone" && 
           (rfid_msg->friendly_supply_zone_non_exchange || rfid_msg->friendly_supply_zone_exchange)) {
    // 合并补给区检测
    return BT::NodeStatus::SUCCESS;
  } 
  else if (rfid_chosen == "friendly_big_resource_island" && rfid_msg->friendly_big_resource_island) {
    return BT::NodeStatus::SUCCESS; //己方大资源岛
  } 
  else if (rfid_chosen == "enemy_big_resource_island" && rfid_msg->enemy_big_resource_island) {
    return BT::NodeStatus::SUCCESS; //对方大资源岛
  } 
  else if (rfid_chosen == "center_gain_point" && rfid_msg->center_gain_point) {
    return BT::NodeStatus::SUCCESS; //中心增益点(UL)
  }

  return BT::NodeStatus::FAILURE;
}

BT::PortsList RFIDCheckCondition::providedPorts() {
  return {
    BT::InputPort<referee_interfaces::msg::Rfid>(
      "rfid_msg", "{@referee_rfid}", "RFID message from referee system"),
    BT::InputPort<std::string>(
      "rfid_chosen", "", "The name of the RFID point to check")
  };
}

} // namespace rm_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<rm_behavior::RFIDCheckCondition>("RFIDCheck");
}
