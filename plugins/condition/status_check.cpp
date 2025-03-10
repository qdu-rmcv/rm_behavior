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

//status_check.cpp

#include "rm_behavior/plugins/condition/status_check.hpp"

namespace rm_behavior
{

statusCheckCondition::statusCheckCondition(const std::string &name, const BT::NodeConfig &config)
: BT::SimpleConditionNode(name, std::bind(&statusCheckCondition::statusCheck, this), config)
{
}

BT::NodeStatus statusCheckCondition::statusCheck()
{
    uint16_t sentry_hp;
    uint16_t projectile_allowance_17mm;  // 注意这里变量名保持了一致
    uint16_t hp_min;
    uint16_t ammo_min;

    if (!getInput("sentry_hp", sentry_hp) || 
        !getInput("projectile_allowance_17mm", projectile_allowance_17mm))
    {
        RCLCPP_ERROR(rclcpp::get_logger("status_check"), "Missing required input ports");
        return BT::NodeStatus::FAILURE;
    }

    // Get the threshold values (with defaults if not specified)
    getInput("hp_min", hp_min);
    getInput("ammo_min", ammo_min);

    const bool is_hp_ok = (sentry_hp >= hp_min);
    const bool is_ammo_ok = (projectile_allowance_17mm >= ammo_min);  // 修正了变量名

    return (is_hp_ok && is_ammo_ok) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList statusCheckCondition::providedPorts()
{
  return {
      BT::InputPort<uint16_t>("sentry_hp"),
      BT::InputPort<uint16_t>("hp_min", 150,"Minimum HP. NOTE: Sentry init/max HP is 400"),
      BT::InputPort<uint16_t>("projectile_allowance_17mm"),
      BT::InputPort<uint16_t>("ammo_min", 10, "Minimum ammunition threshold")};
}

}  // namespace rm_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<rm_behavior::statusCheckCondition>("StatusCheck");
}