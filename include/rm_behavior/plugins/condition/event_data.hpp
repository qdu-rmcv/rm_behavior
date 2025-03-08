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

// event_data_check.hpp
#ifndef RM_BEHAVIOR__PLUGINS__CONDITION__EVENT_DATA_HPP_
#define RM_BEHAVIOR__PLUGINS__CONDITION__EVENT_DATA_HPP_

#include "auto_aim_interfaces/msg/referee.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <string>

namespace rm_behavior {

class eventDataCondition : public BT::ConditionNode {

public:

  eventDataCondition(const std::string &name, const BT::NodeConfig &config);

  static BT::PortsList providedPorts();

private:

  BT::NodeStatus EventDataCheck();

  rclcpp::Logger logger_ = rclcpp::get_logger("EventData");

};

} // namespace rm_behavior

#endif // RM_BEHAVIOR__PLUGINS__CONDITION__EVENT_DATA_HPP_