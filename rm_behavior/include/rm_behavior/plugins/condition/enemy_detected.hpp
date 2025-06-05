// Copyright 2025 Jquark
// Copyright 2025 Lihan Chen
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

#ifndef RM_BEHAVIOR__PLUGINS__CONDITION__ENEMY_DETECTED_HPP_
#define RM_BEHAVIOR__PLUGINS__CONDITION__ENEMY_DETECTED_HPP_


#include "auto_aim_interfaces/msg/armors.hpp"
#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"

namespace rm_behavior {

class EnemyDetectedCondition : public BT::ConditionNode {

public:

  EnemyDetectedCondition(const std::string &name, const BT::NodeConfig &config);
  
  static BT::PortsList providedPorts();

private:

  BT::NodeStatus tick() override;

  rclcpp::Logger logger_ = rclcpp::get_logger("EnemyDetectedCondition");

};

} // namespace rm_behavior

#endif // RM_BEHAVIOR__PLUGINS__CONDITION__ENEMY_DETECTED_HPP_