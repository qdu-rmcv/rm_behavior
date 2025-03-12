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

#ifndef RM_BEHAVIOR__PLUGINS__ACTION__NAV2GOAL_HPP_
#define RM_BEHAVIOR__PLUGINS__ACTION__NAV2GOAL_HPP_

#include <memory>
#include <string>

#include "behaviortree_ros2/bt_action_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "rm_behavior/custom_types.hpp"

namespace rm_behavior {
class Nav2GoalAction
    : public BT::RosActionNode<nav2_msgs::action::NavigateToPose> {

public:
  Nav2GoalAction(const std::string &name, const BT::NodeConfig &conf,
             const BT::RosNodeParams &params)
    : BT::RosActionNode<nav2_msgs::action::NavigateToPose>(name, conf, 
      [&]() {
        auto modified_params = params;
        // 设置默认动作服务器名称
        if (modified_params.default_port_value.empty()) {
          modified_params.default_port_value = "navigate_to_pose";
        }
        modified_params.server_timeout=std::chrono::seconds(500);
        return modified_params;
      }())
  {
  }

  static BT::PortsList providedPorts();

  bool setGoal(Goal &goal) override;

  void onHalt() override;

  BT::NodeStatus onResultReceived(const WrappedResult &wr) override;

  BT::NodeStatus
  onFeedback(const std::shared_ptr<const Feedback> feedback) override;

  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;
};
} // namespace rm_behavior

#endif // RM_BEHAVIOR__PLUGINS__ACTION__NAV2GOAL_HPP_