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

#ifndef RM_BEHAVIOR__PLUGINS__ACTION__NAVIGAT_TO_GOAL_HPP_
#define RM_BEHAVIOR__PLUGINS__ACTION__NAVIGAT_TO_GOAL_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_ros2/bt_action_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace BT {
template <> geometry_msgs::msg::PoseStamped convertFromString(StringView key) {
  auto parts = BT::splitString(key, ';');
  if (parts.size() == 7) {
    geometry_msgs::msg::PoseStamped output;
    output.pose.position.x = convertFromString<double>(parts[0]);
    output.pose.position.y = convertFromString<double>(parts[1]);
    output.pose.position.z = convertFromString<double>(parts[2]);
    output.pose.orientation.x = convertFromString<double>(parts[3]);
    output.pose.orientation.y = convertFromString<double>(parts[4]);
    output.pose.orientation.z = convertFromString<double>(parts[5]);
    output.pose.orientation.w = convertFromString<double>(parts[6]);
    return output;
  } else if (parts.size() == 3) {
    tf2::Quaternion quaternion;
    auto goal_yaw = convertFromString<double>(parts[2]);
    quaternion.setRPY(0, 0, goal_yaw);
    geometry_msgs::msg::PoseStamped output;
    output.pose.position.x = convertFromString<double>(parts[0]);
    output.pose.position.y = convertFromString<double>(parts[1]);
    output.pose.position.z = 0.0;
    output.pose.orientation = tf2::toMsg(quaternion);
    return output;
  } else {
    throw RuntimeError("Invalid input");
  }
}
} // namespace BT

namespace rm_behavior {
class Nav2GoalAction
    : public BT::RosActionNode<nav2_msgs::action::NavigateToPose> {

public:
  Nav2GoalAction(const std::string &name, const BT::NodeConfig &conf,
             const BT::RosNodeParams &params);

  static BT::PortsList providedPorts();

  bool setGoal(Goal &goal) override;

  void onHalt() override;

  BT::NodeStatus onResultReceived(const WrappedResult &wr) override;

  BT::NodeStatus
  onFeedback(const std::shared_ptr<const Feedback> feedback) override;

  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;
};
} // namespace rm_behavior

#endif // RM_BEHAVIOR__PLUGINS__ACTION__NAVIGAT_TO_GOAL_HPP_