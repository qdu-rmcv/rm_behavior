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

#include "rm_behavior/plugins/action/nav2goal.hpp"
#include "rm_behavior/custom_types.hpp"

namespace rm_behavior {

// Nav2GoalAction::Nav2GoalAction(const std::string &name,
//                              const BT::NodeConfig &conf,
//                              const BT::RosNodeParams &params)
// : RosActionNode<nav2_msgs::action::NavigateToPose>(name, conf, params)
// {
// }

bool Nav2GoalAction::setGoal(nav2_msgs::action::NavigateToPose::Goal & goal)
{
  auto receive_goal = getInput<geometry_msgs::msg::PoseStamped>("goal");

  goal.pose.header.frame_id = "map";
  goal.pose.header.stamp = now();
  goal.pose.pose = receive_goal->pose;

  return true;
}

BT::NodeStatus Nav2GoalAction::onResultReceived(const WrappedResult & wr)
{
  switch (wr.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(logger(), "导航成功!");
      return BT::NodeStatus::SUCCESS;

    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(logger(), "导航被服务器中止");
      return BT::NodeStatus::FAILURE;

    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(logger(), "导航被取消");
      return BT::NodeStatus::FAILURE;

    default:
      RCLCPP_ERROR(logger(), "未知的导航结果代码: %d", static_cast<int>(wr.code));
      return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus Nav2GoalAction::onFeedback(
  const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
{
  RCLCPP_DEBUG(logger(), "剩余距离: %f", feedback->distance_remaining);
  return BT::NodeStatus::RUNNING;
}

void Nav2GoalAction::onHalt() { RCLCPP_INFO(logger(), "Nav2GoalAction 已被暂停."); }

BT::NodeStatus Nav2GoalAction::onFailure(BT::ActionNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "Nav2GoalAction 失败，错误代码: %d", error);
  return BT::NodeStatus::FAILURE;
}

BT::PortsList Nav2GoalAction::providedPorts()
{
  BT::PortsList additional_ports = {
    BT::InputPort<geometry_msgs::msg::PoseStamped>(
      "goal", "0;0;0", "导航目标位置，格式为 `x;y;yaw`"),
    BT::InputPort<std::string>(
      "action_client", "navigate_to_pose", "导航动作客户端名称"),
  };
  return providedBasicPorts(additional_ports);
}

}  // namespace rm_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior::Nav2GoalAction, "Nav2Goal");