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

#ifndef RM_BEHAVIOR__RM_BEHAVIOR_CLIENT_HPP_
#define RM_BEHAVIOR__RM_BEHAVIOR_CLIENT_HPP_

#include <memory>
#include <string>

#include "btcpp_ros2_interfaces/action/execute_tree.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/client.hpp"

namespace rm_behavior
{

class BehaviorClient : public rclcpp::Node
{
public:
  using BTExecuteTree = btcpp_ros2_interfaces::action::ExecuteTree;
  using GoalHandleBTExecuateTree = rclcpp_action::ClientGoalHandle<BTExecuteTree>;

  explicit BehaviorClient(const rclcpp::NodeOptions & options);

private:
  void sendGoal();
  void resultCallback(const GoalHandleBTExecuateTree::WrappedResult & result);
  void feedbackCallback(
    GoalHandleBTExecuateTree::SharedPtr goal_handle,
    const std::shared_ptr<const BTExecuteTree::Feedback> feedback);

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_action::Client<BTExecuteTree>::SharedPtr action_client_;
  std::string target_tree_;
};

}  // namespace rm_behavior

#endif  // RM_BEHAVIOR__RM_BEHAVIOR_CLIENT_HPP_
