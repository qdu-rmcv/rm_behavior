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

#include "rm_behavior/rm_behavior_client.hpp"

#include "rclcpp/logging.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

namespace rm_behavior
{

BehaviorClient::BehaviorClient(const rclcpp::NodeOptions & options)
: Node("rm_behavior_client", options)
{
  declare_parameter<std::string>("target_tree", "rm_behavior");
  get_parameter("target_tree", target_tree_);

  action_client_ = rclcpp_action::create_client<BTExecuteTree>(this, "rm_behavior");

  if (!action_client_->wait_for_action_server()) {
    RCLCPP_ERROR(get_logger(), "Action server not available!");
    return;
  }

  timer_ = create_wall_timer(100ms, std::bind(&BehaviorClient::sendGoal, this));
}

void BehaviorClient::sendGoal()
{
  using namespace std::placeholders;
  timer_->cancel();

  auto goal_msg = BTExecuteTree::Goal();
  goal_msg.target_tree = target_tree_;

  RCLCPP_INFO(get_logger(), "Sending goal to execute Behavior Tree: %s", target_tree_.c_str());

  auto options = rclcpp_action::Client<BTExecuteTree>::SendGoalOptions();
  options.feedback_callback = std::bind(&BehaviorClient::feedbackCallback, this, _1, _2);
  options.result_callback = std::bind(&BehaviorClient::resultCallback, this, _1);

  action_client_->async_send_goal(goal_msg, options);
}

void BehaviorClient::resultCallback(
  const GoalHandleBTExecuateTree::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(get_logger(), "Goal succeeded: %s", result.result->return_message.c_str());
      // 成功后可以选择是否立即重新发送目标
      // sendGoal();  // 连续执行
      timer_->reset(); // 等待计时器周期
      break; 
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(get_logger(), "Goal was canceled.");
      sendGoal();  // 取消后立即重试
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(get_logger(), "Goal failed: %s", result.result->return_message.c_str());
      sendGoal();  // 失败后立即重试，不等待计时器
      break;
    case rclcpp_action::ResultCode::UNKNOWN:
      sendGoal();  // 未知状态也立即重试
      break;
  }
}

void BehaviorClient::feedbackCallback(
  GoalHandleBTExecuateTree::SharedPtr /*goal_handle*/,
  const std::shared_ptr<const BTExecuteTree::Feedback> feedback)
{
  RCLCPP_INFO(get_logger(), "Received feedback: %s", feedback->message.c_str());
}

}  // namespace rm_behavior

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rm_behavior::BehaviorClient)
