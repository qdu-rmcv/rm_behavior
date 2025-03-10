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

#ifndef RM_BEHAVIOR_HPP_
#define RM_BEHAVIOR_HPP_

#include "auto_aim_interfaces/msg/referee.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <behaviortree_ros2/tree_execution_server.hpp>

namespace rm_behavior {

class RMBehavior : public BT::TreeExecutionServer
{
public:
  explicit RMBehavior(const rclcpp::NodeOptions& options);
protected:
  void onTreeCreated(BT::Tree& tree) override;
  std::optional<std::string> onTreeExecutionCompleted(BT::NodeStatus status, bool was_cancelled) override;

private:
  // 裁判系统消息订阅器
  rclcpp::Subscription<auto_aim_interfaces::msg::Referee>::SharedPtr referee_sub_;
  
  //  CLI 日志记录器
  std::shared_ptr<BT::StdCoutLogger> logger_cout_;
};

}  // namespace rm_behavior

#endif  // RM_BEHAVIOR_HPP_
