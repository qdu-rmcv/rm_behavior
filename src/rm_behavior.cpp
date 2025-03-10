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

#include "rm_behavior/rm_behavior.hpp"
#include "rclcpp/rclcpp.hpp"

#include "behaviortree_cpp/xml_parsing.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <filesystem>
#include <fstream>

namespace rm_behavior {

RMBehavior::RMBehavior(const rclcpp::NodeOptions& options)
: TreeExecutionServer(options)
{
  // 声明参数
  node()->declare_parameter("bt_xml_dirs", 
    std::vector<std::string>{"/home/jquark/rm/demo_behavior/src/rm_behavior/behavior_trees"});
 
  // 创建裁判系统消息订阅器
  referee_sub_ = node()->create_subscription<auto_aim_interfaces::msg::Referee>(
    "referee", 10, [this](const auto_aim_interfaces::msg::Referee::SharedPtr msg) {
      // 在收到数据后打印哨兵血量信息
      RCLCPP_INFO(node()->get_logger(), "received sentry_hp: %d", msg->sentry_hp);
      
      // 将裁判系统的各项数据更新到黑板变量中
      globalBlackboard()->set("referee_event_data", static_cast<int>(msg->event_data));
      globalBlackboard()->set("referee_time", static_cast<int>(msg->time));
      globalBlackboard()->set("referee_rfid", static_cast<int>(msg->rfid));
      globalBlackboard()->set("referee_base_hp", static_cast<int>(msg->base_hp));
      globalBlackboard()->set("referee_sentry_hp", static_cast<int>(msg->sentry_hp));
      globalBlackboard()->set("referee_outpost_hp", static_cast<int>(msg->outpost_hp));
      globalBlackboard()->set("referee_projectile_allowance", static_cast<int>(msg->projectile_allowance_17mm));
      // 记录日志
      RCLCPP_DEBUG(node()->get_logger(), "Updated referee data to blackboard");
    });
  
  RCLCPP_INFO(node()->get_logger(), "RMBehavior initialized with CLI logger");
}

void RMBehavior::onTreeCreated(BT::Tree& tree)
{
  logger_cout_ = std::make_shared<BT::StdCoutLogger>(tree);
  
  RCLCPP_INFO(node()->get_logger(), "Behavior tree created with CLI logging");
}

std::optional<std::string> RMBehavior::onTreeExecutionCompleted(BT::NodeStatus status, bool was_cancelled)
{
  logger_cout_.reset();
  
  if (was_cancelled) {
    RCLCPP_INFO(node()->get_logger(), "Behavior tree execution was cancelled");
  } else {
    RCLCPP_INFO(node()->get_logger(), "Behavior tree execution completed with status: %s",
                status == BT::NodeStatus::SUCCESS ? "SUCCESS" : "FAILURE");
  }
  
  return std::nullopt;
}

} // namespace rm_behavior

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto action_server = std::make_shared<rm_behavior::RMBehavior>(options);

  RCLCPP_INFO(action_server->node()->get_logger(), "Starting RMBehavior Server");

  rclcpp::executors::MultiThreadedExecutor exec(
    rclcpp::ExecutorOptions(), 0, false, std::chrono::milliseconds(250));
  exec.add_node(action_server->node());
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
