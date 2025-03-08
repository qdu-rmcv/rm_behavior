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

namespace rm_behavior {

RmBehaviorServer::RmBehaviorServer(const rclcpp::NodeOptions & options)
: BT::TreeExecutionServer(options, "rm_behavior_server"),
  tick_count_(0) {
  // 声明参数
  this->declare_parameter("use_logger", true);
  
  // 获取参数
  use_logger_ = this->get_parameter("use_logger").as_bool();
  
  // 设置目标姿态发布器
  goal_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/goal_pose", 10);
    
  // 订阅裁判系统话题，并将各个字段分别映射到不同的黑板变量
  auto referee_subscription = this->create_subscription<auto_aim_interfaces::msg::Referee>(
    "/referee", 10,
    [this](const auto_aim_interfaces::msg::Referee::SharedPtr msg) {
      this->getBlackboard()->set("referee", *msg);
      
      // 将各个字段分别存储为独立的黑板变量，方便直接访问
      this->getBlackboard()->set("event_data", msg->event_data);
      this->getBlackboard()->set("time", msg->time);
      this->getBlackboard()->set("rfid", msg->rfid);
      this->getBlackboard()->set("base_hp", msg->base_hp);
      this->getBlackboard()->set("sentry_hp", msg->sentry_hp);
      this->getBlackboard()->set("outpost_hp", msg->outpost_hp);
      this->getBlackboard()->set("projectile_allowance", msg->projectile_allowance_17mm);
    });
  
  subscriptions_.push_back(referee_subscription);
  RCLCPP_INFO(this->get_logger(), "Subscribed to /referee topic with individual field mapping");
  
  // 记录其他必要的系统状态信息
  // 例如，机器人位置、装甲板检测结果等
  // subscribe<geometry_msgs::msg::PoseStamped>("/robot_pose", "robot_pose");
  // subscribe<armor_detector::msg::ArmorList>("/armor_detector/armors", "detected_armors");
  
  start_time_ = this->now();
  
  RCLCPP_INFO(this->get_logger(), "RmBehaviorServer initialized successfully");
}

bool RmBehaviorServer::onGoalReceived(const std::string & tree_name, const std::string & payload) {
  RCLCPP_INFO(
    this->get_logger(), 
    "Received request to execute behavior tree: %s with payload: %s", 
    tree_name.c_str(), payload.c_str());
  
  // 在这里可以添加验证逻辑，例如检查树名称是否有效
  return true;  // 允许执行行为树
}

void RmBehaviorServer::onTreeCreated(BT::Tree & tree) {
  // 重置计数器
  tick_count_ = 0;
  
  // 初始化日志记录器
  if (use_logger_) {
    logger_ = std::make_shared<BT::StdCoutLogger>(tree);
    RCLCPP_INFO(this->get_logger(), "BehaviorTree logger initialized");
  }
  
  // 将当前时间点记录在黑板上，方便节点使用
  tree.subtrees[0]->blackboard->set("start_time", this->now());
  
  // 初始化裁判系统相关的黑板变量（以防在收到第一条消息前行为树就开始执行）
  auto blackboard = tree.subtrees[0]->blackboard;
  blackboard->set("event_data", 0u);
  blackboard->set("rfid", 0u);
  blackboard->set("base_hp", 0u);
  blackboard->set("sentry_hp", 0u);
  blackboard->set("outpost_hp", 0u);
  blackboard->set("projectile_allowance", 0u);
  
  RCLCPP_INFO(this->get_logger(), "Behavior tree created successfully with initialized referee data");
}

std::optional<BT::NodeStatus> RmBehaviorServer::onLoopAfterTick(BT::NodeStatus status) {
  tick_count_++;
  
  if (tick_count_ % 100 == 0) {
    auto current_time = this->now();
    double elapsed = (current_time - start_time_).seconds();
    RCLCPP_DEBUG(
      this->get_logger(), 
      "Tree has been ticking for %.2f seconds, completed %d ticks", 
      elapsed, tick_count_);
  }
  
  // 检查是否需要中断行为树执行
  // 例如，如果检测到紧急情况，可以返回状态使行为树提前终止
  
  return std::nullopt;  // 继续正常执行
}

std::optional<std::string> RmBehaviorServer::onTreeExecutionCompleted(
  BT::NodeStatus status, bool was_cancelled) {
  if (was_cancelled) {
    RCLCPP_WARN(this->get_logger(), "Behavior tree execution was cancelled");
    return "Tree execution cancelled by client";
  }
  
  if (status == BT::NodeStatus::SUCCESS) {
    RCLCPP_INFO(this->get_logger(), "Behavior tree completed successfully");
    return "Tree execution completed successfully";
  } else {
    RCLCPP_ERROR(this->get_logger(), "Behavior tree execution failed");
    return "Tree execution failed";
  }
}

void RmBehaviorServer::publishGoalPose(const geometry_msgs::msg::PoseStamped & pose) {
  goal_pose_pub_->publish(pose);
  RCLCPP_INFO(
    this->get_logger(), 
    "Published goal pose at position [%.2f, %.2f, %.2f]",
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
}

}  // namespace rm_behavior

// 直接在源文件中实现主函数，无需额外的 main.cpp
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    auto action_server = std::make_shared<rm_behavior::RmBehaviorServer>(options);

    RCLCPP_INFO(action_server->node()->get_logger(), "Starting RmBehaviorServer");

    rclcpp::executors::MultiThreadedExecutor exec(
        rclcpp::ExecutorOptions(), 0, false, std::chrono::milliseconds(250));
    exec.add_node(action_server->node());
    exec.spin();
    exec.remove_node(action_server->node());

    // Groot2 editor requires a model of your registered Nodes.
    // You don't need to write that by hand, it can be automatically
    // generated using the following command.
    std::string xml_models = BT::writeTreeNodesModelXML(action_server->factory());

    // Save the XML models to a file
    std::ofstream file(std::filesystem::path(ROOT_DIR) / "behavior_trees" / "models.xml");
    file << xml_models;

    rclcpp::shutdown();
    return 0;
}