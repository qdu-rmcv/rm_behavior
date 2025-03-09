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
#include <filesystem>
#include <fstream>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "auto_aim_interfaces/msg/referee.hpp"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"

namespace rm_behavior {

RmBehaviorServer::RmBehaviorServer(const rclcpp::NodeOptions & options)
: BT::TreeExecutionServer(options),
  tick_count_(0), use_logger_(false) {
  // 从参数服务器获取树名称，提供默认值
  this->node()->declare_parameter<std::string>("default_tree", "rm_behavior_server");
  std::string default_tree_name;
  this->node()->get_parameter("default_tree", default_tree_name);
  
  // 注册可用的树
  for (const auto & entry : std::filesystem::directory_iterator(this->node()->get_parameter("bt_xml_dir").as_string())) {
    if (entry.path().extension() == ".xml") {
      std::string tree_path = entry.path().string();
      std::string tree_name = entry.path().stem().string();
      RCLCPP_INFO(this->node()->get_logger(), "Registering tree: %s from %s", 
                 tree_name.c_str(), tree_path.c_str());
      this->registerTree(tree_name, tree_path);
    }
  }

  shared_blackboard_ = BT::Blackboard::create();
  this->node()->declare_parameter("use_logger", true);
  use_logger_ = this->node()->get_parameter("use_logger").as_bool();
  goal_pose_pub_ = this->node()->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/goal_pose", 10);
    
  // 订阅裁判系统话题，并将各个字段分别映射到不同的黑板变量
  auto referee_subscription = this->node()->create_subscription<auto_aim_interfaces::msg::Referee>(
    "/referee", 10,
    [this](const auto_aim_interfaces::msg::Referee::SharedPtr msg) {
      auto bb = this->getTreeBlackboard();
      bb->set("sentry_hp", msg->sentry_hp);
      bb->set("outpost_hp", msg->outpost_hp);
      bb->set("base_hp", msg->base_hp);
      bb->set("time", msg->time);
      bb->set("event_data", msg->event_data);
      bb->set("rfid", msg->rfid);
      bb->set("projectile_allowance_17mm", msg->projectile_allowance_17mm);
    });
  
  subscriptions_.push_back(referee_subscription);
  RCLCPP_INFO(this->node()->get_logger(), "Subscribed to /referee topic with individual field mapping");
  
  // 记录其他必要的系统状态信息
  // 例如，机器人位置、装甲板检测结果等
  // subscribe<geometry_msgs::msg::PoseStamped>("/robot_pose", "robot_pose");
  // subscribe<armor_detector::msg::ArmorList>("/armor_detector/armors", "detected_armors");
  
  start_time_ = this->node()->now();
  
  RCLCPP_INFO(this->node()->get_logger(), "RmBehaviorServer initialized successfully");
}

// 获取黑板的统一访问方法实现
BT::Blackboard::Ptr RmBehaviorServer::getTreeBlackboard() {
  if (current_tree_.has_value()) {
    // 如果有活动的树，使用树的黑板
    return current_tree_.value().get().subtrees[0]->blackboard;
  }
  // 否则使用共享黑板
  return shared_blackboard_;
}

bool RmBehaviorServer::onGoalReceived(const std::string & tree_name, const std::string & payload) {
  RCLCPP_INFO(
    this->node()->get_logger(), 
    "Received request to execute behavior tree: %s with payload: %s", 
    tree_name.c_str(), payload.c_str());
  
  // 在这里可以添加验证逻辑，例如检查树名称是否有效
  return true;  // 允许执行行为树
}

void RmBehaviorServer::onTreeCreated(BT::Tree & tree) {
  // 保存树引用
  current_tree_ = std::ref(tree);
  
  // 重置计数器
  tick_count_ = 0;
  
  // 初始化日志记录器
  if (use_logger_) {
    logger_ = std::make_shared<BT::StdCoutLogger>(tree);
    RCLCPP_INFO(this->node()->get_logger(), "BehaviorTree logger initialized");
  }
  
  // 将当前时间点记录在黑板上，方便节点使用
  auto bb = getTreeBlackboard();
  bb->set("start_time", this->node()->now());
  
  // 初始化裁判系统相关的黑板变量（以防在收到第一条消息前行为树就开始执行）
  bb->set("event_data", 0u);
  bb->set("rfid", 0u);
  bb->set("base_hp", 0u);
  bb->set("sentry_hp", 0u);
  bb->set("outpost_hp", 0u);
  bb->set("projectile_allowance", 0u);
  
  // 复制共享黑板中的数据到树黑板
  if (shared_blackboard_) {
    for (const auto& key_str : shared_blackboard_->getKeys()) {
      try {
        std::string key(key_str);
        auto value = shared_blackboard_->getAnyLocked(key);
        if (value) {  // 检查LockedPtr是否有效
          bb->set(key, *value);  // 直接解引用
        }
      } catch (const std::exception& e) {
        RCLCPP_WARN(
          this->node()->get_logger(), 
          "Failed to copy blackboard key %s: %s", 
          std::string(key_str).c_str(), e.what());
      }
    }
  }
  
  RCLCPP_INFO(this->node()->get_logger(), "Behavior tree created successfully with initialized referee data");
}

std::optional<BT::NodeStatus> RmBehaviorServer::onLoopAfterTick(BT::NodeStatus status) {
  tick_count_++;
  
  if (tick_count_ % 100 == 0) {
    auto current_time = this->node()->now();
    double elapsed = (current_time - start_time_).seconds();
    RCLCPP_DEBUG(
      this->node()->get_logger(), 
      "Tree has been ticking for %.2f seconds, completed %d ticks", 
      elapsed, tick_count_);
  }
  
  // 检查是否需要中断行为树执行
  // 例如，如果检测到紧急情况，可以返回状态使行为树提前终止
  
  return std::nullopt;  // 继续正常执行
}

std::optional<std::string> RmBehaviorServer::onTreeExecutionCompleted(
  BT::NodeStatus status, bool was_cancelled) {
  // 树执行完成后，清除树引用
  current_tree_.reset();
  
  if (was_cancelled) {
    RCLCPP_WARN(this->node()->get_logger(), "Behavior tree execution was cancelled");
    return "Tree execution cancelled by client";
  }
  
  if (status == BT::NodeStatus::SUCCESS) {
    RCLCPP_INFO(this->node()->get_logger(), "Behavior tree completed successfully");
    return "Tree execution completed successfully";
  } else {
    RCLCPP_ERROR(this->node()->get_logger(), "Behavior tree execution failed");
    return "Tree execution failed";
  }
}

void RmBehaviorServer::publishGoalPose(const geometry_msgs::msg::PoseStamped & pose) {
  goal_pose_pub_->publish(pose);
  RCLCPP_INFO(
    this->node()->get_logger(), 
    "Published goal pose at position [%.2f, %.2f, %.2f]",
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
}

}  // namespace rm_behavior

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    
    // 可以通过命令行参数覆盖默认值
    std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv);
    if (args.size() > 1) {
        std::string xml_path = args[1];
        std::filesystem::path bt_path(xml_path);
        if (bt_path.parent_path() != "") {
            options.parameter_overrides().push_back(
                rclcpp::Parameter("bt_xml_dir", bt_path.parent_path().string()));
        }
    } else {
        // 设置默认的行为树XML文件目录
        std::string default_bt_xml_dir = std::filesystem::current_path().string() + "/behavior_trees";
        options.parameter_overrides().push_back(
            rclcpp::Parameter("bt_xml_dir", default_bt_xml_dir));
        
        // 确保目录存在
        if (!std::filesystem::exists(default_bt_xml_dir)) {
            std::filesystem::create_directories(default_bt_xml_dir);
        }
    }
    
    auto action_server = std::make_shared<rm_behavior::RmBehaviorServer>(options);
    RCLCPP_INFO(action_server->node()->get_logger(), "Starting RmBehaviorServer");

    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(action_server->node());
    exec.spin();
    exec.remove_node(action_server->node());

    rclcpp::shutdown();
    return 0;
}