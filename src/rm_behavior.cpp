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
  // 声明并获取参数
  this->node()->declare_parameter<std::string>("default_tree", "rm_behavior");
  std::string default_tree_name;
  this->node()->get_parameter("default_tree", default_tree_name);
  
  this->node()->declare_parameter<std::string>("bt_xml_dir", 
    std::filesystem::current_path().string() + "/behavior_trees");
  std::string bt_xml_dir = this->node()->get_parameter("bt_xml_dir").as_string();
  RCLCPP_INFO(this->node()->get_logger(), "Loading behavior trees from: %s", bt_xml_dir.c_str());

  // 初始化共享黑板
  shared_blackboard_ = BT::Blackboard::create();

  // 声明并获取其他参数
  this->node()->declare_parameter("use_logger", true);
  use_logger_ = this->node()->get_parameter("use_logger").as_bool();
  // 设置发布器
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

  // 记录启动时间
  start_time_ = this->node()->now();

  RCLCPP_INFO(this->node()->get_logger(), "RmBehaviorServer initialized successfully");
}

void RmBehaviorServer::registerNodesIntoFactory(BT::BehaviorTreeFactory& factory)
{
  TreeExecutionServer::registerNodesIntoFactory(factory);
  std::string bt_xml_dir = this->node()->get_parameter("bt_xml_dir").as_string();

  try {
    for (const auto & entry : std::filesystem::directory_iterator(bt_xml_dir)) {
      if (entry.path().extension() == ".xml") {
        std::string tree_path = entry.path().string();
        std::string tree_name = entry.path().stem().string();
        RCLCPP_INFO(this->node()->get_logger(), "Registering tree: %s from %s", 
                  tree_name.c_str(), tree_path.c_str());
        try {
          factory.registerBehaviorTreeFromFile(tree_path);
        } catch (const std::exception& e) {
          RCLCPP_ERROR(this->node()->get_logger(), "Failed to register tree from file %s: %s", 
                       tree_path.c_str(), e.what());
        }
      }
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->node()->get_logger(), "Error while loading behavior trees: %s", e.what());
  }
}

BT::Blackboard::Ptr RmBehaviorServer::getTreeBlackboard() {
  try {
    BT::Tree& non_const_tree = const_cast<BT::Tree&>(this->tree());
    if (non_const_tree.rootNode()) {
      return non_const_tree.rootBlackboard();
    }
  }
  catch (const std::exception& e) {
    RCLCPP_DEBUG(this->node()->get_logger(), "Failed to get tree blackboard: %s", e.what());
  }
  return shared_blackboard_;
}

bool RmBehaviorServer::onGoalReceived(const std::string & tree_name, const std::string & payload) {
  RCLCPP_INFO(
    this->node()->get_logger(),
    "Received request to execute behavior tree: %s with payload: %s",
    tree_name.c_str(), payload.c_str());

  return true;
}


void RmBehaviorServer::onTreeCreated(BT::Tree & tree) {
  tick_count_ = 0;

  if (use_logger_) {
    logger_ = std::make_shared<BT::StdCoutLogger>(tree);
    RCLCPP_INFO(this->node()->get_logger(), "BehaviorTree logger initialized");
  }
  auto bb = tree.rootBlackboard();
  bb->set("start_time", this->node()->now());

  // 初始化裁判系统相关的黑板变量
  bb->set("event_data", 0u);
  bb->set("rfid", 0u);
  bb->set("base_hp", 0u);
  bb->set("sentry_hp", 0u);
  bb->set("outpost_hp", 0u);
  bb->set("projectile_allowance_17mm", 0u);

  // 复制共享黑板中的数据到树黑板
  if (shared_blackboard_) {
    for (const auto& key_str : shared_blackboard_->getKeys()) {
      try {
        std::string key(key_str);
        // 键肯定存在，因为它是从 getKeys() 获取的
        bb->set(key, shared_blackboard_->get<BT::Any>(key));
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
  return std::nullopt;
}

std::optional<std::string> RmBehaviorServer::onTreeExecutionCompleted(
  BT::NodeStatus status, bool was_cancelled) {
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

std::optional<std::string> RmBehaviorServer::onLoopFeedback() {
  if (tick_count_ % 10 == 0) {
    auto bb = this->getTreeBlackboard();
    std::stringstream ss;
    ss << "Tree [" << this->treeName() << "] has completed " 
       << tick_count_ << " ticks\n";
    
    // 添加关键黑板值的输出
    try {
      ss << "Referee data: "
         << "event_data=" << bb->get<unsigned int>("event_data") << ", "
         << "rfid=" << static_cast<int>(bb->get<unsigned int>("rfid")) << ", "
         << "base_hp=" << bb->get<unsigned int>("base_hp") << ", "
         << "sentry_hp=" << bb->get<unsigned int>("sentry_hp") << ", "
         << "outpost_hp=" << bb->get<unsigned int>("outpost_hp") << ", "
         << "projectile=" << bb->get<unsigned int>("projectile_allowance_17mm");
    } catch (const std::exception& e) {
      ss << "Error reading blackboard: " << e.what();
    }
    
    return ss.str();
  }
  return std::nullopt;
}

// 目标位置发布方法
void RmBehaviorServer::publishGoalPose(const geometry_msgs::msg::PoseStamped & pose) {
  goal_pose_pub_->publish(pose);
  RCLCPP_INFO(
    this->node()->get_logger(), 
    "Published goal pose at position [%.2f, %.2f, %.2f]",
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
}

}  // namespace rm_behavior

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  rclcpp::NodeOptions options;
  
  std::string default_bt_xml_dir = std::filesystem::current_path().string() + "/behavior_trees";
  options.parameter_overrides().push_back(
    rclcpp::Parameter("bt_xml_dir", default_bt_xml_dir));
  
  if (!std::filesystem::exists(default_bt_xml_dir)) {
    try {
      std::filesystem::create_directories(default_bt_xml_dir);
      std::cout << "Created directory: " << default_bt_xml_dir << std::endl;
    } catch (const std::exception& e) {
      std::cerr << "Failed to create directory: " << e.what() << std::endl;
    }
  }
  
  auto behavior_server = std::make_shared<rm_behavior::RmBehaviorServer>(options);
  
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(behavior_server->node());
  exec.spin();
  
  rclcpp::shutdown();
  return 0;
}