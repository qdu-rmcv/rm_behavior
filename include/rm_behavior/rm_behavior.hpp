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

#ifndef RM_BEHAVIOR__RM_BEHAVIOR_HPP_
#define RM_BEHAVIOR__RM_BEHAVIOR_HPP_

#include <behaviortree_cpp/loggers/bt_cout_logger.h>

#include <memory>
#include <string>
#include <vector>
#include <optional>

#include "behaviortree_ros2/tree_execution_server.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "auto_aim_interfaces/msg/referee.hpp"

namespace rm_behavior {

class RmBehaviorServer : public BT::TreeExecutionServer
{
public:
  explicit RmBehaviorServer(const rclcpp::NodeOptions & options);
  ~RmBehaviorServer() override = default;

  // BT::TreeExecutionServer 重写接口
  bool onGoalReceived(const std::string & tree_name, const std::string & payload) override;
  void onTreeCreated(BT::Tree & tree) override;
  std::optional<BT::NodeStatus> onLoopAfterTick(BT::NodeStatus status) override;
  std::optional<std::string> onTreeExecutionCompleted(
    BT::NodeStatus status, bool was_cancelled) override;

  // 目标位置发布方法
  void publishGoalPose(const geometry_msgs::msg::PoseStamped & pose);

private:
  // 话题订阅模板方法
  template <typename T>
  void subscribe(
    const std::string & topic, const std::string & bb_key,
    const rclcpp::QoS & qos = rclcpp::QoS(10))
  {
    auto subscription = this->create_subscription<T>(
      topic, qos,
      [this, bb_key](const typename T::SharedPtr msg) {
        // 将接收到的消息存入行为树黑板
        this->getBlackboard()->set(bb_key, *msg);
        RCLCPP_DEBUG(this->get_logger(), "Received message on topic %s and saved to blackboard", bb_key.c_str());
      });
      
    subscriptions_.push_back(subscription);
    RCLCPP_INFO(this->get_logger(), "Subscribed to %s on blackboard key %s", topic.c_str(), bb_key.c_str());
  }

  // 存储所有订阅对象
  std::vector<std::shared_ptr<rclcpp::SubscriptionBase>> subscriptions_;
  
  // 目标位置发布者
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub_;
  
  // 行为树日志记录器
  std::shared_ptr<BT::StdCoutLogger> logger_;
  
  // 行为树状态相关
  uint32_t tick_count_;
  bool use_logger_;
  
  // 记录节点启动时间，用于计算运行时间
  rclcpp::Time start_time_;
};

}  // namespace rm_behavior

#endif // RM_BEHAVIOR__RM_BEHAVIOR_HPP_