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

#include "rm_behavior/plugins/action/topic_nav2goal.hpp"

#include "rm_behavior/custom_types.hpp"

namespace rm_behavior
{

TopicNav2GoalAction::TopicNav2GoalAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: RosTopicPubNode<geometry_msgs::msg::PoseStamped>(name, conf, params)
{
}

bool TopicNav2GoalAction::setMessage(geometry_msgs::msg::PoseStamped & msg)
{
  auto goal = getInput<geometry_msgs::msg::PoseStamped>("goal");

  msg.header.stamp = now();
  msg.header.frame_id = "map";
  msg.pose = goal->pose;
  return true;
}

BT::PortsList TopicNav2GoalAction::providedPorts()
{
  BT::PortsList additional_ports = {
    BT::InputPort<geometry_msgs::msg::PoseStamped>(
      "goal", "0;0;0", "Expected goal pose that send to nav2. Fill with format `x;y;yaw`"),
  };
  return providedBasicPorts(additional_ports);
}

}  // namespace rm_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior::TopicNav2GoalAction, "TopicNav2Goal");
