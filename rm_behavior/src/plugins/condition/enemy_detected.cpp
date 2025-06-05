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

#include "rm_behavior/plugins/condition/enemy_detected.hpp"

namespace rm_behavior
{
    EnemyDetectedCondition::EnemyDetectedCondition(
        const std::string &name, const BT::NodeConfig &config)
        : BT::ConditionNode(name, config), logger_(rclcpp::get_logger("EnemyDetected"))
    {
    }

    BT::PortsList EnemyDetectedCondition::providedPorts()
    {
        return {
        BT::InputPort<auto_aim_interfaces::msg::Armors>(
        "key_port", "{@detector_armors}", "Vision detector port on blackboard"),  // 视觉检测器在黑板上的端口
        BT::InputPort<std::vector<int>>(
        "armor_id", "1;2;3;4;6",
        "Expected id of armors. "),  // 期望的装甲板ID，默认为1、2、3、4、6
        BT::InputPort<float>("max_distance", 8.0, "Distance to enemy target"),  // 敌人目标的最大检测距离，默认为8米
        };

        };

    BT::NodeStatus EnemyDetectedCondition::tick()
    {
        std::vector<int> expected_armor_ids;  // 期望检测到的装甲板ID列表
        float max_distance;  // 最大检测距离
        auto msg = getInput<auto_aim_interfaces::msg::Armors>("key_port");  // 获取装甲板检测消息
        if (!msg) {
        RCLCPP_ERROR(logger_, "Detector message is not available");  // 检测器消息不可用
        return BT::NodeStatus::FAILURE;
        }

        getInput("armor_id", expected_armor_ids);  // 获取期望的装甲板ID参数
        getInput("max_distance", max_distance);  // 获取最大检测距离参数

        for (const auto & armor : msg->armors) {
        float distance_to_enemy = std::hypot(armor.pose.position.x, armor.pose.position.y);  // 计算敌人距离

        if (armor.number.empty()) {  // 如果装甲板编号为空，跳过
        continue;
        }
        int armor_id = std::stoi(armor.number);  // 将装甲板编号字符串转换为整数
        const bool is_armor_id_match =  // 检查装甲板ID是否在期望列表中
        std::find(expected_armor_ids.begin(), expected_armor_ids.end(), armor_id) !=
        expected_armor_ids.end();

        const bool is_within_distance = (distance_to_enemy <= max_distance);  // 检查敌人是否在最大检测距离内

    if (is_armor_id_match && is_within_distance) {  // 如果ID匹配且在检测距离内，返回成功
        return BT::NodeStatus::SUCCESS;
        }
    }
        return BT::NodeStatus::FAILURE;  // 没有满足条件的敌人，返回失败
    }

    
    }
#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior::EnemyDetectedCondition>("EnemyDetected");
}