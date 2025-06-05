# 青岛大学哨兵行为决策系统

青岛大学未来战队 2025 赛季 —— 哨兵机器人行为树决策系统开源项目

## 项目简介

本项目为青岛大学 RoboMaster 未来战队开发的**哨兵机器人行为决策系统**，基于 **BehaviorTree.CPP** 框架与 **ROS 2** 构建。该系统实现了模块化、可扩展、易调试的决策逻辑，极大地提升了哨兵在复杂战场环境下的自主性与适应能力。

相比于 24 赛季所使用的电控端拨点控制方式，当前版本采用行为树架构，使决策逻辑更加灵活、可维护性强，并具备良好的可读性和扩展性。

## 设计背景与演进

在 24 赛季传承过程中，老学长曾提到当时的决策逻辑完全依赖电控端进行拨点控制，这种方式存在以下问题：

- **固定僵化**：行为不可配置，难以应对动态变化；
- **缺乏灵活性**：逻辑修改需重新编写烧录代码，不利于战术上的调整；
- **扩展性差**：新增行为或状态需要大量改动。

为了突破这些限制，我们在参考多家开源和团队经验的基础上，引入了 **BehaviorTree.CPP** 框架，并结合 ROS 2 的通信机制构建了当前的决策系统，形成了现在的 `rm_behavior` 功能包。

## 核心架构

### BehaviorTree.CPP + ROS 2

我们使用 **BehaviorTree.CPP (BT.CPP)** 作为行为树框架，其具有以下优势：

- 支持同步/异步节点；
- 可扩展性强，支持自定义节点类型；
- 提供丰富的内置节点（如 Sequence、Selector、Fallback 等）；
- 易于与 ROS 2 集成，实现话题通信、服务调用等行为。

通过编写 `.xml` 文件定义行为树结构，所有行为逻辑都可通过组合节点来实现，极大提升了开发效率和逻辑清晰度。

### rm_behavior 功能包组成

``` text
rm_behavior/
├── include/
│   └── behavior_nodes/     # 行为树节点头文件
├── src/
│   ├── nodes/              # 各类行为节点实现
│   └── main.cpp            # 行为树执行入口
├── behavior_trees/         # 存放 .xml 行为树文件
├── launch/                 # 启动文件
└── CMakeLists.txt          # 编译配置
```

## 使用方法

### 自定义行为节点

本系统支持多种类型的自定义行为节点，主要分为**条件节点**和**动作节点**两大类。编写自定义节点需要继承对应的基类并实现相关接口。

#### 1. 条件节点（Condition Node）

条件节点用于判断某个条件是否满足，返回 `SUCCESS` 或 `FAILURE`。

**头文件示例** (`include/rm_behavior/plugins/condition/example_condition.hpp`)：

```cpp
#ifndef RM_BEHAVIOR__PLUGINS__CONDITION__EXAMPLE_CONDITION_HPP_
#define RM_BEHAVIOR__PLUGINS__CONDITION__EXAMPLE_CONDITION_HPP_

#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"

namespace rm_behavior {

class ExampleCondition : public BT::ConditionNode {
public:
    ExampleCondition(const std::string& name, const BT::NodeConfig& config);
    
    static BT::PortsList providedPorts();

private:
    BT::NodeStatus tick() override;
    
    rclcpp::Logger logger_ = rclcpp::get_logger("ExampleCondition");
};

} // namespace rm_behavior

#endif // RM_BEHAVIOR__PLUGINS__CONDITION__EXAMPLE_CONDITION_HPP_
```

**源文件实现** (`src/plugins/condition/example_condition.cpp`)：

```cpp
#include "rm_behavior/plugins/condition/example_condition.hpp"

namespace rm_behavior {

ExampleCondition::ExampleCondition(
    const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config) {}

BT::PortsList ExampleCondition::providedPorts() {
    return {
        BT::InputPort<int>("threshold", 100, "判断阈值"),
        BT::InputPort<std::string>("key_port", "{@sensor_data}", "传感器数据端口")
    };
}

BT::NodeStatus ExampleCondition::tick() {
    auto threshold = getInput<int>("threshold").value();
    // 实现具体判断逻辑
    bool condition_met = /* 你的判断逻辑 */;
    
    return condition_met ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

} // namespace rm_behavior

// 注册节点到工厂
#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
    factory.registerNodeType<rm_behavior::ExampleCondition>("ExampleCondition");
}
```

#### 2. 同步动作节点（Sync Action Node）

同步动作节点执行即时完成的任务，如数据处理、简单计算等。

```cpp
#include "behaviortree_cpp/action_node.h"

namespace rm_behavior {

class ExampleSyncAction : public BT::SyncActionNode {
public:
    ExampleSyncAction(const std::string& name, const BT::NodeConfig& config)
        : SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("input_data", "输入数据"),
            BT::OutputPort<std::string>("output_data", "输出数据")
        };
    }

    BT::NodeStatus tick() override {
        auto input = getInput<std::string>("input_data").value();
        // 处理逻辑
        std::string result = processData(input);
        setOutput("output_data", result);
        
        return BT::NodeStatus::SUCCESS;
    }

private:
    std::string processData(const std::string& input) {
        // 具体处理逻辑
        return input + "_processed";
    }
};

} // namespace rm_behavior
```

#### 3. ROS话题发布节点

用于发布ROS消息到指定话题，继承自 `BT::RosTopicPubNode`。

```cpp
#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace rm_behavior {

class VelocityPublisher : public BT::RosTopicPubNode<geometry_msgs::msg::Twist> {
public:
    VelocityPublisher(const std::string& name, const BT::NodeConfig& config,
                      const BT::RosNodeParams& params)
        : RosTopicPubNode(name, config, params) {}

    static BT::PortsList providedPorts() {
        return providedBasicPorts({
            BT::InputPort<double>("linear_x", 0.0, "线速度X"),
            BT::InputPort<double>("angular_z", 0.0, "角速度Z")
        });
    }

    bool setMessage(geometry_msgs::msg::Twist& msg) override {
        getInput("linear_x", msg.linear.x);
        getInput("angular_z", msg.angular.z);
        return true;
    }
};

} // namespace rm_behavior
```

#### 4. ROS动作节点

用于调用ROS 2 Action服务，如导航、机械臂控制等长时间任务。

```cpp
#include "behaviortree_ros2/bt_action_node.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

namespace rm_behavior {

class NavigationAction : public BT::RosActionNode<nav2_msgs::action::NavigateToPose> {
public:
    NavigationAction(const std::string& name, const BT::NodeConfig& config,
                     const BT::RosNodeParams& params)
        : BT::RosActionNode<nav2_msgs::action::NavigateToPose>(name, config, params) {}

    static BT::PortsList providedPorts() {
        return providedBasicPorts({
            BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "目标位置")
        });
    }

    bool setGoal(Goal& goal) override {
        getInput("goal", goal.pose);
        return true;
    }

    BT::NodeStatus onResultReceived(const WrappedResult& result) override {
        return result.result->result.result == 0 ? 
               BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

} // namespace rm_behavior
```

#### 5. 配置编译系统

在 `CMakeLists.txt` 中添加新节点的编译配置：

```cmake
# 添加新的条件节点
ament_auto_add_library(example_condition SHARED 
    src/plugins/condition/example_condition.cpp)
list(APPEND plugin_libs example_condition)

# 添加新的动作节点  
ament_auto_add_library(example_action SHARED 
    src/plugins/action/example_action.cpp)
list(APPEND plugin_libs example_action)
```

#### 6. 在行为树XML中使用

```xml
<root BTCPP_format="4">
    <BehaviorTree ID="MainTree">
        <Sequence>
            <ExampleCondition threshold="50" key_port="{@sensor_data}"/>
            <ExampleSyncAction input_data="test_data" output_data="{processed_result}"/>
            <VelocityPublisher linear_x="1.0" angular_z="0.5"
                             server_name="cmd_vel"/>
        </Sequence>
    </BehaviorTree>
</root>
```

## 注意事项

由于云台扫描模式相关代码较为混乱且尚未整理，暂时决定不将其纳入当前版本。若假期完成重构，会将新版功能提交至仓库。

## 鸣谢

感谢以下开源项目对行为树设计的启发：

- [BehaviorTree.CPP 官方文档](https://www.behaviortree.dev/)
- [北理莫斯科大学行为树实现方案](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_behavior)
- [国防科大君临行为决策模块方案](https://gitee.com/nepenthe886/JunLin_navigation2025)


QQ: 1822261285, Jquark
