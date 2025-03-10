#include "rm_behavior/plugins/action/nav2goal.hpp"

namespace rm_behavior {

  Nav2GoalAction::Nav2GoalAction(const std::string &name,
                                 const BT::NodeConfig &conf,
                                 const BT::RosNodeParams &params)
      : RosActionNode<nav2_msgs::action::NavigateToPose>(name, conf, params) {}

  bool Nav2GoalAction::setGoal(nav2_msgs::action::NavigateToPose::Goal &
                                   goal) {
    auto receive_goal = getInput<geometry_msgs::msg::PoseStamped>("goal");

    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = now();
    goal.pose.pose = receive_goal->pose;
    
    // 添加详细的目标点位日志
    RCLCPP_INFO(logger(), "[Nav2Goal] 导航目标点设置: x=%.2f, y=%.2f, z=%.2f, 方向四元数=(%.2f, %.2f, %.2f, %.2f)",
                receive_goal->pose.position.x,
                receive_goal->pose.position.y,
                receive_goal->pose.position.z,
                receive_goal->pose.orientation.x,
                receive_goal->pose.orientation.y,
                receive_goal->pose.orientation.z,
                receive_goal->pose.orientation.w);
    
    return true;
  }

//在命令行答应行为树信息
  BT::NodeStatus Nav2GoalAction::onResultReceived(const WrappedResult &wr) {
    switch (wr.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(logger(), "[Nav2Goal] 导航成功完成！");
      return BT::NodeStatus::SUCCESS;

    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(logger(), "[Nav2Goal] 导航被服务器中止");
      return BT::NodeStatus::FAILURE;

    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(logger(), "[Nav2Goal] 导航被取消");
      return BT::NodeStatus::FAILURE;

    default:
      RCLCPP_ERROR(logger(), "[Nav2Goal] 未知的导航结果代码: %d",
                   static_cast<int>(wr.code));
      return BT::NodeStatus::FAILURE;
    }
  }

  BT::NodeStatus Nav2GoalAction::onFeedback(
      const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback>
          feedback) {
    RCLCPP_INFO(logger(), "[Nav2Goal] 导航进行中: 剩余距离 = %.2f 米",
               feedback->distance_remaining);
    return BT::NodeStatus::RUNNING;
  }

  void Nav2GoalAction::onHalt() {
    RCLCPP_INFO(logger(), "Nav2GoalAction has been halted.");
  }

  BT::NodeStatus Nav2GoalAction::onFailure(BT::ActionNodeErrorCode error) {
    RCLCPP_ERROR(logger(), "Nav2GoalAction failed with error code: %d",
                 error);
    return BT::NodeStatus::FAILURE;
  }

  BT::PortsList Nav2GoalAction::providedPorts() {
    BT::PortsList additional_ports = {
        BT::InputPort<geometry_msgs::msg::PoseStamped>(
            "goal", "0;0;0",
            "Expected goal pose that send to nav2. Fill with format `x;y;yaw`"),
    };
    return providedBasicPorts(additional_ports);
  }
}

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior::Nav2GoalAction, "Nav2Goal");