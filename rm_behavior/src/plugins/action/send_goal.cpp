#include "rm_behavior/plugins/action/send_goal.hpp"
#include "rm_behavior/bt_conversions.hpp"

namespace rm_behavior
{

SendGoalAction::SendGoalAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: RosActionNode<nav2_msgs::action::NavigateToPose>(name, conf, params)
{
}

bool SendGoalAction::setGoal(nav2_msgs::action::NavigateToPose::Goal & goal)
{
  auto res = getInput<geometry_msgs::msg::PoseStamped>("goal_pose");
  if (!res) {
    throw BT::RuntimeError("error reading port [goal_pose]:", res.error());
  }
  goal.pose = res.value();
  goal.pose.header.frame_id = "map";
  goal.pose.header.stamp = now();

  // clang-format off
  std::cout << "Goal_pose: [ "
    << std::fixed << std::setprecision(1)
    << goal.pose.pose.position.x << ", "
    << goal.pose.pose.position.y << ", "
    << goal.pose.pose.position.z << ", "
    << goal.pose.pose.orientation.x << ", "
    << goal.pose.pose.orientation.y << ", "
    << goal.pose.pose.orientation.z << ", "
    << goal.pose.pose.orientation.w << " ]\n";
  // clang-format on

  return true;
}

void SendGoalAction::onHalt()
{
  // 检查 goal_handle_ 是否有效
  if (goal_handle_)
  {
      // 获取目标状态
      const auto status = goal_handle_->get_status();
      RCLCPP_INFO(
          logger(), 
          "[SendGoalAction] Canceling goal (current status: %d)", 
          status
      );
      // 根据目标状态决定是否取消
      if (status == 1 ||
          status == 2 )
      //GOAL_STATUS_ACCEPTED (1): 目标已被接受。   GOAL_STATUS_EXECUTING (2): 目标正在执行。
      {
          cancelGoal(); // 调用基类的取消方法
      }
      else
      {
          RCLCPP_WARN(
              logger(), 
              "[SendGoalAction] Goal already in terminal state, not canceling"
          );
      }
      // 清理 handle，避免悬空引用
      goal_handle_.reset();
  }

  RCLCPP_INFO(logger(), "SendGoalAction has been halted.");
}

BT::NodeStatus SendGoalAction::onResultReceived(const WrappedResult & wr)
{
  switch (wr.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(logger(), "Success!!!");
      return BT::NodeStatus::SUCCESS;
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_INFO(logger(), "Goal was aborted");
      return BT::NodeStatus::FAILURE;
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_INFO(logger(), "Goal was canceled");
      std::cout << "Goal was canceled" << '\n';
      return BT::NodeStatus::FAILURE;
      break;
    default:
      RCLCPP_INFO(logger(), "Unknown result code");
      return BT::NodeStatus::FAILURE;
      break;
  }
}

BT::NodeStatus SendGoalAction::onFeedback(
  const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> /*feedback*/)
{
  //std::cout << "Distance remaining: " << feedback->distance_remaining << '\n';
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SendGoalAction::onFailure(BT::ActionNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "SendGoalAction failed with error code: %d", error);
  return BT::NodeStatus::FAILURE;
}

}  // namespace rm_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior::SendGoalAction, "SendGoal");