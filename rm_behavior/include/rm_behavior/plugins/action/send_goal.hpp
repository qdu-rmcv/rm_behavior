#ifndef RM_BEHAVIOR__PLUGINS__ACTION__SEND_GOAL_HPP_
#define RM_BEHAVIOR__PLUGINS__ACTION__SEND_GOAL_HPP_

#include "behaviortree_ros2/bt_action_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

namespace rm_behavior
{

class SendGoalAction : public BT::RosActionNode<nav2_msgs::action::NavigateToPose>
{
public:
  SendGoalAction(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal_pose"),
      BT::InputPort<std::string>("action_name")};
  }

  bool setGoal(Goal & goal) override;

  void onHalt() override;

  BT::NodeStatus onResultReceived(const WrappedResult & wr) override;

  BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override;

  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;

private:
  rclcpp::Logger logger() { return node_->get_logger(); }
  rclcpp::Time now() { return node_->now(); }

protected:
  using RosActionNode<nav2_msgs::action::NavigateToPose>::goal_handle_;
};
}  // namespace rm_behavior

#endif  // RM_BEHAVIOR___PLUGINS__ACTION__SEND_GOAL_HPP_