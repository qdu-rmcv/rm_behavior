#include "rm_behavior/plugins/condition/status_check.hpp"

namespace rm_behavior {

status_check::status_check(const std::string &name,
                           const BT::NodeConfiguration &config)
    : BT::ConditionNode(name, config) {}

BT::PortsList status_check::providedPorts() {
  return {BT::InputPort<int>("sentry_hp"),
          BT::InputPort<int>("projectile_allowance_17mm"),
          BT::OutputPort<bool>("status_ok")};
}

BT::NodeStatus status_check::tick() {
  int sentry_hp;
  int projectile_allowance_17mm;
  bool status_ok = false;

  if (!getInput("sentry_hp", sentry_hp) ||
      !getInput("projectile_allowance_17mm", projectile_allowance_17mm)) {
    RCLCPP_ERROR(rclcpp::get_logger("status_check"),
                 "Failed to get input ports");
    return BT::NodeStatus::FAILURE;
  }

  // 简单示例：如果 sentry_hp 和 projectile_allowance_17mm 都大于
  // 0，则认为状态正常
  if (sentry_hp > 150 && projectile_allowance_17mm > 0) {
    status_ok = true;
  }

  setOutput("status_ok", status_ok);

  return status_ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

} // namespace rm_behavior