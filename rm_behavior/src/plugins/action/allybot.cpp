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

#include "rm_behavior/plugins/action/allybot.hpp"

#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker.hpp"

using nav2_util::declare_parameter_if_not_declared;
namespace rm_behavior
{

AllyBotAction::AllyBotAction(
  const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
: RosTopicPubNode(name, config, params), current_robot_select_(1)
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  declare_parameter_if_not_declared(node_, name + ".approach_radius", rclcpp::ParameterValue(1.5));
  declare_parameter_if_not_declared(node_, name + ".num_sectors", rclcpp::ParameterValue(36));
  declare_parameter_if_not_declared(node_, name + ".cost_threshold", rclcpp::ParameterValue(50));
  declare_parameter_if_not_declared(
    node_, name + ".robot_base_frame", rclcpp::ParameterValue("base_link"));
  declare_parameter_if_not_declared(
    node_, name + ".transform_tolerance", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node_, name + ".max_visualization_distance", rclcpp::ParameterValue(6.0));
  declare_parameter_if_not_declared(
    node_, name + ".marker_scale_base", rclcpp::ParameterValue(0.2));
  declare_parameter_if_not_declared(node_, name + ".visualize", rclcpp::ParameterValue(false));

  node_->get_parameter(name + ".approach_radius", params_.approach_radius);
  node_->get_parameter(name + ".num_sectors", params_.num_sectors);
  node_->get_parameter(name + ".cost_threshold", params_.cost_threshold);
  node_->get_parameter(name + ".robot_base_frame", params_.robot_base_frame);
  node_->get_parameter(name + ".transform_tolerance", params_.transform_tolerance);
  node_->get_parameter(name + ".max_visualization_distance", params_.max_visualization_distance);
  node_->get_parameter(name + ".marker_scale_base", params_.marker_scale_base);
  node_->get_parameter(name + ".visualize", params_.visualize);
}

BT::PortsList AllyBotAction::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<nav_msgs::msg::OccupancyGrid>(
      "costmap_port", "{@nav_globalCostmap}", "GlobalCostmap port on blackboard"),
    BT::InputPort<referee_interfaces::msg::AllyBot>(
      "allybot_port", "{@referee_allyBot}", "Ally bot information port on blackboard"),
    BT::InputPort<int>(
      "select", "1", "Robot select: 1=hero, 2=engineer, 3=standard_3, 4=standard_4"),
    BT::InputPort<double>(
      "trans_x", "0.0", "Translation in x-axis between coordinate frames"),
    BT::InputPort<double>(
      "trans_y", "0.0", "Translation in y-axis between coordinate frames"),
    BT::InputPort<double>(
      "trans_w", "0.0", "Rotation (yaw) between coordinate frames"),
    BT::OutputPort<PoseStamped>(
      "goal", "{ally_approach_pose}", "Goal pose to approach the selected ally robot"),
  });
}

bool AllyBotAction::setMessage(visualization_msgs::msg::MarkerArray & msg)
{
  auto global_costmap = getInput<nav_msgs::msg::OccupancyGrid>("costmap_port");
  auto ally_bot = getInput<referee_interfaces::msg::AllyBot>("allybot_port");
  auto select = getInput<int>("select");

  if (!global_costmap) {
    RCLCPP_ERROR(node_->get_logger(), "Missing required input: costmap_port");
    return false;
  }
  if (!ally_bot) {
    RCLCPP_ERROR(node_->get_logger(), "Missing required input: allybot_port");
    return false;
  }
  if (!select) {
    RCLCPP_WARN(node_->get_logger(), "No select value provided, using default (1=hero)");
    select = 1;
  }

  current_robot_select_ = select.value();
  
  Point ally_position = getSelectedRobotPosition(ally_bot.value(), current_robot_select_);
  std::string robot_name = getRobotName(current_robot_select_);
  
  // 检查位置是否有效
  if (ally_position.x == 0 && ally_position.y == 0 && ally_position.z == 0) {
    RCLCPP_WARN(node_->get_logger(), "Selected robot %s position is invalid", robot_name.c_str());
    return false;
  }

  std::vector<Point> candidates;
  std::vector<Point> feasible_points;
  PoseStamped robot_pose;

  // 准备坐标变换
  ally_on_costmap_.header.frame_id = global_costmap->header.frame_id;
  ally_on_costmap_.header.stamp = node_->now();
  ally_on_costmap_.point = ally_position;

  // 生成候选点
  candidates = generateCandidatePoints(ally_on_costmap_.point);

  // 筛选可行点
  feasible_points = filterFeasiblePoints(candidates, global_costmap.value());
  if (feasible_points.empty()) {
    RCLCPP_WARN(
      node_->get_logger(), "No feasible approach points found for %s", robot_name.c_str());
    return false;
  }

  // 获取机器人位置
  if (!nav2_util::getCurrentPose(
        robot_pose, *tf_buffer_, global_costmap->header.frame_id, params_.robot_base_frame,
        params_.transform_tolerance)) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to get robot pose");
    return false;
  }

  // 选择最佳点
  const auto best_point = selectBestPoint(feasible_points, robot_pose.pose.position);

  // 创建接近姿态
  const auto approach_pose = createApproachPose(best_point, ally_on_costmap_);
  setOutput("goal", approach_pose);

  // 创建可视化
  if (params_.visualize) {
    createVisualizationMarkers(
      msg, ally_on_costmap_.point, candidates, feasible_points, robot_pose.pose.position,
      global_costmap.value(), robot_name);
  }
  
  return true;
}

Point AllyBotAction::getSelectedRobotPosition(
  const referee_interfaces::msg::AllyBot & ally_bot, int select)
{
  Point raw_position;

  switch (select) {
    case 1:  // hero
      raw_position = ally_bot.hero_position;
      break;
    case 2:  // engineer
      raw_position = ally_bot.engineer_position;
      break;
    case 3:  // standard_3
      raw_position = ally_bot.standard_3_position;
      break;
    case 4:  // standard_4
      raw_position = ally_bot.standard_4_position;
      break;
    default:
      RCLCPP_ERROR(
        node_->get_logger(), "Invalid robot select value: %d (valid: 1-4)", select);
      raw_position.x = raw_position.y = raw_position.z = 0.0;
      return raw_position;
  }

  // 调用 transformCoordinates 对坐标进行转换
  return transformCoordinates(raw_position);
}

std::string AllyBotAction::getRobotName(int select)
{
  switch (select) {
    case 1:
      return "Hero";
    case 2:
      return "Engineer";
    case 3:
      return "Standard_3";
    case 4:
      return "Standard_4";
    default:
      return "Unknown";
  }
}

std::vector<Point> AllyBotAction::generateCandidatePoints(const Point & ally_position)
{
  std::vector<Point> candidates;
  candidates.reserve(params_.num_sectors);

  for (int i =.0; i < params_.num_sectors; ++i) {
    const double angle = i * 2 * M_PI / params_.num_sectors;
    Point p;
    p.x = ally_position.x + params_.approach_radius * cos(angle);
    p.y = ally_position.y + params_.approach_radius * sin(angle);
    p.z = 0.0;
    candidates.push_back(p);
  }
  return candidates;
}

std::vector<Point> AllyBotAction::filterFeasiblePoints(
  const std::vector<Point> & candidates, const nav_msgs::msg::OccupancyGrid & costmap)
{
  std::vector<Point> feasible_points;
  const auto & info = costmap.info;

  for (const auto & p : candidates) {
    const int cell_x = static_cast<int>((p.x - info.origin.position.x) / info.resolution);
    const int cell_y = static_cast<int>((p.y - info.origin.position.y) / info.resolution);

    if (
      cell_x < 0 || cell_x >= static_cast<int>(info.width) || cell_y < 0 ||
      cell_y >= static_cast<int>(info.height)) {
      continue;
    }

    const int index = cell_y * info.width + cell_x;
    const int8_t cost = costmap.data[index];
    if (cost >= 0 && cost <= params_.cost_threshold) {
      feasible_points.push_back(p);
    }
  }
  return feasible_points;
}

Point AllyBotAction::selectBestPoint(
  const std::vector<Point> & feasible_points, const Point & robot_position)
{
  auto compare = [&](const auto & a, const auto & b) {
    const double dx1 = a.x - robot_position.x;
    const double dy1 = a.y - robot_position.y;
    const double dx2 = b.x - robot_position.x;
    const double dy2 = b.y - robot_position.y;
    return (dx1 * dx1 + dy1 * dy1) < (dx2 * dx2 + dy2 * dy2);
  };
  return *std::min_element(feasible_points.begin(), feasible_points.end(), compare);
}

PoseStamped AllyBotAction::createApproachPose(
  const Point & approach_point, const PointStamped & ally_position)
{
  PoseStamped pose;
  pose.header.frame_id = ally_position.header.frame_id;
  pose.header.stamp = node_->now();
  pose.pose.position = approach_point;

  // 朝向队友机器人的方向
  const double dx = ally_position.point.x - approach_point.x;
  const double dy = ally_position.point.y - approach_point.y;
  tf2::Quaternion q;
  q.setRPY(0, 0, atan2(dy, dx));
  pose.pose.orientation = tf2::toMsg(q);
  return pose;
}

bool AllyBotAction::transformPoseInTargetFrame(
  const PointStamped & input_pose, PointStamped & transformed_pose, tf2_ros::Buffer & tf_buffer,
  const std::string target_frame, const double transform_timeout)
{
  static rclcpp::Logger logger = rclcpp::get_logger("transformPoseInTargetFrame");

  try {
    transformed_pose =
      tf_buffer.transform(input_pose, target_frame, tf2::durationFromSec(transform_timeout));
    return true;
  } catch (tf2::LookupException & ex) {
    RCLCPP_ERROR(logger, "No Transform available Error looking up target frame: %s\n", ex.what());
  } catch (tf2::ConnectivityException & ex) {
    RCLCPP_ERROR(logger, "Connectivity Error looking up target frame: %s\n", ex.what());
  } catch (tf2::ExtrapolationException & ex) {
    RCLCPP_ERROR(logger, "Extrapolation Error looking up target frame: %s\n", ex.what());
  } catch (tf2::TimeoutException & ex) {
    RCLCPP_ERROR(logger, "Transform timeout with tolerance: %.4f", transform_timeout);
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(
      logger, "Failed to transform from %s to %s", input_pose.header.frame_id.c_str(),
      target_frame.c_str());
  }

  return false;
}

void AllyBotAction::createVisualizationMarkers(
  visualization_msgs::msg::MarkerArray & msg, const Point & ally_position,
  const std::vector<Point> & candidates, const std::vector<Point> & feasible_points,
  const Point & robot_position, const nav_msgs::msg::OccupancyGrid & costmap,
  const std::string & robot_name)
{
  msg.markers.clear();

  // 队友机器人标记
  visualization_msgs::msg::Marker ally_marker;
  ally_marker.header.frame_id = costmap.header.frame_id;
  ally_marker.ns = "ally_" + robot_name;
  ally_marker.id = 0;
  ally_marker.type = visualization_msgs::msg::Marker::SPHERE;
  ally_marker.pose.position = ally_position;
  ally_marker.scale.x = ally_marker.scale.y = ally_marker.scale.z = 0.3;

  ally_marker.color.g = 1.0;
  ally_marker.color.a = 1.0;
  msg.markers.push_back(ally_marker);

  // 最佳接近点标记
  const auto best_point = selectBestPoint(feasible_points, robot_position);
  visualization_msgs::msg::Marker best_marker;
  best_marker.header.frame_id = costmap.header.frame_id;
  best_marker.ns = "best_" + robot_name;
  best_marker.id = 0;
  best_marker.type = visualization_msgs::msg::Marker::SPHERE;
  best_marker.pose.position = best_point;
  best_marker.scale.x = best_marker.scale.y = best_marker.scale.z = 0.4;
  best_marker.color.g = 1.0;
  best_marker.color.a = 1.0;
  msg.markers.push_back(best_marker);

  // 候选点标记
  const auto & info = costmap.info;
  for (size_t i = 0; i < candidates.size(); ++i) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = costmap.header.frame_id;
    marker.ns = "candidates_" + robot_name;
    marker.id = i;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.pose.position = candidates[i];

    const int cell_x = (candidates[i].x - info.origin.position.x) / info.resolution;
    const int cell_y = (candidates[i].y - info.origin.position.y) / info.resolution;
    int8_t cost = -1;
    if (
      cell_x >= 0 && cell_x < static_cast<int>(info.width) && cell_y >= 0 &&
      cell_y < static_cast<int>(info.height)) {
      cost = costmap.data[cell_y * info.width + cell_x];
    }

    marker.scale.x = marker.scale.y = marker.scale.z =
      params_.marker_scale_base + (cost / 100.0) * 0.3;

    const double distance =
      std::hypot(candidates[i].x - robot_position.x, candidates[i].y - robot_position.y);
    const float alpha =
      0.5 + 0.5 * (1.0 - std::min(distance / params_.max_visualization_distance, 1.0));

    marker.color.r = 1.0;
    marker.color.a = alpha;

    // 检查可行性
    const bool is_feasible = std::any_of(
      feasible_points.begin(), feasible_points.end(),
      [&](const auto & p) { return p.x == candidates[i].x && p.y == candidates[i].y; });

    if (!is_feasible) {
      marker.color.a *= 0.3;
    }

    msg.markers.push_back(marker);
  }

  // 范围圆圈
  visualization_msgs::msg::Marker circle;
  circle.header.frame_id = costmap.header.frame_id;
  circle.ns = "range_" + robot_name;
  circle.id = 0;
  circle.type = visualization_msgs::msg::Marker::LINE_STRIP;
  circle.pose.position.z = 0.05;
  circle.scale.x = 0.05;

  circle.color.g = 1.0;
  circle.color.a = 0.5;

  constexpr int circle_points = 36;
  for (int i = 0; i <= circle_points; ++i) {
    const double angle = i * 2 * M_PI / circle_points;
    Point p;
    p.x = ally_position.x + params_.approach_radius * cos(angle);
    p.y = ally_position.y + params_.approach_radius * sin(angle);
    circle.points.push_back(p);
  }
  msg.markers.push_back(circle);
}

Point AllyBotAction::transformCoordinates(const Point &input_point)
{
  Point transformed_point;

  auto trans_x_opt = getInput<double>("trans_x");
  auto trans_y_opt = getInput<double>("trans_y");
  auto trans_w_opt = getInput<double>("trans_w");

  if (!trans_x_opt || !trans_y_opt || !trans_w_opt) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to get transformation parameters from input ports");
    return input_point;
  }

  double trans_x = trans_x_opt.value();
  double trans_y = trans_y_opt.value();
  double trans_w = trans_w_opt.value();

  // 旋转变换：将输入点从目标坐标系转换到当前坐标系
  double cos_w = cos(-trans_w); // 逆旋转
  double sin_w = sin(-trans_w);
  double x_rotated = cos_w * input_point.x - sin_w * input_point.y;
  double y_rotated = sin_w * input_point.x + cos_w * input_point.y;

  // 平移变换：减去目标坐标系原点的平移量
  transformed_point.x = x_rotated - trans_x;
  transformed_point.y = y_rotated - trans_y;
  transformed_point.z = input_point.z; // z 坐标保持不变

  return transformed_point;
}

}  // namespace rm_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior::AllyBotAction, "AllyBot");