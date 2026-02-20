#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include "ur_msgs/srv/set_io.hpp"

#include <Eigen/Geometry>
#include <thread>
#include <chrono>
#include <future>
#include <cmath>
#include <vector>
#include <string>


using namespace std::chrono_literals;

// ============================================================
// Helper: sleep
// ============================================================
static void sleep_ms(int ms)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

// ============================================================
// Helper: joint-space plan and execute
// ============================================================
static bool planAndExecute(
  moveit::planning_interface::MoveGroupInterface& mgi,
  rclcpp::Logger logger)
{
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (mgi.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Planning failed for group '%s'", mgi.getName().c_str());
    return false;
  }
  if (mgi.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Execution failed for group '%s'", mgi.getName().c_str());
    return false;
  }
  return true;
}

// ============================================================
// Helper: Cartesian plan and execute between two poses
//
// Generates evenly spaced waypoints between start_pose and
// end_pose using lerp (position) and slerp (orientation),
// then plans and executes a straight-line Cartesian path.
// ============================================================
static bool planAndExecuteCartesianBetween(
  moveit::planning_interface::MoveGroupInterface& mgi,
  const geometry_msgs::msg::Pose& start_pose,
  const geometry_msgs::msg::Pose& end_pose,
  int num_waypoints,
  rclcpp::Logger logger,
  double eef_step = 0.005,
  double jump_threshold = 0.0)
{
  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(start_pose);

  Eigen::Vector3d p_start(
    start_pose.position.x,
    start_pose.position.y,
    start_pose.position.z);
  Eigen::Vector3d p_end(
    end_pose.position.x,
    end_pose.position.y,
    end_pose.position.z);

  Eigen::Quaterniond q_start(
    start_pose.orientation.w,
    start_pose.orientation.x,
    start_pose.orientation.y,
    start_pose.orientation.z);
  Eigen::Quaterniond q_end(
    end_pose.orientation.w,
    end_pose.orientation.x,
    end_pose.orientation.y,
    end_pose.orientation.z);
  q_start.normalize();
  q_end.normalize();

  for (int i = 1; i <= num_waypoints; ++i) {
    double t = static_cast<double>(i) / static_cast<double>(num_waypoints);

    Eigen::Vector3d p_interp    = (1.0 - t) * p_start + t * p_end;  // lerp
    Eigen::Quaterniond q_interp = q_start.slerp(t, q_end);           // slerp

    geometry_msgs::msg::Pose wp;
    wp.position.x    = p_interp.x();
    wp.position.y    = p_interp.y();
    wp.position.z    = p_interp.z();
    wp.orientation.w = q_interp.w();
    wp.orientation.x = q_interp.x();
    wp.orientation.y = q_interp.y();
    wp.orientation.z = q_interp.z();

    waypoints.push_back(wp);
  }

  moveit_msgs::msg::RobotTrajectory trajectory;
  double fraction = mgi.computeCartesianPath(
    waypoints, eef_step, jump_threshold, trajectory);

  if (fraction < 0.95) {
    RCLCPP_ERROR(logger, "Cartesian path only %.2f%% achieved for '%s'",
                 fraction * 100.0, mgi.getName().c_str());
    return false;
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory_ = trajectory;

  if (mgi.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Cartesian execution failed for '%s'", mgi.getName().c_str());
    return false;
  }
  return true;
}


static geometry_msgs::msg::Pose poseFromURPendant(
  double x, double y, double z,
  double rx, double ry, double rz)
{
  // UR pendant RX/RY/RZ is a rotation vector (axis-angle):
  // direction = axis of rotation, magnitude = angle in radians
  Eigen::Vector3d rvec(rx, ry, rz);
  double angle = rvec.norm();

  Eigen::Quaterniond q;
  if (angle < 1e-10) {
    q = Eigen::Quaterniond::Identity();
  } else {
    q = Eigen::Quaterniond(Eigen::AngleAxisd(angle, rvec.normalized()));
  }
  q.normalize();

  geometry_msgs::msg::Pose pose;
  pose.position.x    = x;
  pose.position.y    = y;
  pose.position.z    = z;
  pose.orientation.w = q.w();
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  return pose;
}

static void gripper_on(
  rclcpp::Node::SharedPtr node,
  rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr io_client,
  int pin)
{
  auto request = std::make_shared<ur_msgs::srv::SetIO::Request>();
  request->fun   = request->FUN_SET_DIGITAL_OUT;
  request->pin   = pin;
  request->state = request->STATE_ON;
  (void)io_client->async_send_request(request);
  RCLCPP_INFO(node->get_logger(), "Gripper ON (pin %d)", pin);
}

static void gripper_off(
  rclcpp::Node::SharedPtr node,
  rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr io_client,
  int pin)
{
  auto request = std::make_shared<ur_msgs::srv::SetIO::Request>();
  request->fun   = request->FUN_SET_DIGITAL_OUT;
  request->pin   = pin;
  request->state = request->STATE_OFF;
  (void)io_client->async_send_request(request);
  RCLCPP_INFO(node->get_logger(), "Gripper OFF (pin %d)", pin);
}
// ============================================================
// Main
// ============================================================
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("handover_demo");
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  std::thread spinner([&exec]() { exec.spin(); });
  spinner.detach();

  // ---- Group and link names ----
  const std::string left_group_name  = node->declare_parameter<std::string>("left_group", "left_ur16e");
  const std::string right_group_name = node->declare_parameter<std::string>("right_group", "right_ur16e");
  const std::string left_ee_link     = node->declare_parameter<std::string>("left_ee_link", "left_tool0");
  const std::string right_ee_link    = node->declare_parameter<std::string>("right_ee_link", "right_tool0");

  // ---- Named states from SRDF (joint-space moves) ----
  const std::string left_take_named        = node->declare_parameter<std::string>("left_take_named","left_receiving_position");
  const std::string left_present_named     = node->declare_parameter<std::string>("left_present_named","left_handover_position");
  const std::string right_handover_named   = node->declare_parameter<std::string>("right_handover_named","right_prehandover_position");
  const std::string right_handover_named_2 = node->declare_parameter<std::string>("right_handover_named_2", "right_prehandover_position_2");
  const std::string right_handover_named_3 = node->declare_parameter<std::string>("right_handover_named_3", "right_prehandover_position_3");
  const std::string right_final_named      = node->declare_parameter<std::string>("right_final_named", "right_lower_object_position");

  // ---- Cartesian waypoint count ----
  const int cartesian_waypoints = node->declare_parameter<int>("cartesian_waypoints", 20);

  // ---- Cartesian poses from UR pendant values (loaded from yaml) ----
  geometry_msgs::msg::Pose left_waypoint3 = poseFromURPendant(
    node->declare_parameter<double>("left_waypoint3_x",  0.0),
    node->declare_parameter<double>("left_waypoint3_y",  0.0),
    node->declare_parameter<double>("left_waypoint3_z",  0.0),
    node->declare_parameter<double>("left_waypoint3_rx", 0.0),
    node->declare_parameter<double>("left_waypoint3_ry", 0.0),
    node->declare_parameter<double>("left_waypoint3_rz", 0.0));

  geometry_msgs::msg::Pose left_waypoint4 = poseFromURPendant(
    node->declare_parameter<double>("left_waypoint4_x",  0.0),
    node->declare_parameter<double>("left_waypoint4_y",  0.0),
    node->declare_parameter<double>("left_waypoint4_z",  0.0),
    node->declare_parameter<double>("left_waypoint4_rx", 0.0),
    node->declare_parameter<double>("left_waypoint4_ry", 0.0),
    node->declare_parameter<double>("left_waypoint4_rz", 0.0));

  geometry_msgs::msg::Pose right_waypoint3 = poseFromURPendant(
    node->declare_parameter<double>("right_waypoint3_x",  0.0),
    node->declare_parameter<double>("right_waypoint3_y",  0.0),
    node->declare_parameter<double>("right_waypoint3_z",  0.0),
    node->declare_parameter<double>("right_waypoint3_rx", 0.0),
    node->declare_parameter<double>("right_waypoint3_ry", 0.0),
    node->declare_parameter<double>("right_waypoint3_rz", 0.0));

  geometry_msgs::msg::Pose right_waypoint4 = poseFromURPendant(
    node->declare_parameter<double>("right_waypoint4_x",  0.0),
    node->declare_parameter<double>("right_waypoint4_y",  0.0),
    node->declare_parameter<double>("right_waypoint4_z",  0.0),
    node->declare_parameter<double>("right_waypoint4_rx", 0.0),
    node->declare_parameter<double>("right_waypoint4_ry", 0.0),
    node->declare_parameter<double>("right_waypoint4_rz", 0.0));

  //  Gripper IO pins 
  int pin_out_left = 13;
  int pin_out_right = 12;
    

  // ---- IO clients ----
  auto left_io  = node->create_client<ur_msgs::srv::SetIO>("left_io_and_status_controller/set_io");
  auto right_io = node->create_client<ur_msgs::srv::SetIO>("right_io_and_status_controller/set_io");

  // ---- MoveIt interfaces ----
  moveit::planning_interface::MoveGroupInterface left_mgi(node,  left_group_name);
  moveit::planning_interface::MoveGroupInterface right_mgi(node, right_group_name);

  left_mgi.setPlanningTime(20.0);
  right_mgi.setPlanningTime(20.0);
  left_mgi.allowReplanning(true);
  right_mgi.allowReplanning(true);

  left_mgi.setMaxVelocityScalingFactor(0.3);   // 0–1, lower = smoother/slower
  left_mgi.setMaxAccelerationScalingFactor(0.2);
  right_mgi.setMaxVelocityScalingFactor(0.3);
  right_mgi.setMaxAccelerationScalingFactor(0.2);

  sleep_ms(800);

  // ============================================================
  // Step 1: Left arm moves to receiving position (joint-space)
  // ============================================================
  RCLCPP_INFO(node->get_logger(), "Step 1: LEFT -> receiving position");
  left_mgi.setNamedTarget(left_take_named);
  if (!planAndExecute(left_mgi, node->get_logger())) {
    RCLCPP_ERROR(node->get_logger(), "Failed Step 1"); rclcpp::shutdown(); return 1;
  }

  // ============================================================
  // Step 2: Left arm moves to handover position (joint-space)
  // ============================================================
  RCLCPP_INFO(node->get_logger(), "Step 2: LEFT -> handover position");
  left_mgi.setNamedTarget(left_present_named);
  if (!planAndExecute(left_mgi, node->get_logger())) {
    RCLCPP_ERROR(node->get_logger(), "Failed Step 2"); rclcpp::shutdown(); return 1;
  }

  // ============================================================
  // Step 3: Right arm moves to pre-handover position 2 (joint-space)
  // ============================================================
  RCLCPP_INFO(node->get_logger(), "Step 3: RIGHT -> pre-handover position 2");
  right_mgi.setNamedTarget(right_handover_named_2);
  if (!planAndExecute(right_mgi, node->get_logger())) {
    RCLCPP_ERROR(node->get_logger(), "Failed Step 3"); rclcpp::shutdown(); return 1;
  }

  // ============================================================
  // Step 4: Right arm moves to pre-handover position 3 (joint-space)
  // ============================================================
  RCLCPP_INFO(node->get_logger(), "Step 4: RIGHT -> pre-handover position 3");
  right_mgi.setNamedTarget(right_handover_named_3);
  if (!planAndExecute(right_mgi, node->get_logger())) {
    RCLCPP_ERROR(node->get_logger(), "Failed Step 4"); rclcpp::shutdown(); return 1;
  }

  // ============================================================
  // Step 5b: Right arm Cartesian approach to handover waypoint
  // ============================================================
  RCLCPP_INFO(node->get_logger(), "Step 5b: RIGHT Cartesian -> handover waypoint");
  geometry_msgs::msg::Pose right_current = right_mgi.getCurrentPose(right_ee_link).pose;
  if (!planAndExecuteCartesianBetween(right_mgi, right_current, right_waypoint3,
        cartesian_waypoints, node->get_logger())) {
    RCLCPP_ERROR(node->get_logger(), "Failed Step 5b"); rclcpp::shutdown(); return 1;
  }

  sleep_ms(3000);

  // ============================================================
  // Step 6: Right gripper closes (grips object)
  // ============================================================
  RCLCPP_INFO(node->get_logger(), "Step 6: Closing right gripper");
  gripper_on(node, right_io, pin_out_right);
  sleep_ms(5000);

  // Step 7: Left gripper releases object
  RCLCPP_INFO(node->get_logger(), "Step 7: Opening left gripper");
  gripper_off(node, left_io, pin_out_left);
  sleep_ms(500);

 // Step 8: Left arm Cartesian retract away from handover zone
  RCLCPP_INFO(node->get_logger(), "Step 8: LEFT Cartesian -> retract waypoint");
  geometry_msgs::msg::Pose left_current = left_mgi.getCurrentPose(left_ee_link).pose;
  if (!planAndExecuteCartesianBetween(left_mgi, left_current, left_waypoint3,
        cartesian_waypoints, node->get_logger())) {
    RCLCPP_ERROR(node->get_logger(), "Failed Step 8"); rclcpp::shutdown(); return 1;
  }

  // Step 9: Right arm Cartesian move to placement waypoint
  RCLCPP_INFO(node->get_logger(), "Step 9: RIGHT Cartesian -> placement waypoint");
  right_current = right_mgi.getCurrentPose(right_ee_link).pose;
  if (!planAndExecuteCartesianBetween(right_mgi, right_current, right_waypoint4,
        cartesian_waypoints, node->get_logger())) {
    RCLCPP_ERROR(node->get_logger(), "Failed Step 9"); rclcpp::shutdown(); return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Step 9b: LEFT Cartesian -> retract waypoint4");
  left_current = left_mgi.getCurrentPose(left_ee_link).pose;
  if (!planAndExecuteCartesianBetween(left_mgi, left_current, left_waypoint4,
        cartesian_waypoints, node->get_logger())) {
    RCLCPP_ERROR(node->get_logger(), "Failed Step 9b"); rclcpp::shutdown(); return 1;
  }

  // Step 10: Right arm moves to final release position (joint-space)
  RCLCPP_INFO(node->get_logger(), "Step 10: RIGHT -> final release position");
  right_mgi.setNamedTarget(right_final_named);
  if (!planAndExecute(right_mgi, node->get_logger())) {
    RCLCPP_ERROR(node->get_logger(), "Failed Step 10"); rclcpp::shutdown(); return 1;
  }
  sleep_ms(200);
  // ============================================================
  // Step 11: Right gripper releases object into box
  // ============================================================
  RCLCPP_INFO(node->get_logger(), "Step 11: Opening right gripper — releasing object");
  sleep_ms(500);
  gripper_off(node, right_io, pin_out_right);
  sleep_ms(5000);
  RCLCPP_INFO(node->get_logger(), "Handover complete.");
  rclcpp::shutdown();
  return 0;
}