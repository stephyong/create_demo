#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <geometry_msgs/msg/pose.hpp>
#include "ur_msgs/srv/set_io.hpp"

#include <Eigen/Geometry>
#include <thread>
#include <chrono>
#include <future>

using namespace std::chrono_literals;

// ============================================================
// Helpers
// ============================================================

static void sleep_ms(int ms)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

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

// Moves the end-effector in a straight line through a series of evenly-spaced
// waypoints between start_pose and end_pose.
//
// Inputs:
//   mgi            : the arm to move
//   start_pose     : where to begin the Cartesian path (usually current EEF pose)
//   end_pose       : the final target pose
//   num_waypoints  : how many intermediate points to interpolate (more = smoother)
//   logger         : for error logging
//   eef_step       : resolution of each step along the path in metres
//   jump_threshold : 0.0 disables joint-space jump checking
//
// Returns true if path was fully planned and executed, false otherwise.

static bool planAndExecuteCartesianBetween(
  moveit::planning_interface::MoveGroupInterface& mgi,
  const geometry_msgs::msg::Pose& start_pose,
  const geometry_msgs::msg::Pose& end_pose,
  int num_waypoints,
  rclcpp::Logger logger,
  double eef_step = 0.01,
  double jump_threshold = 0.0)
{
  std::vector<geometry_msgs::msg::Pose> waypoints;

  // Always include the start pose as the first waypoint
  waypoints.push_back(start_pose);

  // Extract start and end as Eigen objects for interpolation
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

  // Generate intermediate waypoints by interpolating between start and end.
  // t goes from 0 to 1 across num_waypoints steps.
  // Position is linearly interpolated (lerp).
  // Orientation is spherically interpolated (slerp) — smoothly rotates
  // between the two quaternions rather than jumping.
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

  // Plan the Cartesian path through all waypoints
  static geometry_msgs::msg::Pose poseFromURPendant(
  double x, double y, double z,
  double rx_deg, double ry_deg, double rz_deg)
{
  double rx = rx_deg * M_PI / 180.0;
  double ry = ry_deg * M_PI / 180.0;
  double rz = rz_deg * M_PI / 180.0;

  Eigen::AngleAxisd roll (rx, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitch(ry, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yaw  (rz, Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond q = yaw * pitch * roll;
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

static bool set_gripper(
  rclcpp::Node::SharedPtr node,
  rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr client,
  uint8_t pin,
  bool state,
  const std::string& label)
{
  if (!client->wait_for_service(5s)) {
    RCLCPP_ERROR(node->get_logger(), "IO service not available for %s", label.c_str());
    return false;
  }

  auto req   = std::make_shared<ur_msgs::srv::SetIO::Request>();
  req->fun   = req->FUN_SET_DIGITAL_OUT;
  req->pin   = pin;
  req->state = state ? req->STATE_ON : req->STATE_OFF;

  auto future = client->async_send_request(req);
  if (future.wait_for(5s) != std::future_status::ready) {
    RCLCPP_ERROR(node->get_logger(), "IO service timed out for %s", label.c_str());
    return false;
  }

  RCLCPP_INFO(node->get_logger(), "Gripper %s: %s",
              label.c_str(), state ? "CLOSED" : "OPEN");
  return true;
}

// Offset a pose along its own tool +Y axis by `distance` metres.
// Used to compute the Cartesian approach: step back from the grasp point
// along the gripper's own Y axis, then move straight in.
static geometry_msgs::msg::Pose offsetAlongToolY(
  const geometry_msgs::msg::Pose& pose,
  double distance)
{
  geometry_msgs::msg::Pose result = pose;

  Eigen::Quaterniond q(
    pose.orientation.w,
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z);
  q.normalize();

  Eigen::Vector3d y_world = q * Eigen::Vector3d::UnitY();
  Eigen::Vector3d p(pose.position.x, pose.position.y, pose.position.z);
  Eigen::Vector3d p_offset = p + distance * y_world;

  result.position.x = p_offset.x();
  result.position.y = p_offset.y();
  result.position.z = p_offset.z();
  return result;
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

  // ---- Parameters ----
  const std::string left_group_name   = node->declare_parameter<std::string>("left_group",   "left_ur16e");
  const std::string right_group_name  = node->declare_parameter<std::string>("right_group",  "right_ur16e");
  const std::string left_ee_link      = node->declare_parameter<std::string>("left_ee_link",  "left_tool0");
  const std::string right_ee_link     = node->declare_parameter<std::string>("right_ee_link", "right_tool0");

  // Named states defined in your SRDF
  const std::string left_take_named      = node->declare_parameter<std::string>("left_take_named", "left_receiving_position");
  const std::string left_present_named   = node->declare_parameter<std::string>("left_present_named","left_handover_position");
  const std::string left_after_named      = node->declare_parameter<std::string>("left_after_named", "left_posthandover_position");
  const std::string left_after_named2   = node->declare_parameter<std::string>("left_after_named2", "left_posthandover_position2");
  
  
  const std::string right_handover_named = node->declare_parameter<std::string>("right_handover_named", "right_prehandover_position");
  const std::string right_handover_named_2 = node->declare_parameter<std::string>("right_handover_named_2", "right_prehandover_position_2");
  const std::string right_handover_named_3 = node->declare_parameter<std::string>("right_handover_named_3", "right_prehandover_position_3");
  const std::string right_handover = node->declare_parameter<std::string>("right_handover","right_handover_position");
  const std::string right_posthandover = node->declare_parameter<std::string>("right_posthandover", "right_posthandover_position");
  const std::string right_final_named    = node->declare_parameter<std::string>("right_final_named", "right_lower_object_position");




  const int cartesian_waypoints = node->declare_parameter<int>("cartesian_waypoints", 10);
  geometry_msgs::msg::Pose left_waypoint3 = poseFromURPendant(
    node->declare_parameter<double>("left_waypoint3_x",  0.0),
    node->declare_parameter<double>("left_waypoint3_y",  0.0),
    node->declare_parameter<double>("left_waypoint3_z",  0.0),
    node->declare_parameter<double>("left_waypoint3_rx", 0.0),
    node->declare_parameter<double>("left_waypoint3_ry", 0.0),
    node->declare_parameter<double>("left_waypoint3_rz", 0.0));




  // Gripper IO pins — match your UR controller wiring
  const uint8_t left_gripper_pin  = static_cast<uint8_t>(node->declare_parameter<int>("left_gripper_pin",  13));
  const uint8_t right_gripper_pin = static_cast<uint8_t>(node->declare_parameter<int>("right_gripper_pin", 12));



  // ---- IO clients ----
  auto left_io  = node->create_client<ur_msgs::srv::SetIO>("left_io_and_status_controller/set_io");
  auto right_io = node->create_client<ur_msgs::srv::SetIO>("right_io_and_status_controller/set_io");

  // ---- MoveIt interfaces ----
  moveit::planning_interface::MoveGroupInterface left_mgi(node,  left_group_name);
  moveit::planning_interface::MoveGroupInterface right_mgi(node, right_group_name);

  left_mgi.setPlanningTime(15.0);
  right_mgi.setPlanningTime(15.0);
  left_mgi.allowReplanning(true);
  right_mgi.allowReplanning(true);

  sleep_ms(800);

  // ============================================================
  // Step 1: Left arm moves to receiving position (from human)
  // ============================================================
  RCLCPP_INFO(node->get_logger(), "Step 1: LEFT -> receiving position");
  left_mgi.setNamedTarget(left_take_named);
  if (!planAndExecute(left_mgi, node->get_logger())) {
    RCLCPP_ERROR(node->get_logger(), "Failed Step 1"); rclcpp::shutdown(); return 1;
  }

  // ============================================================
  // Step 2: Left arm moves to handover position (presents object)
  // ============================================================
  RCLCPP_INFO(node->get_logger(), "Step 2: LEFT -> handover position");
  left_mgi.setNamedTarget(left_present_named);
  if (!planAndExecute(left_mgi, node->get_logger())) {
    RCLCPP_ERROR(node->get_logger(), "Failed Step 2"); rclcpp::shutdown(); return 1;
  }

  // ============================================================
  // Step 3: Right arm moves to hardcoded handover position.
  // This named state is defined in your SRDF and positions right
  // gripper facing left gripper, offset along Y by the distance
  // you measured, with the correct orientation already set.
  // ============================================================
  RCLCPP_INFO(node->get_logger(), "Step 3: RIGHT -> handover position");
  right_mgi.setNamedTarget(right_handover_named);
  if (!planAndExecute(right_mgi, node->get_logger())) {
    RCLCPP_ERROR(node->get_logger(), "Failed Step 3"); rclcpp::shutdown(); return 1;
  }
  right_mgi.setNamedTarget(right_handover_named_2);
  if (!planAndExecute(right_mgi, node->get_logger())) {
    RCLCPP_ERROR(node->get_logger(), "Failed Step 3a"); rclcpp::shutdown(); return 1;
  }
  right_mgi.setNamedTarget(right_handover_named_3);
  if (!planAndExecute(right_mgi, node->get_logger())) {
    RCLCPP_ERROR(node->get_logger(), "Failed Step 3b"); rclcpp::shutdown(); return 1;
  }
  right_mgi.setNamedTarget(right_handover);
  if (!planAndExecuteCartesian(right_mgi, node->get_logger())) {
    RCLCPP_ERROR(node->get_logger(), "Failed Step 3c"); rclcpp::shutdown(); return 1;
  }

  // ============================================================
  // Step 5: Cartesian approach — move 4cm along right tool +Y.
  // Right gripper is now at the hardcoded handover position.
  // We offset from its current pose along its own Y axis to
  // compute the grasp target, then move there in a straight line.
  // ============================================================
  RCLCPP_INFO(node->get_logger(), "Step 5: RIGHT Cartesian approach (%.0f mm along tool Y)",
              cartesian_approach_distance * 1000.0);

  geometry_msgs::msg::Pose right_current = right_mgi.getCurrentPose(right_ee_link).pose;
  geometry_msgs::msg::Pose right_grasp   = offsetAlongToolY(right_current, cartesian_approach_distance);

  RCLCPP_INFO(node->get_logger(),
    "Grasp target: x=%.3f y=%.3f z=%.3f",
    right_grasp.position.x, right_grasp.position.y, right_grasp.position.z);

  if (!planAndExecuteCartesian(right_mgi, right_grasp, node->get_logger())) {
    RCLCPP_ERROR(node->get_logger(), "Failed Step 5"); rclcpp::shutdown(); return 1;
  }

  // ============================================================
  // Step 6: Right gripper closes (grips object)
  // ============================================================
  RCLCPP_INFO(node->get_logger(), "Step 6: Closing right gripper");
  if (!set_gripper(node, right_io, right_gripper_pin, true, "right")) {
    RCLCPP_ERROR(node->get_logger(), "Failed Step 6"); rclcpp::shutdown(); return 1;
  }
  sleep_ms(500);

  // ============================================================
  // Step 7: Left gripper releases object
  // ============================================================
  RCLCPP_INFO(node->get_logger(), "Step 7: Opening left gripper");
  if (!set_gripper(node, left_io, left_gripper_pin, false, "left")) {
    RCLCPP_ERROR(node->get_logger(), "Failed Step 7"); rclcpp::shutdown(); return 1;
  }
  sleep_ms(500);

  // ============================================================
  // Step 8: Right arm moves to final placement position
  // ============================================================
  RCLCPP_INFO(node->get_logger(), "Step 8: RIGHT -> final placement position");
  right_mgi.setNamedTarget(right_final_named);
  if (!planAndExecute(right_mgi, node->get_logger())) {
    RCLCPP_ERROR(node->get_logger(), "Failed Step 8"); rclcpp::shutdown(); return 1;
  }

  // ============================================================
  // Step 9: Right gripper releases object into box
  // ============================================================
  RCLCPP_INFO(node->get_logger(), "Step 9: Opening right gripper — releasing into box");
  if (!set_gripper(node, right_io, right_gripper_pin, false, "right")) {
    RCLCPP_ERROR(node->get_logger(), "Failed Step 9"); rclcpp::shutdown(); return 1;
  }
  sleep_ms(500);

  RCLCPP_INFO(node->get_logger(), "Handover complete.");
  rclcpp::shutdown();
  return 0;
}