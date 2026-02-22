#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include "ur_msgs/srv/set_io.hpp"

#include <Eigen/Geometry>
#include <thread>
#include <chrono>
#include <future>
#include <cmath>
#include <vector>
#include <string>

using namespace std::chrono_literals;

static void sleep_ms(int ms)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}


static bool planAndExecute(
  moveit::planning_interface::MoveGroupInterface& mgi,
  rclcpp::Logger logger)
{
  mgi.setStartStateToCurrentState();

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto plan_rc = mgi.plan(plan);
  if (plan_rc != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Planning failed for group '%s' (code=%d)",
                 mgi.getName().c_str(), plan_rc.val);
    return false;
  }


  rclcpp::Clock clock;
  plan.trajectory_.joint_trajectory.header.stamp = clock.now();

  auto exec_rc = mgi.execute(plan);
  if (exec_rc != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Execution failed for group '%s' (code=%d)",
                 mgi.getName().c_str(), exec_rc.val);
    return false;
  }
  return true;
}

static bool planAndExecuteCartesianBetween(
  moveit::planning_interface::MoveGroupInterface& mgi,
  const geometry_msgs::msg::Pose& start_pose,
  const geometry_msgs::msg::Pose& end_pose,
  int num_waypoints,
  rclcpp::Logger logger,
  double velocity_scale = 0.3,      // added these two params so you can
  double accel_scale = 0.2,         // pass in your scaling factors
  double eef_step = 0.01,           // changed default from 0.005 -> 0.01
  double jump_threshold = 5.0)      // changed default from 0.0 -> 5.0
{
  mgi.setStartStateToCurrentState();

  // ---- Build waypoints (unchanged) ----
  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(start_pose);

  Eigen::Vector3d p_start(start_pose.position.x, start_pose.position.y, start_pose.position.z);
  Eigen::Vector3d p_end(end_pose.position.x, end_pose.position.y, end_pose.position.z);

  Eigen::Quaterniond q_start(start_pose.orientation.w, start_pose.orientation.x,
                              start_pose.orientation.y, start_pose.orientation.z);
  Eigen::Quaterniond q_end(end_pose.orientation.w, end_pose.orientation.x,
                            end_pose.orientation.y, end_pose.orientation.z);
  q_start.normalize();
  q_end.normalize();

  for (int i = 1; i <= num_waypoints; ++i) {
    double t = static_cast<double>(i) / static_cast<double>(num_waypoints);

    Eigen::Vector3d    p_interp = (1.0 - t) * p_start + t * p_end;
    Eigen::Quaterniond q_interp = q_start.slerp(t, q_end);

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

  // ---- Compute Cartesian path ----
  moveit_msgs::msg::RobotTrajectory trajectory;
  double fraction = mgi.computeCartesianPath(
    waypoints, eef_step, jump_threshold, trajectory);

  RCLCPP_INFO(logger, "Cartesian path fraction: %.2f%% for '%s'",
              fraction * 100.0, mgi.getName().c_str());

  if (fraction < 0.90) {
    RCLCPP_ERROR(logger, "Cartesian path only %.2f%% achieved for '%s'",
                 fraction * 100.0, mgi.getName().c_str());
    return false;
  }

  robot_trajectory::RobotTrajectory rt(mgi.getRobotModel(), mgi.getName());
  rt.setRobotTrajectoryMsg(*mgi.getCurrentState(), trajectory);

  // Apply velocity and acceleration limits based on your scaling factors
  trajectory_processing::IterativeParabolicTimeParameterization iptp;
  bool time_param_ok = iptp.computeTimeStamps(rt, velocity_scale, accel_scale);
  if (!time_param_ok) {
    RCLCPP_ERROR(logger, "Time parameterisation failed for '%s'", mgi.getName().c_str());
    return false;
  }

  // Convert back to message format so it can be executed
  rt.getRobotTrajectoryMsg(trajectory);

  // ---- Execute ----
  rclcpp::Clock clock;
  trajectory.joint_trajectory.header.stamp = clock.now();

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory_ = trajectory;

  auto exec_rc = mgi.execute(plan);
  if (exec_rc != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Cartesian execution failed for '%s' (code=%d)",
                 mgi.getName().c_str(), exec_rc.val);
    return false;
  }
  return true;
}

static geometry_msgs::msg::Pose poseFromURPendant(
  double x, double y, double z,
  double rx, double ry, double rz)
{
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
  auto request   = std::make_shared<ur_msgs::srv::SetIO::Request>();
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
  auto request   = std::make_shared<ur_msgs::srv::SetIO::Request>();
  request->fun   = request->FUN_SET_DIGITAL_OUT;
  request->pin   = pin;
  request->state = request->STATE_OFF;
  (void)io_client->async_send_request(request);
  RCLCPP_INFO(node->get_logger(), "Gripper OFF (pin %d)", pin);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions opts;
  auto node = rclcpp::Node::make_shared("handover_demo", opts);

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  std::thread spinner([&exec]() { exec.spin(); });

  // Convenience lambda for clean failure exit
  auto fail = [&](const std::string& msg) -> int {
    RCLCPP_ERROR(node->get_logger(), "%s", msg.c_str());
    rclcpp::shutdown();
    if (spinner.joinable()) spinner.join();
    return 1;
  };

  // ---- Group and link names ----
  const std::string left_group_name  = node->declare_parameter<std::string>("left_group",    "left_ur16e");
  const std::string right_group_name = node->declare_parameter<std::string>("right_group",   "right_ur16e");
  const std::string left_ee_link     = node->declare_parameter<std::string>("left_ee_link",  "left_tool0");
  const std::string right_ee_link    = node->declare_parameter<std::string>("right_ee_link", "right_tool0");

  //joint-space moves ----
  std::vector<double> left_initial_position = node->declare_parameter<std::vector<double>>("left_waypoint1", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  std::vector<double> left_handover = node->declare_parameter<std::vector<double>>("left_waypoint2", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  std::vector<double> right_initial_position = node->declare_parameter<std::vector<double>>("right_waypoint1", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  std::vector<double> right_prehandover1 = node->declare_parameter<std::vector<double>>("right_waypoint2", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  std::vector<double> right_prehandover2 = node->declare_parameter<std::vector<double>>("right_waypoint3", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  std::vector<double> right_handover  = node->declare_parameter<std::vector<double>>("right_waypoint4", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  std::vector<double> right_final_position  = node->declare_parameter<std::vector<double>>("right_waypoint5", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

  // ---- Cartesian waypoint count ----
  const int cartesian_waypoints = node->declare_parameter<int>("cartesian_waypoints", 20);

  // ---- Cartesian poses from UR pendant values (loaded from yaml) ----
  geometry_msgs::msg::Pose left_post_handover = poseFromURPendant( //left post handover
    node->declare_parameter<double>("left_waypoint3_x",  0.0),
    node->declare_parameter<double>("left_waypoint3_y",  0.0),
    node->declare_parameter<double>("left_waypoint3_z",  0.0),
    node->declare_parameter<double>("left_waypoint3_rx", 0.0),
    node->declare_parameter<double>("left_waypoint3_ry", 0.0),
    node->declare_parameter<double>("left_waypoint3_rz", 0.0));

  // geometry_msgs::msg::Pose right_handover = poseFromURPendant( //right handover
  //   node->declare_parameter<double>("right_waypoint3_x",  0.0),
  //   node->declare_parameter<double>("right_waypoint3_y",  0.0),
  //   node->declare_parameter<double>("right_waypoint3_z",  0.0),
  //   node->declare_parameter<double>("right_waypoint3_rx", 0.0),
  //   node->declare_parameter<double>("right_waypoint3_ry", 0.0),
  //   node->declare_parameter<double>("right_waypoint3_rz", 0.0));

  // geometry_msgs::msg::Pose right_post_handover_retract = poseFromURPendant( //right post handover
  //   node->declare_parameter<double>("right_waypoint4_x",  0.0),
  //   node->declare_parameter<double>("right_waypoint4_y",  0.0),
  //   node->declare_parameter<double>("right_waypoint4_z",  0.0),
  //   node->declare_parameter<double>("right_waypoint4_rx", 0.0),
  //   node->declare_parameter<double>("right_waypoint4_ry", 0.0),
  //   node->declare_parameter<double>("right_waypoint4_rz", 0.0));

  // geometry_msgs::msg::Pose right_lower_object_vertically = poseFromURPendant( //right lower object to final position
  //   node->declare_parameter<double>("right_waypoint5_x",  0.0),
  //   node->declare_parameter<double>("right_waypoint5_y",  0.0),
  //   node->declare_parameter<double>("right_waypoint5_z",  0.0),
  //   node->declare_parameter<double>("right_waypoint5_rx", 0.0),
  //   node->declare_parameter<double>("right_waypoint5_ry", 0.0),
  //   node->declare_parameter<double>("right_waypoint5_rz", 0.0));

  // ---- Gripper IO pins ----
  const int pin_out_left  = 13;
  const int pin_out_right = 12;

  // ---- IO clients ----
  auto left_io  = node->create_client<ur_msgs::srv::SetIO>("left_io_and_status_controller/set_io");
  auto right_io = node->create_client<ur_msgs::srv::SetIO>("right_io_and_status_controller/set_io");

  if (!left_io->wait_for_service(5s)) {
    RCLCPP_WARN(node->get_logger(), "Left SetIO service not available yet");
  }
  if (!right_io->wait_for_service(5s)) {
    RCLCPP_WARN(node->get_logger(), "Right SetIO service not available yet");
  }

  // ---- MoveIt interfaces ----
  moveit::planning_interface::MoveGroupInterface left_mgi(node,  left_group_name);
  moveit::planning_interface::MoveGroupInterface right_mgi(node, right_group_name);
  RCLCPP_INFO(node->get_logger(), "LEFT  planning frame: %s", left_mgi.getPlanningFrame().c_str()); //world
  RCLCPP_INFO(node->get_logger(), "LEFT  pose ref frame: %s", left_mgi.getPoseReferenceFrame().c_str()); //world 

  RCLCPP_INFO(node->get_logger(), "RIGHT planning frame: %s", right_mgi.getPlanningFrame().c_str()); //world
  RCLCPP_INFO(node->get_logger(), "RIGHT pose ref frame: %s", right_mgi.getPoseReferenceFrame().c_str()); //world

  left_mgi.startStateMonitor(2.0);
  right_mgi.startStateMonitor(2.0);

  left_mgi.setPlanningTime(20.0);
  right_mgi.setPlanningTime(20.0);
  left_mgi.allowReplanning(true);
  right_mgi.allowReplanning(true);

  left_mgi.setMaxVelocityScalingFactor(0.3);
  left_mgi.setMaxAccelerationScalingFactor(0.2);
  right_mgi.setMaxVelocityScalingFactor(0.3);
  right_mgi.setMaxAccelerationScalingFactor(0.2);

  sleep_ms(2000);

  // ============================================================
  // // Step 1: Left arm moves to receiving position (joint-space)
  // // // ============================================================
  // RCLCPP_INFO(node->get_logger(), "Step 1: LEFT -> receiving position");
  // left_mgi.startStateMonitor(5.0);
  // sleep_ms(1000);
  // left_mgi.setJointValueTarget(left_initial_position);
  // if (!planAndExecute(left_mgi, node->get_logger())) {
  //   return fail("Failed Step 1");
  // }
  // sleep_ms(2000);

  // // ============================================================
  // // Step 2: Left arm moves to handover position (joint-space)
  // // ============================================================
  // RCLCPP_INFO(node->get_logger(), "Step 2: LEFT -> handover position");
  // left_mgi.startStateMonitor(5.0);
  // sleep_ms(1000);
  // left_mgi.setJointValueTarget(left_handover);
  // if (!planAndExecute(left_mgi, node->get_logger())) {
  //   return fail("Failed Step 2");
  // }
  // sleep_ms(2000);

  // ============================================================
  // Step 3: Right arm moves to pre-handover position 2 (joint-space)
  // ============================================================
  RCLCPP_INFO(node->get_logger(), "Step 3: RIGHT -> pre-handover position 2");
  right_mgi.startStateMonitor(5.0);
  sleep_ms(1000);
  right_mgi.setJointValueTarget(right_initial_position);
  if (!planAndExecute(right_mgi, node->get_logger())) {
    return fail("Failed Step 3");
  }
  sleep_ms(2000);

  // ============================================================
  // Step 4: Right arm moves to pre-handover position 3 (joint-space)
  // ============================================================
  RCLCPP_INFO(node->get_logger(), "Step 4: RIGHT -> pre-handover position 3");
  right_mgi.startStateMonitor(5.0);
  sleep_ms(1000);

  right_mgi.setJointValueTarget(right_prehandover1);
  if (!planAndExecute(right_mgi, node->get_logger())) {
    return fail("Failed Step 4");
  }
  sleep_ms(2000);
///////PROBLEMS ONWARDS HERE
  // ============================================================
  // Step 5: Right arm Cartesian approach to handover waypoint
  // ============================================================
  RCLCPP_INFO(node->get_logger(), "Step 5: RIGHT Cartesian -> handover waypoint");
  right_mgi.startStateMonitor(5.0);
  // sleep_ms(1000);
  // geometry_msgs::msg::Pose right_current = right_mgi.getCurrentPose(right_ee_link).pose;
  // if (!planAndExecuteCartesianBetween(right_mgi, right_current, right_handover,
  //       cartesian_waypoints, node->get_logger(), 0.3, 0.2)) {
  //   return fail("Failed Step 5");
  // }

  // sleep_ms(3000);
  right_mgi.setJointValueTarget(right_prehandover2);
  if (!planAndExecute(right_mgi, node->get_logger())) {
    return fail("Failed Step 5 (joint-space test)");
  }

  RCLCPP_INFO(node->get_logger(), "Step 4: RIGHT -> pre-handover position 3");
  right_mgi.startStateMonitor(5.0);
  sleep_ms(1000);

  right_mgi.setJointValueTarget(right_handover);
  if (!planAndExecute(right_mgi, node->get_logger())) {
    return fail("Failed Step 4");
  }
  sleep_ms(2000);

  // ============================================================
  // Step 6: Right gripper closes (grips object)
  // ============================================================
  RCLCPP_INFO(node->get_logger(), "Step 6: Closing right gripper");
  gripper_on(node, right_io, pin_out_right);
  sleep_ms(5000);

  // ============================================================
  // Step 7: Left gripper releases object
  // ============================================================
  RCLCPP_INFO(node->get_logger(), "Step 7: Opening left gripper");
  gripper_off(node, left_io, pin_out_left);
  sleep_ms(5000);

  // ============================================================
  // Step 8: Left arm Cartesian retract away from handover zone
  // ============================================================
  // RCLCPP_INFO(node->get_logger(), "Step 8: LEFT Cartesian -> retract waypoint");
  // left_mgi.startStateMonitor(5.0);
  // sleep_ms(1000);
  // geometry_msgs::msg::Pose left_current = left_mgi.getCurrentPose(left_ee_link).pose;

  // if (!planAndExecuteCartesianBetween(left_mgi, left_current, left_post_handover,
  //       cartesian_waypoints, node->get_logger(), 0.3, 0.2)) {
  //   return fail("Failed Step 8");
  // }
  // sleep_ms(2000);

  // ============================================================
  // Step 9: Right arm Cartesian move to placement waypoint
  // ============================================================
  // RCLCPP_INFO(node->get_logger(), "Step 9: RIGHT Cartesian -> placement waypoint");
  // right_mgi.startStateMonitor(5.0);
  // sleep_ms(1000);
  // geometry_msgs::msg::Pose right_current = right_mgi.getCurrentPose(right_ee_link).pose;
  // if (!planAndExecuteCartesianBetween(right_mgi, right_current, right_post_handover_retract,
  //       cartesian_waypoints, node->get_logger(), 0.3, 0.2)) {
  //   return fail("Failed Step 9");
  // }
  // sleep_ms(2000);


  // ============================================================
  // Step 10: Right arm moves to final release position (joint-space)
  // ============================================================
  // RCLCPP_INFO(node->get_logger(), "Step 10: RIGHT -> final release position");
  // right_mgi.startStateMonitor(5.0);
  // sleep_ms(1000);
  // right_mgi.setJointValueTarget(right_final_position);
  // if (!planAndExecute(right_mgi, node->get_logger())) {
  //   return fail("Failed Step 10");
  // }
  // sleep_ms(2000);

  // RCLCPP_INFO(node->get_logger(), "Step 10b: RIGHT Cartesian -> lower object vertically into box");
  // right_mgi.startStateMonitor(5.0);
  // sleep_ms(1000);
  // right_current = right_mgi.getCurrentPose(right_ee_link).pose;
  // if (!planAndExecuteCartesianBetween(right_mgi, right_current, right_lower_object_vertically,
  //       cartesian_waypoints, node->get_logger(), 0.3, 0.2)) {
  //   return fail("Failed Step 10b");
  // }
  // sleep_ms(2000);
  // ============================================================
  // Step 11: Right gripper releases object into box
  // ============================================================
  // RCLCPP_INFO(node->get_logger(), "Step 11: Opening right gripper - releasing object");
  // sleep_ms(2000);
  // gripper_off(node, right_io, pin_out_right);
  // sleep_ms(5000);

  RCLCPP_INFO(node->get_logger(), "Handover complete.");

  rclcpp::shutdown();
  if (spinner.joinable()) spinner.join();
  return 0;
}