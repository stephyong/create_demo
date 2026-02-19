#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>

#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <Eigen/Geometry>
#include "ur_msgs/srv/set_io.hpp"

#include <thread>
#include <chrono>
#include <future>

using namespace std::chrono_literals;

static void sleep_ms(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }


static bool planAndExecute(moveit::planning_interface::MoveGroupInterface& mgi, rclcpp::Logger logger)
{
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto ok = (mgi.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (!ok) {
    RCLCPP_ERROR(logger, "Planning failed for group '%s'", mgi.getName().c_str());
    return false;
  }
  ok = (mgi.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (!ok) {
    RCLCPP_ERROR(logger, "Execution failed for group '%s'", mgi.getName().c_str());
    return false;
  }
  return true;
}

static bool planAndExecuteCartesian(
  moveit::planning_interface::MoveGroupInterface& mgi, 
  const geometry_msgs::msg::Pose& target_pose,
  rclcpp::Logger logger,
  double eef_step = 0.01,  // 1cm resolution
  double jump_threshold = 0.0)  // disable jump threshold
{
  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(target_pose);
  
  moveit_msgs::msg::RobotTrajectory trajectory;
  double fraction = mgi.computeCartesianPath(
    waypoints,
    eef_step,
    jump_threshold,
    trajectory);
  
  if (fraction < 0.95) {  // Less than 95% of path achieved
    RCLCPP_ERROR(logger, "Cartesian path only achieved %.2f%% for group '%s'", 
                 fraction * 100.0, mgi.getName().c_str());
    return false;
  }
  
  // Execute the trajectory
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory_ = trajectory;
  
  if (mgi.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Cartesian execution failed for '%s'", mgi.getName().c_str());
    return false;
  }
  return true;
}

static bool set_gripper( //true means close gripper, false = open gripper
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

//blind approach
static geometry_msgs::msg::Pose computeApproachPose(
  const geometry_msgs::msg::Pose& grasp_pose,
  double distance)
{
  geometry_msgs::msg::Pose approach = grasp_pose;

  // Extract orientation as Eigen quaternion
  Eigen::Quaterniond q(
    grasp_pose.orientation.w,
    grasp_pose.orientation.x,
    grasp_pose.orientation.y,
    grasp_pose.orientation.z);
  q.normalize();

  // Rotate the unit Z vector into world frame — gives us the tool's approach direction
  Eigen::Vector3d z_axis = q * Eigen::Vector3d::UnitZ();

  Eigen::Vector3d p(
    grasp_pose.position.x,
    grasp_pose.position.y,
    grasp_pose.position.z);

  // Step back along tool Z
  Eigen::Vector3d p_approach = p - distance * z_axis;

  approach.position.x = p_approach.x();
  approach.position.y = p_approach.y();
  approach.position.z = p_approach.z();

  return approach;
}


static geometry_msgs::msg::Pose computeRightGraspFromLeft(
  const geometry_msgs::msg::Pose& left_pose,
  double y_offset)      // positive = shift in world +Y
{
  geometry_msgs::msg::Pose right_grasp = left_pose;

  // Rotate orientation 180 degrees about world Z so grippers face each other
  Eigen::Quaterniond q_left(
    left_pose.orientation.w,
    left_pose.orientation.x,
    left_pose.orientation.y,
    left_pose.orientation.z);
  q_left.normalize();

  Eigen::AngleAxisd flip(M_PI, Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond q_right = q_left * Eigen::Quaterniond(flip);
  q_right.normalize();

  right_grasp.orientation.w = q_right.w();
  right_grasp.orientation.x = q_right.x();
  right_grasp.orientation.y = q_right.y();
  right_grasp.orientation.z = q_right.z();

  // Apply lateral offset so right gripper targets the object, not left_tool0 itself
  right_grasp.position.y += y_offset;

  return right_grasp;
}



int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("handover_demo");
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  std::thread spinner([&exec]() { exec.spin(); });
  spinner.detach();

  // ---- Parameters you likely want to adjust ----
  const std::string left_group_name  = node->declare_parameter<std::string>("left_group", "left_ur16e");
  const std::string right_group_name = node->declare_parameter<std::string>("right_group", "right_ur16e");

  const std::string left_ee_link  = node->declare_parameter<std::string>("left_ee_link", "left_tool0");
  const std::string right_ee_link = node->declare_parameter<std::string>("right_ee_link", "right_tool0");
  const std::string left_take_named  = node->declare_parameter<std::string>("left_take_named", "left_receiving_position");
  const std::string left_present_named  = node->declare_parameter<std::string>("left_present_named", "left_handover_position");
  const std::string right_receive_named = node->declare_parameter<std::string>("right_receive_named", "right_prehandover_position");
  const std::string right_final_named   = node->declare_parameter<std::string>("right_final_named", "right_lower_object_position");
  const uint8_t left_gripper_pin  = static_cast<uint8_t>(node->declare_parameter<int>("left_gripper_pin", 13));
  const uint8_t right_gripper_pin = static_cast<uint8_t>(node->declare_parameter<int>("right_gripper_pin", 1));
  const double approach_distance  = node->declare_parameter<double>("approach_distance", 0.05); //right gripper starts 5cm behind grasp point 
  const double right_y_offset     = node->declare_parameter<double>("right_y_offset",    0.15);//position of right gripper relative to left
  auto left_io = node->create_client<ur_msgs::srv::SetIO>("left gripper and status");
  auto right_io = node->create_client<ur_msgs::srv::SetIO>("right gripper and status");

  // ---- MoveIt interfaces ----
  moveit::planning_interface::PlanningSceneInterface psi;
  moveit::planning_interface::MoveGroupInterface left_mgi(node, left_group_name);
  moveit::planning_interface::MoveGroupInterface right_mgi(node, right_group_name);

  // Optional: make planning more forgiving while prototyping
  left_mgi.setPlanningTime(5.0);
  right_mgi.setPlanningTime(5.0);
  sleep_ms(800);

  RCLCPP_INFO(node->get_logger(), "LEFT receives from human");


   //initial pose to receive object
  RCLCPP_INFO(node->get_logger(), "Step 1: Move LEFT to named target '%s'", left_take_named.c_str());
  left_mgi.setNamedTarget(left_take_named);
  if (!planAndExecute(left_mgi, node->get_logger())) {
    RCLCPP_ERROR(node->get_logger(), "Failed at Step 1a");
    rclcpp::shutdown();
    return 1;
  }
  // 1) Move LEFT up high to handover/present pose (named state from SRDF)
  RCLCPP_INFO(node->get_logger(), "Step 2: Move LEFT to named target '%s'", left_present_named.c_str());
  left_mgi.setNamedTarget(left_present_named);
  if (!planAndExecute(left_mgi, node->get_logger())) {
    RCLCPP_ERROR(node->get_logger(), "Failed at Step 2");
    rclcpp::shutdown();
    return 1;
  }

  // 2) Move RIGHT up high to receive-ready pose
  RCLCPP_INFO(node->get_logger(), "Step 3: Move RIGHT to named target '%s'", right_receive_named.c_str());
  right_mgi.setNamedTarget(right_receive_named);
  if (!planAndExecute(right_mgi, node->get_logger())) {
    RCLCPP_ERROR(node->get_logger(), "Failed at Step 3");
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Step 4: computing right grasp from left tool");

  geometry_msgs::msg::Pose left_handover_pose = left_mgi.getCurrentPose(left_ee_link).pose;
  geometry_msgs::msg:: Pose right_grasp_pose = computeRightGraspFromLeft(left_current_pose, right_y_offset);
  geometry_msgs::msg::Pose right_approach_pose = computeApproachPose(right_grasp_pose, approach_distance);

  RCLCPP_INFO(node->get_logger(),"Step 5: right gripper moving to approach pose")
  if (!planAndExecuteCartesian(right_mgi, right_approach_pose, node->get_logger())) {
    RCLCPP_ERROR(node->get_logger(), "Failed Step 5"); rclcpp::shutdown(); return 1;
  }

  //right gripper clamps, left gripper still on 

  RCLCPP_INFO(node->get_logger(),"Step 6: right gripper opens");
  set_gripper(node, right_io_client, right_gripper_pin, false,  "right");  // right opens
  sleep_ms(500);

  RCLCPP_INFO(node->get_logger(),"Step 7: right gripper moving to grab object")
  if (!planAndExecuteCartesian(right_mgi, right_grasp_pose, node->get_logger())) {
    RCLCPP_ERROR(node->get_logger(), "Failed Step 7"); rclcpp::shutdown(); return 1;
  }
  RCLCPP_INFO(node->get_logger(),"Step 8: right gripper closes");
  set_gripper(node, right_io_client, right_gripper_pin, true,  "right");  // right closes
  sleep_ms(500);

  RCLCPP_INFO(node->get_logger(),"Step 9: left gripper opens");
  set_gripper(node, left_io_client, left_gripper_pin, false,  "left");  // left opens
  sleep_ms(500);

//C05 is pin 13 

  // 4) Move RIGHT to final placement pose
  RCLCPP_INFO(node->get_logger(), "Step 10: Move RIGHT to final position '%s'", right_final_named.c_str());
  right_mgi.setNamedTarget(right_final_named);
  if (!planAndExecute(right_mgi, node->get_logger())) {
    RCLCPP_ERROR(node->get_logger(), "Failed at Step 10");
    rclcpp::shutdown(); return 1;
  }

  RCLCPP_INFO(node->get_logger(),"Step 11: right gripper releases object");
  set_gripper(node, right_io_client, right_gripper_pin, false,  "right");  // right opens
  sleep_ms(500);

  RCLCPP_INFO(node->get_logger(), "Handover demo complete.");
  rclcpp::shutdown();
  return 0;
}
