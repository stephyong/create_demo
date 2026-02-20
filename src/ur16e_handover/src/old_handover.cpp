#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>

#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <thread>
#include <chrono>

using namespace std::chrono_literals;

static void attach_box_to_link(       ///change to use attachObject() and detectObject() [prebuilt functions in MoveIt if this doesnt work]
  moveit::planning_interface::PlanningSceneInterface& psi, 
  const std::string& object_id,
  const std::string& link_name,
  const std::vector<std::string>& touch_links)
{
  moveit_msgs::msg::AttachedCollisionObject aco;
  aco.link_name = link_name;
  aco.object.id = object_id;
  aco.touch_links = touch_links;
  aco.object.operation = moveit_msgs::msg::CollisionObject::ADD;  // attach/add

  psi.applyAttachedCollisionObject(aco);
}

static void detach_box_from_link(
  moveit::planning_interface::PlanningSceneInterface& psi,
  const std::string& object_id,
  const std::string& link_name)
{
  moveit_msgs::msg::AttachedCollisionObject aco;
  aco.link_name = link_name;
  aco.object.id = object_id;
  aco.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;  // detach/remove

  psi.applyAttachedCollisionObject(aco);
}

static void sleep_ms(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }


static moveit_msgs::msg::CollisionObject makeCollisionObject(
  const std::string& object_id,
  const std::string& frame_id,
  double size_x, double size_y, double size_z,
  double x, double y, double z,
  double qx, double qy, double qz, double qw)
{
  moveit_msgs::msg::CollisionObject obj;
  obj.id = object_id;
  obj.header.frame_id = frame_id;

  shape_msgs::msg::SolidPrimitive prim;
  prim.type = shape_msgs::msg::SolidPrimitive::BOX;
  prim.dimensions = {size_x, size_y, size_z};

  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.x = qx;
  pose.orientation.y = qy;
  pose.orientation.z = qz;
  pose.orientation.w = qw;

  obj.primitives.push_back(prim);
  obj.primitive_poses.push_back(pose);
  obj.operation = moveit_msgs::msg::CollisionObject::ADD;
  return obj;
}

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

  const std::string cube_id = node->declare_parameter<std::string>("cube_id", "handover_cube");
  const double cube_size_x    = node->declare_parameter<double>("cube_size_x", 0.15); 
  const double cube_size_y    = node->declare_parameter<double>("cube_size_y", 0.15);
  const double cube_size_z    = node->declare_parameter<double>("cube_size_z", 0.15);
  // Cube pose relative to LEFT end-effector (object starts already in left gripper)
  const double cube_x_left = node->declare_parameter<double>("cube_x", 0.0);
  const double cube_y_left = node->declare_parameter<double>("cube_y", 0.0);
  const double cube_z_left = node->declare_parameter<double>("cube_z", 0.05); // 5 cm forward
  
  // Cube pose relative to RIGHT end-effector (adjust these based on your gripper geometry)
  const double cube_x_right = node->declare_parameter<double>("cube_x_right", 0.0);
  const double cube_y_right = node->declare_parameter<double>("cube_y_right", 0.0);
  const double cube_z_right = node->declare_parameter<double>("cube_z_right", 0.05);
  
  // Identity orientation by default
  const double cube_qx = node->declare_parameter<double>("cube_qx", 0.0);
  const double cube_qy = node->declare_parameter<double>("cube_qy", 0.0);
  const double cube_qz = node->declare_parameter<double>("cube_qz", 0.0);
  const double cube_qw = node->declare_parameter<double>("cube_qw", 1.0);

  //ground/table size
  const std::string table_id = node->declare_parameter<std::string>("table_id", "table"); 
  const double table_size_x    = node->declare_parameter<double>("table_size_x", 0.5); 
  const double table_size_y    = node->declare_parameter<double>("table_size_y", 1.2);
  const double table_size_z    = node->declare_parameter<double>("table_size_z", 0.5);
  const double table_qx = node->declare_parameter<double>("table_qx", 0.0);
  const double table_qy = node->declare_parameter<double>("table_qy", 0.0);
  const double table_qz = node->declare_parameter<double>("table_qz", 0.0);
  const double table_qw = node->declare_parameter<double>("table_qw", 1.0);
  const double table_x_base = node->declare_parameter<double>("table_x_base", 0.0);
  const double table_y_base = node->declare_parameter<double>("table_y_base", 0.0);
  const double table_z_base = node->declare_parameter<double>("table_z_base", -0.25); // half below world frame

  // ---- MoveIt interfaces ----
  moveit::planning_interface::PlanningSceneInterface psi;
  moveit::planning_interface::MoveGroupInterface left_mgi(node, left_group_name);
  moveit::planning_interface::MoveGroupInterface right_mgi(node, right_group_name);

  // Optional: make planning more forgiving while prototyping
  left_mgi.setPlanningTime(5.0);
  right_mgi.setPlanningTime(5.0);

  // Give MoveIt some time to connect to /move_group and scene monitors
  sleep_ms(800);

  RCLCPP_INFO(node->get_logger(), "Step 0: Ensure cube is attached to LEFT at start");

  // Clean up any stale object from previous runs
  psi.removeCollisionObjects({cube_id});
  psi.removeCollisionObjects({table_id});
  detach_box_from_link(psi, cube_id, left_ee_link);
  detach_box_from_link(psi, cube_id, right_ee_link);

  sleep_ms(400);

  //adding ground plane
  const std::string world_frame = left_mgi.getPlanningFrame(); // often "world" or "base_link"
  sleep_ms(200);

  moveit_msgs::msg::CollisionObject table = 
    makeCollisionObject(
      table_id, world_frame,
      table_size_x, table_size_y, table_size_z,
      table_x_base, table_y_base, table_z_base, //table positional coordinates
      table_qx, table_qy, table_qz, table_qw );

  psi.applyCollisionObject(table);
  sleep_ms(400);

  // 0A) Add cube in the world, defined in LEFT EE frame (so it appears "in the gripper")
  moveit_msgs::msg::CollisionObject cube_left =
    makeCollisionObject(cube_id, left_ee_link, 
                            cube_size_x, cube_size_y, cube_size_z,
                            cube_x_left, cube_y_left, cube_z_left,
                            cube_qx, cube_qy, cube_qz, cube_qw);

  psi.applyCollisionObject(cube_left);
  sleep_ms(400);

  // 0B) Attach cube to LEFT end-effector
  // touch_links: allow contacts between object and (parts of) the left arm/gripper
  std::vector<std::string> left_touch_links = left_mgi.getLinkNames();
  attach_box_to_link(psi, cube_id, left_ee_link, left_touch_links);
  sleep_ms(400);

  RCLCPP_INFO(node->get_logger(), "Cube '%s' attached to %s", cube_id.c_str(), left_ee_link.c_str());

  RCLCPP_INFO(node->get_logger(), "Step 1: Move LEFT to named target '%s'", left_take_named.c_str());
  left_mgi.setNamedTarget(left_take_named);
  if (!planAndExecute(left_mgi, node->get_logger())) {
    RCLCPP_ERROR(node->get_logger(), "Failed at Step 1a");
    rclcpp::shutdown();
    return 1;
  }
  // 1) Move LEFT up to handover/present pose (named state from SRDF)
  RCLCPP_INFO(node->get_logger(), "Step 1a: Move LEFT to named target '%s'", left_present_named.c_str());
  left_mgi.setNamedTarget(left_present_named);
  if (!planAndExecute(left_mgi, node->get_logger())) {
    RCLCPP_ERROR(node->get_logger(), "Failed at Step 1b");
    rclcpp::shutdown();
    return 1;
  }

  // 2) Move RIGHT to receive-ready pose
  RCLCPP_INFO(node->get_logger(), "Step 2: Move RIGHT to named target '%s'", right_receive_named.c_str());
  right_mgi.setNamedTarget(right_receive_named);
  if (!planAndExecute(right_mgi, node->get_logger())) {
    RCLCPP_ERROR(node->get_logger(), "Failed at Step 2");
    rclcpp::shutdown();
    return 1;
  }

  // 3) Transfer cube ownership: detach from LEFT, redefine in RIGHT frame, attach to RIGHT
  RCLCPP_INFO(node->get_logger(), "Step 3: Transfer cube LEFT -> RIGHT");

  // Detach from LEFT first
  detach_box_from_link(psi, cube_id, left_ee_link);
  sleep_ms(400);
  
  // Remove the old collision object from the scene
  psi.removeCollisionObjects({cube_id});
  sleep_ms(400);

  // Re-add cube, now defined in RIGHT EE frame
  moveit_msgs::msg::CollisionObject cube_right =
    makeCollisionObject(cube_id, right_ee_link,
                            cube_size_x, cube_size_y, cube_size_z,
                            cube_x_right, cube_y_right, cube_z_right,
                            cube_qx, cube_qy, cube_qz, cube_qw);

  psi.applyCollisionObject(cube_right);
  sleep_ms(400);

  // Attach to RIGHT
  std::vector<std::string> right_touch_links = right_mgi.getLinkNames();
  attach_box_to_link(psi, cube_id, right_ee_link, right_touch_links);
  sleep_ms(400);

  RCLCPP_INFO(node->get_logger(), "Cube '%s' now attached to %s", cube_id.c_str(), right_ee_link.c_str());

  // 4) Move RIGHT to final placement pose
  RCLCPP_INFO(node->get_logger(), "Step 4: Move RIGHT to named target '%s'", right_final_named.c_str());
  right_mgi.setNamedTarget(right_final_named);
  if (!planAndExecute(right_mgi, node->get_logger())) {
    RCLCPP_ERROR(node->get_logger(), "Failed at Step 4");
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Handover demo complete.");
  rclcpp::shutdown();
  return 0;
}
