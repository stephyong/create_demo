#include <chrono>
#include <thread>
#include <atomic>
#include <future>
#include <string>
#include <vector>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "ur_msgs/srv/set_io.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_parabolic_time_parameterization.h>
#include <moveit/robot_state/robot_state.h>


#include <geometry_msgs/msg/pose.hpp>
#include <Eigen/Geometry>

using namespace std::chrono_literals;

// ─────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────

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
    plan.trajectory_.joint_trajectory.header.stamp = rclcpp::Time(0);
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
    rclcpp::Logger logger,
    double velocity_scale = 0.3,
    double accel_scale    = 0.2,
    double eef_step       = 0.01,
    double jump_threshold = 5.0)
{
    mgi.setStartStateToCurrentState();

    std::vector<geometry_msgs::msg::Pose> waypoints = {start_pose, end_pose};

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

    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    if (!iptp.computeTimeStamps(rt, velocity_scale, accel_scale)) {
        RCLCPP_ERROR(logger, "Time parameterisation failed for '%s'",
                     mgi.getName().c_str());
        return false;
    }

    rt.getRobotTrajectoryMsg(trajectory);
    trajectory.joint_trajectory.header.stamp = rclcpp::Time(0);

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
    if (angle < 1e-10)
        q = Eigen::Quaterniond::Identity();
    else
        q = Eigen::Quaterniond(Eigen::AngleAxisd(angle, rvec.normalized()));
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
    io_client->async_send_request(request);
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
    io_client->async_send_request(request);
    RCLCPP_INFO(node->get_logger(), "Gripper OFF (pin %d)", pin);
}

// ─────────────────────────────────────────────────────────────
// Main
// ─────────────────────────────────────────────────────────────

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions opts;
    auto node       = rclcpp::Node::make_shared("handover_demo",  opts);
    auto left_node  = rclcpp::Node::make_shared("left_arm_node",  opts);
    auto right_node = rclcpp::Node::make_shared("right_arm_node", opts);

    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.add_node(left_node);
    exec.add_node(right_node);

    std::thread spinner([&exec]() { exec.spin(); });

    // ---- Group and link names ----
    const std::string left_group_name  = node->declare_parameter("left_group",    "left_ur16e");
    const std::string right_group_name = node->declare_parameter("right_group",   "right_ur16e");
    const std::string left_ee_link     = node->declare_parameter("left_ee_link",  "left_tool0");
    const std::string right_ee_link    = node->declare_parameter("right_ee_link", "right_tool0");

    // ---- Joint-space waypoints ----
    std::vector<double> left_initial_position   = node->declare_parameter<std::vector<double>>("left_waypoint1",  {0,0,0,0,0,0});
    std::vector<double> left_handover            = node->declare_parameter<std::vector<double>>("left_waypoint2",  {0,0,0,0,0,0});
    std::vector<double> left_post_handover       = node->declare_parameter<std::vector<double>>("left_waypoint3",  {0,0,0,0,0,0});
    std::vector<double> left_final_position      = node->declare_parameter<std::vector<double>>("left_waypoint4",  {0,0,0,0,0,0});

    std::vector<double> right_initial_position   = node->declare_parameter<std::vector<double>>("right_waypoint1", {0,0,0,0,0,0});
    std::vector<double> right_prehandover1       = node->declare_parameter<std::vector<double>>("right_waypoint2", {0,0,0,0,0,0});
    std::vector<double> right_prehandover2       = node->declare_parameter<std::vector<double>>("right_waypoint3", {0,0,0,0,0,0});
    std::vector<double> right_handover           = node->declare_parameter<std::vector<double>>("right_waypoint4", {0,0,0,0,0,0});
    std::vector<double> right_prehandover2_2     = node->declare_parameter<std::vector<double>>("right_waypoint5", {0,0,0,0,0,0});
    std::vector<double> right_final_position_above = node->declare_parameter<std::vector<double>>("right_waypoint6", {0,0,0,0,0,0});
    std::vector<double> right_final_position_end   = node->declare_parameter<std::vector<double>>("right_waypoint7", {0,0,0,0,0,0});

    const int cartesian_waypoints = node->declare_parameter("cartesian_waypoints", 6);

    // ---- IO clients ----
    auto left_io  = node->create_client<ur_msgs::srv::SetIO>("left_io_and_status_controller/set_io");
    auto right_io = node->create_client<ur_msgs::srv::SetIO>("right_io_and_status_controller/set_io");

    if (!left_io->wait_for_service(5s))
        RCLCPP_WARN(node->get_logger(), "Left SetIO service not available yet");
    if (!right_io->wait_for_service(5s))
        RCLCPP_WARN(node->get_logger(), "Right SetIO service not available yet");

    // ---- MoveIt interfaces ----
    moveit::planning_interface::MoveGroupInterface left_mgi(left_node,  left_group_name);
    moveit::planning_interface::MoveGroupInterface right_mgi(right_node, right_group_name);

    RCLCPP_INFO(node->get_logger(), "LEFT  planning frame : %s", left_mgi.getPlanningFrame().c_str());
    RCLCPP_INFO(node->get_logger(), "LEFT  pose ref frame : %s", left_mgi.getPoseReferenceFrame().c_str());
    RCLCPP_INFO(node->get_logger(), "RIGHT planning frame : %s", right_mgi.getPlanningFrame().c_str());
    RCLCPP_INFO(node->get_logger(), "RIGHT pose ref frame : %s", right_mgi.getPoseReferenceFrame().c_str());

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


    // Pre-compute Cartesian poses from joint configs
    moveit::core::RobotStatePtr fk_state = right_mgi.getCurrentState();

    // right_prehandover2 in Cartesian pose
    fk_state->setJointGroupPositions(right_group_name, right_prehandover2);
    fk_state->updateLinkTransforms();
    Eigen::Isometry3d tf_pre = fk_state->getGlobalLinkTransform(right_ee_link);  // value copy, not reference
    geometry_msgs::msg::Pose prehandover_pose;
    prehandover_pose.position.x = tf_pre.translation().x();
    prehandover_pose.position.y = tf_pre.translation().y();
    prehandover_pose.position.z = tf_pre.translation().z();
    Eigen::Quaterniond q_pre(tf_pre.rotation());
    prehandover_pose.orientation.w = q_pre.w();
    prehandover_pose.orientation.x = q_pre.x();
    prehandover_pose.orientation.y = q_pre.y();
    prehandover_pose.orientation.z = q_pre.z();

    // right_handover in Cartesian pose with +2cm Y offset
    fk_state->setJointGroupPositions(right_group_name, right_handover);
    fk_state->updateLinkTransforms();
    Eigen::Isometry3d tf_hand = fk_state->getGlobalLinkTransform(right_ee_link);  // value copy, not reference
    geometry_msgs::msg::Pose handover_pose;
    handover_pose.position.x = tf_hand.translation().x();
    handover_pose.position.y = tf_hand.translation().y() + 0.02;
    handover_pose.position.z = tf_hand.translation().z();
    Eigen::Quaterniond q_hand(tf_hand.rotation());
    handover_pose.orientation.w = q_hand.w();
    handover_pose.orientation.x = q_hand.x();
    handover_pose.orientation.y = q_hand.y();
    handover_pose.orientation.z = q_hand.z();

    // ---- Action clients for parallel execution ----
    using FollowJT = control_msgs::action::FollowJointTrajectory;

    auto left_controller  = rclcpp_action::create_client<FollowJT>(
        node, "/left_scaled_joint_trajectory_controller/follow_joint_trajectory");
    auto right_controller = rclcpp_action::create_client<FollowJT>(
        node, "/right_scaled_joint_trajectory_controller/follow_joint_trajectory");

    left_controller->wait_for_action_server(5s);
    right_controller->wait_for_action_server(5s);

    // run_parallel helper 
    auto run_parallel = [&](auto left_fn, auto right_fn) -> bool {
        std::atomic<bool> left_success(false), right_success(false);
        std::thread lt([&]() { left_success  = left_fn();  });
        std::thread rt([&]() { right_success = right_fn(); });
        lt.join();
        rt.join();
        return left_success.load() && right_success.load();
    };

    // ---- parallel_move helper ----
    auto parallel_move = [&](
        moveit::planning_interface::MoveGroupInterface& l_mgi,
        moveit::planning_interface::MoveGroupInterface& r_mgi,
        const std::vector<double>& left_target,
        const std::vector<double>& right_target,
        const std::string& step_name) -> bool
    {
        l_mgi.setStartStateToCurrentState();
        r_mgi.setStartStateToCurrentState();
        l_mgi.setJointValueTarget(left_target);
        r_mgi.setJointValueTarget(right_target);

        moveit::planning_interface::MoveGroupInterface::Plan left_plan, right_plan;

        if (!run_parallel(
                [&]() { return l_mgi.plan(left_plan)  == moveit::core::MoveItErrorCode::SUCCESS; },
                [&]() { return r_mgi.plan(right_plan) == moveit::core::MoveItErrorCode::SUCCESS; }))
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to plan: %s", step_name.c_str());
            return false;
        }

        auto left_goal  = FollowJT::Goal();
        auto right_goal = FollowJT::Goal();
        left_goal.trajectory  = left_plan.trajectory_.joint_trajectory;
        right_goal.trajectory = right_plan.trajectory_.joint_trajectory;
        left_goal.trajectory.header.stamp  = rclcpp::Time(0);
        right_goal.trajectory.header.stamp = rclcpp::Time(0);

        auto left_future  = left_controller->async_send_goal(left_goal);
        auto right_future = right_controller->async_send_goal(right_goal);

        auto left_goal_handle  = left_future.get();
        auto right_goal_handle = right_future.get();

        if (!left_goal_handle || !right_goal_handle) {
            RCLCPP_ERROR(node->get_logger(), "Goal rejected: %s", step_name.c_str());
            return false;
        }

        auto left_result  = left_controller->async_get_result(left_goal_handle).get();
        auto right_result = right_controller->async_get_result(right_goal_handle).get();

        if (left_result.code  != rclcpp_action::ResultCode::SUCCEEDED ||
            right_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_ERROR(node->get_logger(), "Execution failed: %s", step_name.c_str());
            return false;
        }
        return true;
    };


    // Guard flag — prevents overlapping calls
    
    std::atomic<bool> handover_in_progress(false);

    // Service definition

    auto handover_service = node->create_service<std_srvs::srv::Trigger>(
        "robot_to_robot_handover",
        [&](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
            std::shared_ptr<std_srvs::srv::Trigger::Response> res)
        {
            // Reject if already running
            if (handover_in_progress.exchange(true)) {
                res->success = false;
                res->message = "Handover already in progress";
                RCLCPP_WARN(node->get_logger(), "robot_to_robot_handover called while busy — rejected");
                return;
            }

            RCLCPP_INFO(node->get_logger(), "robot_to_robot_handover service called — starting sequence");

            

            std::promise<std::pair<bool, std::string>> result_promise;
            auto result_future = result_promise.get_future();

            std::thread([&, p = std::move(result_promise)]() mutable
            {
                // Convenience macro — sets result and returns from the lambda
                auto fail_step = [&](std::promise<std::pair<bool,std::string>>& promise,
                                     const std::string& msg) {
                    RCLCPP_ERROR(node->get_logger(), "%s", msg.c_str());
                    promise.set_value({false, msg});
                    handover_in_progress = false;
                };

                sleep_ms(2000);
                left_mgi.setStartStateToCurrentState();
                right_mgi.setStartStateToCurrentState();


                RCLCPP_INFO(node->get_logger(), "Step 1: both arms to initial positions");
                if (!parallel_move(left_mgi, right_mgi,
                                   left_initial_position, right_initial_position, "Step 1")) {
                    fail_step(p, "Failed Step 1"); return;
                }
                sleep_ms(500);


                RCLCPP_INFO(node->get_logger(), "Step 2: left to handover, right to pre-handover2");
                if (!parallel_move(left_mgi, right_mgi,
                                   left_handover, right_prehandover2, "Step 2")) {
                    fail_step(p, "Failed Step 2"); return;
                }
                sleep_ms(500);


                // RCLCPP_INFO(node->get_logger(), "Step 3: right arm to handover position");
                // right_mgi.setJointValueTarget(right_handover);
                // if (!planAndExecute(right_mgi, node->get_logger())) {
                //     fail_step(p, "Failed Step 3: right to handover"); return;
                // }
                // sleep_ms(1000);

                RCLCPP_INFO(node->get_logger(), "Step 3: right Cartesian move to handover (updated 2cm in the Y direction)");
                if (!planAndExecuteCartesianBetween(right_mgi, prehandover_pose, handover_pose,
                                                    node->get_logger())) {
                    fail_step(p, "Failed Step 3"); return;
                  
                  }
                sleep_ms(500);


                RCLCPP_INFO(node->get_logger(), "Step 4: closing right gripper");
                gripper_on(node, right_io, 13);
                sleep_ms(3000);


                RCLCPP_INFO(node->get_logger(), "Step 5: opening left gripper");
                gripper_off(node, left_io, 13);
                sleep_ms(3000);


                RCLCPP_INFO(node->get_logger(), "Step 6: right retracts to pre-handover2");
                right_mgi.setJointValueTarget(right_prehandover2);
                if (!planAndExecute(right_mgi, node->get_logger())) {
                    fail_step(p, "Failed Step 6: right retract to pre-handover2"); return;
                }


                RCLCPP_INFO(node->get_logger(), "Step 7: left post-handover, right above final");
                if (!parallel_move(left_mgi, right_mgi,
                                   left_post_handover, right_final_position_above,
                                   "Step 7")) {
                    fail_step(p, "Failed Step 7"); return;
                }
                sleep_ms(500);

                
                RCLCPP_INFO(node->get_logger(), "Step 8: left goes home, right goes to final");
                if (!parallel_move(left_mgi, right_mgi,
                                   left_final_position, right_final_position_end,
                                   "Step 8")) {
                    fail_step(p, "Failed Step 8"); return;
                }


                RCLCPP_INFO(node->get_logger(), "Step 9: opening right gripper");
                gripper_off(node, right_io, 13);



                RCLCPP_INFO(node->get_logger(), "Step 10: right retracts above final");
                right_mgi.setJointValueTarget(right_final_position_above);
                if (!planAndExecute(right_mgi, node->get_logger())) {
                    fail_step(p, "Failed Step 10: right retract above"); return;
                }


                RCLCPP_INFO(node->get_logger(), "Step 11: right→home");
                right_mgi.setJointValueTarget(right_initial_position);
                if (!planAndExecute(right_mgi, node->get_logger())) {
                    fail_step(p, "Failed Step 11: right home"); return;
                }

                RCLCPP_INFO(node->get_logger(), "Handover complete.");
                p.set_value({true, "Handover complete"});
                handover_in_progress = false;

            }).detach();



            auto result = result_future.get();
            res->success = result.first;
            res->message = result.second;
        });

    RCLCPP_INFO(node->get_logger(),
                "handover_demo ready — call /robot_to_robot_handover (std_srvs/Trigger) to start");

    // Keep main alive; executor runs in spinner thread
    spinner.join();
    return 0;
}