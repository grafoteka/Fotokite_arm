/*********************************************************************
 * Autor: Jorge De Leon
 * E-mail: jorge.deleon@upm.es
 *
 * First program to try to move the fotokite_arm to the point (2,2,2)
 *
 *********************************************************************/

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

// Beguin program
int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  /* --- MOVE_IT SETUP --- */

  // MoveIt! operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt! the terms "planning group" and "joint model group"
  // are used interchangably.
  static const std::string PLANNING_GROUP = "arm";  //Future version: name=tether -> modify also in moveit_setup

  // The :move_group_interface:`MoveGroup` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  /* --- MOVE_IT VISUALIZATION --- */

  // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("toe");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl(); //Buttons of: Next, Continue, Stop, Break.

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().z() = 1.75; // Measure from origin (0, 0, 0)
  visual_tools.publishText(text_pose, "Demo for Fotokite path planner", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();

  /* --- START DEMO --- */
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  /*
  // Set the target point
  // ^^^^^^^^^^^^^^^^^^^^
  // First goal (1.5, 1.5, 1.5)
  // Plan motion for the group to a desired pose for the end-effector
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = 1.0;
  target_pose1.position.x = 1;
  target_pose1.position.y = 1;
  target_pose1.position.z = 1;
  move_group.setPoseTarget(target_pose1);

  // Visualize the goal
  // ^^^^^^^^^^^^^^^^^^
  // This is for visualize the target_pose1.
  // I think that is better to know where it has to go, before it plan the motion
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line"); // There is not trajectory show
  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  //visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to show the motion");

  // Call the planner to compute the plan and visualize it
  // IMPORTANT: this is just the planner, not asking move_group to move the robot
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // I don't know what this do
  //visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  //visual_tools.trigger();

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to end the demo");
  */

  // Planning to a joint-space goal
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // Now we are going to move joint[0] (Z axis) 90 degrees (PI/2 radians)

  // First create a pointer that references the current robot's state.
  // RobotState is the object that contains all the current position/velocity/acceleration data.
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

  // Get the current set of joint values for the group
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  joint_group_positions[0] = -1.57;  // radians -> 90 degrees
  move_group.setJointValueTarget(joint_group_positions);

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan (joint space goal) %s", success ? "" : "FAILED");

}
