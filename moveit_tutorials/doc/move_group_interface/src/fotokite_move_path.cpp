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


  // Set the target point
  // ^^^^^^^^^^^^^^^^^^^^
  // First  goal ( 1,  1, 1)
  // Second goal ( 1, -1, 1)
  // Third  goal (-1, -1, 1)
  // Fourth goal (-1,  1, 1)

  geometry_msgs::Pose target_pose_1;
  target_pose_1.orientation.w = 1.0;
  target_pose_1.position.x    = 1.0;
  target_pose_1.position.y    = 1.0;
  target_pose_1.position.z    = 1.0;

  geometry_msgs::Pose target_pose_2;
  target_pose_2.orientation.w =  1.0;
  target_pose_2.position.x    =  1.0;
  target_pose_2.position.y    = -1.0;
  target_pose_2.position.z    =  1.0;

  geometry_msgs::Pose target_pose_3;
  target_pose_3.orientation.w =  1.0;
  target_pose_3.position.x    = -1.0;
  target_pose_3.position.y    = -1.0;
  target_pose_3.position.z    =  1.0;

  geometry_msgs::Pose target_pose_4;
  target_pose_4.orientation.w = 1.0;
  target_pose_4.position.x    = target_pose_1.position.x;
  target_pose_4.position.y    = target_pose_1.position.y;
  target_pose_4.position.z    = target_pose_1.position.z;

  move_group.setPoseTarget(target_pose_1);



  // Visualize the goal
  // ^^^^^^^^^^^^^^^^^^
  // This is for visualize the target_pose1.
  // I think that is better to know where it has to go, before it plan the motion
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line"); // There is not trajectory show
  visual_tools.publishAxisLabeled(target_pose_1, "pose1");
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



}
