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

// File openning
#include <iostream>
#include <fstream>

class WaypointWithTime {
 public:
  WaypointWithTime()
      : waiting_time(0), yaw(0.0) {
  }

  WaypointWithTime(double t, float x, float y, float z, float _yaw)
      : position(x, y, z), yaw(_yaw), waiting_time(t) {
  }

  Eigen::Vector3d position;
  double yaw;
  double waiting_time;
};

// Begin program
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
  target_pose_4.orientation.w =  1.0;
  target_pose_4.position.x    = -1.0;
  target_pose_4.position.y    =  1.0;
  target_pose_4.position.z    =  1.0;

  //move_group.setPoseTarget(target_pose_1);

  // Visualize the goal
  // ^^^^^^^^^^^^^^^^^^
  // This is for visualize the target_pose1.
  // I think that is better to know where it has to go, before it plan the motion
  ROS_INFO_NAMED("4 points path", "Visualizing the trajectory line"); // There is not trajectory show
  visual_tools.publishAxisLabeled(target_pose_1, "pose1");
  visual_tools.publishAxisLabeled(target_pose_2, "pose2");
  visual_tools.publishAxisLabeled(target_pose_3, "pose3");
  visual_tools.publishAxisLabeled(target_pose_4, "pose4");
  visual_tools.trigger();

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to show the motion");

  // Call the planner to compute the plan and visualize it
  // IMPORTANT: this is just the planner, not asking move_group to move the robot
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(target_pose_1);
  waypoints.push_back(target_pose_2);
  waypoints.push_back(target_pose_3);
  waypoints.push_back(target_pose_4);

  // Specify the Cartesian path to be interpolated of 1 cm (max step in Cartesian translation = 0.01)
  // Jump threshold as 0.0 -> disabling it. (this is not sure for real robots)
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

  float variable = trajectory.multi_dof_joint_trajectory.points.at(0).transforms[0].translation.z;

  //////////

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL); // This is the trajectory

  std::size_t size_trajectory = trajectory.joint_trajectory.points.size();
          //path_points.joint_trajectory.points.size();
  ROS_INFO("Trajectory size = %d", size_trajectory);

  int count = 0;
  while (count < 1)
     {
      count++;

      moveit_msgs::RobotTrajectory path_points; // Puntos de la trajectoria
      path_points = trajectory;

      //std::vector<trajectory_msgs::MultiDOFJointTrajectoryPoint> trajectory_points;

      std::stringstream ss;

      for (unsigned i=0; i<size_trajectory; i++)
      {
        ss << "point_index: " << i << std::endl
           << "positions: "
           << "[" << path_points.joint_trajectory.points[i].positions[6]
           << "," << path_points.joint_trajectory.points[i].positions[6]
           << "," << path_points.joint_trajectory.points[i].positions[6]
           //<< "," << path_points.joint_trajectory.points[i].positions[3]
           //<< "," << path_points.joint_trajectory.points[i].positions[4]
           //<< "," << path_points.joint_trajectory.points[i].positions[5]
           << "]" << std::endl;
      }

      std::ofstream outfile ("/home/crasar/points.txt",std::ios::app);
      if(!outfile.is_open())
      {
        ROS_INFO("open failed");
      }
      else
      {
          ROS_INFO("File open");
          outfile<<"trajectory"<<count<<std::endl;
          outfile<<path_points.joint_trajectory.joint_names[6]<<std::endl;
                   //multi_dof_joint_trajectory.joint_names[0]<<std::endl;
          outfile<<ss.str()<<std::endl;
          outfile.close();
      }
    }

  for (std::size_t i = 0; i < waypoints.size(); ++i)
  {
    // Iconos de los ejes de coordenadas de cada target_pose
    visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
  }
  visual_tools.trigger();

  // I don't know what this do
  //visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  //visual_tools.trigger();

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to end the demo");



}
