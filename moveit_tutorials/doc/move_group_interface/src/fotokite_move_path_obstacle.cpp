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

  // Set the obstacle
  // ^^^^^^^^^^^^^^^^
  // Define a collision object ROS message
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group.getPlanningFrame();

  // The id of the object is used to identify it
  collision_object.id = "obstacle_1";

  // Define the obstacle as a box and add to the world
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.1;
  primitive.dimensions[1] =   2;
  primitive.dimensions[2] = 0.1;

  // Define a pose for the box (relative to frame_id)
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x =  1.0;
  box_pose.position.y =  0.0;
  box_pose.position.z =  0.5;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  /* --- MOVE_IT VISUALIZATION --- */

  // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("toe");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl(); //Buttons of: Next, Continue, Stop, Break.

  // Add the collision object into the world
  //ROS_INFO_NAMED("tutorial", "Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

  // Show text in RViz of status
  //visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object appears in RViz");

  // Plan a trajectory
  //move_group.setStartState(*move_group.getCurrentState());

  // Set the target point
  // ^^^^^^^^^^^^^^^^^^^^
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = 1.0;
  target_pose1.position.x =    2.0;//0.28;
  target_pose1.position.y =    0.0;//-0.2;
  target_pose1.position.z =    0.5;//0.5;
  move_group.setPoseTarget(target_pose1);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan  %s", success ? "" : "FAILED");

  ROS_INFO_NAMED("tutorial", "Visualizing plan as trajectory line");
  //visual_tools.deleteAllMarkers();
  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  //visual_tools.publishTrajectoryPath(my_plan.trajectory_);
  visual_tools.trigger();
  visual_tools.prompt("next step");

  moveit_msgs::RobotTrajectory trajectory;
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));


  std::vector<std::vector<double>> trajectory_pos(my_plan.trajectory_.joint_trajectory.points.size());
  std::vector<std::vector<double>> finger_state(my_plan.trajectory_.joint_trajectory.points.size());

  for(int i = 0; i < my_plan.trajectory_.joint_trajectory.points.size(); i++)
  {
      trajectory_pos.at(i) = {my_plan.trajectory_.joint_trajectory.points[i].positions[0],
                              my_plan.trajectory_.joint_trajectory.points[i].positions[1],
                              my_plan.trajectory_.joint_trajectory.points[i].positions[2],
                              my_plan.trajectory_.joint_trajectory.points[i].positions[3],
                              my_plan.trajectory_.joint_trajectory.points[i].positions[4],
                              my_plan.trajectory_.joint_trajectory.points[i].positions[5],
                              my_plan.trajectory_.joint_trajectory.points[i].positions[6],
                              my_plan.trajectory_.joint_trajectory.points[i].positions[7],
                              my_plan.trajectory_.joint_trajectory.points[i].positions[8],
                              my_plan.trajectory_.joint_trajectory.points[i].positions[9],
                              my_plan.trajectory_.joint_trajectory.points[i].positions[10],
                              my_plan.trajectory_.joint_trajectory.points[i].positions[11],
                              my_plan.trajectory_.joint_trajectory.points[i].positions[12],
                              my_plan.trajectory_.joint_trajectory.points[i].positions[13],
                              my_plan.trajectory_.joint_trajectory.points[i].positions[15]};

      kinematic_state->setVariablePositions(trajectory_pos[i]);
      kinematic_state->update();

      double finger_state_x = kinematic_state->getGlobalLinkTransform("finger").translation().x();
      double finger_state_y = kinematic_state->getGlobalLinkTransform("finger").translation().y();
      double finger_state_z = kinematic_state->getGlobalLinkTransform("finger").translation().z();

      finger_state.at(i) = {finger_state_x, finger_state_y, finger_state_z};


      // This is from the original code. It prints the translation and orientation of the link in each position
      // Affine3D is a Matrix with 3 rows * 4 columns
      const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("hand_2");
      ROS_INFO_STREAM("Translation: " << end_effector_state.translation());
      ROS_INFO_STREAM("Rotation: " << end_effector_state.rotation());
  }

  // Store the points in txt file
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // The solution is from here: https://answers.ros.org/question/277954/
  // 1- Create a stream with all the data from the link coordinates.
  //    Each line has the 3 values separates by comma.
  // 2- When all the data is stored in the stream is time to open the file
  // 3- If the file is open correctly the data is copy in the file
  std::stringstream ss;

  for (int i = 0; i < my_plan.trajectory_.joint_trajectory.points.size(); i++)
  {
    ss
        << finger_state[i][0]
        << "," << finger_state[i][1]
        << "," << finger_state[i][2]
        << std::endl;

    if(i == (my_plan.trajectory_.joint_trajectory.points.size()-1))
    {
        std::ofstream outfile ("/home/crasar/points.txt",std::ios::app);
        if(!outfile.is_open())
        {
          ROS_INFO("open failed");
        }
        else
        {
            outfile << "There are a total of " << i << " points in the trajectory" <<std::endl; //<<count<<std::endl;
            //outfile<<finger_state[i][0]<<","<<finger_state[i][1]<<","<<finger_state[i][2]<<std::endl;
            //multi_dof_joint_trajectory.joint_names[0]<<std::endl;
            outfile<<ss.str()<<std::endl;
            outfile.close();
            ROS_INFO("File created");
        }
    }
  }


  ROS_INFO("Finish correctly");

}
