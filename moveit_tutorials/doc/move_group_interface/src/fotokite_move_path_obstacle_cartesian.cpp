    /*********************************************************************
    * Autor: Jorge De Leon
    * E-mail: jorge.deleon@upm.es
    *
    * Program to rech a point with an obstacle in the middle.
    * The path is cartesian, so it should be necessary to detect where
    * is the obstacle and then add another waypoint to modify the plan
    *
    *********************************************************************/

#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit/kinematic_constraints/utils.h>

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
    geometry_msgs::Pose pose_collision;
    pose_collision.orientation.w = 1.0;
    pose_collision.position.x =    2.0;//0.28;
    pose_collision.position.y =    0.0;//-0.2;
    pose_collision.position.z =    0.5;//0.5;
    move_group.setPoseTarget(pose_collision);

    visual_tools.publishAxisLabeled(pose_collision, "Goal Pose");

    visual_tools.trigger();

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    // Try to reach with Cartesian path
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // First we try to reach to the goal with a cartesian path.

    //robot_state::RobotState start_state(*move_group.getCurrentState());

    /*std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(move_group.getCurrentPose().pose);
    waypoints.push_back(pose_collision);

    // Specify the Cartesian path to be interpolated of 1 cm (max step in Cartesian translation = 0.01)
    // Jump threshold as 0.0 -> disabling it. (this is not sure for real robots)
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.01;
    const double eef_step = 0.01;
    //double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    double fraction;
    (my_plan, fraction) = move_group.computeCartesianPath(
              waypoints,  //# waypoints to follow
              eef_step,  //# eef_step
              jump_threshold,  //# jump_threshold
              trajectory,
              true); //#avoid_collisions
    ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

    ROS_INFO("Trajectoy points = %d", trajectory.joint_trajectory.points.size());*/

    // Setup planning scene
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    planning_scene::PlanningScene planning_scene(kinematic_model);

    // Self-collision checking
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    planning_scene.checkSelfCollision(collision_request, collision_result);
    ROS_INFO_STREAM("Test 1: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");

    // Checking kinematic constraints
    std::string end_effector_name = joint_model_group->getLinkModelNames().back();

    geometry_msgs::PoseStamped target_pose1;
    target_pose1.pose.orientation.w = 1.0;
    target_pose1.pose.position.x =    1.0;//0.28;
    target_pose1.pose.position.y =    0.0;//-0.2;
    target_pose1.pose.position.z =    0.5;//0.5;
    target_pose1.header.frame_id =  "toe";

    moveit_msgs::Constraints goal_constraint =
            kinematic_constraints::constructGoalConstraints(end_effector_name, target_pose1);

    robot_state::RobotState copied_state = planning_scene.getCurrentState();
    /*copied_state.setToPoseTarget(target_pose1);
    copied_state.update();
    bool constrained = planning_scene.isStateConstrained(copied_state, goal_constraint);
    ROS_INFO_STREAM("Test 8: Random state is " << (constrained ? "constrained" : "not constrained"));
*/


    ROS_INFO("Finish correctly");

    }
