#include "ros/ros.h"
#include "std_msgs/String.h"

#include <moveit_msgs/DisplayTrajectory.h>


void move_group_cb(const moveit_msgs::DisplayTrajectoryConstPtr& msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_group_subscriber");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("move_group/display_planned_path", 1000, move_group_cb);

    ros::spin();

    return 0;
}
