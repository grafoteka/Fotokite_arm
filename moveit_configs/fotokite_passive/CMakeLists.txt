cmake_minimum_required(VERSION 2.8.3)
project(fotokite_passive)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    control_msgs
    trajectory_msgs
    actionlib
    actionlib_msgs
    std_msgs
    )

#generate_messages(DEPENDENCIES actionlib_msgs std_msgs
#)
#catkin_package(CATKIN_DEPENDS actionlib_msgs
#)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(full_action_server src/full_action_server.cpp)
target_link_libraries(full_action_server ${catkin_LIBRARIES})

#install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#  PATTERN "setup_assistant.launch" EXCLUDE)
#install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
