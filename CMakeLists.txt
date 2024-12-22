cmake_minimum_required(VERSION 3.0.2)
project(ir2425_group_09)

find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    geometry_msgs
    apriltag_ros
)

catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
    scripts/nodeA.py
    scripts/nodeB_exploration.py
    scripts/nodeB_navigation.py
    scripts/nodeB_goal_management.py
    scripts/nodeBapriltag_detection.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)