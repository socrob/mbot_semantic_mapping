cmake_minimum_required(VERSION 2.8.3)
project(mbot_semantic_mapping)

find_package(catkin REQUIRED
COMPONENTS
    rospy
    mbot_perception_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    rospy
)

