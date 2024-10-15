cmake_minimum_required(VERSION 3.0.2)
project(gpd_ros)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS cmake_modules message_generation roscpp rospy sensor_msgs std_msgs)

## Generate messages in the 'msg' folder
add_message_files(FILES CloudIndexed.msg CloudSamples.msg CloudSources.msg GraspConfig.msg GraspConfigList.msg SamplesMsg.msg)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES geometry_msgs sensor_msgs std_msgs)

catkin_package(
    CATKIN_DEPENDS message_runtime geometry_msgs sensor_msgs std_msgs
)
