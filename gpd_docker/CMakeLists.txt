cmake_minimum_required(VERSION 3.20)
project(gpd_docker)

# set(gpd_ros_DIR ../gpd_ros)

find_package(catkin REQUIRED COMPONENTS
    std_msgs
    sensor_msgs
    geometry_msgs
    message_generation
    gpd_ros
)

add_message_files(
    FILES
    Grasps.msg
)

add_service_files(
    FILES
    GetGrasps.srv
    ReScoreGrasps.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
    sensor_msgs
    geometry_msgs
    gpd_ros
)

catkin_package(
    CATKIN_DEPENDS
    message_runtime
    std_msgs
    sensor_msgs
    geometry_msgs
)
