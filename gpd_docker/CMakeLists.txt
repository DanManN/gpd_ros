cmake_minimum_required(VERSION 3.20)
project(gpd_docker)

find_package(catkin REQUIRED
    std_msgs
    sensor_msgs
    geometry_msgs
    message_generation
)

add_message_files(
    FILES
    Grasps.msg
)

add_service_files(
    FILES
    GetGrasps.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
    sensor_msgs
    geometry_msgs
)
