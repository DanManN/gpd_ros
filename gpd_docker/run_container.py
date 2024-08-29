#!/usr/bin/env python3

# NOTE: please use only standard libraries
import os
import argparse
import subprocess
from pathlib import Path

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-c", type=str, help="gripper config file")
    parser.add_argument("-d", type=str, default="", help="gripper 2nd config file")
    parser.add_argument(
        "-host", type=str, default="localhost", help="host name or ip-address"
    )
    parser.add_argument(
        "launch_args",
        nargs=argparse.REMAINDER,
        help="launch args in ros style e.g. foo:=var",
    )
    args = parser.parse_args()

    assert args.c != ""
    path = Path(args.c)
    config_file = path.parts[-1]
    config_path = path.parent.absolute()

    if args.d == "":
        docker_run_command = """
            docker run \
                --rm --net=host -it --gpus all \
                -e DISPLAY=$DISPLAY -v /tmp:/tmp \
                -v {config_path}:/mnt \
                gpd_ros:latest \
                /bin/bash -i -c \
                "source ~/.bashrc; \
                roscd gpd_docker; \
                export ROS_IP={ip}; export ROS_MASTER={host}; export ROS_MASTER_URI=http://{host}:11311; \
                roslaunch gpd_docker grasp_server.launch config:=/mnt/{config_file}"
                """.format(
            ip=os.environ['ROS_IP'] if 'ROS_IP' in os.environ else '127.0.0.1',
            host=args.host,
            config_path=config_path,
            config_file=config_file,
        )
    else:
        path2 = Path(args.d)
        config_file2 = path2.parts[-1]
        config_path2 = path2.parent.absolute()
        docker_run_command = """
            docker run \
                --rm --net=host -it --gpus all \
                -e DISPLAY=$DISPLAY -v /tmp:/tmp \
                -v {config_path}:/mnt/conf1 \
                -v {config_path2}:/mnt/conf2 \
                gpd_ros:latest \
                /bin/bash -i -c \
                "source ~/.bashrc; \
                roscd gpd_docker; \
                export ROS_IP={ip}; export ROS_MASTER={host}; export ROS_MASTER_URI=http://{host}:11311; \
                roslaunch gpd_docker grasp_server2.launch config1:=/mnt/conf1/{config_file} config2:=/mnt/conf2/{config_file2}"
                """.format(
            ip=os.environ['ROS_IP'] if 'ROS_IP' in os.environ else '127.0.0.1',
            host=args.host,
            config_path=config_path,
            config_file=config_file,
            config_path2=config_path2,
            config_file2=config_file2,
        )
    print(docker_run_command)
    subprocess.call(docker_run_command, shell=True)
