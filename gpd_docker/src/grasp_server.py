#!/usr/bin/env python3
import copy
import numpy as np

import rospy
import transformations as tf
from std_msgs.msg import Int64
from geometry_msgs.msg import Pose, PoseStamped

from gpd_docker.msg import Grasps
from gpd_docker.srv import GetGrasps, GetGraspsResponse
from gpd_ros.msg import CloudSamples
from gpd_ros.srv import detect_grasps, detect_graspsRequest, detect_graspsResponse


def list_to_pose(pose_list):
    pose_msg = Pose()
    pose_msg.position.x = pose_list[0]
    pose_msg.position.y = pose_list[1]
    pose_msg.position.z = pose_list[2]
    pose_msg.orientation.x = pose_list[3]
    pose_msg.orientation.y = pose_list[4]
    pose_msg.orientation.z = pose_list[5]
    pose_msg.orientation.w = pose_list[6]
    return pose_msg


def matrix_to_pose(matrix):
    translation = list(tf.translation_from_matrix(matrix))
    quaternion = list(tf.quaternion_from_matrix(matrix))
    pose_list = translation + quaternion[1:] + quaternion[:1]
    # pose_list = translation + quaternion
    return list_to_pose(pose_list)


class GraspPlanner():

    def __init__(self):
        # self.gripper_cfg = rospy.get_param('~config')

        # TODO read gripper_cfg for gripper params
        self.hand_depth = 0.0613
        self.hand_height = 0.035
        # self.grasp_plotter = GraspPlotter()

    def get_grasp_poses(
        self,
        points,  # geometry_msgs/Point
        cloud,  # sensor_msgs/PointCloud2
        camera_position,  # geometry_msgs/Point
    ):
        msg = CloudSamples()
        msg.cloud_sources.cloud = cloud
        camera_source = [Int64(0) for x in range(cloud.height * cloud.width)]
        msg.cloud_sources.camera_source = camera_source
        msg.cloud_sources.view_points = [camera_position]
        msg.samples = copy.deepcopy(points)

        rospy.wait_for_service('/detect_grasps_server/detect_grasps', timeout=5)
        try:
            service = rospy.ServiceProxy(
                '/detect_grasps_server/detect_grasps',
                detect_grasps,
            )
            resp = service(msg)
        except rospy.ServiceException as e:
            print('Service call failed:', e)
            return [], []

        pose_list = []
        score_list = []
        for grasp in resp.grasp_configs.grasps:

            pose = np.eye(4)
            pose[:3, 0] = (grasp.approach.x, grasp.approach.y, grasp.approach.z)
            pose[:3, 1] = (grasp.binormal.x, grasp.binormal.y, grasp.binormal.z)
            pose[:3, 2] = (grasp.axis.x, grasp.axis.y, grasp.axis.z)
            pose[:3, 3] = (grasp.position.x, grasp.position.y, grasp.position.z)

            #swap x and z axes
            pose = np.matmul(
                pose,
                [
                    [0., 0., 1., 0.],
                    [0, -1., 0., 0.],
                    [1., 0., 0., 0.],
                    [0., 0., 0., 1.],
                ],
            )

            pose_list.append(matrix_to_pose(pose))
            score_list.append(grasp.score.data)
        return pose_list, score_list

    def handle_grasp_request(self, req):
        grasps, scores = self.get_grasp_poses(
            req.points,
            req.cloud,
            req.camera_position,
        )
        grasps_msg = Grasps()
        grasps_msg.poses = grasps
        grasps_msg.scores = scores

        return GetGraspsResponse(grasps_msg)


if __name__ == "__main__":
    rospy.init_node('gpd_grasp_server')
    grasp_planner = GraspPlanner()
    s = rospy.Service(
        'get_grasps', GetGrasps, grasp_planner.handle_grasp_request
    )
    print("Ready to generate grasps...")
    rospy.spin()
