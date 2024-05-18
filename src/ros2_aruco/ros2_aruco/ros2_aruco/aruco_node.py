"""
This node locates Aruco AR markers in images and publishes their ids and poses.

Published Topics:
    /aruco_poses (geometry_msgs.msg.PoseArray)
       Pose of all detected markers (suitable for rviz visualization)

    /aruco_markers (ros2_aruco_interfaces.msg.ArucoMarkers)
       Provides an array of all poses along with the corresponding
       marker ids.

Parameters:
    marker_size - size of the markers in meters (default .0625)
    aruco_dictionary_id - dictionary that was used to generate markers
                          (default DICT_5X5_250)
    poll_delay - how many seconds to wait between captures
    camera_index - which camera index to open

Author: Nathan Sprague
Version: 10/26/2020

"""

import rclpy
import rclpy.node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import numpy as np
import cv2
import tf_transformations
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from ros2_aruco_interfaces.msg import ArucoMarkers
from rcl_interfaces.msg import ParameterDescriptor, ParameterType


class ArucoNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("aruco_node")

        # Declare and read parameters
        self.declare_parameter(
            name="marker_size",
            value=0.0625,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Size of the markers in meters.",
            ),
        )

        self.declare_parameter(
            name="aruco_dictionary_id",
            value="DICT_5X5_250",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Dictionary that was used to generate markers.",
            ),
        )

        self.declare_parameter(
            name="poll_delay",
            value=0.1,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="How long to wait between captures",
            ),
        )

        self.declare_parameter(
            name="camera_index",
            value=0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description="Which camera index to open.",
            ),
        )

        self.marker_size = (
            self.get_parameter("marker_size").get_parameter_value().double_value
        )
        self.get_logger().info(f"Marker size: {self.marker_size}")

        dictionary_id_name = (
            self.get_parameter("aruco_dictionary_id").get_parameter_value().string_value
        )
        self.get_logger().info(f"Marker type: {dictionary_id_name}")

        poll_delay = (
            self.get_parameter("poll_delay").get_parameter_value().double_value
        )
        self.get_logger().info(f"Poll frequency: {poll_delay}")

        camera_index = (
            self.get_parameter("camera_index").get_parameter_value().integer_value
        )
        self.get_logger().info(f"Camera index: {camera_index}")
        

        # Make sure we have a valid dictionary id:
        try:
            dictionary_id = cv2.aruco.__getattribute__(dictionary_id_name)
            if type(dictionary_id) != type(cv2.aruco.DICT_5X5_100):
                raise AttributeError
        except AttributeError:
            self.get_logger().error(
                "bad aruco_dictionary_id: {}".format(dictionary_id_name)
            )
            options = "\n".join([s for s in dir(cv2.aruco) if s.startswith("DICT")])
            self.get_logger().error("valid options: {}".format(options))

        # Set up publishers
        self.poses_pub = self.create_publisher(PoseArray, "aruco_poses", 10)
        self.markers_pub = self.create_publisher(ArucoMarkers, "aruco_markers", 10)

        # Setup timer and camera
        self.timer = self.create_timer(poll_delay, self.image_callback)
        self.video_capture = cv2.VideoCapture(camera_index)

        if cv2.__version__ < "4.7.0":
            self.get_logger().error("Opencv python must be at least version 4.7.0!")
            return
        
        aruco_dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(aruco_dictionary, parameters)



    def image_callback(self):
        if not self.video_capture.isOpened():
            self.get_logger().warn("Video stream not opened for aruco detection")
            return

        rval, frame = self.video_capture.read()
        
        cv_image = frame
        markers = ArucoMarkers()
        pose_array = PoseArray()

        markers.header.stamp = self.get_clock().now().to_msg()

        corners, marker_ids, rejected = self.detector.detectMarkers(cv_image)

        if marker_ids is not None:
            # rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            #     corners, self.marker_size, self.intrinsic_mat, self.distortion
            # )

            for i, marker_id in enumerate(marker_ids):
                pass
                # pose = Pose()
                # pose.position.x = tvecs[i][0][0]
                # pose.position.y = tvecs[i][0][1]
                # pose.position.z = tvecs[i][0][2]

                # rot_matrix = np.eye(4)
                # rot_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
                # quat = tf_transformations.quaternion_from_matrix(rot_matrix)

                # pose.orientation.x = quat[0]
                # pose.orientation.y = quat[1]
                # pose.orientation.z = quat[2]
                # pose.orientation.w = quat[3]

                # pose_array.poses.append(pose)
                # markers.poses.append(pose)
                markers.marker_ids.append(marker_id[0])

            # self.poses_pub.publish(pose_array)
            self.markers_pub.publish(markers)


def main():
    rclpy.init()
    node = ArucoNode()

    rclpy.spin(node)
    node.video_capture.release()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
