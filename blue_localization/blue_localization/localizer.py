# Copyright 2023, Evan Palmer
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

from abc import ABC
from typing import Any

import cv2
import numpy as np
import rclpy
import tf2_geometry_msgs  # noqa
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Image
from tf2_ros import TransformException  # type: ignore
from tf2_ros import Time
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class Localizer(Node, ABC):
    """Base class for implementing a visual localization interface."""

    MAP_FRAME = "map"
    MAP_NED_FRAME = "map_ned"
    BASE_LINK_FRAME = "base_link"
    BASE_LINK_FRD_FRAME = "base_link_frd"

    def __init__(self, node_name: str) -> None:
        """Create a new localizer.

        Args:
            node_name: The name of the ROS 2 node.
        """
        Node.__init__(self, node_name)
        ABC.__init__(self)

        # Provide access to TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Poses are sent to the ArduPilot EKF
        self.localization_pub = self.create_publisher(
            PoseStamped, "/mavros/vision_pose/pose", 1
        )
        self.odometry_pub = self.create_publisher(Odometry, "/mavros/odometry/out", 1)

    @staticmethod
    def convert_pose_to_transform(
        node: Node, pose: Pose, reference_frame: str, child_frame: str
    ) -> TransformStamped:
        """Convert a Pose message into a TransformStamped.

        Args:
            node: The ROS 2 node that is calling the method.
            pose: The Pose message to convert into a TransformStamped message.
            reference_frame: The frame that the TransformStamped transforms from.
            child_frame: The frame that the TrasnformStamped transforms to.

        Returns:
            The converted TransformStamped message.
        """
        tf = TransformStamped()

        tf.header.stamp = node.get_clock().now().to_msg()
        tf.header.frame_id = reference_frame
        tf.child_frame_id = child_frame

        tf.transform.translation.x = pose.position.x
        tf.transform.translation.y = pose.position.y
        tf.transform.translation.z = pose.position.z

        tf.transform.rotation.x = pose.orientation.x
        tf.transform.rotation.y = pose.orientation.y
        tf.transform.rotation.z = pose.orientation.z
        tf.transform.rotation.w = pose.orientation.w

        return tf


class ArucoMarkerLocalizer(Localizer):
    """Performs localization using ArUco markers."""

    ARUCO_MARKER_TYPES = [
        cv2.aruco.DICT_4X4_50,
        cv2.aruco.DICT_4X4_100,
        cv2.aruco.DICT_4X4_250,
        cv2.aruco.DICT_4X4_1000,
        cv2.aruco.DICT_5X5_50,
        cv2.aruco.DICT_5X5_100,
        cv2.aruco.DICT_5X5_250,
        cv2.aruco.DICT_5X5_1000,
        cv2.aruco.DICT_6X6_50,
        cv2.aruco.DICT_6X6_100,
        cv2.aruco.DICT_6X6_250,
        cv2.aruco.DICT_6X6_1000,
        cv2.aruco.DICT_7X7_50,
        cv2.aruco.DICT_7X7_100,
        cv2.aruco.DICT_7X7_250,
        cv2.aruco.DICT_7X7_1000,
        cv2.aruco.DICT_ARUCO_ORIGINAL,
    ]

    def __init__(self) -> None:
        """Create a new ArUco marker localizer."""
        super().__init__("aruco_marker_localizer")

        self.bridge = CvBridge()

        self.declare_parameter("camera_matrix", [0.0 for _ in range(9)])
        self.declare_parameter("projection_matrix", [0.0 for _ in range(12)])
        self.declare_parameter("distortion_coefficients", [0.0 for _ in range(5)])

        # Get the camera intrinsics
        self.camera_matrix = np.array(
            self.get_parameter("camera_matrix")
            .get_parameter_value()
            .double_array_value,
            np.float32,
        ).reshape(3, 3)

        self.projection_matrix = np.array(
            self.get_parameter("projection_matrix")
            .get_parameter_value()
            .double_array_value,
            np.float32,
        ).reshape(3, 4)

        self.distortion_coefficients = np.array(
            self.get_parameter("distortion_coefficients")
            .get_parameter_value()
            .double_array_value,
            np.float32,
        ).reshape(1, 5)

        self.camera_sub = self.create_subscription(
            Image, "/blue/camera", self.extract_and_publish_pose_cb, 1
        )

    def detect_markers(self, frame: np.ndarray) -> tuple[Any, Any] | None:
        """Detect any ArUco markers in the frame.

        All markers in a frame should be the same type of ArUco marker
        (e.g., 4x4 50) if multiple are expected to be in-frame.

        Args:
            frame: The video frame containing ArUco markers.

        Returns:
            A list of marker corners and IDs. If no markers were found, returns None.
        """
        # Check each tag type, breaking when we find one that works
        for tag_type in self.ARUCO_MARKER_TYPES:
            aruco_dict = cv2.aruco.Dictionary_get(tag_type)
            aruco_params = cv2.aruco.DetectorParameters_create()

            try:
                # Return the corners and ids if we find the correct tag type
                corners, ids, _ = cv2.aruco.detectMarkers(
                    frame, aruco_dict, parameters=aruco_params
                )

                if len(ids) > 0:
                    return corners, ids

            except Exception:
                continue

        # Nothing was found
        return None

    def get_camera_pose(self, frame: np.ndarray) -> tuple[Any, Any, int] | None:
        """Get the pose of the camera relative to any ArUco markers detected.

        If multiple markers are detected, then the "largest" marker will be used to
        determine the pose of the camera.

        Args:
            frame: The camera frame containing ArUco markers.

        Returns:
            The rotation vector and translation vector of the camera in the marker
            frame and the ID of the marker detected. If no marker was detected,
            returns None.
        """
        # Convert to greyscale image then try to detect the tag(s)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detection = self.detect_markers(gray)

        if detection is None:
            return None

        corners, ids = detection

        # If there are multiple markers, get the marker with the "longest" side, where
        # "longest" should be interpreted as the relative size in the image
        side_lengths = [
            abs(corner[0][0][0] - corner[0][2][0])
            + abs(corner[0][0][1] - corner[0][2][1])
            for corner in corners
        ]

        min_side_idx = side_lengths.index(max(side_lengths))
        min_marker_id = ids[min_side_idx]

        # Get the estimated pose
        rot_vec, trans_vec, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners[min_side_idx],
            min_marker_id,
            self.camera_matrix,
            self.distortion_coefficients,
        )

        return rot_vec, trans_vec, min_marker_id

    def extract_and_publish_pose_cb(self, frame: Image) -> None:
        """Get the camera pose relative to the marker and send to the ArduSub EKF.

        Args:
            frame: The BlueROV2 camera frame.
        """
        # Get the pose of the camera in the `marker` frame
        camera_pose = self.get_camera_pose(self.bridge.imgmsg_to_cv2(frame))

        # If there was no marker in the image, exit early
        if camera_pose is None:
            self.get_logger().debug(
                "An ArUco marker could not be detected in the current image"
            )
            return

        rot_vec, trans_vec, marker_id = camera_pose

        # Convert the pose into a PoseStamped message
        pose = PoseStamped()

        pose.header.frame_id = f"marker_{marker_id}"
        pose.header.stamp = self.get_clock().now().to_msg()

        (
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
        ) = trans_vec.squeeze()

        rot_mat, _ = cv2.Rodrigues(rot_vec)

        (
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ) = R.from_matrix(rot_mat).as_quat()

        # Transform the pose from the `marker` frame to the `map` frame
        try:
            pose = self.tf_buffer.transform(pose, "map")
        except TransformException as e:
            self.get_logger().warning(
                f"Could not transform from frame marker_{marker_id} to map: {e}"
            )
            return

        # The pose now represents the transformation from the map frame to the
        # camera frame, but we need to publish the transformation from the map frame
        # to the base_link frame

        # Start by getting the camera to base_link transform
        try:
            tf_camera_to_base = self.tf_buffer.lookup_transform(
                "camera_link", "base_link", Time()
            )
        except TransformException as e:
            self.get_logger().warning(f"Could not access transform: {e}")
            return

        # Convert the tf into a homogeneous tf matrix
        tf_camera_to_base_mat = np.eye(4)
        tf_camera_to_base_mat[:3, :3] = R.from_quat(
            [
                tf_camera_to_base.transform.rotation.x,
                tf_camera_to_base.transform.rotation.y,
                tf_camera_to_base.transform.rotation.z,
                tf_camera_to_base.transform.rotation.w,
            ]
        ).as_matrix()
        tf_camera_to_base_mat[:3, 3] = np.array(
            [
                tf_camera_to_base.transform.translation.x,
                tf_camera_to_base.transform.translation.y,
                tf_camera_to_base.transform.translation.z,
            ]
        )

        # Convert the pose back into a matrix
        tf_map_to_camera_mat = np.eye(4)
        tf_map_to_camera_mat[:3, :3] = R.from_quat(
            [
                pose.pose.orientation.x,  # type: ignore
                pose.pose.orientation.y,  # type: ignore
                pose.pose.orientation.z,  # type: ignore
                pose.pose.orientation.w,  # type: ignore
            ]
        ).as_matrix()
        tf_map_to_camera_mat[:3, 3] = np.array(
            [
                pose.pose.position.x,  # type: ignore
                pose.pose.position.y,  # type: ignore
                pose.pose.position.z,  # type: ignore
            ]
        )

        # Calculate the new transform
        tf_map_to_base_mat = tf_camera_to_base_mat @ tf_map_to_camera_mat

        # Update the pose using the new transform
        (
            pose.pose.position.x,  # type: ignore
            pose.pose.position.y,  # type: ignore
            pose.pose.position.z,  # type: ignore
        ) = tf_map_to_base_mat[3:, 3]

        (
            pose.pose.orientation.x,  # type: ignore
            pose.pose.orientation.y,  # type: ignore
            pose.pose.orientation.z,  # type: ignore
            pose.pose.orientation.w,  # type: ignore
        ) = R.from_matrix(tf_map_to_base_mat[:3, :3]).as_quat()

        self.localization_pub.publish(pose)


class QualisysLocalizer(Localizer):
    """Localize the BlueROV2 using the Qualisys motion capture system."""

    def __init__(self) -> None:
        """Create a new Qualisys motion capture localizer."""
        super().__init__("qualisys_localizer")

        self.declare_parameter("body", "bluerov")

        body = self.get_parameter("body").get_parameter_value().string_value

        self.mocap_sub = self.create_subscription(
            PoseStamped, f"/blue/mocap/qualisys/{body}", self.proxy_pose_cb, 1
        )

    def proxy_pose_cb(self, pose: PoseStamped) -> None:
        """Proxy the pose to the ArduSub EKF.

        The pose published by the Qualisys motion capture system is already defined in
        the map frame. Therefore, all that needs to be done is to proxy this forward to
        the EKF.

        Args:
            pose: The pose of the BlueROV2 identified by the motion capture system.
        """
        self.localization_pub.publish(pose)


class GazeboLocalizer(Localizer):
    """Localize the BlueROV2 using the Gazebo ground-truth data."""

    def __init__(self) -> None:
        """Create a new Gazebo localizer."""
        super().__init__("gazebo_localizer")

        # We need to know the topic to stream from
        self.declare_parameter("gazebo_odom_topic", "")

        # Subscribe to that topic so that we can proxy messages to the ArduSub EKF
        self.create_subscription(
            Odometry,
            self.get_parameter("gazebo_odom_topic").get_parameter_value().string_value,
            self.proxy_odom_cb,
            1,
        )

    def proxy_odom_cb(self, msg: Odometry) -> None:
        """Proxy the pose data from the Gazebo odometry ground-truth data.

        Args:
            msg: The Gazebo ground-truth odometry for the BlueROV2.
        """
        pose = PoseStamped()

        # Pose is provided in the parent header frame
        pose.header.frame_id = msg.header.frame_id
        pose.header.stamp = msg.header.stamp

        # We only need the pose; we don't need the covariance
        pose.pose = msg.pose.pose

        self.localization_pub.publish(pose)


def main_aruco(args: list[str] | None = None):
    """Run the ArUco marker detector."""
    rclpy.init(args=args)

    node = ArucoMarkerLocalizer()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


def main_qualisys(args: list[str] | None = None):
    """Run the Qualisys localizer."""
    rclpy.init(args=args)

    node = QualisysLocalizer()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


def main_gazebo(args: list[str] | None = None):
    """Run the Gazebo localizer."""
    rclpy.init(args=args)

    node = GazeboLocalizer()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
