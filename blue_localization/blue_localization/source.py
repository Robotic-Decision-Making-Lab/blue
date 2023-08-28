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

import asyncio
import json
import select
import socket
import threading
import time
import xml.etree.ElementTree as ET
from abc import ABC
from typing import Any

import gi
import numpy as np
import qtm
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import (
    PoseStamped,
    TwistWithCovariance,
    TwistWithCovarianceStamped,
)
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CameraInfo, Image

gi.require_version("Gst", "1.0")
from gi.repository import Gst  # noqa # type: ignore


class Source(Node, ABC):
    """Base class for a localization source (e.g., camera, sonar, etc.)."""

    def __init__(self, node_name: str) -> None:
        """Create a new localization source.

        Args:
            node_name: The name of the ROS 2 node.
        """
        Node.__init__(self, node_name)
        ABC.__init__(self)


class Camera(Source):
    """BlueROV2 camera source.

    The camera source uses GStreamer to proxy the BlueROV2 camera stream (i.e., frames
    received from GStreamer are converted to ROS ``Image`` messages and republished for
    other packages to use).
    """

    def __init__(self) -> None:
        """Create a new Camera source."""
        super().__init__("camera")

        self.bridge = CvBridge()

        self.declare_parameter("port", 5600)
        self.declare_parameter("camera_matrix", list(np.zeros(9)))
        self.declare_parameter("projection_matrix", list(np.zeros(12)))
        self.declare_parameter("distortion_coefficients", list(np.zeros(5)))
        self.declare_parameter("frame.height", 1080)
        self.declare_parameter("frame.width", 1920)
        self.declare_parameter("distortion_model", "plumb_bob")

        # Get the camera intrinsics
        camera_matrix = (
            self.get_parameter("camera_matrix").get_parameter_value().double_array_value
        )
        projection_matrix = (
            self.get_parameter("projection_matrix")
            .get_parameter_value()
            .double_array_value
        )
        distortion_coefficients = (
            self.get_parameter("distortion_coefficients")
            .get_parameter_value()
            .double_array_value
        )
        frame_height = (
            self.get_parameter("frame.height").get_parameter_value().integer_value
        )
        frame_width = (
            self.get_parameter("frame.width").get_parameter_value().integer_value
        )
        distortion_model = (
            self.get_parameter("distortion_model").get_parameter_value().string_value
        )

        # Create a message with the camera info
        camera_info = CameraInfo()
        camera_info.header.stamp = self.get_clock().now().to_msg()
        camera_info.header.frame_id = "camera_link"
        camera_info.height = frame_height
        camera_info.width = frame_width
        camera_info.distortion_model = distortion_model
        camera_info.d = distortion_coefficients
        camera_info.k = camera_matrix
        camera_info.p = projection_matrix

        self.camera_info_pub = self.create_publisher(
            CameraInfo,
            "/camera/camera_info",
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            ),
        )

        # Go ahead and publish the camera info now
        # This uses transient local durability, so this message will persist for
        # subscribers
        self.camera_info_pub.publish(camera_info)

        self.camera_frame_pub = self.create_publisher(
            Image, "/camera/image_raw", qos_profile_sensor_data
        )

        # Start the GStreamer stream
        self.video_pipe, self.video_sink = self.init_stream(
            self.get_parameter("port").get_parameter_value().integer_value
        )

    def init_stream(self, port: int) -> tuple[Any, Any]:
        """Initialize a GStreamer video stream interface.

        GStreamer is used to receive video frames from the BlueROV2 for processing.

        Args:
            port: The port over which the video feed is being streamed.

        Returns:
            The video pipe and sink.
        """
        Gst.init(None)

        video_source = f"udpsrc port={port}"
        video_codec = (
            "! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264"
        )
        video_decode = (
            "! decodebin ! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert"
        )
        video_sink_conf = (
            "! appsink emit-signals=true sync=false max-buffers=2 drop=true"
        )

        command = " ".join([video_source, video_codec, video_decode, video_sink_conf])

        video_pipe = Gst.parse_launch(command)
        video_pipe.set_state(Gst.State.PLAYING)

        video_sink = video_pipe.get_by_name("appsink0")

        def proxy_frame_cb(sink: Any) -> Any:
            # Convert from a GStreamer frame to a ROS 2 message and publish
            frame = self.gst_to_opencv(sink.emit("pull-sample"))
            self.camera_frame_pub.publish(self.bridge.cv2_to_imgmsg(frame))
            return Gst.FlowReturn.OK

        video_sink.connect("new-sample", proxy_frame_cb)

        return video_pipe, video_sink

    @staticmethod
    def gst_to_opencv(frame: Any) -> np.ndarray:
        """Convert a GStreamer frame to an array.

        Args:
            frame: The GStreamer frame to convert.

        Returns:
            The GStreamer video frame as an array.
        """
        buf = frame.get_buffer()
        caps = frame.get_caps()

        return np.ndarray(
            (
                caps.get_structure(0).get_value("height"),
                caps.get_structure(0).get_value("width"),
                3,
            ),
            buffer=buf.extract_dup(0, buf.get_size()),
            dtype=np.uint8,
        )


class QualisysMotionCapture(Source):
    """Qualisys motion capture system source.

    The Qualisys motion capture source provides an ROS 2 wrapper for the Qualisys
    Python SDK. The source streams the pose of the bluerov body and republishes the pose
    as a `PoseStamped` message.
    """

    def __init__(self) -> None:
        """Create a new Qualisys motion capture source."""
        super().__init__("qualisys_mocap")

        self.declare_parameter("ip", "192.168.254.1")
        self.declare_parameter("port", 22223)
        self.declare_parameter("version", "1.22")
        self.declare_parameter("body", "ROV")

        # Load the parameters
        self.ip = self.get_parameter("ip").get_parameter_value().string_value
        self.port = self.get_parameter("port").get_parameter_value().integer_value
        self.version = self.get_parameter("version").get_parameter_value().string_value
        self.body = self.get_parameter("body").get_parameter_value().string_value

        # Publish the pose using the name of the body as the topic
        self.mocap_pub = self.create_publisher(
            PoseStamped, f"/blue/mocap/qualisys/{self.body}", qos_profile_sensor_data
        )

    @staticmethod
    def create_body_index(params: str) -> dict[str, int]:
        """Create a name to index dictionary from the 6-D parameters.

        This is used to retrieve the specific body of interest from a packet.

        Args:
            params: The 6-D parameters to use for generating the mapping.

        Returns:
            A mapping from the body name to its respective index in a packet.
        """
        xml = ET.fromstring(params)

        body_to_index = {}
        for index, body in enumerate(xml.findall("*/Body/Name")):
            if body is not None:
                body_to_index[body.text.strip()] = index  # type: ignore

        return body_to_index

    async def run_mocap(self) -> None:
        """Run the motion capture system coroutine.

        This implementation is inspired by the `stream_6dof_example.py` example included
        with the Qualisys Python SDK.
        """
        connection = await qtm.connect(self.ip, self.port, self.version)

        # Normally we would want to raise an exception here, but because the exception
        # is returned as part of the future object--which doesn't get captured until
        # keyboard interrupt due to the `spinning` method--we are going to notify
        # the users with a log message and return.
        if connection is None:
            self.get_logger().error(
                "The Qualisys motion capture source failed to establish a connection"
                f" at the address '{self.ip}:{self.port}' using version"
                f" '{self.version}'"
            )
            return

        # Load the 6D parameters
        params = await connection.get_parameters(parameters=["6d"])
        body_index = self.create_body_index(params)

        if self.body not in body_index:
            self.get_logger().error(
                f"The body '{self.body}' is not available. Please make sure that the"
                " body has been properly defined and is enabled."
            )
            return

        # Create a callback to bind to the frame stream
        def proxy_pose_cb(packet: qtm.QRTPacket) -> None:
            _, bodies = packet.get_6d()  # type: ignore

            position, rotation = bodies[body_index[self.body]]

            pose_msg = PoseStamped()

            pose_msg.header.frame_id = "map"
            pose_msg.header.stamp = self.get_clock().now().to_msg()

            # Convert from mm to m and save the position to the message
            (
                pose_msg.pose.position.x,
                pose_msg.pose.position.y,
                pose_msg.pose.position.z,
            ) = (position.x / 1000, position.y / 1000, position.z / 1000)

            # Convert from a column-major rotation matrix to a quaternion
            (
                pose_msg.pose.orientation.x,
                pose_msg.pose.orientation.y,
                pose_msg.pose.orientation.z,
                pose_msg.pose.orientation.w,
            ) = R.from_matrix(np.array(rotation.matrix).reshape((3, 3)).T).as_quat()

            self.mocap_pub.publish(pose_msg)

        # Stream the mocap 6D pose
        await connection.stream_frames(components=["6d"], on_packet=proxy_pose_cb)


class WaterLinkedDvl(Source):
    """Interface for the WaterLinked DVL A50."""

    def __init__(self) -> None:
        """Create a new DVL interface."""
        super().__init__("waterlinked_dvl")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("ip", "192.168.2.95"),
                ("speed_of_sound", 2000),
                ("mounting_rotation_offset", 180.0),
                ("acoustic_enabled", True),
                ("dark_mode_enabled", False),
                ("range_mode", "auto"),
                ("periodic_cycling_enabled", True),
            ],
        )

        self.vel_pub = self.create_publisher(
            TwistWithCovarianceStamped, "/blue/dvl/twist_cov", qos_profile_sensor_data
        )

        self.socket: socket.socket | None = None
        self._poll_t: threading.Thread | None = None

        ip = self.get_parameter("ip").get_parameter_value().string_value
        if not self.connect(ip):
            self.get_logger().info(
                "Failed to setup the DVL localization source. Exiting."
            )
            return

    def destroy_node(self) -> bool:
        """Shutdown the DVL localization source node.

        Returns:
            Whether or not the shutdown succeeded.
        """
        # Close the socket connection
        if self.socket is not None and self._poll_t is not None:
            self._running = False
            self.socket.shutdown(socket.SHUT_RDWR)
            self.socket.close()
            self._poll_t.join()

        return super().destroy_node()

    def connect(self, ip: str, port=16171, timeout: float = 5.0) -> bool:
        """Attempt to establish a connection with the DVL.

        Args:
            ip: The DVL IP address
            port: The DVL port. This should not change.
            timeout: The maximum time allowed to attempt connection before considering
                the attempt failed. Defaults to 5.0 seconds.

        Returns:
            Whether or not the connection succeeded.
        """
        start_t = time.time()

        while time.time() - start_t <= timeout:
            try:
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.connect((ip, port))
                self.socket.setblocking(False)

                # Start polling the thread
                self._running = True
                self._poll_t = threading.Thread(target=self._poll)
                self._poll_t.daemon = True
                self._poll_t.start()
            except socket.error:
                self.get_logger().error(
                    f"Failed to establish a connection with the DVL at {ip}:{port}"
                )
                return False

        return True

    @staticmethod
    def dvl_to_twist(dvl_data: dict[str, Any]) -> TwistWithCovariance:
        """Convert a DVL velocity JSON message into a TwistWithCovariance message.

        Args:
            dvl_data: The DVL JSON data to convert.

        Returns:
            The resulting TwistWithCovariance message.
        """
        twist_cov = TwistWithCovariance()

        twist_cov.twist.linear.x = dvl_data["vx"]
        twist_cov.twist.linear.y = dvl_data["vy"]
        twist_cov.twist.linear.z = dvl_data["vz"]

        covariance = np.zeros((6, 6))
        covariance[:3, :3] = np.array(dvl_data["covariance"])
        twist_cov.covariance = list(covariance.reshape(-1))

        return twist_cov

    def _poll(self) -> None:
        # This shouldn't happen, but add a check anyway to keep mypy happy
        if self.socket is None:
            self.get_logger().error(
                "Cannot poll from the DVL socket without an active connection!"
            )
            return

        data_buffer = ""

        while self._running:
            # Check if the socket is ready to read
            ready_to_read, _, _ = select.select([self.socket], [], [], 60)

            if ready_to_read:
                try:
                    data = self.socket.recv(1024).decode()

                    if data:
                        data_buffer += data
                except socket.error as e:
                    self.get_logger().debug(
                        "An error occurred while attempting to read from the DVL"
                        f" socket: {e}"
                    )

            if len(data_buffer) > 0:
                if "\n" not in data_buffer:
                    continue

                lines = data_buffer.split("\n", 1)

                if len(lines) > 1:
                    data_buffer = lines[1]

                    try:
                        parsed_data = json.loads(lines[0])
                    except json.decoder.JSONDecodeError:
                        # Sometimes we get bad data
                        continue

                    if not parsed_data or "type" not in parsed_data:
                        continue

                    if parsed_data["type"] == "velocity":
                        twist_cov = TwistWithCovarianceStamped()
                        twist_cov.header.frame_id = "base_link"
                        twist_cov.header.stamp = self.get_clock().now().to_msg()
                        twist_cov.twist = self.dvl_to_twist(parsed_data)

                        self.vel_pub.publish(twist_cov)


async def spinning(node: Node) -> None:
    """Spin the ROS 2 node as an asyncio Task.

    Args:
        node: The ROS 2 node to spin.
    """
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.01)
        await asyncio.sleep(0.0001)


def main_camera(args: list[str] | None = None):
    """Run the camera source."""
    rclpy.init(args=args)

    node = Camera()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


def main_qualisys_mocap(args: list[str] | None = None):
    """Run the Qualisys motion capture source."""

    async def run_mocap(args: list[str] | None, loop: asyncio.AbstractEventLoop):
        """Run the Qualisys motion capture system."""
        rclpy.init(args=args)

        node = QualisysMotionCapture()

        spin_task = loop.create_task(spinning(node))
        mocap_task = loop.create_task(node.run_mocap())

        try:
            await spin_task
        except KeyboardInterrupt:
            spin_task.cancel()
            mocap_task.cancel()

        node.destroy_node()
        rclpy.shutdown()

    loop = asyncio.new_event_loop()
    loop.run_until_complete(run_mocap(args, loop))


def main_waterlinked_dvl(args: list[str] | None = None):
    """Run the WaterLinked DVL source."""
    rclpy.init(args=args)

    node = WaterLinkedDvl()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
