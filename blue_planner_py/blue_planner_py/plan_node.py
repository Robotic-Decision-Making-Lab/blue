import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from tf_transformations import euler_from_quaternion
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def position_to_list(position):
    return [position.x, position.y, position.z]


def quaternion_to_list(orientation):
    return [orientation.x, orientation.y, orientation.z, orientation.w]


class MoveBluerov(Node):
    def __init__(self):
        super().__init__("move_bluerov")

        self.subscription = self.create_subscription(
            Odometry, "/model/bluerov2_heavy/odometry", self.odom_callback, 10
        )
        self.subscription

        self.odom_pose_position = []
        self.odom_pose_orientation_rpy = []

        self.publisher_ = self.create_publisher(
            JointTrajectory, "/jt_controller/joint_trajectory", 10
        )
        timer_period = 2.0  # seconds
        self.timer = self.create_timer(timer_period, self.issue_jt_control_command)

    def odom_callback(self, msg):
        self.odom_pose_position = position_to_list(msg.pose.pose.position)
        self.odom_pose_orientation_rpy = list(
            euler_from_quaternion(quaternion_to_list(msg.pose.pose.orientation))
        )

    def issue_jt_control_command(self):
        # TODO: Implement this method
        """
        create two points, point1 and point2, with position and velocity values
        """
        self.get_logger().info(
            "pose: {0}, {1}".format(
                self.odom_pose_position, self.odom_pose_orientation_rpy
            )
        )
        joint_state = []
        joint_state_position = self.odom_pose_position.copy()
        joint_state_orientation = self.odom_pose_orientation_rpy.copy()
        joint_state = joint_state_position
        joint_state.extend(joint_state_orientation)

        if len(joint_state) != 0:
            # self.get_logger().info("planning: {0}".format(joint_state))
            point1 = JointTrajectoryPoint()
            point2 = JointTrajectoryPoint()
            duration1 = Duration(seconds=0.0)
            duration2 = Duration(seconds=4.0)
            point1.time_from_start = duration1.to_msg()
            point2.time_from_start = duration2.to_msg()
            point1.positions = joint_state
            vel_1 = [0.0] * 6
            vel_1[0] = 0.2
            point1.velocities = vel_1
            pos_2 = joint_state.copy()
            pos_2[0] = pos_2[0] + (0.2 * 4.0)
            point2.positions = pos_2
            vel_2 = [0.0] * 6
            point2.velocities = vel_2
            trajectory = JointTrajectory()
            trajectory.joint_names = [
                "joint_x",
                "joint_y",
                "joint_z",
                "joint_rx",
                "joint_ry",
                "joint_rz",
            ]
            trajectory.points = [point1, point2]
            trajectory.header.stamp = self.get_clock().now().to_msg()
            trajectory.header.frame_id = "base_link"
            self.publisher_.publish(trajectory)
            self.get_logger().info("published: {0}".format(trajectory))


def main(args=None):
    rclpy.init(args=args)

    stow_command = MoveBluerov()

    rclpy.spin_once(stow_command)
    stow_command.issue_jt_control_command()
    rclpy.spin(stow_command)
    stow_command.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
