# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__("minimal_publisher")
        self.publisher_ = self.create_publisher(
            Twist, "/integral_sliding_mode_controller/reference", 10
        )
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0.5
        self.get_logger().info(
            "Publishing: %f, %f, %f, %f, %f, %f"
            % (
                vel_msg.linear.x,
                vel_msg.linear.y,
                vel_msg.linear.z,
                vel_msg.angular.x,
                vel_msg.angular.y,
                vel_msg.angular.z,
            )
        )
        self.publisher_.publish(vel_msg)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
