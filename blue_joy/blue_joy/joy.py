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

import rclpy
from geometry_msgs.msg import Twist
from mavros_msgs.msg import OverrideRCIn
from rclpy.node import Node
from rclpy.qos import qos_profile_default
from std_msgs.msg import Bool


class JoyInterface(Node):
    """Interface for sending manual control inputs to a controller."""

    def __init__(self):
        """Create a new joystick manual control interface."""
        super().__init__("joy_interface")

        self.declare_parameter("custom_controller_name", "ismc")
        self.declare_parameters(
            "trims", parameters=[("max_pwm", 1900), ("min_pwm", 1100)]  # type: ignore
        )

        self.min_pwm = (
            self.get_parameter("trims.min_pwm").get_parameter_value().integer_value
        )
        self.max_pwm = (
            self.get_parameter("trims.max_pwm").get_parameter_value().integer_value
        )

        # Keep track of whether or not we are controlling using PWMs or twists
        self.vel_control = False

        # Subscribers
        self.start_vel_cmd_sub = self.create_subscription(
            Bool,
            "/blue/joy_interface/start_vel_control",
            self.start_vel_control_cb,
            qos_profile_default,
        )
        self.start_pwm_cmd_sub = self.create_subscription(
            Bool,
            "/blue/joy_interface/start_pwm_control",
            self.start_pwm_control_cb,
            qos_profile_default,
        )
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            "/blue/joy_interface/cmd_vel",
            self.manual_control_cb,
            qos_profile_default,
        )

        # Publishers
        self.override_rc_in_pub = self.create_publisher(
            OverrideRCIn, "/mavros/rc/override", qos_profile_default
        )

        controller_name = (
            self.get_parameter("custom_controller_name")
            .get_parameter_value()
            .string_value
        )
        self.cmd_vel_pub = self.create_publisher(
            Twist, f"/blue/{controller_name}/cmd_vel", 1  # only publish the latest data
        )

    @staticmethod
    def map_range(
        value: float, in_min: float, in_max: float, out_min: float, out_max: float
    ):
        """Map a value to the provided range.

        Args:
            value: The value to map to the range.
            in_min: The minimum value of the current range.
            in_max: The maximum value of the current range.
            out_min: The minimum value of the mapped range.
            out_max: The maximum value of the mapped range.

        Returns:
            A value mapped from an input range to and output range.
        """
        return (((value - in_min) * (out_max - out_min)) / (in_max - in_min)) + out_min

    def start_vel_control_cb(self, _: Bool) -> None:
        """Direct control inputs to the custom controller.

        Args:
            _: Unused.
        """
        self.vel_control = True

    def start_pwm_control_cb(self, _: Bool) -> None:
        """Direct control inputs to the ArduSub controller.

        Args:
            _: Unused.
        """
        self.vel_control = False

    def manual_control_cb(self, cmd: Twist) -> None:
        """Send the manual control inputs to the desired endpoint.

        Args:
            cmd: The control input to send.
        """
        if self.vel_control:
            self.cmd_vel_pub.publish(cmd)
        else:
            rc = OverrideRCIn()
            channels = [OverrideRCIn.CHAN_NOCHANGE] * 18

            channels[4] = int(
                self.map_range(cmd.linear.x, -1.0, 1.0, self.min_pwm, self.max_pwm)
            )
            channels[5] = int(
                self.map_range(-1 * cmd.linear.y, -1.0, 1.0, self.min_pwm, self.max_pwm)
            )
            channels[2] = int(
                self.map_range(cmd.linear.z, -1.0, 1.0, self.min_pwm, self.max_pwm)
            )
            channels[3] = int(
                self.map_range(
                    -1 * cmd.angular.z, -1.0, 1.0, self.min_pwm, self.max_pwm
                )
            )

            rc = OverrideRCIn()
            rc.channels = channels

            self.override_rc_in_pub.publish(rc)


def main(args: list[str] | None = None):
    """Run the joystick interface."""
    rclpy.init(args=args)

    node = JoyInterface()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
