# Copyright 2023, Evan Palmer
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

from typing import Any

import rclpy
from mavros_msgs.srv import MessageInterval, ParamGet, ParamSetV2
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_srvs.srv import SetBool


class Manager(Node):
    STOPPED_PWM = 1500

    def __init__(self) -> None:
        super().__init__("blue_manager")

        self.battery_state = BatteryState()
        self.connected = False
        self.armed = False
        self.rov_config = "heavy"
        self.backup_params: dict[str, Any] = {}

        # Subscribers
        self.battery_sub = self.create_subscription(
            BatteryState, "/mavros/battery", self._store_battery_state_cb
        )

        # Services
        self.arm_srv = self.create_service(
            SetBool, "/blue/connect", self._manage_connection_cb
        )
        self.connect_srv = self.create_service(
            SetBool, "/blue/arm", self._manage_arming_cb
        )

        # Service clients
        self.set_message_rate_srv_client = self.create_client(
            MessageInterval, "/mavros/set_message_interval", 1
        )
        self.get_param_srv_client = self.create_client(ParamGet, "/mavros/param/get", 1)
        self.set_param_srv_client = self.create_client(
            ParamSetV2, "/mavros/param/set", 1
        )

    def _store_battery_state_cb(self, state: BatteryState) -> None:
        self.battery_state = state

    def check_battery_state(self) -> bool:
        return self.battery_state.percentage <= 0.2

    def _manage_connection_cb(self, connect: bool) -> None:
        if connect:
            self.get_logger().info(
                "Attempting to establish a connection with the BlueROV2..."
            )

            self.connected = self.connect()

            if not self.connected:
                self.get_logger().warning(
                    "Failed to establish a connection with the BlueROV2."
                )
            else:
                self.get_logger().info(
                    "Successfully established a connection with the BlueROV2."
                )
        else:
            self.get_logger().info(
                "Attempting to shutdown the connection to the BlueROV2..."
            )
            self.connected = not self.disconnect()

            if not self.connected:
                self.get_logger().warning(
                    "Successfully shutdown the connection to the BlueROV2."
                )
            else:
                self.get_logger().info("Failed to perform all shutdown proceedures.")

    def connect(self) -> bool:
        ...

    def backup_motor_params(self) -> bool:
        ...

    def disconnect(self) -> bool:
        ...

    def restore_motor_param_backup(self) -> bool:
        ...

    def _manage_arming_cb(self, arm: bool) -> None:
        ...

    def arm(self) -> bool:
        ...

    def disarm(self) -> bool:
        ...


def main(args: list[str] | None = None):
    """Run the ROV manager."""
    rclpy.init(args=args)

    node = Manager()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
