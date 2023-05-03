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
from mavros_msgs.srv import MessageInterval, ParamGet
from rclpy.action import ActionServer
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

from blue_msgs.action import Arm, Connect


class Manager(Node):
    def __init__(self) -> None:
        super().__init__("blue_manager")

        self.connected = False
        self.armed = False
        self.backup_settings: dict[str, Any] = {}
        self.battery_charge = 0.0

        # Declare the manager parameters
        self.declare_parameters(
            "",
            [
                ("min_battery_percent", 15.0),
                ("stopped_pwm", 1500),
                ("rov_config", "heavy"),
                ("max_connection_attempts", 3),
                ("connection_attempt_timeout", 1.0),
            ],
        )

        # Load the parameter values
        self.total_battery_cells = (
            self.get_parameter("total_battery_cells")
            .get_parameter_value()
            .integer_value
        )
        self.min_battery_percent = (
            self.get_parameter("min_battery_percent").get_parameter_value().double_value
        )
        self.rov_config = (
            self.get_parameter("rov_config").get_parameter_value().string_value
        )
        self.max_connection_attempts = (
            self.get_parameter("max_connection_attempts")
            .get_parameter_value()
            .integer_value
        )
        self.connection_attempt_timeout = (
            self.get_parameter("connection_attempt_timeout")
            .get_parameter_value()
            .double_value
        )
        self.stopped_pwm = (
            self.get_parameter("stopped_pwm").get_parameter_value().integer_value
        )

        # Service clients
        self.set_message_interval_client = self.create_client(
            MessageInterval, "/mavros/set_message_interval", 1
        )

        # Action servers
        self.connect_action_server = ActionServer(
            self, Connect, "/blue/connect", self._connect_control_manager_cb
        )
        self.arm_action_server = ActionServer(
            self, Arm, "/blue/arm", self._arm_controller_cb
        )

        # Subscribers
        self.battery_sub = self.create_subscription(
            BatteryState, "/mavros/battery", self._update_battery_level_cb
        )

    def _connect_control_manager_cb(
        self, connection_request: Connect.Goal
    ) -> Connect.Result:
        connection_result = Connect.Result()

        if connection_request.connect:
            result = self.connect(
                self.max_connection_attempts, self.connection_attempt_timeout
            )
        else:
            result = self.disconnect(
                self.max_connection_attempts, self.connection_attempt_timeout
            )

        connection_result.success = result

        return connection_result

    def _arm_controller_cb(self, arm_request: Arm.Goal) -> Arm.Result:
        arm_result = Connect.Result()

        if arm_request.arm:
            result = self.arm()
        else:
            result = self.disarm()

        arm_result.success = result

        return arm_result

    def set_message_intervals(self) -> None:
        ...

    def connect(self, max_attempts: int, timeout: float) -> bool:
        for _ in range(max_attempts):
            self.connected = self.backup_motor_params(timeout)

            if self.connected:
                break

        return self.connected

    def disconnect(self, max_attempts: int, timeout: float) -> bool:
        disarmed = False
        backup_restored = False

        for _ in range(max_attempts):
            if not disarmed:
                disarmed = self.disarm()

            if not backup_restored:
                backup_restored = self.restore_param_backup(timeout)

            if disarmed and backup_restored:
                break

        success = disarmed and backup_restored
        self.connected = not success

        return success

    def arm(self) -> bool:
        if not self.check_battery_level():
            self.get_logger().warning(
                f"Failed to arm the system: the current battery level ("
                f" {self.battery_charge}%) is below the minimum battery level."
            )
            return False

        self.armed = self.enable_pwm_passthrough()

        return self.armed

    def disarm(self) -> bool:
        success = self.stop_thrusters() and self.disable_pwm_passthrough()
        self.armed = not success
        return success

    def backup_motor_params(self, timeout: float) -> bool:
        ...

    def restore_param_backup(self, timeout: float) -> bool:
        ...

    def stop_thrusters(self) -> bool:
        ...

    def enable_pwm_passthrough(self) -> bool:
        ...

    def disable_pwm_passthrough(self) -> bool:
        ...

    def check_battery_level(self) -> bool:
        return self.battery_charge < self.min_battery_percent

    def _update_battery_level_cb(self, battery_state: BatteryState) -> float:
        self.battery_charge = battery_state.percentage

        if not self.check_battery_level():
            self.get_logger().warning(
                f"The current battery level ({self.battery_charge * 100}%) is below"
                f" the minimum charge level. Disarming the system."
            )

            self.disarm()


def main(args: list[str] | None = None):
    """Run the ROV manager."""
    rclpy.init(args=args)

    node = Manager()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
