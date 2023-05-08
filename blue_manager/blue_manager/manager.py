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

from copy import deepcopy

import rclpy
from mavros_msgs.msg import ParamEvent
from mavros_msgs.srv import CommandLong
from rcl_interfaces.msg import Parameter
from rcl_interfaces.srv import SetParameters
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import SetBool


class Manager(Node):
    """Provides an interface between custom controllers and the BlueROV2."""

    STOPPED_PWM = 1500

    def __init__(self) -> None:
        """Create a new control manager."""
        super().__init__("blue_manager")

        # We need a reentrant callback group to get synchronous calls to services from
        # within services
        reentrant_callback_group = ReentrantCallbackGroup()

        self.declare_parameters(
            "",
            [
                ("num_thrusters", 8),
                ("mode_change_timeout", 1.0),
                ("mode_change_retries", 3),
            ],
        )

        self.passthrough_enabled = False
        self.num_thrusters = (
            self.get_parameter("num_thrusters").get_parameter_value().integer_value
        )
        self.timeout = (
            self.get_parameter("mode_change_timeout").get_parameter_value().double_value
        )
        self.retries = (
            self.get_parameter("mode_change_retries")
            .get_parameter_value()
            .integer_value
        )
        self.thruster_params_backup: dict[str, Parameter | None] = {
            f"SERVO{i}_FUNCTION": None for i in range(1, self.num_thrusters + 1)
        }

        # Subscribers
        self.param_event_sub = self.create_subscription(
            ParamEvent, "/mavros/param/event", self.backup_thruster_params_cb, 50
        )

        # Services
        self.set_pwm_passthrough_srv = self.create_service(
            SetBool,
            "/blue/manager/enable_passthrough",
            self.set_pwm_passthrough_mode_cb,
            callback_group=reentrant_callback_group,
        )

        def wait_for_service(client, timeout=1.0) -> None:
            while not client.wait_for_service(timeout_sec=timeout):
                ...

        # Service clients
        self.set_param_srv_client = self.create_client(
            SetParameters,
            "/mavros/param/set_parameters",
            callback_group=reentrant_callback_group,
        )
        wait_for_service(self.set_param_srv_client)

        self.command_long_srv_client = self.create_client(
            CommandLong, "/mavros/cmd/long", callback_group=reentrant_callback_group
        )
        wait_for_service(self.command_long_srv_client)

    def backup_thruster_params_cb(self, event: ParamEvent) -> None:
        """Backup the default thruster parameter values.

        MAVROS publishes the parameter values of all ArduSub parameters when
        populating the parameter server. We subscribe to this topic to receive the
        messages at the same time as MAVROS so that we can avoid duplicating requests
        to the FCU.

        Args:
            event: The default parameter loading event triggered by MAVROS.
        """
        if (
            event.param_id in self.thruster_params_backup
            and self.thruster_params_backup[event.param_id] is None
        ):
            self.thruster_params_backup[event.param_id] = Parameter(
                name=event.param_id, value=event.value
            )

            if None not in self.thruster_params_backup.values():
                self.get_logger().info(
                    "Successfully backed up the thruster parameters."
                )

    def set_pwm(self, thruster: int, pwm: int) -> bool:
        """Set the PWM value of a thruster.

        Args:
            thruster: The thruster number to set the PWM value of.
            pwm: The PWM value to set the thruster to.

        Returns:
            The result of the PWM setting call.
        """
        result: CommandLong.Response = self.command_long_srv_client.call(
            CommandLong(command=183, confirmation=0, param1=thruster, param2=pwm)
        )
        return result.success

    def stop_thrusters(self) -> None:
        """Stop all thrusters."""
        self.get_logger().warning("Stopping all BlueROV2 thrusters.")
        for i in range(1, self.num_thrusters + 1):
            for _ in range(self.retries):
                if self.set_pwm(i, self.STOPPED_PWM):
                    break

    def set_thruster_params(self, parameters: list[Parameter]) -> bool:
        """Set the thruster parameters.

        Args:
            parameters: The parameters to set.

        Returns:
            True if the parameters were successfully set, False otherwise.
        """
        # We would actually prefer to set all of the parameters atomically, but
        # this functionality is not currently supported by MAVROS
        response: SetParameters.Response = self.set_param_srv_client.call(
            SetParameters.Request(parameters=parameters)
        )
        return all([result.successful for result in response.results])

    def set_pwm_passthrough_mode_cb(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        """Set the PWM Passthrough mode.

        Args:
            request: The request to enable/disable PWM passthrough mode.
            response: The result of the request.

        Returns:
            The result of the request.
        """
        if request.data:
            if self.passthrough_enabled:
                response.success = True
                response.message = "The system is already in PWM Passthrough mode."
                return response

            if None in self.thruster_params_backup.values():
                response.success = False
                response.message = (
                    "The thrusters cannot be set to PWM Passthrough mode without first"
                    " being successfully backed up."
                )
                return response

            self.stop_thrusters()

            self.get_logger().warning(
                "Attempting to switch to the PWM Passthrough flight mode. All ArduSub"
                " arming and failsafe procedures will be disabled upon success."
            )

            passthrough_params = deepcopy(self.thruster_params_backup)

            # Set the servo mode to "DISABLED"
            # This disables the arming and failsafe features, but now lets us send PWM
            # values to the thrusters without any mixing
            for param in passthrough_params.values():
                param.value.integer_value = 0  # type: ignore

            for _ in range(self.retries):
                self.passthrough_enabled = self.set_thruster_params(
                    list(passthrough_params.values())
                )
                response.success = self.passthrough_enabled

                if response.success:
                    break

            if response.success:
                response.message = "Successfully switched to PWM Passthrough mode."
            else:
                response.message = "Failed to switch to PWM Passthrough mode."
        else:
            if not self.passthrough_enabled:
                response.success = True
                response.message = (
                    "The system was not in the PWM Passthrough mode to start with."
                )
                return response

            self.stop_thrusters()

            self.get_logger().warning("Attempting to disable PWM Passthrough mode.")

            for _ in range(self.retries):
                self.passthrough_enabled = not self.set_thruster_params(
                    list(self.thruster_params_backup.values())
                )
                response.success = not self.passthrough_enabled

                if response.success:
                    break

            if response.success:
                response.message = "Successfully left PWM Passthrough mode."
            else:
                self.get_logger().warning(
                    "Failed to leave the PWM Passthrough mode. If failure persists,"
                    " the following backup parameters may be restored manually using"
                    f" QGC: {self.thruster_params_backup}"
                )
                response.message = (
                    "Failed to leave PWM Passthrough mode. Good luck soldier."
                )

        return response


def main(args: list[str] | None = None):
    """Run the ROV manager."""
    rclpy.init(args=args)

    node = Manager()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor)

    node.destroy_node()
    rclpy.shutdown()
