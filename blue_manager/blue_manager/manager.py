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

import time
from copy import deepcopy

import rclpy
from mavros_msgs.msg import ParamEvent
from mavros_msgs.srv import CommandLong
from rcl_interfaces.msg import Parameter, ParameterType
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from std_srvs.srv import SetBool


class Manager(Node):
    """Provides an interface between custom controllers and the BlueROV2."""

    STOPPED_PWM = 1500

    def __init__(self) -> None:
        """Create a new control manager."""
        super().__init__("blue_manager")

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
        )

        def wait_for_service(client, timeout=1.0) -> None:
            while not client.wait_for_service(timeout_sec=timeout):
                ...

        # Service clients
        self.set_param_srv_client = self.create_client(
            SetParameters, "/mavros/param/set_parameters"
        )
        wait_for_service(self.set_param_srv_client)

        self.command_long_srv_client = self.create_client(
            CommandLong, "/mavros/cmd/long"
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

    def set_pwm(self, thruster: int, pwm: int) -> None:
        """Set the PWM value of a thruster.

        Args:
            thruster: The thruster number to set the PWM value of.
            pwm: The PWM value to set the thruster to.
        """
        self.command_long_srv_client.call_async(
            CommandLong(command=183, confirmation=0, param1=thruster, param2=pwm)
        )

    def stop_thrusters(self) -> None:
        """Stop all thrusters."""
        for i in range(1, self.num_thrusters + 1):
            self.set_pwm(i, self.STOPPED_PWM)

    def enable_pwm_passthrough_mode(self, timeout: float) -> bool:
        """Enable PWM Passthrough mode.

        Args:
            timeout: The maximum amount of time to wait for the mode to be enabled.

        Returns:
            True if the mode was successfully enabled, False otherwise.
        """
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

        def get_enable_passthrough_result(future):
            try:
                response = future.result()
            except Exception:
                ...
            else:
                if all([result.successful for result in response.results]):
                    self.passthrough_enabled = True
                else:
                    self.passthrough_enabled = False

        # We would actually prefer to set all of the parameters atomically, but
        # this functionality is not currently supported by MAVROS
        future = self.set_param_srv_client.call_async(
            SetParameters.Request(parameters=list(passthrough_params.values()))
        )
        future.add_done_callback(get_enable_passthrough_result)

        start_t = time.time()

        while time.time() - start_t < timeout:
            if self.passthrough_enabled:
                break

            time.sleep(0.1)

        return self.passthrough_enabled

    def disable_pwm_passthrough_mode(self, timeout: float) -> bool:
        """Disable PWM Passthrough mode.

        NOTE: This fails when the original thruster parameters could not be restored.

        Args:
            timeout: The maximum amount of time to wait for the mode to be disabled.

        Returns:
            True if the mode was successfully disabled, False otherwise.
        """
        self.get_logger().info("Attempting to disable PWM Passthrough mode.")

        def get_restore_backup_result(future):
            try:
                response = future.result()
            except Exception:
                ...
            else:
                if all([result.successful for result in response.results]):
                    self.passthrough_enabled = False
                else:
                    self.get_logger().warning(
                        "Failed to leave the PWM Passthrough mode. If failure persists,"
                        " the following backup parameters may be restored manually:"
                        f" {self.thruster_params_backup}"
                    )
                    self.passthrough_enabled = False

        future = self.set_param_srv_client.call_async(
            SetParameters.Request(parameters=list(self.thruster_params_backup.values()))
        )
        future.add_done_callback(get_restore_backup_result)

        start_t = time.time()

        while time.time() - start_t < timeout:
            if not self.passthrough_enabled:
                break

            time.sleep(0.1)

        return not self.passthrough_enabled

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

            # Make sure to stop all of the thrusters before-hand to prevent damaging the
            # thrusters when the custom controller takes over
            self.stop_thrusters()

            for _ in range(self.retries):
                response.success = self.enable_pwm_passthrough_mode(
                    timeout=self.timeout
                )
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
                    "The system was already not in the PWM Passthrough mode."
                )
                return response

            # Stop the thrusters to prevent any damage to the thrusters when ArduSub
            # takes back control
            self.stop_thrusters()

            for _ in range(self.retries):
                response.success = self.disable_pwm_passthrough_mode(
                    timeout=self.timeout
                )
                if response.success:
                    break

            if response.success:
                response.message = "Successfully left PWM Passthrough mode."
            else:
                response.message = (
                    "Failed to leave PWM Passthrough mode. Good luck soldier."
                )

        return response


def main(args: list[str] | None = None):
    """Run the ROV manager."""
    rclpy.init(args=args)

    node = Manager()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
