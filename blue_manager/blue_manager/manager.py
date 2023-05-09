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
from mavros_msgs.msg import OverrideRCIn, ParamEvent
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

        self.passthrough_enabled = False
        self.num_thrusters = 8
        self.timeout = 1.0
        self.retries = 3

        # Get the ROS parameters from the parameter server
        self._get_ros_params()

        # We need a reentrant callback group to get synchronous calls to services from
        # within service callbacks.
        reentrant_callback_group = ReentrantCallbackGroup()

        self.thruster_params_backup: dict[str, Parameter | None] = {
            f"SERVO{i}_FUNCTION": None for i in range(1, self.num_thrusters + 1)
        }

        # Publishers
        self.override_rc_in_pub = self.create_publisher(
            OverrideRCIn, "/mavros/rc/override", 1
        )

        # Subscribers
        self.param_event_sub = self.create_subscription(
            ParamEvent, "/mavros/param/event", self.backup_thruster_params_cb, 50
        )

        # Services
        self.set_pwm_passthrough_srv = self.create_service(
            SetBool,
            "/blue/manager/enable_passthrough",
            self.set_rc_passthrough_mode_cb,
            callback_group=reentrant_callback_group,
        )

        # Service clients
        self.set_param_srv_client = self.create_client(
            SetParameters,
            "/mavros/param/set_parameters",
            callback_group=reentrant_callback_group,
        )

        while not self.set_param_srv_client.wait_for_service(timeout_sec=1.0):
            ...

    def _get_ros_params(self) -> None:
        """Get the ROS parameters from the parameter server."""
        self.declare_parameters(
            "",
            [
                ("num_thrusters", self.num_thrusters),
                ("mode_change_timeout", self.timeout),
                ("mode_change_retries", self.retries),
            ],
        )

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

    @property
    def params_successfully_backed_up(self) -> bool:
        """Whether or not the thruster parameters are backed up.

        Returns:
            Whether or not the parameters are backed up.
        """
        return None not in self.thruster_params_backup.values()

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

            if self.params_successfully_backed_up:
                self.get_logger().info(
                    "Successfully backed up the thruster parameters."
                )

    def set_rc(self, pwm: list[int]) -> None:
        """Set the PWM values of the thrusters.

        Args:
            pwm: The PWM values to set the thruster to. The length of the provided list
                must be equal to the number of thrusters.

        Returns:
            The result of the PWM setting call.
        """
        if len(pwm) != self.num_thrusters:
            raise ValueError(
                "The length of the PWM input must equal the number of thrusters."
            )

        # Change the values of only the thruster channels
        channels = pwm + [OverrideRCIn.CHAN_NOCHANGE] * (18 - self.num_thrusters)

        self.override_rc_in_pub.publish(OverrideRCIn(channels=channels))

    def stop_thrusters(self) -> None:
        """Stop all thrusters."""
        self.get_logger().warning("Stopping all BlueROV2 thrusters.")
        self.set_rc([self.STOPPED_PWM] * self.num_thrusters)

    def set_thruster_params(self, params: list[Parameter]) -> bool:
        """Set the thruster parameters.

        Args:
            params: The ArduSub parameters to set.

        Returns:
            True if the parameters were successfully set, False otherwise.
        """
        # We would actually prefer to set all of the parameters atomically, but
        # this functionality is not currently supported by MAVROS
        response: SetParameters.Response = self.set_param_srv_client.call(
            SetParameters.Request(parameters=params)
        )
        return all([result.successful for result in response.results])

    def set_rc_passthrough_mode_cb(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        """Set the RC Passthrough mode.

        RC Passthrough mode enables users to control the BlueROV2 thrusters directly
        using the RC channels. It is important that users disable their RC transmitter
        prior to enabling RC Passthrough mode to avoid sending conflicting commands to
        the thrusters.

        Args:
            request: The request to enable/disable RC passthrough mode.
            response: The result of the request.

        Returns:
            The result of the request.
        """
        if request.data:
            if self.passthrough_enabled:
                response.success = True
                response.message = "The system is already in RC Passthrough mode."
                return response

            if not self.thruster_params_backup:
                response.success = False
                response.message = (
                    "The thrusters cannot be set to RC Passthrough mode without first"
                    " being successfully backed up."
                )
                return response

            self.get_logger().warning(
                "Attempting to switch to the RC Passthrough flight mode. All ArduSub"
                " arming and failsafe procedures will be disabled upon success."
            )

            passthrough_params = deepcopy(self.thruster_params_backup)

            # Set the servo mode to "RC Passthrough"
            # This disables the arming and failsafe features, but now lets us send PWM
            # values to the thrusters without any mixing
            for param in passthrough_params.values():
                param.value.integer_value = 1  # type: ignore

            for _ in range(self.retries):
                self.passthrough_enabled = self.set_thruster_params(
                    list(passthrough_params.values())
                )
                response.success = self.passthrough_enabled

                if response.success:
                    break

            if response.success:
                response.message = "Successfully switched to RC Passthrough mode."

                self.stop_thrusters()
            else:
                response.message = "Failed to switch to RC Passthrough mode."
        else:
            if not self.thruster_params_backup:
                response.success = False
                response.message = (
                    "The thruster backup parameters have not yet been stored."
                )

            if not self.passthrough_enabled:
                response.success = True
                response.message = (
                    "The system was not in the RC Passthrough mode to start with."
                )
                return response

            self.stop_thrusters()

            self.get_logger().warning("Attempting to disable RC Passthrough mode.")

            for _ in range(self.retries):
                self.passthrough_enabled = not self.set_thruster_params(
                    list(self.thruster_params_backup.values())
                )
                response.success = not self.passthrough_enabled

                if response.success:
                    break

            if response.success:
                response.message = "Successfully left RC Passthrough mode."
            else:
                self.get_logger().warning(
                    "Failed to leave the RC Passthrough mode. If failure persists,"
                    " the following backup parameters may be restored manually using"
                    f" QGC: {self.thruster_params_backup}"
                )
                response.message = (
                    "Failed to leave RC Passthrough mode. Good luck soldier."
                )

        self.get_logger().info(response.message)

        return response


def main(args: list[str] | None = None):
    """Run the ROV manager."""
    rclpy.init(args=args)

    node = Manager()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor)

    node.destroy_node()
    rclpy.shutdown()
