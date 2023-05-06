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
from functools import partial

import rclpy
from mavros_msgs.msg import ParamEvent
from rcl_interfaces.msg import Parameter, ParameterType
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.task import Future
from std_srvs.srv import SetBool, Trigger


class Manager(Node):
    """Provides custom controllers with an interface to the BlueROV2 thrusters."""

    STOPPED_PWM = 1500

    def __init__(self) -> None:
        """Create a new control manager."""
        super().__init__("blue_manager")

        self.passthrough_enabled = False

        self.declare_parameters("", [("num_thrusters", 8)])

        # Maintain a backup of the thruster parameters so that we can restore them when
        # switching modes
        self.thruster_params_backup: dict[str, Parameter | None] = {
            f"SERVO{i}_FUNCTION": None
            for i in range(
                1,
                self.get_parameter("num_thrusters").get_parameter_value().integer_value
                + 1,
            )
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

        # Service clients
        self.set_param_srv_client = self.create_client(
            SetParameters, "/mavros/param/set_parameters"
        )

        while not self.set_param_srv_client.wait_for_service(timeout_sec=1.0):
            ...

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

            # Log this to the terminal in case the backup fails and the user didn't
            # record the default values
            self.get_logger().info(
                f"Saved thruster parameter {event.param_id} with value {event.value}."
            )

            if None not in self.thruster_params_backup.values():
                self.get_logger().info(
                    "Successfully backed up the thruster parameters!"
                )

    def _set_ardusub_params(self, params: list[Parameter]) -> Future:
        """Set ArduSub parameters using MAVROS.

        Args:
            params: The parameter values that should be set.

        Returns:
            A future object which provides access to the parameter setting result.
        """
        return self.set_param_srv_client.call_async(
            SetParameters.Request(parameters=params)
        )

    def set_pwm_passthrough_mode_cb(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        if request.data:
            if None in self.thruster_params_backup.values():
                response.success = False
                response.message = (
                    "The thrusters cannot be set to PWM Passthrough mode without first"
                    " being successfully backed up!"
                )
                return response

            self.get_logger().warning(
                "Attempting to switch to the PWM Passthrough flight mode. All ArduSub"
                " arming and failsafe procedures will be disabled upon success."
            )

            passthrough_params = deepcopy(self.thruster_params_backup)

            # Set the servo mode to "DISABLED"
            # This disables the arming and failsafe features, but now lets us send PWM
            # values to the thrusters without any mixing
            for param in passthrough_params.values():
                param.value.type = ParameterType.PARAMETER_INTEGER  # type: ignore
                param.value.integer_value = 0  # type: ignore

            def get_mode_change_result(future):
                try:
                    response = future.result()
                except Exception:
                    ...
                else:
                    if all([result.successful for result in response.results]):
                        self.get_logger().info(
                            "Successfully switched to PWM Passthrough mode!"
                        )
                        self.passthrough_enabled = True
                    else:
                        self.get_logger().warning(
                            "Failed to switch to PWM Passthrough mode."
                        )
                        self.passthrough_enabled = False

            # We would actually prefer to set all of the parameters atomically, but
            # this functionality is not currently supported by MAVROS
            future = self._set_ardusub_params(list(passthrough_params.values()))
            future.add_done_callback(get_mode_change_result)

            # TODO(evan): Find a better way to get the result of the parameter setting
        else:
            self.restore_backup_params_cb(Trigger.Request(), Trigger.Response())

        response.success = True

        return response


def main(args: list[str] | None = None):
    """Run the ROV manager."""
    rclpy.init(args=args)

    node = Manager()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
