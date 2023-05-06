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
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from rclpy.task import Future
from std_srvs.srv import SetBool, Trigger


class Manager(Node):
    """Provides custom controllers with an interface to the BlueROV2 thrusters."""

    STOPPED_PWM = 1500

    def __init__(self) -> None:
        """Create a new control manager."""
        super().__init__("blue_manager")

        self.connected = False
        self.armed = False

        self.declare_parameters("", [("rov_config", "heavy"), ("backup_timeout", 5.0)])

        rov_config = self.get_parameter("rov_config").get_parameter_value().string_value
        self.backup_timeout = (
            self.get_parameter("backup_timeout").get_parameter_value().double_value
        )

        # The BlueROV2 has two configurations: the heavy configuration with 8 thrusters
        # and the standard configuration with 6 thrusters.
        match rov_config:
            case "heavy":
                num_thrusters = 8
            case _:
                num_thrusters = 6

        # Maintain a backup of the thruster parameters so that we can restore them when
        # switching modes
        self.thruster_params_backup: dict[str, Parameter | None] = {
            f"SERVO{i}_FUNCTION": None for i in range(1, num_thrusters + 1)
        }

        # Subscribers
        self.param_event_sub = self.create_subscription(
            ParamEvent, "/mavros/param/event", self.save_default_thruster_params_cb, 10
        )

        # Services
        # TODO(evan): Is exposing this really the most usable interface?
        # this makes it really easy to accidentally override your original parameters
        # It might be better to instead bring back the "connect" functionality or
        # something similar that protects the original backup
        self.backup_params_srv = self.create_service(
            Trigger, "/blue/param/backup", self.backup_thruster_params_cb
        )
        self.restore_backup_params_srv = self.create_service(
            Trigger, "/blue/param/restore_backup", self.restore_backup_params_cb
        )
        self.set_pwm_passthrough_srv = self.create_service(
            SetBool, "/blue/mode/set_pwm_passthrough", self.set_pwm_passthrough_mode_cb
        )

        def wait_for_service(client):
            while not client.wait_for_service(timeout_sec=1.0):
                ...

        # Service clients
        self.get_param_srv_client = self.create_client(
            GetParameters, "/mavros/param/get_parameters"
        )
        wait_for_service(self.get_param_srv_client)

        self.set_param_srv_client = self.create_client(
            SetParameters, "/mavros/param/set_parameters"
        )
        wait_for_service(self.set_param_srv_client)

    @property
    def thruster_params_backed_up(self) -> bool:
        """All required thruster parameters have been successfully backed up.

        Returns:
            Whether or not the thruster parameters have been backed up.
        """
        return None not in self.thruster_params_backup.values()

    def save_default_thruster_params_cb(self, event: ParamEvent) -> None:
        """Save the default thruster parameter values to the parameter backup.

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

    def _request_ardusub_params(self, param_ids: list[str]) -> Future:
        """Request ArduSub parameter values from MAVROS.

        Args:
            param_ids: The names of the parameters whose values should be requested.

        Returns:
            A future object which provides access to the parameter request result.
        """
        return self.get_param_srv_client.call_async(
            GetParameters.Request(names=param_ids)
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

    def backup_thruster_params_cb(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Create a backup of the thruster function parameters.

        The thruster parameters that get backed up include all SERVOn_FUNCTION
        parameters assigned to be motors (determined by the ROV configuration).
        These are modified by the manager to support the PWM
        Passthrough mode, but need to be restored when the manager is shutdown.

        Args:
            request: The request to backup the thruster parameters.
            response: The result of the backup attempt.

        Returns:
            The result of the backup attempt.
        """
        if not self.thruster_params_backed_up:
            # All thruster parameters should be received when MAVROS is populating the
            # parameter server, but we make sure just in case that failed
            unsaved_params = [
                param_id
                for param_id in self.thruster_params_backup.keys()
                if self.thruster_params_backup[param_id] is None
            ]

            def handle_request_future_cb(future: Future, param_ids: list[str]):
                try:
                    response = future.result()
                except Exception:
                    ...
                else:
                    # The parameter values are provided in the same order as the names
                    # were provided in
                    for param_value, param_id in zip(response.values, param_ids):
                        self.thruster_params_backup[param_id] = Parameter(
                            name=param_id,
                            value=param_value,
                        )

            future = self._request_ardusub_params(unsaved_params)
            future.add_done_callback(
                partial(handle_request_future_cb, param_ids=unsaved_params)
            )

            # Wait for the request to finish
            # NOTE: There are a few methods that have been recommended/documented as
            # ways to block until the request is finished including using
            # `spin_until_future_complete` and `while not future.done()`. Unfortunately
            # both of these methods block indefinitely in this scenario, so we just use
            # a timeout instead
            end_t = time.time() + self.backup_timeout
            while time.time() < end_t:
                if self.thruster_params_backed_up:
                    break

                time.sleep(0.1)

        response.success = self.thruster_params_backed_up

        if not response.success:
            response.message = (
                "Failed to backup the thruster parameters. Please verify"
                " that MAVROS has finished populating the parameter server."
            )
        else:
            response.message = "Successfully backed up all thruster parameters!"

        return response

    def restore_backup_params_cb(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        if not self.thruster_params_backed_up:
            response.success = False
            response.message = (
                "The thruster parameters have not yet been successfully backed up."
                " No backup to restore."
            )

            return response

        self.get_logger().info(
            "Attempting to restore the BlueROV2 backup thruster parameters."
        )

        def handle_restore_backup_result(future):
            try:
                response = future.result()
            except Exception:
                ...
            else:
                if all([result.successful for result in response.results]):
                    # TODO(evan): Do something smarter here
                    self.get_logger().info("whooooooooooooooooooooooooo")

        # We would actually prefer to set all of the parameters atomically, but
        # this functionality is not currently supported by MAVROS
        future = self._set_ardusub_params(list(self.thruster_params_backup.values()))
        future.add_done_callback(handle_restore_backup_result)

        # TODO(evan): Delay and then update the response

        response.success = True

        return response

    def set_pwm_passthrough_mode_cb(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        if request.data:
            if not self.thruster_params_backed_up:
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
                        # TODO(evan): Do something smarter here
                        self.get_logger().info("whooooooooooooooooooooooooo")

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
