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
from functools import partial

import rclpy
from mavros_msgs.msg import ParamEvent
from rcl_interfaces.msg import Parameter, ParameterType
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from rclpy.task import Future
from std_srvs.srv import SetBool


class Manager(Node):
    """Provides custom controllers with an interface to the BlueROV2 thrusters."""

    STOPPED_PWM = 1500

    def __init__(self) -> None:
        """Create a new control manager."""
        super().__init__("blue_manager")

        self.connected = False
        self.armed = False

        self.declare_parameters("", [("rov_config", "heavy")])

        rov_config = self.get_parameter("rov_config").get_parameter_value().string_value

        # The BlueROV2 has two configurations: the heavy configuration with 8 thrusters
        # and the standard configuration with 6 thrusters.
        match rov_config:
            case "heavy":
                num_thrusters = 8
            case _:
                num_thrusters = 6

        # Maintain a backup of the thruster parameters so that we can restore them when
        # disconnecting
        self.thruster_params_backup: dict[str, Parameter | None] = {
            f"SERVO{i}_FUNCTION": None for i in range(1, num_thrusters + 1)
        }

        # Subscribers
        self.param_event_sub = self.create_subscription(
            ParamEvent, "/mavros/param/event", self.save_default_thruster_params_cb, 10
        )

        # Services
        self.connect_srv = self.create_service(
            SetBool, "/blue/connect", self.manage_connection_cb
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

            if self.thruster_params_backed_up:
                self.get_logger().info(
                    "Successfully backed up the ArduSub thruster parameters."
                )

    def request_ardusub_params(self, param_ids: list[str]) -> Future:
        """Request parameter values from ArduSub.

        Args:
            param_ids: The names of the parameters whose values should be requested.

        Returns:
            A future object which provides access to the parameter request result.
        """
        return self.get_param_srv_client.call_async(
            GetParameters.Request(names=param_ids)
        )

    def request_thruster_params(self, thruster_param_ids: list[str]) -> None:
        """Request the thruster parameters which will be overwritten.

        The thruster parameters requested include all MOTORn_FUNCTION parameters
        assigned to be motors. These are modified by the manager to support the PWM
        Pass-through mode, but need to be restored when the manager is shutdown.

        Args:
            thruster_param_ids: The names of the thruster parameters whose values
                should be backed up.
        """

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
                        name=param_id, value=param_value
                    )

        future = self.request_ardusub_params(thruster_param_ids)
        future.add_done_callback(
            partial(handle_request_future_cb, param_ids=thruster_param_ids)
        )

    def manage_connection_cb(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        """Connect/disconnect the manager.

        Args:
            request: The connection service request.
            response: The connection service response.

        Returns:
            The connection service response.
        """
        if request.data:
            response.success = self.connect()
        else:
            response.success = True

        # TODO(Evan): This is wrong
        self.connected = response.success

        return response

    def connect(self) -> bool:
        self.get_logger().warning(
            "Attempting to connect the manager to the BlueROV2. All ArduSub arming"
            " and failsafe procedures will be disabled upon successful connection."
        )
        if not self.thruster_params_backed_up:
            # Get the IDs of the parameters that weren't successfully backed up on
            # initialization
            unsaved_params = [
                param_id
                for param_id in self.thruster_params_backup.keys()
                if self.thruster_params_backup[param_id] is None
            ]

            self.request_thruster_params(unsaved_params)

            if not self.thruster_params_backed_up:
                self.get_logger().error(
                    "Connection attempt failed. The system has not finished backing"
                    " up the thruster parameters or the backup process failed."
                )
                return False

        # Set the motors to PWM passthrough mode
        return self.set_pwm_passthrough_mode()

    def set_pwm_passthrough_mode(self) -> bool:
        if not self.thruster_params_backed_up:
            self.get_logger().error(
                "The thrusters cannot be set to PWM Pass-through mode without first"
                " being backed up."
            )
            return False

        passthrough_params = deepcopy(self.thruster_params_backup)

        # Set the servo mode to "DISABLED"
        # This disables the arming and failsafe features, but now lets us send PWM
        # values to the thrusters without any mixing
        for param in passthrough_params.values():
            param.value.integer_value = 0  # type: ignore
            param.value.type = ParameterType.PARAMETER_INTEGER  # type: ignore

        # We would actually prefer to set all of the parameters atomically; however,
        # this functionality is not supported by MAVROS
        set_param_request = SetParameters.Request(
            parameters=list(passthrough_params.values())
        )

        self.set_param_srv_client.call_async(set_param_request).add_done_callback(
            self.this_is_a_test
        )

        return True

    def this_is_a_test(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().info("Service call failed %r" % (e,))
        else:
            self.get_logger().info(f"Parameter set succeed..maybe?: {response}")

    def disconnect(self) -> bool:
        return False

    def restore_motor_param_backup(self) -> bool:
        ...


def main(args: list[str] | None = None):
    """Run the ROV manager."""
    rclpy.init(args=args)

    node = Manager()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
