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

from typing import Callable

import rclpy
from geometry_msgs.msg import Twist
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_default
from sensor_msgs.msg import Joy
from std_srvs.srv import SetBool


class JoyTeleop(Node):
    MIN_PWM = 1100
    MAX_PWM = 1900

    def __init__(self) -> None:
        super().__init__("joy_controller")

        self.declare_parameters(
            "manual_control",
            parameters=[  # type: ignore
                ("group", "axes"),
                ("surge", 1),
                ("sway", 0),
                ("heave", 4),
                ("yaw", 3),
            ],
        )
        self.declare_parameters(
            "adjust_max_velocity",
            parameters=[  # type: ignore
                ("default_velocity", 0.5),
                ("increment", 0.02),
                ("group", "axes"),
                ("id", 7),
            ],
        )
        self.declare_parameters(
            "arming",
            parameters=[  # type: ignore
                ("group", "buttons"),
                ("arm", 7),
                ("disarm", 6),
            ],
        )
        self.declare_parameters(
            "custom_controller_arming",
            parameters=[  # type: ignore
                ("controller_name", "ismc"),
                ("group", "buttons"),
                ("arm", 6),
                ("disarm", 6),
            ],
        )
        self.declare_parameters(
            "modes",
            parameters=[  # type: ignore
                ("group", "buttons"),
                ("manual_mode", 0),
                ("passthrough_mode", 1),
            ],
        )

        self._button_cbs: dict[str, list[tuple[list[int], Callable[..., None]]]] = {}
        self.joy_sub = self.create_subscription(
            Joy, "/joy", self.joy_cb, qos_profile_default
        )
        self.override_rc_in_pub = self.create_publisher(
            OverrideRCIn, "/mavros/rc/override", qos_profile_default
        )

        custom_controller_name = (
            self.get_parameter("custom_controller_arming.controller_name")
            .get_parameter_value()
            .string_value
        )
        self.custom_controller_cmd_vel_pub = self.create_publisher(
            Twist,
            f"/blue/{custom_controller_name}/cmd_vel",
            1,  # only send the latest command
        )

        # Start in manual mode by default
        self._manual_mode_enabled = True
        self._passthrough_mode_enabled = False
        self._custom_controller_enabled = False

        # Setup the services for the joystick
        def wait_for_client(client) -> None:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f"Waiting for {client.srv_name}...")

        self.arm_ardusub_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.set_ardusub_mode_client = self.create_client(SetMode, "/mavros/set_mode")
        self.arm_blue_client = self.create_client(SetBool, "/blue/cmd/arm")
        self.enable_passthrough_mode_client = self.create_client(
            SetBool, "/blue/cmd/enable_passthrough"
        )

        wait_for_client(self.arm_ardusub_client)
        wait_for_client(self.set_ardusub_mode_client)
        wait_for_client(self.arm_blue_client)
        wait_for_client(self.enable_passthrough_mode_client)

        # Setup the button inputs
        self.setup_manual_control_inputs()
        self.setup_max_velocity_inputs()
        self.setup_ardusub_arming_inputs()
        self.setup_custom_controller_arming_inputs()
        self.setup_mode_inputs()

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

    def setup_manual_control_inputs(self) -> None:
        """Configure the manual control inputs."""
        manual_control_group = (
            self.get_parameter("manual_control.group")
            .get_parameter_value()
            .string_value
        )
        surge_id = (
            self.get_parameter("manual_control.surge")
            .get_parameter_value()
            .integer_value
        )
        sway_id = (
            self.get_parameter("manual_control.sway")
            .get_parameter_value()
            .integer_value
        )
        heave_id = (
            self.get_parameter("manual_control.heave")
            .get_parameter_value()
            .integer_value
        )
        yaw_id = (
            self.get_parameter("manual_control.yaw").get_parameter_value().integer_value
        )

        @self.on_joy_inputs(manual_control_group, [surge_id, sway_id, yaw_id, heave_id])
        def manual_control_cb(
            surge: float, sway: float, heave: float, yaw: float
        ) -> None:
            if self._custom_controller_enabled:
                twist = Twist()

                twist.linear.x = self.map_range(
                    surge, -1.0, 1.0, -abs(self._max_velocity), abs(self._max_velocity)
                )
                twist.linear.y = self.map_range(
                    sway, -1.0, 1.0, -abs(self._max_velocity), abs(self._max_velocity)
                )
                twist.linear.z = self.map_range(
                    heave, -1.0, 1.0, -abs(self._max_velocity), abs(self._max_velocity)
                )
                twist.angular.z = self.map_range(
                    yaw, -1.0, 1.0, -abs(self._max_velocity), abs(self._max_velocity)
                )

                self.custom_controller_cmd_vel_pub.publish(twist)
            else:
                channels = [OverrideRCIn.CHAN_NOCHANGE] * 18

                channels[4] = int(
                    self.map_range(surge, -1.0, 1.0, self.MIN_PWM, self.MAX_PWM)
                )
                channels[5] = int(
                    self.map_range(-1 * sway, -1.0, 1.0, self.MIN_PWM, self.MAX_PWM)
                )
                channels[2] = int(
                    self.map_range(heave, -1.0, 1.0, self.MIN_PWM, self.MAX_PWM)
                )
                channels[3] = int(
                    self.map_range(-1 * yaw, -1.0, 1.0, self.MIN_PWM, self.MAX_PWM)
                )

                rc = OverrideRCIn()
                rc.channels = channels

                self.override_rc_in_pub.publish(rc)

    def setup_max_velocity_inputs(self) -> None:
        """Configure the max velocity inputs."""
        adjust_max_velocity_group = (
            self.get_parameter("adjust_max_velocity.group")
            .get_parameter_value()
            .string_value
        )
        adjust_max_velocity_id = (
            self.get_parameter("adjust_max_velocity.id")
            .get_parameter_value()
            .integer_value
        )
        adjust_max_velocity_increment = (
            self.get_parameter("adjust_max_velocity.increment")
            .get_parameter_value()
            .double_value
        )
        self._max_velocity = (
            self.get_parameter("adjust_max_velocity.default_velocity")
            .get_parameter_value()
            .double_value
        )

        @self.on_joy_inputs(adjust_max_velocity_group, [adjust_max_velocity_id])
        def adjust_max_velocity(adjustment: int) -> None:
            self._max_velocity += adjust_max_velocity_increment * adjustment

    def setup_ardusub_arming_inputs(self) -> None:
        arming_group = (
            self.get_parameter("arming.group").get_parameter_value().string_value
        )
        arm_id = self.get_parameter("arming.arm").get_parameter_value().integer_value

        @self.on_joy_inputs(arming_group, [arm_id])
        def arm_cb(arm: bool) -> None:
            if arm:
                self.arm_ardusub_client.call_async(CommandBool.Request(value=True))

        disarm_id = (
            self.get_parameter("arming.disarm").get_parameter_value().integer_value
        )

        @self.on_joy_inputs(arming_group, [disarm_id])
        def disarm_cb(disarm: bool) -> None:
            if disarm:
                self.arm_ardusub_client.call_async(CommandBool.Request(value=False))

    def setup_custom_controller_arming_inputs(self) -> None:
        custom_controller_group_arming = (
            self.get_parameter("custom_controller_arming.group")
            .get_parameter_value()
            .string_value
        )
        arm_custom_controller_id = (
            self.get_parameter("custom_controller_arming.arm")
            .get_parameter_value()
            .integer_value
        )

        def update_custom_controller_armed_cb(future) -> None:
            if future.result().success:
                self._custom_controller_enabled = True

        def update_custom_controller_disarmed_cb(future) -> None:
            if future.result().success:
                self._custom_controller_enabled = False

        @self.on_joy_inputs(custom_controller_group_arming, [arm_custom_controller_id])
        def arm_custom_controller_cb(arm: bool) -> None:
            if arm:
                future = self.arm_blue_client.call_async(SetBool.Request(data=True))
                future.add_done_callback(update_custom_controller_armed_cb)

        disarm_custom_controller_id = (
            self.get_parameter("custom_controller_arming.disarm")
            .get_parameter_value()
            .integer_value
        )

        @self.on_joy_inputs(
            custom_controller_group_arming, [disarm_custom_controller_id]
        )
        def disarm_custom_controller_cb(disarm: bool) -> None:
            if disarm:
                future = self.arm_blue_client.call_async(SetBool.Request(data=False))
                future.add_done_callback(update_custom_controller_disarmed_cb)

    def setup_mode_inputs(self) -> None:
        modes_group = (
            self.get_parameter("modes.group").get_parameter_value().string_value
        )
        manual_mode_id = (
            self.get_parameter("modes.manual_mode").get_parameter_value().integer_value
        )
        passthrough_mode_id = (
            self.get_parameter("modes.passthrough_mode")
            .get_parameter_value()
            .integer_value
        )
        self._passthrough_mode_enabled = False
        self._manual_mode_enabled = False

        @self.on_joy_inputs(modes_group, [manual_mode_id, passthrough_mode_id])
        def on_mode_change(manual_mode: int, passthrough_mode: int) -> None:
            if manual_mode:
                if self._passthrough_mode_enabled:
                    # TODO: Disable passthrough mode
                    # TODO: attach the mode update to the future
                    ...
                else:
                    self._manual_mode_enabled = True
            elif passthrough_mode:
                if self._manual_mode_enabled:
                    # TODO: Enable passthrough mode
                    # TODO: attach the mode update to the future
                    ...
                else:
                    self._passthrough_mode_enabled = True

    def add_button_cb(
        self, group: str, indices: list[int], callback: Callable[..., None]
    ) -> None:
        """Add a callback to an joystick input.

        Args:
            group: The group to map the input to.
            indices: The indices within the group to map the input to.
            callback: The callback to execute using the provided joystick inputs.

        Raises:
            ValueError: The provided indices are already assigned.
        """
        if len(indices) != len(set(indices)):
            raise ValueError(
                f"The provided button indices contain duplicates: {indices}."
            )

        if group not in self._button_cbs:
            self._button_cbs[group] = []
        else:
            # Make sure that the callback buttons assigned have not already been mapped
            occupied = [
                index for callback in self._button_cbs[group] for index in callback[0]
            ]
            if any(map(lambda v: v in occupied, indices)):
                raise ValueError(
                    f"Invalid callback provided: {indices}. Indices are already"
                    " assigned to another callback."
                )

        self._button_cbs[group].append((indices, callback))

    def on_joy_inputs(self, group: str, indices: list[int]) -> Callable[..., None]:
        """Create a new callback for the given joystick group and indices.

        Args:
            group: The joystick group whose indices should be assigned. This should be
                either "axes" or "buttons".
            indices: The indices of the group that whose values should be passed as
                inputs to the callback.

        Returns:
            A decorator that can be used to register the callback.
        """

        def decorator(func: Callable[..., None]) -> None:
            self.add_button_cb(group, indices, func)

        return decorator

    def signal_callbacks(self, joy: Joy, group: str) -> None:
        """Signal the callbacks for a group.

        Args:
            joy: The current joystick state.
            group: The group whose callbacks should be signaled.
        """
        try:
            for cb in self._button_cbs[group]:
                args = []

                for mapping in cb[0]:
                    if group == "axes":
                        args.append(joy.axes[mapping])
                    else:
                        args.append(joy.buttons[mapping])

                cb[1](*args)
        except KeyError:
            ...

    def joy_cb(self, joy: Joy) -> None:
        """Trigger the button callbacks.

        Args:
            joy: The current joystick button states.
        """
        self.signal_callbacks(joy, "axes")
        self.signal_callbacks(joy, "buttons")


def main(args: list[str] | None = None):
    """Run the ROV manager."""
    rclpy.init(args=args)

    node = JoyTeleop()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor)

    node.destroy_node()
    rclpy.shutdown()
