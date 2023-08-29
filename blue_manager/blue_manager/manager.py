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

from copy import deepcopy

import rclpy
from geographic_msgs.msg import GeoPoint, GeoPointStamped
from mavros_msgs.msg import OverrideRCIn, ParamEvent
from mavros_msgs.srv import CommandHome, MessageInterval
from rcl_interfaces.msg import Parameter, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_default, qos_profile_parameter_events
from std_srvs.srv import SetBool


class Manager(Node):
    """Provides an interface between custom controllers and the BlueROV2."""

    STOPPED_PWM = 1500

    def __init__(self) -> None:
        """Create a new control manager."""
        super().__init__("blue_manager")

        self.declare_parameter("num_thrusters", 8)
        self.declare_parameter("backup_params_file", "")
        self.declare_parameters(
            namespace="message_intervals",
            parameters=[  # type: ignore
                ("ids", [31, 32]),
                ("rates", [100.0, 100.0]),
                ("request_interval", 30.0),
            ],
        )
        self.declare_parameters(
            namespace="mode_change",
            parameters=[("timeout", 1.0), ("retries", 3)],  # type: ignore
        )
        self.declare_parameters(
            namespace="ekf_origin",
            parameters=[  # type: ignore
                ("latitude", 44.65870),
                ("longitude", -124.06556),
                ("altitude", 0.0),
            ],
        )
        self.declare_parameters(
            namespace="home_position",
            parameters=[  # type: ignore
                ("latitude", 44.65870),
                ("longitude", -124.06556),
                ("altitude", 0.0),
                ("yaw", 270.0),
                ("request_interval", 30.0),
            ],
        )

        self.passthrough_enabled = False
        self.num_thrusters = (
            self.get_parameter("num_thrusters").get_parameter_value().integer_value
        )
        self.timeout = (
            self.get_parameter("mode_change.timeout").get_parameter_value().double_value
        )
        self.retries = (
            self.get_parameter("mode_change.retries")
            .get_parameter_value()
            .integer_value
        )

        # We need a reentrant callback group to get synchronous calls to services from
        # within service callbacks.
        reentrant_callback_group = ReentrantCallbackGroup()

        self.thruster_params_backup: dict[str, Parameter | None] = {
            f"SERVO{i}_FUNCTION": None for i in range(1, self.num_thrusters + 1)
        }

        # Try to load the backup parameters from a file
        backup_filepath = (
            self.get_parameter("backup_params_file").get_parameter_value().string_value
        )
        if not self.backup_thruster_params_from_file(backup_filepath):
            self.get_logger().info(
                "Failed to load all thruster parameters from the provided backup file."
                " Attempting to load parameters from MAVROS."
            )
            self.param_event_sub = self.create_subscription(
                ParamEvent,
                "/mavros/param/event",
                self.backup_thruster_params_cb,
                qos_profile_parameter_events,
            )
        else:
            self.get_logger().info(
                "Successfully backed up the thruster parameters from the parameters"
                " file."
            )

        # Publishers
        self.override_rc_in_pub = self.create_publisher(
            OverrideRCIn, "/mavros/rc/override", qos_profile_default
        )
        self.gp_origin_pub = self.create_publisher(
            GeoPointStamped,
            "/mavros/global_position/set_gp_origin",
            qos_profile_default,
        )

        # Services
        self.set_pwm_passthrough_srv = self.create_service(
            SetBool,
            "/blue/cmd/enable_passthrough",
            self.set_rc_passthrough_mode_cb,
            callback_group=reentrant_callback_group,
        )

        # Service clients
        def wait_for_client(client) -> None:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f"Waiting for {client.srv_name}...")

        self.set_param_srv_client = self.create_client(
            SetParameters,
            "/mavros/param/set_parameters",
            callback_group=reentrant_callback_group,
        )
        self.set_message_rates_client = self.create_client(
            MessageInterval, "/mavros/set_message_interval"
        )
        self.set_home_pos_client = self.create_client(
            CommandHome, "/mavros/cmd/set_home"
        )

        wait_for_client(self.set_param_srv_client)
        wait_for_client(self.set_message_rates_client)
        wait_for_client(self.set_home_pos_client)

        # Set the intervals at which the desired MAVLink messages are sent
        self._message_rates_set = False

        def update_rate_set(future):
            self.message_rates_set = future.result().success

        def set_message_rates(
            message_ids: list[int], message_rates: list[float]
        ) -> None:
            if not self.message_rates_set:
                for msg, rate in zip(message_ids, message_rates):
                    future = self.set_message_rates_client.call_async(
                        MessageInterval.Request(message_id=msg, message_rate=rate)
                    )
                    future.add_done_callback(update_rate_set)

        message_request_rate = (
            self.get_parameter("message_intervals.request_interval")
            .get_parameter_value()
            .double_value
        )
        message_ids = list(
            self.get_parameter("message_intervals.ids")
            .get_parameter_value()
            .integer_array_value
        )
        message_rates = list(
            self.get_parameter("message_intervals.rates")
            .get_parameter_value()
            .double_array_value
        )

        # Try setting the message rates until MAVROS indicates that they have been set
        self.message_rate_timer = self.create_timer(
            message_request_rate, lambda: set_message_rates(message_ids, message_rates)
        )

        # Set the home position
        self._home_pos_set = False

        def update_home_position_set(future):
            self.home_position_set = future.result().success

        def set_home_pos(lat: float, lon: float, alt: float, yaw: float) -> None:
            if not self.home_position_set:
                future = self.set_home_pos_client.call_async(
                    CommandHome.Request(
                        current_gps=False,
                        yaw=yaw,
                        latitude=lat,
                        longitude=lon,
                        altitude=alt,
                    )
                )
                future.add_done_callback(update_home_position_set)

        # Set the home position. Some folks have discussed that this is necessary for
        # GUIDED mode
        home_lat = (
            self.get_parameter("home_position.latitude")
            .get_parameter_value()
            .double_value
        )
        home_lon = (
            self.get_parameter("home_position.longitude")
            .get_parameter_value()
            .double_value
        )
        home_alt = (
            self.get_parameter("home_position.altitude")
            .get_parameter_value()
            .double_value
        )
        home_yaw = (
            self.get_parameter("home_position.yaw").get_parameter_value().double_value
        )
        hp_request_rate = (
            self.get_parameter("home_position.request_interval")
            .get_parameter_value()
            .double_value
        )

        # Try setting the home position until MAVROS indicates success
        self.message_rate_timer = self.create_timer(
            hp_request_rate,
            lambda: set_home_pos(home_lat, home_lon, home_alt, home_yaw),
        )

        # Now, set the EKF origin. This is necessary to enable GUIDED mode and other
        # autonomy features with ArduSub
        origin_lat = (
            self.get_parameter("ekf_origin.latitude").get_parameter_value().double_value
        )
        origin_lon = (
            self.get_parameter("ekf_origin.longitude")
            .get_parameter_value()
            .double_value
        )
        origin_alt = (
            self.get_parameter("ekf_origin.altitude").get_parameter_value().double_value
        )

        # Normally, we would like to set the QoS policy to use transient local
        # durability, but MAVROS uses the volitile durability setting for its
        # subscriber. Consequently, we need to publish this every once-in-a-while
        # to make sure that it gets set
        self.set_ekf_origin_timer = self.create_timer(
            15.0,
            lambda: self.set_ekf_origin_cb(
                GeoPoint(latitude=origin_lat, longitude=origin_lon, altitude=origin_alt)
            ),
        )

    @property
    def message_rates_set(self) -> bool:
        """Indicate whether or not the message intervals have been properly set.

        Returns:
            Message intervals have been successfully set.
        """
        return self._message_rates_set

    @message_rates_set.setter
    def message_rates_set(self, rates_set: bool) -> None:
        """Update the flag indicating whether or not the message rates have been set.

        Args:
            rates_set: Flag indicating whether or not the message rates have been set.
        """
        if rates_set:
            self.get_logger().info("Successfully set the message rates!")
        else:
            self.get_logger().warn("Failed to set the desired message rates")

        self._message_rates_set = rates_set

    @property
    def home_position_set(self) -> bool:
        """Indicate whether or not the home position has been properly set.

        Returns:
            Home position has been set to the desired value.
        """
        return self._home_pos_set

    @home_position_set.setter
    def home_position_set(self, pos_set: bool) -> None:
        """Update the flag indicating whether or not the home position has been set.

        Args:
            pos_set: Flag indicating whether or not the desired home position was set.
        """
        if pos_set:
            self.get_logger().info("Successfully set the home position!")
        else:
            self.get_logger().warn("Failed to set the home position!")

        self._home_pos_set = pos_set

    @property
    def backup_params_saved(self) -> bool:
        """Whether or not the thruster parameters are backed up.

        Returns:
            Whether or not the parameters are backed up.
        """
        return None not in self.thruster_params_backup.values()

    def backup_thruster_params_from_file(self, backup_filepath: str) -> bool:
        """Backup thruster parameters from a parameters file.

        Args:
            backup_filepath: The full path to the backup file to load.

        Returns:
            Whether or not the parameters were successfully backed up from the file.
        """
        if not backup_filepath:
            return False

        with open(backup_filepath, "r") as ardusub_params:
            params = ardusub_params.readlines()

            for line in params:
                line = line.rstrip()
                split = line.split(" ")

                try:
                    param_id = split[0]
                except IndexError:
                    continue

                try:
                    param_value = int(split[1])
                except (ValueError, IndexError):
                    continue

                if param_id in self.thruster_params_backup.keys():
                    self.thruster_params_backup[param_id] = Parameter(
                        name=param_id,
                        value=ParameterValue(type=2, integer_value=param_value),
                    )

        return self.backup_params_saved

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

            if self.backup_params_saved:
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
            try:
                for param in passthrough_params.values():
                    param.value.integer_value = 1  # type: ignore
            except AttributeError:
                response.success = False
                response.message = (
                    "Failed to switch to RC Passthrough mode. Please ensure that all"
                    " ArduSub parameters have been loaded prior to attempting to"
                    " switch modes."
                )
                return response

            for _ in range(self.retries):
                self.passthrough_enabled = self.set_thruster_params(
                    list(passthrough_params.values())  # type: ignore
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
                    list(self.thruster_params_backup.values())  # type: ignore
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
                    "Failed to disable RC Passthrough mode. Good luck soldier."
                )

        self.get_logger().info(response.message)

        return response

    def set_ekf_origin_cb(self, origin: GeoPoint) -> None:
        """Set the EKF origin.

        This is required for navigation on a vehicle with one of the provided
        localizers.

        Args:
            origin: The EKF origin to set.
        """
        origin_stamped = GeoPointStamped()
        origin_stamped.header.stamp = self.get_clock().now().to_msg()
        origin_stamped.position = origin
        self.gp_origin_pub.publish(origin_stamped)


def main(args: list[str] | None = None):
    """Run the ROV manager."""
    rclpy.init(args=args)

    node = Manager()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor)

    node.destroy_node()
    rclpy.shutdown()
