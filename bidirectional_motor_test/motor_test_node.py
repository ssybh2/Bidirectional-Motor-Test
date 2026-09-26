#!/usr/bin/env python3

from __future__ import annotations

import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from custom_msgs.msg import ReadDJIRC, WriteDSHOT


SWITCH_UP = 1
SWITCH_MIDDLE = 3
SWITCH_BOTTOM = 2

MODE_FORWARD = 'FORWARD'
MODE_REVERSE = 'REVERSE'
MODE_DISARM = 'DISARM'


class BidirectionalMotorTest(Node):
    """DJI RC right-switch to bidirectional DSHOT600 command bridge.

    User-requested switch mapping:
      right_switch == 2 (bottom) -> disarm
      right_switch == 3 (middle) -> forward idle / forward throttle
      right_switch == 1 (up)     -> reverse idle / reverse throttle

    The node intentionally inserts a zero-output pause before changing direction.
    """

    def __init__(self) -> None:
        super().__init__('bidirectional_motor_test')

        # Topic / channel parameters.
        self.declare_parameter('input_topic', '/DJIRC')
        self.declare_parameter('output_topic', '/dshot600')
        self.declare_parameter('motor_channel', 1)

        # Direct DSHOT 3D ranges.
        # High range: 1048..2047, low range: 48..1047.
        self.declare_parameter('forward_idle_dshot', 1100)
        self.declare_parameter('forward_max_dshot', 1300)
        self.declare_parameter('reverse_idle_dshot', 100)
        self.declare_parameter('reverse_max_dshot', 300)
        self.declare_parameter('invert_direction', False)

        # Optional speed control. Disabled by default: armed modes only idle.
        self.declare_parameter('use_throttle_axis', False)
        self.declare_parameter('throttle_axis', 'left_y')
        self.declare_parameter('throttle_invert', False)
        self.declare_parameter('throttle_deadband', 0.05)
        self.declare_parameter('throttle_expo', 1.0)

        # Safety parameters.
        self.declare_parameter('direction_change_pause_sec', 1.0)
        self.declare_parameter('rc_timeout_sec', 0.25)
        self.declare_parameter('require_disarm_before_arm', True)
        self.declare_parameter('publish_rate_hz', 50.0)

        self.input_topic = str(self.get_parameter('input_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.motor_channel = int(self.get_parameter('motor_channel').value)

        self.forward_idle = int(self.get_parameter('forward_idle_dshot').value)
        self.forward_max = int(self.get_parameter('forward_max_dshot').value)
        self.reverse_idle = int(self.get_parameter('reverse_idle_dshot').value)
        self.reverse_max = int(self.get_parameter('reverse_max_dshot').value)
        self.invert_direction = bool(self.get_parameter('invert_direction').value)

        self.use_throttle_axis = bool(self.get_parameter('use_throttle_axis').value)
        self.throttle_axis = str(self.get_parameter('throttle_axis').value)
        self.throttle_invert = bool(self.get_parameter('throttle_invert').value)
        self.throttle_deadband = float(self.get_parameter('throttle_deadband').value)
        self.throttle_expo = float(self.get_parameter('throttle_expo').value)

        self.direction_pause_sec = float(
            self.get_parameter('direction_change_pause_sec').value
        )
        self.rc_timeout_sec = float(self.get_parameter('rc_timeout_sec').value)
        self.require_disarm_before_arm = bool(
            self.get_parameter('require_disarm_before_arm').value
        )
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self._validate_parameters()

        self.publisher = self.create_publisher(
            WriteDSHOT,
            self.output_topic,
            qos_profile_sensor_data,
        )
        self.subscription = self.create_subscription(
            ReadDJIRC,
            self.input_topic,
            self._rc_callback,
            qos_profile_sensor_data,
        )

        self.latest_rc: Optional[ReadDJIRC] = None
        self.last_rc_ns: Optional[int] = None

        # Interlock: by default the operator must explicitly show DISARM once.
        self.disarm_seen = not self.require_disarm_before_arm

        # The last direction in which a non-zero DSHOT command was sent.
        self.last_motion_mode: Optional[str] = None

        # Timestamp at which output became zero. Used to guarantee a real
        # zero-command interval before entering the opposite direction.
        self.zero_since_ns: Optional[int] = self.get_clock().now().nanoseconds

        self.last_output_value: Optional[int] = None
        self.last_status: Optional[str] = None

        self.timer = self.create_timer(
            1.0 / self.publish_rate_hz,
            self._control_tick,
        )

        # Publish zero immediately on startup.
        self._publish_dshot(0, 'startup: waiting for RC and DISARM')

        self.get_logger().info(
            f'Ready. RC: {self.input_topic}, DSHOT: {self.output_topic}, '
            f'channel: {self.motor_channel}. '
            'right_switch 2=DISARM, 3=FORWARD, 1=REVERSE.'
        )

    def _validate_parameters(self) -> None:
        if self.motor_channel not in (1, 2, 3, 4):
            raise ValueError('motor_channel must be 1, 2, 3, or 4')

        if not (1048 <= self.forward_idle <= self.forward_max <= 2047):
            raise ValueError(
                'forward DSHOT values must satisfy '
                '1048 <= forward_idle_dshot <= forward_max_dshot <= 2047'
            )

        if not (48 <= self.reverse_idle <= self.reverse_max <= 1047):
            raise ValueError(
                'reverse DSHOT values must satisfy '
                '48 <= reverse_idle_dshot <= reverse_max_dshot <= 1047'
            )

        allowed_axes = {'left_x', 'left_y', 'right_x', 'right_y', 'dial'}
        if self.throttle_axis not in allowed_axes:
            raise ValueError(
                f'throttle_axis must be one of {sorted(allowed_axes)}'
            )

        if not (0.0 <= self.throttle_deadband < 1.0):
            raise ValueError('throttle_deadband must be in [0.0, 1.0)')

        if self.throttle_expo <= 0.0:
            raise ValueError('throttle_expo must be > 0')

        if self.direction_pause_sec < 0.0:
            raise ValueError('direction_change_pause_sec must be >= 0')

        if self.rc_timeout_sec <= 0.0:
            raise ValueError('rc_timeout_sec must be > 0')

        if self.publish_rate_hz <= 0.0:
            raise ValueError('publish_rate_hz must be > 0')

    def _rc_callback(self, msg: ReadDJIRC) -> None:
        self.latest_rc = msg
        self.last_rc_ns = self.get_clock().now().nanoseconds

        # Only a valid online RC is allowed to satisfy the startup/recovery
        # disarm interlock.
        if msg.online == 1 and msg.right_switch == SWITCH_BOTTOM:
            self.disarm_seen = True

    def _control_tick(self) -> None:
        now_ns = self.get_clock().now().nanoseconds

        if self.latest_rc is None or self.last_rc_ns is None:
            self._publish_dshot(0, 'waiting for first DJIRC message')
            return

        age_sec = (now_ns - self.last_rc_ns) / 1e9
        if age_sec > self.rc_timeout_sec:
            self._on_link_fault('RC timeout -> DISARM')
            return

        if self.latest_rc.online != 1:
            self._on_link_fault('DJIRC offline')
            return

        desired_mode = self._switch_to_mode(self.latest_rc.right_switch)

        if desired_mode == MODE_DISARM:
            self.disarm_seen = True
            self._publish_dshot(0, 'DISARM: right_switch=2')
            return

        if desired_mode is None:
            self._publish_dshot(
                0,
                f'invalid right_switch={self.latest_rc.right_switch}',
            )
            return

        if self.require_disarm_before_arm and not self.disarm_seen:
            self._publish_dshot(
                0,
                'interlock: move right_switch to 2 (DISARM) first',
            )
            return

        # Direction-change protection:
        # if the requested direction differs from the last non-zero direction,
        # keep output at zero until direction_change_pause_sec has elapsed.
        if (
            self.last_motion_mode is not None
            and desired_mode != self.last_motion_mode
        ):
            if self.zero_since_ns is None:
                self._publish_dshot(
                    0,
                    f'direction change {self.last_motion_mode}->{desired_mode}: stopping',
                )
                return

            zero_elapsed_sec = (now_ns - self.zero_since_ns) / 1e9
            if zero_elapsed_sec < self.direction_pause_sec:
                self._publish_dshot(0, 'direction-change pause')
                return

        command = self._command_for_mode(desired_mode, self.latest_rc)
        self.last_motion_mode = desired_mode
        self._publish_dshot(command, desired_mode)

    def _on_link_fault(self, reason: str) -> None:
        if self.require_disarm_before_arm:
            # After a receiver/network fault, force the operator to explicitly
            # show DISARM again before the motor may restart.
            self.disarm_seen = False
        self._publish_dshot(0, reason)

    @staticmethod
    def _switch_to_mode(right_switch: int) -> Optional[str]:
        if right_switch == SWITCH_BOTTOM:
            return MODE_DISARM
        if right_switch == SWITCH_MIDDLE:
            return MODE_FORWARD
        if right_switch == SWITCH_UP:
            return MODE_REVERSE
        return None

    def _command_for_mode(self, mode: str, rc: ReadDJIRC) -> int:
        throttle = self._throttle_fraction(rc)

        # invert_direction swaps which DSHOT 3D range is associated with the
        # user's logical FORWARD and REVERSE modes.
        use_high_range = (mode == MODE_FORWARD)
        if self.invert_direction:
            use_high_range = not use_high_range

        if use_high_range:
            idle_value = self.forward_idle
            max_value = self.forward_max
        else:
            idle_value = self.reverse_idle
            max_value = self.reverse_max

        return int(round(idle_value + throttle * (max_value - idle_value)))

    def _throttle_fraction(self, rc: ReadDJIRC) -> float:
        if not self.use_throttle_axis:
            return 0.0

        raw = float(getattr(rc, self.throttle_axis))
        if self.throttle_invert:
            raw = -raw

        # This bench-test mapping intentionally uses only the positive half of
        # the selected stick axis. Center and negative values stay at idle.
        if raw <= self.throttle_deadband:
            return 0.0

        normalized = (raw - self.throttle_deadband) / (1.0 - self.throttle_deadband)
        normalized = max(0.0, min(1.0, normalized))
        return normalized ** self.throttle_expo

    def _publish_dshot(self, value: int, status: str) -> None:
        value = int(max(0, min(2047, value)))

        msg = WriteDSHOT()
        msg.channel1 = 0
        msg.channel2 = 0
        msg.channel3 = 0
        msg.channel4 = 0
        setattr(msg, f'channel{self.motor_channel}', value)
        self.publisher.publish(msg)

        now_ns = self.get_clock().now().nanoseconds

        if value == 0:
            if self.last_output_value not in (None, 0):
                self.zero_since_ns = now_ns
            elif self.zero_since_ns is None:
                self.zero_since_ns = now_ns
        else:
            self.zero_since_ns = None

        if value != self.last_output_value or status != self.last_status:
            self.get_logger().info(
                f'{status}: channel{self.motor_channel}={value}'
            )
            self.last_output_value = value
            self.last_status = status

    def publish_disarm(self) -> None:
        self._publish_dshot(0, 'shutdown: DISARM')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BidirectionalMotorTest()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Give DDS several zero commands before tearing the node down.
        for _ in range(5):
            node.publish_disarm()
            time.sleep(0.01)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
