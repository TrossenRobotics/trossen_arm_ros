#!/usr/bin/env python3

# Copyright 2025 Trossen Robotics
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Joystick teleoperation for the Trossen swerve base and lift. This requires connecting
a game controller.

Subscribes to the joy topic and converts the axes/buttons into Twist and lift velocity
commands.

Usage:
    ros2 run joy game_controller_node
    ros2 run trossen_base_bringup joystick_teleop.py

Optional Parameters:
    axis_x (int, default 1): Joy axis for forward/backward (left stick vertical).
    axis_y (int, default 0): Joy axis for strafe (left stick horizontal).
    axis_yaw (int, default 3): Joy axis for rotation (right stick horizontal).
    axis_lift_up (int, default 5): Joy axis for raising the lift (R2).
    axis_lift_down (int, default 2): Joy axis for lowering the lift (L2).
    turbo_button (int, default 4): Joy button that multiplies speeds while held (L1).
    linear_speed (float, default 0.5): Base translation speed at full stick, in m/s.
    angular_speed (float, default 1.0): Base rotation speed at full stick, in rad/s.
    turbo_multiplier (float, default 1.5): Speed multiplier while turbo is held.
    lift_speed (float, default 0.1): Lift speed at full trigger, in m/s.
    joy_timeout (float, default 0.5): Seconds without a Joy message before the robot is
        stopped. Must exceed the joy_node publish period (autorepeat_rate, 20 Hz by default).
    cmd_vel_topic (str, default '/base_controller/cmd_vel'): Topic for base Twist commands.
    lift_topic (str, default '/lift_controller/commands'): Topic for lift velocity commands.
    joy_topic (str, default '/joy'): Topic to read Joy messages from.
"""

import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray

msg = """
Joystick Teleop for SWERVE (PS4/PS5 controller)
-------------------------------------------------
Left stick   : translate (up/down = forward/back, left/right = strafe)
Right stick  : rotate in place (horizontal axis)
R2 trigger   : raise lift
L2 trigger   : lower lift
L1           : turbo (boost speed)

CTRL-C to quit
"""


class JoystickTeleop(Node):
    """Node that turns Joy messages into base Twist and lift velocity commands."""

    def __init__(self):
        """Declare axis/button/speed parameters and set up the joy subscriber and publishers."""
        super().__init__('joystick_teleop')

        self.axis_x = self.declare_parameter('axis_x', 1).value            # left stick vertical
        self.axis_y = self.declare_parameter('axis_y', 0).value            # left stick horizontal
        self.axis_yaw = self.declare_parameter('axis_yaw', 2).value        # right stick horizontal
        self.axis_lift_up = self.declare_parameter('axis_lift_up', 5).value      # R2
        self.axis_lift_down = self.declare_parameter('axis_lift_down', 4).value  # L2
        self.turbo_button = self.declare_parameter('turbo_button', 9).value      # L1

        self.linear_speed = self.declare_parameter('linear_speed', 0.5).value
        self.angular_speed = self.declare_parameter('angular_speed', 1.0).value
        self.turbo_multiplier = self.declare_parameter('turbo_multiplier', 1.5).value
        self.lift_speed = self.declare_parameter('lift_speed', 0.1).value

        cmd_vel_topic = self.declare_parameter('cmd_vel_topic', '/base_controller/cmd_vel').value
        lift_topic = self.declare_parameter('lift_topic', '/lift_controller/commands').value
        joy_topic = self.declare_parameter('joy_topic', '/joy').value

        # Deadman: if joy_node stops publishing, stop the robot
        self.joy_timeout = self.declare_parameter('joy_timeout', 0.5).value
        self.last_joy_time = None
        self.moving = False  # True while the last published command was non-zero

        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.lift_pub = self.create_publisher(Float64MultiArray, lift_topic, 10)
        self.joy_sub = self.create_subscription(Joy, joy_topic, self.joy_callback, 10)
        self.watchdog_timer = self.create_timer(0.1, self.watchdog_callback)

        self.get_logger().info(msg)

    def joy_callback(self, joy):
        """Convert a Joy message into a base Twist and a lift velocity command.

        Args:
            joy (sensor_msgs.msg.Joy): Raw axes/buttons state from joy_node.
        """
        speed = self.linear_speed
        turn = self.angular_speed
        if joy.buttons[self.turbo_button]:
            speed *= self.turbo_multiplier
            turn *= self.turbo_multiplier

        twist = Twist()
        twist.linear.x = joy.axes[self.axis_x] * speed
        twist.linear.y = joy.axes[self.axis_y] * speed
        twist.angular.z = joy.axes[self.axis_yaw] * turn
        self.cmd_vel_pub.publish(twist)

        # Trigger axes rest at 1.0 and go to -1.0 when fully pressed.
        raise_amount = max(0.0, (1.0 - joy.axes[self.axis_lift_up]) / 2.0)
        lower_amount = max(0.0, (1.0 - joy.axes[self.axis_lift_down]) / 2.0)

        lift_vel = (raise_amount - lower_amount) * self.lift_speed
        self.lift_pub.publish(Float64MultiArray(data=[lift_vel]))

        self.last_joy_time = time.monotonic()
        self.moving = any([twist.linear.x, twist.linear.y, twist.angular.z, lift_vel])

    def watchdog_callback(self):
        """Stop the robot if Joy messages stop arriving while it is moving."""
        if not self.moving or self.last_joy_time is None:
            return
        if time.monotonic() - self.last_joy_time > self.joy_timeout:
            self.get_logger().warn(
                f'No Joy message for {self.joy_timeout:.2f}s, stopping the robot')
            self.stop()

    def stop(self):
        """Publish zero base and lift velocity."""
        self.cmd_vel_pub.publish(Twist())
        self.lift_pub.publish(Float64MultiArray(data=[0.0]))
        self.moving = False


def main():
    """Run the joystick teleop node until Ctrl-C, then stop the robot."""
    # Disable rclpy's SIGINT handler so we can ctrl-c
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = JoystickTeleop()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
