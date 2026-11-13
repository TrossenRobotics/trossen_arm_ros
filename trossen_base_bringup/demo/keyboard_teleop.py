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
Keyboard teleoperation for the Trossen swerve base and lift.

Usage:
    ros2 run trossen_base_bringup keyboard_teleop.py

Optional Parameters:
    press_timeout (float, default 0.7): Seconds to wait after a new keypress before
        treating the key as released. Must exceed the OS autorepeat delay.
    repeat_timeout (float, default 0.1): Seconds to wait between autorepeats of a held key
        before treating it as released. Must exceed the OS autorepeat interval.
    linear_speed (float, default 0.3): Base translation speed in m/s.
    angular_speed (float, default 0.5): Base rotation speed in rad/s.
    lift_speed (float, default 0.1): Lift speed in m/s.
    cmd_vel_topic (str, default '/base_controller/cmd_vel'): Topic for base Twist commands.
    lift_topic (str, default '/lift_controller/commands'): Topic for lift velocity commands.
"""

import contextlib
import select
import sys
import termios
import tty

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

msg = """
Keyboard Teleop for SWERVE
---------------------------
u   i   o
j   k   l

i : forward
k : backward
j : strafe left
l : strafe right
u : rotate in place, left
o : rotate in place, right
t : lift up
b : lift down
q/z : increase/decrease max speeds by 10%
Any other key : stop

CTRL-C to quit
"""

# key -> (linear.x, linear.y, angular.z)
moveBindings = {
    'i': (1, 0, 0),
    'k': (-1, 0, 0),
    'j': (0, 1, 0),
    'l': (0, -1, 0),
    'u': (0, 0, 1),
    'o': (0, 0, -1),
}

# key -> lift direction
liftBindings = {
    't': 1,
    'b': -1,
}

# key -> (speed multiplier, turn multiplier)
speedBindings = {
    'q': (1.1, 1.1),
    'z': (0.9, 0.9),
}


def get_key(settings, timeout):
    """Read a single keypress from stdin, or return '' if none arrives in time.

    Args:
        settings (list): Terminal attributes to restore after the raw read.
        timeout (float): Seconds to wait for a keypress before giving up.

    Returns:
        str: The key pressed, or '' on timeout.
    """
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


@contextlib.contextmanager
def raw_terminal():
    """Save the stdin terminal settings and restore them on exit.

    Ensures the terminal is not left in raw mode.

    Yields:
        list: The original terminal settings, to pass to get_key().
    """
    settings = termios.tcgetattr(sys.stdin)
    try:
        yield settings
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


class KeyboardTeleop(Node):
    """Node that turns keypresses into base Twist and lift velocity commands."""

    def __init__(self):
        """Declare the speed, timeout and topic parameters and create the publishers."""
        super().__init__('keyboard_teleop')

        # Terminal does not provide release timeout events so we have to timeout each keypress.
        # We have 2 timeouts since the OS waits about 0.5s until it starts repeating the
        # key (every 0.03s).
        self.press_timeout = self.declare_parameter('press_timeout', 0.7).value
        self.repeat_timeout = self.declare_parameter('repeat_timeout', 0.1).value

        self.linear_speed = self.declare_parameter('linear_speed', 0.3).value
        self.angular_speed = self.declare_parameter('angular_speed', 0.5).value
        self.lift_speed = self.declare_parameter('lift_speed', 0.1).value

        cmd_vel_topic = self.declare_parameter('cmd_vel_topic', '/base_controller/cmd_vel').value
        lift_topic = self.declare_parameter('lift_topic', '/lift_controller/commands').value
        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.lift_pub = self.create_publisher(Float64MultiArray, lift_topic, 10)

        self.last_key = ''
        self.held = False  # True once the current key has started autorepeating

    def handle_key(self, key):
        """Publish base and lift commands for a key ('' meaning no key arrived in time).

        Any unbound key, or no key, publishes a stop.

        Args:
            key (str): The key read from the terminal, or '' on timeout.
        """
        self.held = key != '' and key == self.last_key
        self.last_key = key

        x = y = th = 0
        lift = 0
        if key in moveBindings:
            x, y, th = moveBindings[key]
        elif key in liftBindings:
            lift = liftBindings[key]
        elif key in speedBindings:
            self.linear_speed *= speedBindings[key][0]
            self.angular_speed *= speedBindings[key][1]
            self.lift_speed *= speedBindings[key][0]
            print(
                f'linear_speed: {self.linear_speed:.2f}  '
                f'angular_speed: {self.angular_speed:.2f}  '
                f'lift_speed: {self.lift_speed:.2f}')

        twist = Twist()
        twist.linear.x = x * self.linear_speed
        twist.linear.y = y * self.linear_speed
        twist.angular.z = th * self.angular_speed
        self.cmd_vel_pub.publish(twist)
        self.lift_pub.publish(Float64MultiArray(data=[lift * self.lift_speed]))

    def stop(self):
        """Publish zero base and lift velocity."""
        self.cmd_vel_pub.publish(Twist())
        self.lift_pub.publish(Float64MultiArray(data=[0.0]))

    def run(self):
        """Read keys from the terminal and publish commands until Ctrl-C.

        Always publishes a stop and restores the terminal on exit.
        """
        with raw_terminal() as settings:
            try:
                print(msg)
                while True:
                    timeout = self.repeat_timeout if self.held else self.press_timeout
                    key = get_key(settings, timeout)
                    if key == '\x03':  # Ctrl-C arrives as a character in raw mode
                        break
                    self.handle_key(key)
            finally:
                self.stop()


def main():
    """Run the keyboard teleop node until Ctrl-C, publishing base and lift velocity commands."""
    rclpy.init()
    node = KeyboardTeleop()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
