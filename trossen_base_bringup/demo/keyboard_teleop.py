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

import select
import sys
import termios
import tty

import geometry_msgs.msg
import rclpy
import std_msgs.msg

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


def getKey(settings, timeout):
    """Read a single keypress from stdin, or return '' if none arrives in time.

    Args:
        settings (list): Terminal attributes to restore after the raw read (from termios.tcgetattr).
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


def saveTerminalSettings():
    """Capture the current stdin terminal attributes so they can be restored later.

    Returns:
        list: Terminal attributes from termios.tcgetattr(stdin).
    """
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    """Restore previously saved stdin terminal attributes.

    Args:
        old_settings (list): Terminal attributes as returned by saveTerminalSettings().
    """
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def main():
    """Run the keyboard teleop node until Ctrl-C, publishing base and lift velocity commands."""
    settings = saveTerminalSettings()

    rclpy.init()
    node = rclpy.create_node('keyboard_teleop')

    key_timeout = node.declare_parameter('key_timeout', 0.5).value
    linear_speed = node.declare_parameter('linear_speed', 0.3).value
    angular_speed = node.declare_parameter('angular_speed', 0.5).value
    lift_speed = node.declare_parameter('lift_speed', 0.1).value  # m/s, URDF limit is 0.3

    cmd_vel_topic = node.declare_parameter('cmd_vel_topic', '/base_controller/cmd_vel').value
    lift_topic = node.declare_parameter('lift_topic', '/lift_controller/commands').value
    pub = node.create_publisher(geometry_msgs.msg.Twist, cmd_vel_topic, 10)
    lift_pub = node.create_publisher(std_msgs.msg.Float64MultiArray, lift_topic, 10)

    twist = geometry_msgs.msg.Twist()
    lift_cmd = std_msgs.msg.Float64MultiArray()

    try:
        print(msg)
        while True:
            key = getKey(settings, key_timeout)

            x = y = th = 0
            lift = 0
            if key == '\x03':
                break
            elif key in moveBindings:
                x, y, th = moveBindings[key]
            elif key in liftBindings:
                lift = liftBindings[key]
            elif key in speedBindings:
                linear_speed *= speedBindings[key][0]
                angular_speed *= speedBindings[key][1]
                lift_speed *= speedBindings[key][0]
                print(
                    f'linear_speed: {linear_speed:.2f}  angular_speed: {angular_speed:.2f}  '
                    f'lift_speed: {lift_speed:.2f}')

            # Any other key, or no key within key_timeout, stops everything

            twist.linear.x = x * linear_speed
            twist.linear.y = y * linear_speed
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = th * angular_speed
            pub.publish(twist)

            lift_cmd.data = [lift * lift_speed]
            lift_pub.publish(lift_cmd)

    finally:
        pub.publish(geometry_msgs.msg.Twist())
        lift_pub.publish(std_msgs.msg.Float64MultiArray(data=[0.0]))
        restoreTerminalSettings(settings)
        rclpy.shutdown()


if __name__ == '__main__':
    main()
