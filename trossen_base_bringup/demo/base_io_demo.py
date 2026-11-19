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
Read every Trossen base output once and send every command once.

Reads one message from each topic published by trossen_base.launch.py (joint states, IMU,
battery, odometry, laser scan) and prints it. Then drives the base forward and raises the
lift briefly, stopping both afterwards. The robot will move: make sure the area is clear.

Usage (trossen_base.launch.py should be running):
    ros2 run trossen_base_bringup base_io_demo.py

Optional Parameters:
    read_timeout (float, default 5.0): Seconds to wait for each topic before skipping it.
    linear_speed (float, default 0.2): Base forward speed for the write demo, in m/s.
    lift_speed (float, default 0.05): Lift speed for the write demo, in m/s.
    move_duration (float, default 1.0): Seconds to command each motion before stopping.
"""

import math
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.wait_for_message import wait_for_message
from sensor_msgs.msg import BatteryState, Imu, JointState, LaserScan
from std_msgs.msg import Float64MultiArray


def yaw_from_quaternion(q):
    """Return the yaw angle (rad) of a geometry_msgs Quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class BaseIODemo(Node):
    """Node that reads each base topic once and sends each base command once."""

    def __init__(self):
        """Declare parameters and create the command publishers."""
        super().__init__('base_io_demo')

        self.read_timeout = self.declare_parameter('read_timeout', 5.0).value
        self.linear_speed = self.declare_parameter('linear_speed', 0.2).value
        self.lift_speed = self.declare_parameter('lift_speed', 0.05).value
        self.move_duration = self.declare_parameter('move_duration', 1.0).value

        self.cmd_vel_pub = self.create_publisher(Twist, '/base_controller/cmd_vel', 10)
        self.lift_pub = self.create_publisher(Float64MultiArray, '/lift_controller/commands', 10)

    def read_once(self, msg_type, topic, qos=10):
        """Wait for one message on a topic.

        Returns:
            The message, or None if nothing arrived within read_timeout.
        """
        ok, msg = wait_for_message(
            msg_type, self, topic, qos_profile=qos, time_to_wait=self.read_timeout)
        if not ok:
            self.get_logger().warn(f'No message on {topic} within {self.read_timeout:.1f}s')
            return None
        return msg

    def read_all(self):
        """Read and print one message from each base topic."""
        log = self.get_logger()

        joints = self.read_once(JointState, '/joint_states')
        if joints:
            for name, pos, vel in zip(joints.name, joints.position, joints.velocity):
                log.info(f'[joint_states] {name}: position {pos:.3f}, velocity {vel:.3f}')

        imu = self.read_once(Imu, '/imu_sensor_broadcaster/imu')
        if imu:
            w, a = imu.angular_velocity, imu.linear_acceleration
            log.info(
                f'[imu] yaw {yaw_from_quaternion(imu.orientation):.3f} rad, '
                f'angular velocity ({w.x:.3f}, {w.y:.3f}, {w.z:.3f}) rad/s, '
                f'linear acceleration ({a.x:.3f}, {a.y:.3f}, {a.z:.3f}) m/s^2')

        battery = self.read_once(BatteryState, '/battery_state_broadcaster/battery_state')
        if battery:
            log.info(
                f'[battery] {battery.percentage:.0f}%, {battery.voltage:.2f} V, '
                f'{battery.current:.2f} A, {battery.temperature:.1f} C, '
                f'power supply status {battery.power_supply_status}')

        odom = self.read_once(Odometry, '/odometry_broadcaster/odom')
        if odom:
            p, v = odom.pose.pose, odom.twist.twist
            log.info(
                f'[odom] position ({p.position.x:.3f}, {p.position.y:.3f}) m, '
                f'yaw {yaw_from_quaternion(p.orientation):.3f} rad, '
                f'velocity ({v.linear.x:.3f}, {v.linear.y:.3f}) m/s, '
                f'{v.angular.z:.3f} rad/s')

        # Sensor-data QoS is best effort, which also matches reliable publishers
        scan = self.read_once(LaserScan, '/scan', qos=qos_profile_sensor_data)
        if scan:
            valid = [r for r in scan.ranges if scan.range_min <= r <= scan.range_max]
            nearest = f'{min(valid):.2f} m' if valid else 'none in range'
            log.info(f'[scan] {len(scan.ranges)} ranges, nearest obstacle {nearest}')

    def command_for(self, publish, duration):
        """Call publish() at 10 Hz for duration seconds, then stop."""
        end = time.monotonic() + duration
        while time.monotonic() < end:
            publish()
            time.sleep(0.1)
        self.stop()

    def write_all(self):
        """Drive the base forward, then raise the lift, stopping after each."""
        log = self.get_logger()

        log.info(f'[cmd_vel] Driving forward at {self.linear_speed:.2f} m/s')
        twist = Twist()
        twist.linear.x = self.linear_speed
        self.command_for(lambda: self.cmd_vel_pub.publish(twist), self.move_duration)

        log.info(f'[lift] Raising lift at {self.lift_speed:.2f} m/s')
        lift = Float64MultiArray(data=[self.lift_speed])
        self.command_for(lambda: self.lift_pub.publish(lift), self.move_duration)

    def stop(self):
        """Publish zero base and lift velocity."""
        self.cmd_vel_pub.publish(Twist())
        self.lift_pub.publish(Float64MultiArray(data=[0.0]))


def main():
    """Run the read demo, then the write demo, stopping the robot on exit or Ctrl-C."""
    # Disable rclpy's SIGINT handler so the stop can still be published after Ctrl-C
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = BaseIODemo()

    try:
        node.read_all()
        node.write_all()
        node.get_logger().info('Done')
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
