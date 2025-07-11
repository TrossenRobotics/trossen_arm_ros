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

import math
import time

import rclpy

from controllers import ArmDemoNode, GripperDemoNode  # noqa: I100


def main(args=None):
    rclpy.init(args=args)
    arm = ArmDemoNode(action_name='arm_controller/follow_joint_trajectory')
    gripper = GripperDemoNode(action_name='gripper_controller/gripper_cmd')

    arm_target_position = [0.0, math.pi / 2.0, math.pi / 2.0, 0.0, 0.0, 0.0]  # upright position
    arm_home_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    gripper_open_position = 0.04  # fully open
    gripper_closed_position = 0.0

    # Send to target position
    arm.get_logger().info('Sending arm to target position...')
    future = arm.send_goal(arm_target_position, duration_s=2.0)
    rclpy.spin_until_future_complete(arm, future)
    while arm._is_running:
        rclpy.spin_once(arm)
    arm.get_logger().info('Reached target position.')

    # Open gripper
    gripper.get_logger().info('Opening gripper...')
    future = gripper.send_goal(gripper_open_position)
    rclpy.spin_until_future_complete(gripper, future)
    while gripper._is_running:
        rclpy.spin_once(gripper)
    gripper.get_logger().info('Gripper opened.')

    time.sleep(1.0)

    # Close gripper
    gripper.get_logger().info('Closing gripper...')
    future = gripper.send_goal(gripper_closed_position)
    rclpy.spin_until_future_complete(gripper, future)
    while gripper._is_running:
        rclpy.spin_once(gripper)
    gripper.get_logger().info('Gripper closed.')

    time.sleep(1.0)

    # Send to home position
    arm.get_logger().info('Sending arm to home position...')
    future = arm.send_goal(arm_home_position, duration_s=2.0)
    rclpy.spin_until_future_complete(arm, future)
    while arm._is_running:
        rclpy.spin_once(arm)
    arm.get_logger().info('Reached home position.')

    arm.destroy_node()
    gripper.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
