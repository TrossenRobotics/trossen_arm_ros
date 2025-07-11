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
    arm_1 = ArmDemoNode(
        namespace='trossen_arm_1',
        action_name='arm_controller/follow_joint_trajectory',
    )
    gripper_2 = GripperDemoNode(
        namespace='trossen_arm_2',
        action_name='gripper_controller/gripper_cmd',
    )

    arm_target_position = [0.0, math.pi / 2.0, math.pi / 2.0, 0.0, 0.0, 0.0]  # upright position
    arm_home_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    gripper_open_position = 0.04  # fully open
    gripper_closed_position = 0.0

    # Send arm_1 to target position
    arm_1.get_logger().info('Sending arm_1 to target position...')
    future = arm_1.send_goal(arm_target_position, duration_s=2.0)
    rclpy.spin_until_future_complete(arm_1, future)
    while arm_1._is_running:
        rclpy.spin_once(arm_1)
    arm_1.get_logger().info('Reached target position.')

    # Open gripper_2
    gripper_2.get_logger().info('Opening gripper_2...')
    future = gripper_2.send_goal(gripper_open_position)
    rclpy.spin_until_future_complete(gripper_2, future)
    while gripper_2._is_running:
        rclpy.spin_once(gripper_2)
    gripper_2.get_logger().info('Gripper opened.')

    time.sleep(1.0)

    # Close gripper_2
    gripper_2.get_logger().info('Closing gripper_2...')
    future = gripper_2.send_goal(gripper_closed_position)
    rclpy.spin_until_future_complete(gripper_2, future)
    while gripper_2._is_running:
        rclpy.spin_once(gripper_2)
    gripper_2.get_logger().info('Gripper closed.')

    time.sleep(1.0)

    # Send arm_1 to home position
    arm_1.get_logger().info('Sending arm_1 to home position...')
    future = arm_1.send_goal(arm_home_position, duration_s=2.0)
    rclpy.spin_until_future_complete(arm_1, future)
    while arm_1._is_running:
        rclpy.spin_once(arm_1)
    arm_1.get_logger().info('Reached home position.')

    arm_1.destroy_node()
    gripper_2.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
