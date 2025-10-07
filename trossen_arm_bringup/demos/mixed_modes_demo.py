#!/usr/bin/env python3

# Copyright 2025 Trossen Robotics
# (License header omitted here for brevity; replicate from other demos if required.)

"""Mixed modes demo: arm in position trajectory mode, gripper commanded in effort mode.

Style matches other demo scripts (single_arm_demo, dual_arm_demo): a procedural main that
instantiates minimal helper objects and spins deterministically instead of a monolithic node class.
"""

import math
import time

from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import rclpy

from controllers import ArmDemoNode

GRIPPER_EFFORT_TOPIC = '/gripper_effort_controller/commands'


class GripperEffortPublisher(Node):
    """Demo node for sending effort commands to the Trossen Arm's gripper_effort_controller."""

    def __init__(self, topic: str = GRIPPER_EFFORT_TOPIC):
        super().__init__('gripper_effort_publisher')
        self._pub = self.create_publisher(Float64MultiArray, topic, 10)

    def publish_effort(self, effort: float):
        """Publish an effort to the gripper controller.

        :param effort: Effort to publish in N.
        """
        msg = Float64MultiArray(data=[effort])
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    arm = ArmDemoNode(action_name='arm_controller/follow_joint_trajectory')
    gripper_effort = GripperEffortPublisher()

    arm_target_position = [0.0, math.pi / 2.0, math.pi / 2.0, 0.0, 0.0, 0.0]  # upright position
    arm_home_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    gripper_opening_effort = 10.0  # N
    gripper_closing_effort = -10.0  # N

    # Send to target position
    arm.get_logger().info('Sending arm to target position...')
    future = arm.send_goal(arm_target_position, duration_s=2.0)
    rclpy.spin_until_future_complete(arm, future)
    while arm._is_running:
        rclpy.spin_once(arm)
    arm.get_logger().info('Reached target position.')

    # Open gripper
    gripper_effort.get_logger().info('Opening gripper...')
    gripper_effort.publish_effort(gripper_opening_effort)
    time.sleep(1.0)  # allow time for gripper to open
    gripper_effort.get_logger().info('Gripper opened.')

    time.sleep(1.0)

    # Close gripper
    gripper_effort.get_logger().info('Closing gripper...')
    gripper_effort.publish_effort(gripper_closing_effort)
    time.sleep(1.0)  # allow time for gripper to close
    gripper_effort.get_logger().info('Gripper closed.')

    time.sleep(1.0)

    # Send to home position
    arm.get_logger().info('Sending arm to home position...')
    future = arm.send_goal(arm_home_position, duration_s=2.0)
    rclpy.spin_until_future_complete(arm, future)
    while arm._is_running:
        rclpy.spin_once(arm)
    arm.get_logger().info('Reached home position.')

    arm.destroy_node()
    gripper_effort.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
