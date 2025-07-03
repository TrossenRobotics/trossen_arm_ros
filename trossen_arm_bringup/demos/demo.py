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
Demo script to send the Trossen Arm arm to a pre-defined position and then back to its home
position using the arm_controller's FollowJointTrajectory action interface.
"""

import math
import time
import rclpy
from rclpy.action import ActionClient
from rclpy.constants import S_TO_NS
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class ArmDemo(Node):
    """Demo node for sending trajectory goals to the Trossen Arm's arm_controller."""
    def __init__(self):
        """Initialize the ArmDemo node."""
        super().__init__('arm_demo')
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
        )
        self._action_client.wait_for_server()
        self.joint_names = [
            'joint_0',
            'joint_1',
            'joint_2',
            'joint_3',
            'joint_4',
            'joint_5',
        ]
        self._is_running = False

    def _feedback_callback(self, feedback_msg: FollowJointTrajectory.Impl.FeedbackMessage):
        """
        Log the current joint positions from the action server's feedback.
        """
        feedback: FollowJointTrajectory.Feedback = feedback_msg.feedback
        if feedback.joint_names != self.joint_names:
            self.get_logger().warn(
                'Received feedback with unexpected joint names: %s',
                feedback.joint_names
            )
            return

        self.get_logger().info(f'Current positions: {feedback.actual.positions}')
        self.get_logger().info(f'Desired positions: {feedback.desired.positions}')

    def _get_result_callback(self, future):
        """Log the action server's result and marks the operation as complete."""
        result: FollowJointTrajectory.Result = future.result().result
        self.get_logger().info(f'Goal completed with status: {result.error_code}')
        self._is_running = False

    def _goal_response_callback(self, future):
        """Handle the response from sending a goal to the action server."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().fatal('Goal rejected!')
            exit()

        self.get_logger().info('Goal accepted!')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._get_result_callback)

    def send_goal(self, positions: list[float], duration_s: float = 2.0):
        """
        Send a trajectory goal to the arm_controller.

        :param positions: Target joint positions (radians) for the arm.
        :param duration_s: Time in seconds to reach the target position.
        """
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(duration_s)
        point.time_from_start.nanosec = int((duration_s % 1) * S_TO_NS)
        goal_msg.trajectory.points.append(point)

        future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )

        self._is_running = True
        future.add_done_callback(self._goal_response_callback)

        return future


def main(args=None):
    rclpy.init(args=args)
    node = ArmDemo()

    target_position = [0.0, math.pi / 2.0, math.pi / 2.0, 0.0, 0.0, 0.0]
    home_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # Send to target position
    node.get_logger().info('Sending arm to target position...')
    future = node.send_goal(target_position, duration_s=2.0)
    rclpy.spin_until_future_complete(node, future)

    while node._is_running:
        rclpy.spin_once(node)

    node.get_logger().info('Reached target position.')

    # Wait for a moment before sending the next goal
    time.sleep(1.0)

    # Send to home position
    node.get_logger().info('Sending arm to home position...')
    future = node.send_goal(home_position, duration_s=2.0)
    rclpy.spin_until_future_complete(node, future)

    while node._is_running:
        rclpy.spin_once(node)

    node.get_logger().info('Reached home position.')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
