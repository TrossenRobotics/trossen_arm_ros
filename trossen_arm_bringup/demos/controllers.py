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
Demo controller action clients for the trossen_arm_bringup package.

These modules demonstrate how to control the Trossen Arm's arm and gripper using the action
interfaces provided by the arm_controller and gripper_controller.
"""

import math
import time

from control_msgs.action import GripperCommand, FollowJointTrajectory
import rclpy
from rclpy.action import ActionClient
from rclpy.constants import S_TO_NS
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint


class ArmDemoNode(Node):
    """Demo node for sending trajectory goals to the Trossen Arm's arm_controller."""

    def __init__(
        self,
        namespace: str = '',
        action_name: str = '/arm_controller/follow_joint_trajectory',
    ):
        """
        Initialize the ArmDemo node and connect to the arm_controller action server.

        :param namespace: Optional namespace to prepend to the action name and joint names.
        :param action_name: Optional name of the FollowJointTrajectory action server.
        """
        super().__init__('arm_demo')
        self.joint_names = [
            'joint_0',
            'joint_1',
            'joint_2',
            'joint_3',
            'joint_4',
            'joint_5',
        ]

        if namespace:
            action_name = f'{namespace}/{action_name}'
            for i in range(len(self.joint_names)):
                self.joint_names[i] = f'{namespace}/{self.joint_names[i]}'

        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            action_name=action_name,
        )
        while not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info(
                f"Waiting for '{self._action_client._action_name}' action server..."
            )
        self._is_running = False
        self.get_logger().info(f'ArmDemo initialized with action server: {action_name}')

    def _feedback_callback(self, feedback_msg: FollowJointTrajectory.Impl.FeedbackMessage):
        """
        Log the current joint positions from the action server's feedback.

        :param feedback_msg: Feedback message from the action server.
        """
        feedback: FollowJointTrajectory.Feedback = feedback_msg.feedback
        self.get_logger().info(f'Current positions: {feedback.actual.positions}')
        self.get_logger().info(f'Desired positions: {feedback.desired.positions}')

    def _get_result_callback(self, future):
        """
        Log the action server's result and mark the operation as complete.

        :param future: Future object containing the result from the action server.
        """
        result: FollowJointTrajectory.Result = future.result().result
        self.get_logger().info(f'Goal completed with result: {result}')
        self._is_running = False

    def _goal_response_callback(self, future):
        """
        Handle the response from sending a goal to the action server.

        :param future: Future object containing the goal handle.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._is_running = False
            fatal_msg = 'Gripper goal rejected!'
            self.get_logger().fatal(fatal_msg)
            raise RuntimeError(fatal_msg)

        self.get_logger().info('Goal accepted!')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._get_result_callback)

    def send_goal(self, positions: list[float], duration_s: float = 2.0):
        """
        Send a trajectory goal to the arm_controller.

        :param positions: Target joint positions (radians) for the arm.
        :param duration_s: Time in seconds to reach the target position.
        :return: Future object for the goal request.
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


class GripperDemoNode(Node):
    """Demo node for sending gripper commands to the Trossen Arm's gripper_controller."""

    def __init__(
        self,
        namespace: str = '',
        action_name: str = '/gripper_controller/gripper_cmd',
    ):
        """
        Initialize the GripperDemo node and connect to the gripper_controller action server.

        :param namespace: Optional namespace to prepend to the action name.
        :param action_name: Optional name of the GripperCommand action server.
        """
        super().__init__('gripper_demo')

        if namespace:
            action_name = f'{namespace}/{action_name}'

        self._action_client = ActionClient(
            self,
            GripperCommand,
            action_name,
        )
        while not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info(
                f"Waiting for '{self._action_client._action_name}' action server..."
            )
        self._is_running = False
        self.get_logger().info(f'GripperDemo initialized with action server: {action_name}')

    def _feedback_callback(self, feedback_msg: GripperCommand.Impl.FeedbackMessage):
        """
        Log the current gripper position and effort from the action server's feedback.

        :param feedback_msg: Feedback message from the action server.
        """
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Gripper position: {feedback.position}, effort: {feedback.effort}')

    def _get_result_callback(self, future):
        """
        Log the action server's result and mark the operation as complete.

        :param future: Future object containing the result from the action server.
        """
        result: GripperCommand.Result = future.result().result
        self.get_logger().info(f'Gripper action completed with result: {result}')
        self._is_running = False

    def _goal_response_callback(self, future):
        """
        Handle the response from sending a goal to the gripper action server.

        :param future: Future object containing the goal handle.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._is_running = False
            fatal_msg = 'Gripper goal rejected!'
            self.get_logger().fatal(fatal_msg)
            raise RuntimeError(fatal_msg)

        self.get_logger().info('Gripper goal accepted!')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._get_result_callback)

    def send_goal(self, position: float):
        """
        Send a gripper command goal to the gripper_controller.

        :param position: Target gripper position (meters).
        :return: Future object for the goal request.
        """
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )
        self._is_running = True
        future.add_done_callback(self._goal_response_callback)
        return future
