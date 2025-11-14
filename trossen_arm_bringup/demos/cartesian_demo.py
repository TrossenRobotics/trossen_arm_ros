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
Cartesian Control Demo for Trossen Arm.

This demo demonstrates mode switching between joint position control and Cartesian pose control:
1. Moves arm to "staged" position using joint position control
2. Switches to Cartesian pose controller
3. Executes Cartesian movements
4. Switches back to joint position controller
5. Returns to staged position
6. Moves to home position (all zeros)
"""

import copy
import math
import time

import rclpy
from controller_manager_msgs.srv import SwitchController
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from controllers import ArmDemoNode  # noqa: I100


class CartesianDemoNode(Node):
    """Demo node for sending Cartesian pose commands to the robot."""

    def __init__(
        self,
        namespace: str = '',
        base_link_frame: str = 'base_link',
        ee_frame: str = 'ee_gripper_link',
    ):
        """
        Initialize the CartesianDemo node.

        :param namespace: Optional namespace for the Cartesian pose controller.
        :param base_link_frame: Name of the robot's base link frame.
        :param ee_frame: Name of the end-effector frame.
        """
        super().__init__('cartesian_demo')

        topic_name = 'cartesian_pose_controller/target_pose'
        if namespace:
            topic_name = f'{namespace}/{topic_name}'

        self.publisher = self.create_publisher(
            PoseStamped,
            topic_name,
            10
        )

        self.base_frame = base_link_frame
        self.ee_frame = ee_frame

        # Initialize TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self.get_logger().info(f'CartesianDemo initialized, publishing to: {topic_name}')

    def get_current_ee_pose(self) -> PoseStamped | None:
        """
        Get the current end-effector pose from TF.

        :return: Current pose as PoseStamped, or None if lookup fails.
        """
        try:
            # Look up transform from base_link to ee_gripper_link
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.ee_frame,
                Time(),  # Get latest available transform
                timeout=Duration(seconds=1.0)
            )

            # Convert transform to PoseStamped
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = self.base_frame

            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z

            pose.pose.orientation.x = transform.transform.rotation.x
            pose.pose.orientation.y = transform.transform.rotation.y
            pose.pose.orientation.z = transform.transform.rotation.z
            pose.pose.orientation.w = transform.transform.rotation.w

            self.get_logger().info(
                f'Current EE pose: pos=({pose.pose.position.x:.3f}, '
                f'{pose.pose.position.y:.3f}, {pose.pose.position.z:.3f}), '
                f'quat=({pose.pose.orientation.x:.3f}, {pose.pose.orientation.y:.3f}, '
                f'{pose.pose.orientation.z:.3f}, {pose.pose.orientation.w:.3f})'
            )

            return pose

        except Exception as e:
            self.get_logger().error(f'Failed to lookup transform: {e}')
            return None

    def send_pose(
        self,
        x: float,
        y: float,
        z: float,
        qx: float = 0.0,
        qy: float = 0.0,
        qz: float = 0.0,
        qw: float = 1.0):
        """
        Send a Cartesian pose command.

        :param x, y, z: Position in meters (base frame).
        :param qx, qy, qz, qw: Orientation as quaternion (default: identity).
        """
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame

        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z

        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw


    def stream_commands(self, poses: list[PoseStamped], delay_s: float = 0.002):
        """
        Stream a sequence of Cartesian pose commands.

        :param poses: List of PoseStamped messages to send.
        :param delay_s: Delay in seconds between sending each command.
        """
        for pose in poses:
            self.get_logger().info(
                f'Sent Cartesian pose: pos=('
                f'{pose.pose.position.x:.3f}, '
                f'{pose.pose.position.y:.3f}, '
                f'{pose.pose.position.z:.3f}), '
                f'quat=('
                f'{pose.pose.orientation.x:.3f}, '
                f'{pose.pose.orientation.y:.3f}, '
                f'{pose.pose.orientation.z:.3f}, '
                f'{pose.pose.orientation.w:.3f})'
            )
            self.publisher.publish(pose)
            time.sleep(delay_s)

    def interpolate(self, poses: list[PoseStamped], steps: int = 100) -> list[PoseStamped]:
        """
        Interpolate between a list of Cartesian poses.

        :param poses: List of PoseStamped messages to interpolate between.
        :param steps: Number of interpolation steps between each pair of poses.
        :return: List of interpolated PoseStamped messages.
        """
        def interp(start: float, end: float, t: float) -> float:
            return start + t * (end - start)

        interpolated_poses: list[PoseStamped] = []

        for i in range(len(poses) - 1):
            start = poses[i]
            end = poses[i + 1]

            for step in range(steps):
                t = step / float(steps)
                interp_pose = PoseStamped()
                interp_pose.header.stamp = self.get_clock().now().to_msg()
                interp_pose.header.frame_id = start.header.frame_id

                interp_pose.pose.position.x = interp(start.pose.position.x, end.pose.position.x, t)
                interp_pose.pose.position.y = interp(start.pose.position.y, end.pose.position.y, t)
                interp_pose.pose.position.z = interp(start.pose.position.z, end.pose.position.z, t)

                interp_pose.pose.orientation.x = interp(start.pose.orientation.x, end.pose.orientation.x, t)
                interp_pose.pose.orientation.y = interp(start.pose.orientation.y, end.pose.orientation.y, t)
                interp_pose.pose.orientation.z = interp(start.pose.orientation.z, end.pose.orientation.z, t)
                interp_pose.pose.orientation.w = interp(start.pose.orientation.w, end.pose.orientation.w, t)

                interpolated_poses.append(interp_pose)

        interpolated_poses.append(poses[-1])
        return interpolated_poses


def switch_controller(node: Node, activate_controllers: list, deactivate_controllers: list):
    """
    Switch between controllers using the controller_manager switch_controller service.

    :param node: ROS2 node to use for service client.
    :param activate_controllers: List of controller names to activate.
    :param deactivate_controllers: List of controller names to deactivate.
    :return: True if successful, False otherwise.
    """
    node.get_logger().info(
        f'Switching controllers: deactivating {deactivate_controllers}, '
        f'activating {activate_controllers}...'
    )

    # Create service client
    client = node.create_client(SwitchController, '/controller_manager/switch_controller')

    # Wait for service to be available
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error('Controller manager switch_controller service not available')
        return False

    # Create request
    request = SwitchController.Request()
    request.activate_controllers = activate_controllers
    request.deactivate_controllers = deactivate_controllers
    request.strictness = SwitchController.Request.BEST_EFFORT
    request.activate_asap = True
    request.timeout.sec = 5
    request.timeout.nanosec = 0

    # Call service
    try:
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)

        if future.done():
            response = future.result()
            if response is not None and response.ok:
                node.get_logger().info(
                    f'Successfully switched to {activate_controllers}'
                )
                return True
            else:
                node.get_logger().error('Controller switching failed')
                return False
        else:
            node.get_logger().error('Controller switching service call timed out')
            return False

    except Exception as e:
        node.get_logger().error(f'Error switching controllers: {e}')
        return False


def main(args=None):
    """Run the Cartesian control demo."""
    rclpy.init(args=args)

    arm = ArmDemoNode(action_name='arm_controller/follow_joint_trajectory')
    cartesian = CartesianDemoNode()

    staged_position = [0.0, math.pi / 3.0, math.pi / 6.0, math.pi / 5.0, 0.0, 0.0]  # upright position
    home_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    arm.get_logger().info('Moving to staged position using joint position control')
    future = arm.send_goal(staged_position, duration_s=3.0)
    rclpy.spin_until_future_complete(arm, future)
    while arm._is_running:
        rclpy.spin_once(arm)
    arm.get_logger().info('Reached staged position.')
    time.sleep(1.0)

    arm.get_logger().info('Switching to Cartesian pose controller')
    if not switch_controller(
        arm,
        activate_controllers=['cartesian_pose_controller'],
        deactivate_controllers=['arm_controller']
    ):
        arm.get_logger().error('Failed to switch to Cartesian pose controller. Aborting demo.')
        return
    time.sleep(1.0)

    cartesian.get_logger().info('Executing Cartesian movements')

    # Allow time for controller to initialize and TF to be available
    time.sleep(1.0)

    # Movement 1: Move forward
    cartesian.get_logger().info('Movement 1: Moving forward from current position...')
    initial_pose = cartesian.get_current_ee_pose()
    if initial_pose is None:
        cartesian.get_logger().error('Failed to get current EE pose. Aborting demo.')
        return
    goal_pose = copy.deepcopy(initial_pose)
    goal_pose.pose.position.x += 0.05
    poses = cartesian.interpolate([initial_pose, goal_pose], steps=100)
    cartesian.stream_commands(poses, delay_s=0.005)
    time.sleep(3.0)

    # Movement 2: Move right
    cartesian.get_logger().info('Movement 2: Moving right...')
    current_pose = cartesian.get_current_ee_pose()
    if current_pose is None:
        cartesian.get_logger().error('Failed to get current EE pose. Aborting demo.')
        return
    goal_pose = copy.deepcopy(current_pose)
    goal_pose.pose.position.y -= 0.05
    poses = cartesian.interpolate([current_pose, goal_pose])
    cartesian.stream_commands(poses)
    time.sleep(3.0)

    # Movement 3: Move left
    cartesian.get_logger().info('Movement 3: Moving left...')
    current_pose = cartesian.get_current_ee_pose()
    if current_pose is None:
        cartesian.get_logger().error('Failed to get current EE pose. Aborting demo.')
        return
    goal_pose = copy.deepcopy(current_pose)
    goal_pose.pose.position.y += 0.1
    poses = cartesian.interpolate([current_pose, goal_pose])
    cartesian.stream_commands(poses)
    time.sleep(3.0)

    # Movement 4: Move back to center
    cartesian.get_logger().info('Movement 4: Moving to center...')
    current_pose = cartesian.get_current_ee_pose()
    if current_pose is None:
        cartesian.get_logger().error('Failed to get current EE pose. Aborting demo.')
        return
    goal_pose = copy.deepcopy(initial_pose)
    poses = cartesian.interpolate([current_pose, goal_pose])
    cartesian.stream_commands(poses)
    time.sleep(3.0)

    # Movement 5: Add a rotation (20 degrees around Z axis)
    cartesian.get_logger().info('Movement 5: Rotating 20 degrees around Z axis...')
    current_pose = cartesian.get_current_ee_pose()
    if current_pose is None:
        cartesian.get_logger().error('Failed to get current EE pose. Aborting demo.')
        return
    goal_pose = copy.deepcopy(current_pose)
    # Quaternion for 20° rotation around Z: [0, 0, sin(10°), cos(10°)]
    goal_pose.pose.orientation.x = 0.0
    goal_pose.pose.orientation.y = 0.0
    goal_pose.pose.orientation.z = math.sin(math.radians(10.0))
    goal_pose.pose.orientation.w = math.cos(math.radians(10.0))
    poses = cartesian.interpolate([current_pose, goal_pose])
    cartesian.stream_commands(poses)
    time.sleep(3.0)

    # Movement 6: Rotate back to identity
    cartesian.get_logger().info('Movement 6: Rotating back to identity...')
    current_pose = cartesian.get_current_ee_pose()
    if current_pose is None:
        cartesian.get_logger().error('Failed to get current EE pose. Aborting demo.')
        return
    goal_pose = copy.deepcopy(current_pose)
    goal_pose.pose.orientation.x = 0.0
    goal_pose.pose.orientation.y = 0.0
    goal_pose.pose.orientation.z = 0.0
    goal_pose.pose.orientation.w = 1.0
    poses = cartesian.interpolate([current_pose, goal_pose])
    cartesian.stream_commands(poses)
    time.sleep(3.0)

    cartesian.get_logger().info('Cartesian movements complete.')

    arm.get_logger().info('Switching back to joint position controller')
    if not switch_controller(
        arm,
        activate_controllers=['arm_controller'],
        deactivate_controllers=['cartesian_pose_controller']
    ):
        arm.get_logger().error('Failed to switch back to joint controller. Aborting demo.')
        return
    time.sleep(1.0)

    arm.get_logger().info('Returning to staged position (joint control)')
    future = arm.send_goal(staged_position, duration_s=3.0)
    rclpy.spin_until_future_complete(arm, future)
    while arm._is_running:
        rclpy.spin_once(arm)
    arm.get_logger().info('Returned to staged position.')
    time.sleep(1.0)

    arm.get_logger().info('Moving to home position (all zeros)')
    future = arm.send_goal(home_position, duration_s=3.0)
    rclpy.spin_until_future_complete(arm, future)
    while arm._is_running:
        rclpy.spin_once(arm)
    arm.get_logger().info('Reached home position.')

    cartesian.destroy_node()
    arm.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
