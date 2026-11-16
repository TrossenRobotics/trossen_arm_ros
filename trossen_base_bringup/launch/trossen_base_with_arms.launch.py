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
Launch a Trossen Base together with its two follower arms.
"""

from dataclasses import dataclass

from launch import LaunchDescription
from launch.action import Action
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare


@dataclass
class ArmMount:
    """Where an arm is mounted on the base lift."""

    side: str
    """Arm side, `left` or `right`. Used for the namespace and the joint/link prefix."""

    xyz: tuple[float, float, float]
    """Position of the arm base_link in lift_link in meters."""

    rpy: tuple[float, float, float]
    """Orientation of the arm base_link in lift_link in radians."""


# Mounting poses on the Rivet lift. Must match the *_mount_joint origins in rivet.urdf.xacro.
ARM_MOUNTS = [
    ArmMount(side='left', xyz=(0.0615, 0.275, 0.031), rpy=(0.0, 0.0, 0.0)),
    ArmMount(side='right', xyz=(0.0615, -0.275, 0.031), rpy=(0.0, 0.0, 0.0)),
]


def generate_launch_description_for_arm(mount: ArmMount) -> list[Action]:
    """
    Generate the launch actions for a single arm running in its own namespace.

    Args:
        mount (ArmMount): Side and mounting pose of the arm.

    Returns:
        list[Action]: Static transform, controller manager, robot_state_publisher, and the
            controller spawners (started once the controller manager is up).
    """
    namespace = f'follower_{mount.side}'
    prefix = f'{namespace}_'

    robot_description = Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([
            FindPackageShare('trossen_arm_description'),
            'urdf',
            LaunchConfiguration('arm_model'),
        ]), '.urdf.xacro ',
        'prefix:=', prefix, ' ',
        'variant:=', LaunchConfiguration('arm_variant'), ' ',
        'ros2_control_hardware_type:=',
        LaunchConfiguration(f'{mount.side}_arm_ros2_control_hardware_type'), ' ',
        'ip_address:=', LaunchConfiguration(f'{mount.side}_arm_ip_address'),
    ])

    mount_transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=f'{namespace}_mount_transform_publisher',
        arguments=[
            '--x', str(mount.xyz[0]),
            '--y', str(mount.xyz[1]),
            '--z', str(mount.xyz[2]),
            '--roll', str(mount.rpy[0]),
            '--pitch', str(mount.rpy[1]),
            '--yaw', str(mount.rpy[2]),
            '--frame-id', 'lift_link',
            '--child-frame-id', f'{prefix}base_link',
        ],
        output={'both': 'screen'},
    )

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace=namespace,
        parameters=[
            ParameterFile(
                param_file=PathJoinSubstitution([
                    FindPackageShare('trossen_base_bringup'),
                    'config',
                    'trossen_base_with_arms_controllers.yaml',
                ]),
                allow_substs=True,
            ),
        ],
        remappings=[
            # Relative, so it resolves to this arm's /<namespace>/robot_description
            ('~/robot_description', 'robot_description'),
        ],
        output={'both': 'screen'},
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        parameters=[{
            'robot_description': ParameterValue(robot_description, value_type=str),
        }],
        output={'both': 'screen'},
    )

    controller_spawner_nodes: list[Node] = []
    for controller_name in [
        'arm_controller',
        'gripper_controller',
        'joint_state_broadcaster',
    ]:
        controller_spawner_nodes.append(
            Node(
                name=f'{controller_name}_spawner',
                namespace=namespace,
                package='controller_manager',
                executable='spawner',
                arguments=[
                    controller_name,
                    '--controller-manager', f'/{namespace}/controller_manager',
                ],
                output={'both': 'screen'},
            )
        )

    return [
        mount_transform_node,
        controller_manager_node,
        robot_state_publisher_node,
        RegisterEventHandler(
            OnProcessStart(
                target_action=controller_manager_node,
                on_start=controller_spawner_nodes,
            )
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_model',
            default_value='rivet',
            choices=('rivet',),
            description='model name of the Trossen Base.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'ros2_control_hardware_type',
            default_value='real',
            choices=('real', 'mock_components'),
            description='Use real or mocked hardware interface for the base.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'arm_model',
            default_value='wxpro',
            choices=('wxpro'),
            description='model codename of both arms.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'arm_variant',
            default_value='follower',
            choices=('base', 'follower'),
            description='End effector variant of both arms.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'left_arm_ros2_control_hardware_type',
            default_value='real',
            choices=('real', 'mock_components'),
            description='Use real or mocked hardware interface for the left arm.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'right_arm_ros2_control_hardware_type',
            default_value='real',
            choices=('real', 'mock_components'),
            description='Use real or mocked hardware interface for the right arm.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'left_arm_ip_address',
            default_value='192.168.1.2',
            description='IP address of the left arm.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'right_arm_ip_address',
            default_value='192.168.1.3',
            description='IP address of the right arm.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            choices=('true', 'false'),
            description='Use rviz.'
        )
    )

    #  Wrapped in GroupAction so use_rviz:=false doesn't overwrite this launch file's own
    # launch configurations.
    base_launch_include = GroupAction([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('trossen_base_bringup'),
                'launch',
                'trossen_base.launch.py',
            ]),
        ),
        launch_arguments={
            'robot_model': LaunchConfiguration('robot_model'),
            'ros2_control_hardware_type': LaunchConfiguration('ros2_control_hardware_type'),
            'use_rviz': 'false',
            'robot_description': Command([
                FindExecutable(name='xacro'), ' ',
                PathJoinSubstitution([
                    FindPackageShare('trossen_arm_description'),
                    'urdf',
                    LaunchConfiguration('robot_model'),
                ]), '.urdf.xacro ',
                'include_arms:=false ',
                'ros2_control_hardware_type:=', LaunchConfiguration('ros2_control_hardware_type'),
            ]),
        }.items(),
    )])

    arm_actions: list[Action] = []
    for mount in ARM_MOUNTS:
        arm_actions.extend(generate_launch_description_for_arm(mount))

    rviz2_node = Node(
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d', PathJoinSubstitution([
                FindPackageShare('trossen_base_bringup'),
                'rviz',
                'trossen_base_with_arms.rviz',
            ]),
        ],
        output={'both': 'screen'},
    )

    return LaunchDescription(
        declared_arguments + [base_launch_include] + arm_actions + [rviz2_node]
    )
