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

from dataclasses import dataclass
from typing import Literal
from launch import LaunchDescription, Action
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import (
    OnProcessStart,
)
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import (
    ParameterFile,
)
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


@dataclass
class ArmLaunchConfig:
    """Configuration for a single instance of a Trossen Arm in a mutli-arm launch file."""
    robot_model: str
    """Robot model codename, such as `wxai`"""

    robot_name: str
    """Name of the robot, such as `trossen_arm_1`"""

    arm_variant: Literal['base', 'leader', 'follower']
    """End effector variant of the Trossen Arm, such as `base`, `leader`, or `follower`"""

    arm_side: Literal['none', 'left', 'right']
    """Side of the Trossen Arm, such as `none`, `left`, or `right`"""

    ip_address: str
    """IP address of the robot"""

    ros2_control_hardware_type: Literal['real', 'mock_components']
    """Type of ROS 2 control hardware interface, such as `real` or `mock_components`"""

    ros2_controllers_config_parameter_filename: str
    """Name of the ROS 2 controllers configuration file, such as `controllers.yaml`"""

    x: float
    """X coordinate of the robot in meters"""

    y: float
    """Y coordinate of the robot in meters"""

    z: float
    """Z coordinate of the robot in meters"""

    roll: float
    """Roll angle of the robot in radians"""

    pitch: float
    """Pitch angle of the robot in radians"""

    yaw: float
    """Yaw angle of the robot in radians"""

ROBOTS = [
    ArmLaunchConfig(
        robot_model='wxai',
        robot_name='trossen_arm_1',
        arm_variant='base',
        arm_side='none',
        ip_address='192.168.1.2',
        ros2_control_hardware_type='mock_components',
        ros2_controllers_config_parameter_filename='dual_arm_controllers.yaml',
        x=0.0,
        y=-0.5,
        z=0.0,
        roll=0.0,
        pitch=0.0,
        yaw=0.0,
    ),
    ArmLaunchConfig(
        robot_model='wxai',
        robot_name='trossen_arm_2',
        arm_variant='base',
        arm_side='none',
        ip_address='192.168.1.3',
        ros2_control_hardware_type='mock_components',
        ros2_controllers_config_parameter_filename='dual_arm_controllers.yaml',
        x=0.0,
        y=0.5,
        z=0.0,
        roll=0.0,
        pitch=0.0,
        yaw=0.0,
    ),
]


def generate_launch_description_for_robot(robot: ArmLaunchConfig) -> list[Action]:
    robot_model_launch_arg = robot.robot_model
    robot_description = Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([
            FindPackageShare('trossen_arm_description'),
            'urdf',
            robot_model_launch_arg,
        ]), '.urdf.xacro ',
        'prefix:=', robot.robot_name + '/', ' ',
        'use_world_frame:=false ',
        'arm_variant:=', robot.arm_variant, ' ',
        'arm_side:=', robot.arm_side, ' ',
        'ros2_control_hardware_type:=', robot.ros2_control_hardware_type, ' '
        'ip_address:=', robot.ip_address,
    ])

    static_transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=f'{robot.robot_name}_static_transform_publisher',
        arguments=[
            str(robot.x), str(robot.y), str(robot.z),
            str(robot.roll), str(robot.pitch), str(robot.yaw),
            'world', f'{robot.robot_name}/base_link',
        ],
        output={'both': 'screen'},
    )

    ros2_control_controllers_config_parameter_file = ParameterFile(
        param_file=PathJoinSubstitution([
            FindPackageShare('trossen_arm_bringup'),
            'config',
            robot.ros2_controllers_config_parameter_filename,
        ]),
        allow_substs=True,
    )

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace=robot.robot_name,
        parameters=[
            ros2_control_controllers_config_parameter_file,
            {'robot_description': ParameterValue(robot_description, value_type=str)},
        ],
        # remappings=[
        #     ('~/robot_description', '/robot_description'),
        # ],
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
                namespace=robot.robot_name,
                package='controller_manager',
                executable='spawner',
                arguments=[
                    controller_name,
                    '--controller-manager', f'/{robot.robot_name}/controller_manager',
                    # '--namespace', robot.robot_name,
                ],
                output={'both': 'screen'},
            )
        )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=robot.robot_name,
        parameters=[{
            'robot_description': ParameterValue(robot_description, value_type=str),
        }],
        output={'both': 'screen'},
    )

    return [
        static_transform_node,
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
    robot_actions: list[Action] = []

    for robot in ROBOTS:
        robot_actions.extend(generate_launch_description_for_robot(robot))


    use_rviz_launch_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        choices=('true', 'false'),
        description='Use rviz.'
    )
    rvizconfig_launch_arg = DeclareLaunchArgument(
        'rvizconfig',
        default_value=PathJoinSubstitution([
            FindPackageShare('trossen_arm_bringup'),
            'rviz',
            'dual_arm.rviz',
        ]),
        description='file path to the config file RViz should load.',
    )

    # joint_state_publisher_node = Node(
    #     condition=IfCondition(use_joint_pub_launch_arg),
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     output={'both': 'screen'},
    # )

    rviz2_node = Node(
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d', LaunchConfiguration('rvizconfig'),
        ],
        output={'both': 'screen'},
    )

    ld = LaunchDescription()
    ld.add_action(use_rviz_launch_arg)
    ld.add_action(rvizconfig_launch_arg)
    ld.add_action(rviz2_node)

    for robot_action in robot_actions:
        ld.add_action(robot_action)

    return ld

    # return [
    #     ld.
    #     *robot_nodes,
    #     rviz2_node,
    # ]


# def generate_launch_description() -> LaunchDescription:
#     declared_arguments = []
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             'robot_model',
#             default_value='wxai',
#             choices=('wxai'),
#             description='model codename of the Trossen Arm such as `wxai`.'
#         )
#     )
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             'arm_variant',
#             default_value='base',
#             choices=('base', 'leader', 'follower'),
#             description='End effector variant of the Trossen Arm.',
#         )
#     )
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             'arm_side',
#             default_value='none',
#             choices=('none', 'left', 'right'),
#             description=(
#                 'Side of the Trossen Arm. Note that only the wxai follower variant has a left '
#                 'and right side.'
#             ),
#         )
#     )
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             'ip_address',
#             default_value='192.168.1.2',
#             description='IP address of the robot.',
#         )
#     )
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             'ros2_control_hardware_type',
#             default_value='real',
#             choices=('real', 'mock_components'),
#             description='Use real or mocked hardware interface.'
#         )
#     )
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             'use_world_frame',
#             default_value='false',
#             choices=('true', 'false'),
#             description='Use world frame.'
#         )
#     )
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             'use_rviz',
#             default_value='true',
#             choices=('true', 'false'),
#             description='Use rviz.'
#         )
#     )
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             'rvizconfig',
#             default_value=PathJoinSubstitution([
#                 FindPackageShare('trossen_arm_description'),
#                 'rviz',
#                 'trossen_arm_description.rviz',
#             ]),
#             description='file path to the config file RViz should load.',
#         )
#     )
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             'robot_description',
#             default_value=Command([
#                 FindExecutable(name='xacro'), ' ',
#                 PathJoinSubstitution([
#                     FindPackageShare('trossen_arm_description'),
#                     'urdf',
#                     LaunchConfiguration('robot_model'),
#                     ]), '.urdf.xacro ',
#                 'use_world_frame:=', LaunchConfiguration('use_world_frame'), ' ',
#                 'arm_variant:=', LaunchConfiguration('arm_variant'), ' ',
#                 'arm_side:=', LaunchConfiguration('arm_side'), ' ',
#                 'ros2_control_hardware_type:=', LaunchConfiguration('ros2_control_hardware_type'), ' ',
#                 'ip_address:=', LaunchConfiguration('ip_address'),
#             ])
#         )
#     )

#     return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
