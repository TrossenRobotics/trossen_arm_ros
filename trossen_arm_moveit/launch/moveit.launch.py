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

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import (
    OnProcessStart,
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    rviz_config_file_launch_arg = LaunchConfiguration('rviz_config_file')
    use_moveit_rviz_launch_arg = LaunchConfiguration('use_moveit_rviz')

    moveit_configs = (
        MoveItConfigsBuilder(
            robot_name='wxai',
            package_name='trossen_arm_moveit',
        )
        .robot_description(
            file_path=PathJoinSubstitution([
                FindPackageShare('trossen_arm_description'),
                'urdf',
                'wxai.urdf.xacro',
            ]).perform(context),
            mappings={
                'arm_variant': LaunchConfiguration('arm_variant'),
                'ip_address': LaunchConfiguration('ip_address'),
                'ros2_control_hardware_type': LaunchConfiguration('ros2_control_hardware_type'),
            }
        )
        .robot_description_semantic(
            file_path='config/wxai.srdf.xacro',
            mappings={
                'variant': LaunchConfiguration('arm_variant'),
            },
        )
        .planning_scene_monitor(
            publish_geometry_updates=True,
            publish_state_updates=True,
            publish_transforms_updates=True,
            publish_planning_scene=True,
        )
        .trajectory_execution(
            file_path='config/moveit_controllers.yaml',
            moveit_manage_controllers=True,
        )
        .planning_pipelines(
            default_planning_pipeline='ompl',
            pipelines=[
                'ompl',
            ],
        )
        .robot_description_kinematics(
            file_path='config/kinematics.yaml',
        )
        .joint_limits(
            file_path='config/joint_limits.yaml',
        )
        .sensors_3d(
            file_path='config/sensors_3d.yaml',
        )
        .to_moveit_configs()
    )

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        parameters=[
            moveit_configs.to_dict(),
        ],
        output={'both': 'screen'},
    )

    moveit_rviz_node = Node(
        condition=IfCondition(use_moveit_rviz_launch_arg),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d', rviz_config_file_launch_arg,
        ],
        parameters=[
            moveit_configs.robot_description,
            moveit_configs.robot_description_semantic,
            moveit_configs.planning_pipelines,
            moveit_configs.robot_description_kinematics,
            moveit_configs.joint_limits,
        ],
        output={'both': 'log'},
    )

    ros2_controllers_filepath = PathJoinSubstitution([
        FindPackageShare('trossen_arm_moveit'),
        'config',
        'ros2_controllers.yaml',
    ])

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            ros2_controllers_filepath,
        ],
        remappings=[
            ('~/robot_description', '/robot_description'),
        ],
        output={'both': 'screen'},
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            moveit_configs.robot_description,
        ],
        output={'both': 'log'},
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
                package='controller_manager',
                executable='spawner',
                arguments=[
                    controller_name,
                ],
                output={'both': 'screen'},
            )
        )

    return [
        move_group_node,
        moveit_rviz_node,
        controller_manager_node,
        robot_state_publisher_node,
        RegisterEventHandler(
            OnProcessStart(
                target_action=controller_manager_node,
                on_start=controller_spawner_nodes,
            )
        ),
    ]


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_model',
            default_value='wxai',
            choices=('wxai'),
            description='model codename of the Trossen Arm such as `wxai`.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'arm_variant',
            default_value='base',
            choices=('base', 'follower'),
            description='End effector variant of the Trossen Arm.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'ip_address',
            default_value='192.168.1.2',
            description='IP address of the robot.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'ros2_control_hardware_type',
            default_value='real',
            choices=('real', 'mock_components'),
            description='Use real or mocked hardware interface.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_moveit_rviz',
            default_value='true',
            choices=('true', 'false'),
            description="Launches RViz with MoveIt's RViz configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('trossen_arm_moveit'),
                'config',
                'moveit.rviz',
            ]),
            description='Full path to the RVIZ config file to use.',
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
