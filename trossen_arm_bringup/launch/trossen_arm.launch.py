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
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import (
    OnProcessStart,
)
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
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


def launch_setup(context, *args, **kwargs):

    robot_model_launch_arg = LaunchConfiguration('robot_model')
    robot_description_launch_arg = LaunchConfiguration('robot_description')

    ros2_control_controllers_config_parameter_file = ParameterFile(
        param_file=PathJoinSubstitution([
            FindPackageShare('trossen_arm_bringup'),
            'config',
            'controllers.yaml',
        ]),
        allow_substs=True,
    )

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            ros2_control_controllers_config_parameter_file,
        ],
        remappings=[
            ('~/robot_description', '/robot_description'),
        ],
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
                package='controller_manager',
                executable='spawner',
                arguments=[
                    controller_name,
                ],
                output={'both': 'screen'},
            )
        )

    description_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('trossen_arm_description'),
                'launch',
                'display.launch.py'
            ]),
        ),
        launch_arguments={
            'robot_model': robot_model_launch_arg,
            'robot_description': robot_description_launch_arg,
            'use_joint_pub_gui': 'false',
            'use_rviz': LaunchConfiguration('use_rviz'),
        }.items(),
    )

    return [
        controller_manager_node,
        description_launch_include,
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
            choices=('base', 'leader', 'follower'),
            description='End effector variant of the Trossen Arm.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'arm_side',
            default_value='none',
            choices=('none', 'left', 'right'),
            description=(
                'Side of the Trossen Arm. Note that only the wxai follower variant has a left '
                'and right side.'
            ),
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
            'use_world_frame',
            default_value='false',
            choices=('true', 'false'),
            description='Use world frame.'
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
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_description',
            default_value=Command([
                FindExecutable(name='xacro'), ' ',
                PathJoinSubstitution([
                    FindPackageShare('trossen_arm_description'),
                    'urdf',
                    LaunchConfiguration('robot_model'),
                    ]), '.urdf.xacro ',
                'use_world_frame:=', LaunchConfiguration('use_world_frame'), ' ',
                'arm_variant:=', LaunchConfiguration('arm_variant'), ' ',
                'arm_side:=', LaunchConfiguration('arm_side'), ' ',
                'ros2_control_hardware_type:=', LaunchConfiguration('ros2_control_hardware_type'), ' ',
                'ip_address:=', LaunchConfiguration('ip_address'),
            ])
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
