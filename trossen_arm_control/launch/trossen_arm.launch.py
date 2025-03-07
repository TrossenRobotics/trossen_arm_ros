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
    robot_name_launch_arg = LaunchConfiguration('robot_name').perform(context)

    ros2_control_controllers_config_parameter_file = ParameterFile(
        param_file=PathJoinSubstitution([
            FindPackageShare('trossen_arm_control'),
            'config',
            'trossen_arm_controllers.yaml',
        ]),
        allow_substs=True,
    )

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        # namespace=robot_name_launch_arg,
        parameters=[
            ros2_control_controllers_config_parameter_file,
        ],
        remappings=[
            ('~/robot_description', '/robot_description'),
            # ('~/robot_description', f'/{robot_name_launch_arg}/robot_description'),
        ],
        output={'both': 'screen'},
    )

    spawn_arm_controller_node = Node(
        name='arm_controller_spawner',
        package='controller_manager',
        executable='spawner',
        # namespace=robot_name_launch_arg,
        arguments=[
            # '--controller-manager', f'/{robot_name_launch_arg}/controller_manager',
            '--controller-manager', '/controller_manager',
            'arm_controller',
        ],
        output={'both': 'screen'},
    )

    spawn_gripper_controller_node = Node(
        name='gripper_controller_spawner',
        package='controller_manager',
        executable='spawner',
        # namespace=robot_name_launch_arg,
        arguments=[
            # '--controller-manager', f'/{robot_name_launch_arg}/controller_manager',
            '--controller-manager', '/controller_manager',
            'gripper_controller',
        ],
        output={'both': 'screen'},
    )

    spawn_joint_state_broadcaster_node = Node(
        name='joint_state_broadcaster_spawner',
        package='controller_manager',
        executable='spawner',
        # namespace=robot_name_launch_arg,
        arguments=[
            # '--controller-manager', f'/{robot_name_launch_arg}/controller_manager',
            '--controller-manager', '/controller_manager',
            'joint_state_broadcaster',
        ],
        remappings=[
            ('joint_states', '/joint_states'),
        ],
        output={'both': 'screen'},
    )

    description_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('trossen_arm_description'),
                'launch',
                'description.launch.py'
            ]),
        ),
        launch_arguments={
            'robot_model': robot_model_launch_arg,
            'robot_description': robot_description_launch_arg,
        }.items(),
    )

    return [
        controller_manager_node,
        description_launch_include,
        RegisterEventHandler(
            OnProcessStart(
                target_action=controller_manager_node,
                on_start=[
                    spawn_joint_state_broadcaster_node,
                    spawn_arm_controller_node,
                    # spawn_gripper_controller_node,
                ]
            )
        ),
    ]


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_model',
            default_value='wxai_base',
            description='model codename of the Trossen Arm such as `wxai`.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_name',
            default_value=LaunchConfiguration('robot_model'),
            description=(
                'name of the robot (typically equal to `robot_model`, but could be anything).'
            ),
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
                ]), '.urdf.xacro',
            ])
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
