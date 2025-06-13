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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    LogInfo,
)
from launch.conditions import IfCondition, LaunchConfigurationEquals, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import yaml
# from moveit_configs_utils import MoveItConfigsBuilder


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def launch_setup(context, *args, **kwargs):

    robot_model_launch_arg = LaunchConfiguration('robot_model')
    # robot_name_launch_arg = LaunchConfiguration('robot_name')
    # base_link_frame_launch_arg = LaunchConfiguration('base_link_frame')
    # show_ar_tag_launch_arg = LaunchConfiguration('show_ar_tag')
    # use_world_frame_launch_arg = LaunchConfiguration('use_world_frame')
    # external_urdf_loc_launch_arg = LaunchConfiguration('external_urdf_loc')
    # mode_configs_launch_arg = LaunchConfiguration('mode_configs')
    # use_moveit_rviz_launch_arg = LaunchConfiguration('use_moveit_rviz')
    # rviz_frame_launch_arg = LaunchConfiguration('rviz_frame')
    rviz_config_file_launch_arg = LaunchConfiguration('rviz_config_file')
    # world_filepath_launch_arg = LaunchConfiguration('world_filepath')
    # ros2_control_hardware_type_launch_arg = LaunchConfiguration('ros2_control_hardware_type')
    robot_description_launch_arg = LaunchConfiguration('robot_description')
    robot_description_parameter = ParameterValue(robot_description_launch_arg, value_type=str)

    robot_description = {'robot_description': robot_description_parameter}

    # config_path = PathJoinSubstitution([
    #     FindPackageShare('trossen_arm_moveit'),
    #     'config',
    # ])

    robot_description_semantic = {
        'robot_description_semantic':
            ParameterValue(
                Command([
                    FindExecutable(name='xacro'), ' ', PathJoinSubstitution([
                        FindPackageShare('trossen_arm_moveit'),
                        'config',
                        'wxai.srdf',
                    ]),
                ]),
                value_type=str
            ),
    }

    kinematics_config = load_yaml('trossen_arm_moveit', 'config/kinematics.yaml')

    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters':
                'default_planner_request_adapters/AddTimeOptimalParameterization '
                'default_planner_request_adapters/FixWorkspaceBounds '
                'default_planner_request_adapters/FixStartStateBounds '
                'default_planner_request_adapters/FixStartStateCollision '
                'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
        }
    }

    ompl_planning_pipeline_yaml_file = load_yaml(
        'trossen_arm_moveit', 'config/ompl_planning.yaml'
    )
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_pipeline_yaml_file)

    controllers_config = load_yaml(
        'trossen_arm_moveit', 'config/moveit_controllers.yaml'
    )

    config_joint_limits = load_yaml(
        'trossen_arm_moveit', 'config/wxai_joint_limits.yaml'
    )

    joint_limits = {
        'robot_description_planning': config_joint_limits,
    }

    moveit_controllers = {
        'moveit_simple_controller_manager': controllers_config,
        'moveit_controller_manager':
            'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }

    trajectory_execution_parameters = {
        'moveit_manage_controllers': True,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    # moveit_config = (
    #     MoveItConfigsBuilder('trossen_arm_moveit')
    #     .robot_description(robot_description)
    #     .robot_description_semantic(robot_description_semantic)
    #     .planning_scene_monitor(
    #         publish_robot_description=True,
    #         publish_robot_description_semantic=True,
    #     )
    #     # .trajectory_execution(file_path=)
    # )

    # static_tf_node = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_tf_publisher',
    #     arguments=[
    #         '0.0', '0.0', '0.0',  # x, y, z
    #         '0.0', '0.0', '0.0',  # roll, pitch, yaw
    #         'base_link',  # parent frame
    #         'world',  # child frame
    #     ],
    #     output={'both': 'log'},
    # )

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        parameters=[
            # {
            #     'planning_scene_monitor_options': {
            #         'robot_description':
            #             'robot_description',
            #     },
            # },
            robot_description,
            robot_description_semantic,
            kinematics_config,
            ompl_planning_pipeline_config,
            trajectory_execution_parameters,
            moveit_controllers,
            planning_scene_monitor_parameters,
            joint_limits,
        ],
        output={'both': 'screen'},
    )

    moveit_rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d', rviz_config_file_launch_arg,
        ],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_config,
        ],
        output={'both': 'log'},
    )

    controllers_filepath = PathJoinSubstitution([
        FindPackageShare('trossen_arm_moveit'),
        'config',
        'wxai_controllers.yaml',
    ])

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            controllers_filepath,
        ],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output={'both': 'screen'},
    )

    spawn_arm_controller_node = Node(
        name='arm_controller_spawner',
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
        ],
        output={'both': 'screen'},
    )

    spawn_gripper_controller_node = Node(
        name='gripper_controller_spawner',
        package='controller_manager',
        executable='spawner',
        arguments=[
            'gripper_controller',
        ],
        output={'both': 'screen'},
    )

    spawn_joint_state_broadcaster_node = Node(
        name='joint_state_broadcaster_spawner',
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
        ],
        output={'both': 'screen'},
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_parameter,
        }],
        output={'both': 'log'},
    )

    return [
        move_group_node,
        moveit_rviz_node,
        controller_manager_node,
        spawn_arm_controller_node,
        spawn_gripper_controller_node,
        spawn_joint_state_broadcaster_node,
        robot_state_publisher_node,
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
            'ros2_control_hardware_type',
            default_value='real',
            choices=('real', 'mock_components'),
            description='Use real or mocked hardware interface.'
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
            'arm_variant:=', LaunchConfiguration('arm_variant'), ' ',
            'arm_side:=', LaunchConfiguration('arm_side'), ' ',
            'ros2_control_hardware_type:=', LaunchConfiguration('ros2_control_hardware_type'), ' ',
            ])
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
