# Copyright 2024 Trossen Robotics - All Rights Reserved
#
# Unauthorized copying of this file, via any medium is strictly prohibited
# Proprietary and confidential

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def launch_setup(context, *args, **kwargs):
    robot_name_launch_arg = LaunchConfiguration('robot_name')
    use_rviz_launch_arg = LaunchConfiguration('use_rviz')
    use_joint_pub_launch_arg = LaunchConfiguration('use_joint_pub')
    use_joint_pub_gui_launch_arg = LaunchConfiguration('use_joint_pub_gui')
    rvizconfig_launch_arg = LaunchConfiguration('rvizconfig')
    robot_description_launch_arg = LaunchConfiguration('robot_description')

    loginfo = LogInfo(msg=robot_description_launch_arg)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(robot_description_launch_arg, value_type=str),
        }],
        output={'both': 'screen'},
    )

    joint_state_publisher_node = Node(
        condition=IfCondition(use_joint_pub_launch_arg),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output={'both': 'screen'},
    )

    joint_state_publisher_gui_node = Node(
        condition=IfCondition(use_joint_pub_gui_launch_arg),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output={'both': 'screen'},
    )

    rviz2_node = Node(
        condition=IfCondition(use_rviz_launch_arg),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d', rvizconfig_launch_arg,
        ],
        output={'both': 'screen'},
    )

    return [
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz2_node,
        loginfo,
    ]


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_model',
            choices=('leader'),
            default_value='leader',
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
            'use_rviz',
            default_value='true',
            choices=('true', 'false'),
            description='launches RViz if set to `true`.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_joint_pub',
            default_value='false',
            choices=('true', 'false'),
            description='launches the joint_state_publisher node.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_joint_pub_gui',
            default_value='true',
            choices=('true', 'false'),
            description='launches the joint_state_publisher GUI.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rvizconfig',
            default_value=PathJoinSubstitution([
                FindPackageShare('trossen_arm_description'),
                'rviz',
                'trossen_arm_description.rviz',
            ]),
            description='file path to the config file RViz should load.',
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
