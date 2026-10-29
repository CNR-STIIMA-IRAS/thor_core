# BSD 3-Clause License
#
# Copyright (c) 2026, Nicola Pedrocchi
# All rights reserved.
#
# See the project license for redistribution terms and warranty disclaimer.

"""UR10 with mock hardware and the THOR controller.

Arguments:
  rviz:=true|false     start RViz
  demo:=true|false     start the simulated person, the robot goals and the markers
  chained:=false|true  false: ThorTrajectoryController (config/ur10_controllers.yaml);
                       true: joint_trajectory_controller chained to ThorChainableController
                       (config/ur10_controllers_chained.yaml)

In both cases the trajectory action is /thor_controller/follow_joint_trajectory,
the obstacles go to /thor_controller/obstacles and the state of the MPC is on
/thor_controller/thor_state.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context):
    rviz = LaunchConfiguration('rviz')
    demo = LaunchConfiguration('demo')
    chained = LaunchConfiguration('chained').perform(context).lower() == 'true'
    share = FindPackageShare('thor_bringup')
    controllers = PathJoinSubstitution(
        [share, 'config', 'ur10_controllers_chained.yaml' if chained else 'ur10_controllers.yaml'])
    # Controllers to activate, the THOR one first; and the one with the robot spheres.
    thor_controllers = ['thor_chainable', 'thor_controller'] if chained else ['thor_controller']
    mpc_controller = thor_controllers[0]

    robot_description = {
        'robot_description': ParameterValue(
            Command([FindExecutable(name='xacro'), ' ',
                     PathJoinSubstitution([share, 'urdf', 'ur10.urdf.xacro'])]),
            value_type=str)
    }

    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[robot_description]),
        Node(package='controller_manager', executable='ros2_control_node',
             parameters=[controllers],
             remappings=[('~/robot_description', '/robot_description')],
             output='screen'),
        Node(package='controller_manager', executable='spawner',
             arguments=['joint_state_broadcaster', '--param-file', controllers]),
        Node(package='controller_manager', executable='spawner',
             arguments=thor_controllers + ['--param-file', controllers, '--activate-as-group']),

        Node(package='thor_bringup', executable='human_sim.py', condition=IfCondition(demo),
             parameters=[{'obstacles_topic': '/thor_controller/obstacles'}]),
        Node(package='thor_bringup', executable='robot_goal.py', condition=IfCondition(demo),
             parameters=[robot_description, {'action': '/thor_controller/follow_joint_trajectory'}]),
        Node(package='thor_bringup', executable='thor_markers.py', condition=IfCondition(demo),
             parameters=[{'controllers_file': controllers, 'controller': mpc_controller}]),
        Node(package='rviz2', executable='rviz2', condition=IfCondition(rviz),
             arguments=['-d', PathJoinSubstitution([share, 'rviz', 'thor_demo.rviz'])]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz'),
        DeclareLaunchArgument('demo', default_value='true',
                              description='Start the simulated person and the robot goals'),
        DeclareLaunchArgument('chained', default_value='false',
                              description='joint_trajectory_controller chained to ThorChainableController'),
        OpaqueFunction(function=launch_setup),
    ])
