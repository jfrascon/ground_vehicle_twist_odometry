import os

import ros2_launch_helpers as rlh
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile

from launch import LaunchContext, LaunchDescription, LaunchDescriptionEntity  # noqa


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'namespace', default_value='robot', description='namespace where the node is launched'
            ),
            DeclareLaunchArgument(
                'params_file',
                default_value=os.path.join(
                    get_package_share_directory('ground_vehicle_twist_odometry'),
                    'config',
                    'example_ground_vehicle_twist_odometry.yaml',
                ),
                description='Base YAML with node parameters (str)',
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='False',
                choices=['True', 'true', 'False', 'false'],
                description='Use simulation clock if true',
            ),
            DeclareLaunchArgument(
                'odometry_frame', default_value='robot_odom', description='Odometry frame name for the robot'
            ),
            DeclareLaunchArgument(
                'base_frame', default_value='robot_base_link', description='Base frame name for the robot'
            ),
            DeclareLaunchArgument(
                'publish_tf',
                default_value='True',
                choices=['True', 'true', 'False', 'false'],
                description='Whether to publish the transformation T:<odometry_frame> -> <base_frame>',
            ),
            DeclareLaunchArgument(
                'expected_incoming_twist_msg_rate',
                default_value='50.0',
                description='Expected rate (in Hz) of incoming twist messages',
            ),
            # topic_remappings use syntax topic_remappings:="<from_1>:=<to_>,<from_2>:<to_2>,..." to remap topics.
            # For example, to remap "cmd_vel" to "twist_cmd" and "odom" to "odometry", set
            # ros2 launch ... topic_remappings:= "cmd_vel:twist_cmd,odom:odometry".
            DeclareLaunchArgument('topic_remappings', default_value='', description=rlh.TOPIC_REMAPPINGS_DESC),
            DeclareLaunchArgument(
                'node_options', default_value=rlh.default_node_options_str(), description=rlh.NODE_OPTIONS_DESC
            ),
            DeclareLaunchArgument(
                'logging_options', default_value=rlh.default_logging_options_str(), description=rlh.LOGGING_OPTIONS_DESC
            ),
            OpaqueFunction(function=launch_ground_vehicle_twist_odometry_node),
        ]
    )


def launch_ground_vehicle_twist_odometry_node(ctx: LaunchContext) -> list[LaunchDescriptionEntity]:
    # node_options include 'name', 'output', 'emulate_tty', 'respawn', 'respawn_delay',
    node_options = rlh.process_node_options(LaunchConfiguration('node_options').perform(ctx))
    node_name = str(node_options['name']) or 'ground_vehicle_twist_odometry'

    # The node parameters are ALWAYS loaded from `params_file`.
    # If the user does not pass `params_file`, the launch argument default points to the example
    # YAML installed by this package.
    # If the user passes a custom `params_file`, that YAML file is used instead.
    # Values written as `$(var <launch_argument_name>)` inside the YAML are resolved from the
    # current launch context, so a parameter file can delegate selected values to launch arguments.
    # Values written as literals in the YAML are used as-is.
    # An empty `params_file` is not part of this launch contract. `ParameterFile` expects a real
    # file path, so passing an empty value from a wrapper launch should fail instead of being
    # treated as "no parameter file".

    return [
        Node(
            package='ground_vehicle_twist_odometry',
            executable='ground_vehicle_twist_odometry_node',
            name=node_name,
            namespace=LaunchConfiguration('namespace'),
            parameters=[ParameterFile(LaunchConfiguration('params_file'), allow_substs=True)],
            remappings=rlh.process_topic_remappings(LaunchConfiguration('topic_remappings').perform(ctx)),
            ros_arguments=rlh.process_logging_options(LaunchConfiguration('logging_options').perform(ctx)),
            output=node_options['output'],
            emulate_tty=node_options['emulate_tty'],
            respawn=node_options['respawn'],
            respawn_delay=node_options['respawn_delay'],
        )
    ]
