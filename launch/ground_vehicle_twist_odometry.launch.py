import os
from pathlib import Path
from typing import Any, List

import ros2_launch_helpers as rlh
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue

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
                description='YAML file with node parameters',
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='False',
                choices=['True', 'true', 'False', 'false'],
                description='Use simulation clock if true',
            ),
            DeclareLaunchArgument('odometry_frame', default_value='', description='Odometry frame for the robot'),
            DeclareLaunchArgument('base_frame', default_value='', description='Base frame name for the robot'),
            DeclareLaunchArgument(
                'publish_tf',
                default_value='',
                choices=['True', 'true', 'False', 'false', ''],
                description='Whether to publish the transformation T:<odometry_frame> -> <base_frame>',
            ),
            DeclareLaunchArgument(
                'expected_incoming_twist_msg_rate',
                default_value='',
                description='Expected rate of incoming twist messages',
            ),
            DeclareLaunchArgument('node_remappings', default_value='', description=rlh.REMAPPINGS_DESC),
            DeclareLaunchArgument(
                'node_options', default_value=rlh.default_node_options_str(), description=rlh.NODE_OPTIONS_DESC
            ),
            DeclareLaunchArgument(
                'node_logging_options',
                default_value=rlh.default_logging_options_str(),
                description=rlh.LOGGING_OPTIONS_DESC,
            ),
            OpaqueFunction(function=launch_ground_vehicle_twist_odometry_node),
        ]
    )


def launch_ground_vehicle_twist_odometry_node(ctx: LaunchContext) -> list[LaunchDescriptionEntity]:
    # If the params_file exists, load it as a ParameterFile.
    # If any parameter is also provided to this launch file, it takes precedence over the
    # params_file.
    # This allows to override specific parameters in the params_file without having to create a new
    # params file.
    parameters: List[Any] = []

    params_file = LaunchConfiguration('params_file').perform(ctx)
    odometry_frame = LaunchConfiguration('odometry_frame').perform(ctx)
    base_frame = LaunchConfiguration('base_frame').perform(ctx)
    publish_tf = LaunchConfiguration('publish_tf').perform(ctx)
    expected_incoming_twist_msg_rate = LaunchConfiguration('expected_incoming_twist_msg_rate').perform(ctx)

    if params_file:
        if not Path(params_file).is_file():
            raise FileNotFoundError(f"Params file '{params_file}' does not exist. ")

        parameters.append(ParameterFile(params_file, allow_substs=True))

    if odometry_frame:
        parameters.append({'odometry_frame': odometry_frame})

    if base_frame:
        parameters.append({'base_frame': base_frame})

    if publish_tf:
        parameters.append({'publish_tf': publish_tf.lower() == 'true'})

    if expected_incoming_twist_msg_rate:
        try:
            parameters.append({'expected_incoming_twist_msg_rate': float(expected_incoming_twist_msg_rate)})
        except ValueError as exc:
            raise ValueError(
                'Invalid value for expected_incoming_twist_msg_rate: '
                f"'{expected_incoming_twist_msg_rate}'. Must be a float."
            ) from exc

    parameters.append({'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)})

    # node_options include 'name', 'output', 'emulate_tty', 'respawn', 'respawn_delay',
    node_options = rlh.process_node_options(LaunchConfiguration('node_options').perform(ctx))
    node_name = str(node_options['name']) or 'ground_vehicle_twist_odometry'

    return [
        Node(
            package='ground_vehicle_twist_odometry',
            executable='ground_vehicle_twist_odometry_node',
            namespace=LaunchConfiguration('namespace'),
            name=node_name,
            parameters=parameters,
            remappings=rlh.process_remappings(LaunchConfiguration('node_remappings').perform(ctx)),
            ros_arguments=rlh.process_node_logging_options(LaunchConfiguration('node_logging_options').perform(ctx)),
            output=node_options['output'],
            emulate_tty=node_options['emulate_tty'],
            respawn=node_options['respawn'],
            respawn_delay=node_options['respawn_delay'],
        )
    ]
