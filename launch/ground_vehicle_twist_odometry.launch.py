import os
from typing import Any, Dict

import ros2_launch_helpers as rlh
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue

from launch import LaunchContext, LaunchDescription, LaunchDescriptionEntity  # noqa
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
from launch.utilities.type_utils import normalize_typed_substitution, perform_typed_substitution


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='False',
                choices=['True', 'true', 'False', 'false'],
                description='Use simulation clock if true',
            ),
            DeclareLaunchArgument('namespace', default_value='', description='namespace (Optional, default: "")'),
            # 'robot_name' is used to set 'robot_namespace' and 'robot_prefix'.
            DeclareLaunchArgument('robot_name', default_value='robot', description='The unique name for the robot'),
            # Parameters can be passed through the parameter file or through the launch file.
            # The parameters set in the launch file have precedence over those set in the parameter file.
            DeclareLaunchArgument(
                'params_file',
                default_value=os.path.join(
                    get_package_share_directory('ground_vehicle_twist_odometry'),
                    'config',
                    'example_ground_vehicle_twist_odometry.yaml',
                ),
                description='Base YAML with ros__parameters',
            ),
            DeclareLaunchArgument(
                'odometry_frame',
                default_value='',
                description='Odometry frame name for the robot. Do not prepend robot prefix here '
                '(Optional, default: "")',
            ),
            DeclareLaunchArgument(
                'base_frame',
                default_value='',
                description='Base frame name for the robot. Do not prepend robot prefix here (Optional, default: "")',
            ),
            DeclareLaunchArgument(
                'publish_tf',
                default_value='',
                choices=['True', 'true', 'False', 'false', ''],
                description='Whether to publish the transformation T:<odometry_frame> -> <base_frame> '
                '(Optional, default: "")',
            ),
            DeclareLaunchArgument(
                'expected_incoming_twist_msg_rate',
                default_value='',
                description='Expected rate (in Hz) of incoming twist messages (Optional, default: "")',
            ),
            # This is not a parameter for the node. Its an argument used to select the executable to use.
            # This parameters is not set in the parameter file.
            DeclareLaunchArgument(
                'use_timestamped_twist',
                default_value='False',
                choices=['True', 'true', 'False', 'false'],
                description='Use the executable that uses timestamped twist messages (Default: False)',
            ),
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
    # 'use_timestamped_twist' is used to select the executable to use.
    use_timestamped_twist = perform_typed_substitution(
        ctx, normalize_typed_substitution(LaunchConfiguration('use_timestamped_twist'), bool), bool
    )

    executable = (
        'ground_vehicle_twist_odometry_node_w_timestamp'
        if use_timestamped_twist
        else 'ground_vehicle_twist_odometry_node'
    )

    parameters = []

    params_file = LaunchConfiguration('params_file').perform(ctx).strip()

    # Add parameter file only if it's not empty.
    if params_file:
        parameters.append(ParameterFile(params_file, allow_substs=True))

    # If parameters 'odometry_frame', 'base_frame', 'publish_tf', 'expected_incoming_twist_msg_rate', are set through
    # the launch file, they override those set in the parameter file.
    parameters_dict: Dict[str, Any] = {
        'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    }

    robot_name = LaunchConfiguration('robot_name').perform(ctx)
    robot_prefix = rlh.create_robot_prefix(robot_name)
    robot_ns = rlh.create_robot_namespace(LaunchConfiguration('namespace').perform(ctx), robot_name)

    odometry_frame = LaunchConfiguration('odometry_frame').perform(ctx).strip()

    if odometry_frame:
        parameters_dict['odometry_frame'] = robot_prefix + odometry_frame

    base_frame = LaunchConfiguration('base_frame').perform(ctx).strip()

    if base_frame:
        parameters_dict['base_frame'] = robot_prefix + base_frame

    publish_tf = LaunchConfiguration('publish_tf').perform(ctx).strip()

    if publish_tf:
        parameters_dict['publish_tf'] = bool(publish_tf)

    expected_incoming_twist_msg_rate = LaunchConfiguration('expected_incoming_twist_msg_rate').perform(ctx).strip()

    if expected_incoming_twist_msg_rate:
        parameters_dict['expected_incoming_twist_msg_rate'] = float(expected_incoming_twist_msg_rate)

    parameters.append(parameters_dict)

    # node_options include 'name', 'output', 'emulate_tty', 'respawn', 'respawn_delay',
    node_options = rlh.process_node_options(LaunchConfiguration('node_options').perform(ctx))
    node_name = str(node_options['name']) or 'ground_vehicle_twist_odometry'

    return [
        # Set the LaunchConfiguration 'robot_prefix' before launching the node, since the ParameterFile action
        # needs the 'robot_prefix' to be set to substitute its value in the content of the parameter file.
        SetLaunchConfiguration('robot_prefix', robot_prefix),
        Node(
            package='ground_vehicle_twist_odometry',
            executable=executable,
            name=node_name,
            # Insert the node into the robot_namespace.
            namespace=robot_ns,
            parameters=parameters,
            remappings=rlh.process_topic_remappings(LaunchConfiguration('topic_remappings').perform(ctx)),
            ros_arguments=rlh.process_logging_options(LaunchConfiguration('logging_options').perform(ctx)),
            output=node_options['output'],
            emulate_tty=node_options['emulate_tty'],
            respawn=node_options['respawn'],
            respawn_delay=node_options['respawn_delay'],
        ),
    ]
