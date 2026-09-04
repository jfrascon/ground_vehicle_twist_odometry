import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.utilities.type_utils import normalize_typed_substitution, perform_typed_substitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile, ParameterValue
import ros2_launch_helpers as rlh


def generate_launch_description() -> LaunchDescription:
    """Declare the inputs required to launch the twist odometry node."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'namespace',
                default_value='robot',
                description='Namespace where the node is launched.',
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
                'params_file_allow_substs',
                default_value='True',
                choices=['True', 'true', 'False', 'false'],
                description='Allow ROS launch substitutions in params_file.',
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='False',
                choices=['True', 'true', 'False', 'false'],
                description='Use ROS simulation time when true.',
            ),
            DeclareLaunchArgument(
                'node_args',
                default_value='{"output":"both","ros_arguments":["--log-level","info"]}',
                description=rlh.LAUNCH_ACTION_ARGUMENTS_DESC,
            ),
            rlh.RequireFile(path=LaunchConfiguration('params_file')),
            OpaqueFunction(function=launch_node),
        ]
    )


def launch_node(ctx: LaunchContext) -> list[LaunchDescriptionEntity]:
    params_allow_substs = perform_typed_substitution(
        ctx,
        normalize_typed_substitution(LaunchConfiguration('params_file_allow_substs'), bool),
        bool,
    )

    return [
        Node(
            package='ground_vehicle_twist_odometry',
            executable='ground_vehicle_twist_odometry_node',
            namespace=LaunchConfiguration('namespace'),
            parameters=[
                ParameterFile(LaunchConfiguration('params_file'), allow_substs=params_allow_substs),
                # The launch environment owns clock selection. Place use_sim_time after the YAML
                # file so this launch argument remains authoritative if the file also defines it.
                {
                    'use_sim_time': ParameterValue(
                        LaunchConfiguration('use_sim_time'), value_type=bool
                    )
                },
            ],
            **rlh.resolve_node_arguments(
                LaunchConfiguration('node_args').perform(ctx),
                default_arguments={'name': 'ground_vehicle_twist_odometry'},
                extra_rejected_arguments={'namespace'},
            ),
        )
    ]
