import json
import os
import re

import yaml
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def load_json_config(json_file):
    try:
        with open(json_file, 'r') as file:
            data = json.load(file)
        return data
    except FileNotFoundError:
        print(f"Error: The file '{json_file}' was not found.")
    except json.JSONDecodeError as e:
        print(f'Error: An error occurred while parsing the JSON file: {e}')
    except Exception as e:
        print(f'An unexpected error occurred: {e}')


def load_yaml_config(yaml_file):
    try:
        with open(yaml_file, 'r') as file:
            data = yaml.safe_load(file)
        return data
    except FileNotFoundError:
        print(f"Error: The file '{yaml_file}' was not found.")
    except yaml.YAMLError as e:
        print(f'Error: An error occurred while parsing the YAML file: {e}')
    except Exception as e:
        print(f'An unexpected error occurred: {e}')


def get_node_config_and_ns(yaml_config, namespace=''):
    # Example:
    # namespace_1:
    #   namespace_2:
    #     ...
    #       namespace_n:
    #         node_name:
    #           ros__parameters:
    #             param_1:
    #             param_2:
    #             ...
    #             param_n

    if not isinstance(yaml_config, dict):
        return None

    keys = list(yaml_config.keys())

    if len(keys) != 1:  # only one key allowed until we get the params under ros__parameters field
        return None

    inner_yaml_config = yaml_config[keys[0]]

    if 'ros__parameters' in inner_yaml_config:
        return (yaml_config, namespace)
    else:
        if namespace == '':
            namespace = keys[0]
        else:
            namespace = namespace + '/' + keys[0]
        return get_node_config_and_ns(inner_yaml_config, namespace)


def get_remappings_for_actions(actions_yaml):
    """
    actions_yaml is: [{from: "from_p", to: "to_p"}, {from: "from_q", to: "to_q"} ... ].
    The function will generate from actions_yaml the list:
    [(from_p/_action/feedback, to_p/_action/feedback),
     (from_p/_action/status, to_p/_action/status),
     (from_p/_action/calcel_goal, to_p/_action/calcel_goal),
     (from_p/_action/get_result, to_p/_action/get_result),
     (from_p/_action/send_goal, to_p/_action/send_goal),
     (from_q/_action/feedback, to_q/_action/feedback),
     (from_q/_action/status, to_q/_action/status),
     (from_q/_action/calcel_goal, to_q/_action/calcel_goal),
     (from_q/_action/get_result, to_q/_action/get_result),
     (from_q/_action/send_goal, to_q/_action/send_goal),
     ...
     ]
    """

    remapping_list = []

    if actions_yaml is None or not isinstance(actions_yaml, list):
        return remapping_list

    # Reference: https://github.com/ros2/ros2/issues/1312#issuecomment-1705521109
    # itm is a dictionary, each {from: "from_i", to: "to_i"}
    for item in actions_yaml:
        from_to_list = item.split(':')
        remapping_list.append((from_to_list[0] + '/_action/feedback', from_to_list[1] + '/_action/feedback'))
        remapping_list.append((from_to_list[0] + '/_action/status', from_to_list[1] + '/_action/status'))
        remapping_list.append((from_to_list[0] + '/_action/cancel_goal', from_to_list[1] + '/_action/cancel_goal'))
        remapping_list.append((from_to_list[0] + '/_action/get_result', from_to_list[1] + '/_action/get_result'))
        remapping_list.append((from_to_list[0] + '/_action/send_goal', from_to_list[1] + '/_action/send_goal'))

    return remapping_list


def get_remappings(ros_remappings):
    # This is what we get from yaml configuration file:
    # ros_remappings:
    #   topics:
    #     # subscribers
    #     - "from_1:to_1"
    #     - "from_2:to_2"
    #     ...
    #     # publishers
    #     - "from_i:to_i"
    #     ...
    #   services
    #     # service servers
    #     - "from_l:to_l"
    #     ...
    #     # service clients
    #     - "from_m:to_m"
    #     ...
    #   actions:
    #     # action servers
    #     - "from_p:to_p"
    #     ...
    #     # action clients
    #     - "from_q:to_q"
    #     ...

    remapping_list = []

    if ros_remappings is None:
        return remapping_list

    if 'topics' in ros_remappings and ros_remappings['topics'] is not None:
        # topics is: ["from_1:to_1", "from_2:to_2" ... ].
        # The Node constructor requires [("from_1": "to_1"), ("from_2": "to_2") ... ].
        topics_yaml = ros_remappings['topics']
        remappings_topics = [tuple(item.split(':')) for item in topics_yaml]
        remapping_list.extend(remappings_topics)

    if 'services' in ros_remappings and ros_remappings['services'] is not None:
        # services is: ["from_1:to_1", "from_2:to_2" ... ].
        # The Node constructor requires a list of tuples.
        services_yaml = ros_remappings['services']
        remappings_services = [tuple(item.split(':')) for item in services_yaml]
        remapping_list.extend(remappings_services)

    if 'actions' in ros_remappings and ros_remappings['actions'] is not None:
        # actions is: ["from_p:to_p", "from_q:to_q" ... ].
        # The Node constructor requires a list of tuples.
        actions_yaml = ros_remappings['actions']
        remappings_actions = get_remappings_for_actions(actions_yaml)
        remapping_list.extend(remappings_actions)

    return remapping_list


def get_filenames_from_filename_patterns(yaml_parameters):
    """
    Recursive function to expand filename patterns to get the actual filenames.
    Each filename is associated to its fully qualified key.
    """

    pattern1 = r'package://([^/]+)/(.+)'
    pattern2 = r'file://(/.+)'
    param_list = []

    if yaml_parameters is None or not isinstance(yaml_parameters, dict):
        return param_list

    # Get absolute path from pattern
    # Reference: https://github.com/ros2/ros2/issues/1312#issuecomment-1705521109
    # Given the pattern 'package://<package>/a/relative/path/file.txt
    # we obtain: /absolute/path/to/package/a/relative/path/file.txt
    # Or given the pattern 'file:///absolute/path/to/a/file.txt
    # we obtain /absolute/path/to/a/file.txt

    for param_id, value in yaml_parameters.items():
        if isinstance(value, dict):
            get_filenames_from_filename_patterns(value)
            continue

        # A file path must be a string
        if not isinstance(value, str):
            continue

        match1 = re.search(pattern1, value)

        if match1 is not None:
            package = get_package_share_directory(match1.group(1))
            relative_path = match1.group(2)
            value = os.path.join(package, relative_path)
            yaml_parameters[param_id] = value
            continue

        match2 = re.search(pattern2, value)

        if match2 is not None:
            value = match2.group(1)
            yaml_parameters[param_id] = value


def configure(context, *args, **kwargs):
    lc_config_file = LaunchConfiguration('config_file')
    config_file = lc_config_file.perform(context)
    yaml_config = load_yaml_config(config_file)

    if yaml_config is None:
        raise ValueError('The yaml config is empty.')

    (node_config, namespace) = get_node_config_and_ns(yaml_config)

    if node_config is None:
        raise ValueError(f"The yaml configuration in the file '{config_file}' is invalid")

    node_name = next(iter(node_config))
    ros_parameters = node_config[node_name]['ros__parameters']

    # print(namespace)
    # print(node_name)
    # print(ros_parameters)

    if 'ros_execution' not in ros_parameters or ros_parameters['ros_execution'] is None:
        raise ValueError('The field ros_execution is required in the yaml configuration')

    ros_execution = ros_parameters['ros_execution']

    if 'node_executable' not in ros_execution or ros_execution['node_executable'] is None:
        raise ValueError('The field node_executable is required in the yaml configuration')

    output = 'screen'

    if 'output' in ros_execution and ros_execution['output'] is not None:
        output = ros_execution['output']

    emulate_tty = False

    if 'emulate_tty' in ros_execution and ros_execution['emulate_tty'] is not None:
        emulate_tty = ros_execution['emulate_tty']

    respawn = False

    if 'respawn' in ros_execution and ros_execution['respawn'] is not None:
        respawn = ros_execution['respawn']

    respawn_delay = 0

    if 'respawn_delay' in ros_execution and ros_execution['respawn_delay'] is not None:
        respawn_delay = ros_execution['respawn_delay']

    remapping_list = []

    if 'ros_remappings' in ros_parameters and ros_parameters['ros_remappings'] is not None:
        remapping_list = get_remappings(ros_parameters['ros_remappings'])
        print(remapping_list)

    # Parameters are passed as a list, with each element either a yaml file that contains
    # parameter rules or a dictionary that specifies parameter rules.

    get_filenames_from_filename_patterns(ros_parameters)

    # print(ros_parameters)

    node = Node(
        package='eut_ground_vehicle_twist_odometry',
        executable=ros_execution['node_executable'],
        namespace=namespace,
        name=node_name,
        parameters=[ros_parameters],
        remappings=remapping_list,
        output=output,
        emulate_tty=emulate_tty,
        respawn=respawn,
        respawn_delay=respawn_delay,
    )

    return [node]


def generate_launch_description():
    # If a default value es added to the function DeclareLaunchArgument, then it must be assign
    # to a left-hand-side variable.
    DeclareLaunchArgument('config_file', description='Configuration file')

    ld = LaunchDescription()
    ld.add_action(OpaqueFunction(function=configure))

    return ld
