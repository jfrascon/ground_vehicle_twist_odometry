# [ground_vehicle_twist_odometry]()

`ground_vehicle_twist_odometry` is a ROS 2 package that computes planar odometry (`x`, `y`, `yaw`) by integrating incoming twist commands.

The package provides two executables:

- `ground_vehicle_twist_odometry_node`: subscribes to `geometry_msgs/msg/Twist`.
- `ground_vehicle_twist_odometry_node_w_timestamp`: subscribes to `geometry_msgs/msg/TwistStamped` and uses message timestamps for integration.

## Purpose

This node is useful when a mobile base already exposes an estimate of body twist and you want a lightweight odometry source in ROS 2.

It publishes:

- `nav_msgs/msg/Odometry` on `odom`
- optional TF transform `<odometry_frame> -> <base_frame>`

and provides a reset service:

- `reset_odom` (`std_srvs/srv/Empty`)

## How it works

The node initializes integration when the first twist message is received.

- In non-timestamped mode (`Twist`), time is taken from `node->now()`.
- In timestamped mode (`TwistStamped`), time is taken from `msg.header.stamp`.

For each message, the node computes `dt`, integrates planar motion, publishes odometry, optionally publishes TF, and stores the current timestamp as the previous timestamp.

If a message arrives with `dt < 0`, the node logs a warning and skips that integration step.

The parameter `expected_incoming_twist_msg_rate` is used as a runtime sanity check to warn when incoming twist frequency is lower than expected.

## Main ROS interfaces

## Subscribed topics

- `twist`
  - Type in `ground_vehicle_twist_odometry_node`: `geometry_msgs/msg/Twist`
  - Type in `ground_vehicle_twist_odometry_node_w_timestamp`: `geometry_msgs/msg/TwistStamped`

## Published topics

- `odom` (`nav_msgs/msg/Odometry`)

## Services

- `reset_odom` (`std_srvs/srv/Empty`)

## TF

- Published (if `publish_tf=True`): `<odometry_frame> -> <base_frame>`

## Parameters

- `odometry_frame` (string, default: `odom`)
- `base_frame` (string, default: `base_link`)
- `publish_tf` (bool, default: `true`)
- `expected_incoming_twist_msg_rate` (double, default: `40.0`)

Default parameter file:

- `config/example_ground_vehicle_twist_odometry.yaml`

## Launch file

Launch file:

- `launch/ground_vehicle_twist_odometry.launch.py`

Key arguments:

- `use_sim_time` (bool)
- `namespace` (string)
- `robot_name` (string)
- `params_file` (path to YAML file)
- `odometry_frame` (string, without robot prefix)
- `base_frame` (string, without robot prefix)
- `publish_tf` (bool)
- `expected_incoming_twist_msg_rate` (double)
- `use_timestamped_twist` (bool)
- `topic_remappings` (string)
- `node_options` (string)
- `logging_options` (string)

Notes:

- `odometry_frame` and `base_frame` launch arguments are automatically prefixed with `robot_prefix` derived from `robot_name`.
- Launch-level parameter values override values from `params_file`.

## Usage examples

Build and source:

```bash
cd <workspace_path>
colcon build --merge-install --symlink-install
source install/setup.bash
```

Launch with defaults (Twist input):

```bash
ros2 launch ground_vehicle_twist_odometry ground_vehicle_twist_odometry.launch.py
```

Launch using `TwistStamped` input:

```bash
ros2 launch ground_vehicle_twist_odometry ground_vehicle_twist_odometry.launch.py \
  use_timestamped_twist:=True
```

Launch in a robot namespace with custom frames:

```bash
ros2 launch ground_vehicle_twist_odometry ground_vehicle_twist_odometry.launch.py \
  namespace:=fleet \
  robot_name:=robot_01 \
  odometry_frame:=odom \
  base_frame:=base_link
```

Reset odometry:

```bash
ros2 service call /robot/ground_vehicle_twist_odometry/reset_odom std_srvs/srv/Empty {}
```

Adjust the service path according to your effective namespace and node name.
