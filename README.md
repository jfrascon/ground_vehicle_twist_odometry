# ground_vehicle_twist_odometry

This package provides a ROS 2 node that computes planar odometry (`x`, `y`, `yaw`) by integrating
incoming `geometry_msgs/msg/Twist` messages.

## What this package launches

The launch file starts one `ground_vehicle_twist_odometry_node` node.

That node:

- subscribes to `twist`
- publishes `nav_msgs/msg/Odometry` on `odom`
- provides a `reset_odom` service with type `std_srvs/srv/Empty`
- publishes the TF transform `<odometry_frame> -> <base_frame>` when `publish_tf` is `true`

The `namespace` launch argument sets the ROS namespace used by that node. Use a different namespace
for each robot instance in multirobot scenarios so topics, node names, and parameters do not
collide.

The package also installs `config/example_ground_vehicle_twist_odometry.yaml`, which contains the
default functional parameters for the node. The launch file configures `use_sim_time` separately.

## Configuration model

`params_file` is loaded when it resolves to a non-empty path.

If you do not pass `params_file`, the default value points to the example YAML installed by this
package. If you pass your own YAML file, that file is used instead.

The launch file does not expose individual functional node parameters. Consequently, CLI launch
arguments cannot override selected values from `params_file`. To change a functional parameter,
edit or replace the parameter file.

`use_sim_time` is the deliberate exception. The launch environment decides whether the node uses
the system clock or the ROS simulation clock. The launch file appends this value after loading the
YAML file, so the `use_sim_time` launch argument takes precedence if a custom YAML file also defines
the parameter. Parameter files should therefore omit `use_sim_time`.

When `params_file_allow_substs` is `True`, expressions such as `$(var robot_name)` can use keys
already present in the launch context. This is mainly useful when a parent launch file includes this
launch file and provides those keys. Literal YAML values are always used as written.

`node_args` accepts one JSON object with the supported `launch_ros.actions.Node` arguments.
It can configure the node name, remappings, output, respawn behavior, and ROS arguments.

## How the node integrates twist

The node starts integrating when the first `twist` message is received. For each message, it
computes `dt` from the current node clock, integrates planar motion, publishes odometry, optionally
publishes TF, and stores the current timestamp as the previous timestamp for the next integration
step.

If `dt` is negative, that integration step is skipped and a warning is logged.

`expected_incoming_twist_msg_rate` is a runtime sanity check. If the incoming twist frequency is
lower than this value, the node logs a warning.

## Examples

### Example 1: launch with the default parameter file

```bash
ros2 launch ground_vehicle_twist_odometry ground_vehicle_twist_odometry.launch.py
```

This command uses the default `params_file`, which points to
`config/example_ground_vehicle_twist_odometry.yaml`.

That file contains the functional parameter tree and uses literal values:

```yaml
/**/ground_vehicle_twist_odometry:
  ros__parameters:
    odometry_frame: robot_odom
    base_frame: robot_base_link
    publish_tf: true
    expected_incoming_twist_msg_rate: 40.0
```

### Example 2: launch with a custom parameter file

```bash
ros2 launch ground_vehicle_twist_odometry ground_vehicle_twist_odometry.launch.py \
  namespace:=robot_01 \
  params_file:=/path/to/my_ground_vehicle_twist_odometry.yaml \
  node_args:='{"name":"twist_odometry","output":"screen","emulate_tty":true,"respawn":true,"respawn_delay":2.0,"remappings":[["twist","cmd_vel"],["odom","wheel_odometry"]],"ros_arguments":["--log-level","debug"]}'
```

The custom `params_file` owns every functional node parameter. Use the `use_sim_time` launch
argument to select the clock.

The top-level YAML key in this example is `/**/twist_odometry` because the command above sets the
node name to `twist_odometry` through `node_args`. If you use a different node name, update that
YAML key accordingly.

```yaml
/**/twist_odometry:
  ros__parameters:
    odometry_frame: odom
    base_frame: base_link
    publish_tf: true
    expected_incoming_twist_msg_rate: 50.0
```

### Example 3: substitutions provided by a parent launch file

The parameter file may contain substitutions when a parent launch file already owns the substituted
context keys. For example, a parent can provide the frame names used by several child launch files.

The child launch still receives one parameter file for its functional configuration:

```yaml
/**/ground_vehicle_twist_odometry:
  ros__parameters:
    odometry_frame: $(var robot_odometry_frame)
    base_frame: $(var robot_base_frame)
    publish_tf: true
    expected_incoming_twist_msg_rate: 50.0
```

### Example 4: reset odometry

```bash
ros2 service call /robot/ground_vehicle_twist_odometry/reset_odom std_srvs/srv/Empty {}
```

Adjust the service name to the namespace and node name used by your launch invocation.
