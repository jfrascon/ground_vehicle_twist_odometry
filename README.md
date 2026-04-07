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

The package also installs `config/example_ground_vehicle_twist_odometry.yaml`, which configures the
node and uses `$(var ...)` substitutions so launch arguments can override selected parameter values.

## Configuration model

`params_file` is always loaded as the node parameter file.

If you do not pass `params_file`, the default value points to the example YAML installed by this
package. If you pass your own YAML file, that file is used instead.

Inside the YAML file, values written as `$(var <launch_argument_name>)` are resolved from the
current launch context. Values written as literals are used as-is.

The launch file also exposes these helper-based arguments:

- Topic remappings (`remappings`)
- Logging options (`logging_options`)
- Node options (`node_options`)

Use `logging_options` to change the node log level, for example `log-level=debug`.

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

That file defines the parameter tree, but each value is delegated to a launch argument through
`$(var ...)`. If you launch this command without overriding those arguments, the
`DeclareLaunchArgument` defaults in `ground_vehicle_twist_odometry.launch.py` provide the values. If
you override a launch argument from the CLI or from a parent launch file, the YAML file picks up
that value.

```yaml
/**/ground_vehicle_twist_odometry:
  ros__parameters:
    use_sim_time: $(var use_sim_time)
    odometry_frame: $(var odometry_frame)
    base_frame: $(var base_frame)
    publish_tf: $(var publish_tf)
    expected_incoming_twist_msg_rate: $(var expected_incoming_twist_msg_rate)
```

### Example 2: launch with a custom parameter file and CLI overrides

```bash
ros2 launch ground_vehicle_twist_odometry ground_vehicle_twist_odometry.launch.py \
  namespace:=robot_01 \
  params_file:=/path/to/my_ground_vehicle_twist_odometry.yaml \
  odometry_frame:=odom \
  base_frame:=base_link \
  remappings:="twist:=cmd_vel,odom:=wheel_odometry" \
  logging_options:="log-level=debug" \
  node_options:="name=twist_odometry,output=screen,emulate_tty=True,respawn=True,respawn_delay=2.0"
```

The custom `params_file` can mix literal values and `$(var ...)` substitutions. For example, this
file hardcodes `odometry_frame`, `base_frame`, and `publish_tf`, but it still keeps `use_sim_time`
and `expected_incoming_twist_msg_rate` configurable through launch arguments.

The top-level YAML key in this example is `/**/twist_odometry` because the command above sets the
node name to `twist_odometry` through `node_options`. If you use a different node name, update that
YAML key accordingly.

```yaml
/**/twist_odometry:
  ros__parameters:
    use_sim_time: $(var use_sim_time)
    odometry_frame: odom
    base_frame: base_link
    publish_tf: true
    expected_incoming_twist_msg_rate: $(var expected_incoming_twist_msg_rate)
```

### Example 3: launch with a mostly literal parameter file

In this style, the YAML file fully defines the node parameters except `use_sim_time`, which remains
connected to the launch argument so the same file can be used in both real and simulated runs.

Launch arguments still provide `namespace`, remappings, logging options, and node options, but not
the node parameter values listed below.

```yaml
/**/ground_vehicle_twist_odometry:
  ros__parameters:
    use_sim_time: $(var use_sim_time)
    odometry_frame: robot_odom
    base_frame: robot_base_link
    publish_tf: true
    expected_incoming_twist_msg_rate: 50.0
```

### Example 4: reset odometry

```bash
ros2 service call /robot/ground_vehicle_twist_odometry/reset_odom std_srvs/srv/Empty {}
```

Adjust the service name to the namespace and node name used by your launch invocation.
