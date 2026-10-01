# ground_vehicle_twist_odometry

This package computes planar odometry (`x`, `y`, `yaw`) from samples of the base velocity.
It provides a C++ library that runs without initializing ROS and two ROS 2 executables that
accept `geometry_msgs/msg/Twist` or `geometry_msgs/msg/TwistStamped`.

## What this package launches

The launch file starts `ground_vehicle_twist_odometry_node`, which accepts `Twist`.
The `ground_vehicle_twist_odometry_node_w_timestamp` executable accepts `TwistStamped`.
Both use the same odometry library.

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

The input velocities must describe the origin of `base_frame`, expressed in that frame:
`linear.x` and `linear.y` in m/s, and `angular.z` in rad/s. This contract applies regardless
of whether the robot uses differential, steering, or omnidirectional drive.

The first valid sample establishes the starting time and zero pose. When the next sample arrives,
the library averages the two planar twists and integrates that average over the complete time
interval. The calculation accounts for the robot turning while it moves, including lateral
velocity. It is exact for a constant body twist and approximates motion between changing samples.
The published pose belongs to the new sample time; `odom.twist` contains the *new instantaneous*
twist, not the average used to integrate the pose.

`Twist` has no timestamp, so its executable assigns the node's reception time to each sample.
`TwistStamped` uses `header.stamp`; the publisher must set it to the measurement time and use
a clock consistent across samples. The wrapper does not transform velocities from another frame.

Repeated timestamps replace the retained velocity without integrating. Older timestamps and
non-finite velocities are rejected without changing the pose or retained sample. Every positive
time interval is integrated in full, including an unusually long one. The
`expected_incoming_twist_msg_rate` parameter only triggers a warning when the measured rate
falls below its configured value; it never changes the integration. Set it to the minimum rate
accepted by your producer. `reset_odom` clears the pose and retained sample, so the next valid
sample establishes a new origin.

The published pose and twist covariance diagonals retain fixed values from the earlier node.
These values are **not calibrated uncertainty estimates**; the input `Twist` does not supply
the measurement uncertainty needed to calculate them.

## Use the C++ odometry library

The installed `ground_vehicle_twist_odometry::twist_odometry` target exposes
`GroundVehicleTwistOdometry` without requiring a ROS node:

```cmake
find_package(ground_vehicle_twist_odometry REQUIRED)
target_link_libraries(my_node ground_vehicle_twist_odometry::twist_odometry)
```

```cpp
#include <ground_vehicle_twist_odometry/ground_vehicle_twist_odometry.hpp>

ground_vehicle_twist_odometry::GroundVehicleTwistOdometry odometry;
odometry.update({{0.0, 0.0, 0.0}, 0});
const auto result = odometry.update({{1.0, 0.0, 0.0}, 1'000'000'000});
// The integrated position is x = 0.5 m because the samples are averaged.
const auto pose = odometry.pose();
```

The timestamp is an integer count of nanoseconds from a consistent clock. `update` reports
whether it initialized, integrated, replaced a same-time sample, or rejected an input.
Calls to `update`, `reset`, and state getters must be serialized by the consumer.

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
  node_args:='{"name":"twist_odometry","output":"screen","emulate_tty":true,"respawn":true,"respawn_delay":2.0,"remappings":[["twist","velocity_echo"],["odom","base_odometry"]],"ros_arguments":["--log-level","debug"]}'
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

### Example 4: use timestamped velocity samples

```bash
ros2 run ground_vehicle_twist_odometry ground_vehicle_twist_odometry_node_w_timestamp \
  --ros-args -r __ns:=/robot -r twist:=velocity_echo \
  --params-file /path/to/my_ground_vehicle_twist_odometry.yaml
```

The YAML must match the node name `ground_vehicle_twist_odometry`. The producer must publish
`TwistStamped` and provide a measurement timestamp in `header.stamp`.

### Example 5: reset odometry

```bash
ros2 service call /robot/reset_odom std_srvs/srv/Empty {}
```

Adjust the service name if you change the namespace or remap `reset_odom`.
