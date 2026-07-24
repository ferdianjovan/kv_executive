# KV Executive

## External odometry and PX4

There are two different coordinate origins in this system:

- PX4 EKF local position normally starts close to `(0, 0, 0)`.
- Camera/VIO/LIO odometry may use a world-aligned origin such as
  `x500_depth/odom`, which is offset from the PX4 origin.

PX4 estimates the bias between those origins when it fuses external vision.
Consequently, external-vision fusion does **not** guarantee that
`/mavros/local_position/odom` will contain the same coordinates as the
external odometry. The planner therefore consumes external odometry directly
and transforms it with TF into the OctoMap frame.

### Simulation

Start MAVROS with its `odometry` plugin connected to the Gazebo odometry:

```bash
ros2 launch kv_executive px4.launch.py \
  fcu_url:="udp://:14540@127.0.0.1:14580" \
  drone_name:=x500_depth
```

`drone_name` derives the topic `/<drone_name>/odometry` and the frames
`<drone_name>/odom` and `<drone_name>/base_link`. The three explicit odometry
arguments remain available when a real estimator uses another convention.

The launch file remaps the external `nav_msgs/Odometry` topic to
`/mavros/odometry/out`. In MAVROS terminology, `out` means data sent to the
flight controller; `/mavros/odometry/in` is odometry received from it.
MAVROS converts ROS ENU/FLU conventions to MAVLink `ODOMETRY`, and PX4
receives it as `vehicle_visual_odometry`.

The MAVROS odometry plugin requires these zero-translation static rotations,
which the launch file publishes automatically:

- `<drone_name>/odom` to `<drone_name>/odom_ned`: ENU to NED
  (`roll=pi`, `yaw=pi/2`).
- `<drone_name>/base_link` to `<drone_name>/base_link_frd`: FLU to FRD
  (`roll=pi`).

It also publishes `px4_odom` to `px4_odom_ned` for odometry received from the
FCU. Set `publish_frame_conversions:=false` only when all these transforms are
already supplied by another TF publisher.

The parameter configurator waits for valid odometry before changing PX4. It
force-pulls the complete PX4 parameter catalogue through
`/mavros/param/pull`, then sets:

- `EKF2_EV_CTRL=15`: fuse horizontal position, vertical position, 3D velocity,
  and yaw.
- `EKF2_HGT_REF=1`: keep GPS as the height reference while external vision is
  being flight-tested.

The native MAVROS `/mavros/param/set` service is used instead of writing the
parameter plugin's ROS parameter mirror. This prevents startup races where
MAVROS reports an existing PX4 parameter as undeclared.

Use a smaller `ekf2_ev_ctrl` bitmask if the estimator does not provide every
measurement reliably. For example, `7` excludes external yaw. Set
`configure_px4:=false` if the parameters have already been configured or must
not be changed automatically. Use `ekf2_hgt_ref:=0` for barometric height on a
GNSS-denied vehicle. To deliberately use vision height, pass
`ekf2_hgt_ref:=3`; changing this parameter requires a PX4 reboot.

External vision normally provides a local position, not latitude/longitude.
PX4 modes such as Mission, Return, Hold, and Orbit still require a global
position estimate. Keep simulated/real GNSS enabled, or set a global EKF
origin explicitly for a vision-only system. `COM_ARM_WO_GPS` only relaxes an
arming check; it does not create a global position and is not changed here.

`kv_world.launch.py` applies `EKF2_HGT_REF=1` before SITL starts EKF2, so a
previously persisted Vision setting cannot destabilize height on the next
simulation run. It can also be repaired manually in the PX4 shell:

```text
param show EKF2_HGT_REF
param reset EKF2_HGT_REF
param save
reboot
```

For the current x500 SITL build this restores the GPS height reference
(`EKF2_HGT_REF=1`). Confirm `vehicle_global_position` is valid before arming.

The warning `command 520 unsupported` is unrelated to estimator validity.
Command 520 is the deprecated MAVLink autopilot-capabilities request used
during MAVROS discovery. PX4 returning `MAV_RESULT_UNSUPPORTED` only means
that this compatibility request is not implemented; it does not inhibit
arming or external-odometry fusion.

The planner can be started separately:

```bash
ros2 launch kv_executive autonomy.launch.py \
  drone_name:=x500_depth
```

Or both can be started together:

```bash
ros2 launch kv_executive autonomy.launch.py \
  start_mavros:=true \
  drone_name:=x500_depth
```

The default planner home is the first external-odometry pose. This keeps
altitude bounds in the same coordinate system as OctoMap. Set
`home_from_first_odometry:=false` only when MAVROS `HomePosition` has a valid TF
connection to the OctoMap frame.

### Real vehicle

The topic supplied to `external_odometry_topic` must be the output of an
estimator, for example VIO from a camera or LIO from a 3D LiDAR. A camera image
or LiDAR point cloud alone is not odometry.

Before flight:

1. Change the topic and both frame arguments to exactly match the estimator's
   `nav_msgs/Odometry` message.
2. Set `use_sim_time:=false`.
3. Set `EKF2_EV_POS_X/Y/Z` to the sensor's lever arm from the vehicle centre.
4. Measure and configure `EKF2_EV_DELAY` in QGroundControl if necessary. That
   parameter requires an FCU reboot.
5. Confirm that pose is expressed in the parent frame and twist is expressed
   in `child_frame_id`. The estimate should arrive at 30–50 Hz with realistic,
   nonzero covariance.
6. Verify EKF innovation/test-ratio data before enabling autonomous flight.

Example:

```bash
ros2 launch kv_executive px4.launch.py \
  use_sim_time:=false \
  fcu_url:=/dev/ttyACM0:921600 \
  external_odometry_topic:=/vio/odometry \
  external_odom_parent_frame:=vio_odom \
  external_odom_child_frame:=base_link \
  ekf2_ev_ctrl:=15 \
  ekf2_ev_pos_x:=0.10
```

Useful checks:

```bash
ros2 topic hz /x500_depth/odometry
ros2 topic echo /x500_depth/odometry --once
ros2 topic echo /mavros/local_position/odom --once
ros2 run tf2_ros tf2_echo map x500_depth/odom
ros2 run tf2_ros tf2_echo x500_depth/odom_ned x500_depth/odom
ros2 run tf2_ros tf2_echo x500_depth/base_link_frd x500_depth/base_link
```

`/mavros/local_position/odom` is deliberately labelled `px4_odom` by this
launch file because it has the PX4 EKF origin, not necessarily the external
`map` origin.

Any future path follower must transform each `planned_path` pose from `map`
into the PX4 local frame before publishing a MAVROS local-position setpoint.
Sending the map coordinates directly to `/mavros/setpoint_position/local`
would recreate the same origin-offset bug at the execution stage.
