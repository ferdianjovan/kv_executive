# KV Executive

The executive runs a YASMIN state machine with `TAKEOFF`, `SWEEP`, and
`EXECUTE_PATH` states. The motion planner remains a separate node with the
`plan_path` service. YASMIN is a required Python runtime dependency; the
executive uses its state-machine implementation directly.

## Start and execute a mission

Build the package and source the generated action and service bindings before
launching:

```bash
colcon build --packages-select kv_executive --symlink-install
source install/setup.bash
```

Start the Gazebo/PX4 simulation first, then launch MAVROS, the planner, and the
executive together:

```bash
ros2 launch kv_executive autonomy.launch.py \
  drone_name:=x500_depth \
  start_mavros:=true
```

If MAVROS is already running, omit `start_mavros:=true`.

The default launch starts the nodes and waits for an explicit mission request:

```bash
ros2 action send_goal /execute_mission kv_executive/action/ExecuteMission '{}' --feedback
```

`execute_mission` is now an action, replacing the old Trigger service. Its name
is relative, so its full name follows any node namespace. The action accepts
the goal immediately and provides `state` and `message` feedback during the
flight. Its final result contains `success` and `message`; the action status is
`SUCCEEDED`, `CANCELED`, or `ABORTED`. Only one goal runs at a time. After that
goal reaches a terminal status, the server accepts another goal without a
node restart.

The default goal includes takeoff and requires a disarmed vehicle. To start
another sweep and path while the vehicle is already airborne in `AUTO.LOITER`:

```bash
ros2 action send_goal /execute_mission kv_executive/action/ExecuteMission \
  '{skip_takeoff: true}' --feedback
```

### Cancel a mission

With the installed ROS 2 Jazzy CLI, pressing Ctrl+C after the goal is accepted
requests cancellation and waits for the final result. Alternatively, request
cancellation from another terminal:

```bash
ros2 service call /execute_mission/_action/cancel_goal action_msgs/srv/CancelGoal '{}'
```

The empty request has a zero goal UUID and timestamp, which means cancel all
goals on this action server; the executive allows only one active goal. These
are the action protocol's hidden services, visible with
`ros2 service list --include-hidden-services`. Use the node's namespace in the
command when applicable. A nonempty `goals_canceling` response confirms that
the cancellation was accepted. The final action result arrives after bounded
flight-mode recovery; cancellation does not land or disarm the vehicle.

Cancellation stops the active motion target, holds the latest valid position,
and requests `AUTO.LOITER`. A mission failure follows the same recovery and
returns `ABORTED`. Setpoints continue until telemetry confirms control has
left `OFFBOARD`/`AUTO.TAKEOFF`. If the loiter handoff fails, the final result
reports the failure and the hold stream remains active.

The planner can also be called independently from a terminal:

```bash
ros2 service call /plan_path kv_executive/srv/PlanPath '{}'
```

It returns `success`, `message`, and the new `nav_msgs/Path`; requesting a path
alone does not fly it.

Planning requires external odometry, a ground-home reference, and a nonempty
OctoMap. The response lists each missing input and its configured topic rather
than reporting an OMPL failure for unavailable data. `octomap_topic` defaults
to `/octomap_binary`; its subscription requests the server's retained map so
the planner can start after map publication.

With `kv_world.launch.py`, map input flows from
`/<drone_name>/mid_360/points` through the URDF self-filter to
`/<drone_name>/mid_360/points_self_filtered`, then into `octomap_server`.
If OctoMap is missing, check this pipeline and the simulator launch logs.
The self-filter restarts after an unexpected process exit and saves fault
traces in the launch logs. Missing map data continues to prevent path flight.

Internal callers can use
`node.send_mission_goal(skip_takeoff=False, feedback_callback=None)`. It returns
an asynchronous future for the client goal handle; check `handle.accepted`,
then use `handle.get_result_async()` to observe completion or
`handle.cancel_goal_async()` to request cancellation. `node.cancel_mission()`
requests cancellation of the currently executing server goal, whether started
internally or externally, and returns the cancel-response future (`None` when
idle).

Other Python nodes use an `rclpy.action.ActionClient` with
`kv_executive.action.ExecuteMission` and the relative name `execute_mission`.
The installed `executive_node.py` runs a `MultiThreadedExecutor` so action
callbacks, telemetry, service replies, and setpoint publication continue while
the YASMIN sequence runs.

To request one mission automatically when the executive starts:

```bash
ros2 launch kv_executive autonomy.launch.py \
  drone_name:=x500_depth \
  start_mavros:=true \
  auto_start_mission:=true
```

Mission parameters are installed in `config/mission.yaml` and apply only to
the executive. Supply a different ROS parameter file with
`mission_config:=/absolute/path/mission.yaml`; the planner has its own
`planner_config` argument. The launch argument `auto_start_mission` overrides
the value in the mission parameter file.

FCU state and local odometry have separate freshness deadlines. PX4 sends
HEARTBEAT at 1 Hz, and MAVROS publishes `/mavros/state` from those heartbeats.
The executive uses reliable, transient-local delivery for that topic and a
`state_timeout` of 10 seconds, matching MAVROS's default `conn_timeout`.
`telemetry_timeout` remains 2 seconds for the faster local-odometry stream.
An explicit `connected=false` state aborts immediately. Missing or stale data
errors identify the topic, and stale-data errors include the receive age and
configured deadline. These deadlines measure monotonic wall time since receipt.

State progress is logged and published on `mission_status` (by default
`/mission_status`). Messages include `TAKEOFF: started`, `TAKEOFF: done`,
corresponding messages for the other states, sweep progress, completed path
legs, and the final result. FCU mode changes and MAVROS status messages are
also forwarded there to help diagnose unexpected flight-mode changes. Inspect
them with:

```bash
ros2 topic echo /mission_status
```

One mission runs the following sequence:

- `TAKEOFF` requires the vehicle to be disarmed initially unless the goal sets
  `skip_takeoff: true`, which requires an armed vehicle in `AUTO.LOITER` and
  skips this state. Only use that option when the vehicle is already airborne;
  the executive does not subscribe to landed-state telemetry. The takeoff state
  pulls the PX4 parameter catalogue through MAVROS, reads `MIS_TAKEOFF_ALT`, requests
  `AUTO.TAKEOFF`, arms, and waits for that climb relative to the initial PX4
  local height.
- `SWEEP` holds position and performs one full turn in six 60-degree steps,
  returning to the starting heading. `sweep_step_duration` is the yaw ramp
  duration for each step, five seconds by default. After each ramp, the
  executive waits for the measured position and heading to converge and
  remain within tolerance for `hold_duration`.
- `EXECUTE_PATH` requests one path from `plan_path`, splits it into three
  equal lengths along the path, and executes all three legs consecutively
  using MAVROS local-position setpoints. Split boundaries are interpolated;
  unequal waypoint spacing does not change where the thirds fall. The sweep
  runs once before path execution. Each movement uses a yaw-only heading from
  `atan2(dy, dx)` in PX4 local coordinates. The drone turns in place and waits
  for heading convergence before moving forward along a segment; it keeps
  that heading during translation. Vertical moves and repeated points retain
  the previous heading. Planner pose orientations do not set the flight yaw.
  Turns use the same yaw rate as the sweep (`sweep_step_duration` per 60 degrees).

On completion, the executive confirms `AUTO.LOITER` and returns the action
result. After taking flight control, cancellation or failure recovery refreshes
the hover target from current local odometry and requests `AUTO.LOITER` when
the vehicle is armed and still in `OFFBOARD` or `AUTO.TAKEOFF`. With stale
telemetry, it retains the last validated target. If the handoff fails while the
vehicle remains armed in OFFBOARD, the hold setpoint continues streaming and the
returned message includes the recovery failure. A request rejected because
the vehicle was already armed does not initiate recovery mode changes.
Recovery also preserves an observed pilot or failsafe flight-mode takeover.

### PX4 returning home after the mission

PX4 normally records a home position when arming; that alone does not request
a return. PX4 monitors the setpoint stream while in OFFBOARD. If it stops before
the mode handoff is confirmed, PX4 can trigger its offboard-loss failsafe after
`COM_OF_LOSS_T`; the configured action can be Return or Land. Reaching home and
landing can therefore be a failsafe response, rather than a command from this
mission. Stopping local setpoints after PX4 has confirmed `AUTO.LOITER` does
not itself trigger offboard loss.

The normal completion sequence already confirms `AUTO.LOITER` before stopping
setpoints, so offboard loss alone does not establish the cause of a return
after successful completion. The x500 SITL configuration also defaults
`NAV_DLL_ACT` to Return (`2`), so a ground-station data-link timeout can cause
a return while the vehicle is loitering. The actual mode transitions, PX4
status messages, and flight log are needed to establish which failsafe fired.

The executive keeps the independent setpoint timer alive until a confirmed
handoff, including during cancellation and recovery. It does not change PX4's
failsafe parameters or request Return/Land. Check the actual FCU mode and
messages if a return still occurs:

```bash
ros2 topic echo /mavros/state
ros2 topic echo /mavros/statustext/recv
```

`CMD: Unexpected command 176, result 0` is an accepted mode-change
acknowledgement (`MAV_CMD_DO_SET_MODE`, `MAV_RESULT_ACCEPTED`) without a pending
command-plugin waiter. MAVROS's `set_mode` service sends `SET_MODE` directly;
the executive confirms the actual resulting mode through `/mavros/state`.
This warning alone does not indicate a rejected mode change or telemetry loss.

In the PX4 shell, inspect the offboard-loss delay and action:

```text
param show COM_OF_LOSS_T
param show COM_OBL_RC_ACT
param show COM_DL_LOSS_T
param show NAV_DLL_ACT
```

RC loss, data-link loss, battery, and estimator failsafes can also cause mode
changes; flight logs and PX4 status messages identify the trigger. The
executive aborts when it observes an unexpected mode instead of overriding a
PX4 or pilot failsafe. `waiting for service to become available` from the old
`ros2 service call /execute_mission ...` command now means it is using the
removed Trigger interface; use `ros2 action send_goal` for this version.

Cancel the action while keeping the executive node running. Interrupting the
executive process itself stops its publisher, which can trigger offboard loss
if PX4 has not left OFFBOARD; that is different from canceling a goal.

The active local setpoint is streamed at 20 Hz independently of state-machine
waits while controlling the vehicle in OFFBOARD. Planning takes place in
`AUTO.LOITER`. The executive warms up the setpoint stream for two seconds
before requesting OFFBOARD. `path_speed` defaults to 0.75 m/s,
`position_tolerance` to 0.3 m, and `yaw_tolerance` to five degrees. The mission
requires `hold_duration` (0.5 seconds by default) within both tolerances at
each target. The mission file also defines telemetry, service, planning, mode,
takeoff, and waypoint timeouts so a missing response cannot leave a state
waiting indefinitely.

An unexpected flight mode during planning or OFFBOARD warmup aborts the mission
without switching back to OFFBOARD. The vehicle must remain in `AUTO.LOITER`
while planning. The first path point must stay within `position_tolerance` of
the vehicle, including after warmup; a small alignment move uses the configured
speed ramp. Larger drift requires a fresh mission plan.

The planner's default `minimum_altitude` is 2 m and `bound_margin` is zero,
which permits PX4's usual 2.5 m takeoff height. The separate 0.30 m obstacle
clearance still applies. If `MIS_TAKEOFF_ALT` or the planner altitude bounds
are changed, keep the takeoff height inside those bounds.

## Planned paths and PX4 local coordinates

The planner returns paths in its map frame; MAVROS local setpoints use the PX4
estimator's local origin. Before execution, the executive matches the robot's
TF pose in the path frame to `/mavros/local_position/odom` at that odometry
timestamp. It computes the map-to-PX4-local transform from those two body poses
and applies it to the path. This includes translation and orientation; it does
not assume the origins coincide or publish an identity `map` to `px4_odom`
transform.

This timestamped alignment stays fixed for the returned path. Estimator
origin resets during traversal are not handled. The current baseline executes
the three planned legs without dynamic obstacle replanning.

The executive's `local_odometry_topic` defaults to
`/mavros/local_position/odom` and `local_frame` to `px4_odom`.
`robot_base_frame` defaults to `<drone_name>/base_link` and can be overridden
as a launch argument. TF must connect the path frame to that body frame at
the MAVROS odometry timestamp. The mission stops if telemetry or the required
transform cannot be obtained within the configured timeouts.

## Mission tests

After building and sourcing the workspace, run the mission tests with:

```bash
colcon test --packages-select kv_executive --ctest-args -R '^(mission_|planner_inputs$)'
colcon test-result --verbose
```

These cover geometry, state sequencing, failure handling, and the public ROS
action using an isolated simulated FCU. The action test checks feedback,
cancellation, terminal results, repeated goals, and independent setpoint
publication during mode-service waits. A simulated OFFBOARD-loss watchdog
checks the LOITER handoff and deliberate publisher loss; path tests check
forward-facing yaw after frame conversion and turns at corners. State updates
run at 1 Hz independently of odometry; tests exercise heartbeat gaps, expired
state, stale odometry, and an explicit disconnect. Planner tests use the real
C++ service to check missing inputs and planning with a map published before
the planner starts. These tests do not replace SITL or vehicle flight validation.

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
Start the planner while the vehicle is grounded. Restarting it in flight with
`home_from_first_odometry:=true` would treat the airborne altitude as ground
home and can exclude the current position from its altitude bounds.

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

The mission executive performs this origin alignment before publishing to
`/mavros/setpoint_position/local`, as described above. External-vision fusion
alone does not make planner coordinates valid PX4 local setpoints.
