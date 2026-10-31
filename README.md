# KV Executive

The executive starts each mission in the nested YASMIN state machine
`Exploration`. It takes off once, then repeats `SWEEP` and `EXECUTE_PATH`.
Confirmed smoke or fire interrupts Exploration and enters the `Verify` higher
state after a bounded LOITER handoff. Verify approaches and reacquires smoke,
centers a persistent fire, and resumes Exploration without another takeoff.
A valid battery return/reserve alert aborts the action with bounded LOITER
recovery; a new action goal is required to resume after an abort. The motion
planner, detection monitor and battery monitor run as separate nodes. The
planner exposes `plan_path` for Exploration and `plan_inspection_path` for Verify.
YASMIN is a required Python runtime dependency; the executive uses its
state-machine implementation directly.

## Start and execute a mission

Build the package and source the generated action and service bindings before
launching:

```bash
colcon build --packages-up-to kv_executive --symlink-install
source install/setup.bash
```

Start the Gazebo/PX4 simulation first, then start MAVROS in its own terminal:

```bash
ros2 launch kv_executive px4.launch.py drone_name:=kv_drone
```

Wait for connected MAVROS state and local odometry before starting autonomy
in another terminal:

```bash
ros2 launch kv_executive autonomy.launch.py drone_name:=kv_drone
```

Autonomy defaults `executive_udp_only` to `true`: the executive uses Fast DDS
over UDP with shared memory disabled. This is a scoped workaround for the
observed native Python executor crashes; it leaves the planner and
MAVROS transport environment alone. The launch prints `Mission executive: Fast DDS
UDP-only transport (shared memory disabled)`. Explicit localhost-only settings
select a loopback-only UDP profile. Other selected RMW implementations are
left alone. To preserve an existing custom Fast DDS XML/discovery configuration,
set `executive_udp_only:=false`. See the
[crash investigation](docs/native_crashes.md) for the evidence and test limits.

Autonomy starts the planner and executive; `start_mavros` defaults to `false`.
When MAVROS is already running, autonomy uses that instance and does not apply
the frame-label overrides in `kv_executive/px4.launch.py`. Both the explicit
`px4_odom` label from this package's MAVROS launch and the stock MAVROS `map`
label on its local-odometry stream are supported, as described below.

Start the battery and detection monitors together in another terminal before
the first mission's takeoff:

```bash
ros2 launch kv_executive monitoring.launch.py drone_name:=kv_drone use_sim_time:=true
```

Monitoring starts both monitors by default. It defaults `drone_name` to
`kv_drone` and `use_sim_time` to `true`; match these values to autonomy and
use `use_sim_time:=false` on a real vehicle. Its
`start_battery_monitor` and `start_detection_monitor` arguments can disable
either monitor. Monitor switches and configuration files belong to the
monitoring launch; autonomy does not accept them. Start the detectors
separately with
`ros2 launch kv_perception perception.launch.py drone_name:=kv_drone`.

The standalone detection monitor also defaults `detection_monitor_udp_only` to
`true`, applying the same scoped Fast DDS UDP workaround. `monitoring.launch.py`
forwards this option to the detection monitor; the native battery monitor keeps
its inherited transport settings. Set `detection_monitor_udp_only:=false` to
preserve a custom transport configuration. Other RMW implementations are left
unchanged, and localhost-only discovery remains limited to loopback. Detection
monitor launches enable Python fault traces for any subsequent native crash.

The default launch starts the nodes and waits for an explicit mission request:

```bash
ros2 action send_goal /execute_mission kv_executive/action/ExecuteMission '{}' --feedback
```

`execute_mission` is now an action, replacing the old Trigger service. Its name
is relative, so its full name follows any node namespace. The action accepts
the goal immediately and provides `state` and `message` feedback during the
flight. The same goal stays active across Exploration cycles and transitions
through Verify; finishing a path or verification does not return an action
result or release the mission slot. Canceling returns `CANCELED`; battery
alerts, errors, and Abort/Return requests return `ABORTED`. The result
contains `success` and `message`; there is no automatic successful completion
for repeating Exploration. Only one goal runs at a time.
After that goal reaches a terminal status, the server accepts another goal
without a node restart.

The default goal includes one takeoff and requires a disarmed vehicle. To start
repeating Exploration while the vehicle is already airborne in `AUTO.LOITER`:

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

### Request Verify or abort a mission

Internal code can stop Exploration with `node.request_mission_state('Abort')`.
External nodes publish the same case-sensitive name as `std_msgs/msg/String`
on the relative topic `mission_state_request`, for example:

```bash
ros2 topic pub --once /mission_state_request std_msgs/msg/String '{data: Abort}'
```

An Abort request stops the current motion, requests `AUTO.LOITER`, and ends
the action as `ABORTED`. It does not fly home, land, or resume automatically.
`Verify` requests interrupt Exploration, settle the vehicle in LOITER, and
start the verification sequence within the same action. A Verify request
received during initial takeoff waits until the configured climb is complete.
`Return` remains unimplemented and aborts to LOITER.
Requests are checked during motion and waits. An already dispatched mode or
arming command is observed to its bounded outcome before recovery begins.

To enter Verify manually during Exploration:

```bash
ros2 topic pub --once /mission_state_request std_msgs/msg/String '{data: Verify}'
```

Unsupported names, requests while idle, and requests during cancellation or
recovery are ignored. The Python API returns whether a request was accepted.
Goal-scoped requests include the active action UUID, and pending requests are
cleared when that goal ends. The detection monitor sends `Verify`; the executive
also consumes the battery monitor's `return_alert` directly as described below.

The planner can also be called independently from a terminal:

```bash
ros2 service call /plan_path kv_executive/srv/PlanPath '{}'
```

It returns `success`, `message`, and the new `nav_msgs/Path`; requesting a path
alone does not fly it. Verify uses the explicit endpoint service for targeted
motion; it can also be called independently:

```bash
ros2 service call /plan_to_pose kv_executive/srv/PlanToPose \
  '{target: {header: {frame_id: map}, pose: {position: {x: 5.0, y: 0.0, z: 5.0}, orientation: {w: 1.0}}}}'
```

The target is transformed into the actual OctoMap frame at its header stamp;
a zero stamp requests latest TF. Explicit requests may expand the rolling
planning volume to reach the endpoint, but retain the home-relative altitude
limits and drone/obstacle clearance. Requests with invalid poses, missing TF,
out-of-bounds altitude, or displacement above `maximum_target_distance`
(50 m by default) fail. Approximate OMPL solutions are rejected. Exploration
keeps its original rolling bounds. The planner keeps the existing treatment
of unmapped OctoMap space as free; collision checks use the available occupied
voxels and do not certify unseen space.

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

The mounted MID-360 covers approximately -52 to +7 degrees in elevation,
leaving unobserved air above the aircraft. Verify defaults to the same
occupied-obstacle policy as Exploration: unknown air is permitted, while the
0.75 m inflated body remains checked against current and recent occupied cells.
This assumes unmapped air is traversable; it does not certify unseen space free.
`inspection_require_observed_free_space:=true` restores strict rejection of
unknown body cells and unknown downward columns for deployments with sufficient
sensor coverage. Both modes require actual observed terrain for terrain-relative
flight and retain altitude bounds. Missing or stale maps remain unavailable.

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
the YASMIN state machine runs.

To request repeating Exploration automatically when the executive starts,
start MAVROS, the detectors and `monitoring.launch.py` first with matching
`drone_name` and `use_sim_time`, then run:

```bash
ros2 launch kv_executive autonomy.launch.py \
  drone_name:=kv_drone \
  auto_start_mission:=true
```

Mission parameters are installed in `config/mission.yaml` and apply only to
the executive. Supply a different ROS parameter file with
`mission_config:=/absolute/path/mission.yaml`; the planner has its own
`planner_config` argument. The launch argument `auto_start_mission` overrides
the value in the mission parameter file.

The default remains `false`: bringing the drone online does not itself arm it.
`Exploration` is always the first parent state when a mission is requested.

FCU state and local odometry have separate freshness deadlines. PX4 sends
HEARTBEAT at 1 Hz, and MAVROS publishes `/mavros/state` from those heartbeats.
The executive uses reliable, transient-local delivery for that topic and a
`state_timeout` of 10 seconds, matching MAVROS's default `conn_timeout`.
`telemetry_timeout` remains 2 seconds for the faster local-odometry stream.
An explicit `connected=false` state aborts immediately. Missing or stale data
errors identify the topic, and stale-data errors include the receive age and
configured deadline. These deadlines measure monotonic wall time since receipt.

State progress is logged and published on `mission_status` (by default
`/mission_status`). Messages include `Exploration: started`,
`TAKEOFF: started`, `TAKEOFF: done`, corresponding messages for the other
states, sweep progress, completed path
legs, and the final result. FCU mode changes and MAVROS status messages are
also forwarded there to help diagnose unexpected flight-mode changes. Inspect
them with:

```bash
ros2 topic echo /mission_status
```

The retained `kv_executive/msg/MissionPhase` topic `mission_phase` also records
the active goal UUID, monotonically increasing phase sequence and parent-state
name. The detection monitor uses this typed phase signal to request Verify
only during Exploration and match detections to the current goal.

One active goal runs the following nested sequence:

```text
Exploration
  TAKEOFF (initial entry only, unless skip_takeoff is true)
      -> SWEEP -> EXECUTE_PATH -> SWEEP -> EXECUTE_PATH -> ...
  confirmed smoke/fire -> LOITER handoff -> Verify
Verify
  APPROACH_SMOKE -> REACQUIRE -> CENTER_FIRE or SEARCH_FIRE
  source lost or verification complete -> Exploration/SWEEP
  blocked terrain recovery -> explicit hold in AUTO.LOITER
```

- `TAKEOFF` requires the vehicle to be disarmed initially unless the goal sets
  `skip_takeoff: true`, which requires an armed vehicle in `AUTO.LOITER` and
  skips this state. Only use that option when the vehicle is already airborne;
  the executive does not subscribe to landed-state telemetry. The takeoff state
  pulls the PX4 parameter catalogue through MAVROS, reads `MIS_TAKEOFF_ALT`, requests
  `AUTO.TAKEOFF`, arms, and waits for that climb relative to the initial PX4
  local height.
- `SWEEP` holds position and performs one full turn in six 60-degree steps,
  returning to the starting heading. `sweep_step_duration` is the yaw ramp
  duration for each step, 12 seconds by default (5 degrees/second). After each
  ramp, the executive waits for the measured position and heading to converge and
  remain within tolerance for `sweep_hold_duration` (3 seconds). This stable
  dwell gives the smoke detector time to register consecutive frames before
  the next rotation. Any position or heading drift outside tolerance restarts
  the dwell. The six ramps and dwells take at least 90 seconds, plus OFFBOARD
  warmup and final convergence. Both scan timings can be tuned in `mission.yaml`;
  path convergence continues to use `hold_duration` (0.5 seconds).
- `EXECUTE_PATH` requests one path from `plan_path`, splits it into three
  equal lengths along the path, and executes all three legs consecutively
  using MAVROS local-position setpoints. Split boundaries are interpolated;
  unequal waypoint spacing does not change where the thirds fall. The sweep
  runs before each path execution, and every `EXECUTE_PATH` requests a fresh
  plan from the current position. Each movement uses a yaw-only heading from
  `atan2(dy, dx)` in PX4 local coordinates. The drone turns in place and waits
  for heading convergence before moving forward along a segment; it keeps
  that heading during translation. Vertical moves and repeated points retain
  the previous heading. Planner pose orientations do not set the flight yaw.
  Turns use `path_turn_step_duration` per 60 degrees (5 seconds by default),
  independently of the slower scan timing.

Each sweep and path confirms `AUTO.LOITER` before the next state starts.
Successful path completion loops back to `SWEEP`, without another takeoff or
action request. A failed scan or path ends the goal instead of retrying.
After taking flight control, recovery for cancellation, parent-state requests,
or errors refreshes the hover target from current local odometry and requests `AUTO.LOITER` when
the vehicle is armed and still in `OFFBOARD` or `AUTO.TAKEOFF`. With stale
telemetry, it retains the last validated target. If the handoff fails while the
vehicle remains armed in OFFBOARD, the hold setpoint continues streaming and the
returned message includes the recovery failure. A request rejected because
the vehicle was already armed does not initiate recovery mode changes.
Recovery also preserves an observed pilot or failsafe flight-mode takeover.

### Verify: smoke guidance and visible fire inspection

Entering Verify stops Exploration and confirms `AUTO.LOITER`. Every translation,
including altitude centering and observation viewpoints, uses
`plan_inspection_path`: inflated-body obstacle checks, recent occupied-cell
history, home altitude bounds and terrain-envelope clearance remain mandatory.
The default terrain floor is 5 m; navigation prefers 7.5 m and considers up to
10 m clearance. The planner's configured unknown-air policy applies throughout
planning and in-flight corridor revalidation; terrain heights are never invented.

```text
APPROACH_SMOKE -> short terrain-aware smoke leg -> REACQUIRE
REACQUIRE     -> fresh smoke -> APPROACH_SMOKE or SEARCH_FIRE
SEARCH_FIRE   -> smoke-boundary observation / short terrain-aware leg
              -> boundary viewpoint inspections in CENTER_FIRE
any Verify state -> confirmed visible fire -> CENTER_FIRE
CENTER_FIRE   -> center -> bounded guarded fire approach -> fire_found report
              -> completed report -> Exploration
blocked recovery -> wait in LOITER for newer fresh maps -> bounded retry or explicit abort
```

In `CENTER_FIRE`, distance is measured to the observed smoke wall, never to
an estimated fire position. An opt-in geometry query samples 25 parallel rays
across a 3 m by 3 m patch in front of the camera. Occupied/free changes in recent
map history identify a suspected flickering wall; static or unknown cells alone
do not. The mean range is logged, while the nearest sampled range, measurement
uncertainty and tracking margin limit approach to `verify_smoke_standoff` (10 m).
A missing wall estimate causes a hold and observation retry, not an assumed range.

Fire centering attempts one bounded yaw correction; terrain planning selects
climbing height. A short fire-approach maneuver retains its selected heading
through detector gaps or tracking-ID changes. Each actual planned route is limited
to `verify_fire_leg_max_length` (12 m), with `verify_fire_leg_timeout` (60 s)
starting after OFFBOARD preparation. Terrain, wall clearance, vehicle health and
operator cancellation remain authoritative during motion. The original observed
wall boundary remains enforced through flicker, and newer closer returns tighten
it. The five-second detector wait starts after stopping and cannot cut a climb.
A new movement or successful arrival report requires fresh post-stop fire guidance.
The approach is bounded by 64 legs and the existing inspection deadline; a blocked
approach may still record visible fire from its held viewpoint without claiming
arrival at the wall. Set the step limit to zero for observation-only reporting.
Fire coordinates, when independently available, remain optional report metadata.
These movement changes apply only to `CENTER_FIRE`.
A confirmed fire can interrupt the first smoke leg. The controller stops in
LOITER and requires new post-stop thermal images before further corrections.

`APPROACH_SMOKE` remembers a smoothed smoke bearing and follows it in short
terrain-aware forward steps (`verify_navigation_step`, 3 m by default). It tries
straight 3 m, 1.5 m and 0.75 m candidates, preferring level motion before a
terrain-required climb. An upper-image smoke cue does not command a separate
altitude correction. There are no lateral/retreating fallbacks for smoke pursuit.
Each new Verify episode starts with the current smoke bearing; it does not
average a new site's direction with the previous site's bearing.

After each stop, smoke loss holds the current position and heading until
`verify_smoke_loss_grace` (5 wall-clock seconds by default) has elapsed since the
last accepted confirmation. Each new confirmed glimpse restarts that grace;
fresh, post-stop guidance resumes forward planning. A short confirmed glimpse
between negative frames can be consumed once while still within the normal
2-second image age limit. Bearing memory never makes an old image fresh or
invent a smoke range. A committed short validated leg can finish through a brief
detector gap; it cannot start another leg using stale guidance. Only sustained
loss permits a bounded yaw scan around the remembered bearing. Fire and mission
interrupts take priority throughout the hold.

The inconsistent smoke wall guides smoke-only approach. A fresh smoke ray's
observed, history-marked plume candidate within `verify_smoke_standoff` (10 m)
is a conservative boundary proxy, not a measured fire position. Current smoke
coverage in all four image regions and at least `verify_full_coverage` (0.25)
also stops forward smoke flight and starts observation. A static first-hit
obstacle alone cannot prove a smoke boundary. A stale, unknown or non-plume
surface cannot establish a negative inspection result.

`SEARCH_FIRE` holds for fresh thermal confirmation, then reassesses smoke and
terrain before selecting another short forward smoke leg. Collision-checked
bent paths may follow terrain contours. Fresh camera guidance updates yaw during
execution; a source image's correction is applied only once. Actual forward
translation must exceed position tolerance to count as progress. Lateral motion
or revisiting old viewpoints does not reset the smoke no-progress counter.
Fire approach retains its separate parallax candidates and localized standoff.

Sparse ground returns do not prevent a short forward leg under the default
occupied-voxel policy. The aircraft retains altitude when no ground height is
available; it never invents a ground height or descends on that basis. Native
planning and flight-time validation check the whole path against occupied body
clearance and the 5 m buffer above mapped terrain. The highest occupied voxel
under the footprint, including recent observations, sets that terrain buffer;
holes between lidar returns do not invalidate it. Observed rising terrain still
raises the destination. Optional strict observed-free mode continues to require
complete observed support. Missing or stale maps still block motion.

When current terrain clearance is below the floor, the native planner first
tries simple escape prefixes, then a bounded multi-segment search with level,
upward and diagonal connectors. Every sampled segment retains obstacle and terrain checks.
Until reaching the floor, height and measured terrain clearance cannot decrease,
and the escape stays inside a 2 m horizontal radius around its original anchor.
Once clearance is reached, ordinary terrain-aware OMPL planning can produce
bent paths. An occupied corridor or insufficient mapped terrain clearance can
still reject a route. `START_INVALID=4` means current body/support evidence is invalid;
`NO_CLEARANCE_ESCAPE=6` means start collision/terrain checks passed but the bounded
recovery search found no route. Missing/stale maps have a distinct error from
unknown/occupied body clearance and missing terrain. Recovery holds in LOITER
and polls for a strictly newer, valid source-stamped map before retrying
centering or translation. Each continuous map outage has a
`verify_navigation_map_timeout` budget (20 wall-clock seconds by default).
Receiving a newer valid map ends that outage. Healthy planning and OFFBOARD/LOITER
handoffs do not consume the next outage's budget. Persistent body/terrain
clearance failures still share one budget at the same position; new maps alone
cannot extend that clearance-recovery deadline. Expiry reports a blocked mission
and holds for the operator.
Geometry-service polls during recovery are bounded to 0.5 seconds; temporary
service timeouts remain recoverable within that budget, while telemetry and
flight-control faults still propagate.
Fresh fire evidence, cancellation and battery alerts retain priority throughout
recovery. Map source freshness remains 5 ROS seconds. With real time, accepted-map
reception silence is also limited to 5 wall seconds. With simulation time, a
progressing slow clock does not expire a source-fresh map merely because wall
time runs faster; a frozen simulation clock still blocks inspection after
5 wall seconds. These checks do not change OctoMap production or publication.

Planner receipt diagnostics include source age before/after decoding, binary
bytes, tree nodes and decode duration. Freshness failures distinguish old source
observations from reception silence, invalid/future stamps and clock mismatch.
Stale, duplicate and out-of-order stamped maps cannot replace a newer accepted
snapshot or refresh its receipt time. Recent occupied evidence remains blocking.
Pausing and resuming simulation does not erase observations still within their
source-age window.
History expires with the existing freshness window and is capped at eight maps.

`CENTER_FIRE` uses the bounded movement and smoke-wall policy described above.
Cached images cannot authorize another movement or a report. Cancellation,
battery Return/Abort requests, invalid telemetry and flight-control failures
remain authoritative.

Without visible fire, broad/nearby smoke permits up to three observation
windows, with safe lateral viewpoint changes between them. Rejected or
unfinished viewpoint movements do not count as completed negative inspections.
A negative report requires current broad smoke, fresh smoke-boundary evidence
and no confirmed fire during the inspection. Otherwise the result is
inconclusive. Previously confirmed fire cannot become a negative report during
a detector gap. Exhausted blocked recovery retains an explicit hold outcome.

The controller checks source timestamps, receive age, camera/track identity and
confirmation persistence. Producer confirmation metadata can establish fire
persistence immediately; messages lacking that metadata use the consumer's
same-track frame/time checks. A correction requires a newer source image before
the next correction or report. Feedback identifies acceptance/rejection reasons,
readiness, source-image age and terrain recovery failures. Up to 64 completed
smoke approach legs are allowed by default; sensor waits and map retries do not
consume that movement budget. Legacy `verify_step_min/max` and
`verify_localization_max_steps` parameters remain accepted for compatibility;
fire localization no longer gates flight or completion.

The retained `inspection_reports` topic publishes
`kv_executive/msg/InspectionReport`, separating observer pose/GPS from optional
smoke/fire target coordinates. A positive report can have
`fire_location_valid=false`; unknown coordinates remain NaN, never the UAV's
position relabeled as fire. Missing GPS invalidates only global coordinates.
The legacy `fire_sources` topic still reports the observation viewpoint when
fresh GPS is available. Its exploration exclusion circle is not a fire boundary.

`query_inspection_geometry` retains source-stamped camera rays for optional
multi-view estimation. A fresh nearby, history-associated plume wall can be
recorded with method `lidar_smoke_plume_proxy`; it is an observed wall estimate,
not a measured emitter location. Unassociated obstacles are not smoke positions.
Terrain heights select candidate altitudes; the full corridor must independently
pass the configured collision and terrain policy
before OFFBOARD, after warmup, during flight and on arrival. Revalidation pins
the original escape anchor and cannot restart the below-floor escape after it
has completed. Completed fire views are associated conservatively by observer
and bearing when target coordinates are unavailable; a new bearing, translated
observer or unavailable association permits another inspection.

### PX4 returning home during or after Exploration

PX4 normally records a home position when arming; that alone does not request
a return. PX4 monitors the setpoint stream while in OFFBOARD. If it stops before
the mode handoff is confirmed, PX4 can trigger its offboard-loss failsafe after
`COM_OF_LOSS_T`; the configured action can be Return or Land. Reaching home and
landing can therefore be a failsafe response, rather than a command from this
mission. Stopping local setpoints after PX4 has confirmed `AUTO.LOITER` does
not itself trigger offboard loss.

Normal state handoffs and successful recovery confirm `AUTO.LOITER` before
stopping setpoints, so offboard loss alone does not establish the cause of a
return after a handoff. The x500 SITL configuration also defaults
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
Stock MAVROS labels this local-position stream `map` even though its numbers
are still expressed relative to the PX4 EKF local origin. The executive and
battery monitor therefore accept `local_odometry_frame_aliases: ['map']` by
default. At their configured MAVROS local-odometry input only, a recognized
alias is relabelled to `local_frame` without changing the position or
orientation values. This is a naming compatibility rule for that input; the
planner's actual `map` coordinates still require the body-pose alignment
described above. No identity TF is added between the planner map and PX4 local
coordinates.

Unknown or empty incoming frame labels still fail validation. Set
`local_odometry_frame_aliases: [px4_odom]` in both the executive and battery monitor
parameter files to require the exact `local_frame` label. If a different
odometry topic is configured, list an alias only when its numeric coordinates
are known to use the PX4 local origin and ENU convention.

`robot_base_frame` defaults to `<drone_name>/base_link` and can be overridden
as a launch argument. TF must connect the path frame to that body frame at
the MAVROS odometry timestamp. The mission stops if telemetry or the required
transform cannot be obtained within the configured timeouts.

To diagnose a local-odometry label mismatch before sending a goal:

```bash
ros2 topic echo /mavros/local_position/odom --once --field header
ros2 param get /mission_executive local_frame
ros2 param get /mission_executive local_odometry_frame_aliases
ros2 param get /battery_monitor local_odometry_frame_aliases
```

The raw topic may report `px4_odom` with this package's MAVROS launch or `map`
with the stock MAVROS launch. Earlier simulated-FCU tests published only the
explicit `px4_odom` label, so they did not exercise the stock MAVROS label that
caused the TAKEOFF rejection. Regression coverage now includes both labels
and rejection of unrelated frames.

## Smoke and fire monitoring

`detection_monitor_node.py` listens to the detectors' typed
`kv_perception/msg/SmokeDetection` and `kv_perception/msg/FireDetection`
messages. Either detector can interrupt an active Exploration goal: smoke
must have `detected=true`, `status=CONFIRMED` and `background_ready=true`;
fire must have `detected=true` and `status=CONFIRMED`. The monitor uses the
detectors' existing persistence checks and adds no second confirmation delay.
Here, vicinity means a source visible in the camera; these messages do not
provide a reliable distance in metres.

The monitor follows the executing goal UUID on
`execute_mission/_action/status` and the typed parent phase on `mission_phase`.
It publishes `kv_executive/msg/MissionStateRequest` with `state: Verify` and
that goal's `goal_id` on `mission_state_request_for_goal` once per Exploration
entry. A new action goal and fresh observation rearm the monitor.
It sends no requests outside Exploration, while idle, or while the action is
canceling or terminal.
The executive checks the requested UUID against its active goal and latches
the transition atomically under the goal lock. A delayed request from a
previous goal cannot interrupt a later goal. The existing
`mission_state_request` String topic remains available for manual requests.
Source image header stamps must be nonzero, newer than both the goal's
acceptance stamp and current Exploration entry, and fresh against the
monitor's ROS clock: no more than 2 seconds old or
0.25 seconds in the future by default. Duplicate source frames do not cause
additional requests. A later goal requires a fresh observation from that
goal, so an observation from a previous mission cannot stop a new one. A
monitor joining an executing goal waits for its matching phase and a qualifying
detector observation. The request interrupts Exploration, requests `AUTO.LOITER`,
and enters Verify within the active action. Once Verify completes or loses its
source, Exploration resumes with a new sweep. The executive performs the flight
commands; the monitor only publishes the goal-scoped transition request. It
does not subscribe to odometry or fire-source reports, and applies no
known-view suppression.

`monitoring.launch.py` starts this monitor and the battery monitor by default.
To run detection monitoring alone, disable the battery monitor there or use
the existing individual launch instead:

```bash
ros2 launch kv_executive detection_monitor.launch.py drone_name:=kv_drone
```

The combined and individual monitor launches default `use_sim_time` to `true`;
use the same clock setting and `drone_name` as the executive and detectors,
and set `use_sim_time:=false` on a real vehicle. Both accept
`detection_monitor_config` for a custom
`config/detection_monitor.yaml`, `smoke_detection_topic`, and
`fire_detection_topic`. Detector inputs default to
`/<drone_name>/smoke/detection` and `/<drone_name>/fire/detection`; the monitor
stays in the executive namespace, with relative action-status and request
topics. For a separately namespaced executive, both monitor launches also
accept `action_status_topic` and `mission_state_request_topic` overrides.
Topic launch arguments override the corresponding YAML values. Run one
monitor for each executive, alongside the matching detector inputs.
Pass detector topic overrides to both autonomy and monitoring, and configure
the matching perception outputs. Autonomy also accepts `verify_planning_frame`
and `global_position_topic` for Verify planning and fire-source reporting.

## Battery usage and return estimate

The combined `monitoring.launch.py` starts the battery monitor alongside the
detection monitor. To run battery monitoring alone, disable detection
monitoring there or use the existing individual launch instead:

```bash
ros2 launch kv_executive battery_monitor.launch.py use_sim_time:=true
```

Both monitor launches default `use_sim_time` to `true`; use the same clock
setting as autonomy and `false` on a real vehicle. Both accept
`battery_monitor_config:=/absolute/path/battery_monitor.yaml` to override the
monitor's parameters. Start MAVROS first and wait for telemetry, then start
autonomy and monitoring before sending the first mission goal. This lets the
monitor observe the grounded home pose, battery baseline, and all
mission-status transitions. Run only one instance of each monitor.
Keep `auto_start_mission:=false` (the default) for the staged startup sequence;
if enabling automatic missions, start monitoring before autonomy.

Separate launching provides startup control and makes failures easier to
isolate; it is not a confirmed fix for the native segmentation faults.

The production monitor is the C++ `battery_monitor_node` executable, using
`rclcpp` and a single-threaded executor. Both launch files use it. The legacy
`battery_monitor_node.py` command forwards all arguments to that executable
before importing any Python ROS libraries. The Python model and monitor
modules remain as a reference implementation for regression tests. This
avoids the Python executor path observed in the monitor's repeated native
crashes; the underlying runtime fault has not been established.

The monitor reads `sensor_msgs/BatteryState` from `/mavros/battery`. MAVROS
provides remaining charge in `percentage` as a fraction from 0 to 1; the monitor
converts this to 0–100 percentage points. For example, a change from 0.80 to
0.77 consumes 3 percentage points. Unknown, nonfinite, or out-of-range readings
cannot establish a usable baseline or return estimate. Voltage is not used.

### Simulated battery depletion

The PX4 checkout's battery simulator defaults `SIM_BAT_MIN_PCT` to 50 and
`SIM_BAT_DRAIN` to 60 seconds. It stops reducing the reported battery at 50%.
The slow sweep can reach this floor before `EXECUTE_PATH`, leaving no measured
charge drop for the monitor to learn from. A `0.00` consumption record means
the reported percentage did not decrease; it does not establish zero energy
use. Such records are stored but excluded from the return model.

For a dedicated SITL battery-monitor test, set a longer discharge time and
remove the simulated charge floor in the PX4 console before takeoff, for example:

```text
param set SIM_BAT_DRAIN 1800
param set SIM_BAT_MIN_PCT 0
```

These are simulator settings, not changes to the monitor's arithmetic or a
fix for native crashes. PX4's existing low-battery failsafes still apply.
Use a fresh disarmed simulation run when establishing a new training baseline.

### Recording path usage

`TAKEOFF: done` on `mission_status` establishes the post-takeoff battery
baseline. Measurement starts only at `EXECUTE_PATH: started`, the Exploration
child state that flies the planned random-walk path, and ends when that state finishes or
the mission terminates. The interval's battery baseline is refreshed at its
start, so the intervening scan does not enter the path's measured consumption.
Run the monitor before starting the mission: progress-only retained status
messages cannot reconstruct a path start missed while the node was down.
If fresh battery, odometry, or vehicle state is missing at that start, the node
skips learning for that interval rather than inventing its starting battery.
A mission with `skip_takeoff: true` establishes its baseline when its first
`EXECUTE_PATH` starts. Each path interval is stored in the node's
in-memory dictionary (a C++ map keyed by sample ID) and published on
`battery_usage` as `kv_executive/msg/BatteryUsage`, with start/end poses, accumulated 3D
travel distance, start/end battery percentage, consumption, and outcome. These
records cover repeated Exploration cycles and survive new goals and disarms
while the node runs; they are not
saved across node restarts. Restart the node when changing the battery pack or
vehicle to clear the learned model.

Usable completed intervals must have fresh telemetry, at least
`minimum_sample_distance` (1 m by default), and positive depletion. Their
average cost is distance-weighted:

```text
mean_percent_per_meter = sum(consumed_percent) / sum(distance_m)
return_required_percent = mean_percent_per_meter * straight_line_home_distance_m
buffered_required_percent = return_required_percent * return_safety_factor + reserve_percent
```

The current path interval also contributes once it has sufficient
distance and measurable depletion, including before the first completed
interval. `sample_count` counts usable completed records plus the usable active
interval. Intervals with telemetry gaps or rising charge are not learned.
`battery_return_estimate` publishes throughout armed flight, including sweeps,
as `kv_executive/msg/BatteryReturnEstimate` at `monitor_rate` (2 Hz), including
current/home poses, remaining percentage, straight-line 3D distance, estimated
return cost, buffered cost, margin, availability, and alert status. Check
`estimate_available` before using the estimate fields. The monitor suppresses
estimates when battery, local odometry, or connected vehicle state is stale;
default receipt-age limits are 5, 2, and 10 seconds respectively.

The executive subscribes to `battery_return_estimate` through its configurable
`battery_alert_topic` parameter. A fresh valid message with `return_alert=true`
aborts the active mission, including Verify, and requests `AUTO.LOITER`.
`battery_alert_timeout` defaults to 5 seconds; stale, invalid, or pre-goal
messages cannot abort a new mission. With a learned model, the alert uses the
buffered return cost. Before the model is available (`estimate_available=false`),
the monitor can still alert when remaining charge reaches its configured
reserve. Both alerts stop the mission; the executive adds no separate raw
battery threshold. It does not initiate Return flight. Once recovery finishes,
send a new action goal explicitly to resume; continued alerts can stop that
goal too.

Home uses the latest valid local pose received while disarmed and freezes that
pose at `TAKEOFF: started`, or when the vehicle arms for externally initiated
takeoff. Start the monitor before takeoff when possible. If
it starts airborne, it can use `/mavros/home_position/home` instead: MAVROS's
`HomePosition.position` is numeric local ENU, in the same estimator coordinates
as `/mavros/local_position/odom`. Its subscription accepts retained MAVROS home
messages for airborne startup. Home and distance calculations use that PX4
local frame, not the planner's external-odometry home.

The default alert threshold adds a 30% multiplier to the estimated return cost
and a reserve of 15 percentage points. When remaining battery reaches or falls
below that threshold during armed flight, an `INFO` terminal message reports
the remaining charge, home distance, estimated cost, buffer, and margin. Alerts
repeat at most every `alert_repeat_period` (10 seconds). This node issues no
flight commands. A straight-line consumption estimate does not account for
obstacles, wind, or a separate landing-energy model. Distance integrates raw
odometry, so estimator jitter and resets that retain the same frame name can
distort the learned rate. Use the configurable reserve and factor when
validating the estimate on the vehicle.

```bash
ros2 topic echo /battery_usage
ros2 topic echo /battery_return_estimate
```

## Mission tests

After building and sourcing the workspace, run the mission tests with:

```bash
colcon test --packages-select kv_executive --ctest-args -R '^(mission_|verification$|verify_ros$|battery_|detection_monitor|planner_inputs$)'
colcon test-result --verbose
```

These cover geometry, state sequencing, failure handling, and the public ROS
action using an isolated simulated FCU. The action test checks feedback,
cancellation, terminal results, repeating Exploration, repeated goals, and
independent setpoint publication during mode-service waits. A simulated OFFBOARD-loss watchdog
checks the LOITER handoff and deliberate publisher loss; path tests check
forward-facing yaw after frame conversion and turns at corners. State updates
run at 1 Hz independently of odometry; tests exercise heartbeat gaps, expired
state, stale odometry, and an explicit disconnect. Planner tests use the real
C++ services to check missing inputs, planning with a retained map, explicit
target validation and transforms, exact endpoint/heading preservation, and
reported-fire Exploration goal exclusion. Battery tests cover depletion calculations, path-only
recording, baseline and home handling, telemetry validity, and buffered return
alerts. These tests do not replace SITL or vehicle flight validation.

Detection monitor tests cover either detector independently, confirmed-status
gating, stale/future/zero source stamps, goal and phase boundaries, duplicate
observations, idle suppression and rearming on each
Exploration entry. ROS integration connects typed detector messages to the
real monitor and an active mission. Active-mission tests check transitions into
Verify, LOITER recovery, resuming Exploration, battery aborts and subsequent
goals. Verify controller/navigation unit tests cover bounded branches,
perception guards, waypoint preservation and reporting validation.

The native monitor DDS test checks the installed C++ executable with zero and
positive consumption, weighted learning, frozen home, invalid telemetry and
the buffered alert. The combined action test exercises repeated Exploration
cycles with constant 50% battery and cancellation during a sweep; it also
exercises the legacy executable's forwarding to the native monitor.

For Python nodes exiting with `-11` (SIGSEGV), see the
[native crash investigation](docs/native_crashes.md). The Python entrypoints
emit all-thread fault traces on a native crash, and the action tests include
successive airborne goals using simulated time and separate action clients.

Autonomy and monitoring always run their Python nodes through the installed
same-version shared Python runtime. No launch argument is required:

```bash
ros2 launch kv_executive autonomy.launch.py
ros2 launch kv_executive monitoring.launch.py
```

Keep your usual launch arguments and remove the former
`shared_python_runtime:=true/false` argument from saved commands. The executive
and detection monitor print `[kv_shared_python]` at startup. The launcher uses
the installed Python shared library, retaining the Python version, environment,
ROS interfaces, signal handling and mission sequence. Matching debugger launch
prefixes are composed ahead of this launcher. Planner and battery processes
remain native executables.

This runtime became the normal launch path after recurring faults with Ubuntu
Python 3.12.3 and ROS Jazzy on WSL2 kernel
`6.6.87.2-microsoft-standard-WSL2`. The investigation reproduced the fault in
plain ROS workers without mission code. It did not establish that all WSL
versions, or WSL itself, cause the problem. See the
[October 27 investigation](docs/20261027_reacquire_segfault.md).

On an environment where ordinary Python/rclpy nodes run reliably, including
an unaffected native Linux or WSL installation, this extra launcher is not
needed for mission functionality. A deployment-specific launch file may omit
`prefix=SharedPythonPrefix(...)` and start the same Python executable with the
same parameters. The distributed launch files retain the shared runtime for
consistent behavior; they no longer expose a runtime rollback flag. Do not
choose a different path solely from the operating-system label: validate the
actual ROS/Python environment. Building the provided launcher requires the
configured Python version's development/embedding library (`python3-dev`).

When WSL updates require administrator approval, a workspace-local CycloneDDS
runtime is available without `sudo` or a rebuild. See the
[CycloneDDS setup instructions](docs/native_crashes.md#cyclonedds-alternative-when-wsl-updates-need-approval)
and select it in every ROS terminal, including the simulator and action CLI.
The earlier bounded CycloneDDS controls passed; the full Gazebo graph still
needs validation with this selection.

The installed runtime probe can reproduce executor failures using only plain
ROS workers and standard messages, with no simulation or mission goals:

```bash
ros2 run kv_executive diagnose_ros_executor.py --duration 180
```

It uses a unique namespace and isolated ROS domain, reports its output directory,
and saves worker fault traces and a JSON summary. It inherits
`RMW_IMPLEMENTATION` for controlled middleware comparisons. See the investigation
document for the latest results and their limits.

## External odometry and PX4

There are two different coordinate origins in this system:

- PX4 EKF local position normally starts close to `(0, 0, 0)`.
- Camera/VIO/LIO odometry may use a world-aligned origin such as
  `kv_drone/odom`, which is offset from the PX4 origin.

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
  drone_name:=kv_drone
```

`drone_name` derives the topic `/<drone_name>/odometry` and the frames
`<drone_name>/odom` and `<drone_name>/base_link`. The three explicit odometry
arguments remain available when a real estimator uses another convention.

The launch file relays the external `nav_msgs/Odometry` topic to
`/mavros/odometry/out` without changing its contents. In MAVROS terminology,
`out` means data sent to the
flight controller; `/mavros/odometry/in` is odometry received from it.
MAVROS converts ROS ENU/FLU conventions to MAVLink `ODOMETRY`, and PX4
receives it as `vehicle_visual_odometry`.

MAVROS 2.16 creates plugin nodes under its own `mavros` UAS node and ignores
process-wide plugin topic remappings. The container therefore starts in the
root namespace, and the relay keeps forwarding odometry after the parameter
configurator exits. Leave `namespace` empty for the usual `/mavros` interface;
`namespace:=drone1` puts MAVROS under `/drone1/mavros`. Passing
`namespace:=mavros` would create `/mavros/mavros` and requires consumers to use
that prefix too. When the external topic already is the MAVROS input, the
launch file omits the relay to avoid a forwarding loop; a publisher feeding
MAVROS directly must use reliable QoS.

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
  drone_name:=kv_drone
```

Or both can be started together:

```bash
ros2 launch kv_executive autonomy.launch.py \
  start_mavros:=true \
  drone_name:=kv_drone
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
ros2 topic hz /kv_drone/odometry
ros2 topic echo /kv_drone/odometry --once
ros2 topic echo /mavros/local_position/odom --once
ros2 run tf2_ros tf2_echo map kv_drone/odom
ros2 run tf2_ros tf2_echo kv_drone/odom_ned kv_drone/odom
ros2 run tf2_ros tf2_echo kv_drone/base_link_frd kv_drone/base_link
```

`/mavros/local_position/odom` is deliberately labelled `px4_odom` by this
launch file because it has the PX4 EKF origin, not necessarily the external
`map` origin. A separately started stock MAVROS instance may instead label
the same PX4-local stream `map`; the executive and battery monitor normalize
that configured input alias as described in the frame contract above.

The mission executive performs this origin alignment before publishing to
`/mavros/setpoint_position/local`, as described above. External-vision fusion
alone does not make planner coordinates valid PX4 local setpoints.
