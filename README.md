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
The default terrain floor is 3 m; navigation considers higher clearance up to
10 m clearance. The planner's configured unknown-air policy applies throughout
planning and in-flight corridor revalidation; terrain heights are never invented.

```text
APPROACH_SMOKE -> optional wall probe -> short terrain-aware smoke leg -> REACQUIRE
REACQUIRE     -> fresh guidance -> APPROACH_SMOKE or near-wall inspection
two fresh qualifying near-wall observations -> CENTER_FIRE with fresh fire
                                           -> SEARCH_FIRE without fresh fire
SEARCH_FIRE   -> bounded near-wall inspection in CENTER_FIRE
fresh farther wall -> APPROACH_SMOKE
CENTER_FIRE   -> center -> measured smoke-wall standoff -> fire_found report
              -> completed near-wall search without fire -> fire_not_found report
              -> completed report -> Exploration
blocked recovery -> wait in LOITER for newer fresh maps -> bounded retry or explicit abort
```

Distance is measured to the observed smoke wall, never to an estimated fire
position. Production geometry queries sample 49 parallel rays across a 4.5 m by
4.5 m patch perpendicular to the measurement direction: seven rows and seven
columns, spaced 0.75 m apart. Current returns must survive terrain rejection and
camera association when image bounds are available. Two evidence routes exist:

- Flicker-supported: at least three current returns across three rows and three
  columns, including at least one hit with both occupied and explicitly free
  observations in recent map history.
- Persistent-return: at least six current returns across three rows and three
  columns, without requiring occupied/free changes. Fresh confirmed RGB bounds,
  enabled terrain rejection and an older fresh same-frame map are mandatory.
  The earlier map supplies terrain-classification context; it does not count as
  the controller's second independent wall observation.

Neither route can replay an old hit when the latest map no longer supports it.
Both produce candidates requiring independent reconfirmation. This remains a
suspected smoke boundary, not proof of the physical object's identity. A vertical
background object inside the smoke rectangle can still mimic persistent smoke.
After terrain/image filtering, extended returns are partitioned by depth using
`inspection_wall_cluster_span` (4 m by default). Each group must independently
satisfy its evidence route and three-row/three-column coverage. The nearest
qualifying group, ranked by median depth, supplies the mean, nearest distance and
uncertainty. Separated near/far surfaces cannot borrow support from one another;
groups cannot grow through a chain of small gaps. All occupied cells remain in
collision checking. An isolated nearer return within the selected depth span
still contributes to its conservative nearest distance. Logs show total support,
selected-group support, voxel uncertainty and depth spread separately.
Legacy service requests retain the 25-ray, 3 m mixed-history-only measurement for
compatibility and historical benchmark comparisons.
Throttled diagnostics report current occupied hits, changing-occupancy hits,
qualifying rows/columns, range uncertainty and unavailable query results, so a
missing measurement can be distinguished from insufficient ray coverage.
The mean range is logged, while the nearest sampled range, measurement
uncertainty and tracking margin limit approach to `verify_smoke_standoff` (15 m).
A positive report's conservative distance (nearest range minus uncertainty) must
be inside the configured inspection band, 15–20 m by default. The maximum
accepted uncertainty remains 2 m.

`APPROACH_SMOKE` probes along the next approach bearing. `SEARCH_FIRE` revalidates
the nearby wall before entering inspection. Trusted wall observations first
require physical settling in LOITER: at least three distinct fresh odometry
samples and one continuous wall second below 0.10 m/s translation and 3 degrees/s
rotation. Pose-derived rates include roll/pitch; stable nonzero tilt is allowed.
Repeated/stale data, frozen clocks and a LOITER acknowledgement alone cannot
certify rest. An odometry observation gap of one dwell or more requires a new
settling proof. Settling is bounded by `verify_settle_timeout` (15 s), with a
separate bounded post-settle map wait (15 s). Current body-clearance checks run
during that map wait. A missing optional map update leaves wall range unavailable
while physically settled distant approach can still proceed safely.

Camera observations and the map used for qualification must have source stamps
after the settled viewpoint. A newer fused map can still retain older cells;
its timestamp does not establish that every return was reobserved. Motion or a
turn invalidates the viewpoint, and flight validators never wait for settling.
Retained qualified planes and ordinary map collision checks continue protecting
committed movements. `verify_wall_probe_timeout` limits
the service request to 0.5 seconds by default. Missing returns, insufficient samples and a
probe timeout leave the ordinary checked forward approach available, including
when smoke lies beyond the 40 m measurement range. They do not increment a
range-failure counter or start the inspection clock. Explicit stale-map errors
still invoke map recovery, and ordinary terrain/corridor checks remain mandatory.
In `CENTER_FIRE`, missing wall evidence still prevents distance-based completion;
it does not prevent a separately validated height correction.

A single far candidate can shorten its selected leg, but cannot be projected
through later legs to declare arrival. Two near observations from distinct
source-stamped maps confirm the inspection transition. A first pending vote
survives missing probes inside its original 20-second recheck window, so
valid → missing → fresh valid can confirm arrival. Missing probes never provide
live range or extend the deadline. An expired window remains an explicit
incomplete-inspection abort; a late observation cannot revive it. Repeated queries
on one map do not count twice. Each observation must satisfy the strengthened
latest-map qualification; a new header cannot substitute for current occupied
support. The two distances need not agree with a stationary plane or with the
drone's movement: smoke may move in either direction.

A first near observation withholds further advance for confirmation. Missing
near-wall evidence invokes bounded `REACQUIRE`, with
`verify_wall_recheck_timeout` limiting the episode to 20 wall seconds. Missing
smoke also routes to `REACQUIRE`, avoiding SEARCH/CENTER transition churn.
A fresh qualifying farther measurement resets near confirmation and permits
ordinary checked approach again, including when smoke moves away after arrival.
The inspection clock is not restarted by these transitions. A closer observation
withholds forward motion immediately, without waiting for a second reading;
two independently observed close readings are required for a checked retreat.
The viewpoint keeps its heading while a first near vote awaits confirmation.
After confirmed retreat, the new 20-second recheck starts after physical
settling, rather than while the vehicle is braking. The overall 600-second
inspection clock continues through alignment, settling and observation waits.
Publication still remeasures the boundary: cached observations cannot authorize
a completed report, and no fire report is inferred from proximity alone.

Fire centering attempts one bounded yaw correction. Fresh confirmed thermal
guidance also biases the next goal upward or downward by at most
`verify_vertical_step` (0.5 m), with observed terrain clearance taking priority.
The height decision is independent of wall ranging and is used when selecting
the next approach/inspection movement. A confirmed fire below the image centre
requests descent; one above requests climb. Committed paths still finish through
detector gaps rather than changing their position targets during execution.
Vertical guidance activates outside 7 degrees and releases inside 5 degrees by
default (`verify_vertical_tolerance` plus `verify_vertical_hysteresis`). During
approach, upward and downward guidance use the same selection order: first try a
combined forward/height leg, shortening forward travel for a large vertical image
error; if no such route is available, try one height-only correction of up to
0.5 m; if that is also unavailable, try a checked terrain-following forward leg.
One height-only attempt is permitted from each horizontal viewpoint. A successful
height-only correction must be followed by a fresh approach decision before any
further correction, and useful forward progress renews the height-only allowance.
Fire or smoke must provide fresh heading guidance before any forward translation.
The opportunistic wall probe remains part of approach decisions even while fire
height guidance is active; missing range alone does not prevent approach.
At the wall standoff, up to `verify_vertical_max_corrections` (3) corrections are
best effort: perfect image centering is not an additional report requirement.
Vertical corrections consume the existing fire-leg and inspection-time budgets.
Missing terrain evidence or insufficient clearance prevents descent. A useful
partial correction is allowed when the full request cannot fit above the terrain
floor. Requests smaller than `verify_vertical_minimum_change` (0.10 m) are skipped
before flight. Height-guided goals additionally converge within
`verify_vertical_position_tolerance` (0.05 m), independently of the ordinary
0.30 m position tolerance, with corridor checks continuing during convergence.
The terrain-following fallback retains altitude where possible and climbs as
terrain clearance requires. Height guidance is reconsidered after useful forward
progress, including where terrain drops away. A blocked correction never relaxes
the smoke-wall boundary or authorizes forward travel at the protected standoff.
Each completed correction requires another post-stop observation before another
correction; completed reporting additionally requires fresh wall evidence.

A confirmed fire seen during motion can supply one remembered height preference
after arrival if it disappears above or below the image centre. With fresh smoke
heading guidance, the same combined-first selection order applies. Without fresh
heading guidance, a remembered hint can supply only a bounded height-only search.
The normal 7-degree activation threshold applies. Forward travel shortens as
the vertical image error grows; the final-centering edge threshold does not
force height-only movement during approach.
`verify_vertical_memory_timeout` defaults to 15 source/ROS seconds. A short leg
lasting 30–150 wall seconds in slow simulation can therefore retain usable
direction. This timeout is independent of the post-arrival detector wait.
`verify_vertical_clock_timeout` separately rejects a simulated clock that stops
advancing for more than 5 wall seconds. Clock rewind also invalidates old hints.
Movement since capture must remain within one smoke step plus position tolerance
(3.3 m by default), yaw change must remain within 15 degrees, and target identity
must remain consistent. An unused hint is discarded before another forward leg.

Eligibility is evaluated before arrival scanning or further advance. A rejected
hint logs its exact reason, source/wall ages, angle, travel, rotation and clock
silence. Temporary ineligibility does not consume the hint. Selecting the single
checked correction consumes it, even if terrain or planning blocks execution;
the result distinguishes selection, rejection, and actual height movement.
Unavailable optional candidates permit the next checked alternative above;
map, vehicle, cancellation and actual execution failures retain their existing
recovery/error handling. The fallback does not restart inspection or blocked-route budgets.
Another correction needs new confirmation. Remembered direction never qualifies
as fresh fire for reporting. Detector diagnostics separately retain raw producer
confirmation progress, including `CONFIRMING` messages that cannot steer.

Slow-clock regressions use real 0.1× and 0.2× ROS clocks and verify actual 0.5 m
descent and climb after fire disappears during a long approach leg.
The 0.2× terrain-floor regression also verifies a refused remembered descent,
a completed level forward fallback, and a newly permitted 0.5 m descent after
crossing a terrain drop, all within the same mission goal. Symmetrical fallback
cases check combined movement, one height-only attempt, then terrain-following
forward movement, including persistent fire near the image edge.
Deterministic cases additionally cover 30-, 60-, and 150-wall-second legs,
frozen/rewound clocks, discarded hints, and blocked/no-op descent attempts.
The executive checkpoint immediately before this timing fix is `b2e8d58`.
The baseline before terrain-limited descent fallback is tagged
`checkpoint-before-height-fallback-20261101` in each `kv_` package repository.
The baseline before symmetrical combined-first selection is tagged
`checkpoint-before-symmetric-fire-guidance` in each `kv_` package repository.

A short fire-approach maneuver retains its selected heading
through detector gaps or tracking-ID changes. Each actual planned route is limited
to `verify_fire_leg_max_length` (12 m), with `verify_fire_leg_timeout` (60 s)
starting after OFFBOARD preparation. Terrain, wall clearance, vehicle health and
operator cancellation remain authoritative during motion. The original observed
wall boundary remains enforced through flicker; fresh measurements after the next
settled stop can tighten it. The five-second detector wait starts after stopping
and cannot cut a climb.
Ordinary fire-guided legs and successful arrival reports require fresh post-stop
fire guidance; the one-shot edge-recovery search is the bounded exception above.
The approach is bounded by 64 legs and a 600-second real-time inspection deadline.
A positive report requires fresh confirmed fire and a fresh wall measurement in
the configured 15–20 m inspection band, checked again before publication. The
distance used is the nearest qualifying return minus its uncertainty; the
maximum accepted uncertainty remains 2 m. Blocked movement,
step exhaustion and timeout do not authorize success. A zero step limit permits
reporting only when already at the measured standoff. Fire coordinates, when
independently available, remain optional report metadata. The inspection timer
starts with confirmed-fire inspection or the first accepted arrival at a measured
smoke boundary, including arrival in `APPROACH_SMOKE`. A far-away wall measurement
or broad image coverage alone does not start it.

`APPROACH_SMOKE` remembers a smoothed smoke bearing and follows it in short
terrain-aware forward steps (`verify_navigation_step`, 3 m by default). It tries
straight 3 m, 1.5 m and 0.75 m candidates and bounded forward-oblique candidates
to both sides. Level destinations across the fan precede terrain-required climbs,
so a high straight-ahead terrain estimate cannot preempt a lower side route.
Every route still requires full corridor validation. Ordinary routes also limit
the entire path to 1 m of rise, 1 m of drop, 0.3 m of backward travel and 12 m of
length, relative to the leg's starting pose and selected bearing. Intermediate
vertices and actual progress during execution are checked, so a forward endpoint
cannot hide a backward or high detour. Rejected candidates leave shorter and
side alternatives available. A measured current terrain-floor violation or a
wall confirmed too close on two fresh maps permits bounded safety recovery
(6 m rise, 1 m drop, 3.3 m backward travel); collision and terrain checks still
apply. Logs give route extrema, length and the recovery reason.
An upper-image smoke cue
does not command a separate altitude correction. Ordinary pursuit retains
positive forward progress; the separate checked retreat applies only inside a
measured boundary.
Each new Verify episode starts with the current smoke bearing; it does not
average a new site's direction with the previous site's bearing. Once a smoke leg
is selected, fire appearing or disappearing and broad smoke coverage do not cancel
its planning or execution. Fresh confirmed fire has priority for bounded live yaw
corrections, followed by a two-source-second fire-heading grace period before
smoke can take over. The five-wall-second clock watchdog expires memory only
when the source clock stops progressing; slow progressing simulation retains
its source-time grace. A camera ray is transformed at image acquisition time,
including the camera mount, and yields an absolute bearing/body-yaw target.
Body yaw is solved to center the image horizontally, including the effect of
camera roll on a fire above or below the image center. A ray that cannot be
centered by yaw alone leaves the correction unavailable.
Successfully consumed images are cached across source switches and legs, so an
old relative error cannot be applied again. Commands retain a 5-degree lead cap
and a 12-degree/source-second slew limit; the total turn can exceed 5 degrees.
Stopped fire alignment uses its own 2-degree image/yaw tolerance and a bounded
15-second alignment budget. Any turn requires a new settling proof before wall
qualification. Heading ownership changes and actual/desired/commanded yaw are
logged. Gaps without a usable source retain the last heading. The planned position
path remains unchanged. Terrain, obstacle, map, telemetry and mission-interrupt
checks remain active. Arrival enters `REACQUIRE` and requires new post-stop images;
confirmed fire guides heading and altitude during distant approach. Final
`CENTER_FIRE` inspection requires the two fresh near-wall observations above.
Fire seen during a smoke leg therefore affects a subsequent movement decision,
including the one-shot edge reacquisition described above, never the XYZ path
currently being executed. Fresh fire selects `CENTER_FIRE` once measured
near-wall confirmation is established.

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

The inconsistent smoke wall guides smoke-only approach. A single plume-ray hit
may supply obstacle or location metadata, but does not satisfy the three-ray
qualification or authorize arrival. Coverage in all
four image regions is observation information: it neither stops smoke navigation
nor independently starts timed inspection. A static first-hit obstacle alone
cannot prove a smoke boundary. A stale, unknown or non-plume
surface cannot establish a negative inspection result.
Production aperture probes reject an inclined plane supported by earlier maps
before assembling either evidence route. This rejects stable sloping terrain whose
voxel occupancy jitters, while leaving its occupied cells in the collision map.
The geometry is a heuristic: persistent inclined smoke can resemble terrain and
be rejected. Confirmed RGB smoke detections also publish an angular rectangle;
when available, qualifying hits must project into that fresh camera region.
Invalid or stale explicit image bounds cannot silently bypass association.
Legacy producers without bounds retain terrain-guarded probing, without claiming
camera association. An image rectangle is not proof of smoke at a specific depth.

One close observation holds position for reconfirmation. Wall-based retreat
requires two fresh close observations below 15 m; two prior observations in the
inspection band do not authorize retreat on one later close glitch. Two fresh
observations at or within 20 m confirm proximity; inspection requires the current
measurement to be in the 15–20 m band. This permits `CENTER_FIRE` with confirmed
fire or `SEARCH_FIRE` otherwise. Fresh evidence beyond 20 m resumes approach. The
20-second stationary recheck window starts again after a required retreat
finishes; that movement does not consume its reconfirmation time. The overall
600-second inspection deadline keeps its existing start and is never reset by
these state transitions or retreats.
When a qualifying aperture measurement is available outside the boundary, its
remaining measured clearance limits the selected smoke leg and validated path.
Without qualifying evidence, a short
smoke-guided leg cannot guarantee a precise wall distance.

`SEARCH_FIRE` revalidates current guidance and wall confirmation. A farther
measurement returns to `APPROACH_SMOKE`; missing guidance or near-wall evidence
enters bounded `REACQUIRE`. Only current confirmed proximity permits
`CENTER_FIRE` inspection. Collision-checked approach paths may follow terrain
contours. Fresh camera guidance updates yaw during
execution; a source image's correction is applied only once. Actual forward
translation must exceed position tolerance to count as progress. Lateral motion
or revisiting old viewpoints does not reset the smoke no-progress counter.
Fire approach retains its separate parallax candidates and measured smoke-wall standoff.

Sparse ground returns do not prevent a short forward leg under the default
occupied-voxel policy. The aircraft retains altitude when no ground height is
available; it never invents a ground height or descends on that basis. Native
planning and flight-time validation check the whole path against occupied body
clearance and the 3 m buffer above mapped terrain. The highest occupied voxel
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
Cancellation and battery alerts retain priority throughout recovery. Confirmed
fire resumes handling only after a newer fresh map ends a map outage; fire cannot
restart the 20-second wait. Map source freshness remains 5 ROS seconds. With real
time, accepted-map reception silence is also limited to 5 wall seconds. With simulation time, a
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
Cached images cannot authorize ordinary pursuit or a report; only the explicitly
bounded edge-recovery search may consume remembered direction. Cancellation,
battery Return/Abort requests, invalid telemetry and flight-control failures
remain authoritative.

Without visible fire, measured smoke-boundary proximity permits up to three observation
windows, with safe lateral viewpoint changes between them. Rejected or
unfinished viewpoint movements do not count as completed negative inspections.
A negative report requires all configured observation attempts to finish,
current confirmed smoke, fresh measured boundary/standoff evidence, and no
confirmed fire during the inspection. Smoke need not cover all quadrants.
For an opportunistically measured boundary, the publisher probes again before
and after other report preparation; a cached plane cannot authorize the result.
Otherwise the result is inconclusive. Previously confirmed fire cannot become
a negative report during a detector gap: bounded reacquisition either recovers
fresh fire or finishes as an incomplete, aborted mission holding `AUTO.LOITER`,
without a no-fire report or completed-site marker.

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
smoke/fire target coordinates. A positive report also requires a fresh 3 by 3 m
smoke-wall measurement at the configured standoff. A positive report can have
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

An opt-in smoke-wall benchmark runs the installed native estimator on known
no-smoke terrain with seeded map-registration jitter. It measures per-probe
candidates, per-episode false walls, retreat eligibility and the two-near-map
gate. These are conditional synthetic stress-test rates, not a real mission
false-positive percentage. The default test run skips this benchmark.
After sourcing ROS and the built workspace, run from the workspace root:

```bash
KV_RUN_WALL_FALSE_POSITIVE_BENCHMARK=1 \
  install/kv_executive/lib/kv_executive/python_shared_runtime --python /usr/bin/python3 -- \
  -m pytest -q -s src/kv_executive/test/test_smoke_wall_false_positives.py
```

The original [readable report](test/reports/smoke_wall_false_positives_20261102.md)
and [raw measurements](test/reports/smoke_wall_false_positives_20261102.json)
are retained. A paired guarded rerun writes a
[comparison report](test/reports/smoke_wall_false_positives_guarded_20261102.md)
and [raw measurements](test/reports/smoke_wall_false_positives_guarded_20261102.json).
On the same 240 synthetic terrain episodes the unguarded estimator qualified
false walls in 175 episodes and the guarded estimator in zero. Smoke positive
controls run separately. This result does not measure the rate in flight or
prove that all terrain or smoke shapes are classified correctly.
Set `KV_WALL_FALSE_POSITIVE_REPORT_DIR` to keep a rerun's output elsewhere.
No live mission or detector thresholds are changed by these tests.

The expanded production probe has a separate opt-in terrain regression:

```bash
KV_RUN_WALL_FALSE_POSITIVE_BENCHMARK=1 \
  colcon test --packages-select kv_executive --ctest-args -R '^expanded_wall_evidence$'
```

Its [raw report](test/reports/smoke_wall_expanded_evidence_20261102.json) records
240 episodes of eight maps with 49 terrain-directed ray lanes. With terrain
rejection enabled it accepted zero wall candidates across 1,920 queries. The
reference without terrain rejection accepted candidates in 180 episodes; that
reference also disables the persistent route because its terrain guard is absent.
These are candidate diagnostics, not simulated arrival or retreat decisions.
The measured 95th-percentile service time was about 90 ms, with a 185 ms maximum
on this test machine; the optional production probe budget is 500 ms. Native
tests separately cover both smoke evidence routes and their refusal cases.

The [grouped-depth rerun](test/reports/smoke_wall_grouped_evidence_20261102.json)
uses the current estimator on the same 240 terrain episodes: zero candidates
across 1,920 guarded queries, versus candidates in 180 reference episodes.
The measured 95th-percentile service time was 84 ms, with a 100 ms maximum.
Separate positive-control tests verify that a supported near surface remains
measurable when distant background returns occupy the rest of the aperture.
These synthetic results do not establish an operational false-positive rate.

The package repositories are tagged
`checkpoint-before-wall-validation-motion-limits-20261102` before these changes.
Rollback of the executive and perception must be paired and followed by a rebuild
and restart because their ROS message and service interfaces changed together.
The checkpoint immediately before the expanded probe and confirmation-memory
changes is `checkpoint-before-expanded-wall-evidence-20261102` in each package
repository. This change updates the executive service interface; rebuild and
restart the autonomy nodes together when applying it or rolling it back.

The checkpoint before physical settling, image-time heading and depth-group
selection is `checkpoint-before-settled-wall-heading-20261102` in all three
package repositories. These changes add no ROS interface fields; rebuild the
executive and restart autonomy after applying them or restoring this checkpoint.
The [validation report](test/reports/settled_wall_validation_20261102.md) records
automated results, the remaining post-query settling guard, and flight checks.

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
