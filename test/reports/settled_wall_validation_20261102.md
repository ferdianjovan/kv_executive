# Settled wall observations and fire heading validation

Work is saved, but the plan is incomplete: the post-query settling-loss guard
described below still needs implementation and validation. Work stopped for the
user's departure deadline; all isolated test processes have ended.

The rollback tag is `checkpoint-before-settled-wall-heading-20261102` in
`kv_executive`, `kv_perception`, and `kv_simulator`. Only `kv_executive` has
implementation changes. Rebuild the executive and restart autonomy after
applying or restoring these changes.

## Behavior

- Wall classification requires a continuous one-second stationary LOITER dwell
  with distinct fresh odometry samples, translation at most 0.10 m/s, and full
  angular rate at most 3 degrees/s. Stable camera tilt is permitted.
- A subsequent camera frame and source-stamped map update are required.
  Optional missing wall evidence still permits an otherwise safe distant
  approach; ordinary collision checks and retained wall planes stay active.
- Fire heading uses acquisition-time camera/body transforms, consumes each
  image once, retains fire priority through a brief gap, and supports cumulative
  turns beyond 5 degrees. Stopped alignment verifies a new image within
  2 degrees, including camera roll and vertical image error.
- Pending wall confirmation holds the heading and keeps its original
  20-second deadline. The 600-second inspection clock also covers preparation.
- Extended wall rays are qualified in separate depth groups with a maximum
  4 m span, retaining the existing spatial, terrain, camera and evidence checks.

## Automated results

| Check | Result |
| --- | --- |
| Build | Passed |
| Unit and native planner cases | 1,168 passed |
| Final three-cycle process regression | Passed, 86.54 s |
| Python, C++, CMake and XML style checks | Passed |
| Both negative-inspection ROS variants | Passed |
| Four confirmation-gap ROS variants | Passed across the final scoped runs: 3 cases, then remaining case in 19.37 s |
| Synthetic terrain-jitter benchmark | 0 accepted wall candidates / 240 episodes, 1,920 queries |

The benchmark is a conditional synthetic terrain test, not an operational
false-positive rate. Its raw measurements are in
[the grouped evidence report](smoke_wall_grouped_evidence_20261102.json).

The ROS fixture now gives observers the configured preparation budget, asserts
that the action remains active, and counts controller-accepted source-stamped
votes. Production timeouts and watchdogs were not lengthened.

Final logs are kept at:

- `/tmp/kv_settled_wall_final_build.txt`
- `/tmp/kv_settled_wall_affected_final_checks.txt`
- `/tmp/kv_settled_wall_confirmation_final_checks.txt`
- `/tmp/kv_settled_wall_xml_final_check.txt`
- `/tmp/kv_settled_final_process_regressions.txt`
- `/tmp/verify_ros_settling_final_six.log`
- `/tmp/verify_ros_settling_gap_recorded_final.log`
- `/tmp/verify_ros_settling_gap_state_final.log`
- `/tmp/verify_ros_settling_gap_search_flicker_final.log`

## Remaining implementation issue

Before any accepted near-wall vote, physical readiness can be lost between a
wall query returning and the controller recording that result. The controller
correctly rejects the observation, but can then treat it as ordinary optional
missing range and permit a checked forward approach leg. Physical instability
must instead hold and reacquire a settled viewpoint. Missing range from a
physically stable viewpoint should continue permitting safe distant approach.

The next change should route an unready wall decision through the existing
bounded settling-failure hold. Reset that failure counter only after a
physically valid wall decision, so repeated losses after successful preparation
cannot loop indefinitely. Add regressions for loss during/after a query,
repeated losses, and stable-but-unavailable range, then rerun affected ROS cases.
No rushed runtime fix was applied for this issue.

These are scoped automated checks. Resolve the remaining guard before simulator
flight validation; passing checks do not make the unfinished plan complete.
