# Settled wall observations and fire heading validation

The post-query settling-loss guard is implemented and its relevant automated
checks pass, completing the previously unfinished implementation plan.

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

## Original implementation results (`eb2bfac`)

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

## Post-query settling fix

Before any accepted near-wall vote, physical readiness can be lost between a
wall query returning and the controller recording that result. Previously the
controller rejected the observation but could treat it as optional missing range
and permit a forward leg.

An unready wall decision now uses the existing bounded settling-failure hold and
returns to `REACQUIRE`. Successful preparation alone does not reset that failure
counter: a physically valid wall decision must do so. Repeated query-time losses
therefore reach the existing three-attempt bound. Historical votes, retained
planes, source stamps and existing deadlines survive this hold; only live wall
range is cleared. `CENTER_FIRE` cannot perform its optional height correction
while unready, and the inner fire-approach missing-result path uses the same hold.
Missing range from a physically stable viewpoint still permits safe distant
approach. Existing timeouts and watchdogs are unchanged.

| Follow-up check | Result |
| --- | --- |
| Build | Passed |
| Relevant unit regressions | 964 passed, including 23 new controller cases |
| Three-cycle process regression | Passed, 96.921 s |
| Python style and docstring checks | Passed |
| Focused ROS motion and confirmation checks | 9 passed across scoped runs: 7 existing cases and 2 new motion variants |

Both new motion variants passed in 36.06 s. They inject real odometry motion
during the first wall query, verify LOITER holding without navigation plans,
near-wall votes or reports, then require a newer settled epoch and a full dwell
before recovery. The seven existing cases cover braking, both evidence routes
through missing probes, and forward approach with a stable out-of-range wall.

Follow-up logs:

- `/tmp/kv_remaining_settle_build.txt`
- `/tmp/kv_remaining_settle_regressions.txt`
- `/tmp/kv_remaining_settle_process.txt`
- `/tmp/kv_post_query_settle_ros.log`
- `/tmp/kv_post_query_motion_final_ros.log`

The combined run passed the seven existing cases. Its motion-test assertion
also prohibited permitted forward movement after settling; the corrected two
motion variants passed in the final scoped rerun.

The pre-fix implementation remains available at commit `eb2bfac`. These are
scoped automated checks; a simulator flight remains to be performed.
