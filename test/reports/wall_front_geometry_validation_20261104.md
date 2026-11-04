# Wall-front routing geometry and departure retry validation

Date: 2026-11-04.

## Checkpoint and scope

Previous work was already committed and all three package working trees were
clean before editing. The annotated rollback tag
`checkpoint-before-directional-smoke-zone-20261104` records executive `ab90314`,
perception `5738d9b` and simulator `6d2796d`. Only executive changes in this task.

The flight under investigation reported fire at 15:13:18, then exhausted the
180-second departure budget at 15:16:18. Two subsequent airborne requests failed
at Exploration entry. The old zone was a 25 m circle centered on the wall hit,
so an observer at a normal 15–20 m standoff was already inside it. The startup
guard could reject a stationary current position with the misleading message
that a remaining route was entering the zone.

## Geometry

Let W be the original retained smoke-wall point, O the observer at that wall
measurement, and n the horizontal unit vector from O toward W. For the existing
20 m core and 5 m buffer, R=25 m. Routing now uses:

```
C = W + R*n
protected footprint = horizontal circle(center=C, radius=R)
```

The whole rounded footprint lies behind the wall's front plane, including its
buffer. It still extends vertically, so a route cannot climb over it. For O=0 m
and W=20 m along the viewing direction, C=45 m and the footprint spans 20–70 m,
instead of the previous -5–45 m. Changing the radius changes its width/backward
reach without extending it toward the original observer.

W remains the reported smoke coordinate and semantic association/merge anchor;
C is derived routing geometry only. The original map-frame measurement observer
is copied with the same map timestamp as W and retained through later vehicle
movement. Ordinary report preparation does not replace it with the current
drone pose or camera yaw. Validated legacy observations without that provenance
can use the report's planning pose. Invalid explicit provenance or a degenerate
horizontal direction cannot invent a routing zone or silently fall back to a
bearing-only completed record; the published finding remains available.

No native planner algorithm, ROS message fields, or service fields changed.
The existing all-segment cylindrical keepout implementation consumes C and R.
Occupied-body, terrain and map checks remain independent of semantic tangency.

## Related departure fixes

- An airborne `skip_takeoff:true` request inside a retained zone or its exit
  margin enters Verify and resumes checked departure before ordinary Exploration
  admission. It reuses the saved site identity without publishing another report.
  Cancellation and unsuccessful exit still recover to LOITER.
- A departure leg is planned and validated before turning. The existing flight
  helper performs the fixed-heading turn before translation; the duplicate
  separate OFFBOARD/LOITER turn cycle was removed. Refused routes no longer cause
  a needless turn or OFFBOARD entry.
- Actual zone clearance is rechecked after final yaw and physical settling.
  Drift back inside the exit margin causes another bounded checked departure,
  rather than announcing handoff and immediately failing Exploration.
- Useful, geometrically checked outward progress from a partial leg is retained
  with a fresh starting pose. Interrupted execution must confirm actual LOITER
  before retry; recovery failures, cancellation and the overall deadline are not
  swallowed.
- Logs distinguish an already-inside current position from an unsafe future
  route. They include site/clearance diagnostics and zone wall/center/direction.
  Committed departure movements are labeled `DEPART_SITE`, not `CENTER_FIRE`.

The 180-second deadline, 12-leg movement budget, 3 m default leg, original
inspection/report safeguards and 1 m settled exit margin remain unchanged.

## Results

| Coverage | Passed cases |
| --- | ---: |
| Executive unit/protocol suites | 1,486 |
| Full native planner/geometry DDS suite | 160 |
| Selected executive report/departure DDS cases | 11 |
| Separate process regression: three complete site cycles | 1 |

All **1,658 distinct cases passed** in these scopes. The separate executive and
monitor process completed three complete finding/report/handoff cycles in
112.46 seconds, with original height/yaw guidance checks retained.

The unit/protocol run includes 116 completed-site cases, 324 inspection protocol
cases, 41 cluster-motion cases, 19 departure cases and 14 startup-resume cases.
Existing finding, yaw/height guidance, independent fire-loss counters, physical
settling, source localization, original image encoding and missing-image
regressions remain included. All 147 previous native cases are in the 160-case run.

New coverage exercises 15/20 m wall distances and multiple headings, elevated
observers, every sampled core/buffer point behind the front plane, exact front
tangency, penetration/overflight rejection, multiple zones, occupied-body checks,
original observer provenance after movement, shifted-frame alignment and
unchanged semantic association at W. Startup tests use the actual action entry
path and check Verify-before-escape ordering, no duplicated findings, retry
after failure, cancellation, margin gating and current-vs-route diagnostics.

The 11 selected DDS cases cover both image streams, missing either/both streams,
completed negative reporting, both ends of the inspection band, fire-loss
recovery and normal handoff with no forced retreat. Genuine inside-zone retries
exercise free, blocked and canceled exit with default 3 m checked legs. Their
synthetic start is 3 m behind the front rather than 21 m deep inside the region;
the latter exceeded an earlier 90-second test wait while making outward progress.
Route fixtures use an 8-second mode timeout, as the image cases already did;
production mode timeout/budgets were not increased. An earlier 4-second combined
test warmup bound was exceeded under scheduling load. Latest exact-case XML
results are audited across focused reruns, rather than double-counting retries.

Build, Python style/docstrings, CMake style and C++ formatting checks passed.
No complete Gazebo flight or physical vehicle run was performed. Tests use
isolated DDS domains and synthetic FCUs/sensors, not the vehicle's ROS graph.
The registry remains session memory; no durable restart archive was introduced.

## Artifacts

- `/tmp/kv_wall_front_build.txt`
- `/tmp/kv_wall_front_units.xml` and `.txt`
- `/tmp/kv_directional_registry_units.txt`
- `/tmp/kv_shifted_wall_front_native.xml`
- `/tmp/kv_directional_wall_protocol.xml`
- `/tmp/kv_directional_dds_latest_cases.json`
- `/tmp/kv_directional_verify_dds*.xml` and `.log`
- `/tmp/kv_directional_verify_process.xml` and `.log`
- `/tmp/kv_wall_front_style.txt` and `/tmp/kv_wall_front_flake_retry.txt`

Initial broad DDS runs printed existing teardown `Destroyable` notices and an
unusually long elapsed time; those runs are not used to claim flight speed or
departure performance. The final results distinguish assertions, fixture waits
and actual safety outcomes.
