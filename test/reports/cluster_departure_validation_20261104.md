# Completed clusters and departure validation

Date: 2026-11-04.

## Rollback checkpoint

The previous work was committed and all three package repositories were clean
before editing. The annotated tag
`checkpoint-before-cluster-departure-20261104` records:

| Repository | Previous commit |
| --- | --- |
| kv_executive | `1c7e3e3` |
| kv_perception | `4631828` |
| kv_simulator | `6d2796d` |

This change modifies executive and perception only. Restoring the checkpoint
requires matching both repositories, rebuilding their generated interfaces and
restarting all affected nodes/consumers together.

## Implemented behavior

Completed positive and negative inspections save a measured smoke-wall coordinate
and associated source views before leaving Verify. The coordinate and original
measurement stamp remain fixed. The shared registry retains at most 100 clusters
across movement, inspection resets and subsequent mission requests in the same
executive process. Only an actual source-clock rewind or process restart clears
measured cluster history. Reordered or queued old-epoch odometry cannot establish
pose associations in a reset epoch. Legacy bearing-only compatibility records
and inconclusive viewpoint deferrals retain their existing movement-based retry.

The initial grouping policy is a 20 m horizontal core with a 5 m buffer. It is
an estimated inspection footprint, not a measured plume outline or a fire
coordinate. Nearby reports merge into the fixed original anchor rather than
growing it through chained reports. A merged positive finding is not weakened
by later negative metadata. Detector/report views are stored without duplicating
the embedded image payloads.

Outside candidates need three distinct fresh source observations. A continuous
previously completed component can drift across the border without becoming a
new site; a clearly separate bearing, identity discontinuity, observation gap or
large independently measured position jump requires a new association decision.
No new ranging RPC is required to approach a distinct distant plume. Negative
smoke inspections cannot suppress subsequently confirmed fire.

`DEPART_SITE` follows report publication. Its separate default budget is 180 wall
seconds and 12 legs of 3 m, with 1 m exit clearance beyond buffered footprints.
It rotates toward each proposed departure route and flies forward with fixed
departure heading rather than remembered fire steering. A checked outward-only
prefix handles starts inside inflated buffers. Collision, terrain and body
clearance are never waived. Failure or cancellation retains the completed report;
the mission recovers to LOITER rather than inventing another finding.

The native planner receives authoritative retained snapshots on
`completed_inspection_clusters`. All normal planning edges and inspection
corridors respect every vertical cylinder; climbing above it is not an escape.
Exploration also checks actual position and the remaining route before OFFBOARD,
after warmup and during movement. Explicit escape permission is restricted to
departure, uses a fixed anchor and cannot permit inward motion or re-entry.

Perception keeps existing primary outputs and adds arrays of up to 16 current
independently confirmed smoke/fire candidates. The monitor can request Verify
from a confirmed secondary candidate while a primary remains tentative. The
executive filters candidates before guidance, remembered intent, height recovery
and reporting. Later inspections bind their chosen target so loss does not
silently transfer steering to an unrelated or completed source.

Images arriving before their odometry bracket are retried when polled after pose
arrival. Retrying preserves original receipt age and cannot renew stale data or
increment persistence for duplicate frames. Detector callbacks do not wait for
map/local TF; report completion reuses the final validated report alignment.
The finding/report format, compressed original images and existing wall/terrain
qualification remain in place.

## Verification results

| Coverage | Cases |
| --- | ---: |
| Relevant executive unit/protocol suites, including monitor | 1,423 |
| Native planner and geometry DDS suite | 147 |
| Full perception regression suite, including detector subprocesses | 183 |
| Executive mission/report/departure/safety DDS cases | 24 |
| Monitor legacy and candidate-only mission DDS cases | 4 |
| Separate executive/monitor process, three distinct site cycles | 1 |

All **1,782 distinct pytest cases passed** in the scopes above. The separate
process completed three full finding/report/departure/handoff cycles in 184.21 s.

Executive unit/protocol coverage includes the prior fire-finding, independent
fire-loss counters, height/yaw guidance, physical settling, source localization,
retained smoke reporting, compressed image serialization and image-outage cases.
All 129 previous native cases are included in the 147-case native run. The full
perception suite preserves primary detector/tracker/background and overlay
regressions alongside candidate publication.

New coverage includes:

- Core/buffer boundaries, one-metre edge drift, three distinct outside images,
  replay rejection, changed track IDs, delayed reporting and source-pose loss.
- No blanket all-direction suppression when the observer starts inside the
  inflated cluster. Distinct right-side/lower candidates remain discoverable.
- Cluster persistence beyond 2 m movement; fixed-center merging; bounded storage;
  real clock reset, reordered telemetry and authoritative empty snapshots.
- Every swept edge, high-altitude overflight rejection, exact tangents, overlapping
  cylinders, monotonic outward escape, immutable anchors and no re-entry.
- Native/executive escape-flag agreement, pitched/transformed frames, current-body
  checks before departure yaw, final settling and registry reset during departure.
- Exploration rejection before OFFBOARD, registry changes during warmup, in-flight
  drift and later-route violations, while safe normal three-section paths complete.
- Successful departure before Exploration, no immediate reinspection on the new
  viewpoint, blocked departure retaining the report in LOITER and cancellation.
- Both RGB/thermal evidence, each missing stream, both missing, completed negative
  reporting, both initial wall evidence routes, braking/settling, map replay,
  near-to-far recovery, Abort and battery priority.
- Primary and array feeds, same-header empty revocation, no tentative promotion,
  source/receipt aging, source-time pose retry, duplicate polls and correctly
  selected image metadata when a secondary component becomes the target.

Package builds and Python/CMake/C++ formatting/docstring checks are verified
separately. XML schema validation required network access to the official ROS
schema after the sandboxed fetch failed. Selected ROS tests emitted existing
`Destroyable` notices during participant teardown after passing summaries;
those notices did not represent failed mission assertions.

## Scope and limits

Tests use isolated synthetic sensor streams, fake flight controllers and real
ROS DDS/process boundaries. No complete Gazebo/SITL flight or physical vehicle
experiment was performed for this change. This is relevant regression coverage,
not a claim that every test in the workspace was rerun.

The process test uses three genuinely separated sites: moving 2 m must no longer
authorize reinspection of the old site. Its synthetic smoke/fire bearings are
fixed in the world, rather than following body yaw during a sweep. It uses a
longer checked departure leg to keep subprocess duration bounded; separate unit
and DDS cases exercise the default short-leg departure.

Intermediate process attempts exposed delayed-pose retry starvation and used
synthetic bearings that followed body yaw instead of a fixed world target. The
retry was fixed and directly tested without renewing receipt age; fixture
bearings were corrected. One earlier process attempt aborted safely on OFFBOARD
warmup during its third cycle. The final isolated run used the latest bounded
geometry copies/nonblocking callback association and completed all three cycles;
that result does not establish the exact cause of the earlier timing failure.

The registry is in memory, not a durable restart archive. A smoke-wall point is
not the physical fire emitter, and image-only overlapping plumes may remain
ambiguous. A bound target waits when image-time association is unavailable;
this is not authorization to steer from current pose or an old image. All
geometric cluster boundaries remain policy estimates.

Detailed local artifacts include:

- `/tmp/kv_cluster_commit_checks.txt` and generated CTest xunit reports;
- `/tmp/kv_completed_cluster_native.xml`;
- `/tmp/kv_candidate_perception_all.xml`;
- `/tmp/kv_completed_depart_dds.txt` and `/tmp/kv_completed_quorum_dds.txt`;
- `/tmp/kv_candidate_monitor_dds.xml`;
- `/tmp/kv_candidate_retry_focused.xml`;
- `/tmp/kv_completed_depart_process.txt`;
- `/tmp/kv_cluster_xml_retry.txt`.
