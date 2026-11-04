# Completed-site projection and alternate-view validation

Date: 2026-11-04.

## Checkpoint and scope

All three package working trees were clean before implementation. The annotated
tag `checkpoint-before-projected-completed-site-20261104` preserves executive
`d74806f`, perception `5738d9b`, and simulator `6d2796d`. Only executive source,
configuration, documentation and tests changed. Existing ROS interfaces and
native planner geometry did not change.

The observed duplicate reports were two positive findings and one negative
finding for the same cluster. A routing zone existed throughout that flight;
weak bearing association and publication before spatial matching allowed the
repeated reports. The corrected wall-front routing geometry is retained.

## Resulting behavior

- RGB smoke component bounds plus a 3-degree margin are projected against the
  completed physical coverage. Thermal components provide a center bearing
  with a margin because their messages contain no angular bounds. This uses
  ROS map coordinates, image-time local poses and map transforms from that same
  image timestamp. Transform lookups never wait in detector callbacks.
- Recognition uses finite inspected vertical coverage. Infinite-height routing
  cylinders continue preventing overflight, but do not constitute visual proof
  that every object above or below the site is the same source.
- Projection has no 70 m cutoff and starts no LiDAR query. Overlap alone remains
  ambiguous; three nonmatching overlapping images cannot establish novelty.
  Clearly separated candidates still need three distinct fresh images.
- Completed and ambiguous overlapping directions can schedule alternate views.
  Each site/source/direction has two attempts by default. Queue directions are
  relative to the saved wall, so vehicle parallax and detector track changes do
  not recreate exhausted tasks. Bounded records and tombstones survive action
  retries, clear on actual clock rewind, and remain session memory.
- A useful observation requires a measured 6 m horizontal lateral baseline from
  the initial observer and every prior alternate view. Planned endpoints, pure
  height changes, forward drift and planner success without actual motion cannot
  establish this baseline. The drone physically settles and holds for 2 seconds
  after obtaining its investigation permission.
- Only one source-specific investigation permission is active. Fire tasks have
  priority when both streams need an alternate view. Permission expires after
  60 wall seconds in Exploration and survives its admitted Verify episode.
  Among eligible components, visibly uninspected angular coverage takes priority
  over sticky old tracks. Synthetic smoke-depth anchors affect ordering only;
  they never become measured fire positions or depth identity proof.
- Departure and Exploration schedule these views through existing terrain and
  whole-path collision checks, around completed sites. Outside-zone departure
  also tries short onward/lateral progress before another full sweep. A blocked
  discretionary view consumes a bounded attempt and allows ordinary exploration;
  mandatory escape, cancellation, map/vehicle failures and recovery still keep
  their existing guards and LOITER behavior.
- Confirmed arrival checks the measured wall against saved clusters before
  another fire search. Reporting checks again before image capture and before
  either publisher or site-ID allocation. Known findings are not republished;
  a known positive site cannot receive a replacement negative finding. A genuine
  fire upgrades an earlier negative finding under the original site ID.
- Fresh raw smoke is available only to report-view/evidence checks when the
  navigation filter has excluded that completed site. Image-time wall-direction
  association, freshness, settling, original image encodings and payload limits
  remain required. This preserves negative-to-fire upgrades without reopening
  completed smoke as a navigation target.
- A newest image waiting for pose/TF cannot erase an earlier still-fresh,
  associated eligible image or renew its receipt age. Unknown images without
  valid earlier evidence remain withheld. Explicit negative observations revoke
  guidance. Logs distinguish raw reception, eligibility and admission reasons.

## Validation

| Scope | Passing distinct cases |
| --- | ---: |
| Executive unit/protocol suites, 24 files | 1,643 |
| Full native planner DDS suite | 160 |
| Full perception suite, including image-process DDS retry | 183 |
| Executive report/departure DDS variants | 11 |
| Separate executive/monitor process, three distinct sites | 1 |

All **1,998 distinct cases passed** in these scopes. The separate executive and
monitor processes completed three full finding/report/departure cycles in
162.40 seconds, with the production 6 m alternate-view baseline and original
yaw/height checks retained.

The executive unit suites include actual combined controller/registry/report
flows, a 100 m aligned candidate behind a completed 60 m site, competing old/new
components, duplicate suppression after an alternate view, negative-to-fire
upgrades, invalid/stale/replayed images, bounded queue eviction and clock reset.
The distant-site report is made only after simulated arrival using an 18 m
confirmed wall measurement; the test does not assume 100 m LiDAR reach.
Real TF buffer tests cover image-time transforms with missing newer TF,
translation/rotation, and rejection when only a newer transform is available.
Previous height/yaw guidance, slow descent, movement-failure counters, smoke-wall
qualification and image evidence tests remain included.

The 11 latest unique executive DDS variants include all five image combinations,
fire reporting without fire localization, free/blocked/canceled zone departure,
fire-loss recovery, and the same world source after a track reset and second
Verify episode. That final case retains one inspection report, one legacy report
and the original site ID. Latest results are audited by case identity rather than
counting reruns as additional passes.

Initial DDS runs exposed fixture bounds that assumed departure only rotated:
60-second handoff and 90-second startup waits expired during legitimate checked
translation. Later focused fixtures use 3 m baselines and 1.5 m legs to bound test
duration; the first four DDS variants exercised the production 6 m baseline.
One loaded-host run correctly rejected a 2.08-second-old telemetry sample under
the unchanged 2-second production limit. A negative-result fixture incorrectly
assumed its report position preceded its three inspection viewpoints; that
assertion now applies only to positive cases. An interrupted test was rerun with
a clean context. Sandbox DDS interface restrictions required isolated local
network access for native and perception process tests.

The first three-site process run found a real regression: latest unassociated
images repeatedly vetoed an earlier fresh eligible source at the second site.
This was corrected and covered by deterministic primary/array regressions before
the final process rerun. A subsequent third cycle completed its height correction
and reached zero vertical error immediately before the fixture's 35-second report
wait expired. Its source was admitted and the mission continued making progress;
the fixture now allows 90 seconds for yaw/height correction and source-time
qualification, as it does for departure. Production flight timeouts were not relaxed.

Build, Python style/docstrings, CMake style and C++ formatting checks pass.
No complete Gazebo mission or physical vehicle flight was performed. Perfectly
aligned plumes can remain visually inseparable; alternate viewpoints provide
bounded investigation opportunities rather than guaranteed depth separation.

## Artifacts

- `/tmp/kv_projected_final_units.xml` and `.log`
- `/tmp/kv_projected_native_full.xml` and `.log`
- `/tmp/kv_projected_perception.xml` and `/tmp/kv_projected_perception_ros.xml`
- `/tmp/kv_projected_dds_latest_cases.json`
- `/tmp/kv_projected_verify_dds.xml`, `_retry.xml`, and `_negative.xml`
- `/tmp/kv_projected_process_confirmed.xml` and `.log`
- `/tmp/kv_projected_build_final.log` and `/tmp/kv_projected_style_final.log`
- `/tmp/kv_projected_validation_audit.json`
