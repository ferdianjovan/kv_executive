# Retained smoke coordinates and image evidence validation

Date: 2026-11-04. Pre-change executive commit: `85369af`.
The annotated rollback tag `checkpoint-before-inspection-evidence-20261104`
was created before editing in `kv_executive`, `kv_perception` and `kv_simulator`.
Only `kv_executive` changes in this implementation.

## Result and scope

Completed inspection reports reuse the actual nearest qualified smoke-wall hit
retained when two distinct map observations confirm arrival. Its map coordinate
and measurement timestamp stay fixed if the drone moves or the map/local
alignment changes. Final publication requires fresh forward RGB smoke at a
physically settled viewpoint, with fresh persistent fire for a positive result.
It does not require another wall-ranging query. Negative results still require
all observation attempts and cannot conceal previously confirmed fire.

Reports carry compressed original RGB and lossless thermal images with source
timestamps, matching detector metadata and, when available, interpolated
image-time local poses. Missing images are explicit and do not invalidate an
otherwise valid result. Terrain/collision checks, movement-failure counts,
separate fire-loss counts, cancellation and inspection deadlines remain active.

Tests use generated ROS interfaces and isolated fake-FCU/DDS environments.
No real vehicle commands or complete simulator flight were run for this change.

## Regression results

| Coverage | Passed cases |
| --- | ---: |
| Mission geometry / launch configuration | 38 / 32 |
| Mission executive / mission navigation | 180 / 86 |
| Verification / independent fire-loss recovery | 305 / 27 |
| Inspection navigation and reporting protocol | 318 |
| Heading / physical settling / observation | 30 / 28 / 13 |
| Image evidence using generated messages | 49 |
| Retained-coordinate report viewpoint / controller retention | 21 / 15 |
| Motion policy / source localization / completed sites | 20 / 15 / 50 |
| Native planner/service inputs | 129 |
| DDS report and safety integration | 21 |
| Independent process integration (three successful mission cycles) | 1 |

Total: **1,378 distinct pytest cases passed** in the relevant suites and selected
integration cases. The first 16 unit/protocol suites ran through `colcon test`
with 1,227 cases and no failures or skips. Native planner results combine 128
passing cases from the full run and one corrected test-fixture rerun: an ordinary
surface query initially used unknown space instead of the existing dense-free
fixture; no production behavior was changed to satisfy it. This is a scoped
regression result, not a claim that every workspace test was rerun.

Build, Python style/docstring, CMake style, C++ formatting and package XML checks
passed. XML validation was rerun with network access to fetch the official ROS
schema after the sandbox blocked the first attempt. The first IDL generator
process crashed under the virtual-environment Python; regenerating with the
existing shared-Python launcher succeeded, followed by successful package builds.
The configured cppcheck job skipped its cases because the installed ROS wrapper
disables cppcheck 2.13.0 for known performance issues; it is not counted as a
passing static analysis check.

## Important cases covered

- Native points belong to the selected qualified depth group; an off-axis point
  is the nearest actual Euclidean hit, not a synthesized center-ray point.
  No qualified wall means no valid wall point. Existing distance/uncertainty
  metrics and collision behavior are unchanged.
- Two distinct map timestamps are required for retained reporting. Replayed,
  missing, unqualified or malformed measurements cannot create confirmation.
  Missing later probes preserve a confirmed coordinate; explicit new far/close
  observations invalidate it through the existing movement safety paths.
- Drone translation and map/local alignment changes preserve the map target and
  update local/global offsets using one current transform for targets and observer.
  A current drone pose never substitutes for a target or image acquisition pose.
- Final smoke may occupy one quadrant. A retained-coordinate yaw attempt is
  bounded, visible fire retains heading priority, and physical settling remains
  mandatory. Lack of new ranging after confirmed arrival cannot itself prevent
  reporting. Before arrival, the original wall qualification remains mandatory.
- DDS positive reports were decoded with both images, RGB only, thermal only and
  neither stream; a completed negative inspection also carried both images.
  JPEG pixels decode, 16-bit PNG pixels match exactly, and metadata headers match
  the source image. Missing streams are flagged explicitly.
- Stale, reordered, future-dated, mismatched-frame and mismatched-track images;
  source/receipt aging during encoding; padded rows; endian handling; invalid
  dimensions; unsupported formats; raw/encoded byte caps; episode reset; clock
  rollback; source-time pose interpolation; and briefly delayed odometry.
- Fire loss, later fire appearance, smoke gaps, changed fire identity, late
  evidence changes, map replay, near-to-far invalidation, frozen-clock settling,
  deadline expiry, action cancellation, Abort and battery abort. Image capture
  cannot bypass the existing publication guards or generate a false negative
  after confirmed fire.

## Payload and retention

A deterministic generated-message serialization experiment used a noisy
1920 x 1080 RGB image and a random 640 x 512 16-bit thermal image:

| Item | Bytes |
| --- | ---: |
| Original raw pixels, combined | 6,876,160 |
| Stored RGB JPEG | 243,561 |
| Stored thermal PNG | 657,095 |
| Serialized complete inspection report | 902,016 |

This example reduces total bytes by about **87%**, while thermal pixels decode
exactly. Size depends on image content. Default per-image caps are 256 KiB RGB
and 1 MiB thermal; thermal evidence that cannot fit is marked unavailable rather
than downsampled or encoded lossily. RGB's longest side is at most 1280 pixels.
Detection rectangles remain in original image coordinates with stored/original
dimensions available for display scaling.

The raw cache is limited to 16 MiB total, three frames per stream and 8 MiB per
incoming frame; image subscriptions keep one sample. DDS retains two reports by
default, configurable up to ten. These bounds cover application caches and
payloads, not all middleware or encoder working memory. Record the report topic
for a durable archive; transient-local history is intentionally limited.

Image capture uses a three-second wall deadline, two-second source/receipt age
limits and a 0.5-second RGB/thermal pairing tolerance. A missing pose bracket or
image does not invent metadata: validity flags and status describe what is
available. GPS remains optional. The smoke point is the **last confirmed plume
boundary estimate**, not a guarantee of current smoke extent or fire location.

## Test artifacts

Detailed logs were written to the local test workspace:

- `/tmp/kv_inspection_report_final_checks.txt` and
  `build/kv_executive/test_results/kv_executive/*.xunit.xml`
- `/tmp/kv_nearest_wall_native.{log,xml}` and
  `/tmp/kv_nearest_wall_native_retry.{log,xml}`
- `/tmp/kv_report_evidence_dds.txt`, `/tmp/kv_retained_quorum_dds.txt`,
  `/tmp/kv_retained_smoke_gap_dds.txt`, `/tmp/kv_retained_process_dds.txt`
- `/tmp/kv_inspection_report_final_build.txt` and
  `/tmp/kv_inspection_report_xml_check.txt`

The 14-case quorum/safety DDS batch printed two teardown-only ROS
`Destroyable` notices after its passing summary; all assertions and mission
results passed. No runtime behavior was weakened to suppress those notices.

The `InspectionReport` and `QueryInspectionGeometry` interfaces changed.
Rebuild `kv_executive` and restart autonomy and all report/service consumers
together before the next flight experiment.
