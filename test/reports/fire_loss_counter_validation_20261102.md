# Independent fire-loss recovery counter

Baseline / rollback commit: `5daf663` in `kv_executive`.

## Behavior

Missing usable fire in `REACQUIRE` and `CENTER_FIRE` consumes
`verify_fire_loss_failures`. Blocked paths, height corrections and lack of
movement continue using `verify_search_failures`. Each independent counter
uses the existing `verify_search_recovery_max` limit (3 by default).

Only newly accepted, persistent fire guidance from a physically settled
viewpoint clears the fire-loss count. Its image must be newer than the loss,
arrival, inspection and settling cutoffs. The loss boundary also includes the
latest received source timestamp, so a previously received future-dated frame
cannot refill retries. Remembered confirmation and in-flight sightings cannot
clear this counter. Recovery does not require a wall measurement; successful
publication still requires current qualified wall evidence.

Recovered fire does not clear movement failures. Existing resets after useful
movement remain in place. Movement alone does not clear unresolved fire loss.
Both counters and the loss cutoff are initialized for each new Exploration
episode. Logs distinguish `fire-loss recovery` from movement's
`blocked recovery`, and explicitly report a qualified fire-loss reset.

Inspection, wall, map, settling, no-progress and movement limits retain their
existing behavior. In particular, the 600-second inspection clock and the
20-second wall-revalidation deadline are not restarted by fire recovery.

## Validation

| Check | Result |
| --- | --- |
| Build | Passed |
| Fast Python regression cases | 1,094 passed |
| New deterministic counter cases (included above) | 27 passed |
| Python style, docstrings and CMake lint | Passed |
| ROS fire-loss recovery / independent movement counter | Passed |
| ROS continuous-fire blocked movement | Passed |
| Existing ROS band, height, confirmation-gap and cancellation cases | 10 passed across scoped runs |
| Three-cycle process regression | Passed, 85.476 s |

The new ROS recovery case observed actual fire-loss counts `1, 2`, qualified
settled recovery to `0`, then a new loss at `1`, while movement failures stayed
at `2` and existing timers were preserved. The blocked-movement case observed
`1, 2, 3` and an abort despite continuously usable fire; it published no false
success.

All 12 selected ROS case identities passed across scoped runs. One existing
smoke/flicker confirmation-gap case exhausted the unchanged settling guard
after three odometry observation gaps during concurrent DDS tests. Its isolated
rerun passed in 11.58 s with no test or production changes. The final fire-loss
recovery case passed in 19.82 s.

Deterministic cases also cover stale/unconfirmed/nonpersistent images,
pre-loss and pre-viewpoint frames, duplicate future-dated frames, unsettled
observations, last-retry recovery races, remembered routing, unchanged budgets,
and withholding completion without current wall evidence.

Logs:

- `/tmp/kv_fire_loss_counter_build.txt`
- `/tmp/kv_fire_loss_counter_regressions.txt`
- `/tmp/kv_fire_loss_recovery_ros.log`
- `/tmp/kv_fire_loss_new_ros.log` (blocked-movement case)
- `/tmp/kv_fire_loss_existing_ros.log`
- `/tmp/kv_fire_loss_gap_retry_ros.log`
- `/tmp/kv_fire_loss_counter_process.txt`

These are scoped automated checks. A simulator flight remains to be performed.
