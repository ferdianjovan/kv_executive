# Smoke-wall false-positive sensitivity benchmark

Native planner commit: `672dbc8e50d159e1ef43452fc61d9e5b06bfe36b`. Seed: `67220261102`. Runtime: 2.36 s.

**These are controlled no-smoke terrain stress tests, not a measured mission rate.**

240 independent episodes, 8 maps each; 20 cm OctoMap cells and the real 25-ray, 3 × 3 m native estimator.

Each episode has a randomized voxel phase and starts with an empty local map history. Ranges 8, 16 and 24 m are balanced within every group. The terrain stays physically static while its mapped depth shifts uniformly by the stated amount per frame.

| Slope | Jitter ±m | Episodes | Any accepted 3/2/2 | Any accepted 6/3/3 | Retreat-eligible 3/2/2 | Two near maps 3/2/2 |
|---:|---:|---:|---:|---:|---:|---:|
| 45.0° | 0.000 | 30 | 0/30 (0.0%) | 0/30 (0.0%) | 0/30 (0.0%) | 0/30 (0.0%) |
| 45.0° | 0.025 | 30 | 28/30 (93.3%) | 8/30 (26.7%) | 19/30 (63.3%) | 13/30 (43.3%) |
| 45.0° | 0.050 | 30 | 29/30 (96.7%) | 25/30 (83.3%) | 19/30 (63.3%) | 17/30 (56.7%) |
| 45.0° | 0.100 | 30 | 30/30 (100.0%) | 30/30 (100.0%) | 20/30 (66.7%) | 17/30 (56.7%) |
| 63.4° | 0.000 | 30 | 0/30 (0.0%) | 0/30 (0.0%) | 0/30 (0.0%) | 0/30 (0.0%) |
| 63.4° | 0.025 | 30 | 28/30 (93.3%) | 10/30 (33.3%) | 18/30 (60.0%) | 13/30 (43.3%) |
| 63.4° | 0.050 | 30 | 30/30 (100.0%) | 28/30 (93.3%) | 20/30 (66.7%) | 19/30 (63.3%) |
| 63.4° | 0.100 | 30 | 30/30 (100.0%) | 30/30 (100.0%) | 20/30 (66.7%) | 20/30 (66.7%) |

## Stable and jittered terrain separately

- stable_terrain: 0/60 episodes (0.0%) yielded an accepted false wall under 3/2/2 (episode Wilson 95% interval 0.0%–6.0%); 6/3/3 counterfactual: 0/60 (0.0%).
  - accepted_3_2_2: after excluding the first map, 0/420 (0.0%) queries; eighth map only, 0/60 (0.0%), episode Wilson 95% interval 0.0%–6.0%.
  - counterfactual_6_3_3: after excluding the first map, 0/420 (0.0%) queries; eighth map only, 0/60 (0.0%), episode Wilson 95% interval 0.0%–6.0%.
- jittered_terrain: 175/180 episodes (97.2%) yielded an accepted false wall under 3/2/2 (episode Wilson 95% interval 93.7%–98.8%); 6/3/3 counterfactual: 131/180 (72.8%).
  - accepted_3_2_2: after excluding the first map, 680/1260 (54.0%) queries; eighth map only, 117/180 (65.0%), episode Wilson 95% interval 57.8%–71.6%.
  - counterfactual_6_3_3: after excluding the first map, 403/1260 (32.0%) queries; eighth map only, 68/180 (37.8%), episode Wilson 95% interval 31.0%–45.0%.

## Equally weighted aggregate

**accepted_3_2_2**

- any_accepted: 175/240 (72.9%); episode Wilson 95% interval 67.0%–78.1%.
- retreat_eligible: 116/240 (48.3%); episode Wilson 95% interval 42.1%–54.6%.
- two_successive_near_maps: 99/240 (41.2%); episode Wilson 95% interval 35.2%–47.6%.
- accepted_queries: 680/1920 (35.4%); correlated samples, no independent-query interval.
- accepted_queries_after_first_map: 680/1680 (40.5%); excludes guaranteed history warm-up.
- eighth_map_only: 117/240 (48.8%); episode Wilson 95% interval 42.5%–55.0%.

**counterfactual_6_3_3**

- any_accepted: 131/240 (54.6%); episode Wilson 95% interval 48.3%–60.8%.
- retreat_eligible: 87/240 (36.2%); episode Wilson 95% interval 30.4%–42.5%.
- two_successive_near_maps: 57/240 (23.8%); episode Wilson 95% interval 18.8%–29.5%.
- accepted_queries: 403/1920 (21.0%); correlated samples, no independent-query interval.
- accepted_queries_after_first_map: 403/1680 (24.0%); excludes guaranteed history warm-up.
- eighth_map_only: 68/240 (28.3%); episode Wilson 95% interval 23.0%–34.3%.

## Limits

- Conditional synthetic stress-test rates, not real mission false-positive prevalence.
- All 25 ray lanes target observed steep terrain. This deliberately conditions on the terrain-jitter failure mode, not ordinary open-air or smoke-directed views.
- Ray-aligned sparse terrain returns; raw sensor noise/dropout and OctoMap fusion are not simulated.
- Camera smoke detection/association and full controller decisions are not simulated; this measures the wall estimator given a terrain-looking probe.
- Fixed viewpoint: retreat-eligible events do not mean a retreat was flown.
- Two-successive-near gate is evaluated without executing the intervening retreat.
- Counterfactual 6/3/3 uses current native returns and statistics, not an old binary.
- Query samples within an episode are correlated; Wilson intervals use episodes only.
- Every first map is a history warm-up with zero qualifying rays; separate summaries exclude it and show the eighth-map-only result.
- The aggregate equally weights eight chosen conditions; it is not a field average.

Reproduce from the workspace:

```bash
cd /home/s04fj3/kv_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
KV_RUN_WALL_FALSE_POSITIVE_BENCHMARK=1 \
  install/kv_executive/lib/kv_executive/python_shared_runtime \
  --python /usr/bin/python3 -- -m pytest -q -s \
  src/kv_executive/test/test_smoke_wall_false_positives.py
```
