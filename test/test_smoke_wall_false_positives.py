"""Opt-in native smoke-wall sensitivity benchmark on known non-smoke terrain.

Run with KV_RUN_WALL_FALSE_POSITIVE_BENCHMARK=1. These conditional synthetic
rates measure the effect of map registration jitter, not real mission prevalence.
"""

from collections import defaultdict
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import random
import re
import struct
import subprocess
import time

from geometry_msgs.msg import PointStamped, Vector3Stamped
from kv_executive.srv import QueryInspectionGeometry
from octomap_msgs.msg import Octomap
import pytest

from test_planner_inputs import planner_factory as planner_fixture


SEED = 672_202_611_02
RESOLUTION = 0.2
EPISODES_PER_GROUP = 30
FRAMES_PER_EPISODE = 8
JITTER_HALF_WIDTHS = (0.0, 0.025, 0.05, 0.10)
SLOPES = (1.0, 2.0)
DISTANCES = (8.0, 16.0, 24.0)
ORIGIN = (0.1, 0.1, 6.1)
RAY_GRID = tuple((row, column) for row in range(-2, 3) for column in range(-2, 3))
planner_factory = planner_fixture


def _float32(value):
    return struct.unpack('f', struct.pack('f', value))[0]


def _key(coordinate):
    return math.floor(_float32(coordinate) / RESOLUTION) + 32768


def _encode_octomap(leaves):
    """Serialize sparse, observed free/occupied leaf cells in OcTree binary format."""
    root = {}
    for keys, value in leaves.items():
        node = root
        for bit in range(15, -1, -1):
            child = sum(((key >> bit) & 1) << axis for axis, key in enumerate(keys))
            if bit:
                node = node.setdefault(child, {})
            else:
                node[child] = value
    data = []

    def encode(node):
        bits = sum((3 if isinstance(value, dict) else value) << (2 * index)
                   for index, value in node.items())
        for value in (bits & 255, bits >> 8):
            data.append(value if value < 128 else value - 256)
        for _, value in sorted(node.items()):
            if isinstance(value, dict):
                encode(value)

    encode(root)
    return data


def _terrain_map(distance, slope, phase, translation, *, free_surface=False):
    """Map a static embankment with a uniform depth registration error.

    Twenty-five ray lanes model sparse observations of a continuous planar
    embankment. Each lane marks observed cells before its return free, its return
    occupied, and leaves cells behind the return unknown. The physical surface
    stays fixed; translation is solely an error in its mapped depth.
    """
    leaves = {}
    for row, column in RAY_GRID:
        y = _float32(_float32(ORIGIN[1]) + column * 0.75)
        z = _float32(_float32(ORIGIN[2]) + row * 0.75)
        depth = distance + phase + row * 0.75 / slope + column * 0.75 * 0.15
        base = _key(ORIGIN[0] + depth)
        hit = _key(ORIGIN[0] + depth + translation)
        # +/-10 cm perturbations can change at most one 20 cm voxel. Retain
        # all possible observed free cells relevant to a later jittered return.
        for x in range(base - 2, hit + 1):
            leaves[(x, _key(y), _key(z))] = 1 if x < hit or free_surface else 2
    return _encode_octomap(leaves)


def _request(frame):
    request = QueryInspectionGeometry.Request()
    request.origin = PointStamped()
    request.origin.header.frame_id = frame
    request.origin.point.x, request.origin.point.y, request.origin.point.z = ORIGIN
    request.direction = Vector3Stamped()
    request.direction.header.frame_id = frame
    request.direction.vector.x = 1.0
    request.max_range = 40.0
    request.smoke_wall_width = 3.0
    return request


def _publish_and_query(planner, frame, data):
    message = Octomap()
    message.header.frame_id = frame
    message.header.stamp = planner.node.get_clock().now().to_msg()
    message.binary = True
    message.id = 'OcTree'
    message.resolution = RESOLUTION
    message.data = data
    planner.map_publisher.publish(message)
    response = None

    def current_response():
        nonlocal response
        response = planner.query_geometry(_request(frame))
        return response.map_header == message.header

    planner.wait_for(current_response, 'the source-stamped benchmark map')
    assert response.success, response.message
    return response


def _observation(response):
    coverage = re.search(r'rows=(\d+), columns=(\d+)', response.message)
    assert coverage, response.message
    rows, columns = map(int, coverage.groups())
    native = bool(response.smoke_wall_valid)
    mean = float(response.smoke_wall_range) if native else None
    nearest = float(response.smoke_wall_nearest_range) if native else None
    uncertainty = float(response.smoke_wall_uncertainty) if native else None
    # These are the Python client's range/statistics gates, applied to the
    # actual native response. Metadata is source-fresh and in one fixed frame.
    accepted = native and all(math.isfinite(v) for v in (mean, nearest, uncertainty)) and (
        0.0 < nearest <= mean <= 40.0 and 0.0 <= uncertainty <= 2.0)
    strict = accepted and response.smoke_wall_samples >= 6 and rows >= 3 and columns >= 3
    conservative = nearest - uncertainty if accepted else None
    stamp = response.map_header.stamp
    return {
        'stamp_ns': stamp.sec * 1_000_000_000 + stamp.nanosec,
        'native_qualified': native,
        'qualifying_rays': int(response.smoke_wall_samples),
        'rows': rows, 'columns': columns,
        'mean_m': mean, 'nearest_m': nearest, 'uncertainty_m': uncertainty,
        'conservative_range_m': conservative,
        'accepted_3_2_2': bool(accepted),
        'counterfactual_6_3_3': bool(strict),
    }


def _episode_events(observations, rule):
    streak = 0
    confirmed = False
    last_accepted_stamp = 0
    for observation in observations:
        if not observation[rule]:
            streak = 0
            continue
        if observation['stamp_ns'] <= last_accepted_stamp:
            continue
        last_accepted_stamp = observation['stamp_ns']
        near = observation['conservative_range_m'] <= 15.6
        streak = streak + 1 if near else 0
        confirmed = confirmed or streak >= 2
    return {
        'any_accepted': any(o[rule] for o in observations),
        'any_near': any(o[rule] and o['conservative_range_m'] <= 15.6 for o in observations),
        'retreat_eligible': any(
            o[rule] and o['conservative_range_m'] < 15.0 for o in observations),
        'two_successive_near_maps': confirmed,
    }


def _wilson(successes, count):
    z = 1.959963984540054
    rate = successes / count
    denominator = 1.0 + z * z / count
    centre = (rate + z * z / (2 * count)) / denominator
    radius = z * math.sqrt(rate * (1 - rate) / count + z * z / (4 * count * count)) / denominator
    return [max(0.0, centre - radius), min(1.0, centre + radius)]


def _summarize(episodes):
    result = {'episodes': len(episodes), 'queries': sum(len(e['observations']) for e in episodes)}
    for rule in ('accepted_3_2_2', 'counterfactual_6_3_3'):
        events = [_episode_events(episode['observations'], rule) for episode in episodes]
        rule_result = {}
        for event in ('any_accepted', 'any_near', 'retreat_eligible', 'two_successive_near_maps'):
            count = sum(e[event] for e in events)
            rule_result[event] = {'count': count, 'rate': count / len(episodes),
                                  'wilson_95': _wilson(count, len(episodes))}
        count = sum(o[rule] for e in episodes for o in e['observations'])
        rule_result['accepted_queries'] = {'count': count, 'rate': count / result['queries']}
        measured = [o for e in episodes for o in e['observations'][1:]]
        count = sum(o[rule] for o in measured)
        rule_result['accepted_queries_after_first_map'] = {
            'count': count, 'queries': len(measured), 'rate': count / len(measured)}
        count = sum(e['observations'][-1][rule] for e in episodes)
        rule_result['last_frame_accepted'] = {
            'count': count, 'queries': len(episodes), 'rate': count / len(episodes),
            'wilson_95': _wilson(count, len(episodes))}
        result[rule] = rule_result
    return result


def _write_reports(episodes, elapsed):
    directory = Path(os.environ.get(
        'KV_WALL_FALSE_POSITIVE_REPORT_DIR', str(Path(__file__).parent / 'reports')))
    directory.mkdir(parents=True, exist_ok=True)
    stem = directory / 'smoke_wall_false_positives_20261102'
    grouped = defaultdict(list)
    for episode in episodes:
        grouped[(episode['slope'], episode['jitter_half_width_m'])].append(episode)
    groups = [{'slope': slope, 'jitter_half_width_m': jitter,
               **_summarize(values)} for (slope, jitter), values in sorted(grouped.items())]
    commit = subprocess.check_output(
        ['git', '-C', str(Path(__file__).resolve().parents[1]), 'rev-parse', 'HEAD'],
        text=True).strip()
    report = {
        'generated_utc': datetime.now(timezone.utc).isoformat(), 'commit': commit,
        'seed': SEED, 'elapsed_wall_seconds': elapsed, 'resolution_m': RESOLUTION,
        'frames_per_episode': FRAMES_PER_EPISODE, 'episodes_per_group': EPISODES_PER_GROUP,
        'aperture_m': 3.0, 'rays': 25, 'distances_m': DISTANCES,
        'jitter_model': 'Independent uniform depth registration error per map: [-a,+a].',
        'ground_truth': 'Static no-smoke embankment; only mapped depth is perturbed.',
        'limitations': [
            'Conditional synthetic stress-test rates, not real mission false-positive prevalence.',
            'All 25 ray lanes target observed steep terrain. This deliberately conditions on the '
            'terrain-jitter failure mode, not ordinary open-air or smoke-directed views.',
            'Ray-aligned sparse terrain returns; raw sensor noise/dropout and OctoMap fusion are '
            'not simulated.',
            'Camera smoke detection/association and full controller decisions are not simulated; '
            'this measures the wall estimator given a terrain-looking probe.',
            'Fixed viewpoint: retreat-eligible events do not mean a retreat was flown.',
            'Two-successive-near gate is evaluated without executing the intervening retreat.',
            'Counterfactual 6/3/3 uses current native returns and statistics, not an old binary.',
            'Query samples within an episode are correlated; Wilson intervals use episodes only.',
            'Every first map is a history warm-up with zero qualifying rays; separate summaries '
            'exclude it and show the eighth-map-only result.',
            'The aggregate equally weights eight chosen conditions; it is not a field average.',
        ],
        'overall': _summarize(episodes),
        'stable_terrain': _summarize([e for e in episodes if e['jitter_half_width_m'] == 0.0]),
        'jittered_terrain': _summarize([e for e in episodes if e['jitter_half_width_m'] > 0.0]),
        'groups': groups, 'raw_episodes': episodes,
    }
    stem.with_suffix('.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    lines = [
        '# Smoke-wall false-positive sensitivity benchmark', '',
        f'Native planner commit: `{commit}`. Seed: `{SEED}`. Runtime: {elapsed:.2f} s.', '',
        '**These are controlled no-smoke terrain stress tests, not a measured mission rate.**', '',
        f'{len(episodes)} independent episodes, {FRAMES_PER_EPISODE} maps each; '
        '20 cm OctoMap cells and the real 25-ray, 3 × 3 m native estimator.', '',
        'Each episode has a randomized voxel phase and starts with an empty local map history. '
        'Ranges 8, 16 and 24 m are balanced within every group. The terrain stays physically '
        'static while its mapped depth shifts uniformly by the stated amount per frame.', '',
        '| Slope | Jitter ±m | Episodes | Any accepted 3/2/2 | Any accepted 6/3/3 | '
        'Retreat-eligible 3/2/2 | Two near maps 3/2/2 |',
        '|---:|---:|---:|---:|---:|---:|---:|',
    ]
    for group in groups:
        new, old = group['accepted_3_2_2'], group['counterfactual_6_3_3']
        values = (new['any_accepted'], old['any_accepted'], new['retreat_eligible'],
                  new['two_successive_near_maps'])
        formatted = [f'{value["count"]}/{group["episodes"]} ({value["rate"]:.1%})'
                     for value in values]
        lines.append(f'| {math.degrees(math.atan(group["slope"])):.1f}° | '
                     f'{group["jitter_half_width_m"]:.3f} | {group["episodes"]} | '
                     + ' | '.join(formatted) + ' |')
    lines.extend(['', '## Stable and jittered terrain separately', ''])
    for name in ('stable_terrain', 'jittered_terrain'):
        summary = report[name]
        new, old = (summary['accepted_3_2_2']['any_accepted'],
                    summary['counterfactual_6_3_3']['any_accepted'])
        low, high = new['wilson_95']
        lines.append(f'- {name}: {new["count"]}/{summary["episodes"]} episodes '
                     f'({new["rate"]:.1%}) yielded an accepted false wall under 3/2/2 '
                     f'(episode Wilson 95% interval {low:.1%}–{high:.1%}); '
                     f'6/3/3 counterfactual: {old["count"]}/{summary["episodes"]} '
                     f'({old["rate"]:.1%}).')
        for rule in ('accepted_3_2_2', 'counterfactual_6_3_3'):
            after_first = summary[rule]['accepted_queries_after_first_map']
            final = summary[rule]['last_frame_accepted']
            low, high = final['wilson_95']
            lines.append(f'  - {rule}: after excluding the first map, '
                         f'{after_first["count"]}/{after_first["queries"]} '
                         f'({after_first["rate"]:.1%}) queries; eighth map only, '
                         f'{final["count"]}/{final["queries"]} ({final["rate"]:.1%}), '
                         f'episode Wilson 95% interval {low:.1%}–{high:.1%}.')
    lines.extend(['', '## Equally weighted aggregate', ''])
    for rule in ('accepted_3_2_2', 'counterfactual_6_3_3'):
        result = report['overall'][rule]
        lines.append(f'**{rule}**')
        lines.append('')
        for event in ('any_accepted', 'retreat_eligible', 'two_successive_near_maps'):
            value = result[event]
            low, high = value['wilson_95']
            lines.append(f'- {event}: {value["count"]}/{len(episodes)} '
                         f'({value["rate"]:.1%}); episode Wilson 95% interval '
                         f'{low:.1%}–{high:.1%}.')
        value = result['accepted_queries']
        lines.append(f'- accepted_queries: {value["count"]}/{report["overall"]["queries"]} '
                     f'({value["rate"]:.1%}); correlated samples, no independent-query interval.')
        value = result['accepted_queries_after_first_map']
        lines.append(f'- accepted_queries_after_first_map: {value["count"]}/{value["queries"]} '
                     f'({value["rate"]:.1%}); excludes guaranteed history warm-up.')
        value = result['last_frame_accepted']
        low, high = value['wilson_95']
        lines.append(f'- eighth_map_only: {value["count"]}/{value["queries"]} '
                     f'({value["rate"]:.1%}); episode Wilson 95% interval '
                     f'{low:.1%}–{high:.1%}.')
        lines.append('')
    lines.extend(['## Limits', ''])
    lines.extend(f'- {limitation}' for limitation in report['limitations'])
    lines.extend(['', 'Reproduce from the workspace:', '', '```bash',
                  'cd /home/s04fj3/kv_ws',
                  'source /opt/ros/jazzy/setup.bash', 'source install/setup.bash',
                  'KV_RUN_WALL_FALSE_POSITIVE_BENCHMARK=1 \\',
                  '  install/kv_executive/lib/kv_executive/python_shared_runtime \\',
                  '  --python /usr/bin/python3 -- -m pytest -q -s \\',
                  '  src/kv_executive/test/test_smoke_wall_false_positives.py', '```', ''])
    stem.with_suffix('.md').write_text('\n'.join(lines))
    print(f'Wall benchmark report: {stem.with_suffix(".md")}')
    print(json.dumps(report['overall'], sort_keys=True))


def test_episode_confirmation_requires_two_distinct_new_map_timestamps():
    def observed(stamp, accepted=True):
        return {'stamp_ns': stamp, 'accepted_3_2_2': accepted,
                'conservative_range_m': 15.3 if accepted else None}

    first = observed(1)
    assert not _episode_events([first, first], 'accepted_3_2_2')['two_successive_near_maps']
    assert _episode_events([first, observed(2)], 'accepted_3_2_2')['two_successive_near_maps']
    assert not _episode_events([first, observed(2, False), observed(3)],
                               'accepted_3_2_2')['two_successive_near_maps']


@pytest.mark.skipif(os.environ.get('KV_RUN_WALL_FALSE_POSITIVE_BENCHMARK') != '1',
                    reason='Opt-in native terrain-jitter sensitivity benchmark')
def test_static_terrain_registration_jitter_smoke_wall_false_positive_rates(planner_factory):
    started = time.monotonic()
    planner = planner_factory()
    calibration_frame = 'wall_benchmark_positive_control'
    first = _publish_and_query(
        planner, calibration_frame, _terrain_map(8.0, 2.0, 0.0, 0.0, free_surface=True))
    assert not first.smoke_wall_valid
    positive = _publish_and_query(planner, calibration_frame, _terrain_map(8.0, 2.0, 0.0, 0.0))
    assert positive.smoke_wall_valid and positive.smoke_wall_samples == 25, positive.message
    assert _observation(positive)['accepted_3_2_2']

    rng = random.Random(SEED)
    episodes = []
    for slope in SLOPES:
        for jitter in JITTER_HALF_WIDTHS:
            for replicate in range(EPISODES_PER_GROUP):
                frame = f'wall_benchmark_episode_{len(episodes)}'
                distance = DISTANCES[replicate % len(DISTANCES)]
                phase = rng.uniform(-RESOLUTION / 2, RESOLUTION / 2)
                observations, translations = [], []
                for index in range(FRAMES_PER_EPISODE):
                    translation = rng.uniform(-jitter, jitter)
                    response = _publish_and_query(
                        planner, frame, _terrain_map(distance, slope, phase, translation))
                    observation = _observation(response)
                    if index == 0:
                        assert observation['qualifying_rays'] == 0, response.message
                    if observations:
                        assert observation['stamp_ns'] > observations[-1]['stamp_ns']
                    assert not observation['counterfactual_6_3_3'] or (
                        observation['accepted_3_2_2'])
                    observations.append(observation)
                    translations.append(translation)
                if jitter == 0.0:
                    assert not any(o['accepted_3_2_2'] for o in observations)
                episodes.append({
                    'slope': slope, 'jitter_half_width_m': jitter, 'distance_m': distance,
                    'voxel_phase_m': phase, 'translations_m': translations,
                    'observations': observations,
                })
            print(f'Completed slope={slope:g}, jitter=±{jitter:g} m; episodes={len(episodes)}')
    assert len(episodes) == 240
    _write_reports(episodes, time.monotonic() - started)
