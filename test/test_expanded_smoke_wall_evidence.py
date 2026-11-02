"""Compare extended wall evidence on identical known terrain-jitter episodes."""

from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import random
import time

import pytest

from test_planner_inputs import (
    _extended_wall_request,
    _extended_wall_scene,
    EXTENDED_WALL_GRID,
    planner_factory as planner_fixture,
)
from test_smoke_wall_false_positives import (
    _publish_and_query,
    DISTANCES,
    EPISODES_PER_GROUP,
    FRAMES_PER_EPISODE,
    JITTER_HALF_WIDTHS,
    RESOLUTION,
    SEED,
    SLOPES,
)


planner_factory = planner_fixture
pytestmark = pytest.mark.skipif(
    os.environ.get('KV_RUN_WALL_FALSE_POSITIVE_BENCHMARK') != '1',
    reason='Opt-in synthetic terrain-jitter benchmark',
)


def _summary(episodes, field):
    near_field = field.replace('candidate', 'near')
    return {
        'episodes_with_candidate': sum(any(row[field] for row in episode['observations'])
                                       for episode in episodes),
        'accepted_queries': sum(row[field] for episode in episodes
                                for row in episode['observations']),
        'episodes_with_two_successive_near_candidates': sum(any(
            a[field] and b[field] and a[near_field] and b[near_field]
            for a, b in zip(episode['observations'], episode['observations'][1:]))
            for episode in episodes),
    }


def test_expanded_routes_reject_known_terrain_with_and_without_registration_jitter(
        planner_factory, record_testsuite_property):
    """Exercise production 49-ray evidence, including the zero-flicker fallback."""
    planner = planner_factory()
    rng = random.Random(SEED)
    episodes, durations = [], []
    started = time.perf_counter()
    for slope in SLOPES:
        for jitter in JITTER_HALF_WIDTHS:
            for repetition in range(EPISODES_PER_GROUP):
                distance = DISTANCES[repetition % len(DISTANCES)]
                phase = rng.uniform(-RESOLUTION / 2.0, RESOLUTION / 2.0)
                frame = f'expanded_terrain_episode_{len(episodes)}'
                observations = []
                for _ in range(FRAMES_PER_EPISODE):
                    translation = rng.uniform(-jitter, jitter)
                    _publish_and_query(planner, frame, _extended_wall_scene(
                        slope=slope, distance=distance, phase=phase, translation=translation))
                    reference_request = _extended_wall_request(
                        planner, frame, reject_terrain=False)
                    reference = planner.query_geometry(reference_request)
                    guard_request = _extended_wall_request(planner, frame)
                    query_started = time.perf_counter()
                    guarded = planner.query_geometry(guard_request)
                    durations.append(time.perf_counter() - query_started)
                    assert reference.success and guarded.success, guarded.message
                    assert reference.map_header == guarded.map_header
                    observations.append({
                        'map_stamp_ns': (guarded.map_header.stamp.sec * 1_000_000_000
                                         + guarded.map_header.stamp.nanosec),
                        'registration_shift_m': translation,
                        'reference_candidate': bool(reference.smoke_wall_valid),
                        'guarded_candidate': bool(guarded.smoke_wall_valid),
                        'reference_near': bool(reference.smoke_wall_valid and (
                            reference.smoke_wall_nearest_range
                            - reference.smoke_wall_uncertainty <= 20.0)),
                        'guarded_near': bool(guarded.smoke_wall_valid and (
                            guarded.smoke_wall_nearest_range
                            - guarded.smoke_wall_uncertainty <= 20.0)),
                        'reference_evidence': int(reference.smoke_wall_evidence),
                        'guarded_evidence': int(guarded.smoke_wall_evidence),
                        'guarded_support': int(guarded.smoke_wall_samples),
                        'guarded_mixed_support': int(guarded.smoke_wall_mixed_samples),
                        'query_seconds': durations[-1],
                        'diagnostic': guarded.message,
                    })
                episodes.append({
                    'slope': slope, 'jitter_half_width_m': jitter,
                    'distance_m': distance, 'voxel_phase_m': phase,
                    'observations': observations,
                })

    source = Path(__file__).resolve().parents[1] / 'src' / 'planner_node.cpp'
    summary = {
        'generated_utc': datetime.now(timezone.utc).isoformat(),
        'native_source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
        'seed': SEED, 'episodes': len(episodes),
        'maps_per_episode': FRAMES_PER_EPISODE,
        'queries_per_mode': len(durations),
        'grid': [7, 7], 'rays': len(EXTENDED_WALL_GRID), 'width_m': 4.5,
        'wall_front_selection': 'Nearest independently qualified depth group, maximum 4 m span',
        'resolution_m': RESOLUTION,
        'reference': _summary(episodes, 'reference_candidate'),
        'guarded': _summary(episodes, 'guarded_candidate'),
        'query_max_seconds': max(durations),
        'query_p95_seconds': sorted(durations)[int(0.95 * (len(durations) - 1))],
        'elapsed_seconds': time.perf_counter() - started,
        'limitations': [
            'Synthetic no-smoke terrain scenes; not an operational false-positive rate.',
            'All 49 lanes target a single inclined terrain surface.',
            'A generous camera rectangle deliberately includes every terrain return.',
            'Reference disables terrain rejection, which also disables persistent evidence.',
            'Two-successive-near counts describe candidates, not executed controller actions.',
            'The controller additionally retains a pending vote across missing probes.',
            'Camera detection, sensor fusion, wind and vehicle motion are not simulated.',
            'Measured service latency is local to this test machine and workload.',
        ],
    }
    directory = Path(os.environ.get(
        'KV_WALL_FALSE_POSITIVE_REPORT_DIR', str(Path(__file__).parent / 'reports')))
    directory.mkdir(parents=True, exist_ok=True)
    report = directory / 'smoke_wall_grouped_evidence_20261102.json'
    report.write_text(json.dumps(
        {'summary': summary, 'episodes': episodes}, indent=2, allow_nan=False) + '\n')
    record_testsuite_property('expanded_probe_p95_seconds', summary['query_p95_seconds'])
    record_testsuite_property('expanded_probe_max_seconds', summary['query_max_seconds'])
    print(json.dumps(summary, indent=2))
    assert summary['reference']['accepted_queries'] > 0
    assert summary['guarded']['accepted_queries'] == 0
