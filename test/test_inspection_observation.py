"""Stationary wall observations require post-settle images and map updates."""

from copy import deepcopy
from types import SimpleNamespace

import pytest

from test_inspection_navigation import Harness, header


@pytest.fixture
def executive():
    node = Harness()
    node.smoke_wall_valid = True
    node._verify_wall_observation_context = {'settled_ns': int(99.0 * 1e9)}
    return node


def measure(node):
    return node._verify_measure_wall('rgb_camera', bearing=0.0)


def test_moving_viewpoint_skips_classification_without_waiting_for_a_query(executive):
    executive.wall_observation_ready = False
    assert measure(executive) is None
    assert not any(event[0] == 'query' for event in executive.events)


def test_pre_settle_camera_frame_is_not_replayed_as_current_wall_evidence(executive):
    executive._verify_wall_observation_context['settled_ns'] = int(100.0 * 1e9)
    assert measure(executive) is None
    assert not any(event[0] == 'query' for event in executive.events)


@pytest.mark.parametrize('stamp', [98.5, 99.0])
def test_fresh_but_pre_settle_map_does_not_qualify(executive, stamp):
    def old_map(_request, response):
        response.map_header = header(stamp, 'map')
    executive.on_query = old_map
    assert measure(executive) is None


@pytest.mark.parametrize('change', ['moving', 'new_epoch', 'image_lost'])
def test_observation_change_during_service_wait_withholds_the_result(executive, change):
    def changed(_request, _response):
        if change == 'moving':
            executive.wall_observation_ready = False
        elif change == 'new_epoch':
            executive._verify_wall_observation_context = deepcopy(
                executive._verify_wall_observation_context)
        else:
            executive.smoke.detected = False
    executive.on_query = changed
    assert measure(executive) is None


@pytest.mark.parametrize('route', ['flicker', 'persistent'])
def test_both_routes_accept_fresh_post_settle_inputs(executive, route):
    executive.smoke.angular_bounds_valid = True
    executive.smoke.yaw_min_rad = executive.smoke.vertical_min_rad = -0.5
    executive.smoke.yaw_max_rad = executive.smoke.vertical_max_rad = 0.5

    def route_response(_request, response):
        response.smoke_wall_evidence = 1 if route == 'flicker' else 2
        response.smoke_wall_samples = 3 if route == 'flicker' else 6
        response.smoke_wall_mixed_samples = 1 if route == 'flicker' else 0
        response.smoke_wall_rows = response.smoke_wall_columns = 3
    executive.on_query = route_response
    result = measure(executive)
    assert result is not None and result['evidence'] == route


def _map_wait_clock(node, monkeypatch):
    monkeypatch.setattr('kv_executive.inspection_navigation.time.monotonic', lambda: node.now)
    node._stop = SimpleNamespace(wait=lambda seconds: setattr(node, 'now', node.now + seconds))


def test_post_settle_map_wait_checks_current_clearance_and_new_source_stamp(
        executive, monkeypatch):
    _map_wait_clock(executive, monkeypatch)
    context = executive._verify_wall_observation_context
    context['settled_ns'] = int(executive.now * 1e9)
    assert executive._verify_wait_for_wall_map(context)
    assert context['map_after_epoch'] and context['map_wait_complete']
    requests = [event[1] for event in executive.events if event[0] == 'query']
    assert len(requests) >= 2 and all(len(request.corridor.poses) == 1 for request in requests)


def test_missing_post_settle_map_has_bounded_optional_wait(executive, monkeypatch):
    _map_wait_clock(executive, monkeypatch)
    executive.config['verify_settle_map_timeout'] = 0.2
    executive.on_query = lambda _request, response: setattr(
        response, 'map_header', header(99.0, 'map'))
    context = executive._verify_wall_observation_context
    context['settled_ns'] = int(executive.now * 1e9)
    assert not executive._verify_wait_for_wall_map(context)
    assert executive.now <= 100.21
    assert context['map_wait_complete'] and not context['map_after_epoch']
    assert executive.wall_observation_ready


def test_map_wait_does_not_hide_an_occupied_current_body(executive, monkeypatch):
    from kv_executive.verification import VerificationStartUnavailable
    _map_wait_clock(executive, monkeypatch)
    executive.corridor_safe = False
    with pytest.raises(VerificationStartUnavailable, match='Current body clearance failed'):
        executive._verify_wait_for_wall_map(executive._verify_wall_observation_context)


def test_map_wait_checks_existing_deadlines_instead_of_renewing_them(executive, monkeypatch):
    _map_wait_clock(executive, monkeypatch)

    def expired():
        raise RuntimeError('Original observation deadline expired')
    with pytest.raises(RuntimeError, match='Original observation deadline'):
        executive._verify_wait_for_wall_map(executive._verify_wall_observation_context, expired)
    assert not any(event[0] == 'query' for event in executive.events)
