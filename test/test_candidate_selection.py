"""Choose novel source components without reusing completed-cluster guidance."""

from copy import deepcopy
import math

from geometry_msgs.msg import Point
import kv_executive.inspection_evidence as evidence_module
from kv_executive.inspection_evidence import EVIDENCE_DEFAULTS, InspectionEvidenceMixin
from kv_executive.mission_executive import MissionExecutive
from kv_executive.msg import InspectionReport
import kv_executive.verification as verification
from kv_executive.verification import VerifyController
from kv_perception.msg import (
    FireDetection, FireDetectionArray, SmokeDetection, SmokeDetectionArray,
)
import numpy as np
import pytest
from sensor_msgs.msg import Image
from test_verification import Harness, header
from yasmin import Blackboard


class CandidateHarness(InspectionEvidenceMixin, Harness):
    """Use actual executive callback wiring without starting a ROS participant."""

    _smoke_detection_callback = MissionExecutive._smoke_detection_callback
    _fire_detection_callback = MissionExecutive._fire_detection_callback
    _detection_candidates_callback = MissionExecutive._detection_candidates_callback

    def __init__(self):
        super().__init__()
        self.config.update(EVIDENCE_DEFAULTS)
        self.config['local_frame'] = 'px4_odom'
        self.allow_completed = True
        self.regions = []
        self._inspection_evidence_init()

    def _verify_candidate_is_eligible(self, message, source):
        return self.allow_completed or message.yaw_correction_rad < -0.4

    def _completed_cluster_regions(self):
        return self.regions

    def _completed_detection_ray(self, message, source):
        if message is None:
            return None
        yaw = message.yaw_correction_rad
        return Point(), (math.cos(yaw), math.sin(yaw), 0.0), Point()


@pytest.fixture
def system(monkeypatch):
    executive = CandidateHarness()
    monkeypatch.setattr(verification.time, 'monotonic', lambda: executive.now)
    monkeypatch.setattr(evidence_module.time, 'monotonic', lambda: executive.now)
    controller = VerifyController(executive)
    executive._verify = controller
    executive.now += 0.1
    return executive, controller


def observation(executive, source, *, yaw=0.0, track=1, proven=True, stamp=None):
    stamp = executive.now if stamp is None else stamp
    if source == 'smoke':
        return SmokeDetection(
            header=header(stamp, 'rgb_camera'), detected=True,
            status=SmokeDetection.CONFIRMED, background_ready=True,
            confirmation_frames=3, coverage=0.02, regions=[1], region=1,
            yaw_correction_rad=float(yaw), angular_bounds_valid=True,
            yaw_min_rad=float(yaw - 0.05), yaw_max_rad=float(yaw + 0.05),
            vertical_min_rad=-0.05, vertical_max_rad=0.05,
        )
    return FireDetection(
        header=header(stamp, 'thermal_camera'), detected=True, status=FireDetection.CONFIRMED,
        image_width=100, image_height=100, area_px=100, center_x=50.0, center_y=50.0,
        bbox_x=45, bbox_y=45, bbox_width=10, bbox_height=10,
        track_id=track, source_count=2, confirmation_frames=5 if proven else 0,
        persistence_sec=2.0 if proven else 0.0, yaw_correction_rad=float(yaw),
    )


def batch(executive, source, detections=(), *, stamp=None):
    cls = SmokeDetectionArray if source == 'smoke' else FireDetectionArray
    frame = 'rgb_camera' if source == 'smoke' else 'thermal_camera'
    return cls(header=header(executive.now if stamp is None else stamp, frame),
               detections=list(detections))


def receive_primary(executive, message, source):
    callback = (executive._smoke_detection_callback if source == 'smoke'
                else executive._fire_detection_callback)
    callback(message)


def fresh(controller, source):
    return controller.fresh_smoke() if source == 'smoke' else controller.fresh_fire()


@pytest.mark.parametrize('source', ['smoke', 'fire'])
def test_legacy_primary_remains_available_without_candidate_feed(system, source):
    executive, controller = system
    primary = observation(executive, source)
    receive_primary(executive, primary, source)
    assert fresh(controller, source) == primary
    assert controller.raw_candidates(source) == [primary]
    if source == 'fire':
        assert controller.stable_fire() == primary


@pytest.mark.parametrize('source', ['smoke', 'fire'])
def test_array_primary_preserves_selected_observation_without_double_counting(system, source):
    executive, controller = system
    primary = observation(executive, source)
    receive_primary(executive, primary, source)
    accepted = controller._observation_info[source]['accepted']
    executive._detection_candidates_callback(batch(executive, source, (primary,)), source)
    assert fresh(controller, source) == primary
    assert controller._observation_info[source]['accepted'] == accepted
    if source == 'fire':
        assert controller._fire_frames == 1


@pytest.mark.parametrize('source', ['smoke', 'fire'])
def test_completed_primary_cannot_hide_a_distinct_novel_component(system, source):
    executive, controller = system
    executive.allow_completed = False
    primary = observation(executive, source)
    novel = observation(executive, source, yaw=-0.7, track=2)
    receive_primary(executive, primary, source)
    assert fresh(controller, source) is None
    executive._detection_candidates_callback(batch(executive, source, (primary, novel)), source)
    assert fresh(controller, source) == novel
    assert controller.raw_candidates(source) == [primary, novel]
    if source == 'fire':
        assert controller.stable_fire() == novel


@pytest.mark.parametrize('source', ['smoke', 'fire'])
def test_identical_stamp_empty_array_revokes_selected_candidate(system, source):
    executive, controller = system
    selected = observation(executive, source)
    executive._detection_candidates_callback(batch(executive, source, (selected,)), source)
    assert fresh(controller, source) == selected
    executive._detection_candidates_callback(batch(executive, source), source)
    assert fresh(controller, source) is None
    assert controller.raw_candidates(source) == []
    executive.now += 0.1
    newer_primary = observation(executive, source)
    receive_primary(executive, newer_primary, source)
    assert fresh(controller, source) is None


def test_duplicate_and_reordered_fire_arrays_cannot_add_consumer_persistence(system):
    executive, controller = system
    selected = observation(executive, 'fire', proven=False)
    original = batch(executive, 'fire', (selected,))
    executive._detection_candidates_callback(original, 'fire')
    assert controller._fire_frames == 1
    assert controller.stable_fire() is None
    executive.now += 0.1
    executive._detection_candidates_callback(deepcopy(original), 'fire')
    older = observation(executive, 'fire', proven=False, stamp=executive.now - 0.15)
    executive._detection_candidates_callback(
        batch(executive, 'fire', (older,), stamp=executive.now - 0.15), 'fire')
    assert controller._fire_frames == 1
    assert fresh(controller, 'fire') == selected
    assert controller.stable_fire() is None
    current = observation(executive, 'fire', proven=False)
    executive._detection_candidates_callback(batch(executive, 'fire', (current,)), 'fire')
    assert controller._fire_frames == 2
    assert controller.stable_fire() is None


@pytest.mark.parametrize('source', ['smoke', 'fire'])
def test_current_array_sticks_to_novel_selection_when_candidate_order_changes(system, source):
    executive, controller = system
    executive.allow_completed = False
    completed = observation(executive, source)
    novel = observation(executive, source, yaw=-0.7, track=2)
    executive._detection_candidates_callback(batch(executive, source, (completed, novel)), source)
    executive.now += 0.1
    novel = observation(executive, source, yaw=-0.7, track=2)
    alternative = observation(executive, source, yaw=-1.0, track=3)
    executive._detection_candidates_callback(
        batch(executive, source, (alternative, novel)), source)
    assert fresh(controller, source) == novel


def test_bound_novel_target_loss_does_not_fall_back_to_an_old_smoke_glimpse(system):
    executive, controller = system
    completed_smoke = observation(executive, 'smoke')
    receive_primary(executive, completed_smoke, 'smoke')
    novel_fire = observation(executive, 'fire', yaw=-0.7, track=2)
    receive_primary(executive, novel_fire, 'fire')
    executive.allow_completed = False
    executive.regions = [object()]
    controller.bind_inspection_target()
    assert controller._active_target is not None
    executive._detection_candidates_callback(batch(executive, 'fire'), 'fire')
    executive._detection_candidates_callback(
        batch(executive, 'smoke', (completed_smoke,)), 'smoke')
    assert controller.stable_fire() is None
    assert controller.fresh_smoke() is None
    assert controller._smoke_guidance() is None
    assert not any(event[0] in ('rotate', 'plan', 'navigation') for event in executive.events)


def test_completed_fire_confirmation_cannot_seed_a_height_leg_for_new_target(system):
    executive, controller = system
    completed = observation(executive, 'fire')
    receive_primary(executive, completed, 'fire')
    assert controller.had_fire_confirmation()
    executive.allow_completed = False
    executive._detection_candidates_callback(batch(executive, 'fire', (completed,)), 'fire')
    board = Blackboard()
    controller._capture_height_hint(board)
    try:
        hint = board['verify_height_hint']
    except (KeyError, RuntimeError):
        hint = None
    assert hint is None
    assert not controller._inspection_fire_event(board)
    assert not any(event[0] in ('rotate', 'plan', 'navigation') for event in executive.events)


@pytest.mark.parametrize('source', ['smoke', 'fire'])
def test_bound_target_loss_does_not_switch_to_an_unrelated_novel_direction(system, source):
    executive, controller = system
    executive.allow_completed = False
    executive.regions = [object()]
    chosen = observation(executive, source, yaw=-0.7, track=2)
    executive._detection_candidates_callback(batch(executive, source, (chosen,)), source)
    controller.bind_inspection_target()
    assert controller._active_target is not None
    executive.now += 0.1
    unrelated = observation(executive, source, yaw=-1.4, track=3)
    executive._detection_candidates_callback(batch(executive, source, (unrelated,)), source)
    assert fresh(controller, source) is None


@pytest.mark.parametrize('source', ['smoke', 'fire'])
def test_bound_target_holds_through_missing_image_time_pose_then_recovers(
    system, monkeypatch, source,
):
    executive, controller = system
    executive.allow_completed = False
    executive.regions = [object()]
    chosen = observation(executive, source, yaw=-0.7, track=2)
    executive._detection_candidates_callback(batch(executive, source, (chosen,)), source)
    controller.bind_inspection_target()
    assert controller._active_target is not None
    source_ray = executive._completed_detection_ray
    monkeypatch.setattr(executive, '_completed_detection_ray', lambda *_: None)
    executive.now += 0.1
    unassociated = observation(executive, source, yaw=-0.7, track=2)
    executive._detection_candidates_callback(
        batch(executive, source, (unassociated,)), source)
    assert fresh(controller, source) is None
    assert controller.raw_candidates(source) == [unassociated]
    if source == 'fire':
        assert controller.stable_fire() is None
        assert not controller._inspection_fire_event(Blackboard())
    assert not any(event[0] in ('rotate', 'plan', 'navigation') for event in executive.events)
    monkeypatch.setattr(executive, '_completed_detection_ray', source_ray)
    executive.now += 0.1
    recovered = observation(executive, source, yaw=-0.7, track=2)
    executive._detection_candidates_callback(batch(executive, source, (recovered,)), source)
    assert fresh(controller, source) == recovered


@pytest.mark.parametrize('callback_order', ['primary_first', 'array_first'])
def test_novel_rgb_metadata_wins_over_primary_with_same_image_stamp(system, callback_order):
    executive, controller = system
    executive.allow_completed = False
    primary = observation(executive, 'smoke')
    novel = observation(executive, 'smoke', yaw=-0.7)
    array = batch(executive, 'smoke', (primary, novel))
    callbacks = [lambda: receive_primary(executive, primary, 'smoke'),
                 lambda: executive._detection_candidates_callback(array, 'smoke')]
    for callback in callbacks if callback_order == 'primary_first' else reversed(callbacks):
        callback()
    rgb = np.zeros((8, 8, 3), dtype=np.uint8)
    executive._inspection_rgb_image_callback(Image(
        header=deepcopy(novel.header), encoding='rgb8', width=8, height=8,
        step=24, data=rgb.tobytes()))
    metadata = executive._inspection_evidence_detections['smoke']
    assert len(metadata) == 1
    assert metadata[0].yaw_min_rad == novel.yaw_min_rad
    selected = executive._inspection_evidence_select(
        None, 0, executive._inspection_evidence_limits())['smoke']
    assert selected[1].yaw_min_rad == novel.yaw_min_rad
    report = InspectionReport()
    executive.config['inspection_evidence_timeout'] = 0.1
    executive._inspection_capture_evidence(report)
    assert report.rgb_evidence.valid and report.rgb_evidence.detection_valid
    assert report.rgb_evidence.image.header == novel.header
    assert report.rgb_evidence.yaw_min_rad == novel.yaw_min_rad
    assert not report.thermal_evidence.valid


def _receive_source_channel(executive, message, source, channel):
    if channel == 'primary':
        receive_primary(executive, message, source)
    else:
        cls = SmokeDetectionArray if source == 'smoke' else FireDetectionArray
        executive._detection_candidates_callback(cls(
            header=deepcopy(message.header), detections=[message]), source)


def _bind_source_target(executive, controller, source, channel):
    executive.allow_completed = False
    executive.regions = [object()]
    chosen = observation(executive, source, yaw=-0.7, track=2)
    _receive_source_channel(executive, chosen, source, channel)
    controller.bind_inspection_target()
    assert controller._active_target is not None


@pytest.mark.parametrize('source', ['smoke', 'fire'])
@pytest.mark.parametrize('channel', ['primary', 'array'])
def test_later_pose_accepts_same_cached_image_without_another_camera_callback(
    system, monkeypatch, source, channel,
):
    executive, controller = system
    _bind_source_target(executive, controller, source, channel)
    source_ray = executive._completed_detection_ray
    monkeypatch.setattr(executive, '_completed_detection_ray', lambda *_: None)
    executive.now += 0.1
    pending = observation(executive, source, yaw=-0.7, track=2)
    received = executive.now
    _receive_source_channel(executive, pending, source, channel)
    assert fresh(controller, source) is None
    assert controller.raw_candidates(source) == [pending]
    monkeypatch.setattr(executive, '_completed_detection_ray', source_ray)
    executive.now += 0.1
    assert fresh(controller, source) == pending
    accepted = controller._observation_info[source]['accepted']
    frames = controller._fire_frames
    original_receipt = controller._observations[source][2]
    assert original_receipt == received
    for _ in range(5):
        executive.now += 0.05
        assert fresh(controller, source) == pending
        if source == 'fire':
            assert controller.stable_fire() == pending
    assert controller._observation_info[source]['accepted'] == accepted
    assert controller._fire_frames == frames
    assert controller._observations[source][2] == original_receipt


@pytest.mark.parametrize('source', ['smoke', 'fire'])
@pytest.mark.parametrize('channel', ['primary', 'array'])
def test_source_stale_before_pose_arrival_is_never_revived(system, monkeypatch, source, channel):
    executive, controller = system
    _bind_source_target(executive, controller, source, channel)
    source_ray = executive._completed_detection_ray
    monkeypatch.setattr(executive, '_completed_detection_ray', lambda *_: None)
    executive.now += 0.1
    pending = observation(executive, source, yaw=-0.7, track=2)
    _receive_source_channel(executive, pending, source, channel)
    assert fresh(controller, source) is None
    executive.now += controller.config['verify_observation_timeout'] + 0.1
    monkeypatch.setattr(executive, '_completed_detection_ray', source_ray)
    assert fresh(controller, source) is None
    assert controller.raw_candidates(source) == []
    assert not any(event[0] in ('rotate', 'plan', 'navigation') for event in executive.events)


@pytest.mark.parametrize('source', ['smoke', 'fire'])
@pytest.mark.parametrize('channel', ['primary', 'array'])
def test_delayed_pose_does_not_renew_receipt_age_when_source_clock_is_frozen(
    system, monkeypatch, source, channel,
):
    executive, controller = system
    _bind_source_target(executive, controller, source, channel)
    executive.now += 0.1
    wall = [executive.now]
    monkeypatch.setattr(verification.time, 'monotonic', lambda: wall[0])
    source_ray = executive._completed_detection_ray
    monkeypatch.setattr(executive, '_completed_detection_ray', lambda *_: None)
    pending = observation(executive, source, yaw=-0.7, track=2)
    _receive_source_channel(executive, pending, source, channel)
    assert fresh(controller, source) is None
    wall[0] += 1.5
    monkeypatch.setattr(executive, '_completed_detection_ray', source_ray)
    assert fresh(controller, source) == pending
    assert controller._observations[source][2] == pytest.approx(wall[0] - 1.5)
    wall[0] += 0.6
    assert controller._valid_stamp(pending) is not None  # Source clock remains frozen.
    assert fresh(controller, source) is None
    if source == 'fire':
        assert controller.stable_fire() is None
        wall[0] += controller.config['verify_reacquire_observation_timeout']
        assert not controller._inspection_fire_event(Blackboard())


def test_pose_retry_replaces_primary_rgb_metadata_with_selected_same_image_component(
    system, monkeypatch,
):
    executive, controller = system
    _bind_source_target(executive, controller, 'smoke', 'primary')
    source_ray = executive._completed_detection_ray
    monkeypatch.setattr(executive, '_completed_detection_ray', lambda *_: None)
    executive.now += 0.1
    primary = observation(executive, 'smoke')
    novel = observation(executive, 'smoke', yaw=-0.7)
    receive_primary(executive, primary, 'smoke')
    executive._detection_candidates_callback(batch(executive, 'smoke', (primary, novel)), 'smoke')
    pixels = np.zeros((8, 8, 3), dtype=np.uint8)
    executive._inspection_rgb_image_callback(Image(
        header=deepcopy(novel.header), encoding='rgb8', width=8, height=8,
        step=24, data=pixels.tobytes()))
    assert controller.fresh_smoke() is None
    metadata = executive._inspection_evidence_detections['smoke'][-1]
    assert metadata.yaw_min_rad == primary.yaw_min_rad
    monkeypatch.setattr(executive, '_completed_detection_ray', source_ray)
    assert controller.fresh_smoke() == novel
    assert executive._inspection_evidence_detections['smoke'][-1].yaw_min_rad == novel.yaw_min_rad
    receive_primary(executive, primary, 'smoke')
    assert executive._inspection_evidence_detections['smoke'][-1].yaw_min_rad == novel.yaw_min_rad
    selected = executive._inspection_evidence_select(
        None, 0, executive._inspection_evidence_limits())['smoke']
    assert selected[0].header == novel.header
    assert selected[1].yaw_min_rad == novel.yaw_min_rad


@pytest.mark.parametrize('channel', ['primary', 'array'])
def test_stable_fire_poll_alone_retries_pending_source_frame(system, monkeypatch, channel):
    executive, controller = system
    _bind_source_target(executive, controller, 'fire', channel)
    source_ray = executive._completed_detection_ray
    monkeypatch.setattr(executive, '_completed_detection_ray', lambda *_: None)
    executive.now += 0.1
    pending = observation(executive, 'fire', yaw=-0.7, track=2)
    _receive_source_channel(executive, pending, 'fire', channel)
    assert controller.stable_fire() is None
    monkeypatch.setattr(executive, '_completed_detection_ray', source_ray)
    assert controller.stable_fire() == pending
    frames = controller._fire_frames
    assert controller.stable_fire() == pending
    assert controller._fire_frames == frames
