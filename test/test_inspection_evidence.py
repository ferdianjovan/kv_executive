"""Compressed evidence preserves source pixels, timestamps and bounded resources."""

from copy import deepcopy
import math
from types import SimpleNamespace

import cv2
from geometry_msgs.msg import Point
import kv_executive.inspection_evidence as evidence_module
from kv_executive.inspection_evidence import EVIDENCE_DEFAULTS, InspectionEvidenceMixin
from kv_executive.mission_geometry import quaternion_from_yaw, yaw_from_quaternion
from kv_executive.msg import InspectionReport
from nav_msgs.msg import Odometry
import numpy as np
import pytest
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Image
from std_msgs.msg import Header


def header(stamp=100.0, frame='rgb_camera'):
    result = Header(frame_id=frame)
    result.stamp.sec, result.stamp.nanosec = divmod(round(stamp * 1e9), 1_000_000_000)
    return result


def detection(stamp=100.0, *, source='smoke', detected=True, track=7):
    return SimpleNamespace(
        header=header(stamp, 'rgb_camera' if source == 'smoke' else 'thermal_camera'),
        detected=detected, status=3 if source == 'smoke' else 2, track_id=track,
        source_count=2, bbox_x=4, bbox_y=5, bbox_width=6, bbox_height=7,
        angular_bounds_valid=True, yaw_min_rad=-0.2, yaw_max_rad=0.2,
        vertical_min_rad=-0.1, vertical_max_rad=0.1,
    )


def image(pixels, stamp=100.0, *, encoding='rgb8', frame='rgb_camera', padding=0,
          bigendian=False):
    itemsize = 2 if encoding in {'mono16', '16UC1'} else 1
    channels = pixels.shape[2] if pixels.ndim == 3 else 1
    height, width = pixels.shape[:2]
    if itemsize == 2:
        pixels = pixels.astype('>u2' if bigendian else '<u2')
    rows = [row.tobytes() + bytes(padding) for row in pixels]
    return Image(header=header(stamp, frame), width=width, height=height,
                 step=width * channels * itemsize + padding, is_bigendian=int(bigendian),
                 encoding=encoding, data=b''.join(rows))


class Harness(InspectionEvidenceMixin):
    def __init__(self):
        self.now = self.wall = 100.0
        self.config = {**EVIDENCE_DEFAULTS, 'local_frame': 'px4_odom',
                       'verify_future_tolerance': 0.25}
        self.smoke = self.fire = None
        self.on_wait = lambda: None
        self.clock_rate = 1.0
        self.cancel = False
        self._stop = SimpleNamespace(wait=self.wait)
        self._verify = SimpleNamespace(fresh_smoke=lambda: deepcopy(self.smoke),
                                       stable_fire=lambda: deepcopy(self.fire))
        self._inspection_evidence_init()

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=round(self.now * 1e9)))

    def wait(self, duration):
        self.wall += duration
        self.now += duration * self.clock_rate
        self.on_wait()
        return False

    def _check_running(self):
        if self.cancel:
            raise RuntimeError('operator stopped inspection')

    def pair(self, stamp=None, *, rgb=None, thermal=None):
        stamp = self.now if stamp is None else stamp
        rgb = np.full((24, 32, 3), (25, 75, 160), dtype=np.uint8) if rgb is None else rgb
        thermal = np.arange(24 * 32, dtype=np.uint16).reshape(24, 32) if thermal is None \
            else thermal
        self.smoke, self.fire = detection(stamp), detection(stamp, source='fire')
        self._inspection_rgb_image_callback(image(rgb, stamp))
        self._inspection_thermal_image_callback(image(
            thermal, stamp, encoding='mono16', frame='thermal_camera'))
        self._inspection_evidence_detection_callback(self.smoke, 'smoke')
        self._inspection_evidence_detection_callback(self.fire, 'fire')
        return rgb, thermal

    def odom(self, stamp, x, *, yaw=0.0):
        result = Odometry(header=header(stamp, 'px4_odom'))
        result.pose.pose.position.x = float(x)
        result.pose.pose.orientation = quaternion_from_yaw(yaw)
        self._inspection_evidence_odom_callback(result)
        return result


@pytest.fixture
def node(monkeypatch):
    result = Harness()
    monkeypatch.setattr(evidence_module.time, 'monotonic', lambda: result.wall)
    return result


def capture(node, *, positive=True, after_ns=0, guard=None):
    report = InspectionReport(fire_found=positive)
    node._inspection_capture_evidence(report, node.fire if positive else None,
                                      guard=guard, after_ns=after_ns)
    return report


def decoded(evidence):
    return cv2.imdecode(np.frombuffer(bytes(evidence.image.data), dtype=np.uint8),
                        cv2.IMREAD_UNCHANGED)


def test_both_snapshots_have_matching_detection_and_capture_provenance(node):
    rgb, thermal = node.pair()
    node.odom(99.9, 4.0, yaw=math.radians(170))
    node.odom(100.1, 6.0, yaw=math.radians(-170))
    report = capture(node)
    assert report.evidence_complete and report.fire_found
    assert report.rgb_evidence.image.header == node.smoke.header
    assert report.thermal_evidence.image.header == node.fire.header
    assert np.array_equal(decoded(report.thermal_evidence), thermal)
    assert np.max(np.abs(decoded(report.rgb_evidence).astype(int) - rgb[..., ::-1])) <= 3
    for evidence in (report.rgb_evidence, report.thermal_evidence):
        assert evidence.valid and evidence.detection_valid and evidence.observer_pose_valid
        assert evidence.detection_header == evidence.image.header
        assert evidence.observer_local_pose.header.stamp == evidence.image.header.stamp
        assert evidence.observer_local_pose.header.frame_id == 'px4_odom'
        assert evidence.observer_local_pose.pose.position == Point(x=5.0)
        assert abs(yaw_from_quaternion(evidence.observer_local_pose.pose.orientation)) \
            == pytest.approx(math.pi)
    assert report.thermal_evidence.track_id == 7
    assert report.thermal_evidence.source_count == 2
    assert report.thermal_evidence.bbox_width == 6
    assert report.rgb_evidence.angular_bounds_valid


@pytest.mark.parametrize('encoding,bigendian', [
    ('mono16', False), ('mono16', True), ('16UC1', True),
])
def test_thermal_png_preserves_original_sixteen_bit_pixels_and_padding(node, encoding, bigendian):
    thermal = np.array([[0, 255, 256, 65535], [32768, 1024, 20, 42000]], dtype=np.uint16)
    node.pair(thermal=thermal)
    node.now = 100.1
    node.fire = detection(100.1, source='fire')
    node._inspection_thermal_image_callback(image(
        thermal, 100.1, encoding=encoding, frame='thermal_camera', padding=4,
        bigendian=bigendian))
    report = capture(node)
    assert report.thermal_evidence.valid
    assert report.thermal_evidence.original_encoding == encoding
    assert np.array_equal(decoded(report.thermal_evidence), thermal)


def test_rgb_noise_is_resized_and_compressed_within_the_encoded_byte_budget(node):
    rgb = np.random.default_rng(2094).integers(0, 256, (1080, 1920, 3), dtype=np.uint8)
    node.config['inspection_evidence_rgb_max_bytes'] = 2048
    node.pair(rgb=rgb)
    report = capture(node)
    evidence = report.rgb_evidence
    assert evidence.valid and len(evidence.image.data) <= 2048
    assert evidence.original_width == 1920 and evidence.original_height == 1080
    assert evidence.stored_width <= 1280 and evidence.stored_height < 1080
    assert decoded(evidence).shape[:2] == (evidence.stored_height, evidence.stored_width)


def test_full_resolution_report_serializes_with_bounded_compressed_payload(node):
    rng = np.random.default_rng(11042026)
    rgb = rng.integers(0, 256, (1080, 1920, 3), dtype=np.uint8)
    thermal = rng.integers(0, 65536, (512, 640), dtype=np.uint16)
    node.pair(rgb=rgb, thermal=thermal)
    report = capture(node)
    assert report.evidence_complete
    assert report.rgb_evidence.image.format == 'rgb8; jpeg compressed bgr8'
    assert report.thermal_evidence.image.format == 'mono16; png compressed mono16'
    assert np.array_equal(decoded(report.thermal_evidence), thermal)
    assert len(serialize_message(report)) <= (
        node.config['inspection_evidence_rgb_max_bytes']
        + node.config['inspection_evidence_thermal_max_bytes'] + 16384)


@pytest.mark.parametrize('source', ['smoke', 'fire'])
def test_impossible_payload_budget_is_explicit_without_changing_the_finding(node, source):
    node.pair()
    node.config['inspection_evidence_rgb_max_bytes' if source == 'smoke'
                else 'inspection_evidence_thermal_max_bytes'] = 10
    report = capture(node)
    missing = report.rgb_evidence if source == 'smoke' else report.thermal_evidence
    assert report.fire_found and not report.evidence_complete
    assert not missing.valid and not missing.image.data
    assert 'payload limit' in missing.status


def test_missing_stream_has_a_bounded_wait_and_keeps_the_other_fresh_snapshot(node):
    node._inspection_rgb_image_callback(image(np.zeros((8, 8, 3), dtype=np.uint8)))
    node.smoke = detection()

    def fresh_rgb():
        node.smoke = detection(node.now)
        node._inspection_rgb_image_callback(image(
            np.zeros((8, 8, 3), dtype=np.uint8), node.now))

    node.on_wait = fresh_rgb
    report = capture(node, positive=False)
    assert node.wall == pytest.approx(103.0)
    assert report.rgb_evidence.valid and not report.thermal_evidence.valid
    assert not report.evidence_complete and 'thermal:' in report.evidence_status


def test_negative_thermal_frame_is_original_evidence_with_explicit_negative_metadata(node):
    _, thermal = node.pair()
    node.fire.detected = False
    node.fire.status = 0
    node._inspection_evidence_detections['fire'].clear()
    node._inspection_evidence_detection_callback(node.fire, 'fire')
    node.fire = None
    report = capture(node, positive=False)
    assert report.evidence_complete and not report.fire_found
    assert report.thermal_evidence.detection_valid
    assert not report.thermal_evidence.detected and report.thermal_evidence.detection_status == 0
    assert np.array_equal(decoded(report.thermal_evidence), thermal)


def test_negative_snapshot_without_detection_metadata_is_not_labelled_as_a_detection(node):
    node.pair()
    node._inspection_evidence_detections['fire'].clear()
    report = capture(node, positive=False)
    assert report.thermal_evidence.valid and not report.thermal_evidence.detection_valid


def test_positive_evidence_uses_current_stable_track_and_its_exact_source_image(node):
    node.pair()
    original = deepcopy(node.fire)
    node.now += 0.1
    node.fire = detection(node.now, source='fire')
    pixels = np.full((8, 8), 49152, dtype=np.uint16)
    node._inspection_thermal_image_callback(image(
        pixels, node.now, encoding='mono16', frame='thermal_camera'))
    report = InspectionReport(fire_found=True)
    node._inspection_capture_evidence(report, original)
    assert report.thermal_evidence.image.header == node.fire.header
    assert np.array_equal(decoded(report.thermal_evidence), pixels)


def test_other_fire_track_cannot_supply_positive_evidence(node):
    node.pair()
    original = deepcopy(node.fire)
    node.fire.track_id += 1
    node.config['inspection_evidence_timeout'] = 0.1
    report = InspectionReport(fire_found=True)
    node._inspection_capture_evidence(report, original)
    assert not report.thermal_evidence.valid


def test_rgb_requires_image_stamp_of_the_accepted_smoke_detection(node):
    node.pair()
    node.smoke.header = header(100.1)
    node.config['inspection_evidence_timeout'] = 0.1
    report = capture(node)
    assert not report.rgb_evidence.valid and report.thermal_evidence.valid


def test_fresh_unsynchronised_images_are_retained_without_claiming_a_pair(node):
    node.config['inspection_evidence_pair_slop'] = 0.1
    node.config['inspection_evidence_timeout'] = 0.1
    node.pair()
    node.now = 100.5
    node.fire = detection(100.5, source='fire')
    node._inspection_thermal_image_callback(image(
        np.zeros((8, 8), dtype=np.uint16), 100.5,
        encoding='mono16', frame='thermal_camera'))
    report = capture(node)
    assert report.rgb_evidence.valid and report.thermal_evidence.valid
    assert not report.evidence_complete and 'too far apart' in report.evidence_status


def test_final_view_cutoff_rejects_pre_turn_and_equal_timestamp_images(node):
    node.pair()
    node.config['inspection_evidence_timeout'] = 0.1
    report = capture(node, after_ns=100_000_000_000)
    assert not report.rgb_evidence.valid and not report.thermal_evidence.valid


def test_cancellation_interrupts_an_image_wait_without_waiting_for_timeout(node):
    node.on_wait = lambda: setattr(node, 'cancel', True)
    with pytest.raises(RuntimeError, match='operator stopped'):
        capture(node, positive=False)
    assert node.wall == pytest.approx(100.05)


def test_deadline_guard_remains_authoritative_during_compression(node):
    node.pair()
    checks = []

    def guard():
        checks.append(1)
        if len(checks) > 1:
            raise RuntimeError('inspection deadline')

    with pytest.raises(RuntimeError, match='inspection deadline'):
        capture(node, guard=guard)


def test_newer_detector_frames_during_encoding_do_not_discard_matching_fresh_evidence(
        node, monkeypatch):
    node.pair()
    original = evidence_module.cv2.imencode

    def updating(*args):
        result = original(*args)
        node.now += 0.05
        node.smoke = detection(node.now)
        node.fire = detection(node.now, source='fire')
        return result

    monkeypatch.setattr(evidence_module.cv2, 'imencode', updating)
    report = capture(node)
    assert report.evidence_complete
    assert report.rgb_evidence.image.header.stamp == header().stamp


@pytest.mark.parametrize('rewind', [False, True])
def test_clock_change_or_aging_during_encoding_cannot_publish_stale_pixels(
        node, monkeypatch, rewind):
    node.pair()
    original = evidence_module.cv2.imencode

    def aging(*args):
        result = original(*args)
        node.now = 10.0 if rewind else node.now + 3.0
        return result

    monkeypatch.setattr(evidence_module.cv2, 'imencode', aging)
    report = capture(node)
    assert not report.rgb_evidence.valid and not report.rgb_evidence.image.data
    assert not report.thermal_evidence.valid and not report.thermal_evidence.image.data


def test_duplicate_frames_do_not_extend_receive_freshness_on_a_paused_clock(node):
    node.pair()
    node.clock_rate = 0.0
    node.wall += 2.1
    node._inspection_rgb_image_callback(image(np.zeros((8, 8, 3), dtype=np.uint8)))
    node.config['inspection_evidence_timeout'] = 0.1
    report = capture(node)
    assert not report.rgb_evidence.valid and not report.thermal_evidence.valid


def test_source_clock_rewind_starts_a_clean_image_and_pose_epoch(node):
    node.pair()
    node.odom(100.0, 8.0)
    node.now = 10.0
    node.pair()
    assert all(frame.stamp == 10_000_000_000
               for frames in node._inspection_evidence_frames.values() for frame in frames)
    assert not node._inspection_evidence_poses
    assert capture(node).evidence_complete


def test_new_inspection_episode_clears_old_photos_but_keeps_pose_history(node):
    node.pair()
    node.odom(100.0, 3.0)
    node._inspection_reset_evidence()
    assert node._inspection_evidence_bytes == 0
    assert node._inspection_evidence_poses
    assert node._inspection_evidence_problem['smoke'] == 'no RGB image after episode start'
    node.config['inspection_evidence_timeout'] = 0.1
    assert not capture(node).evidence_complete
    node.now += 0.1
    node.pair()
    assert capture(node).evidence_complete


def test_cache_limits_bound_both_streams_and_detection_history(node):
    node.config['inspection_evidence_cache_bytes'] = 1100
    for index in range(20):
        node.now += 0.01
        node.wall += 0.01
        node.pair(rgb=np.zeros((8, 8, 3), dtype=np.uint8),
                  thermal=np.zeros((8, 8), dtype=np.uint16))
        assert node._inspection_evidence_bytes <= 1100
        assert all(len(items) <= 3 for items in node._inspection_evidence_frames.values())
        assert all(len(items) <= 3 for items in node._inspection_evidence_detections.values())
    assert node._inspection_evidence_bytes == sum(
        len(frame.data) for frames in node._inspection_evidence_frames.values()
        for frame in frames)


def test_oversized_payload_is_rejected_before_copying_it(node):
    class Payload:
        def __len__(self):
            return 9 * 1024 * 1024

        def __bytes__(self):
            raise AssertionError('Oversized data must never be copied')

    bad = SimpleNamespace(header=header(), encoding='rgb8', width=3072, height=1024,
                          step=9216, is_bigendian=False, data=Payload())
    node._inspection_rgb_image_callback(bad)
    assert node._inspection_evidence_bytes == 0


def test_incoming_image_buffer_reuse_cannot_mutate_saved_evidence(node):
    node.pair()
    node.now += 0.1
    node.smoke = detection(node.now)
    value = image(np.full((8, 8, 3), (10, 50, 200), dtype=np.uint8), node.now)
    node._inspection_rgb_image_callback(value)
    for index in range(len(value.data)):
        value.data[index] = 0
    report = capture(node)
    actual = decoded(report.rgb_evidence)
    assert report.rgb_evidence.valid
    assert np.max(np.abs(actual.astype(int) - (200, 50, 10))) <= 3


@pytest.mark.parametrize('bad', ['encoding', 'length', 'step', 'frame', 'zero', 'future', 'stale'])
def test_invalid_image_metadata_or_timestamps_never_enter_the_cache(node, bad):
    value = image(np.zeros((8, 8, 3), dtype=np.uint8))
    if bad == 'encoding':
        value.encoding = '32FC1'
    elif bad == 'length':
        value.data = value.data[:-1]
    elif bad == 'step':
        value.step = 1
    elif bad == 'frame':
        value.header.frame_id = ''
    else:
        value.header = header({'zero': 0.0, 'future': 100.3, 'stale': 97.0}[bad])
    node._inspection_rgb_image_callback(value)
    assert node._inspection_evidence_bytes == 0
    assert not node._inspection_evidence_frames['smoke']


def test_capture_pose_never_extrapolates_or_substitutes_the_current_drone_pose(node):
    node.pair()
    node.odom(99.9, 50.0)
    report = capture(node)
    assert report.evidence_complete and not report.rgb_evidence.observer_pose_valid
    node.now = 101.0
    node.odom(101.0, 100.0)
    report = capture(node)
    assert not report.rgb_evidence.observer_pose_valid  # Bracketing gap exceeds the bound.


def test_capture_waits_briefly_for_both_images_future_odometry_bracket(node):
    node.pair()
    node.odom(99.95, 4.0)
    original_stamp = header().stamp

    def bracket_arrives():
        node.odom(100.05, 6.0)
        # Camera/detection traffic continues; the chosen pair remains fixed.
        node.pair()

    node.on_wait = bracket_arrives
    report = capture(node)
    assert report.evidence_complete
    assert 100.0 < node.wall <= 100.25
    for evidence in (report.rgb_evidence, report.thermal_evidence):
        assert evidence.observer_pose_valid
        assert evidence.image.header.stamp == original_stamp
        assert evidence.observer_local_pose.header.stamp == original_stamp
        assert evidence.observer_local_pose.pose.position == Point(x=5.0)


def test_future_pose_wait_is_shared_and_cannot_exceed_remaining_capture_budget(node):
    node.pair()
    node.odom(99.95, 4.0)
    node.config['inspection_evidence_timeout'] = 0.07
    report = capture(node)
    assert report.evidence_complete
    assert node.wall == pytest.approx(100.07)
    assert not report.rgb_evidence.observer_pose_valid
    assert not report.thermal_evidence.observer_pose_valid


def test_future_pose_wait_is_capped_at_a_quarter_wall_second(node):
    node.pair()
    node.odom(99.95, 4.0)
    report = capture(node)
    assert report.evidence_complete
    assert node.wall == pytest.approx(100.25)
    assert not report.rgb_evidence.observer_pose_valid


@pytest.mark.parametrize('history', ['absent', 'too_old', 'already_bracketed_gap'])
def test_impossible_pose_association_adds_no_capture_wait(node, history):
    node.pair()
    if history == 'too_old':
        node.odom(99.0, 4.0)
    elif history == 'already_bracketed_gap':
        node.odom(99.0, 4.0)
        node.odom(100.1, 6.0)
    report = capture(node)
    assert report.evidence_complete
    assert node.wall == 100.0
    assert not report.rgb_evidence.observer_pose_valid


def test_cancellation_remains_responsive_during_future_pose_wait(node):
    node.pair()
    node.odom(99.95, 4.0)
    node.on_wait = lambda: setattr(node, 'cancel', True)
    with pytest.raises(RuntimeError, match='operator stopped'):
        capture(node)
    assert node.wall == pytest.approx(100.01)


def test_future_pose_wait_does_not_extend_the_valid_age_of_an_image(node):
    node.pair()
    node.now = 101.9
    node.odom(99.95, 4.0)
    report = capture(node)
    assert not report.evidence_complete
    assert not report.rgb_evidence.valid and not report.rgb_evidence.image.data
    assert not report.thermal_evidence.valid and not report.thermal_evidence.image.data
    assert node.wall == pytest.approx(100.25)


def test_pose_history_has_a_fixed_bound_and_ignores_other_frames(node):
    node.config['inspection_evidence_pose_history'] = 3
    for index in range(10):
        node.now += 0.01
        node.odom(node.now, float(index))
    assert len(node._inspection_evidence_poses) == 3
    invalid = node.odom(node.now + 0.01, 12.0)
    invalid.header.frame_id = 'map'
    node._inspection_evidence_odom_callback(invalid)
    assert all(pose.header.frame_id == 'px4_odom' for pose in node._inspection_evidence_poses)


@pytest.mark.parametrize('value', [math.inf, math.nan, 0.0, -1.0, True])
def test_invalid_configuration_fails_before_an_unbounded_wait_or_allocation(node, value):
    node.config['inspection_evidence_timeout'] = value
    with pytest.raises(ValueError, match='finite and positive'):
        capture(node)
