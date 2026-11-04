"""Bounded original-image snapshots for human review of inspection results."""

from copy import deepcopy
from dataclasses import dataclass
import math
import threading
import time
from types import SimpleNamespace

import cv2
from geometry_msgs.msg import PoseStamped
from kv_executive.mission_geometry import interpolate_pose
import numpy as np


EVIDENCE_DEFAULTS = {
    'inspection_evidence_rgb_topic': '/kv_drone/rgb_camera/image',
    'inspection_evidence_thermal_topic': '/kv_drone/thermal_camera/image',
    'inspection_evidence_timeout': 3.0,
    'inspection_evidence_max_age': 2.0,
    'inspection_evidence_pair_slop': 0.5,
    'inspection_evidence_cache_bytes': 16 * 1024 * 1024,
    'inspection_evidence_cache_frames': 3,
    'inspection_evidence_frame_bytes': 8 * 1024 * 1024,
    'inspection_evidence_rgb_max_bytes': 256 * 1024,
    'inspection_evidence_thermal_max_bytes': 1024 * 1024,
    'inspection_evidence_rgb_max_side': 1280,
    'inspection_evidence_rgb_quality': 80,
    'inspection_evidence_pose_history': 128,
    'inspection_evidence_pose_slop': 0.25,
}

_ENCODINGS = {
    'rgb8': (3, 1), 'bgr8': (3, 1), 'rgba8': (4, 1), 'bgra8': (4, 1),
    'mono8': (1, 1), '8UC1': (1, 1), 'mono16': (1, 2), '16UC1': (1, 2),
}


def _stamp_ns(message):
    stamp = message.header.stamp
    return stamp.sec * 1_000_000_000 + stamp.nanosec


@dataclass(frozen=True)
class _Frame:
    header: object
    encoding: str
    width: int
    height: int
    step: int
    bigendian: bool
    data: bytes
    received: float

    @property
    def stamp(self):
        return self.header.stamp.sec * 1_000_000_000 + self.header.stamp.nanosec


class InspectionEvidenceMixin:
    """Keep small image/pose histories and publish bounded compressed evidence."""

    def _inspection_evidence_init(self):
        self._inspection_evidence_lock = threading.RLock()
        self._inspection_evidence_frames = {'smoke': [], 'fire': []}
        self._inspection_evidence_detections = {'smoke': [], 'fire': []}
        self._inspection_evidence_poses = []
        self._inspection_evidence_last_stamps = {'smoke': 0, 'fire': 0}
        self._inspection_evidence_problem = {'smoke': 'no RGB image', 'fire': 'no thermal image'}
        self._inspection_evidence_bytes = 0
        self._inspection_evidence_clock_ns = 0
        self._inspection_evidence_after_ns = 0

    def _inspection_evidence_ensure(self):
        if not hasattr(self, '_inspection_evidence_lock'):
            self._inspection_evidence_init()

    def _inspection_evidence_limits(self):
        limits = {key: self.config.get(key, value) for key, value in EVIDENCE_DEFAULTS.items()}
        for key, value in limits.items():
            if isinstance(EVIDENCE_DEFAULTS[key], str):
                if not isinstance(value, str) or not value.strip():
                    raise ValueError(f'{key} must be a nonempty topic')
                continue
            if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
                raise ValueError(f'{key} must be finite and positive')
            if isinstance(EVIDENCE_DEFAULTS[key], int) and not isinstance(value, int):
                raise ValueError(f'{key} must be an integer')
        if limits['inspection_evidence_rgb_quality'] > 100:
            raise ValueError('inspection_evidence_rgb_quality must be at most 100')
        return limits

    def _inspection_evidence_clock_locked(self):
        now = self.get_clock().now().nanoseconds
        if now < self._inspection_evidence_clock_ns:
            self._inspection_evidence_frames = {'smoke': [], 'fire': []}
            self._inspection_evidence_detections = {'smoke': [], 'fire': []}
            self._inspection_evidence_poses.clear()
            self._inspection_evidence_last_stamps = {'smoke': 0, 'fire': 0}
            self._inspection_evidence_bytes = 0
            self._inspection_evidence_after_ns = 0
        self._inspection_evidence_clock_ns = now
        return now

    def _inspection_reset_evidence(self):
        """Prevent prior-episode images from becoming a new site's evidence."""
        self._inspection_evidence_ensure()
        with self._inspection_evidence_lock:
            now = self._inspection_evidence_clock_locked()
            self._inspection_evidence_frames = {'smoke': [], 'fire': []}
            self._inspection_evidence_detections = {'smoke': [], 'fire': []}
            self._inspection_evidence_bytes = 0
            self._inspection_evidence_after_ns = now
            self._inspection_evidence_problem = {
                'smoke': 'no RGB image after episode start',
                'fire': 'no thermal image after episode start',
            }

    def _inspection_rgb_image_callback(self, message):
        self._inspection_evidence_image_callback(message, 'smoke')

    def _inspection_thermal_image_callback(self, message):
        self._inspection_evidence_image_callback(message, 'fire')

    def _inspection_evidence_image_callback(self, message, source):
        self._inspection_evidence_ensure()
        limits = self._inspection_evidence_limits()
        stamp = _stamp_ns(message)
        encoding, width, height, step = (
            message.encoding, message.width, message.height, message.step)
        supported = (encoding in {'rgb8', 'bgr8', 'rgba8', 'bgra8', 'mono8', '8UC1'}
                     if source == 'smoke' else encoding in {'mono8', '8UC1', 'mono16', '16UC1'})
        reason = ''
        if not supported:
            reason = 'unsupported original image encoding'
        elif (not message.header.frame_id.strip() or len(message.header.frame_id) > 256
              or any(isinstance(value, bool) or not isinstance(value, int) or value <= 0
                     for value in (width, height, step))):
            reason = 'invalid image dimensions or frame'
        else:
            channels, itemsize = _ENCODINGS[encoding]
            size = step * height
            if (step < width * channels * itemsize or len(message.data) != size
                    or size > min(limits['inspection_evidence_frame_bytes'],
                                  limits['inspection_evidence_cache_bytes'])):
                reason = 'invalid or oversized image payload'
        with self._inspection_evidence_lock:
            now = self._inspection_evidence_clock_locked()
            age = (now - stamp) / 1e9
            if (now <= 0 or stamp <= self._inspection_evidence_after_ns
                    or not -self.config.get('verify_future_tolerance', 0.25) <= age
                    <= limits['inspection_evidence_max_age']):
                reason = 'image source timestamp is unavailable or stale'
            if stamp <= self._inspection_evidence_last_stamps[source]:
                reason = 'duplicate or reordered image'
            if reason:
                self._inspection_evidence_problem[source] = reason
                return
            # Check all bounds before copying the incoming payload.
            frame = _Frame(deepcopy(message.header), encoding, width, height, step,
                           bool(message.is_bigendian), bytes(message.data), time.monotonic())
            self._inspection_evidence_last_stamps[source] = stamp
            frames = self._inspection_evidence_frames[source]
            frames.append(frame)
            self._inspection_evidence_bytes += len(frame.data)
            while len(frames) > limits['inspection_evidence_cache_frames']:
                self._inspection_evidence_bytes -= len(frames.pop(0).data)
            while self._inspection_evidence_bytes > limits['inspection_evidence_cache_bytes']:
                oldest = min((items[0].received, key) for key, items in
                             self._inspection_evidence_frames.items() if items)[1]
                self._inspection_evidence_bytes -= len(
                    self._inspection_evidence_frames[oldest].pop(0).data)
            self._inspection_evidence_problem[source] = ''

    def _inspection_evidence_detection_callback(self, message, source):
        """Keep bounded source-stamped metadata, including negative thermal frames."""
        self._inspection_evidence_ensure()
        if source not in ('smoke', 'fire'):
            raise ValueError('Unknown inspection evidence source')
        stamp = _stamp_ns(message)
        with self._inspection_evidence_lock:
            now = self._inspection_evidence_clock_locked()
            if (stamp <= self._inspection_evidence_after_ns or now <= 0
                    or not message.header.frame_id.strip()
                    or len(message.header.frame_id) > 256
                    or not -self.config.get('verify_future_tolerance', 0.25)
                    <= (now - stamp) / 1e9 <= self._inspection_evidence_limits()[
                        'inspection_evidence_max_age']):
                return
            detections = self._inspection_evidence_detections[source]
            if detections and stamp <= _stamp_ns(detections[-1]):
                return
            names = ('status', 'detected', 'track_id', 'source_count', 'bbox_x', 'bbox_y',
                     'bbox_width', 'bbox_height', 'angular_bounds_valid', 'yaw_min_rad',
                     'yaw_max_rad', 'vertical_min_rad', 'vertical_max_rad')
            metadata = SimpleNamespace(header=deepcopy(message.header))
            for name in names:
                if hasattr(message, name):
                    setattr(metadata, name, getattr(message, name))
            detections.append(metadata)
            keep = self._inspection_evidence_limits()['inspection_evidence_cache_frames']
            del detections[:-keep]

    def _inspection_evidence_odom_callback(self, odometry):
        """Retain normalized local poses for capture-time interpolation only."""
        self._inspection_evidence_ensure()
        stamp = _stamp_ns(odometry)
        if stamp <= 0 or odometry.header.frame_id != self.config.get('local_frame', 'px4_odom'):
            return
        try:
            pose = interpolate_pose(odometry.pose.pose, odometry.pose.pose, 0.0)
        except ValueError:
            return
        with self._inspection_evidence_lock:
            now = self._inspection_evidence_clock_locked()
            if now <= 0 or stamp > now + self.config.get('verify_future_tolerance', 0.25) * 1e9:
                return
            poses = self._inspection_evidence_poses
            if poses and stamp <= _stamp_ns(poses[-1]):
                return
            poses.append(PoseStamped(header=deepcopy(odometry.header), pose=pose))
            del poses[:-self._inspection_evidence_limits()['inspection_evidence_pose_history']]

    def _inspection_evidence_pose(self, stamp, limits):
        with self._inspection_evidence_lock:
            poses = list(self._inspection_evidence_poses)
        for index, pose in enumerate(poses):
            current = _stamp_ns(pose)
            if current == stamp:
                return deepcopy(pose)
            if current > stamp and index:
                previous = poses[index - 1]
                before = _stamp_ns(previous)
                if max(stamp - before, current - stamp) / 1e9 > limits[
                        'inspection_evidence_pose_slop']:
                    return None
                result = PoseStamped(header=deepcopy(previous.header), pose=interpolate_pose(
                    previous.pose, pose.pose, (stamp - before) / (current - before)))
                result.header.stamp.sec, result.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
                return result
        return None

    def _inspection_evidence_select(self, fire_message, after_ns, limits):
        controller = getattr(self, '_verify', None)
        smoke = controller.fresh_smoke() if controller is not None else None
        fire = (controller.stable_fire() if controller is not None and fire_message is not None
                else None)
        if fire is not None:
            same_frame = fire.header.frame_id == fire_message.header.frame_id
            same_track = getattr(fire, 'track_id', 0) == getattr(fire_message, 'track_id', 0)
            if not (same_frame and same_track):
                fire = None
        with self._inspection_evidence_lock:
            now = self._inspection_evidence_clock_locked()
            wall = time.monotonic()
            after_ns = max(after_ns, self._inspection_evidence_after_ns)
            selected = {}
            for source, reference in (('smoke', smoke), ('fire', fire)):
                candidates = [frame for frame in self._inspection_evidence_frames[source]
                              if frame.stamp > after_ns
                              and -self.config.get('verify_future_tolerance', 0.25)
                              <= (now - frame.stamp) / 1e9 <= limits['inspection_evidence_max_age']
                              and 0.0 <= wall - frame.received
                              <= limits['inspection_evidence_max_age']]
                if source == 'fire' and fire_message is None:
                    metadata = {(_stamp_ns(item), item.header.frame_id): item
                                for item in self._inspection_evidence_detections[source]}
                    matches = [(frame, metadata.get((frame.stamp, frame.header.frame_id)))
                               for frame in reversed(candidates)]
                    associated = [pair for pair in matches if pair[1] is not None]
                    selected[source] = (associated[0] if associated else
                                        matches[0] if matches else None)
                else:
                    selected[source] = next(((frame, deepcopy(reference))
                                             for frame in reversed(candidates)
                                             if reference is not None
                                             and frame.stamp == _stamp_ns(reference)
                                             and frame.header.frame_id
                                             == reference.header.frame_id),
                                            None)
            return selected

    @staticmethod
    def _inspection_evidence_pixels(frame):
        channels, itemsize = _ENCODINGS[frame.encoding]
        if itemsize == 2:
            dtype = np.dtype('>u2' if frame.bigendian else '<u2')
            pixels = np.ndarray((frame.height, frame.width), dtype=dtype,
                                buffer=frame.data, strides=(frame.step, itemsize))
            return pixels.astype(np.uint16)
        if channels == 1:
            return np.ndarray((frame.height, frame.width), dtype=np.uint8,
                              buffer=frame.data, strides=(frame.step, 1))
        pixels = np.ndarray((frame.height, frame.width, channels), dtype=np.uint8,
                            buffer=frame.data, strides=(frame.step, channels, 1))
        conversion = {'rgb8': cv2.COLOR_RGB2BGR, 'rgba8': cv2.COLOR_RGBA2BGR,
                      'bgra8': cv2.COLOR_BGRA2BGR}.get(frame.encoding)
        return cv2.cvtColor(pixels, conversion) if conversion is not None else pixels

    def _inspection_encode_evidence(self, source, choice, limits, check):
        from kv_executive.msg import InspectionImageEvidence
        evidence = InspectionImageEvidence()
        evidence.status = self._inspection_evidence_problem[source] or 'no matching fresh image'
        if choice is None:
            return evidence
        frame, detection = choice
        evidence.image.header = deepcopy(frame.header)
        evidence.original_encoding = frame.encoding
        evidence.original_width, evidence.original_height = frame.width, frame.height
        try:
            pixels = self._inspection_evidence_pixels(frame)
            if source == 'fire':
                check()
                okay, encoded = cv2.imencode('.png', pixels, [cv2.IMWRITE_PNG_COMPRESSION, 3])
                if not okay or encoded.nbytes > limits['inspection_evidence_thermal_max_bytes']:
                    evidence.status = 'lossless thermal image exceeds payload limit'
                    return evidence
                evidence.image.format = frame.encoding + '; png compressed ' + frame.encoding
            else:
                side = limits['inspection_evidence_rgb_max_side']
                factor = min(1.0, side / max(frame.width, frame.height))
                shape = (max(1, round(frame.width * factor)), max(1, round(frame.height * factor)))
                resized = cv2.resize(pixels, shape, interpolation=cv2.INTER_AREA)
                encoded = None
                for _ in range(8):
                    quality = limits['inspection_evidence_rgb_quality']
                    for quality in dict.fromkeys((quality, min(quality, 50), min(quality, 25))):
                        check()
                        okay, candidate = cv2.imencode(
                            '.jpg', resized, [cv2.IMWRITE_JPEG_QUALITY, quality])
                        if (okay and candidate.nbytes
                                <= limits['inspection_evidence_rgb_max_bytes']):
                            encoded = candidate
                            break
                    if encoded is not None:
                        break
                    height, width = resized.shape[:2]
                    if max(height, width) <= 16:
                        break
                    resized = cv2.resize(resized, (max(1, width // 2), max(1, height // 2)),
                                         interpolation=cv2.INTER_AREA)
                if encoded is None:
                    evidence.status = 'RGB image exceeds payload limit'
                    return evidence
                pixels = resized
                compressed_encoding = 'mono8' if resized.ndim == 2 else 'bgr8'
                evidence.image.format = frame.encoding + '; jpeg compressed ' + compressed_encoding
            check()
            evidence.image.data = encoded.tobytes()
            evidence.stored_height, evidence.stored_width = pixels.shape[:2]
        except cv2.error:
            evidence.status = 'image compression failed'
            return evidence
        evidence.valid, evidence.status = True, 'available'
        pose = self._inspection_evidence_pose(frame.stamp, limits)
        if pose is not None:
            evidence.observer_pose_valid = True
            evidence.observer_local_pose = pose
        if detection is not None:
            evidence.detection_valid = True
            evidence.detection_header = deepcopy(detection.header)
            evidence.detection_status = int(getattr(detection, 'status', 0))
            for name in ('detected', 'track_id', 'source_count', 'bbox_x', 'bbox_y',
                         'bbox_width', 'bbox_height', 'angular_bounds_valid', 'yaw_min_rad',
                         'yaw_max_rad', 'vertical_min_rad', 'vertical_max_rad'):
                if hasattr(detection, name):
                    setattr(evidence, name, getattr(detection, name))
        return evidence

    def _inspection_evidence_still_valid(self, source, choice, fire_message, after_ns, limits):
        if choice is None:
            return False
        frame, _ = choice
        with self._inspection_evidence_lock:
            now = self._inspection_evidence_clock_locked()
            after_ns = max(after_ns, self._inspection_evidence_after_ns)
            valid = (frame.stamp > after_ns
                     and -self.config.get('verify_future_tolerance', 0.25)
                     <= (now - frame.stamp) / 1e9 <= limits['inspection_evidence_max_age']
                     and 0.0 <= time.monotonic() - frame.received
                     <= limits['inspection_evidence_max_age'])
        if not valid or source != 'fire' or fire_message is None:
            return valid
        current = self._verify.stable_fire()
        return (current is not None and current.header.frame_id == frame.header.frame_id
                and getattr(current, 'track_id', 0) == getattr(fire_message, 'track_id', 0))

    def _inspection_evidence_wait_for_pose(self, choices, limits, deadline, check):
        """Briefly await a plausible next odometry bracket for the fixed snapshots."""
        stamps = [choice[0].stamp for choice in choices if choice is not None]
        until = min(deadline, time.monotonic() + 0.25)
        while stamps:
            with self._inspection_evidence_lock:
                self._inspection_evidence_clock_locked()
                poses = self._inspection_evidence_poses
                last = _stamp_ns(poses[-1]) if poses else None
                pending = last is not None and any(
                    0 < (stamp - last) / 1e9 <= limits['inspection_evidence_pose_slop']
                    for stamp in stamps)
            remaining = until - time.monotonic()
            if not pending or remaining <= 0.0:
                return
            check()
            self._stop.wait(min(0.01, remaining))

    def _inspection_capture_evidence(self, report, fire_message=None, guard=None, *, after_ns=0):
        """Attach fresh individual snapshots; missing evidence never fabricates old images."""
        self._inspection_evidence_ensure()
        limits = self._inspection_evidence_limits()

        def check():
            if guard:
                guard()
            checker = getattr(self, '_check_running', None)
            if checker is not None:
                checker()

        deadline = time.monotonic() + limits['inspection_evidence_timeout']
        while True:
            check()
            selected = self._inspection_evidence_select(fire_message, after_ns, limits)
            rgb, thermal = selected['smoke'], selected['fire']
            paired = (rgb is not None and thermal is not None and
                      abs(rgb[0].stamp - thermal[0].stamp) / 1e9
                      <= limits['inspection_evidence_pair_slop'])
            if paired or time.monotonic() >= deadline:
                break
            self._stop.wait(min(0.05, max(0.0, deadline - time.monotonic())))
        self._inspection_evidence_wait_for_pose((rgb, thermal), limits, deadline, check)
        report.rgb_evidence = self._inspection_encode_evidence('smoke', rgb, limits, check)
        report.thermal_evidence = self._inspection_encode_evidence('fire', thermal, limits, check)
        check()
        # Compression and pose association can take time; retain no evidence
        # that became stale or invalid during that work.
        for source, evidence, previous in (
                ('smoke', report.rgb_evidence, rgb), ('fire', report.thermal_evidence, thermal)):
            if evidence.valid and not self._inspection_evidence_still_valid(
                    source, previous, fire_message, after_ns, limits):
                evidence.valid = False
                evidence.image.data = b''
                evidence.status = 'image or associated detection changed during capture'
        report.evidence_complete = bool(
            report.rgb_evidence.valid and report.thermal_evidence.valid and paired)
        if report.evidence_complete:
            report.evidence_status = 'RGB and thermal snapshots available'
        elif report.rgb_evidence.valid and report.thermal_evidence.valid:
            report.evidence_status = 'camera acquisition timestamps are too far apart'
        else:
            report.evidence_status = ('RGB: ' + report.rgb_evidence.status + '; thermal: '
                                      + report.thermal_evidence.status)
