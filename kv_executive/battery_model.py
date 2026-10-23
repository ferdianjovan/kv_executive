"""Learn battery percentage per travelled metre without depending on ROS nodes.

Percentages use the human-readable 0..100 scale, rather than BatteryState's
0..1 fraction. Distances include vertical travel. Records live in memory for
the life of the monitor; each record retains independent stamped pose copies.
"""

from copy import deepcopy
from dataclasses import dataclass
import math

from geometry_msgs.msg import PoseStamped


def _finite(value, description):
    try:
        value = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f'{description} must be finite') from exc
    if not math.isfinite(value):
        raise ValueError(f'{description} must be finite')
    return value


def _percentage(value):
    value = _finite(value, 'Battery percentage')
    if not 0.0 <= value <= 100.0:
        raise ValueError('Battery percentage must be between 0 and 100')
    return value


def _position(pose):
    try:
        frame = pose.header.frame_id
        position = tuple(
            _finite(getattr(pose.pose.position, axis), f'Position {axis}')
            for axis in 'xyz'
        )
        orientation = tuple(
            _finite(getattr(pose.pose.orientation, axis), f'Quaternion {axis}')
            for axis in 'xyzw'
        )
    except AttributeError as exc:
        raise ValueError('A valid stamped pose is required') from exc
    if not isinstance(frame, str) or not frame.strip():
        raise ValueError('Pose must have a nonempty coordinate frame')
    if not any(orientation):
        raise ValueError('Pose quaternion must have a nonzero norm')
    return position


@dataclass
class _Segment:
    start_pose: PoseStamped
    end_pose: PoseStamped
    start_percent: float
    end_percent: float
    distance_m: float = 0.0
    invalid_reason: str = ''


class BatteryUsageModel:
    """Keep WALK samples and estimate a buffered straight-line return budget.

    The learned rate is total consumed percentage divided by total travelled
    metres, so short samples do not receive excessive weight. A usable ongoing
    WALK contributes once, allowing an alert during the first flight. Partial
    canceled/aborted WALKs can teach the model if their telemetry is valid.
    """

    def __init__(self, minimum_distance=1.0, reserve_percent=15.0,
                 return_safety_factor=1.3):
        self.minimum_distance = _finite(minimum_distance, 'Minimum distance')
        if self.minimum_distance <= 0.0:
            raise ValueError('Minimum distance must be positive')
        self.reserve_percent = _percentage(reserve_percent)
        self.return_safety_factor = _finite(
            return_safety_factor, 'Return safety factor'
        )
        if self.return_safety_factor < 1.0:
            raise ValueError('Return safety factor must be at least one')
        self.records = {}
        self._active = None
        self._next_sample_id = 1

    @property
    def active(self):
        """Whether a WALK segment has been started and not yet finished."""
        return self._active is not None

    def begin(self, start_pose: PoseStamped, battery_percent):
        """Start a WALK using contemporaneous pose and battery telemetry."""
        if self.active:
            raise RuntimeError('A battery usage segment is already active')
        _position(start_pose)
        battery_percent = _percentage(battery_percent)
        self._active = _Segment(
            deepcopy(start_pose), deepcopy(start_pose),
            battery_percent, battery_percent,
        )

    def reset_active(self):
        """Discard an unfinished segment, retaining all stored samples."""
        self._active = None

    def invalidate_active(self, reason):
        """Exclude this segment from learning after any telemetry gap/error."""
        if self.active and not self._active.invalid_reason:
            self._active.invalid_reason = str(reason) or 'Invalid telemetry'

    def update(self, pose: PoseStamped, battery_percent):
        """Integrate measured motion; invalid telemetry permanently taints WALK.

        Tiny SOC rebounds up to 0.1 percentage points are ignored. Larger
        increases, including accumulated small increases, indicate charging or
        a battery reset and exclude the segment from the depletion estimate.
        """
        if not self.active:
            raise RuntimeError('No battery usage segment is active')
        segment = self._active
        try:
            position = _position(pose)
            battery_percent = _percentage(battery_percent)
            if pose.header.frame_id != segment.start_pose.header.frame_id:
                raise ValueError('WALK poses must use one coordinate frame')
            if battery_percent > segment.end_percent + 0.1:
                raise ValueError('Battery percentage increased during WALK')
            distance = _finite(
                segment.distance_m + math.dist(
                    _position(segment.end_pose), position
                ),
                'Travelled distance',
            )
        except ValueError as exc:
            self.invalidate_active(str(exc))
            raise
        segment.distance_m = distance
        segment.end_pose = deepcopy(pose)
        segment.end_percent = min(segment.end_percent, battery_percent)

    def _sample(self, segment, sample_id, outcome):
        consumed = max(0.0, segment.start_percent - segment.end_percent)
        reason = segment.invalid_reason
        if not reason and segment.distance_m < self.minimum_distance:
            reason = 'Travelled distance is below the learning minimum'
        if not reason and consumed <= 0.0:
            reason = 'No measurable battery depletion'
        return {
            'sample_id': sample_id,
            'start_pose': deepcopy(segment.start_pose),
            'end_pose': deepcopy(segment.end_pose),
            'distance_m': segment.distance_m,
            'start_battery_percent': segment.start_percent,
            'end_battery_percent': segment.end_percent,
            'consumed_percent': consumed,
            'percentage_per_meter': (
                consumed / segment.distance_m if segment.distance_m else 0.0
            ),
            'valid_for_estimate': not reason,
            'invalid_reason': reason,
            'outcome': outcome,
        }

    def finish(self, end_pose: PoseStamped, battery_percent,
               outcome='completed'):
        """Finalize and store a WALK; return a copy independent of the store."""
        if not isinstance(outcome, str) or not outcome:
            raise ValueError('WALK outcome must be a nonempty string')
        self.update(end_pose, battery_percent)
        sample = self._sample(self._active, self._next_sample_id, outcome)
        self.records[self._next_sample_id] = sample
        self._next_sample_id += 1
        self._active = None
        return deepcopy(sample)

    def estimate(self, current_pose: PoseStamped, home_pose: PoseStamped,
                 remaining_percent):
        """Estimate straight-line return consumption plus reserve and factor.

        Unknown rates are represented by NaN and estimate_available=False.
        The absolute reserve still applies before a usable sample exists.
        """
        current = _position(current_pose)
        home = _position(home_pose)
        if current_pose.header.frame_id != home_pose.header.frame_id:
            raise ValueError('Current and home poses must use one frame')
        remaining_percent = _percentage(remaining_percent)
        home_distance = _finite(math.dist(current, home), 'Home distance')
        samples = [sample for sample in self.records.values()
                   if sample['valid_for_estimate']]
        if self.active:
            active_sample = self._sample(self._active, 0, 'in_progress')
            if active_sample['valid_for_estimate']:
                samples.append(active_sample)
        available = bool(samples)
        mean = (
            math.fsum(sample['consumed_percent'] for sample in samples)
            / math.fsum(sample['distance_m'] for sample in samples)
            if available else math.nan
        )
        required = home_distance * mean if available else math.nan
        buffered = (required * self.return_safety_factor if available else 0.0)
        buffered += self.reserve_percent
        return {
            'home_distance_m': home_distance,
            'mean_percent_per_meter': mean,
            'return_required_percent': required,
            'buffered_required_percent': buffered,
            'return_margin_percent': remaining_percent - buffered,
            'estimate_available': available,
            'return_alert': remaining_percent <= buffered,
            'sample_count': len(samples),
        }
