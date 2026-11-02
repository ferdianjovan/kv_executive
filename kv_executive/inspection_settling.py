"""Prove physical settling before collecting stationary inspection evidence."""

from copy import deepcopy
import math
import time


def _quaternion_angle(first, second):
    a = tuple(getattr(first, axis) for axis in 'xyzw')
    b = tuple(getattr(second, axis) for axis in 'xyzw')
    lengths = math.sqrt(sum(value * value for value in a)), math.sqrt(
        sum(value * value for value in b))
    if not all(math.isfinite(value) and value > 1e-9 for value in lengths):
        return math.inf
    cosine = min(1.0, abs(sum(x * y for x, y in zip(a, b)) / math.prod(lengths)))
    return 2.0 * math.acos(cosine)


def _position_distance(first, second):
    return math.dist(tuple(getattr(first, axis) for axis in 'xyz'),
                     tuple(getattr(second, axis) for axis in 'xyz'))


class InspectionSettlingMixin:
    """Keep stationary evidence separate from mode acknowledgement and motion."""

    def _verify_settling_limits(self):
        limits = {
            'speed': self.config.get('verify_settle_speed', 0.10),
            'angular': self.config.get('verify_settle_angular_rate', math.radians(3.0)),
            'dwell': self.config.get('verify_settle_dwell', 1.0),
            'timeout': self.config.get('verify_settle_timeout', 15.0),
            'clock_timeout': self.config.get('verify_settle_clock_timeout', 5.0),
            'freshness': self.config.get('telemetry_timeout', 2.0),
        }
        if any(isinstance(value, bool) or not math.isfinite(value) or value <= 0.0
               for value in limits.values()):
            raise ValueError('Inspection settling limits must be finite and positive')
        return limits

    def _verify_settling_sample(self, limits):
        state, odometry = self._snapshot(require_armed=True)
        stamp = odometry.header.stamp
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        now_ns = self.get_clock().now().nanoseconds
        pose = deepcopy(odometry.pose.pose)
        values = tuple(getattr(pose.position, axis) for axis in 'xyz') + tuple(
            getattr(pose.orientation, axis) for axis in 'xyzw')
        if not all(math.isfinite(value) for value in values):
            return None, 'odometry contains a nonfinite pose'
        if _quaternion_angle(pose.orientation, pose.orientation) == math.inf:
            return None, 'odometry contains an invalid orientation'
        age = (now_ns - stamp_ns) / 1e9
        if now_ns <= 0 or stamp_ns <= 0:
            return None, 'source clock or odometry timestamp is uninitialized'
        if not -self.config.get('verify_future_tolerance', 0.25) <= age <= limits['freshness']:
            return None, 'odometry source timestamp is stale or in the future'
        return {
            'mode': state.mode, 'pose': pose, 'stamp_ns': stamp_ns,
            'now_ns': now_ns, 'frame': odometry.header.frame_id,
            'twist': deepcopy(getattr(odometry, 'twist', None)),
            'wall': time.monotonic(),
        }, ''

    @staticmethod
    def _verify_settling_rates(previous, current):
        elapsed = (current['stamp_ns'] - previous['stamp_ns']) / 1e9
        if elapsed <= 0.0 or current['frame'] != previous['frame']:
            return math.inf, math.inf
        linear = _position_distance(current['pose'].position, previous['pose'].position) / elapsed
        angular = _quaternion_angle(
            current['pose'].orientation, previous['pose'].orientation) / elapsed
        # Default zero twist/covariance is not evidence of a stopped vehicle.
        # Valid nondefault uncertainty permits twist to tighten pose-derived
        # rates, never to hide translation or rotation visible in odometry.
        twist = current['twist']
        covariance = getattr(twist, 'covariance', ())
        if len(covariance) == 36:
            diagonal = [covariance[index] for index in (0, 7, 14, 21, 28, 35)]
            if (all(math.isfinite(value) and value >= 0.0 for value in diagonal)
                    and any(value > 0.0 for value in diagonal)):
                velocity = getattr(twist, 'twist', None)
                if velocity is not None:
                    components = tuple(getattr(velocity.linear, axis) for axis in 'xyz') + tuple(
                        getattr(velocity.angular, axis) for axis in 'xyz')
                    if all(math.isfinite(value) for value in components):
                        measured_linear = math.sqrt(sum(v * v for v in components[:3]))
                        measured_angular = math.sqrt(sum(v * v for v in components[3:]))
                        linear = max(linear, measured_linear)
                        angular = max(angular, measured_angular)
        return linear, angular

    def _verify_wall_observation_ready(self):
        """Recheck a proved stationary context without waiting or changing mode."""
        context = getattr(self, '_verify_wall_observation_context', None)
        if context is None:
            return False
        limits = self._verify_settling_limits()
        sample, reason = self._verify_settling_sample(limits)
        if sample is not None:
            if sample['mode'] != 'AUTO.LOITER':
                reason = 'vehicle left AUTO.LOITER'
            elif sample['now_ns'] < context['clock_ns']:
                reason = 'source clock moved backwards'
            elif sample['now_ns'] > context['clock_ns']:
                context['clock_ns'], context['clock_wall'] = sample['now_ns'], sample['wall']
            elif sample['wall'] - context['clock_wall'] >= limits['clock_timeout']:
                reason = 'source clock stopped progressing'
            previous = context['sample']
            if not reason and sample['stamp_ns'] < previous['stamp_ns']:
                reason = 'odometry timestamp moved backwards'
            if not reason and sample['stamp_ns'] > previous['stamp_ns']:
                if sample['wall'] - previous['wall'] >= limits['dwell']:
                    reason = 'odometry update gap interrupted the stationary dwell'
                else:
                    linear, angular = self._verify_settling_rates(previous, sample)
                    if linear > limits['speed'] or angular > limits['angular']:
                        reason = 'vehicle resumed translation or rotation'
                    else:
                        context['sample'] = sample
                        context['linear_speed'], context['angular_rate'] = linear, angular
            if not reason and sample['stamp_ns'] == previous['stamp_ns'] and (
                    _position_distance(sample['pose'].position, previous['pose'].position) > 1e-6
                    or _quaternion_angle(sample['pose'].orientation,
                                         previous['pose'].orientation) > 1e-6):
                reason = 'odometry pose changed without a new timestamp'
            if (not reason and sample['stamp_ns'] == previous['stamp_ns']
                    and sample['wall'] - previous['wall'] >= limits['dwell']):
                reason = 'no distinct odometry update during the observation dwell'
            if not reason and (
                    _position_distance(sample['pose'].position, context['pose'].position)
                    > limits['speed'] * limits['dwell'] + 1e-6
                    or _quaternion_angle(sample['pose'].orientation, context['pose'].orientation)
                    > limits['angular'] * limits['dwell'] + 1e-6):
                reason = 'vehicle drifted from the stationary observation pose'
        if reason:
            self._verify_wall_observation_context = None
            self._verify_wall_settle_reason = reason
            return False
        return True

    def _verify_prepare_wall_observation(self, guard=None, *, align_fire=True):
        """Align fire heading, then require fresh low-rate odometry in LOITER."""
        limits = self._verify_settling_limits()

        def check():
            self._check_running()
            if guard:
                guard()

        check()
        if align_fire and not self._verify_align_fire_heading(guard=check):
            self._verify_wall_observation_context = None
            self._verify_wall_settle_reason = 'fire heading alignment is unavailable'
            return False
        if self._verify_wall_observation_ready():
            return True
        self._verify_wall_observation_context = None
        started = time.monotonic()
        clock_ns = self.get_clock().now().nanoseconds
        clock_wall = started
        previous, low_since, low_samples = None, None, 0
        reason = 'waiting for distinct fresh odometry samples'
        reported = False
        while time.monotonic() - started < limits['timeout']:
            check()
            now_ns = self.get_clock().now().nanoseconds
            now_wall = time.monotonic()
            if now_ns < clock_ns:
                reason = 'source clock moved backwards'
                break
            if now_ns > clock_ns:
                clock_ns, clock_wall = now_ns, now_wall
            elif now_wall - clock_wall >= limits['clock_timeout']:
                reason = 'source clock stopped progressing'
                break
            sample, issue = self._verify_settling_sample(limits)
            if sample is None:
                previous, low_since, low_samples = None, None, 0
                reason = issue
            elif sample['mode'] != 'AUTO.LOITER':
                previous, low_since, low_samples = None, None, 0
                reason = 'waiting for AUTO.LOITER'
                if sample['mode'] == 'OFFBOARD':
                    self._loiter()
                else:
                    break
            elif previous is None:
                previous = sample
            elif sample['stamp_ns'] < previous['stamp_ns']:
                previous, low_since, low_samples = sample, None, 0
                reason = 'odometry timestamp moved backwards'
            elif sample['stamp_ns'] > previous['stamp_ns']:
                if sample['wall'] - previous['wall'] >= limits['dwell']:
                    previous, low_since, low_samples = sample, None, 0
                    reason = 'odometry update gap interrupted the stationary dwell'
                    self._stop.wait(0.05)
                    continue
                linear, angular = self._verify_settling_rates(previous, sample)
                if linear <= limits['speed'] and angular <= limits['angular']:
                    if low_since is None:
                        low_since, low_samples = previous['wall'], 1
                    low_samples += 1
                    if sample['wall'] - low_since >= limits['dwell'] and low_samples >= 3:
                        context = {
                            'settled_ns': sample['now_ns'], 'pose': deepcopy(sample['pose']),
                            'odom_stamp_ns': sample['stamp_ns'], 'frame': sample['frame'],
                            'sample': sample, 'clock_ns': sample['now_ns'],
                            'clock_wall': sample['wall'], 'linear_speed': linear,
                            'angular_rate': angular, 'map_after_epoch': False,
                            'map_wait_complete': False,
                        }
                        self._verify_wall_observation_context = context
                        self._verify_wall_settle_reason = ''
                        self._report(
                            'Verify: stationary observation ready; '
                            f'speed={linear:.3f} m/s, angular rate={math.degrees(angular):.2f} '
                            f'deg/s, dwell={sample["wall"] - low_since:.2f} wall s, '
                            f'distinct odometry samples={low_samples}')
                        wait_for_map = getattr(self, '_verify_wait_for_wall_map', None)
                        if wait_for_map is not None:
                            wait_for_map(context, guard=check)
                        else:
                            context['map_wait_complete'] = True
                        return self._verify_wall_observation_ready()
                else:
                    low_since, low_samples = None, 0
                    reason = (f'still moving: speed={linear:.3f} m/s, '
                              f'angular rate={math.degrees(angular):.2f} deg/s')
                previous = sample
            if not reported:
                self._report('Verify: waiting for physical settling before wall observation')
                reported = True
            self._stop.wait(0.05)
        self._verify_wall_settle_reason = reason
        self._report('Verify: stationary observation unavailable: ' + reason)
        return False
