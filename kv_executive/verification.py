"""Bounded, observation-driven verification; all translations use the planner.

Camera coverage never establishes proximity or interrupts a committed smoke leg.
Fire range never selects motion or completion. The search keeps the aircraft level.
"""

from copy import deepcopy
import math
import threading
import time

from kv_executive.inspection_navigation import smoke_wall_standoff_reached
from kv_executive.mission_geometry import yaw_from_quaternion
from yasmin import State, StateMachine


VERIFY_DEFAULTS = {
    'verify_observation_timeout': 2.0,
    'verify_future_tolerance': 0.25,
    'verify_fire_persistence': 1.5,
    'verify_fire_min_frames': 3,
    'verify_track_center_tolerance': 0.08,
    'verify_track_area_ratio': 2.0,
    'verify_reacquire_timeout': 15.0,
    'verify_reacquire_observation_timeout': 5.0,
    'verify_reacquire_scan_step': math.radians(10.0),
    'verify_reacquire_yaw_step': math.radians(5.0),
    'verify_fire_heading_grace': 2.0,
    'verify_heading_rate': math.radians(12.0),
    'verify_heading_clock_timeout': 5.0,
    'verify_fire_heading_tolerance': math.radians(2.0),
    'verify_fire_heading_align_timeout': 15.0,
    'verify_settle_dwell': 1.0,
    'verify_settle_timeout': 15.0,
    'verify_settle_map_timeout': 15.0,
    'verify_settle_speed': 0.10,
    'verify_settle_angular_rate': math.radians(3.0),
    'verify_settle_clock_timeout': 5.0,
    'verify_retry_cooldown': 2.0,
    'verify_smoke_center_timeout': 30.0,
    'verify_smoke_loss_grace': 5.0,
    'verify_viewpoint_climb': 2.0,
    'verify_viewpoint_lateral_step': 2.0,
    'verify_viewpoint_max_steps': 3,
    'verify_viewpoint_timeout': 60.0,
    'verify_center_timeout': 30.0,
    'verify_yaw_tolerance': math.radians(5.0),
    'verify_yaw_step': math.radians(30.0),
    'verify_vertical_step': 0.5,
    'verify_vertical_position_tolerance': 0.05,
    'verify_vertical_minimum_change': 0.10,
    'verify_vertical_tolerance': math.radians(5.0),
    'verify_vertical_hysteresis': math.radians(2.0),
    'verify_vertical_edge_threshold': math.radians(15.0),
    'verify_vertical_max_corrections': 3,
    'verify_vertical_memory_timeout': 15.0,
    'verify_vertical_clock_timeout': 5.0,
    'verify_wall_probe_timeout': 0.5,
    'verify_wall_recheck_timeout': 20.0,
    'verify_minimum_altitude': 2.0,
    'verify_step_min': 5.0,
    'verify_step_max': 20.0,
    'verify_full_coverage': 0.25,
    'verify_motion_speed': 0.5,
    'verify_search_hold': 5.0,
    'verify_search_center_timeout': 15.0,
    'verify_search_recovery_max': 3,
    'verify_localization_max_steps': 6,
    'verify_navigation_step': 3.0,
    'verify_navigation_preferred_agl': 7.5,
    'verify_navigation_max_agl': 10.0,
    'verify_navigation_candidate_limit': 18,
    'verify_navigation_map_wait': 5.0,
    'verify_navigation_map_timeout': 20.0,
    'verify_fire_approach_max_steps': 64,
    'verify_fire_leg_max_length': 12.0,
    'verify_fire_leg_timeout': 60.0,
    'verify_approach_max_rise': 1.0,
    'verify_approach_max_drop': 1.0,
    'verify_approach_max_backtrack': 0.3,
    'verify_approach_max_path_length': 12.0,
    'verify_recovery_max_rise': 6.0,
    'verify_recovery_max_drop': 1.0,
    'verify_recovery_max_backtrack': 3.3,
    'verify_fire_standoff': 10.0,
    'verify_navigation_no_progress_max': 2,
    'verify_smoke_standoff': 15.0,
    'verify_smoke_inspection_max_distance': 20.0,
    'verify_inspection_altitude': 3.0,
    'verify_inspection_lateral_step': 5.0,
    'verify_inspection_max_attempts': 3,
    'verify_inspection_yaw_step': math.radians(5.0),
    'verify_inspection_timeout': 600.0,
    'verify_inspection_max_range': 40.0,
    'verify_inspection_max_uncertainty': 2.0,
    'verify_geometry_timeout': 5.0,
    'verify_max_approaches': 64,
}


class VerificationObservationLost(RuntimeError):
    """A motion's perception guard failed; stop and reacquire from LOITER."""


class VerificationStartUnavailable(VerificationObservationLost):
    """Current map clearance is unavailable; changing the destination cannot fix it."""


class VerificationMapUnavailable(VerificationStartUnavailable):
    """No fresh map is available; hold for newer source observations."""

    def __init__(self, message, source_stamp_ns=0):
        super().__init__(message)
        self.source_stamp_ns = source_stamp_ns


class VerificationRecoveryUnavailable(VerificationObservationLost):
    """The body is observed free, but no bounded clearance recovery was found."""


class _FireFound(VerificationObservationLost):
    """Interrupt a bounded search as soon as a persistent fire is available."""


class _SmokeCenterTimeout(VerificationObservationLost):
    """Stop a preflight correction when its centering budget expires."""


class _InspectionTimeout(VerificationObservationLost):
    """Stop an inspection motion at the overall inspection deadline."""


def _bb_get(blackboard, key, default):
    # YASMIN's Blackboard.get accepts one argument, unlike dict.get.
    try:
        return blackboard[key]
    except (KeyError, RuntimeError):
        return default


class VerifyController:
    """Own perception lifetime and the four Verify sub-state operations."""

    def __init__(self, executive, error_type=RuntimeError):
        """Bind guarded flight helpers and initialize source-image tracking."""
        self.executive = executive
        self.error_type = error_type
        self.config = {**VERIFY_DEFAULTS, **executive.config}
        self._lock = threading.Lock()
        self.reset()

    def reset(self):
        """Discard observations at the beginning of a new action goal."""
        self._height_clock_ns = self._now_ns()
        self._height_clock_progress_at = time.monotonic()
        self._height_clock_epoch = 0
        with self._lock:
            self._epoch_ns = self._now_ns()
            self._observations = {'smoke': None, 'fire': None}
            self._last_stamps = {'smoke': 0, 'fire': 0}
            self._fire_start_ns = 0
            self._fire_frames = 0
            self._fire_reference = None
            self._fire_confirmation = None
            self._fire_confirmation_after_ns = 0
            self._fire_reset_reason = 'no confirmed fire images'
            self._observation_info = {
                source: {'received': 0, 'accepted': 0, 'received_at': None,
                         'stamp_ns': 0, 'status': None, 'detected': None,
                         'detector_frames': 0, 'detector_persistence': 0.0,
                         'detector_track': 0,
                         'reason': 'no messages received'}
                for source in ('smoke', 'fire')
            }
            self._smoke_seen_at = None
            self._smoke_last_confirmed = None
            self._smoke_bearing = None
            self._smoke_bearing_frame = None
            self._smoke_bearing_stamp = 0

    def _now_ns(self):
        return self.executive.get_clock().now().nanoseconds

    def discard_fire_confirmation(self, before_ns=None):
        """End an inspection's pending intent without changing live observations."""
        with self._lock:
            if before_ns is None:
                self._fire_confirmation = None
                self._fire_confirmation_after_ns = 0
            else:
                self._fire_confirmation_after_ns = max(
                    self._fire_confirmation_after_ns, before_ns)
                if (self._fire_confirmation is not None
                        and self._fire_confirmation[1] <= self._fire_confirmation_after_ns):
                    self._fire_confirmation = None

    def had_fire_confirmation(self):
        """Prevent an absent-fire report after confirmed fire in this inspection."""
        with self._lock:
            return self._fire_confirmation is not None

    def _valid_stamp(self, message):
        if self._stamp_issue(message):
            return None
        return self._stamp_ns(message)

    def _stamp_issue(self, message):
        stamp = message.header.stamp
        source_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        now_ns = self._now_ns()
        age = (now_ns - source_ns) / 1e9
        if now_ns <= 0:
            return 'ROS clock is uninitialized'
        if source_ns <= 0:
            return 'image timestamp is uninitialized'
        if source_ns <= self._epoch_ns:
            return 'image predates the mission'
        if age < -self.config['verify_future_tolerance']:
            return 'image timestamp is in the future'
        if age > self.config['verify_observation_timeout']:
            return 'source image is too old'
        return ''

    @staticmethod
    def _confirmed(message, source):
        confirmed = getattr(message, 'CONFIRMED', 3 if source == 'smoke' else 2)
        if not message.detected or message.status != confirmed:
            return False
        if source == 'smoke' and not message.background_ready:
            return False
        if not all(math.isfinite(getattr(message, field)) for field in (
                'yaw_correction_rad', 'vertical_correction_rad')):
            return False
        if source == 'smoke':
            return math.isfinite(message.coverage) and 0.0 <= message.coverage <= 1.0
        return (
            message.image_width > 0 and message.image_height > 0
            and 0 < message.area_px <= message.image_width * message.image_height
            and math.isfinite(message.center_x) and math.isfinite(message.center_y)
            and 0 <= message.center_x < message.image_width
            and 0 <= message.center_y < message.image_height
        )

    @staticmethod
    def _fire_geometry(message):
        return (
            message.center_x / message.image_width,
            message.center_y / message.image_height,
            message.area_px / (message.image_width * message.image_height),
        )

    def _same_fire(self, previous, message):
        if previous.header.frame_id != message.header.frame_id:
            return False
        if getattr(previous, 'track_id', 0) != getattr(message, 'track_id', 0):
            return False
        px, py, pa = self._fire_geometry(previous)
        x, y, area = self._fire_geometry(message)
        return (
            math.hypot(px - x, py - y) <= self.config['verify_track_center_tolerance']
            and max(pa, area) / min(pa, area)
            <= self.config['verify_track_area_ratio']
        )

    def _producer_fire_confirmed(self, message):
        """Use persistence already measured by the detector's source images."""
        persistence = getattr(message, 'persistence_sec', 0.0)
        return (
            bool(message.header.frame_id.strip())
            and getattr(message, 'track_id', 0) > 0
            and math.isfinite(persistence)
            and persistence >= self.config['verify_fire_persistence']
            and getattr(message, 'confirmation_frames', 0)
            >= self.config['verify_fire_min_frames']
        )

    def _receive(self, message, source):
        with self._lock:
            info = self._observation_info[source]
            info.update(
                received=info['received'] + 1, received_at=time.monotonic(),
                stamp_ns=self._stamp_ns(message), status=message.status,
                detected=bool(message.detected),
                detector_frames=getattr(message, 'confirmation_frames', 0),
                detector_persistence=getattr(message, 'persistence_sec', 0.0),
                detector_track=getattr(message, 'track_id', 0),
            )
            source_ns = self._valid_stamp(message)
            if source_ns is None:
                info['reason'] = self._stamp_issue(message) or 'invalid image timestamp'
                return
            if source_ns <= self._last_stamps[source]:
                info['reason'] = 'duplicate or reordered image'
                return  # Duplicate/reordered images cannot establish persistence.
            self._last_stamps[source] = source_ns
            if not self._confirmed(message, source):
                if not message.detected:
                    info['reason'] = 'detected=false'
                elif message.status != getattr(
                        message, 'CONFIRMED', 3 if source == 'smoke' else 2):
                    info['reason'] = 'detection is not confirmed'
                elif source == 'smoke' and not message.background_ready:
                    info['reason'] = 'background calibration is not ready'
                else:
                    info['reason'] = 'invalid detection geometry or guidance'
                self._observations[source] = None
                if source == 'fire':
                    self._fire_frames = 0
                    self._fire_reference = None
                    self._fire_reset_reason = info['reason']
                return
            previous = self._observations[source]
            now = time.monotonic()
            if source == 'fire':
                continuous = (
                    previous is not None and self._fire_reference is not None
                    and (source_ns - previous[1]) / 1e9
                    <= self.config['verify_observation_timeout']
                    and now - previous[2] <= self.config['verify_observation_timeout']
                    and self._same_fire(self._fire_reference, message)
                )
                if not continuous:
                    if previous is None or self._fire_reference is None:
                        self._fire_reset_reason = (
                            'starting confirmed track after ' + self._fire_reset_reason)
                    elif not self._same_fire(self._fire_reference, message):
                        self._fire_reset_reason = 'track identity or geometry changed'
                    else:
                        self._fire_reset_reason = 'confirmed image gap exceeded timeout'
                    self._fire_start_ns = source_ns
                    self._fire_frames = 0
                    self._fire_reference = deepcopy(message)
                self._fire_frames += 1
            self._observations[source] = (deepcopy(message), source_ns, now)
            if source == 'smoke':
                self._smoke_seen_at = now
                self._smoke_last_confirmed = deepcopy(message)
            if source == 'fire' and source_ns > self._fire_confirmation_after_ns and (
                    self._producer_fire_confirmed(message)
                    or (self._fire_frames >= self.config['verify_fire_min_frames']
                        and (source_ns - self._fire_start_ns) / 1e9
                        >= self.config['verify_fire_persistence'])):
                # Intent survives a brief detector miss. This event is never
                # returned as current steering guidance or report evidence.
                self._fire_confirmation = (deepcopy(message), source_ns, now)
            info['accepted'] += 1
            info['reason'] = 'accepted confirmed image'

    def observation_diagnostics(self, source='smoke'):
        """Explain missing guidance without printing on every detector frame."""
        with self._lock:
            info = dict(self._observation_info[source])
            observation = self._observations[source]
            reason = info['reason']
            if observation is not None:
                reason = self._stamp_issue(observation[0]) or reason
                if time.monotonic() - observation[2] > self.config['verify_observation_timeout']:
                    reason = 'confirmed observation reception expired'
            persistence = ''
            if source == 'fire':
                span = (max(0.0, (observation[1] - self._fire_start_ns) / 1e9)
                        if observation is not None else 0.0)
                message = self._fresh_locked('fire')
                producer_ready = message is not None and self._producer_fire_confirmed(message)
                ready = (message is not None and (producer_ready or (
                    self._fire_frames >= self.config['verify_fire_min_frames']
                    and span >= self.config['verify_fire_persistence'])))
                persistence = (
                    f'; consumer_frames={self._fire_frames}/'
                    f'{self.config["verify_fire_min_frames"]}, '
                    f'consumer_persistence={span:.2f}/'
                    f'{self.config["verify_fire_persistence"]:.2f}s, '
                    f'ready={ready}, detector_ready={producer_ready}, '
                    f'detector_frames={info["detector_frames"]}, '
                    f'detector_persistence={info["detector_persistence"]:.2f}s, '
                    f'detector_track={info["detector_track"]}, '
                    f'reset={self._fire_reset_reason}')
        if info['received_at'] is None:
            return f'{source}: no messages received'
        age = (self._now_ns() - info['stamp_ns']) / 1e9
        gap = time.monotonic() - info['received_at']
        return (
            f'{source}: {reason}; received={info["received"]}, accepted={info["accepted"]}, '
            f'status={info["status"]}, detected={info["detected"]}, '
            f'image_age={age:.2f}s, receive_gap={gap:.2f}s{persistence}'
        )

    def smoke_callback(self, message):
        """Accept confirmed smoke in any region; region is not a validity gate."""
        self._receive(message, 'smoke')

    def fire_callback(self, message):
        """Track one consistent fire across distinct, fresh source images."""
        self._receive(message, 'fire')

    def _fresh_locked(self, source):
        observation = self._observations[source]
        if observation is None:
            return None
        message, _, received = observation
        if (time.monotonic() - received > self.config['verify_observation_timeout']
                or self._valid_stamp(message) is None):
            return None
        return deepcopy(message)

    def fresh_smoke(self):
        """Return a private copy of the latest fresh, confirmed smoke image."""
        with self._lock:
            return self._fresh_locked('smoke')

    def fresh_fire(self):
        """Return a private copy of the latest fresh, confirmed fire image."""
        with self._lock:
            return self._fresh_locked('fire')

    def stable_fire(self):
        """Return a fresh fire only after this consumer saw its persistence."""
        with self._lock:
            message = self._fresh_locked('fire')
            observation = self._observations['fire']
            if message is None or observation is None:
                return None
            if (not self._producer_fire_confirmed(message) and (
                    self._fire_frames < self.config['verify_fire_min_frames']
                    or (observation[1] - self._fire_start_ns) / 1e9
                    < self.config['verify_fire_persistence'])):
                return None
            return message

    def all_quadrants(self, smoke):
        """Require broad smoke coverage, not just a blob crossing image center."""
        return (
            smoke is not None
            and smoke.coverage >= self.config['verify_full_coverage']
            and (
                {1, 2, 3, 4}.issubset(set(smoke.regions))
                or all(math.isfinite(value) and value > 0.0
                       for value in smoke.quadrant_coverage)
            )
        )

    def approach_distance(self, smoke):
        """Map apparent coverage to a bounded movement, without claiming range."""
        fraction = min(1.0, max(0.0, smoke.coverage)
                       / self.config['verify_full_coverage'])
        return self.config['verify_step_max'] - fraction * (
            self.config['verify_step_max'] - self.config['verify_step_min'])

    def _pose(self):
        return deepcopy(self.executive._snapshot(require_armed=True)[1].pose.pose)

    def _check(self):
        self.executive._check_running()
        self._height_clock_progress()

    def _pause(self):
        self._check()
        self.executive._stop.wait(0.05)

    @staticmethod
    def _stamp_ns(message):
        if message is None:
            return 0
        stamp = message.header.stamp
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    def _after_motion(self, source, previous, deadline):
        """Require an image acquired after a correction before commanding again."""
        after_ns = max(self._now_ns(), self._stamp_ns(previous))
        deadline = min(deadline, time.monotonic() + self.config['verify_observation_timeout'])
        while time.monotonic() < deadline:
            self._check()
            message = self.fresh_smoke() if source == 'smoke' else self.fresh_fire()
            if message is not None and self._stamp_ns(message) > after_ns:
                return message
            if source == 'smoke' and self.stable_fire() is not None:
                return None  # Let REACQUIRE give the persistent fire priority.
            self._pause()
        return None

    def _smoke_guard(self):
        self._check()
        smoke = self.fresh_smoke()
        if smoke is None:
            raise VerificationObservationLost('Smoke observation lost')
        if self.all_quadrants(smoke):
            raise VerificationObservationLost('Smoke occupies all quadrants')

    def _visible_smoke_guard(self):
        self._check()
        if self.fresh_smoke() is None:
            raise VerificationObservationLost('Smoke observation lost')

    def _smoke_guidance(self, blackboard=None, after_ns=0):
        """Hold through brief occlusion; only fresh images can start another leg."""
        grace = self.config['verify_smoke_loss_grace']
        started = time.monotonic()
        reported = False
        while True:
            self._check()
            if blackboard is not None:
                self._inspection_guard(blackboard, interrupt_fire=True)
            fire = self.stable_fire()
            if fire is not None and self._stamp_ns(fire) > after_ns:
                return None
            smoke = self.fresh_smoke()
            if smoke is not None and self._stamp_ns(smoke) > after_ns:
                return smoke
            with self._lock:
                seen = self._smoke_seen_at
                glimpse = deepcopy(self._smoke_last_confirmed)
                consumed = self._smoke_bearing_stamp
            # A brief confirmed glimpse may be followed by a negative frame
            # between polls. Consume that new, still-fresh glimpse once; never
            # turn the five-second bearing memory into stale steering evidence.
            if (glimpse is not None and self._stamp_ns(glimpse) > max(after_ns, consumed)
                    and not self._stamp_issue(glimpse) and seen is not None
                    and time.monotonic() - seen <= self.config['verify_observation_timeout']):
                return glimpse
            remaining = (started if seen is None else seen) + grace - time.monotonic()
            if remaining <= 0.0:
                self.executive._report(
                    f'Verify: smoke absent for at least {grace:.1f} s; '
                    'starting bounded reacquisition')
                return None
            if not reported:
                self.executive._report(
                    'Verify: smoke briefly occluded; holding position and remembered bearing '
                    f'({grace:.1f} s grace)')
                reported = True
            self.executive._verify_hold(min(0.25, remaining), guard=self._check)

    def _remember_smoke_bearing(self, blackboard, smoke, geometry):
        """Keep a smoothed world bearing, never a guessed smoke distance."""
        raw = geometry.get('navigation_bearing')
        if raw is None or not math.isfinite(raw):
            raw = self.executive._verify_image_bearing(smoke)
        if raw is None or not math.isfinite(raw):
            raw = yaw_from_quaternion(self._pose().orientation)
        stamp = self._stamp_ns(smoke)
        with self._lock:
            if self._smoke_bearing is None or self._smoke_bearing_frame != smoke.header.frame_id:
                self._smoke_bearing = raw
            elif stamp != self._smoke_bearing_stamp:
                change = math.atan2(math.sin(raw - self._smoke_bearing),
                                    math.cos(raw - self._smoke_bearing))
                self._smoke_bearing += 0.25 * change
            self._smoke_bearing_frame = smoke.header.frame_id
            self._smoke_bearing_stamp = stamp
            bearing = self._smoke_bearing
        blackboard['verify_smoke_bearing'] = bearing
        return bearing, {**geometry, 'navigation_bearing': bearing}

    def _check_approach_limit(self, blackboard):
        if _bb_get(blackboard, 'verify_approaches', 0) >= self.config['verify_max_approaches']:
            raise self.error_type('Verify approach limit reached; holding for operator')

    def _settled_observation_after_ns(self):
        context = getattr(self.executive, '_verify_wall_observation_context', None)
        return 0 if context is None else context.get('settled_ns', 0)

    def _prepare_wall_observation(self, blackboard):
        """Bound stationary preparation before reading images or wall geometry."""
        if self.stable_fire() is not None:
            self._inspection_begin(blackboard)

        def guard():
            if self.stable_fire() is not None:
                self._inspection_begin(blackboard)
            self._check_inspection_deadline(blackboard)
            self._check_wall_recheck_deadline(blackboard)

        pending = (_bb_get(blackboard, 'verify_wall_recheck_deadline', None) is not None
                   or _bb_get(blackboard, 'verify_wall_recheck_after_settle', False))
        if self.executive._verify_prepare_wall_observation(guard=guard, align_fire=not pending):
            if _bb_get(blackboard, 'verify_wall_recheck_after_settle', False):
                blackboard['verify_wall_recheck_deadline'] = (
                    time.monotonic() + self.config['verify_wall_recheck_timeout'])
                blackboard['verify_wall_recheck_after_settle'] = False
                self.executive._report(
                    'Verify: retreat settled; starting stationary wall recheck '
                    f'for {self.config["verify_wall_recheck_timeout"]:g} s')
            return True
        return self._settling_unavailable(blackboard)

    def _settling_unavailable(self, blackboard):
        """Bound losses before and after a query without treating them as sensor misses."""
        def guard():
            if self.stable_fire() is not None:
                self._inspection_begin(blackboard)
            self._check_inspection_deadline(blackboard)
            self._check_wall_recheck_deadline(blackboard)

        guard()
        blackboard['verify_wall_latest'] = None
        failures = _bb_get(blackboard, 'verify_settle_failures', 0) + 1
        blackboard['verify_settle_failures'] = failures
        reason = getattr(self.executive, '_verify_wall_settle_reason', '') or 'vehicle not settled'
        self.executive._report(
            f'Verify: stationary observation withheld; {reason}; '
            f'bounded preparation {failures}/{self.config["verify_search_recovery_max"]}')
        if failures >= self.config['verify_search_recovery_max']:
            raise self.error_type(
                'Verify: stationary observation preparation exhausted; holding for operator')
        self.executive._verify_hold(0.25, guard=guard)
        return False

    def _wall_distance(self, wall):
        position = self._pose().position
        return wall['nearest'] - wall['uncertainty'] - sum(
            (getattr(position, axis) - getattr(wall['origin'], axis)) * wall['axis'][i]
            for i, axis in enumerate('xyz'))

    def _wall_geometry(self, blackboard, geometry=None):
        """Constrain this leg with its current qualified aperture measurement."""
        geometry = {**(geometry or {}), 'wall_probe_only': True}
        walls = _bb_get(blackboard, 'verify_smoke_walls', [])
        if walls:
            geometry['smoke_navigation_wall'] = walls[-1]
            geometry['smoke_navigation_walls'] = list(walls)
        return geometry

    def _retain_wall(self, blackboard, wall):
        # A new observation replaces the previous candidate. Moving smoke need
        # not describe the same fixed plane; none of these planes prove arrival.
        blackboard['verify_smoke_wall_probe'] = wall
        blackboard['verify_smoke_walls'] = [wall]

    def _record_wall_observation(self, blackboard, wall):
        """Confirm proximity using two newly supported near measurements."""
        self._check_wall_recheck_deadline(blackboard)
        if not self.executive._verify_wall_observation_ready():
            blackboard['verify_wall_latest'] = None
            self.executive._report(
                'Verify: wall evidence withheld because the vehicle is not settled')
            return
        if wall is None:
            blackboard['verify_wall_latest'] = None
            pending = _bb_get(blackboard, 'verify_wall_recheck_deadline', None) is not None
            confirmed = _bb_get(blackboard, 'verify_smoke_boundary_reached', False)
            if not pending or (
                    confirmed and _bb_get(blackboard, 'verify_wall_near_count', 0) == 2):
                blackboard['verify_wall_near_count'] = 0
            if not pending:
                # Confirmed arrival needs fresh revalidation after a gap. A
                # pending vote, however, survives gaps inside its original
                # bounded window; it is historical evidence, never live range.
                blackboard['verify_wall_close_count'] = 0
                if confirmed:
                    blackboard['verify_wall_recheck_deadline'] = (
                        time.monotonic() + self.config['verify_wall_recheck_timeout'])
                else:
                    blackboard['verify_smoke_walls'] = []
            return
        stamp = wall.get('map_stamp_ns', 0)
        last = _bb_get(blackboard, 'verify_wall_last_stamp_ns', 0)
        if stamp < last:
            self.executive._report('Verify: reordered wall observation ignored')
            return
        if stamp == last:
            previous = _bb_get(blackboard, 'verify_wall_latest', None) or _bb_get(
                blackboard, 'verify_smoke_wall_probe', None)
            if previous is None or self._wall_distance(wall) < self._wall_distance(previous):
                if _bb_get(blackboard, 'verify_wall_latest', None) is not None:
                    blackboard['verify_wall_latest'] = wall
                self._retain_wall(blackboard, wall)
            self.executive._report('Verify: repeated wall observation; confirmation unchanged')
            return
        blackboard['verify_wall_latest'] = wall
        self._retain_wall(blackboard, wall)
        blackboard['verify_wall_last_stamp_ns'] = stamp
        distance = self._wall_distance(wall)
        limit = self.config['verify_smoke_inspection_max_distance']
        self.executive._report(
            f'Verify: qualified wall range mean={wall["mean"]:.2f} m, '
            f'nearest={wall["nearest"]:.2f} m, uncertainty={wall["uncertainty"]:.2f} m, '
            f'conservative distance={distance:.2f} m; '
            f'inspection band={self.config["verify_smoke_standoff"]:g}–{limit:g} m')
        if distance > limit:
            was_near = (_bb_get(blackboard, 'verify_smoke_boundary_reached', False)
                        or _bb_get(blackboard, 'verify_wall_recheck_deadline', None) is not None)
            blackboard['verify_wall_near_count'] = 0
            blackboard['verify_wall_close_count'] = 0
            blackboard['verify_smoke_boundary_reached'] = False
            blackboard['verify_wall_recheck_deadline'] = None
            blackboard['verify_search_active'] = False
            blackboard['verify_inspection_origin'] = None
            blackboard['verify_inspection_attempts'] = 0
            if was_near:
                self.executive._report(
                    'Verify: fresh wall moved beyond inspection band; resuming approach')
            return
        count = min(2, _bb_get(blackboard, 'verify_wall_near_count', 0) + 1)
        blackboard['verify_wall_near_count'] = count
        blackboard['verify_wall_close_count'] = (
            min(2, _bb_get(blackboard, 'verify_wall_close_count', 0) + 1)
            if distance < self.config['verify_smoke_standoff'] else 0)
        close_count = blackboard['verify_wall_close_count']
        self.executive._report(
            f'Verify: wall confirmation {count}/2 from new map observations'
            + (f'; close confirmation {close_count}/2 before retreat'
               if distance < self.config['verify_smoke_standoff'] else ''))
        if close_count == 2 or (count == 2 and distance >= self.config['verify_smoke_standoff']):
            blackboard['verify_smoke_boundary_reached'] = True
            blackboard['verify_wall_recheck_deadline'] = None
            self._inspection_begin(blackboard)
        elif _bb_get(blackboard, 'verify_wall_recheck_deadline', None) is None:
            blackboard['verify_wall_recheck_deadline'] = (
                time.monotonic() + self.config['verify_wall_recheck_timeout'])

    def _probe_smoke_wall(self, blackboard, message, bearing):
        """Probe once; missing distant range leaves normal approach available."""
        wall = self.executive._verify_opportunistic_wall(message.header.frame_id, bearing)
        self._record_wall_observation(blackboard, wall)
        return wall

    def _retain_smoke_geometry(self, blackboard, geometry):
        # A single image ray remains obstacle metadata. It is not the qualified
        # aperture evidence required for the smoke standoff or arrival phase.
        return None

    def _at_known_wall(self, blackboard):
        """Require confirmed, current proximity instead of projecting an old plane."""
        wall = _bb_get(blackboard, 'verify_wall_latest', None)
        return bool(_bb_get(blackboard, 'verify_smoke_boundary_reached', False)
                    and _bb_get(blackboard, 'verify_wall_near_count', 0) >= 2
                    and wall is not None
                    and self.executive._verify_wall_observation_ready()
                    and self.executive._verify_smoke_wall_at_standoff(wall))

    def _check_wall_recheck_deadline(self, blackboard):
        deadline = _bb_get(blackboard, 'verify_wall_recheck_deadline', None)
        if deadline is not None and time.monotonic() >= deadline:
            raise self.error_type(
                'Verify: near-wall revalidation exhausted; inspection incomplete; '
                'holding for operator')

    def _wait_for_wall(self, blackboard):
        if _bb_get(blackboard, 'verify_wall_recheck_after_settle', False):
            self.executive._verify_hold(
                0.25, guard=lambda: self._check_inspection_deadline(blackboard))
            return 'reacquire'
        deadline = _bb_get(blackboard, 'verify_wall_recheck_deadline', None)
        if deadline is None:
            deadline = time.monotonic() + self.config['verify_wall_recheck_timeout']
            blackboard['verify_wall_recheck_deadline'] = deadline
        self._check_wall_recheck_deadline(blackboard)
        self.executive._report('Verify: near-wall evidence unavailable; rechecking')
        self.executive._verify_hold(
            min(0.25, deadline - time.monotonic()),
            guard=lambda: self._check_inspection_deadline(blackboard))
        return 'reacquire'

    def _wall_action(self, blackboard):
        """Keep clearance immediately; reserve inspection for confirmed arrival."""
        if not self.executive._verify_wall_observation_ready():
            self._settling_unavailable(blackboard)
            return 'reacquire'
        # Preparation alone is not progress if its proof is lost during ranging.
        blackboard['verify_settle_failures'] = 0
        wall = _bb_get(blackboard, 'verify_wall_latest', None)
        limit = self.config['verify_smoke_inspection_max_distance']
        if wall is not None:
            if self._wall_distance(wall) < self.config['verify_smoke_standoff']:
                if self._retreat_from_wall(blackboard):
                    return 'reacquire'
                self.executive._report(
                    'Verify: tentative close wall; holding for a second fresh close '
                    'observation before wall-based retreat')
                return self._wait_for_wall(blackboard)
            if self._wall_distance(wall) <= limit:
                if self._at_known_wall(blackboard):
                    return 'ready'
                return self._wait_for_wall(blackboard)
            # A genuinely new farther measurement clears the near phase.
            if (not _bb_get(blackboard, 'verify_smoke_boundary_reached', False)
                    and _bb_get(blackboard, 'verify_wall_recheck_deadline', None) is None):
                return 'approach'
        if (_bb_get(blackboard, 'verify_smoke_boundary_reached', False)
                or _bb_get(blackboard, 'verify_wall_recheck_deadline', None) is not None):
            return self._wait_for_wall(blackboard)
        return 'approach'

    def _retreat_from_wall(self, blackboard):
        if _bb_get(blackboard, 'verify_wall_close_count', 0) < 2:
            return False
        walls = _bb_get(blackboard, 'verify_smoke_walls', [])
        if not walls:
            return False
        wall = min(walls, key=self._wall_distance)
        distance = self._wall_distance(wall)
        if distance >= self.config['verify_smoke_standoff']:
            return False
        target = self._pose()
        retreat = min(self.config['verify_navigation_step'],
                      self.config['verify_smoke_standoff']
                      + self.config.get('position_tolerance', 0.3) - distance)
        for i, axis in enumerate('xyz'):
            setattr(target.position, axis,
                    getattr(target.position, axis) - retreat * wall['axis'][i])
        self.executive._report(
            'Verify: confirmed close wall on two fresh maps; attempting checked retreat')
        moved = self.executive._verify_recovery_plan_to(
            target, geometry={**self._wall_geometry(blackboard),
                              'safety_recovery_reason': 'confirmed_smoke_wall_inside_standoff',
                              'confirmed_wall_retreat': True},
            guard=lambda: self._movement_guard(blackboard), preserve_requested_altitude=True)
        if moved is False:
            raise self.error_type('Verify: measured-wall retreat blocked; holding for operator')
        blackboard['verify_inspection_after_ns'] = self._now_ns()
        # The bounded recheck is a stationary sensor decision. A necessary
        # validated retreat must not use up that window before it can begin.
        blackboard['verify_wall_latest'] = None
        blackboard['verify_wall_near_count'] = 0
        blackboard['verify_wall_close_count'] = 0
        blackboard['verify_wall_recheck_deadline'] = None
        blackboard['verify_wall_recheck_after_settle'] = True
        self.executive._report(
            'Verify: confirmed-wall retreat completed; waiting for physical settling '
            'before stationary wall recheck')
        return True

    def _capture_height_hint(self, blackboard):
        """Capture confirmed guidance and pose on the mission worker, never a callback."""
        fire = self.stable_fire()
        if fire is None:
            with self._lock:
                event = deepcopy(self._fire_confirmation)
            if event is None or time.monotonic() - event[2] > self.config[
                    'verify_observation_timeout'] or self._stamp_issue(event[0]):
                return
            fire = event[0]
        hint = _bb_get(blackboard, 'verify_height_hint', None)
        stamp = self._stamp_ns(fire)
        if stamp <= _bb_get(blackboard, 'verify_height_hint_consumed', 0):
            return
        if hint is None or stamp > self._stamp_ns(hint[0]):
            with self._lock:
                observation = self._observations['fire']
                event = self._fire_confirmation
                received = (observation[2] if observation is not None and observation[1] == stamp
                            else event[2] if event is not None and event[1] == stamp
                            else time.monotonic())
            blackboard['verify_height_hint'] = (fire, self._pose(), received)
            blackboard['verify_height_hint_epoch'] = self._height_clock_epoch

    def _height_clock_progress(self):
        """Track source-clock progress independently of simulation's wall-time speed."""
        now_ns = self._now_ns()
        if now_ns != self._height_clock_ns:
            if now_ns < self._height_clock_ns:
                self._height_clock_epoch += 1
            self._height_clock_ns = now_ns
            self._height_clock_progress_at = time.monotonic()
        silence = time.monotonic() - self._height_clock_progress_at
        simulated = bool(getattr(self.executive.get_clock(), 'ros_time_is_active', False))
        return (now_ns > 0 and (not simulated or silence <= self.config[
            'verify_vertical_clock_timeout'])), silence

    def _begin_forward_leg(self, blackboard):
        """Discard unused arrival guidance before selecting another forward leg."""
        hint = _bb_get(blackboard, 'verify_height_hint', None)
        if hint is not None:
            stamp = self._stamp_ns(hint[0])
            blackboard['verify_height_hint_consumed'] = max(
                stamp, _bb_get(blackboard, 'verify_height_hint_consumed', 0))
            blackboard['verify_height_hint'] = None
            self.executive._report(
                'Verify: discarding unused fire-height hint before selecting another forward leg')

    def _movement_guard(self, blackboard):
        self._check_inspection_deadline(blackboard)
        self._capture_height_hint(blackboard)

    def _height_floor_rejected(self, vertical):
        """Recognize only a downward preference vetoed before flight by terrain."""
        return vertical < 0.0 and getattr(
            self.executive, '_verify_height_rejection', '') == 'terrain_floor'

    def _defer_descent(self, blackboard):
        """Allow one checked approach leg before reconsidering a blocked descent."""
        if _bb_get(blackboard, 'verify_smoke_boundary_reached', False):
            return
        if not _bb_get(blackboard, 'verify_descent_deferred', False):
            self.executive._report(
                'Verify: downward fire-height correction skipped at terrain clearance floor; '
                'deferring descent until one checked forward leg completes')
        blackboard['verify_descent_deferred'] = True

    def _check_inspection_deadline(self, blackboard):
        try:
            self._inspection_guard(blackboard)
        except _InspectionTimeout as error:
            raise self.error_type(
                'Verify: inspection time budget exhausted; inspection incomplete; '
                'holding for operator') from error

    def _remembered_height_fire(self, blackboard):
        """Return one eligible height hint without authorizing horizontal movement."""
        clock_running, clock_silence = self._height_clock_progress()
        self._capture_height_hint(blackboard)
        if self.stable_fire() is not None:
            return False
        hint = _bb_get(blackboard, 'verify_height_hint', None)
        if hint is None:
            return False
        fire, pose, received = hint
        stamp = self._stamp_ns(fire)
        if stamp <= _bb_get(blackboard, 'verify_height_hint_consumed', 0):
            return False
        current = self._pose()
        distance = math.dist(tuple(getattr(current.position, a) for a in 'xyz'),
                             tuple(getattr(pose.position, a) for a in 'xyz'))
        turn = yaw_from_quaternion(current.orientation) - yaw_from_quaternion(pose.orientation)
        age = (self._now_ns() - stamp) / 1e9
        live = self.fresh_fire()
        turn = abs(math.atan2(math.sin(turn), math.cos(turn)))
        reasons = []
        if not clock_running:
            reasons.append('source clock unavailable or frozen')
        if _bb_get(blackboard, 'verify_height_hint_epoch', self._height_clock_epoch) != (
                self._height_clock_epoch):
            reasons.append('source clock rewound')
        if not 0.0 <= age <= self.config['verify_vertical_memory_timeout']:
            reasons.append('image age exceeds source-time limit')
        if distance > self.config['verify_navigation_step'] + self.config.get(
                'position_tolerance', 0.3):
            reasons.append('movement exceeds one short leg')
        if turn > math.radians(15.0):
            reasons.append('yaw change exceeds 15 degrees')
        threshold = self.config['verify_vertical_tolerance'] + self.config[
            'verify_vertical_hysteresis']
        if abs(fire.vertical_correction_rad) <= threshold:
            reasons.append('height error inside activation threshold')
        if live is not None and not self._same_fire(live, fire):
            reasons.append('fire track or camera changed')
        if reasons:
            self.executive._report(
                'Verify: fire-height hint rejected: ' + '; '.join(reasons) + '; '
                f'angle={math.degrees(fire.vertical_correction_rad):+.1f} deg, '
                f'image_age={age:.2f} source s, wall_age={time.monotonic() - received:.2f} s, '
                f'travel={distance:.2f} m, yaw_change={math.degrees(turn):.1f} deg, '
                f'clock_silence={clock_silence:.2f} s, track={getattr(fire, "track_id", 0)}')
            return False
        return fire

    def _height_forward_limit(self, vertical, angle):
        """Shorten a combined leg when forward motion would overwhelm its height change."""
        if not vertical:
            return None
        slope = math.tan(abs(angle))
        if not math.isfinite(slope) or slope <= 0.0:
            return self.config['verify_navigation_step']
        return min(self.config['verify_navigation_step'], 0.8 * abs(vertical) / slope)

    def _attempt_approach_height(self, blackboard, vertical, *, remembered=False):
        """Try one optional height-only correction from this horizontal viewpoint."""
        if _bb_get(blackboard, 'verify_height_only_attempted', False):
            self.executive._report(
                'Verify: height-only correction already attempted from this viewpoint; '
                'requiring forward progress before another')
            return False
        if _bb_get(blackboard, 'verify_fire_approach_steps', 0) >= self.config[
                'verify_fire_approach_max_steps']:
            raise self.error_type(
                'Verify: fire approach step limit reached before smoke-wall standoff; '
                'inspection incomplete; holding for operator')
        blackboard['verify_height_only_attempted'] = True
        before = self._pose()
        target = deepcopy(before)
        target.position.z += vertical
        self.executive._report(
            f'Verify: {"remembered " if remembered else ""}height-only fire correction '
            f'{vertical:+.2f} m; one attempt from this viewpoint')
        self.executive._verify_height_rejection = ''
        moved = self.executive._verify_recovery_plan_to(
            target, geometry=self._wall_geometry(blackboard),
            guard=lambda: self._movement_guard(blackboard), preserve_requested_altitude=True)
        actual = self._pose().position.z - before.position.z
        minimum = self.config['verify_vertical_minimum_change'] - self.config[
            'verify_vertical_position_tolerance']
        useful = moved is not False and actual * math.copysign(1.0, vertical) >= minimum - 1e-6
        self.executive._report(
            f'Verify: height-only correction {"completed" if useful else "unavailable"}; '
            f'requested={vertical:+.2f} m, actual height change={actual:+.2f} m; '
            f'reason={getattr(self.executive, "_verify_height_rejection", "") or "path/progress"}')
        if not useful:
            if self._height_floor_rejected(vertical):
                self._defer_descent(blackboard)
            return False
        blackboard['verify_fire_approach_steps'] = _bb_get(
            blackboard, 'verify_fire_approach_steps', 0) + 1
        blackboard['verify_inspection_after_ns'] = self._now_ns()
        self._inspection_wait_fire(blackboard)
        return True

    def _recover_fire_height(self, blackboard):
        """Use remembered height alone only when no fresh bearing permits a combined leg."""
        if (_bb_get(blackboard, 'verify_wall_recheck_deadline', None) is not None
                or _bb_get(blackboard, 'verify_wall_recheck_after_settle', False)):
            # Wall reconfirmation holds all optional translation. Retain the
            # hint so a fresh far reading can release this hold without loss.
            return False
        fire = self._remembered_height_fire(blackboard)
        if fire is None or fire is False:
            return False
        after_ns = max(_bb_get(blackboard, 'verify_arrival_after_ns', 0),
                       _bb_get(blackboard, 'verify_inspection_after_ns', 0))
        smoke = self._smoke_guidance(after_ns=after_ns)
        if smoke is not None or self.stable_fire() is not None:
            return False
        fire = self._remembered_height_fire(blackboard)
        if not fire:
            return False
        if _bb_get(blackboard, 'verify_height_only_attempted', False):
            return False
        blackboard['verify_height_hint_consumed'] = self._stamp_ns(fire)
        self._inspection_begin(blackboard)
        blackboard['verify_fire_observed'] = True
        vertical = math.copysign(self.config['verify_vertical_step'], fire.vertical_correction_rad)
        return self._attempt_approach_height(blackboard, vertical, remembered=True)

    def _approach_observation(self, blackboard, after_ns=0):
        after_ns = max(after_ns, self._settled_observation_after_ns())
        fire = self.stable_fire()
        if fire is not None and self._stamp_ns(fire) > after_ns:
            if _bb_get(blackboard, 'verify_fire_loss_failures', 0):
                self._recover_fire_loss(blackboard, fire)
            return fire, 'fire'
        smoke = self._smoke_guidance(after_ns=after_ns)
        fire = self.stable_fire()
        if fire is not None and self._stamp_ns(fire) > after_ns:
            if _bb_get(blackboard, 'verify_fire_loss_failures', 0):
                self._recover_fire_loss(blackboard, fire)
            return fire, 'fire'
        return smoke, 'smoke'

    def _approach_bearing(self, blackboard, message, source, geometry):
        if source == 'smoke':
            return self._remember_smoke_bearing(blackboard, message, geometry)
        bearing = geometry.get('navigation_bearing')
        if bearing is None or not math.isfinite(bearing):
            bearing = self.executive._verify_image_bearing(message)
        if bearing is None or not math.isfinite(bearing):
            bearing = yaw_from_quaternion(self._pose().orientation)
        return bearing, {**geometry, 'navigation_bearing': bearing}

    def approach_smoke(self, blackboard):
        """Follow source guidance until two fresh observations qualify as near."""
        if not _bb_get(blackboard, 'verify_entered', False):
            self.executive._verify_parent_entry('Verify')
            with self._lock:
                self._smoke_bearing = None
                self._smoke_bearing_frame = None
            blackboard['verify_entered'] = True
        self._check()
        blackboard['verify_reference_pose'] = self._pose()
        try:
            if self._recover_fire_height(blackboard):
                return 'reacquire'
            if not self._prepare_wall_observation(blackboard):
                return 'reacquire'
            observation, source = self._approach_observation(blackboard, after_ns=max(
                _bb_get(blackboard, 'verify_arrival_after_ns', 0),
                _bb_get(blackboard, 'verify_inspection_after_ns', 0)))
            if observation is None:
                self._recover_fire_height(blackboard)
                return 'reacquire'
            if source == 'fire':
                self._inspection_begin(blackboard)
            geometry = self._inspection_geometry(blackboard, observation, source=source)
            if self._stamp_issue(observation):
                self.executive._report('Verify: image aged during geometry query; reacquiring')
                return 'reacquire'
            before = self._pose()
            bearing, geometry = self._approach_bearing(
                blackboard, observation, source, geometry)
            self._probe_smoke_wall(blackboard, observation, bearing)
            self._check_inspection_deadline(blackboard)
            action = self._wall_action(blackboard)
            if action == 'ready':
                return self._route_fire(blackboard) if self.stable_fire() is not None else 'search'
            if action == 'reacquire':
                return action
            geometry = self._wall_geometry(blackboard, geometry)
            self._check_approach_limit(blackboard)
            fire = self.stable_fire()
            remembered = self._remembered_height_fire(blackboard) if fire is None else None
            height_fire = fire if fire is not None else remembered
            vertical = (self._fire_vertical_preference(blackboard, fire)
                        if fire is not None else math.copysign(
                            self.config['verify_vertical_step'],
                            remembered.vertical_correction_rad)
                        if remembered else 0.0)
            if vertical:
                if _bb_get(blackboard, 'verify_fire_approach_steps', 0) >= self.config[
                        'verify_fire_approach_max_steps']:
                    raise self.error_type(
                        'Verify: fire approach step limit reached before smoke-wall standoff; '
                        'inspection incomplete; holding for operator')
                blackboard['verify_height_hint_consumed'] = self._stamp_ns(height_fire)
                blackboard['verify_fire_observed'] = True
                self._inspection_begin(blackboard)
            elif fire is None:
                self.executive._report(
                    'Verify: fire-height guidance unavailable; '
                    + self.observation_diagnostics('fire'))
            self._begin_forward_leg(blackboard)

            def guard():
                self._movement_guard(blackboard)
            if vertical:
                forward_limit = self._height_forward_limit(
                    vertical, height_fire.vertical_correction_rad)
                self.executive._report(
                    'Verify: trying combined fire-guided leg; '
                    f'forward limit={forward_limit:.2f} m, '
                    f'requested height change={vertical:+.2f} m')
                moved = self.executive._verify_navigation_viewpoint(
                    bearing, geometry=geometry, guard=guard, forward_only=True,
                    maximum_forward=forward_limit, vertical_offset=vertical, allow_oblique=True)
                if moved is not False:
                    actual_height = self._pose().position.z - before.position.z
                    self.executive._report(
                        f'Verify: combined fire-guided leg completed; '
                        f'requested height change={vertical:+.2f} m, '
                        f'actual height change={actual_height:+.2f} m')
                    if actual_height * math.copysign(1.0, vertical) >= (
                            self.config['verify_vertical_minimum_change']
                            - self.config['verify_vertical_position_tolerance'] - 1e-6):
                        blackboard['verify_fire_approach_steps'] = _bb_get(
                            blackboard, 'verify_fire_approach_steps', 0) + 1
                if moved is False:
                    self.executive._report(
                        'Verify: combined fire-guided leg unavailable; '
                        'trying one height-only correction')
                    if self._attempt_approach_height(
                            blackboard, vertical, remembered=remembered is not None):
                        return 'reacquire'
                    self.executive._report(
                        'Verify: height-only correction unavailable; '
                        'trying terrain-following forward fallback')
                    if not self._prepare_wall_observation(blackboard):
                        return 'reacquire'
                    observation, source = self._approach_observation(blackboard, after_ns=max(
                        _bb_get(blackboard, 'verify_arrival_after_ns', 0),
                        _bb_get(blackboard, 'verify_inspection_after_ns', 0)))
                    if observation is None:
                        return 'reacquire'
                    geometry = self._inspection_geometry(blackboard, observation, source=source)
                    if self._stamp_issue(observation):
                        return 'reacquire'
                    bearing, geometry = self._approach_bearing(
                        blackboard, observation, source, geometry)
                    self._probe_smoke_wall(blackboard, observation, bearing)
                    self._check_inspection_deadline(blackboard)
                    action = self._wall_action(blackboard)
                    if action == 'ready':
                        return (self._route_fire(blackboard)
                                if self.stable_fire() is not None else 'search')
                    if action == 'reacquire':
                        return action
                    geometry = self._wall_geometry(blackboard, geometry)
                    moved = self.executive._verify_navigation_viewpoint(
                        bearing, geometry=geometry, guard=guard, forward_only=True,
                        vertical_offset=0.0, allow_oblique=True)
            else:
                self.executive._report(
                    'Verify: source visible; following a short terrain-aware approach leg')
                moved = self.executive._verify_navigation_viewpoint(
                    bearing, geometry=geometry, guard=guard, forward_only=True,
                    vertical_offset=0.0, allow_oblique=True)
            if moved is False:
                reason = getattr(self.executive, '_verify_navigation_failure_reason',
                                 'no validated forward smoke path')
                return self._search_blocked(blackboard, reason)
            arrived = self._pose().position
            displacement = math.dist((arrived.x, arrived.y, arrived.z),
                                     (before.position.x, before.position.y, before.position.z))
            if displacement <= self.config.get('position_tolerance', 0.3):
                return self._search_blocked(blackboard, 'smoke leg did not change viewpoint')
            blackboard['verify_approaches'] = _bb_get(blackboard, 'verify_approaches', 0) + 1
            if not self._navigation_progress(blackboard, before, bearing, moved):
                return self._navigation_stalled(blackboard)
            blackboard['verify_reference_pose'] = self._pose()
            blackboard['verify_arrival_after_ns'] = self._now_ns()
            blackboard['verify_smoke_approach_completed'] = True
            blackboard['verify_smoke_approach_after_ns'] = self._now_ns()
            blackboard['verify_inspection_after_ns'] = self._now_ns()
            self.executive._report(
                'Verify: approach destination reached; reacquiring smoke or fire')
            return 'reacquire'
        except _FireFound:
            return 'reacquire'
        except (VerificationStartUnavailable, VerificationRecoveryUnavailable) as error:
            return self._navigation_start_unavailable(blackboard, error, resume='approach')
        except VerificationObservationLost:
            blackboard['verify_reference_pose'] = self._pose()
            blackboard['verify_inspection_after_ns'] = self._now_ns()
            return 'reacquire'

    def _reacquire_fire(self, blackboard, after_ns=0):
        """Use current confirmed images, including before the first smoke leg."""
        fire = self.stable_fire()
        after_ns = max(after_ns, _bb_get(blackboard, 'verify_inspection_after_ns', 0))
        if fire is not None and self._stamp_ns(fire) > after_ns:
            if _bb_get(blackboard, 'verify_fire_loss_failures', 0):
                self._recover_fire_loss(blackboard, fire)
            return fire
        return None

    def reacquire(self, blackboard):
        """Wait for post-stop guidance without discarding a confirmed fire target."""
        self._check_inspection_deadline(blackboard)
        self._check_wall_recheck_deadline(blackboard)
        if _bb_get(blackboard, 'verify_wall_recheck_after_settle', False):
            if not self._prepare_wall_observation(blackboard):
                return 'reacquire'
        if self._recover_fire_height(blackboard):
            return 'reacquire'
        after_ns = max(_bb_get(blackboard, 'verify_arrival_after_ns', 0),
                       _bb_get(blackboard, 'verify_inspection_after_ns', 0),
                       self._settled_observation_after_ns())
        blackboard['verify_arrival_after_ns'] = 0
        if _bb_get(blackboard, 'verify_wall_recheck_deadline', None) is not None:
            if self._reacquire_fire(blackboard, after_ns) is not None:
                return self._reacquire_result(blackboard, None)
            smoke = self.fresh_smoke()
            if smoke is not None and self._stamp_ns(smoke) > after_ns:
                return self._reacquire_result(blackboard, smoke)
            return self._wait_for_wall(blackboard)
        self._at_known_wall(blackboard)
        self.executive._report('Verify: holding arrival viewpoint for detector confirmation')
        smoke = self._smoke_guidance(after_ns=after_ns)
        if self._reacquire_fire(blackboard, after_ns) is not None:
            return self._reacquire_result(blackboard, None)
        if smoke is not None:
            return self._reacquire_result(blackboard, smoke)
        # Only a missing source permits a bounded yaw scan, never a blind translation.
        reference = _bb_get(blackboard, 'verify_reference_pose', None) or self._pose()
        with self._lock:
            remembered = self._smoke_bearing
        yaw = remembered if remembered is not None else yaw_from_quaternion(reference.orientation)
        scan = self.config['verify_reacquire_scan_step']

        def scan_guard():
            self._check()
            if self.stable_fire() is not None:
                raise _FireFound('Fire appeared during smoke reacquisition')
            if self.fresh_smoke() is not None:
                raise VerificationObservationLost('Smoke returned; stopping reacquisition scan')

        for offset in (scan, -scan, 0.0):
            self._check()
            if self._reacquire_fire(blackboard, after_ns) is not None:
                return self._reacquire_result(blackboard, None)
            smoke = self.fresh_smoke()
            if smoke is not None and self._stamp_ns(smoke) > after_ns:
                return self._reacquire_result(blackboard, smoke)
            try:
                self.executive._verify_rotate(yaw + offset, guard=scan_guard, hold_duration=0.5)
            except _FireFound:
                return self._reacquire_result(blackboard, None)
            except VerificationObservationLost:
                pass
            after_ns = self._now_ns()
            smoke = self._smoke_guidance(after_ns=after_ns)
            if self._reacquire_fire(blackboard, after_ns) is not None:
                return self._reacquire_result(blackboard, None)
            if smoke is not None:
                return self._reacquire_result(blackboard, smoke)
        if (_bb_get(blackboard, 'verify_fire_observed', False) or self.had_fire_confirmation()
                or _bb_get(blackboard, 'verify_smoke_boundary_reached', False)):
            return self._search_blocked(
                blackboard, 'confirmed fire guidance has not recovered', fire_loss=True)
        blackboard['verify_exploration_required'] = True
        self.executive._report('Verify: no fresh smoke or fire after bounded reacquisition')
        return 'explore'

    def _reacquire_result(self, blackboard, smoke):
        fire = self._reacquire_fire(blackboard)
        if smoke is not None or fire is not None:
            if self._at_known_wall(blackboard):
                return self._route_fire(blackboard) if fire is not None else 'search'
            return 'approach'
        return 'reacquire'

    def _fire_guard(self, original, prevent_smoke=False):
        self._check()
        fire = self.fresh_fire()
        if (fire is None or fire.header.frame_id != original.header.frame_id
                or getattr(fire, 'track_id', 0) != getattr(original, 'track_id', 0)):
            raise VerificationObservationLost('Tracked fire observation lost')
        if prevent_smoke:
            self._smoke_guard()

    def _inspection_guard(self, blackboard, interrupt_fire=False):
        self._check()
        if getattr(self.executive, '_verify_wall_observation_context', None) is not None:
            self.executive._verify_wall_observation_ready()
        started = _bb_get(blackboard, 'verify_inspection_started', None)
        if started is not None and time.monotonic() - started >= self.config[
                'verify_inspection_timeout']:
            elapsed = time.monotonic() - started
            self.executive._report(
                f'Verify: inspection deadline exceeded; elapsed={elapsed:.1f} s, '
                f'budget={self.config["verify_inspection_timeout"]:.1f} s')
            raise _InspectionTimeout('Inspection deadline exceeded')
        if interrupt_fire and (self._inspection_fire(blackboard) is not None
                               or self._inspection_fire_event(blackboard)):
            raise _FireFound('Persistent fire found during inspection')

    def _inspection_begin(self, blackboard):
        if _bb_get(blackboard, 'verify_inspection_started', None) is None:
            blackboard['verify_inspection_started'] = time.monotonic()
            blackboard['verify_inspection_attempts'] = 0
            self.executive._report(
                f'Verify: inspection clock started; '
                f'budget={self.config["verify_inspection_timeout"]:.1f} wall seconds')

    def _inspection_fire(self, blackboard):
        fire = self.stable_fire()
        after_ns = max(_bb_get(blackboard, 'verify_inspection_after_ns', 0),
                       _bb_get(blackboard, 'verify_smoke_approach_after_ns', 0),
                       self._settled_observation_after_ns())
        if fire is not None and self._stamp_ns(fire) > after_ns:
            if _bb_get(blackboard, 'verify_fire_loss_failures', 0):
                self._recover_fire_loss(blackboard, fire)
            return fire
        return None

    def _recover_fire_loss(self, blackboard, fire):
        """End a fire-loss episode using newly accepted, settled fire guidance only."""
        if fire is None:
            return False
        after_ns = max(_bb_get(blackboard, 'verify_fire_loss_after_ns', 0),
                       _bb_get(blackboard, 'verify_inspection_after_ns', 0),
                       _bb_get(blackboard, 'verify_smoke_approach_after_ns', 0),
                       _bb_get(blackboard, 'verify_arrival_after_ns', 0),
                       self._settled_observation_after_ns())
        if (self._stamp_ns(fire) <= after_ns
                or not self.executive._verify_wall_observation_ready()):
            return False
        if _bb_get(blackboard, 'verify_fire_loss_failures', 0):
            blackboard['verify_fire_loss_failures'] = 0
            self.executive._report(
                'Verify: fresh settled fire guidance recovered; fire-loss recovery count reset')
        return True

    def _inspection_fire_event(self, blackboard):
        """Retain confirmation intent briefly, without reusing old guidance."""
        with self._lock:
            event = self._fire_confirmation
            if event is None:
                return False
            _, stamp_ns, received = event
            age = (self._now_ns() - stamp_ns) / 1e9
            timeout = self.config['verify_reacquire_observation_timeout']
            return (
                stamp_ns > max(_bb_get(blackboard, 'verify_fire_seen_ns', 0),
                               _bb_get(blackboard, 'verify_smoke_approach_after_ns', 0),
                               self._settled_observation_after_ns())
                and -self.config['verify_future_tolerance'] <= age <= timeout
                and 0.0 <= time.monotonic() - received <= timeout
            )

    def _route_fire(self, blackboard, after_motion=False):
        """Retain fire intent; source range never controls the fire branch."""
        with self._lock:
            event = self._fire_confirmation
            if event is not None:
                blackboard['verify_fire_seen_ns'] = event[1]
        blackboard['verify_fire_pending'] = True
        blackboard['verify_fire_observed'] = True
        if after_motion:
            blackboard['verify_inspection_after_ns'] = self._now_ns()
        return 'fire'

    def _inspection_observation(self, blackboard, after_ns=0):
        """Prefer current fire guidance; never reuse pre-arrival camera images."""
        if not self._prepare_wall_observation(blackboard):
            return None, None
        after_ns = max(after_ns, _bb_get(blackboard, 'verify_inspection_after_ns', 0),
                       _bb_get(blackboard, 'verify_smoke_approach_after_ns', 0),
                       self._settled_observation_after_ns())
        deadline = time.monotonic() + self.config['verify_reacquire_observation_timeout']
        while time.monotonic() < deadline:
            self._inspection_guard(blackboard)
            fire = self.stable_fire()
            smoke = self.fresh_smoke()
            if fire is not None and self._stamp_ns(fire) > after_ns:
                if _bb_get(blackboard, 'verify_fire_loss_failures', 0):
                    self._recover_fire_loss(blackboard, fire)
                self._route_fire(blackboard)
                return fire, 'fire'
            if smoke is not None and self._stamp_ns(smoke) > after_ns:
                return smoke, 'smoke'
            self._pause()
        return None, None

    def _inspection_wait_fire(self, blackboard):
        """Wait at the stopped viewpoint for fresh confirmation after a miss."""
        deadline = time.monotonic() + self.config['verify_reacquire_observation_timeout']
        while time.monotonic() < deadline:
            self._check()
            fire = self._inspection_fire(blackboard)
            if fire is not None:
                return fire
            self._pause()
        return None

    def _inspection_geometry(self, blackboard, message=None, source='smoke'):
        geometry = None
        if message is not None:
            geometry = self.executive._verify_target_geometry(message, source=source)
        blackboard['verify_inspection_geometry'] = geometry
        return geometry or {'point': None, 'ground_z': None, 'source': source}

    def _inspection_fire_center(self, blackboard, geometry=None):
        """Attempt both image axes; current visibility is sufficient to finish."""
        original = self._inspection_fire(blackboard)
        if original is None:
            return None
        deadline = time.monotonic() + self.config['verify_center_timeout']
        blocked = set()

        def guard():
            self._check()
            if time.monotonic() >= deadline:
                raise _SmokeCenterTimeout('Fire centering correction budget expired')
            self._fire_guard(original)

        # Alternate axes so a persistent horizontal error cannot starve vertical centering.
        for _ in range(self.config['verify_inspection_max_attempts']):
            attempted = False
            for axis in ('horizontal', 'vertical'):
                fire = self._inspection_fire(blackboard)
                if fire is None:
                    return None
                error = (fire.yaw_correction_rad if axis == 'horizontal'
                         else fire.vertical_correction_rad)
                if axis in blocked or abs(error) <= self.config['verify_yaw_tolerance']:
                    continue
                if time.monotonic() >= deadline:
                    return fire
                attempted = True
                pose = self._pose()
                try:
                    if axis == 'horizontal':
                        step = max(-self.config['verify_inspection_yaw_step'], min(
                            self.config['verify_inspection_yaw_step'], error))
                        self.executive._report('Verify: attempting horizontal fire centering')
                        self.executive._verify_rotate(
                            yaw_from_quaternion(pose.orientation) + step, guard=guard)
                    else:
                        target = deepcopy(pose)
                        target.position.z = max(
                            self.config['verify_minimum_altitude'],
                            pose.position.z + math.copysign(
                                self.config['verify_vertical_step'], error))
                        self.executive._report('Verify: attempting vertical fire centering')
                        if abs(target.position.z - pose.position.z) < 0.01:
                            blocked.add(axis)
                            continue
                        moved = self.executive._verify_recovery_plan_to(
                            target, guard=guard, preserve_requested_altitude=True)
                        if moved is False:
                            blocked.add(axis)
                            self.executive._report(
                                'Verify: vertical centering unavailable; retaining visible fire')
                            continue
                    blackboard['verify_inspection_after_ns'] = self._now_ns()
                    if self._after_motion('fire', fire, deadline) is None:
                        return None
                except (VerificationStartUnavailable, VerificationRecoveryUnavailable,
                        _SmokeCenterTimeout) as error:
                    blocked.add(axis)
                    self.executive._report(
                        f'Verify: {axis} centering incomplete: {error}; checking visible fire')
                    # A guarded correction may have recovered to LOITER; require a post-stop image.
                    blackboard['verify_inspection_after_ns'] = self._now_ns()
                    if self._inspection_wait_fire(blackboard) is None:
                        return None
                except VerificationObservationLost:
                    blocked.add(axis)
                    blackboard['verify_inspection_after_ns'] = self._now_ns()
                    if self._inspection_wait_fire(blackboard) is None:
                        return None
            if not attempted:
                break
        return self._inspection_fire(blackboard)

    def _inspection_center_with_grace(self, blackboard, geometry):
        """Retry centering after a brief miss using only post-stop fire images."""
        for _ in range(2):
            try:
                centered = self._inspection_fire_center(blackboard, geometry)
                if centered is not None:
                    return centered
            except _InspectionTimeout:
                raise
            except VerificationStartUnavailable:
                raise
            except VerificationObservationLost:
                pass
            blackboard['verify_inspection_after_ns'] = self._now_ns()
            self.executive._report(
                'Verify: fire guidance interrupted; retaining the current viewpoint for recovery')
            if self._inspection_wait_fire(blackboard) is None:
                return None
        return None

    def _center_fire_once(self, blackboard, fire):
        """One best-effort yaw correction before planning fire-informed altitude."""
        error = fire.yaw_correction_rad
        if abs(error) <= self.config['verify_yaw_tolerance']:
            return fire
        cap = self.config['verify_inspection_yaw_step']
        yaw = yaw_from_quaternion(self._pose().orientation) + max(-cap, min(cap, error))
        self.executive._report('Verify/CENTER_FIRE: one bounded fire-heading correction')
        try:
            self.executive._verify_rotate(yaw, guard=self._check)
        except (VerificationStartUnavailable, VerificationRecoveryUnavailable) as error:
            self.executive._report(f'Verify/CENTER_FIRE: centering incomplete: {error}')
        blackboard['verify_inspection_after_ns'] = self._now_ns()
        return self._inspection_wait_fire(blackboard)

    def _fire_vertical_preference(self, blackboard, fire):
        """Consume one fresh image for a bounded, track-consistent height preference."""
        tolerance = self.config['verify_vertical_tolerance']
        hysteresis = self.config['verify_vertical_hysteresis']
        edge = self.config['verify_vertical_edge_threshold']
        step = self.config['verify_vertical_step']
        attempts = self.config['verify_vertical_max_corrections']
        if (not all(math.isfinite(v) for v in (tolerance, hysteresis, edge, step))
                or not 0.0 < tolerance < tolerance + hysteresis < edge < math.pi / 2
                or step <= 0.0 or not isinstance(attempts, int) or isinstance(attempts, bool)
                or attempts < 0):
            raise self.error_type('Invalid fire vertical guidance settings')
        if fire is None or self._stamp_issue(fire):
            return 0.0
        identity = (fire.header.frame_id, getattr(fire, 'track_id', 0))
        if _bb_get(blackboard, 'verify_vertical_track', None) != identity:
            blackboard['verify_vertical_track'] = identity
            blackboard['verify_vertical_stamp'] = 0
            blackboard['verify_vertical_direction'] = 0
            blackboard['verify_vertical_attempts'] = 0
        stamp = self._stamp_ns(fire)
        if stamp <= _bb_get(blackboard, 'verify_vertical_stamp', 0):
            return 0.0
        blackboard['verify_vertical_stamp'] = stamp
        error = fire.vertical_correction_rad
        previous = _bb_get(blackboard, 'verify_vertical_direction', 0)
        direction = 0
        if math.isfinite(error) and abs(error) < math.pi / 2:
            sign = 1 if error > 0.0 else -1
            threshold = tolerance if previous == sign else tolerance + hysteresis
            if abs(error) > threshold:
                direction = sign
        blackboard['verify_vertical_direction'] = direction
        self.executive._report(
            f'Verify: fire vertical angle={math.degrees(error):+.1f} deg; '
            f'requested height change={direction * step:+.2f} m')
        return direction * step

    def _approach_visible_fire(self, blackboard, fire):
        """Finish one checked move through flicker, then require post-stop fire guidance."""
        limit = self.config['verify_fire_approach_max_steps']
        while True:
            self._inspection_guard(blackboard)
            if not self._prepare_wall_observation(blackboard):
                return None
            current_fire = self._inspection_fire(blackboard)
            if current_fire is None:
                return None
            self._capture_height_hint(blackboard)
            vertical = self._fire_vertical_preference(blackboard, current_fire)
            step = _bb_get(blackboard, 'verify_fire_approach_steps', 0)
            height_blocked = False
            # Height guidance does not depend on a successful ranging query.
            # Protect a visible edge target before spending time on wall geometry.
            if (vertical and abs(current_fire.vertical_correction_rad)
                    >= self.config['verify_vertical_edge_threshold'] and step < limit
                    and (not self._at_known_wall(blackboard)
                         or _bb_get(blackboard, 'verify_vertical_attempts', 0)
                         < self.config['verify_vertical_max_corrections'])):
                result = self._move_visible_fire_height(blackboard, vertical)
                if result is None:
                    return None
                if result is not False:
                    continue
                height_blocked = True
            if not self._prepare_wall_observation(blackboard):
                return None
            current_fire = self._inspection_fire(blackboard)
            if current_fire is None:
                return None
            wall = self.executive._verify_center_fire_wall(current_fire.header.frame_id)
            self._record_wall_observation(blackboard, wall)
            action = self._wall_action(blackboard)
            if wall is None:
                if action == 'approach':
                    self._wait_for_wall(blackboard)
                return None
            if action != 'ready':
                return None
            # A geometry query or observation hold may consume the remaining
            # budget. Do not begin another leg after that wait has expired it.
            self._inspection_guard(blackboard)
            current_fire = self._inspection_fire(blackboard)
            if current_fire is None:
                return None
            before = self._pose()
            bearing = yaw_from_quaternion(before.orientation) + current_fire.yaw_correction_rad
            standoff = self.config['verify_smoke_standoff']
            tolerance = self.config.get('position_tolerance', 0.3)
            remaining = wall['nearest'] - wall['uncertainty'] - standoff - tolerance
            self.executive._report(
                f'Verify/CENTER_FIRE: 7 x 7 rays across 4.5 x 4.5 m smoke wall '
                f'mean={wall["mean"]:.2f} m, '
                f'nearest={wall["nearest"]:.2f} m, uncertainty={wall["uncertainty"]:.2f} m')
            at_standoff = smoke_wall_standoff_reached(
                wall, standoff, tolerance,
                max_distance=self.config['verify_smoke_inspection_max_distance'])
            if height_blocked and not at_standoff:
                if self._height_floor_rejected(vertical):
                    self._defer_descent(blackboard)
                    return None
                self._search_blocked(blackboard, 'fire height correction blocked')
                return None
            if self._stamp_ns(current_fire) > _bb_get(blackboard, 'verify_vertical_stamp', 0):
                vertical = self._fire_vertical_preference(blackboard, current_fire)
            vertical_attempts = _bb_get(blackboard, 'verify_vertical_attempts', 0)
            step = _bb_get(blackboard, 'verify_fire_approach_steps', 0)
            if at_standoff and (height_blocked or vertical == 0.0 or step >= limit
                                or vertical_attempts >= self.config[
                                    'verify_vertical_max_corrections']):
                self.executive._report(
                    'Verify/CENTER_FIRE: smoke-wall standoff reached with fresh fire visible')
                return current_fire
            # No fire coordinates or estimated fire range authorize this motion.
            geometry = {'source': 'fire', 'target_valid': False,
                        'navigation_bearing': bearing, 'center_fire_wall': wall,
                        'center_fire_max_path_length': self.config['verify_fire_leg_max_length'],
                        'center_fire_motion_timeout': self.config['verify_fire_leg_timeout']}
            geometry = self._wall_geometry(blackboard, geometry)
            if step >= limit:
                raise self.error_type(
                    'Verify: fire approach step limit reached before smoke-wall standoff; '
                    'inspection incomplete; holding for operator')
            self.executive._report(
                f'Verify/CENTER_FIRE: planning short smoke-wall approach leg {step + 1}/{limit}')
            vertical_only = vertical != 0.0 and (at_standoff or (
                remaining >= 0.0 and abs(current_fire.vertical_correction_rad)
                >= self.config['verify_vertical_edge_threshold']))
            # Detector loss and track-ID changes do not cancel an already
            # committed leg. Vehicle checks and live wall/terrain validation do.
            if vertical_only:
                target = deepcopy(before)
                target.position.z += vertical
                if at_standoff:
                    blackboard['verify_vertical_attempts'] = vertical_attempts + 1
                self.executive._report(
                    f'Verify/CENTER_FIRE: vertical fire correction {vertical:+.2f} m '
                    'before further forward motion')
                blackboard['verify_height_hint_consumed'] = _bb_get(
                    blackboard, 'verify_vertical_stamp', 0)
                moved = self.executive._verify_recovery_plan_to(
                    target, geometry=geometry, guard=lambda: self._movement_guard(blackboard),
                    preserve_requested_altitude=True)
            elif remaining < 0.0:
                target = deepcopy(before)
                retreat = min(self.config['verify_navigation_step'], -remaining)
                target.position.x -= retreat * math.cos(bearing)
                target.position.y -= retreat * math.sin(bearing)
                target.position.z += vertical
                if vertical:
                    blackboard['verify_height_hint_consumed'] = _bb_get(
                        blackboard, 'verify_vertical_stamp', 0)
                moved = self.executive._verify_recovery_plan_to(
                    target, geometry=geometry, guard=lambda: self._movement_guard(blackboard),
                    preserve_requested_altitude=vertical != 0.0)
            else:
                if vertical:
                    blackboard['verify_height_hint_consumed'] = _bb_get(
                        blackboard, 'verify_vertical_stamp', 0)
                self._begin_forward_leg(blackboard)
                moved = self.executive._verify_navigation_viewpoint(
                    bearing, geometry=geometry, guard=lambda: self._movement_guard(blackboard),
                    index=step,
                    maximum_forward=min(self.config['verify_navigation_step'], remaining),
                    vertical_offset=vertical)
            blackboard['verify_fire_approach_steps'] = step + 1
            after = self._pose().position
            displacement = math.dist(
                tuple(getattr(before.position, a) for a in 'xyz'),
                tuple(getattr(after, a) for a in 'xyz'))
            if moved is False or displacement <= tolerance:
                if vertical_only and at_standoff:
                    # Centering is best effort once measured proximity is met.
                    # A rejected path must not loop or bypass fresh post-stop evidence.
                    blackboard['verify_vertical_attempts'] = self.config[
                        'verify_vertical_max_corrections']
                    blackboard['verify_inspection_after_ns'] = self._now_ns()
                    self.executive._report(
                        'Verify/CENTER_FIRE: vertical correction blocked at standoff; '
                        'rechecking visible fire and wall before completion')
                    if self._inspection_wait_fire(blackboard) is None:
                        return None
                    continue
                failures = _bb_get(blackboard, 'verify_search_failures', 0) + 1
                blackboard['verify_search_failures'] = failures
                self.executive._report(
                    f'Verify: fire approach blocked before smoke-wall standoff; '
                    f'recovery {failures}/{self.config["verify_search_recovery_max"]}')
                if failures >= self.config['verify_search_recovery_max']:
                    raise self.error_type(
                        'Verify: fire approach blocked; inspection incomplete; '
                        'holding for operator')
                self.executive._verify_hold(
                    self.config['verify_navigation_map_wait'], guard=self._check)
                return None
            blackboard['verify_search_failures'] = 0
            blackboard['verify_inspection_after_ns'] = self._now_ns()
            self.executive._report(
                f'Verify/CENTER_FIRE: movement completed ({displacement:.2f} m); '
                'holding for post-stop fire confirmation')
            current_fire = self._inspection_wait_fire(blackboard)
            if current_fire is None:
                self.executive._report(
                    'Verify/CENTER_FIRE: fire did not return after stopping; reacquiring')
                return None
            # Localization remains optional report metadata, never the distance gate.
            self._inspection_geometry(blackboard, current_fire, source='fire')

    def _move_visible_fire_height(self, blackboard, vertical):
        """Validate a small height-only leg independently of smoke-wall availability."""
        if _bb_get(blackboard, 'verify_fire_approach_steps', 0) >= self.config[
                'verify_fire_approach_max_steps']:
            raise self.error_type(
                'Verify: fire approach step limit reached before smoke-wall standoff; '
                'inspection incomplete; holding for operator')
        before = self._pose()
        target = deepcopy(before)
        target.position.z += vertical
        if _bb_get(blackboard, 'verify_smoke_boundary_reached', False):
            blackboard['verify_vertical_attempts'] = _bb_get(
                blackboard, 'verify_vertical_attempts', 0) + 1
        self.executive._report(
            f'Verify/CENTER_FIRE: vertical fire correction {vertical:+.2f} m '
            'before wall ranging or further forward motion')
        blackboard['verify_height_hint_consumed'] = _bb_get(
            blackboard, 'verify_vertical_stamp', 0)
        self.executive._verify_height_rejection = ''
        if self.executive._verify_recovery_plan_to(
                target, geometry=self._wall_geometry(blackboard),
                guard=lambda: self._movement_guard(blackboard),
                preserve_requested_altitude=True) is False:
            self.executive._report('Verify: fire height correction blocked by path or terrain')
            if self._height_floor_rejected(vertical):
                self._defer_descent(blackboard)
            return False
        actual = self._pose().position.z - before.position.z
        minimum = self.config['verify_vertical_minimum_change'] - self.config[
            'verify_vertical_position_tolerance']
        self.executive._report(
            f'Verify: height-only fire correction measured height change={actual:+.2f} m')
        if actual * math.copysign(1.0, vertical) < minimum - 1e-6:
            self.executive._report('Verify: height-only correction made no useful height progress')
            return False
        blackboard['verify_fire_approach_steps'] = _bb_get(
            blackboard, 'verify_fire_approach_steps', 0) + 1
        blackboard['verify_inspection_after_ns'] = self._now_ns()
        return self._inspection_wait_fire(blackboard)

    def center_fire(self, blackboard):
        """Inspect only after fresh near-wall confirmation; revalidate moving smoke."""
        self._check()
        try:
            self._inspection_guard(blackboard)
            if not self._prepare_wall_observation(blackboard):
                return 'reacquire'
            observation, source = self._approach_observation(blackboard)
            if observation is None:
                return self._wait_for_wall(blackboard)
            geometry = self._inspection_geometry(blackboard, observation, source=source)
            bearing, geometry = self._approach_bearing(
                blackboard, observation, source, geometry)
            wall = self._probe_smoke_wall(blackboard, observation, bearing)
            self._check_inspection_deadline(blackboard)
            action = self._wall_action(blackboard)
            fire = self._inspection_fire(blackboard)
            if action != 'ready':
                # A missing wall can withhold forward motion, never an otherwise
                # safe fire-height correction while the range is revalidated.
                if (action == 'reacquire' and fire is not None
                        and self.executive._verify_wall_observation_ready()):
                    vertical = self._fire_vertical_preference(blackboard, fire)
                    attempts = _bb_get(blackboard, 'verify_vertical_attempts', 0)
                    if vertical and (not _bb_get(
                            blackboard, 'verify_smoke_boundary_reached', False)
                            or attempts < self.config['verify_vertical_max_corrections']):
                        self._move_visible_fire_height(blackboard, vertical)
                return action
            if fire is not None:
                self._inspection_begin(blackboard)
                self._route_fire(blackboard)
                centered = self._center_fire_once(blackboard, fire)
                if centered is None:
                    return 'reacquire'
                centered = self._approach_visible_fire(blackboard, centered)
                if centered is None:
                    if not _bb_get(blackboard, 'verify_smoke_boundary_reached', False):
                        return 'approach'
                    return 'reacquire'
                geometry = self._inspection_geometry(blackboard, centered, source='fire')
                report = self.executive._verify_report_inspection(
                    geometry, fire_message=centered,
                    attempts=max(1, _bb_get(blackboard, 'verify_inspection_attempts', 0)))
                self.executive._verify_complete_site(report, message=centered, source='fire')
                return 'explore'
            if _bb_get(blackboard, 'verify_fire_observed', False) or self.had_fire_confirmation():
                return self._search_blocked(
                    blackboard, 'confirmed fire guidance has not recovered at the near wall',
                    fire_loss=True)
            self._inspection_begin(blackboard)
            origin = _bb_get(blackboard, 'verify_inspection_origin', None) or self._pose()
            blackboard['verify_inspection_origin'] = origin

            def guard():
                self._inspection_guard(blackboard, interrupt_fire=True)

            while _bb_get(blackboard, 'verify_inspection_attempts', 0) < self.config[
                    'verify_inspection_max_attempts']:
                guard()
                attempts = _bb_get(blackboard, 'verify_inspection_attempts', 0)
                if attempts:
                    target = deepcopy(origin)
                    offset = self.config['verify_viewpoint_lateral_step'] * (
                        1.0 if attempts % 2 else -1.0)
                    target.position.x -= offset * math.sin(bearing)
                    target.position.y += offset * math.cos(bearing)
                    if self.executive._verify_recovery_plan_to(
                            target, geometry=self._wall_geometry(blackboard, geometry),
                            guard=guard) is False:
                        return self._search_blocked(blackboard, 'smoke boundary viewpoint blocked')
                    blackboard['verify_inspection_after_ns'] = self._now_ns()
                    observation, source = self._inspection_observation(blackboard)
                    if source == 'fire':
                        return self._route_fire(blackboard)
                    if observation is None:
                        return 'reacquire'
                smoke = self.fresh_smoke()
                if smoke is None:
                    return self._wait_for_wall(blackboard)
                wall = self._probe_smoke_wall(blackboard, smoke, bearing)
                action = self._wall_action(blackboard)
                if action != 'ready':
                    return action
                self.executive._report(f'Verify: fire inspection {attempts + 1}/'
                                       f'{self.config["verify_inspection_max_attempts"]}')
                self.executive._verify_hold(self.config['verify_search_hold'], guard=guard)
                if self.fresh_smoke() is None:
                    return 'reacquire'
                blackboard['verify_inspection_attempts'] = attempts + 1
            smoke = self.fresh_smoke()
            if smoke is None:
                return 'reacquire'
            geometry = self._inspection_geometry(blackboard, smoke, source='smoke')
            report = self.executive._verify_report_inspection(
                geometry, attempts=blackboard['verify_inspection_attempts'], wall=wall)
            self.executive._verify_complete_site(report, message=smoke, source='smoke')
            return 'explore'
        except _FireFound:
            return self._route_fire(blackboard, after_motion=True)
        except (VerificationStartUnavailable, VerificationRecoveryUnavailable) as error:
            return self._navigation_start_unavailable(blackboard, error, resume='search')
        except _InspectionTimeout as error:
            raise self.error_type(
                'Verify: inspection time budget exhausted before confirmed completion; '
                'inspection incomplete; holding for operator') from error
        except VerificationObservationLost:
            return 'reacquire'

    def _search_blocked(self, blackboard, reason, *, fire_loss=False):
        """Retain a blocked target and finish in LOITER rather than normal exploration."""
        if fire_loss:
            # A new frame may have arrived since the caller observed the loss.
            # Resolve that race before charging or exhausting a fire-only retry.
            fire = self.stable_fire()
            if self._recover_fire_loss(blackboard, fire):
                return (self._route_fire(blackboard) if self._at_known_wall(blackboard)
                        else 'reacquire')
            with self._lock:
                last_stamp = self._last_stamps['fire']
            blackboard['verify_fire_loss_after_ns'] = max(
                _bb_get(blackboard, 'verify_fire_loss_after_ns', 0),
                self._now_ns(), last_stamp)
        key = 'verify_fire_loss_failures' if fire_loss else 'verify_search_failures'
        failures = _bb_get(blackboard, key, 0) + 1
        blackboard[key] = failures
        category = 'fire-loss recovery' if fire_loss else 'blocked recovery'
        self.executive._report(f'Verify: {reason}; {category} '
                               f'{failures}/{self.config["verify_search_recovery_max"]}')
        if failures >= self.config['verify_search_recovery_max']:
            raise self.error_type(f'Verify blocked: {reason}; holding for operator')
        if self._at_known_wall(blackboard) and self._inspection_fire(blackboard) is not None:
            return self._route_fire(blackboard)
        self.executive._verify_hold(self.config['verify_navigation_map_wait'], guard=self._check)
        blackboard['verify_reference_pose'] = self._pose()
        return 'reacquire'

    def _navigation_start_unavailable(self, blackboard, error, resume='reacquire'):
        """Hold for a newer valid map within one bounded recovery episode."""
        deadline = time.monotonic() + self.config['verify_navigation_map_timeout']
        map_unavailable = isinstance(error, VerificationMapUnavailable)
        if not map_unavailable:
            # Persistent clearance failures share a budget until the aircraft
            # moves. A map outage ends when fresh data returns; healthy planning
            # and OFFBOARD handoff must not consume the next outage's budget.
            position = self._pose().position
            current = (position.x, position.y, position.z)
            origin = _bb_get(blackboard, 'verify_navigation_refresh_origin', None)
            if origin is None or math.dist(current, origin) > max(
                    0.1, self.config.get('position_tolerance', 0.3)):
                blackboard['verify_navigation_refresh_origin'] = current
                blackboard['verify_navigation_refresh_deadline'] = deadline
            previous_deadline = _bb_get(blackboard, 'verify_navigation_refresh_deadline', None)
            if previous_deadline is not None:
                deadline = previous_deadline
            blackboard['verify_navigation_refresh_deadline'] = deadline
        after_ns = max(
            getattr(error, 'source_stamp_ns', 0),
            getattr(self.executive, '_verify_last_geometry_stamp_ns', 0),
            _bb_get(blackboard, 'verify_navigation_refresh_stamp_ns', 0),
        )
        # A future stamp or clock reset must not require catching up to an
        # invalid old epoch before accepting genuinely fresh observations.
        ros_now_ns = self._now_ns()
        if after_ns > ros_now_ns + int(self.config['verify_future_tolerance'] * 1e9):
            after_ns = 0
        kind = ('map unavailable' if map_unavailable
                else 'clearance recovery unavailable' if isinstance(
                    error, VerificationRecoveryUnavailable)
                else 'current clearance unavailable')
        self.executive._report(f'Verify: {kind}: {error}')
        self.executive._report(
            'Verify: holding in LOITER for a newer fresh OctoMap; '
            f'recovery budget {max(0.0, deadline - time.monotonic()):.1f} s')
        while time.monotonic() < deadline:
            self._check()
            # Fire cannot end a map outage: CENTER_FIRE would encounter the
            # same stale map and restart this deadline on every transition.
            if (not map_unavailable and self._at_known_wall(blackboard)
                    and self.stable_fire() is not None):
                return self._route_fire(blackboard)
            # Finish the mode handoff and poll from LOITER instead of
            # repeating camera centering before new map evidence arrives.
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            self.executive._verify_hold(
                min(0.25, remaining), guard=self._check)
            self._check()
            if (not map_unavailable and self._at_known_wall(blackboard)
                    and self.stable_fire() is not None):
                return self._route_fire(blackboard)
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            current_ros_ns = self._now_ns()
            if current_ros_ns < ros_now_ns:
                after_ns = 0
            ros_now_ns = current_ros_ns
            stamp_ns = self.executive._verify_fresh_map_stamp(timeout=min(0.5, remaining))
            if time.monotonic() < deadline and stamp_ns > after_ns:
                blackboard['verify_navigation_refresh_stamp_ns'] = stamp_ns
                blackboard['verify_reference_pose'] = self._pose()
                self.executive._report(
                    'Verify: newer fresh OctoMap observed; retrying clearance checks')
                if self._at_known_wall(blackboard) and self.stable_fire() is not None:
                    return self._route_fire(blackboard)
                return resume
        self._check()
        last_map_status = getattr(self.executive, '_verify_map_unavailable_reason', '')
        raise self.error_type(
            f'Verify blocked: fresh map recovery deadline exceeded; {error}; '
            + (f'last map status: {last_map_status}; ' if last_map_status else '') +
            'holding position for operator')

    def _navigation_progress(self, blackboard, before, bearing, moved):
        """Count actual forward progress, not lateral motion or repeated viewpoints."""
        after = self._pose().position
        start = before.position
        displacement = (after.x - start.x, after.y - start.y, after.z - start.z)
        forward = displacement[0] * math.cos(bearing) + displacement[1] * math.sin(bearing)
        tolerance = self.config.get('position_tolerance', 0.3)
        visited = _bb_get(blackboard, 'verify_navigation_visited', None)
        if visited is None:
            visited = [(start.x, start.y, start.z)]
        point = (after.x, after.y, after.z)
        repeat = any(math.dist(point, old) <= max(0.1, tolerance) for old in visited)
        useful = moved is not False and not repeat and forward > tolerance
        if useful:
            blackboard['verify_search_failures'] = 0
            blackboard['verify_height_only_attempted'] = False
            if _bb_get(blackboard, 'verify_descent_deferred', False):
                blackboard['verify_descent_deferred'] = False
                self.executive._report(
                    'Verify: fire-height descent guidance re-enabled after forward progress')
        stalled = 0 if useful else _bb_get(blackboard, 'verify_navigation_stalls', 0) + 1
        blackboard['verify_navigation_stalls'] = stalled
        if math.hypot(*displacement) > tolerance:
            blackboard['verify_navigation_start_refreshes'] = 0
            blackboard['verify_navigation_refresh_deadline'] = None
            blackboard['verify_navigation_refresh_stamp_ns'] = 0
        if not repeat:
            visited.append(point)
        blackboard['verify_navigation_visited'] = visited[-24:]
        self.executive._report(
            f'Verify: navigation progress forward={forward:.2f} m, '
            f'up={displacement[2]:.2f} m; stalled views '
            f'{stalled}/{self.config["verify_navigation_no_progress_max"]}')
        return stalled < self.config['verify_navigation_no_progress_max']

    def _navigation_stalled(self, blackboard):
        return self._search_blocked(
            blackboard, 'planned motion did not make new forward progress; '
            'this is a no-progress limit, not a detected terrain collision')

    def search_fire(self, blackboard):
        """Revalidate near-wall evidence before entering final inspection."""
        self._check_inspection_deadline(blackboard)
        try:
            if self._recover_fire_height(blackboard):
                return 'reacquire'
            if not self._prepare_wall_observation(blackboard):
                return 'reacquire'
            observation, source = self._approach_observation(blackboard)
            if observation is None:
                return self._wait_for_wall(blackboard)
            geometry = self._inspection_geometry(blackboard, observation, source=source)
            bearing, geometry = self._approach_bearing(
                blackboard, observation, source, geometry)
            self._probe_smoke_wall(blackboard, observation, bearing)
            self._check_inspection_deadline(blackboard)
            action = self._wall_action(blackboard)
            if action == 'ready':
                blackboard['verify_search_active'] = True
                return (self._route_fire(blackboard) if self.stable_fire() is not None
                        else 'inspect')
            return action
        except (VerificationStartUnavailable, VerificationRecoveryUnavailable) as error:
            return self._navigation_start_unavailable(blackboard, error, resume='search')
        except _InspectionTimeout as error:
            raise self.error_type(
                'Verify: inspection time budget exhausted; holding for operator') from error
        except VerificationObservationLost:
            return 'reacquire'


_FINAL_OUTCOMES = ['explore', 'aborted', 'canceled', 'return_requested']


class _VerifyState(State):
    def __init__(self, executive, name, operation, routes):
        super().__init__(outcomes=list(dict.fromkeys(routes + _FINAL_OUTCOMES)))
        self.executive = executive
        self.name = name
        self.operation = operation

    def execute(self, blackboard):
        # Imported only at execution time: the executive imports this module.
        from kv_executive.mission_executive import (
            MissionCanceled, MissionError, MissionStateRequested,
        )
        self.executive._report(f'Verify/{self.name}: started')
        try:
            if self.is_canceled():
                raise MissionCanceled('Mission canceled')
            try:
                return self.operation(blackboard)
            except (VerificationStartUnavailable, VerificationRecoveryUnavailable) as error:
                return self.executive._verify._navigation_start_unavailable(
                    blackboard, error, resume='approach' if self.name == 'APPROACH_SMOKE'
                    else 'reacquire')
        except MissionCanceled:
            return 'canceled'
        except MissionStateRequested as request:
            if request.state == 'Return':
                return 'return_requested'
            self.executive._last_error = str(request)
            return 'aborted'
        except (MissionError, ValueError) as error:
            self.executive._last_error = f'Verify/{self.name}: {error}'
            self.executive._report(self.executive._last_error, error=True)
            return 'aborted'


class VerifyStateMachine(StateMachine):
    """Four sub-states with explicit exits to the higher mission machine."""

    def __init__(self, executive):
        """Connect the four controller operations and explicit parent outcomes."""
        super().__init__(outcomes=_FINAL_OUTCOMES)
        controller = executive._verify
        states = (
            ('APPROACH_SMOKE', controller.approach_smoke,
             {'approach': 'APPROACH_SMOKE', 'reacquire': 'REACQUIRE', 'fire': 'CENTER_FIRE',
              'search': 'SEARCH_FIRE'}),
            ('REACQUIRE', controller.reacquire,
             {'approach': 'APPROACH_SMOKE', 'fire': 'CENTER_FIRE', 'search': 'SEARCH_FIRE',
              'reacquire': 'REACQUIRE'}),
            ('CENTER_FIRE', controller.center_fire,
             {'reacquire': 'REACQUIRE', 'search': 'SEARCH_FIRE', 'fire': 'CENTER_FIRE',
              'approach': 'APPROACH_SMOKE'}),
            ('SEARCH_FIRE', controller.search_fire,
             {'reacquire': 'REACQUIRE', 'fire': 'CENTER_FIRE', 'inspect': 'CENTER_FIRE',
              'search': 'SEARCH_FIRE', 'approach': 'APPROACH_SMOKE'}),
        )
        for name, operation, routes in states:
            transitions = {outcome: outcome for outcome in _FINAL_OUTCOMES}
            transitions.update(routes)
            self.add_state(name, _VerifyState(executive, name, operation, list(routes)),
                           transitions=transitions)
        self.set_start_state('APPROACH_SMOKE')
