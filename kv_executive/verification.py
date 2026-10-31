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
    'verify_vertical_tolerance': math.radians(5.0),
    'verify_vertical_hysteresis': math.radians(2.0),
    'verify_vertical_edge_threshold': math.radians(15.0),
    'verify_vertical_max_corrections': 3,
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
    'verify_fire_standoff': 10.0,
    'verify_navigation_no_progress_max': 2,
    'verify_smoke_standoff': 20.0,
    'verify_inspection_altitude': 5.0,
    'verify_inspection_lateral_step': 5.0,
    'verify_inspection_max_attempts': 3,
    'verify_inspection_yaw_step': math.radians(5.0),
    'verify_inspection_timeout': 300.0,
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
                    f'detector_frames={getattr(message, "confirmation_frames", 0)}, '
                    f'detector_persistence={getattr(message, "persistence_sec", 0.0):.2f}s, '
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
            raw = yaw_from_quaternion(self._pose().orientation) + smoke.yaw_correction_rad
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

    def approach_smoke(self, blackboard):
        """Follow current smoke guidance in short terrain-checked legs."""
        if not _bb_get(blackboard, 'verify_entered', False):
            self.executive._verify_parent_entry('Verify')
            # Exploration can turn toward another site in the same camera frame.
            # Keep memory through occlusion within this episode, never across sites.
            with self._lock:
                self._smoke_bearing = None
                self._smoke_bearing_frame = None
            blackboard['verify_entered'] = True
        self._check()
        if self.stable_fire() is not None:
            return self._route_fire(blackboard)
        self._check_approach_limit(blackboard)
        blackboard['verify_reference_pose'] = self._pose()
        try:
            smoke = self._smoke_guidance(after_ns=max(
                _bb_get(blackboard, 'verify_arrival_after_ns', 0),
                _bb_get(blackboard, 'verify_inspection_after_ns', 0)))
            if self.stable_fire() is not None:
                return self._route_fire(blackboard)
            if smoke is None:
                return 'reacquire'
            geometry = self._inspection_geometry(blackboard, smoke, source='smoke')
            if self._stamp_issue(smoke):
                self.executive._report(
                    'Verify: smoke image aged during geometry query; holding for fresh guidance')
                return 'reacquire'
            if self.executive._verify_smoke_boundary_reached(geometry):
                blackboard['verify_smoke_boundary_reached'] = True
                self._inspection_begin(blackboard)
                return 'reacquire'
            before = self._pose()
            bearing, geometry = self._remember_smoke_bearing(blackboard, smoke, geometry)
            self.executive._report(
                'Verify: smoke visible; following a short terrain-aware smoke leg')
            # Finish the selected leg through fire detections and smoke coverage
            # changes. Live heading follows fire when available; the planner and
            # corridor validators still enforce obstacle and terrain clearance.
            moved = self.executive._verify_navigation_viewpoint(
                bearing, geometry=geometry, guard=self._check, forward_only=True)
            if moved is False:
                reason = getattr(self.executive, '_verify_navigation_failure_reason',
                                 'no validated forward smoke path')
                return self._search_blocked(blackboard, reason)
            arrived = self._pose().position
            displacement = math.dist((arrived.x, arrived.y, arrived.z),
                                     (before.position.x, before.position.y, before.position.z))
            if displacement <= self.config.get('position_tolerance', 0.3):
                return self._search_blocked(blackboard, 'smoke leg did not change viewpoint')
            # Sensor waits and map retries are not completed approach legs.
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
            return self._route_fire(blackboard, after_motion=True)
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
        return fire if fire is not None and self._stamp_ns(fire) > after_ns else None

    def reacquire(self, blackboard):
        """Wait for post-stop guidance without discarding a confirmed fire target."""
        self._check()
        after_ns = max(_bb_get(blackboard, 'verify_arrival_after_ns', 0),
                       _bb_get(blackboard, 'verify_inspection_after_ns', 0))
        blackboard['verify_arrival_after_ns'] = 0
        self.executive._report('Verify: holding arrival viewpoint for detector confirmation')
        smoke = self._smoke_guidance(after_ns=after_ns)
        if self._reacquire_fire(blackboard, after_ns) is not None:
            return self._route_fire(blackboard)
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
                return self._route_fire(blackboard)
            smoke = self.fresh_smoke()
            if smoke is not None and self._stamp_ns(smoke) > after_ns:
                return self._reacquire_result(blackboard, smoke)
            try:
                self.executive._verify_rotate(yaw + offset, guard=scan_guard, hold_duration=0.5)
            except _FireFound:
                return self._route_fire(blackboard, after_motion=True)
            except VerificationObservationLost:
                pass
            after_ns = self._now_ns()
            smoke = self._smoke_guidance(after_ns=after_ns)
            if self._reacquire_fire(blackboard, after_ns) is not None:
                return self._route_fire(blackboard)
            if smoke is not None:
                return self._reacquire_result(blackboard, smoke)
        if _bb_get(blackboard, 'verify_fire_observed', False) or self.had_fire_confirmation():
            return self._search_blocked(blackboard, 'confirmed fire guidance has not recovered')
        blackboard['verify_exploration_required'] = True
        self.executive._report('Verify: no fresh smoke or fire after bounded reacquisition')
        return 'explore'

    def _reacquire_result(self, blackboard, smoke):
        if self._reacquire_fire(blackboard) is not None:
            return self._route_fire(blackboard)
        if smoke is not None:
            if (_bb_get(blackboard, 'verify_search_active', False)
                    or _bb_get(blackboard, 'verify_smoke_boundary_reached', False)):
                return 'search'
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
        started = _bb_get(blackboard, 'verify_inspection_started', None)
        if started is not None and time.monotonic() - started >= self.config[
                'verify_inspection_timeout']:
            raise _InspectionTimeout('Inspection deadline exceeded')
        if interrupt_fire and (self._inspection_fire(blackboard) is not None
                               or self._inspection_fire_event(blackboard)):
            raise _FireFound('Persistent fire found during inspection')

    def _inspection_begin(self, blackboard):
        if _bb_get(blackboard, 'verify_inspection_started', None) is None:
            blackboard['verify_inspection_started'] = time.monotonic()
            blackboard['verify_inspection_attempts'] = 0

    def _inspection_fire(self, blackboard):
        fire = self.stable_fire()
        after_ns = max(_bb_get(blackboard, 'verify_inspection_after_ns', 0),
                       _bb_get(blackboard, 'verify_smoke_approach_after_ns', 0))
        if fire is not None and self._stamp_ns(fire) > after_ns:
            return fire
        return None

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
                               _bb_get(blackboard, 'verify_smoke_approach_after_ns', 0))
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
        after_ns = max(after_ns, _bb_get(blackboard, 'verify_inspection_after_ns', 0),
                       _bb_get(blackboard, 'verify_smoke_approach_after_ns', 0))
        deadline = time.monotonic() + self.config['verify_reacquire_observation_timeout']
        while time.monotonic() < deadline:
            self._inspection_guard(blackboard)
            fire = self.stable_fire()
            smoke = self.fresh_smoke()
            if fire is not None and self._stamp_ns(fire) > after_ns:
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
        return direction * step

    def _approach_visible_fire(self, blackboard, fire):
        """Finish one checked move through flicker, then require post-stop fire guidance."""
        limit = self.config['verify_fire_approach_max_steps']
        while True:
            self._inspection_guard(blackboard)
            current_fire = self._inspection_fire(blackboard)
            if current_fire is None:
                return None
            wall = self.executive._verify_center_fire_wall(current_fire.header.frame_id)
            if wall is None:
                self.executive._report(
                    'Verify/CENTER_FIRE: smoke-wall distance unavailable; '
                    'holding for fresh geometry')
                self.executive._verify_hold(
                    self.config['verify_reacquire_observation_timeout'], guard=self._check)
                wall = self.executive._verify_center_fire_wall(current_fire.header.frame_id)
                current_fire = self._inspection_fire(blackboard)
                if wall is None or current_fire is None:
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
                f'Verify/CENTER_FIRE: 3 x 3 m smoke wall mean={wall["mean"]:.2f} m, '
                f'nearest={wall["nearest"]:.2f} m, uncertainty={wall["uncertainty"]:.2f} m')
            at_standoff = smoke_wall_standoff_reached(wall, standoff, tolerance)
            vertical = self._fire_vertical_preference(blackboard, current_fire)
            vertical_attempts = _bb_get(blackboard, 'verify_vertical_attempts', 0)
            step = _bb_get(blackboard, 'verify_fire_approach_steps', 0)
            if at_standoff and (vertical == 0.0 or step >= limit
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
                moved = self.executive._verify_recovery_plan_to(
                    target, geometry=geometry, guard=self._check,
                    preserve_requested_altitude=True)
            elif remaining < 0.0:
                target = deepcopy(before)
                retreat = min(self.config['verify_navigation_step'], -remaining)
                target.position.x -= retreat * math.cos(bearing)
                target.position.y -= retreat * math.sin(bearing)
                target.position.z += vertical
                moved = self.executive._verify_recovery_plan_to(
                    target, geometry=geometry, guard=self._check,
                    preserve_requested_altitude=vertical != 0.0)
            else:
                moved = self.executive._verify_navigation_viewpoint(
                    bearing, geometry=geometry, guard=self._check, index=step,
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

    def center_fire(self, blackboard):
        """Center and approach visible fire, or inspect the smoke boundary."""
        self._check()
        try:
            self._inspection_guard(blackboard)
            fire = self._inspection_fire(blackboard)
            if fire is None and _bb_get(blackboard, 'verify_fire_pending', False):
                fire = self._inspection_wait_fire(blackboard)
            if fire is not None:
                self._inspection_begin(blackboard)
                self._route_fire(blackboard)
                centered = self._center_fire_once(blackboard, fire)
                if centered is None:
                    return 'reacquire'
                centered = self._approach_visible_fire(blackboard, centered)
                if centered is None:
                    return 'reacquire'
                # Localization is optional report metadata, never a movement or completion gate.
                geometry = self._inspection_geometry(blackboard, centered, source='fire')
                report = self.executive._verify_report_inspection(
                    geometry, fire_message=centered,
                    attempts=max(1, _bb_get(blackboard, 'verify_inspection_attempts', 0)))
                self.executive._verify_complete_site(report, message=centered, source='fire')
                return 'explore'
            if _bb_get(blackboard, 'verify_fire_observed', False) or self.had_fire_confirmation():
                self.executive._report(
                    'Verify/CENTER_FIRE: fire guidance missing after observation wait; '
                    'reacquiring')
                return 'reacquire'
            smoke = self.fresh_smoke()
            geometry = self._inspection_geometry(blackboard, smoke, source='smoke')
            if not (smoke is not None
                    and self.executive._verify_smoke_boundary_reached(geometry)):
                return 'search'
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
                    bearing = _bb_get(blackboard, 'verify_smoke_bearing',
                                      yaw_from_quaternion(origin.orientation))
                    if bearing is None:
                        bearing = yaw_from_quaternion(origin.orientation)
                    offset = self.config['verify_viewpoint_lateral_step'] * (
                        1.0 if attempts % 2 else -1.0)
                    target.position.x -= offset * math.sin(bearing)
                    target.position.y += offset * math.cos(bearing)
                    if self.executive._verify_recovery_plan_to(target, guard=guard) is False:
                        return self._search_blocked(blackboard, 'smoke boundary viewpoint blocked')
                    blackboard['verify_inspection_after_ns'] = self._now_ns()
                    observation, source = self._inspection_observation(blackboard)
                    if source == 'fire':
                        return self._route_fire(blackboard)
                    if observation is None:
                        return 'reacquire'
                self.executive._report(f'Verify: fire inspection {attempts + 1}/'
                                       f'{self.config["verify_inspection_max_attempts"]}')
                self.executive._verify_hold(self.config['verify_search_hold'], guard=guard)
                smoke = self.fresh_smoke()
                if smoke is None:
                    return 'reacquire'
                blackboard['verify_inspection_attempts'] = attempts + 1
            smoke = self.fresh_smoke()
            geometry = self._inspection_geometry(blackboard, smoke, source='smoke')
            if (smoke is not None and self.all_quadrants(smoke)
                    and self.executive._verify_smoke_boundary_reached(geometry)):
                report = self.executive._verify_report_inspection(
                    geometry, attempts=blackboard['verify_inspection_attempts'])
                self.executive._verify_complete_site(report, message=smoke, source='smoke')
            else:
                self.executive._report('Verify: bounded smoke inspection inconclusive')
                blackboard['verify_exploration_required'] = True
            return 'explore'
        except _FireFound:
            return self._route_fire(blackboard, after_motion=True)
        except (VerificationStartUnavailable, VerificationRecoveryUnavailable) as error:
            return self._navigation_start_unavailable(blackboard, error, resume='search')
        except _InspectionTimeout:
            raise self.error_type(
                'Verify: inspection time budget exhausted before confirmed completion; '
                'inspection incomplete; holding for operator')
        except VerificationObservationLost:
            return 'reacquire'

    def _search_blocked(self, blackboard, reason):
        """Retain a blocked target and finish in LOITER rather than normal exploration."""
        failures = _bb_get(blackboard, 'verify_search_failures', 0) + 1
        blackboard['verify_search_failures'] = failures
        self.executive._report(f'Verify: {reason}; blocked recovery '
                               f'{failures}/{self.config["verify_search_recovery_max"]}')
        if self._inspection_fire(blackboard) is not None:
            return self._route_fire(blackboard)
        if failures >= self.config['verify_search_recovery_max']:
            raise self.error_type(f'Verify blocked: {reason}; holding for operator')
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
            if not map_unavailable and self.stable_fire() is not None:
                return self._route_fire(blackboard)
            # Finish the mode handoff and poll from LOITER instead of
            # repeating camera centering before new map evidence arrives.
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            self.executive._verify_hold(
                min(0.25, remaining), guard=self._check)
            self._check()
            if not map_unavailable and self.stable_fire() is not None:
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
                if self.stable_fire() is not None:
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
        """Search along smoke-boundary guidance; fire range never selects a goal."""
        self._check()
        blackboard['verify_search_active'] = True

        def guard():
            self._inspection_guard(blackboard, interrupt_fire=True)

        try:
            if self._inspection_fire(blackboard) is not None:
                return self._route_fire(blackboard)
            self._inspection_guard(blackboard)
            smoke = self._smoke_guidance(blackboard)
            if smoke is None:
                return 'reacquire'
            geometry = self._inspection_geometry(blackboard, smoke, source='smoke')
            if self.executive._verify_smoke_boundary_reached(geometry):
                self._inspection_begin(blackboard)
                blackboard['verify_smoke_boundary_reached'] = True
                return 'inspect'
            self.executive._report(
                'Verify: holding smoke viewpoint for fresh thermal confirmation')
            self.executive._verify_hold(self.config['verify_search_hold'], guard=guard)
            observation, source = self._inspection_observation(blackboard)
            if source == 'fire':
                return self._route_fire(blackboard)
            if observation is None:
                return 'reacquire'
            geometry = self._inspection_geometry(blackboard, observation, source='smoke')
            if self._stamp_issue(observation):
                self.executive._report(
                    'Verify: smoke image aged during geometry query; holding for fresh guidance')
                return 'reacquire'
            if self.executive._verify_smoke_boundary_reached(geometry):
                self._inspection_begin(blackboard)
                blackboard['verify_smoke_boundary_reached'] = True
                return 'inspect'
            before = self._pose()
            bearing, geometry = self._remember_smoke_bearing(blackboard, observation, geometry)
            moved = self.executive._verify_navigation_viewpoint(
                bearing, geometry=geometry, guard=guard,
                index=_bb_get(blackboard, 'verify_navigation_legs', 0), forward_only=True)
            if moved is False:
                reason = getattr(self.executive, '_verify_navigation_failure_reason',
                                 'no validated forward smoke path')
                return self._search_blocked(blackboard, reason)
            if not self._navigation_progress(blackboard, before, bearing, moved):
                return self._navigation_stalled(blackboard)
            blackboard['verify_navigation_legs'] = _bb_get(
                blackboard, 'verify_navigation_legs', 0) + 1
            blackboard['verify_search_failures'] = 0
            blackboard['verify_inspection_after_ns'] = self._now_ns()
            blackboard['verify_arrival_after_ns'] = self._now_ns()
            self.executive._report(
                'Verify: terrain smoke leg complete; reassessing the next short leg')
            return 'reacquire'
        except _FireFound:
            return self._route_fire(blackboard, after_motion=True)
        except (VerificationStartUnavailable, VerificationRecoveryUnavailable) as error:
            return self._navigation_start_unavailable(blackboard, error, resume='search')
        except _InspectionTimeout:
            return self._search_blocked(blackboard, 'smoke search deadline reached')
        except VerificationObservationLost:
            blackboard['verify_inspection_after_ns'] = self._now_ns()
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
             {'approach': 'APPROACH_SMOKE', 'reacquire': 'REACQUIRE', 'fire': 'CENTER_FIRE'}),
            ('REACQUIRE', controller.reacquire,
             {'approach': 'APPROACH_SMOKE', 'fire': 'CENTER_FIRE', 'search': 'SEARCH_FIRE',
              'reacquire': 'REACQUIRE'}),
            ('CENTER_FIRE', controller.center_fire,
             {'reacquire': 'REACQUIRE', 'search': 'SEARCH_FIRE', 'fire': 'CENTER_FIRE'}),
            ('SEARCH_FIRE', controller.search_fire,
             {'reacquire': 'REACQUIRE', 'fire': 'CENTER_FIRE', 'inspect': 'CENTER_FIRE',
              'search': 'SEARCH_FIRE'}),
        )
        for name, operation, routes in states:
            transitions = {outcome: outcome for outcome in _FINAL_OUTCOMES}
            transitions.update(routes)
            self.add_state(name, _VerifyState(executive, name, operation, list(routes)),
                           transitions=transitions)
        self.set_start_state('APPROACH_SMOKE')
