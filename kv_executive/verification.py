"""Bounded, observation-driven verification; all translations use the planner.

Camera coverage determines a conservative *step*, never a measured distance.
There is no attitude-control interface here: the search keeps the aircraft level.
"""

from copy import deepcopy
import math
import threading
import time

from kv_executive.mission_geometry import quaternion_from_yaw, yaw_from_quaternion
from yasmin import State, StateMachine


VERIFY_DEFAULTS = {
    'verify_observation_timeout': 2.0,
    'verify_future_tolerance': 0.25,
    'verify_fire_persistence': 1.5,
    'verify_fire_min_frames': 3,
    'verify_track_center_tolerance': 0.08,
    'verify_track_area_ratio': 2.0,
    'verify_reacquire_timeout': 15.0,
    'verify_center_timeout': 30.0,
    'verify_yaw_tolerance': math.radians(5.0),
    'verify_yaw_step': math.radians(30.0),
    'verify_vertical_step': 0.5,
    'verify_minimum_altitude': 2.0,
    'verify_step_min': 5.0,
    'verify_step_max': 20.0,
    'verify_full_coverage': 0.25,
    'verify_target_fire_fraction': 0.005,
    'verify_creep_step': 0.75,
    'verify_creep_max_steps': 4,
    'verify_motion_speed': 0.5,
    'verify_escape_distance': 10.0,
    'verify_escape_climb': 10.0,
    'verify_search_hold': 3.0,
    'verify_max_approaches': 8,
}


class VerificationObservationLost(RuntimeError):
    """A motion's perception guard failed; stop and reacquire from LOITER."""


class _FireFound(VerificationObservationLost):
    """Interrupt a bounded search as soon as a persistent fire is available."""


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

    def _now_ns(self):
        return self.executive.get_clock().now().nanoseconds

    def _valid_stamp(self, message):
        stamp = message.header.stamp
        source_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        now_ns = self._now_ns()
        age = (now_ns - source_ns) / 1e9
        if (source_ns <= max(0, self._epoch_ns) or now_ns <= 0
                or not -self.config['verify_future_tolerance'] <= age
                <= self.config['verify_observation_timeout']):
            return None
        return source_ns

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

    def _receive(self, message, source):
        with self._lock:
            source_ns = self._valid_stamp(message)
            if source_ns is None:
                return
            if source_ns <= self._last_stamps[source]:
                return  # Duplicate/reordered images cannot establish persistence.
            self._last_stamps[source] = source_ns
            if not self._confirmed(message, source):
                self._observations[source] = None
                if source == 'fire':
                    self._fire_frames = 0
                    self._fire_reference = None
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
                    self._fire_start_ns = source_ns
                    self._fire_frames = 0
                    self._fire_reference = deepcopy(message)
                self._fire_frames += 1
            self._observations[source] = (deepcopy(message), source_ns, now)

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
            if (message is None or observation is None
                    or self._fire_frames < self.config['verify_fire_min_frames']
                    or (observation[1] - self._fire_start_ns) / 1e9
                    < self.config['verify_fire_persistence']):
                return None
            return message

    @staticmethod
    def all_quadrants(smoke):
        """Treat any accepted coverage in all four regions as smoke immersion."""
        return smoke is not None and (
            {1, 2, 3, 4}.issubset(set(smoke.regions))
            or all(math.isfinite(value) and value > 0.0
                   for value in smoke.quadrant_coverage)
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

    def _search_guard(self):
        self._check()
        if self.stable_fire() is not None:
            raise _FireFound('Persistent fire found during search')

    def _count_cycle(self, blackboard):
        count = _bb_get(blackboard, 'verify_approaches', 0) + 1
        if count > self.config['verify_max_approaches']:
            raise self.error_type('Verify approach limit reached; holding for operator')
        blackboard['verify_approaches'] = count

    def approach_smoke(self, blackboard):
        """Save a recovery reference, then plan one 5–20 m smoke-directed step."""
        if not _bb_get(blackboard, 'verify_entered', False):
            self.executive._verify_parent_entry('Verify')
            blackboard['verify_entered'] = True
        self._check()
        self._count_cycle(blackboard)
        if self.stable_fire() is not None:
            return 'reacquire'
        smoke = self.fresh_smoke()
        if smoke is None or self.all_quadrants(smoke):
            return 'reacquire'
        reference = self._pose()
        blackboard['verify_reference_pose'] = reference
        distance = self.approach_distance(smoke)
        blackboard['verify_next_step'] = distance
        yaw = yaw_from_quaternion(reference.orientation) + smoke.yaw_correction_rad
        target = deepcopy(reference)
        target.position.x += distance * math.cos(yaw)
        target.position.y += distance * math.sin(yaw)
        target.orientation = quaternion_from_yaw(yaw)
        try:
            self._smoke_guard()  # Do not start a plan on a vanished observation.
            self.executive._verify_plan_to(
                target, guard=self._smoke_guard,
                speed=self.config['verify_motion_speed'],
            )
        except VerificationObservationLost:
            self.executive._report('Verify: stopped approach to reacquire smoke')
        return 'reacquire'

    def reacquire(self, blackboard):
        """Recover reference heading/altitude, center smoke, and select a branch."""
        self._check()
        deadline = time.monotonic() + self.config['verify_reacquire_timeout']
        reference = _bb_get(blackboard, 'verify_reference_pose', None) or self._pose()
        reference_yaw = yaw_from_quaternion(reference.orientation)
        if self.fresh_smoke() is None and self.fresh_fire() is None:
            try:
                self.executive._verify_rotate(reference_yaw, guard=self._search_guard)
                target = self._pose()
                if abs(target.position.z - reference.position.z) > 0.3:
                    target.position.z = reference.position.z
                    self.executive._verify_plan_to(
                        target, guard=self._search_guard,
                        speed=self.config['verify_motion_speed'],
                    )
                for offset in (30.0, -30.0, 0.0):
                    if self.fresh_smoke() is not None:
                        break
                    self.executive._verify_rotate(
                        reference_yaw + math.radians(offset),
                        guard=self._search_guard, hold_duration=0.5,
                    )
            except _FireFound:
                return 'fire'
        while time.monotonic() < deadline:
            self._check()
            if self.stable_fire() is not None:
                return 'fire'
            smoke = self.fresh_smoke()
            if smoke is None:
                self._pause()
                continue
            correction = smoke.yaw_correction_rad
            if abs(correction) > self.config['verify_yaw_tolerance']:
                yaw = yaw_from_quaternion(self._pose().orientation)
                correction = max(-self.config['verify_yaw_step'], min(
                    self.config['verify_yaw_step'], correction))
                try:
                    self.executive._verify_rotate(
                        yaw + correction, guard=self._visible_smoke_guard,
                    )
                except VerificationObservationLost:
                    continue
                if self._after_motion('smoke', smoke, deadline) is None:
                    return 'fire' if self.stable_fire() is not None else 'explore'
                continue
            if self.all_quadrants(smoke):
                return 'search'
            blackboard['verify_next_step'] = self.approach_distance(smoke)
            return 'approach'
        self.executive._report('Verify: no centered smoke or persistent fire; exploring')
        return 'explore'

    def _fire_guard(self, original, prevent_smoke=False):
        self._check()
        fire = self.fresh_fire()
        if (fire is None or fire.header.frame_id != original.header.frame_id
                or getattr(fire, 'track_id', 0) != getattr(original, 'track_id', 0)):
            raise VerificationObservationLost('Tracked fire observation lost')
        if prevent_smoke:
            self._smoke_guard()

    def _escape(self, bearing, climb=0.0):
        """Face away and use a collision-checked retreat at level attitude."""
        away = bearing + math.pi
        self.executive._verify_rotate(away)
        target = self._pose()
        target.orientation = quaternion_from_yaw(away)
        target.position.x += self.config['verify_escape_distance'] * math.cos(away)
        target.position.y += self.config['verify_escape_distance'] * math.sin(away)
        target.position.z += climb
        self.executive._verify_plan_to(target, speed=self.config['verify_motion_speed'])

    def center_fire(self, blackboard):
        """Center one persistent thermal track, report it, and retreat safely."""
        original = self.stable_fire()
        if original is None:
            return 'reacquire'
        deadline = time.monotonic() + self.config['verify_center_timeout']
        creep_steps = 0

        def guard():
            self._fire_guard(original)

        try:
            while time.monotonic() < deadline:
                guard()
                fire = self.fresh_fire()
                pose = self._pose()
                yaw = yaw_from_quaternion(pose.orientation)
                horizontal = fire.yaw_correction_rad
                vertical = fire.vertical_correction_rad
                if self.all_quadrants(self.fresh_smoke()):
                    # A persistent fire would immediately interrupt SEARCH_FIRE.
                    # Retreat first rather than bouncing between those states.
                    self._count_cycle(blackboard)
                    bearing = yaw + horizontal
                    self._escape(bearing)
                    reference = self._pose()
                    reference.orientation = quaternion_from_yaw(bearing)
                    blackboard['verify_reference_pose'] = reference
                    with self._lock:
                        self._observations['fire'] = None
                        self._fire_frames = 0
                        self._fire_reference = None
                    return 'reacquire'
                if abs(horizontal) > self.config['verify_yaw_tolerance']:
                    step = max(-self.config['verify_yaw_step'], min(
                        self.config['verify_yaw_step'], horizontal))
                    self.executive._verify_rotate(yaw + step, guard=guard)
                    if self._after_motion('fire', fire, deadline) is None:
                        return 'reacquire'
                    continue
                if abs(vertical) > self.config['verify_yaw_tolerance']:
                    target = deepcopy(pose)
                    target.position.z = max(
                        self.config['verify_minimum_altitude'],
                        pose.position.z + math.copysign(
                            self.config['verify_vertical_step'], vertical),
                    )
                    if abs(target.position.z - pose.position.z) < 0.01:
                        raise self.error_type('Cannot center fire above minimum safe altitude')
                    self.executive._verify_plan_to(
                        target, guard=guard, speed=self.config['verify_motion_speed'],
                    )
                    if self._after_motion('fire', fire, deadline) is None:
                        return 'reacquire'
                    continue
                stable = self.stable_fire()
                if stable is None:
                    self._pause()
                    continue
                smoke = self.fresh_smoke()
                _, _, area_fraction = self._fire_geometry(stable)
                can_creep = (
                    smoke is not None and area_fraction
                    < self.config['verify_target_fire_fraction']
                    and creep_steps < self.config['verify_creep_max_steps']
                )
                if can_creep:
                    target = deepcopy(pose)
                    target.position.x += self.config['verify_creep_step'] * math.cos(yaw)
                    target.position.y += self.config['verify_creep_step'] * math.sin(yaw)
                    self.executive._verify_plan_to(
                        target, guard=lambda: self._fire_guard(original, prevent_smoke=True),
                        speed=self.config['verify_motion_speed'],
                    )
                    creep_steps += 1
                    if self._after_motion('fire', fire, deadline) is None:
                        return 'reacquire'
                    continue
                guard()
                self.executive._verify_record_fire(stable, yaw)
                self._escape(yaw)
                return 'explore'
        except VerificationObservationLost:
            self.executive._report('Verify: fire/smoke guard stopped centering; reacquiring')
            return 'reacquire'
        raise self.error_type('Verify fire-centering deadline exceeded')

    def search_fire(self, blackboard):
        """Level search: dwell, left 60°, right 120°, left 60°, then retreat."""
        self._check()
        yaw = yaw_from_quaternion(self._pose().orientation)
        try:
            self._search_guard()
            self.executive._verify_hold(
                self.config['verify_search_hold'], guard=self._search_guard,
            )
            for offset in (60.0, -60.0, 0.0):
                self.executive._verify_rotate(
                    yaw + math.radians(offset), guard=self._search_guard,
                    hold_duration=self.config['verify_search_hold'],
                )
            self._search_guard()
        except _FireFound:
            return 'fire'
        self._escape(yaw, climb=self.config['verify_escape_climb'])
        self.executive._verify_rotate(yaw)
        return 'approach'


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
            return self.operation(blackboard)
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
             {'reacquire': 'REACQUIRE'}),
            ('REACQUIRE', controller.reacquire,
             {'approach': 'APPROACH_SMOKE', 'fire': 'CENTER_FIRE', 'search': 'SEARCH_FIRE'}),
            ('CENTER_FIRE', controller.center_fire,
             {'reacquire': 'REACQUIRE'}),
            ('SEARCH_FIRE', controller.search_fire,
             {'approach': 'APPROACH_SMOKE', 'fire': 'CENTER_FIRE'}),
        )
        for name, operation, routes in states:
            transitions = {outcome: outcome for outcome in _FINAL_OUTCOMES}
            transitions.update(routes)
            self.add_state(name, _VerifyState(executive, name, operation, list(routes)),
                           transitions=transitions)
        self.set_start_state('APPROACH_SMOKE')
