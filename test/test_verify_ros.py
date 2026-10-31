"""Exercise detection-driven Verify transitions through DDS and a simulated FCU."""

from copy import deepcopy
import math
import threading
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PointStamped, TransformStamped
from kv_executive.detection_monitor import DetectionMonitor
from kv_executive.mission_geometry import (
    angular_distance, quaternion_from_yaw, yaw_from_quaternion,
)
from kv_executive.msg import (
    BatteryReturnEstimate, FireSourceReport, InspectionReport, MissionPhase, MissionStateRequest,
)
from kv_executive.srv import QueryInspectionGeometry
from kv_perception.msg import FireDetection, SmokeDetection
import pytest
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from test_mission_action import (
    cancel_goal, mission as mission_fixture, mission_result, prepare_airborne,
    received_state, send_goal, wait_for,
)
from tf2_ros import StaticTransformBroadcaster


mission = mission_fixture


class MonitoringSensors(Node):
    """Publish confirmed detector observations and typed battery estimates."""

    def __init__(self):
        super().__init__('verify_test_sensors')
        self.lock = threading.Lock()
        self.source = None
        self.smoke_guidance = None
        self.fire_guidance = None
        self.smoke_coverage = 0.1
        self.smoke_regions = None
        self.smoke_suppressed_until = 0.0
        self.fire_suppressed_until = 0.0
        self.fire_confirmation_frames = 50
        self.fire_persistence_sec = 2.0
        self.fire_positive_times = []
        self.smoke_positive_times = []
        self.trusted_source = True
        self.smoke_boundary_range = None
        self.smoke_wall_range = None
        self.detection_latency = 0.0
        self.closed = False
        self.phases = []
        self.reports = []
        self.inspection_reports = []
        self.inspection_report_times = []
        self.requests = []
        reliable = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        retained = QoSProfile(
            depth=100, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.smoke_publisher = self.create_publisher(
            SmokeDetection, 'smoke/detection', reliable,
        )
        self.fire_publisher = self.create_publisher(
            FireDetection, 'fire/detection', reliable,
        )
        self.battery_publisher = self.create_publisher(
            BatteryReturnEstimate, 'battery_return_estimate', reliable,
        )
        self.phase_subscription = self.create_subscription(
            MissionPhase, 'mission_phase', self.receive_phase, retained,
        )
        self.report_subscription = self.create_subscription(
            FireSourceReport, 'fire_sources', self.receive_report, retained,
        )
        self.inspection_report_subscription = self.create_subscription(
            InspectionReport, 'inspection_reports', self.receive_inspection_report, retained,
        )
        self.request_subscription = self.create_subscription(
            MissionStateRequest, 'mission_state_request_for_goal',
            self.receive_request, reliable,
        )
        self.timer = self.create_timer(0.05, self.publish_observations)

    def receive_phase(self, message):
        with self.lock:
            self.phases.append(deepcopy(message))

    def receive_report(self, message):
        with self.lock:
            self.reports.append(deepcopy(message))

    def receive_inspection_report(self, message):
        with self.lock:
            self.inspection_reports.append(deepcopy(message))
            self.inspection_report_times.append(time.monotonic())

    def receive_request(self, message):
        with self.lock:
            self.requests.append(deepcopy(message))

    def publish_observations(self):
        with self.lock:
            if self.closed:
                return
            stamp = self.get_clock().now().to_msg()
            if self.detection_latency:
                source_ns = self.get_clock().now().nanoseconds - int(self.detection_latency * 1e9)
                stamp.sec, stamp.nanosec = divmod(source_ns, 10**9)
            smoke = SmokeDetection()
            smoke.header.stamp = stamp
            smoke.header.frame_id = 'test_rgb_camera'
            smoke.background_ready = True
            smoke.status = SmokeDetection.SEARCHING
            if (self.source in {'smoke', 'smoke_fire'}
                    and time.monotonic() >= self.smoke_suppressed_until):
                smoke.detected = True
                smoke.status = SmokeDetection.CONFIRMED
                smoke.confirmation_frames = 10
                smoke.region = SmokeDetection.TOP_LEFT
                # A small centered blob crosses the image quadrant boundaries;
                # occupying all four regions alone does not imply immersion.
                smoke.regions = [SmokeDetection.TOP_LEFT, SmokeDetection.TOP_RIGHT,
                                 SmokeDetection.BOTTOM_LEFT, SmokeDetection.BOTTOM_RIGHT]
                if self.smoke_regions is not None:
                    smoke.regions = self.smoke_regions
                smoke.quadrant_coverage = [
                    self.smoke_coverage if region in smoke.regions else 0.0
                    for region in (1, 2, 3, 4)
                ]
                smoke.coverage = self.smoke_coverage
                if self.smoke_guidance is not None:
                    (smoke.yaw_correction_rad,
                     smoke.vertical_correction_rad) = self.smoke_guidance()
                self.smoke_positive_times.append(time.monotonic())
            self.smoke_publisher.publish(smoke)
            fire = FireDetection()
            fire.header.stamp = stamp
            fire.header.frame_id = 'test_thermal_camera'
            fire.status = FireDetection.SEARCHING
            if (self.source in {'fire', 'smoke_fire'}
                    and time.monotonic() >= self.fire_suppressed_until):
                fire.detected = True
                fire.status = FireDetection.CONFIRMED
                fire.image_width = 640
                fire.image_height = 480
                fire.confirmation_frames = self.fire_confirmation_frames
                fire.persistence_sec = self.fire_persistence_sec
                fire.track_id = 42
                fire.source_count = 2
                fire.area_px = 2000
                fire.bbox_x = 295
                fire.bbox_y = 220
                fire.bbox_width = 50
                fire.bbox_height = 40
                fire.center_x = 319.5
                fire.center_y = 239.5
                if self.fire_guidance is not None:
                    (fire.yaw_correction_rad,
                     fire.vertical_correction_rad) = self.fire_guidance()
                self.fire_positive_times.append(time.monotonic())
            self.fire_publisher.publish(fire)

    def battery_alert(self, fcu, *, available=True):
        with fcu.lock:
            pose = deepcopy(fcu.pose)
        stamp = self.get_clock().now().to_msg()
        message = BatteryReturnEstimate()
        message.header.stamp = stamp
        message.header.frame_id = 'px4_odom'
        message.current_pose.header = deepcopy(message.header)
        message.current_pose.pose = pose
        message.home_pose.header = deepcopy(message.header)
        message.home_pose.pose = deepcopy(pose)
        message.remaining_percent = 8.0
        message.buffered_required_percent = 10.0
        message.return_margin_percent = -2.0
        message.home_distance_m = 0.0
        message.mean_percent_per_meter = 0.1 if available else math.nan
        message.return_required_percent = 0.0 if available else math.nan
        message.sample_count = 1 if available else 0
        message.estimate_available = available
        message.return_alert = True
        return message

    def snapshot(self):
        with self.lock:
            return deepcopy(self.phases), deepcopy(self.reports)

    def inspection_snapshot(self):
        with self.lock:
            return deepcopy(self.inspection_reports)


@pytest.fixture
def monitoring_sensors(mission):
    executive, fcu = mission.executive, mission.fcu
    with fcu.lock:
        fcu.offboard_loss_timeout = 2.0
        fcu.heartbeat_interval = 0.2
    # Keep real planning, mode handshakes and observation guards, while bounding
    # the distance and persistence needed by this simulated flight.
    settings = {
        'verify_step_min': 0.4,
        'verify_step_max': 0.8,
        'verify_motion_speed': 3.0,
        'verify_fire_persistence': 0.15,
        'verify_reacquire_timeout': 0.3,
        'verify_reacquire_observation_timeout': 0.5,
        'verify_viewpoint_max_steps': 1,
        'verify_viewpoint_climb': 0.4,
        'verify_viewpoint_timeout': 15.0,
        'verify_search_hold': 0.2,
        'verify_inspection_lateral_step': 0.4,
        'verify_inspection_timeout': 90.0,
        'verify_navigation_step': 0.8,
        'verify_navigation_map_wait': 0.2,
        # Observation-only cases supply an independently measured nearby wall.
        # Dedicated approach cases below enable movement toward a fixed wall.
        'verify_fire_approach_max_steps': 0,
    }
    executive.config.update(settings)
    executive._verify.config.update(settings)
    sensors = MonitoringSensors()
    sensors.smoke_wall_range = (executive.config['verify_smoke_standoff'] + 0.1
                                + executive.config['position_tolerance'])
    original_geometry = fcu.query_inspection_geometry

    def measured_smoke_wall(request, response):
        response = original_geometry(request, response)
        with sensors.lock:
            distance = sensors.smoke_wall_range
        if request.smoke_wall_width and distance is not None:
            response.smoke_wall_range = response.smoke_wall_nearest_range = distance
        return response

    fcu.geometry_service.callback = measured_smoke_wall
    sensors.camera_transforms = StaticTransformBroadcaster(sensors)
    transforms = []
    for camera in ('test_rgb_camera', 'test_thermal_camera'):
        transform = TransformStamped()
        transform.header.frame_id = executive.config['robot_base_frame']
        transform.header.stamp = sensors.get_clock().now().to_msg()
        transform.child_frame_id = camera
        transform.transform.rotation.w = 1.0
        transforms.append(transform)
    sensors.camera_transforms.sendTransform(transforms)
    target_geometry = executive._verify_target_geometry

    def simulated_localized_source(message, source='smoke'):
        # Isolate controller/flight integration from semantic association. Real
        # multiview fitting and native map planning have separate regressions.
        geometry = target_geometry(message, source)
        with sensors.lock:
            trusted = sensors.trusted_source
        with fcu.lock:
            point = deepcopy(fcu.inspection_target)
            observer = deepcopy(fcu.pose.position)
        if source == 'smoke':
            # This fixture explicitly models the observed, flickering smoke
            # boundary. It never substitutes the buried fire's range for it.
            with sensors.lock:
                boundary_range = sensors.smoke_boundary_range
            if boundary_range is not None:
                bearing = geometry.get('navigation_bearing', 0.0)
                geometry.update(
                    obstacle_point=Point(
                        x=observer.x + boundary_range * math.cos(bearing),
                        y=observer.y + boundary_range * math.sin(bearing), z=observer.z),
                    obstacle_range=boundary_range, obstacle_uncertainty=0.1,
                    plume_candidate=True, source='smoke',
                    occupied_samples=3, free_samples=1,
                    source_stamp=deepcopy(message.header.stamp))
            else:
                # An arbitrary occupied surface does not establish a plume.
                # Smoke-boundary scenarios provide explicit evidence above.
                geometry['plume_candidate'] = False
            return geometry
        if trusted and point is not None:
            planning = PointStamped(header=deepcopy(message.header), point=deepcopy(point))
            planning.header.frame_id = executive.config['verify_planning_frame']
            geometry.update(
                point=point, planning_point=planning, target_valid=True, uncertainty=0.1,
                source_stamp=deepcopy(message.header.stamp), source=source,
                method='thermal_multiview_bearings',
                range=math.dist(_point_position(point), _point_position(observer)))
        elif not trusted:
            geometry.update(point=None, planning_point=None, target_valid=False,
                            method='unavailable_semantic_source_range')
        return geometry

    executive._verify_target_geometry = simulated_localized_source
    monitor = DetectionMonitor()
    executor = executive.executor
    executor.add_node(sensors)
    executor.add_node(monitor)
    try:
        wait_for(
            lambda: sensors.smoke_publisher.get_subscription_count() >= 2
            and sensors.fire_publisher.get_subscription_count() >= 2
            and sensors.battery_publisher.get_subscription_count() >= 1,
            3.0, 'detector monitor and executive battery discovery',
        )
        yield sensors
    finally:
        with sensors.lock:
            sensors.closed = True
            sensors.timer.cancel()
        executor.remove_node(monitor)
        executor.remove_node(sensors)
        monitor.destroy_node()
        sensors.destroy_node()


def _start_airborne_exploration(mission):
    mission.executive.config['sweep_step_duration'] = 0.5
    prepare_airborne(mission)
    goal = send_goal(mission, skip_takeoff=True)
    assert goal.accepted
    wait_for(
        lambda: received_state(mission, mode='OFFBOARD'),
        10.0, 'Exploration survey OFFBOARD',
    )
    return goal


def _assert_aborted_and_loitering(mission, sensors, result, *, verified=False):
    assert result.status == GoalStatus.STATUS_ABORTED
    assert not result.result.success
    assert 'AUTO.LOITER confirmed' in result.result.message
    phases, reports = sensors.snapshot()
    expected = {'Exploration', 'Verify'} if verified else {'Exploration'}
    assert phases and {phase.state for phase in phases} == expected
    assert not reports
    with mission.fcu.lock:
        assert mission.fcu.state.mode == 'AUTO.LOITER'
        assert mission.fcu.state.armed
        assert bool(mission.fcu.target_requests) == verified
        assert not mission.fcu.arming_requests
        assert not {'AUTO.TAKEOFF', 'AUTO.RTL', 'AUTO.LAND'} & set(
            mission.fcu.mode_requests)
        assert not mission.fcu.offboard_losses
    assert mission.executive._goal_handle is None
    assert not mission.executive._mission_lock.locked()


def _states(sensors):
    return [phase.state for phase in sensors.snapshot()[0]]


def _assert_same_goal_phases(sensors, goal):
    phases, _ = sensors.snapshot()
    assert all(tuple(phase.goal_id.uuid) == tuple(goal.goal_id.uuid) for phase in phases)
    sequences = [phase.sequence for phase in phases]
    assert sequences == sorted(set(sequences))


def _assert_airborne_safety(mission):
    with mission.fcu.lock:
        assert mission.fcu.state.armed
        assert not mission.fcu.arming_requests
        assert not {'AUTO.TAKEOFF', 'AUTO.RTL', 'AUTO.LAND'} & set(
            mission.fcu.mode_requests)
        assert not mission.fcu.offboard_losses


def _start_smoke_approach(mission, sensors):
    goal = _start_airborne_exploration(mission)
    with sensors.lock:
        sensors.source = 'smoke'
    wait_for(
        lambda: 'Verify' in _states(sensors),
        6.0, 'confirmed smoke to enter Verify',
    )
    settings = mission.executive.config
    wait_for(
        lambda: any(text == 'Verify: entered' for _, text in mission.fcu.statuses),
        2.0 * settings['mode_timeout'] + settings['offboard_warmup'],
        'Verify LOITER handoff before smoke centering',
    )
    result = goal.get_result_async()

    def requested():
        assert not result.done(), 'Mission ended before smoke approach planning: ' + (
            result.result().result.message)
        return bool(mission.fcu.target_requests)

    # Yaw correction can require a full OFFBOARD warmup and LOITER handoff
    # before vertical correction requests its first collision-checked target.
    wait_for(
        requested,
        settings['verify_smoke_center_timeout'] + settings['planning_timeout'],
        'smoke centering to request a collision-checked target',
    )
    return goal


def _complete_initial_smoke_approach(mission, sensors, *, source='smoke', coverage=0.3):
    """Fly the real initial smoke leg before exposing a close-inspection source."""
    after = time.monotonic()
    with sensors.lock:
        sensors.smoke_coverage = 0.1
        sensors.source = 'smoke'
    wait_for(
        lambda: any(stamp > after
                    and text == 'Verify: approach destination reached; reacquiring smoke or fire'
                    for stamp, text in mission.fcu.statuses),
        15.0, 'the initial smoke-directed planned flight to arrive',
    )
    with sensors.lock:
        sensors.smoke_coverage = coverage
        sensors.source = source


def _position(pose):
    return tuple(getattr(pose.position, field) for field in 'xyz')


def _point_position(point):
    return tuple(getattr(point, field) for field in 'xyz')


def _assert_arrival_before_reacquire(mission, target, *, after=0.0):
    """Require the commanded endpoint before the first post-approach search."""
    wait_for(lambda: any(stamp > after and text == 'Verify/REACQUIRE: started'
                         for stamp, text in mission.fcu.statuses),
             3.0, 'REACQUIRE status delivery after the arrived leg')
    with mission.fcu.lock:
        reacquired_at = next(
            stamp for stamp, text in mission.fcu.statuses
            if stamp > after and text == 'Verify/REACQUIRE: started'
        )
        setpoints = deepcopy(mission.fcu.setpoints)
    flight = [message.pose for stamp, mode, message in setpoints
              if after < stamp < reacquired_at and mode == 'OFFBOARD']
    assert any(math.dist(_position(pose), _position(target)) < 1e-6
               for pose in flight), 'REACQUIRE started before the planned endpoint'
    return flight


def _assert_terrain_planning(mission, *, fire_standoff=False):
    with mission.fcu.lock:
        requests = deepcopy(mission.fcu.inspection_planning_requests)
    assert requests
    assert all(request.terrain_envelope and request.minimum_agl >= 5.0 for request in requests)
    if fire_standoff:
        assert any(request.minimum_radius > 0.0 and request.keepout_center.header.frame_id
                   for request in requests)
    else:
        assert all(request.minimum_radius == 0.0 for request in requests)
    return requests


def _use_fixed_smoke_wall(fcu, initial_distance):
    """Measure a fixed wall plane independently of the simulated fire target."""
    original = fcu.geometry_service.callback
    wall = []

    def query(request, response):
        response = original(request, response)
        if request.smoke_wall_width:
            origin = response.ray_origin.point
            if not wall:
                axis = deepcopy(response.ray_direction.vector)
                wall.append((Point(**{
                    a: getattr(origin, a) + initial_distance * getattr(axis, a)
                    for a in 'xyz'}), axis))
            point, axis = wall[0]
            distance = sum((getattr(point, a) - getattr(origin, a)) * getattr(axis, a)
                           for a in 'xyz')
            response.ray_direction.vector = deepcopy(axis)
            response.smoke_wall_range = response.smoke_wall_nearest_range = distance
        return response

    fcu.geometry_service.callback = query
    return wall


def _start_visible_fire(mission, sensors):
    goal = _start_airborne_exploration(mission)
    # Deliver fire first so a smoke-triggered request cannot race confirmation.
    with sensors.lock:
        sensors.source = 'fire'
    wait_for(lambda: mission.executive._verify.stable_fire() is not None,
             3.0, 'fresh producer-qualified fire')
    with sensors.lock:
        sensors.source = 'smoke_fire'
    wait_for(lambda: 'Verify' in _states(sensors), 6.0, 'visible fire verification')
    return goal


def _occlude_smoke_when_a_leg_stops(mission, sensors, *, duration=None, count=1):
    """Occlude camera observations at completed-leg LOITER handoffs."""
    fcu = mission.fcu
    original = fcu.set_mode
    original_plan = fcu.plan_to_pose
    fcu.smoke_plan_times = []
    misses = []

    def plan(request, response):
        with fcu.lock:
            fcu.smoke_plan_times.append(time.monotonic())
        return original_plan(request, response)

    def mode(request, response):
        response = original(request, response)
        with fcu.lock:
            completed_targets = len(fcu.target_requests)
            anchor = deepcopy(fcu.pose)
        if (request.custom_mode == 'AUTO.LOITER' and response.mode_sent
                and completed_targets > len(misses) and len(misses) < count):
            started = time.monotonic()
            with sensors.lock:
                sensors.smoke_suppressed_until = (
                    math.inf if duration is None else started + duration)
                last_positive = sensors.smoke_positive_times[-1]
            misses.append((started, last_positive, anchor, completed_targets))
        return response

    fcu.fake_services[0].callback = mode
    fcu.fake_services[-1].callback = plan
    return misses


def _assert_quiet_smoke_occlusion(mission, miss, until, *, check_current=True):
    started, _, anchor, target_count = miss
    with mission.fcu.lock:
        if check_current:
            assert len(mission.fcu.target_requests) == target_count
        assert not any(started <= stamp < until for stamp in mission.fcu.smoke_plan_times)
        held = [message.pose for stamp, _, message in mission.fcu.setpoints
                if started <= stamp <= until]
        current = deepcopy(mission.fcu.pose)
    if check_current:
        assert math.dist(_position(current), _position(anchor)) < 0.03
        assert abs(angular_distance(yaw_from_quaternion(current.orientation),
                                    yaw_from_quaternion(anchor.orientation))) < 0.05
    assert all(math.dist(_position(pose), _position(anchor)) < 0.03 for pose in held)
    assert all(abs(angular_distance(yaw_from_quaternion(pose.orientation),
                                    yaw_from_quaternion(anchor.orientation))) < 0.05
               for pose in held)


def _wait_for_smoke_grace(mission, *, after=0.0):
    wait_for(
        lambda: any(stamp >= after and 'smoke briefly occluded; holding position' in text
                    for stamp, text in mission.fcu.statuses),
        4.0, 'short smoke occlusion to hold the stopped viewpoint',
    )


def _delay_inspection_maps(fcu, *, duration=None):
    """Withhold source-fresh maps through the real geometry service."""
    original = fcu.query_inspection_geometry
    rejected = []
    fresh = []

    def delayed(request, response):
        now = time.monotonic()
        if not rejected or duration is None or now - rejected[0] < duration:
            with fcu.lock:
                fcu.geometry_requests.append(deepcopy(request))
            rejected.append(now)
            response.success = False
            response.error_code = response.MAP_UNAVAILABLE
            response.message = 'simulated source-stamped OctoMap is ten seconds old'
            response.map_header.frame_id = 'px4_odom'
            response.map_header.stamp = fcu.get_clock().now().to_msg()
            response.map_header.stamp.sec -= 10
            return response
        response = original(request, response)
        fresh.append(now)
        return response

    fcu.geometry_service.callback = delayed
    return rejected, fresh


def _wait_for_map_recovery(mission):
    wait_for(
        lambda: any('holding in LOITER for a newer fresh OctoMap' in text
                    for _, text in mission.fcu.statuses),
        8.0, 'Verify to await new source observations in LOITER',
    )
    wait_for(lambda: received_state(mission, mode='AUTO.LOITER'),
             3.0, 'map recovery LOITER mode confirmation')


def test_stale_map_recovery_holds_then_resumes_the_same_smoke_goal(
    mission, monitoring_sensors,
):
    sensors, fcu = monitoring_sensors, mission.fcu
    rejected, fresh = _delay_inspection_maps(fcu, duration=2.5)
    goal = _start_airborne_exploration(mission)
    with fcu.lock:
        origin = deepcopy(fcu.pose.position)
        # Isolate map recovery from the separately covered terrain-ascent
        # search: this short horizontal leg starts at the preferred clearance.
        fcu.inspection_terrain_z = origin.z - 7.5
    with sensors.lock:
        sensors.source = 'smoke'
    _wait_for_map_recovery(mission)
    assert rejected and not fresh
    assert not goal.get_result_async().done()
    with fcu.lock:
        assert not fcu.target_requests
        assert fcu.state.mode == 'AUTO.LOITER'
    wait_for(lambda: bool(fresh), 5.0, 'a new fresh map after the stale interval')
    assert fresh[0] - rejected[0] >= 2.5
    with fcu.lock:
        waiting = [message.pose.position for stamp, _, message in fcu.setpoints
                   if rejected[0] <= stamp < fresh[0]]
        assert math.dist(_point_position(fcu.pose.position), _point_position(origin)) < 0.03
        assert all(math.dist(_point_position(point), _point_position(origin)) < 0.03
                   for point in waiting)
    settings = mission.executive.config
    wait_for(
        lambda: bool(fcu.inspection_planning_requests),
        settings['planning_timeout'] + settings['service_timeout'],
        'new source observations to permit guarded smoke planning',
    )

    def translated():
        with fcu.lock:
            return math.dist(_point_position(fcu.pose.position), _point_position(origin)) > 0.05

    # Full-leg completion is covered by the smoke flight regressions. Here the
    # recovery must retain its action and begin real motion after fresh evidence.
    wait_for(
        translated,
        settings['offboard_warmup'] + 2.0 * settings['mode_timeout']
        + settings['waypoint_timeout'],
        'the same action to begin its terrain-checked smoke leg',
    )
    with sensors.lock:
        sensors.source = None
    assert not goal.get_result_async().done()
    _assert_terrain_planning(mission)
    _assert_same_goal_phases(sensors, goal)
    cancel_goal(goal)
    _assert_airborne_safety(mission)


def test_repeated_map_outages_resume_after_offboard_warmup_outlasts_first_budget(
    mission, monitoring_sensors,
):
    executive, fcu, sensors = mission.executive, mission.fcu, monitoring_sensors
    executive.config['offboard_warmup'] = 2.0
    executive.config['verify_navigation_map_timeout'] = 1.5
    executive._verify.config['verify_navigation_map_timeout'] = 1.5
    original = fcu.query_inspection_geometry
    outages = []

    def interrupted_maps(request, response):
        now = time.monotonic()
        if not outages:
            outages.append(now)
        unavailable = now - outages[0] < 0.6
        with fcu.lock:
            during_flight = fcu.state.mode == 'OFFBOARD' and len(request.corridor.poses) > 1
        if not unavailable and len(outages) == 1 and during_flight:
            # The first plan succeeded, but its post-warmup map check fails.
            # New data is available again before the LOITER recovery poll.
            outages.append(now)
            unavailable = True
        if unavailable:
            with fcu.lock:
                fcu.geometry_requests.append(deepcopy(request))
            response.success = False
            response.error_code = response.MAP_UNAVAILABLE
            response.message = 'temporary map outage around OFFBOARD handoff'
            response.map_header.frame_id = 'px4_odom'
            response.map_header.stamp = fcu.get_clock().now().to_msg()
            response.map_header.stamp.sec -= 10
            return response
        return original(request, response)

    fcu.geometry_service.callback = interrupted_maps
    goal = _start_airborne_exploration(mission)
    result = goal.get_result_async()
    with fcu.lock:
        origin = deepcopy(fcu.pose.position)
        fcu.inspection_terrain_z = origin.z - 7.5
    with sensors.lock:
        sensors.source = 'smoke'

    def resumed_motion():
        assert not result.done(), 'Temporary map outages ended the mission: ' + (
            result.result().result.message)
        with fcu.lock:
            return (len(fcu.inspection_planning_requests) >= 2
                    and math.dist(_point_position(fcu.pose.position),
                                  _point_position(origin)) > 0.05)

    wait_for(resumed_motion, 18.0, 'same smoke goal to move after both map outages')
    assert len(outages) == 2
    assert outages[1] - outages[0] > executive.config['verify_navigation_map_timeout']
    recoveries = [text for _, text in fcu.statuses
                  if 'holding in LOITER for a newer fresh OctoMap' in text]
    assert len(recoveries) == 2
    assert all('recovery budget 0.0 s' not in text for text in recoveries)
    _assert_same_goal_phases(sensors, goal)
    _assert_terrain_planning(mission)
    cancel_goal(goal)
    _assert_airborne_safety(mission)


@pytest.mark.parametrize('interrupt', ['cancel', 'battery'])
def test_map_recovery_wait_remains_responsive_to_mission_interrupts(
    mission, monitoring_sensors, interrupt,
):
    sensors, fcu = monitoring_sensors, mission.fcu
    rejected, fresh = _delay_inspection_maps(fcu)
    goal = _start_airborne_exploration(mission)
    with sensors.lock:
        sensors.source = 'smoke'
    _wait_for_map_recovery(mission)
    assert rejected and not fresh
    interrupted = time.monotonic()
    if interrupt == 'cancel':
        result = cancel_goal(goal)
        assert result.status == GoalStatus.STATUS_CANCELED
    else:
        sensors.battery_publisher.publish(sensors.battery_alert(fcu))
        result = mission_result(goal)
        assert result.status == GoalStatus.STATUS_ABORTED
        assert any('BATTERY: return budget alert' in text for _, text in fcu.statuses)
    assert time.monotonic() - interrupted < 5.0
    assert not result.result.success
    with fcu.lock:
        assert not fcu.target_requests
        assert fcu.state.mode == 'AUTO.LOITER'
    _assert_same_goal_phases(sensors, goal)
    _assert_airborne_safety(mission)


def test_map_recovery_tolerates_a_temporarily_missing_geometry_service(
    mission, monitoring_sensors,
):
    executive, fcu, sensors = mission.executive, mission.fcu, monitoring_sensors
    _delay_inspection_maps(fcu)
    goal = _start_airborne_exploration(mission)
    result = goal.get_result_async()
    with fcu.lock:
        origin = deepcopy(fcu.pose.position)
        fcu.inspection_terrain_z = origin.z - 7.5
    with sensors.lock:
        sensors.source = 'smoke'
    _wait_for_map_recovery(mission)
    actual_client = executive._inspection_geometry_client
    missing_client = executive.create_client(
        QueryInspectionGeometry, 'temporarily_missing_inspection_geometry')
    finished = False
    try:
        assert not missing_client.service_is_ready()
        executive._inspection_geometry_client = missing_client
        missing_since = time.monotonic()
        wait_for(
            lambda: 'Inspection geometry service unavailable' in getattr(
                executive, '_verify_map_unavailable_reason', ''),
            2.0, 'missing geometry service poll to time out recoverably',
        )

        def remained_active():
            assert not result.done(), 'Mission ended during a temporary geometry service outage'
            return time.monotonic() - missing_since >= 1.5

        wait_for(remained_active, 3.0, 'recovery across repeated missing-service polls')
        with fcu.lock:
            assert fcu.state.mode == 'AUTO.LOITER'
            assert not fcu.target_requests
            assert math.dist(_point_position(fcu.pose.position), _point_position(origin)) < 0.03
        executive._inspection_geometry_client = actual_client
        fcu.geometry_service.callback = fcu.query_inspection_geometry
        wait_for(
            lambda: bool(fcu.inspection_planning_requests),
            10.0, 'the restored geometry service to resume guarded smoke planning',
        )
        assert not result.done()
        _assert_same_goal_phases(sensors, goal)
        cancel_goal(goal)
        finished = True
        _assert_airborne_safety(mission)
    finally:
        executive._inspection_geometry_client = actual_client
        # A failing test leaves client cleanup to the node fixture, after its
        # active mission worker has stopped. Never destroy an in-flight poll.
        if finished:
            executive.destroy_client(missing_client)


@pytest.mark.parametrize('loss_during', ['planning', 'warmup', 'flight'])
def test_smoke_dropout_completes_short_terrain_detour_before_reacquiring(
    mission, monitoring_sensors, loss_during,
):
    """A brief detector miss does not interrupt the current safe short leg."""
    sensors, fcu = monitoring_sensors, mission.fcu
    started, release = threading.Event(), threading.Event()
    routes = []
    original_plan = fcu.plan_to_pose

    def bent_plan(request, response):
        response = original_plan(request, response)
        start, end = deepcopy(response.path.poses[0]), response.path.poses[-1]
        corner = deepcopy(end)
        bearing = yaw_from_quaternion(end.pose.orientation)
        corner.pose.position.x = (start.pose.position.x + end.pose.position.x) / 2.0
        corner.pose.position.y = (start.pose.position.y + end.pose.position.y) / 2.0
        corner.pose.position.x -= 0.2 * math.sin(bearing)
        corner.pose.position.y += 0.2 * math.cos(bearing)
        corner.pose.orientation = quaternion_from_yaw(bearing + 1.0)
        response.path.poses.insert(1, corner)
        routes.append((time.monotonic(), deepcopy(start.pose), deepcopy(corner.pose),
                       deepcopy(end.pose)))
        if len(routes) == 1:
            started.set()
            assert release.wait(5.0)
        return response

    fcu.fake_services[-1].callback = bent_plan
    goal = _start_airborne_exploration(mission)
    with sensors.lock:
        sensors.source = 'smoke'
    try:
        wait_for(started.is_set, 6.0, 'terrain-aware short smoke plan')
        if loss_during == 'planning':
            with sensors.lock:
                sensors.source = None
            wait_for(lambda: mission.executive._verify.fresh_smoke() is None,
                     2.0, 'smoke miss while planning')
        release.set()
        planned_at, start, corner, target = routes[0]
        if loss_during == 'warmup':
            wait_for(lambda: any(stamp > planned_at and mode == 'AUTO.LOITER'
                                 for stamp, mode, _ in fcu.setpoints),
                     3.0, 'OFFBOARD warmup')
            with sensors.lock:
                sensors.source = None
        elif loss_during == 'flight':
            wait_for(lambda: math.dist(_position(fcu.pose), _position(start)) > 0.08,
                     8.0, 'short terrain flight')
            with sensors.lock:
                sensors.source = None
        arrived_status = 'Verify: approach destination reached; reacquiring smoke or fire'
        wait_for(lambda: any(text == arrived_status for _, text in fcu.statuses),
                 18.0, 'arrival before recovery from detector miss')
        flight = _assert_arrival_before_reacquire(mission, target, after=planned_at)
        assert any(math.dist(_position(pose), _position(corner)) < 1e-6 for pose in flight)
        assert math.hypot(target.position.x - start.position.x,
                          target.position.y - start.position.y) <= 1.0
        _assert_terrain_planning(mission)
        assert not sensors.inspection_snapshot()
        assert not goal.get_result_async().done()
        _assert_same_goal_phases(sensors, goal)
    finally:
        release.set()
        cancel_goal(goal)
    _assert_airborne_safety(mission)


def test_broad_smoke_alone_keeps_approaching_without_entering_timed_inspection(
    mission, monitoring_sensors,
):
    sensors, fcu = monitoring_sensors, mission.fcu
    goal = _start_airborne_exploration(mission)
    with sensors.lock:
        sensors.smoke_coverage = 0.4
        sensors.source = 'smoke'
    try:
        wait_for(lambda: len(fcu.target_requests) >= 2, 20.0,
                 'broad smoke to authorize initial and post-arrival smoke legs')
        with fcu.lock:
            statuses = [text for _, text in fcu.statuses]
        assert 'Verify: approach destination reached; reacquiring smoke or fire' in statuses
        assert not any(text in {'Verify/SEARCH_FIRE: started', 'Verify/CENTER_FIRE: started'}
                       for text in statuses)
        assert not sensors.inspection_snapshot()
    finally:
        cancel_goal(goal)
    _assert_airborne_safety(mission)


@pytest.mark.parametrize('detection_during', ['planning', 'flight'])
def test_smoke_leg_finishes_with_fire_heading_and_broad_smoke_before_reacquiring(
    mission, monitoring_sensors, detection_during,
):
    """Fire and broad coverage update observations without canceling a selected leg."""
    sensors, fcu = monitoring_sensors, mission.fcu
    for config in (mission.executive.config, mission.executive._verify.config):
        config['verify_motion_speed'] = 0.5
    started, release = threading.Event(), threading.Event()
    planned = []
    original = fcu.plan_to_pose

    def select_leg(request, response):
        response = original(request, response)
        if not planned:
            planned.append((time.monotonic(), deepcopy(response.path.poses[0].pose),
                            deepcopy(request.target.pose)))
            started.set()
            assert release.wait(5.0)
        return response

    fcu.fake_services[-1].callback = select_leg
    goal = _start_airborne_exploration(mission)
    with sensors.lock:
        sensors.source = 'smoke'
    try:
        wait_for(started.is_set, 6.0, 'selected smoke leg')
        planned_at, start, target = planned[0]
        desired_yaw = yaw_from_quaternion(target.orientation) + 0.15

        def fire_heading():
            with fcu.lock:
                yaw = yaw_from_quaternion(fcu.pose.orientation)
                # A low fire must not rewrite the already validated XYZ path.
                return angular_distance(desired_yaw, yaw), -0.2

        if detection_during == 'flight':
            release.set()
            wait_for(lambda: math.dist(_position(fcu.pose), _position(start)) > 0.08,
                     8.0, 'smoke translation before fire appears')
        with sensors.lock:
            sensors.smoke_coverage = 0.4
            sensors.fire_guidance = fire_heading
            sensors.source = 'smoke_fire'
        wait_for(lambda: mission.executive._verify.stable_fire() is not None,
                 2.0, 'fire confirmation during the selected smoke leg')
        release.set()
        wait_for(lambda: abs(angular_distance(
            yaw_from_quaternion(fcu.pose.orientation), desired_yaw)) < 0.06,
                 8.0, 'live fire-facing yaw without canceling the smoke path')
        with sensors.lock:
            sensors.source = 'smoke'
        wait_for(lambda: mission.executive._verify.stable_fire() is None,
                 2.0, 'fire loss before post-arrival reacquisition')
        arrived = 'Verify: approach destination reached; reacquiring smoke or fire'
        wait_for(lambda: any(stamp > planned_at and text == arrived
                             for stamp, text in fcu.statuses),
                 35.0, 'selected smoke destination despite broad smoke and lost fire')
        _assert_arrival_before_reacquire(mission, target, after=planned_at)
        with fcu.lock:
            arrival = next(stamp for stamp, text in fcu.statuses
                           if stamp > planned_at and text == arrived)
            during_leg = [text for stamp, text in fcu.statuses if planned_at < stamp <= arrival]
        assert not any('movement interrupted' in text for text in during_leg)
        assert not any(text in {'Verify/CENTER_FIRE: started', 'Verify/SEARCH_FIRE: started'}
                       for text in during_leg)
        assert not sensors.inspection_snapshot()
        assert mission.executive._verify.had_fire_confirmation()
        _assert_terrain_planning(mission)
    finally:
        release.set()
        cancel_goal(goal)
    _assert_airborne_safety(mission)


def test_high_smoke_with_small_jitter_stays_forward_and_level_through_a_short_miss(
    mission, monitoring_sensors,
):
    sensors, fcu = monitoring_sensors, mission.fcu
    misses = _occlude_smoke_when_a_leg_stops(mission, sensors, duration=2.0)
    corrections = [0]

    def high_smoke():
        corrections[0] += 1
        return (0.01 if corrections[0] % 2 else -0.01, 0.65)

    with fcu.lock:
        origin = deepcopy(fcu.pose.position)
        fcu.inspection_terrain_z = origin.z - 7.5
    with sensors.lock:
        sensors.smoke_guidance = high_smoke
    goal = _start_smoke_approach(mission, sensors)
    result = goal.get_result_async()
    wait_for(lambda: bool(misses), 15.0, 'the first forward smoke leg to stop')
    _wait_for_smoke_grace(mission, after=misses[0][0])
    wait_for(lambda: len(fcu.target_requests) >= 2, 7.0,
             'fresh confirmed smoke to resume a forward leg without a recovery scan')
    with sensors.lock:
        returned_at = next(stamp for stamp in sensors.smoke_positive_times
                           if stamp >= misses[0][0] + 1.9)
    _assert_quiet_smoke_occlusion(mission, misses[0], returned_at, check_current=False)
    with fcu.lock:
        targets = deepcopy(fcu.target_requests[:2])
        assert fcu.smoke_plan_times[1] >= returned_at
        statuses = [text for _, text in fcu.statuses]
    first, second = (target.pose.position for target in targets)
    direction = (first.x - origin.x, first.y - origin.y)
    norm = math.hypot(*direction)
    assert norm > 0.05
    for start, finish in ((origin, first), (first, second)):
        dx, dy = finish.x - start.x, finish.y - start.y
        assert (dx * direction[0] + dy * direction[1]) / norm > 0.05
        assert abs(dx * direction[1] - dy * direction[0]) / norm < 0.03
        assert finish.z == pytest.approx(origin.z, abs=0.03)
    assert not any('centering smoke with' in text or 'starting bounded reacquisition' in text
                   for text in statuses)
    assert not result.done()
    requests = _assert_terrain_planning(mission)
    assert all(request.target.pose.position.z == pytest.approx(origin.z, abs=0.03)
               for request in requests)
    _assert_same_goal_phases(sensors, goal)
    cancel_goal(goal)
    _assert_airborne_safety(mission)


def test_fresh_smoke_resets_occlusion_grace_before_a_later_long_loss_permits_yaw(
    mission, monitoring_sensors,
):
    sensors, fcu = monitoring_sensors, mission.fcu
    misses = _occlude_smoke_when_a_leg_stops(mission, sensors, count=2)
    with fcu.lock:
        fcu.inspection_terrain_z = fcu.pose.position.z - 7.5
    goal = _start_smoke_approach(mission, sensors)
    wait_for(lambda: bool(misses), 15.0, 'the first stopped smoke viewpoint')
    _wait_for_smoke_grace(mission, after=misses[0][0])
    wait_for(lambda: time.monotonic() - misses[0][1] >= 2.0,
             3.0, 'a short initial smoke loss')
    _assert_quiet_smoke_occlusion(mission, misses[0], time.monotonic())
    with sensors.lock:
        sensors.smoke_suppressed_until = 0.0
    wait_for(lambda: len(misses) == 2, 15.0,
             'new smoke observations to resume and stop the next forward leg')
    second = misses[1]
    _wait_for_smoke_grace(mission, after=second[0])
    wait_for(lambda: time.monotonic() - second[1] >= 3.0,
             4.0, 'the renewed grace period to outlast the original deadline')
    _assert_quiet_smoke_occlusion(mission, second, time.monotonic())
    with fcu.lock:
        assert not any('starting bounded reacquisition' in text for _, text in fcu.statuses)
    wait_for(
        lambda: any('starting bounded reacquisition' in text for _, text in fcu.statuses),
        4.0, 'five seconds of continuous smoke loss to permit bounded reacquisition',
    )
    with fcu.lock:
        scan_after = next(stamp for stamp, text in fcu.statuses
                          if 'starting bounded reacquisition' in text)
    assert scan_after - second[1] >= 4.8
    wait_for(
        lambda: abs(angular_distance(yaw_from_quaternion(fcu.pose.orientation),
                                     yaw_from_quaternion(second[2].orientation))) > 0.05,
        6.0, 'the permitted recovery yaw to begin after sustained smoke loss',
    )
    assert not goal.get_result_async().done()
    with fcu.lock:
        assert len(fcu.target_requests) == second[3]
        assert fcu.pose.position.z == pytest.approx(second[2].position.z, abs=0.03)
    cancel_goal(goal)
    _assert_airborne_safety(mission)


@pytest.mark.parametrize('interrupt', ['fire', 'cancel'])
def test_smoke_occlusion_hold_keeps_fire_and_cancellation_priority(
    mission, monitoring_sensors, interrupt,
):
    sensors, fcu = monitoring_sensors, mission.fcu
    misses = _occlude_smoke_when_a_leg_stops(mission, sensors)
    with fcu.lock:
        fcu.inspection_terrain_z = fcu.pose.position.z - 7.5
    goal = _start_smoke_approach(mission, sensors)
    wait_for(lambda: bool(misses), 15.0, 'the smoke leg to stop before interruption')
    _wait_for_smoke_grace(mission, after=misses[0][0])
    interrupted_at = time.monotonic()
    if interrupt == 'cancel':
        result = cancel_goal(goal)
        assert result.status == GoalStatus.STATUS_CANCELED
        assert time.monotonic() - interrupted_at < 3.0
    else:
        with sensors.lock:
            sensors.source = 'fire'
        wait_for(lambda: bool(sensors.inspection_snapshot()), 4.0,
                 'visible fire to interrupt the smoke grace hold immediately')
        assert sensors.inspection_snapshot()[0].fire_found
        with fcu.lock:
            assert len(fcu.target_requests) == misses[0][3]
        assert not goal.get_result_async().done()
        cancel_goal(goal)
    _assert_same_goal_phases(sensors, goal)
    _assert_airborne_safety(mission)


@pytest.mark.parametrize('fire_distance', [None, 1.0, 200.0])
def test_visible_fire_at_measured_wall_reports_without_fire_localization_or_translation(
    mission, monitoring_sensors, fire_distance,
):
    """A measured wall, not buried-fire coordinates, permits the held report."""
    sensors, fcu = monitoring_sensors, mission.fcu
    with sensors.lock:
        sensors.trusted_source = fire_distance is not None
    with fcu.lock:
        initial = deepcopy(fcu.pose)
        if fire_distance is not None:
            yaw = yaw_from_quaternion(initial.orientation)
            fcu.inspection_target = Point(
                x=initial.position.x + fire_distance * math.cos(yaw),
                y=initial.position.y + fire_distance * math.sin(yaw), z=initial.position.z)
            # The map sees the nearer smoke wall, independently of the fire.
            # A surface response outside the requested ray limit is invalid RPC metadata.
            fcu.inspection_obstacle = Point(
                x=initial.position.x + 5.0 * math.cos(yaw),
                y=initial.position.y + 5.0 * math.sin(yaw), z=initial.position.z)
    goal = _start_visible_fire(mission, sensors)
    wait_for(lambda: bool(sensors.inspection_snapshot()), 12.0,
             'visible fire report independent of its distance')
    with sensors.lock:
        sensors.source = None
    report = sensors.inspection_snapshot()[0]
    assert report.fire_found and report.status == 'fire_found'
    assert report.source_count == 2 and report.track_id == 42
    assert report.observer_global_valid
    assert report.fire_location_valid == (fire_distance is not None)
    if fire_distance is None:
        assert math.isnan(report.fire_local_position.point.x)
    with fcu.lock:
        assert not fcu.target_requests and not fcu.inspection_planning_requests
        assert any(text == 'Verify/CENTER_FIRE: started' for _, text in fcu.statuses)
        verify_at = next(stamp for stamp, text in fcu.statuses if text == 'Verify: entered')
        assert not any(verify_at < stamp < sensors.inspection_report_times[0]
                       and text == 'Exploration: started' for stamp, text in fcu.statuses)
    wait_for(lambda: _states(sensors)[:3] == ['Exploration', 'Verify', 'Exploration'],
             8.0, 'successful fire report to resume the same mission goal')
    assert not goal.get_result_async().done()
    _assert_same_goal_phases(sensors, goal)
    cancel_goal(goal)
    _assert_airborne_safety(mission)


def test_smoke_then_fire_is_approached_before_recording_the_same_mission_goal(
    mission, monitoring_sensors,
):
    sensors, fcu = monitoring_sensors, mission.fcu
    for settings in (mission.executive.config, mission.executive._verify.config):
        settings['verify_fire_approach_max_steps'] = 1
    goal = _start_airborne_exploration(mission)
    with fcu.lock:
        fcu.inspection_terrain_z = fcu.pose.position.z - 7.5
    _complete_initial_smoke_approach(mission, sensors, coverage=0.1)
    _use_fixed_smoke_wall(fcu, mission.executive.config['verify_smoke_standoff'] + 0.8)
    with fcu.lock:
        before_fire = deepcopy(fcu.pose.position)
        fire_point = deepcopy(fcu.inspection_target)
        plans_before_fire = len(fcu.inspection_planning_requests)
    with sensors.lock:
        sensors.source = 'smoke_fire'
    wait_for(lambda: bool(sensors.inspection_snapshot()), 30.0,
             'confirmed fire to receive a guarded closer viewpoint before its report')
    with sensors.lock:
        sensors.source = None
    report = sensors.inspection_snapshot()[0]
    assert report.fire_found and report.fire_location_valid
    recorded = report.observer_local_pose.pose.position
    assert math.dist(_point_position(recorded), _point_position(before_fire)) > 0.05
    assert math.dist(_point_position(recorded), _point_position(fire_point)) < math.dist(
        _point_position(before_fire), _point_position(fire_point))
    assert len(_assert_terrain_planning(mission)) > plans_before_fire
    wait_for(lambda: any('movement completed' in text for _, text in fcu.statuses),
             3.0, 'completed fire approach status delivery')
    _assert_same_goal_phases(sensors, goal)
    assert not goal.get_result_async().done()
    cancel_goal(goal)
    _assert_airborne_safety(mission)


def test_blocked_fire_approach_holds_without_reporting_success(
    mission, monitoring_sensors,
):
    sensors, fcu = monitoring_sensors, mission.fcu
    for settings in (mission.executive.config, mission.executive._verify.config):
        settings['verify_fire_approach_max_steps'] = 1
        settings['verify_navigation_candidate_limit'] = 1
    with fcu.lock:
        origin = deepcopy(fcu.pose.position)
        fcu.inspection_terrain_z = origin.z - 7.5
    with sensors.lock:
        sensors.trusted_source = False
        sensors.smoke_wall_range = mission.executive.config['verify_smoke_standoff'] + 10.0

    def occupied(request, response):
        with fcu.lock:
            fcu.inspection_planning_requests.append(deepcopy(request))
        response.success = False
        response.error_code = response.GOAL_INVALID
        response.message = 'simulated occupied approach corridor'
        return response

    fcu.inspection_planning_service.callback = occupied
    goal = _start_visible_fire(mission, sensors)
    wait_for(lambda: any('fire approach blocked' in text for _, text in fcu.statuses),
             15.0, 'blocked fire approach to withhold successful inspection')
    result = mission_result(goal)
    with sensors.lock:
        sensors.source = None
    assert result.status == GoalStatus.STATUS_ABORTED
    assert not result.result.success
    assert not sensors.inspection_snapshot()
    with fcu.lock:
        assert fcu.inspection_planning_requests and not fcu.target_requests
        assert math.dist(_point_position(fcu.pose.position), _point_position(origin)) < 0.03
        assert any('fire approach blocked' in text for _, text in fcu.statuses)
    _assert_same_goal_phases(sensors, goal)
    _assert_airborne_safety(mission)


@pytest.mark.parametrize('vertical_direction', [-1.0, 1.0])
def test_visible_fire_adjusts_altitude_at_twenty_metre_standoff(
    mission, monitoring_sensors, vertical_direction,
):
    sensors, fcu = monitoring_sensors, mission.fcu
    for config in (mission.executive.config, mission.executive._verify.config):
        config.update(verify_fire_approach_max_steps=3, mode_timeout=5.0)
    with fcu.lock:
        # Keep both the upward and downward correction above the terrain floor.
        fcu.inspection_terrain_z = fcu.pose.position.z - 7.0
        initial = deepcopy(fcu.pose)
    desired_z = initial.position.z + vertical_direction * 0.5

    def guidance():
        with fcu.lock:
            pose = deepcopy(fcu.pose)
        error = desired_z - pose.position.z
        return 0.0, math.copysign(0.2, error) if abs(error) > 0.03 else 0.0

    with sensors.lock:
        sensors.trusted_source = False
        sensors.fire_guidance = guidance
    goal = _start_visible_fire(mission, sensors)
    wait_for(lambda: bool(sensors.inspection_snapshot()), 35.0,
             'bounded altitude correction at the measured wall')
    with sensors.lock:
        sensors.source = None
    report = sensors.inspection_snapshot()[0]
    assert report.fire_found and not report.fire_location_valid
    assert mission.executive.config['verify_smoke_standoff'] == 20.0
    assert report.observer_local_pose.pose.position.z == pytest.approx(desired_z, abs=0.03)
    with fcu.lock:
        requests = deepcopy(fcu.inspection_planning_requests)
        assert requests
        assert all(request.terrain_envelope and request.minimum_agl >= 5.0
                   for request in requests)
        assert any(request.target.pose.position.z == pytest.approx(desired_z, abs=0.03)
                   for request in requests)
        assert all(math.hypot(request.target.pose.position.x - initial.position.x,
                              request.target.pose.position.y - initial.position.y) < 0.03
                   for request in requests)
        assert any(request.smoke_wall_width == 3.0 for request in fcu.geometry_requests)
    cancel_goal(goal)
    _assert_airborne_safety(mission)


def test_visible_fire_survives_blocked_vertical_centering_and_reports(
    mission, monitoring_sensors,
):
    sensors, fcu = monitoring_sensors, mission.fcu
    for config in (mission.executive.config, mission.executive._verify.config):
        config['verify_fire_approach_max_steps'] = 3
    rejected = []

    def reject_vertical(request, response):
        rejected.append(deepcopy(request))
        response.success = False
        response.error_code = response.GOAL_INVALID
        response.message = 'simulated blocked vertical corridor'
        return response

    fcu.inspection_planning_service.callback = reject_vertical
    with fcu.lock:
        fcu.inspection_terrain_z = fcu.pose.position.z - 7.0
    with sensors.lock:
        sensors.trusted_source = False
        sensors.fire_guidance = lambda: (0.0, -0.2)
    goal = _start_visible_fire(mission, sensors)
    wait_for(lambda: bool(sensors.inspection_snapshot()), 20.0,
             'still-visible off-center fire to remain reportable')
    with sensors.lock:
        sensors.source = None
    assert rejected and sensors.inspection_snapshot()[0].fire_found
    assert not sensors.inspection_snapshot()[0].fire_location_valid
    with fcu.lock:
        verify_at = next(stamp for stamp, text in fcu.statuses if text == 'Verify: entered')
        assert not any(verify_at < stamp < sensors.inspection_report_times[0]
                       and text == 'Exploration: started' for stamp, text in fcu.statuses)
    _assert_same_goal_phases(sensors, goal)
    cancel_goal(goal)
    _assert_airborne_safety(mission)


@pytest.mark.parametrize('vertical_direction', [-1.0, 1.0])
def test_fire_approach_combines_forward_motion_with_bounded_altitude_guidance(
    mission, monitoring_sensors, vertical_direction,
):
    sensors, fcu = monitoring_sensors, mission.fcu
    for config in (mission.executive.config, mission.executive._verify.config):
        config['verify_fire_approach_max_steps'] = 3
    with fcu.lock:
        initial = deepcopy(fcu.pose)
        fcu.inspection_terrain_z = initial.position.z - 7.0
    desired_z = initial.position.z + vertical_direction * 0.5
    standoff = mission.executive.config['verify_smoke_standoff']
    wall = _use_fixed_smoke_wall(fcu, standoff + 0.8)

    def guidance():
        with fcu.lock:
            error = desired_z - fcu.pose.position.z
        return 0.0, math.copysign(0.2, error) if abs(error) > 0.03 else 0.0

    with sensors.lock:
        sensors.trusted_source = False
        sensors.fire_guidance = guidance
    goal = _start_visible_fire(mission, sensors)
    wait_for(lambda: bool(sensors.inspection_snapshot()), 30.0,
             'forward fire approach with a bounded vertical correction')
    with sensors.lock:
        sensors.source = None
    report = sensors.inspection_snapshot()[0]
    assert report.fire_found
    position = report.observer_local_pose.pose.position
    assert position.z == pytest.approx(desired_z, abs=0.03)
    assert math.hypot(position.x - initial.position.x, position.y - initial.position.y) > 0.3
    point, axis = wall[0]
    distance = sum((getattr(point, a) - getattr(position, a)) * getattr(axis, a) for a in 'xyz')
    assert standoff <= distance <= standoff + 0.7
    requests = _assert_terrain_planning(mission)
    assert any(request.target.pose.position.z == pytest.approx(desired_z, abs=0.03)
               and math.hypot(request.target.pose.position.x - initial.position.x,
                              request.target.pose.position.y - initial.position.y) > 0.3
               for request in requests)
    cancel_goal(goal)
    _assert_airborne_safety(mission)


def test_fire_after_failed_clearance_recovery_is_kept_in_same_verify(
    mission, monitoring_sensors,
):
    sensors, fcu = monitoring_sensors, mission.fcu
    rejected = []

    def no_escape(request, response):
        rejected.append(time.monotonic())
        response.success = False
        response.error_code = getattr(response, 'NO_CLEARANCE_ESCAPE', 6)
        response.message = 'simulated bounded escape search exhausted'
        return response

    fcu.inspection_planning_service.callback = no_escape
    goal = _start_airborne_exploration(mission)
    with sensors.lock:
        sensors.smoke_boundary_range = mission.executive.config['verify_smoke_standoff'] + 10.0
        sensors.trusted_source = False
        sensors.source = 'smoke'
    wait_for(lambda: bool(rejected), 8.0, 'failed clearance recovery')
    with sensors.lock:
        sensors.source = 'smoke_fire'
    wait_for(lambda: bool(sensors.inspection_snapshot()), 12.0,
             'visible fire retained after rejected movement')
    with sensors.lock:
        sensors.source = None
    assert sensors.inspection_snapshot()[0].fire_found
    with fcu.lock:
        started = next(stamp for stamp, text in fcu.statuses if text == 'Verify: entered')
        reported = sensors.inspection_report_times[0]
        assert not any(started < stamp < reported and text == 'Exploration: started'
                       for stamp, text in fcu.statuses)
    assert not goal.get_result_async().done()
    _assert_same_goal_phases(sensors, goal)
    cancel_goal(goal)
    _assert_airborne_safety(mission)


def test_fire_post_motion_miss_requires_fresh_guidance_before_reporting(
    mission, monitoring_sensors,
):
    sensors = monitoring_sensors
    for config in (mission.executive.config, mission.executive._verify.config):
        config['verify_reacquire_observation_timeout'] = 2.0
    with sensors.lock:
        sensors.trusted_source = False
        sensors.fire_guidance = lambda: (0.2, 0.0)
    original = mission.executive._verify_rotate
    stopped = []

    def move_then_miss(*args, **kwargs):
        moved = original(*args, **kwargs)
        if moved is not False and not stopped:
            with sensors.lock:
                sensors.fire_suppressed_until = math.inf
            wait_for(lambda: mission.executive._verify.fresh_fire() is None,
                     2.0, 'negative frame acquired after vertical centering stopped')
            stopped.append(time.monotonic())
            with sensors.lock:
                sensors.fire_suppressed_until = time.monotonic() + 0.2
        return moved

    mission.executive._verify_rotate = move_then_miss
    goal = _start_visible_fire(mission, sensors)
    wait_for(lambda: bool(sensors.inspection_snapshot()), 20.0,
             'a fresh post-stop detection to complete the fire report')
    with sensors.lock:
        sensors.source = None
        reported = sensors.inspection_report_times[0]
    assert stopped and reported - stopped[0] >= 0.2
    assert sensors.inspection_snapshot()[0].fire_found
    _assert_same_goal_phases(sensors, goal)
    cancel_goal(goal)
    _assert_airborne_safety(mission)


@pytest.mark.parametrize('initial_approach', [False, True])
def test_observed_smoke_boundary_stops_advance_and_bounds_negative_inspection(
    mission, monitoring_sensors, initial_approach,
):
    sensors, fcu = monitoring_sensors, mission.fcu
    # Scale the active viewpoint parameter for this simulated-flight scenario.
    # verify_inspection_lateral_step does not control these CENTER_FIRE views.
    for config in (mission.executive.config, mission.executive._verify.config):
        config['verify_viewpoint_lateral_step'] = 0.4
    goal = _start_airborne_exploration(mission)
    if initial_approach:
        _complete_initial_smoke_approach(mission, sensors)
    with sensors.lock:
        sensors.smoke_boundary_range = 8.0
        sensors.smoke_coverage = 0.3
        sensors.source = 'smoke'
    wait_for(lambda: bool(sensors.inspection_snapshot()), 35.0,
             'three no-fire observation windows at the observed smoke boundary')
    with sensors.lock:
        sensors.source = None
    report = sensors.inspection_snapshot()[0]
    assert not report.fire_found and report.status == 'fire_not_found'
    assert report.inspection_attempts == 3
    assert not report.fire_location_valid and not report.smoke_location_valid
    viewpoints = _assert_terrain_planning(mission)[-2:]
    assert len(viewpoints) == 2
    first, second = (request.target.pose.position for request in viewpoints)
    assert math.hypot(second.x - first.x, second.y - first.y) == pytest.approx(0.8)
    with fcu.lock:
        assert any(text == 'Verify/CENTER_FIRE: started' for _, text in fcu.statuses)
    _assert_same_goal_phases(sensors, goal)
    cancel_goal(goal)
    _assert_airborne_safety(mission)


@pytest.mark.parametrize('interrupt', ['cancel', 'Abort', 'battery'])
def test_verify_motion_accepts_cancel_abort_and_battery_alert(
    mission, monitoring_sensors, interrupt,
):
    sensors = monitoring_sensors
    goal = _start_smoke_approach(mission, sensors)
    wait_for(lambda: mission.fcu.mode_requests.count('OFFBOARD') >= 2
             and received_state(mission, mode='OFFBOARD'),
             8.0, 'Verify OFFBOARD before interrupt')
    if interrupt == 'cancel':
        result = cancel_goal(goal)
        assert 'AUTO.LOITER confirmed' in result.result.message
    else:
        if interrupt == 'Abort':
            mission.fcu.mission_state_publisher.publish(String(data='Abort'))
        else:
            sensors.battery_publisher.publish(sensors.battery_alert(mission.fcu))
        _assert_aborted_and_loitering(mission, sensors, mission_result(goal), verified=True)
    _assert_same_goal_phases(sensors, goal)
    _assert_airborne_safety(mission)
    with sensors.lock:
        sensors.source = None
    next_goal = send_goal(mission, skip_takeoff=True)
    assert next_goal.accepted
    cancel_goal(next_goal)


def test_fire_during_takeoff_waits_for_climb_then_reports_without_approaching(
    mission, monitoring_sensors,
):
    sensors, fcu = monitoring_sensors, mission.fcu
    mission.executive.config['takeoff_timeout'] = 8.0
    with fcu.lock:
        fcu.climb_enabled = False
        takeoff_z = fcu.takeoff_z
    goal = send_goal(mission)
    assert goal.accepted
    wait_for(lambda: received_state(mission, mode='AUTO.TAKEOFF', armed=True),
             6.0, 'armed takeoff before detection')
    with sensors.lock:
        sensors.source = 'smoke_fire'
    wait_for(lambda: bool(sensors.requests), 4.0, 'Verify request during takeoff')
    assert _states(sensors) == ['Exploration']
    assert not sensors.inspection_snapshot() and not fcu.target_requests
    with fcu.lock:
        fcu.climb_enabled = True
    wait_for(lambda: bool(sensors.inspection_snapshot()), 15.0,
             'takeoff completion then visible-fire report')
    with sensors.lock:
        sensors.source = None
    report = sensors.inspection_snapshot()[0]
    assert report.fire_found
    assert report.observer_local_pose.pose.position.z >= (
        takeoff_z - mission.executive.config['takeoff_altitude_tolerance'])
    with fcu.lock:
        assert not fcu.target_requests
        assert fcu.arming_requests == [(True, 'AUTO.TAKEOFF')]
        assert fcu.mode_requests.count('AUTO.TAKEOFF') == 1
        assert not {'AUTO.RTL', 'AUTO.LAND'} & set(fcu.mode_requests)
        assert not fcu.offboard_losses
    _assert_same_goal_phases(sensors, goal)
    cancel_goal(goal)


@pytest.mark.parametrize('available', [True, False])
def test_battery_alert_aborts_and_old_alert_cannot_abort_a_new_goal(
    mission, monitoring_sensors, available,
):
    sensors = monitoring_sensors
    goal = _start_airborne_exploration(mission)
    if available:
        wait_for(lambda: mission.fcu.mode_requests.count('OFFBOARD') >= 2
                 and received_state(mission, mode='OFFBOARD'),
                 15.0, 'random-path OFFBOARD before learned battery alert')
    old_alert = sensors.battery_alert(mission.fcu, available=available)
    sensors.battery_publisher.publish(old_alert)
    _assert_aborted_and_loitering(mission, sensors, mission_result(goal))
    next_goal = send_goal(mission, skip_takeoff=True)
    assert next_goal.accepted
    sensors.battery_publisher.publish(old_alert)
    result = next_goal.get_result_async()
    wait_for(lambda: received_state(mission, mode='OFFBOARD'),
             5.0, 'restarted Exploration')
    assert not result.done()
    sensors.battery_publisher.publish(sensors.battery_alert(mission.fcu, available=available))
    wait_for(result.done, 5.0, 'fresh battery alert abort')
    _assert_aborted_and_loitering(mission, sensors, result.result())


def test_nonalert_battery_estimate_does_not_interrupt_exploration(
    mission, monitoring_sensors,
):
    sensors = monitoring_sensors
    goal = _start_airborne_exploration(mission)
    message = sensors.battery_alert(mission.fcu)
    message.return_alert = False
    sensors.battery_publisher.publish(message)
    result = goal.get_result_async()
    time.sleep(0.2)
    assert not result.done()
    cancel_goal(goal)


def test_center_fire_climbing_move_survives_detector_flicker_and_reconfirms_at_stop(
    mission, monitoring_sensors,
):
    sensors, fcu = monitoring_sensors, mission.fcu
    for settings in (mission.executive.config, mission.executive._verify.config):
        settings.update(verify_fire_approach_max_steps=1,
                        verify_reacquire_observation_timeout=10.0,
                        verify_motion_speed=0.5)
    with fcu.lock:
        initial = deepcopy(fcu.pose.position)
        fcu.inspection_terrain_z = initial.z - 4.5
    original = fcu.plan_inspection_path
    selected = []

    def plan_then_flicker(request, response):
        response = original(request, response)
        if response.success and not selected:
            selected.append(deepcopy(request.target.pose.position))
            with sensors.lock:
                sensors.fire_suppressed_until = time.monotonic() + 12.0
        return response

    fcu.inspection_planning_service.callback = plan_then_flicker
    _use_fixed_smoke_wall(fcu, mission.executive.config['verify_smoke_standoff'] + 0.8)
    goal = _start_visible_fire(mission, sensors)
    # The observer must allow the configured movement budget plus preparation
    # and post-stop confirmation; its deadline must not cancel a valid climb.
    config = mission.executive.config
    observation_deadline = (config['verify_fire_leg_timeout']
                            + config['verify_reacquire_observation_timeout'] + 15.0)
    wait_for(lambda: bool(sensors.inspection_snapshot()), observation_deadline,
             'committed climb through fire loss followed by a fresh stopped observation')
    with sensors.lock:
        sensors.source = None
    report = sensors.inspection_snapshot()[0]
    assert report.fire_found and selected
    assert report.observer_local_pose.pose.position.z > initial.z + 2.0
    with fcu.lock:
        messages = [text for _, text in fcu.statuses]
        assert any('movement completed' in text for text in messages)
        started_at = next(stamp for stamp, text in fcu.statuses
                          if 'starting committed short movement' in text)
        completed_at = next(stamp for stamp, text in fcu.statuses
                            if 'movement completed' in text)
        assert completed_at - started_at <= (config['verify_fire_leg_timeout']
                                             + config['mode_timeout'])
        assert not any('Tracked fire observation lost' in text for text in messages)
        assert any(request.smoke_wall_width == 3.0 for request in fcu.geometry_requests)
        assert not any(request.minimum_radius > 0.0
                       for request in fcu.inspection_planning_requests)
    cancel_goal(goal)
    _assert_airborne_safety(mission)


def test_center_fire_arrives_at_smoke_wall_boundary_with_fresh_fire(mission, monitoring_sensors):
    sensors, fcu = monitoring_sensors, mission.fcu
    for settings in (mission.executive.config, mission.executive._verify.config):
        settings.update(verify_fire_approach_max_steps=8, mode_timeout=5.0)
    standoff = mission.executive.config['verify_smoke_standoff']
    with fcu.lock:
        # Isolate wall approach from the separate terrain-climbing regression.
        fcu.inspection_terrain_z = fcu.pose.position.z - 7.5
    wall = _use_fixed_smoke_wall(fcu, standoff + 1.5)
    goal = _start_visible_fire(mission, sensors)
    wait_for(lambda: bool(sensors.inspection_snapshot()), 60.0,
             'safe smoke-wall arrival with confirmed fire still visible')
    with sensors.lock:
        sensors.source = None
    report = sensors.inspection_snapshot()[0]
    assert report.fire_found
    point, axis = wall[0]
    position = report.observer_local_pose.pose.position
    distance = sum((getattr(point, a) - getattr(position, a)) * getattr(axis, a) for a in 'xyz')
    assert standoff <= distance <= standoff + 1.0
    with fcu.lock:
        assert any('smoke-wall standoff reached with fresh fire visible' in text
                   for _, text in fcu.statuses)
    cancel_goal(goal)
    _assert_airborne_safety(mission)


def test_smoke_final_leg_is_clipped_to_the_twenty_metre_measured_boundary(
    mission, monitoring_sensors,
):
    sensors, fcu = monitoring_sensors, mission.fcu
    for config in (mission.executive.config, mission.executive._verify.config):
        config.update(verify_navigation_step=3.0, position_tolerance=0.3, mode_timeout=5.0)
    goal = _start_airborne_exploration(mission)
    standoff = mission.executive.config['verify_smoke_standoff']
    with fcu.lock:
        initial = deepcopy(fcu.pose)
        bearing = yaw_from_quaternion(initial.orientation)
        wall_point = Point(x=initial.position.x + (standoff + 0.8) * math.cos(bearing),
                           y=initial.position.y + (standoff + 0.8) * math.sin(bearing),
                           z=initial.position.z)
        fcu.inspection_target = deepcopy(wall_point)
        fcu.inspection_obstacle = deepcopy(wall_point)
        fcu.inspection_terrain_z = initial.position.z - 7.5

    def smoke_guidance():
        with fcu.lock:
            pose = deepcopy(fcu.pose)
        # The fixed observed wall gets closer as the simulated aircraft moves.
        sensors.smoke_boundary_range = math.dist(
            _point_position(pose.position), _point_position(wall_point))
        yaw = math.atan2(wall_point.y - pose.position.y, wall_point.x - pose.position.x)
        return angular_distance(yaw, yaw_from_quaternion(pose.orientation)), 0.0

    with sensors.lock:
        sensors.smoke_boundary_range = standoff + 0.8
        sensors.smoke_guidance = smoke_guidance
        sensors.source = 'smoke'
    arrived = 'Verify: approach destination reached; reacquiring smoke or fire'
    wait_for(lambda: any(text == arrived for _, text in fcu.statuses), 35.0,
             'short final smoke leg to finish outside its measured boundary')
    with sensors.lock:
        sensors.source = None
    with fcu.lock:
        assert fcu.target_requests
        target = deepcopy(fcu.target_requests[0].pose)
    advance = ((target.position.x - initial.position.x) * math.cos(bearing)
               + (target.position.y - initial.position.y) * math.sin(bearing))
    assert advance == pytest.approx(0.4, abs=0.03)
    assert target.position.z == pytest.approx(initial.position.z, abs=0.03)
    assert math.dist(_point_position(target.position), _point_position(wall_point)) >= standoff
    _assert_arrival_before_reacquire(mission, target)
    _assert_terrain_planning(mission)
    cancel_goal(goal)
    _assert_airborne_safety(mission)
