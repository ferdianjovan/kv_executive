"""Inspection geometry, guarded path requests and separate target geolocation."""

from copy import deepcopy
import math
import threading
from types import SimpleNamespace

from geometry_msgs.msg import (
    Point, PointStamped, Pose, PoseStamped, Quaternion, Transform, Vector3, Vector3Stamped,
)
import kv_executive.inspection_navigation as inspection
from kv_executive.inspection_navigation import enu_offset_to_fix, InspectionNavigationMixin
from kv_executive.mission_geometry import quaternion_from_yaw, yaw_from_quaternion
from nav_msgs.msg import Odometry, Path
import pytest
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Header


def header(now=100.0, frame='px4_odom'):
    value = Header(frame_id=frame)
    value.stamp.sec = int(now)
    value.stamp.nanosec = round((now % 1.0) * 1e9)
    return value


def target_geometry(now=100.0, x=30.0, y=0.0, z=5.0):
    return {
        'point': Point(x=float(x), y=float(y), z=float(z)),
        'planning_point': PointStamped(
            header=header(now, 'map'), point=Point(x=float(x), y=float(y), z=float(z))),
        'range': float(math.hypot(x, y, z - 20.0)), 'uncertainty': 0.2,
        'method': 'thermal_multiview_bearings', 'ground_z': None,
        'source_stamp': header(now).stamp, 'source': 'fire', 'target_valid': True,
    }


def smoke_boundary_geometry(now=100.0):
    return {
        'source': 'smoke', 'source_stamp': header(now).stamp,
        'map_header': header(now, 'map'), 'plume_candidate': True,
        'obstacle_point': Point(x=8.0, z=20.0), 'obstacle_range': 8.0,
        'obstacle_uncertainty': 0.2,
    }


def coarse_geometry(now=100.0, x=100.0, uncertainty=8.0):
    return {
        'point': None, 'planning_point': None, 'range': math.nan,
        'uncertainty': math.nan, 'target_valid': False, 'ground_z': None,
        'source': 'fire', 'source_stamp': header(now).stamp,
        'method': 'thermal_multiview_bearings',
        'estimate_point': Point(x=x, z=2.0), 'estimate_uncertainty': uncertainty,
        'localization_views': 3, 'localization_baseline': 10.0,
        'localization_parallax': math.radians(5.0),
        'localization_association': 'stable_thermal_track',
    }


class Harness(InspectionNavigationMixin):
    def __init__(self):
        self.now = 100.0
        self.events = []
        self.config = {
            'verify_planning_frame': 'map', 'local_frame': 'px4_odom',
            'planning_timeout': 3.0, 'verify_geometry_timeout': 5.0,
            'verify_inspection_max_range': 40.0, 'verify_future_tolerance': 0.25,
            'verify_inspection_max_uncertainty': 2.0,
            'verify_observation_timeout': 2.0, 'global_position_timeout': 2.0,
            'telemetry_timeout': 2.0, 'verify_smoke_standoff': 10.0,
            'verify_inspection_altitude': 5.0, 'verify_inspection_lateral_step': 5.0,
            'verify_minimum_altitude': 2.0, 'verify_motion_speed': 0.5,
            'verify_yaw_tolerance': math.radians(5.0),
        }
        self._inspection_geometry_client = object()
        self._inspection_planning_client = object()
        self.ray_origin = (0.0, 0.0, 20.0)
        self.ray_target = (35.0, 2.0, 2.0)
        self.odometry = Odometry(header=header())
        self.odometry.pose.pose = Pose(
            position=Point(z=20.0), orientation=quaternion_from_yaw(0.0))
        self._data_lock = threading.Lock()
        self._site_sequence = 0
        self._global_position_received = self.now
        self._global_position = NavSatFix(
            header=header(frame='gps'), status=NavSatStatus(status=NavSatStatus.STATUS_FIX),
            latitude=51.5, longitude=-0.12, altitude=50.0)
        self.conversion = Transform()
        self.conversion.rotation.w = 1.0
        self.smoke = SimpleNamespace(
            header=header(frame='rgb_camera'), detected=True, status=3, CONFIRMED=3,
            background_ready=True, yaw_correction_rad=0.0, vertical_correction_rad=0.0,
            coverage=0.3, regions=[1, 2, 3, 4])
        self.fire = None
        self.fire_confirmed = False
        self._verify = SimpleNamespace(
            fresh_smoke=lambda: deepcopy(self.smoke), stable_fire=lambda: deepcopy(self.fire),
            had_fire_confirmation=lambda: self.fire_confirmed,
            all_quadrants=lambda smoke: (
                smoke.coverage >= 0.25 and {1, 2, 3, 4}.issubset(smoke.regions)),
        )
        self._inspection_report_publisher = SimpleNamespace(
            publish=lambda report: self.events.append(('report', deepcopy(report))))
        self.on_query = lambda request, response: None
        self.ground_valid = True
        self.ground_z = 2.0
        self.ground_uncertainty = 0.1
        self.corridor_safe = True
        self.smoke_wall_valid = False
        self.smoke_wall_nearest = 10.5

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=int(self.now * 1e9), to_msg=lambda: header(self.now).stamp))

    def _check_running(self):
        return None

    def _verify_control_pose(self):
        return SimpleNamespace(mode='AUTO.LOITER'), deepcopy(self.odometry.pose.pose)

    def _snapshot(self, require_armed=False):
        assert require_armed
        return None, deepcopy(self.odometry)

    def _path_frame_transform(self, frame):
        assert frame == 'map'
        return deepcopy(self.conversion)

    def _call(self, client, request, description, timeout=None):
        assert client is self._inspection_geometry_client
        self.events.append(('query', deepcopy(request)))
        fire = 'thermal' in request.origin.header.frame_id
        response = SimpleNamespace(
            success=True, error_code=0, message='known map', map_header=header(self.now, 'map'),
            surface_valid=True,
            surface=PointStamped(header=header(self.now, 'map'), point=Point(
                x=35.0 if fire else 30.0, y=2.0 if fire else 0.0, z=2.0 if fire else 5.0)),
            surface_range=35.0 if fire else 30.0, surface_uncertainty=0.2,
            plume_candidate=False, occupied_samples=3, free_samples=0,
            ground_valid=self.ground_valid,
            ground_surface=PointStamped(
                header=header(self.now, 'map'), point=Point(z=self.ground_z)),
            ground_uncertainty=self.ground_uncertainty, corridor_safe=self.corridor_safe,
            current_clearance_reached=False,
            smoke_wall_valid=self.smoke_wall_valid, smoke_wall_range=self.smoke_wall_nearest,
            smoke_wall_nearest_range=self.smoke_wall_nearest,
            smoke_wall_uncertainty=0.2, smoke_wall_samples=25,
        )
        ray_header = deepcopy(request.origin.header)
        ray_header.frame_id = 'map'
        response.ray_origin = PointStamped(
            header=ray_header, point=Point(**dict(zip('xyz', self.ray_origin))))
        direction = tuple(end - start for end, start in zip(self.ray_target, self.ray_origin))
        norm = math.hypot(*direction)
        response.ray_direction = Vector3Stamped(
            header=deepcopy(ray_header), vector=Vector3(
                **dict(zip('xyz', (value / norm for value in direction)))))
        self.on_query(request, response)
        return response

    def _report(self, text):
        self.events.append(('status', text))

    def _verify_rotate(self, yaw, guard=None):
        if guard:
            guard()
        self.events.append(('rotate', yaw))
        self.odometry.pose.pose.orientation = quaternion_from_yaw(yaw)

    def _verify_record_fire(self, message, bearing, require_centering=True, *, publish=True):
        result = SimpleNamespace(
            site_id=0, message=deepcopy(message), bearing=bearing,
            require_centering=require_centering)
        self.events.append(('legacy_prepare', deepcopy(message)))
        return self._verify_publish_fire_report(result) if publish else result

    def _verify_publish_fire_report(self, report):
        self._site_sequence += 1
        report.site_id = self._site_sequence
        self.events.append((
            'legacy_report', report.message, report.bearing, report.require_centering,
            self._site_sequence))
        return report

    def _verify_plan_to(self, target, **kwargs):
        self.events.append(('plan', deepcopy(target), kwargs))
        from kv_executive.mission_navigation import _inverse
        request = kwargs['request_builder'](
            PoseStamped(header=header(frame='map'), pose=deepcopy(target)),
            _inverse(self.conversion))
        self.events.append(('inspection_plan', request))
        path = Path(header=header(frame='px4_odom'), poses=[
            PoseStamped(pose=deepcopy(self.odometry.pose.pose)),
            PoseStamped(pose=deepcopy(target)),
        ])
        if not kwargs['path_validator'](path):
            return False
        self.odometry.pose.pose = deepcopy(target)
        return True


@pytest.fixture
def executive(monkeypatch):
    harness = Harness()
    monkeypatch.setattr(inspection.time, 'monotonic', lambda: harness.now)
    return harness


def test_source_stamped_camera_ray_is_not_replaced_with_current_drone_position(executive):
    executive.smoke.yaw_correction_rad = 0.2
    executive.smoke.vertical_correction_rad = -0.3
    executive.conversion.translation.x = 10.0

    geometry = executive._verify_target_geometry(executive.smoke)

    request = next(event[1] for event in executive.events if event[0] == 'query')
    assert request.origin.header == executive.smoke.header
    assert request.origin.point == Point()
    assert request.direction.header == executive.smoke.header
    norm = math.hypot(1.0, math.tan(0.2), math.tan(-0.3))
    assert request.direction.vector.x == pytest.approx(1.0 / norm)
    assert request.direction.vector.y == pytest.approx(math.tan(0.2) / norm)
    assert request.direction.vector.z == pytest.approx(math.tan(-0.3) / norm)
    assert geometry['point'] is None and not geometry['target_valid']
    assert geometry['obstacle_point'].x == 40.0
    assert geometry['obstacle_planning_point'].point.x == 30.0
    assert geometry['method'] == 'unavailable_semantic_source_range'


@pytest.mark.parametrize('unknown', ['stale_image', 'no_surface', 'stale_map', 'missing_map'])
def test_unknown_range_never_becomes_assumed_distance(executive, unknown):
    def unavailable(request, response):
        if unknown == 'no_surface':
            response.surface_valid = False
        elif unknown == 'stale_map':
            response.map_header = header(executive.now - 6.0, 'map')
        else:
            response.success = False
            response.error_code = 1
    if unknown == 'stale_image':
        executive.smoke.header = header(executive.now - 3.0, 'rgb_camera')
    else:
        executive.on_query = unavailable

    geometry = executive._verify_target_geometry(executive.smoke)

    assert geometry['point'] is None
    assert math.isnan(geometry['range'])
    assert (executive._verify_inspection_pose(geometry).position
            == executive.odometry.pose.pose.position)
    assert not any(event[0] == 'plan' for event in executive.events)


@pytest.mark.parametrize('code', [2, 3])
def test_invalid_geometry_requests_and_transform_errors_are_not_optional(executive, code):
    from kv_executive.mission_executive import MissionError

    def fail(request, response):
        response.success = False
        response.error_code = code
        response.message = 'bad transform or request'
    executive.on_query = fail

    with pytest.raises(MissionError, match='geometry failed'):
        executive._verify_target_geometry(executive.smoke)


@pytest.mark.parametrize('horizontal, vertical', [
    (math.nan, 0.0), (math.pi / 2, 0.0), (0.0, -math.pi / 2),
])
def test_invalid_pinhole_angles_do_not_issue_geometry_requests(executive, horizontal, vertical):
    executive.smoke.yaw_correction_rad = horizontal
    executive.smoke.vertical_correction_rad = vertical
    assert executive._verify_target_geometry(executive.smoke)['point'] is None
    assert not any(event[0] == 'query' for event in executive.events)


def test_coarse_surface_uncertainty_cannot_authorize_metric_inspection(executive):
    executive.on_query = lambda request, response: setattr(response, 'surface_uncertainty', 3.0)

    geometry = executive._verify_target_geometry(executive.smoke)

    assert geometry['point'] is None
    assert not geometry['target_valid']
    assert geometry['obstacle_uncertainty'] == 3.0
    cached = target_geometry()
    cached['uncertainty'] = 3.0
    assert (executive._verify_inspection_pose(cached).position
            == executive.odometry.pose.pose.position)
    # Fire coordinate uncertainty cannot authorize or prohibit a terrain-safe leg.
    target = deepcopy(executive.odometry.pose.pose)
    target.position.x += 1.0
    assert executive._verify_inspection_plan_to(target, cached)


@pytest.mark.parametrize('flickering', [False, True])
def test_near_tree_or_flickering_surface_never_becomes_the_smoke_target(executive, flickering):
    def obstacle(request, response):
        response.surface.point.x = 2.0
        response.surface_range = 2.0
        response.plume_candidate = flickering
    executive.on_query = obstacle

    geometry = executive._verify_target_geometry(executive.smoke)

    assert geometry['obstacle_range'] == 2.0
    assert geometry['point'] is None and not geometry['target_valid']
    assert (executive._verify_inspection_pose(geometry).position
            == executive.odometry.pose.pose.position)
    assert not executive._verify_source_at_standoff(geometry)


def test_fresh_history_associated_smoke_wall_records_an_explicit_proxy_position(executive):
    executive.smoke_wall_valid = True

    def observed_plume(request, response):
        response.surface.point = Point(x=8.0, z=20.0)
        response.surface_range = 8.0
        response.plume_candidate = True
        response.occupied_samples = 3
        response.free_samples = 1

    executive.on_query = observed_plume
    geometry = executive._verify_target_geometry(executive.smoke)
    report = executive._verify_report_inspection(geometry, attempts=3)
    assert report.smoke_location_valid
    assert report.smoke_location_method == 'lidar_smoke_plume_proxy'
    assert report.smoke_local_position.point == Point(x=8.0, z=20.0)
    assert report.smoke_local_position.point != report.observer_local_pose.pose.position
    assert not report.fire_found and not report.fire_location_valid


def test_localized_fire_standoff_is_enforced_in_native_request_and_flight_validation(executive):
    geometry = target_geometry(x=12.0, y=0.0, z=20.0)
    geometry['fire_keepout_radius'] = 10.0
    target = deepcopy(executive.odometry.pose.pose)
    target.position.x = 1.0
    assert executive._verify_recovery_plan_to(target, geometry)
    request = next(event[1] for event in executive.events if event[0] == 'inspection_plan')
    assert request.minimum_radius == 10.0
    assert request.keepout_center.point == geometry['planning_point'].point
    # A later continuation crossing the same source margin is rejected before
    # any simulated motion, even when the map corridor itself is free.
    before = deepcopy(executive.odometry.pose.pose)
    target.position.x = 4.0
    assert not executive._verify_recovery_plan_to(target, geometry)
    assert executive.odometry.pose.pose == before


def test_thermal_multiview_target_is_separate_from_a_nearer_obstacle(executive):
    def near_tree(request, response):
        response.surface.point = Point(x=2.0, y=0.0, z=20.0)
        response.surface_range = 2.0
    executive.on_query = near_tree
    _seed_localized_fire(executive)

    geometry = executive._verify_target_geometry(executive.fire, 'fire')

    assert geometry['target_valid']
    assert geometry['method'] == 'thermal_multiview_bearings'
    assert geometry['point'].x == pytest.approx(35.0)
    assert geometry['obstacle_point'].x == 2.0
    assert geometry['uncertainty'] > 0.2


def test_thermal_localization_remains_available_when_no_map_ray_hits(executive):
    executive.on_query = lambda request, response: setattr(response, 'surface_valid', False)
    _seed_localized_fire(executive)
    geometry = executive._verify_target_geometry(executive.fire, 'fire')
    assert geometry['target_valid'] and geometry['obstacle_point'] is None


def test_new_verification_episode_resets_previously_localized_source(executive):
    _seed_localized_fire(executive)
    executive._verify_reset_source_localization()
    geometry = executive._verify_target_geometry(executive.fire, 'fire')
    assert not geometry['target_valid'] and geometry['point'] is None


@pytest.mark.parametrize('failure', ['unknown', 'far', 'stale', 'uncertain'])
def test_smoke_boundary_requires_fresh_observed_wall_evidence(executive, failure):
    geometry = smoke_boundary_geometry()
    assert executive._verify_smoke_boundary_reached(geometry)
    if failure == 'unknown':
        geometry['plume_candidate'] = False
    elif failure == 'far':
        geometry['obstacle_point'].x = 30.0
    elif failure == 'stale':
        geometry['source_stamp'] = header(executive.now - 3.0).stamp
    else:
        geometry['obstacle_uncertainty'] = 3.0
    assert not executive._verify_smoke_boundary_reached(geometry)
    assert not executive._verify_source_at_standoff(target_geometry(x=1.0))


def test_standoff_climbs_above_measured_terrain_without_descending(executive):
    executive.ground_z = 25.0
    target = executive._verify_inspection_pose(target_geometry())
    assert target.position.z == pytest.approx(30.4)


def test_fire_coordinates_never_select_an_approach_endpoint(executive):
    geometry = target_geometry(x=30, y=0)
    target = executive._verify_inspection_pose(geometry)
    assert target.position == executive.odometry.pose.pose.position
    geometry['point'] = Point(x=-1000.0, y=500.0, z=-900.0)
    geometry['uncertainty'] = 1000.0
    assert executive._verify_inspection_pose(geometry) == target


def test_already_close_target_never_causes_an_automatic_retreat(executive):
    target = executive._verify_inspection_pose(target_geometry(x=8, y=0))
    assert target.position == executive.odometry.pose.pose.position


def test_ground_inspection_uses_measured_surface_plus_clearance_and_uncertainty(executive):
    geometry = target_geometry()

    target = executive._verify_inspection_pose(geometry, 'ground')

    assert target.position.z == pytest.approx(7.4)
    assert geometry['ground_z'] == 2.0
    assert geometry['ground_method'] == 'conservative_octomap_terrain_envelope'


def test_unknown_terrain_does_not_authorize_descent(executive):
    executive.ground_valid = False
    target = executive._verify_inspection_pose(target_geometry(), 'ground')
    assert target.position.z == 20.0
    assert any('ground height unavailable' in event[1] for event in executive.events
               if event[0] == 'status')


def test_directly_below_fire_can_be_inspected_without_undefined_yaw(executive):
    geometry = target_geometry(x=0, y=0, z=2)
    target = executive._verify_inspection_pose(geometry, 'ground')
    assert target.position.z == pytest.approx(7.4)
    assert yaw_from_quaternion(target.orientation) == 0.0


def test_in_place_plan_keeps_existing_yaw_for_target_directly_below(executive):
    executive.odometry.pose.pose.orientation = quaternion_from_yaw(1.2)
    geometry = target_geometry(x=0, y=0, z=2)
    target = deepcopy(executive.odometry.pose.pose)

    assert executive._verify_inspection_plan_to(target, geometry)

    assert yaw_from_quaternion(executive.odometry.pose.pose.orientation) == pytest.approx(1.2)
    actual_yaw = next(event[1] for event in executive.events if event[0] == 'rotate')
    assert actual_yaw == pytest.approx(1.2)


def test_lateral_viewpoints_use_smoke_bearing_without_a_fire_radius(executive):
    executive.odometry.pose.pose.position.x = 20.0
    origin = deepcopy(executive.odometry.pose.pose)
    geometry = target_geometry(x=30, y=0)
    left = executive._verify_inspection_pose(geometry, 'left', origin)
    executive.odometry.pose.pose = deepcopy(left)
    right = executive._verify_inspection_pose(geometry, 'right', origin)
    assert left.position == Point(x=20.0, y=5.0, z=20.0)
    assert right.position == Point(x=20.0, y=-5.0, z=20.0)
    assert yaw_from_quaternion(left.orientation) == 0.0
    assert yaw_from_quaternion(right.orientation) == 0.0


def test_inspection_requests_image_guidance_and_entire_terrain_corridor_validation(executive):
    geometry = target_geometry()
    target = deepcopy(executive.odometry.pose.pose)
    target.position.x += 1.0
    assert executive._verify_inspection_plan_to(target, geometry)
    event = next(event for event in executive.events if event[0] == 'plan')
    assert event[2]['allow_plan_failure'] is True
    assert 'look_at_point' not in event[2]
    assert callable(event[2]['heading_provider'])
    assert event[2]['planning_client'] is executive._inspection_planning_client
    planned = next(event[1] for event in executive.events if event[0] == 'inspection_plan')
    assert planned.minimum_radius == 0.0
    assert not planned.keepout_center.header.frame_id
    assert planned.minimum_agl == 5.0 and planned.allow_initial_climb
    assert planned.keepout_axis.vector == Vector3(z=1.0)
    request = next(event[1] for event in executive.events
                   if event[0] == 'query' and event[1].corridor.poses)
    assert request.corridor.header.frame_id == 'map'
    assert not request.keepout_center.header.frame_id
    assert request.minimum_radius == 0.0


@pytest.mark.parametrize('requested_agl', [None, 0.0, 2.0])
def test_terrain_inspection_keeps_the_floor_when_source_geometry_has_no_ground(
        executive, requested_agl):
    geometry = target_geometry()
    assert geometry['ground_z'] is None
    target = deepcopy(executive.odometry.pose.pose)
    target.position.x = 5.0

    assert executive._verify_inspection_plan_to(target, geometry, minimum_agl=requested_agl)

    planned = next(event[1] for event in executive.events if event[0] == 'inspection_plan')
    assert planned.minimum_agl == 5.0
    assert planned.terrain_envelope and planned.allow_terrain_escape
    corridor = next(event[1] for event in executive.events
                    if event[0] == 'query' and event[1].corridor.poses)
    assert corridor.minimum_agl == 5.0
    assert corridor.allow_terrain_escape


@pytest.mark.parametrize('operation', ['recovery', 'inspection'])
def test_terrain_escape_revalidation_keeps_its_origin_and_latches_clearance(executive, operation):
    executive.odometry.pose.pose.position.z = 5.0
    origin = deepcopy(executive.odometry.pose.pose.position)
    target = deepcopy(executive.odometry.pose.pose)
    target.position.x = 1.0
    target.position.z = 7.1
    phase = {'reached': False, 'new_terrain_below_floor': False}

    def clearance_evidence(request, response):
        response.current_clearance_reached = phase['reached']
        if request.terrain_escape_complete and phase['new_terrain_below_floor']:
            response.corridor_safe = False
            response.message = 'completed escape cannot restart below the clearance floor'

    executive.on_query = clearance_evidence
    if operation == 'recovery':
        assert executive._verify_recovery_plan_to(target)
    else:
        assert executive._verify_inspection_plan_to(target, target_geometry())
    plan = next(event for event in executive.events if event[0] == 'plan')
    validate = plan[2]['path_validator']
    first = next(event[1] for event in executive.events
                 if event[0] == 'query' and event[1].corridor.poses)
    assert first.escape_origin.header.frame_id == 'map'
    assert first.escape_origin.point == origin
    assert not first.terrain_escape_complete

    # OFFBOARD warmup and later validation use a fresh map alignment and a
    # later actual pose, but the escape anchor stays fixed in local coordinates.
    executive.conversion.translation.x = 10.0
    current = deepcopy(target)
    current.position.x = 1.5
    path = Path(header=header(), poses=[PoseStamped(pose=current), PoseStamped(pose=target)])
    phase['reached'] = True
    assert validate(path)
    measured = [event[1] for event in executive.events
                if event[0] == 'query' and event[1].corridor.poses][-1]
    assert measured.escape_origin.point == Point(x=-10.0, z=5.0)
    assert measured.corridor.poses[0].pose.position.x == -8.5
    assert not measured.terrain_escape_complete

    phase['reached'] = False
    phase['new_terrain_below_floor'] = True
    assert not validate(path)
    rejected = [event[1] for event in executive.events
                if event[0] == 'query' and event[1].corridor.poses][-1]
    assert rejected.escape_origin.point == measured.escape_origin.point
    assert rejected.terrain_escape_complete
    assert not validate(path)
    latest = [event[1] for event in executive.events
              if event[0] == 'query' and event[1].corridor.poses][-1]
    assert latest.terrain_escape_complete


def test_terrain_escape_revalidation_latches_current_clearance_even_if_future_path_is_blocked(
        executive):
    path = Path(header=header(), poses=[PoseStamped(pose=deepcopy(executive.odometry.pose.pose))])
    state = {'origin': deepcopy(executive.odometry.pose.pose.position), 'complete': False}

    def reached_but_blocked(request, response):
        response.current_clearance_reached = True
        response.corridor_safe = False

    executive.on_query = reached_but_blocked
    assert not executive._inspection_map_corridor(
        path, minimum_agl=5.0, allow_initial_climb=True, terrain_envelope=True,
        allow_terrain_escape=True, escape_state=state)
    assert state['complete']


def test_ground_and_corridor_queries_convert_px4_local_coordinates_to_map(executive):
    executive.conversion.translation.x = 10.0
    executive.conversion.translation.y = 20.0
    executive.odometry.pose.pose.position.x = 10.0
    executive.odometry.pose.pose.position.y = 20.0
    geometry = target_geometry(x=40, y=20)
    target = executive._verify_inspection_pose(geometry, 'ground')
    ground = next(event[1].ground_query for event in executive.events
                  if event[0] == 'query' and event[1].ground_query.header.frame_id)
    assert ground.header.frame_id == 'map'
    assert (ground.point.x, ground.point.y) == pytest.approx((0.0, 0.0))
    assert executive._verify_inspection_plan_to(target, geometry)
    request = next(event[1] for event in executive.events
                   if event[0] == 'query' and event[1].corridor.poses)
    start = request.corridor.poses[0].pose.position
    assert (start.x, start.y) == pytest.approx((0.0, 0.0))
    assert not request.keepout_center.header.frame_id
    assert request.minimum_radius == 0.0


def test_unsafe_inspection_corridor_declines_flight(executive):
    executive.corridor_safe = False
    geometry = target_geometry()
    target = executive._verify_inspection_pose(geometry)
    original = deepcopy(executive.odometry.pose.pose)
    assert executive._verify_inspection_plan_to(target, geometry) is False
    assert executive.odometry.pose.pose == original


def test_inspection_corridor_rejection_reports_native_failure_reason(executive):
    reason = 'Inspection corridor contains occupied or unknown body clearance'

    def blocked_corridor(request, response):
        if request.corridor.poses:
            response.corridor_safe = False
            response.message = reason

    executive.on_query = blocked_corridor
    geometry = target_geometry()
    target = executive._verify_inspection_pose(geometry)
    original = deepcopy(executive.odometry.pose.pose)

    assert executive._verify_inspection_plan_to(target, geometry) is False

    assert ('status', f'Verify: inspection corridor rejected: {reason}') in executive.events
    assert executive.odometry.pose.pose == original


def test_inspection_corridor_missing_map_reports_validation_unavailable(executive):
    from kv_executive.verification import VerificationMapUnavailable

    def missing_map(request, response):
        if request.corridor.poses:
            response.success = False
            response.error_code = 1
            response.message = 'No fresh OctoMap available'

    executive.on_query = missing_map
    geometry = target_geometry()
    target = executive._verify_inspection_pose(geometry)
    original = deepcopy(executive.odometry.pose.pose)

    with pytest.raises(VerificationMapUnavailable, match='No fresh OctoMap available'):
        executive._verify_inspection_plan_to(target, geometry)
    assert executive.odometry.pose.pose == original


def test_terrain_recovery_keeps_the_local_up_axis_when_map_alignment_has_pitch(executive):
    executive.conversion.rotation = Quaternion(
        y=math.sin(-0.01 / 2.0), w=math.cos(-0.01 / 2.0))
    target = deepcopy(executive.odometry.pose.pose)
    target.position.x += 1.0
    assert executive._verify_inspection_plan_to(target, target_geometry(), minimum_radius=10.2)
    request = next(event[1] for event in executive.events
                   if event[0] == 'query' and event[1].corridor.poses)
    assert request.minimum_radius == 0.0 and request.minimum_agl == 5.0
    assert not request.keepout_center.header.frame_id
    assert request.keepout_axis.vector.x != 0.0


def test_fire_keepout_radius_never_blocks_a_terrain_safe_lateral_view(executive):
    geometry = target_geometry(x=30.0, y=0.0, z=5.0)
    executive.odometry.pose.pose.position = Point(x=19.9, y=1.4, z=20.0)
    target = deepcopy(executive.odometry.pose.pose)
    target.position.y = -1.4
    assert executive._verify_inspection_plan_to(target, geometry, minimum_radius=10.2)
    assert executive.odometry.pose.pose == target
    assert all(not event[1].keepout_center.header.frame_id
               for event in executive.events if event[0] == 'inspection_plan')


def test_local_standoff_tolerates_only_submicrometre_roundoff(executive):
    geometry = target_geometry()
    target = executive._verify_inspection_pose(geometry)
    target.position.x += 1e-12

    assert executive._verify_inspection_plan_to(target, geometry, minimum_radius=10.2)


def test_unranged_recovery_keeps_requested_heading_and_validates_known_free_corridor(executive):
    target = deepcopy(executive.odometry.pose.pose)
    target.position.z += 2.0
    target.orientation = quaternion_from_yaw(0.7)

    assert executive._verify_recovery_plan_to(target)

    plan = next(event for event in executive.events if event[0] == 'plan')
    assert plan[2]['allow_plan_failure'] is True
    assert 'look_at_point' not in plan[2]
    assert yaw_from_quaternion(executive.odometry.pose.pose.orientation) == pytest.approx(0.7)
    request = next(event[1] for event in executive.events
                   if event[0] == 'query' and event[1].corridor.poses)
    assert request.corridor.header.frame_id == 'map'
    assert not request.keepout_center.header.frame_id
    assert request.minimum_radius == 0.0
    assert request.minimum_agl == 5.0
    assert request.terrain_envelope and request.allow_terrain_escape
    planned = next(event[1] for event in executive.events if event[0] == 'inspection_plan')
    assert planned.minimum_radius == 0.0 and planned.minimum_agl == 5.0
    assert planned.terrain_envelope and planned.allow_terrain_escape


def test_localization_advances_from_actual_pose_and_preserves_heading(executive):
    origin = deepcopy(executive.odometry.pose.pose)
    assert executive._verify_localization_viewpoint(index=0, origin=origin, bearing=0.0)
    assert executive.odometry.pose.pose.position == Point(x=3.0, y=1.5, z=20.0)
    assert executive._verify_localization_viewpoint(index=1, origin=origin, bearing=0.0)
    assert executive.odometry.pose.pose.position == Point(x=6.0, y=0.0, z=20.0)
    assert executive._verify_localization_viewpoint(index=5, origin=origin, bearing=0.0)
    assert executive.odometry.pose.pose.position == Point(x=9.0, y=-1.5, z=20.0)
    assert yaw_from_quaternion(executive.odometry.pose.pose.orientation) == 0.0


def test_localization_shortens_an_unobserved_goal_without_guessing_source_range(executive):
    original = deepcopy(executive.odometry.pose.pose)

    def unknown_beyond_sensor_coverage(request, response):
        if request.corridor.poses and request.corridor.poses[-1].pose.position.y > 2.0:
            response.corridor_safe = False
            response.message = 'Inspection corridor contains unknown body clearance'

    executive.on_query = unknown_beyond_sensor_coverage
    target = deepcopy(original)
    target.position.y = 5.0
    assert executive._verify_adaptive_localization_target(target)

    requested = [event[1].position.y for event in executive.events if event[0] == 'plan']
    assert requested == [5.0, 2.5, 1.25]
    assert executive.odometry.pose.pose.position == Point(y=1.25, z=20.0)
    assert yaw_from_quaternion(executive.odometry.pose.pose.orientation) == 0.0
    assert all(event[1].minimum_radius == 0.0 and event[1].minimum_agl == 5.0
               for event in executive.events if event[0] == 'inspection_plan')
    assert any('shorter localization step of 1.25 m' in event[1]
               for event in executive.events if event[0] == 'status')


def test_localization_shorter_steps_remain_blocked_by_occupied_corridors(executive):
    executive.corridor_safe = False
    origin = deepcopy(executive.odometry.pose.pose)

    target = deepcopy(origin)
    target.position.y = 5.0
    assert not executive._verify_adaptive_localization_target(target)

    requested = [event[1].position.y for event in executive.events if event[0] == 'plan']
    assert requested == [5.0, 2.5, 1.25, 0.625]
    assert executive.odometry.pose.pose == origin
    assert any('no useful localization step' in event[1]
               for event in executive.events if event[0] == 'status')


def test_old_lateral_origin_does_not_send_localization_back_to_the_old_view(executive):
    origin = deepcopy(executive.odometry.pose.pose)
    executive.odometry.pose.pose.position.y = 5.0

    assert executive._verify_localization_viewpoint(index=1, origin=origin, bearing=0.0)
    assert executive.odometry.pose.pose.position == Point(x=3.0, y=3.5, z=20.0)


def test_localization_preserves_a_later_climb_instead_of_returning_to_the_old_altitude(executive):
    origin = deepcopy(executive.odometry.pose.pose)
    executive.odometry.pose.pose.position.z = 23.0

    assert executive._verify_localization_viewpoint(index=1, origin=origin, bearing=0.0)
    assert executive.odometry.pose.pose.position.z == 23.0


def test_localization_substeps_preserve_the_position_tolerance_margin(executive):
    executive.config['position_tolerance'] = 0.4
    executive.corridor_safe = False

    target = deepcopy(executive.odometry.pose.pose)
    target.position.y = 5.0
    assert not executive._verify_adaptive_localization_target(target)
    requested = [event[1].position.y for event in executive.events if event[0] == 'plan']
    assert requested == [5.0, 2.5, 1.25]


def test_explicit_small_localization_steps_remain_useful_with_tighter_pose_tolerance(executive):
    executive.config.update(verify_navigation_step=0.4, position_tolerance=0.03)

    assert executive._verify_localization_viewpoint(index=1, bearing=0.0)

    assert executive.odometry.pose.pose.position == Point(x=0.4, y=-0.2, z=20.0)
    assert len([event for event in executive.events if event[0] == 'plan']) == 1


def test_localization_optional_candidates_do_not_suppress_control_failures(executive):
    from kv_executive.mission_executive import MissionError

    def failed(request, response):
        raise MissionError('inspection service timed out')

    executive.on_query = failed
    with pytest.raises(MissionError, match='inspection service timed out'):
        executive._verify_localization_viewpoint(index=1, bearing=0.0)
    assert not any(event[0] == 'plan' for event in executive.events)


def test_coarse_fire_fit_never_changes_bearing_navigation_or_adds_a_keepout(executive):
    geometry = coarse_geometry(x=-100.0, uncertainty=8.0)
    assert executive._verify_localization_viewpoint(geometry, index=0, bearing=0.0)
    assert executive.odometry.pose.pose.position == Point(x=3.0, y=1.5, z=20.0)
    planned = next(event[1] for event in executive.events if event[0] == 'inspection_plan')
    assert planned.minimum_radius == 0.0 and planned.minimum_agl == 5.0
    assert not planned.keepout_center.header.frame_id
    plan = next(event for event in executive.events if event[0] == 'plan')
    assert 'look_at_point' not in plan[2]
    assert not executive._verify_source_at_standoff(geometry)


@pytest.mark.parametrize('invalid', [
    'raw_obstacle', 'smoke', 'stale', 'uncertain', 'few_views', 'baseline',
    'parallax', 'ambiguous',
])
def test_coarse_advance_requires_fresh_thermal_multiview_provenance(executive, invalid):
    geometry = coarse_geometry()
    if invalid == 'raw_obstacle':
        geometry['method'] = 'camera_bearing_octomap_suspected_plume_surface'
        geometry['obstacle_point'] = Point(x=2.0)
    elif invalid == 'smoke':
        geometry['source'] = 'smoke'
    elif invalid == 'stale':
        geometry['source_stamp'] = header(executive.now - 3.0).stamp
    elif invalid == 'uncertain':
        geometry['estimate_uncertainty'] = 21.0
    elif invalid == 'few_views':
        geometry['localization_views'] = 2
    elif invalid == 'baseline':
        geometry['localization_baseline'] = 1.0
    elif invalid == 'parallax':
        geometry['localization_parallax'] = math.radians(1.0)
    else:
        geometry['localization_association'] = 'ambiguous_blobs'

    assert executive._verify_localization_viewpoint(geometry, index=0)

    assert executive.odometry.pose.pose.position.x == 3.0
    assert executive.odometry.pose.pose.position.y == 1.5
    assert executive.odometry.pose.pose.position.z == 20.0
    assert not executive._verify_source_at_standoff(geometry)


def test_coarse_fire_uncertainty_cannot_block_a_terrain_safe_leg(executive):
    geometry = coarse_geometry()
    executive.odometry.pose.pose.position.x = 75.0
    target = deepcopy(executive.odometry.pose.pose)
    target.position.x = 85.0
    assert executive._verify_inspection_plan_to(target, geometry, navigation_only=True)
    assert executive.odometry.pose.pose.position.x == 85.0
    request = next(event[1] for event in executive.events if event[0] == 'inspection_plan')
    assert request.minimum_radius == 0.0 and not request.keepout_center.header.frame_id


def test_coarse_advance_cannot_request_more_than_ten_metres(executive):
    target = deepcopy(executive.odometry.pose.pose)
    target.position.x = 11.0
    assert executive._verify_inspection_plan_to(
        target, coarse_geometry(), navigation_only=True) is False
    assert not any(event[0] == 'plan' for event in executive.events)


def test_coarse_approach_raises_altitude_above_observed_terrain(executive):
    executive.ground_z = 25.0
    assert executive._verify_localization_viewpoint(coarse_geometry(), index=0)
    assert executive.odometry.pose.pose.position.x == 3.0
    assert executive.odometry.pose.pose.position.z == pytest.approx(30.4)


def test_rejected_coarse_advance_can_try_an_ordinary_safe_observation_view(executive):
    def reject_forward_only(request, response):
        if request.corridor.poses and request.corridor.poses[-1].pose.position.x > 1.0:
            response.corridor_safe = False
    executive.on_query = reject_forward_only

    assert executive._verify_localization_viewpoint(coarse_geometry(), index=0)

    assert executive.odometry.pose.pose.position.x == 0.75
    assert executive.odometry.pose.pose.position.z == 20.0
    assert len([event for event in executive.events if event[0] == 'plan']) == 5


def test_coarse_approach_service_errors_do_not_turn_into_optional_viewpoints(executive):
    from kv_executive.mission_executive import MissionError

    def unavailable(*args):
        raise MissionError('service unavailable')

    executive.on_query = unavailable
    with pytest.raises(MissionError, match='service unavailable'):
        executive._verify_localization_viewpoint(coarse_geometry())
    assert not any(event[0] == 'plan' for event in executive.events)


def test_ranged_recovery_ignores_fire_coordinates(executive):
    geometry = target_geometry()
    target = deepcopy(executive.odometry.pose.pose)
    target.position.z += 2.0

    assert executive._verify_recovery_plan_to(target, geometry)

    plan = next(event for event in executive.events if event[0] == 'plan')
    assert 'look_at_point' not in plan[2]
    assert callable(plan[2]['heading_provider'])


@pytest.mark.parametrize('failure', ['unsafe', 'missing_map'])
def test_unranged_recovery_declines_unsafe_or_unavailable_corridor(executive, failure):
    from kv_executive.verification import VerificationMapUnavailable

    def unavailable(request, response):
        if failure == 'unsafe':
            response.corridor_safe = False
            response.message = 'Inspection corridor contains occupied or unknown body clearance'
        else:
            response.success = False
            response.error_code = 1
            response.message = 'No fresh OctoMap available'

    executive.on_query = unavailable
    target = deepcopy(executive.odometry.pose.pose)
    target.position.z += 2.0
    original = deepcopy(executive.odometry.pose.pose)

    if failure == 'missing_map':
        with pytest.raises(VerificationMapUnavailable, match='No fresh OctoMap available'):
            executive._verify_recovery_plan_to(target)
    else:
        assert executive._verify_recovery_plan_to(target) is False

    assert executive.odometry.pose.pose == original


def test_unranged_recovery_does_not_suppress_service_failure(executive):
    from kv_executive.mission_executive import MissionError

    def service_timeout(request, response):
        raise MissionError('inspection service timed out')

    executive.on_query = service_timeout
    target = deepcopy(executive.odometry.pose.pose)
    target.position.z += 2.0

    with pytest.raises(MissionError, match='service timed out'):
        executive._verify_recovery_plan_to(target)


def test_unknown_source_navigation_prefers_forward_oblique_terrain_relative_goal(executive):
    executive.odometry.pose.pose.position.z = 5.0
    geometry = {'target_valid': False, 'point': None, 'source': 'smoke'}

    def sloped_terrain(request, response):
        if request.ground_query.header.frame_id:
            response.ground_surface.point.z = 2.0 + 0.5 * request.ground_query.point.x

    executive.on_query = sloped_terrain

    assert executive._verify_navigation_viewpoint(0.0, geometry=geometry)

    target = executive.odometry.pose.pose
    assert target.position == Point(x=3.0, y=1.5, z=11.1)
    planned = next(event[1] for event in executive.events if event[0] == 'inspection_plan')
    assert planned.minimum_agl == 5.0
    assert planned.allow_initial_climb and planned.allow_terrain_escape
    assert planned.terrain_envelope
    assert not planned.keepout_center.header.frame_id
    assert geometry == {'target_valid': False, 'point': None, 'source': 'smoke'}
    assert not any(event[0] in ('report', 'legacy_report') for event in executive.events)
    assert any('current terrain-envelope clearance 3.00 m' in event[1]
               for event in executive.events if event[0] == 'status')


def test_forward_only_uses_central_corridor_when_lateral_terrain_is_unknown(executive):
    executive.odometry.pose.pose.position.z = 10.0

    def narrow_observed_corridor(request, response):
        if request.ground_query.header.frame_id:
            response.ground_valid = abs(request.ground_query.point.y) < 0.1

    executive.on_query = narrow_observed_corridor

    assert executive._verify_navigation_viewpoint(0.0, forward_only=True)

    assert executive.odometry.pose.pose.position == Point(x=3.0, z=10.0)
    goals = [event[1].target.pose.position for event in executive.events
             if event[0] == 'inspection_plan']
    assert goals == [Point(x=3.0, z=10.0)]
    assert executive._verify_navigation_failure_reason == ''


@pytest.mark.parametrize('current_z,expected_z', [(6.0, 6.0), (4.0, 5.4)])
def test_forward_only_prefers_level_flight_except_for_required_terrain_climb(
        executive, current_z, expected_z):
    executive.odometry.pose.pose.position.z = current_z
    executive.ground_z = 0.0

    assert executive._verify_navigation_viewpoint(0.0, forward_only=True)

    target = executive.odometry.pose.pose.position
    assert (target.x, target.y, target.z) == pytest.approx((3.0, 0.0, expected_z))
    request = next(event[1] for event in executive.events if event[0] == 'inspection_plan')
    assert request.minimum_agl == 5.0 and request.terrain_envelope
    assert request.allow_terrain_escape


def test_forward_only_tries_a_shorter_level_leg_before_a_higher_long_leg(executive):
    executive.odometry.pose.pose.position.z = 6.0
    executive.ground_z = 0.0

    def blocked_long_level_leg(request, response):
        if len(request.corridor.poses) > 1:
            endpoint = request.corridor.poses[-1].pose.position
            response.corridor_safe = endpoint.x < 3.0 or endpoint.z > 7.0

    executive.on_query = blocked_long_level_leg

    assert executive._verify_navigation_viewpoint(0.0, forward_only=True)

    goals = [event[1].target.pose.position for event in executive.events
             if event[0] == 'inspection_plan']
    assert goals == [Point(x=3.0, z=6.0), Point(x=1.5, z=6.0)]
    assert executive.odometry.pose.pose.position == Point(x=1.5, z=6.0)


@pytest.mark.parametrize('unavailable', ['unverified_clearance', 'occupied'])
def test_forward_only_never_replaces_blocked_progress_with_lateral_motion(
        executive, unavailable):
    executive.odometry.pose.pose.position.z = 6.0
    executive.ground_z = 0.0
    executive.config['verify_navigation_candidate_limit'] = 4

    def forward_unavailable(request, response):
        if unavailable == 'unverified_clearance' and request.ground_query.header.frame_id:
            response.ground_valid = request.ground_query.point.x <= 0.0
        if len(request.corridor.poses) > 1:
            response.corridor_safe = False
            response.message = ('occupied cell in the latest map' if unavailable == 'occupied'
                                else 'unverified terrain clearance in strict mode')

    executive.on_query = forward_unavailable
    before = deepcopy(executive.odometry.pose.pose)

    assert not executive._verify_navigation_viewpoint(0.0, forward_only=True)

    assert executive.odometry.pose.pose == before
    goals = [event[1].target.pose.position for event in executive.events
             if event[0] == 'inspection_plan']
    assert len(goals) <= 4
    assert all(point.x > 0.0 and point.y == 0.0 for point in goals)
    assert goals
    assert 'rejected by planning or clearance checks' in (
        executive._verify_navigation_failure_reason)
    if unavailable == 'occupied':
        assert any('occupied cell in the latest map' in event[1]
                   for event in executive.events if event[0] == 'status')


@pytest.mark.parametrize('safe_height,expected_z', [(8.0, 7.4), (12.0, 12.1)])
def test_navigation_tries_other_clearance_bands_when_preferred_goal_is_unavailable(
        executive, safe_height, expected_z):
    executive.odometry.pose.pose.position.z = 5.0

    def band_clearance(request, response):
        if len(request.corridor.poses) > 1:
            height = request.corridor.poses[-1].pose.position.z
            response.corridor_safe = height < safe_height if safe_height == 8.0 else height > 12.0

    executive.on_query = band_clearance

    assert executive._verify_navigation_viewpoint(0.0)

    target = executive.odometry.pose.pose.position
    assert target.x == 3.0 and target.y == 1.5
    assert target.z == pytest.approx(expected_z)
    requests = [event[1] for event in executive.events if event[0] == 'inspection_plan']
    assert requests[0].target.pose.position.z == pytest.approx(9.6)
    assert all(request.minimum_agl == 5.0 for request in requests)


@pytest.mark.parametrize('condition', ['unknown_body', 'missing_map'])
def test_invalid_navigation_start_issues_one_body_query_and_no_goal_requests(executive, condition):
    from kv_executive.verification import VerificationStartUnavailable

    def invalid_start(request, response):
        assert len(request.corridor.poses) == 1
        assert request.minimum_agl == 0.0
        if condition == 'unknown_body':
            response.corridor_safe = False
            response.message = 'unknown body clearance at current pose'
        else:
            response.success = False
            response.error_code = 1

    executive.on_query = invalid_start

    expected = 'Current body clearance' if condition == 'unknown_body' else 'Inspection map'
    with pytest.raises(VerificationStartUnavailable, match=expected):
        executive._verify_navigation_viewpoint(0.0)

    assert len([event for event in executive.events if event[0] == 'query']) == 1
    assert not any(event[0] == 'plan' for event in executive.events)


def test_missing_ground_height_allows_successive_level_legs_with_clearance_checks(executive):
    executive.ground_valid = False
    executive.odometry.pose.pose.position.z = 10.0

    assert executive._verify_navigation_viewpoint(0.0, forward_only=True)
    assert executive._verify_navigation_viewpoint(0.0, forward_only=True)

    assert executive.odometry.pose.pose.position == Point(x=6.0, z=10.0)
    plans = [event[1] for event in executive.events if event[0] == 'inspection_plan']
    assert [plan.target.pose.position for plan in plans] == [
        Point(x=3.0, z=10.0), Point(x=6.0, z=10.0)]
    corridors = [event[1] for event in executive.events
                 if event[0] == 'query' and len(event[1].corridor.poses) > 1]
    assert len(corridors) == 2
    for request in plans + corridors:
        assert request.minimum_agl == 5.0
        assert request.terrain_envelope
    assert any('ground height unavailable; retaining altitude' in event[1]
               for event in executive.events if event[0] == 'status')


def test_lost_ground_height_does_not_waive_a_new_path_obstacle(executive):
    executive.ground_valid = False
    assert executive._verify_navigation_viewpoint(0.0, forward_only=True)
    before = deepcopy(executive.odometry.pose.pose)

    def new_obstacle(request, response):
        if len(request.corridor.poses) > 1:
            response.corridor_safe = False
            response.message = 'occupied terrain inside required clearance'

    executive.on_query = new_obstacle
    assert not executive._verify_navigation_viewpoint(0.0, forward_only=True)
    assert executive.odometry.pose.pose == before


def test_missing_ground_height_rejects_a_descending_planner_detour(executive):
    executive.ground_valid = False
    assert executive._verify_navigation_viewpoint(0.0, forward_only=True)
    plan = next(event for event in executive.events if event[0] == 'plan')
    validate = plan[2]['path_validator']
    path = Path(header=header(), poses=[
        PoseStamped(pose=Pose(position=Point(z=20.0))),
        PoseStamped(pose=Pose(position=Point(x=1.5, z=18.0))),
        PoseStamped(pose=Pose(position=Point(x=3.0, z=20.0))),
    ])

    assert not validate(path)
    assert any('descending path withheld' in event[1]
               for event in executive.events if event[0] == 'status')


@pytest.mark.parametrize('condition', ['missing', 'stale', 'future'])
def test_map_unavailability_remains_distinct_from_terrain_or_body_rejection(executive, condition):
    from kv_executive.verification import VerificationMapUnavailable

    def unavailable(request, response):
        if condition == 'missing':
            response.success = False
            response.error_code = 1
            response.message = 'source observation unavailable'
        else:
            response.map_header = header(
                executive.now + (1.0 if condition == 'future' else -6.0), 'map')

    executive.on_query = unavailable
    with pytest.raises(VerificationMapUnavailable) as caught:
        executive._verify_recovery_plan_to(deepcopy(executive.odometry.pose.pose))
    assert caught.value.source_stamp_ns > 0
    assert not any(event[0] == 'plan' for event in executive.events)
    assert not any('terrain envelope unavailable' in event[1]
                   for event in executive.events if event[0] == 'status')


def test_fresh_map_poll_preserves_acquisition_stamp_and_does_not_probe_terrain(executive):
    assert executive._verify_fresh_map_stamp(timeout=0.5) == int(executive.now * 1e9)
    request = next(event[1] for event in executive.events if event[0] == 'query')
    assert not request.corridor.poses and not request.ground_query.header.frame_id


def test_map_poll_tolerates_service_timeout_but_preserves_control_faults(executive):
    from kv_executive.mission_executive import MissionError, MissionTimeout
    from kv_executive.srv import QueryInspectionGeometry
    from kv_executive.verification import VerificationMapUnavailable

    def unavailable(*args, **kwargs):
        assert kwargs['timeout'] == 0.5
        raise MissionTimeout('Timed out: Query inspection geometry service')

    executive._call = unavailable
    assert executive._verify_fresh_map_stamp(timeout=0.5) == 0
    assert 'geometry service unavailable' in executive._verify_map_unavailable_reason
    with pytest.raises(VerificationMapUnavailable, match='geometry service unavailable'):
        executive._inspection_query(
            QueryInspectionGeometry.Request(), timeout=0.5, require_map=True)

    def control_fault(*args, **kwargs):
        raise MissionError('FCU disconnected')

    executive._call = control_fault
    with pytest.raises(MissionError, match='FCU disconnected'):
        executive._verify_fresh_map_stamp(timeout=0.5)


def test_navigation_goals_keep_source_time_transformed_camera_bearing(executive):
    executive.conversion.rotation = quaternion_from_yaw(0.5)
    geometry = executive._verify_target_geometry(executive.smoke)
    expected = 0.5 + math.atan2(2.0, 35.0)
    assert geometry['navigation_bearing'] == pytest.approx(expected)

    assert executive._verify_navigation_viewpoint(0.0, geometry=geometry)

    target = executive.odometry.pose.pose
    assert target.position.x == pytest.approx(3.0 * math.cos(expected) - 1.5 * math.sin(expected))
    assert target.position.y == pytest.approx(3.0 * math.sin(expected) + 1.5 * math.cos(expected))
    assert yaw_from_quaternion(target.orientation) == pytest.approx(expected)
    assert geometry['point'] is None and not geometry['target_valid']


def test_navigation_ignores_fire_range_even_when_localization_is_valid(executive):
    geometry = target_geometry(x=-1000.0)
    assert executive._verify_navigation_viewpoint(0.0, geometry=geometry)
    request = next(event[1] for event in executive.events if event[0] == 'inspection_plan')
    assert not request.keepout_center.header.frame_id
    assert request.minimum_radius == 0.0 and request.minimum_agl == 5.0
    assert any('image-bearing guidance' in event[1]
               for event in executive.events if event[0] == 'status')


def test_navigation_candidate_budget_is_bounded_when_all_routes_are_occupied(executive):
    executive.config['verify_navigation_candidate_limit'] = 6
    executive.odometry.pose.pose.position.z = 5.0

    def blocked_destination(request, response):
        if len(request.corridor.poses) > 1:
            response.corridor_safe = False

    executive.on_query = blocked_destination
    assert not executive._verify_navigation_viewpoint(0.0)

    assert len([event for event in executive.events if event[0] == 'inspection_plan']) == 6
    assert executive.odometry.pose.pose.position == Point(z=5.0)


def test_lateral_movement_is_only_a_fallback_after_forward_candidates_fail(executive):
    def blocked_forward(request, response):
        if len(request.corridor.poses) > 1:
            response.corridor_safe = request.corridor.poses[-1].pose.position.x <= 0.0

    executive.on_query = blocked_forward
    assert executive._verify_navigation_viewpoint(0.0)

    positions = [event[1].position for event in executive.events if event[0] == 'plan']
    assert all(point.x > 0.0 for point in positions[:-1])
    assert positions[-1] == Point(y=1.5, z=20.0)


def test_clearance_fallback_uses_observed_five_metres_when_preferred_height_is_unknown(executive):
    executive.odometry.pose.pose.position.z = 5.0

    def low_vertical_corridor_only(request, response):
        if len(request.corridor.poses) > 1:
            endpoint = request.corridor.poses[-1].pose.position
            response.corridor_safe = endpoint.x == endpoint.y == 0.0 and endpoint.z < 8.0

    executive.on_query = low_vertical_corridor_only
    assert executive._verify_navigation_viewpoint(0.0)

    assert executive.odometry.pose.pose.position.z == pytest.approx(7.4)
    requests = [event[1] for event in executive.events if event[0] == 'inspection_plan']
    assert requests[-1].minimum_agl == 5.0
    assert requests[-1].allow_terrain_escape
    assert requests[-1].target.pose.position.z < 8.0
    assert len(requests) <= executive.config.get('verify_navigation_candidate_limit', 18)


def test_recovery_destination_is_raised_above_the_terrain_floor(executive):
    executive.odometry.pose.pose.position.z = 4.0
    target = deepcopy(executive.odometry.pose.pose)
    target.position.z = 4.5

    assert executive._verify_recovery_plan_to(target)

    assert target.position.z == 4.5
    assert executive.odometry.pose.pose.position.z == pytest.approx(7.4)
    request = next(event[1] for event in executive.events if event[0] == 'inspection_plan')
    assert request.minimum_agl == 5.0 and request.allow_terrain_escape


@pytest.mark.parametrize('requested_z, expected_z', [(19.5, 19.5), (3.0, 7.4)])
def test_vertical_image_centering_can_descend_only_to_the_observed_terrain_floor(
        executive, requested_z, expected_z):
    target = deepcopy(executive.odometry.pose.pose)
    target.position.z = requested_z

    assert executive._verify_recovery_plan_to(target, preserve_requested_altitude=True)

    assert executive.odometry.pose.pose.position.z == pytest.approx(expected_z)
    request = next(event[1] for event in executive.events if event[0] == 'inspection_plan')
    assert request.minimum_agl == 5.0 and request.terrain_envelope
    assert request.allow_terrain_escape


def test_floor_blocked_descent_does_not_reverse_into_an_upward_centering_command(executive):
    executive.odometry.pose.pose.position.z = 5.0
    target = deepcopy(executive.odometry.pose.pose)
    target.position.z -= 0.5

    assert not executive._verify_recovery_plan_to(target, preserve_requested_altitude=True)

    assert executive._verify_height_rejection == 'terrain_floor'
    assert executive.odometry.pose.pose.position.z == 5.0
    assert not any(event[0] == 'plan' for event in executive.events)


def test_unknown_terrain_never_authorizes_downward_image_centering(executive):
    executive.ground_valid = False
    target = deepcopy(executive.odometry.pose.pose)
    target.position.z -= 0.5

    assert not executive._verify_recovery_plan_to(target, preserve_requested_altitude=True)

    assert executive._verify_height_rejection == 'terrain_unavailable'
    assert not any(event[0] == 'plan' for event in executive.events)


def test_a_later_plan_refusal_cannot_reuse_a_previous_height_floor_rejection(executive):
    executive.odometry.pose.pose.position.z = 7.4
    target = deepcopy(executive.odometry.pose.pose)
    target.position.z -= 0.5
    assert not executive._verify_recovery_plan_to(target, preserve_requested_altitude=True)
    assert executive._verify_height_rejection == 'terrain_floor'

    executive.odometry.pose.pose.position.z = 20.0
    target.position.z = 19.5
    executive.corridor_safe = False
    assert not executive._verify_recovery_plan_to(target, preserve_requested_altitude=True)
    assert executive._verify_height_rejection == ''


def test_live_heading_prefers_fresh_fire_then_smoke_and_holds_the_last_view_through_gaps(
        executive):
    executive.fire = _persistent_fire()
    executive.fire.yaw_correction_rad = -0.4
    executive.smoke.yaw_correction_rad = 0.3
    heading = executive._verify_live_source_heading(0.7)
    pose = executive.odometry.pose.pose

    assert heading(pose) == pytest.approx(-math.radians(5.0))
    executive.fire.header = header(executive.now - 3.0, 'thermal_camera')
    assert heading(pose) == pytest.approx(math.radians(5.0))
    executive.smoke = None
    assert heading(pose) == pytest.approx(math.radians(5.0))
    assert executive._verify_live_source_heading(0.7)(pose) == 0.7


def test_same_image_does_not_accumulate_yaw_corrections_during_a_path(executive):
    executive.fire = _persistent_fire()
    heading = executive._verify_live_source_heading(0.0)
    pose = executive.odometry.pose.pose
    first = heading(pose)
    executive.odometry.pose.pose.orientation = quaternion_from_yaw(first)

    assert heading(pose) == first

    executive.fire.header = header(executive.now + 0.01, 'thermal_camera')
    assert heading(pose) == pytest.approx(first + math.radians(5.0))


@pytest.mark.parametrize('failure', ['wrong_source', 'stale_map', 'slant_far', 'body_far'])
def test_smoke_boundary_rejects_stale_or_inconsistent_proximity(executive, failure):
    geometry = smoke_boundary_geometry()
    if failure == 'wrong_source':
        geometry['source'] = 'fire'
    elif failure == 'stale_map':
        geometry['map_header'] = header(executive.now - 6.0, 'map')
    elif failure == 'slant_far':
        geometry['obstacle_range'] = 15.0
    else:
        geometry['obstacle_point'].z = 0.0  # Close in XY, too far below in 3D.

    assert not executive._verify_smoke_boundary_reached(geometry)


def test_no_fire_report_accepts_observed_smoke_boundary_without_fire_localization(executive):
    executive.smoke_wall_valid = True
    _trust_smoke_for_commit(executive)

    report = executive._verify_report_inspection({}, attempts=3)

    assert not report.fire_found and report.status == 'fire_not_found'
    assert not report.fire_location_valid and not report.smoke_location_valid
    assert report.observer_global_valid


def test_navigation_forward_limit_bounds_the_first_goal_without_losing_parallax(executive):
    assert executive._verify_navigation_viewpoint(0.0, maximum_forward=0.8)
    assert executive.odometry.pose.pose.position == Point(x=0.8, y=0.4, z=20.0)


def test_start_becoming_invalid_during_planning_stops_the_candidate_fan(executive):
    from kv_executive.verification import VerificationStartUnavailable
    calls = []

    def invalidated(*args, **kwargs):
        calls.append(1)
        raise VerificationStartUnavailable('map changed at start')

    executive._verify_recovery_plan_to = invalidated
    with pytest.raises(VerificationStartUnavailable, match='map changed'):
        executive._verify_navigation_viewpoint(0.0)
    assert len(calls) == 1


def test_exhausted_start_recovery_does_not_repeat_for_each_destination(executive):
    from kv_executive.verification import VerificationRecoveryUnavailable
    calls = []

    def unavailable(*args, **kwargs):
        calls.append(1)
        raise VerificationRecoveryUnavailable('bounded escape exhausted')

    executive._verify_recovery_plan_to = unavailable
    with pytest.raises(VerificationRecoveryUnavailable, match='escape exhausted'):
        executive._verify_navigation_viewpoint(0.0)
    assert len(calls) == 1


def test_navigation_can_use_a_retreating_tangent_to_follow_convex_terrain(executive):
    def only_retreating_tangent(request, response):
        if len(request.corridor.poses) > 1:
            endpoint = request.corridor.poses[-1].pose.position
            response.corridor_safe = endpoint.x < 0.0 and endpoint.y > 2.0

    executive.on_query = only_retreating_tangent

    assert executive._verify_navigation_viewpoint(0.0)

    endpoint = executive.odometry.pose.pose.position
    assert endpoint == Point(x=-0.75, y=3.0, z=20.0)
    assert len([event for event in executive.events if event[0] == 'inspection_plan']) <= 18


def test_enu_geolocation_changes_target_coordinates_in_correct_directions(executive):
    fix = executive._global_position
    target = enu_offset_to_fix(fix, 10.0, 20.0, -5.0, 0.2)
    assert target.latitude > fix.latitude
    assert target.longitude > fix.longitude
    assert target.altitude == 45.0
    assert target.position_covariance[0] >= 0.04


@pytest.mark.parametrize('invalid', ['pole', 'no_fix', 'nan', 'long_offset'])
def test_geolocation_rejects_unsupported_reference_or_offset(executive, invalid):
    fix = deepcopy(executive._global_position)
    east = 10.0
    if invalid == 'pole':
        fix.latitude = 90.0
    elif invalid == 'no_fix':
        fix.status.status = NavSatStatus.STATUS_NO_FIX
    elif invalid == 'nan':
        fix.longitude = math.nan
    else:
        east = 1001.0
    assert enu_offset_to_fix(fix, east, 0.0, 0.0) is None


def test_smoke_only_without_semantic_range_cannot_complete_an_inspection(executive):
    from kv_executive.verification import VerificationObservationLost
    with pytest.raises(VerificationObservationLost, match='fresh observed smoke boundary'):
        executive._verify_report_inspection(target_geometry(), attempts=3)
    assert not any(event[0] in {'report', 'legacy_report'} for event in executive.events)


def test_report_never_relabels_a_stale_anchor_or_obstacle_as_current_smoke(executive):
    executive.smoke_wall_valid = True
    stale_anchor = target_geometry(now=executive.now - 30.0, x=100.0)
    _seed_localized_fire(executive)

    report = executive._verify_report_inspection(stale_anchor, executive.fire, attempts=3)

    assert not report.smoke_location_valid and not report.smoke_global_valid
    assert report.fire_found and report.fire_location_valid
    assert math.isnan(report.smoke_local_position.point.x)


@pytest.mark.parametrize('horizontal', [0.01, 0.3])
def test_below_image_fire_can_be_reported_without_perfect_centering(executive, horizontal):
    executive.smoke_wall_valid = True
    executive.fire = SimpleNamespace(
        header=header(frame='thermal_camera'), detected=True, status=2, CONFIRMED=2,
        yaw_correction_rad=horizontal, vertical_correction_rad=-0.5, source_count=2, track_id=42)
    _seed_localized_fire(executive, executive.fire)

    report = executive._verify_report_inspection(target_geometry(), executive.fire, attempts=1)

    assert report.fire_found and report.fire_location_valid and report.fire_global_valid
    assert report.track_id == 42 and report.source_count == 2
    assert report.fire_local_position.point.x == pytest.approx(35.0)
    assert report.fire_global_position.latitude > report.observer_global_position.latitude
    assert report.fire_global_position.altitude < report.observer_global_position.altitude
    assert next(event for event in executive.events if event[0] == 'legacy_report')[3] is False
    assert report.site_id == next(
        event[4] for event in executive.events if event[0] == 'legacy_report')
    assert executive._site_sequence == 1


def test_missing_surface_and_gps_publish_explicit_unknown_coordinates(executive):
    executive.smoke_wall_valid = True
    _seed_localized_fire(executive)
    executive._global_position.header = header(executive.now - 3.0, 'gps')
    executive.on_query = lambda request, response: setattr(response, 'surface_valid', False)

    report = executive._verify_report_inspection({'point': None}, executive.fire, attempts=3)

    assert not report.observer_global_valid
    assert not report.smoke_location_valid and not report.smoke_global_valid
    assert math.isnan(report.smoke_global_position.latitude)
    assert math.isnan(report.smoke_local_position.point.x)
    assert report.fire_location_valid and not report.fire_global_valid


def test_fire_becoming_persistent_during_final_query_is_never_reported_not_found(executive):
    executive.smoke_wall_valid = True
    _seed_localized_fire(executive)
    confirmed = deepcopy(executive.fire)
    executive.fire = None

    def fire_arrives(request, response):
        executive.fire = deepcopy(confirmed)

    executive.on_query = fire_arrives

    report = executive._verify_report_inspection(target_geometry(), attempts=3)

    assert report.fire_found
    assert report.status == 'fire_found'
    assert report.fire_location_valid


@pytest.mark.parametrize('phase', ['query', 'observer_transform'])
def test_confirmed_then_lost_fire_during_report_work_cannot_publish_not_found(executive, phase):
    from kv_executive.verification import VerificationObservationLost
    _trust_smoke_for_commit(executive)

    def confirmation_then_negative():
        executive.fire = _persistent_fire()
        executive.fire_confirmed = True
        executive.fire = None  # A newer negative detector frame clears guidance.

    if phase == 'query':
        executive.on_query = lambda request, response: confirmation_then_negative()
    else:
        original_transform = executive._path_frame_transform
        calls = []

        def delayed_transform(frame):
            calls.append(frame)
            if len(calls) == 2:  # Smoke ray conversion precedes the observer transform.
                confirmation_then_negative()
            return original_transform(frame)

        executive._path_frame_transform = delayed_transform

    with pytest.raises(VerificationObservationLost, match='Confirmed fire was observed'):
        executive._verify_report_inspection(target_geometry(), attempts=3)

    assert executive.fire is None
    assert not any(event[0] in {'report', 'legacy_report'} for event in executive.events)


def test_fire_loss_during_geometry_queries_cannot_publish_a_found_report(executive):
    from kv_executive.verification import VerificationObservationLost
    executive.fire = SimpleNamespace(
        header=header(frame='thermal_camera'), detected=True, status=2, CONFIRMED=2,
        yaw_correction_rad=0.0, vertical_correction_rad=-0.5, source_count=1, track_id=42)
    original = deepcopy(executive.fire)

    def lose_fire(request, response):
        if request.origin.header.frame_id == 'thermal_camera':
            executive.fire = None
    executive.on_query = lose_fire

    with pytest.raises(VerificationObservationLost):
        executive._verify_report_inspection(target_geometry(), original)
    assert not any(event[0] == 'report' for event in executive.events)


def _persistent_fire(now=100.0, track_id=42):
    return SimpleNamespace(
        header=header(now, 'thermal_camera'), detected=True, status=2, CONFIRMED=2,
        yaw_correction_rad=0.2, vertical_correction_rad=-0.4, source_count=1, track_id=track_id)


def _seed_localized_fire(executive, message=None):
    executive.fire = deepcopy(message) if message is not None else _persistent_fire(executive.now)
    executive.odometry.pose.pose.position.x = 25.0
    for index, y in enumerate((0.0, 10.0, -10.0)):
        executive.now = 100.0 + index * 0.01
        executive.fire.header = header(executive.now, 'thermal_camera')
        executive.ray_origin = (25.0, y, 20.0)
        geometry = executive._verify_target_geometry(executive.fire, 'fire')
    assert geometry['target_valid']


def _trust_smoke_for_commit(executive):
    """Supply a fresh observed smoke wall to exercise final classification guards."""
    original = executive._verify_target_geometry

    def localized(message, source='smoke'):
        geometry = original(message, source)
        if source == 'smoke':
            geometry = smoke_boundary_geometry(executive.now)
        return geometry
    executive._verify_target_geometry = localized


def test_fire_location_used_for_navigation_cannot_be_reported_as_smoke(executive):
    executive.smoke_wall_valid = True
    _seed_localized_fire(executive)
    executive.smoke = None
    fire_anchor = target_geometry(x=35.0)
    fire_anchor['source'] = 'fire'

    report = executive._verify_report_inspection(fire_anchor, deepcopy(executive.fire), attempts=3)

    assert report.fire_found and report.fire_location_valid
    assert not report.smoke_location_valid and not report.smoke_global_valid
    assert math.isnan(report.smoke_local_position.point.x)


def test_fire_at_observed_wall_standoff_can_have_distant_localized_coordinates(executive):
    executive.smoke_wall_valid = True
    _seed_localized_fire(executive)
    executive.odometry.pose.pose.position.x = 0.0
    report = executive._verify_report_inspection(target_geometry(), executive.fire)
    assert report.fire_found
    assert report.status == 'fire_found'
    assert any(event[0] == 'report' for event in executive.events)


def test_visible_fire_can_complete_with_unknown_target_coordinates(executive):
    executive.smoke_wall_valid = True
    executive.fire = _persistent_fire()
    report = executive._verify_report_inspection({}, executive.fire)
    assert report.fire_found and not report.fire_location_valid
    assert report.observer_global_valid
    assert math.isnan(report.fire_local_position.point.x)
    assert any(event[0] == 'legacy_report' for event in executive.events)


@pytest.mark.parametrize('wall_failure', [
    'missing', 'far', 'too_close', 'uncertain', 'few_samples', 'stale_map'])
def test_visible_fire_cannot_report_without_fresh_wall_standoff(executive, wall_failure):
    from kv_executive.verification import VerificationMapUnavailable, VerificationObservationLost
    executive.fire = _persistent_fire()
    executive.smoke_wall_valid = wall_failure != 'missing'

    def invalidate_wall(request, response):
        if not request.smoke_wall_width:
            return
        if wall_failure == 'far':
            response.smoke_wall_range = response.smoke_wall_nearest_range = 19.81
        elif wall_failure == 'too_close':
            response.smoke_wall_range = response.smoke_wall_nearest_range = 9.0
        elif wall_failure == 'uncertain':
            response.smoke_wall_uncertainty = 3.0
        elif wall_failure == 'few_samples':
            response.smoke_wall_samples = 1
        elif wall_failure == 'stale_map':
            response.map_header = header(executive.now - 6.0, 'map')

    executive.on_query = invalidate_wall
    with pytest.raises((VerificationObservationLost, VerificationMapUnavailable)):
        executive._verify_report_inspection({}, executive.fire)

    assert not any(event[0] in {'report', 'legacy_report'} for event in executive.events)
    requests = [event[1] for event in executive.events if event[0] == 'query']
    assert any(request.smoke_wall_width == 3.0 for request in requests)


@pytest.mark.parametrize('late_change', ['missing_wall', 'distant_wall', 'fire_lost'])
def test_found_report_rechecks_wall_and_fire_after_legacy_work(executive, late_change):
    from kv_executive.verification import VerificationObservationLost
    executive.fire = _persistent_fire()
    executive.smoke_wall_valid = True
    wall_queries = []

    def change_at_commit(request, response):
        if not request.smoke_wall_width:
            return
        wall_queries.append(request)
        if len(wall_queries) == 2:
            if late_change == 'missing_wall':
                response.smoke_wall_valid = False
            elif late_change == 'distant_wall':
                response.smoke_wall_range = response.smoke_wall_nearest_range = 19.81
            else:
                executive.fire = None

    original_fire = deepcopy(executive.fire)
    executive.on_query = change_at_commit
    with pytest.raises(VerificationObservationLost):
        executive._verify_report_inspection({}, original_fire)

    assert len(wall_queries) == 2
    assert not any(event[0] in {'report', 'legacy_report'} for event in executive.events)
    assert executive._site_sequence == 0


def test_slow_wall_queries_refresh_fire_and_observer_before_commit(executive):
    executive.fire = _persistent_fire()
    executive.smoke_wall_valid = True
    wall_queries = []

    def receive_fresh_data_while_waiting(request, response):
        if not request.smoke_wall_width:
            return
        wall_queries.append(request)
        executive.now += 3.0
        executive.fire.header = header(executive.now, 'thermal_camera')
        executive.smoke.header = header(executive.now, 'rgb_camera')
        executive.odometry.header = header(executive.now)
        executive._global_position.header = header(executive.now, 'gps')
        executive._global_position_received = executive.now
        response.map_header = header(executive.now, 'map')

    prepare = executive._verify_record_fire

    def require_current_fire(message, *args, **kwargs):
        assert (executive.now - inspection._stamp_ns(message) / 1e9) < 2.0
        return prepare(message, *args, **kwargs)

    executive.on_query = receive_fresh_data_while_waiting
    executive._verify_record_fire = require_current_fire
    report = executive._verify_report_inspection({}, deepcopy(executive.fire))

    assert len(wall_queries) == 2
    assert inspection._stamp_ns(report.observer_local_pose) == int(executive.now * 1e9)
    assert report.observer_global_valid
    assert report.site_id == 1
    assert [event[0] for event in executive.events if event[0] in {'report', 'legacy_report'}] == [
        'legacy_report', 'report']


def test_stale_observer_after_final_wall_query_prevents_both_reports(executive):
    from kv_executive.mission_executive import MissionError
    executive.fire = _persistent_fire()
    executive.smoke_wall_valid = True
    wall_queries = []

    def lose_telemetry(request, response):
        if request.smoke_wall_width:
            wall_queries.append(request)
            if len(wall_queries) == 2:
                executive.odometry.header = header(executive.now - 3.0)

    executive.on_query = lose_telemetry
    with pytest.raises(MissionError, match='Observation pose became stale'):
        executive._verify_report_inspection({}, deepcopy(executive.fire))
    assert executive._site_sequence == 0
    assert not any(event[0] in {'report', 'legacy_report'} for event in executive.events)


@pytest.mark.parametrize('standoff', [10.0, 20.0])
@pytest.mark.parametrize('offset, accepted', [
    (-0.31, False), (-0.3, False), (-0.01, False), (0.0, True), (0.6, True), (0.61, False)])
def test_wall_standoff_uses_conservative_range_without_accepting_inside_boundary(
        standoff, offset, accepted):
    wall = {'nearest': standoff + offset + 0.2, 'uncertainty': 0.2}
    assert inspection.smoke_wall_standoff_reached(wall, standoff, 0.3) is accepted


@pytest.mark.parametrize('late_change', ['fire', 'smoke_lost'])
def test_no_fire_evidence_is_rechecked_after_observer_transform(executive, late_change):
    from kv_executive.verification import VerificationObservationLost
    _trust_smoke_for_commit(executive)
    original_transform = executive._path_frame_transform
    calls = []

    def delayed_transform(frame):
        calls.append(frame)
        if len(calls) == 2:  # After smoke ray conversion, during observer conversion.
            if late_change == 'fire':
                executive.fire = _persistent_fire()
            else:
                executive.smoke = None
        return original_transform(frame)

    executive._path_frame_transform = delayed_transform
    with pytest.raises(VerificationObservationLost):
        executive._verify_report_inspection(target_geometry(), attempts=3)
    assert not any(event[0] == 'report' for event in executive.events)


def test_found_track_is_rechecked_after_observer_transform_without_gps(executive):
    from kv_executive.verification import VerificationObservationLost
    _seed_localized_fire(executive)
    original_fire = deepcopy(executive.fire)
    executive._global_position.status.status = NavSatStatus.STATUS_NO_FIX
    original_transform = executive._path_frame_transform
    calls = []

    def delayed_transform(frame):
        calls.append(frame)
        if len(calls) == 3:  # Smoke, fire, then observer conversion.
            executive.fire.track_id = 99
        return original_transform(frame)

    executive._path_frame_transform = delayed_transform
    with pytest.raises(VerificationObservationLost):
        executive._verify_report_inspection(target_geometry(), original_fire)
    assert not any(event[0] in {'report', 'legacy_report'} for event in executive.events)


def test_fire_loss_after_legacy_preparation_prevents_both_reports(executive):
    executive.smoke_wall_valid = True
    from kv_executive.verification import VerificationObservationLost
    _seed_localized_fire(executive)
    original_fire = deepcopy(executive.fire)
    legacy_report = executive._verify_record_fire

    def lost_after_legacy(*args, **kwargs):
        result = legacy_report(*args, **kwargs)
        executive.fire = None
        return result

    executive._verify_record_fire = lost_after_legacy
    with pytest.raises(VerificationObservationLost):
        executive._verify_report_inspection(target_geometry(), original_fire)
    assert any(event[0] == 'legacy_prepare' for event in executive.events)
    assert not any(event[0] in {'report', 'legacy_report'} for event in executive.events)


def test_center_fire_wall_queries_physical_camera_aperture_and_preserves_mean(executive):
    def wall(request, response):
        assert request.smoke_wall_width == 3.0
        assert request.direction.vector.x == 1.0
        assert request.origin.header.frame_id == 'thermal_camera'
        assert request.origin.header.stamp.sec == 0
        response.smoke_wall_valid = True
        response.smoke_wall_range = 15.0
        response.smoke_wall_nearest_range = 14.0
        response.smoke_wall_uncertainty = 0.2
        response.smoke_wall_samples = 25
    executive.on_query = wall
    measured = executive._verify_center_fire_wall('thermal_camera')
    assert measured['mean'] == 15.0 and measured['nearest'] == 14.0
    assert measured['origin'] == Point(z=20.0)
    assert math.hypot(*measured['axis']) == pytest.approx(1.0)


@pytest.mark.parametrize('reason', ['missing', 'few_samples', 'uncertain', 'nan'])
def test_center_fire_wall_rejects_unverified_distance(executive, reason):
    def wall(request, response):
        response.smoke_wall_valid = reason != 'missing'
        response.smoke_wall_range = 15.0
        response.smoke_wall_nearest_range = math.nan if reason == 'nan' else 14.0
        response.smoke_wall_uncertainty = 3.0 if reason == 'uncertain' else 0.2
        response.smoke_wall_samples = 1 if reason == 'few_samples' else 25
    executive.on_query = wall
    assert executive._verify_center_fire_wall('thermal_camera') is None


def test_center_fire_wall_rejects_map_aging_during_transform(executive):
    executive.smoke_wall_valid = True
    original_transform = executive._path_frame_transform

    def delayed_transform(frame):
        executive.now += 6.0
        return original_transform(frame)

    executive._path_frame_transform = delayed_transform
    assert executive._verify_center_fire_wall('thermal_camera') is None


@pytest.mark.parametrize('live_distance', [None, 12.0, 25.0])
def test_center_fire_wall_path_keeps_old_bound_and_tightens_for_closer_wall(
    executive, live_distance,
):
    wall = {'origin': Point(z=20.0), 'axis': (1.0, 0.0, 0.0), 'mean': 15.0,
            'nearest': 15.0, 'uncertainty': 0.2, 'camera_frame': 'thermal_camera'}
    live = None if live_distance is None else {**wall, 'nearest': live_distance}
    executive._verify_center_fire_wall = lambda frame: live
    geometry = {'center_fire_wall': wall}

    def path(x):
        return Path(poses=[PoseStamped(pose=Pose(position=Point(x=x, z=23.0)))])
    assert executive._verify_center_fire_wall_path(path(1.0), geometry)
    assert not executive._verify_center_fire_wall_path(path(5.0), geometry)
    assert executive._verify_center_fire_wall_path(path(3.0), geometry) == (live_distance != 12.0)
    assert executive._verify_center_fire_wall_path(path(-1.0), geometry)


def test_center_fire_recovery_passes_fixed_scalar_heading_and_movement_limits(executive):
    wall = {'origin': Point(z=20.0), 'axis': (1.0, 0.0, 0.0), 'mean': 20.0,
            'nearest': 20.0, 'uncertainty': 0.2, 'camera_frame': 'thermal_camera'}
    executive._verify_center_fire_wall = lambda frame: None
    geometry = {'center_fire_wall': wall, 'center_fire_max_path_length': 12.0,
                'center_fire_motion_timeout': 60.0}
    target = Pose(position=Point(x=1.0, z=20.0), orientation=quaternion_from_yaw(0.2))
    assert executive._verify_recovery_plan_to(target, geometry=geometry)
    options = next(e[2] for e in executive.events if e[0] == 'plan')
    assert options['heading_provider'](target) == pytest.approx(0.2)
    assert options['maximum_path_length'] == 12.0
    assert options['motion_timeout'] == 60.0


def test_center_fire_can_take_a_short_final_wall_limited_step(executive):
    executive._verify_center_fire_wall = lambda frame: None
    wall = {'origin': Point(z=20.0), 'axis': (1.0, 0.0, 0.0), 'mean': 11.0,
            'nearest': 11.0, 'uncertainty': 0.2, 'camera_frame': 'thermal_camera'}
    geometry = {'center_fire_wall': wall, 'center_fire_max_path_length': 12.0,
                'center_fire_motion_timeout': 60.0}
    assert executive._verify_navigation_viewpoint(0.0, geometry=geometry, maximum_forward=0.5)
    assert executive.odometry.pose.pose.position.x == pytest.approx(0.5)


@pytest.mark.parametrize('vertical_offset', [-0.5, 0.5])
def test_fire_altitude_bias_changes_next_navigation_goal_and_preserves_clearance(
        executive, vertical_offset):
    original = deepcopy(executive.odometry.pose.pose.position)
    assert executive._verify_navigation_viewpoint(0.0, vertical_offset=vertical_offset)
    target = executive.odometry.pose.pose.position
    assert (target.x, target.y, target.z) == pytest.approx(
        (3.0, 1.5, original.z + vertical_offset))
    request = next(e[1] for e in executive.events if e[0] == 'inspection_plan')
    assert request.minimum_agl == 5.0
    assert request.terrain_envelope and request.allow_terrain_escape


def test_fire_descent_can_follow_falling_terrain_without_altitude_ratchet(executive):
    executive.odometry.pose.pose.position.z = 7.6
    assert executive._verify_navigation_viewpoint(0.0, vertical_offset=-0.5)
    # The requested 7.1 m goal is clipped to support + AGL + uncertainty + tolerance.
    assert executive.odometry.pose.pose.position.z == pytest.approx(7.4)


@pytest.mark.parametrize('reason', ['missing_terrain', 'blocked_path'])
def test_unavailable_fire_descent_does_not_fall_back_to_forward_or_climbing_motion(
        executive, reason):
    if reason == 'missing_terrain':
        executive.ground_valid = False
    else:
        def block_path(request, response):
            if len(request.corridor.poses) > 1:
                response.corridor_safe = False
        executive.on_query = block_path
    original = deepcopy(executive.odometry.pose.pose)
    assert not executive._verify_navigation_viewpoint(0.0, vertical_offset=-0.5)
    assert executive.odometry.pose.pose == original
    requests = [e[1] for e in executive.events if e[0] == 'inspection_plan']
    assert all(r.target.pose.position.z < original.position.z for r in requests)
    if reason != 'blocked_path':
        assert not requests


@pytest.mark.parametrize('forward_only', [False, True])
@pytest.mark.parametrize('rising_ground', [False, True])
def test_terrain_limited_fire_descent_selects_a_separate_checked_forward_leg(
        executive, forward_only, rising_ground):
    executive.odometry.pose.pose.position.z = 7.4

    def terrain_ahead(request, response):
        if rising_ground and request.ground_query.point.x > 0.0:
            response.ground_surface.point.z = 3.0

    executive.on_query = terrain_ahead
    assert executive._verify_navigation_viewpoint(
        0.0, forward_only=forward_only, vertical_offset=-0.5)

    target = executive.odometry.pose.pose.position
    assert (target.x, target.y, target.z) == pytest.approx(
        (3.0, 0.0, 8.4 if rising_ground else 7.4))
    plans = [event for event in executive.events if event[0] == 'inspection_plan']
    assert len(plans) == 1
    assert plans[0][1].minimum_agl == 5.0
    assert plans[0][1].terrain_envelope and plans[0][1].allow_terrain_escape
    assert any('trying a checked forward leg' in event[1] for event in executive.events
               if event[0] == 'status')
    assert not any(event[0] in {'report', 'legacy_report'} for event in executive.events)


def test_floor_limited_forward_fallback_still_checks_the_entire_terrain_corridor(executive):
    executive.odometry.pose.pose.position.z = 7.4

    def block_corridor(request, response):
        if len(request.corridor.poses) > 1:
            response.corridor_safe = False

    executive.on_query = block_corridor
    assert not executive._verify_navigation_viewpoint(
        0.0, forward_only=True, vertical_offset=-0.5)
    assert executive.odometry.pose.pose.position == Point(z=7.4)
    requests = [event[1] for event in executive.events if event[0] == 'inspection_plan']
    assert len(requests) == 3
    assert all(request.target.pose.position.z == pytest.approx(7.4) for request in requests)


def test_floor_limited_forward_fallback_obeys_the_same_candidate_budget(executive):
    executive.odometry.pose.pose.position.z = 7.4
    executive.config['verify_navigation_candidate_limit'] = 2

    def block_corridor(request, response):
        if len(request.corridor.poses) > 1:
            response.corridor_safe = False

    executive.on_query = block_corridor
    assert not executive._verify_navigation_viewpoint(
        0.0, forward_only=True, vertical_offset=-0.5)
    assert len([event for event in executive.events if event[0] == 'inspection_plan']) == 2


def test_floor_limited_forward_fallback_can_choose_a_shorter_safe_leg(executive):
    executive.odometry.pose.pose.position.z = 7.4

    def short_corridor_only(request, response):
        if len(request.corridor.poses) > 1:
            response.corridor_safe = request.corridor.poses[-1].pose.position.x <= 1.5

    executive.on_query = short_corridor_only
    assert executive._verify_navigation_viewpoint(
        0.0, forward_only=True, vertical_offset=-0.5)
    assert executive.odometry.pose.pose.position == Point(x=1.5, z=7.4)


@pytest.mark.parametrize('interrupt', ['cancel', 'return'])
def test_floor_limited_forward_fallback_preserves_vehicle_interrupts(executive, interrupt):
    from kv_executive.mission_executive import MissionCanceled, MissionStateRequested
    executive.odometry.pose.pose.position.z = 7.4
    exception = MissionCanceled if interrupt == 'cancel' else MissionStateRequested

    def guard():
        if any(event[0] == 'status' and 'trying a checked forward leg' in event[1]
               for event in executive.events):
            reason = 'vehicle interrupt before fallback' if interrupt == 'cancel' else 'Return'
            raise exception(reason)

    with pytest.raises(exception):
        executive._verify_navigation_viewpoint(
            0.0, forward_only=True, vertical_offset=-0.5, guard=guard)
    assert executive.odometry.pose.pose.position == Point(z=7.4)
    assert not any(event[0] == 'inspection_plan' for event in executive.events)


def test_floor_limited_forward_fallback_stops_for_a_new_map_outage(executive):
    from kv_executive.verification import VerificationMapUnavailable
    executive.odometry.pose.pose.position.z = 7.4

    def stale_fallback_corridor(request, response):
        if len(request.corridor.poses) > 1:
            response.map_header = header(executive.now - 6.0, 'map')

    executive.on_query = stale_fallback_corridor
    with pytest.raises(VerificationMapUnavailable):
        executive._verify_navigation_viewpoint(0.0, forward_only=True, vertical_offset=-0.5)
    assert executive.odometry.pose.pose.position == Point(z=7.4)


def test_descent_becomes_eligible_again_after_a_terrain_following_leg(executive):
    executive.odometry.pose.pose.position.z = 7.4
    assert executive._verify_navigation_viewpoint(
        0.0, forward_only=True, vertical_offset=-0.5)
    assert executive.odometry.pose.pose.position == Point(x=3.0, z=7.4)

    executive.ground_z = 0.0
    assert executive._verify_navigation_viewpoint(
        0.0, forward_only=True, vertical_offset=-0.5)
    assert executive.odometry.pose.pose.position == Point(x=6.0, z=6.9)


@pytest.mark.parametrize('remaining', [0.0, 0.5])
def test_floor_limited_forward_fallback_cannot_cross_fifteen_meter_wall(executive, remaining):
    executive.config['verify_smoke_standoff'] = 15.0
    executive.odometry.pose.pose.position.z = 7.4
    wall = _measured_navigation_wall(executive, nearest=15.5 + remaining)
    wall['origin'].z = 7.4
    geometry = {'wall_probe_only': True, 'smoke_navigation_wall': wall}
    assert executive._verify_navigation_viewpoint(
        0.0, geometry=geometry, forward_only=True, vertical_offset=-0.5) is (remaining > 0.0)
    assert executive.odometry.pose.pose.position.x == pytest.approx(remaining)
    assert executive.odometry.pose.pose.position.z == pytest.approx(7.4)


@pytest.mark.parametrize('offset', [-0.51, 0.51, math.nan, math.inf, True])
def test_navigation_rejects_unbounded_or_invalid_fire_altitude_bias(executive, offset):
    from kv_executive.mission_executive import MissionError
    with pytest.raises(MissionError, match='Navigation viewpoint'):
        executive._verify_navigation_viewpoint(0.0, vertical_offset=offset)
    assert not executive.events


def navigation_smoke_wall(standoff=20.0, now=100.0):
    return {
        **smoke_boundary_geometry(now), 'occupied_samples': 3, 'free_samples': 1,
        'ray_origin': Point(z=20.0),
        'obstacle_point': Point(x=standoff + 1.0, z=20.0),
        'obstacle_range': standoff + 1.0,
    }


@pytest.mark.parametrize('standoff', [10.0, 20.0])
def test_smoke_navigation_clips_final_leg_to_measured_wall(executive, standoff):
    executive.config['verify_smoke_standoff'] = standoff
    geometry = navigation_smoke_wall(standoff)
    original = deepcopy(geometry)
    assert executive._verify_navigation_viewpoint(0.0, geometry=geometry, forward_only=True)
    # One metre remaining less 0.2 m uncertainty and 0.3 m tracking margin.
    target = executive.odometry.pose.pose.position
    assert (target.x, target.y, target.z) == pytest.approx((0.5, 0.0, 20.0))
    assert geometry == original


def test_slow_preflight_retains_measured_smoke_bound_and_accounts_for_actual_position(executive):
    executive.config['verify_smoke_standoff'] = 20.0
    geometry = navigation_smoke_wall()
    preflight = executive._verify_navigation_preflight

    def delayed_preflight(guard):
        preflight(guard)
        executive.now += executive.config['verify_observation_timeout'] + 1.0
        executive.odometry.pose.pose.position.x = 0.1
        return executive._verify_control_pose()[1]

    executive._verify_navigation_preflight = delayed_preflight
    assert executive._verify_navigation_viewpoint(0.0, geometry=geometry, forward_only=True)
    target = executive.odometry.pose.pose.position
    assert (target.x, target.y, target.z) == pytest.approx((0.5, 0.0, 20.0))
    request = next(e[1] for e in executive.events if e[0] == 'inspection_plan')
    assert request.target.pose.position.x == pytest.approx(0.5)
    # It is no longer acceptable as NEW guidance, but its committed bound held.
    assert executive._verify_smoke_navigation_wall(geometry) is None


@pytest.mark.parametrize('bearing', [math.radians(170.0), 0.2, -2.3])
def test_short_clipped_smoke_leg_keeps_level_candidate_at_arbitrary_bearing(executive, bearing):
    executive.config['verify_smoke_standoff'] = 20.0
    origin = Point(x=2.3, y=-5.7, z=9.5)
    executive.odometry.pose.pose.position = deepcopy(origin)
    distance = 20.9
    geometry = {
        **navigation_smoke_wall(), 'ray_origin': deepcopy(origin),
        'obstacle_point': Point(x=origin.x + distance * math.cos(bearing),
                                y=origin.y + distance * math.sin(bearing), z=origin.z),
        'obstacle_range': distance,
    }
    assert executive._verify_navigation_viewpoint(bearing, geometry=geometry, forward_only=True)
    target = executive.odometry.pose.pose.position
    assert (target.x, target.y, target.z) == pytest.approx(
        (origin.x + 0.4 * math.cos(bearing), origin.y + 0.4 * math.sin(bearing), origin.z))
    requests = [e[1] for e in executive.events if e[0] == 'inspection_plan']
    assert len(requests) == 1


@pytest.mark.parametrize('standoff', [10.0, 20.0])
@pytest.mark.parametrize('offset,arrived', [(0.7, True), (0.8, True), (0.81, False)])
def test_smoke_boundary_arrival_avoids_a_final_sub_tolerance_leg(
        executive, standoff, offset, arrived):
    executive.config['verify_smoke_standoff'] = standoff
    geometry = navigation_smoke_wall(standoff)
    geometry['obstacle_point'].x = geometry['obstacle_range'] = standoff + offset
    assert executive._verify_smoke_boundary_reached(geometry) is arrived


@pytest.mark.parametrize('distance,accepted', [
    (10.5, False), (19.81, False), (20.19, False), (20.2, True), (20.8, True), (20.81, False),
])
def test_twenty_metre_setting_applies_to_both_positive_report_publishers(
        executive, distance, accepted):
    from kv_executive.verification import VerificationObservationLost
    executive.config['verify_smoke_standoff'] = 20.0
    executive.fire = _persistent_fire()
    executive.smoke_wall_valid = True
    executive.smoke_wall_nearest = distance
    if accepted:
        assert executive._verify_report_inspection({}, executive.fire).fire_found
        assert any(e[0] == 'legacy_report' for e in executive.events)
    else:
        with pytest.raises(VerificationObservationLost, match='fresh smoke wall'):
            executive._verify_report_inspection({}, executive.fire)
        assert not any(e[0] in {'report', 'legacy_report'} for e in executive.events)


@pytest.mark.parametrize('invalid', [
    'static_surface', 'few_occupied', 'no_free', 'stale_image', 'stale_map',
    'uncertain', 'no_origin',
])
def test_only_fresh_supported_plume_geometry_can_clip_a_smoke_leg(executive, invalid):
    executive.config['verify_smoke_standoff'] = 20.0
    geometry = navigation_smoke_wall()
    if invalid == 'static_surface':
        geometry['plume_candidate'] = False
    elif invalid == 'few_occupied':
        geometry['occupied_samples'] = 1
    elif invalid == 'no_free':
        geometry['free_samples'] = 0
    elif invalid == 'stale_image':
        geometry['source_stamp'] = header(97.0).stamp
    elif invalid == 'stale_map':
        geometry['map_header'] = header(94.0, 'map')
    elif invalid == 'uncertain':
        geometry['obstacle_uncertainty'] = 3.0
    else:
        del geometry['ray_origin']
    assert executive._verify_navigation_viewpoint(0.0, geometry=geometry, forward_only=True)
    assert executive.odometry.pose.pose.position == Point(x=3.0, z=20.0)


def test_smoke_wall_plane_checks_detours_and_survives_image_ageing(executive):
    executive.config['verify_smoke_standoff'] = 20.0
    geometry = navigation_smoke_wall()
    assert executive._verify_navigation_viewpoint(0.0, geometry=geometry, forward_only=True)
    validate = next(e[2]['path_validator'] for e in executive.events if e[0] == 'plan')
    executive.now += 3.0
    safe = Path(header=header(frame='px4_odom'), poses=[
        PoseStamped(pose=Pose(position=Point(x=0.4, z=20.0)))])
    assert validate(safe)
    unsafe = Path(header=header(frame='px4_odom'), poses=[
        PoseStamped(pose=Pose(position=Point(x=0.6, z=20.0))), *safe.poses])
    assert not validate(unsafe)


def test_smoke_wall_clipping_uses_image_ray_origin_and_full_3d_plane(executive):
    executive.config['verify_smoke_standoff'] = 20.0
    geometry = navigation_smoke_wall()
    geometry['ray_origin'] = Point(x=-1.0, z=21.0)
    # A 45-degree downward ray from an earlier camera position.
    geometry['obstacle_point'] = Point(x=20.0, z=0.0)
    geometry['obstacle_range'] = math.hypot(21.0, 21.0)
    wall = executive._verify_smoke_navigation_wall(geometry)
    assert wall['axis'] == pytest.approx((2 ** -0.5, 0.0, -2 ** -0.5))
    allowed = wall['nearest'] - wall['uncertainty'] - 20.0 - 0.3
    assert executive._verify_wall_plane_path(Path(poses=[PoseStamped(pose=Pose(
        position=Point(x=allowed * math.sqrt(2) - 1.1, z=21.0)))]), wall)
    # Same horizontal position, but descending advances through the tilted plane.
    assert not executive._verify_wall_plane_path(Path(poses=[PoseStamped(pose=Pose(
        position=Point(x=allowed * math.sqrt(2) - 1.1, z=20.0)))]), wall)


def test_vertical_fire_motion_preserves_measured_wall_plane(executive):
    executive.config['verify_smoke_standoff'] = 20.0
    executive._verify_center_fire_wall = lambda frame: None
    wall = {'origin': Point(z=20.0), 'axis': (2 ** -0.5, 0.0, -2 ** -0.5),
            'mean': 20.5, 'nearest': 20.5, 'uncertainty': 0.2,
            'camera_frame': 'thermal_camera'}
    geometry = {'center_fire_wall': wall, 'center_fire_max_path_length': 12.0,
                'center_fire_motion_timeout': 60.0}
    target = deepcopy(executive.odometry.pose.pose)
    target.position.z -= 0.5
    assert not executive._verify_recovery_plan_to(
        target, geometry=geometry, preserve_requested_altitude=True)
    assert executive.odometry.pose.pose.position.z == 20.0


@pytest.mark.parametrize('bearing', [0.0, math.pi / 2, -math.pi / 2])
def test_opportunistic_wall_uses_camera_origin_and_rotated_navigation_bearing(executive, bearing):
    executive.smoke_wall_valid = True
    executive.conversion.rotation = quaternion_from_yaw(math.pi / 2)
    executive.conversion.translation.x = 30.0
    wall = executive._verify_opportunistic_wall('rgb_camera', bearing)
    request = next(event[1] for event in executive.events if event[0] == 'query')
    assert request.origin.header.frame_id == 'rgb_camera'
    assert request.origin.point == Point()
    assert request.direction.header.frame_id == 'map'
    assert request.direction.vector.x == pytest.approx(math.cos(bearing - math.pi / 2))
    assert request.direction.vector.y == pytest.approx(math.sin(bearing - math.pi / 2))
    assert request.direction.vector.z == pytest.approx(0.0)
    assert wall['bearing'] == bearing
    assert wall['map_stamp_ns'] == int(executive.now * 1e9)


def test_opportunistic_wall_timeout_does_not_block_ordinary_approach(executive):
    from kv_executive.mission_executive import MissionTimeout
    calls = []

    def timeout(client, request, description, timeout=None):
        calls.append(timeout)
        raise MissionTimeout('probe deadline')

    executive._call = timeout
    assert executive._verify_opportunistic_wall('rgb_camera', 0.0) is None
    assert calls == [0.5]


@pytest.mark.parametrize('failure', [
    'absent', 'few_samples', 'uncertain', 'stale_map', 'map_error'])
def test_opportunistic_wall_distinguishes_no_range_from_bad_map(executive, failure):
    from kv_executive.verification import VerificationMapUnavailable
    executive.smoke_wall_valid = True

    def unavailable(request, response):
        if failure == 'absent':
            response.smoke_wall_valid = False
        elif failure == 'few_samples':
            response.smoke_wall_samples = 1
        elif failure == 'uncertain':
            response.smoke_wall_uncertainty = 3.0
        elif failure == 'stale_map':
            response.map_header = header(executive.now - 6.0, 'map')
        else:
            response.success = False
            response.error_code = 1
            response.message = 'map unavailable'

    executive.on_query = unavailable
    if failure in ('stale_map', 'map_error'):
        with pytest.raises(VerificationMapUnavailable):
            executive._verify_opportunistic_wall('rgb_camera', 0.0)
    else:
        assert executive._verify_opportunistic_wall('rgb_camera', 0.0) is None


def _measured_navigation_wall(executive, nearest=10.8, axis=(1.0, 0.0, 0.0)):
    return {'origin': deepcopy(executive.odometry.pose.pose.position), 'axis': axis,
            'nearest': nearest, 'mean': nearest, 'uncertainty': 0.2,
            'camera_frame': 'rgb_camera', 'bearing': math.atan2(axis[1], axis[0]),
            'map_header': header(executive.now, 'map')}


def test_retained_wall_limits_forward_leg_without_fresh_smoke_geometry(executive):
    wall = _measured_navigation_wall(executive)
    wall['map_header'] = header(executive.now - 10.0, 'map')
    geometry = {'source': 'smoke', 'smoke_navigation_wall': wall}
    assert executive._verify_navigation_viewpoint(0.0, geometry=geometry, forward_only=True)
    assert executive.odometry.pose.pose.position.x == pytest.approx(0.3)
    assert not executive._verify_smoke_wall_at_standoff(wall)


def test_all_retained_wall_planes_limit_a_new_approach_direction(executive):
    xwall = _measured_navigation_wall(executive, nearest=11.5)
    ywall = _measured_navigation_wall(executive, nearest=10.8, axis=(0.0, 1.0, 0.0))
    geometry = {'smoke_navigation_wall': xwall, 'smoke_navigation_walls': [xwall, ywall]}
    assert executive._verify_navigation_viewpoint(
        math.pi / 4, geometry=geometry, forward_only=True)
    position = executive.odometry.pose.pose.position
    assert position.x == pytest.approx(0.3)
    assert position.y == pytest.approx(0.3)


def test_wall_violation_allows_monotonic_retreat_but_not_deeper_entry(executive):
    wall = _measured_navigation_wall(executive, nearest=10.5)
    executive.odometry.pose.pose.position.x = 1.0

    def path(*positions):
        return Path(poses=[PoseStamped(pose=Pose(position=Point(x=x, z=20.0)))
                           for x in positions])

    assert executive._verify_wall_plane_path(path(1.0, 0.5, 0.0), wall)
    assert not executive._verify_wall_plane_path(path(1.0, 1.1, 0.0), wall)
    assert not executive._verify_wall_plane_path(path(1.0, 0.5, 0.7, 0.0), wall)


def test_no_fire_report_accepts_partial_smoke_with_remeasured_wall(executive):
    executive.smoke.coverage = 0.05
    executive.smoke.regions = [1]
    executive.smoke_wall_valid = True
    wall = _measured_navigation_wall(executive)
    wall['map_header'] = header(executive.now - 20.0, 'map')
    report = executive._verify_report_inspection({}, attempts=3, wall=wall)
    assert report.status == 'fire_not_found' and not report.fire_found
    probes = [event for event in executive.events if event[0] == 'query'
              and event[1].smoke_wall_width]
    assert len(probes) == 2


@pytest.mark.parametrize('failure', ['unavailable', 'far', 'close', 'late_loss', 'prior_fire',
                                     'unfinished_search'])
def test_no_fire_report_needs_completed_search_and_current_wall(executive, failure):
    from kv_executive.verification import VerificationObservationLost
    executive.smoke_wall_valid = failure != 'unavailable'
    executive.fire_confirmed = failure == 'prior_fire'
    if failure == 'far':
        executive.smoke_wall_nearest = 30.0
    elif failure == 'close':
        executive.smoke_wall_nearest = 8.0
    probes = []

    def late_loss(request, response):
        if request.smoke_wall_width:
            probes.append(request)
            if failure == 'late_loss' and len(probes) == 2:
                response.smoke_wall_valid = False

    executive.on_query = late_loss
    wall = _measured_navigation_wall(executive)
    with pytest.raises(VerificationObservationLost):
        executive._verify_report_inspection(
            {}, attempts=2 if failure == 'unfinished_search' else 3, wall=wall)
    assert not any(event[0] == 'report' for event in executive.events)


def test_wall_arrival_requires_current_pose_proximity_and_fresh_map(executive):
    wall = _measured_navigation_wall(executive, nearest=12.5)
    assert not executive._verify_smoke_wall_at_standoff(wall)
    executive.odometry.pose.pose.position.x = 2.0
    assert executive._verify_smoke_wall_at_standoff(wall)
    executive.odometry.pose.pose.position.x = 3.0
    assert not executive._verify_smoke_wall_at_standoff(wall)
    executive.odometry.pose.pose.position.x = 2.0
    executive.now += 6.0
    assert not executive._verify_smoke_wall_at_standoff(wall)


def test_fresh_plume_hit_tightens_an_older_retained_aperture_plane(executive):
    retained = _measured_navigation_wall(executive, nearest=30.0)
    geometry = {**navigation_smoke_wall(standoff=10.0),
                'smoke_navigation_wall': retained, 'smoke_navigation_walls': [retained]}
    assert executive._verify_navigation_viewpoint(0.0, geometry=geometry, forward_only=True)
    assert executive.odometry.pose.pose.position.x == pytest.approx(0.5)
    validate = next(event[2]['path_validator'] for event in executive.events if event[0] == 'plan')
    executive.now += 10.0  # The accepted nearer plane survives image ageing during this leg.
    crossing = Path(poses=[PoseStamped(pose=Pose(position=Point(x=0.6, z=20.0)))])
    assert not validate(crossing)


@pytest.mark.parametrize('samples, accepted', [(5, False), (6, True)])
def test_wall_probe_python_guard_requires_at_least_six_samples(executive, samples, accepted):
    executive.smoke_wall_valid = True
    executive.on_query = lambda request, response: setattr(response, 'smoke_wall_samples', samples)
    wall = executive._verify_opportunistic_wall('rgb_camera', 0.0)
    assert (wall is not None) is accepted


def test_legacy_single_ray_cannot_authorize_a_negative_report_without_aperture(executive):
    from kv_executive.verification import VerificationObservationLost
    _trust_smoke_for_commit(executive)
    assert executive._verify_smoke_boundary_reached(smoke_boundary_geometry())
    assert not executive.smoke_wall_valid
    with pytest.raises(VerificationObservationLost, match='fresh observed smoke boundary'):
        executive._verify_report_inspection({}, attempts=3)
    assert not any(event[0] == 'report' for event in executive.events)


def test_negative_report_remeasures_fifteen_meter_wall_without_legacy_geometry(executive):
    executive.config['verify_smoke_standoff'] = 15.0
    executive.smoke_wall_valid = True
    executive.smoke_wall_nearest = 15.5
    report = executive._verify_report_inspection({}, attempts=3)
    assert not report.fire_found
    assert report.status == 'fire_not_found'
    wall_requests = [event[1] for event in executive.events
                     if event[0] == 'query' and event[1].smoke_wall_width]
    assert len(wall_requests) == 2
    assert all(request.origin.header.frame_id == executive.smoke.header.frame_id
               for request in wall_requests)


def test_fire_appearing_during_final_negative_wall_probe_prevents_publication(executive):
    from kv_executive.verification import VerificationObservationLost
    executive.smoke_wall_valid = True
    probes = []

    def fire_arrives(request, response):
        if request.smoke_wall_width:
            probes.append(request)
            if len(probes) == 2:
                executive.fire = _persistent_fire(executive.now)

    executive.on_query = fire_arrives
    with pytest.raises(VerificationObservationLost, match='Smoke or no-fire evidence changed'):
        executive._verify_report_inspection({}, attempts=3)
    assert len(probes) == 2
    assert not any(event[0] == 'report' for event in executive.events)


def test_default_wall_movement_limit_is_fifteen_meters(executive):
    executive.config.pop('verify_smoke_standoff')
    wall = _measured_navigation_wall(executive, nearest=16.0)
    inside = Path(poses=[PoseStamped(pose=Pose(position=Point(x=0.5, z=20.0)))])
    crossing = Path(poses=[PoseStamped(pose=Pose(position=Point(x=0.6, z=20.0)))])
    assert executive._verify_wall_plane_path(inside, wall)
    assert not executive._verify_wall_plane_path(crossing, wall)


@pytest.mark.parametrize('aperture', [False, True])
def test_probe_only_navigation_never_promotes_a_legacy_ray_to_a_wall(executive, aperture):
    geometry = {**navigation_smoke_wall(standoff=10.0), 'wall_probe_only': True}
    if aperture:
        geometry['smoke_navigation_wall'] = _measured_navigation_wall(executive, nearest=11.0)
    assert executive._verify_navigation_viewpoint(0.0, geometry=geometry, forward_only=True)
    expected = 0.5 if aperture else 3.0
    assert executive.odometry.pose.pose.position.x == pytest.approx(expected)
