"""Geometry invariants used by staged flight missions, without ROS nodes."""

import math

from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Transform, Vector3
from kv_executive.mission_geometry import (
    angular_distance,
    interpolate_pose,
    quaternion_from_yaw,
    relative_transform,
    split_path,
    transform_pose,
    yaw_from_quaternion,
)
from nav_msgs.msg import Path
import pytest
from std_msgs.msg import Header


def pose(x=0.0, y=0.0, z=0.0, yaw=0.0):
    return Pose(
        position=Point(x=float(x), y=float(y), z=float(z)),
        orientation=quaternion_from_yaw(yaw),
    )


def stamped(value, seconds=0, frame='map'):
    result = PoseStamped(header=Header(frame_id=frame), pose=value)
    result.header.stamp.sec = seconds
    return result


def path_with(points):
    return Path(
        header=Header(frame_id='map'),
        poses=[stamped(value, index) for index, value in enumerate(points)],
    )


def xyz(value):
    return (value.position.x, value.position.y, value.position.z)


def arc_length(path):
    return math.fsum(
        math.dist(xyz(start.pose), xyz(end.pose))
        for start, end in zip(path.poses, path.poses[1:])
    )


def assert_same_rotation(actual, expected):
    actual_values = (actual.x, actual.y, actual.z, actual.w)
    expected_values = (expected.x, expected.y, expected.z, expected.w)
    dot = math.fsum(a * b for a, b in zip(actual_values, expected_values))
    assert abs(dot) == pytest.approx(1.0, abs=1e-10)


def test_split_uneven_sampling_keeps_corners_and_equal_arc_length():
    # Total length is 9 m; splitting by vertex count would produce unequal legs.
    original = path_with([pose(0, 0), pose(1, 0), pose(1, 3), pose(6, 3)])
    parts = split_path(original)

    assert len(parts) == 3
    assert [arc_length(part) for part in parts] == pytest.approx([3.0] * 3)
    assert xyz(parts[0].poses[-1].pose) == pytest.approx((1, 2, 0))
    assert xyz(parts[1].poses[-1].pose) == pytest.approx((3, 3, 0))
    assert original.poses[1] in parts[0].poses
    assert original.poses[2] in parts[1].poses
    assert parts[0].poses[0] == original.poses[0]
    assert parts[-1].poses[-1] == original.poses[-1]
    assert all(part.header == original.header for part in parts)
    for left, right in zip(parts, parts[1:]):
        assert left.poses[-1] == right.poses[0]
        assert left.poses[-1] is not right.poses[0]


def test_split_boundaries_interpolate_timestamp_and_shortest_yaw():
    original = path_with([
        pose(0),
        pose(1, yaw=math.radians(170)),
        pose(4, yaw=math.radians(-170)),
        pose(9),
    ])
    for stamped_pose, seconds in zip(original.poses, [0, 1, 4, 9]):
        stamped_pose.header.stamp.sec = seconds
    parts = split_path(original)

    boundary = parts[0].poses[-1]
    assert boundary.pose.position.x == pytest.approx(3.0)
    assert boundary.header.frame_id == 'map'
    assert boundary.header.stamp.sec == 3
    assert boundary.header.stamp.nanosec == 0
    expected = math.radians(170 + 20 * (2 / 3))
    assert angular_distance(yaw_from_quaternion(boundary.pose.orientation), expected) \
        == pytest.approx(0.0)
    assert parts[1].poses[-1].header.stamp.sec == 6


def test_split_uses_three_dimensional_length_and_exact_vertex_boundary():
    original = path_with([pose(0, 0, 0), pose(0, 0, 2), pose(4, 0, 2)])
    parts = split_path(original)

    assert [arc_length(part) for part in parts] == pytest.approx([2.0] * 3)
    assert parts[0].poses[-1] == original.poses[1]
    assert parts[1].poses[0] == original.poses[1]
    assert xyz(parts[1].poses[-1].pose) == pytest.approx((2, 0, 2))


def test_split_preserves_repeated_vertices_and_orientations_at_boundary():
    original = path_with([
        pose(0), pose(0, yaw=0.1),
        pose(2), pose(2, yaw=0.4), pose(2, yaw=0.8),
        pose(6), pose(6, yaw=1.0),
    ])
    parts = split_path(original)

    assert len(parts) == 3
    assert [arc_length(part) for part in parts] == pytest.approx([2.0] * 3)
    for vertex in original.poses:
        assert any(vertex in part.poses for part in parts)
    assert parts[0].poses[-1] == parts[1].poses[0] == original.poses[2]
    assert original.poses[3] in parts[1].poses
    assert original.poses[4] in parts[1].poses


def test_split_more_segments_than_original_edges_and_no_input_aliasing():
    original = path_with([pose(0), pose(4)])
    parts = split_path(original, segments=8)
    assert len(parts) == 8
    assert [arc_length(part) for part in parts] == pytest.approx([0.5] * 8)
    assert [part.poses[-1].pose.position.x for part in parts] \
        == pytest.approx([0.5, 1, 1.5, 2, 2.5, 3, 3.5, 4])

    parts[0].poses[0].pose.position.x = -100.0
    parts[0].header.frame_id = 'changed'
    assert original.poses[0].pose.position.x == 0.0
    assert original.header.frame_id == 'map'
    assert parts[1].header.frame_id == 'map'


def test_single_segment_returns_an_independent_complete_copy():
    original = path_with([pose(0), pose(1), pose(1, 2)])
    parts = split_path(original, 1)
    assert parts == [original]
    assert parts[0] is not original
    assert parts[0].poses[1] is not original.poses[1]


@pytest.mark.parametrize('segments', [0, -1, 1.5, True, None])
def test_split_rejects_invalid_segment_count(segments):
    with pytest.raises(ValueError):
        split_path(path_with([pose(0), pose(1)]), segments)


@pytest.mark.parametrize('points', [[], [pose()], [pose(), pose(), pose()]])
def test_split_rejects_empty_or_zero_length_paths(points):
    with pytest.raises(ValueError):
        split_path(path_with(points))


def test_split_rejects_malformed_and_nonfinite_poses_and_mixed_frames():
    with pytest.raises(ValueError):
        split_path(None)
    malformed = path_with([pose(), pose(1)])
    malformed.poses[1].pose.orientation = Quaternion(w=0.0)
    with pytest.raises(ValueError):
        split_path(malformed)

    nonfinite = path_with([pose(), pose(float('nan'))])
    with pytest.raises(ValueError):
        split_path(nonfinite)

    mixed = path_with([pose(), pose(1)])
    mixed.poses[1].header.frame_id = 'odom'
    with pytest.raises(ValueError):
        split_path(mixed)


def test_yaw_normalizes_scaled_huge_and_subnormal_quaternions():
    for scale in [3.0, 1e308, 1e-320]:
        quaternion = Quaternion(z=scale, w=scale)
        assert yaw_from_quaternion(quaternion) == pytest.approx(math.pi / 2)


def test_yaw_wrap_and_signed_distance():
    assert angular_distance(math.radians(-179), math.radians(179)) \
        == pytest.approx(math.radians(2))
    assert angular_distance(math.radians(179), math.radians(-179)) \
        == pytest.approx(math.radians(-2))
    assert angular_distance(2 * math.pi, 0.0) == pytest.approx(0.0, abs=1e-14)
    assert math.isfinite(angular_distance(1e308, -1e308))


@pytest.mark.parametrize('direction', [1, -1])
def test_full_360_yaw_sequence_can_be_accumulated_across_wrap(direction):
    expected = [direction * math.radians(degrees) for degrees in range(0, 361, 15)]
    measured = [yaw_from_quaternion(quaternion_from_yaw(yaw)) for yaw in expected]
    accumulated = math.fsum(
        angular_distance(end, start) for start, end in zip(measured, measured[1:])
    )
    assert accumulated == pytest.approx(direction * math.tau)


def test_interpolation_uses_shortest_yaw_and_linear_position():
    start = pose(0, 2, 4, math.radians(170))
    end = pose(4, 6, 8, math.radians(-170))
    halfway = interpolate_pose(start, end, 0.5)
    assert xyz(halfway) == pytest.approx((2, 4, 6))
    assert abs(yaw_from_quaternion(halfway.orientation)) == pytest.approx(math.pi)
    assert xyz(start) == (0, 2, 4)
    assert xyz(end) == (4, 6, 8)


def test_interpolation_handles_antipodal_equivalent_quaternions_and_roll():
    start = pose()
    end = pose(2)
    end.orientation.w = -1.0
    result = interpolate_pose(start, end, 0.5)
    assert xyz(result) == (1, 0, 0)
    assert_same_rotation(result.orientation, start.orientation)

    # Full quaternion interpolation is needed when map/local frames include roll.
    end.orientation = Quaternion(x=1.0, w=0.0)
    result = interpolate_pose(start, end, 0.5)
    assert_same_rotation(
        result.orientation,
        Quaternion(x=math.sqrt(0.5), w=math.sqrt(0.5)),
    )


@pytest.mark.parametrize('bad_quaternion', [
    Quaternion(w=0.0), Quaternion(x=float('nan'), w=1.0),
    Quaternion(z=float('inf'), w=1.0), None,
])
def test_quaternion_validation(bad_quaternion):
    with pytest.raises(ValueError):
        yaw_from_quaternion(bad_quaternion)


@pytest.mark.parametrize('fraction', [-0.01, 1.01, float('nan'), float('inf'), None])
def test_interpolation_rejects_invalid_fraction(fraction):
    with pytest.raises(ValueError):
        interpolate_pose(pose(), pose(1), fraction)


def test_interpolation_validates_both_endpoints_even_at_fraction_zero():
    invalid = pose()
    invalid.orientation = Quaternion(w=0.0)
    with pytest.raises(ValueError):
        interpolate_pose(pose(), invalid, 0.0)
    with pytest.raises(ValueError):
        interpolate_pose(pose(float('inf')), pose(), 0.5)


def test_map_to_px4_local_transform_rotates_and_translates_waypoint():
    # The same body is at map (10,20,5), yaw +90deg, and PX4 local (1,2,3), yaw 0.
    source_body = pose(10, 20, 5, math.pi / 2)
    local_body = pose(1, 2, 3)
    conversion = relative_transform(local_body, source_body)
    transformed_body = transform_pose(source_body, conversion)
    assert xyz(transformed_body) == pytest.approx((1, 2, 3))
    assert_same_rotation(transformed_body.orientation, local_body.orientation)

    # Two metres ahead in map +Y become two metres ahead in local +X.
    waypoint = transform_pose(pose(10, 22, 5, math.pi / 2), conversion)
    assert xyz(waypoint) == pytest.approx((3, 2, 3))
    assert yaw_from_quaternion(waypoint.orientation) == pytest.approx(0.0, abs=1e-12)


def test_transform_supports_full_3d_rotation_and_nonunit_input():
    conversion = Transform(
        translation=Vector3(x=2.0, y=3.0, z=4.0),
        rotation=Quaternion(x=2.0, w=2.0),  # +90deg roll.
    )
    transformed = transform_pose(pose(0, 1, 0), conversion)
    assert xyz(transformed) == pytest.approx((2, 3, 5))
    assert_same_rotation(
        transformed.orientation,
        Quaternion(x=math.sqrt(0.5), w=math.sqrt(0.5)),
    )


def test_relative_transform_inverts_arbitrary_orientation_consistently():
    source_body = pose(3, -4, 2)
    source_body.orientation = Quaternion(x=0.5, y=0.5, z=0.5, w=0.5)
    local_body = pose(7, 5, -1, 0.7)
    conversion = relative_transform(local_body, source_body)
    transformed = transform_pose(source_body, conversion)
    assert xyz(transformed) == pytest.approx(xyz(local_body))
    assert_same_rotation(transformed.orientation, local_body.orientation)


def test_transform_and_relative_transform_reject_invalid_inputs():
    with pytest.raises(ValueError):
        transform_pose(pose(), Transform(rotation=Quaternion(w=0.0)))
    with pytest.raises(ValueError):
        transform_pose(pose(), None)
    invalid_pose = pose()
    invalid_pose.orientation = Quaternion(w=0.0)
    with pytest.raises(ValueError):
        relative_transform(invalid_pose, pose())
    invalid = Transform(
        translation=Vector3(x=float('nan')),
        rotation=Quaternion(w=1.0),
    )
    with pytest.raises(ValueError):
        transform_pose(pose(), invalid)


@pytest.mark.parametrize('angle', [float('nan'), float('inf'), None])
def test_angle_functions_reject_nonfinite_angles(angle):
    with pytest.raises(ValueError):
        quaternion_from_yaw(angle)
    with pytest.raises(ValueError):
        angular_distance(angle, 0.0)
    with pytest.raises(ValueError):
        angular_distance(0.0, angle)
