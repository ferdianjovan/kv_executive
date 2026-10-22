"""
Validated geometry for mission paths and fixed frame conversions.

Transforms follow the ROS convention: ``transform_pose(pose, transform)``
applies the transform to the pose, including its orientation.  Path distances
are measured in three dimensions, and splitting retains every original vertex.
"""

from copy import deepcopy
import math

from geometry_msgs.msg import Point, Pose, Quaternion, Transform, Vector3
from nav_msgs.msg import Path


def _finite(value, description):
    try:
        value = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f'{description} must be finite') from exc
    if not math.isfinite(value):
        raise ValueError(f'{description} must be finite')
    return value


def _vector_values(vector):
    try:
        return tuple(_finite(getattr(vector, axis), axis) for axis in 'xyz')
    except AttributeError as exc:
        raise ValueError('Expected a vector with finite x, y and z') from exc


def _quaternion_values(quaternion):
    try:
        values = tuple(
            _finite(getattr(quaternion, axis), f'quaternion {axis}')
            for axis in 'xyzw'
        )
    except AttributeError as exc:
        raise ValueError('Expected a quaternion with finite x, y, z and w') from exc
    # Scaling first avoids overflow and accepts finite, nonzero subnormal inputs.
    scale = max(abs(value) for value in values)
    if scale == 0.0:
        raise ValueError('Quaternion must have a nonzero norm')
    scaled = tuple(value / scale for value in values)
    norm = math.hypot(*scaled)
    return tuple(value / norm for value in scaled)


def _pose_values(pose):
    try:
        return _vector_values(pose.position), _quaternion_values(pose.orientation)
    except AttributeError as exc:
        raise ValueError('Expected a pose with a position and orientation') from exc


def _make_quaternion(values):
    return Quaternion(x=values[0], y=values[1], z=values[2], w=values[3])


def _make_pose(position, orientation):
    position = tuple(_finite(value, 'position') for value in position)
    return Pose(
        position=Point(x=position[0], y=position[1], z=position[2]),
        orientation=_make_quaternion(orientation),
    )


def _multiply(left, right):
    x, y, z, w = left
    a, b, c, d = right
    result = Quaternion(
        x=w * a + x * d + y * c - z * b,
        y=w * b - x * c + y * d + z * a,
        z=w * c + x * b - y * a + z * d,
        w=w * d - x * a - y * b - z * c,
    )
    return _quaternion_values(result)


def _rotate(rotation, position):
    x, y, z, w = rotation
    a, b, c = position
    return (
        (1.0 - 2.0 * (y * y + z * z)) * a
        + 2.0 * (x * y - w * z) * b + 2.0 * (x * z + w * y) * c,
        2.0 * (x * y + w * z) * a
        + (1.0 - 2.0 * (x * x + z * z)) * b + 2.0 * (y * z - w * x) * c,
        2.0 * (x * z - w * y) * a + 2.0 * (y * z + w * x) * b
        + (1.0 - 2.0 * (x * x + y * y)) * c,
    )


def yaw_from_quaternion(quaternion: Quaternion) -> float:
    """Return yaw in radians after normalizing a finite, nonzero quaternion."""
    x, y, z, w = _quaternion_values(quaternion)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quaternion_from_yaw(yaw: float) -> Quaternion:
    """Construct a unit quaternion for a finite rotation about the Z axis."""
    half_yaw = 0.5 * _finite(yaw, 'yaw')
    return Quaternion(z=math.sin(half_yaw), w=math.cos(half_yaw))


def angular_distance(target: float, current: float) -> float:
    """Return the signed shortest rotation from current to target in radians."""
    # Reduce before subtracting so two finite large angles cannot overflow.
    target = math.remainder(_finite(target, 'target angle'), math.tau)
    current = math.remainder(_finite(current, 'current angle'), math.tau)
    return math.atan2(math.sin(target - current), math.cos(target - current))


def interpolate_pose(start: Pose, end: Pose, fraction: float) -> Pose:
    """Linearly interpolate position and use shortest-path quaternion SLERP."""
    fraction = _finite(fraction, 'interpolation fraction')
    if not 0.0 <= fraction <= 1.0:
        raise ValueError('Interpolation fraction must lie between zero and one')
    start_position, start_q = _pose_values(start)
    end_position, end_q = _pose_values(end)
    position = tuple(
        (1.0 - fraction) * a + fraction * b
        for a, b in zip(start_position, end_position)
    )
    dot = sum(a * b for a, b in zip(start_q, end_q))
    if dot < 0.0:
        end_q = tuple(-value for value in end_q)
        dot = -dot
    dot = min(1.0, max(-1.0, dot))
    if dot > 0.9995:
        orientation = tuple(
            (1.0 - fraction) * a + fraction * b for a, b in zip(start_q, end_q)
        )
    else:
        angle = math.acos(dot)
        denominator = math.sin(angle)
        start_weight = math.sin((1.0 - fraction) * angle) / denominator
        end_weight = math.sin(fraction * angle) / denominator
        orientation = tuple(
            start_weight * a + end_weight * b for a, b in zip(start_q, end_q)
        )
    return _make_pose(position, _quaternion_values(_make_quaternion(orientation)))


def _interpolate_stamped(start, end, fraction, path_header):
    if fraction <= 0.0:
        return deepcopy(start)
    if fraction >= 1.0:
        return deepcopy(end)
    result = deepcopy(start)
    result.pose = interpolate_pose(start.pose, end.pose, fraction)
    start_ns = start.header.stamp.sec * 1_000_000_000 + start.header.stamp.nanosec
    end_ns = end.header.stamp.sec * 1_000_000_000 + end.header.stamp.nanosec
    stamp_ns = start_ns + round((end_ns - start_ns) * fraction)
    result.header.stamp.sec, result.header.stamp.nanosec = divmod(
        stamp_ns, 1_000_000_000
    )
    result.header.frame_id = (
        start.header.frame_id or end.header.frame_id or path_header.frame_id
    )
    return result


def split_path(path: Path, segments: int = 3) -> list[Path]:
    """
    Split a path into equal arc lengths while retaining its original turns.

    Shared boundaries are duplicated between adjacent output paths. Original
    pose headers are copied; inserted boundaries interpolate timestamps and
    orientations. Nonempty frame IDs must agree because arc length assumes a
    common coordinate frame. Repeated vertices are retained, including their
    orientations, but a path with no translational length is rejected.
    """
    if not isinstance(segments, int) or isinstance(segments, bool) or segments <= 0:
        raise ValueError('Segment count must be a positive integer')
    if not isinstance(path, Path) or not path.poses:
        raise ValueError('Path must contain at least one pose')

    positions = []
    frames = {path.header.frame_id} if path.header.frame_id else set()
    for stamped in path.poses:
        try:
            position, _ = _pose_values(stamped.pose)
            frame = stamped.header.frame_id
        except AttributeError as exc:
            raise ValueError('Path must contain stamped poses') from exc
        positions.append(position)
        if frame:
            frames.add(frame)
    if len(frames) > 1:
        raise ValueError('Path poses must use a common coordinate frame')

    cumulative = [0.0]
    for start, end in zip(positions, positions[1:]):
        distance = math.hypot(*(b - a for a, b in zip(start, end)))
        cumulative.append(_finite(cumulative[-1] + distance, 'path length'))
    total = cumulative[-1]
    if total == 0.0:
        raise ValueError('Path must have nonzero translational length')
    cuts = [total * (index / segments) for index in range(1, segments)]
    if any(a >= b for a, b in zip([0.0] + cuts, cuts + [total])):
        raise ValueError('Path is too short to represent distinct split boundaries')

    result = []
    current = Path(header=deepcopy(path.header), poses=[deepcopy(path.poses[0])])
    cut_index = 0
    for index, (start, end) in enumerate(zip(path.poses, path.poses[1:])):
        start_distance, end_distance = cumulative[index:index + 2]
        endpoint_is_boundary = False
        # A zero-length edge still contributes its original endpoint below.
        while (
            end_distance > start_distance
            and cut_index < len(cuts)
            and cuts[cut_index] <= end_distance
        ):
            boundary_distance = cuts[cut_index]
            fraction = (boundary_distance - start_distance) / (
                end_distance - start_distance
            )
            boundary = _interpolate_stamped(start, end, fraction, path.header)
            current.poses.append(boundary)
            result.append(current)
            current = Path(
                header=deepcopy(path.header), poses=[deepcopy(boundary)]
            )
            endpoint_is_boundary = boundary_distance == end_distance
            cut_index += 1
        if not endpoint_is_boundary:
            current.poses.append(deepcopy(end))
    result.append(current)
    return result


def transform_pose(pose: Pose, transform: Transform) -> Pose:
    """Apply ``transform * pose``, rotating both position and orientation."""
    position, orientation = _pose_values(pose)
    try:
        translation = _vector_values(transform.translation)
        rotation = _quaternion_values(transform.rotation)
    except AttributeError as exc:
        raise ValueError('Expected a transform with translation and rotation') from exc
    rotated = _rotate(rotation, position)
    return _make_pose(
        tuple(a + b for a, b in zip(rotated, translation)),
        _multiply(rotation, orientation),
    )


def relative_transform(local_body: Pose, source_body: Pose) -> Transform:
    """
    Map source coordinates into local using coincident body-frame poses.

    The returned transform is ``T_local_body * inverse(T_source_body)``.
    The two inputs must describe the same body at the same instant.
    """
    local_position, local_orientation = _pose_values(local_body)
    source_position, source_orientation = _pose_values(source_body)
    inverse_source = tuple(-value for value in source_orientation[:3]) + (
        source_orientation[3],
    )
    rotation = _multiply(local_orientation, inverse_source)
    rotated_source = _rotate(rotation, source_position)
    translation = tuple(
        _finite(a - b, 'transform translation')
        for a, b in zip(local_position, rotated_source)
    )
    return Transform(
        translation=Vector3(x=translation[0], y=translation[1], z=translation[2]),
        rotation=_make_quaternion(rotation),
    )
